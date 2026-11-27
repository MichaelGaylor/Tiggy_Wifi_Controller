#!/usr/bin/env python3
"""
A LinuxCNC session without a screen, for testing the Tiggy driver.

run_in_linuxcnc.sh puts this program in the INI file as [DISPLAY]DISPLAY, so
LinuxCNC starts it in place of its user interface.  It then does what an
operator would - leave E-stop, switch on, home, jog, type MDI commands, run
a program, pause, abort, probe, hit E-stop - and after each step compares
where LinuxCNC believes the machine is with where the simulated controller
(tools/firmware_sim.py) really is, to the step.

TIGGY_TEST_PLAN     basic (default), switches (a machine with home switches),
                    threading (G33 on a mill), lathe (X and Z, G33 and G76) or
                    lathe-slow (the same, for a second run at another speed)
TIGGY_TEST_OUTPUTS  the controller's motor output of each joint: 0,1,2 unless
                    given (a lathe: 0,2)
TIGGY_TEST_RPM      spindle speed for the screw cutting (300)
TIGGY_TEST_SAG      how much of its speed the spindle loses during the second
                    G33 pass, as if under the cut (0.15; 0 for none)
TIGGY_TEST_RESULT   file the results are written to
"""

import json
import math
import os
import socket
import sys
import time

import hal
import linuxcnc

PLAN = os.environ.get("TIGGY_TEST_PLAN", "basic")
RESULT = os.environ.get("TIGGY_TEST_RESULT", "")
OUTPUTS = [int(v) for v in os.environ.get("TIGGY_TEST_OUTPUTS", "0,1,2").split(",")]
JOINTS = len(OUTPUTS)
Z = JOINTS - 1                          # the joint that is Z: the last, on a mill and on a lathe
RPM = float(os.environ.get("TIGGY_TEST_RPM", "300"))
SAG = float(os.environ.get("TIGGY_TEST_SAG", "0.15"))


class Failure(Exception):
    pass


def check(cond, text):
    if not cond:
        raise Failure(text)


def llround(v):
    return int(math.floor(abs(v) + 0.5)) * (1 if v >= 0 else -1)


class Sim:
    def __init__(self, port=58500):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.file = self.sock.makefile("rwb")

    def cmd(self, **req):
        self.file.write((json.dumps(req) + "\n").encode())
        self.file.flush()
        return json.loads(self.file.readline().decode())

    def get(self):
        return self.cmd(cmd="get")

    def set(self, **values):
        return self.cmd(cmd="set", **values)


class Session:
    def __init__(self):
        self.c = linuxcnc.command()
        self.s = linuxcnc.stat()
        self.e = linuxcnc.error_channel()
        self.h = hal.component("tiggy-test")
        self.h.ready()
        self.sim = Sim()
        self.scale = [hal.get_value("tiggy.%d.position-scale" % o) for o in OUTPUTS]
        self.base = [0] * JOINTS
        self.lines = []
        self.failed = 0
        self.underruns = 0

    # ---- helpers
    def pin(self, name):
        return hal.get_value(name)

    def poll(self):
        self.s.poll()
        return self.s

    def state_name(self):
        names = {linuxcnc.STATE_ESTOP: "E-stop", linuxcnc.STATE_ESTOP_RESET: "E-stop reset",
                 linuxcnc.STATE_ON: "on", linuxcnc.STATE_OFF: "off"}
        state = self.poll().task_state
        return "%s (estop-latch ok-out %s, tiggy.fault %s, tiggy.ready %s)" % (
            names.get(state, state), self.pin("estop-latch.0.ok-out"), self.pin("tiggy.fault"), self.pin("tiggy.ready"))

    def wait_state(self, state, timeout, what):
        deadline = time.time() + timeout
        while self.poll().task_state != state:
            if time.time() > deadline:
                raise Failure("%s: LinuxCNC is in state %s" % (what, self.state_name()))
            time.sleep(0.02)

    def wait_for(self, cond, timeout, what):
        deadline = time.time() + timeout
        while not cond():
            if time.time() > deadline:
                raise Failure("timed out waiting for " + what)
            time.sleep(0.01)

    def errors(self):
        found = []
        while True:
            err = self.e.poll()
            if not err:
                return found
            found.append(err[1])

    def rebase(self):
        time.sleep(0.1)
        pos = self.sim.get()["pos"]
        self.base = [pos[o] - int(self.pin("tiggy.%d.counts" % o)) for o in OUTPUTS]

    def settle(self, timeout=60):
        """LinuxCNC has stopped commanding motion and the controller has played all of it."""
        def still():
            a = [self.pin("joint.%d.motor-pos-cmd" % j) for j in range(JOINTS)]
            time.sleep(0.03)
            return a == [self.pin("joint.%d.motor-pos-cmd" % j) for j in range(JOINTS)]
        self.wait_for(still, timeout, "LinuxCNC to stop moving")
        self.wait_for(lambda: not self.sim.get()["busy"], 5, "the controller to finish")
        time.sleep(0.08)

    def check_position(self, where):
        pos = self.sim.get()["pos"]
        for j in range(JOINTS):
            cmd = self.pin("joint.%d.motor-pos-cmd" % j)
            want = (llround(cmd * self.scale[j] * 256.0) + 128) // 256
            have = pos[OUTPUTS[j]] - self.base[j]
            check(have == want, "%s: joint %d is at step %d, LinuxCNC commands step %d" % (where, j, have, want))
            fb = self.pin("joint.%d.motor-pos-fb" % j)
            check(abs(fb - want / self.scale[j]) < 1e-6, "%s: joint %d feedback is %r" % (where, j, fb))

    def healthy(self, where):
        errs = self.errors()
        check(not errs, "%s: LinuxCNC reported: %s" % (where, "; ".join(errs)))
        check(self.pin("tiggy.position-corrections") == 0, "%s: the driver corrected its position" % where)
        # A computer that stalls for longer than the queue is not a fault (the
        # position checks still hold), but it is worth knowing about.
        short = int(self.pin("tiggy.underruns"))
        if short != self.underruns:
            self.lines.append("note  %s: the controller ran short of motion %d time(s): this computer "
                              "stalled for longer than the queue" % (where, short - self.underruns))
            self.underruns = short

    def mode(self, m):
        self.c.mode(m)
        self.c.wait_complete()

    def mdi(self, text, timeout=60):
        self.mode(linuxcnc.MODE_MDI)
        self.c.mdi(text)
        result = self.c.wait_complete(timeout)
        check(result != -1, "MDI '%s' did not finish" % text)
        check(result == linuxcnc.RCS_DONE, "MDI '%s' was refused: %s" % (text, "; ".join(self.errors())))
        self.settle()

    def machine_on(self):
        self.c.state(linuxcnc.STATE_ESTOP_RESET)
        self.c.wait_complete()
        self.wait_state(linuxcnc.STATE_ESTOP_RESET, 5, "E-stop was reset")
        self.c.state(linuxcnc.STATE_ON)
        self.c.wait_complete()
        self.wait_state(linuxcnc.STATE_ON, 5, "the machine was switched on")
        self.wait_for(lambda: self.sim.get()["state"] == "IDLE", 5, "the controller to be ready")

    def home_all(self, timeout=120):
        self.mode(linuxcnc.MODE_MANUAL)
        self.c.teleop_enable(0)
        self.c.wait_complete()
        self.c.home(-1)
        self.wait_for(lambda: all(self.poll().homed[:JOINTS]), timeout, "homing")
        self.settle()

    # ---- steps
    def step_ready(self):
        self.wait_for(lambda: self.pin("tiggy.ready"), 15, "tiggy.ready")
        time.sleep(1.0)                     # LinuxCNC's own parts are still starting
        self.wait_state(linuxcnc.STATE_ESTOP, 5, "LinuxCNC should start in E-stop")
        self.wait_for(lambda: self.sim.get()["state"] == "ESTOP", 3,
                      "the controller to be held stopped while the machine is off")
        self.rebase()
        self.errors()

    def step_on_and_home(self):
        self.machine_on()
        self.home_all()
        self.check_position("after homing")
        self.healthy("after homing")

    def step_mdi(self):
        home = [round(v, 4) for v in self.poll().position[:3]]
        for text in ("G0 X12.345 Y23.21 Z-7.5", "G1 F900 X3.001 Y3.002", "G0 X5 Y5", "G2 X15 Y5 I5 J0 F1200",
                     "G3 X5 Y5 I-5 J0", "G1 X5.0125 F60", "G0 X%g Y%g Z%g" % tuple(home)):
            self.mdi(text)
            self.check_position(text)
        self.healthy("MDI")

    def step_program(self):
        path = os.path.join(os.path.dirname(RESULT) or ".", "tiggy_test.ngc")
        with open(path, "w") as f:
            f.write("G21 G90 G64 P0.02 F1500\nG0 X40 Y40 Z0\nG1 Z-1\n")
            for i in range(1, 241):                     # a spiral of short segments
                a = i * math.pi / 30.0
                r = 2.0 + i * 0.05
                f.write("G1 X%.4f Y%.4f Z%.4f\n" % (40.0 + r * math.cos(a), 40.0 + r * math.sin(a), -1.0 - i * 0.01))
            f.write("G0 Z0\nG0 X5 Y5\nM2\n")
        self.mode(linuxcnc.MODE_AUTO)
        self.c.program_open(path)
        self.c.auto(linuxcnc.AUTO_RUN, 0)
        time.sleep(1.5)
        # feed hold in the middle of it, then carry on
        check(self.poll().interp_state != linuxcnc.INTERP_IDLE, "the program is not running: %s"
              % "; ".join(self.errors()))
        self.c.auto(linuxcnc.AUTO_PAUSE)
        self.settle(10)
        self.check_position("paused")
        self.c.auto(linuxcnc.AUTO_RESUME)
        self.wait_for(lambda: self.poll().interp_state == linuxcnc.INTERP_IDLE, 120, "the program to end")
        self.settle()
        self.check_position("after the program")
        self.healthy("program")

    def step_jog_and_abort(self):
        self.mode(linuxcnc.MODE_MANUAL)
        self.c.teleop_enable(1)
        self.c.wait_complete()
        self.c.jog(linuxcnc.JOG_CONTINUOUS, False, 0, 20.0)
        time.sleep(0.8)
        check(self.sim.get()["moving"], "the machine should be jogging")
        self.c.jog(linuxcnc.JOG_STOP, False, 0)
        self.settle()
        self.check_position("after a jog")
        self.c.jog(linuxcnc.JOG_INCREMENT, False, 1, 20.0, 1.2345)
        self.settle()
        self.check_position("after a step jog")
        self.mode(linuxcnc.MODE_MDI)
        self.c.mdi("G1 F600 X40")
        time.sleep(0.7)
        self.c.abort()
        self.settle()
        self.check_position("after an abort")
        self.mdi("G0 X5 Y5")
        self.check_position("back at the start")
        self.healthy("jog and abort")

    def step_probe(self):
        phys = self.sim.get()["phys"]
        touch = phys[2] - 5.0 * abs(self.scale[2])      # 5 mm below
        self.sim.set(probe_z=touch)
        z0 = self.poll().position[2]
        self.mode(linuxcnc.MODE_MDI)
        self.c.mdi("G38.2 Z%.3f F120" % (z0 - 20.0))
        check(self.c.wait_complete(30) != -1, "the probe move did not finish")
        self.settle()
        s = self.poll()
        check(s.probe_tripped, "LinuxCNC did not see the probe")
        found = s.probed_position[2]
        check(abs(found - (z0 - 5.0)) <= 2.0 * 0.03, "probe at %.3f was found at %.3f" % (z0 - 5.0, found))
        check(z0 - 5.0 - s.position[2] <= 2.0 * 0.4, "ran %.2f mm on after contact" % (z0 - 5.0 - s.position[2]))
        self.check_position("after probing")
        self.sim.set(probe_z=None)
        self.mdi("G0 Z%.3f" % z0)
        self.healthy("probe")

    def step_estop(self):
        self.mode(linuxcnc.MODE_MDI)
        self.c.mdi("G1 F1500 X60")
        time.sleep(1.0)
        self.c.state(linuxcnc.STATE_ESTOP)
        self.wait_for(lambda: self.sim.get()["state"] == "ESTOP" and not self.sim.get()["moving"], 1,
                      "the controller to stop on E-stop")
        time.sleep(0.3)
        self.check_position("after E-stop")
        self.errors()
        self.machine_on()
        check(all(self.poll().homed[:JOINTS]), "homing was lost over the E-stop")
        self.mdi("G0 X5")
        self.check_position("after recovering from E-stop")

        # the button on the controller
        self.sim.set(estop=1)
        self.wait_state(linuxcnc.STATE_ESTOP, 3, "the controller's E-stop input was operated")
        check(any("E-stop" in e for e in self.errors()), "no message about the E-stop input")
        self.sim.set(estop=0)
        self.wait_for(lambda: self.pin("tiggy.ready"), 2, "ready after the button is released")
        self.machine_on()
        self.mdi("G0 X8 Y8")
        self.check_position("after the E-stop button")
        self.mdi("G0 X5 Y5")

    def step_home_on_switches(self):
        """Home onto the simulated switches: X and Y at the low end, Z at the top."""
        phys = self.sim.get()["phys"]
        lo = [phys[0] - 4000.0, phys[1] - 4000.0, -2.0e6, -2.0e6, -2.0e6, -2.0e6]
        hi = [2.0e6, 2.0e6, phys[2] + 4000.0, 2.0e6, 2.0e6, 2.0e6]
        self.sim.set(lim_lo=lo, lim_hi=hi)
        self.machine_on()
        self.home_all()
        self.check_position("after homing")
        pos = self.sim.get()["phys"]
        # Each joint is now 2 mm inside its switch (HOME), where the switch is HOME_OFFSET
        for j, switch, sign in ((0, lo[0], 1), (1, lo[1], 1), (2, hi[2], -1)):
            off = (pos[j] - switch) * sign / abs(self.scale[j])
            check(abs(off - 2.0) <= 0.03, "joint %d stands %.3f mm from its switch after homing, not 2" % (j, off))
        check(self.sim.get()["state"] == "IDLE", "homing onto a limit switch must not stop the controller")
        self.healthy("homing on switches")
        # ...and again: it repeats
        first = list(pos)
        self.mdi("G53 G0 X20 Y20 Z-10")
        self.c.mode(linuxcnc.MODE_MANUAL)
        self.c.wait_complete()
        self.c.teleop_enable(0)             # joints can only be un-homed in joint mode
        self.c.wait_complete()
        for j in range(JOINTS):
            self.c.unhome(j)
        self.c.wait_complete()
        self.home_all()
        pos = self.sim.get()["phys"]
        for j in range(JOINTS):
            check(abs(pos[j] - first[j]) <= 0.03 * abs(self.scale[j]),
                  "joint %d homed %d steps away from the first time" % (j, pos[j] - first[j]))
        self.healthy("homing again")

    # ---- runner
    def step_threading(self):
        # Three G33 passes of 8 turns, pitch 2 mm.  In the second the spindle slows by 15 % as if
        # under the cut.  The simulator records every step of Z with the spindle's angle at it.
        sim = self.sim
        sim.cmd(cmd="spindle", follow=0, rpm=RPM, accel=2000.0, ripple=0.0, ripple_hz=0.0)
        self.mdi("M3 S%g" % RPM)
        self.wait_for(lambda: abs(self.pin("tiggy.spindle-speed-fb-rpm") - RPM) < 0.01 * RPM, 20, "the spindle to reach %g rpm" % RPM)
        time.sleep(1.5)
        pitch, depth = 2.0, 16.0
        per_rev = pitch * self.scale[Z] * (-1.0)        # Z goes negative: steps per revolution, signed
        z0 = round(self.poll().position[2], 4)
        sim.cmd(cmd="trace", axis=2)
        for n, slow in enumerate((None, RPM * (1.0 - SAG) if SAG else None, None)):
            if slow:
                sim.cmd(cmd="spindle", follow=0, rpm=slow, accel=RPM / 5.0)     # sags through the pass
            self.mdi("G33 Z%g K%g" % (z0 - depth, pitch), timeout=180)
            sim.cmd(cmd="spindle", follow=0, rpm=RPM, accel=2000.0)
            self.mdi("G0 Z%g" % z0)
            self.check_position("after pass %d" % (n + 1))
            self.wait_for(lambda: abs(self.pin("tiggy.spindle-speed-fb-rpm") - RPM) < 0.01 * RPM, 20, "the spindle to recover")
            time.sleep(1.2)
        self.mdi("M5")
        self.check_passes(per_rev, abs(per_rev) * depth / pitch, 3)
        self.healthy("screw cutting")

    def check_passes(self, per_rev, length, expected, each=True):
        """The record of Z's steps, each with the spindle's angle at it: every pass must keep its
        place on the spindle, and all of them the same place.  per_rev: Z steps per revolution,
        signed; length: steps of a pass."""
        tr = self.sim.cmd(cmd="trace_read")
        turns_at, pos = tr["turns"], tr["pos"]
        cut = 1 if per_rev > 0 else -1
        runs, first = [], None
        for i in range(1, len(pos)):
            cutting = (pos[i] - pos[i - 1]) * cut > 0
            if cutting and first is None:
                first = i
            elif not cutting and first is not None:
                if i - first > 0.5 * length:
                    runs.append((first, i))
                first = None
        if first is not None and len(pos) - first > 0.5 * length:
            runs.append((first, len(pos)))
        check(len(runs) == expected, "expected %d passes in the record of Z's steps, found %d" % (expected, len(runs)))
        places, worst = [], 0.0
        for n, (a, b) in enumerate(runs):
            lo, hi = a + (b - a) // 4, a + (b - a) * 9 // 10
            res = [pos[i] - per_rev * turns_at[i] for i in range(lo, hi)]
            spread = max(res) - min(res)
            worst = max(worst, spread)
            if each:
                self.lines.append("      pass %d: %d steps, Z within %.2f steps of its place on the spindle" % (n + 1, b - a, spread))
            check(spread <= 3.0, "pass %d: Z wanders %.1f steps against the spindle's angle" % (n + 1, spread))
            places.append((sum(res) / len(res)) % abs(per_rev))
        apart = 0.0
        for n in range(1, len(runs)):
            d = (places[n] - places[0] + abs(per_rev) / 2.0) % abs(per_rev) - abs(per_rev) / 2.0
            apart = max(apart, abs(d))
            if each:
                self.lines.append("      pass %d begins %.2f steps from pass 1" % (n + 1, d))
            check(abs(d) <= 0.02 * abs(per_rev), "pass %d begins %.1f steps (%.1f degrees) from pass 1: the threads "
                  "do not line up" % (n + 1, d, d / abs(per_rev) * 360.0))
        if not each:
            self.lines.append("      %d passes: each within %.2f steps of its place on the spindle, all within %.2f steps "
                              "of the first" % (len(runs), worst, apart))

    def step_lathe_moves(self):
        # X is given as a diameter (G7): the cross slide travels half of it
        self.mdi("G0 X40 Z-5")
        self.check_position("G0 X40 Z-5")
        check(abs(self.pin("joint.0.motor-pos-cmd") - 20.0) < 1e-6,
              "X40 as a diameter put the cross slide at %r" % self.pin("joint.0.motor-pos-cmd"))
        for text in ("G1 F300 X20.5 Z-22.25", "G0 X60", "G0 Z2.5", "G2 X50 Z-2.5 I0 K-5 F400", "G0 X0 Z0"):
            self.mdi(text)
            self.check_position(text)
        self.healthy("lathe moves")

    def step_g76(self):
        # A threading cycle: G76 makes pass after pass by itself, each deeper than the last
        # (0.4, 0.8 and 1.2 mm, and one spring pass at that depth).
        sim = self.sim
        sim.cmd(cmd="spindle", follow=0, rpm=RPM, accel=2000.0, ripple=0.0, ripple_hz=0.0)
        self.mdi("M3 S%g" % RPM)
        self.wait_for(lambda: abs(self.pin("tiggy.spindle-speed-fb-rpm") - RPM) < 0.01 * RPM, 20, "the spindle to reach %g rpm" % RPM)
        time.sleep(1.5)
        pitch, start, end = 2.0, 2.0, -16.0
        per_rev = pitch * self.scale[Z] * (-1.0)
        self.mdi("G0 X24 Z%g" % start)
        sim.cmd(cmd="trace", axis=2)
        self.mdi("G76 P%g Z%g I-1 J0.4 K1.2 H1" % (pitch, end), timeout=600)
        self.check_position("after G76")
        self.mdi("M5")
        self.check_passes(per_rev, abs(per_rev) * (start - end) / pitch, 4, each=False)
        self.mdi("G0 X0 Z0")
        self.check_position("back at the start")
        self.healthy("G76")

    def run(self):
        plans = {
            "basic": [self.step_ready, self.step_on_and_home, self.step_mdi, self.step_program,
                      self.step_jog_and_abort, self.step_probe, self.step_estop],
            "switches": [self.step_ready, self.step_home_on_switches, self.step_mdi],
            "threading": [self.step_ready, self.step_on_and_home, self.step_threading],
            "lathe": [self.step_ready, self.step_on_and_home, self.step_lathe_moves, self.step_threading, self.step_g76],
            "lathe-slow": [self.step_ready, self.step_on_and_home, self.step_threading, self.step_g76],
        }
        for fn in plans[PLAN]:
            name = fn.__name__[5:]
            t0 = time.time()
            try:
                fn()
                self.lines.append("PASS  %-20s (%.1f s)" % (name, time.time() - t0))
            except Failure as e:
                self.failed += 1
                self.lines.append("FAIL  %-20s %s" % (name, e))
                self.lines += ["        | " + err for err in self.errors()]
                break                                   # later steps build on this one
            except Exception as e:                      # a mistake in the test itself must show, too
                self.failed += 1
                self.lines.append("FAIL  %-20s %r" % (name, e))
                break
        self.lines.append("ALL PASSED (%s)" % PLAN if not self.failed else "FAILED (%s)" % PLAN)
        text = "\n".join(self.lines) + "\n"
        print(text, flush=True)
        if RESULT:
            with open(RESULT, "a") as f:
                f.write(text)
        try:
            self.c.state(linuxcnc.STATE_ESTOP)
            self.c.wait_complete()
        except Exception:
            pass
        return 1 if self.failed else 0


if __name__ == "__main__":
    sys.exit(Session().run())
