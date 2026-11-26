#!/usr/bin/env python3
"""
A LinuxCNC session without a screen, for testing the Tiggy driver.

run_in_linuxcnc.sh puts this program in the INI file as [DISPLAY]DISPLAY, so
LinuxCNC starts it in place of its user interface.  It then does what an
operator would - leave E-stop, switch on, home, jog, type MDI commands, run
a program, pause, abort, probe, hit E-stop - and after each step compares
where LinuxCNC believes the machine is with where the simulated controller
(tools/firmware_sim.py) really is, to the step.

TIGGY_TEST_PLAN   basic (default) or switches (a machine with home switches)
TIGGY_TEST_RESULT file the results are written to
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
JOINTS = 3


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
        self.scale = [hal.get_value("tiggy.%d.position-scale" % j) for j in range(JOINTS)]
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
        self.base = [pos[j] - int(self.pin("tiggy.%d.counts" % j)) for j in range(JOINTS)]

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
            have = pos[j] - self.base[j]
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
        check(self.c.wait_complete(timeout) != -1, "MDI '%s' did not finish" % text)
        self.settle()

    def machine_on(self):
        self.c.state(linuxcnc.STATE_ESTOP_RESET)
        self.c.wait_complete()
        self.c.state(linuxcnc.STATE_ON)
        self.c.wait_complete()
        self.wait_for(lambda: self.poll().task_state == linuxcnc.STATE_ON, 5, "machine on")
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
        check(self.poll().task_state == linuxcnc.STATE_ESTOP, "LinuxCNC should start in E-stop")
        check(self.sim.get()["state"] == "ESTOP", "the controller should be held stopped while the machine is off")
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
        self.c.auto(linuxcnc.AUTO_PAUSE)
        self.wait_for(lambda: not self.sim.get()["moving"], 3, "the machine to pause")
        time.sleep(0.3)
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
        self.wait_for(lambda: self.poll().task_state == linuxcnc.STATE_ESTOP, 2, "LinuxCNC to go into E-stop")
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
    def run(self):
        plans = {
            "basic": [self.step_ready, self.step_on_and_home, self.step_mdi, self.step_program,
                      self.step_jog_and_abort, self.step_probe, self.step_estop],
            "switches": [self.step_ready, self.step_home_on_switches, self.step_mdi],
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
