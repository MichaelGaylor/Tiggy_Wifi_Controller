#!/usr/bin/env python3
"""
A LinuxCNC session without a screen, against a real Tiggy controller.

run_on_board.sh puts this program in the INI file as [DISPLAY]DISPLAY, so
LinuxCNC starts it in place of its user interface.  It does what an operator
would - leave E-stop, switch on, home, type MDI commands, run a program,
pause, jog, abort, hit E-stop - and after each step compares where LinuxCNC
commands the machine to be with the controller's own count of the steps it
has made (tiggy.N.counts, which the driver takes from the controller's
status reports), to the step.  It ends with some minutes of fast moves on
three axes, and says how the motion queued in the controller held up over
the network meanwhile.

(linuxcnc_session.py does the same against the simulated controller, where
there is a second opinion on every step and inputs can be operated; here the
controller's own word is all there is, and nothing is connected to it.)

Motors, if there are any, move some 60 mm on X and Y and 10 mm down on Z
from where they stand, at up to 50 mm/s.

With TIGGY_TEST_PLAN=threading it cuts threads instead: G33 passes and a G76
cycle on Z, against a spindle that run_on_board.sh has arranged - the
controller's own spindle PWM output linked to its single spindle sensor, so
that LinuxCNC's M3 starts a 'spindle' of 3180 rpm that the axes have no part
in.  Every status report made during a pass gives Z's count and the
controller's reading of the spindle from the same moment (tiggy.2.counts,
tiggy.spindle-turns, tiggy.spindle-angle): Z must keep its place on the
spindle through each pass - also while the spindle's speed is changed in the
middle of one - and every pass must have the same place.  (Each pass begins
at the same spindle speed.  With LinuxCNC, on any machine, where a pass lies
on the work depends on the speed it began at: the axis takes a distance to
reach the spindle's speed that grows with it.)

TIGGY_TEST_PLAN     moves (default) or threading
TIGGY_TEST_RESULT   file the results are written to
TIGGY_TEST_MINUTES  length of the long run at the end (default 2)
TIGGY_TEST_SPEED_BY S (only against the simulator, whose spindle goes by the
                    speed asked for, not by the frequency of a PWM signal;
                    the change of speed in mid-pass is then left out)
"""

import math
import os
import sys
import time

import hal
import linuxcnc

PLAN = os.environ.get("TIGGY_TEST_PLAN", "moves")
RESULT = os.environ.get("TIGGY_TEST_RESULT", "")
MINUTES = float(os.environ.get("TIGGY_TEST_MINUTES", "2"))
SPEED_BY_S = os.environ.get("TIGGY_TEST_SPEED_BY", "") == "S"
JOINTS = 3
Z = 2
HZ, HZ_OTHER = 53, 59                   # the spindle, turns a second: as planned for, and changed to


class Failure(Exception):
    pass


def check(cond, text):
    if not cond:
        raise Failure(text)


def llround(v):
    return int(math.floor(abs(v) + 0.5)) * (1 if v >= 0 else -1)


class Session:
    def __init__(self):
        self.c = linuxcnc.command()
        self.s = linuxcnc.stat()
        self.e = linuxcnc.error_channel()
        self.h = hal.component("tiggy-test")
        self.h.ready()
        self.scale = [hal.get_value("tiggy.%d.position-scale" % j) for j in range(JOINTS)]
        self.lines = []
        self.failed = 0
        self.underruns = 0
        self.resent = 0

    # ---- helpers
    def pin(self, name):
        return hal.get_value(name)

    def counts(self):
        return [int(self.pin("tiggy.%d.counts" % j)) for j in range(JOINTS)]

    def poll(self):
        self.s.poll()
        return self.s

    def state_name(self):
        names = {linuxcnc.STATE_ESTOP: "E-stop", linuxcnc.STATE_ESTOP_RESET: "E-stop reset",
                 linuxcnc.STATE_ON: "on", linuxcnc.STATE_OFF: "off"}
        state = self.poll().task_state
        return "%s (tiggy.connected %s, tiggy.fault %s, tiggy.ready %s)" % (
            names.get(state, state), self.pin("tiggy.connected"), self.pin("tiggy.fault"), self.pin("tiggy.ready"))

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

    def settle(self, timeout=60):
        """LinuxCNC has stopped commanding motion and the controller has played all of it."""
        def still():
            a = [self.pin("joint.%d.motor-pos-cmd" % j) for j in range(JOINTS)]
            time.sleep(0.03)
            return a == [self.pin("joint.%d.motor-pos-cmd" % j) for j in range(JOINTS)]
        self.wait_for(still, timeout, "LinuxCNC to stop moving")

        def played():
            a = self.counts()
            time.sleep(0.15)
            return a == self.counts() and self.pin("tiggy.buffer-ms") < 1.0
        self.wait_for(played, 10, "the controller to finish (%.0f ms still queued)" % self.pin("tiggy.buffer-ms"))

    def check_position(self, where):
        have = self.counts()
        for j in range(JOINTS):
            cmd = self.pin("joint.%d.motor-pos-cmd" % j)
            want = (llround(cmd * self.scale[j] * 256.0) + 128) // 256
            check(have[j] == want, "%s: joint %d is at step %d by the controller's count, LinuxCNC commands step %d"
                  % (where, j, have[j], want))
            fb = self.pin("joint.%d.motor-pos-fb" % j)
            check(abs(fb - want / self.scale[j]) < 1e-6, "%s: joint %d feedback is %r" % (where, j, fb))

    def healthy(self, where):
        errs = self.errors()
        check(not errs, "%s: LinuxCNC reported: %s" % (where, "; ".join(errs)))
        check(self.pin("tiggy.position-corrections") == 0, "%s: the driver corrected its position" % where)
        short, again = int(self.pin("tiggy.underruns")), int(self.pin("tiggy.segments-resent"))
        if short != self.underruns:
            self.lines.append("note  %s: the controller ran short of motion %d time(s): the computer or the "
                              "network stalled for longer than the queue" % (where, short - self.underruns))
            self.underruns = short
        if again != self.resent:
            self.lines.append("note  %s: %d piece(s) of motion sent a second time (no answer within 40 ms)"
                              % (where, again - self.resent))
            self.resent = again

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
        time.sleep(0.5)

    def home_all(self, timeout=120):
        self.mode(linuxcnc.MODE_MANUAL)
        self.c.teleop_enable(0)
        self.c.wait_complete()
        self.c.home(-1)
        self.wait_for(lambda: all(self.poll().homed[:JOINTS]), timeout, "homing")
        self.settle()

    def run_program(self, text, name, pause_after=None, timeout=120):
        path = os.path.join(os.path.dirname(RESULT) or ".", name)
        with open(path, "w") as f:
            f.write(text)
        self.mode(linuxcnc.MODE_AUTO)
        self.c.program_open(path)
        self.c.auto(linuxcnc.AUTO_RUN, 0)
        time.sleep(1.5)
        check(self.poll().interp_state != linuxcnc.INTERP_IDLE, "the program is not running: %s" % "; ".join(self.errors()))
        if pause_after is not None:
            time.sleep(pause_after)
            self.c.auto(linuxcnc.AUTO_PAUSE)
            self.settle(10)
            self.check_position("paused")
            self.c.auto(linuxcnc.AUTO_RESUME)
        queue = []
        deadline = time.time() + timeout
        while self.poll().interp_state != linuxcnc.INTERP_IDLE:
            check(time.time() < deadline, "the program did not end")
            check(self.s.task_state == linuxcnc.STATE_ON, "the machine went off during the program: %s; %s"
                  % (self.state_name(), "; ".join(self.errors())))
            queue.append(self.pin("tiggy.buffer-ms"))
            time.sleep(0.05)
        self.settle()
        return queue

    # ---- steps
    def step_ready(self):
        self.wait_for(lambda: self.pin("tiggy.connected"), 20, "the controller to answer (tiggy.connected)")
        self.wait_for(lambda: self.pin("tiggy.ready"), 15, "tiggy.ready")
        time.sleep(1.0)                     # LinuxCNC's own parts are still starting
        self.wait_state(linuxcnc.STATE_ESTOP, 5, "LinuxCNC should start in E-stop")
        self.errors()
        self.lines.append("      steps per mm %s; switches %s, probe %s, E-stop input %s" % (
            [abs(int(v)) for v in self.scale], "".join(str(int(self.pin("tiggy.switch.%d" % n))) for n in range(6)),
            int(self.pin("tiggy.probe")), int(self.pin("tiggy.estop-input"))))

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
        text = "G21 G90 G64 P0.02 F1500\nG0 X40 Y40 Z0\nG1 Z-1\n"
        for i in range(1, 241):                     # a spiral of short segments
            a = i * math.pi / 30.0
            r = 2.0 + i * 0.05
            text += "G1 X%.4f Y%.4f Z%.4f\n" % (40.0 + r * math.cos(a), 40.0 + r * math.sin(a), -1.0 - i * 0.01)
        text += "G0 Z0\nG0 X5 Y5\nM2\n"
        self.run_program(text, "tiggy_board_spiral.ngc", pause_after=0.0)
        self.check_position("after the program")
        self.healthy("program")

    def step_jog_and_abort(self):
        self.mode(linuxcnc.MODE_MANUAL)
        self.c.teleop_enable(1)
        self.c.wait_complete()
        before = self.counts()
        self.c.jog(linuxcnc.JOG_CONTINUOUS, False, 0, 20.0)
        time.sleep(0.8)
        check(self.counts()[0] != before[0], "the machine should be jogging: X has not moved")
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

    def step_estop(self):
        self.mode(linuxcnc.MODE_MDI)
        self.c.mdi("G1 F1500 X60")
        time.sleep(1.0)
        t0 = time.time()
        self.c.state(linuxcnc.STATE_ESTOP)

        def stopped():
            a = self.counts()
            time.sleep(0.05)
            return a == self.counts()
        self.wait_for(stopped, 1.0, "the controller to stop on E-stop")
        took = time.time() - t0
        time.sleep(0.4)
        self.check_position("after E-stop")
        self.errors()
        self.machine_on()
        check(all(self.poll().homed[:JOINTS]), "homing was lost over the E-stop")
        self.mdi("G0 X5")
        self.check_position("after recovering from E-stop")
        self.lines.append("      E-stop in mid-move: the controller's count stood still %.2f s after the command" % took)

    def step_long_run(self):
        # back and forth on three axes at speed, with arcs, for some minutes: does the queue hold?
        block = ("G0 X10 Y10 Z0\nG1 X55 Y12 Z-4 F2400\nG1 X12 Y48 Z-1\nG2 X32 Y28 I10 J-10 F1800\n"
                 "G3 X12 Y48 I-10 J10\nG1 X50 Y50 Z-6 F3000\nG1 X10.5 Y10.25 Z-0.5\n")
        t0 = time.time()
        self.run_program("G21 G90 G64 P0.05\n" + block + "M2\n", "tiggy_board_block.ngc")
        one = max(2.0, time.time() - t0)
        n = max(1, int(MINUTES * 60.0 / one))
        queue = self.run_program("G21 G90 G64 P0.05\n" + block * n + "G0 X5 Y5 Z0\nM2\n", "tiggy_board_long.ngc",
                                 timeout=MINUTES * 60.0 * 2 + 120)
        self.check_position("after the long run")
        moving = sorted(q for q in queue if q > 0) or [0.0]
        self.lines.append("      %.0f s of motion; queue in the controller %.0f..%.0f ms (median %.0f); underruns %d, "
                          "sent again %d" % (len(queue) * 0.05, moving[0], moving[-1], moving[len(moving) // 2],
                                             int(self.pin("tiggy.underruns")), int(self.pin("tiggy.segments-resent"))))
        self.healthy("long run")

    # ---- screw cutting
    def reading(self):
        """(Z count, spindle turns) from one status report: the pins are read twice, and taken
        when the spindle's did not change in between."""
        for _ in range(200):
            a = (self.pin("tiggy.spindle-turns"), self.pin("tiggy.spindle-angle"))
            z = int(self.pin("tiggy.%d.counts" % Z))
            if a == (self.pin("tiggy.spindle-turns"), self.pin("tiggy.spindle-angle")):
                return z, a[0] + a[1]
        raise Failure("the spindle's reading changes faster than it can be read")

    def spindle_at(self, hz, first=False):
        """Bring the spindle to hz turns a second, and wait until LinuxCNC sees it there."""
        if first or SPEED_BY_S:
            self.mdi("M3 S%d" % (60 * (hz if SPEED_BY_S else HZ)))
        if not SPEED_BY_S:
            hal.set_p("tiggy.spindle-pwm-freq", "%d" % hz)      # the 'spindle' is the PWM signal itself
        try:
            self.wait_for(lambda: abs(self.pin("tiggy.spindle-speed-fb-rpm") - 60.0 * hz) < 60.0 * hz * 0.02, 10,
                          "the spindle to reach %d rpm" % (60 * hz))
        except Failure:
            raise Failure("the spindle is read as %d rpm, not %d: is the spindle output linked to the sensor input?"
                          % (self.pin("tiggy.spindle-speed-fb-rpm"), 60 * hz))
        time.sleep(1.5)

    def cut(self, text, timeout=120, at=None, then=None):
        """An MDI command, with the readings taken while it is carried out.
        at, then: once Z has gone 'at' steps, call then()."""
        self.mode(linuxcnc.MODE_MDI)
        top = int(self.pin("tiggy.%d.counts" % Z))
        self.c.mdi(text)
        seen, last = [], None
        deadline = time.time() + timeout
        while self.c.wait_complete(0.002) == -1:
            check(time.time() < deadline, "'%s' did not finish" % text)
            r = self.reading()
            if r != last:
                seen.append(r)
                last = r
            if then and top - r[0] >= at:
                then()
                then = None
        check(not any("error" in e.lower() or "abort" in e.lower() for e in self.errors()), "'%s' was not carried out" % text)
        self.settle()
        return seen

    def passes(self, seen, per_rev, length):
        """The passes in a list of readings: runs of Z going down for most of 'length' steps.
        Returns for each (readings used, spread of Z about its place on the spindle, that place)."""
        runs, first = [], None
        for i in range(1, len(seen)):
            cutting = seen[i][0] < seen[i - 1][0]
            if cutting and first is None:
                first = i - 1
            elif not cutting and first is not None:
                if seen[first][0] - seen[i - 1][0] > 0.5 * length:
                    runs.append((first, i))
                first = None
        if first is not None and seen[first][0] - seen[-1][0] > 0.5 * length:
            runs.append((first, len(seen)))
        out = []
        for a, b in runs:
            top = seen[a][0]
            # the part cut at the pitch: clear of the run up to speed and of the end
            part = [(z, t) for z, t in seen[a:b] if 0.25 * length < top - z < 0.9 * length]
            check(len(part) > 10, "only %d readings in a pass" % len(part))
            res = [z + per_rev * t for z, t in part]
            # (A report made in the instant the spindle passes its index can count the turn before
            # the angle has gone round, or after: that reading is a whole turn out, and is put back.)
            mid = sorted(res)[len(res) // 2]
            res = [mid + (r - mid + per_rev / 2.0) % per_rev - per_rev / 2.0 for r in res]
            out.append((len(part), max(res) - min(res), (sum(res) / len(res)) % per_rev))
        return out

    def judge(self, found, per_rev, what, changed=()):
        """changed: the passes (counted from 0) in which the spindle's speed was changed.  There Z is
        allowed more room: a sensor that gives one pulse a turn knows of a change only at its next
        pulse, makes up for it in the turn after, and the axis has to follow that."""
        apart = max(abs((place - found[0][2] + per_rev / 2.0) % per_rev - per_rev / 2.0) for _n, _s, place in found)
        self.lines.append("      %s: %d passes; Z within %s steps of its place on the spindle in them, "
                          "the passes within %.1f steps of the first (a turn is %d steps)" % (
                              what, len(found), ", ".join("%.1f" % spread for _n, spread, _p in found), apart, per_rev))
        for n, (_count, spread, _place) in enumerate(found):
            room = (0.03 if n in changed else 0.015) * per_rev        # 11 and 5 degrees of the spindle
            check(spread <= room, "%s, pass %d: Z wanders %.1f steps against the spindle's angle" % (what, n + 1, spread))
        check(apart <= 0.02 * per_rev, "%s: a pass begins %.1f steps (%.1f degrees) from the first: the threads do not "
              "line up" % (what, apart, apart / per_rev * 360.0))

    def step_spindle(self):
        self.spindle_at(HZ, first=True)
        a = self.reading()[1]
        time.sleep(2.0)
        rate = (self.reading()[1] - a) / 2.0
        check(abs(rate - HZ) <= 0.03 * HZ, "the spindle sensor counts %.1f turns a second at %d rpm" % (rate, 60 * HZ))
        self.lines.append("      spindle read as %d rpm, %.1f turns a second counted" % (self.pin("tiggy.spindle-speed-fb-rpm"), rate))
        self.healthy("spindle")

    def step_g33(self):
        # Three passes of some 80 turns at a pitch of 0.49 mm (392 steps a turn: not a round number
        # of steps between two status reports, which would all fall at the same place in a step).
        # In the second the spindle goes from
        # 3180 to 3540 rpm a third of the way along.
        pitch, depth = 0.49, 40.0
        per_rev = pitch * abs(self.scale[Z])
        length = per_rev * depth / pitch
        z0 = round(self.poll().position[2], 4)
        found, fastest = [], 0
        for n in range(3):
            self.spindle_at(HZ)
            faster = None
            if n == 1 and not SPEED_BY_S:
                faster = lambda: hal.set_p("tiggy.spindle-pwm-freq", "%d" % HZ_OTHER)     # noqa: E731
            seen = self.cut("G33 Z%g K%g" % (z0 - depth, pitch), at=length / 3, then=faster)
            fastest = max(fastest, int(self.pin("tiggy.spindle-speed-fb-rpm")))
            self.check_position("after a G33 pass")
            got = self.passes(seen, per_rev, length)
            check(len(got) == 1, "a G33 pass shows as %d passes in Z's readings" % len(got))
            found += got
            self.mdi("G0 Z%g" % z0)
            self.check_position("back at the start")
        if not SPEED_BY_S:
            check(fastest > 60 * HZ * 1.05, "the spindle's speed did not change in the second pass (%d rpm)" % fastest)
            self.lines.append("      in the second pass the spindle went from %d to %d rpm" % (60 * HZ, fastest))
        self.judge(found, per_rev, "G33", changed=() if SPEED_BY_S else (1,))
        self.healthy("G33")

    def step_g76(self):
        # A threading cycle: three passes, each 0.1 mm deeper, and a spring pass
        pitch, depth = 0.49, 40.0
        per_rev = pitch * abs(self.scale[Z])
        self.spindle_at(HZ)
        z0 = round(self.poll().position[2], 4)
        self.mdi("G18")
        self.mdi("G0 X10")
        seen = self.cut("G76 P%g Z%g I-0.2 J0.1 K0.3 H1" % (pitch, z0 - depth), timeout=300)
        self.check_position("after G76")
        found = self.passes(seen, per_rev, per_rev * depth / pitch)
        check(len(found) == 4, "the G76 cycle shows as %d passes in Z's readings, not 4" % len(found))
        self.judge(found, per_rev, "G76")
        self.mdi("M5")
        self.mdi("G17")
        self.mdi("G0 X0")
        self.check_position("back at the start")
        self.healthy("G76")

    def run(self):
        plans = {
            "moves": (self.step_ready, self.step_on_and_home, self.step_mdi, self.step_program,
                      self.step_jog_and_abort, self.step_estop, self.step_long_run),
            "threading": (self.step_ready, self.step_on_and_home, self.step_spindle, self.step_g33, self.step_g76),
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
        self.lines.append("ALL PASSED (board)" if not self.failed else "FAILED (board)")
        text = "\n".join(self.lines) + "\n"
        print(text, flush=True)
        if RESULT:
            with open(RESULT, "a") as f:
                f.write(text)


if __name__ == "__main__":
    session = Session()
    try:
        session.run()
    finally:
        try:
            session.c.state(linuxcnc.STATE_ESTOP)
        except Exception:
            pass
    sys.exit(1 if session.failed else 0)
