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

TIGGY_TEST_RESULT   file the results are written to
TIGGY_TEST_MINUTES  length of the long run at the end (default 2)
"""

import math
import os
import sys
import time

import hal
import linuxcnc

RESULT = os.environ.get("TIGGY_TEST_RESULT", "")
MINUTES = float(os.environ.get("TIGGY_TEST_MINUTES", "2"))
JOINTS = 3


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

    def run(self):
        for fn in (self.step_ready, self.step_on_and_home, self.step_mdi, self.step_program,
                   self.step_jog_and_abort, self.step_estop, self.step_long_run):
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
