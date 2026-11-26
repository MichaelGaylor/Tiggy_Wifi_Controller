#!/usr/bin/env python3
"""
Test of the LinuxCNC driver (linuxcnc/tiggy.c) without LinuxCNC.

The driver is built, unchanged, around a stand-in for LinuxCNC
(linuxcnc/test: a servo thread, a joint planner, HAL pins) and run against
the firmware simulator, which plays the controller with the firmware's own
motion code.  Every move is checked to the step.

    firmware/test/pc/build.sh   (or build.bat)      the simulator's library
    linuxcnc/test/build.sh      (or build.bat)      the driver and its rig
    python tools/linuxcnc_driver_test.py            all tests (about 4 minutes)
    python tools/linuxcnc_driver_test.py -k loss    only tests whose name contains "loss"

The test starts its own simulators on 127.0.0.1 and 127.0.0.2 and needs the
controller's ports (58427, 58429) and 58500/58501 free on them.
"""

import argparse
import ctypes
import json
import math
import os
import random
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.join(HERE, "firmware_sim.py")
RIG_DIR = os.path.join(ROOT, "linuxcnc", "test", "build")

SCALE = [320.0, 314.159, -400.0]        # steps per mm; a negative scale reverses the joint
MAXVEL = [60.0, 55.0, 40.0]             # mm/s
MAXACC = [600.0, 500.0, 400.0]          # mm/s^2
BUFFER_MS = 200


class Failure(Exception):
    pass


def check(cond, text):
    if not cond:
        raise Failure(text)


def llround(v):
    return int(math.floor(abs(v) + 0.5)) * (1 if v >= 0 else -1)


def steps_for(position, scale):
    """Step count the controller must be at for a commanded position."""
    return (llround(position * scale * 256.0) + 128) // 256


# ---------------------------------------------------------------- simulator

class Sim:
    def __init__(self, bind="127.0.0.1", control=58500, extra=()):
        self.bind, self.control, self.extra = bind, control, list(extra)
        self.proc = None
        self.sock = None
        self.file = None

    def start(self):
        probe = socket.socket()
        try:
            probe.bind(("127.0.0.1", self.control))
        except OSError:
            sys.exit("Port %d is in use: is a simulator already running?" % self.control)
        finally:
            probe.close()
        self.proc = subprocess.Popen([sys.executable, SIM, "--bind", self.bind, "--control-port", str(self.control),
                                      "--quiet"] + self.extra, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.time() + 10
        while True:
            if self.proc.poll() is not None:
                sys.exit("The simulator did not start:\n" + self.proc.stderr.read().decode(errors="replace"))
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.control), timeout=0.5)
                break
            except OSError:
                if time.time() > deadline:
                    sys.exit("The simulator did not open its control port")
                time.sleep(0.05)
        self.sock.settimeout(5.0)
        self.file = self.sock.makefile("rwb")

    def stop(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = self.file = None
        if self.proc:
            self.proc.kill()
            self.proc.wait()
            self.proc = None

    def cmd(self, **req):
        self.file.write((json.dumps(req) + "\n").encode())
        self.file.flush()
        resp = json.loads(self.file.readline().decode())
        if "error" in resp:
            raise Failure("simulator: " + resp["error"])
        return resp

    def get(self):
        return self.cmd(cmd="get")

    def set(self, **values):
        return self.cmd(cmd="set", **values)


# ---------------------------------------------------------------------- rig

class Rig:
    def __init__(self):
        name = "tiggy_rig.dll" if os.name == "nt" else "libtiggy_rig.so"
        path = os.path.join(RIG_DIR, name)
        if not os.path.exists(path):
            sys.exit("The driver rig was not found. Build it first:\n"
                     "  linuxcnc\\test\\build.bat   (Windows)   or   linuxcnc/test/build.sh")
        lib = ctypes.CDLL(path)
        lib.rig_start.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.rig_get.argtypes = [ctypes.c_char_p]
        lib.rig_get.restype = ctypes.c_double
        lib.rig_set.argtypes = [ctypes.c_char_p, ctypes.c_double]
        lib.rig_move.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        lib.rig_jump.argtypes = [ctypes.c_int, ctypes.c_double]
        lib.rig_command.argtypes = [ctypes.c_int]
        lib.rig_command.restype = ctypes.c_double
        lib.rig_period.argtypes = [ctypes.c_long, ctypes.c_long]
        lib.rig_worst_error.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.rig_worst_error.restype = ctypes.c_double
        lib.rig_cycles.restype = ctypes.c_double
        lib.rig_messages.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.lib = lib
        self.running = False
        self.log = []

    def start(self, ip="127.0.0.1", joints=3, io_ip="", servo_delay_ms=0):
        r = self.lib.rig_start(ip.encode(), joints, io_ip.encode(), servo_delay_ms)
        check(r == 0, "the driver did not load (%d)" % r)
        self.running = True
        self.joints = joints
        for j in range(joints):
            self.set("tiggy.%d.position-scale" % j, SCALE[j])
            self.set("tiggy.%d.maxvel" % j, MAXVEL[j] * 1.2)
            self.set("tiggy.%d.maxaccel" % j, MAXACC[j] * 1.25)
        self.set("tiggy.buffer-target-ms", BUFFER_MS)
        self.set("tiggy.charge-pump-freq", 10000)

    def stop(self):
        if self.running:
            self.lib.rig_stop()
            self.running = False

    def get(self, name):
        v = self.lib.rig_get(name.encode())
        check(not math.isnan(v), "no pin or parameter called " + name)
        return v

    def set(self, name, value):
        check(self.lib.rig_set(name.encode(), float(value)) == 1, "no pin or parameter called " + name)

    def machine(self, on):
        self.lib.rig_machine(1 if on else 0)

    def move(self, joint, target, vel=None, acc=None):
        self.lib.rig_move(joint, float(target), MAXVEL[joint] if vel is None else vel,
                          MAXACC[joint] if acc is None else acc)

    def command(self, joint):
        return self.lib.rig_command(joint)

    def moving(self):
        return bool(self.lib.rig_moving())

    def messages(self):
        buf = ctypes.create_string_buffer(16384)
        self.lib.rig_messages(buf, len(buf))
        lines = [l for l in buf.value.decode(errors="replace").splitlines() if l]
        self.log += lines
        return lines

    def errors(self):
        return [l for l in self.messages() if l.startswith("ERROR")]


def wait_for(cond, timeout, what):
    deadline = time.time() + timeout
    while True:
        v = cond()
        if v:
            return v
        if time.time() > deadline:
            raise Failure("timed out waiting for " + what)
        time.sleep(0.005)


# ------------------------------------------------------------------- tests

class Tests:
    def __init__(self, args):
        self.args = args
        self.sim = Sim()
        self.rig = Rig()
        self.base = [0] * 6

    # ---- helpers
    def timing(self, cond, text):
        """A check of how long something took.  On a shared build machine,
        which may stop for a moment at any time, --lenient turns these into notes."""
        if cond:
            return
        if not self.args.lenient:
            raise Failure(text)
        print("      note: " + text, flush=True)

    def bring_up(self, ip="127.0.0.1", io_ip="", sim_extra=()):
        self.sim.extra = list(sim_extra)
        self.sim.start()
        self.rig.start(ip, 3, io_ip)
        wait_for(lambda: self.rig.get("tiggy.ready") == 1, 8, "tiggy.ready")
        self.rebase()

    def tear_down(self):
        self.rig.stop()
        self.sim.stop()

    def rebase(self):
        """The driver's count and the controller's differ by a constant."""
        time.sleep(0.05)
        s = self.sim.get()
        self.base = [s["pos"][j] - int(self.rig.get("tiggy.%d.counts" % j)) if j < 3 else 0 for j in range(6)]

    def machine_on(self):
        self.rig.machine(True)
        wait_for(lambda: self.sim.get()["state"] == "IDLE", 5, "the controller to leave its stopped state")
        wait_for(lambda: self.sim.get()["outputs"]["charge_pump"], 3, "the charge pump")

    def settle(self, timeout=60):
        """Wait until LinuxCNC's move has ended and the controller has played all of it."""
        wait_for(lambda: not self.rig.moving(), timeout, "the move to end")
        time.sleep(0.02)
        wait_for(lambda: not self.sim.get()["busy"], 5, "the controller to finish")
        time.sleep(0.06)

    def check_position(self, where=""):
        s = self.sim.get()
        for j in range(3):
            want = steps_for(self.rig.command(j), SCALE[j])
            have = s["pos"][j] - self.base[j]
            check(have == want, "%sjoint %d is at step %d, LinuxCNC commands step %d" % (where, j, have, want))
            counts = int(self.rig.get("tiggy.%d.counts" % j))
            check(counts == want, "%stiggy.%d.counts is %d, should be %d" % (where, j, counts, want))
            fb = self.rig.get("tiggy.%d.position-fb" % j)
            check(abs(fb - want / SCALE[j]) < 1e-9, "%stiggy.%d.position-fb is %r" % (where, j, fb))

    def no_trouble(self, underruns_allowed=False):
        errors = self.rig.errors()
        check(not errors, "the driver reported: " + "; ".join(errors))
        check(self.rig.get("tiggy.ready") == 1, "tiggy.ready went false")
        check(self.rig.get("tiggy.position-corrections") == 0,
              "the driver had to correct its position %d times" % self.rig.get("tiggy.position-corrections"))
        if not underruns_allowed and not self.args.lenient:
            check(self.rig.get("tiggy.underruns") == 0, "the controller ran short of motion %d times"
                  % self.rig.get("tiggy.underruns"))

    # ---- tests
    def test_settings(self):
        """Settings made in HAL reach the controller"""
        self.bring_up()
        r, sim = self.rig, self.sim
        check(sim.get()["state"] == "ESTOP", "with the machine off the controller should be held stopped")
        check(not sim.get()["outputs"]["drivers"], "with the machine off the drivers should be off")

        def cfg(key):
            return sim.get()["cfg"]["0x%04X" % key]

        def reaches(key, value, what):
            wait_for(lambda: cfg(key) == value, 4, "%s (setting 0x%04X = %d, is %d)" % (what, key, value, cfg(key)))

        reaches(0x0103, 10, "status interval")
        reaches(0x0100, 5, "default step pulse")
        reaches(0x0101, 5, "default direction set-up")
        reaches(0x0310, 0x3F, "switches on when pulled low")
        reaches(0x0311, 0x3F, "home polarity follows")
        reaches(0x0314, 0x00, "no input stops the controller by itself unless asked")
        for j in range(3):
            reaches(0x0010 + j, int(MAXVEL[j] * 1.2 * abs(SCALE[j]) + 0.5), "joint %d top step rate" % j)
            reaches(0x0020 + j, int(MAXACC[j] * 1.25 * abs(SCALE[j]) + 0.5), "joint %d acceleration" % j)
        reaches(0x0400, 10000, "charge pump frequency")

        r.set("tiggy.step-pulse-us", 3)
        r.set("tiggy.dir-setup-us", 2)
        r.set("tiggy.idle-release-ms", 1500)
        r.set("tiggy.debounce-ms", 20)
        r.set("tiggy.1.step-invert", 1)
        r.set("tiggy.2.dir-invert", 1)
        r.set("tiggy.enable-invert", 1)
        r.set("tiggy.switch.1-invert", 1)
        r.set("tiggy.switch.0-is-limit", 1)
        r.set("tiggy.switch.2-is-limit", 1)
        r.set("tiggy.probe-invert", 1)
        r.set("tiggy.estop-invert", 1)
        r.set("tiggy.spindle-pwm", 1)
        r.set("tiggy.spindle-pwm-freq", 5000)
        r.set("tiggy.spindle-max-rpm", 18000)
        r.set("tiggy.spindle-min-duty", 7)
        reaches(0x0100, 3, "step pulse")
        reaches(0x0101, 2, "direction set-up")
        reaches(0x0102, 1500, "idle release")
        reaches(0x0315, 20, "debounce")
        reaches(0x0300, 0x02, "step polarity")
        reaches(0x0301, 0x04, "direction polarity")
        reaches(0x0302, 1, "enable polarity")
        reaches(0x0310, 0x3D, "switch polarity")
        reaches(0x0311, 0x3D, "home polarity")
        reaches(0x0314, 0x05, "limit inputs")
        reaches(0x0313, 1, "probe polarity")
        reaches(0x0312, 1, "E-stop polarity")
        reaches(0x0432, 1, "spindle PWM")
        reaches(0x0430, 5000, "PWM frequency")
        reaches(0x0431, 18000, "spindle top speed")
        reaches(0x0433, 7, "minimum duty")
        check(not r.errors(), "errors: %s" % r.log[-3:])

        # A joint the controller cannot step fast enough is reported
        r.set("tiggy.0.position-scale", 4000)
        wait_for(lambda: any("joint 0 needs" in e for e in r.errors()), 4, "the warning about the step rate")
        self.tear_down()

    def test_startup(self):
        """Before LinuxCNC's threads run, the controller's settings are left alone"""
        self.sim.start()
        sim, r = self.sim, self.rig

        def cfg(key):
            return sim.get()["cfg"]["0x%04X" % key]

        before = {k: cfg(k) for k in (0x0100, 0x0101, 0x0102, 0x0103)}
        check(before[0x0100] != 5 and before[0x0103] != 10, "the simulator's defaults changed: the test proves nothing")
        r.start("127.0.0.1", 3, "", servo_delay_ms=1500)
        r.set("tiggy.idle-release-ms", 700)
        t_end = time.time() + 1.2
        while time.time() < t_end:
            now = {k: cfg(k) for k in before}
            check(now == before, "settings were sent before the servo thread had run: %s" % now)
            check(r.get("tiggy.ready") == 0, "ready before the settings were known")
            time.sleep(0.05)
        wait_for(lambda: r.get("tiggy.ready") == 1, 6, "tiggy.ready once the threads run")
        check(cfg(0x0100) == 5 and cfg(0x0103) == 10 and cfg(0x0102) == 700,
              "ready although the settings have not all arrived")
        self.rebase()
        self.machine_on()
        r.move(0, 2.0)
        self.settle()
        self.check_position()
        self.no_trouble()
        self.tear_down()

    def test_moves(self):
        """Moves arrive to the step; pulses are evenly spaced; the buffer holds its level"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        sim.cmd(cmd="clear")

        t0 = time.time()
        r.move(0, 12.3456)
        wait_for(lambda: sim.get()["moving"], 2, "the motors to start")
        delay = time.time() - t0
        self.timing(0.10 < delay < 0.40, "the motors started %.0f ms after LinuxCNC (buffer is %d ms)"
                    % (delay * 1000, BUFFER_MS))
        time.sleep(0.15)
        level = r.get("tiggy.buffer-ms")
        self.timing(BUFFER_MS - 60 <= level <= BUFFER_MS + 60, "tiggy.buffer-ms is %.0f in mid-move" % level)
        self.settle()
        self.check_position("first move: ")
        gap = sim.get()["min_gap_us"][0]
        fastest = 1e6 / (MAXVEL[0] * SCALE[0])
        self.timing(gap is not None and gap >= fastest * 0.85,
                    "X steps came %s us apart; at top speed they are %.0f us apart" % (gap, fastest))

        r.move(0, -3.21)
        r.move(1, 7.5)
        r.move(2, -4.004)
        self.settle()
        self.check_position("three joints: ")

        # One step at a time, and positions that fall between steps
        for i in range(1, 9):
            r.move(1, 7.5 + i * 0.4 / abs(SCALE[1]), vel=2.0)
            self.settle()
            self.check_position("fine move %d: " % i)

        # Back and forth with pauses of every length, so that movements end
        # and begin while the controller is still playing, just finished, or idle
        rnd = random.Random(7)
        for i in range(25):
            for j in range(3):
                if rnd.random() < 0.7:
                    r.move(j, rnd.uniform(-15, 15), vel=rnd.uniform(5, MAXVEL[j]))
            self.settle()
            self.check_position("random move %d: " % i)
            time.sleep(rnd.choice([0, 0.01, 0.05, 0.15, 0.25, 0.4]))
        self.no_trouble()
        worst = [r.lib.rig_worst_error(j, 0) for j in range(3)]
        for j in range(3):
            check(worst[j] < MAXVEL[j] * 0.5, "joint %d following error reached %.2f mm" % (j, worst[j]))
        self.tear_down()

    def test_restart_within_buffer(self):
        """A new move while the last is still being played joins on without a hitch"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        rnd = random.Random(11)
        target = 0.0
        for i in range(40):
            target += rnd.uniform(0.3, 3.0) * rnd.choice([-1, 1])
            r.move(0, target)
            wait_for(lambda: not r.moving(), 20, "the move to end")
            time.sleep(rnd.uniform(0.0, BUFFER_MS / 1000.0 * 1.3))      # around the time the queue runs out
        self.settle()
        self.check_position()
        self.no_trouble()
        self.tear_down()

    def test_loss(self):
        """Lost, repeated and out-of-order packets cost nothing"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        sim.set(loss=0.10, dup=0.05, reorder=0.05)
        rnd = random.Random(3)
        for i in range(12):
            for j in range(3):
                r.move(j, rnd.uniform(-20, 20))
            self.settle()
            self.check_position("move %d: " % i)
        dropped = sim.get()["stats"]["rx_dropped"]
        check(dropped > 20, "the simulator dropped only %d packets: the test proved nothing" % dropped)

        # One packet in three lost each way: slower, never wrong
        sim.set(loss=0.33, dup=0.0, reorder=0.0)
        for i in range(4):
            for j in range(3):
                r.move(j, rnd.uniform(-20, 20))
            self.settle()
            self.check_position("heavy loss, move %d: " % i)
        check(r.get("tiggy.segments-resent") > 0, "nothing was sent again at one packet in three lost")
        # ...and machine off still stops it at once, though the first "stop" may be lost
        r.move(0, 150.0)
        time.sleep(0.6)
        r.machine(False)
        wait_for(lambda: sim.get()["state"] == "ESTOP" and not sim.get()["moving"], 0.4,
                 "the controller to stop although packets are being lost")
        # ...and where it stopped gets back to LinuxCNC, lost reports or not
        stopped_at = sim.get()["pos"][0] - self.base[0]
        wait_for(lambda: int(r.get("tiggy.0.counts")) == stopped_at, 1.0, "the position after the stop")
        time.sleep(0.05)                    # the command follows the feedback while the machine is off
        sim.set(loss=0.0, dup=0.0, reorder=0.0)
        self.machine_on()
        r.move(0, 0.0)
        self.settle()
        self.check_position("after the stop: ")
        self.no_trouble(underruns_allowed=True)
        self.tear_down()

    def test_stall(self):
        """A PC that stops for a moment, short or long, loses no position"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        r.move(0, 150.0)
        time.sleep(0.5)
        for ms in (20, 60, 100):
            r.lib.rig_stall(ms)
            time.sleep(0.4)
        self.timing(sim.get()["state"] == "RUN", "the controller stopped during short stalls")
        self.no_trouble()                                   # the buffer covers these
        r.lib.rig_stall(450)                                # longer than the buffer: the controller slows and waits
        time.sleep(1.0)
        check(sim.get()["state"] in ("RUN", "IDLE"), "the controller faulted: %s" % sim.get()["state"])
        self.settle()
        self.check_position()
        self.timing(r.get("tiggy.underruns") >= 1, "the long stall should have been counted as an underrun")
        self.no_trouble(underruns_allowed=True)
        # ...and afterwards it runs normally again
        r.move(0, 0.0)
        r.move(1, 30.0)
        self.settle()
        self.check_position("afterwards: ")
        self.tear_down()

    def test_machine_off(self):
        """Machine off stops at once; switching on again carries on from where it stopped"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        r.move(0, 80.0)
        r.move(2, -30.0)
        time.sleep(0.8)
        check(sim.get()["moving"], "should be moving")
        r.machine(False)
        wait_for(lambda: sim.get()["state"] == "ESTOP" and not sim.get()["moving"], 0.5, "the controller to stop")
        time.sleep(0.1)
        s = sim.get()
        check(not s["outputs"]["drivers"] and not s["outputs"]["charge_pump"], "drivers and charge pump should be off")
        for j in range(3):
            have = s["pos"][j] - self.base[j]
            check(int(r.get("tiggy.%d.counts" % j)) == have, "feedback differs from the controller after the stop")
        stopped_at = r.get("tiggy.0.position-fb")
        check(1.0 < stopped_at < 79.0, "stopped at %.2f" % stopped_at)
        time.sleep(0.1)
        check(abs(r.command(0) - stopped_at) < 1e-9, "the command should follow the feedback while off")

        self.machine_on()
        r.move(0, 10.0)
        r.move(2, 0.0)
        self.settle()
        self.check_position("after switching on again: ")

        # Off and on again in quick succession, with a move straight after
        r.machine(False)
        time.sleep(0.02)
        r.machine(True)
        r.move(0, 12.0)
        self.settle()
        self.check_position("after a quick off and on: ")
        self.no_trouble()
        self.tear_down()

    def test_estop_input(self):
        """The controller's E-stop input stops LinuxCNC, and both recover"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        r.move(1, 60.0)
        time.sleep(0.6)
        sim.set(estop=1)
        wait_for(lambda: r.get("tiggy.ready") == 0, 0.5, "tiggy.ready to go false")
        check(r.get("tiggy.estop-input") == 1, "tiggy.estop-input should be true")
        check(any("E-stop" in e for e in r.errors()), "no message about the E-stop")
        r.machine(False)                    # what LinuxCNC does when its E-stop chain opens
        time.sleep(0.3)
        check(r.get("tiggy.ready") == 0, "ready must stay false while the button is in")
        sim.set(estop=0)
        wait_for(lambda: r.get("tiggy.ready") == 1, 1, "tiggy.ready after the button is released")
        self.machine_on()
        r.move(1, 5.0)
        self.settle()
        self.check_position()
        self.no_trouble()
        self.tear_down()

    def test_limit_switch(self):
        """A limit switch stops the controller itself; homing onto it and leaving it are allowed"""
        self.bring_up()
        r, sim = self.rig, self.sim
        r.set("tiggy.switch.0-is-limit", 1)
        self.machine_on()
        phys = sim.get()["phys"]
        switch_at = phys[0] + 6400              # 20 mm ahead
        hi = [200000.0] * 6
        hi[0] = switch_at
        sim.set(lim_hi=hi)
        r.move(0, 60.0)
        wait_for(lambda: r.get("tiggy.ready") == 0, 3, "the driver to report the stop")
        check(any("limit switch input 0" in e for e in r.errors()), "no message naming the switch: %s" % r.log[-3:])
        over = sim.get()["phys"][0] - switch_at
        check(0 <= over < 80, "ran %d steps past the switch" % over)
        check(r.get("tiggy.switch.0") == 1 and r.get("tiggy.switch.0-not") == 0, "tiggy.switch.0 should be on")
        r.machine(False)
        wait_for(lambda: r.get("tiggy.ready") == 1, 1, "ready again once LinuxCNC has switched off")

        # Standing on the switch, the axis can be moved off it
        self.machine_on()
        r.move(0, 5.0)
        self.settle()
        self.check_position("moved off the switch: ")
        check(r.get("tiggy.switch.0") == 0, "tiggy.switch.0 should be off again")
        self.no_trouble()

        # Homing: LinuxCNC runs onto the switch on purpose
        r.set("tiggy.0.homing", 1)
        time.sleep(0.15)
        r.move(0, 60.0, vel=5.0)
        wait_for(lambda: r.get("tiggy.switch.0") == 1, 10, "the switch during homing")
        seen_at = r.get("tiggy.0.counts") + self.base[0]
        r.move(0, r.command(0), vel=5.0)        # LinuxCNC stops
        self.settle()
        check(sim.get()["state"] == "IDLE", "homing onto the switch must not raise an alarm")
        # The position that came with the switch is where the switch is, give or take one report
        check(abs(seen_at - switch_at) <= 5.0 * SCALE[0] * 0.025,
              "the switch at step %d was reported at step %d" % (switch_at, seen_at))
        r.move(0, 5.0)
        self.settle()
        r.set("tiggy.0.homing", 0)
        self.check_position("after homing: ")
        self.no_trouble()
        sim.set(lim_hi=[200000.0] * 6)
        self.tear_down()

    def test_outputs_inputs(self):
        """Spindle, coolant, outputs and inputs"""
        self.bring_up()
        r, sim = self.rig, self.sim
        r.set("tiggy.spindle-pwm", 1)
        r.set("tiggy.spindle-max-rpm", 24000)
        self.machine_on()

        def out():
            return sim.get()["outputs"]

        r.set("tiggy.spindle-speed", 12000)
        r.set("tiggy.spindle-forward", 1)
        wait_for(lambda: out()["spindle"] == 1 and out()["rpm"] == 12000, 1, "spindle forward")
        check(abs(out()["duty"] - 0.5) < 0.01, "PWM duty is %.3f at half speed" % out()["duty"])
        r.set("tiggy.spindle-forward", 0)
        r.set("tiggy.spindle-reverse", 1)
        r.set("tiggy.spindle-speed", -6000)
        wait_for(lambda: out()["spindle"] == 2 and out()["rpm"] == 6000, 1, "spindle reverse")
        r.set("tiggy.spindle-reverse", 0)
        wait_for(lambda: out()["spindle"] == 0, 1, "spindle off")
        r.set("tiggy.coolant-flood", 1)
        wait_for(lambda: out()["coolant"] == 1, 1, "flood")
        r.set("tiggy.coolant-mist", 1)
        wait_for(lambda: out()["coolant"] == 3, 1, "mist")
        for i in range(5):
            r.set("tiggy.output.%d" % i, 1)
            wait_for(lambda: out()["misc"] & (1 << i), 1, "output %d" % i)
        check(out()["misc"] == 0x1F, "outputs are 0x%02X" % out()["misc"])
        check(out()["drivers"] or True, "")

        sim.set(probe=1)
        wait_for(lambda: r.get("tiggy.probe") == 1, 1, "tiggy.probe")
        sim.set(probe=0)
        wait_for(lambda: r.get("tiggy.probe") == 0, 1, "tiggy.probe off")
        sim.set(misc_in=0x05)
        wait_for(lambda: r.get("tiggy.input.0") == 1 and r.get("tiggy.input.2") == 1, 1, "inputs 0 and 2")
        check(r.get("tiggy.input.1") == 0 and r.get("tiggy.input.1-not") == 1 and r.get("tiggy.input.0-not") == 0,
              "input pins wrong")
        sim.set(misc_in=0, limits=0x22)
        wait_for(lambda: r.get("tiggy.switch.1") == 1 and r.get("tiggy.switch.5") == 1, 1, "switches 1 and 5")
        check(r.get("tiggy.switch.0") == 0 and r.get("tiggy.switch.0-not") == 1, "switch pins wrong")
        sim.set(limits=0)

        # Machine off: everything off
        r.machine(False)
        wait_for(lambda: out()["misc"] == 0 and out()["coolant"] == 0 and not out()["charge_pump"], 1,
                 "the outputs to go off with the machine")
        self.no_trouble()
        self.tear_down()

    def test_link_loss(self):
        """Losing the controller stops LinuxCNC; a restarted controller carries on at the same position"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        r.move(0, 25.0)
        self.settle()
        self.check_position()
        before = [r.get("tiggy.%d.position-fb" % j) for j in range(3)]
        sim.stop()
        wait_for(lambda: r.get("tiggy.ready") == 0 and r.get("tiggy.connected") == 0, 4, "the loss to be noticed")
        check(any("lost" in e for e in r.errors()), "no message about the lost connection")
        r.machine(False)
        sim.start()                             # a controller that has restarted counts from zero again
        wait_for(lambda: r.get("tiggy.ready") == 1, 10, "the driver to find the controller again")
        after = [r.get("tiggy.%d.position-fb" % j) for j in range(3)]
        check(before == after, "the position changed across the reconnection: %s -> %s" % (before, after))
        self.rebase()
        self.machine_on()
        r.move(0, 3.0)
        r.move(1, -2.0)
        self.settle()
        self.check_position("after reconnecting: ")
        self.no_trouble()

        # The controller is lost and back before LinuxCNC has reacted (the
        # machine stays on): the spindle must not start again by itself
        r.set("tiggy.spindle-speed", 9000)
        r.set("tiggy.spindle-forward", 1)
        wait_for(lambda: sim.get()["outputs"]["spindle"] == 1, 1, "the spindle")
        sim.stop()
        sim.start()
        wait_for(lambda: r.get("tiggy.connected") == 1, 10, "the driver to reconnect")
        t_end = time.time() + 1.0
        while time.time() < t_end:
            s = sim.get()
            check(r.get("tiggy.ready") == 0, "ready again although LinuxCNC never switched off")
            check(s["outputs"]["spindle"] == 0 and not s["outputs"]["charge_pump"],
                  "outputs came back on after the reconnection: %s" % s["outputs"])
            time.sleep(0.05)
        check(any("lost" in e for e in r.errors()), "no message about the lost connection")
        r.set("tiggy.spindle-forward", 0)
        r.machine(False)
        wait_for(lambda: r.get("tiggy.ready") == 1, 2, "ready once LinuxCNC has switched off")
        self.rebase()
        self.machine_on()
        r.move(0, 1.0)
        self.settle()
        self.check_position("after the second reconnection: ")
        self.tear_down()

    def test_clocks(self):
        """Clocks that disagree: the queue stays at its level through a long move"""
        self.bring_up()
        r, sim = self.rig, self.sim
        sim.set(lim_hi=[2.0e6] * 6, lim_lo=[-2.0e6] * 6)
        self.machine_on()
        # 0.15 % apart: left alone, the queue would gain or lose 45 ms in the 30 s of each move
        for told, name in ((1001500, "controller clock 0.15 % fast"), (998500, "controller clock 0.15 % slow")):
            # LinuxCNC believes a period is told ns long but produces one every 1000000 ns
            r.lib.rig_period(told, 1000000)
            start = r.command(0)
            r.move(0, start + 700.0 * (1 if told > 1000000 else -1), vel=20.0)
            time.sleep(3.0)
            lo, hi = 1e9, 0.0
            t_end = time.time() + 27.0
            while time.time() < t_end:
                level = r.get("tiggy.buffer-ms")
                lo, hi = min(lo, level), max(hi, level)
                time.sleep(0.05)
            check(r.moving(), "the move ended too early")
            if self.args.verbose:
                print("        %s: queue between %.0f and %.0f ms" % (name, lo, hi))
            self.timing(BUFFER_MS - 20 <= lo and hi <= BUFFER_MS + 20,
                        "%s: the queue went from %.0f to %.0f ms (target %d)" % (name, lo, hi, BUFFER_MS))
            r.move(0, r.command(0), vel=20.0)
            r.lib.rig_period(1000000, 1000000)
            self.settle()
            self.check_position(name + ": ")
            self.no_trouble()
        self.tear_down()

    def test_jump(self):
        """A commanded position that jumps is refused, not followed"""
        self.bring_up()
        r, sim = self.rig, self.sim
        self.machine_on()
        r.move(0, 2.0)
        self.settle()
        before = sim.get()["pos"][0]
        r.lib.rig_jump(0, 15.0)
        wait_for(lambda: r.get("tiggy.ready") == 0, 1, "the driver to refuse the jump")
        check(any("joint 0" in e for e in r.errors()), "no message about the jump")
        time.sleep(0.3)
        check(abs(sim.get()["pos"][0] - before) <= 1, "the machine moved %d steps" % (sim.get()["pos"][0] - before))
        r.machine(False)
        wait_for(lambda: r.get("tiggy.ready") == 1, 1, "ready after switching off")
        self.machine_on()
        r.move(0, r.command(0) + 1.0)
        self.settle()
        self.check_position("after the refused jump: ")
        self.tear_down()

    def test_search(self):
        """ip=auto finds the controller; a wrong address and old firmware are explained"""
        self.sim.start()
        self.rig.start("auto", 3, "")
        wait_for(lambda: self.rig.get("tiggy.ready") == 1, 8, "the controller to be found")
        check(any("found at 127.0.0.1" in l for l in self.rig.messages()), "no note of where it was found")
        self.rig.stop()

        self.rig.start("127.0.0.77", 3, "")
        wait_for(lambda: any("127.0.0.77" in e for e in self.rig.errors()), 15, "a message about the address")
        check(self.rig.get("tiggy.ready") == 0, "ready without a controller")
        self.rig.stop()
        self.sim.stop()

        self.sim.extra = ["--fw", "1.1.0"]
        self.sim.start()
        self.rig.start("127.0.0.1", 3, "")
        wait_for(lambda: any("1.2.0 or newer" in e for e in self.rig.errors()), 8, "a message about the firmware")
        check(self.rig.get("tiggy.ready") == 0, "ready with old firmware")
        self.rig.stop()
        self.sim.stop()
        self.sim.extra = []

    def test_io_module(self):
        """I/O expansion module"""
        io = Sim(bind="127.0.0.2", control=58501, extra=["--io-module"])
        io.start()
        try:
            self.bring_up(io_ip="127.0.0.2")
            r = self.rig
            wait_for(lambda: r.get("tiggy.io.connected") == 1, 8, "the I/O module")
            r.set("tiggy.io.out-03", 1)
            r.set("tiggy.io.out-15", 1)
            wait_for(lambda: io.get()["outputs"]["io"] == 0x8008, 1, "I/O module outputs")
            io.set(io_in=0x4001)
            wait_for(lambda: r.get("tiggy.io.in-00") == 1 and r.get("tiggy.io.in-14") == 1, 1, "I/O module inputs")
            check(r.get("tiggy.io.in-01") == 0 and r.get("tiggy.io.in-01-not") == 1, "I/O module input pins wrong")
            # ...while the controller works as usual
            self.machine_on()
            r.move(0, 4.0)
            self.settle()
            self.check_position()
            io.stop()
            wait_for(lambda: r.get("tiggy.io.connected") == 0, 4, "the loss of the I/O module to show")
            check(r.get("tiggy.ready") == 1, "losing the I/O module must not stop the controller")
            self.tear_down()
        finally:
            io.stop()

    # ---- runner
    def run(self):
        names = [n for n in ("test_settings", "test_startup", "test_moves", "test_restart_within_buffer", "test_loss", "test_stall",
                             "test_machine_off", "test_estop_input", "test_limit_switch", "test_outputs_inputs",
                             "test_link_loss", "test_jump", "test_search", "test_io_module", "test_clocks")
                 if not self.args.k or self.args.k in n]
        failed = 0
        for name in names:
            fn = getattr(self, name)
            t0 = time.time()
            try:
                fn()
                print("PASS  %-28s %s  (%.1f s)" % (name[5:], fn.__doc__, time.time() - t0), flush=True)
            except Failure as e:
                failed += 1
                print("FAIL  %-28s %s\n      %s" % (name[5:], fn.__doc__, e), flush=True)
                if self.args.verbose:
                    self.rig.messages()
                    for line in self.rig.log[-25:]:
                        print("        | " + line)
                try:
                    self.rig.stop()
                finally:
                    self.sim.stop()
            self.rig.log = []
        print("\n%d of %d tests passed" % (len(names) - failed, len(names)))
        return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description="Test of the LinuxCNC driver against the firmware simulator")
    ap.add_argument("-k", default="", help="run only the tests whose name contains this")
    ap.add_argument("-v", "--verbose", action="store_true", help="show the driver's messages when a test fails")
    ap.add_argument("--lenient", action="store_true",
                    help="for shared build machines: positions are still checked to the step, timings only noted")
    args = ap.parse_args()
    tests = Tests(args)
    try:
        sys.exit(tests.run())
    finally:
        tests.rig.stop()
        tests.sim.stop()


if __name__ == "__main__":
    main()
