#!/usr/bin/env python3
"""
Step output, spindle sensor input and screw cutting, on a real controller with one wire.

Link the X step terminal to the terminal of the single spindle sensor (on the
ESP32-S3-DevKitC with octal memory: GPIO 1 to GPIO 21; the program names the
two if it finds no link).  NOTHING ELSE may be connected to the X step and
spindle speed terminals: a drive there would be run, and so would a spindle.

    python tools/spindle_loopback_test.py 192.168.1.50 --bench

Part 1, the step output.  With the sensor set to one pulse per revolution,
every step of X is a 'revolution' the controller counts.  X is moved at 5 to
10000 steps a second: the count must be the number of steps sent - which is
the step pin itself seen to pulse, not the firmware's own tally - and the
speed reported must be the step rate.  (Much faster and the sensor input can
no longer tell: it takes pulses less than 50 us apart for switch bounce, and
at 14000 steps a second - 71 us apart - it already drops a few in 70000.)

Part 2, screw cutting.  A pass cannot be cut against a spindle that is one of
the axes, so the X step and spindle speed terminals change places in the
settings, and the wire then carries the spindle PWM: at 50 to 61 Hz a spindle
turning at 3000 to 3660 rpm, whatever the axes do.  Z is sent threading
passes, and every status report made while one is played is held against the
thread asked for.  A report gives Z to the step, so it places Z within one
step, its 'band'; what all reports of a pass have in common is its 'offset'
(none, if the pass lies where it should), and what is judged is the offset
and how far any report lies outside its band about it.  Reports, spindle and
steps all go by the controller's one crystal, so with round numbers every
report would fall at the same place in a step and say the same thing: the
spindle speeds and the steps per turn are chosen (53 Hz, 197 steps) so that
the reports fall all over a step instead.  The fine measurement of a thread
is the simulator's (threading_test.py: every step against the true spindle
angle); this shows that the same holds with the real timer, the real
interrupts and a real signal.

The settings changed (spindle sensor, the two terminals, spindle PWM, status
interval) are put back at the end, also when a test fails or the program is
stopped; the controller restarts three times on the way.  Afterwards the two
terminals are stored as settings of their own rather than taken from the
terminal layout - with the same numbers, which are the same in every layout.

    --sim   the target is tools/firmware_sim.py: part 2 only, with the
            simulator's spindle - a check of this program's own measuring
"""

import argparse
import collections
import math
import sys
import time

import threading_test as tt
from protocol_conformance import Client, Sim, ramp_move, STATUS_V11, SEG_SUBSTEP, SEG_LAST

STATUS_MS, PWM_HZ, MAX_RPM, PWM_ON = 0x0103, 0x0430, 0x0431, 0x0432
PIN_STEP_X, PIN_SPINDLE, PIN_INDEX, SENSOR_PPR, SENSOR_MODE = 0x0500, 0x0515, 0x0522, 0x0523, 0x0524
TYPE = {STATUS_MS: 1, PWM_HZ: 1, PWM_ON: 0, PIN_STEP_X: 0, PIN_SPINDLE: 0, SENSOR_PPR: 1, SENSOR_MODE: 0}
NAME = {STATUS_MS: "status interval, ms", PWM_HZ: "spindle PWM frequency", PWM_ON: "spindle PWM",
        PIN_STEP_X: "X step terminal", PIN_SPINDLE: "spindle terminal",
        SENSOR_PPR: "sensor pulses per revolution", SENSOR_MODE: "sensor kind"}

SF_UNDERRUN, SF_WAIT_INDEX, SF_SYNC, SF_SYNC_LOST = 0x02, 0x08, 0x10, 0x20
IDLE, RUN, HOLD = 0, 1, 2
CAP_SPINDLE_SYNC = 0x80

tt.SLICE_US = 5000              # 127 slices in the controller's queue are then 0.6 s of motion: room for a slow network
TURNS = 150                     # length of a pass
PLAN_HZ = 53                    # the spindle speed the passes are planned for, revolutions a second (3180 rpm)


class LogClient(Client):
    """Keeps every status report, each with the controller's own clock ('up', ms) beside it."""
    up = 0

    def __init__(self, ip):
        self.log = []
        self._status = None
        super().__init__(ip)

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, s):
        self._status = s
        if s is not None:
            s["up"] = LogClient.up
            self.log.append(s)

    @staticmethod
    def parse(data):
        p = Client.parse(data)
        if p and p[0] == 0x20 and len(p[1]) >= STATUS_V11.size:
            LogClient.up = STATUS_V11.unpack_from(p[1])[17]
        return p


class Board:
    """The controller, and the settings of it that this program changes."""

    def __init__(self, ip):
        self.ip = ip
        self.open()

    def open(self):
        self.c = LogClient(self.ip)
        self.c.connect()
        self.info = self.c.handshake()
        self.c.send_io()
        self.c.pump(0.3)

    def close(self):
        for s in (self.c.tcp, self.c.udp):
            try:
                s.close()
            except OSError:
                pass

    def read(self, keys):
        return {k: self.c.cfg_get(k) for k in keys}

    def write(self, values):
        """Set and save; a setting that is read at start-up makes the controller restart: wait for it."""
        new = {k: v for k, v in values.items() if self.c.cfg_get(k) != v}
        if not new:
            return
        for k, v in new.items():
            self.c.cfg_set(k, TYPE[k], v)
        r = self.c.tcp_call(0x45)
        assert r and r[0] == 0x24, "saving the settings was not acknowledged"
        self.close()
        time.sleep(2.5)
        end = time.time() + 40
        while True:
            try:
                self.open()
                break
            except (OSError, AssertionError):
                if time.time() > end:
                    raise RuntimeError("the controller does not answer after its settings were changed")
                time.sleep(1.0)
        wrong = ["%s is %s, not %s" % (NAME[k], self.c.cfg_get(k), v) for k, v in new.items() if self.c.cfg_get(k) != v]
        assert not wrong, "settings not taken: " + "; ".join(wrong)


class Tests:
    def __init__(self, only):
        self.only, self.results = only, []

    def __call__(self, name, fn):
        if self.only and not any(w in name for w in self.only.split(",")):
            return
        try:
            note = fn()
            self.results.append((name, "ok"))
            print("ok    %s%s" % (name, ("   [%s]" % note) if note else ""))
        except Exception as e:                      # one failed test must not hide the others
            self.results.append((name, "FAIL: %s" % e))
            print("FAIL  %s: %s" % (name, e))


def fold(x):
    return (x + 0.5) % 1.0 - 0.5


def median(v):
    v = sorted(v)
    return v[len(v) // 2]


def deg(turns):
    return "%.1f deg" % (abs(turns) * 360.0)


def idle(c, stream_no):
    """Controller at rest with nothing queued and Z at 0; returns a new stream."""
    def new():
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0])

    for n in range(80):
        s = c.pump(0.1)
        if s and s["state"] in (6, 7):
            c.send(7)
        elif s and s["state"] in (RUN, HOLD) and n > 5:         # left by a test that failed: drop it
            new()
        elif s and s["state"] == IDLE and not s["jog"] and s["queued_ms"] == 0:
            break
    else:
        raise AssertionError("controller did not become idle: %s" % c.status)
    new()
    z = c.pump(0.1)["pos"][2]
    if z:
        c.stream(stream_no[0], ramp_move([0, 0, -z, 0, 0, 0], 1.0 + abs(z) / 8000.0))
        c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and s["pos"][2] == 0, 30.0, "Z to go back to 0")
        new()
    c.sflags_seen = 0
    return stream_no[0]


# ====================================================================== part 1

def x_move(length, rate, direction):
    """Accelerate over half a second, travel at 'rate' steps a second, decelerate: slices of 1/256 steps."""
    accel = 2.0 * rate
    ta, da = 0.5, rate * 0.25
    assert 2 * da < length
    total = 2 * ta + (length - 2 * da) / rate

    def place(t):
        if t <= ta:
            return 0.5 * accel * t * t
        if t <= total - ta:
            return da + rate * (t - ta)
        return length - 0.5 * accel * max(0.0, total - t) ** 2

    n = int(math.ceil(total * 1e6 / tt.SLICE_US))
    q = [int(round(256 * place(i * tt.SLICE_US / 1e6))) for i in range(n)] + [256 * length]
    segs = [([direction * (q[i + 1] - q[i]), 0, 0, 0, 0, 0], tt.SLICE_US, SEG_SUBSTEP | (SEG_LAST if i == n - 1 else 0))
            for i in range(n)]
    return segs, total


def step_output(board, test):
    c, stream_no = board.c, [40]

    def run(steps, rate, direction):
        """Returns (X travel, pulses counted, speeds reported in mid-move, ran short of motion)."""
        segs, total = x_move(steps, rate, direction)
        st = idle(c, stream_no)
        s0 = c.pump(0.6)                        # (at rest long enough for the sensor to take the 'spindle' as stopped)
        seen, began = [], [None]

        def watch(s):
            if s["state"] == RUN and began[0] is None:
                began[0] = time.perf_counter()
            if began[0] is not None and 0.8 < time.perf_counter() - began[0] < total - 0.7:
                seen.append(s["rpm"])
            return False

        c.stream(st, segs, timeout=total + 30, until=watch)
        end = time.perf_counter() + total + 10
        while time.perf_counter() < end:
            s = c.pump(0.02)
            watch(s)
            if s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == len(segs):
                break
        else:
            raise AssertionError("the move did not finish: %s" % s)
        s1 = c.pump(0.3)
        return s1["pos"][0] - s0["pos"][0], s1["index_count"] - s0["index_count"], seen, bool(c.sflags_seen & SF_UNDERRUN)

    def t_link():
        moved, counted, _seen, _short = run(100, 50, 1)
        back = run(100, 50, -1)
        assert moved == 100 and back[0] == -100, "X moved %d and %d steps for 100 and -100" % (moved, back[0])
        if counted == 0:
            raise AssertionError("no pulse reached the sensor input: link the X step terminal (GPIO %d) to the sensor "
                                 "terminal (GPIO %d)" % (c.cfg_get(PIN_STEP_X), c.cfg_get(PIN_INDEX)))
        assert counted == 100, "100 steps, %d pulses counted" % counted

    def t_rate(rate):
        def fn():
            steps, want = max(40, 5 * rate), 60 * rate
            notes = []
            for direction in (1, -1):
                moved, counted, seen, short = run(steps, rate, direction)
                assert moved == direction * steps, "X moved %d steps for %d" % (moved, direction * steps)
                assert counted == steps, "%d steps, %d pulses counted (%d missing)" % (steps, counted, steps - counted)
                if want <= 65535:
                    assert seen, "no speed reading taken"
                    # (the speed is the time between two pulses: the interrupt that times them may
                    # be 20 us late, which at 500 steps a second is one part in a hundred)
                    assert all(abs(r - want) <= 2 + want * 20e-6 * rate for r in seen), \
                        "speed read as %d..%d rpm, expected %d" % (min(seen), max(seen), want)
                    notes.append("%d..%d rpm" % (min(seen), max(seen)))
                if short:
                    notes.append("ran short of motion for a moment: the network")
            return "%d steps each way, all counted%s" % (steps, "; " + ", ".join(notes) if notes else "")
        return fn

    test("X step reaches the sensor input", t_link)
    if test.results and test.results[-1][1] != "ok":
        return False
    for rate in (5, 50, 500, 1000, 4000, 10000):
        if rate <= 0.7 * board.info["max_rate"]:
            test("%d steps a second" % rate, t_rate(rate))
    idle(c, stream_no)
    return True


# ====================================================================== part 2

def screw_cutting(c, info, test, sim, verbose):
    stream_no = [120]
    turning = dict(hz=50, on=False)
    half_speed = 0 if sim else c.cfg_get(MAX_RPM) // 2          # 50 % duty
    # Z steps per turn of the spindle: a prime, so that the steps made between two reports are never a round number
    spr = next(n for n in (197, 149, 101, 61, 37, 19) if n * 61 <= 0.6 * info["max_rate"] or n == 19)
    OUT, OFFSET = 0.004, 0.004                  # turns: about a degree and a half
    was_ms = c.cfg_get(STATUS_MS)

    def spindle(hz=None, on=None):
        if hz is not None:
            turning["hz"] = hz
        if on is not None:
            turning["on"] = on
        if sim:
            sim(cmd="spindle", follow=0, rpm=60.0 * turning["hz"] if turning["on"] else 0.0, accel=1e6)
            return
        if hz is not None:
            c.cfg_set(PWM_HZ, 1, hz)
        if on is not None:
            c.io.update(spindle=1 if on else 0, rpm=half_speed)
        c.send_io()

    def spinning(hz, settle=1.0):
        """Spindle at hz revolutions a second; returns what the controller makes of it."""
        spindle(hz=hz, on=True)
        c.pump(settle)
        a = c.pump(0.05)
        b = c.pump(2.0)
        revs = (b["index_count"] - a["index_count"]) * 1000.0 / (b["up"] - a["up"])
        assert abs(b["rpm"] - 60 * hz) <= 60 * hz * 0.02, "reports %d rpm with the output at %d Hz" % (b["rpm"], hz)
        assert abs(revs - hz) <= hz * 0.02, "counted %.2f turns a second with the output at %d Hz" % (revs, hz)
        return revs, b["rpm"]

    def make_pass(steps_per_rev=spr):
        tt.ACCEL = steps_per_rev * PLAN_HZ * 2.0        # half a second to the pitch speed
        return tt.Pass(60.0 * PLAN_HZ, length=steps_per_rev * TURNS, steps_per_rev=steps_per_rev)

    def back(p):
        return ramp_move([0, 0, p.length, 0, 0, 0], max(4.5, p.length / (0.5 * info["max_rate"])))

    def play(st, segs, events=(), timeout=60.0):
        """Stream segs to the end.  events: (seconds after the first Z step, function).  Returns the reports."""
        start = len(c.log)
        p0 = c.pump(0.05)["pos"]
        todo = sorted(events, key=lambda e: e[0])
        began = [None]

        def tick(s):
            if began[0] is None and s["pos"][2] != p0[2]:
                began[0] = time.perf_counter()
            while todo and began[0] is not None and time.perf_counter() - began[0] >= todo[0][0]:
                todo.pop(0)[1]()
            return False

        c.stream(st, segs, timeout=timeout, until=tick)
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            s = c.pump(0.005)
            tick(s)
            if s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == (len(segs) & 0xFFFF):
                break
        else:
            raise AssertionError("did not finish: %s" % c.status)
        return p0, c.log[start:]

    def lock(p0, reports, p, label, segs=None):
        """How a pass lay on the spindle (see the top of the file).
        segs: (first, last + 1) segment numbers of the pass, if the reports hold more than one."""
        pts = []
        for s in reports:
            z = (p0[2] - s["pos"][2]) * (1 if p.direction < 0 else -1)
            if segs and not (segs[0] <= s["seg"] < segs[1]):
                continue
            if s["sflags"] & SF_SYNC and 0 < z < p.length:
                turns = s["index_count"] + s["spindle_pos"] / 65536.0
                lo, hi = p.turns_at_step(z), p.turns_at_step(z + 1)        # the report was made between two steps
                pts.append((s["up"], z, turns, 0.5 * (lo + hi), 0.5 * (hi - lo), s["rpm"]))
        assert len(pts) > 20, "%s: only %d reports while in step with the spindle" % (label, len(pts))
        whole = round(median(t - plan for _u, _z, t, plan, _q, _r in pts))      # the turn the pass began at
        res = [(u, z, fold(t - plan - whole), q, rpm) for u, z, t, plan, q, rpm in pts]
        mid = [r for r in res if p.length * 0.25 < r[1] < p.length * 0.75]
        offset = median(r[2] for r in mid)
        out = max(max(0.0, abs(r[2] - offset) - r[3]) for r in res)
        rate = (mid[-1][1] - mid[0][1]) * 1000.0 / max(1, mid[-1][0] - mid[0][0])
        rpm = median(r[4] for r in mid)
        if verbose:
            step = max(1, len(res) // 14)
            print("      %s: %d reports; from the offset, degrees (band): %s" % (
                label, len(res), " ".join("%+.1f(%.1f)" % (360 * (r[2] - offset), 360 * r[3]) for r in res[::step])))
        return dict(offset=offset, out=out, rate=rate, rpm=rpm, res=res, n=len(res), us=offset * 60e6 / rpm if rpm else 0.0)

    def clean(p0):
        s = c.pump(0.2)
        assert s["pos"] == p0, "ended at %s, began at %s" % (s["pos"][:3], p0[:3])
        assert c.sflags_seen & SF_SYNC, "never reported playing by the spindle"
        assert not (c.sflags_seen & (SF_UNDERRUN | SF_SYNC_LOST)), "stream flags seen: %02X" % c.sflags_seen

    def good(m, what="", out=OUT):
        assert m["out"] <= out, "%sa report %s outside its band" % (what, deg(m["out"]))
        assert abs(m["offset"]) <= OFFSET, "%soffset %s (%.0f us)" % (what, deg(m["offset"]), m["us"])

    # ------------------------------------------------------------------ tests
    def t_frequency():
        # The controller started with its PWM at 1 kHz.  A host that then asks for 50 Hz must get
        # 50 Hz: the sensor input says what the pin really does.
        idle(c, stream_no)
        spindle(on=True)
        c.pump(1.0)
        was = c.pump(0.3)["rpm"]
        assert abs(was - 60000) <= 600, "reports %d rpm with the output at 1000 Hz" % was
        spindle(hz=50)
        c.pump(1.0)
        now = c.pump(0.3)["rpm"]
        assert abs(now - 3000) <= 30, "the output was asked to go from 1000 Hz to 50 Hz and is read as %d rpm (%.0f Hz)" % (now, now / 60.0)
        return "1000 Hz read as %d rpm, then 50 Hz as %d" % (was, now)

    def t_seen():
        idle(c, stream_no)
        a, rpm_a = spinning(50)
        b, rpm_b = spinning(57)
        spindle(on=False)
        c.pump(0.6)
        s = c.pump(0.2)
        assert s["rpm"] == 0, "reports %d rpm with the spindle output off" % s["rpm"]
        n = s["index_count"]
        s = c.pump(1.0)
        assert s["index_count"] == n, "counted %d turns with the output off" % (s["index_count"] - n)
        return "50 Hz read as %.2f rev/s, %d rpm; 57 Hz as %.2f rev/s, %d rpm; off as 0" % (a, rpm_a, b, rpm_b)

    def t_period():
        # each turn is timed by one interrupt: how steadily, with Z stepping thousands of times a second?
        st = idle(c, stream_no)
        spinning(PLAN_HZ)
        n = len(c.log)
        c.pump(3.0)
        rest = collections.Counter(s["rpm"] for s in c.log[n:])
        p = make_pass()
        p0, rep = play(st, p.segs + back(p))
        clean(p0)
        busy = collections.Counter(s["rpm"] for s in rep if s["sflags"] & SF_SYNC)

        def spread(h):
            us = [60e6 / r for r in h if r]
            return "%d..%d rpm (%.0f us in %d)" % (min(h), max(h), max(us) - min(us), 1000000 // PLAN_HZ)
        assert max(busy) - min(busy) <= 30, "speed read as %d..%d rpm at a steady %d" % (min(busy), max(busy), 60 * PLAN_HZ)
        return "at rest %s; in a pass %s" % (spread(rest), spread(busy))

    def t_steady():
        st = idle(c, stream_no)
        spinning(PLAN_HZ)
        p = make_pass()
        p0, rep = play(st, p.segs + back(p))
        clean(p0)
        m = lock(p0, rep, p, "steady")
        good(m)
        assert abs(m["rate"] - spr * m["rpm"] / 60.0) <= 0.005 * m["rate"], "Z at %.0f steps/s at %d rpm" % (m["rate"], m["rpm"])
        return "%d reports, the worst %s outside its band; offset %s (%.0f us); Z at %.0f steps/s" % (
            m["n"], deg(m["out"]), deg(m["offset"]), m["us"], m["rate"])

    def t_other_speed():
        # the host planned for 3180 rpm; the spindle is doing 3540
        st = idle(c, stream_no)
        spinning(59)
        p = make_pass()
        p0, rep = play(st, p.segs + back(p))
        clean(p0)
        m = lock(p0, rep, p, "other speed")
        good(m)
        assert abs(m["rate"] - spr * 59) <= 0.01 * m["rate"], "Z at %.0f steps/s: by the clock it would be %d, by the spindle %d" % (
            m["rate"], spr * PLAN_HZ, spr * 59)
        return "worst %s outside its band; offset %s (%.0f us); Z at %.0f steps/s, planned for %d: it went by the spindle" % (
            deg(m["out"]), deg(m["offset"]), m["us"], m["rate"], spr * PLAN_HZ)

    def t_two_passes():
        # the second pass, at another speed, must lie on the first
        st = idle(c, stream_no)
        spinning(PLAN_HZ)
        plans = [make_pass(), make_pass()]
        segs = plans[0].segs + back(plans[0])
        n1 = len(segs)
        segs += plans[1].segs + back(plans[1])
        p0, rep = play(st, segs, events=[(5.0, lambda: spindle(hz=59))], timeout=90.0)
        clean(p0)
        a = lock(p0, rep, plans[0], "first pass", (0, len(plans[0].segs)))
        b = lock(p0, rep, plans[1], "second pass", (n1, n1 + len(plans[1].segs)))
        assert abs(b["rpm"] - a["rpm"]) > 200, "the spindle speed did not change between the passes (%d, %d rpm)" % (a["rpm"], b["rpm"])
        good(a, "first pass: ")
        good(b, "second pass: ")
        apart = b["offset"] - a["offset"]
        assert abs(apart) <= 0.003, "the second pass lies %s from the first" % deg(apart)
        return "at %d and %d rpm: the second lies %s from the first; worst %s and %s outside their bands" % (
            a["rpm"], b["rpm"], deg(apart), deg(a["out"]), deg(b["out"]))

    def t_speed_change():
        # The spindle goes from 53 to 59 rev/s in mid-pass.  (What a report shows is Z against the
        # controller's own reading of the spindle; how far a single sensor's reading is from the
        # truth while the speed changes is the simulator's to measure.)
        st = idle(c, stream_no)
        spinning(PLAN_HZ)
        p = make_pass()
        p0, rep = play(st, p.segs + back(p), events=[(1.2, lambda: spindle(hz=59))])
        clean(p0)
        m = lock(p0, rep, p, "speed change")
        rpms = [r[4] for r in m["res"]]
        assert min(rpms) < 60 * PLAN_HZ + 50 and max(rpms) > 60 * 59 - 50, \
            "the speed did not change during the pass (%d..%d rpm)" % (min(rpms), max(rpms))
        good(m, out=0.02)
        return "speed read as %d..%d rpm in the pass; worst %s outside its band" % (min(rpms), max(rpms), deg(m["out"]))

    def t_waits():
        # spindle at rest when the pass is sent: nothing moves until it turns
        st = idle(c, stream_no)
        spindle(on=False)
        c.pump(1.0)
        p = make_pass()
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs[:100])
        s = c.wait(lambda s: s["sflags"] & SF_WAIT_INDEX, 3.0, "the wait for the index")
        s = c.pump(1.5)
        assert s["pos"] == p0, "moved to %s before the spindle turned" % s["pos"][:3]
        assert s["state"] == RUN and s["sflags"] & SF_WAIT_INDEX, "state %d, flags %02X while waiting" % (s["state"], s["sflags"])
        assert not (c.sflags_seen & SF_UNDERRUN), "reported an underrun while waiting"
        spindle(hz=PLAN_HZ, on=True)
        start = len(c.log)
        rest = p.segs[100:] + back(p)
        c.stream(st, rest, first=100, timeout=60.0)
        total = 100 + len(rest)
        c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == total, 30.0, "the pass to finish")
        clean(p0)
        m = lock(p0, c.log[start:], p, "from rest")
        good(m)
        return "waited 1.5 s without a step; then worst %s outside its band, offset %s" % (deg(m["out"]), deg(m["offset"]))

    def t_spindle_stops():
        # the spindle stops in mid-pass: Z stops with it and stays; the host then gives the pass up
        st = idle(c, stream_no)
        spinning(PLAN_HZ)
        p = make_pass()
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs, until=lambda s: p0[2] - s["pos"][2] > p.length // 3, timeout=30.0)
        spindle(on=False)
        c.pump(0.5)
        a = c.pump(0.05)
        b = c.pump(1.0)
        assert a["pos"][2] == b["pos"][2], "Z moved %d steps in a second with the spindle at rest" % (b["pos"][2] - a["pos"][2])
        into = p0[2] - b["pos"][2]
        assert 0 < into < p.length, "Z is %d steps into a pass of %d" % (into, p.length)
        assert b["state"] == RUN and b["sflags"] & SF_SYNC and b["rpm"] == 0, \
            "state %d, flags %02X, %d rpm with the spindle stopped in a pass" % (b["state"], b["sflags"], b["rpm"])
        t0 = time.perf_counter()
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0])
        took = time.perf_counter() - t0
        s = c.pump(0.3)
        assert s["state"] == IDLE and s["pos"][2] == b["pos"][2], "after the flush: state %d, Z %d" % (s["state"], s["pos"][2])
        c.stream(stream_no[0], ramp_move([0, 0, into, 0, 0, 0], 3.0))
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 10.0, "the move back")
        assert s["pos"] == p0, "back at %s, began at %s" % (s["pos"][:3], p0[:3])
        return "Z stopped %d steps into the pass and stayed; flushed in %.2f s and taken back" % (into, took)

    def t_step_rates():
        # other step rates and spindle speeds
        rows, failed, rates = [], [], []
        for hz, steps_per_rev in ((53, 19), (53, 61), (51, spr), (57, spr), (61, spr), (53, 293)):
            if steps_per_rev * hz > 0.8 * info["max_rate"]:
                continue
            rates.append(steps_per_rev * hz)
            st = idle(c, stream_no)
            spinning(hz, settle=0.5)
            p = make_pass(steps_per_rev)
            p0, rep = play(st, p.segs + back(p))
            clean(p0)
            m = lock(p0, rep, p, "%d Hz, %d steps a turn" % (hz, steps_per_rev))
            rows.append("%d steps/s at %d rpm: offset %+.0f us, worst %s" % (steps_per_rev * hz, 60 * hz, m["us"], deg(m["out"])))
            # (the coarser the steps, the less exactly the reports give the offset)
            if m["out"] > OUT or abs(m["offset"]) > OFFSET + 0.1 / steps_per_rev:
                failed.append(rows[-1])
        if verbose or failed:
            for row in rows:
                print("      " + row)
        assert not failed, "; ".join(failed)
        return "%d passes, Z at %d to %d steps a second" % (len(rows), min(rates), max(rates))

    c.cfg_set(STATUS_MS, 1, 10)             # a status report every 10 ms
    try:
        if not sim:
            test("spindle PWM frequency changed from 1 kHz to 50 Hz", t_frequency)
        test("spindle output read back as a spindle: speed and count", t_seen)
        test("how steadily a turn is timed", t_period)
        test("pass at a steady spindle speed", t_steady)
        test("pass with the spindle faster than planned for", t_other_speed)
        test("two passes at different speeds lie on each other", t_two_passes)
        test("spindle speeds up in mid-pass", t_speed_change)
        test("waits for a spindle at rest", t_waits)
        test("spindle stops in mid-pass: Z stops with it, and the pass can be given up", t_spindle_stops)
        test("passes at other step rates and spindle speeds", t_step_rates)
    finally:
        try:
            spindle(hz=50, on=False)
            idle(c, stream_no)
            if sim:                             # (on a controller main() puts every setting back)
                c.cfg_set(STATUS_MS, 1, was_ms)
        except Exception as e:
            print("could not bring the controller to rest: %s" % e)


# ====================================================================== main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip")
    ap.add_argument("--bench", action="store_true",
                    help="yes: nothing is connected to the X step and spindle speed terminals but the link")
    ap.add_argument("--sim", action="store_true", help="the target is tools/firmware_sim.py (part 2 only)")
    ap.add_argument("-k", default="", help="only run tests whose name contains this (several: separate with commas)")
    ap.add_argument("-v", action="store_true", help="print the measurements of every pass")
    args = ap.parse_args()
    test = Tests(args.k)

    if args.sim:
        board = Board(args.ip)
        sim = Sim()
        sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        sim(cmd="encoder", mode="pulse", ppr=1, index=1)
        sim(cmd="spindle", follow=0, rpm=0.0, accel=1e6, ripple=0.0, ripple_hz=0.0)
        info = board.c.handshake()
        try:
            screw_cutting(board.c, info, test, sim, args.v)
        finally:                                # leave the simulated machine as it was found
            sim(cmd="encoder", mode="quad", ppr=0, index=1)
            sim(cmd="spindle", follow=1, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
            sim(cmd="set", lim_lo=[-4000] * 6, lim_hi=[200000] * 6, phys=[0] * 6)
    else:
        if not args.bench:
            print(__doc__)
            print("Not started: say --bench when the controller is on the bench with only the link on it.")
            return 2
        board = Board(args.ip)
        was = board.read(TYPE)
        step_pin, spindle_pin = was[PIN_STEP_X], was[PIN_SPINDLE]
        try:
            board.write({SENSOR_MODE: 1, SENSOR_PPR: 1})

            def t_offered():
                assert board.info["caps"] & CAP_SPINDLE_SYNC, "capabilities %02X" % board.info["caps"]

            test("screw cutting offered once a single sensor is set", t_offered)
            print("--- step output: X step (GPIO %d) into the sensor input (GPIO %d)" % (step_pin, board.c.cfg_get(PIN_INDEX)))
            if all(r[1] == "ok" for r in test.results) and step_output(board, test):
                if spindle_pin == 0xFF or step_pin == 0xFF:
                    print("--- screw cutting: left out, this controller has no spindle output to stand in for a spindle")
                else:
                    board.write({PIN_STEP_X: spindle_pin, PIN_SPINDLE: step_pin, PWM_ON: 1, PWM_HZ: 1000})
                    print("--- screw cutting: the spindle output on GPIO %d as the spindle" % step_pin)
                    screw_cutting(board.c, board.info, test, None, args.v)
        finally:
            try:
                board.write(was)
                print("--- settings as they were")
            except Exception as e:
                print("\nTHE SETTINGS WERE NOT PUT BACK (%s).  They were:" % e)
                for k, v in was.items():
                    print("    %-32s (0x%04X) %s" % (NAME[k], k, v))

    failed = [r for r in test.results if r[1] != "ok"]
    print("\n%d passed, %d failed" % (len(test.results) - len(failed), len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
