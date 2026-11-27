#!/usr/bin/env python3
"""
The encoder inputs on a real controller, with no encoder: it makes the signals itself.

Three wire links, each from an output to an input (the program names the
terminals of the board it finds, and says which link is missing if one is):

    spindle output  to  encoder A        ESP32-S3-DevKitC, octal memory:  GPIO 9  to GPIO 16
    charge pump     to  encoder B                                         GPIO 39 to GPIO 17
    Z step          to  index pulse                                       GPIO 42 to GPIO 21

NOTHING ELSE may be connected to those six terminals - a spindle would be
run, a drive on Z stepped.

    python tools/encoder_loopback_test.py 192.168.1.50 --bench

A and B.  The spindle output at half duty and the charge pump are both square
waves from the chip's PWM timers, which run from one clock.  Set to the same
frequency they stay as they stand to each other - which is what an encoder's
A and B are: so many lines a second, in the one direction or the other as
chance has it when they start.  (Chance may also have them change together,
which is no signal: the program then starts the charge pump again.)  With the
encoder set to 400 lines, 10600 lines a second are a spindle at 1590 rpm.

The index pulse.  Z is stepped by the clock, one step for every 400 lines:
that pulse comes once per turn of the count, and it comes from the same
crystal as A and B, so it always meets the count at the same place - unless
the count is wrong.

What is tested, the encoder set up with its index pulse:
  - A and B are counted: the speed read is the frequency made.
  - Every Z pulse is taken as an index pulse, and found at one place.
Then as an encoder without one, so that nothing corrects the count:
  - No count is lost or gained, at 10600 to 62300 lines a second (a quarter
    of a million counts): Z is stepped by the clock, twenty steps to a turn
    of the count, and its steps are held against the count in every status
    report made meanwhile.  A report says the count was here before a step
    and there after it; thousands of them together say where the steps fall
    to a count or so, and that must not move in up to three million counts.
  - Screw cutting by A and B, as in encoder_bench_test.py: a pass lies where
    it should on the spindle, a second pass on the first, Z stops with the
    signal when it is switched off in mid-pass, and a pass sent while it is
    off waits for it.

What it does not test is an encoder: its own signals, its wiring, and
whatever stands between it and these inputs.  It shows that what reaches the
inputs is read rightly, at speed, by the real pulse counter, the real
interrupts and the firmware as flashed.

The settings changed (terminal layout to four axes, the spindle sensor,
spindle PWM and its frequency, the charge pump's frequency, status interval)
are put back at the end, also when a test fails or the program is stopped;
the controller restarts three times on the way.  Take the links off again
afterwards: with six axes two of those inputs are limit inputs.

    --sim   the target is tools/firmware_sim.py, which is given the links
"""

import argparse
import math
import sys
import time

import encoder_bench_test as ebt
import spindle_loopback_test as slt
from encoder_bench_test import (Follow, index_place, LAYOUT, SENSOR_MODE, SENSOR_PPR, STATUS_MS, PIN_A, PIN_B, PIN_INDEX,
                                FOUR_AXES, WITH_INDEX, NO_INDEX, CAP_ENCODER, CAP_SPINDLE_SYNC)
from protocol_conformance import Sim, SEG_SUBSTEP, SEG_LAST
from spindle_loopback_test import Board, Tests, idle, IDLE, SF_UNDERRUN

CP_HZ, PWM_HZ, MAX_RPM, PWM_ON = 0x0400, 0x0430, 0x0431, 0x0432
PIN_STEP_Z, PIN_SPINDLE, PIN_PUMP = 0x0502, 0x0515, 0x0517
slt.TYPE.update({CP_HZ: 1})
slt.NAME.update({CP_HZ: "charge pump frequency"})
IO_CHARGE_PUMP = 0x04

LINES = 400                     # what the encoder is set to: 1600 counts to a turn
CLOCK = 80.0e6                  # of the chip's PWM timers
RATES = (10600, 25000, 45000, 62300)    # lines a second; the first is also the spindle of the threading passes
SECONDS = 12                    # each is counted for
STEPS = 20                      # of Z to a turn of the count, while it is counted against the clock


def made(asked, steps):
    """The frequency a PWM timer of 'steps' steps really makes of the one asked for: the
    clock divided by a number with eight binary places."""
    return CLOCK * 256.0 / (steps * math.floor(CLOCK * 256.0 / (asked * steps) + 0.5))


def pick(wanted):
    """The frequency nearest 'wanted' that the spindle's timer (1024 steps) and the charge
    pump's (256) make exactly alike, as (what to ask for, clock periods of a quarter line).

    And one that is no round number: status reports, steps and the signal all go by one
    crystal, and with round numbers every report would fall at the same place in a turn
    and say the same thing."""
    for d in range(3000):
        for f in (wanted + d, wanted - d):
            n = int(round(CLOCK / 4.0 / f))
            if 1000 <= f <= 65535 and abs(CLOCK / 4.0 / f - n) < 0.1 and math.gcd(n, 10) == 1 and \
                    abs(made(f, 1024) - made(f, 256)) < 1e-9 * f:
                return f, n
    raise AssertionError("no frequency near %d that both timers make alike" % wanted)


class Signal:
    """A and B: the spindle output at half duty and the charge pump, at one frequency."""

    def __init__(self, c):
        self.c, self.asked, self.n, self.is_on, self.since = c, None, None, False, 0.0
        self.half = c.cfg_get(MAX_RPM) // 2

    @property
    def lines(self):                            # lines a second
        return CLOCK / 4.0 / self.n

    @property
    def rpm(self):
        return 60.0 * self.lines / LINES

    def on(self, wanted=None):
        if wanted is not None:
            asked, self.n = pick(wanted)
            if asked != self.asked:
                self.c.cfg_set(PWM_HZ, 1, asked)
                self.c.cfg_set(CP_HZ, 1, asked)
                self.asked = asked
        self.c.io.update(spindle=1, rpm=self.half)
        self.c.io["flags"] |= IO_CHARGE_PUMP
        self.c.send_io()
        self.is_on, self.since = True, time.perf_counter()

    def off(self):
        self.c.io.update(spindle=0, rpm=0)
        self.c.io["flags"] &= ~IO_CHARGE_PUMP
        self.c.send_io()
        self.is_on = False

    def again(self):
        """Start the charge pump anew: A and B then stand to each other some other way."""
        self.c.io["flags"] &= ~IO_CHARGE_PUMP
        self.c.send_io()
        self.c.pump(0.15)
        self.c.io["flags"] |= IO_CHARGE_PUMP
        self.c.send_io()
        self.since = time.perf_counter()

    def right(self, s):
        return abs(s["rpm"] - self.rpm) <= max(2.0, 0.003 * self.rpm)

    def settle(self, tries=8):
        """Wait until the count runs at the signal's speed.  Returns the starts it took, 0 if it never did."""
        for n in range(1, tries + 1):
            self.c.pump(0.6)
            if all(self.right(s) for s in self.c.log[-20:]):
                return n
            self.again()
        return 0


class Own:
    """At the bench in place of the person with the drill (encoder_bench_test.by_drill): switches the signal."""

    def __init__(self, signal, wanted):
        self.signal, self.wanted = signal, wanted

    def cue(self, what, _text):
        if what == "drill":
            self.signal.on(self.wanted)
        else:
            self.signal.off()

    def waiting(self):
        g = self.signal
        if g.is_on and time.perf_counter() - g.since > 1.2 and not g.right(g.c.status):
            g.again()


def one_place(rows, turns_per_ms):
    """rows: (the controller's clock in ms, angle of the count as a fraction of a turn, Z steps made).
    Z made STEPS steps, evenly, for every turn the count should have made.  A step made
    between two reports found the count somewhere between what the two say; if the count
    keeps to the clock every step finds it where it belongs, and that place lies within
    what all the pairs of reports allow.
    Returns (how far they contradict each other, in steps of Z - negative: by how much they
             agree; the direction the count ran in; steps)."""
    best = None
    t0, a0 = rows[0][0], rows[0][1]
    for way in (1, -1):
        # the count at each report, in steps of Z since the first: the angle says where in a
        # turn, the clock which turn (it is the same crystal: there is nothing to drift)
        w = [STEPS * way * (angle + round(a0 + way * turns_per_ms * (up - t0) - angle)) for up, angle, _z in rows]
        lo, hi, steps = -1e30, 1e30, 0
        for j in range(1, len(rows)):
            first, last = rows[j - 1][2] + 1, rows[j][2]
            if last >= first:
                lo, hi, steps = max(lo, w[j - 1] - first), min(hi, w[j] - last), steps + last - first + 1
        if steps and (best is None or lo - hi < best[0]):
            best = (lo - hi, way, steps)
    return best


def count_by_clock(c, signal, stream_no):
    """Z stepped by the clock, STEPS steps for every turn of the count, for SECONDS; returns one_place()."""
    period = LINES * signal.n // 20             # us: the time of LINES lines
    assert period * 20 == LINES * signal.n
    slices = int(SECONDS * 1e6 / period)
    n = slices * STEPS
    segs = [([0, 0, 256 * STEPS, 0, 0, 0], period, SEG_SUBSTEP | (SEG_LAST if i == slices - 1 else 0)) for i in range(slices)]
    for attempt in (1, 2, 3):
        st = idle(c, stream_no)
        start = len(c.log)
        z0 = c.pump(0.05)["pos"][2]
        c.stream(st, segs, timeout=SECONDS + 30)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == slices, SECONDS + 30, "Z to finish")
        assert s["pos"][2] - z0 == n, "Z made %d steps of %d" % (s["pos"][2] - z0, n)
        if not c.sflags_seen & SF_UNDERRUN:
            break
        assert attempt < 3, "the controller ran short of motion each time: the network"
    assert all(signal.right(r) for r in c.log[start:]), "the speed read did not stay at %d rpm" % signal.rpm
    rows = [(r["up"], r["spindle_pos"] / 65536.0, r["pos"][2] - z0) for r in c.log[start:]]
    return one_place(rows, signal.lines / LINES / 1000.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip")
    ap.add_argument("--bench", action="store_true",
                    help="yes: nothing is connected to the spindle output, the charge pump, Z step and the encoder inputs but the links")
    ap.add_argument("-k", default="", help="only run tests whose name contains this (several: separate with commas)")
    ap.add_argument("-v", action="store_true", help="print the measurements of every pass")
    ap.add_argument("--sim", action="store_true", help="the target is tools/firmware_sim.py")
    ap.add_argument("--sim-links", default="", help=argparse.SUPPRESS)        # missing=2 (1 A, 2 B, 3 index), slip=100 (ppm)
    ap.add_argument("--expect", default="", help=argparse.SUPPRESS)           # succeed only if a test fails saying this
    args = ap.parse_args()
    if not (args.bench or args.sim):
        print(__doc__)
        print("Not started: say --bench when the controller is on the bench with only the three links on it.")
        return 2

    test = Tests(args.k)
    sim = None
    if args.sim:
        sim = Sim()
        spec = {k: int(v) for k, v in (item.split("=") for item in args.sim_links.split(",") if item)}
        sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        sim(cmd="fit", lines=0)
        sim(cmd="links", on=1, missing=spec.get("missing", 0), slip=spec.get("slip", 0) / 1e6)

    board = Board(args.ip)
    was = board.read((LAYOUT, SENSOR_MODE, SENSOR_PPR, STATUS_MS, PWM_ON, PWM_HZ, CP_HZ))
    signal = None
    try:
        board.write({LAYOUT: FOUR_AXES, SENSOR_MODE: WITH_INDEX, SENSOR_PPR: LINES, PWM_ON: 1, PWM_HZ: 10000, CP_HZ: 10000})
        c = board.c
        pin = {k: c.cfg_get(k) for k in (PIN_SPINDLE, PIN_A, PIN_PUMP, PIN_B, PIN_STEP_Z, PIN_INDEX)}
        link = {"A": "GPIO %d (spindle output) to GPIO %d (encoder A)" % (pin[PIN_SPINDLE], pin[PIN_A]),
                "B": "GPIO %d (charge pump) to GPIO %d (encoder B)" % (pin[PIN_PUMP], pin[PIN_B]),
                "Z": "GPIO %d (Z step) to GPIO %d (index pulse)" % (pin[PIN_STEP_Z], pin[PIN_INDEX])}
        stream_no = [60]

        def t_set_up():
            missing = [name for k, name in ((PIN_SPINDLE, "a spindle output"), (PIN_PUMP, "a charge pump"), (PIN_STEP_Z, "a Z step output"),
                                            (PIN_A, "an encoder A input"), (PIN_B, "an encoder B input"), (PIN_INDEX, "an index input"))
                       if pin[k] == 0xFF]
            assert not missing, "with four axes this controller has no terminal for " + ", ".join(missing)
            caps = board.info["caps"]
            assert caps & CAP_ENCODER and caps & CAP_SPINDLE_SYNC, "capabilities %02X: no encoder, or no screw cutting by it" % caps

        def stirs(a, b):
            """True if the count goes to and fro with only the one signal running."""
            c.io.update(spindle=1 if a else 0, rpm=signal.half if a else 0)
            c.io["flags"] = (c.io["flags"] | IO_CHARGE_PUMP) if b else (c.io["flags"] & ~IO_CHARGE_PUMP)
            c.send_io()
            c.pump(0.4)
            n = len(c.log)
            c.pump(0.8)
            return len(set(s["spindle_pos"] for s in c.log[n:])) > 1

        def t_counted():
            idle(c, stream_no)
            signal.on(RATES[0])
            starts = signal.settle()
            if not starts:
                a, b = stirs(True, False), stirs(False, True)
                signal.off()
                missing = [link[k] for k, there in (("A", a), ("B", b)) if not there]
                assert not missing, "nothing arrives: link " + " and ".join(missing)
                raise AssertionError("A and B both arrive, but %d lines a second are read as %d rpm, not %d" % (
                    signal.lines, c.status["rpm"], signal.rpm))
            s = c.pump(0.2)
            return "%.1f lines a second read as %d rpm%s" % (signal.lines, s["rpm"],
                                                              "" if starts == 1 else " (the charge pump started %d times)" % starts)

        def t_index():
            f = Follow(c)                       # (follows the count from rest)
            signal.off()
            c.pump(0.6)
            f.take()
            signal.on(RATES[0])
            assert signal.settle(), "the count does not run at %d rpm" % signal.rpm
            period = LINES * signal.n // 20
            n = int(10e6 / period)
            st = idle(c, stream_no)
            f.take()
            row, i0, z0 = len(f.rows) - 1, c.status["index_count"], c.status["pos"][2]
            c.stream(st, [([0, 0, 256, 0, 0, 0], period, SEG_SUBSTEP | (SEG_LAST if i == n - 1 else 0)) for i in range(n)], timeout=40.0)
            s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == n, 40.0, "Z to finish")
            c.pump(0.2)
            f.take()
            steps, got = s["pos"][2] - z0, c.status["index_count"] - i0
            assert steps == n, "Z made %d steps of %d" % (steps, n)
            assert got, "no index pulse arrives: link " + link["Z"]
            assert got == steps, "%d Z pulses, %d taken as index pulses" % (steps, got)
            assert not c.sflags_seen & SF_UNDERRUN, "the controller ran short of motion: the network"
            per, uneven, off, fine = index_place(f, f.pulses_from(row), LINES)
            allowed = 0.004 + 40e-6 * signal.lines / LINES          # a degree and a half, and 40 us at this speed
            assert abs(per - 1.0) <= 0.002, "an index pulse every %.4f turns of the count" % per
            assert uneven <= allowed and off <= allowed, "the index is found up to %.1f counts from where it was learnt" % (
                max(uneven, off) * 4 * LINES)
            return "%d Z pulses, each taken as an index pulse, all at one place in the count (to the %.1f counts the reports tell)" % (
                got, max(off, fine) * 4 * LINES)

        def t_rate(wanted):
            def fn():
                idle(c, stream_no)
                signal.off()
                c.pump(0.3)
                signal.on(wanted)
                starts = signal.settle()
                assert starts, "%d lines a second are read as %d rpm, not %d" % (signal.lines, c.status["rpm"], signal.rpm)
                gap, way, steps = count_by_clock(c, signal, stream_no)
                gap *= 4.0 * LINES / STEPS      # in counts
                counts = 4.0 * signal.lines * SECONDS
                # a count either way, and the steps themselves come within some 30 us of when they are due
                allowed = 1.0 + 30e-6 * 4.0 * signal.lines
                assert gap <= allowed, "counted %s, the count moved %.1f counts against the clock in %.1f million" % (
                    "upwards" if way > 0 else "downwards", gap, counts / 1e6)
                return "%d rpm, counted %s; %d steps of Z each found the count where it belongs, to %.1f counts, in %.1f million counts" % (
                    signal.rpm, "upwards" if way > 0 else "downwards", steps, max(gap, 0.0) if gap > 0 else -gap, counts / 1e6)
            return fn

        test("set up with four axes: terminals for the spindle output, charge pump, Z step and the encoder; screw cutting offered", t_set_up)
        print("--- links: %s; %s; %s" % (link["A"], link["B"], link["Z"]))
        if all(r[1] == "ok" for r in test.results):
            c.cfg_set(STATUS_MS, 1, 10)             # a status report every 10 ms
            c.pump(0.5)
            signal = Signal(c)
            print("--- as an encoder of %d lines with index pulse" % LINES)
            test("A and B, made by the controller, are counted", t_counted)
            if all(r[1] == "ok" for r in test.results):
                test("Z step pulses are index pulses, all at one place in the count", t_index)
                signal.off()
                board.write({SENSOR_MODE: NO_INDEX})
                c = board.c
                c.cfg_set(STATUS_MS, 1, 10)
                c.pump(0.5)
                signal = Signal(c)
                print("--- as an encoder without index pulse: nothing corrects the count")
                for wanted in RATES:
                    test("%d lines a second: no count lost or gained" % wanted, t_rate(wanted))
                signal.off()
                print("--- screw cutting by A and B")
                ebt.by_drill(board, test, Own(signal, RATES[0]), Follow(c),
                             argparse.Namespace(lines=LINES, wait=20, v=args.v, no_index=True), own=True)
    finally:
        try:
            if signal:
                signal.off()
        except Exception:
            pass
        try:
            board.write(was)
            print("--- settings as they were.  Take the links off again: with six axes two of those inputs are limit inputs")
        except Exception as e:
            print("\nTHE SETTINGS WERE NOT PUT BACK (%s).  They were:" % e)
            for k, v in was.items():
                print("    %-32s (0x%04X) %s" % (slt.NAME[k], k, v))
        if sim:                                     # leave the simulated machine as it was found
            sim(cmd="links", on=0)
            sim(cmd="set", lim_lo=[-4000] * 6, lim_hi=[200000] * 6, phys=[0] * 6)

    failed = [r for r in test.results if r[1] != "ok"]
    print("\n%d passed, %d failed" % (len(test.results) - len(failed), len(failed)))
    if args.expect:
        return 0 if any(args.expect in r[1] for r in failed) else 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
