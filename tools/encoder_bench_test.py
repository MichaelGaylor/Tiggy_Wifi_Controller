#!/usr/bin/env python3
"""
A spindle encoder on the bench: its inputs, its index pulse, and screw cutting by it.

The encoder need not be on a machine.  Wire it to the controller (below), then
turn it by hand, and afterwards with a cordless drill on its shaft:

    python tools/encoder_bench_test.py 192.168.1.50 --lines 600 --bench

The program says what to do next, and beeps each time it does (--say: it
speaks instead, on Windows).  It goes by what the controller sees, so there is
no hurry: it waits for you (--wait, 3 minutes unless told otherwise).

Part 1, by hand.  Turn the shaft slowly one way for a dozen turns, then back
for a few.
  - A and B are counted, and the count goes back when the shaft does.
  - The index pulse comes once a turn and always at the same place in the
    count: so the number of lines is the one given, and no count was lost.
    If the number is wrong the program says what the encoder looks like.

Part 2, with a drill.  Hold the encoder's body, the drill on its shaft: run it
steadily (300 rpm or more, not beyond what the encoder is made for), stop when
told, run it again when told.
  - The speed by the count is the speed by the index pulses: no counts are
    lost at speed.
  - Threading passes are stepped out on Z and every status report made
    meanwhile is held against the thread asked for, as in
    spindle_loopback_test.py: a pass lies where it should on the spindle, a
    second pass lies on the first, Z stops with the spindle when the drill
    stops in mid-pass, and a pass sent to a spindle at rest waits for it and
    begins at its index.
A drill in the hand never runs steadily, which is the point: Z has to follow
the spindle, not the clock.

Wiring.  The encoder needs the four-axis terminal layout, which this program
sets (and puts back at the end, with the sensor settings, unless --keep): on
the ESP32-S3-DevKitC with octal memory A is GPIO 16, B is GPIO 17, the index
pulse GPIO 21; the program names the terminals of the board it finds.  The
encoder's 0 V goes to the controller's GND.  THE INPUTS TAKE 3.3 V AND NO
MORE:
  - open-collector (NPN) outputs connect directly; for the drill, add a
    pull-up of 2.2 to 4.7 kilohms from each signal to 3.3 V (the chip's own
    pull-ups are too weak for more than hand speed);
  - push-pull, TTL or line-driver outputs of 5 V or more go through a divider
    each (5 V: 1 kilohm in series, 2 kilohms from the input to GND); of a line
    driver use A, B and Z and leave the inverted signals unconnected.

--bench says that no drive is connected to the Z terminals: the passes are
stepped out there, some 30000 steps and back.

    --no-index   the encoder has no index pulse (or it is not connected)
    --part hand  only part 1;  --part drill  only part 2
    --keep       leave the controller set up for the encoder afterwards
    --sim        the target is tools/firmware_sim.py: the program turns the
                 simulated encoder itself - a check of its own measuring
"""

import argparse
import subprocess
import sys
import time

import spindle_loopback_test as slt
import threading_test as tt
from protocol_conformance import Sim, ramp_move
from spindle_loopback_test import Board, Tests, fold, deg, idle, IDLE, RUN, SF_UNDERRUN, SF_WAIT_INDEX, SF_SYNC

STATUS_MS, PIN_A, PIN_B, PIN_INDEX, SENSOR_PPR, SENSOR_MODE, LAYOUT = 0x0103, 0x0520, 0x0521, 0x0522, 0x0523, 0x0524, 0x0601
slt.TYPE.update({LAYOUT: 0})
slt.NAME.update({LAYOUT: "terminal layout", SENSOR_PPR: "encoder lines"})
CAP_ENCODER, CAP_SPINDLE_SYNC = 0x10, 0x80
FOUR_AXES, WITH_INDEX, NO_INDEX = 1, 0, 2

USUAL = (100, 200, 250, 256, 300, 360, 400, 500, 512, 600, 720, 800, 1000, 1024, 1200, 1250, 1500, 1800,
         2000, 2048, 2500, 3600, 4096, 5000)       # lines encoders are made with
AT_REST = 2                     # rpm: an encoder standing on an edge may show a turn or two a minute
OUT, OFFSET, APART = 0.004, 0.004, 0.003        # turns: what a pass may be out by (as in spindle_loopback_test.py)


class Follow:
    """How far the encoder has turned, from the angle in every status report.

    The angle comes round once a turn; how many turns lie between two reports is known from
    how fast it was going at the report before (and, over a gap in the reports, from the
    speed reported).  Each report is given the result as 'turns', counted in the direction
    the encoder last ran in."""

    def __init__(self, c):
        self.c, self.n = c, len(c.log)
        self.turns, self.rate, self.prev, self.sign = 0.0, 0.0, None, 1
        self.rows = []          # (controller's clock in ms, turns, rpm, angle as reported): one per report
        self.pulses = []        # (number of the first row after it, how many): index pulses

    def take(self):
        log = self.c.log
        while self.n < len(log):
            s = log[self.n]
            self.n += 1
            p = self.prev
            if p is not None:
                dt = s["up"] - p["up"]
                if dt <= 0:                     # the same report twice (or the controller restarted)
                    s["turns"] = self.sign * self.turns
                    if dt < 0:
                        self.prev, self.rate = s, 0.0
                    continue
                raw = ((s["spindle_pos"] - p["spindle_pos"]) & 0xFFFF) / 65536.0
                expect = self.rate * dt
                if dt > 25:
                    expect = (1 if self.rate >= 0 else -1) * (p["rpm"] + s["rpm"]) / 120000.0 * dt
                d = expect + fold(raw - expect)
                self.turns += d
                self.rate = d / dt
                if abs(self.rate) > 0.0005:     # half a turn a second
                    self.sign = 1 if self.rate > 0 else -1
                k = (s["index_count"] - p["index_count"]) & 0xFFFFFFFF
                if 0 < k < 100000:
                    self.pulses.append((len(self.rows), k))
            self.prev = s
            s["turns"] = self.sign * self.turns
            self.rows.append((s["up"], self.turns, s["rpm"], s["spindle_pos"]))

    def pulses_from(self, row):
        return [p for p in self.pulses if p[0] > row]


class Bench:
    """The person at the bench: told what to do next - or, with the simulator, the program itself."""

    def __init__(self, sim, spec, say):
        self.sim, self.spec, self.say = sim, spec, say
        self.way = -1 if spec.get("reverse") else 1

    def cue(self, what, text):
        print(">>> " + text, flush=True)
        try:
            if self.say and sys.platform == "win32":
                subprocess.Popen(["powershell", "-NoProfile", "-Command",
                                  "Add-Type -AssemblyName System.Speech; "
                                  "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('%s')" % text.replace("'", "")],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            elif sys.platform == "win32":
                import winsound
                winsound.MessageBeep()
            else:
                sys.stdout.write("\a")
                sys.stdout.flush()
        except Exception:                       # no sound is no reason to stop
            pass
        if self.sim and not self.spec.get("idle"):
            hand = dict(accel=300.0, ripple=0.3, ripple_hz=0.7)
            drill = dict(accel=4000.0, ripple=0.05, ripple_hz=3.0)
            act = {"forwards": dict(rpm=55.0 * self.way, **hand), "backwards": dict(rpm=-45.0 * self.way, **hand),
                   "rest": dict(rpm=0.0, **hand), "drill": dict(rpm=1130.0 * self.way, **drill),
                   "stop": dict(rpm=0.0, **drill), "done": dict(rpm=0.0, **drill)}[what]
            self.sim(cmd="spindle", follow=0, **act)


def rest(c, f, seconds=0.6):
    """True when the encoder has stood still for 'seconds'."""
    c.pump(seconds)
    f.take()
    since = f.rows[-1][0] - 1000 * seconds
    rows = [r for r in f.rows[-200:] if r[0] >= since]
    return len(rows) > 1 and all(r[2] <= AT_REST for r in rows) and abs(rows[-1][1] - rows[0][1]) < 0.01


def stretch(c, f, wait, turns, pulses, sense=0):
    """Follow the encoder from when it begins to turn - the way 'sense' says (1 or -1 as it
    counts), if given - until it has made 'turns' turns and 'pulses' index pulses, or has
    stood still again for a while, or nothing happens for 'wait' seconds.
    Returns the row it began at, how far it turned (signed), its pulses, and how often the
    angle reported changed at all."""
    f.take()
    row, t0, began, changes, last = len(f.rows) - 1, f.turns, None, 0, f.rows[-1][3] if f.rows else 0
    moved_at = end = time.perf_counter() + wait
    while True:
        c.pump(0.05)
        n = len(f.rows)
        f.take()
        for r in f.rows[n:]:
            if r[3] != last:
                changes, last = changes + 1, r[3]
        now, moved = time.perf_counter(), f.turns - t0
        if began is None:
            if sense and moved * sense < 0:             # still going the other way
                row, t0 = len(f.rows) - 1, f.turns
            elif abs(moved) > 0.25:
                began, moved_at, mark, end = now, now, f.turns, now + 150.0
        else:
            if abs(f.turns - mark) > 0.02:
                moved_at, mark = now, f.turns
            got = sum(k for _r, k in f.pulses_from(row))
            if abs(moved) >= turns and got >= pulses:
                break
            if pulses and got == 0 and abs(moved) >= max(6.0, turns):       # no index pulse is coming
                break
            if now - moved_at > 4.0:                    # put down
                break
        if now > end:
            break
    return dict(row=row, moved=f.turns - t0, pulses=f.pulses_from(row), changes=changes)


def index_place(f, pulses, lines):
    """What the index pulses of a stretch say (the first two left out: the controller learns
    from them where the index is, and a reversal makes it learn again).
    Returns (turns of the count per index pulse, how uneven that was, how far the index is
    from where the controller has it, how closely the reports tell).

    The first report after a pulse was made up to one report's turning past the index: each
    such report says the index lies between two places, and all of them together say it
    more closely than any one."""
    use = pulses[2:]
    assert len(use) >= 4, "only %d index pulses to go by" % sum(k for _r, k in pulses)
    at = [f.rows[r][1] for r, _k in use]
    step = [abs(f.rows[r][1] - f.rows[r - 1][1]) for r, _k in use]      # how far it turns between two reports
    n = sum(k for _r, k in use[1:])
    per = abs(at[-1] - at[0]) / n
    uneven = max(max(0.0, abs(abs(at[i + 1] - at[i]) / use[i + 1][1] - per) - step[i] - step[i + 1]) for i in range(len(use) - 1))
    way = 1 if at[-1] > at[0] else -1
    past = [fold(f.rows[r][3] / 65536.0) * way for r, _k in use]
    lo, hi = max(p - st for p, st in zip(past, step)), min(past)        # the index is between these
    uneven = max(uneven, lo - hi)
    return per, uneven, max(0.0, lo, -hi), max(hi, -lo)


def by_hand(board, test, bench, f, args):
    c, lines = board.c, args.lines
    count = 1.0 / (4 * lines)
    pins = "GPIO %d and %d" % (c.cfg_get(PIN_A), c.cfg_get(PIN_B))
    found = {}

    def t_counted():
        assert rest(c, f, 1.0), "the encoder is turning: leave it at rest until told"
        bench.cue("forwards", "Turn the encoder by hand, slowly, about a turn a second, one way for twelve turns")
        a = stretch(c, f, args.wait, 10.0, 0 if args.no_index else 10)
        found["a"] = a
        if abs(a["moved"]) < 0.25:
            if a["changes"] >= 20:
                raise AssertionError("the count goes to and fro and gets nowhere: only one of A and B arrives (%s)" % pins)
            raise AssertionError("nothing is counted: A and B do not arrive (%s) - or the encoder has no supply, "
                                 "or no common 0 V with the controller" % pins)
        assert abs(a["moved"]) >= 3.0, "turned only %.1f turns of the count" % abs(a["moved"])
        return "%.1f turns counted, %s" % (abs(a["moved"]), "upwards" if a["moved"] > 0 else
                                           "downwards: for a count that goes up this way round, change A and B over")

    def t_index():
        a = found["a"]
        got = sum(k for _r, k in a["pulses"])
        assert got, "no index pulse in %.1f turns: it does not arrive on GPIO %d (an encoder without one: --no-index)" % (
            abs(a["moved"]), c.cfg_get(PIN_INDEX))
        per, uneven, off, fine = index_place(f, a["pulses"], lines)
        if abs(per - 1.0) > 0.006:
            if per < 0.3:
                raise AssertionError("an index pulse every %.2f turns of the count: the encoder has far fewer than %d lines" % (per, lines))
            seen = lines * per
            likely = min(USUAL, key=lambda n: abs(n - seen))
            raise AssertionError("an index pulse every %.3f turns of the count: the encoder has about %d lines%s, not %d" % (
                per, int(round(seen)), " (%d, most likely)" % likely if abs(likely - seen) <= 0.005 * seen else "", lines))
        assert uneven <= 0.01, "the index pulses come unevenly, up to %s out: noise on the index wire, or counts lost" % deg(uneven)
        assert off <= max(0.004, 3 * count), "the index is read up to %s from where it was learnt: counts are lost" % deg(off)
        return "%d pulses, one a turn, all at one place in the count (to the %.1f deg the reports tell): %d lines" % (
            got, 360 * max(off, fine), lines)

    def t_back():
        a = found["a"]
        sense = 1 if a["moved"] > 0 else -1
        bench.cue("backwards", "Now turn it the other way, eight turns")
        b = stretch(c, f, args.wait, 7.0, 0 if args.no_index else 7, -sense)
        bench.cue("rest", "That will do")
        assert b["moved"] * sense < -3.0, "turned back only %.1f turns of the count" % abs(b["moved"])
        note = "%.1f turns counted back" % abs(b["moved"])
        if not args.no_index:
            got = sum(k for _r, k in b["pulses"])
            assert abs(got - abs(b["moved"])) <= 1.5, "%d index pulses in %.1f turns back" % (got, abs(b["moved"]))
            _per, uneven, off, fine = index_place(f, b["pulses"], lines)
            assert uneven <= 0.01, "going back the index pulses come unevenly, up to %s out" % deg(uneven)
            assert off <= max(0.004, 3 * count), "going back the index is read up to %s from where it was learnt" % deg(off)
            note += ", %d index pulses, all at one place (to %.1f deg)" % (got, 360 * max(off, fine))
        return note

    test("turned by hand: A and B are counted", t_counted)
    if "a" in found and abs(found["a"]["moved"]) >= 3.0:
        if args.no_index:
            print("      (no index pulse: nothing says whether %d lines is right - %.1f turns were counted: as many as you made?)" % (
                lines, abs(found["a"]["moved"])))
        else:
            test("index pulse once a turn, always at the same place", t_index)
        if all(r[1] == "ok" for r in test.results):
            test("turned the other way: counted back", t_back)
    return all(r[1] == "ok" for r in test.results)


def by_drill(board, test, bench, f, args):
    c, info, lines = board.c, board.info, args.lines
    stream_no = [150]
    plan = {}

    def running(seconds):
        f.take()
        since = f.rows[-1][0] - 1000 * seconds
        return [r for r in f.rows[-2000:] if r[0] >= since]

    def spin_up(what):
        """Wait for the drill to run steadily; returns its speed in turns a second."""
        end = time.perf_counter() + args.wait
        while time.perf_counter() < end:
            c.pump(0.2)
            rpm = [r[2] for r in running(1.5)]
            if len(rpm) > 50 and min(rpm) >= 200 and max(rpm) - min(rpm) <= 0.25 * max(rpm):
                return sum(rpm) / len(rpm) / 60.0
        raise AssertionError("%s: the drill is not running steadily at 200 rpm or more (%d rpm)" % (what, c.status["rpm"]))

    def make_pass(hz, seconds=5.0):
        spr = next(n for n in (197, 149, 101, 61, 37, 19) if n * hz * 1.5 <= 0.6 * info["max_rate"] or n == 19)
        tt.ACCEL = spr * hz * 2.0                       # half a second to the pitch speed
        return tt.Pass(60.0 * hz, length=spr * int(max(40, seconds * hz)), steps_per_rev=spr), spr

    def back(p):
        return ramp_move([0, 0, p.length, 0, 0, 0], max(4.5, p.length / (0.5 * info["max_rate"])))

    def lay(p0, rep, p, label, segs=None):
        f.take()                                        # (gives every report its 'turns')
        return slt.pass_on_spindle(p0, rep, p, label, segs, args.v)

    def good(m, what=""):
        assert m["out"] <= OUT, "%sa report %s outside its band" % (what, deg(m["out"]))
        assert abs(m["offset"]) <= OFFSET, "%soffset %s (%.0f us)" % (what, deg(m["offset"]), m["us"])

    def t_speed():
        if not any(rest(c, f) for _n in range(5)):      # (it may only just have been put down)
            bench.cue("stop", "Stop the drill")
            end = time.perf_counter() + args.wait
            while not rest(c, f):
                assert time.perf_counter() < end, "the encoder does not come to rest"
        bench.cue("drill", "Run the drill, steadily, and keep it running")
        spin_up("waited %d s" % args.wait)
        c.pump(0.5)
        f.take()
        row = len(f.rows) - 1
        c.pump(6.0)
        f.take()
        rows = f.rows[row:]
        rpm = [r[2] for r in rows]
        hz = sum(rpm) / len(rpm) / 60.0
        plan["hz"] = hz
        note = "%d to %d rpm, A and B at %d lines a second" % (min(rpm), max(rpm), int(lines * max(rpm) / 60.0))
        if args.no_index:
            return note
        pulses = f.pulses_from(row)
        assert len(pulses) >= 10, "%d index pulses in six seconds at %d rpm" % (len(pulses), 60 * hz)
        first, last = pulses[0][0], pulses[-1][0]
        turns = sum(k for _r, k in pulses[1:])
        by_index = turns * 1000.0 / (f.rows[last][0] - f.rows[first][0])
        between = f.rows[first:last + 1]
        by_count = sum(0.5 * (a[2] + b[2]) * (b[0] - a[0]) for a, b in zip(between, between[1:])) / (between[-1][0] - between[0][0]) / 60.0
        assert abs(by_count / by_index - 1.0) <= 0.01, \
            "%.2f turns a second by the count, %.2f by the index pulses: counts are %s" % (
                by_count, by_index, "lost" if by_count < by_index else "added (noise)")
        _per, uneven, off, fine = index_place(f, pulses, lines)
        assert uneven <= 0.01, "at speed the index pulses come unevenly, up to %s out: noise, or counts lost" % deg(uneven)
        assert off <= 0.01, "at speed the index is read up to %s from where it was learnt" % deg(off)
        return note + "; %.2f turns a second by the count, %.2f by the index; %d index pulses, all at one place (to %.1f deg)" % (
            by_count, by_index, len(pulses), 360 * max(off, fine))

    def t_pass():
        st = idle(c, stream_no)
        hz = plan["hz"] = spin_up("pass")
        p, spr = make_pass(hz)
        p0, rep = slt.play_segments(c, st, p.segs + back(p))
        slt.ended_clean(c, p0)
        m = lay(p0, rep, p, "pass")
        good(m)
        rpms = [r[4] for r in m["res"]]
        return "%d steps a turn at %d to %d rpm: %d reports, the worst %s outside its band; offset %s (%.0f us)" % (
            spr, min(rpms), max(rpms), m["n"], deg(m["out"]), deg(m["offset"]), m["us"])

    def t_two():
        st = idle(c, stream_no)
        hz = spin_up("two passes")
        plans = [make_pass(hz)[0], make_pass(hz)[0]]
        segs = plans[0].segs + back(plans[0])
        n1 = len(segs)
        segs += plans[1].segs + back(plans[1])
        p0, rep = slt.play_segments(c, st, segs, timeout=90.0)
        slt.ended_clean(c, p0)
        a = lay(p0, rep, plans[0], "first pass", (0, len(plans[0].segs)))
        b = lay(p0, rep, plans[1], "second pass", (n1, n1 + len(plans[1].segs)))
        good(a, "first pass: ")
        good(b, "second pass: ")
        apart = b["offset"] - a["offset"]
        assert abs(apart) <= APART, "the second pass lies %s from the first" % deg(apart)
        return "at %d and %d rpm: the second lies %s from the first" % (a["rpm"], b["rpm"], deg(apart))

    def t_stops():
        st = idle(c, stream_no)
        hz = spin_up("pass to be stopped")
        p, spr = make_pass(hz, seconds=45.0)
        p0 = c.pump(0.05)["pos"]
        told = [False]

        def watch(s):
            if not told[0] and p0[2] - s["pos"][2] > 12 * spr:
                told[0] = True
                bench.cue("stop", "Stop the drill")
            return told[0] and s["rpm"] <= AT_REST

        c.stream(st, p.segs, until=watch, timeout=60.0)
        end = time.perf_counter() + 60.0
        while not watch(c.pump(0.02)):
            assert c.status["state"] == RUN and time.perf_counter() < end, "the drill was not stopped while the pass was played"
        c.pump(0.5)
        a = c.pump(0.05)
        b = c.pump(1.0)
        assert a["pos"][2] == b["pos"][2], "Z moved %d steps in a second with the spindle at rest" % (b["pos"][2] - a["pos"][2])
        into = p0[2] - b["pos"][2]
        assert 0 < into < p.length, "Z is %d steps into a pass of %d" % (into, p.length)
        assert b["state"] == RUN and b["sflags"] & SF_SYNC and b["rpm"] <= AT_REST, \
            "state %d, flags %02X, %d rpm with the spindle stopped in a pass" % (b["state"], b["sflags"], b["rpm"])
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0])
        s = c.pump(0.3)
        assert s["state"] == IDLE and s["pos"][2] == b["pos"][2], "after the flush: state %d, Z %d" % (s["state"], s["pos"][2])
        c.stream(stream_no[0], ramp_move([0, 0, into, 0, 0, 0], max(3.0, into / (0.4 * info["max_rate"]))))
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 20.0, "the move back")
        assert s["pos"] == p0, "back at %s, began at %s" % (s["pos"][:3], p0[:3])
        return "Z stopped %d steps into the pass and stayed; the pass given up, Z taken back" % into

    def t_waits():
        st = idle(c, stream_no)
        end = time.perf_counter() + args.wait
        while not rest(c, f):
            assert time.perf_counter() < end, "the encoder does not come to rest"
        p, _spr = make_pass(plan["hz"])
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs[:100])
        c.wait(lambda s: s["sflags"] & SF_WAIT_INDEX, 3.0, "the wait for the index")
        s = c.pump(1.5)
        assert s["pos"] == p0, "moved to %s before the spindle turned" % s["pos"][:3]
        assert s["state"] == RUN and s["sflags"] & SF_WAIT_INDEX, "state %d, flags %02X while waiting" % (s["state"], s["sflags"])
        assert not (c.sflags_seen & SF_UNDERRUN), "reported an underrun while waiting"
        start = len(c.log)
        bench.cue("drill", "Run the drill again")
        c.wait(lambda s: s["pos"][2] != p0[2], args.wait, "the drill to turn")
        rest_of = p.segs[100:] + back(p)
        c.stream(st, rest_of, first=100, timeout=60.0)
        total = 100 + len(rest_of)
        c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and s["next_seg"] == total, 60.0, "the pass to finish")
        slt.ended_clean(c, p0)
        m = lay(p0, c.log[start:], p, "from rest")
        good(m)
        rpms = [r[4] for r in m["res"]]
        return "waited 1.5 s without a step; then, with the drill running up from %d to %d rpm, the worst report %s outside its band, offset %s" % (
            min(rpms), max(rpms), deg(m["out"]), deg(m["offset"]))

    try:
        test("spun by the drill: speed read" if args.no_index else "spun by the drill: no counts lost", t_speed)
        if "hz" in plan:
            test("threading pass played by the encoder", t_pass)
            test("a second pass lies on the first", t_two)
            test("drill stopped in mid-pass: Z stops with it, and the pass can be given up", t_stops)
            test("a pass waits for the spindle" if args.no_index else "a pass waits for the spindle and begins at its index", t_waits)
    finally:
        bench.cue("done", "Stop the drill. That is all")
        try:
            idle(c, stream_no)
        except Exception as e:
            print("could not bring the controller to rest: %s" % e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip")
    ap.add_argument("--lines", type=int, required=True, help="lines of the encoder (pulses per revolution of one channel)")
    ap.add_argument("--no-index", action="store_true", help="the encoder has no index pulse")
    ap.add_argument("--bench", action="store_true", help="yes: no drive is connected to the Z terminals")
    ap.add_argument("--part", choices=("hand", "drill", "both"), default="both")
    ap.add_argument("--wait", type=int, default=180, help="seconds to wait for you each time (default 180)")
    ap.add_argument("--keep", action="store_true", help="leave the controller set up for the encoder")
    ap.add_argument("--say", action="store_true", help="speak what to do next (Windows)")
    ap.add_argument("-k", default="", help="only run tests whose name contains this (several: separate with commas)")
    ap.add_argument("-v", action="store_true", help="print the measurements of every pass")
    ap.add_argument("--sim", action="store_true", help="the target is tools/firmware_sim.py")
    ap.add_argument("--sim-encoder", default="", help=argparse.SUPPRESS)       # lines=1024,index=0,b_open=1,reverse=1,idle=1
    ap.add_argument("--expect", default="", help=argparse.SUPPRESS)            # succeed only if a test fails saying this
    args = ap.parse_args()
    assert 16 <= args.lines <= 16000, "an encoder of 16 to 16000 lines"
    if not (args.bench or args.sim):
        print(__doc__)
        print("Not started: say --bench when no drive is connected to the Z terminals.")
        return 2

    test = Tests(args.k)
    sim, spec = None, {}
    if args.sim:
        sim = Sim()
        spec = {k: int(v) for k, v in (item.split("=") for item in args.sim_encoder.split(",") if item)}
        sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        sim(cmd="spindle", follow=0, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
        sim(cmd="fit", lines=spec.get("lines", args.lines), index=spec.get("index", 0 if args.no_index else 1),
            b_open=spec.get("b_open", 0))
    bench = Bench(sim, spec, args.say)

    board = Board(args.ip)
    keys = (LAYOUT, SENSOR_MODE, SENSOR_PPR, STATUS_MS)
    was = board.read(keys)
    try:
        board.write({LAYOUT: FOUR_AXES, SENSOR_MODE: NO_INDEX if args.no_index else WITH_INDEX, SENSOR_PPR: args.lines})
        c = board.c
        pin = {k: c.cfg_get(k) for k in (PIN_A, PIN_B, PIN_INDEX)}

        def t_set_up():
            assert pin[PIN_A] != 0xFF and pin[PIN_B] != 0xFF, "this controller has no terminals for an encoder's A and B"
            assert args.no_index or pin[PIN_INDEX] != 0xFF, "this controller has no terminal for the index pulse"
            caps = board.info["caps"]
            assert caps & CAP_ENCODER and caps & CAP_SPINDLE_SYNC, "capabilities %02X: no encoder, or no screw cutting by it" % caps
            return "A on GPIO %d, B on GPIO %d%s" % (pin[PIN_A], pin[PIN_B],
                                                     "" if args.no_index else ", index pulse on GPIO %d" % pin[PIN_INDEX])

        test("set up for an encoder of %d lines: it has its terminals, and screw cutting is offered" % args.lines, t_set_up)
        if all(r[1] == "ok" for r in test.results):
            c.cfg_set(STATUS_MS, 1, 10)             # a status report every 10 ms
            c.pump(0.5)
            f = Follow(c)
            ok = True
            if args.part in ("hand", "both"):
                print("--- by hand")
                ok = by_hand(board, test, bench, f, args)
            if args.part in ("drill", "both"):
                if ok:
                    print("--- with a drill")
                    by_drill(board, test, bench, f, args)
                else:
                    print("--- with a drill: left out until the encoder is read rightly by hand")
    finally:
        try:
            if sim:
                sim(cmd="spindle", follow=0, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
            if args.keep:
                board.write({STATUS_MS: was[STATUS_MS]})
                print("--- the controller stays set up for the encoder (four axes, %d lines)" % args.lines)
            else:
                board.write(was)
                print("--- settings as they were")
        except Exception as e:
            print("\nTHE SETTINGS WERE NOT PUT BACK (%s).  They were:" % e)
            for k, v in was.items():
                print("    %-32s (0x%04X) %s" % (slt.NAME[k], k, v))
        if sim:                                     # leave the simulated machine as it was found
            sim(cmd="fit", lines=0)
            sim(cmd="spindle", follow=1, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
            sim(cmd="set", lim_lo=[-4000] * 6, lim_hi=[200000] * 6, phys=[0] * 6)

    failed = [r for r in test.results if r[1] != "ok"]
    print("\n%d passed, %d failed" % (len(test.results) - len(failed), len(failed)))
    if args.expect:
        return 0 if any(args.expect in r[1] for r in failed) else 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
