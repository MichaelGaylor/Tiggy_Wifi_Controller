#!/usr/bin/env python3
"""
Screw-cutting tests: the controller playing a move in step with its spindle.

Runs against tools/firmware_sim.py (the firmware's own motion code on a PC,
with a simulated spindle and spindle sensor):

    python tools/firmware_sim.py --quiet &
    python tools/threading_test.py

Each test streams threading passes the way a host does: a move along Z
planned for some spindle speed, marked to be played by the spindle and to
begin at its index.  The simulator records every step of Z together with
the true angle of the spindle at that instant - which is the thread that
would have been cut - and the test compares that with the thread asked for:
every step must fall at the spindle angle the plan puts it at, counted from
an index position, whatever the spindle's speed was doing.

The figure printed with each result is the worst error found, in degrees of
spindle rotation (0.36 degrees is a thousandth of the pitch).
"""

import argparse
import bisect
import math
import struct
import sys
import threading
import time

from protocol_conformance import Client, Sim, ramp_move, SEG_SUBSTEP, SEG_SYNC, SEG_INDEX, SEG_LAST

STEPS_PER_REV = 600         # Z steps per turn of the spindle: a 1.5 mm pitch at 400 steps/mm
LENGTH = 6000               # steps: ten turns of thread
ACCEL = 60000.0             # steps/s^2
SLICE_US = 2000
INDEX_AT = 0.37             # where in the revolution the simulator's index pulse is
RETRACT = 200               # X steps

IDLE, RUN, HOLD, ALARM = 0, 1, 2, 6
SF_UNDERRUN, SF_WAIT_INDEX, SF_SYNC, SF_SYNC_LOST = 0x02, 0x08, 0x10, 0x20
CAP_SPINDLE_SYNC = 0x80


class Pass:
    """One threading pass as a host plans it for a spindle speed of plan_rpm:
    accelerate, travel at the pitch, decelerate; cut into slices of sub-steps."""

    def __init__(self, plan_rpm, length=LENGTH, direction=-1, at_index=True, sync=True, steps_per_rev=STEPS_PER_REV):
        self.plan_rpm, self.length, self.direction = plan_rpm, length, direction
        v = steps_per_rev * plan_rpm / 60.0
        ta, da = v / ACCEL, v * v / (2 * ACCEL)
        assert 2 * da < length
        total = 2 * ta + (length - 2 * da) / v
        self.duration = total

        def place(t):
            if t <= ta:
                return 0.5 * ACCEL * t * t
            if t <= total - ta:
                return da + v * (t - ta)
            return length - 0.5 * ACCEL * max(0.0, total - t) ** 2

        n = int(math.ceil(total * 1e6 / SLICE_US))
        self.q = [int(round(256 * place(i * SLICE_US / 1e6))) for i in range(n)] + [256 * length]
        ref = int(round(plan_rpm * 1000))
        self.segs = []
        for i in range(n):
            flags = SEG_SUBSTEP | (SEG_SYNC if sync else 0) | (SEG_INDEX if sync and at_index and i == 0 else 0)
            seg = ([0, 0, direction * (self.q[i + 1] - self.q[i]), 0, 0, 0], SLICE_US, flags)
            self.segs.append(seg + (ref,) if sync else seg)

    def turns_at_step(self, c):
        """How far the spindle has turned since the run began when step number c (from 1) falls due."""
        # the controller's accumulator starts half way: the step comes when the travel reaches c - 1/2
        target = 256 * c - (127 if self.direction < 0 else 128)
        i = bisect.bisect_left(self.q, target) - 1
        t = (i + (target - self.q[i]) / float(self.q[i + 1] - self.q[i])) * SLICE_US / 1e6
        return t * self.plan_rpm / 60.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="", help="only run tests whose name contains this (several: separate with commas)")
    ap.add_argument("-v", action="store_true", help="print the measurements of every pass")
    args = ap.parse_args()

    sim = Sim()
    c = Client("127.0.0.1")
    results = []
    stream_no = [100]

    def test(name, fn):
        if args.k and not any(word in name for word in args.k.split(",")):
            return
        try:
            note = fn()
            results.append((name, "ok"))
            print("ok    %s%s" % (name, ("   [%s]" % note) if note else ""))
        except Exception as e:                      # one failed test must not hide the others
            results.append((name, "FAIL: %s" % e))
            print("FAIL  %s: %s" % (name, e))

    def get():
        return sim(cmd="get")

    def spin(rpm):
        """Bring the spindle to rpm and wait until the controller has followed."""
        sim(cmd="spindle", follow=0, rpm=rpm, accel=3000.0, ripple=0.0, ripple_hz=0.0)
        end = time.perf_counter() + 5.0
        while time.perf_counter() < end:
            c.pump(0.05)
            g = get()["spindle"]
            if abs(g["rpm"] - rpm) < 0.01 and abs(g["tracked_rpm"] - abs(rpm)) <= max(0.5, abs(rpm) * 0.002):
                return
        raise AssertionError("spindle speed not tracked: true %.2f, tracked %.2f" % (g["rpm"], g["tracked_rpm"]))

    def ready(spindle_rpm=300.0, sensor=("quad", 1024, 1), fresh=False):
        """Controller idle, the sensor asked for fitted, the spindle turning at spindle_rpm.
        The spindle has turned before, so the controller knows where its index is - unless
        'fresh', which is a controller just switched on beside a spindle at rest."""
        sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        sim(cmd="encoder", mode=sensor[0], ppr=sensor[1], index=sensor[2])
        sim(cmd="spindle", follow=0, rpm=0.0 if fresh else spindle_rpm, accel=3000.0, ripple=0.0, ripple_hz=0.0)
        for n in range(40):
            s = c.pump(0.1)
            if s and s["state"] in (6, 7):
                c.send(7)
            elif s and s["state"] in (RUN, HOLD):   # left over from a test that failed: drop it
                sim(cmd="resume")
                stream_no[0] = (stream_no[0] + 1) & 0xFF
                c.flush(stream_no[0])
            elif s and s["state"] == IDLE and not get()["busy"]:
                break
        else:
            raise AssertionError("controller did not become idle: %s" % c.status)
        if fresh:
            assert spindle_rpm == 0.0
            spin(0.0)
            sim(cmd="encoder", mode=sensor[0], ppr=sensor[1], index=sensor[2])
        elif sensor[1]:
            spin(spindle_rpm if spindle_rpm else 300.0)
            c.pump(0.8)                         # a few turns: the index has been seen more than once
            if not spindle_rpm:
                spin(0.0)
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0])
        c.sflags_seen = 0
        return stream_no[0]

    def between_passes():
        """Clear of the work, back to the start, in again - by the clock, as a host sends them."""
        return (ramp_move([RETRACT, 0, 0, 0, 0, 0], 0.1) + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7) +
                ramp_move([-RETRACT, 0, 0, 0, 0, 0], 0.1))

    def run(st, segs, events=(), timeout=40.0):
        """Stream segs and wait for the end.  events: (seconds after the start, function) run meanwhile,
        each given an automation connection of its own."""
        failures = []

        def later(fn):
            try:
                fn(Sim())
            except Exception as e:
                failures.append(e)

        sim(cmd="trace", axis=2)
        p0 = c.pump(0.05)["pos"]
        timers = [threading.Timer(delay, later, args=(fn,)) for delay, fn in events]
        for t in timers:
            t.start()
        try:
            c.stream(st, segs, timeout=timeout)
            s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, timeout, "the moves to finish")
        finally:
            for t in timers:
                t.join()
        if failures:
            raise failures[0]
        return p0, s

    def cuts(direction=-1):
        """The recorded steps of Z, split into the passes made in 'direction': lists of (time, turns)."""
        tr = sim(cmd="trace_read")
        passes, prev, cur = [], None, None
        for t, turns, pos in zip(tr["t"], tr["turns"], tr["pos"]):
            d = 0 if prev is None else (1 if pos > prev else -1)
            if prev is None:
                d = direction                       # (the first step recorded begins the first pass)
            prev = pos
            if d == direction:
                if cur is None:
                    cur = []
                    passes.append(cur)
                cur.append((t, turns))
            else:
                cur = None
        return passes

    def measure(steps, p, spindle_dir=1, origin=None):
        """Compare a recorded pass with its plan.  Returns (where the run began, worst error in turns)."""
        assert len(steps) == p.length, "pass made %d steps, planned %d" % (len(steps), p.length)
        if origin is None:
            start = steps[0][1] - spindle_dir * p.turns_at_step(1)
            origin = INDEX_AT + round(start - INDEX_AT)
        worst = 0.0
        for n, (_t, turns) in enumerate(steps, 1):
            err = spindle_dir * (turns - origin) - p.turns_at_step(n)
            if abs(err) > abs(worst):
                worst = err
        return origin, worst

    def degrees(turns):
        return "%.3f deg" % (abs(turns) * 360.0)

    def approach(steps=30000, rate=30000.0, ramp=0.3):
        """A move of X that is still at full speed when it ends: whatever stops it there
        is still slowing it down when the segment after it is reached."""
        segs, done, t, slice_s = [], 0.0, 0.0, 0.005
        while done < steps:
            v = rate * min(1.0, (t + slice_s / 2) / ramp)
            d = min(v * slice_s, steps - done)
            segs.append(([int(round(256 * (done + d))) - int(round(256 * done)), 0, 0, 0, 0, 0], 5000, SEG_SUBSTEP))
            done += d
            t += slice_s
        return segs

    def new_stream():
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        return stream_no[0]

    def broken_off(p0, what):
        """After a pass was broken into: stopped part of the way, for good, and said so."""
        s = c.wait(lambda s: s["sflags"] & SF_SYNC_LOST, 3.0, "the controller to report the pass abandoned (%s)" % what)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and not (s["flags"] & 0x04), 2.0, "standstill")
        at = s["pos"]
        assert 0 < p0[2] - at[2] < LENGTH, "%s: Z is %d steps into a pass of %d" % (what, p0[2] - at[2], LENGTH)
        assert not (s["sflags"] & (SF_SYNC | SF_WAIT_INDEX)), "spindle flags still set: %02X" % s["sflags"]
        sim(cmd="resume")
        s = c.pump(0.6)
        assert s["pos"] == at and s["state"] == IDLE, "%s: moved on to %s afterwards (state %d)" % (what, s["pos"][:3], s["state"])
        # the host takes it from where the controller stopped
        st = new_stream()
        c.flush(st)
        s = c.pump(0.1)
        assert not (s["sflags"] & SF_SYNC_LOST), "still reported after the flush: %02X" % s["sflags"]
        c.stream(st, ramp_move([p0[0] - at[0], 0, p0[2] - at[2], 0, 0, 0], 0.6))
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 3.0, "the move back")
        assert s["pos"] == p0, "back at %s, began at %s" % (s["pos"][:3], p0[:3])
        return p0[2] - at[2]

    def check_passes(plans, tolerance, spindle_dir=1):
        recorded = cuts()
        assert len(recorded) == len(plans), "%d passes recorded, %d sent" % (len(recorded), len(plans))
        assert not (c.sflags_seen & SF_UNDERRUN), "the controller ran short of motion during the test (underrun reported)"
        worst = 0.0
        for n, (steps, p) in enumerate(zip(recorded, plans)):
            origin, err = measure(steps, p, spindle_dir)
            if args.v or abs(err) > tolerance:
                print("      pass %d: began at %.4f turns, worst error %s" % (n + 1, origin, degrees(err)))
                along = ["%+.2f" % (360.0 * (spindle_dir * (steps[i][1] - origin) - p.turns_at_step(i + 1)))
                         for i in range(0, len(steps), len(steps) // 12)]
                print("              error along it, degrees: %s" % " ".join(along))
            assert abs(err) <= tolerance, "pass %d is out by %s (allowed %s)" % (n + 1, degrees(err), degrees(tolerance))
            worst = max(worst, abs(err))
        return worst

    # ----------------------------------------------------------------- tests
    def t_tracking():
        ready(300.0)
        info = c.handshake()
        assert info["caps"] & CAP_SPINDLE_SYNC, "capabilities %02X: spindle synchronisation not advertised" % info["caps"]
        s = c.pump(0.3)
        assert abs(s["rpm"] - 300) <= 1, "reports %d rpm at 300" % s["rpm"]
        n0, t0 = s["index_count"], time.perf_counter()
        s = c.pump(2.0)
        turns = (time.perf_counter() - t0) * 5.0
        assert abs((s["index_count"] - n0) - turns) <= 1.5, "counted %d index pulses in %.1f turns" % (s["index_count"] - n0, turns)
        sim(cmd="spindle", rpm=1450.0, accel=2000.0)
        c.pump(1.5)
        s = c.pump(0.3)
        assert abs(s["rpm"] - 1450) <= 2, "reports %d rpm at 1450" % s["rpm"]
        sim(cmd="spindle", rpm=0.0, accel=3000.0)
        c.pump(1.2)
        s = c.pump(0.3)
        assert s["rpm"] == 0, "reports %d rpm at standstill" % s["rpm"]

    def t_constant():
        st = ready(300.0)
        p = Pass(300.0)
        p0, s = run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7))
        assert s["pos"] == p0, "ended at %s, began at %s" % (s["pos"][:3], p0[:3])
        assert c.sflags_seen & SF_SYNC, "never reported playing by the spindle"
        assert not (s["sflags"] & (SF_WAIT_INDEX | SF_SYNC)), "still reports spindle flags at rest: %02X" % s["sflags"]
        assert not (c.sflags_seen & SF_UNDERRUN), "reported an underrun"
        worst = check_passes([p], 0.0005)
        # the pulse train itself: at the pitch speed the steps must be evenly spaced
        steps = cuts()[0]
        cruise = [steps[i + 1][0] - steps[i][0] for i in range(len(steps) // 4, 3 * len(steps) // 4)]
        nominal = 1e6 / (STEPS_PER_REV * 5.0)
        assert min(cruise) >= 0.8 * nominal and max(cruise) <= 1.2 * nominal, \
            "step spacing %d..%d us at a nominal %.0f us" % (min(cruise), max(cruise), nominal)
        return "%s; steps %d..%d us apart at a nominal %.0f" % (degrees(worst), min(cruise), max(cruise), nominal)

    def t_plan_differs():
        st = ready(287.3)
        p = Pass(300.0)                             # the host believed 300
        run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7))
        return degrees(check_passes([p], 0.0005))

    def t_droop():
        st = ready(300.0)
        sim(cmd="spindle", ripple=0.02, ripple_hz=6.0)
        p = Pass(300.0)
        # the tool bites: the spindle loses a sixth of its speed in a fraction of a second
        run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7),
            events=[(0.9, lambda s: s(cmd="spindle", rpm=250.0, accel=400.0))])
        steps = cuts()[0]
        took = (steps[-1][0] - steps[0][0]) / 1e6
        assert took > p.duration * 1.05, "the pass took %.2f s: the spindle did not slow down during it" % took
        return degrees(check_passes([p], 0.0008))

    def t_passes():
        st = ready(300.0)
        sim(cmd="spindle", ripple=0.06, ripple_hz=0.23)     # a different speed on every pass
        plans = [Pass(300.0) for _ in range(4)]
        segs = []
        for p in plans:
            segs += p.segs + between_passes()
        p0, s = run(st, segs)
        assert s["pos"] == p0, "ended at %s, began at %s" % (s["pos"][:3], p0[:3])
        took = [(r[-1][0] - r[0][0]) / 1e6 for r in cuts()]
        assert max(took) - min(took) > 0.05, "the passes all took the same time (%s s): the speed did not vary" % took
        return "%s over 4 passes" % degrees(check_passes(plans, 0.0008))

    def t_stopped_spindle():
        st = ready(0.0)
        p = Pass(300.0)
        seen = {}

        def look(s):
            seen["before"] = s(cmd="get")
            s(cmd="spindle", rpm=300.0, accel=1500.0)

        p0, s = run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7), events=[(1.0, look)])
        g = seen["before"]
        assert g["sync"] == 1, "not waiting for the index with the spindle at rest (sync %d)" % g["sync"]
        assert g["pos"] == p0, "moved to %s before the spindle turned" % g["pos"][:3]
        assert g["state"] == "RUN", "state %s while waiting for the index" % g["state"]
        assert c.sflags_seen & SF_WAIT_INDEX, "never reported waiting for the index"
        assert not (c.sflags_seen & SF_UNDERRUN), "reported an underrun while waiting"
        return degrees(check_passes([p], 0.0015))   # (it began on a spindle still gathering speed)

    def t_index_unknown():
        # switched on beside a spindle at rest: the controller has yet to see the index pulse
        st = ready(0.0, fresh=True)
        p = Pass(300.0)
        p0, s = run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7),
                    events=[(0.6, lambda s: s(cmd="spindle", rpm=300.0, accel=1500.0))])
        return degrees(check_passes([p], 0.0015))

    def t_stall():
        st = ready(300.0)
        p = Pass(300.0)
        seen = {}

        def stop(s):
            s(cmd="spindle", rpm=0.0, accel=1200.0)

        def look(s):
            a = s(cmd="get")
            time.sleep(0.3)
            b = s(cmd="get")
            seen.update(a=a, b=b)
            s(cmd="spindle", rpm=300.0, accel=1200.0)

        p0, s = run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7), events=[(0.9, stop), (1.5, look)])
        a, b = seen["a"], seen["b"]
        assert a["spindle"]["rpm"] == 0, "the spindle had not stopped"
        assert a["pos"][2] == b["pos"][2], "Z moved %d steps while the spindle stood still" % (b["pos"][2] - a["pos"][2])
        assert 0 < p0[2] - a["pos"][2] < LENGTH, "the spindle did not stop in mid-pass"
        assert a["sync"] == 2 and a["state"] == "RUN", "state %s, sync %d with the spindle stalled" % (a["state"], a["sync"])
        return degrees(check_passes([p], 0.0015))

    def t_reverse():
        st = ready(-300.0)
        plans = [Pass(300.0), Pass(300.0)]
        run(st, plans[0].segs + between_passes() + plans[1].segs + between_passes())
        return degrees(check_passes(plans, 0.0005, spindle_dir=-1))

    def t_low_resolution():
        st = ready(300.0, sensor=("quad", 100, 1))
        sim(cmd="spindle", ripple=0.03, ripple_hz=1.3)
        plans = [Pass(300.0), Pass(300.0)]
        run(st, plans[0].segs + between_passes() + plans[1].segs + between_passes())
        return degrees(check_passes(plans, 0.0035))     # (one count of this encoder is 0.9 degrees)

    def t_no_index():
        st = ready(300.0, sensor=("quad", 1024, 0))
        sim(cmd="spindle", ripple=0.05, ripple_hz=0.31)
        plans = [Pass(300.0) for _ in range(3)]
        segs = []
        for p in plans:
            segs += p.segs + between_passes()
        run(st, segs)
        recorded = cuts()
        assert len(recorded) == 3, "%d passes recorded" % len(recorded)
        # no index pulse: where a pass begins is arbitrary, but it must be the same place every time
        first = recorded[0][0][1] - plans[0].turns_at_step(1)
        worst = 0.0
        for n, (steps, p) in enumerate(zip(recorded, plans)):
            start = steps[0][1] - p.turns_at_step(1)
            origin = first + round(start - first)
            _o, err = measure(steps, p, origin=origin)
            assert abs(err) <= 0.0008, "pass %d is out by %s against the first" % (n + 1, degrees(err))
            worst = max(worst, abs(err))
        return degrees(worst)

    def t_pulse_sensor():
        st = ready(300.0, sensor=("pulse", 1, 1))
        plans = [Pass(300.0), Pass(300.0)]
        run(st, plans[0].segs + between_passes() + plans[1].segs + between_passes())
        return degrees(check_passes(plans, 0.001))

    def t_pulse_sensor_droop():
        st = ready(300.0, sensor=("pulse", 1, 1))
        p = Pass(300.0, length=2 * LENGTH)
        # the spindle sags from 300 to 285 rpm, losing 8 rpm in every turn (2.7 %) while it does
        run(st, p.segs + ramp_move([0, 0, 2 * LENGTH, 0, 0, 0], 1.0),
            events=[(0.9, lambda s: s(cmd="spindle", rpm=285.0, accel=40.0))])
        # One pulse per turn cannot see a change of speed until the turn is over: while the
        # speed changes the thread is out by about what the speed changed by in a turn...
        worst = check_passes([p], 1.3 * 0.027)
        # ...and once the speed is steady again it is where it should be
        steps = cuts()[0]
        origin, _err = measure(steps, p)
        settled = max(abs((steps[i][1] - origin) - p.turns_at_step(i + 1)) for i in range(len(steps) * 2 // 3, len(steps)))
        assert settled <= 0.0005, "still out by %s after the speed had settled" % degrees(settled)
        return "%s during the sag, %s after it" % (degrees(worst), degrees(settled))

    def t_hold():
        # A pass that is stopped is spoilt; played on by the clock it would be cut across the
        # thread, or against a spindle that has stopped.  So it ends there, and the host is told
        # (at once: there is far more of the pass in hand than a tail).
        st = ready(300.0)
        p = Pass(300.0)
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7), until=lambda s: p0[2] - s["pos"][2] > LENGTH // 3)
        assert c.status["sflags"] & SF_SYNC, "not playing by the spindle a third of the way in"
        sim(cmd="hold")
        return "stopped %d steps into the pass" % broken_off(p0, "feed hold")

    def t_starved():
        st = ready(300.0)
        p = Pass(300.0)
        p0 = c.pump(0.05)["pos"]
        half = len(p.segs) // 2
        c.stream(st, p.segs[:half])                 # ...and the host sends no more
        s = c.wait(lambda s: s["sflags"] & SF_SYNC_LOST, 3.0, "the pass to be given up")
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0 and not (s["flags"] & 0x04), 2.0, "standstill")
        at = s["pos"]
        assert c.sflags_seen & SF_UNDERRUN, "the underrun was not reported"
        # the rest of the pass arrives after all: too late, the spindle has gone on without the axes
        try:
            c.stream(st, p.segs[half:half + 100], first=half, timeout=0.7)
        except AssertionError:
            pass                                    # (not accepted: that stream is over)
        s = c.pump(0.2)
        assert s["pos"] == at, "Z went on %d steps when more of the pass arrived" % (at[2] - s["pos"][2])
        return "stopped %d steps into the pass" % broken_off(p0, "queue ran dry")

    def t_trickle():
        # The controller is slowing down for want of motion in mid-pass when a little more of
        # the pass turns up - not its end, and not enough to go on with.
        st = ready(300.0)
        p = Pass(300.0)
        p0 = c.pump(0.05)["pos"]
        half = len(p.segs) // 2
        c.stream(st, p.segs[:half])
        s = c.wait(lambda s: s["sflags"] & SF_UNDERRUN, 3.0, "the underrun")
        c.stream(st, p.segs[half:half + 15], first=half, until=lambda s: s["sflags"] & SF_SYNC_LOST, timeout=1.0)
        return "stopped %d steps into the pass" % broken_off(p0, "a little more of a pass that had run dry")

    def t_late_tail():
        # The end of a pass is no different from the end of any move: a host that is a moment
        # late with the last of it must not find the pass thrown away.
        st = ready(300.0)
        p = Pass(300.0)
        back = ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7)
        p0 = c.pump(0.05)["pos"]
        sim(cmd="trace", axis=2)
        keep = 20                                   # the last 40 ms of the pass, as it comes to rest
        c.stream(st, p.segs[:-keep])
        c.wait(lambda s: s["sflags"] & SF_UNDERRUN, 5.0, "the controller to run short of motion")
        c.pump(0.05)
        last = [seg[:2] + (seg[2] | (SEG_LAST if i == keep - 1 else 0),) + seg[3:] for i, seg in enumerate(p.segs[-keep:])]
        c.stream(st, last, first=len(p.segs) - keep)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 3.0, "the end of the pass")
        assert not (c.sflags_seen & SF_SYNC_LOST), "the pass was abandoned for a late tail"
        assert p0[2] - s["pos"][2] == LENGTH, "the pass ended %d steps from its start, not %d" % (p0[2] - s["pos"][2], LENGTH)
        steps = cuts()[0]
        # all but the tail was cut in step with the spindle
        n = int(len(steps) * 0.95)
        origin = INDEX_AT + round(steps[0][1] - p.turns_at_step(1) - INDEX_AT)
        worst = max(abs((steps[i][1] - origin) - p.turns_at_step(i + 1)) for i in range(n))
        assert worst <= 0.0008, "out by %s before the tail" % degrees(worst)
        st = new_stream()
        c.flush(st)
        c.stream(st, back)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 3.0, "the move back")
        assert s["pos"] == p0, "back at %s, began at %s" % (s["pos"][:3], p0[:3])
        return "%s up to the tail" % degrees(worst)

    def t_runaway():
        # planned for 300 rpm, and the spindle goes to 800: the axes are not driven after it
        st = ready(300.0)
        p = Pass(300.0)
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7), until=lambda s: p0[2] - s["pos"][2] > LENGTH // 4)
        sim(cmd="spindle", rpm=800.0, accel=6000.0)
        went = broken_off(p0, "spindle far faster than planned")
        steps = cuts()[0]
        fastest = min(steps[i + 1][0] - steps[i][0] for i in range(len(steps) - 1))
        nominal = 1e6 / (STEPS_PER_REV * 5.0)
        assert fastest >= nominal / 1.6, "steps %d us apart: more than half as fast again as planned (%.0f us)" % (fastest, nominal)
        return "stopped %d steps into the pass" % went

    def t_hold_before_pass():
        # The stop is still slowing X down when the pass is reached: it must not be forgotten there.
        st = ready(300.0)
        p = Pass(300.0)
        lead = approach()
        p0 = c.pump(0.05)["pos"]
        seen = {}

        def hold(s):
            while s(cmd="get")["pos"][0] - p0[0] < 30000 - 3000:    # a tenth of a second from its end
                time.sleep(0.003)
            s(cmd="hold")
            time.sleep(0.5)
            a = s(cmd="get")
            time.sleep(0.3)
            seen.update(a=a, b=s(cmd="get"))
            s(cmd="resume")

        back = ramp_move([-30000, 0, LENGTH, 0, 0, 0], 1.2)
        _p0, s = run(st, lead + p.segs + back, events=[(0.0, hold)])
        a, b = seen["a"], seen["b"]
        assert a["state"] == "HOLD" and b["state"] == "HOLD", "state %s, then %s, after feed hold" % (a["state"], b["state"])
        assert a["pos"][2] == p0[2] and b["pos"][2] == p0[2], \
            "Z went %d steps into the pass while held" % (p0[2] - b["pos"][2])
        assert a["pos"][0] - p0[0] == 30000, "X stopped %d steps into its move of 30000" % (a["pos"][0] - p0[0])
        assert not (c.sflags_seen & SF_SYNC_LOST), "the pass was abandoned, though it had not begun"
        assert s["pos"] == p0, "ended at %s, began at %s" % (s["pos"][:3], p0[:3])
        return degrees(check_passes([p], 0.0005))

    def t_flush_before_pass():
        st = ready(300.0)
        p = Pass(300.0)
        lead = approach()
        p0 = c.pump(0.05)["pos"]
        c.stream(st, lead + p.segs, until=lambda s: s["pos"][0] - p0[0] > 30000 - 4500)
        t0 = time.perf_counter()
        st = new_stream()
        c.flush(st, mode=0)                         # ramp to a stop, then drop the rest
        took = time.perf_counter() - t0
        s = c.pump(0.3)
        assert took < 0.8, "the flush took %.2f s" % took
        assert s["pos"][2] == p0[2], "Z went %d steps into the pass after the flush" % (p0[2] - s["pos"][2])
        assert s["state"] == IDLE and not (s["sflags"] & (SF_SYNC | SF_WAIT_INDEX | SF_SYNC_LOST)), \
            "state %d, stream flags %02X after the flush" % (s["state"], s["sflags"])
        c.stream(st, ramp_move([p0[0] - s["pos"][0], 0, 0, 0, 0, 0], 0.8))
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 3.0, "the move back")
        assert s["pos"] == p0, "back at %s, began at %s" % (s["pos"][:3], p0[:3])
        return "flushed in %.2f s" % took

    def t_fast():
        # 30000 steps a second: the step pin's own timing has to hold whatever the spindle does
        st = ready(600.0)
        sim(cmd="spindle", ripple=0.04, ripple_hz=3.0)
        p = Pass(600.0, length=30000, steps_per_rev=3000)
        run(st, p.segs + ramp_move([0, 0, 30000, 0, 0, 0], 1.2))
        steps = cuts()[0]
        assert len(steps) == 30000, "the pass made %d steps of 30000" % len(steps)
        assert not (c.sflags_seen & (SF_SYNC_LOST | SF_UNDERRUN)), "stream flags %02X" % c.sflags_seen
        gaps = [steps[i + 1][0] - steps[i][0] for i in range(len(steps) - 1)]
        assert min(gaps) >= 10, "two steps %d us apart: the pin was not low for its time" % min(gaps)
        origin, err = measure(steps, p)
        assert abs(err) <= 0.001, "out by %s" % degrees(err)
        return "%s; steps never closer than %d us (nominal %.0f)" % (degrees(err), min(gaps), 1e6 / 30000.0)

    def t_anywhere():
        # without INDEX_START a run begins at once, wherever the spindle is - and keeps the pitch
        st = ready(300.0)
        sim(cmd="spindle", ripple=0.05, ripple_hz=0.7)
        p = Pass(300.0, at_index=False)
        run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7))
        steps = cuts()[0]
        origin = steps[0][1] - p.turns_at_step(1)
        _o, err = measure(steps, p, origin=origin)
        assert abs(err) <= 0.0008, "out by %s" % degrees(err)
        assert not (c.sflags_seen & (SF_WAIT_INDEX | SF_SYNC_LOST)), "stream flags %02X" % c.sflags_seen
        return degrees(err)

    def t_anywhere_from_rest():
        # a single sensor says nothing until the spindle has turned: the run begins when it has
        st = ready(0.0, sensor=("pulse", 1, 1))
        p = Pass(300.0, at_index=False)
        p0, s = run(st, p.segs + ramp_move([0, 0, LENGTH, 0, 0, 0], 0.7),
                    events=[(0.8, lambda s: s(cmd="spindle", rpm=300.0, accel=1500.0))])
        assert s["pos"] == p0, "ended at %s, began at %s" % (s["pos"][:3], p0[:3])
        assert not (c.sflags_seen & SF_SYNC_LOST), "the run was abandoned"
        steps = cuts()[0]
        assert len(steps) == LENGTH, "the pass made %d steps" % len(steps)
        # (it began on a spindle still gathering speed; what matters is the pitch once it is steady)
        k = len(steps) // 2
        origin = steps[k][1] - p.turns_at_step(k + 1)
        worst = max(abs((steps[i][1] - origin) - p.turns_at_step(i + 1)) for i in range(k, len(steps)))
        assert worst <= 0.001, "out by %s over the second half" % degrees(worst)
        return degrees(worst)

    def t_several_pulses():
        # Four pulses a turn tell the speed, but not which of them is which: no thread can be
        # cut by that, and the controller does not pretend otherwise.
        st = ready(300.0, sensor=("pulse", 4, 1))
        info = c.handshake()
        assert not (info["caps"] & CAP_SPINDLE_SYNC), "advertises spindle synchronisation with four pulses per revolution"
        s = c.pump(0.5)
        assert abs(s["rpm"] - 300) <= 2, "reports %d rpm at 300" % s["rpm"]
        p0 = s["pos"]
        c.send(1, bytes([1, 1, st, 0]) + struct.pack("<6iIIIIHBB", 0, 0, -2560, 0, 0, 0, 2000, 0, 0, 300000, 0,
                                                     0x10 | SEG_SUBSTEP | SEG_SYNC | SEG_INDEX, 0))
        s = c.wait(lambda s: s["state"] == ALARM, 1.0, "the segment to be refused")
        assert s["pos"] == p0, "moved"
        c.send(7)
        c.wait(lambda s: s["state"] == IDLE, 2.0, "reset")

    def t_flush_while_waiting():
        st = ready(0.0)
        p = Pass(300.0)
        p0 = c.pump(0.05)["pos"]
        c.stream(st, p.segs[:100])
        s = c.wait(lambda s: s["sflags"] & SF_WAIT_INDEX, 2.0, "the wait for the index")
        c.pump(0.3)
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0], mode=0)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 2.0, "idle after the flush")
        assert s["pos"] == p0, "moved to %s" % s["pos"][:3]
        assert not (s["sflags"] & (SF_WAIT_INDEX | SF_SYNC)), "spindle flags still set: %02X" % s["sflags"]
        # and the stream that follows is not waiting for anything
        back = ramp_move([0, 0, -300, 0, 0, 0], 0.3) + ramp_move([0, 0, 300, 0, 0, 0], 0.3)
        c.stream(stream_no[0], back)
        s = c.wait(lambda s: s["state"] == IDLE and s["queued_ms"] == 0, 3.0, "an ordinary move afterwards")
        assert s["pos"] == p0, "ordinary move ended at %s" % s["pos"][:3]

    def t_no_sensor():
        st = ready(0.0, sensor=("quad", 0, 0))
        info = c.handshake()
        assert not (info["caps"] & CAP_SPINDLE_SYNC), "advertises spindle synchronisation without a sensor"
        p0 = c.pump(0.05)["pos"]
        c.send(1, bytes([1, 1, st, 0]) + struct.pack("<6iIIIIHBB", 0, 0, -2560, 0, 0, 0, 2000, 0, 0, 300000, 0,
                                                     0x10 | SEG_SUBSTEP | SEG_SYNC | SEG_INDEX, 0))
        s = c.wait(lambda s: s["state"] == ALARM, 1.0, "the segment to be refused")
        assert s["pos"] == p0, "moved"
        c.send(7)
        c.wait(lambda s: s["state"] == IDLE, 2.0, "reset")

    c.connect()
    c.handshake()
    c.send_io()
    c.pump(0.2)
    for a in range(3):
        c.cfg_set(0x0010 + a, 2, 40000)
        c.cfg_set(0x0020 + a, 2, 100000)

    test("spindle speed and index are tracked", t_tracking)
    test("thread at a steady spindle speed", t_constant)
    test("thread when the spindle is not at the speed planned for", t_plan_differs)
    test("thread while the spindle slows down in the cut", t_droop)
    test("four passes at different spindle speeds line up", t_passes)
    test("waits for a spindle that is not turning", t_stopped_spindle)
    test("index not yet seen when the pass is sent", t_index_unknown)
    test("spindle stalls in mid-pass: axes stop with it and go on with it", t_stall)
    test("spindle in reverse", t_reverse)
    test("encoder of 100 lines", t_low_resolution)
    test("encoder without an index pulse: passes still line up", t_no_index)
    test("one pulse per revolution, steady speed", t_pulse_sensor)
    test("one pulse per revolution, spindle slowing", t_pulse_sensor_droop)
    test("thread at 30000 steps a second", t_fast)
    test("run that begins wherever the spindle is", t_anywhere)
    test("run that begins anywhere, single sensor, spindle at rest", t_anywhere_from_rest)
    test("feed hold in mid-pass: the pass is abandoned, not played on", t_hold)
    test("queue runs dry in mid-pass: abandoned", t_starved)
    test("more of a pass arrives while the controller is stopping for want of it: abandoned", t_trickle)
    test("last of a pass arrives late: finished, not abandoned", t_late_tail)
    test("spindle runs away from the axes: abandoned", t_runaway)
    test("feed hold just before a pass: held, and the pass then cut in step", t_hold_before_pass)
    test("flush just before a pass", t_flush_before_pass)
    test("flush while waiting for the index", t_flush_while_waiting)
    test("no spindle sensor: not advertised, segment refused", t_no_sensor)
    test("single sensor with several pulses per revolution: speed only", t_several_pulses)

    # leave the simulated machine as it was found
    sim(cmd="encoder", mode="quad", ppr=0, index=1)
    sim(cmd="spindle", follow=1, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
    sim(cmd="trace", axis=-1)
    sim(cmd="set", lim_lo=[-4000] * 6, lim_hi=[200000] * 6, phys=[0] * 6)

    failed = [r for r in results if r[1] != "ok"]
    print("\n%d passed, %d failed" % (len(results) - len(failed), len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
