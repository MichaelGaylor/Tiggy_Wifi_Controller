#!/usr/bin/env python3
"""
Tests of firmware behaviour that the protocol conformance suite does not
reach: G-code mode's speed-described moves, feed hold, homing started
locally, a host that falls behind, and the quality of the pulse train.

Runs against tools/firmware_sim.py (the firmware's own motion code on a PC):

    python tools/firmware_sim.py --quiet &
    python tools/firmware_selftest.py
"""

import math
import sys
import time

from protocol_conformance import Client, Sim, ramp_move

RUN, HOLD, IDLE = "RUN", "HOLD", "IDLE"


def main():
    sim = Sim()
    c = Client("127.0.0.1")
    results = []

    def test(name, fn):
        try:
            fn()
            results.append((name, "ok"))
            print("ok    %s" % name)
        except Exception as e:                      # one failed test must not hide the others
            results.append((name, "FAIL: %s" % e))
            print("FAIL  %s: %s" % (name, e))

    def get():
        return sim(cmd="get")

    def wait(cond, timeout, what):
        end = time.perf_counter() + timeout
        s = get()
        while time.perf_counter() < end:
            s = get()
            if cond(s):
                return s
            c.pump(0.01)                                # keeps the host heartbeat going
        raise AssertionError("timed out waiting for %s (state %s, alarm %s, pos %s)"
                             % (what, s["state"], s["alarm"], s["pos"][:3]))

    def idle():
        if get()["state"] in ("ESTOP", "ALARM"):
            sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None)
            c.send(7)
            c.pump(0.2)
        # the simulated machine: switches far away unless a test wants them, axes in the middle
        sim(cmd="set", lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        return wait(lambda s: s["state"] == IDLE and not s["busy"], 5.0, "controller idle")

    def legacy(steps, entry, exit_, accel, duration_us=0):
        assert sim(cmd="legacy", steps=steps, entry=entry, exit=exit_, accel=accel, duration_us=duration_us)["ok"]

    # ----------------------------------------------------------------- tests
    def t_legacy_triangle():
        p0 = idle()["pos"]
        sim(cmd="clear")
        t0 = time.perf_counter()
        legacy([2000, 1000, 0, 0, 0, 0], 0, 0, 20000)
        wait(lambda s: s["state"] == RUN, 1.0, "move to start")
        s = wait(lambda s: s["state"] == IDLE and not s["busy"], 5.0, "move to finish")
        took = time.perf_counter() - t0
        d = [s["pos"][i] - p0[i] for i in range(6)]
        assert d == [2000, 1000, 0, 0, 0, 0], "position delta %s" % d
        expect = 2 * math.sqrt(2000 / 20000.0)          # accelerate half way, decelerate the rest
        assert expect - 0.05 <= took <= expect + 0.25, "took %.3f s, expected about %.3f s" % (took, expect)
        peak = math.sqrt(20000.0 * 2000)
        assert s["min_gap_us"][0] >= 0.9e6 / peak, "X stepped faster than the profile allows: %s us" % s["min_gap_us"][0]

    def t_legacy_cruise_speed():
        p0 = idle()["pos"]
        sim(cmd="clear")
        t0 = time.perf_counter()
        legacy([4000, 0, 0, 0, 0, 0], 0, 0, 20000, duration_us=2000000)     # cruise at 2000 steps/s
        s = wait(lambda s: s["state"] == IDLE and not s["busy"] and s["pos"][0] != p0[0], 6.0, "move to finish")
        took = time.perf_counter() - t0
        assert s["pos"][0] - p0[0] == 4000, "moved %d" % (s["pos"][0] - p0[0])
        assert 2.0 <= took <= 2.4, "took %.3f s, expected about 2.1 s" % took
        assert s["min_gap_us"][0] >= 480, "cruise speed exceeded: step gap %s us" % s["min_gap_us"][0]

    def t_legacy_chain():
        p0 = idle()["pos"]
        sim(cmd="clear")
        t0 = time.perf_counter()
        legacy([1000, 500, 0, 0, 0, 0], 0, 2000, 20000, duration_us=500000)
        legacy([2000, 1000, 0, 0, 0, 0], 2000, 2000, 20000, duration_us=1000000)
        legacy([1000, 500, 0, 0, 0, 0], 2000, 0, 20000, duration_us=500000)
        s = wait(lambda s: s["state"] == IDLE and not s["busy"] and s["pos"][0] - p0[0] == 4000, 6.0, "moves to finish")
        took = time.perf_counter() - t0
        assert s["pos"][1] - p0[1] == 2000, "Y moved %d" % (s["pos"][1] - p0[1])
        assert 2.0 <= took <= 2.4, "took %.3f s, expected about 2.1 s (no stop between the moves)" % took
        assert s["min_gap_us"][0] >= 480, "speed spike at a junction: step gap %s us" % s["min_gap_us"][0]

    def t_feed_hold():
        p0 = idle()["pos"]
        legacy([-4000, 0, 0, 0, 0, 0], 0, 0, 20000, duration_us=2000000)
        wait(lambda s: s["state"] == RUN and s["feed"] > 1500, 2.0, "cruise")
        sim(cmd="hold")
        s = wait(lambda s: s["state"] == HOLD and s["feed"] == 0, 1.0, "standstill after hold")
        held = s["pos"][0]
        assert held != p0[0] and held - p0[0] != -4000, "hold did not stop mid-move"
        c.pump(0.3)
        assert get()["pos"][0] == held, "axis moved while held"
        sim(cmd="resume")
        s = wait(lambda s: s["state"] == IDLE and not s["busy"], 5.0, "move to finish after resume")
        assert s["pos"][0] - p0[0] == -4000, "moved %d in total" % (s["pos"][0] - p0[0])

    def t_local_home():
        idle()
        sim(cmd="set", phys=[-1000, 0, 0, 0, 0, 0], lim_lo=[-4000] * 6, lim_hi=[200000] * 6)
        assert sim(cmd="home", mask=1)["ok"], "home command refused"

        wait(lambda s: s["state"] == "HOMING", 1.0, "homing to start")
        s = wait(lambda s: s["state"] != "HOMING", 30.0, "homing to finish")
        assert s["state"] == IDLE, "homing ended in %s" % s["state"]
        assert s["pos"][0] == 0, "X is %d after homing" % s["pos"][0]
        off = s["phys"][0] - (-4000)                    # switch at -4000, pull-off 100 steps
        assert 100 <= off <= 140, "X stands %d steps from its switch, expected just over 100" % off

    def t_starved_stream():
        idle()
        for a in range(3):
            c.cfg_set(0x0010 + a, 2, 40000)
            c.cfg_set(0x0020 + a, 2, 20000)
        c.send_io()
        st = (c.pump(0.2)["stream"] + 1) & 0xFF
        c.flush(st)
        p0 = c.pump(0.1)["pos"]
        segs = ramp_move([3000, 0, 0, 0, 0, 0], 1.0)
        sent = sum(seg[0][0] for seg in segs[:120])
        c.stream(st, segs[:120])                        # 0.6 s of a 1 s move, then the host stalls
        s = c.wait(lambda s: s["sflags"] & 2, 2.0, "starvation to be reported")
        assert s["state"] == 1, "state %d when the data ran low: expected to be holding the move" % s["state"]
        assert 0 < s["pos"][0] - p0[0] < sent, "ran low at %d of %d steps" % (s["pos"][0] - p0[0], sent)
        s = c.pump(1.2)                                 # nothing more comes: what was sent is played out

        assert s["state"] == 0 and s["queued_ms"] == 0, "not at rest after the stall: %s" % s
        assert s["pos"][0] - p0[0] == sent, "played %d of the %d steps received" % (s["pos"][0] - p0[0], sent)
        c.stream(st, segs)                              # the host comes back
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 5.0, "move to finish")
        assert s["pos"][0] - p0[0] == 3000, "moved %d in total" % (s["pos"][0] - p0[0])

    def t_brief_stall():
        idle()
        c.send_io()
        st = (c.pump(0.2)["stream"] + 1) & 0xFF
        c.flush(st)
        sim(cmd="clear")
        p0 = c.pump(0.1)["pos"]
        segs = ramp_move([3000, 0, 0, 0, 0, 0], 1.0)
        c.stream(st, segs[:120])
        c.pump(0.15)                                    # a hiccup shorter than the wait: the move carries on
        c.stream(st, segs)
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 5.0, "move to finish")
        assert s["pos"][0] - p0[0] == 3000, "moved %d in total" % (s["pos"][0] - p0[0])


    def t_pulse_spacing():
        idle()
        for a in range(3):
            c.cfg_set(0x0010 + a, 2, 60000)
        c.cfg_set(0x0100, 1, 5)                         # 5 us step pulse
        c.send_io()
        st = (c.pump(0.2)["stream"] + 1) & 0xFF
        c.flush(st)
        sim(cmd="clear")
        p0 = c.pump(0.1)["pos"]
        # 50 kHz on X with Y and Z at awkward ratios to it: more than the
        # interrupt can do, so it must be played a little slower - but exactly
        segs = [([250, 173, -97, 0, 0, 0], 5000, 0)] * 100 + [([250, 173, -97, 0, 0, 0], 5000, 2)]
        c.stream(st, segs)
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 5.0, "move to finish")
        d = [s["pos"][i] - p0[i] for i in range(3)]
        assert d == [25250, 17473, -9797], "position delta %s" % d
        g = get()["min_gap_us"]
        # every step: 5 us high and at least 5 us low
        assert min(g[:3]) >= 10, "steps of one axis %s us apart: closer than the pulse timing allows" % g[:3]

        st2 = (st + 1) & 0xFF
        c.flush(st2)
        back = ramp_move([-d[0], -d[1], -d[2], 0, 0, 0], 1.5)
        c.stream(st2, back)
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 6.0, "return move")

    c.connect()
    c.handshake()
    c.send_io()
    c.pump(0.2)
    idle()
    for a in range(3):                                  # known tuning whatever ran before
        c.cfg_set(0x0010 + a, 2, 20000)
        c.cfg_set(0x0020 + a, 2, 20000)
    c.cfg_set(0x0323, 2, 100)

    test("speed-described move: distance, duration, no overspeed", t_legacy_triangle)
    test("speed-described move: honours its cruise speed", t_legacy_cruise_speed)
    test("speed-described moves join without stopping", t_legacy_chain)
    test("feed hold ramps to a stop and resumes exactly", t_feed_hold)
    test("homing started locally (G-code mode)", t_local_home)
    test("starved stream: holds, then plays out what it has, then continues", t_starved_stream)
    test("brief stall in a stream is ridden out", t_brief_stall)

    test("pulse spacing at 50 kHz on three axes", t_pulse_spacing)

    # leave the simulated machine as it was found
    sim(cmd="set", lim_lo=[-4000] * 6, lim_hi=[200000] * 6, phys=[0] * 6)

    failed = [r for r in results if r[1] != "ok"]
    print("\n%d passed, %d failed" % (len(results) - len(failed), len(failed)))

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
