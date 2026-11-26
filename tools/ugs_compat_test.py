#!/usr/bin/env python3
"""
TiggyUGS against the firmware.

TiggyUGS (https://github.com/MichaelGaylor/Tiggy_UGS) talks to the controller in
its "WiFi Packet" mode with protocol 1.1: speed-described motion segments, paced
by the free space the status report shows.  This test does not imitate it - it
drives TiggyUGS's own connection, planner and sender, the way its main window
does, against the firmware simulator:

    python tools/firmware_sim.py --quiet            (in another window)
    <TiggyUGS python> tools/ugs_compat_test.py [--ugs path\\to\\Tiggy_UGS]

It needs TiggyUGS's source and its Python packages (PyQt6); run it with the
interpreter of TiggyUGS's virtual environment.  Nothing of TiggyUGS's saved
settings is read or written: its main window is not started.
"""

import argparse
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from protocol_conformance import Sim  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ugs", default=os.path.normpath(os.path.join(HERE, "..", "..", "Tiggy_UGS")),
                    help="TiggyUGS source directory")
    ap.add_argument("--only", default="", help="run only the tests whose name contains this")
    args = ap.parse_args()
    if not os.path.isfile(os.path.join(args.ugs, "connection", "wifi_packet.py")):
        sys.exit("TiggyUGS source not found in %s (use --ugs)" % args.ugs)
    sys.path.insert(0, args.ugs)

    from PyQt6.QtCore import QCoreApplication
    from connection.wifi_packet import WiFiPacketConnection
    from core.gcode_parser import parse_file
    from core.gcode_sender import GCodeSender
    from core.planner import MotionPlanner
    from core.protocol import ConfigKey, ValueType

    app = QCoreApplication(sys.argv[:1])
    sim = Sim()
    host = "127.0.0.1"
    SPM = 200.0                                         # steps per mm used for the test

    def wait(cond, timeout, what):
        end = time.monotonic() + timeout
        while True:
            app.processEvents()
            v = cond()
            if v:
                return v
            if time.monotonic() > end:
                raise AssertionError("timed out waiting for %s (controller: %s, TiggyUGS sees: %s)" % (
                    what, brief(), last.get("state")))
            time.sleep(0.005)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.005)

    def brief():
        g = sim(cmd="get")
        return "%s pos %s queue %s" % (g["state"], g["pos"][:3], g["queue"])

    def pos():
        return sim(cmd="get")["pos"]

    def at_rest():
        g = sim(cmd="get")
        return g["state"] == "IDLE" and not g["busy"] and not g["moving"]

    # -- what TiggyUGS's main window sets up --------------------------------
    last = {}                                           # the latest status as TiggyUGS decoded it
    errors = []
    sent = []                                           # every segment TiggyUGS sent
    conn = WiFiPacketConnection()
    conn.status_updated.connect(lambda s: last.update(s))
    conn.error_occurred.connect(lambda m: errors.append(m))
    planner = MotionPlanner()
    sender = GCodeSender()
    done = []
    sender.completed.connect(lambda: done.append(True))
    sender.error_occurred.connect(lambda m: errors.append(m))

    real_send = conn.send_motion_segments

    def recording_send(segments):
        sent.extend(segments)
        real_send(segments)
    conn.send_motion_segments = recording_send

    def sent_steps(first=0):
        tot = [0] * 6
        for s in sent[first:]:
            for a in range(6):
                tot[a] += s["steps"][a]
        return tot

    def program(text):
        f = tempfile.NamedTemporaryFile("w", suffix=".nc", delete=False)
        f.write(text)
        f.close()
        try:
            return parse_file(f.name)
        finally:
            os.unlink(f.name)

    def run(text, during=None, timeout=60.0):
        """Run a program the way the Run button does; returns (start position, index of its first segment)."""
        wait(at_rest, 5.0, "the controller to be at rest before a program")
        p0, first = pos(), len(sent)
        del done[:]
        planner.reset()                                 # as when a file is opened in a fresh session
        planner.configure(steps_per_mm=list(conn.steps_per_mm))
        sender.load_file(program(text))
        if last.get("state") == "Hold":
            conn.feed_resume()
        sender.start()
        if during:
            during()
        else:
            wait(lambda: done, timeout, "the program to be sent")
            wait(at_rest, timeout, "the machine to finish the program")
        return p0, first

    def expect_played(p0, first, what):
        want = sent_steps(first)
        got = [b - a for a, b in zip(p0, pos())]
        assert got == want, "%s: TiggyUGS sent %s steps in %d segments, the controller moved %s" % (
            what, want[:3], len(sent) - first, got[:3])

    # -- tests ---------------------------------------------------------------
    def t_discover():
        found = WiFiPacketConnection.discover(timeout=2.0)
        assert found, "TiggyUGS's search found no controller"
        ip, info = found[0]
        assert info.get("firmware_version"), "no firmware version from the search: %s" % info

    def t_connect():
        sim(cmd="set", estop=0, limits=0, probe=0, probe_z=None, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)
        conn.connect_to(host)
        assert conn.is_connected, "TiggyUGS did not connect: %s" % errors
        info = conn._device_info
        assert info.get("firmware_version", "").startswith("1.2"), "firmware version read as %r" % info
        assert info.get("num_axes") == 6, info
        wait(lambda: last.get("state") == "Idle", 3.0, "an Idle status in TiggyUGS")
        assert last["buffer_total"] > 0, last
        sender.set_connection(conn)
        sender.set_planner(planner)

    def t_config():
        for k in (ConfigKey.STEPS_PER_MM_X, ConfigKey.STEPS_PER_MM_Y, ConfigKey.STEPS_PER_MM_Z):
            assert conn.write_config(k, ValueType.FLOAT, SPM), "steps/mm not accepted"
        for a, k in enumerate((ConfigKey.MAX_RATE_X, ConfigKey.MAX_RATE_Y, ConfigKey.MAX_RATE_Z)):
            assert conn.write_config(k, ValueType.UINT32, 20000), "maximum rate not accepted"
            assert conn.write_config(ConfigKey.ACCEL_X + a, ValueType.UINT32, 60000), "acceleration not accepted"
        conn._read_steps_per_mm()                       # what TiggyUGS does when it connects
        assert conn.steps_per_mm[:3] == [SPM] * 3, "steps/mm read back as %s" % conn.steps_per_mm
        planner.configure(steps_per_mm=list(conn.steps_per_mm))

    def t_jog():
        p0 = pos()
        conn.jog(0, +1, 600.0)                          # mm/min
        wait(lambda: last.get("state") == "Jog", 1.0, "the Jog state")
        pump(0.4)
        conn.jog_stop(0)
        wait(at_rest, 3.0, "the jog to stop")
        moved = pos()[0] - p0[0]
        assert 200 < moved < 2200, "0.4 s of jogging at 10 mm/s moved %d steps" % moved
        wait(lambda: last.get("state") == "Idle", 1.0, "Idle after the jog")
        assert abs(last["positions"][0] - pos()[0] / SPM) < 0.01, "TiggyUGS shows X %s mm" % last["positions"][0]

    def t_step_jog():
        # the segment TiggyUGS's main window builds for a 1 mm step at 600 mm/min
        p0 = pos()
        steps, speed = int(1.0 * SPM), int(600 / 60.0 * SPM)
        seg = {"steps": [0, -steps, 0, 0, 0, 0],
               "duration_us": max(1000, int(steps / max(speed, 1) * 1000000)),
               "entry_speed_sqr": 0, "exit_speed_sqr": 0, "acceleration": 5000 * 100,
               "segment_id": 0, "flags": 0x0A}
        real_send([seg])
        wait(lambda: pos()[1] != p0[1], 1.0, "the step to start")
        wait(at_rest, 3.0, "the step to finish")
        assert pos()[1] - p0[1] == -steps, "a 1 mm step moved %d steps" % (pos()[1] - p0[1])

    def t_program():
        p0, first = run("""(square with a rounded corner)
G21 G90
M3 S8000
G0 X10 Y0
G1 X20 Y0 F1200
G1 X20 Y10
G3 X10 Y20 I-10 J0
G1 X0 Y20
G0 Z5
G0 X0 Y0 Z0
M5
""")
        assert len(sent) - first > 8, "the arc was not cut into segments (%d in all)" % (len(sent) - first)
        expect_played(p0, first, "program")
        assert not errors, errors

    def t_long_program():
        # Far more moves than the controller holds: TiggyUGS has to pace them by the room the status
        # report shows.  (Short moves: TiggyUGS starts every segment from rest, so each takes ~0.1 s.)
        lines = ["G21 G90", "G1 F6000"]
        for i in range(1, 301):
            lines.append("G1 X%.3f Y%.3f" % (i * 0.1, 0.1 if i % 2 else 0.0))
        lines.append("G0 X0 Y0")
        most = [0]

        def during():
            end = time.monotonic() + 180.0
            while not (done and at_rest()):
                assert time.monotonic() < end, "program not finished (controller: %s)" % brief()
                pump(0.05)
                if last.get("buffer_total"):
                    most[0] = max(most[0], last["buffer_total"] - last["buffer_available"])
        p0, first = run("\n".join(lines) + "\n", during=during)
        assert len(sent) - first >= 300
        expect_played(p0, first, "long program")
        # TiggyUGS aims at 60 segments waiting; if it never saw the queue fill it was not being paced
        assert 40 <= most[0] <= 80, "TiggyUGS saw at most %d segments waiting in the controller" % most[0]
        assert not errors, errors

    def t_pause():
        lines = ["G21 G90", "G1 F1500"] + ["G1 X%.3f" % (i * 0.5) for i in range(1, 121)] + ["G0 X0"]

        def during():
            wait(lambda: last.get("state") == "Run", 3.0, "the Run state")
            pump(0.5)
            sender.pause()                              # the Pause button
            conn.feed_hold()
            wait(lambda: last.get("state") == "Hold", 2.0, "the Hold state")
            pump(0.4)
            a = pos()
            pump(0.4)
            assert pos() == a, "still moving in feed hold"
            if last.get("state") == "Hold":             # the Run button
                conn.feed_resume()
            sender.start()
            wait(lambda: done, 60.0, "the rest of the program to be sent")
            wait(at_rest, 60.0, "the machine to finish")
        p0, first = run("\n".join(lines) + "\n", during=during)
        expect_played(p0, first, "paused program")

    def t_stop():
        lines = ["G21 G90", "G1 F1500"] + ["G1 X%.3f" % (i * 0.5) for i in range(1, 201)]

        def during():
            wait(lambda: last.get("state") == "Run", 3.0, "the Run state")
            pump(0.6)
            sender.stop()                               # the Stop button
            conn.reset()
            wait(at_rest, 2.0, "the machine to stop")
        p0, first = run("\n".join(lines) + "\n", during=during)
        a = pos()
        assert a[0] - p0[0] < sent_steps(first)[0], "everything sent was played although Stop was pressed"
        pump(0.5)
        assert pos() == a, "moved again after Stop"
        wait(lambda: last.get("state") == "Idle", 1.0, "Idle after Stop")
        t_step_jog()                                    # and it takes motion again

    def t_estop():
        conn.jog(0, -1, 600.0)
        wait(lambda: last.get("state") == "Jog", 1.0, "the Jog state")
        conn.estop()
        wait(lambda: last.get("state") == "E-Stop", 1.0, "the E-Stop state")
        a = pos()
        pump(0.3)
        assert pos() == a, "moving in E-stop"
        conn.reset()
        wait(lambda: last.get("state") == "Idle", 2.0, "Idle after reset")

    def t_outputs():
        conn.set_io(spindle_state=1, spindle_rpm=12000, coolant_state=1, misc_outputs=0x05)
        wait(lambda: sim(cmd="get")["outputs"]["spindle"] == 1, 1.0, "the spindle output")
        o = sim(cmd="get")["outputs"]
        assert o["rpm"] == 12000 and o["coolant"] == 1 and o["misc"] == 0x05, o
        assert o["charge_pump"], "charge pump off although TiggyUGS is connected"
        wait(lambda: last.get("spindle_state") == 1 and last.get("misc_outputs") == 0x05, 1.0, "the outputs in TiggyUGS")
        conn.set_io()
        wait(lambda: sim(cmd="get")["outputs"]["spindle"] == 0, 1.0, "the spindle output off")

    def t_home():
        sim(cmd="set", lim_lo=[pos()[0] - 1500] + [-4000000] * 5, lim_hi=[4000000] * 6)
        conn.home(axis_mask=0x01)
        wait(lambda: last.get("state") == "Homing", 1.0, "the Homing state")
        wait(lambda: last.get("state") != "Homing", 30.0, "homing to finish")
        assert last.get("state") == "Idle", "homing ended in %s (alarm %s)" % (last.get("state"), last.get("alarm_code"))
        assert pos()[0] == 0, "X is %d after homing" % pos()[0]
        sim(cmd="set", lim_lo=[-4000000] * 6)

    def t_idle_link():
        # TiggyUGS sends nothing but a ping every five seconds while idle
        pump(7.0)
        assert conn.is_connected and not errors, errors
        assert last.get("state") == "Idle", last.get("state")
        assert conn.read_config(ConfigKey.STEPS_PER_MM_X, ValueType.FLOAT) == SPM, "control channel no longer answers"

    tests = [
        ("TiggyUGS's search finds the controller", t_discover),
        ("connect, firmware version, status", t_connect),
        ("settings written and read back", t_config),
        ("continuous jog", t_jog),
        ("step jog is exact", t_step_jog),
        ("program with rapids, feeds and an arc", t_program),
        ("long program: nothing lost", t_long_program),
        ("pause and resume", t_pause),
        ("stop in a program", t_stop),
        ("E-stop and reset", t_estop),
        ("spindle, coolant and outputs", t_outputs),
        ("homing", t_home),
        ("link stays up while idle", t_idle_link),
    ]

    failed = 0
    for name, fn in tests:
        if args.only and args.only not in name and fn not in (t_connect, t_config):
            continue
        del errors[:]
        try:
            fn()
            print("PASS  %s" % name)
        except AssertionError as e:
            failed += 1
            print("FAIL  %s\n      %s" % (name, e))
            try:                                        # leave the next test a clean start
                sender.stop()
                conn.jog_stop()
                conn.reset()
                pump(0.5)
                conn.feed_resume()
                pump(0.5)
            except Exception:
                pass
    try:
        conn.disconnect_from()
    except Exception:
        pass
    print("\n%d failed" % failed if failed else "\nall passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
