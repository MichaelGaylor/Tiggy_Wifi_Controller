#!/usr/bin/env python3
"""
TiggyUGS's "Serial GRBL" mode against a real controller.

    <TiggyUGS python> tools/ugs_serial_test.py COM14 [--ugs path\\to\\Tiggy_UGS]

Drives TiggyUGS's own serial connection and G-code sender (its main window is not started, its
saved settings are not touched).  The axes move: no motors connected, or the machine free to move.
"""
import argparse
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--ugs", default=os.path.normpath(os.path.join(HERE, "..", "..", "Tiggy_UGS")))
    args = ap.parse_args()
    assert args.port.upper() != "COM1"
    sys.path.insert(0, args.ugs)
    from PyQt6.QtCore import QCoreApplication
    from connection.serial_grbl import SerialGrblConnection
    from core.gcode_parser import parse_file
    from core.gcode_sender import GCodeSender

    app = QCoreApplication(sys.argv[:1])
    last, errors, done = {}, [], []
    conn = SerialGrblConnection()
    conn.status_updated.connect(lambda s: last.update(s))
    conn.error_occurred.connect(lambda m: errors.append(m))
    sender = GCodeSender()
    sender.completed.connect(lambda: done.append(True))
    sender.error_occurred.connect(lambda m: errors.append(m))

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.005)

    def wait(cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            app.processEvents()
            if cond():
                return
            time.sleep(0.005)
        raise AssertionError("timed out waiting for %s (TiggyUGS sees %s at %s; errors %s)" % (
            what, last.get("state"), last.get("positions", [])[:3], errors))

    def pos():
        # machine position: TiggyUGS shows work coordinates, worked out with the last offset it was told
        return [round(p + w, 3) for p, w in zip(last["positions"][:3], conn._wco[:3])]

    def home():
        conn.send_line("G90 G0 X0 Y0 Z0")
        pump(0.4)
        wait(lambda: last.get("state") == "Idle" and all(abs(v) < 0.011 for v in pos()), 30.0, "the way back to zero")

    def program(text):
        f = tempfile.NamedTemporaryFile("w", suffix=".nc", delete=False)
        f.write(text)
        f.close()
        try:
            return parse_file(f.name)
        finally:
            os.unlink(f.name)

    failed = 0

    def test(name, fn):
        nonlocal failed
        del errors[:]
        try:
            fn()
            print("PASS  %s" % name)
        except AssertionError as e:
            failed += 1
            print("FAIL  %s\n      %s" % (name, e))
            try:
                sender.stop()
                conn.reset()
                pump(1.5)
                conn.send_line("$X")
                pump(0.5)
            except Exception:
                pass

    def t_connect():
        conn.connect_to(args.port)
        assert conn.is_connected, "not connected: %s" % errors
        conn.reset()                                    # as after a fresh start
        pump(1.5)
        conn.send_line("$X")
        wait(lambda: last.get("state") == "Idle", 4.0, "an Idle status")
        sender.set_connection(conn)
        for line in ("$100=100", "$101=100", "$102=100", "$110=3000", "$111=3000", "$112=3000", "$120=500", "$121=500",
                     "$122=500", "G21 G90"):
            conn.send_line(line)
            pump(0.15)

    def t_jog():
        wait(lambda: last.get("state") == "Idle", 3.0, "Idle")
        p0 = pos()
        conn.jog(0, +1, 600.0)                          # TiggyUGS asks for 10 m and cancels on release
        wait(lambda: last.get("state") == "Jog", 2.0, "the Jog state")
        pump(0.5)
        conn.jog_stop(0)
        wait(lambda: last.get("state") == "Idle", 3.0, "the jog to stop")
        moved = pos()[0] - p0[0]
        assert 1.0 < moved < 20.0, "half a second of jogging at 10 mm/s moved %.2f mm" % moved
        a = pos()
        pump(0.5)
        assert pos() == a, "moved on after the jog was cancelled"

    def t_program():
        home()
        p0 = pos()
        del done[:]
        lines = ["G21 G90", "G0 X5 Y0", "G1 X15 Y0 F1500", "G1 X15 Y10", "G3 X5 Y20 I-10 J0", "G1 X0 Y20", "G0 Z3"]
        lines += ["G1 X%.2f Y%.2f F3000" % (i * 0.2, 20 - (i % 2) * 0.3) for i in range(1, 80)]
        lines += ["G0 X0 Y0 Z0", "M2"]
        sender.load_file(program("\n".join(lines) + "\n"))
        sender.start()
        wait(lambda: done, 60.0, "the program to be sent")
        pump(0.5)
        wait(lambda: last.get("state") == "Idle", 60.0, "the machine to finish")
        pump(0.4)
        moved = [round(b - a, 3) for a, b in zip(p0, pos())]
        assert all(abs(m) < 0.011 for m in moved), "the program ended %s mm from where it started" % moved
        # TiggyUGS writes to the port from three threads without a lock; on Windows pyserial then
        # reports "Write timeout" now and then although the bytes go out.  Its quirk, not the board's.
        real = [e for e in errors if "Write timeout" not in e]
        assert not real, real
        if len(real) != len(errors):
            print("note  TiggyUGS reported %d serial write timeout(s) of its own making" % (len(errors) - len(real)))

    def t_pause():
        home()
        p0 = pos()
        del done[:]
        sender.load_file(program("G21 G90\nG1 X30 F900\nG1 X0\nM2\n"))
        sender.start()
        wait(lambda: last.get("state") == "Run", 3.0, "the Run state")
        pump(0.6)
        sender.pause()
        conn.feed_hold()
        wait(lambda: last.get("state", "").startswith("Hold"), 3.0, "the Hold state")
        pump(0.5)
        a = pos()
        pump(0.5)
        assert pos() == a, "moving in feed hold"
        conn.feed_resume()
        sender.start()
        wait(lambda: done, 30.0, "the program to be sent")
        wait(lambda: last.get("state") == "Idle", 30.0, "the machine to finish")
        pump(0.4)
        assert abs(pos()[0] - p0[0]) < 0.011, "ended %.3f mm from the start" % (pos()[0] - p0[0])

    def t_stop():
        home()
        del done[:]
        sender.load_file(program("G21 G90\nG1 X60 F900\nM2\n"))
        sender.start()
        wait(lambda: last.get("state") == "Run", 3.0, "the Run state")
        pump(0.6)
        sender.stop()                                   # the Stop button
        conn.reset()
        pump(1.5)
        a, sa = pos(), last.get("state")
        pump(0.6)
        b, sb = pos(), last.get("state")
        assert a == b, "moving after Stop: %s (%s) then %s (%s)" % (a, sa, b, sb)
        assert b[0] < 59.0, "the move ran to its end although Stop was pressed"

    test("connect and unlock", t_connect)
    test("jog, cancelled on release", t_jog)
    test("program with rapids, feeds, an arc and 80 short moves", t_program)
    test("pause and resume", t_pause)
    test("stop", t_stop)
    try:
        conn.disconnect_from()
    except Exception:
        pass
    print("\n%d failed" % failed if failed else "\nall passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
