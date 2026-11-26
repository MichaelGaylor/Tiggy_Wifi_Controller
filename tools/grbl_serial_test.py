#!/usr/bin/env python3
"""
The controller's G-code (GRBL-compatible) interface on its USB serial port, on a real board.

    python tools/grbl_serial_test.py COM14              (the USB serial port)
    python tools/grbl_serial_test.py 192.168.1.43       (the same interface over the network, port 23)

This is the interface Universal G-code Sender, CNCjs and TiggyUGS's "Serial GRBL" mode use.  The
test moves the axes: run it on a board with no motors connected, or with the machine free to move
20 mm in X, Y and Z.  It needs pyserial.
"""
import re
import sys
import time

STATUS = re.compile(r"<(\w+)(?::\d+)?\|MPos:([-\d.,]+)")


class Telnet:
    """The G-code interface over the network, with the few calls of a serial port this test uses."""

    def __init__(self, host):
        import socket
        self.sock = socket.create_connection((host, 23), timeout=5.0)
        self.sock.settimeout(0.05)
        self.timeout = socket.timeout

    def write(self, data):
        self.sock.sendall(data)

    def read(self, n):
        try:
            return self.sock.recv(n)
        except self.timeout:
            return b""

    def reset_input_buffer(self):
        while self.read(4096):
            pass


class Grbl:
    def __init__(self, port):
        if port.upper().startswith("COM") or port.startswith("/dev/"):
            import serial
            self.s = serial.Serial()
            self.s.port, self.s.baudrate, self.s.timeout = port, 115200, 0.05
            self.s.dtr = False                          # leave the board's reset line alone
            self.s.rts = False
            self.s.open()
        else:
            self.s = Telnet(port)
        time.sleep(0.3)
        self.s.reset_input_buffer()
        self.buf = b""

    def lines(self, seconds):
        """Whatever arrives within the time, as lines."""
        end = time.time() + seconds
        out = []
        while True:
            waiting = getattr(self.s, "in_waiting", None)
            if waiting is None:
                self.buf += self.s.read(256)            # network: returns with whatever has come
            elif waiting:
                self.buf += self.s.read(waiting)        # serial: what is there, without waiting for more
            else:
                time.sleep(0.001)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                out.append(line.decode("ascii", "replace").strip())
            if time.time() >= end:
                break
        return [line for line in out if line]

    def cmd(self, text, timeout=3.0):
        """Send a line; return the lines up to its ok / error."""
        self.s.write((text + "\n").encode())
        end = time.time() + timeout
        out = []
        while time.time() < end:
            for line in self.lines(0.05):
                out.append(line)
                if line == "ok" or line.startswith("error") or line.startswith("ALARM"):
                    return out
        raise AssertionError("no answer to %r (got %s)" % (text, out))

    def status(self):
        self.lines(0.005)                               # nothing left over from an earlier question
        for attempt in range(3):
            self.s.write(b"?")
            end = time.time() + 0.4
            while time.time() < end:                    # the answer usually comes within milliseconds
                for line in self.lines(0.01):
                    m = STATUS.search(line)
                    if m:
                        return m.group(1), [float(v) for v in m.group(2).split(",")], line
        raise AssertionError("no status report")

    def wait(self, state, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            st = self.status()
            if st[0] == state:
                return st
        raise AssertionError("timed out waiting for %s (last: %s)" % (what, st[2]))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    g = Grbl(sys.argv[1])
    failed = 0

    def test(name, fn):
        nonlocal failed
        try:
            fn()
            print("PASS  %s" % name)
        except AssertionError as e:
            failed += 1
            print("FAIL  %s\n      %s" % (name, e))
            g.s.write(b"\x18")                          # soft reset, then unlock, for the next test
            g.lines(1.0)
            try:
                g.cmd("$X")
            except AssertionError:
                pass

    def t_hello():
        for attempt in range(4):                        # (a board just switched on takes a moment)
            g.s.write(b"\x18")                          # soft reset: the greeting follows
            got = g.lines(1.5)
            if any("Grbl" in line for line in got):
                break
        assert any("Grbl" in line for line in got), "no greeting after a soft reset: %s" % got
        g.cmd("$X")
        st = g.status()
        assert st[0] == "Idle", "state after reset and unlock: %s" % st[2]

    def t_settings():
        out = g.cmd("$$")
        names = {line.split("=")[0] for line in out if line.startswith("$")}
        for want in ("$100", "$101", "$102", "$110", "$120"):
            assert want in names, "setting %s missing from $$ (%d settings listed)" % (want, len(names))
        info = g.cmd("$I")
        assert any("VER" in line for line in info), "no version in $I: %s" % info

    steps = {}

    def t_setup():
        # known tuning for the moves below: 100 steps/mm, 3000 mm/min, 500 mm/s^2
        for n, v in ((100, 100), (101, 100), (102, 100), (110, 3000), (111, 3000), (112, 3000), (120, 500), (121, 500),
                     (122, 500)):
            out = g.cmd("$%d=%s" % (n, v))
            assert out[-1] == "ok", "$%d=%s refused: %s" % (n, v, out)
        out = g.cmd("$$")
        for line in out:
            if line.startswith("$") and "=" in line:
                steps[line.split("=")[0]] = line.split("=")[1].split()[0]
        assert float(steps["$100"]) == 100.0 and float(steps["$110"]) == 3000.0, "settings not kept: %s %s" % (
            steps.get("$100"), steps.get("$110"))
        assert g.cmd("G21 G90 G94")[-1] == "ok"
        assert g.cmd("G92 X0 Y0 Z0")[-1] == "ok"

    def t_move():
        p0 = g.status()[1]
        t0 = time.time()
        assert g.cmd("G1 X10 Y5 F1200")[-1] == "ok"
        g.wait("Run", 1.0, "the Run state")
        st = g.wait("Idle", 6.0, "the move to finish")
        took = time.time() - t0
        moved = [b - a for a, b in zip(p0, st[1])]
        assert abs(moved[0] - 10) < 0.011 and abs(moved[1] - 5) < 0.011, "G1 X10 Y5 moved %s" % moved[:3]
        want = (10 ** 2 + 5 ** 2) ** 0.5 / 20.0          # mm at 20 mm/s
        assert want * 0.9 < took < want + 0.6, "11.18 mm at 1200 mm/min took %.2f s" % took

    def t_arc_and_rapid():
        p0 = g.status()[1]
        for line in ("G0 X20 Y5", "G2 X30 Y5 I5 J0 F900", "G3 X20 Y5 I-5 J0", "G0 X10 Y5 Z2", "G1 Z0 F300"):
            assert g.cmd(line, timeout=8.0)[-1] == "ok", line
        st = g.wait("Idle", 15.0, "the moves to finish")
        moved = [b - a for a, b in zip(p0, st[1])]
        assert all(abs(m) < 0.011 for m in moved[:3]), "ended %s from where it started" % moved[:3]

    def t_hold():
        p0 = g.status()[1]
        assert g.cmd("G1 X-10 F600")[-1] == "ok"         # 20 mm at 10 mm/s
        g.wait("Run", 1.0, "the Run state")
        time.sleep(0.6)
        g.s.write(b"!")
        st = g.wait("Hold", 2.0, "the Hold state")
        time.sleep(0.5)
        held = g.status()
        assert held[0] == "Hold" and held[1] == g.status()[1], "moving in feed hold"
        assert -19.5 < held[1][0] - p0[0] < -1, "held at %s" % held[1]
        g.s.write(b"~")
        st = g.wait("Idle", 6.0, "the move to finish after resume")
        assert abs(st[1][0] - p0[0] + 20) < 0.011, "X moved %.3f in all" % (st[1][0] - p0[0])

    def t_jog():
        p0 = g.status()[1]
        assert g.cmd("$J=G91 G21 X5 F600")[-1] == "ok"
        g.wait("Jog", 1.0, "the Jog state")
        st = g.wait("Idle", 4.0, "the jog to finish")
        assert abs(st[1][0] - p0[0] - 5) < 0.011, "jog moved %.3f" % (st[1][0] - p0[0])
        assert g.cmd("$J=G91 G21 X50 F600")[-1] == "ok"
        g.wait("Jog", 1.0, "the Jog state")
        time.sleep(0.4)
        g.s.write(b"\x85")                              # jog cancel
        st = g.wait("Idle", 3.0, "the jog to stop")
        assert 1 < st[1][0] - p0[0] - 5 < 30, "cancelled jog went %.3f" % (st[1][0] - p0[0] - 5)

    def t_many_lines():
        # a sender's stream: many short lines, each acknowledged, nothing lost
        p0 = g.status()[1]
        assert g.cmd("G91")[-1] == "ok"
        for i in range(120):
            out = g.cmd("G1 X0.1 Y%s F3000" % ("0.1" if i % 2 else "-0.1"), timeout=8.0)
            assert out[-1] == "ok", "line %d: %s" % (i, out)
        assert g.cmd("G90")[-1] == "ok"
        st = g.wait("Idle", 20.0, "the lines to be played")
        assert abs(st[1][0] - p0[0] - 12) < 0.011 and abs(st[1][1] - p0[1]) < 0.011, "120 lines ended %s from the start" % (
            [round(b - a, 3) for a, b in zip(p0, st[1])][:2])

    def t_no_drift():
        # moves whose lengths are not whole steps: the rounding of each must not add up
        assert g.cmd("G90 G0 X0 Y0 Z0")[-1] == "ok"
        g.wait("Idle", 20.0, "the way to zero")
        time.sleep(0.3)
        zero = g.status()[1]                            # machine position of the work zero
        x = y = 0.0
        for i in range(300):
            x += 0.0437 if i % 3 else -0.0291
            y += 0.0263 if i % 2 else -0.0189
            assert g.cmd("G1 X%.4f Y%.4f F3000" % (x, y), timeout=8.0)[-1] == "ok"
        want = (round(x * 100) / 100.0, round(y * 100) / 100.0)   # 100 steps/mm
        st = g.wait("Idle", 30.0, "the moves to be played")
        time.sleep(0.3)
        st = g.status()
        got = (st[1][0] - zero[0], st[1][1] - zero[1])
        assert abs(got[0] - want[0]) < 0.0005 and abs(got[1] - want[1]) < 0.0005, (
            "after 300 moves the machine is at X%.3f Y%.3f; the program ends at X%.4f Y%.4f" % (got[0], got[1], x, y))
        assert g.cmd("G0 X0 Y0")[-1] == "ok"
        g.wait("Idle", 20.0, "the way back")
        time.sleep(0.3)
        st = g.status()
        assert abs(st[1][0] - zero[0]) < 0.0005 and abs(st[1][1] - zero[1]) < 0.0005, (
            "back at zero the machine is %.3f, %.3f away from it" % (st[1][0] - zero[0], st[1][1] - zero[1]))

    def t_prompt():
        # feed hold and jog cancel are single bytes: they must not wait in a buffer
        took = []
        for n in range(20):
            time.sleep(0.03)
            g.lines(0.005)
            t0 = time.time()
            g.s.write(b"?")
            while not any(STATUS.search(line) for line in g.lines(0.002)):
                assert time.time() - t0 < 1.0, "no answer to a status query"
            took.append(time.time() - t0)
        took.sort()
        if isinstance(g.s, Telnet):
            # over a radio link the odd answer is late whatever the controller does: most must be prompt
            assert took[len(took) // 2] < 0.03, "half the status queries took more than %.0f ms" % (took[len(took) // 2] * 1000)
        else:
            assert took[-1] < 0.05, "a status query took up to %.0f ms to be answered" % (took[-1] * 1000)

    def t_reset_in_motion():
        assert g.cmd("G90 G1 X-40 F600")[-1] == "ok"
        g.wait("Run", 1.0, "the Run state")
        time.sleep(0.5)
        g.s.write(b"\x18")
        got = g.lines(1.5)
        assert any("Grbl" in line for line in got), "no greeting after reset in motion: %s" % got
        a = g.status()
        time.sleep(0.4)
        b = g.status()
        assert a[1] == b[1], "still moving after a soft reset"
        g.cmd("$X")
        assert g.status()[0] == "Idle", "not Idle after reset and unlock: %s" % g.status()[2]

    test("greeting, unlock, status report", t_hello)
    test("settings and version", t_settings)
    test("settings written and read back", t_setup)
    test("a feed move: distance and time", t_move)
    test("rapids and arcs return to the start", t_arc_and_rapid)
    test("feed hold and resume", t_hold)
    test("jog and jog cancel", t_jog)
    test("120 short lines, nothing lost", t_many_lines)
    test("300 odd-length moves: no drift", t_no_drift)
    test("soft reset while moving", t_reset_in_motion)
    test("a real-time command is answered at once", t_prompt)
    print("\n%d failed" % failed if failed else "\nall passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
