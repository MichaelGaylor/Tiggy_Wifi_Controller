#!/usr/bin/env python3
"""
Tiggy protocol conformance test (protocol 1.2).

Runs the same checks against the simulator (tools/tiggy_sim.py) or a real
controller, so the firmware and the simulator are held to one behaviour.

    python protocol_conformance.py 127.0.0.1 --sim          simulator, all tests
    python protocol_conformance.py 192.168.1.50             real board, no motion
    python protocol_conformance.py 192.168.1.50 --motion    real board, moves axes!

Motion tests move the X, Y and Z axes by a few hundred steps and jog them.
Only pass --motion when the motors are free to move.  Tests that need an
input to change (E-stop input, probe, limit switches) run only with --sim,
where the simulator's automation port provides them.
"""

import argparse
import json
import random
import socket
import struct
import sys
import time

MAGIC, VER = 0x574D4333, 1
UDP_MOTION_PORT, UDP_STATUS_PORT, TCP_PORT = 58427, 58428, 58429
HDR = struct.Struct("<IBBHIIH")
SEG = struct.Struct("<6iIIIIHBB")
STATUS_V11 = struct.Struct("<6iHH8BHIi4BHHIHH")
STATUS_V12 = struct.Struct("<HBBBBH")
STATES = ["IDLE", "RUN", "HOLD", "JOG", "HOMING", "PROBING", "ALARM", "ESTOP", "DISCONNECTED"]
SEG_PROBE, SEG_LAST, SEG_TIMED, SEG_SUBSTEP = 4, 2, 0x10, 0x20
SEG_SYNC, SEG_INDEX = 0x40, 0x80


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Client:
    def __init__(self, ip, status_port=0):
        self.ip = ip
        self.seq = 0
        self.tcp = None
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("0.0.0.0", status_port))      # 0 = any port, with REPLY_TO_SOURCE
        self.udp.settimeout(0.02)
        self.status = None
        self.status_count = 0
        self.sflags_seen = 0                    # every stream flag reported since a test last cleared this
        self.io = dict(misc=0, spindle=0, rpm=0, coolant=0, flags=0x03)   # WATCHDOG | REPLY_TO_SOURCE
        self.last_io = 0.0
        self.last_ping = time.perf_counter()

    # ---- packets
    def pkt(self, ptype, payload=b""):
        self.seq += 1
        h = HDR.pack(MAGIC, VER, ptype, len(payload), self.seq, int(time.perf_counter() * 1e6) & 0xFFFFFFFF, 0)
        return h[:16] + struct.pack("<H", crc16(h + payload)) + payload

    @staticmethod
    def parse(data):
        if len(data) < HDR.size:
            return None
        magic, ver, ptype, plen, _s, _t, crc = HDR.unpack_from(data)
        if magic != MAGIC or ver != VER or len(data) < HDR.size + plen:
            return None
        body = data[:HDR.size + plen]
        if crc16(body[:16] + b"\0\0" + body[18:]) != crc:
            return None
        return ptype, body[HDR.size:]

    # ---- TCP
    def connect(self):
        self.tcp = socket.create_connection((self.ip, TCP_PORT), timeout=3.0)
        self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def tcp_call(self, ptype, payload=b""):
        frame = self.pkt(ptype, payload)
        self.tcp.sendall(struct.pack("<H", len(frame)) + frame)
        hdr = self._recvn(2)
        n = struct.unpack("<H", hdr)[0]
        return self.parse(self._recvn(n))

    def _recvn(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.tcp.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("TCP closed")
            buf += chunk
        return buf

    def handshake(self):
        r = self.tcp_call(0x40, struct.pack("<I32s", (1 << 24) | (2 << 16), b"conformance"))
        assert r and r[0] == 0x41, "no handshake response"
        fw, axes, caps, cap, maxrate, name = struct.unpack_from("<IBBHI32s", r[1])
        return dict(fw=fw, axes=axes, caps=caps, buffer=cap, max_rate=maxrate, name=name.split(b"\0")[0].decode())

    def cfg_set(self, key, vtype, value):
        fmt = {0: "<B", 1: "<H", 2: "<I", 3: "<i", 4: "<f"}[vtype]
        r = self.tcp_call(0x43, struct.pack("<HH64s", key, vtype, struct.pack(fmt, value)))
        assert r and r[0] == 0x24, "config set 0x%04X not acknowledged" % key

    def cfg_get(self, key):
        r = self.tcp_call(0x42, struct.pack("<HH64s", key, 0, b""))
        assert r and r[0] == 0x44, "config get 0x%04X: no response" % key
        k, vtype = struct.unpack_from("<HH", r[1])
        fmt = {0: "<B", 1: "<H", 2: "<I", 3: "<i", 4: "<f"}.get(vtype)
        return struct.unpack_from(fmt, r[1], 4)[0] if fmt else r[1][4:].split(b"\0")[0]

    # ---- UDP
    def send(self, ptype, payload=b""):
        self.udp.sendto(self.pkt(ptype, payload), (self.ip, UDP_MOTION_PORT))

    def send_io(self):
        io = self.io
        self.send(9, struct.pack("<BBHBBH", io["misc"], io["spindle"], io["rpm"], io["coolant"], io["flags"], 0))
        self.last_io = time.perf_counter()

    def pump(self, seconds=0.0):
        """Keep the heartbeat going and read status for a while."""
        end = time.perf_counter() + seconds
        while True:
            now = time.perf_counter()
            if now - self.last_io > 0.1:
                self.send_io()
            if self.tcp and now - self.last_ping > 2.0:
                self.last_ping = now
                r = self.tcp_call(0x50, struct.pack("<I", self.seq))
                assert r and r[0] == 0x51, "no PONG on the control channel"
            try:
                data, _ = self.udp.recvfrom(512)
                p = self.parse(data)
                if p and p[0] == 0x20 and len(p[1]) >= STATUS_V11.size:
                    f = STATUS_V11.unpack_from(p[1])
                    s = dict(pos=list(f[0:6]), buf_avail=f[6], buf_total=f[7], state=f[8], alarm=f[9],
                             limits=f[10], probe=f[11], homing=f[12], home_sw=f[13], estop_in=f[14], flags=f[15],
                             seg=f[16], feed=f[18], misc_out=f[19], misc_in=f[20], spindle=f[21], coolant=f[22],
                             rpm=f[23], spindle_pos=f[24], index_count=f[25],
                             next_seg=0, stream=0, sflags=0, homed=0, jog=0, queued_ms=0, v12=False)
                    if len(p[1]) >= STATUS_V11.size + STATUS_V12.size:
                        g = STATUS_V12.unpack_from(p[1], STATUS_V11.size)
                        s.update(next_seg=g[0], stream=g[1], sflags=g[2], homed=g[3], jog=g[4], queued_ms=g[5], v12=True)
                        self.sflags_seen |= g[2]
                    self.status = s
                    self.status_count += 1
            except socket.timeout:
                pass
            if now >= end:
                return self.status

    def wait(self, cond, timeout, what):
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            s = self.pump(0.02)
            if s is not None and cond(s):
                return s
        raise AssertionError("timed out waiting for %s (last status: %s)" % (what, self.status))

    def flush(self, stream, mode=1):
        end = time.perf_counter() + 3.0
        while time.perf_counter() < end:
            self.send(0x0A, struct.pack("<BB2x", stream, mode))
            s = self.pump(0.06)
            if s and s["stream"] == stream and not (s["sflags"] & 4) and s["next_seg"] == 0:
                return
        raise AssertionError("stream flush to %d not confirmed" % stream)

    def stream(self, stream, segs, drop=None, timeout=20.0, until=None, first=0):
        """Send (steps[6], duration_us, flags) segments reliably; returns when all are accepted.
        A segment may have a fourth item: the spindle speed it was planned for, rpm * 1000
        (for SEG_SYNC).  'drop' is an optional function(packet_index) -> bool to discard
        packets on purpose.  'until' is an optional function(status) -> bool: when it says
        so, sending stops there and then.  'first' is the number of segments this stream
        has been sent already (by an earlier call)."""
        base, nxt, sent_pkts, last_progress = 0, 0, 0, time.perf_counter()
        end = time.perf_counter() + timeout
        while base < len(segs):
            s = self.pump(0.0)
            if until and s and until(s):
                return sent_pkts
            if s and s["stream"] == stream:
                acked = base + ((s["next_seg"] - first - base) & 0xFFFF)
                if base < acked <= nxt:
                    base, last_progress = acked, time.perf_counter()
                room = s["buf_avail"] - (nxt - base)
                if time.perf_counter() - last_progress > 0.15:
                    nxt, last_progress = base, time.perf_counter()       # resend everything unacknowledged
                while nxt < len(segs) and room > 0:
                    n = min(8, len(segs) - nxt, room)
                    body = struct.pack("<BBBB", n, 1, stream, 0)
                    for i in range(nxt, nxt + n):
                        st, dur, fl = segs[i][:3]
                        ref = segs[i][3] if len(segs[i]) > 3 else 0
                        body += SEG.pack(*st, dur, 0, 0, ref, (first + i) & 0xFFFF, fl | SEG_TIMED, 0)
                    if not (drop and drop(sent_pkts)):
                        self.send(1, body)
                    sent_pkts += 1
                    nxt += n
                    room -= n
            assert time.perf_counter() < end, "stream stalled at segment %d of %d" % (base, len(segs))
            self.pump(0.01)
        return sent_pkts


class Sim:
    """Automation channel of tiggy_sim.py"""

    def __init__(self, port=58500):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self.f = self.s.makefile("rwb")

    def __call__(self, **req):
        self.f.write((json.dumps(req) + "\n").encode())
        self.f.flush()
        return json.loads(self.f.readline().decode())


def ramp_move(axis_steps, seconds, slice_us=5000):
    """A trapezoid-profile move cut into timed slices, like a planner would send."""
    n = int(seconds * 1e6 / slice_us)
    ramp = max(1, n // 4)
    weights = [min(1.0, (i + 0.5) / ramp, (n - i - 0.5) / ramp) for i in range(n)]
    total = sum(weights)
    segs, acc = [], [0.0] * 6
    done = [0] * 6
    for i, w in enumerate(weights):
        steps = []
        for a in range(6):
            acc[a] += axis_steps[a] * w / total
            st = int(round(acc[a])) - done[a]
            done[a] += st
            steps.append(st)
        segs.append((steps, slice_us, SEG_LAST if i == n - 1 else 0))
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ip")
    ap.add_argument("--sim", action="store_true", help="target is tiggy_sim.py on this PC (enables input tests)")
    ap.add_argument("--motion", action="store_true", help="allow tests that move the axes")
    args = ap.parse_args()
    if args.sim:
        args.motion = True

    results = []

    def test(name, fn, needs_motion=False, needs_sim=False):
        if (needs_motion and not args.motion) or (needs_sim and not args.sim):
            results.append((name, "skipped"))
            print("SKIP  %s" % name)
            return
        try:
            fn()
            results.append((name, "ok"))
            print("ok    %s" % name)
        except Exception as e:                      # one failed test must not hide the others
            results.append((name, "FAIL: %s" % e))
            print("FAIL  %s: %s" % (name, e))

    c = Client(args.ip)
    sim = Sim() if args.sim else None
    info = {}
    stream_no = [10]

    def new_stream():
        stream_no[0] = (stream_no[0] + 1) & 0xFF
        c.flush(stream_no[0])
        return stream_no[0]

    def recover():
        """Bring the device back to IDLE between tests."""
        if sim:
            sim(cmd="set", estop=0, probe=0, limits=0, probe_z=None, loss=0, dup=0, reorder=0)
        c.io.update(misc=0, spindle=0, rpm=0, coolant=0, flags=0x03)
        c.send(3, struct.pack("<B3x", 0xFF))
        for n in range(30):
            s = c.pump(0.1)
            if s and s["state"] in (6, 7):
                c.send(7)
            elif s and s["state"] == 0 and not s["jog"]:
                return
            elif s and n == 10:
                c.send(4)               # still busy with something a failed test left behind: abort it
        raise AssertionError("device did not return to IDLE: %s" % c.status)


    # ------------------------------------------------------------------ tests
    def t_discover():
        d = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        d.settimeout(1.0)
        d.sendto(c.pkt(0x0B), (args.ip, UDP_MOTION_PORT))
        data, addr = d.recvfrom(512)
        p = Client.parse(data)
        assert p and p[0] == 0x25 and len(p[1]) >= 48, "bad discovery response"
        fw, caps = struct.unpack_from("<IB", p[1])
        assert caps & 0x40, "device does not advertise STREAM_V2"
        d.close()

    def t_handshake():
        c.connect()
        info.update(c.handshake())
        assert info["fw"] >= (1 << 24) | (2 << 16), "firmware %08X is older than 1.2.0" % info["fw"]
        assert info["caps"] & 0x40, "STREAM_V2 capability missing"
        assert info["buffer"] >= 32 and info["max_rate"] >= 20000

    def t_config():
        for a in range(3):
            c.cfg_set(0x0001 + a, 4, 200.0)
            c.cfg_set(0x0010 + a, 2, 40000)
            c.cfg_set(0x0020 + a, 2, 60000)
        assert abs(c.cfg_get(0x0001) - 200.0) < 1e-3 and c.cfg_get(0x0010) == 40000 and c.cfg_get(0x0020) == 60000
        # the tests below expect the device's own defaults for these, whatever a host set before
        c.cfg_set(0x0315, 1, 20)        # input debounce, ms
        c.cfg_set(0x0433, 0, 7)         # minimum PWM duty, %
        assert c.cfg_get(0x0315) == 20 and c.cfg_get(0x0433) == 7
        c.cfg_set(0x0315, 1, 5)
        c.cfg_set(0x0433, 0, 0)
        c.cfg_set(0x0314, 0, 0x3F)      # every limit input raises an alarm
        c.cfg_set(0x0432, 0, 0)         # spindle output on/off (the outputs test switches PWM on itself)


    def t_status():
        c.send_io()
        s = c.wait(lambda s: True, 1.0, "first status report")
        assert s["v12"], "status report has no 1.2 fields"
        n0 = c.status_count
        c.pump(1.0)
        rate = c.status_count - n0
        assert 12 <= rate <= 30, "expected ~20 status reports per second, got %d" % rate

    def t_stream_exact():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        segs = ramp_move([400, -250, 123, 0, 0, 0], 1.0)
        t0 = time.perf_counter()
        c.stream(st, segs)
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0 and s["next_seg"] == len(segs), 5.0, "move to finish")
        took = time.perf_counter() - t0
        d = [s["pos"][i] - p0[i] for i in range(6)]
        assert d == [400, -250, 123, 0, 0, 0], "position delta %s" % d
        assert 0.95 <= took <= 1.6, "1.0 s move took %.2f s" % took
        assert not (s["sflags"] & 2), "underrun reported"

    def t_substep():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        # 333.3 steps/s on X and -41.7 steps/s on Y for 3 s, in 5 ms slices of
        # 1/256-step units: 426.67 and -53.33 units per slice.
        segs, ax, ay = [], 0.0, 0.0
        for i in range(600):
            ax += 333.3333 * 0.005 * 256
            ay -= 41.6667 * 0.005 * 256
            qx, qy = int(round(ax)), int(round(ay))
            ax -= qx
            ay -= qy
            segs.append(([qx, qy, 0, 0, 0, 0], 5000, SEG_SUBSTEP | (SEG_LAST if i == 599 else 0)))
        c.stream(st, segs)
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0 and s["next_seg"] == len(segs), 8.0, "move to finish")
        dx, dy = s["pos"][0] - p0[0], s["pos"][1] - p0[1]
        assert abs(dx - 1000) <= 1 and abs(dy + 125) <= 1, "sub-step move ended at X%+d Y%+d (expected +1000 -125)" % (dx, dy)
        c.stream(new_stream(), ramp_move([-dx, -dy, 0, 0, 0, 0], 0.6))
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "return move")

    def t_stream_loss():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        segs = ramp_move([-400, 250, -123, 0, 0, 0], 1.5)
        rng = random.Random(7)
        pkts = c.stream(st, segs, drop=lambda i: rng.random() < 0.3)   # lose 30% of the packets
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0 and s["next_seg"] == len(segs), 6.0, "move to finish")
        d = [s["pos"][i] - p0[i] for i in range(6)]
        assert d == [-400, 250, -123, 0, 0, 0], "position delta %s after packet loss (%d packets sent)" % (d, pkts)

    def t_stream_duplicates():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        segs = ramp_move([300, 0, 0, 0, 0, 0], 0.5)
        body = struct.pack("<BBBB", 8, 1, st, 0)
        for i in range(8):
            body += SEG.pack(*segs[i][0], segs[i][1], 0, 0, 0, i, SEG_TIMED, 0)
        for _ in range(4):
            c.send(1, body)                                         # same packet four times
        c.pump(0.2)
        c.stream(st, segs)                                          # then the whole move again from 0
        s = c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "move to finish")
        assert s["pos"][0] - p0[0] == 300, "duplicates changed the distance: %d" % (s["pos"][0] - p0[0])
        c.stream(new_stream(), ramp_move([-300, 0, 0, 0, 0, 0], 0.5))
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "return move")

    def t_stale_stream():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        body = struct.pack("<BBBB", 1, 1, (st + 5) & 0xFF, 0) + SEG.pack(500, 0, 0, 0, 0, 0, 100000, 0, 0, 0, 0, SEG_TIMED | SEG_LAST, 0)
        c.send(1, body)
        s = c.pump(0.4)
        assert s["pos"] == p0 and s["next_seg"] == 0, "segment for another stream was executed"

    def t_flush_stops():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        c.stream(st, [([40, 0, 0, 0, 0, 0], 10000, 0)] * 100)       # 1 s of motion, 4000 steps
        c.wait(lambda s: s["state"] == 1, 2.0, "motion to start")
        c.pump(0.2)
        c.flush((st + 1) & 0xFF, mode=0)
        stream_no[0] = (st + 1) & 0xFF
        s = c.wait(lambda s: s["state"] == 0, 2.0, "stop after flush")
        moved = s["pos"][0] - p0[0]
        assert 0 < moved < 4000, "flush did not stop the move (moved %d)" % moved
        p1 = s["pos"]
        assert c.pump(0.4)["pos"] == p1, "axis kept moving after flush"
        c.stream(new_stream(), ramp_move([-moved, 0, 0, 0, 0, 0], 0.6))
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "return move")

    def t_jog():
        recover()
        p0 = c.pump(0.1)["pos"]
        jog = struct.pack("<BbBBI", 1, 1, 1, 0, 4000)               # Y+, deadman, 4000 steps/s
        t_end = time.perf_counter() + 0.6
        peak = 0
        while time.perf_counter() < t_end:
            c.send(2, jog)
            s = c.pump(0.08)
            peak = max(peak, abs(s["feed"]))
        assert s["state"] == 3 and s["jog"] & 2, "not in JOG state: %s" % s
        c.send(3, struct.pack("<B3x", 1))
        s = c.wait(lambda s: s["state"] == 0, 2.0, "jog to stop")
        moved = s["pos"][1] - p0[1]
        assert 1200 < moved < 3200, "jog moved %d steps" % moved
        assert 3000 <= peak <= 4100, "jog peak rate %d" % peak
        # deadman: start a jog and then stay silent on the jog channel
        c.send(2, jog)
        c.pump(0.2)
        s = c.wait(lambda s: s["state"] == 0, 2.0, "deadman stop")
        back = c.pump(0.1)["pos"][1] - p0[1]
        c.stream(new_stream(), ramp_move([0, -back, 0, 0, 0, 0], 0.8))
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "return move")

    def t_outputs():
        recover()
        c.cfg_set(0x0432, 0, 1)
        c.cfg_set(0x0431, 2, 10000)
        c.io.update(misc=0x03, spindle=1, rpm=2500, coolant=1, flags=0x07)
        c.send_io()
        s = c.wait(lambda s: s["spindle"] == 1 and s["misc_out"] == 3 and s["coolant"] == 1, 1.0, "outputs on")
        if sim:
            o = sim(cmd="get")["outputs"]
            assert abs(o["duty"] - 0.25) < 0.01 and o["charge_pump"], "spindle duty/charge pump: %s" % o
        c.io.update(misc=0, spindle=0, rpm=0, coolant=0, flags=0x03)
        c.send_io()
        c.wait(lambda s: s["spindle"] == 0 and s["misc_out"] == 0, 1.0, "outputs off")

    def t_estop_reset():
        recover()
        c.io.update(spindle=1, rpm=1000, misc=1)
        c.send_io()
        c.pump(0.2)
        c.send(4)
        s = c.wait(lambda s: s["state"] == 7, 1.0, "ESTOP state")
        s = c.pump(0.3)
        assert s["spindle"] == 0 and s["misc_out"] == 0, "outputs still on in E-stop"
        c.io.update(spindle=0, rpm=0, misc=0)
        c.send(7)
        c.wait(lambda s: s["state"] == 0, 1.0, "IDLE after reset")

    def t_estop_input():
        recover()
        sim(cmd="set", estop=1)
        c.wait(lambda s: s["state"] == 7 and s["estop_in"] == 1, 1.0, "ESTOP from input")
        c.send(7)
        assert c.pump(0.3)["state"] == 7, "reset accepted while the E-stop input is active"
        sim(cmd="set", estop=0)
        c.send(7)
        c.wait(lambda s: s["state"] == 0, 1.0, "IDLE after reset")

    def t_home():
        recover()
        c.cfg_set(0x0020, 2, 60000)
        payload = struct.pack("<BBBB18I6B2x", 0x01, 0x00, 1, 0, *([3000] * 6 + [300] * 6 + [200] * 6 + list(range(6))))
        c.send(8, payload)
        c.wait(lambda s: s["state"] == 4, 1.0, "HOMING state")
        s = c.wait(lambda s: s["state"] != 4, 30.0, "homing to finish")
        assert s["state"] == 0 and s["homed"] & 1, "homing failed: %s" % s
        assert s["pos"][0] == 0, "X not zero after homing: %d" % s["pos"][0]
        assert s["limits"] & 1 == 0, "axis left on the switch"

    def t_probe():
        recover()
        st = new_stream()
        p0 = c.pump(0.1)["pos"]
        sim(cmd="set", probe_z=sim(cmd="get")["phys"][2] - 300)
        c.stream(st, [([0, 0, -10, 0, 0, 0], 10000, SEG_PROBE)] * 80)   # 800 steps down at 1000 steps/s
        s = c.wait(lambda s: s["sflags"] & 1, 3.0, "probe contact")
        s = c.pump(0.3)
        travel = p0[2] - s["pos"][2]
        assert 295 <= travel <= 320, "probe stopped after %d steps (plate at 300)" % travel
        assert s["state"] == 0 and s["queued_ms"] == 0, "queue not cleared after probe contact"
        sim(cmd="set", probe_z=None)
        c.stream(new_stream(), ramp_move([0, 0, travel, 0, 0, 0], 0.5))
        c.wait(lambda s: s["state"] == 0 and s["queued_ms"] == 0, 4.0, "retract")

    def t_limit_alarm():
        recover()
        st = new_stream()
        c.stream(st, [([20, 0, 0, 0, 0, 0], 10000, 0)] * 100)
        c.wait(lambda s: s["state"] == 1, 2.0, "motion to start")
        sim(cmd="set", limits=1)
        s = c.wait(lambda s: s["state"] == 6, 1.0, "ALARM on limit")
        assert s["alarm"] == 1, "alarm code %d" % s["alarm"]
        sim(cmd="set", limits=0)
        c.send(7)
        c.wait(lambda s: s["state"] == 0, 1.0, "IDLE after reset")

    def t_watchdog():
        recover()
        c.io.update(spindle=1, rpm=500)
        c.send_io()
        c.pump(0.3)
        time.sleep(1.6)                              # go silent: no heartbeat at all
        c.io.update(spindle=0, rpm=0)
        s = c.wait(lambda s: s["state"] == 6, 2.0, "ALARM after host silence")
        assert s["alarm"] == 31 and s["spindle"] == 0, "watchdog: alarm %d spindle %d" % (s["alarm"], s["spindle"])
        c.send(7)
        c.wait(lambda s: s["state"] == 0, 1.0, "IDLE after reset")

    test("discovery", t_discover)
    test("handshake", t_handshake)
    if not c.tcp:
        print("cannot continue without a control connection")
        return 1
    test("config set/get", t_config)
    test("status reports", t_status)
    test("timed stream: exact position and duration", t_stream_exact, needs_motion=True)
    test("timed stream: fractional (sub-step) segments", t_substep, needs_motion=True)
    test("timed stream: survives packet loss", t_stream_loss, needs_motion=True)
    test("timed stream: duplicates ignored", t_stream_duplicates, needs_motion=True)
    test("segments for a stale stream ignored", t_stale_stream)
    test("flush stops a running move", t_flush_stops, needs_motion=True)
    test("jog: ramps, state, deadman", t_jog, needs_motion=True)
    test("outputs, spindle PWM, charge pump", t_outputs)
    test("E-stop packet and reset", t_estop_reset)
    test("E-stop input blocks reset", t_estop_input, needs_sim=True)
    test("homing", t_home, needs_sim=True)
    test("probe", t_probe, needs_sim=True)
    test("limit switch alarm", t_limit_alarm, needs_sim=True)
    test("host watchdog", t_watchdog)

    try:
        recover()
    except AssertionError:
        pass
    failed = [r for r in results if r[1].startswith("FAIL")]
    print("\n%d passed, %d failed, %d skipped" % (
        sum(1 for r in results if r[1] == "ok"), len(failed), sum(1 for r in results if r[1] == "skipped")))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
