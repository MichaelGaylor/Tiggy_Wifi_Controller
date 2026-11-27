#!/usr/bin/env python3
"""
Tiggy firmware simulator - the controller's real motion code on a PC.

tiggy_sim.py is an independent model of how a controller should behave.
This program instead runs the firmware's own motion core and step engine
(firmware/main/motion, compiled for the PC by firmware/test/pc/build.bat or
build.sh) behind the same UDP and TCP ports as a board, with a simulated
machine attached: axes with limit switches, a probe, an E-stop input.

Use it to test firmware changes and host software without hardware:

    firmware\\test\\pc\\build.bat
    python tools/firmware_sim.py
    python tools/protocol_conformance.py --sim --motion 127.0.0.1

The automation port (127.0.0.1:58500, one JSON object per line) is the same
as tiggy_sim.py's:  {"cmd":"get"}  {"cmd":"set","estop":1}  ...

With --encoder there is a spindle with a sensor on it as well, for testing
screw cutting.  It follows the spindle output unless told otherwise:

    {"cmd":"spindle","rpm":300,"accel":2000}     turn at 300 rpm whatever the output says
    {"cmd":"spindle","follow":1}                 follow the spindle output again
    {"cmd":"encoder","mode":"pulse","ppr":1}     change the sensor
    {"cmd":"fit","lines":600,"index":1}          an encoder of 600 lines on the four-axis layout's
                                                 terminals, whatever the settings take it for
                                                 ("b_open":1: its B signal not connected; "lines":0: none)
    {"cmd":"trace","axis":2}                     record the steps of Z with the spindle angle at each
    {"cmd":"trace_read"}
"""

import argparse
import ctypes
import json
import os
import random
import socket
import struct
import sys
import threading
import time

AXES = 6
UDP_MOTION_PORT, TCP_PORT = 58427, 58429
STATE_NAMES = ["IDLE", "RUN", "HOLD", "JOG", "HOMING", "PROBING", "ALARM", "ESTOP", "DISCONNECTED"]
# Whole-number settings reported on the automation port (rates, timing, polarity, homing, spindle)
CFG_INT_KEYS = (list(range(0x0010, 0x0016)) + list(range(0x0020, 0x0026)) + list(range(0x0100, 0x0104)) +
                list(range(0x0300, 0x0303)) + list(range(0x0310, 0x0316)) + list(range(0x0320, 0x0324)) +
                [0x0400] + list(range(0x0430, 0x0434)))


class Info(ctypes.Structure):
    _fields_ = [("phys", ctypes.c_double * AXES), ("pos", ctypes.c_int32 * AXES),
                ("state", ctypes.c_int32), ("alarm", ctypes.c_int32),
                ("misc", ctypes.c_int32), ("spindle", ctypes.c_int32), ("rpm", ctypes.c_int32),
                ("coolant", ctypes.c_int32), ("charge_pump", ctypes.c_int32), ("drivers", ctypes.c_int32),
                ("io", ctypes.c_int32), ("duty", ctypes.c_double),
                ("queued_us", ctypes.c_uint32), ("queue_count", ctypes.c_uint32),
                ("min_gap_us", ctypes.c_uint32 * AXES), ("steps", ctypes.c_uint64 * AXES),
                ("isr_calls", ctypes.c_uint64), ("feed", ctypes.c_uint32), ("busy", ctypes.c_int32),
                ("sync", ctypes.c_int32), ("spindle_turns", ctypes.c_double), ("spindle_rpm", ctypes.c_double),
                ("tracked_rpm_milli", ctypes.c_uint32), ("index_count", ctypes.c_uint32)]

TRACE_MAX = 200000


def load_library(path):
    here = os.path.dirname(os.path.abspath(__file__))
    build = os.path.join(here, "..", "firmware", "test", "pc", "build")
    candidates = [path] if path else [os.path.join(build, "fwsim.dll"), os.path.join(build, "libfwsim.so")]
    for c in candidates:
        if c and os.path.exists(c):
            lib = ctypes.CDLL(os.path.abspath(c))
            lib.fws_run.argtypes = [ctypes.c_uint64]
            lib.fws_now.restype = ctypes.c_uint64
            lib.fws_init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint32]
            lib.fws_udp.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint16]
            lib.fws_tcp_session.argtypes = [ctypes.c_uint32]
            lib.fws_tcp.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
            lib.fws_outbox.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32),
                                       ctypes.POINTER(ctypes.c_uint16)]
            lib.fws_set_inputs.argtypes = [ctypes.c_int] * 5
            lib.fws_set_limits.argtypes = [ctypes.c_double * AXES, ctypes.c_double * AXES]
            lib.fws_set_probe_z.argtypes = [ctypes.c_int, ctypes.c_double]
            lib.fws_set_phys.argtypes = [ctypes.c_double * AXES]
            lib.fws_get.argtypes = [ctypes.POINTER(Info)]
            lib.fws_cfg.argtypes = [ctypes.c_uint16]
            lib.fws_cfg.restype = ctypes.c_uint32
            lib.fws_log.argtypes = [ctypes.c_char_p, ctypes.c_int]
            lib.fws_encoder.argtypes = [ctypes.c_int] * 3
            lib.fws_fit.argtypes = [ctypes.c_int] * 3
            lib.fws_restart_wanted.restype = ctypes.c_int
            lib.fws_spindle.argtypes = [ctypes.c_int] + [ctypes.c_double] * 4
            lib.fws_trace.argtypes = [ctypes.c_int]
            lib.fws_trace_read.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
                                           ctypes.POINTER(ctypes.c_int32), ctypes.c_int]
            return lib
    sys.exit("The firmware library was not found. Build it first:\n"
             "  firmware\\test\\pc\\build.bat   (Windows)   or   firmware/test/pc/build.sh")


def claim(kind, address):
    """A socket on one of the controller's ports - which no other simulator on this PC may
    have: of two, each would get part of what a host sends (on Linux the datagrams go to one
    and the connections to the other), and tests fail for no reason they can show."""
    s = socket.socket(socket.AF_INET, kind)
    if kind == socket.SOCK_STREAM and os.name != "nt":
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)     # (there: a port just closed can be taken again, no more)
    try:
        s.bind(address)
    except OSError:
        sys.exit("Port %d is in use: another simulator is running on this PC. Close it first." % address[1])
    return s


def ip_to_int(ip):
    return struct.unpack("<I", socket.inet_aton(ip))[0]


def int_to_ip(value):
    return socket.inet_ntoa(struct.pack("<I", value))


class FirmwareSim:
    def __init__(self, args):
        self.args = args
        self.lib = load_library(args.lib)
        self.lock = threading.RLock()
        self.t0 = time.perf_counter()
        fw = [int(x) for x in args.fw.split(".")]
        self.lib.fws_init(1 if args.io_module else 0, args.encoder, (fw[0] << 24) | (fw[1] << 16) | fw[2])
        if args.encoder and (args.encoder_mode != "quad" or args.no_index):
            self.lib.fws_encoder(2 if args.encoder_mode == "pulse" else 1, args.encoder, 0 if args.no_index else 1)
        self.spindle = dict(follow=1, rpm=0.0, accel=3000.0, ripple=0.0, ripple_hz=0.0)
        self.base_us = self.lib.fws_now()
        self.force = {"estop": 0, "probe": 0, "limits": 0, "misc_in": 0, "io_in": 0}
        self.lim_lo = [-4000.0] * AXES
        self.lim_hi = [200000.0] * AXES
        self.probe_z = None
        self.udp = None
        self.tcp_owner = None
        self.tcp_ip = None
        self.last_host_rx = 0.0
        self.hold_reorder = None
        self.stats = dict(rx=0, rx_dropped=0, tx=0)
        self.log_lines = []

    # ------------------------------------------------------------------ util
    def log(self, msg):
        line = "%s %s" % (time.strftime("%H:%M:%S"), msg)
        self.log_lines.append(line)
        del self.log_lines[:-400]
        if not self.args.quiet:
            print(line, flush=True)

    def advance(self):
        """Run the firmware up to the present.  Call with the lock held."""
        self.lib.fws_run(self.base_us + int((time.perf_counter() - self.t0) * 1e6))
        buf = ctypes.create_string_buffer(512)
        ip, port = ctypes.c_uint32(), ctypes.c_uint16()
        while True:
            n = self.lib.fws_outbox(buf, len(buf), ctypes.byref(ip), ctypes.byref(port))
            if n <= 0:
                break
            if self.args.loss and random.random() < self.args.loss:
                continue
            try:
                self.udp.sendto(buf.raw[:n], (int_to_ip(ip.value), port.value))
                self.stats["tx"] += 1
            except OSError:
                pass
        text = ctypes.create_string_buffer(4096)
        if self.lib.fws_log(text, len(text)) > 0:
            for line in text.value.decode("latin1").splitlines():
                self.log("fw: " + line)

    def apply_inputs(self):
        f = self.force
        self.lib.fws_set_inputs(f["estop"], f["probe"], f["limits"], f["misc_in"], f["io_in"])
        self.lib.fws_set_limits((ctypes.c_double * AXES)(*self.lim_lo), (ctypes.c_double * AXES)(*self.lim_hi))
        self.lib.fws_set_probe_z(0 if self.probe_z is None else 1, 0.0 if self.probe_z is None else self.probe_z)

    # --------------------------------------------------------------- threads
    def run_udp(self):
        while True:
            try:
                data, addr = self.udp.recvfrom(2048)
            except OSError:
                continue
            self.stats["rx"] += 1
            if self.args.loss and random.random() < self.args.loss:
                self.stats["rx_dropped"] += 1
                continue
            batch = [(data, addr)]
            if self.args.reorder and random.random() < self.args.reorder and self.hold_reorder is None:
                self.hold_reorder = (data, addr)
                continue
            if self.hold_reorder is not None:
                batch.append(self.hold_reorder)
                self.hold_reorder = None
            if self.args.dup and random.random() < self.args.dup:
                batch.append((data, addr))
            with self.lock:
                self.advance()
                for d, a in batch:
                    if a[0] == self.tcp_ip:
                        self.last_host_rx = time.perf_counter()
                    self.lib.fws_udp(d, len(d), ip_to_int(a[0]), a[1])
                self.advance()

    def run_clock(self):
        while True:
            time.sleep(0.001)
            with self.lock:
                self.advance()

    def run_tcp(self):
        srv = self.tcp_srv
        srv.listen(2)
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=self.serve_tcp, args=(conn, addr), daemon=True).start()

    def serve_tcp(self, conn, addr):
        # One host at a time.  A second connection is turned away while the
        # host that owns the controller is still talking to it; it may take
        # over only when that host has gone quiet.
        if self.tcp_owner is not None and time.perf_counter() - self.last_host_rx < 3.0:
            self.log("TCP connect from %s refused: in use by %s" % (addr[0], self.tcp_ip))
            conn.close()
            return
        self.log("TCP connect from %s" % addr[0])
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(2.0)
        me = threading.current_thread()
        with self.lock:
            self.tcp_owner = me
            self.tcp_ip = addr[0]
            self.last_host_rx = time.perf_counter()
            self.advance()
            self.lib.fws_tcp_session(ip_to_int(addr[0]))
        buf = b""
        resp = ctypes.create_string_buffer(512)
        last_rx = time.perf_counter()
        try:
            while self.tcp_owner is me:
                try:
                    chunk = conn.recv(1024)
                except socket.timeout:
                    if time.perf_counter() - last_rx > 30.0:
                        raise ConnectionError("host silent for 30 s")
                    continue
                if not chunk:
                    break
                last_rx = self.last_host_rx = time.perf_counter()
                buf += chunk
                while len(buf) >= 2:
                    n = struct.unpack_from("<H", buf)[0]
                    if n == 0 or n > 256:
                        raise ConnectionError("bad frame length %d" % n)
                    if len(buf) < 2 + n:
                        break
                    frame, buf = buf[2:2 + n], buf[2 + n:]
                    with self.lock:
                        self.advance()
                        rn = self.lib.fws_tcp(frame, len(frame), resp, len(resp))
                        restart = self.lib.fws_restart_wanted()
                    if rn > 0:
                        conn.sendall(struct.pack("<H", rn) + resp.raw[:rn])
                    if restart:
                        # a controller restarts to take up a new layout or spindle sensor
                        time.sleep(1.0)
                        raise ConnectionError("restarting to take up the new settings")
        except (OSError, ConnectionError) as e:
            self.log("TCP session ended: %s" % e)
        finally:
            conn.close()
            with self.lock:
                if self.tcp_owner is me:
                    self.tcp_owner = None
                    self.tcp_ip = None
                    self.advance()

                    self.lib.fws_tcp_session(0)
                    self.log("TCP disconnect from %s" % addr[0])

    # ---------------------------------------------------- automation channel
    def run_control(self):
        srv = self.ctl_srv
        srv.listen(4)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=self.serve_control, args=(conn,), daemon=True).start()

    def serve_control(self, conn):
        f = conn.makefile("rwb")
        try:
            for line in f:
                try:
                    resp = self.control(json.loads(line.decode("utf-8")))
                except Exception as e:           # report the problem to the test, keep serving
                    resp = {"error": repr(e)}
                f.write((json.dumps(resp) + "\n").encode("utf-8"))
                f.flush()
        except OSError:
            pass
        finally:
            conn.close()

    def control(self, req):
        cmd = req.get("cmd", "get")
        with self.lock:
            self.advance()
            if cmd == "set":
                for k in self.force:
                    if k in req:
                        self.force[k] = int(req[k])
                if "probe_z" in req:
                    self.probe_z = None if req["probe_z"] is None else float(req["probe_z"])
                if "lim_lo" in req:
                    self.lim_lo = [float(v) for v in req["lim_lo"]]
                if "lim_hi" in req:
                    self.lim_hi = [float(v) for v in req["lim_hi"]]
                if "phys" in req:
                    self.lib.fws_set_phys((ctypes.c_double * AXES)(*[float(v) for v in req["phys"]]))
                for k in ("loss", "dup", "reorder"):
                    if k in req:
                        setattr(self.args, k, float(req[k]))
                self.apply_inputs()
            elif cmd == "clear":
                self.lib.fws_clear_stats()
            elif cmd == "spindle":
                sp = self.spindle
                if "rpm" in req and "follow" not in req:
                    sp["follow"] = 0
                for k in sp:
                    if k in req:
                        sp[k] = float(req[k]) if k != "follow" else int(req[k])
                self.lib.fws_spindle(sp["follow"], sp["rpm"], sp["accel"], sp["ripple"], sp["ripple_hz"])
            elif cmd == "encoder":
                self.lib.fws_encoder(2 if req.get("mode", "quad") == "pulse" else 1, int(req.get("ppr", 0)),
                                     int(req.get("index", 1)))
            elif cmd == "fit":
                self.lib.fws_fit(int(req.get("lines", 0)), int(req.get("index", 1)), int(req.get("b_open", 0)))
            elif cmd == "trace":
                self.lib.fws_trace(int(req.get("axis", 2)))
            elif cmd == "trace_read":
                t = (ctypes.c_double * TRACE_MAX)()
                turns = (ctypes.c_double * TRACE_MAX)()
                pos = (ctypes.c_int32 * TRACE_MAX)()
                n = self.lib.fws_trace_read(t, turns, pos, TRACE_MAX)
                return {"t": list(t[:n]), "turns": list(turns[:n]), "pos": list(pos[:n])}
            elif cmd == "log":
                return {"log": list(self.log_lines)}
            elif cmd == "hold":
                self.lib.fws_feed_hold()
            elif cmd == "resume":
                self.lib.fws_feed_resume()
            elif cmd == "home":
                return {"ok": bool(self.lib.fws_home(int(req.get("mask", 1))))}
            elif cmd == "legacy":
                # one old-form segment: steps[6], entry speed, exit speed, acceleration (steps/s, steps/s^2)
                seg = struct.pack("<6iIIIIHBB", *[int(v) for v in req["steps"]], int(req.get("duration_us", 0)),

                                  int(float(req.get("entry", 0)) ** 2 * 1000), int(float(req.get("exit", 0)) ** 2 * 1000),
                                  int(float(req.get("accel", 5000)) * 100), int(req.get("id", 0)), int(req.get("flags", 0)), 0)
                return {"ok": bool(self.lib.fws_legacy_push(seg))}
            info = Info()
            self.lib.fws_get(ctypes.byref(info))
            return {
                "state": STATE_NAMES[info.state] if 0 <= info.state < len(STATE_NAMES) else str(info.state),
                "alarm": info.alarm, "pos": list(info.pos), "phys": list(info.phys),
                "moving": bool(info.feed), "busy": bool(info.busy), "feed": info.feed,
                "queue": info.queue_count, "queued_ms": round(info.queued_us / 1000.0, 1),
                "outputs": dict(misc=info.misc, spindle=info.spindle, rpm=info.rpm, duty=round(info.duty, 4),
                                coolant=info.coolant, charge_pump=bool(info.charge_pump),
                                drivers=bool(info.drivers), io=info.io),
                "min_gap_us": [None if g == 0xFFFFFFFF else g for g in info.min_gap_us],
                "steps": list(info.steps), "isr_calls": info.isr_calls,
                "sync": info.sync,
                "spindle": dict(turns=info.spindle_turns, rpm=round(info.spindle_rpm, 3),
                                tracked_rpm=info.tracked_rpm_milli / 1000.0, index_count=info.index_count),
                "force": dict(self.force), "stats": dict(self.stats),
                "cfg": {("0x%04X" % k): self.lib.fws_cfg(k) for k in CFG_INT_KEYS},
            }


def main():
    ap = argparse.ArgumentParser(description="Tiggy firmware simulator (the firmware's motion code on a PC)")
    ap.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    ap.add_argument("--control-port", type=int, default=58500, help="local automation port")
    ap.add_argument("--loss", type=float, default=0.0, help="fraction of UDP packets to drop (both directions)")
    ap.add_argument("--dup", type=float, default=0.0, help="fraction of received UDP packets to duplicate")
    ap.add_argument("--reorder", type=float, default=0.0, help="fraction of received UDP packets to delay behind the next")
    ap.add_argument("--fw", default="1.2.0", help="firmware version to report")
    ap.add_argument("--io-module", action="store_true", help="run as an I/O expansion module")
    ap.add_argument("--encoder", type=int, default=0, metavar="PPR",
                    help="simulate a spindle with a sensor: an encoder of PPR lines, or PPR pulses per revolution")
    ap.add_argument("--encoder-mode", choices=("quad", "pulse"), default="quad",
                    help="quad: quadrature encoder (default); pulse: a single pulse sensor")
    ap.add_argument("--no-index", action="store_true", help="quadrature encoder without an index pulse")
    ap.add_argument("--lib", default="", help="path of fwsim.dll / libfwsim.so")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if os.name == "nt":
        ctypes.windll.winmm.timeBeginPeriod(1)      # 1 ms sleeps

    ports = (claim(socket.SOCK_DGRAM, (args.bind, UDP_MOTION_PORT)), claim(socket.SOCK_STREAM, (args.bind, TCP_PORT)),
             claim(socket.SOCK_STREAM, ("127.0.0.1", args.control_port)))
    sim = FirmwareSim(args)
    sim.apply_inputs()
    sim.udp, sim.tcp_srv, sim.ctl_srv = ports
    for fn in (sim.run_udp, sim.run_tcp, sim.run_control, sim.run_clock):
        threading.Thread(target=fn, daemon=True).start()
    sim.log("Tiggy firmware simulator ready: TCP %d, UDP %d, control 127.0.0.1:%d"
            % (TCP_PORT, UDP_MOTION_PORT, args.control_port))
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
