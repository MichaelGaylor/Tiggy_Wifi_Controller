#!/usr/bin/env python3
"""
Tiggy controller simulator.

Speaks the same network protocol as the controller firmware (protocol 1.2),
so host software - the Mach3 plugin, the LinuxCNC driver, the test tools -
can be developed and tested on a PC with no board attached.  It models:

  * the TCP control channel (handshake, config get/set, ping)
  * the UDP motion channel (timed + sequenced segments, stream flush, jog
    with ramps and deadman, homing against simulated switches, E-stop,
    reset, feed hold, output control with host watchdog, discovery)
  * status reports at 20 Hz
  * a simple machine: every axis has a home/limit switch at each end of its
    travel and there is a probe plate under the Z axis

It can also drop, duplicate and reorder packets to prove that a host copes
with a bad WiFi link, and it records every move it executes so a test can
check distance, duration, peak velocity and peak acceleration.

    python tiggy_sim.py                 run on all interfaces
    python tiggy_sim.py --loss 0.1      drop 10% of packets in both directions

A second, local-only TCP port (default 58500) takes one JSON object per
line for test automation, e.g.  {"cmd":"get"}  {"cmd":"set","estop":1}.
"""

import argparse
import json
import math
import random
import socket
import struct
import threading
import time
from collections import deque

# --------------------------------------------------------------------------
# Protocol constants (mirror protocol/wifi_cnc_protocol.h)
# --------------------------------------------------------------------------
MAGIC = 0x574D4333
PROTO_VERSION = 1
AXES = 6
UDP_MOTION_PORT, UDP_STATUS_PORT, TCP_PORT = 58427, 58428, 58429

PKT_MOTION, PKT_JOG, PKT_JOG_STOP, PKT_ESTOP, PKT_HOLD, PKT_RESUME = 1, 2, 3, 4, 5, 6
PKT_RESET, PKT_HOME, PKT_IO, PKT_FLUSH, PKT_DISCOVER = 7, 8, 9, 0x0A, 0x0B
PKT_STATUS, PKT_HOME_COMPLETE, PKT_PROBE_RESULT, PKT_ACK, PKT_DISCOVER_RESP = 0x20, 0x22, 0x23, 0x24, 0x25
PKT_HS_REQ, PKT_HS_RESP, PKT_CFG_GET, PKT_CFG_SET, PKT_CFG_RESP, PKT_CFG_SAVE = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45
PKT_PING, PKT_PONG = 0x50, 0x51

ST_IDLE, ST_RUN, ST_HOLD, ST_JOG, ST_HOMING, ST_PROBING, ST_ALARM, ST_ESTOP = range(8)
STATE_NAMES = ["IDLE", "RUN", "HOLD", "JOG", "HOMING", "PROBING", "ALARM", "ESTOP"]

ALARM_NONE, ALARM_LIMIT_X, ALARM_ESTOP, ALARM_HOST_LOST, ALARM_HOME_FAIL = 0, 1, 20, 31, 40

SEG_PROBE, SEG_LAST, SEG_TIMED, SEG_SUBSTEP = 0x04, 0x02, 0x10, 0x20
SUBSTEP_SCALE = 256
MOTION_SEQUENCED = 0x01
IO_WATCHDOG, IO_REPLY_SRC, IO_CHARGE_PUMP, IO_LIMIT_OVERRIDE = 1, 2, 4, 8
JOG_DEADMAN = 1
HOME_PARAMS = 1
STREAM_PROBE_TRIPPED, STREAM_UNDERRUN, STREAM_FLUSHING = 1, 2, 4
CAP_CHARGE_PUMP, CAP_MISC_OUT, CAP_MISC_IN, CAP_SPINDLE_PWM, CAP_STREAM_V2 = 1, 2, 4, 8, 0x40
CAP_SPINDLE_ENCODER, CAP_IO_MODULE = 0x10, 0x20

HOST_WATCHDOG_MS, JOG_DEADMAN_MS = 1000, 500
BUFFER_CAPACITY = 127
START_QUEUE_MS = 100          # begin playback once this much motion is queued...
START_MAX_WAIT_MS = 150       # ...or this long after the first segment arrived
MIN_STEP_INTERVAL_US = 10

HDR = struct.Struct("<IBBHIIH")
SEG = struct.Struct("<6iIIIIHBB")
STATUS = struct.Struct("<6iHH8BHIi4BHHIHHHBBBBH")

# config keys -> (type, default)   types: 0=u8 1=u16 2=u32 3=i32 4=float 5=str
CFG_TYPES = {}
for _a in range(AXES):
    CFG_TYPES[0x0001 + _a] = (4, 800.0)      # steps per mm
    CFG_TYPES[0x0010 + _a] = (2, 20000)      # max rate, steps/s
    CFG_TYPES[0x0020 + _a] = (2, 5000)       # acceleration, steps/s^2
CFG_TYPES.update({
    0x0100: (1, 3), 0x0101: (1, 2), 0x0102: (1, 255), 0x0103: (1, 50),
    0x0300: (0, 0), 0x0301: (0, 0), 0x0302: (0, 0), 0x0310: (0, 0x3F), 0x0311: (0, 0x3F),
    0x0312: (0, 0), 0x0313: (0, 0), 0x0314: (0, 0x3F), 0x0315: (1, 5),
    0x0320: (0, 0), 0x0321: (2, 5000), 0x0322: (2, 500), 0x0323: (2, 100),
    0x0400: (1, 0), 0x0430: (1, 1000), 0x0431: (2, 24000), 0x0432: (0, 0), 0x0433: (0, 0),
})
# GPIO assignments of the 6-axis Tiggy Pro board (0xFF = not fitted), keys 0x0500..
_PINS = [1, 2, 42, 41, 40, 39,          # step X..C
         3, 4, 38, 33, 34, 35,          # direction X..C
         8,                             # enable
         5, 6, 7, 15, 16, 17,           # limit / home X..C
         14, 13, 9, 48, 10, 11, 12]     # probe, E-stop, spindle, LED, charge pump, misc out 0/1
for _i, _p in enumerate(_PINS):
    CFG_TYPES[0x0500 + _i] = (0, _p)
CFG_TYPES.update({0x0520: (0, 0xFF), 0x0521: (0, 0xFF), 0x0522: (0, 0xFF),      # spindle encoder
                  0x0530: (0, 18), 0x0531: (0, 21), 0x0532: (0, 36), 0x0533: (0, 0xFF)})    # misc inputs


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


_tx_seq = 0


def make_packet(ptype: int, payload: bytes = b"") -> bytes:
    global _tx_seq
    _tx_seq = (_tx_seq + 1) & 0xFFFFFFFF
    hdr = HDR.pack(MAGIC, PROTO_VERSION, ptype, len(payload), _tx_seq, int(time.perf_counter() * 1e6) & 0xFFFFFFFF, 0)
    c = crc16(hdr + payload)
    return hdr[:16] + struct.pack("<H", c) + payload


def parse_packet(data: bytes):
    """Returns (type, payload, sequence) or None when the packet is not valid."""
    if len(data) < HDR.size:
        return None
    magic, ver, ptype, plen, seq, _ts, crc = HDR.unpack_from(data)
    if magic != MAGIC or ver != PROTO_VERSION or len(data) < HDR.size + plen:
        return None
    body = data[:HDR.size + plen]
    if crc16(body[:16] + b"\0\0" + body[18:]) != crc:
        return None
    return ptype, body[HDR.size:], seq


class Segment:
    __slots__ = ("steps", "dur", "sid", "flags", "arrived")

    def __init__(self, steps, dur, sid, flags):
        self.steps, self.dur, self.sid, self.flags = steps, dur, sid, flags
        self.arrived = time.perf_counter()


class Sim:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.log_lines = deque(maxlen=400)
        self.cfg = {k: v[1] for k, v in CFG_TYPES.items()}
        self.cfg_extra = {}
        fw = [int(x) for x in getattr(args, "fw", "1.2.0").split(".")]
        self.fw_version = (fw[0] << 24) | (fw[1] << 16) | fw[2]
        self.io_module = bool(getattr(args, "io_module", False))
        self.encoder_ppr = int(getattr(args, "encoder", 0) or 0)
        self.io_out = 0
        self.spindle_turns = 0.0

        # machine model (steps)
        self.phys = [0.0] * AXES            # physical position, never zeroed
        self.origin = [0.0] * AXES          # reported position = phys - origin
        self.lim_lo = [-4000.0] * AXES      # switch closes at or below this
        self.lim_hi = [200000.0] * AXES     # ...and at or above this
        self.probe_z = None                 # probe closes when phys[2] <= this
        self.force = {"estop": 0, "probe": 0, "limits": 0, "misc_in": 0, "io_in": 0}

        # motion
        self.state = ST_IDLE
        self.alarm = ALARM_NONE
        self.queue = deque()
        self.cur = None                     # executing segment
        self.cur_pos = 0.0                  # microseconds into the current segment
        self.cur_base = [0.0] * AXES        # phys at the start of the current segment
        self.rate = 1.0                     # time scale (1 = real time)
        self.ramp = None                    # (direction, seconds) while ramping the time scale
        self.after_ramp = None
        self.stream_id = 0
        self.next_seg_id = 0
        self.stream_flags = 0
        self.stream_closed = False
        self.pending_stream = None
        self.first_queued = None
        self.exec_seg_id = 0
        self.feed_rate = 0.0

        # jog / homing
        self.jog_v = [0.0] * AXES
        self.jog_target = [0.0] * AXES
        self.jog_deadline = [None] * AXES
        self.home = None
        self.homed_mask = 0

        # outputs
        self.misc_out = 0
        self.spindle_state = 0
        self.spindle_rpm = 0
        self.spindle_duty = 0.0
        self.coolant = 0
        self.charge_pump = False
        self.io_flags = 0
        self.drivers_enabled = False

        # host tracking
        self.host_ip = None
        self.tcp_host = None
        self.status_port = UDP_STATUS_PORT
        self.last_host_rx = 0.0
        self.watchdog_armed = False
        self.udp = None

        # statistics and move recording
        self.stats = dict(rx=0, rx_bad=0, rx_dropped=0, rx_foreign=0, seg_accepted=0, seg_dup=0,
                          seg_gap=0, seg_full=0, seg_wrong_stream=0, underruns=0, estops=0,
                          watchdog_trips=0, flushes=0, tx_status=0)
        self.episodes = deque(maxlen=200)
        self.ep = None
        self.prev_seg_v = None
        self.hold_reorder = None

    # ------------------------------------------------------------------ util
    def log(self, msg):
        line = "%s %s" % (time.strftime("%H:%M:%S"), msg)
        self.log_lines.append(line)
        if not self.args.quiet:
            print(line, flush=True)

    def pos(self):
        return [int(round(self.phys[i] - self.origin[i])) for i in range(AXES)]

    def limit_mask(self):
        m = self.force["limits"]
        for i in range(AXES):
            if self.phys[i] <= self.lim_lo[i] or self.phys[i] >= self.lim_hi[i]:
                m |= 1 << i
        return m & 0x3F

    def probe_active(self):
        if self.force["probe"]:
            return True
        return self.probe_z is not None and self.phys[2] <= self.probe_z

    def moving(self):
        return self.cur is not None or any(abs(v) > 1e-9 for v in self.jog_v)

    def queued_ms(self):
        q = sum(s.dur for s in self.queue)
        if self.cur:
            q += max(0.0, self.cur.dur - self.cur_pos)
        return q / 1000.0

    # ------------------------------------------------------------- recording
    def ep_begin(self, kind):
        if self.ep is None:
            self.ep = dict(kind=kind, t0=time.perf_counter(), start=self.pos(), segs=0,
                           vmax=[0.0] * AXES, amax=[0.0] * AXES, planned_us=0)
            self.prev_seg_v = None

    def ep_end(self):
        if self.ep is not None:
            e = self.ep
            e["t1"] = time.perf_counter()
            e["duration"] = round(e["t1"] - e["t0"], 4)
            e["end"] = self.pos()
            e["delta"] = [e["end"][i] - e["start"][i] for i in range(AXES)]
            e["vmax"] = [round(v, 1) for v in e["vmax"]]
            e["amax"] = [round(a, 1) for a in e["amax"]]
            e["planned"] = round(e.pop("planned_us") / 1e6, 4)
            del e["t0"], e["t1"]
            self.episodes.append(e)
            self.ep = None

    def ep_segment(self, seg):
        e = self.ep
        if e is None:
            return
        e["segs"] += 1
        e["planned_us"] += seg.dur
        v = [seg.steps[i] * 1e6 / seg.dur for i in range(AXES)]
        for i in range(AXES):
            e["vmax"][i] = max(e["vmax"][i], abs(v[i]))
        if self.prev_seg_v is not None:
            pv, pd = self.prev_seg_v
            dt = (pd + seg.dur) / 2e6
            for i in range(AXES):
                e["amax"][i] = max(e["amax"][i], abs(v[i] - pv[i]) / dt)
        self.prev_seg_v = (v, seg.dur)

    # -------------------------------------------------------------- stopping
    def stop_stream(self, clear=True):
        """Stop segment playback at once, where it is."""
        self.cur = None
        self.cur_pos = 0.0
        self.rate = 1.0
        self.ramp = None
        self.after_ramp = None
        self.feed_rate = 0.0
        if clear:
            self.queue.clear()
            self.first_queued = None

    def stop_jog(self):
        for i in range(AXES):
            self.jog_v[i] = 0.0
            self.jog_target[i] = 0.0
            self.jog_deadline[i] = None

    def outputs_off(self):
        self.misc_out = 0
        self.spindle_state = 0
        self.spindle_rpm = 0
        self.spindle_duty = 0.0
        self.coolant = 0
        self.charge_pump = False

    def enter_fault(self, state, alarm, why):
        self.log("%s: %s" % (STATE_NAMES[state], why))
        self.stop_stream()
        self.stop_jog()
        self.home = None
        self.stream_closed = True
        self.outputs_off()
        self.drivers_enabled = False
        self.state = state
        self.alarm = alarm
        self.ep_end()
        if state == ST_ESTOP:
            self.stats["estops"] += 1

    def begin_ramp_down(self, then):
        """Decelerate playback by stretching time, then call 'then'."""
        if self.cur is None:
            then()
            return
        t_stop = 0.02
        for i in range(AXES):
            v = abs(self.cur.steps[i]) * 1e6 / self.cur.dur
            a = max(1.0, float(self.cfg[0x0020 + i]))
            t_stop = max(t_stop, v / a)
        self.ramp = (-1, min(t_stop, 2.0))
        self.after_ramp = then

    def apply_flush(self, new_stream):
        self.stop_stream()
        self.stream_id = new_stream
        self.next_seg_id = 0
        self.stream_flags = 0
        self.stream_closed = False
        self.pending_stream = None
        self.stats["flushes"] += 1
        if self.state in (ST_RUN, ST_HOLD):
            self.state = ST_IDLE
        self.ep_end()

    # ---------------------------------------------------------- UDP handlers
    def on_motion(self, payload):
        if len(payload) < 4:
            return
        count, flags, stream, _ = struct.unpack_from("<BBBB", payload)
        if count == 0 or count > 8 or len(payload) != 4 + count * SEG.size:
            self.stats["rx_bad"] += 1
            return
        if self.state in (ST_ESTOP, ST_ALARM, ST_JOG, ST_HOMING):
            return
        sequenced = bool(flags & MOTION_SEQUENCED)
        if sequenced and (stream != self.stream_id or self.stream_closed or self.pending_stream is not None):
            self.stats["seg_wrong_stream"] += count
            return
        for n in range(count):
            f = SEG.unpack_from(payload, 4 + n * SEG.size)
            steps, dur, sid, sflags = list(f[0:6]), f[6], f[10], f[11]
            if sequenced:
                if sid != self.next_seg_id:
                    behind = ((self.next_seg_id - sid) & 0xFFFF) < 0x8000
                    self.stats["seg_dup" if behind else "seg_gap"] += 1
                    continue
            if len(self.queue) >= BUFFER_CAPACITY:
                self.stats["seg_full"] += 1
                break
            if sflags & SEG_TIMED and sflags & SEG_SUBSTEP:
                # fractional steps: the machine model works in floating point
                steps = [v / SUBSTEP_SCALE for v in steps]
            n_dom = max(abs(s) for s in steps)
            if not (sflags & SEG_TIMED):
                # Legacy segment: the simulator plays it at its entry speed.
                v = max(100.0, math.sqrt(f[7] / 1000.0))
                dur = int(n_dom * 1e6 / v) if n_dom else 1
            dur = max(dur, int(n_dom * MIN_STEP_INTERVAL_US), 1)
            self.queue.append(Segment(steps, dur, sid, sflags))
            if self.first_queued is None:
                self.first_queued = time.perf_counter()
            self.stats["seg_accepted"] += 1
            if sequenced:
                self.next_seg_id = (self.next_seg_id + 1) & 0xFFFF

    def on_jog(self, payload):
        if len(payload) < 8:
            return
        axis, direction, flags, _r, speed = struct.unpack_from("<BbBBI", payload)
        if axis >= AXES or direction not in (1, -1):
            return
        if self.state not in (ST_IDLE, ST_JOG) or self.cur is not None or self.queue:
            return
        speed = min(float(speed), float(self.cfg[0x0010 + axis]))
        self.jog_target[axis] = direction * speed
        self.jog_deadline[axis] = (time.perf_counter() + JOG_DEADMAN_MS / 1000.0) if flags & JOG_DEADMAN else None
        if self.state == ST_IDLE:
            self.state = ST_JOG
            self.drivers_enabled = True
            self.ep_begin("jog")

    def on_jog_stop(self, payload):
        axis = payload[0] if payload else 0xFF
        for i in range(AXES):
            if axis == 0xFF or axis == i:
                self.jog_target[i] = 0.0
                self.jog_deadline[i] = None

    def on_home(self, payload):
        if len(payload) < 4 or self.state != ST_IDLE or self.moving() or self.queue:
            return
        mask, dir_mask, flags, _ = struct.unpack_from("<BBBB", payload)
        mask &= 0x3F
        if mask == 0:
            return
        seek = [float(self.cfg[0x0321])] * AXES
        latch = [float(self.cfg[0x0322])] * AXES
        pull = [float(self.cfg[0x0323])] * AXES
        switch = list(range(AXES))
        if flags & HOME_PARAMS and len(payload) >= 4 + 72 + 6:
            vals = struct.unpack_from("<18I6B", payload, 4)
            seek, latch, pull = [float(v) for v in vals[0:6]], [float(v) for v in vals[6:12]], [float(v) for v in vals[12:18]]
            switch = list(vals[18:24])
        else:
            dir_mask = self.cfg[0x0320]
        axes = [i for i in range(AXES) if mask & (1 << i)]
        # An axis without a switch of its own copies the first axis that has one.
        owners = [i for i in axes if switch[i] < AXES]
        if not owners:
            return
        for i in axes:
            if switch[i] >= AXES:
                switch[i] = switch[owners[0]]
        self.home = dict(mask=mask, axes=axes, dir={i: (1 if dir_mask & (1 << i) else -1) for i in axes},
                         seek=seek, latch=latch, pull=pull, switch=switch, phase="start",
                         t_phase=time.perf_counter(), mark={})
        self.homed_mask &= ~mask
        self.state = ST_HOMING
        self.drivers_enabled = True
        self.ep_begin("home")
        self.log("homing start mask=0x%02X" % mask)

    def on_io(self, payload, addr):
        if len(payload) < 8:
            return
        misc, sp_state, sp_rpm, coolant, flags, io_out = struct.unpack_from("<BBHBBH", payload)
        self.io_flags = flags
        if self.io_module:
            self.io_out = io_out if io_out else misc     # older hosts drive channels 0-7 through misc
        # The watchdog follows the flag: a host that stops setting it (as the
        # last thing it does before a clean exit) stands the watchdog down.
        self.watchdog_armed = bool(flags & IO_WATCHDOG)
        if self.host_ip == addr[0]:
            self.status_port = addr[1] if flags & IO_REPLY_SRC else UDP_STATUS_PORT
        if self.state in (ST_ESTOP, ST_ALARM):
            return                       # outputs stay off until the fault is cleared
        self.misc_out = misc & 0x1F
        self.spindle_state = sp_state if sp_state in (0, 1, 2) else 0
        self.spindle_rpm = sp_rpm
        self.coolant = coolant & 3
        if flags & IO_WATCHDOG:
            self.charge_pump = bool(flags & IO_CHARGE_PUMP)
        max_rpm = max(1, int(self.cfg[0x0431]))
        if self.spindle_state == 0:
            self.spindle_duty = 0.0
        elif self.cfg[0x0432]:
            duty = min(1.0, sp_rpm / max_rpm)
            self.spindle_duty = max(duty, self.cfg[0x0433] / 100.0)
        else:
            self.spindle_duty = 1.0

    def on_flush(self, payload):
        if len(payload) < 2:
            return
        stream, mode = payload[0], payload[1]
        if stream == self.stream_id and self.pending_stream is None:
            return
        if self.pending_stream is not None:
            return
        if mode == 0 and self.cur is not None and self.state not in (ST_ESTOP, ST_ALARM):
            self.pending_stream = stream
            self.stream_flags |= STREAM_FLUSHING
            self.queue.clear()
            self.begin_ramp_down(lambda: self.apply_flush(stream))
        else:
            self.apply_flush(stream)

    def on_reset(self):
        if self.state in (ST_ESTOP, ST_ALARM):
            if self.force["estop"]:
                return                   # refuse while the E-stop input is active
            self.state = ST_IDLE
            self.alarm = ALARM_NONE
            self.log("reset -> IDLE")

    def handle_udp(self, data, addr):
        self.stats["rx"] += 1
        pkt = parse_packet(data)
        if pkt is None:
            self.stats["rx_bad"] += 1
            return
        ptype, payload, _seq = pkt
        now = time.perf_counter()

        if ptype == PKT_DISCOVER:
            resp = struct.pack("<IBBBB6s2x32s", self.fw_version, self.capabilities(), 1 if self.io_module else 0,
                               1 if self.host_ip else 0, AXES, b"\x02\x00SIM\x01", self.device_name())
            self.udp.sendto(make_packet(PKT_DISCOVER_RESP, resp), addr)
            return

        with self.lock:
            # Which host do we listen to?  The TCP session owner if there is
            # one, otherwise the first UDP sender until it goes quiet.
            if self.tcp_host is not None:
                if addr[0] != self.tcp_host:
                    self.stats["rx_foreign"] += 1
                    return
                self.host_ip = self.tcp_host
            elif self.host_ip is None or now - self.last_host_rx > 3.0:
                self.host_ip = addr[0]
                self.status_port = UDP_STATUS_PORT
            elif addr[0] != self.host_ip:
                self.stats["rx_foreign"] += 1
                return
            self.last_host_rx = now

            if ptype == PKT_MOTION:
                self.on_motion(payload)
            elif ptype == PKT_JOG:
                self.on_jog(payload)
            elif ptype == PKT_JOG_STOP:
                self.on_jog_stop(payload)
            elif ptype == PKT_ESTOP:
                if self.state != ST_ESTOP:
                    self.enter_fault(ST_ESTOP, ALARM_ESTOP, "host E-stop")
            elif ptype == PKT_HOLD:
                if self.state == ST_RUN and self.ramp is None:
                    self.state = ST_HOLD
                    if self.cur is not None:
                        self.begin_ramp_down(lambda: None)
                    else:
                        self.rate = 0.0
            elif ptype == PKT_RESUME:
                if self.state == ST_HOLD:
                    self.state = ST_RUN
                    self.after_ramp = None
                    self.ramp = (1, 0.2)
            elif ptype == PKT_RESET:
                self.on_reset()
            elif ptype == PKT_HOME:
                self.on_home(payload)
            elif ptype == PKT_IO:
                self.on_io(payload, addr)
            elif ptype == PKT_FLUSH:
                self.on_flush(payload)

    # ------------------------------------------------------------- executor
    def tick(self, dt):
        """Advance the machine by dt seconds of wall-clock time."""
        now = time.perf_counter()

        # E-stop input
        if self.force["estop"] and self.state != ST_ESTOP:
            self.enter_fault(ST_ESTOP, ALARM_ESTOP, "E-stop input")

        # Host watchdog / lost session
        if self.watchdog_armed and now - self.last_host_rx > HOST_WATCHDOG_MS / 1000.0:
            self.watchdog_armed = False
            active = (self.moving() or self.queue or self.spindle_state or self.misc_out or self.charge_pump
                      or self.state in (ST_HOMING, ST_HOLD))
            if active and self.state not in (ST_ESTOP, ST_ALARM):
                self.stats["watchdog_trips"] += 1
                self.enter_fault(ST_ALARM, ALARM_HOST_LOST, "host watchdog expired")

        # Time-scale ramp (controlled stop / resume)
        if self.ramp is not None:
            direction, secs = self.ramp
            self.rate += direction * dt / max(secs, 1e-3)
            if direction < 0 and self.rate <= 0.02:
                self.rate = 0.0
                self.ramp = None
                cb, self.after_ramp = self.after_ramp, None
                if cb:
                    cb()
            elif direction > 0 and self.rate >= 1.0:
                self.rate = 1.0
                self.ramp = None

        # Segment playback (time is scaled while ramping to or from a stop)
        if self.state in (ST_IDLE, ST_RUN, ST_HOLD) and self.rate > 0.0:
            self.play(dt * 1e6 * self.rate)

        # Jog
        if self.state == ST_JOG:
            self.tick_jog(dt, now)
        elif self.state == ST_HOMING:
            self.tick_home(dt, now)

        # Hard limits
        if self.state in (ST_RUN, ST_JOG) and self.moving() and not (self.io_flags & IO_LIMIT_OVERRIDE):
            lm = self.limit_mask() & self.cfg[0x0314]
            if lm:
                axis = next(i for i in range(AXES) if lm & (1 << i))
                self.enter_fault(ST_ALARM, ALARM_LIMIT_X + axis, "limit switch %s" % "XYZABC"[axis])

    def play(self, dt_us):
        if self.cur is None:
            if not self.queue or self.state not in (ST_IDLE, ST_RUN) or self.pending_stream is not None:
                return
            waited = (time.perf_counter() - self.first_queued) * 1000.0 if self.first_queued else 0.0
            ready = (sum(s.dur for s in self.queue) >= START_QUEUE_MS * 1000 or waited >= START_MAX_WAIT_MS
                     or any(s.flags & SEG_LAST for s in self.queue))
            if not ready:
                return
            self.ep_begin("stream")
            self.load_next()
            self.state = ST_RUN
            self.drivers_enabled = True
            self.stream_flags &= ~STREAM_UNDERRUN

        remaining = dt_us
        while self.cur is not None and remaining > 0:
            seg = self.cur
            if seg.flags & SEG_PROBE and self.probe_active():
                self.stream_flags |= STREAM_PROBE_TRIPPED
                self.stream_closed = True
                self.log("probe contact at %s" % self.pos())
                self.stop_stream()
                self.state = ST_IDLE
                self.ep_end()
                return
            room = seg.dur - self.cur_pos
            use = min(room, remaining)
            self.cur_pos += use
            remaining -= use
            frac = self.cur_pos / seg.dur
            for i in range(AXES):
                self.phys[i] = self.cur_base[i] + seg.steps[i] * frac
            if self.cur_pos >= seg.dur - 1e-9:
                for i in range(AXES):
                    self.phys[i] = self.cur_base[i] + seg.steps[i]
                last = bool(seg.flags & SEG_LAST)
                if self.queue:
                    self.load_next()
                else:
                    self.cur = None
                    self.first_queued = None
                    self.feed_rate = 0.0
                    if not last and self.next_seg_id and self.pending_stream is None:
                        self.stream_flags |= STREAM_UNDERRUN
                        self.stats["underruns"] += 1
                    if self.state == ST_RUN:
                        self.state = ST_IDLE
                    self.ep_end()

    def load_next(self):
        seg = self.queue.popleft()
        self.cur = seg
        self.cur_pos = 0.0
        self.cur_base = list(self.phys)
        self.exec_seg_id = seg.sid
        self.feed_rate = max(abs(s) for s in seg.steps) * 1e6 / seg.dur
        self.ep_segment(seg)

    def tick_jog(self, dt, now):
        busy = False
        for i in range(AXES):
            if self.jog_deadline[i] is not None and now > self.jog_deadline[i]:
                self.jog_target[i] = 0.0
                self.jog_deadline[i] = None
            a = max(1.0, float(self.cfg[0x0020 + i]))
            dv = self.jog_target[i] - self.jog_v[i]
            step = a * dt
            self.jog_v[i] += max(-step, min(step, dv))
            self.phys[i] += self.jog_v[i] * dt
            if self.ep:
                self.ep["vmax"][i] = max(self.ep["vmax"][i], abs(self.jog_v[i]))
            if abs(self.jog_v[i]) > 1e-9 or abs(self.jog_target[i]) > 1e-9:
                busy = True
        self.feed_rate = max(abs(v) for v in self.jog_v)
        if not busy:
            self.state = ST_IDLE
            self.ep_end()

    def tick_home(self, dt, now):
        h = self.home
        raw = self.limit_mask()
        # 'lm' has a bit per AXIS: the state of the switch input that homes it
        lm = sum(1 << i for i in h["axes"] if raw & (1 << h["switch"][i]))

        def drive(i, v):
            a = max(1.0, float(self.cfg[0x0020 + i]))
            dv = v - self.jog_v[i]
            self.jog_v[i] += max(-a * dt, min(a * dt, dv))
            self.phys[i] += self.jog_v[i] * dt

        def stopped():
            return all(abs(self.jog_v[i]) < 1e-9 for i in h["axes"])

        def nxt(phase):
            h["phase"], h["t_phase"], h["mark"] = phase, now, {}

        if now - h["t_phase"] > 60.0:
            self.enter_fault(ST_ALARM, ALARM_HOME_FAIL, "homing timed out in phase %s" % h["phase"])
            return

        p = h["phase"]
        if p == "start":
            nxt("clear" if lm & h["mask"] else "seek")
        elif p in ("clear", "pulloff", "final"):
            # move away from the switch until it opens, then a further pull-off distance
            done = True
            for i in h["axes"]:
                active = bool(lm & (1 << i))
                if active:
                    h["mark"].pop(i, None)
                elif i not in h["mark"]:
                    h["mark"][i] = self.phys[i]
                far = i in h["mark"] and abs(self.phys[i] - h["mark"][i]) >= h["pull"][i]
                drive(i, 0.0 if far else -h["dir"][i] * h["latch"][i])
                if not far or abs(self.jog_v[i]) > 1e-9:
                    done = False
            if done:
                if p == "clear":
                    nxt("seek")
                elif p == "pulloff":
                    nxt("latch")
                else:
                    for i in h["axes"]:
                        self.origin[i] = self.phys[i]
                    self.homed_mask |= h["mask"]
                    self.log("homing complete mask=0x%02X" % h["mask"])
                    self.udp_send_host(make_packet(PKT_HOME_COMPLETE, struct.pack("<BB2x", h["mask"], 1)))
                    self.home = None
                    self.state = ST_IDLE
                    self.ep_end()
        elif p == "seek":
            for i in h["axes"]:
                drive(i, 0.0 if lm & (1 << i) else h["dir"][i] * h["seek"][i])
            if all(lm & (1 << i) for i in h["axes"]) and stopped():
                nxt("pulloff")
        elif p == "latch":
            for i in h["axes"]:
                if lm & (1 << i):
                    self.jog_v[i] = 0.0          # slow approach: stop on the spot
                else:
                    drive(i, h["dir"][i] * h["latch"][i])
            if all(lm & (1 << i) for i in h["axes"]):
                nxt("final")

    # ---------------------------------------------------------------- status
    def capabilities(self):
        if self.io_module:
            return CAP_IO_MODULE | CAP_STREAM_V2
        caps = CAP_CHARGE_PUMP | CAP_MISC_OUT | CAP_MISC_IN | CAP_SPINDLE_PWM
        if self.fw_version >= ((1 << 24) | (2 << 16)):
            caps |= CAP_STREAM_V2
        if self.encoder_ppr:
            caps |= CAP_SPINDLE_ENCODER
        return caps

    def device_name(self):
        return b"Tiggy I/O Module Simulator" if self.io_module else b"Tiggy Simulator"

    def spindle_feedback(self):
        """(rpm, position 0..65535, index count) of the simulated spindle encoder."""
        if not self.encoder_ppr:
            return 0, 0, 0
        now = time.perf_counter()
        last = getattr(self, "_spin_t", now)
        self._spin_t = now
        rpm = self.spindle_rpm if self.spindle_state else 0
        self.spindle_turns += rpm / 60.0 * (now - last)
        return min(65535, int(rpm)), int((self.spindle_turns % 1.0) * 65535), int(self.spindle_turns) & 0xFFFFFFFF

    def build_status(self):
        lm = self.limit_mask()
        flags = 0x01 | (0x02 if self.tcp_host else 0) | (0x04 if self.moving() else 0) | (0x08 if self.state == ST_HOMING else 0)
        jog_mask = sum(1 << i for i in range(AXES) if abs(self.jog_v[i]) > 1e-9 or abs(self.jog_target[i]) > 1e-9) if self.state == ST_JOG else 0
        p = self.pos()
        body = STATUS.pack(
            p[0], p[1], p[2], p[3], p[4], p[5],
            max(0, BUFFER_CAPACITY - len(self.queue)), BUFFER_CAPACITY,
            self.state, self.alarm, lm, 1 if self.probe_active() else 0,
            (self.home["mask"] if self.home else 0), lm, 1 if self.force["estop"] else 0, flags,
            self.exec_seg_id & 0xFFFF, int(time.perf_counter() * 1000) & 0xFFFFFFFF, int(self.feed_rate),
            self.misc_out, self.force["misc_in"] & 0x0F, self.spindle_state, self.coolant,
            *self.spindle_feedback(), self.force["io_in"] & 0xFFFF, self.io_out & 0xFFFF,
            self.next_seg_id, self.stream_id, self.stream_flags, self.homed_mask, jog_mask,
            min(65535, int(self.queued_ms())))
        return make_packet(PKT_STATUS, body)

    def udp_send_host(self, pkt):
        if self.host_ip and self.udp:
            if self.args.loss and random.random() < self.args.loss:
                return
            try:
                self.udp.sendto(pkt, (self.host_ip, self.status_port))
            except OSError:
                pass

    # --------------------------------------------------------------- threads
    def run_udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.args.bind, UDP_MOTION_PORT))
        self.udp = s
        while True:
            try:
                data, addr = s.recvfrom(2048)
            except OSError:
                continue
            if self.args.loss and random.random() < self.args.loss:
                self.stats["rx_dropped"] += 1
                continue
            batch = [(data, addr)]
            if self.args.reorder and random.random() < self.args.reorder:
                if self.hold_reorder is None:
                    self.hold_reorder = (data, addr)
                    continue
            if self.hold_reorder is not None:
                batch.append(self.hold_reorder)
                self.hold_reorder = None
            if self.args.dup and random.random() < self.args.dup:
                batch.append((data, addr))
            for d, a in batch:
                self.handle_udp(d, a)

    def run_exec(self):
        last = time.perf_counter()
        next_status = last
        while True:
            time.sleep(0.001)
            now = time.perf_counter()
            dt, last = now - last, now
            with self.lock:
                self.tick(min(dt, 0.1))
                if now >= next_status:
                    next_status = max(next_status + 0.05, now)
                    alive = self.host_ip and (self.tcp_host or now - self.last_host_rx < 3.0)
                    if alive:
                        self.stats["tx_status"] += 1
                        self.udp_send_host(self.build_status())

    def run_tcp(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.args.bind, TCP_PORT))
        srv.listen(2)
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=self.serve_tcp, args=(conn, addr), daemon=True).start()

    def serve_tcp(self, conn, addr):
        self.log("TCP connect from %s" % addr[0])
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(2.0)
        with self.lock:
            self.tcp_host = addr[0]
            self.host_ip = addr[0]
            self.status_port = UDP_STATUS_PORT
            self.last_host_rx = time.perf_counter()
        me = threading.current_thread()
        self.tcp_owner = me
        buf = b""
        try:
            while self.tcp_owner is me:
                try:
                    chunk = conn.recv(1024)
                except socket.timeout:
                    # A quiet control channel is fine while the host is still
                    # talking on UDP; give up after 30 s of total silence.
                    if time.perf_counter() - self.last_host_rx > 30.0:
                        raise ConnectionError("host silent for 30 s")
                    continue
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 2:
                    n = struct.unpack_from("<H", buf)[0]
                    if n == 0 or n > 256:
                        raise ConnectionError("bad frame length %d" % n)
                    if len(buf) < 2 + n:
                        break
                    frame, buf = buf[2:2 + n], buf[2 + n:]
                    resp = self.handle_tcp(frame)
                    if resp:
                        conn.sendall(struct.pack("<H", len(resp)) + resp)
        except (OSError, ConnectionError) as e:
            self.log("TCP session ended: %s" % e)
        finally:
            conn.close()
            with self.lock:
                if self.tcp_owner is me:
                    self.tcp_host = None
                    self.log("TCP disconnect from %s" % addr[0])
                    # Losing the session while the watchdog is armed is
                    # treated like a watchdog expiry.
                    if self.watchdog_armed:
                        self.last_host_rx = 0.0
                    self.host_ip = None

    def handle_tcp(self, frame):
        pkt = parse_packet(frame)
        if pkt is None:
            return None
        ptype, payload, seq = pkt
        with self.lock:
            self.last_host_rx = time.perf_counter() if self.tcp_host else self.last_host_rx
            if ptype == PKT_HS_REQ:
                body = struct.pack("<IBBHI32sBBBB", self.fw_version, AXES, self.capabilities(), BUFFER_CAPACITY,
                                   int(1e6 / MIN_STEP_INTERVAL_US), self.device_name(),
                                   1 if self.io_module else 0, (self.encoder_ppr >> 8) & 0xFF,
                                   self.encoder_ppr & 0xFF, 16 if self.io_module else 0)
                return make_packet(PKT_HS_RESP, body)
            if ptype == PKT_PING:
                return make_packet(PKT_PONG, payload[:4].ljust(4, b"\0"))
            if ptype == PKT_CFG_SET and len(payload) >= 68:
                key, vtype = struct.unpack_from("<HH", payload)
                raw = payload[4:68]
                want = CFG_TYPES.get(key, (vtype, 0))[0]
                if vtype == want:
                    fmt = {0: "<B", 1: "<H", 2: "<I", 3: "<i", 4: "<f"}.get(vtype)
                    val = struct.unpack_from(fmt, raw)[0] if fmt else raw.split(b"\0")[0].decode("latin1")
                    (self.cfg if key in CFG_TYPES else self.cfg_extra)[key] = val
                else:
                    self.log("config 0x%04X: wrong type %d (expected %d) - ignored" % (key, vtype, want))
                return make_packet(PKT_ACK, struct.pack("<IB3x", seq, ptype))
            if ptype == PKT_CFG_GET and len(payload) >= 4:
                key = struct.unpack_from("<H", payload)[0]
                vtype, _d = CFG_TYPES.get(key, (0, 0))
                val = self.cfg.get(key, self.cfg_extra.get(key, 0xFF if 0x0500 <= key < 0x0600 else 0))
                fmt = {0: "<B", 1: "<H", 2: "<I", 3: "<i", 4: "<f"}.get(vtype, "<B")
                raw = struct.pack(fmt, val) if not isinstance(val, str) else val.encode("latin1")
                return make_packet(PKT_CFG_RESP, struct.pack("<HH64s", key, vtype, raw))
            if ptype == PKT_CFG_SAVE:
                return make_packet(PKT_ACK, struct.pack("<IB3x", seq, ptype))
        return None

    # ---------------------------------------------------- automation channel
    def run_control(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.args.control_port))
        srv.listen(4)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=self.serve_control, args=(conn,), daemon=True).start()

    def serve_control(self, conn):
        f = conn.makefile("rwb")
        try:
            for line in f:
                try:
                    req = json.loads(line.decode("utf-8"))
                    resp = self.control(req)
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
            if cmd == "set":
                for k in ("estop", "probe", "limits", "misc_in", "io_in"):
                    if k in req:
                        self.force[k] = int(req[k])
                if "probe_z" in req:
                    self.probe_z = None if req["probe_z"] is None else float(req["probe_z"])
                if "lim_lo" in req:
                    self.lim_lo = [float(v) for v in req["lim_lo"]]
                if "lim_hi" in req:
                    self.lim_hi = [float(v) for v in req["lim_hi"]]
                if "phys" in req:
                    self.phys = [float(v) for v in req["phys"]]
                for k in ("loss", "dup", "reorder"):
                    if k in req:
                        setattr(self.args, k, float(req[k]))
            elif cmd == "clear":
                self.episodes.clear()
                for k in self.stats:
                    self.stats[k] = 0
            elif cmd == "episodes":
                return {"episodes": list(self.episodes)}
            elif cmd == "log":
                return {"log": list(self.log_lines)}
            return {
                "state": STATE_NAMES[self.state], "alarm": self.alarm, "pos": self.pos(),
                "phys": [round(v, 2) for v in self.phys], "moving": self.moving(),
                "queue": len(self.queue), "queued_ms": round(self.queued_ms(), 1),
                "stream": self.stream_id, "next_seg": self.next_seg_id, "stream_flags": self.stream_flags,
                "homed": self.homed_mask, "limits": self.limit_mask(), "probe": self.probe_active(),
                "outputs": dict(misc=self.misc_out, spindle=self.spindle_state, rpm=self.spindle_rpm,
                                duty=round(self.spindle_duty, 4), coolant=self.coolant,
                                charge_pump=self.charge_pump, drivers=self.drivers_enabled, io=self.io_out),
                "host": self.host_ip, "tcp": self.tcp_host, "watchdog": self.watchdog_armed,
                "io_flags": self.io_flags, "cfg": {("0x%04X" % k): v for k, v in sorted(self.cfg.items())},
                "stats": dict(self.stats), "episodes": len(self.episodes), "force": dict(self.force),
            }


def main():
    ap = argparse.ArgumentParser(description="Tiggy controller simulator")
    ap.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    ap.add_argument("--control-port", type=int, default=58500, help="local automation port")
    ap.add_argument("--loss", type=float, default=0.0, help="fraction of UDP packets to drop (both directions)")
    ap.add_argument("--dup", type=float, default=0.0, help="fraction of received UDP packets to duplicate")
    ap.add_argument("--reorder", type=float, default=0.0, help="fraction of received UDP packets to delay behind the next")
    ap.add_argument("--fw", default="1.2.0", help="firmware version to report (e.g. 1.1.0 to test the 'too old' path)")
    ap.add_argument("--io-module", action="store_true", help="behave as an I/O expansion module")
    ap.add_argument("--encoder", type=int, default=0, metavar="PPR", help="simulate a spindle encoder with this resolution")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()


    sim = Sim(args)
    sim.tcp_owner = None
    for fn in (sim.run_udp, sim.run_tcp, sim.run_control, sim.run_exec):
        threading.Thread(target=fn, daemon=True).start()
    sim.log("Tiggy simulator ready: TCP %d, UDP %d, control 127.0.0.1:%d" % (TCP_PORT, UDP_MOTION_PORT, args.control_port))
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
