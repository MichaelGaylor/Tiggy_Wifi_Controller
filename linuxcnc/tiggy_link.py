#!/usr/bin/env python3
"""
Tiggy motion controller - finding it and talking to it from Python.

Used by tiggy_config.py to find controllers on the network, read and change
their settings, watch their inputs and jog a motor while it is being set up.
(LinuxCNC itself talks to the controller through the driver, tiggy.c; this
is for when LinuxCNC is not running - a controller works for one program at
a time.)

    python3 tiggy_link.py                 list the controllers found
    python3 tiggy_link.py 192.168.1.50    ask one address, and show what it reports
"""

import socket
import struct
import subprocess
import sys
import time

MAGIC, PROTOCOL = 0x574D4333, 1
UDP_PORT, TCP_PORT = 58427, 58429
HEADER = struct.Struct("<IBBHIIH")
STATUS = struct.Struct("<6iHH8BHIi4BHHIHH")         # the report of protocol 1.1 ...
STATUS_12 = struct.Struct("<HBBBBH")                # ... and what 1.2 adds
DISCOVER = struct.Struct("<IBBBB6s2x32s")
HANDSHAKE = struct.Struct("<IBBHI32sBBBB")

# packet types
P_JOG, P_JOG_STOP, P_ESTOP, P_RESET, P_IO, P_DISCOVER = 0x02, 0x03, 0x04, 0x07, 0x09, 0x0B
P_STATUS, P_ACK, P_FOUND = 0x20, 0x24, 0x25
P_HELLO, P_HELLO_BACK, P_GET, P_SET, P_VALUE, P_SAVE, P_PING, P_PONG = 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x50, 0x51

STATES = ["idle", "running", "held", "jogging", "homing", "probing", "alarm", "E-stop", "no host"]
ALARMS = {1: "limit switch X", 2: "limit switch Y", 3: "limit switch Z", 4: "limit switch A", 5: "limit switch B",
          6: "limit switch C", 10: "probe did not touch", 11: "ran out of motion", 20: "E-stop input",
          30: "watchdog", 31: "the host stopped talking", 40: "homing failed"}

# capabilities
CAP_CHARGE_PUMP, CAP_OUTPUTS, CAP_INPUTS, CAP_SPINDLE_PWM = 0x01, 0x02, 0x04, 0x08
CAP_SENSOR, CAP_IO_MODULE, CAP_STREAM, CAP_SPINDLE_SYNC = 0x10, 0x20, 0x40, 0x80

# settings: name -> (key, kind)    kinds: B 8 bits, H 16, I 32, f float
SETTINGS = {
    "steps_x": (0x0001, "f"), "max_rate_x": (0x0010, "I"), "accel_x": (0x0020, "I"),       # + axis number
    "step_pulse_us": (0x0100, "H"), "dir_setup_us": (0x0101, "H"), "idle_release_ms": (0x0102, "H"),
    "status_ms": (0x0103, "H"),
    "invert_step": (0x0300, "B"), "invert_dir": (0x0301, "B"), "invert_enable": (0x0302, "B"),
    "invert_limit": (0x0310, "B"), "invert_home": (0x0311, "B"), "invert_estop": (0x0312, "B"),
    "invert_probe": (0x0313, "B"), "limit_mask": (0x0314, "B"), "debounce_ms": (0x0315, "H"),
    "pump_hz": (0x0400, "H"), "pwm_hz": (0x0430, "H"), "pwm_max_rpm": (0x0431, "I"), "pwm": (0x0432, "B"),
    "pwm_min_duty": (0x0433, "B"),
    "sensor_lines": (0x0523, "H"), "sensor_kind": (0x0524, "B"),
    "device_mode": (0x0600, "B"), "terminals": (0x0601, "B"),
}
KIND = {"B": (0, "<B"), "H": (1, "<H"), "I": (2, "<I"), "f": (4, "<f")}

# The spindle sensor, as the controller's two settings have it (sensor_kind, and sensor_lines of 0 for none)
SENSOR_NONE, SENSOR_PULSE, SENSOR_ENCODER, SENSOR_ENCODER_NO_INDEX = "none", "pulse", "encoder", "encoder_no_index"
# The terminals: all six motor outputs, or four and the rest for the sensor, inputs and outputs
TERMINALS_SIX, TERMINALS_FOUR = 0, 1


class LinkError(Exception):
    pass


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


_sequence = [0]


def packet(kind, payload=b""):
    _sequence[0] += 1
    head = HEADER.pack(MAGIC, PROTOCOL, kind, len(payload), _sequence[0], int(time.monotonic() * 1e6) & 0xFFFFFFFF, 0)
    return head[:16] + struct.pack("<H", crc16(head + payload)) + payload


def parse(data):
    """(kind, payload) of a packet, or None if it is not one."""
    if len(data) < HEADER.size:
        return None
    magic, version, kind, length, _seq, _time, crc = HEADER.unpack_from(data)
    if magic != MAGIC or version != PROTOCOL or len(data) < HEADER.size + length:
        return None
    body = data[:HEADER.size + length]
    if crc16(body[:16] + b"\0\0" + body[18:]) != crc:
        return None
    return kind, body[HEADER.size:]


def version_text(v):
    return "%d.%d.%d" % (v >> 24, (v >> 16) & 0xFF, v & 0xFFFF)


def broadcast_addresses():
    """Where to call "who is there?": every network this computer is on."""
    found = ["255.255.255.255"]
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, timeout=3).stdout
        for line in out.splitlines():
            words = line.split()
            if "brd" in words and "lo" not in words[1]:
                found.append(words[words.index("brd") + 1])
    except (OSError, subprocess.SubprocessError):
        pass
    return found


def discover(also=(), wait=0.8):
    """The controllers that answer: a list of dictionaries, nearest address first.
    also: addresses to ask directly (a broadcast does not cross every router, and not out of WSL)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(0.1)
    ask = packet(P_DISCOVER)
    targets = broadcast_addresses() + ["192.168.4.1"] + [a for a in also if a and a != "auto"]
    found = {}
    end = time.monotonic() + wait
    sent = 0.0
    while time.monotonic() < end:
        if time.monotonic() - sent > 0.3:               # (a single packet is easily lost on WiFi)
            sent = time.monotonic()
            for target in targets:
                try:
                    s.sendto(ask, (target, UDP_PORT))
                except OSError:
                    pass
        try:
            data, sender = s.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            break
        p = parse(data)
        if not p or p[0] != P_FOUND or len(p[1]) < DISCOVER.size:
            continue
        version, caps, mode, busy, axes, mac, name = DISCOVER.unpack_from(p[1])
        found[sender[0]] = dict(address=sender[0], name=name.split(b"\0")[0].decode("ascii", "replace"),
                                version=version, firmware=version_text(version), capabilities=caps,
                                io_module=mode == 1, busy=bool(busy), axes=axes,
                                serial=":".join("%02X" % b for b in mac))
    s.close()
    return sorted(found.values(), key=lambda c: tuple(int(x) for x in c["address"].split(".")))


class Controller:
    """One controller, for the time this program has it.  poll() must be called often (every
    50 ms or so): it keeps the controller's watchdog fed and takes in its status reports."""

    def __init__(self, address):
        self.address = address
        self.tcp = None
        self.udp = None
        self.info = {}
        self.status = None
        self.status_at = 0.0
        self.out = dict(outputs=0, spindle=0, rpm=0, coolant=0)
        self._sent_io = self._sent_ping = 0.0

    # ---- the connection
    def open(self):
        try:
            self.tcp = socket.create_connection((self.address, TCP_PORT), timeout=3.0)
        except OSError as e:
            raise LinkError("no answer from %s (%s)" % (self.address, e.strerror or e))
        self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("0.0.0.0", 0))
        self.udp.setblocking(False)
        try:
            kind, body = self._call(P_HELLO, struct.pack("<I32s", (1 << 24) | (2 << 16), b"tiggy_config"))
        except LinkError:
            self.close()
            raise LinkError("%s accepts a connection but does not answer as a Tiggy controller: another program "
                            "(LinuxCNC, Mach3) is using it, or the address is something else's" % self.address)
        if kind != P_HELLO_BACK or len(body) < HANDSHAKE.size - 4:
            self.close()
            raise LinkError("%s does not answer as a Tiggy controller" % self.address)
        body = body + b"\0" * 4
        version, axes, caps, queue, rate, name, mode, ppr_hi, ppr_lo, channels = HANDSHAKE.unpack_from(body)
        self.info = dict(version=version, firmware=version_text(version), axes=axes, capabilities=caps,
                         max_step_rate=rate, name=name.split(b"\0")[0].decode("ascii", "replace"),
                         io_module=mode == 1)
        self.poll()
        return self.info

    def close(self):
        for s in (self.tcp, self.udp):
            try:
                if s:
                    s.close()
            except OSError:
                pass
        self.tcp = self.udp = None
        self.status = None

    @property
    def connected(self):
        return self.tcp is not None

    def _call(self, kind, payload=b""):
        if not self.tcp:
            raise LinkError("not connected")
        frame = packet(kind, payload)
        try:
            self.tcp.settimeout(3.0)
            self.tcp.sendall(struct.pack("<H", len(frame)) + frame)
            size = struct.unpack("<H", self._read(2))[0]
            answer = parse(self._read(size))
        except (OSError, struct.error) as e:
            raise LinkError("the connection to the controller was lost (%s)" % e)
        if not answer:
            raise LinkError("the controller's answer could not be read")
        return answer

    def _read(self, n):
        data = b""
        while len(data) < n:
            chunk = self.tcp.recv(n - len(data))
            if not chunk:
                raise OSError("closed by the controller")
            data += chunk
        return data

    # ---- settings
    @staticmethod
    def _key(name, axis):
        key, kind = SETTINGS[name]
        return key + axis, kind

    def get(self, name, axis=0):
        key, kind = self._key(name, axis)
        _kind, body = self._call(P_GET, struct.pack("<HH64s", key, 0, b""))
        return struct.unpack_from(KIND[kind][1], body, 4)[0]

    def set(self, name, value, axis=0):
        key, kind = self._key(name, axis)
        number, form = KIND[kind]
        value = float(value) if kind == "f" else max(0, int(round(value)))
        answer, _body = self._call(P_SET, struct.pack("<HH64s", key, number, struct.pack(form, value)))
        if answer != P_ACK:
            raise LinkError("the controller did not take the setting %s" % name)

    def save(self):
        """Makes the settings permanent.  The controller restarts a second later if one of them is
        only read at start-up (the terminals, the spindle sensor): see restarted()."""
        answer, _body = self._call(P_SAVE)
        if answer != P_ACK:
            raise LinkError("the controller did not save its settings")

    def restarted(self, wait=40.0):
        """After save(): waits out a restart and connects again.  (If it does not restart,
        this is simply a reconnection.)"""
        self.close()
        time.sleep(2.5)
        end = time.monotonic() + wait
        while True:
            try:
                return self.open()
            except LinkError:
                if time.monotonic() > end:
                    raise LinkError("the controller at %s did not come back after its settings were saved" % self.address)
                time.sleep(1.0)

    # ---- the spindle sensor and the terminals, which need a restart
    def sensor(self):
        """(kind, lines per revolution) as set on the controller."""
        lines, kind = self.get("sensor_lines"), self.get("sensor_kind")
        if lines == 0:
            return SENSOR_NONE, 0
        return {1: SENSOR_PULSE, 2: SENSOR_ENCODER_NO_INDEX}.get(kind, SENSOR_ENCODER), lines

    def set_sensor(self, kind, lines):
        self.set("sensor_kind", {SENSOR_PULSE: 1, SENSOR_ENCODER_NO_INDEX: 2}.get(kind, 0))
        self.set("sensor_lines", 0 if kind == SENSOR_NONE else (1 if kind == SENSOR_PULSE else max(1, lines)))

    # ---- while connected
    def _send(self, kind, payload=b""):
        if self.udp:
            try:
                self.udp.sendto(packet(kind, payload), (self.address, UDP_PORT))
            except OSError:
                pass

    def poll(self):
        """Feeds the watchdog, reads the status reports.  Returns the latest report (or None)."""
        if not self.udp:
            return None
        now = time.monotonic()
        if now - self._sent_io > 0.1:
            self._sent_io = now
            o = self.out
            # flags: watchdog (stop if this program goes away) and "answer to where this came from"
            self._send(P_IO, struct.pack("<BBHBBH", o["outputs"], o["spindle"], o["rpm"], o["coolant"], 0x03, 0))
        if now - self._sent_ping > 2.0:
            self._sent_ping = now
            self._call(P_PING, struct.pack("<I", 0))
        while True:
            try:
                data, _sender = self.udp.recvfrom(512)
            except (BlockingIOError, OSError):
                break
            p = parse(data)
            if p and p[0] == P_STATUS and len(p[1]) >= STATUS.size:
                f = STATUS.unpack_from(p[1])
                s = dict(position=list(f[0:6]), state=f[8], alarm=f[9], switches=f[10], probe=bool(f[11]),
                         estop=bool(f[14]), moving=bool(f[15] & 0x04), outputs=f[19], inputs=f[20],
                         rpm=f[23], angle=f[24] / 65536.0, turns=f[25], jogging=0)
                if len(p[1]) >= STATUS.size + STATUS_12.size:
                    s["jogging"] = STATUS_12.unpack_from(p[1], STATUS.size)[4]
                self.status, self.status_at = s, now
        if self.status and now - self.status_at > 1.5:
            self.status = None                          # (gone quiet)
        return self.status

    def jog(self, axis, direction, steps_per_second):
        """Keep calling this (every 100 ms or so) for as long as the axis should move."""
        self._send(P_JOG, struct.pack("<BbBBI", axis, 1 if direction > 0 else -1, 0x01, 0, max(1, int(steps_per_second))))

    def stop(self, axis=0xFF):
        self._send(P_JOG_STOP, struct.pack("<B3x", axis))

    def reset(self):
        """Leave E-stop or an alarm."""
        self._send(P_RESET)

    def state_text(self):
        s = self.status
        if not s:
            return "no reports"
        if s["state"] == 6:
            return "alarm: " + ALARMS.get(s["alarm"], "number %d" % s["alarm"])
        return STATES[s["state"]] if s["state"] < len(STATES) else "state %d" % s["state"]


def main():
    if len(sys.argv) > 1:
        c = Controller(sys.argv[1])
        try:
            info = c.open()
        except LinkError as e:
            sys.exit(str(e))
        print("%s at %s: firmware %s, up to %d steps a second on an axis" % (
            info["name"], c.address, info["firmware"], info["max_step_rate"]))
        print("spindle sensor: %s, %d per revolution; terminals: %s" % (
            c.sensor() + ("six axes" if c.get("terminals") == TERMINALS_SIX else "four axes and inputs/outputs",)))
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            c.poll()
            time.sleep(0.02)
        s = c.status
        if s:
            print("%s; switches %s, probe %d, E-stop input %d, inputs %s; spindle %d rpm" % (
                c.state_text(), format(s["switches"], "06b")[::-1], s["probe"], s["estop"],
                format(s["inputs"], "04b")[::-1], s["rpm"]))
        c.close()
        return
    found = discover()
    if not found:
        sys.exit("No controller answered.  (Across some routers, and from inside WSL, a controller has to be "
                 "asked by its address: python3 tiggy_link.py 192.168.1.50)")
    for c in found:
        print("%-16s %-18s firmware %-8s %s%s" % (c["address"], c["name"], c["firmware"], c["serial"],
                                                 "   in use by a program" if c["busy"] else ""))


if __name__ == "__main__":
    main()
