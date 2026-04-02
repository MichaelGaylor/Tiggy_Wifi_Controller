#!/usr/bin/env python3
"""
WiFi CNC Controller - Protocol Test Tool

Standalone tool for testing the ESP32 firmware without Mach3.
Connects via TCP for handshake/config and sends motion segments via UDP.

Usage:
    python protocol_test.py <ESP32_IP> [command]
    python protocol_test.py discover

Commands:
    discover    - Find ESP32 on the network (no IP needed)
    handshake   - Connect and perform handshake (default)
    status      - Listen for status reports
    jog         - Jog X axis forward at 1000 steps/sec for 2 seconds
    motion      - Send a test motion segment (1000 steps on X at 500 steps/sec)
    estop       - Send emergency stop
    reset       - Send reset command
    home        - Send home command for all axes
    config      - Read all configuration values from ESP32
    config-set  - Set a config value: config-set <key_hex> <value> [type]
    wifi-setup  - Set WiFi credentials: wifi-setup <SSID> <password>
"""

import socket
import struct
import time
import sys
import threading

# Protocol constants (must match wifi_cnc_protocol.h)
WCNC_MAGIC = 0x574D4333
WCNC_VERSION = 1

UDP_MOTION_PORT = 58427
UDP_STATUS_PORT = 58428
TCP_CONTROL_PORT = 58429

# Packet types
PKT_MOTION_SEGMENT = 0x01
PKT_JOG_COMMAND = 0x02
PKT_JOG_STOP = 0x03
PKT_ESTOP = 0x04
PKT_FEED_HOLD = 0x05
PKT_FEED_RESUME = 0x06
PKT_RESET = 0x07
PKT_HOME_COMMAND = 0x08

PKT_IO_CONTROL = 0x09
PKT_STATUS_REPORT = 0x20
PKT_ALARM = 0x21
PKT_HOME_COMPLETE = 0x22
PKT_PROBE_RESULT = 0x23
PKT_ACK = 0x24

PKT_HANDSHAKE_REQ = 0x40
PKT_HANDSHAKE_RESP = 0x41
PKT_CONFIG_GET = 0x42
PKT_CONFIG_SET = 0x43
PKT_CONFIG_RESP = 0x44
PKT_CONFIG_SAVE = 0x45
PKT_PING = 0x50
PKT_PONG = 0x51

# Config keys
CFG_STEPS_PER_MM_X = 0x0001
CFG_STEPS_PER_MM_Y = 0x0002
CFG_STEPS_PER_MM_Z = 0x0003
CFG_STEPS_PER_MM_A = 0x0004
CFG_STEPS_PER_MM_B = 0x0005
CFG_STEPS_PER_MM_C = 0x0006
CFG_MAX_RATE_X = 0x0010
CFG_MAX_RATE_Y = 0x0011
CFG_MAX_RATE_Z = 0x0012
CFG_MAX_RATE_A = 0x0013
CFG_MAX_RATE_B = 0x0014
CFG_MAX_RATE_C = 0x0015
CFG_ACCEL_X = 0x0020
CFG_ACCEL_Y = 0x0021
CFG_ACCEL_Z = 0x0022
CFG_ACCEL_A = 0x0023
CFG_ACCEL_B = 0x0024
CFG_ACCEL_C = 0x0025
CFG_STEP_PULSE_US = 0x0100
CFG_DIR_SETUP_US = 0x0101
CFG_STEP_IDLE_DELAY_MS = 0x0102
CFG_STATUS_INTERVAL_MS = 0x0103
CFG_WIFI_SSID = 0x0200
CFG_WIFI_PASSWORD = 0x0201
CFG_IP_MODE = 0x0210
CFG_STATIC_IP = 0x0211
CFG_STATIC_GATEWAY = 0x0212
CFG_STATIC_NETMASK = 0x0213
CFG_INVERT_STEP = 0x0300
CFG_INVERT_DIR = 0x0301
CFG_INVERT_ENABLE = 0x0302
CFG_INVERT_LIMIT = 0x0310
CFG_INVERT_HOME = 0x0311
CFG_INVERT_ESTOP = 0x0312
CFG_INVERT_PROBE = 0x0313
CFG_HOMING_DIR_MASK = 0x0320
CFG_HOMING_SEEK_RATE = 0x0321
CFG_HOMING_FEED_RATE = 0x0322
CFG_HOMING_PULLOFF = 0x0323
CFG_CHARGE_PUMP_FREQ = 0x0400
CFG_SPINDLE_PWM_FREQ = 0x0430
CFG_SPINDLE_MAX_RPM = 0x0431

# Pin assignments (uint8 = GPIO number, requires reboot)
CFG_PIN_STEP_X = 0x0500
CFG_PIN_STEP_Y = 0x0501
CFG_PIN_STEP_Z = 0x0502
CFG_PIN_STEP_A = 0x0503
CFG_PIN_STEP_B = 0x0504
CFG_PIN_STEP_C = 0x0505
CFG_PIN_DIR_X = 0x0506
CFG_PIN_DIR_Y = 0x0507
CFG_PIN_DIR_Z = 0x0508
CFG_PIN_DIR_A = 0x0509
CFG_PIN_DIR_B = 0x050A
CFG_PIN_DIR_C = 0x050B
CFG_PIN_ENABLE = 0x050C
CFG_PIN_LIMIT_X = 0x050D
CFG_PIN_LIMIT_Y = 0x050E
CFG_PIN_LIMIT_Z = 0x050F
CFG_PIN_LIMIT_A = 0x0510
CFG_PIN_LIMIT_B = 0x0511
CFG_PIN_LIMIT_C = 0x0512
CFG_PIN_PROBE = 0x0513
CFG_PIN_ESTOP = 0x0514
CFG_PIN_SPINDLE = 0x0515
CFG_PIN_LED = 0x0516
CFG_PIN_CHARGE_PUMP = 0x0517
CFG_PIN_MISC_OUT0 = 0x0518
CFG_PIN_MISC_OUT1 = 0x0519

# Spindle encoder config keys (v1.1)
CFG_PIN_ENCODER_A = 0x0520
CFG_PIN_ENCODER_B = 0x0521
CFG_PIN_ENCODER_INDEX = 0x0522
CFG_ENCODER_PPR = 0x0523
CFG_ENCODER_MODE = 0x0524
CFG_ENCODER_FILTER_NS = 0x0525

# Misc input pin assignments (v1.1)
CFG_PIN_MISC_IN0 = 0x0530
CFG_PIN_MISC_IN1 = 0x0531
CFG_PIN_MISC_IN2 = 0x0532
CFG_PIN_MISC_IN3 = 0x0533

# Device mode (v1.1)
CFG_DEVICE_MODE = 0x0600

# I/O module config (v1.1)
CFG_IO_PIN_COUNT = 0x0610
CFG_IO_DIR_MASK = 0x0611
CFG_IO_PULLUP_MASK = 0x0612
CFG_IO_INVERT_MASK = 0x0613
CFG_IO_PIN_BASE = 0x0620  # 0x0620..0x062F = IO channel 0..15

# Ethernet (W5500 SPI) pin config
CFG_PIN_ETH_MOSI = 0x0700
CFG_PIN_ETH_MISO = 0x0701
CFG_PIN_ETH_SCLK = 0x0702
CFG_PIN_ETH_INT = 0x0703
CFG_PIN_ETH_SPI_HOST = 0x0704

# Value types
VAL_UINT8 = 0
VAL_UINT16 = 1
VAL_UINT32 = 2
VAL_INT32 = 3
VAL_FLOAT = 4
VAL_STRING = 5

# Config value max length
CONFIG_VALUE_LEN = 64

# Config key metadata: (name, value_type)
CONFIG_KEYS = {
    CFG_STEPS_PER_MM_X: ("Steps/mm X", VAL_FLOAT),
    CFG_STEPS_PER_MM_Y: ("Steps/mm Y", VAL_FLOAT),
    CFG_STEPS_PER_MM_Z: ("Steps/mm Z", VAL_FLOAT),
    CFG_STEPS_PER_MM_A: ("Steps/mm A", VAL_FLOAT),
    CFG_STEPS_PER_MM_B: ("Steps/mm B", VAL_FLOAT),
    CFG_STEPS_PER_MM_C: ("Steps/mm C", VAL_FLOAT),
    CFG_MAX_RATE_X: ("Max rate X", VAL_UINT32),
    CFG_MAX_RATE_Y: ("Max rate Y", VAL_UINT32),
    CFG_MAX_RATE_Z: ("Max rate Z", VAL_UINT32),
    CFG_MAX_RATE_A: ("Max rate A", VAL_UINT32),
    CFG_MAX_RATE_B: ("Max rate B", VAL_UINT32),
    CFG_MAX_RATE_C: ("Max rate C", VAL_UINT32),
    CFG_ACCEL_X: ("Accel X", VAL_UINT32),
    CFG_ACCEL_Y: ("Accel Y", VAL_UINT32),
    CFG_ACCEL_Z: ("Accel Z", VAL_UINT32),
    CFG_ACCEL_A: ("Accel A", VAL_UINT32),
    CFG_ACCEL_B: ("Accel B", VAL_UINT32),
    CFG_ACCEL_C: ("Accel C", VAL_UINT32),
    CFG_STEP_PULSE_US: ("Step pulse us", VAL_UINT16),
    CFG_DIR_SETUP_US: ("Dir setup us", VAL_UINT16),
    CFG_STEP_IDLE_DELAY_MS: ("Idle delay ms", VAL_UINT16),
    CFG_STATUS_INTERVAL_MS: ("Status interval ms", VAL_UINT16),
    CFG_WIFI_SSID: ("WiFi SSID", VAL_STRING),
    CFG_WIFI_PASSWORD: ("WiFi Password", VAL_STRING),
    CFG_IP_MODE: ("IP mode", VAL_UINT8),
    CFG_STATIC_IP: ("Static IP", VAL_UINT32),
    CFG_STATIC_GATEWAY: ("Static gateway", VAL_UINT32),
    CFG_STATIC_NETMASK: ("Static netmask", VAL_UINT32),
    CFG_INVERT_STEP: ("Invert step", VAL_UINT8),
    CFG_INVERT_DIR: ("Invert dir", VAL_UINT8),
    CFG_INVERT_ENABLE: ("Invert enable", VAL_UINT8),
    CFG_INVERT_LIMIT: ("Invert limit", VAL_UINT8),
    CFG_INVERT_HOME: ("Invert home", VAL_UINT8),
    CFG_INVERT_ESTOP: ("Invert E-Stop", VAL_UINT8),
    CFG_INVERT_PROBE: ("Invert probe", VAL_UINT8),
    CFG_HOMING_DIR_MASK: ("Homing dir mask", VAL_UINT8),
    CFG_HOMING_SEEK_RATE: ("Homing seek rate", VAL_UINT32),
    CFG_HOMING_FEED_RATE: ("Homing feed rate", VAL_UINT32),
    CFG_HOMING_PULLOFF: ("Homing pulloff", VAL_UINT32),
    CFG_CHARGE_PUMP_FREQ: ("Charge pump freq", VAL_UINT16),
    CFG_SPINDLE_PWM_FREQ: ("Spindle PWM freq", VAL_UINT16),
    CFG_SPINDLE_MAX_RPM: ("Spindle max RPM", VAL_UINT32),
    # Pin assignments
    CFG_PIN_STEP_X: ("Pin Step X", VAL_UINT8),
    CFG_PIN_STEP_Y: ("Pin Step Y", VAL_UINT8),
    CFG_PIN_STEP_Z: ("Pin Step Z", VAL_UINT8),
    CFG_PIN_STEP_A: ("Pin Step A", VAL_UINT8),
    CFG_PIN_STEP_B: ("Pin Step B", VAL_UINT8),
    CFG_PIN_STEP_C: ("Pin Step C", VAL_UINT8),
    CFG_PIN_DIR_X: ("Pin Dir X", VAL_UINT8),
    CFG_PIN_DIR_Y: ("Pin Dir Y", VAL_UINT8),
    CFG_PIN_DIR_Z: ("Pin Dir Z", VAL_UINT8),
    CFG_PIN_DIR_A: ("Pin Dir A", VAL_UINT8),
    CFG_PIN_DIR_B: ("Pin Dir B", VAL_UINT8),
    CFG_PIN_DIR_C: ("Pin Dir C", VAL_UINT8),
    CFG_PIN_ENABLE: ("Pin Enable", VAL_UINT8),
    CFG_PIN_LIMIT_X: ("Pin Limit X", VAL_UINT8),
    CFG_PIN_LIMIT_Y: ("Pin Limit Y", VAL_UINT8),
    CFG_PIN_LIMIT_Z: ("Pin Limit Z", VAL_UINT8),
    CFG_PIN_LIMIT_A: ("Pin Limit A", VAL_UINT8),
    CFG_PIN_LIMIT_B: ("Pin Limit B", VAL_UINT8),
    CFG_PIN_LIMIT_C: ("Pin Limit C", VAL_UINT8),
    CFG_PIN_PROBE: ("Pin Probe", VAL_UINT8),
    CFG_PIN_ESTOP: ("Pin E-Stop", VAL_UINT8),
    CFG_PIN_SPINDLE: ("Pin Spindle", VAL_UINT8),
    CFG_PIN_LED: ("Pin LED", VAL_UINT8),
    CFG_PIN_CHARGE_PUMP: ("Pin Charge Pump", VAL_UINT8),
    CFG_PIN_MISC_OUT0: ("Pin Misc Out 0", VAL_UINT8),
    CFG_PIN_MISC_OUT1: ("Pin Misc Out 1", VAL_UINT8),
    # Spindle encoder
    CFG_PIN_ENCODER_A: ("Pin Encoder A", VAL_UINT8),
    CFG_PIN_ENCODER_B: ("Pin Encoder B", VAL_UINT8),
    CFG_PIN_ENCODER_INDEX: ("Pin Encoder Index", VAL_UINT8),
    CFG_ENCODER_PPR: ("Encoder PPR", VAL_UINT16),
    CFG_ENCODER_MODE: ("Encoder Mode", VAL_UINT8),
    CFG_ENCODER_FILTER_NS: ("Encoder Filter ns", VAL_UINT16),
    # Misc input pins
    CFG_PIN_MISC_IN0: ("Pin Misc In 0", VAL_UINT8),
    CFG_PIN_MISC_IN1: ("Pin Misc In 1", VAL_UINT8),
    CFG_PIN_MISC_IN2: ("Pin Misc In 2", VAL_UINT8),
    CFG_PIN_MISC_IN3: ("Pin Misc In 3", VAL_UINT8),
    # Device mode
    CFG_DEVICE_MODE: ("Device Mode", VAL_UINT8),
    # I/O module
    CFG_IO_PIN_COUNT: ("IO Pin Count", VAL_UINT8),
    CFG_IO_DIR_MASK: ("IO Dir Mask", VAL_UINT16),
    CFG_IO_PULLUP_MASK: ("IO Pullup Mask", VAL_UINT16),
    CFG_IO_INVERT_MASK: ("IO Invert Mask", VAL_UINT16),
}

# Machine states
STATES = {
    0: "IDLE", 1: "RUN", 2: "HOLD", 3: "JOG",
    4: "HOMING", 5: "PROBING", 6: "ALARM", 7: "ESTOP", 8: "DISCONNECTED"
}

MAX_AXES = 6
sequence = 0


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT matching the C implementation."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_header(pkt_type: int, payload_len: int) -> bytes:
    """Build an 18-byte packet header."""
    global sequence
    seq = sequence
    sequence += 1
    timestamp = int(time.time() * 1000000) & 0xFFFFFFFF

    # Pack header with checksum=0
    hdr = struct.pack('<IBBHIIH',
                      WCNC_MAGIC,       # magic (4)
                      WCNC_VERSION,     # version (1)
                      pkt_type,         # packet_type (1)
                      payload_len,      # payload_length (2)
                      seq,              # sequence (4)
                      timestamp,        # timestamp_us (4)
                      0)                # checksum (2) - placeholder
    return hdr


def finalize_packet(packet: bytes) -> bytes:
    """Compute and insert CRC-16 checksum into packet."""
    # Zero out checksum bytes (last 2 of 18-byte header)
    mutable = bytearray(packet)
    mutable[16] = 0
    mutable[17] = 0
    crc = crc16_ccitt(bytes(mutable))
    struct.pack_into('<H', mutable, 16, crc)
    return bytes(mutable)


def validate_packet(data: bytes) -> bool:
    """Validate a received packet's checksum."""
    if len(data) < 18:
        return False
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != WCNC_MAGIC:
        return False
    mutable = bytearray(data)
    received_crc = struct.unpack_from('<H', data, 16)[0]
    mutable[16] = 0
    mutable[17] = 0
    computed_crc = crc16_ccitt(bytes(mutable))
    return computed_crc == received_crc


# ===================================================================
# TCP Functions
# ===================================================================

def tcp_send(sock: socket.socket, packet: bytes):
    """Send a length-prefixed TCP packet."""
    length = struct.pack('<H', len(packet))
    sock.sendall(length + packet)


def tcp_recv(sock: socket.socket) -> bytes:
    """Receive a length-prefixed TCP packet."""
    len_buf = b''
    while len(len_buf) < 2:
        chunk = sock.recv(2 - len(len_buf))
        if not chunk:
            raise ConnectionError("TCP connection closed")
        len_buf += chunk

    pkt_len = struct.unpack('<H', len_buf)[0]
    data = b''
    while len(data) < pkt_len:
        chunk = sock.recv(pkt_len - len(data))
        if not chunk:
            raise ConnectionError("TCP connection closed")
        data += chunk

    return data


def do_handshake(ip: str) -> dict:
    """Perform TCP handshake with ESP32."""
    print(f"Connecting to {ip}:{TCP_CONTROL_PORT}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((ip, TCP_CONTROL_PORT))
    print("TCP connected")

    # Build handshake request
    host_name = b"ProtocolTest\x00" + b'\x00' * 20  # 32 bytes
    payload = struct.pack('<I', 0x01000000) + host_name  # host_version + host_name
    header = build_header(PKT_HANDSHAKE_REQ, len(payload))
    packet = finalize_packet(header + payload)

    tcp_send(sock, packet)
    print("Handshake request sent")

    # Receive response
    resp_data = tcp_recv(sock)
    if not validate_packet(resp_data):
        print("ERROR: Invalid handshake response checksum")
        sock.close()
        return {}

    if len(resp_data) < 18 + 44:  # header + response payload
        print(f"ERROR: Handshake response too short ({len(resp_data)} bytes)")
        sock.close()
        return {}

    # Parse response
    fw_ver = struct.unpack_from('<I', resp_data, 18)[0]
    num_axes = resp_data[22]
    caps = resp_data[23]
    buf_cap = struct.unpack_from('<H', resp_data, 24)[0]
    max_rate = struct.unpack_from('<I', resp_data, 26)[0]
    dev_name = resp_data[30:62].split(b'\x00')[0].decode('ascii', errors='replace')

    result = {
        'firmware_version': f"{(fw_ver>>24)&0xFF}.{(fw_ver>>16)&0xFF}.{fw_ver&0xFFFF}",
        'num_axes': num_axes,
        'capabilities': caps,
        'buffer_capacity': buf_cap,
        'max_step_rate': max_rate,
        'device_name': dev_name,
        'socket': sock,
    }

    # Extended handshake fields (v1.1)
    device_mode = 0
    encoder_ppr = 0
    io_channel_count = 0
    if len(resp_data) >= 18 + 48:
        device_mode = resp_data[62]
        encoder_ppr = (resp_data[63] << 8) | resp_data[64]
        io_channel_count = resp_data[65]

    result['device_mode'] = device_mode
    result['encoder_ppr'] = encoder_ppr
    result['io_channel_count'] = io_channel_count

    mode_str = "I/O module" if device_mode == 1 else "motion controller"
    print(f"\n=== Handshake Response ===")
    print(f"  Device:           {dev_name}")
    print(f"  Firmware:         {result['firmware_version']}")
    print(f"  Mode:             {mode_str}")
    print(f"  Axes:             {num_axes}")
    print(f"  Buffer capacity:  {buf_cap} segments")
    print(f"  Max step rate:    {max_rate} steps/sec")
    print(f"  Capabilities:     0x{caps:02X}")
    if encoder_ppr > 0:
        print(f"  Encoder PPR:      {encoder_ppr}")
    if io_channel_count > 0:
        print(f"  I/O channels:     {io_channel_count}")
    print()

    return result


# ===================================================================
# UDP Functions
# ===================================================================

def send_udp(ip: str, packet: bytes):
    """Send a UDP packet to the ESP32 motion port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(packet, (ip, UDP_MOTION_PORT))
    sock.close()


def listen_status(ip: str, duration: float = 10.0):
    """Listen for UDP status reports."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_STATUS_PORT))
    sock.settimeout(1.0)

    # Send a ping via UDP to register our address
    ping_hdr = build_header(PKT_RESET, 0)
    ping_pkt = finalize_packet(ping_hdr)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.sendto(ping_pkt, (ip, UDP_MOTION_PORT))
    udp_sock.close()

    print(f"Listening for status reports on port {UDP_STATUS_PORT}...")
    print(f"{'Time':>8}  {'State':>12}  {'BufAvail':>8}  {'X':>10}  {'Y':>10}  {'Z':>10}  "
          f"{'FeedRate':>10}  {'Limits':>6}  {'Home':>6}  {'EStop':>5}  "
          f"{'MiscOut':>7}  {'MiscIn':>6}  {'Spndl':>5}  {'Cool':>4}")
    print("-" * 140)

    start = time.time()
    while time.time() - start < duration:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue

        if not validate_packet(data):
            continue

        pkt_type = data[5]
        if pkt_type != PKT_STATUS_REPORT:
            continue

        if len(data) < 18 + 46:
            continue

        # Parse status report (offset 18 = after header)
        off = 18
        positions = struct.unpack_from('<6i', data, off)
        off += 24
        buf_avail, buf_total = struct.unpack_from('<HH', data, off)
        off += 4
        state, alarm, limits, probe, homing, home_sw, estop_in, flags = struct.unpack_from('<8B', data, off)
        off += 8
        seg_id = struct.unpack_from('<H', data, off)[0]
        off += 2
        uptime = struct.unpack_from('<I', data, off)[0]
        off += 4
        feed_rate = struct.unpack_from('<i', data, off)[0]
        off += 4

        # Extended fields (v1.1) - backward compatible
        misc_out = misc_in = spindle_st = coolant_st = 0
        if len(data) >= 18 + 50:
            misc_out, misc_in, spindle_st, coolant_st = struct.unpack_from('<4B', data, off)
            off += 4

        # Spindle encoder + I/O module (v1.1, 12 bytes)
        enc_rpm = enc_pos = 0
        enc_idx = 0
        io_in = io_out = 0
        if len(data) >= 18 + 62:
            enc_rpm, enc_pos, enc_idx, io_in, io_out = struct.unpack_from('<HHIHH', data, off)

        elapsed = time.time() - start
        state_name = STATES.get(state, f"?{state}")
        print(f"{elapsed:8.2f}  {state_name:>12}  {buf_avail:>8}  "
              f"{positions[0]:>10}  {positions[1]:>10}  {positions[2]:>10}  "
              f"{feed_rate:>10}  0x{limits:02X}  0x{home_sw:02X}  {'YES' if estop_in else 'no':>5}  "
              f"0x{misc_out:02X}  0x{misc_in:02X}  {spindle_st:>5}  {coolant_st:>4}"
              f"  RPM:{enc_rpm:>5}" if enc_rpm else "")

    sock.close()
    print("\nDone listening.")


def send_jog(ip: str, axis: int = 0, direction: int = 1,
             speed: int = 1000, duration: float = 2.0):
    """Send jog command, wait, then stop."""
    # Start jog
    payload = struct.pack('<bbbxI', axis, direction, 0, speed)
    header = build_header(PKT_JOG_COMMAND, len(payload))
    packet = finalize_packet(header + payload)
    send_udp(ip, packet)
    print(f"Jog started: axis={axis} dir={direction} speed={speed} steps/sec")

    time.sleep(duration)

    # Stop jog
    payload = struct.pack('<Bxxx', axis)
    header = build_header(PKT_JOG_STOP, len(payload))
    packet = finalize_packet(header + payload)
    send_udp(ip, packet)
    print(f"Jog stopped after {duration}s")


def send_motion_segment(ip: str, steps_x: int = 1000, speed: float = 500.0):
    """Send a single motion segment."""
    # Build motion segment
    steps = [steps_x, 0, 0, 0, 0, 0]
    duration_us = int(abs(steps_x) / speed * 1000000)
    speed_sqr = int(speed * speed * 1000)
    accel = 500000  # 5000 * 100

    seg_data = struct.pack('<6iIIIIHBB',
                           *steps,
                           duration_us,
                           speed_sqr,      # entry_speed_sqr
                           speed_sqr,      # exit_speed_sqr
                           accel,          # acceleration
                           0,              # segment_id
                           0,              # flags
                           0)              # reserved

    # Build motion packet (1 segment)
    seg_count_data = struct.pack('<Bxxx', 1)
    payload = seg_count_data + seg_data
    header = build_header(PKT_MOTION_SEGMENT, len(payload))
    packet = finalize_packet(header + payload)

    send_udp(ip, packet)
    print(f"Motion segment sent: X={steps_x} steps at {speed} steps/sec "
          f"({duration_us/1000:.1f}ms)")


def send_estop(ip: str):
    """Send E-Stop (3x redundant)."""
    for i in range(3):
        header = build_header(PKT_ESTOP, 0)
        packet = finalize_packet(header)
        send_udp(ip, packet)
    print("E-STOP sent (3x)")


def send_reset(ip: str):
    """Send reset command."""
    header = build_header(PKT_RESET, 0)
    packet = finalize_packet(header)
    send_udp(ip, packet)
    print("Reset sent")


def send_home(ip: str, axis_mask: int = 0x3F):
    """Send home command."""
    payload = struct.pack('<Bxxx', axis_mask)
    header = build_header(PKT_HOME_COMMAND, len(payload))
    packet = finalize_packet(header + payload)
    send_udp(ip, packet)
    print(f"Home command sent: axis_mask=0x{axis_mask:02X}")


# ===================================================================
# TCP Config Functions
# ===================================================================

def send_config_get(sock: socket.socket, key: int):
    """Send a CONFIG_GET packet and return (value_type, value_bytes)."""
    payload = struct.pack('<HH', key, 0) + b'\x00' * CONFIG_VALUE_LEN
    header = build_header(PKT_CONFIG_GET, len(payload))
    packet = finalize_packet(header + payload)
    tcp_send(sock, packet)
    resp = tcp_recv(sock)
    if not validate_packet(resp) or resp[5] != PKT_CONFIG_RESP:
        return None, None
    # Parse: header(18) + key(2) + value_type(2) + value(64)
    if len(resp) < 18 + 4 + CONFIG_VALUE_LEN:
        return None, None
    resp_key, resp_vtype = struct.unpack_from('<HH', resp, 18)
    value_data = resp[22:22 + CONFIG_VALUE_LEN]
    return resp_vtype, value_data


def format_config_value(vtype, value_data):
    """Format a config value for display based on its type."""
    if vtype == VAL_UINT8:
        return f"0x{value_data[0]:02X} ({value_data[0]})"
    elif vtype == VAL_UINT16:
        v = struct.unpack_from('<H', value_data, 0)[0]
        return str(v)
    elif vtype == VAL_UINT32:
        v = struct.unpack_from('<I', value_data, 0)[0]
        return str(v)
    elif vtype == VAL_INT32:
        v = struct.unpack_from('<i', value_data, 0)[0]
        return str(v)
    elif vtype == VAL_FLOAT:
        v = struct.unpack_from('<f', value_data, 0)[0]
        return f"{v:.4f}"
    elif vtype == VAL_STRING:
        return value_data.split(b'\x00')[0].decode('utf-8', errors='replace')
    return f"(unknown type {vtype})"


def send_config_set(sock: socket.socket, key: int, value, value_type=None):
    """Send a CONFIG_SET packet over TCP.

    value can be a string (auto-detected as VAL_STRING) or a numeric value.
    If value_type is not specified, it is looked up from CONFIG_KEYS.
    """
    if isinstance(value, str):
        vtype = VAL_STRING
        value_bytes = value.encode('utf-8') + b'\x00'
        value_padded = value_bytes.ljust(CONFIG_VALUE_LEN, b'\x00')[:CONFIG_VALUE_LEN]
    else:
        vtype = value_type if value_type is not None else CONFIG_KEYS.get(key, (None, VAL_UINT32))[1]
        value_padded = b'\x00' * CONFIG_VALUE_LEN
        if vtype == VAL_UINT8:
            value_padded = struct.pack('<B', int(value)) + b'\x00' * (CONFIG_VALUE_LEN - 1)
        elif vtype == VAL_UINT16:
            value_padded = struct.pack('<H', int(value)) + b'\x00' * (CONFIG_VALUE_LEN - 2)
        elif vtype == VAL_UINT32:
            value_padded = struct.pack('<I', int(value)) + b'\x00' * (CONFIG_VALUE_LEN - 4)
        elif vtype == VAL_INT32:
            value_padded = struct.pack('<i', int(value)) + b'\x00' * (CONFIG_VALUE_LEN - 4)
        elif vtype == VAL_FLOAT:
            value_padded = struct.pack('<f', float(value)) + b'\x00' * (CONFIG_VALUE_LEN - 4)

    payload = struct.pack('<HH', key, vtype) + value_padded
    header = build_header(PKT_CONFIG_SET, len(payload))
    packet = finalize_packet(header + payload)
    tcp_send(sock, packet)
    resp = tcp_recv(sock)
    if validate_packet(resp) and resp[5] == PKT_ACK:
        return True
    return False


def send_config_save(sock: socket.socket):
    """Send a CONFIG_SAVE packet over TCP."""
    header = build_header(PKT_CONFIG_SAVE, 0)
    packet = finalize_packet(header)
    tcp_send(sock, packet)
    resp = tcp_recv(sock)
    if validate_packet(resp) and resp[5] == PKT_ACK:
        return True
    return False


def do_config_read(ip: str):
    """Read and display all configuration values from ESP32."""
    result = do_handshake(ip)
    sock = result.get('socket')
    if not sock:
        print("ERROR: Handshake failed")
        return

    print("=== Configuration ===")
    print(f"{'Key':>6}  {'Name':<22}  {'Value'}")
    print("-" * 60)

    for key in sorted(CONFIG_KEYS.keys()):
        name, expected_type = CONFIG_KEYS[key]
        vtype, vdata = send_config_get(sock, key)
        if vtype is not None:
            val_str = format_config_value(vtype, vdata)
            print(f"0x{key:04X}  {name:<22}  {val_str}")
        else:
            print(f"0x{key:04X}  {name:<22}  (read failed)")

    sock.close()
    print()
    print("Done.")


def do_wifi_setup(ip: str, ssid: str, password: str):
    """Set WiFi credentials and save. ESP32 will auto-reboot."""
    result = do_handshake(ip)
    sock = result.get('socket')
    if not sock:
        print("ERROR: Handshake failed")
        return

    print(f"Setting WiFi SSID: '{ssid}'")
    if not send_config_set(sock, CFG_WIFI_SSID, ssid):
        print("ERROR: Failed to set SSID")
        sock.close()
        return

    print(f"Setting WiFi password: {'*' * len(password)}")
    if not send_config_set(sock, CFG_WIFI_PASSWORD, password):
        print("ERROR: Failed to set password")
        sock.close()
        return

    print("Saving configuration...")
    if send_config_save(sock):
        print("Config saved. ESP32 will reboot in ~1 second to join your network.")
    else:
        print("ERROR: Config save failed")

    sock.close()


# ===================================================================
# Discovery
# ===================================================================

def discover_esp32(timeout: float = 5.0) -> str:
    """Broadcast a UDP packet and listen for status report to find ESP32 IP."""
    # Build a harmless JOG_STOP packet (no-op when not jogging)
    payload = struct.pack('<Bxxx', 0)
    header = build_header(PKT_JOG_STOP, len(payload))
    packet = finalize_packet(header + payload)

    # Send broadcast on motion port
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    tx_sock.sendto(packet, ('255.255.255.255', UDP_MOTION_PORT))
    tx_sock.close()

    # Listen for status report response
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(('0.0.0.0', UDP_STATUS_PORT))
    rx_sock.settimeout(1.0)

    print(f"Searching for ESP32 on the network (up to {timeout:.0f}s)...")

    found = set()
    start = time.time()
    while time.time() - start < timeout:
        try:
            data, addr = rx_sock.recvfrom(256)
        except socket.timeout:
            # Re-broadcast periodically
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            tx.sendto(packet, ('255.255.255.255', UDP_MOTION_PORT))
            tx.close()
            continue

        if not validate_packet(data):
            continue
        if data[5] != PKT_STATUS_REPORT:
            continue

        ip = addr[0]
        if ip not in found:
            found.add(ip)
            print(f"  Found ESP32 at: {ip}")

    rx_sock.close()

    if not found:
        print("No ESP32 devices found. Check that the ESP32 is powered on")
        print("and connected to the same network.")
        return ""

    result = list(found)[0]
    if len(found) > 1:
        print(f"\nMultiple devices found. Using first: {result}")
    else:
        print(f"\nESP32 discovered at: {result}")
    return result


# ===================================================================
# Main
# ===================================================================

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Handle discover as a special case (no IP needed)
    if sys.argv[1] == "discover":
        discover_esp32()
        return

    ip = sys.argv[1]
    command = sys.argv[2] if len(sys.argv) > 2 else "handshake"

    if command == "handshake":
        result = do_handshake(ip)
        if result.get('socket'):
            result['socket'].close()

    elif command == "status":
        duration = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
        listen_status(ip, duration)

    elif command == "jog":
        send_jog(ip)

    elif command == "motion":
        steps = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
        speed = float(sys.argv[4]) if len(sys.argv) > 4 else 500.0
        send_motion_segment(ip, steps, speed)

    elif command == "estop":
        send_estop(ip)

    elif command == "reset":
        send_reset(ip)

    elif command == "home":
        send_home(ip)

    elif command == "config":
        do_config_read(ip)

    elif command == "config-set":
        if len(sys.argv) < 5:
            print("Usage: protocol_test.py <IP> config-set <key_hex> <value>")
            print("  key_hex: config key in hex (e.g. 0x0310 for invert_limit)")
            print("  value:   value to set (auto-detected type from key)")
            sys.exit(1)
        key = int(sys.argv[3], 0)
        value_str = sys.argv[4]
        result = do_handshake(ip)
        sock = result.get('socket')
        if not sock:
            print("ERROR: Handshake failed")
            sys.exit(1)
        key_info = CONFIG_KEYS.get(key)
        if key_info:
            name, vtype = key_info
        else:
            name, vtype = f"0x{key:04X}", VAL_UINT32
        # Parse value based on type
        if vtype == VAL_STRING:
            val = value_str
        elif vtype == VAL_FLOAT:
            val = float(value_str)
        else:
            val = int(value_str, 0)
        if send_config_set(sock, key, val, vtype):
            print(f"Set {name} = {value_str}")
            print("Saving to NVS...")
            if send_config_save(sock):
                print("Saved.")
            else:
                print("Save failed.")
        else:
            print(f"Failed to set {name}")
        sock.close()

    elif command == "wifi-setup":
        if len(sys.argv) < 5:
            print("Usage: protocol_test.py <IP> wifi-setup <SSID> <password>")
            sys.exit(1)
        ssid = sys.argv[3]
        password = sys.argv[4]
        do_wifi_setup(ip, ssid, password)

    elif command == "demo":
        # Full demo: handshake, send motion, listen for status
        result = do_handshake(ip)
        if result.get('socket'):
            result['socket'].close()
            time.sleep(0.5)

            print("\n--- Sending test motion ---")
            send_motion_segment(ip, 2000, 1000.0)

            print("\n--- Listening for status ---")
            listen_status(ip, 5.0)

    else:
        print(f"Unknown command: {command}")
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
