"""
Tiggy DevKitC carrier, revision A: the circuit and the placement, in one place.

Everything else is generated from this file (see build.py): the project's own
symbol and footprint library, the schematic, and - from the netlist KiCad exports
from that schematic - the board.

A part is  P(ref, symbol, value, footprint, pins, group, pcb)
  pins   {pin number: net name}; a pin left out is not connected
  group  the block of the schematic it is drawn in
  pcb    (x, y, rotation) in mm from the board's top left corner, rotation in
         degrees counter-clockwise as KiCad shows it
"""

NAME = "tiggy-devkitc-carrier"
REV = "A"
BOARD_W, BOARD_H = 182.0, 95.0
RX = 37.0                                               # the relays and the supply, along the right edge

PARTS = []


def P(ref, sym, value, fp, pins, group, pcb, **fields):
    assert ref not in [p["ref"] for p in PARTS], ref
    PARTS.append(dict(ref=ref, sym=sym, value=value, fp=fp, pins={str(k): v for k, v in pins.items()},
                      group=group, pcb=pcb, fields=fields))


R0603 = "Resistor_SMD:R_0603_1608Metric"
R0805 = "Resistor_SMD:R_0805_2012Metric"
R1206 = "Resistor_SMD:R_1206_3216Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
LED0805 = "LED_SMD:LED_0805_2012Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
MC4 = "Connector_Phoenix_MC:PhoenixContact_MC_1,5_4-G-3.81_1x04_P3.81mm_Horizontal"
MC6 = "Connector_Phoenix_MC:PhoenixContact_MC_1,5_6-G-3.81_1x06_P3.81mm_Horizontal"
MSTB2 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal"
MSTB3 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G-5,08_1x03_P5.08mm_Horizontal"
SOIC14 = "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm"
SOIC20W = "Package_SO:SOIC-20W_7.5x12.8mm_P1.27mm"
HEADER3 = "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical"

_n = {}


def ref(prefix):
    _n[prefix] = _n.get(prefix, 0) + 1
    return "%s%d" % (prefix, _n[prefix])


def R(value, a, b, group, pcb, fp=R0603):
    P(ref("R"), "Device:R", value, fp, {1: a, 2: b}, group, pcb)


def C(value, a, b, group, pcb, fp=C0603):
    P(ref("C"), "Device:C", value, fp, {1: a, 2: b}, group, pcb)


def LED(value, anode, cathode, group, pcb):
    P(ref("D"), "Device:LED", value, LED0805, {2: anode, 1: cathode}, group, pcb)


# ---------------------------------------------------------------------------
# The ESP32-S3-DevKitC-1 (octal memory: firmware environment esp32s3-devkitc).
# Pin numbers of the symbol: 1-22 are header J1 from the aerial end, 23-44
# header J3 from the aerial end.
#
# The firmware has two terminal layouts for it.  With "six axes" GPIO 40, 11,
# 39 and 12 are step and direction of the B and C drives; with "four axes" they
# are relay 3, relay 1, the charge pump and relay 2.  Four jumpers (SELECT,
# below) send each of them one way or the other.
# ---------------------------------------------------------------------------
DEVKIT_J1 = ["3V3", "3V3", "RST", "IO4", "IO5", "IO6", "IO7", "IO15", "IO16", "IO17", "IO18", "IO8", "IO3", "IO46",
             "IO9", "IO10", "IO11", "IO12", "IO13", "IO14", "5V", "GND"]
DEVKIT_J3 = ["GND", "TX", "RX", "IO1", "IO2", "IO42", "IO41", "IO40", "IO39", "IO38", "IO37", "IO36", "IO35", "IO0",
             "IO45", "IO48", "IO47", "IO21", "IO20", "IO19", "GND", "GND"]
GPIO_NET = {
    "3V3": "+3V3", "5V": "5V_ESP", "GND": "GND", "RST": "EN",
    "IO1": "STEP_X", "IO2": "STEP_Y", "IO42": "STEP_Z", "IO41": "STEP_A",
    "IO3": "DIR_X", "IO4": "DIR_Y", "IO38": "DIR_Z", "IO10": "DIR_A",
    "IO8": "ENABLE", "IO9": "SPINDLE",
    "IO40": "IO40", "IO11": "IO11", "IO39": "IO39", "IO12": "IO12",      # to the jumpers
    "IO5": "LIM_X", "IO6": "LIM_Y", "IO7": "LIM_Z", "IO15": "LIM_A",
    "IO14": "PROBE", "IO13": "ESTOP", "IO18": "AUX1",
    "IO16": "ENC_A", "IO17": "ENC_B", "IO21": "ENC_Z",
    "IO45": "ETH_MOSI", "IO46": "ETH_MISO", "IO0": "ETH_SCLK", "IO47": "ETH_INT",
    # not used: TX, RX (the USB serial port), IO35-37 (memory), IO48 (LED), IO19/20 (USB)
}
# (pin of the ESP32, what it is with six axes, what it is with four)
SELECT = [("IO40", "STEP_B", "OUT3"), ("IO11", "DIR_B", "OUT1"), ("IO39", "STEP_C", "CPUMP"), ("IO12", "DIR_C", "OUT2")]

DK_X, DK_Y = 46.57, 33.66                               # header J1 pin 1 (aerial end, left row)
DK_ROW = 22.86


def dk_pin(name):
    """Board position of a DevKitC header pin, by its name."""
    if name in DEVKIT_J1:
        return DK_X, DK_Y + 2.54 * DEVKIT_J1.index(name)
    return DK_X + DK_ROW, DK_Y + 2.54 * DEVKIT_J3.index(name)


pins = {}
for i, n in enumerate(DEVKIT_J1 + DEVKIT_J3):
    if n in GPIO_NET:
        pins[i + 1] = GPIO_NET[n]
P("A1", "tiggy:ESP32-S3-DevKitC-1", "ESP32-S3-DevKitC-1", "tiggy:ESP32-S3-DevKitC-1_Socket", pins, "Controller",
  (DK_X, DK_Y, 0))

# ---------------------------------------------------------------------------
# Supply: 12 to 24 V in, 5 V for the board, the DevKitC fed through a diode so
# that its USB port never supplies the relays.
# ---------------------------------------------------------------------------
G = "Supply"
P("J1", "Connector_Generic:Conn_01x02", "12-24V", MSTB2, {1: "VIN_IN", 2: "GND"}, G, (RX + 134.4, 80.9, 90))
P("F1", "Device:Polyfuse", "1.1A 33V", "Fuse:Fuse_1812_4532Metric", {1: "VIN_IN", 2: "VIN_F"}, G, (RX + 126.8, 88.6, 0))
P(ref("D"), "Diode:SS34", "SS34", "Diode_SMD:D_SMA", {2: "VIN_F", 1: "VIN"}, G, (RX + 119.2, 88.6, 180))
P(ref("D"), "Device:D_TVS", "SMBJ28CA", "Diode_SMD:D_SMB", {1: "VIN", 2: "GND"}, G, (RX + 125.6, 72.0, 90))
P(ref("C"), "Device:C_Polarized", "100u 35V", "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm", {1: "VIN", 2: "GND"}, G,
  (RX + 109.0, 89.3, 0))
C("10u 50V", "VIN", "GND", G, (RX + 111.0, 83.2, 0), C1206)
P("U1", "Converter_DCDC:R-78E5.0-1.0", "R-78E5.0-1.0", "Converter_DCDC:Converter_DCDC_RECOM_R-78E-0.5_THT",
  {1: "VIN", 2: "GND", 3: "+5V"}, G, (RX + 113.0, 78.0, 0))
C("22u 10V", "+5V", "GND", G, (RX + 119.6, 83.2, 0), C1206)
P(ref("D"), "Diode:SS34", "SS34", "Diode_SMD:D_SMA", {2: "+5V", 1: "5V_ESP"}, G, (58.0, 90.0, 0))
R("1k", "+5V", "PWR_LED", G, (RX + 117.5, 69.6, 0))
LED("green", "PWR_LED", "GND", G, (RX + 113.5, 69.6, 0))
C("10u", "+3V3", "GND", G, (78.0, 81.0, 90), C1206)

# ---------------------------------------------------------------------------
# Step and direction for the drives, 5 V push-pull: X, Y, Z and A always, B and
# C when the jumpers are set for six axes.  The pull-downs keep the outputs
# quiet while the ESP32 is in reset (and B and C quiet with four axes).
# ---------------------------------------------------------------------------
G = "Drives"
U2 = (94.5, 20.4)
P("U2", "74xx:74AHCT541", "74AHCT541", SOIC20W, {
    1: "GND", 19: "GND", 20: "+5V", 10: "GND",
    2: "DIR_X", 3: "STEP_X", 4: "DIR_Y", 5: "STEP_Y", 6: "DIR_Z", 7: "STEP_Z", 8: "DIR_A", 9: "STEP_A",
    18: "X_DIR", 17: "X_STEP", 16: "Y_DIR", 15: "Y_STEP", 14: "Z_DIR", 13: "Z_STEP", 12: "A_DIR", 11: "A_STEP",
}, G, (U2[0], U2[1], 90))
C("100n", "+5V", "GND", G, (86.4, 18.0, 90))
DRIVES = [("X", "J2"), ("Y", "J3"), ("Z", "J4"), ("A", "J5"), ("B", "J13"), ("C", "J14")]
for i, (axis, plug) in enumerate(DRIVES):
    P(plug, "Connector_Generic:Conn_01x04", "%s drive" % axis, MC4,
      {1: "%s_STEP" % axis, 2: "%s_DIR" % axis, 3: "%s_EN" % axis, 4: "GND"}, "Drives" if axis in "XYZA" else "Drives B, C",
      (75.97 + 18.57 * i, 8.6, 180))
# pull-downs, each beside the DevKitC pin it belongs to, inside the socket rows: the signal's end
# towards the pin, the 0 V end towards the open middle of the board
for name in ("IO1", "IO2", "IO42", "IO41", "IO38"):
    x, y = dk_pin(name)
    R("10k", GPIO_NET[name], "GND", G, (x - 3.6, y, 180))
for name in ("IO4", "IO8", "IO3", "IO9", "IO10"):
    x, y = dk_pin(name)
    R("10k", GPIO_NET[name], "GND", G, (x + 3.6, y, 0))

# Enable: a buffer for each drive, so that none has more than one drive's input to supply
G = "Enable"
P("U7", "74xx:74AHCT541", "74AHCT541", SOIC20W, {
    1: "GND", 19: "GND", 20: "+5V", 10: "GND",
    2: "ENABLE", 3: "ENABLE", 4: "ENABLE", 5: "ENABLE", 6: "ENABLE", 7: "ENABLE", 8: "GND", 9: "GND",
    18: "X_EN", 17: "Y_EN", 16: "Z_EN", 15: "A_EN", 14: "B_EN", 13: "C_EN",
}, G, (112.0, 20.4, 90))
C("100n", "+5V", "GND", G, (103.9, 18.0, 90))

# B and C, and the jumpers that give them their four pins: one three-pin header
# for each pin, side by side, so that the four links are all moved the same way
G = "Drives B, C"
P("U6", "74xx:74AHCT125", "74AHCT125", SOIC14, {
    1: "GND", 2: "STEP_B", 3: "B_STEP", 4: "GND", 5: "DIR_B", 6: "B_DIR",
    10: "GND", 9: "STEP_C", 8: "C_STEP", 13: "GND", 12: "DIR_C", 11: "C_DIR", 14: "+5V", 7: "GND",
}, G, (129.0, 20.4, 90))
C("100n", "+5V", "GND", G, (122.8, 19.0, 90))
JUMPERS, JUMPER_PITCH = (96.0, 40.0), 5.08               # pin 1 of the first: pin 1 six axes, pin 3 four
for i, (pin, six, four) in enumerate(SELECT):
    P("JP%d" % (2 + i), "Jumper:Jumper_3_Bridged12", "3-pin header, jumper", HEADER3, {2: pin, 1: six, 3: four}, G,
      (JUMPERS[0] + JUMPER_PITCH * i, JUMPERS[1], 0))
    R("10k", six, "GND", G, (125.2 + 2.54 * i, 26.6, 270))

# ---------------------------------------------------------------------------
# Inputs: seven opto-couplers.  A switch or an NPN sensor between the terminal
# and 0 V lights the channel's LED and pulls the ESP32's input low.
# ---------------------------------------------------------------------------
G = "Inputs"
CHANNELS = [("X", "LIM_X"), ("Y", "LIM_Y"), ("Z", "LIM_Z"), ("A", "LIM_A"), ("AUX1", "AUX1"), ("ESTOP", "ESTOP"),
            ("PROBE", "PROBE")]
for i, (name, net) in enumerate(CHANNELS):
    y = 31.2 + 6.9 * i
    R("4k7", "VIN", "IN_%s_A" % name, G, (17.5, y - 1.27, 0), R1206)
    LED("yellow", "IN_%s_A" % name, "IN_%s_B" % name, G, (22.7, y - 1.27, 180))
    P("OK%d" % (i + 1), "Isolator:LTV-817S", "LTV-817S", "Package_DIP:SMDIP-4_W9.53mm",
      {1: "IN_%s_B" % name, 2: "IN_%s" % name, 3: "GND", 4: net}, G, (31.3, y, 0))
    R("10k", "+3V3", net, G, (39.8, y - 1.6, 0))
    C("10n", net, "GND", G, (39.8, y + 1.6, 0))
P("J6", "Connector_Generic:Conn_01x06", "Limits", MC6,
  {1: "VIN", 2: "GND", 3: "IN_X", 4: "IN_Y", 5: "IN_Z", 6: "IN_A"}, G, (8.6, 31.5, 270))
P("J7", "Connector_Generic:Conn_01x06", "Inputs", MC6,
  {1: "VIN", 2: "GND", 3: "IN_AUX1", 4: "IN_ESTOP", 5: "IN_PROBE", 6: "GND"}, G, (8.6, 57.5, 270))

# ---------------------------------------------------------------------------
# Spindle sensor: three fast inputs with 5 V pull-ups, clamped, squared up by
# Schmitt triggers running on 3.3 V.  Without a sensor they are auxiliary
# inputs 2 to 4.
# ---------------------------------------------------------------------------
G = "Spindle sensor"
P("J8", "Connector_Generic:Conn_01x06", "Encoder", MC6,
  {1: "+5V", 2: "GND", 3: "ENC_A_T", 4: "ENC_B_T", 5: "ENC_Z_T", 6: "GND"}, G, (31.2, 8.6, 180))
P("U4", "74xx:74HC14", "74LVC14", "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", {
    1: "ENC_A_F", 2: "ENC_A_N", 3: "ENC_A_N", 4: "ENC_A",
    5: "ENC_B_F", 6: "ENC_B_N", 9: "ENC_B_N", 8: "ENC_B",
    13: "ENC_Z_F", 12: "ENC_Z_N", 11: "ENC_Z_N", 10: "ENC_Z",      # (each pair of gates side by side)
    14: "+3V3", 7: "GND",
}, G, (37.5, 21.6, 0))
C("100n", "+3V3", "GND", G, (42.6, 17.0, 90))
for i, ch in enumerate("ABZ"):
    x = 23.58 - 3.81 * i
    R("2k2", "+5V", "ENC_%s_T" % ch, G, (x, 16.4, 90))
    R("4k7", "ENC_%s_T" % ch, "ENC_%s_F" % ch, G, (x, 20.5, 270), R0805)
    P(ref("D"), "Diode:BAT54S", "BAT54S", SOT23, {1: "GND", 2: "+5V", 3: "ENC_%s_F" % ch}, G,
      (x, 24.8, 90))

# ---------------------------------------------------------------------------
# Relays on auxiliary outputs 1 to 3 (with four axes), and the spindle's speed
# as 0 to 10 V.
# ---------------------------------------------------------------------------
G = "Outputs"
for i in range(3):
    n = i + 1
    y = 22.62 + 18.5 * i
    P("K%d" % n, "Relay:G5LE-1", "G5LE-1 5V", "Relay_THT:Relay_SPDT_Omron-G5LE-1",
      {1: "RLY%d_COM" % n, 2: "+5V", 5: "RLY%d_K" % n, 3: "RLY%d_NO" % n, 4: "RLY%d_NC" % n}, G, (RX + 109.0, y, 90))
    P("J%d" % (9 + i), "Connector_Generic:Conn_01x03", "Relay %d" % n, MSTB3,
      {1: "RLY%d_NO" % n, 2: "RLY%d_COM" % n, 3: "RLY%d_NC" % n}, G, (RX + 134.4, y + 5.08, 90))
    # a MOSFET, not a Darlington: the coil gets its full 5 V
    P("Q%d" % (n + 1), "Transistor_FET:AO3400A", "AO3400A", SOT23, {1: "OUT%d" % n, 2: "GND", 3: "RLY%d_K" % n}, G,
      (RX + 103.4, y - 6.2, 0))
    # its gate is held low while the ESP32 is in reset, and when the jumpers are set for six axes
    R("10k", "OUT%d" % n, "GND", G, (RX + 99.8, y - 6.2, 180))
    P(ref("D"), "Diode:1N4148WS", "1N4148WS", "Diode_SMD:D_SOD-323", {1: "+5V", 2: "RLY%d_K" % n}, G,
      (RX + 103.4, y - 2.4, 90))
    LED("red", "RLY%d_L" % n, "RLY%d_K" % n, G, (RX + 103.4, y + 1.6, 270))
    R("1k", "+5V", "RLY%d_L" % n, G, (RX + 103.4, y + 5.4, 90))
C("10u", "+5V", "GND", G, (RX + 97.6, 30.0, 0), C1206)
C("100n", "+5V", "GND", G, (RX + 99.0, 49.6, 0))

G = "Spindle speed"
# 5 V buffers for the spindle's PWM and for the charge pump; the other two gates are not used
P("U3", "74xx:74AHCT125", "74AHCT125", SOIC14, {
    4: "GND", 5: "SPINDLE", 6: "PWM_5V", 10: "GND", 9: "CPUMP", 8: "CP_O",
    1: "+5V", 2: "GND", 13: "+5V", 12: "GND", 14: "+5V", 7: "GND",
}, G, (79.0, 20.4, 90))
C("100n", "+5V", "GND", G, (72.8, 19.0, 90))
R("10k", "CPUMP", "GND", G, (86.4, 22.4, 90))              # low while the jumpers are set for six axes
P("U5", "Amplifier_Operational:LM358", "LM358", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", {
    3: "PWM_F", 2: "AO_FB", 1: "AO_OUT", 5: "GND", 6: "AO_NC", 7: "AO_NC", 8: "VIN", 4: "GND",
}, G, (51.2, 20.2, 0))
R("10k", "PWM_5V", "PWM_F", G, (57.8, 20.4, 0))
C("4u7", "PWM_F", "GND", G, (57.8, 22.4, 0))               # with the 10 k: ripple under 1 % at 1 kHz
R("10k", "AO_OUT", "AO_FB", G, (46.6, 18.9, 270))          # beside the amplifier's pins 1 and 2
R("10k", "AO_FB", "GND", G, (44.3, 19.725, 180))
R("100R", "AO_OUT", "AO_T", G, (51.2, 15.8, 0))
C("100n 50V", "VIN", "GND", G, (57.8, 17.6, 0))
P("J12", "Connector_Generic:Conn_01x06", "Spindle", MC6,
  {1: "AO_T", 2: "GND", 3: "PWM_5V", 4: "CP_O", 5: "+5V", 6: "GND"}, G, (57.4, 8.6, 180))

# ---------------------------------------------------------------------------
# Ethernet: a socket for a USR-ES1 (W5500) module.  The firmware has no pin for
# chip select: it expects the module always selected.  Here the module is only
# selected a moment after the ESP32 leaves reset, so that its data output cannot
# hold the ESP32's GPIO46 high while that is being read as a boot option.
# ---------------------------------------------------------------------------
G = "Ethernet"
ETH_X, ETH_Y = 102.16, 85.1                             # module header J1 pin 1, module turned jack down
P("A2", "tiggy:USR-ES1", "USR-ES1 (W5500)", "tiggy:USR-ES1_Socket", {
    1: "GND", 2: "GND", 3: "ETH_MOSI", 4: "ETH_SCLK", 5: "ETH_CS", 6: "ETH_INT",
    7: "GND", 8: "+3V3", 9: "+3V3", 11: "ETH_RST", 12: "ETH_MISO",
}, G, (ETH_X, ETH_Y, 180))
x, y = dk_pin("IO0")
R("4k7", "+3V3", "ETH_SCLK", G, (x - 3.6, y, 0))        # GPIO0 must be high at reset
x, y = dk_pin("IO47")
R("10k", "+3V3", "ETH_INT", G, (x - 3.6, y, 0))
x, y = dk_pin("IO46")
R("10k", "ETH_MISO", "GND", G, (x + 3.6, y, 0))
P("Q1", "Transistor_FET:2N7002", "2N7002", SOT23, {1: "CS_GATE", 2: "GND", 3: "ETH_CS"}, G, (57.0, 38.2, 0))
R("47k", "EN", "CS_GATE", G, (50.4, 38.74, 0))    # with the 1 uF: selected 20 to 80 ms after reset ends
P(ref("D"), "Diode:1N4148WS", "1N4148WS", "Diode_SMD:D_SOD-323", {2: "CS_GATE", 1: "EN"}, G, (51.6, 36.3, 180))
C("1u", "CS_GATE", "GND", G, (57.0, 35.2, 0))
R("10k", "+3V3", "ETH_CS", G, (61.6, 37.4, 90))
R("1k", "EN", "ETH_RST", G, (56.6, 41.2, 0))
P("JP1", "Jumper:SolderJumper_2_Open", "CS to 0V", "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
  {1: "ETH_CS", 2: "GND"}, G, (92.0, 70.5, 180))
C("100n", "+3V3", "GND", G, (85.6, 80.0, 90))

G = "Mechanical"
for i, (x, y) in enumerate([(4.5, 4.5), (BOARD_W - 4.5, 4.5), (4.5, BOARD_H - 4.5), (BOARD_W - 4.5, BOARD_H - 4.5)]):
    P("H%d" % (i + 1), "Mechanical:MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", {}, G, (x, y, 0))

# Supply nets that need telling KiCad's rule check they are driven
POWER_FLAGS = ["GND", "VIN", "+3V3"]

# Net classes: (name, track width, clearance, via diameter, via drill, [net patterns])
NETCLASSES = [
    ("Default", 0.25, 0.2, 0.6, 0.3, []),
    ("Power", 0.5, 0.2, 0.8, 0.4, ["+5V", "+3V3", "VIN", "VIN_IN", "VIN_F", "5V_ESP", "GND", "RLY?_K"]),
    ("Relay", 1.2, 1.0, 1.2, 0.6, ["RLY?_COM", "RLY?_NO", "RLY?_NC"]),
]

# Terminal names for the silkscreen: connector reference -> text for each pin
TERMINALS = {
    "J1": ["+12-24V", "0V"],
    "J2": ["STEP", "DIR", "EN", "0V"], "J3": ["STEP", "DIR", "EN", "0V"],
    "J4": ["STEP", "DIR", "EN", "0V"], "J5": ["STEP", "DIR", "EN", "0V"],
    "J13": ["STEP", "DIR", "EN", "0V"], "J14": ["STEP", "DIR", "EN", "0V"],
    "J6": ["+V", "0V", "X", "Y", "Z", "A"],
    "J7": ["+V", "0V", "IN1", "ESTOP", "PROBE", "0V"],
    "J8": ["5V", "0V", "A", "B", "Z", "0V"],
    "J9": ["NO1", "COM1", "NC1"], "J10": ["NO2", "COM2", "NC2"], "J11": ["NO3", "COM3", "NC3"],
    "J12": ["0-10V", "0V", "PWM", "CP", "5V", "0V"],
}
