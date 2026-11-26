"""Draws the schematic: one page for each function of the board, the parts of a function laid out
the way its signals flow and joined by wires.

What is connected to what comes from design.py; this file only says where each part is drawn.  A
part is asked for by what it does ("the resistor between STEP_X and GND"), never by its number, and
build.py then compares the nets of the drawing with design.py pin for pin, so a slip of the pen
here cannot change the circuit unnoticed.

Positions are in steps of 2.54 mm (P), x to the right and y downwards.  Names on wires that leave
a page are global labels; KiCad writes the pages they lead to beside them.
"""
import os

import design
import draw
import symlib
from sexp import Q, dump

SCH_VERSION = "20260306"
PROJECT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
KICAD_SYMBOLS = os.path.join(os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad"), "symbols")
U = 2.54
BY_REF = {p["ref"]: p for p in design.PARTS}
uid = draw.uid
ROOT = uid("root")



def from_controller(net):
    """What the ESP32 drives (the rest it reads): the arrow of a label shows which way a signal goes."""
    return net.startswith(("STEP_", "DIR_", "OUT")) or net in ("ENABLE", "SPINDLE", "CPUMP", "ETH_MOSI", "ETH_SCLK", "EN")


def P(x, y):
    return (round(x * U, 4), round(y * U, 4))


def _gate(off, unit):
    if all(abs(dx) < 0.01 for dx, dy in off.values()):         # the supply pins of a logic IC
        return (6.4, -1.2, 6.4, 1.3, "left")
    return (-3.8, -5.3, 1.7, 4.9, "left")


def _connector(off, unit):
    top = min(dy for dx, dy in off.values())
    return (0.0, top - 2.7, 0.0, top - 4.7, None)


CAPTIONS = {
    "Connector_Generic:Conn_01x02": _connector, "Connector_Generic:Conn_01x03": _connector,
    "Connector_Generic:Conn_01x04": _connector, "Connector_Generic:Conn_01x06": _connector,
    "74xx:74AHCT541": (-7.62, -19.3, 1.7, -19.3, "left"),
    "74xx:74AHCT125": _gate, "74xx:74HC14": _gate,
    "Amplifier_Operational:LM358": (1.0, -5.7, 1.0, 5.7, "left"),
    "Isolator:LTV-817S": (-5.08, -5.6, -5.08, 5.7, "left"),
    "Relay:G5LE-1": (8.5, -8.3, 13.5, -8.3, "left"),
    "Transistor_FET:AO3400A": (5.0, -1.3, 5.0, 1.3, "left"), "Transistor_FET:2N7002": (5.0, -1.3, 5.0, 1.3, "left"),
    "Diode:BAT54S": (0.0, 3.3, 0.0, 5.2, None),
    "Converter_DCDC:R-78E5.0-1.0": (-5.08, -3.7, -0.6, -3.7, "left"),
    "tiggy:ESP32-S3-DevKitC-1": (-10.16, -29.2, -4.5, -29.2, "left"),
    "tiggy:USR-ES1": (-7.62, -8.9, -2.5, -8.9, "left"),
}

_drawn = set()


def pick(prefix, a, b, group=None, value=None):
    """The two-pin part between nets a and b (the next one not yet drawn): (reference, pin on a, pin on b)."""
    for p in design.PARTS:
        ref = p["ref"]
        if ref in _drawn or ref.rstrip("0123456789") != prefix or len(p["pins"]) != 2:
            continue
        if (group and p["group"] != group) or (value and p["value"] != value):
            continue
        (n1, v1), (n2, v2) = p["pins"].items()
        if (v1, v2) == (a, b):
            _drawn.add(ref)
            return ref, n1, n2
        if (v2, v1) == (a, b):
            _drawn.add(ref)
            return ref, n2, n1
    raise KeyError("no %s left between %s and %s" % (prefix, a, b))


def with_pin(sym, pin, net):
    """The part with this symbol whose pin is on the net."""
    hits = [p["ref"] for p in design.PARTS if p["sym"] == sym and p["pins"].get(str(pin)) == net]
    assert len(hits) == 1, (sym, pin, net, hits)
    return hits[0]


class Sheet(draw.Page):
    def two(self, prefix, a, b, at, towards, group=None, value=None, text=None):
        """Draw the two-pin part between nets a and b with its a end at the point, the other end
        towards "R", "L", "U" or "D".  Returns where the b end is."""
        ref, pa, pb = pick(prefix, a, b, group, value)
        return self.put(ref, pa, at, towards, text=text)[pb]

    def gnd(self, at, towards="D"):
        self.power("GND", at, {"D": 0, "R": 90, "U": 180, "L": 270}[towards])

    def sup(self, kind, at, towards="U"):
        self.power(kind, at, {"U": 0, "L": 90, "D": 180, "R": 270}[towards])

    def to_gnd(self, prefix, net, at, **kw):
        """A part from the point down to 0 V."""
        self.gnd(self.two(prefix, net, "GND", at, "D", **kw))

    def from_sup(self, prefix, kind, net, at, **kw):
        """A part from the point up to a supply."""
        self.sup(kind, self.two(prefix, net, kind, at, "U", **kw))

    def signal(self, net, at, side):
        """The name of a signal that comes from, or goes to, another page."""
        out = from_controller(net)
        if self.key != "controller":
            out = not out
        self.glabel(net, at, 180 if side == "L" else 0, "output" if out else "input")

    def terminals(self, ref, pins, side):
        """What is printed on the board beside each terminal, beside the connector's symbol."""
        for i, name in enumerate(design.TERMINALS[ref]):
            x, y = pins[str(i + 1)]
            if side == "L":
                self.text(name, (x - 7.3, y), justify="right")
            else:
                self.text(name, (x + 7.3, y), justify="left")


# ---------------------------------------------------------------------------
def page_controller(pg):
    a = pg.place("A1", *P(56, 40))
    nets = BY_REF["A1"]["pins"]
    for i in range(44):
        num = str(i + 1)
        left, p = i < 22, a[str(i + 1)]
        net = nets.get(num)
        if net is None:
            pg.nc(p)
        elif net not in ("GND", "+3V3"):
            end = (p[0] - 4 * U, p[1]) if left else (p[0] + 4 * U, p[1])
            pg.wire(p, end)
            if net == "5V_ESP":
                pg.glabel(net, end, 180, "input")
            else:
                pg.signal(net, end, "L" if left else "R")
    pg.wire(a["1"], P(47, 30), P(47, 28))                   # 3.3 V: out of the DevKitC's own regulator
    pg.wire(a["2"], P(49, 31), P(49, 30))
    pg.sup("+3V3", P(47, 28))
    pg.wire(a["22"], P(49, 51), P(49, 53))
    pg.gnd(P(49, 53))
    pg.wire(a["23"], P(63, 30))
    pg.gnd(P(63, 30), "R")
    pg.wire(a["43"], P(63, 50), P(63, 53))
    pg.wire(a["44"], P(63, 51))
    pg.gnd(P(63, 53))
    pg.text("The pins are in the order of the DevKitC's two headers, the aerial at the top.", P(40, 57.5))
    pg.text("Pins without a name here are not used (the cross): the USB serial port, the memory, the LED, USB.", P(40, 59))

    for i in range(4):                                      # the fixing holes
        pg.place("H%d" % (i + 1), *P(10 + 4 * i, 60))
    pg.text("Fixing holes", P(8, 57))


def page_supply(pg):
    y = 30
    j = pg.place("J1", *P(12, y), mirror="y")
    pg.terminals("J1", j, "L")
    f = pg.two("F", "VIN_IN", "VIN_F", P(18, y), "R")
    pg.wire(j["1"], P(18, y))
    pg.label("VIN_IN", P(14.5, y))
    k = pg.two("D", "VIN_F", "VIN", P(25, y), "R")
    pg.wire(f, P(25, y))
    pg.label("VIN_F", P(21.5, y))
    pg.wire(j["2"], P(15, y + 1), P(15, y + 3))
    pg.gnd(P(15, y + 3))

    u = pg.place("U1", *P(59, y))                           # 12-24 V rail, then the 5 V converter
    pg.wire(k, u["1"])
    pg.sup("VIN", P(31, y))
    pg.to_gnd("D", "VIN", P(34, y))
    pg.to_gnd("C", "VIN", P(41, y), value="100u 35V")
    pg.to_gnd("C", "VIN", P(48, y), value="10u 50V")
    pg.sup("PWR_FLAG", P(53, y))
    pg.gnd(u["2"])

    d = pg.two("D", "+5V", "5V_ESP", P(82, y), "R")         # 5 V rail
    pg.wire(u["3"], P(82, y))
    pg.to_gnd("C", "+5V", P(66, y), group="Supply")
    pg.sup("+5V", P(71, y))
    lamp = pg.two("R", "+5V", "PWR_LED", P(75, y), "D")
    pg.to_gnd("D", "PWR_LED", lamp)
    pg.wire(d, P(87, y))
    pg.glabel("5V_ESP", P(87, y), 0, "output")
    pg.text("lit: 5 V is there", P(70, y + 9))
    pg.text("so that the DevKitC's USB socket", P(79, y + 2.5))
    pg.text("cannot feed the relays", P(79, y + 4))

    y = 48                                                  # 3.3 V, and telling KiCad what is a supply
    pg.sup("+3V3", P(22, y))
    pg.wire(P(22, y), P(22, y + 1), P(27, y + 1))
    pg.to_gnd("C", "+3V3", P(22, y + 1), group="Supply")
    pg.sup("PWR_FLAG", P(27, y + 1))
    pg.text("3.3 V comes from the DevKitC's own regulator (Controller page).", P(30, y + 3))
    pg.sup("PWR_FLAG", P(22, y + 11))
    pg.wire(P(22, y + 11), P(22, y + 13))
    pg.gnd(P(22, y + 13))
    pg.text("0 V: one plane, both sides of the board.", P(30, y + 12.5))


def page_drives(pg):
    # enable: one buffer for all four drives
    e = pg.place("U3", *P(52, 16), unit=1)
    pg.wire(P(16, 16), e["2"])
    pg.signal("ENABLE", P(16, 16), "L")
    pg.to_gnd("R", "ENABLE", P(41, 16))
    pg.gnd(e["1"])
    pg.label("EN_O", P(56, 16))

    # step and direction
    u = pg.place("U2", *P(52, 38))
    nets = BY_REF["U2"]["pins"]
    for i in range(8):
        y, x = 33 + i, 18 + 3.5 * i
        net = nets[str(2 + i)]
        pg.wire(P(16, y), u[str(2 + i)])
        pg.signal(net, P(16, y), "L")
        pg.wire(P(x, y), P(x, 44))
        pg.two("R", net, "GND", P(x, 44), "D")              # held low while the ESP32 is in reset
    pg.wire(P(18, 47), P(52, 47))
    pg.wire(u["1"], P(46, 42), P(46, 47))                   # both output enables: always on
    pg.wire(u["19"], P(46, 43))
    pg.wire(u["10"], P(52, 47))
    pg.gnd(P(52, 47))
    pg.sup("+5V", P(52, 25))
    pg.wire(P(52, 25), u["20"])
    pg.wire(P(52, 26), P(58, 26))
    pg.to_gnd("C", "+5V", P(58, 26), group="Drives")

    conn = {"X": ("J2", 24), "Y": ("J3", 32), "Z": ("J4", 40), "A": ("J5", 48)}
    pins = {}
    for axis, (ref, yc) in conn.items():
        pins[axis] = pg.place(ref, *P(78, yc), mirror="x")  # (pin 1 at the bottom, so that no two wires cross)
        pg.terminals(ref, pins[axis], "R")
        pg.gnd(pins[axis]["4"], "L")
    turn_at = {"X_DIR": 59, "X_STEP": 60, "Y_DIR": 61, "Y_STEP": 62, "Z_DIR": 62, "Z_STEP": 61, "A_DIR": 60, "A_STEP": 59}
    for i in range(8):
        net = nets[str(18 - i)]
        to = pins[net[0]]["1" if net.endswith("STEP") else "2"]
        x = turn_at[net]
        pg.wire(u[str(18 - i)], P(x, 33 + i), (x * U, to[1]), to)
        pg.label(net, (69.5 * U, to[1]))
    pg.wire(e["3"], P(68, 16), P(68, 47), pins["A"]["3"])   # enable, down past all four
    for axis in "XYZ":
        pg.wire((68 * U, pins[axis]["3"][1]), pins[axis]["3"])

    # the rest of U3
    s = pg.place("U3", *P(24, 60), unit=5)
    pg.sup("+5V", P(24, 53))
    pg.wire(P(24, 53), s["14"])
    pg.wire(P(24, 54), P(33, 54), P(33, 58.5))
    c = pg.two("C", "+5V", "GND", P(33, 58.5), "D", group="Drives")
    pg.wire(c, P(33, 66), P(24, 66))
    pg.wire(s["7"], P(24, 66))
    pg.gnd(P(24, 66))
    d = pg.place("U3", *P(48, 60), unit=4)
    pg.wire(d["12"], P(44, 60), P(44, 61))
    pg.gnd(P(44, 61))
    pg.nc(d["11"])
    pg.wire(d["13"], P(48, 64), P(54, 64), P(54, 57))
    pg.sup("+5V", P(54, 57))
    pg.text("spare gate, switched off", P(44, 66.5))
    pg.text("U3's other two gates: Spindle speed page", P(20, 69))


def _input(pg, x0, y0, n, name, net, terminal, via=None):
    """One opto-isolated input, drawn left to right from x0, its lamp and resistor on the line y0."""
    pg.sup("VIN", P(x0 - 1, y0))
    r = pg.two("R", "VIN", "IN_%s_A" % name, P(x0, y0), "R")
    pg.wire(P(x0 - 1, y0), P(x0, y0))
    d = pg.two("D", "IN_%s_A" % name, "IN_%s_B" % name, P(x0 + 4, y0), "R")
    pg.wire(r, P(x0 + 4, y0))
    o = pg.place("OK%d" % n, *P(x0 + 12, y0 + 1))
    pg.wire(d, o["1"])
    pg.wire(terminal, *((via or []) + [o["2"]]))
    pg.wire(o["4"], P(x0 + 26, y0))
    pg.signal(net, P(x0 + 26, y0), "R")
    pg.from_sup("R", "+3V3", net, P(x0 + 18, y0))
    pg.to_gnd("C", net, P(x0 + 22, y0))
    pg.wire(o["3"], P(x0 + 15, y0 + 3))
    pg.gnd(P(x0 + 15, y0 + 3))


def page_inputs(pg):
    ch = {name: (i + 1, name, net) for i, (name, net) in enumerate(design.CHANNELS)}
    j = pg.place("J6", *P(10, 39), mirror="y")              # limits: terminals at x = 12, y = 37 to 42
    pg.terminals("J6", j, "L")
    pg.wire(j["1"], P(13, 37), P(13, 35))
    pg.sup("VIN", P(13, 35))
    pg.gnd(j["2"], "R")
    _input(pg, 22, 17, *ch["X"], j["3"], [P(15, 39), P(15, 19)])
    _input(pg, 22, 28, *ch["Y"], j["4"], [P(17, 40), P(17, 30)])
    _input(pg, 22, 39, *ch["Z"], j["5"])
    _input(pg, 22, 50, *ch["A"], j["6"], [P(15, 42), P(15, 52)])

    j = pg.place("J7", *P(60, 30), mirror="y")              # the other inputs: x = 62, y = 28 to 33
    pg.terminals("J7", j, "L")
    pg.wire(j["1"], P(63, 28), P(63, 26))
    pg.sup("VIN", P(63, 26))
    pg.gnd(j["2"], "R")
    pg.wire(j["6"], P(63, 33), P(63, 35))
    pg.gnd(P(63, 35))
    _input(pg, 72, 17, *ch["AUX1"], j["3"], [P(65, 30), P(65, 19)])
    _input(pg, 72, 29, *ch["ESTOP"], j["4"])
    _input(pg, 72, 41, *ch["PROBE"], j["5"], [P(65, 32), P(65, 43)])
    pg.text("Each input: a switch or an NPN sensor between its terminal and 0V lights the lamp", P(58, 58))
    pg.text("and pulls the ESP32's pin low.  The +V terminals are for sensors that need a supply.", P(58, 59.5))


def page_sensor(pg):
    j = pg.place("J8", *P(10, 32), mirror="y")              # terminals at x = 12, y = 30 to 35
    pg.terminals("J8", j, "L")
    pg.wire(j["1"], P(13, 30), P(13, 28))
    pg.sup("+5V", P(13, 28))
    pg.gnd(j["2"], "R")
    pg.wire(j["6"], P(13, 35), P(13, 37))
    pg.gnd(P(13, 37))
    rows = (("A", 21, "3", [P(15, 32), P(15, 21)], (1, 2)), ("B", 33, "4", [], (3, 4)),
            ("Z", 45, "5", [P(15, 34), P(15, 45)], (6, 5)))
    for ch, y, pin, via, units in rows:
        t, f = "ENC_%s_T" % ch, "ENC_%s_F" % ch
        pg.wire(j[pin], *(via + [P(26, y)]))
        pg.from_sup("R", "+5V", t, P(22, y), text="left")   # pull-up: open-collector sensors
        r = pg.two("R", t, f, P(26, y), "R")
        d = pg.place(with_pin("Diode:BAT54S", 3, f), *P(33, y + 3), rot=180)
        pg.wire(P(33, y), d["3"])                           # clamp to 0 V and 5 V
        pg.sup("+5V", pg.wire(d["2"], P(29, y + 3)))
        pg.gnd(pg.wire(d["1"], P(37, y + 3)))
        g1 = pg.place("U4", *P(41, y), unit=units[0])
        g2 = pg.place("U4", *P(50, y), unit=units[1])
        ins = sorted(g1, key=lambda n: g1[n][0])            # (input, output) of the gate
        ins2 = sorted(g2, key=lambda n: g2[n][0])
        pg.wire(r, g1[ins[0]])
        pg.wire(g1[ins[1]], g2[ins2[0]])
        pg.wire(g2[ins2[1]], P(57, y))
        pg.signal("ENC_%s" % ch, P(57, y), "R")
    w = pg.place("U4", *P(84, 31), unit=7)
    pg.sup("+3V3", P(84, 24))
    pg.wire(P(84, 24), w["14"])
    pg.wire(P(84, 25), P(93, 25), P(93, 29.5))
    c = pg.two("C", "+3V3", "GND", P(93, 29.5), "D", group="Spindle sensor")
    pg.wire(c, P(93, 37), P(84, 37))
    pg.wire(w["7"], P(84, 37))
    pg.gnd(P(84, 37))
    pg.text("A, B: an encoder's two tracks.  Z: its index, or one pulse per turn from a single sensor.", P(12, 54))
    pg.text("Sensors with open-collector (NPN) or 5 V push-pull outputs; up to 24 V on a terminal does no harm.", P(12, 55.5))
    pg.text("With no sensor fitted the three are spare inputs 2 to 4.", P(12, 57))


def page_relays(pg):
    for n in (1, 2, 3):
        t = 15 + 17 * (n - 1)
        coil = "RLY%d_K" % n
        k = pg.place("K%d" % n, *P(48, t + 3), mirror="x")  # coil at x = 46, contacts 49 to 51
        lamp = pg.two("R", "+5V", "RLY%d_L" % n, P(30, t), "D")
        pg.two("D", "RLY%d_L" % n, coil, lamp, "D")
        d = pg.two("D", coil, "+5V", P(39, t + 4.5), "U", text="left")   # takes the coil's current at switch-off
        pg.wire(d, P(39, t))
        pg.wire(P(39, t + 4.5), P(39, t + 6))
        pg.wire(P(30, t), k["2"])
        pg.sup("+5V", P(39, t))
        pg.wire(P(30, t + 6), k["5"])
        pg.label(coil, P(40, t + 6))
        q = pg.place("Q%d" % (n + 1), *P(45, t + 9))
        pg.wire(k["5"], q["3"])
        pg.gnd(q["2"])
        pg.wire(P(20, t + 9), q["1"])
        pg.signal("OUT%d" % n, P(20, t + 9), "L")
        pg.to_gnd("R", "OUT%d" % n, P(35, t + 9))           # off while the ESP32 is in reset
        j = pg.place("J%d" % (8 + n), *P(68, t + 3))
        pg.terminals("J%d" % (8 + n), j, "R")
        pg.wire(k["1"], P(50, t - 1), P(58, t - 1), P(58, t + 3), j["2"])
        pg.wire(k["3"], P(51, t + 7), P(54, t + 7), P(54, t + 2), j["1"])
        pg.wire(k["4"], P(49, t + 8), P(60, t + 8), P(60, t + 4), j["3"])
        pg.label("RLY%d_NO" % n, P(61, t + 2))
        pg.label("RLY%d_COM" % n, P(61, t + 3))
        pg.label("RLY%d_NC" % n, P(61, t + 4))
    pg.sup("+5V", P(86, 20))
    pg.wire(P(86, 20), P(86, 21), P(92, 21))
    pg.to_gnd("C", "+5V", P(86, 21), group="Outputs", value="10u")
    pg.to_gnd("C", "+5V", P(92, 21), group="Outputs", value="100n")
    pg.text("beside the relays", P(86, 27.5))
    pg.text("Contacts: 30 V at most on this", P(84, 34))
    pg.text("revision of the board.", P(84, 35.5))


def page_spindle(pg):
    y = 26                                                  # speed: PWM, buffered to 5 V, then smoothed and doubled
    b = pg.place("U3", *P(25, y), unit=2)
    pg.wire(P(14, y), b["5"])
    pg.signal("SPINDLE", P(14, y), "L")
    pg.to_gnd("R", "SPINDLE", P(18, y))
    pg.gnd(b["4"])
    f = pg.two("R", "PWM_5V", "PWM_F", P(33, y), "R")
    pg.wire(b["6"], P(33, y))
    pg.label("PWM_5V", P(28.5, y))
    a = pg.place("U5", *P(47, y + 1), unit=1)
    pg.wire(f, a["3"])
    pg.to_gnd("C", "PWM_F", P(39, y))
    p = pg.place("U5", *P(47, y + 1), unit=3, text="hide")
    pg.sup("VIN", P(46, y - 7))
    pg.wire(P(46, y - 7), p["8"])
    pg.wire(P(46, y - 6), P(52, y - 6))
    pg.to_gnd("C", "VIN", P(52, y - 6), value="100n 50V")
    pg.wire(p["4"], P(46, y + 5))
    pg.gnd(P(46, y + 5))
    j = pg.place("J12", *P(68, y + 3))                      # terminals at x = 66, y + 1 to y + 6
    pg.terminals("J12", j, "R")
    r = pg.two("R", "AO_OUT", "AO_T", P(54, y + 1), "R")
    pg.wire(a["1"], P(54, y + 1))
    pg.wire(r, j["1"])
    pg.label("AO_T", P(58, y + 1))
    fb = pg.two("R", "AO_OUT", "AO_FB", P(49, y + 8), "L", text="below")
    pg.wire(P(52, y + 1), P(52, y + 8), P(49, y + 8))
    pg.wire(fb, P(43, y + 8), P(43, y + 2), a["2"])
    pg.to_gnd("R", "AO_FB", P(43, y + 8))
    pg.wire(P(31, y), P(31, y + 14), P(60, y + 14), P(60, y + 3), j["3"])
    pg.gnd(j["2"], "L")
    pg.sup("+5V", j["5"], "L")
    pg.gnd(j["6"], "L")
    pg.text("10 k, 4u7: ripple under 1 %", P(31.5, y - 4.5))
    pg.text("at 1 kHz", P(31.5, y - 3.2))
    pg.text("gain 2: 0 to 10 V", P(47.5, y + 12))

    y2 = 48                                                 # charge pump: a square wave while the controller is well
    c = pg.place("U3", *P(25, y2), unit=3)
    pg.wire(P(14, y2), c["9"])
    pg.signal("CPUMP", P(14, y2), "L")
    pg.to_gnd("R", "CPUMP", P(18, y2))
    pg.gnd(c["10"])
    pg.wire(c["8"], P(62, y2), P(62, y + 4), j["4"])
    pg.label("CP_O", P(28.5, y2))

    s = pg.place("U5", *P(47, 60), unit=2)                  # the amplifier's other half, parked
    pg.wire(s["5"], P(42, 59))
    pg.gnd(P(42, 59))
    pg.wire(s["6"], P(43, 61), P(43, 64), P(52, 64), P(52, 60), s["7"])
    pg.text("spare half, parked", P(54, 60.5))
    pg.text("The 0-10V output needs 12 V or more at the supply terminal to reach 10 V.", P(14, 68))
    pg.text("U3's supply pins and its enable gate: Drives page.", P(14, 69.5))


def page_ethernet(pg):
    m = pg.place("A2", *P(64, 32))                          # left pins at x = 60, right at x = 68, y = 30 to 35
    pg.wire(m["1"], P(59, 30))
    pg.wire(m["2"], P(59, 31), P(59, 30))
    pg.gnd(P(59, 30), "L")
    pg.wire(m["3"], P(57, 32), P(57, 20), P(42, 20))
    pg.signal("ETH_MOSI", P(42, 20), "L")
    pg.wire(m["4"], P(56, 33), P(56, 27), P(42, 27))
    pg.signal("ETH_SCLK", P(42, 27), "L")
    pg.from_sup("R", "+3V3", "ETH_SCLK", P(50, 27))         # GPIO0 must be high when the ESP32 starts
    pg.wire(m["6"], P(57, 35), P(57, 41), P(44, 41))
    pg.signal("ETH_INT", P(44, 41), "L")
    pg.from_sup("R", "+3V3", "ETH_INT", P(47, 41))

    q = pg.place("Q1", *P(35, 38))                          # chip select: low a moment after reset ends
    pg.wire(m["5"], P(36, 34), q["3"])
    pg.gnd(q["2"])
    pg.label("ETH_CS", P(37, 34))
    pg.from_sup("R", "+3V3", "ETH_CS", P(46, 34))
    pg.wire(P(53, 34), P(53, 35))
    jp = pg.put("JP1", 1, P(53, 35), "D", text="left")
    pg.gnd(jp["2"])
    pg.signal("EN", P(12, 38), "L")
    r = pg.two("R", "EN", "CS_GATE", P(20, 38), "R", text="below")
    pg.wire(P(12, 38), P(20, 38))
    pg.wire(r, q["1"])
    d = pg.two("D", "EN", "CS_GATE", P(20, 35), "R")
    pg.wire(P(18, 38), P(18, 35), P(20, 35))
    pg.wire(d, P(26, 35), P(26, 38))
    pg.to_gnd("C", "CS_GATE", P(29, 38))
    pg.label("CS_GATE", P(29.5, 38))

    pg.wire(P(16, 38), P(16, 14), P(82, 14), P(82, 22))     # reset, to the module as well
    r = pg.two("R", "EN", "ETH_RST", P(82, 22), "D")
    pg.wire(r, P(82, 34), m["11"])
    pg.gnd(m["7"], "R")
    pg.wire(m["8"], P(72, 31), P(72, 26))
    pg.wire(m["9"], P(70, 32), P(70, 31))
    pg.sup("+3V3", P(72, 26))
    pg.wire(P(72, 28), P(76, 28))
    pg.to_gnd("C", "+3V3", P(76, 28), group="Ethernet")
    pg.nc(m["10"])
    pg.wire(m["12"], P(70, 35), P(70, 42), P(78, 42))
    pg.signal("ETH_MISO", P(78, 42), "R")
    pg.to_gnd("R", "ETH_MISO", P(73, 42))                   # GPIO46 must be low when the ESP32 starts

    pg.text("The firmware has no pin for chip select: it expects the module always selected.  Here Q1 selects it", P(12, 52))
    pg.text("20 to 80 ms after the ESP32 leaves reset (47 k and 1 u), so that the module's data output cannot hold", P(12, 53.5))
    pg.text("GPIO46 high while the ESP32 reads it as a start-up option.  D lets the capacitor empty at once on a reset.", P(12, 55))
    pg.text("JP1, bridged with solder, selects the module all the time (for a module that then stays off the bus).", P(12, 56.5))


# (key, title, file, note, drawn by)
PAGES = [
    ("controller", "Controller", None,
     "ESP32-S3-DevKitC-1 (the version with octal memory) on two socket strips.\n"
     "Firmware environment esp32s3-devkitc, terminal layout \"four axes\".", page_controller),
    ("supply", "Supply", "supply.kicad_sch",
     "12 to 24 V in; 5 V for the board from a switching converter.", page_supply),
    ("drives", "Drives", "drives.kicad_sch",
     "Step, direction and enable for four drives, as 5 V push-pull outputs.", page_drives),
    ("inputs", "Inputs", "inputs.kicad_sch",
     "Seven opto-isolated inputs: four limit switches, emergency stop, probe, one spare.", page_inputs),
    ("sensor", "Spindle sensor", "spindle-sensor.kicad_sch",
     "Three fast inputs for an encoder on the spindle (screw cutting), squared up by Schmitt triggers on 3.3 V.", page_sensor),
    ("relays", "Relays", "relays.kicad_sch",
     "Three relays on the auxiliary outputs 1 to 3, each with a lamp.", page_relays),
    ("spindle", "Spindle speed", "spindle-speed.kicad_sch",
     "The spindle's speed as PWM and as 0 to 10 V for a frequency converter, and the charge pump signal.", page_spindle),
    ("ethernet", "Ethernet", "ethernet.kicad_sch",
     "Socket for a USR-ES1 (W5500) module.", page_ethernet),
]


def sheet_uuid(key):
    return uid("sheet", key)


def build():
    """Returns {file name: text} for the root page and the others."""
    _drawn.clear()
    libs = [KICAD_SYMBOLS, PROJECT]
    pages = []
    for number, (key, title, file, note, fn) in enumerate(PAGES, 1):
        path = "/" + ROOT if number == 1 else "/%s/%s" % (ROOT, sheet_uuid(key))
        pg = Sheet(key, title, design.NAME, path, libs, BY_REF, number=number, note=note, captions=CAPTIONS)
        fn(pg)
        pages.append(pg)

    # every part, and every unit of it, drawn once
    drawn = [x for pg in pages for x in pg.placed]
    assert len(drawn) == len(set(drawn)), sorted(x for x in drawn if drawn.count(x) > 1)
    want = {(p["ref"], u) for p in design.PARTS for u in symlib.units(symlib.load(libs, p["sym"]))}
    assert set(drawn) == want, "not drawn: %s; not in the design: %s" % (sorted(want - set(drawn)), sorted(set(drawn) - want))

    # the other pages, as a list down the right of the first
    sheets = []
    root = pages[0]
    root.text("The other pages", P(86, 14.5), size=1.8, bold=True)
    for number, (key, title, file, note, fn) in enumerate(PAGES[1:], 2):
        x, y = P(86, 17 + 5.5 * (number - 2))
        sheets.append(["sheet", ["at", x, y], ["size", 14 * U, 2.5 * U], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                       ["on_board", "yes"], ["dnp", "no"], ["fields_autoplaced", "yes"],
                       ["stroke", ["width", 0.1524], ["type", "solid"]], ["fill", ["color", 0, 0, 0, "0.0000"]],
                       ["uuid", sheet_uuid(key)],
                       ["property", Q("Sheetname"), Q(title), ["at", x, round(y - 0.7, 4), 0],
                        draw.font(1.27, "left bottom")],
                       ["property", Q("Sheetfile"), Q(file), ["at", x, round(y + 2.5 * U + 0.6, 4), 0], ["hide", "yes"],
                        draw.font(1.27, "left top")],
                       ["instances", ["project", Q(design.NAME), ["path", Q("/" + ROOT), ["page", Q(str(number))]]]]])
        root.text("page %d" % number, (x + 1.5, y + 1.25 * U), justify="left")

    out = {}
    for number, (pg, (key, title, file, note, fn)) in enumerate(zip(pages, PAGES), 1):
        block = ["title_block", ["title", Q("Tiggy DevKitC carrier - %s" % title)], ["rev", Q(design.REV)],
                 ["comment", 1, Q("Four axes, Ethernet, spindle sensor, opto-isolated inputs, three relays")],
                 ["comment", 2, Q("Drawn by gen/gen_sch.py from gen/design.py - change it there")]]
        tree = pg.tree(SCH_VERSION, ROOT if number == 1 else uid("pagefile", key), block, sheets if number == 1 else ())
        out[file or design.NAME + ".kicad_sch"] = dump(tree) + "\n"
    return out


if __name__ == "__main__":
    files = build()
    for name, text in files.items():
        with open(os.path.join(PROJECT, name), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print("schematic drawn: %d parts on %d pages" % (len(design.PARTS), len(files)))
