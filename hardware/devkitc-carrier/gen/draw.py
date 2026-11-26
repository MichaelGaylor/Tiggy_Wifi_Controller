"""A page of a KiCad schematic, drawn from Python: symbols placed by one of their pins, wires,
junction dots where wires meet, supply symbols, labels.

Coordinates are millimetres, y downwards as on the screen, everything on KiCad's 1.27 mm grid.
"""
import uuid

import symlib
from sexp import Q

GRID = 1.27
NS = uuid.UUID("6f3a9d2e-5b1c-4a77-9c0e-7d1e5a0c1001")
SUPPLY = {"GND": "power:GND", "+5V": "power:+5V", "+3V3": "power:+3V3", "VIN": "tiggy:VIN", "PWR_FLAG": "power:PWR_FLAG"}


def uid(*key):
    return Q(str(uuid.uuid5(NS, "/".join(str(k) for k in key))))


def r3(v):
    return round(v + 0.0, 4)


def on_grid(p):
    return all(abs(v / GRID - round(v / GRID)) < 1e-6 for v in p)


def turn(x, y, rot, mirror=None):
    """A point of a library symbol (y upwards) as an offset on the page, for KiCad's rotation
    (counter-clockwise) and mirror (applied before the rotation, as KiCad does)."""
    y = -y
    if mirror == "y":
        x = -x
    elif mirror == "x":
        y = -y
    for _ in range((rot // 90) % 4):
        x, y = y, -x
    return x, y


def level(rot, mirror, justify):
    """How a caption has to be stored to be read level, from the side asked for, on a symbol that
    is turned or mirrored: KiCad turns a symbol's captions with it.  Returns (angle, justify)."""
    angle = 90 if (rot // 90) % 2 else 0
    along = (1.0, 0.0) if angle == 0 else (0.0, 1.0)        # the way the text runs, before the symbol is turned
    if justify in ("left", "right") and turn(along[0], along[1], rot, mirror)[0] < 0:
        justify = "right" if justify == "left" else "left"
    return angle, justify


def font(size=1.27, justify=None, bold=False):
    f = ["font", ["size", size, size]]
    if bold:
        f.append(["bold", "yes"])
    e = ["effects", f]
    if justify:
        e.append(["justify"] + justify.split())
    return e


class Page:
    def __init__(self, key, title, project, path, lib_dirs, parts, number=1, note="", paper="A4", captions=None):
        self.key, self.title, self.project, self.path, self.paper = key, title, project, path, paper
        self.number, self.note = number, note
        self.lib_dirs, self.parts = lib_dirs, parts         # parts: reference -> part of design.PARTS
        self.captions = captions or {}                      # symbol -> where its reference and value go
        self.items, self.syms = [], {}
        self.segments = []                                  # wires, as ((x1, y1), (x2, y2))
        self.nodes = set()                                  # pin ends, label and supply symbol points
        self.pins_at = {}                                   # point -> number of pins there
        self.placed = []                                    # (reference, unit)
        self.labels = []                                    # (net, point) of the global labels
        self.npower = 0

    # ---- symbols ---------------------------------------------------------
    def lib(self, lib_id):
        if lib_id not in self.syms:
            self.syms[lib_id] = symlib.load(self.lib_dirs, lib_id)
        return self.syms[lib_id]

    def offsets(self, lib_id, unit, rot, mirror):
        return {p[1]: turn(p[3], p[4], rot, mirror) for p in symlib.pins(self.lib(lib_id)) if p[0] in (unit, 0)}

    def place(self, ref, x, y, rot=0, mirror=None, unit=1, text=None):
        """Put a part (one unit of it) with its origin at x, y.  Returns {pin number: point}.

        text: where its reference and its value are written - (reference dx, dy, value dx, dy,
        justify) from the origin, or "hide", "left" (of an upright part), "below" (a level one);
        left out, the symbol's usual place."""
        part = self.parts[ref]
        assert (ref, unit) not in self.placed, "drawn twice: %s unit %d" % (ref, unit)
        self.placed.append((ref, unit))
        x, y = r3(x), r3(y)
        assert on_grid((x, y)), "off the grid: %s at %s" % (ref, (x, y))
        off = self.offsets(part["sym"], unit, rot, mirror)
        pts = {n: (r3(x + dx), r3(y + dy)) for n, (dx, dy) in off.items()}
        hide = text == "hide"
        if text is None and part["sym"] in self.captions:
            text = self.captions[part["sym"]]
            if callable(text):
                text = text(off, unit)
        if not isinstance(text, tuple):
            flat = {(abs(dx) > abs(dy)) for dx, dy in off.values()} if off else {True}
            if len(off) == 2 and flat == {False}:           # pins above and below
                text = (-3.0, -1.2, -3.0, 1.3, "right") if text == "left" else (3.0, -1.2, 3.0, 1.3, "left")
            elif len(off) <= 2:                             # pins left and right, or none
                text = (0.0, 3.0, 0.0, 4.9, None) if text == "below" else (0.0, -4.8, 0.0, -2.9, None)
            else:
                xs = [dx for dx, dy in off.values()]
                ys = [dy for dx, dy in off.values()]
                text = (min(xs) + 2.54, min(ys) - 3.6, min(xs) + 2.54, max(ys) + 3.8, "left")
        rdx, rdy, vdx, vdy, just = text
        angle, just = level(rot, mirror, just)
        s = ["symbol", ["lib_id", Q(part["sym"])], ["at", x, y, rot]]
        if mirror:
            s.append(["mirror", mirror])
        s += [["unit", unit], ["body_style", 1], ["exclude_from_sim", "no"],
              ["in_bom", "no" if ref.startswith(("H", "JP")) else "yes"],     # as their footprints say
              ["on_board", "yes"], ["in_pos_files", "yes"], ["dnp", "no"], ["uuid", uid("sym", ref, unit)],
              self._prop("Reference", ref, x + rdx, y + rdy, just=just, hide=hide, angle=angle),
              self._prop("Value", part["value"], x + vdx, y + vdy, just=just, hide=hide, angle=angle),
              self._prop("Footprint", part["fp"], x, y, hide=True),
              self._prop("Datasheet", "", x, y, hide=True),
              self._prop("Description", "", x, y, hide=True)]
        for name, value in part.get("fields", {}).items():
            s.append(self._prop(name, value, x, y, hide=True))
        for p in symlib.pins(self.lib(part["sym"])):
            if p[0] in (unit, 0):
                s.append(["pin", Q(p[1]), ["uuid", uid("pin", ref, unit, p[1])]])
        s.append(["instances", ["project", Q(self.project), ["path", Q(self.path), ["reference", Q(ref)], ["unit", unit]]]])
        self.items.append(s)
        for pt in pts.values():
            self.nodes.add(pt)
            self.pins_at[pt] = self.pins_at.get(pt, 0) + 1
        return pts

    def put(self, ref, pin, at, towards=None, rot=None, mirror=None, unit=1, text=None):
        """Put a part so that one of its pins is at a point.  For a two-pin part, `towards`
        ("R", "L", "U", "D") says where its other pin is to lie."""
        part = self.parts[ref]
        pin = str(pin)
        if rot is None:
            want = {"R": (1, 0), "L": (-1, 0), "U": (0, -1), "D": (0, 1)}[towards]
            for rot in (0, 90, 180, 270):
                off = self.offsets(part["sym"], unit, rot, mirror)
                assert len(off) == 2, "towards= is for two-pin parts"
                other = [n for n in off if n != pin][0]
                dx, dy = off[other][0] - off[pin][0], off[other][1] - off[pin][1]
                if dx * want[0] + dy * want[1] > 0 and abs(dx * want[1] - dy * want[0]) < 1e-6:
                    break
            else:
                raise AssertionError((ref, towards))
        off = self.offsets(part["sym"], unit, rot, mirror)
        return self.place(ref, at[0] - off[pin][0], at[1] - off[pin][1], rot, mirror, unit, text)

    def _prop(self, name, value, x, y, hide=False, just=None, size=1.27, angle=0):
        p = ["property", Q(name), Q(value), ["at", r3(x), r3(y), angle]]
        if hide:
            p.append(["hide", "yes"])
        p.append(font(size, just))
        return p

    def power(self, kind, at, rot=0):
        """A supply symbol with its pin at the point: "GND", "+5V", "+3V3", "VIN", "PWR_FLAG".
        rot turns it counter-clockwise from its usual way up."""
        lib_id = SUPPLY[kind]
        self.lib(lib_id)
        self.npower += 1
        ref = "%s%d%02d" % ("#FLG" if kind == "PWR_FLAG" else "#PWR", self.number, self.npower)
        x, y = r3(at[0]), r3(at[1])
        assert on_grid((x, y)), "off the grid: %s at %s" % (kind, (x, y))
        dx, dy = turn(0, -3.9 if kind == "GND" else 3.7, rot)          # its name, beyond the symbol
        angle, just = level(rot, None, {0: None, 90: "right", 180: None, 270: "left"}[rot])
        self.items.append(["symbol", ["lib_id", Q(lib_id)], ["at", x, y, rot], ["unit", 1], ["body_style", 1],
                           ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"], ["in_pos_files", "yes"],
                           ["dnp", "no"], ["uuid", uid("pwr", self.key, self.npower)],
                           self._prop("Reference", ref, x, y, hide=True),
                           self._prop("Value", kind, x + dx, y + dy, hide=(kind == "GND"), just=just, size=1.1, angle=angle),
                           self._prop("Footprint", "", x, y, hide=True), self._prop("Datasheet", "", x, y, hide=True),
                           self._prop("Description", "", x, y, hide=True),
                           ["pin", Q("1"), ["uuid", uid("pwrpin", self.key, self.npower)]],
                           ["instances", ["project", Q(self.project), ["path", Q(self.path), ["reference", Q(ref)],
                                                                   ["unit", 1]]]]])
        self.nodes.add((x, y))
        self.pins_at[(x, y)] = self.pins_at.get((x, y), 0) + 1

    # ---- wires and names -------------------------------------------------
    def wire(self, *pts):
        """A wire through the points, each leg level or upright.  Returns the last point."""
        pts = [(r3(p[0]), r3(p[1])) for p in pts]
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            assert a[0] == b[0] or a[1] == b[1], "a wire neither level nor upright: %s %s on %s" % (a, b, self.key)
            assert on_grid(a) and on_grid(b), "a wire off the grid: %s %s on %s" % (a, b, self.key)
            self.segments.append((a, b))
        return pts[-1]

    def glabel(self, net, at, rot=0, shape="passive"):
        """A name that joins this point to every other of the same name, on any page.
        rot 0: the name to the right of the point; 180: to the left."""
        x, y = r3(at[0]), r3(at[1])
        just = "right" if rot in (180, 270) else "left"
        beyond = len(net) * 1.07 + 4.6                       # where KiCad writes the pages it is also on
        self.items.append(["global_label", Q(net), ["shape", shape], ["at", x, y, rot], ["fields_autoplaced", "yes"],
                           font(justify=just), ["uuid", uid("gl", self.key, net, x, y)],
                           ["property", Q("Intersheetrefs"), Q("${INTERSHEET_REFS}"),
                            ["at", r3(x - beyond if rot == 180 else x + beyond), y, 0], font(1.0, just)]])
        self.nodes.add((x, y))
        self.labels.append((net, (x, y)))

    def label(self, net, at, rot=0):
        """A name for the wire at this point (a net's name on the board; on this page only)."""
        x, y = r3(at[0]), r3(at[1])
        self.items.append(["label", Q(net), ["at", x, y, rot],
                           font(justify="right bottom" if rot in (180, 270) else "left bottom"),
                           ["uuid", uid("ll", self.key, net, x, y)]])
        self.nodes.add((x, y))

    def nc(self, at):
        self.items.append(["no_connect", ["at", r3(at[0]), r3(at[1])], ["uuid", uid("nc", self.key, r3(at[0]), r3(at[1]))]])

    def text(self, s, at, size=1.27, bold=False, justify="left bottom"):
        self.items.append(["text", Q(s), ["exclude_from_sim", "no"], ["at", r3(at[0]), r3(at[1]), 0],
                           font(size, justify, bold), ["uuid", uid("text", self.key, s[:40], r3(at[0]), r3(at[1]))]])

    # ---- finishing -------------------------------------------------------
    def _wires_and_junctions(self):
        """The wires, cut wherever something joins them part-way, and a dot wherever three or more
        ends meet (KiCad joins wires only at their ends and at dots)."""
        ends = set(self.nodes)
        for a, b in self.segments:
            ends.add(a)
            ends.add(b)
        pieces = []
        for a, b in self.segments:
            lo, hi = (a, b) if a <= b else (b, a)
            inner = [p for p in ends if p != lo and p != hi and
                     ((p[0] == lo[0] == hi[0] and lo[1] < p[1] < hi[1]) or (p[1] == lo[1] == hi[1] and lo[0] < p[0] < hi[0]))]
            chain = [lo] + sorted(inner) + [hi]
            pieces += list(zip(chain, chain[1:]))
        pieces = sorted(set(pieces))
        count = {}
        for a, b in pieces:
            count[a] = count.get(a, 0) + 1
            count[b] = count.get(b, 0) + 1
        out = [["wire", ["pts", ["xy", a[0], a[1]], ["xy", b[0], b[1]]], ["stroke", ["width", 0], ["type", "default"]],
                ["uuid", uid("w", self.key, a, b)]] for a, b in pieces]
        for p, n in sorted(count.items()):
            if n + self.pins_at.get(p, 0) >= 3:
                out.append(["junction", ["at", p[0], p[1]], ["diameter", 0], ["color", 0, 0, 0, 0],
                            ["uuid", uid("j", self.key, p)]])
        return out

    def tree(self, version, file_uuid, title_block, extra=()):
        """The page as a KiCad schematic file."""
        self.text(self.title, (15.0, 17.0), size=3.0, bold=True)
        y = 22.0
        for line in self.note.split("\n") if self.note else []:
            self.text(line, (15.0, y), size=1.5)
            y += 3.0
        libs = ["lib_symbols"] + [self.syms[k] for k in sorted(self.syms)]
        tree = ["kicad_sch", ["version", version], ["generator", Q("eeschema")], ["generator_version", Q("10.0")],
                ["uuid", file_uuid], ["paper", Q(self.paper)], title_block, libs]
        tree += self.items + self._wires_and_junctions() + list(extra)
        if self.number == 1:
            tree.append(["sheet_instances", ["path", Q("/"), ["page", Q("1")]]])
        tree.append(["embedded_fonts", "no"])
        return tree
