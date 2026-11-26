"""Writes the schematic from design.py.

Every part is drawn with the name of its net on each pin, the parts of one block of the circuit
side by side under the block's heading.  Connections are by those names: no wires are drawn.
"""
import math
import os
import uuid

import design
import symlib
from sexp import Q, dump, find, first

SCH_VERSION = "20260306"
NS = uuid.UUID("6f3a9d2e-5b1c-4a77-9c0e-7d1e5a0c1001")
CHAR = 1.08                                             # width of a label character, mm
GAP = 3.81

NOTES = {
    "Controller": "ESP32-S3-DevKitC-1 (octal memory), firmware environment esp32s3-devkitc, terminal layout \"four axes\".",
    "Supply": "12 to 24 V in.  5 V for the board; the DevKitC is fed through D3 so that its USB port never supplies the relays.",
    "Drives": "Step and direction as 5 V push-pull outputs.  The 10 k resistors hold them low while the ESP32 is in reset.",
    "Inputs": "A switch or an NPN sensor between a terminal and 0 V lights the channel's LED and pulls the ESP32's input low.",
    "Spindle sensor": "Encoder A, B, index (or one pulse per revolution on Z).  5 V pull-ups, clamped; up to 24 V signals tolerated.  Without a sensor: inputs 2 to 4.",
    "Outputs": "Relays on auxiliary outputs 1 to 3 (contacts for low voltage only on this revision).",
    "Spindle speed": "The spindle PWM filtered and doubled: 0 to 10 V for a frequency converter (needs 12 V or more at the supply terminal).",
    "Ethernet": "Socket for a USR-ES1 (W5500) module.  Q1 selects it only after the ESP32 has left reset, so the module cannot disturb GPIO46 as a boot option; R on IO0 keeps GPIO0 high at reset.",
    "Mechanical": "",
}


def uid(*key):
    return Q(str(uuid.uuid5(NS, "/".join(str(k) for k in key))))


def snap(v):
    return round(v / 1.27) * 1.27


def font(size=1.27, justify=None, bold=False):
    f = ["font", ["size", size, size]]
    if bold:
        f.append(["bold", "yes"])
    e = ["effects", f]
    if justify:
        e.append(["justify"] + justify.split())
    return e


def prop(name, value, x, y, hide=False, justify=None):
    p = ["property", Q(name), Q(value), ["at", round(x, 3), round(y, 3), 0]]
    if hide:
        p.append(["hide", "yes"])
    p.append(font(justify=justify))
    return p


class Cell:
    """One unit of one part, with the room its labels need."""

    def __init__(self, part, sym, unit, nunits):
        self.part, self.sym, self.unit, self.nunits = part, sym, unit, nunits
        self.pins = [p for p in symlib.pins(sym) if p[0] in (unit, 0)]
        nets = part["pins"]
        xs = [p[3] for p in self.pins] or [0.0]
        ys = [p[4] for p in self.pins] or [0.0]
        self.horizontal = any(p[5] in (0, 180) for p in self.pins)
        l, r, t, b = min(xs) - 2.0, max(xs) + 2.0, max(ys) + 2.0, min(ys) - 2.0
        for (_, num, _, x, y, ang, _, _) in self.pins:
            n = len(nets.get(num, "")) * CHAR + 2.0
            if ang == 0:
                l = min(l, x - n)
            elif ang == 180:
                r = max(r, x + n)
            elif ang == 270:
                t = max(t, y + n)
            else:
                b = min(b, y - n)
        text = max(len(part["ref"]), len(part["value"])) * CHAR + 4.0
        if self.horizontal or not self.pins:
            t += 6.5
            b -= 4.0
            r = max(r, min(xs) + text)
        else:
            l = min(l, -3.5)
            r = max(r, 3.0 + text)
        self.l, self.r, self.t, self.b = l, r, t, b
        self.w, self.h = r - l, t - b


def build():
    libs = [os.path.join(os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad"), "symbols"), PROJECT]
    syms, cells = {}, {}
    for part in design.PARTS:
        lid = part["sym"]
        if lid not in syms:
            syms[lid] = symlib.load([libs[1]] if lid.startswith("tiggy:") else [libs[0]], lid)
        units = symlib.units(syms[lid])
        cells.setdefault(part["group"], []).extend(Cell(part, syms[lid], u, len(units)) for u in units)
    flag = symlib.load([libs[0]], "power:PWR_FLAG")

    # --- lay the blocks out in two columns ------------------------------------
    items = []
    col_w = 395.0
    col_x = [15.0, 15.0 + col_w + 12.0]
    col_y = [22.0, 22.0]
    root = uid("root")
    for group in dict.fromkeys(p["group"] for p in design.PARTS):
        c = 0 if col_y[0] <= col_y[1] else 1
        gx, gy = col_x[c], col_y[c]
        items.append(["text", Q(group), ["exclude_from_sim", "no"], ["at", gx, gy + 5.0, 0],
                      font(3.0, "left bottom", bold=True), ["uuid", uid("title", group)]])
        if NOTES.get(group):
            items.append(["text", Q(NOTES[group]), ["exclude_from_sim", "no"], ["at", gx, gy + 9.5, 0],
                          font(1.6, "left bottom"), ["uuid", uid("note", group)]])
        x, y, row_h = gx + 3.0, gy + 13.0, 0.0
        for cell in cells[group]:
            if x + cell.w > gx + col_w and x > gx + 3.0:
                x, y, row_h = gx + 3.0, y + row_h + GAP, 0.0
            sx, sy = snap(x - cell.l), snap(y + cell.t)
            items += place(cell, sx, sy, root)
            x += cell.w + GAP
            row_h = max(row_h, cell.h)
        bottom = y + row_h + 3.0
        items.append(["rectangle", ["start", gx, gy + 11.0], ["end", gx + col_w, bottom],
                      ["stroke", ["width", 0.15], ["type", "dash"]], ["fill", ["type", "none"]],
                      ["uuid", uid("frame", group)]])
        col_y[c] = bottom + 6.0

    # power flags, beside the last block
    c = 0 if col_y[0] <= col_y[1] else 1
    x, y = col_x[c] + 10.0, col_y[c] + 14.0
    for i, net in enumerate(design.POWER_FLAGS):
        fx, fy = snap(x + 25.0 * i), snap(y)
        items.append(["symbol", ["lib_id", Q("power:PWR_FLAG")], ["at", fx, fy, 0], ["unit", 1], ["body_style", 1],
                      ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"], ["in_pos_files", "yes"],
                      ["dnp", "no"], ["uuid", uid("flag", net)],
                      prop("Reference", "#FLG%02d" % (i + 1), fx, fy - 5.0, hide=True),
                      prop("Value", "PWR_FLAG", fx, fy - 3.5),
                      prop("Footprint", "", fx, fy, hide=True), prop("Datasheet", "", fx, fy, hide=True),
                      prop("Description", "", fx, fy, hide=True),
                      ["pin", Q("1"), ["uuid", uid("flagpin", net)]],
                      ["instances", ["project", Q(design.NAME), ["path", Q("/" + root),
                                                                 ["reference", Q("#FLG%02d" % (i + 1))], ["unit", 1]]]]])
        items.append(["label", Q(net), ["at", fx, fy, 270], font(justify="right bottom"), ["uuid", uid("flaglabel", net)]])
    col_y[c] = y + 25.0

    # a sheet the size of what is on it (plus the title block)
    width = math.ceil((col_x[1] + col_w + 15.0) / 10.0) * 10
    height = math.ceil((max(col_y) + 45.0) / 10.0) * 10

    lib_symbols = ["lib_symbols"] + [syms[k] for k in sorted(syms)] + [flag]
    sch = ["kicad_sch", ["version", SCH_VERSION], ["generator", Q("eeschema")], ["generator_version", Q("10.0")],
           ["uuid", root], ["paper", Q("User"), width, height],
           ["title_block", ["title", Q("Tiggy DevKitC carrier")], ["rev", Q(design.REV)],
            ["comment", 1, Q("Four axes, Ethernet, spindle sensor, opto-isolated inputs, three relays")],
            ["comment", 2, Q("Generated from gen/design.py - change the circuit there")]],
           lib_symbols] + items + [["sheet_instances", ["path", Q("/"), ["page", Q("1")]]], ["embedded_fonts", "no"]]
    return dump(sch) + "\n"


def place(cell, sx, sy, root):
    part, out = cell.part, []
    key = (part["ref"], cell.unit)
    pins_all = symlib.pins(cell.sym)
    body_l = min([p[3] + (p[6] if p[5] == 0 else 0) for p in cell.pins] or [-3.0])
    top = max([p[4] for p in cell.pins] or [2.0])
    bot = min([p[4] for p in cell.pins] or [-2.0])
    if cell.horizontal or not cell.pins:
        ref_at, val_at, just = (sx + body_l, sy - top - 5.0), (sx + body_l, sy - bot + 4.6), "left"
    else:
        ref_at, val_at, just = (sx + 3.2, sy - 1.4), (sx + 3.2, sy + 1.4), "left"
    in_bom = "no" if part["ref"].startswith(("H", "JP")) else "yes"     # as their footprints say
    s = ["symbol", ["lib_id", Q(part["sym"])], ["at", round(sx, 3), round(sy, 3), 0], ["unit", cell.unit],
         ["body_style", 1], ["exclude_from_sim", "no"], ["in_bom", in_bom], ["on_board", "yes"], ["in_pos_files", "yes"],
         ["dnp", "no"], ["uuid", uid("sym", *key)],
         prop("Reference", part["ref"], ref_at[0], ref_at[1], justify=just),
         prop("Value", part["value"], val_at[0], val_at[1], justify=just),
         prop("Footprint", part["fp"], sx, sy, hide=True),
         prop("Datasheet", "", sx, sy, hide=True),
         prop("Description", "", sx, sy, hide=True)]
    for name, value in part["fields"].items():
        s.append(prop(name, value, sx, sy, hide=True))
    for p in pins_all:
        if p[0] in (cell.unit, 0):
            s.append(["pin", Q(p[1]), ["uuid", uid("pin", *key, p[1])]])
    s.append(["instances", ["project", Q(design.NAME), ["path", Q("/" + root), ["reference", Q(part["ref"])],
                                                        ["unit", cell.unit]]]])
    out.append(s)
    for (_, num, _, x, y, ang, _, _) in cell.pins:
        px, py = round(sx + x, 3), round(sy - y, 3)
        net = part["pins"].get(num)
        if net is None:
            out.append(["no_connect", ["at", px, py], ["uuid", uid("nc", *key, num)]])
            continue
        rot = {0: 180, 180: 0, 270: 90, 90: 270}[ang]
        out.append(["label", Q(net), ["at", px, py, rot],
                    font(justify="right bottom" if rot in (180, 270) else "left bottom"),
                    ["uuid", uid("label", *key, num)]])
    return out


PROJECT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

if __name__ == "__main__":
    text = build()
    with open(os.path.join(PROJECT, design.NAME + ".kicad_sch"), "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("schematic written: %d parts" % len(design.PARTS))
