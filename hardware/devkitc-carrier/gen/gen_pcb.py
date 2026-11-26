"""Builds the unrouted board from the netlist KiCad exported from the schematic.

Run with KiCad's own Python:   python.exe gen_pcb.py netlist.net board.kicad_pcb
Parts go where design.py puts them; nets, references and the link to each schematic symbol come
from the netlist, so the board cannot drift from the schematic.
"""
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design  # noqa: E402
from sexp import find, first, parse  # noqa: E402

KICAD_FP = os.path.join(os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad"), "footprints")
PROJECT = os.path.normpath(os.path.join(HERE, ".."))
OX, OY = 50.0, 40.0                                     # the board's top left corner on the sheet


def mm(v):
    return pcbnew.FromMM(float(v))


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def turn(dx, dy, rot):
    """A footprint-local offset as it lies on the board after KiCad's rotation."""
    a = math.radians(rot)
    return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)


def read_netlist(path):
    with open(path, encoding="utf-8") as f:
        root = parse(f.read())
    comps = {}
    for c in find(first(root, "components"), "comp"):
        # where its symbol is: the page (nothing for the first one), then the symbol itself
        page = str(first(first(c, "sheetpath"), "tstamps")[1])
        comps[str(first(c, "ref")[1])] = dict(value=str(first(c, "value")[1]), footprint=str(first(c, "footprint")[1]),
                                              path=page + str(first(c, "tstamps")[1]))
    pin_net = {}
    for n in find(first(root, "nets"), "net"):
        name = str(first(n, "name")[1])
        for node in find(n, "node"):
            pin_net[(str(first(node, "ref")[1]), str(first(node, "pin")[1]))] = name
    return comps, pin_net


def text(board, s, x, y, size=0.8, rot=0, h=None, layer=pcbnew.F_SilkS):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(size * 0.15))
    t.SetTextAngleDegrees(rot)
    t.SetHorizJustify(h if h is not None else pcbnew.GR_TEXT_H_ALIGN_CENTER)
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    t.SetPosition(pt(x, y))
    board.Add(t)
    return t


def zone(board, corners, layers):
    z = pcbnew.ZONE(board)
    ls = pcbnew.LSET()
    for layer in layers:
        ls.AddLayer(layer)
    z.SetLayerSet(ls)
    o = z.Outline()
    o.NewOutline()
    for x, y in corners:
        o.Append(mm(OX + x), mm(OY + y))
    board.Add(z)
    return z


def main(netlist, out):
    comps, pin_net = read_netlist(netlist)
    by_ref = {p["ref"]: p for p in design.PARTS}
    assert set(comps) == set(by_ref), sorted(set(comps) ^ set(by_ref))

    board = pcbnew.BOARD()
    board.GetDesignSettings().SetCopperLayerCount(2)
    tb = board.GetTitleBlock()
    tb.SetTitle("Tiggy DevKitC carrier")
    tb.SetRevision(design.REV)

    nets = {}
    for name in sorted(set(pin_net.values())):
        nets[name] = pcbnew.NETINFO_ITEM(board, name)
        board.Add(nets[name])

    for ref in sorted(comps, key=lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))):
        part, comp = by_ref[ref], comps[ref]
        assert comp["footprint"] == part["fp"], (ref, comp["footprint"], part["fp"])
        lib, name = part["fp"].split(":")
        libdir = os.path.join(PROJECT, lib + ".pretty") if lib == "tiggy" else os.path.join(KICAD_FP, lib + ".pretty")
        fp = pcbnew.FootprintLoad(libdir, name)
        assert fp is not None, part["fp"]
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(comp["value"])
        fp.SetPath(pcbnew.KIID_PATH(comp["path"]))
        x, y, rot = part["pcb"]
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            net = pin_net.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
        small = ref[0] in "RCD" or ref.startswith("OK") or ref.startswith("Q") or ref.startswith("U")
        r = fp.Reference()
        r.SetTextSize(pcbnew.VECTOR2I(mm(0.7 if small else 1.0), mm(0.7 if small else 1.0)))
        r.SetTextThickness(mm(0.1 if small else 0.15))
        if ref.startswith("H"):
            r.SetVisible(False)
        if ref.startswith("A") or (ref[0] in "RCDQ"):    # sockets: the silkscreen says what goes there; small
            r.SetLayer(pcbnew.F_Fab)                    # parts: on the assembly drawing, not across their pads
        if ref in design.TERMINALS:                     # a connector's reference goes inside its outline
            wide = "MSTB" in part["fp"]
            dx, dy = turn(-2.3 if wide else (-0.5 if len(design.TERMINALS[ref]) > 4 else -1.7), 6.0 if wide else 6.9, rot)
            r.SetPosition(pt(x + dx, y + dy))
            r.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
            r.SetTextThickness(mm(0.12))
            r.SetTextAngleDegrees(0 if rot == 180 else 90)
        if ref == "U1":                                 # on its own outline, clear of the parts above it
            r.SetPosition(pt(x + 2.54, y - 4.2))
        elif ref[0] == "U":                             # the other ICs: where the part will sit
            r.SetPosition(pt(x, y))
            r.SetTextAngleDegrees(0)
        board.Add(fp)

    # outline
    e = pcbnew.PCB_SHAPE(board)
    e.SetShape(pcbnew.SHAPE_T_RECT)
    e.SetStart(pt(0, 0))
    e.SetEnd(pt(design.BOARD_W, design.BOARD_H))
    e.SetLayer(pcbnew.Edge_Cuts)
    e.SetWidth(mm(0.1))
    board.Add(e)

    # 0 V: a plane on the back.  (For routing it is on the back only, so that the router gives every
    # surface-mounted 0 V pad its own via to it; finish_pcb.py then pours the front as well.)
    w, h = design.BOARD_W, design.BOARD_H
    z = zone(board, [(0.4, 0.4), (w - 0.4, 0.4), (w - 0.4, h - 0.4), (0.4, h - 0.4)], [pcbnew.B_Cu])
    z.SetNet(nets["GND"])
    z.SetZoneName("0V")
    z.SetLocalClearance(mm(0.3))
    z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)   # reliefs for the pins to be soldered by hand only
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)

    # no copper pour below the DevKitC's aerial
    ax, ay = design.DK_X, design.DK_Y
    k = zone(board, [(ax + 1.4, ay - 8.4), (ax + 21.5, ay - 8.4), (ax + 21.5, ay - 1.6), (ax + 1.4, ay - 1.6)],
             [pcbnew.F_Cu, pcbnew.B_Cu])
    k.SetIsRuleArea(True)
    k.SetZoneName("aerial")
    k.SetDoNotAllowZoneFills(True)
    k.SetDoNotAllowTracks(False)
    k.SetDoNotAllowVias(False)
    k.SetDoNotAllowPads(False)
    k.SetDoNotAllowFootprints(False)

    # what each terminal is: beside it, clear of the connector's body
    left, right = pcbnew.GR_TEXT_H_ALIGN_LEFT, pcbnew.GR_TEXT_H_ALIGN_RIGHT
    for ref, names in design.TERMINALS.items():
        x, y, rot = by_ref[ref]["pcb"]
        pitch = 5.08 if "MSTB" in by_ref[ref]["fp"] else 3.81
        inx, iny = turn(0, -1, rot)                      # away from the plug
        for i, name in enumerate(names):
            px, py = turn(pitch * i, 0, rot)
            if abs(iny) > 0.5:                           # connector on the top edge: text below the pin
                text(board, name, x + px, y + py + iny * 3.4, size=0.7 if len(name) > 3 else 0.8)
            elif pitch > 5:                              # relays, supply: up the gap between part and plug
                text(board, name, x + px + inx * 3.8, y + py, size=0.7, rot=90)
            else:
                text(board, name, x + px + inx * 2.5, y + py, h=left if inx > 0 else right)
    for ref, title in (("J2", "X DRIVE"), ("J3", "Y DRIVE"), ("J4", "Z DRIVE"), ("J5", "A DRIVE"), ("J8", "ENCODER"),
                       ("J12", "SPINDLE")):
        x, y, rot = by_ref[ref]["pcb"]
        n = len(design.TERMINALS[ref])
        text(board, title, x - 3.81 * (n - 1) / 2.0, y + 4.75, size=0.9)
    text(board, "LIMITS", 11.0, 27.9, size=1.0, h=left)
    text(board, "INPUTS", 11.0, 54.5, size=1.0, h=left)
    text(board, "Tiggy DevKitC carrier", 74.4, 60.0, size=1.4, h=left)
    text(board, "rev %s" % design.REV, 74.4, 62.6, size=1.1, h=left)
    text(board, "relay contacts: 30 V max", 74.4, 65.0, size=0.9, h=left)

    pcbnew.SaveBoard(out, board, True)                  # True: leave the project file alone
    print("board written: %d footprints, %d nets" % (len(comps), len(nets)))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
