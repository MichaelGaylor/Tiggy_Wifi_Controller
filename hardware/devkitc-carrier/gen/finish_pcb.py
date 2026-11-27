"""Puts a routing onto the freshly generated board, fills the copper pours and saves it.

    python.exe finish_pcb.py routed.kicad_pcb unrouted.kicad_pcb out.kicad_pcb     (KiCad's own Python)

The tracks and vias are taken from the routed board, everything else from the unrouted one, so a
change that moves no pad and joins nothing differently (silkscreen, values, text, the schematic
redrawn, nets renamed) does not need routing again.  If a pad has moved or the connections have
changed the routing is refused.  The project file beside the unrouted board gives the net classes
the pours keep clear by.

A patch of a pour that the tracks have cut off from the rest is joined to it with a via (stitch).
"""
import fnmatch
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design  # noqa: E402

OX, OY = 50.0, 40.0                                     # the board's top left corner on the sheet (as gen_pcb.py)


def classes(board):
    """Checks that every net design.py puts in a class is in it on the board (a net named on one
    page of the schematic carries the page's name in front, which the project file's patterns have
    to allow for), and returns a line saying how many each class has."""
    found = {}
    for name, net in board.GetNetsByName().items():
        name = str(name)
        for cls, *_, patterns in design.NETCLASSES:
            if any(fnmatch.fnmatchcase(name.rsplit("/", 1)[-1], p) for p in patterns):
                if net.GetNetClassName() != cls:
                    sys.exit("net %s should be in class %s but is in %s" % (name, cls, net.GetNetClassName()))
                found[cls] = found.get(cls, 0) + 1
    return ", ".join("%s %d nets" % kv for kv in sorted(found.items()))


def pads(board):
    """{(reference, pad, x, y): net} of a board."""
    return {(fp.GetReference(), p.GetNumber(), p.GetPosition().x, p.GetPosition().y): p.GetNetname()
            for fp in board.GetFootprints() for p in fp.Pads()}


LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)
MM = pcbnew.FromMM


def uid(item):
    return item.m_Uuid.AsString()


def groups(board, code):
    """The pads of a net grouped by the copper that joins them: [set of ids, ...] with the largest
    group first (the ids of the tracks and vias of a group are in its set too), and {id: pad}."""
    conn = board.GetConnectivity()
    pads = {uid(p): p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetCode() == code}
    left, out = set(pads), []
    while left:
        first = left.pop()
        joined = {first} | {uid(i) for i in conn.GetConnectedItems(pads[first], 0)}
        out.append(joined)
        left -= joined
    out.sort(key=lambda g: -len(g & set(pads)))
    return out, pads


def inside(polys, piece, x, y, r):
    """Whether a disc lies within one piece of a filled pour (its centre and two rings are tried)."""
    if not polys.Contains(pcbnew.VECTOR2I(x, y), piece):
        return False
    for rr, n in ((r, 16), (r // 2, 8)):
        for k in range(n):
            a = 2 * math.pi * k / n
            if not polys.Contains(pcbnew.VECTOR2I(int(x + rr * math.cos(a)), int(y + rr * math.sin(a))), piece):
                return False
    return True


def stitch(board):
    """Joins what the tracks have cut off from the rest of a pour.

    The router takes every pad of the pour's net that it has given a via as connected, but the
    tracks of other nets can ring a patch of the pour, and the pads on that patch are then joined to
    each other and to nothing else.  Where the patch lies over (or under) the rest of the pour on
    the other side of the board, one via through both joins them.  Returns where vias were added;
    stops the build if a patch cannot be joined."""
    added = []
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        code, net = z.GetNetCode(), z.GetNetname()
        via_d, drill = next((via, hole) for name, width, gap, via, hole, patterns in reversed(design.NETCLASSES)
                            if not patterns or any(fnmatch.fnmatchcase(net, p) for p in patterns))
        for _ in range(50):
            board.BuildConnectivity()
            found, pads = groups(board, code)
            if len(found) < 2:
                break
            main, cut = found[0], found[1]
            # surface-mounted pads and vias lie within the copper of the pour they are part of
            at = {(name, layer): [] for name in ("main", "cut") for layer in LAYERS}
            for name, group in (("main", main), ("cut", cut)):
                for p in (pads[i] for i in group & set(pads)):
                    for layer in LAYERS:
                        if p.IsOnLayer(layer) and not p.HasHole():
                            at[(name, layer)].append(p.GetPosition())
            for t in board.GetTracks():
                if t.GetNetCode() == code and t.Type() == pcbnew.PCB_VIA_T:
                    name = "main" if uid(t) in main else "cut" if uid(t) in cut else None
                    for layer in LAYERS if name else ():
                        at[(name, layer)].append(t.GetPosition())
            fills = {layer: z.GetFilledPolysList(layer) for layer in LAYERS}
            pieces = {key: {i for i in range(fills[key[1]].OutlineCount()) for q in points if fills[key[1]].Contains(q, i)}
                      for key, points in at.items()}
            keep_off = [(p.GetPosition(), p.GetBoundingRadius()) for fp in board.GetFootprints() for p in fp.Pads()]
            keep_off += [(t.GetPosition(), t.GetWidth(pcbnew.F_Cu) // 2) for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]
            spot = None
            for margin in (0.5, 0.3, 0.15):                 # copper to spare round the via: as much as there is
                r = MM(via_d / 2 + margin)
                for a, b in (LAYERS, LAYERS[::-1]):
                    for i in sorted(pieces[("cut", a)]):
                        box = fills[a].Outline(i).BBox()
                        for y in range(box.GetTop(), box.GetBottom() + 1, MM(0.2)):
                            for x in range(box.GetLeft(), box.GetRight() + 1, MM(0.2)):
                                if inside(fills[a], i, x, y, r) and \
                                        any(inside(fills[b], j, x, y, r) for j in pieces[("main", b)]) and \
                                        all(math.hypot(x - c.x, y - c.y) >= rad + MM(via_d / 2 + 0.25) for c, rad in keep_off):
                                    spot = spot or (x, y)
                            if spot:
                                break
                        if spot:
                            break
                    if spot:
                        break
                if spot:
                    break
            if not spot:
                who = sorted("%s pad %s" % (pads[i].GetParentFootprint().GetReference(), pads[i].GetNumber()) for i in cut & set(pads))
                sys.exit("cut off from the rest of %s by the tracks, with no room for a via to join them: %s.  "
                         "Move a part or route again." % (net, ", ".join(who)))
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(*spot))
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetDrill(MM(drill))
            v.SetWidth(pcbnew.F_Cu, MM(via_d))
            v.SetNetCode(code)
            board.Add(v)
            added.append("%s at %.1f, %.1f" % (net, pcbnew.ToMM(spot[0]) - OX, pcbnew.ToMM(spot[1]) - OY))
            pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        else:
            sys.exit("the pour of %s is still in pieces after 50 vias" % net)
    return added


def main(routed, unrouted, out):
    src, dst = pcbnew.LoadBoard(routed), pcbnew.LoadBoard(unrouted)
    was, now = pads(src), pads(dst)
    if set(was) != set(now):
        sys.exit("pads have moved since the board was routed (%s ...): route it again" % (sorted(set(was) ^ set(now))[:2],))
    name, back = {}, {}                                 # a net of the routed board -> what it is called now
    for pad in sorted(was):
        if name.setdefault(was[pad], now[pad]) != now[pad] or back.setdefault(now[pad], was[pad]) != was[pad]:
            sys.exit("the connections have changed since the board was routed (at %s pad %s): route it again" % pad[:2])
    renamed = sum(1 for a, b in name.items() if a != b)
    assert not list(dst.GetTracks()), "the unrouted board has tracks"
    segs = vias = 0
    for t in src.GetTracks():
        net = dst.FindNet(name[t.GetNetname()])
        assert net is not None, t.GetNetname()
        if t.Type() == pcbnew.PCB_VIA_T:
            v = pcbnew.PCB_VIA(dst)
            v.SetPosition(t.GetPosition())
            v.SetViaType(t.GetViaType())
            v.SetLayerPair(t.TopLayer(), t.BottomLayer())
            v.SetDrill(t.GetDrillValue())
            v.SetWidth(pcbnew.F_Cu, t.GetWidth(pcbnew.F_Cu))
            v.SetNet(net)
            dst.Add(v)
            vias += 1
        else:
            s = pcbnew.PCB_TRACK(dst)
            s.SetStart(t.GetStart())
            s.SetEnd(t.GetEnd())
            s.SetWidth(t.GetWidth())
            s.SetLayer(t.GetLayer())
            s.SetNet(net)
            dst.Add(s)
            segs += 1
    for z in dst.Zones():                               # the 0 V plane was on the back only for routing
        if not z.GetIsRuleArea():
            both = pcbnew.LSET()
            both.AddLayer(pcbnew.F_Cu)
            both.AddLayer(pcbnew.B_Cu)
            z.SetLayerSet(both)
    pcbnew.ZONE_FILLER(dst).Fill(dst.Zones())
    joined = stitch(dst)
    pcbnew.SaveBoard(out, dst, True)                    # True: leave the project file alone
    print("net classes: " + classes(dst))
    print("%d track segments, %d vias%s; pours filled" % (segs, vias, ", %d nets renamed" % renamed if renamed else ""))
    if joined:
        print("pour cut by tracks, joined with a via: " + "; ".join(joined))


if __name__ == "__main__":
    main(*sys.argv[1:4])
