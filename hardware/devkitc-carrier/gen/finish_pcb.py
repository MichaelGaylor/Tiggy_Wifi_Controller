"""Puts a routing onto the freshly generated board, fills the copper pours and saves it.

    python.exe finish_pcb.py routed.kicad_pcb unrouted.kicad_pcb out.kicad_pcb     (KiCad's own Python)

The tracks and vias are taken from the routed board, everything else from the unrouted one, so a
change that moves no pad and joins nothing differently (silkscreen, values, text, the schematic
redrawn, nets renamed) does not need routing again.  If a pad has moved or the connections have
changed the routing is refused.  The project file beside the unrouted board gives the net classes
the pours keep clear by.
"""
import fnmatch
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design  # noqa: E402


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
    pcbnew.SaveBoard(out, dst, True)                    # True: leave the project file alone
    print("net classes: " + classes(dst))
    print("%d track segments, %d vias%s; pours filled" % (segs, vias, ", %d nets renamed" % renamed if renamed else ""))


if __name__ == "__main__":
    main(*sys.argv[1:4])
