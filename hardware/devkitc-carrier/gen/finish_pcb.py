"""Puts a routing onto the freshly generated board, fills the copper pours and saves it.

    python.exe finish_pcb.py routed.kicad_pcb unrouted.kicad_pcb out.kicad_pcb     (KiCad's own Python)

The tracks and vias are taken from the routed board, everything else from the unrouted one, so a
change that moves no pad and no net (silkscreen, values, text) does not need routing again.  If a
pad or a net has changed the routing is refused.  The project file beside the unrouted board gives
the net classes the pours keep clear by.
"""
import sys

import pcbnew


def pads(board):
    return sorted((fp.GetReference(), p.GetNumber(), p.GetPosition().x, p.GetPosition().y, p.GetNetname())
                  for fp in board.GetFootprints() for p in fp.Pads())


def main(routed, unrouted, out):
    src, dst = pcbnew.LoadBoard(routed), pcbnew.LoadBoard(unrouted)
    if pads(src) != pads(dst):
        changed = sorted(set(pads(src)) ^ set(pads(dst)))
        sys.exit("pads or nets have changed since the board was routed (%s ...): route it again" % (changed[:2],))
    assert not list(dst.GetTracks()), "the unrouted board has tracks"
    segs = vias = 0
    for t in src.GetTracks():
        net = dst.FindNet(t.GetNetname())
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
    print("%d track segments, %d vias; pours filled" % (segs, vias))


if __name__ == "__main__":
    main(*sys.argv[1:4])
