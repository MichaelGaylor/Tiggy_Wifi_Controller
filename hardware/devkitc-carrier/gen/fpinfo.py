"""Print pad positions and courtyard extents of library footprints (run with KiCad's python)."""
import os
import sys
import pcbnew

K = os.path.join(os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad"), "footprints")
for name in sys.argv[1:]:
    lib, fp = name.split(":")
    m = pcbnew.FootprintLoad(K + "\\" + lib + ".pretty", fp)
    if m is None:
        print("MISSING", name)
        continue
    pads = [(p.GetNumber(), round(pcbnew.ToMM(p.GetPosition().x), 2), round(pcbnew.ToMM(p.GetPosition().y), 2)) for p in m.Pads()]
    bb = m.GetBoundingBox(False)
    cy = m.GetCourtyard(pcbnew.F_CrtYd).BBox()
    print(name)
    print("   pads", pads if len(pads) <= 8 else pads[:3] + ["..."] + pads[-2:])
    print("   courtyard x %.2f..%.2f  y %.2f..%.2f" % (pcbnew.ToMM(cy.GetLeft()), pcbnew.ToMM(cy.GetRight()),
                                                       pcbnew.ToMM(cy.GetTop()), pcbnew.ToMM(cy.GetBottom())))
