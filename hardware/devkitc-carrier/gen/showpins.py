import os
import sys
import symlib
K = os.path.join(os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad"), "symbols")
for lid in sys.argv[1:]:
    s = symlib.load([K], lid)
    ps = symlib.pins(s)
    print(lid, "units", symlib.units(s))
    print("   " + "  ".join("%s%s=%s(%s)" % ("u%d:" % u if len(symlib.units(s)) > 1 else "", num, name, t[:3])
                            for u, num, name, x, y, a, l, t in sorted(ps, key=lambda p: (p[0], int(p[1]) if p[1].isdigit() else 0))))
