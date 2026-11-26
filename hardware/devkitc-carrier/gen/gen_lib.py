"""The project's own symbols and footprints: the sockets for the two plug-in modules.

Dimensions of the ESP32-S3-DevKitC-1 are Espressif's (drawing DXF_ESP32-S3-DevKitC-1_V1.1_20220429):
board 25.40 x 62.74 mm, two rows of 22 pins at 2.54 mm, the rows 22.86 mm apart, the last pin 8.00 mm
from the USB end.  A second row of holes 2.54 mm further out takes the 25.4 mm wide boards sold under
the same name.

Those of the USR-ES1 (W5500) module are from USR IOT's data sheet: board 23 x 25 mm, two rows of six
pins 20.32 mm apart, pin 1 of each 6.4 mm from the edge the socket overhangs by 3.5 mm.  Seen from
above with the socket away from you: left row J1 = GND GND MOSI SCLK SCS INT, right row J2 =
GND 3.3V 3.3V NC RST MISO, pin 1 of each at the socket end (the WIZ820io arrangement).
"""
import os

import design


def _fp(name, descr, items):
    return '(footprint "%s" (version 20221018) (generator tiggy_gen) (layer "F.Cu")\n  (descr "%s")\n  (attr through_hole)\n%s)\n' % (
        name, descr, "".join("  " + i + "\n" for i in items))


def _line(x1, y1, x2, y2, layer, w=0.12):
    return '(fp_line (start %.3f %.3f) (end %.3f %.3f) (stroke (width %.2f) (type solid)) (layer "%s"))' % (
        x1, y1, x2, y2, w, layer)


def _rect(x1, y1, x2, y2, layer, w=0.12):
    return [_line(x1, y1, x2, y1, layer, w), _line(x2, y1, x2, y2, layer, w), _line(x2, y2, x1, y2, layer, w),
            _line(x1, y2, x1, y1, layer, w)]


def _text(kind, text, x, y, layer, size=1.0, rot=0):
    return '(fp_text %s "%s" (at %.3f %.3f %d) (layer "%s") (effects (font (size %.2f %.2f) (thickness %.2f))))' % (
        kind, text, x, y, rot, layer, size, size, size * 0.15)


def _pad(num, x, y, first=False):
    return '(pad "%s" thru_hole %s (at %.3f %.3f) (size 1.7 1.7) (drill 1.0) (layers "*.Cu" "*.Mask"))' % (
        num, "rect" if first else "circle", x, y)


def devkit_footprint():
    it = [_text("reference", "REF**", 11.43, -9.0, "F.SilkS"),
          _text("value", "ESP32-S3-DevKitC-1", 11.43, 30.0, "F.Fab", rot=90)]
    for n in range(22):
        it.append(_pad(n + 1, 0, 2.54 * n, n == 0))
        it.append(_pad(n + 23, 22.86, 2.54 * n))
        it.append(_pad(n + 23, 25.40, 2.54 * n))           # for the boards that are 2.54 mm wider
    # the module as it sits: board outline, aerial, USB end
    it += _rect(-1.27, -1.40, 24.13, 61.34, "F.Fab", 0.10)
    it += _rect(2.43, -7.40, 20.43, -1.40, "F.Fab", 0.10)
    for x in (-1.27, 24.13):                               # corners of the outline on the silkscreen
        s = 1 if x < 0 else -1
        it += [_line(x, -1.40, x + 4 * s, -1.40, "F.SilkS")]   # (the USB end is the board's edge)
    it += [_line(-1.27, -1.40, -1.27, -1.9, "F.SilkS"), _line(24.13, -1.40, 24.13, -1.9, "F.SilkS")]
    it += _rect(2.43, -7.40, 20.43, -1.40, "F.SilkS")
    it.append('(fp_text user "AERIAL" (at 11.43 -4.4) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
    it.append('(fp_text user "USB" (at 11.43 59.6) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
    it.append('(fp_text user "22.86" (at 22.86 55.6 90) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.09))))')
    it.append('(fp_text user "25.40" (at 25.40 55.6 90) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.09))))')
    # courtyard: the socket strips only - parts go under the module
    it += _rect(-1.8, -1.8, 1.8, 55.14, "F.CrtYd", 0.05)
    it += _rect(21.06, -1.8, 27.2, 55.14, "F.CrtYd", 0.05)
    return _fp("ESP32-S3-DevKitC-1_Socket",
               "Two 1x22 sockets for an ESP32-S3-DevKitC-1 (rows 22.86 mm apart; second right-hand row for 25.4 mm boards)", it)


def usr_es1_footprint():
    it = [_text("reference", "REF**", 10.16, 16.6, "F.SilkS"),
          _text("value", "USR-ES1", 10.16, 6.35, "F.Fab")]
    for n in range(6):
        it.append(_pad(n + 1, 0, 2.54 * n, n == 0))
        it.append(_pad(n + 7, 20.32, 2.54 * n))
    it += _rect(-1.34, -6.4, 21.66, 18.6, "F.Fab", 0.10)       # the module's board
    it += _rect(2.11, -9.9, 18.21, 11.4, "F.Fab", 0.10)        # its RJ45 socket
    for x in (-1.34, 21.66):
        s = 1 if x < 0 else -1
        it += [_line(x, 18.6, x + 4 * s, 18.6, "F.SilkS"), _line(x, -6.4, x + 3 * s, -6.4, "F.SilkS"),
               _line(x, -6.4, x, -2.0, "F.SilkS"), _line(x, 14.7, x, 18.6, "F.SilkS")]
    it.append('(fp_text user "RJ45" (at 10.16 -4.6) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
    it += _rect(-1.8, -1.8, 1.8, 14.5, "F.CrtYd", 0.05)
    it += _rect(18.52, -1.8, 22.12, 14.5, "F.CrtYd", 0.05)
    return _fp("USR-ES1_Socket", "Two 1x6 sockets, 20.32 mm apart, for a USR-ES1 / WIZ820io style W5500 module", it)


def _symbol(name, ref, footprint, descr, left, right, width):
    """A box with the pins down its two sides: left/right are lists of (number, name)."""
    rows = max(len(left), len(right))
    top = 2.54 * (rows - 1) / 2.0
    top = round(top / 2.54) * 2.54
    hw = width / 2.0
    f = '(effects (font (size 1.27 1.27)))'
    out = ['(symbol "%s" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)' % name,
           '  (property "Reference" "%s" (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) (justify left)))' % (ref, -hw, top + 5.08),
           '  (property "Value" "%s" (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) (justify left)))' % (name, -hw, top + 3.0),
           '  (property "Footprint" "%s" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))' % footprint,
           '  (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
           '  (property "Description" "%s" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))' % descr,
           '  (symbol "%s_0_1" (rectangle (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.254) (type default)) (fill (type background))))' % (
               name, -hw, top + 1.9, hw, -(2.54 * (rows - 1) - top) - 1.9),
           '  (symbol "%s_1_1"' % name]
    for side, pins in ((0, left), (1, right)):
        for i, (num, pname) in enumerate(pins):
            x = -hw - 2.54 if side == 0 else hw + 2.54
            out.append('    (pin passive line (at %.2f %.2f %d) (length 2.54) (name "%s" %s) (number "%s" %s))' % (
                x, top - 2.54 * i, 0 if side == 0 else 180, pname, f, num, f))
    out.append('  )')
    out.append(')')
    return "\n".join("  " + line for line in out)


def _supply_symbol(name):
    """A supply symbol like KiCad's own +5V, for a supply its library has no name for."""
    f = '(effects (font (size 1.27 1.27)))'
    h = '(effects (font (size 1.27 1.27)) (hide yes))'
    line = '(stroke (width 0) (type default)) (fill (type none))'
    out = ['(symbol "%s" (power) (pin_numbers (hide yes)) (pin_names (offset 0) (hide yes)) (exclude_from_sim no) (in_bom yes) (on_board yes)' % name,
           '  (property "Reference" "#PWR" (at 0 -3.81 0) %s)' % h,
           '  (property "Value" "%s" (at 0 3.556 0) %s)' % (name, f),
           '  (property "Footprint" "" (at 0 0 0) %s)' % h,
           '  (property "Datasheet" "" (at 0 0 0) %s)' % h,
           '  (property "Description" "Supply symbol: joins everything on the net %s" (at 0 0 0) %s)' % (name, h),
           '  (symbol "%s_0_1"' % name,
           '    (polyline (pts (xy -0.762 1.27) (xy 0 2.54)) %s)' % line,
           '    (polyline (pts (xy 0 2.54) (xy 0.762 1.27)) %s)' % line,
           '    (polyline (pts (xy 0 0) (xy 0 2.54)) %s))' % line,
           '  (symbol "%s_1_1"' % name,
           '    (pin power_in line (at 0 0 90) (length 0) (name "" %s) (number "1" %s)))' % (f, f),
           ')']
    return "\n".join("  " + line for line in out)


def symbol_library():
    j1 = [(i + 1, n) for i, n in enumerate(design.DEVKIT_J1)]
    j3 = [(i + 23, n) for i, n in enumerate(design.DEVKIT_J3)]
    es_l = list(zip(range(1, 7), ["GND", "GND", "MOSI", "SCLK", "SCSn", "INTn"]))
    es_r = list(zip(range(7, 13), ["GND", "3V3", "3V3", "NC", "RSTn", "MISO"]))
    return "(kicad_symbol_lib (version 20231120) (generator tiggy_gen)\n%s\n%s\n%s\n)\n" % (
        _supply_symbol("VIN"),
        _symbol("ESP32-S3-DevKitC-1", "A", "tiggy:ESP32-S3-DevKitC-1_Socket",
                "Espressif ESP32-S3-DevKitC-1 on sockets: pins 1-22 header J1, 23-44 header J3, from the aerial end",
                j1, j3, 20.32),
        _symbol("USR-ES1", "A", "tiggy:USR-ES1_Socket",
                "USR-ES1 W5500 SPI Ethernet module on sockets: pins 1-6 header J1, 7-12 header J2", es_l, es_r, 15.24))


def write(project_dir):
    pretty = os.path.join(project_dir, "tiggy.pretty")
    os.makedirs(pretty, exist_ok=True)
    for name, text in (("ESP32-S3-DevKitC-1_Socket", devkit_footprint()), ("USR-ES1_Socket", usr_es1_footprint())):
        with open(os.path.join(pretty, name + ".kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    with open(os.path.join(project_dir, "tiggy.kicad_sym"), "w", encoding="utf-8", newline="\n") as f:
        f.write(symbol_library())
    with open(os.path.join(project_dir, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write('(sym_lib_table\n  (version 7)\n  (lib (name "tiggy")(type "KiCad")(uri "${KIPRJMOD}/tiggy.kicad_sym")(options "")(descr "Tiggy carrier board parts"))\n)\n')
    with open(os.path.join(project_dir, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write('(fp_lib_table\n  (version 7)\n  (lib (name "tiggy")(type "KiCad")(uri "${KIPRJMOD}/tiggy.pretty")(options "")(descr "Tiggy carrier board parts"))\n)\n')


if __name__ == "__main__":
    write(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    print("library written")
