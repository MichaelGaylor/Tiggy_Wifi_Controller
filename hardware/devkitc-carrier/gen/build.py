#!/usr/bin/env python3
"""
Generates the whole KiCad project from design.py and checks it with KiCad's own tools.

    python gen/build.py [--no-route | --keep-routing] [--tracklayer DIR]

Steps: project library -> schematic -> electrical rule check -> netlist -> board with the parts
placed -> autorouting (Tracklayer, https://github.com/MichaelGaylor/Tracklayer) -> copper pours ->
design rule check with the schematic compared against the board.  Reports land in build/.

Needs KiCad 10 (kicad-cli and the Python that comes with it) and, for routing, Tracklayer with
numpy.  Hand edits to the .kicad_sch / .kicad_pcb are overwritten: change design.py.
"""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import design  # noqa: E402


def find_kicad():
    for base in (os.environ.get("KICAD_BIN", ""), r"C:\Program Files\KiCad\10.0\bin", "/usr/bin",
                 "/Applications/KiCad/KiCad.app/Contents/MacOS"):
        exe = os.path.join(base, "kicad-cli.exe" if os.name == "nt" else "kicad-cli")
        if base and os.path.isfile(exe):
            py = os.path.join(base, "python.exe") if os.name == "nt" else sys.executable
            return exe, py
    sys.exit("kicad-cli not found: set KICAD_BIN to KiCad's bin directory")


def run(cmd, **kw):
    print("  >", " ".join(os.path.basename(c) if os.path.isabs(c) else c for c in cmd[:3]), "...")
    r = subprocess.run(cmd, cwd=kw.pop("cwd", PROJECT), capture_output=True, text=True, **kw)
    if r.returncode:
        sys.exit("FAILED:\n" + r.stdout[-3000:] + r.stderr[-3000:])
    return r.stdout + r.stderr


def count(report, word):
    return sum(1 for line in open(report, encoding="utf-8", errors="replace") if line.strip().startswith("; " + word))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-route", action="store_true", help="stop at the placed, unrouted board")
    ap.add_argument("--keep-routing", action="store_true",
                    help="use the routing of the last build again (refused if a pad or a net has changed)")
    ap.add_argument("--tracklayer", default=os.environ.get("TRACKLAYER", ""), help="Tracklayer's source directory, if it is not installed")
    ap.add_argument("--route-args", default="--engine grid --pitch 0.1 --no-fanout --layer-cost B.Cu=1.5",
                    help="options for Tracklayer (default: %(default)s - its grid engine routes this board in a few minutes)")
    args = ap.parse_args()
    cli, kipy = find_kicad()
    name = design.NAME
    build = os.path.join(PROJECT, "build")
    os.makedirs(build, exist_ok=True)
    sch, pcb, pro = (os.path.join(PROJECT, name + ext) for ext in (".kicad_sch", ".kicad_pcb", ".kicad_pro"))

    print("library, schematic, project file")
    run([sys.executable, os.path.join(HERE, "gen_lib.py")])
    run([cli, "sym", "upgrade", "--force", os.path.join(PROJECT, "tiggy.kicad_sym")])
    run([sys.executable, os.path.join(HERE, "gen_sch.py")])
    run([sys.executable, os.path.join(HERE, "gen_pro.py")])

    print("electrical rule check")
    erc = os.path.join(build, "erc.rpt")
    run([cli, "sch", "erc", "--severity-all", "-o", erc, sch])
    print("     %d errors, %d warnings" % (count(erc, "error"), count(erc, "warning")))
    if count(erc, "error"):
        sys.exit("see " + erc)
    net = os.path.join(build, "netlist.net")
    run([cli, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", net, sch])
    run([cli, "sch", "export", "pdf", "-o", os.path.join(PROJECT, name + "-schematic.pdf"), sch])
    run([cli, "sch", "export", "bom", "--fields", "Reference,Value,Footprint,${QUANTITY}", "--labels",
         "References,Value,Footprint,Quantity", "--group-by", "Value,Footprint", "--exclude-dnp",
         "-o", os.path.join(PROJECT, name + "-bom.csv"), sch])

    print("board")
    unrouted = os.path.join(build, "unrouted.kicad_pcb")
    print("    ", run([kipy, os.path.join(HERE, "gen_pcb.py"), net, unrouted]).strip().splitlines()[-1])
    shutil.copyfile(pro, os.path.join(build, "unrouted.kicad_pro"))
    if args.no_route:
        shutil.copyfile(unrouted, pcb)
        print("unrouted board written to", pcb)
        return 0

    routed = os.path.join(build, "routed.kicad_pcb")
    if args.keep_routing and os.path.isfile(routed):
        print("routing: kept from the last build")
    else:
        route(args, unrouted, routed, build)
    print("    ", run([kipy, os.path.join(HERE, "finish_pcb.py"), routed, unrouted, pcb]).strip().splitlines()[-1])

    run([cli, "pcb", "export", "pdf", "--layers", "F.Fab,F.SilkS,Edge.Cuts", "--mode-single", "--black-and-white",
         "-o", os.path.join(PROJECT, name + "-assembly.pdf"), pcb])

    print("design rule check")
    drc = os.path.join(build, "drc.rpt")
    run([cli, "pcb", "drc", "--schematic-parity", "--severity-all", "-o", drc, pcb])
    text = open(drc, encoding="utf-8", errors="replace").read()
    for line in text.splitlines():
        if line.startswith("** Found"):
            print("    ", line.strip("* "))
    kinds = {}
    for line in text.splitlines():
        if line.startswith("["):
            kinds[line[1:line.index("]")]] = kinds.get(line[1:line.index("]")], 0) + 1
    print("    ", ", ".join("%d %s" % (n, k) for k, n in sorted(kinds.items())) or "nothing reported")
    return 0


def route(args, unrouted, routed, build):
    print("routing (this takes a few minutes)")
    env = dict(os.environ)
    if args.tracklayer:
        env["PYTHONPATH"] = args.tracklayer + os.pathsep + env.get("PYTHONPATH", "")
    if os.path.exists(routed):
        os.remove(routed)
    r = subprocess.run([sys.executable, "-m", "tracklayer", unrouted, "-o", routed, "--svg",
                        os.path.join(build, "routed.svg")] + args.route_args.split(),
                       cwd=PROJECT, env=env, capture_output=True, text=True)
    out = r.stdout + r.stderr
    if not os.path.isfile(routed):                      # (it also ends with an error when a net is left unrouted)
        sys.exit("routing failed:\n" + out[-3000:])
    with open(os.path.join(build, "route.log"), "w", encoding="utf-8") as f:
        f.write(out)
    print("    ", "\n     ".join(out.strip().splitlines()[-4:]))


if __name__ == "__main__":
    sys.exit(main())
