"""Compares the schematic as drawn with the circuit in design.py, pin for pin.

    python check_nets.py netlist.net        (the netlist KiCad exported from the schematic)

The pages are drawn by hand in gen_sch.py; this is what makes sure that every wire there joins
exactly the pins design.py says are joined, and nothing else.
"""
import sys

import design
from sexp import find, first, parse


def drawn(path):
    """{net name: set of (reference, pin)} from a KiCad netlist."""
    with open(path, encoding="utf-8") as f:
        root = parse(f.read())
    nets = {}
    for n in find(first(root, "nets"), "net"):
        nets[str(first(n, "name")[1])] = {(str(first(x, "ref")[1]), str(first(x, "pin")[1])) for x in find(n, "node")}
    return nets


def wanted():
    nets = {}
    for p in design.PARTS:
        for pin, net in p["pins"].items():
            nets.setdefault(net, set()).add((p["ref"], pin))
    return nets


def compare(path):
    """Returns ({design net: its name in the schematic}, [what is wrong])."""
    have, want = drawn(path), wanted()
    where = {pin: name for name, pins in have.items() for pin in pins}
    names, wrong = {}, []
    for net, pins in sorted(want.items()):
        found = {where.get(pin) for pin in pins}
        if None in found:
            wrong.append("%s: not connected at all: %s" % (net, sorted(p for p in pins if p not in where)))
        found.discard(None)
        if len(found) > 1:
            wrong.append("%s: drawn as %d separate nets: %s" % (net, len(found), sorted(found)))
        for name in found:
            extra = have[name] - pins
            if extra:
                wrong.append("%s: also joined to %s" % (net, sorted(extra)))
            names[net] = name
    used = {pin for pins in want.values() for pin in pins}
    for name, pins in sorted(have.items()):
        loose = pins - used
        if len(pins) > 1 and loose:
            wrong.append("%s joins pins the design leaves unconnected: %s" % (name, sorted(loose)))
    for supply in ("GND", "+5V", "+3V3", "VIN"):
        if names.get(supply) != supply:
            wrong.append("the supply %s is called %s in the schematic" % (supply, names.get(supply)))
    return names, sorted(set(wrong))


if __name__ == "__main__":
    names, wrong = compare(sys.argv[1])
    for line in wrong:
        print("  " + line)
    if wrong:
        sys.exit("the schematic does not match design.py")
    print("schematic and design.py agree: %d nets" % len(names))
