"""Symbols out of KiCad's libraries, flattened ("extends" resolved) for embedding in a schematic."""
import copy
import os

from sexp import Q, find, first, parse

_cache = {}


def _lib(path):
    if path not in _cache:
        with open(path, encoding="utf-8") as f:
            _cache[path] = {s[1]: s for s in find(parse(f.read()), "symbol")}
    return _cache[path]


def load(lib_dirs, lib_id):
    """Return the flattened (symbol "Lib:Name" ...) node for lib_id."""
    lib, name = lib_id.split(":")
    for d in lib_dirs:
        p = os.path.join(d, lib + ".kicad_sym")
        if os.path.isfile(p) and name in _lib(p):
            syms = _lib(p)
            node = copy.deepcopy(syms[name])
            ext = first(node, "extends")
            if ext is not None:
                parent = copy.deepcopy(syms[ext[1]])
                assert first(parent, "extends") is None, "two levels of extends: " + lib_id
                props = {p[1]: p for p in find(node, "property")}
                out = [parent[0], Q(name)]
                for item in parent[2:]:
                    if item[0] == "property":
                        out.append(props.pop(item[1], item))
                    elif item[0] == "symbol":
                        item[1] = Q(name + item[1][len(ext[1]):])
                        out.append(item)
                    else:
                        out.append(item)
                pos = max(i for i, it in enumerate(out) if isinstance(it, list) and it[0] == "property") + 1
                for p in props.values():
                    out.insert(pos, p)
                    pos += 1
                node = out
            node[1] = Q(lib_id)
            return node
    raise KeyError(lib_id)


def pins(sym):
    """[(unit, number, name, x, y, angle, length, electrical type)] of a flattened symbol."""
    out = []
    for sub in find(sym, "symbol"):
        unit = int(sub[1].rsplit("_", 2)[1])
        style = int(sub[1].rsplit("_", 2)[2])
        if style > 1:
            continue                                    # De Morgan alternative body
        for p in find(sub, "pin"):
            at = first(p, "at")
            out.append((unit, str(first(p, "number")[1]), str(first(p, "name")[1]), float(at[1]), float(at[2]),
                        int(float(at[3])) if len(at) > 3 else 0, float(first(p, "length")[1]), p[1]))
    return out


def units(sym):
    return sorted({u for u, *_ in pins(sym) if u > 0}) or [1]
