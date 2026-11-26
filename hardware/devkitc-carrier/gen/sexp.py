"""Minimal S-expression reader and writer for KiCad files.

A list is a Python list whose first item is the tag; a quoted string is a Q (so it is written
back quoted), anything else a plain str.
"""


class Q(str):
    """A string that was (or must be) written in double quotes."""


def parse(text):
    pos, n = 0, len(text)
    stack, cur = [], None
    root = None
    while pos < n:
        c = text[pos]
        if c in " \t\r\n":
            pos += 1
        elif c == "(":
            new = []
            if cur is not None:
                cur.append(new)
                stack.append(cur)
            cur = new
            pos += 1
        elif c == ")":
            if stack:
                cur = stack.pop()
            else:
                root, cur = cur, None
            pos += 1
        elif c == '"':
            out, pos = [], pos + 1
            while text[pos] != '"':
                if text[pos] == "\\":
                    pos += 1
                    out.append({"n": "\n", "t": "\t", "r": "\r"}.get(text[pos], text[pos]))
                else:
                    out.append(text[pos])
                pos += 1
            pos += 1
            cur.append(Q("".join(out)))
        else:
            start = pos
            while pos < n and text[pos] not in " \t\r\n()":
                pos += 1
            cur.append(text[start:pos])
    return root


def _atom(a):
    if isinstance(a, Q):
        return '"' + a.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\n") + '"'
    if isinstance(a, float):
        s = ("%.6f" % a).rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(a)


def dump(node, indent=0):
    """Serialise; short lists stay on one line."""
    if not isinstance(node, list):
        return _atom(node)
    flat = "(" + " ".join(dump(x) for x in node) + ")"
    if len(flat) <= 100 and "\n" not in flat:
        return flat
    pad = "\t" * (indent + 1)
    head = [node[0]]
    rest = node[1:]
    while rest and not isinstance(rest[0], list):
        head.append(rest.pop(0))
    lines = ["(" + " ".join(_atom(x) for x in head)]
    for x in rest:
        lines.append(pad + dump(x, indent + 1))
    return "\n".join(lines) + "\n" + "\t" * indent + ")"


def find(node, tag):
    return [x for x in node[1:] if isinstance(x, list) and x and x[0] == tag]


def first(node, tag, default=None):
    for x in node[1:]:
        if isinstance(x, list) and x and x[0] == tag:
            return x
    return default
