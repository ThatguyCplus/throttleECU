"""Minimal S-expression reader/writer for KiCad files."""


class Str(str):
    """A quoted string token (as opposed to a bare atom)."""


def parse(text):
    tokens = _tokenize(text)
    pos = 0

    def read():
        nonlocal pos
        tok = tokens[pos]
        pos += 1
        if tok == "(":
            out = []
            while tokens[pos] != ")":
                out.append(read())
            pos += 1
            return out
        return tok

    return read()


def _tokenize(text):
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            out.append(c)
            i += 1
        elif c == '"':
            j, buf = i + 1, []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j:j + 2])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            out.append(Str("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            out.append(text[i:j])
            i = j
    return out


def dump(node, indent=0):
    tab = "\t" * indent
    if not isinstance(node, list):
        return _atom(node)
    if all(not isinstance(x, list) for x in node):
        return tab + "(" + " ".join(_atom(x) for x in node) + ")"
    # Keep element order: KiCad 7's parser rejects e.g. (effects hide (font ...)).
    first = next(i for i, x in enumerate(node) if isinstance(x, list))
    lines = [tab + "(" + " ".join(_atom(x) for x in node[:first])]
    for x in node[first:]:
        lines.append(dump(x, indent + 1) if isinstance(x, list) else "\t" * (indent + 1) + _atom(x))
    lines.append(tab + ")")
    return "\n".join(lines)


def _atom(x):
    if isinstance(x, Str):
        return '"' + x + '"'
    return str(x)


def find_symbol(lib_tree, name):
    for item in lib_tree:
        if isinstance(item, list) and item[:1] == ["symbol"] and item[1] == name:
            return item
    raise KeyError(name)
