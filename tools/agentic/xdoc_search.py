"""Cross-document search: the scripted checklist of decision D102.

Every documentation task searches the project documents for the names it changed, the old
values and the decision numbers, and reports every hit as changed, or as left with its reason.
This tool lists the hits; the report gives each one its verdict.

Usage (run it with python -I, from any folder):

    python -I tools/team/xdoc_search.py "D93" "D1-D9" "87 buses"
    python -I tools/team/xdoc_search.py --terms-file terms.txt --out hits.txt
    python -I tools/team/xdoc_search.py --regex "D1 ?- ?D[0-9]+"

Options:
    --terms-file FILE  one search term per line (repeatable); blank lines and lines that start
                       with # are skipped
    --regex            the terms are Python regular expressions (default: plain text)
    --ignore-case      match without regard to case (default: case-sensitive)
    --width N          characters of each hit line shown around the match (default 200;
                       0 shows the whole line)
    --out FILE         also write the listing to FILE (UTF-8, LF)
    --root DIR         the project root (default: two folders above this file)
    --show-skipped     list the binary files that were skipped

The scope (D102): docs/ without docs/report/ (docs/requirements/, docs/safety/,
docs/registers/ and docs/hardware/ are part of docs/), framework/,
tools/dashboard/project_cards.json and tools/dashboard/notes.json. A file with a NUL byte is
binary: it is skipped and counted. Every other file is read as UTF-8 (undecodable bytes are
replaced), and its lines are numbered as an editor numbers them.

Each hit is printed as  path:line: [term] text.  The listing ends with the count by file, the
count by term and the totals. The exit code is 0 whether or not there are hits.
"""
import argparse
import os
import re
import sys

SCOPE_DIRS = ("docs", "framework", "docs/requirements", "docs/safety", "docs/registers", "docs/hardware")
SCOPE_FILES = ("tools/dashboard/project_cards.json", "tools/dashboard/notes.json")
EXCLUDE_DIRS = ("docs/report",)


def default_root():
    """The project root: this file is <root>/tools/team/xdoc_search.py."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def rel(root, path):
    return os.path.relpath(path, root).replace(os.sep, "/")


def norm(path):
    return os.path.normcase(os.path.normpath(os.path.realpath(path)))


def scope_files(root):
    """Every file in the scope, each once, sorted by its path; and the scope entries not found."""
    excluded = {norm(os.path.join(root, d)) for d in EXCLUDE_DIRS}
    seen, files, missing = set(), [], []

    def add(path):
        key = norm(path)
        if key not in seen:
            seen.add(key)
            files.append(path)

    for d in SCOPE_DIRS:
        top = os.path.join(root, d)
        if not os.path.isdir(top):
            missing.append(d + "/")
            continue
        for dirpath, dirnames, filenames in os.walk(top):
            dirnames[:] = sorted(n for n in dirnames if norm(os.path.join(dirpath, n)) not in excluded)
            for name in sorted(filenames):
                add(os.path.join(dirpath, name))
    for f in SCOPE_FILES:
        path = os.path.join(root, f)
        if os.path.isfile(path):
            add(path)
        else:
            missing.append(f)
    files.sort(key=lambda p: rel(root, p))
    return files, missing


def read_terms(args, parser):
    terms = list(args.terms)
    for name in args.terms_file:
        with open(name, encoding="utf-8") as f:
            for raw in f:
                term = raw.rstrip("\r\n")
                if term.strip() and not term.lstrip().startswith("#"):
                    terms.append(term)
    if not terms:
        parser.error("give at least one search term, on the command line or with --terms-file")
    return terms


def excerpt(line, start, end, width):
    """The line, or a window of `width` characters round the match, marked with ... where cut."""
    if width <= 0 or len(line) <= width:
        return line.strip()
    left = max(0, start - (width - (end - start)) // 2)
    right = min(len(line), left + width)
    left = max(0, right - width)
    text = line[left:right]
    return ("..." if left > 0 else "") + text + ("..." if right < len(line) else "")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Cross-document search for the documentation checklist (D102).")
    parser.add_argument("terms", nargs="*", help="the search terms")
    parser.add_argument("--terms-file", action="append", default=[], help="a file with one term per line")
    parser.add_argument("--regex", action="store_true", help="the terms are regular expressions")
    parser.add_argument("--ignore-case", action="store_true", help="match without regard to case")
    parser.add_argument("--width", type=int, default=200, help="characters shown round each match (0 = whole line)")
    parser.add_argument("--out", help="also write the listing to this file")
    parser.add_argument("--root", help="the project root")
    parser.add_argument("--show-skipped", action="store_true", help="list the binary files skipped")
    args = parser.parse_args(argv)

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

    root = os.path.abspath(args.root) if args.root else default_root()
    terms = read_terms(args, parser)
    flags = re.IGNORECASE if args.ignore_case else 0
    patterns = [(t, re.compile(t if args.regex else re.escape(t), flags)) for t in terms]

    listing = []

    def emit(text=""):
        listing.append(text)
        print(text)

    files, missing = scope_files(root)
    kind = "regular expressions" if args.regex else "plain text"
    case = "case-insensitive" if args.ignore_case else "case-sensitive"
    emit(f"== Cross-document search (D102), root {root}")
    emit(f"== Terms ({kind}, {case}): " + ", ".join(f'"{t}"' for t in terms))
    emit("== Scope: docs/ without docs/report/, framework/, " + ", ".join(SCOPE_FILES))
    for m in missing:
        emit(f"== Not found, so not searched: {m}")
    emit()

    by_file, by_term = {}, {t: 0 for t in terms}
    skipped = []
    for path in files:
        with open(path, "rb") as f:
            data = f.read()
        if b"\x00" in data:
            skipped.append(rel(root, path))
            continue
        name = rel(root, path)
        text = data.decode("utf-8", errors="replace")
        for number, line in enumerate(text.split("\n"), 1):
            line = line.rstrip("\r")
            for term, pattern in patterns:
                m = pattern.search(line)
                if m:
                    by_file[name] = by_file.get(name, 0) + 1
                    by_term[term] += 1
                    emit(f"{name}:{number}: [{term}] {excerpt(line, m.start(), m.end(), args.width)}")

    emit()
    emit("== Count by file")
    for name in sorted(by_file):
        emit(f"{by_file[name]:6d}  {name}")
    if not by_file:
        emit("     0  (no hits)")
    emit("== Count by term")
    for term in terms:
        emit(f"{by_term[term]:6d}  {term}")
    if args.show_skipped and skipped:
        emit("== Binary files skipped")
        for name in skipped:
            emit(f"        {name}")
    total = sum(by_file.values())
    emit(f"== {len(files) - len(skipped)} files searched, {len(skipped)} binary files skipped, "
         f"{total} hits in {len(by_file)} files")

    if args.out:
        out_dir = os.path.dirname(os.path.abspath(args.out))
        os.makedirs(out_dir, exist_ok=True)
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(listing) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
