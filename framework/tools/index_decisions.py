"""Regenerate the decision log index from the decisions table of a markdown document.

The decision log is the decisions table of the architecture document (rows that start with "| D<n> |").
This tool writes the index docs/registers/decisions.json: one entry per decision with its id, its title
and where it is (file:line). Run it after every change to the decisions table.

  python framework/tools/index_decisions.py [--source docs/architecture.md] [--out docs/registers/decisions.json]
"""
import argparse
import datetime
import json
import os
import re

ROW = re.compile(r'^\| (D\d+) \| ([^|]+?) \|')


def index(source):
    rows = []
    with open(source, encoding='utf-8') as f:
        for n, line in enumerate(f, 1):
            m = ROW.match(line)
            if m:
                rows.append({'id': m.group(1), 'title': m.group(2).strip(), 'where': f'{source}:{n}'})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='docs/architecture.md')
    ap.add_argument('--out', default='docs/registers/decisions.json')
    a = ap.parse_args()
    rows = index(a.source)
    old = {}
    if os.path.exists(a.out):
        with open(a.out, encoding='utf-8') as f:
            old = json.load(f)
    ids = [r['id'] for r in rows]
    dup = sorted({i for i in ids if ids.count(i) > 1})
    out = {'about': old.get('about', f'The decision log index, generated from the decisions table of {a.source}.'),
           'updated': datetime.date.today().isoformat(), 'count': len(rows), 'decisions': rows}
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(f'{len(rows)} decisions indexed from {a.source} into {a.out}' + (f'; duplicate ids: {dup}' if dup else ''))


if __name__ == '__main__':
    main()
