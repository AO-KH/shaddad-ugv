"""Read a Phase 1 suite report (report.html) and write verification.json for make_mbd_chapters.py.

Run: python -I extract_verification.py <suite report folder> <verification.json>
"""
import html
import json
import os
import re
import sys

folder, out = sys.argv[1], sys.argv[2]
s = open(os.path.join(folder, 'report.html'), encoding='utf-8', errors='replace').read()


def cells(row):
    return [html.unescape(re.sub(r'\s+', ' ', re.sub(r'<[^>]+>', '', c))).strip()
            for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', row, flags=re.S)]


# summary table: Case | What it tests | Result | Checks | Key numbers | Time
rows = []
for r in re.findall(r'<tr>(.*?)</tr>', s, flags=re.S):
    c = cells(r)
    if len(c) >= 4 and c[2] in ('PASS', 'KNOWN', 'FAIL', 'ERROR', 'SKIPPED', 'BLOCKED') and '.' not in c[0]:
        rows.append([c[0], c[1], c[2], c[3]])
seen, uniq = set(), []
for r in rows:
    if r[0] not in seen:
        seen.add(r[0])
        uniq.append(r)
n = {k: sum(1 for r in uniq if r[2] == k) for k in ('PASS', 'KNOWN', 'FAIL', 'ERROR')}
txt = re.sub(r'\s+', ' ', re.sub(r'<[^>]+>', ' ', s))
m = re.search(r'Verdict:?\s*(\w+)', txt)
verdict = m.group(1) if m else ('PASS' if n['FAIL'] == 0 and n['ERROR'] == 0 else 'FAIL')
mins = re.search(r'in (\d+) min', txt)
summary = ('The Full suite ran on 27 October 2026 on the real-robot model: **%d of %d cases PASS**, %d FAIL, %d KNOWN, %d ERROR%s. '
           'The failing checks are explained in the next section.' %
           (n['PASS'], len(uniq), n['FAIL'], n['KNOWN'], n['ERROR'], (' in %s minutes' % mins.group(1)) if mins else ''))
json.dump({'full': {'summary': summary, 'verdict': verdict,
                    'caption': 'Phase 1 Full suite on the real-robot model (report `%s`)' % os.path.relpath(folder, os.path.join(folder, '..', '..', '..')).replace('\\', '/'),
                    'rows': uniq}}, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print(summary)
print(len(uniq), 'cases')
