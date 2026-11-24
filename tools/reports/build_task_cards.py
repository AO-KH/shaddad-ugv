"""Build the NASL PDF of the project's task cards (the user's request of 2026-11-15): every card with its entry and
exit gates, predecessors and successors, inputs, outputs, lead and support, verification, size, priority, risks and
escalation, plus the life cycle, the card map, the critical path and the traceability to the review items.

usage:
  python tools/reports/build_task_cards.py [--no-pdf] [--version 1.0] [--date YYYY-MM-DD]

Source: tools/dashboard/project_cards.json (the single source; the dashboard's Project tab shows the same file).
Output: work/reports/project/NASL_PLN-001_Shaddad_Task_Cards_<date>_v<version>.docx and .pdf (the dashboard lists them
under "Project documents"). Figures are drawn as SVG and rendered to PNG by Edge headless, as in
build_engineering_report.py; the NASL look comes from nasl_docx.py. No installs.
"""
import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
from collections import defaultdict
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import nasl_docx as N  # noqa: E402
import build_engineering_report as E  # noqa: E402

_PJ = os.path.join(ROOT, 'project.json')
PROJ = json.load(open(_PJ, encoding='utf-8')) if os.path.exists(_PJ) else {}
PNAME = PROJ.get('project', 'Shaddad UGV')
PCODE = PROJ.get('code', 'SHD')
PENT = PROJ.get('entity', 'NASL Technologies (Robotat)')
CARDS = os.path.join(ROOT, *PROJ.get('paths', {}).get('cards', 'tools/dashboard/project_cards.json').split('/'))
OUTDIR = os.path.join(ROOT, 'work', 'reports', 'project')
FIGDIR = os.path.join(OUTDIR, 'fig')
DOC_NO = f'NASL-{PCODE}-PLN-001'

GATE_LABEL = {'G0': 'G0 MCR', 'G1': 'G1 SRR + SFR', 'G2': 'G2 PDR', 'G3': 'G3 CDR', 'BUILD': 'Build', 'G4': 'G4 TRR',
              'G5': 'G5 FCA / SVR', 'G6': 'G6 PRR', 'G7': 'G7 PCA'}
STATUS_LABEL = {'done': 'Done', 'in progress': 'In progress', 'in review': 'In review', 'needs you': 'Needs you',
                'blocked': 'Blocked', 'ready': 'Ready', 'todo': 'To do', 'planned': 'Planned', 'proposed': 'Proposed',
                'cancelled': 'Cancelled'}
STATUS_COLOR = {'done': N.NAVY, 'in progress': N.PURPLE, 'in review': N.PURPLE, 'needs you': N.MAGENTA,
                'blocked': N.GREY, 'ready': N.GREY, 'todo': N.GREY, 'planned': N.GREY, 'proposed': N.GREY,
                'cancelled': N.GREY}
SCORE_COLOR = {'met': N.NAVY, 'waived': N.PURPLE, 'partly': N.MAGENTA, 'not met': N.MAGENTA, 'n/a': N.GREY,
               'not assessed': N.GREY}
GATE_STATUS = {'open': 'open', 'in review': 'in review', 'passed': 'passed', 'passed with actions': 'passed with actions',
               'delta review': 'delta review', 'no-go': 'no-go'}


def readiness(g):
    m = [c for c in g.get('criteria', []) if c.get('class') == 'M']
    ok = [c for c in m if c.get('score') in ('met', 'waived', 'n/a')]
    return len(ok), len(m)
PRIO_LABEL = {'now': 'Now', 'next': 'Next', 'later': 'Later', 'done': 'Done'}
SIZE_TEXT = {'S': 'S (one engineer, one pass)', 'M': 'M (two or three engineers, or two passes)',
             'L': 'L (several engineers and system runs)', 'XL': 'XL (every subsystem, or several safety chains)'}
SIZE_W = {'S': 1, 'M': 2, 'L': 3, 'XL': 4}
LEVEL = {'L': 'low', 'M': 'medium', 'H': 'high'}


def nb(s):
    """No line break inside an ID: G1-01, G2-DSC, T-01, R-05, NASL-SHD-PLN-001."""
    return re.sub(r'(?<![A-Za-z0-9])((?:G\d|[A-Z]{1,4}))-(?=[A-Z0-9])', '\\1\u2011', s)


# ------------------------------------------------------------------ data
def load():
    D = json.load(open(CARDS, encoding='utf-8'))
    cards = D['cards']
    by = {c['id']: c for c in cards}
    succ = defaultdict(list)
    for c in cards:
        for p in c.get('predecessors', []):
            succ[p].append(c['id'])
    for c in cards:
        c['successors'] = succ.get(c['id'], [])
    return D, cards, by


def critical_path(cards, by):
    """The longest chain of open cards, weighted by size (S 1, M 2, L 3, XL 4)."""
    open_ = {c['id'] for c in cards if c['status'] != 'done'}
    memo = {}

    def best(n):
        if n in memo:
            return memo[n]
        w = SIZE_W.get(by[n].get('size', 'M'), 2)
        tail = max((best(s) for s in by[n]['successors'] if s in open_), key=lambda t: t[0], default=(0, []))
        memo[n] = (w + tail[0], [n] + tail[1])
        return memo[n]

    starts = [n for n in open_ if not any(p in open_ for p in by[n].get('predecessors', []))]
    return max((best(n) for n in starts), key=lambda t: t[0])


def decision_names(by):
    """'decision 2 (zone in front of the body, R-02)' in G1-03's links -> {'2': 'zone in front of the body'}."""
    out = {}
    for l in by.get('G1-03', {}).get('links', []):
        m = re.match(r'decision (\d+) \(([^,)]+)', l)
        if m:
            out[m.group(1)] = m.group(2)
    return out


# ------------------------------------------------------------------ figures (SVG, rendered by Edge)
FONT = "font-family=\"Segoe UI, Arial, sans-serif\""
HEX = {'navy': '#' + N.NAVY, 'mag': '#' + N.MAGENTA, 'pur': '#' + N.PURPLE, 'lil': '#' + N.LILAC, 'grey': '#' + N.GREY}


def fig_gates(D, cards, path):
    W, H = 1500, 268
    gates = D['gates']
    n = len(gates)
    gap, x0 = 8, 10
    w = (W - 2 * x0 - (n - 1) * gap) / n
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    for i, g in enumerate(gates):
        cs = [c for c in cards if c['gate'] == g['id']]
        done = sum(c['status'] == 'done' for c in cs)
        x = x0 + i * (w + gap)
        tip = 26
        pts = (f'{x},{20} {x + w - tip},{20} {x + w},{90} {x + w - tip},{160} {x},{160} '
               + (f'{x + tip},{90}' if i else f'{x},{90}'))
        passed = g.get('status') in ('passed', 'passed with actions')
        state = {'passed with actions': 'passed, actions'}.get(g.get('status', 'open'), g.get('status', 'open'))
        s.append(f'<polygon points="{pts}" fill="{HEX["navy"] if passed else HEX["lil"]}" stroke="{HEX["navy"]}" stroke-width="2"/>')
        tc = '#ffffff' if passed else HEX['navy']
        cx = x + w / 2 + (6 if i else -4)
        s.append(f'<text x="{cx}" y="68" text-anchor="middle" {FONT} font-size="26" font-weight="700" fill="{tc}">'
                 f'{escape(g["id"] if g["id"] != "BUILD" else "Build")}</text>')
        line2 = 'code, integrate' if g['id'] == 'BUILD' else g['name']
        s.append(f'<text x="{cx}" y="100" text-anchor="middle" {FONT} font-size="21" fill="{tc}">{escape(line2)}</text>')
        s.append(f'<text x="{cx}" y="132" text-anchor="middle" {FONT} font-size="17" fill="{tc}">'
                 f'{escape(g["scope"])}</text>')
        # progress under the chevron
        bw = w - 40
        s.append(f'<rect x="{x + 20}" y="182" width="{bw}" height="12" fill="#e4e2ef"/>')
        if cs:
            s.append(f'<rect x="{x + 20}" y="182" width="{bw * done / len(cs):.1f}" height="12" fill="{HEX["navy"]}"/>')
        s.append(f'<text x="{x + w / 2}" y="222" text-anchor="middle" {FONT} font-size="19" fill="{HEX["grey"]}">'
                 f'{done} of {len(cs)} cards done</text>')
        s.append(f'<text x="{x + w / 2}" y="250" text-anchor="middle" {FONT} font-size="18" font-weight="700" '
                 f'fill="{HEX["navy"] if passed else HEX["grey"]}">{escape(state)}</text>')
    s.append('</svg>')
    open(path, 'w', encoding='utf-8').write('\n'.join(s))


def fig_network(D, cards, by, path):
    gates = [g['id'] for g in D['gates']]
    cols = {g: [c['id'] for c in cards if c['gate'] == g] for g in gates}
    gi = {c['id']: gates.index(c['gate']) for c in cards}
    # order each column by the mean row of its neighbours in other columns (two sweeps each way)
    for _ in range(3):
        for direction in (1, -1):
            seq = gates if direction == 1 else gates[::-1]
            pos = {n: i for g in gates for i, n in enumerate(cols[g])}
            for g in seq:
                def key(n, g=g):
                    nbr = by[n].get('predecessors', []) if direction == 1 else by[n]['successors']
                    ys = [pos[m] for m in nbr if gi[m] != gates.index(g)]
                    return (sum(ys) / len(ys)) if ys else pos[n]
                cols[g].sort(key=key)
                pos.update({n: i for i, n in enumerate(cols[g])})
    W = 1500
    pitch = W / len(gates)
    bw, bh, vgap, top = 132, 40, 16, 92
    rows = max(len(v) for v in cols.values())
    H = top + rows * (bh + vgap) + 70
    xy = {}
    for i, g in enumerate(gates):
        n = len(cols[g])
        y0 = top + (rows - n) * (bh + vgap) / 2
        for j, cid in enumerate(cols[g]):
            xy[cid] = (i * pitch + (pitch - bw) / 2, y0 + j * (bh + vgap))
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H:.0f}" viewBox="0 0 {W} {H:.0f}">',
         f'<rect width="{W}" height="{H:.0f}" fill="#ffffff"/>',
         '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
         '<path d="M0,0 L10,5 L0,10 z" fill="#8a89a6"/></marker></defs>']
    for i, g in enumerate(gates):
        s.append(f'<rect x="{i * pitch + 6:.1f}" y="8" width="{pitch - 12:.1f}" height="{H - 16:.0f}" rx="6" '
                 f'fill="{"#f6f5fb" if i % 2 == 0 else "#efeef7"}"/>')
        s.append(f'<text x="{i * pitch + pitch / 2:.1f}" y="48" text-anchor="middle" {FONT} font-size="22" '
                 f'font-weight="700" fill="{HEX["navy"]}">{escape(GATE_LABEL[g])}</text>')
        cs = [by[c] for c in cols[g]]
        s.append(f'<text x="{i * pitch + pitch / 2:.1f}" y="74" text-anchor="middle" {FONT} font-size="16" '
                 f'fill="{HEX["grey"]}">{sum(c["status"] == "done" for c in cs)} / {len(cs)} done</text>')
    # edges under the boxes
    for c in cards:
        for p in c.get('predecessors', []):
            (x1, y1), (x2, y2) = xy[p], xy[c['id']]
            ya, yb = y1 + bh / 2, y2 + bh / 2
            if gi[p] < gi[c['id']]:      # forward: right side to left side
                xa, xb = x1 + bw, x2
                k = max(40, (xb - xa) / 2)
                d = f'M{xa:.1f},{ya:.1f} C{xa + k:.1f},{ya:.1f} {xb - k:.1f},{yb:.1f} {xb:.1f},{yb:.1f}'
                st = ''
            elif gi[p] == gi[c['id']]:   # same gate: an arc on the right of the column
                xa = xb = x1 + bw
                k = 28 + 0.12 * abs(yb - ya)
                d = f'M{xa:.1f},{ya:.1f} C{xa + k:.1f},{ya:.1f} {xb + k:.1f},{yb:.1f} {xb:.1f},{yb:.1f}'
                st = ''
            else:                         # backward (scheduled after a later gate's card)
                xa, xb = x1, x2 + bw
                d = f'M{xa:.1f},{ya:.1f} C{xa - 60:.1f},{ya:.1f} {xb + 60:.1f},{yb:.1f} {xb:.1f},{yb:.1f}'
                st = ' stroke-dasharray="6 4"'
            s.append(f'<path d="{d}" fill="none" stroke="#8a89a6" stroke-width="1.5" opacity="0.75"{st} '
                     'marker-end="url(#a)"/>')
    fill = {'done': HEX['navy'], 'in progress': HEX['pur'], 'needs you': HEX['mag'], 'todo': '#ffffff'}
    for c in cards:
        x, y = xy[c['id']]
        f = fill.get(c['status'], '#ffffff')
        tc = HEX['navy'] if c['status'] == 'todo' else '#ffffff'
        s.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw}" height="{bh}" rx="5" fill="{f}" stroke="{HEX["navy"]}" '
                 'stroke-width="1.6"/>')
        s.append(f'<text x="{x + bw / 2:.1f}" y="{y + bh / 2 + 7:.1f}" text-anchor="middle" {FONT} font-size="20" '
                 f'font-weight="700" fill="{tc}">{escape(c["id"])}</text>')
    # legend
    ly = H - 34
    lx = 20
    for k, lab in [('done', 'Done'), ('in progress', 'In progress'), ('needs you', 'Needs you'), ('todo', 'To do')]:
        s.append(f'<rect x="{lx}" y="{ly - 16}" width="34" height="20" rx="4" fill="{fill[k]}" stroke="{HEX["navy"]}" '
                 'stroke-width="1.5"/>')
        s.append(f'<text x="{lx + 44}" y="{ly}" {FONT} font-size="18" fill="{HEX["grey"]}">{lab}</text>')
        lx += 190
    s.append(f'<text x="{lx + 20}" y="{ly}" {FONT} font-size="18" fill="{HEX["grey"]}">Arrow: predecessor to successor; '
             'dashed: scheduled after a card of a later gate</text>')
    s.append('</svg>')
    open(path, 'w', encoding='utf-8').write('\n'.join(s))


# ------------------------------------------------------------------ the document
ID_ON_NAVY = 'CFC4EE'   # the card id in the header band: a light lilac that reads on navy


def span_cell(inner, total, fill, pad, borders='', valign='top'):
    """A cell across both columns of a card table, with its own fill, padding and borders."""
    tcpr = (f'<w:tcPr><w:tcW w:w="{total}" w:type="dxa"/><w:gridSpan w:val="2"/>{borders}<w:shd w:val="clear" '
            f'w:color="auto" w:fill="{fill}"/><w:tcMar><w:top w:w="{pad[0]}" w:type="dxa"/><w:left w:w="{pad[1]}" '
            f'w:type="dxa"/><w:bottom w:w="{pad[0]}" w:type="dxa"/><w:right w:w="{pad[1]}" w:type="dxa"/></w:tcMar>'
            f'<w:vAlign w:val="{valign}"/></w:tcPr>')
    return f'<w:tc>{tcpr}{inner}</w:tc>'


def card_table(d, c, by, dnames):
    """One card as one table, which starts a page. Its first row, the header band with the card's id and title, is a
    header row: Word repeats it at the top of every page the card runs onto, so each page starts with the task."""
    def ref(i):
        return f"**{i}** {by[i]['title']}" if i in by else i

    def bl(items):
        return [{'text': nb(t), 'bullet': True} for t in items] or ['-']

    st = c['status']
    widths = (1950, 7076)
    W = sum(widths)
    xml = []
    # 1. the header band: the id and the title
    hdr = d.para(d.run(c['id'], size=32, color=ID_ON_NAVY, bold=True) + d.run('   ', size=32)
                 + d.inline(nb(c['title']), size=32, color='FFFFFF', bold=True),
                 keep_next=True, keep_lines=True, spacing='<w:spacing w:after="0" w:line="252" w:lineRule="auto"/>')
    accent = f'<w:tcBorders><w:bottom w:val="single" w:sz="24" w:space="0" w:color="{N.MAGENTA}"/></w:tcBorders>'
    xml.append('<w:tr><w:trPr><w:cantSplit/><w:tblHeader/></w:trPr>'
               + span_cell(hdr, W, N.NAVY, (120, 170), accent, 'center') + '</w:tr>')
    # 2. at a glance
    prio = '' if c.get('priority') in ('done', None) else f" · priority {PRIO_LABEL.get(c.get('priority'), '-').lower()}"
    meta = d.para(d.inline(nb(f"{GATE_LABEL.get(c['gate'], c['gate'])} · WBS {c.get('wbs', '-')} · "
                              f"**{STATUS_LABEL.get(st, st)}**{prio} · size {c.get('size', '-')} · lead: "
                              f"{re.sub(r' *[(].*', '', c.get('lead', '-'))}"), size=17, color=N.GREY),
                  keep_next=True, keep_lines=True, spacing='<w:spacing w:after="0"/>')
    xml.append('<w:tr><w:trPr><w:cantSplit/></w:trPr>' + span_cell(meta, W, 'F7F6FB', (60, 170), '', 'center') + '</w:tr>')
    # 3. the purpose: large and clean, in a callout with a purple bar
    lab = d.para(d.run('PURPOSE', size=15, color=N.PURPLE, bold=True, spacing=30), keep_next=True, keep_lines=True,
                 spacing='<w:spacing w:after="50"/>')
    pur = d.para(d.inline(nb(c.get('purpose', '')), size=27, color=N.NAVY), keep_next=True, keep_lines=True,
                 spacing='<w:spacing w:after="0" w:line="276" w:lineRule="auto"/>')
    bar = f'<w:tcBorders><w:left w:val="single" w:sz="36" w:space="0" w:color="{N.PURPLE}"/></w:tcBorders>'
    xml.append('<w:tr><w:trPr><w:cantSplit/></w:trPr>' + span_cell(lab + pur, W, N.LILAC, (150, 220), bar, 'center')
               + '</w:tr>')
    # 4. the description: what the task is, the work, how, and what it delivers (this row may run onto the next page)
    paras = []
    for block in (c.get('description') or '').split('\n\n'):
        lines = [x for x in block.split('\n') if x.strip()]
        txt = [x for x in lines if not x.startswith('- ')]
        if txt:
            paras.append({'text': nb(' '.join(txt))})
        paras += [{'text': nb(x[2:]), 'bullet': True} for x in lines if x.startswith('- ')]
    if paras:
        cell = d._cell({'paras': paras}, W, 'FFFFFF', 19, False, 'top', (110, 170))
        xml.append('<w:tr>' + cell.replace(f'<w:tcW w:w="{W}" w:type="dxa"/>',
                                           f'<w:tcW w:w="{W}" w:type="dxa"/><w:gridSpan w:val="2"/>', 1) + '</w:tr>')
    # 5. the definition, grouped
    groups = []
    en, ex = c.get('entry', {}), c.get('exit', {})
    groups.append(('Start and finish', [
        ('Entry gate', [{'text': f"**{nb(en.get('gate', '-'))}**"}] + bl(en.get('criteria', []))),
        ('Exit gate', [{'text': f"**{nb(ex.get('gate', '-'))}**"}] + bl(ex.get('criteria', [])))]))
    groups.append(('Place in the plan', [
        ('Predecessors', bl([ref(i) for i in c.get('predecessors', [])]) if c.get('predecessors') else ['None']),
        ('Successors', bl([ref(i) for i in c['successors']]) if c['successors'] else ['None'])]))
    groups.append(('Inputs and outputs', [('Inputs', bl(c.get('inputs', []))), ('Outputs', bl(c.get('outputs', [])))]))
    rk = [f"{r['risk']} (likelihood {LEVEL.get(r['likelihood'], r['likelihood'])}, impact "
          f"{LEVEL.get(r['impact'], r['impact'])}). Mitigation: {r['mitigation']}" for r in c.get('risks', [])]
    checks = [('Verification', [nb(c.get('verification', '-'))])]
    if rk:
        checks.append(('Risks', bl(rk)))
    if c.get('escalation'):
        checks.append(('Escalation', bl([f"{e['when']}: to {e['to']}" for e in c['escalation']])))
    groups.append(('Checks, risks and escalation', checks))
    people = [('Lead', [{'text': f"**{nb(c.get('lead', '-'))}**"}])]
    if c.get('support'):
        people.append(('Support', bl(c['support'])))
    people.append(('Status and size', [{'text': f"**{STATUS_LABEL.get(st, st)}**{prio} · size "
                                                f"{SIZE_TEXT.get(c.get('size'), c.get('size', '-'))}", 'color': STATUS_COLOR.get(st)}]))
    if c.get('effort'):
        people.append(('Effort (actual)', [c['effort']]))
    if c.get('decisions'):
        people.append(("The owner's decisions", bl([f"{n}: {dnames[n]}" if n in dnames else n for n in c['decisions']])))
    if c.get('links'):
        people.append(('Linked items', [nb(', '.join(c['links']))]))
    if c.get('report') or c.get('task'):
        t = []
        if c.get('report'):
            t.append(f"Report: `{os.path.basename(c['report'])}`")
        if c.get('task'):
            t.append(f"Work log: `{c['task']}`")
        people.append(('Report and work log', ['; '.join(t)]))
    if c.get('note'):
        people.append(('Note', [nb(c['note'])]))
    groups.append(('People and records', people))
    for gtitle, rows in groups:
        head = (f'<w:tc><w:tcPr><w:tcW w:w="{sum(widths)}" w:type="dxa"/><w:gridSpan w:val="2"/><w:shd w:val="clear" '
                f'w:color="auto" w:fill="{N.NAVY}"/><w:tcMar><w:top w:w="35" w:type="dxa"/><w:left w:w="100" w:type="dxa"/>'
                '<w:bottom w:w="35" w:type="dxa"/><w:right w:w="100" w:type="dxa"/></w:tcMar></w:tcPr>'
                + d.para(d.run(gtitle.upper(), size=15, color='FFFFFF', bold=True, spacing=20), keep_next=True,
                         keep_lines=True, spacing='<w:spacing w:after="0"/>') + '</w:tc>')
        xml.append(f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{head}</w:tr>')
        for k, paras in rows:
            cells = (d._cell({'paras': [k], 'color': N.NAVY, 'bold': True}, widths[0], N.LILAC, 18, False, 'top', (55, 100))
                     + d._cell({'paras': paras}, widths[1], 'FFFFFF', 18, False, 'top', (55, 100)))
            xml.append(f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{cells}</w:tr>')
    d.add(d._tbl(list(widths), ''.join(xml)))
    d.add(d.SPACER)


def heading1(d, t, page_break=False):
    if page_break:
        d.h1(t)
    else:
        d.add(d.para(d.run(t), style='Heading1', keep_next=True, spacing='<w:spacing w:before="360"/>'))


def build(D, cards, by, version, date, figs):
    d = N.Doc({})
    images = []
    counts = {k: sum(c['status'] == k for c in cards) for k in STATUS_LABEL}
    total = len(cards)
    w, chain = critical_path(cards, by)
    dnames = decision_names(by)

    # cover
    d.add(d.para(d.run('PROJECT DOCUMENT', size=20, color=N.MAGENTA, bold=True, spacing=40),
                 spacing='<w:spacing w:before="200"/>'))
    d.add(d.para(d.run(f'{PNAME}: project task cards', size=56, color=N.NAVY, bold=True), style='Title',
                 spacing='<w:spacing w:after="80" w:line="240" w:lineRule="auto"/>'))
    d.add(d.para(d.run('The task cards of the tailored R&D life cycle', size=32, color=N.PURPLE),
                 spacing='<w:spacing w:after="160"/>'))
    d.add(d.para('', pbdr=f'<w:pBdr><w:bottom w:val="single" w:sz="18" w:space="1" w:color="{N.MAGENTA}"/></w:pBdr>',
                 spacing='<w:spacing w:after="200"/>', ind='<w:ind w:right="6200"/>'))
    d.add(d.para(d.run(f'What each of the {total} task cards is for, when it may start and when it ends (its entry and '
                       'exit gates), what comes before and after it, its inputs and outputs, who leads it, how it is '
                       'verified, its risks and when to escalate. The project is done when every card is done and every '
                       'gate is passed.', size=23, color='3F3F4A', italic=True),
                 spacing='<w:spacing w:after="280" w:line="276" w:lineRule="auto"/>'))
    d.add(N.Doc.inline_image('rId7', 3781425, 2524125, 1, 'shaddad_platform.png', 'Shaddad tracked platform',
                             spacing_after=40))
    d.add(d.para(d.run('Shaddad tracked platform (photo: nasl-tech.com)', size=16, color=N.GREY, italic=True),
                 spacing='<w:spacing w:after="280"/>'))
    d.kv_table([('Document type', 'Project document - task card definitions'), ('Document No.', DOC_NO),
                ('Entity', PENT), ('Version', version), ('Date', date),
                ('Prepared by', 'the lead systems engineer, from the project\'s task cards'),
                ('Reviewer', 'the independent reviewer'), ('Approver', 'the project owner'),
                ('Classification', 'Internal')], widths=(2707, 6319))

    # document control
    d.add(d.para(d.run('Document Control'), style='NASLPageTitle', page_break=True))
    HISTORY = [('1.0', '2026-11-15', 'First issue: every card with its entry and exit gates, predecessors and successors, '
                                      'inputs, outputs, lead, verification, size, risks and escalation'),
               ('1.1', '2026-11-15', 'Re-planned in the engineering management system (NASL-EMS-HBK-001): G0, the success '
                                     'criteria and readiness of every gate, a WBS code on every card, the heritage cards and '
                                     'the new cards'),
               ('1.2', '2026-11-16', 'The owner\'s requests: each card opens with its description (what the task is, the '
                                     'work, how, and what it delivers), and its definition is grouped as start and finish, '
                                     'place in the plan, inputs and outputs, checks and risks, people and records; the team '
                                     'is named by its roles; sizes by the scope of the work'),
               ('1.3', '2026-11-16', 'Gate G0 passed with actions (NASL-SHD-GRR-001): the G0 criteria, RIDs and decision; '
                                     'G0-01 and G0-02 done; the exit criteria that G1-01 takes on from the delta re-review; '
                                     'the reviewer and the approver named by role'),
               ('1.4', '2026-11-16', 'CR-002, approved by the owner ("Approve option 1"): 10 new cards (the safety programme, '
                                     'version control, model credibility, the schedule, the handbook v1.2, the embedded '
                                     'toolchain) and 3 proposed for a later phase; 20 cards changed; 7 new gate criteria; the '
                                     'functional safety engineer'),
               ('1.5', '2026-11-16', 'The owner\'s request on the layout: each card starts on a page of its own under a '
                                     'header band with its id and title, repeated on every page the card runs onto; the '
                                     'purpose in a larger callout; each gate opens with the list of its cards'),
               ('1.6', '2026-11-23', 'G1-01 and G1-03 done (2026-11-18 and 2026-11-21); G1-04 waits for the owner (the '
                                     'supplier request, D96); V-07 proposed for product certification in the Kingdom of '
                                     'Saudi Arabia (D113); G2-FSC takes its inputs from G1-01; T-07 added at the owner\'s '
                                     'request of 2026-11-22 and done on 2026-11-23 (NASL-SHD-SDD-002 and NASL-SHD-ER-007 '
                                     'approved; D117 to D122, with their changes on G2-MCU, G2-PWR, G2-GNC and T-02)')]
    hist = [[v, dt_, 'the lead', {'paras': ['the independent reviewer'], 'color': N.MAGENTA, 'bold': True}, ch]
            for v, dt_, ch in HISTORY if float(v) <= float(version)]
    d.table(['Version', 'Date', 'Prepared by', 'Reviewer', 'Changes'], hist,
            [992, 1353, 1805, 1805, 3071], valign='center')
    d.add(d.para(d.run('Document information', size=24, color=N.NAVY, bold=True),
                 spacing='<w:spacing w:before="240" w:after="80"/>'))
    d.kv_table([('Title', f'{PNAME}: project task cards'), ('Document No.', DOC_NO),
                ('Request', 'The project owner, 2026-11-15: "i want to make pdf that define each task card and the entering and '
                            'exit gates, successor, inputs, outputs and the lead for the task, risks and Escalation also '
                            'you can add anything that will be good fit for a task cards"'),
                ('Scope', f'The {total} task cards of the project, under the gates G1 to G7 of docs/lifecycle.md'),
                ('Source', f"tools/dashboard/project_cards.json (updated {D.get('updated', '')[:16].replace('T', ' ')}); "
                           "the dashboard's Project tab shows the same file"),
                ('Related', 'the project plan NASL-SHD-PMP-001; the review report NASL-SHD-ANR-001; the engineering reports NASL-SHD-ER-001 '
                            'to 004'),
                ('Framework', 'NASL-EMS-HBK-001, the Engineering Handbook: the life cycle, the gates and their success criteria, '
                              'the card fields and states, the approval of a gate'),
                ('File', f'NASL_PLN-001_{PNAME.split()[0]}_Task_Cards_{date}_v{version}.docx / .pdf')])

    d.add(d.para(d.run('Contents'), style='NASLPageTitle', page_break=True))
    d.toc()

    # summary
    d.h1('Summary')
    nxt = [c for c in cards if c.get('priority') in ('now', 'next')]
    passed = [g['id'] for g in D['gates'] if g.get('status') == 'passed']
    d.p(f"**The project has {total} task cards under {len(D['gates'])} gates.** Done: {counts['done']}; in progress: "
        f"{counts['in progress']}; waiting on the owner: {counts['needs you']}; to do: {counts['todo']}. "
        + (f"Gates passed: {', '.join(passed)}." if passed else
           "No gate is passed yet: the first to hold is G1, the system review re-run on the as-built robot."))
    d.p('**Under way and next:**')
    d.bullets([nb(f"**{c['id']}** {c['title']}: {PRIO_LABEL[c['priority']].lower()}; lead "
                  + re.sub(r'\s*\(.*\)$', '', c['lead'])) for c in nxt])
    d.p('**The longest chain of open work** to the end of the project, weighted by size:')
    d.p(nb(' → '.join(chain)) + '.')
    d.p('Every card in section 5 has the same fields: its gate and status, the lead and the support, the purpose, the '
        'entry and exit gates with their criteria, the predecessors and successors, the inputs and outputs, the '
        'verification, the size, the risks with their mitigation, the escalation, and the owner\'s decisions it needs. '
        'All 87 review items sit on a card (Appendix A).')

    # 1 purpose
    heading1(d, '1. Purpose and use of this document', page_break=True)
    d.p('This document defines every task card of the Shaddad project, so that each one can be planned, started, '
        'checked and closed against the same rules. It is the printed form of `tools/dashboard/project_cards.json`; '
        'the dashboard\'s Project tab shows the same cards live.')
    d.bullets(['**To start a card,** check its entry gate: its predecessors done and its entry criteria met.',
               '**To close a card,** check its exit gate: the exit criteria met, the outputs in place, and the definition '
               'of done in section 3.2.',
               '**To change a card,** edit the JSON file and rebuild this PDF with '
               '`python tools/reports/build_task_cards.py`; the successors are derived from the predecessors.'])

    # 2 life cycle
    heading1(d, '2. The life cycle and its gates')
    d.p('The project follows the life cycle of the Engineering Handbook NASL-EMS-HBK-001, tailored in the project plan '
        f'NASL-{PCODE}-PMP-001: G0 for the project start, one combined system review (G1), PDR and CDR per subsystem (G2, '
        'G3), the build, a test readiness review per campaign (G4), then the verification and configuration audits (G5, '
        'G7). PRR (G6) applies only if production starts.')
    images.append(('rId101', 'fig1_gates.png', figs[0]))
    d.figure('rId101', figs[0], 15.5, 'fig1_gates.png', 'The gates and the cards done')
    d.caption('Figure', 'f1', 'The gates of the tailored life cycle, with the cards done under each.')
    d.caption('Table', 't1', 'The gates: the question each answers, what it locks, its status and readiness, and where it stands.')
    rows = []
    for g in D['gates']:
        ok, m = readiness(g)
        rows.append([nb(GATE_LABEL[g['id']]), g['question'], g['locks'],
                     {'paras': [f"{GATE_STATUS.get(g.get('status', 'open'), g.get('status'))}; readiness {ok} / {m}"],
                      'bold': True, 'color': N.NAVY if g.get('status') in ('passed', 'passed with actions') else N.MAGENTA
                      if g.get('status') == 'in review' else N.GREY},
                     nb(g.get('assessment', ''))])
    d.table(['Gate', 'Question', 'Locks', 'Status', 'Where it stands'], rows,
            [1100, 2350, 1600, 1250, 2726], size=17, keep_all=False)
    d.h2('2.1 Gate success criteria and readiness')
    d.p('Every gate has success criteria (NASL-EMS-HBK-001 section 4.4): **M** mandatory, met or waived by the owner with '
        'a reason; **D** desirable. The lead scores each one (met, partly, not met, waived, n/a, or not assessed while the '
        'gate is far ahead); the **readiness** is the share of mandatory criteria met or waived. A gate is approved by '
        'the owner after the readiness check and the independent assessment: go, go with actions, delta review or no-go '
        '(handbook section 4.5).')
    for g in D['gates']:
        crit = g.get('criteria', [])
        if not crit:
            continue
        ok, m = readiness(g)
        d.caption('Table', f"tc{g['id']}", f"{GATE_LABEL[g['id']]}: success criteria, scored on "
                                         f"{D.get('updated', '')[:10]} (readiness {ok} of {m} mandatory).")
        d.table(['ID', 'Criterion', 'Class', 'Score', 'Evidence or note'],
                [[nb(c['id']), c['criterion'], c.get('class', ''),
                  {'paras': [c.get('score', 'not assessed')], 'bold': True, 'color': SCORE_COLOR.get(c.get('score'), N.GREY)},
                  c.get('note') or c.get('evidence', '')] for c in crit],
                [1050, 3500, 650, 1150, 2676], size=16, keep_all=False)

    # 3 how a card works
    heading1(d, '3. How a task card works')
    d.h2('3.1 The fields of a card')
    d.caption('Table', 't2', 'The fields of a task card.')
    d.table(['Field', 'Meaning'], [
        ['ID and title', 'The card\'s name. The prefix gives the gate: G0 to G3 cards, B for Build, T for G4, V for G5 and '
                         'P for G7; H marks a heritage card (work done before the framework, closed with its records). An id '
                         'never changes: T-01 to T-03 kept theirs when CR-002 moved them to G1.'],
        ['Gate and WBS', 'The life-cycle gate the card serves, and its element of the work breakdown structure '
                         '(1 project management, 2 systems engineering, 3.n a subsystem, 4 integration and V&V, 5 build '
                         'and release, 6 tools).'],
        ['Status', 'To do, ready, in progress, in review, needs you (waits on the owner), blocked, or done.'],
        ['Priority', 'Now (in progress), next (to start next), later, or done.'],
        ['Size', 'S, M, L or XL, by the scope of the work (section 3.5).'],
        ['Lead', 'The one accountable for the card: a subsystem engineer, a specialist, the lead systems engineer or the '
                 'project owner.'],
        ['Support', 'Who else works on it.'],
        ['Purpose', 'Why the card exists, in one sentence.'],
        ['Description', 'What the task is, the work it involves, how it is done and what it delivers; for a done card, '
                        'what was done and its outcome.'],
        ['Entry gate', 'When the card may start: the gate or cards before it, then the entry criteria.'],
        ['Exit gate', 'Where the card ends: the gate review or the card it feeds, then the exit criteria (its acceptance).'],
        ['Predecessors', 'The cards that must be done first.'],
        ['Successors', 'The cards that wait for this one (derived from their predecessors).'],
        ['Inputs', 'What the card needs: data, documents, decisions, review items.'],
        ['Outputs', 'What the card delivers.'],
        ['Verification', 'How the exit criteria are checked: Test, Analysis, Inspection or Demonstration.'],
        ['Risks', 'What could go wrong, with its likelihood, impact and mitigation (section 3.5).'],
        ['Escalation', 'When the work goes up a level, and to whom (section 3.4).'],
        ['The owner\'s decisions', 'The numbered decisions of the review that the card needs.'],
        ['Linked items', 'Review items (R-xx), STATUS items and documents.'],
        ['Report and work log', 'The PDF report and the work log of a card that ran.'],
    ], [2200, 6826], size=18)

    d.h2('3.2 Status, priority and the definition of done')
    d.p('A card moves from to do to in progress when its entry gate is met and the lead dispatches it. It shows needs you '
        'while it waits on a decision, data or an action of the owner. **A card is done when:**')
    d.numbered(['its exit criteria are met and checked by the stated verification;',
                'its outputs exist where the card says;',
                'the changes are under configuration control: snapshots, a decision row for each decision, and the test '
                'baseline updated;',
                'the docs keeper has updated the docs;',
                'for a task group, its PDF engineering report is issued (the owner\'s rule of 2026-11-15);',
                'its status, report and task are updated in `tools/dashboard/project_cards.json`.'])
    d.p('**A gate is passed** when every card under it is done and its review (G1-05, the PDR and CDR cards, T-05, V-04 '
        'or P-03) records the owner\'s decision: go, go with actions, a delta review or no-go.')

    d.h2('3.3 Roles')
    d.caption('Table', 't3', 'Who does what on the cards.')
    d.table(['Role', 'Who', 'On the cards'], [
        ['Project owner', 'the approval authority', 'Decides the open decisions, the requirements, the budget, the safety '
                                                   'policy and the hardware; chairs the change board; approves the gates; '
                                                   'runs anything on the real robot; signs the baselines.'],
        ['Lead systems engineer', 'one, for the project', 'Plans and assigns the cards, chairs the gate reviews, keeps the '
                                                          'cards, the registers and the dashboard, writes the task reports, '
                                                          'and puts each decision to the owner.'],
        ['Subsystem engineers', 'structure; power and BMS; drive; MCU firmware; electronics and comms; sensors and '
                                'navigation; GNC', 'Lead the cards of their subsystem; change only their own parts, one '
                                                  'pass at a time, with a step report.'],
        ['Specialists', 'integration engineer; test engineer; independent reviewer; functional safety engineer; '
                        'configuration and documentation engineer; data analyst; technical writer',
         'The system and the top level; running and sorting the tests; the independent check before a change is kept; '
         'the safety life cycle, assessed independently of the designs (CR-002); the documents and version control; the '
         'logs; the formal reports.'],
    ], [1700, 2900, 4426], size=18)

    d.h2('3.4 Escalation')
    d.p('Each card lists its own escalation triggers. They all follow this ladder:')
    d.caption('Table', 't4', 'The escalation ladder.')
    d.table(['Level', 'To', 'When', 'How'], [
        ['1', 'the card\'s lead', 'A check fails for a reason inside the lead\'s own files.', 'Fix it in the same pass.'],
        ['2', 'the lead systems engineer', 'The work needs another owner\'s part, an interface change (bus, enum, ICD) or a '
                                           'test\'s expected value; a check fails twice; an engineer is blocked; the effort '
                                           'passes the estimate.', 'At once, in the step report, as a question or a ruling '
                                                                   'request.'],
        ['3', 'the project owner', 'A decision that changes a requirement, the safety policy, the hardware or a supplier; '
                                   'cost beyond the approved budget; any action on the real robot; security settings; '
                                   'anything sent or published.', 'A question with the options and a recommendation; the '
                                                                  'answer is recorded as a decision row.'],
    ], [700, 1700, 4026, 2600], size=18)
    d.note('A safety finding goes to the lead systems engineer at once and to the owner the same day, whatever the card.')

    d.h2('3.5 Risk and size scales')
    d.caption('Table', 't5', 'The scales used on the cards.')
    d.table(['Scale', 'Low (L)', 'Medium (M)', 'High (H)'], [
        ['Likelihood', 'Unlikely', 'Possible', 'Likely, or already seen'],
        ['Impact', 'Cost or a short delay only', 'A result or a gate slips', 'Safety, the real robot or a baseline'],
    ], [1500, 2300, 2400, 2826], size=18)
    d.p('**Size,** by the scope of the work: **S** one engineer, one pass; **M** two or three engineers, or two passes; '
        '**L** several engineers and system runs; **XL** a review of every subsystem, or a change across several safety '
        'chains. For scale, the finished task groups took: the MATLAB Function conversion 5 engineering passes (L), the '
        'critical fixes 11 (XL), the review 12 (XL) and the S4 fixes 13 (L, with long system runs). The sizes of the '
        'open cards are estimates [EST]; the effort of a done card is recorded with it.')

    # 4 card map
    heading1(d, '4. The card map', page_break=True)
    d.p('Figure 2 shows every card under its gate, coloured by status, with an arrow from each predecessor to its '
        'successor. Table 6 lists them with their lead, status, priority and size.')
    images.append(('rId102', 'fig2_network.png', figs[1]))
    with open(figs[1], 'rb') as f:
        head = f.read(24)
    pw, ph = int.from_bytes(head[16:20], 'big'), int.from_bytes(head[20:24], 'big')
    d.figure('rId102', figs[1], min(16.0, 19.0 * pw / ph), 'fig2_network.png', 'The card map')
    d.caption('Figure', 'f2', 'The card map: each card under its gate, coloured by status; an arrow runs from a '
                              'predecessor to its successor.')
    d.p(f'**The longest chain of open work** (weight {w}, with S 1, M 2, L 3 and XL 4): ' + nb(' → '.join(chain)) + '.')
    d.caption('Table', 't6', 'All task cards.')
    rows = []
    for c in cards:
        rows.append([nb(c['id']), nb(c['title']), c.get('lead', '-'),
                     {'paras': [STATUS_LABEL.get(c['status'], c['status'])], 'color': STATUS_COLOR.get(c['status']),
                      'bold': True},
                     PRIO_LABEL.get(c.get('priority'), '-'), c.get('size', '-'),
                     nb(', '.join(c.get('predecessors', [])) or '-'), nb(', '.join(c['successors']) or '-')])
    d.table(['ID', 'Title', 'Lead', 'Status', 'Priority', 'Size', 'Predecessors', 'Successors'], rows,
            [820, 2250, 1500, 900, 760, 500, 1150, 1146], size=15, keep_all=False)

    # 5 the cards
    heading1(d, '5. The task cards', page_break=True)
    d.p('The cards by gate. Each gate opens with its question, where it stands and the list of its cards. **Each card then '
        'starts on a page of its own,** under a header band with its id and title; the band repeats at the top of every page '
        'the card runs onto. Under it come a line at a glance (gate, WBS, status, priority, size, lead), the purpose, and the '
        'description: what the task is, the work it involves, how it is done and what it delivers. The definition follows '
        'in five groups: start and finish (the entry and exit gates), the place in the plan (predecessors and successors), '
        'inputs and outputs, checks, risks and escalation, and people and records. Section 3.1 defines the fields.')
    for gi, g in enumerate(D['gates']):
        cs = [c for c in cards if c['gate'] == g['id']]
        title = f"5.{gi + 1} {GATE_LABEL[g['id']]}: {g['title'].lower() if g['id'] != 'BUILD' else 'fabricate, code, integrate'}"
        while d.parts and d.parts[-1] == d.SPACER:
            d.parts.pop()
        d.add(d.para(d.run(title), style='Heading2', page_break=gi > 0))
        locks = f" **Locks:** {g['locks']}." if g.get('locks', '-') != '-' else ''
        d.p(f"**Question:** {g['question']}{locks} **Where it stands:** {nb(g.get('assessment', ''))}")
        if cs:
            d.caption('Table', f"tg{g['id']}", f"The cards of {GATE_LABEL[g['id']]}, each on the pages that follow.")
            d.table(['Card', 'Title', 'Status', 'Size', 'Lead'],
                    [[{'paras': [c['id']], 'bold': True, 'color': N.NAVY}, nb(c['title']),
                      {'paras': [STATUS_LABEL.get(c['status'], c['status'])], 'color': STATUS_COLOR.get(c['status'])},
                      c.get('size', '-'), re.sub(r' *[(].*', '', c.get('lead', '-'))] for c in cs],
                    [950, 3850, 1150, 600, 2476], size=17, keep_all=False)
        for c in cs:
            # a new page for every card: a 1 pt paragraph that breaks the page, then the card's table
            while d.parts and d.parts[-1] == d.SPACER:
                d.parts.pop()
            d.add(d.para('', page_break=True,
                         spacing='<w:spacing w:before="0" w:after="0" w:line="20" w:lineRule="exact"/>'))
            card_table(d, c, by, dnames)

    # appendices
    heading1(d, 'Appendix A. The review items by card', page_break=True)
    d.p('Each of the 87 improvement areas of the review (the ranked improvement list of NASL-SHD-ANR-001) and the card that carries it. '
        'An item on more than one card is listed with each.')
    where = defaultdict(list)
    for c in cards:
        for l in c.get('links', []) + c.get('inputs', []):
            for r in re.findall(r'R-(\d\d)(?: to R-(\d\d))?', l):
                a = int(r[0])
                b = int(r[1]) if r[1] else a
                for k in range(a, b + 1):
                    if c['id'] not in where[k]:
                        where[k].append(c['id'])
    items = [(k, ', '.join(where.get(k, ['-']))) for k in range(1, 88)]
    per = (len(items) + 2) // 3
    cols3 = [items[i * per:(i + 1) * per] for i in range(3)]
    rowsA = []
    for i in range(per):
        r = []
        for col in cols3:
            if i < len(col):
                r += [nb(f'R-{col[i][0]:02d}'), nb(col[i][1])]
            else:
                r += ['', '']
        rowsA.append(r)
    d.caption('Table', 'tA', 'Review items and their cards.')
    d.table(['Item', 'Card(s)'] * 3, rowsA, [800, 2208, 800, 2208, 800, 2210], size=16, keep_all=False)
    missing = [k for k in range(1, 88) if k not in where]
    if missing:
        d.note('Not on a card: ' + ', '.join(f'R-{k:02d}' for k in missing))

    heading1(d, 'Appendix B. The owner\'s open decisions by card')
    dcards = defaultdict(list)
    for c in cards:
        for n in c.get('decisions', []):
            dcards[n].append(c['id'])
    d.caption('Table', 'tB', 'The open decisions of the review, and the cards that need them.')
    crs = {}
    cpath = os.path.join(ROOT, 'docs', 'registers', 'changes.json')
    if os.path.exists(cpath):
        crs = {c['id']: c['title'] for c in json.load(open(cpath, encoding='utf-8')).get('change_requests', [])}
    d.table(['Decision', 'Topic', 'Needed by'],
            [[nb(n), dnames.get(n, crs.get(n, '-')), nb(', '.join(dcards[n]))]
             for n in sorted(dcards, key=lambda x: (not x.isdigit(), int(x) if x.isdigit() else 0, x))],
            [1100, 4200, 3726], size=18, keep_all=False)
    d.p('Settled so far: decision 1 (brakes first, STO as backup: D71), 3 (implement off for the rest of a hold: D72) '
        'and 10 (the sprayer is mounted on the deck).')

    heading1(d, 'References')
    d.bullets(['`tools/dashboard/project_cards.json`: the task cards (the source of this document).',
               '`docs/lifecycle.md`: the tailored life cycle and the gate procedure.',
               'NASL-SHD-ANR-001, the review report: the 87 improvement areas and the 20 decisions.',
               'NASL-SHD-ER-001 to 004: the engineering reports of the finished task groups (`work/reports/engineering/`).',
               'The team playbook and the reports guide: how the team works and reports.'])
    return d, images


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-pdf', action='store_true')
    ap.add_argument('--version', default='1.0')
    ap.add_argument('--date', default=dt.date.today().isoformat())
    a = ap.parse_args()
    D, cards, by = load()
    os.makedirs(FIGDIR, exist_ok=True)
    E.FIGDIR = FIGDIR
    svg1, svg2 = os.path.join(FIGDIR, 'task_cards_gates.svg'), os.path.join(FIGDIR, 'task_cards_network.svg')
    fig_gates(D, cards, svg1)
    fig_network(D, cards, by, svg2)
    figs = [E.to_png(svg1), E.to_png(svg2)]
    assert all(figs), figs
    d, images = build(D, cards, by, a.version, a.date, figs)
    name = f'NASL_PLN-001_{PNAME.split()[0]}_Task_Cards_{a.date}_v{a.version}'
    docx = os.path.join(OUTDIR, name + '.docx')
    meta = {'title': f'{PNAME}: project task cards', 'date': a.date,
            'footer': f'NASL Technologies | {PNAME} Project Task Cards | Internal',
            'subject': 'The task cards of the tailored R&D life cycle',
            'keywords': 'Shaddad, UGV, task cards, life cycle, gates, NASL, Robotat',
            'description': f'NASL Technologies (Robotat) - Internal - {DOC_NO} - Version {a.version} - {a.date}'}
    nbody = N.package(d, docx, meta, images)
    print(f"written {docx}: tables {d.n['Table']}, figures {d.n['Figure']}, body {nbody / 1e3:.0f} kB")
    if not a.no_pdf:
        out = os.path.join(OUTDIR, name + '.pdf')
        r = subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', E.PS1,
                            '-Docx', docx, '-Pdf', out], capture_output=True, text=True, timeout=900)
        print((r.stdout or '').strip() or (r.stderr or '').strip()[-800:])


if __name__ == '__main__':
    main()
