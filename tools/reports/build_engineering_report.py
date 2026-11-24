"""Build the NASL PDF engineering report of an agent task group (the user's standing rule of 2026-11-15).

usage:
  python tools/reports/build_engineering_report.py <id> [--no-pdf]
  python tools/reports/build_engineering_report.py --all [--no-pdf]

Inputs:
  - work/agentic/reports/<id>.md: the agent task report (ATR) that tools/dashboard/task_report.py writes from the
    lead's engineering intro (work/agentic/reports/src/<id>.md), the Agents log and the journal. Run
    task_report.py first (or bash work/agentic/reports/src/regenerate.sh).
  - tools/reports/engineering_reports.json: the report's entry (document number, title, subtitle, cover summary,
    version, date, history, request, scope).
Output: work/reports/engineering/<file>.docx and <file>.pdf; the dashboard lists the PDFs under "Engineering
reports". Figures: PNG as they are; SVG (the ATR's run timeline and effort charts) rendered to PNG with Edge
headless at 2x and cached in work/reports/engineering/fig/.
The NASL look (template, styles, header and footer, page background) comes from tools/reports/nasl_docx.py.
No installs: python standard library, plus Microsoft Word (COM) for the PDF and Edge for SVG figures.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import nasl_docx as N  # noqa: E402

CONFIG = os.path.join(HERE, 'engineering_reports.json')
OUTDIR = os.path.join(ROOT, 'work', 'reports', 'engineering')
FIGDIR = os.path.join(OUTDIR, 'fig')
EDGE = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
PS1 = os.path.join(HERE, 'convert_to_pdf.ps1')


# ------------------------------------------------------------------ markdown -> blocks
def strip_links(s):
    s = re.sub(r'!\[([^\]]*)\]\([^)]*\)', r'\1', s)
    return re.sub(r'\[([^\]]+)\]\(([^)]+)\)',
                  lambda m: m.group(1) if not m.group(2).startswith('http') else f'{m.group(1)} ({m.group(2)})', s)


def clean(s):
    s = strip_links(s.strip())
    s = s.replace('&lt;', '<').replace('&gt;', '>').replace('&amp;', '&')
    s = re.sub(r'(?<!\*)\*(?!\*)([^*\n]+?)(?<!\*)\*(?!\*)', r'\1', s)   # *italic* -> plain
    s = re.sub(r'(?<![\w-])([A-Z]{1,4})-(?=\d)', '\\1' + chr(0x2011), s)  # R-04: no line break at the hyphen
    return s


def split_row(line):
    """The cells of a markdown table row: split only on unescaped '|', then unescape '\\|'."""
    s = line.strip()
    if s.startswith('|'):
        s = s[1:]
    if s.endswith('|') and not s.endswith('\\|'):
        s = s[:-1]
    return [c.strip().replace('\\|', '|') for c in re.split(r'(?<!\\)\|', s)]


def parse(text):
    lines = text.replace('\r\n', '\n').split('\n')
    out, i, n = [], 0, len(lines)
    item_re = re.compile(r'^(\s*)([-*+]|\d+[.)])\s+(.*)$')
    while i < n:
        s = lines[i].rstrip()
        if not s.strip() or re.match(r'^\s*(---|\*\*\*|___)\s*$', s) or s.strip().startswith('<!--'):
            if s.strip().startswith('<!--') and '-->' not in s:
                while i < n and '-->' not in lines[i]:
                    i += 1
            i += 1
            continue
        if s.startswith('```'):
            j, buf = i + 1, []
            while j < n and not lines[j].startswith('```'):
                buf.append(lines[j].rstrip())
                j += 1
            out.append(('code', '\n'.join(buf)))
            i = j + 1
            continue
        m = re.match(r'^(#{1,4})\s+(.*)$', s)
        if m:
            out.append(('h', len(m.group(1)), clean(m.group(2))))
            i += 1
            continue
        if s.lstrip().startswith('|'):
            rows = []
            while i < n and lines[i].lstrip().startswith('|'):
                cells = split_row(lines[i])
                if not all(re.match(r'^:?-{2,}:?$', c) for c in cells if c):
                    rows.append([clean(c) for c in cells])
                i += 1
            out.append(('table', rows))
            continue
        m = re.match(r'^!\[(.*?)\]\((.*?)\)\s*$', s.strip())
        if m:
            out.append(('fig', m.group(1), m.group(2)))
            i += 1
            continue
        m = re.match(r'^(Table|Figure) ([A-Z]?\d+)[.:]\s+(.*)$', s.strip())
        if m and m.group(1) == 'Table':
            out.append(('tcap', clean(m.group(3))))
            i += 1
            continue
        if s.startswith('>'):
            buf = []
            while i < n and lines[i].startswith('>'):
                buf.append(lines[i].lstrip('>').strip())
                i += 1
            out.append(('quote', clean(' '.join(buf))))
            continue
        if item_re.match(s):
            items = []
            while i < n:
                t = lines[i].rstrip()
                m = item_re.match(t)
                if m:
                    ind = len(m.group(1).replace('\t', '    '))
                    items.append([ind, m.group(2)[0].isdigit(), m.group(3)])
                    i += 1
                elif t.strip() and items and (len(t) - len(t.lstrip())) > 0 and not t.lstrip().startswith('|'):
                    items[-1][2] += ' ' + t.strip()          # continuation line of the last item
                    i += 1
                else:
                    break
            levels = sorted({it[0] for it in items})
            out.append(('list', [(levels.index(it[0]), it[1], clean(it[2])) for it in items]))
            continue
        buf = [s.strip()]
        i += 1
        while i < n:
            t = lines[i].rstrip()
            if (not t.strip() or t.startswith(('#', '|', '```', '>', '![')) or item_re.match(t)
                    or re.match(r'^(Table|Figure) ([A-Z]?\d+)[.:]\s', t.strip())):
                break
            buf.append(t.strip())
            i += 1
        out.append(('p', clean(' '.join(buf))))
    return out


# ------------------------------------------------------------------ figures
def to_png(path):
    full = path if os.path.isabs(path) else os.path.join(ROOT, path)
    if not os.path.exists(full):
        return None
    if full.lower().endswith('.png'):
        return full
    if full.lower().endswith('.svg'):
        os.makedirs(FIGDIR, exist_ok=True)
        png = os.path.join(FIGDIR, os.path.splitext(os.path.basename(full))[0] + '.png')
        if not os.path.exists(png) or os.path.getmtime(png) < os.path.getmtime(full):
            head = open(full, encoding='utf-8').read(400)
            w = int(float(re.search(r"width=['\"]([\d.]+)", head).group(1)))
            h = int(float(re.search(r"height=['\"]([\d.]+)", head).group(1)))
            url = 'file:///' + full.replace('\\', '/')
            if os.path.exists(png):
                os.remove(png)
            subprocess.run([EDGE, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                            '--force-device-scale-factor=2', f'--window-size={w},{h}',
                            f'--screenshot={png}', url], capture_output=True, timeout=120)
            # Edge's launcher returns before its renderer has written the file: wait for a stable file
            last = -1
            for _ in range(60):
                time.sleep(0.5)
                size = os.path.getsize(png) if os.path.exists(png) else -1
                if size > 0 and size == last:
                    break
                last = size
        return png if os.path.exists(png) else None
    return None


# ------------------------------------------------------------------ the document
def col_widths(rows):
    """Column widths in dxa: each column gets at least its longest word (no broken words), the rest of the
    text width is shared in proportion to the column's content length."""
    k = max(len(r) for r in rows)
    L, W = [6] * k, [4] * k
    for r in rows:
        for j, c in enumerate(r):
            L[j] = max(L[j], min(len(c), 70))
            W[j] = max(W[j], min(max((len(t) for t in c.split()), default=0), 24))
    mins = [(w + 2) * 105 for w in W]
    if sum(mins) >= N.TEXT_W:
        w = [int(N.TEXT_W * m / sum(mins)) for m in mins]
    else:
        free = N.TEXT_W - sum(mins)
        w = [m + int(free * l / sum(L)) for m, l in zip(mins, L)]
    w[-1] += N.TEXT_W - sum(w)
    return w


def emit_list(d, items):
    num = None
    for lvl, ordered, text in items:
        if ordered and lvl == 0:
            if num is None:
                num = d.next_num
                d.next_num += 1
                d.nums.append(num)
            d.add(d.para(d.inline(text), style='ListParagraph', num=(num, 0),
                         spacing='<w:spacing w:after="70"/>'))
        else:
            d.add(d.para(d.inline(text), style='ListParagraph', num=(2, min(lvl, 3)),
                         spacing='<w:spacing w:after="70"/>'))


def build(R, md_text):
    blocks = parse(md_text)
    # the ATR header table (right after the "# title") -> document information
    info = []
    if blocks and blocks[0][0] == 'h' and blocks[0][1] == 1:
        blocks = blocks[1:]
    if blocks and blocks[0][0] == 'table':
        info = [(r[0], r[1]) for r in blocks[0][1] if len(r) >= 2 and r[0]]
        blocks = blocks[1:]

    d = N.Doc({})
    images, rid_n = [], 101

    # cover
    d.add(d.para(d.run('ENGINEERING REPORT', size=20, color=N.MAGENTA, bold=True, spacing=40),
                 spacing='<w:spacing w:before="200"/>'))
    d.add(d.para(d.run(re.sub(r'(?<=\w)-(?=\w)', '‑', R['title']), size=56, color=N.NAVY, bold=True), style='Title',
                 spacing='<w:spacing w:after="80" w:line="240" w:lineRule="auto"/>'))
    d.add(d.para(d.run(R['subtitle'], size=32, color=N.PURPLE), spacing='<w:spacing w:after="160"/>'))
    d.add(d.para('', pbdr=f'<w:pBdr><w:bottom w:val="single" w:sz="18" w:space="1" w:color="{N.MAGENTA}"/></w:pBdr>',
                 spacing='<w:spacing w:after="200"/>', ind='<w:ind w:right="6200"/>'))
    d.add(d.para(d.run(R['summary'], size=23, color='3F3F4A', italic=True),
                 spacing='<w:spacing w:after="280" w:line="276" w:lineRule="auto"/>'))
    d.add(N.Doc.inline_image('rId7', 3781425, 2524125, 1, 'shaddad_platform.png', 'Shaddad tracked platform',
                             spacing_after=40))
    d.add(d.para(d.run('Shaddad tracked platform (photo: nasl-tech.com)', size=16, color=N.GREY, italic=True),
                 spacing='<w:spacing w:after="280"/>'))
    d.kv_table([('Report type', 'Engineering report - task group'),
                ('Document No.', R['doc_no']),
                ('Entity', 'NASL Technologies (Robotat)'),
                ('Version', R['version']),
                ('Date', R['date']),
                ('Prepared by', 'the Shaddad engineering team and the lead systems engineer'),
                ('Reviewer', 'the independent reviewer'),
                ('Approver', 'the project owner'),
                ('Classification', 'Internal')], widths=(2707, 6319))

    # document control
    d.add(d.para(d.run('Document Control'), style='NASLPageTitle', page_break=True))
    d.table(['Version', 'Date', 'Prepared by', 'Reviewer', 'Approver', 'Changes'],
            [[v, dt, 'the Shaddad engineering team',
              {'paras': ['the independent reviewer'], 'color': N.MAGENTA, 'bold': True},
              {'paras': ['the project owner'], 'color': N.MAGENTA, 'bold': True}, ch]
             for v, dt, ch in R['history']], [992, 1353, 1600, 1500, 1400, 2181], valign='center')
    d.add(d.para(d.run('Document information', size=24, color=N.NAVY, bold=True),
                 spacing='<w:spacing w:before="240" w:after="80"/>'))
    rows = [('Title', R['title']), ('Document No.', R['doc_no'])]
    rows += [(k, v) for k, v in info if k not in ('Report',)]
    rows += [('Request', R['request']), ('Scope', R['scope']), ('File', R['file'] + '.docx / .pdf'),
             ('Source', f"work/team/reports/{R['id']}.md (ATR-{R['id']}) and the step reports in its references")]
    d.kv_table(rows)

    # contents
    d.add(d.para(d.run('Contents'), style='NASLPageTitle', page_break=True))
    d.toc()

    # body
    pending_cap = None
    for b in blocks:
        kind = b[0]
        if kind == 'h':
            lvl, t = b[1], b[2]
            if lvl <= 2:
                if t == 'Summary' or t.startswith(('Appendix', 'References')):
                    d.h1(t)                      # these start a new page
                else:
                    d.add(d.para(d.run(t), style='Heading1', keep_next=True,
                                 spacing='<w:spacing w:before="360"/>'))
            elif lvl == 3:
                d.h2(t)
            else:
                d.h3(t)
        elif kind == 'p':
            d.p(b[1])
        elif kind == 'list':
            emit_list(d, b[1])
        elif kind == 'tcap':
            pending_cap = b[1]
        elif kind == 'table':
            rows = b[1]
            if not rows:
                continue
            if pending_cap:
                d.caption('Table', f't{d.n["Table"] + 1}', pending_cap)
                pending_cap = None
            k = max(len(r) for r in rows)
            rows = [r + [''] * (k - len(r)) for r in rows]
            d.table(rows[0], rows[1:], col_widths(rows), size=18 if k >= 5 else 19)
        elif kind == 'fig':
            cap, path = b[1], b[2]
            png = to_png(path)
            if not png:
                d.note(f'Figure not available: `{path}`')
                continue
            rid = f'rId{rid_n}'
            rid_n += 1
            media = f'fig{rid_n - 101}_' + re.sub(r'[^\w.]', '_', os.path.basename(png))
            images.append((rid, media, png))
            with open(png, 'rb') as f:
                head = f.read(24)
            pw, ph = int.from_bytes(head[16:20], 'big'), int.from_bytes(head[20:24], 'big')
            width_cm = min(15.5, 16.0 * pw / ph) if ph else 15.5    # at most 16 cm tall, so a caption fits
            d.figure(rid, png, width_cm, media, cap)
            d.caption('Figure', f'f{d.n["Figure"] + 1}', re.sub(r'^Figure [A-Z]?\d+[.:]\s*', '', cap))
        elif kind == 'code':
            d.code(b[1])
        elif kind == 'quote':
            d.note(b[1])
    return d, images


def make(R, pdf=True):
    src = os.path.join(ROOT, 'work', 'team', 'reports', R['id'] + '.md')
    md = open(src, encoding='utf-8').read()
    d, images = build(R, md)
    os.makedirs(OUTDIR, exist_ok=True)
    docx = os.path.join(OUTDIR, R['file'] + '.docx')
    meta = {'title': R['title'], 'date': R['date'],
            'footer': f"NASL Technologies | {R['footer']} | Internal",
            'subject': R['subtitle'], 'keywords': 'Shaddad, UGV, engineering report, task group, NASL, Robotat',
            'description': f"NASL Technologies (Robotat) - Internal - {R['doc_no']} - Version {R['version']} - {R['date']}"}
    nbody = N.package(d, docx, meta, images)
    print(f"written {docx}: tables {d.n['Table']}, figures {d.n['Figure']}, body {nbody / 1e3:.0f} kB")
    if pdf:
        out = os.path.join(OUTDIR, R['file'] + '.pdf')
        r = subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', PS1,
                            '-Docx', docx, '-Pdf', out], capture_output=True, text=True, timeout=900)
        print((r.stdout or '').strip() or (r.stderr or '').strip()[-800:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('id', nargs='?')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--no-pdf', action='store_true')
    a = ap.parse_args()
    cfg = json.load(open(CONFIG, encoding='utf-8'))['reports']
    todo = cfg if a.all else [r for r in cfg if r['id'] == a.id]
    if not todo:
        sys.exit(f'no entry for {a.id!r} in {CONFIG}')
    for R in todo:
        make(R, pdf=not a.no_pdf)


if __name__ == '__main__':
    main()
