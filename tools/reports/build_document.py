"""Build any controlled document (a handbook, a plan, a procedure, a register report) as a NASL Word file and PDF.

The source is a markdown file whose first lines are a document header in an HTML comment (invisible in a markdown
viewer), for example:

    <!--
    doc_no: NASL-EMS-HBK-001
    doc_type: ENGINEERING HANDBOOK
    title: Engineering Handbook
    subtitle: How we plan, build, verify and document engineering projects
    summary: One paragraph for the cover.
    version: 1.0
    date: 2026-11-16
    status: Issued for approval
    history: 1.0 | 2026-11-16 | First issue
    prepared_by: the lead (the main Claude session)
    reviewer: [TBD]
    approver: [TBD]
    entity: NASL Technologies
    classification: Internal
    footer: Engineering Handbook
    file: NASL_EMS-HBK-001_Engineering_Handbook_v1.0
    outdir: framework/build
    chapter_breaks: yes
    cover_image: none
    -->

`history` may repeat (one line per version: version | date | changes). `cover_image: shaddad` puts the Shaddad photo
on the cover. `chapter_breaks: yes` starts every `## ` chapter on a new page.

The body: `## ` chapters, `### ` sections, `#### ` subsections; tables with a "Table N. caption" line above them;
figures as ![Figure N: caption](path.svg or .png) (SVG is rendered by Edge headless at 2x); lists, quotes and code.

usage:
  python tools/reports/build_document.py <source.md> [--no-pdf]
  python tools/reports/build_document.py --index [<folder> ...]     list the controlled documents found
No installs: python standard library, Microsoft Word (COM) for the PDF and Edge for SVG figures.
"""
import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import nasl_docx as N  # noqa: E402
import build_engineering_report as E  # noqa: E402

KEYS = ('doc_no', 'doc_type', 'title', 'subtitle', 'summary', 'version', 'date', 'status', 'prepared_by', 'reviewer',
        'approver', 'entity', 'classification', 'footer', 'file', 'outdir', 'chapter_breaks', 'cover_image', 'request',
        'scope', 'related')


def read_header(text):
    """The document header: key: value lines in the first HTML comment."""
    m = re.match(r'\s*<!--(.*?)-->', text, re.S)
    if not m:
        return None
    meta = {'history': []}
    for line in m.group(1).splitlines():
        k, sep, v = line.strip().partition(':')
        if not sep:
            continue
        k, v = k.strip(), v.strip()
        if k == 'history':
            meta['history'].append([p.strip() for p in v.split('|')])
        elif k in KEYS:
            meta[k] = v
    return meta if meta.get('doc_no') else None


def render_body(d, blocks, images, chapter_breaks):
    pending_cap, first = None, True
    for b in blocks:
        kind = b[0]
        if kind == 'h':
            lvl, t = b[1], b[2]
            if lvl == 1:
                continue                                  # the "# title" line: the cover has it
            if lvl == 2:
                if (chapter_breaks and not first) or t == 'Summary' or t.startswith(('Appendix', 'References')):
                    d.h1(t)
                else:
                    d.add(d.para(d.run(t), style='Heading1', keep_next=True, spacing='<w:spacing w:before="360"/>'))
                first = False
            elif lvl == 3:
                d.h2(t)
            else:
                d.h3(t)
        elif kind == 'p':
            d.p(b[1])
        elif kind == 'list':
            E.emit_list(d, b[1])
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
            d.table(rows[0], rows[1:], E.col_widths(rows), size=17 if k >= 5 else 18, keep_all=len(rows) <= 6)
        elif kind == 'fig':
            cap, path = b[1], b[2]
            png = E.to_png(path)
            if not png:
                d.note(f'Figure not available: `{path}`')
                continue
            rid = f'rId{101 + len(images)}'
            media = f'fig{len(images) + 1}_' + re.sub(r'[^\w.]', '_', os.path.basename(png))
            images.append((rid, media, png))
            with open(png, 'rb') as f:
                head = f.read(24)
            pw, ph = int.from_bytes(head[16:20], 'big'), int.from_bytes(head[20:24], 'big')
            width_cm = min(15.5, 16.0 * pw / ph) if ph else 15.5
            d.figure(rid, png, width_cm, media, cap)
            d.caption('Figure', f'f{d.n["Figure"] + 1}', re.sub(r'^Figure [A-Z]?\d+[.:]\s*', '', cap))
        elif kind == 'code':
            d.code(b[1])
        elif kind == 'quote':
            d.note(b[1])


def build(meta, body_md, src_rel):
    d = N.Doc({})
    images = []
    # cover
    d.add(d.para(d.run(meta.get('doc_type', 'DOCUMENT').upper(), size=20, color=N.MAGENTA, bold=True, spacing=40),
                 spacing='<w:spacing w:before="200"/>'))
    d.add(d.para(d.run(re.sub(r'(?<=\w)-(?=\w)', '‑', meta['title']), size=56, color=N.NAVY, bold=True),
                 style='Title', spacing='<w:spacing w:after="80" w:line="240" w:lineRule="auto"/>'))
    if meta.get('subtitle'):
        d.add(d.para(d.run(meta['subtitle'], size=30, color=N.PURPLE), spacing='<w:spacing w:after="160"/>'))
    d.add(d.para('', pbdr=f'<w:pBdr><w:bottom w:val="single" w:sz="18" w:space="1" w:color="{N.MAGENTA}"/></w:pBdr>',
                 spacing='<w:spacing w:after="200"/>', ind='<w:ind w:right="6200"/>'))
    if meta.get('summary'):
        d.add(d.para(d.run(meta['summary'], size=23, color='3F3F4A', italic=True),
                     spacing='<w:spacing w:after="320" w:line="276" w:lineRule="auto"/>'))
    if meta.get('cover_image', 'none') == 'shaddad':
        d.add(N.Doc.inline_image('rId7', 3781425, 2524125, 1, 'shaddad_platform.png', 'Shaddad tracked platform',
                                 spacing_after=40))
        d.add(d.para(d.run('Shaddad tracked platform (photo: nasl-tech.com)', size=16, color=N.GREY, italic=True),
                     spacing='<w:spacing w:after="280"/>'))
    d.kv_table([('Document type', meta.get('doc_type', '').capitalize()), ('Document No.', meta['doc_no']),
                ('Entity', meta.get('entity', 'NASL Technologies')), ('Version', meta.get('version', '1.0')),
                ('Date', meta.get('date', '')), ('Status', meta.get('status', 'Draft')),
                ('Prepared by', meta.get('prepared_by', '[TBD]')), ('Reviewer', meta.get('reviewer', '[TBD]')),
                ('Approver', meta.get('approver', '[TBD]')), ('Classification', meta.get('classification', 'Internal'))],
               widths=(2707, 6319))
    # document control
    d.add(d.para(d.run('Document Control'), style='NASLPageTitle', page_break=True))
    hist = meta['history'] or [[meta.get('version', '1.0'), meta.get('date', ''), 'First issue']]
    d.table(['Version', 'Date', 'Prepared by', 'Reviewer', 'Changes'],
            [[h[0], h[1] if len(h) > 1 else '', meta.get('prepared_by', '[TBD]').split(' (')[0],
              {'paras': [meta.get('reviewer', '[TBD]')], 'color': N.MAGENTA, 'bold': True},
              h[2] if len(h) > 2 else ''] for h in hist],
            [992, 1353, 1805, 1805, 3071], valign='center')
    d.add(d.para(d.run('Document information', size=24, color=N.NAVY, bold=True),
                 spacing='<w:spacing w:before="240" w:after="80"/>'))
    info = [('Title', meta['title']), ('Document No.', meta['doc_no']), ('Status', meta.get('status', 'Draft'))]
    for k, lab in (('request', 'Request'), ('scope', 'Scope'), ('related', 'Related')):
        if meta.get(k):
            info.append((lab, meta[k]))
    info += [('Source', src_rel + ' (the controlled source; this file is generated from it)'),
             ('File', meta['file'] + '.docx / .pdf')]
    d.kv_table(info)
    d.add(d.para(d.run('Contents'), style='NASLPageTitle', page_break=True))
    d.toc()
    render_body(d, E.parse(body_md), images, meta.get('chapter_breaks', 'no').lower() in ('yes', 'true', '1'))
    return d, images


def make(src, pdf=True):
    full = src if os.path.isabs(src) else os.path.join(ROOT, src)
    text = open(full, encoding='utf-8').read()
    meta = read_header(text)
    if not meta:
        sys.exit(f'{src}: no document header (an HTML comment with doc_no, title, ... at the top)')
    src_rel = os.path.relpath(full, ROOT).replace('\\', '/')
    outdir = os.path.join(ROOT, meta.get('outdir', 'work/reports/documents'))
    os.makedirs(outdir, exist_ok=True)
    E.FIGDIR = os.path.join(outdir, 'fig')
    os.makedirs(E.FIGDIR, exist_ok=True)
    d, images = build(meta, text, src_rel)
    docx = os.path.join(outdir, meta['file'] + '.docx')
    m = {'title': meta['title'], 'date': meta.get('date', ''),
         'footer': f"{meta.get('entity', 'NASL Technologies').split(' (')[0]} | {meta.get('footer', meta['title'])} | "
                   f"{meta.get('classification', 'Internal')}",
         'subject': meta.get('subtitle', ''), 'keywords': f"{meta.get('doc_type', '')}, NASL",
         'description': f"{meta.get('entity', 'NASL Technologies')} - {meta.get('classification', 'Internal')} - "
                        f"{meta['doc_no']} - Version {meta.get('version', '1.0')} - {meta.get('date', '')}"}
    nbody = N.package(d, docx, m, images)
    print(f"written {docx}: tables {d.n['Table']}, figures {d.n['Figure']}, body {nbody / 1e3:.0f} kB")
    if pdf:
        out = os.path.join(outdir, meta['file'] + '.pdf')
        r = subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', E.PS1,
                            '-Docx', docx, '-Pdf', out], capture_output=True, text=True, timeout=900)
        print((r.stdout or '').strip() or (r.stderr or '').strip()[-800:])


def index(folders):
    rows = []
    for f in folders or ['framework', 'docs']:
        for p in glob.glob(os.path.join(ROOT, f, '**', '*.md'), recursive=True):
            meta = read_header(open(p, encoding='utf-8').read(4000))
            if meta and '<' not in meta['doc_no']:            # templates carry placeholders: not documents
                rows.append((meta['doc_no'], meta.get('version', ''), meta.get('status', ''), meta['title'],
                             os.path.relpath(p, ROOT).replace('\\', '/')))
    for r in sorted(rows):
        print(' | '.join(r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', nargs='*')
    ap.add_argument('--no-pdf', action='store_true')
    ap.add_argument('--index', action='store_true')
    a = ap.parse_args()
    if a.index:
        index(a.source)
        return
    for s in a.source:
        make(s, pdf=not a.no_pdf)


if __name__ == '__main__':
    main()
