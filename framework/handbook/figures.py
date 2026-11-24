"""Draw the figures of the Engineering Handbook (NASL-EMS-HBK-001) as SVG files in framework/handbook/fig/.

usage: python framework/handbook/figures.py
The document builder (tools/reports/build_document.py) renders each SVG to PNG with Edge headless at 2x.
Plain python, no installs. Colours are the NASL report colours.
"""
import os
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'fig')
NAVY, MAG, PUR, LIL, GREY, LINE = '#2B2F7E', '#A64B9C', '#6E4FA3', '#EEEAF6', '#5F6070', '#6E6D8A'
FONT = 'font-family="Segoe UI, Arial, sans-serif"'


def text(x, y, s, size=22, weight=400, fill=NAVY, anchor='middle', italic=False):
    lines = s.split('\n')
    lh = size * 1.22
    y0 = y - (len(lines) - 1) * lh / 2 + size * 0.35
    st = ' font-style="italic"' if italic else ''
    return ''.join(f'<text x="{x:.1f}" y="{y0 + i * lh:.1f}" text-anchor="{anchor}" {FONT} font-size="{size}" '
                   f'font-weight="{weight}" fill="{fill}"{st}>{escape(t)}</text>' for i, t in enumerate(lines))


def box(x, y, w, h, s, fill=LIL, stroke=NAVY, color=NAVY, size=22, weight=400, rx=8, sw=2, dash=False):
    da = ' stroke-dasharray="7 5"' if dash else ''
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{sw}"{da}/>' + text(x + w / 2, y + h / 2, s, size, weight, color))


def diamond(cx, cy, w, h, s, fill='#ffffff', stroke=MAG, color=MAG, size=22, weight=700):
    pts = f'{cx},{cy - h / 2} {cx + w / 2},{cy} {cx},{cy + h / 2} {cx - w / 2},{cy}'
    return (f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="2.5"/>'
            + text(cx, cy, s, size, weight, color))


def arrow(x1, y1, x2, y2, color=LINE, dash=False, label=None, lsize=18, lx=0, ly=-10, anchor='middle', sw=2.2):
    da = ' stroke-dasharray="7 5"' if dash else ''
    s = (f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{sw}"{da} '
         'marker-end="url(#ah)"/>')
    if label:
        s += text((x1 + x2) / 2 + lx, (y1 + y2) / 2 + ly, label, lsize, 400, GREY, anchor, italic=True)
    return s


def path(d, color=LINE, dash=False, sw=2.2):
    da = ' stroke-dasharray="7 5"' if dash else ''
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{sw}"{da} marker-end="url(#ah)"/>'


def svg(name, W, H, parts):
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
            '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
            f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{LINE}"/></marker></defs>'
            f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), 'w', encoding='utf-8') as f:
        f.write(head + ''.join(parts) + '</svg>')


def f1_layers():
    p = [box(20, 20, 560, 470, '', fill='#F6F5FB', stroke=NAVY, rx=12),
         text(300, 58, 'FRAMEWORK', 26, 700), text(300, 90, 'company level, the same for every project', 19, 400, GREY),
         box(50, 120, 500, 76, 'Engineering Handbook  NASL-EMS-HBK-001\nhow we plan, build, verify and document', size=20),
         box(50, 210, 500, 76, 'Templates\ncard, gate package, CR, test procedure, reports', size=20),
         box(50, 300, 500, 76, 'Tools\ndashboard, document and report builders, runners', size=20),
         box(50, 390, 500, 76, 'Team kit\nroles, work rules, briefs, step reports', size=20),
         box(720, 20, 560, 470, '', fill='#F6F5FB', stroke=MAG, rx=12),
         text(1000, 58, 'PROJECT INSTANCE', 26, 700, MAG), text(1000, 90, 'one per project, made with the framework', 19, 400, GREY),
         box(750, 120, 500, 60, 'Project plan: tailoring, WBS, team', size=20, stroke=MAG),
         box(750, 192, 500, 60, 'Task cards under the gates', size=20, stroke=MAG),
         box(750, 264, 500, 60, 'Registers: requirements, risks, decisions, changes', size=19, stroke=MAG),
         box(750, 336, 500, 60, 'Baselines and controlled documents', size=20, stroke=MAG),
         box(750, 408, 500, 60, 'Dashboard: the live status', size=20, stroke=MAG),
         arrow(585, 210, 712, 210, label='copy and\nconfigure', ly=-30, lsize=17),
         arrow(712, 330, 585, 330, dash=True, label='lessons\nlearned', ly=34, lsize=17)]
    svg('f1_layers.svg', 1300, 510, p)


def f2_lifecycle():
    W = 1500
    row1 = [('Concept\nneed, ConOps, charter', 'G0', 'MCR', ''),
            ('Requirements and\nfunctional design', 'G1', 'SRR + SFR', 'functional baseline'),
            ('Preliminary design\nper subsystem', 'G2', 'PDR', 'allocated baseline'),
            ('Detailed design\nper subsystem', 'G3', 'CDR', 'build-to baseline')]
    row2 = [('Build, code\nand integrate', 'G4', 'TRR', 'go-ahead per campaign'),
            ('Verification\ncampaigns', 'G5', 'FCA / SVR', 'verified baseline'),
            ('Production\npreparation (optional)', 'G6', 'PRR', ''),
            ('Configuration audit\nand handover', 'G7', 'PCA', 'final product baseline')]
    p = []
    for r, (y, row) in enumerate(((50, row1), (330, row2))):
        for i, (ph, g, rv, bl) in enumerate(row):
            x = 20 + i * 340
            p.append(box(x, y, 230, 96, ph, size=21))
            gx = x + 285
            p.append(arrow(x + 230, y + 48, gx - 47, y + 48))
            p.append(diamond(gx, y + 48, 92, 92, g, size=24))
            p.append(text(gx, y + 118, rv, 19, 700, NAVY))
            if bl:
                p.append(text(gx, y + 146, bl, 17, 400, MAG, italic=True))
            if i < 3:
                p.append(arrow(gx + 47, y + 48, x + 340, y + 48))
    p.append(path(f'M1372,98 L1440,98 L1440,240 L135,240 L135,{330 - 4}'))
    p.append(text(1325, 522, 'then operations and support', 18, 400, GREY, italic=True))
    p.append(text(720, 280, 'every gate: success criteria, a readiness check, the review, the owner\'s decision', 19, 400, GREY, italic=True))
    svg('f2_lifecycle.svg', W, 540, p)


def f3_states():
    xs = [30, 245, 460, 675, 890, 1105]
    names = ['Proposed', 'Planned', 'Ready', 'In work', 'In review', 'Done']
    labels = ['owner\napproves', 'entry\ncriteria met', 'lead\ndispatches', 'pass done,\nreport in', 'exit criteria\nmet, record in']
    p = []
    for i, (x, n) in enumerate(zip(xs, names)):
        fill = NAVY if n == 'Done' else LIL
        p.append(box(x, 120, 170, 72, n, fill=fill, color='#ffffff' if n == 'Done' else NAVY, size=23, weight=700))
        if i < 5:
            p.append(arrow(x + 170, 156, xs[i + 1], 156))
            p.append(text(x + 192, 76, labels[i], 17, 400, GREY, italic=True))
    p.append(path('M975,192 C975,262 790,262 790,196'))
    p.append(text(1000, 236, 'fix once\n(real breakage only)', 17, 400, GREY, 'start', italic=True))
    p += [box(470, 330, 200, 66, 'Needs you\n(owner decision)', fill='#ffffff', stroke=MAG, color=MAG, size=19),
          box(700, 330, 200, 66, 'Blocked\n(waits on a card)', fill='#ffffff', stroke=GREY, color=GREY, size=19),
          arrow(705, 192, 600, 326), arrow(620, 330, 722, 196, dash=True),
          arrow(745, 192, 770, 326), arrow(790, 330, 765, 196, dash=True),
          box(160, 330, 200, 66, 'Cancelled\n(reason recorded)', fill='#ffffff', stroke=GREY, color=GREY, size=19, dash=True),
          arrow(115, 192, 220, 326, dash=True), arrow(330, 192, 270, 326, dash=True), arrow(545, 192, 330, 326, dash=True)]
    svg('f3_states.svg', 1300, 420, p)


def f4_intake():
    p = [box(20, 240, 230, 100, 'A request\nfrom the owner', fill=NAVY, color='#ffffff', size=22, weight=700),
         arrow(250, 290, 318, 290), diamond(410, 290, 180, 130, 'What\nis it?', size=21)]
    rows = [('Question', 'Answer it. No card.'),
            ('New work', 'New card(s): WBS element, gate,\nlead, entry and exit, size; owner approves'),
            ('Change to\na baseline', 'Change request: impact analysis,\nthe change board decides; then card(s)'),
            ('Defect or\nnonconformance', 'Issue or NCR: disposition (fix, waive,\nchange the requirement); then a card')]
    for i, (k, v) in enumerate(rows):
        y = 40 + i * 130
        p.append(path(f'M500,290 C540,290 540,{y + 45} 590,{y + 45}'))
        p.append(box(594, y, 200, 90, k, size=20, weight=700))
        p.append(arrow(794, y + 45, 850, y + 45))
        p.append(box(854, y, 420, 90, v, fill='#ffffff', size=19))
    svg('f4_intake.svg', 1300, 570, p)


def f5_cycle():
    steps = ['1 Check entry\nthe card is ready', '2 Brief\nshared and per engineer', '3 Dispatch\nowners in parallel',
             '4 Engineer pass\nsnapshot, change,\nrebuild, checks', '5 Integration\ncheck (cross-\nsubsystem)',
             '6 Tests\nagainst the baseline', '7 Independent\nreview: keep, fix\nonce, or ask',
             '8 Docs\nupdated', '9 Record\nATR and PDF report', '10 Close\ncard, registers,\ndashboard']
    p = []
    for i, s in enumerate(steps):
        r, c = divmod(i, 5)
        x, y = 25 + c * 255, 40 + r * 210
        fill = NAVY if i == 9 else LIL
        p.append(box(x, y, 220, 120, s, fill=fill, color='#ffffff' if i == 9 else NAVY, size=19,
                     weight=700 if i in (0, 9) else 400))
        if c < 4:
            p.append(arrow(x + 220, y + 60, x + 255, y + 60))
    p.append(path('M1155,160 L1155,205 L135,205 L135,246'))
    svg('f5_cycle.svg', 1300, 380, p)


def f6_change():
    p = [box(20, 40, 250, 110, 'Change request\nwhat, why, who', size=20, weight=700),
         arrow(270, 95, 330, 95),
         box(334, 40, 330, 110, 'Impact analysis by the owners\nrequirements, interfaces, tests,\ncost, schedule, safety', size=19),
         arrow(664, 95, 724, 95),
         diamond(830, 95, 200, 150, 'Change\nboard\n(the owner)', size=19),
         arrow(930, 95, 1000, 95, label='approve', ly=-14),
         box(1004, 40, 276, 110, 'Implementation\ncard(s)', size=20),
         arrow(830, 170, 830, 236, label='reject or defer', lx=80, ly=0),
         box(690, 240, 280, 76, 'Recorded with the reason;\nthe CR is closed', fill='#ffffff', stroke=GREY, color=GREY, size=18),
         arrow(1142, 150, 1142, 236),
         box(1004, 240, 276, 76, 'Verification\nof the change', size=20),
         path('M1142,316 L1142,370 L560,370'),
         box(290, 340, 266, 66, 'Baseline updated,\nCR closed', fill=NAVY, color='#ffffff', size=20, weight=700)]
    svg('f6_change.svg', 1300, 430, p)


def f7_gate():
    steps = ['1 Readiness check\nscore every success\ncriterion', '2 Review package\nthe evidence\nper criterion',
             '3 Independent\nassessment:\nfindings as RIDs', '4 Gate review\nthe board walks the\ncriteria and RIDs']
    p = []
    for i, s in enumerate(steps):
        x = 20 + i * 245
        p.append(box(x, 40, 220, 130, s, size=18, weight=700 if i == 0 else 400))
        p.append(arrow(x + 220, 105, x + 245, 105))
    p.append(diamond(1105, 105, 170, 150, 'Decision\nby the owner', size=19))
    outs = [('Go', 'all mandatory criteria met or waived;\nno Critical or Major RID open', NAVY),
            ('Go with actions', 'mandatory criteria met or waived; open\nMajor or Minor RIDs on cards with dates', PUR),
            ('Delta review', 'some mandatory criteria not met:\nfix, then re-review only those', MAG),
            ('No-go', 'a basic problem: stay in the phase\nand re-plan', GREY)]
    for i, (k, v, col) in enumerate(outs):
        y = 230 + i * 78
        p.append(path(f'M1105,180 C1105,{y + 30} 900,{y + 30} 838,{y + 30}'))
        p.append(box(30, y, 230, 60, k, fill=col, color='#ffffff', size=20, weight=700))
        p.append(box(270, y, 560, 60, v, fill='#ffffff', stroke=col, color=col, size=17))
    p.append(box(860, 560, 420, 66, '5 Record: the gate report (PDF), the decision\nrecord, the gate status, the baseline locked',
                 fill=NAVY, color='#ffffff', size=17))
    svg('f7_gate.svg', 1300, 650, p)


def f8_tools():
    src = ['Task cards (JSON)', 'Registers: requirements,\nrisks, decisions, changes', 'Document sources\n(markdown + header)',
           'Models, builders,\nparameters', 'Run logs and test results']
    gen = ['Card builder', 'Register views', 'Document builder', 'Builders and the\nbatch runner', 'Test baseline tool']
    out = ['Task-card PDF', 'Registers in the docs', 'Controlled PDFs\n(NASL template)', 'Models and code', 'Known failures,\nnew failures']
    p = [text(150, 30, 'SOURCES (edited)', 20, 700), text(520, 30, 'GENERATORS', 20, 700), text(880, 30, 'OUTPUTS (generated)', 20, 700)]
    for i in range(5):
        y = 50 + i * 100
        p.append(box(20, y, 260, 80, src[i], size=18))
        p.append(arrow(280, y + 40, 386, y + 40))
        p.append(box(390, y, 260, 80, gen[i], fill='#ffffff', size=18))
        p.append(arrow(650, y + 40, 746, y + 40))
        p.append(box(750, y, 260, 80, out[i], size=18))
        p.append(arrow(1010, y + 40, 1096, 290))
    p.append(box(1100, 210, 180, 160, 'Dashboard\nreads it all,\nwrites nothing', fill=NAVY, color='#ffffff', size=19, weight=700))
    svg('f8_tools.svg', 1300, 560, p)


if __name__ == '__main__':
    for f in (f1_layers, f2_lifecycle, f3_states, f4_intake, f5_cycle, f6_change, f7_gate, f8_tools):
        f()
    print('figures written to', OUT)
