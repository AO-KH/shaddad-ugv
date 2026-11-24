"""Build NASL-SHD-SDD-002, the embedded software design description, from the inventories and C traces of card T-07.

usage:
  python tools/reports/build_sw_design.py [--date YYYY-MM-DD] [--version 1.0] [--status Draft] [--no-pdf]

Reads (paths from the project root):
  work/team/t07/inv/{modes,frames,mcu,bms,gnc,nav}.json   the inventories (schema t07-inventory-1)
  work/team/t07/trace/<model>.json                        the C traces (tools/reports/codegen_trace.py)
  work/team/t07/sil/<model>.json                          the SIL back-to-back records, when they exist
  work/team/t07/cov/coverage.json                         the transitions seen in the runs, when it exists
  work/team/t07/logs/full/<class>.log                     the full run, when it exists
Writes the flowcharts, the state diagrams and the overview as SVG in work/team/t07/sdd/fig/, the document source
work/team/t07/sdd/SDD-002.md, and (unless --no-pdf) the Word file and PDF through tools/reports/build_document.py into
work/reports/design/. Plain python, no installs; Edge renders the SVG figures in the builder.
"""
import argparse
import datetime
import glob
import json
import math
import os
import re
import subprocess
import sys
from collections import OrderedDict, defaultdict
from xml.sax.saxutils import escape

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T07 = os.path.join(ROOT, 'work', 'team', 't07')
SDD = os.path.join(T07, 'sdd')
FIG = os.path.join(SDD, 'fig')
PARTS = [('modes', 'The system states and modes'), ('frames', 'The frames'), ('mcu', 'MCU firmware'),
         ('bms', 'BMS firmware'), ('gnc', 'GNC on the companion'), ('nav', 'NavFusion and Perception on the companion')]
NAVY, MAG, PUR, LIL, GREY, LINE, GREEN = '#2B2F7E', '#A64B9C', '#6E4FA3', '#EEEAF6', '#5F6070', '#6E6D8A', '#2E7D5B'
FONT = 'font-family="Segoe UI, Arial, sans-serif"'
CW = 7.1                     # the average width of a 13 px character
STEP_NAMES = re.compile(r'(_step\d*|_initialize|_terminate|_output|_update|^main|^rt_OneStep)$')
SEV_ORDER = {'defect': 0, 'test gap': 1, 'doc': 2, 'observation': 3}


# ----------------------------------------------------------------------------------------------------------- helpers
def rel(p):
    return os.path.relpath(p, ROOT).replace('\\', '/')


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def cell(s):
    """A markdown table cell: one line, no pipes."""
    s = '' if s is None else str(s)
    return re.sub(r'\s+', ' ', s.replace('|', '/')).strip()


def joinmax(items, n, more='more'):
    items = [str(i) for i in items if i]
    if len(items) <= n:
        return ', '.join(items)
    return ', '.join(items[:n]) + ', and %d %s' % (len(items) - n, more)


def wrap(text, width):
    lines, cur = [], ''
    for w in str(text).split():
        while len(w) > width:
            if cur:
                lines.append(cur)
                cur = ''
            lines.append(w[:width])
            w = w[width:]
        if len(cur) + len(w) + (1 if cur else 0) <= width:
            cur = (cur + ' ' + w).strip()
        else:
            if cur:
                lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines or ['']


def svg_text(x, y, lines, size=13, weight=400, fill=NAVY, anchor='middle', lh=None):
    lh = lh or size * 1.25
    y0 = y - (len(lines) - 1) * lh / 2 + size * 0.35
    return ''.join('<text x="%.1f" y="%.1f" text-anchor="%s" %s font-size="%d" font-weight="%d" fill="%s">%s</text>'
                   % (x, y0 + i * lh, anchor, FONT, size, weight, fill, escape(t)) for i, t in enumerate(lines))


def svg_doc(w, h, body):
    marker = ('<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
              'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>'
              '<marker id="arrg" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
              'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker></defs>' % (LINE, '#A9A8BC'))
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">%s'
            '<rect width="100%%" height="100%%" fill="white"/>%s</svg>' % (w, h, w, h, marker, body))


def label_box(x, y, lines, size=11, fill=GREY, anchor='middle'):
    wpx = max(len(t) for t in lines) * size * 0.56 + 6
    hpx = len(lines) * size * 1.2 + 4
    x0 = x - wpx / 2 if anchor == 'middle' else (x - 3 if anchor == 'start' else x - wpx + 3)
    return ('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="white" opacity="0.92"/>'
            % (x0, y - hpx / 2, wpx, hpx) + svg_text(x, y, lines, size, 400, fill, anchor, size * 1.2))


def write_svg(name, w, h, body):
    os.makedirs(FIG, exist_ok=True)
    path = os.path.join(FIG, name + '.svg')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(svg_doc(int(w), int(h), body))
    return rel(path)


# ------------------------------------------------------------------------------------------------------- flowcharts
def flow_size(n):
    t = n.get('type', 'process')
    if t == 'decision':
        lines = wrap(n.get('text', ''), 20)
        tw, th = max(len(s) for s in lines) * CW, len(lines) * 16.5
        return lines, tw * 1.5 + 44, th * 1.55 + 34
    lines = wrap(n.get('text', ''), 26 if t != 'start' and t != 'end' else 28)
    w = max(120, max(len(s) for s in lines) * CW + (40 if t in ('io', 'subroutine') else 28))
    return lines, w, len(lines) * 16.5 + 18


def flow_shape(n, x, y, w, h, lines):
    t = n.get('type', 'process')
    if t in ('start', 'end'):
        s = ('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="%.1f" fill="%s"/>'
             % (x - w / 2, y - h / 2, w, h, min(h / 2, 22), NAVY))
        return s + svg_text(x, y, lines, 13, 600, 'white')
    if t == 'decision':
        pts = '%.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f' % (x, y - h / 2, x + w / 2, y, x, y + h / 2, x - w / 2, y)
        return ('<polygon points="%s" fill="white" stroke="%s" stroke-width="2"/>' % (pts, MAG)
                + svg_text(x, y, lines, 13, 400, NAVY))
    if t == 'io':
        k = 12
        pts = '%.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f' % (x - w / 2 + k, y - h / 2, x + w / 2 + k, y - h / 2,
                                                       x + w / 2 - k, y + h / 2, x - w / 2 - k, y + h / 2)
        return ('<polygon points="%s" fill="%s" stroke="%s" stroke-width="1.5"/>' % (pts, LIL, PUR)
                + svg_text(x, y, lines, 13))
    s = ('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="4" fill="%s" stroke="%s" stroke-width="1.5"/>'
         % (x - w / 2, y - h / 2, w, h, LIL, NAVY))
    if t == 'subroutine':
        s += ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.5"/>'
              '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.5"/>'
              % (x - w / 2 + 9, y - h / 2, x - w / 2 + 9, y + h / 2, NAVY,
                 x + w / 2 - 9, y - h / 2, x + w / 2 - 9, y + h / 2, NAVY))
    return s + svg_text(x, y, lines, 13)


def flowchart(flow, name):
    """A top-down layered flowchart; back edges and long skips run in side lanes."""
    nodes = OrderedDict((n['id'], n) for n in flow.get('nodes', []) if n.get('id'))
    if not nodes:
        return None
    edges = [e for e in flow.get('edges', []) if e.get('from') in nodes and e.get('to') in nodes]
    out = defaultdict(list)
    for e in edges:
        out[e['from']].append(e)
    start = next((k for k, n in nodes.items() if n.get('type') == 'start'), next(iter(nodes)))
    state, back, disc = {}, set(), []

    def dfs(u):
        state[u] = 1
        disc.append(u)
        for e in out[u]:
            v = e['to']
            if state.get(v) == 1:
                back.add(id(e))
            elif v not in state:
                dfs(v)
        state[u] = 2

    sys.setrecursionlimit(10000)
    dfs(start)
    for k in nodes:
        if k not in state:
            dfs(k)
    rank = {k: 0 for k in nodes}
    for _ in range(len(nodes) + 1):
        changed = False
        for e in edges:
            if id(e) in back:
                continue
            if rank[e['to']] < rank[e['from']] + 1:
                rank[e['to']] = rank[e['from']] + 1
                changed = True
        if not changed:
            break
    geo = {k: flow_size(n) for k, n in nodes.items()}
    rows = defaultdict(list)
    for k in disc:
        rows[rank[k]].append(k)
    nrank = max(rows) + 1
    preds = defaultdict(list)
    for e in edges:
        if id(e) not in back:
            preds[e['to']].append(e['from'])
    gap_x, gap_y = 46, 58
    widths = [sum(geo[k][1] for k in rows[r]) + gap_x * (len(rows[r]) - 1) for r in range(nrank)]
    core = max(widths)
    xs, ys = {}, {}
    y = 30
    for r in range(nrank):
        row = rows[r]
        if r > 0:
            def bc(k):
                p = [xs[q] for q in preds[k] if q in xs]
                return sum(p) / len(p) if p else core / 2
            row.sort(key=bc)
        h = max(geo[k][2] for k in row)
        total = widths[r]
        x = (core - total) / 2
        if len(row) == 1:
            p = [xs[q] for q in preds[row[0]] if q in xs]
            if p:
                x = min(max(sum(p) / len(p) - geo[row[0]][1] / 2, 0), core - geo[row[0]][1])
        for k in row:
            xs[k] = x + geo[k][1] / 2
            ys[k] = y + h / 2
            x += geo[k][1] + gap_x
        y += h + gap_y
    lanes_l = [e for e in edges if id(e) not in back and rank[e['to']] > rank[e['from']] + 1]
    lanes_r = [e for e in edges if id(e) in back or rank[e['to']] <= rank[e['from']] and id(e) not in back]
    pad_l, pad_r = 30 + 16 * len(lanes_l), 30 + 16 * len(lanes_r)
    W, H = core + pad_l + pad_r, y - gap_y + 30
    for k in xs:
        xs[k] += pad_l
    body = []
    labels = []
    used_exit = defaultdict(set)

    def exit_point(u, v, side=None):
        x, y_, w, h = xs[u], ys[u], geo[u][1], geo[u][2]
        if nodes[u].get('type') == 'decision':
            if side is None:
                dx = xs[v] - x
                side = 'bottom' if abs(dx) < 12 or 'bottom' not in used_exit[u] and abs(dx) < w / 2 else (
                    'right' if dx > 0 else 'left')
                if side in used_exit[u]:
                    side = next(s for s in ('bottom', 'right', 'left') if s not in used_exit[u]) \
                        if len(used_exit[u]) < 3 else side
            used_exit[u].add(side)
            return side, {'bottom': (x, y_ + h / 2), 'right': (x + w / 2, y_), 'left': (x - w / 2, y_)}[side]
        if side in ('right', 'left'):
            return side, (x + (w / 2 if side == 'right' else -w / 2), y_)
        return 'bottom', (x, y_ + h / 2)

    for e in edges:
        u, v = e['from'], e['to']
        lab = e.get('label')
        if id(e) not in back and rank[v] == rank[u] + 1:
            side, (x0, y0) = exit_point(u, v)
            xt, yt = xs[v], ys[v] - geo[v][2] / 2
            if side == 'bottom':
                ym = y0 + (yt - y0) / 2
                d = 'M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f L%.1f,%.1f' % (x0, y0, x0, ym, xt, ym, xt, yt)
                lx, ly, anc = x0 + 5, y0 + 11, 'start'
            else:
                d = 'M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f' % (x0, y0, xt, y0, xt, yt)
                lx, ly, anc = x0 + (8 if side == 'right' else -8), y0 - 9, 'start' if side == 'right' else 'end'
        elif id(e) not in back and rank[v] > rank[u] + 1:
            k = lanes_l.index(e)
            lane = pad_l - 22 - 16 * k
            side, (x0, y0) = exit_point(u, v, 'left')
            xt, yt = xs[v] - geo[v][1] / 2, ys[v]
            if nodes[v].get('type') == 'decision':
                xt = xs[v] - geo[v][1] / 2
            d = 'M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f L%.1f,%.1f' % (x0, y0, lane, y0, lane, yt, xt, yt)
            lx, ly, anc = x0 - 8, y0 - 9, 'end'
        else:
            k = lanes_r.index(e)
            lane = pad_l + core + 22 + 16 * k
            side, (x0, y0) = exit_point(u, v, 'right')
            xt, yt = xs[v] + geo[v][1] / 2, ys[v]
            d = 'M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f L%.1f,%.1f' % (x0, y0, lane, y0, lane, yt, xt, yt)
            lx, ly, anc = x0 + 8, y0 - 9, 'start'
        body.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.6" marker-end="url(#arr)"/>' % (d, LINE))
        if lab:
            labels.append(label_box(lx, ly, wrap(lab, 18), 11, MAG, anc))
    for k, n in nodes.items():
        body.append(flow_shape(n, xs[k], ys[k], geo[k][1], geo[k][2], geo[k][0]))
    return write_svg(name, W, H, ''.join(body + labels))


# -------------------------------------------------------------------------------------------------- state diagrams
def box_point(cx, cy, hw, hh, dx, dy):
    tx = hw / abs(dx) if dx else float('inf')
    ty = hh / abs(dy) if dy else float('inf')
    t = min(tx, ty)
    return cx + dx * t, cy + dy * t


def wrap_name(s, width):
    """A state name on several lines, broken after an underscore, never inside a word (review finding T07-CHK-06)."""
    if len(s) <= width or '_' not in s:
        return wrap(s, width)
    lines, cur = [], ''
    for part in re.findall(r'[^_]*_|[^_]+$', s):
        if cur and len(cur) + len(part) > width:
            lines.append(cur)
            cur = part
        else:
            cur += part
    if cur:
        lines.append(cur)
    return lines


def state_diagram(fsm, name, seen=None):
    """States in rows by their distance from the initial state; one curved arrow per (from, to) pair, labelled with
    the transition ids; '*' transitions start from an 'any state' box. Solid arrows were seen in the runs (when the
    coverage is known), dashed ones were not."""
    states = [s.get('name') for s in fsm.get('states', []) if s.get('name')]
    if not states:
        return None
    trans = fsm.get('transitions', [])
    pairs = OrderedDict()
    for t in trans:
        a, b = t.get('from'), t.get('to')
        if b not in states or (a not in states and a != '*'):
            continue
        pairs.setdefault((a, b), []).append(str(t.get('id')))
    init = fsm.get('initial') if fsm.get('initial') in states else states[0]
    adj = defaultdict(list)
    for (a, b) in pairs:
        if a != '*' and a != b:
            adj[a].append(b)
    dist, queue = {init: 0}, [init]
    while queue:
        u = queue.pop(0)
        for v in adj[u]:
            if v not in dist:
                dist[v] = dist[u] + 1
                queue.append(v)
    far = max(dist.values()) + 1
    for s in states:
        dist.setdefault(s, far)
    parents = defaultdict(list)
    for (a, b) in pairs:
        if a != '*' and a != b:
            parents[b].append(a)
    geo = {}
    vals = {s.get('name'): s.get('value') for s in fsm.get('states', [])}
    for s in states:
        lines = wrap_name(s, 16)
        sub = '' if vals.get(s) in (None, '') else '= %s' % vals.get(s)
        w = max(104, max(len(t) for t in lines + [sub]) * 7.8 + 30)
        h = len(lines) * 17 + (14 if sub else 0) + 20
        geo[s] = (lines, sub, w, h)
    by_dist = defaultdict(list)
    for s in states:
        by_dist[dist[s]].append(s)
    any_box = any(a == '*' for (a, b) in pairs)
    gap_x, gap_y, max_row = 100, 112, 5
    rows, xs, ys = [], {}, {}
    for r in sorted(by_dist):                      # rows by the distance from the initial state, at most 5 wide
        group = by_dist[r]
        if r > 0:
            placed = {q: i for i, q in enumerate(s for row in rows for s in row)}

            def bc(s):
                p = [placed[q] for q in parents[s] if q in placed]
                return sum(p) / len(p) if p else 1e9
            group.sort(key=bc)
        for i in range(0, len(group), max_row):
            rows.append(group[i:i + max_row])
    widths = [sum(geo[s][2] for s in row) + gap_x * (len(row) - 1) for row in rows]
    core = max(widths + [300])
    pad = 90
    y = 60 + (76 if any_box else 0)
    for row, wd in zip(rows, widths):
        h = max(geo[s][3] for s in row)
        x = pad + (core - wd) / 2
        for s in row:
            xs[s], ys[s] = x + geo[s][2] / 2, y + h / 2
            x += geo[s][2] + gap_x
        y += h + gap_y
    W, H = core + 2 * pad + 40, y - gap_y + 56
    body, labels = [], []
    if any_box:
        ax, ay, aw, ah = pad + 60, 46, 120, 34
        body.append('<rect x="%.1f" y="%.1f" width="%d" height="%d" rx="10" fill="white" stroke="%s" '
                    'stroke-dasharray="5 4" stroke-width="1.5"/>' % (ax - aw / 2, ay - ah / 2, aw, ah, GREY)
                    + svg_text(ax, ay, ['any state'], 12, 400, GREY))
        xs['*'], ys['*'], geo['*'] = ax, ay, ([], '', aw, ah)
    boxes = {s: (xs[s], ys[s], geo[s][2] / 2 + 6, geo[s][3] / 2 + 6) for s in xs}
    taken = []                                      # label rectangles already placed

    def in_box(px, py, skip):
        return any(k not in skip and abs(px - bx) < hw and abs(py - by) < hh for k, (bx, by, hw, hh) in boxes.items())

    def bez(p0, c, p1, t):
        return ((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * c[0] + t * t * p1[0],
                (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * c[1] + t * t * p1[1])

    def free(r):
        x0, y0, x1, y1 = r
        for k, (bx, by, hw, hh) in boxes.items():
            if x0 < bx + hw - 4 and x1 > bx - hw + 4 and y0 < by + hh - 4 and y1 > by - hh + 4:
                return False
        return all(x0 >= q[2] or x1 <= q[0] or y0 >= q[3] or y1 <= q[1] for q in taken)

    def place_label(p0, c, p1, nx, ny, lab):
        wpx = max(len(t) for t in lab) * 11 * 0.56 + 8
        hpx = len(lab) * 13.2 + 6
        for dist_ in (9, 22, 36):
            for t in (0.5, 0.42, 0.58, 0.34, 0.66, 0.26, 0.74, 0.18, 0.82):
                for side in (1, -1):
                    px, py = bez(p0, c, p1, t)
                    px, py = px + nx * dist_ * side, py + ny * dist_ * side
                    r = (px - wpx / 2, py - hpx / 2, px + wpx / 2, py + hpx / 2)
                    if free(r):
                        taken.append(r)
                        return px, py
        px, py = bez(p0, c, p1, 0.5)
        taken.append((px - wpx / 2, py - hpx / 2, px + wpx / 2, py + hpx / 2))
        return px, py

    w0, h0 = geo[init][2], geo[init][3]
    ix, iy = xs[init] - w0 / 2 - 34, ys[init]
    body.append('<circle cx="%.1f" cy="%.1f" r="7" fill="%s"/>' % (ix, iy, NAVY))
    body.append('<path d="M%.1f,%.1f L%.1f,%.1f" stroke="%s" stroke-width="1.6" marker-end="url(#arr)"/>'
                % (ix + 7, iy, xs[init] - w0 / 2, iy, NAVY))
    taken.append((ix - 10, iy - 10, xs[init] - w0 / 2, iy + 10))
    for (a, b), ids in pairs.items():
        was_seen = None if seen is None else any(i in seen for i in ids)
        color = LINE if was_seen in (None, True) else '#A9A8BC'
        dash = '' if was_seen in (None, True) else ' stroke-dasharray="6 4"'
        mk = 'arr' if was_seen in (None, True) else 'arrg'
        shown = [i + ('*' if (was_seen and i not in seen) else '') for i in ids]   # unseen on a solid arrow
        lab = wrap(', '.join(shown), 16)
        if a == b:
            x, y_, w, h = xs[a], ys[a], geo[a][2], geo[a][3]
            x0, y0, x1, y1 = x + w / 2, y_ - 8, x + w / 2, y_ + 8
            body.append('<path d="M%.1f,%.1f C%.1f,%.1f %.1f,%.1f %.1f,%.1f" fill="none" stroke="%s" '
                        'stroke-width="1.6"%s marker-end="url(#%s)"/>'
                        % (x0, y0, x0 + 46, y0 - 34, x1 + 46, y1 + 34, x1, y1, color, dash, mk))
            wpx = max(len(t) for t in lab) * 11 * 0.56 + 8
            taken.append((x0 + 46, y_ - 8 * len(lab), x0 + 54 + wpx, y_ + 8 * len(lab)))
            labels.append(label_box(x0 + 50, y_, lab, 11, MAG, 'start'))
            continue
        ca, cb = (xs[a], ys[a]), (xs[b], ys[b])
        dx, dy = cb[0] - ca[0], cb[1] - ca[1]
        dl = math.hypot(dx, dy) or 1
        nx, ny = -dy / dl, dx / dl
        base = 26 if (b, a) in pairs else 16
        best = None
        for k in (1, 2, 3.2, 4.6, 6.2, -1, -2, -3.2, -4.6, -6.2):   # bend the arrow until it clears the other boxes
            off = base * k
            c = ((ca[0] + cb[0]) / 2 + nx * off, (ca[1] + cb[1]) / 2 + ny * off)
            p0 = box_point(ca[0], ca[1], geo[a][2] / 2, geo[a][3] / 2, c[0] - ca[0], c[1] - ca[1])
            p1 = box_point(cb[0], cb[1], geo[b][2] / 2, geo[b][3] / 2, c[0] - cb[0], c[1] - cb[1])
            hits = sum(in_box(*bez(p0, c, p1, t / 25.0), skip={a, b}) for t in range(2, 24))
            if best is None or hits < best[0]:
                best = (hits, c, p0, p1)
            if hits == 0:
                break
        _, c, p0, p1 = best
        body.append('<path d="M%.1f,%.1f Q%.1f,%.1f %.1f,%.1f" fill="none" stroke="%s" stroke-width="1.6"%s '
                    'marker-end="url(#%s)"/>' % (p0[0], p0[1], c[0], c[1], p1[0], p1[1], color, dash, mk))
        lx, ly = place_label(p0, c, p1, nx, ny, lab)
        labels.append(label_box(lx, ly, lab, 11, MAG))
    for s in states:
        lines, sub, w, h = geo[s]
        x, y_ = xs[s], ys[s]
        body.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="12" fill="%s" stroke="%s" '
                    'stroke-width="%s"/>' % (x - w / 2, y_ - h / 2, w, h, LIL, NAVY, '2.4' if s == init else '1.5'))
        ty = y_ - (7 if sub else 0)
        body.append(svg_text(x, ty, lines, 13, 700, NAVY))
        if sub:
            body.append(svg_text(x, y_ + h / 2 - 12, [sub], 11, 400, GREY))
    return write_svg(name, W, H, ''.join(body + labels))


# ---------------------------------------------------------------------------------------------------- the overview
def overview(stats):
    def box(x, y, w, h, title, sub, fill=LIL, stroke=NAVY, color=NAVY):
        return ('<rect x="%d" y="%d" width="%d" height="%d" rx="8" fill="%s" stroke="%s" stroke-width="2"/>'
                % (x, y, w, h, fill, stroke) + svg_text(x + w / 2, y + 22, [title], 15, 700, color)
                + svg_text(x + w / 2, y + 22 + 14 + 9 * len(sub), sub, 12, 400, GREY))

    def link(x0, y0, x1, y1, lab, both=False, lx=None, ly=None):
        s = ('<path d="M%d,%d L%d,%d" stroke="%s" stroke-width="2" marker-end="url(#arr)"%s/>'
             % (x0, y0, x1, y1, LINE, ' marker-start="url(#arr)"' if both else ''))
        return s + label_box(lx if lx is not None else (x0 + x1) / 2, ly if ly is not None else (y0 + y1) / 2 - 10,
                             [lab], 12, MAG)

    def sub(k):
        s = stats.get(k)
        return ['%d units, %d functions' % (s['units'], s['functions']), '%d state machines, %s C lines'
                % (s['fsms'], '{:,}'.format(s['c_lines']))] if s else []

    b = []
    b.append(box(40, 40, 230, 96, 'RC transmitter', ['and the RC receiver', '(the plant)'], 'white', GREY, GREY))
    b.append(box(40, 330, 230, 96, 'ESC L and ESC R', ['the four motors and brakes', '(the plant; state machine)'],
                 'white', GREY, GREY))
    b.append(box(370, 170, 300, 110, 'MCU firmware', ['STM32H743, tasks of 1 to 100 ms'] + sub('mcu')))
    b.append(box(370, 470, 300, 100, 'BMS firmware', ['STM32G4'] + sub('bms')))
    b.append(box(780, 40, 330, 104, 'GNC', ['companion (ROS 2)'] + sub('gnc')))
    b.append(box(780, 300, 330, 104, 'NavFusion and Perception', ['companion (ROS 2)'] + sub('nav')))
    b.append(box(800, 480, 290, 80, 'Sensors', ['IMU, RTK GNSS, 2-D and 3-D LiDAR,', 'odometry (the plant)'],
                 'white', GREY, GREY))
    b.append(link(270, 88, 370, 200, 'SBUS', lx=330, ly=128))
    b.append(link(370, 260, 270, 360, 'CAN1', True, lx=330, ly=300))
    b.append(link(520, 280, 520, 470, 'CAN1', True, lx=548, ly=380))
    b.append(link(670, 205, 780, 100, 'CAN2', True, lx=730, ly=140))
    b.append(link(945, 300, 945, 144, 'state', lx=980, ly=222))
    b.append(link(945, 480, 945, 404, 'scans, fixes', lx=990, ly=446))
    b.append(link(670, 250, 780, 340, 'CAN2', True, lx=730, ly=310))
    return write_svg('sdd_overview', 1150, 600, ''.join(b))


# ----------------------------------------------------------------------------------------------------- C locations
class Traces:
    def __init__(self):
        self.cache = {}

    def get(self, inv, model):
        key = model
        if key in self.cache:
            return self.cache[key]
        tr = None
        for c in inv.get('code', []):
            if c.get('model') == model and c.get('trace') and os.path.isfile(os.path.join(ROOT, c['trace'])):
                tr = load(os.path.join(ROOT, c['trace']))
                break
        if tr is None:
            p = os.path.join(T07, 'trace', model + '.json')
            tr = load(p) if os.path.isfile(p) else None
        if tr is not None:
            fb = defaultdict(set)
            for path, rows in tr['blocks'].items():
                for r in rows:
                    fb[(r['file'], r['function'])].add(path)
            tr['_fb'] = fb
            tr['_fn'] = {(f['file'], f['name']): f for f in tr['functions']}
        self.cache[key] = tr
        return tr


def under(path, prefixes):
    return any(path == p or path.startswith(p + '/') for p in prefixes)


def c_location(tr, blocks, mfile=None):
    """The C functions and lines of a function: by its own MATLAB source file when the trace maps that file's lines
    (so functions that share one block, such as the NavEKF's prediction and updates, each get their own C), else by
    its blocks. A dedicated C function is given whole. The place with the most traced lines comes first, so an inlined
    MATLAB Function leads with its step code, not with a helper (review findings T07-CHK-02 and -04)."""
    if tr is None:
        return 'no C (the model did not generate)', []
    blocks = [b for b in blocks if b]
    ext = tr.get('external_lines') or {}
    mkeys = [os.path.splitext(os.path.basename(str(m)))[0] for m in (mfile if isinstance(mfile, list) else [mfile])
             if m]
    mkeys = [k for k in mkeys if k in ext]
    hits = {}
    if mkeys:
        for k in mkeys:
            for rows in ext[k].values():
                for r in rows:
                    key = (r['file'], r['function'])
                    a, b, n = hits.get(key, (r['line'], r['line'], 0))
                    hits[key] = (min(a, r['line']), max(b, r['line']), n + 1)
    else:
        for path, rows in tr['blocks'].items():
            if under(path, blocks):
                for r in rows:
                    k = (r['file'], r['function'])
                    a, b, n = hits.get(k, (r['first'], r['last'], 0))
                    hits[k] = (min(a, r['first']), max(b, r['last']), n + r['refs'])
    if not hits:
        return 'not found in the C trace', []
    items = []
    for (f, fn), (a, b, n) in hits.items():
        own = fn != '(file scope)' and not STEP_NAMES.search(fn) and (f, fn) in tr['_fn']
        if mkeys:
            whole = own and any(k in fn for k in mkeys)
        else:
            whole = own and all(under(p, blocks) for p in tr['_fb'][(f, fn)])
        if whole:
            g = tr['_fn'][(f, fn)]
            items.append((f, fn, g['first'], g['last'], True, n))
        else:
            items.append((f, fn, a, b, False, n))
    items.sort(key=lambda it: (it[1] == '(file scope)', -it[5], not it[4]))  # a real C function first, the most
    # traced lines first among them, a C function of its own on a tie; file-scope code (data, or a static helper
    # the trace could not name) last
    items = [it[:5] for it in items]
    parts = []
    for f, fn, a, b, whole in items[:4]:
        where = 'data' if fn == '(file scope)' else '%s()' % fn
        parts.append('%s: %s, lines %d to %d%s' % (f, where, a, b, ' (the whole function)' if whole else ''))
    text = '; '.join(parts) + ('; and %d more places' % (len(items) - 4) if len(items) > 4 else '')
    return text, items


def short_block(b, unit_name):
    """The block path without an in-memory harness prefix (cg_<unit>/<unit>/... reads <unit>/...)."""
    if isinstance(b, list):
        return [short_block(x, unit_name) for x in b]
    m = re.match(r'^cg_[^/]+/(.*)$', b or '')
    return m.group(1) if m else b


# ------------------------------------------------------------------------------------------------------- chapters
class Writer:
    def __init__(self):
        self.out = []
        self.figs = 0
        self.tabs = 0

    def __call__(self, s=''):
        self.out.append(s)

    def table(self, caption, header, rows):
        if not rows:
            return
        self.tabs += 1                      # the builder numbers the captions itself; it needs a digit here
        self.out.append('Table %d. %s' % (self.tabs, caption))
        self.out.append('')
        self.out.append('| ' + ' | '.join(header) + ' |')
        self.out.append('|' + '---|' * len(header))
        for r in rows:
            self.out.append('| ' + ' | '.join(cell(c) for c in r) + ' |')
        self.out.append('')

    def figure(self, caption, path):
        if path and os.path.isfile(os.path.join(ROOT, path)):
            self.figs += 1
            self.out.append('![Figure %d: %s](%s)' % (self.figs, caption, path))
            self.out.append('')


def fsm_section(w, fsm, part_key, cov, heading='####'):
    fid = fsm.get('id', 'FSM')
    seen = None
    if cov and fid in cov.get('fsm', {}):
        seen = {t for t, scn in cov['fsm'][fid].items() if scn}
    w('%s %s: %s' % (heading, fid, fsm.get('name', '')))
    w('')
    src = fsm.get('source', '')
    blk = fsm.get('block')
    txt = 'The state machine is in `%s`' % src
    if blk:
        txt += ', in the block `%s`' % (', '.join(blk) if isinstance(blk, list) else blk)
    txt += '. It has %d states and %d transitions; it starts in %s.' % (
        len(fsm.get('states', [])), len(fsm.get('transitions', [])), fsm.get('initial'))
    if fsm.get('state_signal'):
        txt += ' In the runs its state is the signal %s.' % fsm['state_signal'].rstrip('.')
    w(txt)
    w('')
    if fsm.get('note'):
        w(str(fsm['note']))
        w('')
    path = state_diagram(fsm, 'fsm_' + re.sub(r'\W+', '_', fid), seen)
    cap = 'The state machine %s (%s). Each arrow is labelled with its transition ids; the guards are in the ' \
          'table below.' % (fid, fsm.get('name', ''))
    if seen is not None:
        cap += ' A dashed arrow was not seen in the runs; on a solid arrow, an id marked * was not seen.'
    w.figure(cap, path)
    w.table('The states of %s.' % fid, ['State', 'Value', 'Meaning', 'Outputs'],
            [[s.get('name'), s.get('value', ''), s.get('meaning', ''), s.get('outputs', '')]
             for s in fsm.get('states', [])])
    rows = []
    for t in fsm.get('transitions', []):
        r = [t.get('id'), t.get('from'), t.get('to'), t.get('guard', ''), t.get('action', ''), t.get('line', ''),
             joinmax(t.get('tests', []), 2)]
        if seen is not None:
            r.append(joinmax(cov['fsm'][fid].get(str(t.get('id')), []), 4) or 'not seen')
        rows.append(r)
    head = ['Id', 'From', 'To', 'Guard', 'Action', 'Line', 'Tests'] + (['Seen in the runs'] if seen is not None else [])
    w.table('The transitions of %s, each with its guard as the code reads and its line in %s.'
            % (fid, os.path.basename(src)), head, rows)


def part_chapter(w, num, title, inv, traces, cov, stats_out, key):
    w('## %d. %s' % (num, title))
    w('')
    w(inv.get('summary', ''))
    w('')
    if inv.get('target'):
        w('**Where it runs:** %s.' % str(inv['target']).rstrip('.'))
        w('')
    code_rows = []
    c_lines = 0
    for c in inv.get('code', []):
        tr = traces.get(inv, c.get('model'))
        lines = sum(f['lines'] for f in tr['files']) if tr else c.get('c_lines', '')
        if isinstance(lines, int):
            c_lines += lines
        code_rows.append([c.get('model'), tr['code_dir'] if tr else c.get('dir', ''),
                          len(tr['files']) if tr else c.get('files', ''),
                          '{:,}'.format(lines) if isinstance(lines, int) else lines, c.get('generated', ''),
                          c.get('status', '')])
    w.table('The C code of the %s, generated from the saved models for this document.' % title.split(' on ')[0],
            ['Model', 'Code folder', 'Files', 'C lines', 'Generated', 'Status'], code_rows)
    if any(str(r[1]).startswith('work/team/t07/code/') for r in code_rows):
        w("The code folders above are copies of the code as it was traced, kept with this document: later builds "
          "rewrite the project's code generation folders under `work/`, and the lines quoted here would move.")
        w('')
    for c in inv.get('code', []):                   # a copy generated again after its folder was rewritten
        tr = traces.get(inv, c.get('model'))
        if not tr:
            continue
        mainc = os.path.join(ROOT, tr['code_dir'], '%s.c' % tr.get('model', c.get('model')))
        traced = str(tr.get('info', {}).get('generated_on', ''))
        if not (os.path.isfile(mainc) and traced):
            continue
        with open(mainc, encoding='utf-8', errors='replace') as fh:
            head = ''.join(next(fh, '') for _ in range(20))
        m = re.search(r'generated on\s*:\s*(.+)', head)
        if m and m.group(1).strip() != traced:
            w('The copy of %s was generated again on %s, with the same model and settings, after a later build had '
              'rewritten its folder; it matches the trace of %s line for line, every traced line in place.'
              % (c.get('model'), m.group(1).strip(), traced))
            w('')
    settings = sorted({c.get('settings') for c in inv.get('code', []) if c.get('settings')})
    if settings:
        w('**The generation settings,** set in memory and never saved: %s' % settings[0].rstrip('.') + '.')
        w('')
    nfun = 0
    for u in inv.get('units', []):
        nfun += len(u.get('functions', []))
        tr = traces.get(inv, u.get('model'))
        w('### %s %s (%s)' % (u.get('id'), u.get('title', ''), u.get('name', '')))
        w('')
        w(u.get('purpose', ''))
        w('')
        facts = []
        if u.get('rate'):
            facts.append('**Rate:** %s.' % str(u['rate']).rstrip('.'))
        if u.get('source'):
            facts.append('**Source:** `%s`.' % str(u['source']).rstrip('.'))
        if u.get('standalone'):
            facts.append('**Its C:** %s' % str(u['standalone']).rstrip('.') + '.')
        if facts:
            w(' '.join(facts))
            w('')
        ins, outs_ = u.get('inputs', []), u.get('outputs', [])

        def as_group(rows, key):
            vals = {str(r.get(key, '')) for r in rows}
            v = vals.pop() if len(rows) > 1 and len(vals) == 1 else ''
            return v if (',' in v or ' and ' in v) else ''      # one text that names several places
        g_in, g_out = as_group(ins, 'from'), as_group(outs_, 'to')
        said = []
        if g_in:
            said.append('the inputs come from %s' % g_in.rstrip('.'))
        if g_out:
            said.append('the outputs go to %s' % g_out.rstrip('.'))
        if said:
            w("The inventory names where this unit's signals come from and go to for the unit as a whole, not signal "
              "by signal: %s." % '; '.join(said))
            w('')
        io = [['in', i.get('name'), i.get('type', ''), 'as above' if g_in else i.get('from', ''), i.get('icd', '')]
              for i in ins]
        io += [['out', o.get('name'), o.get('type', ''), 'as above' if g_out else o.get('to', ''), o.get('icd', '')]
               for o in outs_]
        w.table('The inputs and outputs of %s.' % u.get('name'), ['Way', 'Signal', 'Type', 'From or to', 'ICD'], io)
        if u.get('screenshot'):
            w.figure('The Simulink diagram of %s, as saved.' % u.get('name'), u['screenshot'])
        fpath = flowchart(u.get('flow', {}), 'flow_' + re.sub(r'\W+', '_', str(u.get('id'))))
        w.figure('The flowchart of %s (%s): its logic, step by step.' % (u.get('id'), u.get('title', '')), fpath)
        w('**The functions.**')
        w('')
        for f in u.get('functions', []):
            blocks = f.get('block') if isinstance(f.get('block'), list) else [f.get('block')]
            loc, _ = c_location(tr, blocks, f.get('mfile'))
            sb = short_block(blocks, u.get('name'))
            w('**%s %s** (%s). %s' % (f.get('id'), f.get('name'), f.get('kind', ''), str(f.get('does', '')).strip()))
            w('')
            items = ['*Block:* `%s`' % joinmax(sb, 3, 'more blocks'),
                     '*C code* (in `%s`): %s' % (tr['code_dir'] if tr else '-', loc)]
            if f.get('mfile'):
                items.append('*MATLAB source:* `%s`' % f['mfile'])
            items.append('*Tests:* %s' % (joinmax(f.get('tests', []), 4) or 'none (a gap; see the gaps below)'))
            if f.get('requirements'):
                items.append('*Requirements:* %s' % ', '.join(f['requirements']))
            for it in items:
                w('- ' + it)
            w('')
    for fsm in inv.get('fsms', []):
        if key == 'modes':
            continue
        if fsm is inv['fsms'][0]:
            w('### The state machines')
            w('')
        fsm_section(w, fsm, key, cov)
    if inv.get('gaps'):
        w('### The gaps')
        w('')
        for g in inv['gaps']:
            w('- ' + cell(g))
        w('')
    stats_out[key] = {'units': len(inv.get('units', [])), 'functions': nfun, 'fsms': len(inv.get('fsms', [])),
                      'c_lines': c_lines}


def modes_chapter(w, num, inv, cov):
    sm = inv.get('system_modes', {})
    w('## %d. The system states and modes' % num)
    w('')
    w(inv.get('summary', ''))
    w('')
    w('### %d.1 The modes of the robot' % num)
    w('')
    for m in sm.get('modes', []):
        w('**%s.** %s' % (m.get('name'), str(m.get('meaning', '')).strip()))
        w('')
        rows = [[k, v] for k, v in (m.get('states') or {}).items()]
        if rows:
            w.table('The state of each machine in the mode %s.' % m.get('name'), ['Machine', 'State'], rows)
    w('### %d.2 The event chains' % num)
    w('')
    w('Each chain is the order in which the machines change state after one event, with the signal that carries '
      'each change and the time it takes.')
    w('')
    for c in sm.get('chains', []):
        req = c.get('requirement')
        req = ', '.join(req) if isinstance(req, list) else (req or '')
        w('**%s.**%s' % (str(c.get('event')).rstrip('.'), (' Requirements: %s.' % req) if req else ''))
        w('')
        w.table('The chain after: %s.' % str(c.get('event')).rstrip('.'),
                ['Step', 'Machine', 'From', 'To', 'Carried by', 'Within (s)'],
                [[i + 1, s.get('machine'), s.get('from', ''), s.get('to'), s.get('signal', ''), s.get('within_s', '')]
                 for i, s in enumerate(c.get('steps', []))])
        if c.get('note'):
            w(str(c['note']))
            w('')
    w('### %d.3 The state signals in the runs' % num)
    w('')
    w.table('The logged signal that holds each machine\'s state in the system runs.', ['Machine', 'Signal', 'How it is read'],
            [[s.get('machine'), s.get('signal'), s.get('how_read', '')] for s in sm.get('state_signals', [])])
    if cov and cov.get('timelines'):
        w('### %d.4 The modes in the runs' % num)
        w('')
        for tl in cov['timelines']:
            w.figure(tl.get('caption', 'The states over time in %s.' % tl.get('scenario', '')), tl.get('figure'))
    for fsm in inv.get('fsms', []):
        w('### %d.%d %s' % (num, 5 if cov and cov.get('timelines') else 4, fsm.get('name', fsm.get('id'))))
        w('')
        fsm_section(w, fsm, 'modes', cov)


def frames_chapter(w, num, inv):
    w('## %d. The frames: sending and receiving' % num)
    w('')
    w(inv.get('summary', ''))
    w('')
    frames = inv.get('frames', [])
    w.table('The frames of the embedded software.', ['Id', 'Frame', 'Bus and identifier', 'Direction', 'Rate',
                                                     'ICD match', 'Tests'],
            [[f.get('id'), f.get('name'), '%s %s' % (str(f.get('bus', '')).split('(')[0].strip(), f.get('frame_id', '')),
              f.get('direction'), f.get('rate'), str(f.get('icd_match', '')).split(':')[0],
              len(f.get('tests', []))] for f in frames])
    for f in frames:
        w('### %s %s' % (f.get('id'), f.get('name')))
        w('')
        snd, rcv = f.get('sender') or {}, f.get('receiver') or {}
        w('**%s**, %s, %s. **Sent by** %s (`%s`); **received by** %s (`%s`). **Time-out:** %s. **Checks:** %s'
          % (f.get('direction'), f.get('bus'), (('identifier %s' % f['frame_id']) if f.get('frame_id') else
                                                 'no identifier'),
             snd.get('part', ''), short_block(snd.get('block', ''), ''), rcv.get('part', ''),
             short_block(rcv.get('block', ''), ''), str(f.get('timeout', '')).rstrip('.'),
             str(f.get('checks', '')).rstrip('.') + '.'))
        w('')
        w.table('The byte layout of %s.' % f.get('name'), ['Bytes', 'Bits', 'Field', 'Type', 'Scale', 'Offset', 'Unit',
                                                          'Value'],
                [[r.get('bytes', ''), r.get('bits', ''), r.get('field'), r.get('type', ''), r.get('scale', ''),
                  r.get('offset', ''), r.get('unit', ''), r.get('value', '')] for r in f.get('layout', [])])
        if f.get('flow', {}).get('nodes'):
            w.figure('%s from the sender to the receiver.' % f.get('name'),
                     flowchart(f['flow'], 'frame_' + re.sub(r'\W+', '_', f.get('id', ''))))
        w('**Against the ICD:** %s. **Tests:** %s.' % (str(f.get('icd_match', '')).rstrip('.'),
                                                     joinmax(f.get('tests', []), 6) or 'none'))
        w('')


def verification_chapter(w, num, invs, cov):
    w('## %d. The verification of the code' % num)
    w('')
    w('### %d.1 The tests named for the functions' % num)
    w('')
    rows = []
    for key, title in PARTS:
        inv = invs.get(key)
        if not inv:
            continue
        funs = [f for u in inv.get('units', []) for f in u.get('functions', [])]
        trs = [t for s in inv.get('fsms', []) for t in s.get('transitions', [])]
        tests = ({x for f in funs for x in f.get('tests', [])} | {x for tr_ in trs for x in tr_.get('tests', [])}
                 | {x for fr in inv.get('frames', []) for x in fr.get('tests', [])})
        rows.append([title, len(funs), sum(1 for f in funs if f.get('tests')), len(trs),
                     sum(1 for t in trs if t.get('tests')), len(tests)])
    w.table('The functions and transitions with a test, by part.', ['Part', 'Functions', 'With a test', 'Transitions',
                                                                     'With a test', 'Distinct tests named'], rows)
    w('### %d.2 The generated C against the model (SIL)' % num)
    w('')
    sil = [load(p) for p in sorted(glob.glob(os.path.join(T07, 'sil', '*.json')))]
    if sil:
        w('Each record runs the same stimulus in normal mode and in software-in-the-loop mode, where the generated C '
          'is compiled and run as a program, and compares every root output: integers, booleans and enumerations '
          'bit-exact, floating point within the tolerance stated for each signal.')
        w('')
        srows = []
        for s in sil:
            outs = s.get('outputs', []) or []
            worst = ''
            if outs:
                def ratio(o):
                    try:
                        return float(o.get('max_abs_err') or 0) / max(float(o.get('tol') or 0), 1e-300)
                    except (TypeError, ValueError):
                        return 0
                o = max(outs, key=ratio)
                worst = '%s: %s (tolerance %s)' % (o.get('name'), o.get('max_abs_err'), o.get('tol'))
            srows.append([s.get('model'), s.get('status'), s.get('stimulus', ''), len(outs), worst, s.get('wall_s', '')])
        w.table('The SIL back-to-back records.', ['Model', 'Result', 'Stimulus', 'Outputs', 'Largest error',
                                                  'Wall time (s)'], srows)
        spec = [o for s in sil for o in (s.get('outputs', []) or [])
                if str(o.get('rule', '')).startswith(('covariance:', 'nis:', 'P diagonal:'))]
        if spec:
            top = max(float(o.get('max_abs_err') or 0) for o in spec)
            w('The NavFusion and Perception records use three special rules for %d outputs, with their reasons in '
              '`tests/sensors/t_t07_nav_sil.m` (lines 20 to 24). A covariance element is held within 1e-7 x '
              'sqrt(|C_ii C_jj|), and a diagonal element of P within 1e-7 x |reference|: P spans about seven decades, '
              'so a bound relative to each element would fail on the smallest ones, and an absolute bound would not '
              "test them. The gate's NIS is held within 1e-6 x max(1, |reference|): it is a quadratic form of the "
              'innovation, a difference of nearly equal numbers taken through the inverse of S. The largest of these '
              'errors is %.2g, below 1e-9, so each is also inside the default rule for a double.' % (len(spec), top))
            w('')
    else:
        w('No SIL record exists yet: the SIL runs follow in the campaign (card T-07, step 9).')
        w('')
    w('### %d.3 The full test run' % num)
    w('')
    frows = []
    done = set()
    pat = re.compile(r'^== (.+): total (\d+), passed (\d+), failed (\d+), incomplete (\d+)(?:, (\d+) s)?')
    for sub, run in (('full', 'full run'), ('rerun', 'step 9')):   # step 9: the T-07 classes written after the
        for lg in sorted(glob.glob(os.path.join(T07, 'logs', sub, '*.log'))):   # full run fixed its class list
            cls = os.path.splitext(os.path.basename(lg))[0]
            if cls in done:
                continue
            for line in open(lg, encoding='utf-8', errors='replace'):
                m = pat.match(line)
                if m:
                    frows.append([cls, run] + [m.group(i) or '' for i in range(2, 7)])
                    done.add(cls)
                    break
    if frows:
        w.table('The test runs: every test class in the full run (the scenario reruns on), and the six T-07 classes '
                'written after the full run fixed its class list, run in step 9.',
                ['Class', 'Run', 'Total', 'Passed', 'Failed', 'Incomplete', 'Time (s)'], frows)
    else:
        w('The full run of every test class has not finished yet (card T-07, step 8); its result is in the '
          'engineering report NASL-SHD-ER-007.')
        w('')
    if cov and cov.get('fsm'):
        w('### %d.4 The transitions seen in the runs' % num)
        w('')
        crows = []
        for fid, tmap in cov['fsm'].items():
            n = len(tmap)
            k = sum(1 for v in tmap.values() if v)
            crows.append([fid, n, k, '%.0f %%' % (100.0 * k / n) if n else ''])
        w.table('The transitions of each state machine that the runs exercised.', ['State machine', 'Transitions',
                                                                                    'Seen', 'Share'], crows)


def archived(text):
    """A code folder under work/ that a finding cites reads as its archived copy, the code as traced (T07-CHK-01)."""
    def sub(m):
        name = m.group(1)
        return 'work/team/t07/code/' + name if os.path.isdir(os.path.join(T07, 'code', name)) else m.group(0)
    return re.sub(r'(?<![\w/])work/([A-Za-z0-9_]+_ert_rtw)\b', sub, str(text))


def findings_chapter(w, num, invs):
    w('## %d. The findings' % num)
    w('')
    w('The engineers recorded these while they generated, traced and tested the code. None was fixed in this work: '
      'a defect goes to the project owner, and a change to the software\'s behaviour needs the owner\'s approval.')
    w('')
    rows = []
    for key, title in PARTS:
        for f in (invs.get(key) or {}).get('findings', []):
            what = f.get('what') or ''
            stat = str(f.get('status') or '').strip()
            if stat:                                  # e.g. a test gap closed in pass B
                what = '%s %s%s.' % (what, stat[0].upper(), stat[1:].rstrip('.'))
            rows.append((SEV_ORDER.get(str(f.get('severity', '')).split(' ')[0], 4), f.get('id', ''),
                         [f.get('id'), f.get('severity'), archived(what), archived(f.get('evidence', ''))]))
    rows.sort(key=lambda r: (r[0], r[1]))
    w.table('The findings, the defects first.', ['Id', 'Severity', 'What', 'Evidence'], [r[2] for r in rows])


def appendix_trace(w, invs, traces):
    w('## Appendix A. From each function to its C code')
    w('')
    w('One row per function: the C function that holds most of its traced code, with its lines, and how many other '
      'places hold the rest; the number of tests that name it; and its requirements. The chapters give every place.')
    w('')
    for key, title in PARTS:
        inv = invs.get(key)
        if not inv or not inv.get('units'):
            continue
        rows = []
        for u in inv['units']:
            tr = traces.get(inv, u.get('model'))
            for f in u.get('functions', []):
                blocks = f.get('block') if isinstance(f.get('block'), list) else [f.get('block')]
                _, items = c_location(tr, blocks, f.get('mfile'))
                main = items[0] if items else None
                more = (' and %d more' % (len(items) - 1)) if len(items) > 1 else ''
                rows.append([f.get('id'), f.get('name'), main[0] if main else '-',
                             ((main[1] + '()' if main[1] != '(file scope)' else 'file scope') + more) if main
                             else ('no C' if tr is None else 'not found'),
                             ('%d to %d' % (main[2], main[3])) if main else '',
                             len(f.get('tests', [])), ', '.join(f.get('requirements', []))])
        w.table('%s: each function and its C code.' % title, ['Function', 'Name', 'C file', 'C function', 'Lines',
                                                              'Tests', 'Requirements'], rows)


def appendix_files(w, invs, traces):
    w('## Appendix B. The C files')
    w('')
    for key, title in PARTS:
        inv = invs.get(key)
        if not inv:
            continue
        for c in inv.get('code', []):
            tr = traces.get(inv, c.get('model'))
            if not tr:
                continue
            fun = defaultdict(int)
            for f in tr['functions']:
                fun[f['file']] += 1
            w.table('The files of %s (`%s`).' % (c.get('model'), tr['code_dir']), ['File', 'Lines', 'C functions'],
                    [[f['name'], f['lines'], fun.get(f['name'], 0)] for f in tr['files']])


# ------------------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--date', default=datetime.date.today().isoformat())
    ap.add_argument('--version', default='1.0')
    ap.add_argument('--status', default='Draft')
    ap.add_argument('--no-pdf', action='store_true')
    a = ap.parse_args()
    invs = {}
    for key, _ in PARTS:
        p = os.path.join(T07, 'inv', key + '.json')
        if os.path.isfile(p):
            invs[key] = load(p)
    covp = os.path.join(T07, 'cov', 'coverage.json')
    cov = load(covp) if os.path.isfile(covp) else None
    traces = Traces()
    w = Writer()
    stats = {}
    body = Writer()
    body.figs = 0
    # the chapters first, so the summary can count them
    chapters = Writer()
    n = 3
    if 'modes' in invs:
        modes_chapter(chapters, n, invs['modes'], cov)
        n += 1
    if 'frames' in invs:
        frames_chapter(chapters, n, invs['frames'])
        n += 1
    for key, title in PARTS[2:]:
        if key in invs:
            part_chapter(chapters, n, title, invs[key], traces, cov, stats, key)
            n += 1
    verification_chapter(chapters, n, invs, cov)
    findings_chapter(chapters, n + 1, invs)
    appendix_trace(chapters, invs, traces)
    appendix_files(chapters, invs, traces)
    nunits = sum(s['units'] for s in stats.values())
    nfun = sum(s['functions'] for s in stats.values())
    nfsm = sum(len(i.get('fsms', [])) for i in invs.values())
    ntr = sum(len(f.get('transitions', [])) for i in invs.values() for f in i.get('fsms', []))
    nlines = sum(s['c_lines'] for s in stats.values())
    nframes = len(invs.get('frames', {}).get('frames', []))
    nfind = sum(len(i.get('findings', [])) for i in invs.values())
    ndef = sum(1 for i in invs.values() for f in i.get('findings', []) if str(f.get('severity', '')).startswith('defect'))
    file = 'NASL_SDD-002_Shaddad_Embedded_Software_Design_%s_v%s' % (a.date, a.version)
    summary = ('How the robot\'s embedded software works, function by function: the MCU firmware, the BMS firmware, '
               'and the GNC, navigation and perception software of the companion. %d units and %d functions, each '
               'tied to its generated C (%s lines), its tests and its requirements; %d state machines with %d '
               'transitions; %d frames from sender to receiver; the system modes and the event chains.'
               % (nunits, nfun, '{:,}'.format(nlines), nfsm, ntr, nframes))
    head = OrderedDict([
        ('doc_no', 'NASL-SHD-SDD-002'), ('doc_type', 'SOFTWARE DESIGN DESCRIPTION'),
        ('title', 'Shaddad UGV: embedded software design description'),
        ('subtitle', 'Each function of the embedded software, its C code, its flowchart, its states and its tests'),
        ('summary', summary), ('version', a.version), ('date', a.date), ('status', a.status),
        ('history', '%s | %s | First issue (card T-07)' % (a.version, a.date)),
        ('prepared_by', 'the lead systems engineer, with the MCU firmware, power and BMS, GNC, sensors and navigation, '
                        'electronics and comms, and integration engineers'),
        ('reviewer', 'the independent reviewer'), ('approver', 'the project owner'),
        ('entity', 'NASL Technologies (Robotat)'), ('classification', 'Internal'),
        ('footer', 'Shaddad UGV Embedded Software Design'), ('file', file), ('outdir', 'work/reports/design'),
        ('chapter_breaks', 'yes'), ('cover_image', 'shaddad'),
        ('request', 'The project owner, 2026-11-22: test the embedded software and its C code, and document every '
                    'function with its C code, the flowcharts, and the states and modes of the system (card T-07)'),
        ('scope', 'The embedded software of the twin: the MCU firmware, the BMS firmware, GNC, NavFusion and '
                  'Perception. Not the plant models, except the sending side of the frames; not the STM32 board '
                  'builds'),
        ('related', 'NASL-SHD-REQ-008 (the requirements); NASL-SHD-ICD-001 (the interfaces); NASL-SHD-SDD-001 (the '
                    'system design description, planned); NASL-SHD-ER-007 (the campaign report); card T-07')])
    w('<!--')
    for k, v in head.items():
        w('%s: %s' % (k, v))
    w('-->')
    w('# Shaddad UGV: embedded software design description')
    w('')
    w('## Summary')
    w('')
    w(summary)
    w('')
    w('- **Generated, not written by hand.** The text of each function comes from its owner\'s inventory, and each C '
      'location from the trace comments that Embedded Coder writes into the code, so the document is rebuilt '
      'whenever the model changes.')
    w('- **The state machines** are drawn from their code: every state, every transition with its guard and its '
      'source line, and the tests that exercise it.')
    w('- **%d findings** came out of the work, %d of them defects; chapter %d lists them. None was fixed here.'
      % (nfind, ndef, n + 1))
    w('')
    w('## 1. Purpose and scope')
    w('')
    w('This document explains how each part of the robot\'s embedded software works and where its code is. The '
      'project owner asked for it on 2026-11-22: "every function should be related to its c code and documented, '
      'the chartflow should be demonstrated and states and mode of the system". It covers the software that runs on '
      'the robot\'s controllers: the MCU firmware (STM32H743), the BMS firmware (STM32G4), and the GNC, NavFusion '
      'and Perception software of the companion computer. The plant models appear only where they send or receive '
      'a frame. The verification of the code is summarised in chapter %d, and the campaign that made it is reported '
      'in NASL-SHD-ER-007.' % n)
    w('')
    w('**How it is made.** Each software owner generated the C from the saved models, with the trace comments on, '
      'and listed each function of each unit in an inventory: what it does, its Simulink block, its MATLAB source, '
      'its tests and its requirements, with a flowchart of the unit and a table of each state machine. '
      '`tools/reports/codegen_trace.py` reads the trace comments of the generated C and maps every block to the C '
      'function and lines that implement it; `tools/reports/t07_check_inventory.py` checks each inventory against '
      'the trace, the tests and the requirements register; `tools/reports/build_sw_design.py` builds this document.')
    w('')
    w('**How to read a function.** Each function has an id (for example MCU-01-F1), a name and its kind (a '
      'subsystem, a MATLAB Function with its source file, a MATLAB System). Then: its Simulink block path; its C code, '
      'as the file, the C function and the lines (a C function that implements nothing else is given whole); its '
      'MATLAB source when it has one; the tests that name it; and the requirements it serves.')
    w('')
    w('## 2. The embedded software at a glance')
    w('')
    w.figs = chapters.figs  # numbered below by the builder; the index here is only for the markdown
    w.figure('The embedded software, its controllers and its links.', overview(stats))
    rows = []
    for key, title in PARTS[2:]:
        if key in invs:
            s = stats[key]
            rows.append([title, ', '.join(invs[key].get('models', [])[:3]) + (' and more' if len(invs[key].get(
                'models', [])) > 3 else ''), invs[key].get('target', ''), s['units'], s['functions'], s['fsms'],
                '{:,}'.format(s['c_lines'])])
    w.table('The parts of the embedded software.', ['Part', 'Models', 'Runs on', 'Units', 'Functions',
                                                    'State machines', 'C lines'], rows)
    out = w.out + chapters.out
    nt = nf = 0
    for i, line in enumerate(out):                 # the front part is written last: number every caption in order
        m = re.match(r'^Table \d+\. (.*)$', line)
        if m:
            nt += 1
            out[i] = 'Table %d. %s' % (nt, m.group(1))
            continue
        m = re.match(r'^!\[Figure \d+: (.*)$', line)
        if m:
            nf += 1
            out[i] = '![Figure %d: %s' % (nf, m.group(1))
    os.makedirs(SDD, exist_ok=True)
    src = os.path.join(SDD, 'SDD-002.md')
    with open(src, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out) + '\n')
    print('written %s: %d units, %d functions, %d state machines (%d transitions), %d frames, %d findings, '
          '%d SVG figures' % (rel(src), nunits, nfun, nfsm, ntr, nframes, nfind, len(glob.glob(os.path.join(FIG, '*.svg')))))
    if not a.no_pdf:
        r = subprocess.run([sys.executable, '-I', os.path.join(ROOT, 'tools', 'reports', 'build_document.py'), src],
                           cwd=ROOT)
        sys.exit(r.returncode)


if __name__ == '__main__':
    main()
