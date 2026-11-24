"""Write the engineering report of one task group (the Team tab) to work/agentic/reports/<task-id>.md.

Since 2026-11-16 (the project owner's rule): every report names the team by its roles only (the project owner, the
lead systems engineer, the structure engineer, ...), with effort in engineering passes and time; the roster that says
who fills each role is project.json "team". The lead's intro and the journal entries pass through team_text().

usage:
  python tools/dashboard/task_report.py <task-id> --journal "<journal task name>" [--journal ...]
                                        [--intro <markdown file>] [--ref <path> ...]

The user asked on 2026-11-14 for a report of every agent task in the dashboard's Reports section ("like a documenting
the task"), then "more like engineering report, include visual aid". The report:
  - a header table: report number, task, period, status, prepared by, reviewed by, effort;
  - the lead's body from --intro: Summary, 1 Background and objective, 2 Method, 3 Results (with numbered figures,
    images as `![Figure n: caption](path)`, paths relative to the project root), 4 Discussion, 5 Conclusions and
    recommendations;
  - Appendix A, the agent work: a timeline of the agent runs and the effort by agent (SVG figures drawn here, in
    work/agentic/reports/fig/, numbered after the body's figures) and the steps of the task;
  - Appendix B, every agent run with its model, time, tokens and result;
  - Appendix C, the journal entries of the task (the --journal names);
  - References: the journal entries' references and the --ref paths that exist.
Running it again rewrites the report and its figures from the current logs and the intro.
"""
import argparse
import datetime as dt
import json
import os
import re
import sys
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import agents  # noqa: E402

OUT = os.path.join(ROOT, 'work', 'agentic', 'reports')
FIG = os.path.join(OUT, 'fig')
JOURNAL = os.path.join(HERE, 'journal.jsonl')
KIND = {'start': 'Start', 'done': 'Done', 'result': 'Result', 'ruling': 'Ruling', 'decision': "Owner's decision",
        'needs-you': 'Needs you', 'resolved': 'Resolved', 'issue': 'Issue', 'note': 'Note', 'run': 'Run',
        'report': 'Report', 'change': 'Change'}
STATE = {'done': 'Done', 'working': 'Working', 'waiting': 'Waiting', 'blocked': 'Blocked', 'failed': 'Failed',
         'needs-you': 'Needs you', 'skipped': 'Skipped', 'running': 'In progress', 'queued': 'Queued'}
# figure colours (figures are drawn on white, like a printed report): one per role
ROLE = {'engineer': '#6722ba', 'reviewer': '#c86dd7', 'tester': '#0284c7', 'log-analyst': '#17277f',
        'lead': '#76718b', 'docs-keeper': '#b54708', 'report-writer': '#b54708'}
FONT = "font-family='Segoe UI, Arial, sans-serif'"


_PJ = os.path.join(ROOT, 'project.json')
TEAM = {}
if os.path.exists(_PJ):
    TEAM = {v: k for k, v in json.load(open(_PJ, encoding='utf-8')).get('team', {}).items()}
TEAM.update({'lead': 'lead systems engineer', 'you': 'project owner', 'user': 'project owner'})
LEGEND = {'engineer': 'subsystem engineer', 'reviewer': 'independent reviewer', 'tester': 'test engineer',
          'log-analyst': 'data analyst', 'lead': 'lead systems engineer', 'docs-keeper': 'documentation engineer',
          'report-writer': 'technical writer'}


def short(a):
    """The role that fills the work, e.g. 'shaddad-structure' -> 'structure engineer'."""
    a = a or ''
    return TEAM.get(a, TEAM.get(a.replace('shaddad-', ''), a.replace('shaddad-', '')))


def role(a):
    s = (a or '').replace('shaddad-', '')
    return s if s in ROLE else 'engineer'


_TT = [
    (r'\bthe lead \(the main Claude session\)', 'the lead systems engineer'),
    (r'\bthe lead \(main Claude session\)', 'the lead systems engineer'),
    (r'\bthe main (Claude )?session\b', 'the lead systems engineer'),
    (r'\bthe Shaddad agent team\b', 'the Shaddad engineering team'),
    (r'\bthe agent team\b', 'the engineering team'),
    (r'\b[Aa]gent task report\b', 'task report'),
    (r'\bAgents task\b', 'task group'), (r'\bAgents log\b', 'work log'), (r'\bthe Agents tab\b', 'the Team tab'),
    (r'\bagentic baseline\b', 'test baseline'),
    (r'\bagent runs?\b', lambda m: 'engineering passes' if m.group(0).endswith('s') else 'engineering pass'),
    (r'\bagent time\b', 'engineering time'), (r'\bsubagents?\b', 'engineers'),
    (r'\bengineer agents?\b', 'engineers'), (r'\bagents\b', 'engineers'), (r'\bagent\b', 'engineer'),
    (r'\bAgents\b', 'Engineers'), (r'\bAgent\b', 'Engineer'),
    # the work log's run notes: "step 11, own session: X" -> "step 11: X"; the session token note goes
    (r',\s*own session:\s*', ': '), (r'\bown session:\s*', ''),
    (r'\s*Tokens: the session total for [^.;|]*[.;]?', ''),
    # tokens before the model names, so that "(Opus, 230k tokens)" loses both
    (r',?\s*(?:about\s+)?[\d.]+\s*[kM]\s+(?:more\s+)?tokens?(?:\s+so far)?', ''),
    (r'\s*\((?:about\s+)?[\d.]+\s*[kM]?\s*(?:more\s+)?tokens?[^)]*\)', ''),
    (r'\btokens?\b', 'effort'),
    (r'\s*\((?:Opus|opus|Haiku|haiku|Sonnet|sonnet|Fable|fable)(?:,\s*[\d.]+\s*[kM])?\)', ''),
    (r'\b(Simulink|MATLAB) Agentic Toolkit\b', r'\1 toolkit'),
    (r',\s*all (?:Opus|opus|Haiku|haiku|Sonnet|sonnet|Fable|fable)\b', ''),
    (r',\s*(?:Opus|opus|Haiku|haiku|Sonnet|sonnet|Fable|fable)\b', ''),
    # MATLAB sessions are MATLAB instances; the user is the project owner; the specialists by their roles
    (r'\bfresh sessions\b', 'fresh MATLAB instances'),
    (r'\bMATLAB sessions\b', 'MATLAB instances'), (r'\bMATLAB session\b', 'MATLAB instance'),
    (r"\bthe user's\b", "the project owner's"), (r'\bthe user\b', 'the project owner'),
    (r"\bThe user's\b", "The project owner's"), (r'\bThe user\b', 'The project owner'),
    (r'\btesters\b', 'test engineers'), (r'\btester\b', 'test engineer'),
    (r'\bdocs[ -]keeper\b', 'documentation engineer'),
]


_PATHS = [('work/agentic/', 'work/team/'), ('tools/agentic/', 'tools/team/'),
          ('work\\agentic\\', 'work\\team\\'), ('tools\\agentic\\', 'tools\\team\\')]


def team_paths(s):
    """The team folders by their role names: work/team and tools/team (since 2026-11-16 both names reach the same
    folders, so a cited path works either way)."""
    for a, b in _PATHS:
        s = s.replace(a, b)
    return s


def team_text(s):
    """Name the team by its roles: agent names, sessions, models and tokens out (the owner's rule of 2026-11-16)."""
    s = s or ''
    for a, b in sorted(TEAM.items(), key=lambda kv: -len(kv[0])):
        if a.startswith('shaddad-'):
            s = re.sub(rf'\b{re.escape(a)}\b', f'the {b}', s)
    s = re.sub(r'\bthe the\b', 'the', s)
    for pat, rep in _TT:
        s = re.sub(pat, rep, s)
    return s


def tok(n):
    if not n:
        return ''
    return f'{n / 1e6:.2f} M' if n >= 1e6 else f'{round(n / 1000)}k'


def dur(s):
    if s is None:
        return ''
    s = round(s)
    if s < 60:
        return f'{s} s'
    if s < 3600:
        return f'{s // 60} min'
    return f'{s // 3600} h {round((s % 3600) / 60)} min'


def when(t):
    return (t or '')[:16].replace('T', ' ')


def cell(x):
    return str(x or '').replace('|', '\\|').replace('\n', ' ')


def ts(s):
    return dt.datetime.fromisoformat(s[:19])


def svg_timeline(runs, path):
    """Figure: the agent runs on a time axis, one row per run, coloured by role; failed or blocked runs outlined red."""
    rs = sorted([r for r in runs if r['start']], key=lambda r: r['start'])
    if not rs:
        return False
    now = dt.datetime.now()
    t0 = min(ts(r['start']) for r in rs)
    t1 = max(ts(r['end']) if r['end'] else now for r in rs)
    span = max((t1 - t0).total_seconds(), 600.0)
    W, L, R, top, rh = 960, 230, 30, 46, 26
    H = top + rh * len(rs) + 64
    def x(tt):
        return L + (W - L - R) * (tt - t0).total_seconds() / span
    steps = [300, 600, 900, 1800, 3600, 7200, 10800, 21600, 43200, 86400]
    step = next((s for s in steps if span / s <= 9), 86400)
    o = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}' {FONT}>",
         f"<rect width='{W}' height='{H}' fill='#ffffff'/>",
         f"<text x='{L}' y='22' font-size='14' font-weight='600' fill='#0d0b14'>Engineering passes over time</text>",
         f"<text x='{W - R}' y='22' font-size='11' fill='#4c4659' text-anchor='end'>{t0:%Y-%m-%d %H:%M} to {t1:%Y-%m-%d %H:%M}</text>"]
    tk = t0.replace(second=0, microsecond=0)
    tk += dt.timedelta(seconds=(-(tk - tk.replace(hour=0, minute=0)).total_seconds()) % step)
    while tk <= t1:
        xx = x(tk)
        lab = tk.strftime('%H:%M') if span < 86400 * 1.5 else tk.strftime('%d %b %H:%M')
        o.append(f"<line x1='{xx:.1f}' y1='{top - 6}' x2='{xx:.1f}' y2='{top + rh * len(rs)}' stroke='#e9e6f0'/>")
        o.append(f"<text x='{xx:.1f}' y='{top + rh * len(rs) + 16}' font-size='10.5' fill='#4c4659' text-anchor='middle'>{lab}</text>")
        tk += dt.timedelta(seconds=step)
    for k, r in enumerate(rs):
        y = top + rh * k
        a, b = ts(r['start']), ts(r['end']) if r['end'] else now
        x0, x1 = x(a), max(x(b), x(a) + 3)
        col = ROLE[role(r['agent'])]
        bad = r['status'] in ('failed', 'blocked')
        o.append(f"<text x='{L - 8}' y='{y + 16}' font-size='11.5' fill='#0d0b14' text-anchor='end'>{escape(short(r['agent']))}</text>")
        extra = (" stroke='#b42318' stroke-width='2'" if bad else '') + (" fill-opacity='0.55'" if r['status'] == 'working' else '')
        o.append(f"<rect x='{x0:.1f}' y='{y + 4}' width='{x1 - x0:.1f}' height='{rh - 9}' rx='3' fill='{col}'{extra}/>")
        lab = dur(r['seconds']) if r['seconds'] else ('running' if r['status'] == 'working' else '')
        if lab:
            inside = (x1 - x0) > 7 * len(lab)
            o.append(f"<text x='{(x0 + 6) if inside else (x1 + 5):.1f}' y='{y + 16}' font-size='10.5' "
                     f"fill='{'#ffffff' if inside else '#4c4659'}'>{escape(lab)}</text>")
    ly = top + rh * len(rs) + 40
    lx = L
    for name in [n for n in ROLE if any(role(r['agent']) == n for r in rs)]:
        o.append(f"<rect x='{lx}' y='{ly - 10}' width='12' height='12' rx='2' fill='{ROLE[name]}'/>")
        o.append(f"<text x='{lx + 17}' y='{ly}' font-size='11' fill='#0d0b14'>{escape(LEGEND.get(name, name))}</text>")
        lx += 30 + 7 * len(LEGEND.get(name, name))
    if any(r['status'] in ('failed', 'blocked') for r in rs):
        o.append(f"<rect x='{lx}' y='{ly - 10}' width='12' height='12' rx='2' fill='#ffffff' stroke='#b42318' stroke-width='2'/>")
        o.append(f"<text x='{lx + 17}' y='{ly}' font-size='11' fill='#0d0b14'>stopped or blocked</text>")
    o.append('</svg>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(o))
    return True


def svg_effort(runs, path):
    """Figure: the engineering time of the task by role, horizontal bars, with the number of passes."""
    agg = {}
    for r in runs:
        a = short(r['agent'])
        g = agg.setdefault(a, {'n': 0, 's': 0, 'role': role(r['agent'])})
        g['n'] += 1
        g['s'] += r['seconds'] or 0
    rows = sorted([(k, v) for k, v in agg.items() if v['s']], key=lambda kv: -kv[1]['s'])
    if not rows:
        return False
    W, L, R, top, rh = 960, 230, 170, 46, 30
    H = top + rh * len(rows) + 30
    mx = max(v['s'] for _, v in rows)
    o = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}' {FONT}>",
         f"<rect width='{W}' height='{H}' fill='#ffffff'/>",
         f"<text x='{L}' y='22' font-size='14' font-weight='600' fill='#0d0b14'>Engineering time by role</text>",
         f"<text x='{W - 20}' y='22' font-size='11' fill='#4c4659' text-anchor='end'>total {dur(sum(v['s'] for _, v in rows))}</text>"]
    for k, (a, v) in enumerate(rows):
        y = top + rh * k
        w = (W - L - R) * v['s'] / mx
        o.append(f"<text x='{L - 8}' y='{y + 18}' font-size='11.5' fill='#0d0b14' text-anchor='end'>{escape(a)}</text>")
        o.append(f"<rect x='{L}' y='{y + 5}' width='{max(w, 2):.1f}' height='{rh - 10}' rx='3' fill='{ROLE[v['role']]}'/>")
        o.append(f"<text x='{L + w + 6:.1f}' y='{y + 18}' font-size='11' fill='#4c4659'>"
                 f"{escape(dur(v['s']))} · {v['n']} pass{'es' if v['n'] > 1 else ''}</text>")
    o.append('</svg>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(o))
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('task')
    ap.add_argument('--journal', action='append', default=[])
    ap.add_argument('--intro')
    ap.add_argument('--ref', action='append', default=[])
    a = ap.parse_args()

    data = agents.read()
    task = next((t for t in data['tasks'] if t['id'] == a.task), None)
    if task is None:
        sys.exit(f'no task {a.task!r} in agents.jsonl')
    runs = [r for r in data['runs'] if r['task'] == a.task]
    names = set(a.journal)
    jr = []
    if os.path.exists(JOURNAL):
        with open(JOURNAL, encoding='utf-8') as f:
            for line in f:
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if e.get('task') in names:
                    jr.append(e)
    jr.sort(key=lambda e: e.get('at', ''))
    intro = ''
    if a.intro:
        with open(a.intro, encoding='utf-8') as f:
            intro = team_text(f.read().strip())

    times = [r['start'] for r in runs] + [r['end'] for r in runs if r['end']] + [e['at'] for e in jr] + [task['start']]
    if task.get('end'):
        times.append(task['end'])
    t0, t1 = min(times)[:10], max(times)[:10]
    tokens = sum(r['tokens'] or 0 for r in runs)
    secs = sum(r['seconds'] or 0 for r in runs)
    who = {}
    for r in runs:
        who[short(r['agent'])] = who.get(short(r['agent']), 0) + 1
    authors = ', '.join(k for k in who if k not in ('independent reviewer', 'lead systems engineer')) or 'the lead systems engineer'
    reviews = who.get('independent reviewer', 0)

    L = [f"# {team_text(task['title'])}", '',
         '| | |', '|---|---|',
         f"| Report | ATR-{task['id']} (task report), written {dt.datetime.now():%Y-%m-%d %H:%M} from the work log and the journal |",
         f"| Task | {cell(team_text(task['title']))} (`{task['id']}`) |",
         f"| Period | {t0}{' to ' + t1 if t1 != t0 else ''} |",
         f"| Status | {STATE.get(task['status'], task['status'])}; {sum(s['status'] == 'done' for s in task['steps'])} of {len(task['steps'])} steps done |",
         f"| Prepared by | the engineering team ({cell(authors)}) and the lead systems engineer |",
         f"| Reviewed by | {('the independent reviewer, ' + str(reviews) + ' review' + ('s' if reviews > 1 else '')) if reviews else 'no independent review in this task'} |",
         f"| Effort | {len(runs)} engineering pass{'es' if len(runs) != 1 else ''}, {dur(secs) or '0 s'} of engineering time |", '']
    if intro:
        L += [intro, '']

    # ---- Appendix A: the agent work, with the two figures numbered after the body's
    os.makedirs(FIG, exist_ok=True)
    nfig = len(re.findall(r'^!\[Figure', intro, flags=re.M))
    L += ['## Appendix A. The work by role', '']
    rel = lambda p: os.path.relpath(p, ROOT).replace('\\', '/')
    f1 = os.path.join(FIG, f"{task['id']}_timeline.svg")
    if svg_timeline(runs, f1):
        nfig += 1
        L += [f"![Figure {nfig}: the engineering passes of the task over time, one row per pass, coloured by role. The "
              f"label gives the pass's time; a red outline marks a pass that stopped or was blocked.]({rel(f1)})", '']
    f2 = os.path.join(FIG, f"{task['id']}_effort.svg")
    if svg_effort(runs, f2):
        nfig += 1
        L += [f"![Figure {nfig}: the engineering time by role, with the number of passes.]({rel(f2)})", '']
    L += ['Table A1. The steps of the task.', '', '| # | Who | What | State |', '|---|---|---|---|']
    for s in task['steps']:
        L.append(f"| {s['n']} | {cell(short(s['agent']))} | {cell(team_text(s['what']))} | {STATE.get(s['status'], s['status'])} |")
    L += ['', '## Appendix B. The engineering passes', '']
    if runs:
        L += ['| Started | Role | Work | Time | State | Result |', '|---|---|---|---|---|---|']
        for r in runs:
            L.append(f"| {when(r['start'])} | {cell(short(r['agent']))} | {cell(team_text(r['what']))} | "
                     f"{dur(r['seconds'])} | {STATE.get(r['status'], r['status'])} | {cell(team_text(r['summary']))} |")
    else:
        L.append('No engineering pass is logged for this task.')
    L += ['', '## Appendix C. The journal', '']
    if jr:
        L += ['| Time | Kind | Entry |', '|---|---|---|']
        for e in jr:
            L.append(f"| {when(e['at'])} | {KIND.get(e['kind'], e['kind'])} | {cell(team_text(e['text']))} |")
    else:
        L.append('No journal entries.')
    refs = []
    for p in a.ref + [e.get('ref') for e in jr]:
        if p and p not in refs and os.path.exists(os.path.join(ROOT, p)):
            refs.append(p)
    if refs:
        L += ['', '## References', '']
        L += [f'{k}. `{p}`' for k, p in enumerate(refs, 1)]
    out = os.path.join(OUT, f'{a.task}.md')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(team_paths('\n'.join(L)) + '\n')
    print(f'wrote {rel(out)}: {nfig} figures, {len(runs)} runs, {len(jr)} journal entries, {len(refs)} references')


if __name__ == '__main__':
    main()
