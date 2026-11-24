"""The dashboard's journal: one JSON object per line in journal.jsonl, oldest first.

Entries come from two places. People and tools add them with log.py (work started or done, results, rulings,
decisions, anything that needs the user). The dashboard server adds its own: after each reading of the project,
observe() compares it with the reading before (kept in .journal_state.json) and notes what changed, such as a MATLAB
run that started or ended, a new test result, a plan task that started or finished, a report written, model files
changed or a sleep of the computer.

Fields: at (local ISO time), kind, text; optional task, area, ref (a project path or a URL), by.
"""
import datetime as dt
import json
import os
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, 'journal.jsonl')
STATE = os.path.join(HERE, '.journal_state.json')
KINDS = ('start', 'done', 'result', 'ruling', 'decision', 'needs-you', 'resolved', 'issue', 'note', 'run', 'report',
         'change', 'sleep')
_lock = threading.Lock()


def now():
    return dt.datetime.now().isoformat(timespec='seconds')


def append(kind, text, at=None, **extra):
    if kind not in KINDS:
        raise ValueError(f'kind must be one of {", ".join(KINDS)}')
    e = {'at': at or now(), 'kind': kind, 'text': str(text).strip()}
    e.update({k: v for k, v in extra.items() if v})
    with _lock, open(PATH, 'a', encoding='utf-8') as f:
        f.write(json.dumps(e, ensure_ascii=False) + '\n')
    return e


def read(n=500):
    out = []
    try:
        with open(PATH, encoding='utf-8') as f:
            for ln in f:
                ln = ln.strip()
                if ln:
                    try:
                        out.append(json.loads(ln))
                    except ValueError:
                        continue
    except OSError:
        return []
    out.sort(key=lambda e: e.get('at', ''), reverse=True)
    return out[:n]


# ---- what the server notices between two readings
def _keys(snap):
    m = snap.get('machine') or {}
    tests = (snap.get('tests') or {}).get('classes') or []
    return {
        'runs': {str(p['pid']): {'command': p['command'], 'started': p['started']} for p in m.get('matlab') or [] if p['kind'] == 'batch'},
        'tests': {c['class']: c['latest'] for c in tests if c.get('latest')},
        'tasks': {f"{p['file']}#{t['n']}": {'status': t['status'], 'name': t['name'], 'plan': p['title']}
                  for p in snap.get('plans') or [] for t in p['tasks']},
        'reports': {r['file']: {'at': r['at'], 'title': r['title'], 'kind': r['kind']} for r in snap.get('reports') or []
                    if r['kind'] in ('Task report', 'Layout report')},
        'sleeps': {s['from']: s for s in m.get('sleeps') or []},
        'models': {f['file']: f['at'] for f in (snap.get('models') or {}).get('all') or []},
    }


def _changes(old, cur):
    out = []
    for pid, r in cur['runs'].items():
        if pid not in old.get('runs', {}):
            out.append(dict(kind='run', text=f"MATLAB batch run started: {r['command']}", by='dashboard'))
    for pid, r in old.get('runs', {}).items():
        if pid not in cur['runs']:
            mins = (dt.datetime.now() - dt.datetime.fromisoformat(r['started'])).total_seconds() / 60
            out.append(dict(kind='run', text=f"MATLAB batch run ended after {mins:.0f} min: {r['command']}", by='dashboard'))
    for cls, r in cur['tests'].items():
        o = old.get('tests', {}).get(cls)
        if not o or o.get('at') != r.get('at'):
            res = f"{r['passed']}/{r['total']} passed" + (f", {r['failed']} failed" if r['failed'] else '') + \
                  (f", {r['incomplete']} incomplete" if r['incomplete'] else '')
            out.append(dict(kind='result', text=f"{cls}: {res}" + (f" ({r['label']})" if r.get('label') else ''),
                            ref=r.get('log'), by='dashboard'))
    for key, t in cur['tasks'].items():
        o = old.get('tasks', {}).get(key)
        if o and o['status'] != t['status']:
            n = key.rsplit('#', 1)[1]
            if t['status'] == 'running':
                out.append(dict(kind='start', text=f"{t['plan']}: Task {n} started: {t['name']}", by='dashboard'))
            elif t['status'] == 'done':
                out.append(dict(kind='done', text=f"{t['plan']}: Task {n} done: {t['name']}", by='dashboard'))
    for f, r in cur['reports'].items():
        o = old.get('reports', {}).get(f)
        if not o or o['at'] != r['at']:
            out.append(dict(kind='report', text=f"{r['kind']} {'written' if not o else 'updated'}: {r['title']}", ref=f, by='dashboard'))
    for k, s in cur['sleeps'].items():
        if k not in old.get('sleeps', {}):
            out.append(dict(kind='sleep', at=s['from'], text=f"The computer slept {s['hours']} h (until {s['to'][11:16]}); "
                            'batch runs paused', by='dashboard'))
    changed = sorted(f for f, at in cur['models'].items() if old.get('models', {}).get(f) != at)
    if changed and old.get('models'):
        names = [os.path.splitext(os.path.basename(f))[0] for f in changed]
        more = f' and {len(names) - 6} more' if len(names) > 6 else ''
        out.append(dict(kind='change', text=f"{len(names)} model file{'s' if len(names) > 1 else ''} changed: "
                        f"{', '.join(names[:6])}{more}", by='dashboard'))
    return out


def observe(snap):
    """Log what changed since the last reading. The first reading only records the state."""
    cur = _keys(snap)
    try:
        with open(STATE, encoding='utf-8') as f:
            old = json.load(f)
    except (OSError, ValueError):
        old = None
    added = []
    if old is not None:
        for e in _changes(old, cur):
            added.append(append(**e))
    with open(STATE, 'w', encoding='utf-8') as f:
        json.dump(cur, f, ensure_ascii=False)
    return added
