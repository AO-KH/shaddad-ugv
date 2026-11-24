"""Shaddad UGV dashboard: read the project's state from its own files. Read only: nothing here writes to the project.

collect() returns one dict for the page. Each part is gathered on its own, so a part that fails is reported in
'errors' and the rest still shows.
"""
import base64
import datetime as dt
import glob
import json
import os
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIRS = [os.path.join('work', 'layout', 'logs'), os.path.join('work', 'reports')]
# the team's batch test runs, one log per class (work/team/<task>/logs/full/t_drive.log): read for the test results only
TEST_LOG_DIRS = [os.path.join('work', 'team', '*', 'logs', 'full'), os.path.join('work', 'team', '*', 'logs', 'rerun')]
SKIP_DIRS = {'slprj', '__pycache__', '.git', 'snap', 'runs', 'cg_base', 'cg_after', 'backup', 'trial', 'before_rebuild',
             'checkpoint', 'artifact-files', 'images', 'stylesheets', '_rels'}
SKIP_EXT = {'.slxc', '.mexw64', '.asv', '.autosave', '.bak', '.tmp'}
DOMAINS = ['structure', 'power', 'drive', 'rc', 'electronics', 'sensors', 'gnc', 'integration']
DOMAIN_LABEL = {'structure': 'Structure', 'power': 'Power', 'drive': 'Drive', 'rc': 'RC link', 'electronics': 'Electronics',
                'sensors': 'Sensors', 'gnc': 'GNC', 'integration': 'Integration'}
TOP_MODELS = {'Shaddad_System': 'integration', 'Shaddad_Plant': 'integration', 'Shaddad_MCU': 'electronics',
              'Shaddad_BMS': 'power', 'Shaddad_Companion': 'gnc', 'Shaddad_NavFusion': 'sensors',
              'Shaddad_Perception': 'sensors', 'Shaddad_GNC': 'gnc'}
TEST_FOLDER_DOMAIN = {'system': 'integration'}
_cache = {}


def rel(p):
    return os.path.relpath(p, ROOT).replace('\\', '/')


def iso(ts):
    return dt.datetime.fromtimestamp(ts).isoformat(timespec='seconds') if ts else None


def read(p, tail=None):
    try:
        with open(p, 'rb') as f:
            if tail:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - tail))
            return f.read().decode('utf-8', 'replace').replace('\r', '')
    except OSError:
        return ''


def cached(p, fn):
    """fn(path) once per (mtime, size) of the file."""
    try:
        st = os.stat(p)
    except OSError:
        return None
    key = (p, fn.__name__)
    hit = _cache.get(key)
    if hit and hit[0] == (st.st_mtime, st.st_size):
        return hit[1]
    val = fn(p)
    _cache[key] = ((st.st_mtime, st.st_size), val)
    return val


def table_rows(lines):
    rows = []
    for ln in lines:
        if not ln.startswith('|'):
            continue
        cells = [c.strip() for c in ln.strip().strip('|').split('|')]
        if all(re.fullmatch(r':?-{2,}:?', c) for c in cells if c):
            continue
        rows.append(cells)
    return rows


def plain(s):
    return re.sub(r'\*\*|`', '', s).strip()


def section(text, head_re):
    """The lines of the markdown section whose heading matches head_re, up to the next heading of the same level."""
    lines = text.split('\n')
    for i, ln in enumerate(lines):
        m = re.match(r'^(#{1,4}) ', ln)
        if m and re.match(head_re, ln):
            lvl = len(m.group(1))
            out = []
            for ln2 in lines[i + 1:]:
                m2 = re.match(r'^(#{1,4}) ', ln2)
                if m2 and len(m2.group(1)) <= lvl:
                    break
                out.append(ln2)
            return out
    return []


# ---- the computer: MATLAB processes, memory, disk, sleeps (one PowerShell call)
PS = r"""
$p = @(Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe'" | ForEach-Object {
  [pscustomobject]@{ pid = $_.ProcessId; parent = $_.ParentProcessId; started = $_.CreationDate.ToString('s'); cmd = [string]$_.CommandLine;
    mem = [int64]$_.WorkingSetSize } })
$o = Get-CimInstance Win32_OperatingSystem
$e = @(Get-WinEvent -FilterHashtable @{ LogName = 'System'; ProviderName = 'Microsoft-Windows-Kernel-Power'; Id = 506, 507, 42;
  StartTime = (Get-Date).AddDays(-4) } -ErrorAction SilentlyContinue | ForEach-Object { [pscustomobject]@{ t = $_.TimeCreated.ToString('s'); id = $_.Id } })
[pscustomobject]@{ procs = $p; total_kb = [int64]$o.TotalVisibleMemorySize; free_kb = [int64]$o.FreePhysicalMemory;
  boot = $o.LastBootUpTime.ToString('s'); events = $e } | ConvertTo-Json -Depth 4 -Compress
"""


def machine():
    enc = base64.b64encode(PS.encode('utf-16-le')).decode()
    r = subprocess.run(['powershell', '-NoProfile', '-NonInteractive', '-EncodedCommand', enc], capture_output=True,
                       text=True, timeout=90, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    d = json.loads(r.stdout or '{}')
    now = dt.datetime.now()
    raw = d.get('procs') or []
    pids = {p['pid'] for p in raw}
    procs, by_pid = [], {}
    # a MATLAB launched by another MATLAB (the win64 worker of a -batch run) is part of its parent's run
    for p in sorted(raw, key=lambda p: p['parent'] in pids):
        cmd = p.get('cmd') or ''
        if p['parent'] in by_pid:
            by_pid[p['parent']]['mem_gb'] += round((p.get('mem') or 0) / 2 ** 30, 2)
            by_pid[p['pid']] = by_pid[p['parent']]
            continue
        batch = re.search(r'(^|\s)"?-batch"?(\s|$)', cmd) is not None
        m = re.search(r"openProject\('[^']*'\);\s*(.*?)(?:\s*%\s*(t\d+cap_\S+))?\"?\s*$", cmd)
        what = re.sub(r"addpath\('[^']*'\);\s*", '', (m.group(1) if m else '').strip().strip('"'))
        started = dt.datetime.fromisoformat(p['started'])
        item = {'pid': p['pid'], 'kind': 'batch' if batch else 'desktop',
                'command': (what or 'batch MATLAB')[:160] if batch else 'MATLAB desktop session',
                'started': started.isoformat(timespec='seconds'), 'minutes': round((now - started).total_seconds() / 60, 1),
                'mem_gb': round((p.get('mem') or 0) / 2 ** 30, 2)}
        by_pid[p['pid']] = item
        procs.append(item)
    for item in procs:
        item['mem_gb'] = round(item['mem_gb'], 2)
    marks = sorted((dt.datetime.fromisoformat(e['t']), 0 if e['id'] == 507 else 1) for e in d.get('events') or [])
    start, spans = None, []
    for t, enter in marks:
        if enter:
            start = start or t
        elif start:
            spans.append([start, t])
            start = None
    merged = []
    for a, b in spans:
        if merged and (a - merged[-1][1]).total_seconds() < 120:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    sleeps = [{'from': a.isoformat(timespec='seconds'), 'to': b.isoformat(timespec='seconds'),
               'hours': round((b - a).total_seconds() / 3600, 1)} for a, b in merged if (b - a).total_seconds() > 600]
    du = shutil.disk_usage(ROOT)
    return {'matlab': procs, 'ram_total_gb': round(d.get('total_kb', 0) / 2 ** 20, 1), 'ram_free_gb': round(d.get('free_kb', 0) / 2 ** 20, 1),
            'disk_total_gb': round(du.total / 2 ** 30), 'disk_free_gb': round(du.free / 2 ** 30, 1), 'boot': d.get('boot'),
            'sleeps': sleeps[::-1][:8], 'asleep_since': start.isoformat(timespec='seconds') if start else None}


# ---- docs/STATUS.md: the documented state (verdict, regression, scenarios, environment, open problems)
def status_doc():
    p = os.path.join(ROOT, 'docs', 'STATUS.md')
    text = read(p)
    lines = text.split('\n')
    regression = {}
    for c in table_rows(section(text, r'^## 4\. ')):
        m = re.search(r'(t\w+|tScenarios)\.m', c[0])
        if m and len(c) >= 6 and c[2].isdigit():
            regression[m.group(1)] = {'owner': c[1], 'tests': int(c[2]), 'passed': int(c[3]), 'failed': int(c[4]),
                                      'incomplete': int(c[5]), 'notes': plain(c[6]) if len(c) > 6 else ''}
    scen = []
    for c in table_rows(section(text, r'^### 5\.2 ')):
        if len(c) >= 3 and re.match(r'^S\d', c[0]):
            scen.append({'scenario': plain(c[0]), 'result': plain(c[1]), 'numbers': plain(c[2])[:300]})
    env = [{'item': plain(c[0]), 'state': plain(c[1])[:220]} for c in table_rows(section(text, r'^## 3\. ')) if c and c[0] != 'Item']
    verdict = [{'item': plain(c[0]), 'result': plain(c[1])[:300]} for c in table_rows(section(text, r'^## 1\. ')) if c and c[0] != 'Item']
    problems, group = [], ''
    for ln in section(text, r'^## 6\. '):
        g = re.match(r'^\*\*(.+?)\*\*', ln)
        m = re.match(r'^(\d+)\.\s+(.*)', ln)
        if m:
            body = plain(m.group(2))
            tasks = sorted(set(re.findall(r'\bT(\d\d)\b', m.group(2))))
            problems.append({'n': int(m.group(1)), 'group': group, 'text': body[:420], 'tasks': ['T' + t for t in tasks]})
        elif g:
            group = g.group(1).rstrip('.: ')
    title = next((ln[2:] for ln in lines if ln.startswith('# ')), 'STATUS')
    stamp = next((ln for ln in lines[1:6] if ln.strip()), '')
    return {'path': rel(p), 'updated': iso(os.path.getmtime(p)) if os.path.isfile(p) else None, 'title': title, 'stamp': plain(stamp),
            'regression': regression, 'scenarios': scen, 'environment': env, 'verdict': verdict, 'problems': problems,
            'markdown': text}


# ---- docs/build_plan.md: the 17 build tasks and their domains
def build_plan():
    text = read(os.path.join(ROOT, 'docs', 'build_plan.md'))
    out = []
    for m in re.finditer(r'^## (T\d\d) - (.+?) \(([^)]*)\)\s*$', text, re.M):
        dom = m.group(3).split('/')[0].strip().lower()
        out.append({'id': m.group(1), 'title': m.group(2).strip(), 'domain': dom if dom in DOMAINS else 'integration'})
    return out


# ---- plans and their work logs (docs/superpowers/plans, .superpowers/sdd/*/progress.md)
def plans():
    ledgers = {}
    for lp in glob.glob(os.path.join(ROOT, '.superpowers', 'sdd', '*', 'progress.md')):
        first = read(lp, None).split('\n', 1)[0]
        m = re.search(r'plan:\s*(\S+)', first)
        if m:
            ledgers[m.group(1).replace('\\', '/')] = lp
    out = []
    for pp in sorted(glob.glob(os.path.join(ROOT, 'docs', 'superpowers', 'plans', '*.md'))):
        text = read(pp)
        title = next((ln[2:].strip() for ln in text.split('\n') if ln.startswith('# ')), os.path.basename(pp))
        tasks = [{'n': int(m.group(1)), 'name': plain(m.group(2))} for m in re.finditer(r'^#{2,3} Task (\d+)[:.]?\s+(.+)$', text, re.M)]
        lp = ledgers.get(rel(pp))
        done, started, tail, last = set(), set(), [], None
        if lp:
            led = read(lp)
            done = {int(n) for n in re.findall(r'^Task (\d+): complete', led, re.M)}
            ws = os.path.dirname(lp)
            # started: the task has its decisions or a report, or the work log says it was dispatched or started
            # (briefs are often written ahead, so a brief alone does not count)
            started = {int(m.group(1)) for f in os.listdir(ws) for m in [re.match(r'task-(\d+)[a-z]?-(decisions|report)\.md$', f)] if m}
            started |= {int(n) for n in re.findall(r'^Task (\d+)(?: Part \w+)?: (?:dispatched|started)', led, re.M)}
            entries = [ln for ln in led.split('\n') if ln.strip() and not ln.startswith('#')]
            tail = [e[:500] for e in entries[-8:]][::-1]
            last = iso(os.path.getmtime(lp))
        for t in tasks:
            t['status'] = 'done' if t['n'] in done else ('running' if t['n'] in started else 'waiting')
        cur = next((t for t in tasks if t['status'] == 'running'), None)
        state = ('Finished' if tasks and len(done) >= len(tasks) else ('In progress' if done or started else 'Not started'))
        out.append({'file': rel(pp), 'title': re.sub(r'\s*Implementation Plan$', '', title), 'tasks': tasks, 'done': len(done),
                    'total': len(tasks), 'state': state, 'current': cur, 'ledger': rel(lp) if lp else None, 'ledger_tail': tail,
                    'ledger_updated': last, 'updated': iso(os.path.getmtime(pp))})
    return out


# ---- tests: every test class, its documented result, its newest logged run and the newest Fast report
def _totals(p):
    found = {}
    text = read(p)
    for m in re.finditer(r'^== (t\w+|tScenarios)(\S*)( [^:\n]*)?: total (\d+), passed (\d+), failed (\d+), incomplete (\d+)',
                         text, re.M):
        if m.group(2):          # one test of a class (t_infra/testX): not a class summary
            continue
        found[m.group(1)] = {'label': (m.group(3) or '').strip(), 'total': int(m.group(4)), 'passed': int(m.group(5)),
                             'failed': int(m.group(6)), 'incomplete': int(m.group(7))}
    return found or _runtests_totals(p, text)


def _runtests_totals(p, text):
    """A log of one class run with runtests: MATLAB's TestResult display ('1x34 TestResult array') and its Totals line."""
    name = os.path.splitext(os.path.basename(p))[0]
    if not re.match(r'^(t_\w+|tScenarios)$', name):
        return {}
    tot = re.findall(r'^Totals:\s+(\d+) Passed, (\d+) Failed, (\d+) Incomplete\.', text, re.M)
    if not tot:
        return {}
    passed, failed, incomplete = (int(x) for x in tot[-1])
    arr = re.findall(r'^\s*\d+\D(\d+) TestResult array', text, re.M)
    if arr:
        total = int(arr[-1])
    elif re.search(r'^\s*TestResult with properties', text, re.M):
        total = 1
    else:
        total = passed + failed
    return {name: {'label': '', 'total': total, 'passed': passed, 'failed': failed, 'incomplete': incomplete}}


def _junit(p):
    out = {}
    try:
        root = ET.parse(p).getroot()
    except ET.ParseError:
        return out
    for s in root.iter('testsuite'):
        fails = [c.get('name') for c in s.iter('testcase') if c.find('failure') is not None or c.find('error') is not None]
        out[s.get('name')] = {'tests': int(s.get('tests', 0)), 'failed': int(s.get('failures', 0)) + int(s.get('errors', 0)),
                              'skipped': int(s.get('skipped', 0)), 'failing': fails[:12]}
    return out


def tests(doc):
    classes = []
    for p in sorted(glob.glob(os.path.join(ROOT, 'tests', '**', 't*.m'), recursive=True)):
        name = os.path.splitext(os.path.basename(p))[0]
        if not re.match(r'^(t_\w+|tScenarios)$', name):
            continue
        folder = os.path.basename(os.path.dirname(p))
        classes.append({'class': name, 'file': rel(p), 'domain': TEST_FOLDER_DOMAIN.get(folder, folder)})
    latest = {}
    logs = []
    for d in LOG_DIRS + TEST_LOG_DIRS:
        logs += glob.glob(os.path.join(ROOT, d, '*.log'))
    for p in sorted(logs, key=os.path.getmtime):
        for cls, r in (cached(p, _totals) or {}).items():
            latest[cls] = dict(r, at=iso(os.path.getmtime(p)), log=rel(p))
    fast = sorted(glob.glob(os.path.join(ROOT, 'work', 'reports', 'Fast_*.xml')), key=os.path.getmtime)
    fast_res = cached(fast[-1], _junit) if fast else {}
    for c in classes:
        c['documented'] = doc['regression'].get(c['class'])
        c['latest'] = latest.get(c['class'])
        c['fast'] = (fast_res or {}).get(c['class'])
    return {'classes': classes, 'fast_report': rel(fast[-1]) if fast else None, 'fast_at': iso(os.path.getmtime(fast[-1])) if fast else None}


# ---- models and benches
def models():
    groups, files = {}, []
    for base in ('models', 'subsystems', 'tests'):
        for p in glob.glob(os.path.join(ROOT, base, '**', '*.slx'), recursive=True):
            r = rel(p)
            parts = r.split('/')
            name = os.path.splitext(parts[-1])[0]
            if base == 'models':
                g, dom = 'Top-level models', TOP_MODELS.get(name, 'integration')
            elif base == 'subsystems':
                g, dom = 'subsystems/' + parts[1], parts[1] if parts[1] in DOMAINS else 'integration'
            else:
                g, dom = 'tests/' + parts[1] + ' (benches)', TEST_FOLDER_DOMAIN.get(parts[1], parts[1])
            st = os.stat(p)
            files.append({'file': r, 'name': name, 'group': g, 'domain': dom, 'kb': round(st.st_size / 1024), 'at': iso(st.st_mtime),
                          'ts': st.st_mtime})
            gg = groups.setdefault(g, {'group': g, 'domain': dom, 'count': 0, 'mb': 0.0, 'newest': None, 'newest_file': None, 'ts': 0})
            gg['count'] += 1
            gg['mb'] += st.st_size / 2 ** 20
            if st.st_mtime > gg['ts']:
                gg.update(ts=st.st_mtime, newest=iso(st.st_mtime), newest_file=name)
    for g in groups.values():
        g['mb'] = round(g['mb'], 1)
        g.pop('ts')
    recent = sorted(files, key=lambda f: -f['ts'])[:15]
    for f in files:
        f.pop('ts')
    return {'groups': sorted(groups.values(), key=lambda g: g['group']), 'recent': recent, 'count': len(files),
            'by_domain': {d: sum(1 for f in files if f['domain'] == d) for d in DOMAINS},
            'all': [{'file': f['file'], 'at': f['at']} for f in files]}


# ---- reports: the agent task reports (tools/dashboard/task_report.py) and their step reports, the task reports and
# decisions of the plans, layout reports, STATUS.md and the HTML test reports
REPORT_GLOBS = [('work/agentic/reports/*.md', 'Agent task report'), ('work/agentic/*/*/report.md', 'Agent step report'),
                ('work/agentic/*/report.md', 'Agent summary'), ('.superpowers/sdd/*/task-*-report.md', 'Task report'), ('.superpowers/sdd/*/task-*-decisions.md', 'Task decisions'),
                ('.superpowers/sdd/*/final-review.md', 'Review'), ('work/layout/*.md', 'Layout report'), ('docs/STATUS.md', 'Project status'),
                ('work/reports/*/index.html', 'Test report'), ('work/reports/*/report.html', 'Test report'),
                ('work/reports/*.log', 'Run log'),
                # the review phase (2026-11-15): the ranked list, the system review, the brief, the ten section reviews
                # and the NASL report's PDF
                ('work/agentic/review/*.md', 'Review findings'), ('work/agentic/review/sections/*.md', 'Review section'),
                ('work/reports/review_*/*.pdf', 'Review findings'),
                # the PDF engineering report of every agent task group (the user's rule of 2026-11-15)
                ('work/reports/engineering/*.pdf', 'Engineering report'), ('work/reports/project/*.pdf', 'Project document'),
                ('framework/build/*.pdf', 'Framework document'),
                ('work/reports/requirements/*.pdf', 'Requirements specification'),
                # the design descriptions (card T-07: NASL-SHD-SDD-002)
                ('work/reports/design/*.pdf', 'Design description')]


def reports():
    out = []
    for pat, kind in REPORT_GLOBS:
        for p in glob.glob(os.path.join(ROOT, *pat.split('/'))):
            r = rel(p)
            if p.endswith('.md'):
                head = read(p)[:2500]
                title = next((ln.lstrip('# ').strip() for ln in head.split('\n') if ln.startswith('#')), os.path.basename(p))
            elif p.endswith(('.log', '.pdf')):
                title = os.path.splitext(os.path.basename(p))[0]
            else:
                folder = os.path.basename(os.path.dirname(p))
                title = ('Regression report ' if folder.startswith('Fast_') else 'Run report ') + folder
            m = re.match(r'\.superpowers/sdd/([^/]+)/', r)
            n = re.search(r'task-(\d+[a-z]?)-', r)
            out.append({'file': r, 'kind': kind, 'title': title[:160], 'plan': m.group(1) if m else '',
                        'task': n.group(1) if n else '', 'at': iso(os.path.getmtime(p)), 'kb': round(os.path.getsize(p) / 1024),
                        'readable': p.endswith(('.md', '.log'))})
    return sorted(out, key=lambda x: x['at'] or '', reverse=True)[:200]


# ---- what comes next: the running and waiting tasks of every unfinished plan, and the open items
def upcoming(pl, nt):
    out = []
    for p in pl or []:
        if p['state'] == 'Finished':
            continue
        nxt = True
        for t in p['tasks']:
            if t['status'] == 'running':
                state, kind = 'Running', 'run'
            elif t['status'] == 'waiting':
                state, kind = ('Next' if nxt and p['state'] == 'In progress' else 'Queued'), 'muted'
                nxt = False
            else:
                continue
            out.append({'what': f"Task {t['n']}: {t['name']}", 'where': p['title'], 'state': state, 'kind': kind, 'source': p['file']})
    for i in (nt or {}).get('items', []):
        if i.get('state') in ('Tip', 'Known', 'Done', 'In progress'):
            continue
        out.append({'what': i['item'], 'where': i.get('area', ''), 'state': i['state'], 'kind': i.get('kind', 'info'),
                    'source': 'notes.json', 'next': i.get('next', '')})
    return out


# ---- saved scenario results (work/results)
def scenarios(doc):
    runs = {}
    for p in glob.glob(os.path.join(ROOT, 'work', 'results', '*.mat')):
        m = re.match(r'^(.*?)_(\d{8})_(\d{6})\.mat$', os.path.basename(p))
        sid, ts = (m.group(1), os.path.getmtime(p)) if m else (os.path.splitext(os.path.basename(p))[0], os.path.getmtime(p))
        r = runs.setdefault(sid, {'scenario': sid, 'runs': 0, 'latest': None, 'ts': 0, 'file': None})
        r['runs'] += 1
        if ts > r['ts']:
            r.update(ts=ts, latest=iso(ts), file=rel(p))
    out = sorted(runs.values(), key=lambda r: -r['ts'])
    for r in out:
        r.pop('ts')
    return {'saved': out, 'documented': doc['scenarios']}


# ---- documents
def docs():
    out = []
    pats = ['docs/*.md', 'docs/specs/*.md', 'docs/hardware/*.md', 'docs/superpowers/plans/*.md', 'docs/superpowers/specs/*.md']
    for pat in pats:
        for p in glob.glob(os.path.join(ROOT, *pat.split('/'))):
            head = read(p, None)[:3000]
            title = next((ln.lstrip('# ').strip() for ln in head.split('\n') if ln.startswith('#')), os.path.basename(p))
            out.append({'file': rel(p), 'title': title[:140], 'at': iso(os.path.getmtime(p)), 'kb': round(os.path.getsize(p) / 1024)})
    return sorted(out, key=lambda d: d['at'] or '', reverse=True)


# ---- what changed lately, and the newest logs
def kind_of(r):
    ext = os.path.splitext(r)[1].lower()
    if ext == '.slx':
        return 'model'
    if r.startswith('tests/'):
        return 'test'
    if r.startswith('docs/') or ext == '.md':
        return 'doc'
    if r.startswith('params/') or r.startswith('buses/') or ext in ('.csv', '.json', '.txt'):
        return 'data'
    if r.startswith('tools/'):
        return 'tool'
    if ext == '.m':
        return 'script'
    if ext == '.mat':
        return 'result'
    return 'other'


def activity(hours=72):
    cutoff = time.time() - hours * 3600
    out = []
    for top in ('models', 'subsystems', 'tests', 'scripts', 'utilities', 'params', 'buses', 'docs', 'work', 'tools'):
        base = os.path.join(ROOT, top)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS and not d.startswith('.')]
            r_dir = rel(dirpath)
            if r_dir.startswith('work/layout/logs') or r_dir.startswith('docs/report/build'):
                continue
            for f in filenames:
                if os.path.splitext(f)[1].lower() in SKIP_EXT or f.endswith('.log'):
                    continue
                p = os.path.join(dirpath, f)
                try:
                    mt = os.path.getmtime(p)
                except OSError:
                    continue
                if mt >= cutoff:
                    r = rel(p)
                    out.append({'file': r, 'at': iso(mt), 'kind': kind_of(r), 'kb': round(os.path.getsize(p) / 1024)})
    return sorted(out, key=lambda a: a['at'], reverse=True)[:60]


SUMMARY_RE = re.compile(r'^(== .*|.*same=\d.*|.*new failures \d+.*|interface equal: .*|.*\d+ mismatches.*|PASSED\s.*|FAILED\s.*|'
                        r'.*getAlgebraicLoops.*|.*elapsed ~\d+ s.*|CAP: .*)$', re.M)


def _last_line(p):
    t = re.sub(r'[\x00-\x08\x0b-\x1f\x7f]', '', read(p, 40000))
    hits = SUMMARY_RE.findall(t)
    if hits:
        return hits[-1].strip()[:220]
    noise = ('In ', '[', '- ', 'To ', 'Suggested', 'Quantization', 'difference of', 'run-time data type')
    # a line that starts in lower case continues a wrapped MATLAB warning
    lines = [ln.strip() for ln in t.split('\n')
             if len(re.findall(r'[A-Za-z0-9]', ln)) >= 4 and 'ERROR:' not in ln and not ln.lstrip().startswith(noise)
             and not re.match(r'\s*[a-z]', ln)]
    return lines[-1][:220] if lines else ''


IMG_EXT = ('.png', '.jpg', '.jpeg', '.svg')


def _plot_title(folder, ts):
    m = re.match(r'^(plots|rowtest)(?:_([A-Za-z]\w*?))?_(\d{8})_(\d{6})$', folder)
    if m:
        when_ = dt.datetime.strptime(m.group(3) + m.group(4), '%Y%m%d%H%M%S').strftime('%b %d, %H:%M').replace(' 0', ' ')
        if m.group(1) == 'rowtest':
            return f'Row test run, {when_}'
        return f'Live-drive log {m.group(2) or ""}, {when_}'.replace(' ,', ',')
    return folder


def plots():
    """Sets of plot images saved by earlier runs: each folder of work/reports that holds images (the HTML reports' own
    icons left out), work/results and the images loose in work/."""
    sets = []
    folders = [os.path.join(ROOT, 'work', 'reports', d) for d in os.listdir(os.path.join(ROOT, 'work', 'reports'))
               if os.path.isdir(os.path.join(ROOT, 'work', 'reports', d))]
    folders += [os.path.join(ROOT, 'work', 'results'), os.path.join(ROOT, 'work')]
    for d in folders:
        try:
            names = os.listdir(d)
        except OSError:
            continue
        imgs = sorted(n for n in names if n.lower().endswith(IMG_EXT) and os.path.isfile(os.path.join(d, n)))
        if not imgs:
            continue
        figs = sum(1 for n in names if n.lower().endswith('.fig'))
        ts = max(os.path.getmtime(os.path.join(d, n)) for n in imgs)
        folder = os.path.basename(d)
        title = {'results': 'Saved in work/results', 'work': 'Saved in work/'}.get(folder, _plot_title(folder, ts))
        sets.append({'title': title, 'folder': rel(d), 'at': iso(ts), 'figs': figs,
                     'images': [{'file': rel(os.path.join(d, n)), 'name': n, 'kb': round(os.path.getsize(os.path.join(d, n)) / 1024)}
                                for n in imgs]})
    return sorted(sets, key=lambda s: s['at'], reverse=True)


def run_logs():
    """Every run log of the layout plan (work/layout/logs) and of work/reports, newest first, with its run (the name's
    first part: t6, t15, t16r, full, ...) and its last summary line. Served by /api/logs, read only when asked."""
    files = glob.glob(os.path.join(ROOT, 'work', 'layout', 'logs', '*.log')) + glob.glob(os.path.join(ROOT, 'work', 'reports', '*.log'))
    out = []
    for p in sorted(files, key=os.path.getmtime, reverse=True):
        name = os.path.basename(p)
        out.append({'file': rel(p), 'name': name, 'run': name.split('_', 1)[0], 'at': iso(os.path.getmtime(p)),
                    'kb': round(os.path.getsize(p) / 1024), 'last': cached(p, _last_line) or ''})
    return out


def logs(n=16):
    files = []
    for d in LOG_DIRS:
        files += [p for p in glob.glob(os.path.join(ROOT, d, '*')) if os.path.isfile(p) and p.endswith(('.log', '.xml'))]
    files = sorted(files, key=os.path.getmtime, reverse=True)[:n]
    return [{'file': rel(p), 'at': iso(os.path.getmtime(p)), 'kb': round(os.path.getsize(p) / 1024),
             'last': cached(p, _last_line) if p.endswith('.log') else 'JUnit report'} for p in files]


# ---- the toolchain as the newest MATLAB startup printed it
def _startup_table(p):
    t = read(p, None)[:12000]
    if '=== Shaddad UGV startup' not in t:
        return None
    rows = re.findall(r'^\s*"([^"]+)"\s+"([^"]+)"\s+"([^"]*)"', t, re.M)
    return [{'item': a, 'state': b, 'note': c} for a, b, c in rows]


def toolchain():
    files = []
    for d in LOG_DIRS:
        files += glob.glob(os.path.join(ROOT, d, '*.log'))
    for p in sorted(files, key=os.path.getmtime, reverse=True)[:40]:
        rows = cached(p, _startup_table)
        if rows:
            return {'from': rel(p), 'at': iso(os.path.getmtime(p)), 'items': rows}
    return {'from': None, 'at': None, 'items': []}


def notes():
    p = os.path.join(HERE, 'notes.json')
    try:
        return json.load(open(p, encoding='utf-8'))
    except (OSError, ValueError):
        return {'items': [], 'updated': None}


# ---- the project's task cards under the tailored life-cycle gates (docs/lifecycle.md)
CARD_STATES = ['done', 'in progress', 'needs you', 'todo']


def project():
    p = os.path.join(HERE, 'project_cards.json')
    if not os.path.isfile(p):
        return None
    d = json.load(open(p, encoding='utf-8'))
    cards = d.get('cards', [])
    succ = {}                                   # successors, always derived from the predecessors
    for c in cards:
        for q in c.get('predecessors', []):
            succ.setdefault(q, []).append(c['id'])
    for c in cards:
        c['successors'] = succ.get(c['id'], [])
    pdfs = sorted(glob.glob(os.path.join(ROOT, 'work', 'reports', 'project', 'NASL_PLN-001_*.pdf')), key=os.path.getmtime)
    d['pdf'] = rel(pdfs[-1]) if pdfs else None

    def count(cs):
        n = {k: 0 for k in CARD_STATES}
        for c in cs:
            s = c.get('status', 'todo')
            n[s] = n.get(s, 0) + 1
        return n

    for g in d.get('gates', []):
        cs = [c for c in cards if c.get('gate') == g['id']]
        g['counts'], g['cards'] = count(cs), len(cs)
        mand = [c for c in g.get('criteria', []) if c.get('class') == 'M']   # readiness: mandatory met or waived
        g['readiness'] = {'ok': sum(c.get('score') in ('met', 'waived', 'n/a') for c in mand), 'mandatory': len(mand)}
    d['counts'], d['total'] = count(cards), len(cards)
    d['complete'] = bool(cards) and d['counts']['done'] == len(cards) and all(
        g.get('status') in ('passed', 'passed with actions') for g in d.get('gates', []) if g['id'] != 'G6')
    d['path'], d['file_updated'] = rel(p), iso(os.path.getmtime(p))
    return d


# ---- one card per subsystem
def subsystems(doc, bp, tst, mdl, plans_):
    task_dom = {t['id']: t['domain'] for t in bp}
    cards = []
    for d in DOMAINS:
        cls = [c for c in tst['classes'] if c['domain'] == d]
        passed = failed = total = incomplete = 0
        for c in cls:
            r = c['latest'] or c['documented']
            if r:
                total += r.get('total', r.get('tests', 0))
                passed += r['passed']
                failed += r['failed']
                incomplete += r['incomplete']
        probs = [p for p in doc['problems'] if any(task_dom.get(t) == d for t in p['tasks'])]
        newest = max((g['newest'] for g in mdl['groups'] if g['domain'] == d and g['newest']), default=None)
        cards.append({'domain': d, 'label': DOMAIN_LABEL[d], 'models': mdl['by_domain'].get(d, 0), 'test_classes': len(cls),
                      'tests': total, 'passed': passed, 'failed': failed, 'incomplete': incomplete, 'problems': len(probs),
                      'problem_ids': [p['n'] for p in probs], 'build_tasks': [t['id'] for t in bp if t['domain'] == d],
                      'last_change': newest})
    return cards


def collect():
    t0 = time.time()
    out, errors = {}, []

    def part(name, fn, *a):
        try:
            out[name] = fn(*a)
        except Exception as e:  # a part that fails is reported, the rest still shows
            errors.append(f'{name}: {type(e).__name__}: {e}')
            out[name] = None
        return out[name]

    doc = part('status', status_doc) or {'regression': {}, 'scenarios': [], 'problems': []}
    bp = part('build_plan', build_plan) or []
    pl = part('plans', plans) or []
    tst = part('tests', tests, doc) or {'classes': []}
    mdl = part('models', models) or {'groups': [], 'by_domain': {}}
    part('scenarios', scenarios, doc)
    part('docs', docs)
    part('activity', activity)
    part('logs', logs)
    part('toolchain', toolchain)
    nt = part('notes', notes)
    part('project', project)
    part('reports', reports)
    part('plots', plots)
    part('upcoming', upcoming, pl, nt)
    part('machine', machine)
    part('subsystems', subsystems, doc, bp, tst, mdl, pl)
    out['meta'] = {'generated': dt.datetime.now().isoformat(timespec='seconds'), 'took_s': round(time.time() - t0, 2),
                   'root': ROOT, 'project': 'Shaddad UGV'}
    out['errors'] = errors
    return out


if __name__ == '__main__':
    d = collect()
    print(json.dumps({k: (len(v) if isinstance(v, (list, dict)) else v) for k, v in d.items()}, indent=1)[:2000])
    print('errors:', d['errors'], 'took', d['meta']['took_s'], 's')
