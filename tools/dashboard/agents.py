"""Agent activity for the dashboard's Agents tab: the team (.claude/agents/*.md) and the work the lead logs.

usage:
  python tools/dashboard/agents.py task  --id ID --title "T" --steps "agent: what; agent: what; ..."
  python tools/dashboard/agents.py start --task ID --agent NAME --what "W" [--step N] [--model M] [--at ISO]
                                         (prints the run id)
  python tools/dashboard/agents.py done  --run RUN [--status done|blocked|failed|needs-you] [--tokens N] [--seconds S]
                                         [--summary "S"] [--at ISO]
  python tools/dashboard/agents.py step  --task ID --step N --status done|blocked|needs-you|running|skipped [--at ISO]
                                         (for steps without an agent run: the lead's own work, your decisions)
  python tools/dashboard/agents.py end   --task ID [--status done|blocked] [--summary "S"] [--at ISO]

Events go to tools/dashboard/agents.jsonl, one JSON object per line. The page reads them through /api/agents.
Steps are numbered from 1 in the order of --steps; an agent of 'lead' is the main Claude session, 'you' is the user.
"""
import argparse
import datetime
import json
import os
import re
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
LOG = os.path.join(HERE, 'agents.jsonl')
AGENT_DIR = os.path.join(ROOT, '.claude', 'agents')
SUBSYSTEM = ['shaddad-mcu', 'shaddad-bms-power', 'shaddad-drive', 'shaddad-structure', 'shaddad-electronics-comms',
             'shaddad-sensors-nav', 'shaddad-gnc', 'shaddad-integration']
SPECIALIST = ['shaddad-tester', 'shaddad-reviewer', 'shaddad-log-analyst', 'shaddad-report-writer', 'shaddad-docs-keeper', 'shaddad-safety']


def now():
    return datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')


def append(ev):
    ev.setdefault('at', now())
    with open(LOG, 'a', encoding='utf-8', newline='\n') as f:
        f.write(json.dumps(ev, ensure_ascii=False) + '\n')


def team():
    """The agents from their files (name, model, the first sentence of the description), the lead first."""
    out = [{'name': 'lead', 'title': 'Lead (main session)', 'group': 'lead', 'model': 'opus',
            'role': 'Talks with you, splits each task, picks the agents, merges their results and asks for your approvals.'}]
    files = sorted(f for f in os.listdir(AGENT_DIR) if f.endswith('.md')) if os.path.isdir(AGENT_DIR) else []
    found = {}
    for f in files:
        with open(os.path.join(AGENT_DIR, f), encoding='utf-8') as fh:
            text = fh.read()
        m = re.match(r'^---\n(.*?)\n---\n', text, re.S)
        if not m:
            continue
        meta = {}
        for line in m.group(1).split('\n'):
            k, _, v = line.partition(':')
            meta[k.strip()] = v.strip()
        name = meta.get('name') or f[:-3]
        desc = meta.get('description', '')
        first = re.split(r'(?<=\.)\s', desc, maxsplit=2)
        role = ' '.join(first[1:2]) if len(first) > 1 else desc
        found[name] = {'name': name, 'title': re.sub(r'(?i)\s+of the Shaddad UGV( Simulink)? project\.?$', '', first[0]).strip(),
                       'group': 'subsystem' if name in SUBSYSTEM else 'specialist' if name in SPECIALIST else 'other',
                       'model': meta.get('model', ''), 'role': role}
    for n in SUBSYSTEM + SPECIALIST + sorted(k for k in found if k not in SUBSYSTEM + SPECIALIST):
        if n in found:
            out.append(found[n])
    return out


def read():
    runs, tasks, order = {}, {}, []
    if os.path.exists(LOG):
        with open(LOG, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                ev = e.get('ev')
                if ev == 'task':
                    steps = [{'n': i + 1, 'agent': a.strip(), 'what': w.strip(), 'status': 'waiting', 'runs': []}
                             for i, (a, _, w) in enumerate(s.partition(':') for s in e.get('steps', []))]
                    # a task event with a known id redefines its steps (a changed plan): keep its start and its place
                    start = tasks[e['id']]['start'] if e['id'] in tasks else e['at']
                    tasks[e['id']] = {'id': e['id'], 'title': e.get('title', e['id']), 'start': start, 'end': None,
                                      'status': e.get('status', 'running'), 'summary': e.get('summary', ''), 'steps': steps}
                    if e['id'] not in order:
                        order.append(e['id'])
                elif ev == 'start':
                    runs[e['run']] = {'id': e['run'], 'task': e.get('task'), 'agent': e.get('agent'), 'what': e.get('what', ''),
                                      'step': e.get('step'), 'model': e.get('model', ''), 'start': e['at'], 'end': None,
                                      'status': 'working', 'tokens': None, 'seconds': None, 'summary': ''}
                    if e.get('task') in tasks and tasks[e['task']]['status'] == 'queued':   # a queued task starts with its first run
                        tasks[e['task']]['status'] = 'running'
                elif ev == 'done' and e.get('run') in runs:
                    r = runs[e['run']]
                    r.update(end=e['at'], status=e.get('status', 'done'), tokens=e.get('tokens'), seconds=e.get('seconds'),
                             summary=e.get('summary', ''))
                elif ev == 'step' and e.get('task') in tasks:
                    for s in tasks[e['task']]['steps']:
                        if s['n'] == e.get('step'):
                            s['status'] = e.get('status', 'done')
                            s['at'] = e['at']
                elif ev == 'end' and e.get('task') in tasks:
                    tasks[e['task']].update(end=e['at'], status=e.get('status', 'done'), summary=e.get('summary', ''))
    for r in runs.values():
        t = tasks.get(r['task'])
        if not t:
            continue
        for s in t['steps']:
            if s['n'] == r['step'] or (r['step'] is None and s['agent'] == r['agent'] and s['status'] in ('waiting', 'working')):
                s['runs'].append(r['id'])
                # a step event logged after the run is the step's state (the lead closing a step by hand wins)
                if not (s.get('at') and s['at'] >= (r['end'] or r['start'])):
                    s['status'] = r['status']
                break
    tm = team()
    by = {a['name']: a for a in tm}
    for a in tm:
        mine = sorted((r for r in runs.values() if r['agent'] == a['name']), key=lambda r: r['start'])
        a['runs'] = len(mine)
        a['tokens'] = sum(r['tokens'] or 0 for r in mine)
        live = [r for r in mine if r['status'] == 'working']
        last = live[-1] if live else (mine[-1] if mine else None)
        a['state'] = 'working' if live else (last['status'] if last else 'idle')
        a['last'] = ({'what': last['what'], 'task': last['task'], 'start': last['start'], 'end': last['end'],
                      'summary': last['summary']} if last else None)
    unknown = sorted({r['agent'] for r in runs.values() if r['agent'] not in by})
    return {'updated': now(), 'team': tm, 'unknown_agents': unknown,
            'runs': sorted(runs.values(), key=lambda r: r['start']), 'tasks': [tasks[i] for i in reversed(order)]}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('task'); p.add_argument('--id', required=True); p.add_argument('--title', required=True)
    p.add_argument('--steps', required=True); p.add_argument('--at')
    p.add_argument('--status', help="'queued' for a task that has not started (the kanban shows it as one To do card)")
    p.add_argument('--summary', help='one line shown on the queued card')
    p = sub.add_parser('start'); p.add_argument('--task', required=True); p.add_argument('--agent', required=True)
    p.add_argument('--what', required=True); p.add_argument('--step', type=int); p.add_argument('--model', default='')
    p.add_argument('--at'); p.add_argument('--run')
    p = sub.add_parser('done'); p.add_argument('--run', required=True); p.add_argument('--status', default='done')
    p.add_argument('--tokens', type=int); p.add_argument('--seconds', type=float); p.add_argument('--summary', default='')
    p.add_argument('--at')
    p = sub.add_parser('step'); p.add_argument('--task', required=True); p.add_argument('--step', type=int, required=True)
    p.add_argument('--status', default='done'); p.add_argument('--at')
    p = sub.add_parser('end'); p.add_argument('--task', required=True); p.add_argument('--status', default='done')
    p.add_argument('--summary', default=''); p.add_argument('--at')
    a = ap.parse_args()
    at = {'at': a.at} if getattr(a, 'at', None) else {}
    if a.cmd == 'task':
        extra = {k: v for k, v in (('status', a.status), ('summary', a.summary)) if v}
        append(dict(ev='task', id=a.id, title=a.title, steps=[s.strip() for s in a.steps.split(';') if s.strip()], **extra, **at))
    elif a.cmd == 'start':
        rid = a.run or f'{a.task}-{a.agent.replace("shaddad-", "")}-{uuid.uuid4().hex[:4]}'
        append(dict(ev='start', run=rid, task=a.task, agent=a.agent, what=a.what, step=a.step, model=a.model, **at))
        print(rid)
    elif a.cmd == 'done':
        append(dict(ev='done', run=a.run, status=a.status, tokens=a.tokens, seconds=a.seconds, summary=a.summary, **at))
    elif a.cmd == 'step':
        append(dict(ev='step', task=a.task, step=a.step, status=a.status, **at))
    elif a.cmd == 'end':
        append(dict(ev='end', task=a.task, status=a.status, summary=a.summary, **at))


if __name__ == '__main__':
    main()
