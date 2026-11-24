"""Check a software inventory of card T-07 (schema t07-inventory-1) before it goes to the design description.

usage: python tools/reports/t07_check_inventory.py work/team/t07/inv/<component>.json [--quiet]

It checks:
- the fields of the schema, and that ids are unique;
- that each function's block path resolves in the trace of its model (work/team/t07/trace/<model>.json, made by
  tools/reports/codegen_trace.py), and which traced blocks fall under no function (the unassigned blocks);
- that each test named as <class>/<method> exists in tests/**/<class>.m;
- that each requirement exists in docs/requirements/system.yaml;
- each flowchart (one start, at least one end, decisions with two labelled ways out, edges between known nodes);
- each state machine (states unique, transitions between known states, a guard on each, the line inside the file).
It prints a summary per unit and exits 1 on any error. Warnings (an unassigned block, a function with no test) do
not fail it but are listed. Plain python, no installs.
"""
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TRIVIAL = {'Inport', 'Outport', 'BusCreator', 'BusSelector', 'BusAssignment', 'SignalConversion', 'Terminator',
           'Ground', 'Mux', 'Demux', 'From', 'Goto', 'Reshape', 'SignalConversion generated from',
           'RateTransition generated from', 'EnablePort', 'TriggerPort', 'ActionPort'}
NODE_TYPES = {'start', 'end', 'process', 'decision', 'io', 'subroutine'}
UNIT_KEYS = ('id', 'name', 'title', 'model', 'rate', 'purpose', 'inputs', 'outputs', 'functions', 'flow')
TOP_KEYS = ('schema', 'component', 'models', 'owner', 'summary', 'units', 'fsms', 'gaps', 'findings')


class Report:
    def __init__(self):
        self.errors, self.warnings = [], []

    def err(self, where, text):
        self.errors.append('%s: %s' % (where, text))

    def warn(self, where, text):
        self.warnings.append('%s: %s' % (where, text))


def load_trace(inv, model):
    for c in inv.get('code', []):
        if c.get('model') == model and c.get('trace'):
            p = os.path.join(ROOT, c['trace'])
            if os.path.isfile(p):
                return json.load(open(p, encoding='utf-8'))
    p = os.path.join(ROOT, 'work', 'team', 't07', 'trace', model + '.json')
    return json.load(open(p, encoding='utf-8')) if os.path.isfile(p) else None


def test_index():
    idx = {}
    for f in glob.glob(os.path.join(ROOT, 'tests', '**', '*.m'), recursive=True):
        idx.setdefault(os.path.splitext(os.path.basename(f))[0], f)
    return idx


def has_method(path, method, cache={}):
    if path not in cache:
        cache[path] = open(path, encoding='utf-8', errors='replace').read()
    return re.search(r'function\s+(?:\[?[\w,\s]*\]?\s*=\s*)?' + re.escape(method) + r'\s*\(', cache[path]) is not None


def req_ids():
    txt = open(os.path.join(ROOT, 'docs', 'requirements', 'system.yaml'), encoding='utf-8').read()
    return set(re.findall(r'id:\s*(REQ-SYS-\d{3})', txt))


def check_tests(rep, where, tests, tidx):
    for t in tests or []:
        cls, _, meth = t.partition('/')
        if cls not in tidx:
            rep.err(where, 'test class not found: ' + t)
        elif meth and not has_method(tidx[cls], meth):
            rep.err(where, 'test method not found: ' + t)


def check_flow(rep, where, flow):
    if not flow:
        rep.err(where, 'no flowchart')
        return
    nodes = {n.get('id'): n for n in flow.get('nodes', [])}
    if len(nodes) != len(flow.get('nodes', [])):
        rep.err(where, 'flowchart node ids are not unique')
    for n in nodes.values():
        if n.get('type') not in NODE_TYPES:
            rep.err(where, 'flowchart node %s has type %r' % (n.get('id'), n.get('type')))
    if sum(1 for n in nodes.values() if n.get('type') == 'start') != 1:
        rep.err(where, 'a flowchart needs exactly one start node')
    if not any(n.get('type') == 'end' for n in nodes.values()):
        rep.err(where, 'a flowchart needs an end node')
    outs = {}
    for e in flow.get('edges', []):
        if e.get('from') not in nodes or e.get('to') not in nodes:
            rep.err(where, 'flowchart edge between unknown nodes: %s -> %s' % (e.get('from'), e.get('to')))
        outs.setdefault(e.get('from'), []).append(e)
    for n in nodes.values():
        if n.get('type') == 'decision':
            es = outs.get(n['id'], [])
            if len(es) < 2 or not all(e.get('label') for e in es):
                rep.err(where, 'decision %s needs two or more labelled ways out' % n['id'])


def check_fsm(rep, fsm, tidx):
    where = 'FSM ' + str(fsm.get('id'))
    for k in ('id', 'name', 'source', 'states', 'initial', 'transitions'):
        if k not in fsm:
            rep.err(where, 'missing field ' + k)
    names = [s.get('name') for s in fsm.get('states', [])]
    if len(set(names)) != len(names):
        rep.err(where, 'state names are not unique')
    if fsm.get('initial') not in names:
        rep.err(where, 'initial state %r is not a state' % fsm.get('initial'))
    src = os.path.join(ROOT, fsm.get('source', ''))
    nlines = len(open(src, encoding='utf-8', errors='replace').read().splitlines()) if os.path.isfile(src) else 0
    if not nlines:
        rep.err(where, 'source file not found: %s' % fsm.get('source'))
    ids, reached = set(), {fsm.get('initial')}
    for t in fsm.get('transitions', []):
        tw = where + ' ' + str(t.get('id'))
        if t.get('id') in ids:
            rep.err(tw, 'transition id repeated')
        ids.add(t.get('id'))
        for end in ('from', 'to'):
            if t.get(end) not in names and t.get(end) != '*':
                rep.err(tw, '%s state %r is not a state' % (end, t.get(end)))
        if not t.get('guard'):
            rep.err(tw, 'no guard')
        if nlines and not (isinstance(t.get('line'), int) and 1 <= t['line'] <= nlines):
            rep.err(tw, 'line %r is not a line of %s' % (t.get('line'), fsm.get('source')))
        check_tests(rep, tw, t.get('tests'), tidx)
        if not t.get('tests'):
            rep.warn(tw, 'no test exercises this transition (the run coverage may still show it)')
        reached.add(t.get('to'))
    for s in names:
        if s not in reached:
            rep.warn(where, 'state %s has no transition into it' % s)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    quiet = '--quiet' in sys.argv
    inv = json.load(open(path, encoding='utf-8'))
    rep = Report()
    for k in TOP_KEYS:
        if k not in inv:
            rep.err('inventory', 'missing field ' + k)
    if inv.get('schema') != 't07-inventory-1':
        rep.err('inventory', 'schema must be t07-inventory-1')
    tidx, reqs = test_index(), req_ids()
    seen = set()
    lines = []
    model_paths = {}
    for u in inv.get('units', []):
        where = 'unit ' + str(u.get('id'))
        for k in UNIT_KEYS:
            if k not in u:
                rep.err(where, 'missing field ' + k)
        if u.get('id') in seen:
            rep.err(where, 'unit id repeated')
        seen.add(u.get('id'))
        trace = load_trace(inv, u.get('model', ''))
        paths, n_ok, n_fun, n_test = [], 0, 0, 0
        for f in u.get('functions', []):
            fw = where + ' ' + str(f.get('id'))
            n_fun += 1
            if f.get('id') in seen:
                rep.err(fw, 'function id repeated')
            seen.add(f.get('id'))
            for k in ('id', 'name', 'block', 'kind', 'does'):
                if not f.get(k):
                    rep.err(fw, 'missing field ' + k)
            blocks = f.get('block') if isinstance(f.get('block'), list) else [f.get('block')]
            found = False
            for b in blocks:
                if not b:
                    continue
                paths.append(b)
                if trace is not None:
                    hit = any(k == b or k.startswith(b + '/') for k in trace['blocks'])
                    if hit:
                        found = True
                    else:
                        rep.err(fw, 'block path not in the C trace: ' + b)
            n_ok += 1 if found else 0
            check_tests(rep, fw, f.get('tests'), tidx)
            if f.get('tests'):
                n_test += 1
            else:
                rep.warn(fw, 'no test named for this function')
            for r in f.get('requirements', []):
                if r not in reqs:
                    rep.err(fw, 'requirement not in the register: ' + r)
        check_flow(rep, where, u.get('flow'))
        if trace is None:
            rep.warn(where, 'no C trace for model %s (code not generated, or the trace file missing)' % u.get('model'))
        else:
            model_paths.setdefault(u.get('model'), (trace, []))[1].extend(paths)
        lines.append('%-10s %-28s functions %3d, in the C trace %3d, with tests %3d'
                     % (u.get('id'), str(u.get('name'))[:28], n_fun, n_ok, n_test))
    for model, (trace, paths) in model_paths.items():      # blocks of a model that no unit's function covers
        types = trace.get('types', {})
        unassigned = [b for b in trace['blocks'] if types.get(b) not in TRIVIAL
                      and not any(b == p or b.startswith(p + '/') for p in paths)]
        lines.append('model %-22s traced blocks %4d, under no function %4d' % (model, len(trace['blocks']), len(unassigned)))
        for b in unassigned[:15]:
            rep.warn('model ' + model, 'traced block under no function: %s (%s)' % (b, types.get(b, '?')))
        if len(unassigned) > 15:
            rep.warn('model ' + model, '... and %d more traced blocks under no function' % (len(unassigned) - 15))
    for fsm in inv.get('fsms', []):
        check_fsm(rep, fsm, tidx)
    for fr in inv.get('frames', []):                          # step 2: the frames, sender to receiver
        fw = 'frame ' + str(fr.get('id'))
        if fr.get('id') in seen:
            rep.err(fw, 'frame id repeated')
        seen.add(fr.get('id'))
        for k in ('id', 'name', 'bus', 'direction', 'rate', 'sender', 'receiver', 'layout', 'checks', 'icd_match'):
            if k not in fr:
                rep.err(fw, 'missing field ' + k)
        for side in ('sender', 'receiver'):
            s = fr.get(side) or {}
            if not s.get('block'):
                rep.err(fw, side + ' has no block path')
                continue
            tr = load_trace(inv, s.get('model', ''))
            if tr is not None and not any(k == s['block'] or k.startswith(s['block'] + '/') for k in tr['blocks']):
                rep.err(fw, '%s block not in the C trace of %s: %s' % (side, s.get('model'), s['block']))
        if not fr.get('layout') or not all(f.get('field') for f in fr.get('layout', [])):
            rep.err(fw, 'the layout needs a field name on every row')
        check_tests(rep, fw, fr.get('tests'), tidx)
        if not fr.get('tests'):
            rep.warn(fw, 'no test named for this frame')
        if fr.get('flow', {}).get('nodes'):
            check_flow(rep, fw, fr['flow'])
        for r in fr.get('requirements', []):
            if r not in reqs:
                rep.err(fw, 'requirement not in the register: ' + r)
    sm = inv.get('system_modes')                              # step 6: the system's states and modes
    if sm is not None:
        for m in sm.get('modes', []):
            if not m.get('name') or not isinstance(m.get('states'), dict) or not m.get('meaning'):
                rep.err('system mode ' + str(m.get('name')), 'needs name, meaning and a states map')
        for c in sm.get('chains', []):
            cw = 'chain ' + str(c.get('event'))
            if not c.get('steps'):
                rep.err(cw, 'no steps')
            for s in c.get('steps', []):
                if not s.get('machine') or not s.get('to'):
                    rep.err(cw, 'a step needs machine and to')
            for r in ([c['requirement']] if isinstance(c.get('requirement'), str) else c.get('requirement') or []):
                if r and r not in reqs:
                    rep.err(cw, 'requirement not in the register: ' + r)
        if not sm.get('state_signals'):
            rep.err('system modes', 'no state_signals')
    print('%s: %d units, %d state machines' % (inv.get('component'), len(inv.get('units', [])), len(inv.get('fsms', []))))
    for line in lines:
        print('  ' + line)
    print('errors %d, warnings %d' % (len(rep.errors), len(rep.warnings)))
    for e in rep.errors:
        print('  ERROR ' + e)
    if not quiet:
        for w in rep.warnings:
            print('  warn  ' + w)
    sys.exit(1 if rep.errors else 0)


if __name__ == '__main__':
    main()
