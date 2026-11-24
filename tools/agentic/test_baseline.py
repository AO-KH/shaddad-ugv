"""Known-failure baseline of the Shaddad test classes, for the agentic test-and-fix workflow.

A run log holds one line per test, 'PASSED|FAILED|INCOMPLETE <name> (<s> s)', each failing line followed by
'    | <diagnostic>' lines, and a summary line '== <label>: total N, passed N, failed N, incomplete N, N s'
(the format of utilities/shaddad_run_tests.m and of the layout plan's full test run).

usage:
  python tools/agentic/test_baseline.py build <log> [<log> ...] [-o tests/test_baseline.json] [--merge]
  python tools/agentic/test_baseline.py compare <log> [<log> ...] [-b tests/test_baseline.json]

build    writes the baseline from the logs; --merge keeps the tests and classes of the existing file that these
         logs do not contain. A log of a whole class replaces that class; a partial log (some tests of a class, e.g.
         a 'Name' run) updates only its tests and the class counts, and is listed under the class's 'updates'.
         Class notes ('note') are always kept. A log without its summary line is skipped.
compare  sorts the tests of new logs against the baseline: new failures (passed in the baseline), failing tests
         the baseline does not have, still failing (known), fixed (failed in the baseline, pass now).
         Exit code 1 when there are new failures.
"""
import argparse
import datetime
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT = os.path.join(ROOT, 'tests', 'test_baseline.json')
LINE = re.compile(r'^(PASSED|FAILED|INCOMPLETE)\s+(.+?) \(([\d.]+) s\)\s*$')
DIAG = re.compile(r'^    \| (.*)$')
SUMMARY = re.compile(r'^== (.+): total (\d+), passed (\d+), failed (\d+), incomplete (\d+)(?:, (\d+) s)?\s*$')


def parse(path):
    """Return (tests, summary): tests maps name -> {status, seconds[, diagnostic]}; summary is None when missing."""
    tests, summary, last = {}, None, None
    with open(path, encoding='utf-8', errors='replace') as f:
        for raw in f:
            line = raw.rstrip('\r\n')
            m = LINE.match(line)
            if m:
                last = m.group(2)
                tests[last] = {'status': m.group(1), 'seconds': float(m.group(3))}
                continue
            m = DIAG.match(line)
            if m and last and tests[last]['status'] != 'PASSED' and 'diagnostic' not in tests[last]:
                tests[last]['diagnostic'] = m.group(1)[:400]
                continue
            m = SUMMARY.match(line)
            if m:
                summary = {'label': m.group(1), 'total': int(m.group(2)), 'passed': int(m.group(3)),
                           'failed': int(m.group(4)), 'incomplete': int(m.group(5)),
                           'seconds': int(m.group(6)) if m.group(6) else None}
    return tests, summary


def rel(path):
    try:
        return os.path.relpath(os.path.abspath(path), ROOT).replace('\\', '/')
    except ValueError:
        return path


def build(args):
    old = {}
    if os.path.exists(args.o):
        with open(args.o, encoding='utf-8') as f:
            old = json.load(f)
    classes = dict(old.get('classes', {})) if args.merge else {}
    tests = dict(old.get('tests', {})) if args.merge else {}
    for log in args.logs:
        t, s = parse(log)
        if s is None or not t:
            print('skipped (no summary line or no tests):', rel(log))
            continue
        names = sorted({n.split('/')[0] for n in t})
        day = datetime.date.fromtimestamp(os.path.getmtime(log)).isoformat()
        for c in names:
            known = {k for k in tests if k.split('/')[0] == c}
            in_log = {k: v for k, v in t.items() if k.split('/')[0] == c}
            if args.merge and c in classes and not known <= set(in_log):
                # a partial log (some tests of the class): update those tests and the class counts only
                key = {'PASSED': 'passed', 'FAILED': 'failed', 'INCOMPLETE': 'incomplete'}
                for k, v in in_log.items():
                    was = tests.get(k)
                    if was is None:
                        classes[c]['total'] += 1
                    else:
                        classes[c][key[was['status']]] -= 1
                    classes[c][key[v['status']]] += 1
                    tests[k] = v
                classes[c].setdefault('updates', []).append({'log': rel(log), 'date': day, 'tests': len(in_log)})
                continue
            for k in known:
                del tests[k]
            tests.update(in_log)
            if s['label'] == c:
                entry = dict(s, log=rel(log), date=day)
            else:
                # a log of several classes has one summary line per class and parse() keeps the last one:
                # count this class from its own test lines (2026-11-15: t_drive got t_mf_equiv's 6 / 6)
                cnt = {'PASSED': 0, 'FAILED': 0, 'INCOMPLETE': 0}
                for v in in_log.values():
                    cnt[v['status']] = cnt.get(v['status'], 0) + 1
                entry = {'label': c, 'total': len(in_log), 'passed': cnt['PASSED'], 'failed': cnt['FAILED'],
                         'incomplete': cnt['INCOMPLETE'],
                         'seconds': int(round(sum(v['seconds'] for v in in_log.values()))),
                         'log': rel(log), 'date': day}
            note = old.get('classes', {}).get(c, {}).get('note')
            if note:
                entry['note'] = note
            classes[c] = entry
        print('read', rel(log), '-', s['label'], '- %d tests' % len(t))
    out = {'about': 'Test results the agentic test-and-fix workflow compares new runs against. Rebuilt by '
                    'tools/agentic/test_baseline.py build; a class note says why its failures are known.',
           'updated': datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S'),
           'classes': dict(sorted(classes.items())), 'tests': dict(sorted(tests.items()))}
    with open(args.o, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(out, f, indent=1)
        f.write('\n')
    bad = sum(1 for v in tests.values() if v['status'] != 'PASSED')
    print('wrote %s: %d classes, %d tests, %d not passing' % (rel(args.o), len(classes), len(tests), bad))


def compare(args):
    with open(args.b, encoding='utf-8') as f:
        base = json.load(f)['tests']
    new_fail, unknown_fail, known, fixed, passed = [], [], [], [], 0
    for log in args.logs:
        t, s = parse(log)
        if s is None:
            print('note: %s has no summary line (the run may not have finished)' % rel(log))
        for n, v in sorted(t.items()):
            b = base.get(n)
            if v['status'] == 'PASSED':
                if b and b['status'] != 'PASSED':
                    fixed.append(n)
                else:
                    passed += 1
            elif b is None:
                unknown_fail.append((n, v))
            elif b['status'] == 'PASSED':
                new_fail.append((n, v))
            else:
                known.append(n)
    print('new failures (passed in the baseline): %d' % len(new_fail))
    for n, v in new_fail:
        print('  %-10s %s\n             %s' % (v['status'], n, v.get('diagnostic', '')[:300]))
    print('failing, not in the baseline: %d' % len(unknown_fail))
    for n, v in unknown_fail:
        print('  %-10s %s\n             %s' % (v['status'], n, v.get('diagnostic', '')[:300]))
    print('still failing (known): %d' % len(known))
    print('fixed (failed in the baseline, pass now): %d' % len(fixed))
    for n in fixed:
        print('  ' + n)
    print('passed: %d' % passed)
    return 1 if new_fail else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build')
    b.add_argument('logs', nargs='+')
    b.add_argument('-o', default=DEFAULT)
    b.add_argument('--merge', action='store_true')
    c = sub.add_parser('compare')
    c.add_argument('logs', nargs='+')
    c.add_argument('-b', default=DEFAULT)
    a = ap.parse_args()
    if a.cmd == 'build':
        build(a)
        return 0
    return compare(a)


if __name__ == '__main__':
    sys.exit(main())
