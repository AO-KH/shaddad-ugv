"""Add an entry to the Shaddad UGV dashboard's journal (tools/dashboard/journal.jsonl).

usage: python tools/dashboard/log.py <kind> "<text>" [--task T] [--area A] [--ref path-or-url] [--by NAME] [--at ISO]

kinds: start, done, result, ruling, decision, needs-you, resolved, issue, note (run, report, change and sleep are the
dashboard's own). The page shows the entry at its next refresh.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import journal  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='Add an entry to the dashboard journal')
    ap.add_argument('kind', choices=journal.KINDS)
    ap.add_argument('text')
    ap.add_argument('--task')
    ap.add_argument('--area')
    ap.add_argument('--ref')
    ap.add_argument('--by', default='Claude')
    ap.add_argument('--at', help='local ISO time; default now')
    a = ap.parse_args()
    e = journal.append(a.kind, a.text, at=a.at, task=a.task, area=a.area, ref=a.ref, by=a.by)
    print(f"logged {e['at']} {e['kind']}: {e['text'][:100]}")


if __name__ == '__main__':
    main()
