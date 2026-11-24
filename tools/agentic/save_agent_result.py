"""Save an agent's final message (its report) to a file, for the lead.

  python tools/agentic/save_agent_result.py <agentId> <dest> [--header "line"]...

The harness refuses report files from subagents, so the lead saves them. This copies the agent's last message
from its transcript (~/.claude/projects/*/<session>/subagents/agent-<agentId>.jsonl) into <dest>, after the
optional header lines, without passing the text through the lead's context. It prints only the path and the size.
"""
import argparse
import glob
import json
import os
import sys


def final_text(path):
    """The text of the last assistant message in a transcript (all its text blocks, in order)."""
    items = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                e = json.loads(line)
            except ValueError:
                continue
            m = e.get('message')
            if e.get('type') != 'assistant' or not isinstance(m, dict):
                continue
            c = m.get('content')
            texts = [x.get('text', '') for x in c if isinstance(x, dict) and x.get('type') == 'text'] \
                if isinstance(c, list) else ([c] if isinstance(c, str) else [])
            items.append((e.get('requestId'), [t for t in texts if t]))
    # the last request that produced text, with every text block of that request
    for i in range(len(items) - 1, -1, -1):
        if items[i][1]:
            rid = items[i][0]
            return '\n\n'.join(t for r, ts in items if r == rid for t in ts)
    return ''


def main():
    ap = argparse.ArgumentParser(description='Save an agent\'s final message to a file.')
    ap.add_argument('agent', help='the agentId from the Agent tool result')
    ap.add_argument('dest', help='the file to write (its folder is created)')
    ap.add_argument('--header', action='append', default=[], help='a line written before the message')
    a = ap.parse_args()
    pat = os.path.join(os.path.expanduser('~'), '.claude', 'projects', '*', '*', 'subagents',
                       'agent-%s.jsonl' % a.agent)
    hits = glob.glob(pat)
    if not hits:
        sys.exit('no transcript for agent %s (%s)' % (a.agent, pat))
    text = final_text(max(hits, key=os.path.getmtime)).strip()
    if not text:
        sys.exit('agent %s has no final text yet' % a.agent)
    os.makedirs(os.path.dirname(os.path.abspath(a.dest)), exist_ok=True)
    with open(a.dest, 'w', encoding='utf-8', newline='\n') as f:
        for h in a.header:
            f.write(h + '\n')
        if a.header:
            f.write('\n')
        f.write(text + '\n')
    print('wrote %s: %d characters' % (a.dest, len(text)))


if __name__ == '__main__':
    main()
