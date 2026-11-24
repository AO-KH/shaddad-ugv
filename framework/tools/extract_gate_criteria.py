"""Extract the standard gate success criteria from the Engineering Handbook (section 4.4) into
framework/templates/gate_success_criteria.json, so that the criteria have one source: the handbook.

usage: python framework/tools/extract_gate_criteria.py
Run it after every change of the criteria tables in framework/handbook/engineering_handbook.md.
"""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
SRC = os.path.join(FW, 'handbook', 'engineering_handbook.md')
OUT = os.path.join(FW, 'templates', 'gate_success_criteria.json')


def main():
    text = open(SRC, encoding='utf-8').read()
    ver = re.search(r'^version:\s*(\S+)', text, re.M).group(1)
    crit = {}
    for line in text.splitlines():
        m = re.match(r'^\|\s*(G\d)-SC(\d+)\s*\|(.*)\|\s*$', line)
        if not m:
            continue
        cells = [c.strip() for c in m.group(3).split('|')]
        gate = m.group(1)
        crit.setdefault(gate, []).append({'id': f'{gate}-SC{m.group(2)}', 'criterion': cells[0], 'evidence': cells[1],
                                          'threshold': cells[2], 'class': cells[3]})
    data = {'about': ('The standard success criteria of every gate (NASL-EMS-HBK-001 section 4.4, version ' + ver + '). '
                      'Generated from the handbook by framework/tools/extract_gate_criteria.py: do not edit by hand. '
                      'class M = mandatory (met, or waived by the owner with a reason), D = desirable. A project copies '
                      'these into its gates, adds its own criteria, and scores each one: met, partly, not met, waived, n/a.'),
            'source': 'framework/handbook/engineering_handbook.md', 'handbook_version': ver,
            'scores': ['met', 'partly', 'not met', 'waived', 'n/a', 'not assessed'],
            'decisions': {'go': 'every mandatory criterion met or waived; no Critical or Major RID open',
                          'go with actions': 'every mandatory criterion met or waived; Major or Minor RIDs open, each on a '
                                             'card with a date; none touches safety',
                          'delta review': 'one or more mandatory criteria not met, closable within the phase',
                          'no-go': 'a basic problem: stay in the phase and re-plan'},
            'gates': crit}
    with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write('\n')
    print(f'{sum(len(v) for v in crit.values())} criteria for {len(crit)} gates -> {OUT}')


if __name__ == '__main__':
    main()
