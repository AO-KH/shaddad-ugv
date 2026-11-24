"""Generate and build the system requirements specification NASL-SHD-REQ-008 from the requirements register.

usage:
  python tools/reports/build_requirements_spec.py              write the source, build the Word file and the PDF,
                                                                then check the PDF
  python tools/reports/build_requirements_spec.py --no-pdf     write the source and build the Word file only
  python tools/reports/build_requirements_spec.py --md-only    write the source only
  python tools/reports/build_requirements_spec.py --check      check the PDF already built for --date
  option: --date YYYY-MM-DD, the build date (default: today)

What it reads:
  - docs/requirements/system.yaml, the register: the only source of the specification's content (the system and
    the phase, the needs, the functions, the allocation codes, the safety goals, the dated standards and the
    requirements, functional and technical);
  - docs/registers/decisions.json: the titles of the decision rows that the requirements cite (Appendix A);
  - tools/dashboard/project_cards.json: card G1-03's open review decisions, and the titles of the cards in the RID
    list; the questions of the review decisions come from the decision list of the review report NASL-SHD-ANR-001
    (work/reports/review_20261115/report_content.py, read with ast, never run).
What it writes:
  - docs/requirements/NASL_REQ-008_system_requirements.md, the document's source (generated: never edit it);
  - work/reports/requirements/<file>.docx and .pdf, built with tools/reports/build_document.py (Word makes the PDF).

The fixed prose lives here: the cover summary, the summary, the scope, how to read a requirement, the section
texts, and the RID list for G1-05 (RIDS), merged from the integration engineer's records of part 2. For a new
version, set ISSUE (the version, the subtitle, a history row; fix the date of every issued row) and review the
fixed prose and RIDS.

Version 1.0 (part 1) held the functional requirements; version 2.0 (part 2) adds the technical requirements, one
section per kind, the dated standards and the RID list.

Layout: the build runs build_document.make() with these hooks, set for the build only:
  - each requirement's table stays on one page where it fits, with fixed column widths (build_document keeps
    tables of up to six rows only);
  - the other tables of this document get fixed column widths, by their header row; a few short ones are kept on
    one page (KEEP);
  - the scope line of each function, the kind line of each kind and the paragraph right before each table are kept
    whole and with what follows;
  - a section's first requirement table too tall to share a page with its heading runs on to the next page,
    without repeating its first row (FLOW, from est_height);
  - the hyphens of IDs (REQ-SYS-001, G1-04, NASL-SHD-REQ-008, the short "-077" and "-2" references, dates and
    negative values) become non-breaking (U+2011), so that Word never wraps an ID at its hyphen. build_document
    already does this for short IDs such as R-04. "1.0 +/- 0.1" stays on one line (no-break spaces);
  - long editions ("2018+Amd1:2020") may break after their "+", and the paths in the narrow columns of the needs
    table after their slashes and underscores (zero-width spaces).
  The source keeps plain hyphens; a text search of the PDF must allow for U+2011, U+00A0 and U+200B.
The check also flags a section heading that stands apart from its first requirement table, and a reference broken
after a lone hyphen.
Paths: work/agentic/ and tools/agentic/ are written by their team names, work/team/ and tools/team/ (the same
folders).
Needs PyYAML, and PyMuPDF for the check.
"""
import argparse
import ast
import datetime
import json
import os
import re
import sys
from collections import Counter, defaultdict

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import build_document as BD  # noqa: E402
import build_engineering_report as E  # noqa: E402
import nasl_docx as N  # noqa: E402

REGISTER = 'docs/requirements/system.yaml'
SOURCE = 'docs/requirements/NASL_REQ-008_system_requirements.md'
DECISIONS = 'docs/registers/decisions.json'
CARDS = 'tools/dashboard/project_cards.json'
ANR_CONTENT = 'work/reports/review_20261115/report_content.py'
OUTDIR = 'work/reports/requirements'
FOOTER = 'System requirements specification'

ISSUE = {
    'version': '2.0',
    'subtitle': 'The functional and technical requirements',
    'status': 'Approved',
    # one row per version: (version, date, changes); None means the build date (fix it once the version is issued)
    'history': [('1.0', '2026-11-17', 'Part 1, the functional requirements (card G1-01); approved by the project owner '
                                      'on 2026-11-17 ("Approve part 1 (Recommended)")'),
                ('2.0', '2026-11-18', 'Part 2, the technical requirements and the values (card G1-01); approved by the '
                                      'project owner on 2026-11-18 ("Approve part 2 (Recommended)")')],
}

# ------------------------------------------------------------------ fixed prose
REQUEST = ('The project owner, 2026-11-16: "for now we can start G1-01 Re-baseline the system requirements on the '
           'as-built robot but we can work on the functional requirements of the system then we will work on the '
           'technical requirements"; part 2 started on 2026-11-17 ("Start part 2")')
SCOPE_LINE = ('Card G1-01, parts 1 and 2: the functional and technical requirements of the system on the robot as '
              'built')
COVER_SUMMARY = ('What the Shaddad UGV shall do as a system, and the values it shall meet, in each mode and situation '
                 'of this phase: {req} requirements, {nfun} functional under {fun} system functions and {ntec} '
                 'technical of five kinds, {safe} of them safety requirements, traced to {needs} stakeholder needs '
                 'and {goals} safety goals, each with its verification and its state on the robot as built.')
SCOPE_LEAD = '**Scope:**'
KIND_LEAD = '**Kind:**'
KINDS = [  # the register's kind order, with what each kind holds (the shared brief of part 2, section 4)
    ('performance', 'Performance', 'a speed, a time, a distance, an accuracy, an endurance, a slope'),
    ('physical', 'Physical', 'the mass, the size, the pack, the motors and the implements as built'),
    ('environment', 'Environment', 'the ambient temperature, the sun, the dust, the water, the ground and the sites'),
    ('interface', 'Interface', "the operator's RC and console, the dock, the implements, and the links between the "
                               "robot's parts that the operator or the dock sees"),
    ('constraint', 'Constraint', 'a standard, a regulation, a decision of the project owner, the as-built hardware'),
]
CAMPAIGN = {'simulation': 'in simulation', 'bench': 'on the bench', 'field': 'in the field'}
STATES = ('implemented', 'partly', 'not implemented', 'hardware', 'unknown')
STATE_MEANING = {
    'implemented': 'In the twin and its firmware, with a passing test where one exists',
    'partly': 'Met in part; the note names what is missing and the card or STATUS item that closes it',
    'not implemented': 'Not provided today; the note names the card that provides it',
    'hardware': 'Waits on the as-built data of the robot (card G1-04)',
    'unknown': 'Not shown either way yet; the note names the missing evidence',
}
STATE_PHRASE = {'implemented': 'implemented', 'partly': 'partly met', 'not implemented': 'not implemented',
                'hardware': 'waiting on hardware data', 'unknown': 'unknown'}
STATE_TITLE = {'not implemented': 'Not implemented', 'hardware': 'Waiting on hardware data', 'unknown': 'Unknown'}
ROWS_EXPLAINED = [
    ('ID and text', 'The first row: the requirement\'s ID and its text, in bold.'),
    ('Kind', 'Technical requirements only: performance, physical, environment, interface or constraint.'),
    ('Function', 'Technical requirements only: the function it serves, or none for a system-wide requirement.'),
    ('Rationale', 'Why the robot needs it, and for a value where the value comes from and, for safety, its margin.'),
    ('Source', 'What it traces up to: stakeholder needs (N-), safety goals (SG-), decision rows (D), change '
               'requests (CR-) and charter sections.'),
    ('Parent', 'The safety goal that it realises, or none.'),
    ('Allocation', 'The parts of the system that provide it: the codes of Appendix B.'),
    ('Safety', 'Yes for a safety requirement, one that realises a safety goal.'),
    ('TBD', 'A value still to set, with its owner and when; or none.'),
    ('Verification', 'The method, the level ({levels}) and the campaign ({campaigns}), then the reference: the '
                     'scenario, the test, or the card that plans it.'),
    ('As built', 'The state on the robot as built ({{ref:states}}).'),
    ('Where', 'The part of the twin or its firmware that provides it today.'),
    ('Note', 'The evidence, and the card or STATUS item that closes a gap.'),
]
# The RIDs for G1-05: what the review G1-05 must still see closed, by the card that closes it. Merged from the
# integration engineer's records of part 2: work/team/g1-01/p2/integration/report_step13.md section 4.1 (the decided
# values pending on a card), report_step19.md section 7 (the new pending marks) and report_step19c.md (REQ-SYS-209
# and -210, the open mark on -114, -165 and -205, and the points on -044, -126, -127, -149 and -155). REQ-SYS-059's
# top-up moved to -208 in step 19. The IDs are checked against the register at every build; a scenario criterion
# without a requirement of its own is named in words.
RIDS = [
    ('G2-GNC', [21, 22, 26, 27, 40, 87, 88, 97, 99, 113, 117, 122, 126, 127, 131, 133, 135, 177, 207, 209],
     "in the scenarios, S4's inset, S5's VFH and S5b's hold"),
    ('G2-NAV', [20, 21, 23, 95, 96, 124], ''),
    ('G2-PWR', [48, 54, 55, 60, 101, 103, 136, 138, 162, 163, 164, 204, 208], ''),
    ('G2-MCU', [31, 54, 69, 77, 112, 115, 116, 209, 210], ''),
    ('G2-DRV', [112, 145, 155, 166], ''),
    ('G2-FSC', [46, 48, 95, 98, 111, 114, 131, 205, 209], ''),
    ('G2-INT', [20, 33, 87, 97, 112, 123, 133, 143],
     "the criteria of the scenario scripts: S2's speed and window, S4's clearance, S5's pedestrian, head-on case "
     "and mower run, S5b's scripted resume, S6, and S7's 0.25 m and 5 s values"),
    ('G2-COM', [12, 142], ''),
    ('G2-STR', [44, 149, 155], ''),
    ('T-04', [130], ''),
    ('G1-04', [31, 44, 54, 55, 60, 63, 69, 70, 73, 81, 103, 108, 109, 114, 149, 150, 153, 158, 160, 165, 205],
     'the values provisional under D96, final with the delivered electronics\' data'),
]
D103_UNCONFIRMED = [33, 46, 48, 98]  # D103: set by risk assessment, not yet confirmed against the standards

# ------------------------------------------------------------------ the tables: header rows and column widths
HEAD = {
    'counts': ('Item', 'Count', 'Section'),
    'rows': ('Row', 'What it holds'),
    'needs': ('ID', 'Stakeholder', 'Need', 'Source', 'Traced in'),
    'functions': ('ID', 'Function', 'Scope', 'Functional', 'Technical'),
    'goals': ('ID', 'Goal', 'Safe state', 'PL', 'Hazards'),
    'standards': ('ID', 'Standard', 'Title', 'Edition', 'Binding', 'Cited by'),
    'need_trace': ('Need', 'Requirements', 'Count'),
    'goal_trace': ('Goal', 'Requirements', 'Count'),
    'allocation': ('Code', 'Part of the system', 'Functional', 'Technical', 'Safety requirements'),
    'function_counts': ('Function', 'Requirements', 'Safety', 'With a TBD', 'Implemented', 'Partly',
                        'Not implemented', 'Hardware', 'Unknown'),
    'kind_counts': ('Kind', 'Requirements', 'Safety', 'With a TBD', 'Implemented', 'Partly',
                    'Not implemented', 'Hardware', 'Unknown'),
    'states': ('State', 'Meaning', 'Requirements', 'Safety requirements'),
    'state_list': ('Requirement', 'Function or kind', 'Safety', 'Cards'),
    'rids': ('Card', 'What waits on it', 'Requirements'),
    'tbd_left': ('Requirement', 'Kind', 'The value to set', 'Owner', 'By'),
    'open_decisions': ('Decision', 'The question', 'Review items'),
    'decisions': ('Decision', 'Title', 'Cited by'),
    'codes': ('Code', 'What it covers'),
}
WIDTHS = {  # dxa; each row adds up to the text width, 9026
    HEAD['counts']: [4300, 3200, 1526],
    HEAD['rows']: [1900, 7126],
    HEAD['needs']: [700, 1750, 3300, 1950, 1326],
    HEAD['functions']: [700, 2000, 4026, 1150, 1150],
    HEAD['goals']: [700, 4000, 1926, 500, 1900],
    HEAD['standards']: [750, 1500, 2526, 1400, 1600, 1250],
    HEAD['need_trace']: [900, 6826, 1300],
    HEAD['goal_trace']: [900, 6826, 1300],
    HEAD['allocation']: [850, 3526, 1500, 1500, 1650],
    HEAD['function_counts']: [950, 1350, 800, 800, 1250, 800, 1250, 950, 876],
    HEAD['kind_counts']: [1300, 1250, 750, 750, 1150, 750, 1150, 950, 976],
    HEAD['states']: [1750, 4076, 1450, 1750],
    HEAD['state_list']: [1500, 2400, 900, 4226],
    HEAD['rids']: [1100, 2700, 5226],
    HEAD['tbd_left']: [1350, 1250, 3300, 2026, 1100],
    HEAD['open_decisions']: [1300, 5226, 2500],
    HEAD['decisions']: [1300, 3926, 3800],
    HEAD['codes']: [1100, 7926],
}
KEEP = {HEAD['rows'], HEAD['allocation'], HEAD['function_counts'], HEAD['kind_counts']}  # kept on one page
PATH_TABLES = {HEAD['needs']}  # tables whose narrow columns hold paths longer than the column
REQ_WIDTHS = [1650, 7376]
REQ_CELL = re.compile('^REQ\u2011SYS\u2011\\d{3}$')
# A section's first requirement table stays with the section's heading and its scope or kind line. When the table
# is too tall to share a page with them, Word would leave the heading alone on its page, so that table may run on
# to the next page; its first row is not repeated there, so that its ID stands once as a first cell, and the label
# column names the rows that run on. The estimate is calibrated on the v2.0 build: it reads about
# 1.1 times the measured height, and 660 pt of estimate is about the usable page less the heading.
FLOW_AT = 660
FLOW = set()  # filled by requirements() and technical(): the first tables estimated taller than FLOW_AT
INTROS = set()  # filled by Body.table(): the paragraphs right before a table, kept whole and with their table


# ------------------------------------------------------------------ helpers
def one(s):
    """The register's folded text on one line, with the folders by their team names."""
    s = ' '.join(str(s).split())
    return s.replace('work/agentic/', 'work/team/').replace('tools/agentic/', 'tools/team/')


def cell(s):
    return one(s).replace('|', '\\|')


def row(cells):
    return '| ' + ' | '.join(cells) + ' |'


def and_join(items):
    items = list(items)
    return items[0] if len(items) == 1 else ', '.join(items[:-1]) + ' and ' + items[-1]


def safety_phrase(n, k):
    head = f'{n} requirement' + ('' if n == 1 else 's')
    if k == 0:
        return head + (', none a safety requirement' if n > 1 else ', not a safety requirement')
    if k == n:
        return head + (', a safety requirement' if n == 1 else ', both safety requirements' if n == 2
                       else ', all of them safety requirements')
    return head + (', one of them a safety requirement' if k == 1 else f', {k} of them safety requirements')


def rid(n):
    return f'REQ-SYS-{n:03d}'


def id_ranges(ids):
    """REQ-SYS-001 to -117 and REQ-SYS-207 to -210, from a list of IDs."""
    nums = sorted(int(i[-3:]) for i in ids)
    runs, start = [], nums[0]
    for a, b in zip(nums, nums[1:] + [None]):
        if b != a + 1:
            runs.append(rid(start) if a == start else f'{rid(start)} to -{a:03d}')
            start = b
    return and_join(runs)


def is_tech(r):
    return 'kind' in r


def has_tbd(r):
    return one(r['tbd']) != 'none'


class Body:
    """The markdown lines of the body, with {{n:key}} table numbers and {{ref:key}} references resolved last."""

    def __init__(self):
        self.lines = []

    def add(self, *lines):
        self.lines.extend(lines)

    def table(self, key, caption, rows, head=None):
        head = head or HEAD[key]
        before = next((x for x in reversed(self.lines) if x.strip()), '')
        if not re.match(r'^(#|\||\s*[-*+] |\s*\d+[.)] |Table \d|<!--|>)', before):
            INTROS.add(before.strip())
        if caption:
            self.add(f'Table {{{{n:{key}}}}}. {caption}')
        self.add(row(head), row(['---'] * len(head)), *[row(r) for r in rows], '')

    def text(self):
        out, num = '\n'.join(self.lines), {}
        for key in re.findall(r'\{\{n:(\w+)\}\}', out):
            num.setdefault(key, len(num) + 1)
        out = re.sub(r'\{\{n:(\w+)\}\}', lambda m: str(num[m.group(1)]), out)
        return re.sub(r'\{\{ref:(\w+)\}\}', lambda m: f'Table {num[m.group(1)]}', out)


# ------------------------------------------------------------------ inputs
def load():
    R = yaml.safe_load(open(os.path.join(ROOT, REGISTER), encoding='utf-8'))
    dec = {d['id']: d for d in json.load(open(os.path.join(ROOT, DECISIONS), encoding='utf-8'))['decisions']}
    cards = {c['id']: c for c in json.load(open(os.path.join(ROOT, CARDS), encoding='utf-8'))['cards']}
    tree = ast.parse(open(os.path.join(ROOT, ANR_CONTENT), encoding='utf-8').read())
    anr = next(ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'DECISIONS')
    return R, dec, cards, {d[0]: d for d in anr}


def fields(r):
    """The texts of a requirement that the document prints."""
    v, ab = r['verification'], r['as_built']
    return [r['text'], r['rationale'], ' '.join(map(str, r['source'])), str(r['parent']), str(r['tbd']),
            v['reference'], ab['where'], ab['note']]


def tbd_parts(t):
    """A TBD as (the value to set, its owner, by when), with or without its [value: part 2] mark."""
    t = re.sub(r'^\[value: part 2\]\s*', '', one(t))
    m = re.match(r'^(.*?);\s*owner (.*?);\s*(?:by|before) (.*)$', t)
    return m.groups() if m else (t, '', '')


def est_height(r):
    """The height of a requirement's table in points, estimated from the length of its texts (about 74 bold and
    84 plain characters to a line of the value column, 11.6 pt a line, 6 pt of padding a row)."""
    v, ab = r['verification'], r['as_built']
    vals = [one(r['rationale']), ', '.join(map(str, r['source'])), str(r['parent']), ', '.join(r['allocation']),
            'Yes', one(r['tbd']), one(v['reference']) + ' ' * 28, ab['state'], one(ab['where']), one(ab['note'])]
    if is_tech(r):
        vals = ['kind', 'a function name of some length'] + vals
    lines = -(-len(one(r['text'])) // 74) + sum(max(1, -(-len(x) // 84)) for x in vals)
    return lines * 11.6 + (len(vals) + 1) * 6


def flow_if_tall(mine):
    if mine and est_height(mine[0]) > FLOW_AT:
        FLOW.add(mine[0]['id'])


def doc_order(R):
    """The requirements in the order of the document: the functional ones by function (section 5), then the
    technical ones by kind (section 6), each in the register's order."""
    reqs = R['requirements']
    out = [r for f in R['functions'] for r in reqs if not is_tech(r) and r['function'] == f['id']]
    out += [r for k, _, _ in KINDS for r in reqs if is_tech(r) and r['kind'] == k]
    return out


# ------------------------------------------------------------------ the document
def counts(R):
    reqs = R['requirements']
    fun = [r for r in reqs if not is_tech(r)]
    tec = [r for r in reqs if is_tech(r)]
    tbd = [r for r in reqs if has_tbd(r)]
    c = {'req': len(reqs), 'fun': len(R['functions']), 'nfun': len(fun), 'ntec': len(tec),
         'safe': sum(r['safety'] for r in reqs), 'safe_fun': sum(r['safety'] for r in fun),
         'safe_tec': sum(r['safety'] for r in tec), 'needs': len(R['needs']), 'goals': len(R['safety_goals']),
         'tbd': len(tbd), 'tbd_safe': sum(r['safety'] for r in tbd), 'std': len(R.get('standards', [])),
         'state': Counter(r['as_built']['state'] for r in reqs),
         'kind': Counter(r['kind'] for r in tec),
         'status': Counter(r.get('status', '') for r in reqs),
         'part1': [n['id'] for n in R['needs'] if str(n['traced_in']).startswith('part 1')],
         'fun_ids': [r['id'] for r in fun], 'tec_ids': [r['id'] for r in tec]}
    c['part2_only'] = [n['id'] for n in R['needs'] if n['id'] not in c['part1']]
    c['needs_traced'] = all(any(n['id'] in r['source'] for r in reqs) for n in R['needs'])
    return c


def or_join(items):
    items = list(items)
    return items[0] if len(items) == 1 else ', '.join(items[:-1]) + ' or ' + items[-1]


def header(c, date):
    ver = ISSUE['version']
    file = f'NASL_REQ-008_Shaddad_System_Requirements_{date}_v{ver}'
    keys = [('doc_no', 'NASL-SHD-REQ-008'), ('doc_type', 'REQUIREMENTS SPECIFICATION'),
            ('title', 'Shaddad UGV: system requirements specification'), ('subtitle', ISSUE['subtitle']),
            ('summary', COVER_SUMMARY.format(**c)), ('version', ver), ('date', date), ('status', ISSUE['status'])]
    keys += [('history', f'{v} | {d or date} | {ch}') for v, d, ch in ISSUE['history']]
    keys += [('prepared_by', 'the lead systems engineer, with the Shaddad engineering team'),
             ('reviewer', 'the independent reviewer'), ('approver', 'the project owner'),
             ('entity', 'NASL Technologies'), ('classification', 'Internal'), ('footer', FOOTER), ('file', file),
             ('outdir', OUTDIR), ('request', REQUEST), ('scope', SCOPE_LINE),
             ('related', 'the charter NASL-SHD-CHR-001; the hazard log NASL-SHD-HAZ-001; the safety plan '
                         'NASL-SHD-SAF-001; the decision log (docs/architecture.md); card G1-01'),
             ('chapter_breaks', 'yes'), ('cover_image', 'none')]
    return file, '<!--\n' + ''.join(f'{k}: {v}\n' for k, v in keys) + '-->\n'


def summary(b, R, c):
    reqs = R['requirements']
    st = c['state']
    b.add('## Summary', '',
          '**This specification states what the Shaddad UGV shall do as a system, and the values it shall meet.** '
          'It holds the functional requirements of part 1 of card G1-01, approved at version 1.0, and adds the '
          'technical requirements and the values of part 2. It is generated from the requirements register '
          '`docs/requirements/system.yaml` and issued for the project owner\'s approval.', '')
    b.table('counts', 'The specification in counts.', [
        ['Stakeholder needs', f"{c['needs']}, " + ('every one traced to a requirement' if c['needs_traced']
                                                   else 'not all traced to a requirement'), '2, 8.1'],
        ['System functions', str(c['fun']), '3'],
        ['Safety goals, the top-level safety requirements', str(c['goals']), '4'],
        ['Functional requirements', f"{c['nfun']}: {id_ranges(c['fun_ids'])}", '5'],
        ['Technical requirements', f"{c['ntec']}, of five kinds: {id_ranges(c['tec_ids'])}", '6'],
        ['Safety requirements', f"{c['safe']}: {c['safe_fun']} functional and {c['safe_tec']} technical",
         '5, 6, 8.2'],
        ['Requirements with a value still to set (TBD)',
         f"{c['tbd']}, " + ('none a safety requirement' if not c['tbd_safe'] else
                            f"{c['tbd_safe']} of them safety requirements"), '10.2'],
        ['Standards, each dated', str(c['std']), '7']])
    meth = Counter(r['verification']['method'] for r in reqs)
    camp = Counter(r['verification']['campaign'] for r in reqs)
    b.add('- **By kind** (section 6): ' + and_join(f"{c['kind'][k]} {k}" for k, _, _ in KINDS) + '.',
          '- **On the robot as built** (section 9): ' + and_join(
              f'{st[s]} {STATE_PHRASE[s]}' for s in STATES if st[s]) + '.',
          '- **Verification:** ' + and_join(f'{n} by {m.lower()}' for m, n in meth.most_common()) + '; '
          + and_join(f'{n} {CAMPAIGN.get(k, k)}' for k, n in camp.most_common()) + '.',
          f"- **Open items** (section 10): the RIDs for the review G1-05, by the card that closes them; the "
          f"{c['tbd']} values still to set; and the review decisions still open on card G1-03.", '')


def scope(b, R, c):
    reqs = R['requirements']
    levels = [k for k in ('unit', 'subsystem', 'system', 'PIL/HIL', 'field')
              if any(r['verification']['level'] == k for r in reqs)]
    camps = [k for k in ('simulation', 'bench', 'field') if any(r['verification']['campaign'] == k for r in reqs)]
    agreed, draft = c['status'].get('agreed', 0), c['status'].get('draft', 0)
    # part 1 was REQ-SYS-001 to -114; the functional requirements above it were added in part 2
    new_fun = sorted(r['id'] for r in reqs if not is_tech(r) and int(r['id'].rsplit('-', 1)[1]) > 114)
    fun_tbd = sorted(r['id'] for r in reqs if not is_tech(r) and has_tbd(r))
    b.add('## 1. Scope', '', '### 1.1 The system', '', one(R['system']), '', '### 1.2 This phase', '',
          one(R['phase']), '', '### 1.3 Part 1 and part 2', '',
          '- **Part 1, version 1.0,** approved by the project owner on 2026-11-17: the functional requirements. '
          'Each says what the robot shall do, in which mode or situation, and with what result, seen at the '
          'system\'s boundary: the robot\'s motion and implement, the operator\'s RC and console, the dock, and the '
          'people nearby. A requirement does not say how; the part of the twin that provides it today is in its '
          '"Where" row.',
          f"- **Part 2, this version:** the technical requirements, {c['ntec']} of them, each with a value that a "
          'test, an analysis, an inspection or a demonstration can judge; the values that part 1 left to part 2; '
          f"and the {c['std']} standards, each dated (D103). Part 2 also added {len(new_fun)} functional "
          f"requirements ({id_ranges(new_fun)})" + (
              f", and of part 1's 51 values marked `[value: part 2]` it set all but "
              f"{and_join(fun_tbd)}'s" if fun_tbd else ", and set every value that part 1 left to it") + '.',
          (f'- **In the register,** the {agreed} requirements of version 1.0 are agreed and the {draft} of part 2 '
           'are draft, until the approval of this version.' if draft else
           f'- **In the register,** all {agreed} requirements are agreed: the functional requirements of version 1.0 '
           'since 2026-11-17, and the whole baseline since the project owner\'s approval of 2026-11-18 (D110).'),
          ('- **Approval:** the project owner approved the functional requirements at version 1.0, and approves the '
           'whole baseline at this version.' if draft else
           '- **Approval:** the project owner approved the functional requirements at version 1.0 on 2026-11-17 '
           '("Approve part 1 (Recommended)"), and the whole baseline at this version on 2026-11-18 ("Approve part 2 '
           '(Recommended)"; D110).'), '',
          '### 1.4 How to read a requirement', '',
          '- **One "shall" in each requirement,** in one of five patterns (EARS):',
          '  - "The robot shall ...";',
          '  - "When &lt;event&gt;, the robot shall ...";',
          '  - "While &lt;state&gt;, the robot shall ...";',
          '  - "If &lt;unwanted event&gt;, then the robot shall ...";',
          '  - "Where &lt;feature&gt;, the robot shall ...".',
          '- **"The robot"** means the whole system of section 1.1, as in the safety goals.',
          '- **The quality rules:** each requirement is necessary, singular, unambiguous, verifiable, feasible on '
          'the robot as built, and traceable up and down (NASL-EMS-HBK-001 section 7.2, from ISO/IEC/IEEE 29148).',
          '- **The rows of a requirement** are those of {{ref:rows}}.', '')
    b.table('rows', 'The rows of a requirement table.', [
        [f'**{k}**', v.replace('{levels}', or_join(levels)).replace('{campaigns}', or_join(camps))]
        for k, v in ROWS_EXPLAINED])
    b.add('### 1.5 How to read a technical requirement', '',
          '- **A system requirement with a value:** "The robot shall ..." with a number and a unit that a test, an '
          'analysis, an inspection or a demonstration can judge. Like a functional requirement, it is about the '
          'whole robot as built, seen at its boundary.',
          '- **Its kind:**',
          *[f'  - {k}: {d};' if i < len(KINDS) - 1 else f'  - {k}: {d}.' for i, (k, _, d) in enumerate(KINDS)],
          '- **Its function:** the function it serves, or none for a system-wide requirement.',
          '- **Where its value comes from** is in its rationale: a decision row or a change request; the charter or '
          'a need; a standard, with its edition; the parameters, with the leaf `P.x.y` and its file; or a test or '
          'scenario result, with its date.',
          '- **A safety value** states its margin and why that margin is safe. No safety requirement carries a TBD.',
          '- **A decided value that the twin does not hold yet** is marked "decided, pending", with the card that '
          'changes the twin; the note keeps today\'s value, so that every run until the change can still be read. '
          'Each such value is a RID for the review G1-05 (section 10.1).',
          '- **A provisional value** rests on the delivered electronics (D96) and becomes final with card G1-04\'s '
          'data. A value set by risk assessment says that it is not yet confirmed against the standards (D103).', '')
    b.add('### 1.6 How this document is made', '',
          'The register `docs/requirements/system.yaml` is the controlled source of the requirements and the '
          'standards. The script `tools/reports/build_requirements_spec.py` writes the source of this document, '
          '`docs/requirements/NASL_REQ-008_system_requirements.md`, from the register, and builds the Word file '
          'and the PDF with `tools/reports/build_document.py`. To change a requirement, change the register and '
          'rebuild; never edit this document. The RID list of section 10.1 merges the integration engineer\'s '
          'records of part 2, and the open decisions come from card G1-03 and the review report NASL-SHD-ANR-001. '
          'The decision titles of Appendix A come from the decision index `docs/registers/decisions.json`.', '')


def needs_functions_goals(b, R, c):
    reqs = R['requirements']
    b.add('## 2. Stakeholder needs', '',
          f"The {c['needs']} needs itemise Table 1 of the project management plan NASL-SHD-PMP-001 and the charter "
          'NASL-SHD-CHR-001 v1.3. "Traced in" says what meets the need: the functional requirements of part 1, or '
          f"the technical requirements and the plan of part 2. Of the needs, {len(c['part1'])} are traced in part 1, "
          f"and {and_join(c['part2_only'])} in part 2 only. Section 8.1 traces each need to its requirements.", '')
    b.table('needs', 'The stakeholder needs.', [
        [n['id'], cell(n['stakeholder']), cell(n['need']), cell(n['source']), cell(n['traced_in'])]
        for n in R['needs']])
    per_f = Counter(r['function'] for r in reqs if not is_tech(r))
    per_t = Counter(r['function'] for r in reqs if is_tech(r))
    wide = per_t.get('none', 0)
    b.add('## 3. The system functions', '',
          f"Each functional requirement belongs to one of the {c['fun']} functions, and each technical requirement "
          f"names the function it serves; {wide} technical requirements serve none, as they are system-wide. "
          '{{ref:functions}} gives the scope of each function and its requirements; section 5 follows the same '
          'order.', '')
    b.table('functions', 'The system functions.',
            [[f['id'], cell(f['name']), cell(f['scope']), str(per_f[f['id']]), str(per_t[f['id']])]
             for f in R['functions']]
            + [['', 'System-wide', '', '', str(wide)],
               ['**Total**', '', '', f"**{c['nfun']}**", f"**{c['ntec']}**"]])
    b.add('## 4. The safety goals', '',
          f"The {c['goals']} safety goals of the hazard log NASL-SHD-HAZ-001 v1.2 are the system's top-level "
          'safety requirements (D80). They stand here word for word, with their safe states; the hazard log is '
          'their source and owner. Section 8.2 traces them to the requirements that realise them.', '')
    b.table('goals', 'The safety goals.', [
        [g['id'], cell(g['text']), cell(g['safe_state']), cell(g['pl']), ', '.join(g['hazards'])]
        for g in R['safety_goals']])
    by = defaultdict(list)
    for g in R['safety_goals']:
        by[g['status_in_hazard_log']].append(g['id'])
    parts = [(f"{len(v)} goals are {k} ({', '.join(v)})" if len(v) > 1 else f'{v[0]} is {k}')
             for k, v in sorted(by.items(), key=lambda kv: -len(kv[1]))]
    b.add(f'In the hazard log, {and_join(parts)}.', '')


def requirement(b, r, fnames):
    v, ab = r['verification'], r['as_built']
    tbd = one(r['tbd'])
    rows = []
    if is_tech(r):
        f = r.get('function', 'none')
        rows += [('Kind', r['kind'][:1].upper() + r['kind'][1:]),
                 ('Function', 'None: system-wide' if f == 'none' else f'{f} {fnames.get(f, "")}'.strip())]
    rows += [('Rationale', cell(r['rationale'])),
             ('Source', ', '.join(map(str, r['source']))),
             ('Parent', 'None' if r['parent'] == 'none' else str(r['parent'])),
             ('Allocation', ', '.join(r['allocation'])),
             ('Safety', 'Yes' if r['safety'] else 'No'),
             ('TBD', 'None' if tbd == 'none' else cell(tbd)),
             ('Verification', f"**{v['method']}**, {v['level']} level, {CAMPAIGN.get(v['campaign'], v['campaign'])}: "
                              f"{cell(v['reference'])}"),
             ('As built', f"**{ab['state'][:1].upper() + ab['state'][1:]}**"),
             ('Where', cell(ab['where'])),
             ('Note', cell(ab['note']))]
    b.add(row([r['id'], cell(r['text'])]), row(['---', '---']), *[row([f'**{k}**', x]) for k, x in rows], '')


def requirements(b, R):
    reqs = R['requirements']
    fnames = {f['id']: one(f['name']) for f in R['functions']}
    b.add('## 5. The functional requirements', '',
          'One section per function, in the order of section 3. Each requirement is one table: the first row holds '
          'its ID and its text, and the rows below hold its attributes ({{ref:rows}}). Within a function the '
          'requirements keep the order of the register. The technical requirements that serve a function are in '
          'section 6, by kind; each names its function.', '')
    for i, f in enumerate(R['functions'], 1):
        mine = [r for r in reqs if not is_tech(r) and r['function'] == f['id']]
        flow_if_tall(mine)
        b.add(f"### 5.{i} {f['id']} {one(f['name'])}", '',
              f"{SCOPE_LEAD} {cell(f['scope'])} ({safety_phrase(len(mine), sum(r['safety'] for r in mine))}.)", '')
        for r in mine:
            requirement(b, r, fnames)


def technical(b, R):
    reqs = R['requirements']
    fnames = {f['id']: one(f['name']) for f in R['functions']}
    b.add('## 6. The technical requirements', '',
          'One section per kind, in the register\'s order. Each requirement is the same table as a functional one, '
          'with two more rows: its kind and the function it serves (section 1.5).', '')
    for i, (k, title, what) in enumerate(KINDS, 1):
        mine = [r for r in reqs if is_tech(r) and r['kind'] == k]
        if not mine:
            continue
        flow_if_tall(mine)
        b.add(f'### 6.{i} {title}', '',
              f"{KIND_LEAD} {what} ({safety_phrase(len(mine), sum(r['safety'] for r in mine))}.)", '')
        for r in mine:
            requirement(b, r, fnames)


def standards(b, R, c):
    reqs = R['requirements']
    cites = defaultdict(set)
    for r in reqs:
        for m in re.findall(r'\bSTD-(\d{2})\b', ' '.join(map(str, fields(r)))):
            cites[f'STD-{m}'].add(r['id'])
    kinds = Counter()
    for s in R['standards']:
        t = one(s['binding']).lower()
        kinds['binding' if t.startswith('binding') else 'to choose' if t.startswith('to confirm')
              else 'state of the art'] += 1
    b.add('## 7. The standards', '',
          f"The {c['std']} standards and regulations that apply, from the register, each dated. D103 sets the "
          'jurisdiction, the Kingdom of Saudi Arabia, with the binding SASO and CST layer in its own column: '
          f"{kinds['binding']} are binding, {kinds['state of the art']} are the state of the art, and "
          f"{kinds['to choose']} are still to choose. ISO 13849-1:2023 stays the rating method. \"Cited by\" names the "
          'requirements whose texts cite the standard by its ID.', '')
    b.table('standards', 'The standards, with their editions and whether they bind.', [
        [s['id'], cell(s['number']), cell(s['title']), cell(s['edition']), cell(s['binding']),
         ', '.join(sorted(cites.get(s['id'], []))) or '-'] for s in R['standards']])


def traceability(b, R, c):
    reqs = R['requirements']
    b.add('## 8. Traceability', '', '### 8.1 The needs and their requirements', '',
          'Each need with the requirements, functional and technical, that name it in their source. '
          + ('Every need is a source of at least one requirement.'
             if all(any(n['id'] in r['source'] for r in reqs) for n in R['needs'])
             else 'A need without a requirement is marked.'), '')
    rows = []
    for n in R['needs']:
        k = sorted(r['id'] for r in reqs if n['id'] in r['source'])
        rows.append([n['id'], ', '.join(k) if k else '**None**', str(len(k))])
    b.table('need_trace', 'The needs and the requirements that trace to them.', rows)
    childless = [g['id'] for g in R['safety_goals'] if not any(r['parent'] == g['id'] for r in reqs)]
    b.add('### 8.2 The safety goals and their requirements', '',
          'Each safety goal with the safety requirements, functional and technical, that realise it: the '
          'requirements whose parent it is. '
          + (f"{and_join(childless)} {'has' if len(childless) == 1 else 'have'} no child requirement."
             if childless else 'Every goal has at least one child requirement.'), '')
    b.table('goal_trace', 'The safety goals and the requirements that realise them.', [
        [g['id'], ', '.join(sorted(r['id'] for r in reqs if r['parent'] == g['id'])) or 'None',
         str(sum(r['parent'] == g['id'] for r in reqs))] for g in R['safety_goals']]
            + [['**Total**', '', f"**{sum(r['parent'] != 'none' for r in reqs)}**"]])
    alloc_f = Counter(a for r in reqs if not is_tech(r) for a in r['allocation'])
    alloc_t = Counter(a for r in reqs if is_tech(r) for a in r['allocation'])
    alloc_s = Counter(a for r in reqs if r['safety'] for a in r['allocation'])
    b.add('### 8.3 The allocation', '')
    b.table('allocation', 'The requirements allocated to each part of the system.', [
        [k, cell(v.split(':')[0]), str(alloc_f[k]), str(alloc_t[k]), str(alloc_s[k])]
        for k, v in R['allocation_codes'].items()])
    b.add('A requirement can be allocated to several parts, so the counts add up to '
          f"{sum(alloc_f.values())} functional and {sum(alloc_t.values())} technical, not {c['nfun']} and "
          f"{c['ntec']}.", '')

    def tally(mine, first):
        n = {'all': len(mine), 'safe': sum(r['safety'] for r in mine), 'tbd': sum(has_tbd(r) for r in mine)}
        n.update(Counter(r['as_built']['state'] for r in mine))
        return [first] + [str(n.get(k, 0)) for k in ('all', 'safe', 'tbd') + STATES], n

    b.add('### 8.4 The functions and the kinds', '')
    rows, tot = [], Counter()
    for f in R['functions']:
        r_, n = tally([r for r in reqs if not is_tech(r) and r['function'] == f['id']], f['id'])
        rows.append(r_)
        tot.update(n)
    rows.append(['**Total**'] + [f'**{tot.get(k, 0)}**' for k in ('all', 'safe', 'tbd') + STATES])
    b.table('function_counts', 'The functional requirements of each function, with their safety, TBD and '
            'as-built counts.', rows)
    rows, tot = [], Counter()
    for k, title, _ in KINDS:
        r_, n = tally([r for r in reqs if is_tech(r) and r['kind'] == k], title)
        rows.append(r_)
        tot.update(n)
    rows.append(['**Total**'] + [f'**{tot.get(k, 0)}**' for k in ('all', 'safe', 'tbd') + STATES])
    b.table('kind_counts', 'The technical requirements of each kind, with their safety, TBD and as-built counts.',
            rows)


def as_built(b, R, c):
    reqs = R['requirements']
    st = c['state']
    sst = Counter(r['as_built']['state'] for r in reqs if r['safety'])
    b.add('## 9. The robot as built', '',
          'Each requirement records its state on the robot as built, from 2026-10-27 onward. The sources are the '
          'as-built record `docs/hardware/real_robot_components.md`, the parameters, the decisions, '
          '`docs/STATUS.md`, the review\'s ranked list and the hazard log. The states rest on the checks of parts 1 '
          'and 2, the test baseline of 2026-11-15 and the dated runs that the notes cite. Part 2 changed documents '
          'only: no model, parameter or test.', '')
    b.table('states', 'The requirements by their state on the robot as built.',
            [[f'**{s[:1].upper() + s[1:]}**', STATE_MEANING[s], str(st[s]), str(sst[s])] for s in STATES]
            + [['**Total**', '', f"**{c['req']}**", f"**{c['safe']}**"]])
    for i, s in enumerate(('not implemented', 'hardware', 'unknown'), 1):
        mine = sorted((r for r in reqs if r['as_built']['state'] == s), key=lambda r: r['id'])
        if not mine:
            continue
        key = 'list_' + s.replace(' ', '_')
        b.add(f'### 9.{i} {STATE_TITLE[s]}', '',
              f"{{{{ref:{key}}}}} lists {'it' if len(mine) == 1 else 'them'}: "
              f"{safety_phrase(len(mine), sum(r['safety'] for r in mine))}, {'' if len(mine) == 1 else 'each '}"
              'with the cards that close it. Each requirement\'s note, in its own table, says what is missing.', '')
        b.table(key, f'The requirements {STATE_PHRASE[s]} on the robot as built.',
                [[r['id'], r['kind'] if is_tech(r) else r['function'], 'Yes' if r['safety'] else 'No',
                  ', '.join(r['cards']) or 'None'] for r in mine], head=HEAD['state_list'])


def open_items(b, R, c, cards, anr):
    reqs = R['requirements']
    known = {r['id'] for r in reqs}
    for card, nums, _ in RIDS:
        missing = [rid(n) for n in nums if rid(n) not in known]
        if missing:
            sys.exit(f'RIDS names {missing} under {card}, which the register does not hold: review RIDS')
    bad = [rid(n) for n in D103_UNCONFIRMED if rid(n) not in known]
    if bad:
        sys.exit(f'D103_UNCONFIRMED names {bad}, which the register does not hold')
    all_rids = sorted({rid(n) for _, nums, _ in RIDS for n in nums})
    osv = sorted(r['id'] for r in reqs if 'open safety value' in ' '.join(map(one, fields(r))))
    b.add('## 10. Open items', '', '### 10.1 The RIDs for G1-05', '',
          'A RID records what the review G1-05 must still see closed. Most are decided values that the twin does '
          'not hold yet, and values provisional until the delivered electronics\' data. '
          f"{{{{ref:rids}}}} gathers them by the card that closes them: {len(all_rids)} requirements, some under "
          'more than one card. Each requirement\'s note says what is pending, and keeps today\'s value.', '')
    b.table('rids', 'The RIDs for G1-05, by the card that closes them.', [
        [card, cell(cards.get(card, {}).get('title', card) + (f'; {note}' if note else '')),
         ', '.join(rid(n) for n in sorted(set(nums)))] for card, nums, note in RIDS])
    if osv:
        b.add(f"- **Open safety values for the project owner:** {and_join(osv)}. Each one's rationale or note "
              'gives the conflict, the options and the card that decides it.')
    b.add(f"- **Set by risk assessment and not yet confirmed against the standards** (D103): "
          f"{and_join(rid(n) for n in D103_UNCONFIRMED)}.", '',
          'Source: `work/team/g1-01/p2/integration/report_step13.md` section 4.1, `report_step19.md` section 7 and '
          '`report_step19c.md`, the integration engineer\'s records of part 2.', '')
    tbd = sorted((r for r in reqs if has_tbd(r)), key=lambda r: r['id'])
    b.add('### 10.2 The values still to set', '',
          f"The requirements carry {len(tbd)} TBDs, " + (
              'none of them on a safety requirement' if not c['tbd_safe'] else
              f"{c['tbd_safe']} of them on safety requirements") + '. Each names the value, its owner and the '
          'event by which it is set, and each goes to G1-05 as a RID.', '')
    b.table('tbd_left', 'The values still to set, with their owners.', [
        [r['id'], r['kind'] if is_tech(r) else 'functional', *[cell(x) for x in tbd_parts(r['tbd'])]] for r in tbd])
    open_ = [int(x) for x in cards['G1-03']['decisions']]
    b.add('### 10.3 The review decisions still open', '',
          f"Card G1-03 still has {len(open_)} of the review's {len(anr)} decisions open. None of them is needed to "
          'state a requirement of this version: where a requirement touches one, it is written with today\'s '
          'behaviour and values. Their options are in the review report NASL-SHD-ANR-001, section 7.', '')
    b.table('open_decisions', 'The review decisions still open on card G1-03.',
            [[str(n), cell(anr[n][1]), cell(anr[n][2])] for n in open_])


def appendices(b, R, dec):
    reqs = R['requirements']
    cites = defaultdict(list)
    for r in reqs:
        for d in sorted({int(m) for m in re.findall(r'\bD(\d+)\b', ' '.join(map(str, fields(r))))}):
            cites[d].append(r['id'])
    cites = {d: sorted(v) for d, v in cites.items()}
    unknown = [f'D{d}' for d in cites if f'D{d}' not in dec]
    if unknown:
        sys.exit(f'the requirements cite {unknown}, which the decision index does not hold')
    b.add('## Appendix A. The decisions cited', '',
          f'The {len(cites)} decision rows that the requirements cite, with their titles from the decision index '
          '`docs/registers/decisions.json`. The rows themselves are in the decisions table of '
          '`docs/architecture.md`.', '')
    b.table('decisions', 'The decision rows that the requirements cite.',
            [[f'D{d}', cell(dec[f'D{d}']['title']), ', '.join(cites[d])] for d in sorted(cites)])
    b.add('## Appendix B. The allocation codes', '',
          'The codes name the components of Shaddad_System and the domains of the plant (`docs/architecture.md` '
          'sections 4 and 5).', '')
    b.table('codes', 'The allocation codes.', [[k, cell(v)] for k, v in R['allocation_codes'].items()])


def write_source(date):
    R, dec, cards, anr = load()
    c = counts(R)
    file, head = header(c, date)
    b = Body()
    b.add('<!-- Generated from docs/requirements/system.yaml by tools/reports/build_requirements_spec.py; '
          'do not edit -->', '# Shaddad UGV: system requirements specification', '')
    summary(b, R, c)
    scope(b, R, c)
    needs_functions_goals(b, R, c)
    requirements(b, R)
    technical(b, R)
    standards(b, R, c)
    traceability(b, R, c)
    as_built(b, R, c)
    open_items(b, R, c, cards, anr)
    appendices(b, R, dec)
    text = head + b.text().rstrip('\n') + '\n'
    with open(os.path.join(ROOT, SOURCE), 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print(f"written {SOURCE}: {len(text.splitlines())} lines, {c['req']} requirements "
          f"({c['nfun']} functional, {c['ntec']} technical)")
    return R, file


# ------------------------------------------------------------------ the build, with the layout hooks
ID_RE = re.compile(r'(?<![\w\u2011-])(?=[\w\u2011-]*\d)[A-Z][A-Z0-9]*(?:[-\u2011][A-Z0-9]+)+(?![\w\u2011-])')
SHORT_RE = re.compile(r'(?:^|(?<=[\s(,;/]))-(?=\d)')       # "REQ-SYS-024, -077", "ISO 18497-1, -2", "-0.61 m"
NUM_RE = re.compile(r'(?<=\d)-(?=\d)')                       # dates and ranges: 2026-11-17, 45-50
DIM_RE = re.compile(r'(?<![\w\u2011-])(\d)-(D)(?![\w\u2011-])')  # 2-D, 3-D
LETTER_RE = re.compile(r'(?<![\w\u2011-])([A-Z])-(?=[a-z])')     # E-stop
PM_RE = re.compile(r' ?\+/- ?')                                   # 1.0 +/- 0.1 stays on one line
AMD_RE = re.compile(r'(?<=\d)\+(?=[A-Za-z])')                     # 2018+Amd1:2020 may break after its "+"
PATH_RE = re.compile(r'[\w.\u2011-]+(?:/[\w.\u2011-]+){2,}')          # a path, in a narrow column of PATH_TABLES


def path_breaks(c):
    """A long path may break after its slashes and underscores (zero-width spaces), so that a narrow column
    does not cut a name in two."""
    if not isinstance(c, str):
        return c
    return PATH_RE.sub(lambda m: re.sub(r'([/_])', '\\1\u200b', m.group(0)), c)


def nobreak(s):
    """Non-breaking hyphens in the IDs of s; code spans (paths, commands) keep theirs."""
    out = []
    for part in re.split(r'(`[^`]*`)', s):
        if not part.startswith('`'):
            part = ID_RE.sub(lambda m: m.group(0).replace('-', '\u2011'), part)
            part = DIM_RE.sub('\\1\u2011\\2', NUM_RE.sub('\u2011', SHORT_RE.sub('\u2011', part)))
            part = LETTER_RE.sub('\\1\u2011', part)
            part = PM_RE.sub(lambda m: m.group(0).replace(' ', '\u00a0').replace('-', '\u2011'), part)
            part = AMD_RE.sub('+\u200b', part)  # a zero-width space: a place to break a long edition
        out.append(part)
    return ''.join(out)


class Layout:
    """The layout hooks of this document (see the module docstring), set for the build only."""

    def __enter__(self):
        self.saved = clean, table, para = E.clean, N.Doc.table, N.Doc.p

        def keep_table(d, head, rows, widths, *a, **kw):
            flow = False
            if head and REQ_CELL.match(str(head[0])):
                flow = str(head[0]).replace('\u2011', '-') in FLOW
                widths, kw['keep_all'] = REQ_WIDTHS, not flow
            else:
                widths = WIDTHS.get(tuple(head or ()), widths)
                if tuple(head or ()) in KEEP:
                    kw['keep_all'] = True
                if tuple(head or ()) in PATH_TABLES:
                    rows = [[path_breaks(c) for c in r] for r in rows]
            out = table(d, head, rows, widths, *a, **kw)
            if flow:  # the parts are the table, then a spacer: its first row is not repeated, so its ID stands once
                d.parts[-2] = d.parts[-2].replace('<w:tblHeader/>', '')
            return out

        intros = {nobreak(clean(t)) for t in INTROS}

        def keep_scope(d, s, keep_next=False, **kw):
            if s.startswith((SCOPE_LEAD, KIND_LEAD)) or s in intros:  # whole, and with what follows
                d.add(d.para(d.inline(s), keep_next=True, keep_lines=True))
                return None
            return para(d, s, keep_next=keep_next, **kw)

        E.clean = lambda s: nobreak(clean(s))
        N.Doc.table, N.Doc.p = keep_table, keep_scope
        return self

    def __exit__(self, *exc):
        E.clean, N.Doc.table, N.Doc.p = self.saved


# ------------------------------------------------------------------ the check of the built PDF
SCAN = re.compile(r'(?i)\bAI\b|agent|claude|\btokens?\b|\bsessions?\b')                    # PRO-001 step 6
SCAN_WIDE = re.compile(r'(?i)\bAI\b|agent|claude|\bopus\b|haiku|sonnet|fable|\btokens?\b|\bsessions?\b|subagent'
                       r'|shaddad-[a-z]')


def norm(s):
    s = s.replace('\u2011', '-').replace('\u00ad', '')
    return ' '.join(re.sub(r'-[ \t]*\n\s*', '-', s).split())


def check(R, pdf):
    import pymupdf
    doc = pymupdf.open(pdf)
    texts = [pg.get_text() for pg in doc]
    full = norm('\n'.join(texts))
    ids = [r['id'] for r in doc_order(R)]
    out = {'pages': len(doc), 'requirements_in_register': len(R['requirements'])}
    # the defining cell of each requirement: its ID in white on the navy first row of its table; a section heading
    # of chapters 5 and 6 must stand on the page where its first requirement table starts
    first, alone, waiting = [], [], []
    for i, pg in enumerate(doc):
        for blk in pg.get_text('dict')['blocks']:
            for ln in blk.get('lines', []):
                t = ''.join(s['text'] for s in ln['spans']).replace('\u2011', '-').strip()
                if re.fullmatch(r'REQ-SYS-\d{3}', t) and ln['spans'][0]['color'] == 0xFFFFFF:
                    first.append(t)
                    alone += [(p, h) for p, h in waiting if p != i + 1]
                    waiting = []
                if ln['spans'][0]['size'] >= 13.5 and re.match(r'^[56]\.\d+ ', t):
                    waiting.append((i + 1, t[:40]))
    alone += waiting
    n = Counter(first)
    out['ids_first_cell'] = len(n)
    out['ids_missing'] = [i for i in ids if n[i] == 0]
    out['ids_twice'] = [i for i in ids if n[i] > 1]
    out['ids_not_in_register'] = sorted(set(first) - set(ids))
    out['ids_in_document_order'] = first == ids
    out['ids_anywhere'] = len(set(re.findall(r'REQ-SYS-\d{3}', full)) & set(ids))
    out['headings_alone'] = alone
    out['goals_word_for_word'] = sum(norm(g['text']) in full for g in R['safety_goals'])
    out['goals_missing'] = [g['id'] for g in R['safety_goals'] if norm(g['text']) not in full]
    meta = ' '.join(str(v) for v in doc.metadata.values()) + ' ' + (doc.get_xml_metadata() or '')
    out['scan_text'] = [(i + 1, m.group()) for i, t in enumerate(texts) for m in SCAN.finditer(t)]
    out['scan_wide_text'] = [(i + 1, m.group()) for i, t in enumerate(texts) for m in SCAN_WIDE.finditer(t)]
    out['scan_properties'] = [m.group() for m in SCAN_WIDE.finditer(meta)]
    foot = re.compile(r'NASL Technologies \| ' + re.escape(FOOTER) + r' \| Internal')
    out['blank_pages'] = [i + 1 for i, t in enumerate(texts) if len(re.sub(r'[\s\d]', '', foot.sub('', t))) < 20]
    ends = []
    for i, t in enumerate(texts):
        lines = t.split('\n')
        for a, z in zip(lines, lines[1:]):
            if a.rstrip().endswith('-') and z.strip()[:1].isalnum():
                ends.append((i + 1, a.rstrip()[-24:] + ('|digit' if z.strip()[:1].isdigit() else '')))
    out['lines_ending_in_a_hyphen'] = [(p, e.replace('|digit', '')) for p, e in ends]
    out['id_wrapped_at_hyphen'] = [
        (p, e.replace('|digit', '')) for p, e in ends
        if re.search(r'\b[A-Z][A-Z0-9]*(-[A-Z0-9]+)*-(\|digit)?$|/[\w.]+-(\|digit)?$|(^|[\s(,;])-\|digit$', e)]
    out['toc_filled'] = 'The contents are filled in when Word updates the fields.' not in full
    ok = (not out['ids_missing'] and not out['ids_twice'] and not out['ids_not_in_register']
          and out['ids_in_document_order'] and not out['headings_alone'] and not out['goals_missing']
          and not out['scan_wide_text']
          and not out['scan_properties'] and not out['blank_pages'] and not out['id_wrapped_at_hyphen']
          and out['toc_filled'])
    for k, v in out.items():
        print(f'{k}: {v}')
    print('check:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='Generate and build NASL-SHD-REQ-008 from the requirements register.')
    ap.add_argument('--date', default=datetime.date.today().isoformat(), help='the build date, YYYY-MM-DD')
    ap.add_argument('--no-pdf', action='store_true', help='build the Word file only')
    ap.add_argument('--md-only', action='store_true', help='write the source only')
    ap.add_argument('--check', action='store_true', help='check the PDF already built for --date')
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    pdf = os.path.join(ROOT, OUTDIR, f"NASL_REQ-008_Shaddad_System_Requirements_{a.date}_v{ISSUE['version']}.pdf")
    if a.check:
        sys.exit(check(load()[0], pdf))
    R, file = write_source(a.date)
    if a.md_only:
        return
    with Layout():
        BD.make(SOURCE, pdf=not a.no_pdf)
    if not a.no_pdf:
        sys.exit(check(R, pdf))


if __name__ == '__main__':
    main()
