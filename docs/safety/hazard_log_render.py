"""Render the hazard log record docs/safety/hazard_log.json as the controlled document docs/safety/hazard_log.md.

The JSON file is the record; the markdown is generated from it (never edit hazard_log.md by hand). Build the PDF with
    python tools/reports/build_document.py docs/safety/hazard_log.md
usage:
    python docs/safety/hazard_log_render.py [--check]
--check only verifies the record and prints the counts:
  - each PLr is the risk graph's value for its S, F and P;
  - each hazard traces to its goals and back, and each goal takes the highest level of its hazards;
  - each goal lists exactly the safety functions of its hazards;
  - each "met today" is one of the defined classes, and each goal's status follows from its hazards;
  - each walk-through cell lists exactly the mowing hazards whose cases, modes and places put them there;
  - each goal with a PL is in a class-1 escalation, and every escalation names known goals;
  - each escalation has a defined state, and each open one names the card that works on it.
"""
import json
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'hazard_log.json')
OUT = os.path.join(HERE, 'hazard_log.md')
ORDER = 'abcde'
PLACES = ['greenhouse aisles', 'palm groves', 'the slope test ground', 'the dock and the charging place']
ANY_PLACE, ANY_MODE = 'any place', 'any mode'
E_STATES = ('open', 'open, its question answered', 'decided')
RANK = {'no': 0, 'unknown': 1, 'partly': 2, 'met': 3}
GOAL_STATUS = {'no': 'not met', 'unknown': 'unknown', 'partly': 'partly met', 'met': 'met'}


def sfkey(s):
    return int(s.split('-')[1])


def derived_cell(hazards, flt, place):
    """The mowing hazards that arise in a walk-through cell: the case, the modes and the place all match."""
    out = []
    cases = flt.get('case')
    cases = [cases] if isinstance(cases, str) else cases
    for h in hazards:
        if h['phase'] != 'mowing':
            continue
        if cases and not set(cases) & set(h['cases']):
            continue
        if flt.get('modes') and not (set(flt['modes']) & set(h['modes']) or ANY_MODE in h['modes']):
            continue
        if place in h['places'] or ANY_PLACE in h['places']:
            out.append(h['id'])
    return out


def goal_status(goal, H):
    if goal['plr'] == 'n/a':
        return 'open'
    worst = min((H[x]['meets'] for x in goal['hazards'] if H[x]['meets'] in RANK), key=lambda m: RANK[m])
    return GOAL_STATUS[worst]


def check(d):
    H = {h['id']: h for h in d['hazards']}
    G = {g['id']: g for g in d['safety_goals']}
    F = {f['id'] for f in d['safety_functions']}
    graph, errs = d['method']['graph'], []
    classes = set(d['met_classes'])
    for h in d['hazards']:
        if h['plr'] != 'n/a' and graph[f"{h['S']} {h['F']} {h['P']}"] != h['plr']:
            errs.append(f"{h['id']}: PLr {h['plr']} is not the graph's for {h['S']} {h['F']} {h['P']}")
        if h['meets'] not in classes and h['phase'] == 'mowing':
            errs.append(f"{h['id']}: met today '{h['meets']}' is not a defined class")
        if (h['plr'] == 'n/a') != (h['meets'] == 'by other measures') and h['phase'] == 'mowing':
            errs.append(f"{h['id']}: PLr n/a goes with 'by other measures' and only with it")
        for f in h['functions']:
            if f not in F:
                errs.append(f"{h['id']}: unknown function {f}")
        for g in h['goals']:
            if g not in G:
                errs.append(f"{h['id']}: unknown goal {g}")
            elif h['phase'] == 'mowing' and h['id'] not in G[g]['hazards']:
                errs.append(f"{h['id']}: not listed under {g}")
    for g in G.values():
        hs = [H[x] for x in g['hazards'] if x in H]
        if len(hs) != len(g['hazards']):
            errs.append(f"{g['id']}: lists an unknown hazard")
        if any(g['id'] not in h['goals'] for h in hs):
            errs.append(f"{g['id']}: a hazard does not trace back")
        lv = [h['plr'] for h in hs if h['plr'] != 'n/a']
        top = max(lv, key=ORDER.index) if lv else 'n/a'
        if top != g['plr']:
            errs.append(f"{g['id']}: PLr {g['plr']}, its hazards reach {top}")
        fns = sorted({f for h in hs for f in h['functions']}, key=sfkey)
        if fns != g['functions']:
            errs.append(f"{g['id']}: functions {g['functions']}, its hazards give {fns}")
        if goal_status(g, H) != g['status']:
            errs.append(f"{g['id']}: status '{g['status']}', its hazards give '{goal_status(g, H)}'")
    for w in d['walkthrough']:
        for p in PLACES:
            want = derived_cell(d['hazards'], w['filter'], p)
            if w.get(p, []) != want:
                errs.append(f"walk-through '{w['case']}' / {p}: {w.get(p, [])} against {want}")
    seen = {hz for w in d['walkthrough'] for p in PLACES for hz in w.get(p, [])}
    for h in d['hazards']:
        if h['phase'] == 'mowing' and h['id'] not in seen:
            errs.append(f"{h['id']}: not in the walk-through")
    class1 = {g for e in d['escalations'] if e['class'] == '1' for g in e['goals']}
    for e in d['escalations']:
        for g in e['goals']:
            if g not in G:
                errs.append(f"{e['id']}: unknown goal {g}")
        if e.get('state') not in E_STATES:
            errs.append(f"{e['id']}: state '{e.get('state')}' is not one of {E_STATES}")
        elif e['state'] != 'decided' and not e.get('cards'):
            errs.append(f"{e['id']}: open, but names no card that works on it")
        if e.get('state', '').startswith('open') and e['class'] == '1' and 'G2-FSC' not in e.get('cards', []):
            errs.append(f"{e['id']}: an open class-1 item not on card G2-FSC (D80)")
    for g in G.values():
        if g['plr'] != 'n/a' and g['id'] not in class1:
            errs.append(f"{g['id']}: PL {g['plr']}, {g['status']}, but in no class-1 escalation")
    return errs


def lc(s):
    """Lower-case the first letter of a name for use inside a sentence, unless it starts an acronym."""
    return s[0].lower() + s[1:] if s and not s[:2].isupper() else s


def cell(s):
    return str(s).replace('|', '/').replace('\n', ' ')


def table(cap, head, rows):
    out = [f'Table {cap}', '', '| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    out += ['| ' + ' | '.join(cell(c) for c in r) + ' |' for r in rows]
    return out + ['']


def render(d):
    H = d['hazards']
    mow = [h for h in H if h['phase'] == 'mowing']
    later = [h for h in H if h['phase'] != 'mowing']
    G, F, E = d['safety_goals'], d['safety_functions'], d['escalations']
    lv = Counter(h['plr'] for h in mow)
    met = Counter(h['meets'] for h in mow)
    gl = Counter(g['plr'] for g in G)
    c1 = [e for e in E if e['class'] == '1']
    dec = [e for e in E if e['class'] != '1']
    sam, ap = d['sam_file'], d['approval']
    gparts = [f"{gl[k]} at PL {k}" for k in reversed(ORDER) if gl[k]] + ([f"{gl['n/a']} with no PL"] if gl['n/a'] else [])
    goal_levels = ', '.join(gparts[:-1]) + ' and ' + gparts[-1] if len(gparts) > 1 else gparts[0]
    hl = [k for k in reversed(ORDER) if lv[k]]
    req = ', '.join(f"PL {k} for {lv[k]}" + (' hazards' if i == 0 else '') for i, k in enumerate(hl))
    n = 0

    def cap(text):
        nonlocal n
        n += 1
        return f'{n}. {text}'

    L = ['<!--',
         f"doc_no: {d['doc_no']}",
         'doc_type: HAZARD LOG',
         f"title: {d['title']}",
         'subtitle: The hazards of the mowing phase, their ratings, the safety goals and the safety functions that meet them',
         'summary: The hazard analysis and risk assessment of the Shaddad UGV for the mowing phase. Every operating mode and '
         'abnormal case of the charter is walked through in the greenhouse aisles, the palm groves, the slope test ground and '
         f"at the dock. {len(mow)} hazards are rated with the risk graph of ISO 13849-1:2023; they give {len(G)} safety goals, "
         f"which the project owner accepted as the top-level safety requirements ({ap['decision']}): {goal_levels}, with "
         "people kept out of the robot's zone in a greenhouse (D79). No goal can be shown met on the robot as built, and "
         'most need hardware it does not have: the gaps are escalated to the project owner and worked on cards G2-FSC and '
         'G1-04.',
         f"version: {d['version']}",
         f"date: {d['date']}",
         f"status: {d['status']}"]
    L += [f'history: {v} | {dt} | {txt}' for v, dt, txt in d['history']]
    L += [f"prepared_by: {d['prepared_by']}",
          f"reviewer: {d['reviewer']}",
          f"approver: {d['approver']}",
          'entity: NASL Technologies (Robotat)',
          'classification: Internal',
          'footer: Shaddad UGV Hazard Log',
          f"file: NASL_HAZ-001_Shaddad_Hazard_Log_v{d['version']}",
          'outdir: work/reports/safety',
          'chapter_breaks: no',
          'cover_image: none',
          'request: Card G1-07 (CR-002, approved by the owner on 2026-11-16: "Approve option 1"); the independent review of '
          'v1.0 (RID-G1-07-01 to -08); the owner\'s decisions of 2026-11-16 on v1.1 (D79, D80); E-07 by D103',
          'scope: The mowing phase (D76): RC driving, autonomous missions, docking and charging, the E-stop with reset and '
          're-arm, and maintenance, in the places of the charter. Tillage and spraying are later-phase entries, rated '
          'provisionally. Hazards to people; damage to the robot only where it leads to harm.',
          'related: NASL-SHD-SAF-001 (the safety plan); NASL-SHD-CHR-001 (the charter); NASL-SHD-CR-001 (the sprayer '
          'envelope); NASL-SHD-CR-002 (the plan after G0); NASL-SHD-PMP-001 (the project plan); NASL-SHD-ANR-001 (the '
          'review report); docs/safety/hazard_log.json (the record); docs/safety/hazard_log.mldatx (its Safety '
          'Analysis Manager copy, OP-01)',
          '-->',
          f"# {d['title']}",
          '',
          '## Summary',
          '',
          f"**{len(mow)} hazards in the mowing phase, {len(later)} more for later phases.** Every mode and abnormal case "
          'of the charter was walked through in each place it names (section 3). Each hazard is rated with the risk graph '
          'of ISO 13849-1:2023 and the project rules of section 2, and traced to a safety goal and to the safety '
          'functions of the twin.',
          '',
          f"- **Required levels:** {req}; {lv['n/a']} are reduced without a control function.",
          '- **Safety goals:** ' + '; '.join(f"{g['id']} {lc(g['name'])} (PL {g['plr']})" if g['plr'] != 'n/a'
                                           else f"{g['id']} {lc(g['name'])} (no PL)" for g in G) + '.',
          '- **The greenhouse rule sets the levels.** The owner keeps people out of the robot\'s zone while it mows a '
          'greenhouse aisle (D79; decision E-09: "Keep people out (Recommended)"), so no goal is above PL d. People at '
          'work beside the robot would put six goals at PL e and would need safety-rated person detection.',
          f"- **Approved.** The project owner decided on {ap['date']}, card {ap['card']}: \"{ap['words']}\" "
          f"({ap['decision']}): {ap['what']}.",
          f"- **Met today:** no goal can be shown met on the robot as built. Of the {len(mow)} hazards, the twin partly "
          f"meets {met['partly']} and does not meet {met['no']}; for {met['unknown']} the evidence is unknown; "
          f"{met['by other measures']} rest on design measures and procedures still to be shown (the classes are defined "
          'in section 2).',
          '- **Why:** the as-built record has no E-stop chain or STO yet, estimated brakes and no perception sensors; the '
          'owner confirms E-stop buttons and a safety relay with STO wiring on the robot (D80), whose data come from the '
          'supplier on card G1-04. In the twin every safety function except the hardware E-stop chain (with the STO and '
          'the brakes it acts on) is a single-channel software function in one microcontroller or in the companion, '
          'which reaches PL b at most.',
          f"- **Escalated** to the project owner (section 7): {len(c1)} class-1 items, each a safety goal the as-built "
          'hardware cannot meet, now worked on cards G2-FSC and G1-04 (D80); and '
          f"{len(dec)} plain decisions: the greenhouse operating concept (E-09), decided (D79), and the editions and the "
          'jurisdiction (E-07), decided (D103).',
          '',
          '## 1. Purpose and scope',
          '',
          'This log records the hazards of the Shaddad UGV, the risk of each, the safety goal it gives and the safety '
          'function that meets that goal today, or the gap. It is the record of card G1-07 under the safety plan '
          'NASL-SHD-SAF-001, and it feeds the requirements baseline (G1-01), the SRR (G1-05) and the functional safety '
          'concept (G2-FSC).',
          '',
          f"- **The system:** {d['basis']['system']}",
          f"- **The operating concept:** {d['basis']['operating_concept']}",
          '- **The people exposed:** ' + '; '.join(f'{k}, who {v}' for k, v in d['people'].items()) + '.',
          f"- **The safe state:** {d['safe_state']}",
          '- **The record:** `docs/safety/hazard_log.json` holds the log; this document is generated from it by '
          '`docs/safety/hazard_log_render.py`, whose `--check` verifies the levels, the traces, the classes and the '
          f"walk-through. Hazards are never renumbered. {sam['note']}",
          '',
          '## 2. The method',
          '',
          '**Identification.** The hazards come from a walk-through of every operating mode and abnormal case of the '
          'charter (section 6) in each place it names, with the hazard list of ISO 12100:2010 (Annex B) and the hazards '
          'of automated agricultural machines of ISO 18497-1:2024: the moving machine and its implement, instability on '
          'slopes, unexpected start-up, failures of the control system and of perception, energy (the pack, the '
          'charger), and foreseeable misuse. The sources were the charter, the as-built record, the specs, the decisions '
          'D30 to D80, the open problems of the status record and the review report NASL-SHD-ANR-001 (2026-11-15).',
          '',
          '**Rating.** Each hazardous event is rated with the three parameters of ISO 13849-1:2023 Annex A, without the '
          'safety function meant to reduce it. The safety plan (section 4) explains the choice of this method over the '
          'ISO 25119 risk graph.',
          '',
          f"**The exposure F.** {d['method']['F_definition']}",
          '']
    L += table(cap('The risk parameters and how this project applies them.'), ['Parameter', 'Class', 'Applied when'],
               [['Severity', k, v] for k, v in d['method']['S'].items()] +
               [['Frequency and exposure', k, v] for k, v in d['method']['F'].items()] +
               [['Possibility of avoiding', k, v] for k, v in d['method']['P'].items()])
    g = d['method']['graph']
    L += table(cap('The risk graph of ISO 13849-1 Annex A: the required performance level.'),
               ['Severity', 'Frequency', 'P1', 'P2'],
               [[s, f, g[f'{s} {f} P1'], g[f'{s} {f} P2']] for s in ('S1', 'S2') for f in ('F1', 'F2')])
    L += ['**Rules of this issue:**', ''] + [f'{i}. {r}' for i, r in enumerate(d['method']['rules'], 1)] + ['']
    na = len(d['method']['assumptions'])
    L += [f"**The operating concept behind the exposure classes:** the owner's greenhouse rule (1) and the assumptions "
          f"of this log (2 to {na}).", ''] + [f'{i}. {a}' for i, a in enumerate(d['method']['assumptions'], 1)] + ['']
    L += ['**What "met today" and "status today" mean.** Each hazard gets one class for what meets it today, judged on '
          'the twin and the as-built record; each goal\'s status follows from its hazards, and `--check` verifies it.', '']
    L += table(cap('The classes of "met today" (hazards) and "status today" (goals).'), ['For', 'Class', 'Means'],
               [['A hazard', k, v] for k, v in d['met_classes'].items()] +
               [['A goal', k, v] for k, v in d['status_classes'].items()])

    L += ['## 3. The walk-through', '',
          'Every row is a mode or a case of the charter (section 6: the normal use, the abnormal cases, the modes) or of '
          'the maintainer\'s work (section 5); every column is a place the charter names. A cell lists the hazards whose '
          'cases, modes and places put them there; a cell that reads "none" means the case does not arise in that place. '
          'The cells are derived from the hazards, and `--check` verifies them.', '']
    L += table(cap('The walk-through: the cases of the charter in each place.'), ['Case'] + [p.capitalize() for p in PLACES],
               [[w['case']] + [', '.join(w.get(p, [])) or 'none' for p in PLACES] for w in d['walkthrough']])

    L += ['## 4. The hazards of the mowing phase', '']
    L += table(cap('The hazards at a glance.'), ['ID', 'Hazard', 'S F P', 'PLr', 'Goal', 'Met today'],
               [[h['id'], h['title'], f"{h['S']} {h['F']} {h['P']}", h['plr'], ', '.join(h['goals']), h['meets']]
                for h in mow])
    groups = []
    for h in mow:
        if h['group'] not in groups:
            groups.append(h['group'])
    for k, grp in enumerate(groups, 1):
        L += [f'### 4.{k} {grp}', '']
        for h in (x for x in mow if x['group'] == grp):
            fn = '; '.join(f"{f} {lc(next(x['name'] for x in F if x['id'] == f))}" for f in h['functions']) or 'none'
            L += [f"#### {h['id']} {h['title']}", '',
                  f"- **Mode and place:** {', '.join(h['modes'])}; {', '.join(h['places'])}. **Cases:** {', '.join(h['cases'])}.",
                  f"- **Cause:** {h['cause']}",
                  f"- **Hazardous event:** {h['event']}",
                  f"- **Exposed:** {', '.join(h['exposed'])}. **Harm:** {h['harm']}.",
                  f"- **Rating:** {h['S']} {h['F']} {h['P']}, **PLr {h['plr']}**. {h['why']}",
                  f"- **Safety goal:** {', '.join(h['goals'])}. **Safety function today:** {fn}. "
                  f"**Other measures:** {h['other_measures']}.",
                  f"- **Met today:** {h['meets']}." + (f" **Gap:** {h['gap']}" if h['gap'] else ''),
                  ]
            if h['refs']:
                L += [f"- **References:** {'; '.join(h['refs'])}."]
            L += ['']

    L += ['## 5. The safety goals', '',
          'The safety goals are the top-level safety requirements of this phase. Card G1-01 takes them into the '
          'requirements baseline; the functional safety concept (G2-FSC) gives each one its safety functions, safe '
          'state, reaction time, architecture and verification. Each goal takes the highest level of its hazards and '
          'lists the safety functions of its hazards.', '',
          f"**Accepted.** The project owner accepted these goals on {ap['date']}, card {ap['card']}: \"{ap['words']}\" "
          f"({ap['decision']}), as the top-level safety requirements at the levels the greenhouse rule (D79) sets.", '']
    L += table(cap('The safety goals.'), ['ID', 'Goal', 'PLr', 'Hazards', 'Status today'],
               [[x['id'], x['goal'], x['plr'], ', '.join(x['hazards']), x['status']] for x in G])
    for x in G:
        L += [f"#### {x['id']} {x['name']}", '',
              f"- **Requirement:** {x['goal']}",
              f"- **Safe state:** {x['safe_state']}. **PLr:** {x['plr']}." + (f" {x['plr_note']}" if x['plr_note'] else ''),
              f"- **Hazards:** {', '.join(x['hazards'])}. **Safety functions today:** "
              f"{', '.join(x['functions']) or 'none (non-functional measures)'}.",
              f"- **Status:** {x['status']}. {x['gap']}", '']

    L += ['## 6. The safety functions today', '',
          'The safety functions of the twin, as the specs and the decisions describe them, with what the as-built record '
          'shows of them and a preliminary judgement of the level their architecture can reach. The concept (G2-FSC) '
          'and the FMEA (G3-FME) settle the levels; this log does not design them.', '']
    L += table(cap('The safety functions, where they run and what their architecture can reach.'),
               ['ID', 'Function', 'Where', 'In the as-built record', 'Architecture (preliminary)'],
               [[x['id'], x['name'], x['where'], x['robot'], x['architecture']] for x in F])
    for x in F:
        L += [f"#### {x['id']} {x['name']}", '', f"- **Does:** {x['does']}", f"- **Evidence in the twin:** {x['evidence']}.", '']

    L += ['## 7. Escalations, findings and open points', '',
          '**Escalated to the project owner, through the lead systems engineer.** Each class-1 item is a safety goal the '
          'as-built hardware cannot meet; closing it needs new hardware or supplier data, which is the owner\'s decision. '
          'E-07 and E-09 are plain decisions, not class-1 changes. The functional safety engineer does not design the '
          'fixes.', '',
          '**The owner\'s decisions of 2026-11-16 (D79, D80).** E-09 is decided and the question of E-01 is answered. '
          'The class-1 items are worked on card G2-FSC, whose exit asks for each one the architecture that meets its '
          'goal at the level D79 and D80 set, or a change request for new hardware; the data of the safety devices and '
          'the supplier\'s data come on card G1-04. Nothing is bought now (D80).', '',
          '**The owner\'s decision of 2026-11-18 (D103).** E-07 is decided: the Kingdom of Saudi Arabia, with every '
          'standard dated in the safety plan\'s Table 1. No hazard is rated again.', '']
    L += table(cap('The escalations.'), ['ID', 'Class', 'Goals', 'State', 'Worked on', 'What and why'],
               [[e['id'], e['class'], ', '.join(e['goals']) or 'none',
                 e['state'] + (f" ({e['decision']})" if e['decision'] else ''), ', '.join(e['cards']) or 'none', e['text']]
                for e in E])
    L += [f"**Findings for the concept (G2-FSC) and the subsystem engineers** (severity: {d['finding_scale']})", '']
    L += [f"- **{f['id']} ({f['severity']}):** {f['text']}" for f in d['findings']] + ['']
    L += ['**Open points of this issue:**', '',
          f"- {sam['note']}",
          '- **The operator\'s and the maintainer\'s view** (the risk on card G1-07): the owner, today the only operator '
          'and maintainer of the robot, is asked to check the walk-through (Table 4) for a missing case, and to confirm '
          'assumptions 2 and 5 of section 2 (the palm-grove rows, the charging place), by G1-05.',
          '- Each hazard is rated again when the concept, the supplier data or a change request changes its cause or its '
          'measures.', '']

    L += ['## 8. Later-phase entries: tillage and spraying', '',
          'Rated provisionally, from the estimated tiller data and CR-001. Cards G2-TIL and G2-SPR rate them again before '
          'those phases start.', '']
    L += table(cap('The later-phase entries.'), ['ID', 'Hazard', 'S F P', 'PLr', 'Goal', 'Measure today'],
               [[h['id'], h['title'], f"{h['S']} {h['F']} {h['P']}", h['plr'], ', '.join(h['goals']), h['other_measures']]
                for h in later])
    L += ['## References', '',
          '- NASL-SHD-SAF-001, the safety plan; NASL-SHD-CHR-001 v1.2, the project charter; NASL-SHD-CR-001 v1.2 and '
          'NASL-SHD-CR-002 v1.1, the change requests; NASL-SHD-PMP-001, the project plan.',
          '- ISO 12100:2010, risk assessment and risk reduction; ISO 13849-1:2023 and ISO 13849-2:2012, safety-related '
          'parts of control systems; ISO 18497-1 to -4:2024, safety of partially automated, semi-autonomous and '
          'autonomous agricultural machinery; ISO 25119-1:2018 with Amd 1:2020, -2:2019, -3:2018 with Amd 1:2020 and '
          '-4:2018 with Amd 1:2020, safety-related parts of control systems of agricultural machinery; ISO 13850:2015, '
          'the emergency stop function; IEC 60204-1:2016 with AMD1:2021, electrical equipment of machines. The '
          'jurisdiction is the Kingdom of Saudi Arabia (D103). The full dated list, with the binding Saudi layer (the SASO '
          'machinery regulation TR 01-08-21-180 and the standards it adopts, the CST radio specifications and the Labor '
          'Law), is in NASL-SHD-SAF-001 v1.4, Table 1; the mower standard is chosen on card G2-FSC.']
    L += [f'- {s}' for s in d['basis']['sources']]
    return '\n'.join(L) + '\n'


def main():
    d = json.load(open(SRC, encoding='utf-8'))
    errs = check(d)
    for e in errs:
        print('CHECK:', e)
    mow = [h for h in d['hazards'] if h['phase'] == 'mowing']
    print(f"hazards {len(d['hazards'])} (mowing {len(mow)}), goals {len(d['safety_goals'])}, "
          f"functions {len(d['safety_functions'])}, walk-through cells {len(d['walkthrough']) * len(PLACES)}, "
          f"escalations {len(d['escalations'])}, errors {len(errs)}")
    if '--check' in sys.argv or errs:
        sys.exit(1 if errs else 0)
    with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
        f.write(render(d))
    print('written', OUT)


if __name__ == '__main__':
    main()
