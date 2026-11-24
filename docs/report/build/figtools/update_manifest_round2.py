"""Fix round 2: manifest entries for the re-printed / regenerated figures."""
import json, os
from PIL import Image
FIG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'figs'))
CH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'chapters'))
mp = os.path.join(FIG, 'manifest.json')
man = json.load(open(mp, encoding='utf8'))
caps = {}
for f in os.listdir(CH):
    for b in json.load(open(os.path.join(CH, f), encoding='utf8'))['blocks']:
        if b.get('type') == 'figure':
            caps.setdefault(b['file'], b.get('caption', ''))
src = {
    'F10_battery_pack.png': 'print -s Pack_Grouped16 (subsystems/power), 200 dpi, re-printed 2026-10-23 with the Probe block widened in memory (model closed without saving); build/figtools/print_f10_battery_pack.m',
    'F05_rc_link_sim_split.png': 'two crops of F05_rc_link_sim.png (print -s RC_Link_Sim, 200 dpi) at one scale, 10 pt block names print at about 6 pt; build/figtools/make_f05_split.py',
    'F33_project_structure.png': 'drawn in MATLAB from the project folder listing (2026-10-23; regenerated in fix round 2: 39 build_*.m + helper shaddad_bu.m); build/figtools/diag_f33.m',
}
by = {m['file']: m for m in man}
for f, s in src.items():
    w, h = Image.open(os.path.join(FIG, f)).size
    m = by.get(f)
    if m is None:
        m = {'file': f, 'caption': caps.get(f, ''), 'source': s, 'ok': True, 'width_px': w, 'height_px': h}
        man.append(m)
    m.update({'source': s, 'width_px': w, 'height_px': h})
    if f in caps:
        m['caption'] = caps[f]
for m in man:
    p = os.path.join(FIG, m['file'])
    if m['file'].endswith('_overview.png') and os.path.exists(p):
        m['width_px'], m['height_px'] = Image.open(p).size
        big = m['file'] in ('F03_mcu_top_overview.png', 'F04_mcu_task10_overview.png')
        m['note'] = 'regenerated in fix round 2: overview height %s, fade 0.35' % ('16 cm (page of its own)' if big else '11 cm')
    if m['file'] == 'F05_rc_link_sim.png':
        m['note'] = 'source of F05_rc_link_sim_split.png; the report uses the split version (fix round 2)'
json.dump(man, open(mp, 'w', encoding='utf8'), indent=2, ensure_ascii=False)
print(len(man), 'entries')
