"""Add the figures generated in fix round 1 to figs/manifest.json (fallback captions and provenance)."""
import glob, json, os
from PIL import Image
R = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
FIG = os.path.join(R, 'figs')
man = json.load(open(os.path.join(FIG, 'manifest.json'), encoding='utf8'))
have = {m['file'] for m in man}
caps = {}
for f in sorted(glob.glob(os.path.join(R, 'chapters', '*.json'))):
    for b in json.load(open(f, encoding='utf8'))['blocks']:
        if b.get('type') == 'figure':
            caps.setdefault(b['file'], b.get('caption', ''))
SRC = {
    '_overview.png': 'overview of {orig} (print -s, 200 dpi), lightened, with numbered callouts placed from get_param(...,\'Position\'); build/figtools/make_callout_figs.py',
    '_detail.png': 'native-resolution crop(s) of {orig} (print -s, 200 dpi) at a scale where 10 pt block names print at 6 pt in the 15.9 cm column, with the same callouts; build/figtools/make_callout_figs.py',
}
ORIG = {'F09_power_plant': 'F09_power_plant_arranged.png (print -s of an in-memory copy of Power_Plant after Simulink.BlockDiagram.arrangeSystem, re-printed 2026-10-23; project file unchanged)'}
for f in sorted(caps):
    if f in have:
        continue
    p = os.path.join(FIG, f)
    if not os.path.exists(p):
        continue
    w, h = Image.open(p).size
    src = ''
    for suf, txt in SRC.items():
        if f.endswith(suf):
            base = f[:-len(suf)]
            src = txt.format(orig=ORIG.get(base, base + '.png'))
    if f == 'F16_matlab_desktop_crop.png':
        src = 'F16_matlab_desktop.png with the title bar (licence text), the Copilot side panel and the status bar cropped; build/figtools/make_screens.py'
    if f == 'F17_simulink_editor_callouts.png':
        src = 'F17_simulink_editor.png with the title bar cropped and Table 15 callouts added; build/figtools/make_screens.py'
    man.append({'file': f, 'caption': caps[f], 'source': src, 'ok': True, 'width_px': w, 'height_px': h})
    print('added', f)
for m in man:
    if m['file'] == 'F22_photo_mower.jpg':
        m['note'] = 'not used from report v1.0 fix round 1: image origin and usage rights not confirmed'
    if m['file'] in ('F16_matlab_desktop.png', 'F17_simulink_editor.png'):
        m['note'] = 'original capture shows the licence type in the title bar; the report uses the cropped version'
p = os.path.join(FIG, 'F09_power_plant_arranged.png')
if 'F09_power_plant_arranged.png' not in {m['file'] for m in man} and os.path.exists(p):
    w, h = Image.open(p).size
    man.append({'file': 'F09_power_plant_arranged.png', 'caption': 'Power_Plant after arrangeSystem (source print of the overview and detail figures).', 'source': ORIG['F09_power_plant'] + '; build/figtools/print_power_plant.m', 'ok': True, 'width_px': w, 'height_px': h})
json.dump(man, open(os.path.join(FIG, 'manifest.json'), 'w', encoding='utf8'), indent=2, ensure_ascii=False)
print(len(man), 'entries')
