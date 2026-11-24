"""make_f05_split.py - two-panel detail version of the RC_Link_Sim print (fix round 2).

F05_rc_link_sim.png (print -s RC_Link_Sim, 200 dpi, 3053 x 561 px) is a full-width strip whose block names
print at about 4 pt in the 15.9 cm column. This splits it into
  (a) inputs op / flt, rate transitions RT_op_* / RT_flt_*, RC_Transmitter, RC_Radio_Channel, RC_Receiver
  (b) RC_Receiver output, RT_rc_frame, SBUS_Encoder, UART_Line_Sim and the outputs rc_frame / sbus
at one common scale (panel (a) spans the column: 10 pt Simulink names print at about 6.1 pt).
Output: figs/F05_rc_link_sim_split.png
"""
import os
from PIL import Image, ImageDraw, ImageFont

FIG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'figs'))
NAVY = (43, 47, 126)
src = Image.open(os.path.join(FIG, 'F05_rc_link_sim.png')).convert('RGB')
W, H = src.size
crops = [(0, 0, 2062, H), (2050, 80, W, H)]   # RC_Receiver box spans x = 1657..2048 px; (b) has no blocks above y = 80
COL_PT = 451.0
pxpt = (crops[0][2] - crops[0][0]) / COL_PT
f = ImageFont.truetype(r'C:\Windows\Fonts\calibrib.ttf', int(round(9 * pxpt)))
panels = []
for i, c in enumerate(crops):
    p = src.crop(c)
    pad = int(round(14 * pxpt))   # room for the panel label above the diagram
    q = Image.new('RGB', (p.size[0], p.size[1] + pad), 'white')
    q.paste(p, (0, pad))
    d = ImageDraw.Draw(q)
    d.rectangle((0, 0, q.size[0] - 1, q.size[1] - 1), outline=(150, 150, 160), width=3)
    d.text((int(3 * pxpt), int(2 * pxpt)), '(%s)' % 'ab'[i], fill=NAVY, font=f)
    panels.append(q)
gap = int(5 * pxpt)
Wt = max(p.size[0] for p in panels)
Ht = sum(p.size[1] for p in panels) + gap
out = Image.new('RGB', (Wt, Ht), 'white')
out.paste(panels[0], (0, 0))
out.paste(panels[1], (0, panels[0].size[1] + gap))
fn = os.path.join(FIG, 'F05_rc_link_sim_split.png')
out.save(fn, dpi=(200, 200), optimize=True)
print(fn, out.size, '10 pt names print at %.1f pt' % (10 * 200 / 72 / pxpt))
