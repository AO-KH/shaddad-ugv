"""Collage of the real robot components (figure M16) for the model-based design report.

Tiles a-c are photos (prep_real_photos.py), tiles d-f are the 3-D renders of the electronics
(real_electronics_3d.m, label-free versions), each with a label strip.
Run: python -I make_real_collage.py <project root>
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont, ImageOps

root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
figs = os.path.join(root, 'docs', 'report', 'figs')
tiles = [
    ('M02_motor_nameplate.jpg', 'Traction motor: Movematic BLDC, 48 V, 750 W, 3000 rpm'),
    ('M14_battery_spec_img4.jpg', 'Battery: 13S15P Molicel P42A, 46.8 V, 63 Ah (XMachines box)'),
    ('M03_sprayer_pump.jpg', 'Sprayer module: brass piston pump, 12 x 8 mm hose'),
    ('M55_electronics_box_3d_plain.png', 'Electronics box (3-D render after the photo)'),
    ('M50_xt_ps_board_3d_plain.png', 'XT_PS V1.0 control board with the STM32 Nucleo-32 (3-D render)'),
    ('M51_stm32_ftdi_3d_plain.png', 'STM32 USB-C core board and FT232RL USB-UART (3-D render)'),
]
TW, TH, LH, GAP = 600, 450, 64, 14
cols, rows = 3, 2
W = cols*TW + (cols + 1)*GAP
H = rows*(TH + LH) + (rows + 1)*GAP
canvas = Image.new('RGB', (W, H), 'white')
draw = ImageDraw.Draw(canvas)
try:
    font = ImageFont.truetype('calibri.ttf', 24)
except OSError:
    font = ImageFont.load_default()
navy = (43, 47, 126)
for k, (f, label) in enumerate(tiles):
    r, c = divmod(k, cols)
    x0 = GAP + c*(TW + GAP)
    y0 = GAP + r*(TH + LH + GAP)
    im = Image.open(os.path.join(figs, f)).convert('RGB')
    im = ImageOps.fit(im, (TW, TH), Image.LANCZOS, centering=(0.5, 0.5))
    canvas.paste(im, (x0, y0))
    draw.rectangle([x0, y0 + TH, x0 + TW - 1, y0 + TH + LH - 1], fill=(238, 234, 246))
    # wrap the label into at most two lines
    words, lines, cur = label.split(), [], ''
    for w in words:
        t = (cur + ' ' + w).strip()
        if draw.textlength(t, font=font) > TW - 20:
            lines.append(cur)
            cur = w
        else:
            cur = t
    lines.append(cur)
    for j, ln in enumerate(lines[:2]):
        draw.text((x0 + 10, y0 + TH + 6 + 27*j), ln, fill=navy, font=font)
    draw.text((x0 + 8, y0 + 6), '(%s)' % 'abcdef'[k], fill='white', font=font, stroke_width=3, stroke_fill='black')
canvas.save(os.path.join(figs, 'M16_real_components_collage.jpg'), quality=90, optimize=True)
print('M16_real_components_collage.jpg', canvas.size)
