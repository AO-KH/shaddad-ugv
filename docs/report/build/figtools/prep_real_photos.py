"""Prepare the real-robot photos and drawings for the model-based design report (figures M01..M16).

Sources: docs/hardware/source/ (photos and PDFs received 2026-10-27). Output: docs/report/figs/M*.jpg / .png
(EXIF orientation applied, longest side <= 1600 px, JPEG quality 88). Run: python -I prep_real_photos.py <project root>
"""
import io
import os
import sys

import pymupdf
from PIL import Image, ImageOps

root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
src = os.path.join(root, 'docs', 'hardware', 'source')
out = os.path.join(root, 'docs', 'report', 'figs')


def save_photo(name_in, name_out, crop=None, max_side=1600):
    im = Image.open(os.path.join(src, name_in))
    im = ImageOps.exif_transpose(im).convert('RGB')
    if crop:
        w, h = im.size
        im = im.crop((int(crop[0] * w), int(crop[1] * h), int(crop[2] * w), int(crop[3] * h)))
    s = max_side / max(im.size)
    if s < 1:
        im = im.resize((round(im.size[0] * s), round(im.size[1] * s)), Image.LANCZOS)
    p = os.path.join(out, name_out)
    im.save(p, quality=88, optimize=True)
    print(name_out, im.size)


# photos (crop boxes as fractions of the image: left, top, right, bottom)
save_photo('spec_sheet_dimensions_weight.jpg', 'M01_spec_sheet.png')
save_photo('drive_motor_nameplate_movematic.jpg', 'M02_motor_nameplate.jpg')
save_photo('sprayer_pump.jpg', 'M03_sprayer_pump.jpg')
save_photo('electronics_box_xt_ps_nucleo.jpeg', 'M04_electronics_box.jpg')
save_photo('electronics_ftdi_stm32_board.jpeg', 'M05_stm32_ftdi.jpg')
save_photo('electronics_ads1115_adc.jpg', 'M06_ads1115.jpg')
save_photo('electronics_lm2596_buck.jpg', 'M07_lm2596.jpg', crop=(0.22, 0.38, 0.85, 0.72))
save_photo('electronics_xmachines_spr_v1_3_board.jpg', 'M08_spr_board.jpg')
save_photo('electronics_ftdi_closeup.jpg', 'M09_ftdi_closeup.jpg')

# battery box drawing XMT-P1-E01-000 (A3 landscape vector PDF) at 200 dpi
d = pymupdf.open(os.path.join(src, 'battery_box_drawing_XMT-P1-E01-000.pdf'))
pix = d[0].get_pixmap(dpi=200)
pix.save(os.path.join(out, 'M10_battery_box_drawing.png'))
print('M10_battery_box_drawing.png', pix.width, pix.height)

# embedded photos of the battery spec sheet (page 2: cell pack, existing box, SB120 connector)
d = pymupdf.open(os.path.join(src, 'battery_spec_sheet.pdf'))
k = 0
for pno, page in enumerate(d):
    for img in page.get_images(full=True):
        x = d.extract_image(img[0])
        im = Image.open(io.BytesIO(x['image'])).convert('RGB')
        k += 1
        print('spec sheet page', pno + 1, 'image', k, im.size)
        im.save(os.path.join(out, 'M1%d_battery_spec_img%d.jpg' % (k, k)), quality=88)
# page renders of the spec sheet (text pages, for the appendix)
for pno in range(len(d)):
    pix = d[pno].get_pixmap(dpi=110)
    pix.save(os.path.join(out, 'M15_battery_spec_p%d.png' % (pno + 1)))
    print('M15_battery_spec_p%d.png' % (pno + 1), pix.width, pix.height)
