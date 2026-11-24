"""pdf_fullres_images.py - restore full-resolution figures in the Word-exported PDF.

Word's PDF export down-samples pictures to about 200 ppi. The dense Simulink diagrams
(e.g. 6382 px wide) then lose their labels when the reader zooms in. This step finds every
figure image in the PDF (in document order, matched by aspect ratio against the build's
figure order) and replaces its pixel data with the original file, capped at MAX_PPI at the
printed size. Page layout is not changed.

usage: python -I pdf_fullres_images.py <pdf> <figure_order.json> <fig_dir>
"""
import io
import json
import os
import sys
import zlib

import pymupdf
from PIL import Image

MAX_PPI = 500          # cap, keeps the PDF size reasonable
BG_SIZE = (1241, 1754)  # NASL page background (not a figure)


def main():
    pdf_path, order_path, fig_dir = sys.argv[1:4]
    order = json.load(open(order_path, encoding='utf8'))
    sizes = {}
    for f in set(order):
        with Image.open(os.path.join(fig_dir, f)) as im:
            sizes[f] = im.size
    doc = pymupdf.open(pdf_path)
    # collect figure placements in reading order
    placements = []
    seen_xref = set()
    for pno, page in enumerate(doc):
        infos = page.get_image_info(xrefs=True)
        infos = [i for i in infos if (i['width'], i['height']) != BG_SIZE and i['xref']]
        infos.sort(key=lambda i: (round(i['bbox'][1]), i['bbox'][0]))
        for i in infos:
            placements.append((pno, i))
    k = 0
    mapping = {}  # xref -> (file, display width in inch)
    for pno, i in placements:
        ar = i['width'] / i['height']
        match = None
        for j in range(k, min(k + 3, len(order))):
            w, h = sizes[order[j]]
            if abs((w / h) / ar - 1) < 0.02:
                match = j
                break
        if match is None:
            print(f'  page {pno + 1}: image {i["width"]}x{i["height"]} not matched, left as is')
            continue
        k = match + 1
        disp_in = (i['bbox'][2] - i['bbox'][0]) / 72.0
        if i['xref'] not in mapping:
            mapping[i['xref']] = (order[match], disp_in, pno)
    if k != len(order):
        print(f'  warning: matched {k} of {len(order)} figures')
    replaced = 0
    for xref, (f, disp_in, pno) in mapping.items():
        src = os.path.join(fig_dir, f)
        w, h = sizes[f]
        cur_w = doc.xref_get_key(xref, 'Width')[1]
        cur_w = int(cur_w) if cur_w.isdigit() else 0
        target_w = min(w, int(MAX_PPI * disp_in))
        if target_w <= cur_w * 1.1:
            continue
        with Image.open(src) as im:
            im = im.convert('RGB')
            if target_w < w:
                im = im.resize((target_w, round(h * target_w / w)), Image.LANCZOS)
            nw, nh = im.size
            if f.lower().endswith(('.jpg', '.jpeg')):
                buf = io.BytesIO(); im.save(buf, 'JPEG', quality=90)
                data, filt = buf.getvalue(), '/DCTDecode'
            else:
                data, filt = zlib.compress(im.tobytes(), 9), '/FlateDecode'
        # overwrite the existing image object in place (no duplicate objects, same placement)
        doc.update_stream(xref, data, compress=False)
        for key in ('SMask', 'Mask', 'Decode', 'DecodeParms', 'Interpolate'):
            doc.xref_set_key(xref, key, 'null')
        doc.xref_set_key(xref, 'Filter', filt)
        doc.xref_set_key(xref, 'Width', str(nw))
        doc.xref_set_key(xref, 'Height', str(nh))
        doc.xref_set_key(xref, 'ColorSpace', '/DeviceRGB')
        doc.xref_set_key(xref, 'BitsPerComponent', '8')
        replaced += 1
    tmp = pdf_path + '.tmp'
    doc.save(tmp, garbage=3, deflate=True)
    doc.close()
    os.replace(tmp, pdf_path)
    print(f'full-resolution figures restored: {replaced} image(s); PDF {os.path.getsize(pdf_path) / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
