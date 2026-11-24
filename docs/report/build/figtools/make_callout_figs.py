"""make_callout_figs.py - readable versions of the dense Simulink prints.

For each printed diagram (print -s, 200 dpi) this writes
  <base>_overview.png : the full diagram, lightened, with numbered callouts (and short labels)
                        on the blocks the report text and tables discuss;
  <base>_detail.png   : one or two native-resolution crops of the region the text discusses,
                        scaled so that 10 pt block names print at >= 6 pt in a 451 pt column,
                        with the same callout numbers.
Block boxes come from get_param(...,'Position') (positions.json) mapped to image pixels with the
offset fitted in fit_offsets.m (offsets.json); scale 200/72 px per Simulink unit.
"""
import json
import os
from PIL import Image, ImageDraw, ImageFont

WD = os.path.dirname(os.path.abspath(__file__))
FIG = r'C:\Users\aomar\Desktop\Robot project\docs\report\figs'
NAVY = (43, 47, 126)
MAGENTA = (166, 75, 156)
PURPLE = (110, 79, 163)
WHITE = (255, 255, 255)
COL_PT = 451.0          # text column width in pt
MAX_H_PT = 510.0        # 18 cm
OVERVIEW_MAX_H_PT = 312.0   # 11 cm: overviews are navigation aids (fix round 2); build_report.js caps them at 11 cm too
FB = r'C:\Windows\Fonts\calibrib.ttf'

S = json.load(open(os.path.join(WD, 'positions.json')))
O = json.load(open(os.path.join(WD, 'offsets.json')))


def font(px):
    return ImageFont.truetype(FB, max(8, int(round(px))))


def block_boxes(key):
    v = S[key]
    o = O[key]
    s = o['s']
    out = {}
    for b in (v['blocks'] if isinstance(v['blocks'], list) else [v['blocks']]):
        l, t, r, bt = b['pos']
        out[b['name']] = (o['ox'] + l * s, o['oy'] + t * s, o['ox'] + r * s, o['oy'] + bt * s)
    return out


def union(boxes):
    return (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))


def draw_callouts(img, callouts, boxes, scale, offset, pxpt, labels=True, clip=None, force_anchor=None):
    """scale: original->image factor; offset: (x0, y0) of the crop in original px."""
    d = ImageDraw.Draw(img)
    W, H = img.size
    r = 4.6 * pxpt
    fnum = font(6.6 * pxpt)
    flab = font(6.6 * pxpt)
    pad = 1.5 * pxpt
    lw = max(2, int(round(0.8 * pxpt)))
    placed = []
    for c in callouts:
        groups = c['boxes']
        first = True
        for g in groups:
            if any(n not in boxes for n in g):
                missing = [n for n in g if n not in boxes]
                raise KeyError('%s: missing blocks %s' % (c.get('label'), missing))
            ob = union([boxes[n] for n in g])
            if clip is not None:
                # skip boxes outside the crop
                if ob[2] < clip[0] or ob[0] > clip[2] or ob[3] < clip[1] or ob[1] > clip[3]:
                    continue
            bx = ((ob[0] - offset[0]) * scale - pad, (ob[1] - offset[1]) * scale - pad,
                  (ob[2] - offset[0]) * scale + pad, (ob[3] - offset[1]) * scale + pad)
            d.rounded_rectangle(bx, radius=2.5 * pxpt, outline=MAGENTA, width=lw)
            anc = force_anchor or c.get('anchor', 'tl')
            cx = bx[0] if 'l' in anc else bx[2]
            cy = bx[1] if 't' in anc else bx[3]
            if anc == 'r':
                cx, cy = bx[2], (bx[1] + bx[3]) / 2
            if anc == 'l':
                cx, cy = bx[0], (bx[1] + bx[3]) / 2
            cx = min(max(cx, r + 2), W - r - 2)
            cy = min(max(cy, r + 2), H - r - 2)
            if c.get('num') is not None:
                d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=NAVY, outline=WHITE, width=max(2, lw // 2))
                t = str(c['num'])
                tb = d.textbbox((0, 0), t, font=fnum)
                d.text((cx - (tb[2] + tb[0]) / 2, cy - (tb[3] + tb[1]) / 2), t, fill=WHITE, font=fnum)
            if labels and first and c.get('label'):
                t = c['label']
                tb = d.textbbox((0, 0), t, font=flab)
                tw, th = tb[2] - tb[0], tb[3] - tb[1]
                side = c.get('side', 'r')
                gap = r + 1.5 * pxpt
                if side == 'r':
                    lx, ly = cx + gap, cy - th / 2
                elif side == 'l':
                    lx, ly = cx - gap - tw, cy - th / 2
                elif side == 'a':
                    lx, ly = cx - tw / 2, cy - r - 2.5 * pxpt - th
                else:
                    lx, ly = cx - tw / 2, cy + r + 2.5 * pxpt
                lx += c.get('dx', 0) * pxpt
                ly += c.get('dy', 0) * pxpt
                lx = min(max(lx, 3 * pxpt), W - tw - 3 * pxpt)
                ly = min(max(ly, 3 * pxpt), H - th - 3 * pxpt)
                m = 1.1 * pxpt
                d.rounded_rectangle((lx - m, ly - m, lx + tw + m, ly + th + m), radius=1.5 * pxpt,
                                    fill=WHITE, outline=NAVY, width=max(1, lw // 2))
                d.text((lx - tb[0], ly - tb[1]), t, fill=NAVY, font=flab)
                placed.append((lx, ly, tw, th))
            first = False


def load_rgb(path):
    im = Image.open(path)
    if im.mode in ('RGBA', 'LA', 'P'):
        im = im.convert('RGBA')
        bg = Image.new('RGBA', im.size, WHITE + (255,))
        bg.alpha_composite(im)
        im = bg
    return im.convert('RGB')


def overview(spec):
    key = spec['key']
    src = load_rgb(os.path.join(FIG, S[key]['png']))
    W, H = src.size
    disp_w = min(COL_PT, spec.get('max_h_pt', OVERVIEW_MAX_H_PT) * W / H)
    pxpt = 5.0
    Wout = int(round(disp_w * pxpt))
    scale = Wout / W
    im = src.resize((Wout, int(round(H * scale))), Image.LANCZOS)
    im = Image.blend(im, Image.new('RGB', im.size, WHITE), spec.get('fade', 0.35))
    draw_callouts(im, spec['callouts'], block_boxes(key), scale, (0, 0), pxpt, labels=spec.get('labels', True))
    out = os.path.join(FIG, spec['base'] + '_overview.png')
    im.save(out, dpi=(200, 200), optimize=True)
    return out, im.size


def detail(spec):
    key = spec['key']
    src = load_rgb(os.path.join(FIG, S[key]['png']))
    panels = []
    maxw = max(p[2] - p[0] for p in spec['crops'])
    pxpt = maxw / COL_PT
    boxes = block_boxes(key)
    for i, cr in enumerate(spec['crops']):
        x0, y0, x1, y1 = [int(round(v)) for v in cr]
        x0 = max(0, x0); y0 = max(0, y0); x1 = min(src.size[0], x1); y1 = min(src.size[1], y1)
        p = src.crop((x0, y0, x1, y1))
        draw_callouts(p, spec['callouts'], boxes, 1.0, (x0, y0), pxpt, labels=False, clip=(x0, y0, x1, y1), force_anchor=spec.get('detail_anchor', 'tl'))
        dd = ImageDraw.Draw(p)
        dd.rectangle((0, 0, p.size[0] - 1, p.size[1] - 1), outline=(150, 150, 160), width=3)
        if len(spec['crops']) > 1:
            t = '(%s)' % 'abcdef'[i]
            f = font(9 * pxpt)
            tb = dd.textbbox((0, 0), t, font=f)
            dd.rectangle((8, 8, 8 + tb[2] + 2 * pxpt, 8 + tb[3] + 2 * pxpt), fill=WHITE)
            dd.text((8 + pxpt, 8 + pxpt), t, fill=NAVY, font=f)
        panels.append(p)
    gap = int(4 * pxpt)
    Wt = max(p.size[0] for p in panels)
    Ht = sum(p.size[1] for p in panels) + gap * (len(panels) - 1)
    out_im = Image.new('RGB', (Wt, Ht), WHITE)
    y = 0
    for p in panels:
        out_im.paste(p, (0, y))
        y += p.size[1] + gap
    out = os.path.join(FIG, spec['base'] + '_detail.png')
    out_im.save(out, dpi=(200, 200), optimize=True)
    return out, out_im.size, pxpt


SPECS = [
    {
        'key': 'F01_system_top_png', 'base': 'F01_system_top',
        'callouts': [
            {'num': 1, 'label': 'ScenarioIn', 'boxes': [['In_ScenarioIn']], 'side': 'a'},
            {'num': 2, 'label': 'Operator_Console', 'boxes': [['Operator_Console']], 'side': 'a'},
            {'num': 3, 'label': 'Operator_RC', 'boxes': [['Operator_RC']], 'side': 'b', 'anchor': 'bl'},
            {'num': 4, 'label': 'MCU_In', 'boxes': [['MCU_In']], 'side': 'l', 'anchor': 'bl'},
            {'num': 5, 'label': 'MCU', 'boxes': [['MCU']], 'side': 'b', 'anchor': 'br'},
            {'num': 6, 'label': 'BMS', 'boxes': [['BMS']], 'side': 'a', 'anchor': 'tr'},
            {'num': 7, 'label': 'Plant', 'boxes': [['Plant']], 'side': 'a'},
            {'num': 8, 'label': 'CAN1_Net', 'boxes': [['CAN1_Net']], 'side': 'l'},
            {'num': 9, 'label': 'Sensors', 'boxes': [['Sensors']], 'side': 'l'},
            {'num': 10, 'label': 'CAN2_Link', 'boxes': [['CAN2_Link']], 'side': 'l', 'anchor': 'bl'},
            {'num': 11, 'label': 'MissionCmd_RT', 'boxes': [['MissionCmd_RT']], 'side': 'b', 'anchor': 'bl'},
            {'num': 12, 'label': 'GNC_Variant', 'boxes': [['GNC_Variant']], 'side': 'l'},
            {'num': 13, 'label': 'Sim_Monitors', 'boxes': [['Sim_Monitors']], 'side': 'a', 'anchor': 'tr'},
            {'num': 14, 'label': 'Viz_Logging', 'boxes': [['Viz_Logging']], 'side': 'b', 'anchor': 'br'},
            {'num': 15, 'label': '*_Delay', 'boxes': [['EscFb_Delay', 'EscSlow_Delay', 'Impl_Delay', 'Bms_Delay'], ['ElecHw_Delay'], ['Adc_Delay'], ['BMS_Cmd_Delay']], 'side': 'r', 'anchor': 'tr'},
        ],
        'crops': [(1130, 3770, 3210, 5560)],
    },
    {
        'key': 'F09_power_plant_png', 'base': 'F09_power_plant',
        'callouts': [
            {'num': 1, 'label': 'Battery_Pack', 'boxes': [['Battery_Pack']], 'side': 'a'},
            {'num': 2, 'label': 'Pack_Thermal', 'boxes': [['Pack_Thermal']], 'side': 'a'},
            {'num': 3, 'label': 'PDU', 'boxes': [['PDU']], 'side': 'l'},
            {'num': 4, 'label': 'Safety_Chain', 'boxes': [['Safety_Chain']], 'side': 'a'},
            {'num': 5, 'label': 'LV_Distribution', 'boxes': [['LV_Distribution']], 'side': 'l'},
            {'num': 6, 'label': 'Implement_Loads', 'boxes': [['Implement_Loads']], 'side': 'l'},
            {'num': 7, 'label': 'Dock_Charger', 'boxes': [['Dock_Charger']], 'side': 'b', 'anchor': 'bl'},
            {'num': 8, 'label': 'Power_Sensors', 'boxes': [['Power_Sensors']], 'side': 'l'},
        ],
        'crops': [(700, 840, 2780, 1660), (700, 3990, 2780, 4945)],
    },
    {
        'key': 'F07_drive_side_png', 'base': 'F07_drive_side',
        'callouts': [
            {'num': 1, 'label': 'ESC (drive_esc)', 'boxes': [['ESC']], 'side': 'a'},
            {'num': 2, 'label': 'Plant: motor variant', 'boxes': [['Plant']], 'side': 'a'},
            {'num': 3, 'label': 'Gearbox', 'boxes': [['Gearbox']], 'side': 'a'},
            {'num': 4, 'label': 'Brake', 'boxes': [['Brake']], 'side': 'a'},
            {'num': 5, 'label': 'Thermal', 'boxes': [['Thermal']], 'side': 'a'},
            {'num': 6, 'label': '1 ms re-timing', 'boxes': [['ZOH_brk'], ['ZOH_T'], ['ZOH_meas'], ['ZOH_gb']], 'side': 'b', 'anchor': 'bl'},
            {'num': 7, 'label': 'Fault injection', 'boxes': [['FaultSel', 'FaultStuck'], ['FaultSides']], 'side': 'a'},
            {'num': 8, 'label': 'Power and energy integrals', 'boxes': [['p_dc', 'p_spr', 'p_shaft', 'P_loss', 'p_diff'], ['E_dc', 'E_spr', 'E_loss']], 'side': 'b', 'anchor': 'bl'},
            {'num': 9, 'label': 'Truth bus', 'boxes': [['Truth']], 'side': 'r'},
        ],
        'crops': [(540, 250, 2620, 530), (2560, 250, 4640, 530)],
    },
    {
        'key': 'F08_drive_esc_png', 'base': 'F08_drive_esc',
        'callouts': [
            {'num': 1, 'label': 'Cmd_In', 'boxes': [['Cmd_In']], 'side': 'b', 'anchor': 'bl'},
            {'num': 2, 'label': 'Speed_Meas', 'boxes': [['Speed_Meas']], 'side': 'l'},
            {'num': 3, 'label': 'Temp_Sens', 'boxes': [['Temp_Sens']], 'side': 'l'},
            {'num': 4, 'label': 'Torque_Limiter', 'boxes': [['Torque_Limiter']], 'side': 'b', 'anchor': 'bl'},
            {'num': 5, 'label': 'Protections', 'boxes': [['Protections']], 'side': 'r', 'anchor': 'tr'},
            {'num': 6, 'label': 'ESC_SM', 'boxes': [['ESC_SM']], 'side': 'a', 'anchor': 'tr'},
            {'num': 7, 'label': 'Speed_Ctrl', 'boxes': [['Speed_Ctrl']], 'side': 'l'},
            {'num': 8, 'label': 'Output_Stage', 'boxes': [['Output_Stage']], 'side': 'b', 'anchor': 'br'},
            {'num': 9, 'label': 'Feedback', 'boxes': [['Feedback']], 'side': 'l'},
        ],
        'crops': [(1900, 300, 3980, 1880)],
    },
    {
        'key': 'F03_mcu_top_png', 'base': 'F03_mcu_top', 'max_h_pt': 453.0,  # 16 cm: each overview sits on a page of its own (fix round 2)
        'callouts': [
            {'num': 1, 'label': 'Task_1k', 'boxes': [['Task_1k']], 'side': 'a', 'anchor': 'tr'},
            {'num': 2, 'label': 'Task_5', 'boxes': [['Task_5']], 'side': 'b', 'anchor': 'br'},
            {'num': 3, 'label': 'Task_10', 'boxes': [['Task_10']], 'side': 'r', 'anchor': 'tr'},
            {'num': 4, 'label': 'Task_50', 'boxes': [['Task_50']], 'side': 'l'},
            {'num': 5, 'label': 'Task_100', 'boxes': [['Task_100']], 'side': 'l'},
            {'num': 6, 'label': 'CAN1_Codec, CAN2_Codec', 'boxes': [['CAN1_Codec'], ['CAN2_Codec']], 'side': 'l'},
            {'num': 7, 'label': 'IO_In', 'boxes': [['IO_In']], 'side': 'r', 'anchor': 'tr'},
            {'num': 8, 'label': 'IO_Out', 'boxes': [['IO_Out']], 'side': 'l'},
        ],
        'crops': [(2200, 880, 4280, 2900)],
    },
    {
        'key': 'F04_mcu_task10_png', 'base': 'F04_mcu_task10', 'max_h_pt': 453.0,  # 16 cm: each overview sits on a page of its own (fix round 2)
        'callouts': [
            {'num': 1, 'label': 'RC_Mapping', 'boxes': [['RC_Mapping']], 'side': 'b', 'anchor': 'bl'},
            {'num': 2, 'label': 'Odom (sens_odom_compute)', 'boxes': [['Odom']], 'side': 'b', 'anchor': 'bl'},
            {'num': 3, 'label': 'Safety_Supervisor', 'boxes': [['Safety_Supervisor']], 'side': 'b', 'anchor': 'bl'},
            {'num': 4, 'label': 'Mode_Manager', 'boxes': [['Mode_Manager']], 'side': 'a'},
            {'num': 5, 'label': 'Vehicle_Ctrl', 'boxes': [['Vehicle_Ctrl']], 'side': 'a', 'anchor': 'tr'},
            {'num': 6, 'label': 'Telemetry', 'boxes': [['Telemetry']], 'side': 'l'},
            {'num': 7, 'label': 'Bus creators', 'boxes': [['Platform_BC'], ['Status_BC']], 'side': 'r'},
            {'num': 8, 'label': 'Alive counter', 'boxes': [['Alive10_prev', 'Alive10', 'Alive10_inc']], 'side': 'b', 'anchor': 'bl'},
        ],
        'crops': [(4250, 3930, 6330, 5880)],
    },
    {
        'key': 'F15_mode_manager_png', 'base': 'F15_mode_manager',
        'callouts': [
            {'num': 1, 'label': 'DI bit masks', 'boxes': [['DI_key_on_mask', 'DI_key_on', 'DI_safety_ok_mask', 'DI_lid_closed', 'DI_dock_present_mask'], ['DI_dock_present']], 'side': 'a'},
            {'num': 2, 'label': 'Standstill', 'boxes': [['Standstill'], ['V_abs']], 'side': 'r', 'anchor': 'tr'},
            {'num': 3, 'label': 'Self-test checks', 'boxes': [['Drv_age_max', 'Drv_fresh', 'Bms_fresh', 'V24_ok', 'V12_ok', 'V5_ok', 'Selftest_ok']], 'side': 'b', 'anchor': 'bl'},
            {'num': 4, 'label': 'Mode_FSM', 'boxes': [['Mode_FSM']], 'side': 'l'},
            {'num': 5, 'label': 'Bms_req', 'boxes': [['Bms_req']], 'side': 'b', 'anchor': 'br'},
            {'num': 6, 'label': 'Parameter constants', 'boxes': [['Prm_mask', 'Prm_flag'], ['Prm_n']], 'side': 'l'},
        ],
        'crops': [(880, 1000, 2960, 2720)],
    },
]

if __name__ == '__main__':
    import sys
    only = sys.argv[1:]
    res = {}
    for sp in SPECS:
        if only and sp['base'] not in only:
            continue
        o, sz = overview(sp)
        d, dsz, pxpt = detail(sp)
        print('%-20s overview %s  detail %s  (10 pt block names print at %.1f pt)' % (
            sp['base'], sz, dsz, 10 * COL_PT / (dsz[0] * 72 / 200)))
