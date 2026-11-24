"""Crop the licence title bars (and the Copilot side panel) from the two window captures and add
Table 15 callouts to the Simulink editor capture."""
import os
from PIL import Image, ImageDraw
import make_callout_figs as M
FIG = M.FIG
# MATLAB desktop: drop the title bar (y < 38), the Copilot Chat panel and side bar (x >= 1232)
# and the empty lower part of the window (y >= 860)
im = M.load_rgb(os.path.join(FIG, 'F16_matlab_desktop.png'))
im.crop((0, 38, 1232, 860)).save(os.path.join(FIG, 'F16_matlab_desktop_crop.png'), dpi=(144, 144), optimize=True)
# Simulink editor: drop the title bar, keep the toolstrip, canvas and status bar
im = M.load_rgb(os.path.join(FIG, 'F17_simulink_editor.png')).crop((0, 38, 1942, 1222))
boxes = {
    'CAN1_Net': (562, 380, 775, 580), 'CAN2_Link': (880, 625, 1093, 800), 'MCU_In': (1162, 775, 1305, 1160),
    'MCU': (1393, 1053, 1568, 1112), 'Sim_Monitors': (1610, 987, 1787, 1160),
    'EscFb_Delay': (968, 302, 1012, 345), 'EscSlow_Delay': (968, 383, 1012, 427), 'Impl_Delay': (968, 465, 1012, 508),
    'Bms_Delay': (968, 540, 1012, 583), 'ElecHw_Delay': (313, 822, 355, 865), 'Adc_Delay': (650, 872, 693, 915),
    'BMS_Cmd_Delay': (594, 1065, 637, 1108),
}
boxes = {k: (v[0], v[1] - 38, v[2], v[3] - 38) for k, v in boxes.items()}
call = [
    {'num': 4, 'boxes': [['MCU_In']]}, {'num': 5, 'boxes': [['MCU']]}, {'num': 8, 'boxes': [['CAN1_Net']]},
    {'num': 10, 'boxes': [['CAN2_Link']]}, {'num': 13, 'boxes': [['Sim_Monitors']], 'anchor': 'tr'},
    {'num': 15, 'boxes': [['EscFb_Delay'], ['EscSlow_Delay'], ['Impl_Delay'], ['Bms_Delay'], ['ElecHw_Delay'], ['Adc_Delay'], ['BMS_Cmd_Delay']]},
]
M.draw_callouts(im, call, boxes, 1.0, (0, 0), im.size[0] / M.COL_PT, labels=False)
im.save(os.path.join(FIG, 'F17_simulink_editor_callouts.png'), dpi=(144, 144), optimize=True)
print('ok')
