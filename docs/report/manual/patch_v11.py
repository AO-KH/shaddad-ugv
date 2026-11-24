"""Patch make_manual_chapters.py for manual v1.1: live drive (PS5 / RC transmitter, 3-D view) and troubleshooting."""
import os

D = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(D, 'make_manual_chapters.py')
s = open(p, encoding='utf-8').read()


def rep(a, b):
    global s
    if a not in s:
        raise SystemExit('pattern not found: ' + a[:80])
    s = s.replace(a, b)


rep('driving the robot with the simulated SBUS remote-control transmitter. The run includes',
    'driving the robot by remote control, either with a scripted plan or live with a PS5 controller or an RC '
    'transmitter, with a 3-D view of the robot. The run includes')
rep('{"type": "p", "text": "Four commands cover the normal workflow:"},',
    '{"type": "p", "text": "Five commands cover the normal workflow:"},')
rep('"res = phase1_test_drive();      % demo RC drive: summary, figure, replay (about 5 min)\\n"',
    '"res = phase1_test_drive();      % scripted RC drive: summary, figure, replay (about 5 min)\\n"\n'
    '                             "res = phase1_live_drive();      % drive live with a PS5 controller or RC transmitter, 3-D view\\n"')
rep('["Optional", "Simulink Coder (code generation for the STM32)',
    '["Live drive", "Simulink 3D Animation (Simulation 3D Joystick block); a PS5 controller (USB or Bluetooth) or an RC '
    'transmitter with a USB joystick mode"],\n        ["Optional", "Simulink Coder (code generation for the STM32)')

LIVE = r'''chapters.append({"id": "um05_live", "title": "Drive Live with a PS5 Controller or RC Transmitter", "blocks": [
    {"type": "p", "text": "`phase1_live_drive` lets you drive the robot yourself and watch it in 3-D ({fig:F35_live3d_side.png}). Your controller feeds the simulated RC transmitter, so the commands pass the same SBUS frames, STM32 decoder, mode manager, safety logic, CAN, motor controllers, motors and tracks as in the tests. The controller is detected automatically: 6 axes and 13-18 buttons = PS5 layout, otherwise the RC transmitter layout."},
    {"type": "figure", "file": "F35_live3d_side.png", "caption": "Live 3-D view (side camera) with the mower: MCU mode, speed, belt speeds and SOC at the top, camera buttons and Stop at the right, the RC channels the MCU receives at the bottom right."},
    {"type": "h2", "text": "Start"},
    {"type": "code", "text": "phase1_joystick_check                          % first time: check sticks and buttons (close the window to end)\n"
                             "res = phase1_live_drive();                     % PS5 or RC transmitter (auto), 3-D view\n"
                             "res = phase1_live_drive('Implement', 1);       % with the mower (2 tiller, 3 sprayer)\n"
                             "res = phase1_live_drive('Camera', 2);          % 1 chase, 2 side, 3 top, 4 free (mouse)\n"
                             "res = phase1_live_drive('Source', 'demo');     % no controller: a scripted drive, same view\n"
                             "shaddad.Robot3DView.replay(res.file)           % replay any saved run in 3-D"},
    {"type": "note", "kind": "info", "text": "After `Controller found` MATLAB compiles the model for about 2-3 minutes (the first time) without further output; the 3-D window opens when the run starts. Wait for **STANDBY** in the HUD, keep the sticks centred, then arm."},
    {"type": "h2", "text": "PS5 controller"},
    {"type": "p", "text": "Connect it by USB cable or Bluetooth; no setup on the controller is needed. Its buttons spring back, so the model latches arm, E-stop, speed, HOLD, lights and implement."},
    {"type": "table", "id": "tab_ps5", "caption": "PS5 controls", "header": ["Control", "Action"], "widths": [32, 68], "rows": [
        ["Left stick up / down", "Throttle (forward / reverse)"],
        ["Right stick left / right", "Steering"],
        ["Cross", "Arm / disarm (sticks centred)"],
        ["Circle", "E-stop (latched, also disarms)"],
        ["Triangle (hold)", "Reset after an E-stop, then press Cross to arm again"],
        ["R1 / L1", "Speed scale up / down: 25 - 50 - 75 - 100 % (starts at 100 %)"],
        ["Options", "MANUAL / HOLD"],
        ["Square / Create", "Lights on / off; implement on / off"],
        ["L2 / R2 (held)", "Implement lift up / down"],
    ]},
    {"type": "h2", "text": "RC transmitter"},
    {"type": "p", "text": "Select **USB Joystick (HID)** on the radio (EdgeTX/OpenTX Classic mode: CH1-CH8 become axes 1-8, CH9 and up become buttons). If CH7 has no knob on your radio, set `scen.joy.ax(7) = 0` (full speed)."},
    {"type": "table", "id": "tab_rc", "caption": "RC transmitter controls", "header": ["Channel", "Action"], "widths": [32, 68], "rows": [
        ["CH2 / CH1", "Throttle / steering"],
        ["CH5 (3-pos)", "Low = MANUAL, middle = HOLD"],
        ["CH6", "Arm (switch on with the sticks centred)"],
        ["CH7", "Speed scale (knob)"],
        ["CH12", "Remote E-stop (latched); to continue: CH12 off, CH9 reset, cycle CH6"],
    ]},
    {"type": "p", "text": "Mappings are in `params/param_sim.m`: `scen.joy.pad` (PS5 axis and button numbers, dead zone) and `scen.joy.ax / inv / btn` (transmitter). Run `shaddad_params` after a change."},
    {"type": "h2", "text": "The 3-D view"},
    {"type": "figure", "file": "F37_live3d_sprayer.png", "caption": "Free camera with the sprayer: tank on the deck, boom with nozzles at the rear, tracks with moving tread bars, sprockets and road wheels."},
    {"type": "bullets", "items": [
        "Cameras: **Chase**, **Side**, **Top**, **Free** buttons or keys **1-4** (Free: rotate and zoom with the mouse).",
        "The tread bars move and the sprockets spin with the measured belt speeds; the trail shows the path.",
        "**Stop** button or **Esc** ends the run; the summary and the log file follow.",
        "Speed on the reference PC: about 0.85x real time with the view (1.07x without, about 0.6x with an implement). Simulation pacing caps the run at real time when the PC is fast enough.",
    ]},
    {"type": "note", "kind": "warning", "text": "The fast preset uses a kinematic chassis: no pitch or roll on slopes. Use `'Fidelity','full'` to see the body tilt (slower). The on-screen dashboard (`VC_OPSRC = 2`) does not work yet."},
]})

'''
i0 = s.index('chapters.append({"id": "um05_dashboard"')
i1 = s.index('chapters.append({"id": "um06_results"')
s = s[:i0] + LIVE + s[i1:]

rep('        ["A run ends with `ok=0`", "Read `r.msg` for the error text"],',
    '        ["A run ends with `ok=0`", "Read `r.msg` for the error text"],\n'
    '        ["`No USB controller found`", "Connect the PS5 controller or the transmitter (USB Joystick mode); check with `phase1_joystick_check`"],\n'
    '        ["Nothing happens after `Controller found`", "The model is compiling (2-3 min the first time); wait for the 3-D window"],\n'
    '        ["Live drive: robot does not arm", "Sticks centred, then Cross (PS5) or CH6 (RC); after an E-stop first Triangle / CH9 reset"],\n'
    '        ["Live drive: steering or throttle reversed", "Flip the sign in `scen.joy.pad.inv_*` (PS5) or `scen.joy.inv` (RC), run `shaddad_params`"],')
open(p, 'w', encoding='utf-8').write(s)
print('make_manual_chapters.py patched for v1.1')
