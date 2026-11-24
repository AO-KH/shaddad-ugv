# Live RC drive with the 3-D view (Phase 1)

Drive the Shaddad digital twin with a **PS5 controller** or your **RC transmitter** and watch the robot move in 3-D.
The controller is detected automatically (6 axes and 13–18 buttons = PS5 layout, otherwise transmitter layout).

## What runs

Your controller (PS5 or transmitter in USB joystick mode) → **Simulation 3D Joystick** block → `joy_map` (stick and switch mapping)
→ simulated transmitter, SBUS frames and receiver (`RC_Link_Sim`) → STM32 firmware model (SBUS decoder, RC
mapping, mode manager, safety, vehicle control) → CAN → motor controllers → motors and gearboxes → tracks on
the soil, powered by the battery and BMS. The robot is drawn live by `shaddad.Robot3DView` (`VC_VIZ = 3`).

## PS5 controller (DualSense)

Connect it by USB cable or Bluetooth (Windows sees it as "Wireless Controller"). No setup on the controller.

| Control | Action |
|---|---|
| Left stick up / down | Throttle (forward / reverse) |
| Right stick left / right | Steering |
| **Cross** | Arm / disarm (sticks centred) |
| **Circle** | E-stop (latched; also disarms) |
| **Triangle** (hold) | Reset after an E-stop or fault, then press Cross to arm again |
| R1 / L1 | Speed scale up / down: 25 – 50 – 75 – 100 % (starts at 100 %) |
| Options | MANUAL / HOLD |
| Square | Lights on / off |
| Create | Implement on / off |
| L2 / R2 | Implement lift up / down (held) |

Because gamepad buttons spring back, `joy_map` latches arm, E-stop, speed, HOLD, lights and implement. The
mapping is `scen.joy.pad` in `params/param_sim.m` (axis and button numbers, dead zone `dz`).

```matlab
phase1_joystick_check          % live view of sticks, buttons and the resulting drive commands
res = phase1_live_drive();     % drive (PS5 auto-detected)
```

## RC transmitter

### One-time setup

1. Connect the transmitter to the PC by USB and select **USB Joystick (HID)** on the radio
   (EdgeTX/OpenTX "Classic" mode: CH1–CH8 become axes 1–8, CH9 and up become buttons 1, 2, …).
2. Check the mapping:

   ```matlab
   phase1_joystick_check
   ```

   Move every stick and switch. Expected: throttle up = `stick_thr` positive, steering right =
   `stick_steer` positive, CH5 low = `MANUAL`, CH6 up = armed, CH12 = E-stop.
3. If something is reversed or on the wrong channel, edit `scen.joy` in `params/param_sim.m` and run
   `shaddad_params`:

   | Field | Meaning | Default |
   |---|---|---|
   | `ax`  | axis index for [steer thr aux_lv aux_lh mode arm speed impl] (0 = not used) | `1:8` |
   | `inv` | sign per axis entry (-1 reverses it) | all `1` |
   | `btn` | button index for [reset lights estop] (CH9, CH10, CH12) | `[1 2 4]` |
   | `sw3` | threshold of the 3-position switches (mode, implement) | `0.33` |

   If CH7 has no knob on your radio, set `ax(7) = 0` (full speed); otherwise its lowest position is 25 % speed.

## Drive

```matlab
res = phase1_live_drive();                          % PS5 or transmitter (auto), 3-D view, real-time pacing
res = phase1_live_drive('Controller', 'ps5');       % force a layout ('ps5' | 'transmitter')
res = phase1_live_drive('Implement', 1);            % with the mower (2 tiller, 3 sprayer)
res = phase1_live_drive('Camera', 2);               % 1 chase, 2 side, 3 top, 4 free (mouse)
res = phase1_live_drive('Source', 'demo');          % no hardware: a scripted drive, same view
shaddad.Robot3DView.replay(res.file)                % replay any saved run in 3-D (also test drives / scenarios)
```

1. Wait until the HUD shows **STANDBY** (about 1 s simulated) with the sticks centred.
2. Arm: **Cross** (PS5) or **CH6** on with CH5 low (transmitter) → **MANUAL**.
3. Drive with the sticks; PS5 R1/L1 or transmitter CH7 set the speed scale.
4. E-stop: **Circle** (PS5) or **CH12** (transmitter), latched. To continue: Triangle (PS5) or CH12 off + CH9
   (transmitter), then arm again.
5. Stop with the red **Stop** button or **Esc**. Cameras: buttons or keys **1–4**.

The first start compiles the model (**about 2–3 minutes** on the reference PC, with no output) before the
3-D window opens; later starts are faster.

## Check a run

Every run is saved as `work/results/L1_<date>_<time>.mat`. After the run:

```matlab
plot_run(res)                                   % all signals, six pages (below)
plot_run('L1')                                  % newest live drive (also 'D1', 'S1', ... or a log name)
plot_run(res, 'Pages', {'drive', 'battery'})    % some pages only
plot_run(res, 'TimeRange', [200 236])           % zoom (the time axes of a page are linked)
plot_run(res, 'Save', true)                     % PNGs in work/reports/plots_<log name>/
shaddad.Robot3DView.replay(res.file)            % 3-D replay
```

| Page | What it shows |
|---|---|
| drive | sticks and speed scale, speed and yaw-rate command vs response, path on the field (colour = speed, red = outside the field map), MCU mode |
| tracks | track speed reference / encoder / belt / ground, motor speed, torque with feed-forward and MCU limit, slip, motor power and losses |
| battery | SOC (true, BMS, MCU), pack voltage and current, power, cumulative energy and Wh/km, cell voltages and temperatures, BMS limits, remaining energy |
| power | LV rails, HV bus, contactors and safety chain, conversion losses, motor / ESC / enclosure temperatures |
| rc | all 14 RC channels, link quality and lost frames, SBUS counters, MCU fault and warning bits, command source and ESC states |
| nav | position and heading estimate vs truth (Phase 2), roll / pitch, accelerations, GNSS, navigation flags, slip, ground contact |

Grey bands = not armed, yellow = HOLD, red = ESTOP / FAULT. All six pages take about a minute.

## Speed on the reference PC (measured 2026-10-26, Normal mode)

| Setting | Real-time factor |
|---|---|
| Fast preset, no view (`'View','none'`) | about 1.07× |
| Fast preset, 3-D view at 10 Hz (default) | about 0.85× |
| With the mower or sprayer | about 0.6× |
| `'Fidelity','full'` | about 0.3–0.5× |

Pacing holds the run at real time when the PC is fast enough; otherwise the robot moves slightly slower than
real time (the HUD shows the simulated time). Accelerator mode would give full real time with the view but is
blocked by Windows Application Control on this PC.

## Known limits

* No controller connected → `phase1_live_drive` stops at once with a message (use `'Source','demo'`).
* The Simulation 3D Joystick block has no D-pad (POV) output, so the PS5 D-pad is not used.
* The on-screen dashboard (`VC_OPSRC = 2`) does not work yet: its sliders are bound to Constant blocks whose
  values are expressions (`P.scen.dash.*`).
* The fast preset uses a kinematic chassis: no pitch or roll on slopes. Use `'Fidelity','full'` to see the body
  tilt on slopes (slower).
* VR Sink / X3D worlds and the old Joystick Input block were removed in MATLAB R2026b; Simulation 3D (Unreal)
  runs at about 0.37× on this PC's integrated graphics, so the view uses MATLAB graphics.

## Files

`scripts/phase1_live_drive.m`, `scripts/phase1_joystick_check.m`, `scripts/plot_run.m`, `scripts/scenarios/scn_L1_live_drive.m`,
`utilities/+shaddad/Robot3DView.m`, `utilities/joy_map.m`, `params/param_sim.m` (`viz.Ts3d`, `viz.camera`,
`scen.joy`), `scripts/build/build_viz_logging.m` (VC_VIZ 3), `scripts/build/build_shaddad_system.m`
(Operator_Console/Joystick, VC_OPSRC 3).
