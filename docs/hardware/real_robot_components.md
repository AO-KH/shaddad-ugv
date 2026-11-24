# Real robot: components, dimensions and masses (model data sheet)

Source data received 2026-10-27 (files in `docs/hardware/source/`). The model in `params/` is re-parameterised
to this robot. Every value is marked:

* **[DATA]** read from a source file (the file is named);
* **[EST]** estimated (from the drawings, from the photos or from typical parts of this class) - replace it
  when the real value is known, then run `shaddad_params` and the tests listed at the end.

## 1. Vehicle

| Item | Value | Source |
|---|---|---|
| Robot | XMachines "MAX" tracked platform (3 road wheels per side, sprocket and idler at the ends) | [DATA] `spec_sheet_dimensions_weight.jpg` |
| Overall length, robot + towed module | 1730 mm | [DATA] spec sheet (the spec sheet's towed-module figure; Shaddad carries the sprayer on the deck, owner, 2026-11-15) |
| Overall width | 780 mm | [DATA] spec sheet |
| Overall height | 570 mm | [DATA] spec sheet |
| Mass, robot | 220 kg | [DATA] spec sheet |
| Mass, module (sprayer) | 50 kg (dry) | [DATA] spec sheet ("Robot + Module: 220 kg + 50 kg") |
| Robot body length (track) | 1.00 m | [EST] the 1730 mm bracket includes the towed module (user, 2026-10-27); side view scaled by that bracket gives about 0.94 m; the sprayer is on the deck (owner, 2026-11-15) |
| Track gauge B (centre to centre) | 0.60 m | [EST] front view |
| Track width b | 0.13 m | [EST] typical rubber track for 200-300 kg |
| Ground contact length L | 0.75 m | [EST] first to last road wheel in the side view |
| Sprocket pitch radius | 0.090 m | [EST] side view |
| Road wheels | 3 per side, radius 0.077 m | [DATA] count, [EST] size |
| Ground clearance | 0.12 m | [EST] |
| Sprayer tank | 40 L | [EST] fits the 50 kg module |
| Top speed | 1.41 m/s (5 km/h) at 3000 rpm | [EST] user choice: "estimate for 5 km/h" |

### Mass breakdown (as built: sums to 220 kg)

| Component | kg | Note |
|---|---|---|
| Chassis frame | 81.7 | [EST] remainder to the 220 kg total (with the four as-built drive units) |
| Body panels, deck rail | 20 | [EST] |
| Track assembly, each side (rubber track, sprocket, idler, 3 road wheels, frame) | 2 x 22 | [EST] |
| Drive unit at each sprocket, 4 (motor 3.5, gearbox 3.0, brake 0.5, half dual ESC 0.8, cables 0.4) | 4 x 8.2 | [EST] motor mass typical for an 80 mm 750 W BLDC |
| With the proposed 1.0 kW units (motor 4.5, gearbox 3.5) | 4 x 9.7 | [EST] robot 226 kg (+6 kg) |
| Battery pack (195 cells 13.7 kg, 3 mm aluminium box 3.3 kg, BMS / busbars / cables 1.5 kg) | 18.5 | [EST] from [DATA] cell count and box drawing |
| Electronics box (boards, enclosure, harness) | 6 | [EST] |
| Sensor mast (Phase 2 GNSS / LiDAR; not in the photos) | 3 | [EST] model addition |
| Cooling, misc. | 4 | [EST] |
| Front coupler / rear hitch | 4 / 6 | [EST] |

## 2. Battery (13S15P Li-ion NMC, about 3 kWh)

| Item | Value | Source |
|---|---|---|
| Cells | Molicel P42A (21700), 195 cells = 13 series x 15 parallel | [DATA] `battery_spec_sheet.pdf` |
| Cell capacity | 4.2 Ah -> pack 63 Ah | [DATA] |
| Voltage | 46.8 V nominal (3.6 V x 13), 54.6 V full (4.2 V x 13) | [DATA] |
| Energy | about 3.0 kWh (46.8 V x 63 Ah = 2.95 kWh) | [DATA] |
| Empty voltage | 39.0 V (3.0 V x 13); BMS cell cut-off 2.8 V | [EST] P42A datasheet range 2.5-4.2 V |
| Cell resistance | 14-16 mOhm DC at 25 degC, mid SOC | [EST] typical P42A |
| Cell current | 45 A continuous discharge, 4.2 A standard charge | [EST] P42A datasheet values |
| Cell mass | 70 g | [EST] P42A datasheet value |
| BMS | 13S Li-ion / NMC, 120-150 A continuous, 200-300 A peak, CAN 500 kbit/s | [DATA] |
| Connectors | SB120 main power, SB50 charger, GX-12 6-pin CAN (pin 1 CANH, pin 2 CANL) | [DATA] |
| Charger | 54.6 V, 20 A (about 0.33 C, 2-3 h 0-100 %), 100-240 VAC, PFC, IP54, fan cooled | [DATA] |
| Enclosure | 393 x 358.5 x 87 mm, 3 mm aluminium sheet, handle; cables 550 mm | [DATA] `battery_box_drawing_XMT-P1-E01-000.pdf` (FLIC FARM PRIVATE LIMITED, 04-09-2025) |

## 3. Drive

| Item | Value | Source |
|---|---|---|
| Traction motors | 4 x Movematic BLDC, P/N 080-240-0x (80 mm flange), 48 V DC, 750 W, 3000 rpm: two per track, one on the front and one on the rear sprocket | [DATA] `drive_motor_nameplate_movematic.jpg` (S/N CFH0136); count and layout: user, 2026-10-27 |
| Rated torque | 2.39 N m (750 W at 3000 rpm) | derived from [DATA] |
| Peak torque | 7.16 N m (3 x rated) | [EST] |
| No-load speed at 48 V | 3600 rpm; mechanical max 4000 rpm | [EST] |
| Pole pairs, flux, resistance, inductance, inertia | 4, 0.0184 Wb, 64 mOhm, 0.25 mH, 2.0e-4 kg m^2 | [EST] consistent with 48 V / 3000 rpm / 2.39 N m |
| Gearbox | 20:1, efficiency 0.90, one per motor | [EST] gives 1.41 m/s at 3000 rpm with the 0.090 m sprocket |
| Motor controllers (ESC) | one dual-channel 48 V BLDC/FOC controller per side (both motors of a side get the same command), 20 A rms continuous, 50 A rms peak per channel, CAN | [DATA] dual ESC per side: user, 2026-10-27; ratings [EST] |
| Holding brake | 5 N m per motor (motor side) | [EST] the four hold the full robot on a 20 deg slope with margin 4 |

### Proposed motor upgrade (model default, 2026-10-27)

The model uses a 1.0 kW unit in place of the 750 W Movematic (`params/param_motor_unit.m`, option `kw1`;
`shaddad_params('Motor','movematic750')` gives the as-built robot). Sizing per motor, four motors, 316 kg robot
with a full sprayer, farm-soil motion resistance 0.107 (S6 run), skid-steer lateral resistance 0.5 (wet loam 0.6):

| Duty point | Torque per motor |
|---|---|
| flat spraying, 1.0 m/s | 0.42 N m |
| 20 deg climb, 1.0 m/s | 1.72 N m |
| 20 deg climb + 0.8 m/s^2 | 2.03 N m |
| pivot turn on flat ground | 1.63 N m |
| turn on the 20 deg slope | 2.85 N m |
| turn on the 20 deg slope, wet loam | 3.42 N m |
| RMS of a hilly-farm spraying cycle (farm soil / wet loam) | 1.45 / 1.81 N m |

At 45 degC ambient the 750 W motor gives about 2.15 N m continuous: RMS margin 1.19 on wet loam, and every slope
turn above the rating. The proposed unit (all [EST]):

| Item | 1.0 kW unit |
|---|---|
| Rating | 48 V, 1.0 kW, 3000 rpm, 3.18 N m continuous, 9.55 N m peak (2.0 kW peak power) |
| Electrical | 4 pole pairs, 0.0184 Wb (no-load 3600 rpm at 48 V), 48 mOhm, 0.19 mH, 2.8e-4 kg m^2 |
| Mass | motor 4.5 kg, gearbox 3.5 kg (20:1, T2B 180 N m) |
| ESC channel | 27 A rms continuous, 65 A rms peak, 55 A DC motoring |
| Margin at 45 degC | 1.58 on the wet-loam RMS, slope turns within the continuous rating on farm soil |

It keeps the 48 V bus, the 20:1 gear and the 5 km/h top speed, and its currents stay within the 150 / 250 A BMS
(a 1.5 kW unit would need a larger BMS). The traction fuses grow from 60 to 125 A per side.

## 4. Electronics (seen in the photos)

| Item | Source |
|---|---|
| XMachines "XT_PS V1.0" board: +48 V input, CAN, optocoupled I/O, carries an STM32 Nucleo-32 board | [DATA] `electronics_box_xt_ps_nucleo.jpeg` |
| Second controller: STM32 board with USB-C, BOOT0 / NRST / KEY buttons ("Black Pill" style) | [DATA] `electronics_ftdi_stm32_board.jpeg` |
| FTDI-style USB-UART adapter (red, DTR/RX/TX/VCC/CTS/GND, 3.3 / 5 V jumper) | [DATA] `electronics_ftdi_closeup.jpg` |
| ADS1115 / ADS1015 16-bit I2C ADC, current-sensor header "CUR" | [DATA] `electronics_ads1115_adc.jpg` |
| LM2596 DC-DC buck module (max 40 V input, so fed from a lower rail, not from 48 V) | [DATA] `electronics_lm2596_buck.jpg` |
| XMachines "SPR V1.3" sprayer board | [DATA] `electronics_xmachines_spr_v1_3_board.jpg` |
| Optocouplers on a perfboard, 8-channel level shifter | [DATA] electronics box photo |

The firmware model keeps its STM32 target for now (user, 2026-10-27); simulation results do not depend on it.

**Safety devices (owner, 2026-11-16):** the robot has E-stop buttons and a safety relay with STO wiring. Their parts, category and wiring are pending the supplier's data (card G1-04); they are not in these photos or the spec sheet.

## 5. Sprayer module

| Item | Value | Source |
|---|---|---|
| Pump | brass piston (HTP-type) sprayer pump with pressure regulator, under the chassis | [DATA] `sprayer_pump.jpg` |
| Hose | 12 mm OD x 8 mm ID, high pressure | [DATA] hose print |
| Module mass | 50 kg dry | [DATA] spec sheet |
| Tank | 40 L | [EST] |
| Pump flow / pressure | about 9 L/min at the boom, 3 bar at the nozzles | [EST] (model values kept) |
| Pump motor | 24 V PM DC | [EST] (model values kept; not visible in the photos) |

## 6. What changed in the model (2026-10-27)

| Area | Change | Files |
|---|---|---|
| Structure | 1.00 x 0.78 x 0.57 m, 220 kg mass table, gauge 0.60 m, track width 0.13 m, contact length 0.75 m, sprocket 0.090 m, 3 road wheels, sprayer module 50 kg + 40 L tank (mounted on the deck: the owner's decision of 2026-11-15, "Mounted on the deck"; the spec render's towed module on a caster wheel is not this robot's configuration) | `params/param_structure.m` |
| Gauge factor chi | re-identified on the new geometry with `structure_identify_chi`: farm soil 1.394 (soils 1.32-1.49) | `params/param_structure.m` |
| Drive | Movematic 750 W motor (2.39 / 7.16 N m, 3000 / 3600 rpm), gearbox 20:1, ESC 20 / 50 A rms, brake 5 N m, speed-PI gains scaled with the new inertia, ESC ramps from the gear ratio | `params/param_drive.m` |
| Drive (second change, 2026-10-27) | four motors, two per side on the front and rear sprockets, one dual ESC per side: each side is one equivalent drive of two units (torque, current, inertia, brake and thermal capacity x 2); proposed 1.0 kW unit as the model default; traction fuses 125 A; mass table with four drive units | `params/param_motor_unit.m`, `params/param_drive.m`, `params/param_structure.m`, `params/param_power.m` |
| Tilt (S2 fix, 2026-10-27) | IMU level calibration (roll 1.93 deg, pitch -2.13 deg from a standstill on level ground) subtracted in `fw_imu_tilt`; TILT_WARN 22 -> 25 deg | `params/param_electronics.m`, `scripts/build/build_fw_platform.m`, `scripts/imu_level_calibration.m` |
| Drive tuning (D64, 2026-10-27) | speed PI re-tuned for the four-motor drive on the two-mass model: Kp_w 0.030 / Ki_w 0.090 (PM 62 deg, GM 15.0 dB, Ms 1.37; the gains scaled with the inertia gave Ms 1.65); elasto-plastic brake (Simscape component `shaddad_drive.elastoplastic_brake`): the former friction element chattered at the 1 ms step and, with two brakes per side, diverged in ESTOP (S8a); two-mass bench masses from the robot mass (113 / 158 kg) | `params/param_drive.m`, `scripts/build/build_drive.m`, `subsystems/drive/+shaddad_drive/elastoplastic_brake.ssc` |
| Track relaxation length | `P.structure.track.ell` = L/2 = 0.375 m; it had stayed at 0.50 m of the former 1.00 m track, so the tracks slipped like a 1.00 m track (T4 / T7 slip 25 % below the analytic value) | `params/param_structure.m` |
| Test benches and criteria | the drive benches take their load from the P of each run (they had kept the 300 kg / 0.128 m load of the former robot); the battery bench `tb_pack_bms` rebuilt (it still referenced the 16-group pack model `Pack_Grouped16`); T-EL-16 LV draw + 23 W for the two extra brake holds; T-PWR-06b overload = 1.5 x F_TL rating (still fails: at 187.5 A the 150 A BMS opens the main contactor after 1.3 s, before the 125 A fuse; fuse / BMS coordination is open) | `tests/drive/tb_drive_run.m`, `tb_drive_load.m`, `build_tb_drive.m`, `tests/electronics/t_elec_plant.m`, `tests/power/pp_case_def.m` |
| Battery | 13S15P Molicel P42A NMC (cylindrical 21700 cells, NMC OCV / resistance tables), BMS 150 / 250 A, charger 54.6 V 20 A, enclosure thermal, fuses; 13 cell groups in buses, BMS and pack builders | `params/param_power.m`, `buses/buses_power.m`, `scripts/build/build_shaddad_battery.m`, `build_power_pack.m`, `build_shaddad_bms.m` |
| MCU limits | top speed 1.41 m/s (manual 1.4, auto 1.2), overspeed 1.70 m/s, DCL warning 25 A | `params/param_electronics.m` |
| GNC | footprint radius, obstacle corridor, Stanley wheelbase from the geometry; energy per metre 0.12 Wh/m | `params/param_gnc.m` |
| Sensors | IMU, LiDAR and perception self-box at the new body | `params/param_sensors.m` |
| Shift budget | 3 h shift, 520 W working / 380 W transit, end-of-shift SOC 0.32 at end of life | `params/param_power.m` |
| 3-D view | real robot shape (tracks, 3 road wheels, red body, deck rail, 40 L tank), cameras for a 1 m robot | `utilities/+shaddad/Robot3DView.m` |
| Tests / scenarios | expectations derived from P instead of the old numbers; motor-efficiency points (drive T6) pending the motor datasheet | `tests/**`, `scripts/scenarios/scn_S2..S5` |

Rebuilt: `build_shaddad_battery`, `build_power_pack`, `build_shaddad_bms('model')`, `build_power_plant`. A backup of the
previous robot (sources and models) is in `work/backup/pre_real_robot_20261027.zip`.

## 7. Open items (send these to replace the estimates)

1. Track: gauge, track width, ground contact length, sprocket teeth and pitch.
2. Gearbox ratio and type of the four gearboxes (both sprockets are driven).
3. Dual-channel motor controller model (current ratings, CAN protocol) and the motor datasheet (peak torque, Kt, R, L,
   inertia, pole pairs); for the upgrade, the chosen 1.0 kW motor and its datasheet.
4. Sprayer: tank volume, pump model (flow and pressure), pump motor, number of nozzles, boom width.
5. Main controller chip of the Nucleo-32 board and of the USB-C STM32 board.
6. LV supply: which converter makes the 12 V / 24 V rails from 48 V (the LM2596 accepts at most 40 V).
7. Safety devices: the E-stop buttons (number and place), the safety relay (model and category) and the STO wiring of the ESCs (card G1-04).
