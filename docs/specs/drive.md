# DRIVE domain spec: traction motors, motor drives (ESC), gearboxes, parking brakes

> **Revision C (2026-11-18, card G1-01 part 2).** The whole spec is brought in line with the parameters of the four-motor robot (`params/param_drive.m`, `params/param_motor_unit.m`) and the decisions in force; `docs/architecture.md` section 2b overrides it where they differ. **The default motor is the 1 kW unit** (option `kw1`), a planned upgrade of the robot's four 750 W motors (D95, CR-004). The as-built Movematic 750 W motor stays selectable (`shaddad_params('Motor','movematic750')`): this spec gives both where they differ, and every result records its motor. **The delivered ESCs are the baseline, provisionally** (D96): the ESC model of 2.9, 2.10 and 6.4 is an assumption until the ESC manual arrives (card G1-04, R-14).

> **Fix pass (2026-11-18, G1-01 part 2, step 19).** The re-check's nine findings are corrected. The STO signal comes from POWER's `Safety_Chain` (D8). The DC ports have D16's names, and the root ports are D6's frame buses (5.1, 5.2). The section also corrects the speed PI's reset rule, `tau_avail`, the state machine's priority, T21's results and the D51 note, and adds REQ-SYS-031's park chain. Section 10 now maps the 72 requirements allocated to DRV (step 19 added DRV to REQ-SYS-204 and -205, and to the zone's motion stop, REQ-SYS-209).

> **Earlier revisions.** B (2026-10-19): the deceleration chain (D33), the network tie (D35), the loop breakers (D36), the energy audit (D40), the two-mass speed loop (D50), the gearbox rating rule (D51), the bench-only FOC variant (D52). The four-motor robot (2026-10-27): two motors per track, the speed PI, the elasto-plastic brake and the benches that follow P (D64). D70 (2026-11-14): three MATLAB Functions in `drive_esc`. Critical fixes (2026-11-15, D71): 2.9 rows 3, 5 and 6, `t_stop_max`, tests T19b to T19d. Their values are those of this revision.

Project: Shaddad_UGV (tracked skid-steer agricultural UGV, NASL-Tech "Shaddad" class)
Domain key: `drive`. Status: DESIGN, built and tested in the twin (`tests/drive/t_drive.m`). MATLAB R2026b. The library paths of section 3 were checked in the live session when the domain was built.

**Sources.** Each value is the leaf `P.drive.<field>` (its file and line in section 6), the motor unit `P.drive.motor_unit.<field>` (`u.<field>` in `param_motor_unit.m`), a decision row, the as-built record `docs/hardware/real_robot_components.md` (section 3), or a test result with its date. The parameter files mark each value as an estimate [EST] or a datum [DATA]; most drive values are estimates until the supplier data arrive (G1-04).

---

## 0. Key decisions (summary)

| Topic | Decision |
|---|---|
| Motors | **Four** 48 V BLDC traction motors (surface PMSM model), **two per track**: one at the front and one at the rear sprocket (as built: count and layout [DATA], 2026-10-27). **Default, the 1 kW unit** (`kw1`, D95): 1.0 kW and 3.18 N·m continuous at 3000 rpm, 9.55 N·m peak, 2.0 kW peak power, 4.5 kg [EST]; the unit is still to be chosen. **As built, the Movematic 080-240** (`movematic750`): 750 W at 3000 rpm [DATA, nameplate], 2.39 N·m continuous, 7.16 N·m peak, 3.5 kg [EST]. Both: 4 pole pairs, 0.0184 Wb, no-load 3600 rpm at 48 V, 4000 rpm mechanical limit [EST]. |
| Model of a side | **One equivalent drive of the side's two units in parallel** (same speed and voltage; one ESC node per side gives both the same command), `P.drive.n_mot_side` = 2: torque, power, current, inertia, iron losses, brake and thermal capacity × 2; winding resistance and inductance / 2. Per side 6.36 N·m continuous and 19.1 N·m peak (750 W: 4.78 and 14.32 N·m). |
| Gearbox | One 2-stage planetary per motor, **i = 20, eta = 0.90** [EST]: 1.41 m/s (5 km/h) at the rated 3000 rpm with the 0.090 m sprocket. Per gearbox T2N / T2B / T2NOT = 120 / 180 / 240 N·m (750 W: 120 / 140 / 240) [EST]; T2B covers the motor peak through the gear, D51's rule (9.55 × 20 × 0.90 = 172 N·m; 750 W: 129 N·m). |
| Sprocket | Pitch radius **r_s = 0.090 m** [EST] (`P.structure.r_sprocket`, read by `param_drive`); STRUCTURE owns it. |
| Brake | Spring-applied, electrically released (fail-safe) holding brake on each motor shaft, **5 N·m static** [EST]: the four hold the robot with the full sprayer (316 kg) on 20° with a margin of 4.2 (2.7). |
| Motor-controller split | **One dual-channel commercial 48 V FOC controller (ESC) per side**, as delivered (D96, provisional), CiA-402 style, one CAN node per side. The STM32 sends speed setpoints, a torque feed-forward and the motoring and regenerative torque limits over CAN1 at 100 Hz. The STM32 does **not** run FOC. |
| Plant fidelity (default) | **`Motor & Drive (System Level)`** (Simscape Electrical) with its thermal variant: torque-reference driven, loss-map based, no switching or phase states, so multi-hour missions are feasible. |
| High-fidelity variant | `PMSM` + `Average-Value Voltage Source Converter (Three-Phase)` + ee_sl_lib PMSM current control, bench-only (D52). |
| Kinematic variant | Ideal Angular Velocity Source plus an energy-equivalent DC load, for fast GNC Monte Carlo runs. |
| ESC firmware model | Plain Simulink at 1 kHz plus three MATLAB Functions (section 4): state machine, setpoint ramp, speed PI (Kp_w 0.030, Ki_w 0.090, D64), torque limiter (envelope, I²t, thermal, voltage, DC current), protections, brake sequencing, status word, CAN codecs. |
| MATLAB Function blocks | **Three, in `drive_esc`** (section 4): the state machine `ESC_SM` (`esc_sm.m`, D1, Stateflow not installed), and since D70 (2026-11-14) the torque limiter's logic `Torque_Limiter/LimitLogic` (`esc_torque_limit.m`) and the status word and feedback bus `Feedback/FbPack` (`esc_feedback.m`), bit-identical to the block networks they replace. The loss maps and envelopes are generated offline in `param_drive.m`. |

---

## 1. Purpose and scope

### 1.1 In scope (DRIVE owns)
For each side (Left = 1, Right = 2), modelled as one equivalent drive of the side's two motors:
1. **Traction motors**: electromechanical conversion, rotor inertia, losses, torque-speed envelope as a function of the DC voltage.
2. **ESC (motor controller)**: power-stage losses, DC current draw, internal speed and torque control, limits, protections, state machine, brake driver, motor temperature sensing, encoder processing, CAN protocol (codec, timing, time-outs).
3. **Gearboxes** (motor to sprocket): ratio, load-dependent mesh loss, no-load drag, output inertia.
4. **Parking/emergency brakes**: spring-applied brakes, coil electrical load, actuation dynamics, stick-slip holding.
5. **Thermal**: motor winding and housing, ESC heat sink, ambient and solar loading, derating.
6. **Drive sensors**: motor encoder (as seen by the ESC), DC current and voltage sensing in the ESC, winding temperature sensor, truth signals for logging and SENSORS.
7. **The CAN codecs of the drive frames**: `drive_cmd_pack` / `drive_cmd_unpack` and `drive_fb_pack` / `drive_fb_unpack`, single and integer types only, which the MCU reuses. The CAN1 network itself (latency, frame drops) is modelled in `CAN1_Net_Sim` by ELECTRONICS.

### 1.2 Out of scope (owned by neighbours)
| Neighbour | Interface with DRIVE |
|---|---|
| **POWER** | Main 48 V DC bus: the 13S NMC pack, 39.0 to 54.6 V in operation, 46.8 V nominal (`P.power.pack`), contactors, precharge, traction fuses (125 A per side), BMS. Supplies the DC electrical ports `DC48_TracL_p` / `DC48_TracL_n` and `DC48_TracR_p` / `DC48_TracR_n` (D16, ICD 2). Owns the **Electrical Reference** and **the only safety chain** (`Safety_Chain`, D8): its output Safety_OK, the relay's immediate contacts, is the hardwired STO signal `sto_ok` of both ESCs (stop category 0, D86; the STO time is REQ-SYS-063's). The pack's charge-current limit reaches the ESC regen limit through the MCU (`tau_lim_reg`). |
| **STRUCTURE** | Sprockets, tracks, road wheels, ground interaction, vehicle mass and yaw. Receives the mechanical rotational ports `Spr_L/Spr_R` (one per side: the side's two driven sprockets as one). Owns `r_sprocket`. The reflected drive inertia at the sprocket, 0.224 kg·m² per side (750 W: 0.160), is included dynamically through the gearbox; the DRIVE mass budget is in 6.9. |
| **MCU (STM32 firmware, Model Reference)** | Sends the DRV_CMD frames every `P.drive.Ts_can` and receives the FB frames, which `drive_system` decodes into `Bus_DriveCmd` and packs from `Bus_DriveFb` (5.1, 5.2). Runs vehicle-level control: RC/GNC to (v, omega), skid-steer kinematics to motor speed setpoints, traction and torque limits, supervisor. Its HW I/O variant uses the FDCAN blocks with the DRIVE CAN protocol (2.10) and reuses `drive_cmd_pack` and `drive_fb_unpack`. Its copy of the drive values (`P.electronics.drv`) is read from `P.drive`, so it follows the motor option. |
| **ELECTRONICS** | The CAN1 physical layer, `CAN1_Net_Sim` and the harness. The safety chain is POWER's (D8): revision A's `Safety_Chain_HW` is not built. |
| **SENSORS** | May use `Bus_DriveTruth` (true motor and sprocket angle and speed) to model extra sensors. The **motor encoder is modelled here** (inside the ESC) and reaches the MCU over CAN. SENSORS must not model it a second time. |
| **GNC / RC** | No direct interface. They reach the drives only through the MCU. Phase 1 (RC) and phase 2 (GNC) use the **same** drive interface. |
| **STRUCTURE (the world's environment)** | Ambient temperature `T_amb` (K) for the thermal model: `env.T_amb` of STRUCTURE's environment bus (ICD 3.2), from `P.world.env.T_amb` = 318.15 K (45 °C) by default. |

---

## 2. Physical and algorithmic model

### 2.1 Load cases (sizing)
**Design mass:** the robot with the heaviest implement load of the envelope, the sprayer with its full 40 L tank: **316 kg** with the 1 kW units (226 kg robot + 50 kg module + 40 kg liquid) and 310 kg with the as-built motors (220 kg robot). Per side `P.drive.bench.m_side` = 158 kg (750 W: 155 kg). Sprocket r_s = 0.090 m, gear N = 20, eta = 0.90. Motion resistance: farm soil f = 0.107 (the S6 run, as-built record section 3), sandy soil 0.08 (bench T3), firm ground 0.05 (`P.drive.bench.f_rr`). The motor torque of a side is F·r_s/(N·eta).

**Table 2.1. Load cases per side** (forces at the sprockets, motor torque of the side's two motors).

| # | Case | 1 kW: force (N) | 1 kW: motor torque (N·m) | 750 W: force (N) | 750 W: motor torque (N·m) |
|---|---|---|---|---|---|
| L1 | 20° climb, farm soil (f 0.107) | 686 | 3.43 | 673 | 3.36 |
| L2 | L1 + 0.8 m/s² (with the reflected inertia, 27.7 and 19.8 kg per side) | 834 | 4.17 | 813 | 4.06 |
| L3 | 18.5° climb, farm soil (the sprayer's descent limit, CR-001) | 649 | 3.25 | 637 | 3.18 |
| L4 | 20° climb, sandy soil (f 0.08, bench T3) | 647 | 3.23 | 634 | 3.17 |
| L5 | Flat, firm ground (f 0.05) | 77 | 0.39 | 76 | 0.38 |
| L6 | Adhesion limit, mu_adh 0.7 [EST] (the structure spec's shear model sets the real limit) | 1085 | 5.42 | 1064 | 5.32 |

**Turns** (the as-built record's sizing, per motor, 316 kg, skid-steer lateral resistance 0.5, wet loam 0.6): pivot turn on flat ground 1.63 N·m, turn on the 20° slope 2.85 N·m, on wet loam 3.42 N·m; the RMS of a hilly-farm spraying cycle is 1.45 N·m on farm soil and 1.81 N·m on wet loam.

**Design points per side:** the continuous duty, 2.9 to 3.6 N·m RMS for the two motors, against 6.36 N·m continuous (750 W: 4.78 N·m, or about 4.3 N·m at 45 °C ambient by the as-built record's estimate); the highest demand, 6.84 N·m in the turn on a 20° slope on wet loam, against 19.1 N·m peak (750 W: 14.32 N·m). The robot is adhesion-limited: the side's peak through the gear, 19.1 × 20 × 0.90 / 0.090 = 3820 N, is 3.5 times L6.

**Speeds:** the rated 3000 rpm gives 1.41 m/s (`P.electronics.lim.v_track_max` = 1.414 m/s), the no-load 3600 rpm at 48 V gives 1.70 m/s (`P.electronics.lim.v_overspeed`), and the 4000 rpm mechanical limit, where the ESC trips (2.9), gives 1.89 m/s.

### 2.2 Gear ratio and motor torque
`P.drive.gear.k_v2w` = N/r_s = 222.2 (rad/s)/(m/s): the ratio N = 20 [EST] keeps the 5 km/h top speed at the motors' rated speed.

**Table 2.2. The drive of one side.**

| Quantity | Formula | 1 kW (default) | 750 W |
|---|---|---|---|
| Motor torque needed, continuous (L1) | F·r_s/(N·eta) | 3.43 N·m | 3.36 N·m |
| Motor torque needed, peak (L2) | | 4.17 N·m | 4.06 N·m |
| Continuous / peak torque | 2 × unit | 6.36 / 19.1 N·m | 4.78 / 14.32 N·m |
| Motor speed at 0.5 / 1.0 / 1.2 / 1.41 m/s | v·N/r_s | 1061 / 2122 / 2546 / 3000 rpm | the same |
| Reflected inertia at the sprocket | J·N² | 0.224 kg·m² (27.7 kg equivalent) | 0.160 kg·m² (19.8 kg) |
| Power at the sprocket, 20° at 1.0 m/s on farm soil | F·v | 686 W | 673 W |
| DC power, 20° at 1.0 m/s on sandy soil (T3, 2026-11-15) | measured | 847 W at 46.4 V, 18.2 A | not run |

### 2.3 Motor electrical model and voltage check
Surface PMSM, per unit: p = 4 pole pairs, lambda_pm = 0.0184 Wb, so Kt = 1.5·p·lambda = **0.1104 N·m/A_pk**. 1 kW: Rs = 48 mΩ, L = 190 µH, J = 2.8e-4 kg·m²; 750 W: 64 mΩ, 250 µH, 2.0e-4 kg·m² (all [EST], consistent with 48 V, 3000 rpm and the rated torque). The side's equivalent has the same Kt, half the resistance and inductance, and twice the inertia (6.3).
Steady-state phase voltage: V_ph = sqrt((lambda·omega_e + Rs·I)² + (omega_e·L·I)²). SVPWM needs Vdc ≥ √3·V_ph.

**Table 2.3. The DC voltage each operating point needs, per unit.**

| Operating point | 1 kW: current / Vdc needed | 750 W: current / Vdc needed |
|---|---|---|
| 3000 rpm, rated torque | 28.8 A_pk / 44.1 V | 21.6 A_pk / 44.1 V |
| 2000 rpm, peak torque (the envelope's corner) | 86.5 A_pk / 41.4 V | 64.9 A_pk / 41.3 V |
| 3000 rpm, the peak envelope at 48 V (6.37 and 4.77 N·m) | 57.7 A_pk / 50.8 V | 43.2 A_pk / 50.6 V |
| 3600 rpm, no load | 0 / 48.1 V | 0 / 48.1 V |

The pack gives 39.0 to 54.6 V (46.8 V nominal); the motors are rated at 48 V, the envelope's reference voltage `P.drive.motor.V_nom`. The rated point needs about 44 V, so the top speed at rated torque falls on a low pack, as the voltage-dependent envelope (2.4) models. **Finding (card G2-DRV):** above 2500 rpm the peak envelope at 48 V asks for more voltage than the unit's electrical estimates give (50.8 V at 3000 rpm). Both are estimates, and the FOC variant (T15) is compared only at the T1 and T3 torques.

### 2.4 Torque-speed envelope (Motor & Drive "tabulated2D")
Peak envelope per side at V_nom = 48 V: the peak torque up to 2000 rpm, then the unit's peak power (2.0 kW; 750 W: 1.5 kW) folding to 0 at 3600 rpm; the table is 0 above the 4000 rpm mechanical limit. Voltage scaling: T(w, V) = T_nom(w·V_nom/V), the corner speed proportional to Vdc.

```
w_t   [rpm] = [0 500 1000 1500 2000 2500 3000 3300 3600 3800 4000 4200]
T_pk_nom   (1 kW, side)  = [19.1 19.1 19.1 19.1 19.1 15.28 12.74 7.0 0 0 0 0]          % N·m
T_cont_nom (1 kW, side)  = [6.36 6.36 6.36 6.36 6.36 6.36 6.36 4.0 0 0 0 0]          % N·m, ESC I2t fold-back target
T_pk_nom   (750 W, side) = [14.32 14.32 14.32 14.32 14.32 11.46 9.54 5.2 0 0 0 0]
T_cont_nom (750 W, side) = [4.78 4.78 4.78 4.78 4.78 4.78 4.78 3.0 0 0 0 0]
vdc_t [V]   = [36 42 48 54.6]
T_pk_2D(:,j) = interp1(w_t, T_pk_nom, min(w_t*V_nom/vdc_t(j), w_t(end)), 'linear', 0);  T_pk_2D(w_t > n_mech_max, :) = 0
```

**Table 2.4. `P.drive.motor.T_pk_2D` per side, N·m** (rows: speed; columns: DC voltage).

| Speed (rpm) | 1 kW, 36 V | 42 V | 48 V | 54.6 V | 750 W, 36 V | 42 V | 48 V | 54.6 V |
|---|---|---|---|---|---|---|---|---|
| 0 to 1500 | 19.1 | 19.1 | 19.1 | 19.1 | 14.32 | 14.32 | 14.32 | 14.32 |
| 2000 | 14.43 | 16.92 | 19.1 | 19.1 | 10.82 | 12.69 | 14.32 | 14.32 |
| 2500 | 6.22 | 13.47 | 15.28 | 17.59 | 4.62 | 10.09 | 11.46 | 13.19 |
| 3000 | 0 | 4.00 | 12.74 | 14.58 | 0 | 2.97 | 9.54 | 10.93 |
| 3300 | 0 | 0 | 7.00 | 13.24 | 0 | 0 | 5.20 | 9.92 |
| 3600 | 0 | 0 | 0 | 9.59 | 0 | 0 | 0 | 7.16 |
| 3800 | 0 | 0 | 0 | 6.05 | 0 | 0 | 0 | 4.50 |
| 4000 | 0 | 0 | 0 | 1.95 | 0 | 0 | 0 | 1.45 |
| 4200 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

The plant block uses the **peak** envelope (`overtorque_param = no`). The continuous rating is kept by the ESC I²t fold-back (2.9) and by the thermal model, so there are no hidden over-torque states inside the block.

### 2.5 Loss and efficiency model (motor + ESC)
Losses referred to the DC supply and tabulated against (speed, |torque|) at the two temperatures `P.drive.motor.Tmeas1` = 25 °C and `Tmeas2` = 125 °C; the block interpolates on the thermal-port temperature:

```
I      = |T|/Kt                                   % q-axis peak current
P_cu   = 1.5·Rs·(1 + alpha_cu·(T_w − 25))·I²
P_fe   = k_h·|omega_e| + k_e·omega_e²
P_esc  = 3·R_on·(I/√2)² + k_sw·V_nom·(I/√2)
P_loss = P_cu + P_fe + P_esc                      % bearing and seal friction is mechanical: rotor damping lam
```
The side's coefficients are in 6.3 and 6.4: Rs 24 mΩ (750 W: 32 mΩ), k_h 0.025 W/(rad/s) (0.0192), k_e 1.3e-5 W/(rad/s)² (1.0e-5), R_on 2 mΩ, k_sw 1.6e-4 W/(V·A), V_nom 48 V; rotor damping `lam` 1e-4 N·m·s/rad per side (10 W at 3000 rpm).

**Table 2.5. Check values of the loss law per side** (motor + ESC; winding at 25 °C / 120 °C; computed for this revision from the P.drive values).

| Point | 1 kW: loss (W) | 1 kW: efficiency | 750 W: loss (W) | 750 W: efficiency |
|---|---|---|---|---|
| 3000 rpm, rated torque (6.36 / 4.78 N·m) | 182 / 226 | 91.7 / 89.8 % | 136 / 169 | 91.7 / 89.9 % |
| 2122 rpm (1.0 m/s), 3.32 N·m (the T3 point) | 68 / 80 | 91.6 / 90.2 % | 71 / 88 | 91.2 / 89.4 % |
| 1061 rpm (0.5 m/s), half the continuous torque | 46 / 57 | 88.4 / 86.0 % | 35 / 43 | 88.5 / 86.1 % |
| 2000 rpm, peak torque | 1198 / 1601 | 76.9 / 71.4 % | 882 / 1183 | 77.3 / 71.7 % |

Measured (T3, 2026-11-15, 1 kW): 87.1 % from the DC terminals to the motor shaft at 2122 rpm and 3.32 N·m, which includes about 32 W of brake-coil hold, ESC logic and rotor damping. The maps are estimates: T6 waits for the motors' efficiency maps (9, open question 5).
Electrical side, as the block implements it: P_dc = T·omega + P_loss(omega, T, T_w). In regen P_dc goes negative and the current returns to the pack.
ESC logic draw: `P.drive.esc.P_q` = 5 W per side as a fixed resistor, `R_q` = V_nom²/P_q = 460.8 Ω. The brake coils are a variable resistor (2.7). Both are physical, so they stay well-posed when the contactor opens.

### 2.6 Thermal model (per side)
Three-node lumped network of foundation thermal blocks. The side's two motors and ESC channels are one node set (capacities × 2, resistances / 2), and the values of the 3.5 kg motor are scaled with the unit's mass (capacity with the mass, surface with the mass to the power 2/3) [EST]:

```
            P_loss (inside Motor & Drive, node W, C_w)
 [W winding] --R_wh--> [H housing, C_h] --(convection h_h·A_h)--> [AMB]
     |                                \--R_hc (flange to chassis)--> [AMB]
     |--Controlled Heat Flow Rate Source (A = W -> B = E), S = P_esc_est--> [E ESC heat sink, C_esc] --R_esc--> [AMB]
 [AMB] = Controlled Temperature Source, S = T_amb + dT_solar
```

**Table 2.6. The thermal network of a side** (`P.drive.thermal`).

| Element | Leaf | 1 kW | 750 W |
|---|---|---|---|
| Winding capacity (Motor & Drive `thermal_mass`) | `C_w` | 1414 J/K | 1100 J/K |
| Winding to housing | `R_wh` | 0.0972 K/W | 0.125 K/W |
| Housing mass and heat capacity (C_h) | `m_h`, `cp_h` | 6.43 kg, 750 J/(kg·K) (4821 J/K) | 5.0 kg, 750 J/(kg·K) (3750 J/K) |
| Housing convection | `h_h`, `A_h` | 10 W/(m²·K) on 0.166 m² (1.66 W/K) | 10 W/(m²·K) on 0.140 m² (1.40 W/K) |
| Housing to chassis (flange) | `R_hc` | 0.254 K/W | 0.300 K/W |
| ESC heat sink mass and heat capacity (C_esc) | `m_esc`, `cp_esc` | 1.0 kg, 900 J/(kg·K) (900 J/K) | the same |
| ESC heat sink to ambient | `R_esc` | 0.40 K/W | the same |
| Solar load | `dT_solar` | 10 K | the same |
| Initial and default ambient temperature | `T_amb0` | 318.15 K (45 °C) | the same |

* The Motor & Drive thermal variant puts **all** of P_loss into its single port H (node W). The Controlled Heat Flow Rate Source moves the ESC share (P_esc_est, which the ESC model computes from i_q and Vdc by the formula of 2.5) from W to the ESC node. Energy is conserved (T13) and no MATLAB code is needed.
* **Steady state at the rated point** (3000 rpm, rated torque), with the copper-temperature feedback of 2.5 (computed for this revision): winding to ambient 0.276 K/W (750 W: 0.336 K/W). At 45 °C + 10 K solar the motor loss is about 213 W and the winding settles at about 114 °C (750 W: 159 W, 109 °C); at 50 °C + 10 K solar, 120 °C (750 W: 114 °C). Both stay below the 140 °C derating start (`P.drive.esc.T_mot_derate`). These rest on estimated thermal data, and the as-built record estimates only 2.15 N·m continuous per 750 W motor at 45 °C, below its 2.39 N·m nameplate rating. T5 at the right rating is open (R-57, section 9).
* Time constants: winding about 140 s (C_w·R_wh), housing about 860 s (750 W: 790 s).
* ESC heat sink: about 10 W at the rated point (4 K above ambient); 91 W at the peak torque (750 W: 51 W), which would settle 36 K above ambient with a time constant of 360 s. The ESC derates from 85 °C (`P.drive.esc.T_esc_derate`), so its derating matters only for long peaks in 50 °C ambient.

### 2.7 Parking/emergency brakes
* Spring-applied, coil-released (fail-safe), one per motor: **5 N·m static per brake** [EST] (`P.drive.motor_unit.brake_T`), so a side holds `P.drive.brake.T_static` = 10 N·m at the motor shaft and slides at `T_dyn` = 8.4 N·m. Holding at the side's sprockets: 10 × 20 = 200 N·m, with no credit for the gearbox's reverse efficiency.
* **Holding demand and margin.** On 20° with the design mass (158 kg per side) the slope needs 47.7 N·m at the sprocket, 2.39 N·m at the motor: **margin 4.2** (750 W robot, 155 kg per side: 4.3; on the sprayer's 18.5°: 4.5; with no implement on 20°: 5.9).
* **Friction model (D64).** The elasto-plastic Simscape component `shaddad_drive.elastoplastic_brake` (`subsystems/drive/+shaddad_drive`): a bristle (LuGre-type) state holds the shaft as a torsional spring below mu_s·r_eff·N and slides with the Stribeck curve to mu_c·r_eff·N; the torque scales with the engagement N/(N + N_eps), so a released brake transmits nothing. N = F_spring·(1 − f_rel), with F_spring = 500 N per side (250 N per brake), r_eff = 0.05 m, mu_s = 0.40 (10 N·m), mu_c = 0.333, Stribeck velocity w_st = 0.1 rad/s, bristle stiffness k_b = 50 N·m/rad and damping c_b = 0.3 N·m·s/rad per side (25 and 0.15 per brake) [EST]. The former Rotational Friction (AB) changed sign at every 1 ms step when engaged at standstill and, with two brakes per side, diverged (S8a; D64).
* **Actuation** (Simulink, 1 ms): `f_rel` (0 = engaged, 1 = released) follows the ESC brake command through a Rate Limiter: released in `t_release` = 40 ms, engaged in `t_engage` = 60 ms [EST]. The `brake_stuck` fault injection forces f_rel = 0.
* **From a park request to the brakes applied** (T19b, 2026-11-15): DISABLING comes with the next command frame, 10 ms after the request, and the brakes are fully applied after `t_engage` (60 ms): 70 ms after the request in all. With the MCU's 10 ms trip sample, the descent-cap trip's chain is 81 ms, inside REQ-SYS-031's trip reaction time of 0.09 s (G1-01 part 2, step 17; the trip's prediction `t_pred` follows on G2-MCU). Provisional (D96, G1-04): if the delivered ESC needs up to 0.10 s for its share, REQ-SYS-031 becomes 0.12 s.
* **Coils:** 48 V; per brake 25 W pull-in for `t_pullin` = 0.2 s, then 12 W hold (PWM duty `duty_hold` = 0.48). Per side one Variable Resistor across DC+/DC−: R = R_coil/max(duty, `duty_min`), R_coil = 46 Ω (the side's two 92 Ω coils in parallel).
* **Creep** (D64, T8): 2.4e-5 rad/s at the motor, 0.0004 m/h at the track.
* **Stops on the brakes with the STO** (T9, 2026-11-15, 1 kW, 158 kg per side): from the top speed 1.41 m/s, 0.125 m on flat ground and 0.168 m on the 20° descent, with a peak sprocket torque of 156 and 164 N·m (T2NOT of the side 480 N·m). Such a brakes-locked stop decelerates at about 8 m/s²; on a descent it can pitch the robot over its front contact, which the descent cap (D74) and the park criterion (D82) keep in check (REQ-SYS-075).
* **The brake confirmation** in FB1 (`brake_released`, status b4) echoes the ESC's coil command, so a mechanically failed brake is not detected (STATUS 27, R-42; card G2-DRV).

### 2.8 Gearboxes
* `Gear Box` (classic foundation, ideal ratio `P.drive.gear.N` = 20). Port B = motor side, F = sprocket side (omega_B = N·omega_F). The side's two gearboxes are one.
* **Load-dependent mesh loss** (existing blocks, no custom code): an Ideal Torque Sensor measures the output torque T_out, and an Ideal Torque Source between the output node and the case applies
  `T_loss = −k_loss·|T_out|·tanh(omega_out/omega_th)`, with k_loss = (1−eta)/eta = 0.111 and omega_th = 0.5 rad/s.
  This dissipates (1−eta)/eta·|P_out|, which gives eta_fwd = 0.90 and eta_back ≈ 0.90. The PS chain is PS Abs → PS Gain → PS Product ← PS Math Function (tanh) ← PS Gain (1/omega_th). The algebraic loop stays inside the Simscape DAE (well-posed because k_loss < 1).
* **No-load drag** (churning, seals) at the input, per side: `Rotational Friction` (classic) with breakaway 0.08 N·m, Coulomb 0.06 N·m, breakaway velocity 0.5 rad/s and viscous 1.2e-4 N·m·s/rad [EST], about 31 W at 3000 rpm.
* **Output inertia** `J_out` = 0.008 kg·m² per side (0.004 per gearbox: carrier and shaft) as an `Inertia` block. Backlash (15 arcmin) is **not** modelled (risk R7).
* **Ratings per side** (two gearboxes): T2N / T2B / T2NOT = 240 / 360 / 480 N·m (750 W: 240 / 280 / 480) [EST]. D51's row still quotes the ratings of the former single-motor drive; the four-motor ratings (2 × `u.gear_T`) meet its rule, T2B at least the motor peak through the gear, which `shaddad_params` asserts (`T_peak·N·eta ≤ T_acc_out`). With the sprocket turning, the peak motor torque gives 342.6 N·m at the output (T23, 2026-11-15, 1 kW; N·T_peak·eta = 343.8 N·m). At a locked sprocket the smooth loss law has no loss at standstill, so the full N·T_peak = 382 N·m reaches the output: 106 % of T2B and 80 % of T2NOT (T23 information row).

### 2.9 ESC (motor controller) model

**As built: one dual-channel commercial FOC traction controller per side, commanded over CAN; the STM32 runs vehicle-level control only.** The delivered ESCs are the baseline, provisionally (D96). The model below is a behavioural assumption: the ESC's manual, protocol, speed loop and fault reactions are still to come (card G1-04, R-14), and ICD section 4 is provisional.

Why this split (the original design case, which the delivered robot follows):
1. The side's 130 A_rms peak at 48 V (750 W: 100 A_rms) needs dedicated power electronics: a MOSFET bridge, gate drivers, shunt sensing, a DC link, thermal design, EMC.
2. Commercial 48 V traction controllers (CiA-402 / CANopen class) already provide FOC, encoder and Hall interfaces, a brake driver, I²t, thermal derating and STO, with safety documentation.
3. Real-time decoupling: the STM32 keeps the 100 Hz vehicle loops, the SBUS decode and the supervisor with a low CPU load. The FOC timing cannot be disturbed by GNC or communication.
4. Fault containment and field replaceability.

The alternative (the STM32 running FOC with the mcblib `Field-Oriented Current Controller`, which was verified) stays a future option. It needs a custom inverter board, and safety certification gets harder.

**Control mode:** default **speed mode** (velocity setpoint + torque feed-forward + torque limits). It is robust to CAN latency because the ESC closes a 1 kHz speed loop. **Torque mode** is available for future traction-control work.

ESC model loop structure (all at `Ts_esc` = 1 ms):
```
CAN cmd (10 ms, via Rate Transition) -> watchdog -> ESC_SM -> setpoint ramp -> speed PI -> + tau_ff
   -> Saturation Dynamic [lo, up] from the torque limiter -> torque_enable gate -> Tr (to the plant, N·m)
speed meas: theta_m -> Quantizer(2·pi/enc_cpr) -> (θq − θq[k−n_diff])/(n_diff·Ts) -> LPF tau_wf -> w_meas
```
* **Speed PI** (Discrete PID Controller, parallel, PI), D64: **Kp_w = 0.030 N·m/(rad/s), Ki_w = 0.090 N·m/rad** (Ki/Kp = 3 as in D50), designed on the D50 two-mass model: the belt side `P.drive.bench2m.J_b` = 7.8e-4 kg·m² at the motor (750 W: 6.2e-4), the vehicle half-mass through the shear spring `P.drive.bench2m.k_x` (40 kN/m, 61 kN/m and 540 kN/m) with damping tau_d·k_x (tau_d 0.01 s), a 5 ms torque lag, a 2 ms filter and a 2.5 ms delay including the 1 ms command Unit Delay of D36. Rigid crossover `w_c` = Kp_w/J_eq: 8.0 rad/s (750 W: 8.5 rad/s), with `J_eq` 3.76e-3 kg·m² (750 W: 3.54e-3 kg·m²). Four 1 kW units: PM 62°, GM 15.0 dB, Ms 1.37, zeta 0.57; four 750 W units: PM 63°, GM 13.1 dB, Ms 1.46, zeta 0.51 (D64; T21 bounds: Ms ≤ 1.6, zeta ≥ 0.35). Measured in T21 (2026-11-15, 1 kW): Ms 1.10 to 1.37, zeta 0.57 to 0.70, PM 62° to 67°, vehicle overshoot at most 0.06 %. Anti-windup clamping with external limits. **Level reset while the torque is off or not in speed use** (PiReset = NOT(torque_en AND use_speed), `Speed_Ctrl` in `build_drive.m`): in INIT, NOT_READY, READY, FAULT and STO; in PARKED, and in DISABLING and QUICK_STOP, from `t_engage_wait` (80 ms) after the brake engages; and in TORQUE mode outside the hold-zero states. The integrator runs in ENABLING, from zero after a reset (the hill start below). Tr and the brake command pass a 1 ms Unit Delay before the plant.
* **Setpoint ramp** (D33): Rate Limiter Dynamic, ±`a_max` = 497.8 rad/s² normal (2.24 m/s² × k_v2w, above every MCU stop profile, so the MCU shaper is the only limiter) and ±`a_qs` = 444.4 rad/s² in QUICK_STOP (2.0 m/s²).
* **Inertia feed-forward in the hold-zero states** (QUICK_STOP, DISABLING, ENABLING, PARKED, where no MCU feed-forward exists): `J_ff` = J_eq times the ESC's own ramp; without it the stop would lag and overshoot with the low gains (T10).
* **Torque limiter** (upper and lower, quadrant-aware; `esc_torque_limit.m`):
  ```
  tau_env   = 2-D LUT(|n| rpm, Vdc)  (the same T_pk_2D as the plant)
  tau_cont  = 1-D LUT(|n|)           (T_cont_nom), floored at tau_cont_min in the I2t ratio
  x_I2t'    = ((tau/T_cont)² − 1)/k_boost  if |tau| > T_cont,   k_boost = ((T_pk/T_cont)² − 1)·t_boost
              = −(1 − (tau/T_cont)²)/t_recover                otherwise;  x in [0, 1]
  exhausted = Relay(x; on x_on, off x_off);  tau_I2t = exhausted ? tau_cont : tau_env
  f_Tm = LUT(T_motor_meas: T_mot_derate -> [1 0]);  f_Te = LUT(T_esc: T_esc_derate -> [1 0])
  f_uv = LUT(Vdc: V_uv_derate -> [0 1]) (motoring);  f_ov = LUT(Vdc: V_ov_derate -> [1 0]) (regen)
  tau_dcm = I_dc_max_mot·Vdc/max(|w|, w_min_dc);  tau_dcr = I_dc_max_regen·Vdc/max(|w|, w_min_dc)
  tau_mot = min(tau_lim_mot, tau_I2t·f_Tm·f_Te·f_uv, tau_dcm)
  tau_reg = min(tau_lim_reg, tau_I2t·f_Tm·f_Te·f_ov, tau_dcr)
  w >= +w_quad: up = tau_mot, lo = −tau_reg;  w <= −w_quad: up = tau_reg, lo = −tau_mot;  |w| < w_quad: ±tau_mot
  tau_avail = min(tau_I2t·f_Tm·f_Te·f_uv, tau_dcm)   (reported in the feedback: the ESC's own capability, without
              tau_lim_mot, which the MCU sets from tau_avail; a min with it would deadlock: STATUS open problem 1)
  ```
  Values: `t_boost` 30 s and `t_recover` 60 s; the relay is on at `x_on` = 1 − 1e-6 and off at `x_off` = 0.01, so the peak comes back after about 60 s of zero torque (T4); `k_boost` = 240.6 s (750 W: 239.2 s); the derating bands are 140 to 165 °C for the winding, 85 to 100 °C for the ESC, 37 to 40 V for motoring and 57 to 60 V for regen; `w_min_dc` 50 rad/s and `w_quad` 5 rad/s. At the pack's lowest operating voltage, 39.0 V, the motoring torque is two thirds of its limit.
* **Protections** (mcblib `Protection Relay`, debounced): overvoltage 62 V for 5 ms; undervoltage 35 V for 50 ms, latched only when the DC link collapses while the power stage is enabled (armed by `torque_en` of the previous 1 ms sample and held while Vdc < V_on; a dead DC link in INIT, NOT_READY, READY, STO, FAULT or PARKED, for example after an E-stop opened K1/K2, is only NOT_READY, so no fault reset is needed when HV returns: T24); overspeed at the 4000 rpm mechanical limit (`w_overspeed` = 418.9 rad/s, 1.89 m/s) for 10 ms; overcurrent |i_dc| above `I_oc_fault` 143 A (750 W: 104 A), set at 1.3 times the DC motoring limit, for 10 ms. Thresholds via Compare To Constant: motor overtemperature 443.15 K (170 °C), ESC overtemperature 378.15 K (105 °C), stall |tau| > 0.9·T_pk and |w| < 5 rad/s (warning after 5 s, fault after 10 s). CAN time-out: mcblib `Software Watchdog Timer`, reset on an alive-counter change, `n_can_to` = 100 samples at 1 ms (100 ms). Encoder (2 samples) and brake (100 samples) faults come from fault injection.
* **Hill start (open, R-37 and the review's DRV-05).** PARKED turns the torque off, which resets the speed PI, and ENABLING is a hold-zero state where the MCU's feed-forward is ignored. So when the brakes release, 50 to 90 ms into ENABLING, gravity is unopposed until the integrator recharges: about 0.07 to 0.12 m of roll-back on 20° with 158 kg per side (DRV-05's estimate; S2spray's roll-back speed after the release is 0.061 m/s). REQ-SYS-112's roll-back limit is proposed at 0.05 m in G1-01 part 2. The fix (keep the integrator through PARKED, or preload the stored holding torque before the brake releases), with a bench case in `t_drive`, is pending on card G2-MCU with G2-DRV.
* **State machine** (`ESC_SM`, `esc_sm.m`, DS402-like, `state` uint8):

| # | State | Torque | Brake | Exit conditions |
|---|---|---|---|---|
| 0 | INIT | off | engaged | after `t_init` 0.1 s → 1 |
| 1 | NOT_READY | off | engaged | Vdc > V_on (38 V) and sto_ok → 2 |
| 2 | READY | off | engaged | enable and no fault and no CAN timeout and not park_req → 3 |
| 3 | ENABLING | on (holds zero speed) | release after `t_premag` 50 ms | after `t_premag` + `t_release` (40 ms) → 4; park_req → 5 |
| 4 | OPERATION | on | released (substate PARKED: w_ref = 0 and \|w\| < `w_stop` 2 rad/s for `t_park` 2 s gives the brake engaged and the torque off; leaves to ENABLING on a non-zero command) | !enable or park_req → 5; CAN timeout → 6 |
| 5 | DISABLING | on, ramp to 0 | engage at once on park_req (any speed); otherwise when \|w\| < 2 rad/s for `t_stop_hold` 0.2 s, or `t_stop_max` 2.0 s after the stop began (any speed) | 80 ms (`t_engage_wait`) after engage, torque off → 2; enable again → 4 (brake still released) or 3 (brake engaged) |
| 6 | QUICK_STOP | on, ramp at `a_qs` 444.4 rad/s² | engage at standstill, at once on park_req, or `t_stop_max` 2.0 s after the stop began (any speed) | 80 ms after engage → 7 (latched CAN_TIMEOUT) |
| 7 | FAULT | **off immediately** | engaged | fault_reset rising edge and no active fault → 2 |
| 8 | STO | **off immediately** (hardware) | engaged | sto_ok → 1 |

Global priority: STO > hard fault > undervoltage (Vdc < `V_off` 35 V → 1 NOT_READY, from rows 2 to 6; T12) > CAN timeout > operator commands.

* **Fault code** (uint16 bits): 0 OVERCURRENT, 1 OVERVOLTAGE, 2 UNDERVOLTAGE, 3 MOTOR_OVERTEMP, 4 ESC_OVERTEMP, 5 OVERSPEED, 6 CAN_TIMEOUT, 7 ENCODER, 8 BRAKE, 9 STALL, 10 STO_ACTIVE (info).
* **Status word** (uint16 bits): 0 READY, 1 OP_ENABLED, 2 FAULT, 3 DERATING, 4 BRAKE_RELEASED, 5 MODE_SPEED, 6 MODE_TORQUE, 7 TORQUE_LIMIT_ACTIVE, 8 STO_ACTIVE, 9 QUICKSTOP, 10 STALL_WARNING.

### 2.10 CAN protocol (DRIVE-defined; the MCU implements it with FDCAN in classic mode)
500 kbit/s classic CAN1, 11-bit IDs, little-endian, node ID n = 1 (Left) / 2 (Right); both channels of a side's ESC take the same command. The layout is ICD section 4 (provisional, D96); T16 checks the codecs against it byte by byte.

| Frame | ID | Rate | Payload (8 bytes) |
|---|---|---|---|
| CMD (MCU→ESC) | 0x200+n | 100 Hz | u16 ctrl [b0 enable, b1 park_req, b2 fault_reset, b3-4 mode (0 OFF, 1 SPEED, 2 TORQUE), b8-15 alive]; i16 w_ref [1 rpm/bit, motor shaft]; i16 tau_ff [0.01 N·m/bit]; u8 tau_lim_mot [0.1 N·m/bit]; u8 tau_lim_reg [0.1 N·m/bit] |
| FB1 (ESC→MCU) | 0x180+n | 100 Hz | u16 status; i16 speed [1 rpm/bit]; i16 tau_est [0.01 N·m/bit]; i16 i_dc [0.1 A/bit] |
| FB2 | 0x280+n | 20 Hz | u16 v_dc [0.01 V/bit]; i16 i_q [0.1 A/bit]; i16 T_motor [0.1 °C/bit]; u8 T_esc [1 °C/bit, offset −40 °C]; u8 tau_avail [0.1 N·m/bit] |
| FB3 | 0x380+n | 100 Hz | i32 enc_cnt [4096 counts/rev, wraps]; u16 fault_code; u8 alive; u8 state |

ESC time-out: 10 missed CMD frames (100 ms) trigger QUICK_STOP. MCU time-out: FB1 or FB3 older than 50 ms gives `rx_ok = false` (`P.drive.can.t_rx_timeout`, 5 frames) and the class-A fault DRV_L/R_TIMEOUT (`P.electronics.safety.drv_timeout` = 0.05 s, ICD section 6). The u8 torque limits reach 25.5 N·m, above both motors' peak per side. A CANopen heartbeat of the real ESC is not modelled. The drive frames make 640 frames/s, about 15 % of CAN1; the bus load as a whole is in the electronics spec.

### 2.11 Fidelity variants and simulation speed
| Variant (`VC_DRIVE_FID`) | Content | Continuous states/side | Required step | Use |
|---|---|---|---|---|
| **1 SYS (default)** | Motor & Drive (System Level, thermal) + gearbox + brake + thermal | about 8 (rotor, torque lag, brake bristle, sensor angle, 3 thermal, J_out tied by the ratio) | Simscape local solver 1 ms (≤ Tc/3 = 1.7 ms) | Missions of 1 to 8 h, RC and GNC development |
| 2 FOC (bench-only, D52) | PMSM (thermal) + average-value VSC (3-phase) + DC-link C + ee_sl_lib current control at 10 kHz; only in `tests/drive/tb_drive_foc` (its own Solver Configuration), rejected by `shaddad_params` in the system | about 12 | 50 µs plant / 100 µs control | ≤ 60 s validation of the SYS map, current limits, transients |
| 3 IDEAL | Ideal Angular Velocity Source (first-order lag `P.drive.ideal.tau_v` 50 ms, acceleration limit `P.drive.ideal.a_max` 497.8 rad/s²) + DC load P_mech + k_loss·\|P_mech\| (eta 0.80) + `P_aux` 8 W; network tie (D35, D52): Rotational Electromechanical Converter K 1e-3 + 1 MOhm thermal resistor to DC−, and a Controlled Heat Flow Rate Source of the loss into the thermal port H, so the ports equal SYS and no isolated network exists | about 2 | 1 ms | Fast GNC Monte Carlo, the S3 endurance set |

**Simulation speed** (a design estimate of the original build): the drive adds about 16 states and about 80 discrete blocks per side; an 8 h mission at 1 ms is 28.8 M steps, about 15 min wall-clock at 30 µs per system step. **T17** (the 1 h synthetic duty in under 60 s, 60 times real time) stays blocked without a C compiler (D56 item 4): the Normal and JIT Accelerator modes stay below it (resolved by D121, 2026-11-23: MinGW64 14.2.0 is the C compiler, D56 item 4 is done, and accelerator runs work on this PC, so T17 can be run again; it has not been rerun). The FOC variant runs at about 0.2 to 0.5 times real time; T15's 10 s runs take about 80 s.

---

## 3. Block list (verified in the live R2026b session)

Notation: Simscape mask names as read with `get_param(blk,'DialogParameters')`; units go in the `<name>_unit` companions. The builder `scripts/build/build_drive.m` sets every mask from `P.drive` (the leaves of section 6).

### 3.1 Plant: SYS variant (`drive_plant_sys.slx`)
| Block | Library path (verified) | Key mask parameters |
|---|---|---|
| Motor & Drive (System Level) | `ee_lib/Electromechanical/Motor & Drive` + newline + `(System Level)` (use `sprintf('ee_lib/Electromechanical/Motor & Drive\n(System Level)')`) | Variant: `ee.electromech.motor_and_drive_thermal` (component variants verified: `motor_and_drive`, `motor_and_drive_thermal`). `torque_speed_param='ee.enum.electromech.envelope.tabulated2D'`; `T_t_2D=P.drive.motor.T_pk_2D`; `w_t=P.drive.motor.w_t_rpm` (`w_t_unit='rpm'`); `vdc_t=P.drive.motor.vdc_t`; `overtorque_param='ee.enum.electromech.overtorque.no'`; `Tc=P.drive.motor.Tc`; `EnableSwitch='false'`; `losses_param='ee.enum.electromech.motorAndDrive.lossParameterization.TabulatedLossDataAsAFunctionOfSpeedAndTorque'`; `w_eff_vec=P.drive.motor.w_eff_rpm` (`rpm`); `T_eff_vec=P.drive.motor.T_eff`; `losses_mat=P.drive.motor.loss_T1`; `losses_mat_T2=P.drive.motor.loss_T2`; `Tmeas=P.drive.motor.Tmeas1`; `Tmeas2=P.drive.motor.Tmeas2` (degC); `Res=P.drive.motor.Res`; `J=P.drive.motor.J`; `lam=P.drive.motor.lam`; `speed0=0`; `thermal_mass=P.drive.thermal.C_w`; `initial_temperature=P.drive.thermal.T_amb0` |
| Current Sensor | `ee_lib/Sensors & Transducers/Current Sensor` | DC line into the ESC (i_dc, + = from the pack) |
| Voltage Sensor | `ee_lib/Sensors & Transducers/Voltage Sensor` | Vdc at the ESC terminals |
| Capacitor (DC link) and Resistor (its ESR) | `fl_lib/Electrical/Electrical Elements/Capacitor`, `.../Resistor` | `c=P.drive.esc.C_link`, initial voltage `P.drive.esc.vc0`; ESR `R=P.drive.esc.ESR_link` |
| Resistor (logic) | `fl_lib/Electrical/Electrical Elements/Resistor` | `R=P.drive.esc.R_q` (the ESC logic draw `P_q`) |
| Resistor (bleed) | `fl_lib/Electrical/Electrical Elements/Resistor` | `R=P.drive.esc.R_bleed` (keeps the nodes defined with the contactor open) |
| Variable Resistor | `fl_lib/Electrical/Electrical Elements/Variable Resistor` | `Rmin=1`; PS input R = `P.drive.brake.R_coil`/max(duty, `P.drive.brake.duty_min`) (brake coils) |
| Ideal Rotational Motion Sensor (rotor) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor` | `reference='foundation.enum.MeasurementReference.difference'`; `velocity_port='true'`; `position_port='true'`; `wrap_angle='false'` |
| Ideal Torque Sensor (motor shaft) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor` | (none) |
| Mechanical Rotational Reference | `fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference` | motor case, gearbox case, brake reaction |
| Simulink-PS Converter (Tr, R_coil, P_esc, T_amb) | `nesl_utility/Simulink-PS Converter` | `Unit` = `N*m`, `Ohm`, `W`, `K`; `FilteringAndDerivatives='zero'` (inputs piecewise constant at 1 ms, so no extra states) |
| PS-Simulink Converter (about 10) | `nesl_utility/PS-Simulink Converter` | `Unit` = `rad/s`, `rad`, `N*m`, `A`, `V`, `K` |

### 3.2 Gearbox (`drive_gearbox.slx`)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| Gear Box | `fl_lib/Mechanical/Mechanisms/Gear Box` | `ratio=P.drive.gear.N` |
| Rotational Friction (no-load drag, input side) | `fl_lib/Mechanical/Rotational Elements/Rotational Friction` | `brkwy_trq=P.drive.gear.T_nl_brk`, `Col_trq=P.drive.gear.T_nl_C`, `brkwy_vel=P.drive.gear.w_nl_brk`, `visc_coef=P.drive.gear.c_nl` |
| Inertia (output side) | `fl_lib/Mechanical/Rotational Elements/Inertia` | `inertia=P.drive.gear.J_out`, `num_ports='foundation.enum.numPorts2.one'` |
| Ideal Torque Sensor (output) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor` | T_out to the loss law and the truth |
| Ideal Rotational Motion Sensor (output) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor` | velocity + position (sprocket truth) |
| Ideal Torque Source (mesh loss) | `fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source` | between the output node and the case |
| PS Abs | `fl_lib/Physical Signals/Nonlinear Operators/PS Abs` | |
| PS Gain (× 3) | `fl_lib/Physical Signals/Functions/PS Gain` | `P.drive.gear.k_loss`, 1/`P.drive.gear.w_th`, −1 |
| PS Math Function | `fl_lib/Physical Signals/Functions/PS Math Function` | `fcn_choice='foundation.enum.math_function.tanh'` (tanh in the enum, verified) |
| PS Product | `fl_lib/Physical Signals/Functions/PS Product` | |

### 3.3 Brake (`drive_brake.slx`)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| Simscape Component (D64) | `nesl_utility/Simscape Component`, component `shaddad_drive.elastoplastic_brake` (`subsystems/drive/+shaddad_drive`) | `r_eff`, `mu_s`, `mu_c`, `w_st`, `k_b`, `c_b`, `w_d`, `T_eps`, `w_eps`, `w_r`, `N_eps` = `P.drive.brake.*`; housing = `Mechanical Rotational Reference`. It replaces the four AB blocks below, which stay documented for the fallback (R3). |
| Rotational Friction (AB) | `fl_lib/Rotational/Elements/Rotational Friction (AB)` | (former model) `normal_force_spec='foundation.enum.constant_variable.variable'`; `radius=P.drive.brake.r_eff`; `brkwy_coef=P.drive.brake.mu_s`; `Col_coef=P.drive.brake.mu_c`; `visc_coef=0` |
| Interface (AB-Rotational) | `fl_lib/Rotational/Utilities/Interface (AB-Rotational)` | (former model) `angle_based_port='foundation.enum.RotationalInterfacePort.Bab'` (default) |
| Rotational World (AB) | `fl_lib/Rotational/Elements/Rotational World (AB)` | (former model) brake housing = chassis |
| Torque Sensor (AB) | `fl_lib/Rotational/Sensors/Torque Sensor (AB)` | (former model) brake torque truth |
| Simulink-PS Converter | `nesl_utility/Simulink-PS Converter` | `Unit='N'` (normal force) |
| Rate Limiter (actuator) | `simulink/Discontinuities/Rate Limiter` | `RisingSlewLimit=1/P.drive.brake.t_release`, `FallingSlewLimit=-1/P.drive.brake.t_engage`, `InitialCondition=0` |
| Gain / Sum / Constant | `simulink/Math Operations/Gain`, `.../Sum`, `simulink/Sources/Constant` | N = F_spring·(1 − f_rel) |

### 3.4 Thermal (`drive_thermal.slx`)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| Thermal Resistance (× 3) | `fl_lib/Thermal/Thermal Elements/Thermal Resistance` | `resistance` = `P.drive.thermal.R_wh`, `R_hc`, `R_esc` |
| Thermal Mass (× 2) | `fl_lib/Thermal/Thermal Elements/Thermal Mass` | housing `mass=P.drive.thermal.m_h`, `sp_heat=P.drive.thermal.cp_h`; ESC `mass=P.drive.thermal.m_esc`, `sp_heat=P.drive.thermal.cp_esc`; initial `T=P.drive.thermal.T_amb0` |
| Convective Heat Transfer | `fl_lib/Thermal/Thermal Elements/Convective Heat Transfer` | `area=P.drive.thermal.A_h`, `heat_tr_coeff=P.drive.thermal.h_h` |
| Controlled Heat Flow Rate Source | `fl_lib/Thermal/Thermal Sources/Controlled Heat Flow Rate Source` | A = winding node, B = ESC node, S = P_esc_est |
| Controlled Temperature Source | `fl_lib/Thermal/Thermal Sources/Controlled Temperature Source` | S = T_amb + `P.drive.thermal.dT_solar` |
| Temperature Sensor (× 3) | `fl_lib/Thermal/Thermal Sensors/Temperature Sensor` | `temperature_measure` default (difference to the Thermal Reference) |
| Thermal Reference | `fl_lib/Thermal/Thermal Elements/Thermal Reference` | |

### 3.5 ESC controller (`drive_esc.slx`, discrete 1 ms)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| MATLAB Function (ESC_SM) | `simulink/User-Defined Functions/MATLAB Function` | `esc_sm.m` (D1: Stateflow is not installed); timers counted in ticks of `P.drive.Ts_esc`, from `P.drive.sm_vec`; states in 2.9 |
| Discrete PID Controller (speed PI) | `simulink/Discrete/Discrete PID Controller` | `Controller='PI'`, `Form='Parallel'`, `P=P.drive.esc.Kp_w`, `I=P.drive.esc.Ki_w`, `SampleTime=P.drive.Ts_esc`, `IntegratorMethod='Backward Euler'`, `LimitOutput='on'`, `SatLimitsSource='external'`, `AntiWindupMode='clamping'`, `ExternalReset='level'`, `InitialConditionForIntegrator=0` |
| Rate Limiter Dynamic | `simulink/Discontinuities/Rate Limiter Dynamic` | up/lo = ±`P.drive.esc.a_max` or ±`P.drive.esc.a_qs` |
| Saturation Dynamic | `simulink/Discontinuities/Saturation Dynamic` | final torque clamp (torque mode and the feed-forward path) |
| 2-D Lookup Table | `simulink/Lookup Tables/2-D Lookup Table` | `Table=P.drive.motor.T_pk_2D`, `BreakpointsForDimension1=P.drive.motor.w_t_rpm`, `BreakpointsForDimension2=P.drive.motor.vdc_t`, `ExtrapMethod='Clip'` |
| 1-D Lookup Table (× 5) | `simulink/Lookup Tables/1-D Lookup Table` | T_cont envelope, f_Tm, f_Te, f_uv, f_ov (`ExtrapMethod='Clip'`) |
| Discrete-Time Integrator (I²t) | `simulink/Discrete/Discrete-Time Integrator` | `gainval=1`, `LimitOutput='on'`, `UpperSaturationLimit=1`, `LowerSaturationLimit=0`, `SampleTime=P.drive.Ts_esc` |
| Relay (boost exhausted) | `simulink/Discontinuities/Relay` | `OnSwitchValue=P.drive.esc.x_on`, `OffSwitchValue=P.drive.esc.x_off`, `OnOutputValue=1`, `OffOutputValue=0` |
| Quantizer (encoder) | `simulink/Discontinuities/Quantizer` | `QuantizationInterval=2*pi/P.drive.esc.enc_cpr`, `SampleTime=P.drive.Ts_esc` |
| Delay | `simulink/Discrete/Delay` | `DelayLength=P.drive.esc.n_diff` |
| Discrete Transfer Fcn (speed LPF, torque estimate, temperature-sensor lag) | `simulink/Discrete/Discrete Transfer Fcn` | `Numerator=[1-a]`, `Denominator=[1 -a]`, a = exp(−Ts/tau) with tau = `tau_wf`, `Tc` and `tau_Tsens`: `P.drive.esc.a_wf`, `a_Tc`, `a_Ts` |
| Math Function (mod, counter wrap) | `simulink/Math Operations/Math Function` | `mod` for the int32 wrap of enc_cnt |
| Protection Relay (× 4) | `mcblib/Protection and Diagnostics/Protection Relay` | `Protection` in {Overcurrent, Overspeed, Overvoltage, Undervoltage} (enum verified), `CountMax`, `SamplesForDebounce` (`n_oc`, `n_os`, `n_ov`, `n_uv`) |
| Software Watchdog Timer | `mcblib/Protection and Diagnostics/Software Watchdog Timer` | `maxCount=P.drive.esc.n_can_to`, reset by Detect Change(alive) |
| Detect Change | `simulink/Logic and Bit Operations/Detect Change` | alive counter |
| Compare To Constant | `simulink/Logic and Bit Operations/Compare To Constant` | temperature and stall thresholds |
| Bit Set / Bitwise Operator / Shift Arithmetic / Extract Bits | `simulink/Logic and Bit Operations/...` (all four verified) | fault word (Protections); the status word is in `esc_feedback.m` (D70, section 4) |
| MinMax, Abs, Product, Divide, Switch, Multiport Switch, Logical Operator, Data Type Conversion | standard `simulink/...` (verified) | protections, speed loop and codecs; the torque limiter's arithmetic between its Lookup Tables, I²t integrator and Relay is in `esc_torque_limit.m` (D70, section 4) |
| MATLAB Function (× 2, D70) | `simulink/User-Defined Functions/MATLAB Function` | `Torque_Limiter/LimitLogic` → `esc_torque_limit.m`, `Feedback/FbPack` → `esc_feedback.m` (section 4) |
| Counter Limited | `simulink/Sources/Counter Limited` | ESC alive tick, `uplimit` = 256·`n_alive` − 1 (into `FbPack`) |
| Rate Transition (in/out) | `simulink/Signal Attributes/Rate Transition` | `Integrity='on'`, `Deterministic='on'`, `OutPortSampleTime` = `P.drive.Ts_esc` (in) / `P.drive.Ts_can` (out) |

### 3.6 CAN codecs (`drive_cmd_pack`, `drive_cmd_unpack`, `drive_fb_pack`, `drive_fb_unpack`)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| Byte Pack | `mcblib/Signal Management/Byte Pack` (links to `svdutilitieslib/Byte Pack`) | `maskPackedDataType='uint8'`, `datatypes` per frame (2.10), `byteAlign=1` |
| Byte Unpack | `mcblib/Signal Management/Byte Unpack` | `dimensions` and `datatypes` per frame (2.10), `byteAlign=1` |
| Quantizer and Data Type Conversion | `simulink/Discontinuities/Quantizer`, `simulink/Signal Attributes/Data Type Conversion` | the protocol resolutions `P.drive.can.res_w` (1 rpm), `res_tau` (0.01 N·m), `res_tau_lim` (0.1 N·m), `res_i` (0.1 A), `res_v` (0.01 V), `res_T` (0.1 K); the u8 T_esc offset `P.drive.can.T_esc_ofs` |
| Frame age and rx_ok (`drive_fb_unpack`) | standard `simulink/...` | invalid frames hold the last payload; `rx_ok` false after `P.drive.can.n_rx_timeout` frames |
| (MCU HW variant, owned by MCU) FDCAN Write / FDCAN Read | `stm32blockslib/FDCAN Write`, `stm32blockslib/FDCAN Read` | `FDCANModule='FDCAN1'`, `CANFrameType='CAN'` (enum {CAN, CAN-FD} verified), `IdentifierType='Standard (11-bit identifier)'`, `IdentifierVia='Dialog'`, `Identifier=0x201/0x202`, `DataLengthVia='Dialog'`, `DataLength=P.drive.can.len`, `DataFormatWriteMode='Raw data'`, `SampleTime=P.drive.Ts_can` |

The CAN1 network model (latency, frame drops, the bus between the codecs) is `CAN1_Net_Sim` (ELECTRONICS).

### 3.7 FOC variant (`drive_plant_foc.slx`, bench-only)
| Block | Library path (verified) | Key parameters |
|---|---|---|
| PMSM | `ee_lib/Electromechanical/Permanent Magnet/PMSM` | variant `ee.electromech.pmsm.pm_rotor.pmsm.abc_thermal` (verified). `nPolePairs=P.drive.motor.p`, flux linkage `pm_flux_linkage=P.drive.motor.lambda_pm`, `stator_param` LdLqL0 with `Ld`, `Lq`, `L0` = `P.drive.motor.Ld`, `Lq`, `L0`, `Rs=P.drive.motor.Rs`, empirical iron loss `losses_oc=P.drive.foc.losses_oc` at `f_losses=P.drive.foc.f_losses`, `J=P.drive.motor.J`, `lam=P.drive.motor.lam`, `alpha=P.drive.motor.alpha_cu`, thermal masses `P.drive.foc.C_stator` and `C_rotor` |
| Average-Value Voltage Source Converter (Three-Phase) | `ee_lib/Semiconductors & Converters/Converters/Average-Value Voltage Source Converter (Three-Phase)` | variant `abc_thermal`; `input_option` = ModWave, `frequency_option` = variable, loss profile `Ploss_vect=P.drive.foc.Ploss_vect`, `Irms_vect=P.drive.foc.Irms_vect` (the P_esc law of 2.5), `Vdc_nom=P.drive.motor.V_nom`, `thermal_mass=P.drive.foc.thermal_mass_inv` |
| Capacitor (DC link) | `fl_lib/Electrical/Electrical Elements/Capacitor` | `P.drive.foc.C_link` + ESR |
| PMSM Current Reference Generator | `ee_sl_lib/PMSM Control/PMSM Current Reference Generator` | `RefType='Zero d-axis control'`, `Vnom=P.drive.motor.V_nom`, `Pmax=P.drive.foc.P_max`, `Tmax=P.drive.motor.T_peak`, `p`, `psim=P.drive.motor.lambda_pm`, `Ld`, `Lq`, `Rs`, `Ts=P.drive.Ts_foc`. Ports TqRef, wMechanical, Vdc → idqRef, TqRefSat, TqLim |
| PMSM Current Controller | `ee_sl_lib/PMSM Control/PMSM Current Controller` | `Kp_id=Kp_iq=P.drive.foc.Kp_i`, `Ki_id=Ki_iq=P.drive.foc.Ki_i` (pole-zero cancellation at a 500 Hz current bandwidth: Kp = Lq·2π·500, Ki = Rs·2π·500), `Ts=P.drive.Ts_foc`. Ports idqRef, idq, vdqFF, VphMax, Reset → vdqRef |
| Park / Inverse Park Transform | `ee_sl_lib/Mathematical Transforms/Park Transform`, `.../Inverse Park Transform` | theta_e = p·theta_m |
| Current Sensor (Three-Phase) | `ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)` | |
| PMSM Torque Estimator | `ee_sl_lib/PMSM Control/PMSM Torque Estimator` | for the tau_est feedback |

### 3.8 Shared infrastructure
| Block | Path (verified) | Notes |
|---|---|---|
| Variant Subsystem (plant fidelity) | `simulink/Ports & Subsystems/Variant Subsystem` | `VariantControlMode='expression'`, choices `VC_DRIVE_FID==1/2/3`, `VariantActivationTime='update diagram'` (needed with physical ports) |
| Subsystem Reference | `simulink/Ports & Subsystems/Subsystem Reference` | `ReferencedSubsystem='drive_side'` and so on |
| Bus Creator / Bus Selector | `simulink/Signal Routing/Bus Creator`, `.../Bus Selector` | `OutDataTypeStr='Bus: Bus_...'` |
| Solver Configuration | `nesl_utility/Solver Configuration` | **Not inside DRIVE.** Exactly one per network, owned by the system integrator. The benches use `UseLocalSolver='on'`, `LocalSolverChoice='NE_BACKWARD_EULER_ADVANCER'` (enum verified; `NE_PARTITIONING_ADVANCER` as an option), `LocalSolverSampleTime=1e-3`, `DoFixedCost='on'`, `MaxNonlinIter=3` |
| Bench only: DC Voltage Source, Resistor, Electrical Reference, Inertia, Ideal Torque Source, Ideal Angular Velocity Source | `fl_lib/Electrical/Electrical Sources/DC Voltage Source`, `fl_lib/Electrical/Electrical Elements/Electrical Reference`, `fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source` (verified) | pack and vehicle equivalents (section 8) |

---

## 4. MATLAB Function / MATLAB System blocks

**Three MATLAB Functions, all in `drive_esc`** (register: architecture section 11):
* `ESC_SM` calls `subsystems/drive/esc_sm.m`, the DS402 state machine (D1: Stateflow is not installed).
* D70 (MATLAB Function plan Task 3, 2026-11-14): two logic-only block networks became one MATLAB Function each,
  bit-identical to the networks they replace (`tests/integration/t_mf_equiv`, `testDriveEscEquivalent`; the
  system compare S0, S1, S8a, R1, S2, S6c against `work/layout/runs/pre_mf`):
  * `Torque_Limiter/LimitLogic` → `subsystems/drive/esc_torque_limit.m`: the I²t rate, the thermal, voltage and
    DC-current limits, the quadrant selection, `tau_avail` and the DERATING flag of 2.9. The Lookup Tables (envelope
    `EnvPeak`, continuous rating `EnvCont`, the derating factors `f_Tm`, `f_Te`, `f_uv`, `f_ov`), the I²t
    Discrete-Time Integrator and its Relay `Exhausted` stay blocks around it, and so do `AbsW` and `ToRpm` (the
    speed in rpm in front of the tables: computed inside the function, it would close an algebraic loop through
    them). The integrator (Forward Euler) has no direct feedthrough, so `LimitLogic` → `I2t` → `Exhausted` →
    `LimitLogic` is no algebraic loop.
  * `Feedback/FbPack` → `subsystems/drive/esc_feedback.m`: the status word, the alive counter and
    `Bus_DriveSideFb`. The alive tick `Tick` (Counter Limited, a state) stays a block in front of it.
  * Each block holds a verbatim copy of its file, and its constants enter from a Constant block (`TlPar`, `FbPar`):
    both against the R2026b JIT relink assertion `'o != nullptr'` three Subsystem Reference levels deep (see
    `MFB` in `scripts/build/build_drive.m`). The file stays the single source: after editing it, rebuild with
    `build_drive({'esc'})` (`t_mf_equiv/testDriveEscUsesFunctions` checks that the copies are current).
  * Each cast is that of the Data Type Conversion it replaces (Nearest, saturating; for these unsigned and float
    targets that is MATLAB's own cast). The MinMax blocks are `local_min`: like MATLAB's `min` they skip NaN, but a
    tie of +0 and −0 gives the later input (measured, R2026b). The Saturations are comparisons that pass NaN, as the
    blocks do.

Every other runtime function maps onto an existing block:

| Need | Existing-block solution |
|---|---|
| Bit-level CAN frame packing and unpacking | `Byte Pack` / `Byte Unpack` (mcblib) + Shift Arithmetic / Bitwise Operator / Extract Bits |
| ESC state machine | MATLAB Function `ESC_SM` (D1). Fallback: Multiport Switch + Unit Delay "state register" + Compare To Constant (all verified). |
| Gear load-dependent loss | PS blocks + Ideal Torque Source and Sensor (2.8) |
| ESC heat split | Controlled Heat Flow Rate Source (2.6) |
| I²t, derating, voltage and current limits | Discrete-Time Integrator, Relay, 1-D and 2-D LUT; the arithmetic between them in `esc_torque_limit.m` (D70) |
| Encoder count wrap, speed estimation | Quantizer, Math Function (mod), Delay, Discrete Transfer Fcn |

Offline parameter generation (MATLAB functions, **not blocks**), as local functions in `params/param_drive.m`:
```matlab
function D = param_drive(r_spr, option)     % r_spr = P.structure.r_sprocket; option = motor of param_motor_unit
u = param_motor_unit(option);               % one unit ('kw1' default | 'movematic750'); side = n_mot_side units
...
D.motor.T_pk_2D = local_envelope2D(D.motor);
[D.motor.loss_T1, D.motor.loss_T2] = local_lossmap(D.motor, D.esc);
end

function T2D = local_envelope2D(m)
  wt = m.w_t_rpm; T2D = zeros(numel(wt), numel(m.vdc_t));
  for j = 1:numel(m.vdc_t)
     T2D(:,j) = interp1(wt, m.T_pk_nom, min(wt*m.V_nom/m.vdc_t(j), wt(end)), 'linear', 0);
  end
  T2D(wt > m.n_mech_max, :) = 0;
end

function [L1, L2] = local_lossmap(m, esc)
  we = m.p*m.w_eff_rpm(:)*pi/30;  [WE, TT] = ndgrid(we, m.T_eff(:)');
  I = TT/m.Kt;  Irms = I/sqrt(2);
  Pfe  = m.k_h*abs(WE) + m.k_e*WE.^2;
  Pesc = 3*esc.R_on*Irms.^2 + esc.k_sw*m.V_nom*Irms;
  Pcu  = @(Tc) 1.5*m.Rs*(1 + m.alpha_cu*(Tc - 25)).*I.^2;
  L1 = Pcu(m.Tmeas1) + Pfe + Pesc;   L2 = Pcu(m.Tmeas2) + Pfe + Pesc;   % [numel(w) x numel(T)] W
end
```

---

## 5. Interfaces

### 5.1 Signal inputs: the root inports of `drive_system.slx` (ICD 3.2, the Drive row)
| # | Port | Producer | Type | Ts | Notes |
|---|---|---|---|---|---|
| 1 | `can_rx` | MCU, through `CAN1_Net_Sim` | `Bus_Can1_McuFast`, the DRV_CMD frames (D6) | `Ts.can_drive`, 10 ms | decoded inside `drive_system` by `drive_cmd_unpack` into `Bus_DriveCmd` (L and R sub-buses, 5.4) |
| 2 | `sto_ok` | POWER: `Safety_Chain`, its output Safety_OK (D8) | boolean | `Ts.base`, 1 ms | false = Safe Torque Off on both ESCs (hardwired, bypasses CAN) |
| 3 | `T_amb` | STRUCTURE: `env.T_amb`, the world's ambient | double, K | `Ts.hk`, 100 ms | ambient air temperature; a Rate Transition takes it to the 1 ms rate |
| 4 | `fault_inj` | the plant input (mapped from `Bus_FaultInject`) or a bench | `Bus_DriveFaultInj` | `Ts.base`, 1 ms | default constant struct `P.drive.fault_inj_default` |

### 5.2 Signal outputs: the root outports
| # | Port | Consumer | Type | Ts | Notes |
|---|---|---|---|---|---|
| 1 | `can_tx_fast` | MCU, through `CAN1_Net_Sim` | `Bus_Can1_EscFast`, the FB1 and FB3 frames | `Ts.can_drive`, 10 ms | packed inside `drive_system` by `drive_fb_pack` from `Bus_DriveFb` (single and integer types); the MCU decodes them with `drive_fb_unpack` |
| 2 | `can_tx_slow` | MCU, through `CAN1_Net_Sim` | `Bus_Can1_EscSlow`, the FB2 frames | `Ts.can_fb2`, 50 ms | the same |
| 3 | `truth` | SENSORS, logging, `Sim_Monitors`, POWER and STRUCTURE checks | `Bus_DriveTruth` (double) | `Ts.base`, 1 ms | |

### 5.3 Physical (Simscape) ports
| Port | Domain | Connects to | Convention |
|---|---|---|---|
| `DC48_TracL_p`, `DC48_TracL_n` | electrical (foundation.electrical) | POWER: the fused 48 V feed of the left ESC (125 A fuse per side, `P.power.pdu.F_TL`) | i > 0 drawn from the pack into `DC48_TracL_p` |
| `DC48_TracR_p`, `DC48_TracR_n` | electrical | POWER: the right ESC feed | i > 0 drawn from the pack into `DC48_TracR_p` |
| `Spr_L` | mechanical rotational (**classic `foundation.mechanical.rotational`**) | STRUCTURE: the left drive sprockets | positive omega = forward track motion on that side. The torque is the gearboxes' output. The case reaction goes to an internal Mechanical Rotational Reference. |
| `Spr_R` | mechanical rotational (classic) | STRUCTURE: the right drive sprockets | the same (the mirrored mounting is handled by the ESC polarity, so no sign flip in the model) |
| (optional) `H_amb` | thermal | a world thermal node, if one exists | default: the internal Controlled Temperature Source driven by `T_amb` |

DRIVE contains **no Solver Configuration and no Electrical Reference**. Both belong to the integrator and POWER. A stand-alone bench adds them.

### 5.4 Bus definitions (`buses/buses_drive.m`, owned by INTEGRATION)

**Bus_DriveSideCmd** (MCU to ESC, Ts 0.01)
| Element | Type | Dim | Units | Description |
|---|---|---|---|---|
| enable | boolean | 1 | – | request OPERATION_ENABLED |
| mode | uint8 | 1 | – | 0 OFF, 1 SPEED (default), 2 TORQUE |
| w_ref | single | 1 | rad/s | motor-shaft speed setpoint (= `P.drive.gear.k_v2w` · v_track) |
| tau_ff | single | 1 | N·m | torque feed-forward (SPEED) or setpoint (TORQUE), motor shaft |
| tau_lim_mot | single | 1 | N·m | ≥ 0, motoring torque limit (magnitude) from the MCU limiter (BMS DCL) |
| tau_lim_reg | single | 1 | N·m | ≥ 0, regenerative torque limit (magnitude) from the MCU limiter (BMS CCL) |
| park_req | boolean | 1 | – | force the brake engaged and the torque off |
| fault_reset | boolean | 1 | – | rising edge clears the latched faults |
| alive | uint8 | 1 | – | rolling counter (+1 per frame); frozen 100 ms gives QUICK_STOP |

The design ranges (Min, Max) of `w_ref`, `tau_ff`, `tau_lim_mot` and `tau_lim_reg` in `buses_drive.m` are still those of the former single-motor drive (±502.7 rad/s and 16 N·m), below the 1 kW side's 19.1 N·m peak: a proposal for INTEGRATION (section 9).

**Bus_DriveCmd**: `L : Bus_DriveSideCmd`, `R : Bus_DriveSideCmd`.

**Bus_DriveSideFb** (ESC to MCU, Ts 0.01)
| Element | Type | Dim | Units | Description |
|---|---|---|---|---|
| state | uint8 | 1 | – | 0 INIT ... 8 STO (2.9) |
| status | uint16 | 1 | – | bit field (2.9) |
| fault_code | uint16 | 1 | – | bit field (2.9) |
| w_m | single | 1 | rad/s | measured motor speed (1 rpm resolution) |
| enc_cnt | int32 | 1 | counts | motor encoder, 4096 per revolution, wraps; the MCU uses a wrap-safe difference |
| tau_est | single | 1 | N·m | estimated motor torque |
| tau_avail | single | 1 | N·m | available motoring torque after all derating |
| i_q | single | 1 | A | q-axis current (peak) |
| i_dc | single | 1 | A | DC input current (+ = discharge) |
| v_dc | single | 1 | V | DC-link voltage |
| T_motor | single | 1 | K | winding sensor (5 s lag) |
| T_esc | single | 1 | K | heat-sink temperature |
| brake_released | boolean | 1 | – | the brake state of the ESC (echoes its coil command, 2.7) |
| alive | uint8 | 1 | – | ESC rolling counter |
| rx_ok | boolean | 1 | – | FB1 and FB3 fresh (< 50 ms), set by the MCU decoder |

**Bus_DriveFb**: `L : Bus_DriveSideFb`, `R : Bus_DriveSideFb`.

**Bus_DriveSideTruth** (plant truth, double, Ts 1e-3)
| Element | Units | Element | Units |
|---|---|---|---|
| w_m | rad/s | theta_m | rad (unwrapped) |
| w_spr | rad/s | theta_spr | rad (unwrapped) |
| tau_m | N·m (motor shaft) | tau_spr | N·m (gearbox output) |
| tau_brake | N·m | f_brake_rel | – (0 engaged ... 1 released) |
| v_dc | V | i_dc | A |
| p_dc | W | p_spr | W (sprocket mechanical) |
| P_loss | W (motor + ESC) | T_winding | K |
| T_housing | K | T_esc | K |
| E_dc | J (∫p_dc dt) | E_spr | J (∫p_spr dt) |
| E_loss | J (∫(p_dc − p_spr) dt: motor, ESC, gearbox, brake and coil losses) | | |

**Bus_DriveTruth**: `L`, `R` : `Bus_DriveSideTruth`.

**Bus_DriveSideFaultInj**: `can_drop` boolean (reserved: the frame drops are injected in `CAN1_Net_Sim`; keep false), `enc_fail` boolean, `brake_stuck` boolean, `tau_scale` double (1 = nominal, applied to Tr; 0 with enable = a drive trip), `T_offset` double (K, added to the winding temperature sensor). **Bus_DriveFaultInj**: `L`, `R`.

---

## 6. Parameters (`params/param_drive.m` returns `D`, assembled as `P.drive`)

The unit data are in `params/param_motor_unit.m` (`u`, kept as `P.drive.motor_unit`); `param_drive` builds the side from `n_mot_side` = 2 units. Where the two motors differ, the 750 W column gives the value of `shaddad_params('Motor','movematic750')`.

### 6.1 Rates and variants (`param_drive.m`:22-24, 165)
| Field | Value | Units | Justification |
|---|---|---|---|
| Ts_esc | 1e-3 | s | ESC speed-loop rate (real ESCs: 1 to 4 kHz) |
| Ts_can | 1e-2 | s | 100 Hz command and feedback |
| Ts_can_slow | 5e-2 | s | FB2, 20 Hz |
| Ts_plant (recommended local solver step) | 1e-3 | s | ≤ Tc/3 |
| Ts_foc / Ts_foc_plant | 1e-4 / 5e-5 | s | FOC variant (bench-only) |
| fidelity (→ `VC_DRIVE_FID`) | 1 | – | 1 SYS, 2 FOC, 3 IDEAL |
| fid_system | [1 3] | – | the values allowed in `Shaddad_Plant` (2 FOC is bench-only, D52) |
| n_mot_side | 2 | – | motors per side (front and rear sprocket) |

### 6.2 Kinematics and gearbox (`D.gear`, `param_drive.m`:28-35)
| Field | 1 kW (default) | 750 W | Units | Source |
|---|---|---|---|---|
| N | 20 | 20 | – | [EST] 1.41 m/s at 3000 rpm with the 0.090 m sprocket (2.2) |
| eta | 0.90 | 0.90 | – | [EST] 2-stage planetary |
| k_loss | 0.111 | 0.111 | – | (1−eta)/eta |
| w_th | 0.5 | 0.5 | rad/s | tanh smoothing (sprocket side) |
| T_nl_brk / T_nl_C | 0.08 / 0.06 | 0.08 / 0.06 | N·m | [EST] no-load drag of the side, motor side |
| w_nl_brk | 0.5 | 0.5 | rad/s | |
| c_nl | 1.2e-4 | 1.2e-4 | N·m·s/rad | [EST] |
| J_out | 0.008 | 0.008 | kg·m² | carrier + output shaft, 0.004 per gearbox |
| T_rated_out / T_acc_out / T_estop_out | 240 / 360 / 480 | 240 / 280 / 480 | N·m | [EST] T2N / T2B / T2NOT of the side, 2 × `u.gear_T` (D51's rule, 2.8) |
| backlash_arcmin | 15 | 15 | arcmin | information only (not modelled) |
| mass | 7.0 | 6.0 | kg | [EST] 2 × `u.gear_mass` |
| r_spr | 0.090 | 0.090 | m | = `P.structure.r_sprocket` [EST] |
| k_v2w | 222.2 | 222.2 | (rad/s)/(m/s) | N/r_spr, used by the MCU |

### 6.3 Motor of a side (`D.motor`, `param_drive.m`:40-54) and the unit (`u`, `param_motor_unit.m`:18-47)
| Field | 1 kW (default) | 750 W | Units | Source |
|---|---|---|---|---|
| u.P_rated | 1000 | 750 | W | 1 kW [EST], 750 W [DATA] nameplate |
| P_rated (side) | 2000 | 1500 | W | 2 × unit |
| n_rated | 3000 | 3000 | rpm | [DATA] |
| u.T_cont / u.T_peak | 3.18 / 9.55 | 2.39 / 7.16 | N·m | rated torque at 3000 rpm; peak 3 × rated [EST] |
| T_cont / T_peak (side) | 6.36 / 19.1 | 4.78 / 14.32 | N·m | 2 × unit; the peak for t_boost 30 s |
| P_peak_int (side) | 4000 | 3000 | W | 2 × the unit's peak power (2.0 and 1.5 kW) |
| n_max / n_mech_max | 3600 / 4000 | 3600 / 4000 | rpm | [EST] no-load at 48 V; mechanical limit and overspeed trip |
| p | 4 | 4 | – | 8 poles [EST] |
| lambda_pm | 0.0184 | 0.0184 | Wb | [EST] no-load 3600 rpm at 48 V |
| Kt | 0.1104 | 0.1104 | N·m/A_pk | 1.5·p·lambda |
| u.Rs / Rs (side) | 48e-3 / 24e-3 | 64e-3 / 32e-3 | Ω | [EST] phase, 25 °C; side = unit/2 |
| u.L, u.L0 / Ld = Lq, L0 (side) | 190e-6, 90e-6 / 95e-6, 45e-6 | 250e-6, 120e-6 / 125e-6, 60e-6 | H | [EST] SPM, 48 V class; side = unit/2 |
| alpha_cu | 3.93e-3 | 3.93e-3 | 1/K | copper |
| u.k_h, u.k_e / k_h, k_e (side) | 0.0125, 6.5e-6 / 0.025, 1.3e-5 | 0.0096, 5.0e-6 / 0.0192, 1.0e-5 | W/(rad/s), W/(rad/s)² | [EST] iron loss; side = 2 × unit |
| u.J / J (side) | 2.8e-4 / 5.6e-4 | 2.0e-4 / 4.0e-4 | kg·m² | [EST] rotor; side = 2 × unit |
| lam | 1e-4 | 1e-4 | N·m·s/rad | bearings and seals of the side (2 × 0.5e-4) |
| Tc | 5e-3 | 5e-3 | s | ESC current-loop equivalent torque lag, until the ESC is identified (D50) |
| Res | 3e-3 | 3e-3 | Ω | DC harness from the distribution to the ESC (POWER must not count it twice) |
| V_nom | 48 | 48 | V | the motors' rated voltage, the envelope's reference |
| w_t_rpm | [0 500 1000 1500 2000 2500 3000 3300 3600 3800 4000 4200] | the same | rpm | 2.4 |
| T_pk_nom (side) | [19.1 19.1 19.1 19.1 19.1 15.28 12.74 7.0 0 0 0 0] | [14.32 14.32 14.32 14.32 14.32 11.46 9.54 5.2 0 0 0 0] | N·m | 2 × `u.T_pk_nom` (2.4) |
| T_cont_nom (side) | [6.36 6.36 6.36 6.36 6.36 6.36 6.36 4.0 0 0 0 0] | [4.78 4.78 4.78 4.78 4.78 4.78 4.78 3.0 0 0 0 0] | N·m | 2 × `u.T_cont_nom` (2.4) |
| vdc_t | [36 42 48 54.6] | the same | V | the 13S pack's range (39.0 to 54.6 V in operation) with margin below |
| T_pk_2D | computed by `local_envelope2D` | computed by `local_envelope2D` | N·m | speed rows by voltage columns, tabulated in the envelope section |
| w_eff_rpm | [-4000 -3000 -2000 -1000 -400 0 400 1000 2000 3000 4000] | the same | rpm | loss-map axis |
| T_eff | [0 2.75 5.5 8.25 11 13.75 16.5 19.25 22] | [0 2 4 6 8 10 12 14 16] | N·m | loss-map axis, 0 to ceil(1.1·T_peak) |
| loss_T1 / loss_T2 | computed (11×9) | computed (11×9) | W | 2.5 at Tmeas1 = 25 °C / Tmeas2 = 125 °C |
| Tmeas1 / Tmeas2 | 25 / 125 | 25 / 125 | °C | |
| u.mass / mass (side) | 4.5 / 9.0 | 3.5 / 7.0 | kg | 1 kW [EST] (86 to 90 mm frame); 750 W 80 mm flange [EST] |

### 6.4 ESC of a side (`D.esc`, `param_drive.m`:58-110; provisional, D96)
| Field | 1 kW (default) | 750 W | Units | Source |
|---|---|---|---|---|
| node_id | [1 2] | [1 2] | – | L, R (one CAN node per side; both channels take the same command) |
| I_rms_cont / I_rms_peak | 54 / 130 | 40 / 100 | A | 2 × the channel rating `u.esc` [EST] (the as-built ESC: 20 / 50 A_rms per channel [EST]) |
| I_dc_max_mot / I_dc_max_regen | 110 / 40 | 80 / 30 | A | 2 × `u.esc` [EST]; the MCU keeps the pack within DCL and CCL |
| R_on | 2e-3 | 2e-3 | Ω | effective per phase leg (4 mΩ per channel / 2) |
| k_sw | 1.6e-4 | 1.6e-4 | W/(V·A) | switching loss |
| P_q, R_q | 5, 460.8 | 5, 460.8 | W, Ω | logic draw, R_q = V_nom²/P_q |
| V_on / V_off | 38 / 35 | 38 / 35 | V | power-up thresholds (13S NMC: 39.0 to 54.6 V) |
| V_uv_derate | [37 40] | [37 40] | V | motoring torque 0 → 1 |
| V_ov_derate | [57 60] | [57 60] | V | regen torque 1 → 0 |
| V_ov_fault / V_uv_fault | 62 / 35 | 62 / 35 | V | |
| I_oc_fault | 143 | 104 | A | 1.3 × I_dc_max_mot, DC, 10 ms |
| J_eq | 3.76e-3 | 3.54e-3 | kg·m² | J + m_side·r_spr²/N² (the side's half-mass reflected to the motor) |
| Kp_w / Ki_w | 0.030 / 0.090 | 0.030 / 0.090 | N·m/(rad/s), N·m/rad | D64, the four-motor re-tune on the two-mass model |
| w_c | 8.0 | 8.5 | rad/s | Kp_w/J_eq, rigid crossover |
| cmd_delay | 1 | 1 | samples | 1 ms Unit Delay on every ESC → plant command (D36) |
| a_max / a_qs | 497.8 / 444.4 | 497.8 / 444.4 | rad/s² | 2.24 / 2.0 m/s² × k_v2w (D33) |
| J_ff | 3.76e-3 | 3.54e-3 | kg·m² | = J_eq, the ESC's own ramp feed-forward in the hold-zero states |
| enc_cpr / n_diff | 4096 / 4 | 4096 / 4 | counts, samples | 1024 ppr × 4 |
| tau_wf | 2e-3 | 2e-3 | s | speed LPF |
| t_boost / t_recover | 30 / 60 | 30 / 60 | s | I²t |
| x_on / x_off | 1 − 1e-6 / 0.01 | the same | – | I²t relay (peak again after about 60 s of zero torque, T4) |
| k_boost | 240.6 | 239.2 | s | ((T_peak/T_cont)² − 1)·t_boost |
| tau_cont_min | 0.3 | 0.3 | N·m | floor of the continuous rating in the I²t ratio (2 × 0.15) |
| T_mot_derate / T_mot_fault | [413.15 438.15] / 443.15 | the same | K | 140 → 165 °C / 170 °C (class H) |
| T_esc_derate / T_esc_fault | [358.15 373.15] / 378.15 | the same | K | 85 → 100 °C / 105 °C |
| tau_Tsens | 5 | 5 | s | winding sensor lag |
| w_overspeed | 418.9 | 418.9 | rad/s | 4000 rpm (`n_mech_max`), 1.89 m/s |
| t_can_timeout / n_can_to | 0.1 / 100 | the same | s, samples | 10 frames |
| t_premag / t_stop_hold / w_stop / t_park / t_init | 0.05 / 0.2 / 2.0 / 2.0 / 0.1 | the same | s, s, rad/s, s, s | brake sequencing (2.9) |
| t_stop_max | 2.0 | 2.0 | s | [EST] DISABLING / QUICK_STOP brake time-out from the start of the stop (critfix R-01); tick value `P.drive.sm.n_stop_max` = 2000, element 16 of `P.drive.sm_vec` |
| stall_tau_frac / stall_w / t_stall_warn / t_stall_fault | 0.9 / 5 / 5 / 10 | the same | –, rad/s, s, s | |
| C_link / ESR_link / R_bleed / vc0 | 2.2e-3 / 10e-3 / 1e4 / 0 | the same | F, Ω, Ω, V | [EST] DC link inside the ESC (POWER's precharge sees it, D98) |
| w_min_dc / w_quad | 50 / 5 | the same | rad/s | speed floor of the DC-current limits; quadrant threshold |
| n_oc / n_ov / n_uv / n_os | 10 / 5 / 50 / 10 | the same | samples | Protection Relay debounce at 1 ms |
| n_ot / n_enc / n_brake | 1 / 2 / 100 | the same | samples | temperature, encoder and brake fault debounce |
| mass | 1.6 | 1.6 | kg | [EST] the dual-channel ESC of the side (2 × 0.8) |

### 6.5 Brake of a side (`D.brake`, `param_drive.m`:119-123)
| Field | Value (both motors) | Units | Source |
|---|---|---|---|
| T_static / T_dyn | 10 / 8.4 | N·m | [EST] 2 × 5 N·m per brake (`u.brake_T`); 0.84 × static |
| F_spring | 500 | N | 2 × 250 |
| r_eff | 0.05 | m | lumped two faces |
| mu_s / mu_c | 0.40 / 0.333 | – | |
| w_st / k_b / c_b / w_d (D64) | 0.1 / 50 / 0.3 / 1 | rad/s, N·m/rad, N·m·s/rad, rad/s | [EST] bristle 25 N·m/rad and 0.15 N·m·s/rad per brake |
| T_eps / w_eps / w_r / N_eps (D64) | 1e-4 / 1e-3 / 1e-4 / 1 | N·m, rad/s, rad/s, N | numerical guards (a released brake gives no torque) |
| t_release / t_engage | 0.04 / 0.06 | s | [EST] |
| t_engage_wait | 0.08 | s | torque stays on 80 ms after the brake command drops (2.9) |
| R_coil / duty_hold / duty_min / t_pullin | 46 / 0.48 / 1e-4 / 0.2 | Ω, –, –, s | two 92 Ω coils in parallel |
| mass | 1.0 | kg | 2 × 0.5 |

### 6.6 Thermal of a side (`D.thermal`, `param_drive.m`:140-141)
Table 2.6 gives every field for both motors; `cp_h` 750 and `cp_esc` 900 J/(kg·K), `h_h` 10 W/(m²·K), `dT_solar` 10 K and `T_amb0` 318.15 K (45 °C if the world gives none) are common.

### 6.7 CAN (`D.can`, `param_drive.m`:147-153)
| Field | Value |
|---|---|
| bitrate | 500e3 bit/s |
| id_cmd / id_fb1 / id_fb2 / id_fb3 | 0x200 / 0x180 / 0x280 / 0x380 (+ node) |
| res_w / res_tau / res_tau_lim / res_i / res_v / res_T | 2·pi/60 rad/s, 0.01 N·m, 0.1 N·m, 0.1 A, 0.01 V, 0.1 K |
| T0 / T_esc_ofs | 273.15 K (K to °C on the wire) / 40 °C (the u8 T_esc offset, −40 to 215 °C) |
| t_rx_timeout / n_rx_timeout | 0.05 s / 5 frames (MCU side, `drive_fb_unpack` rx_ok) |
| len | 8 bytes (classic DLC of every drive frame) |

### 6.8 Fault-injection defaults and the IDEAL variant
`fault_inj_default.L = fault_inj_default.R = struct('can_drop',false,'enc_fail',false,'brake_stuck',false,'tau_scale',1,'T_offset',0)`.
`D.ideal`: `tau_v` 0.05 s, `a_max` 497.8 rad/s², `eta` 0.80 (`k_loss` 0.25), `P_aux` 8 W, `V_min` 20 V and `V_gate` 10 V (no current below about 20 V); `D.tie`: K 1e-3 V·s/rad, R 1e6 Ω (D35).

### 6.9 Mass and budget handed to STRUCTURE and POWER
* **Mass per side** (`P.drive.mass_per_side`): 2 × (motor 4.5 + gearbox 3.5 + brake 0.5 + half the dual ESC 0.8 + cables 0.4) = **19.4 kg** with the 1 kW units, 2 × 8.2 = 16.4 kg with the 750 W (38.8 and 32.8 kg for the robot) [EST]. The 220 kg of the robot's spec sheet include the four as-built units, so the 1 kW units add 6 kg (226 kg).
* **Inertia per side:** `J_out` 0.008 kg·m² at the sprocket; the reflected motor inertia `P.drive.J_reflected_spr` = 0.224 kg·m² (750 W: 0.160) is included dynamically through the gearbox, so STRUCTURE must not add it again.
* **POWER, per side:** 18.2 A on the 20° climb at 1 m/s (T3, sandy soil, 158 kg, 2026-11-15); at most `I_dc_max_mot` 110 A motoring and `I_dc_max_regen` 40 A regenerating (750 W: 80 and 30 A); the MCU keeps the sum of both sides within the pack's DCL and CCL (REQ-SYS-053). Standby: the ESC logic 5 W per side; while driving, the four brake coils hold 12 W each (25 W each for 0.2 s at release). The mission energy follows D101 (no fixed shift; the power spec and card G2-PWR).

---

## 7. Internal structure, files and model types

```
subsystems/drive/
  drive_system.slx        Subsystem Reference  (placed in models/Shaddad_System.slx through Shaddad_Plant)
   |- drive_cmd_unpack    Subsystem Reference  CAN1 DRV_CMD frames -> Bus_DriveSideCmd (codegen-clean)
   |- Rate Transitions    10 ms <-> 1 ms
   |- side_L : drive_side.slx  (Subsystem Reference, mask: side = 1)
   |- side_R : drive_side.slx  (side = 2)
   |- drive_fb_pack       Subsystem Reference  Bus_DriveSideFb -> FB1/FB2/FB3 frames (codegen-clean)
  drive_side.slx          Subsystem Reference (masked)
   |- esc     : drive_esc.slx        Subsystem Reference, atomic, Ts = 1 ms (MATLAB Function ESC_SM inside)
   |- plant   : Variant Subsystem (VC_DRIVE_FID, activation 'update diagram')
   |     |- SYS   : drive_plant_sys.slx    (Motor & Drive thermal, DC link, sensors, aux loads)
   |     |- FOC   : drive_plant_foc.slx    (PMSM + AVVSC + current control at 1e-4 s, bench-only)
   |     |- IDEAL : drive_plant_ideal.slx
   |- gear    : drive_gearbox.slx    Subsystem Reference (physical)
   |- brake   : drive_brake.slx      Subsystem Reference (physical + 1 ms actuator)
   |- thermal : drive_thermal.slx    Subsystem Reference (physical)
   |- truth   : bus assembly (PS-Simulink, 1 ms)
  drive_cmd_pack.slx, drive_fb_unpack.slx   the MCU's halves of the codecs (codegen-clean)
  esc_sm.m, esc_torque_limit.m, esc_feedback.m   the MATLAB Functions of drive_esc (section 4)
  +shaddad_drive/elastoplastic_brake.ssc   the brake component (D64)
params/param_drive.m, params/param_motor_unit.m   parameter functions (+ the local envelope and loss-map generators)
buses/buses_drive.m       Bus_DriveSideCmd / DriveCmd / DriveSideFb / DriveFb / DriveSideTruth / DriveTruth / DriveSideFaultInj / DriveFaultInj
scripts/build/build_drive.m      the builder of every file above
tests/drive/build_tb_drive.m     the benches tb_drive (one side: DC source P.drive.bench.V_src 46.8 V + R_src 20 mΩ,
                                 Electrical Reference, Solver Configuration, load = Inertia m_side·r_s² at the sprocket
                                 + Rotational Friction (rolling) + Ideal Torque Source (grade), or a dyno speed source),
                                 tb_drive_sys (both sides), tb_drive_2mass (D50 two-mass load), tb_drive_foc (FOC)
tests/drive/t_drive.m            the acceptance tests of section 8, with the helpers tb_drive_*.m
```
* Model types: every DRIVE piece is a **Subsystem Reference** (they hold Simscape physical ports, and the ESC is third-party firmware, so it is not code-generated by us). The codecs are Subsystem References too, but codegen-compliant (single and integer types, no doubles: test `testCodecTypes`), because the STM32 Model Reference instantiates them.
* Sample times: CAN 10 ms (FB2 50 ms); ESC 1 ms; Simscape 1 ms local solver (SYS, IDEAL) or 50 µs (FOC); the thermal sensor lag at 1 ms. Every rate change goes through a **Rate Transition** (deterministic, ZOH).
* `drive_side` mask parameter: `side` (1 or 2) selects `P.drive.esc.node_id(side)` and the fault-injection sub-bus. No sign parameter (positive = forward on both sides).
* The variant control variable `VC_DRIVE_FID` is created by `shaddad_params.m` from `P.drive.fidelity`.

---

## 8. Verification (`tests/drive/t_drive.m`, SYS variant unless noted)

**The bench load per side follows P (D64):** `P.drive.bench.m_side` = 158 kg (750 W: 155 kg), so J_load = m_side·r_s² = 1.28 kg·m² at the sprocket (750 W: 1.26); rolling resistance through Rotational Friction (Coulomb = f_rr·m_side·g·r_s, `P.drive.bench.f_rr` 0.05 unless a test sets it); grade through an Ideal Torque Source (m_side·g·sin(th)·r_s = 47.7 N·m at 20°); pack equivalent `P.drive.bench.V_src` = 46.8 V + `R_src` 20 mΩ. The criteria read their limits from P (T_peak, T_cont, T2B, T2NOT, the top speed), so they follow the motor option.

**The baseline** (`tests/test_baseline.json`, full run of 2026-11-15, the 1 kW default): 34 tests, 29 pass, 5 stop as incomplete (T2, T5, T6, T8, T17), about 16 min. The measured values below are from that run; the 750 W option has not been run through `t_drive`.

| ID | Scenario | Pass criteria | Measured (2026-11-15, 1 kW) |
|---|---|---|---|
| T1 | SPEED mode, MCU-shaped reference 0 → 1.0 m/s, flat | v ≥ 0.95 m/s 1.5 s after the command; overshoot < 5 %; \|e_ss\| < 0.01 m/s after 3 s | 0.977 m/s; 0.01 %; 0.0001 m/s |
| T2 | Unramped step 0 → 0.5 m/s (a_max = inf) | torque ≤ T_peak (no wind-up); overshoot < 10 % and 2 % settling < 0.6 s are the former gains' criteria | 2.88 N·m; 13.4 % and 1.04 s: **incomplete**, a known conflict (the D64 gains' rigid poles −4.0 ± 2.8j rad/s give settling ≥ 1.0 s); the criterion waits for G2-DRV |
| T3 | 20° climb at 1.0 m/s, sandy soil (rolling resistance 0.08, set by the test) | speed error < 2 %; tau_m = (grade + rolling)/(N·eta) + no-load drag ± 5 %; i_dc·v_dc = p_dc ± 2 %; motor + ESC efficiency 75 to 97 % | 0.00 %; 3.320 N·m (expected 3.320); 18.23 A at 46.4 V, 847 W; 87.1 % |
| T4 | Stall: TORQUE mode at T_peak against a locked sprocket | T_peak ± 1.25 % for 30 ± 1 s; then fold-back to ≤ T_cont within 1 s; the peak available again after 60 ± 5 s of zero torque | 29.98 s at 19.10 N·m; 6.36 N·m; 59.4 s |
| T5 | Dyno, rated point, 2 h; then 50 °C + solar | (test text) 3000 rpm and 8 N·m at 40 °C: T_winding(2 h) = 129 ± 8 °C, 63 % rise 20 ± 5 min; at 50 °C + solar: derating active, T_winding never > 165 °C | **incomplete** (Long, not run); its numbers are the former motor's, so the test waits for the motors' rating on G2-DRV (R-57, DRV-06) |
| T6 | Efficiency spot checks | (test text) the former motor's datasheet points | **incomplete** (pending): waits for the motors' efficiency maps (open question 5) |
| T7 | Regen: 20° descent at 1.0 m/s | speed error < 2 %; regenerating (i_dc < 0) with p_dc = −(30 to 95 %) of the gravity power; at a stiff 59 V source the regen torque ≤ 33 % of the regen limit and no overvoltage fault | 0.00 %; −6.42 A, −301 W = 57 % of 530 W; 1.70 N·m = 8.9 %, no fault |
| T8 | Brake hold on 20°, drive disabled, 600 s | brake engaged 60 ± 10 ms after the command; creep < 0.05 m/h at the track; \|w_m\| < 2e-3 rad/s | **incomplete** in the baseline (Long, not run); D64's run: creep 0.0004 m/h, \|w_m\| 2.4e-5 rad/s |
| T9 | STO at the top speed (`P.electronics.lim.v_track_max` 1.41 m/s), flat and 20° downhill | torque command 0 within 1 ms; stop < 0.6 m (flat) and < 1.0 m (downhill); peak \|tau_spr\| < T2NOT; STO_ACTIVE in the status (b8) and the fault code (b10) | 0 ms; 0.125 m and 0.168 m; 156 and 164 N·m (T2NOT 480 N·m); bits set |
| T10 | CAN time-out: alive frozen at t = 5 s at 1 m/s | QUICK_STOP at 100 ± 10 ms; deceleration 2.0 ± 0.15 m/s²; brake engaged at standstill (< 0.011 m/s); fault bit 6; state 7 | 100 ms; 2.027 m/s²; 0.0001 m/s; set; 7 |
| T11 | Undervoltage: Vdc ramp 51 → 34 V above T_peak | motoring limit linear 1 → 0 from 40 to 37 V (± 2 % of T_peak); UV fault 50 ms below 35 V; no solver failure | 0.00 %; 50 ms; no error |
| T12 | Contactor opens while coasting at 1 m/s | no solver error; NOT_READY within 10 ms of Vdc < 35 V; brake engages; the FB frames continue | 0 ms; f_rel 0; 190 alive values, rx_ok, state 1 |
| T24 | STO at 3 s, contactor open 3.5 to 7 s | no UNDERVOLTAGE latched, no FAULT; READY or OPERATION when HV returns | none; none; state 4 |
| T13 | Energy balance over a T3 + T7 sequence | \|∫p_dc − (∫p_spr + ∫losses + ΔE_kin + ΔE_th)\| < 0.5 % of ∫\|p_dc\| | 0.0002 % |
| T14 | Sign, direction and side independence: w_ref L = +100, R = −100 rad/s | w_spr L > 0, R < 0; enc_cnt L up, R down; no cross-coupling | ±5.000 rad/s; ±166 894 counts; 0 rad/s |
| T15 | SYS against FOC (the T1 and T3 profiles, 10 s each) | speed RMS difference < 2 %; DC energy difference < 3 %; peak i_dc difference < 5 % | 0.28 %; 1.95 %; 1.07 % |
| T16 | Codec round trip with 10 000 random commands and feedbacks | decoded = quantised(original) bit-exactly; alive wrap 255 → 0; invalid-frame hold; frame bytes = ICD 4; rx_ok false from an age of 50 ms | 0 mismatches; all as expected |
| T17 | Simulation speed: 1 h synthetic duty, both sides, ideal DC source | wall-clock < 60 s (≥ 60 × real time); FOC 10 s scenario < 10 min | **incomplete** (blocked without a C compiler, D56; resolved by D121, 2026-11-23: MinGW64 is the C compiler, so it can be run again, not yet rerun); FOC 79 s |
| T18 | Frame drops p = 0.1 | no false CAN time-out (it needs 10 drops in a row); speed RMS error increase < 1 % | 72 of 801 dropped, longest run 2; none; +0.043 point |
| T19 | tau_lim_reg = 0 on a 20° descent at 1 m/s | regen torque ≤ 0.1 N·m for 2 s | 0.005 N·m (the side runs away to 1.93 m/s, OVERSPEED, FAULT) |
| T19b | `park_req` at 6.2 s on 20°, alone and in the MCU frame (enable 0, park_req 1, both limits 0) | brake applied within `t_engage_wait` 80 ms of DISABLING; speed to 0; no OVERSPEED, no FAULT; READY; peak \|tau_spr\| < T2NOT | 60 ms after DISABLING at 1.14 m/s (1.16 m/s with the MCU frame); 0.17 m; READY; 170 N·m |
| T19c | Enable 0 at 6.0 s, on 8° and 20° | 8°: the time-out applies the brake 2.00 to 2.08 s after DISABLING, no OVERSPEED, no FAULT, READY; 20°: brake within 2.08 s (the overspeed trip may come first) | 8°: 2.060 s, 2.50 m, READY; 20°: 0.548 s, through FAULT (OVERSPEED first) |
| T19d | Alive frozen at 5.9 s (QUICK_STOP at 6.000 s), on 8° and 20° | QUICK_STOP 100 ± 10 ms after the freeze; 8°: the time-out applies the brake 2.00 to 2.08 s after QUICK_STOP, no OVERSPEED; FAULT with CAN_TIMEOUT (bit 6) | 8°: 2.060 s, FAULT 0x0040; 20°: 0.570 s (OVERSPEED first) |
| T20 | Encoder fault | FAULT within 10 ms; brake engaged within 70 ms | 2 ms; 62 ms |
| TRIP | tau_scale 0 with enable | OVERCURRENT bit 0 and FAULT within 10 ms; \|tau_est\| < 0.05 N·m after 40 ms | 1 ms; 0.0002 N·m |
| T21 | Two-mass bench, k_x 40 / 61 / 540 kN/m, m_side 113 and 158 kg (750 W: 110 and 155 kg) | Ms ≤ 1.6, zeta_min ≥ 0.35, vehicle overshoot ≤ 2 %; the loop model within 15 % RMS of the bench | Ms 1.10 to 1.37, zeta 0.57 to 0.70, PM 62° to 67°, ≤ 0.06 %; 2.4 % and 2.0 % |
| T22 | MCU stop profile at 2.0 m/s² from 1.2 m/s | speed error ≤ 0.05 m/s (no ESC ramp limitation) | 0.037 m/s |
| T23 | Peak motor torque through the gearboxes, sprocket at ±2 rad/s | \|tau_spr\| ≤ T2B; load spectrum logged (T9 runs) | 342.6 N·m (T2B 360 N·m); locked sprocket 382.0 N·m (information, below T2NOT 480 N·m) |
| IDEAL | IDEAL variant on the T1 profile | v(2.5 s) ≥ 0.9 m/s; i_dc > 0 when motoring; thermal port fed | 0.954 m/s; 3.44 A; fed |
| Others | Interfaces, algebraic loops, variants, codec types, code generation | 0 interface mismatches; 0 algebraic loops on the three benches; both system variants compile; no double inside the codecs | all pass |

**Planned on card G2-DRV:** the 750 W option through `t_drive`, with the 750 W descent table on card G2-STR, before the robot moves on its 750 W motors with firmware built from the twin (D95's flashing condition; the lead systems engineer's ruling of step 19, point 24); until then no drive analysis here covers the 750 W robot's runs; T2, T5 and T6 re-baselined on the motors fitted; the hill-start roll-back bench case (2.9, REQ-SYS-112); a detectable brake and stall fault (R-42).

---

## 9. Open questions, risks, convention feedback

### Open questions
1. **Sprocket radius and gear ratio** (STRUCTURE, the supplier): 0.090 m and 20:1 are estimates (G1-04). A change rescales k_v2w and the top speed.
2. **The ESCs** (D96, R-14): the delivered dual-channel ESCs' manual, CAN protocol, speed loop, torque lag, fault reactions, STO wiring and current ratings (card G1-04). Then identify Kp, Ki, Tc and the encoder resolution, map the protocol and mark ICD 4 final. **The 1 kW unit's channel rating (27 / 65 A_rms) is above the as-built ESC's estimate (20 / 50 A_rms per channel):** the upgrade of D95 may need new ESCs as well.
3. **Regen acceptance:** with a full or hot pack (CCL 0) the MCU stops regen (`tau_lim_reg` 0) and the descent becomes stop-go on the brakes (D74). Whether the robot needs a brake resistor stays open.
4. **The motors:** the 1 kW unit is still to be chosen (D95); the Movematic datasheet is still being collected (G1-04). Rs, L, lambda, the iron-loss, envelope and thermal data are estimates: replace them with datasheet values or a test efficiency map (the block takes the tabulated map directly). **Until the 1 kW units are fitted, firmware from the twin goes on the robot only with torque limits that match the motors fitted** (D95: 14.32 N·m per side for the 750 W, not 19.1).
5. **Efficiency maps** of the motors and ESCs for T6, and the 750 W motor's continuous rating at 45 °C (2.15 N·m estimated, against the thermal model's 109 °C winding at its rated point).
6. **The hill start** (R-37): the unpark starts with zero holding torque (2.9); the fix is pending on G2-MCU with G2-DRV.
7. **Brake monitoring** (STATUS 27, R-42): the brake confirmation echoes the coil command; a mechanically failed brake is not detected before motion.

### Risks
| ID | Risk | Mitigation |
|---|---|---|
| R1 | Port labels of `Motor & Drive (System Level)` could not be read in read-only mode. | Resolved at the build: the ports are wired by `get_param(blk,'PortHandles')` and checked by `testInterfaces` and `testVariantsCompile`. |
| R2 | Programmatic selection of the thermal variant. | `simscape.compiler.sli.internal.setActiveVariant` in the builder; checked at every build. |
| R3 | Mixed rotational domains (classic + AB) in the former brake. | Replaced by the elasto-plastic component (D64) in the classic domain. Fallback: classic Ideal Torque Source with T = −T_brk·tanh(omega/omega_brk). |
| R4 | Stateflow availability. | Not installed: the state machine is a MATLAB Function (D1). |
| R5 | A single Simscape network spans POWER, DRIVE and STRUCTURE, so one Solver Configuration and one local-solver step for all. DRIVE needs Ts ≤ 1.7 ms. | The integrator owns the solver; Backward Euler 1 ms, fixed cost. |
| R6 | Motor & Drive DC current ∝ P/V when V → 0 (contactor open while spinning). | The ESC drops Tr below V_off; bleed resistor; T12 and T24 pass. |
| R7 | Backlash and track-sprocket compliance are not in DRIVE. | STRUCTURE models the track compliance with damping; the speed PI is designed on it (T21). |
| R8 | Parking creep and stiff dynamics near zero speed from the friction model. | The elasto-plastic brake (D64): 0.0004 m/h creep (T8), no chatter at the 1 ms step. |
| R9 | Algebraic loop in the PS chain of the gear mesh loss (Simscape-internal). | Well-posed (k_loss < 1); fallback: a constant-efficiency Rotational Damper. |
| R10 | The ESC model is a behavioural approximation of third-party firmware (state machine, ramps, limit logic), and the delivered ESC is unknown (D96). | Keep the parameters in `D.esc`; validate against the ESC manual and bench logs (G1-04, G2-DRV). |
| R11 | Codegen: the codecs are used inside the STM32 Model Reference. | Single and integer types only (`testCodecTypes`); `Byte Pack` uses the target endianness (STM32 little-endian = CANopen); T16 bit-exact. |
| R12 | The FOC variant is slow (50 µs). | Bench-only (D52), short scenarios (T15). |
| R13 | Hot climate: the site's ambient reaches 50 °C with sun, and the 750 W motor's continuous rating at 45 °C is estimated at 90 % of its nameplate. | Derating is modelled and reported (tau_avail) to the MCU and GNC; the 1 kW upgrade (D95) gives a margin of 1.58 on the wet-loam RMS. |
| R14 | The envelope and the electrical data of the units disagree above 2500 rpm (2.3). | Both are estimates; set from the datasheets (open question 4), then re-run T15 at high speed. |

### Feedback on the global conventions
* **Subsystem Reference for the plant**: good. For DRIVE, the ESC is also a Subsystem Reference (not Model Reference), because it is third-party firmware that is never code-generated and it must sit with the physical network at 1 ms.
* **HW/SIM variants**: the MCU's drive I/O in SIM passes the CAN1 frames through `CAN1_Net_Sim`; in HW it is `drive_cmd_pack` → FDCAN Write and FDCAN Read → `drive_fb_unpack`. The ESC is never "HW" from the MCU's point of view; it is always external.
* **Solver Configuration**: subsystems must not contain one (exactly one per network).
* **Variant activation**: Variant Subsystems with Simscape blocks need `VariantActivationTime='update diagram'`, not `startup`.

---

## 10. The system requirements allocated to the drive

The register `docs/requirements/system.yaml` (NASL-SHD-REQ-008) allocates **72 requirements** to DRV, as it stands after the fix pass of G1-01 part 2 (step 19, read on 2026-11-18). They are the 31 functional requirements of part 1, nine functional ones that the drive's step 9 proposed (REQ-SYS-024, -030, -076, -079, -095, -106, -108, -113 and -114), the draft REQ-SYS-209, the motion stop for a person in the blades' protective zone (the lead's ruling of step 19, point 7), and 31 technical ones, twelve of which are safety requirements (step 19 added DRV to REQ-SYS-204 and -205). The register was read after the rulings of points 6 and 7 had been written in (REQ-SYS-209 and -210; REQ-SYS-210, the speed cap with the blades running in RC driving, has no DRV). For each one the tables give the drive's share, where this spec meets it, and the drive test that shows it.

### 10.1 The functional requirements (41)

| Requirement | The drive's share | Spec | Drive test |
|---|---|---|---|
| REQ-SYS-001 power-up wait; -079 restart after a watchdog reset or a brown-out; -106 drives disabled while charging | INIT, NOT_READY and READY keep the torque off and the brakes engaged until an enable; a restarted or charging robot sends none, so the brakes stay held | 2.9 rows 0 to 2 | T12, T24 |
| REQ-SYS-006 disarm, controlled stop and park | DISABLING with park_req: brakes at once, torque off 80 ms later | 2.9 row 5 | T19b |
| REQ-SYS-080 pack isolated | spring brakes engage without power; no DC link, no torque | 2.7 | T12 |
| REQ-SYS-008, -009 RC driving and its stop | tracking of the MCU's shaped reference, with no ESC ramp limitation | 2.9 | T1, T22 |
| REQ-SYS-024, -025, -026, -035, -037, -054, -067, -068, -071, -076, -095, -108, -113, -114 stop and park or hold | the park on park_req, or the ESC's own park after 2 s at zero command (D66) | 2.9 rows 4 and 5 | T19b |
| REQ-SYS-027, -028 path following and turns | track-speed tracking; the turn torque within the peak | 2.1, 2.9 | T1, T14, T21 |
| REQ-SYS-030 the safe speed for a brakes-locked stop | the brakes' dynamic torque and their engage time set the stop's deceleration | 2.7 | T9 |
| REQ-SYS-031 descent-cap trip, then park within the trip reaction time | the park at once on park_req: brakes applied 70 ms after the request, a chain of 81 ms with the MCU's 10 ms sample, inside 0.09 s; 0.12 s if the delivered ESC needs 0.10 s (D96) | 2.7, 2.9 row 5 | T19b |
| REQ-SYS-209 the motion stop for a person in the blades' protective zone (safety, draft) | the controlled stop and the hold that the zone's sizing assumes: the drive follows the MCU's controlled stop, then parks on park_req and holds on the brakes until the operator's resume (D84) | 2.9 rows 4 and 5 | T22, T19b |
| REQ-SYS-053 pack current within its limits | the ESC's DC-current limits; regen within the MCU's tau_lim_reg | 2.9 | T19, T7 |
| REQ-SYS-055 precharge | the ESC's DC link and logic load per side | 6.4 | – (POWER's tests) |
| REQ-SYS-063 E-stop: STO and the brakes | STO row 8: torque off at once and the brakes engaged; the twin's ESC acts within 1 ms of `sto_ok` falling, inside REQ-SYS-063's STO time (at most 50 ms in G1-01 part 2, provisional until G1-04, D96) | 2.9 row 8 | T9 |
| REQ-SYS-066 remote E-stop | follows the MCU's ESTOP deceleration, then the park | 2.9 | T22, T19b |
| REQ-SYS-069 controller stall | the ESC's 0.1 s command time-out into QUICK_STOP, then the brakes | 2.9 row 6 | T10, T19d |
| REQ-SYS-070 drive feedback lost | the class-A reaction through the STO (row 8); a dead ESC's brakes engage unpowered | 2.9, 2.10 | T18, T9 |
| REQ-SYS-072 every stop held on the brakes | the brakes' holding margin of 4.2 on 20° | 2.7 | T8 |
| REQ-SYS-073 stop monitor, STO | STO row 8 | 2.9 | T9 |
| REQ-SYS-074, -109 no downhill speed gain, overspeed | regen holding within tau_lim_reg; the ESC's own 4000 rpm overspeed trip as the last stage | 2.9 | T7, T19 |
| REQ-SYS-075 no tip-over in a brakes-locked stop | the brakes' torque and engage time set the stop's deceleration (about 8 m/s²) | 2.7 | T9 |
| REQ-SYS-081 track speed mismatch | the ESC's speed feedback (1 rpm, FB1) as one of the two measurements | 2.10 | T16 |
| REQ-SYS-097 slope speed | 1.0 m/s on 20° within the continuous rating, with either motor | 2.1, 2.2 | T3 |
| REQ-SYS-104 mission on one charge | the drive's losses | 2.5 | T13 (energy audit) |
| REQ-SYS-112 hill-start roll-back | the unpark; about 0.1 m today (open, 2.9) | 2.9 | planned (G2-DRV) |

### 10.2 The technical requirements (31)

| Requirement | Kind | The drive's share | Spec | Drive test |
|---|---|---|---|---|
| REQ-SYS-119 the RC speed limit down to the 0.20 reserve | performance | 1.41 m/s at the rated 3000 rpm; about 40 V needed on level ground | 2.1, 2.3 | planned: T1 to 1.40 m/s at the reserve's voltage (G2-DRV) |
| REQ-SYS-120 RC speed and turn tracking | performance | tracking of the shaped reference (T1: 0.977 m/s 1.5 s after the command, steady error 0.0001 m/s) | 2.9 | T1 |
| REQ-SYS-122 stop inside the zone's boundary (safety) | performance | the park, and the drive's following of the MCU's stop | 2.9 | T22, T19b |
| REQ-SYS-125 row speed | performance | speed tracking | 2.9 | T1, T3 |
| REQ-SYS-127 lane-end turns | performance | the side speeds of the turn, within the peak torque | 2.1 | T14, T21 |
| REQ-SYS-128 tractive force, continuous and for 30 s | performance | 2.54 kN continuous through the gears (750 W: 1.91 kN, about 1.72 kN at 45 °C) | 2.1, 2.2 | T3, T4 |
| REQ-SYS-129 longitudinal slopes | performance | the slope torques (L1, L3) within the continuous rating; regen and the brakes on the descent | 2.1, 2.7 | T3, T7, T8, T19b |
| REQ-SYS-130 cross-slopes | performance | the turn torque and the brakes' hold; the slide is STRUCTURE's | 2.1, 2.7 | T8 |
| REQ-SYS-139 coming to rest after an E-stop (safety) | performance | the STO and the spring brakes: on the bench the side comes to rest 0.125 m after an STO at 1.41 m/s on the bench's flat ground (158 kg per side) | 2.7, 2.9 row 8 | T9 |
| REQ-SYS-140 the remote E-stop's deceleration (safety) | performance | follows the MCU's 2.0 m/s² profile within 0.037 m/s; the ESC ramp lies above it (D33) | 2.9 | T22 |
| REQ-SYS-143 the stop after an RC link loss (safety) | performance | follows the MCU's 1.0 m/s² stop, then the park | 2.9 | T22, T19b |
| REQ-SYS-144 brake torque against the slope's demand (safety) | performance | a margin of 4.2 with the design mass on 20° [EST] | 2.7 | analysis (2.7); T8 |
| REQ-SYS-145 creep on the brakes (safety) | performance | 0.0004 m/h (D64) | 2.7 | T8 (Long, not in the baseline) |
| REQ-SYS-146 both brakes engaged after a park request (safety) | performance | 70 ms after the request (T19b); the confirmation echoes the coil command (STATUS 27) | 2.7, 2.9 | T19b |
| REQ-SYS-158 the drive as built | physical | two motors per track, 20:1 gearboxes, a brake per motor, one dual-channel ESC per track | 0, 1.1 | – (inspection, G1-04) |
| REQ-SYS-159 the motors' rating | physical | the 1 kW unit (D95); the as-built 750 W motors are a recorded deviation | 6.3 | T4 |
| REQ-SYS-160 the ESC channels' rating | physical | the 1 kW option's channel rating [EST]; the as-built ESC is estimated lower (9, open question 2) | 6.4 | – (inspection) |
| REQ-SYS-161 the gearboxes' rating | physical | the 1 kW option's ratings [EST] | 2.8, 6.2 | T23 |
| REQ-SYS-164 the ambient range | environment | the thermal model's ambient and its 10 K solar load; the derating | 2.6, 2.9 | T5 (to re-baseline) |
| REQ-SYS-166 tractive force at 50 °C within the derating starts | environment | the rated point's steady states, 120 °C (750 W: 114 °C) [EST] | 2.6 | T5 (to re-baseline, R-57) |
| REQ-SYS-167 the sites' soils | environment | the drive's torque covers the adhesion limit (L6) | 2.1 | T3, T7 |
| REQ-SYS-172 ingress protection | environment | the motors' IP rating is not in the drive's data (G1-04) | – | – |
| REQ-SYS-190 performance levels to ISO 13849 | constraint | the STO and the spring brakes as parts of the safety functions | 2.7, 2.9 | – (G2-FSC) |
| REQ-SYS-194 the STO to IEC 61800-5-2 (safety) | constraint | the delivered ESCs' STO input (D96, G1-04) | 2.9 | – (the ESC manual) |
| REQ-SYS-195 no unexpected start-up, ISO 14118 (safety) | constraint | the ESC's start interlocks: torque off and brakes engaged until an enable | 2.9 rows 0 to 2 | T12, T24 |
| REQ-SYS-200 EMC to ISO 14982 | constraint | the ESCs' and motors' emissions and immunity, not modelled | – | – |
| REQ-SYS-201 the safety functions' immunity, IEC 61000-6-7 | constraint | the ESC's STO and the brakes, not modelled | – | – |
| REQ-SYS-202 the delivered hardware, with provisional values | constraint | the provisional ESC model (D96) | 2.9, 2.10, 6.4 | – |
| REQ-SYS-203 torque limits that match the fitted motors (safety) | constraint | D95's condition; on the 750 W robot the 1 kW limit would pass the gearboxes' T2B | 2.8, 9 | – (inspection, G2-MCU, B-00) |
| REQ-SYS-204 the traction bus below 60 V DC in every state (safety) | constraint | regeneration within the ESC's 57 to 60 V derating band and under its 62 V overvoltage trip; with K1 open the twin's regen lifts the bus 0.7 V above 60 V, so the twin meets it only in part (pending on card G2-PWR: a clamp below 60 V at its rated current and above the charger's 55.9 V, or no regen once K1 can open; the lead's ruling of step 19, point 12) | 2.9 | – (G2-PWR) |
| REQ-SYS-205 the safety components within their rating in the hot case (safety) | constraint | the ESCs' STO inputs within their rated ambient temperature: the delivered ESCs' rating and place (D96, G1-04); the conflict with REQ-SYS-114 and -165 is an open safety value for the owner (the lead's ruling of step 19, point 1) | 2.9 | – (G1-04) |
