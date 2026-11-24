# Shaddad UGV - System Integration and Simulation Framework Specification

> **Revision B note (2026-10-19).** `docs/architecture.md` section 2b (D30-D63) and `docs/build_plan.md`
> override this spec where they differ. Sections changed for this domain: E4 energy audit closed with plant integrals (eps_el, eps_mech, D40); E5 latency budget and measurement points (D37); 7.4 parameter behaviour Tunable in simulation, Inlined for codegen, P_meta (D58); local solvers for the reference run set as block parameters by run_scenario (D58); 7.7 rapid accelerator only with VC_TEB 0, VC_VIZ 0 and FAST2D, missions run the top model in Accelerator, endurance fidelity set (D56); T-INT-03 targets S1 RTF >= 5 (10 with MinGW), S3 RTF >= 4; T-INT-05 compiled-rate set `P.sim.Ts_compiled_allowed` incl. 0.014 / 0.007 inside RC_Link_Sim (D32); S3 SOC0 0.95 and SOC_end >= soc_rth (D46; superseded by D101, section 9.2); thermal ports may connect child SSRs inside one domain top SSR (D52); scenario criteria of architecture section 12 are binding.

> **G1-01 part 2 note (2026-11-18).** Brought in line with the parameters and the decisions D1 to D110 by the integration engineer (card G1-01 part 2, steps 12, 19 and 19c): D106, D107 and D108 change the scenario criteria of section 9.2 (S4 and S5), D110 records the baseline that this spec is part of, and D104, D105 and D109 change nothing in this spec. The parameters are `params/param_sim.m` (`P.sim`, `P.frames`, `P.viz`, `P.log`, `P.scen`) and `params/param_world.m` (`P.world`), assembled by `params/shaddad_params.m`; there is no `param_integration.m`. Section 9.2 repeats the scenarios of `docs/architecture.md` section 12, which binds, with the criteria of D100, D101, D106 and D108 and the value of each requirement that names a scenario. Where the sweep of the spec numbers (`work/team/g1-01/p2/sweep/out/integration.csv`) found a value of the earlier robot, it is corrected; the rest of the design text records how the framework was built, and the ICD binds over it where they differ.
>
> **G1-01 part 2, step 19 note (2026-11-18).** D10 dropped integration's `Env_Terrain`: the environment and the worlds are the structure domain's (`structure_env`, the single producer of `Bus_EnvState`, 0.1 s; OQ-02). D7 gives the power domain every LV converter and rail load, and D8 makes the power domain's `Safety_Chain` the only safety chain. Where sections 2 to 10 still name `Env_Terrain`, `Bus_Terrain`, `Bus_EnvSlow`, `Bus_ObstacleSet` or revision A's plant pieces (`Structure_Tracks`, `Power_System`, `Drive_Train`, `Electronics_HW`, `Implements`), they record revision A's design and are superseded: ICD sections 2 and 3 bind (the physical connections, and the 11 components with their links), with the buses of ICD section 10. Sections 1.1, 1.2, 2.1, 4.4, 6, 7.1, 7.12, T-INT-04, T-INT-11 and 9.2 are corrected in this pass.

| Item | Value |
|---|---|
| KEY | `integration` |
| Domain | System integration and simulation framework (top model, hierarchy, solver, rates, logging, visualization, scenarios, data management, tests, build order, frames and sign conventions) |
| Status | Design spec v1.0, build-ready |
| Target release | MATLAB R2026b (26.2.0.3386108), checked live in the shared session on 2026-10-18 |
| Owner of files | `models/Shaddad_System.slx`, `models/Shaddad_Plant.slx` (wiring only), `subsystems/integration/*`, `params/param_sim.m`, `params/shaddad_params.m`, `params/shaddad_configsets.m`, `buses/shaddad_buses.m`, `buses/buses_sim.m`, `scripts/*`, `tests/integration/*`, `tests/system/*`, `tests/run_regression.m` |

Environment facts checked live (they affect the design):
- Stateflow installed (`sflib` present, `toolbox/stateflow` present). `license('test','Stateflow')` = 1.
- Simulink Test installed (`sltestlib`, `sltest.harness.create`, `sltest.TestCase`, `sltest.testmanager.*`).
- Simulink Coder (`toolbox/rtw`), Embedded Coder, MATLAB Coder installed. Parallel Computing Toolbox installed (`parsim`, `batchsim`).
- `license('test','Simulink_Coverage')` and `'SL_Verification_Validation'` return 1, but the `slcoverage`/`slcheck` toolbox folders are missing. Treat coverage and Model Advisor checks as **optional**.
- **No MEX C compiler is selected** (`mex.getCompilerConfigurations('C','Selected')` is empty). `sys/lcc64` exists, so Simulink's bundled LCC-win64 can build simulation targets. See Risks R-02 (resolved by D121, 2026-11-23: MinGW64 is the C and C++ compiler, so its install is done).
- MATLAB project `Shaddad_UGV` is loaded. Its `SimulinkCacheFolder` and `SimulinkCodeGenFolder` are both `work/`. There are no startup files and no shortcuts yet.
- Simscape `Connection Port` `ConnectionType` strings used in shipped libraries: `'Inherit: auto'` and `'Connection: foundation.electrical.electrical'`.

---

## 1. Purpose and scope

### 1.1 In scope (owned by integration)
1. Top model `models/Shaddad_System.slx`: layout, wiring, root scenario input, rate-transition wrappers, dashboards.
2. Plant wrapper model `models/Shaddad_Plant.slx`. It contains only wiring of the domain Subsystem References, the single Simscape `Solver Configuration`, and electrical references.
3. Environment and terrain: dropped from integration by D10. The environment and the worlds are the structure domain's (`structure_env` inside `structure_plant`, the single producer of `Bus_EnvState`; OQ-02); integration keeps only the scenario overrides `Bus_EnvOverride` (6.1).
4. Visualization and logging `subsystems/integration/Viz_Logging.slx` (live 2D bird's-eye view, XY graph, SDI Record), and the logging manifest.
5. Simulation monitors `subsystems/integration/Sim_Monitors.slx`: KPIs, collision check, NaN guard, stop conditions.
6. Project-wide **coordinate frames, units and sign conventions** (Section 3). These are binding on all domains.
7. Solver and configuration sets, multirate scheduling rules, rate-transition policy, and the simulation-speed budget.
8. Parameter, bus, variant and configuration-set data management. `shaddad_params.m` and `shaddad_buses.m` assemble what the domains provide.
9. Scenario framework (S0-S8), scenario runner, criteria evaluation, `parsim` batches.
10. Simulink Test and `matlab.unittest` conventions, the regression suite, and the build-order and dependency plan.
11. MATLAB project startup/shutdown scripts and shortcuts.

### 1.2 Out of scope (owned by neighbouring domains; integration only defines the interface contract)

| Domain KEY | Owns | Integration interface |
|---|---|---|
| `structure` | Chassis rigid-body dynamics, track-terrain (skid-steer) interaction, mass properties, the implements' load mechanics (D9); the environment and the worlds (D10, OQ-02) | `Bus_TruthState` and `Bus_EnvState` out (ICD 3); Simscape rotational ports `Spr_L`, `Spr_R`, `Impl_Shaft` (ICD 2) |
| `power` | The 13S15P NMC pack (Simscape Battery), BMS, contactor/precharge, fuses, the LV converters and rail loads (D7), the E-stop safety chain `Safety_Chain`: STO, K3 and the delayed K1/K2 coil supply (D8); the implement drives (D9) | the power buses of ICD 3; Simscape electrical `DC48_TracL_p/n`, `DC48_TracR_p/n` and rotational `Impl_Shaft` (ICD 2) |
| `drive` | 4 traction motors, two per track, on one dual-channel ESC per side (averaged), gearboxes, spring brakes | the drive buses of ICD 3; ports `DC48_TracL_p/n`, `DC48_TracR_p/n`, `Spr_L/R` (ICD 2) |
| `electronics` | Harness, the LV demand (`Bus_LV_LoadDemand`), `Electronics_Plant`, MCU board power, MCU-GNC link model (CAN-FD); to the safety chain the MCU gives only DO0 and the DO1 re-arm pulse (D8) | the electronics buses of ICD 3; no physical port (D7); `subsystems/electronics/Link_MCU_GNC.slx` |
| `sensors` | RTK GNSS, 9-axis IMU, encoders, 2D/3D LiDAR, analog sensing | the sensor buses of ICD 3 |
| `rc` | RC transmitter, SBUS encoder, RF link, receiver, SBUS decoding (MCU side) | `Bus_OperatorCmd` in (from integration); `Bus_SbusStream` out (ICD 3) |
| `mcu` / `electronics` | STM32 firmware model `Shaddad_MCU(_App).slx` | Root I/O buses, rates, IO variant pattern (Section 7.3) |
| `gnc` | Companion GNC model `Shaddad_GNC(_App).slx`: EKF fusion, planning, guidance, mission manager | Root I/O buses, rates, phase variant |

---

## 2. Architecture and model description

### 2.1 Top-level data flow (`models/Shaddad_System.slx`)

```
                 +-------------------------------- root In Bus Element "ScenarioIn" (Bus_ScenarioIn, Ts = 0.01 s) ----------------------+
                 | .op                         .flt (faults)                         .obs (dyn. obstacles)      .env (overrides)          |
                 v                               |                                       |                          |                   |
      +-----------------------+                  |                                       v                          v                   |
      | Operator_Console      |  op  +-----------v-----------+   sbus   +-----------+  act   +---------------------------------+     |
      | (virtual subsys:      |----->| Operator_RC  (SSR,rc) |--------->| RT_MCU_In |------->| [Model] Plant = Shaddad_Plant   |     |
      |  Variant Source       |      | Tx->SBUS->RF->Rx      |  1 ms    +-----+-----+        |  Env_Terrain, Structure_Tracks, |     |
      |  VC_OPSRC + dashboard)|      +-----------------------+                |              |  Power_System, Drive_Train,     |     |
      +-----------------------+                                               v              |  Electronics_HW, Implements,    |     |
                                                                     +-----------------+     |  Solver Configuration           |     |
                                         gnc2mcu_rx (1 ms)  +------->| [Model] MCU     |     +---+-----------------------------+     |
                                                            |        | Shaddad_MCU     |--act--> RT_Plant_In (in Plant model)        |
                                                            |        +--+-----------+--+         | truth,pwr,drv,elec,impl,           |
                                                            |   mcu2gnc_tx (5-100ms)| mcu_status | terrain, env_slow (1 ms / 0.1 s)    |
                                                            |           v           v            v                                    |
                                                  +---------+-----------------+  +------------------+   sens_mcu (1 ms)                |
                                                  | Link_MCU_GNC (SSR, elec)  |  | Sensors (SSR)    |---------------> RT_MCU_In       |
                                                  +---------+-----------------+  +--------+---------+                                 |
                                                 mcu2gnc_rx |  ^ gnc2mcu_tx (50 ms)        | sens_gnc (5 ms)                          |
                                                            v  |                           v                                          |
                                                  +--------------------------------------------+                                      |
                                                  | GNC_Variant (Variant Subsystem, VC_PHASE)  |<-- RT_GNC_In                         |
                                                  |  1: GNC_Stub   2: [Model] Shaddad_GNC       |---> gnc_status (0.2 s)               |
                                                  +--------------------------------------------+                                      |
                                                                                                                                      |
      +--------------------------+   +--------------------------+      Dashboard panel (simulink_hmi_blocks) bound to top-level        |
      | Viz_Logging (SSR)        |   | Sim_Monitors (SSR)       |      signals / Operator_Console constants                            |
      | VC_VIZ: none/BEV/XY      |   | KPI, collision, NaN,     |                                                                      |
      | Record (SDI)             |   | Stop Simulation          |<---------------------------------------------------------------------+
      +--------------------------+   +--------------------------+
```

The rates in the diagram follow ICD section 3, which binds: the CAN2 frames mcu_imu 5 ms, mcu_odom 10 ms, mcu_status 50 ms and mcu_power 0.1 s; comp_cmd 50 ms; gnc_status 0.2 s. The diagram is revision A's: the top level is the 11 components of ICD section 3.1 joined by direct links, and D10 dropped `Env_Terrain` from the plant (the step-19 note).

Main decisions and why:
- **The Plant is a Model Reference (`Shaddad_Plant.slx`), built from domain Subsystem References.** Simscape physical lines cannot cross a Model block boundary, so every physical network must sit inside one model. Wrapping the whole physical plant in one referenced model gives four things: (a) the top model has no physical lines; (b) the most expensive part to compile gets its own cached accelerator target; (c) the plant can be tested on its own with a harness; (d) agents can build plant pieces in parallel while integration owns only the wiring. This refines the global convention, which still holds inside the plant model ("plant pieces are Subsystem Reference files").
- `Sensors` stays a top-level Subsystem Reference (signal-only). It can later be swapped for HIL sensor feeds without touching the plant.
- Controllers are Model References with bus-typed root I/O: `Shaddad_MCU.slx` (contains IO variants and references `Shaddad_MCU_App.slx`) and `Shaddad_GNC.slx` (likewise with `Shaddad_GNC_App.slx`).
- Phase 1 (RC only) runs with `VC_PHASE = 1`, where the GNC block is replaced by `GNC_Stub` (constant `Bus_GNC2MCU` with `valid = false`). The whole RC chain can therefore be built and verified before the GNC exists.

### 2.2 Equations owned by integration

**(E1) Kinematic oracle for frame tests (skid-steer, ideal):**
`v = (v_R + v_L)/2`, `omega = (v_R - v_L)/(chi * B)`, `x_dot = v cos(psi)`, `y_dot = v sin(psi)`, `psi_dot = omega`.
B = `P.structure.geom.B` = 0.60 m is the track gauge (as-built record, [EST]). chi is the skid-steer path-curvature factor (chi = 1 for the ideal oracle; the structure domain provides the physical value, `P.structure.track.chi_nom` = 1.394 on farm soil and 1.32 to 1.49 over the soils, D39). Implemented with `robotmobilelib/Differential Drive Kinematic Model` (`TrackWidth = P.structure.geom.B`, `WheelRadius = P.drive.gear.r_spr` = 0.090 m) in T-INT-04.

**(E2) Terrain grade in the body frame** (revision A's Env_Terrain, dropped by D10: the structure domain's environment and contact model own it):
- `z = Z(x,y)`, `z_x = dZ/dx`, `z_y = dZ/dy` (precomputed 2-D tables).
- `n_ENU = [-z_x, -z_y, 1] / sqrt(1 + z_x^2 + z_y^2)`
- `gamma_long = atan( z_x cos(psi) + z_y sin(psi) )`: positive means uphill ahead.
- `gamma_lat = atan( -z_x sin(psi) + z_y cos(psi) )`: positive means the left side is higher.
- On static terrain, `theta_static = -gamma_long` and `phi_static = +gamma_lat` (FLU convention, Section 3).

**(E3) Ambient temperature and solar profile** (Sine Wave blocks):
- `T_amb(t) = T_mean + A sin( 2 pi (t/3600 + h0 - h_pk + 6)/24 )`. With T_mean = 311.15 K, A = 9 K and h_pk = 15 h (`P.world.env.diurnal`, used when `P.world.env.use_diurnal` is true; the default is the constant 45 deg C design ambient `P.world.env.T_amb` = 318.15 K), this gives 29 to 47 deg C, peaking at 15:00 local.
- `G_solar(t) = max(0, G_pk sin( pi (t/3600 + h0 - 6)/12 ))` for daylight 06:00-18:00.

**(E4) Energy audit** (Sim_Monitors, T-INT-08):
- `E_batt = integral(V_term * I_batt dt)`
- `eps = | E_batt - SUM(E_loads) - SUM(E_loss) - (m g dz + 0.5 m dv^2) | / max(|E_batt|, 1 Wh)` (revision A form)
- Revision B (D40), from 1 ms plant integrals: `eps_el = |(E_out - E_in) + E_chg_bus - (E_trac_dc + E_impl_dc + E_lv + E_dcdc_loss + E_pdu_loss)| / max(E_out, 1 Wh)`
  (Bus_PowerTruth) and `eps_mech = |sum(E_spr) - E_diss_J - dE_mech| / max(sum(E_spr), 1 kJ)` (Bus_DriveSideTruth, Bus_StructureDiag).
- Pass if eps <= 1 %.

**(E5) RC end-to-end latency budget:** (revision B, D37: measured from TX frame emission to the DRV_CMD frame at the MCU output = 14 ms frame RT + 3 ms bytes + <= 1 ms poll + <= 10 ms Task_10 phase = 18-28 ms; mixing runs in Task_10, not in the 1 kHz task; ScenarioIn stick -> ESC reference 38-62 ms)
`t_lat = T_frame(14 ms, or 7 ms high-speed) + T_tx(25 bytes x 12 bit / 100 kbaud = 3.0 ms) + T_poll(<= 1 ms) + T_task(<= 1 ms if mixing runs in the 1 kHz task) + T_delay(1 ms compute delay) <= 20 ms`. Requirement: <= 30 ms.

**(E6) Simulation cost model:**
- `t_wall = N_base * (c_ssc + c_sl) + SUM_k N_k * c_k`, with `N_base = T_sim / Ts_base`.
- `RTF = T_sim / t_wall`.
- For RTF >= 20 at Ts_base = 1 ms, the average budget is <= 50 us of wall time per simulated millisecond (Section 7.7). This was revision A's design target; D56 re-baselined the targets to RTF >= 5 for S1 and RTF >= 4 for S3 (`P.sim.rtf_min_short`, `P.sim.rtf_min_mission`).

---

## 3. Coordinate frames, units and sign conventions (binding on all domains)

### 3.1 Frames

| Frame | Symbol | Definition |
|---|---|---|
| Geodetic | LLA | WGS-84 latitude/longitude [deg] and ellipsoidal height [m]. Field origin `P.world.lla0` = [24.1556, 47.3120, 430] (Al-Kharj palm region, KSA; scenario-overridable; `P.sensors.refLLA` is the same). |
| World / map | W (ENU) | Local tangent plane at `lla0`: x = East, y = North, z = Up. Right-handed. The map frame M equals W: occupancy maps use the same axes, with the map origin (lower-left) at `P.world.occ.origin` in W. Conversions: `lla2enu` / `enu2lla` (verified in `toolbox/shared/coordinates`). |
| Body | B (FLU) | x Forward, y Left, z Up (ROS REP-103). **Origin = "base_footprint"**: the center of the two tracks' contact footprint, projected onto the ground contact plane (z_B = 0 is the track bottom). CG position `P.structure.mp.rcg0_FLU` ([0.0076, 0, 0.267] m with no implement and the 1 kW units). |
| Track frames | L, R | Parallel to B, origins at `[0, +B/2, 0]` (left) and `[0, -B/2, 0]` (right), B = `P.structure.geom.B` = 0.60 m. |
| Sensor frames | S_imu, S_gnss1/2, S_lid2d, S_lid3d | FLU-aligned with B; the lever arms and any mounting rotation are the sensors domain's (sensors spec 7.1; no P leaf `P.sensors.R_BS_*` exists). |
| NED / FRD | - | **Not used in project signals.** Allowed only inside library blocks that require it, with an explicit conversion at the block boundary: `R_NED_ENU = [0 1 0; 1 0 0; 0 0 -1]`, `R_FRD_FLU = diag([1 -1 -1])`. |

### 3.2 Attitude and angle conventions
- Euler sequence **ZYX** (yaw psi, pitch theta, roll phi), frame rotation from W to B: `R_WB = Rz(psi) Ry(theta) Rx(phi)` maps body vectors to world vectors.
- **Yaw psi:** about +z_W (Up), measured from **East, counterclockwise positive**, wrapped to (-pi, pi]. psi = 0 faces East; psi = +pi/2 faces North.
- **Compass heading and GNSS course:** measured from North, clockwise, in [0, 360) deg. `heading_deg = mod(90 - rad2deg(psi), 360)`. Checked live: `gpsSensor` in ENU reports course 90 deg when moving East and 0 deg when moving North.
- **Pitch theta:** positive means **nose down** (FLU, about +y Left). Checked live: pitch +10 deg moves body x to `[0.985 0 -0.174]` in W. **Climbing a slope gives a negative theta.** Use the terrain grade `gamma_long` (positive uphill) in requirements and plots to avoid confusion.
- **Roll phi:** positive means the **left side is up** (right side down).
- **Quaternion:** MATLAB `quaternion` (Hamilton, scalar-first `[w x y z]`), orientation as a frame rotation W to B. `v_B = rotateframe(q, v_W)`, `v_W = rotatepoint(q, v_B)`, `eulerd(q,'ZYX','frame') = [psi theta phi]`. Checked live: for yaw +90 deg, `rotateframe(q,[0 1 0]) = [1 0 0]`.
- **Body rates:** `omega_B = [p q r]` in FLU. r > 0 means turning left (CCW seen from above).
- All internal angles are in rad. Parameters given in degrees must carry a `_deg` suffix and be converted inside `param_*.m`.

### 3.3 Motion, force, electrical and thermal signs

| Quantity | Positive direction |
|---|---|
| Track speed `v_L`, `v_R` and sprocket speed `w_L`, `w_R` | Moves the robot **forward** on that side. Physical motor rotation sense is mirrored per side inside `drive` (no P leaf names it). Every interface signal is forward-positive. |
| Turn | `v_R > v_L` gives omega > 0 (left turn). |
| Traction force / sprocket torque | Forward-accelerating the robot. |
| Encoder counts | Increase when that side moves forward. |
| Battery current `I_batt` | **Discharge positive** (out of the + terminal). Regen is negative. |
| Load power `P_x` | Consumption positive. |
| Heat flow into a thermal mass | Positive. Temperatures are in **K** in all signals; deg C is used only on dashboards. |
| Accelerometer (**MATLAB sensor-model convention**) | Checked live: `imuSensor`, `ReferenceFrame='ENU'`, at rest and level, reads `[0 0 -9.81]` (NED: `[0 0 +9.81]`). MATLAB's sensor and filter blocks use `a_meas = R_BW*(g - a)`, the **negative of the physical specific force**. Real IMUs (BNO055, ICM-20948, ...) report specific force (`+9.81` on the up axis at rest). **Rule (revision B, superseding revision A's):** the buses carry the physical specific force (`buses/shaddad_buses.m`; `Bus_ImuRaw` z at rest = +9.79 m/s^2, architecture section 12); the sensor models convert the MATLAB convention at their output. |
| Gyro | rad/s, FLU, right-handed. |
| Magnetometer | uT on the buses (`buses/shaddad_buses.m`). `P.frames.mag_ENU_uT` = [1.745, 34.116, -26.317] uT (WMM2025 at `lla0`, epoch 2026.8; declination +2.93 deg, inclination 37.6 deg, \|B\| = 43.1 uT; computed with `wrldmagm`). |
| Gravity | `P.frames.g` = 9.7876 m/s^2, the local value at the site (`gravitywgs84(430, 24.15)`). The MATLAB IMU model assumes 9.81 inside (`P.frames.imu_g_internal`). |
| Library blocks with **NED defaults** | `mspsensorlib/GPS` and `mspsensorlib/IMU` have `DispRefFrame = 'NED'` by default and **must be set to `'ENU'`**. `uavutilslib/UAV Animation` defaults to `InertialZDirection = 'Down'`. `aerolibtransform2/Flat Earth to LLA` is NED-only, so do not use it; use `enu2lla`. |

### 3.4 Units and naming
- SI everywhere (m, s, kg, N, N*m, rad, rad/s, V, A, W, J, K, Pa, T). Set the `Unit` property on **every** `Simulink.BusElement`. The top model contains a `Unit System Configuration` block with `UnitSystems = 'SI'`. Unit-mismatch diagnostics are set to `warning`.
- Signal names are lowerCamel or snake case (owner's choice) and **must not contain unit suffixes**; units live in the bus element `Unit`. Exceptions: `_deg` parameters and `_K` overrides in scenario structs.
- Every measurement or message bus contains `t_stamp` (double s, plant) or `t_ms` (uint32 ms, MCU), plus `seq` (uint16/uint32) and `valid` (boolean).
- Common mode enumeration (proposed; owner `mcu`): `ShaddadMode : uint8 {INIT=0, STANDBY=1, RC_MANUAL=2, AUTO=3, HOLD=4, FAILSAFE=5, ESTOP=6, FAULT=7}`, a `Simulink.IntEnumType` classdef in `buses/enums/ShaddadMode.m`. Integration displays it and uses it in monitors.

---

## 4. Exact block list (all paths checked in the live R2026b session)

Note on names: several library block names contain a newline. In `add_block`, Simulink also accepts the path with a single space in place of the newline, which was checked live for `nesl_utility/Solver Configuration`, `nesl_utility/Simulink-PS Converter`, `fl_lib/.../Controlled Temperature Source`, and `ee_lib/Electromechanical/Motor & Drive (System Level)`. The builder should still use `sprintf('...\n...')` for those marked (NL). Exception: `Check \nStatic Range` has a trailing space before the newline.

### 4.1 Structural / hierarchy

| # | Block | Library path | Used in | Key mask/dialog parameters (real names) and values |
|---|---|---|---|---|
| 1 | Model (Model Reference) | `simulink/Ports & Subsystems/Model` | Top: `MCU`, `Plant`; GNC_Variant choice 2 | `ModelName` = `'Shaddad_MCU'` / `'Shaddad_Plant'` / `'Shaddad_GNC'`; `SimulationMode` in {`Normal`,`Accelerator`,`Software-in-the-loop (SIL)`,`Processor-in-the-loop (PIL)`}; missions use `'Accelerator'`, debugging uses `'Normal'`; `CodeInterface = 'Model reference'`; `ScheduleRatesWith = 'Ports'` |
| 2 | Subsystem Reference | `simulink/Ports & Subsystems/Subsystem Reference` | Top: `Operator_RC`, `Sensors`, `Link_MCU_GNC`, `Viz_Logging`, `Sim_Monitors`. Plant model: `Env_Terrain`, `Structure_Tracks`, `Power_System`, `Drive_Train`, `Electronics_HW`, `Implements` | `ReferencedSubsystem` = file name without `.slx` (e.g. `'Power_System'`); `TreatAsAtomicUnit = 'off'` (virtual); `MinAlgLoopOccurrences = 'off'` |
| 3 | Variant Subsystem | `simulink/Ports & Subsystems/Variant Subsystem` | Top: `GNC_Variant`; Viz_Logging: `Viz_Variant`; Shaddad_MCU/GNC: `IO_In`, `IO_Out` (pattern) | `VariantControlMode = 'expression'` (enum: `expression/label/sim codegen switching`); `VariantActivationTime = 'update diagram'`; `PropagateVariantConditions = 'off'`; choice conditions such as `VC_PHASE == 1`, `VC_PHASE == 2` |
| 4 | Variant Source | `simulink/Signal Routing/Variant Source` | Operator_Console | `VariantControls = {'VC_OPSRC == 1','VC_OPSRC == 2'}`; `VariantControlMode='expression'`; `VariantActivationTime='update diagram'`; `AllowZeroVariantControls='off'` |
| 5 | In Bus Element | `simulink/Ports & Subsystems/In Bus Element` | Root `ScenarioIn`; all SSR/Model inputs | `PortName`, `OutDataTypeStr = 'Bus: Bus_ScenarioIn'`, `SampleTime = 'P.sim.Ts_scen'` (root), `BusVirtuality = 'inherit'`, `Interpolate = 'off'` |
| 6 | Out Bus Element | `simulink/Ports & Subsystems/Out Bus Element` | All SSR/Model outputs | `PortName`, `OutDataTypeStr = 'Bus: Bus_<Name>'` |
| 7 | Bus Creator / Bus Selector | `simulink/Signal Routing/Bus Creator`, `simulink/Signal Routing/Bus Selector` | Wiring, GNC_Stub | Bus Creator `OutDataTypeStr='Bus: Bus_GNC2MCU'`, `NonVirtualBus='on'` at Model-block inputs |
| 8 | Signal Specification | `simulink/Signal Attributes/Signal Specification` | In front of every Model block input | `OutDataTypeStr='Bus: Bus_<Name>'`, `SampleTime` = consumer rate |
| 9 | Unit System Configuration | `simulink/Ports & Subsystems/Unit System Configuration` | Top, Plant | `AllowAllUnitSystems = 'off'`, `UnitSystems = {'SI'}` (cell; default `{'SI','English','SI (extended)','CGS'}`) |
| 10 | DocBlock / Model Info | `simulink/Model-Wide Utilities/DocBlock`, `simulink/Model-Wide Utilities/Model Info` | Top, Plant | Documentation and version stamp |

### 4.2 Rates, data types, loop breaking

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 11 | Rate Transition | `simulink/Signal Attributes/Rate Transition` | `Integrity='on'`, `Deterministic='on'`, `OutPortSampleTimeOpt='Specify'`, `OutPortSampleTime='P.sim.Ts_xxx'`, `InitialCondition='0'` (or bus IC struct `P.ic.Bus_<Name>`). Placed in the `RT_MCU_In`, `RT_GNC_In`, `RT_GNC_Out`, `RT_Plant_In` wrappers and at consumer-side SSR inputs. |
| 12 | Unit Delay | `simulink/Discrete/Unit Delay` | Compute-latency delay on every controller output (`SampleTime` = controller task rate, `InitialCondition` = safe value); 1-step pose delay at the Env_Terrain input (`SampleTime='P.sim.Ts_base'`) |
| 13 | Data Type Conversion | `simulink/Signal Attributes/Data Type Conversion` | double to single/int at the MCU/GNC IO_In SIM choices |
| 14 | Zero-Order Hold | `simulink/Discrete/Zero-Order Hold` | Only for scalar holds where an RT is not needed |
| 15 | Ground / Terminator / Constant | `simulink/Sources/Ground`, `simulink/Sinks/Terminator`, `simulink/Sources/Constant` | Stubs; `GNC_Stub` constants `P.ic.<Bus>` on all outputs, with `comp_cmd.valid = false` (ICD 3.1) |

### 4.3 Simscape plumbing (inside `Shaddad_Plant.slx` and in the domain SSRs)

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 16 | Solver Configuration (NL) | `sprintf('nesl_utility/Solver\nConfiguration')` | **Exactly one per network, at Shaddad_Plant top level**, connected to the DC48 negative node. `UseLocalSolver='on'`; `LocalSolverChoice='NE_BACKWARD_EULER_ADVANCER'` (allowed: `NE_BACKWARD_EULER_ADVANCER/NE_TRAPEZOIDAL_ADVANCER/NE_NDF2_ADVANCER/NE_VARIABLE_STEP_ADVANCER/NE_PARTITIONING_ADVANCER/NE_BACKWARD_EULER_INTERVAL_ADVANCER`); `LocalSolverSampleTime='P.sim.ssc.Ts'` (1e-3); `DoFixedCost='on'`; `MaxNonlinIter='3'`; `MaxModeIter='2'`; `PartitionMethod='FAST'` if Partitioning is selected (enum `BALANCED/FAST/ALG_EQS/STABLE/MERGE/MULTICORE/ROBUST`); `EquationFormulation='NE_TIME_EF'`; `DoDC='off'`; `LinearAlgebra='auto'`; `ConsistencyTolSource='GLOBAL'`; `ResidualTolerance='1e-9'` |
| 17 | Connection Port | `nesl_utility/Connection Port` | In every SSR file with physical ports: `Port`, `Side` (`Left`/`Right`), `ConnectionType` explicit: `'Connection: foundation.electrical.electrical'` (verified), `'Connection: foundation.mechanical.rotational.rotational'` and `'Connection: foundation.thermal.thermal'` (same naming scheme; confirm in Stage 1 smoke test) |
| 18 | Electrical Reference | `fl_lib/Electrical/Electrical Elements/Electrical Reference` | One on the DC48 negative node at Plant top; domains may add local ones |
| 19 | Mechanical Rotational Reference (NL) | `sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference')` | Used by domains for gearbox housings; none at the top |
| 20 | Simulink-PS Converter (NL) | `sprintf('nesl_utility/Simulink-PS\nConverter')` | For discrete command inputs into local-solver networks: `FilteringAndDerivatives='zero'`; where a derivative is needed: `'filter'`, `SimscapeFilterOrder='1'`, `InputFilterTimeConstant='P.sim.ssc.tau_filt'` (2e-3); `Unit` explicit |
| 21 | PS-Simulink Converter (NL) | `sprintf('nesl_utility/PS-Simulink\nConverter')` | `Unit` explicit SI, `VectorFormat='inherit'` |
| 22 | Controlled Temperature Source (NL) | `sprintf('fl_lib/Thermal/Thermal Sources/Controlled\nTemperature Source')` | Each physical SSR makes its own ambient node from the `Bus_Terrain.T_amb` signal. **No thermal ports cross SSR boundaries** (keeps networks decoupled and files independent). |
| 23 | Simscape Bus (optional) | `nesl_utility/Simscape Bus` | Optional bundling of `DC48_p/DC48_n`. Not used in v1 (plain ports are clearer to debug). |
| 24 | Probe | `nesl_utility/Probe` | Debug only; key variables must be exported in the domain `*Meas` buses |

### 4.4 Environment and terrain (revision A's `Env_Terrain.slx`, dropped by D10)

D10 dropped this file: the environment is the structure domain's `structure_env` (`scripts/build/build_structure_env.m`, `Bus_EnvState`). The rows below record revision A's design; their leaves `P.scen.terrain.*` and `P.scen.obs_static` do not exist (the terrain is `P.world.terrain.xE`, `yN`, `H`, `dHdE`, `dHdN`).

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 25 | 2-D Lookup Table | `simulink/Lookup Tables/2-D Lookup Table` | Four instances: `Z`, `Zx`, `Zy` (`InterpMethod='Linear point-slope'`, `ExtrapMethod='Clip'`), and `SoilID`/`GnssZone` (`InterpMethod='Nearest'`, class maps). Allowed `InterpMethod` values: `Flat/Nearest/Linear point-slope/Linear Lagrange/Cubic spline/Akima spline`; `ExtrapMethod`: `Clip/Linear/Cubic spline`. `BreakpointsForDimension1='P.scen.terrain.xg'`, `BreakpointsForDimension2='P.scen.terrain.yg'`, `Table='P.scen.terrain.Z'` etc.; `IndexSearchMethod='Evenly spaced points'`; `BeginIndexSearchUsingPreviousIndexResult='on'`; `DiagnosticForOutOfRangeInput='Warning'`; `SampleTime='-1'` |
| 26 | Direct Lookup Table (n-D) | `simulink/Lookup Tables/Direct Lookup Table (n-D)` | Soil and GNSS-zone property tables indexed by class id (0-based: `id-1`) |
| 27 | Sine Wave | `simulink/Sources/Sine Wave` | `SineType='Time based'`, `TimeSource='Use simulation time'`, `Amplitude='P.world.env.diurnal.T_amp'`, `Bias='P.world.env.diurnal.T_mean'`, `Frequency='2*pi/86400'`, `Phase='2*pi*(P.world.env.diurnal.t0_hour - P.world.env.diurnal.h_peak + 6)/24'`, the environment rate (`Ts.env`, 0.1 s); a second instance for `G_solar`. Built by `scripts/build/build_structure_env.m`: the environment is the structure domain's (OQ-02) |
| 28 | Trigonometric Function, Product, Sum, Math Function, Saturation, MinMax | `simulink/Math Operations/Trigonometric Function`, `.../Product`, `.../Sum`, `.../Math Function`, `simulink/Discontinuities/Saturation`, `simulink/Math Operations/MinMax` | Equations (E2) and (E3) |
| 29 | Sort | `dspstat3/Sort` (DSP System Toolbox) | Nearest-K obstacle selection: `otype='Index'`, `dir='Ascending'` (squared distances from robot to `P.scen.obs_static` at 0.1 s) |
| 30 | Selector | `simulink/Signal Routing/Selector` | Takes the first 12 indices (the nearest-K count of revision A; no P leaf: the obstacle sets are `P.sensors.env` and `P.sim.mon`) |
| 31 | Vector Concatenate | `simulink/Math Operations/Vector Concatenate` | Static + dynamic obstacle merge |
| 32 | Coordinate Transformation Conversion | `navlib/Utilities/Coordinate Transformation Conversion` | `InputRepresentation='Euler Angles'`, `InputEulerSequence='ZYX'`, `OutputRepresentation='Homogeneous Transformation'`, `ShowTrVecInputPort='on'`, `SimulateUsing='Code generation'` (robot pose to 4x4 for collision geometry) |

### 4.5 Monitors (`Sim_Monitors.slx`)

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 33 | Collision Box | `robotslcollision/Collision Box` | Inputs X, Y, Z, Pose: the footprint `P.sim.mon.footprint` (1.00 x 0.78 m, from `P.structure.geom`), `Z` = `P.sim.mon.z_robot` (1.1 m), `Pose` from #32; `SimulateUsing='Code generation'` |
| 34 | Collision Cylinder | `robotslcollision/Collision Cylinder` | Inputs Radius, Length, Pose: obstacle radius, 2.0 m, pose `[I, [x y 1]']` |
| 35 | Check Collision | `robotslcollision/Check Collision` | Inputs Geom1, Geom2; outputs CollStatus, SepDist, WitnessPts; `SimulateUsing='Code generation'` |
| 36 | For Each Subsystem | `simulink/Ports & Subsystems/For Each Subsystem` | Wraps #34 and #35 over the nearest `P.sim.mon.Kc` cylinders (12), `Kb` boxes (8) and `Nd` dynamic obstacles (4) at `P.sim.Ts_monitor` (0.1 s) |
| 37 | Discrete-Time Integrator | `simulink/Discrete/Discrete-Time Integrator` | `E_batt`, `E_regen`, distance travelled; `SampleTime='P.sim.Ts_1k'`, `IntegratorMethod='Integration: Forward Euler'` (Ts 1 ms) |
| 38 | Relational Operator | `simulink/Logic and Bit Operations/Relational Operator` | `Operator='isNaN'` (enum includes `isInf isNaN isFinite`) on truth states (NaN guard); `'<='` for the SoC floor |
| 39 | Logical Operator | `simulink/Logic and Bit Operations/Logical Operator` | OR of stop conditions |
| 40 | Assertion | `simulink/Model Verification/Assertion` | `Enabled='on'`, `StopWhenAssertionFail='on'` (NaN guard; collision when `P.scen.stop_on_collision`), `SampleTime='P.sim.Ts_10'` |
| 41 | Stop Simulation | `simulink/Sinks/Stop Simulation` | Mission done / SoC floor / wall-time guard |
| 42 | Check Static Range (NL with trailing space) | `sprintf('simulink/Model Verification/Check \nStatic Range')` | Range sanity on SoC [0,1], cell temperature [250, 360] K (`P.sim.mon.T_cell_rng`), `\|v\|` <= 3 m/s (`P.sim.mon.v_abs_max`); `enabled='on'`, `stopWhenAssertionFail='off'` (logs warnings) |
| 43 | Digital Clock | `simulink/Sources/Digital Clock` | `SampleTime='P.sim.Ts_10'` for the mission timer |

### 4.6 Visualization, logging, dashboards

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 44 | MATLAB System (bird's-eye view) | `simulink/User-Defined Functions/MATLAB System` | `System='shaddad.BirdsEyeView'` (Section 5), `SimulateUsing='Interpreted execution'`, sample time 0.1 s through the object's `SampleTime` property; inside `Viz_Variant` choice `VC_VIZ == 1` |
| 45 | XY Graph | `simulink/Sinks/XY Graph` | `NumPorts` 2 (x_ENU, y_ENU truth) and 2 (estimate); `Decimation='1'` after RT at 0.1 s; choice `VC_VIZ == 2` |
| 46 | Record | `simulink/Sinks/Record` | Interactive runs only: `NumPorts`, `Layout` (e.g. 2x2 grid), `Decimation='1'`, `MaxDataPoints='inf'` |
| 47 | Scope | `simulink/Sinks/Scope` | Unit benches only; **not** in mission configurations |
| 48 | Dashboard Gauge / Half Gauge / Linear Gauge | `simulink_hmi_blocks/Gauge`, `simulink_hmi_blocks/Half Gauge`, `simulink_hmi_blocks/Linear Gauge` | `Binding` to the logged signals `v_meas` (Limits [-2.5 2.5] m/s), `SoC` (0..100 %), `T_cell_max` (shown in deg C); `ScaleColors` green/amber/red |
| 49 | Lamp / MultiStateImage | `simulink_hmi_blocks/Lamp`, `simulink_hmi_blocks/MultiStateImage` | `Binding` to `mcu_status.mode`; `StateColors` per `ShaddadMode` |
| 50 | Slider / Knob / Toggle Switch / Rotary Switch / Push Button | `simulink_hmi_blocks/Slider`, `.../Knob`, `.../Toggle Switch`, `.../Rotary Switch`, `.../Push Button` | Interactive RC: bound to `Constant` blocks in `Operator_Console/Manual` (thr, steer `Limits [-1 1]`; `sw_arm`, `sw_mode` `States` 0/1/2, `sw_estop`) |
| 51 | Dashboard Scope | `simulink_hmi_blocks/Dashboard Scope` | `TimeSpan='30'`, v_cmd vs v_meas, w_cmd vs w_meas |
| 52 | Heading Indicator (optional) | `aerolibhmi/Heading Indicator` | Compass heading (deg) from psi |
| 53 | UAV Animation (evaluated, **not used**) | `uavutilslib/UAV Animation` | Exists (`UAVType` Multirotor/FixedWing, `InertialZDirection` Up/Down, `RotationFormat` Quaternion/Euler ZYX). Rejected: it draws a UAV mesh in 3D with no field map, path or footprint. |

### 4.7 Test infrastructure

| # | Block | Library path | Key parameters |
|---|---|---|---|
| 54 | Test Sequence | `sltestlib/Test Sequence` | Stimulus in harnesses (step and when-based transitions) |
| 55 | Test Assessment | `sltestlib/Test Assessment` | `verify()` statements with the numeric criteria of Section 9 |
| 56 | Differential Drive Kinematic Model | `robotmobilelib/Differential Drive Kinematic Model` | Frame oracle in T-INT-04: `VehicleInputs='Wheel Speeds'`, `WheelRadius='P.drive.gear.r_spr'` (0.090 m), `TrackWidth='P.structure.geom.B'` (0.60 m), `InitialState='[0;0;0]'`, `SimulateUsing='Code generation'` |
| 57 | Chart (Stateflow) | `sflib/Chart` | Not used by integration (MCU and GNC mode logic only) |

Pattern references for other domains (checked to exist, owned by `mcu`/`gnc`): `stm32blockslib/UART//USART Read` (`UARTModule, DataType, DataLength, BlockingMode, BlockingTimeout, OutputLength, OutputStatus, SampleTime`), `stm32blockslib/FDCAN Read` and `FDCAN Write`, `stm32blockslib/Encoder` (`TimerModule, EnableResetInput, OutputDirection, SampleTime`), `stm32blockslib/PWM Output`, `stm32blockslib/Digital Port Read`, `stm32blockslib/Analog to Digital Converter`, `roslib/ROS 2` (sublibrary). Sensor models: `mspsensorlib/GPS`, `mspsensorlib/IMU`, `mspsensorlib/INS` (**set `DispRefFrame='ENU'`**).

---

## 5. MATLAB Function / MATLAB System blocks (integration-owned)

Integration builds Env_Terrain, Sim_Monitors and logging from library blocks, with two exceptions: the live bird's-eye view (5.1) and the separation functions of the collision monitors (5.3, D70).

### 5.1 `shaddad.BirdsEyeView` (MATLAB System, Interpreted execution, simulation-only)
**Why no library block works.** No installed block draws a ground robot's oriented footprint over a 2D field map together with the planned path, the LiDAR returns and the trail:
- `uavutilslib/UAV Animation` is a 3D UAV mesh with no map.
- `simulink/Sinks/XY Graph` shows only points and lines: no footprint, heading, map raster or scan.
- The Automated Driving Bird's-Eye Scope is an app tied to driving-scenario actors.
- Robotics `Simulation 3D Robot` needs Unreal Engine and is far too heavy.

`XY Graph` is kept as the cheap live alternative (`VC_VIZ = 2`).

- File: `utilities/+shaddad/BirdsEyeView.m` (System object). The offline `utilities/shaddad_replay_birdseye.m` reuses the same class, so live and replay drawing share one code path.
- Placement: `Viz_Logging/Viz_Variant/BEV` (choice `VC_VIZ == 1`). Disabled (`VC_VIZ = 0`) in all batch, accelerator-heavy and rapid-accelerator runs. Interpreted MATLAB System blocks are not supported in rapid accelerator.
- Sample time: 0.1 s (`P.viz.Ts`). Inputs pass through Rate Transitions to 0.1 s.

```matlab
classdef BirdsEyeView < matlab.System
  % Live 2D bird's-eye view of the Shaddad UGV (ENU, map frame = world).
  properties (Nontunable)
    MapImage      = false(600,1200)   % logical occupancy grid (rows = y, cols = x)
    MapResolution = 10                % cells per metre (P.world.occ.res)
    MapOrigin     = [0 0]             % [x y] of lower-left cell in ENU [m]
    Footprint     = [1.0 0.78]        % [L W] m (P.viz.footprint)
    TrailLength   = 20000             % points (P.viz.trail_len)
    ScanDecim     = 4                 % plot every n-th beam
    SampleTime    = 0.1
  end
  properties (Access = private)
    hFig; hAx; hBody; hTrail; hEst; hPath; hScan; hObs; hTxt; buf; k
  end
  methods (Access = protected)
    function num = getNumInputsImpl(~), num = 7; end
    % u1 pose_true [x;y;psi] (3x1 double)   u2 pose_est [x;y;psi] (3x1)
    % u3 path_xy (Np x 2 double, NaN-padded)
    % u4 scan_xy_B (Ns x 2 single, body frame, NaN = no return)
    % u5 obs_xyr (16 x 3 double, NaN rows unused)   u6 mode (uint8)  u7 soc (double)
    function sts = getSampleTimeImpl(obj)
      sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.SampleTime);
    end
    function setupImpl(obj)
      obj.hFig = figure('Name','Shaddad bird''s-eye','NumberTitle','off');
      obj.hAx  = axes(obj.hFig); hold(obj.hAx,'on'); axis(obj.hAx,'equal');
      % map raster: imagesc with XData/YData in metres, YDir normal
      % body: patch(L x W rectangle + heading triangle) parented to hgtransform obj.hBody
      % trail/est/path/scan/obs: preallocated line/scatter objects (NaN data)
      obj.buf = nan(obj.TrailLength,2); obj.k = 0;
    end
    function stepImpl(obj,pT,pE,path,scanB,obs,mode,soc)
      if ~isvalid(obj.hFig), return; end           % user closed the window
      obj.hBody.Matrix = makehgtform('translate',[pT(1) pT(2) 0],'zrotate',pT(3));
      obj.k = mod(obj.k, obj.TrailLength) + 1; obj.buf(obj.k,:) = pT(1:2)';
      % update trail (circular buffer), estimate marker, path line (drop NaN)
      % scan: rotate body points by psi, translate by [x y], plot every ScanDecim-th
      % obstacles: circles via a single line object with NaN separators
      % title text: t, mode name (ShaddadMode), SoC %, |e_pos| = norm(pT(1:2)-pE(1:2))
      drawnow limitrate                              % caps at about 20 fps
    end
    function releaseImpl(~), end                     % keep the figure for inspection
  end
end
```

### 5.2 Offline functions (MATLAB scripts, not blocks)

| Function | Why it is offline and not a block |
|---|---|
| `utilities/kpi_coverage.m(logsout, P)` | Swept-area coverage percentage (rasterize the implement swath polygon along the logged pose, 10 Hz, on a 0.05 m grid). No block computes polygon-swept coverage, and doing it online at 1 kHz would cost about 5-10 % of RTF. Post-processing takes about 1 s per hour of mission. |
| `utilities/kpi_truth_xte.m(logsout, P)` | Truth cross-track error against the planned path logged from GNC. The path is variable and evaluation is cheaper offline. |
| `utilities/eval_criteria.m(out, crit)` | Evaluates the pass/fail tables of Section 9 on `out.logsout`. |
| `utilities/shaddad_replay_birdseye.m(file)` | Replays a saved run with `shaddad.BirdsEyeView`, faster than real time, with optional `VideoWriter` MP4 export. |

### 5.3 `mon_sep_circle`, `mon_sep_rect` (MATLAB Function, simulation-only, D70)
**Why a function.** The planar separation of the footprint from one obstacle row was a network of element-wise blocks in each For Each of `Sim_Monitors` (26 blocks for a circle, 73 for a box): pure single-rate logic, which one function states more clearly (D70). The functions do the same operations in the same order, so every output stays bit-identical (`tests/integration/t_mf_equiv`).

- Files: `subsystems/integration/mon_sep_circle.m` (rows `[x y r]`) and `mon_sep_rect.m` (rows `[x y yaw hx hy]`), both `%#codegen`.
- Placement: the MATLAB Function `Sep` in each For Each Subsystem of `Sim_Monitors` (`FE_Cyl`, `FE_Dyn`: circle; `FE_Box`: box), called in one line, `sep = mon_sep_*(row, p, R, hA)`: `p` and `R` the footprint centre and rotation (body to world), `hA` the footprint half extents from the Constant `P.sim.mon.footprint(:)/2` on a fourth input (no parameter data: in R2026b it trips the Stateflow JIT relink assertion inside Subsystem References). The For Each's outport `sep` keeps the dimensions `[1 1]`, so its output stays a K x 1 matrix as before. The block holds a verbatim copy of the file as a local function of the same name, written by `build_sim_monitors`: with the external call, the first compile after a load hit the R2026b Stateflow JIT relink assertion `'o != nullptr'` (the remedy of `build_structure`). The file stays the single source; rebuild after editing it (`t_mf_equiv/testMonitorsUseFunctions` checks that the copies are current).
- Result: the separation in m, negative when they overlap. Circle: the signed distance of the centre to the footprint rectangle, minus r. Box: apart, the smallest signed distance of a corner of either rectangle to the other; overlapping (every separating-axis margin >= 0), minus the smallest margin.

---

## 6. Interfaces

### 6.1 Root input of `Shaddad_System`: `ScenarioIn` (`Bus_ScenarioIn`, Ts = `P.sim.Ts_scen` = 0.01 s, ZOH)
It is driven by `Simulink.SimulationInput.setExternalInput(ds)`, where `ds` is a `Simulink.SimulationData.Dataset` with one element built by `Simulink.SimulationData.createStructOfTimeseries('Bus_ScenarioIn', ...)`. With no external input the root port reads zeros, which is a safe idle: key on, no E-stop pressed, transmitter off, no faults, no obstacles.

The element lists below are those of `buses/buses_sim.m`, which bind, as ICD section 10 prints them (card G1-01 part 2, step 19: revision A's names `thr`, `steer`, `speed_knob`, `tx_on`, `estop_hw` and `link_mcu_gnc_loss` are gone, and the body-mounted inputs are `hmi`).

**`Bus_ScenarioIn`** (owner: integration)

| Element | Type | Dim | Unit | Description |
|---|---|---|---|---|
| `op` | `Bus: Bus_OperatorCmd` | 1 | - | Operator intent |
| `hmi` | `Bus: Bus_HmiInputs` | 1 | - | Body-mounted E-stops, bumpers, key, reset |
| `flt` | `Bus: Bus_FaultInject` | 1 | - | Fault injection |
| `obs` | `Bus: Bus_ObstaclesDyn` | 1 | - | Dynamic obstacles |
| `env` | `Bus: Bus_EnvOverride` | 1 | - | Environment overrides |
| `mcmd` | `Bus: Bus_MissionCmd` | 1 | - | Ground-station mission command (GNC) |

`Bus_MissionCmd` is the GNC domain's (`buses/buses_gnc.m`).

**`Bus_OperatorCmd`** (owner: integration; consumer: `rc` `RC_Link_Sim`, which maps intent to SBUS channels via `P.rc.ch_map`)

| Element | Type | Unit | Range | Meaning |
|---|---|---|---|---|
| `stick_thr` | double | 1 | [-1, 1] | Throttle stick, + = forward |
| `stick_steer` | double | 1 | [-1, 1] | Steering stick, + = stick RIGHT = turn right (omega < 0) |
| `stick_aux_lv` | double | 1 | [-1, 1] | Left stick vertical (spare) |
| `stick_aux_lh` | double | 1 | [-1, 1] | Left stick horizontal (spare) |
| `knob_speed` | double | 1 | [-1, 1] | Speed-scale knob S1: -1 = 25 % (`P.electronics.rcmap.s_min` = 0.25), +1 = 100 % |
| `sw_mode` | double | - | [0, 3] | CH5 SA 3-pos: 1 MANUAL, 2 HOLD, 3 AUTO (0 treated as 1) |
| `sw_arm` | double | - | [0, 1] | CH6 SF arm switch: 0 disarmed, 1 armed |
| `sw_impl` | double | - | [0, 3] | CH8 SB implement 3-pos: 1 OFF, 2 ON, 3 ON + auto-spray |
| `sw_lift` | double | - | [0, 3] | CH11 SC lift 3-pos: 1 UP, 2 HOLD, 3 DOWN |
| `sw_lights` | double | - | [0, 1] | CH10 SD work lights: 0 off, 1 on |
| `sw_estop` | double | - | [0, 1] | CH12 SE remote (software) E-STOP: 0 run, 1 STOP |
| `btn_reset` | double | - | [0, 1] | CH9 SH momentary fault/E-stop reset: 1 pressed |
| `sw_mission` | double | - | [0, 3] | CH13 SG 3-pos momentary: 1 none, 2 START/RESUME, 3 PAUSE |
| `btn_rth` | double | - | [0, 1] | CH14 SI momentary return-to-home: 1 pressed |
| `tx_power_on` | double | - | [0, 1] | Transmitter powered: 0 off (no RF frames), 1 on |

**`Bus_HmiInputs`** (owner: integration; consumers: `Power_Plant/Safety_Chain`, `Electronics_Plant/HW_Frontend_Sim`): the hard-wired operator inputs on the robot body; all false = normal operation.

| Element | Type | Meaning |
|---|---|---|
| `estop_front_pressed` | boolean | Front mushroom E-stop pressed |
| `estop_rear_pressed` | boolean | Rear mushroom E-stop pressed |
| `wireless_estop_tripped` | boolean | Certified wireless E-stop (optional) tripped |
| `bumper_front` | boolean | Front bumper switch actuated |
| `bumper_rear` | boolean | Rear bumper switch actuated |
| `reset_btn` | boolean | Safety-relay reset button pressed (rising edge re-arms) |
| `key_off` | boolean | Key switch OFF (false = key ON) |
| `lid_open` | boolean | Electronics enclosure lid open |

**`Bus_FaultInject`** (owner: integration; consumers: `RC_Link_Sim`, `Shaddad_SensorModels`, `Shaddad_Plant`, `CAN1_Net_Sim`, `Link_MCU_GNC`, `MCU_In`)

| Element | Type | Unit | Meaning |
|---|---|---|---|
| `rc_link_loss` | boolean | - | RF frames lost (receiver sets frame_lost, then failsafe after P.rc.rx_failsafe_time) |
| `rc_line_silent` | boolean | - | Receiver SBUS output stops (no bytes) |
| `rc_byte_err_rate` | double | 1 | Probability of a corrupted SBUS byte |
| `gnss_outage` | boolean | - | No GNSS fix (fixType 0, newData false) |
| `gnss_force_float` | boolean | - | RTK fixed forced to float |
| `gnss_corr_loss` | boolean | - | NTRIP / RTK correction link lost (corrAge grows) |
| `imu_fault` | uint8 | - | IMU fault: 0 none, 1 stuck, 2 bias step, 3 dropout (invalid) |
| `lidar_blackout` | boolean | - | LiDAR returns no points (valid false) |
| `enc_fault_L` | boolean | - | Left motor encoder lost (ESC ENCODER fault) |
| `enc_fault_R` | boolean | - | Right motor encoder lost |
| `motor_fault_L` | boolean | - | Left drive trips (tau_scale = 0, overcurrent fault) |
| `motor_fault_R` | boolean | - | Right drive trips |
| `brake_stuck_L` | boolean | - | Left parking brake stuck engaged |
| `brake_stuck_R` | boolean | - | Right parking brake stuck engaged |
| `estop_ch2_stuck` | boolean | - | E-stop channel 2 contact welded closed (discrepancy test) |
| `bms_fault` | boolean | - | +0.5 V on the measured voltage of cell group 1, so the BMS trips on over-voltage (`P.power.bms.V_ov` 4.25 V, qualified over `P.power.bms.qual.OV` = 1) and opens the contactor |
| `can1_drop_mask` | uint32 | - | CAN1 frame drop: b0 DRV_CMD_L b1 DRV_CMD_R b2 FB1_L b3 FB1_R b4 FB3_L b5 FB3_R b6 FB2_L b7 FB2_R b8 IMPL_CMD b9 BMS_REQ b10 IMPL_FB b11 all BMS frames |
| `can2_loss` | boolean | - | All CAN-FD frames MCU<->companion lost |
| `can1_busoff` | boolean | - | MCU FDCAN1 bus-off: MCU_In sets fault_inject b4, CAN1_Net_Sim drops all MCU-originated CAN1 frames |
| `can2_busoff` | boolean | - | MCU FDCAN2 bus-off: MCU_In sets fault_inject b5, Link_MCU_GNC drops all CAN-FD frames |
| `task_stall` | uint8 | - | MCU task alive-counter stall: b0 10 ms task, b1 50 ms task, b2 100 ms task, b3 force overrun |
| `dust_level` | double | 1 | Airborne dust level for LiDAR clutter |

**`Bus_ObstaclesDyn`** (owner: integration; consumers: `Shaddad_SensorModels`, `Sim_Monitors`): `n` uint8 (0..4); `xy` double [4x2] m (ENU); `v_xy` double [4x2] m/s; `r` double [4x1] m.

**`Bus_EnvOverride`** (owner: integration; consumer: `structure_env`): `use_T` boolean; `T_amb` double K; `use_G` boolean; `G_solar` double W/m^2; `use_soil` boolean; `soil_id` uint8; `dock_contact_force` boolean.

### 6.2 Integration-produced buses

D10 dropped revision A's `Env_Terrain` and its buses (`Bus_Terrain`, `Bus_EnvSlow`, `Bus_ObstacleSet`): the environment is the structure domain's `structure_env`, the single producer of `Bus_EnvState` (0.1 s; `buses/buses_structure.m`), the static obstacles are `P.world.obstacles` and the dynamic ones `Bus_ObstaclesDyn`. Integration produces `Bus_ScenarioIn` with its parts (6.1) and `Bus_SimKPI`; ICD section 10 lists every bus.

**`Bus_SimKPI`** (`Sim_Monitors`, Ts = 0.1 s; logged)

| Element | Type | Unit | Meaning |
|---|---|---|---|
| `t_mission` | double | s | Time since McuStatus.mode first became AUTO |
| `dist` | double | m | Integral of \|v\| of point R (truth) |
| `E_batt` | double | J | Net battery terminal energy = 3600*(power_truth.E_out_Wh - E_in_Wh), discharge positive |
| `E_regen` | double | J | Charge energy into the pack = 3600*power_truth.E_in_Wh |
| `eps_el` | double | 1 | Electrical energy-audit residual (architecture D40) |
| `eps_mech` | double | 1 | Mechanical energy-audit residual (architecture D40) |
| `clear_min` | double | m | Minimum footprint-to-obstacle separation so far |
| `collision` | boolean | - | Any collision now |
| `n_coll` | uint16 | - | Number of collision onsets |
| `soc_min` | double | 1 | Running minimum true pack SOC |
| `T_cell_max` | double | K | Running maximum true cell temperature |
| `I_batt_peak` | double | A | Running maximum \|I_pack\| at 1 ms (= power_truth.I_pack_peak_A) |
| `out_of_map` | boolean | - | Point R outside the DEM extent |
| `stop_reason` | uint8 | - | 0 running, 1 StopTime, 2 mission done, 3 SoC floor, 4 collision, 5 NaN, 6 assertion, 7 wall-time |

### 6.3 Top-level signal interface (integration wiring contract)

ICD section 3.1 binds: the root port `ScenarioIn` with its Signal Specification, the 11 components joined by direct links (28 links between components, and `scen` to 9 of them), each component's input ports named after the buses it takes, its picks and the rates at each port. Revision A's table of this section named buses that the model does not have (`Bus_VehicleTruth`, `Bus_PowerMeas`, `Bus_DriveMeas`, `Bus_ElecIO`, `Bus_ImplMeas`, `Bus_ActCmd`, `Bus_SensMCU`, `Bus_SensGNC`, `Bus_SBUSLine`, `Bus_MCUStatus`, `Bus_GNCStatus`, `Bus_GNC2MCU`, `Bus_MCU2GNC`); it is dropped in favour of ICD section 3, where the buses are, for example, `Bus_TruthState`, `Bus_McuStatus`, `Bus_GncStatus`, `Bus_SbusStream` and the CAN frames `Bus_CanFrame64` (card G1-01 part 2, step 19).

### 6.4 Minimum elements integration requires from other domains (consumer contract)

`Sim_Monitors`, `Viz_Logging` and the scenario criteria read the elements that their picks in ICD section 3.1 name (for example `plant.truth`, `plant.power_truth`, `plant.drive_truth`, `plant.struct_diag`, `mcu.telem`, `gnc.gnc_status`, `gnc.nav_state`), with the element lists of ICD section 10; each bus's owner keeps them. Revision A's table of required elements is dropped with 6.3's (card G1-01 part 2, step 19).

### 6.5 Physical (Simscape) connections, all inside `Shaddad_Plant.slx`

ICD section 2 binds; this table repeats it (card G1-01 part 2, step 19, from D7, D15 and D16).

| Connection label | Domain (ConnectionType) | SSR port to SSR port | Notes |
|---|---|---|---|
| `DC48_TracL_p`, `DC48_TracL_n` | `Connection: foundation.electrical.electrical` | `Power_Plant` <-> `drive_system` | The left ESC's DC link after the fuse F_TL: the traction bus, 46.8 V nominal (13S NMC, 39.0 to 54.6 V; `P.power.pack.V_nom`). The **Solver Configuration** at the `Shaddad_Plant` top sits on `DC48_TracL_n` (D15). |
| `DC48_TracR_p`, `DC48_TracR_n` | electrical | `Power_Plant` <-> `drive_system` | The right ESC's DC link. |
| `Spr_L`, `Spr_R` | `Connection: foundation.mechanical.rotational.rotational` | `drive_system` (gearbox output) <-> `structure_plant` (drive sprockets) | Forward-positive sign convention applies at the port. |
| `Impl_Shaft` | rotational | `Power_Plant` (implement motor shaft) <-> `structure_plant` (blade, rotor or pump load) | D9; with no implement, `Impl_None` with its network tie (ICD 2, D35). |
| (none) | thermal | - | **Rule:** no thermal port crosses an SSR boundary. Ambient comes from the `Bus_EnvState.T_amb` signal (`structure_env`) through a local Controlled Temperature Source. |

The 24, 12, 5 and 3.3 V rails sit inside `Power_Plant/LV_Distribution`, and the electronics have no physical port (D7); the single Electrical Reference of the traction and LV network is inside `Power_Plant/PDU` at the GND node after the shunt (D15).

Rules for Simscape across Subsystem Reference boundaries:
1. Inside an SSR file, physical ports are `nesl_utility/Connection Port` blocks. `Side`: Left = sources/upstream, Right = loads/downstream. `ConnectionType` is explicit, never `Inherit: auto`, so a wrong-domain connection fails at compile time. Port names exactly as in ICD section 2.
2. **No `Solver Configuration` inside an SSR that has external physical ports.** Only `Shaddad_Plant` has one. An SSR that contains an internal network with no external physical port must contain its own Solver Configuration.
3. Unit benches (`tests/<domain>/tb_<Name>.slx`) add their own Solver Configuration, references and ideal sources/loads.
4. Model blocks never carry physical connections. Any physical coupling between plant pieces must stay inside `Shaddad_Plant`.
5. Every SSR exposes the physical quantities needed by other domains, logging and tests as Simulink signals in its `*Meas` bus. Simscape logging is off in missions (Section 7.6).

---

## 7. Internal structure

### 7.1 File tree and model types

```
models/
  Shaddad_System.slx        TOP (integration)                        - top model, no Simscape
  Shaddad_Plant.slx         Model Reference (integration wiring)     - all Simscape networks (ICD 3.2)
  Shaddad_MCU.slx           Model Reference (mcu)                    - IO_In/IO_Out Variant Subsystems + Model block -> Shaddad_MCU_App
  Shaddad_MCU_App.slx       Model Reference (mcu)                    - pure firmware logic; SIL/PIL unit
  Shaddad_GNC.slx           Model Reference (gnc)                    - IO variants (SIM / ROS 2) + Model block -> Shaddad_GNC_App
  Shaddad_GNC_App.slx       Model Reference (gnc)                    - pure GNC logic
subsystems/
  integration/   (NEW folder - add to project path)
    (Env_Terrain.slx: dropped by D10; the environment is structure_env)
    Viz_Logging.slx         SSR  (top)
    Sim_Monitors.slx        SSR  (top)
  structure/Structure_Tracks.slx, structure/Implements.slx (proposed owner, OQ-01)
  power/Power_System.slx
  drive/Drive_Train.slx
  electronics/Electronics_HW.slx, electronics/Link_MCU_GNC.slx
  sensors/Sensors.slx
  rc/Operator_RC.slx
params/   param_<domain>.m (each returns a struct; integration: param_sim.m), shaddad_params.m,
          shaddad_configsets.m
buses/    buses_<domain>.m (each returns a struct of Simulink.Bus; integration: buses_sim.m),
          shaddad_buses.m (assembles, checks, assigns in base), enums/ShaddadMode.m
scripts/  shaddad_startup.m, shaddad_shutdown.m, run_scenario.m, run_all_scenarios.m,
          build_mcu.m, build_gnc.m, clean_work.m,
          scenarios/scn_S0_smoke.m ... scn_S8_estop.m, scn_base_input.m
utilities/+shaddad/BirdsEyeView.m, shaddad_replay_birdseye.m, eval_criteria.m, kpi_coverage.m,
          kpi_truth_xte.m, shaddad_check_interfaces.m, shaddad_icd_report.m, scn_make_input.m
tests/    run_regression.m, integration/t*.m, system/tScenarios.m, <domain>/t*.m, <domain>/tb_*.slx,
          baselines/*.mat
work/     cache, codegen, logs/, results/, reports/  (ignored)
```

Top model contents (layout grid in pixels `[left top right bottom]`, left to right = signal flow):

| Block | Type | Position |
|---|---|---|
| `ScenarioIn` (In Bus Element) | root port | [30 300 60 320] |
| `Operator_Console` | virtual Subsystem (Variant Source `VC_OPSRC` + Manual constants for dashboard binding) | [100 80 260 180] |
| `Operator_RC` | SSR `Operator_RC` | [320 80 480 180] |
| `RT_MCU_In` | virtual Subsystem (RTs + Signal Specification) | [540 60 620 260] |
| `MCU` | Model `Shaddad_MCU` | [660 60 860 260] |
| `Link_MCU_GNC` | SSR | [920 60 1080 160] |
| `RT_GNC_In` / `RT_GNC_Out` | virtual Subsystems | [1120 60 1200 160] / [1120 180 1200 260] |
| `GNC_Variant` | Variant Subsystem (`VC_PHASE`) | [1240 60 1440 260] |
| `Plant` | Model `Shaddad_Plant` | [660 340 860 600] |
| `Sensors` | SSR | [920 340 1080 600] |
| `Viz_Logging` | SSR | [1240 340 1440 460] |
| `Sim_Monitors` | SSR | [1240 480 1440 600] |
| Dashboard panel | `simulink_hmi_blocks` | [100 640 1440 820] |

`Shaddad_Plant` contents: ICD section 3.2 binds (`Plant_In`, `structure_plant`, `Power_Plant`, `drive_system`, `Electronics_Plant`, the `Solver Configuration` on `DC48_TracL_n` and the `Plant_Out` holds); revision A's list (`RT_Plant_In`, `Env_Terrain`, `Structure_Tracks`, `Power_System`, `Drive_Train`, `Electronics_HW`, `Implements`) is superseded. Physical lines are named with Connection Labels.

### 7.2 Variant controls (`params/shaddad_params.m` sets the `VC_*` scalars in base)

The variant controls and their choices are those of the ICD's variant table (`docs/icd_static_sections.md`), which binds: `VC_TARGET` (0 SIM, 1 HW), `VC_PHASE` (1 RC only, 2 autonomy), `VC_STRUCT_FID`, `VC_IMPL` (0 none, 1 mower, 2 tiller, 3 sprayer), `VC_IMPL_DRIVE`, `VC_PACK`, `VC_CHG`, `VC_DRIVE_FID`, `VC_OPSRC`, `VC_LIDAR`, `VC_ROW_TRACKER`, `VC_VIZ` and `VC_TEB`. They are plain double scalars, because variant expressions cannot index structs. D99 makes VFH the default bypass (`VC_TEB` 0); the parameters still hold `P.gnc.variant.teb` = 1 until card G2-GNC. Revision A's names in this section (`VC_IO`, `VC_FID_DRIVE`, `VC_FID_BATT`, `VC_LIDAR3D`) are retired.

`startup` activation is avoided for Simscape-containing variants: every choice would have to compile, and Simscape support for it is the domain's risk. Batches are grouped by variant configuration (Section 7.8).

### 7.3 Controller IO-variant pattern (refines the global convention)
`Shaddad_MCU.slx` = `IO_In` (Variant Subsystem) -> Model block `Shaddad_MCU_App` -> `IO_Out` (Variant Subsystem).
- **SIM choice (`VC_TARGET == 0`):** IO_In converts plant-side buses to logical inputs (double to single, encoder angle to counts are done by sensors). IO_Out converts logical outputs to `Bus_ActCmd` and includes the **1-step compute-latency `Unit Delay`** (Ts = the producing task rate).
- **HW choice (`VC_TARGET == 1`):** STM32 blocks (`UART//USART Read` for SBUS, `Encoder`, `ADC`, `Digital Port Read`, `FDCAN Read/Write`, `PWM Output`). Root inports are terminated and root outports are grounded. For deployment `Shaddad_MCU` is built as the top model with `cs_STM32`.
- `Shaddad_MCU_App` has no I/O blocks. It is the unit for MIL, SIL and PIL.
- The same pattern applies to GNC with a ROS 2 HW choice (`roslib/ROS 2` Subscribe/Publish).
- Alternative considered: `VariantControlMode='sim codegen switching'` picks the HW choice automatically during code generation, but it also does so for SIL. Explicit `VC_TARGET` is clearer.

### 7.4 Solver configuration (`params/shaddad_configsets.m`)
All models (Top, Plant, MCU, MCU_App, GNC, GNC_App) carry a `Simulink.ConfigSetRef` named `Shaddad_Ref` with `SourceName = 'cs_Shaddad'`. The base variable `cs_Shaddad` is a copy of one of the configurations below, so one assignment switches every model consistently. MCU models also carry `STM32_Ref -> cs_STM32` and GNC models carry `GNC_CG_Ref -> cs_GNC_CG`. Only `build_mcu.m` and `build_gnc.m` activate those.

| Parameter (real name) | `cs_SimFixed` (default: missions, codegen-equivalent) | `cs_SimRef` (accuracy reference) |
|---|---|---|
| `SolverType` | `Fixed-step` | `Variable-step` |
| `Solver` | `ode3` | `daessc` |
| `FixedStep` | `P.sim.Ts_base` = 1e-3 | - |
| `MaxStep` / `RelTol` / `AbsTol` | - | 1e-3 / 1e-4 / 1e-6 |
| `EnableMultiTasking` | `on` (rate-monotonic semantics identical to the STM32 RTOS-less multitasking scheduler) | - |
| `AutoInsertRateTranBlk` | `off` | `off` |
| `MultiTaskRateTransMsg` / `SingleTaskRateTransMsg` | `error` / `error` | `error` / `error` |
| `AlgebraicLoopMsg` / `ArtificialAlgebraicLoopMsg` | `error` / `error` | `error` / `error` |
| `StrictBusMsg` | `ErrorOnBusTreatedAsVector` | same |
| `InheritedTsInSrcMsg` | `error` | same |
| `UnconnectedInputMsg` | `warning` | same |
| `DefaultParameterBehavior` | `Inlined` (missions) / `Tunable` (interactive, Fast Restart sweeps) | `Tunable` |
| `SignalLogging`, `SignalLoggingName`, `SaveFormat`, `ReturnWorkspaceOutputs` | `on`, `logsout`, `Dataset`, `on` | same |
| `LoggingToFile`, `LoggingFileName` | `on` if StopTime > 600 s; `work/logs/<scn>_<stamp>.mat` | `off` |
| `SaveOutput`, `SaveState`, `DSMLogging` | `off`, `off`, `off` | same |
| `SimCompilerOptimization` | `on` | `on` |
| `UpdateModelReferenceTargets` | `IfOutOfDate` | same |
| `SimscapeLogType` | `none` (missions) / `local` (unit and system tests) | `local` |
| `SimscapeLogDecimation`, `SimscapeLogLimitData`, `SimscapeLogDataHistory` | 10, `on`, 5000 | 1, `on`, 50000 |
| `SimscapeCompileComponentReuse` | `on` (two identical drive units) | `on` |
| `SimscapeMultithreadedCompilation` | `on` | `on` |
| `SimscapeLogSimulationStatistics` | `off` (`on` in T-INT-03 profiling) | `on` |
| Solver Configuration `UseLocalSolver` | `on`, BE, 1 ms, fixed cost | **`off`** (set by `in.setBlockParameter` or `set_param` on `Shaddad_Plant` in the reference test) |
| Model param `EnablePacing` / `PacingRate` | `off`; `on`/1 only for interactive dashboard RC driving | `off` |
| Model param `FastRestart` | `on` for sweeps in `run_all_scenarios` | `off` |

Why fixed-step `ode3` at 1 ms: every Simscape network runs on its own local fixed-step solver (Backward Euler, L-stable), and all controllers are discrete. The only Simulink continuous states left are the chassis and track states in `structure`. **Constraint on structure:** any Simulink continuous mode must keep `|lambda|` <= `P.sim.lambda_max_explicit` = 1000 1/s, a time constant >= 1 ms (explicit RK3 stability: `|lambda h|` <= 1 < 2.5 at h = 1 ms). Stiff track-shear or slip dynamics must be put in Simscape, discretized, or regularized. The model then matches the multitasking target schedule exactly, supports rapid accelerator and `parsim`, and stays deterministic.

### 7.5 Multirate scheduling

Allowed rate set `P.sim.Ts_allowed` (all integer multiples of the 1 ms base; any other rate is a review failure, D13), with the fields of the sample-time struct `Ts` (`params/shaddad_params.m`):

| Ts [s] | `Ts` fields | Used for |
|---|---|---|
| 0.001 | `base`, `esc`, `ssc`, `safety`, `enc` | Simscape local solver; MCU base task (SBUS bytes, safety supervisor); ESC speed loops; encoders; energy integration |
| 0.005 | `imu`, `nav_imu` | IMU sampling; the navigation IMU path; the CAN2 frame mcu_imu |
| 0.01 | `ctrl`, `can_drive`, `nav`, `odom`, `bms_fast`, `afe`, `impl_ctrl`, `scen` | MCU vehicle control (Task_10); drive CAN; navigation fusion; odometry; BMS fast loop; scenario input |
| 0.05 | `comm`, `can_fb2`, `can_slow`, `gnc` | MCU Task_50 and mcu_status; drive FB2; GNC guidance and control, path follower, comp_cmd |
| 0.1 | `hk`, `bms_slow`, `gnss`, `lidar`, `perception`, `gnc_avoid`, `env`, `monitor`, `log`, `encl_ssc`, `spray` | GNSS fixes, LiDAR scans, perception, local avoidance, environment, monitors, logging (missions) |
| 0.2 | `scanmatch`, `teb`, `gnc_status`, `mcmd` | Scan matching, TEB, gnc_status, mission commands |
| 1.0 | `plan`, `bms_bal` | Global planner, BMS balancing |
| 0.007 / 0.014 | `rc_frame` (= `P.rc.frame_period`) | SBUS frames, only inside RC_Link_Sim (`P.sim.Ts_rc_allowed`) |

Rate-transition policy:
1. **Model-reference root ports declare explicit sample times.** All transitions into or out of a Model block live in the top-level (or Plant-level) `RT_*` wrapper subsystems, followed by a `Signal Specification` that pins the bus type.
2. **SSR inputs:** the consumer SSR contains RTs at its In Bus Elements when the producer rate differs (consumer-side transitions).
3. Every RT is set to `Integrity='on'`, `Deterministic='on'`. Fast-to-slow is a ZOH sample with no delay. Slow-to-fast adds one slow-period delay in multitasking. This is intentional and matches the target.
4. **Algebraic-loop breaking:** controller outputs pass through a 1-sample compute delay (IO_Out SIM choice). Env_Terrain takes pose through a 1-sample (1 ms) delay. Plant truth outputs may be direct-feedthrough. `AlgebraicLoopMsg = error`.
5. Asynchronous messages (SBUS bytes, CAN frames) are modeled as 1 kHz polled buffers with `seq`/`valid`. No function-call or async-event modeling in v1.

### 7.6 Logging plan
- Mechanism: signal logging (`logsout`, Dataset) on named lines. The per-port parameters `DataLogging='on'`, `DataLoggingNameMode='Custom'`, `DataLoggingName=<manifest name>`, `DataLoggingDecimateData='on'`, `DataLoggingDecimation` = `round(P.log.Ts/Ts_signal)` (for example 100 for 1 kHz signals in missions) are all checked to exist. Signals inside model references are logged through `Simulink.SimulationData.ModelLoggingInfo` (Signal Logging Selector), built by `scn_base_input.m` from `P.log.manifest`.
- Missions (StopTime > 600 s): `LoggingToFile='on'`, `Ts_log = 0.1 s`, about 120 scalar channels, giving roughly 140-200 MB for 4 h. LiDAR scans and paths are logged at 1 Hz only.
- Tests: `Ts_log = 0.01 s`, full manifest, `SimscapeLogType = 'local'` for the unit under test only.
- SDI: `Simulink.sdi.setAutoArchiveMode(true)` with an archive limit of 10 runs in `shaddad_startup.m` to cap memory. `Record` block only when `VC_VIZ > 0`.
- Logging manifest `P.log.manifest` (name, source bus.element, decimation):
  - Truth: `pos_ENU`, `eul`, `v_B(1)`, `omega_B(3)`, `v_track`, `slip`.
  - Commands: `v_cmd`, `w_cmd`, `mode`, `rc_ok`, `rc_failsafe`, `sbus_ch1..4`.
  - Power: `SoC`, `V_bus`, `I_batt`, `P_batt`, `T_cell_max`, `contactor_closed`.
  - Drive: `tau_m`, `w_m`, `I_dc`, `T_wind`.
  - GNC: `pose_est`, `P_diag`, `gnss_fix`, `xte_est`, `gnc_mode`, `wp_idx`, `path_xy` @1 Hz.
  - Environment: `env_slow.gnss_mode`, `terrain.grade_long`, `terrain.T_amb`.
  - KPI: all `Bus_SimKPI` elements.

### 7.7 Simulation-speed plan (multi-hour missions)

| Measure | Effect |
|---|---|
| Averaged (non-switching) power electronics only; PWM switching models are forbidden in system models | Removes 20 kHz events; enables the 1 ms local solver |
| `VC_DRIVE_FID` 1 (system-level Motor & Drive) or 3 (ideal, the endurance set) in missions; the FOC variant (2) only on the drive bench (D52) | About 5-10x lower drive cost |
| Simscape local solver BE, `DoFixedCost='on'`, `MaxNonlinIter=3`, 1 ms; Partitioning (`FAST`) if the power network has contactors and diodes and qualifies | Bounded cost per step |
| Plant, MCU and GNC Model blocks in `Accelerator`; top in `accelerator` | 10-30x over Normal |
| Rapid accelerator for S3 endurance and batches (`VC_VIZ = 0`, no interpreted MATLAB System, sensor models `SimulateUsing='Code generation'`) | Another 1.5-2x |
| Logging at 10 Hz to file, Simscape logging off | Memory bounded, I/O < 5 % |
| `parsim` across scenarios (Parallel Computing Toolbox, `SetupFcn = @shaddad_startup`, `TransferBaseWorkspaceVariables='off'`, `UseFastRestart='on'`) | N scenarios in about the time of the longest |
| 3D LiDAR off in missions (`VC_LIDAR` 1, FAST2D); 2D scan 360 beams at 10 Hz | 3D ray casting would cost about 10-50 ms per scan, i.e. a 2-5x slowdown |
| Collision check and obstacle selection at 10 Hz, K = 12 nearest | Negligible |
| Operating-point save/restore (`SaveOperatingPoint`, `LoadInitialState`, `InitialState` model params) | Lets long missions be split and resumed |

Per-step budget at 1 ms for RTF >= 20 (<= 50 us wall per step, accelerator; revision A's design target, re-baselined by D56):

| Component | Target |
|---|---|
| Simscape plant | <= 20 us |
| Chassis and tracks | <= 5 us |
| MCU (1 kHz) | <= 3 us |
| Sensors (amortized) | <= 5 us |
| GNC (amortized: EKF 200 Hz, guidance 50 Hz, planner 1 Hz, LiDAR 10 Hz) | <= 12 us |
| Monitors and logging | <= 5 us |

Expected performance (revision A's estimates, to be measured in T-INT-03; on the project PC the Normal-mode system RTF is about 0.45 to 0.5 and the top model in Accelerator gives 1.07, STATUS item 14):

| Scenario | Normal | Accelerator | Rapid accelerator |
|---|---|---|---|
| S1 (120 s) | 40-60 s wall (RTF 2-3) | 5-10 s (RTF 12-25) | - |
| S3 (4 h) | about 1.5-3 h | 8-15 min (RTF 16-30) | 5-8 min |

First accelerator build: 2-5 min with LCC. Incremental rebuilds: 20-60 s.

### 7.8 Scenario framework
- `scripts/scenarios/scn_<ID>_<name>.m`: `function [in, crit] = scn_S1_rc_drive(P)` returns a `Simulink.SimulationInput('Shaddad_System')` and a criteria struct array (`signal, metric, op, limit, window`). Each scenario sets:
  - `StopTime`
  - `setVariable('P', Pmod)` with the scenario map, terrain tables, obstacles, initial pose and SoC
  - `setVariable('VC_*', ...)`
  - `setExternalInput(ds)` built by `utilities/scn_make_input.m` from compact waypoint tables (`[t, value]`, ZOH)
  - seeds `P.sim.seed.*`
  - logging options
- `scripts/scn_base_input.m(P, name)` holds the shared defaults.
- `scripts/run_scenario.m(id, 'Mode','accelerator'|'normal'|'rapid', 'Viz',0|1|2, 'Save',true)` runs `out = sim(in)`, evaluates `res = eval_criteria(out, crit)`, prints a PASS/FAIL table, and saves `work/results/<id>_<yyyymmdd_HHMMSS>.mat` (in, crit, res, summary KPIs, wall time, RTF).
- `scripts/run_all_scenarios.m` groups inputs by variant set, then runs `parsim(ins, 'ShowProgress','on', 'SetupFcn',@shaddad_startup, 'UseFastRestart','on')`.
- Determinism: random sources take seeds from `P.sim.seed_base` (20261018) and their offsets `P.sim.seed.*`: gnss 1, imu 2, lidar2d 3, lidar3d 4, rc 6, obstacles 7, terrain 8, power 9, adc 10, spray 11.

### 7.9 Parameter, bus and data management (recommendation)
**Recommendation: text `.m` files are the single source of truth, loaded into the base workspace by the project startup. Do not use a hand-edited Simulink Data Dictionary in the build phases.**

Why:
- Several agents author domains in parallel. `.m` files are diffable and mergeable, while `.sldd` is a binary SQLite store with single-writer conflicts.
- `SimulationInput.setVariable('P', Pmod)` gives `parsim`-safe scenario variation without mutating shared state.
- Model references can read the base workspace (`EnableAccessToBaseWorkspace='on'`, the default; checked live).

Structure:
- `params/param_<domain>.m` returns a struct; it is pure, with no `assignin`.
- `params/shaddad_params.m` builds `P` = `struct('structure',..,'power',..,'drive',..,'electronics',..,'rc',..,'sensors',..,'gnc',..)` and adds the integration fields `P.sim`, `P.frames`, `P.env`, `P.viz`, `P.log`, `P.scen` and `P.ic` (bus initial-condition structs generated with `Simulink.Bus.createMATLABStruct`). It runs sanity checks (all rates in the allowed set, units), then `assignin('base','P',P)`.
- `buses/buses_<domain>.m` returns a struct whose fields are `Bus_<Name>` `Simulink.Bus` objects. **This refines the single-file convention so agents never edit the same file.** `buses/shaddad_buses.m` calls all of them, errors on duplicate names, checks that every nested `Bus: ` type resolves and that every element has a `Unit` and `Description`, then assigns them in base.
- `utilities/shaddad_check_interfaces.m` compiles the top model with stubs and reports port/bus mismatches. `utilities/shaddad_icd_report.m` generates `docs/ICD.md` from the bus objects.
- Codegen phase (Stage 6): `scripts/build_mcu.m` generates `work/dd/Shaddad_MCU.sldd` from the same `.m` sources. Calibration gains become `Simulink.Parameter` with storage classes. The file is never hand-edited and never committed.

### 7.10 MATLAB project startup, shutdown and shortcuts
- **Startup file** `scripts/shaddad_startup.m` (register with `addStartupFile`):
  1. `shaddad_params` (it also sets the `VC_*` scalars and the `Bus_*` objects), `shaddad_configsets('fixed')`
  2. Check for a compiler. Warn if no MinGW (LCC fallback).
  3. Check licenses and toolboxes (warn if Simscape Battery or Electrical is missing).
  4. `Simulink.sdi.setAutoArchiveMode(true)`
  5. Print a banner with the release and git status.
  It must **not** `cd`, `clear all` or `bdclose all` (the MATLAB session is shared).
- **Shutdown file** `scripts/shaddad_shutdown.m`: removes only the project's base variables (`P`, `Bus_*`, `VC_*`, `cs_*`). It does not close or save models.
- **Shortcuts** (group "Shaddad"):

| Shortcut | Target |
|---|---|
| Open System | `open_system('Shaddad_System')` |
| Reload Data | `scripts/shaddad_startup.m` |
| Run S1 RC Drive | `run_scenario('S1')` |
| Interactive RC (dashboard) | `run_scenario('S1','Op','dashboard','Viz',1)`, sets `EnablePacing` |
| Fast Regression | `tests/run_regression('Fast')` |
| Full Regression | `tests/run_regression('All')` |
| Replay Bird's-Eye | `shaddad_replay_birdseye` (last result) |
| Check Interfaces | `shaddad_check_interfaces` |
| ICD Report | `shaddad_icd_report` |
| Build MCU (STM32) | `scripts/build_mcu.m` |
| Clean work/ | `scripts/clean_work.m` |

### 7.11 Test conventions (Simulink Test + `matlab.unittest`)
- **Signal-only SSRs and model references:** Simulink Test harnesses created with `sltest.harness.create(target, 'Name','<Unit>_H<nn>_<purpose>', 'Source','Inport' or 'Test Sequence', 'Sink','Outport' or 'Test Assessment', 'SaveExternally', false)`. Harnesses are stored with the owning file. Assessments use `verify()` in `sltestlib/Test Assessment` with numeric limits.
- **Physical SSRs:** hand-built benches `tests/<domain>/tb_<Unit>.slx`. Auto-harness sources are not meaningful for physical ports, and harness support for Connection Ports is to be confirmed in Stage 1. A bench contains the SSR plus its own Solver Configuration, ideal sources and loads.
- **Test code:** `matlab.unittest` classes derived from `sltest.TestCase` (checked to exist), in `tests/<domain>/t<Unit>.m`. They are text files, so parallel-safe. Tags: `Unit`, `Integration`, `System`, `Fast` (< 60 s each), `Long`, `HW`, `Ref`.
- **Baselines:** `tests/baselines/<test>.mat`, compared with `Simulink.sdi.compareRuns` using per-signal `AbsTol`/`RelTol` (set in the test).
- **Runner:** `tests/run_regression.m(tagSet)` uses `matlab.unittest.TestSuite.fromFolder('tests','IncludingSubfolders',true)` with a `HasTag` selector, plus `TestReportPlugin` (HTML to `work/reports`) and `XMLPlugin` (JUnit). Coverage only if Simulink Coverage turns out to be installed. Fast suite: <= 10 min. Full suite: <= 60 min with `parsim`. Nightly adds S3.
- Integration owns `tests/integration`, `tests/system`, `tests/run_regression.m` and the optional Test Manager file `tests/Shaddad_Regression.mldatx`. Domains contribute only `.m` test classes and benches.

### 7.12 Build order and dependency plan

| Stage | Content (owner) | Exit criterion |
|---|---|---|
| 0 Infra | Integration: `param_sim.m`, `shaddad_params.m`, `buses_sim.m`, `shaddad_buses.m`, `shaddad_configsets.m`, startup/shutdown, shortcuts, `ShaddadMode`, `subsystems/integration` on path | `shaddad_startup` runs clean; all `Bus_*` resolve |
| 1 Interface stubs | Every domain saves its SSR or model with **final ports and bus types** and placeholder content (Ground/Constant at outputs, ideal sources/loads on physical ports). Integration wires `Shaddad_Plant` and `Shaddad_System`. Smoke test: Connection Port across SSR, `ConnectionType` strings, harness on a physical SSR. | T-INT-01 (top compiles), T-INT-05 |
| 2 Physical leaves (parallel) | power, drive, structure (with the environment, `structure_env`, D10), electronics, sensors, rc (Operator_RC) with benches | Each domain's unit tests pass; Plant compiles; S0 smoke |
| 3 MCU firmware | `Shaddad_MCU_App` (SBUS decode, mode manager, mixer, speed PI, failsafe), `Shaddad_MCU` SIM IO | MCU unit tests; MIL with Plant |
| 4 Phase-1 integration | `VC_PHASE = 1`: S1, S2, S3, S6, S8 | All Phase-1 criteria pass; T-INT-02/03/04/07 |
| 5 GNC (Phase 2) | `Shaddad_GNC_App` (EKF fusion, planners, guidance, mission manager) + `Shaddad_GNC` SIM IO | S4, S5, S7 pass |
| 6 HW variants and codegen | `VC_TARGET = 1` STM32 drivers, `cs_STM32`, SIL/PIL of `Shaddad_MCU_App`, ROS 2 IO for GNC | `slbuild` with no errors; SIL vs MIL equivalence (max abs diff <= 1e-5 on single outputs) |
| 7 Hardening | Regression suite, profiling, ICD freeze, docs | Full regression green; RTF targets met |

Dependency graph:
```
buses_* -> shaddad_buses -> stubs -> Shaddad_Plant -> Shaddad_System
param_* -> shaddad_params ----------^
MCU_App -> MCU
GNC_App -> GNC
```
Integration never edits another domain's files. Domains never edit `models/Shaddad_System.slx` or `Shaddad_Plant.slx`. They request wiring changes through their spec or bus file.

---

## 8. Parameters (`params/param_sim.m` returns `S` with the fields `sim`, `frames`, `viz`, `log` and `scen`, assembled as `P.sim`, `P.frames`, `P.viz`, `P.log` and `P.scen`; the environment and the worlds are `P.world`, from `params/param_world.m`)

The values are those of the parameters (G1-01 part 2, step 12); the parameter files are the source.

| Parameter | Value | Unit | Source / justification |
|---|---|---|---|
| `sim.Ts_base` | 1e-3 | s | Simscape local solver, MCU base task, ESC, encoders |
| `sim.Ts_allowed` | [1e-3 5e-3 1e-2 5e-2 0.1 0.2 1.0] | s | The allowed rate set (Section 7.5, D13) |
| `sim.Ts_rc_allowed` | [0.007 0.014] | s | SBUS frame periods, only inside RC_Link_Sim |
| `sim.Ts_compiled_allowed` | [0 Inf] with `Ts_allowed` and `Ts_rc_allowed` | s | T-INT-05 compiled-rate audit (D32) |
| `sim.Ts_scen` | 0.01 | s | Scenario external input (100 Hz is enough for stick profiles) |
| `sim.Ts_monitor` | 0.1 | s | Sim_Monitors: KPIs, collision check |
| `sim.compute_delay` | 1 | samples | Controller output latency (GNC -> MCU link) |
| `sim.solver_id` | 1 | - | 1 = ode3 fixed step (`P_meta.sim.solver_names`); the reference run uses `daessc` (Section 7.4); the Simscape local solver is `P_meta.sim.ssc_solver` (`NE_BACKWARD_EULER_ADVANCER`) |
| `sim.ref.RelTol`, `AbsTol`, `MaxStep` | 1e-4, 1e-6, 1e-3 | -, -, s | Accuracy-reference configuration |
| `sim.ssc.Ts` | 1e-3 | s | Simscape local solver step (Backward Euler, fixed cost); equals Ts_base |
| `sim.ssc.MaxNonlinIter`, `MaxModeIter` | 3, 2 | - | Fixed-cost local solver; enough for averaged electrical |
| `sim.ssc.tau_filt` | 2e-3 | s | Simulink-PS input filter where derivatives are needed |
| `sim.ssc.Ts_fault`, `Ts_encl` | 1e-4, 0.1 | s | Fuse and short-circuit harness tests only; the electronics-enclosure thermal network |
| `sim.lambda_max_explicit` | 1000 | 1/s | Limit on Simulink continuous states with ode3 at 1 ms (Section 7.4) |
| `sim.seed_base`, `sim.seed.*` | 20261018; gnss 1, imu 2, lidar2d 3, lidar3d 4, rc 6, obstacles 7, terrain 8, power 9, adc 10, spray 11 | - | Deterministic noise (Section 7.8) |
| `sim.rtf_min_mission` / `rtf_min_short` | 4 / 5 | - | T-INT-03 targets: S3 with the endurance fidelity set, and S1 (10 with MinGW-w64), in accelerator (D56) |
| `sim.wall_timeout` | 1800 | s | `sim(...,'Timeout',...)` guard for batch runs |
| `sim.stop_soc` | 0.05 | 1 | Stop if the true SOC < 5 % |
| `sim.endurance` | VC_PACK 2, VC_DRIVE_FID 3, VC_STRUCT_FID 2, VC_LIDAR 1, VC_TEB 0, VC_VIZ 0, lidar_div 2 | - | The endurance fidelity set (D56) |
| `sim.mon.footprint` | [1.00, 0.78] | m | Collision footprint L x W (`P.structure.geom.L_overall`, `W_overall`) |
| `sim.mon.Kc` / `Kb` / `Nd` | 12 / 8 / 4 | - | Nearest static cylinders and boxes, and dynamic obstacles, checked per 0.1 s |
| `sim.mon.z_robot` | 1.1 | m | Robot height with the mast: obstacles entirely above it are ignored |
| `sim.mon.E_out_min_Wh`, `E_spr_min_J` | 1, 1000 | Wh, J | Energy-audit denominator floors (D40) |
| `sim.mon.v_abs_max`, `T_cell_rng` | 3, [250 360] | m/s, K | Check Static Range limits (warnings only) |
| `sim.ovr.R_pre` | 4.7 | Ohm | Scenario value of `P.power.pdu.R_pre`. D98 removes this override; the twin changes on card G2-PWR |
| `sim.stub.*` | for example `V_bus` 46.8 V, `T_amb` 318.15 K | mixed | Interface-stub values, used only until a domain replaces its stub |
| `frames.g` | 9.7876 | m/s^2 | Local gravity at the site (`gravitywgs84(430, 24.15)`); `frames.imu_g_internal` = 9.81 is the value inside the MATLAB IMU model |
| `frames.mag_ENU_uT` | [1.745, 34.116, -26.317] | uT | WMM2025 at `lla0`, epoch 2026.8 |
| `frames.mag_dec_deg`, `mag_dip_deg` | 2.93, 37.6 | deg | Same source |
| `frames.R_NED_ENU` | [0 1 0; 1 0 0; 0 0 -1] | - | Frame conversion |
| `frames.R_FRD_FLU` | diag([1 -1 -1]) | - | Frame conversion |
| `viz.enable`, `viz.Ts`, `viz.Ts3d` | 0, 0.1, 0.1 | -, s, s | Live views off by default; the bird's-eye and 3-D views at 10 Hz |
| `viz.trail_len`, `scan_decim` | 20000, 4 | - | About 33 min of trail at 10 Hz |
| `viz.footprint` | [1.00, 0.78] | m | Robot L x W (`P.structure.geom`) |
| `viz.camera` | 1 | - | 3-D view camera: 1 chase, 2 side, 3 top, 4 free |
| `log.Ts_mission`, `log.Ts_test` | 0.1, 0.01 | s | Section 7.6 |
| `log.to_file_threshold` | 600 | s | Log to file above this StopTime |
| `log.sdi_archive_runs` | 10 | - | SDI memory cap |
| `scen.soc0` | 0.95 | 1 | Default initial SOC (D28) |
| `scen.stop_on_collision`, `stop_on_mission_done` | true, true | - | End failing runs early; end the run at mission_done (S3 sets it false) |
| `scen.T_amb_hot` | 323.15 | K | 50 deg C worst-case override (S3 hot case) |
| `scen.world_id` | 1 to 7 | - | `shaddad_params('World', k)`: 1 flat_test, 2 slope_test, 3 palm_farm, 4 greenhouse, 5 solar_farm, 6 soil_strips, 7 row_field |
| `scen.dash`, `scen.joy`, `scen.rowtest`, `scen.rowdrv` | - | - | The dashboard's initial switches, the live-drive joystick map, and the RC row test (`params/param_rowtest.m`, `utilities/row_test_prm.m`) |
| `world.lla0` | [24.1556, 47.3120, 430] | deg, deg, m | Al-Kharj palm district (scenario-overridable); `P.sensors.refLLA` is the same |
| `world.env.T_amb` | 318.15 | K | 45 deg C design ambient |
| `world.env.diurnal` | T_mean 311.15 K, T_amp 9 K, h_peak 15 h, t0_hour 6 h, G_pk 1000 W/m^2 | mixed | (E3): 29 to 47 deg C, the maximum at 15:00, t = 0 at 06:00 local (AST, UTC+3); used when `world.env.use_diurnal` is true (default false) |
| `world.env.G_solar` | 1000 | W/m^2 | Clear-sky Saudi summer noon |
| `world.env.T_ground_offset`, `T_sky_offset` | 15, -12 | K | Sunlit soil above the air temperature; the sky below it |
| `world.occ.res`, `world.occ.origin` | 10, [-10 -10] | cells/m, m | Occupancy grid: 0.1 m cells, its lower-left corner in W |
| `world.soilTable`, `world.soilDefault` | 5 soils; default 2 (farm soil, sandy loam) | - | The soil table is the structure spec's section 6.6 (owner: structure) |
| `world.terrain`, `world.obstacles`, `world.rows`, `world.dock`, `world.gnssZones` | per world | - | Built by `param_world(k)`. The greenhouse house is 9.6 x 37.5 m with wall posts (world 4, S4); the palm grove is planted on an 8 x 8 m grid (world 3, `P.gnc.cov.palm_spacing`); the GNSS sky-view factors are the sensors domain's |
| the S2 ramp | 20 deg for S2 and S2b, and for the no-implement variants of S2c and S2d; 18.5 deg for the sprayer variants S2spray, S2c and S2d (CR-001; `scn_S2_slope` scales the ramp with its `Slope` option); length 20 m (x = 10 to 30 m), fillets 3 m, across the whole 60 m wide world | deg, m | slope_test (world 2): the S2 slope requirement |
| the sensor lever arms | `P.sensors.imu.r`, `P.sensors.gnss.rAnt1` / `rAnt2`, `P.sensors.lidar.r` | m | The sensors domain's values (sensors spec 7.1); the track frames from `P.structure.geom.B` (0.60 m) |

---

## 9. Verification

### 9.1 Integration framework tests (`tests/integration/`)

| ID | Test | Pass/fail criteria |
|---|---|---|
| T-INT-01 | Top-model compile with stubs, then with real units | Update diagram completes without errors in < 60 s (stubs) / < 180 s (full, Normal). Zero algebraic-loop warnings. Zero implicit rate transitions. |
| T-INT-02 | Fixed vs reference solver equivalence on S1 (120 s) and S2 (60 s) | RMS(v_fixed - v_ref) <= 1 % of max \|v\|. SoC difference <= 5e-4. RMS(I_batt diff) <= 2 % of peak. Final position difference <= 0.05 m. |
| T-INT-03 | Simulation speed (accelerator incl. the top model, `VC_VIZ=0`, mission logging; revision B D56) | S1: RTF >= 5 (>= 10 with MinGW-w64). S3 (endurance fidelity set; the mission of D101): RTF >= 4. Normal-mode S1: RTF >= 1. Simscape statistics: no step with > 3 nonlinear iterations in > 1 % of steps. Simulink Profiler report saved. |
| T-INT-04 | Frame and sign conventions (kinematic oracle E1 + sensor models) | v_L = v_R = 1 m/s for 10 s from psi0 = 0: x = +10 m +/- 2 %, \|y\| < 0.05 m. psi0 = +90 deg: y = +10 m +/- 2 %. v_R > v_L: psi increasing, omega_B(3) > 0. GNSS course with psi = 0: 90 +/- 1 deg. IMU level at rest: the truth specific force z = +`P.frames.g` (9.7876 m/s^2) +/- 0.05, and the `Bus_ImuRaw` acceleration z = +9.79 m/s^2 +/- 0.5 (the physical specific force, D12), as `t_framework/testConventionsFromResults` checks. Nose-up on the run's ramp: the mean pitch on the ramp is -th +/- 0.02 rad, th the run's slope (20 deg for S2 and S2b, 18.5 deg for the sprayer variants; for 20 deg, theta = -0.349 +/- 0.02 rad). D100 takes the reference for S2 and S2spray from the structure reference with the contact compliance (+/- 0.02 rad); the script keeps the rigid value until card G2-INT (section 9.2). |
| T-INT-05 | Rate audit | Compiled sample times are a subset of `P.sim.Ts_compiled_allowed` = {0, Inf, 1e-3, 5e-3, 1e-2, 5e-2, 0.1, 0.2, 1, 0.007, 0.014}; 0.007 / 0.014 only inside RC_Link_Sim. Every Model-block root port has an explicit non-constant Ts. `MultiTaskRateTransMsg` = error compiles. |
| T-INT-06 | Logging | S3 log file <= 300 MB. Every manifest signal is present in `logsout`. Decimation correct (1 kHz signals at 10 Hz in missions). |
| T-INT-07 | Determinism | Two S5 runs with the same seeds: max abs difference of logged pose = 0. `parsim` (2 workers) equals serial: max abs difference = 0. |
| T-INT-08 | Energy audit (E4, revision B) on S1 and S3 | eps_el <= 1 % and eps_mech <= 1 %. |
| T-INT-09 | Operating-point restart | Save at t = 600 s, resume to 1200 s vs continuous run: pose difference <= 1e-3 m, SoC difference <= 1e-6. |
| T-INT-10 | Variant matrix compile | A covering array of `VC_PHASE` {1,2} x `VC_DRIVE_FID` {1,3} x `VC_IMPL` {0..3} x `VC_PACK` {1,2} x `VC_VIZ` x `VC_TEB` x `VC_OPSRC` compiles, every value of each control in at least one of its 8 rows (`VC_TARGET = 0`; `t_framework`). `Shaddad_MCU` with `VC_TARGET = 1` + `cs_STM32` generates code without errors (Stage 6). |
| T-INT-11 | Env_Terrain unit (harness): dropped by D10 | The environment is the structure domain's (`structure_env`, `Bus_EnvState`); its grade, soil-zone and obstacle checks belong to the structure domain's benches (structure spec section 8). |
| T-INT-12 | Sim_Monitors unit (harness) | The footprint box `P.sim.mon.footprint` (1.00 x 0.78 m) at the origin vs cylinder r = 0.3 m at (1.5, 0): SepDist = 1.5 - 0.5 - 0.3 = 0.70 +/- 0.01 m (`t_framework/testMonitorSepDist`). Overlapping: CollStatus = 1 and the Stop Simulation fires within 0.1 s. NaN injected into truth: the assertion stops the run with `stop_reason = 5`. |

### 9.2 System scenarios (`scripts/scenarios`, `tests/system/tScenarios.m`)

The scenarios and their pass criteria are those of `docs/architecture.md` section 12, which binds; this table repeats them (G1-01 part 2, steps 12 and 19, D84, D99, D100, D101, D106, D108), with the value of each requirement that names a scenario. Where a script still checks a criterion the old way, the row says what it checks today and the card that changes it. The park criterion (D82) applies to every brakes-locked stop of S2 to S6.

| ID | Scenario (setup) | Pass/fail criteria |
|---|---|---|
| S0 Smoke | Phase 1. 10 s, all inputs zero (key on, TX off), flat_test | pos drift < 1e-3 m; abs(dSOC) < 1e-4; no NaN/Inf; after the self-test the robot waits in FAULT, drives disabled and brakes engaged, until the operator's deliberate reset and a new arm command (D91, in place of D30's automatic re-arm); the timing criteria of the power-up are set when card G2-MCU builds D91. **Today's script:** the automatic re-arm of D30 (ICD 6): DO0 high <= 0.05 s, DO1 pulse 50 +/- 2 ms, Safety_OK and Safety_HV_OK true <= 0.2 s; HV_Ready true <= 1.0 s; MCU mode STANDBY <= 1.5 s and at 10 s; no class-A bit at 10 s (card G2-MCU builds D91, then the criteria change on G2-INT) |
| S1 RC drive | Phase 1. 120 s, flat farm soil, no implement. t = 0.5 s TX on; t = 2 s arm edge (sticks neutral); t = 5 s throttle for v_target 1.2 m/s; t = 30 s steer +0.5; t = 45 s steer 0; t = 52 s throttle released (the stop from 1.2 m/s); t = 60 s throttle for -0.6 m/s; t = 80 s throttle 0 | 90 % speed rise <= 2.0 s; overshoot <= 5 %; steady error <= 0.03 m/s; yaw rate within +/-10 % of w_ref after 2 s and negative for steer > 0; TX frame -> DRV_CMD frame <= 30 ms (budget 18-28 ms) and ScenarioIn stick -> ESC speed reference <= 65 ms (D37); stop distance after the release from 1.2 m/s <= 0.95 m (replay 0.88 m, D33); reverse speed at 75 s within 0.05 m/s of its target; eps_el and eps_mech <= 1 % (T-INT-08); no collision; RTF >= 5 (accelerator). Today: the RTF criterion is BLOCKED without a C compiler (resolved by D121, 2026-11-23: the compiler is there; the script still marks the criterion BLOCKED until card T-02 changes it, STATUS 63) |
| S2 Slope | Phase 1. slope_test (20 deg), no implement (226 kg) (CR-001), v_target 0.8 m/s, stop 30 s mid-ramp, resume (the hill start) | mean ground speed on the ramp >= 0.30 m/s on farm soil, climbing and descending (REQ-SYS-097); slip <= 25 %; peak abs(I_pack) <= the BMS continuous limit `P.power.bms.I_dis_cont` (150 A; 1 ms peak, D40); roll-back <= 0.05 m during the hold and over the hill start (D100, REQ-SYS-112); winding rise <= 20 K; no BMS/ESC trip; no TILT_WARN/TILT_STOP during start or stop on the ramp (D47); no OVERSPEED (bit 16) on the hill start (D74); gearbox output torque <= T2N `P.drive.gear.T_rated_out` (240 N m per side, 10 s moving mean) and <= T2B `T_acc_out` (360 N m per side with the 1 kW units of D95, 280 N m with the as-built 750 W) peak (D51); the mean pitch on the ramp within +/- 0.02 rad of the structure reference with the contact compliance (D100). **Today's script:** mean ramp speed >= 0.75 m/s (measured 0.725, a known deviation, STATUS 5.3); the pitch against the rigid value: the mean pitch on the ramp within +/- 0.02 rad of -20 deg (T-INT-04); the roll-back only during the 30 s hold (card G2-STR for the reference, G2-INT for the script and the speed) |
| S2b Class-B stop on the slope | Phase 1. as S2 (no implement, 20 deg): RC link lost 1 s after t_rel while the robot stands on the ramp, before the ESC's own park | stands at the loss (abs(v) <= 0.1 m/s); CTRL_STOP <= 0.13 s after the loss (D37); parked <= 1.1 s after CTRL_STOP and STANDBY <= 1.5 s with both brakes engaged and both ESCs READY (D71); torque limits held from the park request to STANDBY and 0 in STANDBY (D71); roll-back <= 0.05 m; no STOP_FAIL (bit 22); HV and Safety_OK kept; no ESTOP, FAULT, ESC QUICK_STOP, ESC FAULT or STO |
| S2spray Sprayer climb | Phase 1. nominal S2 with the full-tank sprayer (316 kg) on an 18.5 deg ramp (CR-001): the heaviest climb, and the sprayer's reverse cap at the hill start | the S2 criteria under its own id, with REQ-SYS-097's 0.40 m/s for the sprayer's climb, the pitch against the compliance reference (D100); no OVERSPEED on the hill start (D74). **Today's script:** the rigid pitch reference. Known deviations: mean ramp speed 0.735 m/s against 0.75, and the T-INT-04 pitch error 0.023 rad against 0.02 (STATUS 5.3) (card G2-INT) |
| S2c Stop-go descent, no regen | Phase 1. sprayer (316 kg) on an 18.5 deg ramp: from the plateau at x 36 m, facing down the slope, at the 0.8 m/s throttle, CCL 0; 14 operator reset and re-arm cycles. Also run with no implement on 20 deg (`Implement` 0, steps 12b and 12d of G2-DSC); the descent table at the 0.75 deg IMU error bound (D83) | at least one OVERSPEED park; the MCU target never above `ref_fwd` at the MCU's pitch; every OVERSPEED park safe by the park criterion, (a) speed and (b) pitch (`utilities/scn_descent_parks.m`, D82); no STOP_FAIL, ESC FAULT or TILT_STOP; held on the brakes (creep <= 0.02 m) |
| S2d Hardware E-stop on the descent | Phase 1. as S2c with regen available (normal CCL) and the throttle held; the front E-stop at `EstopTime` (default 40 s). The no-implement run uses a 1.2 m/s throttle and the E-stop at 18.8 s, on the straight part (steps 12b and 12d) | moving at the E-stop; STO <= 50 ms after the plant-side edge (REQ-SYS-063, provisional under D96); both brakes engaged <= 0.15 s after it; stop distance <= `StopDist` (0.10 m with the sprayer, 0.6 m with no implement); the park criterion (a) and (b); no TILT_STOP; held (creep <= 0.02 m). **Today's script:** STO <= 20 ms, tighter than REQ-SYS-063: a regression check on the twin's 17 ms chain (no change needed) |
| S3 Endurance | Phase 2. phase-2 mowing (`palm_demo`, the mower on along the rows, 1.0 m/s) at the pack's end of life (SOH 0.80, R0 x 1.5) from SOC 0.95, at 45 C ambient (hot case 50 C), until the return threshold, then the return to the dock; endurance fidelity set `P.sim.endurance` (D101; D45's and D46's shifts superseded) | arrives at the dock with at least the 0.20 reserve; no BMS trip; T_cell_max <= 328.15 K; eps_el <= 1 % and eps_mech <= 1 % (D40); RTF >= 4 (accelerator, D56). **Today's script:** the mission repeated for `P.power.budget.t_shift` (3 h) in BOL, EOL and hot variants: completes the duration; SOC_end >= soc_rth = `P.power.budget.soc_window(2)` (0.323); no BMS trip; T_cell_max <= 328.15 K; eps_el, eps_mech <= 1 %; RTF >= 4, BLOCKED without a C compiler (card G2-INT, size L, with the mowing budget of G2-PWR; resolved by D121, 2026-11-23: the compiler is there; the script still marks the RTF BLOCKED, and S3's RTF waits on card G2-INT) |
| S4 Greenhouse coverage | Phase 2. 9.6 x 37.5 m house (`greenhouse_std`), mower 1.0 m swath 10 % sidelap, 0.5 m/s, GNSS zone SVF 0.12, LiDAR localisation | coverage >= 95 % of the floor inset by c_in = 0.50 m (D106: post edge + clearance + the cross-track allowance of D100's 0.15 m maximum; REQ-SYS-121); time <= 22 min (D107); truth cross-track RMS <= 0.05 m and max <= 0.15 m over the whole of every row, entry included (D100); clearance >= 0.125 m to posts and walls, judged with the deck (D87, D100; D106's margin at the 0.15 m tracking maximum, REQ-SYS-087); 0 collisions; scan-match valid > 80 % of attempts; scripted operator resumes under D84 and D92 until the wall holds are fixed (D100). **Today's script:** the inset c_in = 0.40 m (`P.gnc.cov.gh_post_edge` 0.20 + `gh_c_clear` 0.15 + `gh_c_xte` 0.05, `scn_S4_greenhouse.m`; D106's 0.50 m on card G2-GNC); the clearance >= 0.15 m of the body only (the deck is not monitored, R-06) and no scripted resumes. Since D72 the run with the mower holds at the lane-end walls and reaches no row (STATUS 18); the last full run (2026-11-14) passes 6 of 7, failing the max at 0.252 m on a row entry (cards G2-GNC for the footprint and the row entry, G2-INT for the script) |
| S5 Palm patrol + obstacles | Phase 2. 5 aisles x 88 m + 4 turns at 1.0 m/s, no implement; 0.6 m box on aisle 2 appended to `P.world.obstacles.box` by `scn_S5` (occupancy rebuilt, D42); pedestrian (r 0.3 m, 0.8 m/s) crossing aisle 4 (ScenarioIn.obs); VFH bypass (D99, decided, pending on card G2-GNC); a head-on case: a pedestrian walking toward the robot on an aisle (D108, card G2-INT) | 0 collisions; the slow-down judged from the obstacle distance that the design acts on: the target speed starts to fall within 0.5 s of d_obs reaching the distance where v_obs drops below the cruise speed (about 2.7 m at 1.0 m/s); the hold as in T6 (within 0.1 s of d_obs under 1.5 m, at least 1.2 m from the box); clearance >= 0.5 m to the box, until G2-FSC rules on detours round a person; T7's lane bound (2.5 m) and abs(XTE) < 0.1 m within 5 m past the box; >= 1.0 m to a pedestrian who crosses its path and meets the robot, with the operator's resume of D84 (REQ-SYS-133); in the head-on case, the contact speed measured (D108); route time <= 1.25 x nominal (D100). **Today's script:** deceleration start <= 0.5 s after the box enters the 10 m corridor; abs(XTE) < 0.1 m within 3 m; the hold not checked; the pedestrian never meets the robot (32.7 m in the last run, 2026-10-23) and no scripted resume (card G2-INT; D84 itself on G2-GNC); no head-on case (card G2-INT); the TEB bypass (`VC_TEB` 1 from `P.gnc.variant.teb`) until card G2-GNC |
| S5b Lost scan in AUTO | Phase 2. aisle 1 (104 m) at 1.0 m/s, bypass off; LiDAR blackout 35-38 s on the ROW and 74-82 s during an OBSTACLE_HOLD before a static person (r 0.3 m, present 45-80 s) | on a ROW at the first blackout; SENSOR_LOST hold <= 0.4 s after each blackout starts; still in the hold, SENSOR_LOST shown and stopped while the scan is blank; after each blackout the robot stays held after the scan returns and moves off only on a scripted operator resume (D84, REQ-SYS-035); `fsm.impl_allow` false from the loss until the operator's restart (D92); an OBSTACLE/PERSON hold active at the second blackout; 0 collisions. **Today's script:** the H1 release back to AUTO_MISSION 1.8-2.6 s after the scan returns, with `fsm.impl_allow` false to the end of each hold (`scn_S5_palm_obstacles.m`; card G2-GNC for the hold logic, G2-INT for the script) |
| S6 RC failsafe | Phase 1. 1.0 m/s MANUAL: (a) rc_link_loss at 20 s, back at 25 s; (b) rc_line_silent; (c) byte error rate 0.01 from 10 to 25 s; (d) byte error rate 0.05 from 10 to 25 s | (a),(b) CTRL_STOP <= 0.13 s after loss onset (D37); the controlled stop judged as S8b: mean deceleration over the constant phase 1.0 +/- 0.1 m/s^2 and 100 ms peak <= 1.2 m/s^2; standstill in the MCU's terms (below 0.05 m/s within v0/a_ctrlstop + 0.3 s), then parked on both brakes within 1.0 s; stopped within 0.8 m of the loss (D100); ESTOP never entered (D34); no motion after link return until arm off -> on. (c) no command glitch > 2 % FS, corrupted frames rejected, MANUAL kept. (d) no stick glitch > 2 % FS, corrupted frames rejected, CTRL_STOP <= 0.2 s after RC_LINK_LOST, ESTOP never entered. **Today's script:** (a),(b): deceleration (100 ms moving average) <= 1.05 m/s^2 and standstill (< 0.01 m/s) <= 1.5 s after detection, with no stop-distance or park check; S6a and S6b fail on both (known: 1.099 m/s^2 and 1.61 s, 2026-11-15). The deceleration band is confirmed against a coasting stop when the script changes (card G2-INT) |
| S7 GNSS dropout | Phase 2. straight 88 m aisle at 1.0 m/s, 30 s outage then 10 s float; variants: farm soil, wet loam, 10 K enclosure temperature ramp (D49) | position error at outage end <= 0.25 m (REQ-SYS-020); heading error <= 2 deg; truth XTE <= 0.3 m during outage; within REQ-SYS-020's open-field values (0.15 m across, 0.25 m along) within 5 s of the RTK fix; NEES in 95 % bound >= 90 % of samples (REQ-SYS-124); at least 20 m driven during the outage; 0 collisions (all variants). **Today's script:** the position error at outage end <= 0.5 m, and < 0.05 m within 5 s of the RTK fix (card G2-INT) |
| S8 E-stop | Phase 1. 1.2 m/s MANUAL; (a) front E-stop at 10 s, released at 15 s, reset at 17 s, re-arm at 19-20 s; (b) remote E-stop CH12 at 10 s | (a) STO <= 50 ms after the plant-side hmi edge (Plant_In output, D62; REQ-SYS-063, provisional under D96); MCU fault bit HW_ESTOP <= 11 ms after the DI change (1 ms DI delay); stop distance <= 0.6 m; no motion (abs(v) <= 0.02 m/s) from 2 s after ESTOP entry until reset + re-arm; moves again after the re-arm. (b) ESTOP <= 0.2 s after the switch; mean deceleration over the constant-deceleration phase 2.0 +/- 0.1 m/s^2 (100 ms moving-average peak <= 2.4); stop distance from the CH12 switch <= 0.70 m (replay 0.62 m incl. 92 ms worst-case latency, D33); ESTOP latched. **Today's script:** (a) STO <= 20 ms, tighter than REQ-SYS-063: a regression check on the twin's 17 ms chain (no change needed) |
| R1 RC row test | Phase 1. `scn_R1_row_test` through `phase1_row_test`: a simulated driver drives the robot by RC up and down crop rows in world 7 (row_field): 2-30 rows, 1-240 min, ambient -10 to 55 C, no implement or the sprayer (pump on until the tank is empty), full detail or the fast preset | a free test: no pass criteria; the row-test report (procedure NASL-SHD-TP-001) records the run |
| L1 Live drive | Phase 1. `scn_L1_live_drive` through `phase1_live_drive`: a real transmitter in USB joystick mode, the dashboard or a demo plan; 3-D view, real-time pacing; any world and implement | a free test: no pass criteria (an interactive drive) |
| D1 Test drive | Phase 1. `scn_D1_test_drive` through `phase1_test_drive`: a manoeuvre plan [t v w] converted to stick positions, within the RC limits; optional remote E-stop, reset and RC-loss events | a free test: no pass criteria |

---

## 10. Open questions and risks

### 10.1 Open questions
- **OQ-01 Implements ownership.** Mower, tiller and sprayer actuator models are not assigned to a domain. Proposal: `drive` owns the implement motors and pumps (electrical port `DC48`); `structure` owns implement mass, drag force and yaw moment. `subsystems/structure/Implements.slx` sits on both sides. Fallback: integration builds a simple Simscape load (Controlled Current Source from `P_impl(t)`) plus a drag-force signal. **Settled by D9:** structure owns the load mechanics and power owns the electrical drive and the implement ECU; they meet at `Impl_Shaft`.
- **OQ-02 Environment ownership.** This spec assigns Env_Terrain to integration under `subsystems/integration/`. The folder is new and must be added to the project path in Stage 0. **Settled:** the environment and the worlds are the structure domain's (`params/param_world.m`, `scripts/build/build_structure_env.m`); D10 dropped integration's Env_Terrain.
- **OQ-03 Plant as a Model Reference.** Confirm with all physical domains. Simscape logging inside referenced models is not relied on (meas buses instead). Changing variants inside the plant forces a plant-target rebuild of about 1-3 min.
- **OQ-04 Track and terramechanics formulation** (Simscape vs Simulink continuous vs discrete). It decides direct feedthrough and the ode3 stiffness constraint (`|lambda|` <= `P.sim.lambda_max_explicit` = 1000 1/s, a time constant >= 1 ms; section 7.4).
- **OQ-05 STM32 part.** H7 has a double-precision FPU; F4/G4 are single-precision only. This sets interface bus types (single assumed). The registered boards include `STM32 Nucleo H743ZI2`, `STM32H7xx Based (Single-core)`, `STM32G4xx Based`, `STM32F4xx Based`. **Settled for the twin:** the STM32H743 is the firmware's development target and the robot's delivered controller is the baseline, provisionally (D96).
- **OQ-06 Companion computer and OS** (e.g. Jetson Orin or x86 IPC, ROS 2 distribution). This sets the GNC HW variant and its codegen toolchain. **Settled:** NVIDIA Jetson Orin NX 16 GB, JetPack 6, ROS 2 Humble (`P_meta.hw.companion`).
- **OQ-07 P field names.** This spec adds the top-level fields `P.sim`, `P.frames`, `P.env`, `P.viz`, `P.log`, `P.scen`, `P.ic` to the global convention. **Settled:** `P.sim`, `P.frames`, `P.viz`, `P.log`, `P.scen`, `P.world` and `P.ic`; there is no `P.env` (the environment is `P.world.env`).
- **OQ-08 Dual-antenna GNSS heading** (moving baseline): yes or no. **Settled:** yes, the dual ZED-F9P moving base (`P.sensors.gnss.rAnt1`, `rAnt2`).
- **OQ-09 Spring-applied motor brakes** (affects S2 hold and S8 stop distance). **Settled:** yes, a spring brake on each of the four motors (5 N m per motor [EST], `P.drive.motor_unit.brake_T`).
- **OQ-10 SBUS receiver failsafe mode** (hold / no-pulses / preset) and the high-speed (7 ms) vs normal (14 ms) frame period. **Settled:** the normal 14 ms frames (`P.rc.frame_period`) and the preset failsafe, which must not command the remote E-stop (`P.rc.failsafe_raw`, checked by `shaddad_params`).
- **OQ-11 3D LiDAR fidelity in missions.** Proposal: 2D or virtual-2D in missions, 3D only in perception tests.
- **OQ-12 Simulink Coverage and Simulink Check** are licensed but their toolbox folders are missing. Install them or drop coverage and Model Advisor gates.

### 10.2 Risks
- **R-01 Simulation speed.** 4 h at 1 ms is 14.4 M steps. Any sub-millisecond dynamics (PWM switching, stiff slip states, a fine thermal mesh) or an interpreted MATLAB System at 1 kHz will collapse RTF. Mitigation: fidelity variants, the per-step budget (Section 7.7), T-INT-03 gates in the fast suite, Simulink Profiler and Simscape statistics.
- **R-02 Compiler.** No MEX compiler is selected; only the bundled LCC-win64 is available. Accelerator and rapid-accelerator builds will be slower and some features may be limited. Install the "MATLAB Support for MinGW-w64 C/C++ Compiler" add-on. STM32 deployment additionally needs STM32CubeMX and the GNU Arm toolchain (STM32 blockset hardware setup). Verify before Stage 6. **Resolved by D121 (2026-11-23), but for the STM32 tools:** MinGW64 14.2.0 is the C and C++ compiler, from the MATLAB support package, so the add-on install is done; with Smart App Control off since 2026-11-22, accelerator and rapid accelerator builds run on this PC. STM32CubeMX and the GNU Arm toolchain are still missing (card B-00).
- **R-03 Algebraic loops** between direct-feedthrough controllers and local-solver Simscape outputs. Mitigation: compute-delay Unit Delays, the 1 ms pose delay into Env_Terrain, `AlgebraicLoopMsg = error`.
- **R-04 Rate-transition semantics** differ between single- and multitasking. Mitigation: `EnableMultiTasking = 'on'` in every model, deterministic RTs, no auto-inserted RTs.
- **R-05 Frame and sign mistakes.**
  - The MATLAB accelerometer convention is the negative of specific force.
  - `mspsensorlib` GPS and IMU default to NED.
  - UAV Animation defaults to Z down.
  - Flat-Earth blocks are NED-only.
  - Pitch is positive nose-down in FLU.
  Mitigation: Section 3 rules and T-INT-04.
- **R-06 Parallel authoring conflicts** (shared `.slx` files, buses). Mitigation: the file-ownership table, per-domain bus and param files, interface-first stubs (Stage 1), `shaddad_check_interfaces`, and never editing someone else's file.
- **R-07 Interpreted MATLAB System blocks** (viz, any sensor model left in interpreted mode) prevent rapid accelerator and slow accelerator runs. Mitigation: `VC_VIZ = 0` in batches and `SimulateUsing = 'Code generation'` for sensor and collision blocks.
- **R-08 Dashboard binding** works only to blocks in the top model's own hierarchy, not inside model references. Mitigation: dashboard and manual constants live in the top model's `Operator_Console`; status comes from logged lines in the top model.
- **R-09 Inlined parameters** (`DefaultParameterBehavior = Inlined`) make P changes trigger accelerator rebuilds. Mitigation: use `Tunable` plus Fast Restart for sweeps; keep structural changes (map sizes, Ns) out of sweeps.
- **R-10 Large data.** Logging LiDAR scans and paths at high rate can exhaust memory. Mitigation: 1 Hz decimation, log to file, SDI archive cap.
- **R-11 Codegen.**
  - No double types in the MCU (F4/G4).
  - Fixed-size arrays only (SBUS buffer uint8[16]).
  - Enums need `Simulink.IntEnumType` with a defined storage type.
  - The HW variant needs `update diagram` activation so only one IO choice is compiled.
  - SBUS is inverted UART: H7/G4/F7 USART supports RX inversion, F4 needs an external inverter (electronics).
- **R-12 Harness support for SSRs with Connection Ports** is unconfirmed in R2026b. Mitigation: hand-built benches for physical SSRs, plus a Stage 1 smoke test.
- **R-13 Shared MATLAB session.** Startup and shutdown must not use `clear all`, `bdclose all` or `cd`. Test runners load and close only the models they opened.
