# GNC Specification: Guidance, Navigation-Control, Path Planning and Mission Management

> **Revision B note (2026-10-19).** `docs/architecture.md` section 2b (D30-D63) and `docs/build_plan.md`
> override this spec where they differ. Sections changed for this domain: command envelope from the MCU limits: v_max 1.5 m/s then (1.2 m/s today, `P.electronics.lim.v_max_auto`), |v| + |omega| chi B/2 <= v_track_max (D43); MPC prediction model with a 2-sample input dead time (D41); d_stop_req with t_lat 0.35 s (D33); v_work_palm 1.0 m/s and vegetation speed limit (D46); omega limited by tilt, ALIGN inhibited above 10 deg, no headland turns on slopes > 10 deg (D53); greenhouse inset c_in = 0.45 m (D54; 0.40 m with today's geometry, and 0.50 m by D106, decided, pending on card G2-GNC; 2.7); no `plan` root port, MissionSource Constant inside Shaddad_GNC (D57); `fsm.rc_required_in_auto` removed, the MCU owns RC-loss behaviour (D44); TEB behind `VC_TEB`, interpreted-only, 10 x 10 m window (D56); packaged in `Shaddad_Companion` (D38).

> **Critfix (2026-11-15).** Sections changed for this domain: 2.6 (scan-loss entry and the valid-scan release), 2.9 (implement latch), 4.1 (T5, H1 to H3 and the scan input), 5.4 (Bus_ObsFlags scan_valid), 5.5 (HoldReason 10 SENSOR_LOST), 6 (fsm.t_scan_lost, tb.scn_scan).

> **G1-01 part 2 (2026-11-18).** Brought in line with the parameters of the robot as built (2026-10-27 onward: 1.0 x 0.78 m, track gauge 0.60 m; 226 kg with the default 1 kW motors of D95, 220 kg as built with the four 750 W motors, 291 kg with the mower) and the decisions D1 to D110 (D104, D105, D109 and D110 change nothing in this spec; D106, D107 and D108 are marked where they apply). Sections changed: 0, 1.2, 2.2 to 2.9, 3.1, 3.3, 4.1, 4.4, 5.2, 5.5, 5.7, 6, 8 and 9. A value that is decided but not yet in the parameters is marked "decided, pending on card G2-GNC", next to the value the parameters hold today: D84 (the operator's resume after a person stop), D87 (pivots only where the footprint fits), D92 (after a protective stop, the blades restart on the operator's command), D93 (no return from the operator's pause on a low charge), D99 (VFH the default bypass, and the bypass gates) and D106 (the greenhouse inset widened to 0.50 m). D107 makes S4's greenhouse within 22 min a requirement (2.7). The values come from `params/param_gnc.m` and the leaves it reads. The fix pass of step 19 (2026-11-18) corrected the hold triggers (2.6, 4.1), D92's scope (2.9, 4.1), the interfaces left from before D19, D20, D23, D38 and D57 (0, 1.2, 3.3, 4.4, 5.1, 5.2, 5.5, 7, 9), the mission-file keys (5.7), the file and variant names (4, 6, 7, 8, 9) and T9's criterion (8). It also brought in REQ-SYS-088's navigation share (6, 9), REQ-SYS-122's stop margin (4.1, 6), D108's longer person field on open ground (2.6, 9) and the one stop model of the lead's ruling 5a (2.5).

Project: Shaddad_UGV (tracked skid-steer agricultural UGV: 226 kg with the default 1 kW motors (D95; 220 kg as built with the four 750 W motors), 291 kg with the mower, 316 kg with the full-tank sprayer; `P.structure.mp.m0` by configuration, with the 1 kW motors)
Domain key: `gnc`. Phase 2 autonomy. Runs on the companion computer as a Model Reference.
MATLAB R2026b. Every library path marked "verified" below was confirmed in the live session with `get_param(...,'BlockType')` / `DialogParameters`.

---

## 0. Findings from the live MATLAB session that drive this design

These are the findings of the first design session (2026-10), on the earlier 600 kg robot (1.4 x 1.1 m, track gauge 0.85 m). They are kept as the design record; the values of today's robot are in Sections 2 and 6.

| # | Finding (verified) | Consequence for the design |
|---|---|---|
| F1 | `license('test','Stateflow')` = 1, but Stateflow is not in `ver` and there is no `Stateflow 26.2 *.xml` in `matlabroot/appdata/products`. Only the shared Stateflow runtime that ships with Simulink is present (`sflib.slx` exists). | **The Stateflow product is not installed.** The mission manager is a MATLAB Function block FSM with Simulink enumerations (Section 4.1). It is laid out as a transition table so it can be ported 1:1 to a Stateflow chart if Stateflow is installed later. Simulink Coverage and Simulink Design Verifier: licence test = 1 but not installed, so no formal coverage. `sltestlib/Test Sequence` and `sltestlib/Test Assessment` are available through Simulink Test. |
| F2 | navlib has only three algorithm blocks: `navlib/Control Algorithms/{Pure Pursuit, Timed Elastic Band (TEB), Vector Field Histogram}`. There is no global path planner block in any installed library. | Hybrid A* replanning goes in a MATLAB Function (`plannerHybridAStar` has `%#codegen`). Local obstacle bypass uses the existing TEB block. |
| F3 | In R2026b the **Pure Pursuit block outputs `Linear Velocity` and `Curvature`, not angular velocity**. Optional ports: `Desired Linear Velocity`, `Lookahead Distance` (in) and `Target Direction` (out). | omega_cmd = v_cmd x Curvature (Product block). Target Direction feeds VFH `TargetDir`. |
| F4 | `drivinglib/Velocity Profiler` (System object `driving.internal.planning.VelocityProfiler`) gives **one cruise speed per direction-segment, set by the segment's maximum curvature**. Tested: a 39 m row followed by an R = 0.81 m turn in one call gave v = 0.493 m/s along the whole row. On a row by itself it gave the expected jerk-limited ramp 0 -> 2.0 -> 0.3 m/s. | Velocity profiling is done **per mission segment** (row, turn and transit separately), with end speed = entry speed of the next segment. The current-segment and next-segment geometry windows are smoothed by two separate Path Smoother Spline blocks. |
| F5 | `drivinglib/Path Smoother Spline` and `drivingvehiclecontroller/Lateral Controller Stanley` use **degrees** for heading. Smoother on 200 to 800 poses took 0.22 s (interpreted, first call). | Gain blocks for deg<->rad at the block boundaries. The smoother runs only on segment change, inside a Triggered Subsystem. |
| F6 | `uavCoverageSpace` / `uavCoveragePlanner` (UAV Toolbox, `%#codegen`) plan boustrophedon rows. For a 40 x 9 m greenhouse with UnitWidth 0.9 and Sidelap 0.1: 26 waypoints, lane spacing 0.81 m, plan time 0.65 s. Headland turns are not modelled. | Coverage is planned **offline in the mission compiler**. It uses `uavCoveragePlanner` for lanes, then `dubinsConnection` for headland turns: an R = 0.4 m U-turn is 1.27 m (LSL); an R = 1.2 m turn on 0.81 m spacing is 7.79 m (RLR bulb). |
| F7 | `plannerHybridAStar` on a 40 x 20 m map at 0.1 m resolution with a 4 x 4 m obstacle, inflation 0.75 m, R_min 1.5 m: path found in 1.2 s (interpreted, first call), 36.9 m. | Replanning is event-triggered with a 1-1.5 s budget. On target it must run in its own (slow) task. |
| F8 | Offline MPC check (mpc + mpcmove, kinematic error model with yaw-rate lag tau = 0.25 s (true plant 0.30 s), RTK noise 1.5 cm / 0.3 deg, slope drift 0.03 m/s): settles from 0.3 m offset in 1.3-2.3 s, overshoot 1.3-1.4 cm, cross-track RMS 5-6 mm, mean < 1 mm at v = 0.4/0.8/1.2/2.0 m/s. A heading-error weight >= 1 gives a steady-state offset (2-9 cm) under drift, so it must stay small (0.1). | MPC row tracker settings are fixed in Section 6. Pure Pursuit has steady-state offset e_ss ~ L_d * d / v (5.6 cm for L_d = 1.5 m) and Stanley e_ss ~ d / k (3.75 cm), so neither meets the 5 cm requirement on side slopes. **Adaptive MPC is the default row tracker.** |
| F9 | TEB block ports: in `Pose, Vel, MapInfo, RefPath`; out `VelCmds, Timestamps, OptPath, SolnInfo`. MapInfo is a fixed-size struct `{Resolution, GridLocationInWorld, GridOriginInLocal, GridSize, OccupancyMatrix}`. Outputs are variable-size, up to 10000 rows. SolnInfo has `HasReachedGoal, TrajectoryCost, LastFeasibleIdx, ExitFlag`. | `Bus_Perception.map` (Bus_TebMapInfo) carries exactly these fields, in TEB's verified order Resolution, GridLocationInWorld, GridSize, OccupancyMatrix, GridOriginInLocal (D23). TEB runs only in the BYPASS sub-mode. |
| F10 | Block names under Model Verification contain newlines, e.g. `sprintf('simulink/Model\nVerification/Check \nStatic Range')`. | Build scripts must use `sprintf` with `\n` in these paths. |

---

## 1. Purpose and scope

### 1.1 In scope (owned by `gnc`)
1. **Mission manager**: top-level autonomy state machine IDLE / MANUAL / AUTO_MISSION / PAUSE / OBSTACLE_HOLD / RETURN_TO_HOME / FAULT, with AUTO sub-modes ALIGN, TRANSIT, ROW, TURN, BYPASS, REJOIN, RTH_TRANSIT, DOCK_APPROACH, DOCKED. Also fault supervision for autonomy (nav integrity, geofence, stuck, tilt, tracking error, MCU link, BMS) and SOC-based return-to-dock.
2. **Mission definition and compilation**: field map / waypoint file format (JSON), and the offline mission compiler (coverage rows, headland turns, transit, dock approach) that produces the fixed-size `Bus_MissionPlan`.
3. **Path serving and segment preparation**: windowing the mission path, spline smoothing, and per-segment velocity profiling.
4. **Path analysis**: closest point, cross-track error, heading error, curvature preview, progress.
5. **Path following**: Pure Pursuit for transit, turns and RTH; Adaptive MPC (default) or Stanley (variant) for in-row precision; pivot ALIGN law.
6. **Speed planning / governor**: curvature, slope, obstacle distance, navigation accuracy, SOC and mode limits, plus accel/jerk shaping.
7. **Local obstacle handling**: corridor check, VFH free-direction monitor, TEB local bypass.
8. **Global replanning**: Hybrid A* on an occupancy map for RTH, rejoin after manual/pause, and lane-skip.
9. **Implement manager**: mower / tiller / sprayer commands keyed to segment type and mode, including spray rate vs speed.
10. **Command output** to the STM32 (Bus_GncCmd plus heartbeat), status/telemetry (Bus_GncStatus), and verification metrics (XTE RMS etc.).

### 1.2 Out of scope (owned by neighbours)
| Neighbour domain | What it owns | Interface with gnc |
|---|---|---|
| `sensors` | RTK GNSS, IMU, encoders and LiDAR models; **sensor fusion EKF/INS** producing the navigation state; LiDAR pre-processing (ground/vegetation removal); local occupancy grid; object tracks; vision spray demand | Produces Bus_NavState, Bus_Perception (D23: the filtered scan, the protective stop used as person_near, and the TEB map, in one bus) and Bus_SprayDemand |
| `electronics` (STM32 MCU firmware) | RC/SBUS arbitration and the final mode authority; (v, omega) -> track-speed mixing; yaw-rate and speed loops; implement drivers; E-stop chain; companion<->MCU link (CAN-FD or UART) | Consumes Bus_GncCmd; produces Bus_McuStatus |
| `rc` | SBUS decoding and channel mapping (mode switch, aux mission buttons) | Mapped into Bus_McuStatus.rc_* by the MCU |
| `power` | Battery pack and BMS (SOC, faults, energy) | Produces Bus_BmsStatus; parameter `P.power.pack.E_Wh` |
| `drive` | Motors, gearboxes, tracks, terramechanics, slip | No direct signals. Its closed-loop yaw-rate / speed time constants set the MPC prediction-model parameters (P.gnc.mpc.tau_r) |
| `structure` | Geometry and mass | Read-only parameters: `P.structure.geom` (L_overall, W_overall, B, the contact length L), `P.structure.impl`, `P.structure.descent` |

GNC has **no Simscape physical ports**. It is purely discrete-time signal processing.

---

## 2. Algorithmic model

### 2.1 Frames and conventions
- World frame: local **ENU**. The origin is the mission datum (RTK base station or field corner). x = East, y = North, z = Up. Yaw psi is measured counter-clockwise from East, in (-pi, pi].
- Body frame: FLU (x forward, y left, z up), origin at the track-centre footprint centroid. LiDAR scans must arrive in this frame (`base_link`).
- Automated Driving blocks (Path Smoother Spline, Stanley) take poses as `[x y theta_deg]`. Navigation Toolbox objects (Pure Pursuit, VFH, TEB, Hybrid A*) use radians.
- Signed cross-track error e_y is positive when the robot is **left** of the path. Heading error e_psi = wrap(psi - psi_ref).

### 2.2 Vehicle model assumed by guidance (planning/prediction level)
Unicycle with slip disturbances (skid-steer):

```
x_dot   = v cos(psi) - v_y sin(psi)
y_dot   = v sin(psi) + v_y cos(psi)            v_y: lateral drift (side slope, track slip), |v_y| <= 0.05 m/s
psi_dot = r
r_dot   = (r_cmd - r)/tau_r                    closed-loop yaw-rate response of MCU + drive, tau_r ~ 0.25 s
v_dot   = (v_cmd - v)/tau_v                    tau_v ~ 0.35 s
v_L,R   = v -/+ r * chi * B/2                  B = 0.60 m track gauge, chi = 1.394 skid-steer gauge factor (P.electronics.mix)
```
Track-speed feasibility, enforced in CommandShaping (revision B, D43): |v| + |omega| * chi * B/2 <= v_track_max = 1.414 m/s (`P.electronics.lim.v_track_max`) with chi and B from `P.electronics.mix` (chi 1.394, B 0.60 m: chi B/2 = 0.418 m), the same mixer geometry as the MCU.

**Descent envelope (card G2-DSC, D74).** CommandShaping's `DescentEnv` (`gnc_descentEnvelope`) scales the speed to the cap at the nav pitch: a forward speed above cap(pitch), or a reverse speed beyond cap(−pitch), is scaled to the cap, with omega scaled by the same factor. A pivot (v = 0) is not limited. The tiller's cap is 0 in reverse at every slope. §7 lists the subsystems.

### 2.3 Path error dynamics (Frenet, used by MPC and PathAnalyzer)
For a reference path of curvature kappa_ref:
```
e_y_dot   = v sin(e_psi) + v_y               ~ v e_psi + d
e_psi_dot = r - v kappa_ref / (1 - kappa_ref e_y)  ~ r - v kappa_ref
```
MPC prediction model (scheduled on v; discretised by ZOH at Ts = 0.05 s on every step):
```
x = [e_y; e_psi; r],  u = [r_cmd (MV); kappa_ref (MD); d (UD)],  y = [e_y; e_psi]
Ac = [0 v 0; 0 0 1; 0 0 -1/tau_r]
Bc = [0 0 1; 0 -v 0; 1/tau_r 0 0]
C  = [1 0 0; 0 1 0],  D = 0
```
The unmeasured input disturbance d (lateral drift) is modelled as a random walk (integrator, gain `mpc.ud_gain` 0.04 in scaled units). This makes the tracker offset-free in e_y on side slopes (F8). The model as built adds the 2-sample input dead time (D41) as two states, x = [e_y; e_psi; r; u(k-1); u(k-2)] (`mpc.nx` 5), and two white unmeasured channels on e_y and e_psi (`mpc.w_ey` 0.7, `mpc.w_epsi` 0.01) for the jumps at a row entry, so the model has 5 inputs (`mpc.nu`).

### 2.4 Path-following laws
- **Pure Pursuit** (transit, turns, RTH, rejoin): kappa = 2 sin(alpha)/L_d, omega = v kappa. Lookahead is scheduled as L_d = interp(v; [0 0.3 0.6 1.2 2.0] -> [0.6 0.8 1.0 1.5 2.0] m). Pure Pursuit is exact on constant-radius arcs at steady state, so headland turns are tracked well. On straight rows with drift d it leaves e_ss ~ L_d d / v.
- **Stanley** (variant ROW_STANLEY): delta = e_psi + atan(k e_y / v), with a virtual front axle L_v = 0.375 m (`stan.Lv`, half the 0.75 m track contact length), so omega = v tan(delta)/L_v. Linearised: e_y_ddot + (v/L_v) e_y_dot + (k v/L_v) e_y = 0, giving omega_n = sqrt(k v / L_v) and zeta = sqrt(v / (4 k L_v)). With k = 0.8 and L_v = 0.375: zeta = 0.58 / 0.82 / 1.00 at v = 0.4 / 0.8 / 1.2 m/s. The block takes k_block = k (1 + v_nom)/v_nom = 1.8 (`stan.k_block`, v_nom 0.8 m/s), which gives this law at v_nom. Steady-state offset under drift is d/k.
- **Adaptive MPC** (variant ROW_MPC, default): min sum ||W_y (y_k)||^2 + ||W_du du_k||^2 subject to |r_cmd| <= 1.0 rad/s and |du| <= 0.1 rad/s per step. The plant model is updated each step from the measured v. Bumpless transfer: the MPC external-MV inport receives the omega actually applied in the previous step (Unit Delay), so its estimator stays consistent while Pure Pursuit is active.
- **ALIGN (pivot)**: when |e_psi| > 45 deg at the start of a segment that allows pivoting, with the tilt within `align_tilt` (10 deg, D53): v = 0, omega = sat(1.5 e_psi, +/-0.6 rad/s), until |e_psi| < 8 deg. The same pivot serves the resume (P1, above 30 deg); the trim at the end of the dock approach (R4, above 2 deg) pivots too, without the flag or the tilt check.

### 2.5 Speed planning
Per segment: v_prof(s) from the Velocity Profiler on the current segment, with StartVelocity = measured v and EndVelocity = seg.v_end. seg.v_end = min(v_seg(next), sqrt(a_lat_max / kappa_max(next))).
Runtime governor:
```
v_gov = min( v_prof(s), v_seg_max, v_obs(d_obs), v_xte(|e_y|), v_head(|e_psi|),
             v_pitch(|theta|), v_roll(|phi|), v_nav(sigma_pos), v_soc(SOC), v_mode(state), v_override )
v_cmd = RateLimiterDynamic(v_gov, +a_acc*Ts, -a_dec*Ts)     a_acc = 0.5 m/s^2, a_dec = 1.0 m/s^2
```
Stopping-distance consistency: d_stop_req = v t_lat + v^2/(2 a_dec), with t_lat = 0.35 s (revision B: GNC -> ESC about 0.06 s + jerk ramp about 0.2 s + margin) and a_dec = `P.electronics.shaper.a_dec_auto` (1.0 m/s², now achievable because the ESC ramp no longer caps it). The v_obs table [0 1.5 2.0 3.0 4.0 6.0] m -> [0 0 0.6 1.2 1.6 2.0] m/s keeps the robot stopping at least 1.5 m from the obstacle for every entry. Example: at d = 3 m and v = 1.2 m/s, d_stop_req = 0.42 + 0.72 = 1.14 m < 1.5 m. This logged check (`spd.t_lat`) is a design check of the v_obs table. The margins that rest on the robot's stop (REQ-SYS-029, -035, -046, -058) use the one stop model of the lead's ruling 5a, which REQ-SYS-029 states: REQ-SYS-132's detection time of 0.25 s, the GNC step and the link to the ESC, then the AUTO stop the robot makes, from a replay of the MCU's shaper (1.0 m/s^2, jerk 5 m/s^3, K_v 4, the ESC's 40 ms lag): about 0.90 m from 1.2 m/s, about 1.33 m in all from a person entering the field (0.36 s at 1.2 m/s is 0.43 m, then the 0.90 m stop; REQ-SYS-029's statement of the model, the lead systems engineer's ruling of step 19, point 9). On that model the v_obs table still holds: from 1.2 m/s at d = 3 m the robot stops about 1.67 m from the obstacle, outside the 1.5 m hold distance.

**Descent limit (card G2-DSC, D74).** v_pitch(|theta|) = min(LUT(|theta|), `gnc_descentLimit`). The limit is the cap at the nav pitch, held at its peak over `t_hold` 1.0 s, together with the minimum over k of sqrt(c_k² + 2·a_la·max(0, d_k − v·t_resp)) along the planned path (the DEM chord look-ahead, which sees segments A and B). cap(theta) = max(0, table[implement_type + 1, first breakpoint ≥ theta − tol] − dv), where `table` is `P.structure.descent` (forward) and dv = `P.electronics.lim.descent.dv` (0.10 m/s, so AUTO plans to the MCU's speed target). The limit is forward only; `Bus_GncDebug.v_limits(6)` carries it. Parameters: §6, `desc`.

**Inputs (card G2-DSC).** The GNC reads the same table and the same rule as the MCU, from the same parameters, but not all the same inputs:
- **Pitch:** the GNC reads the navigation pitch, which has no level calibration (review item R-21, open). The MCU reads `fw_imu_tilt`, which subtracts its level calibration `pitch0` (−0.03714 rad, `params/param_electronics.m`).
- **Implement:** the GNC takes the plan's `implement_type`. The MCU takes the DI IMPL_ID coding plug, and reads an IMPL_FB of another type as unknown.
- **Unknown implement:** the GNC takes the lowest of the four implement rows, without the raised-deck row. The MCU's unknown row is the lowest of all its rows, the raised deck included.
- **Binding:** the MCU's cap binds every command. So these differences are not a safety gap: the GNC's plan can differ from what the MCU applies, and the MCU caps it anyway. No system run drives an AUTO descent (the S2 family runs in RC); an AUTO descent check is open on card G2-GNC.

### 2.6 Obstacle corridor (pure Simulink blocks, 10 Hz)
For each LiDAR beam i (range r_i, angle a_i, in body frame): x_i = r_i cos a_i, y_i = r_i sin a_i.
```
in_i  = (x_i > x_f) & (x_i < x_f + L_c) & (|y_i| < w_c)       x_f = 0.50 m (front bumper, obs.x_f), L_c = 6 m, w_c = 0.54 m (W/2 + 0.15)
d_obs = min_i( in_i ? (x_i - x_f) : Inf )
```
As built (T15) the corridor follows the path, not the body axis: the beam points are tested against a tube of half-width w_c around `obs.n_tube` 31 path samples from the robot to x_f + L_c ahead along the path (window A, then B), and d_obs is the along-path distance of the nearest sample in reach, less x_f. A body-frame box would see the greenhouse walls in every headland turn and curved transit. The tube has the body's width only; the implement's reach (the mower deck, 1.20 m ahead of the centre) is not in it (R-06, card G2-GNC).

OBSTACLE_HOLD is entered when d_obs < 1.5 m, when VFH SteerDir is NaN for 0.2 s (`fsm.t_vfh`), when person_near is set, or when the scan is lost (invalid for more than `protNStale` scans, 0.2 s; SENSOR_LOST takes priority). The d_obs term is off in a bypass, its rejoin and the dock approach; there a person, a lost scan or a blocked VFH (0.2 s) still holds the robot, so a blocked VFH also holds D99's VFH bypass. In RTH_PLANNING and DOCKED, where the robot is stopped, R6 does not apply and no obstacle condition holds it (`gnc_missionFSM.m`: `dObsOn`, `obsHit`, R6). It is released when d_obs > 2.0 m, VFH is clear and no person is seen for 1.9 s (`fsm.t_clear_eff`) on valid scans. After a person or a lost scan in the hold, D84 makes the release wait for the operator's resume (decided, pending on card G2-GNC). The person field is the perception's (REQ-SYS-131: 0.5 m + 0.8 s x |v| beyond the front of the body or the fitted implement). By D108 (decided, pending on cards G2-FSC and G2-GNC) it grows ahead on open ground (palm groves, transits) to at least 3.4 m at 1.0 m/s and 4.0 m at 1.2 m/s (REQ-SYS-131, the upper ends of D108's 3.0 to 3.4 m and 3.6 to 4.0 m), for a person walking head-on; in a greenhouse it stays as it is (D79). The planning side is question 13 of section 9.

### 2.7 Coverage path generation (offline mission compiler)
1. Convert the boundary from LLA to ENU with `lla2enu`. Inset it by c_in = gh_post_edge + gh_c_clear + gh_c_xte + max(0, W_chassis/2 - w_tool/2) (revision B, D54); D54 set 0.20 + 0.15 + 0.05 + 0.05 = 0.45 m with the earlier 1.1 m wide body. With today's geometry (W 0.78 m, the mower's 1.00 m swath, so the last term is 0) the compiler computes **0.40 m**: the outer lane centre is 0.90 m from the wall line, the body edge 0.51 m (0.31 m clear of the 0.20 m post envelope) and the edge of the 1.05 m wide deck 0.375 m (0.175 m clear). The validation keeps the footprint of the body and the deck at least `cov.clear_min` 0.15 m from the posts and walls along the whole plan (testCompilerGreenhouse). gh_c_xte allows 0.05 m of cross-track error, below the 0.15 m maximum that D100 sets for the rows (REQ-SYS-027, -087). D106 (decided, pending on card G2-GNC): gh_c_xte becomes 0.15 m, the whole of D100's maximum, so c_in = 0.50 m: the outer lane centre is 1.00 m from the wall line, the body edge 0.61 m (0.41 m clear of the post envelope) and the deck edge 0.475 m (0.275 m clear, about 0.125 m at the 0.15 m tracking maximum). Coverage is measured on the floor inset by c_in.
2. If the boundary is concave, split it with `coverageDecomposition`.
3. Plan lanes: `uavCoverageSpace(Polygons, UnitWidth = w_tool, Sidelap, UnitLength = 0.1)` with `setCoveragePattern(...,'SweepAngle', theta_rows)`, then `uavCoveragePlanner(...,Solver="MinTraversal")`.
4. Order lanes. If lane spacing s_l < 2 R_turn, use **skip-row** order (1,3,5,...,6,4,2). The single adjacent switch-back is a Dubins bulb (RLR).
5. Join lane ends with `dubinsConnection(MinTurningRadius = R_turn)`. Add a transit from home to the first lane, a transit from the last lane to the dock-approach start, and a 3 m straight dock-approach segment.
6. Resample every segment to exactly N_raw = 100 poses. Compute length, kappa_max, v_seg and v_end. Write Bus_MissionPlan.

Time estimate: T = sum(L_row/v_row) + N_turn (L_turn/v_turn + 2 t_ramp) + T_transit.
Greenhouse cross-check, the S4 house (`greenhouse_std`, world 4: 9.6 x 37.5 m; the mower's 1.00 m swath, sidelap 0.10, lane spacing 0.90 m, R 0.81 m): 10 lanes; the compiler estimates 13.0 min and T8 runs it in 13.2 min (2026-11-16). The first design took a 40 x 9 m house with a 0.9 m mower: 12 lanes, about 19-20 min, against a published benchmark of about 22 min. S4 keeps 22 min as its upper limit, a requirement since D107 (verified by S4). T8 takes 25 min, with the run within 15 % of the estimate. The 0.50 m inset of D106 moves the outer lanes 0.10 m inward; the estimate is recomputed when card G2-GNC changes the inset.

### 2.8 Return-to-home energy threshold
```
E_home   = k_tort * d_home * e_m * k_margin        k_tort = 1.4, e_m = 0.12 Wh/m, k_margin = 1.3
soc_rth  = soc_reserve + E_home / E_nom            soc_reserve = 0.20
```
e_m (`fsm.e_m`) is 0.12 Wh/m today. It has no recorded source (review GNC-14) and disagrees with the row-test energy (R-19), so it is to be derived from a phase-2 row test (cards G2-GNC and G2-PWR); the first design took 0.24 Wh/m for the 600 kg robot. With E_nom = 2948 Wh (`P.power.pack.E_Wh`) and d_home = 500 m, soc_rth = 0.20 + 1.4 x 500 x 0.12 x 1.3 / 2948 = 0.237 (T12, 2026-11-16). The reserve 0.20 is D101's. This is computed with blocks (Sum, Product, Gain, hypot via Math Function).

### 2.9 Implement laws
- Spray flow: Q [L/min] = R_app [L/ha] x v [m/s] x w_boom [m] x 0.006. Example: 300 L/ha x 0.8 x 1.2 x 0.006 = 1.73 L/min. Spraying is enabled only on ROW segments with v >= 0.3 m/s. section_mask = vision_mask AND enable.
- Tiller: raise when s_rem < 0.5 m on a ROW segment and during TURN/TRANSIT. Lower once 0.3 m into the ROW.
- Mower: blades run on ROW and TURN. Blades stop on TRANSIT, PAUSE, FAULT and MANUAL (in MANUAL the MCU runs them from the RC implement switch), after more than 3 s in OBSTACLE_HOLD, and immediately on person_near. D92 (decided, pending on card G2-GNC): after a stop for a protective reason (an emergency stop, a person or lost scans, any other hold or pause, a tilt stop, a fault, a raised deck) the blades restart only on the operator's deliberate command; the planned starts within a mission (the rows and turns after a transit) stay automatic (REQ-SYS-050). Today the blades restart by themselves on ROW and TURN, after H1 and after P2. The robot will not move (v_gov = 0) until mcu.implement_ready is true (blade speed above 90 % of the target, `P.power.impl.ecu.ready_frac`), with a 10 s timeout leading to PAUSE.
- **Mower deck (card G2-DSC, D81):** the deck is commanded to `impl.mower_h` in every state, and the GNC never raises it. No motion until the deck counts as down: 6.35 s of AUTO or RTH since power-up or MANUAL (`fsm.t_deck`, §6). Until then the robot holds with `IMPLEMENT_NOT_READY` (hold reason 6, `buses/HoldReason.m`) and v_cap is 0.
- **Implement latch (critfix 2026-11-15, R-03):** once person_near is seen in an OBSTACLE_HOLD (at entry or later), or the scan is lost in it, `impl_allow` stays false for the rest of that hold: the mower and the tiller rotor stay off and the tiller stays raised. H1 back to ROW or TURN restarts them; the latch is renewed at each hold entry. The immediate person_near stop and H4 (3 s) are unchanged; the sprayer is already off in any hold (`spray_on` needs AUTO_MISSION). D84 makes the latch end only on the operator's resume (decided, pending on card G2-GNC).

---

## 3. Block list (exact library paths)

### 3.1 Algorithm blocks (all verified in the R2026b session)

| Block | Library path | Used in | Key mask parameters (real names) and values |
|---|---|---|---|
| Pure Pursuit | `robotmobilelib/Pure Pursuit` (same System object as `navlib/Control Algorithms/Pure Pursuit`, `nav.slalgs.internal.PurePursuit`) | PathTracker | `DesiredLinearVelocityPort='on'`, `LookaheadDistancePort='on'`, `TargetDirPort='on'`, `MaxCurvature='P.gnc.pp.kappa_max'` (2.5), `SimulateUsing='Code generation'`. Ports: in `Pose` [x y psi], `Waypoints` [799x2], `Desired Linear Velocity`, `Lookahead Distance`; out `Linear Velocity`, `Curvature`, `Target Direction` |
| Vector Field Histogram | `navlib/Control Algorithms/Vector Field Histogram` | LocalAvoidance (0.1 s) | `NumAngularSectors=180`, `DistanceLimits=[0.3 8]`, `HistogramThresholds=[3 10]`, `RobotRadius='P.gnc.vfh.RobotRadius'` (0.634 = r_circ), `SafetyDistance=0.3`, `MinTurningRadius=0.5`, `TargetDirectionWeight=5`, `CurrentDirectionWeight=2`, `PreviousDirectionWeight=2`, `SimulateUsing='Code generation'`. Ports: `Ranges`, `Angles`, `TargetDir` -> `SteerDir` (NaN = blocked). D99: VFH steers the default bypass (VC_TEB 0; decided, pending on card G2-GNC: the parameters still select TEB, `P.gnc.variant.teb` 1) |
| Timed Elastic Band (TEB) | `navlib/Control Algorithms/Timed Elastic Band (TEB)` | LocalAvoidance / Bypass (Enabled Subsystem, 0.2 s) | `ControlMode='Path Following with Obstacle Avoidance'`, `SampleTime='P.gnc.teb.Ts'` (0.2), `LookAheadTime=4`, `GoalTolerance=[0.1 0.1 0.087]`, `MaxVelocity=[0.6 0.6]`, `MaxReverseVelocity=NaN`, `MaxAcceleration=[0.5 1.0]`, `MinTurningRadius=0.5`, `ReferenceDeltaTime=0.3`, `NumIteration=3`, `Time=10`, `Smoothness=1000`, `Obstacle=50`, `Shape='Rectangle'`, `Dimension='P.gnc.teb.Dimension'` ([1.0 0.78], L x W), `FixedTransform=se2` (default), `ObstacleSafetyMargin=0.3`. Ports: `Pose`, `Vel`, `MapInfo`, `RefPath` -> `VelCmds`, `Timestamps`, `OptPath`, `SolnInfo`. D99: a development option, kept out of system runs until it passes T7 |
| Path Smoother Spline (x2: current, next) | `drivinglib/Path Smoother Spline` | PathServer/SegmentPrep (triggered) | `NumSmoothPoses=400`, `MinSeparation=1e-3`, `SampleTime=-1` (it must inherit inside the triggered subsystem; the default 0.5 is wrong here), `ShowOptionalOutports='on'`. Ports: `RefPoses` [100x3, deg], `RefDirections` [100x1] -> `Poses` [400x3, deg], `Directions`, `CumLengths`, `Curvatures` |
| Velocity Profiler | `drivinglib/Velocity Profiler` | PathServer/SegmentPrep (current segment only, see F4) | `MaxSpeed='P.gnc.spd.v_max'` (1.2), `MaxLongitudinalAccel=0.5`, `MaxLongitudinalDecel=1.0`, `MaxLongitudinalJerk=1.0`, `MaxLateralAccel=0.3`, `ShowOptionalOutport='off'`, `SampleTime=-1`, `SimulateUsing='Code generation'`. Ports: `Directions`, `CumLengths`, `Curvatures`, `StartVelocity`, `EndVelocity` -> `Velocities` |
| Adaptive MPC Controller | `mpclib/Adaptive MPC Controller` | PathTracker/RowTracker variant ROW_MPC | `mpcobj='mpcRowObj'`, `x0=''`, `md_inport='on'`, `mv_inport='on'` (external MV), `return_qpstatus='on'`, `BlockDataType='double'`, `SampleTimeInherited='off'`, `MaximumP='P.gnc.mpc.p'` (40). Ports: `model`, `mo or x`, `ref`, `md`, `ext.mv` -> `mv`, `qp.status` |
| Lateral Controller Stanley | `drivingvehiclecontroller/Lateral Controller Stanley` (reached from `drivinglib/Vehicle Control`) | PathTracker/RowTracker variant ROW_STANLEY | `ModelType='Kinematic bicycle model'`, `PositionGainF='P.gnc.stan.k_block'` (1.8), `PositionGainR='P.gnc.stan.k_block'`, `Wheelbase='P.gnc.stan.Lv'` (0.375), `MaxSteeringAngle=60`. Dynamic-only parameters stay at default: `YawRateGain`, `DelayGain`, `VehicleMass`, `Lf`, `Lr`, `TireStiffness`. Ports: `RefPoses`, `CurrPoses` (deg), `CurrVelocity`, `Direction` -> `SteerCmd` (deg) |
| Moving RMS | `dspstat3/Moving RMS` | StatusMetrics | `Method='Sliding window'`, `SpecifyWindowLength='on'`, `WindowLength=200` (10 s at 20 Hz), `SimulateUsing='Code generation'` |

### 3.2 Simulink infrastructure blocks (all verified)

| Block | Library path | Typical settings |
|---|---|---|
| MATLAB Function (x4, see Section 4) | `simulink/User-Defined Functions/MATLAB Function` | Parameter data `P` (non-tunable); bus-typed I/O |
| Variant Subsystem | `simulink/Ports & Subsystems/Variant Subsystem` | `VariantControlMode='expression'`, `VariantActivationTime='update diagram'` (`'code compile'` if both trackers must be in the code) |
| Model | `simulink/Ports & Subsystems/Model` | Top model: `ModelName='Shaddad_GNC'`, `SimulationMode='Accelerator'` for long missions |
| Subsystem Reference | `simulink/Ports & Subsystems/Subsystem Reference` | One per GNC sub-block (Section 7) |
| Triggered Subsystem | `simulink/Ports & Subsystems/Triggered Subsystem` | `TriggerType='either'` on the window-version counter (SegmentPrep, Replanner) |
| Enabled Subsystem | `simulink/Ports & Subsystems/Enabled Subsystem` | TEB bypass (enable = sub-mode BYPASS); XTE metrics (enable = ROW) |
| Rate Transition | `simulink/Signal Attributes/Rate Transition` | `Integrity='on'`, `Deterministic='on'`, `OutPortSampleTime` = destination rate |
| Selector | `simulink/Signal Routing/Selector` | `NumberOfDimensions=2`, `IndexOptions='Starting index (port),Select all'`, `OutputSizes='[100]'` (mission window extraction) |
| Multiport Switch | `simulink/Signal Routing/Multiport Switch` | `DataPortOrder='Specify indices'`, `DataPortIndices` = enum values (path source, tracker select) |
| Vector Concatenate | `simulink/Signal Routing/Vector Concatenate` | Builds the 799x2 Pure Pursuit window |
| 1-D Lookup Table | `simulink/Lookup Tables/1-D Lookup Table` | Speed-governor limits, L_d(v), omega_max(v). `InterpMethod='Linear point-slope'`, `ExtrapMethod='Clip'` |
| MinMax | `simulink/Math Operations/MinMax` | `Function='min'`, `Inputs=11` (speed governor) |
| Rate Limiter Dynamic | `simulink/Discontinuities/Rate Limiter Dynamic` | v_cmd accel/decel; omega rate (+/-2 rad/s^2) |
| Saturation / Saturation Dynamic | `simulink/Discontinuities/Saturation`, `.../Saturation Dynamic` | omega_cmd within +/-omega_max(v); align omega within +/-0.6 |
| Detect Change | `simulink/Logic and Bit Operations/Detect Change` | Trigger SegmentPrep on [seg_idx, path_src, replan_ver] |
| Compare To Constant / Relational Operator / Logical Operator / Bitwise Operator | `simulink/Logic and Bit Operations/...` | Corridor test, implement logic, fault word |
| Enumerated Constant | `simulink/Sources/Enumerated Constant` | `OutDataTypeStr='Enum: GncState'` etc. |
| Counter Free-Running | `simulink/Sources/Counter Free-Running` | `NumBits=8`, heartbeat `seq` |
| Unit Delay / Delay | `simulink/Discrete/Unit Delay`, `simulink/Discrete/Delay` | Loop breaking (FSM feedback, MPC ext.mv) |
| Product, Gain, Sum, Bias, Abs, Trigonometric Function, Math Function, Sqrt | `simulink/Math Operations/...` | Corridor geometry, deg<->rad, omega = v*kappa, Stanley delta -> omega, track-speed scaling |
| Bus Creator / Bus Selector, In/Out Bus Element | `simulink/Signal Routing/...`, `simulink/Ports & Subsystems/In Bus Element`, `.../Out Bus Element` | Bus I/O |
| Assertion / Check Static Range | `simulink/Model Verification/Assertion`; `sprintf('simulink/Model\nVerification/Check \nStatic Range')` | Runtime sanity checks (omega, v bounds) in SIM |

### 3.3 Harness and deployment blocks (verified)

| Block | Library path | Use |
|---|---|---|
| Differential Drive Kinematic Model | `robotmobilelib/Differential Drive Kinematic Model` | GNC harness plant surrogate: `VehicleInputs='Vehicle Speed & Heading Angular Velocity'`, `TrackWidth='P.structure.geom.B'` (0.60), `WheelRadius='P.structure.r_sprocket'` (0.09), `InitialState=[x0;y0;psi0]` |
| Transfer Fcn First Order | `simulink/Discrete/Transfer Fcn First Order` | Harness v / r lags: `PoleZ=exp(-Ts/tau)` |
| Test Sequence / Test Assessment | `sltestlib/Test Sequence`, `sltestlib/Test Assessment` | Scenario stimuli and pass/fail (Section 8) |
| Signal Editor | `simulink/Sources/Signal Editor` | Mission command scenarios |
| ROS 2 Subscribe / Publish | `ros2lib/Subscribe`, `ros2lib/Publish` | Deployment wrapper (mission upload, status) |
| ROS 2 Read Scan / Read Occupancy Grid | `ros2lib/Read Scan`, `ros2lib/Read Occupancy Grid` | Deployment wrapper (`MaximumArrayLength=360`, `MaximumGridSize=[300 300]`) |
| ROS 2 CAN Transmit / Receive | `ros2lib/CAN Transmit`, `ros2lib/CAN Receive` | Not used: classic CAN only (`MsgLength <= 8`, verified, D38). The companion <-> STM32 link is CAN-FD (D19), through `Companion_IO`'s `ros2lib/Subscribe` `/can2/rx` and `ros2lib/Publish` `/can2/tx` and a SocketCAN-FD bridge node (D38; ICD 3.1) |

### 3.4 Evaluated and rejected (verified to exist)
- `uavalgslib/Waypoint Follower`, `uavalgslib/Path Manager`, `uavalgslib/Obstacle Avoidance`: UAV-centred (3-D, course/altitude). Pure Pursuit and VFH fit ground use better.
- `mpclib/Automated Driving/Path Following Control System`: Ackermann bicycle model with coupled longitudinal control. Not suited to skid-steer, and the speed loop belongs on the MCU.
- `drivingvehiclecontroller/Longitudinal Controller Stanley`: outputs car accel/decel commands. The speed loop is on the MCU.
- `mpclib/Multiple MPC Controllers`: valid fallback if a MATLAB-Function-free MPC is required (speed-binned controllers at v = 0.3/0.6/0.9/1.2/1.6/2.0 m/s).

### 3.5 MATLAB functions used offline (no block; MATLAB code at mission-compile time)
`uavCoverageSpace`, `uavCoveragePlanner`, `coverageDecomposition`, `dubinsConnection`, `lla2enu`, `jsondecode`, `mpc`, `setindist`, `setoutdist`, `c2d`. These are used in `scripts/gnc_compileMission.m` and `scripts/gnc_designMPC.m`.

---

## 4. MATLAB Function blocks (only where no block exists)

All four are code-generation compatible. Their bodies live in `subsystems/gnc/<name>.m` and are called from the MATLAB Function block (one line), so the code can be diffed and unit tested.

### 4.1 `gnc_missionFSM`: mission manager (base rate 0.05 s)
**Why no block:** the Stateflow product is not installed (F1). A Switch Case / Unit Delay network for 7 states, 10 sub-modes and about 30 guarded transitions with timers would be unreadable and hard to verify. The MATLAB Function keeps the state in persistent variables typed as Simulink enumerations.

Signature:
```matlab
function [fsm, faultWord] = gnc_missionFSM(nav, mcu, bms, mcmd, obsFlags, pe_z, plan, rp, tebInfo, socRth, P)
% nav      Bus_NavCond   (conditioned nav: pose, v, pitch, roll, nav_ok, nav_degraded)
% mcu      Bus_McuStatus
% bms      Bus_BmsStatus
% mcmd     Bus_MissionCmdCond (edge-detected command from HMI or RC aux)
% obsFlags Bus_ObsFlags  (d_obs, blocked_vfh, person_near, scan_valid; scan_valid = obs.scan_valid, the scan validity)
% pe_z     Bus_PathError delayed by one step (Unit Delay -> no algebraic loop)
% plan     Bus_MissionPlan (header + segment table only; path table not passed)
% rp       Bus_ReplanResult (ok, ver, n, busy, fail_count)
% tebInfo  TEB SolnInfo (ExitFlag, LastFeasibleIdx, HasReachedGoal)
% socRth   double, from block math (2.8)
% fsm      Bus_FsmOut: state, sub, seg_idx, nxt_idx, path_src, trk_sel, mode_req, cmd_valid,
%          impl_allow, replan_req, replan_goal[3], replan_kind, hold_reason, pause_reason,
%          mission_done, s_obs, timers
```

Transition table (evaluated top to bottom every step; the first match fires):

| ID | From | Condition | To | Entry action |
|---|---|---|---|---|
| G1 | any != FAULT | `mcu.estop_active` OR MCU status age > 0.2 s OR `bms.bms_critical` OR (state in {AUTO, RTH, HOLD} AND (nav lost > 0.3 s, where nav_ok also needs the position sigma under `fsm.sigma_lost` 0.30 m, OR outside the boundary grown by 0.5 m for 0.2 s OR pitch > 25 deg OR roll > 18 deg)) OR (state in {AUTO, RTH} AND (stuck: a command of at least 0.3 m/s with a measured speed under 0.05 m/s for 5 s, OR abs(e_y) > 0.5 m, or no valid path error, for 3 s on TRANSIT, ROW, TURN, RTH_TRANSIT or DOCK_APPROACH)) | FAULT | v = 0, implements off, set faultWord bit |
| G2 | any not in {FAULT, MANUAL} | `mcu.rc_mode_sw == MANUAL` AND `mcu.rc_link_ok` | MANUAL | remember `missionActive`, cmd_valid = false |
| T1 | IDLE | cmd START AND rc_sw == AUTO AND nav_ok AND plan.valid AND SOC > 0.35 AND `mcu.drive_ready` | AUTO_MISSION | seg = 1; sub = ALIGN if abs(e_psi) > 45 deg on a segment that permits a pivot (seg column 10) with the tilt within `align_tilt`, else sub(seg.type); the same rule at every segment entry (S1). The bench option `fsm.auto_start` (false) starts without the command |
| T2 | IDLE | cmd RTH AND rc_sw == AUTO AND nav_ok AND `mcu.drive_ready` AND plan.valid | RETURN_TO_HOME | replan_req(goal = dock-approach start) |
| T3 | MANUAL | rc_sw != MANUAL | PAUSE if missionActive else IDLE | |
| T4 | MANUAL | rc link lost (MCU failsafe already braking) | IDLE (or PAUSE if missionActive) | warning bit |
| T5 | AUTO_MISSION | d_obs < 1.5 m (not in BYPASS, a bypass rejoin, DOCK_APPROACH or DOCKED) OR blocked_vfh for 0.2 s OR person_near OR scan lost (invalid for more than `protNStale` scans, 0.2 s debounce) | OBSTACLE_HOLD | hist = AUTO; record s_obs; spray off; reason SENSOR_LOST > PERSON > VFH_BLOCKED > OBSTACLE (SENSOR_LOST shown while the scan is lost) |
| T6 | AUTO_MISSION | cmd PAUSE OR rc_sw == HOLD | PAUSE | reason OPERATOR |
| 36 | AUTO_MISSION, OBSTACLE_HOLD, RETURN_TO_HOME (not DOCKED) | `mcu.drive_ready` false for `fsm.t_drive_lost` (0.5 s), OR `drive_ready` false AND `rc_link_ok` false (at once) | PAUSE | reason RC_LOSS when the RC link is lost, else OPERATOR; no auto resume. The timer ignores the ESC unpark: a parked ESC (drive spec 2.9) reports OPERATION, so `drive_ready` holds while parked, and the next command passes ENABLING (`t_premag` + `t_release` = 0.09 s), a 0.1-0.15 s drop at the 50 ms MCU status. The MCU owns the safety reaction (RC loss D44, E-stop, CTRL_STOP); E-stop and a lost MCU link are G1. Before 2026-11-14 T36 fired at once and stopped every AUTO mission once D66 let the ESCs park |
| T7 | AUTO_MISSION, RTH | nav_degraded for > 2 s (`fsm.t_nav_degr`). InputConditioning: nav_degraded = navValid AND (quality < `q_good` (neither RTK fixed nor LiDAR-localised) OR cross-track sigma on OR along-track sigma on OR odometry inconsistent). Cross- / along-track sigma = sqrt(n'Cn) / sqrt(t'Ct): C the E-N position covariance (posStd(1:2), covPose(1,2)), t / n the tangent / normal of the path heading `pe.psi_ref` one 0.05 s step late; max(posStd(1:2)) for both without a valid path error. Hysteresis (Relays): cross-track on at >= `sigma_pause` (0.15 m), off at <= `sigma_degr` (0.10 m); along-track on at >= `sigma_along` (0.25 m), off at <= `sigma_along_off` (0.20 m) | PAUSE | reason NAV; autoResume = P.gnc.fsm.auto_resume_nav. Before 2026-11-14 the sigma term was max(posStd(1:2)) > 0.10 m: in the S4 rows the along-aisle sigma, which the scan match cannot observe well in a self-similar aisle, crossed it every 2-3 m (19 pauses in the Nov 13 run) |
| T8 | AUTO_MISSION | SOC < socRth before the final transit home OR cmd RTH | RETURN_TO_HOME | replan_req(goal = dock-approach start) |
| T9 | AUTO_MISSION | cmd ABORT | IDLE | |
| T10 | AUTO_MISSION | `mission_done` (last segment complete) | RTH if end_action == RTH else IDLE | |
| S1 | AUTO/sub any | pe_z.s >= seg.length - 0.05 m | seg++ ; sub = sub(seg.type) | new window (SegmentPrep triggers) |
| S2 | AUTO/ALIGN | abs(e_psi) < 8 deg | sub(seg.type) | |
| S3 | AUTO/BYPASS | end of the detour path reached AND d_obs > 2.0 m (`fsm.d_release`) | sub(seg.type), path_src = MISSION | TEB disabled |
| S4 | AUTO/BYPASS | TEB ExitFlag infeasible OR LastFeasibleIdx < 3 (`fsm.teb_lfi_min`), for 3 s (15 calls of 0.2 s, `fsm.teb_t_bad`) | sub REJOIN (wait rp) | replan_req(goal = path point at s_obs + 4 m), kind LOCAL |
| S5 | AUTO/REJOIN | rp.ok AND replan path end reached | sub(seg.type), path_src = MISSION | |
| (REJOIN) | AUTO/REJOIN, waiting for rp (not a bypass rejoin, nor a rejoin resumed from a paused bypass) | LOCAL plan failed | sub REJOIN (wait rp) | replan_req(same goal), kind GLOBAL (no lane mask, R_min 1.5 m); only a failed GLOBAL plan counts as a try (4.4). BYPASS, its S4 rejoin and a rejoin resumed from a paused bypass stay LOCAL (lane half-width, 8 T7) |
| H1 | OBSTACLE_HOLD | clear (d_obs > 2.0 m AND ~blocked AND ~person) for 1.9 s (`t_clear_eff`) on valid scans; a missed scan restarts the clear timer | hist (AUTO or RTH) | D84 (decided, pending on card G2-GNC): after a person or a lost scan in the hold, to PAUSE, and on only on the operator's RESUME |
| H2 | OBSTACLE_HOLD | t_hold > 10 s AND hist AUTO on a ROW AND no person in the field AND seg.allow_bypass AND scan valid for 1.9 s (`t_clear_eff`) AND not (`P.gnc.fsm.no_rev`(implement + 1) AND `teb(1)` ≠ −1) (card G2-DSC: no TEB bypass with an implement that may not reverse; an unknown implement counts as one that may not reverse) | AUTO_MISSION, sub BYPASS | TEB enabled. D99 (decided, pending on card G2-GNC): VFH the default; no automatic bypass after a person or a lost scan in the hold, nor with an implement whose footprint the bypass checks do not cover (the mower included); a blocked lane then holds, pauses (H3) and alerts |
| H3 | OBSTACLE_HOLD | t_hold > 120 s | PAUSE | reason SENSOR_LOST while the scan is lost, else OBST_TIMEOUT; operator alert |
| H4 | OBSTACLE_HOLD | t_hold > 3 s | (stay) | mower off |
| P1 | PAUSE | cmd RESUME AND rc_sw == AUTO AND nav_ok | AUTO_MISSION | paused in BYPASS or REJOIN: always sub REJOIN with replan_req (kind LOCAL; when the pause interrupted a bypass, i.e. paused in BYPASS, in its S4 rejoin or in a rejoin resumed from one, no GLOBAL fallback: the detour stays in the lane, 8 T7); otherwise, if abs(e_y) < 0.3 m and abs(e_psi) < 30 deg: continue; if abs(e_y) < 0.3 m and abs(e_psi) >= 30 deg on a segment that allows pivoting (seg column 10), with tilt <= align_tilt and not on the dock approach: sub ALIGN (pivot in place, then S2; no replan); else sub REJOIN with replan_req (kind LOCAL, GLOBAL fallback). A forward-only LOCAL plan in the lane mask cannot turn the robot round (S4 rejoin at 458.8 s, 175 deg off the lane, 2026-11-13) |
| P2 | PAUSE | reason NAV AND autoResume AND (nav_ok AND NOT nav_degraded) for `fsm.t_nav_rec` (5 s) | AUTO_MISSION | as P1 (continue, ALIGN or REJOIN) |
| P3 | PAUSE | (cmd RTH OR (SOC < socRth, the pause interrupted a mission, before its final transit)) AND rc_sw == AUTO AND nav_ok AND `mcu.drive_ready` AND plan.valid | RETURN_TO_HOME | D93 (decided, pending on card G2-GNC): the low-charge exit goes; the robot holds and tells the operator (REQ-SYS-105) |
| P4 | PAUSE | cmd ABORT | IDLE | |
| R1 | RTH/RTH_PLANNING | rp.ok | sub RTH_TRANSIT, path_src = REPLAN | |
| R2 | RTH/RTH_PLANNING | rp.fail_count >= 3 | PAUSE | reason PLANNER_FAIL |
| R3 | RTH/RTH_TRANSIT | end of replan path | sub DOCK_APPROACH, seg = plan.dock_seg | v_cap 0.2 m/s, row tracker on |
| R4 | RTH/DOCK_APPROACH | end of the dock segment, after an in-place heading trim (ALIGN tracker) while abs(e_psi) > `fsm.dock_tol_yaw` 2 deg, until it is below 1 deg | sub DOCKED | v = 0 |
| R5 | RTH/DOCKED | `mcu.charging` OR 2 s elapsed | IDLE | |
| R6 | RTH (not RTH_PLANNING or DOCKED) | obstacle conditions of T5 | OBSTACLE_HOLD | hist = RTH |
| (HOLD, RTH) | OBSTACLE_HOLD, RETURN_TO_HOME (not DOCKED) | cmd PAUSE OR rc_sw == HOLD; 36 and (in RTH) T7 as in AUTO; cmd ABORT | PAUSE (OPERATOR); IDLE on ABORT | |
| F1 | FAULT | cmd RESET_FAULT AND no active fault condition AND ~estop AND rc_sw != AUTO | IDLE | clear faultWord |

**Deck wait and raised deck (card G2-DSC).** In AUTO, while the mower deck does not yet count as down (§2.9), the robot holds with `IMPLEMENT_NOT_READY` and v_cap is 0. With the deck reported raised, v_cap is also limited by the raised-deck row of the measured pitch (`desc.v_fwd_raised` / `desc.v_rev_raised`). The descent limit of §2.5 limits every GNC command.

**Decided, pending on card G2-GNC (G1-01 part 2).** The table is the FSM as built. The decisions change it: D84, H1 after a person or a lost scan in the hold waits for the operator's resume (REQ-SYS-039, -049); D87, the compiler sets seg column 10 only where the footprint with the fitted implement fits, and a blocked pivot holds (REQ-SYS-091 to -093); D92, after a protective stop the blades restart only on the operator's command, while the planned starts within a mission stay automatic (REQ-SYS-050); D93, P3 loses its low-charge exit (REQ-SYS-015, -105); D99, VFH the default and the bypass gates (REQ-SYS-040, -099). G1-01 part 2 also proposes values per kind of path for the cross-track fault (G1) and the T7 gate (REQ-SYS-022, -026), and a stop inside the boundary (REQ-SYS-025; REQ-SYS-122: at least 0.30 m inside it in a greenhouse, 0.50 m in the open); they enter this table when card G2-GNC changes the parameters.

Output mapping: mode_req = {IDLE: STOP, MANUAL: MANUAL_ACK, AUTO/RTH: AUTO, PAUSE/HOLD: HOLD, FAULT: FAULT_STOP}. cmd_valid is true only in AUTO and RTH (excluding RTH_PLANNING and DOCKED). The geofence uses `inpolygon(x, y, plan.boundary(1:nb,:))` against the boundary grown by 0.5 m (computed offline), for 0.2 s (`fsm.t_geofence`), on the reference point only. REQ-SYS-025 asks for the stop before any part of the robot or its implement passes the boundary (FSE-13). REQ-SYS-122 asks for the stop at least 0.30 m inside it in a greenhouse and 0.50 m in the open, the stricter reading (the lead systems engineer's ruling of step 19, point 10). With REQ-SYS-095's values (point 11: 0.15 m and 0.5 s in a greenhouse aisle, 0.35 m and 0.7 s in the open) the undetected error with the drift before the stop is about 0.24 m in a greenhouse and 0.47 m in the open, inside both margins (decided, pending on card G2-GNC: a predictive check of the footprint over the stop distance, with the margin by kind of site, taken from the plan's mission type).

Pseudo-code skeleton:
```matlab
persistent st sub seg hist tHold tClr tNav tStuck tXte missionActive sObs ...
if isempty(st), st = GncState.IDLE; sub = AutoSub.NONE; seg = uint16(1); ... end
f = computeFaults(nav, mcu, bms, pe_z, st, P);           % bitword, timers advance by P.gnc.Ts
if f ~= 0 && st ~= GncState.FAULT,  [st, sub] = enterFault(f);
elseif rcManual(mcu) && st ~= GncState.FAULT && st ~= GncState.MANUAL, st = GncState.MANUAL;
else
  switch st
    case GncState.IDLE,          ...  % T1, T2
    case GncState.MANUAL,        ...  % T3, T4
    case GncState.AUTO_MISSION,  ...  % T5-T10, then S1-S5 on sub
    case GncState.PAUSE,         ...  % P1-P4
    case GncState.OBSTACLE_HOLD, ...  % H1-H4
    case GncState.RETURN_TO_HOME,...  % R1-R6
    case GncState.FAULT,         ...  % F1
  end
end
fsm = packOutputs(st, sub, seg, ...);   % trk_sel, path_src, impl_allow, mode_req, cmd_valid
```

### 4.2 `gnc_pathAnalyzer`: closest point and tracking errors (0.05 s)
**Why no block:** no library block computes cross-track error, heading error, arc-length progress or a curvature preview on a polyline. Pure Pursuit keeps its projection internal, and the Stanley block needs the reference pose supplied from outside (the Automated Driving examples use a non-library helper).
```matlab
function pe = gnc_pathAnalyzer(pose, PA, sA, kA, vA, PB, sB, kB, winVer, v, P)
% PA [400x3] (x,y,deg), sA/kA/vA [400x1]: current segment; PB/sB/kB: next segment
persistent i0 lastVer
if isempty(i0) || winVer ~= lastVer, i0 = 1; lastVer = winVer; end
% monotonic local search i0-5 .. i0+60 (prevents snapping to the neighbouring lane 0.81 m away)
for i = max(1,i0-5):min(399,i0+60)
    a = PA(i,1:2); b = PA(i+1,1:2); ab = b-a; t = min(max(dot(p-a,ab)/dot(ab,ab),0),1);
    q = a + t*ab; d2 = sum((p-q).^2); keep best (ib, tb, qb)
end
i0 = ib;
psiRef = deg2rad(PA(ib,3)) + t*wrap(deg2rad(PA(ib+1,3)-PA(ib,3)));
xte    = (ab(1)*(p(2)-qb(2)) - ab(2)*(p(1)-qb(1)))/norm(ab);   % + = left
epsi   = wrapToPi(pose(3) - psiRef);
s      = sA(ib) + tb*(sA(ib+1)-sA(ib));   sRem = sA(end) - s;
kRef   = interp(kA); vRef = interp(vA);
for k = 1:P.gnc.mpc.p   % curvature preview for the MPC measured disturbance
    sk = s + max(v,0.2)*k*P.gnc.Ts;
    kPrev(k) = interp1([sA; sA(end)+sB(2:end)], [kA; kB(2:end)], sk, 'linear', kB(end));
end
pe = struct('xte',xte,'epsi',epsi,'psi_ref',psiRef,'kappa_ref',kRef,'v_ref',vRef, ...
            's',s,'s_rem',sRem,'idx',ib,'kappa_prev',kPrev,'xy_ref',qb,'valid',d2 < 4);
```

### 4.3 `gnc_mpcPlantUpdate`: LTV model bus for Adaptive MPC (0.05 s)
**Why no block:** the Adaptive MPC `model` inport needs a discretised plant structure built from the current speed. No library block builds it (the MPC Toolbox examples also use a MATLAB Function).
```matlab
function mdl = gnc_mpcPlantUpdate(v, P)
v  = max(abs(v), 0.2);  tau = P.gnc.mpc.tau_r;  Ts = P.gnc.Ts;
Ac = [0 v 0; 0 0 1; 0 0 -1/tau];   Bc = [0 0 1; 0 -v 0; 1/tau 0 0];
M  = expm([Ac Bc; zeros(3,6)]*Ts);
mdl.Plant   = struct('A',M(1:3,1:3),'B',M(1:3,4:6),'C',[1 0 0;0 1 0],'D',zeros(2,3));
mdl.Nominal = struct('X',zeros(3,1),'U',zeros(3,1),'Y',zeros(2,1),'DX',zeros(3,1));
```
Output type: `Bus_MpcModel` (Plant: A 3x3, B 3x3, C 2x3, D 2x3; Nominal: X 3x1, U 3x1, Y 2x1, DX 3x1).

### 4.4 `gnc_globalReplanner`: Hybrid A* replanning (triggered; 1 s polling rate)
**Why no block:** there is no global path-planner block in navlib, robotmobilelib or uavlib (F2). `plannerHybridAStar`, `binaryOccupancyMap`, `validatorOccupancyMap` and `stateSpaceSE2` all support code generation.
```matlab
function rp = gnc_globalReplanner(req, start, goal, kind, lmap, P)
% kind LOCAL (rejoin/lane-skip): plan on Bus_Perception.map (300x300 @0.1 m, D23), merged with static keep-outs
% kind GLOBAL (RTH): plan on static field grid P.gnc.map.static_occ (400x400 @0.25 m), window of start/goal + 10 m
occ = select/merge grids;   map = binaryOccupancyMap(occ, res);  map.GridLocationInWorld = origin;
inflate(map, P.gnc.hastar.r_infl);                           % 0.69 m (circumscribed radius 0.634 m + 0.06 m)
ss = stateSpaceSE2;  ss.StateBounds = [xl; yl; -pi pi];
sv = validatorOccupancyMap(ss);  sv.Map = map;  sv.ValidationDistance = 0.1;
pl = plannerHybridAStar(sv, MinTurningRadius=1.5 (LOCAL: 1.0), MotionPrimitiveLength=1.2 (LOCAL: 1.0), ...
         ReverseCost=5, DirectionSwitchingCost=10, AnalyticExpansionInterval=3, ...
         InterpolationDistance=0.25, MaxNumNodes=20000);
if ~isStateValid(sv,start), start = backOff(start, 0.5); end  % start inside the inflated zone
ref = plan(pl, start, goal);
rp.ok = ref.NumStates > 1;
rp.path = resampleByArcLength(ref.States, 100);              % [100x3], heading -> deg, dir column
rp.ver = rp.ver + 1;  rp.n = 100;  rp.t_plan = toc-equivalent counter
```
Retry policy (in the FSM): up to 3 attempts, moving the goal 4 m further along the path each time, then PAUSE with PLANNER_FAIL. In a REJOIN after a pause (not a bypass rejoin), a failed LOCAL plan is followed by a GLOBAL plan to the same goal (no lane mask, R_min, window of start and goal + win_margin on the static grid, merged with the perception grid), and only a failed GLOBAL plan counts as an attempt: at most 3 LOCAL + GLOBAL pairs. BYPASS, its S4 rejoin and a rejoin resumed after a pause that interrupted a bypass stay LOCAL, so the detour keeps to the lane half-width (8, T7).

### 4.5 Harness-only MATLAB Function (not in the deployed model)
`gnc_harnessLidar(pose, map)` uses `rangeSensor` on a `binaryOccupancyMap` to produce the 360-beam scan when the `sensors` plant is not connected. It is replaced by the `sensors` domain model in system simulation.

---

## 5. Interfaces

### 5.1 Root inputs of `Shaddad_GNC.slx` (bus-typed root inports)

| Inport | Bus | Producer | Rate | Elements the GNC reads (the full bus: ICD 6) |
|---|---|---|---|---|
| nav_state | Bus_NavState | sensors (Shaddad_NavFusion) | 0.01 s | t s; pos_W double[3] m (ENU); eul double[3] rad (ZYX: yaw CCW from East, pitch + nose down, roll); vel_B double[3] m/s; yawRate rad/s; posStd double[3] m; covPose double[3x3] (pE, pN, psi); quality uint8 (NavQuality); navValid boolean |
| perception | Bus_Perception | sensors (Shaddad_Perception) | 0.1 s | ranges_B single[360] m (NaN = no return) and angles_B single[360] rad, about base_link; scanValid boolean; protectiveStop boolean (used as person_near, D23); map Bus_TebMapInfo (the TEB MapInfo, F9) |
| mcu_status | Bus_McuStatus | electronics (Companion_IO, CAN2 MCU_STATUS) | 0.05 s | rc_mode_sw uint8 (0 MANUAL, 1 HOLD, 2 AUTO); rc_link_ok; rc_aux_cmd uint8 (MissionCmd) and rc_aux_seq; estop_active; drive_ready; implement_ready; charging; heartbeat; v_track_meas single[2] m/s; v_applied m/s; valid |
| bms_status | Bus_BmsStatus | power (Companion_IO, CAN2 MCU_POWER) | 0.1 s | SOC single (0-1); bms_critical boolean; valid boolean |
| mcmd | Bus_MissionCmd | HMI / ground station (sim: the root ScenarioIn.mcmd) | 0.2 s | cmd uint8 (MissionCmd); cmd_seq uint8 (new command on change); speed_override single (0-1); the bus also carries mission_id and end_action, which the GNC takes from the plan |
| spray | Bus_SprayDemand | sensors (vision) | 0.1 s | section_mask uint16; target_density single; valid boolean |

The ports, buses and rates are those of ICD 3.1. There is no `plan` port: the Constant `MissionSource` inside `Shaddad_GNC` reads `P.gnc.mission` (D57). D23 replaced the separate obstacle and local-map buses of the first design with Bus_Perception (NaN = no return, `person_near := protectiveStop`).

**Rate rule (coordination requirement):** every producer rate must be an integer divisor or multiple of 0.05 s. Allowed rates are 0.01, 0.05, 0.1, 0.2 and 1 s. A 0.02 s (50 Hz) producer cannot feed the 0.05 s GNC base rate with a deterministic Rate Transition, because 0.05/0.02 = 2.5.

### 5.2 Root outputs

**Bus_GncCmd**: to `electronics` (STM32), every 0.05 s. Physical units here; `Companion_IO` packs them into CAN2 COMP_CMD 0x100 (D19; see below).

| Element | Type | Dim | Unit / range |
|---|---|---|---|
| seq | uint8 | 1 | rolling heartbeat; MCU declares GNC stale after 200 ms |
| t_stamp | double | 1 | s |
| cmd_valid | boolean | 1 | true only in AUTO / RTH motion |
| gnc_ready | boolean | 1 | GNC healthy (nav_ok, plan valid, no GNC fault), also in IDLE; lets the MCU start AUTO (D20) |
| mode_req | uint8 (GncModeReq) | 1 | 0 STOP, 1 MANUAL_ACK, 2 AUTO, 3 HOLD, 4 FAULT_STOP |
| gnc_state | uint8 (GncState) | 1 | info |
| v_cmd | single | 1 | m/s, declared [-0.5, 1.5] in the bus today (`buses/buses_gnc.m`); [-0.5, 1.2] by D43 as restated (decided, pending on card G2-GNC; the ICD's notes to section 10); commanded within [-0.5, 1.2] today (`cmd.v_min_reverse`, `spd.v_max` = min(2.0, `v_max_auto`)) |
| omega_cmd | single | 1 | rad/s, [-1.2, 1.2], CCW positive |
| accel_limit | single | 1 | m/s^2 (MCU ramp cap) |
| soft_stop_req | boolean | 1 | controlled decel to 0 |
| v_ground_est | single | 1 | m/s, the fused ground speed (the MCU's traction control) |
| v_ground_valid | boolean | 1 | |
| nav_fix | uint8 (NavQuality) | 1 | the navigation quality |
| mower_enable | boolean | 1 | |
| mower_speed_cmd | single | 1 | rad/s (293 = 2800 rpm) |
| deck_height_cmd | single | 1 | m [0.03, 0.12] (`P.structure.impl.mower.h_range`) |
| tiller_enable | boolean | 1 | |
| tiller_speed_cmd | single | 1 | rad/s (26.2 = 250 rpm) |
| tiller_depth_cmd | single | 1 | m [0, 0.12] (`P.structure.impl.tiller.d_max`) |
| spray_enable | boolean | 1 | |
| spray_section_mask | uint16 | 1 | bit i = nozzle section i (8 used) |
| spray_flow_cmd | single | 1 | L/min |
| beacon_cmd | uint8 | 1 | 0 off, 1 auto (amber steady), 2 hold (amber flash), 3 fault (red) |
| buzzer_cmd | boolean | 1 | |

**Packing (D19, settled):** `Companion_IO` packs Bus_GncCmd into CAN2 COMP_CMD 0x100 (32 B, every 50 ms; ICD 5 gives the byte layout and governs): v_cmd, omega_cmd, accel_limit and v_ground_est as single; one flags byte with cmd_valid, gnc_ready, soft_stop_req, v_ground_valid, the three implement enables and buzzer_cmd; mower_speed_cmd u16 [0.1 rad/s], tiller_speed_cmd u16 [0.01 rad/s], deck_height_cmd u8 [1 mm], tiller_depth_cmd u8 [1 mm], spray_flow_cmd u16 [0.01 L/min]; t_stamp is not sent. The 0x210 / 0x211 frames of the first design are not used.

**Bus_GncStatus**: to telemetry / HMI / logging, every 0.2 s: gnc_state uint8, sub_mode uint8, mission_type uint8, mission_id uint16, seg_idx uint16, n_seg uint16, progress single (0-1), area_done_m2 single, xte single m, heading_err single rad, xte_rms single m, v_ref single m/s, dist_home single m, soc_rth single, fault_word uint32, hold_reason uint8, pause_reason uint8, hold_timer single s, replan_count uint16, tracker_active uint8, nav_ok boolean, eta_s single.

**Bus_GncDebug**: logging only, every 0.05 s: pose_ref double[3], kappa_ref, v_prof, v_gov, v_limits single[11] (the inputs to MinMax), omega_pp, omega_row, omega_teb, d_obs, vfh_steer, mpc_qp_status int32, window_ver uint16.

### 5.3 Bus_MissionPlan (owned by gnc; produced by `scripts/gnc_compileMission.m`)
| Element | Type | Dim | Description |
|---|---|---|---|
| valid | boolean | 1 | |
| mission_id | uint16 | 1 | |
| mission_type | uint8 (MissionType) | 1 | 1 PALM_ROW_PATROL, 2 GREENHOUSE_COVERAGE, 3 SOLAR_ROW_MOWING, 4 GOTO |
| implement_type | uint8 (ImplementType) | 1 | 0 NONE, 1 MOWER, 2 TILLER, 3 SPRAYER |
| n_seg | uint16 | 1 | used segments (<= 512) |
| dock_seg | uint16 | 1 | index of the dock-approach segment |
| path | single | [51200 x 4] | per segment 100 rows: x, y (m ENU), theta (deg), dir (+1/-1) |
| seg | single | [512 x 10] | type, dir, length m, v_seg_max m/s, v_end m/s, impl_mode, allow_bypass, lane_halfwidth m, lane_id, pivot_allowed |
| boundary | double | [64 x 2] | ENU vertices of the geofence (grown by 0.5 m) |
| n_bnd | uint8 | 1 | |
| home | double | [1 x 3] | x, y, psi |
| dock | double | [1 x 3] | x, y, psi |
| datum_lla | double | [1 x 3] | deg, deg, m |
| swath | single | 1 | m |
| area_m2 | single | 1 | m^2 (for progress) |
| speed_work | single | 1 | m/s |
| end_action | uint8 | 1 | 0 STOP, 1 RTH |

### 5.4 Internal buses (gnc-owned)
Bus_PathWindow (PA [400x3], sA, kA, vA [400x1], PB [400x3], sB, kB [400x1], ver uint16), Bus_PathError (4.2), Bus_FsmOut (4.1), Bus_ReplanResult (ok, ver, n, path [100x3], fail_count, t_plan), Bus_MpcModel (4.3), Bus_NavCond (pose[3], v, pitch, roll, nav_ok, nav_degraded, sigma_pos), Bus_ObsFlags (d_obs, blocked_vfh, person_near, vfh_steer, scan_valid).

### 5.5 Enumerations (MATLAB enumeration classes derived from uint8, one file each: `buses/<Enum>.m`)
- GncState: IDLE 0, MANUAL 1, AUTO_MISSION 2, PAUSE 3, OBSTACLE_HOLD 4, RETURN_TO_HOME 5, FAULT 6
- AutoSub: NONE 0, ALIGN 1, TRANSIT 2, ROW 3, TURN 4, BYPASS 5, REJOIN 6, RTH_PLANNING 7, RTH_TRANSIT 8, DOCK_APPROACH 9, DOCKED 10
- SegType: NONE 0, TRANSIT 1, ROW 2, TURN 3, DOCK 4, TERMINAL 5
- MissionCmd: NONE 0, LOAD 1, START 2, PAUSE 3, RESUME 4, ABORT 5, RTH 6, RESET_FAULT 7
- GncModeReq: STOP 0, MANUAL_ACK 1, AUTO 2, HOLD 3, FAULT_STOP 4
- TrackerSel: STOP 0, PP 1, ROW 2, TEB 3, ALIGN 4
- PathSrc: MISSION 0, REPLAN 1
- HoldReason: NONE 0, OBSTACLE 1, PERSON 2, VFH_BLOCKED 3, NAV_DEGRADED 4, OPERATOR 5, IMPLEMENT_NOT_READY 6, RC_LOSS 7, OBST_TIMEOUT 8, PLANNER_FAIL 9, SENSOR_LOST 10
- Fault word (uint32 bits): 0 NAV_INVALID, 1 NAV_DEGRADED, 2 MCU_LINK_LOST, 3 XTE_EXCEEDED, 4 STUCK, 5 TILT, 6 BMS_CRITICAL, 7 PLANNER_FAIL, 8 OBST_TIMEOUT, 9 GEOFENCE, 10 MISSION_INVALID, 11 ESTOP, 12 IMPLEMENT_TIMEOUT

### 5.6 Physical (Simscape) ports
None. GNC is a discrete-time controller model.

### 5.7 Mission / field-map input format (`scripts/missions/<site>.json`)
```json
{
  "mission_id": 101, "mission_type": "GREENHOUSE_COVERAGE", "world_id": 4,
  "datum": {"lat": 24.1556, "lon": 47.312, "alt": 430},
  "work_area_lla": [[24.1556,47.312],[24.1556,47.312094461],[24.155938568,47.312094461],[24.155938568,47.312]],
  "rows_lla": [],                  // optional lane centrelines [{"p0":[lat,lon],"p1":[lat,lon]}] (palm/solar)
  "keep_out": [],                  // [{"type":"circle","lat":..,"lon":..,"r":0.15} | {"type":"polygon","lla":[[..],[..],[..]]}]
  "sweep_angle_deg": 90, "swath_m": 1.0, "sidelap": 0.10,
  "turn_radius_m": 0.81, "skip_rows": "auto", "inset": {"post_edge_m": 0.20, "c_clear_m": 0.15, "c_xte_m": 0.05},  // D106: c_xte_m 0.15 (decided, pending on card G2-GNC)
  "speed_work": 0.5, "speed_transit": 1.2, "speed_turn_max": 0.5,
  "implement": {"type":"MOWER","speed_rpm":2800,"height_m":0.06},
  "allow_bypass": false, "lane_halfwidth_m": 0.4,
  "home": {"lat":24.155581943,"lon":47.312047231,"yaw_enu_deg":90},
  "dock": {"lat":24.155563886,"lon":47.312047231,"yaw_enu_deg":90,"approach_m":3.0},
  "via_in_lla":  [{"lat":24.155602709,"lon":47.312047231,"yaw_enu_deg":90}],
  "via_out_lla": [{"lat":24.155602709,"lon":47.312047231,"yaw_enu_deg":-90},
                  {"lat":24.155577429,"lon":47.312047231,"yaw_enu_deg":-90}],
  "end_action": "RTH"
}
```
The keys are those that `scripts/gnc_compileMission.m` reads and documents in its header; the example is `greenhouse_std.json` (world 4). Explicit `rows_lla` (surveyed palm-tree lanes or solar-table aisles; test missions may give `rows_enu`) override the planner-generated lanes. `yaw_enu_deg` is the ENU yaw, CCW from East (ICD 1), not a compass heading; `geofence_lla` (default: the world's boundary) and `transit_radius_m` (default: `turn_radius_m`) are optional. The compiler validates the result: every pose must clear the keep-outs by at least W/2 + 0.30 m, segment count <= 512, and every turn must satisfy R >= R_min. It writes `P.gnc.mission` (Bus_MissionPlan struct) and `P.gnc.map.static_occ` (static occupancy grid, 400x400 @0.25 m).

---

## 6. Parameters (`params/param_gnc.m` returns struct `gnc`, assembled as `P.gnc`)

| Field | Value | Unit | Source / justification |
|---|---|---|---|
| **Timing** | | | |
| Ts | 0.05 | s | 20 Hz is about 20x the yaw-loop bandwidth (tau_r 0.25 s, about 0.6 Hz); equals the MPC Ts |
| Ts_avoid | 0.1 | s | LiDAR scan rate 10 Hz |
| Ts_teb | 0.2 | s | TEB optimisation cost; 5 Hz is adequate at a 0.6 m/s bypass speed |
| Ts_status | 0.2 | s | telemetry |
| Ts_plan | 1.0 | s | replanner polling; event-triggered |
| **Geometry (read from P.structure)** | | | |
| L, W, B_track | 1.0, 0.78, 0.60 | m | `P.structure.geom.L_overall`, `W_overall`, `B`: the robot as built |
| x_front | 0.50 | m | L/2, front bumper offset from origin |
| r_circ | 0.634 | m | sqrt(0.5^2 + 0.39^2), the body's circumscribed radius (the mower deck reaches 1.31 m, R-06) |
| **Path window** | | | |
| N_raw | 100 | - | poses per segment in the mission table |
| N_smooth | 400 | - | per smoother (0.1 m spacing on 40 m rows, 0.5 m on 200 m rows) |
| S_max | 512 | - | max segments (solar about 40 aisles x 4 passes x 2, plus margin) |
| seg_end_tol | 0.05 | m | segment completion tolerance |
| **Speed** | | | |
| spd.v_max | min(2.0, P.electronics.lim.v_max_auto) = 1.2 | m/s | D43: the MCU's AUTO limit binds (REQ-SYS-029) |
| spd.v_work_palm / v_work_greenhouse / v_work_solar | 1.0 / 0.5 / 0.8 | m/s | palm 1.0 (set by D46's shift budget, which D101 superseded; to be confirmed by D101's phase-2 mowing energy run); greenhouse 0.5 from the 22 min benchmark of the first design (2.7); the mission file's speed_work governs |
| spd.v_transit | min(1.5, v_max_auto) = 1.2 | m/s | D43 |
| spd.v_turn_max | 0.5 | m/s | the default turn speed; a mission's speed_turn_max (0.8 in the palm missions) governs its turns |
| spd.v_dock | 0.2 | m/s | the docking speed (REQ-SYS-058) |
| spd.a_acc / a_dec | 0.5 / 1.0 | m/s^2 | a_acc for a load-carrying tracked vehicle on soil; a_dec = `P.electronics.shaper.a_dec_auto`, the MCU's AUTO deceleration (D33) |
| spd.jerk | 1.0 | m/s^3 | Velocity Profiler |
| spd.a_lat | 0.3 | m/s^2 | limits track shear on turns: R = 0.81 m gives 0.49 m/s |
| spd.obs_d / obs_v | [0 1.5 2 3 4 6] / [0 0 0.6 1.2 1.6 2.0] | m, m/s | stopping-distance check, 2.5 |
| spd.xte_x / xte_v | [0 0.1 0.3 0.5] / [2 2 0.5 0.3] | m, m/s | slow down when off-track |
| spd.head_x / head_v | [0 10 30 60] deg / [2 2 0.5 0.2] | rad, m/s | |
| spd.pitch_x / pitch_v | [0 10 15 20 25] deg / [2 1.2 0.8 0.5 0] | rad, m/s | the 20 deg design slope (charter); the descent cap (`desc`) binds downhill |
| spd.roll_x / roll_v | [0 8 12 15 18] deg / [2 1.0 0.6 0.3 0] | rad, m/s | rollover margin (track gauge 0.60 m, CoG 0.27 m above the body origin without an implement, `P.structure.mp.rcg0`) |
| spd.nav_x / nav_v | [0 0.05 0.10 0.30] / [2 2 0.5 0] | m, m/s | RTK-float degradation |
| spd.soc_low / v_soc_low | 0.15 / 0.6 | -, m/s | energy saving |
| desc.theta_bp / v_fwd / v_rev / v_fwd_raised / v_rev_raised | read from `P.structure.descent` (12 breakpoints; rows = implement + 1: 0 none, 1 mower, 2 tiller, 3 sprayer; the raised rows 1 × 12); nothing copied | deg, m/s | card G2-DSC; one source, so the GNC and the MCU read the same table |
| desc.tol / t_hold / n_hold | 1e-9 / 1.0 / 20 | rad, s, - | breakpoint tolerance; the nav pitch is held at its peak for 1 s [EST] (n_hold at Ts 0.05 s) |
| desc.L_c / ds / d_look / n_look | `P.structure.geom.L` (0.75) / 0.25 / 4.25 / 18 | m, m, m, - | the track contact length, over which the body pitch is the DEM chord; the look-ahead spacing and length |
| desc.a_la / t_resp | 0.5 / 2.0 | m/s², s | the deceleration the look-ahead plans with, and the response time, both [EST] |
| desc.dv | `P.electronics.lim.descent.dv` (0.10; 0 if the field is absent) | m/s | the margin below the cap: AUTO plans to the MCU's speed target |
| desc.no_rev | [0 0 1 0] | - | the user's rule: no reversing with the tiller (rows = implement + 1); also `P.gnc.fsm.no_rev` for H2 |
| **Pure Pursuit** | | | |
| pp.kappa_max | 2.5 | 1/m | R_min 0.4 m |
| pp.Ld_v / Ld | [0 0.3 0.6 1.2 2.0] / [0.6 0.8 1.0 1.5 2.0] | m/s, m | |
| **Stanley (variant)** | | | |
| stan.k / k_block | 0.8 / 1.8 | 1/s | the law's gain; the block's PositionGainF = k (1 + v_nom)/v_nom at v_nom 0.8 m/s; zeta 0.58-1.00 over 0.4-1.2 m/s (2.4) |
| stan.Lv | 0.375 | m | `P.structure.geom.L`/2: the virtual front axle at half the track contact length |
| stan.delta_max | 60 | deg | |
| **MPC (default row tracker; offline-verified, F8)** | | | |
| mpc.Ts / p / m | 0.05 / 40 / 8 | s, -, - | 2 s preview (1.6 m at 0.8 m/s) |
| mpc.tau_r / n_delay | 0.25 / 2 | s / samples | closed-loop yaw-rate time constant plus 2 x 0.05 s input dead time (GNC -> ESC 0.06 s + MCU AUTO shaper 0.05 s, revision B D41); identify from the HiFi loop |
| mpc.W_ov | [1 0.1] | - | e_y, e_psi; a larger e_psi weight causes offset under drift |
| mpc.W_dmv | 1.0 | - | |
| mpc.scale_ov | [0.1 0.1] | m, rad | |
| mpc.u_min / u_max | -1.0 / 1.0 | rad/s | |
| mpc.du_max | 0.1 | rad/s per step | 2 rad/s^2 |
| mpc.ud_gain | 0.04 | - | `setindist`: the drift channel's integrator gain (2.3) |
| mpc.noise | diag([0.15 0.05]) | scaled | RTK sigma 1.5 cm, heading sigma 0.3 deg |
| mpc.w_ey / w_epsi / epsi_max | 0.7 / 0.01 / 0.25 | -, -, rad | white UD channels on e_y and e_psi for the row-entry jumps, tuned with the D41 delay; the soft bound on e_psi (the approach angle at a row entry) |
| mpc.v_min_model / v_design / v_engage | 0.2 / 0.8 / 0.2 | m/s | the model's speed floor; the nominal model of gnc_designMPC; the MPC omega is scaled by min(1, abs(v)/v_engage), so no pivot on the row at standstill |
| **Align** | | | |
| align.enter / exit | 45 / 8 | deg | |
| align.K / w_max | 1.5 / 0.6 | 1/s, rad/s | |
| align.pitch_inhibit / roll_inhibit | 10 / 10 | deg | D53: no ALIGN pivot above 10 deg (`fsm.align_tilt` = pitch_inhibit, against max(abs(pitch), abs(roll))) |
| **Command shaping** | | | |
| cmd.w_max_v / w_max | [0 0.5 1 2] / [1.2 1.0 0.6 0.3] | m/s, rad/s | a_lat <= 0.6 m/s^2 |
| cmd.w_rate | 2.0 | rad/s^2 | |
| cmd.v_track_max | 1.414 | m/s | = `P.electronics.lim.v_track_max`: abs(v) + abs(omega) chi B/2 stays within the MCU's track-speed limit (D43) |
| cmd.v_min_reverse | -0.5 | m/s | = -`P.electronics.lim.v_rev_auto` |
| cmd.w_tilt_x / w_tilt | [0 8 12 16 20] deg / [1.2 1.2 0.6 0.3 0.2] | rad, rad/s | D53: omega_max against max(abs(pitch), abs(roll)) |
| cmd.chi / B | 1.394 / 0.60 | -, m | = `P.electronics.mix`, the MCU's mixer geometry |
| **Obstacles** | | | |
| obs.d_stop / d_release | 1.5 / 2.0 | m | |
| obs.w_c / L_c | 0.54 / 6.0 | m | the corridor tube's half-width W/2 + 0.15 (the body only, R-06) and its reach; the compiler keeps JSON keep-outs W/2 + 0.30 from the path (`cov.clear_keepout`), so they give no false holds |
| obs.t_clear / t_bypass / t_hold_max / t_mower_off | 2 / 10 / 120 / 3 | s | |
| vfh.* | see Section 3.1 | | RobotRadius 0.634 = r_circ (the body only, R-06) |
| teb.* | see Section 3.1 | | Dimension [1.0 0.78] = [L W] (the body only, R-06); MaxVelocity(1) 0.6 m/s is the bypass speed (`fsm.v_bypass`), which skips the governor (R-41) |
| **Hybrid A*** | | | |
| hastar.R_min / mpl / n_prim | 1.5 / 1.2 / 5 | m, m, - | F7 |
| hastar.R_min_local / mpl_local | 1.0 / 1.0 | m | LOCAL plans (rejoin, bypass) inside the lane mask |
| hastar.cost_rev / cost_switch / aei / interp | 5 / 10 / 3 / 0.25 | -, -, -, m | discourage reversing with an implement |
| hastar.r_infl | 0.694 | m | r_circ + 0.06; `hastar.r_infl_min` = W/2 + 0.10 = 0.49 m when the start or goal lies inside it (walls) |
| hastar.max_nodes / t_budget | 20000 / 1.5 | -, s | |
| hastar.rejoin_off / retries | 4.0 / 3 | m, - | |
| hastar.slope_turn_max / slope_head_tol / slope_base | 10 / 10 / 1.0 | deg, deg, m | D53 in replans: no turning primitive where the DEM slope over the footprint length exceeds 10 deg; there only headings within 10 deg of the fall line |
| hastar.win_margin / vdist | 10 / 0.1 | m | the GLOBAL window margin round the start and goal; the validator's ValidationDistance |
| map.local_res / local_N | 10 / 300 | cells/m, cells | 30 x 30 m robot-centred |
| map.static_res / static_N | 4 / 400 | cells/m, cells | 100 x 100 m window |
| **Mission manager** | | | |
| fsm.fix_min | 4 | - | not read as built (review GNC-13); the quality term uses `fsm.q_good` |
| fsm.sigma_auto | 0.05 | m | not read as built (review GNC-13) |
| fsm.q_good | 4 | - | T7's quality term: NavQuality >= 4 (RTK_FIXED or LIDAR_LOCALIZED) is not degraded. D69 kept it; in the open it pauses the mission 2 s after the RTK fix is lost (STATUS 4, REQ-SYS-021) |
| fsm.sigma_pause / sigma_degr | 0.15 / 0.10 | m | T7 cross-track sigma (4.1): degraded at >= sigma_pause, clear again at <= sigma_degr (Relay in InputConditioning). In the S4 rows the cross-aisle sigma is at most 0.061 m (the full S4 run of 2026-11-14, `work/agentic/s4fix/report.md`): about 2.5x margin; nav round 2 gave 0.050 m median and 0.058 m max (NEES 0.55) |
| fsm.sigma_along / sigma_along_off | 0.25 / 0.20 | m | T7 along-track sigma: degraded at >= 0.25 m, clear again at <= 0.20 m. 2.7x the S4 along-aisle maximum (0.093 m, p99 0.091, NEES 1.66), the cross-track ratio; 0.05 m below sigma_lost, so a lost along-track position pauses (P2 can resume) before nav_ok drops (NAV_INVALID, FAULT). At the row ends the headland turn runs across the aisle, where the along-aisle sigma is the cross-track one (0.15 m) |
| fsm.sigma_lost | 0.30 | m | nav_ok = navValid AND max(posStd(1:2)) < sigma_lost (InputConditioning). It sits below D89's 2 m flag, so a sigma of 0.30 m ends the mission in FAULT (REQ-SYS-023; the value is for card G2-GNC) |
| fsm.t_nav_lost / t_nav_degr / t_nav_rec | 0.3 / 2.0 / 5.0 | s | |
| fsm.t_mcu_timeout | 0.2 | s | 4 MCU status periods at 0.05 s |
| fsm.t_scan_lost | = P.sensors.per.protNStale × P.sensors.lidar.Ts (0.2 s) | s | derived from sensors; a lost scan is invalid for more than protNStale scans (SENSOR_LOST, 2.6 and 4.1) |
| tb.scn_scan | [0 1; 1e5 1] | [t valid] | bench scan table; an invalid scan is blank (ranges NaN, protectiveStop false), as a LiDAR loss was published before the fix |
| fsm.t_drive_lost | 0.5 | s | T36 (4.1): `drive_ready` false this long pauses; at least 3x the ESC unpark seen by the GNC (ENABLING 0.09 s, 0.1-0.15 s at the 50 ms MCU status). The MCU stops the robot itself, so the delay costs no safety margin |
| fsm.xte_fault / t_xte | 0.5 / 3.0 | m, s | the cross-track fault of G1 (the path bound of REQ-SYS-026), on TRANSIT, ROW, TURN, RTH_TRANSIT and DOCK_APPROACH only |
| fsm.t_stuck / v_stuck_cmd / v_stuck_meas | 5.0 / 0.3 / 0.05 | s, m/s, m/s | the stuck fault of G1 (REQ-SYS-113); it misses commands below 0.3 m/s, the 0.2 m/s dock approach and pivots included (R-10) |
| fsm.pitch_fault / roll_fault | 25 / 18 | deg | |
| fsm.geofence_margin | 0.5 | m | the boundary is grown by this offline (5.3); REQ-SYS-025 and FSE-13 ask for the stop inside the boundary, and REQ-SYS-122 for at least 0.30 m inside it in a greenhouse and 0.50 m in the open (decided, pending on card G2-GNC, with the kind of site taken from the plan's mission type) |
| fsm.soc_start_min / soc_reserve / soc_critical | 0.35 / 0.20 / 0.10 | - | LiFePO4 longevity, heat margin at 45-50 C; soc_reserve 0.20 is D101's reserve; soc_critical is not read as built (the BMS flag bms_critical acts) |
| fsm.k_tort / e_m / k_margin | 1.4 / 0.12 / 1.3 | -, Wh/m, - | 2.8; e_m has no recorded source and disagrees with the row test (R-19, review GNC-14) |
| fsm.E_nom_Wh | = `P.power.pack.E_Wh` (2948.4) | Wh | the pack as built; the power domain owns it |
| (removed) fsm.rc_required_in_auto | - | - | revision B (D44): owned by the MCU, `P.electronics.safety.rc_required_in_auto` = true; the FSM sets HoldReason RC_LOSS from `Bus_McuStatus.rc_link_ok` |
| fsm.auto_resume_nav | true | - | P2 resumes by itself after a NAV pause; whether it may is for the safety concept (G2-FSC; REQ-SYS-022) |
| fsm.resume_xte / resume_epsi | 0.3 / 30 | m, deg | |
| fsm.t_impl_ready | 10 | s | the ready time (REQ-SYS-102) |
| fsm.t_deck / fsm.deck_tol | 6.35 / 0.01 | s, m | the deck counts as down after 6.35 s of AUTO or RTH since power-up or MANUAL (card G2-DSC, D81); `deck_tol` 1 cm [EST]; the deck height is an estimate until a real report exists (STATUS 37) |
| fsm.t_docked / dock_tol_xy / dock_tol_yaw | 2.0 / 0.03 / 2 | s, m, deg | R5: DOCKED to IDLE after 2 s even without the charging flag (no docked check, R-10); the R4 trim below 2 deg; T12's tolerances |
| fsm.t_geofence / t_vfh | 0.2 / 0.2 | s | the geofence-breach and VFH-blocked persistence |
| fsm.v_bypass / teb_t_bad / teb_lfi_min | 0.6 / 3.0 / 3 | m/s, s, - | the bypass speed (`teb.MaxVelocity(1)`); TEB infeasible for 15 calls (3 s), or LastFeasibleIdx below 3, ends a bypass (S4) |
| fsm.replan_end_tol / nav_mode_gh | 0.10 / 2 | m, - | the end of a replanned path (R3, S3, S5); Bus_NavCmd.navMode for GREENHOUSE_COVERAGE (LIDAR_PRIOR_MAP) |
| **Implements** | | | |
| impl.mower_w / mower_h / mower_speed | 1.00 / 0.06 / 293.2 rad/s (2800 rpm) | m, m, rad/s | `P.structure.impl.mower.w`, the cutting height, `mower.w_nom` |
| impl.till_w / till_depth / till_speed | 0.60 / 0.08 / 26.18 rad/s (250 rpm) | m, m, rad/s | `P.structure.impl.tiller.w`, `d_nom`, `w_nom`; no tiller use until real tiller data exist (STATUS 36) |
| impl.till_lift_lead / lower_lag | 0.5 / 0.3 | m | |
| impl.boom_w / n_sect / R_app / v_min | 1.2 / 8 / 300 / 0.3 | m, -, L/ha, m/s | vision sections give up to 60% savings |
| **Coverage presets** | | | |
| cov.gh_swath / gh_sidelap / gh_R / gh_skip | 1.00 / 0.10 / 0.81 / 1 | m, -, m, - | the mower's swath (`P.structure.impl.mower.w`), lane spacing 0.90 m, skip-row, no bypass (greenhouse_std.json; the S4 house is 9.6 x 37.5 m) |
| cov.gh_post_edge / gh_c_clear / gh_c_xte | 0.20 / 0.15 / 0.05 | m | D54's inset terms (2.7): c_in 0.40 m today. D106: gh_c_xte 0.15 m, the 0.15 m maximum of D100, so c_in 0.50 m (decided, pending on card G2-GNC, with the mission files' inset block; REQ-SYS-027, -087) |
| cov.wall_clear / clear_min / clear_keepout | 0.20 / 0.15 / 0.30 | m | the footprint clearance kept in headland turns (0.15 + 0.05 m of cross-track error; S4's turns reach 0.20 m, 2026-11-14). REQ-SYS-088 asks for 0.5 m + 0.8 m per m/s of the turn speed + 0.10 m guidance + 0.10 m navigation (1.1 m at 0.5 m/s; the lead's ruling 3), so that the protective field stays off the walls; the turn speed in the headland or the field's shape in the turns, chosen so that the S4 house fits, is card G2-GNC's design (9, question 12); the minimum footprint clearance the compiler accepts; the body-edge clearance to JSON keep-outs |
| cov.slope_turn_max / slope_step | 10 / 1.0 | deg, m | D53: no headland turn where the slope exceeds 10 deg; the lane is shortened in 1 m steps |
| cov.dock_approach / terminal_len | 3.0 / 2.0 | m | the straight dock approach; the terminal extension (window B only) |
| cov.mower_x / mower_len | 0.90 / 0.60 | m | the deck centre ahead of the reference point and the deck length (`P.structure.impl.mower.pos`, `dims`): the deck front is 1.20 m ahead |
| cov.raster / via_tol / entry_Rs / entry_dstep / entry_dmax | 0.05 / 0.02 / 10.0 / 0.05 / 0.5 | m | the coverage raster; the coincident-waypoint tolerance; the lane lead-in S-curve when a forward entry would bring the footprint within clear_min of a wall |
| cov.palm_spacing / palm_R / palm_halfwidth | 8.0 / 2.0 / 2.5 | m | date-palm 8-10 m grid; skip 0, bypass on; palm_demo is 6 lanes x 104 m |
| cov.solar_pitch / solar_R | 7.0 / 1.5 | m | solar_demo: 6 aisles, piles r 0.15 m, bypass within the aisle |
| **Metrics** | | | |
| met.rms_window | 200 | samples | 10 s |
| **Variant control (base workspace, see risks)** | | | |
| VC_ROW_TRACKER | 1 (MPC) / 2 (STANLEY) | - | set from `P.gnc.variant.row_tracker` (`params/shaddad_params.m`) |
| VC_TEB (`P.gnc.variant.teb`) | 1 (TEB) today; D99: 0 (VFH) | - | decided, pending on card G2-GNC |

The MPC object `mpcRowObj` is built by `scripts/gnc_designMPC.m` from `P.gnc.mpc` and placed in the base workspace (see Risk R8).

---

## 7. Internal structure and files

```
models/Shaddad_GNC.slx                         Model Reference (ert, FixedStepDiscrete, base 0.05 s)
 |- In:  nav_state, perception, mcu_status, bms_status, mcmd, spray  (bus-typed, 5.1; no plan port, D57)
 |- InputConditioning      subsystems/gnc/gnc_InputConditioning.slx  (SubRef)  Rate Transitions -> 0.05 s;
 |                          nav_ok / nav_degraded / ages; odometry consistency |v_nav - mean(v_track)| < 0.3 m/s;
 |                          T7 sigma across / along the lane (pe one step late, Unit Delay pe_z; 2 Fcn, 2 Relays);
 |                          MissionCmd + rc_aux_cmd merge with Detect Change on cmd_seq
 |- MissionManager         subsystems/gnc/gnc_MissionManager.slx     (SubRef)  MATLAB Fcn gnc_missionFSM,
 |                          Unit Delays on PathError / Replan / TEB info, soc_rth block math
 |- PathServer             subsystems/gnc/gnc_PathServer.slx         (SubRef)  Selector (mission rows by seg_idx,
 |   |                      nxt_idx), Multiport Switch (MISSION/REPLAN), Detect Change -> window_ver
 |   '- SegmentPrep         Triggered Subsystem: 2 x Path Smoother Spline, Velocity Profiler (current seg),
 |                          deg<->rad Gains, holds Bus_PathWindow
 |- PathAnalyzer           subsystems/gnc/gnc_PathAnalyzer.slx       (SubRef)  MATLAB Fcn gnc_pathAnalyzer
 |- LocalAvoidance         subsystems/gnc/gnc_LocalAvoidance.slx     (SubRef)
 |   |- Corridor (0.1 s)    vector Trig / Product / Relational / Switch / MinMax  -> d_obs
 |   |- VFH (0.1 s)         Vector Field Histogram (TargetDir from Pure Pursuit, via Rate Transition)
 |   '- Bypass (0.2 s)      Enabled Subsystem: Timed Elastic Band (TEB) + Selector (VelCmds row 1)
 |- SpeedGovernor          subsystems/gnc/gnc_SpeedGovernor.slx      (SubRef)  1-D LUTs, MinMax(11), Rate Limiter Dynamic
 |                          MATLAB Fcn DescentCap (gnc_descentCap, gnc_descentLimit; card G2-DSC)
 |- PathTracker            subsystems/gnc/gnc_PathTracker.slx        (SubRef)
 |   |- PurePursuit         Pure Pursuit + LUT(Ld) + Product(omega = v*kappa)
 |   |- RowTracker          Variant Subsystem  [VC_ROW_TRACKER==1: ROW_MPC  (MATLAB Fcn gnc_mpcPlantUpdate
 |   |                                          + Adaptive MPC Controller, ext.mv = z^-1 omega_applied)]
 |   |                                         [VC_ROW_TRACKER==2: ROW_STANLEY (Lateral Controller Stanley
 |   |                                          + deg->rad + tan + Product + Gain 1/Lv)]
 |   |- Align               Gain + Saturation
 |   '- TrackerSelect       Multiport Switch on TrackerSel (STOP/PP/ROW/TEB/ALIGN)
 |- GlobalReplanner        subsystems/gnc/gnc_GlobalReplanner.slx    (SubRef)  Triggered Subsystem (replan_req edge),
 |                          MATLAB Fcn gnc_globalReplanner, output held (Bus_ReplanResult); 1 s task on target
 |- ImplementManager       subsystems/gnc/gnc_ImplementManager.slx   (SubRef)  Logical / Switch / Product / Enum Const
 |- CommandShaping         subsystems/gnc/gnc_CommandShaping.slx     (SubRef)  omega_max(v) Saturation Dynamic,
 |                          omega rate limit, track-speed scaling, deadband, heartbeat counter, Bus_GncCmd
 |                          MATLAB Fcn DescentEnv (gnc_descentEnvelope; card G2-DSC)
 |- StatusMetrics          subsystems/gnc/gnc_StatusMetrics.slx      (SubRef)  Enabled(ROW) Moving RMS, progress,
 |                          Rate Transition -> 0.2 s, Bus_GncStatus; Bus_GncDebug
 '- Out: GncCmd (0.05 s), GncStatus (0.2 s), GncDebug (0.05 s)

subsystems/gnc/gnc_missionFSM.m, gnc_pathAnalyzer.m, gnc_mpcPlantUpdate.m, gnc_globalReplanner.m  (%#codegen)
subsystems/gnc/gnc_descentCap.m, gnc_descentLimit.m, gnc_descentEnvelope.m  (card G2-DSC; plain functions, called by the DescentCap and DescentEnv blocks)
models/Shaddad_Companion.slx      the deployable companion top (D38): Companion_IO (HW: ros2lib Subscribe /can2/rx and
                                  Publish /can2/tx, a SocketCAN-FD bridge) + Shaddad_NavFusion + Shaddad_Perception + Shaddad_GNC
scripts/gnc_compileMission.m      JSON -> P.gnc.mission + static map   (offline, MATLAB)
scripts/gnc_designMPC.m           builds mpcRowObj from P.gnc.mpc
scripts/missions/greenhouse_std.json, palm_demo.json, solar_demo.json
params/param_gnc.m                parameter struct (Section 6)
buses/buses_gnc.m                   Bus_GncCmd, Bus_GncStatus, Bus_GncDebug, Bus_MissionPlan, Bus_MissionCmd,
                                    Bus_PathWindow, Bus_PathError, Bus_FsmOut, Bus_ReplanResult, Bus_MpcModel,
                                    Bus_NavCond, Bus_ObsFlags, Bus_TebMapCrop; the enums of 5.5 in buses/<Enum>.m
tests/gnc/tb_gnc.slx              tracking bench (Section 8; tests/gnc/t_gnc_tracking.m, matlab.unittest)
tests/gnc/tb_gnc_mission.slx      mission bench (Section 8; tests/gnc/t_gnc_mission.m, matlab.unittest)
```

**Model configuration (Shaddad_GNC.slx):** Solver FixedStepDiscrete, FixedStep 0.05. `EnableMultiTasking='on'` for target builds (the replanner and TEB get their own tasks). SystemTargetFile `ert.tlc`, TargetLang C++ (ROS 2 node). ProdHWDeviceType `ARM Compatible->ARM 64-bit (LP64)` (assumes a Linux companion computer, see open questions). `MATLABDynamicMemAlloc='on'` with a 64 kB threshold (needed by the planner objects). `ModelReferenceNumInstancesAllowed='Single'`. Root I/O is bus-typed, with "Default parameter behavior" set to Inlined except P.gnc tunables.

**Sample-time map:** 0.05 s for FSM, PathServer, PathAnalyzer, SpeedGovernor, PathTracker, ImplementManager and CommandShaping. 0.1 s for Corridor and VFH. 0.2 s for TEB, Status and the TEB map crop (Bus_TebMapCrop, from Bus_Perception.map). 1 s for replanner polling. Inherited (triggered) for SegmentPrep and Replanner. Inf for MissionPlan. Rate Transitions sit at every boundary: inputs in InputConditioning; 0.1 -> 0.05 for d_obs/VFH; 0.2 -> 0.05 for the TEB command; 0.05 -> 0.2 for status.

**Algebraic-loop prevention:** (1) FSM <- PathAnalyzer feedback through a Unit Delay. (2) MPC `ext.mv` = Unit Delay of the shaped omega_cmd. (3) Pure Pursuit TargetDir -> VFH crosses a rate transition, so it is delayed. (4) Replanner and TEB results reach the FSM through Unit Delays.

**Simulation speed estimate:** about 0.3-1 ms of GNC per 50 ms step in Accelerator mode (MPC QP: 8 decision variables plus slack, about 0.1-0.3 ms compiled; interpreted mpcmove measured 2.1 ms). A 4 h mission is 288,000 steps, so roughly 1.5-5 min of GNC wall time; the Simscape plant dominates. Per-event costs: SegmentPrep about 0.2-0.3 s interpreted (5-20 ms compiled) about 500 times per mission; Hybrid A* 0.2-1.5 s per replan (rare); TEB 20-100 ms per call, only in BYPASS. Set `SimulateUsing='Code generation'` on all System blocks.

---

## 8. Verification (the benches `tests/gnc/tb_gnc.slx` and `tests/gnc/tb_gnc_mission.slx`)

**Harness:** a Model block running Shaddad_GNC (Accelerator). Plant surrogate: Transfer Fcn First Order lags on v (tau 0.35 s) and r (tau 0.30 s, deliberately different from the model's 0.25 s), feeding a Differential Drive Kinematic Model ('Vehicle Speed & Heading Angular Velocity'). Lateral drift d is injected by Discrete-Time Integrators of d * [-sin psi, cos psi] added to x, y. Sensor surrogate: Random Number blocks for RTK noise (sigma 0.015 m, yaw 0.3 deg) and gnss_fix; harness LiDAR (4.5). MCU/BMS surrogate: Test Sequence (RC switch, E-stop, heartbeat, implement_ready, SOC ramp). Pass/fail is checked by Test Assessment blocks and matlab.unittest. Full-plant integration repeats T1-T4, T6, T8 and T12 in Shaddad_System with relaxed limits (in brackets). The tracking bench (tb_gnc, t_gnc_tracking) never starts its mission (R-30): T1 to T5 do not run, and T1, T3 Pure Pursuit and the envelope and tilt checks pass with the robot parked (card T-03).

| ID | Scenario | Pass criteria |
|---|---|---|
| T1 | Straight 60 m row, v_work 0.8 m/s, RTK noise, no drift | XTE RMS (Moving RMS, settled) < 2 cm [full plant: < 5 cm, the requirement]; max abs(XTE) < 6 cm; abs(mean XTE) < 1 cm; abs(e_psi) RMS < 1.5 deg |
| T2 | Row entry with 0.3 m lateral offset, v 0.8 | abs(XTE) < 5 cm within 4 s [5 s]; overshoot < 5 cm; no omega saturation longer than 0.5 s |
| T3 | Side-slope drift d = 0.03 m/s on a 60 m row (all three trackers) | MPC: abs(mean XTE) < 1 cm, RMS < 2 cm. Stanley: abs(mean) < 4.5 cm. PP: reported only (expected about L_d d / v = 5.6 cm), which documents why MPC is the default |
| T4 | Greenhouse headland turn (skip-row, R 0.81 m) | max abs(XTE) in the turn < 15 cm (0.10 m decided by REQ-SYS-127, pending on card G2-GNC); abs(XTE) < 5 cm within 2 m after row entry; peak abs(omega) <= 1.2 rad/s; abs(v) + abs(omega) chi B/2 <= `cmd.v_track_max` 1.414 m/s at all times (chi B/2 = 0.418); v at turn apex <= 0.5 m/s |
| T5 | Speed step 0 -> 1.0 m/s (transit, straight) | v_cmd reaches 1.0 m/s in 2.0 +/- 0.15 s (a = 0.5 m/s^2); v_cmd overshoot 0; abs(v_meas - v_cmd) < 0.05 m/s after 3 s; deceleration to 0 within 1.0 m/s^2 |
| T6 | Static 0.5 m box on the row 10 m ahead, removed after 5 s | OBSTACLE_HOLD within 0.1 s of d_obs < 1.5 m; stop with front-to-box distance >= 1.2 m; spray off within 0.05 s; mower off at 3 +/- 0.05 s; resume 2.0 +/- 0.1 s after removal; minimum clearance > 1.0 m |
| T7 | Persistent 1 m obstacle in a palm lane (bypass allowed) | BYPASS starts at 10 +/- 0.1 s; footprint clearance >= 0.3 m; lateral deviation <= 2.5 m (lane half-width); abs(XTE) < 10 cm within 5 m past the obstacle; bypass complete < 40 s |
| T8 | Greenhouse full coverage, greenhouse_std (9.6 x 37.5 m, 10 lanes, mower) | completion within 25 min and within 15 % of the compiler's estimate; coverage >= 98 % of the inset floor (0.05 m raster of swath footprints); overlap of the planned rows <= 15 %; row XTE RMS < 3 cm; no geofence violation; no obstacle hold; navMode 2 (2026-11-16: 13.2 min, 98.8 %, 13.5 %, 0.0068 m) |
| T9 | Palm patrol, palm_demo (6 lanes x 104 m) at 1.0 m/s | completion within +/-5 % of the compiler's estimate; row XTE RMS < 5 cm; commanded turn speed, and true speed over the middle half of each turn, <= 0.8 m/s (2026-11-16: +2.5 %, 0.0075 m, 0.764 m/s; the true entry peak, 0.821 m/s, is reported, not judged) |
| T10 | Solar row mowing, solar_demo (6 aisles, piles r 0.15 m) | minimum body distance to the piles >= 0.30 m; no obstacle hold; every ROW driven (2026-11-16: 1.561 m, 0, 6 of 6) |
| T11 | Mode logic (Test Sequence) | every transition in 4.1 fired at least once (logged transition matrix 7x7 plus sub-modes); RC MANUAL during AUTO -> MANUAL within 1 step and cmd_valid false; RC back to AUTO -> PAUSE; RESUME -> AUTO with REJOIN when abs(e_y) > 0.3 m; ABORT -> IDLE; E-stop -> FAULT within 1 step; RESET_FAULT is refused while rc_sw == AUTO. S4 fixes (2026-11-14): `drive_ready` drops of 0.1 and 0.3 s do not pause, a drop of t_drive_lost pauses (OPERATOR) at t_drive_lost +/- 0.025 s in AUTO, OBSTACLE_HOLD and RTH, and at once with the RC link lost (testT36DriveReadyDebounce, scripted and bench); a resume on the lane 175 deg off goes to ALIGN without a replan and then on along the lane, REJOIN off the lane, tilted or on a TURN (testResumeAlignHeading); a failed LOCAL rejoin is followed by GLOBAL to the same goal, PLANNER_FAIL after 3 LOCAL + GLOBAL pairs, a bypass rejoin and a bypass resumed after a pause (also after a second pause) stay LOCAL; on greenhouse lane 5 at the S4 pose LOCAL fails and GLOBAL succeeds (testRejoinGlobalFallback) |
| T12 | SOC ramp 0.40 -> 0.18 during a mission | RTH triggered at SOC = soc_rth +/- 0.005; arrival at the dock with SOC >= 0.18; dock final abs(XTE) < 3 cm and abs(e_psi) < 2 deg; DOCKED -> IDLE when charging is true; the SOC follows a scripted ramp, so T12 does not verify the reserve on arrival (REQ-SYS-057; S3, D101) |
| T13 | RTK degradation: fix -> float (sigma 0.15 m) for 10 s, then fixed. T7 sigma rule (S4 fix 2026-11-14): sigma profiles on InputConditioning alone, and RTK fixed with the sigma stepped in the mission bench | v capped <= 0.5 within 1 step; PAUSE (NAV) after 2.0 +/- 0.1 s; automatic resume 5.0 +/- 0.1 s after recovery. Sigma rule, on InputConditioning alone (testT7NavSigmaRule; covariance built from the sigmas along / across the lane, nav_degraded as expected at every 0.05 s step): the cross-track sigma switches on at the first step >= 0.15 m and off at the first <= 0.10 m, with no change while it stays in between or chatters by +/-3 mm at either threshold; on a lane 30 deg off East with the ellipse along it (0.24 / 0.05 m) nav_degraded stays false (max(posStd(1:2)) 0.21 m, without covPose(1,2) 0.152 m), and a cross-track ramp switches where it reaches 0.15 / 0.10 m; on a lane at 120 deg the along-track sigma switches on at 0.25 m and off at 0.20 m; without a valid path error the rule uses max(posStd(1:2)). In the mission bench (testT7NavSigmaPause): sigma 0.105 / 0.145 m does not pause, 0.16 m pauses after 2.0 +/- 0.1 s, the band does not resume, nominal sigma resumes after 5.0 +/- 0.1 s |
| T14 | nav_valid false for 0.4 s | FAULT NAV_INVALID at 0.30 +/- 0.05 s; v_cmd = 0 in the same step; cmd_valid false |
| T15 | MCU status frozen | FAULT MCU_LINK_LOST at 0.20 +/- 0.05 s |
| T16 | Geofence: pose pushed 0.6 m outside the grown geofence (1.1 m outside the site boundary) | FAULT GEOFENCE within 0.5 s (0.20 s, 2026-11-16) |
| T17 | Stuck: plant v forced to 0 with v_cmd 0.5 | FAULT STUCK at 5.0 +/- 0.1 s |
| T18 | Hybrid A* RTH on a 100 x 100 m static map with keep-outs | success; path clearance >= r_infl; t_plan < 1.5 s (MATLAB), < 0.5 s (SIL); path length <= 1.3 x the shortest grid path |
| T19 | Code generation | `slbuild('Shaddad_GNC')` with ert succeeds; zero algebraic loops; SIL execution time per 0.05 s step p99 < 5 ms (excluding replanner and TEB tasks); SIL vs normal-mode equivalence on T1 and T6 with abs(diff v, omega) < 1e-6 |
| T20 | Long run: 4 h palm patrol, kinematic plant | Accelerator run >= 50x real time; no NaN/Inf in Bus_GncCmd; seq heartbeat never stalls |
| T21 | `testDescentCapRule` (t_gnc_mission, card G2-DSC): every descent entry is read from `P.structure.descent` through the helper `descRef`; one column per breakpoint, the breakpoints rise, and no row rises with pitch; 432 sweep pairs over every breakpoint, each bin's middle, a point just steeper than each breakpoint, uphill and above the last breakpoint, rows 0 to 3, an unknown implement and the raised deck, in both directions | every value equals the table rule (step 10: 0 of 63 values differ) |
| T22 | `testDescentGovernorCap` (9 cases; the sprayer at 10, 15, 20, 20.25 and 21.2 deg among them) | the governor bench gives 0.95 / 0.60 / 0.15 / 0.10 / 0 m/s for the sprayer at 10 / 15 / 20 / 20.25 / 21.2 deg (the table less dv) |
| T23 | `testDescentLookAhead` (world 2, the 20 deg ramp, the sprayer at 1.0 m/s, nav pitch 0 so only the DEM counts; StopTime 30 s) | in AUTO throughout, the speed falls to each class's cap before the class (0.152 m/s at the 15 deg point against a cap of 0.15) and holds 0.150 m/s on the ramp, with no stop |
| T24 | `testTillerNoReverse` (the tiller fitted, a reverse command) | the speed command never goes below 0; no TEB bypass (H2): the robot holds, then resumes or pauses with OBST_TIMEOUT |
| T25 | `testMowerDeckDownWhenMoving` (card G2-DSC, D81): the raised-deck rows bound the cap while the robot moves; the robot waits for the deck to count as down at the start | no motion before the deck counts as down (IMPLEMENT_NOT_READY until 6.35 s); the raised rows are read from `P.structure.descent` |

---

## 9. Open questions and risks

### Open questions
1. **electronics/drive:** Does the STM32 close a **yaw-rate loop** (IMU gyro z) and a speed loop, giving the first-order tau_r ~ 0.25 s the MPC assumes? Open-loop (v, omega) mixing on soil gives an uncertain yaw gain (0.6-1.0 from track slip) and degrades row tracking.
2. **electronics (settled by D19 and D38):** the companion <-> STM32 link is CAN-FD on FDCAN2, with Bus_GncCmd in COMP_CMD 0x100 (5.2; ICD 5), through `Companion_IO`'s ros2lib Subscribe and Publish and a SocketCAN-FD bridge node (`ros2lib/CAN Transmit` and `CAN Receive` are classic CAN only). Nothing is left open.
3. Companion hardware (Jetson Orin, Raspberry Pi 5, x86 IPC?) determines the codegen device type, the feasibility of the TEB/HA* timing budget, and whether ROS 2 is the runtime.
4. **sensors (settled by D23 and ICD 3.1):** Bus_NavState comes at 0.01 s, in ENU with yaw CCW from East; Bus_Perception carries the TEB MapInfo fields in TEB's order and the scan about base_link (5.1). Nothing is left open.
5. **rc:** SBUS channel mapping: 3-position mode switch (MANUAL / HOLD / AUTO) and aux buttons (START/RESUME, PAUSE, RTH) forwarded as `rc_aux_cmd`. Required behaviour on RC loss in AUTO: continue (rc_required_in_auto = false) or stop?
6. Settled for this phase by D97: the robot has no dock and is charged by hand under an approved procedure; F-10 is verified in the twin, with the 3 cm / 2 deg tolerance assumed (`fsm.dock_tol_xy`, `dock_tol_yaw`).
7. Implement data: real working widths, spray section count, and whether the vision sprayer mask comes from `sensors` or from an implement ECU.
8. GNSS availability inside greenhouses. If RTK is often float, `sensors` must provide LiDAR-based localisation, otherwise T8 fails the XTE requirement.
9. Source of static maps (palm trunk survey, solar pile layout). Without them, solar piles become dynamic-obstacle false positives.
10. Mission upload mechanism (ground-station app vs ROS 2 topic) and whether onboard mission compilation is ever needed. The `uavCoveragePlanner` code-generation path has not been tested inside a MATLAB Function block.
11. **G2-GNC, CR-001** (STATUS 34): the mission planner has no slope limit per implement, so the full-tank sprayer is not kept off ground steeper than 18.5 deg in planning. D111 (2026-11-18) narrows the envelope further: on wet loam, missions only up to 18 deg (REQ-SYS-089, with REQ-SYS-129), and with the tiller, descents forward only (REQ-SYS-075, card G2-TIL). Needed: a slope limit per implement, owned by structure next to the table (proposed `P.structure.slope_max`, which P.gnc reads); a slope check in `gnc_compileMission` (the DEM sampled every 0.25 m along every leg, the steepest slope in any direction); the same limit at run time in `gnc_slopeValidator` for replans, rejoins, RTH and docking; no TEB bypass where the lane corridor crosses steep ground; and a test (a sprayer mission over the 20 deg ramp is rejected or routed around). The governor and the MCU cap stay as the backstop. The look-ahead's settings (t_resp, a_la, t_hold) are estimates to confirm in a HiFi AUTO descent with the sprayer.
12. **G2-GNC, REQ-SYS-088** (the lead systems engineer's rulings of 2026-11-18): the turn clearance is 0.5 m + 0.8 m per m/s of the turn speed + 0.10 m guidance + 0.10 m navigation, 1.1 m at 0.5 m/s, so that the protective field stays off the end walls in the lane-end turns, with a greenhouse turn's navigation error (REQ-SYS-020) inside it. How the S4 house fits is this card's design, with the perception engineer: a slower turn in the headland (0.9 m at 0.25 m/s) or a field shaped for the turns.
13. **G2-GNC, D108** (the project owner, 2026-11-18; decided, pending on cards G2-FSC and G2-GNC): for a person walking head-on, the person field ahead grows on open ground (palm groves, transits) to at least 3.4 m at 1.0 m/s and 4.0 m at 1.2 m/s (REQ-SYS-131, the upper ends of D108's ranges); in a greenhouse it stays as it is (D79). This card checks that the planned paths keep the longer field off the trunks and other fixed objects ahead, in the row-end turns and the transits above all, or lower the speed where they come close; otherwise each one holds the robot until the operator resumes (D84). Whether REQ-SYS-088's clearance rule applies outdoors with the longer field is part of this check. S5 gets a head-on case on card G2-INT.

### Risks
- **R1 Stateflow not installed:** the FSM is a MATLAB Function, so it is less visual and there is no formal decision coverage (Simulink Coverage not installed either). Mitigation: the transition-table layout, logged transition matrix (T11), and porting 1:1 to a chart if Stateflow is installed.
- **R2 Velocity Profiler behaviour (F4):** it gives one cruise speed per direction-segment. Multi-segment windows would slow whole rows to turn speed, so per-segment profiling is mandatory.
- **R3 Rate compatibility:** 50 Hz producers cannot feed the 20 Hz GNC deterministically. The system-wide rate set must be agreed (5.1).
- **R4 Mission plan size:** path table single [51200x4] = 0.8 MB on a constant-sample-time inport. This is fine for codegen (passed by reference) but adds model update / Accelerator build time. Fallback: S_max 256 with mission legs.
- **R5 TEB:** variable-size outputs up to 10000 rows need dynamic memory, and it is the heaviest block (20-100 ms). It is restricted to BYPASS at 5 Hz. If it overruns on target, fall back to HA* rejoin only.
- **R6 Hybrid A* runtime:** 0.2-1.5 s. It must run in its own task (multitasking with Rate Transitions). In single-tasking SIL it will overrun the 50 ms step; in simulation it only costs wall time.
- **R7 Algebraic loops:** FSM <-> path window <-> analyzer, and MPC ext.mv. Broken with explicit Unit Delays (7). Building without them will raise algebraic-loop errors in the Model Reference.
- **R8 Convention conflicts:** (a) the Adaptive MPC mask `mpcobj` needs an MPC object variable name, so `mpcRowObj` lives in the base workspace rather than inside P (a struct field such as `P.gnc.mpcObj` may not be accepted). (b) Variant control `VC_ROW_TRACKER` is a base variable, because variant expressions referencing struct fields are not assumed to work. (c) MATLAB Function parameter `P` must contain only numeric/logical fields; strings and objects in P break codegen.
- **R9 Degrees vs radians:** Path Smoother Spline and Stanley use degrees. One missed Gain gives about 57x heading errors. Assertions check abs(e_psi) <= pi.
- **R10 Tall vegetation:** LiDAR returns from grass to be mowed can cause permanent OBSTACLE_HOLD. `sensors` must filter by height or classify. GNC exposes the corridor width and d_stop as tunables.
- **R11 Terrain dependence:** skid-steer yaw response (tau_r, gain) varies about 2x between sand, wet soil and concrete greenhouse floors. The MPC disturbance estimator absorbs bias but not gain errors. Plan gain-scheduled tau_r or online identification, and re-run T1-T4 at +/-50% tau_r.
- **R12 Coverage planner limits:** uavCoveragePlanner is UAV-oriented (ignores turn feasibility and the chassis-vs-tool width difference). The compiler adds the boundary inset, skip-row ordering and Dubins turns. With today's mower wider than the body (1.00 m against 0.78 m), the strip within c_in of the wall line is not mowed: 0.40 m today (0.20 m beyond the post envelope), 0.50 m with D106 (0.30 m beyond it; decided, pending on card G2-GNC).
- **R13 Variant activation time:** 'update diagram' generates code for only the active row tracker. Use 'code compile' or 'startup' if the deployed binary must switch trackers at run time.
- **R14 Greenhouse GNSS (see Q8)** directly threatens the row tolerance of REQ-SYS-027 (RMS 0.05 m and maximum 0.15 m over the whole row, D100).
