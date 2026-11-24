# Shaddad UGV - SENSORS & SENSOR FUSION Specification (KEY = sensors)

> **Revision B note (2026-10-19).** `docs/architecture.md` section 2b (D30-D63) and `docs/build_plan.md`
> override this spec where they differ. Sections changed for this domain: 3.3 odometry numbers (N_g 24, r_s 0.128 m -> 122 230 counts/m, from P); IMU temperature coefficients = ICM-20948 datasheet (D49); EKF slip prior s0 0.005 and slip hold during GNSS + scan-match denial (D49); EKF `EnableMultirate off`, corrections at 0.01 s gated by enable ports (D63); chi0 from `P.structure.track.chi_nom` (D39); F1 `sens_env_skyview` replaced by 2-D Lookup Tables on precomputed `P.world.gnssSvf` / `gnssZoneId` (D59); F7 scan filter and the F9 sector minima / protective field built from blocks (D59); F11 scan matching with Resolution 10, MaxRange 10 m, window [0.3 0.3] (measured 29 ms per call, D55); `sens_acq_imu` / `sens_odom_compute` built in T11 (D62).

> **S4 nav fix round 2 (2026-11-14).** Sections changed: 3.2 GNSS velocity noise (zero mean per axis, added after
> the GPS blocks; P2 `VelocityAccuracy` 0), 3.5 channel 2 R without RTK (`navgate.kRvel`) and channel 5 start guard
> (`navgate.smWeakRatio`, `smCap`), 3.7 / F11 along-aisle covariance of the scan match from the scan geometry
> (`per.smGeoGap`, `smGeoDmax`), 7.3 / 7.6 / 7.8 the new parameters, 9 T-NAV-12 and T-PER-06.

> **Critfix (2026-11-15).** Sections changed: 3.7 (protective field from the fitted implement, lost or stale scan, body-only self box), 7.8 (protective-field and self-box parameters), F6 (IMU-loss timers, transient NIS fault), 9 (T-PER-02 extended by tPER02b, T-PER-07 and T-NAV-13 added).

> **G1-01 part 2 (2026-11-18).** Brought in line with the parameters (`P` of 2026-11-17) and the decisions in force (D1 to D110; D104 to D107 and D110 change nothing in this spec, and D108 and D109 are in 3.7 and 7.8) by the sensors and navigation engineer. Sections changed: 0, 1, 3.3, 4.1, 4.2, 6 and 8 (no encoder model: the ESC model gives the counts in DRV_FB3, D3; odometry every 0.01 s with the EKF update every 0.05 s, D13; the STM32 NavLite was not built, the MCU's tilt is D47's; the files and benches as built); 2 and 7.1 (the datum, the track gauge and the IMU and LiDAR mounts from `P`); 3.4 and 7.5 (the LiDAR mount, four moving obstacles); 3.5, 4.3.1, 7.6 and 10.2 (`EnableMultirate` off, D63; `x0` and `qc` of D49); 3.6, F6, 7.6 and 9 (D89: `navValid` is a 2 m health flag, and the GNC's gate of D69 judges the accuracy); 3.7, F11 and 7.8 (the scan matcher of D55, the row window, the parameter names of `P`, the near zone of D84); 7.2 (the temperature coefficients of D49); 7.3 (the parameter names of `P`); 7.9 (the sensors see the worlds of `P.world`); 9 (T-SEN-03, T-SEN-04, T-NAV-14); 10.1 (answered questions, a dated standard, D103). The sensor hardware classes (ICM-20948, ZED-F9P, OS1-32) stay design assumptions until the as-built data arrive (card G1-04).

> **G1-01 part 2, step 19 (2026-11-18).** The fix pass on the independent re-check: the solar table's 0.8 m edge (D10); FULL3D's 31 x 360 cloud; the IMU Filter's initial process noise from `P`; F6's RUN_LIDAR condition; `Bus_NavCmd` at 0.05 s; the built element names of `Bus_ImuMeas`, `Bus_OdomMeas` and `Bus_NavState`; T-NAV-04's 104 m row, T-NAV-07's 240 s run, T-NAV-11's 3-D RMS; the Hall-only figures; the pending marks of D84 and D99; the test person of REQ-SYS-033 with the residual risks of D109 (3.7); the longer field ahead on open ground of D108 (3.7, 7.8, pending); REQ-SYS-023's dead-reckoning limit beside the parameter (7.6); T-PER-01's 0.30 m case and T-PER-02's test persons (planned on card G1-02).

Version 0.1 (design phase) - 2026-10-18 - MATLAB R2026b
Status: build-ready proposal. Every library path marked "verified" was opened in the live R2026b session
(`load_system` + `get_param(...,'DialogParameters')`) and the mask parameter names below are the real names.

---

## 0. Executive summary

* Plant-side **sensor models** (Subsystem Reference, `subsystems/sensors/Shaddad_SensorModels.slx`) turn the true vehicle
  state (STRUCTURE) and motor shaft angles (DRIVE) into realistic raw sensor data:
  * 9-axis IMU (ICM-20948 class, 200 Hz) with the existing **`mspsensorlib/IMU`** block (noise, bias instability,
    random walk, temperature drift, quantization, range), plus lever arm, mounting misalignment, track vibration and DLPF.
  * Dual-antenna RTK GNSS (u-blox ZED-F9P moving-base class, 10 Hz) with three existing **`mspsensorlib/GPS`** blocks
    (RTK-FIXED / RTK-FLOAT / SINGLE noise sets), a fix-state Markov chain driven by a sky-view factor computed from the
    environment (palm canopy, greenhouse, solar panels), 80 ms latency, outliers and dropouts.
  * Track odometry: no encoder model here. The ESC model (DRIVE) gives the motor encoder counts (4096 counts/rev)
    in DRV_FB3 (D3), and the MCU turns them into track speeds (`sens_odom_compute`, every 0.01 s, D13).
  * 3D LiDAR (Ouster OS1-32 class, 10 Hz), two fidelities in a Variant Subsystem:
    **FAST2D** (default, mission-length; Navigation Toolbox `rangeSensor` ray casting on a rasterised obstacle map) and
    **FULL3D** (existing **`uavsimlib/UAV Scenario Lidar`** block on a `uavScenario` built from the same geometry).
* Companion-computer **navigation fusion** (Model Reference `models/Shaddad_NavFusion.slx`, 100 Hz): existing
  **`mspfiltlib/IMU Filter`** for roll/pitch + existing **`cstblocks_estimation/Extended Kalman Filter`** (one rate,
  0.01 s, `EnableMultirate` off, D63; 5 measurement channels gated by enable ports) with an 11-state skid-steer model that estimates **pose, velocity, lateral skid, gyro/accel
  bias, left/right track slip and the effective-gauge (ICR) factor**. Measurements: GNSS position, GNSS velocity,
  dual-antenna heading, track odometry, LiDAR scan-to-map pose (greenhouse / GNSS-denied).
* Companion-computer **perception** (Model Reference `models/Shaddad_Perception.slx`, 10 Hz): scan filtering, rolling
  local occupancy grid in exactly the struct format consumed by the `navlib` **TEB** block, obstacle list, sector
  ranges / protective-field flag (also VFH-ready ranges+angles; VFH is the decided default bypass and TEB a development option, D99: the switch is pending on card G2-GNC, and today `P.gnc.variant.teb` = 1, so the twin still runs TEB), palm/solar row detection, and scan-to-map localization.
* STM32-side **acquisition** Variant Subsystems (SIM / HW) using the existing **`stm32IMUsensorslib/ICM20948 IMU Sensor`**
  block, and the odometry computation subsystem. The optional STM32 "NavLite" of revision A was not built: the
  MCU's own tilt estimate (`fw_imu_tilt`, D47) holds the tilt limits.
* **Decision: fusion runs on the companion computer** (double precision, LiDAR compute, NTRIP/RTK handling); STM32 only
  samples the IMU, computes the odometry from the ESC's counts, timestamps and forwards (IMU 200 Hz, odometry 100 Hz, D13).
* **Stateflow is NOT installed** (`license('test','Stateflow')` returns 1 but `ver('stateflow')` is empty), so the two
  state machines (GNSS fix chain, nav mode manager) are MATLAB Function blocks with persistent state. If Stateflow is
  later installed, they can be swapped 1:1 for `sflib/Chart` blocks (verified to exist in the library file).

---

## 1. Purpose & scope

### 1.1 In scope (owned by SENSORS)
| Item | Location |
|---|---|
| Plant sensor models: IMU, GNSS (incl. dual-antenna heading), track encoders, LiDAR (2D-fast / 3D) | `subsystems/sensors/*.slx` (Subsystem Reference) |
| Environment geometry seen by the sensors (the worlds of `P.world`, structure spec: palm rows, greenhouse walls and posts, solar tables, static and dynamic obstacles, GNSS sky-view zones) and the prior map with its as-built error | `P.sensors.env` (copied from `P.world`), `P.sensors.per.priorMapErr`; the FULL3D scene from `utilities/sens_env_to_uavscenario.m` |
| Sensor-acquisition Variant Subsystems (SIM/HW) for STM32 (IMU) and companion (GNSS, LiDAR) | `subsystems/sensors/sens_acq_*.slx` |
| Odometry computation (the ESC's DRV_FB3 counts -> track speeds) inside the MCU firmware (Task_10) | `subsystems/sensors/sens_odom_compute.slx` |
| NavLite on STM32 (tilt alarm, yaw-rate, fallback): not built; the MCU's `fw_imu_tilt` holds the tilt limits (D47) | none |
| Navigation fusion (attitude filter, EKF, gating, nav mode manager, outputs) | `models/Shaddad_NavFusion.slx` (Model Reference) |
| LiDAR perception (scan filter, local occupancy grid, obstacles, sectors, protective field, row detection, scan-to-map localization) | `models/Shaddad_Perception.slx` (Model Reference) |
| Parameters, buses (sensor/nav part), tests | `params/param_sensors.m`, `buses/shaddad_buses.m` (sensor section), `tests/sensors/` |

### 1.2 Out of scope (neighbouring domains)
| Domain | What they own | Interface with SENSORS |
|---|---|---|
| **STRUCTURE** | rigid-body / track-terrain dynamics, terrain, mounting points (CAD) | provides `Bus_TrueState` (must include slip truth); confirms base_link and sensor mount positions |
| **DRIVE** | traction motors, gearbox, motor control (FOC) | provides motor shaft angles/speeds/currents; owns gear ratio, sprocket radius, backlash |
| **POWER** | battery, DC bus | DC currents (alternative source for magnetometer disturbance); sensor power rails |
| **ELECTRONICS / MCU** | STM32 firmware model, companion computer, comms (CAN-FD/Ethernet), time sync, enclosure thermal, NTRIP modem | integrates `sens_acq_*` and `sens_odom_compute`; provides `Bus_ElecEnv` (enclosure temperature, correction-link status); power budget of sensors (Section 6.9) |
| **GNC** | path planning, path following, missions, TEB/VFH/Pure Pursuit, safety logic | consumes `Bus_NavState`, `Bus_Perception`; provides `Bus_NavCmd` (nav mode, reset, prior map id); may read `P.sensors.env.rowLines` for coverage planning |
| **RC** | SBUS decoding, Phase-1 manual drive | none (the NavLite of revision A was not built) |
| Implement / vision sprayer | camera + weed detection | NOT in this spec; see open question Q8 (abstract detection model proposed) |

---

## 2. Conventions (frames, signs, units)

| Item | Convention | Reason / note |
|---|---|---|
| World frame W | Local tangent plane **ENU** at site datum `P.sensors.refLLA` = `P.world.lla0` = [24.1556 47.3120 430] (deg, deg, m; Al-Kharj palm region, replace per site) | Navigation/Robotics toolbox 2D poses [x y theta] and ROS REP-103 |
| Body frame B (`base_link`) | **FLU** (x forward, y left, z up); origin at ground contact plane, mid-gauge, mid track contact length | Proposal to STRUCTURE - must be confirmed (Q1) |
| Euler angles | ZYX [yaw psi, pitch theta, roll phi], right-handed. psi measured CCW from East. **Positive pitch = nose DOWN** (rotation about +y left) | verified: `quaternion([30 10 5],'eulerd','ZYX','frame')` and `quat2eul` give identical angles, so the toolbox orientation quaternion = body-to-world orientation |
| GNSS heading | receivers report heading CW from North; psi_ENU = wrap(pi/2 - heading_N - psi_baseline) | |
| Accelerometer sign | **Physical specific force** on all buses (flat & level reads [0 0 +9.79]) - same as the real ICM-20948 | **Verified:** the Sensor Fusion Toolbox `imuSensor` / `IMU` block outputs the *negative* of specific force (ENU flat = [0 0 -9.81]; forward accel +1 gives -1). Therefore: Gain -1 after the IMU block (plant) and Gain -1 before IMU Filter / ecompass (companion). Test T-SEN-02 guards this. |
| Units | SI everywhere (magnetic field in uT because the IMU block uses uT) | |
| Data types | plant double; STM32 single (IMU, odom); companion double (nav), single (scan arrays), boolean (occupancy) | |

Site constants (computed in R2026b): g = **9.7876 m/s^2** (`gravitywgs84(430,24.15)`); WMM2025 field at 2026.8:
NED = [34.116 1.745 26.317] uT -> **ENU = [1.745 34.116 -26.317] uT**, declination +2.93 deg (E), dip 37.6 deg, F = 43.1 uT.

---

## 3. Physical / algorithmic models

### 3.1 IMU (ICM-20948 class, 200 Hz)

Kinematics at the IMU location r_I (body), with true state from STRUCTURE:

```
a_I,W   = a_W + R_WB ( alpha_B x r_I + omega_B x (omega_B x r_I) )        (lever arm, 3x3 Cross Product blocks)
omega_W = R_WB omega_B
q_WI    = q_WB (x) q_BI      (q_BI = true mounting misalignment, firmware assumes identity)
```
Vibration (track/ground excitation), added to the IMU-block inputs:
```
a_vib = |v_B,x| / 1 m/s * BP_10-60Hz( BLWN ) * sigma_avib     sigma_avib = [0.6 0.5 1.2] m/s^2 rms at 1 m/s
w_vib = |v_B,x| / 1 m/s * BP_10-60Hz( BLWN ) * sigma_wvib     sigma_wvib = [0.03 0.03 0.01] rad/s rms at 1 m/s
```
Sensor errors inside `mspsensorlib/IMU` (per axis): y = S*(1+ts*dT) * M * x + b0 + b_T*dT + b_BI(t) + b_RW(t) + n_white,
quantised to Resolution and saturated to MeasurementRange (dT = T - 25 C, T from ELECTRONICS enclosure model).
Then the chip DLPF (50 Hz) is modeled as `H(z) = 0.7921/(1 - 0.2079 z^-1)` (a = exp(-2*pi*50/200)).
Magnetometer disturbance from traction currents (body-fixed): `m_dist = k_I * (|I_L| + |I_R|)`, k_I = [0.04 0.02 0.05] uT/A
(~6-8 deg heading error at 120 A) -> magnetometer used only for stationary initialization.

### 3.2 GNSS (dual ZED-F9P moving base, 10 Hz)

Antenna position/velocity truth: `p_a = p_W + R_WB r_a`, `v_a = v_W + R_WB (omega_B x r_a)`.
Latency: truth delayed by tau = 0.08 s (Delay block, 8 samples at 0.01 s) before 10 Hz sampling.
Three `GPS` blocks run in parallel on the same delayed truth with different noise sets (Gauss-Markov error with
DecayFactor) and seeds; a Multiport Switch selects the active one by fix state.
Velocity (S4 nav fix round 2): `v_meas = v_a(t - tau) + n`, n zero-mean white per axis E N U with sigma
`sigVel(fix)` (NOFIX: SINGLE), seeds `gnss.seedVel`. The GPS blocks run with `VelocityAccuracy` 0 and model the
position only: `gpsSensor` (`fusion.internal.GPSSensorBase.stepImpl`, R2026b) adds the magnitude
`sqrt(e1^2 + e2^2)` of its 2-D velocity noise to the ground speed and a course noise of sigma
`VelocityAccuracy/speed` rad added in degrees, so its velocity reads `sqrt(pi/2) sigVel` = 1.2533 sigVel high along
the track (S4: +0.026 m/s FIXED, +0.058 m/s SINGLE measured) with almost no noise across it. That bias was part of what drove the EKF
slip to -0.03..-0.08 in the greenhouse (true +0.008); without it the slip still reads -0.025 / -0.019 in S4
(open, STATUS 16d). The nav bench had added its own noise since T12.

Fix-state chain (codes = NMEA GGA quality: 0 NOFIX, 1 SINGLE, 4 RTK_FIXED, 5 RTK_FLOAT), per step dt = 0.1 s,
transition probability p = 1 - exp(-lambda*dt), lambdas in 1/s:
```
FIXED -> FLOAT : 0.002 + 0.50 (1-SVF)^2           ; forced if corrAge > 10 s
FLOAT -> FIXED : 0.20 SVF^3                        ; only if corrections OK and corrAge < 10 s
FLOAT -> SINGLE: 0.30 (1-SVF)^3                    ; forced if corrAge > 30 s
SINGLE-> FLOAT : 0.50 SVF^2                        ; only if corrections OK
any   -> NOFIX : 2.0 max(0, 0.25 - SVF)
NOFIX -> SINGLE: 0.5 SVF + 0.05
Moving-base heading (RELPOS) : lose 0.002 + 0.30 (1-SVF)^2 ; gain 0.50 SVF^2 ; valid only if RELPOS fixed and fix >= 1
Outliers: in FLOAT/SINGLE with SVF < 0.7, p = 0.005 per epoch, jump U(2,5) m in random direction, 1 epoch
```
Sky-view factor SVF in [0.02, 1]:
```
SVF = svf_zone(p_a) * prod_i ( 1 - a_c * g(rho_i) ),  rho_i = |p_a,xy - c_i| / Rc_i
g(rho) = 1 (rho <= 0.5);  (1.3 - rho)/0.8 (0.5 < rho < 1.3);  0 otherwise ;  a_c = 0.45
svf_zone: open 1.0, greenhouse interior 0.12, under-panel strip 0.45, between panel rows 0.75
```
Reported accuracies: hAcc = sigmaH(fix) * clamp(1 + 0.15 n, 0.7, 1.5) (same for vAcc, sAcc); numSV = round(6 + 22 SVF).
Dual-antenna heading: psi_meas = psi + n, sigma = 0.5 deg (0.8 m baseline), same latency.

### 3.3 Track encoders and odometry

```
count_i  = wrap_uint32( round( theta_m,i * CPR / (2 pi) ) )                 (the ESC model, DRV_FB3, D3)
dCount_i = int32( count_i[k] - count_i[k-5] )   (uint32 wrap-safe window difference, reinterpret)   (MCU, 0.01 s)
vTrack_i = dCount_i * 2 pi r_s / (CPR * N_g * T_w)                         (sprocket pitch-line speed, T_w = 0.05 s)
```
CPR = 4096 (`P.drive.esc.enc_cpr`), N_g = 20 (`P.drive.gear.N`), r_s = 0.09 m (`P.structure.r_sprocket`) -> **about 144,900 counts/m**; speed resolution over the 0.05 s window (`P.sensors.odom.window`, 5 steps of `P.electronics.drv.odom_window`) = 1.4e-4 m/s
(negligible). The dominant odometry error is **slip** (v_ground = (1 - s) vTrack), which emerges from the STRUCTURE
track-terrain model and is estimated by the EKF.

### 3.4 LiDAR (Ouster OS1-32 class)

Hardware basis: 32 ch, 360 deg x 45 deg (+-22.5), 1024 x 10 Hz, 0.5-90 m (45 m @ 10 %), +-1-3 cm, IP68/69K,
-40...+60 C (fits 45-50 C ambient), ~20 W @ 24 V, Gigabit Ethernet to companion. Mount r_L = [0.45 0 0.75] m [EST] (`P.sensors.lidar.r`: the front of the 1.0 m body, above the 0.57 m deck), level.
**Why 3D:** 20 deg slopes make a planar scanner hit ground or overshoot obstacles, the panel lower edge (0.8 m, D10; `P.world.obstacles.box` zBase in world 5) and
overhanging fronds need height information, and people / low obstacles must be detected while pitching. GNC however
consumes a **2D height-band projection** (z in [0.15, 1.20] m above the ground, `P.sensors.lidar.zBand`), occupancy grid and
obstacle list, so two simulation fidelities are provided:

* **FAST2D** (default; any mission length): 360 beams x 1 deg, `rangeSensor` ray casting on a `binaryOccupancyMap`
  rasterised (once, at first call) from all primitives whose vertical extent overlaps the band; second map layer for
  semi-transparent greenhouse film (return probability 0.6, otherwise ray passes through); analytic ray-circle
  intersection for up to 4 moving obstacles (people; see 7.5); range noise sigma_r = 0.02 m (`lidar.sigR`) (1 + 0.005 r); dropouts p = 0.005
  + 0.03 * dust; dust clutter p = 0.02 * dust with spurious ranges U(0.3, 3.0) m; effective max range
  40 (1 - 0.5 dust) m.
* **FULL3D** (validation runs <= 10 min): `UAV Scenario Lidar` (31 x 360 organized cloud, the nEl x nAz of 7.5) on a `uavScenario`
  containing the same geometry (cylinders for trunks/posts/piles, polygons for walls, boxes and panel tables).
  The companion then runs `sens_lidar_cloud2scan` (leveling with roll/pitch, ground/band filter, 1 deg binning).
  In HW the same function runs on the real cloud (32 x 1024).

### 3.5 Navigation EKF (companion, `cstblocks_estimation/Extended Kalman Filter`)

**State x (11):**

| # | Symbol | Unit | Meaning | Model |
|---|---|---|---|---|
| 1 | pE | m | base_link East | kinematic |
| 2 | pN | m | base_link North | kinematic |
| 3 | pU | m | base_link Up | kinematic |
| 4 | psi | rad | yaw (ENU, CCW from East), wrapped | gyro-driven |
| 5 | v | m/s | body-x speed (along track direction) | accel-driven |
| 6 | vy | m/s | body-y skid velocity (side-slope drift, skid-steer ICR) | Gauss-Markov tau_vy = 1 s |
| 7 | bg | rad/s | level-frame yaw-rate bias | random walk |
| 8 | ba | m/s^2 | body-x accel bias | random walk |
| 9 | sL | - | left track longitudinal slip | GM to s0 = 0.005, tau_s = 20 s while aided; **held (tau_s_denied 1e6 s, prior = estimate at denial onset, process noise 2e-6) during GNSS + scan-match denial** (revision B, D49), clamp [-0.5, 0.6] |
| 10 | sR | - | right track longitudinal slip | same |
| 11 | chi | - | effective gauge factor, B_eff = chi B | GM to chi0 = `P.structure.track.chi_nom` (identified in T03, D39), tau_chi = 300 s, clamp [1.0, 2.5] |

**Inputs from preprocessing (100 Hz):** level-frame yaw rate `psidot_m = (wy sin(phi) + wz cos(phi)) / cos(theta)`;
body-x specific force fx (physical convention); sin/cos of theta from the IMU Filter.

**Process model** (Euler, dt = 0.01 s), omega = psidot_m - bg:
```
pE+  = pE + dt ( v cT cPsi - vy sPsi )
pN+  = pN + dt ( v cT sPsi + vy cPsi )
pU+  = pU - dt  v sT                              (theta > 0 = nose down = descending)
psi+ = wrap( psi + dt omega )
v+   = v  + dt ( fx - ba + g sT + omega vy )      (stationary nose-down: fx = -g sT -> vdot = 0)
vy+  = vy (1 - dt/tau_vy)
bg+  = bg ;  ba+ = ba
sL+  = sL + dt (s0 - sL)/tau_s ;  sR+ likewise
chi+ = chi + dt (chi0 - chi)/tau_chi
```
Jacobian F = I + dt A, non-zero A entries:
`A(1,4) = -(v cT sPsi + vy cPsi)`, `A(1,5) = cT cPsi`, `A(1,6) = -sPsi`,
`A(2,4) = v cT cPsi - vy sPsi`, `A(2,5) = cT sPsi`, `A(2,6) = cPsi`, `A(3,5) = -sT`, `A(4,7) = -1`,
`A(5,6) = omega`, `A(5,7) = -vy`, `A(5,8) = -1`, `A(6,6) = -1/tau_vy`, `A(9,9) = A(10,10) = -1/tau_s`, `A(11,11) = -1/tau_chi`.

**Measurement models** (all additive noise; lever arm l = R_WB(psi,theta,phi) r_a):

| Ch | Measurement | Rate | h(x,u) | Wrapping | R |
|---|---|---|---|---|---|
| 1 | GNSS antenna position [E N U] | 10 Hz | `p + l - tau * v_a`, `v_a = [pdotE - omega l_y; pdotN + omega l_x; pdotU]` (latency back-extrapolation) | no | diag(kR^2 [hAcc^2 hAcc^2 vAcc^2]), time-varying |
| 2 | GNSS antenna velocity [E N] | 10 Hz | `v_a(1:2)` | no | update: diag(kRvel(fix)^2 [sAcc^2 sAcc^2]), kRvel = [SINGLE FLOAT FIXED] = kR [1 1 1] in the simulation (kR [3 2 1] cost accuracy once the plant velocity was zero-mean: the lead's S4 test of 2026-11-14); a hook for the real robot, where the Doppler velocity under a greenhouse roof may be less certain than sAcc says; the chi-square test keeps kR |
| 3 | Heading (dual antenna, or ecompass at init only) | 10 Hz | `psi` | `[-pi pi]` | headAcc^2 kR^2 (ecompass: (8 deg)^2) |
| 4 | Track speeds [vL vR] (sprocket) | 20 Hz (every 5th 0.01 s step, D13) | `[(v - omega chi B/2)/(1-sL) ; (v + omega chi B/2)/(1-sR)]` | no | diag(sigma_od^2), sigma_od = 0.01 + 0.03 |vTrack| |
| 5 | LiDAR scan-to-map pose [E N psi] | 5 Hz | `[pE; pN; psi]` | `[-Inf Inf; -Inf Inf; -pi pi]` | max(Cov_match, floor) x 2; start guard on a weak translation axis (an aisle, eigenvalue ratio >= smWeakRatio 4): when the prior along it exceeds both smCap 0.15 m and the match's own weak-axis sigma (about 0.24 m in S4, so smCap binds in under 10 % of the matches) and the match moved less than its own sigma, that axis gets the prior's variance (the update at most halves it), before the chi-square test. It delays a match that keeps returning its start by a few scans (about 3, 0.6 s, from a 0.6 m prior); it does not catch it |

Observability: v from GNSS velocity/position; omega from gyro; with GNSS present sL, sR are observable on straight
driving and chi during turns; in GNSS denial slip/chi revert to priors (bounded by Gauss-Markov), odometry plus
scan-matching then carry the solution. Odometry at standstill (vL = vR = 0) implies v = 0 and omega = 0, giving ZUPT
and gyro-bias observability without a separate channel.

Innovation gating (chi-square, p = 0.999): m=1: 10.83, m=2: 13.82, m=3: 16.27. After N_max consecutive rejections the
measurement is force-accepted with R x k_force (avoids lock-out after a genuine jump, e.g. RTK re-fix).

### 3.6 Attitude and initialization

* Roll/pitch: `mspfiltlib/IMU Filter` (6-axis; no magnetometer because of motor-current disturbance) at 100 Hz.
  Centripetal compensation before the filter: a_kin,y = v_hat[k-1] * omega[k] (Unit Delay on EKF v).
* Initial heading when dual-antenna heading is unavailable: `mspfiltlib/ecompass` (stationary, |I| < 5 A),
  psi_true = psi_mag - declination; fed into EKF channel 3 through a Multiport Switch with R = (8 deg)^2.
* No runtime state injection is needed: x0 = 0 with a large P0 (sigma_pos = 100 m, sigma_psi = pi); first GNSS fix
  and heading converge the filter in 1-2 updates. `navValid` is the health flag of D89: the mode manager raises it once
  the filter is aligned (position sigma under `nav.sigPos` 0.15 m and yaw sigma under `nav.sigYaw` 2 deg, ALIGN
  to RUN), and keeps it while the filter runs, the IMU is valid, an absolute update came within `nav.tAbsMax` 10 s
  (35 s decided by REQ-SYS-023, pending on card G2-NAV; 7.6)
  and the larger of the East and North position sigmas stays under `navmode.sigPosValid` 2 m. The accuracy that
  the path needs is judged by the GNC's gate of D69 (0.15 m across the lane, 0.25 m along it), not by `navValid`.

### 3.7 Perception algorithms (10 Hz)

* Scan filter: NaN removal, self-mask box x in [-0.55, 0.55], y in [-0.42, 0.42] m (body only), isolated
  near-return (dust) removal: r < 3 m and both neighbours differ by > 0.3 m.
* Local occupancy grid: `occupancyMap(30,30,10)` rolling with the robot (`move(...,'MoveType','Absolute')`),
  `insertRay` (hit 0.7 / miss 0.4, saturation [0.12 0.97], max range 30 m), output thresholded at 0.65 into the TEB
  `mapInfo` struct (fields verified from `nav.slalgs.internal.TimedElasticBand`: Resolution, GridLocationInWorld,
  GridSize, OccupancyMatrix, GridOriginInLocal).
* Obstacles: adaptive break-point segmentation (lambda = 10 deg, sigma_r = 0.02 m), clusters >= 2 points,
  centroid / radius / min range, up to 20, sorted by range; 8 sector minima (45 deg each, sector 1 centred forward,
  CCW); protective fields from the fitted implement (`per.protXF` front, `per.protXR` rear): the front field starts at the front of the body or implement and runs length 0.5 + 0.8 |v| m, width 1.08 m (on open ground, palm groves and transits, the field ahead grows to at least 3.4 m at 1.0 m/s and 4.0 m at 1.2 m/s (REQ-SYS-131, the upper ends of D108's ranges), for a person walking head-on: D108, decided, pending on cards G2-FSC and G2-GNC; in a greenhouse it stays, the residual risk accepted under D79; today `per.protL0` 0.5 m and `per.protKv` 0.8 s on every site); the rear field starts at the rear of the body or implement. Both fields are on below 0.1 m/s (`per.protVBoth`); above it only the field in the direction of travel is on. protectiveStop is also set on a lost or stale scan (more than `per.protNStale` 2 consecutive invalid scans; 1-2 invalid scans hold the last result). A stale scan here is an invalid one: a frozen cloud that the driver keeps re-publishing stays valid and is not caught (STATUS 28, card G2-NAV). One kept return in a field raises protectiveStop; within 3 m an isolated return is removed as dust (`per.dustR`, `dustDr`), so there a body must give two adjacent returns. The zone from the body front to about 0.45 m ahead lies inside the LiDAR's 0.5 m minimum range and is not seen; D84 counts it as occupied until the operator's resume (decided, pending on card G2-GNC; today H1 releases the hold after 1.9 s of clear valid scans, `P.gnc.fsm.t_clear_eff`).
* **Test person** (REQ-SYS-033; set by risk assessment and not yet confirmed against ISO 18497-2:2024 and ISO 18497-4:2024, D103): an adult standing, walking at up to 1.6 m/s or kneeling, with a body 0.3 m wide (up to 1.0 m high kneeling, 1.8 m standing); a person lying on the ground wherever the field lies beyond 1.1 m from the LiDAR (nearer than that a body 0.3 m high is below the lowest beam, which is 0.54 m above the ground at the 0.5 m minimum range); and a child of about six, 0.2 m wide and 1.1 m tall. The child gives at least three adjacent 1 deg returns within 3 m, and two at D108's far corner (about 4.8 m from the LiDAR with the mower at 1.2 m/s), where one return suffices. Residual risks accepted under D109, both on card G2-FSC: a person lying within 1.1 m of the LiDAR at a start or a resume (D84's near zone, REQ-SYS-004's warning and D79), and a person hidden in grass taller than the body (the operator checks the area before a mission). The twin's person is a cylinder of radius 0.25 m at 1 m/s; FAST2D sees a lying body at any range, which is optimistic, so the lying case needs FULL3D (planned on card G1-02).
* Row detection (palm 8 m rows, solar piles): candidate clusters with radius 0.05-0.5 m in x in [-4, 20] m (`per.rowXWin`),
  |y| in [1.0, 6.0] m; RANSAC line fit per side (inlier 0.25 m, 50 iterations, >= 3 inliers); outputs lateral offset
  to row centre, heading error, side distances, fit RMS.
* Scan-to-map localization (5 Hz, greenhouse / GNSS-denied): virtual reference scan from the prior map at the predicted
  lidar pose (`rangeSensor`), `matchScansGrid(cur, ref, 'TranslationSearchRange',[0.3 0.3], 'RotationSearchRange',0.1745,
  'Resolution',10, 'MaxRange',10)` (D55; revision A had [0.5 0.5], 20 and 30 m), correction = predicted (+) relPose, covariance rotated to W and floored.
  Live check in R2026b (greenhouse aisle test): matchScansGrid recovered [0.15 0.05 0.031] vs true [0.154 0.035 0.02]
  with along-aisle variance 2.7x the cross-aisle variance (corridor degeneracy correctly reflected); NDT `matchScans`
  failed along the aisle (0.04 m vs 0.15 m), so **matchScansGrid is selected**. Interpreted timing: rangeSensor 9 ms,
  matchScansGrid 81 ms per call claimed in revision A; the review measured 9-19 s with MaxRange 30 / Resolution 20. Revision B (D55): Resolution 10, MaxRange 10 m, TranslationSearchRange [0.3 0.3] -> **29 ms per call** (interpreted, 9.6 m aisle, correct pose), 5 Hz and only when active.
  Along-aisle covariance (S4 nav fix round 2, 2026-11-14): `stats.Covariance` samples the score in +-10 cells and
  keeps every sample above 60 % of the best, so along an aisle (posts and end walls hold < 40 % of the points) it
  returned the sampling window (0.52 m) and the EKF ignored a match good to 0.07-0.12 m. The weak translation axis is
  now the matcher's strong-axis variance times the anisotropy of the point information of the matched scan
  (line points: their normal; isolated points such as posts, jambs, edges: both axes; `per.smGeoGap` 0.25 m,
  `smGeoDmax` 0.30 m), capped at the search window. On 349 logged S4 aisle scans: 0.15-0.19 m (median) against
  0.085-0.12 m RMS error, per-scan NEES 0.2-0.5; consecutive matches are correlated over about 3 matches at most
  while driving; the honesty at standstill is not measured (the S4 run after the change had 7 standing matches). Capture along the aisle is one-sided: a start behind the truth is recovered from 0.6 m, from 0.15 /
  0.3 m ahead one third of the offset remains, and from 0.45-0.6 m ahead deeper than 6 m the start comes back; the
  start guard of channel 5 (3.5) only delays that case by a few scans, it does not catch it (open, STATUS 16d).

---

## 4. Exact block list (verified in live R2026b unless stated)

### 4.1 Plant sensor models (`Shaddad_SensorModels.slx` and children)

| # | Block (library path) | Verified | Use | Key mask parameters (real names) and values |
|---|---|---|---|---|
| P1 | `mspsensorlib/IMU` (System `fusion.simulink.imuSensor`; in: Linear Acceleration, Angular Velocity, Orientation; out: Accel, Gyro, Mag) | yes | IMU sensor errors | `DispRefFrame='ENU'`, `TemperaturePort='on'`, `MagneticFieldPort='off'`, `MagneticFieldENU='P.sensors.imu.magENU'`, `SeedDouble='P.sensors.imu.seed'`, `SimulateUsing='Code generation'`, `AccelParamsMeasurementRange`, `AccelParamsResolution`, `AccelParamsConstantBias`, `AccelParamsAxesMisalignment`, `AccelParamsNoiseDensity`, `AccelParamsBiasInstability`, `AccelParamsRandomWalk`, `AccelParamsNoiseType='double-sided'`, `AccelParamsTemperatureBias`, `AccelParamsTemperatureScaleFactor`, `GyroParams*` (same set + `GyroParamsAccelerationBias`), `MagParams*` - values in 6.2. Orientation input = quaternion q_WI (4x1, [w x y z]). Driven at 0.005 s via Rate Transition. |
| P2 | `mspsensorlib/GPS` x3 (System `fusion.internal.simulink.gpsSensor`; in: Position, Velocity; out: LLA, Velocity, Groundspeed, Course) | yes | RTK-FIXED / RTK-FLOAT / SINGLE noise sets | `DispRefFrame='ENU'`, `PositionInputFormat='Local'`, `ReferenceLocation='P.sensors.refLLA'`, `HorizontalPositionAccuracy`, `VerticalPositionAccuracy`, `VelocityAccuracy` (`'0'` since the S4 nav fix round 2: the velocity noise is added after the switch, zero mean per axis, section 3.2), `DecayFactor`, `SeedDouble` (derived from the seed base in `param_sensors.m`), `SimulateUsing='Code generation'`. Accuracy params are tunable only via set_param, not signals -> three blocks. Inputs from Rate Transition at 0.1 s. |
| P3 | `simulink/Discrete/Delay` | yes | GNSS latency on truth (and heading) | `DelayLength='P.sensors.gnss.latencySteps'` (8), `InitialCondition='0'`, `SampleTime='0.01'` |
| P4 | `simulink/Signal Attributes/Rate Transition` | yes | all rate boundaries | `Integrity='on'`, `Deterministic='on'`, `OutPortSampleTime` = 0.005 / 0.01 / 0.1 / 0.001 |
| P5 | `simulink/Signal Routing/Multiport Switch` | yes | select GNSS block by fix state; heading source | `Inputs='3'`, `DataPortOrder='Specify indices'` with `DataPortIndices='{4,5,1}'`, `DataPortForDefault='Last data port'` |
| P6 | `simulink/Sources/Uniform Random Number` | yes | Markov transitions (4-vector), outlier/drop/dust/film draws (360-vectors) | `Minimum='0'`, `Maximum='1'`, `Seed` (from `P.sim.seed_base`, `param_sensors.m`), `SampleTime` 0.1 |
| P7 | `simulink/Sources/Random Number` | yes | heading noise, hAcc report jitter, LiDAR range noise (360-vector) | `Mean`, `Variance`, `Seed` (from `P.sim.seed_base`, `param_sensors.m`), `SampleTime` 0.1 |
| P8 | `simulink/Sources/Band-Limited White Noise` x2 | yes | accel & gyro vibration source | `Cov='P.sensors.imu.vib.cov'`, `Ts='0.005'`, `seed` (401-406) |
| P9 | `simulink/Discrete/Discrete Transfer Fcn` | yes | vibration band-pass (10-60 Hz, 2nd order), IMU DLPF (1st order 50 Hz) | `Numerator`, `Denominator` (from `P.sensors.imu.vib.bpNum/bpDen`, `P.sensors.imu.dlpf.num/den`), `SampleTime='0.005'` |
| P10 | `simulink/Math Operations/Product` | yes | R_WB * vector (`Multiplication='Matrix(*)'`), |v| scaling (element-wise) | `Multiplication` |
| P11 | `aerolibutil/3x3 Cross Product` | yes (exists, SubSystem) | lever-arm terms alpha x r, omega x (omega x r), omega x r_a | - |
| P12 | `navlib/Utilities/Coordinate Transformation Conversion` | yes | quaternion <-> rotation matrix, q_WI from R_WB R_BI | `InputRepresentation='Rotation Matrix'`, `OutputRepresentation='Quaternion'`, `SimulateUsing='Code generation'` |
| P13 | `simulink/Math Operations/Gain` | yes | -1 accel sign flip, CPR/(2 pi) | `Gain` |
| P14 | `simulink/Discontinuities/Quantizer` | yes | encoder counts: not used, the ESC model gives them (D3) | - |
| P15 | `simulink/Signal Attributes/Data Type Conversion` x2 per side | yes | encoder counts: not used, the ESC model gives them (D3) | - |
| P16 | `simulink/User-Defined Functions/MATLAB Function` | yes | `sens_gnss_fixchain`, `sens_lidar2d_fast` (Section 5); `sens_env_skyview` is replaced by 2-D Lookup Tables on `P.world.gnssSvf` / `gnssZoneId` (D59) | - |
| P17 | `simulink/Ports & Subsystems/Variant Subsystem` | yes | LiDAR fidelity FAST2D / FULL3D | `VariantControlMode='expression'`, `VariantActivationTime='update diagram'`, choices `SENS_LIDAR_MODE==1` / `==2` |
| P18 | `uavsimlib/UAV Scenario Configuration` | yes | FULL3D scenario holder | `UAVScenarioObject='sens_uavScene'` (base workspace object built by `utilities/sens_env_to_uavscenario.m`), `SampleTime='0.1'` |
| P19 | `uavsimlib/UAV Scenario Motion Write` (in: Position, Orientation, Velocity, Accleration [sic], AngularVelocity) | yes | write robot pose into scenario | `PlatName='''Shaddad'''`, `ReferenceFrame='ENU'` |
| P20 | `uavsimlib/UAV Scenario Lidar` (inner System `uav.sluav.internal.system.UAVScenarioLidarGenerator`, interpreted) | yes | FULL3D point cloud | `SensorName='Shaddad/Lidar'`, `MaxRange='45'`, `RangeAccuracy='0.015'`, `AzimuthLimits='[-180 180]'`, `AzimuthResolution='1.0'`, `ElevationLimits='[-22.5 22.5]'`, `ElevationResolution='1.45'`, `HasNoise='on'`, `HasOrganizedOutput='on'`, `SampleTime='0.1'` |
| P21 | `simulink/Signal Routing/Bus Creator` / `Bus Selector` | yes | bus packing | `OutDataTypeStr='Bus: Bus_ImuRaw'` etc. |
| P22 | `simulink/Discontinuities/Saturation` | yes | clamp SVF, |v| scaling | `UpperLimit`, `LowerLimit` |
| P23 | `simulink/Math Operations/Trigonometric Function` | yes | heading wrap via atan2(sin, cos) | `Operator='atan2'` / `'sin'` / `'cos'` |
| P24 | `simulink/Ports & Subsystems/Subsystem Reference` | yes | all plant sensor files | `ReferencedSubsystem` |

Rejected alternatives (and why): `drivingscenarioandsensors/Lidar Point Cloud Generator` (needs drivingScenario actors,
cuboid geometry, road inputs); `robotsim3dlib/Simulation 3D Lidar` (Unreal engine, GPU, unusable for multi-hour runs);
`mspsensorlib/INS` (outputs a finished INS solution, not raw GNSS - kept only as an optional "ideal nav" stub for GNC
unit tests, verified: `MountingLocation`, `RollAccuracy`, `PitchAccuracy`, `YawAccuracy`, `PositionAccuracy`,
`VelocityAccuracy`, `TimeInput`, `PositionErrorFactor`).

### 4.2 STM32 acquisition (inside ELECTRONICS firmware model)

| # | Block | Verified | Use | Key parameters |
|---|---|---|---|---|
| S1 | `stm32IMUsensorslib/ICM20948 IMU Sensor` (System `matlabshared.sensors.simulink.ICM20948Block`; out: Acceleration, Angular Rate, Magnetic Field, Temperature) | yes | HW variant of `sens_acq_imu` | `I2CModule='I2C1'`, `I2CAddress='0x69'`, `BitRate='400000'` (ICM-20948 I2C max 400 kHz; default 1e6 must be changed), `AccelerometerRange='+/-4g'`, `AccelerometerBW='50.4 Hz'`, `GyroscopeRange='+/-500 dps'`, `GyroscopeBW='51.2 Hz'`, `MagnetometerODR='100 Hz'`, `IsActiveTemp='on'`, `DataType='single'`, `SampleTime='0.005'` |
| S2 | `stm32blockslib/Encoder` (System `stm32cube.blocks.EncoderBlock`) x2 | yes | not used: the ESCs count the encoders and send DRV_FB3 over CAN (D3) | - |
| S3 | `simulink/Discrete/Difference` | yes | wrap-safe count delta (uint32) | `ICPrevInput='0'`, sample time inherited (0.01; window of 5 steps) |
| S4 | `simulink/Signal Attributes/Data Type Conversion` | yes | uint32 delta -> int32 reinterpret | `OutDataTypeStr='int32'`, `ConvertRealWorld='Stored Integer (SI)'` |
| S5 | `simulink/Math Operations/Gain` | yes | window counts -> m/s | 2 pi `odom.rs` / (`odom.CPR` `odom.Ng` `odom.window`), single |
| S6 | `mspfiltlib/IMU Filter` | yes | not built (NavLite dropped; the MCU's tilt is `fw_imu_tilt`, D47) | - |
| S7 | `simulink/Ports & Subsystems/Variant Subsystem` | yes | SIM / HW | `VariantControlMode='expression'`, choices `SHADDAD_TARGET==0` (SIM) / `==1` (HW) - global variable name to be fixed with ELECTRONICS |
| S8 | `simulink/Discrete/Discrete FIR Filter` | yes | IMU 2-sample averaging option | `Coefficients='[0.5 0.5]'` |

### 4.3 Companion: `Shaddad_NavFusion.slx`

| # | Block | Verified | Use | Key parameters |
|---|---|---|---|---|
| N1 | `simulink/Discrete/Discrete FIR Filter` | yes | 200 -> 100 Hz anti-alias average of IMU | `Coefficients='[0.5 0.5]'`, `SampleTime='0.005'` (+ Rate Transition to 0.01) |
| N2 | `mspfiltlib/IMU Filter` (in: Accel, Gyro; out: Orientation, Angular Velocity) | yes | roll / pitch | `DispRefFrame='ENU'`, `OrientationFormat='quaternion'`, `DecimationFactor='1'`, `AccelerometerNoise='0.05'`, `GyroscopeNoise='1e-5'`, `GyroscopeDriftNoise='1e-11'`, `LinearAccelerationNoise='0.3'`, `LinearAccelerationDecayFactor='0.7'`, `InitialProcessNoise='P.sensors.nav.imufQ0'` (diag [1e-2 1e-2 1e-2 4e-3 4e-3 4e-3 0.0096 0.0096 0.0096]; the toolbox default left a 3 deg tilt error at 10 s), `SimulateUsing='Code generation'` |
| N3 | `mspfiltlib/ecompass` | yes | initial magnetic heading | `ReferenceFrame='ENU'`, `OrientationFormat='quaternion'`, `SimulateUsing='Code generation'` |
| N4 | `navlib/Utilities/Coordinate Transformation Conversion` | yes | quaternion -> Euler ZYX | `InputRepresentation='Quaternion'`, `OutputRepresentation='Euler Angles'`, `OutputEulerSequence='ZYX'` |
| N5 | `aerolibtransform2/LLA to Flat Earth` (in: lat/lon, h, href) | yes | LLA -> local (N,E,D) then Selector/Gain -> ENU | `units='Metric (MKS)'`, `ptype='Earth (WGS84)'`, `LL0='P.sensors.refLLA(1:2)'`, `psi='0'`, `refPosPort='off'`; href = `P.sensors.refLLA(3)` |
| N6 | `aerolibtransform2/Flat Earth to LLA` | yes | NavState LLA output | `LL0='P.sensors.refLLA(1:2)'`, `psi='0'` |
| N7 | `cstblocks_estimation/Extended Kalman Filter` | yes | main nav filter | see table 4.3.1 |
| N8 | `simulink/Ports & Subsystems/Resettable Subsystem` | yes | wraps N7 for NavReset | reset port `ResetTriggerType='rising'` (see risk R2) |
| N9 | `simulink/Discrete/Unit Delay` | yes | break gating loop (xhat, P), v_hat for centripetal comp. | `InitialCondition='P.sensors.ekf.x0'`, `SampleTime='0.01'` |
| N10 | `simulink/User-Defined Functions/MATLAB Function` | yes | `sens_nav_gate`, `sens_nav_mode` (Section 5) | - |
| N11 | `simulink/Signal Routing/Multiport Switch` | yes | heading source (dual antenna / ecompass) | `Inputs='2'` |
| N12 | `simulink/Signal Attributes/Rate Transition` | yes | 0.005/0.01/0.05/0.1/0.2 boundaries | `Integrity='on'`, `Deterministic='on'` |
| N13 | `ros2lib/Subscribe` | yes | HW variant: GNSS topics (`/ubx_nav_pvt`, `/ubx_nav_rel_pos_ned`) | `topicSource='Specify your own'`, `topic`, `messageType`, `SubscriberMode='At sample time'`, `sampleTime='0.1'` (custom ublox msgs require `ros2genmsg`, Q3) |

#### 4.3.1 Extended Kalman Filter block settings (real parameter names)

| Parameter | Value |
|---|---|
| `StateTransitionFcn` | `sens_ekf_f` |
| `HasStateTransitionJacobianFcn` / `StateTransitionJacobianFcn` | `'on'` / `sens_ekf_dfdx` (bring-up may use `'off'` = numeric) |
| `HasAdditiveProcessNoise` | `Additive` |
| `ProcessNoise` | `P.sensors.ekf.Qd` (11x11) |
| `HasTimeVaryingProcessNoise` | `off` |
| `InitialState` / `InitialStateCovariance` | `P.sensors.ekf.x0` / `P.sensors.ekf.P0` |
| `MeasurementFcn1..5` | `sens_ekf_h_gnsspos`, `sens_ekf_h_gnssvel`, `sens_ekf_h_heading`, `sens_ekf_h_odom`, `sens_ekf_h_lidarpose` |
| `HasMeasurementJacobianFcn1..5` / `MeasurementJacobianFcn1..5` | `'on'` / `sens_ekf_dh_gnsspos`, `..._gnssvel`, `..._heading`, `..._odom`, `..._lidarpose` |
| `HasMeasurementEnablePort1..5` | `on` (driven by `sens_nav_gate`) |
| `HasAdditiveMeasurementNoise1..5` | `Additive` |
| `HasTimeVaryingMeasurementNoise1..5` | `on` (R ports from `sens_nav_gate`) |
| `MeasurementNoise1..5` | defaults `P.sensors.ekf.R0{j}` (unused when time-varying) |
| `HasMeasurementWrapping3`, `HasMeasurementWrapping5` | `on` (functions return `[z, bounds]` - verified in `EKFCorrectorAdditive.m`: `[zEstimated,wrapping] = h(x, varargin{:})`) |
| `HasMeasurementWrapping1,2,4` | `off` |
| `EnableMultirate` | `off` (D63): every correction runs at 0.01 s when `sens_nav_gate` enables it |
| `StateTransitionFcnSampleTime` | `0.01` |
| `MeasurementFcn1SampleTime..5` | not used (multirate off); the channels arrive every 0.1, 0.1, 0.1, 0.05 and 0.2 s (`P.sensors.ekf.measTs`) |
| `UseCurrentEstimator` | `on` |
| `OutputStateCovariance` | `on` |
| `DataType` | `double` |
| Extra-input ports | f: `StateTransitionFcnInputs` (11x1); h1: `MeasurementFcn1Inputs` (7x1); h2: (6x1); h4: (2x1) |

### 4.4 Companion: `Shaddad_Perception.slx`

| # | Block | Verified | Use | Key parameters |
|---|---|---|---|---|
| C1 | `simulink/User-Defined Functions/MATLAB Function` | yes | `sens_lidar_cloud2scan`, `sens_scan_filter`, `sens_occmap_update`, `sens_obstacle_extract`, `sens_row_detect`, `sens_scanmatch_loc` | - |
| C2 | `simulink/Ports & Subsystems/Variant Subsystem` | yes | scan source: FAST2D scan pass-through / cloud2scan (FULL3D or HW) | `SENS_LIDAR_MODE` |
| C3 | `ros2lib/Subscribe` + `ros2lib/Read Point Cloud` | yes | HW: `/ouster/points` (sensor_msgs/PointCloud2) | Subscribe `topic='/ouster/points'`, `messageType='sensor_msgs/PointCloud2'`, `sampleTime='0.1'`; Read Point Cloud `MaximumPointCloudSize='[32 1024]'`, `PreserveStructure='on'`, `ShowErrorCodeOutput='on'`, `VariableSizeOutputs='off'` |
| C4 | `motalgorithmslib/Global Nearest Neighbor Multi Object Tracker` | yes | OPTIONAL phase 2b: track moving obstacles (people) | `FilterInitializationFcn='initcvekf'`, `MaxNumTracks='20'`, `MaxNumSensors='1'`, `AssignmentThreshold='[30 Inf]'`, `TrackLogic='History'`, `pHistoryConfThreshold='[2 3]'`, `pHistoryDelThreshold='[5 5]'`, `SimulateUsing='Code generation'` (detections need objectDetection bus) |
| C5 | `simulink/Signal Attributes/Rate Transition` | yes | 0.1 -> 0.2 s scan-match | |
| C6 | `simulink/Discrete/Unit Delay` | yes | delayed nav pose input (breaks Nav<->Perception loop) | `SampleTime='0.01'` |

### 4.5 Consumers already verified for interface compatibility (owned by GNC)
* `navlib/Control Algorithms/Timed Elastic Band (TEB)` - map input is a struct with fields Resolution,
  GridLocationInWorld, GridSize, OccupancyMatrix (logical), GridOriginInLocal -> `Bus_TebMapInfo` matches.
* `navlib/Control Algorithms/Vector Field Histogram` - inputs Ranges, Angles, TargetDir -> `Bus_Perception.ranges_B/angles_B`.
* `robotmobilelib/Pure Pursuit` - Pose [x y theta] -> `Bus_NavState.pos_W(1:2)`, `eul(1)`.

### 4.6 Test-harness blocks
* `robotmobilelib/Differential Drive Kinematic Model` (verified: `VehicleInputs='Vehicle Speed & Heading Angular Velocity'`,
  `WheelRadius`, `TrackWidth='P.sensors.geom.B'` (0.60 m), `InitialState`, `SimulateUsing='Code generation'`) - standalone truth generator.
* `simulink/Sinks/To Workspace` (verified) - logging (decimated; never log the 300x300 map at 10 Hz).

---

## 5. MATLAB Function / MATLAB System blocks (only where no block exists)

All functions are `%#codegen`, fixed-size (NaN padding), stored in `subsystems/sensors/` (on project path) and called
from MATLAB Function blocks (parameters passed as non-tunable struct parameters, e.g. `P.sensors.env`).
No MATLAB System block is needed.

| # | Function | Where / rate | Why no block exists |
|---|---|---|---|
| F1 | `[svf, zoneId] = sens_env_skyview(pAnt, env)` | plant, 0.1 s | replaced by 2-D Lookup Tables on `P.world.gnssSvf` / `gnssZoneId` (D59); the pseudo-code below is revision A's |
| F2 | `[fixType, relposFixed, outlier] = sens_gnss_fixchain(svf, u, corrOK, corrAge, prm)` | plant, 0.1 s | State machine; Stateflow not installed (would be `sflib/Chart`) |
| F3 | `[ranges, valid] = sens_lidar2d_fast(poseL, t, nR, uDrop, uDust, uFilm, dust, env, prm)` | plant, 0.1 s | `rangeSensor` is a function/System object without a Simulink block; map must be rasterised from env |
| F4 | `[ranges, valid, nPts] = sens_lidar_cloud2scan(xyz, nAz, rp, prm)` | companion, 0.1 s | Lidar Toolbox not installed; no point-cloud-to-scan block |
| F5 | `[en, R1..R5, nis, rej] = sens_nav_gate(xPrev, Pprev, meas..., u1, u2, u4, prm)` | companion, 0.01 s | EKF block has no innovation gating; builds time-varying R from receiver accuracies |
| F6 | `[mode, quality, navValid, aligned, headSel, lidarLocOn, resetReq] = sens_nav_mode(...)` | companion, 0.01 s | State machine (Stateflow not installed) |
| F7 | `[rangesF, anglesB, rangesB] = sens_scan_filter(ranges, angles, prm)` | companion, 0.1 s | vector filtering + frame change |
| F8 | `map = sens_occmap_update(poseL, ranges, angles, valid, prm)` | companion, 0.1 s | `occupancyMap`/`insertRay` have no block |
| F9 | `[obs, sectors, protStop] = sens_obstacle_extract(rangesF, angles, v, prm)` | companion, 0.1 s | segmentation algorithm |
| F10 | `row = sens_row_detect(obs, prm)` | companion, 0.1 s | RANSAC line fit (no block) |
| F11 | `sm = sens_scanmatch_loc(poseNav, ranges, angles, active, map, prm)` | companion, 0.2 s | `matchScansGrid` / `rangeSensor` have no block |
| EKF model functions | `sens_ekf_f`, `sens_ekf_dfdx`, `sens_ekf_h_*`, `sens_ekf_dh_*` | called by EKF block | required by the existing EKF block (not blocks themselves) |

Pseudo-code (condensed):

```matlab
function [svf, zoneId] = sens_env_skyview(pAnt, env)          % F1
svf = 1; zoneId = uint8(0);
for k = 1:env.nZones                                          % polygons: greenhouse, panel strips
  nv = env.zoneNv(k);
  if inpolygon(pAnt(1), pAnt(2), env.zoneX(k,1:nv), env.zoneY(k,1:nv))
     svf = min(svf, env.zoneSvf(k)); zoneId = env.zoneType(k); end
end
for i = 1:env.nPalm                                           % canopy circles, bbox pre-test
  d = hypot(pAnt(1)-env.palm(i,1), pAnt(2)-env.palm(i,2)); rho = d/env.palm(i,4);
  if rho < 1.3, g = min(1, (1.3-rho)/0.8); svf = svf*(1 - env.canopyAtt*g); end
end
svf = max(svf, 0.02);
```
```matlab
function [fixType, relposFixed, outlier] = sens_gnss_fixchain(svf, u, corrOK, corrAge, prm)   % F2, u = 4 uniforms
persistent s h; if isempty(s), s = prm.initFix; h = true; end
dt = prm.Ts; p = @(lam) 1 - exp(-lam*dt);
lamNF = 2.0*max(0, 0.25 - svf);
switch s
 case 4, if corrAge > 10 || u(1) < p(0.002 + 0.5*(1-svf)^2), s = 5; elseif u(2) < p(lamNF), s = 0; end
 case 5, if corrOK && corrAge < 10 && u(1) < p(0.2*svf^3), s = 4;
         elseif corrAge > 30 || u(1) > 1 - p(0.3*(1-svf)^3), s = 1; elseif u(2) < p(lamNF), s = 0; end
 case 1, if corrOK && u(1) < p(0.5*svf^2), s = 5; elseif u(2) < p(lamNF), s = 0; end
 otherwise, if u(1) < p(0.5*svf + 0.05), s = 1; end
end
if h, h = ~(u(3) < p(0.002 + 0.3*(1-svf)^2)); else, h = u(3) < p(0.5*svf^2); end
fixType = uint8(s); relposFixed = h && s >= 1;
outlier = (s == 5 || s == 1) && svf < 0.7 && u(4) < prm.pOutlier;
```
```matlab
function [ranges, valid] = sens_lidar2d_fast(poseL, t, nR, uDrop, uDust, uFilm, dust, env, prm)  % F3
persistent mapO mapF rs
if isempty(mapO)
  mapO = binaryOccupancyMap(env.W, env.H, prm.mapRes); mapO.GridLocationInWorld = env.origin;
  mapF = binaryOccupancyMap(env.W, env.H, prm.mapRes); mapF.GridLocationInWorld = env.origin;
  rasterise circles (r, zmin..zmax overlapping prm.zBand), boxes, segments (thickness 1 cell) -> setOccupancy(mapO, xy, true)
  rasterise film segments -> mapF
  rs = rangeSensor('Range',[prm.rMin prm.rMax],'HorizontalAngle',[-pi pi-prm.res], ...
                   'HorizontalAngleResolution',prm.res,'RangeNoise',0);           % 360 beams
end
[rO, ang] = rs(poseL', mapO);
rD = ray_circle(poseL, ang, dyn_positions(env.dyn, t));  rO = min(rO, rD, 'omitnan');
if env.hasFilm, rF = rs(poseL', mapF); m = rF < rO & uFilm < prm.pRetFilm; rO(m) = rF(m); end
r = rO + prm.sigR*(1 + 0.005*rO).*nR;
r(r > prm.rMax*(1 - 0.5*dust) | uDrop < prm.pDrop + 0.03*dust) = NaN;
m = uDust < prm.pDustMax*dust;  r(m) = 0.3 + 2.7*uDust(m)./(prm.pDustMax*dust);
ranges = single(r); valid = true;
```
```matlab
function [ranges, valid, nPts] = sens_lidar_cloud2scan(xyz, nAz, rp, prm)     % F4 (xyz 32x1024x3 NaN-padded)
P = reshape(xyz(:,1:nAz,:), [], 3)';  P = P(:, all(isfinite(P),1));
PB  = prm.R_BL*P + prm.r_L;                                  % to base_link
PLv = rotY(rp(2))*rotX(rp(3))*PB;                            % level with estimated pitch/roll
k = PLv(3,:) > prm.zBand(1) & PLv(3,:) < prm.zBand(2) & ~inSelfBox(PB, prm.selfBox);
dx = PLv(1,k) - prm.r_L(1); dy = PLv(2,k) - prm.r_L(2);
az = atan2(dy, dx); rr = hypot(dx, dy); bin = floor((az + pi)/prm.res) + 1;
ranges = NaN(360,1,'single'); for each point: ranges(bin) = min(ranges(bin), rr);
nPts = nnz(k); valid = nPts > 50;
```
```matlab
function [en,R1,R2,R3,R4,R5,nis,rej] = sens_nav_gate(xP, PP, g, hd, od, sm, u1, u2, u4, prm)  % F5
persistent rc; if isempty(rc), rc = zeros(5,1); end
% build R from reports
R1 = diag(prm.kR^2*[g.hAcc^2 g.hAcc^2 g.vAcc^2]); R2 = diag(prm.kR^2*[g.sAcc^2 g.sAcc^2]);
R3 = prm.kR^2*hd.acc^2;  R4 = diag((prm.od0 + prm.od1*abs(od.v)).^2);  R5 = max(sm.cov, prm.smFloor)*prm.smInfl;
% (built: R1 / R2 of the update use kRpos(fix) / kRvel(fix), the test uses kR; R5 passes the start guard, 3.5)
avail = [g.new && g.fix>0; g.new && g.fix>0; hd.valid; od.valid; sm.valid];
for j = 1:5 (if avail(j))
  [zh, bnd] = h_j(xP, u_j);  H = dh_j(xP, u_j);  nu = wrapRes(z_j - zh, bnd);
  S = H*PP*H' + R_j;  nis(j) = nu'*(S\nu);  pass = nis(j) < prm.chi2(m_j);
  if ~pass, rc(j) = rc(j)+1; if rc(j) > prm.Nmax(j), pass = true; R_j = R_j*prm.kForce(j); rc(j) = 0; end
  else, rc(j) = 0; end
  en(j) = pass;
end
rej = rc;
```
```matlab
function [mode,quality,navValid,aligned,headSel,lidarLocOn,resetReq] = sens_nav_mode(cmd, P, fix, tAbs, nisAvg, stat, prm)  % F6
% states: INIT -> ALIGN -> RUN_GNSS | RUN_LIDAR | DEAD_RECKONING -> FAULT
% INIT: IMU valid for 2 s -> ALIGN
% ALIGN: stationary; headSel = dual antenna if valid else ecompass (only if |I|<5 A);
%        aligned when sigma_pos < 0.15 m and sigma_psi < 2 deg -> RUN_*
% RUN_LIDAR when cmd.navMode==2, or AUTO with a prior map and no RTK epoch (fix not 4 or 5) for > 5 s (navmode.tLidarSwitch)
% DEAD_RECKONING when no absolute update for > 2 s (nav.drTimeout); navValid false after 10 s without one
%   (nav.tAbsMax; the GNC stops, G1)
% navValid (D89, a health flag) = aligned && RUN_* && tSinceAbs < tAbsMax && max(sigma_E, sigma_N) < 2 m
%   (navmode.sigPosValid) && IMU valid; the accuracy that the path needs is the GNC's gate (D69)
% FAULT when moving-average NIS > 3x expected for 5 s -> resetReq pulse (rising edge to Resettable Subsystem); this FAULT is transient
% IMU invalid (imu_valid false) for navmode.tImuLossInv 0.1 s -> navValid false, quality 0 (INVALID); counted in every state
% IMU invalid for navmode.tImuLossFault 1 s from ALIGN, RUN_GNSS, RUN_LIDAR or DEAD_RECKONING -> FAULT with a reset pulse;
%   FAULT is held while the IMU is invalid; a second reset and INIT follow when it is valid again
% a stuck IMU reaches NavFusion as valid false after P.electronics.safety.imu_frozen_t (0.1 s)
% quality: 0 INVALID,1 DR,2 GNSS_SINGLE,3 GNSS_FLOAT,4 RTK_FIXED,5 LIDAR_LOCALIZED
```
```matlab
function map = sens_occmap_update(poseL, ranges, angles, valid, prm)      % F8
persistent om; if isempty(om), om = occupancyMap(prm.size, prm.size, prm.res);
  om.ProbabilityUpdateValues ... (hit 0.7 / miss 0.4); om.ProbabilitySaturation = [0.12 0.97]; end
move(om, poseL(1:2)', 'MoveType','Absolute', 'FillValue', 0.5);
if valid, insertRay(om, poseL', double(ranges), double(angles), prm.maxRange); end
map.Resolution = om.Resolution; map.GridLocationInWorld = om.GridLocationInWorld;
map.GridSize = om.GridSize; map.OccupancyMatrix = occupancyMatrix(om) > prm.occThr;
map.GridOriginInLocal = om.GridOriginInLocal;
```
```matlab
function sm = sens_scanmatch_loc(poseNav, ranges, angles, active, mapPrm, prm)   % F11
persistent mp rs; if isempty(mp), mp = rasterise(mapPrm, 20); rs = rangeSensor(same beams, no noise); end
sm = invalidSM(); if ~active, return; end
poseLp = compose(poseNav, prm.mountL);  [rr, aa] = rs(poseLp', mp);
cur = removeInvalidData(lidarScan(double(ranges), double(angles))); ref = removeInvalidData(lidarScan(rr, aa));
if cur.Count < 60 || ref.Count < 60, return; end
[rel, st] = matchScansGrid(cur, ref, 'InitialPose',[0 0 0], 'TranslationSearchRange',[0.3 0.3], ...   % D55
                           'RotationSearchRange',0.1745, 'Resolution',10, 'MaxRange',10);
inside = all(abs(rel(1:2)) < 0.9*0.3) && abs(rel(3)) < 0.9*0.1745;
sm.valid = inside && st.Score/cur.Count >= prm.minScoreRatio;
poseB = compose(compose(poseLp, rel), inverse(prm.mountL));
J = blkdiag(rot2(poseLp(3)), 1);  C = J*st.Covariance*J';
sm.pose = poseB; sm.cov = max(C, diag(prm.floor)); sm.score = st.Score; sm.nPts = cur.Count;
% (built: the weak translation axis of st.Covariance from the scan geometry before the rotation, 3.7)
```
`sens_obstacle_extract`, `sens_row_detect`, `sens_scan_filter` implement Section 3.7 directly (segmentation loop,
sector minima, protective-field box test, RANSAC with fixed 50 iterations and `coder`-safe random draws from a
deterministic LCG seeded per scan).

EKF function signatures (exact):
```matlab
xNext = sens_ekf_f(x, u)              % u = [psidot_m; fx; sinTheta; cosTheta; dt; g; tau_vy; tau_s; s0; tau_chi; chi0]
F     = sens_ekf_dfdx(x, u)
z     = sens_ekf_h_gnsspos(x, u1)     % u1 = [r_a(3); phi; theta; tau; psidot_m]           -> 3x1
z     = sens_ekf_h_gnssvel(x, u2)     % u2 = [r_a(3); phi; theta; psidot_m]                -> 2x1
[z,b] = sens_ekf_h_heading(x)         % b = [-pi pi]                                       -> 1x1
z     = sens_ekf_h_odom(x, u4)        % u4 = [psidot_m; B]                                 -> 2x1
[z,b] = sens_ekf_h_lidarpose(x)       % b = [-Inf Inf; -Inf Inf; -pi pi]                   -> 3x1
H     = sens_ekf_dh_<name>(x[, u])
```

---

## 6. Interfaces

Sample-time key: Ts_imu = 0.005, Ts_nav = 0.01, Ts_odom = 0.01 (EKF odometry update every 0.05 s, D13), Ts_gnss = Ts_lidar = 0.1, Ts_sm = 0.2 s. The encoder counts come from the ESC model in DRV_FB3 (D3). The built buses are in `buses/buses_sensors.m` and the ICD; where this section differs, they rule.

### 6.1 Inputs

| Signal (bus element) | From | Type | Dim | Units | Rate |
|---|---|---|---|---|---|
| `Bus_TrueState.t` | STRUCTURE | double | 1 | s | continuous (sampled by RT) |
| `.pos_W` | STRUCTURE | double | 3 | m (ENU, base_link) | cont. |
| `.vel_W` | STRUCTURE | double | 3 | m/s | cont. |
| `.acc_W` | STRUCTURE | double | 3 | m/s^2 (kinematic, no gravity) | cont. |
| `.q_WB` | STRUCTURE | double | 4 | - ([w x y z], the frame rotation W->B, `buses/buses_structure.m`) | cont. |
| `.eul_WB` | STRUCTURE | double | 3 | rad [yaw pitch roll] ZYX | cont. |
| `.omega_B` | STRUCTURE | double | 3 | rad/s (FLU) | cont. |
| `.alpha_B` | STRUCTURE | double | 3 | rad/s^2 | cont. |
| `.vel_B` | STRUCTURE | double | 3 | m/s | cont. |
| `.slip_LR` | STRUCTURE | double | 2 | - (truth, for evaluation) | cont. |
| `.vGround_LR` | STRUCTURE | double | 2 | m/s (truth track-ground speed) | cont. |
| `Bus_DriveSensorFeed.thetaMotor_LR` | DRIVE | double | 2 | rad, unwrapped, +ve = forward track motion both sides | cont. |
| `.omegaMotor_LR` | DRIVE | double | 2 | rad/s | cont. |
| `.iDC_LR` | DRIVE (or POWER) | double | 2 | A (inverter DC current) | cont. |
| `Bus_ElecEnv.T_imu_degC` | ELECTRONICS | double | 1 | degC (enclosure / PCB) | 0.1 s |
| `.gnssCorrOK` | ELECTRONICS | boolean | 1 | - (NTRIP link up) | 0.1 s |
| `.gnssCorrAge` | ELECTRONICS | double | 1 | s | 0.1 s |
| `.sensPowerOK` | ELECTRONICS | uint8 | 1 | bitmask b0 IMU, b1 GNSS, b2 LiDAR | 0.1 s |
| `P.sensors.env.dustLevel` (scenario signal option) | scenario | double | 1 | 0..1 | 0.1 s |
| `Bus_NavCmd.navMode` | GNC | uint8 | 1 | 0 AUTO, 1 GNSS, 2 LIDAR_PRIOR_MAP, 3 DR_TEST | 0.05 s |
| `.navReset` | GNC | boolean | 1 | rising edge | 0.05 s |
| `.priorMapId` | GNC | uint8 | 1 | - | 0.05 s |
| `Bus_ImuMeas` (companion input) | ELECTRONICS (STM32) | see 6.3 | | | 0.005 s |
| `Bus_OdomMeas` (companion input) | ELECTRONICS (STM32) | see 6.3 | | | 0.01 s |

Defaults when a neighbour is not yet available: T_imu = 45 degC, corrOK = true, corrAge = 1 s, iDC = 0, dust = 0.

### 6.2 Outputs

| Signal | To | Type | Dim | Units | Rate |
|---|---|---|---|---|---|
| `Bus_ImuRaw` (mirrors ICM20948 block outputs) | ELECTRONICS (STM32 SIM variant) | single | 3/3/3/1 | m/s^2, rad/s, uT, degC | 0.005 |
| `Bus_EncoderRaw.count_LR` | not built: the ESC model gives the counts in DRV_FB3 (D3) | | | | |
| `Bus_GnssRaw` | companion SIM variant (`sens_acq_gnss`) | mixed | - | - | 0.1 |
| `Bus_LidarScanRaw` (FAST2D) | companion SIM variant (`sens_acq_lidar`) | single | 360 | m | 0.1 |
| `Bus_LidarCloudRaw` (FULL3D) | companion SIM variant | single | 32x1024x3 | m | 0.1 |
| `Bus_SensTruthDiag` | logging / tests | double | - | - | 0.1 |
| `Bus_ImuMeas`, `Bus_OdomMeas` (from `sens_acq_imu` and `sens_odom_compute` inside the firmware; NavLite not built) | companion | single | - | - | 0.005 / 0.01 |
| `Bus_NavState` | GNC, RC (telemetry), logging | double | - | - | 0.01 |
| `Bus_NavDiag` | logging, GNC health | double | - | - | 0.01 |
| `Bus_Perception` | GNC (TEB, VFH, safety, row following) | mixed | - | - | 0.1 |
| `Bus_ScanMatchMeas` | Perception -> NavFusion (internal to sensors) | double | - | - | 0.2 |

### 6.3 Bus definitions (for `buses/shaddad_buses.m`)

**Bus_ImuRaw** (0.005 s): `Acceleration` single[3] m/s^2 (sensor frame, physical convention); `AngularRate` single[3] rad/s;
`MagneticField` single[3] uT; `Temperature` single degC.

**Bus_EncoderRaw**: not built (the ESC model gives the counts in DRV_FB3, D3).

**Bus_GnssRaw** (0.1 s): `t_meas` double s (time of validity); `lla` double[3] (deg, deg, m); `vel_ENU` double[3] m/s;
`fixType` uint8 (0/1/4/5); `numSV` uint8; `hAcc` single m; `vAcc` single m; `sAcc` single m/s; `headingValid` boolean;
`heading_deg` double (CW from N, baseline); `headAcc_deg` single; `corrAge` single s; `newData` boolean.

**Bus_LidarScanRaw** (0.1 s): `t_scan` double; `ranges` single[360] (NaN = no return, sensor frame, beam k at
angle -pi + (k-1) deg); `angles` single[360] rad; `valid` boolean.

**Bus_LidarCloudRaw** (0.1 s): `t_scan` double; `xyz` single[32x1024x3] (sensor frame, NaN padded); `nAz` uint16; `valid` boolean.

**Bus_SensTruthDiag** (0.1 s): `svf` double; `zoneId` uint8; `fixTypeTruth` uint8; `relposFixed` boolean;
`gnssPosErr` double[3]; `outlier` boolean; `dustLevel` double; `lidarPoseTruth` double[3].

**Bus_ImuMeas** (0.005 s, STM32 -> companion): `accel_B` single[3] m/s^2 (body FLU, physical, calibrated);
`gyro_B` single[3] rad/s; `mag_B` single[3] uT; `temp_degC` single degC; `t_us` uint32 us; `seq` uint16; `valid` boolean.

**Bus_OdomMeas** (0.01 s): `vTrack_LR` single[2] m/s (over the 0.05 s window); `encCnt_LR` int32[2] (the latest counts, wrapping); `wMotor_LR` single[2] rad/s; `t_us` uint32; `valid` boolean.

**Bus_NavLite**: not built (NavLite dropped; the MCU's tilt limits are D47's, electronics spec 2.7.10).

**Bus_NavCmd** (0.05 s, the GNC's rate, D13): `navMode` uint8; `navReset` boolean; `priorMapId` uint8.

**Bus_NavState** (0.01 s):

| Element | Type | Dim | Units | Note |
|---|---|---|---|---|
| `t` | double | 1 | s | |
| `pos_W` | double | 3 | m | base_link ENU |
| `lla` | double | 3 | deg, deg, m | |
| `eul` | double | 3 | rad | [yaw pitch roll], yaw from EKF, pitch/roll from IMU Filter |
| `q_WB` | double | 4 | - | |
| `vel_B` | double | 3 | m/s | [v vy 0] |
| `vel_W` | double | 3 | m/s | |
| `yawRate` | double | 1 | rad/s | psidot_m - bg |
| `omega_B` | double | 3 | rad/s | bias-corrected gyro |
| `accel_B` | double | 3 | m/s^2 | kinematic (gravity removed) |
| `slip_LR` | double | 2 | - | |
| `chi` | double | 1 | - | |
| `gyroBiasZ`, `accelBiasX` | double | 1, 1 | rad/s, m/s^2 | |
| `posStd` | double | 3 | m | sqrt diag P |
| `yawStd`, `velStd` | double | 1, 1 | rad, m/s | |
| `covPose` | double | 3x3 | m^2, rad^2 | [pE pN psi] |
| `quality` | uint8 | 1 | enum | 0 INVALID..5 LIDAR_LOCALIZED |
| `navValid`, `headingValid`, `aligned`, `lidarLocActive` | boolean | 1 each | | |
| `gnssFixType` | uint8 | 1 | | |
| `tSinceAbsUpdate` | double | 1 | s | |
| `resetCount` | uint16 | 1 | | |

**Bus_NavDiag** (0.01 s): `nis` double[5]; `enable` boolean[5]; `rejCount` double[5]; `gnssLatency` double; `ekfTrace` double;
`P_diag` double[11]; `x` double[11]; `modeState` uint8.

**Bus_ScanMatchMeas** (0.2 s): `valid` boolean; `t` double; `pose` double[3] (base_link in W); `cov` double[3x3]; `score` double; `nPts` uint16.

**Bus_TebMapInfo** (0.1 s; field order = TEB default struct): `Resolution` double (10); `GridLocationInWorld` double[1x2] m;
`GridSize` double[1x2] ([300 300]); `OccupancyMatrix` boolean[300x300]; `GridOriginInLocal` double[1x2] m.

**Bus_RowFeature** (0.1 s): `valid` boolean; `lateralOffset` single m (+ = robot left of row centre); `headingErr` single rad;
`distLeft`, `distRight` single m; `nLeft`, `nRight` uint8; `fitRms` single m.

**Bus_Perception** (0.1 s): `t_scan` double; `scanValid` boolean; `ranges_B` single[360] m; `angles_B` single[360] rad
(base_link, for VFH); `minSectorRange` single[8] m; `protectiveStop` boolean; `protFieldLen` single m; `obsCount` uint8;
`obsPos_B` single[20x2] m; `obsRadius` single[20] m; `obsRange` single[20] m; `row` Bus_RowFeature; `map` Bus_TebMapInfo;
`scanMatch` Bus_ScanMatchMeas.

### 6.4 Simscape physical ports
**None.** The sensor domain is signal-only. Sensor electrical loads are given to ELECTRONICS as numbers (6.9) for their
power-distribution model; IMU temperature arrives as a signal (`Bus_ElecEnv.T_imu_degC`) from the ELECTRONICS thermal
network (PS-Simulink conversion on their side).

---

## 7. Parameters (`params/param_sensors.m` -> `P.sensors`)

### 7.1 General / frames
| Name | Value | Unit | Source / justification |
|---|---|---|---|
| `refLLA` | [24.1556 47.3120 430] | deg, deg, m | site datum (Al-Kharj), `P.world.lla0`; per mission |
| `g` | 9.7876 | m/s^2 | `gravitywgs84(430,24.15)` |
| `imu.magENU` | [1.745 34.116 -26.317] | uT | WMM2025 @ 2026.8 (`wrldmagm`) |
| `magDecl` | 2.93 | deg | WMM2025 |
| `geom.B` | 0.60 | m | track gauge, `P.structure.geom.B` |
| `imu.r` | [0.25 0.00 0.40] | m | MCU board in the electronics bay [EST, real robot] |
| `imu.eulMis_deg` | [0.3 -0.5 0.4] | deg | residual mounting misalignment (truth) |
| `gnss.rAnt1` | [-0.40 0.00 1.05] | m | primary (rear) antenna on mast |
| `gnss.rAnt2` | [0.40 0.00 1.05] | m | moving-base (front), baseline 0.80 m along x |
| `gnss.psiBaseline` | 0 | rad | baseline aligned with body x |
| `lidar.r` | [0.45 0.00 0.75] | m | front of the 1.0 m body, above the 0.57 m deck [EST] |
| `lidar.eul_deg` | [0 0 0] | deg | level mount |
| `SHADDAD_TARGET` / `SENS_LIDAR_MODE` | 0 / 1 | - | variant controls (SIM / FAST2D) |

### 7.2 IMU (ICM-20948, Ts = 0.005 s) - values passed to `mspsensorlib/IMU` from `P.sensors.imu.accel`, `.gyro` and `.mag`
| Parameter | Value | Justification |
|---|---|---|
| `AccelParamsMeasurementRange` | 39.2266 m/s^2 | +-4 g |
| `AccelParamsResolution` | 1.1971e-3 (m/s^2)/LSB | 8 g / 2^16 |
| `AccelParamsConstantBias` | [0.05 -0.04 0.08] m/s^2 | 5-8 mg residual after 6-position calibration |
| `AccelParamsAxesMisalignment` | 100 on diag, 0.5 off-diag (%) | +-2 % spec, calibrated residual |
| `AccelParamsNoiseDensity` | 2.2555e-3 (m/s^2)/sqrt(Hz) x3 | 230 ug/sqrt(Hz) datasheet |
| `AccelParamsBiasInstability` | 4.9e-4 m/s^2 | ~50 ug typical consumer MEMS |
| `AccelParamsRandomWalk` | 2.0e-4 (m/s^2)sqrt(Hz) | typical |
| `AccelParamsTemperatureBias` | 7.8e-3 (m/s^2)/degC | +-0.80 mg/C, the datasheet limit (DS-000189, D49) |
| `AccelParamsTemperatureScaleFactor` | 0.02 %/degC | spec 0.026 %/C |
| `GyroParamsMeasurementRange` | 8.7266 rad/s | +-500 dps |
| `GyroParamsResolution` | 2.6632e-4 (rad/s)/LSB | 1000 dps / 2^16 |
| `GyroParamsConstantBias` | [0.0061 -0.0073 0.0049] rad/s | 0.35/0.42/0.28 dps turn-on, estimated online |
| `GyroParamsAxesMisalignment` | 100 diag, 0.5 off-diag (%) | |
| `GyroParamsAccelerationBias` | 1.78e-4 (rad/s)/(m/s^2) | 0.1 dps/g |
| `GyroParamsNoiseDensity` | 2.618e-4 (rad/s)/sqrt(Hz) | 0.015 dps/sqrt(Hz) datasheet |
| `GyroParamsBiasInstability` | 4.85e-5 rad/s | 10 deg/h |
| `GyroParamsRandomWalk` | 2.0e-5 (rad/s)sqrt(Hz) | typical |
| `GyroParamsTemperatureBias` | 8.7e-4 (rad/s)/degC | +-0.05 dps/C, the datasheet limit (DS-000189, D49) |
| `GyroParamsTemperatureScaleFactor` | 0.02 %/degC | |
| `MagParamsMeasurementRange` | 4900 uT | AK09916 |
| `MagParamsResolution` | 0.15 uT/LSB | AK09916 |
| `MagParamsConstantBias` | [1.5 -2.0 1.0] uT | hard-iron residual |
| `MagParamsNoiseDensity` | 0.085 uT/sqrt(Hz) | ~0.6 uT rms @ 50 Hz BW |
| `MagParamsBiasInstability` | 0.1 uT | |
| `MagParamsTemperatureBias` | 0.02 uT/degC | |
| `Temperature` (if port off) | 45 degC | Saudi enclosure typical (`imu.T_default_degC`) |
| `SeedDouble` | `P.sim.seed_base + P.sim.seed.imu` | |
| `imu.kMagI` | [0.04 0.02 0.05] uT/A | traction-cable field estimate |
| `imu.vib.sigA` / `sigW` | [0.6 0.5 1.2] m/s^2 / [0.03 0.03 0.01] rad/s at 1 m/s | tracked-vehicle on soil, elastomer-isolated PCB |
| `imu.vib.band` | [10 60] Hz, 2nd-order Butterworth band-pass @ 200 Hz | track pitch & road-wheel excitation |
| `imu.dlpf.num/den` | 0.7921 / [1 -0.2079] | 50 Hz 1st-order @ 200 Hz |

### 7.3 GNSS (Ts = 0.1 s)
| Name | Value | Unit | Justification |
|---|---|---|---|
| `gnss.latency` / `latencySteps` | 0.08 / 8 | s / steps @0.01 | F9P RTK output + transport |
| `gnss.sigH` (FIXED/FLOAT/SINGLE) | 0.015 / 0.30 / 1.20 | m (1 sigma per axis) | F9P 0.01 m + 1 ppm; float 0.2-0.5 m; SPS ~1.5 m CEP |
| `gnss.sigV` | 0.030 / 0.60 / 2.50 | m | ~2x horizontal |
| `gnss.sigVel` | 0.02 / 0.03 / 0.05 | m/s | F9P 0.05 m/s spec; zero-mean white noise per axis added after the GPS blocks (3.2) |
| `gnss.seedVel` | seed_base + 100 seed.gnss + 12..14 | | velocity noise E N U (S4 nav fix round 2) |
| `gnss.decay` | 0.90 / 0.995 / 0.998 | - | correlation 1 s / 20 s / 50 s at 10 Hz |
| `gnss.seeds` | from `P.sim.seed_base` | | the three GPS blocks |
| `gnss.markov.*` (lambdas) | Section 3.2 | 1/s | tuned so palm-row RTK-fixed fraction ~40-80 % |
| `gnss.pOutlier`, `gnss.outlierMag` | 0.005, U(2,5) | -, m | multipath under canopy / near panels |
| `gnss.head_sigma_deg` | 0.5 | deg | 0.4 deg @ 1 m baseline scaled to 0.8 m |
| `gnss.corrAgeFixLimit`, `corrAgeFloatLimit` | 10, 30 | s | F9P RTK timeout behaviour |
| `gnss.initFix` | 0 | code | cold start NOFIX |
| `gnss.svf.*` | open 1.0, greenhouse 0.12, under-panel 0.45, between panels 0.75, canopyAtt a_c 0.45 | - | engineering estimate (validate with field logs) |
| `gnss.repJitter` | 0.15 | - | hAcc report scatter |

### 7.4 Encoder / odometry
| Name | Value | Unit | Justification |
|---|---|---|---|
| `odom.CPR` | 4096 | counts/rev motor | `P.drive.esc.enc_cpr`, 1024 PPR quadrature in the ESC |
| `odom.Ng` | 20 | - | `P.drive.gear.N` |
| `odom.rs` | 0.09 | m | sprocket pitch radius, `P.structure.r_sprocket` |
| `odom.window` | 0.05 | s | count difference over 5 steps (`P.electronics.drv.odom_window`) |
| `odom.Ts` | 0.01 | s | MCU Task_10 (D13); the EKF uses every 5th sample |
| `odom.sig0`, `odom.sig1` | 0.01 m/s, 0.03 | -, - | chordal (polygon) speed ripple ~4 % for 11 teeth, vibration |

### 7.5 LiDAR
| Name | Value | Unit | Justification |
|---|---|---|---|
| `lidar.Ts` | 0.1 | s | 10 Hz |
| `lidar.nBeams`, `res` | 360, 1 deg | | FAST2D virtual scan |
| `lidar.rMin`, `rMax` | 0.5, 40 | m | OS1 min range; 45 m @10 % -> 40 m usable |
| `lidar.sigR` | 0.02 | m | OS1 precision 1-3 cm |
| `lidar.zBand` | [0.15 1.20] | m | scan band above the ground (body 0.57 m, mast 1.05 m) |
| `lidar.pDrop`, `pDustMax`, `pRetFilm` | 0.005, 0.02, 0.6 | - | dropouts, dust clutter, PE film return |
| `lidar.mapRes`, `mapResGreenhouse` | 10 (palm, solar), 20 (greenhouse) | cells/m | thin greenhouse posts (r 0.05 m) |
| `lidar.maxCircles/Segments/Boxes/Dyn` | 1500 / 300 / 300 / 4 | - | fixed sizes for codegen |
| `lidar.full3d.*` | MaxRange 45, RangeAccuracy 0.015, Az [-180 180] @ 1.0, El [-22.5 22.5] @ 1.45 | | OS1-32 geometry, decimated azimuth for speed |
| `lidar.cloudSize` | [32 1024] | | HW cloud |

### 7.6 Navigation EKF
| Name | Value | Unit / note |
|---|---|---|
| `ekf.Ts` | 0.01 | s |
| `ekf.measTs` | [0.1 0.1 0.1 0.05 0.2] | s; odometry every 5th 0.01 s step (D13) |
| `ekf.x0` | [0 0 0 0 0 0 0 0 0.005 0.005 chi0]' | slip prior s0 0.005 (D49); chi0 = `P.structure.track.chi_nom` 1.3936 |
| `ekf.P0` | diag([1e4 1e4 100 9.87 0.25 0.01 7.6e-5 0.04 0.01 0.01 0.09]) | sigma 100 m pos, pi yaw, 0.5 deg/s bias |
| `ekf.qc` (continuous PSD) | [4e-4 4e-4 2.5e-3 7e-7 2.5e-3 5e-3 1e-8 2e-6 6.4e-4 6.4e-4 1.5e-4] | units^2/s; yaw = 10x gyro ARW^2 (vibration, scale factor); v = (0.5 m/s^2 vib)^2*dt; vy = 2*0.05^2/1; slip = 2*0.08^2/20; chi = 2*0.15^2/300; bg 1e-8 and ba 2e-6 raised for the enclosure's temperature rate (D49) |
| `ekf.Qd` | diag(qc)*0.01 | per step |
| `ekf.tau_vy`, `tau_s`, `s0`, `tau_chi`, `chi0` | 1.0 s, 20 s, 0.005, 300 s, `P.structure.track.chi_nom` (1.3936) | revision B (D39, D49); plus `tau_s_denied` 1e6 s, `qc_slip_denied` 2e-6, `t_denied` 0.5 s |
| `ekf.kR` | 1.5 | std inflation for coloured GNSS noise |
| `ekf.chi2` | [10.83 13.82 16.27] | m = 1/2/3, p = 0.999 |
| `ekf.Nmax` | [30 30 30 25 10] | consecutive rejections |
| `ekf.kForce` | [25 25 25 100 10] | R multiplier on forced accept |
| `ekf.smFloor` | diag([0.03^2 0.03^2 (0.5 deg)^2]) | |
| `ekf.smInfl` | 2.0 | |
| `navgate.kRvel` | kR [1 1 1] = [1.5 1.5 1.5] | GNSS velocity R inflation [SINGLE FLOAT FIXED], update only (the gate keeps kR); kR [3 2 1] cost accuracy in the S4 test of 2026-11-14 (along-aisle 0.095 against 0.065 m RMS, NEES 1.66 against 1.08) [EST: check the receiver's Doppler under the film roof against a LiDAR reference] |
| `navgate.kRpos`, `navgate.nBypass` | [5 5 2.5], 5 | GNSS position R inflation [SINGLE FLOAT FIXED] of the update; position epochs accepted without the gate after a fix improvement |
| `navgate.smWeakRatio`, `navgate.smCap` | 4, 0.15 m | channel-5 start guard: weak axis when its R5 variance is >= 4 x the other; prior sigma along it beyond 0.15 m and beyond the match's own weak-axis sigma (about 0.24 m in S4, so 0.15 m binds in under 10 % of the matches); it delays, it does not catch, a match that follows its start (a start 0.3 m ahead moves the match about 0.2 m, so the guard does not fire there) |
| `ekf.R0{1..5}` | 0.015^2 I3, 0.02^2 I2, (0.5 deg)^2, 0.02^2 I2, smFloor | defaults |
| `nav.sigPos`, `nav.sigYaw` | 0.15 m, 2 deg | alignment: ALIGN goes to RUN_GNSS or RUN_LIDAR once both sigmas are below these; `sigYaw` also bounds `headingValid` |
| `navmode.sigPosValid` | 2.0 m | `navValid` is false once the larger of the East and North position sigmas reaches 2 m: the health flag of D89. The accuracy that the path needs is judged by the GNC's gate of D69 (gnc spec T7), not by `navValid`. An error that the sigmas do not show is REQ-SYS-095's: beyond 0.15 m in a greenhouse aisle or on the dock approach a controlled stop starts within 0.5 s, and beyond 0.35 m in the open within 0.7 s, on the drift model of 0.3 m/s^2 lateral from rest (the lead systems engineer's ruling of step 19, point 11; no such monitor is built: decided, pending on cards G2-FSC and G2-NAV) |
| `nav.tAbsMax` | 10 s | `navValid` is false after 10 s without an accepted absolute update (a GNSS position or a scan match). REQ-SYS-023 sets 35 s (the lead systems engineer's ruling 3 of the merge, provisional); the parameter changes on card G2-NAV |
| `navmode.tImuLossInv`, `navmode.tImuLossFault` | 0.1 s, 1 s | `navValid` is false after 0.1 s of invalid IMU; FAULT follows after 1 s (D73) |
| `navmode.tLidarSwitch`, `tGnssReturn`, `tLidarFresh` | 5 s, 2 s, 1 s | RUN_LIDAR after 5 s without RTK on a site with a prior map; back to RUN_GNSS after 2 s of RTK fixed; a scan match counts as fresh for 1 s |
| `navmode.nisK`, `nisT`, `nisTau` | 3, 5 s, 2 s | FAULT when the mean normalised NIS stays above 3 for 5 s (time constant 2 s) |
| `nav.t_imu_valid` | 2 s | INIT goes to ALIGN after 2 s of valid IMU |
| `nav.drTimeout` | 2 s | -> DEAD_RECKONING |
| `nav.ecompass_sig`, `ecompass_iMax` | 8 deg, 5 A | |

### 7.7 Attitude filters
| Name | Value |
|---|---|
| `imuf.AccelerometerNoise` | 0.05 (m/s^2)^2 (white 2.5e-4 + vibration) |
| `imuf.GyroscopeNoise` | 1e-5 (rad/s)^2 |
| `imuf.GyroscopeDriftNoise` | 1e-11 (rad/s)^2 |
| `imuf.LinearAccelerationNoise` | 0.3 (m/s^2)^2 |
| `imuf.LinearAccelerationDecayFactor` | 0.7 |
| `navlite.tilt` | not built (NavLite dropped); the tilt limits are the MCU's (D47, electronics spec 2.7.10) |

### 7.8 Perception
| Name | Value |
|---|---|
| `per.selfBox` | x [-0.55 0.55], y [-0.42 0.42] m (body only) |
| `per.mapSize`, `mapRes`, `occThr`, `hit`, `miss`, `sat`, `maxRange` | 30 m, 10 cells/m, 0.65, 0.7, 0.4, [0.12 0.97], 30 m |
| `per.segLambda`, `minPts`, `maxObs`, `maxRadius` | 10 deg, 2, 20, 2.0 m |
| `per.nSectors`, `secLen` | 8 sectors of 45 beams (45 deg) |
| `per.protL0`, `protKv`, `protW` | 0.5 m, 0.8 s, 1.08 m (on open ground the field ahead grows per D108 to at least 4.0 m at 1.2 m/s, REQ-SYS-131; decided, pending on cards G2-FSC and G2-GNC) |
| `per.protXF`, `per.protXR` | from `P.structure.impl` (fitted implement): mower 1.20 / -0.55 m, tiller 0.55 / -1.125 m, sprayer and none 0.55 / -0.55 m |
| `per.protVBoth`, `per.protNStale` | 0.1 m/s, 2 scans [EST] |
| `per.rowRRange`, `rowXWin`, `rowYWin`, `rowInlier`, `rowIter`, `rowMinInl` | [0.05 0.5] m, [-4 20] m, [1 6] m, 0.25 m, 50, 3 |
| `per.smTs`, `smTrans`, `smRot`, `smRes`, `smMinScoreRatio`, `smMaxRange` | 0.2 s, [0.3 0.3] m, 0.1745 rad, 10 cells/m, 0.5, 10 m (D55) |
| `per.priorMapErr` | 0.03 m (as-built deviation of the prior map) |
| `per.smGeoGap`, `per.smGeoDmax` | 0.25 m, 0.30 m (weak axis of the scan-match covariance from the scan geometry: line / isolated point test, correspondence distance; S4 nav fix round 2) |
| `per.dustR`, `dustDr` | 3.0 m, 0.3 m (isolated near-return removal, 3.7) |
| `per.minPtsFarRange` | 8 m (beyond it a one-beam cluster counts) |
| `per.smNMin`, `smCovFloor` | 60 points; [0.02^2 0.02^2 (0.3 deg)^2] |
| `per.cloudMinPts` | 50 band points (`sens_lidar_cloud2scan` valid) |

### 7.9 Environment geometry (the worlds of `P.world`)
Since build task T11 the sensors see the worlds of `P.world` (structure spec): `P.sensors.env` copies their obstacles, canopies and GNSS zones (`params/param_sensors.m`), and the revision A builder `utilities/sens_env_build.m` is gone. The worlds' geometry is the structure spec's; for example the greenhouse world is 9.6 m x 37.5 m (`P.sensors.env.zoneX`, `zoneY` in world 4), not revision A's 9.0 m x 40.0 m. The prior map of the scan matcher is the world's map perturbed by `per.priorMapErr` 0.03 m.

### 7.10 Sensor power (to ELECTRONICS)
LiDAR 20 W @ 24 V (peak 28 W at start-up, < 60 C); 2x GNSS receivers 0.7 W each @ 5 V + active antennas 0.1 W each;
IMU 0.03 W @ 3.3 V; NTRIP LTE modem owned by ELECTRONICS.

---

## 8. Internal structure, files, model types, sample times

```
models/Shaddad_System.slx (top, other domains)
 |- Plant
 |   |- Structure (STRUCTURE)  --Bus_TrueState-->|
 |   |- Drive (DRIVE)          --Bus_DriveSensorFeed-->|
 |   |- Sensors : Subsystem Reference subsystems/sensors/Shaddad_SensorModels.slx
 |        |- sens_imu_model.slx       (SSR)  200 Hz  -> Bus_ImuRaw
 |        |- sens_gnss_model.slx      (SSR)  10 Hz (+100 Hz delay line) -> Bus_GnssRaw, Bus_SensTruthDiag
 |        |     |- 2-D Lookup Tables (SVF, D59), MATLAB Fcn sens_gnss_fixchain ; 3x GPS ; Multiport Switch
 |        |  (no encoder model: the ESC model gives the counts in DRV_FB3, D3)
 |        |- sens_lidar_model.slx     (SSR, Variant Subsystem SENS_LIDAR_MODE)
 |              |- FAST2D : MATLAB Fcn sens_lidar2d_fast             -> Bus_LidarScanRaw
 |              |- FULL3D : UAV Scenario Configuration/Motion Write/Lidar -> Bus_LidarCloudRaw
 |- STM32 firmware (ELECTRONICS, Model Reference) contains:
 |   |- sens_acq_imu.slx       (SSR, Variant SIM | HW: ICM20948 block) -> Bus_ImuMeas (after axis map + calib + timestamp)
  |   |- sens_odom_compute.slx  (SSR, MCU Task_10, 0.01 s)               -> Bus_OdomMeas (from DRV_FB3)
  |- Companion computer (GNC/ELECTRONICS top, Model Reference) contains:
     |- sens_acq_gnss.slx      (SSR, Variant SIM | HW ros2 Subscribe)  -> Bus_GnssRaw
     |- sens_acq_lidar.slx     (SSR, Variant SIM | HW ros2 Subscribe + Read Point Cloud)
     |- models/Shaddad_Perception.slx (Model Reference, base 0.1 s, 0.2 s scan-match) -> Bus_Perception, Bus_ScanMatchMeas
     |- models/Shaddad_NavFusion.slx  (Model Reference, base 0.005 s, rates 0.01/0.05/0.1/0.2) -> Bus_NavState, Bus_NavDiag
           |- NavPreproc (FIR avg, sign flip, centripetal comp., psidot_m, u-vector assembly, LLA->ENU, heading conversion)
           |- AttitudeFilter (IMU Filter, ecompass, Coordinate Transformation Conversion)
           |- NavGate (MATLAB Fcn sens_nav_gate, Unit Delays)
           |- NavEKF (Resettable Subsystem -> Extended Kalman Filter block)
           |- NavModeManager (MATLAB Fcn sens_nav_mode)
           |- NavOutput (Flat Earth to LLA, Bus Creator Bus_NavState / Bus_NavDiag)
```

| File | Type | Rates |
|---|---|---|
| `subsystems/sensors/Shaddad_SensorModels.slx` | Subsystem Reference (plant) | 0.001/0.005/0.01/0.1 |
| `subsystems/sensors/sens_imu_model.slx`, `sens_gnss_model.slx`, `sens_lidar_model.slx` | Subsystem Reference | as above |
| `subsystems/sensors/sens_acq_imu.slx`, `sens_odom_compute.slx` | Subsystem Reference used inside the MCU Model Reference | 0.005 / 0.01 |
| `subsystems/sensors/sens_acq_gnss.slx`, `sens_acq_lidar.slx` | Subsystem Reference (companion) | 0.1 |
| `models/Shaddad_NavFusion.slx` | Model Reference (codegen: ERT/ROS 2 node for companion), bus-typed root I/O | 0.005 base |
| `models/Shaddad_Perception.slx` | Model Reference | 0.1 base |
| `subsystems/sensors/*.m` | F1-F11, EKF functions | - |
| `utilities/sens_env_to_uavscenario.m` | helper (the FULL3D scene) | - |
| `params/param_sensors.m` | returns struct `P.sensors` | - |
| `tests/sensors/tb_sens_plant.slx`, `tb_nav.slx`, `tb_perception.slx`, `tb_per_scanfilter.slx`, `tb_per_tebvfh.slx` (built by `scripts/build/build_tb_*.m`), `t_sensors.m`, `t_nav.m`, `t_perception.m` | benches + matlab.unittest | - |

Solver settings for the two Model References: FixedStepDiscrete, fixed step 0.005 s, SingleTasking in simulation,
multitasking allowed in deployment; "Dynamic memory allocation in MATLAB functions" ON for the companion target
(needed by `lidarScan/removeInvalidData`); OFF / not used on STM32.

---

## 9. Verification (standalone harnesses, numeric pass/fail)

Benches: `tb_sens_plant.slx` = truth generator (`Differential Drive Kinematic Model` + slip injection + 3D
attitude profile) -> `Shaddad_SensorModels`; `tb_nav.slx` adds SIM acquisition, odometry and NavFusion, and `tb_perception.slx` runs Perception.
Statistics over >= 5 seeds unless noted.

| ID | Scenario | Pass criteria |
|---|---|---|
| T-SEN-01 | IMU static level, 60 s, 25 C, motors off | mean accel = [0.05 -0.04 9.7876+0.08] +- 0.01 m/s^2; std accel_z 0.0183 +- 20 % m/s^2; std gyro 2.12e-3 +- 20 % rad/s; mean gyro = ConstantBias +- 5e-4 rad/s |
| T-SEN-02 | Sign/convention: static 10 deg nose-down; yaw spin 0.5 rad/s | Bus_ImuRaw accel_x = -1.700 +- 0.02, accel_z = +9.639 +- 0.02 m/s^2; gyro_z = 0.5 + bias +- 0.005 rad/s; IMU Filter pitch = +10 +- 0.3 deg |
| T-SEN-03 | Lever arm: spot turn at 1.0 rad/s, with the IMU at r_I = [0.25 0 0.40] m (`P.sensors.imu.r`) | mean accel_x = -0.25 +- 0.01 m/s^2 (`tSEN03_LeverArm` still expects 0.35 m and -0.35 m/s^2: a known failure, card G2-NAV) |
| T-SEN-04 | Temperature ramp of 10 K | gyro bias change 10 x `GyroParamsTemperatureBias` = 8.7e-3 rad/s per axis +- 20 %; accel change 10 x `AccelParamsTemperatureBias` plus the scale-factor term on the measured acceleration |
| T-SEN-05 | GNSS open sky, stationary 300 s, RTK fixed | 100 % fix = 4 after 30 s; horizontal per-axis std 0.015 +- 0.005 m; heading std 0.5 +- 0.1 deg |
| T-SEN-06 | GNSS latency: 1 m/s straight | cross-correlation delay 0.08 +- 0.01 s |
| T-SEN-07 | Palm row centre, 160 m @ 1 m/s | RTK fixed fraction in [40, 80] %, float [15, 50] %, no-fix < 5 %; headlands fixed > 98 % after 30 s |
| T-SEN-08 | Greenhouse interior 10 min | fixed < 5 %; single+no-fix > 80 %; outlier rate 0.5 +- 0.2 % of float/single epochs |
| T-SEN-09 | Encoder: 3000 rpm 60 s, forced counter wrap (16-bit test variant too) | odometry speed mean error < 1e-4 m/s; no sample deviates > 1 count at wrap |
| T-SEN-10 | Odometry slip bias: plant s = 0.08 straight | raw odometry distance / truth = 1.087 +- 0.005 |
| T-SEN-11 | LiDAR FAST2D geometry: trunk r 0.3 m at 5.0 m ahead of lidar; wall at 10 m | beam 0 deg mean = 4.70 +- 0.06 m (20 scans); wall range std 0.020 +- 0.005 m; dropout fraction 0.5 +- 0.2 % |
| T-SEN-12 | LiDAR greenhouse film | fraction of film returns 0.6 +- 0.05 |
| T-SEN-13 | FAST2D vs FULL3D consistency (palm scene, 60 s) | median abs range difference on trunk beams < 0.05 m; ground points in 2D scan < 1 % on flat and 10 deg slope |
| T-SEN-14 | Performance | FAST2D compiled < 5 ms/scan; full sensor model adds < 15 min wall-clock to a 4-h FAST2D mission; FULL3D <= 0.2 s/scan |
| T-NAV-01 | Static alignment, RTK fixed + dual antenna | after 10 s sigma_pos < 0.03 m, sigma_psi < 0.8 deg; bg error < 0.002 rad/s at 30 s; navValid within 15 s |
| T-NAV-02 | Open field lawn-mower pattern 10 min @ 1 m/s, RTK fixed | horizontal RMS < 0.03 m, max < 0.08 m; yaw RMS < 0.5 deg; speed RMS < 0.03 m/s; mean 3-DOF pose NEES in [1.5, 4.5] |
| T-NAV-03 | Slip step s 0.03 -> 0.15 at t = 60 s (straight), then 0.5 rad/s turns | |s_hat - s| < 0.02 within 10 s; chi_hat within +-0.1 of truth within 60 s of turning |
| T-NAV-04 | Palm row under canopy (fix/float alternating), 104 m in the row (y 8 to 112 m, `P.world` world 3), driven at 1 m/s (131 m) | cross-track RMS < 0.10 m, max < 0.25 m; yaw RMS < 1.0 deg; FAULT/reset count = 0 |
| T-NAV-05 | Forced 30 s GNSS outage incl. 90 deg turn | position error at end < 0.5 m; yaw drift < 2 deg; back to < 0.05 m within 5 s of RTK re-fix |
| T-NAV-06 | Outlier injection 2-5 m at 0.5 %/epoch (float) | >= 95 % outliers rejected; < 1 % clean epochs rejected; no position spike > 0.10 m |
| T-NAV-07 | Greenhouse coverage of 240 s @ 0.6 m/s (the test's run; revision A planned 22 min), lidar localization, prior map with 3 cm errors | horizontal RMS < 0.10 m, max < 0.25 m; yaw RMS < 1.5 deg; scan-match valid > 80 % of attempts; NEES < 3x nominal |
| T-NAV-08 | Latency compensation: 2 m/s straight, tau 0.08 s | along-track bias < 0.02 m (uncompensated reference 0.16 m) |
| T-NAV-09 | NavReset pulse at t = 120 s | states return to x0/P0 (verifies Resettable Subsystem resets the EKF Data Store Memory); navValid false then true within 15 s |
| T-NAV-10 | ecompass init, no dual antenna, stationary, motors off | initial yaw error < 8 deg; declination sign verified |
| T-NAV-11 | 20 deg slope climb and side-slope traverse (STRUCTURE ramp) | pitch error < 1.0 deg; side-slip vy estimate within 0.03 m/s of truth; 3-D position RMS < 0.05 m (RTK; the test judges `rmsP3`, the vertical error included; the horizontal RMS was 0.039 m on 2026-11-08, so rows on a slope meet the row values of REQ-SYS-020) |
| T-NAV-12 | Gate unit (S4 nav fix round 2): GNSS velocity R per fix; channel-5 start guard (aisle along N, `t_nav/tGateVelAndSmGuard`) | R2 = (kRvel(fix) sAcc)^2, FIXED = kR; an uncertain start (0.30 m) and a match that did not move: weak-axis R5 = prior variance; a moved match, a confident start or an isotropic match: R5 unchanged |
| T-NAV-13 | IMU loss (`t_nav/tNAV13_ImuLoss`): dropout and stuck IMU, greenhouse LiDAR mode at 0.6 m/s; unit test `tModeUnitImuLoss` (tag fast) | Observed after the change: dropout: navValid false 0.10 s after the loss, FAULT at 1.00 s, held to the end, one reset; stuck IMU: navValid false after 0.20 s, FAULT at 1.10 s, one reset |
| T-NAV-14 | Validity health flag (D89; planned on card G1-02 as a `tModeUnit` case) | navValid false once the larger of the East and North position sigmas reaches 2 m (`navmode.sigPosValid`) and after 10 s without an absolute update (`nav.tAbsMax`); true below both |
| T-PER-01 | Obstacle 0.3 m box, open field, 1 m/s | detection probability per scan > 95 % within 20 m; position error < 0.10 m; false obstacles < 0.1 per scan at dust 0.5; the box is 0.3 x 0.3 x 0.5 m, and a box 0.30 m tall (REQ-SYS-036) is planned on card G1-02 |
| T-PER-02 | Protective field: the twin's person (r 0.25 m) enters at 1 m/s; the test persons of REQ-SYS-033 (1.6 m/s, the child, the lying person beyond 1.1 m) are planned on card G1-02 | protectiveStop within one scan (<= 0.15 s); extended by tPER02b (the field starts at the fitted implement, 3.7) |
| T-PER-07 | Lost scan (`t_perception/tPER07_ScanLost`): blank scans | protectiveStop from the 3rd consecutive invalid scan; 1-2 invalid scans hold the last result (no stop); true from start until the first valid scan |
| T-PER-03 | Row detection palm row (and solar piles) | lateral offset RMS < 0.10 m; heading error RMS < 1.5 deg; valid > 90 % of interior scans |
| T-PER-04 | Local map: 5 s stationary 3 m from wall, then drive 20 m | wall cells p > 0.9, free cells p < 0.3; no tearing after rolling move (map compared to truth raster, IoU > 0.8) |
| T-PER-05 | TEB / VFH compatibility | `Bus_TebMapInfo` drives `navlib/Control Algorithms/Timed Elastic Band (TEB)` and ranges/angles drive `Vector Field Histogram` with zero compile errors |
| T-PER-06 | Along-aisle honesty of the scan match on 197 logged S4 greenhouse-aisle scans (`tests/sensors/per_sm_aisle_scans.mat`, `t_perception/tSM01_AlongAisleHonesty`), from the logged start and from the true pose | valid > 90 %; along-aisle sigma median < 0.3 m (not the sampling window); per-scan along-aisle NEES <= 2; >= 95 % of the errors within 3 sigma; cross-aisle RMS < 0.06 m; the capture from 0.3 m behind / ahead is printed |
| T-SYS-01 | Codegen check | `Shaddad_NavFusion` and `Shaddad_Perception` build with Embedded Coder (ert/ROS 2 target) without errors; `sens_acq_imu` and `sens_odom_compute` build for STM32 in single precision |

---

## 10. Open questions and risks

### 10.1 Open questions
1. **STRUCTURE:** confirm base_link definition (ground-plane, mid-gauge, mid-contact) and FLU/ENU frames; can
   `Bus_TrueState` include `alpha_B`, `acc_W`, `slip_LR`, `vGround_LR`? Confirm sensor mount coordinates (IMU, mast
   height 1.05 m for antennas, LiDAR 0.75 m [EST]) and robot height (zBand upper limit).
2. **DRIVE:** encoder type/CPR (incremental 1024 PPR vs Hall-only: Hall-only (48 states/rev) gives ~1,700 counts/m
   and ~6 % quantization at 0.5 m/s @ 50 Hz), gear ratio 20, sprocket pitch radius 0.09 m (answered in `P`),
   sign convention of the right motor, whether encoder timers are free on the STM32 (TIM2/TIM5) or counts come over CAN (answered: the ESC sends them in DRV_FB3, D3)
   from a commercial motor controller.
3. **ELECTRONICS:** confirm fusion on the companion computer (proposal: Jetson Orin NX / industrial x86, ROS 2);
   STM32 -> companion link (CAN-FD vs Ethernet) and time synchronisation; ublox ROS 2 message generation
   (`ros2genmsg` for `ublox_ubx_msgs`) or a UBX serial parser; enclosure temperature signal; NTRIP link model
   (`gnssCorrOK`, `gnssCorrAge`); global variant variable name (`SHADDAD_TARGET`).
4. Dual-antenna GNSS (two F9P, 0.8 m baseline) - acceptable for cost? Without it, heading at standstill comes only from
   the magnetometer (+-8 deg) and course-over-ground when moving.
5. LiDAR model selection (Ouster OS1-32 vs Livox Mid-360 vs Hesai XT32) and whether a certified 2D safety scanner
   (ISO 18497-2:2024, obstacle protection systems; not yet confirmed against the text, D103) is required in addition - it would add a protective-field sensor model.
6. **GNC:** confirm the TEB map bus format, 100 Hz NavState rate, quality enum semantics, use of `rowLines`, who
   commands `navMode` (mission manager) and how prior maps are selected.
7. Stateflow: `license('test','Stateflow')` = 1 but `ver('stateflow')` is empty, so it is treated as not installed.
   If the user installs it, F2 and F6 become `sflib/Chart` blocks.
8. Vision-guided sprayer camera is not in this spec. Proposal: abstract detection model (per-plant Bernoulli detection
   p_d = 0.90, false-positive 0.05 within the boom footprint) owned by an implement/GNC agent; the Unreal
   `Simulation 3D Camera` block is too heavy for mission runs.
9. Site datum and geometry of the first target farm (row spacing 8 m vs 10 m, greenhouse 9 x 40 m vs gutter-connected).

### 10.2 Risks
| ID | Risk | Mitigation |
|---|---|---|
| R1 | Stateflow unavailable | MATLAB Function state machines (F2, F6) |
| R2 | EKF block keeps x/P in Data Store Memory; behaviour inside a Resettable Subsystem not confirmed | test T-NAV-09 at build; fallback = MATLAB System wrapping `extendedKalmanFilter` (codegen supported) with a reset input |
| R3 | `UAV Scenario Lidar` runs interpreted only (inner MATLAB System `SimulateUsing = Interpreted execution`), needs `sens_uavScene` in the base workspace, blocks Rapid Accelerator / `parsim` without per-worker scenario rebuild; ~0.05-0.2 s/scan | FULL3D only for <= 10-min validation; FAST2D default |
| R4 | Large signals: 300x300 logical map and 32x1024x3 cloud at 10 Hz -> 4-h logging could reach ~1.3 GB | no logging of map/cloud, decimated logging of NavState (10 Hz), Bus signals kept virtual where possible |
| R5 | GNSS block accuracies not signal-driven | three parallel blocks + switch; switching jumps are realistic (fix/float transitions) |
| R6 | Accelerometer sign convention of the toolbox (negative specific force) | single Gain -1 at plant output and before IMU Filter/ecompass; test T-SEN-02 |
| R7 | Greenhouse corridor degeneracy, PE-film transparency, crop growth vs prior map | matchScansGrid with covariance (verified behaviour), posts in map, R floor, gating, map update procedure |
| R8 | Slip / chi unobservable without GNSS | Gauss-Markov priors (bounded covariance), scan matching; the GNC's gate pauses the mission (D69), and navValid drops after 10 s without an absolute update (D89) |
| R9 | Magnetometer corrupted by 48 V traction currents | magnetometer used only stationary with |I| < 5 A; dual antenna preferred |
| R10 | Time sync STM32 <-> companion and GNSS latency jitter in HW (ideal in SIM) | timestamps in buses; latency term tau in h1 driven by measured age |
| R11 | Algebraic loops: gate uses EKF outputs; Perception uses nav pose and feeds scan-match back | Unit Delays on xhat/P and on pose into Perception (10 ms staleness, < 1 cm at 1 m/s) |
| R12 | Codegen: `rangeSensor`, `binaryOccupancyMap`, `occupancyMap`, `matchScansGrid`, `lidarScan` carry `%#codegen` (verified) but need dynamic memory on the companion; first-call map rasterisation 1-2 s and ~2-7 MB RAM | companion target only; fixed max sizes; rasterise at init |
| R13 | FAST2D ignores 3D effects (fronds, negative obstacles, ground undulation) | FULL3D validation runs; GNC safety margins; open question Q5 |
| R14 | Simulation speed: EKF + sensors ~1 min, FAST2D raycast 2-7 min, occupancy map 2-5 min per simulated 4 h; the Simscape plant will dominate | `SimulateUsing = 'Code generation'` on all MATLAB System blocks, compiled MATLAB Functions, Accelerator mode for the Model References |
| R15 | Multirate EKF requires measurement sample times to be integer multiples of 0.01 s | closed: multirate is off (D63); every correction runs at 0.01 s, enabled at the channel rates 0.05 / 0.1 / 0.2 s |
| R16 | Fix-chain lambdas and SVF model are engineering estimates | calibrate against field logs (NMEA quality histograms under canopy) |
| R17 | ICM-20948 on I2C at 400 kHz: ~0.7 ms per 9-axis read at 200 Hz (~14 % bus time) | acceptable; SPI would need a custom driver (blockset block is I2C only) |

### 10.3 Comments on the global conventions
* Subsystem Reference for plant sensors and Model Reference for NavFusion/Perception are suitable. Splitting
  Perception from NavFusion is recommended (separate rates, separate ROS 2 nodes, independent tests).
* The SIM/HW variant boundary should be **at the driver block** (ICM20948, Encoder, ROS 2 Subscribe), with SIM buses
  that mirror the driver outputs exactly (`Bus_ImuRaw` = ICM20948 outputs), so all calibration/axis-mapping/odometry
  code runs in both variants.
* "Double in plant" is fine. In the companion, keep nav in double (ENU coordinates up to 1e3-1e4 m); STM32 single only.
