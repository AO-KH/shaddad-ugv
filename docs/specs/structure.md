# STRUCTURE domain spec: chassis, mass properties, tracks, track-terrain interaction, implements, world/terrain

> **Revision B note (2026-10-19).** `docs/architecture.md` section 2b (D30-D63) and `docs/build_plan.md`
> override this spec where they differ. Sections changed for this domain: 2.5 / 2.6 / 3.2 / 4.2 track shear replaced by the per-point isotropic 2-D law (D39); chi identified in T03 and single-sourced (D39); 2-D shear damping uses a 1 ms delayed belt speed and implement load torques use 1 ms delayed shaft speeds (D36); `impl_mech` exported at 0.01 s (D32); T9 / T10 / T18 recalibrated and T7b / T18b added (D39, D53); 6.9 raised beds removed (D54); 3.5 Flat Earth to LLA href is an input port (D62).

> **Revision C note (2026-11-18, card G1-01 part 2).** The values of the as-built robot (XMachines MAX, `params/param_structure.m`) replace those of the earlier reference design: the geometry (§6.1), the mass breakdown and the mass properties with the default 1 kW motor and the as-built 750 W motor (§6.2, §6.3; D95), the track, contact and implement values (§6.4, §6.5, §6.7), the numbers derived from them in §2 and §5.5, the world (§6.9) and the test criteria as `t_structure` runs them (§8; T10 per D100). The enclosure thermal block and the obstacle monitor were never built in this domain (§2.9, §4.3); the buses are listed in `buses/buses_structure.m` and the ICD (§5.4). The decisions in force are D1 to D110; D104 to D110 change no structure value (step 19 and its consistency pass).

Project: Shaddad_UGV (tracked skid-steer agricultural UGV, NASL-Tech "Shaddad" class)
Domain key: `structure`. Status: BUILT; the values are the as-built robot's (G1-01 part 2). MATLAB R2026b.
Every library path in section 3 was checked in the live MATLAB session (`get_param(path,'Handle')`, `DialogParameters`, `PortHandles`) unless it is marked "not verified".

---

## 0. Key decisions (summary)

| Topic | Decision |
|---|---|
| Rigid body | **Aerospace Blockset `Simple Variable Mass 6DOF (Euler Angles)`** (built-in block `EOM6DOFBodyEuler`, `aerolib6dof2`). It gives full 6-DOF dynamics. Heave, roll and pitch come from compliant terrain contact, so on slopes the attitude, load transfer and IMU signals are physical. Sprayer tank depletion enters through the block's `dm/dt` input. Multibody is not installed, robotmobilelib models are kinematic only, and the Aerospace 3DOF works in the x-z plane only, so none of these fit. |
| Accelerations | Separate `6DOF Acceleration` and `6DOF Angular Acceleration` blocks (in R2026b the EOM block outputs states only). |
| Sprocket / track belt | **Simscape foundation (classic `foundation.mechanical.*`)**: `Inertia`, `Rotational Friction`, `Wheel and Axle` (r_s = 0.090 m), translational `Mass` (belt), `Ideal Force Source` (ground shear force on the belt). Same classic rotational domain as DRIVE's `Spr_L/Spr_R`. The AB/PB families are not used. |
| Track-terrain shear | **Isotropic 2-D Dahl/Janosi shear per contact point (revision B, D39).** Each of the 14 points carries a 2-vector shear state driven by the resultant slip velocity (belt minus ground, longitudinal and lateral), so turning forces longitudinal slip as in Wong's theory. In steady straight running it reproduces the lumped Janosi-Hanamoto slip curve exactly; at zero speed it is a shear spring with damping (no slip-ratio singularity, no creep on slopes). Derivatives and forces in MF-B, 28 Integrator states in blocks. |
| Normal contact | 2 x 7 contact points (segment-centred along the 0.75 m contact length). Each point is a compliant spring-damper against a DEM terrain looked up with `2-D Lookup Table` blocks. |
| Implements | Variant Subsystem `VC_IMPL` (0 none / 1 mower / 2 tiller / 3 sprayer). Each choice is a physical rotational load on port `Impl_Shaft` plus a chassis wrench. POWER's implement motor drives the shaft (`VC_IMPL_DRIVE == 2`, the default); the internal ideal speed source (`== 1`) is for the harness only. |
| World / terrain | Shared struct **`P.world`**, owned here and built by `params/param_world.m` (`utilities/world_build.m`): DEM, soil grids, vegetation grid, obstacles, rows, boundary, occupancy grid, environment (T_amb, solar), dock. Consumed by STRUCTURE (contact), SENSORS (LiDAR/GNSS) and GNC (a-priori map). |
| Frames | World data in **site ENU** (x East, y North, z Up). Plant integrates in **flat-Earth NED** (Aerospace convention). Body **FRD** inside the domain. The truth bus carries the state at R in site ENU and body FLU, with `pose2D` for GNC (ICD section 1). |
| Fidelity variants | `VC_STRUCT_FID == 1` HiFi (default, above). `== 2` LoFi: `Differential Drive Kinematic Model` (robotmobilelib) plus quasi-static sprocket loads, for long Monte-Carlo GNC runs. |
| MATLAB code | Two MATLAB Function blocks for the physics (contact normal law; planar forces and wrench with the shear derivatives), and two for the mower's gauge wheels (HiFi only, card G2-DSC, D81). No MATLAB System blocks. The obstacle and geofence monitors are in `Sim_Monitors` (D70), not in this domain. |
| Downhill speed limit (card G2-DSC) | The table `S.descent` (§2.11) is the highest speed at which a brakes-locked stop may start, by implement and measured downhill pitch, always on (D74). Its values come from the settle, the park criterion and the hill start (steps 4, 8, 11 and 13 of the descent task), at the 0.75 deg IMU error bound of D83. |
| Solver | No Solver Configuration inside STRUCTURE. The network spans POWER-DRIVE-STRUCTURE and the integrator owns it. Baseline: global fixed-step `ode3` at 1 ms plus Simscape local Backward Euler at 1 ms. STRUCTURE needs h ≤ 2 ms. |

---

## 1. Purpose and scope

### 1.1 In scope (STRUCTURE owns)
1. Chassis rigid body: mass, CG and inertia tensor for every configuration (base, +mower, +tiller, +sprayer with any tank fill). The CG shifts as the tank empties.
2. Track system: sprocket/idler/road-wheel inertia, internal track losses, track belt mass, and conversion from sprocket rotation to belt translation. **`r_sprocket` is owned here (0.090 m [EST], agreed with DRIVE, which copies it as `P.drive.gear.r_spr`).**
3. Track-terrain interaction: normal contact (sinkage, load distribution, lift-off), longitudinal shear traction (slip), lateral shear (skid-steer turning resistance and side-slope holding), soil rolling resistance, grade (through gravity in 6-DOF), soil types.
4. Planar and full 6-DOF body motion on 3-D terrain (x, y, z, roll, pitch, yaw and their rates).
5. Implement mechanical loads: mower blade torque and deck drag; tiller soil torque, horizontal and vertical forces; sprayer pump torque and tank mass flow. Also the actuators for deck height and tiller depth.
6. Enclosure thermal node (solar load, convection, sky radiation) as an optional thermal port for POWER/ELECTRONICS: designed, not built (§2.9).
7. World/terrain model `P.world` and the environment signals `T_amb`, `T_sky`, `T_ground`, `G_solar`, the soil class under R and the dock contact (`Bus_EnvState`).
8. True vehicle state for SENSORS and for GNC evaluation (`Bus_TruthState`, `Bus_TrackState`). The collision and geofence monitors for tests are in `Sim_Monitors` (D70).

### 1.2 Out of scope (owned by neighbours)
| Neighbour | What they own | Interface to STRUCTURE |
|---|---|---|
| **DRIVE** | Traction motors (four, two per track; the 1 kW unit by default, the as-built 750 W selectable, D95), ESCs, gearboxes (`P.drive.gear.N` = 20), brakes, **the reflected motor inertia (`P.drive.J_reflected_spr`: 0.224 kg·m² per side with the 1 kW motors, 0.160 with the 750 W) and the gearbox output inertia (`P.drive.gear.J_out` = 0.008 kg·m²)**. STRUCTURE must not add them again. | Mechanical rotational ports `Spr_L`, `Spr_R` (classic domain). Positive omega = forward track motion on that side. |
| **POWER** | Battery, BMS, 48 V bus, auxiliary loads, the implement motor and its drive | the implement motor on `Impl_Shaft` (`VC_IMPL_DRIVE == 2`); `Bus_ImplMechState` (shaft speed and torque, tank level) to `Power_Plant/Implement_Loads`; the battery mass and position feed P.structure (`cfg.batteryMass_kg` = `P.power.pack.mass_kg`, 18.5 kg [EST]; `cfg.batteryPos_m` = [0.05 0 −0.30] m) |
| **ELECTRONICS / MCU** | Implement controller (deck/tiller lift actuator, nozzle valves, pump), E-stop, display | `Bus_ImplementCmd` in (nozzle valves, lift duty), `Bus_ImplMechState` out (lift potentiometer position, tank level) |
| **SENSORS** | IMU, GNSS, encoders (motor encoders are in DRIVE), LiDAR models, mounting lever arms (SENSORS' own parameters) | `Bus_TruthState`, `Bus_TrackState`, `P.world` (obstacles/terrain for ray casting) |
| **GNC** | Planning, guidance, fusion, missions | `P.world.rows / boundary / occ` (a-priori map), `pose2D` truth (evaluation only, never used for control) |
| **RC** | SBUS link | none |

---

## 2. Physical model

### 2.1 Frames and sign conventions
* **Site frame S (ENU)**: origin at `P.world.lla0` = [24.1556 N, 47.3120 E, 430 m] (Al-Kharj palm district, adjustable). x = East, y = North, z = Up. Terrain, obstacles, rows, maps and the occupancy grid are stored in S.
* **Plant inertial frame n (NED, flat Earth)**: N = y_S, E = x_S, D = −z_S. The 6-DOF block integrates CG position `Xe` and velocity `Ve` in n.
* **Body frame b (FRD)**: x forward, y right, z down. Origin of the 6-DOF states is the CG. The fixed geometric **reference point R** is at ground level (undeformed track bottom), midway between the track centrelines and at mid contact length. All component positions in section 6 are given in body axes relative to R, so z < 0 means above ground.
* Euler angles ZYX [phi theta psi] (NED to body). psi is measured clockwise from North. `yaw_enu = atan2(sin(pi/2 − psi), cos(pi/2 − psi))`.
* Track index i = 1 (left, y = −B/2) and 2 (right, y = +B/2). **Sprocket positive rotation = forward on both sides** (DRIVE handles motor mirroring by ESC polarity).
* Forces in the contact model act **on the vehicle**. F_x,i > 0 is traction pushing the vehicle forward. The belt receives −F_x,i.

### 2.2 Rigid-body dynamics (6-DOF, Simple Variable Mass)
```
m (v̇_b + ω × v_b) = F_b                         (vre_flag off: no thrust term from dm/dt)
J ω̇ + ω × (J ω) + J̇ ω = M_b
ṗ_n = C_nb v_b ,  Euler kinematics (ZYX)
m(t) = m_e + ∫ dm/dt ,   J(m) linear between J_e (tank empty) and J_f (tank full)   [block-internal]
F_b = F_grav + Σ_j F_c,j + F_impl + F_ext ,     F_grav = m C_bn [0 0 g]^T ,  g = 9.7876 m/s² (P.frames.g, Riyadh latitude/altitude)
M_b = Σ_j r_j × F_c,j + r_impl × F_impl + M_impl + M_ext
```
For configurations without a sprayer, `mass_f = mass_e + 1e-3` kg, `inertia_f = inertia_e` and dm/dt = 0. The 6-DOF block assumes the CG is fixed in the body. The tank-induced CG shift (4.4 cm in z and under 1 mm in x with the 40 L tank) is applied to the contact lever arms through r_cg(m) (2.3), which is exact for the forces. The resulting slow drift of the CG state is quasi-static (about 8 min to drain the tank at full flow) and its effect is negligible.

### 2.3 Contact-point kinematics (Simulink blocks)
Per track there are Nc = 7 points, segment-centred along the contact length L:
x_j^R = −L/2 + (j − 0.5)·L/Nc, which gives {±0.3214, ±0.2143, ±0.1071, 0} m for L = 0.75 m. y_j^R = ∓B/2 (∓0.30 m) and z_j^R = 0. This gives M = 14 points.
With uniform pressure, the segment-centred layout gives mean|x| = 0.1837 m (continuous L/4 = 0.1875 m, −2%) and mean x² = 0.0459 m² (continuous L²/12 = 0.0469 m²). The real track carries its load on 3 road wheels per side, so the equal points may overstate the turning moment (§9, open question 6).
```
r_cg(m)  = (m_e r_cg,e + (m − m_e) r_tank) / m              (body, w.r.t. R)
r_j      = r_j^R − r_cg                                      (3×M, w.r.t. CG)
p_j      = Xe·1ᵀ + C_nb r_j                                  (NED)
v_j^b    = v_b·1ᵀ + Ω̂ r_j ,  Ω̂ = [0 −r q; r 0 −p; −q p 0]
v_j^n    = C_nb v_j^b
```

### 2.4 Normal contact (MATLAB Function MF-A `trackContactNormal`)
Terrain is looked up at (E_j, N_j): height h (up, m) and gradients h_E = ∂h/∂E, h_N = ∂h/∂N. Soil properties come from pre-blended grids (linear interpolation gives smooth soil transitions over one 0.25 m cell).
```
n_z    = 1/sqrt(1 + h_E² + h_N²)
pen_j  = (D_j + h_j)·n_z                         (> 0 = below the surface; the ground surface is at D = −h)
ṗen_j  = (v_D,j + h_E v_E,j + h_N v_N,j)·n_z
k_j = k_c·kScale_j ,  c_j = c_c·sqrt(kScale_j)
N_j    = [pen_j > 0] · max(0, k_j pen_j + c_j ṗen_j)
N_i    = Σ_{j∈i} N_j ;  per-track soil values are N-weighted means (plain means if N_i < 1 N)
v_g,i  = mean_{j∈i} (v_j^b)_x                    (ground speed of track i along body x; equal for all points of a rigid track)
```
The normal force is applied along −z_b. The chassis follows the terrain through the contact, so the angle between body z and the terrain normal is small: about 1°, set by static pitch compliance.

### 2.5 Track shear: per-point isotropic 2-D Dahl-relaxation model (revision B, D39)
**Binding law (supersedes the track-level longitudinal state below and the separate lateral state of 2.6).** For point j of track i:
```
v_slip,j = [ v_b,i - v_x,j ;  -v_y,j ]                      (belt minus ground, body axes; v_b,i from the belt Mass)
ds_j/dt  = (v_slip,j - |v_slip,j| s_j)/K_j - (|v_b,i|/l) s_j    s_j = [s_x; s_y], |s_j| <= 1, l = L/2 = 0.375 m
mu_j     = mu_p mu_lat |s_j| / sqrt((mu_lat s_x)^2 + (mu_p s_y)^2)     (friction ellipse along the shear direction)
F_j      = sat_|.|( mu_j N_j [ g(|s_j|) s_j/|s_j| + tau_d ds_j/dt ],  mu_j N_j )      (damping term uses v_b,i delayed 1 ms, D36)
F_x,i    = sum_{j in i} F_x,j  -> belt (Ideal Force Source -F_x,i);   rolling resistance on the body only
```
Straight running (v_y = 0, all points equal) gives exactly the old lumped curve below, so T4 is unchanged. In a turn the lateral
point velocities make |v_slip| large and the isotropic law forces longitudinal slip; the effective gauge factor chi = (v_bR - v_bL)/(B r)
then lies in the 1.3-1.8 range reported for tracked skid-steer vehicles (the old law gave 1.01-1.08). Eigenvalue as before,
-(|v_slip|/K + |v_b|/l) >= -400 1/s.

*Revision A text, retained for the straight-line derivation:*

### 2.5a Longitudinal track shear (revision A, straight-line reference)
Physical idea: Janosi-Hanamoto shear, τ = τ_max(1 − e^{−j/K}). Track elements enter the contact patch at belt speed |v_b| (patch renewal). A normalised shear state s ∈ [−1, 1] is used:
```
v_s,i  = v_b,i − v_g,i                                      (slip velocity, belt minus ground)
ṡ_x,i  = (v_s,i − |v_s,i|·s_x,i)/K_i − (|v_b,i|/ℓ)·s_x,i       ℓ = L/2 = 0.375 m (relaxation length)
F_x,i  = sat( μp,i N_i [ g(s_x,i) + τ_d ṡ_x,i ] ,  ±μp,i N_i )
g(s)   = sgn(s)·f( 2|s|/(1 − |s|) ),   f(x) = 1 − (1 − e^{−x})/x   (lumped Janosi-Hanamoto over the contact length)
```
Properties, all checked analytically:
* **Steady state** (ṡ = 0, driving): s = iℓ/(iℓ + K) with slip i = v_s/v_b. Then g(s) = f(iL/K), which is exactly the lumped Janosi-Hanamoto drawbar curve F = μpN[1 − K/(iL)(1 − e^{−iL/K})]. Braking (|v_b| < |v_g|) gives Wong's skid kinematics j = (v_s/v_b)·x.
* **Standstill** (v_b = v_g = 0): ṡ = 0. The state holds, so a parked robot on a 20° slope has **zero creep**. It has only an elastic deflection K·s ≈ 12 mm on farm soil.
* **Small motion**: F ≈ (μpN/K)·δ + τ_d(μpN/K)·δ̇. This is a Kelvin-Voigt shear spring with k_x = μpN/K (29 kN/m on farm soil and 199 kN/m on asphalt per track with no implement, N = 1106 N; 41 and 278 kN/m with the full-tank sprayer).
* **Full sliding**: s → ±1 after about 3K of sliding (7.5 cm on farm soil). Force reversal needs about 2·3K. s stays bounded in [−1, 1] (Integrator limits as a safety net).
* **No division and no ε regularisation.** Eigenvalue −(|v_s|/K + |v_b|/ℓ), at most about −400 s⁻¹ (spinning at 2 m/s on asphalt), so it is stable with explicit ode3 at h = 1 ms (hλ = −0.4, limit −2.5).
* Coupled belt/body mode on asphalt, no implement: ω ≈ sqrt(k_x/m_belt,eq) = sqrt(199e3/40.3) ≈ 70 rad/s (11 Hz), ζ ≈ 0.35 with τ_d = 0.01 s (m_belt,eq = 8 kg belt + 3.7 kg track inertia J_eq/r_s² + 28.6 kg DRIVE inertia (J_reflected_spr + J_out)/r_s² with the 1 kW motors; 20.7 kg with the 750 W, which gives 77 rad/s). With the full-tank sprayer on asphalt: 83 rad/s (92 rad/s with the 750 W). Farm soil: 27 rad/s.

### 2.6 Rolling resistance and wrench (MATLAB Function MF-B `trackPlanarForces`)
Revision B: the lateral shear state s_y,j is the second component of the 2-D state of 2.5; the friction-ellipse factor eta below is no
longer used (the isotropic law couples the directions). Rolling resistance, the wrench sums and M_turn are unchanged.
Per contact point there is a lateral shear state s_y,j (14 Integrator states). It uses the same Dahl-relaxation form, renewed at the belt speed of its track:
```
η_i    = sqrt( max(η_min², 1 − (F_x,i/(μp,i N_i + 1))²) )           friction-ellipse reduction, η_min = 0.1
ṡ_y,j  = (v_y,j − |v_y,j| s_y,j)/K_j − (|v_b,i|/ℓ) s_y,j
F_y,j  = −sat( μlat,j η_i N_j (g(s_y,j) + τ_d ṡ_y,j), ±μlat,j η_i N_j )
F_rr,j = −f_r,j N_j tanh(v_x,j / v_ε,rr)          v_ε,rr = 0.05 m/s (soil rolling resistance acts on the body; zero at standstill)
F_x,j  = F_x,i · N_j / N_i                       (track traction distributed by normal load)
F_c,j  = [F_x,j + F_rr,j ;  F_y,j ;  −N_j]   (body axes, applied at point j)
F_c    = Σ_j F_c,j ,   M_c = Σ_j r_j × F_c,j ,   M_turn = Σ_j (r_j)_x F_y,j
```
* Pure spot turn: v_y,j = r·x_j, so M_turn → −μlat·η·Σ N_j|x_j| ≈ −μlat W L/4 (classic skid-steer turning-resistance moment, uniform pressure). A longitudinal CG offset or load transfer skews the pressure, which shifts the instantaneous centre of rotation automatically.
* Rolling with side force: s_ss = βℓ/(βℓ + K) with β = v_y/u. The cornering stiffness is μlat N L/(2K) per track, the same shear modulus as longitudinally (isotropic shear).
* Side-slope parking: zero creep (state holds).
* Turning resistance that depends on radius (Wong's empirical μt = μmax/(0.925 + 0.15R/B)) is **not** added. The point model produces the ICR shift physically. See open question Q6.

### 2.7 Sprocket / belt (Simscape network per side, file `structure_track_side.slx`)
```
J_eq ω̇_s = T_Spr − T_fric(ω_s) − r_s F_axle            Inertia + Rotational Friction (internal track loss)
m_belt v̇_b = F_axle − F_x                             Wheel and Axle (v_b = r_s ω_s) + Mass + Ideal Force Source (−F_x)
```
* T_fric (Rotational Friction): Coulomb T_c = f_c·N_nom·r_s (f_c = 0.015), breakaway 1.3·T_c at 0.05 rad/s, viscous 0.005·N_nom·r_s² (rubber-track bending, road-wheel bearings, idler). N_nom is the static load per track of the active configuration, computed in `param_structure` (`P.structure.N_nom`: 1106 N with no implement and the 1 kW motors, 1077 N with the 750 W, 1424 N with the mower, 1522 N with the tiller, 1546 N with the full-tank sprayer).
* The mechanical reference of this network is the chassis (body frame). The belt mass appears here only as circulating inertia (½ m_belt v_b²). Its translational mass is part of the rigid-body mass. Momentum of the top and bottom belt runs cancels.

### 2.8 Implement loads (section 3.6 blocks)
**Mower (front out-front deck, 2 counter-rotating blades lumped on one equivalent shaft):**
```
T_load = k_aero ω|ω| + D_b ω + T_cut ,   T_cut = e_A·w·max(v_fwd,0)·ρ_veg / max(|ω|, 0.3 ω_nom) · sgn(ω)
F_impl = [ −(f_gw λ_gw m_deck g + c_veg ρ_veg w)·tanh(v_fwd/0.05) ;  0 ;  0 ]  at r_deck   (the gauge wheels' normal load is not in this wrench: the two caster gauge wheels are a HiFi contact, §3.3 and §6.5; their rolling drag stays here)
M_impl,z = k_yaw·T_cut   (k_yaw = 0 for counter-rotating blades)
```
Nominal: ω = 293 rad/s (2800 rpm), idle 300 W, cutting 800 J/m² × 1.0 m × 1 m/s = 800 W, which gives **P_shaft = 1.1 kW**. Dense weeds (ρ_veg = 2.5) give 2.3 kW.

**Tiller (rear rotary, down-cut):**
```
d = depth from actuator (Rate Limiter 0.03 m/s + 1st order τ = 0.3 s, 0…0.12 m)
P_soil = e_V·w·d·max(v_fwd,0) ;  T_soil = P_soil/max(|ω|, 0.3 ω_nom)·(1 + a_r sin(n_t θ_rotor)) + T_idle·[d>0]
F_impl = [ κ_push T_soil/R_rot − μ_skid λ_skid m_till g ;  0 ;  −κ_lift T_soil/R_rot − λ_skid m_till g ]  at r_rotor = [−0.95 0 −(R_rot − d)]
```
Nominal: 0.6 m × 0.08 m × 0.3 m/s × 80 kJ/m³ = 1152 W, plus idle 105 W (T_idle 4 N·m at 26.18 rad/s, 250 rpm), which gives **P_shaft = 1.26 kW**, T_rotor = 48 N·m at 26.18 rad/s.

**Sprayer (tank on top deck, 8-nozzle boom, diaphragm pump):**
```
p = p_set·[pumpEnable ∧ V_tank > 0] ;  Q = Σ_k duty_k·Q_ref·sqrt(p/p_ref) ;  V̇_tank = −Q  (Integrator, limits [0, V_max])
dm/dt = −ρ_liq Q  → 6-DOF ;  T_pump = D_p p/η_m  on Impl_Shaft
```
Nominal: 8 × 0.6 L/min = 4.8 L/min at 3 bar. The 40 L tank lasts 8.3 min at full duty and about 17 min with vision-guided 50% duty. Pump torque is 0.19 N·m (59 W at 314 rad/s).

### 2.9 Enclosure thermal (optional, `structure_enclosure_thermal.slx`)
**Not built.** This block is designed but was never built: `structure_plant` has no `H_encl` port and P has no `S.thermal`. POWER and ELECTRONICS model their own heat, against `Bus_EnvState`. The design below is kept for a later card.
```
C_encl Ṫ = Q_port + α_s A_s G_solar − h(v) A_c (T − T_amb) − εσ A_r (T⁴ − T_sky⁴) ,   h(v) = 5.7 + 3.8·|v_fwd|  W/(m²K)
```
Peak solar input is 0.65 × 1.6 m² × 1000 W/m² ≈ 1.0 kW, which dominates electronics losses in Saudi summer. The thermal port `H_encl` is the enclosure air/structure node.

### 2.10 LoFi variant (`VC_STRUCT_FID == 2`)
* Per side (Simscape): `Spr_*` → `Inertia` (J_eq + m_belt r_s² + (m/2) r_s²) → `Rotational Friction` → `Ideal Torque Source` (load T_L,i = r_s F_L,i) → `Ideal Rotational Motion Sensor` → ω_i.
* `Differential Drive Kinematic Model`: `VehicleInputs='Wheel Speeds'`, `WheelRadius = r_s(1 − i_nom)`, `TrackWidth = χ·B` (χ = `P.structure.track.chi_nom` = 1.3936, the effective skid-steer gauge identified from HiFi, D39), `InitialState = [x_E0; y_N0; yaw_enu0]`, `SimulateUsing='Code generation'`.
* Quasi-static loads (blocks): F_L,i = f_r N_i + (m/2) g sin θ_long ∓ (μlat W L/(4B))·tanh(r_enu/0.05), where the upper sign is for the left (inner on a left turn) track. θ_long and φ_lat come from DEM gradients at R projected on the heading. N_i = (W/2)cos θ_long ∓ W·h_cg·sin φ_lat / B.
* The same `Bus_TruthState` is built from the planar state: v_b = [v;0;0], ω = [0; 0; −ψ̇_enu], quasi-static attitude, specific force = [v̇; v·r; 0] − C_bn[0;0;g].

---

### 2.11 Downhill speed limit (card G2-DSC; D74, D75, D81)

The limit is the highest speed at which a brakes-locked stop may start without tipping the robot over its front contact (forward) or its rear contact (reverse). It is the table `S.descent` (`params/param_structure.m`), read by the MCU and the GNC. The MCU applies it in every mode, RC and the hardware E-stop included.

**Envelope (CR-001, D75).** The 20 deg design slope holds with no implement, the mower (deck down) and the tiller forward. The full-tank sprayer descends slopes of up to 18.5 deg only. Traction limits a climb as well: the steady slip reaches about 30 % at 18 deg on wet loam, 24 deg on dry sand, 28 deg on farm soil and 38 deg on concrete (structure descent step 2, `work/agentic/descent/structure/report_part2.md`); a 20 deg climb on farm soil runs at about 9 % slip (T7 with the full-tank sprayer at 0.5 m/s: 9.31 % measured, 9.10 % analytic; S2 with no implement: 0.725 m/s at a 0.8 m/s command). The cross-slope limit is the GNC's roll governor (`P.gnc.spd.roll_x` / `roll_v`: 0.3 m/s at 15 deg, 0 at 18 deg); T8 holds 15 deg of side slope parked and T11 (not automated) traverses it.

**Why.** In the system run S2c (regen blocked, CCL 0, full-tank sprayer 316 kg, a brakes-locked stop from 1.175 m/s on 15 deg) the robot pitched over its front contact. The pitch-over is physical for the deck-mounted full tank; the motion after the flip is not, because the hull has no contact with the ground in the model.

**Axis.** The measured downhill body pitch in the direction of travel, peak-held over 1 s: `fw_imu_tilt` pitch times the sign of the speed in the MCU, and the nav pitch or its look-ahead in the GNC. The body pitch relative to the terrain is the settle (below), with the leading end down positive; a measured pitch therefore reads high by the settle.

**Rule.** For each pre-stop state (coasting with no motor torque; braked or brakes held; the hill start), geometry (model; sharp-edge track end) and soil (farm; concrete), the entry is the lowest over all scenarios of:
- the steepest terrain of the bin, θ_eff = max(b, θ such that θ + settle(θ) − 0.75 deg = b), where b is the breakpoint and 0.75 deg is the IMU error bound (D83, the owner, 2026-11-16; 0.5 deg before structure step 13), taken as reading low;
- the planar safe start speed at θ_eff, min(v_tip,model / 1.3, v_tip,sharp) − dv(θ), rounded down to 0.05 m/s, with dv(θ) = 0.05 + g(sin θ − 0.035 cos θ) · 0.05 s (the speed gained before the brakes engage);
- an entry at or above 1.44 m/s (the MCU trip of a top-speed entry) gives the top speed, 1.41 m/s, which is no cap.

The rows that found no tip up to 3 m/s keep their earlier values (the mower and the tiller forward, and the raised mower in reverse: the 2 m/s test ceiling). The planar model uses the HiFi contact and shear laws; the contact stiffness and damping, the track-end shape and the tiller data are estimates [EST].

**Table `S.descent`, m/s.** Breakpoints θ_bp = [0 5 10 15 20 20.5 21 21.5 22 23 24 25] deg. A value applies from the previous breakpoint up to its own (the steeper breakpoint): a pitch of 19.9 deg takes the 20 deg value (0.25 m/s for the sprayer forward), not the 15 deg one. The entries are at the 0.75 deg IMU error bound (D83; 0.5 deg before structure step 13).

| Row | 0 | 5 | 10 | 15 | 20 | 20.5 | 21 | 21.5 | 22 | 23 | 24 | 25 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| none, forward | 1.41 | 1.41 | 1.41 | 1.41 | 1.35 | 1.30 | 1.25 | 1.20 | 1.15 | 1.05 | 0.95 | 0 |
| mower (deck down), forward | 1.41 | 1.41 | 1.40 | 1.35 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 0 |
| tiller, forward | 1.41 | 1.41 | 1.40 | 1.35 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 0 |
| sprayer (full tank), forward | 1.41 | 1.40 | 1.05 | 0.70 | 0.25 | 0.20 | 0.15 | 0.05 | 0 | 0 | 0 | 0 |
| none, reverse | 1.41 | 1.41 | 1.41 | 1.41 | 1.41 | 1.41 | 1.41 | 1.41 | 1.41 | 1.35 | 1.25 | 0 |
| mower (deck down), reverse | 1.41 | 1.41 | 1.40 | 1.35 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 0 |
| tiller, reverse (no reversing, D74) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| sprayer (full tank), reverse | 1.41 | 1.35 | 1.00 | 0.65 | 0.20 | 0.15 | 0.10 | 0 | 0 | 0 | 0 | 0 |
| mower, deck raised, forward (`v_fwd_raised`) | 0.15 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| mower, deck raised, reverse (`v_rev_raised`) | 1.41 | 1.41 | 1.40 | 1.35 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 1.30 | 0 |

The MCU's speed target is `ref` = max(cap − 0.10 m/s, 0), and its trip is the cap, floored at 0.05 m/s; an entry at the top speed trips at 1.44 m/s and has no cap (`P.electronics.lim.descent`, electronics spec §6). The implement is the DI IMPL_ID coding plug; an unknown implement takes the smallest entry of all rows. The GNC plans and commands within `ref` (cap − 0.10 m/s) and reads the same table at the nav pitch (GNC spec §2.5).

**Settle: the body pitch relative to the terrain, deg** (planar model with the HiFi geometry; leading end down positive). Farm soil; concrete in brackets. "Coasting" means no motor torque; "braked" means a steady braked descent, which equals standing with the brakes held. The measured pitch reads the terrain plus the settle ± 0.75 deg (D83).

| Load, direction | coasting, 0 to 25 deg | braked, 0 deg | 10 deg | 20 deg | 22 deg | 25 deg |
|---|---|---|---|---|---|---|
| none, forward | +0.20 (+0.10) | +0.06 | +0.43 | +0.78 (+0.62) | +0.85 | +0.95 |
| none, reverse | +0.09 (+0.01) | −0.06 | +0.31 | +0.67 (+0.53) | +0.74 | +0.84 |
| mower, deck down, forward | +0.17 (+0.12) | +0.13 | +0.22 | +0.30 (+0.24) | +0.32 | +0.34 |
| mower, deck down, reverse | −0.09 (−0.09) | −0.15 | −0.05 | +0.04 (+0.03) | +0.06 | +0.09 |
| mower, deck raised, forward | +4.0 to +4.4 (+1.9 to +2.0) | +3.40 | tips over (+5.4 at 5 deg) | tips over | tips over | tips over |
| mower, deck raised, reverse | −2.3 to −2.5 (−1.6 to −1.7) | −3.44 | −1.82 | −1.11 (−0.87) | −0.99 | −0.81 |
| tiller raised, forward | −4.0 to −4.4 (−2.1 to −2.3) | falls backward | −3.04 | −1.57 (−1.18) | −1.34 | −1.14 |
| tiller raised, reverse | +8.0 to +8.8 (+2.5 to +2.7) | falls over backward at every slope | | | | |
| sprayer (full tank), forward | +0.24 to +0.27 (+0.07) | −0.03 | +0.73 | +1.46 (+1.15) | +1.72 | +2.27 |
| sprayer (full tank), reverse | +0.29 to +0.33 (+0.10) | +0.03 | +0.78 | +1.55 (+1.21) | +1.81 | +2.50 |

The sharp-edge geometry gives somewhat larger magnitudes (−6.3 deg for the raised tiller coasting forward); the table uses each geometry's own settle. HiFi checks: the sprayer braked at 19.4 to 20 deg gives +1.6 to +1.7 in S2d; the mower on the flat gives +0.83 in T3, against a planar +0.13 to +0.18 (the planar gauge wheel has no preload).

**Park criterion** (a brakes-locked stop that starts from a trip counts as safe when both hold):
- **(a) speed:** v_park ≤ V_lin(p_trip) + Δv(θ_R). v_park is the largest |speed| from the trip until both brakes engage; p_trip is the MCU's held downhill pitch at the trip; V_lin is the run's row and direction interpolated linearly in θ_bp at p_trip (0 at 25 deg and above); Δv(θ_R) = 0.05 + g(sin θ_R − 0.035 cos θ_R) · 0.05 s, at the terrain slope under the robot's reference point R at the trip.
- **(b) pitch relative to the terrain:** the peak from the trip until 2 s after the brakes engage is at most ½ (atan(d/h) − θ_R,max), where θ_R,max is the steepest downhill terrain under R in that same window (from the trip to 2 s after the brakes engage). The check in `utilities/scn_descent_parks.m` uses it, which is stricter than the terrain at the trip; the speed term (a) still uses the terrain at the trip. For the full-tank sprayer, d = 0.324 m forward and 0.319 m in reverse, h = 0.375 m; the static tip angles are 40.8° forward and 40.4° in reverse, so the limits are 15.4 / 12.9 / 10.4 deg at 10 / 15 / 20 deg of terrain.

Linear interpolation is safe because the safe start speed is concave in pitch (the bounds fall by 0.051 to 0.063 m/s per 0.5 deg from 20 to 22.5 deg); at a bin edge V_lin equals the entry of the edge just crossed. The check is `utilities/scn_descent_parks.m` in the integration runs.

**Hill start (step 11).** The hill start is a third pre-stop state: the robot stands on its brakes (the braked settle), the brakes release with no motor torque, the robot rolls back from rest, and the brakes lock when the true speed reaches v_park. Its planar tip speed is about the coasting value, and up to 8 % lower with the sharp-edge track end. The table covers it: a robot standing on its brakes reads its braked settle, 0.5 to 1.2 deg more than the coasting robot in the reverse sprayer bins, so its terrain is shallower than the bin assumes. The MCU's hill-start window (`t_release` 0.5 s [EST], D74) is safe in every case asked:
- the full-tank sprayer in reverse on 20.2 deg of terrain, parking from 0.263 m/s: a margin of +0.141 m/s to the safe start speed;
- at the 18.5 deg edge of CR-001, +0.19 m/s, even at the upper bound of the roll-back speed (0.389 m/s);
- no implement, and the mower, at 20 deg: +0.40 m/s or more at their own roll-back speeds;
- the tightest case in the envelope is the 10 to 15 deg reverse bin on a climb of 14.87 deg (14.62 deg before step 13): +0.085 m/s at 0.5 deg of IMU error, about +0.06 m/s at 0.75 deg (D83), both at the dv bound of the roll-back speed (interpolated from the step-13 bounds, not simulated).

No case is unsafe at the 0.75 deg bound (structure step 13, D83), so `t_release` can stay at 0.5 s; a shorter window below about 0.1 s would bring back the S2 false trip. The rule for any later change: trip + a(θ) × (trip-to-brake time) ≤ the largest safe park speed, where θ is the steepest terrain at which a robot standing on its brakes reads the bin and a = g(sin θ − f_rr cos θ) is the roll-back acceleration with no motor torque.

**Mower deck and stop-go.** The mower deck lifts. Raised to 0.09 m (the gauge wheels off the ground) the robot is nose-heavy, and a locked stop tips it from about 0.23 m/s on the flat; hence the `*_raised` rows, which the MCU applies as a backstop when the deck is reported up while moving (D81). **In the twin this backstop does not work yet:** the structure and the MCU read the lift position in opposite senses (§5.4), so the raised rows apply with the deck low and not with it raised. One sense end to end is pending: the plant follows the sensor and the MCU, a height fraction with 0 at the bottom (the lead systems engineer's ruling of step 19, point 22); the twin changes on card G2-STR, and card G2-INT checks that a raised deck selects the raised rows. Without regeneration (CCL 0) the robot cannot slow as the slope steepens, so the descent is stop-go: every brakes-locked stop meets the park criterion: it starts within V_lin(held pitch) + Δv and meets the pitch bound (b). At a breakpoint crossing the step cap can fall below the speed the robot already has, so a park can start near the entry of the bin just crossed. Example: S2c park 2 (sprayer, CCL 0) at 0.787 m/s and a held 15.2 deg, against a 0.25 m/s cap in that class; margins 0.056 m/s and 2.1 deg. Each OVERSPEED park latches (class B) and needs an operator reset and re-arm. The owner chose to keep the latch (2026-11-15).

---

## 3. Block list (verified in the live R2026b session)

Library paths are given with spaces. The true names of several foundation and nesl blocks contain line breaks. `get_param`/`add_block` accept the space form (verified for every row marked ✔). **Exception:** the DCM-to-quaternion block's true name contains a double space (`'aerolibtransform2/Direction Cosine Matrix  to Quaternions'`).

### 3.1 Track side network (`structure_track_side.slx`, Simscape, instantiated twice)
| Block | Library path | ✔ | Key mask parameters (real names) |
|---|---|---|---|
| Connection Port `Spr` | `nesl_utility/Connection Port` | ✔ | Side = Left, port 1 |
| Inertia (sprocket, idler, road wheels) | `fl_lib/Mechanical/Rotational Elements/Inertia` | ✔ | `inertia=P.structure.track.J_eq`, `num_ports='foundation.enum.numPorts2.one'`, `w_specify='on'`, `w=P.structure.init.v0/P.structure.r_sprocket` |
| Rotational Friction (internal track loss) | `fl_lib/Mechanical/Rotational Elements/Rotational Friction` | ✔ | `brkwy_trq=P.structure.track.T_brk`, `brkwy_vel=P.structure.track.w_brk`, `Col_trq=P.structure.track.T_col`, `visc_coef=P.structure.track.c_visc` |
| Mechanical Rotational Reference | `fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference` | ✔ | (chassis frame) |
| Wheel and Axle (sprocket to belt) | `fl_lib/Mechanical/Mechanisms/Wheel and Axle` | ✔ | `R=P.structure.r_sprocket`, `or='foundation.enum.DriveOrientation.Positive'`. Ports A (rotational), P (translational) |
| Mass (track belt) | `fl_lib/Mechanical/Translational Elements/Mass` | ✔ | `mass=P.structure.track.m_belt`, `num_ports='foundation.enum.numPorts2.one'`, `v_specify='on'`, `v=P.structure.init.v0` |
| Mechanical Translational Reference | `fl_lib/Mechanical/Translational Elements/Mechanical Translational Reference` | ✔ | |
| Ideal Force Source (shear on belt) | `fl_lib/Mechanical/Mechanical Sources/Ideal Force Source` | ✔ | R → belt node, C → translational reference; S = −F_x |
| Ideal Translational Motion Sensor (belt speed) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Translational Motion Sensor` | ✔ | `reference='foundation.enum.MeasurementReference.difference'`, `velocity_port='true'`, `position_port='false'` |
| Ideal Rotational Motion Sensor (sprocket truth) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor` | ✔ | `reference='...difference'`, `velocity_port='true'`, `position_port='true'`, `wrap_angle='false'` |
| Ideal Torque Sensor (sprocket torque truth) | `fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor` | ✔ | in series between `Spr` and the Inertia |
| Simulink-PS Converter (F_x) | `nesl_utility/Simulink-PS Converter` | ✔ | `Unit='N'`, `FilteringAndDerivatives='zero'` (force source needs no derivative) |
| PS-Simulink Converter ×4 | `nesl_utility/PS-Simulink Converter` | ✔ | `Unit='m/s'` (v_b), `'rad'`, `'rad/s'`, `'N*m'` |

### 3.2 Shear integration (`structure_track_shear.slx`)
Revision B: `simulink/Continuous/Integrator` with 28 states (IC 0, limits +/-1), input ds/dt [28x1] from MF-B, plus `simulink/Math Operations/Sum of Elements`
per track for F_x,i and a `simulink/Discrete/Unit Delay` (1 ms) on v_b for the damping term. The revision-A track-level block list below is retired.

#### 3.2a Longitudinal shear law (revision A, retired)
| Block | Library path | ✔ | Settings |
|---|---|---|---|
| Subtract (v_s) | `simulink/Math Operations/Subtract` | ✔ | v_b − v_g |
| Abs ×2 | `simulink/Math Operations/Abs` | ✔ | |v_s|, |v_b|; `ZeroCross='off'` |
| Product ×5 | `simulink/Math Operations/Product` | ✔ | element-wise (`Multiplication='Element-wise(.*)'`) |
| Divide (÷K) | `simulink/Math Operations/Divide` | ✔ | |
| Gain (1/ℓ), Gain (τ_d) | `simulink/Math Operations/Gain` | ✔ | `1/P.structure.track.ell`, `P.structure.track.tau_d` |
| Integrator (s_x) | `simulink/Continuous/Integrator` | ✔ | `InitialCondition=[0;0]`, `LimitOutput='on'`, `UpperSaturationLimit=1`, `LowerSaturationLimit=-1` |
| 1-D Lookup Table g(s) | `simulink/Lookup Tables/1-D Lookup Table` | ✔ | `BreakpointsForDimension1=P.structure.track.gS`, `Table=P.structure.track.gG`, `InterpMethod='Linear point-slope'`, `ExtrapMethod='Clip'`, `IndexSearchMethod='Binary search'` |
| Add | `simulink/Math Operations/Add` | ✔ | g + τ_d ṡ |
| Unary Minus | `simulink/Math Operations/Unary Minus` | ✔ | lower bound |
| Saturation Dynamic | `simulink/Discontinuities/Saturation Dynamic` | ✔ | up = μpN, lo = −μpN |

### 3.3 Contact (`structure_contact.slx`)
| Block | Library path | ✔ | Settings |
|---|---|---|---|
| Math Function (transpose) | `simulink/Math Operations/Math Function` | ✔ | `Operator='transpose'`: C_nb = DCM_beᵀ |
| Product (matrix) ×5 | `simulink/Math Operations/Product` | ✔ | `Multiplication='Matrix(*)'`: C_nb·r_j, Xe·1ᵀ, v_b·1ᵀ, Ω̂·r_j, C_nb·v_j^b |
| Create 3x3 Matrix | `aerolibutil/Create 3x3 Matrix` | ✔ | Ω̂ from Demux(ω), Unary Minus, Constant 0 |
| Demux, Constant, Add, Subtract | `simulink/Signal Routing/Demux`, `simulink/Sources/Constant`, `.../Add`, `.../Subtract` | ✔ | Constant `ones(1,14)`, `P.structure.contact.Rc_R` (3×14) |
| Selector ×6 | `simulink/Signal Routing/Selector` | ✔ | `NumberOfDimensions=2`, rows N, E, D of p_j and of v_j^n |
| 2-D Lookup Table ×9 | `simulink/Lookup Tables/2-D Lookup Table` | ✔ | u1 = E (1×14), u2 = N (1×14); `BreakpointsForDimension1=P.world.terrain.xE`, `BreakpointsForDimension2=P.world.terrain.yN`, `Table`= H, dHdE, dHdN, muPeak, K, muLat, fRoll, kScale, veg (veg is evaluated at the deck point only); `InterpMethod='Linear point-slope'`, `ExtrapMethod='Clip'`, **`IndexSearchMethod='Evenly spaced points'`** (O(1) lookup), `DiagnosticForOutOfRangeInput='None'` (the out-of-map flag is in MF-C) |
| MATLAB Function MF-A `trackContactNormal` | `simulink/User-Defined Functions/MATLAB Function` | ✔ | section 4.1 |
| MATLAB Function MF-B `trackPlanarForces` | `simulink/User-Defined Functions/MATLAB Function` | ✔ | section 4.2 |
| Integrator (s_y, 14 states) | `simulink/Continuous/Integrator` | ✔ | IC `zeros(14,1)`, limits ±1 |
| Bias / Gain / Divide (r_cg(m)) | `simulink/Math Operations/Bias`, `.../Gain`, `.../Divide` | ✔ | r_cg = (m_e r_e + (m − m_e) r_tank)/m |
| Gauge wheels (mower, HiFi only; card G2-DSC, D81) | two MATLAB Function blocks in `structure_contact`, calling `subsystems/structure/structure_gauge_points.m` and `structure_gauge_contact.m` | ✔ | two caster gauge wheels of the rigid deck (`P.structure.contact.gw`): normal force only, using the track points' contact law without the soil scale; the force goes into the contact wrench and the power into `P_diss`; LoFi unchanged |

### 3.4 Body dynamics (`structure_body6dof.slx`)
| Block | Library path | ✔ | Key parameters |
|---|---|---|---|
| Simple Variable Mass 6DOF (Euler Angles) | `aerolib6dof2/Simple Variable Mass 6DOF (Euler Angles)` | ✔ (BlockType `EOM6DOFBodyEuler`) | `units='Metric (MKS)'`, `mtype='Simple Variable'`, `xme_0=P.structure.init.xme0`, `Vm_0=P.structure.init.Vb0`, `eul_0=P.structure.init.eul0`, `pm_0=[0 0 0]`, `mass_0=P.structure.mp.m0`, `mass_e=P.structure.mp.me`, `mass_f=P.structure.mp.mf`, `inertia_e=P.structure.mp.Je`, `inertia_f=P.structure.mp.Jf`, `vre_flag='off'`, `mass_flag='on'` (exposes mass/inertia for the acceleration blocks), `zeroCrossing='off'`, state names `{'U','v','w'}`, `{'Xe','Ye','Ze'}`, `{'p','q','r'}`, `{'phi','theta','psi'}`, `'mass'`. Inputs: F_xyz, M_xyz, dm/dt. Outputs (library copy, mass_flag off, 7 ports): Ve, Xe, φθψ, DCM_be, Vb, ωb, tank status; **mass_flag='on' adds mass-property ports. Confirm their order from the block icon at build time.** |
| 6DOF Acceleration | `aerolib6dof2/6DOF Acceleration` | ✔ (masked, In: Vb, omega, mass, Forces; Out: Abb, Abe) | `units='Metric (MKS)'`, `mtype='Simple Variable'`, `vre_flag='off'`. Abe = inertial acceleration in body axes |
| 6DOF Angular Acceleration | `aerolib6dof2/6DOF Angular Acceleration` | ✔ (In: omega, I, Moment; Out: omegaDot) | `units='Metric (MKS)'`, `mtype='Simple Variable'` (if this adds a dI/dt input, feed it from the 6-DOF mass-property outputs) |
| Product (matrix), Constant [0;0;g] | `simulink/Math Operations/Product`, `simulink/Sources/Constant` | ✔ | F_grav = m·DCM_be·[0;0;g] |
| Add (wrench sum) ×2 | `simulink/Math Operations/Add` | ✔ | F: contact + gravity + implement + external; M likewise |
| 3x3 Cross Product | `aerolibutil/3x3 Cross Product` | ✔ (In A, B; Out C) | r_impl × F_impl |

### 3.5 Truth outputs (`structure_truth.slx`)
| Block | Library path | ✔ | Settings |
|---|---|---|---|
| Direction Cosine Matrix to Quaternions | `aerolibtransform2/Direction Cosine Matrix  to Quaternions` (double space) | ✔ | quat_be (scalar first) |
| Flat Earth to LLA | `aerolibtransform2/Flat Earth to LLA` | ✔ (BlockType `FlatEarth2LLA`, In: Xe, href; Out: μl, h) | `units='Metric (MKS)'`, `ptype='Earth (WGS84)'`, `LL0=P.world.lla0(1:2)`, `psi=0`, `refPosPort='off'`; href input = P.world.lla0(3) |
| Trigonometric Function (sin, cos, atan2) | `simulink/Math Operations/Trigonometric Function` | ✔ | yaw_enu wrap |
| Subtract, Product (matrix), Selector, Mux | standard `simulink/...` | ✔ | specific force = Abe − DCM·[0;0;g]; p_R = Xe − C_nb r_cg |
| Bus Creator ×5 | `simulink/Signal Routing/Bus Creator` | ✔ | `OutDataTypeStr='Bus: Bus_TruthState'` etc. |

### 3.6 Implements (`structure_implement.slx` and choice files)
| Block | Library path | ✔ | Use / settings |
|---|---|---|---|
| Variant Subsystem ×2 | `simulink/Ports & Subsystems/Variant Subsystem` | ✔ | `VariantControlMode='expression'`; choices `VC_IMPL==0/1/2/3`; drive choices `VC_IMPL_DRIVE==1/2`; **`VariantActivationTime='update diagram'`** (needed with physical ports) |
| Subsystem Reference | `simulink/Ports & Subsystems/Subsystem Reference` | ✔ | `ReferencedSubsystem='structure_impl_mower'` etc. |
| Connection Port `Impl_Shaft` | `nesl_utility/Connection Port` | ✔ | rotational |
| Inertia (blade / rotor / pump) | `fl_lib/Mechanical/Rotational Elements/Inertia` | ✔ | `inertia=P.structure.impl.<x>.J` |
| Rotational Damper (bearings) | `fl_lib/Mechanical/Rotational Elements/Rotational Damper` | ✔ | `D=P.structure.impl.<x>.D` |
| Ideal Torque Source (load torque) | `fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source` | ✔ | S = −T_load (opposes rotation) |
| Ideal Angular Velocity Source (internal ideal drive, `VC_IMPL_DRIVE==1`) | `fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source` | ✔ | driven by the ramped `speedCmd` via S-PS `FilteringAndDerivatives='filter'`, `InputFilterTimeConstant=0.05` |
| Ideal Rotational Motion Sensor, Ideal Torque Sensor | as in 3.1 | ✔ | shaft speed, angle (tiller ripple), torque |
| Rate Limiter | `simulink/Discontinuities/Rate Limiter` | ✔ | deck ±0.02 m/s, tiller ±0.03 m/s, speed ramp ±100 rad/s² |
| Transfer Fcn | `simulink/Continuous/Transfer Fcn` | ✔ | actuator lag 1/(τs+1) |
| Saturation | `simulink/Discontinuities/Saturation` | ✔ | height/depth limits |
| Integrator (tank volume) | `simulink/Continuous/Integrator` | ✔ | `LimitOutput='on'`, limits [0, V_max], IC `fill0·V_max` |
| Sqrt, Sum of Elements, Product, Gain, MinMax, Trigonometric Function, Compare To Constant | standard `simulink/...` | ✔ | flows, cutting and soil torque, ripple sin(n_t θ) |
| 2-D Lookup Table (veg density at deck) | as in 3.3 | ✔ | `Table=P.world.veg.rho` |

### 3.7 Enclosure thermal (`structure_enclosure_thermal.slx`)
Not built (§2.9); the block list is kept with the design.

| Block | Library path | ✔ | Parameters |
|---|---|---|---|
| Thermal Mass | `fl_lib/Thermal/Thermal Elements/Thermal Mass` | ✔ | `mass=P.structure.thermal.m_eq`, `sp_heat=P.structure.thermal.cp_eq`, `num_ports='foundation.enum.numPorts2.one'`, `T` IC = T_amb0 |
| Convective Heat Transfer | `fl_lib/Thermal/Thermal Elements/Convective Heat Transfer` | ✔ | `thermal_type='foundation.enum.constant_variable.variable'` (h from PS input), `area=P.structure.thermal.A_conv`, `min_heat_tr_coeff=5.7` |
| Radiative Heat Transfer | `fl_lib/Thermal/Thermal Elements/Radiative Heat Transfer` | ✔ | `area=P.structure.thermal.A_rad`, `rad_tr_coeff=0.9*5.670374e-8` |
| Controlled Heat Flow Rate Source (solar) | `fl_lib/Thermal/Thermal Sources/Controlled Heat Flow Rate Source` | ✔ | S = α_s A_s G_solar |
| Controlled Temperature Source ×2 (T_amb, T_sky) | `fl_lib/Thermal/Thermal Sources/Controlled Temperature Source` | ✔ | |
| Thermal Reference, Temperature Sensor | `fl_lib/Thermal/Thermal Elements/Thermal Reference`, `fl_lib/Thermal/Thermal Sensors/Temperature Sensor` | ✔ | |
| Connection Port `H_encl` | `nesl_utility/Connection Port` | ✔ | thermal |

### 3.8 LoFi variant (`structure_lofi.slx`)
| Block | Library path | ✔ | Parameters |
|---|---|---|---|
| Differential Drive Kinematic Model | `robotmobilelib/Differential Drive Kinematic Model` | ✔ | `VehicleInputs='Wheel Speeds'`, `WheelRadius=P.structure.r_sprocket*(1-P.structure.lofi.i_nom)`, `TrackWidth=P.structure.lofi.chi*P.structure.geom.B`, `WheelSpeedRange=[-inf inf]`, `InitialState=P.structure.init.pose2D0(:)`, **`SimulateUsing='Code generation'`** (the default 'Interpreted execution' is slow) |
| Inertia, Rotational Friction, Ideal Torque Source, Ideal Rotational Motion Sensor, S-PS/PS-S | as in 3.1 | ✔ | |
| 2-D Lookup Table ×3 (h, dHdE, dHdN at R) | as in 3.3 | ✔ | |

### 3.9 Shared / harness
| Block | Library path | ✔ | Note |
|---|---|---|---|
| Solver Configuration | `nesl_utility/Solver Configuration` | ✔ | **Harness only.** `UseLocalSolver='on'`, `LocalSolverChoice='NE_BACKWARD_EULER_ADVANCER'`, `LocalSolverSampleTime=1e-3`, `DoFixedCost='on'`, `MaxNonlinIter=3` |
| Ideal Angular Velocity Source / Ideal Torque Source | `fl_lib/Mechanical/Mechanical Sources/...` | ✔ | harness sprocket drivers |
| Mechanical Rotational Reference | as above | ✔ | harness |

Libraries checked and **not used**: `fl_lib/Rotational/*` (AB) and `fl_lib/Translational/*` (PB). These are a different domain from the ee_lib motors and DRIVE's classic ports. `aerolib3dof` (x-z plane only). Simscape Multibody / Driveline / Vehicle Dynamics are not installed.

---

## 4. MATLAB Function blocks (only where no block exists)

The blocks call path-visible helper files in `subsystems/structure/`: `structure_contact_normal.m` (MF-A), `structure_planar_forces.m` (MF-B), and for the mower's gauge wheels `structure_gauge_points.m` and `structure_gauge_contact.m`. That way the same code is unit-tested in `tests/structure/t_structure.m`, which also checks that each block holds the current copy (T03). Parameters are passed as non-tunable struct parameters (`P.structure.contact`, `P.structure.track`). The code is codegen-compatible (needed for Accelerator / Rapid Accelerator).

### 4.1 MF-A `trackContactNormal` (continuous, inherited sample time)
**Why no block:** compliant terrain contact on 14 points with gating, slope-normal projection and per-track load-weighted soil blending has no Simulink/Simscape block. Spatial Contact Force exists only in Simscape Multibody, which is not installed. Everything around MF-A (kinematics, terrain lookups, integrators, shear law) uses blocks.
```matlab
function [N_pt, N_trk, muP_trk, K_trk, vG_trk, pen_pt] = trackContactNormal(Pn, Vn, Vb_pt, h, hE, hN, kS, muP, K, prm)
% Pn, Vn, Vb_pt: 3x14 ; h,hE,hN,kS,muP,K: 1x14 ; prm.kc, prm.cc, prm.Nc
nz   = 1 ./ sqrt(1 + hE.^2 + hN.^2);
pen  = (Pn(3,:) + h) .* nz;
penD = (Vn(3,:) + hE.*Vn(2,:) + hN.*Vn(1,:)) .* nz;
N_pt = (pen > 0) .* max(0, prm.kc*kS.*pen + prm.cc*sqrt(kS).*penD);
for i = 1:2
  idx = (i-1)*prm.Nc + (1:prm.Nc);
  N_trk(i) = sum(N_pt(idx));
  w = N_pt(idx) / max(N_trk(i), 1);  if N_trk(i) < 1, w(:) = 1/prm.Nc; end
  muP_trk(i) = sum(w .* muP(idx));   K_trk(i) = sum(w .* K(idx));
  vG_trk(i)  = mean(Vb_pt(1, idx));
end
pen_pt = pen;
```
Outputs: N_pt [14×1], N_trk [2×1], muP_trk [2×1], K_trk [2×1], vG_trk [2×1], pen_pt [14×1] (double). Inputs depend only on 6-DOF states, so there is no algebraic loop.

### 4.2 MF-B `trackPlanarForces` (continuous)
Revision B signature: `[F_c, M_c, sDot, Fx_trk, M_turn, P_diss, chi_eff] = trackPlanarForces(N_pt, Vb_pt, s, vB_trk, vB_trk_d, muP, muLat, fR, K, Rcg, prm)`
with `s` [28x1] = [s_x(1:14); s_y(1:14)], `vB_trk_d` the 1 ms delayed belt speed (damping only). Per point: v_slip, sDot, mu_j and F_j of 2.5;
Fx_trk(i) = sum of F_x,j; F_pt = [F_x,j + F_rr,j; F_y,j; -N_j]. The listing below is the revision-A version.
**Why no block:** per-point lateral shear derivative with the friction-ellipse coupling, rolling resistance and the Σ r_j × F_j wrench over 14 points. As blocks this needs about 30 element-wise blocks plus per-point cross products. The Aerospace `3x3 Cross Product` handles only single vectors. One reviewable function is clearer, and no single block does it.
```matlab
function [F_c, M_c, sYdot, Fy_trk, M_turn, P_diss] = trackPlanarForces(N_pt, Vb_pt, sY, Fx_trk, N_trk, muP_trk, vB_trk, muLat, fR, K, Rcg, prm)
% sY: 14x1 lateral shear states (Integrator output), Rcg: 3x14 lever arms (w.r.t. CG)
F_pt = zeros(3,14); sYdot = zeros(14,1);
for i = 1:2
  idx = (i-1)*prm.Nc + (1:prm.Nc);
  eta = sqrt(max(prm.etaMin^2, 1 - (Fx_trk(i)/(muP_trk(i)*N_trk(i) + 1))^2));
  vy  = Vb_pt(2,idx);  vx = Vb_pt(1,idx);
  sYdot(idx) = (vy - abs(vy).*sY(idx)')./K(idx) - abs(vB_trk(i))/prm.ell*sY(idx)';
  cap = muLat(idx).*eta.*N_pt(idx)';
  Fy  = -min(max(cap.*(interp1q_g(sY(idx), prm) + prm.tau_d*sYdot(idx)'), -cap), cap);  % g(s) via prm.gS/gG
  Frr = -fR(idx).*N_pt(idx)'.*tanh(vx/prm.vEpsRR);
  Fx  = Fx_trk(i)*N_pt(idx)'/max(N_trk(i),1);
  F_pt(:,idx) = [Fx + Frr; Fy; -N_pt(idx)'];
end
F_c = sum(F_pt,2);  M_c = sum(cross(Rcg, F_pt, 1), 2);
Fy_trk = [sum(F_pt(2,1:7)); sum(F_pt(2,8:14))];  M_turn = sum(Rcg(1,:).*F_pt(2,:));
P_diss = -sum(sum(F_pt(1:2,:).*Vb_pt(1:2,:)));       % rolling + lateral shear power (diagnostic)
```
`interp1q_g` is a local codegen-safe linear interpolation of the same g(s) table used by the 1-D Lookup Table in 3.2, so there is one source of data. Inputs come from states (6-DOF, integrators, the Simscape belt velocity) and from the block-computed F_x, so there is no loop.

### 4.3 MF-C `obstacleProximity` (discrete, Ts = 0.1 s, in `structure_obstacle_monitor.slx`)
**Not built in this domain.** The separation and geofence monitors are in `Sim_Monitors` (`mon_sep_rect`, `mon_sep_circle`; D70), owned by the integration engineer. The design below is the original proposal.

**Why no block:** distance from the robot footprint to the scenario obstacle set. Robotics System Toolbox provides `collisionBox`, `collisionCylinder` and `checkCollision` (codegen-capable) as functions, not blocks.
```matlab
function [minDist, collision, outOfBounds, outOfMap] = obstacleProximity(pose2D, footprint, cyl, box, boundary, mapLim)
% footprint = [Lx Ly Lz] of robot+implement envelope; cyl: n×6 [xE yN r h zBase class]; box: n×9
robot = collisionBox(footprint(1), footprint(2), footprint(3));  robot.Pose = trvec2tform([pose2D(1:2) footprint(3)/2])*axang2tform([0 0 1 pose2D(3)]);
minDist = inf;  collision = false;
for k = 1:size(cyl,1)  % prefilter by planar distance < 3 m before checkCollision
   ... [c, d] = checkCollision(robot, obstacleCyl(k));  collision = collision | c;  minDist = min(minDist, d);
end
... same for boxes ...;  outOfBounds = ~inpolygon(pose2D(1), pose2D(2), boundary(:,1), boundary(:,2));
outOfMap = pose2D(1) < mapLim(1) || pose2D(1) > mapLim(2) || pose2D(2) < mapLim(3) || pose2D(2) > mapLim(4);
```
Fallback if `checkCollision` causes codegen trouble: an analytic rectangle-to-circle and rectangle-to-rectangle (SAT) distance in plain MATLAB.

**Offline (not blocks):** `param_structure.m`, `param_world.m`, `utilities/world_build.m` (+ scenario builders), `utilities/structure_mass_properties.m`, `utilities/structure_initial_state.m` (sets z, roll, pitch and settled sinkage at the start pose to avoid a drop transient), `tests/structure/structure_analytic_refs.m`.

---

## 5. Interfaces

### 5.1 Physical (Simscape) ports of `structure_plant.slx`
| Port | Domain | Connects to | Convention |
|---|---|---|---|
| `Spr_L` | mechanical rotational, **classic `foundation.mechanical.rotational.rotational`** | DRIVE left gearbox output (`Spr_L`) | +ω = forward track motion. STRUCTURE contains sprocket, idler, road wheels and belt only. |
| `Spr_R` | same | DRIVE right gearbox output (`Spr_R`) | same |
| `Impl_Shaft` | mechanical rotational (classic) | POWER's implement motor in `Power_Plant` (`VC_IMPL_DRIVE == 2`, the default). Unconnected in the harness (`== 1`, internal ideal speed source). | +ω = nominal implement direction |

STRUCTURE contains **no Solver Configuration**. The thermal port `H_encl` of the design is not built (§2.9). The ports and rates below are those of ICD section 3.2 (the `Structure` row), which binds.

### 5.2 Signal inputs
| Name | Producer | Bus | Ts | Notes |
|---|---|---|---|---|
| `impl_cmd` | `Electronics_Plant` (`Actuator_Outputs_Sim`, the decoded MCU DO and PWM) | `Bus_ImplementCmd` | 0.01 s | nozzle valves and lift duty; `speedCmd` for the harness only |
| `env_dist` | Constant `P.ic.Bus_EnvDisturbance` (zero in system runs) | `Bus_EnvDisturbance` | constant | external wrench at the CG for drawbar and push tests |
| `env_ovr` | the scenario, through `Plant_In` | `Bus_EnvOverride` | 0.1 s | environment override of the scenario |

### 5.3 Signal outputs
| Name | Consumer | Bus | Ts | Notes |
|---|---|---|---|---|
| `truth` | SENSORS (IMU, GNSS, LiDAR pose), `Sim_Monitors`, `Viz_Logging`, logging | `Bus_TruthState` | continuous; ZOH 1 ms at the plant root | ENU/FLU at R; never used for control. Continuous → discrete needs no Rate Transition. |
| `tracks` | `Sim_Monitors`, logging, tests | `Bus_TrackState` | continuous; ZOH 1 ms | |
| `impl_mech` | `Power_Plant` (`Implement_Loads`), `Electronics_Plant` (lift potentiometer ADC), logging | `Bus_ImplMechState` | 0.01 s (internal ZOH, D32) | |
| `env` | `Power_Plant`, `drive_system` (`T_amb`), `Electronics_Plant`, SENSORS | `Bus_EnvState` | 0.1 s | the single producer of `T_amb` |
| `diag` | logging, tests | `Bus_StructureDiag` | continuous; ZOH 0.01 s at the plant root | |

### 5.4 Bus definitions (`buses/buses_structure.m`, owner structure; listed in the ICD)
The element lists, units and ranges are defined in `buses/buses_structure.m`; the ICD is generated from them, so this spec does not repeat them. In short (all double unless stated):
- **`Bus_TruthState`** (1 ms): the true state at R in ENU/FLU (the 6-DOF integrates in NED/FRD and `structure_truth` converts): position, velocity and acceleration in W; `q_WB`, `R_WB` and `eul` (ZYX, θ + = nose down); body rates and accelerations; the physical specific force at R; `r_cg_B`; `lla`; `pose2D`; `speed_fwd`; `mass`; `terrainH`; the terrain grade along and across the body (`grade_long`, `grade_lat`); and the belt and ground speeds and slip of each track.
- **`Bus_TrackState`** (1 ms; index 1 = left, 2 = right): belt and ground speed, slip, shear state, `Fx`, `Fy`, `Fn`, the sprocket angle, speed and torque, sinkage, `muPeak`; `Mturn`; `Pslip`.
- **`Bus_ImplementCmd`** (0.01 s): `nozzleOn` [8] boolean, `liftDuty` (−1 to 1, + lowers the tool), `speedCmd` (harness only).
- **`Bus_ImplMechState`** (0.01 s): `type` (uint8), shaft speed, torque and power, tool height, the lift position `liftPos01` (in today's twin 0 at the top of the travel, 1 at the bottom: (h_range(2) − h)/span for the mower, d/d_max for the tiller; it follows the sensor, 0 at the bottom, on card G2-STR, as the note below says) and `liftMoving`, tank level and mass, spray flow, line pressure, vegetation density at the deck, `F_impl_B`.
- **`Bus_EnvDisturbance`**: `F_ext_B`, `M_ext_B` at the CG.
- **`Bus_EnvState`**: `T_amb`, `T_sky`, `T_ground`, `G_solar`, `soil_id` (uint8), `dock_contact` (boolean), `dist_dock`. It updates every 0.1 s.
- **`Bus_StructureDiag`** (0.01 s): `N_pt`, `pen_pt`, `sX`, `sY` (14 each), `slopeLong`, `slopeLat`, `P_diss`, `E_mech`, `E_diss_J`, `E_impl_shaft_J`, `chi_eff`, `outOfMap`. The obstacle distance and collision flags of the design are in `Sim_Monitors` (D70).

**The lift position's sense disagrees with the MCU's reading** (G1-01 step-14 re-check). The electronics plant passes `liftPos01` to the potentiometer ADC unchanged, and the MCU reads it as a height fraction, 0 at the bottom of the range: the deck height is h_range(1) + span × lift_pos, the deck counts as raised at lift_pos ≥ `lim.descent.lift_raised` (0.667), and the tiller depth is −d_max (1 − lift_pos) (`build_fw_control.m`; the electronics spec's `lift_raised` row). So in the twin the deck-height feedback, the tiller depth and the D81 raised-deck backstop all read inverted. Ruled by the lead systems engineer (step 19, point 22), as the structure engineer recommended: the plant follows the sensor and the MCU, a height fraction with 0 at the bottom and 1 at the top, as the electronics spec reads it. Only the structure twin changes, on card G2-STR (`buses/buses_structure.m`, `build_structure.m`), with the ICD row regenerated then; card G2-INT adds a system check that a raised deck selects the raised rows. The real potentiometer's direction is set in the MCU's calibration once card G1-04 gives it (pending).

### 5.5 Requirements derived for DRIVE (from this model's parameters; DRIVE's design covers them)
Per side, with the 1 kW motors (the 750 W motors in brackets where they differ, D95):
- **Sprocket speed** 0…15.7 rad/s: the top speed 1.41 m/s (`P.electronics.lim.v_track_max`) over r_s 0.090 m.
- **Steady climb of 10 deg, full-tank sprayer (316 kg), farm soil:** about 36 N·m, that is m g (sin θ + f_r cos θ) r_s / 2 plus the internal track friction.
- **Climb of 20 deg, full-tank sprayer, farm soil, 0.5 m/s:** 59.1 N·m with 9.3 % slip (T7).
- **Spot turn on asphalt, full-tank sprayer:** at most about 70 N·m, from the turning-moment bound μ_lat W L/4 over the gauge plus the rolling resistance. T9 measures a turning moment of 186.5 N·m with no implement on farm soil, against the 228.1 N·m bound.
- **Adhesion limit** μ_p N r_s: 125 N·m on asphalt with the full-tank sprayer, 66 N·m on farm soil with no implement. That is below the drive's peak sprocket torque T_peak N η = 344 N·m (258 N·m), so a track slips before the drive saturates.
- **Continuous sprocket torque** T_cont N η = 114.5 N·m (86.0 N·m) covers the 20 deg climb with a margin of 1.9 (1.5).
- DRIVE's gearbox ratings (`P.drive.gear.T_rated_out` 240 N·m, `T_acc_out` 360 N·m (280 N·m), `T_estop_out` 480 N·m) cover these.

---

## 6. Parameters

`params/param_structure.m`: `S = param_structure(cfg, W, g)`. `cfg` fields (all optional): `implement` (0 none, 1 mower, 2 tiller, 3 sprayer; = `VC_IMPL`, default 0), `tankFill0` (0..1, default 1), `batteryMass_kg` (default 18.5 = `P.power.pack.mass_kg` [EST]), `batteryPos_m` (default [0.05 0 −0.30]), `fidelity` (1 HiFi, 2 LoFi), `implDrive` (1 harness ideal drive, 2 POWER's motor; default 2), `motorOption` (the traction motor unit of `param_motor_unit`: empty = the default 1 kW unit, D95; `'movematic750'` = the as-built motor). `W` is `P.world` (for the initial state) and `g` is `P.frames.g`. `shaddad_params.m` sets `P.structure` and the variant controls `VC_STRUCT_FID`, `VC_IMPL`, `VC_IMPL_DRIVE` (plain double scalars in the base workspace, ICD section 8; the Variant Subsystems activate at 'update diagram'). It must build `P.world` **before** `P.structure`, because `structure_initial_state` needs the DEM.

### 6.1 Geometry (`S.geom`, `S.r_sprocket`)
The robot as built: XMachines MAX (`docs/hardware/real_robot_components.md`). [DATA] are the supplier's figures; the rest is estimated from the drawings [EST] (`params/param_structure.m:26-33`).

| Name | Value | Units | Source / justification |
|---|---|---|---|
| `L_overall`, `W_overall` | 1.00, 0.78 | m | the base robot, no implement. W [DATA]; the track body 1.0 m [EST] (the 1730 mm of the spec sheet includes the module that its render draws towed; the sprayer is mounted on the deck, the owner, 2026-11-15). With the mower the robot is about 1.70 m long and 1.05 m wide (the deck reaches 1.20 m ahead of R); with the tiller about 1.63 m long (to 1.125 m behind R) |
| `H_body`, `H_mast` | 0.57, 1.05 | m | H [DATA]; the mast [EST]. Under solar tables (lower edge 0.8 m, §6.9) the mast must fold below the edge |
| `B` (track gauge, centre-centre) | 0.60 | m | [EST]. L/B = 1.25, in the good skid-steer range 1.0–1.8 |
| `b` (track width) | 0.13 | m | 130 mm rubber track [EST] |
| `L` (contact length) | 0.75 | m | [EST]. Mean ground pressure m g / (2 b L): 11.3 kPa with no implement (226 kg; 11.0 kPa with the 750 W motors), 13.3 kPa on the tracks with the mower (its gauge wheels carry 261 N, T3), 15.6 kPa with the tiller, 15.9 kPa with the full-tank sprayer |
| **`r_sprocket`** (`S.r_sprocket`, also `S.track.r_s` and `P.drive.gear.r_spr`) | **0.090** | m | [EST] pitch radius from the side view, agreed with DRIVE |
| `r_idler`, `r_roadwheel`, `nRoadWheels` | 0.090, 0.077, 3 per side | m, – | [EST] from the drawings |
| `groundClearance` | 0.12 | m | [EST]. Raised beds and obstacles above it are out of the envelope (D54) |
| mounting lever arms | – | | SENSORS' own parameters (`P.sensors`) |

### 6.2 Mass breakdown (`S.mass.components`, body FRD w.r.t. R; boxes for inertia)
The total is the as-built robot's 220 kg [DATA] with its four Movematic 750 W units; the split is [EST] (`params/param_structure.m:35-58`). The chassis frame takes the remainder, so another motor option changes the total by 4 × its unit-mass difference: 226 kg with the default 1 kW units (D95).

| Component | m [kg] | centre [x y z] m | box [lx ly lz] m | Note |
|---|---|---|---|---|
| chassisFrame | 81.7 | [0 0 −0.25] | [0.95 0.50 0.20] | the remainder to the as-built 220 kg |
| bodyPanels | 20 | [0 0 −0.48] | [0.95 0.70 0.18] | panels, deck rail |
| trackAssyL / trackAssyR | 22 / 22 | [0 −0.30 −0.17] / [0 0.30 −0.17] | [0.94 0.13 0.34] | rubber track 8, sprocket 2, idler 2, 3 road wheels 3, frame 7 |
| driveLF, driveLR, driveRF, driveRR | 9.7 each (8.2 with the 750 W) | x ±0.34 (front / rear), y −0.17 (left) / +0.17 (right), z −0.22 | [0.24 0.12 0.12] | one unit at each sprocket: motor, gearbox, brake, half the side's dual ESC, cables (`param_motor_unit`) |
| batteryPack | 18.5 | [0.05 0 −0.30] | [0.393 0.3585 0.087] | 13S15P NMC, box XMT-P1-E01 (POWER, `P.power.pack.mass_kg` [EST]) |
| electronicsBay | 6 | [0.25 0 −0.40] | [0.30 0.20 0.10] | MCU boards, DC/DC, harness |
| sensorMast | 3 | [0.30 0 −0.75] | [0.10 0.20 0.35] | phase-2 GNSS and LiDAR |
| coolingMisc | 4 | [−0.15 0 −0.40] | [0.30 0.30 0.10] | |
| frontCoupler / rearHitch | 4 / 6 | [0.50 0 −0.25] / [−0.50 0 −0.25] | [0.08 0.40 0.15] | implement interfaces |
| **Total** | **226.0** (220.0 with the 750 W) | | | |

Implements (`S.impl.<x>.mass/pos/dims`): mower 65 kg [EST] at [0.90 0 −0.15], box [0.60 1.05 0.25]; tiller 85 kg [EST] at [−0.90 0 −0.30], box [0.45 0.75 0.40]; sprayer dry 50 kg [DATA, spec sheet] at [−0.05 0 −0.62], box [0.80 0.65 0.25]; sprayer liquid 40 kg full (40 L [EST] × 1000 kg/m³) at [0 0 −0.68], box [0.50 0.40 0.20].

### 6.3 Mass properties (computed by `structure_mass_properties.m`: point/box masses + parallel axis; values checked in MATLAB)
With the default 1 kW motors unless stated; T1 reproduces every row (t_structure, 2026-11-15).

| Config | m [kg] | r_cg w.r.t. R [m] | Ixx | Iyy | Izz | J13 [kg·m²] |
|---|---|---|---|---|---|---|
| base | 226 | [0.0076 0 −0.2670] | 11.30 | 22.35 | 27.25 | 0.60 |
| base, 750 W motors | 220 | [0.0078 0 −0.2683] | 11.10 | 21.60 | 26.35 | 0.60 |
| + mower | 291 | [0.2070 0 −0.2409] | 18.30 | 65.52 | 75.37 | −4.67 |
| + tiller | 311 | [−0.2404 0 −0.2760] | 16.48 | 75.87 | 83.55 | −1.25 |
| + sprayer, full | 316 | [−0.0025 0 −0.3751] | 23.34 | 35.73 | 33.18 | −0.20 |
| + sprayer, empty | 276 | [−0.0028 0 −0.3309] | 18.42 | 30.51 | 31.81 | −0.23 |

J13 is the tensor element −∫xz dm, so the full tensor is J = Σ[J_box + m(|p|²I − ppᵀ)], about the CG. `S.mp` = {m0, me, mf, Je, Jf, rcg_e, rcg_f, r_tank, rcg0, rcg0_FLU}.

**Static tip-over angles** (the slope at which the CG passes over the tipping edge, rigid body, deg). Forward and rearward: over the end contact points, x = ±0.321 m with the model geometry, and x = ±0.298 m with the sharp-edge track end that §2.11 takes as a floor (the second value of each pair). The sharp edge costs 1 to 2 deg where the CG is near the middle and 4.5 to 4.7 deg near an end contact. Lateral: over the track's outer edge (0.365 m) and over its centre line (0.30 m), the model's contact line.

| Config | forward (facing downhill), model / sharp edge | rearward (facing uphill), model / sharp edge | lateral, outer edge | lateral, contact line |
|---|---|---|---|---|
| no implement | 49.6 / 47.4 | 50.9 / 48.9 | 53.8 | 48.3 |
| mower, deck down, both gauge wheels on the ground | 75.7 (over its gauge wheels at x 1.15 m) | 65.5 / 64.5 | 56.6 | 51.2 |
| mower, deck down, gauge wheels lifted | 25.4 / **20.7** | 65.5 / 64.5 | 56.6 | 51.2 |
| mower, deck raised (0.09 m) | 23.7 / **19.2** | 63.7 / 62.7 | 54.4 | 49.0 |
| tiller, raised | 63.8 / 62.9 | **16.4 / 11.8** | 52.9 | 47.4 |
| sprayer, full tank | 40.8 / 38.7 | 40.4 / 38.2 | 44.2 | 38.7 |
| sprayer, empty | 44.4 / 42.3 | 43.9 / 41.7 | 47.8 | 42.2 |

Three cases come below or close to the 20 deg design slope, all near an end contact:
- **The raised tiller facing uphill** tips backward at 16.4 deg static, 11.8 deg with the sharp-edge track end (descent step 2). The climbing torque lowers this further; it is not yet quantified. So the tiller cannot climb the design slope. It is outside this phase (D76, card G2-TIL; real tiller data, §9 item 8); D74 forbids only reversing, and D111 (2026-11-18) narrows REQ-SYS-075 to match: with the tiller the robot descends the design slope forward only, and its climbs wait on card G2-TIL.
- **The raised mower deck facing downhill** tips at 23.7 deg, 19.2 deg with the sharp edge, below the design slope. So the deck stays down whenever the robot moves (D81); in the twin the MCU's backstop waits for the lift-position fix (§5.4).
- **The deck-down mower** holds 75.7 deg only while both gauge wheels are on the ground. If they lift (a dip or a crest under the deck, which reaches 1.2 m ahead), it tips forward at 25.4 deg, 20.7 deg with the sharp edge, 0.7 deg above the design slope. The descent table's mower rows rest on the same assumption, and no rule yet keeps the wheels down (§9 item 11).

Every other configuration keeps at least 19.7 deg of static margin at the slope of its envelope; the tightest is the full-tank sprayer facing uphill, 38.2 deg with the sharp edge at 18.5 deg. In pitch the three cases above lie below the MCU's tilt stop (28 deg) and warning (25 deg), which act on max(\|roll\|, \|pitch\|), so the tilt stop protects them in roll only: in pitch they rest on D76, D81 and the gauge wheels. The dynamic criterion of a brakes-locked stop is the park criterion of §2.11: the peak pitch relative to the terrain at most half the rotation to the static tip. On a 20 deg climb the centre of pressure leaves the middle third and the track fronts unload (T7), which matters for turning on slopes (T7b).

### 6.4 Track and shear model (`S.track`)
| Name | Value | Units | Source / justification |
|---|---|---|---|
| `J_eq` | 0.03 | kg·m² | [EST] sprocket + idler + 3 road wheels (equivalent mass J_eq/r_s² = 3.7 kg). **Excludes DRIVE inertia.** |
| `m_belt` | 8 | kg | [EST] 130 mm rubber track |
| `f_c` → `T_col` | 0.015 → 0.015·N_nom·r_s (1.49 N·m with no implement, 2.09 N·m with the full-tank sprayer) | – → N·m | internal resistance of rubber tracks ≈ 2–3% of load (Wong, *Theory of Ground Vehicles*, 4th ed., ch. 6) |
| `T_brk`, `w_brk` | 1.3·T_col (1.94 N·m with no implement), 0.05 | N·m, rad/s | breakaway ≈ 4.5 mm/s belt |
| `c_visc` | 0.0448 with no implement (half a percent of N_nom·r_s²) | N·m·s/rad | speed-dependent part ≈ 1% of load at 2 m/s |
| `Nc` | 7 | – | per track (segment-centred) |
| `ell` | L/2 = 0.375 | m | relaxation length = L/2 (exact match to lumped Janosi) |
| `tau_d` | 0.01 | s | shear damping. Gives ζ ≈ 0.35 on asphalt and keeps the explicit damper step limit > 9 ms |
| `etaMin` | 0.10 | – | friction-ellipse floor (revision A only; the isotropic law needs none) |
| `shear_model`, `vb_delay` | 2, 1 | –, samples | 2 = the per-point isotropic 2-D law (D39; 1 = revision A, retired); the belt speed of the damping term is delayed one 1 ms sample (D36) |
| `chi_nom` | 1.3936 | – | effective gauge factor B_eff/B, identified by `utilities/structure_identify_chi.m` on HiFi (farm soil, R = 2 m turn at 0.5 m/s, D39); the single source of `P.electronics.mix.chi`, the EKF prior, `P.structure.lofi.chi` and the GNC turn limits. T10's re-identification is decided (D100, §8) |
| `chi_soil` | [1.4038 1.3936 1.4882 1.3281 1.3237] | – | the same per soil class 1 to 5 (sand, farm, wet, greenhouse, asphalt) |
| `chi_rmin`, `v_eps_slip` | 0.05, 0.05 | rad/s, m/s | chi_eff is NaN below this yaw rate; slip = (v_b − v_g)/max(\|v_b\|, v_eps_slip) |
| `vEpsRR` | 0.05 | m/s | rolling-resistance smoothing |
| `gS` | [−1 −0.995 −0.98 −0.95 −0.9 −0.8 −0.7 −0.6 −0.5 −0.4 −0.3 −0.2 −0.15 −0.1 −0.05 −0.02 −0.01 0 …mirror…] | – | breakpoints of g(s), mirrored about 0 |
| `gG` | odd; for s ≥ 0: 0, 0.0100, 0.0201, 0.0508, 0.1033, 0.1574, 0.2131, 0.3284, 0.4477, 0.5677, 0.6833, 0.7877, 0.8750, 0.9444, 0.9737, 0.9898, 0.9975, 1, mirrored as an odd function | – | g(s) = f(2s/(1−s)) (computed in MATLAB) |

### 6.5 Contact (`S.contact`)
| Name | Value | Units | Justification |
|---|---|---|---|
| `Rc_R` | x = ±{0.3214, 0.2143, 0.1071}, 0; y = −0.30 (L) / +0.30 (R); z = 0; the first seven points left, the last seven right | m | section 2.3 |
| `kc` | 3.2e4 per point (Σ 4.48e5) | N/m | [EST] rubber track + undercarriage compliance. Static deflection 4.94 mm with no implement (226 kg; T2 measures 4.94 mm) to 6.90 mm with the full-tank sprayer (316 kg) on hard ground, ×1/kScale on soft soil |
| `cc` | 1.0e3 per point | N·s/m | [EST] heave ζ = 0.70, roll ζ ≈ 0.93, pitch ζ ≈ 0.47 (base, asphalt; 0.59, 0.65 and 0.37 with the full-tank sprayer) |
| `gw` (mower gauge wheels, [EST]) | two caster wheels at x 1.15 m and y ±0.45 m; k 3.2e4 N/m and c 1e3 N·s/m per wheel; height z −0.0184 m, derived so that the wheels carry 40 % of the deck weight at rest; `on` is 1 only with the mower | m, N/m, N·s/m | the rigid deck on its caster gauge wheels (owner, 2026-11-15: "Rigid, gauge wheels on ground"); `lambda_gw` sets the height |
| Modal check (base, asphalt; uncoupled, about the CG) | heave 44.5 rad/s (7.1 Hz), roll 60 rad/s, pitch 30 rad/s | | all ≪ 1/h, so not stiff |

### 6.6 Soil table (`P.world.soilTable`, owned by world; per-property grids are pre-blended)
| id | Soil | μp (peak traction) | K [m] | μlat | f_r | kScale | Source |
|---|---|---|---|---|---|---|---|
| 1 | dry sand (drift) | 0.60 | 0.025 | 0.45 | 0.10 | 0.5 | μp = c/p + tan φ with c = 1.04 kPa, φ = 28°, p ≈ 16 kPa (Wong); sinkage about 20–26 mm |
| 2 | farm soil, sandy loam (default) | 0.66 | 0.025 | 0.55 | 0.07 | 0.8 | c = 1.7 kPa, φ = 29° (Wong, sandy loam) |
| 3 | wet/irrigated loam (palm basin) | 0.50 | 0.030 | 0.45 | 0.12 | 0.6 | c ≈ 4 kPa, φ ≈ 15° |
| 4 | greenhouse floor (compacted soil/gravel) | 0.80 | 0.010 | 0.60 | 0.05 | 1.0 | engineering estimate |
| 5 | asphalt / concrete, dry | 0.90 | 0.005 | 0.75 | 0.035 | 1.0 | rubber-on-asphalt; K is lug compliance |

Analytic steady slip at 1 m/s on flat ground (any mass, since F_rr/N is mass-independent; L = 0.75 m): asphalt 0.053 %, greenhouse 0.174 %, farm 0.762 %, sand 1.255 %, wet 2.306 % (T4 measures each within 1 %). Every soil value is an estimate [EST] until the slope brake test (card T-04).

### 6.7 Implements (`S.impl`)
| Name | Value | Units | Note |
|---|---|---|---|
| mower `w`, `nBlades`, `R_blade` | 1.00, 2, 0.26 | m, –, m | out-front deck, counter-rotating |
| mower `J`, `D` | 0.05, 1e-4 | kg·m², N·m·s/rad | 2 blades + rotors lumped; the bearing drag of 1e-4 (8.6 W at 293 rad/s) leaves the T12 target of 1100 W to the aero idle and the cutting (the earlier 1e-3 took 86 W) |
| mower `w_nom`, `k_aero` | 293.2 (2800 rpm), 1.19e-5 | rad/s, N·m·s² | P_idle = k ω³ = 300 W, tip speed 76 m/s |
| mower `e_A`, `c_veg` | 800, 40 | J/m², N/m | medium grass/weeds; ρ_veg ∈ [0, 3] from the veg grid |
| mower `lambda_gw`, `f_gw`, `h_range`, `h_rate`, `h_tau` | 0.4, 0.10, [0.03 0.12], 0.02, 0.2 | –, –, m, m/s, s | gauge wheels: `lambda_gw` sets the wheel height and the share of the deck weight the wheels carry at rest (§6.5, `contact.gw`) |
| mower `h0`, `k_yaw`, `v_eps`, `w_min_frac` | 0.12, 0, 0.05, 0.3 | m, –, m/s, – | the deck's initial height, no yaw torque (counter-rotating blades), drag smoothing, the floor of the cutting-torque speed |
| tiller `w`, `d_max`, `d_nom`, `R_rot` | 0.60, 0.12, 0.08, 0.18 | m | rear rotary |
| tiller `J`, `w_nom`, `T_idle`, `e_V` | 0.35, 26.18 (250 rpm), 4, 80e3 | kg·m², rad/s, N·m, J/m³ | rotary-tillage specific energy 40–150 kJ/m³ (sandy loam ≈ 80) |
| tiller `kPush`, `kLift`, `lambda_skid`, `mu_skid`, `a_r`, `n_t` | 0.3, 0.2, 0.3, 0.4, 0.3, 6 | – | down-cut thrust, uplift, depth skid, torque ripple |
| tiller `d_rate`, `d_tau`, `x_rot`, `d_on` | 0.03, 0.3, −0.95, 1e-3 | m/s, s, m, m | the depth actuator, the rotor position behind R, the depth at which the tool is in the ground |
| sprayer `V_max`, `rho_liq`, `nNoz`, `Q_ref`, `p_ref`, `p_set` | 0.040 (40 L), 1000, 8, 1.0e-5 (0.6 L/min), 3e5, 3e5 | m³, kg/m³, –, m³/s, Pa, Pa | [EST] the tank and nozzles; the module's dry mass, 50 kg, is the spec sheet's [DATA] |
| sprayer `D_p`, `w_pump`, `eta_m`, `eta_v`, `J` | 5.3e-7, 314.16 (3000 rpm), 0.85, 0.90, 2e-3 | m³/rad, rad/s, –, –, kg·m² | [EST] diaphragm pump, 9 L/min max (D_p ω η_v) |
| sprayer `fill0`, `x_boom` | the cfg `tankFill0` (default 1), −0.20 | –, m | the initial tank fill; the boom behind R |
| `eta_impl` | 0.85 | – | the implement drive's efficiency |
| `drive.a_max`, `drive.tau_f`, `lift.duty_dead` | 100, 0.05, 0.05 | rad/s², s, – | the harness ideal drive (`VC_IMPL_DRIVE == 1`): its speed ramp and S-PS filter; a lift duty above `duty_dead`, away from an end stop, is `liftMoving` |
| `descent` (`S.descent`, card G2-DSC) | twelve breakpoints; four rows per direction (`v_fwd`, `v_rev`) and the two raised-deck rows | deg, m/s | the cap table of §2.11; read by the MCU (`lim.descent`) and by the GNC (`desc`) |

### 6.8 Enclosure thermal (`S.thermal`)
Not built (§2.9): P has no `S.thermal`. The design values were: `m_eq` = 60 kg, `cp_eq` = 500 J/(kg·K) (C = 30 kJ/K) · `A_conv` = 4.0 m² · `h0` = 5.7, `h1` = 3.8 W/(m²K)/(m/s) · `alpha_s` = 0.65 (red paint) · `A_s` = 1.6 m² · `eps` = 0.9 · `A_rad` = 2.5 m² · `dT_sky` = −12 K.

### 6.9 World (`params/param_world.m` → `P.world`; builder `utilities/world_build.m`)
| Field | Content |
|---|---|
| `name` (worldId) | 1 'flat_test' (the default), 2 'slope_test', 3 'palm_farm', 4 'greenhouse', 5 'solar_farm', 6 'soil_strips', 7 'row_field' |
| `lla0` | [24.1556 47.3120 430] |
| `env` | `T_amb` = 318.15 K (45 °C, the design ambient; an optional daily profile, `use_diurnal` 0 by default: `T_mean` 311.15 K ± `T_amp` 9 K, that is 302–320 K, peaking at 15 h), `G_solar` = 1000 W/m² times the shade grid (0.6 in the greenhouse, 0.2 under the panels), `T_ground` = T_amb + 15 K (333.15 K), `T_sky` = T_amb − 12 K, `g` = 9.7876 |
| `terrain` | `res` = 0.25 m (`row_field`: the field's span over 240 cells, 0.3125 m for the row test), `xE` (1×Nx), `yN` (1×Ny), `H`, `dHdE`, `dHdN` (Nx×Ny; Table(i,j) ↔ (xE(i), yN(j))) |
| `soilTable`, `soilId`, `soil.muPeak/K/muLat/fRoll/kScale` | table 6.6 + blended grids |
| `veg.rho` | vegetation density grid (0–3) for mower/sprayer |
| `obstacles.cyl` | n×6 [xE yN r h zBase class] (1 palm trunk, 2 post, 3 solar pier, 7 person-size static) |
| `obstacles.box` | n×9 [xE yN zBase lx ly lz yaw class id] (4 greenhouse wall, 5 static box / test obstacle, 6 solar table (overhead, zBase 0.8 m)); revision B: no raised beds in the greenhouse mower mission (beds 0.25 m exceed the 0.12 m ground clearance, D54); `gnssSvf` / `gnssZoneId` grids added (D59) |
| `rows` | n×5 [x1E y1N x2E y2N laneId] lane centrelines for row-by-row missions |
| `boundary` | polygon m×2 (geofence) |
| `start` | [xE yN yaw_enu] |
| `occ` | `grid` (logical, binaryOccupancyMap semantics: row 1 = max y), `res` = 10 cells/m, `origin` = [xE0 yN0] |

The worlds (`params/param_world.m`, `utilities/world_scn_*.m`):
- **`flat_test`** (the default) and **`slope_test`**: 60 m × 60 m at 0.25 m (241 × 241), farm soil; one palm trunk and one row. `slope_test` adds the 20 deg design ramp along +x from x = 10 to 30 m, with 3 m fillets and a plateau after it.
- **`palm_farm`**: a grid of 84 m (E) × 140 m (N) at 0.25 m. 7 rows of date palms north–south at 8 m, 13 palms per row at 8 m; trunks r 0.30 to 0.40 m, 6 m high; canopy circles r 3 to 4 m. Each palm sits in an irrigation basin: a berm ring of r 1.5 m, 0.15 m high and 0.4 m wide, with wet loam (soil 3) inside r 1.3 m. Sandy loam elsewhere, with dry-sand discs (r 3 m) over 10 % of it. 1.5 cm RMS roughness (0.5 m correlation) and a 0.5 % slope northwards. Geofence 80 m × 136 m.
- **`greenhouse`**: the house 9.6 m × 37.5 m (360 m²) on a 30 m × 58 m grid, the greenhouse floor (soil 4) inside; film walls 3 m high with a 3 m door; steel posts r 0.05 m every 3.0 m along both walls (post edge 0.20 m from the wall, D54); no raised beds (D54).
- **`solar_farm`**: 60 m × 60 m, flat, farm soil, vegetation 2.0. Five east–west table rows 4.0 m deep at a 7 m pitch, lower edge 0.8 m; piers r 0.08 m every 6 m; aisles between the rows.
- **`soil_strips`**: five 10 m strips of soils 1 to 5 for the slip and turning tests.
- **`row_field`**: sized to the rows of the RC row test (`row_test_rows`).

None of the worlds has a slope above the 20 deg design ramp, apart from the flanks of the palm-basin berms (0.15 m high, above the 0.12 m clearance and outside the envelope; up to 21 deg in the 0.25 m DEM, about 28 deg in the berm's own profile), and none has a cross-slope; the cross-slope cases are test planes in `t_structure` (T7b, T8).

### 6.10 Initial state and simulation (`S.init`, `S.sim`, `S.lofi`)
`init.xme0`, `init.eul0` from `structure_initial_state(P.world, S)`: settled sinkage, terrain-matched roll/pitch, static CG pitch; the start pose includes the mower's gauge wheels (card G2-DSC; without them the mower starts with a large wheel penetration). `init.Vb0 = [v0 0 0]`, `init.v0 = 0`. `sim.h_max` = 2e-3 s. `lofi.chi` = chi_nom (1.3936); `lofi.i_nom` = 0.0076, the steady straight-line slip on farm soil, so that LoFi and HiFi travel the same distance on a straight row (T18); `lofi.h_cg`, `lofi.J_side` and `lofi.Izz` follow the active configuration.

---

## 7. Internal structure, files and model types

All STRUCTURE files are **Subsystem References** (plant, not code-generated), written by `scripts/build/build_structure.m` and `build_structure_env.m`. There are no Model References in this domain. Files live in `subsystems/structure/` unless stated otherwise.

```
structure_plant.slx                     [SR] domain top (ICD 3.2). Ports: Spr_L, Spr_R, Impl_Shaft | impl_cmd, env_dist, env_ovr | truth, tracks, impl_mech, env, diag
├─ Fidelity (Variant Subsystem, VC_STRUCT_FID, 'update diagram')
│  ├─ structure_hifi.slx                [SR]
│  │  ├─ TrackL, TrackR : structure_track_side.slx   [SR ×2, Simscape]
│  │  ├─ structure_track_shear.slx                   [SR, Simulink, 28 shear states]
│  │  ├─ structure_contact.slx                       [SR] kinematics + 9 terrain lookups + MF-A + MF-B + the gauge-wheel blocks
│  │  └─ structure_body6dof.slx                      [SR] gravity, wrench sum, 6DOF, accelerations, r_cg(m)
│  └─ structure_lofi.slx                [SR]
├─ structure_implement.slx              [SR] Variant VC_IMPL: structure_impl_none/mower/tiller/sprayer.slx; inner variant VC_IMPL_DRIVE
├─ structure_env.slx                    [SR] Bus_EnvState from P.world.env, the scenario override and the shade grid (Constant or Sine Wave for the daily profile)
└─ structure_truth.slx                  [SR] NED/FRD → ENU/FLU at R: bus assembly, quaternion, LLA, pose2D, specific force
Not built: structure_enclosure_thermal.slx (§2.9), structure_obstacle_monitor.slx (§4.3; the monitors are in Sim_Monitors, D70)
Helpers: subsystems/structure/structure_contact_normal.m, structure_planar_forces.m, structure_gauge_points.m, structure_gauge_contact.m
params/param_structure.m, params/param_world.m
utilities/world_build.m, world_scn_palm_farm.m, world_scn_greenhouse.m, world_scn_solar_farm.m, world_scn_soil_strips.m,
          world_gnss_grids.m, world_occupancy.m, world_check.m, world_plot.m,
          structure_mass_properties.m, structure_initial_state.m, structure_identify_chi.m
tests/structure/t_structure.m, t_world.m, tb_structure.slx, tb_structure_env.slx, structure_analytic_refs.m, structure_tb_run.m
```
**Sample times.** The plant is continuous (0); `impl_mech` leaves through a 0.01 s Zero-Order Hold (D32) and `env` at 0.1 s. Inputs from discrete controllers (0.01 s) enter a continuous plant without Rate Transition blocks (ZOH). The network shared with DRIVE/POWER uses the integrator's Simscape local solver (Backward Euler, 1 ms, fixed cost). Global solver is fixed-step `ode3`, 1 ms. STRUCTURE limit: **h ≤ 2 ms** (lateral shear damper and lateral eigenvalues). Structure-only harness: `ode23t`, MaxStep 2 ms is also fine.

**States (HiFi).** 6-DOF 13 (incl. mass) + the 2-D shear 28 (D39) + Simscape 2 per side (ω/v_b tied) + implement 1–3, about 47. Cost per step: 14-point vectors, 9 vector lookups (O(1)), 2 small MATLAB Functions, 6-DOF. That is about 15–30 µs in Normal mode and about 5–10 µs in Accelerator. A 4 h mission at 1 ms (14.4 M steps) costs about 2–7 min for STRUCTURE alone. System speed is set by POWER/DRIVE networks and SENSORS (LiDAR ray casting).

---

## 8. Verification (benches `tests/structure/tb_structure.slx` and `tb_structure_env.slx`, class `t_structure.m`)

Bench `tb_structure`: `structure_plant` + Solver Configuration + per-side drivers (ideal angular velocity source with a ramp, ideal torque source, or free), set by `structure_tb_run.m` and `tb_structure_defaults.m`. Analytic references from `structure_analytic_refs.m` use the same parameter struct, so every criterion follows P. HiFi unless noted; fixed-step ode3 at 1 ms. The criteria below are those `t_structure` checks; the measured values are from its last full run (2026-11-15, `work/agentic/logs/20261115_descent_structure_final.log`): 21 tests, 19 pass, and T10 and T19 fail, both known failures in `tests/test_baseline.json`. The build checks (T03: interfaces, the MATLAB Function bodies, the compile of every variant) run with them.

| ID | Scenario | Pass criteria (measured 2026-11-15) |
|---|---|---|
| T1 | `param_structure` mass properties for the 5 configurations, 1 kW motors | m, r_cg and J within 0.1 % of `structure_mass_properties.m` (§6.3); m = 226 / 291 / 311 / 316 / 276 kg ± 0.05 kg; base r_cg [0.0076 0 −0.2670] ± 6e-4 m |
| T1b | Settled initial state (`structure_initial_state`) | the static contact equilibrium converges; pitch equals the analytic value (flat asphalt −0.0484 deg; 20 deg ramp −0.3608 rad with the contact compliance) |
| T2 | Static settle, flat asphalt, base, released 2 mm above the settled state | at 3 s: ΣN = m g ± 0.5 % (2212 N); sinkage = the analytic value ± 0.3 mm (4.94 mm); pitch = the analytic value ± 0.05 deg (−0.048 deg, nose up +); every point loaded (least 149 N) |
| T3 | Static settle of every implement on level asphalt, released at the settled state (card G2-DSC) | pitch = the static solution ± 0.05 deg and track loads ± 0.5 %: none −0.048 deg; mower −0.833 deg, its gauge wheels carry 0.411 of the deck weight (261 N; λ_gw 0.40 ± 5 %); raised tiller +7.35 deg (8 of 14 track points lifted); full-tank sprayer +0.022 deg |
| T4 | Speed-controlled straight run at 1.0 m/s, per soil | steady slip within max(10 %, 0.05 % abs) of the analytic value: asphalt 0.053 %, greenhouse 0.174 %, farm 0.762 %, sand 1.255 %, wet 2.306 %; lateral drift < 1 cm |
| T5 | Torque step on both sprockets, asphalt, base | mean acceleration over 0.3–0.8 s within 1 % of the analytic value with the viscous track loss (4.786 m/s²; measured 4.753) |
| T6 | Drawbar pull, farm soil | not automated; the design criterion is the DP–slip curve within ±5 % of μ_p W f(iL/K) − f_r W |
| T7 | Grade: 20 deg ramp, farm soil, full-tank sprayer (316 kg), 0.5 m/s | climbs without stall (v > 0.4 m/s; 0.453); slip within 1.5 % abs of the analytic value (9.10 %; 9.31 %); sprocket torque within 5 % of the analytic value (59.2 N·m; 59.1) |
| T7b | Turn of R = 2 m on 15 and 20 deg slopes, 316 kg, farm soil (D53) | 15 deg: a downhill slide at the traction limit ≤ 0.1 m and a positive traction margin (0.000 m; 0.252); 20 deg: reported (margin 0.183) |
| T8 | Parking hold: 20 deg longitudinal slope and 15 deg side slope, sprockets locked, 70 s | drift between 5 and 65 s < 1 mm (0.000 and 0.079 mm); no shear state at its limit; the elastic settle is reported (21.5 and 37.3 mm) |
| T9 | Spot turn: belts ±0.5 m/s, farm soil, base | chi_eff in [1.25, 1.8] (1.449); longitudinal slip > 15 % on both tracks (31 %); chi_nom and chi_soil written by `structure_identify_chi` and single-sourced to the MCU mixer; \|M_turn\| reported against μ_lat W L/4 (186.5 against 228.1 N·m) |
| T10 | Steady turn R = 2 m at 0.5 m/s, farm soil | **Decided (D100), pending on card G2-STR:** chi re-identified on T10's own manoeuvre, with a 1 % self-check (chi_eff equals chi_nom within 1 %); Wong's estimate reported, its gap accepted until a field turning test. **Today** T10 checks chi_eff within ±10 % of Wong and the sprocket torques within ±15 %, and fails, a known failure: chi 1.417 against Wong's 1.266, torques −14.0 / 31.5 N·m against −10.7 / 28.1. chi_nom (1.3936) is 1.7 % below T10's own measurement, so the new self-check passes only after the re-identification |
| T11 | Side-slope traverse | not automated |
| T12 | Implement steady loads | mower at 1.0 m/s, ρ_veg 1, ω 293.2 rad/s: P_shaft = 1100 W ± 3 % (1100.0), deck drag 65.5 N ± 5 % (65.4). Sprayer, 8 nozzles at 3 bar for 300 s: tank 16.0 ± 0.5 L (16.07), line pressure 3 bar, the mass lost equals the liquid used ± 0.05 kg. The tiller is not automated |
| T13 to T15 | Terrain features, soil transition, power balance | not automated as separate tests; T18 checks the energy closure |
| T16 | Frames: drive north 10 m; LLA; standing on the 20 deg ramp | pos_W(2) = 10 ± 0.02 m and pose2D = [0 10 π/2] ± 0.02; lla within 1e-7 deg of `enu2lla`; on the ramp, eul(2) and the specific force equal the reference with the contact compliance (`structure_analytic_refs` 'slope_frames': −0.3608 rad, the chassis 0.0117 rad more nose-up than the rigid −0.349) ± 0.01 rad and ± 0.03 m/s². D100 makes this compliance reference S2's pitch criterion (± 0.02 rad, for S2 and S2spray; the reference on card G2-STR, the script on card G2-INT; today S2 and S2spray check the mean ramp pitch within ± 0.02 rad of the rigid ramp angle, T-INT-04) |
| T17 | Obstacle monitor | not in this domain (`Sim_Monitors`, D70) |
| T18 | HiFi against LoFi: a 100 m row and a 180 deg headland turn, palm_farm | end position difference < 1.0 m (0.40 m); sprocket energy within 10 % (1.6 %); energy closure ≤ 1 % (0.002 %) |
| T18b | Closed loop with a yaw-rate PI (Kp 0.4, Ki 2.0) | heading difference < 5 deg (0.33 deg) |
| T19 | Performance (Accelerator, ideal drives, palm_farm) | HiFi real-time factor ≥ 20 (32.7); LoFi ≥ 100 (68.0: a known failure, limited by the PC) |

---

## 9. Open questions, risks, convention feedback

### Open questions
1. **Battery mass and position** (POWER): settled. The pack is 18.5 kg [EST] (`P.power.pack.mass_kg`), placed at [0.05 0 −0.30] m (`P.structure.cfg.batteryPos_m`), and `param_structure(cfg)` takes the POWER value. Decision row D17 (a 118.8 kg pack, a 469.2 kg robot) predates the as-built robot.
2. **Implement motors**: settled. POWER models the implement motor on `Impl_Shaft` (`VC_IMPL_DRIVE == 2`, the default); the internal ideal speed source is for the harness. The mower's peak of about 2.3 kW and the tiller's of about 1.3 kW matter for the 48 V bus (POWER).
3. **Implement layout**: the model has the out-front mower deck on its gauge wheels (D81), a rear rotary tiller and the 40 L sprayer module on the deck (the owner, 2026-11-15). Only the sprayer's dry mass is the supplier's figure; the rest is estimated (G1-04 collects the implements' data).
4. **Frames**: settled by ICD section 1. The truth is in site ENU and body FLU at R (`Bus_TruthState`, `pose2D`); the 6-DOF integrates in NED/FRD inside the domain.
5. **`P.world` ownership**: settled. STRUCTURE owns it (`params/param_world.m`, §6.9); SENSORS casts its LiDAR rays against `obstacles` and the DEM.
6. **The turning model.** Radius-dependent turning resistance (Wong's empirical μt(R/B)) and road-wheel pressure peaks are not modelled, and the 7 equal contact points spread each track's load more evenly than its 3 road wheels do, which may overstate the turning moment (the review's STR-02). With D100, T10 accepts the gap to Wong until a field turning test (card G2-STR: steady turns of R 2 to 4 m and spot turns on soil and concrete); μ_lat and chi are then calibrated from it.
7. Robot height under solar tables (lower edge 0.8 m) requires a folding mast. Is this acceptable for SENSORS (GNSS antenna height)?
8. **Tiller data** (owner; card G2-DSC): the tiller (85 kg, 0.90 m behind the centre) is estimated. In HiFi the raised tiller falls over backward at rest on flat farm soil, and a gentle forward start on asphalt tips it. Facing uphill it tips backward at 16.4 deg static (11.8 deg with the sharp-edge track end, descent step 2); the climbing torque lowers this further, not yet quantified. So it cannot climb the 20 deg design slope (§6.3), and D111 (2026-11-18) narrows REQ-SYS-075 to match: with the tiller the robot descends forward only. The no-reversing rule (D74) holds, but the tiller's forward row only covers braking, and real tiller data are needed before any use (card G2-TIL).
9. **Descent inputs** [EST] (card G2-DSC): the contact stiffness and damping, the friction in a locked skid, the track-end shape and the tank height set the tip speed to within about ±50 %; with μp + fr at or below about 0.53 there is no tip. A brake test on a slope (card T-04) would calibrate them. The IMU error bound is 0.75 deg (D83, the owner, 2026-11-16; 0.5 deg before structure step 13), and card G2-NAV confirms it with real IMU data.
10. **Mass figures** (structure; STATUS 40): settled in revision C (§6.2, §6.3, with both motors). The decision record D17 is still to be reconciled (the configuration and documentation engineer).
11. **The mower's gauge wheels** (card G2-STR; G1-01 step-14 re-check, REQ-SYS-149): the deck-down mower's forward tip angle of 75.7 deg, the descent table's mower rows and REQ-SYS-149 all assume both caster gauge wheels on the ground. If they lift, the tip angle is 25.4 deg (20.7 deg with the sharp edge). Either show that they cannot lift within the envelope (the deck's reach of 1.2 m over the basins, berms and crests of the sites), or exclude the state by a rule.
12. **The 750 W robot** (cards G2-STR and G2-DRV; the lead systems engineer's ruling of step 19, point 24): the descent table (§2.11) is computed for the default 1 kW robot (D95). The as-built 750 W robot weighs 220 kg, with its centre of gravity 1.3 mm higher, which lowers the static tip angles by about 0.1 deg, inside every margin of §6.3. Its descent table is not computed: that run is pending on card G2-STR, with the drive's `t_drive` run on G2-DRV, before the robot moves on its 750 W motors with firmware built from the twin (D95's flashing condition). Until then no descent analysis here covers the 750 W robot.

### Risks
| ID | Risk | Mitigation |
|---|---|---|
| S1 | Port order of `Simple Variable Mass 6DOF (Euler Angles)` with `mass_flag='on'` (and whether `6DOF Angular Acceleration` with `mtype='Simple Variable'` needs dI/dt) could not be read. Built-in blocks expose no port labels in read-only mode. | At build: check labels on the block icon in a scratch model. Fallback: compute Abe = F_b/m and ω̇ = J⁻¹(M − ω×Jω) with Product/Invert 3x3 Matrix blocks, using mass from `mass_0 + ∫dm/dt`. |
| S2 | Explicit coupling across the Simulink/Simscape boundary (shear force computed in Simulink, belt integrated in Simscape). | The Dahl-relaxation law makes the coupling a moderate spring-damper (about 70 to 92 rad/s, §2.5a; the highest with the full-tank sprayer on asphalt and the 750 W motors; h ω < 0.2 at 2 ms). Stable at h ≤ 2 ms. Do **not** reintroduce a slip-ratio formula with ε, which produced τ ≈ 0.1 ms stiffness in the design study. |
| S3 | Algebraic loops | All contact inputs come from states (6-DOF, integrators, Simscape v_b). F_x feeds Simscape and MF-B only. Rule: never feed Abe/ω̇ back into the force path. |
| S4 | Lateral damper/eigenvalue limits the step (h ≤ 2 ms). | Documented in `S.sim.h_max` (2 ms). The system and T19 run at the 1 ms base step. |
| S5 | DEM table size (9 MB) slows model update and the Rapid Accelerator rebuild. | 0.25 m grid, 'Evenly spaced points'. Tables come from `P.world` (tunable) so scenario changes need no rebuild. |
| S6 | Robot leaves the DEM: 'Clip' extrapolation gives flat ground. | `outOfMap` flag in Diag; tests assert false. |
| S7 | Tank slosh, implement compliance, track tension/derailment, obstacle impact forces are not modelled. Collision is detection-only. | Out of scope; flagged in Diag. |
| S8 | CG shift with tank level vs a 6-DOF that assumes a fixed CG | Contact lever arms use r_cg(m) exactly. The quasi-static state drift is negligible (verified by T12 settle). |
| S9 | Variant Subsystems containing Simscape blocks | `VariantActivationTime='update diagram'`. Each choice has identical Connection Ports. |
| S10 | `Differential Drive Kinematic Model` defaults to interpreted execution (slow). | `SimulateUsing='Code generation'`. |
| S11 | Stateflow: `license('test','Stateflow')` = 1 but `ver` does not list Stateflow (same finding as DRIVE R4). STRUCTURE needs no Stateflow. | Inform RC/GNC/ELECTRONICS. |

### Feedback on the global conventions
* **Solver Configuration**: STRUCTURE, DRIVE and POWER form one Simscape network (mechanical through `Spr_*`, electrical through the motors, thermal through `H_encl`). Exactly one Solver Configuration belongs to the integrator. The global convention should say this explicitly.
* **`P.world`** should be added to the parameter assembly order: `P.world` → `P.structure` (needs the DEM for the initial state) → others.
* **Rate Transition**: needed only between discrete rates (multitasking). Continuous plant outputs sampled by discrete sensors need none.
* **Classic vs AB/PB mechanical libraries**: all domains should use the classic `fl_lib/Mechanical/*` blocks at shared ports (`foundation.mechanical.rotational`). AB blocks only behind `Interface (AB-Rotational)`.
* **Frames**: settled by ICD section 1: site ENU for maps, GNC and the truth, body FLU at R, and `pose2D` as the hand-off; NED/FRD stays inside this domain.
