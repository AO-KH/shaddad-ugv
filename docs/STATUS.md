# Shaddad UGV Simulink/Simscape model: system status

Project: `C:\Users\aomar\Desktop\Robot project\Shaddad_UGV.prj`. MATLAB R2026b (26.2.0.3386108) on Windows 11 with 16 GB RAM.
Source of truth: `docs/architecture.md` (D1-D122; its section 2b overrides the specs), `docs/icd.md`, `docs/specs/*.md`, the parameters `params/*.m` (the as-built robot since 2026-10-27, `docs/hardware/real_robot_components.md`) and the requirements `docs/requirements/system.yaml`. `docs/build_plan.md` is the record of the 17 build tasks.

## Current state (2026-11-18)

Brought up to date by card G1-01 part 2, step 15, its fix pass, step 19, and the consistency pass, step 19c. The project owner approved this baseline on 2026-11-18 (D110, step 22) and decided D111 to D113 with it. Section 6 lists the problems that are open, each with its card. The closed and overtaken problems keep their old numbers under "Closed and overtaken problems", and the acceptance of 2026-10-23 with the later result sections is kept after them for the record (sections 0 to 5 and 9). A number marked "not re-run" is older than the change named with it.

**The model.** All 17 build tasks are built (card H-01), and the layout clean-up, the direct links of the top level and the MATLAB Function conversion of Tasks 1 to 3 and 5 are done (H-03, H-04, G3-MF). `Shaddad_System` compiles with 0 warnings, 0 algebraic loops and 0 interface mismatches (card G2-DSC, 2026-11-16, section 5.3). The parameters describe the as-built robot: about 220 kg with its four 750 W motors and 226 kg with the 1 kW units that stay the model's default (D95), and the 13S15P NMC pack of about 2.95 kWh. The delivered electronics are the baseline, provisionally (D96): the twin's STM32G4 BMS is a variant, and the STM32H743 is the firmware's development target.

**The tests: the baseline of 2026-11-15** (`tests/test_baseline.json`; `python tools/team/test_baseline.py compare <log>` sorts a new run against it): 22 classes and 528 tests, **462 passed, 51 failed and 15 incomplete**. Every failure is a known one, with its reason in the class note. The counts below are per test; the class summaries in the file count a few tests twice.

| Class | Tests | Passed | Failed | Incomplete | Run |
|---|---|---|---|---|---|
| `tScenarios` | 15 | 6 | 8 | 1 | 2026-11-08 |
| `t_drive` | 34 | 29 | 0 | 5 | 2026-11-15 |
| `t_elec_plant` | 15 | 15 | 0 | 0 | 2026-11-15 |
| `t_framework` | 29 | 25 | 3 | 1 | 2026-11-08 |
| `t_gnc_mission` | 32 | 30 | 2 | 0 | 2026-11-15 |
| `t_gnc_tracking` | 16 | 11 | 5 | 0 | 2026-11-15 |
| `t_infra` | 20 | 14 | 6 | 0 | 2026-11-08 |
| `t_layout` | 58 | 57 | 1 | 0 | 2026-11-12 |
| `t_mcu_control` | 44 | 43 | 0 | 1 | 2026-11-15 |
| `t_mcu_platform` | 25 | 19 | 4 | 2 | 2026-11-15 |
| `t_mf_equiv` | 6 | 6 | 0 | 0 | 2026-11-15 |
| `t_nav` | 27 | 27 | 0 | 0 | 2026-11-08 |
| `t_pack_bms` | 27 | 18 | 9 | 0 | 2026-11-12 |
| `t_perception` | 17 | 13 | 4 | 0 | 2026-11-15 |
| `t_power_plant` | 26 | 21 | 4 | 1 | 2026-11-08 |
| `t_rc_link` | 13 | 13 | 0 | 0 | 2026-11-07 |
| `t_row_driver` | 48 | 48 | 0 | 0 | 2026-11-07 |
| `t_row_test_system` | 3 | 3 | 0 | 0 | 2026-11-07 |
| `t_sensors` | 19 | 15 | 3 | 1 | 2026-11-15 |
| `t_structure` | 20 | 18 | 2 | 0 | 2026-11-07 |
| `t_targets` | 17 | 14 | 0 | 3 | 2026-11-08 |
| `t_world` | 17 | 17 | 0 | 0 | 2026-11-07 |
| **Total** | **528** | **462** | **51** | **15** | |

The one `t_layout` failure, the MCU layout test, passes since 2026-11-16 (closed item 23). The baseline stays as recorded until the tester rebuilds it on card T-03.

**The scenarios: the last saved run of each** (`work/results/<id>_<date>_<time>.mat`). `tScenarios` re-scores the newest one, and its result in the baseline is the third column. Architecture section 12 gives each scenario's criteria as decided and as the script checks them today.

| Scenario | Last saved run | In the baseline | Note |
|---|---|---|---|
| S0 power-up | 2026-11-15 | passed | D91's wait in FAULT changes it (cards G2-MCU, then G2-INT) |
| S1 RC drive | 2026-11-12 | passed | the RTF criterion is still marked BLOCKED (item 15) |
| S2 slope | 2026-11-16 | failed, on S2.v_mean only (a known deviation) | section 5.3 |
| S2b, S2spray, S2c, S2d | 2026-11-16 | not in `tScenarios` | section 5.3; item 57 |
| S3 endurance | never run | incomplete | D101 re-scopes it (item 33) |
| S4 greenhouse | 2026-11-14 (the full run); a 100 s run on 2026-11-16 | failed, on S4.xte_max | the full run completes: 13.7 min, coverage 0.971, XTE RMS 0.040 m; items 16e and 18; D100 and D107 set its criteria |
| S5 palm patrol | 2026-10-23 | failed | not re-run since D68 and D72; D100 sets its criteria |
| S5b palm patrol, lost scan | 2026-11-16 | not in `tScenarios` | 13 of 13 |
| S6a, S6b | 2026-11-15 | failed | D100 judges them as S8b (card G2-INT) |
| S6c, S6d | 2026-11-15 | passed | S6c cannot fail as built (item 41) |
| S7farm | 2026-10-23 | failed | not re-run since D68 (items 4 and 5) |
| S7wet, S7temp | 2026-11-08 | failed | |
| S8a, S8b | 2026-11-15 | passed | |
| R1 RC row test | 2026-11-14 | not in `tScenarios` | a free run, `docs/row_test.md` |
| L1 live RC drive | 2026-10-27 | not in `tScenarios` | a free run, `docs/live_rc_drive.md` |
| D1 test drive | 2026-10-27 | not in `tScenarios` | a free run |

**Decided, pending on their cards.** The owner's decisions that change a parameter, a model, a script or a test, which their cards do later (the shared brief of G1-01 part 2, section 3). Until each change the model does what the middle column says, and every run is read with it.

| Decision | The model today | Card |
|---|---|---|
| D84: a person stop resumes only on the operator's command, and the near zone counts as occupied | a person hold ends by itself (H1) | G2-GNC |
| D85 and D90: IMU_FAULT is class B in AUTO, latched, with the RC limp at the design-slope cap | class C (derate 0.25, yaw loop off) | G2-MCU |
| D86: the E-stop stops traction as category 0 | as built; T-PWR-04's stopping check is still to come | G2-PWR |
| D87: the resume pivot only where the footprint fits | no clearance check (item 16c) | G2-GNC |
| D88 and D91: after every switch-on or reset the robot waits in FAULT for the operator | the relay re-arms at SELFTEST (item 32) | G2-MCU, then S0 on G2-INT |
| D89: navigation validity is a health flag (2 m) | the code already uses 2 m; the re-alignment without RTK stays open (item 26) | G2-NAV |
| D92: the blades restart only on the operator's command | the implement restarts after a hold (H1) | G2-GNC |
| D93: a low charge in an operator's pause alerts and waits | the return leaves the pause (P3) | G2-GNC |
| D94: the sealed pack, charged only when cool, is the default | the vented pack with dock air (items 16a and 54) | G2-PWR |
| D95: the 1 kW units stay the default, the 750 W selectable | both in the parameters; no firmware goes on the robot until its torque limits match the motors fitted | G2-DRV, G2-MCU, B-00 |
| D96: the delivered electronics are the baseline | provisional values; ICD sections 4 and 6 are marked | G1-04 |
| D98: precharge limits that fit any hardware, with an early short check | `R_pre` 22 Ohm with the 4.7 Ohm scenario override (item 10) | G2-PWR |
| D99: VFH is the default bypass | `P.gnc.variant.teb` = 1 (TEB) | G2-GNC |
| D100: the scenario criteria of S4, S5, S6, S2 and T10 | the scripts' criteria (architecture section 12) | G2-INT, G2-STR |
| D101: a mowing mission on one charge | `P.power.budget` and S3's script are the 3 h shift | G2-PWR, G2-INT |
| D104: a wrong but plausible IMU detected within 1.0 s | no such detection | G2-FSC, with G2-MCU and G2-NAV |
| D105: the pack in storage | the storage procedure is still to be written | G2-PWR |
| D106: the greenhouse inset 0.50 m | `c_in` 0.40 m | G2-GNC |
| D107: the S4 greenhouse mown within 22 min | no work-rate criterion (13.2 min on 2026-11-16) | G2-INT |
| D108: a longer person field ahead on open ground, for a person walking head-on (REQ-SYS-131: at least 3.4 m at 1.0 m/s and 4.0 m at 1.2 m/s) | the field 0.5 m + 0.8 s x speed on every site (item 46) | G2-FSC, G2-GNC; S5's head-on case on G2-INT |
| D111: with the tiller the robot descends forward only, and it plans missions on wet loam only up to 18 deg (REQ-SYS-075, REQ-SYS-089) | no slope limit per implement or soil in the planner (item 34); the tiller's climbs are not analysed (item 36) | G2-GNC, G2-TIL |
| D112: in full sun one mission from a cool start, then park in the shade, until the S3 run on the sealed pack | S3 not yet run on the sealed pack; no sun shield | G2-INT, G2-PWR |
| D117: after an ESC command time-out, the deliberate reset and a new arm recover the robot | only a power cycle recovers it (item 60) | G2-MCU |
| D118: on a hardware E-stop, K1 stays closed for the 1.5 s stop-category-1 delay | K1 opens 0.148 s after the press (item 61) | G2-MCU, checked on G2-PWR |
| D121: S1 measures its accelerator RTF | S1.rtf is hard-coded BLOCKED (item 63) | T-02 |

**Environment.** Simulink Coder and Embedded Coder are installed (the startup report of 2026-11-07), and MinGW64 14.2.0 is the C and C++ compiler. STM32CubeMX and the GNU Tools for STM32 are missing, so the STM32 board builds stay BLOCKED (card B-00). Windows Code Integrity blocked unsigned accelerator MEX files on this PC until 2026-11-22, when the project owner turned Smart App Control off (item 20, closed); SIL and accelerator runs work since, and the runs still default to `'Mode','normal'`. Restart MATLAB between long runs (section 3, memory).

## 6. Open problems (priority order)

Each open problem names its card. The closed and overtaken ones (1, 2, 3, 6, 8, 9, 16b's regression, 19, 20, 23, 24, 38 and 40) are under "Closed and overtaken problems", after section 8, with their evidence.

**Phase-2 mission behaviour:**

4. **Card G2-GNC, with G2-NAV. Not re-run since D68:** the numbers are S7farm's of 2026-10-23. T15/T12: a GNSS outage gives PAUSE NAV_DEGRADED about 4 s after RTK is lost, with no automatic resume. S7 requires driving through a 30 s outage on dead reckoning (`P.gnc.fsm` navigation-degraded policy against NavFusion `navValid`/quality). The cause (the GNC and the sensors and navigation engineers, G1-01 part 2): the GNC's quality term (`q_good`, D69; FLOAT and dead reckoning count as degraded) pauses the mission 2 s after the RTK fix is lost, and the 0.30 m `sigma_lost` FAULT then ends it. The way out of the error budget is the quality term on G2-GNC and, on G2-NAV, `tAbsMax` 35 s, the honesty and the re-convergence. Requirements: REQ-SYS-021, -022 and -023.
5. **Card G2-NAV. Not re-run in the system since D68.** T12: the covariance is optimistic (NEES in bound 78.7 % vs 90 %), and the position error is still 0.24 m 5 s after RTK fixed returns. The bench runs of 2026-11-08 (sensors and navigation engineer): NEES in bound 0.41, 0.41 and 0.44 against 0.9, and `pos_rec` 0.24 m 5 s after RTK fixed returns.
7. **Card G2-GNC. The numbers of 2026-10-23, not re-run.** T14: MPC row tracker yaw oscillation (2.15 deg RMS vs 1.5 deg) and pass overlap (23 % vs 15 %). The velocity profiler gives one cruise speed per replanned path (T15 -> T14).

**Other system findings** (from T16):

10. **Decided by D98, pending on card G2-PWR:** about 7 to 8 Ohm, a settled-bus test before the main contactor closes, an early short check at about 0.2 s and at most 200 J over a whole retry sequence; the 4.7 Ohm override goes. Precharge: the ESC idle load holds V_bus at 87 % of V_pack with `P.power.pdu.R_pre = 22 Ohm`. The scenarios apply the integration override `P.sim.ovr.R_pre = 4.7 Ohm` (`utilities/scn_integration_overrides.m`). T05/T04 must decide the real value.
11. **Card G2-MCU.** `Electronics_Plant/DI_Latency` initial condition reads "E-stop loops open" at t = 0. Shaddad_System works around it with the `ElecHw_Delay` IC `scn_ic_elec_hw` (T10).
12. **Card G2-INT.** Reference-solver mode (T-INT-02) does not run: `run_scenario(id,'Solver','ref')` fails with a data-integrity error between `Electronics_Thermal/T_delay` and `Solver_Encl` (T10). When that network keeps its local solver (tried and reverted during this acceptance), the next error is a sample-time mismatch: `structure_hifi/StructureDiag` mixes continuous and 0.001 s signals into a nonvirtual Bus Creator (T03). Fixing it needs Rate Transitions / ZOH at those PS-Simulink outputs, or a reference mode that keeps those local solvers.
13. **Card G2-NAV.** The mspsensorlib IMU System object cannot save SimState (T11), so operating-point restart (T-INT-09) and resumable long missions are impossible.
14. **Card G2-INT. The numbers of 2026-10-23, not re-run;** S3 is re-scoped by D101 (item 33), and accelerator MEX files were blocked on this PC until 2026-11-22 (item 20, closed). Speed: Normal-mode system RTF is about 0.45-0.5 in a fresh session (about 0.12 when the session is memory-loaded). Top-only accelerator gives 1.07. S3 (4 h) therefore takes about 8 h wall even in a fresh session. Next step: Model blocks (Plant, MCU, BMS, Companion) in Accelerator mode (D56), which needs `SimulationMode` per Model block. RTF targets in Accelerator: S1 >= 5, S3 >= 4. D115 (2026-11-21): S3 waits for the simplified vehicle model's measured speed gain (card G2-STR), and its real-time target is restated after it (card G2-INT).
15. **Card G2-INT.** The criterion `S1.rtf` in `scn_S1_rc_drive.m` is still hard-coded BLOCKED ("no C compiler"), although a compiler now exists (T16).

**Hardware / code generation** (BLOCKED until STM32CubeMX/CubeCLT and GNU Arm are installed; Simulink Coder is installed since 2026-11-07):

16. **Card B-00. The first three defects are closed (the fix pass of 2026-10-23, section 0); the register placeholders stay open.** Known HW-variant defects already found by the T17 preflight: Shaddad_BMS IO_In/IO_Out HW (FDCAN Read output order, a missing 64 -> 8 Selector; T04); `fw_io_in_hw` FDCAN Read port order (T08, `build_fw_platform.m` line 829); Shaddad_Companion with VC_TARGET 1 has an algebraic loop through `companion_io_ros2` (T10); register placeholders at library defaults (`build_targets('registers','apply')`, T17).

16a. **Card G2-PWR.** Update (BMS and power engineer, G1-01 part 2): with D94 the charge stops above 45 deg C, and the CCL reaches 0 from 50 deg C today; the 120 W allowance stays open. Regen with a hot pack (follow-up of D65, 2026-11-12): from 50 degC the BMS allows no charge current (CCL 0), so the MCU limits regen to T_regen from the 120 W allowance alone, about 0.32 N m per side at 1.0 m/s (0.45 N m at 0.65 m/s); since D65 the robot drives at 1.0 m/s at 55 degC, so braking and downhill speed hold rest on the spring brakes and drag. Measured in `work/agentic/logs/derate_55_analysis.log` (row test `rowtest_20261112_135241`); flagged in the 2026-10-28 derate assessment, section 7. Update (drive, critfix 2026-11-15, R-01): this now holds only in part. The ESC applies the spring brakes on every park_req (the end of CTRL_STOP, the end of ESTOP.DECEL, and FAULT), and 2 s after a DISABLING or QUICK_STOP that cannot reach standstill. With CCL 0 the ESC trip at 1.885 m/s comes 0.5 s after the regen drop on 20 deg, 1.15 s on 12 deg and 4.5 s on 8 deg. So a stop on a steep descent without an early park_req (CAN timeout, or a disable without park_req) still ends in the OVERSPEED FAULT at about 1.93 m/s; see item 19. Checked on the bench (T19b to T19d). Update (card G2-DSC, 2026-11-16): checked in the system by S2c and S2d with CCL 0 (section 5.3); see item 19.

16b. **Card B-01 (the check on the robot).** MCU yaw loop at commanded standstill (follow-up of D66, 2026-11-14): the fix is in the model only. The real robot needs the firmware regenerated and flashed (item 16's tools), and the companion must send exactly 0.0f while paused. Check on the robot: hold AUTO with the GNC paused for 2-5 min, logging `w_mix − w_ref` (exactly 0 about 1.3 s into the pause), `vtrack_ref` and the heading, then drive straight and look for yaw drift. Left as is: `fw_imu_tilt` gates its gyro bias on the measured standstill only (telemetry `yaw_rate_meas`). The closed-loop bench gyro has no bias, so no test exercises the new bias gate or the drive-disabled term. Report: `work/agentic/s4fix/mcu/report.md`. The park regression found with this fix is closed by D67 (16b under "Closed and overtaken problems").

16c. **Decided by D87, pending on card G2-GNC:** the resume pivot only on lanes that the mission compiler flags as wide enough for the footprint with the fitted implement, and a hold when a pivot is blocked. No new hold reason is added now; D84, D93 and D99 will need them, on G2-GNC. Limits of D67 (2026-11-14; accepted for now by the user the same day, for the improvement list of the review phase): the resume pivot has no clearance check. On the S4 wall lanes it clears the wall only within 0.066 m of the lane centre, it is allowed up to 0.3 m off, and nothing detects a blocked pivot (no ALIGN timeout; the stuck check needs |v| >= 0.3 m/s). The rejoin GLOBAL fallback can U-turn about 3 m into the next lane. A fix would need a per-lane pivot and clearance flag in the mission compiler, or a perception clearance gate. Also open: a real loss of the drive still pauses with reason OPERATOR (a separate reason needs an enum change in `buses/`).

16d. **Card G2-NAV.** Open after D68 (nav, 2026-11-14; report `work/agentic/s4fix/nav2/report.md`): the EKF slip still reads -0.019 / -0.018 in the S4 rows (truth +0.007), so v_nav is 2.5 % high and the along-aisle error keeps a +0.08 m forward bias; the matcher's capture along an aisle is one-sided (from 0.45-0.6 m ahead it returns its start), and the channel-5 start guard only delays such a match by a few scans; an EKF-side error is left (the channel-2 NIS in the house is above its expected value; the 2.2 deg pitch offset at rest is a candidate); the along-aisle honesty at standstill is not measured; the matcher's fixed 0.25 m line gap would give an optimistic sigma in an aisle without posts; tSM01's NEES bound (2) is loose against the 0.2 measured; the companion code generation of the matcher is unverified. The user accepted the start guard as a delay for now (2026-11-14, for the improvement list); a 100 s S4 run with `kRvel` = kR (the lead, 2026-11-14) showed the velocity weighting cost accuracy in the simulation (along-aisle 0.095 against 0.065 m RMS, NEES 1.66 against 1.08, slip -0.019 against -0.010), so `kRvel` is now kR x [1 1 1]; the channel-2 NIS in the house is 2.77 with it (2 expected), a small EKF-side error left (the pitch offset is a candidate).

16e. **Card G2-GNC.** D100 keeps the 0.15 m maximum over the whole row, the entry included; the entry transient (0.23 m estimated, 0.25 m true) is guidance's to fix. Open after D69 (T7 on the cross-track sigma, 2026-11-14; report `work/agentic/s4fix/t7/report.md`): the closed-loop check never reached a headland turn (the 100 s run ends in the first row). At the apex of a turn the cross-lane check judges the along-aisle sigma, 0.093 m at most in the rows, a 1.6x margin against 0.15 m. The full S4 run of 2026-11-14 covered them: across-lane sigma max 0.070 m in TURN (2.1x margin), 0.061 m in ROW; along max 0.090 m; no pause in the whole mission. S4 now completes (13.7 min, coverage 0.971, XTE RMS 0.040 m); only S4.xte_max fails (0.252 m against 0.15 m, peaks 0.4-0.7 s after each row entry from a headland turn; dashboard item s4xte). The bench sets both posStd axes equal and covPose to 0, so only the harness test covers the projection. Real robot: the checks listed in D69.

16f. **Card G3-MF4.** Open after the MATLAB Function plan (D70, 2026-11-14; report `work/agentic/reports/mfplan.md`): the collision monitors and drive_esc Feedback / Torque_Limiter are converted, bit-identical in `t_mf_equiv` and in six whole-model compare runs. Left:
- **Task 4, the GNC managers** (`gnc_ImplementManager`, `gnc_PathServer`), waits for a working `build_targets('Companion')`, which still fails at HybridAStar (dashboard item companion).
- **Each converted block holds a word-for-word copy of its `.m`**, because of the R2026b JIT relink assertion. After editing `mon_sep_*.m` or `esc_feedback.m` / `esc_torque_limit.m`, rerun `build_sim_monitors` or `build_drive({'esc'})`. `t_mf_equiv` fails while a copy is stale.
- **Feedback's casts:** through the ports, 11 of Feedback's 22 former Data Type Conversions see only booleans, uint8 or the internal tick. Their cast types were checked in the Task 3 block probe (`work/agentic/logs/20261114_mf_drv_probe.log`), not by `t_mf_equiv`.
- **`ESC_SM` still calls the external `esc_sm.m`.** If the JIT assertion of section 4.1 keeps appearing, that block is the remaining candidate.

**Test maintenance (stale expectations, no model defect):**

17. **Card T-03.** `t_infra` (87 -> 88 buses, 2 Solver Configurations, Operator_RC instance excluded from the RC-rate check, retry on the R2026b assertion), `t_mcu_platform` (T09 units are no longer stubs), `t_gnc_tracking` (START / auto_start, geofence-safe rows), `t_sensors` (create `sens_uavScene` when it is missing). For `t_gnc_tracking` the real cause is review R-30: the bench never starts its mission, and four tests pass with the robot standing still; the baseline's class note names the earlier cause (GNC engineer, G1-01 part 2). D114 (2026-11-21): `t_infra`'s bus and Solver Configuration counts are consistency checks, so they take their values from the parameters.

**Open after the critical fixes (critfix, 2026-11-15):**

18. **Decided by D87, D100 and D120, pending on cards G2-GNC and G2-INT:** the turn keeps to the lanes flagged wide enough for the footprint with the deck, and S4's clearance is judged with the deck, with scripted operator resumes. The turn clearance of REQ-SYS-088 is 1.1 m at 0.5 m/s: 0.5 m + 0.8 m per m/s of turn speed + 0.10 m guidance + 0.10 m navigation (the lead systems engineer's ruling 3, 2026-11-18, on the GNC engineer's 1.0 m); the turn speed that fits the house is card G2-GNC's design. S4 with the mower holds at the lane-end walls until R-06 (sensors-nav and gnc, 2026-11-15). The new protective field (D72) sees the greenhouse end walls 1.22-1.48 m ahead at each lane-end turn, inside the mower field (1.20-1.70 m at standstill). A person hold there cannot clear while the robot stands still, and bypass is off in this mission. The integration 100 s S4 (2026-11-15) confirms it: 2 PERSON holds, the second from 29.5 s on the turn into lane 1 (the east wall 0.05 m into the standstill field, 0.39 m from the deck front) never releases, so the robot never reaches a row (0 collisions). The recommended fix is the R-06 footprint work: the mission compiler and turn planner keep the deck front at least 0.5 m plus a margin from walls and posts. The alternative, a shorter standstill field ahead of an implement, gives less protection. Card T-07 confirmed it on 2026-11-23: the full S4 rerun (1365 s) pauses with OBST_TIMEOUT at 155.4 s, coverage 0 (finding T07-INT-01, `work/team/t07/reports/step08b_s4.md`). The owner decided at the T-07 review (D120, 2026-11-23; NASL-SHD-ER-007, decision 4): S4 stays a known failure under D72 until R-06 on card G2-GNC, with D100's scripted resumes on G2-INT, and the shorter standstill field is not taken.

21. **Decided by D84 (2026-11-17), pending on card G2-GNC:** after a person stop the robot moves off, and restarts its implement, only on the operator's deliberate resume command, and the near zone of the body (about 0.05 to 0.45 m in front) counts as occupied until then; a near-zone sensor stays a candidate for the safety concept (G2-FSC). The zone in front of the body (R-02; review decision 2, decided by D84). The fields of D72 start at the fitted implement, so the zone 0.55 to 1.20 m ahead (above the mower deck) and the near zone in front of the body are not covered. The sensors report's option was a field from the body front to the implement front plus the field length (one more parameter).

22. **Card T-03 (review R-64).** `testTEL27_NoPulses` passes by timing only (MCU, 2026-11-15). Its last check wants CTRL_STOP to end at its 6.0 s mark, and parking now ends about 0.09 s later. It stays a timing pass until R-64 fixes its sequence. D114 (2026-11-21) sets what it checks: the controlled stop, the park on the brakes and STANDBY (REQ-SYS-067).

25. **Card G2-GNC (D87: the footprint per implement).** The protective field follows the build setting, not the fitted implement (review, 2026-11-15). protXF / protXR come from VC_IMPL when P is built, so a companion built for the mower and run with no implement or the tiller starts its front field at 1.20 m instead of 0.55 m; nothing compares VC_IMPL with MCU_STATUS implement_type. With R-06: pick the field at run time from implement_type (body front when unknown), or fault when the build and the fitted implement differ.

26. **Card G2-NAV (D85 sets the fault class, D89 the validity, D104 the detection time; the re-alignment without RTK stays here).** An IMU loss of 1 s or more ends navigation in the greenhouse (review, 2026-11-15; D73 with R-44). After the navigation FAULT the filter restarts from P0, and ALIGN needs sigma_pos below 0.15 m, which needs RTK because the scan matcher is off in ALIGN. The GNC FAULTs after about 0.4 s anyway. Recovery is untested (T-NAV-13 ends at FAULT).

27. **Card G2-DRV, with the ESC's data from G1-04 (review R-14).** The brake confirmation only echoes the brake command (review, 2026-11-15). FB1 brake_released follows the ESC's coil command (esc_sm.m), so a mechanically stuck or worn brake still reaches STANDBY; check (c) then trips the STO after 0.2 s of motion, which cannot hold the robot. What the real ESC's FB1 b4 means waits for its manual (R-14).

28. **Card G2-NAV (review R-22).** On the robot, a frozen but re-published LiDAR cloud stays valid (review, 2026-11-15). valid = a new message and error code 0, so a driver that keeps re-publishing a stale cloud is not caught; the simulated LiDAR can only black out. A content check (as for the IMU) or a timestamp age once R-22 is settled.

**Open after the descent pass (card G2-DSC, 2026-11-16):**

29. **Card G2-MCU.** **Full-stick top-speed OVERSPEED at a slope entry** (integration step 12b; the owner's decision of 2026-11-16: an exit criterion of card G2-MCU). At 1.40 m/s on the 20 deg ramp the top-speed trip (1.44 m/s) fires at the fillet entry, on 4.5 deg of terrain, and parks the robot with the latch set. The park is safe (margin 0.030 m/s), but every full-stick descent then needs an operator reset. The margin is only 0.04 m/s (the manual top speed `v_max_man` 1.40 against the trip), and the gravity overshoot plus the 0.08 s prediction use it up (today's `lim.descent.t_pred`; 0.09 s is decided for REQ-SYS-031, pending on card G2-MCU, and uses a little more of the margin). The S2d run therefore uses a 1.2 m/s throttle. The trip margin and the manual top speed are MCU parameters.

30. **Card G2-MCU.** **Reverse trip after an uphill hill start** (integration step 12 point 4; MCU step 9 point 5; card G2-MCU). During the uphill acceleration after a hill start the body squats nose-up and the reverse held pitch rises: 18.9 to 20.6 deg in S2spray, and 20.5 to 22.3 deg in the 20 deg replay. The reverse trip then drops to 0.10 m/s (within the 1 s hold) or 0.05 m/s. Nothing happens while the robot climbs, but a stop with a roll-back inside that second meets the lower trip. On measured climbs above 21 deg the trip is the 0.05 m/s floor, and a measured roll-back alone would trip; that does not arise inside CR-001's envelope. There is also no HiFi evidence yet for a hill start without regeneration (structure step 11 point 2): a hill start with CCL 0 on the 20 deg ramp could join the S2 reruns.

31. **Card G2-INT.** **S2 baseline and the S4 scan match** (integration step 12 point 6). The `runs/today` S2 of `shaddad_layout_runs` is now the sprayer run, so a new S2 baseline is needed before the next layout-runs comparison. S4_100's scan match is 0.815 (limit > 0.8), lower because the 100 s window now starts 6.35 s later (the deck-down gate). Watch it in the next full S4.

32. **Decided by D91 (with D88), pending on card G2-MCU:** after every switch-on, watchdog reset or brown-out the robot waits in FAULT, and the relay is not re-armed until the operator's reset and a new arm command. **Safety relay re-arm** (the functional safety engineer's hazard log, F-01, Major; D30). The MCU's reset pulse re-arms the safety relay on the reset button, on RC CH9 over the SBUS radio (not safety-rated), and once automatically on every SELFTEST entry. After a controller restart the relay therefore re-arms with no manual action. The concept (G2-FSC) and the MCU engineer decide the fix: `docs/safety/hazard_log.md`.

33. **Cards G2-INT and G2-PWR (D101 makes S3 the low-energy return: mow at end of life until the return threshold, then return with at least the reserve); the sensor-fault scenarios stay with G2-INT.** **No scenario for the low-energy return, or for sensor faults other than lost GNSS** (hazard log F-06, Minor; the charter §6; goals SG-04 and SG-05). S3 waits for D115's outcome (2026-11-21: the simplified vehicle model's speed gain measured first, card G2-STR).

34. **Card G2-GNC.** **CR-001 in mission planning** (card G2-GNC; GNC step 10 point 1). The mission planner has no slope limit per implement, so the full-tank sprayer is not kept off ground steeper than 18.5 deg in planning. Until it is, the cap stops the sprayer at the top of steeper ground (its speed target is 0 from 21.5 deg). Needed: a slope limit per implement owned by structure, a slope check in `gnc_compileMission`, the same limit at run time, and a test. Spec: `docs/specs/gnc.md` §9 item 11. D111 (2026-11-18) adds the soil: missions on wet loam only up to 18 deg (REQ-SYS-089, with REQ-SYS-129).

35. **Card T-04.** **Estimated descent inputs and the slope brake test** (structure step 1 point 3; card T-04). The contact stiffness, the damping and the friction of a locked skid are estimates, and they set the tip speed to within about ±50 %. With μp + fr at or below about 0.53 there is no tip. A brake test on a slope would calibrate them, and CR-001's option 3 needs the same test.

36. **Card G2-TIL (the tillage mission, a later phase).** **Tiller data** (structure step 7 point 2 and step 4 point 3). The tiller (85 kg, 0.90 m behind the centre) is estimated. In HiFi the raised tiller falls over backward at rest on flat farm soil, and a gentle forward start on asphalt tips it. The no-reversing rule holds, but the forward row only covers braking. Real tiller data are needed before any use. Facing uphill the raised tiller tips backward at 16.4 deg static and 11.8 deg with the sharp-edge track end; the climbing torque lowers it further, not yet quantified. So it cannot climb the 20 deg design slope (structure spec §6.3; structure engineer, G1-01 part 2, step 19). D111 (2026-11-18) narrows REQ-SYS-075 to match: with the tiller the robot descends the design slope forward only, and its climbs wait on this card.

37. **Card G2-MCU, with G2-GNC.** **Deck height for the MCU and the GNC** (MCU part 2, points 1 and 2; GNC part 2, point 1). The raised-deck backstop reads the lift potentiometer (0.667 of the range; deck 0.09 m [EST]), and the GNC's deck wait uses an estimate. No parameter gives the height at which the gauge wheels leave the ground. A real deck report (MCU_STATUS, or a lift height on CAN2) would replace the estimate. A stuck potentiometer that reads low would hide a raised deck.

39. **Card G2-MCU.** **Remote E-stop with CCL 0 can trip STOP_FAIL (a)** (MCU step 8 point 8; the review of 2026-11-15). Coasting gives about 0.9 m/s² against the 1.0 m/s² bound of the stop monitor, so a remote E-stop on flat ground with CCL 0 can trip STOP_FAIL (a) and open HV. This is not changed; the stop monitor's bound needs a decision.

**Open from G1-01 part 2 (2026-11-17 and 2026-11-18), from the owners' reports:**

41. **Card G2-COM (review R-47).** S6c cannot fail as built: the twin's decoder rejects every injected error because each one carries a UART error flag, and an error that passes parity is not modelled (MCU firmware engineer). It overtakes item 8.
42. **Card G2-GNC.** The S4 transits' cross-track error, 0.49 m estimated and 0.51 m true, against the 0.5 m path bound (GNC engineer).
43. **Cards G2-GNC and G2-INT.** T12 (`t_gnc_mission`) does not verify the reserve on arrival, because its SOC is scripted; S3 must (D101) (GNC engineer). S3 waits for D115's outcome (2026-11-21).
44. **Card G2-NAV.** `tSEN03_LeverArm` expects `imu.r(1)` 0.35 m, while `P` has 0.25 m (sensors and navigation engineer).
45. **Card G2-GNC.** The quality term pauses palm-row missions on FLOAT, 40 % of T-NAV-04's row (sensors and navigation engineer).
46. **Decided by D108 (2026-11-18), pending on cards G2-FSC and G2-GNC, with S5's head-on case on card G2-INT:** on open ground the field ahead is lengthened for a person walking toward the robot (about 3.0 to 3.4 m at 1.0 m/s, and 3.6 to 4.0 m at 1.2 m/s; REQ-SYS-131 takes the upper ends, at least 3.4 m and 4.0 m); in a greenhouse the residual risk is accepted under D79. Earlier text: cards G1-02 and G2-FSC; the field length covers a standing or crossing person, but not one walking toward the robot (sensors and navigation engineer).
47. **Card G2-MCU.** The 2-position RC switches (arm, reset, lights, return home) are decoded with no debounce (finding F1 of the electronics and comms engineer; ICD 7).
48. **Card G2-FSC (escalation E-06).** Without the implement drive's braking the blades take about 112 s to run down to 10 % speed (structure engineer, finding 8.1).
49. **Cards G2-DRV and G2-INT (review R-37, DRV-05).** The hill-start roll-back: since D66 every restart on a slope unparks with no holding torque; REQ-SYS-112 sets 0.05 m (drive engineer), not met today (the functional safety engineer, step 17).
50. **Card G2-DRV.** The 750 W option has not been run through `t_drive` (drive engineer).
51. **Card G2-DRV, with G1-04 (review R-57).** T5 and T6 use the former motor's rated point and wait for the motors' real rating (drive engineer).
52. **Card G2-DRV, with G1-04.** The 1 kW unit's ESC rating is above the as-built ESC's estimate (drive engineer).
53. **Card G2-PWR.** The shift tests (`shift_cycle_T20_45C`, `shift_cycle_T20_50C`, `shift_bench_speed`) and the 4 h end-of-life tests test criteria that D101 supersedes (D45, D46). They are to be restated or retired on G2-PWR, not fixed as defects (BMS and power engineer). Under D114 (2026-11-21) a test that judges a requirement takes the register's value and cites its ID, here REQ-SYS-104's D101 mission.
54. **Card G2-PWR.** The hot-charge tests T-PWR-10 and 10c test the vented pack at 90 A, while D94 makes the sealed pack the default (R-04 decided; BMS and power engineer).
55. **Card G2-STR.** T10, the turning test of `t_structure`: decided by D100 (chi re-identified on T10 with a 1 % self-check, and the gap to Wong accepted until a field turning test). It stays a known failure until then (structure engineer).
56. **Card G2-MCU, with G2-STR.** In every S4 run since 2026-10-23 the deck-lift PWM (`mcu_out10.pwm_duty(2)`) sits at its 0.6 limit from 13.10 s, and the implement draws about 90 W with the blade off (dashboard item decklift; not analysed yet).
57. **Card G2-INT.** S2b, S2spray, S2c, S2d and S5b run only through `run_scenario`; `tScenarios` does not re-score them (integration engineer).
58. **Card G2-VAL (review R-59, ELC-08).** The [DATA] and [EST] tags on the electronics parameters are still to be set (electronics and comms engineer).
59. **Card G2-STR, with the check on G2-INT (the lead systems engineer's ruling of step 19, point 22).** The twin reads the lift position inverted: the structure plant gives `liftPos01` 0 at the top of the travel, and the MCU reads it as a height fraction, 0 at the bottom. So in the twin the deck-height feedback, the tiller depth and D81's raised-deck backstop all read inverted, and the descent table's raised rows apply with the deck low (structure spec §2.11 and §5.4). The plant follows the sensor and the MCU: only the structure twin changes (`buses/buses_structure.m`, `build_structure.m`), the ICD row is then regenerated, and G2-INT checks that a raised deck selects the raised rows. The real potentiometer's direction is set in the MCU's calibration once card G1-04 gives it (structure engineer).

**Open after card T-07 (2026-11-23; the findings register of NASL-SHD-ER-007, section 8; the owner decided at the T-07 review on 2026-11-23, D117 to D122):**

60. **Decided by D117 (2026-11-23), pending on card G2-MCU.** **T07-MODES-01, confirmed as T07-MCU-12: after an ESC command time-out, only a power cycle recovers the robot.** When the drive command is lost at one ESC for 0.2 s while its feedback still arrives, the robot goes to FAULT, and neither the RC reset nor a new arm recovers it. It stays fail-safe, on its brakes. REQ-SYS-002, -064 and -078 expect the deliberate reset and a new arm to recover it. The owner took option a (ER-007 decision 1): a change to the firmware or the ESC, with the MCU changes of card G2-MCU, so that the reset and a new arm recover the robot; until then a power cycle is the recovery (integration engineer, MCU firmware engineer).

61. **Decided by D118 (2026-11-23), pending on card G2-MCU, with the check against the power spec on G2-PWR.** **T07-MODES-05: on a hardware E-stop, K1 opens 0.148 s after the press, not after the 1.5 s stop-category-1 delay.** The E-stop drops the MCU's high-voltage request at once. The power spec counts on the 1.5 s delay to keep the 48 V bus up for the blades' braked run-down, and REQ-SYS-048's as-built note cites a test with K1 closed for 1.5 s. The owner took option a (ER-007 decision 2): K1 stays closed for the 1.5 s delay after a hardware E-stop, so the power spec and REQ-SYS-048's note stand, and the functional safety engineer reviews the change (integration engineer).

62. **Decided by D122 (2026-11-23): each test goes to the owner of its class.** **T07-RUN-01: five known failures now stop with an error in the full run** (ER-007 Table 12): a test-code defect (`testT12SocRth`), the tracking bench that never starts its mission (T2 and T5, review R-30), a missing test file (`tFull3dCloud2Scan`, with `tSEN13` and `tSEN14`), and one not yet classified (`testD37_Latency`). The owner took option a (ER-007 decision 6): `testT12SocRth`, T2 and T5 go to the GNC engineer, T2 and T5 with the bench fix R-30 on card T-03; `tFull3dCloud2Scan`, with `tSEN13` and `tSEN14`, to the sensors and navigation engineer; `testD37_Latency` to the MCU firmware engineer.

63. **Decided by D121 (2026-11-23), pending on card T-02.** **T07-RUN-02: D31 was out of date, so S1's accelerator RTF check is hard-coded BLOCKED.** `scripts/scenarios/scn_S1_rc_drive.m` line 66 sets S1.rtf BLOCKED for want of a C compiler (D31), though the PC has one: the startup reports MinGW64, T-07's SIL builds used it, and Smart App Control is off (item 20, closed). So the accelerator RTF is never measured. The owner took option a (ER-007 decision 5): D121 resolves D31's blocker, and S1's check is to be measured instead, a test change on card T-02 (integration engineer).

64. **Card T-02 (D122).** **T07-RUN-03: the baseline compare does not tell an error from a failure.** It counts a known failure that turns into an error as still failing (test engineer).

**Review items fixed (critfix, 2026-11-15):** R-01 (MCU and drive; the steep-descent residual is item 19), R-02 (the scan fail-safe part), R-03 (the latch and the field part), R-05 (the detection part) and R-34 (the mode manager reads a pivot turn as standstill). What stays open from them: the zone in front of the body (item 21) and the lane-end turns (item 18).

**Accepted spec deviations** (Incomplete with messages, as listed on 2026-10-23): T-PWR-01/02/09 and the precharge timing (T04); T2 step and T5 thermal t63 (T06, D50 gains); T-SEN-07 palm GNSS band (T02 world data: `gnssSvf` = 1 on the aisle centres); the static slope truth includes the contact-compliance pitch (T03). For the drive the baseline of 2026-11-15 gives today's reasons: T02 has the former gains' criterion (D64), T05 and T06 wait for the motors' data (R-57), T05, T08 and T17 need `T06_LONG=1`, and T17 needs a C compiler (resolved by D121, 2026-11-23: the compiler is there, so T17 now needs only `T06_LONG=1` and a rerun; it has not been rerun).

## 7. How to run

```matlab
% open the project (runs shaddad_startup: P, Ts, P_meta, VC_*, 88 Bus_*, cs_*, mpcRowObj)
openProject('C:\Users\aomar\Desktop\Robot project\Shaddad_UGV.prj');

% compile check of the top model (both phases)
load_system('Shaddad_System');
VC_PHASE = 1; set_param('Shaddad_System', 'SimulationCommand', 'update');
VC_PHASE = 2; set_param('Shaddad_System', 'SimulationCommand', 'update');
VC_PHASE = 1;

% S0 power-up / re-arm smoke test (10 s)
r0 = run_scenario('S0');

% S1 RC (SBUS) manual drive (120 s)
r1  = run_scenario('S1');                         % Normal mode
r1s = run_scenario('S1', 'StopTime', 20);         % short run: latencies + first step
r1a = run_scenario('S1', 'Mode', 'accelerator');  % needs the MinGW compiler (installed); runs since Smart App Control is off (item 20, closed); its RTF check stays BLOCKED (item 63)

% autonomous Phase-2 missions (VC_PHASE 2 is set by the scenario)
r4 = run_scenario('S4');                          % greenhouse coverage (about 14 min simulated: 13.7 min on 2026-11-14)
r7 = run_scenario('S7farm', 'StopTime', 70);      % palm aisle with a GNSS outage 20-50 s
r5 = run_scenario('S5');                          % palm patrol + box + pedestrian
R  = run_all_scenarios({'S0', 'S1', 'S6a'});      % several, grouped by world / variant set

% replay a saved run (bird's-eye view) and re-evaluate criteria
shaddad_replay_birdseye();                        % latest work/results/S*.mat
shaddad_replay_birdseye(r1.file);

% regression
res = run_regression('Fast');                     % tagged Fast, about 3 min, HTML + JUnit in work/reports
res = runtests('tests/rc/t_rc_link.m');           % one domain
% the whole tree: runtests('tests', 'IncludeSubfolders', true) takes several hours;
% run it per test class and restart MATLAB between heavy classes (memory)
% long checks: setenv('T06_LONG','1'), setenv('T08_LONG','1'), setenv('T09_LONG','1'), setenv('T17_LONG','1')
% fresh re-simulation of scenario results: setenv('SHADDAD_SCN_RERUN','1'); S3: setenv('SHADDAD_S3','1')

% rebuild (idempotent generators; each closes, deletes and regenerates its .slx, laid out by shaddad_layout at the save)
% everything, dependencies first: build_all in a batch MATLAB only (it closes every loaded model without saving):
%   matlab -batch "openProject('C:\Users\aomar\Desktop\Robot project\Shaddad_UGV.prj'); build_all('All')"
%   scopes 'NoMCU' (default), 'MCU', 'All' ('NoMCU', then 'MCU', then build_shaddad_system)
calls = build_all('All', 'DryRun', true);         % only lists the calls, in any session
% one file family at a time:
build_shaddad_plant(); build_shaddad_companion(); build_shaddad_system();   % integration models
build_sim_monitors(); build_viz_logging();                                  % T16 subsystems
build_shaddad_mcu('model');      % NOT build_shaddad_mcu('all') - it overwrites the T09 units with stubs
```

The scenario scripts set the variant controls through `SimulationInput` (`scn_base_input`), and the base workspace is not changed.
`run_scenario` saves to `work/results/<id>_<time>.mat` (`'Save', false` to skip). `'Solver','ref'` is broken (open problem 12).
RC row test (Phase 1, added 2026-10-28): `phase1_row_test` lets a simulated driver drive crop rows by RC from your inputs and saves every plot page; see `docs/row_test.md` (scenario R1). The live RC drive is scenario L1 (`docs/live_rc_drive.md`), and the test drive D1 is `scripts/scenarios/scn_D1_test_drive.m`.

## 8. Model map

### 8.1 Top-level models (`models/`)

| File | Purpose |
|---|---|
| Shaddad_System.slx | Top model (ICD 3.1, model layout spec 5). The root input ScenarioIn (In Bus Element) and 11 components joined by direct lines (design 2026-11-08), each a subsystem with one input port per bus that feeds it, named after that bus, and one output bus (Viz_Logging has none; monitors ends in Term_monitors): Operator_Console (VC_OPSRC), Operator_RC (RC_Link_Sim), MCU (MCU_In, Model Shaddad_MCU), BMS (Model Shaddad_BMS), CAN1_Net (CAN1_Net_Sim), Plant (Model Shaddad_Plant), Sensors (Shaddad_SensorModels), CAN2_Link (Link_MCU_GNC), GNC (MissionCmd_RT, GNC_Variant: VC_PHASE 1 stub / 2 Shaddad_Companion), Viz_Logging, Sim_Monitors. A component picks each signal it needs from the port named after its source (e.g. element `truth.pose2D` of port `plant`) right in front of the block it feeds. Built by `build_shaddad_system.m` (T01, T16). |
| Shaddad_Plant.slx | All Simscape networks: Structure, Power, Drive, Electronics (ICD 2, 3.2). Built by `build_shaddad_plant.m`. |
| Shaddad_MCU.slx | STM32H743 firmware, the development target (D96): tasks 1k/5/10/50/100 ms, IO_In/IO_Out variants (VC_TARGET), CAN1/CAN2 codecs (T08, T09). |
| Shaddad_BMS.slx | The twin's STM32G4 BMS firmware, a variant of the delivered BMS (D96): SOC/SOH EKFs, contactor/precharge FSM, balancing, CAN (T04). |
| Shaddad_Companion.slx | Companion computer (Phase 2): NavFusion + Perception + GNC + companion_io (D38). |
| Shaddad_NavFusion.slx | Multi-rate EKF fusion of RTK GNSS + IMU + odometry + LiDAR scan match in the MATLAB System `sens_nav_ekf` (the fallback of D63, taken in T12), Bus_NavState (T12). |
| Shaddad_Perception.slx | LiDAR scan filter, obstacle extraction, VFH sectors, occupancy, scan matching (T13). |
| Shaddad_GNC.slx | Mission manager FSM, path server/analyser, Pure Pursuit/Stanley/MPC trackers, speed governor, avoidance, replanning, implement manager, command shaping (T14, T15). |

### 8.2 Subsystem References (`subsystems/`)

| Folder / file | Purpose |
|---|---|
| **structure/** (T02, T03) | |
| structure_plant.slx | Structure domain top (ICD 3.2): sprockets and implement shaft in; truth, tracks, implement mechanics and diagnostics out |
| structure_hifi.slx / structure_lofi.slx | Fidelity variants (VC_STRUCT_FID): HiFi 6DOF with 2-D track shear and contact, LoFi differential-drive kinematics |
| structure_body6dof.slx | Simple Variable Mass 6DOF (NED/FRD) |
| structure_contact.slx | 14 contact points: kinematics, terrain lookups, normal forces; plus the mower's two gauge wheels (card G2-DSC, D81) |
| structure_track_side.slx / structure_track_shear.slx | Sprocket/belt network per track; 2-D shear states and traction (D39) |
| structure_truth.slx | NED/FRD -> ENU/FLU truth (Bus_TruthState), LLA (D62) |
| structure_env.slx | Ambient conditions from P.world (Bus_EnvState) (T02) |
| structure_implement.slx + structure_impl_none/mower/tiller/sprayer.slx | Implement load mechanics on Impl_Shaft (VC_IMPL 0-3) |
| structure_gauge_points.m, structure_gauge_contact.m | The mower's two caster gauge wheels of the rigid deck, HiFi contact (card G2-DSC, D81; `P.structure.contact.gw`) |
| **power/** (T04, T05) | |
| Power_Plant.slx | Power domain top: battery, pack thermal, PDU, safety chain, LV, implements, dock charger, sensors |
| Pack_Grouped.slx / Pack_Lumped.slx | The 13S15P NMC pack (Battery Builder), VC_PACK 1 grouped (balancing) / 2 lumped |
| Shaddad_Battery_lib.slx / Shaddad_Battery_Lumped_lib.slx | Generated Simscape Battery libraries for the two packs |
| Pack_Thermal.slx | Pack air node, fan, dock air (the vented variant, D94), solar load |
| PDU.slx | Contactors K1-K4, precharge, fuses, shunt, the only electrical reference of DC48 |
| Safety_Chain.slx | Dual-channel E-stop chain, STO, re-arm (D30) |
| LV_Distribution.slx | 48->24/12 V and 12->5 V DC-DC converters, rails, UVLO |
| Dock_Charger.slx | Dock contacts and CC-CV charger (VC_CHG): the twin's dock, as the robot has none in this phase (D97) |
| Impl_None/Mower/Tiller/Sprayer.slx | Implement electric drives (VC_IMPL), network tie in Impl_None |
| impl_ecu.slx | Implement drive controller (IMPL_CMD/FB CAN) |
| Power_Sensors.slx | Measurement and energy integrals -> Bus_BMS_Meas, Bus_PowerStatus, Bus_PowerTruth (D40) |
| bms_can_pack/unpack.slx, bmsreq_can_pack/unpack.slx, implcmd_can_pack/unpack.slx, implfb_can_pack/unpack.slx | CAN1 codecs (ICD 4) |
| **drive/** (T06) | |
| drive_system.slx | Both traction sides, DRV_CMD/FB codecs and rate transitions |
| drive_side.slx | One side: ESC, plant variant, gearbox, brake, thermal |
| drive_esc.slx | ESC firmware: CAN watchdog, DS402 state machine, speed loop, torque limiter, protections (Feedback and Torque_Limiter are the MATLAB Functions `esc_feedback` / `esc_torque_limit` since 2026-11-14, D70, bit-identical to the former block networks) |
| drive_plant_sys.slx / drive_plant_ideal.slx / drive_plant_foc.slx | Motor plant variants (VC_DRIVE_FID 1 system level, 3 ideal, FOC bench only) |
| drive_gearbox.slx, drive_brake.slx, drive_thermal.slx | 20:1 gearboxes (two per side, modelled as one) with mesh loss, spring brake, winding/housing thermal network |
| drive_cmd_pack/unpack.slx, drive_fb_pack/unpack.slx | DRV_CMD / DRV_FB1-3 codecs |
| **rc/** (T07) | |
| RC_Link_Sim.slx | Operator_RC: transmitter -> radio -> receiver -> SBUS encoder -> UART line (1 ms) |
| rc_transmitter / rc_radio_channel / rc_receiver / rc_sbus_encoder / rc_uart_line_sim.slx | Link stages (frame period 14 ms / 7 ms inside RC_Link_Sim only) |
| **electronics/** (T08, T09, T10) | |
| fw_sbus_decoder, fw_rc_mapping, fw_imu_tilt, fw_adc_scaling, fw_telemetry, fw_can1_codec, fw_can2_codec | MCU platform units (T08) |
| fw_io_in_sim / fw_io_out_sim / fw_io_in_hw / fw_io_out_hw | MCU IO variants (VC_TARGET 0 SIM / 1 STM32 blocks) |
| fw_mode_manager, fw_safety_supervisor, fw_vehicle_ctrl, fw_implement_aux, fw_watchdog_mgr | MCU control units (T09); the descent cap (`Descent_cap`, in fw_vehicle_ctrl) and the descent trip (`Descent_trip`, in fw_safety_supervisor), card G2-DSC |
| can2_*_pack / can2_*_unpack (10 files) | CAN-FD MCU <-> companion codecs (ICD 5) |
| Electronics_Plant.slx + elp_hw_frontend, elp_actuators, elp_elec_env, elp_lv_demand, elp_thermal, elp_watchdog_reset | Electronics plant: DI/ADC front end, DO/PWM to the power hardware, enclosure thermal network, LV demand, IWDG (T10) |
| CAN1_Net_Sim.slx / Link_MCU_GNC.slx | CAN1 bus and CAN2 link simulation in Shaddad_System |
| companion_io.slx / companion_io_ros2.slx | Companion CAN-FD IO (SIM; HW via ROS 2 /can2, model reference) |
| **sensors/** (T11, T12, T13) | |
| Shaddad_SensorModels.slx | Sensor plant: IMU (SSR), GNSS and LiDAR (inline), spray vision |
| sens_imu_model / sens_gnss_model / sens_lidar_model / sens_spray_vision | Sensor models (mspsensorlib, gpsSensor, FAST2D/FULL3D LiDAR) |
| sens_acq_imu / sens_odom_compute | MCU-side IMU acquisition and odometry (Shaddad_MCU tasks 5 / 10) |
| sens_acq_gnss / sens_acq_lidar | Companion acquisition (HW ROS 2 / SIM choices) |
| sens_scan_filter.slx | LiDAR scan filter (T13) |
| **gnc/** (T14, T15) | |
| gnc_InputConditioning, gnc_MissionManager, gnc_PathServer, gnc_PathAnalyzer, gnc_PathTracker, gnc_SpeedGovernor, gnc_LocalAvoidance, gnc_GlobalReplanner, gnc_ImplementManager, gnc_CommandShaping, gnc_StatusMetrics | GNC units in Shaddad_GNC |
| gnc_descentCap.m, gnc_descentLimit.m, gnc_descentEnvelope.m | The descent limit of the speed governor and the CommandShaping envelope (card G2-DSC, D74) |
| **integration/** (T16) | |
| Sim_Monitors.slx | Collision/clearance (exact footprint distances from the MATLAB Functions `mon_sep_rect` / `mon_sep_circle` in the For Each blocks FE_Box, FE_Cyl and FE_Dyn since 2026-11-14, D70, bit-identical to the former block networks; robotslcollision only as the reference oracle in t_framework), energy audit, KPIs, NaN guard, stop conditions |
| Viz_Logging.slx | Live visualisation (VC_VIZ: none / bird's-eye MATLAB System / XY Graph / live 3-D robot, `shaddad.Robot3DView`) |

### 8.3 Code, data and tests

| Path | Purpose |
|---|---|
| params/shaddad_params.m, param_<domain>.m, shaddad_configsets.m | P, Ts, P_meta, VC_*; config sets cs_SimFixed/cs_SimRef/cs_GenARM/cs_STM32H7/cs_STM32G4/cs_Companion/cs_Shaddad |
| buses/buses_<domain>.m, shaddad_buses.m, bus_ic.m, enum classes | 88 Bus_* objects, IC structs, enumerations (GncState, ShaddadMode, ...) |
| scripts/shaddad_startup.m / shaddad_shutdown.m | project startup / shutdown |
| scripts/build/build_*.m, shaddad_bu.m | idempotent model generators (one per file family; `build_all` runs them all, dependencies first) and build helper; `shaddad_layout*.m` the layout engine its save runs |
| scripts/run_scenario.m, run_all_scenarios.m, scripts/scenarios/scn_S0..S8_*.m, scn_R1_row_test.m, scn_L1_live_drive.m, scn_D1_test_drive.m | scenario runner and the S0-S8, R1, L1 and D1 definitions (architecture section 12) |
| scripts/gnc_compileMission.m, gnc_designMPC.m, scripts/missions/*.json | mission compiler, MPC design, mission files |
| utilities/ | world builders (world_*), interface/rate checks (shaddad_check_interfaces, shaddad_rate_audit), ICD report, scenario helpers (scn_*), KPIs (kpi_coverage, kpi_truth_xte, eval_criteria), replay (shaddad_replay_birdseye, +shaddad/BirdsEyeView), the MATLAB Function equivalence harness (mf_equiv, D70), utilities/stm32 (target sets, .ioc) |
| tests/<domain>/t_*.m, tb_*.slx | matlab.unittest classes and benches per domain; tests/run_regression.m; `tests/test_baseline.json`, the known failures with their reasons (Current state). `tests/integration/t_mf_equiv.m` (6 Fast tests, 2026-11-14): the D70 functions are bit-identical to the former blocks, compared against the snapshots in `work/agentic/snap/mfplan/task-2/` and `task-3/` (keep them; without them the two equivalence tests are skipped as incomplete), and each block's copy of its `.m` is current |
| work/ | cache folder (slprj, accelerator targets), results (work/results), reports (work/reports), T04/T05 result caches |
| docs/ | build plan, architecture, ICD, specs, build_issues.md, hw_bringup.md, this file |

## Closed and overtaken problems (old numbers kept)

Moved here from section 6 on 2026-11-18 (card G1-01 part 2, step 15; 16b and 23 in its fix pass, step 19; 20 on 2026-11-23, card T-07), each with its evidence. The earlier text follows each one, for the record.

1. **Closed 2026-10-23 (the fix pass, section 0).** S1 passes 12 of 12, and since D70 the fix is line 91 of `subsystems/drive/esc_torque_limit.m` (`tau_avail = local_min(tau_th_mot, tau_dcm)`). Earlier text: **T06, `drive_esc/Torque_Limiter` (`scripts/build/build_drive.m` line 642 when this was written; fixed in the fix pass, section 0; since D70, 2026-11-14, the logic is the MATLAB Function `subsystems/drive/esc_torque_limit.m`, where line 91 holds the fix, `tau_avail = local_min(tau_th_mot, tau_dcm)`).** FB2 `tau_avail` is wired from `TauMot = min(tau_lim_mot, TauThMot, TauDcm)`. That value contains the limit the MCU commanded, while the MCU commands `tau_lim_mot = min(T_peak, tau_avail, T_pow)`. Both start at 0 and stay there (zero fixed point). Fix:
   `MINMAX(s, 'TauAvail', 'min', 2); L(s, 'TauThMot/1', 'TauAvail/1'); L(s, 'TauDcm/1', 'TauAvail/2'); L(s, 'TauAvail/1', 'tau_avail/1');` (the builder code of the time)
   in place of `L(s, 'TauMot/1', 'tau_avail/1')`. Then rebuild the drive files and re-run `t_drive`. The alternative fix on the T09 side is to drop tau_avail from `fw_vehicle_ctrl/Torque_Limits/Mot_min`.
2. **Closed 2026-10-23 (the fix pass, section 0).** `scripts/build/build_shaddad_mcu.m:373` wires `Mode_Manager/1 -> Is_auto`, and the GNC enters AUTO_MISSION in S4 and S5. Earlier text: **T08, `scripts/build/build_shaddad_mcu.m` lines 372-373 (line 329 when this was written).** The Compare To Constant `Task_10/Is_auto` (mode == 6) has no input, so `Bus_McuStatus.drive_ready` is always false and the GNC never leaves IDLE. Fix: add `L(t, 'Mode_Manager/1', 'Is_auto/1');` after the CMP line and run `build_shaddad_mcu('model')`. Do not run `build_shaddad_mcu('all')` or `build_fw_platform()` with the default list: they overwrite the five T09 units with stubs (T09 -> T08 issue, still open).
3. **Closed 2026-11-14; the lane-end holds that came with D72 are item 18.** The full S4 of 2026-11-14 completes: mission_done 1, coverage 0.971 and scan match 0.94, with only short 2 s PERSON holds (`work/agentic/logs/20261114_s4fix_s4check.log`). Earlier text: T13/T15: `Bus_Perception.protectiveStop` and `person_near` fire on static walls in greenhouse door transits and headland turns, so the robot goes to OBSTACLE_HOLD (PERSON) at 25.6 s in S4. T15 reported it with 89 + 15 offending poses. Static-map masking or a footprint-aware field is needed.
6. **Closed 2026-11-14.** The full S4 of 2026-11-14 accepts 0.94 of the scan matches; the 100 s S4 of 2026-11-16 gives 0.815 against > 0.8, which item 31 watches. Earlier text: T13: greenhouse scan-match acceptance is 78.6 % (> 80 % required). T13 also notes that along-aisle errors reach 0.13 m.
8. **Closed 2026-10-23 for the injected errors (section 0: S6c 6 of 6, S6d 4 of 4; both passed in the baseline of 2026-11-15); overtaken by review R-47, item 41.** Earlier text: S6c: corrupted SBUS bytes reach the MCU as valid frames (channel glitch 1.25 FS while 1912 frames are rejected). The UART line simulation should mark corrupted bytes with parity/frame errors (T07), or the decoder should check them (T08).
9. **Closed 2026-10-23 (section 0: S8a 6 of 6; S8a and S8b passed in the baseline of 2026-11-15).** The ESC's undervoltage latches only while the power stage is enabled (`build_drive.m`). Earlier text: S8a: after HW E-stop, release and reset, the MCU goes FAULT because the ESCs report undervoltage after K1 opened, so there is no restart (T09/T06/T04).

16b. **Closed 2026-11-14 by D67: the park regression found with the yaw-loop fix; the check on the robot stays open as item 16b.** The full S4 of 2026-11-14 then ran with no pause in the whole mission (item 16e). Earlier text, for the record: **Open regression, found 2026-11-14 by the nav settling run.** With u = 0 the ESCs park (drive spec 2.9) after `t_park` = 2 s at zero command. The next command then unparks through ENABLING (about 0.1 s). `drive_ready` (ICD: both ESCs OPERATION or READY) drops for that 0.1 s, and the GNC's T36 turns it into a PAUSE with OPERATOR that never resumes on its own. S4 stops at 15.4 s, and every AUTO mission that stops for 2 s or more is affected. Before D66 the yaw loop's residual output kept the ESCs from parking. The proposed fix is a GNC debounce of T36. Evidence: `work/agentic/logs/20261114_s4fix_lead_parkcheck.log`, `work/agentic/s4fix/nav/report.md`. **Fixed 2026-11-14 by D67:** T36 now waits 0.5 s of `drive_ready` loss; the 70 s closed-loop S4 had 0 OPERATOR pauses through three unparks (`work/agentic/s4fix/gnc/report.md`).

19. **Closed 2026-11-16 (card G2-DSC, D74; section 5.3).** The descent cap limits the speed target to the cap less 0.10 m/s on every slope, in every mode, and every brakes-locked stop meets the park criterion: it starts within V_lin(held pitch) + Δv and meets the pitch bound (b). At a breakpoint crossing the step cap can fall below the speed the robot already has, so a park can start near the entry of the bin just crossed. Example: S2c park 2 (sprayer, CCL 0) at 0.787 m/s and a held 15.2 deg, against a 0.25 m/s cap in that class; margins 0.056 m/s and 2.1 deg. The no-implement E-stop run (S2d, step 12d, on the 0.75 deg table) came to its E-stop at 1.14 m/s, below the ESC's own trip of about 1.93 m/s. The system run S2c no longer pitches over without regeneration. The stop-go with the latch is the owner's choice (2026-11-15, "Keep the latch"). Remaining: the remote E-stop with CCL 0 (item 39). Earlier text, for the record: a steep descent with CCL 0 relied on the MCU's speed-follow OVERSPEED (bit 16, D71), because the ESC time-out catches a stop only up to about 10 deg (drive, 2026-11-15). Above that the ESC overspeed trip (about 1.93 m/s) comes first. A time-out short enough for 20 deg (under 0.5 s) would cut into normal stops, since a QUICK_STOP from 1.41 m/s takes about 0.9 s. The speed-follow term parks earlier, but the integration run S2c (2026-11-15; 14.6 deg, CCL 0) shows the next problem: parking at 1.175 m/s stops the tracks in 0.05 s and in the model the robot pitches over its nose (pitch 15 to 86 deg, ends inverted). Before D71 the brake came only at the ESC trip (about 1.9 m/s). The user chose on 2026-11-15: the structure engineer first checks whether the pitch-over is real (track grip against the tip-over margin; the chassis model has no body-ground contact), then a downhill speed limit whenever the battery cannot take charge.

20. **Closed 2026-11-23 (card T-07).** The project owner turned Smart App Control off on 2026-11-22, at about 23:09, and Windows reports it off (`work/team/t07/ledger.md`). SIL and accelerator runs work: `t_perception/tPER05_TebVfh` passes in the full run of 2026-11-23 (`work/team/t07/logs/full/t_perception.log`). Earlier text, for the record: **The user's call (an environment setting, no card); it limits card G2-INT's speed plan.** Windows Code Integrity blocks unsigned MEX files (environment problem, the user's call). It blocked `work/tb_per_tebvfh_msf.mexw64` (unsigned, built 2026-11-04), so `tPER05_TebVfh` fails on this PC; that bench does not use protectiveStop. Any accelerator-mode run may fail the same way for every engineer. It is a security setting and was not changed.

23. **Closed 2026-11-15 (the critical fixes; the review's RID-G1T-03).** The allowance of `fw_safety_supervisor` is 646 crossings, with its reason (`utilities/shaddad_layout_allowance.m:1026`: +194 for the new inputs 3 and 4, R-01 and R-05). The supervisor has measured 564 to 568 crossings since (`work/agentic/logs/20261115_descent_mcu_checks.log`, `work/agentic/logs/20261116_descent_mcu_step9b.log`), and `t_layout/testDomainLayout(domain=mcu)` passes (2026-11-16, `work/agentic/logs/20261116_descent_integration_step12_toplevel.log`). The test baseline of 2026-11-15 still counts that test as failed until it is rebuilt. A tidier placement of inputs 3 and 4 is an improvement for card G2-MCU, not an open problem. Earlier text, for the record: **Card G2-MCU.** Minor layout (MCU, 2026-11-15): the supervisor's inputs 3 and 4 (`stop_fail_prev`, `vtrack_ref_prev`) sit far from their consumers, so `fw_safety_supervisor` has 646 crossings against an allowance of 452. The allowance is open.

24. **Closed 2026-11-16 (card G2-DSC, the D71 speed-follow term; review finding 1).** The reference of each track is held at the lowest speed seen while the speed is above it (rf = max(|ref|, min(rf_prev, |v|))), so a regen-limited deceleration no longer trips; a speed that rises again by 0.2 m/s for 0.2 s still does. The bench runs T-EL-56 and T-EL-57 use CCL 0. The remote E-stop with CCL 0 can still trip STOP_FAIL (a): item 39. Earlier text, for the record: with CCL 0 the speed-follow OVERSPEED also trips on ordinary stops (review, 2026-11-15, High). It fires on any track lag above 0.2 m/s in the driving modes; with regen limited to the 120 W allowance, a MANUAL stop from 1.4 m/s on firm ground or a 2-3 deg downhill stays above it for more than 0.2 s, so OVERSPEED latches (class B, reset needed) and the CTRL_STOP parks at once at about 0.5-0.8 m/s. A remote E-stop with CCL 0 can trip STOP_FAIL (a) (coasting about 0.9 against the 1.0 m/s2 bound), so HV goes off. The MCU bench runs with CCL 100 A, so no test shows it. It goes with the regen-limited follow-up of item 19: gate the term during commanded decelerations, or make the stop and drive profiles follow CCL, plus CCL-0 bench cases.

38. **Closed 2026-11-16 by D83 (the owner: 'Adopt 0.75 degrees (Recommended)').** The table allows 0.75 deg of IMU error (structure step 13): 11 entries are lowered by 0.05 m/s and listed in structure §2.11, and the no-implement S2c and S2d were rerun on it (§5.3). Card G2-NAV confirms the bound with real IMU data. Earlier text, for the record: IMU error bound for the table (structure step 8 point 5; the sensors or MCU owner). The table assumes 0.5 deg of IMU error, and HiFi S2c read up to 0.7 deg low on the fillet. At 0.75 deg, four sprayer entries drop by 0.05 m/s: forward 21.5 deg to 0.05 and 22 deg to 0; reverse 10 deg to 1.00 (inside CR-001's envelope) and 21.5 deg to 0. The park criterion inherits the same bound, so only the pitch check (b) tests the physics. The owner decides between the 0.5 deg bound and 0.75 deg.

40. **Closed 2026-11-18 (card G1-01 part 2).** The structure spec is revision C, with both motors in §6.2 and §6.3 (structure, step 10); the decision record D17 is marked as superseded by the as-built record (step 15); and the integration spec's S2 row gives 226 kg (step 12). Earlier text: **Mass figures predate the current mass model** (structure steps 8 and 11; D17). The decision record D17 (469.2 kg curb, 609 kg with the full sprayer, a 118.8 kg pack), the structure spec §6.3 (450 / 590 / 490 kg) and the integration S2 row (600 kg) do not match `params/param_structure.m`: 226 kg with no implement and 316 kg with the full-tank sprayer, with the pack at 18.5 kg [EST] in `params/param_power.m`. The structure engineer re-tabulates §6.3 and reconciles D17. The S2 runs use the current figures.

## History: the acceptance of 2026-10-23 and the later result sections

Sections 0 to 5 and 9 are kept as written, for the record. Since then:
- Simulink Coder and Embedded Coder are installed (2026-11-07).
- A C compiler is installed. Windows Code Integrity blocked unsigned accelerator MEX files on this PC until 2026-11-22 (item 20, closed), so until then the lines below that say accelerator mode works did not hold here.
- The test numbers of section 4 are superseded by the baseline of 2026-11-15 (Current state).
- In places these sections describe the robot before the as-built re-parameterisation of 2026-10-27 (for example S3 as a 4 h shift, now D101).

System acceptance run: 2026-10-23, 00:45-06:40. MATLAB R2026b (26.2.0.3386108) on Windows 11 with 16 GB RAM.

## 0. Fix pass (2026-10-23, after acceptance) — read this first

Sections 1-9 below describe the acceptance run *before* this fix pass. These items are now fixed in the build scripts, and the models are regenerated:

| Item | Fix | Verified |
|---|---|---|
| Torque deadlock (item 1) | `build_drive.m` Torque_Limiter: `tau_avail = min(TauThMot, TauDcm)`, so the commanded limit is no longer fed back | S1 **PASS 12/12** (Normal mode, RTF 0.47) |
| `drive_ready` never true (item 2) | `build_shaddad_mcu.m`: `Mode_Manager/1 -> Is_auto` wired | GNC enters AUTO_MISSION at 15.2 s in S4 and S5 |
| SBUS accepts corrupted frames (S6c) | `sbus_frame_sync.m`: a frame is rejected if it overlaps a UART-error chunk, has a byte after its footer, or has a channel outside 172..1811 | S6c (now 1 % byte errors) **PASS 6/6**; new S6d (5 %, controlled stop) **PASS 4/4** |
| No restart after E-stop reset (S8a) | `build_drive.m` ESC: undervoltage latches only while the power stage is enabled | S8a **PASS 6/6**. The `no_restart` window now starts 2 s after ESTOP entry. |
| HW-variant defects (BMS FDCAN/DI, MCU FDCAN port order, Companion loop) | `build_shaddad_bms.m`, `build_fw_platform.m` (`fw_io_in_hw`), `build_electronics_plant.m` (`companion_io` Unit Delay) | With VC_TARGET 1: BMS and Companion update **OK**. MCU is **BLOCKED** only by the missing STM32CubeCLT. `Shaddad_System` SIM: 0 algebraic loops. |

Phase-2 results after the fix pass (Normal mode):
* **S5 palm patrol, full run: mission completes.** 5 aisles, `mission_done` at 648 s, route time 1.06 x nominal, 0 collisions, XTE RMS 0.019 m. Failures: box clearance 0.40 m (needs >= 0.5), deceleration starts 6.1 s after the box enters the corridor (needs <= 0.5 s), rejoin 6.2 m (needs <= 3 m). The pedestrian never came near the robot (32.7 m), so its timing in the scenario needs retuning.
* **S4 greenhouse, first 120 s: blocked.** It reaches AUTO_MISSION, then enters OBSTACLE_HOLD with reason PERSON at 25.6 s at the greenhouse entrance and toggles between HOLD and AUTO. Coverage 0. Scan-match accepts 89.8 % and the run has 0 collisions.

Still open: the Phase-2 policy issues of section 6 items 3-7 (protective field on static structure, GNSS-outage pause, obstacle bypass timing), stale unit tests (section 4.1), spec deviations, and Normal-mode speed. **Accelerator MEX files are blocked by Windows Application Control on this PC**, so use `'Mode','normal'`.

## 1. Verdict

| Item | Result |
|---|---|
| Build | All 17 build tasks produced their files: 8 models, 117 subsystem/library files and 39 build scripts (`scripts/build/build_*.m`). The integration build scripts regenerate the top-level models idempotently: `t_infra/testBuildScripts` and `t_framework/testBuildIdempotent` pass. |
| Top model compiles | **Yes.** `Shaddad_System` compiles for VC_PHASE 1 and 2. It has 0 algebraic loops (`AlgebraicLoopMsg = error`) and no automatically inserted rate transitions (`AutoInsertRateTranBlk = off`). |
| Regression (`tests/`, all 18 test classes) | 391 tests: **328 passed, 31 failed, 30 incomplete, 2 not run**. Details in section 4. |
| S0 power-up | **PASS**, 11 of 11 criteria. |
| S1 RC (SBUS) drive, delivered files | **FAIL.** RC latencies pass (28 ms, 50 ms), but **the robot never moves** because of two one-line wiring defects (section 6, items 1 and 2). |
| S1 with the two defects fixed in memory (probe, nothing saved) | **PASS**, 12 of 12 criteria. The RTF criterion is still marked BLOCKED by stale scenario text. |
| Phase-2 autonomous mission, delivered files | **FAIL.** The GNC stays IDLE because MCU `drive_ready` is never true (defect 2). |
| Phase-2 with the probe fixes | Starts and tracks: farm aisle truth XTE 0.034 m max, 0.018 m RMS. It then stops. S4 greenhouse goes to OBSTACLE_HOLD (PERSON) at the door. S7 farm goes to PAUSE (NAV_DEGRADED) 4 s into the GNSS outage. See 5.2. |
| Code generation / STM32 builds | **BLOCKED.** Simulink Coder is not installed (D31). STM32CubeMX/CubeCLT and GNU Arm tools are missing. Since then Simulink Coder and Embedded Coder are installed (the startup report lists both OK, 2026-11-07); the STM32 tools are still missing, so the STM32 board builds stay BLOCKED. |

**New since the build tasks ran:** a C compiler is now installed and detected (MinGW64 Compiler (C) 14.2.0 from the MATLAB support package).
Accelerator mode therefore works. Earlier reports that say "BLOCKED: no C compiler" are out of date (section 3).

## 2. Where things stand

* Phase 1 (SBUS remote control) is functionally complete. RC link, SBUS decoding, MCU control, ESC, drive, power and the structure plant all work end to end once the two wiring defects are fixed. The probe S1 run meets every motion, latency, stop-distance and energy-audit criterion.
* Phase 2 (autonomous GNC with RTK GNSS + IMU + encoder + LiDAR fusion) compiles and starts in the system. Mission-level behaviour then stops at the GNC/perception/navigation policy issues listed in section 6 (items 3-7).
* The domain benches are mostly green (section 4). The red benches come from stale test expectations, performance targets in Normal mode, and known spec deviations.

## 3. Environment (measured 2026-10-23)

| Item | State |
|---|---|
| Simulink Coder | MISSING. Every ERT/STM32/SIL/PIL item is BLOCKED (13 Incomplete tests). Installed since: the startup report of 2026-11-07 lists Simulink Coder and Embedded Coder OK. |
| Embedded Coder, STM32 Microcontroller Blockset, ROS Toolbox, Simscape (+Electrical, +Battery) | installed |
| C/C++ compiler (mex) | **MinGW64 14.2.0, OK.** It was not detected when T01-T17 ran, so accelerator mode works now. |
| STM32CubeMX / STM32CubeCLT / GNU Tools for STM32 | MISSING |
| Memory | Each Shaddad_System compile grows the MATLAB process by about 0.5-1 GB, and the memory is not released. Above about 9 GB the PC pages and runs 4-5x slower (Normal-mode RTF 0.12 instead of 0.49). The shared MATLAB session crashed once during the regression (around 02:15, probably out of memory). I then restarted MATLAB (`quit force`, project reopened) between heavy test groups. **Restart MATLAB between long runs.** |

## 4. Regression

The regression ran `runtests` on every test class in `tests/**`. The suite was split into bounded chunks with `matlab.unittest.TestSuite.fromFile(...)` index ranges, because a single MATLAB call may not run longer than 30 min. Results are stored in the session scratch folder.
`run_regression('All')` runs the same suite but takes several hours, and one call cannot fit.

| Test class | Owner | Tests | Passed | Failed | Incomplete | Notes |
|---|---|---|---|---|---|---|
| tests/integration/t_infra.m | T01 | 22 | 14 | 7 | 1 | Failures: stale expectations and one R2026b flake (6.4) |
| tests/structure/t_world.m | T02 | 18 | 18 | 0 | 0 | testBuildScriptsAndPorts needs no model loaded that references structure_env |
| tests/structure/t_structure.m | T03 | 20 | 19 | 1 | 0 | T19_performance now runs with the compiler: HiFi RTF 31.6 (>= 20) PASS, LoFi 67.5 (>= 100) FAIL |
| tests/power/t_pack_bms.m | T04 | 27 | 21 | 1 | 5 | speed_T19 3.8x / 20.7x vs 10x / 100x (cached Normal-mode runs); 4 SPEC-DEVIATION + 1 codegen |
| tests/power/t_power_plant.m | T05 | 26 | 25 | 1 | 0 | shift_bench_speed 14.5x vs 100x (cached Normal-mode run); about 41 min |
| tests/drive/t_drive.m | T06 | 30 | 25 | 0 | 5 | T02 spec conflict (D50); T05/T08/T17 gated by T06_LONG=1; codegen |
| tests/rc/t_rc_link.m | T07 | 13 | 13 | 0 | 0 | |
| tests/electronics/t_mcu_platform.m | T08 | 24 | 17 | 4 | 3 | Tests still expect the T08 stubs that T09 replaced; testD37_Latency throws |
| tests/electronics/t_mcu_control.m | T09 | 35 | 33 | 0 | 2 | rebuild gated by T09_LONG=1; codegen |
| tests/electronics/t_elec_plant.m | T10 | 15 | 15 | 0 | 0 | |
| tests/sensors/t_sensors.m | T11 | 19 | 17 | 0 | 2 | tSEN13/14 need the base variable `sens_uavScene` (fresh session); T-SEN-07 world-data deviation; codegen |
| tests/sensors/t_nav.m | T12 | 24 | 23 | 0 | 1 | codegen |
| tests/sensors/t_perception.m | T13 | 15 | 14 | 0 | 1 | tPER05 Accelerator part now runs and passes. tCompiledTiming failed while the PC was paging and passed on re-run. codegen |
| tests/gnc/t_gnc_tracking.m | T14 | 16 | 10 | 5 | 1 | T2/T3 x2/T4/T5 written against the old MissionManager stub (6.4) |
| tests/gnc/t_gnc_mission.m | T15 | 22 | 21 | 0 | 1 | testT20Accelerator now passes; codegen |
| tests/electronics/t_targets.m | T17 | 21 | 15 | 0 | 6 | 3 builds BLOCKED (Simulink Coder); 3 preflights gated by T17_LONG=1. With Simulink Coder installed since, layout plan Task 9 (2026-10-31) measured 14 passed, 2 failed, 5 incomplete: the Companion build runs and fails in `Shaddad_GNC/GlobalReplanner/Replan/HybridAStar`; the MCU and BMS builds are BLOCKED by the missing STM32 tools |
| tests/integration/t_framework.m | T16 | 30 | 27 | 2 | 1 | testFixedVsRef void (robot not moving); testConventionsFromResults ramp pitch void; op-point BLOCKED (IMU SimState) |
| tests/system/tScenarios.m | T16 | 14 | 1 | 10 | 1 | Evaluates the latest saved result per scenario; S7wet/S7temp have no result and were not run (2) |
| **Total** | | **391** | **328** | **31** | **30** | 2 not run |

The 30 incomplete tests are 13 codegen BLOCKED, 9 long checks gated by environment variables, 6 SPEC-DEVIATION/CONFLICT assumptions, and 2 other BLOCKED (IMU SimState, S3 run time).

### 4.1 Failing tests (31)

| Test | Cause | Owner |
|---|---|---|
| tScenarios/testS1, S2, S4, S5, S6a, S6b, S6c, S7farm, S8a, S8b | The robot does not move (defects 1 and 2). S6c also accepts corrupted frames. | T06, T08 (then T07, T09) |
| t_framework/testFixedVsRef, testConventionsFromResults | void: the robot does not move | T06, T08 |
| t_infra/testStartup, testStartupFreshProcess | expect 87 Bus_* objects, there are 88 (Bus_TebMapCrop, T15) | T01 (stale test) |
| t_infra/testPlantStructure | expects 1 Solver Configuration, there are 2 (Electronics_Thermal network, D15) | T01 (stale test) |
| t_infra/testSystemPhase1, testSystemPhase2 | the updates pass (84.3 s / 61.9 s); the rate check flags the Operator_RC instance block itself as "0.007/0.014 outside RC_Link_Sim" (t_framework testRateAudit excludes it) | T01 (stale test) |
| t_infra/testPlantVariantMatrix | R2026b `Stateflow:Assertion o != nullptr` on the first Shaddad_Plant update; the test does not retry | T01 test / R2026b bug (drive_esc is 3 SSR levels deep, T06) |
| t_infra/testRunScenarioRef | `run_scenario(...,'Solver','ref')` fails (open problem 12) | T16, T03, T10 |
| t_mcu_platform/testFilesAndBuildOutputs, testModelStructure, testTelemetryStatusFrames | expect the T08 stubs of the T09 units | T08 (stale test) |
| t_mcu_platform/testD37_Latency | exception "Non-scalar in Uniform output" with the real T09 units | T08 |
| t_gnc_tracking/testT2RowEntry, testT3DriftMPC, testT3DriftStanley, testT4HeadlandTurn, testT5SpeedStep | the real T15 FSM needs START (or `P.gnc.fsm.auto_start`) and geofence-safe rows (`tests/gnc/gnc_tb_run.m`) | T14 |
| t_structure/T19_performance | LoFi accelerator RTF 67.5 < 100 (HiFi 31.6 passes) | T03 |
| t_pack_bms/speed_T19, t_power_plant/shift_bench_speed | cached Normal-mode RTFs (3.8x / 20.7x; 14.5x); re-measure in Accelerator now that a compiler exists | T04, T05 |

## 5. System compile and scenarios

### 5.1 Top model update (T-INT-01)

| Run | VC_PHASE 1 | VC_PHASE 2 | Criterion |
|---|---|---|---|
| Shared session, about 120 models already loaded (first step of this run) | 195.3 s, OK | 266.0 s, OK | < 180 s: FAIL in a loaded session |
| Fresh MATLAB session (t_infra testSystemPhase1/2) | 84.3 s, OK | 61.9 s, OK | < 180 s: PASS |
| t_framework/testSystemUpdate (fresh session) | PASS | PASS | |

In every run: 0 algebraic loops (`AlgebraicLoopMsg = error`), `AutoInsertRateTranBlk = off`, `MultiTaskRateTransMsg = error`, and compiled rates within `P.sim.Ts_compiled_allowed` (testRateAudit PASS). The 8 points of the T-INT-10 variant matrix compile (testVariantMatrix 8/8).

### 5.2 Scenario runs of this acceptance

Delivered files, Normal mode. Results are saved in `work/results/` (`S0_20261023_035827.mat`, `S1_20261023_040332.mat`, `S4_20261023_041212.mat`).

| Scenario | Result | Key numbers (measured vs criterion) |
|---|---|---|
| S0 power-up (10 s) | **PASS 11/11** | DO0 0.02 s (<= 0.05); DO1 pulse 50 ms (48-52); Safety_OK / Safety_HV_OK 0.061 s (<= 0.2); HV_Ready 0.5 s (<= 1.0); STANDBY 1.01 s (<= 1.5) and at 10 s; class-A bits 0; drift 4.8e-19 m; abs dSOC 1.6e-5 (< 1e-4); RTF 0.121 |
| S1 RC drive (truncated to 20 s) | **FAIL** | TX frame -> DRV_CMD 28 ms (<= 30) PASS; stick -> ESC reference 50 ms (<= 65) PASS; MCU BOOT -> STANDBY 1.01 s -> MANUAL 2.04 s; ESCs in OPERATION; w_ref_motor 229/221 rad/s, but **tau_lim_mot = 0 / 0 N m, max abs(v) = 7e-17 m/s**: rise, ss-error and yaw criteria fail; eps_el 2.1e-14, eps_mech 1.1e-13; RTF 0.123 |
| S4 greenhouse, Phase 2 (truncated to 25 s) | **FAIL** | GNC state IDLE for the whole run; MCU AUTO_START 10.03 s, AUTO 13.03 s; `drive_ready` false (Is_auto unconnected); coverage 0; clearance 1.53 m; scan match valid 5.9 % (> 80 %); truth dSOC 6.7e-5 in 25 s (standby load); RTF 0.110 |

Probe with the two defects fixed **in memory only**. Nothing was saved, and the file timestamps of `drive_esc.slx` and `Shaddad_MCU.slx` were checked unchanged. The edits are those of section 6, items 1 and 2 (scratch helper `acc_probe_fix.m`).

| Scenario | Result | Key numbers |
|---|---|---|
| S1 RC drive, full 120 s | **PASS 12/12** | 90 % rise 1.51 s (<= 2.0); overshoot -0.6 % (<= 5 %); steady error 0.0072 m/s (<= 0.03); yaw-rate tracking 4.9 % (<= 10 %), sign correct (-0.350 rad/s for right steer); TX -> DRV_CMD 28 ms (<= 30); stick -> ESC 50 ms (<= 65); stop distance from 1.2 m/s 0.845 m (<= 0.95); reverse speed error 0.0038 m/s (<= 0.05); eps_el 6.1e-14, eps_mech 4.4e-5 (<= 1 %); 0 collisions. Peak speed 1.192 m/s. Truth SOC 0.958540 -> 0.957498 (0.104 % in 120 s), E_out 16.6 Wh, I_pack peak 23.5 A. **RTF 0.494** (Normal, fresh session) |
| S1, 20 s, `'Mode','accelerator'` (top model accelerated, Model blocks Normal) | criteria equal to Normal within 1e-4 | exec 18.7 s for 20 s: **RTF 1.07**; build and compile about 6 min. T-INT-03 needs >= 5 with the Model blocks accelerated too; that was not tried. |
| S4 greenhouse, 60 s | FAIL (no progress) | AUTO_MISSION/TRANSIT at 15.2 s; **OBSTACLE_HOLD, reason PERSON, at 25.6 s** (static walls trigger the protective field), toggling AUTO/HOLD at 38-41 s; progress 1.5 % at 60 s; max speed 1.01 m/s; clearance 0.42 m; scan match valid 78.6 % (> 80 %); RTF 0.108 (loaded session) |
| S7farm GNSS dropout, 70 s | FAIL (stops) | AUTO at 15.2 s; before the outage truth XTE max 0.034 m, RMS 0.018 m, mean speed 0.91 m/s; **PAUSE NAV_DEGRADED at 24.2 s** (outage from 20 s), never resumes, so 3.7 m driven (needs >= 20 m); pos error at outage end 0.10 m (<= 0.5) PASS; heading 0.68 deg (<= 2) PASS; XTE in outage 0.025 m (<= 0.3) PASS; error 5 s after RTK return 0.238 m (< 0.05) FAIL; NEES in bound 78.7 % (>= 90 %) FAIL; RTF 0.433 |

Scenarios not re-simulated in this run: S2, S5, S6a/b/c, S7farm and S8a/b were last simulated by T16 on 2026-10-22/23 and are evaluated by `tScenarios`; every one fails because the robot does not move. S3 (4 h BOL/EOL/hot) and S7wet/S7temp were never run. See T16's notes for S6/S8, also copied to section 6.

### 5.3 Descent speed cap (card G2-DSC, 2026-11-15 to 2026-11-16)

Card G2-DSC built the always-on speed cap (D74), the sprayer's operating limit on descents (D75, CR-001) and the mounting and deck rules (D81). The runs below are the integration engineer's step 12, 12b and 12d system runs, made after the structure, MCU and GNC steps. The two no-implement runs on 20 deg (S2c, S2d) were rerun in step 12d on the 0.75 deg table (D83); their rows show the step 12d numbers. Every brakes-locked stop meets the structure park criterion (`docs/specs/structure.md` §2.11), and no run tips over, sets STOP_FAIL or has an ESC fault. Step reports: `work/agentic/descent/`.

| Run | Configuration | Result | Key numbers |
|---|---|---|---|
| S2 | no implement (226 kg), 20 deg | 11 of 12; `testS2` fails on S2.v_mean only | mean ramp speed 0.725 m/s against 0.75 (a known deviation: the traction demand on a slope does not depend on the mass); pitch error −0.0098 rad (pass); `no_overspeed` pass; I_peak 21.4 A; roll-back in the hold 0 mm |
| S2b | no implement, 20 deg, RC link lost 1 s after t_rel | 11 of 11 | roll-back 1.7 mm; park 0.50 s; STANDBY 0.61 s |
| S2spray | full-tank sprayer (316 kg), 18.5 deg | 10 of 12: the two known deviations | mean ramp speed 0.735 m/s against 0.75 (a known deviation); pitch error 0.0227 rad against 0.02; `no_overspeed` pass |
| S2c | sprayer, 18.5 deg, CCL 0, 14 re-arm cycles | 10 of 10 | 2 OVERSPEED parks (14 in the earlier run at 20 deg), both passing the park criterion, then 12 operator stops; the sprayer holds the capped 0.15 m/s; closest stop: park 2, 0.787 m/s against 0.843 (margin 0.056 m/s), at a held 15.2 deg against a 0.25 m/s cap in that class; its pitch margin is 2.1 deg (10.6 against 12.7 deg) and its rear stations lift 8.6 cm |
| S2c, no implement | 20 deg, CCL 0, 14 cycles (step 12d, on the 0.75 deg table; step 12b was on 0.5 deg) | 11 of 11 | 15 OVERSPEED parks at 1.21 to 1.29 m/s; worst margins (a) 0.220 m/s (park 2: 1.269 against 1.489 m/s), (b) 10.7 deg; the run's worst speed margin (run_v) 0.250 m/s. Only park 2 came earlier (0.02 s). The other 14 are not the descent trip: they are speed-follow trips (inferred by elimination from their trip times and speeds, not traced) |
| S2d | sprayer, 18.5 deg, hardware E-stop at 40 s | 9 of 9 | E-stop at x 25.08 m: STO 15 ms, brakes 0.09 s, stop 0.024 m, held 1.3 mm; park (a) 0.162 m/s against 0.501, (b) 2.2 deg against 11.3 |
| S2d, no implement | 20 deg, throttle 1.2 m/s, E-stop at 18.8 s (step 12d, on the 0.75 deg table) | 9 of 9 | no OVERSPEED before the E-stop. The E-stop falls on the straight at the capped speed: x 20.42 m, terrain 19.97 deg, speed 1.14 m/s (truth), with the target at the cap (1.05 to 1.10 m/s) throughout the 2 s before it. STO 15 ms, brakes 0.09 s, stop 0.280 m (limit 0.6 m), held 1.8 mm; park (a) 1.142 against 1.457 m/s, margin 0.315 m/s; (b) 2.92 against 14.82 deg, margin 11.90 deg |
| S4, 100 s | greenhouse, Phase 2 | scan match 0.815 (limit > 0.8); otherwise as before | the deck-down gate holds the robot from 15.20 to 21.55 s (IMPLEMENT_NOT_READY, t_deck 6.35 s); first motion at 21.89 s (was 15.54 s); the two PERSON holds come 6 s later; no OVERSPEED or tilt edge (item 31) |
| S5b | palm patrol with obstacles | 13 of 13 | identical to before |
| Top level | `Shaddad_System` | compiles with 0 warnings; 0 algebraic loops; 0 interface mismatches | `testDomainLayout` 11 of 11; `test_baseline.py compare`: 0 new failures |

**Hill start.** The roll-back after the brakes release does not trip OVERSPEED. In S2spray the roll-back is 0.061 m/s (truth) and 0.080 m/s (encoder) against a trip of 0.20 m/s at a held 18.9 deg, a margin of 0.12 m/s. The case that separates the old rule from the window (the sprayer on 20 deg, outside CR-001) was replayed on the morning S2 log: the rule before the window trips at 61.72 s (prediction 0.159 m/s against a 0.15 m/s trip at a held 20.5 deg), while the measured speed peaks at 0.090 m/s. The window removes that trip. The structure margins are in `docs/specs/structure.md` §2.11 (the tightest in the envelope: +0.085 m/s at 0.5 deg, about +0.06 m/s at 0.75 deg, D83).

**Bench, MCU (CCL 0, 226 kg; T-EL-54 to T-EL-61).** Before the change a MANUAL stop from 1.4 m/s on 4 deg would have latched OVERSPEED 0.58 s after release and parked at about 1.3 m/s, and an AUTO stop from 1.2 m/s on 4 deg would have latched at 0.69 s. After it, neither stop trips: MANUAL stops in 2.54 s on 4 deg and 1.58 s on flat ground; AUTO stops in 1.76 s on 4 deg. The sprayer's CCL-0 descent (T-EL-58, moved from 19.5 to 14.5 deg in step 9) is capped, with stop-go parks and no STOP_FAIL. The tiller's reverse target is 0 at every pitch. The steep descent and the speed-follow term are items 19 and 24.

**GNC (`t_gnc_mission`, step 10).** 37 tests: 35 pass, and the two known failures (testT7BypassTeb, testT12SocRth) are as before; all five descent tests pass. The governor at the bench nav pitch gives 0.95 / 0.60 / 0.15 / 0.10 / 0 m/s for the sprayer at 10 / 15 / 20 / 20.25 / 21.2 deg. The look-ahead on world 2 slows the sprayer to 0.152 m/s at the 15 deg point (cap 0.15), then holds 0.150 m/s on the ramp, in AUTO throughout. Baseline compare: 0 new failures.

**Known deviations left in this pass.** S2.v_mean (0.725 against 0.75, no implement), S2spray.v_mean (0.735) and S2spray.pitch (0.023 rad against 0.02). The full-stick nuisance trip is item 29.

## 9. What this acceptance changed

* Wrote this file. No other project file was changed. A trial change in `scripts/run_scenario.m` (keep the Electronics_Thermal local solver in 'ref' mode) did not fix open problem 12 and was reverted.
* New scenario result files in `work/results/` (`S0_20261023_035827`, `S1_20261023_040332` (20 s), `S4_20261023_041212` (25 s)); `tScenarios` now evaluates these as the latest S0/S1/S4. The `T01ref_*` / `T01smoke_*` files come from `t_infra` runs.
* The regression rebuilt the integration models (`t_infra/testBuildScripts`: Shaddad_Plant, Shaddad_Companion, Shaddad_System at 05:41-05:43) and Sim_Monitors/Viz_Logging (`t_framework/testBuildIdempotent`), as those tests are designed to do.
* The probe edits of drive_esc and Shaddad_MCU were made in memory only and discarded by restarting MATLAB; both files keep their 2026-10-22 timestamps.
