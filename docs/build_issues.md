# Cross-task interface notes from completed tasks (T01, T02, T07)

Builders: read the entries whose target names your task and address them if valid.

## T01 -> T04/T05 (power)

The Unit 'Wh' on elements of Bus_PowerTruth and Bus_BMS_Telemetry is not in the Simulink unit database, so every compile of Shaddad_Plant, Shaddad_BMS and Shaddad_System warns 'Units Wh ... are not defined'. Please change the Unit in buses_power.m to a database unit such as 'W*h' (check it first), then rerun shaddad_buses('check') and shaddad_icd_report.

## T01 -> T10 (electronics)

companion_io output 3 is named 'mcu_status_dec' in the build_stubs table and in the stub, not 'mcu_status' as in ICD 3.1. Input 3 is already called 'mcu_status' (the CAN-FD frame), and Simulink block names must be unique. Keep this name when you replace the stub, or shaddad_check_interfaces will report a mismatch.

## T01 -> T03 (structure)

The ICD gives no Ts for the structure_plant input env_dist. The contract and stub use 0.001 s (Ts.base); Shaddad_Plant drives it from Constant P.ic.Bus_EnvDisturbance at Ts.base. Continuous outputs (truth, tracks, diag) are declared with Ts 0. The checker also accepts the stub's 0.001 s and an inherited (-1) SSR Outport for them.

## T01 -> T16 (integration)

Three points. (1) A bus-object-typed root In Bus Element accepts no SampleTime or Interpolate setting, and its block cannot be named after its PortName. So the root port is block 'In_ScenarioIn' with PortName ScenarioIn and BusVirtuality nonvirtual, followed by Signal Specification 'Spec_ScenarioIn' (Ts.scen, nonvirtual) that pins the 0.01 s rate. scn_make_input writes each ZOH step as two samples at the same time instant, which gives an exactly piecewise-constant input. (2) Simulink does not allow Simscape logging inside the referenced Shaddad_Plant. cs_SimRef keeps SimscapeLogType 'local' as the plan specifies, but run_scenario('ref') sets cs_Shaddad.SimscapeLogType = 'none' for the top-level run. (3) Extra tools now available: shaddad_check_interfaces(..., 'Compiled', true) checks compiled SSR/Model instance ports, shaddad_rate_audit gives the T-INT-05 audit, and shaddad_bu('finalize'/'closedirty') are build helpers. run_scenario is still a skeleton; eval_criteria and the scn_S* files are yours.

## T01 -> T17 (electronics/hardware) and anyone editing params/shaddad_configsets.m

The four codegen config sets are cached in work/shaddad_configsets_cache.mat, keyed on the full text of shaddad_configsets.m and the MATLAB version, because setting the STM32 HardwareBoard is slow. Any edit to that file triggers one cache rebuild of about 115 s on the next startup; afterwards startup takes 1-5 s. shaddad_configsets('fixed'/'ref') now calls refresh() on every loaded Shaddad_Ref ConfigSetRef. Without that, the first update after cs_Shaddad is replaced fails with 'Cannot apply change to solver type since model compilation is underway'.

## T01 -> T17 (electronics/hardware) and anyone editing params/shaddad_configsets.m

Startup no longer sets HardwareBoard on the codegen sets. In a fresh session, shaddad_startup / shaddad_configsets('fixed'|'ref') create cs_STM32H7, cs_STM32G4 and cs_Companion with every field except HardwareBoard. The missing fields are listed in rep.deferred, noted in the set's Description, and shown as DEFERRED in the startup report. The reason is cost: setting HardwareBoard takes 158 s (STM32H7), 80 s (STM32G4) and 4 s (ROS 2) in a fresh process, and even loading the complete sets from the cache costs about 4 s. Call shaddad_configsets('codegen') before any code generation or board build. It loads the complete sets from work/shaddad_configsets_cache.mat (1 s warm, 5 s in a fresh process), or rebuilds them and writes the cache (about 135 s warm, 4-5 min cold). After that, 'fixed'/'ref' calls in the same session keep the complete sets. Use shaddad_configsets('codegen', true) to force a rebuild. The cache key is now the settings data in local_defs plus CACHE_FMT and the MATLAB version, not the file text: edit the D.cg entries in local_defs for your STM32 / ROS 2 fields, and bump CACHE_FMT if you change how the sets are built. The environment variable SHADDAD_CS_CACHE redirects the cache file (tests use it).

## T01 -> T16 (integration)

Four points. (1) The root port is In Bus Element block 'In_ScenarioIn' (PortName ScenarioIn, nonvirtual), followed by Signal Specification 'Spec_ScenarioIn' (Ts.scen); a bus-object In Bus Element accepts no SampleTime. scn_make_input writes each ZOH step as two samples at the same time instant. (2) Simulink does not allow Simscape logging inside the referenced Shaddad_Plant, so run_scenario('ref') sets cs_Shaddad.SimscapeLogType = 'none' for the top-level run. (3) Available tools: shaddad_check_interfaces(..., 'Compiled', true), shaddad_rate_audit, shaddad_bu('finalize'/'closedirty'), and t_infra.fresh(tc, lines), a test helper that runs code in a fresh MATLAB process with the project path, usable for parsim-worker or cold-start checks. (4) run_scenario is still a skeleton; eval_criteria and the scn_S* files are yours.

## T02 -> T03

structure_env.slx is ready for structure_plant: in 1 pose2D double[3] = [xE yN psi_enu] (ENU, yaw from East CCW, SampleTime 0), in 2 speed double (signed forward speed of R, SampleTime 0; |speed| is compared with dock.v_max), in 3 env_ovr Bus_EnvOverride (Ts.env 0.1); out 1 env Bus_EnvState (nonvirtual, Ts.env). It contains its own Zero-Order Holds (0.1 s), so feed the continuous truth directly. P.world obstacle heights (cyl zBase/h, box zBase/lz) are heights above the local ground.

## T02 -> T16

Empty obstacle lists hold one all-zero dummy row with n* = 0 (e.g. palm_farm has nBox = 0 and box = zeros(1,9)). scn_S5 must replace that dummy row instead of appending: if nBox == 0, set box = newRow, else append, then nBox = nBox + 1. Then rebuild the grid with P.world.occ = world_occupancy(P.world); it returns the whole occ struct {res, origin, grid}. Consumers must only read rows 1..n*.

## T02 -> T11

P.world.gnssSvf and P.world.gnssZoneId are built for every world, sized like the terrain grid: Table(i,j) <-> (xE(i), yN(j)). Canopy attenuation is applied as a plain multiplication by canopyAtt inside each canopy circle, as the build plan says, not the 1.3 R ramp of sensors spec F1. world_gnss_grids('defaults') and world_occupancy('defaults').zBand copy P.sensors.gnss.svf and P.sensors.lidar.zBand / mapRes because P.world is assembled before P.sensors. If you change those sensor values, the world-side copies must change too. tests/structure/t_world asserts they are equal.

## T02 -> T13/T14

Each P.world.occ is a 10 cells/m grid with row 1 = max y. Solar tables have zBase 0.8 m, below the top of the 0.15-1.20 m band, so they are marked occupied. The greenhouse film segments sit on the inner wall faces (x = 0 / 9.6, y = 0 / 37.5) and are buffered 0.05 m, so they reach 0.05 m into the house; the wall boxes lie outside the 9.6 x 37.5 m footprint. Lanes: palm_farm has 6 aisles at x = 8k+8, y 8..112. greenhouse has 1 row at x = 4.8, y 0..37.5; the mission compiler applies the c_in = 0.45 m inset (D54). solar_farm has 6 aisles at y = 7k+3.5, x 0..40. soil_strips has 5 lanes at y = 10k-5, x 0..40.

## T07 -> T08

RC_Link_Sim timing for your bench and D37 budget. The sbus port carries a frame sampled at TX time k*Tf from (k+1)*Tf. Its first byte arrives at +15 ms (14 + phase0 0.37 ms + 0.12 ms, then the next tick) and its last byte at about +18 ms. With the default phase0 0.37 ms and 200 ppm drift, chunks are [3..5 8/9 8/9 4..5], not [8 8 9] (that pattern needs phase0 = 0). rc_frame is the receiver frame through the same 14 ms deterministic RT, so it is aligned with the bytes. Reference codec: bytes = sbus_ref_encode(ch Nx16, flags Nx4 logical [ch17 ch18 frame_lost failsafe] or Nx1 flags byte, footer); [ch uint16 Nx16, fl struct (ch17/ch18/frame_lost/failsafe/byte/footer), ok] = sbus_ref_decode(bytes Nx25). ok checks the 0x0F header, the 0x00 or SBUS2 footer and reserved flag bits = 0. Seeds and phase can be changed per run with P.rc.uart.{phase0,drift_ppm,p_err,p_drop} and P.rc.p_loss through SimulationInput.setVariable('P', ...).

## T07 -> T16

Bus_OperatorCmd.tx_power_on must be 1 in every scenario and in the Operator_Console placeholders. With P.ic.Bus_OperatorCmd or scn_make_input defaults (missing = 0), tx_power_on = 0, so every RF frame is lost and the receiver goes to failsafe after P.rc.rx_failsafe_time (1 s). The first frame at t = 0 is always lost (frame_lost = 1 at port time 14 ms), because the op input RT outputs P.ic.Bus_OperatorCmd (TX off) for the first 10 ms. A 0.01 s scenario edge reaches the RC frame chain 10 ms later (D32 input RT).

## T07 -> T08

Corrected RC_Link_Sim timing for your bench and D37 budget. A frame sampled by the TX at k*Tf is released at (k+1)*Tf. At the line level its first byte completes at Tf + ph + Tb = 14.12..15.12 ms; ph is the drifting sub-ms phase (default phase0 0.37 ms, +2.8 us per frame, wraps every 357 frames / 5 s). At the sbus port the first byte appears at +15 ms (about 90 % of frames) or +16 ms (about 10 %, when ph > 0.88 ms), and the last byte about 3 ms later. Chunks per frame are partial / 8-9 / 8-9 / partial over 4 ticks, e.g. [4 8 9 4] or [5 8 8 4]. Every 1 ms tick fully inside a frame carries 8 or 9 bytes. Exactly [8 8 9] appears only for an aligned start (phase0 = 0 with drift 0, or ph in (0.96, 1] ms). rc_frame goes through the same deterministic 14 ms RT and is aligned with the release. Reference codec: bytes = sbus_ref_encode(ch Nx16, flags Nx4 [ch17 ch18 frame_lost failsafe] or an Nx1 flags byte, footer); [ch, fl, ok] = sbus_ref_decode(bytes Nx25). Per-run variation: P.rc.uart.{phase0,drift_ppm,p_err,p_drop} and P.rc.p_loss via SimulationInput.setVariable('P', ...).
