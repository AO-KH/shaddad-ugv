# Shaddad UGV - Interface Control Document (ICD)

> **Generated file.** `docs/icd.md` is produced by `utilities/shaddad_icd_report.m` from
> `docs/icd_static_sections.md` (sections 1-9, hand-written) and the bus definitions in
> `buses/buses_<domain>.m` (section 10, generated). Edit those sources, never `icd.md` itself.
> Architecture and rationale: `docs/architecture.md`. Build tasks: `docs/build_plan.md`.
> **Revision B (2026-10-19):** review fixes D30-D63 of `docs/architecture.md` section 2b are applied
> here (safety re-arm sequence, network tie, rate transitions, loop breakers, CAN2 loop breaker on
> MCU_STATUS, RC failsafe semantics, GNC plan port removed, CAN-FD companion bridge, VC_TEB).
> **G1-01 part 2 (2026-11-18):** brought in line with the parameters (`params/*.m`, the as-built robot since
> 2026-10-27) and the decisions D1-D122 (D111 to D122 change no interface; D117 and D118 wait on card G2-MCU), with the values of the fix pass (step 19). Sections 4 and 6 are provisional (D96). A value that a decision sets but
> the parameters, models or scripts do not hold yet reads "decided, pending on card <card>", and today's value is
> kept for the runs until the change. The bus definitions of section 10 that wait on a card are listed after section 9.

## 1. Binding conventions

| Item | Rule |
|---|---|
| Units | SI on every bus. Temperatures in K, except fields suffixed `_degC`. Magnetic field in uT. Angles in rad, except fields suffixed `_deg`. Every `Simulink.BusElement` carries `Unit` and `Description`. |
| World frame W | Site ENU at `P.world.lla0 = [24.1556 47.3120 430]` (deg, deg, m). x = East, y = North, z = Up. Map frame = W. |
| Body frame B | FLU (x forward, y left, z up), origin **R = base_footprint = base_link**: ground contact plane, mid-gauge, mid contact length. |
| Attitude | ZYX Euler `[psi theta phi]`. psi from East, CCW positive, wrapped to (-pi, pi]. **theta > 0 = nose DOWN** (climbing gives theta < 0). phi > 0 = left side up. Quaternion `q_WB` = MATLAB frame-rotation quaternion W->B, scalar first (`v_B = rotateframe(q, v_W)`). |
| Heading / course | Compass and GNSS heading: from North, clockwise, degrees: `heading_deg = mod(90 - rad2deg(psi), 360)`. |
| NED / FRD | Only inside the STRUCTURE 6DOF block and the Aerospace transform blocks. Converted at `structure_truth.slx`: `R_NED_ENU = [0 1 0; 1 0 0; 0 0 -1]`, `R_FRD_FLU = diag([1 -1 -1])`, `psi_ENU = wrap(pi/2 - psi_NED)`, `theta_FLU = -theta_FRD`, `phi_FLU = phi_FRD`. |
| Accelerometer | **Physical specific force on every bus** (level, at rest: +g on the up axis), identical to the real ICM-20948 driver block. The MATLAB library blocks `mspsensorlib/IMU`, `mspfiltlib/IMU Filter`, `mspfiltlib/ecompass`, `mspfiltlib/Complementary Filter` use the negative convention; a `Gain -1` sits directly at each of them and nowhere else. |
| Library NED defaults | `mspsensorlib/GPS`, `mspsensorlib/IMU`: set `DispRefFrame = 'ENU'`. |
| Motion signs | Track / sprocket / motor speed and torque are **forward-positive on both sides** (motor mirroring is handled by ESC polarity, never by a model sign). `v_R > v_L` gives omega > 0 (left turn). Encoder counts increase when that side moves forward. |
| Electrical signs | Battery / DC current **discharge positive**; a positive current on a `_p` port flows into the consumer. Load power consumption positive. |
| Heat | Heat flow into a thermal mass positive. **No thermal port crosses a plant-level domain boundary** (the `Structure`, `Power`, `Drive`, `Electronics` blocks of `Shaddad_Plant`); ambient enters as `Bus_EnvState.T_amb`. Inside one domain top SSR, child SSRs may share thermal ports (`Pack_*` <-> `Pack_Thermal`, `drive_plant_*` <-> `drive_thermal`), `ConnectionType = 'Connection: foundation.thermal.thermal'` (D52). |
| Data types | Plant: double. STM32 firmware and its CAN payloads: single and integers (MCU H7 has a DP-FPU, but single is used for code efficiency and parity with the G4 BMS). Companion: double for navigation, single on CAN-FD. Booleans are `boolean`. Enumerations travel as `uint8` with the codes of section 9. |
| Multirate | Nonvirtual buses crossing a Model-block boundary are single-rate. Every Model-block root port declares an explicit sample time (never Inf). All Rate Transitions explicit, `Integrity = on`, `Deterministic = on`, which requires an **integer rate ratio**: a 10 ms <-> 14 ms exchange is chained through 1 ms (D32). Allowed port / task rates: {1, 5, 10, 50, 100, 200, 1000} ms. The SBUS frame period 14 ms (7 ms option) exists only **inside** `RC_Link_Sim` and never crosses its boundary; the compiled-rate audit set is `P.sim.Ts_compiled_allowed` = {0, Inf, allowed rates, 0.007, 0.014}. Where a deterministic slow-to-fast RT would add an unwanted delay to a piecewise-constant scenario input, the criterion is measured at the plant-side signal (D62). |
| No-return / invalid | LiDAR no return = NaN. Every message bus carries `valid` (and `seq`/`alive` where it is a CAN message). |

## 2. Physical (Simscape) connections - all inside `models/Shaddad_Plant.slx`

| Label | Domain (`ConnectionType`) | Port A (file / Connection Port) | Port B | Notes |
|---|---|---|---|---|
| `DC48_TracL_p` | `Connection: foundation.electrical.electrical` | `Power_Plant / DC48_TracL_p` (after F_TL 125 A, `P.power.pdu.F_TL`, + 0.98 mOhm, `P.power.pdu.R_cab_trac`) | `drive_system / DC48_TracL_p` | Left ESC DC+, i > 0 into the ESC |
| `DC48_TracL_n` | electrical | `Power_Plant / DC48_TracL_n` (GND, after shunt) | `drive_system / DC48_TracL_n` | **Solver Configuration** (Shaddad_Plant top) is attached to this line |
| `DC48_TracR_p` | electrical | `Power_Plant / DC48_TracR_p` | `drive_system / DC48_TracR_p` | Right ESC DC+ |
| `DC48_TracR_n` | electrical | `Power_Plant / DC48_TracR_n` | `drive_system / DC48_TracR_n` | |
| `Spr_L` | `Connection: foundation.mechanical.rotational.rotational` (classic) | `drive_system / Spr_L` (gearbox output) | `structure_plant / Spr_L` (sprocket) | omega > 0 = forward track motion |
| `Spr_R` | rotational (classic) | `drive_system / Spr_R` | `structure_plant / Spr_R` | |
| `Impl_Shaft` | rotational (classic) | `Power_Plant / Impl_Shaft` (implement motor shaft; after the 12:1 gearbox for the tiller) | `structure_plant / Impl_Shaft` (blade / rotor / pump load) | omega > 0 = nominal implement direction. VC_IMPL == 0: power side `Impl_None` = Rotational Damper 1e-4 N m s/rad + **network tie**; structure side 1e-3 kg m^2 inertia |

Rules: exactly **one Solver Configuration** per physical network, located at the top level of
`Shaddad_Plant` (domain files never contain one; their test benches add their own). The single
Electrical Reference of the traction/LV network is inside `Power_Plant/PDU` at the GND node after
the shunt. Domains may add Mechanical Rotational / Translational References and Thermal References
locally. `Electronics_Plant/Electronics_Thermal` is a separate network without external ports and
therefore owns its own Solver Configuration (local Backward Euler, 0.1 s). Connection Ports always
have an explicit `ConnectionType` (never `Inherit: auto`).

**Network-connectivity rule (D35).** In every variant combination, every mechanical and thermal node of
`Shaddad_Plant` must belong to the network that carries the Solver Configuration (DC48). A motor
(Motor & Drive, DC Motor) provides this coupling. Every choice or stub **without** a motor
(`Impl_None`, `drive_plant_ideal`, the T01 stubs of `drive_system` and `Power_Plant`) contains a
**network tie**: `sprintf('fl_lib/Electrical/Electrical Elements/Rotational\nElectromechanical\nConverter')`
(`K = P.drive.tie.K` = 1e-3 V s/rad; R port on the shaft node, C port on the local Mechanical Rotational
Reference; electrical - on the local DC- / GND port, electrical + through `ee_lib/Passive/Resistor`
`P.drive.tie.R` = 1 MOhm to DC-). Parasitic drag K^2 w/R = 1e-12 w N m, dissipation < 1e-9 W. Where a
thermal node must also be tied (`drive_plant_ideal` -> `drive_thermal`), the 1 MOhm resistor uses
`ComponentPath = 'ee.passive.resistor_thermal'` with its thermal port on that node. T01 compiles
`Shaddad_Plant` for VC_IMPL 0..3 x VC_DRIVE_FID {1, 3} before stubs are handed out.

## 3. Root ports of every model / Subsystem Reference (wiring contract)

Port numbers are 1-based in the order listed. "Ts" is the sample time declared at the port.
Stubs created in build task T01 must have exactly these ports; domain builders must keep them.

### 3.1 `models/Shaddad_System.slx` (top, no Simscape)

The top level holds only the root port `ScenarioIn`, its Signal Specification `Spec_ScenarioIn`, the 11
components below and the Terminator `Term_monitors`, joined by direct lines (design
`docs/superpowers/specs/2026-11-08-top-level-direct-links-design.md`, in place of option A of model layout spec
5). A component is a virtual Subsystem with one input port per bus that feeds it, named after that bus (In Bus
Element blocks, one per signal it picks, named `<port>_<path>`, or `<port>_bus` for the whole bus), and one
output port named after its bus (Out Bus Element blocks `<bus>_<signal>`, one per element; `Viz_Logging` has
none). Each link is one virtual bus line from the producer's output to the consumer's port of that name, named
after the bus (the links table below); `scen` runs from `Spec_ScenarioIn`, one line with a branch to each of
the 9 components that read it, and `monitors`, which nothing reads, ends in `Term_monitors`. A pick is
`<port>.<signal>`, `<signal>` being the producer's output name below; a consumer that needs only part of a bus
picks the sub-element (port `mcu`, element `out10.can1_drv`; port `plant`, element `truth.pose2D`), right in
front of the input it feeds. Unused outputs stay elements of their bus.
Inside a component, the main block is named after its file and keeps the root ports of its row in 3.2 / 3.3
or of this table; Signal Specifications (data type, Ts, nonvirtual bus) sit right in front of every Model
input that no Bus Creator drives.

| Block | Type | Inputs (name : bus : Ts) | Outputs (name : bus : Ts) |
|---|---|---|---|
| `ScenarioIn` (block `In_ScenarioIn`) | root In Bus Element | - | ScenarioIn : Bus_ScenarioIn : 0.01 |
| `Spec_ScenarioIn` | Signal Specification (Bus_ScenarioIn, Ts.scen, nonvirtual) | ScenarioIn | scen : Bus_ScenarioIn : 0.01 (the input `scen` of 9 components) |
| `Term_monitors` | Terminator | monitors (from `Sim_Monitors`) | - |

| Component (bus) | Inside: main block and its ports (name : bus : Ts) | Picks, in input order (`<port>.<element>`) | Bus elements (name : bus : Ts) |
|---|---|---|---|
| `Operator_Console` (`operator`) | Variant Source `OpSource`, `VC_OPSRC`: 1 ScenarioIn.op (`op_ext`); 2 `Dashboard` (Constants `P.scen.dash.*` bound to dashboard blocks); 3 `Joystick` (Simulation 3D Joystick + `joy_map`); 4 `RowDriver`, the simulated driver of the RC row test (`row_driver_step`), fed pose : double[3], v_fwd : double m/s, mcu_mode : double, tank_L : double L, each through a Rate Transition to 0.01 and a Unit Delay (`RT_*`, `*_Delay`; `DTC_mcu_mode` before `RT_mcu_mode`), active with choice 4 only | scen.op; for choice 4: plant.truth.pose2D, plant.truth.speed_fwd, mcu.telem.mode, plant.impl_mech.tankLevel | op : Bus_OperatorCmd : 0.01 |
| `Operator_RC` (`rc`) | SSR `RC_Link_Sim`: 1 op : Bus_OperatorCmd : 0.01; 2 flt : Bus_FaultInject : 0.01 (inside: RT 0.01 -> 0.001 -> frame period, D32) | operator.op, scen.flt | sbus : Bus_SbusStream : 0.001; rc_frame : Bus_RcFrame : 0.001 (RT frame period -> 0.001 inside, logging copy) |
| `MCU` (`mcu`) | `MCU_In`, virtual Subsystem (Rate Transitions + nonvirtual Bus Creators): 1 sbus, 2 elec_hw, 3 flt, 4 imu_raw, 5 esc_fast_rx, 6 esc_slow_rx, 7 impl_rx, 8 comp_cmd_rx, 9 elec_adc, 10 bms_rx (in the order of the buses they feed) -> in1k : Bus_McuIn1k : 0.001; in5 : Bus_McuIn5 : 0.005; in10 : Bus_McuIn10 : 0.01; in50 : Bus_McuIn50 : 0.05; in100 : Bus_McuIn100 : 0.1 (`fault_inject` = task_stall (b0-b3) OR can1_busoff << 4 OR can2_busoff << 5 OR (imu_fault == 3) << 6); then Model `Shaddad_MCU` 1 in1k .. 5 in100. Loop breakers (rule 8, D36): `EscFb_Delay` (0.01), `EscSlow_Delay`, `Impl_Delay` (0.05), `Bms_Delay` (0.1) and `Adc_Delay` (0.05) in front of MCU_In inputs 5-7, 9 and 10: the MCU reads the frame / ADC result of the previous period | rc.sbus, plant.elec_hw_d, scen.flt, sensors.imu_raw, can1.esc_fast_rx, can1.esc_slow_rx, can1.impl_rx, can2.comp_cmd_rx, plant.elec_adc, can1.bms_rx | out1k : Bus_McuOut1k : 0.001; out5 : Bus_McuOut5 : 0.005; out10 : Bus_McuOut10 : 0.01; out50 : Bus_McuOut50 : 0.05; out100 : Bus_McuOut100 : 0.1; telem : Bus_FwTelemetry : 0.01 |
| `BMS` (`bms`) | Model `Shaddad_BMS`: 1 meas : Bus_BMS_Meas : 0.01; 2 can_rx : Bus_Can1_McuSlow : 0.05; out 1 cmd : Bus_BMS_Cmd : 0.01 through `BMS_Cmd_Delay` (0.01, the ECU's one-sample compute delay, rule 8, D36), 2 can_tx, 3 telem | plant.bms_meas, can1.mcu_slow_rx | cmd_d : Bus_BMS_Cmd : 0.01 (the delayed command); can_tx : Bus_Can1_Bms : 0.1; telem : Bus_BMS_Telemetry : 0.1 |
| `CAN1_Net` (`can1`) | SSR `CAN1_Net_Sim`: 1 mcu_fast : Bus_Can1_McuFast : 0.01; 2 mcu_slow : Bus_Can1_McuSlow : 0.05; 3 esc_fast : Bus_Can1_EscFast : 0.01; 4 esc_slow : Bus_Can1_EscSlow : 0.05; 5 impl : Bus_Can1_Impl : 0.05; 6 bms : Bus_Can1_Bms : 0.1; 7 flt : Bus_FaultInject : 0.01; 8 elec_hw : Bus_ElecHwIn : 0.001 | mcu.out10.can1_drv, mcu.out50.can1_slow, plant.can_esc_fast, plant.can_esc_slow, plant.can_impl, bms.can_tx, scen.flt, plant.elec_hw_d | mcu_fast_rx, mcu_slow_rx, esc_fast_rx, esc_slow_rx, impl_rx, bms_rx: the same buses delivered, same Ts |
| `Plant` (`plant`) | Model `Shaddad_Plant`: 1 scen : Bus_ScenarioIn : 0.01; 2 mcu_out1k : Bus_McuOut1k : 0.001; 3 mcu_out10 : Bus_McuOut10 : 0.01; 4 can_mcu_fast : Bus_Can1_McuFast : 0.01; 5 can_mcu_slow : Bus_Can1_McuSlow : 0.05; 6 bms_cmd : Bus_BMS_Cmd : 0.01; out 9 elec_hw : Bus_ElecHwIn : 0.001 through `ElecHw_Delay` (0.001, the DI word the MCU samples, rule 8, D36) | scen, mcu.out1k, mcu.out10, can1.mcu_fast_rx, can1.mcu_slow_rx, bms.cmd_d | truth : Bus_TruthState : 0.001; tracks : Bus_TrackState : 0.001; drive_truth : Bus_DriveTruth : 0.001; power_truth : Bus_PowerTruth : 0.1; bms_meas : Bus_BMS_Meas : 0.01; can_esc_fast : Bus_Can1_EscFast : 0.01; can_esc_slow : Bus_Can1_EscSlow : 0.05; can_impl : Bus_Can1_Impl : 0.05; elec_hw_d : Bus_ElecHwIn : 0.001 (the delayed word; MCU, CAN1 and CAN2 read it); elec_adc : Bus_ElecAdc : 0.05; elec_env : Bus_ElecEnv : 0.1; env : Bus_EnvState : 0.1; indicators : Bus_IndicatorOut : 0.1; impl_mech : Bus_ImplMechState : 0.01; impl_elec : Bus_ImplElecState : 0.01; struct_diag : Bus_StructureDiag : 0.01; power_status : Bus_PowerStatus : 0.001 |
| `Sensors` (`sensors`) | SSR `Shaddad_SensorModels`: 1 truth : Bus_TruthState : 0.001; 2 drive_truth : Bus_DriveTruth : 0.001; 3 env : Bus_EnvState : 0.1; 4 elec_env : Bus_ElecEnv : 0.1; 5 flt : Bus_FaultInject : 0.01; 6 obs : Bus_ObstaclesDyn : 0.01 | plant.truth, plant.drive_truth, plant.env, plant.elec_env, scen.flt, scen.obs | imu_raw : Bus_ImuRaw : 0.005; gnss_raw : Bus_GnssRaw : 0.1; lidar_scan : Bus_LidarScanRaw : 0.1; lidar_cloud : Bus_LidarCloudRaw : 0.1; spray : Bus_SprayDemand : 0.1; sens_diag : Bus_SensTruthDiag : 0.1 |
| `CAN2_Link` (`can2`) | SSR `Link_MCU_GNC`: 1 mcu_imu : Bus_CanFrame64 : 0.005; 2 mcu_odom : Bus_CanFrame64 : 0.01; 3 mcu_status : Bus_CanFrame64 : 0.05; 4 mcu_power : Bus_CanFrame64 : 0.1; 5 comp_cmd : Bus_CanFrame64 : 0.05; 6 flt : Bus_FaultInject : 0.01; 7 elec_hw : Bus_ElecHwIn : 0.001 | mcu.out5.can2_imu, mcu.out10.can2_odom, mcu.out50.can2_status, mcu.out100.can2_power, gnc.comp_cmd, scen.flt, plant.elec_hw_d | mcu_imu_rx : 0.005; mcu_odom_rx : 0.01; mcu_status_rx : 0.05 (**one-sample Unit Delay = MCU status assembly delay, breaks the MCU-GNC loop**, D41); mcu_power_rx : 0.1; comp_cmd_rx : 0.05 (no delay; the MCU's deterministic Task_50 -> Task_10 RT is the 50 ms GNC compute delay) |
| `GNC` (`gnc`) | `MissionCmd_RT`, Rate Transition 0.01 -> 0.2 (D32), then Variant Subsystem `GNC_Variant`, `VC_PHASE` (1 `GNC_Stub`, 2 `Companion`): 1 mcu_imu : Bus_CanFrame64 : 0.005; 2 mcu_odom : 0.01; 3 mcu_status : 0.05; 4 mcu_power : 0.1; 5 gnss_raw : Bus_GnssRaw : 0.1; 6 lidar_scan : Bus_LidarScanRaw : 0.1; 7 lidar_cloud : Bus_LidarCloudRaw : 0.1; 8 spray : Bus_SprayDemand : 0.1; 9 mcmd : Bus_MissionCmd : 0.2 (from `MissionCmd_RT`) | can2.mcu_imu_rx, can2.mcu_odom_rx, can2.mcu_status_rx, can2.mcu_power_rx, sensors.gnss_raw, sensors.lidar_scan, sensors.lidar_cloud, sensors.spray, scen.mcmd (into `MissionCmd_RT`) | comp_cmd : Bus_CanFrame64 : 0.05; nav_state : Bus_NavState : 0.01; perception : Bus_Perception : 0.1; gnc_status : Bus_GncStatus : 0.2; gnc_debug : Bus_GncDebug : 0.05; nav_diag : Bus_NavDiag : 0.01 |
| `Viz_Logging` (-) | SSR `Viz_Logging` (Variant `VC_VIZ`): 1 truth; 2 nav_state; 3 gnc_status; 4 perception; 5 telem : Bus_FwTelemetry; 6 indicators : Bus_IndicatorOut | plant.truth, gnc.nav_state, gnc.gnc_status, gnc.perception, mcu.telem, plant.indicators | - |
| `Sim_Monitors` (`monitors`) | SSR `Sim_Monitors`: 1 truth : 0.001; 2 power_truth : 0.1; 3 drive_truth : 0.001; 4 obs : Bus_ObstaclesDyn : 0.01; 5 telem : Bus_FwTelemetry : 0.01; 6 gnc_status : 0.2; 7 struct_diag : 0.01 | plant.truth, plant.power_truth, plant.drive_truth, scen.obs, mcu.telem, gnc.gnc_status, plant.struct_diag | kpi : Bus_SimKPI : 0.1 |

Links: the input ports of each component, top to bottom (port 1 first), each named after the bus it takes (28
links between components, and `scen` to 9 of them).

| Component | Input ports |
|---|---|
| `Operator_Console` | scen, plant, mcu |
| `Operator_RC` | scen, operator |
| `MCU` | scen, rc, sensors, can1, plant, can2 |
| `BMS` | can1, plant |
| `CAN1_Net` | mcu, scen, plant, bms |
| `Plant` | mcu, scen, can1, bms |
| `Sensors` | scen, plant |
| `CAN2_Link` | scen, mcu, plant, gnc |
| `GNC` | scen, sensors, can2 |
| `Viz_Logging` | mcu, plant, gnc |
| `Sim_Monitors` | scen, plant, mcu, gnc |

`GNC/GNC_Variant/Companion` contents (VC_PHASE == 2): Model `Shaddad_Companion` (D38: the single
deployable companion top, `models/Shaddad_Companion.slx`) which contains `Companion_IO` (SSR
`companion_io`), Model `Shaddad_NavFusion`, Model `Shaddad_Perception`, Model `Shaddad_GNC` and the
Perception nav_state Unit Delay; its root ports are the `GNC_Variant` ports above. The mission plan
is **not** a port: a Constant `MissionSource` inside `Shaddad_GNC` reads `P.gnc.mission` (D57).
`GNC_Stub` (VC_PHASE == 1): Terminators on all inputs; Constants `P.ic.<Bus>` on all outputs
(`comp_cmd.valid = false`, so the MCU never gets `comp_ok`).

| Block (inside Companion) | Inputs | Outputs |
|---|---|---|
| `Companion_IO` (SSR `companion_io`, Variant `VC_TARGET`: SIM = root frames straight into the CAN-FD codecs; HW = `ros2lib/Subscribe` `/can2/rx` and `ros2lib/Publish` `/can2/tx` (`std_msgs/UInt8MultiArray`, 69 B = id u32 LE, len u8, data 64 B) fed by an external SocketCAN-FD bridge node on the Jetson, D38; `ros2lib/CAN Receive`/`CAN Transmit` are classic CAN only, `MsgLength <= 8`, verified) | 1 mcu_imu : Bus_CanFrame64 : 0.005; 2 mcu_odom : 0.01; 3 mcu_status : 0.05; 4 mcu_power : 0.1; 5 gnc_cmd : Bus_GncCmd : 0.05 | 1 imu_meas : Bus_ImuMeas : 0.005; 2 odom_meas : Bus_OdomMeas : 0.01; 3 mcu_status : Bus_McuStatus : 0.05; 4 bms_status : Bus_BmsStatus : 0.1; 5 comp_cmd : Bus_CanFrame64 : 0.05 |
| `NavFusion` (Model `Shaddad_NavFusion`) | 1 imu : Bus_ImuMeas : 0.005; 2 odom : Bus_OdomMeas : 0.01; 3 gnss : Bus_GnssRaw : 0.1; 4 scanmatch : Bus_ScanMatchMeas : 0.2; 5 navcmd : Bus_NavCmd : 0.05; 6 bms : Bus_BmsStatus : 0.1 | 1 nav_state : Bus_NavState : 0.01; 2 nav_diag : Bus_NavDiag : 0.01 |
| `Perception` (Model `Shaddad_Perception`) | 1 scan : Bus_LidarScanRaw : 0.1; 2 cloud : Bus_LidarCloudRaw : 0.1; 3 nav_state : Bus_NavState : 0.01 | 1 perception : Bus_Perception : 0.1; 2 scanmatch : Bus_ScanMatchMeas : 0.2 |
| `GNC` (Model `Shaddad_GNC`) | 1 nav_state : Bus_NavState : 0.01; 2 perception : Bus_Perception : 0.1; 3 mcu_status : Bus_McuStatus : 0.05; 4 bms_status : Bus_BmsStatus : 0.1; 5 mcmd : Bus_MissionCmd : 0.2; 6 spray : Bus_SprayDemand : 0.1 (no `plan` port, D57) | 1 gnc_cmd : Bus_GncCmd : 0.05; 2 nav_cmd : Bus_NavCmd : 0.05; 3 gnc_status : Bus_GncStatus : 0.2; 4 gnc_debug : Bus_GncDebug : 0.05 |

### 3.2 `models/Shaddad_Plant.slx` (all Simscape)
Root inputs/outputs = the `Plant` row of 3.1. Continuous plant signals reach root outports through
`Zero-Order Hold` blocks at the declared Ts. Internal blocks:

| Block | Type | Physical ports | Inputs (name : bus : Ts) | Outputs (name : bus : Ts) |
|---|---|---|---|---|
| `Plant_In` | virtual Subsystem | - | 1 scen_hmi : Bus_HmiInputs : 0.01; 2 scen_flt : Bus_FaultInject : 0.01; 3 scen_env : Bus_EnvOverride : 0.01 (the elements hmi, flt and env of the root input scen, picked right in front of Plant_In by the Bus Selector `Sel_scen`) | hmi : Bus_HmiInputs : 0.001; flt : Bus_FaultInject : 0.001; drv_flt : Bus_DriveFaultInj : 0.001 (mapping in architecture 6.3); env_ovr : Bus_EnvOverride : 0.1. The 10 ms -> 1 ms RT delays hmi by one 10 ms sample; E-stop latency criteria are measured from this plant-side hmi (D62) |
| `Structure` | SSR `structure_plant` | `Spr_L`, `Spr_R`, `Impl_Shaft` (all Left side) | 1 impl_cmd : Bus_ImplementCmd : 0.01; 2 env_dist : Bus_EnvDisturbance (Constant `P.ic.Bus_EnvDisturbance`); 3 env_ovr : Bus_EnvOverride : 0.1 | 1 truth : Bus_TruthState : 0; 2 tracks : Bus_TrackState : 0; 3 impl_mech : Bus_ImplMechState : **0.01** (internal Zero-Order Hold, D32); 4 env : Bus_EnvState : 0.1; 5 diag : Bus_StructureDiag : 0 |
| `Power` | SSR `Power_Plant` | `DC48_TracL_p`, `DC48_TracL_n`, `DC48_TracR_p`, `DC48_TracR_n`, `Impl_Shaft` (Right side) | 1 hmi : 0.001; 2 hw_cmd : Bus_PowerHwCmd : 0.001; 3 lv_demand : Bus_LV_LoadDemand : 0.01; 4 bms_cmd : Bus_BMS_Cmd : 0.01; 5 env : Bus_EnvState : 0.1; 6 impl_mech : Bus_ImplMechState : 0.01; 7 can_rx : Bus_Can1_McuSlow : 0.05; 8 flt : Bus_FaultInject : 0.001 | 1 bms_meas : Bus_BMS_Meas : 0.01; 2 power_status : Bus_PowerStatus : 0.001; 3 impl_elec : Bus_ImplElecState : 0.01; 4 can_tx : Bus_Can1_Impl : 0.05; 5 power_truth : Bus_PowerTruth : 0.1; 6 safety_ok : boolean : 0.001 |
| `Drive` | SSR `drive_system` | `DC48_TracL_p`, `DC48_TracL_n`, `DC48_TracR_p`, `DC48_TracR_n` (Left), `Spr_L`, `Spr_R` (Right) | 1 can_rx : Bus_Can1_McuFast : 0.01; 2 sto_ok : boolean : 0.001; 3 T_amb : double K : 0.1; 4 fault_inj : Bus_DriveFaultInj : 0.001 | 1 can_tx_fast : Bus_Can1_EscFast : 0.01; 2 can_tx_slow : Bus_Can1_EscSlow : 0.05; 3 truth : Bus_DriveTruth : 0.001 |
| `Electronics` | SSR `Electronics_Plant` (no physical ports) | - | 1 mcu_out1k : Bus_McuOut1k : 0.001; 2 mcu_out10 : Bus_McuOut10 : 0.01; 3 power_status : Bus_PowerStatus : 0.001; 4 hmi : Bus_HmiInputs : 0.001; 5 env : Bus_EnvState : 0.1; 6 impl_mech : Bus_ImplMechState : 0.01; 7 flt : Bus_FaultInject : 0.001 | 1 elec_hw : Bus_ElecHwIn : 0.001 (via 1 ms Unit Delay = DI sampling latency, breaks the MCU in1k -> out1k -> watchdog-reset loop, D36); 2 elec_adc : Bus_ElecAdc : 0.05; 3 elec_env : Bus_ElecEnv : 0.1; 4 hw_cmd : Bus_PowerHwCmd : 0.001 (via 1 ms Unit Delay); 5 impl_cmd : Bus_ImplementCmd : 0.01; 6 lv_demand : Bus_LV_LoadDemand : 0.01; 7 indicators : Bus_IndicatorOut : 0.1 |
| `Solver Configuration` | `nesl_utility/Solver Configuration` | on line `DC48_TracL_n` | - | - |
| `Plant_Out` ZOHs | `simulink/Discrete/Zero-Order Hold` | - | every continuous signal before a root outport (truth 0.001, tracks 0.001, struct_diag 0.01) | - |

Plant signal lines: Structure.impl_mech (0.01) -> Power.6, Electronics.6, out14. Structure.env -> Power.5,
Electronics.5, Drive.3 (`env.T_amb` via Bus Selector), out12. Power.safety_ok -> Drive.2.
Power.power_status -> Electronics.3, out17. Electronics.hw_cmd -> Power.2. Electronics.impl_cmd ->
Structure.1. Electronics.lv_demand -> Power.3. Root in6 bms_cmd -> Power.4. Root in5
can_mcu_slow -> Power.7. Root in4 can_mcu_fast -> Drive.1. Root in2/in3 -> Electronics.1/.2.

### 3.3 Controller models
| Model | Inputs | Outputs | Config |
|---|---|---|---|
| `Shaddad_MCU` | as `MCU` row of 3.1 | as `MCU` row | `cs_Shaddad` (sim), `cs_STM32H7` (build); FixedStepDiscrete 1 ms, multitasking |
| `Shaddad_BMS` | as `BMS` row | as `BMS` row | `cs_Shaddad`, `cs_STM32G4`; base 10 ms |
| `Shaddad_NavFusion`, `Shaddad_Perception`, `Shaddad_GNC` | section 3.1 Companion table | section 3.1 Companion table | `cs_Shaddad`, `cs_Companion` (ert C++, ARM64 LP64) |

## 4. CAN1 - classic CAN 2.0B, 500 kbit/s, 11-bit IDs, little-endian (STM32 FDCAN1) - provisional (D96)

**Provisional (D96).** The ESC frames are the twin's CiA-402-class drive, and the BMS frames are the twin's G4 BMS,
a variant of the delivered BMS. The delivered ESCs' and BMS's protocols come with card G1-04. The drive's codecs
match this layout (T16: 0 byte mismatches on CMD, FB1, FB2 and FB3). K4 and the dock and charging flags belong to
the twin's dock: the robot has no dock in this phase and is charged by hand (D97).

Encoding everywhere: `mcblib/Signal Management/Byte Pack` / `Byte Unpack` (`byteAlign = 1`,
uint8 output) preceded/followed by scaling `Gain` + `Data Type Conversion` (round to nearest,
saturate). The same codec subsystem files are used by the producer, the consumer and the HW
variant (`stm32blockslib/FDCAN Write`/`FDCAN Read`, `DataFormatWriteMode = 'Raw data'`).
Bit fields are assembled with `Bit Set`/`Bitwise Operator`/`Shift Arithmetic`/`Extract Bits`.

| ID | Name | Producer -> consumer | Period | Payload (byte offset: type field [resolution]) |
|---|---|---|---|---|
| 0x201 / 0x202 | DRV_CMD_L / _R | MCU -> ESC node 1 / 2 | 10 ms | 0: u16 ctrl [b0 enable, b1 park_req, b2 fault_reset, b3-4 mode (0 OFF, 1 SPEED, 2 TORQUE), b8-15 alive]; 2: i16 w_ref [1 rpm, motor shaft]; 4: i16 tau_ff [0.01 N m]; 6: u8 tau_lim_mot [0.1 N m]; 7: u8 tau_lim_reg [0.1 N m] |
| 0x181 / 0x182 | DRV_FB1_L / _R | ESC -> MCU | 10 ms | 0: u16 status; 2: i16 speed [1 rpm]; 4: i16 tau_est [0.01 N m]; 6: i16 i_dc [0.1 A] |
| 0x281 / 0x282 | DRV_FB2_L / _R | ESC -> MCU | 50 ms | 0: u16 v_dc [0.01 V]; 2: i16 i_q [0.1 A]; 4: i16 T_motor [0.1 degC]; 6: u8 T_esc [1 degC, offset -40]; 7: u8 tau_avail [0.1 N m] |
| 0x381 / 0x382 | DRV_FB3_L / _R | ESC -> MCU | 10 ms | 0: i32 enc_cnt [4096 counts/rev, wraps]; 4: u16 fault_code; 6: u8 alive; 7: u8 state |
| 0x140 | IMPL_CMD | MCU -> implement ECU | 50 ms | 0: u8 ctrl [b0 enable, b1 fault_reset]; 1: u8 alive; 2: i16 speed_cmd [1 rpm at blade / tiller rotor]; 4: u16 reserved; 6: u16 reserved |
| 0x141 | IMPL_FB | implement ECU -> MCU | 50 ms | 0: u8 status [b0 running, b1 fault, b2 ready]; 1: u8 impl_type; 2: i16 speed [1 rpm]; 4: i16 current [0.1 A]; 6: u8 temp [1 degC, offset -40]; 7: u8 alive |
| 0x3D0 | BMS_REQ | MCU -> BMS | 50 ms | 0: u8 flags [b0 HV_Enable_Req, b1 Charge_Enable_Req, b2 Fault_Reset_Req, b3 Shutdown_Req]; 1: u8 alive; 2-7 reserved |
| 0x351 | BMS_LIMITS | BMS -> MCU | 100 ms | 0: u16 CVL [0.1 V]; 2: i16 CCL [0.1 A]; 4: i16 DCL [0.1 A]; 6: u16 DVL [0.1 V] |
| 0x355 | BMS_SOC | BMS -> MCU | 100 ms | 0: u16 SOC [1 %]; 2: u16 SOH [1 %]; 4: u16 SOC_hires [0.01 %]; 6: u16 SOE [0.01 %] |
| 0x356 | BMS_VIT | BMS -> MCU | 100 ms | 0: i16 V_pack [0.01 V]; 2: i16 I_pack [0.1 A, + discharge]; 4: i16 T_batt [0.1 degC]; 6: u16 V_bus [0.01 V] |
| 0x35A | BMS_ALARM | BMS -> MCU | 100 ms | 0: u32 Fault_Word; 4: u32 Warning_Word |
| 0x373 | BMS_CELLS | BMS -> MCU | 100 ms | 0: u16 Vcell_min [1 mV]; 2: u16 Vcell_max [1 mV]; 4: u16 Tcell_min [0.1 K]; 6: u16 Tcell_max [0.1 K] |
| 0x3C0 | BMS_STATE | BMS -> MCU | 100 ms | 0: u8 Contactor_State; 1: u8 flags [b0 HV_Ready, b1 HV_Opening_Warn, b2 K1_Closed, b3 K2_Closed, b4 K4_Closed, b5 Charging_Active, b6 Charge_Complete, b7 Balancing_Active]; 2: u8 Fault_Level; 3: u8 alive; 4: u16 Runtime_Remaining [1 min]; 6: u16 E_remaining [10 Wh] |
| 0x3C1 | BMS_POWER | BMS -> MCU | 100 ms | 0: u16 P_dis_lim [10 W]; 2: u16 P_chg_lim [10 W]; 4: i16 P_pack [10 W]; 6: u8 Fan_Cmd [1 %]; 7: reserved |

DRV_CMD during a soft stop (D71, parking): the MCU sends enable 0 and park_req 1, and holds tau_lim_mot and tau_lim_reg (the torque limits) until both FB1 frames confirm the brakes. A confirm is FB1 status b4 (brake_released) 0, with the ESC state at or below READY or at or above FAULT, rx_ok and drv_age <= drv_timeout. The MCU reads FB1 b4. Only then do the limits go to 0 and the mode goes to STANDBY or ESTOP.SAFE.

Timeouts: ESC -> QUICK_STOP after 100 ms without a new CMD alive value; MCU -> `rx_ok = false`
and DRV_x_TIMEOUT after 50 ms without FB1/FB3; MCU -> BMS_TIMEOUT after 0.5 s; BMS ->
MCU_CMD_TIMEOUT (HV_Enable_Req treated as false) after 0.5 s frozen BMS_REQ alive; implement
ECU -> rotor stop after 0.2 s. Bus load about 21 % (770 frames/s).

ESC stop time-out (drive, D71): DISABLING without park_req, and QUICK_STOP, apply the spring brake at standstill (|w| < 2 rad/s for 0.2 s). Whatever the speed, they apply it at the latest 2.0 s after the stop began (`P.drive.esc.t_stop_max`, [EST]). A stop begins on entry to DISABLING or QUICK_STOP; a DISABLING that turns into QUICK_STOP keeps counting from the DISABLING entry. park_req applies the brake at once, whatever the speed, from OPERATION or ENABLING. The torque goes off 80 ms after the brake; the ESC then goes to READY from DISABLING, or to FAULT with CAN_TIMEOUT latched from QUICK_STOP.

## 5. CAN2 - CAN-FD 1 Mbit/s arbitration / 2 Mbit/s data (BRS), MCU FDCAN2 <-> Jetson (SocketCAN)

| ID | Name | Producer -> consumer | Period | Len | Payload (byte offset: type field [resolution]) |
|---|---|---|---|---|---|
| 0x100 | COMP_CMD | companion -> MCU | 50 ms | 32 | 0: u8 seq; 1: u8 mode_req; 2: u8 gnc_state; 3: u8 flags [b0 cmd_valid, b1 gnc_ready, b2 soft_stop_req, b3 v_ground_valid, b4 mower_enable, b5 tiller_enable, b6 spray_enable, b7 buzzer_cmd]; 4: single v_cmd; 8: single omega_cmd; 12: single accel_limit; 16: single v_ground_est; 20: u16 spray_section_mask; 22: u8 beacon_cmd; 23: u8 nav_fix; 24: u16 mower_speed_cmd [0.1 rad/s]; 26: u16 tiller_speed_cmd [0.01 rad/s]; 28: u8 deck_height_cmd [1 mm]; 29: u8 tiller_depth_cmd [1 mm]; 30: u16 spray_flow_cmd [0.01 L/min] |
| 0x200 | MCU_STATUS | MCU -> companion | 50 ms | 48 | 0: u32 t_ms; 4: u8 mode; 5: u8 cmd_src; 6: u8 rc_mode_sw; 7: u8 rc_aux_cmd; 8: u8 rc_aux_seq; 9: u8 flags [b0 rc_link_ok, b1 rc_failsafe, b2 estop_active, b3 drive_ready, b4 power_ready, b5 implement_ready, b6 charging, b7 wdg_reset_seen]; 10: u8 implement_type; 11: u8 heartbeat; 12: u32 fault_word; 16: u32 warn_word; 20: u32 drive_fault_word; 24: single v_track_L; 28: single v_track_R; 32: single v_applied; 36: single w_applied; 40: u16 gnc_cmd_age_ms; 42: u8 rc_lq [1 %]; 43: reserved; 44: single T_encl [K] |
| 0x201 | MCU_IMU | MCU -> companion | 5 ms | 48 | 0: u32 t_us; 4: u16 seq; 6: u8 valid; 7: reserved; 8: single acc[3] (body FLU, physical); 20: single gyr[3]; 32: single mag[3] [uT]; 44: single temp [degC] |
| 0x202 | MCU_ODOM | MCU -> companion | 10 ms | 32 | 0: u32 t_us; 4: i32 enc_cnt_L; 8: i32 enc_cnt_R; 12: single v_track_L; 16: single v_track_R; 20: single w_m_L; 24: single w_m_R; 28: u8 valid; 29-31 reserved |
| 0x203 | MCU_POWER | MCU -> companion | 100 ms | 64 | 0..51: single V_pack, I_pack, SOC, SOH, T_cell_max, T_cell_min, V_cell_min, V_cell_max, CCL, DCL, P_dis_lim, E_remaining_Wh, Runtime_s; 52: u32 alarm_word; 56: u32 warn_word; 60: u8 fault_level; 61: u8 contactor_state; 62: u8 flags [b0 hv_ready, b1 hv_opening_warn, b2 charging, b3 charge_complete, b4 bms_critical, b5 valid]; 63: u8 alive |

MCU: companion stale (`COMP_TIMEOUT`) when `seq` has not changed for 0.2 s. Companion:
`Bus_McuStatus.valid = false` after 0.2 s without MCU_STATUS. In SIM `Link_MCU_GNC` delays
**MCU_STATUS** by one 50 ms sample (loop breaker), passes COMP_CMD undelayed, and drops all frames
when `flt.can2_loss`, `flt.can2_busoff` or `elec_hw.mcu_reset`. GNC -> MCU Task_10 latency = 50 ms
(Task_50 -> Task_10 RT) + 10 ms (Task_10 -> ESC RT) = 60 ms (was 110 ms in revision A).
HW: the Jetson runs a SocketCAN-FD bridge node (`can0`, 1 M / 2 M BRS) that republishes every frame
as `std_msgs/UInt8MultiArray` on `/can2/rx` and sends `/can2/tx` messages (69 B: id u32 LE, len u8,
data 64 B); `companion_io` HW uses `ros2lib/Subscribe` / `ros2lib/Publish` (D38).
Bus load about 9 % (an estimate with worst-case stuffing; MCU_IMU at 200 frames/s is about 6 %). The companion
computer and its bridge are the design's: the as-built record lists no companion computer (card G1-04).

## 6. STM32H743 I/O maps - provisional (D96)

**Provisional (D96).** This is the map of the development target, the STM32H743. The robot's controller is the
supplier's board; its map comes with card G1-04, with where the IMU (I2C1 here) and the companion computer connect.
The as-built record lists no IMU, GNSS, LiDAR or companion computer.

**DI word (`Bus_McuIn1k.di_word`, uint16, GPIOF[0:13])**: b0 ESTOP_CH1_OK (1 = loop closed),
b1 ESTOP_CH2_OK, b2 SAFETY_OK (relay immediate contacts), b3 BUMPER_FRONT (1 = pressed),
b4 BUMPER_REAR, b5 RESET_BTN, b6 K1_AUX_FB, b7 K2_AUX_FB, b8 IMPL_ID0, b9 IMPL_ID1 (implement
coding plug: ID = VC_IMPL), b10 LID_CLOSED, b11 KEY_ON, b12 SAFETY_HV_OK, b13 DOCK_PRESENT (the twin's dock, D97).

**DO word (`Bus_McuOut1k.do_word`, uint32, GPIOG[0:15] + GPIOE[0:3])**: b0 MCU_SAFETY_OK
(watchdog-relay contact in the E-stop loop) = wdg healthy AND NOT (fault_word AND
`P.electronics.safety.maskDO0`, bits 5-8, 22, 23, 25 = MCU-internal health only) AND mode != BOOT; the
hardware-chain consequence bits 0, 27, 29, 31 never drop DO0 (D30). b1 SAFETY_RESET_REQ = monostable
pulse `t_pulse` 50 ms, started (a) once automatically on SELFTEST entry (D91 replaces this automatic re-arm: decided, pending on
card G2-MCU) and (b) on every reset_req
rising edge (DI RESET_BTN or RC CH9), in both cases only after DO0 has been high for
`t_do0_settle` 20 ms. b2 IMPL24_EN (K3 request),
b3 BEACON_GREEN, b4 BEACON_AMBER, b5 BEACON_RED, b6 BUZZER, b7 WORK_LIGHTS,
b8 COMPANION_PWR_EN, b9 LIDAR_PWR_EN, b10..b17 NOZZLE_1..8, b18 LIFT_DIR (1 = down),
b19 spare. All DO = 0 is the safe state (MCU reset / watchdog).

**PWM (`Bus_McuOut10.pwm_duty`, single 0..1)**: [1] spray pump TIM1 CH1 20 kHz, [2] lift
actuator TIM1 CH2 20 kHz (direction = DO b18), [3] enclosure fan 1 TIM4 CH1 25 kHz, [4] fan 2
TIM4 CH2.

**ADC (`Bus_McuIn50.adc_raw`, uint16 counts, Vref 3.3 V, 16 bit)**: [1] Vpack (divider 21),
[2] Vbus (21), [3] Ipack Hall (1.65 V + 4.1667 mV/A), [4] V24 (11), [5] V12 (5.7), [6] V5 (2),
[7] enclosure NTC (10 k / B 3950 / 10 k pull-up), [8] lift potentiometer (0..3.3 V = 0..1).

**UART**: USART2 RX (PD6), SBUS 100 kbaud 8E2 inverted (RXINV). **I2C1**: ICM-20948 at 0x69,
400 kHz. **FDCAN1**: CAN1 table. **FDCAN2**: CAN2 table. **IWDG1**: 50 ms (PR /4, RLR 399).
**TIM2**: 32-bit free-running at 1 MHz -> `Bus_McuIn1k.t_us` (HW: `stmutilitieslib/Register Read//Write`
TIM2 CNT). **RCC_RSR** IWDG1RSTF (bit 26) -> `reset_flag` (Register Read at boot, then RMVF write). Only IWDG1RSTF is read today; D88 and D91 add the brown-out and
power-on flags and a reset-cause field in MCU_STATUS (decided, pending on card G2-MCU).
**FDCAN_PSR** BO bit -> bus-off bits 25/26 (Register Read per FDCAN, Task_10). All configured in the
CubeMX project of T17 (D31, D60).

**Power-up / re-arm sequence (D30, normative; S0 criteria).** t = 0: MCU BOOT, DO0 = 0, safety relay
open (SR latch reset dominant). BOOT -> SELFTEST after one tick; DO0 rises as soon as the watchdog
ages are healthy (< 20 ms). 20 ms later the MCU emits the automatic DO1 pulse (50 ms); in
`Safety_Chain` the reset edge is stretched to `P.power.safety.t_reset_hold` = 50 ms, so
Set = Loop_OK AND Reset_pulse closes the relay: Safety_OK and Safety_HV_OK rise 20 ms later (< 0.1 s).
`hv_enable_req` (key on AND NOT class A AND mode not BOOT/FAULT) -> BMS precharge -> HV_Ready < 1.0 s.
SELFTEST -> STANDBY at `selftest_t` (1.0 s) when rails, drives, BMS and IMU are fresh. With an E-stop
pressed at power-up the relay stays open, HW_ESTOP latches, the MCU enters ESTOP; release + reset button
(or CH9) -> MCU clears the latch, re-issues DO1 -> relay closes -> STANDBY; a new arm edge is needed to move.

**Decided, pending on card G2-MCU (D88, D91).** After every switch-on, watchdog reset or brown-out the robot waits
in FAULT, with the brakes held and the safety relay not re-armed, until the operator's deliberate reset and a new
arm command. The sequence above is today's model, kept for the runs until the change (S0 follows on card G2-INT).

**MCU supervisor fault word (`fault_word`, uint32)** - class A: immediate torque removal (ESC
disable + MCU_SAFETY_OK low); B: controlled stop at 1.0 m/s^2, then park (park_req, limits held until both brakes confirm), then STANDBY; C: derate / inhibit.

| Bit | Name | Detection | t_persist | Class | Latched |
|---|---|---|---|---|---|
| 0 | HW_ESTOP | DI ESTOP_CH1_OK or CH2_OK low | 0 | A | yes |
| 1 | REMOTE_ESTOP | RC CH12 = STOP, evaluated only on frames with failsafe = 0 and frame_lost = 0 (held otherwise), no channel debounce | 0.04 s | ESTOP state | yes |
| 2 | RC_LINK_LOST | rc_age > 0.10 s | 0 | B | no |
| 3 | RC_FAILSAFE | SBUS failsafe flag | 0 | B | no |
| 4 | COMP_TIMEOUT | COMP_CMD seq unchanged > 0.2 s (AUTO_START and AUTO only) | 0 | B | no |
| 5 / 6 | DRV_L / R_TIMEOUT | FB1 or FB3 age > 50 ms | 0 | A | yes |
| 7 / 8 | DRV_L / R_FAULT | ESC state FAULT or fault_code != 0 (bit 10 STO excluded) | 0 | A | yes |
| 9 | BMS_TIMEOUT | any BMS frame age > 0.5 s | 0 | B | yes |
| 10 | BMS_ALARM | BMS fault_level == 3 | 0 | B | yes |
| 11 | BMS_WARN | warning word != 0 or DCL < 25 A (`dcl_warn`) | 0 | info (no derate; BMS limits act through DCL / CCL, spec 2.7.9) | no |
| 12 / 13 | ENC_L / R_MISMATCH | abs(v_enc - w_m*r_s/N) > 0.3 m/s | 0.2 s | B | yes |
| 14 | TILT_WARN | max(abs(roll), abs(pitch)) > 25 deg (level-calibrated estimate), detected only while bit 24 is clear (previous sample) | 0.5 s, held (not restarted) on an invalid IMU sample | C (derate 0.5) | no |
| 15 | TILT_STOP | > 28 deg, detected only while bit 24 is clear (previous sample) | 0.5 s, held (not restarted) on an invalid IMU sample | B | yes |
| 16 | OVERSPEED | (a) abs(v_track) > 1.70 m/s (`lim.v_overspeed`); (b) the descent trip (D74, see note): drive on (modes MANUAL to ESTOP) and odometry valid, the predicted body speed above the descent cap of the measured pitch; (c) the speed-follow term, in the driving modes only (see note): on either track, abs(v_track) above its held reference by more than 0.2 m/s (`lim.v_follow_dv`); odometry valid | (a) and (c) 0.2 s; (b) 0 | B | yes |
| 17 | HV_LOST | BMS HV_Ready false while ACTIVE | 0.2 s | B | yes |
| 18 | HV_OPENING | BMS HV_Opening_Warn | 0 | B | no |
| 19 | RAIL_UV | V24 < 21.6 or V12 < 11.0 or V5 < 4.6 V | 0.2 s | B | no |
| 20 | ENCL_TEMP_WARN | T_encl > 70 degC | 5 s | C (derate 0.5) | no |
| 21 | ENCL_TEMP_CRIT | T_encl > 80 degC | 5 s | B | yes |
| 22 | STOP_FAIL | MCU stop monitor, cases (a) to (c) in the note below | 0 | A | yes |
| 23 | TASK_OVERRUN | 3 consecutive overruns | 0 | B, and in maskDO0: DO0 drops and the relay removes STO at once; bit 31 then gives FAULT | yes |
| 24 | IMU_FAULT | IMU invalid (`sens_acq_imu`: b6 SIM, or abs(acc) outside [5, 15] m/s^2, or all 9 raw axes bit-identical for `P.electronics.safety.imu_frozen_t` 0.1 s) | 0.5 s | C (derate 0.25, yaw loop off); decided: B in AUTO_START and AUTO, latched until an operator reset (D85, D90; see the note), pending on card G2-MCU | no |
| 25 | CAN1_BUSOFF | FDCAN1_PSR.BO (HW) / `fault_inject` b4 = `flt.can1_busoff` (SIM) | 0 | A | yes |
| 26 | CAN2_BUSOFF | FDCAN2_PSR.BO (HW) / `fault_inject` b5 = `flt.can2_busoff` (SIM) | 0 | B | yes |
| 27 | SAFETY_CHAIN_DISCREPANCY | ESTOP_CH1_OK != CH2_OK | 0.05 s | A | yes |
| 28 | IMPL_FAULT | IMPL_FB fault or timeout 0.2 s | 0.2 s | C (implement off) | yes |
| 29 | BUMPER | DI bumper | 0 | A | yes |
| 30 | LID_OPEN | DI LID_CLOSED low | 1 s | C (AUTO inhibit) | no |
| 31 | SAFETY_RELAY_OPEN | DI SAFETY_OK low (STO active) **while ACTIVE** (MANUAL/HOLD/AUTO_START/AUTO/CTRL_STOP); in BOOT/SELFTEST/STANDBY/ESTOP/FAULT an open relay only inhibits arming | 0 | A | no |

Class A bits 0, 27, 29, 31 are consequences of the hardware chain: they stop the robot (ESC disable)
but are excluded from DO0 (`maskDO0`), otherwise the loop could never re-close (D30).

maskDO0 = bits 5-8, 22, 23, 25 (D71). Bit 22 is not a hardware-chain consequence, so it drops DO0.

Notes to bits 14, 15, 16, 22 and 24 (D71, D73, D74; bit 24 also D85, D90, D104):
- Bit 16 (D71, D74). (b) The descent trip sets bit 16 in the same sample, with no persistence. The predicted body speed is abs(v) + max(a_track·sign(v), 0)·`t_pred` (`lim.descent.t_pred`: 0.08 s today; 0.09 s decided for REQ-SYS-031, pending on card G2-MCU); the trip fires when it is above the descent cap of the measured pitch (`lim.descent.ref_*` and `trip_*`: the cap; 1.44 m/s at a top-speed entry), and either the speed reference is steady or rising, or the speed is 0.2 m/s above a falling reference. The reference is falling when it fell by more than `a_fall` 0.1 m/s² × Ts in one sample, or it is more than `dv_fall` 0.02 m/s above the cap's speed target. Hill start: while a brake is engaged, and for `lim.descent.t_release` 0.5 s after both brakes release (`drive_fb` brake_released), with the reference in the direction of travel at most `yaw.cmd_still_v`, the `t_pred` term is dropped; the trip on the measured speed stays. (c) The speed-follow term, in the driving modes only (MANUAL, HOLD, AUTO_START, AUTO; previous sample's mode): the faster track above its held reference rf = max(abs(vtrack_ref), min(rf_prev, abs(v_track))) by more than 0.2 m/s for 0.2 s. A CTRL_STOP or ESTOP.DECEL with bit 16 set, entered or during its deceleration, parks at once, with no regen deceleration; so does a remote E-stop with bit 16 set from ACTIVE or from a decelerating CTRL_STOP. From SELFTEST or STANDBY the remote E-stop goes to ESTOP directly, since the robot is not moving.
- Bit 22, STOP_FAIL: set by the mode manager's stop monitor, one sample after its inputs. (a) In CTRL_STOP, ESTOP.DECEL or the ESTOP park: abs(v) > v0 + 0.1 m/s, or, from 0.5 s after entry, abs(v) > max(v0 - 0.5 a t, 0.05 m/s), with a = 1.0 or 2.0 m/s^2. (b) Parking: no brake confirmation 0.5 s after park_req. (c) Motion above 0.05 m/s for 0.2 s: in STANDBY from entry; in ESTOP.SAFE and FAULT only after standstill was seen there. Latched.
- Bits 14 and 15: the detection holds only while bit 24 is clear (previous sample). The persistence timer holds, rather than restarts, on a sample with imu_valid false.
- Bit 24: the yaw PI resets on every invalid IMU sample. Class C is today's model. Decided by D85 and D90, pending on card G2-MCU: class B in AUTO_START and AUTO (a controlled stop, the park on the brakes, the mission ends), latched until an operator reset with a valid IMU; in RC driving the operator may still drive, at the descent cap of the 20 deg design slope with the quarter-speed derate (about 0.35 m/s; 0.15 m/s forward and 0.10 m/s in reverse with the full-tank sprayer). D104 sets the detection of a wrong but plausible IMU within 1.0 s (the method on card G2-FSC).

## 7. RC / SBUS channel map (TX16S Mode 2, FrSky SBUS 172..992..1811)

| Ch | Function | Control | Decode |
|---|---|---|---|
| 1 | steer | right stick X | x = sat((raw-992)/819.5); deadband 0.05 + expo 0.35; omega_rc = -x * w_max_man * s |
| 2 | throttle | right stick Y | deadband 0.05 + expo 0.20; v_rc = x * v_max_man * s |
| 3, 4 | spare sticks | left stick | - |
| 5 | mode | SA 3-pos | 1 MANUAL / 2 HOLD / 3 AUTO (debounced: a new position passes 30 to 40 ms after the change); forwarded as rc_mode_sw = pos-1 |
| 6 | arm | SF 2-pos | rising edge = arm_req (sticks neutral required for MANUAL) |
| 7 | speed scale | S1 pot | s = 0.25 + (x+1)/2 * 0.75 |
| 8 | implement | SB 3-pos | OFF / ON / ON + auto-spray |
| 9 | reset | SH momentary | fault / E-stop reset request |
| 10 | work lights | SD 2-pos | DO b7 |
| 11 | lift | SC 3-pos | UP / HOLD / DOWN (MANUAL only) |
| 12 | remote E-STOP | SE 2-pos (latching) | 1811 = STOP -> ESTOP state (software stop, not safety rated); threshold decode without tapped-delay debounce, only frames with failsafe = 0 and frame_lost = 0 |
| 13 | mission | SG 3-pos momentary | edge to pos 2 = START/RESUME (rc_aux_cmd 2), pos 3 = PAUSE (3) |
| 14 | return home | SI momentary | edge = RTH (6) |
| 15, 16 | spare | - | - |
| flags | b0 CH17, b1 CH18, b2 frame_lost, b3 failsafe | - | link lost after 0.10 s without a valid frame |

Switch decoding today: the 2-position switches (arm, reset, lights, return home) are decoded as x > 0 with no
debounce; the 3-position switches need 4 agreeing samples at 10 ms (`rcmap.debounce_n`: a new position must last 30 ms
to pass, and is accepted 30 to 40 ms after the change); CH12 has no
debounce, but fault bit 1 persists 40 ms. The missing debounce of the 2-position switches is an open item on card
G2-MCU.

Receiver failsafe (FLAG_PRESET after 1.0 s): sticks 992, mode 992 (HOLD), arm 172, **E-STOP 172 (RUN)**.
Failsafe frames raise RC_FAILSAFE (class B, controlled stop); link loss is detected earlier by RC_LINK_LOST
(rc_age > 0.10 s, detection <= 0.13 s after RF loss). The remote E-stop is never inferred from a
failsafe frame (D34). Latency (D37): TX frame emission -> DRV_CMD frame at the MCU output <= 28 ms
(14 ms frame RT + 3 ms bytes + 1 ms poll + <= 10 ms Task_10 phase).

## 8. Variant controls (plain double scalars in base, created by `shaddad_params`)

| Variable | Values | Activation | Used in |
|---|---|---|---|
| `VC_TARGET` | 0 SIM / 1 HW | update diagram | IO_In / IO_Out of Shaddad_MCU, Shaddad_BMS; Companion_IO; NavFusion / Perception sensor acquisition |
| `VC_PHASE` | 1 RC only (GNC_Stub) / 2 GNC active | update diagram | Shaddad_System/GNC/GNC_Variant |
| `VC_STRUCT_FID` | 1 HiFi 6DOF / 2 LoFi kinematic | update diagram | structure_plant |
| `VC_IMPL` | 0 none / 1 mower / 2 tiller / 3 sprayer | update diagram | structure_implement, Power_Plant/Implement_Loads (both must agree: single variable) |
| `VC_IMPL_DRIVE` | 1 internal ideal speed source (structure harness only) / 2 POWER motor via Impl_Shaft | update diagram | structure_implement |
| `VC_PACK` | 1 Grouped (`Pack_Grouped.slx`) / 2 Lumped (`Pack_Lumped.slx`) | update diagram | Power_Plant/Battery_Pack |
| `VC_CHG` | 1 BMS-controlled current source / 2 `batt_lib/Cyclers/Charger` | update diagram | Power_Plant/Dock_Charger (the twin's dock; the robot has no dock in this phase, D97) |
| `VC_DRIVE_FID` | 1 SYS / 3 IDEAL (2 FOC = bench-only, rejected by `shaddad_params`, D52) | update diagram | drive_side/plant |
| `VC_OPSRC` | 1 scenario input / 2 dashboard / 3 joystick (`Joystick`: a transmitter in USB joystick mode through Simulation 3D Joystick + `joy_map`, mapping `P.scen.joy`; `vrlib/Joystick Input` was removed in R2026b) / 4 RC row test driver (`RowDriver`, scenario R1, `docs/row_test.md`) | update diagram | Shaddad_System/Operator_Console |
| `VC_LIDAR` | 1 FAST2D / 2 FULL3D | update diagram | sens_lidar_model, Shaddad_Perception scan source |
| `VC_ROW_TRACKER` | 1 Adaptive MPC / 2 Stanley | update diagram | Shaddad_GNC/PathTracker/RowTracker |
| `VC_VIZ` | 0 none / 1 bird's-eye MATLAB System / 2 XY Graph / 3 live 3-D robot (`shaddad.Robot3DView`, the live RC drive L1, `docs/live_rc_drive.md`) | update diagram | Viz_Logging |
| `VC_TEB` | 1 TEB in BYPASS (interpreted-only block, Normal/Accelerator) / 0 no TEB (VFH + Hybrid A* rejoin; endurance, rapid accelerator). Default by D99: 0 (VFH); 1 (TEB) is a development option, kept out of system runs until it passes T7 (decided, pending on card G2-GNC; `P.gnc.variant.teb` is 1 today) | update diagram | Shaddad_GNC/LocalAvoidance (D56) |

## 9. Enumeration codes (classes in `buses/*.m`, storage uint8, `addClassNameToEnumNames = true`)

| Enum | Codes |
|---|---|
| ShaddadMode | 0 BOOT, 1 SELFTEST, 2 STANDBY, 3 MANUAL, 4 HOLD, 5 AUTO_START, 6 AUTO, 7 CTRL_STOP, 8 ESTOP, 9 FAULT |
| ShaddadCmdSrc | 0 NONE, 1 RC, 2 AUTO |
| EscState | 0 INIT, 1 NOT_READY, 2 READY, 3 ENABLING, 4 OPERATION, 5 DISABLING, 6 QUICK_STOP, 7 FAULT, 8 STO |
| BmsContactorState | 0 INIT, 1 STANDBY, 2 PRECHARGE, 3 CLOSE_MAIN, 4 HV_ON, 5 OPENING, 6 WELD_CHECK, 7 PRE_FAIL, 8 CHARGING, 9 CHARGE_DONE, 10 FAULT |
| GncState | 0 IDLE, 1 MANUAL, 2 AUTO_MISSION, 3 PAUSE, 4 OBSTACLE_HOLD, 5 RETURN_TO_HOME, 6 FAULT |
| AutoSub | 0 NONE, 1 ALIGN, 2 TRANSIT, 3 ROW, 4 TURN, 5 BYPASS, 6 REJOIN, 7 RTH_PLANNING, 8 RTH_TRANSIT, 9 DOCK_APPROACH, 10 DOCKED |
| SegType | 0 NONE, 1 TRANSIT, 2 ROW, 3 TURN, 4 DOCK, 5 TERMINAL |
| MissionCmd | 0 NONE, 1 LOAD, 2 START, 3 PAUSE, 4 RESUME, 5 ABORT, 6 RTH, 7 RESET_FAULT |
| GncModeReq | 0 STOP, 1 MANUAL_ACK, 2 AUTO, 3 HOLD, 4 FAULT_STOP |
| TrackerSel | 0 STOP, 1 PP, 2 ROW, 3 TEB, 4 ALIGN |
| PathSrc | 0 MISSION, 1 REPLAN |
| HoldReason | 0 NONE, 1 OBSTACLE, 2 PERSON, 3 VFH_BLOCKED, 4 NAV_DEGRADED, 5 OPERATOR, 6 IMPLEMENT_NOT_READY, 7 RC_LOSS, 8 OBST_TIMEOUT, 9 PLANNER_FAIL, 10 SENSOR_LOST |
| NavQuality | 0 INVALID, 1 DR, 2 GNSS_SINGLE, 3 GNSS_FLOAT, 4 RTK_FIXED, 5 LIDAR_LOCALIZED |
| MissionType | 0 NONE, 1 PALM_ROW_PATROL, 2 GREENHOUSE_COVERAGE, 3 SOLAR_ROW_MOWING, 4 GOTO |
| ImplementType | 0 NONE, 1 MOWER, 2 TILLER, 3 SPRAYER |

## Notes to section 10: bus definitions that wait on a card

Section 10 is generated from `buses/buses_<domain>.m`, which card G1-01 part 2 does not change (the bus files are
the integration engineer's). These ranges and texts are stale. Each changes on its card, and the ICD is then
regenerated.

| Bus and element | Today | Should be | Card |
|---|---|---|---|
| `Bus_DriveSideCmd.w_ref` | range +/-502.7 rad/s (4800 rpm, the former motor) | +/-418.9 rad/s, the motor's mechanical maximum (`n_mech_max` 4000 rpm) | G2-DRV |
| `Bus_DriveSideCmd.tau_ff`, `tau_lim_mot`, `tau_lim_reg`; `Bus_DriveSideFb.tau_avail` | range to 16 N m (the former motor) | at least 19.1 N m per side (`P.electronics.drv.T_peak`, the 1 kW units), or the u8 wire's 25.5 N m | G2-DRV |
| `Bus_GncCmd.v_cmd` | range [-0.5, 1.5] m/s, "envelope = MCU AUTO limits, D43" | [-0.5, 1.2] m/s (`P.electronics.lim.v_max_auto` 1.2) | G2-GNC |
| `Bus_ImplMechState.tankLevel`, `tankMass` | range [0, 100] | [0, 40] (the 40 L tank) | G2-STR |
| `Bus_ImplMechState.liftPos01` | "normalised 0 (up) .. 1 (down)": the twin's sense, the opposite of the MCU's and the sensor's | a height fraction, 0 at the bottom and 1 at the top, as the potentiometer and the MCU read it (the lead systems engineer's ruling of step 19, point 22) | G2-STR |
| `Bus_BMS_Meas.Safety_HV_OK` | "Safety relay delayed (cat-1) contacts closed" | "Safety relay delayed contacts closed (the 48 V hold-up, D86)" | G2-PWR |
| `Bus_NavState.navValid` | no description | D89's health flag: aligned and running, the IMU valid, an absolute update within `tAbsMax` 35 s (REQ-SYS-023; 10 s in the parameters today) and the position sigma under `sigPosValid` 2 m; the accuracy gate is the GNC's (D69) | G2-NAV |
| The producer and consumer texts of `Bus_DriveSideCmd`, `Bus_DriveSideFb`, `Bus_Can1_McuFast`, `Bus_Can1_EscFast` and `Bus_Can1_EscSlow` | the former codecs `drive_can_pack` / `drive_can_unpack` | `drive_cmd_pack` (MCU) / `drive_cmd_unpack` (into `drive_system`) and `drive_fb_pack` (out of `drive_system`) / `drive_fb_unpack` (MCU) | G2-DRV |


## 10. Bus definitions (generated from buses/buses_<domain>.m)

88 buses. Ts = nominal producer sample time (0 continuous, Inf constant, inherited = sub-bus type used inside other buses).

### 10.1 Index

| Bus | Owner | Ts [s] | Producer | Consumers | Elements |
|---|---|---|---|---|---|
| [Bus_OperatorCmd](#bus_operatorcmd) | integration | 0.01 | Shaddad_System/Operator_Console (sys.operator.op) | Shaddad_System/Operator_RC/RC_Link_Sim | 15 |
| [Bus_HmiInputs](#bus_hmiinputs) | integration | 0.01 | Shaddad_System root ScenarioIn.hmi | Power_Plant/Safety_Chain, Electronics_Plant/HW_Frontend_Sim | 8 |
| [Bus_FaultInject](#bus_faultinject) | integration | 0.01 | Shaddad_System root ScenarioIn.flt | RC_Link_Sim, Shaddad_SensorModels, Shaddad_Plant (drive/power/electronics), CAN1_Net_Sim, Link_MCU_GNC, MCU_In assembly | 22 |
| [Bus_ObstaclesDyn](#bus_obstaclesdyn) | integration | 0.01 | Shaddad_System root ScenarioIn.obs | Shaddad_SensorModels (LiDAR), Sim_Monitors (collision) | 4 |
| [Bus_EnvOverride](#bus_envoverride) | integration | 0.01 | Shaddad_System root ScenarioIn.env | structure_env (Structure) | 7 |
| [Bus_ScenarioIn](#bus_scenarioin) | integration | 0.01 | Simulink.SimulationInput.setExternalInput (root In Bus Element ScenarioIn) | Operator_Console, Shaddad_Plant, Shaddad_SensorModels, CAN nets, GNC_Variant | 6 |
| [Bus_SimKPI](#bus_simkpi) | integration | 0.1 | Sim_Monitors | logging, eval_criteria | 14 |
| [Bus_TruthState](#bus_truthstate) | structure | 0.001 | structure_plant/structure_truth (exported by Shaddad_Plant at 1 ms via ZOH) | Shaddad_SensorModels, Sim_Monitors, Viz_Logging, logging (never used for control) | 23 |
| [Bus_TrackState](#bus_trackstate) | structure | 0.001 | structure_plant | logging, Sim_Monitors, tests | 14 |
| [Bus_ImplementCmd](#bus_implementcmd) | structure | 0.01 | Electronics_Plant/Actuator_Outputs_Sim (decoded MCU DO/PWM) | structure_plant/structure_implement | 3 |
| [Bus_ImplMechState](#bus_implmechstate) | structure | 0.01 | structure_plant/structure_implement | Power_Plant/Implement_Loads (ECU feedback), Electronics_Plant (lift pot ADC), logging | 13 |
| [Bus_EnvDisturbance](#bus_envdisturbance) | structure | 0.001 | test harness (Constant zero struct in system) | structure_plant | 2 |
| [Bus_EnvState](#bus_envstate) | structure | 0.1 | structure_plant/structure_env | Power_Plant, drive_system (T_amb), Electronics_Plant, Shaddad_SensorModels, logging | 7 |
| [Bus_StructureDiag](#bus_structurediag) | structure | 0.01 | structure_plant | logging, tests | 12 |
| [Bus_BMS_Meas](#bus_bms_meas) | power | 0.01 | Power_Plant/Power_Sensors | Shaddad_BMS (IO_In SIM) | 17 |
| [Bus_BMS_Cmd](#bus_bms_cmd) | power | 0.01 | Shaddad_BMS (IO_Out SIM) | Power_Plant (PDU coils via Safety_Chain, balancing, fans, charger) | 8 |
| [Bus_BMS_Telemetry](#bus_bms_telemetry) | power | 0.1 | Shaddad_BMS/BMS_Core/Telemetry | Shaddad_BMS CAN codec (frames 0x351..0x3C1), logging | 40 |
| [Bus_BmsReq](#bus_bmsreq) | power | 0.05 | Shaddad_MCU (CAN1 frame 0x3D0 BMS_REQ, decoded in Shaddad_BMS) | Shaddad_BMS/BMS_Core/ModeManager | 5 |
| [Bus_BmsStatus](#bus_bmsstatus) | power | 0.1 | Shaddad_MCU (decoded CAN1 BMS frames) / Companion_IO (decoded CAN2 MCU_POWER) | Shaddad_MCU tasks (torque limiter, supervisor), Shaddad_GNC, Shaddad_NavFusion | 28 |
| [Bus_PowerHwCmd](#bus_powerhwcmd) | power | 0.001 | Electronics_Plant/Actuator_Outputs_Sim (MCU DO/PWM decode) | Power_Plant | 6 |
| [Bus_LV_LoadDemand](#bus_lv_loaddemand) | power | 0.01 | Electronics_Plant/LV_Load_Demand | Power_Plant/LV_Distribution (Dynamic Load PS inputs) | 4 |
| [Bus_PowerStatus](#bus_powerstatus) | power | 0.001 | Power_Plant/Power_Sensors | Electronics_Plant/HW_Frontend_Sim (ADC, DI), logging | 27 |
| [Bus_ImplElecState](#bus_implelecstate) | power | 0.01 | Power_Plant/Implement_Loads (implement ECU + motor) | Power_Plant (IMPL_FB frame codec), logging | 10 |
| [Bus_PowerTruth](#bus_powertruth) | power | 0.1 | Power_Plant (Probe / PS-Simulink, decimated) | Sim_Monitors (energy audit), logging | 24 |
| [Bus_DriveSideCmd](#bus_drivesidecmd) | drive | 0.01 | Shaddad_MCU/Task_10/Drive_Cmd (decoded inside drive_system by drive_can_unpack) | drive_system/drive_esc | 9 |
| [Bus_DriveCmd](#bus_drivecmd) | drive | 0.01 | Shaddad_MCU | drive_system | 2 |
| [Bus_DriveSideFb](#bus_drivesidefb) | drive | 0.01 | drive_system/drive_esc (packed by drive_can_pack, decoded in Shaddad_MCU) | Shaddad_MCU (limiter, supervisor, odometry) | 15 |
| [Bus_DriveFb](#bus_drivefb) | drive | 0.01 | Shaddad_MCU (decoder) | Shaddad_MCU tasks | 2 |
| [Bus_DriveSideTruth](#bus_drivesidetruth) | drive | 0.001 | drive_system/drive_side (PS-Simulink) | Shaddad_SensorModels (mag disturbance), Sim_Monitors, logging | 19 |
| [Bus_DriveTruth](#bus_drivetruth) | drive | 0.001 | drive_system | Shaddad_SensorModels, Sim_Monitors, logging | 2 |
| [Bus_DriveSideFaultInj](#bus_drivesidefaultinj) | drive | 0.001 | Shaddad_Plant/Plant_In (mapped from Bus_FaultInject) | drive_system/drive_side | 5 |
| [Bus_DriveFaultInj](#bus_drivefaultinj) | drive | 0.001 | Shaddad_Plant/Plant_In | drive_system | 2 |
| [Bus_SbusStream](#bus_sbusstream) | electronics / rc | 0.001 | RC_Link_Sim/UART_Line_Sim (SIM) \| STM32 UART/USART Read USART2 (HW) | Shaddad_MCU/Task_1k/SBUS_Decoder | 3 |
| [Bus_RcFrame](#bus_rcframe) | electronics / rc | 0.014 | RC_Link_Sim/RC_Receiver | RC_Link_Sim/SBUS_Encoder, logging | 6 |
| [Bus_CanFrame8](#bus_canframe8) | electronics / rc | inherited | CAN codec subsystems (Byte Pack) | CAN codec subsystems (Byte Unpack) | 4 |
| [Bus_CanFrame64](#bus_canframe64) | electronics / rc | inherited | CAN-FD codec subsystems (Byte Pack) | CAN-FD codec subsystems (Byte Unpack) | 4 |
| [Bus_Can1_McuFast](#bus_can1_mcufast) | electronics / rc | 0.01 | Shaddad_MCU/Task_10 (drive_can_pack) | CAN1_Net_Sim -> drive_system (drive_can_unpack) | 2 |
| [Bus_Can1_McuSlow](#bus_can1_mcuslow) | electronics / rc | 0.05 | Shaddad_MCU/Task_50 | CAN1_Net_Sim -> Power_Plant/Implement ECU (impl_cmd), Shaddad_BMS (bms_req) | 2 |
| [Bus_Can1_EscFast](#bus_can1_escfast) | electronics / rc | 0.01 | drive_system (drive_can_pack) | CAN1_Net_Sim -> Shaddad_MCU | 4 |
| [Bus_Can1_EscSlow](#bus_can1_escslow) | electronics / rc | 0.05 | drive_system (drive_can_pack) | CAN1_Net_Sim -> Shaddad_MCU | 2 |
| [Bus_Can1_Impl](#bus_can1_impl) | electronics / rc | 0.05 | Power_Plant/Implement_Loads (implement ECU codec) | CAN1_Net_Sim -> Shaddad_MCU | 1 |
| [Bus_Can1_Bms](#bus_can1_bms) | electronics / rc | 0.1 | Shaddad_BMS (IO_Out SIM, BMS CAN codec) | CAN1_Net_Sim -> Shaddad_MCU | 7 |
| [Bus_McuIn1k](#bus_mcuin1k) | electronics / rc | 0.001 | Shaddad_System/MCU/MCU_In (RT + Signal Specification) | Shaddad_MCU IO_In | 5 |
| [Bus_McuIn5](#bus_mcuin5) | electronics / rc | 0.005 | Shaddad_System/MCU/MCU_In | Shaddad_MCU IO_In | 1 |
| [Bus_McuIn10](#bus_mcuin10) | electronics / rc | 0.01 | Shaddad_System/MCU/MCU_In | Shaddad_MCU IO_In | 1 |
| [Bus_McuIn50](#bus_mcuin50) | electronics / rc | 0.05 | Shaddad_System/MCU/MCU_In | Shaddad_MCU IO_In | 5 |
| [Bus_McuIn100](#bus_mcuin100) | electronics / rc | 0.1 | Shaddad_System/MCU/MCU_In | Shaddad_MCU IO_In | 1 |
| [Bus_McuOut1k](#bus_mcuout1k) | electronics / rc | 0.001 | Shaddad_MCU IO_Out | Electronics_Plant (Actuator_Outputs_Sim, Watchdog_Reset_Sim) | 2 |
| [Bus_McuOut5](#bus_mcuout5) | electronics / rc | 0.005 | Shaddad_MCU IO_Out | Link_MCU_GNC | 1 |
| [Bus_McuOut10](#bus_mcuout10) | electronics / rc | 0.01 | Shaddad_MCU IO_Out | CAN1_Net_Sim, Link_MCU_GNC, Electronics_Plant (PWM) | 3 |
| [Bus_McuOut50](#bus_mcuout50) | electronics / rc | 0.05 | Shaddad_MCU IO_Out | CAN1_Net_Sim, Link_MCU_GNC | 2 |
| [Bus_McuOut100](#bus_mcuout100) | electronics / rc | 0.1 | Shaddad_MCU IO_Out | Link_MCU_GNC | 1 |
| [Bus_FwTelemetry](#bus_fwtelemetry) | electronics / rc | 0.01 | Shaddad_MCU/Task_10/Telemetry | logging, Sim_Monitors, Viz_Logging | 38 |
| [Bus_McuStatus](#bus_mcustatus) | electronics / rc | 0.05 | Companion_IO (decoded CAN2 0x200 MCU_STATUS) | Shaddad_GNC | 26 |
| [Bus_ElecHwIn](#bus_elechwin) | electronics / rc | 0.001 | Electronics_Plant/HW_Frontend_Sim + Watchdog_Reset_Sim | Shaddad_System/MCU/MCU_In (Bus_McuIn1k), CAN1_Net_Sim, Link_MCU_GNC (through Shaddad_System/Plant/ElecHw_Delay, sys.plant.elec_hw_d) | 3 |
| [Bus_ElecAdc](#bus_elecadc) | electronics / rc | 0.05 | Electronics_Plant/HW_Frontend_Sim | Shaddad_System/MCU/MCU_In (Bus_McuIn50) | 2 |
| [Bus_ElecEnv](#bus_elecenv) | electronics / rc | 0.1 | Electronics_Plant (Electronics_Thermal, NTRIP link model, rail OK) | Shaddad_SensorModels | 5 |
| [Bus_IndicatorOut](#bus_indicatorout) | electronics / rc | 0.1 | Electronics_Plant/Actuator_Outputs_Sim | Viz_Logging, logging | 6 |
| [Bus_ImuRaw](#bus_imuraw) | sensors | 0.005 | Shaddad_SensorModels/sens_imu_model (SIM) \| ICM20948 IMU Sensor block (HW) | Shaddad_MCU IO_In -> sens_acq_imu | 4 |
| [Bus_GnssRaw](#bus_gnssraw) | sensors | 0.1 | Shaddad_SensorModels/sens_gnss_model (SIM) \| ROS 2 ublox topics (HW) | Shaddad_NavFusion IO_In | 13 |
| [Bus_LidarScanRaw](#bus_lidarscanraw) | sensors | 0.1 | Shaddad_SensorModels/sens_lidar_model FAST2D | Shaddad_Perception IO_In | 4 |
| [Bus_LidarCloudRaw](#bus_lidarcloudraw) | sensors | 0.1 | Shaddad_SensorModels/sens_lidar_model FULL3D (UAV Scenario Lidar) \| ROS 2 /ouster/points | Shaddad_Perception IO_In (sens_lidar_cloud2scan) | 4 |
| [Bus_SensTruthDiag](#bus_senstruthdiag) | sensors | 0.1 | Shaddad_SensorModels | logging, tests | 8 |
| [Bus_SprayDemand](#bus_spraydemand) | sensors | 0.1 | Shaddad_SensorModels/sens_spray_vision (abstract camera + detector) \| ROS 2 vision node (HW) | Shaddad_GNC/ImplementManager | 4 |
| [Bus_ImuMeas](#bus_imumeas) | sensors | 0.005 | Companion_IO (decoded CAN2 0x201 MCU_IMU) | Shaddad_NavFusion | 7 |
| [Bus_OdomMeas](#bus_odommeas) | sensors | 0.01 | Companion_IO (decoded CAN2 0x202 MCU_ODOM; computed in Shaddad_MCU sens_odom_compute) | Shaddad_NavFusion | 5 |
| [Bus_NavCmd](#bus_navcmd) | sensors | 0.05 | Shaddad_GNC/MissionManager | Shaddad_NavFusion/NavModeManager | 3 |
| [Bus_NavState](#bus_navstate) | sensors | 0.01 | Shaddad_NavFusion/NavOutput | Shaddad_GNC, Shaddad_Perception (delayed), Viz_Logging, logging | 26 |
| [Bus_NavDiag](#bus_navdiag) | sensors | 0.01 | Shaddad_NavFusion | logging, tests | 8 |
| [Bus_ScanMatchMeas](#bus_scanmatchmeas) | sensors | 0.2 | Shaddad_Perception/sens_scanmatch_loc | Shaddad_NavFusion (EKF channel 5) | 6 |
| [Bus_TebMapInfo](#bus_tebmapinfo) | sensors | 0.1 | Shaddad_Perception/sens_occmap_update | Shaddad_GNC/LocalAvoidance (TEB MapInfo) | 5 |
| [Bus_RowFeature](#bus_rowfeature) | sensors | 0.1 | Shaddad_Perception/sens_row_detect | Shaddad_GNC (row following aid), logging | 8 |
| [Bus_Perception](#bus_perception) | sensors | 0.1 | Shaddad_Perception | Shaddad_GNC (corridor, VFH, TEB, hold logic), Viz_Logging | 14 |
| [Bus_GncCmd](#bus_gnccmd) | gnc | 0.05 | Shaddad_GNC/CommandShaping (packed by Companion_IO into CAN2 0x100 COMP_CMD) | Shaddad_MCU (decoded in Task_50) | 24 |
| [Bus_MissionCmd](#bus_missioncmd) | gnc | 0.2 | Shaddad_System root ScenarioIn.mcmd (ground station in HW) | Shaddad_GNC/InputConditioning | 5 |
| [Bus_MissionPlan](#bus_missionplan) | gnc | Inf | scripts/gnc_compileMission.m -> P.gnc.mission (Constant MissionSource inside Shaddad_GNC, no root port, D57) | Shaddad_GNC | 17 |
| [Bus_GncStatus](#bus_gncstatus) | gnc | 0.2 | Shaddad_GNC/StatusMetrics | Viz_Logging, Sim_Monitors, logging (HW: ROS 2 topic) | 24 |
| [Bus_GncDebug](#bus_gncdebug) | gnc | 0.05 | Shaddad_GNC | logging | 12 |
| [Bus_PathWindow](#bus_pathwindow) | gnc | 0.05 | Shaddad_GNC/PathServer/SegmentPrep | Shaddad_GNC/PathAnalyzer, PathTracker | 13 |
| [Bus_PathError](#bus_patherror) | gnc | 0.05 | Shaddad_GNC/PathAnalyzer (gnc_pathAnalyzer) | Shaddad_GNC MissionManager (delayed), InputConditioning (delayed), PathTracker, SpeedGovernor, LocalAvoidance | 11 |
| [Bus_FsmOut](#bus_fsmout) | gnc | 0.05 | Shaddad_GNC/MissionManager (gnc_missionFSM) | Shaddad_GNC subsystems | 22 |
| [Bus_ReplanResult](#bus_replanresult) | gnc | 0.05 | Shaddad_GNC/GlobalReplanner (gnc_globalReplanner) | Shaddad_GNC MissionManager (delayed), PathServer | 7 |
| [Bus_MpcPlant](#bus_mpcplant) | gnc | inherited | gnc_mpcPlantUpdate | Adaptive MPC Controller (model port) | 4 |
| [Bus_MpcNominal](#bus_mpcnominal) | gnc | inherited | gnc_mpcPlantUpdate | Adaptive MPC Controller (model port) | 4 |
| [Bus_MpcModel](#bus_mpcmodel) | gnc | 0.05 | Shaddad_GNC/PathTracker/RowTracker (gnc_mpcPlantUpdate) | Adaptive MPC Controller | 2 |
| [Bus_NavCond](#bus_navcond) | gnc | 0.05 | Shaddad_GNC/InputConditioning | Shaddad_GNC subsystems | 10 |
| [Bus_TebMapCrop](#bus_tebmapcrop) | gnc | 0.2 | Shaddad_GNC/LocalAvoidance/Bypass (crop of Bus_Perception.map) | Timed Elastic Band (TEB) MapInfo | 5 |
| [Bus_ObsFlags](#bus_obsflags) | gnc | 0.05 | Shaddad_GNC/LocalAvoidance | Shaddad_GNC MissionManager, SpeedGovernor | 5 |

### 10.2 Elements

<a id="bus_operatorcmd"></a>
#### Bus_OperatorCmd

Operator stick/switch intent fed to the simulated RC transmitter (Mode 2 TX16S-class).  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Shaddad_System/Operator_Console (sys.operator.op). **Consumers:** Shaddad_System/Operator_RC/RC_Link_Sim.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `stick_thr` | double | 1 | 1 | [-1, 1] | Throttle stick, + = forward |
| 2 | `stick_steer` | double | 1 | 1 | [-1, 1] | Steering stick, + = stick RIGHT = turn right (omega < 0) |
| 3 | `stick_aux_lv` | double | 1 | 1 | [-1, 1] | Left stick vertical (spare) |
| 4 | `stick_aux_lh` | double | 1 | 1 | [-1, 1] | Left stick horizontal (spare) |
| 5 | `knob_speed` | double | 1 | 1 | [-1, 1] | Speed-scale knob S1 (-1 = min 25 %, +1 = 100 %) |
| 6 | `sw_mode` | double | 1 | - | [0, 3] | CH5 SA 3-pos: 1 MANUAL, 2 HOLD, 3 AUTO (0 treated as 1) |
| 7 | `sw_arm` | double | 1 | - | [0, 1] | CH6 SF arm switch: 0 disarmed, 1 armed |
| 8 | `sw_impl` | double | 1 | - | [0, 3] | CH8 SB implement 3-pos: 1 OFF, 2 ON, 3 ON + auto-spray |
| 9 | `sw_lift` | double | 1 | - | [0, 3] | CH11 SC lift 3-pos: 1 UP, 2 HOLD, 3 DOWN |
| 10 | `sw_lights` | double | 1 | - | [0, 1] | CH10 SD work lights: 0 off, 1 on |
| 11 | `sw_estop` | double | 1 | - | [0, 1] | CH12 SE remote (software) E-STOP: 0 run, 1 STOP |
| 12 | `btn_reset` | double | 1 | - | [0, 1] | CH9 SH momentary fault/E-stop reset: 1 pressed |
| 13 | `sw_mission` | double | 1 | - | [0, 3] | CH13 SG 3-pos momentary: 1 none, 2 START/RESUME, 3 PAUSE |
| 14 | `btn_rth` | double | 1 | - | [0, 1] | CH14 SI momentary return-to-home: 1 pressed |
| 15 | `tx_power_on` | double | 1 | - | [0, 1] | Transmitter powered: 0 off (no RF frames), 1 on |

<a id="bus_hmiinputs"></a>
#### Bus_HmiInputs

Hard-wired operator inputs on the robot body. All false = normal operation.  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Shaddad_System root ScenarioIn.hmi. **Consumers:** Power_Plant/Safety_Chain, Electronics_Plant/HW_Frontend_Sim.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `estop_front_pressed` | boolean | 1 | - | - | Front mushroom E-stop pressed |
| 2 | `estop_rear_pressed` | boolean | 1 | - | - | Rear mushroom E-stop pressed |
| 3 | `wireless_estop_tripped` | boolean | 1 | - | - | Certified wireless E-stop (optional) tripped |
| 4 | `bumper_front` | boolean | 1 | - | - | Front bumper switch actuated |
| 5 | `bumper_rear` | boolean | 1 | - | - | Rear bumper switch actuated |
| 6 | `reset_btn` | boolean | 1 | - | - | Safety-relay reset button pressed (rising edge re-arms) |
| 7 | `key_off` | boolean | 1 | - | - | Key switch OFF (false = key ON) |
| 8 | `lid_open` | boolean | 1 | - | - | Electronics enclosure lid open |

<a id="bus_faultinject"></a>
#### Bus_FaultInject

Scenario fault injection flags (all false/0 = no fault).  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Shaddad_System root ScenarioIn.flt. **Consumers:** RC_Link_Sim, Shaddad_SensorModels, Shaddad_Plant (drive/power/electronics), CAN1_Net_Sim, Link_MCU_GNC, MCU_In assembly.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `rc_link_loss` | boolean | 1 | - | - | RF frames lost (receiver sets frame_lost, then failsafe after P.rc.rx_failsafe_time) |
| 2 | `rc_line_silent` | boolean | 1 | - | - | Receiver SBUS output stops (no bytes) |
| 3 | `rc_byte_err_rate` | double | 1 | 1 | [0, 1] | Probability of a corrupted SBUS byte |
| 4 | `gnss_outage` | boolean | 1 | - | - | No GNSS fix (fixType 0, newData false) |
| 5 | `gnss_force_float` | boolean | 1 | - | - | RTK fixed forced to float |
| 6 | `gnss_corr_loss` | boolean | 1 | - | - | NTRIP / RTK correction link lost (corrAge grows) |
| 7 | `imu_fault` | uint8 | 1 | - | [0, 3] | IMU fault: 0 none, 1 stuck, 2 bias step, 3 dropout (invalid) |
| 8 | `lidar_blackout` | boolean | 1 | - | - | LiDAR returns no points (valid false) |
| 9 | `enc_fault_L` | boolean | 1 | - | - | Left motor encoder lost (ESC ENCODER fault) |
| 10 | `enc_fault_R` | boolean | 1 | - | - | Right motor encoder lost |
| 11 | `motor_fault_L` | boolean | 1 | - | - | Left drive trips (tau_scale = 0, overcurrent fault) |
| 12 | `motor_fault_R` | boolean | 1 | - | - | Right drive trips |
| 13 | `brake_stuck_L` | boolean | 1 | - | - | Left parking brake stuck engaged |
| 14 | `brake_stuck_R` | boolean | 1 | - | - | Right parking brake stuck engaged |
| 15 | `estop_ch2_stuck` | boolean | 1 | - | - | E-stop channel 2 contact welded closed (discrepancy test) |
| 16 | `bms_fault` | boolean | 1 | - | - | Inject +0.5 V on cell-group 1 voltage measurement (BMS OV fault) |
| 17 | `can1_drop_mask` | uint32 | 1 | - | - | CAN1 frame drop: b0 DRV_CMD_L b1 DRV_CMD_R b2 FB1_L b3 FB1_R b4 FB3_L b5 FB3_R b6 FB2_L b7 FB2_R b8 IMPL_CMD b9 BMS_REQ b10 IMPL_FB b11 all BMS frames |
| 18 | `can2_loss` | boolean | 1 | - | - | All CAN-FD frames MCU<->companion lost |
| 19 | `can1_busoff` | boolean | 1 | - | - | MCU FDCAN1 bus-off: MCU_In sets fault_inject b4, CAN1_Net_Sim drops all MCU-originated CAN1 frames |
| 20 | `can2_busoff` | boolean | 1 | - | - | MCU FDCAN2 bus-off: MCU_In sets fault_inject b5, Link_MCU_GNC drops all CAN-FD frames |
| 21 | `task_stall` | uint8 | 1 | - | [0, 15] | MCU task alive-counter stall: b0 10 ms task, b1 50 ms task, b2 100 ms task, b3 force overrun |
| 22 | `dust_level` | double | 1 | 1 | [0, 1] | Airborne dust level for LiDAR clutter |

<a id="bus_obstaclesdyn"></a>
#### Bus_ObstaclesDyn

Dynamic obstacles (people, vehicles) in site ENU; rows > n are ignored.  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Shaddad_System root ScenarioIn.obs. **Consumers:** Shaddad_SensorModels (LiDAR), Sim_Monitors (collision).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `n` | uint8 | 1 | - | [0, 4] | Number of active dynamic obstacles |
| 2 | `xy` | double | [4 x 2] | m | - | Centre position [x_E y_N] per obstacle |
| 3 | `v_xy` | double | [4 x 2] | m/s | - | Velocity [vE vN] per obstacle |
| 4 | `r` | double | 4 | m | [0, 5] | Radius (cylinder model) |

<a id="bus_envoverride"></a>
#### Bus_EnvOverride

Scenario overrides of the environment.  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Shaddad_System root ScenarioIn.env. **Consumers:** structure_env (Structure).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `use_T` | boolean | 1 | - | - | Override ambient temperature |
| 2 | `T_amb` | double | 1 | K | [233.15, 343.15] | Ambient temperature when use_T |
| 3 | `use_G` | boolean | 1 | - | - | Override solar irradiance |
| 4 | `G_solar` | double | 1 | W/m^2 | [0, 1400] | Global horizontal irradiance when use_G |
| 5 | `use_soil` | boolean | 1 | - | - | Override soil class everywhere (contact model uses this soil id) |
| 6 | `soil_id` | uint8 | 1 | - | [0, 5] | Soil class 1..5 (P.world.soilTable) |
| 7 | `dock_contact_force` | boolean | 1 | - | - | Force dock contacts closed (charging tests without docking) |

<a id="bus_scenarioin"></a>
#### Bus_ScenarioIn

Root scenario input of Shaddad_System (ZOH, 100 Hz).  
**Owner:** integration. **Ts:** 0.01 s. **Producer:** Simulink.SimulationInput.setExternalInput (root In Bus Element ScenarioIn). **Consumers:** Operator_Console, Shaddad_Plant, Shaddad_SensorModels, CAN nets, GNC_Variant.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `op` | Bus: Bus_OperatorCmd | 1 | - | - | Operator intent |
| 2 | `hmi` | Bus: Bus_HmiInputs | 1 | - | - | Body-mounted E-stops, bumpers, key, reset |
| 3 | `flt` | Bus: Bus_FaultInject | 1 | - | - | Fault injection |
| 4 | `obs` | Bus: Bus_ObstaclesDyn | 1 | - | - | Dynamic obstacles |
| 5 | `env` | Bus: Bus_EnvOverride | 1 | - | - | Environment overrides |
| 6 | `mcmd` | Bus: Bus_MissionCmd | 1 | - | - | Ground-station mission command (GNC) |

<a id="bus_simkpi"></a>
#### Bus_SimKPI

Simulation key performance indicators and stop reason.  
**Owner:** integration. **Ts:** 0.1 s. **Producer:** Sim_Monitors. **Consumers:** logging, eval_criteria.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_mission` | double | 1 | s | - | Time since McuStatus.mode first became AUTO |
| 2 | `dist` | double | 1 | m | - | Integral of \|v\| of point R (truth) |
| 3 | `E_batt` | double | 1 | J | - | Net battery terminal energy = 3600*(power_truth.E_out_Wh - E_in_Wh), discharge positive |
| 4 | `E_regen` | double | 1 | J | - | Charge energy into the pack = 3600*power_truth.E_in_Wh |
| 5 | `eps_el` | double | 1 | 1 | - | Electrical energy-audit residual (architecture D40) |
| 6 | `eps_mech` | double | 1 | 1 | - | Mechanical energy-audit residual (architecture D40) |
| 7 | `clear_min` | double | 1 | m | - | Minimum footprint-to-obstacle separation so far |
| 8 | `collision` | boolean | 1 | - | - | Any collision now |
| 9 | `n_coll` | uint16 | 1 | - | - | Number of collision onsets |
| 10 | `soc_min` | double | 1 | 1 | [0, 1] | Running minimum true pack SOC |
| 11 | `T_cell_max` | double | 1 | K | - | Running maximum true cell temperature |
| 12 | `I_batt_peak` | double | 1 | A | - | Running maximum \|I_pack\| at 1 ms (= power_truth.I_pack_peak_A) |
| 13 | `out_of_map` | boolean | 1 | - | - | Point R outside the DEM extent |
| 14 | `stop_reason` | uint8 | 1 | - | - | 0 running, 1 StopTime, 2 mission done, 3 SoC floor, 4 collision, 5 NaN, 6 assertion, 7 wall-time |

<a id="bus_truthstate"></a>
#### Bus_TruthState

True vehicle state at reference point R, ENU/FLU.  
**Owner:** structure. **Ts:** 0.001 s. **Producer:** structure_plant/structure_truth (exported by Shaddad_Plant at 1 ms via ZOH). **Consumers:** Shaddad_SensorModels, Sim_Monitors, Viz_Logging, logging (never used for control).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t` | double | 1 | s | - | Simulation time |
| 2 | `pos_W` | double | 3 | m | - | Position of R in W (ENU) |
| 3 | `vel_W` | double | 3 | m/s | - | Velocity of R in W |
| 4 | `acc_W` | double | 3 | m/s^2 | - | Kinematic acceleration of R in W (gravity excluded) |
| 5 | `q_WB` | double | 4 | 1 | - | MATLAB frame-rotation quaternion W->B, scalar first (v_B = rotateframe(q,v_W)) |
| 6 | `R_WB` | double | [3 x 3] | 1 | - | DCM body->world (v_W = R_WB*v_B) |
| 7 | `eul` | double | 3 | rad | - | ZYX [psi theta phi]; psi CCW from East, theta + = nose DOWN, phi + = left side up |
| 8 | `vel_B` | double | 3 | m/s | - | Velocity of R in B (FLU) |
| 9 | `omega_B` | double | 3 | rad/s | - | Body rates [p q r] FLU, r > 0 = turning left |
| 10 | `alpha_B` | double | 3 | rad/s^2 | - | Angular acceleration in B |
| 11 | `acc_B` | double | 3 | m/s^2 | - | Kinematic acceleration of R in B (gravity excluded) |
| 12 | `specForce_B` | double | 3 | m/s^2 | - | PHYSICAL specific force at R in B (a - g; reads [0 0 +g] at rest, level) |
| 13 | `r_cg_B` | double | 3 | m | - | CG position w.r.t. R in B (FLU) |
| 14 | `lla` | double | 3 | - | - | Geodetic position of R [lat deg, lon deg, h m] (WGS-84) |
| 15 | `pose2D` | double | 3 | - | - | [x_E m, y_N m, psi rad] of R (GNC convention) |
| 16 | `speed_fwd` | double | 1 | m/s | - | vel_B(1) |
| 17 | `mass` | double | 1 | kg | [100, 400] | Current total mass (sprayer tank depletes) |
| 18 | `terrainH` | double | 1 | m | - | DEM height under R |
| 19 | `grade_long` | double | 1 | rad | - | Terrain slope along body x, + = uphill ahead |
| 20 | `grade_lat` | double | 1 | rad | - | Terrain cross slope, + = left side higher |
| 21 | `vBelt_LR` | double | 2 | m/s | - | Belt (sprocket pitch-line) speed [L R], + = forward |
| 22 | `vGround_LR` | double | 2 | m/s | - | Track ground speed along body x [L R] |
| 23 | `slip_LR` | double | 2 | 1 | - | (v_belt - v_ground)/max(\|v_belt\|,0.05) [L R] |

<a id="bus_trackstate"></a>
#### Bus_TrackState

Per-track terramechanics state; index 1 = left, 2 = right.  
**Owner:** structure. **Ts:** 0.001 s. **Producer:** structure_plant. **Consumers:** logging, Sim_Monitors, tests.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `v_belt` | double | 2 | m/s | - | Belt speed |
| 2 | `v_ground` | double | 2 | m/s | - | Ground speed of the track along body x |
| 3 | `slip` | double | 2 | 1 | - | Diagnostic slip ratio |
| 4 | `s_shear` | double | 2 | 1 | [-1, 1] | Normalised longitudinal shear state |
| 5 | `Fx` | double | 2 | N | - | Traction force on vehicle, + forward |
| 6 | `Fy` | double | 2 | N | - | Lateral force on vehicle (FLU y) |
| 7 | `Fn` | double | 2 | N | [0, inf] | Normal load per track |
| 8 | `sprAngle` | double | 2 | rad | - | Sprocket angle (unwrapped) |
| 9 | `sprSpeed` | double | 2 | rad/s | - | Sprocket speed, + forward |
| 10 | `sprTorque` | double | 2 | N*m | - | Sprocket input torque from DRIVE |
| 11 | `sinkage` | double | 2 | m | - | N-weighted mean penetration |
| 12 | `muPeak` | double | 2 | 1 | - | Load-weighted peak traction coefficient |
| 13 | `Mturn` | double | 1 | N*m | - | Lateral-shear turning moment about z |
| 14 | `Pslip` | double | 2 | W | - | Slip power Fx*(v_belt - v_ground) |

<a id="bus_implementcmd"></a>
#### Bus_ImplementCmd

Implement actuator commands seen by the mechanics.  
**Owner:** structure. **Ts:** 0.01 s. **Producer:** Electronics_Plant/Actuator_Outputs_Sim (decoded MCU DO/PWM). **Consumers:** structure_plant/structure_implement.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `nozzleOn` | boolean | 8 | - | - | Spray nozzle solenoid i open (DO bits 10..17) |
| 2 | `liftDuty` | double | 1 | 1 | [-1, 1] | Lift actuator duty, + = lower tool (DOWN), - = raise |
| 3 | `speedCmd` | double | 1 | rad/s | - | Harness only (VC_IMPL_DRIVE==1 ideal drive): shaft speed command; 0 in system |

<a id="bus_implmechstate"></a>
#### Bus_ImplMechState

Mechanical state of the active implement.  
**Owner:** structure. **Ts:** 0.01 s. **Producer:** structure_plant/structure_implement. **Consumers:** Power_Plant/Implement_Loads (ECU feedback), Electronics_Plant (lift pot ADC), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `type` | uint8 | 1 | - | [0, 3] | 0 none, 1 mower, 2 tiller, 3 sprayer |
| 2 | `shaftSpeed` | double | 1 | rad/s | - | Impl_Shaft speed (blade / tiller rotor-side motor shaft / pump) |
| 3 | `shaftTorque` | double | 1 | N*m | - | Load torque on Impl_Shaft |
| 4 | `mechPower` | double | 1 | W | - | shaftSpeed*shaftTorque |
| 5 | `toolHeight` | double | 1 | m | - | Mower deck height above ground or tiller depth (+ down) |
| 6 | `liftPos01` | double | 1 | 1 | [0, 1] | Lift actuator position normalised 0 (up) .. 1 (down) for the potentiometer |
| 7 | `liftMoving` | boolean | 1 | - | - | Lift actuator moving (\|duty\| > 0.05 and not at end stop) |
| 8 | `tankLevel` | double | 1 | L | [0, 100] | Sprayer tank liquid volume |
| 9 | `tankMass` | double | 1 | kg | [0, 100] | Sprayer tank liquid mass |
| 10 | `sprayFlow` | double | 1 | L/min | - | Total nozzle flow |
| 11 | `linePressure` | double | 1 | Pa | [0, 600000] | Boom line pressure |
| 12 | `vegDensity` | double | 1 | 1 | [0, 3] | Vegetation density at the deck (P.world.veg) |
| 13 | `F_impl_B` | double | 3 | N | - | Implement force on chassis, body FLU |

<a id="bus_envdisturbance"></a>
#### Bus_EnvDisturbance

External wrench at CG for drawbar / push tests.  
**Owner:** structure. **Ts:** 0.001 s. **Producer:** test harness (Constant zero struct in system). **Consumers:** structure_plant.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `F_ext_B` | double | 3 | N | - | External force at CG, body FLU |
| 2 | `M_ext_B` | double | 3 | N*m | - | External moment about CG, body FLU |

<a id="bus_envstate"></a>
#### Bus_EnvState

Ambient conditions at the robot and dock contact state (single producer of T_amb).  
**Owner:** structure. **Ts:** 0.1 s. **Producer:** structure_plant/structure_env. **Consumers:** Power_Plant, drive_system (T_amb), Electronics_Plant, Shaddad_SensorModels, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `T_amb` | double | 1 | K | [233.15, 343.15] | Ambient air temperature |
| 2 | `T_sky` | double | 1 | K | - | Effective sky temperature |
| 3 | `T_ground` | double | 1 | K | - | Ground surface temperature |
| 4 | `G_solar` | double | 1 | W/m^2 | [0, 1400] | Global horizontal irradiance at the robot (zone-attenuated) |
| 5 | `soil_id` | uint8 | 1 | - | [1, 5] | Soil class under R (1..5) |
| 6 | `dock_contact` | boolean | 1 | - | - | Charger contacts closed (within P.world.dock tolerance and stopped, or override) |
| 7 | `dist_dock` | double | 1 | m | - | Distance from R to dock pose |

<a id="bus_structurediag"></a>
#### Bus_StructureDiag

Contact-model diagnostics.  
**Owner:** structure. **Ts:** 0.01 s. **Producer:** structure_plant. **Consumers:** logging, tests.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `N_pt` | double | 14 | N | - | Normal force per contact point (1..7 left, 8..14 right) |
| 2 | `pen_pt` | double | 14 | m | - | Penetration per contact point |
| 3 | `sX` | double | 14 | 1 | [-1, 1] | Longitudinal component of the isotropic 2-D shear state per point (D39) |
| 4 | `sY` | double | 14 | 1 | [-1, 1] | Lateral component of the isotropic 2-D shear state per point (D39) |
| 5 | `slopeLong` | double | 1 | rad | - | Terrain slope along heading at R |
| 6 | `slopeLat` | double | 1 | rad | - | Terrain cross slope at R |
| 7 | `P_diss` | double | 1 | W | - | Track-terrain dissipation: 2-D shear slip + rolling resistance + internal track friction |
| 8 | `E_mech` | double | 1 | J | - | Kinetic (body + belts + sprocket/track inertia) + potential energy |
| 9 | `E_diss_J` | double | 1 | J | - | Integral of P_diss plus the work of the implement wrench on the chassis (D40) |
| 10 | `E_impl_shaft_J` | double | 1 | J | - | Integral of the implement shaft power absorbed by the load mechanics (D40) |
| 11 | `chi_eff` | double | 1 | 1 | - | Instantaneous effective gauge factor (v_bR - v_bL)/(B*r), NaN when \|r\| < 0.05 rad/s |
| 12 | `outOfMap` | boolean | 1 | - | - | R outside DEM extent (terrain clipped) |

<a id="bus_bms_meas"></a>
#### Bus_BMS_Meas

AFE / shunt / NTC / status measurements as seen by the BMS ECU (noise, offset, quantisation, ZOH).  
**Owner:** power. **Ts:** 0.01 s. **Producer:** Power_Plant/Power_Sensors. **Consumers:** Shaddad_BMS (IO_In SIM).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `V_cell_V` | single | 13 | V | [0, 5] | Cell-group voltages (AFE) |
| 2 | `T_cell_K` | single | 13 | K | - | Cell-group NTC temperatures (5 s lag) |
| 3 | `I_pack_A` | single | 1 | A | - | Pack current (shunt), + = discharge |
| 4 | `V_pack_V` | single | 1 | V | - | Pack terminal voltage |
| 5 | `V_bus_V` | single | 1 | V | - | DC bus voltage after K1 (precharge check) |
| 6 | `V_dock_V` | single | 1 | V | - | Dock / charger contact voltage |
| 7 | `T_amb_K` | single | 1 | K | - | Ambient NTC |
| 8 | `T_air_K` | single | 1 | K | - | Pack enclosure air NTC |
| 9 | `K1_fb` | boolean | 1 | - | - | Main contactor aux contact closed |
| 10 | `K2_fb` | boolean | 1 | - | - | Precharge relay aux contact closed |
| 11 | `K4_fb` | boolean | 1 | - | - | Charge contactor aux contact closed |
| 12 | `Dock_Present` | boolean | 1 | - | - | Dock contacts detected |
| 13 | `Safety_OK` | boolean | 1 | - | - | Safety relay immediate contacts closed |
| 14 | `Safety_HV_OK` | boolean | 1 | - | - | Safety relay delayed (cat-1) contacts closed |
| 15 | `Key_On` | boolean | 1 | - | - | Key switch on |
| 16 | `FuseMain_OK` | boolean | 1 | - | - | Main fuse intact |
| 17 | `Fuse_Blown` | uint8 | 1 | - | - | Fuse bit-word b0 MAIN b1 TL b2 TR b3 IMP b4 AUX b5 CHG b6 24IMP |

<a id="bus_bms_cmd"></a>
#### Bus_BMS_Cmd

BMS ECU hardware outputs.  
**Owner:** power. **Ts:** 0.01 s. **Producer:** Shaddad_BMS (IO_Out SIM). **Consumers:** Power_Plant (PDU coils via Safety_Chain, balancing, fans, charger).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `K1_Cmd` | boolean | 1 | - | - | Main contactor coil request (AND Safety_HV_OK in hardware) |
| 2 | `K2_Cmd` | boolean | 1 | - | - | Precharge relay coil request |
| 3 | `K4_Cmd` | boolean | 1 | - | - | Charge contactor coil request |
| 4 | `Bal_Cmd` | boolean | 13 | - | - | Passive balancing switch per cell group |
| 5 | `Fan_Cmd` | single | 1 | 1 | [0, 1] | Pack ventilation fan command |
| 6 | `Charger_Enable` | boolean | 1 | - | - | Dock charger output enable (via dock comms) |
| 7 | `Charger_I_Req_A` | single | 1 | A | [0, 30] | Charger current request |
| 8 | `Charger_V_Req_V` | single | 1 | V | [0, 55.9] | Charger voltage limit request |

<a id="bus_bms_telemetry"></a>
#### Bus_BMS_Telemetry

Full BMS telemetry (internal to the BMS ECU and logging).  
**Owner:** power. **Ts:** 0.1 s. **Producer:** Shaddad_BMS/BMS_Core/Telemetry. **Consumers:** Shaddad_BMS CAN codec (frames 0x351..0x3C1), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `AliveCounter` | uint8 | 1 | - | - | +1 per 0.1 s |
| 2 | `Contactor_State` | uint8 | 1 | - | [0, 10] | BmsContactorState: 0 INIT 1 STANDBY 2 PRECHARGE 3 CLOSE_MAIN 4 HV_ON 5 OPENING 6 WELD_CHECK 7 PRE_FAIL 8 CHARGING 9 CHARGE_DONE 10 FAULT |
| 3 | `HV_Ready` | boolean | 1 | - | - | K1 closed and precharged |
| 4 | `HV_Opening_Warn` | boolean | 1 | - | - | HV will open: ramp torque to 0 |
| 5 | `K1_Closed` | boolean | 1 | - | - | Aux contact |
| 6 | `K2_Closed` | boolean | 1 | - | - | Aux contact |
| 7 | `K4_Closed` | boolean | 1 | - | - | Aux contact |
| 8 | `Charging_Active` | boolean | 1 | - | - |  |
| 9 | `Charge_Complete` | boolean | 1 | - | - |  |
| 10 | `SOC_pack` | single | 1 | 1 | [0, 1] | Display SOC |
| 11 | `SOC_cell` | single | 13 | 1 | [0, 1] | EKF SOC per group |
| 12 | `SOC_cc` | single | 1 | 1 | [0, 1] | Coulomb-counting SOC (plausibility) |
| 13 | `SOH` | single | 1 | 1 | [0, 1.2] | State of health |
| 14 | `SOE` | single | 1 | 1 | [0, 1] | State of energy |
| 15 | `E_remaining_Wh` | single | 1 | W*h | - | Remaining usable energy |
| 16 | `Runtime_Remaining_s` | single | 1 | s | - | Runtime estimate at P_avg60 |
| 17 | `V_pack_V` | single | 1 | V | - |  |
| 18 | `V_bus_V` | single | 1 | V | - |  |
| 19 | `I_pack_A` | single | 1 | A | - | + = discharge |
| 20 | `P_pack_W` | single | 1 | W | - |  |
| 21 | `V_cell_min_V` | single | 1 | V | - |  |
| 22 | `V_cell_max_V` | single | 1 | V | - |  |
| 23 | `V_cell_avg_V` | single | 1 | V | - |  |
| 24 | `idx_cell_min` | uint8 | 1 | - | - | 1-based |
| 25 | `idx_cell_max` | uint8 | 1 | - | - | 1-based |
| 26 | `T_cell_min_K` | single | 1 | K | - |  |
| 27 | `T_cell_max_K` | single | 1 | K | - |  |
| 28 | `T_pack_air_K` | single | 1 | K | - |  |
| 29 | `I_dis_lim_A` | single | 1 | A | [0, 250] | Discharge current limit (DCL), magnitude |
| 30 | `I_chg_lim_A` | single | 1 | A | [0, 30] | Charge current limit (CCL), magnitude |
| 31 | `P_dis_lim_W` | single | 1 | W | - | 10 s discharge power capability |
| 32 | `P_chg_lim_W` | single | 1 | W | - | 10 s charge power capability |
| 33 | `V_chg_lim_V` | single | 1 | V | - | Charge voltage limit CVL |
| 34 | `V_dis_lim_V` | single | 1 | V | - | Discharge voltage limit DVL |
| 35 | `Balancing_Active` | boolean | 1 | - | - |  |
| 36 | `Balancing_Cmd` | boolean | 13 | - | - |  |
| 37 | `Fan_Cmd` | single | 1 | 1 | [0, 1] |  |
| 38 | `Fault_Level` | uint8 | 1 | - | [0, 3] | 0 none, 1 warn, 2 derate, 3 HV off |
| 39 | `Fault_Word` | uint32 | 1 | - | - | b0 CELL_OV b1 CELL_UV b2 OVERCURRENT b3 OVER_TEMP b4 UNDER_TEMP b5 V_SENSOR b6 PRECHARGE_FAIL b7 K1_WELD b8 FUSE_MAIN b9 FUSE_TL b10 FUSE_TR b11 FUSE_IMP b12 FUSE_AUX b13 FUSE_CHG b14 CHARGER_OVP b15 CELL_CONTACT b16 SAFETY_OPEN b17 LV24 b18 LV12 b19 LV5 b20 MCU_CMD_TIMEOUT |
| 40 | `Warning_Word` | uint32 | 1 | - | - | b0 UV b1 OV b2 OT(>50C) b3 SOC_low(<15%) b4 SOC_crit(<8%) b5 DERATE b6 IMBALANCE b7 FAN |

<a id="bus_bmsreq"></a>
#### Bus_BmsReq

MCU requests to the BMS (MCU never drives contactor coils directly).  
**Owner:** power. **Ts:** 0.05 s. **Producer:** Shaddad_MCU (CAN1 frame 0x3D0 BMS_REQ, decoded in Shaddad_BMS). **Consumers:** Shaddad_BMS/BMS_Core/ModeManager.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `AliveCounter` | uint8 | 1 | - | - | +1 per MCU 50 ms step; frozen 0.5 s => MCU_CMD_TIMEOUT |
| 2 | `HV_Enable_Req` | boolean | 1 | - | - | Request HV (precharge + main contactor) |
| 3 | `Charge_Enable_Req` | boolean | 1 | - | - | Allow charging when docked |
| 4 | `Fault_Reset_Req` | boolean | 1 | - | - | Rising edge clears latched BMS faults |
| 5 | `Shutdown_Req` | boolean | 1 | - | - | Controlled HV opening |

<a id="bus_bmsstatus"></a>
#### Bus_BmsStatus

Controller-side view of the BMS state (single). Fields not carried on a link are 0.  
**Owner:** power. **Ts:** 0.1 s. **Producer:** Shaddad_MCU (decoded CAN1 BMS frames) / Companion_IO (decoded CAN2 MCU_POWER). **Consumers:** Shaddad_MCU tasks (torque limiter, supervisor), Shaddad_GNC, Shaddad_NavFusion.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `V_pack` | single | 1 | V | - |  |
| 2 | `I_pack` | single | 1 | A | - | + = discharge |
| 3 | `P_pack` | single | 1 | W | - |  |
| 4 | `SOC` | single | 1 | 1 | [0, 1] |  |
| 5 | `SOH` | single | 1 | 1 | [0, 1.2] |  |
| 6 | `T_cell_max` | single | 1 | K | - |  |
| 7 | `T_cell_min` | single | 1 | K | - |  |
| 8 | `V_cell_min` | single | 1 | V | - |  |
| 9 | `V_cell_max` | single | 1 | V | - |  |
| 10 | `CVL` | single | 1 | V | - | Charge voltage limit |
| 11 | `DVL` | single | 1 | V | - | Discharge voltage limit |
| 12 | `CCL` | single | 1 | A | [0, inf] | Charge current limit (magnitude) |
| 13 | `DCL` | single | 1 | A | [0, inf] | Discharge current limit (magnitude) |
| 14 | `P_dis_lim` | single | 1 | W | - |  |
| 15 | `P_chg_lim` | single | 1 | W | - |  |
| 16 | `E_remaining_Wh` | single | 1 | W*h | - |  |
| 17 | `Runtime_s` | single | 1 | s | - |  |
| 18 | `alarm_word` | uint32 | 1 | - | - | = Bus_BMS_Telemetry.Fault_Word |
| 19 | `warn_word` | uint32 | 1 | - | - | = Bus_BMS_Telemetry.Warning_Word |
| 20 | `fault_level` | uint8 | 1 | - | - | 0..3 |
| 21 | `contactor_state` | uint8 | 1 | - | - | BmsContactorState |
| 22 | `hv_ready` | boolean | 1 | - | - |  |
| 23 | `hv_opening_warn` | boolean | 1 | - | - |  |
| 24 | `charging` | boolean | 1 | - | - |  |
| 25 | `charge_complete` | boolean | 1 | - | - |  |
| 26 | `bms_critical` | boolean | 1 | - | - | fault_level == 3 |
| 27 | `alive` | uint8 | 1 | - | - | Last BMS alive counter |
| 28 | `valid` | boolean | 1 | - | - | All BMS frames fresh (< 0.5 s) |

<a id="bus_powerhwcmd"></a>
#### Bus_PowerHwCmd

MCU hardware outputs that act on the power plant.  
**Owner:** power. **Ts:** 0.001 s. **Producer:** Electronics_Plant/Actuator_Outputs_Sim (MCU DO/PWM decode). **Consumers:** Power_Plant.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `MCU_Safety_OK` | boolean | 1 | - | - | MCU watchdog-relay contact in the E-stop loop (DO0); MCU can open, never close the loop |
| 2 | `Safety_Reset_Req` | boolean | 1 | - | - | Remote reset pulse (DO1), equivalent to the reset button |
| 3 | `ImpPwr24_Enable` | boolean | 1 | - | - | K3 24 V implement relay request (DO2), AND Safety_OK in hardware |
| 4 | `Spray_Pump_Duty` | double | 1 | 1 | [0, 1] | Pump PWM duty (TIM1 CH1) |
| 5 | `Spray_Nozzle_Cmd` | boolean | 8 | - | - | Nozzle solenoids (DO10..17) |
| 6 | `Lift_Duty` | double | 1 | 1 | [-1, 1] | Lift actuator duty, + = down (TIM1 CH2 with DO18 direction) |

<a id="bus_lv_loaddemand"></a>
#### Bus_LV_LoadDemand

Electronics rail power demand (W, not current: voltage-independent, no algebraic loop).  
**Owner:** power. **Ts:** 0.01 s. **Producer:** Electronics_Plant/LV_Load_Demand. **Consumers:** Power_Plant/LV_Distribution (Dynamic Load PS inputs).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `P_24V_W` | double | 1 | W | [0, 600] | 24 V rail demand (beacon, work lights, 3D LiDAR via LIDAR_PWR_EN); excludes pack fans, K-coils, implement 24 V |
| 2 | `P_12V_W` | double | 1 | W | [0, 300] | 12 V rail demand (MCU board, Jetson via COMPANION_PWR_EN, LTE/NTRIP modem, cameras, display, enclosure fans) |
| 3 | `P_5V_W` | double | 1 | W | [0, 25] | 5 V rail demand (RC receiver, both GNSS receivers) |
| 4 | `P_3V3_W` | double | 1 | W | [0, 1] | 3.3 V rail demand (IMU, CAN transceivers logic) |

<a id="bus_powerstatus"></a>
#### Bus_PowerStatus

True physical power-plant values; ELECTRONICS adds ADC quantisation.  
**Owner:** power. **Ts:** 0.001 s. **Producer:** Power_Plant/Power_Sensors. **Consumers:** Electronics_Plant/HW_Frontend_Sim (ADC, DI), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `V_pack_V` | double | 1 | V | - | Pack terminal voltage |
| 2 | `I_pack_A` | double | 1 | A | - | Pack current, + = discharge |
| 3 | `V_bus_V` | double | 1 | V | - | DC bus after K1 |
| 4 | `V_24V_V` | double | 1 | V | - |  |
| 5 | `I_24V_A` | double | 1 | A | - |  |
| 6 | `V_24IMP_V` | double | 1 | V | - | 24 V implement rail after K3 |
| 7 | `V_12V_V` | double | 1 | V | - |  |
| 8 | `I_12V_A` | double | 1 | A | - |  |
| 9 | `V_5V_V` | double | 1 | V | - |  |
| 10 | `V_3V3_V` | double | 1 | V | - |  |
| 11 | `Rail_24V_OK` | boolean | 1 | - | - | within +/-10 % |
| 12 | `Rail_12V_OK` | boolean | 1 | - | - |  |
| 13 | `Rail_5V_OK` | boolean | 1 | - | - |  |
| 14 | `Rail_3V3_OK` | boolean | 1 | - | - |  |
| 15 | `K1_Closed` | boolean | 1 | - | - |  |
| 16 | `K2_Closed` | boolean | 1 | - | - |  |
| 17 | `K3_Closed` | boolean | 1 | - | - |  |
| 18 | `K4_Closed` | boolean | 1 | - | - |  |
| 19 | `Safety_OK` | boolean | 1 | - | - | Immediate safety contacts (STO) |
| 20 | `Safety_HV_OK` | boolean | 1 | - | - | Delayed safety contacts (K1/K2 coil supply) |
| 21 | `EStop_Any` | boolean | 1 | - | - | Any E-stop / bumper / wireless trip active |
| 22 | `EStop_Ch1_OK` | boolean | 1 | - | - | E-stop loop channel 1 closed |
| 23 | `EStop_Ch2_OK` | boolean | 1 | - | - | E-stop loop channel 2 closed (can be stuck by fault injection) |
| 24 | `Safety_Latched_Off` | boolean | 1 | - | - | Relay latched off, waiting for reset |
| 25 | `Dock_Present` | boolean | 1 | - | - |  |
| 26 | `Key_On` | boolean | 1 | - | - |  |
| 27 | `Fuse_Blown` | uint8 | 1 | - | - | b0 MAIN b1 TL b2 TR b3 IMP b4 AUX b5 CHG b6 24IMP |

<a id="bus_implelecstate"></a>
#### Bus_ImplElecState

Electrical / controller state of the implement drive.  
**Owner:** power. **Ts:** 0.01 s. **Producer:** Power_Plant/Implement_Loads (implement ECU + motor). **Consumers:** Power_Plant (IMPL_FB frame codec), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `Impl_Type` | uint8 | 1 | - | [0, 3] | 0 none, 1 mower, 2 tiller, 3 sprayer |
| 2 | `Impl_Running` | boolean | 1 | - | - | Rotor/pump commanded on and turning |
| 3 | `Impl_Fault` | boolean | 1 | - | - | Implement ECU fault |
| 4 | `Impl_Ready` | boolean | 1 | - | - | Speed >= 90 % of target (mower/tiller) or pressure OK (sprayer) |
| 5 | `Impl_Speed_radps` | double | 1 | rad/s | - | Blade / tiller-rotor / pump speed |
| 6 | `Impl_Torque_Nm` | double | 1 | N*m | - | Motor torque |
| 7 | `Impl_Power_W` | double | 1 | W | - | Electrical input power |
| 8 | `Impl_Current_A` | double | 1 | A | - | DC input current |
| 9 | `Impl_Temp_K` | double | 1 | K | - | Motor temperature (lumped) |
| 10 | `Fault_Code` | uint16 | 1 | - | - | b0 OC b1 OV b2 UV b3 OT b4 CAN_TIMEOUT b5 STALL b6 SAFETY_OFF |

<a id="bus_powertruth"></a>
#### Bus_PowerTruth

Plant truth for evaluation only (double). Energy integrals and I_pack_peak_A are computed at 1 ms inside Power_Sensors (D40) and only sampled at 0.1 s.  
**Owner:** power. **Ts:** 0.1 s. **Producer:** Power_Plant (Probe / PS-Simulink, decimated). **Consumers:** Sim_Monitors (energy audit), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `SOC_true` | double | 13 | 1 | [0, 1] | True SOC per cell group |
| 2 | `V_cell_true` | double | 13 | V | - |  |
| 3 | `T_cell_true` | double | 13 | K | - |  |
| 4 | `SOC_pack_true` | double | 1 | 1 | [0, 1] | min-max normalised pack SOC (same formula as BMS) |
| 5 | `V_term_V` | double | 1 | V | - | Pack terminal voltage |
| 6 | `I_pack_A` | double | 1 | A | - | + = discharge |
| 7 | `P_pack_W` | double | 1 | W | - |  |
| 8 | `E_out_Wh` | double | 1 | W*h | - | Discharged energy |
| 9 | `E_in_Wh` | double | 1 | W*h | - | Charged energy (regen + charger) |
| 10 | `E_loss_batt_Wh` | double | 1 | W*h | - | Battery internal losses |
| 11 | `P_grid_W` | double | 1 | W | - | Charger grid power |
| 12 | `P_dcdc_loss_W` | double | 1 | W | - | Sum of DC-DC losses |
| 13 | `P_pdu_loss_W` | double | 1 | W | - | Fuses, shunt, contactors, cables |
| 14 | `E_trac_dc_Wh` | double | 1 | W*h | - | Integral of the DC power delivered to both traction ports DC48_TracL/R (1 ms) |
| 15 | `E_impl_dc_Wh` | double | 1 | W*h | - | Integral of the implement electrical input power (IMP48 + V24IMP loads, 1 ms) |
| 16 | `E_lv_Wh` | double | 1 | W*h | - | Integral of LV rail load power at the rails (Dynamic Loads, LDO, coil holds, pack fans, BMS supply) |
| 17 | `E_dcdc_loss_Wh` | double | 1 | W*h | - | Integral of P_dcdc_loss_W (1 ms) |
| 18 | `E_pdu_loss_Wh` | double | 1 | W*h | - | Integral of P_pdu_loss_W incl. precharge resistor and bus bleed (1 ms) |
| 19 | `E_chg_bus_Wh` | double | 1 | W*h | - | Integral of the charger power delivered into the bus at DOCK_P (1 ms) |
| 20 | `I_pack_peak_A` | double | 1 | A | - | Running maximum \|I_pack\| evaluated at 1 ms |
| 21 | `T_cell_max_K` | double | 1 | K | - |  |
| 22 | `T_cell_min_K` | double | 1 | K | - |  |
| 23 | `contactor_closed` | boolean | 1 | - | - | K1 closed |
| 24 | `fuse_blown` | uint8 | 1 | - | - | as Bus_PowerStatus.Fuse_Blown |

<a id="bus_drivesidecmd"></a>
#### Bus_DriveSideCmd

Command to one traction ESC (CiA-402 style speed mode).  
**Owner:** drive. **Ts:** 0.01 s. **Producer:** Shaddad_MCU/Task_10/Drive_Cmd (decoded inside drive_system by drive_can_unpack). **Consumers:** drive_system/drive_esc.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `enable` | boolean | 1 | - | - | Request OPERATION_ENABLED |
| 2 | `mode` | uint8 | 1 | - | [0, 2] | 0 OFF, 1 SPEED (default), 2 TORQUE |
| 3 | `w_ref` | single | 1 | rad/s | [-502.7, 502.7] | Motor-shaft speed setpoint (= P.drive.gear.k_v2w * v_track) |
| 4 | `tau_ff` | single | 1 | N*m | [-16, 16] | Torque feed-forward (SPEED) or setpoint (TORQUE), motor shaft |
| 5 | `tau_lim_mot` | single | 1 | N*m | [0, 16] | Motoring torque limit (magnitude) from MCU limiter |
| 6 | `tau_lim_reg` | single | 1 | N*m | [0, 16] | Regenerative torque limit (magnitude) from MCU limiter (BMS CCL) |
| 7 | `park_req` | boolean | 1 | - | - | Force brake engaged and torque off |
| 8 | `fault_reset` | boolean | 1 | - | - | Rising edge clears latched ESC faults |
| 9 | `alive` | uint8 | 1 | - | - | Rolling counter +1 per frame; frozen 100 ms => QUICK_STOP |

<a id="bus_drivecmd"></a>
#### Bus_DriveCmd

Left/right drive commands.  
**Owner:** drive. **Ts:** 0.01 s. **Producer:** Shaddad_MCU. **Consumers:** drive_system.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `L` | Bus: Bus_DriveSideCmd | 1 | - | - | Left ESC (node 1) |
| 2 | `R` | Bus: Bus_DriveSideCmd | 1 | - | - | Right ESC (node 2) |

<a id="bus_drivesidefb"></a>
#### Bus_DriveSideFb

Feedback from one ESC.  
**Owner:** drive. **Ts:** 0.01 s. **Producer:** drive_system/drive_esc (packed by drive_can_pack, decoded in Shaddad_MCU). **Consumers:** Shaddad_MCU (limiter, supervisor, odometry).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `state` | uint8 | 1 | - | [0, 8] | EscState: 0 INIT 1 NOT_READY 2 READY 3 ENABLING 4 OPERATION 5 DISABLING 6 QUICK_STOP 7 FAULT 8 STO |
| 2 | `status` | uint16 | 1 | - | - | b0 READY b1 OP_ENABLED b2 FAULT b3 DERATING b4 BRAKE_RELEASED b5 MODE_SPEED b6 MODE_TORQUE b7 TORQUE_LIMIT_ACTIVE b8 STO_ACTIVE b9 QUICKSTOP b10 STALL_WARNING |
| 3 | `fault_code` | uint16 | 1 | - | - | b0 OVERCURRENT b1 OVERVOLTAGE b2 UNDERVOLTAGE b3 MOTOR_OVERTEMP b4 ESC_OVERTEMP b5 OVERSPEED b6 CAN_TIMEOUT b7 ENCODER b8 BRAKE b9 STALL b10 STO_ACTIVE |
| 4 | `w_m` | single | 1 | rad/s | - | Measured motor speed (1 rpm resolution) |
| 5 | `enc_cnt` | int32 | 1 | - | - | Motor encoder count, 4096/rev, wraps; use wrap-safe difference |
| 6 | `tau_est` | single | 1 | N*m | - | Estimated motor torque |
| 7 | `tau_avail` | single | 1 | N*m | [0, 16] | Available motoring torque after all derating |
| 8 | `i_q` | single | 1 | A | - | q-axis current (peak) |
| 9 | `i_dc` | single | 1 | A | - | DC input current, + = discharge |
| 10 | `v_dc` | single | 1 | V | - | DC link voltage |
| 11 | `T_motor` | single | 1 | K | - | Winding sensor (5 s lag) |
| 12 | `T_esc` | single | 1 | K | - | Heat-sink temperature |
| 13 | `brake_released` | boolean | 1 | - | - | Parking brake released |
| 14 | `alive` | uint8 | 1 | - | - | ESC rolling counter |
| 15 | `rx_ok` | boolean | 1 | - | - | FB1 and FB3 fresh (< 50 ms), set by the MCU decoder |

<a id="bus_drivefb"></a>
#### Bus_DriveFb

Left/right ESC feedback.  
**Owner:** drive. **Ts:** 0.01 s. **Producer:** Shaddad_MCU (decoder). **Consumers:** Shaddad_MCU tasks.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `L` | Bus: Bus_DriveSideFb | 1 | - | - | Left ESC |
| 2 | `R` | Bus: Bus_DriveSideFb | 1 | - | - | Right ESC |

<a id="bus_drivesidetruth"></a>
#### Bus_DriveSideTruth

Plant truth of one drive (double).  
**Owner:** drive. **Ts:** 0.001 s. **Producer:** drive_system/drive_side (PS-Simulink). **Consumers:** Shaddad_SensorModels (mag disturbance), Sim_Monitors, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `w_m` | double | 1 | rad/s | - | Motor speed |
| 2 | `theta_m` | double | 1 | rad | - | Motor angle (unwrapped) |
| 3 | `w_spr` | double | 1 | rad/s | - | Sprocket (gearbox output) speed |
| 4 | `theta_spr` | double | 1 | rad | - | Sprocket angle (unwrapped) |
| 5 | `tau_m` | double | 1 | N*m | - | Motor shaft torque |
| 6 | `tau_spr` | double | 1 | N*m | - | Gearbox output torque |
| 7 | `tau_brake` | double | 1 | N*m | - | Brake friction torque |
| 8 | `f_brake_rel` | double | 1 | 1 | [0, 1] | Brake release fraction 0 engaged .. 1 released |
| 9 | `v_dc` | double | 1 | V | - | DC voltage at ESC terminals |
| 10 | `i_dc` | double | 1 | A | - | DC current into ESC, + = discharge |
| 11 | `p_dc` | double | 1 | W | - | DC power |
| 12 | `p_spr` | double | 1 | W | - | Sprocket mechanical power |
| 13 | `P_loss` | double | 1 | W | - | Motor + ESC losses |
| 14 | `T_winding` | double | 1 | K | - |  |
| 15 | `T_housing` | double | 1 | K | - |  |
| 16 | `T_esc` | double | 1 | K | - |  |
| 17 | `E_dc` | double | 1 | J | - | Integral of p_dc |
| 18 | `E_spr` | double | 1 | J | - | Integral of p_spr (sprocket mechanical energy, 1 ms) |
| 19 | `E_loss` | double | 1 | J | - | Integral of p_dc - p_spr (motor + ESC + gearbox + brake + coil losses, 1 ms) |

<a id="bus_drivetruth"></a>
#### Bus_DriveTruth

Left/right drive truth.  
**Owner:** drive. **Ts:** 0.001 s. **Producer:** drive_system. **Consumers:** Shaddad_SensorModels, Sim_Monitors, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `L` | Bus: Bus_DriveSideTruth | 1 | - | - | Left |
| 2 | `R` | Bus: Bus_DriveSideTruth | 1 | - | - | Right |

<a id="bus_drivesidefaultinj"></a>
#### Bus_DriveSideFaultInj

Drive fault injection for one side.  
**Owner:** drive. **Ts:** 0.001 s. **Producer:** Shaddad_Plant/Plant_In (mapped from Bus_FaultInject). **Consumers:** drive_system/drive_side.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can_drop` | boolean | 1 | - | - | Reserved (frame drops are injected in CAN1_Net_Sim); keep false |
| 2 | `enc_fail` | boolean | 1 | - | - | Encoder signal lost -> ESC ENCODER fault |
| 3 | `brake_stuck` | boolean | 1 | - | - | Brake forced engaged |
| 4 | `tau_scale` | double | 1 | 1 | [0, 2] | Multiplier on the torque reference (1 nominal, 0 = drive trip) |
| 5 | `T_offset` | double | 1 | K | - | Offset added to the winding temperature sensor |

<a id="bus_drivefaultinj"></a>
#### Bus_DriveFaultInj

Left/right drive fault injection.  
**Owner:** drive. **Ts:** 0.001 s. **Producer:** Shaddad_Plant/Plant_In. **Consumers:** drive_system.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `L` | Bus: Bus_DriveSideFaultInj | 1 | - | - | Left |
| 2 | `R` | Bus: Bus_DriveSideFaultInj | 1 | - | - | Right |

<a id="bus_sbusstream"></a>
#### Bus_SbusStream

Bytes received on the SBUS UART during the last 1 ms tick (mirrors UART Read).  
**Owner:** electronics / rc. **Ts:** 0.001 s. **Producer:** RC_Link_Sim/UART_Line_Sim (SIM) \| STM32 UART/USART Read USART2 (HW). **Consumers:** Shaddad_MCU/Task_1k/SBUS_Decoder.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `bytes` | uint8 | 32 | - | - | Received bytes, valid 1..len, rest 0 |
| 2 | `len` | uint8 | 1 | - | [0, 32] | Number of valid bytes this tick |
| 3 | `uart_err` | boolean | 1 | - | - | Parity / framing error in this chunk |

<a id="bus_rcframe"></a>
#### Bus_RcFrame

Receiver output frame (period P.rc.frame_period).  
**Owner:** electronics / rc. **Ts:** 0.014 s. **Producer:** RC_Link_Sim/RC_Receiver. **Consumers:** RC_Link_Sim/SBUS_Encoder, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `ch_raw` | uint16 | 16 | - | [0, 2047] | Raw SBUS channel values (172..1811, mid 992) |
| 2 | `ch17` | boolean | 1 | - | - | Digital channel 17 |
| 3 | `ch18` | boolean | 1 | - | - | Digital channel 18 |
| 4 | `frame_lost` | boolean | 1 | - | - | Flags bit2: RF frame lost |
| 5 | `failsafe` | boolean | 1 | - | - | Flags bit3: receiver failsafe active |
| 6 | `sent` | boolean | 1 | - | - | A frame is emitted in this period (false in NO_PULSES failsafe / line silent) |

<a id="bus_canframe8"></a>
#### Bus_CanFrame8

One classic CAN 2.0B frame slot (11-bit ID). valid = frame transmitted in this sample.  
**Owner:** electronics / rc. **Ts:** inherited s. **Producer:** CAN codec subsystems (Byte Pack). **Consumers:** CAN codec subsystems (Byte Unpack).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `id` | uint32 | 1 | - | [0, 2047] | 11-bit identifier |
| 2 | `len` | uint8 | 1 | - | [0, 8] | DLC |
| 3 | `data` | uint8 | 8 | - | - | Payload bytes (little-endian fields) |
| 4 | `valid` | boolean | 1 | - | - | Frame present in this sample |

<a id="bus_canframe64"></a>
#### Bus_CanFrame64

One CAN-FD frame slot (11-bit ID, BRS, up to 64 bytes).  
**Owner:** electronics / rc. **Ts:** inherited s. **Producer:** CAN-FD codec subsystems (Byte Pack). **Consumers:** CAN-FD codec subsystems (Byte Unpack).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `id` | uint32 | 1 | - | [0, 2047] | 11-bit identifier |
| 2 | `len` | uint8 | 1 | - | [0, 64] | Payload length (8,12,16,20,24,32,48,64) |
| 3 | `data` | uint8 | 64 | - | - | Payload bytes, zero padded |
| 4 | `valid` | boolean | 1 | - | - | Frame present in this sample |

<a id="bus_can1_mcufast"></a>
#### Bus_Can1_McuFast

CAN1 frames MCU -> traction ESCs, 100 Hz.  
**Owner:** electronics / rc. **Ts:** 0.01 s. **Producer:** Shaddad_MCU/Task_10 (drive_can_pack). **Consumers:** CAN1_Net_Sim -> drive_system (drive_can_unpack).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `drv_cmd_L` | Bus: Bus_CanFrame8 | 1 | - | - | 0x201 DRV_CMD_L |
| 2 | `drv_cmd_R` | Bus: Bus_CanFrame8 | 1 | - | - | 0x202 DRV_CMD_R |

<a id="bus_can1_mcuslow"></a>
#### Bus_Can1_McuSlow

CAN1 frames MCU -> implement ECU and BMS, 20 Hz.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Shaddad_MCU/Task_50. **Consumers:** CAN1_Net_Sim -> Power_Plant/Implement ECU (impl_cmd), Shaddad_BMS (bms_req).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `impl_cmd` | Bus: Bus_CanFrame8 | 1 | - | - | 0x140 IMPL_CMD |
| 2 | `bms_req` | Bus: Bus_CanFrame8 | 1 | - | - | 0x3D0 BMS_REQ |

<a id="bus_can1_escfast"></a>
#### Bus_Can1_EscFast

CAN1 frames ESCs -> MCU, 100 Hz.  
**Owner:** electronics / rc. **Ts:** 0.01 s. **Producer:** drive_system (drive_can_pack). **Consumers:** CAN1_Net_Sim -> Shaddad_MCU.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `fb1_L` | Bus: Bus_CanFrame8 | 1 | - | - | 0x181 DRV_FB1_L |
| 2 | `fb1_R` | Bus: Bus_CanFrame8 | 1 | - | - | 0x182 DRV_FB1_R |
| 3 | `fb3_L` | Bus: Bus_CanFrame8 | 1 | - | - | 0x381 DRV_FB3_L |
| 4 | `fb3_R` | Bus: Bus_CanFrame8 | 1 | - | - | 0x382 DRV_FB3_R |

<a id="bus_can1_escslow"></a>
#### Bus_Can1_EscSlow

CAN1 frames ESCs -> MCU, 20 Hz.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** drive_system (drive_can_pack). **Consumers:** CAN1_Net_Sim -> Shaddad_MCU.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `fb2_L` | Bus: Bus_CanFrame8 | 1 | - | - | 0x281 DRV_FB2_L |
| 2 | `fb2_R` | Bus: Bus_CanFrame8 | 1 | - | - | 0x282 DRV_FB2_R |

<a id="bus_can1_impl"></a>
#### Bus_Can1_Impl

CAN1 frame implement ECU -> MCU, 20 Hz.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Power_Plant/Implement_Loads (implement ECU codec). **Consumers:** CAN1_Net_Sim -> Shaddad_MCU.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `impl_fb` | Bus: Bus_CanFrame8 | 1 | - | - | 0x141 IMPL_FB |

<a id="bus_can1_bms"></a>
#### Bus_Can1_Bms

CAN1 frames BMS -> MCU, 10 Hz (Victron/Pylontech-style + Shaddad extensions).  
**Owner:** electronics / rc. **Ts:** 0.1 s. **Producer:** Shaddad_BMS (IO_Out SIM, BMS CAN codec). **Consumers:** CAN1_Net_Sim -> Shaddad_MCU.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `limits` | Bus: Bus_CanFrame8 | 1 | - | - | 0x351 BMS_LIMITS |
| 2 | `soc` | Bus: Bus_CanFrame8 | 1 | - | - | 0x355 BMS_SOC |
| 3 | `vit` | Bus: Bus_CanFrame8 | 1 | - | - | 0x356 BMS_VIT |
| 4 | `alarm` | Bus: Bus_CanFrame8 | 1 | - | - | 0x35A BMS_ALARM |
| 5 | `cells` | Bus: Bus_CanFrame8 | 1 | - | - | 0x373 BMS_CELLS |
| 6 | `state` | Bus: Bus_CanFrame8 | 1 | - | - | 0x3C0 BMS_STATE |
| 7 | `power` | Bus: Bus_CanFrame8 | 1 | - | - | 0x3C1 BMS_POWER |

<a id="bus_mcuin1k"></a>
#### Bus_McuIn1k

MCU inputs at the 1 kHz base task.  
**Owner:** electronics / rc. **Ts:** 0.001 s. **Producer:** Shaddad_System/MCU/MCU_In (RT + Signal Specification). **Consumers:** Shaddad_MCU IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `sbus` | Bus: Bus_SbusStream | 1 | - | - | SBUS UART chunk |
| 2 | `di_word` | uint16 | 1 | - | - | Digital inputs (docs/icd.md DI map) |
| 3 | `t_us` | uint32 | 1 | us | - | Free-running microsecond timestamp (wraps 71.6 min) |
| 4 | `reset_flag` | boolean | 1 | - | - | Watchdog reset seen (por pulse) - forces BOOT |
| 5 | `fault_inject` | uint32 | 1 | - | - | SIM only: b0 stall 10 ms task, b1 stall 50 ms, b2 stall 100 ms, b3 force overrun, b4 CAN1 bus-off, b5 CAN2 bus-off, b6 IMU invalid |

<a id="bus_mcuin5"></a>
#### Bus_McuIn5

MCU inputs at 200 Hz.  
**Owner:** electronics / rc. **Ts:** 0.005 s. **Producer:** Shaddad_System/MCU/MCU_In. **Consumers:** Shaddad_MCU IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `imu` | Bus: Bus_ImuRaw | 1 | - | - | ICM-20948 outputs (SIM: Shaddad_SensorModels) |

<a id="bus_mcuin10"></a>
#### Bus_McuIn10

MCU inputs at 100 Hz.  
**Owner:** electronics / rc. **Ts:** 0.01 s. **Producer:** Shaddad_System/MCU/MCU_In. **Consumers:** Shaddad_MCU IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can1_esc` | Bus: Bus_Can1_EscFast | 1 | - | - | ESC FB1/FB3 frames |

<a id="bus_mcuin50"></a>
#### Bus_McuIn50

MCU inputs at 20 Hz.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Shaddad_System/MCU/MCU_In. **Consumers:** Shaddad_MCU IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can1_escslow` | Bus: Bus_Can1_EscSlow | 1 | - | - | ESC FB2 frames |
| 2 | `can1_impl` | Bus: Bus_Can1_Impl | 1 | - | - | Implement ECU feedback frame |
| 3 | `can2_comp` | Bus: Bus_CanFrame64 | 1 | - | - | 0x100 COMP_CMD from the companion |
| 4 | `adc_raw` | uint16 | 8 | - | - | ADC1 counts [Vpack Vbus Ipack V24 V12 V5 NTC_encl LiftPot] |
| 5 | `mcu_temp_degC` | single | 1 | degC | - | MCU die temperature (ADC3) |

<a id="bus_mcuin100"></a>
#### Bus_McuIn100

MCU inputs at 10 Hz.  
**Owner:** electronics / rc. **Ts:** 0.1 s. **Producer:** Shaddad_System/MCU/MCU_In. **Consumers:** Shaddad_MCU IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can1_bms` | Bus: Bus_Can1_Bms | 1 | - | - | BMS frames |

<a id="bus_mcuout1k"></a>
#### Bus_McuOut1k

MCU outputs at 1 kHz.  
**Owner:** electronics / rc. **Ts:** 0.001 s. **Producer:** Shaddad_MCU IO_Out. **Consumers:** Electronics_Plant (Actuator_Outputs_Sim, Watchdog_Reset_Sim).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `do_word` | uint32 | 1 | - | - | Digital outputs (docs/icd.md DO map) |
| 2 | `wdg_kick` | boolean | 1 | - | - | IWDG1 refresh this tick |

<a id="bus_mcuout5"></a>
#### Bus_McuOut5

MCU outputs at 200 Hz.  
**Owner:** electronics / rc. **Ts:** 0.005 s. **Producer:** Shaddad_MCU IO_Out. **Consumers:** Link_MCU_GNC.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can2_imu` | Bus: Bus_CanFrame64 | 1 | - | - | 0x201 MCU_IMU |

<a id="bus_mcuout10"></a>
#### Bus_McuOut10

MCU outputs at 100 Hz.  
**Owner:** electronics / rc. **Ts:** 0.01 s. **Producer:** Shaddad_MCU IO_Out. **Consumers:** CAN1_Net_Sim, Link_MCU_GNC, Electronics_Plant (PWM).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can1_drv` | Bus: Bus_Can1_McuFast | 1 | - | - | Drive command frames |
| 2 | `can2_odom` | Bus: Bus_CanFrame64 | 1 | - | - | 0x202 MCU_ODOM |
| 3 | `pwm_duty` | single | 4 | 1 | [0, 1] | PWM duty 0..1 [pump TIM1CH1, lift TIM1CH2, fan1 TIM4CH1, fan2 TIM4CH2] |

<a id="bus_mcuout50"></a>
#### Bus_McuOut50

MCU outputs at 20 Hz.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Shaddad_MCU IO_Out. **Consumers:** CAN1_Net_Sim, Link_MCU_GNC.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can1_slow` | Bus: Bus_Can1_McuSlow | 1 | - | - | IMPL_CMD, BMS_REQ frames |
| 2 | `can2_status` | Bus: Bus_CanFrame64 | 1 | - | - | 0x200 MCU_STATUS |

<a id="bus_mcuout100"></a>
#### Bus_McuOut100

MCU outputs at 10 Hz.  
**Owner:** electronics / rc. **Ts:** 0.1 s. **Producer:** Shaddad_MCU IO_Out. **Consumers:** Link_MCU_GNC.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `can2_power` | Bus: Bus_CanFrame64 | 1 | - | - | 0x203 MCU_POWER (BMS forward) |

<a id="bus_fwtelemetry"></a>
#### Bus_FwTelemetry

Firmware internals for logging (single).  
**Owner:** electronics / rc. **Ts:** 0.01 s. **Producer:** Shaddad_MCU/Task_10/Telemetry. **Consumers:** logging, Sim_Monitors, Viz_Logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `mode` | uint8 | 1 | - | - | ShaddadMode |
| 2 | `cmd_src` | uint8 | 1 | - | - | ShaddadCmdSrc: 0 NONE 1 RC 2 AUTO |
| 3 | `fault_word` | uint32 | 1 | - | - | Supervisor fault word (docs/icd.md) |
| 4 | `warn_word` | uint32 | 1 | - | - | Supervisor warning word |
| 5 | `rc_ch_norm` | single | 16 | 1 | [-1, 1] | Normalised RC channels |
| 6 | `rc_link_ok` | boolean | 1 | - | - |  |
| 7 | `rc_failsafe` | boolean | 1 | - | - |  |
| 8 | `rc_frame_lost` | boolean | 1 | - | - |  |
| 9 | `rc_lq` | single | 1 | 1 | [0, 1] | Link quality |
| 10 | `rc_age` | single | 1 | s | - | Time since last good frame |
| 11 | `v_target` | single | 1 | m/s | - | Arbitrated target speed |
| 12 | `w_target` | single | 1 | rad/s | - | Arbitrated target yaw rate |
| 13 | `v_ref` | single | 1 | m/s | - | Shaped speed reference |
| 14 | `w_ref` | single | 1 | rad/s | - | Shaped yaw-rate reference |
| 15 | `a_ref` | single | 1 | m/s^2 | - |  |
| 16 | `alpha_ref` | single | 1 | rad/s^2 | - |  |
| 17 | `w_mix` | single | 1 | rad/s | - | Yaw-rate after yaw-rate PI correction |
| 18 | `yaw_rate_meas` | single | 1 | rad/s | - | Bias-corrected gyro z |
| 19 | `vtrack_ref` | single | 2 | m/s | - | Track speed references [L R] |
| 20 | `vtrack_meas` | single | 2 | m/s | - | Encoder track speeds [L R] |
| 21 | `w_ref_motor` | single | 2 | rad/s | - | ESC speed setpoints |
| 22 | `tau_ff` | single | 2 | N*m | - |  |
| 23 | `tau_lim_mot` | single | 2 | N*m | - |  |
| 24 | `tau_lim_reg` | single | 2 | N*m | - |  |
| 25 | `k_traction` | single | 2 | 1 | [0, 1] | Traction-control factor |
| 26 | `roll` | single | 1 | rad | - | Complementary-filter roll |
| 27 | `pitch` | single | 1 | rad | - | Complementary-filter pitch (+ nose down) |
| 28 | `V_pack` | single | 1 | V | - |  |
| 29 | `I_pack` | single | 1 | A | - |  |
| 30 | `SOC` | single | 1 | 1 | - |  |
| 31 | `T_encl_degC` | single | 1 | degC | - |  |
| 32 | `comp_age` | single | 1 | s | - | Age of last COMP_CMD |
| 33 | `sbus_ok` | uint32 | 1 | - | - | Good frames |
| 34 | `sbus_bad` | uint32 | 1 | - | - | Rejected frames |
| 35 | `sbus_skip` | uint32 | 1 | - | - | Bytes skipped while hunting header |
| 36 | `overruns` | uint32 | 1 | - | - | Task overruns |
| 37 | `power_ready` | boolean | 1 | - | - | BMS HV_Ready seen |
| 38 | `drive_state` | uint8 | 2 | - | - | EscState [L R] |

<a id="bus_mcustatus"></a>
#### Bus_McuStatus

MCU status as seen by the companion computer.  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Companion_IO (decoded CAN2 0x200 MCU_STATUS). **Consumers:** Shaddad_GNC.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_mcu` | double | 1 | s | - | MCU time (unwrapped t_ms) |
| 2 | `mode` | uint8 | 1 | - | - | ShaddadMode: 0 BOOT 1 SELFTEST 2 STANDBY 3 MANUAL 4 HOLD 5 AUTO_START 6 AUTO 7 CTRL_STOP 8 ESTOP 9 FAULT |
| 3 | `cmd_src` | uint8 | 1 | - | - | 0 NONE 1 RC 2 AUTO |
| 4 | `rc_mode_sw` | uint8 | 1 | - | [0, 2] | RC mode switch: 0 MANUAL, 1 HOLD, 2 AUTO |
| 5 | `rc_aux_cmd` | uint8 | 1 | - | - | MissionCmd from RC aux channels (0 NONE 2 START 3 PAUSE 6 RTH) |
| 6 | `rc_aux_seq` | uint8 | 1 | - | - | +1 on every new rc_aux_cmd edge |
| 7 | `rc_link_ok` | boolean | 1 | - | - |  |
| 8 | `rc_failsafe` | boolean | 1 | - | - |  |
| 9 | `estop_active` | boolean | 1 | - | - | HW or remote E-stop active |
| 10 | `drive_ready` | boolean | 1 | - | - | MCU in AUTO with both ESCs OPERATION/READY and HV ready |
| 11 | `power_ready` | boolean | 1 | - | - | BMS HV_Ready |
| 12 | `implement_ready` | boolean | 1 | - | - | Implement at speed / pressure |
| 13 | `charging` | boolean | 1 | - | - | BMS charging active |
| 14 | `wdg_reset_seen` | boolean | 1 | - | - |  |
| 15 | `implement_type` | uint8 | 1 | - | - | 0 none 1 mower 2 tiller 3 sprayer |
| 16 | `heartbeat` | uint8 | 1 | - | - | Rolling counter +1 per frame |
| 17 | `fault_word` | uint32 | 1 | - | - | MCU supervisor fault word |
| 18 | `warn_word` | uint32 | 1 | - | - | MCU supervisor warning word |
| 19 | `drive_fault_word` | uint32 | 1 | - | - | ESC fault codes: L in bits 0..15, R in bits 16..31 |
| 20 | `v_track_meas` | single | 2 | m/s | - | Encoder track speeds [L R] |
| 21 | `v_applied` | single | 1 | m/s | - | Shaped v_ref sent to the mixer |
| 22 | `w_applied` | single | 1 | rad/s | - | Applied yaw-rate reference |
| 23 | `gnc_cmd_age_ms` | uint16 | 1 | ms | - | Age of last valid COMP_CMD at the MCU |
| 24 | `rc_lq` | single | 1 | 1 | [0, 1] | RC link quality |
| 25 | `T_encl` | single | 1 | K | - | Electronics enclosure temperature |
| 26 | `valid` | boolean | 1 | - | - | Frame fresh at the companion (< 0.2 s) |

<a id="bus_elechwin"></a>
#### Bus_ElecHwIn

Digital front end and reset emulation.  
**Owner:** electronics / rc. **Ts:** 0.001 s. **Producer:** Electronics_Plant/HW_Frontend_Sim + Watchdog_Reset_Sim. **Consumers:** Shaddad_System/MCU/MCU_In (Bus_McuIn1k), CAN1_Net_Sim, Link_MCU_GNC (through Shaddad_System/Plant/ElecHw_Delay, sys.plant.elec_hw_d).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `di_word` | uint16 | 1 | - | - | Packed DI bits (docs/icd.md DI map) |
| 2 | `reset_flag` | boolean | 1 | - | - | Pulse (one tick) after an emulated watchdog reset |
| 3 | `mcu_reset` | boolean | 1 | - | - | MCU held in reset (all MCU TX frames dropped, DO forced 0) |

<a id="bus_elecadc"></a>
#### Bus_ElecAdc

ADC front-end counts (16-bit, Vref 3.3 V).  
**Owner:** electronics / rc. **Ts:** 0.05 s. **Producer:** Electronics_Plant/HW_Frontend_Sim. **Consumers:** Shaddad_System/MCU/MCU_In (Bus_McuIn50).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `adc_raw` | uint16 | 8 | - | - | [Vpack Vbus Ipack V24 V12 V5 NTC_encl LiftPot] |
| 2 | `mcu_temp_degC` | single | 1 | degC | - | MCU die temperature = T_encl + 15 K |

<a id="bus_elecenv"></a>
#### Bus_ElecEnv

Electronics-side conditions needed by the sensor models.  
**Owner:** electronics / rc. **Ts:** 0.1 s. **Producer:** Electronics_Plant (Electronics_Thermal, NTRIP link model, rail OK). **Consumers:** Shaddad_SensorModels.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `T_imu_degC` | double | 1 | degC | - | IMU (MCU PCB) temperature = T_encl + 5 K |
| 2 | `T_encl_K` | double | 1 | K | - | Enclosure air temperature |
| 3 | `gnssCorrOK` | boolean | 1 | - | - | RTK correction link up |
| 4 | `gnssCorrAge` | double | 1 | s | [0, inf] | Age of RTK corrections |
| 5 | `sensPowerOK` | uint8 | 1 | - | - | b0 IMU, b1 GNSS, b2 LiDAR powered (rails OK and DO enables) |

<a id="bus_indicatorout"></a>
#### Bus_IndicatorOut

Indicator and fan states.  
**Owner:** electronics / rc. **Ts:** 0.1 s. **Producer:** Electronics_Plant/Actuator_Outputs_Sim. **Consumers:** Viz_Logging, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `beacon_green` | boolean | 1 | - | - |  |
| 2 | `beacon_amber` | boolean | 1 | - | - |  |
| 3 | `beacon_red` | boolean | 1 | - | - |  |
| 4 | `buzzer` | boolean | 1 | - | - |  |
| 5 | `work_lights` | boolean | 1 | - | - |  |
| 6 | `fan_duty` | double | 2 | 1 | [0, 1] | Enclosure fans |

<a id="bus_imuraw"></a>
#### Bus_ImuRaw

Mirrors the ICM20948 IMU Sensor block outputs (sensor frame).  
**Owner:** sensors. **Ts:** 0.005 s. **Producer:** Shaddad_SensorModels/sens_imu_model (SIM) \| ICM20948 IMU Sensor block (HW). **Consumers:** Shaddad_MCU IO_In -> sens_acq_imu.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `Acceleration` | single | 3 | m/s^2 | [-39.2266, 39.2266] | Specific force, sensor axes (physical sign) |
| 2 | `AngularRate` | single | 3 | rad/s | [-8.7266, 8.7266] | Angular rate, sensor axes |
| 3 | `MagneticField` | single | 3 | uT | - | Magnetic field, sensor axes |
| 4 | `Temperature` | single | 1 | degC | - | Die temperature |

<a id="bus_gnssraw"></a>
#### Bus_GnssRaw

Dual-antenna RTK receiver output (primary antenna).  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_SensorModels/sens_gnss_model (SIM) \| ROS 2 ublox topics (HW). **Consumers:** Shaddad_NavFusion IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_meas` | double | 1 | s | - | Time of validity (= t - latency) |
| 2 | `lla` | double | 3 | - | - | [lat deg, lon deg, h m] primary antenna |
| 3 | `vel_ENU` | double | 3 | m/s | - | Antenna velocity ENU |
| 4 | `fixType` | uint8 | 1 | - | - | NMEA GGA quality: 0 NOFIX 1 SINGLE 4 RTK_FIXED 5 RTK_FLOAT |
| 5 | `numSV` | uint8 | 1 | - | - | Satellites used |
| 6 | `hAcc` | single | 1 | m | - | Reported horizontal accuracy (1 sigma) |
| 7 | `vAcc` | single | 1 | m | - | Reported vertical accuracy |
| 8 | `sAcc` | single | 1 | m/s | - | Reported speed accuracy |
| 9 | `headingValid` | boolean | 1 | - | - | Moving-base RELPOS heading valid |
| 10 | `heading_deg` | double | 1 | deg | [0, 360] | Baseline heading, CW from North |
| 11 | `headAcc_deg` | single | 1 | deg | - | Heading accuracy |
| 12 | `corrAge` | single | 1 | s | - | Correction age |
| 13 | `newData` | boolean | 1 | - | - | New epoch in this sample |

<a id="bus_lidarscanraw"></a>
#### Bus_LidarScanRaw

2-D height-band scan in the LiDAR frame (beam k at angle -pi + (k-1) deg).  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_SensorModels/sens_lidar_model FAST2D. **Consumers:** Shaddad_Perception IO_In.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_scan` | double | 1 | s | - | Scan time |
| 2 | `ranges` | single | 360 | m | - | Range per beam, NaN = no return |
| 3 | `angles` | single | 360 | rad | - | Beam angle CCW from LiDAR x |
| 4 | `valid` | boolean | 1 | - | - | Scan valid |

<a id="bus_lidarcloudraw"></a>
#### Bus_LidarCloudRaw

Organised 3-D point cloud, sensor frame, NaN padded.  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_SensorModels/sens_lidar_model FULL3D (UAV Scenario Lidar) \| ROS 2 /ouster/points. **Consumers:** Shaddad_Perception IO_In (sens_lidar_cloud2scan).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_scan` | double | 1 | s | - | Scan time |
| 2 | `xyz` | single | [32 x 1024 x 3] | m | - | Organised cloud [rings x azimuth x xyz] |
| 3 | `nAz` | uint16 | 1 | - | - | Used azimuth columns (360 in SIM, 1024 on HW) |
| 4 | `valid` | boolean | 1 | - | - | Cloud valid |

<a id="bus_senstruthdiag"></a>
#### Bus_SensTruthDiag

Sensor-model truth diagnostics.  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_SensorModels. **Consumers:** logging, tests.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `svf` | double | 1 | 1 | [0, 1] | GNSS sky-view factor |
| 2 | `zoneId` | uint8 | 1 | - | - | 0 open, 1 greenhouse, 2 under panel, 3 between panels |
| 3 | `fixTypeTruth` | uint8 | 1 | - | - | Fix-chain state |
| 4 | `relposFixed` | boolean | 1 | - | - |  |
| 5 | `gnssPosErr` | double | 3 | m | - | Reported minus true antenna position (ENU) |
| 6 | `outlier` | boolean | 1 | - | - |  |
| 7 | `dustLevel` | double | 1 | 1 | [0, 1] |  |
| 8 | `lidarPoseTruth` | double | 3 | - | - | [x_E y_N psi] of the LiDAR |

<a id="bus_spraydemand"></a>
#### Bus_SprayDemand

Vision-sprayer section demand.  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_SensorModels/sens_spray_vision (abstract camera + detector) \| ROS 2 vision node (HW). **Consumers:** Shaddad_GNC/ImplementManager.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_stamp` | double | 1 | s | - |  |
| 2 | `section_mask` | uint16 | 1 | - | - | Bit i = weed/target detected under boom section i (8 used) |
| 3 | `target_density` | single | 1 | 1 | [0, 1] | Mean detected density 0..1 |
| 4 | `valid` | boolean | 1 | - | - |  |

<a id="bus_imumeas"></a>
#### Bus_ImuMeas

Calibrated IMU sample forwarded by the MCU (body FLU, physical specific force).  
**Owner:** sensors. **Ts:** 0.005 s. **Producer:** Companion_IO (decoded CAN2 0x201 MCU_IMU). **Consumers:** Shaddad_NavFusion.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `accel_B` | single | 3 | m/s^2 | - | Specific force, body FLU |
| 2 | `gyro_B` | single | 3 | rad/s | - | Angular rate, body FLU |
| 3 | `mag_B` | single | 3 | uT | - | Magnetic field, body FLU |
| 4 | `temp_degC` | single | 1 | degC | - |  |
| 5 | `t_us` | uint32 | 1 | us | - | MCU timestamp of the sample |
| 6 | `seq` | uint16 | 1 | - | - | Sample counter |
| 7 | `valid` | boolean | 1 | - | - |  |

<a id="bus_odommeas"></a>
#### Bus_OdomMeas

Track odometry from the ESC motor encoders (50 ms window).  
**Owner:** sensors. **Ts:** 0.01 s. **Producer:** Companion_IO (decoded CAN2 0x202 MCU_ODOM; computed in Shaddad_MCU sens_odom_compute). **Consumers:** Shaddad_NavFusion.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `vTrack_LR` | single | 2 | m/s | - | Sprocket pitch-line speed [L R] = dCount*K_enc/0.05 s |
| 2 | `encCnt_LR` | int32 | 2 | - | - | Latest encoder counts [L R] (wrapping) |
| 3 | `wMotor_LR` | single | 2 | rad/s | - | ESC-reported motor speed [L R] |
| 4 | `t_us` | uint32 | 1 | us | - | MCU timestamp |
| 5 | `valid` | boolean | 1 | - | - | Both ESC feedbacks fresh |

<a id="bus_navcmd"></a>
#### Bus_NavCmd

Navigation mode requests.  
**Owner:** sensors. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/MissionManager. **Consumers:** Shaddad_NavFusion/NavModeManager.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `navMode` | uint8 | 1 | - | [0, 3] | 0 AUTO, 1 GNSS, 2 LIDAR_PRIOR_MAP, 3 DR_TEST |
| 2 | `navReset` | boolean | 1 | - | - | Rising edge resets the EKF |
| 3 | `priorMapId` | uint8 | 1 | - | - | Prior map index (0 = P.world.occ) |

<a id="bus_navstate"></a>
#### Bus_NavState

Fused navigation state of base_link (ENU / FLU).  
**Owner:** sensors. **Ts:** 0.01 s. **Producer:** Shaddad_NavFusion/NavOutput. **Consumers:** Shaddad_GNC, Shaddad_Perception (delayed), Viz_Logging, logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t` | double | 1 | s | - |  |
| 2 | `pos_W` | double | 3 | m | - | base_link position ENU |
| 3 | `lla` | double | 3 | - | - | [lat deg, lon deg, h m] |
| 4 | `eul` | double | 3 | rad | - | ZYX [yaw pitch roll]; yaw CCW from East; pitch + = nose down |
| 5 | `q_WB` | double | 4 | 1 | - | Frame-rotation quaternion W->B, scalar first |
| 6 | `vel_B` | double | 3 | m/s | - | [v vy 0] |
| 7 | `vel_W` | double | 3 | m/s | - |  |
| 8 | `yawRate` | double | 1 | rad/s | - | psidot_m - bg |
| 9 | `omega_B` | double | 3 | rad/s | - | Bias-corrected gyro |
| 10 | `accel_B` | double | 3 | m/s^2 | - | Kinematic acceleration (gravity removed) |
| 11 | `slip_LR` | double | 2 | 1 | - | Estimated track slip |
| 12 | `chi` | double | 1 | 1 | - | Estimated effective-gauge factor |
| 13 | `gyroBiasZ` | double | 1 | rad/s | - |  |
| 14 | `accelBiasX` | double | 1 | m/s^2 | - |  |
| 15 | `posStd` | double | 3 | m | - | sqrt(diag P) of position |
| 16 | `yawStd` | double | 1 | rad | - |  |
| 17 | `velStd` | double | 1 | m/s | - |  |
| 18 | `covPose` | double | [3 x 3] | - | - | Covariance of [pE pN psi] |
| 19 | `quality` | uint8 | 1 | - | - | NavQuality: 0 INVALID 1 DR 2 GNSS_SINGLE 3 GNSS_FLOAT 4 RTK_FIXED 5 LIDAR_LOCALIZED |
| 20 | `navValid` | boolean | 1 | - | - |  |
| 21 | `headingValid` | boolean | 1 | - | - |  |
| 22 | `aligned` | boolean | 1 | - | - |  |
| 23 | `lidarLocActive` | boolean | 1 | - | - |  |
| 24 | `gnssFixType` | uint8 | 1 | - | - | Last GNSS fix type |
| 25 | `tSinceAbsUpdate` | double | 1 | s | - | Time since last absolute (GNSS/LiDAR) update |
| 26 | `resetCount` | uint16 | 1 | - | - |  |

<a id="bus_navdiag"></a>
#### Bus_NavDiag

EKF diagnostics.  
**Owner:** sensors. **Ts:** 0.01 s. **Producer:** Shaddad_NavFusion. **Consumers:** logging, tests.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `nis` | double | 5 | - | - | Normalised innovation squared per channel |
| 2 | `enable` | boolean | 5 | - | - | Measurement accepted per channel |
| 3 | `rejCount` | double | 5 | - | - | Consecutive rejections |
| 4 | `gnssLatency` | double | 1 | s | - |  |
| 5 | `ekfTrace` | double | 1 | - | - | trace(P) |
| 6 | `P_diag` | double | 11 | - | - | diag(P) |
| 7 | `x` | double | 11 | - | - | EKF state |
| 8 | `modeState` | uint8 | 1 | - | - | Nav mode manager state |

<a id="bus_scanmatchmeas"></a>
#### Bus_ScanMatchMeas

LiDAR scan-to-prior-map pose of base_link.  
**Owner:** sensors. **Ts:** 0.2 s. **Producer:** Shaddad_Perception/sens_scanmatch_loc. **Consumers:** Shaddad_NavFusion (EKF channel 5).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `valid` | boolean | 1 | - | - |  |
| 2 | `t` | double | 1 | s | - |  |
| 3 | `pose` | double | 3 | - | - | [x_E m, y_N m, psi rad] |
| 4 | `cov` | double | [3 x 3] | - | - | Covariance in W |
| 5 | `score` | double | 1 | - | - | matchScansGrid score |
| 6 | `nPts` | uint16 | 1 | - | - | Valid points |

<a id="bus_tebmapinfo"></a>
#### Bus_TebMapInfo

Rolling 30 x 30 m occupancy grid; FIELD ORDER = TEB default MapInfo struct (verified).  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_Perception/sens_occmap_update. **Consumers:** Shaddad_GNC/LocalAvoidance (TEB MapInfo).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `Resolution` | double | 1 | - | - | Cells per metre (10) |
| 2 | `GridLocationInWorld` | double | [1 x 2] | m | - | World location of the grid lower-left corner |
| 3 | `GridSize` | double | [1 x 2] | - | - | [rows cols] = [300 300] |
| 4 | `OccupancyMatrix` | boolean | [300 x 300] | - | - | true = occupied (p > 0.65); row 1 = max y |
| 5 | `GridOriginInLocal` | double | [1 x 2] | m | - | Grid origin in local frame |

<a id="bus_rowfeature"></a>
#### Bus_RowFeature

Palm / solar-pile row geometry relative to the robot.  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_Perception/sens_row_detect. **Consumers:** Shaddad_GNC (row following aid), logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `valid` | boolean | 1 | - | - |  |
| 2 | `lateralOffset` | single | 1 | m | - | + = robot left of row centre |
| 3 | `headingErr` | single | 1 | rad | - |  |
| 4 | `distLeft` | single | 1 | m | - |  |
| 5 | `distRight` | single | 1 | m | - |  |
| 6 | `nLeft` | uint8 | 1 | - | - | Inliers left |
| 7 | `nRight` | uint8 | 1 | - | - | Inliers right |
| 8 | `fitRms` | single | 1 | m | - |  |

<a id="bus_perception"></a>
#### Bus_Perception

Perception products (base_link frame).  
**Owner:** sensors. **Ts:** 0.1 s. **Producer:** Shaddad_Perception. **Consumers:** Shaddad_GNC (corridor, VFH, TEB, hold logic), Viz_Logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t_scan` | double | 1 | s | - |  |
| 2 | `scanValid` | boolean | 1 | - | - | Scan of this step is valid (read by the protective field and, through Bus_ObsFlags.scan_valid, the GNC hold logic) |
| 3 | `ranges_B` | single | 360 | m | - | Filtered ranges about base_link, NaN = no return (VFH Ranges) |
| 4 | `angles_B` | single | 360 | rad | - | Beam angles CCW from body x (VFH Angles) |
| 5 | `minSectorRange` | single | 8 | m | - | 8 x 45 deg sectors, sector 1 centred forward, CCW |
| 6 | `protectiveStop` | boolean | 1 | - | - | Object inside the speed-dependent protective field (front field from protXF, the fitted implement front, while v > -0.1 m/s; rear field from protXR while v < 0.1 m/s), or more than protNStale invalid scans in a row (true from the start until the first valid scan); used as person_near |
| 7 | `protFieldLen` | single | 1 | m | - | 0.5 + 0.8*\|v\| |
| 8 | `obsCount` | uint8 | 1 | - | [0, 20] | Valid obstacle rows |
| 9 | `obsPos_B` | single | [20 x 2] | m | - | Obstacle centroids, body frame |
| 10 | `obsRadius` | single | 20 | m | - |  |
| 11 | `obsRange` | single | 20 | m | - |  |
| 12 | `row` | Bus: Bus_RowFeature | 1 | - | - |  |
| 13 | `map` | Bus: Bus_TebMapInfo | 1 | - | - |  |
| 14 | `scanMatch` | Bus: Bus_ScanMatchMeas | 1 | - | - | Latest scan-match result (held) |

<a id="bus_gnccmd"></a>
#### Bus_GncCmd

Companion -> MCU command (MCU stays final mode authority).  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/CommandShaping (packed by Companion_IO into CAN2 0x100 COMP_CMD). **Consumers:** Shaddad_MCU (decoded in Task_50).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `seq` | uint8 | 1 | - | - | Rolling heartbeat; MCU declares companion stale after 0.2 s without change |
| 2 | `t_stamp` | double | 1 | s | - | Companion time (not transmitted) |
| 3 | `cmd_valid` | boolean | 1 | - | - | v/omega valid (AUTO/RTH motion only) |
| 4 | `gnc_ready` | boolean | 1 | - | - | GNC healthy: nav_ok, plan valid, no GNC fault (allows MCU AUTO_START) |
| 5 | `mode_req` | uint8 | 1 | - | [0, 4] | GncModeReq: 0 STOP 1 MANUAL_ACK 2 AUTO 3 HOLD 4 FAULT_STOP |
| 6 | `gnc_state` | uint8 | 1 | - | - | GncState (info) |
| 7 | `v_cmd` | single | 1 | m/s | [-0.5, 1.5] | Forward speed command (envelope = MCU AUTO limits, D43) |
| 8 | `omega_cmd` | single | 1 | rad/s | [-1.2, 1.2] | Yaw-rate command, CCW + |
| 9 | `accel_limit` | single | 1 | m/s^2 | [0, 2] | MCU ramp cap |
| 10 | `soft_stop_req` | boolean | 1 | - | - | Controlled deceleration to 0 |
| 11 | `v_ground_est` | single | 1 | m/s | - | Fused ground speed (traction control) |
| 12 | `v_ground_valid` | boolean | 1 | - | - |  |
| 13 | `nav_fix` | uint8 | 1 | - | - | NavQuality |
| 14 | `mower_enable` | boolean | 1 | - | - |  |
| 15 | `mower_speed_cmd` | single | 1 | rad/s | [0, 330] | Blade speed (293.2 = 2800 rpm) |
| 16 | `deck_height_cmd` | single | 1 | m | [0.03, 0.12] | Mower deck height |
| 17 | `tiller_enable` | boolean | 1 | - | - |  |
| 18 | `tiller_speed_cmd` | single | 1 | rad/s | [0, 30] | Rotor speed (26.18 = 250 rpm) |
| 19 | `tiller_depth_cmd` | single | 1 | m | [0, 0.12] | Tiller depth, + down |
| 20 | `spray_enable` | boolean | 1 | - | - |  |
| 21 | `spray_section_mask` | uint16 | 1 | - | - | Bit i = nozzle section i (8 used) |
| 22 | `spray_flow_cmd` | single | 1 | L/min | [0, 10] | Total spray flow |
| 23 | `beacon_cmd` | uint8 | 1 | - | [0, 3] | 0 off 1 auto 2 hold 3 fault |
| 24 | `buzzer_cmd` | boolean | 1 | - | - |  |

<a id="bus_missioncmd"></a>
#### Bus_MissionCmd

Mission command; a new command is detected on a change of cmd_seq.  
**Owner:** gnc. **Ts:** 0.2 s. **Producer:** Shaddad_System root ScenarioIn.mcmd (ground station in HW). **Consumers:** Shaddad_GNC/InputConditioning.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `cmd` | uint8 | 1 | - | [0, 7] | MissionCmd: 0 NONE 1 LOAD 2 START 3 PAUSE 4 RESUME 5 ABORT 6 RTH 7 RESET_FAULT |
| 2 | `cmd_seq` | uint8 | 1 | - | - | Increment to issue cmd |
| 3 | `mission_id` | uint16 | 1 | - | - |  |
| 4 | `end_action` | uint8 | 1 | - | [0, 1] | 0 STOP 1 RTH |
| 5 | `speed_override` | single | 1 | 1 | [0, 1] | Speed scale (0 = use 1) |

<a id="bus_missionplan"></a>
#### Bus_MissionPlan

Compiled mission (fixed size, 512 segments x 100 poses).  
**Owner:** gnc. **Ts:** Inf s. **Producer:** scripts/gnc_compileMission.m -> P.gnc.mission (Constant MissionSource inside Shaddad_GNC, no root port, D57). **Consumers:** Shaddad_GNC.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `valid` | boolean | 1 | - | - |  |
| 2 | `mission_id` | uint16 | 1 | - | - |  |
| 3 | `mission_type` | uint8 | 1 | - | - | MissionType: 1 PALM_ROW_PATROL 2 GREENHOUSE_COVERAGE 3 SOLAR_ROW_MOWING 4 GOTO |
| 4 | `implement_type` | uint8 | 1 | - | - | 0 NONE 1 MOWER 2 TILLER 3 SPRAYER |
| 5 | `n_seg` | uint16 | 1 | - | [0, 512] | Used segments |
| 6 | `dock_seg` | uint16 | 1 | - | - | Index of the dock-approach segment |
| 7 | `path` | single | [51200 x 4] | - | - | Per segment 100 rows: x m, y m (ENU), theta deg, dir +1/-1 |
| 8 | `seg` | single | [512 x 10] | - | - | type, dir, length m, v_seg_max m/s, v_end m/s, impl_mode, allow_bypass, lane_halfwidth m, lane_id, pivot_allowed |
| 9 | `boundary` | double | [64 x 2] | m | - | Geofence vertices ENU (grown by 0.5 m) |
| 10 | `n_bnd` | uint8 | 1 | - | - | Used boundary vertices |
| 11 | `home` | double | 3 | - | - | [x m, y m, psi rad] |
| 12 | `dock` | double | 3 | - | - | [x m, y m, psi rad] |
| 13 | `datum_lla` | double | 3 | - | - | [lat deg, lon deg, h m] (must equal P.world.lla0) |
| 14 | `swath` | single | 1 | m | - |  |
| 15 | `area_m2` | single | 1 | m^2 | - |  |
| 16 | `speed_work` | single | 1 | m/s | - |  |
| 17 | `end_action` | uint8 | 1 | - | - | 0 STOP 1 RTH |

<a id="bus_gncstatus"></a>
#### Bus_GncStatus

GNC telemetry.  
**Owner:** gnc. **Ts:** 0.2 s. **Producer:** Shaddad_GNC/StatusMetrics. **Consumers:** Viz_Logging, Sim_Monitors, logging (HW: ROS 2 topic).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `gnc_state` | uint8 | 1 | - | - | GncState: 0 IDLE 1 MANUAL 2 AUTO_MISSION 3 PAUSE 4 OBSTACLE_HOLD 5 RETURN_TO_HOME 6 FAULT |
| 2 | `sub_mode` | uint8 | 1 | - | - | AutoSub: 0 NONE 1 ALIGN 2 TRANSIT 3 ROW 4 TURN 5 BYPASS 6 REJOIN 7 RTH_PLANNING 8 RTH_TRANSIT 9 DOCK_APPROACH 10 DOCKED |
| 3 | `mission_type` | uint8 | 1 | - | - |  |
| 4 | `mission_id` | uint16 | 1 | - | - |  |
| 5 | `seg_idx` | uint16 | 1 | - | - |  |
| 6 | `n_seg` | uint16 | 1 | - | - |  |
| 7 | `progress` | single | 1 | 1 | [0, 1] |  |
| 8 | `area_done_m2` | single | 1 | m^2 | - |  |
| 9 | `xte` | single | 1 | m | - | Cross-track error (estimated) |
| 10 | `heading_err` | single | 1 | rad | - |  |
| 11 | `xte_rms` | single | 1 | m | - | 10 s moving RMS on ROW segments |
| 12 | `v_ref` | single | 1 | m/s | - |  |
| 13 | `dist_home` | single | 1 | m | - |  |
| 14 | `soc_rth` | single | 1 | 1 | - | Return-to-home SOC threshold |
| 15 | `fault_word` | uint32 | 1 | - | - | b0 NAV_INVALID b1 NAV_DEGRADED b2 MCU_LINK_LOST b3 XTE_EXCEEDED b4 STUCK b5 TILT b6 BMS_CRITICAL b7 PLANNER_FAIL b8 OBST_TIMEOUT b9 GEOFENCE b10 MISSION_INVALID b11 ESTOP b12 IMPLEMENT_TIMEOUT |
| 16 | `hold_reason` | uint8 | 1 | - | - | HoldReason |
| 17 | `pause_reason` | uint8 | 1 | - | - | HoldReason |
| 18 | `hold_timer` | single | 1 | s | - |  |
| 19 | `replan_count` | uint16 | 1 | - | - |  |
| 20 | `tracker_active` | uint8 | 1 | - | - | TrackerSel: 0 STOP 1 PP 2 ROW 3 TEB 4 ALIGN |
| 21 | `nav_ok` | boolean | 1 | - | - |  |
| 22 | `eta_s` | single | 1 | s | - |  |
| 23 | `pose_est` | double | 3 | - | - | [x m, y m, psi rad] used by GNC |
| 24 | `mission_done` | boolean | 1 | - | - |  |

<a id="bus_gncdebug"></a>
#### Bus_GncDebug

GNC internals.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC. **Consumers:** logging.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `pose_ref` | double | 3 | - | - | [x y psi] of the closest path point |
| 2 | `kappa_ref` | single | 1 | 1/m | - |  |
| 3 | `v_prof` | single | 1 | m/s | - | Velocity-profile speed |
| 4 | `v_gov` | single | 1 | m/s | - | Governor speed |
| 5 | `v_limits` | single | 12 | m/s | - | MinMax inputs: prof seg obs xte head pitch roll nav soc mode override veg |
| 6 | `omega_pp` | single | 1 | rad/s | - |  |
| 7 | `omega_row` | single | 1 | rad/s | - |  |
| 8 | `omega_teb` | single | 1 | rad/s | - |  |
| 9 | `d_obs` | single | 1 | m | - | Corridor obstacle distance (Inf = clear) |
| 10 | `vfh_steer` | single | 1 | rad | - | VFH steering direction (NaN = blocked) |
| 11 | `mpc_qp_status` | int32 | 1 | - | - |  |
| 12 | `window_ver` | uint16 | 1 | - | - |  |

<a id="bus_pathwindow"></a>
#### Bus_PathWindow

Smoothed and profiled current (A) and next (B) segment.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/PathServer/SegmentPrep. **Consumers:** Shaddad_GNC/PathAnalyzer, PathTracker.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `PA` | double | [400 x 3] | - | - | Current segment poses [x m, y m, theta deg] |
| 2 | `sA` | double | 400 | m | - | Cumulative length |
| 3 | `kA` | double | 400 | 1/m | - | Curvature |
| 4 | `vA` | double | 400 | m/s | - | Profiled speed |
| 5 | `PB` | double | [400 x 3] | - | - | Next segment poses |
| 6 | `sB` | double | 400 | m | - |  |
| 7 | `kB` | double | 400 | 1/m | - |  |
| 8 | `ver` | uint16 | 1 | - | - | Window version (+1 per SegmentPrep run) |
| 9 | `segA` | uint16 | 1 | - | - | Index of segment A |
| 10 | `segB` | uint16 | 1 | - | - | Index of segment B |
| 11 | `typeA` | uint8 | 1 | - | - | SegType of A |
| 12 | `typeB` | uint8 | 1 | - | - | SegType of B |
| 13 | `valid` | boolean | 1 | - | - |  |

<a id="bus_patherror"></a>
#### Bus_PathError

Tracking errors w.r.t. the current window.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/PathAnalyzer (gnc_pathAnalyzer). **Consumers:** Shaddad_GNC MissionManager (delayed), InputConditioning (delayed), PathTracker, SpeedGovernor, LocalAvoidance.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `xte` | double | 1 | m | - | + = robot left of path |
| 2 | `epsi` | double | 1 | rad | - | wrap(psi - psi_ref) |
| 3 | `psi_ref` | double | 1 | rad | - |  |
| 4 | `kappa_ref` | double | 1 | 1/m | - |  |
| 5 | `v_ref` | double | 1 | m/s | - |  |
| 6 | `s` | double | 1 | m | - | Arc length along segment A |
| 7 | `s_rem` | double | 1 | m | - | Remaining length of A |
| 8 | `idx` | uint16 | 1 | - | - | Closest pose index in A |
| 9 | `kappa_prev` | double | 40 | 1/m | - | Curvature preview over the MPC horizon |
| 10 | `xy_ref` | double | 2 | m | - | Closest point |
| 11 | `valid` | boolean | 1 | - | - | Closest distance < 2 m |

<a id="bus_fsmout"></a>
#### Bus_FsmOut

Mission-manager outputs.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/MissionManager (gnc_missionFSM). **Consumers:** Shaddad_GNC subsystems.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `state` | uint8 | 1 | - | - | GncState |
| 2 | `sub` | uint8 | 1 | - | - | AutoSub |
| 3 | `seg_idx` | uint16 | 1 | - | - |  |
| 4 | `nxt_idx` | uint16 | 1 | - | - |  |
| 5 | `path_src` | uint8 | 1 | - | - | PathSrc: 0 MISSION 1 REPLAN |
| 6 | `trk_sel` | uint8 | 1 | - | - | TrackerSel |
| 7 | `mode_req` | uint8 | 1 | - | - | GncModeReq |
| 8 | `cmd_valid` | boolean | 1 | - | - |  |
| 9 | `gnc_ready` | boolean | 1 | - | - |  |
| 10 | `impl_allow` | boolean | 1 | - | - | Implement allowed (ROW / TURN); false for the rest of an OBSTACLE_HOLD once a person was seen or the scan was lost |
| 11 | `replan_req` | boolean | 1 | - | - | Toggles to request a replan |
| 12 | `replan_goal` | double | 3 | - | - | [x y psi] |
| 13 | `replan_kind` | uint8 | 1 | - | - | 0 LOCAL 1 GLOBAL |
| 14 | `hold_reason` | uint8 | 1 | - | - | HoldReason |
| 15 | `pause_reason` | uint8 | 1 | - | - | HoldReason |
| 16 | `mission_done` | boolean | 1 | - | - |  |
| 17 | `teb_enable` | boolean | 1 | - | - |  |
| 18 | `v_cap` | double | 1 | m/s | - | Mode speed cap |
| 19 | `s_obs` | double | 1 | m | - | Arc length where the obstacle hold started |
| 20 | `t_hold` | double | 1 | s | - |  |
| 21 | `fault_word` | uint32 | 1 | - | - | As Bus_GncStatus.fault_word |
| 22 | `trans_id` | uint8 | 1 | - | - | Transition fired in this step (gnc_missionFSM table: G1 1, G2 2, T1..T10 11..20, S1..S5 21..25, H1..H3 31..33, impl. timeout 35, RC loss 36, P1..P4 41..44, R1..R6 51..56, F1 61; 0 none) |

<a id="bus_replanresult"></a>
#### Bus_ReplanResult

Hybrid A* replanning result (held).  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/GlobalReplanner (gnc_globalReplanner). **Consumers:** Shaddad_GNC MissionManager (delayed), PathServer.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `ok` | boolean | 1 | - | - |  |
| 2 | `busy` | boolean | 1 | - | - |  |
| 3 | `ver` | uint16 | 1 | - | - | +1 per successful plan |
| 4 | `n` | uint16 | 1 | - | [0, 100] | Used poses |
| 5 | `path` | double | [100 x 4] | - | - | [x m, y m, theta deg, dir] |
| 6 | `fail_count` | uint8 | 1 | - | - |  |
| 7 | `t_plan` | double | 1 | s | - |  |

<a id="bus_mpcplant"></a>
#### Bus_MpcPlant

Discrete LTV plant.  
**Owner:** gnc. **Ts:** inherited s. **Producer:** gnc_mpcPlantUpdate. **Consumers:** Adaptive MPC Controller (model port).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `A` | double | [5 x 5] | - | - | x = [e_y e_psi r u(k-1) u(k-2)] (2-sample MV delay, D41) |
| 2 | `B` | double | [5 x 5] | - | - | u = [r_cmd (MV) kappa_ref (MD) d w_y w_psi (UD)] |
| 3 | `C` | double | [2 x 5] | - | - | y = [e_y e_psi] |
| 4 | `D` | double | [2 x 5] | - | - |  |

<a id="bus_mpcnominal"></a>
#### Bus_MpcNominal

Nominal operating point.  
**Owner:** gnc. **Ts:** inherited s. **Producer:** gnc_mpcPlantUpdate. **Consumers:** Adaptive MPC Controller (model port).

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `X` | double | [5 x 1] | - | - | column matrix (Adaptive MPC model inport requirement) |
| 2 | `U` | double | [5 x 1] | - | - |  |
| 3 | `Y` | double | [2 x 1] | - | - |  |
| 4 | `DX` | double | [5 x 1] | - | - |  |

<a id="bus_mpcmodel"></a>
#### Bus_MpcModel

Adaptive MPC model input.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/PathTracker/RowTracker (gnc_mpcPlantUpdate). **Consumers:** Adaptive MPC Controller.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `Plant` | Bus: Bus_MpcPlant | 1 | - | - |  |
| 2 | `Nominal` | Bus: Bus_MpcNominal | 1 | - | - |  |

<a id="bus_navcond"></a>
#### Bus_NavCond

Conditioned navigation state.  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/InputConditioning. **Consumers:** Shaddad_GNC subsystems.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `t` | double | 1 | s | - |  |
| 2 | `pose` | double | 3 | - | - | [x m, y m, psi rad] |
| 3 | `v` | double | 1 | m/s | - |  |
| 4 | `yaw_rate` | double | 1 | rad/s | - |  |
| 5 | `pitch` | double | 1 | rad | - | + nose down |
| 6 | `roll` | double | 1 | rad | - |  |
| 7 | `sigma_pos` | double | 1 | m | - | max(posStd(1:2)) |
| 8 | `nav_ok` | boolean | 1 | - | - |  |
| 9 | `nav_degraded` | boolean | 1 | - | - |  |
| 10 | `gnss_fix` | uint8 | 1 | - | - | NavQuality of nav_state (0 INVALID 1 DR 2 GNSS_SINGLE 3 GNSS_FLOAT 4 RTK_FIXED 5 LIDAR_LOCALIZED) |

<a id="bus_tebmapcrop"></a>
#### Bus_TebMapCrop

TEB map window: P.gnc.teb.map_window (10 m) crop of the robot-centred perception grid, 2 x 2 max-pooled to 5 cells/m (D56). Field order = TEB MapInfo.  
**Owner:** gnc. **Ts:** 0.2 s. **Producer:** Shaddad_GNC/LocalAvoidance/Bypass (crop of Bus_Perception.map). **Consumers:** Timed Elastic Band (TEB) MapInfo.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `Resolution` | double | 1 | - | - | Cells per metre (5) |
| 2 | `GridLocationInWorld` | double | [1 x 2] | m | - | World location of the crop lower-left corner |
| 3 | `GridSize` | double | [1 x 2] | - | - | [rows cols] |
| 4 | `OccupancyMatrix` | boolean | [50 x 50] | - | - | true = occupied; row 1 = max y |
| 5 | `GridOriginInLocal` | double | [1 x 2] | m | - |  |

<a id="bus_obsflags"></a>
#### Bus_ObsFlags

Obstacle flags (rate-transitioned to 0.05 s).  
**Owner:** gnc. **Ts:** 0.05 s. **Producer:** Shaddad_GNC/LocalAvoidance. **Consumers:** Shaddad_GNC MissionManager, SpeedGovernor.

| # | Element | Type | Dims | Unit | Range | Description |
|---|---|---|---|---|---|---|
| 1 | `d_obs` | double | 1 | m | - | Corridor distance, Inf = clear |
| 2 | `blocked_vfh` | boolean | 1 | - | - | VFH SteerDir is NaN |
| 3 | `person_near` | boolean | 1 | - | - | = Bus_Perception.protectiveStop |
| 4 | `vfh_steer` | double | 1 | rad | - |  |
| 5 | `scan_valid` | boolean | 1 | - | - | = Bus_Perception.scanValid (false until the first scan); lost scan -> OBSTACLE_HOLD SENSOR_LOST |
