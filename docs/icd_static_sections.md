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

