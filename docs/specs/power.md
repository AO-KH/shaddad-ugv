# Shaddad UGV - POWER domain specification (KEY = power)

> **Revision C (2026-11-18, card G1-01 part 2).** This revision brings the spec in line with the robot as built
> (2026-10-27: the XMachines 13S15P NMC pack, its BMS and its charger) and with `params/param_power.m`, which is the
> source of every value here. `docs/architecture.md` section 2b overrides this spec where they differ. Written in:
> D86 (the E-stop stops traction as category 0, section 2.5), D94 (cool charging of the sealed pack, 2.3 and 2.9),
> D96 (the delivered BMS, charger and ESCs are the baseline, provisionally), D97 (no dock on the robot in this phase,
> 2.8), D98 (the precharge limits, 2.4 and 4.1) and D101 (the energy per mowing mission, 2.1).
>
> **Decided, not yet in the twin.** Where a decision changes a parameter, a model or a test, the text gives the
> decided value with its row and "(the twin changes on card G2-PWR)", and the value that the parameters hold today
> stays in the text, so that every run until the change can still be read.
>
> **Fix pass (2026-11-18, card G1-01 part 2, step 19).** The independent re-check's ten findings are written in, with the register's values as merged: REQ-SYS-063 at most 50 ms, REQ-SYS-048 at rest within 2.0 s, D105 for storage. The part-2 values that waited for the owner's approval (REQ-SYS-054's 3 s, REQ-SYS-208's top-up band) were no longer called decided; the project owner approved them with the baseline on 2026-11-18 (D110), so they are decided, pending on card G2-PWR (4.1).
>
> **History.** Revision A (2026-10-18) designed the domain around a former 16S3P LFP pack of 15.36 kWh. Revision B
> (2026-10-19) applied D30 to D63. The real pack of 2026-10-27 changed the parameters but not this text; revision C
> replaces every value of the former pack. The revision-B text is kept in
> `work/team/snap/20261117_g1-01_p2_step7/docs/specs/power.md`.

Battery system, power distribution, LV rails, implement electrical loads, charger and dock contacts, E-stop chain
and BMS.

| Item | Value |
|---|---|
| Spec owner | the BMS and power engineer |
| Status | Revision C (2026-11-18), the as-built baseline of card G1-01 |
| Plant model type | Subsystem Reference (Simscape physical), `subsystems/power/Power_Plant.slx` |
| Controller model type | Model Reference (code-generatable BMS firmware), `models/Shaddad_BMS.slx`: the twin's STM32G4 BMS, a variant of the delivered BMS (D96) |
| Pack (as built) | XMachines 13S15P, 195 Molicel P42A 21700 NMC cells [DATA]: 46.8 V nominal (`P.power.pack.V_nom`), 39.0 to 54.6 V operating (`V_oper_min` / `V_oper_max`), 63 Ah (`Q_Ah`), 2,948 Wh (`E_Wh`), 18.5 kg [EST] (`mass_kg`), sealed 3 mm aluminium box 393 x 358.5 x 87 mm [DATA] |
| BMS (as built) | delivered 13S Li-ion NMC BMS [DATA]: 120 to 150 A continuous, 200 to 300 A peak, CAN 500 kbit/s. The twin uses the top of the continuous range, 150 A (`P.power.bms.I_dis_cont`), and a peak of 250 A (`P.power.bms.I_dis_peak`), provisionally (D96); the 120 A lower bound goes to G1-04. Thresholds, frames and behaviour not yet known (G1-04, R-15) |
| Charger (as built) | delivered stand-alone CC-CV charger [DATA]: 54.6 V (`P.power.charger.cccv_V`), 20 A (`I_max`), 100 to 240 VAC, PFC, IP54. Plugged in by hand; no dock in this phase (D97) |
| LV rails (twin) | 24 V / 600 W, 12 V / 300 W, 5 V / 25 W (from 12 V), 3.3 V LDO (from 5 V) (`P.power.lv`) |
| Mission energy | D101: each greenhouse and palm-grove mowing mission of phase 2 on one charge, from SOC 0.95, ending with at least the 0.20 reserve plus 10 % of the usable window (SOC 0.275) at end of life (SOH 0.80). That window holds 1,592 Wh at end of life, and gives the loads 1,544 Wh (section 2.1) |

All block paths in this document were confirmed in the live R2026b session with `load_system` + `find_system`/`get_param`, unless they are marked **NOT VERIFIED**. Mask parameter names are the real names read with `get_param(blk,'DialogParameters')` or `get_param(blk,'MaskNames')`.

Several library block names contain a **newline** character. When calling `add_block`, build the path with `sprintf`, for example `sprintf('batt_lib/Cells/Battery Equivalent\nCircuit')`, `sprintf('nesl_utility/Solver\nConfiguration')`, `sprintf('nesl_utility/Simulink-PS\nConverter')`, `sprintf('nesl_utility/PS-Simulink\nConverter')`, `sprintf('batt_lib/Thermal/Array of Thermal\nNodes Connector')`. `get_param` also accepts the space form, but `add_block` should use the exact name.

---

## 1. Purpose and scope

### 1.1 In scope (POWER owns these)
1. **Battery pack**: a 13S15P NMC equivalent-circuit model with thermal behaviour, generated with the Simscape Battery *Battery Builder* (`batteryCell` -> `batteryParallelAssembly` -> `batteryModule` -> `buildBattery`, `scripts/build/build_shaddad_battery.m`). Each cell model uses `batt_lib/Cells/Battery Equivalent Circuit`. Passive balancing, the ambient thermal path and cell-to-cell variation are included.
2. **Pack enclosure thermal model**: ambient temperature, solar load, convection and radiation. The real box is sealed; the fan path and the dock air are kept as the vented-pack variant (D94; section 2.3).
3. **Power distribution unit (PDU)**: main fuse, current shunt, key switch, precharge relay and resistor, main contactor, charge contactor, branch fuses for left traction, right traction, implement, aux and charge, DC-bus capacitance and bleed, and cable resistances.
4. **E-stop / safety chain**: a hardwired safety relay with reset. Its immediate contacts drive STO, the implement ECU's stop input and the 24 V implement relay; its delayed contacts drive the main and precharge contactor coils (section 2.5; traction stops as category 0, D86).
5. **LV distribution**: averaged DC-DC converters 48->24 V, 48->12 V and 12->5 V, a 3.3 V LDO, the 24 V implement relay, rail loads driven by the ELECTRONICS power demand, fan and coil loads, and rail sensing.
6. **Implement electrical loads**: a Variant Subsystem with None / Mower (48 V, 1.8 kW drive) / Tiller (48 V, 12:1) / Sprayer (24 V pump + nozzle valves). Each includes its implement ECU (a COTS drive controller with its speed loop). The load mechanics are STRUCTURE's (D9, `P.structure.impl`).
7. **Charger and dock contacts**: the twin's dock with its interlocked charger (F-10 is verified in the twin, D97); the robot's own charger is the stand-alone unit above.
8. **BMS firmware model** (the twin's G4 BMS, D96): protection, fault qualification, SOC/SOE/SOH and power estimation, charge/discharge current limits, CC-CV charge control, passive balancing, fan control, the contactor/precharge/charge mode manager and telemetry.
9. **BMS sensor models** for the AFE cell voltages, shunt current, NTCs and pack/bus/dock voltages: noise, offset, quantization and ZOH.

### 1.2 Out of scope (owned by neighbors)
The interfaces as built are in `docs/icd.md` (sections 3.2 and 4) and `buses/buses_power.m`; this table names the owners.

| Neighbor | What they own | Interface with POWER |
|---|---|---|
| **DRIVE** | The four motors and two dual-channel ESCs (incl. DC-link capacitors), gearboxes, sprockets, spring brakes | Simscape electrical ports `DC48_TracL_p/n`, `DC48_TracR_p/n`. Hardwired `Safety_OK` to STO |
| **ELECTRONICS / MCU** (STM32) | MCU firmware, companion computer, LiDARs, GNSS, IMU, RC receiver loads, MCU ADC/GPIO/FDCAN, the CAN link to the BMS | `Bus_PowerHwCmd` (MCU DO/PWM -> POWER), `Bus_LV_LoadDemand` (ELEC -> POWER), BMS_REQ (`Bus_BmsReq`, MCU -> BMS), the BMS frames (`Bus_BMS_Telemetry` -> `Bus_BmsStatus`), `Bus_PowerStatus`, IMPL_CMD / IMPL_FB (implement ECU) |
| **STRUCTURE** | Chassis and track dynamics, mass properties, the environment (ambient, sun), the dock geometry, the implement load mechanics | `Bus_EnvState` (ambient T, solar irradiance, dock contact); `Impl_Shaft` (implement torque and speed). POWER provides the pack mass, size and position parameters |
| **RC** | SBUS decoding, failsafe | Indirect only. The RC E-stop and failsafe reach the chain through `MCU_Safety_OK`; a certified wireless E-stop would enter as `WirelessEStop_OK` in Loop_OK |
| **GNC** | Mission and return-to-dock logic | Uses the BMS's SOC, runtime and power limits through the MCU |
| **SCENARIO / top level** | Operator actions: E-stop buttons, reset, key switch | `Bus_HmiInputs` |

---

## 2. Physical and algorithmic model description

### 2.1 Energy budget and pack sizing

**The pack as built** (the as-built record, `docs/hardware/real_robot_components.md`; `params/param_power.m:18-67`):
- **Cells:** Molicel P42A, cylindrical 21700 NMC: 4.2 Ah and 3.6 V nominal [DATA], 70 g [EST] (the P42A data sheet's value). Cell tables [EST] from typical P42A data (section 6.2).
- **13S15P:** 195 cells, 46.8 V nominal, 54.6 V at CV (4.20 V per cell), 39.0 V operating minimum (3.0 V per cell), and the absolute maximum `P.power.pack.V_abs_max` 55.25 V (4.25 V per cell). The BMS cuts off at `P.power.bms.V_dvl` 36.4 V (2.80 V per cell).
- **The 60 V line (REQ-SYS-204).**
  - **K1 closed:** the bus stays below 60 V DC, because the pack is at most 55.25 V and the charger's OVP acts at 55.9 V. So the bus stays in the 48 V class (to be confirmed against the standards, D103).
  - **K1 open:** the implement's regenerative braking can lift the isolated bus to the 60 V DC-link clamp (section 2.7); `dead_bus_coastdown` recorded 60.7 V (2026-11-08). The ESCs derate their regen up to 60 V (`P.drive.esc.V_ov_derate`) and trip at 62 V (`P.drive.esc.V_ov_fault`).
  - **The fix** is for card G2-PWR: a clamp that holds below 60 V at its rated current and stays above the charger's 55.9 V, or a claim limited to the states shown.
- **Capacity and energy:** 63 Ah, E_nom = 2,948 Wh (`P.power.pack.E_Wh`).
- **Mass:** 195 x 70 g = 13.65 kg of cells; with the 3.3 kg box and 1.5 kg of BMS, busbars and cables, `P.power.pack.mass_kg` 18.5 kg [EST] (a mass factor of 1.355).

**The energy requirement (D101, binding).** No fixed shift. Each greenhouse and palm-grove mowing mission of phase 2 finishes on one charge with at least 10 % of the usable energy above the 0.20 reserve, at the pack's end of life (SOH 0.80, `P.power.budget.soh_eol`), from SOC 0.95 (`P.power.pack.SOC0`). The mission therefore ends at SOC 0.275 or above:

```
usable window       = 0.95 - 0.20                     = 0.75 of the capacity
margin (D101)       = 0.10 x 0.75                     = 0.075
lowest end SOC      = 0.20 + 0.075                    = 0.275
window energy, EOL  = E_nom x SOH_eol x (0.95 - 0.275) = 2,948 x 0.80 x 0.675 = 1,592 Wh stored in the window
                      (at the nominal 3.6 V per cell; 1,990 Wh at BOL)
to the loads, EOL   = 1,592 Wh x eta_dis 0.97 (P.power.budget.eta_dis, the budget's convention) = 1,544 Wh
```

D101 notes that 3 h of palm mowing would take about 3.7 to 4.3 kWh, so with 1,544 Wh to the loads a mission at end of life lasts about 1.1 to 1.25 h of palm mowing. The energy per hour of each phase-2 mowing mission comes from a phase-2 mowing energy run after D94 and D95, as an owned TBD to G1-05 (the BMS and power engineer with the GNC engineer; R-19, R-20). The sprayer and the tiller get their budgets on their own cards (G2-SPR, G2-TIL).

**Today's budget parameter is superseded.** `P.power.budget` still holds a 3 h sprayer shift, labelled D46 in `param_power.m`; D46's own row set a 4 h mowing shift. The parameter holds working 520 W, transit 380 W, 70 % duty, 3 shifts a day, 1.43 kWh a shift, end SOC 0.32 at end of life (`params/param_power.m:256-263`). D101 supersedes D45 and D46; the budget and the shift-cycle bench T-PWR-20 change on card G2-PWR.

**Peak power and the BMS limits.** The delivered BMS allows 120 to 150 A continuous and 200 to 300 A peak [DATA]. The twin uses 150 A and 250 A (`P.power.bms.I_dis_cont`, `I_dis_peak`), the top of the continuous range: provisional under D96, with the 120 A lower bound for G1-04. The discharge current limit tapers above 45 deg C (section 2.9, item 6), and the MCU's torque limiter keeps the motoring torque within DCL and the regen within CCL (D2, D65). A 1.5 x overload of a 125 A traction fuse (187.5 A) exceeds the BMS's continuous limit, so the BMS trips before the fuse (T-PWR-06b, section 8).

**The pack in storage (D105, REQ-SYS-163).** With its key switch off, the robot keeps every cell at or above 3.0 V for at least 90 days from a storage charge of 40 to 60 %, in the shade (D94). Until the bought BMS's sleep draw is shown to meet this (G1-04), storage beyond 30 days needs the main connector (SB120) unplugged and a monthly check. The twin's BMS supply (`P.power.pdu.P_bms_supply` 2 W), drawn upstream of K1, would empty a pack stored at half charge in about 31 days.

### 2.2 Cell and pack electrical model (ECM)
Each of the 13 series groups (15 cells in parallel, lumped into one model by `ModelResolution = "Grouped"`, `SeriesGrouping = ones(1,13)`, `ParallelGrouping = ones(1,13)`, so `NumModels = 13`) is a 1-RC Thevenin model with a lumped thermal mass:

```
dSOC/dt  = -i_cell / (3600 * Q_Ah)                       (i > 0 = discharge in the Shaddad buses; the batt_sl_lib blocks take + = charge)
dv1/dt   = -v1/tau1(SOC,T) + i_cell / C1,  C1 = tau1/R1
v_term   = OCV(SOC,T) - i_cell*R0(SOC,T) - v1
m*cp*dT/dt = i_cell^2*R0 + v1^2/R1 - (T - T_node)/R_amb_cell
```

Tables are cell-level; the builder scales them for 15P. The BMS uses group-level tables (`P.power.group`: R/15, Q x 15 = 63 Ah).

- OCV(SOC, T) = OCV25(SOC) + kT(SOC) * (T - 298.15)
- R0(SOC, T) = R0_25(SOC) * fR0(T)
- R1(SOC, T) = R1_25(SOC) * fR1(T)
- tau1 = tau1_25 (30 s) * ftau(T)

All values are in section 6.2. Pack-level check at 50 % SOC and 25 deg C (a hand check):
- Cell-group R0 path: 13 x 14 mOhm / 15 = 12.1 mOhm. Adding R1 (13 x 4 mOhm / 15 = 3.5 mOhm) gives 15.6 mOhm.
- Adding the 12 inter-group links (12 x 0.10 mOhm), the main fuse 0.3, the shunt 0.1, K1 0.2 and the pack cable 0.34 mOhm gives about 14.3 mOhm from the pack to the bus (R0 only).
- Bolted short-circuit current at the bus: about 48.1 V / 14.3 mOhm = 3.4 kA.

Cell-to-cell variation uses `CellParameterVariation = "PercentDeviation"` (it requires a non-Lumped resolution). Values: capacity sigma 1 %, R0 sigma 3 %, initial SOC spread +/-1 % (uniform), seeded with `RandStream('mt19937ar','Seed',42)` inside `param_power.m`, so the global `rng` is not touched.

**Fast variant `Pack_Lumped`**: the same cell with `ModelResolution = "Lumped"` (1 model). Cell vectors are synthesized as `V_cell = V_module/13 * ones(13,1)` and `T_cell = T * ones(13,1)`.

### 2.3 Pack enclosure thermal network

**The pack as built is sealed** (D94): a 3 mm aluminium box of 393 x 358.5 x 87 mm and 3.3 kg [DATA], with no fan and no vent. There is no cooling hardware in this phase. The pack is parked, stored and charged in the shade, with the supplier's 55 deg C label as its limit. It is charged only while it is cool: the charge starts at 40 deg C or less on the hottest cell and stops above 45 deg C, which on hot days means overnight (D94). On the robot this is a charging procedure that the project owner approves, until the bought BMS reports its cell temperatures; the procedure earns no PL credit, so the BMS cut-off stays the safety function (SF-17).

**The twin today** still runs the vented pack: the fan path is active and, while docked, the dock supplies conditioned air at `P.power.thermal.dock_air.T_air` 303.15 K (30 deg C); the dock air is on (`dock_air.enable` true, D45). D94 makes the sealed pack with the fixed 20 A charger the default and keeps the vented pack with dock air as a variant (the twin changes on card G2-PWR).

Thermal nodes (`P.power.thermal`, `P.power.pack`):
- 13 cell-group masses (inside the generated module; cell `Cth` = 70 J/K, 195 cells)
- `AIR` node: internal structure and air, `m_int` 2 kg x `cp_int` 900 J/(kg K)
- `WALL` node: the aluminium box, `m_wall` 3.3 kg x `cp_wall` 900 J/(kg K)
- `AMB`: controlled temperature source

Equations:
```
cells -> AIR : R_amb_cell = 20 K/W per cell (module AmbientThermalPath = "CellBasedThermalResistance")
AIR  <-> WALL: G_aw  = h_in * A_int = 4 * 0.41 = 1.64 W/K                      (Convective Heat Transfer, constant)
AIR  <-> AMB : G_fan = h_fan * k_filter * Fan_Cmd * A_int = 20 * 1.0 * 0.41 = 8.2 W/K at full fan
               (vented variant only; thermal_type = variable; dock air replaces AMB while docked when enabled)
WALL <-> AMB : G_ext = h_ext * A_ext = 10 * 0.41 = 4.1 W/K                      (Convective)
             + radiation  Q = eps*sigma*A_ext*(T_w^4 - T_a^4), eps = 0.85 (rad_tr_coeff = 4.82e-8)
Solar into WALL: Q_sol = alpha_sol * A_sol * G_solar = 0.6 * 0.14 * G            (84 W at 1000 W/m^2)
```

Hand check, sealed box at 45 deg C in full sun (1000 W/m^2): 84 W into about 6.9 W/K (4.1 W/K convection plus about 2.8 W/K linearized radiation) settles the wall about 12 K above ambient, near 57 deg C, above the 45 deg C charge stop and near the supplier's 55 deg C label. Hence the shade of D94. The sealed run of 2026-11-15 charged from SOC 0.10 to 0.254 in 5 h at 45 deg C in full sun (D94).

At the 50 deg C hot case of REQ-SYS-164 the same sun gives about 62 deg C, above the BMS's 60 deg C over-temperature cut. The cells follow the wall slowly: 13.65 kJ/K of cells through about 1.4 W/K (the 1.64 W/K air-wall path in series with the cells' 9.75 W/K to the air) is a time constant of about 2.5 h. So a mission of 1.1 to 1.25 h that starts with a cool pack ends well below these steady values, but a long exposure does not. REQ-SYS-162's 55 deg C and REQ-SYS-164's hot case cannot hold over a long time in full sun without a sun shield or a rule to work and park in the shade. This is a known risk, for cards G2-PWR and G2-INT. The lead systems engineer's ruling (step 19, point 5): the values stand, with the operating rule as the interim measure. The project owner adopted the rule on 2026-11-18 (D112), beside D94's charging procedure: in full sun, one mission from a cool start, then park in the shade, until S3 on the sealed pack checks the shortfall on card G2-INT; a sun shield is raised only if that run confirms it.

### 2.4 Power distribution topology (single-line)

```
 PACK_P o--[R_cab_pack 0.34mΩ]--[F_MAIN 150A]--o BATT_P
                                                 |---[K2 precharge]--[R_PRE 22Ω]---+
                                                 |---[K1 main]---------------------+--o DCBUS_P --+--[F_TL 125A]--[R_cab 0.98mΩ]--o DC48_TracL_p  (to DRIVE L)
                                                 |                                  |            +--[F_TR 125A]--[R_cab 0.98mΩ]--o DC48_TracR_p  (to DRIVE R)
                                                 |                                  |            +--[F_IMP 40A]--o IMP48_P --> Implement variant (Mower/Tiller)
                                                 |                                  |            +--[C_PDU 100uF, ESR 5mΩ, bleed 5kΩ]--GND
                                                 |---[S_KEY]--[F_AUX 30A]--o AUX48 --+--> DCDC_24 (600W) --o V24 --+-- LOAD_24AUX (Dynamic Load)
                                                 |                                    |                          +--[K3]--[F_24IMP 20A]--o V24_IMP --> sprayer / deck lift
                                                 |                                    +--> DCDC_12 (300W) --o V12 --+-- LOAD_12 (Dynamic Load)
                                                 |                                                                 +--> DCDC_5 (25W) --o V5 --+-- LOAD_5
                                                 |                                                                                             +-- LDO 3V3 (current sink)
                                                 |---[K4 charge]--[F_CHG 30A]--o DOCK_P --[R_dock 15mΩ]--[S_DOCK]--|>|--+ Charger (controlled I-source) || 10kΩ
                                                 |---BMS_SUPPLY (Constant Power Load 2 W)
 PACK_N o--[R_SHUNT 100µΩ]--(I_pack sensor)--o GND  == Electrical Reference + Solver Configuration (ONLY instance in the system network)
          all returns: DC48_TracL_n, DC48_TracR_n, implement -, LV -, charger -  ---> GND
```

The fuse ratings fit the 150 A BMS and the SB120 main connector, and each side's dual-channel ESC (2 x 55 A DC motoring with the 1.0 kW units) [EST] (`P.power.pdu`). F_IMP 40 A fits the mower's 1.8 kW drive; the tiller's 3 kW envelope would need a larger fuse (card G2-TIL).

**Precharge, today** (`P.power.pdu.R_pre` 22 Ohm). The assumed total bus capacitance is `P.power.pdu.C_bus_total_assumed` 5.5 mF: the two ESC DC links, the implement drive's link and the PDU's 0.1 mF. Under D96 the ESCs' DC-link data are still to come (G1-04).
- `V_bus(t) = V_pack (1 - e^(-t/τ))` with `τ = R_pre C_bus = 22 x 5.5e-3 = 0.121 s`: 95 % at 0.36 s, 97 % at 0.42 s
- Peak current 54.6 V / 22 Ohm = 2.5 A, peak resistor power 136 W, energy about 8.2 J per precharge (½ C V²)
- When K1 closes at 97 %, the step is about 1.6 V. With about 18 to 20 mOhm of loop resistance (the pack, the PDU and each side's ESC link ESR) the spike is about 80 to 90 A with τ of about 0.1 ms, so I^2t = C ΔV² / (2R) is about 0.4 A^2s
- Shorted bus (T-PWR-03b): about 128 W for the 1.5 s time-out, about 190 J per attempt and about 580 J over the three attempts
- With the ESC's standby load the bus settles near 87 % of V_pack at 22 Ohm and never reaches 97 % (R-09, STATUS 10); the scenarios hide it with the override `P.sim.ovr.R_pre` = 4.7 Ohm, which puts about 850 J into the resistor on a shorted bus.

**Precharge, decided (D98; the twin changes on card G2-PWR).** The requirements state limits that fit any hardware: the precharge current within the precharge element's ratings (10 A for the twin's K2), at most 200 J into the precharge resistor over a whole retry sequence on a shorted bus, at most 2 A^2s when the main contactor closes, and HV ready within 1.0 s of the request (REQ-SYS-055, -103). The twin's design becomes about 7 to 8 Ohm, sized for the worst standby load on file with the mower fitted, with a settled-bus test before K1 closes, an early short check at about 0.2 s and a longer time-out for an unknown capacitance; the 4.7 Ohm override goes. Hand check at 7.5 Ohm: τ = 41 ms, peak 7.3 A, 398 W into a shorted bus, about 80 J per 0.2 s short check, so two attempts fit within 200 J (three need a check of about 0.17 s). A 2 A^2s close allows a step of up to about 3.6 V (6.7 % of the pack voltage). The values stay provisional until the ESC data arrive (G1-04); under D96 the robot's own precharge is the bought BMS's or the ESC's.

**Bus bleed.** 5 kOhm (`g_bleed` 2e-4 S) gives τ = 27.5 s with 5.5 mF. It is used for weld detection: a healthy bus decays to 0.83 V_pack after 5 s (`t_weld` 5 s, `k_weld` 0.95).

**Fuse model** (`ee_lib/Switches & Breakers/Fuse`): a tabulated time-current curve (`I_vec`, `t_vec`), with `exposemeasurement = visible` so the blown state can be read for diagnostics.

### 2.5 Safety / E-stop chain (hardwired, plant side, 1 ms discrete logic)

```
Loop_OK      = ~EStop_Front_Pressed & ~EStop_Rear_Pressed & WirelessEStop_OK & ~Bumper_Pressed & MCU_Safety_OK & Key_Switch_On
Reset_edge   = rising(Safety_Reset_Req | Reset_Button_Pressed)
Reset_pulse  = OnOffDelay(Reset_edge, Td_on = 0, Td_off = P.power.safety.t_reset_hold = 0.05 s)    % D30
Armed (Q)    = SR flip-flop: Set = Loop_OK & Reset_pulse, Reset = ~Loop_OK (reset dominant), Q0 = P.power.safety.Q0
Safety_OK    = OnOffDelay(Armed, Td_on = t_relay_on 0.020 s, Td_off = t_relay_off 0.015 s)   % immediate contacts -> DRIVE STO, implement ECU stop, K3
Safety_HV_OK = OnOffDelay(Armed, Td_on = t_relay_on 0.020 s, Td_off = t_cat1 1.5 s)         % delayed contacts -> K1/K2 coil supply
K1_coil = BMS.K1_Cmd & Safety_HV_OK ;  K2_coil = BMS.K2_Cmd & Safety_HV_OK
K3_coil = MCU.ImpPwr24_Enable & Safety_OK
K4_coil = BMS.K4_Cmd & Dock_Contact            % charging allowed while E-stop pressed (robot docked, traction bus open)
each coil -> OnOffDelay(pickup t_on, dropout t_off) -> Simulink-PS -> SPST Relay (PS control)
```

**Traction stops as category 0 (D86).** On an E-stop the immediate contacts drop `Safety_OK` after `t_relay_off` (15 ms): the ESCs' STO removes the drive torque at once and the spring brakes stop the robot. S8a passes with STO within 20 ms of the edge (the twin's chain takes about 17 ms) and a stop in 0.20 m from 1.2 m/s. **REQ-SYS-063** requires the STO within at most 50 ms of the button, bounded by REQ-SYS-139's 0.6 m stop. The 50 ms is provisional until the delivered relay's release and the ESCs' STO reaction are known (G1-04). The descent cap keeps every such stop within the park criterion (CR-003, D82).

**What the delayed contacts do.** `Safety_HV_OK` drops `t_cat1` = 1.5 s later, and only then do K1 and K2 open. The delay is not a category-1 stop of traction: the drives have no torque from the first 15 ms. It keeps the 48 V bus up for two things:
- **The implement's braked run-down.** On `Safety_OK` false the implement ECU forces its speed reference to 0 and brakes the blades at up to `T_brake`, with the regen current flowing back into the pack (T-PWR-14: the blades below 10 % of speed within 3.0 s). REQ-SYS-048 requires the blades at rest within 2.0 s. The twin meets neither the time nor the rest, because the blades coast once K1 opens; the change is pending on cards G2-FSC and G2-PWR (RID-G1-01-SAF-01). If the 48 V bus dropped at once (`t_cat1` = 0), the blades would coast: about 2 min to 10 % on windage alone (a hand check from `P.structure.impl.mower` J 0.05 kg m^2 and k_aero 1.19e-5 N m s^2 at 293.2 rad/s).
- **The BMS's controlled opening** of K1 under low current.

The blades' run-down and its stop category are a question for the safety concept (G2-FSC, hazard log E-06, D86). K3 cuts the 24 V implement rail (deck lift, sprayer) at once.

**The chain in the heat (REQ-SYS-205).** Every safety part of the chain (the safety relay, the controller's safety inputs and outputs, the STO inputs) must stay within its rated ambient in the hot case. The twin's box air reaches 55.3 deg C at 50 deg C ambient with the fans (electronics spec 2.9.4), against the typical 55 deg C of a compact safety relay. The delivered relay's rating and place are G1-04 data. The conflict with REQ-SYS-114's 80 deg C enclosure stop is an open safety value with the lead systems engineer and the project owner (step 17, point 8).

MCU firmware can **open** the loop (`MCU_Safety_OK`, DO0, which is watchdog driven) but can **never close** it. Re-arming always needs a reset edge. Under D30, `MCU_Safety_OK` depends only on MCU-internal health, and the MCU emits a 50 ms `Safety_Reset_Req` pulse (DO1) on every reset request. Under D88 and D91, every switch-on, watchdog reset or brown-out waits for the operator's deliberate reset: the automatic DO1 pulse at the self-test goes (card G2-MCU). The chain itself needs no change. `Q0` = 1 is a simulation convenience with no effect, because Loop_OK is false in BOOT (DO0 low) and resets the latch at the start. The 50 ms stretch guarantees that a manual button edge and Loop_OK overlap. Power-up sequence: ICD section 6.

### 2.6 LV rails
The `ee_lib` DC-DC Converter is an averaged, behavioral, non-switching model. It regulates `v_out = v_ref` with a first-order response (`model_dynamics = yes`, τ = 2 to 5 ms soft start). Input power is `P_in = P_out + P_fixed + k I_out^2`, where k is set so that efficiency equals `eff_rated` at `P_rated` (`losses_option = proportionalToSquaredCurrent`). Below `v1_min` the converter is off and presents R1/R2. Negatives are common to GND (non-isolated). The 24 V and 12 V converters switch off below 36 V (`v1_min`), below the pack's 39.0 V operating minimum, so the rails hold over the whole pack window. A rail counts as OK within +/-10 % of its nominal voltage (`P.power.lv.rail_ok_tol`).

LV loads use `ee_lib/Passive/Dynamic Load` (DC). Its physical-signal input is the power demand P (W), and below `min_supply_voltage` (0.7 x V_nom) it becomes resistive. Demands are:
`P_24AUX = Bus_LV_LoadDemand.P_24V_W + P_fan*Fan_Cmd`, `P_12 = P_12V_W + Σ coil holds`, `P_5 = P_5V_W`.

The 3.3 V LDO is a current sink on V5, `I = P_3V3_W/3.3 + Iq`. Its status voltage is `V_3V3 = min(3.3, V5 - V_dropout)`.

The rails, converters and loads are the twin's design. Under D96 the robot's own LV supplies (the supplier's controller board and its converters) are still to be recorded (G1-04).

### 2.7 Implement loads (Variant Subsystem, `VC_IMPL`)
All 48 V implement drives use `Motor & Drive (System Level)`, an averaged drive with a DC-side power balance. They use a **voltage-dependent envelope** (`torque_speed_param = tabulated2D`, zero torque for V_dc of 30 V or less) so that a dead bus (K1 open) never needs infinite current. DRIVE handles a dead bus another way. Its envelope starts at 36 V (`P.drive.motor.vdc_t`). Its ESC cuts the motoring torque to zero between 40 and 37 V (`P.drive.esc.V_uv_derate`) and trips at 35 V (`P.drive.esc.V_uv_fault`; drive spec 2.4). The load mechanics are STRUCTURE's, through `Impl_Shaft` (D9, `P.structure.impl`).

- **Mower** (`P.power.impl.mower`): direct drive (`ratio` 1) to the blade shaft, nominal 293.2 rad/s (2800 rpm, `w_nom`). The envelope is 10 N m up to 1700 rpm, then the 1.8 kW constant-power curve to 3600 rpm, and 0 at 4000 rpm. Efficiency 89.4 % at 3000 rpm and 4.5 N m (`eff`, with no iron or base losses, which a dead bus cannot carry). The speed PI runs at 10 ms (Kp 0.6, Ki 1.5, limits -10 to +10 N m), with a ramp of +150 / -400 rad/s^2.
  - **The implement ECU** (`impl_ecu`) forces the speed reference to 0 when `Safety_OK` is false, when it is disabled, or when the IMPL_CMD frames stop for `t_can_timeout` 0.2 s. The blades are then braked at up to `T_brake` 10 N m (regenerative, within the envelope) until they fall below `run_frac` x w_nom.
  - **Today's target:** after an E-stop the blades fall below 10 % of speed within 3 s (T-PWR-14, passes).
  - **REQ-SYS-048** requires them at rest within 2.0 s of every blade stop. That value is set by risk assessment and is not yet confirmed against the mower standard, which is chosen on G2-FSC (D103). The twin meets neither the 2.0 s nor the rest; the change is pending on cards G2-FSC and G2-PWR (RID-G1-01-SAF-01).
  - The blade inertia (0.05 kg m^2) and the windage (`k_aero` 1.19e-5 N m s^2) are STRUCTURE's (`P.structure.impl.mower`).
- **Tiller** (`P.power.impl.tiller`): motor through a 12:1 gearbox to the rotor, nominal 26.18 rad/s (250 rpm). The envelope is 12 N m motor torque up to 2000 rpm, 9.5 N m at 3000 rpm and 8.2 N m at 3500 rpm (about 3 kW). The PI has Kp 0.1, Ki 0.5 and `T_brake` 12 N m. Tillage missions are not in this phase.
- **Sprayer** (on V24_IMP, `P.power.impl.sprayer`): a 24 V PM DC pump motor driven by an averaged buck (`Average-Value DC-DC Converter`, 97 %, duty = `Spray_Pump_Duty`, soft start `duty_slew` 1.5 /s). Pump torque is `k_p ω^2` and flow is `Q = k_q ω`. Eight nozzle solenoids of 8 W each are modeled as a Dynamic Load, P = 8 W x the number of open nozzles.
- **Deck lift** (mower variant, on V24_IMP): a 150 W Dynamic Load while moving (`lift_P_W`). The deck height is STRUCTURE's (`P.structure.impl.mower.h_range` 0.03 to 0.12 m, `h_rate` 0.02 m/s).
- **None**: Open Circuit blocks on the implement ports, the shaft damper `D_none` and the network tie (D35).

A DC-link clamp on each 48 V implement drive (`P.power.impl.dc_clamp`, 60 V) limits an isolated link pumped by regenerative braking. It conducts only above the pack's 55.25 V absolute maximum, so never with K1 closed. With K1 open it holds the isolated bus at its 60 V breakdown plus `Rz` times its current, so it caps the bus at or just above 60 V, not below it. `dead_bus_coastdown` recorded 60.7 V, with the implement ECU's own over-voltage logic (2026-11-08). See REQ-SYS-204 and card G2-PWR (section 2.1).

### 2.8 Charger and dock contacts

**The robot (D97).** The robot has no dock in this phase. It is charged by hand from its stand-alone charger: 54.6 V, 20 A, 100 to 240 VAC, PFC, IP54, fan cooled, through the SB50 connector [DATA]. It follows a charging procedure that the project owner approves (with D94). The charger cannot be throttled: the BMS can only let it charge or cut it.

**The twin.** The twin keeps its modelled dock and interlocked charger, and verifies docking and charging (F-10) with them.
- **Variant 1, BMS_CTRL** (`VC_CHG` 1, the default today): a controlled current source.
  - `I_chg = Dock_Contact * Charger_Enable * min(I_req_BMS, I_max, P_max/V_dock)`, with `I_max` 20 A and `P_max` 1200 W, then a rate limiter (20 A/s) and a first-order filter (τ 0.5 s)
  - Hardware OVP: I = 0 if V_dock > `V_ovp` = 55.9 V
  - Output diode (no back-feed, no inrush when K4 closes) and a 10 kOhm bleed
  - Grid power is `V I / eta`, with `eta` 0.92
- **Variant 2, CCCV_BLOCK** (`VC_CHG` 2): `batt_lib/Cyclers/Charger`, a mode-chart CC->CV->OFF source. It charges at 20 A (`cccv_I`) up to 54.6 V (`cccv_Vth`), holds a CV of 54.6 V (`cccv_V`) and ends at 2 A (`cccv_Ith`). It is used for standalone pack tests without the BMS.
- **D94 (the twin changes on card G2-PWR):** the robot's fixed 20 A charger becomes the twin's default, enabled or cut by the BMS. The BMS's request no longer throttles it.

**Charge time, a hand check.** From the 0.20 reserve to full, 0.8 x 63 Ah = 50.4 Ah at 20 A takes 2.5 h of CC, plus a CV taper of about 0.3 to 0.5 h: about 3 h. At end of life (50.4 Ah) it takes about 2.4 h. It runs only inside D94's temperature window.

### 2.9 BMS algorithms (`models/Shaddad_BMS.slx`, single precision)

**Which BMS (D96).** The delivered 13S BMS is the hardware that the requirements aim at, provisionally. The twin's STM32G4 BMS (`Shaddad_BMS`) is a variant: it models the functions below, and it is the design if the robot later takes it. The delivered BMS's thresholds, frames and behaviour come from the supplier (G1-04, R-15); until then every value below is the twin's.

1. **Measurement conditioning**:
   - `V_cell[13]`, `T_cell[13]`, `I_pack`, `V_pack`, `V_bus`, `V_dock`
   - Min/max with index: `dspstat3/Maximum` / `Minimum`, `fcn = Value and Index`
   - Plausibility: `|Σ V_cell - V_pack| < 1.0 V` (`FaultySensorThreshold`)
2. **Protection** (Simscape Battery BMS blocks; `P.power.bms`). Critical faults are latched in the fault word until a reset; the library blocks use a short disqualification (`disqual` 0.1 s). Warnings come from a second Voltage Monitoring instance with auto-clear.
   - Cell OV 4.25 V for 1 s (`V_ov`), cell UV 2.80 V for 2 s (`V_uv`); warnings at 4.22 V (`V_ov_warn`) and 3.00 V (`V_uv_warn`), each qualified over five seconds
   - Overcurrent: `|I|` above the current limit plus 20 A (`OC_ErrorLimit`) for 1 s
   - Temperature: OT 60 deg C (`T_ot` 333.15 K) and UT -20 deg C (`T_ut` 253.15 K), each for 5 s; the OT warning at 50 deg C (`T_warn`)
   - Sensor fault, contact fault (present, disabled: `contact_enable` false), precharge failure, K1 weld, fuse blown, charger OVP (55.9 V)
3. **SOC estimation**:
   - Per group, EKF (1 RC) at 0.1 s: state `x = [SOC; v1]`, input `i_group = I_pack`, measurement `V_cell`. Since D48, the variable-capacity estimators take `CellAH` from the capacity estimator, rate-limited (`est.AH_rate_lim`) and bounded (`est.AH_bounds`, 42 to 65.1 Ah)
   - Tables are group-level (Q = 63 Ah, R/15)
   - The NMC OCV slopes over the whole window (section 6.2), so the filter corrects the SOC from the voltage everywhere; a Coulomb-counting estimator runs in parallel for plausibility
   - Pack SOC (display): `SOC_pack = SOC_min / (1 - (SOC_max - SOC_min))`, saturated to [0, 1]. It reaches 0 when the weakest group is empty and 1 when the strongest group is full.
4. **SOE / runtime**:
   - Energy-counting SOE at pack level, EnergyCapacity = CellAH x 3.6 V x 13 (`est.V_nom_pack` 46.8 V)
   - `Runtime_Remaining_s = SOE*Enom*SOH*(SOC_pack - 0.10)/max(P_avg, 200 W)`, where P_avg is a moving average over 600 s (`P_avg_window`, sampled at the balancing rate). The power floor is `runtime_P_min` and the reserve `soc_reserve_runtime`
5. **SOH**: Battery Capacity Estimator (Least Squares) feeding the SOH Estimator (Capacity-Based), with AH_new = 63 and AH_eol = 50.4 (0.8 x 63).
6. **Current/power limits**:
   - Discharging and Charging Current Limit blocks: MinMax inputs, temperature tables, and the voltage-headroom term `(V - V_lim)/R0` with `R0_limit` 2.0 mOhm
   - Discharge (DCL), hottest cell: 150 A up to 45 deg C, 125 A at 50, 75 A at 55, 0 at 60 deg C; coolest cell: 40 A at -20 deg C, 75 A at -10, 110 A at 0, 150 A from 10 deg C
   - SOC taper on discharge: x0.5 below 10 % and x0.2 below 5 % (1-D Lookup on SOC_min)
   - Battery Power Estimator: the 10 s power capability `P_dis_lim_W` / `P_chg_lim_W` (limits -11.7 kW / +1.64 kW)
7. **Charge control**:
   - Battery CC-CV (MinMax input, CV at 4.20 V per group, PI Kp 750 / Ki 150). Request = `min(CC-CV output, I_chg_lim, 30 A)` (`I_chg_max` [EST]), and the charger limits it to 20 A.
   - The charge is complete when `V_max >= 4.195 V & I < 2 A` for 60 s (`charge_done`, C/30).
   - **The charge temperature window.** Today's table, hottest cell: 30 A up to 40 deg C, 15 A at 45 deg C, 0 from 50 deg C. Coolest cell: 0 A at 0 deg C, 10 A at 5, 20 A at 10, 30 A from 15 deg C. D94 sets no charge above 45 deg C and a start only at 40 deg C or less on the hottest cell (the twin changes on card G2-PWR: today 15 A at 45 deg C and a charge up to 50 deg C, with no start condition). With the fixed 20 A charger, a limit below 20 A can only cut the charge: from about 43 deg C at the hot end, and below 10 deg C at the cold end.
8. **Balancing**: Passive Cell Balancing on V_cell, 10 mV threshold, 3 mV hysteresis, 5 s delays. It is enabled only near the top of charge: `(CHARGING | REST) & V_max > 4.00 V` (`V_bal_enable`; REST is `|I| < I_rest` 2 A). With `R_bal` 60 Ohm the bleed is about 70 mA (0.29 W) per group at 4.2 V, about 0.22 % SOC per 2 h.
9. **Fan control** (the vented variant): Battery Coolant Control, ON-OFF at 40 deg C on and 35 deg C off. The command is ANDed with `T_cell_max > T_amb - 1 K` (so hotter outside air is not pulled in) and ORed with `T_cell_max > 50 C` (forced). The sealed pack has no fan (D94).
10. **Mode manager**: the MATLAB Function `bms_mode_manager.m`, section 4.1. It sequences precharge, main contactor, opening, weld check, charge and fault.
11. **Telemetry**: a Bus Creator to `Bus_BMS_Telemetry` at 0.1 s, with an alive counter, the fault and warning bit words (section 5.4) and the contactor state. The CAN frames are in ICD section 4, provisional under D96.

### 2.10 Sensor models (plant -> BMS, inside `Power_Sensors`)
`y = ZOH_Ts( Quantize( LPF_tau( x*(1+gain_err) + offset + σ·N(0,1) ) ) )`

The random sources use `simulink/Sources/Random Number` with fixed seeds (`P.power.sensors.seed` 1000; offsets from a `RandStream` with seed 1001).

| Channel (`P.power.sensors`) | σ | offset | gain err | quant | LPF τ | Ts |
|---|---|---|---|---|---|---|
| Cell V (AFE), x13 (`Vcell`) | 1.0 mV | +/-2 mV (seeded per channel) | 0 | 0.1 mV | none | 10 ms |
| I_pack (`Ipack`) | 0.05 A | 0.10 A | 0.3 % | 0.01 A | 1 ms | 10 ms |
| V_pack, V_bus, V_dock (`Vpack`) | 20 mV | 0 | 0.2 % | 1 mV | none | 10 ms |
| T_cell NTC, x13 (`Tcell`) | 0.3 K | +/-0.5 K (seeded) | 0 | 0.1 K | 5 s | 10 ms |

---

## 3. Exact block list (verified in R2026b)

The "Verified" column means the library path was found with `find_system` or `get_param`. Parameter names are the real mask or dialog names. The values are the `P.power` leaves of section 6, which the builders write into the masks.

### 3.1 Battery pack (`Pack_Grouped.slx` / `Pack_Lumped.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| B1 | Builder cell model (template) | `batt_lib/Cells/Battery Equivalent⏎Circuit` (`batt.cells.batteryEquivalentCircuit`) | yes | `batteryCylindricalGeometry` (21700: radius `P.power.cell.D`/2, height `H`), mass and capacity from `P.power.cell`; `CellModelOptions.BlockParameters`: `ThermalModel="LumpedThermalMass"`, `RCPairs="rc1"`, `CurrentDependence=Disabled`, `HysteresisModel=none`, `SelfDischargeResistor=Disabled`, `CyclingAgingMethod=disabled`. The pack SSRs overwrite every mask value with a `P.power` expression (`BatteryCapacity`, `SOCBreakpoints`, `TemperatureBreakpoints`, `OpenCircuitVoltageThermal`, `R0Thermal`, `R1Thermal`, `Tau1Thermal`, `BatteryThermalMass`, `VoltageRange`, init `stateOfCharge`, `batteryTemperature`) |
| B2 | Generated module `Shaddad_Pack` | generated: `subsystems/power/Shaddad_Battery_lib.slx/Shaddad_Pack` (by `build_shaddad_battery.m`) | yes (built) | `batteryModule(batteryParallelAssembly(cell,15),13)`, `ModelResolution="Grouped"`, `SeriesGrouping=ones(1,13)`, `ParallelGrouping=ones(1,13)` (NumModels 13), `BalancingStrategy="Passive"`, `AmbientThermalPath="CellBasedThermalResistance"`, `CoolantThermalPath="None"`, `CellParameterVariation="PercentDeviation"`, `NonCellResistance="on"`, `InterCellThermalPath="off"`, `MassFactor=P.power.pack.MassFactor` (1.355) |
| B3 | Fast module `Shaddad_Pack_Lumped` | generated: `subsystems/power/Shaddad_Battery_Lumped_lib.slx` | yes (built) | same, with `ModelResolution="Lumped"`, `BalancingStrategy="None"`, `CellParameterVariation="NoVariation"` |
| B4 | Probe (cell-group V/SOC/T truth) | `nesl_utility/Probe` | yes | variables `vParallelAssembly`, `socParallelAssembly`, `batteryTemperature` (checked by `t_pack_bms` generated_battery_interface) |
| B5 | Array of thermal nodes (only if `AmbH` is an array port) | `batt_lib/Thermal/Array of Thermal⏎Nodes Connector` | yes | n/a |
| B6 | PS-Simulink converters for the probe | `nesl_utility/PS-Simulink⏎Converter` | yes | `Unit` = `V`, `1`, `K`, `A`. `VectorFormat` = 1-D |

### 3.2 Pack thermal (`Pack_Thermal.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| T1 | AIR node mass | `fl_lib/Thermal/Thermal Elements/Thermal Mass` | yes | `mass=P.power.thermal.m_int`, `sp_heat=P.power.thermal.cp_int`, `num_ports=one`, init `T=P.power.thermal.T_init_K` |
| T2 | WALL node mass | same | yes | `mass=P.power.thermal.m_wall`, `sp_heat=P.power.thermal.cp_wall`, `T=T_init_K` |
| T3 | AIR-WALL convection | `fl_lib/Thermal/Thermal Elements/Convective Heat Transfer` | yes | `thermal_type=constant`, `area=P.power.thermal.A_int`, `heat_tr_coeff=P.power.thermal.h_in` |
| T4 | Fan ventilation AIR-AMB (vented variant) | same | yes | `thermal_type=variable`, `area=P.power.thermal.A_int`, `min_heat_tr_coeff=1e-3`. PS input h = `h_fan*k_filter*Fan_Cmd` |
| T5 | WALL-AMB external convection | same | yes | `area=P.power.thermal.A_ext`, `heat_tr_coeff=P.power.thermal.h_ext` |
| T6 | WALL-AMB radiation | `fl_lib/Thermal/Thermal Elements/Radiative Heat Transfer` | yes | `area=P.power.thermal.A_ext`, `rad_tr_coeff=P.power.thermal.rad_tr_coeff` |
| T7 | Ambient | `fl_lib/Thermal/Thermal Sources/Controlled Temperature Source` | yes | PS input from `env.T_amb` (Simulink-PS `Unit=K`, `FilteringAndDerivatives=filter`, `InputFilterTimeConstant=1`); the dock air (`dock_air.T_air`) replaces it at the fan inlet while docked when `dock_air.enable` |
| T8 | Solar load | `fl_lib/Thermal/Thermal Sources/Controlled Heat Flow Rate Source` | yes | PS input `alpha_sol*A_sol*G_solar` (W) |
| T9 | Temperature sensors AIR / WALL | `fl_lib/Thermal/Thermal Sensors/Temperature Sensor` | yes | default |
| T10 | Thermal Reference | `fl_lib/Thermal/Thermal Elements/Thermal Reference` | yes | for the sources and sensors |

### 3.3 PDU (`PDU.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| D1 | F_MAIN | `ee_lib/Switches & Breakers/Fuse` (`ee.switches.fuse`) | yes | tabulated time-current option, `I_vec=P.power.pdu.F_MAIN.I_vec`, `t_vec=...t_vec`, `R_closed=P.power.pdu.F_MAIN.R_closed`, `G_open=1e-8`, `exposemeasurement=ee.enum.exposemeasurement.visible` |
| D2-D7 | F_TL, F_TR, F_IMP, F_AUX, F_CHG, F_24IMP | same | yes | per section 6.3 |
| D8 | K1 main contactor | `ee_lib/Switches & Breakers/Relays/SPST Relay` (`ee.switches.relays.relay_spst`) | yes | `block_choices=ee.enum.controlport.ps`, `R_closed=P.power.pdu.K1.R_closed`, `G_open=1e-8`, `exposemeasurement=hidden`, `T_brk_C_S=0`, `T_mak_C_S=0`. Pickup and dropout come from the On-Off Delay in the command path |
| D9 | K2 precharge relay | same | yes | `R_closed=P.power.pdu.K2.R_closed` |
| D10 | K4 charge contactor | same | yes | `R_closed=P.power.pdu.K4.R_closed` |
| D11 | R_PRE | `ee_lib/Passive/Resistor` | yes | `R=P.power.pdu.R_pre` (22 Ohm today; about 7 to 8 Ohm under D98, the twin changes on card G2-PWR), `enable_operating_limits=ee.enum.enable.no` |
| D12 | R_SHUNT | `ee_lib/Passive/Resistor` | yes | `R=P.power.pdu.R_shunt` |
| D13 | R_cab_pack, R_cab_TL, R_cab_TR | `ee_lib/Passive/Resistor` | yes | `R=P.power.pdu.R_cab_pack`, `R_cab_trac` (both sides) |
| D14 | C_PDU (snubber + bleed) | `ee_lib/Passive/Capacitor` | yes | `c=P.power.pdu.C_pdu`, `r=ESR_pdu`, `g=g_bleed` (5 kOhm), init `vc=0` |
| D15 | S_KEY | `ee_lib/Switches & Breakers/SPST Switch` (`ee.switches.spst_ps`) | yes | `R_closed=P.power.pdu.R_key`, `G_open=1e-8`, the switching threshold at half of the 0/1 key command, `model_dynamics=0` |
| D16 | Pack current sensor | `ee_lib/Sensors & Transducers/Current Sensor` | yes | n/a |
| D17 | V_pack, V_bus, V_dock sensors | `ee_lib/Sensors & Transducers/Voltage Sensor` | yes | n/a |
| D18 | BMS supply | `ee_lib/Passive/Constant Power Load` | yes | `load_type=ee.enum.dc_ac.dc`, `consumed_power=P.power.pdu.P_bms_supply`, `min_supply_voltage=20` |
| D19 | GND | `ee_lib/Connectors & References/Electrical Reference` | yes | the only instance for the electrical network |
| D20 | Solver Configuration | `nesl_utility/Solver⏎Configuration` | yes | local Backward Euler with fixed cost at `P.sim.ssc` in the system (one 1 ms network, D56); on the pack bench `tb_pack_bms` the iterations come from `P.power.tb` |
| D21 | Coil command delays | `ee_sl_lib/General Control/On-Off Delay` | yes | `Td_on=P.power.pdu.K1.t_on`, `Td_off=...t_off`, `IC=0`, `Ts=P.power.Ts.plant` |
| D22 | Simulink-PS (coil, key) | `nesl_utility/Simulink-PS⏎Converter` | yes | `Unit=1`, `FilteringAndDerivatives=zero` (step inputs) |
| D23 | Physical ports | `nesl_utility/Connection Port` | yes | names `DC48_TracL_p`, `DC48_TracL_n`, `DC48_TracR_p`, `DC48_TracR_n` (section 5.2) |

### 3.4 Safety chain (`Safety_Chain.slx`, discrete 1 ms)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| S1 | Loop AND | `simulink/Logic and Bit Operations/Logical Operator` | yes | `Operator=AND`, `Inputs=6` (with NOT blocks for the pressed signals) |
| S2 | Reset edge and stretch | `simulink/Logic and Bit Operations/Detect Rise Positive` + `ee_sl_lib/General Control/On-Off Delay` | yes | `vinit=0`; the stretch has no on-delay and the off-delay `P.power.safety.t_reset_hold` (D30) |
| S3 | Safety relay latch | `ee_sl_lib/General Control/Set-Reset Flip-Flop` | yes | `UndefState=Reset` (reset dominant), `Q0=P.power.safety.Q0`, `Ts=P.power.Ts.plant` |
| S4 | Immediate contacts | `ee_sl_lib/General Control/On-Off Delay` | yes | `Td_on=P.power.safety.t_relay_on`, `Td_off=P.power.safety.t_relay_off`, `IC=Q0` |
| S5 | Delayed contacts (the 48 V hold-up, section 2.5) | `ee_sl_lib/General Control/On-Off Delay` | yes | `Td_on=P.power.safety.t_relay_on`, `Td_off=P.power.safety.t_cat1`, `IC=Q0` |
| S6 | Coil gating ANDs | `simulink/Logic and Bit Operations/Logical Operator` | yes | AND |

### 3.5 LV distribution (`LV_Distribution.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| L1 | DCDC_24 | `ee_lib/Semiconductors & Converters/Converters/DC-DC Converter` (`ee.semiconductors.converters.dc_dc_converter`; ports LConn1/2 = in +/-, RConn1/2 = out +/-) | yes | `Vref_option=ee.enum.converters.voltageref.internal`, `v_ref=24`, `P_rated=600`, `droop_enabled=false`, `reversible=ee.enum.converters.directionality.unidirectional`, `losses_option=ee.enum.converters.lossesOption.proportionalToSquaredCurrent`, `P_fixed=4`, `eff_rated=94`, `model_dynamics=ee.enum.converters.dynamics.yes`, `tau=0.005`, `v_ini=0`, `v1_min=36`, `R1=1e4`, `R2=1e4` |
| L2 | DCDC_12 | same | yes | `v_ref=12`, `P_rated=300`, `P_fixed=3`, `eff_rated=93`, `tau=0.005`, `v1_min=36` |
| L3 | DCDC_5 (from V12) | same | yes | `v_ref=5`, `P_rated=25`, `P_fixed=0.3`, `eff_rated=90`, `tau=0.002`, `v1_min=8` |
| L4 | LDO 3.3 V sink | `fl_lib/Electrical/Electrical Sources/Controlled Current Source` | yes | PS input `P_3V3_W/3.3 + 5e-3` |
| L5 | LOAD_24AUX, LOAD_12, LOAD_5 | `ee_lib/Passive/Dynamic Load` (`ee.passive.dynamic_load`) | yes | `load_type=ee.enum.dc_ac.dc`, `min_supply_voltage=0.7*V_nom`, `pow_min=1e-3`, `G_reverse=1e-6`, `deltaV=0.5`. PS power input through Simulink-PS (`Unit=W`, `FilteringAndDerivatives=filter`, `InputFilterTimeConstant=2e-3`) |
| L6 | K3 implement 24 V relay | `ee_lib/Switches & Breakers/Relays/SPST Relay` | yes | `block_choices=ps`, `R_closed=P.power.pdu.K3.R_closed`, `G_open=1e-8`, the switching threshold at half of the 0/1 command (a literal in the builder) |
| L7 | Rail sensors | `ee_lib/Sensors & Transducers/Voltage Sensor`, `.../Current Sensor` | yes | n/a |
| L8 | 3.3 V status | `simulink/Math Operations/MinMax` | yes | `Function=min` of [3.3, V5 - 0.25] |

### 3.6 Implement loads (`Implement_Loads` Variant Subsystem; choices `Impl_None.slx`, `Impl_Mower.slx`, `Impl_Tiller.slx`, `Impl_Sprayer.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| I1 | Variant container | `simulink/Ports & Subsystems/Variant Subsystem` | yes | `VariantControlMode=expression`, `VariantActivationTime=update diagram` (required with Simscape ports). Choices: `VC_IMPL==0/1/2/3` |
| I2 | Choice wrappers | `simulink/Ports & Subsystems/Subsystem Reference` | yes | `ReferencedSubsystem='Impl_Mower'` etc. Every choice has identical ports (ICD 3.2) |
| I3 | Mower / tiller drive | `ee_lib/Electromechanical/Motor & Drive (System Level)` (`ee.electromech.motor_and_drive`; 3 L + 3 R ports) | yes | `torque_speed_param=ee.enum.electromech.envelope.tabulated2D`, `w_t=P.power.impl.<mower|tiller>.w_t_rpm`, `vdc_t=...vdc_t` ([1 30 40 60] V), `T_t_2D` (section 6.6), `overtorque_param=no`, `losses_param=SingleEfficiencyMeasurement`, `eff` (89.4 / 89.3 %), `w_eff`, `T_eff`, `Piron=0`, `Pbase=0`, `Tc=0.02`, `J=J_motor`, `lam`, `EnableSwitch=false`, `speed0=0`, thermal variant `noThermalPort` |
| I4 | Blade / rotor inertia and load | STRUCTURE's, through `Impl_Shaft` | n/a | `P.structure.impl.mower.J` (0.05 kg m^2), `k_aero`, the tiller rotor and soil (D9) |
| I5 | Tiller gearbox | `fl_lib/Mechanical/Mechanisms/Gear Box` | yes | `ratio=P.power.impl.tiller.ratio` (12). Orientation: motor on the input. Rotor speed = motor/12 in T-PWR-15 |
| I7 | Speed sensing | `fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor` | yes | `velocity_port=true`, `position_port=false` |
| I9 | Mech reference | `fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference` | yes | n/a |
| I11 | Speed controller (implement ECU) | `simulink/Discrete/Discrete PID Controller` | yes | `Controller=PI`, `P`, `I` (section 6.6), `SampleTime=P.power.Ts.impl_ctrl`, `LimitOutput=on`, `UpperSaturationLimit=+T_max`, `LowerSaturationLimit=-T_brake`, `AntiWindupMode=clamping`, reset on the disable edge |
| I12 | Speed ramp | `simulink/Discontinuities/Rate Limiter` | yes | `RisingSlewLimit=ramp_up`, `FallingSlewLimit=ramp_dn` (+150 / -400 rad/s^2 for the mower) |
| I13 | Pump motor | `ee_lib/Electromechanical/Brushed Motors/DC Motor` (`ee.electromech.brushed.dc_motor`) | yes | `field_option=pm`, `parameterization=circuit_param`, `Ra=0.20`, `La=0.25e-3`, `emf_or_torque=kv`, `Kv=6.5e-3` V/rpm, `J=7e-5` kg m^2 (rotor + pump), `lam=1e-5` |
| I14 | Pump PWM driver (averaged) | `ee_lib/Semiconductors & Converters/Converters/Average-Value DC-DC Converter` (ports: duty PS, in +/-, out +/-) | yes | `implementation_option=behavioral`, `converter_type=buck`, `control_option=dutycycle`, `efficiency_option=constant`, `efficiency_fixed=97` |
| I15 | Nozzles + deck lift loads | `ee_lib/Passive/Dynamic Load` | yes | DC, `min_supply_voltage=P.power.impl.load_min_V` (16.8 V) |
| I16 | Implement current sensor | `ee_lib/Sensors & Transducers/Current Sensor` | yes | n/a |
| I17 | None choice terminations | `ee_lib/Connectors & References/Open Circuit` | yes | plus the network tie on Impl_Shaft (D35): `fl_lib/Electrical/Electrical Elements/Rotational Electromechanical Converter` K 1e-3 and a 1 MOhm `ee_lib/Passive/Resistor` to GND, so the shaft node stays in the DC48 network |
| I18 | Implement ECU | `impl_ecu.slx` (Subsystem Reference, mask `drv`, `impl_type`) | yes | `P.power.impl.ecu`: CAN time-out `t_can_timeout` 0.2 s, ready at `ready_frac` 0.9 of the target, fault word (OC 120 A, OV 58.4 V, UV 30 V, OT, CAN time-out, stall, safety off), 10 ms compute delay (D36) |

### 3.7 Charger (`Dock_Charger.slx`, Variant `VC_CHG`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| C1 | Charger current source | `fl_lib/Electrical/Electrical Sources/Controlled Current Source` | yes | PS input `I_chg` (A) |
| C2 | Output diode | `ee_lib/Semiconductors & Converters/Diode` | yes | variant `ideal`, `Vf=P.power.charger.diode_Vf`, `Ron=diode_Ron`, `Goff=diode_Goff` |
| C3 | Bleed | `ee_lib/Passive/Resistor` | yes | `R=P.power.charger.R_bleed` |
| C4 | Dock contacts | `ee_lib/Passive/Resistor` + `ee_lib/Switches & Breakers/SPST Switch` | yes | `R=P.power.charger.R_dock`. The switch is driven by `env.dock_contact` (STRUCTURE's dock geometry), with its threshold at half of the 0/1 command |
| C5 | Charger ramp / filter | `simulink/Discontinuities/Rate Limiter` + `ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)` | yes | slew +/-`P.power.charger.slew` (20 A/s); a unity-gain filter with `T=P.power.charger.tau` (0.5 s), sampled at `P.power.Ts.plant` |
| C6 | Charger limit math | `simulink/Math Operations/MinMax`, `Divide`, `Product` | yes | min(I_req, `I_max`, `P_max`/V) |
| C7 | OVP | `fl_lib/Physical Signals/Nonlinear Operators/PS Switch` | yes | threshold `P.power.charger.V_ovp` (55.9 V) |
| C8 | Standalone CC-CV charger | `batt_lib/Cyclers/Charger` (`batt.cyclers.charger`) | yes | `charging_current=P.power.charger.cccv_I` (20 A), `voltage_threshold=cccv_Vth` (54.6 V), `charging_voltage=cccv_V` (54.6 V), `current_threshold=cccv_Ith` (2 A); `trigger` = dock contact AND the pack voltage on the pins |

### 3.8 Sensors and status (`Power_Sensors.slx`)
| ID | Block | Library path | Verified | Key parameters |
|---|---|---|---|---|
| M1 | Noise | `simulink/Sources/Random Number` | yes | `Mean=0`, `Variance=σ^2`, `Seed=P.power.sensors.seed+k`, `SampleTime=0.01` |
| M2 | Quantizer | `simulink/Discontinuities/Quantizer` | yes | `QuantizationInterval` per section 2.10 |
| M3 | ZOH | `simulink/Discrete/Zero-Order Hold` | yes | `SampleTime=P.power.Ts.afe` |
| M4 | NTC lag | `ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)` | yes | `T=P.power.sensors.Tcell.tau` (5 s), sampled at `P.power.Ts.plant` |
| M5 | Cast to single | `simulink/Signal Attributes/Data Type Conversion` | yes | `OutDataTypeStr=single` |
| M6 | Bus packing | `simulink/Signal Routing/Bus Creator` | yes | `OutDataTypeStr=Bus: Bus_BMS_Meas` / `Bus_PowerStatus` / `Bus_PowerTruth` |
| M7 | Fuse-state bit word | `simulink/Math Operations/Gain` + `Sum` + `Data Type Conversion` | yes | sum of b_k*2^k cast to uint8 |

### 3.9 BMS (`models/Shaddad_BMS.slx`)
| ID | Block | Library path | Verified | Key parameters (`P.power.bms`, section 6.8) |
|---|---|---|---|---|
| E1 | Cell V protection (critical) | `batt_sl_lib/Protection/Battery Voltage Monitoring` (in: CellVoltage, PackVoltage; out: UndervoltageError, OvervoltageError, SensorFaultyError) | yes | `UndervoltageLimit=V_uv` (2.80), `OvervoltageLimit=V_ov` (4.25), `FaultySensorThreshold` (1.0), `QualTimeUndervoltage=qual.UV` (2), `QualTimeOvervoltage=qual.OV` (1), `QualTimeSensorFaulty=qual.sensor` (2), every `DisqualTime*=disqual` (0.1), `Ts=-1`, single |
| E2 | Cell V warnings | same | yes | `UndervoltageLimit=V_uv_warn` (3.00), `OvervoltageLimit=V_ov_warn` (4.22), qualification and disqualification 5 s |
| E3 | Overcurrent | `batt_sl_lib/Protection/Battery Current Monitoring` (in: Current, ChargingtLimit, DischargingtLimit) | yes | `ErrorLimit=OC_ErrorLimit` (20 A), `QualTime=qual.OC` (1), `DisqualTime=disqual`, `Ts=-1` |
| E4 | Temperature | `batt_sl_lib/Protection/Battery Temperature Monitoring` | yes | `MinMaxFlag=on`, `UndertemperatureLimit=T_ut` (253.15 K), `OvertemperatureLimit=T_ot` (333.15 K), `QualTimeUndertemp=qual.OT`, `QualTimeOvertemp=qual.OT` (5), `Disqual*=disqual` |
| E5 | Generic fault qualification (precharge, weld, fuse, LV rail, charger OVP) | `batt_sl_lib/Protection/Fault Qualification` | yes | `QualTime=qual_aux` (0.1), `DisqualTime=disqual`, `Ts=-1` |
| E6 | Cell contact | `batt_sl_lib/Protection/Battery Cell Contact Monitoring` | yes | present, disabled by `contact_enable` (false) until tuned |
| E7 | SOC EKF (x13), D48 | `batt_sl_lib/Estimators/SOC Estimator (Kalman Filter, Variable Capacity)` (in: Current, CellVoltage, CellTemperature, CellAH, InitialSOC) | yes | EKF, one time constant, `ekf.Q` / `R` / `P0`; CellAH from the capacity estimator, rate limit `est.AH_rate_lim`, bounds `est.AH_bounds` |
| E8 | SOC coulomb counting, D48 | `batt_sl_lib/Estimators/SOC Estimator (Coulomb Counting, Variable Capacity)` | yes | `CurrentInputPartition=off`, CellAH input as E7, `Ts=0.1` |
| E9 | Power capability | `batt_sl_lib/Estimators/Battery Power Estimator` | yes | `Ns=P.power.pack.Ns` (13), `Np=1` (group-level), `AH=P.power.group.AH` (63), `vmin=pwr_est.vmin` (2.80), `vmax=pwr_est.vmax` (4.20), `SOCmin=0.05`, `SOCmax=1`, `imin=-250`, `imax=30`, `Pmin=pwr_est.Pmin` (-11,700 W), `Pmax=pwr_est.Pmax` (1,638 W), `TimeHorizon=10`, `ChargingEfficiency=0.99`, group tables |
| E10 | Capacity estimate | `batt_sl_lib/Estimators/Battery Capacity Estimator (Least Squares)` | yes | `lsAlgorithm=Ordinary least squares`, `SocChangeThreshold=cap_est.SocChangeThreshold` (0.05), `gamma=0.99`, `Ts=1` |
| E11 | SOH | `batt_sl_lib/Estimators/SOH Estimator (Capacity-Based)` | yes | `AH_new=soh.AH_new` (63), `AH_eol=soh.AH_eol` (50.4) |
| E12 | SOE, D48 | `batt_sl_lib/Estimators/SOE Estimator (Energy Counting, Variable Energy Capacity)` (in: Current, Voltage, EnergyCapacity, InitialSOE) | yes | `CurrentInputPartition=off`, EnergyCapacity = CellAH x `est.V_nom_pack` (46.8 V), `Ts=0.1` |
| E13 | Discharge current limit | `batt_sl_lib/Current Management/Battery Discharging Current Limit` | yes | `MinMaxFlag=on`, `CellUndervoltageLimit=V_uv` (2.80), `R0=R0_limit` (2.0e-3), `T_vec=dis.T_vec`, `CurrentLimitCoolestCell=dis.I_cool`, `CurrentLimitHottestCell=dis.I_hot` |
| E14 | Charge current limit | `batt_sl_lib/Current Management/Battery Charging Current Limit` | yes | `MinMaxFlag=on`, `CellOvervoltageLimit=V_ov_warn` (4.22), `R0=R0_limit`, `T_vec=chg.T_vec`, `chg.I_cool`, `chg.I_hot` |
| E15 | CC-CV | `batt_sl_lib/Current Management/Battery CC-CV` (in: ChargingEnabled, CellVoltage, CurrentWhenCharging, CurrentWhenDischarging) | yes | `MinMaxFlag=on`, `MaxCellVoltage=V_cv` (4.20), `Kp=cccv.Kp` (750), `Ki=cccv.Ki` (150), `Kaw=1`, `Kt=1`, `Ts=0.1` |
| E16 | Balancing | `batt_sl_lib/Cell Balancing/Passive Cell Balancing` (in: u, Enable; out: Command, BalancingActive) | yes | `Threshold=bal.Threshold` (0.010), `HysteresisBand=bal.Hyst` (0.003), `Td_on=bal.Td_on` (5), `Td_off=bal.Td_off` (5), `Ts=1`, `dataType=boolean` |
| E17 | Fan control (vented variant) | `batt_sl_lib/Thermal Management/Battery Coolant Control` (in: AmbientTemperature, CoolantTemperature, CellTemperature) | yes | `MinMaxFlag=on`, `ControlStrategy=ON-OFF control`, `TempOn=P.power.thermal.TempOn` (313.15), `TempOff=TempOff` (308.15) |
| E18 | Mode manager | MATLAB Function `subsystems/power/bms_mode_manager.m` | yes | section 4.1 |
| E19 | Min/max with index | `dspstat3/Maximum`, `dspstat3/Minimum` | yes | `fcn=Value and Index`, `indexBase=One` |
| E20 | P_avg | `dspstat3/Moving Average` | yes | sliding window `P_avg_window` (600 samples at 1 s) |
| E21 | Charge-complete timer | `ee_sl_lib/General Control/On-Off Delay` | yes | on-delay `charge_done.t` (60 s), no off-delay, at `P.power.Ts.bms_slow` |
| E22 | SOC taper | `simulink/Lookup Tables/1-D Lookup Table` | yes | `soc_taper`: bp [0 0.05 0.10], table [0.2 0.5 1] (flat, clipped) |
| E23 | Rate transitions | `simulink/Signal Attributes/Rate Transition` | yes | `Integrity=on`, `Deterministic=on` (0.01 to 0.1 to 1 s) |
| E24 | Output delay (breaks loops) | `simulink/Discrete/Unit Delay` | yes | on `Bus_BMS_Cmd` |
| E25 | Bus I/O | `simulink/Signal Routing/Bus Creator` / `Bus Selector` | yes | bus types in section 5 |
| E26 | HW variant (target only) | `simulink/Ports & Subsystems/Variant Subsystem` containing `stm32blockslib/SPI Controller Transfer` (AFE), `stm32blockslib/Analog to Digital Converter`, `stm32blockslib/Digital Port Write` (contactor drivers), `stm32blockslib/Digital Port Read` (aux contacts), `stm32blockslib/FDCAN Write` / `FDCAN Read` | yes | `VC_TARGET==0` SIM pass-through / `==1` HW (`P.power.bms.hw`: AFE frame of 2 bytes x 26 values, LSB 0.1 mV, NTC 10 k / B 3950) |

---

## 4. Stateflow and MATLAB Function blocks (only where no block exists)

The power plant (`Power_Plant` and its SSRs) contains **no MATLAB Function block** (checked by `t_power_plant` no_matlab_function_blocks). Every equation there is built from library blocks. The BMS model holds the mode manager below; the HW variant has one deferred decoder.

### 4.1 `BMS_ModeManager` (MATLAB Function, `subsystems/power/bms_mode_manager.m`)

**Why not a library block:** none exists. A search of `batt_lib`, `batt_sl_lib`, `ee_lib`, `ee_sl_lib` and `fl_lib` for contactor, precharge or E-stop blocks returned nothing. It was first designed as a Stateflow chart; as built it is a MATLAB Function with the same states, ticked at 0.01 s (`chart.dt`), code-generatable.

**Interface.**
```
inputs : HV_Enable_Req, Charge_Enable_Req, Fault_Reset_Req, Shutdown_Req (boolean)   % from BMS_REQ (Bus_BmsReq); HV_Enable_Req is false while the MCU commands are timed out (mcu_cmd_timeout 0.5 s)
         Safety_OK (= Safety_HV_OK, the delayed contacts), Dock_Present, FaultCritical (boolean), V_pack, V_bus, I_pack (single)
         ChargeComplete (boolean)
outputs: K1_Cmd, K2_Cmd, K4_Cmd, Charger_Enable, HV_Ready, HV_Opening_Warn, Charging_Active, Charge_Complete (boolean)
         Contactor_State (uint8 enum), Fault_PrechargeFail, Fault_Weld (boolean), PrechargeRetries (uint8)
params : P.power.bms.chart: t_init 0.1, t_pre_min 0.25, t_pre_max 1.5, k_pre 0.97, t_overlap 0.05, t_open_wait 0.2,
         I_open_max 10, n_pre_retry 3, t_retry 5, t_weld 5, k_weld 0.95, t_chg_en 0.1, dt 0.01
```

States and transitions as built (priority shown by order):
```
INIT (0)        : all open. after(t_init) -> STANDBY
STANDBY (1)     : K1=K2=K4=0; Charger_Enable=0
   [FaultCritical]                                             -> FAULT
   [HV_Enable_Req && Safety_OK && !Dock_Present && armed]      -> PRECHARGE
   [Dock_Present && Charge_Enable_Req]                         -> CHARGING
PRECHARGE (2)   : K2=1
   [!HV_Enable_Req || !Safety_OK]                              -> STANDBY
   [after(t_pre_min) && V_bus >= k_pre*V_pack]                 -> CLOSE_MAIN
   [after(t_pre_max)] {PrechargeRetries++}                     -> PRE_FAIL
PRE_FAIL (7)    : K2=0; [PrechargeRetries>=n_pre_retry]{Fault_PrechargeFail=1} -> FAULT ; [after(t_retry)] -> STANDBY
CLOSE_MAIN (3)  : K1=1, K2=1 ; [after(t_overlap)]                -> HV_ON
HV_ON (4)       : K1=1, K2=0, HV_Ready=1 ; PrechargeRetries=0
   [!Safety_OK]                                                -> STANDBY (hardware already opened K1; HV_Ready=0)
   [FaultCritical || !HV_Enable_Req || Shutdown_Req]           -> OPENING
OPENING (5)     : HV_Opening_Warn=1 (MCU fault bit 18 HV_OPENING, class B: the MCU stops)
   [abs(I_pack) < I_open_max || after(t_open_wait)]            -> WELD_CHECK  {K1=0}
WELD_CHECK (6)  : K1=0 ; [after(t_weld)]: if V_bus > k_weld*V_pack {Fault_Weld=1} -> FAULT else -> STANDBY
CHARGING (8)    : K4=1, Charger_Enable=after(t_chg_en), Charging_Active=1
   [FaultCritical]                                             -> FAULT
   [!Dock_Present || !Charge_Enable_Req]                       -> STANDBY
   [ChargeComplete]                                            -> CHARGE_DONE
CHARGE_DONE (9) : K4=0 ; Charge_Complete=1 ; [!Dock_Present]   -> STANDBY
FAULT (10)      : all open, latched ; [rising(Fault_Reset_Req) && !FaultCritical] -> STANDBY
```

Re-entering HV after a `Safety_OK` drop needs an `HV_Enable_Req` falling edge followed by a rising edge: the `armed` latch inside STANDBY (PRE_FAIL retries keep it armed). `t_pack_bms` mode_manager_table drives every transition tick by tick.

**Decided or required changes, not yet in the twin (card G2-PWR):**
- **The precharge (D98, REQ-SYS-055, -103):** close K1 on a settled bus instead of the fixed 97 %, an early short check at about 0.2 s, a longer time-out for an unknown capacitance, and the number of attempts kept within 200 J over the whole retry sequence.
- **The charge window (D94, REQ-SYS-060):** a charge starts only with the hottest cell at 40 deg C or less, and CHARGING stops above 45 deg C.
- **The top-up and the undocking (the agreed REQ-SYS-059, the full charge, -062, and -208, the top-up; R-45, R-46):**
  - CHARGE_DONE returns to CHARGING below the top-up band (REQ-SYS-208, card G2-PWR; the band is a part-2 value, below).
  - Ending a charge switches the charger off and waits for the current to fall below `I_open_max` before K4 opens; the dock pins are then checked dead (the K4 weld).
  - STANDBY may precharge with the dock contact made once the charge has ended; K1 and K4 are never closed together (REQ-SYS-106).

**Part-2 values, approved with the baseline.** These are the BMS and power engineer's proposals (G1-01 part 2, step 7), merged into the register, and accepted provisional by the functional safety engineer where they are safety values (step 17). The project owner approved them with the baseline on 2026-11-18 (D110), so each is decided, pending on card G2-PWR, which makes the twin's changes:
- **REQ-SYS-054's warning:** OPENING holds K1 for at least 3 s with the warning out for the causes that may wait: a low charge, a cell under-voltage, an over- or under-temperature, a request to switch the high voltage off. It opens at once for an overcurrent, a cell over-voltage, a measurement or contact fault, a fuse, a charger OVP or a lost MCU. The margin over the whole chain from the warning to the brakes is about 0.4 s (13 %). Today OPENING waits at most `t_open_wait` 0.2 s.
- **REQ-SYS-208's top-up band** (card G2-PWR; REQ-SYS-059 keeps the full charge): CHARGE_DONE returns to CHARGING when the SOC falls below 0.95.

### 4.2 `AFE_Decode` (MATLAB Function, HW variant only, deferred to the target phase)

**Why:** no Simulink or STM32 block exists for a battery-monitor AFE (for example ADBMS6830 or TI BQ76952). `stm32blockslib/SPI Controller Transfer` returns raw bytes, and decoding the register map needs code. It is not used in SIM. Under D96 the HW variant serves the twin's G4 BMS only.
```
function [V_cell, T_ntc, valid] = AFE_Decode(rx_bytes)  %#codegen
% rx_bytes: uint8[4*Ns] from SPI Controller Transfer; V_cell single[Ns] (V); T_ntc single[Ns] (K); valid boolean
%   for k = 1:Ns: raw = uint16(rx(2k-1)) + bitshift(uint16(rx(2k)),8); V_cell(k) = single(raw)*LSB_V + OFFS_V
%   PEC/CRC check -> valid; NTC: R = Rpu*raw/(4095-raw); T = 1/(1/T0 + log(R/R0)/B)
```

---

## 5. Interfaces

The interfaces as built, with their elements, types and rates, are in `docs/icd.md` (sections 3.2 and 4) and `buses/buses_power.m`; they govern where this section differs. Every cell-group vector has `P.power.pack.Ns` = 13 elements.

### 5.1 Signal inputs to `Power_Plant.slx`
| Signal / bus | Producer | Type | Dim | Units | Ts |
|---|---|---|---|---|---|
| `Bus_PowerHwCmd` (DO0 MCU_Safety_OK, DO1 Safety_Reset_Req, K3 enable, implement and pump commands) | ELECTRONICS plant (MCU DO/PWM decode) | bus | n/a | n/a | 0.001 s |
| `Bus_LV_LoadDemand` | ELECTRONICS | bus (single) | n/a | W | 0.01 s |
| `Bus_EnvState` (T_amb, G_solar, dock_contact) | STRUCTURE / scenario | bus (double) | n/a | n/a | 0.1 s |
| `Bus_HmiInputs` (E-stop buttons, bumper, reset button, key switch, wireless E-stop) | Scenario / operator | bus (boolean) | n/a | n/a | 0.001 s |
| `Bus_BMS_Cmd` | BMS model | bus | n/a | n/a | 0.01 s |
| IMPL_CMD (0x140, implement ECU) | MCU over CAN1 | frame | n/a | n/a | 0.05 s |

### 5.2 Signal outputs and physical ports of `Power_Plant.slx`
| Signal / port | Consumer | Type | Dim | Units | Ts |
|---|---|---|---|---|---|
| **`DC48_TracL_p`, `DC48_TracL_n`** | DRIVE left ESC DC+ / DC- | Simscape electrical (`foundation.electrical.electrical`) | n/a | V, A | n/a |
| **`DC48_TracR_p`, `DC48_TracR_n`** | DRIVE right ESC DC+ / DC- | Simscape electrical | n/a | V, A | n/a |
| `Safety_OK` | DRIVE (STO of both ESCs) and MCU | boolean | 1 | n/a | 0.001 s |
| `Bus_BMS_Meas` | BMS model | bus (single) | n/a | n/a | 0.01 s |
| `Bus_PowerStatus` | ELECTRONICS/MCU (ADC/GPIO side) | bus | n/a | n/a | 0.001 s |
| `Bus_ImplElecState`, IMPL_FB (0x141) | MCU, logging | bus (single), frame | n/a | n/a | 0.01 s, 0.05 s |
| `Bus_PowerTruth` | logging / tests only | bus (double) | n/a | n/a | 0.1 s (decimated) |

**Rules for DRIVE:**
- Connect each ESC's DC+ and DC- to these ports.
- Do NOT place an Electrical Reference or a Solver Configuration in the DRIVE network when it is integrated. GND and the solver live in `Power_Plant/PDU`, so a second reference would bypass the shunt.
- Put the DC-link capacitor inside the ESC subsystem (2.2 mF per side and ESR 10 mOhm assumed; the ESCs' data come under G1-04).
- Handle a dead bus (K1 open). DRIVE does it with the ESC's undervoltage derate (the motoring torque 0 below 37 V) and its 35 V fault, on an envelope that starts at 36 V (drive spec 2.4). The implement drives use the zero-torque envelope at 30 V or less instead (section 2.7).
- Positive current on a `_p` port means power flows into DRIVE. Regenerative current flows back into the pack, and the BMS limits it through `I_chg_lim_A` / `P_chg_lim_W` sent to the MCU.

### 5.3 BMS model (`models/Shaddad_BMS.slx`) root I/O
| Port | Bus | Direction | Ts |
|---|---|---|---|
| In1 | `Bus_BMS_Meas` | from Power_Plant (through Rate Transition) | 0.01 s |
| In2 | BMS_REQ (`Bus_BmsReq`) | from MCU (CAN1 0x3D0) | 0.05 s |
| Out1 | `Bus_BMS_Cmd` | to Power_Plant (with Unit Delay) | 0.01 s |
| Out2 | `Bus_BMS_Telemetry` | to the BMS CAN codec (frames 0x351 to 0x3C1, ICD section 4) | 0.1 s |

### 5.4 Bus definitions (`buses/buses_power.m`)

**`Bus_BMS_Telemetry`** (BMS -> CAN codec, 0.1 s; frames 0x351 to 0x3C1, provisional under D96)
| Element | Type | Dim | Units | Notes |
|---|---|---|---|---|
| AliveCounter | uint8 | 1 | n/a | +1 per 0.1 s |
| Contactor_State | uint8 | 1 | n/a | enum from section 4.1 |
| HV_Ready | boolean | 1 | n/a | K1 closed, precharged |
| HV_Opening_Warn | boolean | 1 | n/a | the MCU stops |
| K1_Closed, K2_Closed, K4_Closed | boolean | 1 | n/a | aux contacts |
| Charging_Active, Charge_Complete | boolean | 1 | n/a | n/a |
| SOC_pack | single | 1 | - (0-1) | display SOC |
| SOC_cell | single | Ns (13) | - | EKF |
| SOC_cc | single | 1 | - | coulomb counting (plausibility) |
| SOH | single | 1 | - | n/a |
| SOE | single | 1 | - | n/a |
| E_remaining_Wh | single | 1 | Wh | n/a |
| Runtime_Remaining_s | single | 1 | s | used by GNC return-to-dock |
| V_pack_V, V_bus_V | single | 1 | V | n/a |
| I_pack_A | single | 1 | A | +discharge |
| P_pack_W | single | 1 | W | n/a |
| V_cell_min_V, V_cell_max_V, V_cell_avg_V | single | 1 | V | n/a |
| idx_cell_min, idx_cell_max | uint8 | 1 | n/a | 1-based |
| T_cell_min_K, T_cell_max_K, T_pack_air_K | single | 1 | K | n/a |
| I_dis_lim_A, I_chg_lim_A | single | 1 | A | magnitudes |
| P_dis_lim_W, P_chg_lim_W | single | 1 | W | 10 s capability |
| Balancing_Active | boolean | 1 | n/a | n/a |
| Balancing_Cmd | boolean | Ns (13) | n/a | n/a |
| Fan_Cmd | single | 1 | 0-1 | n/a |
| Fault_Level | uint8 | 1 | n/a | 0 none, 1 warn, 2 derate, 3 HV off (`fault.mask_level2`, `mask_level3`) |
| Fault_Word | uint32 | 1 | bits | b0 CELL_OV, b1 CELL_UV, b2 OVERCURRENT, b3 OVER_TEMP, b4 UNDER_TEMP, b5 V_SENSOR, b6 PRECHARGE_FAIL, b7 K1_WELD, b8 FUSE_MAIN, b9 FUSE_TL, b10 FUSE_TR, b11 FUSE_IMP, b12 FUSE_AUX, b13 FUSE_CHG, b14 CHARGER_OVP, b15 CELL_CONTACT, b16 SAFETY_OPEN, b17 LV24, b18 LV12, b19 LV5, b20 MCU_CMD_TIMEOUT |
| Warning_Word | uint32 | 1 | bits | b0 UV_warn (3.00 V), b1 OV_warn (4.22 V), b2 OT_warn (> 50 deg C), b3 SOC_low (< 0.15), b4 SOC_crit (< 0.08), b5 DERATE (DCL < 149 A or CCL < 29 A), b6 IMBALANCE (> 30 mV above 4.10 V), b7 FAN |

**`Bus_BMS_Meas`** (Power_Plant -> BMS, 0.01 s, single unless noted)
`V_cell_V[13]`, `T_cell_K[13]`, `I_pack_A`, `V_pack_V`, `V_bus_V`, `V_dock_V`, `T_amb_K`, `T_air_K`, `K1_fb`/`K2_fb`/`K4_fb` (boolean), `Dock_Present` (boolean), `Safety_OK` (boolean), `Key_On` (boolean), `FuseMain_OK` (boolean), `Fuse_Blown` (uint8).

**`Bus_BMS_Cmd`** (BMS -> Power_Plant, 0.01 s)
`K1_Cmd`, `K2_Cmd`, `K4_Cmd` (boolean), `Bal_Cmd[13]` (boolean), `Fan_Cmd` (single 0-1), `Charger_Enable` (boolean), `Charger_I_Req_A` (single, >= 0), `Charger_V_Req_V` (single).

**`Bus_BmsReq`** (MCU -> BMS, CAN1 0x3D0 BMS_REQ, 0.05 s): `HV_Enable_Req`, `Charge_Enable_Req`, `Fault_Reset_Req`, `Shutdown_Req` (boolean) and an alive counter. The BMS flags MCU_CMD_TIMEOUT when the frames stop for 0.5 s (`P.power.bms.mcu_cmd_timeout`).

**`Bus_LV_LoadDemand`** (ELECTRONICS -> POWER, 0.01 s): `P_24V_W`, `P_12V_W`, `P_5V_W`, `P_3V3_W` (single, W, >= 0). These are power demands, not currents, so they do not depend on rail voltage and cannot form algebraic loops.

**`Bus_PowerStatus`** (Power_Plant -> MCU): `V_pack_V`, `I_pack_A`, rail voltages and currents, `Rail_*_OK` (within +/-10 %), `K1_Closed` to `K4_Closed`, `Safety_OK`, `Safety_HV_OK`, `EStop_Any`, `EStop_Ch1_OK`, `EStop_Ch2_OK`, `Safety_Latched_Off`, `Dock_Present`, `Key_On`, `Fuse_Blown` (uint8: b0 MAIN, b1 TL, b2 TR, b3 IMP, b4 AUX, b5 CHG, b6 24IMP) and `V_bus_V`. These are true physical values; ELECTRONICS adds ADC quantization in its own SIM variant.

**`Bus_ImplElecState`** (0.01 s): `Impl_Type` (uint8: 0 none, 1 mower, 2 tiller, 3 sprayer), `Impl_Running`, `Impl_Fault`, `Impl_Ready` (boolean), `Impl_Speed_radps`, `Impl_Torque_Nm`, `Impl_Power_W` (electrical), `Impl_Current_A`, `Impl_Temp_K` (single) and `Fault_Code` (uint16: b0 OC, b1 OV, b2 UV, b3 OT, b4 CAN_TIMEOUT, b5 STALL, b6 SAFETY_OFF).

**`Bus_PowerTruth`** (logging, double): `SOC_true[13]`, `V_cell_true[13]`, `T_cell_true[13]`, `I_pack_A`, `P_pack_W`, the energy integrals (D40) and the losses per subsystem.

### 5.5 Static outputs to STRUCTURE (parameters, not signals)
`P.power.pack.mass_kg` = 18.5 kg [EST]; `P.power.pdu.mass_kg` = 8 kg; the pack box `P.power.pack.dims` = 0.393 x 0.3585 x 0.087 m [DATA], lying flat, at `pos_FRD` = [0.05 0 -0.30] m [EST]. STRUCTURE confirms the location.

---

## 6. Parameters (`params/param_power.m` -> `P.power`)

`param_power.m` returns a struct. `shaddad_params.m` derives the variant controls `VC_PACK` (1 Grouped, 2 Lumped), `VC_CHG` (1 BMS-controlled source, 2 batt_lib Charger) and `VC_IMPL` (shared with STRUCTURE) as plain scalars, because variant expressions should not index struct fields. All values are SI. [DATA] marks the supplier's data, [EST] an estimate.

### 6.1 Sample times and solver (`P.power.Ts`)
| Name | Value | Unit | Justification |
|---|---|---|---|
| Ts.plant | 1e-3 | s | plant logic, coil delays, safety (relay time constants of 8-20 ms) |
| Ts.local / local_mission | 1e-3 / 1e-3 | s | Simscape local solver; one 1 ms network in the system, so no 5 ms mission step (D56) |
| Ts.afe | 0.01 | s | AFE/shunt sampling |
| Ts.bms_fast / bms_slow / bal / telem | 0.01 / 0.1 / 1.0 / 0.1 | s | protection / estimators / balancing / CAN telemetry |
| Ts.impl_ctrl / can_rx | 0.01 / 0.05 | s | implement ECU speed loop / CAN receive |

### 6.2 Cell (`P.power.cell`), Molicel P42A 21700 NMC
| Name | Value | Unit | Source |
|---|---|---|---|
| Q_Ah | 4.2 | Ah | [DATA] P42A |
| V_nom / V_cv / V_max_abs / V_min_abs | 3.6 / 4.20 / 4.25 / 2.50 | V | P42A range 2.5-4.2 V; CV 4.20 V |
| mass / cp / Cth | 0.070 / 1000 / 70 | kg / J/(kg K) / J/K | mass and cp [EST] (the mass is the P42A data sheet's value) |
| D / H (dims) | 0.021 / 0.070 | m | 21700 (`batteryCylindricalGeometry`) |
| SOC_bp | [0 .05 .10 .20 .35 .50 .65 .80 .90 .95 .98 1] | - | n/a |
| T_bp | [273.15 298.15 318.15 333.15] | K | covers 0 to 60 deg C |
| OCV25 | [2.60 3.30 3.45 3.55 3.62 3.70 3.82 3.96 4.06 4.11 4.15 4.20] | V | [EST] NMC rest OCV with the end-of-discharge knee; OCV(0) 2.60 V so that the 1C cell reaches the 2.5 V cut-off just before SOC 0 (T-PWR-00) |
| kT | [0.3 0.3 0.2 0.1 0.05 0 0 0 0 0 0 0]*1e-3 | V/K | [EST] |
| R0_25 | [22 18 16 15 14 14 14 14 14 15 15 16]*1e-3 | Ohm | [EST] DC pulse resistance |
| fR0(T) | [2.5 1.0 0.8 0.75] | - | [EST] |
| R1_25 | [8 6 5 4.5 4 4 4 4 4 4.5 5 5]*1e-3 | Ohm | [EST] |
| fR1(T) | [2.5 1.0 0.70 0.60] | - | [EST] |
| tau1_25 / ftau(T) | 30 / [1.3 1.0 0.9 0.85] | s / - | [EST] |
| VoltageRange | [2.0 4.4] | V | wide enough that BMS faults trigger before solver assertions |
| I_dis_cont / I_dis_peak10s / I_chg_max | 45 / 60 / 4.2 | A per cell | [EST] P42A data sheet: 45 A continuous, 4.2 A standard charge |

### 6.3 Pack and PDU (`P.power.pack`, `P.power.pdu`)
| Name | Value | Unit | Justification |
|---|---|---|---|
| Ns / Np | 13 / 15 | - | [DATA] 13S15P, 195 cells |
| Q_Ah / E_Wh / V_nom | 63 / 2948.4 / 46.8 | Ah / Wh / V | derived (Q x Np, V_nom x Q) |
| V_oper_min / V_oper_max / V_abs_max | 39.0 / 54.6 / 55.25 | V | 3.0 / 4.20 / 4.25 V per cell |
| ModelResolution / SeriesGrouping / ParallelGrouping | "Grouped" / ones(1,13) / ones(1,13) | - | 13 models |
| MassFactor / mass_kg | 1.355 / 18.5 | - / kg | [EST] cells 13.65 + box 3.3 + BMS, busbars 1.5 kg |
| dims / pos_FRD | [0.393 0.3585 0.087] / [0.05 0 -0.30] | m | [DATA] box XMT-P1-E01, lying flat; position [EST] |
| R_noncell | 0.10e-3 per inter-group link (12 links) | Ohm | [EST] busbar + 2 bolted joints |
| R_amb_cell | 20 | K/W per cell | [EST] |
| R_bal / R_bal_switch | 60 / 0.1 | Ohm | [EST] 70 mA bleed at 4.2 V |
| SOC0 / SOC0_spread | 0.95 / 0.01 | - | mission start (D101) |
| var.cap_sigma / var.R0_sigma / seed | 0.01 / 0.03 / 42 | - | `PercentDeviation` |
| F_MAIN | I_rated 150; I_vec = 150 x [1.1 1.5 2 3 5 10]; t_vec [3600 300 30 3 0.3 0.02]; R 0.3e-3 | A, s, Ohm | [EST] for the 150 A BMS and the SB120 main connector |
| F_TL, F_TR | 125; 125 x [1.1 1.5 2 3 5 10]; [3600 200 20 2 0.2 0.01]; 0.5e-3 | A, s, Ohm | [EST] each side's dual-channel ESC (2 x 55 A DC with the 1.0 kW units) |
| F_IMP | 40; 40 x [1.1 1.5 2 3 5 10]; same t_vec; 1.2e-3 | A, s, Ohm | [EST] the mower's 1.8 kW drive |
| F_AUX | 30; [33 45 60 90 150 300]; [3600 60 5 0.5 0.05 0.005]; 2.5e-3 | A, s, Ohm | LV converters |
| F_CHG | 30; 30 x [1.1 1.5 2 3 5 10]; [3600 200 20 2 0.2 0.01]; 1.5e-3 | A, s, Ohm | the 20 A charger |
| F_24IMP | 20; [22 30 40 60 100 200]; [3600 60 5 0.5 0.05 0.005]; 4e-3 | A, s, Ohm | pump + nozzles + deck lift, 13 A or less |
| K1 (main) | R 0.2e-3; G_open 1e-8; t_on 0.015; t_off 0.008 | Ohm, S, s | EV200-class contactor [EST] |
| K2 (precharge) | 50e-3; 1e-8; 0.010; 0.005 | Ohm, S, s | 10 A relay (the D98 limit) |
| K4 (charge) | 0.3e-3; 1e-8; 0.015; 0.008 | Ohm, S, s | n/a |
| R_pre | 22 | Ohm | τ = 0.121 s with 5.5 mF; about 7 to 8 Ohm under D98 (the twin changes on card G2-PWR) |
| R_shunt | 100e-6 | Ohm | 500 A / 50 mV shunt |
| C_pdu / ESR_pdu / g_bleed | 100e-6 / 5e-3 / 2e-4 | F / Ohm / S | snubber + 5 kOhm bleed (τ 27.5 s with 5.5 mF) |
| C_bus_total_assumed | 5.5e-3 | F | 2 x 2.2 mF ESC + 1.0 mF implement drive + 0.1 mF PDU (the ESC data come under G1-04) |
| R_cab_pack / R_cab_trac | 0.34e-3 / 0.98e-3 | Ohm | [EST] pack cable / each traction cable |
| R_key | 10e-3 | Ohm | key switch + aux relay |
| P_bms_supply | 2 | W | the twin's BMS + AFE draw, upstream of K1 |
| pdu.mass_kg (PDU/LV box) | 8 | kg | n/a |

### 6.4 Thermal (`P.power.thermal`)
| Name | Value | Unit | Justification |
|---|---|---|---|
| T_init_K / T_amb_design / T_amb_max | 308.15 / 318.15 / 323.15 | K | 35 deg C start, 45 deg C design, 50 deg C peak |
| m_int, cp_int | 2, 900 | kg, J/(kg K) | [EST] internal structure and air |
| m_wall, cp_wall | 3.3, 900 | kg, J/(kg K) | [DATA] 3 mm aluminium box |
| A_int / h_in | 0.41 / 4 | m^2 / W/(m^2 K) | [EST] internal natural convection |
| A_ext / h_ext | 0.41 / 10 | m^2 / W/(m^2 K) | [EST] external, light wind under the body shell |
| eps / rad_tr_coeff | 0.85 / 4.82e-8 | - / W/(m^2 K^4) | [EST] |
| alpha_sol / A_sol | 0.6 / 0.14 | - / m^2 | [EST] partly shaded by the body |
| h_fan / k_filter | 20 / 1.0 (0.5 = clogged filter) | W/(m^2 K) / - | vented variant only |
| P_fan | 24 | W at 24 V | vented variant only |
| TempOn / TempOff / T_force | 313.15 / 308.15 / 323.15 | K | fan on 40, off 35, forced 50 deg C |
| dock_air: enable / T_air / fan_force | true / 303.15 / true | - / K / - | D45 (the vented variant with dock air); D94 turns it off by default (card G2-PWR) |

### 6.5 LV (`P.power.lv`)
| Name | Value | Unit |
|---|---|---|
| dcdc24: v_ref, P_rated, eff_rated, P_fixed, tau, v1_min, R1, R2 | 24, 600, 94, 4, 0.005, 36, 1e4, 1e4 | V, W, %, W, s, V, Ohm, Ohm |
| dcdc12 | 12, 300, 93, 3, 0.005, 36, 1e4, 1e4 | same |
| dcdc5 (from 12 V) | 5, 25, 90, 0.3, 0.002, 8, 1e4, 1e4 | same |
| ldo33: V_out, V_dropout, Iq | 3.3, 0.25, 5e-3 | V, V, A |
| Load min_supply fraction / deltaV / pow_min | 0.7 / 0.5 / 1e-3 | - / V / W |
| Coil holds K1 / K2 / K3 / K4 | 2.0 / 1.5 / 0.5 / 2.0 | W at 12 V |
| K3 (`P.power.pdu.K3`): R_closed, t_on, t_off | 5e-3, 0.008, 0.004 | Ohm, s, s |
| Rail OK window (`rail_ok_tol`) on V_nom [24 12 5 3.3] | +/-10 % | - |

### 6.6 Implements (`P.power.impl`)
| Name | Value | Unit | Justification |
|---|---|---|---|
| mower.w_t_rpm / vdc_t | [0 1700 2000 2500 3000 3600 4000] / [1 30 40 60] | rpm / V | 1 V first breakpoint (the block needs > 0) |
| mower.T_t_2D | [0 0 10 10; 0 0 10 10; 0 0 8.59 8.59; 0 0 6.88 6.88; 0 0 5.7 5.7; 0 0 4.8 4.8; 0 0 0 0] | N m | 10 N m, then the 1.8 kW constant-power curve |
| mower.eff / w_eff_rpm / T_eff / Piron / Pbase / Tc | 89.4 / 3000 / 4.5 / 0 / 0 / 0.02 | % / rpm / N m / W / W / s | no constant losses on a dead bus; eff raised so that the copper loss at (w_eff, T_eff) is 167.8 W |
| mower.J_motor / lam / ratio | 4e-4 / 1e-4 / 1 | kg m^2 / N m s/rad / - | direct drive; the blade inertia is STRUCTURE's |
| mower.w_nom | 293.2 | rad/s | 2800 rpm, from `P.structure.impl.mower.w_nom` |
| mower PI: Kp / Ki / T_brake / T_max / ramp_up / ramp_dn | 0.6 / 1.5 / 10 / 10 / 150 / -400 | N m s/rad / N m/rad / N m / N m / rad/s^2 / rad/s^2 | T_brake equals the envelope |
| mower.C_link / C_link_esr | 1.0e-3 / 10e-3 | F / Ohm | drive DC link on IMP48 |
| tiller.w_t_rpm / vdc_t | [0 2000 3000 3500 4000] / [1 30 40 60] | rpm / V | n/a |
| tiller.T_t_2D | [0 0 12 12; 0 0 12 12; 0 0 9.5 9.5; 0 0 8.2 8.2; 0 0 0 0] | N m (motor) | about 3 kW |
| tiller.eff / T_eff / ratio / J_motor / w_nom_rotor | 89.3 / 6 / 12 / 6e-4 / 26.18 | % / N m / - / kg m^2 / rad/s | 250 rpm rotor |
| tiller PI: Kp / Ki / T_brake / ramp_up / ramp_dn | 0.1 / 0.5 / 12 / 15 / -40 | n/a | n/a |
| sprayer pump: Ra, La, Kv, J, lam | 0.20 Ohm, 0.25e-3 H, 6.5e-3 V/rpm, 7e-5 kg m^2, 1e-5 N m s/rad | n/a | 24 V PM DC pump |
| sprayer: buck_eff / w_nom / k_p / k_q / duty_slew | 97 % / 314.16 rad/s / 4.9e-6 N m s^2 / 0.0255 L/min per rad/s / 1.5 1/s | n/a | bench stub, 8 L/min |
| nozzle_P_W / nNoz / lift_P_W / load_min_V | 8 / 8 / 150 / 16.8 | W / - / W / V | n/a |
| ecu: t_can_timeout / ready_frac / run_frac / I_oc / V_ov / V_uv | 0.2 / 0.9 / 0.1 / 120 / 58.4 / 30 | s / - / - / A / V / V | implement ECU (section 3.6, I18) |
| dc_clamp: BV / Rz | 60 / 0.05 | V / Ohm | above the pack's 55.25 V maximum |

### 6.7 Charger and safety (`P.power.charger`, `P.power.safety`)
| Name | Value | Unit |
|---|---|---|
| I_max / P_max / V_ovp / eta / slew / tau | 20 / 1200 / 55.9 / 0.92 / 20 / 0.5 | A / W / V / - / A/s / s |
| R_dock / diode Vf, Ron, Goff / R_bleed | 15e-3 / 0.7, 2e-3, 1e-8 / 1e4 | Ohm / V, Ohm, S / Ohm |
| CCCV block: cccv_I / cccv_Vth / cccv_V / cccv_Ith | 20 / 54.6 / 54.6 / 2 | A / V / V / A |
| safety.Q0 / t_relay_on / t_relay_off / t_cat1 / t_reset_hold | 1 (a simulation convenience, section 2.5) / 0.020 / 0.015 / 1.5 / 0.05 | - / s / s / s / s |

The charger values are the delivered charger's: 54.6 V and 20 A [DATA]; OVP 4.30 V per cell and the efficiency [EST].

### 6.8 BMS (`P.power.bms`)
| Name | Value | Unit |
|---|---|---|
| V_ov / V_ov_warn / V_uv / V_uv_warn / V_cv / V_bal_enable | 4.25 / 4.22 / 2.80 / 3.00 / 4.20 / 4.00 | V |
| T_ot / T_ut / T_warn | 333.15 / 253.15 / 323.15 | K |
| Qual times: OV / UV / OC / OT / sensor | 1 / 2 / 1 / 5 / 2 | s |
| OC_ErrorLimit / disqual / qual_aux | 20 / 0.1 / 0.1 | A / s / s |
| Group tables for EKF and limits (`P.power.group`) | R0_mat = R0/15, R1_mat = R1/15, tau1_mat = tau1, V0_mat = OCV, AH = 63 | n/a |
| EKF Q / R / P0 / SOC_init_bias | [1e-7 0; 0 1e-6] / 1e-4 / [1e-2 0; 0 1e-4] / 0 (T-PWR-11 adds +0.10 in its case) | n/a |
| Discharge limit T_vec | [253.15 263.15 273.15 283.15 318.15 323.15 328.15 333.15] | K |
| CurrentLimitCoolestCell (dis, `dis.I_cool`) | [40 75 110 150 150 150 150 150] | A |
| CurrentLimitHottestCell (dis, `dis.I_hot`) | [150 150 150 150 150 125 75 0] | A |
| Charge limit T_vec | [273.15 278.15 283.15 288.15 313.15 318.15 323.15 328.15] | K |
| CurrentLimitCoolestCell (chg, `chg.I_cool`) | [0 10 20 30 30 30 30 30] | A |
| CurrentLimitHottestCell (chg, `chg.I_hot`) | [30 30 30 30 30 15 0 0] (changed by D94, below the table) | A |
| R0_limit | 2.0e-3 | Ohm (group resistance limit) |
| I_dis_cont / I_dis_peak / I_chg_max | 150 / 250 / 30 | A (the twin's values: the top of the delivered BMS's 120-150 A continuous and within its 200-300 A peak [DATA], provisional under D96; charge 30 A [EST]) |
| soc_taper: bp / k | [0 0.05 0.10] / [0.2 0.5 1] | - |
| CC-CV Kp / Ki / Kaw / Kt | 750 / 150 / 1 / 1 | A/V, A/(V s) |
| Charge complete (`charge_done`): V / I / t | 4.195 / 2 / 60 | V / A / s |
| Balancing (`bal`): Threshold / Hyst / Td_on / Td_off | 0.010 / 0.003 / 5 / 5 | V / V / s / s |
| Power estimator (`pwr_est`): vmin / vmax / imin / imax / Pmin / Pmax | 2.80 / 4.20 / -250 / 30 / -11700 / 1638 | V / V / A / A / W / W |
| SOH (`soh`): AH_new / AH_eol | 63 / 50.4 | Ah |
| Estimators (`est`): AH_rate_lim / AH_bounds / V_nom_pack / soc_err_max | 1e-3 / [42 65.1] / 46.8 / 0.03 | Ah/s / Ah / V / - |
| Chart (`chart`) | section 4.1 | n/a |
| MCU command time-out (`mcu_cmd_timeout`) | 0.5 | s (no BMS_REQ update: HV_Enable_Req treated as false) |
| Runtime: P_avg_window / runtime_P_min / soc_reserve_runtime | 600 / 200 / 0.10 | samples (1 s) / W / - |
| Warnings (`warn`): SOC_low / SOC_crit / dV_imb / V_imb_top / I_dis_derate / I_chg_derate | 0.15 / 0.08 / 0.030 / 4.10 / 150 / 30 | - / - / V / V / A / A |
| Charger requests: V_cvl / V_dvl / V_dock_ovp / I_rest | 54.6 / 36.4 / 55.9 / 2 | V / V / V / A |

D94 changes the hot end of the charge table: no charge with the hottest cell above 45 deg C, and a charge that starts only with the hottest cell at 40 deg C or less (the twin changes on card G2-PWR).

### 6.9 Sensors (`P.power.sensors`)
As in section 2.10, plus `seed = 1000`. Per-channel offsets are drawn from a `RandStream` with seed 1001.

### 6.10 Budget (`P.power.budget`, superseded by D101)
A 3 h sprayer shift, labelled D46 in `param_power.m`; D46's own row set a 4 h mowing shift. Its values: `t_shift` 3 h, `duty_cut` 0.70, `P_cut_W` 520 W, `P_noncut_W` 380 W, `shifts_per_day` 3, `soc_window` [0.95 0.32], `soh_eol` 0.8, `eta_dis` 0.97. It is kept until card G2-PWR replaces it with the per-mission budget of section 2.1.

---

## 7. Internal structure, files, model types, rates

### 7.1 Files
```
scripts/build/build_shaddad_battery.m  % Battery Builder -> subsystems/power/Shaddad_Battery_lib.slx, Shaddad_Battery_Lumped_lib.slx (+ _param.m); run on a cell or pack change only (it also rewrites the generated parameter files)
scripts/build/build_power_pack.m       % Pack_Grouped, Pack_Lumped, Pack_Thermal
scripts/build/build_power_plant.m      % Power_Plant and its SSRs, impl_ecu, the implement and BMS CAN codecs
scripts/build/build_shaddad_bms.m      % models/Shaddad_BMS.slx
scripts/build/build_tb_pack_bms.m, build_tb_power_plant.m   % the benches
params/param_power.m                   % returns P.power (section 6), incl. derived group tables and seeded variation vectors
buses/buses_power.m                    % Bus_BMS_Meas, Bus_BMS_Cmd, Bus_BMS_Telemetry, Bus_BmsReq, Bus_BmsStatus, Bus_PowerHwCmd,
                                       %     Bus_LV_LoadDemand, Bus_PowerStatus, Bus_ImplElecState, Bus_PowerTruth
subsystems/power/Power_Plant.slx       % Subsystem Reference (top of POWER plant)
subsystems/power/Pack_Grouped.slx      % Subsystem Reference, variant choice VC_PACK==1
subsystems/power/Pack_Lumped.slx       % Subsystem Reference, variant choice VC_PACK==2
subsystems/power/Pack_Thermal.slx, PDU.slx, Safety_Chain.slx, LV_Distribution.slx, Dock_Charger.slx, Power_Sensors.slx
subsystems/power/Impl_None.slx | Impl_Mower.slx | Impl_Tiller.slx | Impl_Sprayer.slx, impl_ecu.slx
subsystems/power/bms_can_*, bmsreq_can_*, implcmd_can_*, implfb_can_*   % CAN codecs
subsystems/power/bms_mode_manager.m    % the mode manager (section 4.1)
models/Shaddad_BMS.slx                 % Model Reference, BMS firmware (bus root I/O, single, fixed-step discrete)
tests/power/t_pack_bms.m, t_power_plant.m   % test classes (section 8), helpers pp_* and pwr_*
tests/power/tb_pack_bms, tb_power_plant, tb_lv_rails, tb_impl_codec, tb_shift_cycle   % benches; each default stimulus is its tb_*_default.m
```

### 7.2 Hierarchy of `Power_Plant.slx`
```
Power_Plant (Subsystem Reference)
├── Battery_Pack            Variant Subsystem [VC_PACK: 1 Pack_Grouped | 2 Pack_Lumped]  (ports +,-,AmbH ; in Bal_Cmd[13]; out T, SOC, V truth)
├── Pack_Thermal            SSRef (ports AmbH ; in bms_cmd (Fan_Cmd), env (T_amb, G_solar, dock_contact) ; out bus thermal (T_air, T_wall, Fan_eff))
├── PDU                     SSRef (fuses, shunt, K1/K2/K4, R_pre, S_KEY, C_PDU, cables, GND, Solver Configuration, ports DC48_*)
├── Safety_Chain            SSRef (Bus_HmiInputs, MCU_Safety_OK, BMS/MCU coil requests -> coil commands, Safety_OK)
├── LV_Distribution         SSRef (AUX48 in, DCDC 24/12/5, LDO, K3, loads, V24_IMP ports)
├── Implement_Loads         Variant Subsystem [VC_IMPL: 0 None | 1 Mower | 2 Tiller | 3 Sprayer]
├── Dock_Charger            SSRef (Variant inside)
└── Power_Sensors           SSRef (-> Bus_BMS_Meas, Bus_PowerStatus, Bus_PowerTruth)
```
Since the model layout cleanup, Pack_Thermal, Safety_Chain, PDU, LV_Distribution, Dock_Charger and Power_Sensors take In Bus
Element ports (each element picked right in front of its consumer), and Pack_Thermal, Safety_Chain, PDU, LV_Distribution and
Dock_Charger send one output bus each (`thermal`, `safety`, `pdu`, `lv`, `charger`); Power_Plant keeps its ICD 3.2 ports
(`build_power_pack.m`, `build_power_plant.m`).

The BMS Model block sits **beside** `Power_Plant` in the top model, so that the plant stays a pure Subsystem Reference. Signal path: Rate Transition (1 ms to 10 ms), then BMS, then Unit Delay and Rate Transition back.

### 7.3 `Shaddad_BMS.slx` internals
`IO_In` (Variant SIM/HW, `VC_TARGET`) -> `BMS_Core` (atomic) -> `IO_Out` (Variant SIM/HW).

`BMS_Core` contains `Meas` (0.01), `Protection` (0.01), `Limits` (0.01), `Estimation` (0.1), `ChargeCtrl` (0.1), `Balancing` (1.0), `Thermal` (0.1), `ModeManager` (MATLAB Function `bms_mode_manager.m`, 0.01), `FaultWord` (0.01), `CmdOut` (0.01), `Telemetry` (0.1) and a BMS_REQ decode (0.05), as `build_shaddad_bms.m` builds them. Rate Transition blocks sit between rates.

Configuration: FixedStepDiscrete, base 0.01 s, `SingleTasking` for STM32 bare-metal, single-precision default, Embedded Coder `ert.tlc`. Hardware board: the twin's STM32G4 BMS board. **O1 is settled provisionally by D96:** the delivered BMS is the baseline and this model is its variant; the final choice comes with the supplier's data (G1-04), before G2-PWR starts.

### 7.4 Simulation speed estimate
- Network states: Grouped pack 39 (SOC, v1, T x 13) + thermal 2 + DC-DC 3 + PDU cap 1 + implement 1-2 + DRIVE (DC links 2 + motors). That is about 50-60 states, with switching modes from relays, fuses, the diode and the Dynamic Loads.
- Backward Euler with fixed cost at 1 ms; the plant has one network at `P.sim.ssc.Ts` = 1 ms (D56), so the 5 ms mission step of revision A is not available. Long runs use the Lumped pack (`VC_PACK` 2) and the endurance fidelity set. Splitting the slow pack/LV/thermal network from the drive network is a possible later optimisation (R5).
- Fault and fuse tests use the 0.1 ms step `P.sim.ssc.Ts_fault` = 1e-4 s in the bench only.
- Disable Simscape logging (`SimscapeLogType = none`) in mission runs and log decimated buses instead.

---

## 8. Verification (`tests/power/t_pack_bms.m` on `tb_pack_bms`, `tests/power/t_power_plant.m` on `tb_power_plant`)

The DRIVE stub on the plant bench is, per side, a 2.2 mF capacitor (ESR 10 mOhm) in parallel with a controlled current source driven by a current profile. The ELECTRONICS stub is a constant `Bus_LV_LoadDemand`. Ambient 25 deg C unless stated otherwise. The known failures and their reasons are in `tests/test_baseline.json`. Several tests still check the former pack's numbers (R-29); the criteria below are the as-built pack's, and the column "Test today" says where the code still differs. Those tests are restated on card G2-PWR.

| ID | Scenario | Pass/fail criteria (as-built pack) | Test today |
|---|---|---|---|
| T-PWR-00 | Single-cell check (`Battery Equivalent Circuit` alone, 1C CC to the cut-off at 25 deg C) | Capacity 4.2 Ah +/- 1 %. V at 50 % SOC under 1C within 0.01 V of the value from the cell tables | as here; passes |
| T-PWR-01 | Pack CC discharge from 100 % to the 2.80 V cut-off, 25 deg C | Delivered charge = the limiting group's capacity (63 Ah with its variation) x (1 - its residual SOC) within 0.5 %. SOC_true decreases (sign check). Energy and V_pack at 50 % from the OCV table of 6.2 (about 2.9 kWh; about 48 V at 50 %) | also checks the former 300 Ah, 15.0-15.6 kWh and 52.0 V; fails (R-29) |
| T-PWR-02 | HPPC: 10 s pulse at 50 % SOC, 25 deg C | Instantaneous ΔV/ΔI within 15 % and the 10 s value within 5 % of the cell tables: 13 x (R0 [+ R1 (1 - e^(-10/τ1))]) / 15 + the links and K1, about 13.5 / 14.5 mOhm | as here; passes |
| T-PWR-03 | Precharge into 5.5 mF (`HV_Enable_Req` rising at t = 1 s) | Today: V_bus >= 0.97 V_pack at 0.42 +/- 0.05 s after K2 closes (τ = 22 Ohm x 5.5 mF); K1 closes after, K2 opens 50 +/- 10 ms later; K1 I^2t <= 2 A^2s; no fuse blown; HV_Ready within 0.60 s of the request. D98's limits: precharge current <= 10 A, close <= 2 A^2s, HV ready within 1.0 s; with the ESC standby load on the bench after the D98 change (card G2-PWR) | as today's; passes. **T-PWR-03.g** (`t_pack_bms` precharge_bench): K2 closes within 30 ms of the BMS-visible request (decided 2026-11-12); passes |
| T-PWR-03b | Precharge into a shorted bus (0.1 Ohm across DC48_TracL) | PRE_FAIL at 1.5 +/- 0.02 s; K1 never closes; three attempts, then FAULT with Fault_Word b6. D98: at most 200 J into the resistor over the whole retry sequence (card G2-PWR) | checks 200 J per attempt (about 190 J each, about 580 J over the three); passes |
| T-PWR-04 | E-stop (front) pressed at t0 with 100 A traction load | Safety_OK false <= 20 ms (STO). K3 open <= 30 ms. K1 opens at t0 + 1.5 s +/- 30 ms (the delayed contacts). After that, I_pack < 6 A (LV + BMS only). Releasing the E-stop does NOT re-close K1. Reset plus an HV_Enable_Req edge restores HV_Ready within 1.0 s. D86 adds a stopping check (card G2-PWR). The traction torque is zero within REQ-SYS-063's STO time after the button: at most 50 ms, provisional until G1-04, bounded by REQ-SYS-139's 0.6 m. The check runs with the drive model in place of the stubs (S8a measures the distance) | as here without the stopping check; passes |
| T-PWR-05 | LV steps: 12 V 0 to 150 W, 24 V 0 to 600 W, 5 V 0 to 15 W | Steady-state V within +/-1 %. Transient within +/-8 %, recovered (within 2 %) <= 25 ms. Efficiency at rated: 24 V 94 +/- 1 %, 12 V 93 +/- 1 %. UVLO: AUX48 ramp down, so rails off below 36 V and Rail_*_OK false. V_3V3 >= 3.2 V whenever V5 >= 3.6 V | as here; passes (`lv_steps_T05`, `lv_uvlo_rail_ok`) |
| T-PWR-06 | Short circuit at DC48_TracL (5 mOhm, at the fault step `P.sim.ssc.Ts_fault` = 1e-4 s) | F_TL blows <= 10 ms. F_MAIN intact. Fuse_Blown b1 set. BMS Fault_Word b9 set within 0.2 s | as here; passes |
| T-PWR-06b | 1.5 x F_TL rating on TracL (187.5 A for the 125 A fuse) | F_TL blows within 150-300 s, F_MAIN intact. The overload exceeds the BMS's continuous limit (150 A in the twin; 120 to 150 A for the delivered BMS), so the BMS trips first: the case is restated within the BMS's envelope on card G2-PWR | fails |
| T-PWR-07 | BMS overcurrent: a discharge of 200 A, 50 A above the twin's 150 A continuous limit (`I_dis_cont`), so above the trip at the current limit + 20 A = 170 A | OvercurrentError at 1.00 +/- 0.02 s. K1 opens <= 0.25 s after the fault. Fault latched. Fault_Reset clears only when current is normal | as here; passes (its log line still prints the former pack's 350 A: cosmetic, card G2-PWR) |
| T-PWR-08 | Cell OV: charger forced past the BMS (+0.5 V injected on one cell measurement, charger at the CCL) | OV fault after the 1.0 s qualification. K4 opens <= 30 ms later. Max V_cell_true <= 4.30 V (V_ov + 50 mV), which is above the cell's 4.25 V absolute maximum: the bound is restated to 4.25 V on card G2-PWR | as here, with the 4.30 V bound; passes (`ov_T08`, `ov_forced_charge`) |
| T-PWR-09 | CC-CV charge at 25 deg C from SOC 0.10 | CC at min(20 A charger, limits). CV hold V_cell_max = 4.20 V with overshoot <= 15 mV. Complete (I < 2 A for 60 s) in about 3.2 h (2.8 h of CC at 20 A plus the taper). SOC_true_min >= 0.97 at the end. Grid energy <= 3.2 kWh (a hand check: 56.7 Ah at about 49.7 V / 0.92 = 3.06 kWh) | checks the former 100 A, 3.55 V, 3.3 h and 15.8 kWh; fails (R-29) |
| T-PWR-10 | Hot charge, sealed pack (D94): 45 deg C ambient in the shade, the fixed 20 A charger | The charge does not start with the hottest cell above 40 deg C, and stops above 45 deg C; no OT fault. **T-PWR-10b** (the vented variant with dock air, 50 deg C): the former criteria (0.10 -> 0.95 <= 3.6 h, T_cell <= 45 deg C) as a variant check. **T-PWR-10c** (dock air off) is retired | today's cases are the former pack's (90 A, fans auto, <= 52 deg C): T10 and T10c fail, T10b and T10_fans_off pass; restated on card G2-PWR |
| T-PWR-11 | SOC estimation over a mowing mission of D101 (SOC_init_est = SOC_true + 0.10) | abs(SOC_pack_est - SOC_pack_true) <= 0.03 after 30 min. RMS over the mission <= 0.02. Runtime_Remaining_s error <= 15 % in the last hour | runs the former 4 h, 1.95 kW profile; fails (R-29) |
| T-PWR-12 | Balancing: one group +3 % SOC, three CC-CV cycles | Balancing only while V_max > 4.00 V. Per-channel bleed 0.07 +/- 0.02 A (4.2 V / 60 Ohm). The SOC spread decreases every cycle. No balancing during discharge | checks the former 0.17 A and 3.40 V; fails (R-29) |
| T-PWR-13 | End of life (capacity x0.8, R0 x1.5) at 45 deg C, 1000 W/m^2, from SOC 0.95 | D101: the design mowing mission ends with SOC_true >= 0.275. **T-PWR-13b** (D48): estimation at EOL, abs(SOC_est - SOC_true) <= 0.03, Runtime_Remaining error <= 10 % at SOC 0.3 | runs D46's 4 h duty and checks SOC >= 0.215 (superseded); fails |
| T-PWR-14 | Mower: spin-up to 293.2 rad/s, then E-stop | 90 % speed in <= 3.5 s. Steady speed within 2 %, ready flag set. I_imp peak <= 45 A. After E-stop, blades below 10 % speed within 3.0 s, with regenerative braking into the pack while K1 is closed. This is today's check: REQ-SYS-048 requires rest within 2.0 s, which the twin does not meet (cards G2-FSC, G2-PWR) | as here; passes |
| T-PWR-15 | Tiller: steady 26.18 rad/s, load step | Rotor speed = motor/12 (+/-0.5 %); rotor speed at LF = 1 within 2 % of 26.18 rad/s | as here; passes |
| T-PWR-16 | Sprayer: duty 0 to 1, all 8 nozzles | Pump at 90 % within 1.0 s. I_24IMP <= 13 A. V24_IMP >= 23.0 V. Ready flag set | as here; passes |
| T-PWR-17 | Bus/telemetry compliance | All Bus_* signals match their definitions. Telemetry period 0.1 s. AliveCounter increments every frame. No NaN/Inf over a T-PWR-13 run | as here |
| T-PWR-18 | Dead-bus robustness: implement and DRIVE stub torque commanded while K1 is open | No solver error. Bus current <= 1 A. Implement torque 0 when V_dc <= 30 V (blade speed < 0.1 rad/s) | as here; passes (`dead_bus_T18`, `dead_bus_coastdown`) |
| T-PWR-19 | Speed/codegen | Grouped at 1 ms runs >= 10x real time; Lumped >= 100x. `Shaddad_BMS` builds with Embedded Coder (ert, single) with no double-precision warnings. Estimated BMS core step <= 300 µs on STM32G4 at 170 MHz | wall-clock speeds; `speed_T19` fails, `codegen_GenARM` passes |
| T-PWR-20 | 24 h shift-cycle bench (D45) | Superseded by D101 (no fixed shift) and D94 (cool charging); retired or restated as a mission-and-charge cycle on card G2-PWR | `shift_cycle_T20_45C` and `_50C` fail, `shift_bench_speed` incomplete |

---

## 9. Open questions and risks

### Open questions
- **O1** Where the BMS runs. **Settled provisionally by D96:** the delivered 13S BMS is the baseline, and `Shaddad_BMS.slx` is its twin variant. Its CAN map follows the delivered BMS once the supplier's frames arrive (ICD section 4, provisional). Final choice with the supplier's data (G1-04).
- **O2** The ESCs' DC-link capacitance (5.5 mF total assumed) and standby load. They set the precharge (D98); they come with the ESC data (G1-04).
- **O3** Closed: ELECTRONICS adopted the power buses (ICD 3.2 and 4).
- **O4** Stop category. **Closed for traction by D86:** category 0, STO at once and the spring brakes. **Open for the implement:** the blades' run-down and its stop category (the 1.5 s hold-up of the 48 V bus) are a question for the safety concept (G2-FSC, hazard log E-06).
- **O5** The tilling power: out of this phase (G2-TIL). F_IMP 40 A would not carry the tiller's 3 kW envelope.
- **O6** The pack's place and its sun exposure: the box position is [EST] (`pos_FRD`); D94 asks for shade when parked, stored and charged. STRUCTURE provides the effective solar area.
- **O7** Regen into a full or hot pack. CCL is 0 at the CV limit and from 50 deg C (from 45 deg C after D94), so the MCU limits regen and a descent becomes stop-go on the brakes (D74). No brake chopper is fitted. Open for G2-PWR and G2-DRV.

### Risks
- **R1 Generated battery block.** Built and checked (`t_pack_bms` generated_battery_interface). `build_shaddad_battery.m` also rewrites the generated parameter files, so it runs only on a cell or pack change.
- **R2 Fuse `prm` option.** It is a numeric combobox. The chosen value exposes `I_vec`/`t_vec` (T-PWR-06 passes).
- **R3 Sign conventions.** The Simscape Battery BMS blocks take + = charge; the Shaddad buses + = discharge, so the BMS feeds -I_pack to the library blocks (verified T04; T-PWR-01 and T-PWR-09 check the sign).
- **R4 Cell data.** The NMC tables are [EST] from typical P42A data. The real cells' OCV, resistance and their temperature dependence come with the supplier's data or a bench test (G1-04). Hysteresis is not modeled.
- **R5 Algebraic loops.** Plant to BMS to plant is broken by the 10 ms Rate Transitions and the Unit Delay on `Bus_BMS_Cmd`. Implement speed loops are discrete at 10 ms behind Rate Transitions. Load demands are power, not voltage-dependent.
- **R6 Single Simscape network.** POWER, DRIVE and parts of STRUCTURE form one network. Exactly one Solver Configuration (in `PDU`) and one Electrical Reference (GND) are allowed.
- **R7 Dead-bus singularity** in `Motor & Drive (System Level)`. The V_dc-dependent envelope (`tabulated2D`, section 2.7) avoids it.
- **R8 Variant Subsystems with physical ports** require `VariantActivationTime = update diagram`. Every choice must expose identical connection ports, and the None choice needs Open Circuit blocks.
- **R9 Simulation speed and memory** for long missions. Use the Lumped pack and decimated logging.
- **R10 Stiff transients** (contactor close inrush τ about 0.1 ms, fuse clearing) are not resolved at 1 ms. Backward Euler stays stable but under-resolves the peaks, so fault tests use the fault step `P.sim.ssc.Ts_fault` = 1e-4 s.
- **R11 Codegen.** Cast all inputs to single and keep the table parameters single (`single(...)` in the masks). The mode manager counts ticks at 0.01 s.
- **R12 The bought BMS and charger (D96).** Until the supplier's data arrive, the twin's BMS values are provisional for the robot. The robot's BMS may cut without the warning of REQ-SYS-054, and its charger cannot be throttled (section 2.8).

---

## Appendix A - `scripts/build/build_shaddad_battery.m` (summary)
The builder is the source; in short:
```matlab
g = batteryCylindricalGeometry();  g.Radius = simscape.Value(P.cell.D/2,'m');  g.Height = simscape.Value(P.cell.H,'m');
c = batteryCell(g);  c.Mass = simscape.Value(P.cell.mass,'kg');  c.Capacity = simscape.Value(P.cell.Q_Ah,'A*hr');
c.CellModelOptions.BlockParameters: ThermalModel "LumpedThermalMass", RCPairs "rc1", CurrentDependence "Disabled", HysteresisModel "none"
m  = batteryModule(batteryParallelAssembly(c, P.pack.Np), P.pack.Ns);    % 15P, 13S
m.ModelResolution "Grouped", SeriesGrouping/ParallelGrouping ones(1,13), BalancingStrategy "Passive",
m.AmbientThermalPath "CellBasedThermalResistance", CellParameterVariation "PercentDeviation", NonCellResistance "on",
m.MassFactor P.pack.MassFactor, Name "Shaddad_Pack"            % and the Lumped twin "Shaddad_Pack_Lumped"
buildBattery(m, LibraryName="Shaddad_Battery", Directory=subsystems/power, MaskParameters="VariableNamesByType")
% build_power_pack then writes every mask value from P.power, so the generated *_param.m scripts are never run
```

## Appendix B - Hand-check numbers (for reviewers)
- Pack OCV at rest (13 x OCV25): 10 % SOC 44.9 V, 50 % 48.1 V, 100 % 54.6 V. CV 54.6 V
- Precharge today (22 Ohm, 5.5 mF): τ = 0.121 s, t95 = 0.362 s, t97 = 0.424 s, I_pk = 2.5 A, E = 8.2 J per precharge; shorted bus about 190 J per 1.5 s attempt
- Precharge under D98 (7.5 Ohm): τ = 41 ms, I_pk = 7.3 A, 398 W into a short, about 80 J per 0.2 s check
- K1 close at 97 %: ΔV about 1.6 V, loop R about 18 to 20 mOhm, I_pk about 80 to 90 A, τ about 0.1 ms, I^2t = C ΔV²/(2R) about 0.4 A^2s
- Bleed: τ = 27.5 s, so V_bus/V_pack = 0.834 after 5 s (weld-check threshold 0.95)
- Balancing: 70 mA, 0.29 W per channel at 4.2 V, 0.14 Ah per 2 h (0.22 % SOC)
- Bolted short circuit at the bus: about 3.4 kA; F_MAIN (150 A) clears 1.5 kA in 20 ms
- Mission energy at end of life (D101): 2,948 Wh x 0.80 x (0.95 - 0.275) = 1,592 Wh stored in the window; x 0.97 = 1,544 Wh to the loads
- Charge from the 0.20 reserve to full at 20 A: 50.4 Ah, 2.5 h of CC plus the taper, about 3 h
- Storage draw: the twin's 2 W BMS supply empties a pack stored at SOC 0.5 (1,474 Wh) in about 31 days; D105 asks for 90 days to 3.0 V
- Blade coast without braking, 293.2 rad/s to 10 % on windage alone: 9 J / (k_aero ω0) = about 129 s
- Sealed box in full sun at 45 deg C: 84 W into about 6.9 W/K, about 12 K above ambient
