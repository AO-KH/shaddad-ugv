# Clean, traceable models: one system bus at the top, buses inside

Status: approved by the user (2026-10-29). Owner of the decisions: the user; calls marked **[call]** were
made by the designer and can be changed.

## 1. Intent

The user's words (2026-10-29): "the high level system should be more organized each block should have buses for
the input and for the output, also make the model more organized from inside the wining and everything should be
easy to trace". The request came with two screenshots: the top model `Shaddad_System` and `Power_Plant` (inside
the plant). Asked how deep the clean-up should go, the user answered "full clean".

Why: the user studies runs (row tests, Phase 1 cases) and traces signals back through the model to explain what
the plots show. Today no diagram follows the signal flow:
- most diagrams are placed by Simulink's auto-arrange (`Simulink.BlockDiagram.arrangeSystem`);
- the power files use a creation-order grid.

In the top model, about 50 helper blocks surround the 12 main blocks: Signal Specifications, loop-breaker Unit
Delays, Rate Transitions, selectors and terminators. Together they need more than 100 lines. `Power_Sensors`
alone takes 25 lines from 8 blocks, so following one signal means chasing crossing lines.

Success: open any generated diagram and follow any signal from where it is made to where it is used, without
hunting. The top model shows each component as one block with one bus in and one bus out.

## 2. Goals, non-goals, anti-goals

Goals
- **Top model (option A of the mockups).** Each component has one input bus and one output bus, joined by one
  system bus (section 5).
- **Inside (option C).** Inputs are picked by name next to the block that uses them. Blocks that exchange several
  signals pass one named bus. Every diagram has a tidy left-to-right layout (section 6).
- **Every generated diagram.** That covers the 8 models, the 118 subsystem and library files and the 27 test
  benches: 402 diagrams in all, counting every subsystem level inside them (section 2.1).
- **One layout engine** used by every builder, so the diagrams stay clean after every rebuild (section 7).
- **A measured result.** A layout check reports on every diagram before and after (section 8).

Non-goals
- No change to what the model computes: parameters, rates, initial conditions, loop-breaker timing, variants.
- No change to the interfaces fixed in the ICD (section 3 tables). The ports of the Model files and of the
  ICD-listed Subsystem Reference files keep their names, order, types and rates. The MCU's root ports are its
  firmware interface.
- No new features, and no renamed logged signals.

Anti-goals (failure even if the diagrams look clean)
- Any change in simulation results (section 9).
- Goto/From tags, hidden or overlapping lines, lines that run behind blocks.
- Hand-edited `.slx` files.
- New nesting beyond the one wrapper around each top-level component.

### 2.1 Inventory before the change

Measured on 2026-10-29 with a read-only script over every `.slx` file in `models/`, `subsystems/` and `tests/`.
The inside of a Subsystem Reference block, a library link or a MATLAB Function block is counted in its own file.
The data is in `work/layout/inventory_before_20261029.csv`, and the script in `work/layout/inv_diagrams.m`.

| | Models | Subsystem and library files | Test benches | Total |
|---|---|---|---|---|
| Files | 8 | 118 | 27 | 153 |
| Diagrams (file roots and subsystem levels) | 58 | 288 | 56 | 402 |
| Blocks | 1,618 | 7,603 | 2,073 | 11,294 |
| Signal lines | 1,786 | 7,743 | 2,204 | 11,733 |
| Physical lines | 7 | 497 | 172 | 676 |
| Line crossings | 13,077 | 23,919 | 17,225 | 54,221 |
| Line segments running behind a block | 2,057 | 1,089 | 1,441 | 4,587 |
| Unnamed signal lines (Bus Selector and Demux outputs not counted) | 565 | 3,497 | 1,202 | 5,264 |

A crossing is counted per pair of crossing segments, so one line across a bundle of 25 lines counts 25.

How the crossings are spread:
- 126 diagrams are already clean: no crossings and no lines behind blocks.
- The median diagram has 6.5 crossings. 83 diagrams have more than 100, and 11 have more than 1,000.

The worst diagrams, by crossings:
- `tb_power_plant` and `tb_shift_cycle`: 4,331 each.
- `Shaddad_NavFusion/NavMeas`: 3,825, plus 1,343 segments behind blocks.
- `tb_nav/TruthGen`: 3,554.
- `Shaddad_NavFusion/NavOutput`: 2,011.
- `PDU`: 1,428.
- `structure_lofi`: 1,355.
- `fw_safety_supervisor`: 1,327.
- `LV_Distribution`: 1,245.
- `Power_Plant`: 1,113.
- `tb_pack_bms`: 1,090.
- The top level of `Shaddad_System`: 858.

The blocks with the most inputs:
- `Shaddad_BMS/BMS_Core/Telemetry`: 40.
- `fw_telemetry`: 38.
- `fw_safety_supervisor`: 32.
- `fw_rc_mapping`: 31.
- `Power_Sensors`: 27.

Most of these are Bus Creators fed from all over their diagram.

## 3. Approaches considered

Top model (mockups A and B, shown to the user 2026-10-29)
- **A: one system bus (picked).** Every component sends one bus into a single system bus, which goes back into
  every component. Each component picks what it needs by name. The top level has no crossings, and adding a
  signal never rewires it. Trade-off: the top level doesn't show who feeds whom. Inside each component, every
  input is labelled with its source (`sys.plant.elec_hw`).
- B: direct bus lines. Each component has one output bus, drawn to each user: 37 connections. B shows the data
  flow at the top, but the feedback loops (MCU-Plant, MCU-CAN1, GNC-CAN2) still cross, and every new dependency
  adds a line at the top.
- Dropped: changing the Model blocks' own ports to one bus each. The MCU's ports are its firmware interface, and a
  nonvirtual bus at a Model boundary carries one rate, while the MCU's inputs run at five rates.

Inside (mockups C and D on `Power_Plant`)
- **C: buses inside too (picked).** Inputs are picked next to use, and a producer-consumer pair that exchanges
  several signals passes one named bus. `Power_Sensors` gets 7 bus lines instead of 25. Trade-off: the ports of
  inner subsystem files change, and so do the benches that feed them.
- D: same wiring, tidied. Only the positions change. It's the safest option, but busy diagrams stay busy.
- Dropped: hand-placing every diagram. It's several times the work and has to be redone after every change.

Why A with C: it's the user's choice. It gives the cleanest top level, and labels at the point of use are what
make tracing easy.

## 4. Constraints

- **Generated models.** Models come from the build scripts (build plan rule 3) and are never edited by hand.
  This work changes the builders and the shared helpers, then regenerates every file.
- **MATLAB.** MATLAB runs in separate batch processes (`matlab.exe -batch`), one heavy process at a time
  (16 GB RAM), and never in the user's desktop session.
- **MCU builds.** Never run `build_shaddad_mcu('all')` or `build_shaddad_mcu()`: they call `build_fw_platform()`
  with no argument, which rewrites the five T09 control units as stubs (STATUS.md section 7). The MCU's parts come
  from four places:
  - T08 platform units (11 files): `build_fw_platform`.
  - T09 control units: `build_fw_control`. These are `fw_safety_supervisor`, `fw_mode_manager`,
    `fw_vehicle_ctrl`, `fw_implement_aux` and `fw_watchdog_mgr`.
  - The CAN2 codecs: `build_can2_codecs`.
  - The model itself: `build_shaddad_mcu('model')`.

  The only safe order is:
  1. `build_can2_codecs`
  2. `build_fw_platform` with the 11 T08 unit names only
  3. `build_fw_control`
  4. `build_shaddad_mcu('model')`

  The MCU is in scope (decision of 2026-10-29, section 15).
- **ICD contract.** `shaddad_check_interfaces` reports 0 mismatches, also with `'Compiled', true`.
- **Rates (rule 7) and loop breakers (rule 8).** Every Rate Transition and Unit Delay keeps its rate, its
  initial condition and its place in its signal path. There are 0 algebraic loops.
- **Physical (Simscape) networks.** `arrangeSystem` can detach physical branch lines; this was found on the
  DC-DC converter negatives of `LV_Distribution` (comment in `build_power_plant.m`). Every layout step must keep
  every physical connection, as checked by the existing net check (`local_netcheck` in `build_power_plant.m`).
- **Logging.** The logged names in `scn_logging` stay the same. `plot_run`, the row test and the Phase 1 report
  read the logs by name.
- **Bus objects.** Bus objects come only from `buses/buses_<domain>.m` (rule 2). The inner buses introduced here
  are virtual and need no new bus objects **[call]**.
- **Other.** P stays numeric (D58). Everything is in English.

## 5. The top model (option A)

> 2026-11-08: the top level of `Shaddad_System` now has direct links between its 11 blocks instead of `System_Bus`;
> `2026-11-08-top-level-direct-links-design.md` replaces option A for the top level. The rest of this section is the
> record of option A.

### 5.1 Structure

The top level holds only these blocks:
- the root port `ScenarioIn` and its Signal Specification `Spec_ScenarioIn` (both unchanged);
- the 11 components;
- one virtual Bus Creator, `System_Bus`.

The components are virtual subsystems built by `build_shaddad_system`. Top to bottom, in signal-flow order:

| Component | Bus name | Inside (the main block is named after its file) |
|---|---|---|
| `Operator_Console` | `operator` | the console itself: `OpSource` and its four choices |
| `Operator_RC` | `rc` | `RC_Link_Sim` (Subsystem Reference) |
| `MCU` | `mcu` | `MCU_In`, then `Shaddad_MCU` (Model) |
| `BMS` | `bms` | `Shaddad_BMS` (Model) |
| `CAN1_Net` | `can1` | `CAN1_Net_Sim` (Subsystem Reference) |
| `Plant` | `plant` | `Shaddad_Plant` (Model) |
| `Sensors` | `sensors` | `Shaddad_SensorModels` (Subsystem Reference) |
| `CAN2_Link` | `can2` | `Link_MCU_GNC` (Subsystem Reference) |
| `GNC` | `gnc` | `GNC_Variant` (`GNC_Stub` or `Shaddad_Companion`) |
| `Viz_Logging` | (no outputs) | `Viz_Logging` (Subsystem Reference) |
| `Sim_Monitors` | `monitors` | `Sim_Monitors` (Subsystem Reference) |

Ports of each component:
- **Input.** Exactly one input port, `sys`, made of In Bus Element blocks, one block per picked signal.
- **Output.** One output port named after its bus, made of Out Bus Element blocks, one block per element.
  `Viz_Logging` has no output port.

`System_Bus` takes the scenario (from `Spec_ScenarioIn`, line named `scen`) and each component's output (each
line named after its bus). Its output line, `sys`, feeds every component.

Element paths take the form `sys.<bus>.<signal>`. `<signal>` is the component's ICD output name, for example
`sys.plant.elec_hw` or `sys.mcu.out10`. When a consumer needs only part of a bus, it picks the sub-element, for
example `sys.mcu.out10.can1_drv` or `sys.plant.truth.pose2D`. Each pick block shows its path, so its name says
where the signal comes from.

### 5.2 Where today's helper blocks go

Each helper goes into the component whose timing or interface it models, as its comment in the builder says
today **[call]**:
- **Signal Specifications** in front of Model inputs go into the consuming component, right in front of its
  Model block.
- **`MCU_In`** (Bus Creators, Rate Transitions, `t_us`, `fault_inject`) goes into `MCU`.
- **Loop breakers (rule 8, D36):**
  - `EscFb_Delay`, `EscSlow_Delay`, `Impl_Delay`, `Bms_Delay` and `Adc_Delay` go into `MCU`. Their comment
    says the MCU reads the frame or ADC result of the previous period.
  - `BMS_Cmd_Delay` goes into `BMS` (the BMS ECU's one-sample compute delay) and is published as `bms.cmd_d`.
  - `ElecHw_Delay` is the sampled DI word used by the MCU, CAN1 and CAN2. It goes into `Plant` and is published
    once as `plant.elec_hw_d`, so all three users see the same delayed word **[call]**. It keeps its logged name
    `elec_hw`.
- **`MissionCmd_RT`** (0.01 s to 0.2 s, D32) goes into `GNC`.
- **The row-driver inputs** go into `Operator_Console`: `Sel_truth`, `Sel_telem`, `Sel_impl_mech`,
  `DTC_mcu_mode`, the `RT_*` Rate Transitions and the `*_Delay` Unit Delays. The console picks
  `plant.truth.pose2D`, `plant.truth.speed_fwd`, `mcu.telem.mode` and `plant.impl_mech.tankLevel` directly, so the
  selectors disappear. The VC_OPSRC 4 variant condition still propagates to the Rate Transitions and delays from
  `OpSource`.
- **`Sel_out5`, `Sel_out10`, `Sel_out50`, `Sel_out100` and `Sel_ScenarioIn`** are removed. Consumers pick the
  sub-elements instead.
- **Terminators on unused outputs** are removed. Unused outputs stay as elements of their component's bus:
  `tracks`, `impl_elec`, `power_status`, `rc_frame`, `bms_telem`, `sens_diag`, `gnc_debug`, `nav_diag`, `kpi`.

### 5.3 Logging

The 20 logged ports in `scn_logging` keep their names and decimation. Their block paths move inside the
components: for example `Plant/Shaddad_Plant` port 1 for `truth`, `Plant/ElecHw_Delay` port 1 for `elec_hw`, and
`MCU/Shaddad_MCU` port 1 for `mcu_out1k`. The table in `utilities/scn_logging.m` changes together with the build
step that sets port logging. The three `Sim_Monitors` internals logged by the override (`Min_cyl`, `Min_box`,
`Min_dyn`) move to `Sim_Monitors/Sim_Monitors/...`.

### 5.4 Layout of the top level

The layout follows mockup A:
- `ScenarioIn` is at the top.
- The components sit in one column, in the order of the table.
- Each output line runs straight right into `System_Bus`.
- The `sys` line returns below the column and runs up a trunk on the left, with one branch into each component.

## 6. Inside every file (option C)

### 6.1 Rules for every diagram

- **L1 Flow.** Signals flow left to right, and feedback paths run below.
- **L2 Ports.** Inputs enter from the left and outputs leave to the right, in port order when they sit at the
  edge. Connection Ports sit on their declared side.
- **L3 Picks next to use.** Each block's inputs from a bus are picked right in front of it (section 6.2), not
  split at the diagram edge and wired across.
- **L4 Helpers.** Rate Transitions, Unit Delays, Data Type Conversions, Signal Specifications and Zero-Order
  Holds sit next to the port or block they serve.
- **L5 Line names.** Every signal line is named: a signal line after its signal, a bus line after its bus. Lines
  out of a Bus Selector are the exception, because they already show the element name.
- **L6 No clutter.** No Goto/From, no lines behind blocks, and no overlapping blocks or labels.
- **L7 Physical band.** Physical networks sit in their own band below the signal flow, laid out by the same
  engine. Every connection must stay connected.
- **L8 Variants.** Variant choices are stacked in a column and keep identical ports (rule 6).

### 6.2 Picks: how a signal is taken from a bus

- **Inner files and subsystems.** Inner subsystems, and inner Subsystem Reference files not listed in the ICD
  tables, use In Bus Element and Out Bus Element ports. A port can have several pick blocks, each showing
  `port.element`.
- **Model files and ICD-listed files.** These keep classic Inport and Outport blocks at the root, with explicit
  types and rates (rule 5). The feasibility check (section 13) found that a bus-object typed In Bus Element port
  can't carry a sample time. Inside, each consumer gets its own Bus Selector placed right in front of it, fed by
  one bus line from the port.
- **Bus ports of inner files.** These are untyped virtual buses, so their rates come from the signals that drive
  them.

### 6.3 Buses between inner blocks

- **Bundling.** When one block sends three or more signals to the same block, they travel as one virtual bus
  **[call: the threshold of 3]**. The bus is built at the producer, with a virtual Bus Creator next to it or an
  Out Bus Element port when the producer is a subsystem or a Subsystem Reference file. It's picked at the
  consumer.
- **Combiners.** Small blocks that only combine inputs for one consumer move next to it or into it, for
  example the Mux and Sum blocks in front of `Power_Sensors`.
- **Ripple to benches.** These changes alter only the ports of inner Subsystem Reference files, such as
  `Power_Sensors`, `PDU` and `LV_Distribution`. The benches and tests that feed those files directly change in
  the same step, for example `tb_pack_bms` and `t_pack_bms` for `Power_Sensors` (section 10).

### 6.4 Model roots

- **`Shaddad_Plant`.** Inputs feed picks for `Plant_In`, `Structure`, `Power`, `Drive` and `Electronics`, whose
  ICD 3.2 ports are unchanged, and those feed the outputs. The physical band holds the Power-Drive DC48 lines,
  the Drive-Structure springs, the Power-Structure implement shaft and the Solver Configuration.
- **`Shaddad_Companion`, `Shaddad_GNC`, `Shaddad_NavFusion`, `Shaddad_Perception`, `Shaddad_BMS`.** The same
  rules apply.
- **`Shaddad_MCU` and the firmware units.** These are in scope and follow the same rules. Their builders run in
  the safe order of section 4. The MCU step waits until the derate review's changes to `build_fw_control` have
  landed.
- **Generated-code interface.** The MCU's root ports and its tasks stay as they are. Inside the units, any new
  buses are virtual, so the generated code computes the same thing.
- **Files shared with the MCU.** 13 Subsystem Reference files inside the MCU come from other domains' builders,
  for example `drive_cmd_pack`, `bms_can_unpack`, the `can2_*` codecs, `sens_acq_imu` and `sens_odom_compute`.
  When one of their ports changes, the MCU unit that uses it changes in the same step.

## 7. The layout engine

A new file, `scripts/build/shaddad_layout.m`. Every builder calls it at the end of each diagram, instead of
`arrangeSystem`, the grid, `local_arrange` or `ARR`, and `shaddad_bu('save')` calls it too. For each diagram:

1. **Read.** Collect the blocks, the signal connections (source port, destination ports, line name) and the
   physical connections (port pairs).
2. **Fingerprint.** Record every source port with its destination ports and names, and every physical node
   with its set of ports.
3. **Place.**
   - Columns follow the signal flow: longest path from the inputs, with feedback edges reversed, preferring the
     edges out of delays.
   - Within a column, blocks are ordered to reduce crossings (barycenter sweeps using port positions).
   - Inputs and outputs sit at the edges in port order, unless they are picks (L3).
   - Straight chains are aligned so their lines run straight.
   - Subsystems, Bus Creators, Bus Selectors and Muxes are sized to their port count.
   - There is room left for the name labels.
4. **Route.** Delete the lines and add them back between the same ports with smart autorouting, then restore the
   line names.
5. **Assert.** Check that the fingerprint is unchanged and run the physical net check. Any mismatch fails the
   build.

The engine is deterministic: the same diagram always gets the same layout.

## 8. Layout check and tools

- **`utilities/shaddad_layout_check.m`** (built from the inventory script). For each diagram it reports
  overlapping blocks, lines through blocks, line crossings, unnamed signal lines and unconnected ports, and
  writes a CSV report.
- **Targets.**
  - 0 overlapping blocks, 0 lines through blocks, 0 unconnected ports.
  - 0 unnamed signal lines, as L5 defines them.
  - Crossings reported, with the total at least 90 % below the baseline of 54,221 **[call]**. Every diagram that
    still has crossings is listed with its reason (a feedback loop or a physical network).
- **`utilities/shaddad_compare_logs.m`** (new). Compares the logged signals of two runs, element by element.

## 9. Proving that nothing changed

- **Rebuild first, without layout changes [call].** Several saved files are older than their builders' last edit:
  - the five T09 units (10-22) against `build_fw_control.m` (10-27);
  - most T08 units against `build_fw_platform.m` (10-27);
  - the CAN2 codecs against `build_can2_codecs.m`;
  - most drive files and the drive benches against `build_drive.m` and `build_tb_drive.m` (10-27);
  - `Electronics_Plant` and its `elp_*` files against `build_electronics_plant.m` (10-23).

  Some builders also read parameter values when they build. So step 1 regenerates every file in scope with
  today's builders and no layout change, then compares its runs with runs on today's files. If they match, today
  is the baseline. If they differ, the differences go to the user with their causes before any layout work starts,
  and the rebuilt set becomes the reference once the user accepts it. The builders are the source of truth (rule
  3), so builder edits are never undone to match old files. Every later comparison is against this baseline, so
  "bit-identical" measures the layout work alone.
- **Baseline runs.** A fixed set of runs, with their logs kept in `work/`
  **[call: the Phase 1 cases and a 2-minute R1 row test with the sprayer]**.
- **Compare after every step that rebuilds models.** Rerun the set and compare: every logged signal must be
  bit-identical. A difference is a defect until it is explained. If a difference can't be avoided (for example,
  the order of a sum's operands), it goes to the user with the reason before it is accepted.
- **Other checks:**
  - `shaddad_check_interfaces` reports 0 mismatches, static and compiled;
  - 0 algebraic loops;
  - `run_regression('Fast')` shows no new failures against the baseline;
  - the layout check (section 8) meets its targets.

## 10. Files this touches

- **Builders.** Everything in `scripts/build`, plus the bench builders and harnesses in `tests/` that save
  `.slx` files.
- **Shared helpers.** `shaddad_bu.m` (save) and the new `shaddad_layout.m`.
- **Utilities.** `scn_logging.m` (paths), plus the new `shaddad_layout_check.m` and `shaddad_compare_logs.m`.
- **Tests.**
  - `t_infra.m`: the paths of `GNC_Variant`, `MissionCmd_RT` and `MCU_In`, now inside components.
  - `t_framework.m`: the logging paths and `Operator_RC`.
  - Tests that feed inner files whose ports change.
- **Docs.**
  - `icd_static_sections.md` 3.1 (the top-level wiring); `icd.md` is regenerated.
  - `build_plan.md` rule 3 (the layout engine replaces `arrangeSystem`).
  - `architecture.md`, wherever a decision names a top-level block (D32 c, D36).
  - `STATUS.md`.
  - The description strings in `buses_*.m` that name top-level paths.
- **Models.** Every `.slx` file is regenerated.

## 11. Rollout and checkpoints

1. Rebuild with today's builders and no layout change, then record the baseline (section 9) and the
   before-inventory. **Checkpoint if this rebuild changes any result.**
2. Build the engine and the layout check, and try them on a few diagrams: `Power_Plant`, the `Shaddad_Plant` top,
   `LV_Distribution` and one bench.
3. Build the top model (A). **Checkpoint: send the user screenshots of the new top model and the first cleaned
   diagrams before the rollout.**
4. Roll out domain by domain (C plus the engine): power, drive, structure, electronics, sensors, RC, GNC and the
   companion, BMS, then the benches. Compare after each domain.
5. Do the MCU once the derate review's changes have landed: `build_can2_codecs`, then `build_fw_platform` with
   the 11 T08 names, then `build_fw_control`, then `build_shaddad_mcu('model')`. Compare again.
6. Update the docs, run the full final check, and write a before/after report with screenshots.

## 12. Questions for the user

1. **MCU firmware diagrams: include them, or leave the MCU's inside as it is?** Answered on 2026-10-29: include
   them, built in the safe order of section 4, after the derate review's changes have landed (section 15).

## 13. Feasibility checked before writing (2026-10-29)

A throwaway model in the scratchpad tested the parts of this design that Simulink might refuse. It had:
- three wrapper subsystems around a Model block;
- one virtual system bus fed back into every wrapper;
- nested picks (`m.b.z` inside a nonvirtual element);
- elements at 1 ms and 10 ms;
- a real loop through a Unit Delay.

Results:
- **The hub works.** The model compiled with 0 algebraic loops and simulated. Each wrapper had exactly one input
  port and one output port. A port logged inside a wrapper appeared in `logsout` under its custom name. The hub
  doesn't create artificial loops, because dependencies stay element by element.
- **Several pick blocks on one port.** Copy an existing block of that port (`add_block(existingPick, newPath)`),
  then set its `Element`. Adding a second block from the library with the same `PortName` fails.
- **Block and port names.** A bus element block can't have the same name as its port. Pick blocks get
  generated names such as `sys_plant_elec_hw` **[call]**; the label on the block still shows
  `sys.plant.elec_hw`.
- **Sample times.** A bus-object typed In Bus Element port can't carry a sample time ("Unable to modify
  attributes of this element because a bus object defines it"). That's why Model files and ICD-listed files keep
  classic Inports (section 6.2).

## 14. Left to the builder

- Spacing constants, the details of the ordering heuristic, and block sizes.
- The exact grouping of buses within the rules.
- The rebuild order: dependencies first.
- Which runs form the baseline, within the intent of section 9.
- The report format.

## 15. Decisions taken with the user (2026-10-29)

- Scope: a full clean, covering every diagram.
- Top model: A, one system bus.
- Inside: C, buses inside too.
- MCU firmware: included. The MCU builders run in the safe order of section 4, after the derate review's changes to
  `build_fw_control` have landed.
