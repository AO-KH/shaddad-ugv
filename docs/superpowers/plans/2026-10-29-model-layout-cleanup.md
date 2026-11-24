# Clean, Traceable Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Regenerate every Shaddad model so that:
- the top model is 11 components with one bus in and one bus out each, joined by one system bus;
- every diagram flows left to right, with picks next to use, buses between blocks and named lines;
- no simulation result changes.

**Architecture:** The builders stay the only source of every `.slx`.
- A new layout engine, `shaddad_layout`, re-places each diagram's blocks and re-adds its lines between the same
  ports. It asserts an unchanged connectivity fingerprint, and every builder calls it instead of `arrangeSystem`
  or the creation-order grid.
- The top model wraps the existing Model and Subsystem Reference blocks in 11 components joined by one virtual
  `System_Bus`.
- Inside the files, picks sit next to their consumers and inner blocks exchange virtual buses.
- Each step is proven by a bit-identical comparison of logged signals against a baseline, and by a per-diagram
  layout check.

**Tech Stack:** MATLAB R2026b, Simulink, Simscape (Electrical, Battery), the programmatic build API (`add_block`,
`add_line`, `set_param`), matlab.unittest, and batch MATLAB processes.

**Spec:** `docs/superpowers/specs/2026-10-29-model-layout-cleanup-design.md` (approved 2026-10-29). Read it before
any task. Its section numbers are quoted below as "spec N".

## Global Constraints

- **Generated models.** Every `.slx` comes from its builder (build plan rule 3), so never edit a model by hand.
  Change the builder, then rerun it.
- **MATLAB in batch only.** Run MATLAB only as a separate batch process:
  `"C:\Program Files\MATLAB\R2026b\bin\matlab.exe" -batch "cd('C:\Users\aomar\Desktop\Robot project'); openProject('Shaddad_UGV.prj'); <cmd>"`.
  - Run one heavy process at a time (16 GB RAM).
  - Never use the user's desktop MATLAB or the matlab MCP tools.
  - Use `run_in_background` for calls over 10 min.
- **MCU builds.** Never run `build_shaddad_mcu('all')` or `build_shaddad_mcu()`, and never call `build_fw_platform`
  without unit names. The MCU order is:
  1. `build_can2_codecs`
  2. `build_fw_platform` with the 11 T08 names
  3. `build_fw_control`
  4. `build_shaddad_mcu('model')`
- **ICD contract.** `shaddad_check_interfaces` reports 0 mismatches, both static and with `'Compiled', true`. The
  root ports of Model files and of ICD-listed Subsystem Reference files stay classic Inport and Outport blocks with
  unchanged names, order, types and rates.
- **Rates and loop breakers** (rule 7, rule 8, D36). Every Rate Transition and Unit Delay keeps its rate, its
  initial condition and its place in its signal path. `Simulink.BlockDiagram.getAlgebraicLoops` reports 0.
- **Logged names** (`scn_logging`) don't change.
- **Buses.** Inner buses are virtual, so no new bus objects are needed. Bus objects come only from
  `buses/buses_<domain>.m`.
- **Lines.** No Goto/From blocks, and no hand-set line points.
- **Bit-identical results.** Every logged leaf of the compare set (`t` and `v`) is bit-identical to the baseline
  (`isequaln`). A difference is a defect until it is explained, and an unavoidable one goes to the user before it
  is accepted.
- **Style.** P stays numeric (D58). Everything is in English. Help text and comments follow the style of the
  builder being edited.
- **Other sessions.** The derate review session may change `build_fw_control.m`, so Task 15 starts only after its
  changes have landed. No rebuild runs while another session is running simulations.
- **Bus element ports** (spec 13).
  - Create the first block of a port from the library and set its `PortName`. Create every other block of that
    port by copying it (`add_block(existingBlock, newPath)`), then set its `Element`; adding a second block from
    the library with the same `PortName` fails.
  - A bus element block's name must differ from its port's name.
  - A bus-object typed In Bus Element port can't carry a sample time.
- **Paths that tests and `build_targets` use.** Never rename a block, and never move a block that a test or
  `build_targets` addresses by path into a new subsystem. The task blocks list those paths.
- **Shared files keep their ports.** The 13 files shared with the MCU keep their ports in every task:
  - `drive_cmd_pack` and `drive_fb_unpack`;
  - `implcmd_can_pack` and `implfb_can_unpack`;
  - `bmsreq_can_pack` and `bms_can_unpack`;
  - `can2_mcu_{imu,odom,status,power}_pack` and `can2_comp_cmd_unpack`;
  - `sens_acq_imu` and `sens_odom_compute`.
- **No git.** Before a task edits files, snapshot them under
  `.superpowers/sdd/2026-10-29-model-layout/snap/task-N/`. The ledger is
  `.superpowers/sdd/2026-10-29-model-layout/progress.md`. "Record" in the steps below means both.

## Review Focus

1. **Physical connections after re-layout.** A detached branch line or two merged nodes still compiles but leaves
   a terminal floating. Expected: every physical node keeps exactly its set of ports. Tests are in Task 4
   (`testPhysicalBranch`, and the `LV_Distribution` trial, the diagram where `arrangeSystem` detached lines).
2. **Variant choices the compare set never runs.** These are `VC_IMPL` 1 and 2, `VC_PACK` 2, `VC_CHG` 2,
   `VC_OPSRC` 2 and 3, `VC_TARGET` 1, `VC_PHASE` 2, `VC_DRIVE_FID`, `VC_STRUCT_FID`, `VC_LIDAR`, `VC_TEB`,
   `VC_ROW_TRACKER` and `VC_VIZ` 1. Expected: every choice that compiled before still compiles. The test is
   `shaddad_variant_sweep` (Task 3), run in every model-changing task for its controls and in Task 16 for all.
3. **Properties that live on lines and ports.** Signal names, logging (name, decimation), test points, signal-object
   resolution and storage classes must not change when lines are re-added. The test is the fingerprint (Task 4,
   `testFingerprintKeepsPortProperties`).
4. **Dashboard bindings.** The HMI blocks in `Operator_Console/Dashboard` are bound to Constant `Value` parameters.
   Expected: they are still bound after the console is rebuilt and re-laid out. The test is Task 5,
   `testDashboardBindings`.
5. **Long missions that log to file** through `scn_logging('override')`, with the moved paths and the
   `Sim_Monitors` internals. Expected: every listed signal exists. The test is Task 5, `testLoggingOverrideLong`.

---

### Task 1: Compare tool, run set and the "today" runs

**Files:**
- Create: `utilities/shaddad_compare_logs.m`
- Create: `utilities/shaddad_layout_runs.m`
- Create: `tests/integration/t_layout.m` (a class with tag `'Layout'`; this task adds the compare tests)
- Data: `work/layout/runs/<tag>/<case>.mat`

**Interfaces:**
- Produces: `[same, rep] = shaddad_compare_logs(a, b)`.
  - `a` and `b` are run files from `shaddad_layout_runs` (variable `res` with field `S`), or `S` structs in the
    `scn_collect` form (`S.<log>.<path>.t` and `.v`).
  - `rep` is a table with one row per leaf present in either input, with columns:
    - `path`: a string such as `"truth.pose2D"`;
    - `nA`, `nB`: sample counts, 0 when the leaf is missing;
    - `identical`: `isequaln` on both `t` and `v`;
    - `max_abs_diff`: NaN when the sizes differ;
    - `first_t`: the time of the first differing sample, NaN when identical.
  - `same` is `all(rep.identical)`.
- Produces: `files = shaddad_layout_runs(tag, cases)`.
  - `cases` is a cellstr. The quick set `{'S0','S1','S8a','R1'}` is the default. The full set adds `'S2'` and
    `'S6c'`, and `'quick'` or `'full'` may be passed instead of a list.
  - Each case runs as `run_scenario(id, 'Save', false)`. R1 runs with
    `'Args', {'Rows', 4, 'Minutes', 2, 'Ambient', 45, 'Sprayer', true}`.
  - Each case saves `res` to `work/layout/runs/<tag>/<id>.mat`, with fields `id`, `S`, `wall_s`, `rtf`, `ok` and
    `msg`, and no SimulationOutput.
  - Returns the paths of the files written.

- [ ] **Step 1: Write the failing tests** in `t_layout`:
  - `testCompareIdentical`: two equal S structs with a logical leaf, an int32 leaf, and a double leaf that contains
    NaN. Expect `same == true` and every `rep.identical` true.
  - `testCompareDifferent`: the same, with one `v` sample at `t = 0.3` raised by 1e-12. Expect `same == false`, and
    on that row `first_t == 0.3` and `max_abs_diff == 1e-12` (within eps).
  - `testCompareMissing`: a leaf present only in A. Expect `same == false` and `nB == 0` on that row.
- [ ] **Step 2: Run the tests and see them fail.** Batch: `runtests('tests/integration/t_layout.m')`. Expected:
  the 3 tests fail with "Undefined function shaddad_compare_logs".
- [ ] **Step 3: Implement `shaddad_compare_logs`.** Walk both S structs recursively; a leaf is a struct with
  fields `t` and `v`.
- [ ] **Step 4: Run the tests and see them pass.** Same command; 3 passed.
- [ ] **Step 5: Implement `shaddad_layout_runs`.** Create the folders when missing. Don't change base-workspace
  variables other than what `run_scenario` already does.
- [ ] **Step 6: Check that runs are deterministic.** In two separate batch processes, run
  `shaddad_layout_runs('det1', {'S1'})` and `shaddad_layout_runs('det2', {'S1'})`, then compare the files.
  Expected: `same == true`. If not, stop and report the leaves that differ: the bit-identical rule needs
  deterministic runs.
- [ ] **Step 7: Record the today runs.** Run one batch process per case of the full set, in the background:
  `shaddad_layout_runs('today', {'<id>'})` for S0, S1, S8a, R1, S2 and S6c. Expected: 6 files with `ok == true`.
- [ ] **Step 8: Record.**

### Task 2: `build_all` and the rebuild-first baseline (spec 9)

**Files:**
- Create: `scripts/build/build_all.m`
- Modify: `tests/integration/t_layout.m` (add `testBuildAllOrder`)
- Data: `work/layout/runs/rebuilt0/`, `work/layout/rebuild0_report.md`, `work/layout/regression_baseline.txt`

**Interfaces:**
- Produces: `calls = build_all(scope, 'DryRun', tf)`.
  - `scope` is one of:
    - `'NoMCU'` (the default): every builder except `build_can2_codecs`, `build_fw_platform`, `build_fw_control`
      and `build_shaddad_mcu`;
    - `'MCU'`: the MCU order of the Global Constraints;
    - `'All'`: `'NoMCU'`, then `'MCU'`, then `build_shaddad_system`.
  - It runs the builders in the dependency order of the section **Build order** (at the end of this plan), with a
    Subsystem Reference or Model file always built after the files it instantiates. `build_shaddad_system` runs
    last.
  - It stops at the first builder error, and the error names the builder.
  - `calls` is the cellstr of calls in order, for example `"build_power_plant()"` or
    `"build_fw_platform({'fw_sbus_decoder', ...})"`. With `'DryRun', true`, nothing is built.

- [ ] **Step 1: Write the failing test** `testBuildAllOrder`. With `calls = build_all('All', 'DryRun', true)`:
  - every builder of the Build order appears exactly once, and the bench builders appear after the files they load;
  - no call is `build_shaddad_mcu()`, `build_shaddad_mcu('all')` or `build_fw_platform()` without arguments;
  - `build_can2_codecs`, `build_fw_platform(...)`, `build_fw_control` and `build_shaddad_mcu('model')` appear in
    that order;
  - the last call is `build_shaddad_system()`.
- [ ] **Step 2: Run it and see it fail** (undefined function).
- [ ] **Step 3: Implement `build_all`.**
- [ ] **Step 4: Run it and see it pass.**
- [ ] **Step 5: Rebuild with today's builders.** In the background: `build_all('NoMCU')`, then
  `build_shaddad_system()` (it may take an hour). Expected: no error, and every listed `.slx` gets a new date.
- [ ] **Step 6: Compare.** Run `shaddad_layout_runs('rebuilt0', 'full')`, one batch process per case. For each
  case, compare `rebuilt0` with `today`.
- [ ] **Step 7: Write `work/layout/rebuild0_report.md`.** For each case, give `same` and, when it differs, the
  differing leaves with `first_t`. Name the builder change that explains each difference by comparing the old and
  new files' contents and dates.
- [ ] **Step 8: Run the other baselines.** Record these results in `work/layout/regression_baseline.txt` (the
  failures that exist before any layout work):
  - `shaddad_check_interfaces('Compiled', true)`: expected 0;
  - `run_regression('Fast')`.
- [ ] **Step 9: CHECKPOINT** (controller).
  - If every case has `same == true`, the baseline tag is `today`.
  - Otherwise, show the user `rebuild0_report.md` and wait. Once the user accepts, the baseline tag is `rebuilt0`.
  - Write the chosen tag in the ledger. Every later "compare" means: compare against that tag.
- [ ] **Step 10: Record.**

### Task 3: Layout check, allowances and the variant sweep

**Files:**
- Create: `utilities/shaddad_layout_check.m`, starting from `work/layout/inv_diagrams.m` (read-only today, already
  tested on 402 diagrams)
- Create: `utilities/shaddad_layout_allowance.m`
- Create: `utilities/shaddad_variant_sweep.m`
- Create: `utilities/shaddad_layout_domains.m`
- Modify: `tests/integration/t_layout.m`
- Data: `work/layout/inventory_before.csv`, `work/layout/variant_sweep_before.csv`

**Interfaces:**
- Produces: `D = shaddad_layout_domains()`.
  - `D` is a struct with one field per domain: `top`, `power`, `drive`, `structure`, `electronics`, `sensors`, `nav`,
    `rc`, `gnc`, `bms_int`, `mcu`.
  - Each field is a cellstr of file base names: the files the domain's task regenerates or checks, benches
    included. The counts are: top 1, power 25, drive 17, structure 16, electronics 13, sensors 8, nav 10, rc 10,
    gnc 15, bms_int 9, mcu 29.
  - Together the fields cover all 153 files exactly once. A test checks this against `dir` of `models`,
    `subsystems` and `tests`.
- Produces: `T = shaddad_layout_check(target, 'Csv', file)`.
  - `target` is one of: a cellstr of file base names, a folder relative to the root (`'models'`,
    `'subsystems/power'`, `'tests/drive'`), or `'all'`.
  - `T` has one row per diagram. It keeps the columns of `inv_diagrams.m` and adds `overlaps` (pairs of block
    rectangles that intersect, each extended downwards by 16 px for its name label) and `unconnected` (unconnected
    input, output and physical ports).
  - Files are loaded and closed again, and nothing is saved.
- Produces: `[pass, F] = shaddad_layout_check('targets', T)`.
  - `F` is the table of rows that fail spec 8: `overlaps`, `through_blocks`, `unnamed` and `unconnected` must be 0,
    and `crossings` must be within the allowance.
  - `pass` is `isempty(F)`.
- Produces: `A = shaddad_layout_allowance()`. A table with columns `diagram` (full path, e.g.
  `'PDU/Isolation'`), `max_crossings` and `reason`, which must not be empty. An unlisted diagram allows 0. It starts
  empty; the domain tasks add rows.
- Produces: `R = shaddad_variant_sweep(controls)`.
  - `controls` is a cellstr of `VC_*` names, every control by default.
  - The values tried for each control are those that appear in the `VariantControl` expressions (`VC_X == k`) of
    the models.
  - For each control and value, it sets that value, keeps the others at their defaults and updates
    `Shaddad_System`.
  - `R` is a table with columns `control`, `value`, `ok` and `msg`.
  - It restores the base values afterwards.

- [ ] **Step 1: Write the failing tests.**
  - `testCheckCounts`: a model built in the test, with exactly 1 crossing of two lines, 1 line through a block,
    1 pair of overlapping blocks, 1 unnamed line and 1 unconnected input. Expect the counts 1, 1, 1, 1 and 1.
  - `testCheckClean`: a straight 3-block chain with named lines. Expect all zeros.
  - `testTargetsAllowance`: given a T row with 4 crossings, it fails without an allowance row and passes with
    `max_crossings` 4.
- [ ] **Step 2: Run them and see them fail.**
- [ ] **Step 3: Implement `shaddad_layout_check` and `shaddad_layout_allowance`.**
- [ ] **Step 4: Run them and see them pass.**
- [ ] **Step 5: Add `testDomainsCoverAllFiles`, then `testDomainLayout(domain)`** (tag `'Layout'` only, not
  `'Fast'`).
  - `TestParameter domain = {'top','power','drive','structure','electronics','sensors','nav','rc','gnc','bms_int','mcu'}`.
  - The test runs `shaddad_layout_check(D.(domain))` and verifies `shaddad_layout_check('targets', T)`.
  - All 11 fail now; each domain task makes its own pass.
- [ ] **Step 6: Implement `shaddad_variant_sweep`, and run it on the baseline models** into
  `work/layout/variant_sweep_before.csv`. A choice that already fails today is recorded, not fixed. The rule for
  later tasks is "no new failures".
- [ ] **Step 7: Take the before-inventory on the baseline models.** Run
  `shaddad_layout_check('all', 'Csv', 'work/layout/inventory_before.csv')`. Expected: about 402 diagrams, and
  crossings near 54,221 (exact only when Task 2 changed nothing).
- [ ] **Step 8: Record.**

### Task 4: The layout engine (spec 7)

**Files:**
- Create: `scripts/build/shaddad_layout.m`
- Create: `scripts/build/shaddad_layout_fingerprint.m`
- Modify: `scripts/build/shaddad_bu.m`. In `'save'`, replace `Simulink.BlockDiagram.arrangeSystem(name)` with
  `shaddad_layout(name, 'Recursive', true)`, and update its help line. Builders that save through `shaddad_bu`
  then get every inner diagram laid out too.
- Modify: `tests/integration/t_layout.m`
- Data: `work/layout/trial/*.png`, `work/layout/trial/trial.csv`

**Interfaces:**
- Consumes: `shaddad_layout_check` (Task 3).
- Produces: `info = shaddad_layout(sys, 'Recursive', tf)`.
  - It lays out diagram `sys` (a block diagram or a subsystem path).
  - With `'Recursive', true` it also lays out every subsystem level of the same file. It doesn't go into Subsystem
    Reference blocks, Model blocks, library links, or MATLAB Function and Stateflow blocks.
  - `info` has fields `diagrams` (cellstr), `blocks` and `lines`.
  - It errors with `shaddad_layout:fingerprint` when the connectivity or the line and port properties change, and
    with `shaddad_layout:physical` when a physical port loses its node.
  - It never renames, adds or deletes a block.
- Produces: `fp = shaddad_layout_fingerprint(sys)`, a canonical struct for one diagram. `isequal(fp1, fp2)` is the
  check. It records:
  - for every signal source port (`'Block/k'`): the sorted destination ports and the line name;
  - the source port's `DataLogging`, `DataLoggingNameMode`, `DataLoggingName`, `DataLoggingDecimateData`,
    `DataLoggingDecimation`, `TestPoint`, `MustResolveToSignalObject` and storage class settings;
  - for every physical node: the sorted set of its members (`'Block/LConn k'`, `'Block/RConn k'`).
- **Placement:** follow spec 7 step 3 and rules L1-L8 of spec 6.1. The implementer chooses the spacing constants.
  Two placements are fixed:
  - **picks:** In Bus Element blocks and Bus Selectors that feed one consumer sit directly left of it, aligned
    with the input port they feed;
  - **physical blocks:** they go in a band below the signal blocks.
- **Routing:** delete the lines and add each connection back between the same port handles with
  `add_line(sys, srcH, dstH, 'autorouting', 'smart')`, physical ones included. Then restore the names, and assert
  the fingerprint.

- [ ] **Step 1: Write the failing tests.** Each builds its own small model in a temporary folder.
  - `testFingerprintKeepsPortProperties`: 12 blocks with branches, named lines, one port logged with a custom name
    and decimation 5, and one test point. After `shaddad_layout`, the fingerprint is equal and the port settings
    are read back unchanged.
  - `testFlowLeftToRight`: `In1 -> Gain -> Sum -> Out1`, plus `In2 -> Sum/2`. x increases strictly along the
    chain. The inports are in port order, top to bottom, at the smallest x, and the outport is at the largest x.
  - `testFeedbackBelow`: `Sum -> Unit Delay -> Sum/2`. Expect the Unit Delay's x greater than the Sum's x, and the
    layout check counts `overlaps` 0 and `through_blocks` 0.
  - `testDenseNoOverlap`: a seeded random 30-block diagram (`rng(7)`). Expect `overlaps` 0, `through_blocks` 0,
    and the fingerprint unchanged.
  - `testPhysicalBranch`: a Connection Port, 3 `ee_lib` resistors on one node (a branched physical line) and an
    Electrical Reference. After layout, the physical nodes have the same members and every port is connected.
  - `testPicksNextToConsumer`: In Bus Element picks `u.a` and `u.b` feed two different Gains. Each pick's right
    edge is within 80 px left of its Gain, and its output port y equals the Gain's input port y.
  - `testDeterministic`: two layouts give identical positions.
  - `testVariantStack`: a Variant Subsystem with 3 choices. Expect the choices in one column and `overlaps` 0.
- [ ] **Step 2: Run them and see them fail.**
- [ ] **Step 3: Implement `shaddad_layout_fingerprint`, then `shaddad_layout`.**
- [ ] **Step 4: Run them and see them pass,** together with the Task 1 and Task 3 tests.
- [ ] **Step 5: Trial on real diagrams, in memory, without saving:**
  - `Power_Plant`, `PDU`, `LV_Distribution` and `tb_lv_rails` with `'Recursive', true`;
  - the root of `Shaddad_Plant`.

  For each, compare the fingerprint before and after (an error stops the trial), write the `shaddad_layout_check`
  counts before and after to `work/layout/trial/trial.csv`, and save `print('-s<sys>', '-dpng')` images to
  `work/layout/trial/`. Then close without saving. Expected: fingerprints equal, physical nets intact, `overlaps` 0
  and `through_blocks` 0. Crossings will still be there, because this task changes only positions; buses come in
  the domain tasks.
- [ ] **Step 6: Change `shaddad_bu('save')`** as described under Files. Nothing is rebuilt in this task.
- [ ] **Step 7: Record.**

### Task 5: The top model, option A (spec 5)

**Files:**
- Modify: `scripts/build/build_shaddad_system.m`. Rework it into the components, `System_Bus` and the picks. The
  existing local functions are reused: `local_operator_console` becomes the `Operator_Console` component, and
  `local_mcu_in` and `local_gnc_variant` go inside `MCU` and `GNC`.
- Modify: `utilities/scn_logging.m`: the block paths of the 20 rows, and the `Sim_Monitors` internals path
  (line 72).
- Modify the tests that use top-level paths:
  - `tests/integration/t_infra.m`: `MCU_In` (line 345), `GNC_Variant`, `MissionCmd_RT`, the Model block checks
    (lines 224-238);
  - `tests/integration/t_framework.m`: the `Operator_RC` path (line 355);
  - `tests/electronics/t_mcu_control.m:289`: `[m '/MCU']` becomes `[m '/MCU/Shaddad_MCU']`;
  - `tests/sensors/t_sensors.m:473`: `[mdl '/Sensors']` becomes `[mdl '/Sensors/Shaddad_SensorModels']`.
- Modify the text that names top-level paths: `utilities/scn_ic_elec_hw.m` (its help line), and the bus
  descriptions in `buses/buses_electronics.m` (lines 75-96, 191, 196) and `buses/buses_sim.m:6`.
- Modify: `docs/icd_static_sections.md`, section 3.1 (the top-level wiring). Regenerate `docs/icd.md` with
  `shaddad_icd_report`.
- Modify: `tests/integration/t_layout.m`.

**Interfaces:**
- Consumes: `shaddad_layout` (Task 4), the compare tools (Task 1), `shaddad_layout_check` and
  `shaddad_variant_sweep` (Task 3).
- Produces these names, which the later tasks and tests rely on:
  - **Components and bus names, in this order:** `Operator_Console` (`operator`), `Operator_RC` (`rc`), `MCU`
    (`mcu`), `BMS` (`bms`), `CAN1_Net` (`can1`), `Plant` (`plant`), `Sensors` (`sensors`), `CAN2_Link` (`can2`),
    `GNC` (`gnc`), `Viz_Logging` (no output), `Sim_Monitors` (`monitors`).
  - **Component ports.** Each component has one In Bus Element port, `sys`, and one Out Bus Element port named
    after its bus. Its main block is named after its file: `MCU/Shaddad_MCU`, `BMS/Shaddad_BMS`,
    `Plant/Shaddad_Plant`, `Operator_RC/RC_Link_Sim`, `CAN1_Net/CAN1_Net_Sim`, `Sensors/Shaddad_SensorModels`,
    `CAN2_Link/Link_MCU_GNC`, `GNC/GNC_Variant`, `Viz_Logging/Viz_Logging`, `Sim_Monitors/Sim_Monitors`.
  - **`System_Bus`.** A virtual Bus Creator. Its input lines are named `scen` (from `Spec_ScenarioIn`), then the
    bus names in the order above. Its output line is `sys`.
  - **Pick blocks** are named `'sys_'` plus the element path with dots replaced by underscores (for example
    `sys_plant_elec_hw`). Out Bus Element blocks are named `<bus>_<signal>` (for example `plant_truth`).
  - **Published delayed signals:** `plant.elec_hw_d` from `Plant/ElecHw_Delay` (logged as `elec_hw`), and
    `bms.cmd_d` from `BMS/BMS_Cmd_Delay`.
  - **Where each helper goes:** as spec 5.2 says.
- **Logging:** the 20 rows keep their names, decimation and `Ts_signal`. Block paths:
  - `truth`, `drive_truth`, `power_truth`, `env`, `struct_diag` and `power_status`: `Plant/Shaddad_Plant`, the
    same port numbers as today;
  - `elec_hw`: `Plant/ElecHw_Delay` 1;
  - `mcu_out1k`, `mcu_out10` and `telem`: `MCU/Shaddad_MCU` 1, 3 and 6;
  - `rc_frame`: `Operator_RC/RC_Link_Sim` 2;
  - `bms_telem`: `BMS/Shaddad_BMS` 3;
  - `kpi`: `Sim_Monitors/Sim_Monitors` 1;
  - `nav_state`, `gnc_status`, `gnc_debug` and `nav_diag`: `GNC/GNC_Variant` 2, 4, 5 and 6;
  - `scen_in`: `Spec_ScenarioIn` 1;
  - `imu_raw` and `gnss_raw`: `Sensors/Shaddad_SensorModels` 1 and 2.

- [ ] **Step 1: Write the failing tests** (in `t_layout`, with the `t_infra` path edits):
  - `testTopStructure`:
    - the top level has exactly the root port `ScenarioIn`, `Spec_ScenarioIn`, `System_Bus` and the 11
      components;
    - each component has 1 inport and 1 outport, except `Viz_Logging`, which has no outport;
    - `System_Bus`'s input line names are exactly `scen` and the bus names in order.
  - `testLoggingPaths`: every `scn_logging('table')` row's block exists, and its port has `DataLogging` `'on'` with
    `DataLoggingName` equal to the row's name.
  - `testLoggingOverrideLong` (Review Focus 5): `mi = scn_logging('override', P, 2*P.log.to_file_threshold)`. Every
    `mi.Signals(k).BlockPath` resolves to an existing block with that port.
  - `testDashboardBindings` (Review Focus 4): every `simulink_hmi_blocks` block in
    `Shaddad_System/Operator_Console/Dashboard` has a Binding whose BlockPath exists, with ParamName `'Value'`.
- [ ] **Step 2: Run them and see them fail** on today's top model.
- [ ] **Step 3: Implement the top model** in `build_shaddad_system` (spec 5.1-5.4), using the mockup A layout:
  - the components in one column, in the table order;
  - each output straight into `System_Bus`;
  - `sys` returning below the column and up a trunk on the left.
  Use `shaddad_layout` for the inside of each component, and set the top level's positions explicitly or with
  `shaddad_layout`, whichever gives the mockup arrangement.
- [ ] **Step 4: Rebuild.** In batch: `build_shaddad_system()`.
- [ ] **Step 5: Run the tests and see them pass:** `t_layout` (top, plus the earlier tasks), `t_infra`,
  `t_framework`, the edited methods of `t_mcu_control` and `t_sensors`, and `testDomainLayout('top')`. The top's
  diagrams are the root and each component.
- [ ] **Step 6: Run the checks:**
  - `shaddad_check_interfaces('Compiled', true)`: 0;
  - `Simulink.BlockDiagram.getAlgebraicLoops('Shaddad_System')`: 0;
  - `shaddad_variant_sweep({'VC_OPSRC','VC_PHASE','VC_VIZ'})`: no new failures against
    `variant_sweep_before.csv`.
- [ ] **Step 7: Compare.** Run `shaddad_layout_runs('top')` (one process per case) against the baseline tag.
  Expected: `same == true` for all 4.
- [ ] **Step 8: Update the docs and text:** ICD 3.1, then `shaddad_icd_report`, then the bus descriptions and the
  `scn_ic_elec_hw` help line.
- [ ] **Step 9: Export images** of the top level and of each component's inside to `work/layout/top/`.
- [ ] **Step 10: CHECKPOINT** (controller). Send the user the top-level image, two component images and the
  Task 4 trial images. Wait for the go-ahead before Task 6.
- [ ] **Step 11: Record.**

### Domain recipe (Tasks 6 to 15)

Every domain task runs these steps on its own builders and files. Each task block below gives the specifics:
files, tests, variant controls and special notes.

- [ ] **R1: See the domain test fail.** `testDomainLayout` with the task's domain fails on today's files
  (crossings, unnamed lines or lines behind blocks above target).
- [ ] **R2: Snapshot** the builders and tests the task edits.
- [ ] **R3: Edit the builders** (spec 6 and 7).
  - **Placement.** Remove the domain's own placement: the creation-order grids and the `ARR`, `arrange`,
    `local_arrange` and `SLOT` calls to `arrangeSystem`. Every file is laid out by `shaddad_layout(<file>,
    'Recursive', true)` before it's saved, either through `shaddad_bu('save')` or by an explicit call in the
    builder's own save function.
  - **Picks** (spec 6.2). Inner files may get In Bus Element ports with picks. ICD-listed files and Model roots
    keep their classic ports, with a Bus Selector right in front of each consumer.
  - **Buses** (spec 6.3). When one block sends 3 or more signals to the same block, they travel as one virtual
    bus: a Bus Creator at the producer, or an Out Bus Element port of an inner file, with picks at the consumer.
  - **Combiners** (spec 6.3). A small block that only combines inputs for one consumer (a Mux or Sum) moves next to
    that consumer, or into it when the consumer is an inner file of the same builder.
  - **Other rules.** Name every signal line (L5), place helpers next to their port (L4), physical blocks in their
    band (L7), and variant choices stacked (L8).
  - **Computation stays the same.** Keep the same blocks, parameters and sample times, the same operand order in
    sums and products, and every Rate Transition and delay at its place in its path.
  - **Ports that change.** When an inner file's ports change, update its parent's builder and every bench and test
    that feeds it in the same task. Name the new ports in the task report.
- [ ] **R4: Rebuild** the domain's files in Build order, then its benches, in batch.
- [ ] **R5: Run the domain tests** listed in the task block. Expected: no failure that isn't already in
  `regression_baseline.txt`.
- [ ] **R6: Pass the domain test.** `testDomainLayout` passes for the domain. A crossing that can't be removed (a
  feedback loop or a physical network) gets a row in `shaddad_layout_allowance` with a concrete reason, which the
  reviewer checks.
- [ ] **R7: Run the checks.**
  - `shaddad_check_interfaces('Compiled', true)`: 0;
  - `getAlgebraicLoops`: 0 on `Shaddad_System` and on the domain's benches;
  - `shaddad_variant_sweep(<the task's controls>)`: no new failures.
- [ ] **R8: Compare.** Run `shaddad_layout_runs('<domain>')` (the quick set, one process per case) against the
  baseline tag. Expected: `same` for every case. A difference stops the task; the controller takes it to the user.
- [ ] **R9: Export images** of the domain's main diagrams to `work/layout/<domain>/`, with `print('-s<sys>',
  '-dpng')`.
- [ ] **R10: Record.**

### Task 6: Power

**Files:**
- Modify these builders:
  - `scripts/build/build_power_pack.m`;
  - `scripts/build/build_power_plant.m`: drop the `A()` creation-order grid and `ARR`, and keep `local_netcheck`
    before every save;
  - `scripts/build/build_tb_pack_bms.m`;
  - `scripts/build/build_tb_power_plant.m`: drop its `A()` grid.
- Regenerate the 18 files of `build_power_pack` (4) and `build_power_plant` (14), and the benches `tb_pack_bms`,
  `tb_power_plant`, `tb_shift_cycle`, `tb_lv_rails` and `tb_impl_codec`.
- Check but don't rebuild the two battery libraries, `Shaddad_Battery_lib` and `Shaddad_Battery_Lumped_lib`. Their
  builder `build_shaddad_battery` is not rerun, because they already have 0 crossings. The domain counts 25 files.
- Tests:
  - `tests/power/t_power_plant.m`;
  - `tests/power/t_pack_bms.m`: edit it where it feeds `Power_Sensors`;
  - `tests/electronics/t_mcu_platform.m`: its CAN1 round trip uses `implcmd_can_unpack` and `implfb_can_pack`.

**Interfaces:**
- Produces: `Power_Sensors` takes one bus per source block, as in mockup C. The sources are `pack`, `pdu`, `lv`,
  `safety`, `thermal`, `impl` and `charger`, plus picks from `env` and `flt`. Its parent `Power_Plant` and the bench
  `tb_pack_bms` change with it.
- Keeps: the ICD 3.2 ports of `Power_Plant`, and the ports of `implcmd_can_pack` and `implfb_can_unpack`.

**Specifics:**
- Variant controls: `VC_IMPL`, `VC_PACK`, `VC_CHG`, `VC_IMPL_DRIVE`.
- `t_power_plant` counts the MATLAB Function, Solver Configuration and Electrical Reference blocks over all
  variants. These counts don't change.
- Build order: `build_power_pack`, then `build_power_plant`, then `build_tb_pack_bms` and `build_tb_power_plant`.
- Scale: this domain held 17,941 of the 54,221 crossings: `tb_power_plant` and `tb_shift_cycle` 4,331 each,
  `PDU` 1,428, `LV_Distribution` 1,245 and `Power_Plant` 1,113.

### Task 7: Drive

**Files:**
- Modify these builders:
  - `scripts/build/build_drive.m`:
    - drop `grid_place` and its persistent counter, which makes the layout depend on what ran before;
    - drop the `arrange()` calls on the `drive_esc` subsystems;
    - make `finish()` call `shaddad_layout(<file>, 'Recursive', true)` before it saves.
  - `tests/drive/build_tb_drive.m`: drop its own `A()` grid. Note that `build_drive('side_foc_body')` fills
    `tb_drive_foc/Drive`.
- Regenerate the 13 drive files and the 4 drive benches.
- Tests:
  - `tests/drive/t_drive.m`: the port lists of `drive_plant_sys` and `drive_plant_ideal` must match;
  - `t_infra` (`Shaddad_Plant/Drive`);
  - `t_mcu_platform` and `t_mcu_control`, through the codecs.

**Interfaces:**
- Keeps: the ICD 3.2 ports of `drive_system`; the ports of `drive_cmd_pack` and `drive_fb_unpack`; and the
  identical ports of the `VC_DRIVE_FID` choices (rule 6).

**Specifics:**
- Variant controls: `VC_DRIVE_FID`.
- Build order: `build_drive()` (in its own part order), then `build_tb_drive()`.

### Task 8: Structure

**Files:**
- Modify: `scripts/build/build_structure_env.m`, `scripts/build/build_structure.m`,
  `scripts/build/build_tb_structure.m`, `scripts/build/build_tb_structure_env.m`.
- Regenerate `structure_env`, the 13 `structure_*` files, `tb_structure` and `tb_structure_env`.
- Tests:
  - `tests/structure/t_structure.m`: the paths `structure_contact/MF_A_trackContactNormal` and
    `/MF_B_trackPlanarForces`, and the interface check of `structure_plant`;
  - `t_world`: the inports of `structure_env` (names, order, types, `SampleTime`, `PortDimensions`), and no MATLAB
    Function or Simscape blocks in it;
  - `t_framework`;
  - `t_infra`.

**Interfaces:**
- Keeps: the ports of `structure_plant` (ICD 3.2) and `structure_env`, and the identical ports of the variant
  choices.

**Specifics:**
- Variant controls: `VC_STRUCT_FID`, `VC_IMPL`.
- `arrangeSystem` has laid out these physical diagrams until now. The engine asserts every physical node.
- Build order: `build_structure_env`, then `build_structure`, then the two benches.

### Task 9: Electronics plant and the plant model root

**Files:**
- Modify these builders:
  - `scripts/build/build_electronics_plant.m`: the `SLOT()` calls to `arrangeSystem`, the `FrameIO` variant and the
    ROS 2 dispatch subsystems;
  - `scripts/build/build_tb_elec_plant.m`: `ARR`;
  - `scripts/build/build_shaddad_plant.m`: the `Plant_In` call to `arrangeSystem`, and the physical lines of the
    root.
- Regenerate the 6 `elp_*` files, `Electronics_Plant`, `CAN1_Net_Sim`, `Link_MCU_GNC`, `companion_io_ros2`,
  `companion_io`, `tb_elec_plant` and `Shaddad_Plant`.
- Tests:
  - `tests/electronics/t_elec_plant.m`: the paths `companion_io_ros2/Sel_msg`, `/Sub_can2_rx` and
    `/Pub_can2_tx/Publish`, and the root block `FrameIO` in `companion_io`;
  - `t_infra`, for `Shaddad_Plant`:
    - exactly one block named `Solver Configuration`, connected to Drive or Power;
    - the domain references;
    - the `ZOH_*` sample times;
    - `Plant_In` output 3;
  - `t_targets`.

**Interfaces:**
- Keeps: the ICD ports of `Electronics_Plant`, `CAN1_Net_Sim`, `Link_MCU_GNC`, `companion_io` and
  `Shaddad_Plant`. `run_scenario` finds the Solver Configuration blocks under `Shaddad_Plant/Electronics/` for the
  D58 `'ref'` solver, so their names and paths stay.

**Specifics:**
- Variant controls: `VC_TARGET`, `VC_IMPL`.
- `elp_thermal` is a Simscape network with its own Solver Configuration.
- Plant root (spec 6.4):
  - the inputs feed Bus Selector picks in front of `Plant_In`, `Structure`, `Power`, `Drive` and `Electronics`;
  - the physical band holds the DC48 lines, the springs, the implement shaft and the Solver Configuration.
- Build order: `build_electronics_plant`, then `build_tb_elec_plant`, then `build_shaddad_plant` (after Tasks 6-8).

### Task 10: Sensor models

**Files:**
- Modify: `scripts/build/build_sensor_models.m` (the `arrangeSystem` calls on the inline `GNSS` and `LiDAR`
  subsystems), `scripts/build/build_tb_sens_plant.m`.
- Regenerate `sens_imu_model`, `sens_gnss_model`, `sens_lidar_model`, `sens_spray_vision`,
  `Shaddad_SensorModels`, `sens_acq_imu`, `sens_odom_compute` and `tb_sens_plant`.
- Tests:
  - `tests/sensors/t_sensors.m`:
    - the inline `GNSS` and `LiDAR` must have the same relative block paths and types as `sens_gnss_model` and
      `sens_lidar_model`;
    - `tb_sens_plant`: the blocks `/Sensors`, `/AcqIMU`, `/Odom` and the `o_*` outports;
  - `t_nav`, `t_framework`, `t_mcu_platform`.

**Interfaces:**
- Keeps: the ICD ports of `Shaddad_SensorModels`, and the ports of `sens_acq_imu` and `sens_odom_compute`.

**Specifics:**
- Changes to the GNSS and LiDAR models go into the local functions they share with the inline copies, so the
  copies stay identical.
- Variant controls: `VC_LIDAR`.
- The builder creates the base variable `sens_uavScene`.

### Task 11: Navigation and perception models

**Files:**
- Modify these builders:
  - `scripts/build/build_navfusion.m`: its 9 inner subsystems were never laid out;
  - `scripts/build/build_perception.m`: `ARR`;
  - `scripts/build/build_tb_nav.m`;
  - `scripts/build/build_tb_perception.m`.
- Regenerate `sens_acq_gnss`, `Shaddad_NavFusion`, `sens_scan_filter`, `sens_acq_lidar`, `Shaddad_Perception`,
  `tb_nav`, `tb_perception`, `tb_per_scanfilter`, `tb_per_tebvfh` and `tb_per_tebvfh_top`.
- Tests:
  - `tests/sensors/t_nav.m`:
    - the outports `nav_state` and `nav_diag`;
    - `NavMeas/LLA2Flat`, `NavOutput/Flat2LLA` and `AttitudeFilter/NegAcc`;
    - `NavEKF` at depth 1;
    - the MATLAB Functions at depth 1 are exactly `NavGate` and `NavModeManager`;
    - the block names in `IO_In/HW` equal those of `sens_acq_gnss`;
  - `tests/sensors/t_perception.m`: the outports, and the set of MATLAB Function names.

**Interfaces:**
- Keeps: the ICD ports of `Shaddad_NavFusion` and `Shaddad_Perception`.

**Specifics:**
- `IO_In/HW` is an inline copy of `sens_acq_gnss` built by the same functions, so the two change together.
- Build `tb_per_tebvfh` before `tb_per_tebvfh_top`, which references it as a Model.
- Variant controls: `VC_TEB`, `VC_TARGET`.
- Scale: this domain held 12,506 crossings and 3,234 lines behind blocks; `NavMeas` alone had 3,825.

### Task 12: RC link

**Files:**
- Modify: `scripts/build/build_rc_link.m` (`local_save`, `local_finalize`, and the inline `RC_Link_Sim/UART_Line_Sim`).
- Regenerate `rc_transmitter`, `rc_radio_channel`, `rc_receiver`, `rc_sbus_encoder`, `rc_uart_line_sim`,
  `RC_Link_Sim`, and the benches `tb_rc_link`, `tb_rc_sbus_encoder`, `tb_rc_link_speed` and `tb_rc_uart_line`.
- Tests:
  - `tests/rc/t_rc_link.m`:
    - the path and ports of `tb_rc_link/RC_Link`;
    - the inline `UART_Line_Sim` must equal `rc_uart_line_sim.slx` (relative paths, block types, dialog
      parameters);
  - `t_mcu_platform`, `t_framework`, `t_infra`.

**Interfaces:**
- Keeps: the ICD ports of `RC_Link_Sim`.

**Specifics:**
- Close every model that references the RC files before building; the builder refuses otherwise.
- The inline UART copy gets exactly the changes of `rc_uart_line_sim`.
- Rate rule D32: 0.014 s and 0.007 s exist only inside `RC_Link_Sim`.

### Task 13: GNC and the companion model root

**Files:**
- Modify: `scripts/build/build_shaddad_gnc.m`, `scripts/build/build_gnc_mission.m`, `scripts/build/build_tb_gnc.m`,
  `scripts/build/build_shaddad_companion.m`.
- Regenerate the 11 `gnc_*` files, `Shaddad_GNC`, `tb_gnc_mission`, `tb_gnc` and `Shaddad_Companion`.
- Tests:
  - `tests/gnc/t_gnc_tracking.m`: `tb_gnc/GNC` has 6 inputs and 4 outputs, and all 11 files keep the port contract
    from `build_shaddad_gnc('ports')`;
  - `tests/gnc/t_gnc_mission.m`: `gnc_MissionManager/MissionFSM` and `gnc_GlobalReplanner/Replan/HybridAStar`;
  - `t_infra`: the companion's `NavState_Delay`, and `GNC` has 6 inputs;
  - `t_targets`.

**Interfaces:**
- Keeps: the port contract of the 11 `gnc_*` files, and the ICD ports of `Shaddad_GNC` and `Shaddad_Companion`.

**Specifics:**
- `build_shaddad_gnc` writes stubs only for files whose Description starts with `SHADDAD_GNC_STUB`, so never
  write that prefix.
- Variant controls: `VC_ROW_TRACKER`, `VC_TEB`, `VC_PHASE`.
- The companion root follows spec 6.4.
- Build order: `build_shaddad_gnc`, then `build_gnc_mission`, then `build_tb_gnc`, then `build_shaddad_companion`
  (after Tasks 9 and 11).

### Task 14: BMS, monitors and visualisation

**Files:**
- Modify: `scripts/build/build_shaddad_bms.m` (`ARR` on every subsystem), `scripts/build/build_sim_monitors.m`,
  `scripts/build/build_viz_logging.m` (`local_arrange`), `scripts/build/build_tb_sim_monitors.m`,
  `scripts/build/build_tb_frames.m`.
- Regenerate `bmsreq_can_pack`, `bmsreq_can_unpack`, `bms_can_pack`, `bms_can_unpack`, `Shaddad_BMS`,
  `Sim_Monitors`, `Viz_Logging`, `tb_sim_monitors` and `tb_frames`.
- Tests:
  - `tests/power/t_pack_bms.m`: `Shaddad_BMS/BMS_Core` and the codec harnesses;
  - `t_targets`: `Shaddad_BMS/IO_In/HW/{rx_data,Frame_BC,DI_bits,DI_u16}`;
  - `t_framework`;
  - `t_row_driver`: it matches the message text `Sim_Monitors/NaN_Assertion`.

**Interfaces:**
- Keeps:
  - the ICD ports of `Shaddad_BMS`, `Sim_Monitors` and `Viz_Logging`;
  - the ports of `bmsreq_can_pack` and `bms_can_unpack`;
  - the paths `Sim_Monitors/{Min_cyl,Min_box,Min_dyn}`, which `scn_logging` logs;
  - the paths that `build_targets('peripherals')` reads: `IO_In/HW/{AFE_SPI,ADC,DI,FDCAN1_Read}` and
    `IO_Out/HW/DO_contactors`.

**Specifics:**
- `build_shaddad_bms` reads `P.power.pack.Ns`, `di_n` and `do_n`, and builds `'codecs'` before `'model'`.
- Variant controls: `VC_TARGET`, `VC_VIZ`.

### Task 15: MCU firmware, after the derate review's changes have landed

**Files:**
- Modify these builders:
  - `scripts/build/build_can2_codecs.m`;
  - `scripts/build/build_fw_platform.m`: the T08 units only, and its T09 stubs are never written again;
  - `scripts/build/build_fw_control.m`;
  - `scripts/build/build_shaddad_mcu.m`: `ARR` on the tasks, and the IO_In and IO_Out shells;
  - `scripts/build/build_tb_mcu_rc.m`;
  - `scripts/build/build_tb_mcu_closed_loop.m`.
- Regenerate, in the MCU order: the 10 `can2_*` codecs, the 11 T08 units, the 5 T09 units and `Shaddad_MCU`; then
  `tb_mcu_rc` and `tb_mcu_closed_loop`.
- Tests:
  - `tests/electronics/t_mcu_platform.m`:
    - task atomicity and rates;
    - the IO SIM and HW choices;
    - more than 20 Rate Transitions;
    - exactly one MATLAB Function, `Frame_Sync`;
    - about 18 HW block paths in `fw_io_in_hw` and `fw_io_out_hw`;
    - no algebraic loops.
  - `tests/electronics/t_mcu_control.m`:
    - the T09 port contract;
    - `fw_mode_manager/Mode_FSM`;
    - `Shaddad_MCU/Task_10/{Safety_Supervisor,Mode_Manager,Vehicle_Ctrl}`, `Task_50/Implement_Aux` and
      `Task_1k/Watchdog_Mgr`;
    - run once with `T09_LONG=1` for `testRebuildIdempotent`.
  - `t_elec_plant`, `t_sensors`, `t_targets`, `t_infra`.

**Interfaces:**
- Keeps:
  - the MCU root ports and the generated-code interface;
  - the tasks and their rates;
  - the T09 port contract;
  - the ports of the 13 shared files;
  - the port numbers the MCU's own logging uses: `Task_1k` out 1, the `busoff` output of `IO_In`, `Task_5` out 2,
    `Task_10/RC_Mapping` out 1;
  - the HW block paths that `build_targets` reads.

**Specifics:**
- **Gate.** Start only after the derate review's changes to `build_fw_control.m` have landed. Then repeat spec 9's
  rebuild-first for the MCU parts: the MCU order with today's builders and no layout change, then compare. A
  difference goes to the user before any layout change (CHECKPOINT). The result becomes the MCU baseline.
- New buses inside the units are virtual (spec 6.4), so the generated code computes the same thing.
- Variant controls: `VC_TARGET`.

### Task 16: Docs, final checks and the before/after report

**Files:**
- Modify the docs:
  - `docs/build_plan.md`: rule 3 names `shaddad_layout` instead of `arrangeSystem` and `build_all` as the way to
    rebuild, with one paragraph on picks and bus element ports;
  - `docs/architecture.md`: D32 c and D36, for the top-level blocks that moved;
  - `docs/STATUS.md`: the model map (the top-level components) and how to rebuild;
  - `docs/icd.md`: regenerated.
- Create: `work/layout/report.md`, `work/layout/inventory_after.csv`, `work/layout/final/*.png`.

- [ ] **Step 1: Check every diagram.** Run `shaddad_layout_check('all', 'Csv', 'work/layout/inventory_after.csv')`.
  Expected: every diagram passes the targets, and total crossings are at most 5,422 (90 % below 54,221).
- [ ] **Step 2: Run the layout tests.** Every `t_layout` test passes (tag `Layout`).
- [ ] **Step 3: Sweep every variant.** `shaddad_variant_sweep()` over all controls: no new failures against
  `variant_sweep_before.csv`.
- [ ] **Step 4: Check interfaces and loops.** `shaddad_check_interfaces('Compiled', true)`: 0. Algebraic loops: 0.
- [ ] **Step 5: Run the suites.** `run_regression('Fast')`: no new failures against `regression_baseline.txt`.
  `run_phase1_tests('Quick')`: the same verdict as before the work.
- [ ] **Step 6: Compare the full set.** Run `shaddad_layout_runs('final', 'full')` against the baseline tag.
  Expected: `same` for all 6 cases.
- [ ] **Step 7: Update the docs** listed under Files.
- [ ] **Step 8: Write the report.**
  - A before/after table per domain: files, diagrams, crossings, lines behind blocks, unnamed lines.
  - The allowance list, with reasons.
  - Before and after images of the top level, `Shaddad_Plant`, `Power_Plant`, `PDU`, `Shaddad_MCU/Task_10` and
    `Shaddad_NavFusion/NavMeas`.
- [ ] **Step 9: Record.**

## Build order

`build_all` runs the builders in this order, dependencies first; each builder keeps its own internal order.

`'NoMCU'`:
1. `build_drive`, `build_shaddad_bms`, `build_rc_link` (with its benches), `build_sensor_models`, `build_navfusion`,
   `build_perception`, `build_structure_env`, `build_sim_monitors`, `build_viz_logging`, `build_shaddad_gnc`.
2. `build_power_pack`, `build_structure`, `build_electronics_plant`, `build_gnc_mission` (with `tb_gnc_mission`).
3. `build_power_plant`.
4. `build_shaddad_plant`, `build_shaddad_companion`.
5. The benches: `build_tb_pack_bms`, `build_tb_power_plant`, `build_tb_drive`, `build_tb_structure`,
   `build_tb_structure_env`, `build_tb_elec_plant`, `build_tb_sens_plant`, `build_tb_nav`, `build_tb_perception`,
   `build_tb_gnc`, `build_tb_sim_monitors`, `build_tb_frames`.

`'MCU'`:
1. `build_can2_codecs`
2. `build_fw_platform`, given the 11 T08 names
3. `build_fw_control`
4. `build_shaddad_mcu('model')`
5. `build_tb_mcu_rc`
6. `build_tb_mcu_closed_loop`

`'All'`: `'NoMCU'`, then `'MCU'`, then `build_shaddad_system`.

Notes:
- `build_all` first closes every loaded model without saving. It runs in a batch MATLAB, and `build_rc_link` refuses
  to rebuild a file that a loaded model references.
- `build_shaddad_battery` is not part of it. Its generated libraries are already clean, and it rewrites parameter
  files.
- `build_stubs` and `build_project` don't build models.
- `build_electronics_plant` uses the CAN2 codecs, which `'NoMCU'` doesn't rebuild; they already exist.
