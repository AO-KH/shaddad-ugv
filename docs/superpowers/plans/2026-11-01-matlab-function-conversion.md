# MATLAB Function Conversion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Start condition:** don't start any task until the layout plan
(`docs/superpowers/plans/2026-10-29-model-layout-cleanup.md`, Tasks 11-16 and its final review) is finished and the
user says to start this plan. Task 4 has its own gate.

**Goal:** Replace the logic-only block networks of seven subsystems with one MATLAB Function each, removing about 250
blocks from the diagrams, while every output stays bit-identical.

**Architecture:** Each converted subsystem keeps its ports. Inside, one MATLAB Function block calls a `%#codegen`
function in `subsystems/<domain>/<name>.m` in one line. Lookup tables, delays, integrators, rate transitions and For Each
partitioning stay as blocks around it. A new tool, `utilities/mf_equiv.m`, runs the snapshot (old) and the rebuilt
(new) subsystem side by side on the same inputs and requires bit-identical outputs. The system compare
(`shaddad_layout_runs` with `shaddad_compare_logs`) checks the whole model.

**Tech Stack:** MATLAB R2026b, Simulink, the project's builders (`scripts/build/build_*.m`), `matlab.unittest`.

**Spec:** none written. The decisions below come from the 2026-11-01 conversation:
- an inventory of every subsystem level of the 153 `.slx` files;
- the project's blocks-first rule (`docs/architecture.md` section 11, D59, `docs/build_plan.md` rule 4);
- the user's choice to convert these candidates.

Rulings made without a spec are provisional.

## Decisions (provisional)

1. **Which subsystems.**

   | Subsystem | Blocks / connections today | Runs in | Function |
   |---|---|---|---|
   | `Sim_Monitors/FE_Box` | 73 / 96 | simulation only | `mon_sep_rect` |
   | `Sim_Monitors/FE_Cyl` | 26 / 28 | simulation only | `mon_sep_circle` |
   | `Sim_Monitors/FE_Dyn` | 26 / 28 | simulation only | `mon_sep_circle` |
   | `drive_esc/Feedback` | 65 / 68 | simulation only | `esc_feedback` |
   | `drive_esc/Torque_Limiter` | 50 / 67 | simulation only | `esc_torque_limit` |
   | `gnc_ImplementManager` | 50 / 70 | code-generated (Companion) | `gnc_implementManager` |
   | `gnc_PathServer` | 43 / 63 | code-generated (Companion) | `gnc_pathServer` |

   Everything else keeps blocks first.
2. **The rule.** A new decision, D65 in `docs/architecture.md`, records this exception, and section 11's register
   lists the six functions. Build plan rule 4 points to D65.
3. **Bit-identical.** Every output stays bit-identical. If a conversion can't, stop and ask the user; never loosen the
   check.
4. **Known bugs stay.** `t_framework/testMonitorBoxOracle` fails today: a check expects 0.44 and gets 0.6. The
   conversion keeps that output. Fixing it is a separate behaviour change for the user to approve.
5. **After the layout plan.** The layout plan's acceptance needs bit-identical runs and no computational change.
   Builders save through `shaddad_bu('save')`, which lays each file out again, so no layout rework is lost.

## Global Constraints

- **MATLAB Functions.**
  - A MATLAB Function block only for the six functions above. Its body lives in a `%#codegen` `.m` file next to the
    model (`subsystems/<domain>/`) and is called in one line (build plan rule 4).
  - Parameters come in as non-tunable sub-structs of `P` (for example `P.drive.esc`), never the whole `P` (D58).
- **Builds.** The builder regenerates every `.slx` and saves it through `shaddad_bu('save')`. Never hand-edit a model.
- **Interfaces.** Ports stay the same: names, order, data types and sample times. `shaddad_check_interfaces('Compiled',
  true)` reports 0.
- **What stays as blocks.** Lookup Tables, Unit Delays, Discrete-Time Integrators, Rate Transitions and the For Each
  partitioning. The MATLAB Function takes only the combinational logic between them.
- **Integer casts.** Each cast in a function reproduces its Data Type Conversion block's rounding mode (`RndMeth`) and
  overflow setting (`SaturateOnIntegerOverflow`). MATLAB's own casts round to nearest and saturate.
- **MATLAB processes.**
  - Run MATLAB only as separate batch processes (`matlab.exe -batch`), one at a time.
  - Never use the user's desktop MATLAB or the matlab MCP tools.
  - Never run `build_shaddad_mcu('all')` or `build_shaddad_mcu()`, never call `build_fw_platform` without unit names,
    and run no MCU or BMS build.
- **No git.** Before editing, copy each file a task changes to
  `.superpowers/sdd/2026-11-01-matlab-function-conversion/snap/task-N/` and check that the copy is equal (the layout
  plan's Ruling I). The snapshot `.slx` is also the old side of `mf_equiv`.
- **Hands off.** Don't edit `docs/report/**`. Subagents use opus or haiku only.

## Review Focus

1. **Touching or empty rows.** A monitor row that just touches the footprint (sep = 0), and a zero-size row: the sign
   of `sep` and the collision flag must match the old blocks.
2. **Zeros and non-finite inputs.** `v_dc` = 0, `w_meas` = 0 and NaN from a failed sensor: the function returns exactly
   what the blocks did (Inf, NaN, or the block's saturation).
3. **Rounding and overflow in casts** (Global Constraints). `Feedback` alone has 22 Data Type Conversions.
4. **Execution order around kept states.** No new algebraic loop, and `Torque_Limiter`'s Unit Delay and integrator see
   the same input at the same step.
5. **Code generation of the GNC functions.** Fixed sizes (N_raw = 100 rows), no dynamic memory and no unsupported calls.

---

### Task 1: The rule, the equivalence tool and the baseline

**Files:**
- Create: `utilities/mf_equiv.m`, `tests/integration/t_mf_equiv.m`
- Modify: `docs/architecture.md`:
  - the decisions table: add D65;
  - section 11's register: add six rows;
  - the sentence after the register.
- Modify: `docs/build_plan.md` rule 4 (one clause)

**Interfaces:**
- Produces `r = mf_equiv(oldSlx, newSlx, sub, U)`:
  - `oldSlx`, `newSlx`: the paths of the snapshot file and the rebuilt file;
  - `sub`: the subsystem path inside the file, without the model name (`'Feedback'`), or `''` for a whole Subsystem
    Reference file;
  - `U`: a `Simulink.SimulationData.Dataset` with one element per inport, in port order (bus inports as structures of
    timeseries), on the subsystem's own sample time;
  - `r.same`: logical, `isequal` over every sample of every output;
  - `r.nSamples`: the number of samples compared;
  - `r.firstDiff`: char naming the output, time, old value and new value; `''` when same.
- Approach:
  1. Copy `oldSlx` to `tempdir` under a unique name.
  2. Build a harness model in memory holding both versions side by side: copied blocks for a `sub`, Subsystem Reference
     blocks for `''`.
  3. Feed both from `U` and log the outputs.
  4. Close everything without saving.

- [ ] **Step 1:** Write `t_mf_equiv` (tag Fast), with two tests on a temporary model that holds a 3-block subsystem:
  - `testSameCopies` compares the subsystem with its own copy: `r.same` is true and `r.nSamples` is 101;
  - `testOneGainChanged` changes the copy's Gain from 2 to 2.0000001: `r.same` is false, and `r.firstDiff` names
    output 1 at t = 0.
- [ ] **Step 2:** Run it in batch with `runtests('tests/integration/t_mf_equiv.m')`. Expected: FAIL (`mf_equiv`
  undefined).
- [ ] **Step 3:** Implement `mf_equiv` as Interfaces says.
- [ ] **Step 4:** Run `t_mf_equiv` again. Expected: 2 passed.
- [ ] **Step 5:** Update the rule in the docs:
  - add D65 to `docs/architecture.md`;
  - add the six section-11 rows, each with the reason "pure single-rate logic: one function reads clearer than N
    blocks";
  - end the sentence "Everything else ... uses library blocks." with ", except the D65 rows above";
  - add "(exceptions: D65)" to build plan rule 4.
- [ ] **Step 6:** Record the baseline.
  - Run `shaddad_layout_runs('pre_mf', 'full')` (S0, S1, S8a, R1, S2, S6c), one case per batch process.
  - Save before-run logs of four test files: `t_framework` (tags Fast and Integration, without
    `testBuildIdempotent`), `t_drive`, `t_gnc_mission` and `t_gnc_tracking`.
- [ ] **Step 7:** Snapshot the new and changed files (this task changes no `.slx`).

### Task 2: Collision monitors (Sim_Monitors)

**Files:**
- Create: `subsystems/integration/mon_sep_circle.m`, `subsystems/integration/mon_sep_rect.m`
- Modify: `scripts/build/build_sim_monitors.m` (`local_foreach`, its `'circle'` and `'rect'` cases)
- Regenerate: `subsystems/integration/Sim_Monitors.slx` (`build_sim_monitors()`)
- Test: `tests/integration/t_mf_equiv.m` (two tests added); `tests/integration/t_framework.m` (unchanged, its monitor
  oracles)

**Interfaces:**
- Produces `sep = mon_sep_circle(row, p, R)` and `sep = mon_sep_rect(row, p, R)`:
  - `row`: one obstacle row (3 columns for `FE_Cyl` and `FE_Dyn`, 5 for `FE_Box`);
  - `p`: the footprint centre (2x1);
  - `R`: the footprint rotation (2x2, body to world);
  - `sep`: the separation in m, negative when overlapping, computed in the same expression order as today's blocks.
- Each `FE_*` keeps its For Each Subsystem and its ports `row`, `p`, `R` -> `sep`, with one MATLAB Function `Sep`
  between them.

- [ ] **Step 1:** Add `testMonitorsUseFunctions` to `t_mf_equiv`. It checks that each of `Sim_Monitors/FE_Cyl`,
  `FE_Box` and `FE_Dyn` holds exactly one MATLAB Function block, whose script calls `mon_sep_circle(` (Cyl, Dyn) or
  `mon_sep_rect(` (Box). Run it. Expected: FAIL.
- [ ] **Step 2:** Add `testMonitorsEquivalent`. It calls `mf_equiv(<snap>/Sim_Monitors.slx,
  'subsystems/integration/Sim_Monitors.slx', 'FE_Box', U)`, and the same for `FE_Cyl` and `FE_Dyn`.
  - Inputs: 500 samples of random rows with a fixed seed (x and y within ±20 m, sizes 0.2-3 m, yaw ±pi), plus the edge
    rows of Review Focus 1.
  - Expected: `r.same` true.
  - It passes against itself before the change, and guards the rebuild.
- [ ] **Step 3:** Snapshot. Write the two functions. Change `local_foreach` to add the MATLAB Function and drop the
  block network.
- [ ] **Step 4:** Rebuild with `build_sim_monitors()` in batch.
- [ ] **Step 5:** Run the tests.
  - `t_mf_equiv`: all pass.
  - `t_framework` (Fast and Integration, without `testBuildIdempotent`): the same results as Task 1's before run,
    with `testMonitorBoxOracle` still failing at 0.6 against 0.44.
- [ ] **Step 6:** Compare the whole model, and check the layout.
  - Run `shaddad_layout_runs('mf_mon', 'full')`, and compare each case with `shaddad_compare_logs` against `pre_mf`.
    Expected: `same` for all six.
  - `testDomainLayout(domain='bms_int')` passes. Lower the allowance rows of the changed diagrams.

### Task 3: drive_esc Feedback and Torque_Limiter

**Files:**
- Create: `subsystems/drive/esc_feedback.m`, `subsystems/drive/esc_torque_limit.m`
- Modify: `scripts/build/build_drive.m`:
  - the `Feedback` block network, from line 858;
  - the `Torque_Limiter` network, from line 602.
- Regenerate: `subsystems/drive/drive_esc.slx`, with `build_drive()`. Use its option to build one file if it has one;
  otherwise the other drive files are regenerated unchanged.
- Test: `tests/integration/t_mf_equiv.m` (two tests added); `tests/drive/t_drive.m` (unchanged)

**Interfaces:**
- Produces `fb = esc_feedback(mode, state, brake_released, fault_word, derating, lim_active, sto_ok, stall_warn,
  w_meas, enc_cnt, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc)`:
  - `fb` is typed `Bus: Bus_DriveSideFb`;
  - its inputs come from the In Bus Element picks (`cmd_in.mode`, `sm.state`, `sm.brake_released`, `sm.fault_word`)
    and the 13 scalar inports;
  - the output port `fb` is unchanged;
  - the MATLAB Function block is `Feedback/FbPack`.
- Produces `esc_torque_limit(...)`, in the block `Torque_Limiter/LimitLogic`: the combinational part of
  `Torque_Limiter`.
  - Its two Lookup Tables (`EnvPeak` and the second one), the Unit Delay and the Discrete-Time Integrator stay blocks,
    wired to the function's inputs and outputs. The exact signature follows from these kept blocks.
  - Unchanged: the ports `cmd_in` (`tau_lim_reg`, `tau_lim_mot`), `w_meas`, `v_dc`, `T_mot_meas`, `T_esc` and
    `tau_applied` -> `up`, `lo`, `tau_avail` and `derating`, and the sample time `P.drive.Ts_esc`.

- [ ] **Step 1:** Add `testDriveEscUsesFunctions`. It checks that `Feedback` holds one MATLAB Function calling
  `esc_feedback(`, and `Torque_Limiter` one calling `esc_torque_limit(`. Expected: FAIL.
- [ ] **Step 2:** Add `testDriveEscEquivalent`: `mf_equiv` on `'Feedback'` and `'Torque_Limiter'`.
  - Inputs: 2000 random samples over each input's range, covering:
    - every `sm.state` value from 0 to 8, and both `mode` values;
    - `v_dc` = 0, and a NaN in one sensor input (Review Focus 2);
    - values at x.5 and past the integer limits of every Data Type Conversion (Review Focus 3);
    - temperatures past the derating knees.
  - Expected: `r.same` true.
- [ ] **Step 3:** Snapshot. Write the two functions, with casts as Global Constraints says. Change the two builder
  sections.
- [ ] **Step 4:** Rebuild in batch.
- [ ] **Step 5:** Run `t_mf_equiv`: all pass. Run `t_drive`: the same results as Task 1's before run.
- [ ] **Step 6:** Compare the whole model, and check the layout.
  - Run `shaddad_layout_runs('mf_drive', 'full')` and compare against `pre_mf`. Expected: `same` for all six.
  - `testDomainLayout(domain='drive')` passes, with lowered rows.

### Task 4: GNC managers (gated)

**Gate.** The user confirmed on 2026-11-04 that these two code-generated parts stay in scope. One condition is left:
start only when the Companion code build, `build_targets('Companion')`, passes. When this plan was written it failed in
`Shaddad_GNC/GlobalReplanner/Replan/HybridAStar`; fixing that is separate work. Until the build passes, this task waits.
Tasks 1-3 and 5 don't depend on it; if it runs after Task 5, repeat Task 5's checks for it.

**Files:**
- Create: `subsystems/gnc/gnc_implementManager.m`, `subsystems/gnc/gnc_pathServer.m`
- Modify: `scripts/build/build_gnc_mission.m` (`local_gnc_ImplementManager`, line 577) and
  `scripts/build/build_shaddad_gnc.m` (`local_gnc_PathServer`, line 490)
- Regenerate: `subsystems/gnc/gnc_ImplementManager.slx`, `subsystems/gnc/gnc_PathServer.slx`. Use each builder's
  option to build one file if it has one.
- Modify: `tests/gnc/t_gnc_mission.m`: add the two functions to its list of MATLAB Function blocks (line 62).
- Test: `t_mf_equiv` (two tests added), `t_gnc_mission`, `t_gnc_tracking`

**Interfaces:**
- Produces `impl = gnc_implementManager(fsm, pe, nav, mcu, spray, plan, obs, prm)`:
  - the seven bus inputs arrive as structures;
  - `impl` is typed `Bus: Bus_GncCmd`;
  - `prm` is the non-tunable sub-struct of `P.gnc` that the function reads (D58);
  - the sample time is `Ts.gnc`.
- Produces `win = gnc_pathServer(fsm, plan, rp, prm)`, with `win` typed `Bus: Bus_PathWindow` and a fixed size of
  N_raw = 100 rows.
- The MATLAB Function blocks are `gnc_ImplementManager/ImplLogic` and `gnc_PathServer/WindowLogic`.
- Both Subsystem Reference files keep their ports, so `Shaddad_GNC` and `build_shaddad_gnc('ports')` see no change.

- [ ] **Step 1:** Add `testGncUsesFunctions`. In `t_gnc_mission`'s list (line 62), add the block paths
  `gnc_ImplementManager/ImplLogic` and `gnc_PathServer/WindowLogic`, and add `gnc_implementManager` and
  `gnc_pathServer` to the names it accepts. Expected: FAIL (the blocks don't exist yet).
- [ ] **Step 2:** Add `testGncEquivalent`: `mf_equiv(<snap>/gnc_ImplementManager.slx, <new>, '', U)`, and the same
  for `gnc_PathServer`.
  - Inputs: 300 random samples of the bus inputs, plus the FSM states that `gnc_fsm_driver` produces.
  - Expected: `r.same` true.
- [ ] **Step 3:** Snapshot. Write the functions (`%#codegen`, fixed sizes). Change the two builders.
- [ ] **Step 4:** Rebuild in batch.
- [ ] **Step 5:** Run the tests, then the code checks.
  - `t_mf_equiv`: all pass.
  - `t_gnc_mission` and `t_gnc_tracking`: the same results and measured lines as Task 1's before run.
  - `coder.screener` on both functions: no unsupported calls.
  - `build_targets('Companion')` passes. Run it only with the user's go-ahead, since it is a code build.
- [ ] **Step 6:** `testDomainLayout(domain='gnc')` passes, with lowered rows.

### Task 5: Close-out

**Files:**
- Modify: `docs/STATUS.md` (the new test file and its counts)

- [ ] **Step 1:** Check the interfaces and loops. `shaddad_check_interfaces('Compiled', true)` reports 0, and
  `getAlgebraicLoops` reports 0 on `Shaddad_System`, `tb_drive`, `tb_sim_monitors` and `tb_gnc_mission`.
- [ ] **Step 2:** Run the Fast block against `work/layout/regression_baseline.txt`. Expected: no new failure.
- [ ] **Step 3:** Export before/after images of the converted diagrams with `print -s`.
- [ ] **Step 4:** Update `docs/STATUS.md`, and write a short report covering:
  - the blocks before and after, per subsystem;
  - the equivalence results;
  - every ruling.
