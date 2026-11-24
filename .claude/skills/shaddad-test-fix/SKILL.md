---
name: shaddad-test-fix
description: Test-and-fix loop for the Shaddad UGV Simulink project. Runs a test class, a single test, a tag or the Fast tier; compares the results with the known-failure baseline (tests/test_baseline.json); diagnoses each new failure in the models with the Simulink Agentic Toolkit tools; fixes the cause in the builder scripts; rebuilds, reruns and reports. Use when the user asks to run, check or fix Shaddad tests, or to verify a model change.
---

# Shaddad test-and-fix loop

One pass: run, sort the results, fix each new failure once, rerun, report. If a fix does not hold or the cause stays unclear, stop and report with the evidence; a second round needs the user's word.

## Where the MATLAB work runs

- **The user's shared MATLAB**, through the MCP tools. The user connects it once per MATLAB session with `addpath("C:\Users\aomar\.matlab\agentic-toolkits\simulink"); satk_initialize`, which shares the session and loads the toolkit's MATLAB side. The MCP server runs with `--matlab-session-mode=existing`, so it can never start a MATLAB of its own. Use it for:
  - model inspection: `model_overview`, `model_read`, `model_query_params`, `model_resolve_params`, `model_read_diagnostics`, `model_check`;
  - single short tests and quick builds.
  - If the tools find no shared session, ask the user to share theirs; never start a MATLAB through the tools. If the Shaddad project is not open there, ask the user to open it.
- **A separate batch MATLAB** for anything over about 5 minutes (whole classes, the Fast tier, sweeps, scenario runs), run in the background:
  `bash tools/agentic/run_batch.sh <cap_s> work/agentic/logs/<date>_<scope>.log "shaddad_run_tests('<class>')"`.
  It waits for any other batch MATLAB, so only one heavy MATLAB runs at a time on this 16 GB PC.
- **Manners in the user's session:**
  - Work through functions.
  - Never `clear`, `close all`, `bdclose all`, and never overwrite base-workspace variables.
  - Put the current folder back.
  - Before rebuilding, saving or closing a loaded model, check `bdIsDirty`; if the user has unsaved changes, ask.

### Class times on this PC (from the baseline)

| Time | Classes |
|---|---|
| Under 2 min | t_world, t_rc_link, t_row_driver, t_elec_plant |
| 2–10 min | t_gnc_tracking 2, t_targets 6, t_drive 7, t_row_test_system 7, t_structure 9, t_perception 9, t_sensors 10, t_gnc_mission 10 |
| 11–20 min | t_layout 11, t_infra 13, t_framework 15, tScenarios 16, t_power_plant 20 |
| 1 h and more | t_nav 1 h, t_mcu_platform 1.4 h, t_pack_bms 2–4 h, t_mcu_control several hours (its baseline time includes sleep) |

The Fast tier (`run_regression('Fast')`) aims at 10 min.

## Steps

1. **Scope.**
   - It is a class (`t_sensors`), a test (`t_infra/testStartup`), a tag, or the Fast tier. If the user gave none, propose the Fast tier.
   - Log the start: `python tools/dashboard/log.py start "<scope>" --task "Test-and-fix"`.
2. **Run** with `shaddad_run_tests`, in batch for long scopes:
   - It prints one line per test.
   - `'Name', 'test*'` and `'Tag', 'X'` pick tests.
   - `'All', true` adds the left-out code-build and rebuild tests.
   - The Fast tier is `shaddad_run_tests('*', 'Tag', 'Fast')`: the same 66 tests as `run_regression('Fast')`, but one line per test, so the baseline compare can read it.
   - With the agent team, `shaddad-tester` does the runs and the sort. Each new failure then goes to the subsystem engineer that owns the failing test or model, and `shaddad-reviewer` checks the fix.
3. **Sort:** `python tools/agentic/test_baseline.py compare <log>`. Each known failure has its reason in the class note of tests/test_baseline.json; leave known failures alone unless the user asks.
4. **Diagnose each new failure.**
   1. Read its diagnostic and rerun it alone: `shaddad_run_tests('<class>', 'Name', '<test>')`.
   2. Find the model, block, signal or parameter at fault with the model tools.
   3. Find the builder that makes that part:
      - `build_all('All', 'DryRun', true)` lists the builders in build order;
      - `grep -rn "<block or model name>" scripts/build` finds the line.
   4. Decide the kind of fault: the model or its builder, an outdated test expectation, or the environment (timing on this laptop, missing tools, saved results).
5. **Fix the cause in the builder**, or in the function the model calls.
   - Never fix it only in the .slx: the builders regenerate every model and would undo it.
   - Rebuild only what is needed, dependencies first (build order in `build_all`'s help).
   - Never run `build_shaddad_mcu('all')` or `build_shaddad_mcu()`. Never run `build_fw_platform` without unit names. No STM32 code builds: those tools are not installed.
   - Keep each file's line endings.
   - Changing what a test expects needs the user's yes.
6. **Check the fix.**
   - Rerun the failing test, then the whole scope, and compare again.
   - After a builder change, also run the checks below.
7. **Report.**
   - For each new failure: its cause, the fix (file:line) and the result. Then what still fails, and why.
   - When the user accepts the new results, refresh the baseline: `python tools/agentic/test_baseline.py build <logs> --merge`.
   - Log: `python tools/dashboard/log.py result|issue|done "<text>" --task "Test-and-fix"`.

## Checks after a builder change

- **Layout:**
  - `T = shaddad_layout_check({'<model>'}); [pass, F] = shaddad_layout_check('targets', T)`. The allowed crossings are in utilities/shaddad_layout_allowance.m.
  - `model_check` for unconnected ports and dangling lines.
- **Interfaces:** `shaddad_check_interfaces('Compiled', true)` returns 0 mismatches.
- **Structure tests:** `shaddad_run_tests('t_layout')`.
- **Results that must not change:** compare scenario runs with `shaddad_compare_logs`.

## Rules

- Never edit docs/report/**.
- P stays purely numeric (D58).
- One pass per task; fix only real breakage.
- Leave the user's open models and workspace as they are, beyond what a step needs.
