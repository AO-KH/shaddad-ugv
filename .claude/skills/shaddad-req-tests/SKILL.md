---
name: shaddad-req-tests
description: Turn a Shaddad UGV domain spec into requirement drafts and automated model tests. Extracts the testable requirements of docs/specs/<domain>.md, maps the existing tests to them, writes Gherkin scenarios for the gaps, runs them with the Simulink Agentic Toolkit's model_test (Simulink Test) and keeps a coverage table. Use when the user asks for requirements, test scenarios, verification or test coverage of a Shaddad domain.
---

# Shaddad requirements to tests

Each domain has a spec: drive, electronics, gnc, integration, power, sensors and structure, in docs/specs/<domain>.md. Each spec starts with a Revision B note: docs/architecture.md section 2b (D30 to D65 and any newer) and docs/build_plan.md override the spec where they differ. So check each requirement against them before writing it down.

With the agent team, the subsystem engineer that owns the domain does all the steps, and `shaddad-reviewer` checks the tests before they are kept.

The MATLAB work runs where the shaddad-test-fix skill says:
- short work in the user's shared MATLAB, with the same manners;
- long runs in a batch MATLAB.

## Steps

1. **Extract** each testable statement, with:
   - an ID;
   - the text;
   - the source (the spec section);
   - the model and signal it constrains;
   - the pass value, with its tolerance and conditions;
   - the verification method: test, analysis or inspection.

   Keep the spec's own IDs: EL-xx for electronics, INT-xx for integration, PWR-xx for power, NAV-xx for sensors. Drive, gnc and structure have none: propose DRV-xx, GNC-xx and STR-xx, and say so.
2. **Draft.**
   - Requirements Toolbox is not installed, so the drafts are YAML, in docs/requirements/<domain>.yaml. (In that case the toolkit skill generate-requirement-drafts writes YAML too.)
   - The user reviews the drafts before any test is written, because they decide what passing means.
3. **Map the existing tests** (tests/<domain>/t_*.m and tests/system/tScenarios.m; the scenarios' pass criteria are in scripts/scenarios/scn_*.m).
   - Their names carry the specs' test numbers: testT02_step, overload_T06b, tSEN03_LeverArm, testTEL27_NoPulses.
   - Mark each requirement covered, partly covered or not covered, with the test names.
4. **Write tests for the gaps** with the toolkit's testing skill (testing-simulink-models).
   - Write Gherkin scenarios tagged with the requirement IDs, in tests/requirements/<domain>/. `model_test` runs them (Simulink Test is installed).
   - Test the unit model or its bench (tests/<domain>/tb_*.slx, a few seconds) rather than Shaddad_System (minutes).
   - The builders regenerate the models, so a test harness must not live inside a generated .slx. Keep harnesses external, or let the test create them.
   - The first time, look at how the toolkit stores scenarios and harnesses, and follow it.
   - **How `model_test` works on the Shaddad models** (found 2026-11-12 on T-PWR-03; details in `docs/requirements/power_coverage.md`):
     - **Configuration swap.** Every model uses `cs_Shaddad` through a configuration reference, which refuses the `set_param` that `model_test` makes on its harness. So wrap each run in the in-memory swap: `pwr_mt_config('<model>', 'local')`, then `model_test`, then `pwr_mt_config('<model>', 'restore')` (`tests/power/pwr_mt_config.m`, usable for any model). Never save the model with the local copy.
     - **The model name:** pass the bare model name (for example `Shaddad_BMS.slx`), the same as the `model` line in the feature file.
     - **Components:** a component must have plain signal ports. A bench with no root ports, a part with physical ports, or a subsystem whose bus ports have no bus object cannot be a component. Test the controller unit (for example `Shaddad_BMS/BMS_Core/ModeManager/BMS_ModeManager`), and leave chains through the plant to the MATLAB tests.
     - **Draft mode** fails on a component that runs at its own rate (illegal rate transition), so use `draft_mode` false.
     - **No tags** in this Gherkin dialect: put the requirement ID in the scenario title.
     - **Assessment windows** are closed intervals, checked at every instant, so put window edges at the requirement's own limits.
     - **What it leaves behind:**
       - the harness registry `models/<model>_harnessInfo.xml`, next to the generated model;
       - the harness and its inputs in `work/slprj/mbd_agent/test/<model>/`.

       Both are rebuilt on every run. A failed run also leaves `<model>_<component>_test.mldatx` and some `.mat` files in the current folder; remove them.
     - **Coverage:** `coverage` 'decision' gives no data here, because Simulink Coverage is not installed.
5. **Run and sort.** A new test that fails is either a model fault or a wrong scenario.
   - A model fault goes to the shaddad-test-fix loop, or into the report.
   - Never loosen a scenario just to make it pass; that needs the user's yes.
6. **Keep a coverage table** in docs/requirements/<domain>_coverage.md: ID, requirement, tests, last result and date.
7. **Log each step:** `python tools/dashboard/log.py result "<text>" --task "Requirements to tests"`.
