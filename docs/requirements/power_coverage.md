# Power requirements: test coverage (T-PWR-03 precharge)

Coverage of `docs/requirements/power.yaml` (T-PWR-03 and its parts .a to .i), steps 3 to 6 of the `shaddad-req-tests` skill. Owner: shaddad-bms-power. Last update 2026-11-12.

The user decided the open points on 2026-11-12 (each requirement's `decision` field): .a is measured from K2 closing, .g is at most 30 ms, .c applies to the commands, R_pre stays 22 Ohm, and .h and .i are kept. A requirement is `Implemented` in `power.yaml` only where a test now passes for it.

## Coverage table

| ID | Requirement | Tests (existing; new) | Last result | Date |
|---|---|---|---|---|
| T-PWR-03 | Precharge the discharged DC bus through K2 and R_pre, close K1, open K2, set HV_Ready | `t_power_plant/precharge_T03` (whole sequence, `tb_power_plant`, VC_IMPL 1, 5.5 mF) | Pass. Status stays Review while .d, .g and .i are not Implemented | 2026-11-12 |
| T-PWR-03.a | V_bus reaches 0.97 V_pack 0.42 +/- 0.05 s after K2 closes | `precharge_T03` (dt_97) | Pass: 0.445 s | 2026-11-12 |
| T-PWR-03.b | K1 only after t_pre_min = 0.25 s in PRECHARGE and only with V_bus >= 0.97 V_pack | `precharge_T03` (K1 after the true 0.97 crossing); new `T-PWR-03.b-1`, `T-PWR-03.b-2` | Pass: K1 at 1.565 s, after the crossing; b-1 and b-2 pass | 2026-11-12 |
| T-PWR-03.c | K2 kept closed t_overlap = 0.05 s after the K1 command, then opened (commands) | `precharge_T03` (dK2open); new `T-PWR-03.c-1` | Pass: 50 ms; c-1 passes | 2026-11-12 |
| T-PWR-03.d | K1 inrush I^2t <= 2 A^2s | Analysis (`power.yaml` note); `precharge_T03` logs it | 0.037 A^2s at Backward Euler 1 ms, which does not resolve the 0.1 ms tau; no test verdict | 2026-11-12 |
| T-PWR-03.e | No fuse blows during precharge and K1 closing | `precharge_T03` (Fuse_Blown) | Pass: 0 | 2026-11-12 |
| T-PWR-03.f | HV_Ready by 1.60 s and kept | `precharge_T03` (first HV_Ready); new `T-PWR-03.c-1` (HV_ON held to the end of the run; HV_Ready = state HV_ON in `bms_mode_manager.m`) | Pass: 1.60 s, exactly at the limit | 2026-11-12 |
| T-PWR-03.g | K2 closes <= 30 ms after the BMS sees HV_Enable_Req | `t_pack_bms/precharge_bench` (verify `t_K2 - t_req_bms <= 0.030`); no `model_test` scenario possible (limit 7 below) | Pass: K2 closes 30 ms after the BMS-visible request (<= 30 ms, decided 2026-11-12); the 20 ms build plan T04 figure is now a log line, an accepted deviation | 2026-11-12 |
| T-PWR-03.h | PRECHARGE only on a valid request (Safety_OK, no dock, re-armed), with K2 only; back to STANDBY with K2 open when the request or Safety_OK drops; a new request edge after a Safety_OK drop | New `T-PWR-03.h-1` to `T-PWR-03.h-5`; `t_pack_bms/mode_manager_table` (re-arm after a Safety_OK drop from HV_ON; the PRECHARGE branch runs the same assignment) | Pass: 5 scenarios, 25 assessments; `mode_manager_table` passes | 2026-11-12 |
| T-PWR-03.i | R_pre peak current, power and energy per precharge | Analysis | No test | - |

## New scenarios

File `tests/requirements/power/T-PWR-03.feature`: model `Shaddad_BMS.slx`, component `Shaddad_BMS/BMS_Core/ModeManager/BMS_ModeManager` (the MATLAB Function block, 0.01 s). Each scenario title starts with its requirement ID, because the dialect has no tags.

| Scenario | Checks | Result 2026-11-12 |
|---|---|---|
| T-PWR-03.h-1 | Request at 1.0 s: STANDBY before, PRECHARGE with K2 and without K1 within 0.01 s | Pass |
| T-PWR-03.h-2 | No PRECHARGE while Dock_Present; PRECHARGE within 0.01 s of undocking | Pass |
| T-PWR-03.h-3 | No PRECHARGE while Safety_OK is false; PRECHARGE within 0.01 s of Safety_OK | Pass |
| T-PWR-03.h-4 | HV_Enable_Req drop in PRECHARGE: STANDBY with K2 open within 0.01 s | Pass |
| T-PWR-03.h-5 | Safety_OK drop in PRECHARGE: STANDBY with K2 open within 0.01 s | Pass |
| T-PWR-03.b-1 | Bus already at 0.978 V_pack: no K1 before 1.25 s, K1 at the 1.25 s sample | Pass |
| T-PWR-03.b-2 | Bus at 0.964 V_pack: no K1, PRECHARGE held | Pass |
| T-PWR-03.c-1 | K2 falls 0.050 +/- 0.010 s after the K1 command, HV_ON held | Pass |

8 of 8 scenarios and 35 of 35 assessments pass (`model_test`, full compile, shared MATLAB, about 11 s).

### How to run

`model_test` fails on every Shaddad model as it stands (limit 1), so a run needs an in-memory configuration swap around it, with `tests/power/pwr_mt_config.m`:

1. `pwr_mt_config('Shaddad_BMS', 'local')`
2. `model_test` with model `Shaddad_BMS.slx` (the bare name: it must equal the `model` line of the file), the feature file, `draft_mode` false.
3. `pwr_mt_config('Shaddad_BMS', 'restore')`

The helper never saves the model and clears its dirty flag after each swap (limit 2).

## Where `model_test` keeps its files

- **Harness.** It is an external Simulink Test harness of the component, `component_<hash>_final_harness.slx` (`_draft_harness` in draft mode). It lives with `<name>_cache.json` and `_HarnessInputs_<n>.mat` in the Simulink cache folder: `work/slprj/mbd_agent/test/Shaddad_BMS/`, plus `work/slprj/mbd_agent/_meta/`. It is rebuilt from the model on every run.
- **Registration.** `models/Shaddad_BMS_harnessInfo.xml` sits next to the generated model and lists both harnesses. It is not part of the `.slx`, and the builder does not know it.
- **Test file.** `<model>_<component path>_test.mldatx` goes into MATLAB's current folder (the project root) and is loaded in the Test Manager. A completed run removes it. A failed run leaves it, together with `_HarnessInputs*.mat` files and sometimes the harness itself, in the current folder.
- **Debug signals.** For each failed scenario they go to `work/slprj/mbd_agent/test/Shaddad_BMS/debug/<scenario>_signals.mat`.
- **Compile caches.** These are `work/Shaddad_BMS.slxc`, `work/<harness>.slxc`, `work/slprj/sim/varcache/` and `work/slprj/_sfprj/`.
- **The model.** `Shaddad_BMS.slx` is not changed, except that a dirty model is saved before the harness is created (limit 2).

## Limits found (toolkit skill `testing-simulink-models` 2.0.1, R2026b)

1. **Configuration reference.** `model_test` rebuilds its harness from the model and then calls `set_param` on the harness configuration. That fails on `cs_Shaddad`, which every Shaddad model uses by reference: `Simulink:ConfigSet:ConfigSetRef_SetParamNotAllowed`, "Could not create Simulink Test file". This happens in draft and full mode alike, and also with a cached harness: "Optimistic sync failed, recreating harness: Interface changed (CUT: 0 in/0 out ...)". The workaround is `pwr_mt_config`.
2. **Auto-save.** A dirty main model is saved before the harness is created ("had unsaved changes and was automatically saved before test harness creation"), even when it is a generated model.
3. **Draft mode.** Draft mode cannot run a 0.01 s component: its 1 ms input sources have no rate transition (`Simulink:DataTransfer:IllegalIPortRateTrans`). Use the full compile.
4. **File structure.** There are no tags, and each file holds one front matter, so one model and one component.
5. **Inputs.** The only inputs are `const`, `step` (one transition) and `ramp(a -> b @ t1 .. t2)`; the skill reference does not document `ramp`. On the single-typed V_bus input the ramp behaves as a step at t2: CLOSE_MAIN came at 1.50 s, the ramp end, instead of the 0.97 crossing at 1.37 s. A signal that falls and rises again cannot be expressed, so the re-arm case of .h stays in `mode_manager_table`.
6. **Windows.** `when t == [a .. b]` is the closed interval a <= t <= b. An assessment must hold at every instant of the window, edges included, with the outputs held between their 0.01 s samples. A window edge therefore sits at the requirement limit, not between samples.
7. **Components.** A component needs signal ports: no physical ports, Inport blocks named like the port, and bus element ports with a bus object. So the K2 contact (`Power_Plant/PDU`, physical ports) and both benches (no root ports) are out of reach, and so is `BMS_Core/CmdOut`, which holds the 10 ms output delay ("Component has virtual bus without a bus object, which is not supported yet"). No component carries the .g chain from the request to the contact.
8. **Coverage.** `coverage` `decision` returns no data, because Simulink Coverage is licensed but not installed.
