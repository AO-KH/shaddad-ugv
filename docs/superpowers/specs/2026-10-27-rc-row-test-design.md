# RC row test: simulate a remote-controlled row-driving session from user inputs

Status: draft for the user's approval (2026-10-27). Owner of the decisions: the user; calls marked
**[call]** were made by the designer and can be changed.

## 1. Intent

The user wants to run their own long tests of the real-robot model the way the real robot is used today: a
person drives it by remote control (RC) up and down crop rows. At the start of a run they are asked a few
questions (how many rows, how many minutes, the ambient temperature, sprayer or not); the simulation then runs
without any 3-D animation; afterwards every plot is saved so they can study the RC link, the battery (current,
energy consumption, state of charge), the temperatures and all other signals of the session.

Why: the existing Phase 1 tools cover short scripted manoeuvres (`phase1_test_drive`, about 1 min) and live
driving with a controller in real time (`phase1_live_drive`). Neither gives a long, realistic, repeatable
working session whose loads (straight rows, turns at the row ends, sprayer pump) match the field.

## 2. Goals, non-goals, anti-goals

Goals
- One command starts a row test; a small window asks the inputs; the run is unattended.
- The robot is driven by a **simulated human driver** that watches the robot and moves the RC sticks, so the
  whole Phase 1 chain runs exactly as with a real operator: sticks -> RC transmitter -> SBUS -> STM32 decoder,
  RC mapping, mode manager, vehicle control -> CAN -> motor controllers -> motors, gearboxes, brakes -> tracks
  on soil, powered by the 48 V pack and BMS.
- Rows covered are measured, not guessed; a run with the same inputs gives the same result.
- After the run all plot pages are saved (PNG to look at, MATLAB figure to zoom), with a short summary.

Non-goals
- Autonomous (Phase 2) row following; the GNC is not used.
- 3-D animation or live view during the run (the user asked for none; it also costs speed).
- Pass/fail criteria: this is a free test, not a verification case.
- A faster low-fidelity variant (possible later).
- Real hardware in the loop.

Anti-goals (would make it a failure even if it runs)
- A driver that bypasses the RC chain (for example by writing speed commands into the MCU): the RC, SBUS and
  MCU plots would no longer show a real session.
- Signals missing from the saved plots, or plots that cannot be reopened and zoomed.
- Runs that differ from one another with the same inputs.

## 3. Approaches considered for the driver

| Approach | What it is | Trade-off |
|---|---|---|
| 1 Timed stick script | stick moves worked out in advance from the field (straight for L/v, U-turn at a fixed turn rate, ...), through the existing `phase1_test_drive` plan | no model change, but nothing watches the robot: it drifts off the rows after a few turns (track slip differs in every turn), so the rows covered become a guess |
| **2 Simulated driver that watches the robot (chosen)** | a driver block inside the simulation sees the robot's true position and heading, as a person at the field edge does, and moves the sticks to follow the rows and turn at the ends | needs a new operator choice in the system model and a rebuild; rows covered are real and the run repeats exactly |
| 3 The user drives | `phase1_live_drive` with the PS5 controller in the 3-D view on a field with rows | no driver model, but real time only, the user at the controller for the whole run, never the same twice |

The user chose 2 (2026-10-27). Approach 3 stays available as it is.

## 4. What the user does

1. In MATLAB: `phase1_row_test` (name **[call]**, in `scripts/`, next to `phase1_test_drive` and
   `phase1_live_drive`).
2. A window asks, with the last used values pre-filled **[call: remembered in work/rowtest_last.mat]**:
   - number of rows (integer, 2 .. 30 **[call]**: with one row the return to row 1 would need a loop turn),
   - minutes of driving (driving time after the robot is armed),
   - ambient temperature in degC,
   - sprayer: yes / no (check box),
   - fast: a check box, off by default (user decision: full detail by default, fast as an option).
   It shows the estimated wall-clock time for these inputs (section 8) and has Run and Cancel buttons.
3. The run starts in a separate MATLAB process in the background, as the Phase 1 suite runs its cases
   (`matlab -batch`) **[call]**: runs last hours (section 8), a fresh process keeps memory below the point
   where this PC slows down (about 9 GB, docs/STATUS.md), and the user's MATLAB stays free. The Command Window
   prints the start time, the expected end time and the folder the results will go to; the background process
   writes a progress log there.
4. At the end the background process writes the summary and all plots to that folder and opens nothing
   (**[call]**: the user studies the saved files; `plot_run` can reopen any page).

The same function also accepts the inputs as arguments, so a run can be scripted without the window
**[call]**: `phase1_row_test('Rows', 8, 'Minutes', 20, 'Ambient', 40, 'Sprayer', true)`.

## 5. Fixed settings (first version)

All **[call]**, in one settings struct `P.scen.rowtest` (numeric only, D58) so they can be changed later
without touching code:
- field: rows 50 m long, 3 m apart (centre to centre), row 1 starting at the origin heading east, the rows
  laid side by side to the north; headlands 6 m beyond the row ends;
- ground: flat, farm soil (soil class 2), no obstacles;
- driving: 1.0 m/s along the rows, 0.5 m/s in the turns; RC speed knob at 100 %;
- sprayer (when chosen): implement 3 mounted, tank full (40 L) at the start, pump switched on along the rows
  and off in the turns and on the return run, using the operator's implement switch (`sw_impl`) as a real
  operator does; when the tank runs empty the run goes on with the pump off. As built (user decision
  2026-10-28, Task 11): the driver also sees the true tank volume (`impl_mech.tankLevel`, as an operator sees
  the spray stop) and keeps the switch OFF once the tank is down to `tank_empty_L` (0.1 L **[EST]**,
  `P.scen.rowtest`); there is no refill. The 40 L lasts about 8.3 min of spraying, about 10 min of driving
  (10 min 25 s measured with 6 rows at 30 degC; a little less with many rows or when the rows are driven
  slower, as under the 45 degC derate);
- battery: state of charge 95 % at the start (`P.scen.soc0`);
- temperatures: the ambient temperature of the input drives the environment (`ScenarioIn env.use_T / T_amb`,
  as scenario S3 does), and every thermal state starts at ambient (pack `P.power.thermal.T_init_K`, drive
  `P.drive.thermal.T_amb0`, electronics enclosure `P.electronics...T0`), as for a robot that stood outside.

## 6. The field

World 1 (flat test field) cannot hold it: it is 60 x 60 m (x, y in -10 .. 50 m) with an asphalt strip, a sand
strip and a palm trunk at (25, 10) inside. The row test gets its own procedural world **[call: world id 7,
`row_field`, in param_world]** (as built; this spec first said id 6): flat, farm soil everywhere, no obstacles,
sized to the field plus the headlands and a margin.

Constraint for the builder: worlds 1 and 2 use a 241 x 241 terrain grid at 0.25 m. If the model needs that grid
size fixed (parameter dimensions baked into a compiled model), keep 241 x 241 and let the resolution follow
the field size; otherwise size the grid freely. The input window limits the number of rows so that the field
fits the world: with the grid kept at 241 x 241 points and its resolution following the field size (at least
0.25 m), 30 rows of 50 m at 3 m fit **[call]**. Rows repeat (section 7), so a small field still gives long runs.
As built, the step and the centre of the grid are rounded to a 1/64 m lattice (0.3125 m for up to 18 rows), so
the grid is evenly spaced to the last bit, as the plant's 2-D lookup tables require.

## 7. The simulated driver

Behaviour (what a careful operator does):
- **Start**: transmitter on, sticks neutral, mode switch MANUAL, then the arm switch, as `scn_rc_tab` does today
  (transmitter on at 0.5 s, armed at 2 s); the driving time of the input starts when the robot is armed.
- **Along a row**: throttle for the row speed; steering to stay on the row centre line, aiming at a point a few
  metres ahead on the line (pure-pursuit style **[call]**: look-ahead about 2.5 m at 1 m/s).
- **At a row end**: slow to the turn speed before the end, turn 180 deg into the next row (semicircle of radius
  spacing / 2 = 1.5 m, at 0.5 m/s a yaw rate of 0.33 rad/s, inside the RC limits), line up and speed up.
  The rows are driven in a serpentine: row 1 east, row 2 west, and so on.
- **After the last row**: drive back along the headland to row 1 and drive the rows again in the same order
  (user decision: start again until the time is up). With an odd number of rows the robot is then at the far
  end and enters row 1 from that end **[call]**.
- **Like a person** **[call]**: about 0.3 s reaction delay on what it sees, sticks moved smoothly (rate limited:
  full travel in no less than about 0.3 s), a small slow steering unsteadiness. The unsteadiness is a fixed sum
  of slow sines (as built; no random numbers), so the same inputs give the same run.
- **End**: when the minutes are up the driver releases the sticks (neutral), the robot stops, then disarms;
  the simulation ends a few seconds later.
- **Interruptions** (user decision): through a derate (speed, torque or current limited by the MCU or BMS) the
  driver keeps driving; the robot is only slower. When the robot stops for a reason the driver did not cause
  (the MCU leaves MANUAL: CONTROLLED STOP, ESTOP or FAULT, for example after a battery trip) the driver
  releases the sticks and the run ends early; there is no automatic reset. The summary gives the reason (MCU
  mode, MCU fault bits, BMS fault word) and the time. The driver therefore also gets the MCU mode from the
  telemetry (`MCU/6`, through the same Rate Transition and Unit Delay as the pose; as built, the 0.3 s reaction
  delay applies to the pose only); ending the run early needs a stop condition in the model (for
  example a Stop Simulation block on the driver's "ended" state) **[left to the builder]**.

Speed and turn targets are turned into stick positions with the existing inverse of the MCU RC mapping
(`utilities/scn_rc_stick.m`), so the MCU decodes exactly the intended speed and turn rate. As built, the full
yaw-rate target of the pure pursuit goes through that table. The spec's first idea, a steering correction added
as a stick offset, was dropped: a small offset would fall inside the MCU's +-5 % stick dead zone and be lost.

Where it lives **[call]**:
- `Shaddad_System/Operator_Console` gets a fourth choice of its Variant Source: `VC_OPSRC == 4`, subsystem
  `RowDriver` (built by `scripts/build/build_shaddad_system.m`; models are never edited by hand). Choices 1-3
  are unchanged.
- The driver needs the robot's true pose: the truth bus (`Plant/1`, `pose2D` = [x_E y_N psi], `speed_fwd`)
  is routed to a new input of `Operator_Console` through a Rate Transition to `Ts.scen` and a Unit Delay
  (no algebraic loop; part of the reaction delay). As built, the console has inputs 2 pose, 3 v_fwd,
  4 mcu_mode and 5 tank_L (the sprayer tank volume from `Plant/14` `impl_mech.tankLevel`, Task 11), each
  through its own Rate Transition and Unit Delay.
- The driver logic is a small state machine (start, row, slow-down, turn, return, end) plus the steering law,
  written as a MATLAB function in `utilities/` called from a MATLAB Function block, like `joy_map` and the
  ESC state machine (Stateflow is not installed, D1); its parameters come from `P.scen.rowtest` as a
  sub-struct (MATLAB Function blocks get sub-structs only, D58).

## 8. The run

- New scenario `R1` (`scripts/scenarios/scn_R1_row_test.m`), run by `run_scenario('R1', ...)` in Normal mode
  like `D1`; the log goes to `work/results/R1_<yyyymmdd_HHMMSS>.mat`.
- Variant controls: `VC_PHASE 1`, `VC_OPSRC 4`, `VC_VIZ 0` (no 3-D view), `VC_IMPL 3` with the sprayer, else 0.
- Fidelity (user decision): **full detail by default**, the same plant variants as `D1` (grouped pack with its
  cell groups and BMS, SYS drive with its losses and temperatures, high-fidelity tracks). With the "fast" box
  ticked: the fast preset of `phase1_live_drive` (`'Fidelity','fast'`: lumped pack, ideal drive, kinematic
  chassis); the summary and every saved page say which fidelity the run used.
- Stop time: arming time + minutes x 60 s + about 10 s for the stop.
- Speed (docs/live_rc_drive.md, measured 2026-10-26): the **fast preset** (lumped pack, ideal drive,
  kinematic chassis) runs at about 1.07x real time without the view and about 0.6x with an implement; **full
  fidelity** runs at about 0.3-0.5x (lower with the sprayer); plus 2-3 min to compile. The window shows the
  estimate for the chosen inputs. Which fidelity to use is open question Q1 (section 13).
- `run_scenario`'s default 5 h timeout must follow the stop time (a 3 h full-fidelity run can take 8-12 h).
  As built, `run_scenario` parses its `'Timeout'` option but never applies it, so no run is stopped on wall
  time. The row test therefore passes no timeout, and nothing stops a slow run. The launcher's lock lapses after
  3 x the estimate.
- As built, the measured speeds (2026-10-28, with enough free memory) are 0.55x real time for full detail, 0.51x
  with the sprayer, 1.20x for fast and 0.92x for fast with the sprayer. A run also has a fixed part of about
  2.5 min. With little free memory the runs are 2-6x slower (`params/param_rowtest.m`, docs/row_test.md).
- Logging must keep long runs manageable **[left to the builder]**: decimate as the long scenarios do (S3) so
  that a 3 h run stays well below 1 GB and every `plot_run` page still has its signals.

## 9. After the run: what is saved

One folder per run **[call]**: `work/reports/rowtest_<yyyymmdd_HHMMSS>/` with
- the six existing `plot_run` pages (drive, tracks, battery, power, rc, nav), each as PNG and as a MATLAB
  `.fig` (reopen and zoom later);
- a new **field page** (PNG and `.fig`): the path over the rows (rows drawn, path coloured by time), rows
  completed against time, the distance from the row centre line along the rows, and the energy used per row;
- `summary.txt`: the inputs; rows completed (counting repeats) and how many times the field was covered;
  driving time, distance, mean speed; energy from the pack, Wh per row, Wh/km; state of charge at start and
  end; peak pack current; highest cell, motor-winding, motor-controller and enclosure temperatures; largest
  distance from the row line; any MCU fault or warning bits and BMS faults that were set; wall-clock time and
  real-time factor;
- the path of the run log in `work/results` (the log is not copied **[call]**).

A row counts as completed when the robot travelled at least 95 % of its length within 0.5 m of its centre line
**[call]**; the count comes from the true path after the run, independent of the driver.

`plot_run` gets an option to save the `.fig` files and the field page as a seventh page for R1 logs; its six
pages and its saving of PNGs (`'Save', true`, `work/reports/plots_<log>`) stay as they are for other runs.
As built:
- plot_run has `'Folder'`, `'Fig'` and `'Note'` options; the note, the fidelity, goes into every page title.
- The field page is not a plot_run page, because plot_run has neither the rows nor the summary figures. The
  worker saves it as `7_field` next to the six pages, and `row_test_field_page` redraws it from the log and
  `summary.mat` (docs/row_test.md).

## 10. Constraints

- Models are generated by the build scripts, never edited by hand. Never run `build_shaddad_mcu('all')` or
  `build_shaddad_mcu()`; the MCU does not change here.
- P stays numeric (D58): the inputs and settings are numbers; text goes to P_meta if needed.
- Sequencing with the report session (2026-10-27: Full suite running, then a locking brake model, then a
  second Full suite): the driver's stand-alone bench (section 11) can be built at once, in new files only.
  Everything that edits shared files or rebuilds `Shaddad_System` waits until the report session has finished
  its second Full suite and delivered the report: `param_sim.m`, `param_world.m`, `shaddad_params.m` (world
  names), `build_shaddad_system.m`, `plot_run.m`, `tests/integration/t_infra.m` (it checks for exactly three
  operator-console choices and must accept four).
- The real-robot model has just changed (four motors, D64; a locking brake model is being added by the report
  session); the row test uses whatever the model is at run time.
- English only; NASL conventions for anything written for others. The user manual stays unchanged unless the
  user asks for a section.

## 11. Verification

- The driver on its own: a small bench (kinematic skid-steer robot, the RC mapping) checks that it stays within
  0.3 m of the row line on straights, completes the turns, returns to row 1 and stops at the time limit.
- System: no new failures against a baseline taken on the model as the report session leaves it (S8a and D2
  failed on the four-motor model before the locking brake; the baseline is the state after it): t_infra (four
  operator choices), t_framework, the Phase 1 Quick suite.
- A sample run: 4 rows, 5 minutes, 30 degC, sprayer on; the summary and the seven pages are saved, the rows
  count matches the path, the pump runs only along the rows, the temperatures start at 30 degC.

## 12. Left to the builder

Block-level layout of `RowDriver`; the exact steering law and its tuning (within the behaviour above);
decimation of the logs; the layout of the field page; the window's look (a small uifigure is suggested,
because a check box is needed); how the last inputs are remembered.

## 13. Decisions taken with the user (2026-10-27)

- Driver: approach 2, a simulated driver that watches the robot (section 3).
- Inputs asked at the start: rows, minutes, ambient temperature, sprayer yes / no, fast yes / no.
- Rows finished before the time is up: start again from row 1 until the time is up.
- No 3-D animation during the run; all plots saved after the run.
- Fidelity: full detail by default, a "fast" box for the fast preset.
- Interruptions: keep driving through derates; end the run and report on a stop the driver did not cause.
