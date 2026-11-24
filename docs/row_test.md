# RC row test (Phase 1)

Simulate a working session of the Shaddad UGV as it is used today: a person drives it by remote control (RC) up and
down crop rows. You choose how many rows, how many minutes, the ambient temperature, and whether the sprayer works.
A simulated driver then drives the rows through the whole RC chain, in a separate MATLAB and without a 3-D view.
When the run ends, every plot page and a short summary are saved in one folder, ready to study: the RC link, the
battery (current, energy, state of charge), the temperatures and all other signals.

The reference sample is `work/reports/rowtest_20261112_132942`: the default inputs (6 rows, 10 minutes, 45 degC, no
sprayer, full detail), made after D65 (a BMS warning no longer derates). Kept for comparison: `rowtest_20261028_173747`,
the same inputs made before D65. Two earlier samples are `rowtest_20261028_155031` (full detail) and
`rowtest_20261028_160548` (fast), both with 4 rows, 5 minutes, 30 degC and the sprayer on. Their `summary.txt` and
`summary.mat` were written again from their logs with the final summary, but their `progress.log` files and pages
predate the final code: the log shows an older estimate and a time limit that is not applied, and the page titles do
not give the fidelity.

## What runs

The simulated driver watches the robot's true position, as a person at the field edge would, and with the sprayer
the true level in its tank (a person sees the spray stop when the tank is empty). It moves the sticks and switches
of a simulated transmitter. From there the whole Phase 1 chain runs as with a real operator: transmitter
→ SBUS → STM32 firmware model (SBUS decoder, RC mapping, mode manager, vehicle control) → CAN → motor controllers →
motors, gearboxes and brakes → tracks on farm soil, powered by the 48 V pack and its BMS. The driver never writes
speed commands into the MCU.

What the driver does:

1. Transmitter on at 0.5 s, sticks centred, mode switch MANUAL; arm switch on at 2 s. The driving time you asked for
   starts when the MCU is in MANUAL.
2. The rows are 50 m long and 3 m apart. Row 1 starts at the origin and points east; the others lie side by side to
   the north. Along a row the driver drives at 1.0 m/s and slows to 0.5 m/s 3 m before the end. At the row end it
   makes a 180° U-turn (radius 1.5 m) into the next row. On the first pass row 1 runs east, row 2 west, and so on;
   with an odd number of rows the next pass enters row 1 from its east end and drives it west.
3. After the last row it drives back along the headland to row 1 and starts again. It goes on until the time is up.
4. With the sprayer: implement 3 is mounted with a full 40 L tank. The driver switches the pump on along the rows and
   off in the turns and on the way back (implement switch, CH8). The nozzles use about 4.8 L/min, so the tank lasts
   about 8.3 minutes of spraying, which is about 10 minutes of driving: 10 min 25 s in the 30 degC example below, a
   little less with many rows (9.9 minutes with 30) and when the rows are driven slower (9.4 to 9.9 minutes at
   0.71 m/s, the derated hot run before D65; both estimated on the driver bench). So with the sprayer at the default
   10 minutes and 45 degC, the tank was estimated to run empty at about the end of the run; this is not measured at
   the 0.85 m/s of the reference sample yet. When the tank is empty, the driver
   switches the pump off, as an operator who sees the spray stop would, and drives the rest of the run with the pump
   off; there is no refill (and the robot is 40 kg lighter). It switches off at once when 0.1 L is left
   (`tank_empty_L` in `params/param_rowtest.m`), so the pump never runs dry. To see when the tank emptied:
   - the rc page (`5_rc`): CH8 goes OFF in the middle of a row and stays OFF;
   - the power page (`4_power`): the 24 V rail current, which falls to about 1 A in every turn, stays at that level
     along the rows too;
   - the battery page (`3_battery`): the LV electronics power likewise stays at its turn level, about 60 W, and the
     implement energy line goes flat (a small line, about 15 Wh: zoom in on the `.fig`);
   - the run log: `truth.mass` stops falling.

   A 14-minute fast run at 30 degC with 6 rows (`work/reports/rowtest_20261028_194350`) shows it: the tank emptied
   after 10 min 25 s of driving, halfway along the 10th row; along the rows the 24 V rail current fell from 5.5 A to
   1.0 A, and the energy per row from about 11.2 Wh to 9.1 Wh (no pump load, 40 kg less).
5. When the time is up: sticks centred until the robot stands, then disarm. The MCU passes through `CTRL_STOP` for
   half a second on its way to `STANDBY` (its normal disarm), and the simulation ends a few seconds later.
6. Like a person: it reacts 0.3 s late, moves the sticks smoothly, and its steering wobbles a little. The wobble is
   a fixed sum of slow sines, so the same inputs always give the same run.
7. Interruptions: through a derate (speed, torque or current limited by the MCU or the BMS) it keeps driving, only
   slower. When the MCU leaves MANUAL for a reason the driver did not cause (controlled stop, E-stop, fault), it
   centres the sticks and the run ends early. There is no automatic reset.

A hot pack warns but does not derate (D65, 2026-11-12). Above about 40 degC the BMS lowers its charge-current limit and
runs its fan, and says so in its warning word (bits DERATE and FAN). A non-zero warning word sets MCU bit 11
(BMS_WARN), which is information only: the MCU no longer halves the speed cap, the yaw-rate cap or the motor torque
limit for it. The BMS limits reach the drives only through the torque limiter: its discharge limit (DCL) limits
motoring, its charge limit (CCL) limits regeneration. At the default 45 degC bit 11 stays set for the whole run, but
the speed is not reduced: the reference sample covers 9 rows (repeats counted) in 10 minutes, 509 m at 0.85 m/s on
average, with a peak pack current of 12.9 A, a hottest true cell of 45.2 degC and no BMS fault. Before D65 the same
inputs gave 6 rows and 393 m at 0.65 m/s, because the speed cap was 0.71 m/s instead of 1.0 m/s. (The 30 degC samples,
below the 40 degC limit, ran 4 rows in 5 minutes at 0.85 m/s.) The summary lists bit 11 under the MCU faults and warnings; the BMS warning word
itself is in the log (`bms_telem.Warning_Word`). At 55 degC, the highest ambient, the BMS also sets its
over-temperature warning (above 50 degC), allows no charging current and lowers its discharge limit to 53 to 71 A. A
1-minute fast run there (`rowtest_20261112_135241`) still ended "time up" with no BMS fault, at 0.89 m/s on average
with bit 11 set; the BMS over-temperature fault is at 60 degC.

Fixed settings: flat farm soil with no obstacles (world 7, `row_field`); state of charge 95 % at the start (nominal:
the full-detail pack spreads its cell groups by ±1 % around it, so the summary shows 95.9 % in full detail and 95.0 %
in fast); the pack, the motors, the motor controllers and the electronics enclosure all start at the ambient
temperature, which also drives the environment. The world and the ambient start states are set in
`scripts/scenarios/scn_R1_row_test.m`; the start state of charge is `pk.SOC0` in `params/param_power.m` (see also
`P.scen.soc0` in `params/param_sim.m`); the field, the driver and the run-time figures are in
`params/param_rowtest.m`.

## Start a run

### With the window

```matlab
phase1_row_test
```

A small window asks the five inputs. It is pre-filled with the values of your last start (kept in
`work/rowtest_last.mat`), or the defaults the first time. Under the inputs it shows the estimated wall-clock time, or
in red what is wrong with an input. **Run** checks the inputs again and starts the run; a bad input gets a message
box and the window stays. **Cancel**, or closing the window, starts nothing.

### Without the window (scripted)

```matlab
job = phase1_row_test('Rows', 8, 'Minutes', 20, 'Ambient', 40, 'Sprayer', true);              % full detail
job = phase1_row_test('Rows', 4, 'Minutes', 5, 'Ambient', 30, 'Sprayer', true, 'Fast', true); % fast preset
job = phase1_row_test(..., 'DryRun', true);       % make the folder and run.cmd only; start nothing
job = phase1_row_test(..., 'Background', false);  % run in this MATLAB instead (busy for the whole run)
job = phase1_row_test(..., 'Window', true);       % the window, pre-filled with the inputs given
```

When `'Rows'`, `'Minutes'`, `'Ambient'` or `'Sprayer'` is missing, the window opens. Without the window a missing
`'Fast'` means full detail. `job` holds the run folder (`job.folder`), the inputs (`job.cfg`), the estimate in minutes
(`job.est_min`) and the start command (`job.cmd`); it is `[]` when you cancel the window.

Either way the Command Window prints the start time, the expected end time and the run folder, and your MATLAB stays
free (except with `'Background', false`). The run itself goes on in a second MATLAB, in a minimised window called
**Shaddad row test** on the taskbar. Leave that window alone (see Known limits); it closes by itself at the end. That
MATLAB uses 4 to 5 GB of memory and the project's shared build caches in `work/` (`*.slxc`, `slprj`), so do not
simulate the model in your desktop MATLAB while a row test runs; reading files and plotting old runs is fine.

## Inputs

| Input | Range | Default | Meaning |
|---|---|---|---|
| `Rows` | 2 to 30, whole number | 6 | Number of rows. With one row the way back to row 1 would need a loop turn, so 2 is the least. |
| `Minutes` | 1 to 240 | 10 | Driving time, counted from the moment the robot is armed (MANUAL). |
| `Ambient` | -10 to 55 degC | 45 | Ambient temperature. Every thermal state starts at it. Above about 40 degC the BMS lowers its charge limit, above 45 degC its discharge limit; the MCU applies both in its torque limiter (see What runs). |
| `Sprayer` | true / false | false | Implement 3 (sprayer) mounted, pump on along the rows until the tank is empty. |
| `Fast` | true / false | false | Fast preset. It simplifies three parts: one lumped pack instead of the 13 cell groups, an ideal drive (a fixed efficiency instead of the motor and controller models) and a kinematic chassis (no pitch or roll; quasi-static track loads). It keeps the rest: the BMS, the thermal networks of the pack and the drive (cell, winding and controller temperatures), the electronics and the whole RC chain. Off = full detail: the 13 cell groups, the motor and controller models with their losses, the high-fidelity tracks. |

The defaults and limits are in `params/param_rowtest.m`. A value outside its range is refused with a sentence that
names the input and its range.

## How long it takes

Estimated wall-clock time = 3 min + the minutes / the real-time factor of the case. The window shows it, and the
Command Window prints the expected end time. The real-time factor is simulated time / computing time of the
simulation itself: 0.5 means two minutes of computing for each simulated minute. The 3 min cover everything else:
starting the background MATLAB, compiling the model, and saving the log and the seven pages.

Measured on 2026-10-28 on this PC (16 GB, Normal mode; the desktop MATLAB open, 7.3 to 8.6 GB free before the start):

| Case | Real-time factor measured | Used for the estimate | 5 min run, measured | Estimate: 5 min | 60 min | 240 min |
|---|---|---|---|---|---|---|
| Full detail | 0.55 | 0.5 | | 13 min | 2 h 03 min | 8 h 03 min |
| Full detail + sprayer | 0.51 | 0.5 | 12.6 min | 13 min | 2 h 03 min | 8 h 03 min |
| Fast | 1.20 | 1.1 | | 8 min | 58 min | 3 h 41 min |
| Fast + sprayer | 0.92 | 0.9 | 7.8 min | 9 min | 1 h 10 min | 4 h 30 min |

The cases with the sprayer were measured on the two 5-minute samples, the others on 1-minute runs. The fixed part
took 2 min 17 s (fast) and 2 min 34 s (full detail): 13 s to start the background MATLAB, 1.3 to 1.5 min to compile,
about 8 s to set up the scenario and save the log and the summary, and 40 s to draw and save the seven pages. The
pre-D65 sample (`rowtest_20261028_173747`, full detail, 10 minutes) took 20.9 minutes against its estimate of 23 minutes:
real-time factor 0.54, fixed part 2 min 14 s (14 s to start, 1.5 min to compile, 3 s for the log and the summary, and
28 s for the seven pages of its 10 Hz log). The reference sample took 21 min 52 s (real-time factor 0.49).

**Fast or full detail.** The fast box makes the simulation about twice as fast (0.92 against 0.51 with the sprayer,
1.20 against 0.55 without). It replaces the cell groups by one lumped pack, the motor drive by an ideal one, and the
chassis by a kinematic one (no pitch or roll); it keeps the BMS and the thermal networks, so a fast run still gives
the BMS reading and the cell, winding and controller temperatures. On this flat field the two 5-minute samples agree
closely: 4 rows each, 52.5 Wh against 51.4 Wh from the pack, a largest distance from the row line of 0.10 m against
0.09 m, motor windings at 36.2 degC against 35.2 degC. Use full detail when you study the cells, the drive losses
or the tracks.

**Memory.** The background MATLAB took 4 to 5 GB in the sample runs; the log of a long run adds to that (estimated
up to about 2 GB more for 3 hours). With little free memory Windows starts paging and a run takes two to six times
as long (earlier measurements with 0.7 GB free gave real-time factors of 0.14 to 0.25). Before a long run, close
other heavy programs, or restart your MATLAB if it has run many simulations.

## What you get

One folder per run, `work/reports/rowtest_<yyyymmdd_HHMMSS>/`. Nothing opens at the end; look in the folder.

| File | What it holds |
|---|---|
| `progress.log` | The steps of the background MATLAB with the time of day; the full error report if a step fails. Its `run_scenario R1:` line ends in `ok=1, pass=0`: `ok=1` means the simulation ran without an error, and `pass=0` is normal, since the row test has no pass criteria. |
| `summary.txt` | The summary (below). |
| `summary.mat` | The same figures for MATLAB: `M` (the summary figures), `rows` (the row centre lines), `cfg` (the inputs and settings), `log_file` (the run log). |
| `1_drive` ... `6_nav` `.png` and `.fig` | The six `plot_run` pages: drive, tracks, battery, power, rc, nav (what each shows: `docs/live_rc_drive.md`, "Check a run"). |
| `7_field.png` and `.fig` | The field page (below). |
| `inputs.mat` | The inputs of the run. |
| `run.cmd` | The start file of the background MATLAB (written only for a background run, not with `'Background', false`). |
| `done.txt` | Written last, with the finish time. When it is there, the run is over. |

The `.png` files are for looking. The `.fig` files open in MATLAB (double-click, or `openfig`) to zoom and read
values; the time axes of a reopened page stay linked in time (`XLim`), so zooming one in time zooms the others to the
same time span, while each value axis keeps its own range. Every page title gives the fidelity of the run: the six
`plot_run` pages end with `full detail` or `fast`, the field page says `full fidelity` or `fast fidelity`. The run log
itself stays in `work/results/R1_<yyyymmdd_HHMMSS>.mat`; it is not copied, and `summary.txt` gives its path.

Sizes. Runs longer than about 9.7 minutes log every signal at 10 Hz: the pre-D65 sample (10 minutes) has a run log
of 17 MB and a run folder of 10 MB (its largest page, `3_battery.fig`, is 2.6 MB). That is about 240 MB per simulated
hour before compression, and the log file is 0.43 of it, so a 3-hour run should give a log of about 0.3 GB and a
folder of about 160 MB (scaled from the pre-D65 sample; no run that long has been made yet). Shorter runs keep
every signal at its own rate (up to 1 kHz) and give the largest files per minute: the 5-minute samples have a run log
of 100 MB (full detail) and 83 MB (fast) and a run folder of about 90 MB each, of which the power page `4_power.fig`
alone is 63 MB; their log and folder grow by about 20 MB and 18 MB per simulated minute.

### summary.txt

- the run log, the fidelity and how the run ended;
- the inputs;
- rows completed (repeats counted), how many times the whole field was covered, driving time, distance and mean
  speed;
- energy from the pack (net of regeneration), energy per row (two figures, see below), energy per km, state of
  charge at the start and the end, peak pack current;
- the highest temperatures: the true cells and the BMS reading of them, the motor windings, the motor controllers
  and the enclosure (the MCU's sensor);
- the largest distance from a row line (along the rows only; the headlands and turns are left out);
- every MCU fault and warning bit and every BMS fault bit that was set during the run (bit numbers: `docs/icd.md`);
- simulated time, the wall-clock time of the simulation (compile included; without the start-up and the pages) and
  its real-time factor.

The maxima, the peak current and the fault bits count from the moment the MCU is past its self-test, so the start-up
values do not show. "Cell" is the hottest true cell group (`power_truth.T_cell_max_K`); "Cell (BMS reading)" is what
the BMS measured, with its sensor offset and noise: in the reference sample it read up to 46.7 degC while the true
cells reached 45.2 degC (31.6 against 30.1 degC in the 5-minute samples). Every cell group's true temperature is in
the log (`power_truth.T_cell_true`). "Enclosure (MCU sensor)" is the MCU's own temperature sensor; the true air
temperature in the enclosure is not logged.

"Energy per row" is the mean of the field page's bars: the energy between one completed row and the next (the first
from arming), so it includes the turn into each row, and the row after a return to row 1 also carries that return.
"Energy per row, whole run" divides all the energy from the pack by the rows completed, so it also carries the
start-up before arming and everything after the last completed row (the unfinished pass, the stop): in the
pre-D65 sample 11.25 Wh against 9.29 Wh; in the reference sample 8.94 Wh against 8.85 Wh. With the sprayer, a run that outlasts its tank mixes rows driven with and
without the pump in both figures: the 14-minute example gives 10.91 Wh per row, against about 11.2 Wh before the
tank emptied and 9.1 Wh after; the field page's bars show the step.

### The field page (7_field)

| Tile | What it shows |
|---|---|
| Path on the rows | The rows (green, numbered) and the true path, coloured by time; start and end marked. |
| Rows completed | A step at every completed row, against time. |
| Distance from the row line | The signed distance from the row line along the rows (+ = north), one trace per pass. |
| Energy per row | One bar per completed row, in the order they were completed: the energy since the previous row (the first counts from arming, so each bar includes the turn into its row). |

### How the rows are counted

A row counts when the robot's true path covers at least 95 % of its length (47.5 m of 50 m) in one unbroken stretch
within 0.5 m of the row's centre line. Either direction counts, and every pass counts again. The count is taken from
the true path after the run, so it does not depend on what the driver thinks it did. A row cut short when the time
runs out still counts when at least 95 % of it was covered, and not otherwise. "Field covered" is the count of the
least-driven row.

## Look at a run again

```matlab
folder = fullfile(currentProject().RootFolder, 'work', 'reports', 'rowtest_20261028_173747');   % a run folder
s = load(fullfile(folder, 'summary.mat'));       % M, rows, cfg, log_file
plot_run(s.log_file)                             % the six pages again, from the run log
plot_run(s.log_file, 'Pages', {'battery'}, 'TimeRange', [100 200])
plot_run('R1')                                   % the newest R1 log in work/results
r = load(s.log_file, 'S');                       % the field page again
[~, name] = fileparts(s.log_file);
row_test_field_page(r.S, s.M, s.rows, 'Name', name, 'Cfg', s.cfg);
openfig(fullfile(folder, '7_field.fig'));        % or a saved page, as it was drawn
```

The log does not hold the fidelity, so pages redrawn with `plot_run` give it only when you pass it as a note, for
example `plot_run(s.log_file, 'Note', 'full detail')`; the saved pages have it.

## How a run ends

The line "Ended" of `summary.txt` says why:

| Ended | Meaning |
|---|---|
| `time up` | The minutes were driven; the driver stopped and disarmed the robot. The normal case. |
| `battery at the SoC floor` | The state of charge fell to the floor (5 %) and the simulation stopped there. |
| `MCU <MODE> at <t> s` | The MCU left MANUAL before the time was up (for example `CTRL_STOP`, `ESTOP` or `FAULT` after a battery trip); the driver centred the sticks and the run ended. The fault bits in the summary say why. |
| `MCU never reached MANUAL` | The robot could not be armed; the driver gave up 10 s after the arm switch. |
| `run ended early at <t> s` | The simulation stopped while the MCU was still in MANUAL, for another reason (stop reason given). |
| `simulation error: <message>` | The simulation failed. |

Two more lines can appear at the end of `summary.txt`:

- `error: <message>`: a step before or after the simulation failed (for example saving a page); `progress.log` has
  the full report.
- `error: the background MATLAB ended (exit <code>) before the row test finished`: the background MATLAB itself
  stopped (licence refused, project start-up error, crash, killed). `run.cmd` writes this line and `done.txt`.

## One run at a time

A row test holds the lock `work/rowtest.lock` while it runs. A second start is refused with a message naming the
running folder, until that folder has `done.txt` or the run started more than 3 x its estimate ago. Two runs at once
would not fit in this PC's memory. To start anyway: `phase1_row_test(..., 'Force', true)`.

## Known limits

* Closing the minimised **Shaddad row test** window, or shutting the PC down, kills the run: the background MATLAB
  and its guard end together, so neither `summary.txt` nor `done.txt` is written, and the lock stays until 3 x the
  estimate. Start the next run with `'Force', true` (or delete `work/rowtest.lock`).
* No time limit: nothing stops a slow run. A run always ends in simulated time: the driver ends it a few seconds
  after the time is up (at the latest at its stop time, 2 s + the minutes + 15 s), or it ends earlier for one of the
  reasons under How a run ends; a busy PC only makes it take longer. After 3 x its estimate the lock lapses, and a
  second start is no longer refused.
* `progress.log` is silent while the simulation runs (between "compile and run" and "Summary written").
* All warnings are switched off during the run (`warning('off', 'all')`, as in `phase1_test_drive`, which keeps the
  model's own warnings out: parameter precision loss, counter wrap and similar), so no warning shows in
  `progress.log`. The warning states are put back at the end.
* The visible window and its message box for a bad input have not been tried by an automatic test: try
  `phase1_row_test` once with no arguments.
* Runs shorter than about 9.7 minutes give the largest files per minute (see the sizes above).

## Files

`scripts/phase1_row_test.m` (start), `scripts/phase1_row_run.m` (the background run),
`scripts/scenarios/scn_R1_row_test.m` (scenario R1), `params/param_rowtest.m` (inputs, limits, field, driver, run-time
figures), `utilities/row_driver_step.m` and `row_driver_init.m` (the driver), `utilities/row_test_*.m` (inputs check,
window, rows, row counting, summary, field page, driver parameters), `scripts/plot_run.m` (the six pages),
`params/param_world.m` (world 7), `scripts/build/build_shaddad_system.m` (the RowDriver operator choice, `VC_OPSRC`
4). Tests: `tests/rc/t_row_driver.m` and `tests/rc/t_row_test_system.m`. Design:
`docs/superpowers/specs/2026-10-27-rc-row-test-design.md`.
