function R = param_rowtest()
%PARAM_ROWTEST  RC row test settings: run inputs, field, simulated driver, run time (feature RC row test).
%   R = param_rowtest() -> P.scen.rowtest (numeric only, D58). The reasons for the values are in
%   docs/superpowers/specs/2026-10-27-rc-row-test-design.md (sections 4-9). Tags: [DATA] = taken from an existing
%   source (named on the line); [EST] = assumed or estimated value (a designer's choice, can be changed).

%% ---- run inputs: defaults of the input window (spec section 4) [EST] ----
R.n_rows = 6; R.minutes = 10; R.T_amb_C = 45; R.sprayer = 0; R.fast = 0;   % rows; driving minutes after arming; degC; sprayer 0/1; fast fidelity 0/1

%% ---- field, m (spec sections 5 and 6) [EST] ----
R.L_row = 50; R.spacing = 3; R.headland = 6; R.margin = 5;   % row length; spacing centre to centre; headland beyond each row end; world margin

%% ---- driving, m and m/s (spec sections 5 and 7) [EST] ----
R.v_row = 1.0; R.v_turn = 0.5; R.d_slow = 3; R.L_look = 2.5;   % speed along the rows; speed in the turns; slow-down distance before a row end; look-ahead

%% ---- simulated driver, behaving like a person (spec section 7) [EST] ----
R.t_react = 0.3; R.t_stick = 0.3; R.wobble = 0.02; R.f_wobble = [0.13 0.31];   % reaction delay s; shortest full stick travel s; steering unsteadiness (amplitude, stick units; frequencies Hz)

%% ---- start, end and abort timing, s (spec section 7) [EST] ----
R.t_tx = 0.5; R.t_arm = 2; R.t_arm_wait = 10; R.t_abort = 0.5; R.t_end_hold = 1; R.t_disarm = 2;   % transmitter on and arm = [DATA] scn_rc_tab(0.5, 2) (scripts/scenarios); arming wait, abort, sticks neutral before disarm, disarm = [EST]

%% ---- sprayer tank, L (spec section 5) [EST] ----
% the driver switches the pump off (CH8 OFF) once the tank holds this much or less, as an operator who sees the spray
% stop would, and leaves it off (no refill); 0.1 L is about 1.25 s of spraying at the nozzles' 4.8 L/min
R.tank_empty_L = 0.1;

%% ---- row completed, m and fraction (spec section 9) [EST] ----
R.row_tol = 0.5; R.row_frac = 0.95;   % a row counts when the robot covers row_frac of its length within row_tol of the centre line

%% ---- limits of the input window (spec section 4) [EST] ----
R.n_rows_range = [2 30]; R.minutes_range = [1 240]; R.T_amb_C_range = [-10 55];   % rows (integer); minutes; degC

%% ---- run time estimates: [DATA] measured on this PC 2026-10-28, rounded to the slow side (docs/row_test.md) ----
% measured: rtf (simulated / execution time) 0.55 0.51 1.20 0.92 in the order below; t_compile 154 s (full + implement)
% and 137 s (fast + implement)
R.rtf = [0.5 0.5 1.1 0.9];   % real-time factor: full, full + implement, fast, fast + implement
R.t_compile = 180;           % s, all but the simulated minutes: MATLAB start, compile, saving the log and the pages
end
