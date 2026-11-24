function [in, crit] = scn_R1_row_test(~, varargin)
%SCN_R1_ROW_TEST  RC row test (Phase 1): a simulated driver drives the robot by remote control up and down crop rows.
%
%   [in, crit] = scn_R1_row_test(P, 'Rows', n, 'Minutes', m, 'Ambient', T_C, 'Sprayer', tf, 'Fast', tf)
%   (normally called through run_scenario('R1', 'Args', {...}) by phase1_row_run, the worker of phase1_row_test;
%   the P passed in is not used: the scenario builds its own)
%
%   'Rows'      number of rows, a whole number 2 .. 30 (default: param_rowtest, 6)
%   'Minutes'   driving time after the robot is armed, 1 .. 240 min (default 10)
%   'Ambient'   ambient temperature, -10 .. 55 degC (default 45)
%   'Sprayer'   true: implement 3 (sprayer), the pump switched on along the rows until the tank is empty; false
%               (default): no implement
%   'Fast'      true: the fast preset P.sim.endurance (lumped pack, ideal drive, kinematic chassis); false (default):
%               full detail
%   'StopTime'  default t_arm + 60 x minutes + 15 s (the driver ends the run before it, see below)
%   'TankFill'  for tests only (the window and phase1_row_test do not offer it): the sprayer tank's fill at the start,
%               0 .. 1 of its 40 L (default 1 = full, spec section 5), so that a short run can empty the tank
%
%   Spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, sections 5-8. The inputs are checked against the
%   limits of param_rowtest (row_test_inputs). P = shaddad_params for world 7 (row_field) with the implement; the field
%   is rebuilt for these rows (param_world(7, g, R)) and the robot settled at its start P.world.start, half a headland
%   before row 1 heading east (structure_initial_state); P.scen.rowtest = R, P.scen.rowdrv = row_test_prm(P, R) (the
%   RowDriver parameters). Every thermal state starts at the ambient temperature (pack, drive, electronics enclosure),
%   which also drives the environment (P.world.env.T_amb and ScenarioIn env.use_T / T_amb, as S3). The robot is driven
%   by the RowDriver operator choice (VC_OPSRC 4) through the whole RC chain, without the 3-D view (VC_VIZ 0): the
%   driver arms at t_arm, drives the rows for the minutes, stops and disarms the robot and ends the run (Stop
%   Simulation, a few seconds after the time is up); it ends it earlier when the MCU leaves MANUAL. Logging as every
%   scenario (scn_logging: every signal at 10 Hz when the stop time is above P.log.to_file_threshold). A free test:
%   no pass / fail criteria (crit = []).

R = param_rowtest();
ip = inputParser;
ip.addParameter('Rows', R.n_rows);
ip.addParameter('Minutes', R.minutes);
ip.addParameter('Ambient', R.T_amb_C);
ip.addParameter('Sprayer', R.sprayer);
ip.addParameter('Fast', R.fast);
ip.addParameter('StopTime', []);
ip.addParameter('TankFill', 1, @(x) isnumeric(x) && isscalar(x) && isreal(x) && x >= 0 && x <= 1);
ip.parse(varargin{:});
o = ip.Results;

%% ---- inputs -> R (numbers, D58) ----
cfg = R;
cfg.n_rows = o.Rows; cfg.minutes = o.Minutes; cfg.T_amb_C = o.Ambient; cfg.sprayer = o.Sprayer; cfg.fast = o.Fast;
[ok, msg, R] = row_test_inputs(cfg, R);
if ~ok, error('scn_R1:inputs', '%s', msg); end

%% ---- parameters: the field, the driver, the start pose ----
P = shaddad_params('World', 7, 'Implement', 3*R.sprayer, 'Phase', 1, 'Assign', false);
P.world = param_world(7, P.frames.g, R);
if o.TankFill ~= 1
    % tests only, a part-filled tank: the structure parameters again from the same configuration with that fill (the
    % sprayer's fill0, the mass properties m0 / rcg0, the static track loads); the MCU keeps its parameters (its
    % firmware is set for the implement with a full tank)
    c = P.structure.cfg;
    c.tankFill0 = o.TankFill;
    P.structure = param_structure(c, P.world, P.frames.g);
end
P.scen.rowtest = R;
P.scen.rowdrv = row_test_prm(P, R);
P.structure.init = structure_initial_state(P);       % settled chassis state at P.world.start of this field

%% ---- ambient temperature: the environment and every thermal start state ----
T = R.T_amb_C + 273.15;
P.power.thermal.T_init_K = T;                         % pack cells and pack enclosure
P.drive.thermal.T_amb0 = T;                           % motor windings and housings, motor controllers
P.electronics.therm.T0 = T;                           % electronics enclosure
P.world.env.T_amb = T;
tab = struct();
tab.env.use_T = [0 1];
tab.env.T_amb = [0 T];

%% ---- variant controls, stop time, input ----
VC = struct('VC_PHASE', 1, 'VC_IMPL', 3*R.sprayer, 'VC_VIZ', 0, 'VC_OPSRC', 4);
if R.fast
    E = P.sim.endurance;
    for f = {'VC_PACK', 'VC_DRIVE_FID', 'VC_STRUCT_FID'}
        VC.(f{1}) = E.(f{1});
    end
end
T_stop = o.StopTime;
if isempty(T_stop), T_stop = R.t_arm + 60*R.minutes + 15; end
P = scn_integration_overrides(P);
in = scn_base_input(P, 'R1', 'StopTime', T_stop, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T_stop);
crit = [];                                   % free test: no pass/fail criteria
end
