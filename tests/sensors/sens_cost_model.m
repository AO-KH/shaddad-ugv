function [w1, w0] = sens_cost_model(P, pose, T, mode)
%SENS_COST_MODEL  Run-phase wall-clock cost of the sensor models in isolation (T-SEN-14, build task T11).
%
%   [wSens, wBase] = sens_cost_model(P, pose, T)            ('all')
%     wBase: temporary model with the six input Constant blocks only (terminated)
%     wSens: the same Constant blocks into Shaddad_SensorModels (outputs into Terminators)
%     The added cost of the sensor model for T simulated seconds is wSens - wBase.
%   [wScan, wBlack] = sens_cost_model(P, pose, T, 'fast2d')
%     sens_lidar_model alone (VC_LIDAR 1) with lidar_blackout false (wScan, full ray casting) and true
%     (wBlack, the MATLAB Function returns before casting): FAST2D cost per scan = (wScan - wBlack)/(T/0.1).
%   [w3, w2] = sens_cost_model(P, pose, T, 'full3d')
%     sens_lidar_model alone with VC_LIDAR 2 (w3, FULL3D, needs sens_uavScene in base for P.world) and
%     VC_LIDAR 1 (w2, FAST2D): FULL3D cost per scan about (w3 - w2)/(T/0.1).
%   pose = [xE yN zU psi] of base_link (level). The inputs are bus-typed Constant blocks at the ICD rates
%   (P.ic values; truth at the pose, RTK corrections OK, 25 C, no faults, no dynamic obstacles), so the
%   measurement contains neither root-input interpolation nor logging, only the sensor blocks themselves.
%   Every configuration runs in Fast Restart (compiled once; 'full3d' recompiles every run, the UAV Scenario
%   blocks do not support Fast Restart); each value is the run-phase time (w(T) - w(1)) T/(T - 1), minimum of
%   three repetitions (shared machine). Normal simulation mode.
%   Temporary models are in memory only (never saved); cs_SimFixed with BlockReduction 'off' and no logging.

if nargin < 4, mode = 'all'; end
names = {'truth', 'drive_truth', 'env', 'elec_env', 'flt', 'obs'};
buses = {'Bus_TruthState', 'Bus_DriveTruth', 'Bus_EnvState', 'Bus_ElecEnv', 'Bus_FaultInject', 'Bus_ObstaclesDyn'};
tsx = {'Ts.base', 'Ts.base', 'Ts.hk', 'Ts.hk', 'Ts.ctrl', 'Ts.ctrl'};
% ---- input values ----
V = struct();
for k = 1:6, V.(names{k}) = P.ic.(buses{k}); end
psi = pose(4);
R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
V.truth.pos_W(:) = pose(1:3);  V.truth.R_WB = R;  V.truth.q_WB(:) = [cos(psi/2) 0 0 sin(psi/2)];
V.truth.eul(:) = [psi 0 0];  V.truth.pose2D(:) = [pose(1:2) psi];  V.truth.lla(:) = P.world.lla0;
V.truth.terrainH = pose(3);
V.elec_env.T_imu_degC = cast(25, 'like', V.elec_env.T_imu_degC);
V.elec_env.gnssCorrOK = cast(1, 'like', V.elec_env.gnssCorrOK);
V.elec_env.gnssCorrAge = cast(1, 'like', V.elec_env.gnssCorrAge);
switch mode
    case 'fast2d'
        cfgs = {1, 0; 1, 1};  use = [1 5 6];  dut = 'sens_lidar_model';   % {VC_LIDAR, blackout}
    case 'full3d'
        cfgs = {2, 0; 1, 0};  use = [1 5 6];  dut = 'sens_lidar_model';
    otherwise
        cfgs = {1, 0; 1, 0};  use = 1:6;  dut = 'Shaddad_SensorModels';
end
w = zeros(1, 2);
for cfg = 1:2
    withDut = ~strcmp(mode, 'all') || cfg == 1;
    v = V;
    v.flt.lidar_blackout = cast(cfgs{cfg, 2}, 'like', v.flt.lidar_blackout);
    m = 'tmp_T11_cost';
    if bdIsLoaded(m), close_system(m, 0); end
    new_system(m);
    c = copy(evalin('base', 'cs_SimFixed'));  c.Name = 'c';
    attachConfigSet(m, c, true);  setActiveConfigSet(m, 'c');
    set_param(m, 'StopTime', num2str(T), 'BlockReduction', 'off', 'SaveOutput', 'off', 'SignalLogging', 'off', ...
        'SaveTime', 'off', 'SaveState', 'off', 'SaveFinalState', 'off', 'ReturnWorkspaceOutputs', 'on');
    for k = use
        add_block('simulink/Sources/Constant', [m '/' names{k}], 'Value', ['tmp_T11_in.' names{k}], ...
            'OutDataTypeStr', ['Bus: ' buses{k}], 'SampleTime', tsx{k});
    end
    if withDut
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [m '/S'], 'ReferencedSubsystem', dut);
        for j = 1:numel(use), add_line(m, [names{use(j)} '/1'], sprintf('S/%d', j)); end
        ph = get_param([m '/S'], 'PortHandles');
        for j = 1:numel(ph.Outport)
            add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', m, j));
            add_line(m, sprintf('S/%d', j), sprintf('T%d/1', j));
        end
    else
        for k = use
            add_block('simulink/Sinks/Terminator', sprintf('%s/T_%s', m, names{k}));
            add_line(m, [names{k} '/1'], sprintf('T_%s/1', names{k}));
        end
    end
    in = Simulink.SimulationInput(m);
    in = in.setVariable('P', P);  in = in.setVariable('VC_LIDAR', cfgs{cfg, 1});
    in = in.setVariable('tmp_T11_in', v);
    % FULL3D: the UAV Scenario blocks do not survive Fast Restart (the scenario handle is consumed by the first
    % run), so that mode recompiles every run (its cost per scan is large against the compile scatter)
    fr = ~strcmp(mode, 'full3d');
    if fr
        try
            set_param(m, 'FastRestart', 'on');
        catch %#ok<CTCH> block without Fast Restart support: every run recompiles
            fr = false;
        end
    end
    try
        sim(in.setModelParameter('StopTime', '0.2'));
    catch e
        if ~strcmp(e.identifier, 'Stateflow:Assertion'), local_close(m, fr); rethrow(e); end
        sim(in.setModelParameter('StopTime', '0.2'));
    end
    tt = zeros(2, 3);
    try
        for r = 1:3
            t0 = tic;  sim(in.setModelParameter('StopTime', '1'));  tt(1, r) = toc(t0);
            t0 = tic;  sim(in.setModelParameter('StopTime', num2str(T)));  tt(2, r) = toc(t0);
        end
    catch e
        local_close(m, fr);
        rethrow(e);
    end
    w(cfg) = (min(tt(2, :)) - min(tt(1, :)))*T/(T - 1);
    local_close(m, fr);
end
w1 = w(1);  w0 = w(2);
end

function local_close(m, fr)
if fr
    try
        set_param(m, 'FastRestart', 'off');
    catch %#ok<CTCH>
    end
end
close_system(m, 0);
end
