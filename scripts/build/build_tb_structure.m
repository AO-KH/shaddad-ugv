function build_tb_structure()
%BUILD_TB_STRUCTURE  Build the STRUCTURE plant bench tests/structure/tb_structure.slx (owner structure, T03).
%
%   Bench = structure_plant (Subsystem Reference) + one Solver Configuration per physical network (left
%   sprocket, right sprocket, implement shaft: local Backward Euler, P.sim.ssc.Ts, fixed cost) + sprocket
%   drivers + implement command + environment sources. Every scenario value comes from the model-workspace
%   struct tbs (override per run with Simulink.SimulationInput.setVariable('tbs', tbs, 'Workspace', 'tb_structure')):
%     tbs.t, tbs.uL, tbs.uR  breakpoints [s] / sprocket commands (rad/s in 'speed', N m in 'torque' mode),
%                            1-D Lookup Tables on the simulation clock (linear, clip)
%     tbs.rref               yaw-rate reference [rad/s] on tbs.t for the yaw-rate PI wrapper (T18b, as the MCU:
%                            Kp tbs.pi_Kp, Ki tbs.pi_Ki, authority +/- tbs.pi_lim, on/off tbs.pi_on, measured
%                            truth.omega_B(3) through a 1 ms Unit Delay), correction dw = tbs.kw*u added
%                            to the right and subtracted from the left sprocket command
%     tbs.t_impl, tbs.w_impl implement shaft speed command (impl_cmd.speedCmd, used by VC_IMPL_DRIVE == 1)
%     tbs.nozzleOn, tbs.liftDuty, tbs.envdist (Bus_EnvDisturbance), tbs.envovr (Bus_EnvOverride), tbs.decim
%   Drivers: Variant Subsystem in label mode, LabelModeActiveChoice 'speed' (Ideal Angular Velocity Source
%   through an S-PS first-order filter P.sim.ssc.tau_filt) or 'torque' (Ideal Torque Source).
%   Outputs (To Workspace, Timeseries, decimation tbs.decim): tb_truth, tb_tracks, tb_impl, tb_env, tb_diag,
%   tb_E_spr (1 ms integral of sum(sprTorque .* sprSpeed), the sprocket input energy).
%   Needs P, Ts, VC_*, Bus_* (shaddad_startup) to compile; built by this script only (rule 3).
%   Layout (model layout spec 6, 7): no block positions here; shaddad_bu('save') lays out every level with
%   shaddad_layout before it saves. Every signal line is named (L5, shaddad_bu('nline')).

name = 'tb_structure';
root = char(currentProject().RootFolder);
f = fullfile(root, 'tests', 'structure', [name '.slx']);
for m = {name}
    if bdIsLoaded(m{1}), close_system(m{1}, 0); end
end
shaddad_bu('fresh', name, f, 'model');
shaddad_configsets('attach', name);
ws = get_param(name, 'ModelWorkspace');
ws.assignin('tbs', tb_structure_defaults());

A = @(lib, blk, varargin) shaddad_bu('add', lib, [name '/' blk], varargin{:});
L = @(src, dst, nm) shaddad_bu('nline', name, src, dst, nm);    % '' -> the source line's name, else the block's
PH = @(b) get_param([name '/' b], 'PortHandles');
SC = sprintf('nesl_utility/Solver\nConfiguration');
RREF = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');

%% ---- plant ----
A('simulink/Ports & Subsystems/Subsystem Reference', 'Plant', 'ReferencedSubsystem', 'structure_plant');

%% ---- sprocket drivers (label-mode variant: 'speed' | 'torque') ----
vs = [name '/Drivers'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
ch = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
ch = setdiff(ch, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(ch), delete_block(ch{k}); end
local_ports(vs);
modes = {'Speed', 'speed', 'fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source', 'rad/s', 'filter'; ...
         'Torque', 'torque', 'fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', 'N*m', 'zero'};
for k = 1:size(modes, 1)
    s = [vs '/' modes{k, 1}];
    add_block('built-in/SubSystem', s, 'VariantControl', modes{k, 2});
    local_ports(s);
    for sd = {'L', 'R'}
        q = sd{1};
        if strcmp(modes{k, 5}, 'filter')
            shaddad_bu('add', 'nesl_utility/Simulink-PS Converter', [s '/SPS_' q], 'Unit', modes{k, 4}, ...
                'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', 'P.sim.ssc.tau_filt');
        else
            shaddad_bu('add', 'nesl_utility/Simulink-PS Converter', [s '/SPS_' q], 'Unit', modes{k, 4}, ...
                'FilteringAndDerivatives', 'zero');
        end
        shaddad_bu('add', modes{k, 3}, [s '/Source_' q]);
        shaddad_bu('add', RREF, [s '/Ref_' q]);
        shaddad_bu('nline', s, ['cmd' q '/1'], ['SPS_' q '/1'], ['cmd' q]);
        src = get_param([s '/Source_' q], 'PortHandles'); sp = get_param([s '/SPS_' q], 'PortHandles');
        rr = get_param([s '/Ref_' q], 'PortHandles'); cpp = get_param([s '/' q], 'PortHandles');
        shaddad_bu('pline', s, cpp.RConn(1), src.LConn(1));     % R -> sprocket
        shaddad_bu('pline', s, src.RConn(2), rr.LConn(1));      % C -> reference
        shaddad_bu('pline', s, sp.RConn(1), src.RConn(1));      % S
    end
end
set_param(vs, 'VariantControlMode', 'label', 'LabelModeActiveChoice', 'speed');

%% ---- sprocket commands + yaw-rate PI wrapper ----
A('simulink/Sources/Clock', 'Clock');
lut = {'ExtrapMethod', 'Clip', 'InterpMethod', 'Linear point-slope'};
A('simulink/Lookup Tables/1-D Lookup Table', 'cmdL_tab', 'BreakpointsForDimension1', 'tbs.t', 'Table', 'tbs.uL', lut{:});
A('simulink/Lookup Tables/1-D Lookup Table', 'cmdR_tab', 'BreakpointsForDimension1', 'tbs.t', 'Table', 'tbs.uR', lut{:});
A('simulink/Lookup Tables/1-D Lookup Table', 'rref_tab', 'BreakpointsForDimension1', 'tbs.t', 'Table', 'tbs.rref', lut{:});
L('Clock/1', 'cmdL_tab/1', 't'); L('Clock/1', 'cmdR_tab/1', ''); L('Clock/1', 'rref_tab/1', '');
A('simulink/Signal Routing/Bus Selector', 'Truth_sel', 'OutputSignals', 'omega_B');
A('simulink/Signal Routing/Selector', 'yaw_rate', 'IndexMode', 'One-based', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '3', 'InputPortWidth', '3');
A('simulink/Discrete/Unit Delay', 'gyro_delay', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
A('simulink/Math Operations/Sum', 'yaw_err', 'Inputs', '+-');
A('simulink/Math Operations/Gain', 'Kp', 'Gain', 'tbs.pi_Kp');
A('simulink/Math Operations/Gain', 'Ki', 'Gain', 'tbs.pi_Ki');
A('simulink/Continuous/Integrator', 'PI_int', 'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'tbs.pi_lim', 'LowerSaturationLimit', '-tbs.pi_lim');
A('simulink/Math Operations/Sum', 'PI_sum', 'Inputs', '++');
A('simulink/Discontinuities/Saturation', 'PI_lim', 'UpperLimit', 'tbs.pi_lim', 'LowerLimit', '-tbs.pi_lim');
A('simulink/Math Operations/Gain', 'PI_on', 'Gain', 'tbs.pi_on*tbs.kw');
A('simulink/Math Operations/Sum', 'cmdL', 'Inputs', '+-');
A('simulink/Math Operations/Sum', 'cmdR', 'Inputs', '++');
L('Plant/1', 'Truth_sel/1', 'truth');
L('Truth_sel/1', 'yaw_rate/1', '');
L('yaw_rate/1', 'gyro_delay/1', 'r_meas');
L('rref_tab/1', 'yaw_err/1', 'r_ref'); L('gyro_delay/1', 'yaw_err/2', 'r_meas_d');
L('yaw_err/1', 'Kp/1', 'e_r'); L('yaw_err/1', 'Ki/1', '');
L('Ki/1', 'PI_int/1', 'ki_e'); L('Kp/1', 'PI_sum/1', 'u_p'); L('PI_int/1', 'PI_sum/2', 'u_i');
L('PI_sum/1', 'PI_lim/1', 'u_pi_raw'); L('PI_lim/1', 'PI_on/1', 'u_pi');
L('cmdL_tab/1', 'cmdL/1', 'cmdL_ol'); L('PI_on/1', 'cmdL/2', 'dw');
L('cmdR_tab/1', 'cmdR/1', 'cmdR_ol'); L('PI_on/1', 'cmdR/2', '');
L('cmdL/1', 'Drivers/1', 'cmdL'); L('cmdR/1', 'Drivers/2', 'cmdR');

%% ---- implement command (Bus_ImplementCmd at Ts.ctrl) ----
A('simulink/Sources/Constant', 'nozzleOn', 'Value', 'tbs.nozzleOn', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.ctrl');
A('simulink/Sources/Constant', 'liftDuty', 'Value', 'tbs.liftDuty', 'SampleTime', 'Ts.ctrl');
A('simulink/Sources/Digital Clock', 'Clock_ctrl', 'SampleTime', 'Ts.ctrl');
A('simulink/Lookup Tables/1-D Lookup Table', 'speedCmd', 'BreakpointsForDimension1', 'tbs.t_impl', 'Table', 'tbs.w_impl', lut{:});
A('simulink/Signal Routing/Bus Creator', 'ImplCmd', 'Inputs', '3', 'OutDataTypeStr', 'Bus: Bus_ImplementCmd', 'NonVirtualBus', 'on');
L('Clock_ctrl/1', 'speedCmd/1', 't_ctrl');
L('nozzleOn/1', 'ImplCmd/1', 'nozzleOn'); L('liftDuty/1', 'ImplCmd/2', 'liftDuty'); L('speedCmd/1', 'ImplCmd/3', 'speedCmd');
L('ImplCmd/1', 'Plant/1', 'impl_cmd');

%% ---- environment ----
A('simulink/Sources/Constant', 'EnvDist', 'Value', 'tbs.envdist', 'OutDataTypeStr', 'Bus: Bus_EnvDisturbance', 'SampleTime', 'Ts.base');
A('simulink/Sources/Constant', 'EnvOvr', 'Value', 'tbs.envovr', 'OutDataTypeStr', 'Bus: Bus_EnvOverride', 'SampleTime', 'Ts.env');
L('EnvDist/1', 'Plant/2', 'env_dist'); L('EnvOvr/1', 'Plant/3', 'env_ovr');

%% ---- outputs ----
tw = {'tb_truth', 1, 'truth'; 'tb_tracks', 2, 'tracks'; 'tb_impl', 3, 'impl_mech'; 'tb_env', 4, 'env'; ...
      'tb_diag', 5, 'diag'};
for k = 1:size(tw, 1)
    A('simulink/Sinks/To Workspace', tw{k, 1}, 'VariableName', tw{k, 1}, 'SaveFormat', 'Timeseries', ...
        'Decimation', 'tbs.decim', 'SampleTime', '-1', 'MaxDataPoints', 'inf');
    L(sprintf('Plant/%d', tw{k, 2}), [tw{k, 1} '/1'], tw{k, 3});
end
A('simulink/Signal Routing/Bus Selector', 'Spr_sel', 'OutputSignals', 'sprTorque,sprSpeed');
A('simulink/Math Operations/Product', 'P_spr', 'Inputs', '**');
A('simulink/Math Operations/Sum of Elements', 'P_spr_sum');
A('simulink/Discrete/Discrete-Time Integrator', 'E_spr', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
A('simulink/Sinks/To Workspace', 'tb_E_spr', 'VariableName', 'tb_E_spr', 'SaveFormat', 'Timeseries', ...
    'Decimation', 'tbs.decim', 'SampleTime', '-1', 'MaxDataPoints', 'inf');
L('Plant/2', 'Spr_sel/1', 'tracks');
L('Spr_sel/1', 'P_spr/1', ''); L('Spr_sel/2', 'P_spr/2', '');
L('P_spr/1', 'P_spr_sum/1', 'P_spr_LR'); L('P_spr_sum/1', 'E_spr/1', 'P_spr'); L('E_spr/1', 'tb_E_spr/1', 'E_spr');

%% ---- physical networks and their Solver Configurations ----
P = @(blk, port) shaddad_bu('pport', [name '/' blk], port);
shaddad_bu('pline', name, P('Drivers', 'L'), P('Plant', 'Spr_L'));
shaddad_bu('pline', name, P('Drivers', 'R'), P('Plant', 'Spr_R'));
scs = {'SC_L', 'Spr_L'; 'SC_R', 'Spr_R'; 'SC_Impl', 'Impl_Shaft'};
for k = 1:size(scs, 1)
    A(SC, scs{k, 1}, 'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
        'LocalSolverSampleTime', 'P.sim.ssc.Ts', 'DoFixedCost', 'on', 'MaxNonlinIter', 'P.sim.ssc.MaxNonlinIter', ...
        'MaxModeIter', 'P.sim.ssc.MaxModeIter');
    sc = PH(scs{k, 1});
    shaddad_bu('pline', name, sc.RConn(1), P('Plant', scs{k, 2}));
end
set_param(name, 'Description', ['tb_structure (T03): bench of structure_plant (HiFi / LoFi, implements) with ideal ' ...
    'sprocket speed or torque drivers, implement drive command, environment overrides and the yaw-rate PI ' ...
    'wrapper of T18b; scenario struct tbs in the model workspace. Built by scripts/build/build_tb_structure.m.']);
shaddad_bu('save', name, f);
try
    pr = currentProject();
    if isempty(pr.findFile(f)), pr.addFile(f); end
catch %#ok<CTCH>
end
close_system(name, 0);
end

function local_ports(s)
shaddad_bu('add', 'simulink/Sources/In1', [s '/cmdL'], 'Port', '1');
shaddad_bu('add', 'simulink/Sources/In1', [s '/cmdR'], 'Port', '2');
shaddad_bu('add', 'nesl_utility/Connection Port', [s '/L'], 'Port', '1', 'Side', 'Right', ...
    'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
shaddad_bu('add', 'nesl_utility/Connection Port', [s '/R'], 'Port', '2', 'Side', 'Right', ...
    'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
end
