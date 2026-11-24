function build_tb_gnc()
%BUILD_TB_GNC  Build the GNC tracking bench tests/gnc/tb_gnc.slx (build task T14, rule 10; gnc spec 8).
%
%   Device under test: Model block 'GNC' -> models/Shaddad_GNC.slx (6 inputs, Normal mode).
%   Plant surrogate (Ts.ctrl = 0.01 s, revision B D41):
%     gnc_cmd.v_cmd / omega_cmd -> Rate Transition 0.05 -> 0.01 (deterministic, 0.05 s = MCU Task_50 ->
%     Task_10) -> Delay P.gnc.tb.n_dead samples (0.05 s; total input dead time 0.1 s) -> MCU AUTO shaper model
%     (speed: a = sat(K_v e, -a_dec_auto, a_acc_auto), jerk-limited j_max; yaw: alpha = sat(K_w_auto e,
%     +/-alpha_auto), jerk-limited jw_auto; Discrete-Time Integrators; P.electronics.shaper) -> first-order
%     lags v (P.gnc.tb.tau_v 0.35 s) and yaw rate (tau_r 0.30 s; the MPC model assumes 0.25 s) ->
%     robotmobilelib/Differential Drive Kinematic Model ('Vehicle Speed & Heading Angular Velocity', track
%     width P.structure.geom.B, initial state P.gnc.tb.x0) + lateral drift P.gnc.tb.drift * [-sin psi, cos psi]
%     integrated (side-slope surrogate).
%   Sensor / MCU / BMS surrogates: Bus_NavState at 0.01 s from the truth plus RTK noise (Random Number,
%     sigma P.gnc.tb.sig_pos / sig_yaw), pitch / roll P.gnc.tb.tilt, quality RTK_FIXED, navValid;
%     Bus_McuStatus at 0.05 s (AUTO switch, link ok, drive_ready, heartbeat, encoder track speeds
%     v -/+ r chi B/2); Bus_BmsStatus SOC P.gnc.tb.soc; perception / mcmd / spray = P.ic constants.
%   Outputs (yout, Dataset): 1 truth [x y psi v r] 0.01 s, 2 gnc_cmd, 3 gnc_debug, 4 gnc_status,
%     5 shaped [v_ref w_ref] 0.01 s.
%   Scenario data come from P (P.gnc.mission, P.gnc.tb.*) and VC_ROW_TRACKER: tests pass them with
%   Simulink.SimulationInput.setVariable (with mpcRowObj), so the shared base workspace is not modified.
%   Configuration: embedded copy 'tb_cfg' of cs_SimFixed (1 ms, ode3: the kinematic model is continuous).
%   Diagram (layout spec 6): saved through shaddad_bu('save'), which lays it out; every signal line is named (L5), through
%   shaddad_bu('nline'): the name given, else the name its source port's line already has, else its source block's name.

name = 'tb_gnc';
root = char(currentProject().RootFolder);
f = fullfile(root, 'tests', 'gnc', [name '.slx']);
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
    shaddad_configsets('fixed');
end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_GncCmd'',''var'')')
    shaddad_params();
end
shaddad_bu('fresh', name, f, 'model');
cs = copy(evalin('base', 'cs_SimFixed'));
cs.Name = 'tb_cfg';
attachConfigSet(name, cs, true);
setActiveConfigSet(name, 'tb_cfg');
try
    detachConfigSet(name, 'Configuration');
catch %#ok<CTCH> default set already gone
end
set_param(name, 'StopTime', '10', 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'SaveTime', 'on', 'SignalLogging', 'off');

A = @(lib, blk, varargin) shaddad_bu('add', lib, [name '/' blk], varargin{:});
L = @(src, dst, nm) shaddad_bu('nline', name, src, dst, nm);
ts = 'Ts.ctrl';

%% ---- device under test ----
A('simulink/Ports & Subsystems/Model', 'GNC', 'ModelName', 'Shaddad_GNC', 'SimulationMode', 'Normal');

%% ---- command path: RT -> dead time -> MCU AUTO shaper -> lags ----
A('simulink/Signal Routing/Bus Selector', 'CmdSel', 'OutputSignals', 'v_cmd,omega_cmd');
L('GNC/1', 'CmdSel/1', '');
for s = {'v', 'w'}
    x = s{1};  k = 1 + strcmp(x, 'w');
    A('simulink/Signal Attributes/Data Type Conversion', ['cmd_' x], 'OutDataTypeStr', 'double');
    L(sprintf('CmdSel/%d', k), ['cmd_' x '/1'], '');
    A('simulink/Signal Attributes/Rate Transition', ['RT_' x], 'Integrity', 'on', 'Deterministic', 'on', ...
        'OutPortSampleTime', ts);
    L(['cmd_' x '/1'], ['RT_' x '/1'], '');
    A('simulink/Discrete/Delay', ['dead_' x], 'DelayLength', 'P.gnc.tb.n_dead', 'InitialCondition', '0', ...
        'SampleTime', ts);
    L(['RT_' x '/1'], ['dead_' x '/1'], ['cmd_' x '_dead']);
end
% speed axis
A('simulink/Math Operations/Sum', 'ev', 'Inputs', '+-');
A('simulink/Math Operations/Gain', 'Kv', 'Gain', 'P.electronics.shaper.K_v');
A('simulink/Discontinuities/Saturation', 'a_sat', 'UpperLimit', 'P.electronics.shaper.a_acc_auto', ...
    'LowerLimit', '-P.electronics.shaper.a_dec_auto');
A('simulink/Discontinuities/Rate Limiter', 'jerk_v', 'RisingSlewLimit', 'P.electronics.shaper.j_max', ...
    'FallingSlewLimit', '-P.electronics.shaper.j_max', 'SampleTimeMode', 'inherited');
A('simulink/Discrete/Discrete-Time Integrator', 'v_ref', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'SampleTime', ts, 'InitialCondition', '0');
L('dead_v/1', 'ev/1', ''); L('v_ref/1', 'ev/2', 'v_ref'); L('ev/1', 'Kv/1', '');
L('Kv/1', 'a_sat/1', ''); L('a_sat/1', 'jerk_v/1', ''); L('jerk_v/1', 'v_ref/1', 'a_ref');
% yaw axis
A('simulink/Math Operations/Sum', 'ew', 'Inputs', '+-');
A('simulink/Math Operations/Gain', 'Kw', 'Gain', 'P.electronics.shaper.K_w_auto');
A('simulink/Discontinuities/Saturation', 'al_sat', 'UpperLimit', 'P.electronics.shaper.alpha_auto', ...
    'LowerLimit', '-P.electronics.shaper.alpha_auto');
A('simulink/Discontinuities/Rate Limiter', 'jerk_w', 'RisingSlewLimit', 'P.electronics.shaper.jw_auto', ...
    'FallingSlewLimit', '-P.electronics.shaper.jw_auto', 'SampleTimeMode', 'inherited');
A('simulink/Discrete/Discrete-Time Integrator', 'w_ref', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'SampleTime', ts, 'InitialCondition', '0');
L('dead_w/1', 'ew/1', ''); L('w_ref/1', 'ew/2', 'w_ref'); L('ew/1', 'Kw/1', '');
L('Kw/1', 'al_sat/1', ''); L('al_sat/1', 'jerk_w/1', ''); L('jerk_w/1', 'w_ref/1', 'alpha_ref');
% closed-loop drive / yaw-rate lags
A('simulink/Discrete/Transfer Fcn First Order', 'lag_v', 'PoleZ', 'exp(-Ts.ctrl/P.gnc.tb.tau_v)');
A('simulink/Discrete/Transfer Fcn First Order', 'lag_r', 'PoleZ', 'exp(-Ts.ctrl/P.gnc.tb.tau_r)');
L('v_ref/1', 'lag_v/1', ''); L('w_ref/1', 'lag_r/1', '');
%% ---- kinematic plant + lateral drift ----
A('robotmobilelib/Differential Drive Kinematic Model', 'Kinematics', 'VehicleInputs', ...
    'Vehicle Speed & Heading Angular Velocity', 'TrackWidth', 'P.structure.geom.B', ...
    'WheelRadius', 'P.structure.r_sprocket', 'InitialState', 'P.gnc.tb.x0(:)', 'SimulateUsing', 'Code generation');
% 0.01 s -> continuous (deterministic slow-to-fast Rate Transition = one 0.01 s sample of the dead time)
A('simulink/Signal Attributes/Rate Transition', 'RT_vc', 'OutPortSampleTime', '0', 'Integrity', 'on', 'Deterministic', 'on');
A('simulink/Signal Attributes/Rate Transition', 'RT_rc', 'OutPortSampleTime', '0', 'Integrity', 'on', 'Deterministic', 'on');
L('lag_v/1', 'RT_vc/1', 'v_true'); L('lag_r/1', 'RT_rc/1', 'r_true');
L('RT_vc/1', 'Kinematics/1', ''); L('RT_rc/1', 'Kinematics/2', '');
A('simulink/Sinks/Terminator', 'T_stateDot'); L('Kinematics/2', 'T_stateDot/1', 'state_dot');
A('simulink/Signal Routing/Demux', 'state', 'Outputs', '3');
L('Kinematics/1', 'state/1', 'state');
A('simulink/Signal Attributes/Rate Transition', 'ZOH_psi', 'OutPortSampleTime', ts, 'Integrity', 'on', 'Deterministic', 'on');
L('state/3', 'ZOH_psi/1', 'psi');
A('simulink/Math Operations/Trigonometric Function', 'sin_psi', 'Operator', 'sin');
A('simulink/Math Operations/Trigonometric Function', 'cos_psi', 'Operator', 'cos');
L('ZOH_psi/1', 'sin_psi/1', ''); L('ZOH_psi/1', 'cos_psi/1', '');
A('simulink/Math Operations/Gain', 'drift_x', 'Gain', '-P.gnc.tb.drift');
A('simulink/Math Operations/Gain', 'drift_y', 'Gain', 'P.gnc.tb.drift');
L('sin_psi/1', 'drift_x/1', ''); L('cos_psi/1', 'drift_y/1', '');
A('simulink/Discrete/Discrete-Time Integrator', 'dx', 'IntegratorMethod', 'Integration: Forward Euler', 'SampleTime', ts);
A('simulink/Discrete/Discrete-Time Integrator', 'dy', 'IntegratorMethod', 'Integration: Forward Euler', 'SampleTime', ts);
L('drift_x/1', 'dx/1', ''); L('drift_y/1', 'dy/1', '');
A('simulink/Signal Attributes/Rate Transition', 'ZOH_x', 'OutPortSampleTime', ts, 'Integrity', 'on', 'Deterministic', 'on');
A('simulink/Signal Attributes/Rate Transition', 'ZOH_y', 'OutPortSampleTime', ts, 'Integrity', 'on', 'Deterministic', 'on');
L('state/1', 'ZOH_x/1', 'x_kin'); L('state/2', 'ZOH_y/1', 'y_kin');
A('simulink/Math Operations/Sum', 'x_true', 'Inputs', '++');
A('simulink/Math Operations/Sum', 'y_true', 'Inputs', '++');
L('ZOH_x/1', 'x_true/1', ''); L('dx/1', 'x_true/2', '');
L('ZOH_y/1', 'y_true/1', ''); L('dy/1', 'y_true/2', '');
% psi wrapped to [-pi, pi)
A('simulink/Math Operations/Bias', 'psi_b', 'Bias', 'pi');
A('simulink/Math Operations/Gain', 'psi_n', 'Gain', '1/(2*pi)');
A('simulink/Math Operations/Rounding Function', 'psi_f', 'Operator', 'floor');
A('simulink/Math Operations/Gain', 'psi_2pi', 'Gain', '2*pi');
A('simulink/Math Operations/Sum', 'psi_w', 'Inputs', '+-');
L('ZOH_psi/1', 'psi_b/1', ''); L('psi_b/1', 'psi_n/1', ''); L('psi_n/1', 'psi_f/1', '');
L('psi_f/1', 'psi_2pi/1', ''); L('ZOH_psi/1', 'psi_w/1', ''); L('psi_2pi/1', 'psi_w/2', '');

%% ---- navigation surrogate (Bus_NavState, 0.01 s) ----
noise = {'n_x', 'P.gnc.tb.sig_pos^2', 'P.gnc.tb.seed_x'; 'n_y', 'P.gnc.tb.sig_pos^2', 'P.gnc.tb.seed_y'; ...
         'n_psi', 'P.gnc.tb.sig_yaw^2', 'P.gnc.tb.seed_psi'};
for k = 1:3
    A('simulink/Sources/Random Number', noise{k, 1}, 'Mean', '0', 'Variance', noise{k, 2}, 'Seed', noise{k, 3}, ...
        'SampleTime', ts);
end
A('simulink/Math Operations/Sum', 'x_meas', 'Inputs', '++');  L('x_true/1', 'x_meas/1', 'x'); L('n_x/1', 'x_meas/2', '');
A('simulink/Math Operations/Sum', 'y_meas', 'Inputs', '++');  L('y_true/1', 'y_meas/1', 'y'); L('n_y/1', 'y_meas/2', '');
A('simulink/Math Operations/Sum', 'psi_meas', 'Inputs', '++'); L('psi_w/1', 'psi_meas/1', 'psi_w'); L('n_psi/1', 'psi_meas/2', '');
A('simulink/Sources/Constant', 'zero', 'Value', '0', 'SampleTime', ts);
A('simulink/Signal Routing/Mux', 'pos_W', 'Inputs', '3');
L('x_meas/1', 'pos_W/1', ''); L('y_meas/1', 'pos_W/2', ''); L('zero/1', 'pos_W/3', '');
A('simulink/Sources/Constant', 'tilt', 'Value', 'P.gnc.tb.tilt(:)', 'SampleTime', ts);
A('simulink/Signal Routing/Mux', 'eul', 'Inputs', '[1 2]');
L('psi_meas/1', 'eul/1', ''); L('tilt/1', 'eul/2', '');
A('simulink/Signal Routing/Mux', 'vel_B', 'Inputs', '3');
L('lag_v/1', 'vel_B/1', ''); L('zero/1', 'vel_B/2', ''); L('zero/1', 'vel_B/3', '');
A('simulink/Sources/Constant', 'posStd', 'Value', '[P.gnc.tb.sig_pos; P.gnc.tb.sig_pos; 2*P.gnc.tb.sig_pos]', 'SampleTime', ts);
A('simulink/Sources/Constant', 'quality', 'Value', 'uint8(NavQuality.RTK_FIXED)', 'OutDataTypeStr', 'uint8', 'SampleTime', ts);
A('simulink/Sources/Constant', 'valid', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', ts);
A('simulink/Sources/Digital Clock', 'clock', 'SampleTime', ts);
A('simulink/Sources/Constant', 'NavIC', 'Value', 'P.ic.Bus_NavState', 'OutDataTypeStr', 'Bus: Bus_NavState', 'SampleTime', ts);
A('simulink/Signal Routing/Bus Assignment', 'NavState', 'AssignedSignals', ...
    't,pos_W,eul,vel_B,yawRate,posStd,quality,navValid,headingValid,aligned,gnssFixType');
L('NavIC/1', 'NavState/1', '');
src = {'clock/1', 'pos_W/1', 'eul/1', 'vel_B/1', 'lag_r/1', 'posStd/1', 'quality/1', 'valid/1', 'valid/1', 'valid/1', 'quality/1'};
for k = 1:numel(src), L(src{k}, sprintf('NavState/%d', k + 1), ''); end
L('NavState/1', 'GNC/1', 'nav_state');

%% ---- MCU status (0.05 s): AUTO, link ok, drive ready, heartbeat, encoder track speeds ----
tsc = 'Ts.gnc';
A('simulink/Math Operations/Gain', 'w_track', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2');
L('lag_r/1', 'w_track/1', '');
A('simulink/Math Operations/Sum', 'vL', 'Inputs', '+-');  L('lag_v/1', 'vL/1', ''); L('w_track/1', 'vL/2', '');
A('simulink/Math Operations/Sum', 'vR', 'Inputs', '++');  L('lag_v/1', 'vR/1', ''); L('w_track/1', 'vR/2', '');
A('simulink/Signal Routing/Mux', 'vLR', 'Inputs', '2');   L('vL/1', 'vLR/1', ''); L('vR/1', 'vLR/2', '');
A('simulink/Signal Attributes/Rate Transition', 'RT_vLR', 'OutPortSampleTime', tsc, 'Integrity', 'on', 'Deterministic', 'on');
L('vLR/1', 'RT_vLR/1', '');
A('simulink/Signal Attributes/Data Type Conversion', 'vLR_s', 'OutDataTypeStr', 'single');
L('RT_vLR/1', 'vLR_s/1', '');
A('simulink/Sources/Counter Free-Running', 'heartbeat', 'NumBits', '8', 'tsamp', tsc);
A('simulink/Sources/Constant', 'sw_auto', 'Value', 'uint8(2)', 'OutDataTypeStr', 'uint8', 'SampleTime', tsc);
A('simulink/Sources/Constant', 'mcu_true', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', tsc);
A('simulink/Sources/Constant', 'McuIC', 'Value', 'P.ic.Bus_McuStatus', 'OutDataTypeStr', 'Bus: Bus_McuStatus', 'SampleTime', tsc);
A('simulink/Signal Routing/Bus Assignment', 'McuStatus', 'AssignedSignals', ...
    'rc_mode_sw,rc_link_ok,drive_ready,power_ready,heartbeat,v_track_meas,valid');
L('McuIC/1', 'McuStatus/1', '');
src = {'sw_auto/1', 'mcu_true/1', 'mcu_true/1', 'mcu_true/1', 'heartbeat/1', 'vLR_s/1', 'mcu_true/1'};
for k = 1:numel(src), L(src{k}, sprintf('McuStatus/%d', k + 1), ''); end
L('McuStatus/1', 'GNC/3', 'mcu_status');

%% ---- BMS, perception, mission command, spray ----
A('simulink/Sources/Constant', 'BmsIC', 'Value', 'P.ic.Bus_BmsStatus', 'OutDataTypeStr', 'Bus: Bus_BmsStatus', 'SampleTime', 'Ts.hk');
A('simulink/Sources/Constant', 'soc', 'Value', 'single(P.gnc.tb.soc)', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.hk');
A('simulink/Sources/Constant', 'bms_true', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.hk');
A('simulink/Signal Routing/Bus Assignment', 'BmsStatus', 'AssignedSignals', 'SOC,valid');
L('BmsIC/1', 'BmsStatus/1', ''); L('soc/1', 'BmsStatus/2', ''); L('bms_true/1', 'BmsStatus/3', '');
L('BmsStatus/1', 'GNC/4', 'bms_status');
A('simulink/Sources/Constant', 'Perception', 'Value', 'P.ic.Bus_Perception', 'OutDataTypeStr', 'Bus: Bus_Perception', ...
    'SampleTime', 'Ts.perception');
L('Perception/1', 'GNC/2', 'perception');
A('simulink/Sources/Constant', 'MissionCmd', 'Value', 'P.ic.Bus_MissionCmd', 'OutDataTypeStr', 'Bus: Bus_MissionCmd', ...
    'SampleTime', 'Ts.mcmd');
L('MissionCmd/1', 'GNC/5', 'mcmd');
A('simulink/Sources/Constant', 'Spray', 'Value', 'P.ic.Bus_SprayDemand', 'OutDataTypeStr', 'Bus: Bus_SprayDemand', ...
    'SampleTime', 'Ts.spray');
L('Spray/1', 'GNC/6', 'spray');

%% ---- outputs ----
A('simulink/Signal Routing/Mux', 'truth_mux', 'Inputs', '5');
L('x_true/1', 'truth_mux/1', ''); L('y_true/1', 'truth_mux/2', ''); L('psi_w/1', 'truth_mux/3', '');
L('lag_v/1', 'truth_mux/4', ''); L('lag_r/1', 'truth_mux/5', '');
A('simulink/Sinks/Out1', 'truth', 'Port', '1');       L('truth_mux/1', 'truth/1', 'truth');
A('simulink/Sinks/Out1', 'gnc_cmd', 'Port', '2');     L('GNC/1', 'gnc_cmd/1', 'gnc_cmd');
A('simulink/Sinks/Out1', 'gnc_debug', 'Port', '3');   L('GNC/4', 'gnc_debug/1', 'gnc_debug');
A('simulink/Sinks/Out1', 'gnc_status', 'Port', '4');  L('GNC/3', 'gnc_status/1', 'gnc_status');
A('simulink/Signal Routing/Mux', 'shaped_mux', 'Inputs', '2');
L('v_ref/1', 'shaped_mux/1', ''); L('w_ref/1', 'shaped_mux/2', '');
A('simulink/Sinks/Out1', 'shaped', 'Port', '5');      L('shaped_mux/1', 'shaped/1', 'shaped');
A('simulink/Sinks/Terminator', 'T_navcmd');           L('GNC/2', 'T_navcmd/1', 'nav_cmd');

set_param(name, 'Description', ['tb_gnc: GNC tracking bench (T14, gnc spec 8, D41). See scripts/build/build_tb_gnc.m. ' ...
    'Scenario data: P.gnc.mission, P.gnc.tb.*, VC_ROW_TRACKER, mpcRowObj via SimulationInput.setVariable.']);
shaddad_bu('save', name, f);
close_system(name, 0);
end
