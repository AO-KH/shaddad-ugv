function out = mcu_ctrl_harness(op, varargin)
%MCU_CTRL_HARNESS  Temporary unit harnesses for the T09 firmware units (tests/electronics/t_mcu_control).
%
%   mdl = mcu_ctrl_harness('build', kind)     create (once per session, not saved) the harness model
%       kind 'sup'   fw10 (+ mode_prev, stop_fail_prev, vtrack_ref_prev) -> fw_safety_supervisor (Ts.ctrl)
%            'ctrl'  fw10 -> Safety_Supervisor -> Mode_Manager -> Vehicle_Ctrl, Unit Delay mode_prev,
%                    wired as Shaddad_MCU/Task_10 (build_shaddad_mcu), except the supervisor inputs 3 and 4
%                    stop_fail_prev / vtrack_ref_prev: open-loop harness inputs here          (Ts.ctrl)
%            'aux'   fw50 -> fw_implement_aux                                           (Ts.comm)
%            'wdg'   alive / fault_inject / fault_word / mode / rearm_req -> fw_watchdog_mgr (Ts.base)
%   r = mcu_ctrl_harness('sim', kind, T, u)       run for T s; u = struct of input overrides (field names =
%       leaf paths with '.' -> '_', e.g. u.rc_sw_mode, u.tilt_pitch, u.bms (struct of bus fields));
%       a value is a constant (row) or a ZOH table [t value...] (rows: time then the value, applies from t);
%       for a bus-typed leaf each field may be a constant or a table. r: .t and one field per output
%       (rows = samples; drive_cmd as a struct of arrays).
%   d = mcu_ctrl_harness('defaults', kind)        nominal (healthy, STANDBY-able) inputs
%   mcu_ctrl_harness('close')                     close the harness models
%   The platform-bus layouts (fw10 / fw50) mirror Shaddad_MCU/Task_10 and Task_50 (T08 Description).

switch op
    case 'build'
        out = local_build(varargin{1});
    case 'defaults'
        out = local_defaults(varargin{1});
    case 'sim'
        [kind, T] = varargin{1:2};
        u = struct(); if numel(varargin) >= 3 && ~isempty(varargin{3}), u = varargin{3}; end
        Pv = []; if numel(varargin) >= 4, Pv = varargin{4}; end
        out = local_sim(kind, T, u, Pv);
    case 'close'
        for k = {'sup', 'ctrl', 'aux', 'wdg'}
            m = ['tmp_T09_h_' k{1}];
            if bdIsLoaded(m), close_system(m, 0); end
        end
        out = [];
    otherwise
        error('mcu_ctrl_harness:op', 'unknown op %s', op);
end
end

% ===========================================================================================
function [spec, ts, outs] = local_spec(kind)
% spec rows: {path, type, dims}; outs rows: {name, src port, type}
switch kind
    case {'sup', 'ctrl'}
        spec = local_fw10();
        ts = 'Ts.ctrl';
        if strcmp(kind, 'sup')
            spec(end+1, :) = {'mode_prev', 'uint8', 1};
            outs = {'fault_word', 'Sup/1', 'uint32'; 'warn_word', 'Sup/2', 'uint32'; 'fault_cls', 'Sup/3', 'boolean'; ...
                'k_der', 'Sup/4', 'single'};
        else
            outs = {'fault_word', 'Sup/1', 'uint32'; 'warn_word', 'Sup/2', 'uint32'; 'fault_cls', 'Sup/3', 'boolean'; ...
                'k_der', 'Sup/4', 'single'; 'mode', 'Mm/1', 'uint8'; 'cmd_src', 'Mm/2', 'uint8'; ...
                'drive_enable', 'Mm/3', 'boolean'; 'rearm_req', 'Mm/4', 'boolean'; 'bms_req', 'Mm/5', 'boolean'; ...
                'drive_cmd', 'Vc/1', 'Bus: Bus_DriveCmd'; 'vc_tel', 'Vc/2', 'single'; 'stop_ctl', 'Mm/6', 'boolean'};
        end
        % supervisor inputs 3 and 4 (D71): stop_ctl(2) and vc_tel(8:9) of the previous sample (Unit Delays in Task_10);
        % open-loop harness inputs here (local_sim: the reference follows the injected track speeds unless set)
        spec(end+1, :) = {'stop_fail_prev', 'boolean', 1};
        spec(end+1, :) = {'vtrack_ref_prev', 'single', 2};
    case 'aux'
        spec = local_fw50();
        ts = 'Ts.comm';
        outs = {'do_aux', 'Ia/1', 'uint32'; 'pwm_duty', 'Ia/2', 'single'; 'impl_enable', 'Ia/3', 'boolean'; ...
            'impl_fault_reset', 'Ia/4', 'boolean'; 'impl_speed_rpm', 'Ia/5', 'single'};
    case 'wdg'
        spec = {'alive', 'uint8', 3; 'fault_inject', 'uint32', 1; 'fault_word', 'uint32', 1; 'mode', 'uint8', 1; ...
            'rearm_req', 'boolean', 1};
        ts = 'Ts.base';
        outs = {'wdg_kick', 'Wd/1', 'boolean'; 'mcu_safety_ok', 'Wd/2', 'boolean'; 'safety_reset_req', 'Wd/3', 'boolean'};
    otherwise
        error('mcu_ctrl_harness:kind', 'unknown kind %s', kind);
end
end

function s = local_rc()
s = {'ch_norm', 'single', 16; 'thr', 'single', 1; 'steer', 'single', 1; 'speed_scale', 'single', 1; 'v_rc', 'single', 1; ...
    'w_rc', 'single', 1; 'sw_mode', 'uint8', 1; 'rc_mode_sw', 'uint8', 1; 'sw_impl', 'uint8', 1; 'sw_lift', 'uint8', 1; ...
    'sw_mission', 'uint8', 1; 'arm', 'boolean', 1; 'arm_req', 'boolean', 1; 'sticks_neutral', 'boolean', 1; ...
    'override', 'boolean', 1; 'estop_remote', 'boolean', 1; 'reset_req', 'boolean', 1; 'lights', 'boolean', 1; ...
    'rc_aux_cmd', 'uint8', 1; 'rc_aux_seq', 'uint8', 1; 'rc_ok', 'boolean', 1; 'rc_link_lost', 'boolean', 1; ...
    'failsafe', 'boolean', 1; 'frame_lost', 'boolean', 1; 'last_valid', 'boolean', 1; 'lq', 'single', 1; ...
    'lq_warn', 'boolean', 1; 'rc_age', 'single', 1; 'n_ok', 'uint32', 1; 'n_bad', 'uint32', 1; 'n_skip', 'uint32', 1};
end

function s = local_adc()
s = {'V_pack', 'single', 1; 'V_bus', 'single', 1; 'I_pack', 'single', 1; 'V24', 'single', 1; 'V12', 'single', 1; ...
    'V5', 'single', 1; 'T_encl', 'single', 1; 'lift_pos', 'single', 1; 'T_mcu', 'single', 1};
end

function s = local_impl()
s = {'running', 'boolean', 1; 'fault', 'boolean', 1; 'ready', 'boolean', 1; 'impl_type', 'uint8', 1; ...
    'speed_rpm', 'single', 1; 'current_A', 'single', 1; 'temp_degC', 'single', 1; 'alive', 'uint8', 1; ...
    'valid', 'boolean', 1; 'age', 'single', 1; 'rx_ok', 'boolean', 1};
end

function s = local_fw10()
pre = @(p, c) [cellfun(@(x) [p '.' x], c(:, 1), 'UniformOutput', false), c(:, 2:3)];
tilt = {'roll', 'single', 1; 'pitch', 'single', 1; 'tilt_gate', 'boolean', 1; 'yaw_rate_meas', 'single', 1; ...
    'gyro_bias', 'single', 1; 'imu_valid', 'boolean', 1};
s = [pre('rc', local_rc()); ...
    {'di_word', 'uint16', 1; 'reset_req', 'boolean', 1; 'reset_evt', 'boolean', 1; 'wdg_reset_seen', 'boolean', 1; ...
     'overrun', 'boolean', 1; 'busoff', 'boolean', 2; 't_us', 'uint32', 1; 'drive_fb', 'Bus: Bus_DriveFb', 1; ...
     'drv_age', 'single', 2; 'odom', 'Bus: Bus_OdomMeas', 1; 'a_track', 'single', 1; 'v_track', 'single', 1}; ...
    pre('tilt', tilt); {'bms', 'Bus: Bus_BmsStatus', 1; 'bms_age', 'single', 1}; pre('adc', local_adc()); ...
    pre('impl', local_impl()); {'gnc_cmd', 'Bus: Bus_GncCmd', 1; 'comp_ok', 'boolean', 1; 'comp_age', 'single', 1}];
end

function s = local_fw50()
pre = @(p, c) [cellfun(@(x) [p '.' x], c(:, 1), 'UniformOutput', false), c(:, 2:3)];
st = [{'mode', 'uint8', 1; 'cmd_src', 'uint8', 1; 'fault_word', 'uint32', 1; 'warn_word', 'uint32', 1; 'k_der', 'single', 1}; ...
    pre('rc', local_rc()); ...
    {'estop_active', 'boolean', 1; 'drive_ready', 'boolean', 1; 'wdg_reset_seen', 'boolean', 1; ...
     'drive_fault_word', 'uint32', 1; 'v_track_LR', 'single', 2; 'v_track', 'single', 1; 'vc_tel', 'single', 11}];
s = [pre('st', st); {'di_word', 'uint16', 1; 'gnc_cmd', 'Bus: Bus_GncCmd', 1; 'comp_ok', 'boolean', 1; ...
    'comp_age', 'single', 1}; pre('adc', local_adc()); pre('impl', local_impl()); {'bms', 'Bus: Bus_BmsStatus', 1}];
end

% ===========================================================================================
function d = local_defaults(kind)
P = evalin('base', 'P');
di = P.electronics.di;
d = struct();
rc = struct('ch_norm', zeros(1, 16), 'thr', 0, 'steer', 0, 'speed_scale', 1, 'v_rc', 0, 'w_rc', 0, 'sw_mode', 1, ...
    'rc_mode_sw', 0, 'sw_impl', 1, 'sw_lift', 2, 'sw_mission', 1, 'arm', 0, 'arm_req', 0, 'sticks_neutral', 1, ...
    'override', 0, 'estop_remote', 0, 'reset_req', 0, 'lights', 0, 'rc_aux_cmd', 0, 'rc_aux_seq', 0, 'rc_ok', 1, ...
    'rc_link_lost', 0, 'failsafe', 0, 'frame_lost', 0, 'last_valid', 1, 'lq', 1, 'lq_warn', 0, 'rc_age', 0, ...
    'n_ok', 0, 'n_bad', 0, 'n_skip', 0);
adc = struct('V_pack', 51.2, 'V_bus', 51.2, 'I_pack', 0, 'V24', 24, 'V12', 12, 'V5', 5, 'T_encl', 318.15, ...
    'lift_pos', 0.5, 'T_mcu', 313.15);
impl = struct('running', 0, 'fault', 0, 'ready', 0, 'impl_type', 0, 'speed_rpm', 0, 'current_A', 0, 'temp_degC', 25, ...
    'alive', 0, 'valid', 0, 'age', 10, 'rx_ok', 0);
bms = P.ic.Bus_BmsStatus;
bms.V_pack = 51.2; bms.DCL = 300; bms.CCL = 100; bms.hv_ready = true; bms.valid = true; bms.SOC = 0.8;
gnc = P.ic.Bus_GncCmd;
di_word = 2^di.estop_ch1_ok + 2^di.estop_ch2_ok + 2^di.safety_ok + 2^di.lid_closed + 2^di.key_on + 2^di.safety_hv_ok;
switch kind
    case {'sup', 'ctrl'}
        fb = P.ic.Bus_DriveFb;
        for sd = {'L', 'R'}
            fb.(sd{1}).state = 2; fb.(sd{1}).tau_avail = P.electronics.drv.T_peak; fb.(sd{1}).rx_ok = true;
            fb.(sd{1}).v_dc = 51.2; fb.(sd{1}).T_motor = 300; fb.(sd{1}).T_esc = 300;
        end
        od = P.ic.Bus_OdomMeas; od.valid = true;
        f = fieldnames(rc); for k = 1:numel(f), d.(['rc_' f{k}]) = rc.(f{k}); end
        d.di_word = di_word; d.reset_req = 0; d.reset_evt = 0; d.wdg_reset_seen = 0; d.overrun = 0; d.busoff = [0 0];
        d.t_us = 0; d.drive_fb = fb; d.drv_age = [0 0]; d.odom = od; d.a_track = 0; d.v_track = 0;
        tl = struct('roll', 0, 'pitch', 0, 'tilt_gate', 1, 'yaw_rate_meas', 0, 'gyro_bias', 0, 'imu_valid', 1);
        f = fieldnames(tl); for k = 1:numel(f), d.(['tilt_' f{k}]) = tl.(f{k}); end
        d.bms = bms; d.bms_age = 0;
        f = fieldnames(adc); for k = 1:numel(f), d.(['adc_' f{k}]) = adc.(f{k}); end
        f = fieldnames(impl); for k = 1:numel(f), d.(['impl_' f{k}]) = impl.(f{k}); end
        d.gnc_cmd = gnc; d.comp_ok = 0; d.comp_age = 10;
        d.stop_fail_prev = 0; d.vtrack_ref_prev = [0 0];     % open loop here; local_sim: the reference follows odom
        if strcmp(kind, 'sup'), d.mode_prev = 2; end
    case 'aux'
        st = struct('mode', 2, 'cmd_src', 0, 'fault_word', 0, 'warn_word', 0, 'k_der', 1);
        f = fieldnames(st); for k = 1:numel(f), d.(['st_' f{k}]) = st.(f{k}); end
        f = fieldnames(rc); for k = 1:numel(f), d.(['st_rc_' f{k}]) = rc.(f{k}); end
        d.st_estop_active = 0; d.st_drive_ready = 0; d.st_wdg_reset_seen = 0; d.st_drive_fault_word = 0;
        d.st_v_track_LR = [0 0]; d.st_v_track = 0; d.st_vc_tel = zeros(1, 11);
        d.di_word = di_word; d.gnc_cmd = gnc; d.comp_ok = 1; d.comp_age = 0;
        f = fieldnames(adc); for k = 1:numel(f), d.(['adc_' f{k}]) = adc.(f{k}); end
        f = fieldnames(impl); for k = 1:numel(f), d.(['impl_' f{k}]) = impl.(f{k}); end
        d.bms = bms;
    case 'wdg'
        d = struct('alive', [0 0 0], 'fault_inject', 0, 'fault_word', 0, 'mode', 2, 'rearm_req', 0);
end
end

% ===========================================================================================
function mdl = local_build(kind)
mdl = ['tmp_T09_h_' kind];
if bdIsLoaded(mdl), return, end
[spec, ts, outs] = local_spec(kind);
nfw = size(local_fw10(), 1);                              % the fw10 leaves come first in spec ('sup', 'ctrl')
new_system(mdl);
shaddad_configsets('attach', mdl);
% root inports (one per leaf), virtual bus tree
for k = 1:size(spec, 1)
    p = [mdl '/' strrep(spec{k, 1}, '.', '_')];
    add_block('simulink/Sources/In1', p, 'Port', num2str(k), 'OutDataTypeStr', spec{k, 2}, 'SampleTime', ts);
    if startsWith(spec{k, 2}, 'Bus:')
        set_param(p, 'BusOutputAsStruct', 'on');
    else
        set_param(p, 'PortDimensions', num2str(spec{k, 3}));
    end
end
switch kind
    case 'sup'
        top = local_tree(mdl, spec(1:nfw, :), 'fw10');
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Sup'], 'ReferencedSubsystem', 'fw_safety_supervisor');
        L(mdl, top, 'Sup/1', 'fw10'); L(mdl, 'mode_prev/1', 'Sup/2', 'mode_prev');
        L(mdl, 'stop_fail_prev/1', 'Sup/3', 'stop_fail_prev'); L(mdl, 'vtrack_ref_prev/1', 'Sup/4', 'vtrack_ref_prev');
    case 'ctrl'
        top = local_tree(mdl, spec(1:nfw, :), 'fw10');
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Sup'], 'ReferencedSubsystem', 'fw_safety_supervisor');
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Mm'], 'ReferencedSubsystem', 'fw_mode_manager');
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Vc'], 'ReferencedSubsystem', 'fw_vehicle_ctrl');
        add_block('simulink/Discrete/Unit Delay', [mdl '/Mode_prev'], 'InitialCondition', '0', 'SampleTime', '-1');
        L(mdl, top, 'Sup/1', 'fw10'); L(mdl, 'Mode_prev/1', 'Sup/2', 'mode_prev');
        L(mdl, 'stop_fail_prev/1', 'Sup/3', 'stop_fail_prev'); L(mdl, 'vtrack_ref_prev/1', 'Sup/4', 'vtrack_ref_prev');
        L(mdl, top, 'Mm/1'); L(mdl, 'Sup/1', 'Mm/2', 'fault_word'); L(mdl, 'Sup/3', 'Mm/3', 'fault_cls');
        L(mdl, 'Mm/1', 'Mode_prev/1', 'mode');
        L(mdl, top, 'Vc/1'); L(mdl, 'Mm/1', 'Vc/2'); L(mdl, 'Mm/2', 'Vc/3', 'cmd_src'); L(mdl, 'Mm/3', 'Vc/4', 'drive_enable');
        L(mdl, 'Sup/1', 'Vc/5'); L(mdl, 'Sup/4', 'Vc/6', 'k_der');
        % stop_ctl [limits_hold stop_fail] (D71): limits_hold to Vc/7 as in Task_10; stop_fail leaves as the output
        % stop_ctl (here the supervisor input stop_fail_prev is a harness input, open loop)
        add_block('simulink/Signal Routing/Demux', [mdl '/Stop_ctl'], 'Outputs', '2');
        add_block('simulink/Sinks/Terminator', [mdl '/T_stop_fail']);
        L(mdl, 'Mm/6', 'Stop_ctl/1', 'stop_ctl'); L(mdl, 'Stop_ctl/1', 'Vc/7', 'limits_hold');
        L(mdl, 'Stop_ctl/2', 'T_stop_fail/1', 'stop_fail');
    case 'aux'
        top = local_tree(mdl, spec, 'fw50');
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Ia'], 'ReferencedSubsystem', 'fw_implement_aux');
        L(mdl, top, 'Ia/1', 'fw50');
    case 'wdg'
        add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Wd'], 'ReferencedSubsystem', 'fw_watchdog_mgr');
        for k = 1:5, L(mdl, [spec{k, 1} '/1'], sprintf('Wd/%d', k), spec{k, 1}); end
end
for k = 1:size(outs, 1)
    add_block('simulink/Sinks/Out1', [mdl '/o_' outs{k, 1}], 'Port', num2str(k), 'OutDataTypeStr', outs{k, 3});
    L(mdl, outs{k, 2}, ['o_' outs{k, 1} '/1']);
end
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch %#ok<CTCH>
end
end

function src = local_tree(mdl, spec, name)
% top-level virtual Bus Creator 'BC_<name>' following the dotted leaf paths; returns 'BC_<name>/1'
heads = cellfun(@(p) strtok(p, '.'), spec(:, 1), 'UniformOutput', false);
u = unique(heads, 'stable');
bc = ['BC_' name];
add_block('simulink/Signal Routing/Bus Creator', [mdl '/' bc], 'Inputs', num2str(numel(u)));
for k = 1:numel(u)
    rows = strcmp(heads, u{k});
    if sum(rows) == 1 && strcmp(spec{find(rows, 1), 1}, u{k})
        s = [u{k} '/1'];                                  % leaf inport named after the element
    else
        sub = spec(rows, :);
        sub(:, 1) = cellfun(@(p) p(numel(u{k}) + 2:end), sub(:, 1), 'UniformOutput', false);
        s = local_subtree(mdl, sub, u{k});
    end
    h = add_line(mdl, s, sprintf('%s/%d', bc, k), 'autorouting', 'smart');
    set_param(h, 'Name', u{k});
end
src = [bc '/1'];
end

function src = local_subtree(mdl, sub, full)
% sub: rows relative to 'full' (e.g. full 'st.rc', sub path 'sw_impl'); inports are named full_path with '_'
heads = cellfun(@(p) strtok(p, '.'), sub(:, 1), 'UniformOutput', false);
u = unique(heads, 'stable');
bc = ['BC_' strrep(full, '.', '_')];
add_block('simulink/Signal Routing/Bus Creator', [mdl '/' bc], 'Inputs', num2str(numel(u)));
for k = 1:numel(u)
    rows = strcmp(heads, u{k});
    if sum(rows) == 1 && strcmp(sub{find(rows, 1), 1}, u{k})
        s = [strrep([full '.' u{k}], '.', '_') '/1'];
    else
        ss = sub(rows, :);
        ss(:, 1) = cellfun(@(p) p(numel(u{k}) + 2:end), ss(:, 1), 'UniformOutput', false);
        s = local_subtree(mdl, ss, [full '.' u{k}]);
    end
    h = add_line(mdl, s, sprintf('%s/%d', bc, k), 'autorouting', 'smart');
    set_param(h, 'Name', u{k});
end
src = [bc '/1'];
end

% ===========================================================================================
function r = local_sim(kind, T, u, Pv)
% u: struct (one run) or cell array of structs (fast-restart batch, T scalar or one per run)
mdl = local_build(kind);
[spec, tsExpr, outs] = local_spec(kind);
Ts = evalin('base', tsExpr);
d = local_defaults(kind);
multi = iscell(u);
if ~multi, u = {u}; end
if isscalar(T), T = repmat(T, 1, numel(u)); end
in = Simulink.SimulationInput.empty;
tv = cell(1, numel(u));
for j = 1:numel(u)
    f = fieldnames(u{j});
    bad = setdiff(f, fieldnames(d));
    assert(isempty(bad), 'mcu_ctrl_harness:field', 'unknown input(s): %s', strjoin(bad, ', '));
    % the track speed reference of the previous sample (supervisor input vtrack_ref_prev, OVERSPEED speed-follow)
    % follows the injected track speeds unless the test sets it: a reference that the tracks follow
    if isfield(d, 'vtrack_ref_prev') && ~isfield(u{j}, 'vtrack_ref_prev') && isfield(u{j}, 'odom') && ...
            isstruct(u{j}.odom) && isfield(u{j}.odom, 'vTrack_LR')
        u{j}.vtrack_ref_prev = u{j}.odom.vTrack_LR;
    end
    t = (0:round(T(j)/Ts))'*Ts;
    tv{j} = t;
    ds = Simulink.SimulationData.Dataset;
    for k = 1:size(spec, 1)
        nm = strrep(spec{k, 1}, '.', '_');
        v = d.(nm);
        if isfield(u{j}, nm), ov = u{j}.(nm); else, ov = []; end
        if startsWith(spec{k, 2}, 'Bus:')
            B = evalin('base', strtrim(extractAfter(spec{k, 2}, 'Bus:')));
            el = local_busval(B, v, ov, t);
        else
            el = timeseries(local_series(v, ov, t, spec{k, 2}, spec{k, 3}), t);
        end
        ds = ds.addElement(el, nm);
    end
    x = Simulink.SimulationInput(mdl);
    x = x.setExternalInput(ds);
    x = x.setModelParameter('StopTime', sprintf('%.6f', T(j)), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
        'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'off');
    if ~isempty(Pv), x = x.setVariable('P', Pv); end
    in(j) = x; %#ok<AGROW>
end
if numel(in) > 1
    o = sim(in, 'UseFastRestart', 'on', 'ShowProgress', 'off', 'ShowSimulationManager', 'off');
else
    o = sim(in);
end
res = cell(1, numel(o));
for j = 1:numel(o)
    if ~isempty(o(j).ErrorMessage), error('mcu_ctrl_harness:sim', '%s', o(j).ErrorMessage); end
    y = o(j).yout;
    r1 = struct('t', tv{j});
    for k = 1:size(outs, 1)
        r1.(outs{k, 1}) = local_flat(y{k}.Values, numel(tv{j}));   % root outports in port order
    end
    res{j} = r1;
end
if multi, r = res; else, r = res{1}; end
end

function x = local_series(v, ov, t, type, dims)
% constant v (row of dims values) unless ov is given: a constant row (dims values) or a ZOH table
% [t value...] (dims + 1 columns; each row applies from its time on, earlier samples keep v)
x = repmat(double(v(:)'), numel(t), 1);
if ~isempty(ov)
    ov = double(ov);
    if numel(ov) == dims
        x = repmat(ov(:)', numel(t), 1);
    elseif size(ov, 2) == dims + 1
        for r = 1:size(ov, 1)
            k = t >= ov(r, 1) - 1e-9;
            x(k, :) = repmat(ov(r, 2:end), sum(k), 1);
        end
    else
        error('mcu_ctrl_harness:value', 'value of size %s for a %d-element input', mat2str(size(ov)), dims);
    end
end
if strcmp(type, 'boolean'), x = x ~= 0; else, x = cast(x, type); end
end

function s = local_busval(B, v, ov, t)
s = struct();
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    if startsWith(e.DataType, 'Bus:')
        Bs = evalin('base', strtrim(extractAfter(e.DataType, 'Bus:')));
        o2 = [];
        if isstruct(ov) && isfield(ov, e.Name), o2 = ov.(e.Name); end
        s.(e.Name) = local_busval(Bs, v.(e.Name), o2, t);
    else
        o2 = [];
        if isstruct(ov) && isfield(ov, e.Name), o2 = ov.(e.Name); end
        dims = prod(e.Dimensions);
        s.(e.Name) = timeseries(local_series(v.(e.Name), o2, t, e.DataType, dims), t);
    end
end
end

function s = local_flat(v, n)
if isstruct(v)
    s = struct();
    f = fieldnames(v);
    for k = 1:numel(f), s.(f{k}) = local_flat(v.(f{k}), n); end
    return
end
D = squeeze(v.Data);
if isvector(D)
    D = D(:);
elseif size(D, 1) ~= n
    D = reshape(D, [], n).';
end
if islogical(D) || isinteger(D), s = D; else, s = double(D); end
end

function h = L(sys, src, dst, nm)
h = add_line(sys, src, dst, 'autorouting', 'smart');
if nargin >= 4 && ~isempty(nm) && ~strcmp(get_param(get_param(h, 'SrcBlockHandle'), 'BlockType'), 'BusSelector') ...
        && isempty(get_param(h, 'Name'))
    set_param(h, 'Name', nm);
end
end
