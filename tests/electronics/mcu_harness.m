function out = mcu_harness(op, varargin)
%MCU_HARNESS  Temporary unit harnesses for the T08 firmware units (tests/electronics/t_mcu_platform).
%
%   mdl = mcu_harness('build', kind)    create (once per session, not saved) the harness model
%       kind 'rcmap'    rc_dec signals (0.01) -> fw_rc_mapping                       (log: rc)
%            'decchain' SBUS stream (0.001) -> fw_sbus_decoder -> RT 0.01 -> fw_rc_mapping (log: rc_dec, rc)
%            'tilt'     encoder counts (0.01) -> sens_odom_compute -> RT 0.005 -> fw_imu_tilt (+ imu_meas)
%            'adc'      adc_raw / mcu_temp (0.05) -> fw_adc_scaling
%            'can2_<msg>'  bus x (0.01) -> can2_<msg>_pack -> can2_<msg>_unpack -> y (+ frame)
%            'can1'     random commands / feedback through fw_can1_codec and the T04/T05/T06 counterparts
%   out = mcu_harness('sim', kind, T, inputs, P)   run (inputs: Simulink.SimulationData.Dataset or [])
%   ds  = mcu_harness('ds', {name, value}, ...)    Dataset of root inputs in port order; a value is a
%         timeseries, or a struct of timeseries for a bus port
%   All harnesses use ConfigSetRef Shaddad_Ref (fixed step 1 ms, multitasking) and are closed by
%   mcu_harness('close').

switch op
    case 'build'
        out = local_build(varargin{1});
    case 'sim'
        [kind, T, ds] = varargin{1:3};
        P = []; if numel(varargin) >= 4, P = varargin{4}; end
        mdl = local_build(kind);
        in = Simulink.SimulationInput(mdl);
        if ~isempty(ds), in = in.setExternalInput(ds); end
        in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
            'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
        if ~isempty(P), in = in.setVariable('P', P); end
        o = sim(in);
        if ~isempty(o.ErrorMessage), error('mcu_harness:sim', '%s', o.ErrorMessage); end
        out = o;
    case 'ds'
        ds = Simulink.SimulationData.Dataset;
        for k = 1:numel(varargin)
            ds = ds.addElement(varargin{k}{2}, varargin{k}{1});
        end
        out = ds;
    case 'close'
        for k = {'rcmap', 'decchain', 'tilt', 'adc', 'can1', 'can2_comp_cmd', 'can2_mcu_status', 'can2_mcu_imu', ...
                'can2_mcu_odom', 'can2_mcu_power'}
            m = ['tmp_T08_h_' k{1}];
            if bdIsLoaded(m), close_system(m, 0); end
        end
        out = [];
    otherwise
        error('mcu_harness:op', 'unknown op %s', op);
end
end

% ===========================================================================================
function mdl = local_build(kind)
mdl = ['tmp_T08_h_' kind];
if bdIsLoaded(mdl), return, end
new_system(mdl);
shaddad_configsets('attach', mdl);
if startsWith(kind, 'can2_')
    local_can2(mdl, extractAfter(kind, 'can2_'));
else
    feval(['local_' kind], mdl);
end
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch %#ok<CTCH>
end
end

function local_rcmap(m)
IN(m, 'ch_raw', 1, 'uint16', 'Ts.ctrl', '16');
IN(m, 'frame_lost', 2, 'boolean', 'Ts.ctrl', '1');
IN(m, 'failsafe', 3, 'boolean', 'Ts.ctrl', '1');
IN(m, 'last_valid', 4, 'boolean', 'Ts.ctrl', '1');
c = {'ch17', 'false', 'boolean'; 'ch18', 'false', 'boolean'; 'frame_new', 'false', 'boolean'; 'frame_valid', 'false', 'boolean'; ...
    'rc_age', '0', 'single'; 'rc_link_lost', 'false', 'boolean'; 'rc_ok', 'true', 'boolean'; 'lq', '1', 'single'; ...
    'lq_warn', 'false', 'boolean'; 'n_ok', '0', 'uint32'; 'n_bad', '0', 'uint32'; 'n_skip', '0', 'uint32'};
for k = 1:size(c, 1)
    A(m, 'simulink/Sources/Constant', ['C_' c{k, 1}], 'Value', c{k, 2}, 'OutDataTypeStr', c{k, 3}, 'SampleTime', 'Ts.ctrl');
end
el = {'ch_raw', 'ch_raw/1'; 'ch17', 'C_ch17/1'; 'ch18', 'C_ch18/1'; 'frame_lost', 'frame_lost/1'; ...
    'failsafe', 'failsafe/1'; 'frame_new', 'C_frame_new/1'; 'frame_valid', 'C_frame_valid/1'; 'last_valid', 'last_valid/1'; ...
    'rc_age', 'C_rc_age/1'; 'rc_link_lost', 'C_rc_link_lost/1'; 'rc_ok', 'C_rc_ok/1'; 'lq', 'C_lq/1'; ...
    'lq_warn', 'C_lq_warn/1'; 'n_ok', 'C_n_ok/1'; 'n_bad', 'C_n_bad/1'; 'n_skip', 'C_n_skip/1'};
BCV(m, 'RcDec', el);
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Mapping', 'ReferencedSubsystem', 'fw_rc_mapping');
L(m, 'RcDec/1', 'RC_Mapping/1', 'rc_dec');
A(m, 'simulink/Sinks/Terminator', 'T_rc');
L(m, 'RC_Mapping/1', 'T_rc/1', 'rc');
LOG(m, 'RC_Mapping', 1, 'rc');
end

function local_decchain(m)
IN(m, 'bytes', 1, 'uint8', 'Ts.base', '32');
IN(m, 'len', 2, 'uint8', 'Ts.base', '1');
IN(m, 'uart_err', 3, 'boolean', 'Ts.base', '1');
A(m, 'simulink/Signal Routing/Bus Creator', 'Sbus', 'Inputs', '3', 'OutDataTypeStr', 'Bus: Bus_SbusStream', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(m, 'bytes/1', 'Sbus/1'); L(m, 'len/1', 'Sbus/2'); L(m, 'uart_err/1', 'Sbus/3');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Decoder', 'ReferencedSubsystem', 'fw_sbus_decoder');
L(m, 'Sbus/1', 'Decoder/1', 'sbus');
A(m, 'simulink/Signal Attributes/Rate Transition', 'RT', 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', 'Ts.ctrl', 'InitialCondition', '0');
L(m, 'Decoder/1', 'RT/1', 'rc_dec');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Mapping', 'ReferencedSubsystem', 'fw_rc_mapping');
L(m, 'RT/1', 'RC_Mapping/1');
A(m, 'simulink/Sinks/Terminator', 'T_rc');
L(m, 'RC_Mapping/1', 'T_rc/1', 'rc');
LOG(m, 'Decoder', 1, 'rc_dec');
LOG(m, 'RC_Mapping', 1, 'rc');
end

function local_tilt(m)
IN(m, 'enc', 1, 'int32', 'Ts.ctrl', '2');
IN(m, 'wm', 2, 'single', 'Ts.ctrl', '2');
IN(m, 't_us', 3, 'uint32', 'Ts.ctrl', '1');
IN(m, 'fbv', 4, 'boolean', 'Ts.ctrl', '1');
IN(m, 'imu', 5, 'Bus: Bus_ImuMeas', 'Ts.imu', '');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Odom', 'ReferencedSubsystem', 'sens_odom_compute');
nmIn = {'enc', 'wm', 't_us', 'fbv'};
for k = 1:4, L(m, [nmIn{k} '/1'], sprintf('Odom/%d', k)); end
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_v', 'OutputSignals', 'vTrack_LR');
L(m, 'Odom/1', 'Sel_v/1', 'odom');
A(m, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(m, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5');
L(m, 'Sel_v/1', 'V_sum/1'); L(m, 'V_sum/1', 'V_mean/1');
RT(m, 'RT_a', 'Ts.imu'); RT(m, 'RT_v', 'Ts.imu');
L(m, 'Odom/2', 'RT_a/1', 'a_track'); L(m, 'Sel_v/1', 'RT_v/1', 'v_track_LR');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Tilt', 'ReferencedSubsystem', 'fw_imu_tilt');
L(m, 'imu/1', 'Tilt/1', 'imu'); L(m, 'RT_a/1', 'Tilt/2'); L(m, 'RT_v/1', 'Tilt/3');
nm = {'roll', 'pitch', 'tilt_gate', 'yaw_rate_meas', 'gyro_bias'};
for k = 1:5
    A(m, 'simulink/Sinks/Out1', nm{k}, 'Port', num2str(k));
    L(m, sprintf('Tilt/%d', k), [nm{k} '/1'], nm{k});
end
A(m, 'simulink/Sinks/Out1', 'a_track', 'Port', '6'); L(m, 'Odom/2', 'a_track/1');
A(m, 'simulink/Sinks/Out1', 'v_track', 'Port', '7'); L(m, 'V_mean/1', 'v_track/1', 'v_track');
end

function local_adc(m)
IN(m, 'adc_raw', 1, 'uint16', 'Ts.comm', '8');
IN(m, 'mcu_temp', 2, 'single', 'Ts.comm', '1');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'ADC', 'ReferencedSubsystem', 'fw_adc_scaling');
L(m, 'adc_raw/1', 'ADC/1'); L(m, 'mcu_temp/1', 'ADC/2');
nm = {'V_pack', 'V_bus', 'I_pack', 'V24', 'V12', 'V5', 'T_encl', 'lift_pos', 'T_mcu'};
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel', 'OutputSignals', strjoin(nm, ','));
L(m, 'ADC/1', 'Sel/1', 'adc');
for k = 1:numel(nm)
    A(m, 'simulink/Sinks/Out1', nm{k}, 'Port', num2str(k)); L(m, sprintf('Sel/%d', k), [nm{k} '/1']);
end
end

function local_can2(m, msg)
S = struct('comp_cmd', 'Bus_GncCmd', 'mcu_status', 'Bus_McuStatus', 'mcu_imu', 'Bus_ImuMeas', ...
    'mcu_odom', 'Bus_OdomMeas', 'mcu_power', 'Bus_BmsStatus');
bus = S.(msg);
IN(m, 'x', 1, ['Bus: ' bus], 'Ts.ctrl', '');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Pack', 'ReferencedSubsystem', ['can2_' msg '_pack']);
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Unpack', 'ReferencedSubsystem', ['can2_' msg '_unpack']);
L(m, 'x/1', 'Pack/1', 'x');
if strcmp(msg, 'mcu_status')
    IN(m, 't_ms', 2, 'uint32', 'Ts.ctrl', '1');
    L(m, 't_ms/1', 'Pack/2', 't_ms');
end
L(m, 'Pack/1', 'Unpack/1', 'frame');
A(m, 'simulink/Sinks/Out1', 'y', 'Port', '1', 'OutDataTypeStr', ['Bus: ' bus]);
A(m, 'simulink/Sinks/Out1', 'frame', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_CanFrame64');
L(m, 'Unpack/1', 'y/1', 'y'); L(m, 'Pack/1', 'frame/1');
end

function local_can1(m)
% inputs (all random per sample): 1 drive_cmd Bus_DriveCmd 0.01, 2 fb_L / 3 fb_R Bus_DriveSideFb 0.01,
% 4 impl_en boolean 0.05, 5 impl_rst boolean 0.05, 6 impl_rpm single 0.05, 7 req boolean[4] 0.05,
% 8..15 implfb_can_pack inputs (0.05), 16 bms_telem Bus_BMS_Telemetry 0.1, 17 bms_on boolean 0.1
IN(m, 'drive_cmd', 1, 'Bus: Bus_DriveCmd', 'Ts.ctrl', '');
IN(m, 'fb_L', 2, 'Bus: Bus_DriveSideFb', 'Ts.ctrl', '');
IN(m, 'fb_R', 3, 'Bus: Bus_DriveSideFb', 'Ts.ctrl', '');
IN(m, 'impl_en', 4, 'boolean', 'Ts.comm', '1');
IN(m, 'impl_rst', 5, 'boolean', 'Ts.comm', '1');
IN(m, 'impl_rpm', 6, 'single', 'Ts.comm', '1');
IN(m, 'req', 7, 'boolean', 'Ts.comm', '4');
fbn = {'running', 'boolean'; 'fault', 'boolean'; 'ready', 'boolean'; 'impl_type', 'uint8'; 'speed_rpm', 'single'; ...
    'current_A', 'single'; 'temp_degC', 'single'; 'alive', 'uint8'};
for k = 1:8, IN(m, ['ifb_' fbn{k, 1}], 7 + k, fbn{k, 2}, 'Ts.comm', '1'); end
IN(m, 'bms_telem', 16, 'Bus: Bus_BMS_Telemetry', 'Ts.hk', '');
IN(m, 'bms_on', 17, 'boolean', 'Ts.hk', '1');
% ESC feedback frames (drive_fb_pack, T06): FB1/FB3 at 0.01, FB2 through RT 0.01 -> 0.05
for sd = {'L', 1; 'R', 2}'
    A(m, 'simulink/Ports & Subsystems/Subsystem Reference', ['FBP_' sd{1}], 'ReferencedSubsystem', 'drive_fb_pack');
    A(m, 'simulink/Sources/Constant', ['Node_' sd{1}], 'Value', num2str(sd{2}), 'OutDataTypeStr', 'uint8', 'SampleTime', 'Ts.ctrl');
    L(m, ['fb_' sd{1} '/1'], ['FBP_' sd{1} '/1']); L(m, ['Node_' sd{1} '/1'], ['FBP_' sd{1} '/2']);
    RT(m, ['RT_fb2_' sd{1}], 'Ts.comm');
    L(m, ['FBP_' sd{1} '/2'], ['RT_fb2_' sd{1} '/1']);
end
BC(m, 'EscFast', 'Bus_Can1_EscFast', {'FBP_L/1', 'FBP_R/1', 'FBP_L/3', 'FBP_R/3'});
BC(m, 'EscSlow', 'Bus_Can1_EscSlow', {'RT_fb2_L/1', 'RT_fb2_R/1'});
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'IFBP', 'ReferencedSubsystem', 'implfb_can_pack');
for k = 1:8, L(m, ['ifb_' fbn{k, 1} '/1'], sprintf('IFBP/%d', k)); end
BC(m, 'ImplRx', 'Bus_Can1_Impl', {'IFBP/1'});
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'BMSP', 'ReferencedSubsystem', 'bms_can_pack');
L(m, 'bms_telem/1', 'BMSP/1');
% bms_on = false drops every BMS frame (valid := false, data held)
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_bms', 'OutputSignals', 'limits,soc,vit,alarm,cells,state,power');
L(m, 'BMSP/1', 'Sel_bms/1');
slots = {'limits', 'soc', 'vit', 'alarm', 'cells', 'state', 'power'};
for k = 1:7
    A(m, 'simulink/Signal Routing/Bus Assignment', ['Gate_' slots{k}], 'AssignedSignals', 'valid');
    A(m, 'simulink/Signal Routing/Bus Selector', ['V_' slots{k}], 'OutputSignals', 'valid');
    A(m, 'simulink/Logic and Bit Operations/Logical Operator', ['And_' slots{k}], 'Operator', 'AND', 'Inputs', '2');
    L(m, sprintf('Sel_bms/%d', k), ['Gate_' slots{k} '/1']); L(m, sprintf('Sel_bms/%d', k), ['V_' slots{k} '/1']);
    L(m, ['V_' slots{k} '/1'], ['And_' slots{k} '/1']); L(m, 'bms_on/1', ['And_' slots{k} '/2']);
    L(m, ['And_' slots{k} '/1'], ['Gate_' slots{k} '/2']);
end
BC(m, 'BmsRx', 'Bus_Can1_Bms', cellfun(@(s) ['Gate_' s '/1'], slots, 'UniformOutput', false));
% codec under test
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'Codec', 'ReferencedSubsystem', 'fw_can1_codec');
L(m, 'EscFast/1', 'Codec/1'); L(m, 'EscSlow/1', 'Codec/2'); L(m, 'ImplRx/1', 'Codec/3'); L(m, 'BmsRx/1', 'Codec/4');
L(m, 'drive_cmd/1', 'Codec/5'); L(m, 'impl_en/1', 'Codec/6'); L(m, 'impl_rst/1', 'Codec/7');
L(m, 'impl_rpm/1', 'Codec/8'); L(m, 'req/1', 'Codec/9');
% TX frames decoded by the receivers' codecs (T06 drive_cmd_unpack, T05 implcmd_can_unpack, T04 bmsreq_can_unpack)
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_fast', 'OutputSignals', 'drv_cmd_L,drv_cmd_R');
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_slow', 'OutputSignals', 'impl_cmd,bms_req');
L(m, 'Codec/1', 'Sel_fast/1'); L(m, 'Codec/2', 'Sel_slow/1');
for sd = {'L', 1; 'R', 2}'
    A(m, 'simulink/Ports & Subsystems/Subsystem Reference', ['DCU_' sd{1}], 'ReferencedSubsystem', 'drive_cmd_unpack');
    L(m, sprintf('Sel_fast/%d', sd{2}), ['DCU_' sd{1} '/1']); L(m, ['Node_' sd{1} '/1'], ['DCU_' sd{1} '/2']);
end
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'ICU', 'ReferencedSubsystem', 'implcmd_can_unpack');
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', 'BRU', 'ReferencedSubsystem', 'bmsreq_can_unpack');
L(m, 'Sel_slow/1', 'ICU/1'); L(m, 'Sel_slow/2', 'BRU/1');
% outputs
o = {'cmd_L', 'DCU_L/1'; 'cmd_R', 'DCU_R/1'; 'icu_en', 'ICU/1'; 'icu_rst', 'ICU/2'; 'icu_alive', 'ICU/3'; ...
    'icu_rpm', 'ICU/4'; 'req_out', 'BRU/1'; 'drive_fb', 'Codec/3'; 'drv_age', 'Codec/4'; 'bms', 'Codec/6'; ...
    'bms_age', 'Codec/7'};
for k = 1:size(o, 1)
    A(m, 'simulink/Sinks/Out1', o{k, 1}, 'Port', num2str(k)); L(m, o{k, 2}, [o{k, 1} '/1'], o{k, 1});
end
A(m, 'simulink/Sinks/Terminator', 'T_icu_valid'); L(m, 'ICU/5', 'T_icu_valid/1');
A(m, 'simulink/Sinks/Terminator', 'T_bru_valid'); L(m, 'BRU/2', 'T_bru_valid/1');
A(m, 'simulink/Sinks/Terminator', 'T_fv'); L(m, 'Codec/8', 'T_fv/1');
A(m, 'simulink/Sinks/Terminator', 'T_impl'); L(m, 'Codec/5', 'T_impl/1', 'impl');
LOG(m, 'Codec', 5, 'impl');
end

% ---- helpers ------------------------------------------------------------------------------
function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function h = L(sys, src, dst, nm)
h = add_line(sys, src, dst, 'autorouting', 'smart');
if nargin >= 4 && ~isempty(nm) && ~strcmp(get_param(get_param(h, 'SrcBlockHandle'), 'BlockType'), 'BusSelector') ...
        && isempty(get_param(h, 'Name'))
    set_param(h, 'Name', nm);
end
end

function IN(m, name, port, type, ts, dims)
p = A(m, 'simulink/Sources/In1', name, 'Port', num2str(port), 'OutDataTypeStr', type, 'SampleTime', ts);
if ~isempty(dims), set_param(p, 'PortDimensions', dims); end
end

function RT(m, name, ts)
A(m, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', ts, 'InitialCondition', '0');
end

function BC(m, name, bus, srcs)
A(m, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(srcs)), 'OutDataTypeStr', ['Bus: ' bus], ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(srcs), L(m, srcs{k}, sprintf('%s/%d', name, k)); end
end

function BCV(m, name, map)
A(m, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(size(map, 1)));
for k = 1:size(map, 1)
    h = add_line(m, map{k, 2}, sprintf('%s/%d', name, k), 'autorouting', 'smart');
    if isempty(get_param(h, 'Name')), set_param(h, 'Name', map{k, 1}); end
end
end

function LOG(m, blk, port, nm)
ph = get_param([m '/' blk], 'PortHandles');
set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', nm);
end
