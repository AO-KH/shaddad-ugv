function build_shaddad_mcu(what)
%BUILD_SHADDAD_MCU  Build models/Shaddad_MCU.slx, the STM32H743 firmware model (owner: electronics, T08).
%
%   build_shaddad_mcu()          units (build_can2_codecs, build_fw_platform) + model
%   build_shaddad_mcu('model')   only the model (the Subsystem Reference units must exist)
%
%   Root ports: docs/icd.md 3.1 MCU row (in1k..in100 Bus_McuIn*, out1k..out100 Bus_McuOut*, telem
%   Bus_FwTelemetry), ConfigSetRef Shaddad_Ref. Structure (build plan T08.1):
%     IO_In / IO_Out    Variant Subsystems (VC_TARGET: 0 SIM fw_io_in_sim / fw_io_out_sim, 1 HW *_hw),
%                       VariantActivationTime 'update diagram'
%     Task_1k  (0.001)  fw_sbus_decoder, fw_watchdog_mgr, DO word, reset counter, ms tick
%     Task_5   (0.005)  sens_acq_imu (T11), fw_imu_tilt
%     Task_10  (0.01)   fw_rc_mapping, sens_odom_compute (T11), fw_safety_supervisor, fw_mode_manager,
%                       fw_vehicle_ctrl, fw_telemetry, platform bus fw10, status bus st10, alive counter
%     Task_50  (0.05)   fw_adc_scaling, fw_implement_aux, MCU_STATUS assembly, alive counter
%     Task_100 (0.1)    alive counter
%     CAN1_Codec / CAN2_Codec   multirate codec layers (fw_can1_codec, fw_can2_codec)
%   Every inter-task signal passes an explicit Rate Transition (Integrity, Deterministic) at the top
%   level: the signals into Task_1k, Task_5, Task_10 and Task_50 in the plain (virtual) subsystems RT_to_Task1k,
%   RT_to_Task5, RT_to_Task10 and RT_to_Task50, each Rate Transition in its own path, which hand their task one
%   virtual bus (rt1k, rt5, rt10, rt50; layout spec 6.3), the other four (RT_fi_100, RT_t_ms, RT_bms_req, RT_pwm)
%   at the root. Requires P, Ts, VC_* and Bus_* in the base workspace (shaddad_params).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of the file
%   with shaddad_layout('Shaddad_MCU', 'Recursive', true): the root, the IO_In / IO_Out shells (their SIM and HW
%   choices stacked, L8), the four Rate Transition groups and the five tasks. Every signal line is named (L5): a line
%   keeps the first name a call gives it, and local_name_rest names every line still without a name after its source
%   block ('_k' for output k of a block with several outputs, as shaddad_bu('nline')); a Bus Selector's output keeps
%   '<element>'. Inside the tasks, one pick right in front of each consumer of a bus (spec 6.2): Task_10 takes rt10
%   through an In Bus Element port (a pick block rt10_<element> per consumer; its Bus Selectors on a classic port
%   gave line overlaps with the drive_fb picks), the other tasks take their bus through a classic port with one Bus
%   Selector per consumer (one pick feeds a pair of like compares or a run of consecutive inputs); the lines into the
%   bus-typed Status_BC of Task_50 are named after their elements.

if nargin < 1, what = 'all'; end
if strcmpi(what, 'all')
    build_can2_codecs();
    build_fw_platform();
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
mdl = 'Shaddad_MCU';
f = fullfile(root, 'models', [mdl '.slx']);
for m = {'Shaddad_System'}            % a loaded parent would keep the old model locked
    if bdIsLoaded(m{1}) && strcmp(get_param(m{1}, 'Dirty'), 'off'), close_system(m{1}, 0); end
end
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
set_param(mdl, 'Description', local_desc());

%% ---- root ports ----
ins = {'in1k', 'McuIn1k', 'Ts.base'; 'in5', 'McuIn5', 'Ts.imu'; 'in10', 'McuIn10', 'Ts.ctrl'; ...
    'in50', 'McuIn50', 'Ts.comm'; 'in100', 'McuIn100', 'Ts.hk'};
for k = 1:size(ins, 1)
    A(mdl, 'simulink/Sources/In1', ins{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ['Bus: Bus_' ins{k, 2}], ...
        'SampleTime', ins{k, 3}, 'BusOutputAsStruct', 'on');
end
outs = {'out1k', 'McuOut1k', 'Ts.base'; 'out5', 'McuOut5', 'Ts.imu'; 'out10', 'McuOut10', 'Ts.ctrl'; ...
    'out50', 'McuOut50', 'Ts.comm'; 'out100', 'McuOut100', 'Ts.hk'; 'telem', 'FwTelemetry', 'Ts.ctrl'};
for k = 1:size(outs, 1)
    A(mdl, 'simulink/Sinks/Out1', outs{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ['Bus: Bus_' outs{k, 2}], ...
        'SampleTime', outs{k, 3}, 'BusOutputAsStruct', 'on');
end

%% ---- IO variants ----
ioIn = {'in1k', 'in5', 'in10', 'in50', 'in100'};
ioInOut = {'sbus', 'di_word', 't_us', 'reset_flag', 'fault_inject', 'imu', 'can1_esc', 'busoff', ...
    'can1_escslow', 'can1_impl', 'can2_comp', 'adc_raw', 'mcu_temp_degC', 'can1_bms'};
local_variant([mdl '/IO_In'], ioIn, ioInOut, 'fw_io_in_sim', 'fw_io_in_hw');
ioOutIn = {'do_word', 'wdg_kick', 'can2_imu', 'can1_drv', 'can2_odom', 'pwm_duty', 'can1_slow', 'can2_status', 'can2_power'};
ioOut = {'out1k', 'out5', 'out10', 'out50', 'out100'};
local_variant([mdl '/IO_Out'], ioOutIn, ioOut, 'fw_io_out_sim', 'fw_io_out_hw');
for k = 1:numel(ioIn), L(mdl, [ioIn{k} '/1'], sprintf('IO_In/%d', k), ioIn{k}); end
for k = 1:numel(ioOut), L(mdl, sprintf('IO_Out/%d', k), [ioOut{k} '/1'], ioOut{k}); end
io = @(nm) sprintf('IO_In/%d', find(strcmp(ioInOut, nm)));
oo = @(nm) sprintf('IO_Out/%d', find(strcmp(ioOutIn, nm)));

%% ---- tasks ----
local_task_1k(mdl); local_task_5(mdl); local_task_10(mdl); local_task_50(mdl); local_task_100(mdl);
SSR(mdl, 'CAN1_Codec', 'fw_can1_codec');
SSR(mdl, 'CAN2_Codec', 'fw_can2_codec');

%% ---- IO_In -> tasks / codecs ----
L(mdl, io('sbus'), 'Task_1k/1', 'sbus');
L(mdl, io('di_word'), 'Task_1k/2', 'di_word');
L(mdl, io('t_us'), 'Task_1k/3', 't_us');
L(mdl, io('reset_flag'), 'Task_1k/4', 'reset_flag');
L(mdl, io('fault_inject'), 'Task_1k/5', 'fault_inject');
RTL(mdl, 'RT_fi_100', 'Ts.hk', '0', io('fault_inject'), 'Task_100/1');
L(mdl, io('imu'), 'Task_5/1', 'imu');
L(mdl, io('can1_esc'), 'CAN1_Codec/1', 'can1_esc');
L(mdl, io('busoff'), 'Task_10/2', 'busoff');
L(mdl, io('can1_escslow'), 'CAN1_Codec/2', 'can1_escslow');
L(mdl, io('can1_impl'), 'CAN1_Codec/3', 'can1_impl');
L(mdl, io('can1_bms'), 'CAN1_Codec/4', 'can1_bms');
L(mdl, io('can2_comp'), 'CAN2_Codec/1', 'can2_comp');
L(mdl, io('adc_raw'), 'Task_50/1', 'adc_raw');
L(mdl, io('mcu_temp_degC'), 'Task_50/2', 'mcu_temp_degC');

%% ---- task and codec outputs, and the Rate Transitions that serve a consumer of fewer than 3 such signals ----
L(mdl, 'Task_1k/2', oo('do_word'), 'do_word');
L(mdl, 'Task_1k/3', oo('wdg_kick'), 'wdg_kick');
RTL(mdl, 'RT_t_ms', 'Ts.comm', '0', 'Task_1k/5', 'CAN2_Codec/5', 't_ms');
L(mdl, 'Task_5/1', 'CAN2_Codec/2', 'imu_meas');
L(mdl, 'Task_10/1', 'CAN1_Codec/5', 'drive_cmd');
L(mdl, 'Task_10/2', 'CAN2_Codec/3', 'odom');
RTL(mdl, 'RT_bms_req', 'Ts.comm', '0', 'Task_10/5', 'CAN1_Codec/9', 'bms_req');
L(mdl, 'Task_10/11', 'telem/1', 'telem');
RTL(mdl, 'RT_pwm', 'Ts.ctrl', '0', 'Task_50/2', oo('pwm_duty'), 'pwm_duty');
L(mdl, 'Task_50/3', 'CAN1_Codec/6', 'impl_enable');
L(mdl, 'Task_50/4', 'CAN1_Codec/7', 'impl_fault_reset');
L(mdl, 'Task_50/5', 'CAN1_Codec/8', 'impl_speed_rpm');
L(mdl, 'Task_50/6', 'CAN2_Codec/4', 'mcu_status');
L(mdl, 'CAN1_Codec/1', oo('can1_drv'), 'can1_drv');
L(mdl, 'CAN1_Codec/2', oo('can1_slow'), 'can1_slow');
L(mdl, 'CAN1_Codec/3', 'Task_10/3', 'drive_fb');
L(mdl, 'CAN1_Codec/4', 'Task_10/4', 'drv_age');
L(mdl, 'CAN1_Codec/5', 'Task_50/7', 'impl');
L(mdl, 'CAN1_Codec/6', 'CAN2_Codec/6', 'bms');
A(mdl, 'simulink/Sinks/Terminator', 'T_bms_frame_valid');
L(mdl, 'CAN1_Codec/8', 'T_bms_frame_valid/1', 'bms_frame_valid');
L(mdl, 'CAN2_Codec/1', oo('can2_imu'), 'can2_imu');
L(mdl, 'CAN2_Codec/2', oo('can2_odom'), 'can2_odom');
L(mdl, 'CAN2_Codec/3', oo('can2_status'), 'can2_status');
L(mdl, 'CAN2_Codec/4', oo('can2_power'), 'can2_power');
L(mdl, 'CAN2_Codec/5', 'Task_50/4', 'gnc_cmd');
L(mdl, 'CAN2_Codec/6', 'Task_50/5', 'comp_ok');
L(mdl, 'CAN2_Codec/7', 'Task_50/6', 'comp_age');

%% ---- the Rate Transitions into each task that receives 3 or more signals from other rates (decision 11) ----
% rows: element of the task's bus, Rate Transition (name, output rate, initial condition), the type the task's own
% port declared, source, line name at the root
local_rt_group(mdl, 'RT_to_Task1k', 'rt1k', { ...
    'alive10',      'RT_alive10',       'Ts.base', '0',                  'uint8',              'Task_10/10',         'alive10'
    'alive50',      'RT_alive50',       'Ts.base', '0',                  'uint8',              'Task_50/7',          'alive50'
    'alive100',     'RT_alive100',      'Ts.base', '0',                  'uint8',              'Task_100/1',         'alive100'
    'fault_word',   'RT_fault_word_1k', 'Ts.base', '0',                  'uint32',             'Task_10/8',          'fault_word'
    'mode',         'RT_mode_1k',       'Ts.base', '0',                  'uint8',              'Task_10/7',          'mode'
    'rearm_req',    'RT_rearm_1k',      'Ts.base', '0',                  'boolean',            'Task_10/9',          'rearm_req'
    'do_aux',       'RT_do_aux',        'Ts.base', '0',                  'uint32',             'Task_50/1',          'do_aux'});
local_rt_group(mdl, 'RT_to_Task5', 'rt5', { ...
    't_us',         'RT_t_us_5',        'Ts.imu',  '0',                  'uint32',             io('t_us'),           't_us'
    'fault_inject', 'RT_fi_5',          'Ts.imu',  '0',                  'uint32',             io('fault_inject'),   'fault_inject'
    'a_track',      'RT_a_track',       'Ts.imu',  '0',                  'single',             'Task_10/3',          'a_track'
    'v_track_LR',   'RT_v_track_LR',    'Ts.imu',  '0',                  'single',             'Task_10/4',          'v_track_LR'});
local_rt_group(mdl, 'RT_to_Task10', 'rt10', { ...
    'rc_dec',       'RT_rc_dec',        'Ts.ctrl', '0',                  '',                   'Task_1k/1',          'rc_dec'
    'di_word',      'RT_di_10',         'Ts.ctrl', '0',                  'uint16',             io('di_word'),        'di_word'
    't_us',         'RT_t_us_10',       'Ts.ctrl', '0',                  'uint32',             io('t_us'),           't_us'
    'fault_inject', 'RT_fi_10',         'Ts.ctrl', '0',                  'uint32',             io('fault_inject'),   'fault_inject'
    'reset_cnt',    'RT_reset_cnt',     'Ts.ctrl', '0',                  'uint8',              'Task_1k/4',          'reset_cnt'
    'tilt',         'RT_tilt',          'Ts.ctrl', '0',                  '',                   'Task_5/2',           'tilt'
    'bms',          'RT_bms_10',        'Ts.ctrl', 'P.ic.Bus_BmsStatus', 'Bus: Bus_BmsStatus', 'CAN1_Codec/6',       'bms'
    'bms_age',      'RT_bms_age',       'Ts.ctrl', '0',                  'single',             'CAN1_Codec/7',       'bms_age'
    'adc',          'RT_adc',           'Ts.ctrl', '0',                  '',                   'Task_50/8',          'adc'
    'impl',         'RT_impl',          'Ts.ctrl', '0',                  '',                   'CAN1_Codec/5',       'impl'
    'gnc_cmd',      'RT_gnc_cmd',       'Ts.ctrl', 'P.ic.Bus_GncCmd',    'Bus: Bus_GncCmd',    'CAN2_Codec/5',       'gnc_cmd'
    'comp_ok',      'RT_comp_ok',       'Ts.ctrl', '0',                  'boolean',            'CAN2_Codec/6',       'comp_ok'
    'comp_age',     'RT_comp_age',      'Ts.ctrl', '0',                  'single',             'CAN2_Codec/7',       'comp_age'});
local_rt_group(mdl, 'RT_to_Task50', 'rt50', { ...
    'st',           'RT_st10',          'Ts.comm', '0',                  '',                   'Task_10/6',          'st10'
    'di_word',      'RT_di_50',         'Ts.comm', '0',                  'uint16',             io('di_word'),        'di_word'
    'fault_inject', 'RT_fi_50',         'Ts.comm', '0',                  'uint32',             io('fault_inject'),   'fault_inject'
    'bms',          'RT_bms_50',        'Ts.comm', 'P.ic.Bus_BmsStatus', 'Bus: Bus_BmsStatus', 'CAN1_Codec/6',       'bms'});
L(mdl, 'RT_to_Task1k/1', 'Task_1k/6', 'rt1k');
L(mdl, 'RT_to_Task5/1', 'Task_5/2', 'rt5');
L(mdl, 'RT_to_Task10/1', 'Task_10/1', 'rt10');
L(mdl, 'RT_to_Task50/1', 'Task_50/3', 'rt50');

%% ---- logging for the benches (no effect on generated code without MAT-file logging) ----
LOG(mdl, 'Task_1k', 1, 'rc_dec');
LOG(mdl, 'IO_In', find(strcmp(ioInOut, 'busoff')), 'busoff');
LOG(mdl, 'Task_5', 2, 'tilt');
LOG([mdl '/Task_10'], 'RC_Mapping', 1, 'rc');

local_name_rest(mdl);
shaddad_bu('save', mdl, f);
shaddad_bu('finalize', mdl, f);
close_system(mdl, 0);
end

% ===========================================================================================
function local_variant(s, ins, outs, simRef, hwRef)
add_block('simulink/Ports & Subsystems/Variant Subsystem', s);
ch = find_system(s, 'MatchFilter', @Simulink.match.allVariants, 'SearchDepth', 1, 'LookUnderMasks', 'all');
ch = setdiff(ch, {s});
for k = 1:numel(ch)
    if bdIsLoaded(bdroot(s)) && getSimulinkBlockHandle(ch{k}) > 0, delete_block(ch{k}); end
end
for k = 1:numel(ins), add_block('simulink/Sources/In1', [s '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [s '/' outs{k}], 'Port', num2str(k)); end
add_block('simulink/Ports & Subsystems/Subsystem Reference', [s '/SIM'], 'ReferencedSubsystem', simRef, ...
    'VariantControl', 'VC_TARGET == 0');
add_block('simulink/Ports & Subsystems/Subsystem Reference', [s '/HW'], 'ReferencedSubsystem', hwRef, ...
    'VariantControl', 'VC_TARGET == 1');
set_param(s, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

% ===========================================================================================
function local_rt_group(mdl, name, bus, rows)
% the Rate Transitions of the signals one task receives from other rates, stacked in the plain (virtual) subsystem
% name in front of it (decision 11): input k, a classic port named rows{k,7} with the type rows{k,5} that the task's
% own port declared, passes its own Rate Transition rows{k,2} (output rate rows{k,3}, initial condition rows{k,4},
% Integrity and Deterministic on) to the element rows{k,1} of the Out Bus Element port bus, so the task gets them as
% one virtual bus (spec 6.3); the root line from rows{k,6} into input k is named rows{k,7}
s = [mdl '/' name];
add_block('built-in/SubSystem', s);
for k = 1:size(rows, 1)
    p = A(s, 'simulink/Sources/In1', rows{k, 7}, 'Port', num2str(k));
    if ~isempty(rows{k, 5}), set_param(p, 'OutDataTypeStr', rows{k, 5}); end
    RT(s, rows{k, 2}, rows{k, 3}, rows{k, 4});
    L(s, [rows{k, 7} '/1'], [rows{k, 2} '/1'], rows{k, 7});
    shaddad_bu('outbus', s, bus, rows{k, 1}, [rows{k, 2} '/1'], rows{k, 1});
    L(mdl, rows{k, 6}, sprintf('%s/%d', name, k), rows{k, 7});
end
end

% ===========================================================================================
function t = TASK(mdl, name, ts, ins, outs)
t = [mdl '/' name];
add_block('built-in/SubSystem', t, 'TreatAsAtomicUnit', 'on', 'SystemSampleTime', ts);
for k = 1:size(ins, 1)
    p = A(t, 'simulink/Sources/In1', ins{k, 1}, 'Port', num2str(k));
    if size(ins, 2) >= 2 && ~isempty(ins{k, 2}), set_param(p, 'OutDataTypeStr', ins{k, 2}); end
end
for k = 1:numel(outs), A(t, 'simulink/Sinks/Out1', outs{k}, 'Port', num2str(k)); end
end

function local_task_1k(mdl)
% port 6 rt1k: the bus of RT_to_Task1k (alive10, alive50, alive100, fault_word, mode, rearm_req, do_aux)
t = TASK(mdl, 'Task_1k', 'Ts.base', {'sbus', 'Bus: Bus_SbusStream'; 'di_word', 'uint16'; 't_us', 'uint32'; ...
    'reset_flag', 'boolean'; 'fault_inject', 'uint32'; 'rt1k', ''}, ...
    {'rc_dec', 'do_word', 'wdg_kick', 'reset_cnt', 't_ms'});
SSR(t, 'SBUS_Decoder', 'fw_sbus_decoder');
L(t, 'sbus/1', 'SBUS_Decoder/1', 'sbus');
L(t, 'SBUS_Decoder/1', 'rc_dec/1', 'rc_dec');
for p = {'di_word', 't_us'}
    A(t, 'simulink/Sinks/Terminator', ['T_' p{1}]); L(t, [p{1} '/1'], ['T_' p{1} '/1'], p{1});
end
% watchdog manager (T09 unit)
A(t, 'simulink/Signal Routing/Vector Concatenate', 'Alive_vec', 'NumInputs', '3');
PICK(t, 'rt1k/1', 'rt1k', 'Sel_alive', {'alive10', 'alive50', 'alive100'}, {'Alive_vec/1', 'Alive_vec/2', 'Alive_vec/3'});
SSR(t, 'Watchdog_Mgr', 'fw_watchdog_mgr');
L(t, 'Alive_vec/1', 'Watchdog_Mgr/1', 'alive'); L(t, 'fault_inject/1', 'Watchdog_Mgr/2', 'fault_inject');
PICK(t, 'rt1k/1', 'rt1k', 'Sel_wdg', {'fault_word', 'mode', 'rearm_req'}, ...
    {'Watchdog_Mgr/3', 'Watchdog_Mgr/4', 'Watchdog_Mgr/5'});
L(t, 'Watchdog_Mgr/1', 'wdg_kick/1', 'wdg_kick');
% DO word = (do_aux AND NOT 3) OR DO0 OR DO1 << 1 (ICD 6)
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Aux_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^32-4', 'BitMaskRealWorld', 'Stored Integer');
DTC(t, 'DO0_u32', 'uint32'); DTC(t, 'DO1_u32', 'uint32');
A(t, 'simulink/Logic and Bit Operations/Shift Arithmetic', 'DO1_shift', 'BitShiftNumber', '1', 'BitShiftDirection', 'Left');
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', 'DO_or', 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '3');
PICK(t, 'rt1k/1', 'rt1k', 'Sel_do_aux', {'do_aux'}, {'Aux_mask/1'});
L(t, 'Watchdog_Mgr/2', 'DO0_u32/1', 'mcu_safety_ok'); L(t, 'Watchdog_Mgr/3', 'DO1_u32/1', 'safety_reset_req');
L(t, 'DO1_u32/1', 'DO1_shift/1');
L(t, 'Aux_mask/1', 'DO_or/1'); L(t, 'DO0_u32/1', 'DO_or/2'); L(t, 'DO1_shift/1', 'DO_or/3');
L(t, 'DO_or/1', 'do_word/1', 'do_word');
% watchdog-reset event counter (reset_flag pulses; Task_10 detects the change -> BOOT, sticky wdg_reset_seen)
DTC(t, 'Rst_u8', 'uint8');
A(t, 'simulink/Math Operations/Sum', 'Rst_sum', 'Inputs', '++', 'OutDataTypeStr', 'uint8', 'SaturateOnIntegerOverflow', 'off');
A(t, 'simulink/Discrete/Unit Delay', 'Rst_prev', 'InitialCondition', '0', 'SampleTime', '-1');
L(t, 'reset_flag/1', 'Rst_u8/1', 'reset_flag'); L(t, 'Rst_u8/1', 'Rst_sum/1'); L(t, 'Rst_prev/1', 'Rst_sum/2', 'reset_cnt_prev');
L(t, 'Rst_sum/1', 'Rst_prev/1', 'reset_cnt'); L(t, 'Rst_sum/1', 'reset_cnt/1');
% millisecond tick (HAL tick) for MCU_STATUS t_ms
A(t, 'simulink/Sources/Counter Free-Running', 'Ms_tick', 'NumBits', '32', 'tsamp', 'Ts.base');
L(t, 'Ms_tick/1', 't_ms/1', 't_ms');
end

function local_task_5(mdl)
% port 2 rt5: the bus of RT_to_Task5 (t_us, fault_inject, a_track, v_track_LR)
t = TASK(mdl, 'Task_5', 'Ts.imu', {'imu', 'Bus: Bus_ImuRaw'; 'rt5', ''}, {'imu_meas', 'tilt'});
SSR(t, 'IMU_Acq', 'sens_acq_imu');
L(t, 'imu/1', 'IMU_Acq/1', 'imu');
PICK(t, 'rt5/1', 'rt5', 'Sel_acq', {'t_us', 'fault_inject'}, {'IMU_Acq/2', 'IMU_Acq/3'});
SSR(t, 'IMU_Tilt', 'fw_imu_tilt');
L(t, 'IMU_Acq/1', 'IMU_Tilt/1', 'imu_meas');
PICK(t, 'rt5/1', 'rt5', 'Sel_track', {'a_track', 'v_track_LR'}, {'IMU_Tilt/2', 'IMU_Tilt/3'});
L(t, 'IMU_Acq/1', 'imu_meas/1');
A(t, 'simulink/Signal Routing/Bus Selector', 'Sel_valid', 'OutputSignals', 'valid');
DTC(t, 'Imu_valid', 'boolean'); L(t, 'Sel_valid/1', 'Imu_valid/1');
L(t, 'IMU_Acq/1', 'Sel_valid/1');
BCV(t, 'Tilt_BC', {'roll', 'IMU_Tilt/1'; 'pitch', 'IMU_Tilt/2'; 'tilt_gate', 'IMU_Tilt/3'; ...
    'yaw_rate_meas', 'IMU_Tilt/4'; 'gyro_bias', 'IMU_Tilt/5'; 'imu_valid', 'Imu_valid/1'});
L(t, 'Tilt_BC/1', 'tilt/1', 'tilt');
end

function local_task_10(mdl)
% port 1 rt10: the bus of RT_to_Task10 (rc_dec, di_word, t_us, fault_inject, reset_cnt, tilt, bms, bms_age, adc, impl,
% gnc_cmd, comp_ok, comp_age), an In Bus Element port (decision 11, spec 6.2): one pick block rt10_<element> right in front
% of each consumer; the classic ports busoff, drive_fb, drv_age follow it (shaddad_bu('portorder'))
t = TASK(mdl, 'Task_10', 'Ts.ctrl', {'busoff', 'boolean'; 'drive_fb', 'Bus: Bus_DriveFb'; 'drv_age', 'single'}, ...
    {'drive_cmd', 'odom', 'a_track', 'v_track_LR', 'bms_req', 'st10', 'mode', 'fault_word', 'rearm_req', 'alive10', 'telem'});
SSR(t, 'RC_Mapping', 'fw_rc_mapping');
IP(t, 'rt10', 'rc_dec', 'RC_Mapping/1');
% odometry (T11 sens_odom_compute) from the ESC feedback (one pick of drive_fb in front of each consumer)
A(t, 'simulink/Signal Routing/Vector Concatenate', 'Enc_LR', 'NumInputs', '2');
A(t, 'simulink/Signal Routing/Vector Concatenate', 'Wm_LR', 'NumInputs', '2');
LOGIC(t, 'Fb_valid', 'AND', 2);
PICK(t, 'drive_fb/1', 'drive_fb', 'Sel_fb_enc', {'L.enc_cnt', 'R.enc_cnt'}, {'Enc_LR/1', 'Enc_LR/2'});
PICK(t, 'drive_fb/1', 'drive_fb', 'Sel_fb_wm', {'L.w_m', 'R.w_m'}, {'Wm_LR/1', 'Wm_LR/2'});
PICK(t, 'drive_fb/1', 'drive_fb', 'Sel_fb_rx', {'L.rx_ok', 'R.rx_ok'}, {'Fb_valid/1', 'Fb_valid/2'});
SSR(t, 'Odom', 'sens_odom_compute');
L(t, 'Enc_LR/1', 'Odom/1', 'enc_cnt_LR'); L(t, 'Wm_LR/1', 'Odom/2', 'wMotor_LR');
IP(t, 'rt10', 't_us', 'Odom/3');
L(t, 'Fb_valid/1', 'Odom/4', 'fb_valid');
A(t, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(t, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5');
DTC(t, 'V_track_LR', 'single');
PICK(t, 'Odom/1', 'odom', 'Sel_vtrack_sum', {'vTrack_LR'}, {'V_sum/1'});
PICK(t, 'Odom/1', 'odom', 'Sel_vtrack_dtc', {'vTrack_LR'}, {'V_track_LR/1'});
L(t, 'V_sum/1', 'V_mean/1');
L(t, 'Odom/1', 'odom/1'); L(t, 'Odom/2', 'a_track/1', 'a_track'); L(t, 'V_track_LR/1', 'v_track_LR/1');
% reset request = DI RESET_BTN OR RC CH9; watchdog-reset event / sticky flag; overrun; alive counter
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', 'DI_reset', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^P.electronics.di.reset_btn', 'BitMaskRealWorld', 'Stored Integer');
A(t, 'simulink/Logic and Bit Operations/Compare To Zero', 'DI_reset_on', 'relop', '~=', 'OutDataTypeStr', 'boolean');
A(t, 'simulink/Signal Routing/Bus Selector', 'Sel_rc_reset', 'OutputSignals', 'reset_req');
LOGIC(t, 'Reset_req', 'OR', 2);
IP(t, 'rt10', 'di_word', 'DI_reset/1'); L(t, 'DI_reset/1', 'DI_reset_on/1');
L(t, 'RC_Mapping/1', 'Sel_rc_reset/1', 'rc');
L(t, 'DI_reset_on/1', 'Reset_req/1'); L(t, 'Sel_rc_reset/1', 'Reset_req/2');
A(t, 'simulink/Logic and Bit Operations/Detect Change', 'Reset_evt', 'vinit', '0');
A(t, 'simulink/Logic and Bit Operations/Compare To Zero', 'Wdg_seen', 'relop', '~=', 'OutDataTypeStr', 'boolean');
IP(t, 'rt10', 'reset_cnt', 'Reset_evt/1'); IP(t, 'rt10', 'reset_cnt', 'Wdg_seen/1');
local_fi_bit(t, 'Overrun', {'rt10', 'fault_inject'}, 3);
DTC(t, 'Ovr_u32', 'uint32');
A(t, 'simulink/Math Operations/Sum', 'Ovr_sum', 'Inputs', '++', 'OutDataTypeStr', 'uint32', 'SaturateOnIntegerOverflow', 'on');
A(t, 'simulink/Discrete/Unit Delay', 'Ovr_prev', 'InitialCondition', '0', 'SampleTime', '-1');
L(t, 'Overrun/1', 'Ovr_u32/1'); L(t, 'Ovr_u32/1', 'Ovr_sum/1'); L(t, 'Ovr_prev/1', 'Ovr_sum/2', 'overruns_prev');
L(t, 'Ovr_sum/1', 'Ovr_prev/1', 'overruns');
local_fi_bit(t, 'Stall10', {'rt10', 'fault_inject'}, 0);
local_alive(t, 'Alive10', 'Stall10/1');
L(t, 'Alive10/1', 'alive10/1', 'alive10');
% ---- platform bus fw10 (input of the T09 units); each pick's line carries its element's name into the virtual bus ----
BCV(t, 'Platform_BC', {'rc', 'RC_Mapping/1'; 'di_word', {'rt10', 'di_word'}; 'reset_req', 'Reset_req/1'; ...
    'reset_evt', 'Reset_evt/1'; 'wdg_reset_seen', 'Wdg_seen/1'; 'overrun', 'Overrun/1'; 'busoff', 'busoff/1'; ...
    't_us', {'rt10', 't_us'}; 'drive_fb', 'drive_fb/1'; 'drv_age', 'drv_age/1'; 'odom', 'Odom/1'; 'a_track', 'Odom/2'; ...
    'v_track', 'V_mean/1'; 'tilt', {'rt10', 'tilt'}; 'bms', {'rt10', 'bms'}; 'bms_age', {'rt10', 'bms_age'}; ...
    'adc', {'rt10', 'adc'}; 'impl', {'rt10', 'impl'}; 'gnc_cmd', {'rt10', 'gnc_cmd'}; 'comp_ok', {'rt10', 'comp_ok'}; ...
    'comp_age', {'rt10', 'comp_age'}});
% ---- T09 units ----
SSR(t, 'Safety_Supervisor', 'fw_safety_supervisor');
SSR(t, 'Mode_Manager', 'fw_mode_manager');
SSR(t, 'Vehicle_Ctrl', 'fw_vehicle_ctrl');
A(t, 'simulink/Discrete/Unit Delay', 'Mode_prev', 'InitialCondition', '0', 'SampleTime', '-1');
% one-sample delays of unit outputs that the supervisor reads on its inputs 3 and 4 (D71, R-01, 2026-11-15), as the mode
% on input 2 (not through fw10, so that Platform_BC stays out of the loops): the mode manager's stop monitor
% stop_ctl(2) (fault bit 22 STOP_FAIL) and the track speed references vc_tel(8:9) of fw_vehicle_ctrl (the OVERSPEED
% speed-follow condition, bit 16); their inputs are fed after the units below
A(t, 'simulink/Discrete/Unit Delay', 'Stop_fail_prev', 'InitialCondition', 'false', 'SampleTime', '-1');
A(t, 'simulink/Discrete/Unit Delay', 'Vtrack_ref_prev', 'InitialCondition', '[0 0]', 'SampleTime', '-1');
L(t, 'Platform_BC/1', 'Safety_Supervisor/1', 'fw10'); L(t, 'Mode_prev/1', 'Safety_Supervisor/2', 'mode_prev');
L(t, 'Stop_fail_prev/1', 'Safety_Supervisor/3', 'stop_fail_prev');
L(t, 'Vtrack_ref_prev/1', 'Safety_Supervisor/4', 'vtrack_ref_prev');
L(t, 'Platform_BC/1', 'Mode_Manager/1'); L(t, 'Safety_Supervisor/1', 'Mode_Manager/2', 'fault_word');
L(t, 'Safety_Supervisor/3', 'Mode_Manager/3', 'fault_cls');
L(t, 'Mode_Manager/1', 'Mode_prev/1', 'mode');
L(t, 'Platform_BC/1', 'Vehicle_Ctrl/1'); L(t, 'Mode_Manager/1', 'Vehicle_Ctrl/2'); L(t, 'Mode_Manager/2', 'Vehicle_Ctrl/3', 'cmd_src');
L(t, 'Mode_Manager/3', 'Vehicle_Ctrl/4', 'drive_enable'); L(t, 'Safety_Supervisor/1', 'Vehicle_Ctrl/5');
L(t, 'Safety_Supervisor/4', 'Vehicle_Ctrl/6', 'k_der');
% stop_ctl [limits_hold stop_fail] (D71): limits_hold to the vehicle control in the same sample, stop_fail to the
% supervisor input 3 through Stop_fail_prev; the references vc_tel(8:9) to its input 4 through Vtrack_ref_prev
A(t, 'simulink/Signal Routing/Demux', 'Stop_ctl', 'Outputs', '2'); L(t, 'Mode_Manager/6', 'Stop_ctl/1', 'stop_ctl');
L(t, 'Stop_ctl/1', 'Vehicle_Ctrl/7', 'limits_hold'); L(t, 'Stop_ctl/2', 'Stop_fail_prev/1', 'stop_fail');
A(t, 'simulink/Signal Routing/Selector', 'Vtrack_ref', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '[8 9]', 'InputPortWidth', '11');
L(t, 'Vehicle_Ctrl/2', 'Vtrack_ref/1', 'vc_tel'); L(t, 'Vtrack_ref/1', 'Vtrack_ref_prev/1', 'vtrack_ref');
L(t, 'Vehicle_Ctrl/1', 'drive_cmd/1', 'drive_cmd');
L(t, 'Mode_Manager/5', 'bms_req/1', 'bms_req');
L(t, 'Mode_Manager/1', 'mode/1'); L(t, 'Safety_Supervisor/1', 'fault_word/1'); L(t, 'Mode_Manager/4', 'rearm_req/1', 'rearm_req');
% ---- telemetry (inputs 10-13 picked from rt10) ----
SSR(t, 'Telemetry', 'fw_telemetry');
src = {'RC_Mapping/1', 'Mode_Manager/1', 'Mode_Manager/2', 'Safety_Supervisor/1', 'Safety_Supervisor/2', ...
    'Vehicle_Ctrl/2', 'Vehicle_Ctrl/1', 'drive_fb/1', 'Odom/1', {'rt10', 'tilt'}, {'rt10', 'bms'}, {'rt10', 'adc'}, ...
    {'rt10', 'comp_age'}, 'Ovr_sum/1'};
for k = 1:numel(src)
    if iscell(src{k}), IP(t, src{k}{1}, src{k}{2}, sprintf('Telemetry/%d', k)); else, L(t, src{k}, sprintf('Telemetry/%d', k)); end
end
L(t, 'Telemetry/1', 'telem/1', 'telem');
% ---- status signals for MCU_STATUS (Task_50) and the implement unit: st10 ----
CMP(t, 'Is_estop', '==', '8');
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Estop_bits', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '3', 'BitMaskRealWorld', 'Stored Integer');
A(t, 'simulink/Logic and Bit Operations/Compare To Zero', 'Estop_bit_on', 'relop', '~=', 'OutDataTypeStr', 'boolean');
LOGIC(t, 'Estop_active', 'OR', 2);
L(t, 'Mode_Manager/1', 'Is_estop/1'); L(t, 'Safety_Supervisor/1', 'Estop_bits/1'); L(t, 'Estop_bits/1', 'Estop_bit_on/1');
L(t, 'Is_estop/1', 'Estop_active/1'); L(t, 'Estop_bit_on/1', 'Estop_active/2');
CMP(t, 'Is_auto', '==', '6');
L(t, 'Mode_Manager/1', 'Is_auto/1');
for sd = {'L', 'R'}
    x = sd{1};
    CMP(t, ['Rdy_' x], '==', '2'); CMP(t, ['Op_' x], '==', '4'); LOGIC(t, ['Esc_ok_' x], 'OR', 2);
    SELB(t, 'drive_fb/1', 'drive_fb', ['Sel_fb_state_' x], {[x '.state']});      % one pick for the pair of compares
    L(t, ['Sel_fb_state_' x '/1'], ['Rdy_' x '/1']); L(t, ['Sel_fb_state_' x '/1'], ['Op_' x '/1']);
    L(t, ['Rdy_' x '/1'], ['Esc_ok_' x '/1']); L(t, ['Op_' x '/1'], ['Esc_ok_' x '/2']);
end
A(t, 'simulink/Signal Routing/Bus Selector', 'Sel_hv', 'OutputSignals', 'hv_ready');     % fed by a pick of rt10.bms
IP(t, 'rt10', 'bms', 'Sel_hv/1');
LOGIC(t, 'Drive_ready', 'AND', 4);
L(t, 'Is_auto/1', 'Drive_ready/1'); L(t, 'Esc_ok_L/1', 'Drive_ready/2'); L(t, 'Esc_ok_R/1', 'Drive_ready/3'); L(t, 'Sel_hv/1', 'Drive_ready/4');
DTC(t, 'Fc_L_u32', 'uint32'); DTC(t, 'Fc_R_u32', 'uint32');
A(t, 'simulink/Logic and Bit Operations/Shift Arithmetic', 'Fc_R_shift', 'BitShiftNumber', '16', 'BitShiftDirection', 'Left');
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Drv_fault_word', 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '2');
PICK(t, 'drive_fb/1', 'drive_fb', 'Sel_fb_fc_L', {'L.fault_code'}, {'Fc_L_u32/1'});
PICK(t, 'drive_fb/1', 'drive_fb', 'Sel_fb_fc_R', {'R.fault_code'}, {'Fc_R_u32/1'});
L(t, 'Fc_R_u32/1', 'Fc_R_shift/1');
L(t, 'Fc_L_u32/1', 'Drv_fault_word/1'); L(t, 'Fc_R_shift/1', 'Drv_fault_word/2');
BCV(t, 'Status_BC', {'mode', 'Mode_Manager/1'; 'cmd_src', 'Mode_Manager/2'; 'fault_word', 'Safety_Supervisor/1'; ...
    'warn_word', 'Safety_Supervisor/2'; 'k_der', 'Safety_Supervisor/4'; 'rc', 'RC_Mapping/1'; ...
    'estop_active', 'Estop_active/1'; 'drive_ready', 'Drive_ready/1'; 'wdg_reset_seen', 'Wdg_seen/1'; ...
    'drive_fault_word', 'Drv_fault_word/1'; 'v_track_LR', 'V_track_LR/1'; 'v_track', 'V_mean/1'; 'vc_tel', 'Vehicle_Ctrl/2'});
L(t, 'Status_BC/1', 'st10/1', 'st10');
shaddad_bu('portorder', t, {'rt10', 'busoff', 'drive_fb', 'drv_age'}, 'Inport');
end

function local_task_50(mdl)
% port 3 rt50: the bus of RT_to_Task50 (st = the Task_10 status bus st10, di_word, fault_inject, bms)
t = TASK(mdl, 'Task_50', 'Ts.comm', {'adc_raw', 'uint16'; 'mcu_temp_degC', 'single'; 'rt50', ''; ...
    'gnc_cmd', 'Bus: Bus_GncCmd'; 'comp_ok', 'boolean'; 'comp_age', 'single'; 'impl', ''}, ...
    {'do_aux', 'pwm_duty', 'impl_enable', 'impl_fault_reset', 'impl_speed_rpm', 'status', 'alive50', 'adc'});
SSR(t, 'ADC_Scaling', 'fw_adc_scaling');
L(t, 'adc_raw/1', 'ADC_Scaling/1', 'adc_raw'); L(t, 'mcu_temp_degC/1', 'ADC_Scaling/2', 'mcu_temp_degC');
L(t, 'ADC_Scaling/1', 'adc/1', 'adc');
% ---- platform bus fw50 (input of fw_implement_aux); the picks of rt50 keep the element names st, di_word, bms ----
SELB(t, 'rt50/1', 'rt50', 'Sel_fw_st', {'st', 'di_word'});
SELB(t, 'rt50/1', 'rt50', 'Sel_fw_bms', {'bms'});
BCV(t, 'Platform_BC', {'st', 'Sel_fw_st/1'; 'di_word', 'Sel_fw_st/2'; 'gnc_cmd', 'gnc_cmd/1'; 'comp_ok', 'comp_ok/1'; ...
    'comp_age', 'comp_age/1'; 'adc', 'ADC_Scaling/1'; 'impl', 'impl/1'; 'bms', 'Sel_fw_bms/1'});
SSR(t, 'Implement_Aux', 'fw_implement_aux');
L(t, 'Platform_BC/1', 'Implement_Aux/1', 'fw50');
nm = {'do_aux', 'pwm_duty', 'impl_enable', 'impl_fault_reset', 'impl_speed_rpm'};
for k = 1:5, L(t, sprintf('Implement_Aux/%d', k), [nm{k} '/1'], nm{k}); end
SELB(t, 'rt50/1', 'rt50', 'Sel_fi_stall', {'fault_inject'});
local_fi_bit(t, 'Stall50', 'Sel_fi_stall/1', 1);
local_alive(t, 'Alive50', 'Stall50/1');
L(t, 'Alive50/1', 'alive50/1', 'alive50');
% ---- MCU_STATUS (0x200) assembly from the Task_10 status bus (fast-to-slow RT): one pick of rt50 (st, bms) or impl per
% run of consecutive Status_BC inputs, right in front of it (spec 6.2) ----
LOGIC(t, 'Link_ok', 'NOT', 1);
PICK(t, 'rt50/1', 'rt50', 'Sel_st_link', {'st.rc.rc_link_lost'}, {'Link_ok/1'});
A(t, 'simulink/Signal Routing/Selector', 'V_applied', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '3', 'InputPortWidth', '11');
A(t, 'simulink/Signal Routing/Selector', 'W_applied', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '4', 'InputPortWidth', '11');
SELB(t, 'rt50/1', 'rt50', 'Sel_st_vc', {'st.vc_tel'});             % one pick for the pair of Selectors
L(t, 'Sel_st_vc/1', 'V_applied/1'); L(t, 'Sel_st_vc/1', 'W_applied/1');
DTC(t, 'Impl_type_u8', 'uint8');
PICK(t, 'impl/1', 'impl', 'Sel_impl_type', {'impl_type'}, {'Impl_type_u8/1'});
A(t, 'simulink/Math Operations/Gain', 'Age_ms', 'Gain', '1000');
DTC(t, 'Age_ms_u16', 'uint16');
L(t, 'comp_age/1', 'Age_ms/1', 'comp_age'); L(t, 'Age_ms/1', 'Age_ms_u16/1');
A(t, 'simulink/Sources/Counter Free-Running', 'Heartbeat', 'NumBits', '8', 'tsamp', 'Ts.comm');
CST(t, 'T_mcu_na', '0', 'double', 'Ts.comm');
CST(t, 'Valid', 'true', 'boolean', 'Ts.comm');
SELB(t, 'rt50/1', 'rt50', 'Sel_st_s02', {'st.mode', 'st.cmd_src', 'st.rc.rc_mode_sw', 'st.rc.rc_aux_cmd', 'st.rc.rc_aux_seq'});
SELB(t, 'rt50/1', 'rt50', 'Sel_st_s08', {'st.rc.failsafe', 'st.estop_active', 'st.drive_ready'});
SELB(t, 'rt50/1', 'rt50', 'Sel_bms_hv', {'bms.hv_ready'});
SELB(t, 'impl/1', 'impl', 'Sel_impl_ready', {'ready'});
SELB(t, 'rt50/1', 'rt50', 'Sel_bms_chg', {'bms.charging'});
SELB(t, 'rt50/1', 'rt50', 'Sel_st_wdg', {'st.wdg_reset_seen'});
SELB(t, 'rt50/1', 'rt50', 'Sel_st_s17', {'st.fault_word', 'st.warn_word', 'st.drive_fault_word', 'st.v_track_LR'});
SELB(t, 'rt50/1', 'rt50', 'Sel_st_lq', {'st.rc.lq'});
SELB(t, 'ADC_Scaling/1', 'adc', 'Sel_adc', {'T_encl'});
BCB(t, 'Status_BC', 'Bus_McuStatus', {'t_mcu', 'T_mcu_na/1'; 'mode', 'Sel_st_s02/1'; 'cmd_src', 'Sel_st_s02/2'; ...
    'rc_mode_sw', 'Sel_st_s02/3'; 'rc_aux_cmd', 'Sel_st_s02/4'; 'rc_aux_seq', 'Sel_st_s02/5'; 'rc_link_ok', 'Link_ok/1'; ...
    'rc_failsafe', 'Sel_st_s08/1'; 'estop_active', 'Sel_st_s08/2'; 'drive_ready', 'Sel_st_s08/3'; ...
    'power_ready', 'Sel_bms_hv/1'; 'implement_ready', 'Sel_impl_ready/1'; 'charging', 'Sel_bms_chg/1'; ...
    'wdg_reset_seen', 'Sel_st_wdg/1'; 'implement_type', 'Impl_type_u8/1'; 'heartbeat', 'Heartbeat/1'; ...
    'fault_word', 'Sel_st_s17/1'; 'warn_word', 'Sel_st_s17/2'; 'drive_fault_word', 'Sel_st_s17/3'; ...
    'v_track_meas', 'Sel_st_s17/4'; 'v_applied', 'V_applied/1'; 'w_applied', 'W_applied/1'; ...
    'gnc_cmd_age_ms', 'Age_ms_u16/1'; 'rc_lq', 'Sel_st_lq/1'; 'T_encl', 'Sel_adc/1'; 'valid', 'Valid/1'});
L(t, 'Status_BC/1', 'status/1', 'status');
end

function local_task_100(mdl)
t = TASK(mdl, 'Task_100', 'Ts.hk', {'fault_inject', 'uint32'}, {'alive100'});
local_fi_bit(t, 'Stall100', 'fault_inject/1', 2);
local_alive(t, 'Alive100', 'Stall100/1');
L(t, 'Alive100/1', 'alive100/1', 'alive100');
end

function local_fi_bit(t, name, src, bit)
% boolean = (fault_inject AND 2^bit) ~= 0; src 'Blk/k', or {port, element} for a pick of an In Bus Element port (IP)
A(t, 'simulink/Logic and Bit Operations/Bitwise Operator', [name '_mask'], 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', num2str(2^bit), 'BitMaskRealWorld', 'Stored Integer');
A(t, 'simulink/Logic and Bit Operations/Compare To Zero', name, 'relop', '~=', 'OutDataTypeStr', 'boolean');
if iscell(src), IP(t, src{1}, src{2}, [name '_mask/1']); else, L(t, src, [name '_mask/1']); end
L(t, [name '_mask/1'], [name '/1']);
end

function local_alive(t, name, stallSrc)
% task alive counter (uint8, +1 per task step, frozen while the task is emulated as stalled)
A(t, 'simulink/Discrete/Unit Delay', [name '_prev'], 'InitialCondition', '0', 'SampleTime', '-1');
CST(t, [name '_one'], '1', 'uint8', '-1');
A(t, 'simulink/Math Operations/Sum', [name '_inc'], 'Inputs', '++', 'OutDataTypeStr', 'uint8', 'SaturateOnIntegerOverflow', 'off');
A(t, 'simulink/Signal Routing/Switch', name, 'Criteria', 'u2 ~= 0');
L(t, [name '_prev/1'], [name '_inc/1']); L(t, [name '_one/1'], [name '_inc/2']);
L(t, [name '_prev/1'], [name '/1']); L(t, stallSrc, [name '/2']); L(t, [name '_inc/1'], [name '/3']);
L(t, [name '/1'], [name '_prev/1']);
end

function s = local_desc()
s = ['Shaddad_MCU (T08): STM32H743 firmware. Ports ICD 3.1: in 1 in1k Bus_McuIn1k 0.001, 2 in5 Bus_McuIn5 0.005, ' ...
    '3 in10 Bus_McuIn10 0.01, 4 in50 Bus_McuIn50 0.05, 5 in100 Bus_McuIn100 0.1; out 1 out1k Bus_McuOut1k 0.001, ' ...
    '2 out5 Bus_McuOut5 0.005, 3 out10 Bus_McuOut10 0.01, 4 out50 Bus_McuOut50 0.05, 5 out100 Bus_McuOut100 0.1, ' ...
    '6 telem Bus_FwTelemetry 0.01. IO_In / IO_Out Variant Subsystems (VC_TARGET 0 SIM / 1 HW, update diagram); atomic ' ...
    'tasks Task_1k / Task_5 / Task_10 / Task_50 / Task_100; CAN1_Codec / CAN2_Codec (multirate); explicit ' ...
    'deterministic Rate Transitions between rates (those into Task_1k, Task_5, Task_10, Task_50 in the virtual ' ...
    'subsystems RT_to_Task1k / RT_to_Task5 / RT_to_Task10 / RT_to_Task50, one virtual bus rt1k / rt5 / rt10 / rt50 ' ...
    'into each task). Platform buses for the T09 units (virtual): ' ...
    'fw10 (Task_10) = rc (fw_rc_mapping bus), di_word uint16, reset_req (DI RESET_BTN OR RC CH9), reset_evt (one ' ...
    'sample after a watchdog-reset pulse), wdg_reset_seen, overrun (fault_inject b3), busoff boolean[2] [CAN1 CAN2], ' ...
    't_us uint32, drive_fb Bus_DriveFb, drv_age single[2] s, odom Bus_OdomMeas, a_track single, v_track single (mean ' ...
    'track speed), tilt (roll pitch tilt_gate yaw_rate_meas gyro_bias imu_valid), bms Bus_BmsStatus (valid = all ' ...
    'frames < bms_timeout), bms_age single, adc (V_pack V_bus I_pack V24 V12 V5 T_encl lift_pos T_mcu), impl (running ' ...
    'fault ready impl_type speed_rpm current_A temp_degC alive valid age rx_ok), gnc_cmd Bus_GncCmd (Task_50 -> Task_10 ' ...
    'deterministic RT, D41), comp_ok, comp_age. Safety_Supervisor reads the previous sample of three unit outputs on ' ...
    'its inputs 2-4 (Unit Delays Mode_prev, Stop_fail_prev, Vtrack_ref_prev): mode, fw_mode_manager stop_ctl(2) (fault ' ...
    'bit 22 STOP_FAIL, D71) and fw_vehicle_ctrl vc_tel(8:9) (the OVERSPEED speed-follow condition); stop_ctl(1) ' ...
    'limits_hold goes from Mode_Manager to Vehicle_Ctrl/7 in the same sample. fw50 (Task_50) = st (Task_10 status ' ...
    'bus st10: mode cmd_src fault_word ' ...
    'warn_word k_der rc estop_active drive_ready wdg_reset_seen drive_fault_word v_track_LR v_track vc_tel, through RT ' ...
    '0.01 -> 0.05), di_word, gnc_cmd, comp_ok, comp_age, adc, impl, bms. Task alive counters freeze on fault_inject ' ...
    'b0/b1/b2 (stall emulation) and feed fw_watchdog_mgr; DO word = (do_aux AND NOT 3) OR DO0 OR DO1 << 1. ' ...
    'Built by scripts/build/build_shaddad_mcu.m.'];
end

% ===========================================================================================
% helpers
% ===========================================================================================
function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function h = L(sys, src, dst, nm)
% signal line src -> dst ('Blk/k'); nm names it when its source port's line has no name yet (the first name given
% stays); a Bus Selector's output keeps '<element>'. local_name_rest names the lines that no call names (L5)
h = add_line(sys, src, dst, 'autorouting', 'smart');
if nargin >= 4 && ~isempty(nm)
    s = get_param(h, 'SrcBlockHandle');
    if ~strcmp(get_param(s, 'BlockType'), 'BusSelector') && isempty(get_param(h, 'Name'))
        set_param(h, 'Name', nm);
    end
end
end

function SELB(sys, src, nm, blk, sigs)
% a Bus Selector blk that selects sigs from the bus line of src ('Blk/k'), which L names nm when it has no name yet;
% the caller wires its outputs (a pick right in front of its consumer, spec 6.2, or one pick for a pair of like blocks)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
end

function PICK(sys, src, nm, blk, sigs, dsts)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line of src (named nm,
% see SELB) and feeds its output k to dsts{k} ('Blk/j')
SELB(sys, src, nm, blk, sigs);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end

function IP(sys, port, el, dst, nm)
% the pick of element el of the In Bus Element port 'port' of sys right in front of dst ('Blk/k'), spec 6.2:
% shaddad_bu('pick') (block <port>_<el>, a further pick <port>_<el>_2, ...), its line named nm (default el)
if nargin < 5, nm = el; end
shaddad_bu('pick', sys, port, el, dst, nm);
end

function local_name_rest(mdl)
% L5 (layout spec 6.1): every signal line of every level of the model (shaddad_layout_diagrams) that no call named
% takes its source block's name, '_k' for output k of a block with several outputs, as shaddad_bu('nline') names a
% line; a line out of a Bus Selector keeps the '<element>' name R2026b gives it
d = shaddad_layout_diagrams(mdl);
for q = 1:numel(d)
    b = find_system(d{q}, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
        'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
    b = b(~strcmp(b, d{q}));
    for k = 1:numel(b)
        if strcmp(get_param(b{k}, 'BlockType'), 'BusSelector'), continue, end
        ph = get_param(b{k}, 'PortHandles');
        for j = 1:numel(ph.Outport)
            ln = get_param(ph.Outport(j), 'Line');
            if ln == -1 || ~isempty(get_param(ln, 'Name')), continue, end
            nm = get_param(b{k}, 'Name');
            if numel(ph.Outport) > 1, nm = sprintf('%s_%d', nm, j); end
            set_param(ln, 'Name', nm);
        end
    end
end
end

function p = SSR(sys, name, ref)
p = A(sys, 'simulink/Ports & Subsystems/Subsystem Reference', name, 'ReferencedSubsystem', ref);
end

function p = DTC(sys, name, type)
p = A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', type, ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
end

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function p = CMP(sys, name, op, c)
p = A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', name, 'relop', op, 'const', c, ...
    'OutDataTypeStr', 'boolean');
end

function p = LOGIC(sys, name, op, nin)
p = A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(nin));
end

function p = RT(sys, name, ts, ic)
p = A(sys, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', ts, 'InitialCondition', ic);
end

function RTL(sys, name, ts, ic, src, dst, nm)
RT(sys, name, ts, ic);
if nargin < 7, nm = ''; end
L(sys, src, [name '/1'], nm);
L(sys, [name '/1'], dst);
end

function p = BCV(sys, name, map)
% virtual Bus Creator: input k fed by map{k,2} ('Blk/k', or {port, element} for a pick of an In Bus Element port, IP),
% its line named map{k,1} when it has no name yet (a virtual bus takes its element names from its input lines)
p = A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(size(map, 1)));
for k = 1:size(map, 1)
    if iscell(map{k, 2})
        IP(sys, map{k, 2}{1}, map{k, 2}{2}, sprintf('%s/%d', name, k), map{k, 1});
        continue
    end
    h = add_line(sys, map{k, 2}, sprintf('%s/%d', name, k), 'autorouting', 'smart');
    s = get_param(h, 'SrcBlockHandle');
    if ~strcmp(get_param(s, 'BlockType'), 'BusSelector') && isempty(get_param(h, 'Name'))
        set_param(h, 'Name', map{k, 1});
    end
end
end

function p = BCB(sys, name, bus, map)
% nonvirtual Bus Creator with a bus object; map {element, source} in any order; a line into it without a name yet
% takes its element's name (L, first name stays)
B = evalin('base', bus);
el = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
assert(numel(el) == size(map, 1) && all(ismember(el, map(:, 1))), 'build_shaddad_mcu:bus', ...
    '%s: map does not cover the elements of %s', name, bus);
p = A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(el)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(el)
    L(sys, map{strcmp(map(:, 1), el{k}), 2}, sprintf('%s/%d', name, k), el{k});
end
end

function LOG(sys, blk, port, nm)
ph = get_param([sys '/' blk], 'PortHandles');
set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', nm);
end
