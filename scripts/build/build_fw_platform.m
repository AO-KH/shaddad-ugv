function files = build_fw_platform(which)
%BUILD_FW_PLATFORM  MCU firmware platform units of Shaddad_MCU (owner: electronics, task T08).
%
%   build_fw_platform()          build every unit below (Subsystem Reference files)
%   build_fw_platform(names)     build only these units (e.g. {'fw_sbus_decoder'})
%   files = build_fw_platform(...)   project-relative paths written
%
%   subsystems/electronics/
%     fw_sbus_decoder       Task_1k  MATLAB Function sbus_frame_sync (spec 4.1) + commutil2 unpack + link supervision
%     fw_rc_mapping         Task_10  ICD 7 channel map: deadband, expo, 3-position decode + Tapped Delay debounce,
%                                    speed scale, arm edge, sticks_neutral, override, remote E-stop (valid frames
%                                    only, D34), mission / RTH edges -> virtual bus rc
%     fw_imu_tilt           Task_5   linear-acceleration compensation (D47) + Complementary Filter, gyro-z bias
%     fw_can1_codec         10/50/100 ms  CAN1 codecs of T04/T05/T06 + frame ages / valid flags
%     fw_can2_codec         5/10/50/100 ms  CAN-FD codecs can2_* (build_can2_codecs) + companion age
%     fw_adc_scaling        Task_50  ADC counts -> engineering values (NTC 1-D Lookup)
%     fw_telemetry          Task_10  -> Bus_FwTelemetry
%     fw_io_in_sim / fw_io_in_hw / fw_io_out_sim / fw_io_out_hw    IO_In / IO_Out variant choices (VC_TARGET)
%     fw_mode_manager, fw_safety_supervisor, fw_vehicle_ctrl, fw_implement_aux, fw_watchdog_mgr
%                           stubs with the final ports for T09 (port list in each Description)
%   Requires P, Ts and Bus_* in the base workspace (shaddad_params). Build rule 3 (build_plan.md).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of each file
%   with shaddad_layout(<file>, 'Recursive', true), the Enabled, For Iterator and age / debounce subsystems included.
%   Every signal line is named (L5, shaddad_bu('nline'); a Bus Selector's output keeps '<element>'; the virtual Bus
%   Creators through shaddad_bu('vbus'), their element names = line names). The root ports of every unit stay classic
%   (Shaddad_MCU, mcu_harness and the benches wire them by number). Inside, one Bus Selector right in front of each
%   consumer (spec 6.2), fed by the bus line of the port; in fw_telemetry the telemetry signals travel as one virtual
%   bus 'tel' in the order of their producers (virtual Bus Creator Tel, spec 6.3), picked by Sel_tel right in front of
%   the bus-typed Telem_BC in the element order of Bus_FwTelemetry. fw_io_in_hw and fw_io_out_hw (no compile on a
%   machine without STM32CubeCLT): line names only, no new bus, selector or port.

all = {'fw_sbus_decoder', 'fw_rc_mapping', 'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', ...
    'fw_adc_scaling', 'fw_telemetry', 'fw_io_in_sim', 'fw_io_in_hw', 'fw_io_out_sim', 'fw_io_out_hw', ...
    'fw_mode_manager', 'fw_safety_supervisor', 'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
bad = setdiff(which, all);
assert(isempty(bad), 'build_fw_platform:unit', 'Unknown unit(s): %s', strjoin(bad, ', '));
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
files = {};
for k = 1:numel(which)
    n = which{k};
    f = fullfile(root, 'subsystems', 'electronics', [n '.slx']);
    shaddad_bu('fresh', n, f, 'ssr');
    feval(['local_' n], n);
    shaddad_bu('save', n, f);
    close_system(n, 0);
    files{end+1} = fullfile('subsystems', 'electronics', [n '.slx']); %#ok<AGROW>
end
end

% ===========================================================================================
% fw_sbus_decoder (Task_1k)
% ===========================================================================================
function local_fw_sbus_decoder(n)
BT = 'Ts.base';
IN(n, 'sbus', 1, 'Bus: Bus_SbusStream', BT);
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_sbus', 'OutputSignals', 'bytes,len,uart_err');
L(n, 'sbus/1', 'Sel_sbus/1', 'sbus');
CST(n, 'HDR', 'P.electronics.sbus.header', 'uint8', BT);
CST(n, 'GAP_TICKS', 'P.electronics.sbus.gap_ticks', 'uint8', BT);
CST(n, 'RAW_MIN', 'P.electronics.rcmap.raw_min', 'uint16', BT);   % channel plausibility (ICD 7, rev. C)
CST(n, 'RAW_MAX', 'P.electronics.rcmap.raw_max', 'uint16', BT);
mf = A(n, 'simulink/User-Defined Functions/MATLAB Function', 'Frame_Sync');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = sprintf(['function [payload, flags, frame_new, n_ok, n_bad, n_skip] = fcn(bytes, len, uart_err, HDR, GAP_TICKS, RAW_MIN, RAW_MAX)\n' ...
    '%%#codegen\n' ...
    '%% body: subsystems/electronics/sbus_frame_sync.m (docs/specs/electronics.md 4.1, revision C)\n' ...
    '[payload, flags, frame_new, n_ok, n_bad, n_skip] = sbus_frame_sync(bytes, len, uart_err, HDR, GAP_TICKS, RAW_MIN, RAW_MAX);\n']);
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = false;
cfg.Description = ['SBUS frame synchroniser (spec 4.1, rev. C: a UART error taints every frame with a byte in the ' ...
    'flagged chunk; 25-byte length, footer, reserved flag bits, channel range RAW_MIN..RAW_MAX). HDR / GAP_TICKS / ' ...
    'RAW_MIN / RAW_MAX arrive on inputs (no parameter data).'];
L(n, 'Sel_sbus/1', 'Frame_Sync/1'); L(n, 'Sel_sbus/2', 'Frame_Sync/2'); L(n, 'Sel_sbus/3', 'Frame_Sync/3');
L(n, 'HDR/1', 'Frame_Sync/4', 'HDR'); L(n, 'GAP_TICKS/1', 'Frame_Sync/5', 'GAP_TICKS');
L(n, 'RAW_MIN/1', 'Frame_Sync/6', 'RAW_MIN'); L(n, 'RAW_MAX/1', 'Frame_Sync/7', 'RAW_MAX');
% ---- latch on frame_new (Enabled Subsystem, held outputs): unpack, flags, validity, LQ ----
s = ENSUB(n, 'Latch', {'payload', 'flags'}, ...
    {'ch_raw', 'uint16(P.electronics.rcmap.raw_mid)'; 'ch17', 'false'; 'ch18', 'false'; ...
     'frame_lost', 'false'; 'failsafe', 'false'; 'last_valid', 'false'; 'lq', 'single(P.electronics.sbus.lq_ic)'});
A(s, 'commutil2/Integer to Bit Converter', 'Pay_bits', 'nbits', '8', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
A(s, 'commutil2/Bit to Integer Converter', 'Ch11', 'nbits', 'P.electronics.sbus.ch_bits', 'bitOrder', 'LSB first', ...
    'outDtype', 'uint16');
L(s, 'payload/1', 'Pay_bits/1', 'payload'); L(s, 'Pay_bits/1', 'Ch11/1', 'payload_bits');
L(s, 'Ch11/1', 'ch_raw/1', 'ch_raw');
A(s, 'commutil2/Integer to Bit Converter', 'Flag_bits', 'nbits', '8', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
A(s, 'simulink/Signal Routing/Demux', 'Flag_dmx', 'Outputs', '8');
L(s, 'flags/1', 'Flag_bits/1', 'flags'); L(s, 'Flag_bits/1', 'Flag_dmx/1', 'flag_bits');
L(s, 'Flag_dmx/1', 'ch17/1', 'ch17'); L(s, 'Flag_dmx/2', 'ch18/1', 'ch18');
L(s, 'Flag_dmx/3', 'frame_lost/1', 'frame_lost'); L(s, 'Flag_dmx/4', 'failsafe/1', 'failsafe');
for j = 5:8
    A(s, 'simulink/Sinks/Terminator', sprintf('T_rsv%d', j));
    L(s, sprintf('Flag_dmx/%d', j), sprintf('T_rsv%d/1', j), sprintf('flag_rsv%d', j));
end
LOGIC(s, 'Lost_or_FS', 'NOR', 2);
L(s, 'Flag_dmx/3', 'Lost_or_FS/1'); L(s, 'Flag_dmx/4', 'Lost_or_FS/2');
L(s, 'Lost_or_FS/1', 'last_valid/1', 'last_valid');
% LQ = first-order filter of (1 - frame_lost), one update per frame: lq += alpha*(good - lq)
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Not_lost', 'Operator', 'NOT', 'Inputs', '1');
DTC(s, 'Good_s', 'single');
A(s, 'simulink/Discrete/Unit Delay', 'LQ_prev', 'InitialCondition', 'single(P.electronics.sbus.lq_ic)', 'SampleTime', '-1');
A(s, 'simulink/Math Operations/Sum', 'LQ_err', 'Inputs', '+-');
A(s, 'simulink/Math Operations/Gain', 'LQ_alpha', 'Gain', 'P.electronics.sbus.lq_alpha');
A(s, 'simulink/Math Operations/Sum', 'LQ_new', 'Inputs', '++');
L(s, 'Flag_dmx/3', 'Not_lost/1'); L(s, 'Not_lost/1', 'Good_s/1', 'not_lost'); L(s, 'Good_s/1', 'LQ_err/1', 'good');
L(s, 'LQ_prev/1', 'LQ_err/2', 'lq_prev'); L(s, 'LQ_err/1', 'LQ_alpha/1', 'lq_err'); L(s, 'LQ_alpha/1', 'LQ_new/1', 'lq_step');
L(s, 'LQ_prev/1', 'LQ_new/2'); L(s, 'LQ_new/1', 'LQ_prev/1', 'lq'); L(s, 'LQ_new/1', 'lq/1');
L(n, 'Frame_Sync/1', 'Latch/1', 'payload'); L(n, 'Frame_Sync/2', 'Latch/2', 'flags');
ENLINE(n, 'Frame_Sync/3', 'Latch', 'frame_new');
% ---- frame_valid, rc_age (integer ticks), link supervision ----
LOGIC(n, 'Frame_valid', 'AND', 2);
L(n, 'Frame_Sync/3', 'Frame_valid/1'); L(n, 'Latch/6', 'Frame_valid/2', 'last_valid');
AGE(n, 'Age', 'round(P.electronics.sbus.age_sat/Ts.base)');
L(n, 'Frame_valid/1', 'Age/1', 'frame_valid');
DTC(n, 'Age_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Age_Ts', 'Gain', 'Ts.base');
L(n, 'Age/1', 'Age_s/1', 'age_ticks'); L(n, 'Age_s/1', 'Age_Ts/1', 'age_ticks_s');
CMP(n, 'Link_lost', '>', 'round(P.electronics.sbus.link_timeout/Ts.base)');
L(n, 'Age/1', 'Link_lost/1');
LOGIC(n, 'RC_ok', 'NOR', 2);
L(n, 'Link_lost/1', 'RC_ok/1', 'rc_link_lost'); L(n, 'Latch/5', 'RC_ok/2', 'failsafe');
CMP(n, 'LQ_warn', '<', 'P.electronics.sbus.lq_warn');
L(n, 'Latch/7', 'LQ_warn/1', 'lq');
% ---- output: virtual bus rc_dec ----
BCV(n, 'RcDec_BC', {'ch_raw', 'Latch/1'; 'ch17', 'Latch/2'; 'ch18', 'Latch/3'; 'frame_lost', 'Latch/4'; ...
    'failsafe', 'Latch/5'; 'frame_new', 'Frame_Sync/3'; 'frame_valid', 'Frame_valid/1'; 'last_valid', 'Latch/6'; ...
    'rc_age', 'Age_Ts/1'; 'rc_link_lost', 'Link_lost/1'; 'rc_ok', 'RC_ok/1'; 'lq', 'Latch/7'; ...
    'lq_warn', 'LQ_warn/1'; 'n_ok', 'Frame_Sync/4'; 'n_bad', 'Frame_Sync/5'; 'n_skip', 'Frame_Sync/6'});
OUT(n, 'rc_dec', 1, '');
L(n, 'RcDec_BC/1', 'rc_dec/1', 'rc_dec');
set_param(n, 'Description', ['fw_sbus_decoder (T08, Task_1k, docs/specs/electronics.md 2.6.3 / 4.1). in 1 sbus ' ...
    'Bus_SbusStream (Ts.base). out 1 rc_dec virtual bus (Ts.base): ch_raw uint16[16] (latched, IC raw_mid), ch17, ' ...
    'ch18, frame_lost, failsafe (boolean, flags of the last frame), frame_new (boolean pulse), frame_valid (= frame_new ' ...
    'AND NOT frame_lost AND NOT failsafe), last_valid (latched validity of the last frame), rc_age single s (restarts ' ...
    'only on valid frames, saturates at P.electronics.sbus.age_sat, starts saturated), rc_link_lost (rc_age > ' ...
    'link_timeout, integer ticks), rc_ok (NOT rc_link_lost AND NOT failsafe), lq single (first-order filter of ' ...
    '1 - frame_lost per frame, alpha lq_alpha), lq_warn (lq < lq_warn), n_ok, n_bad, n_skip uint32 (wrapping). ' ...
    'MATLAB Function Frame_Sync -> subsystems/electronics/sbus_frame_sync.m (rev. C: a frame is accepted only with no ' ...
    'uart_err on any chunk that carried one of its bytes, exactly 25 bytes, valid footer, reserved flag bits 0 and every ' ...
    'channel in rcmap.raw_min..raw_max; n_bad counts rejected frames); unpack in the Enabled Subsystem Latch: ' ...
    'commutil2 Integer to Bit (8, LSB first) -> Bit to Integer (11, LSB first, uint16).']);
end

% ===========================================================================================
% fw_rc_mapping (Task_10)
% ===========================================================================================
function local_fw_rc_mapping(n)
CT = 'Ts.ctrl';
IN(n, 'rc_dec', 1, '', CT);
% ---- normalise: x = sat(sign*(raw - mid)/819.5, -1, 1) ----
DTC(n, 'Raw_s', 'single');
A(n, 'simulink/Math Operations/Bias', 'Mid', 'Bias', '-single(P.electronics.rcmap.raw_mid)');
A(n, 'simulink/Math Operations/Gain', 'Norm', 'Gain', 'single(P.electronics.rcmap.sign/P.electronics.rcmap.raw_half)', ...
    'Multiplication', 'Element-wise(K.*u)');
A(n, 'simulink/Discontinuities/Saturation', 'Sat_norm', 'UpperLimit', '1', 'LowerLimit', '-1');
PICK(n, 'rc_dec/1', 'Sel_dec_raw', {'ch_raw'}, {'Raw_s/1'}, 'rc_dec');
L(n, 'Raw_s/1', 'Mid/1'); L(n, 'Mid/1', 'Norm/1'); L(n, 'Norm/1', 'Sat_norm/1');
chans = {'steer', 'thr', 'mode', 'arm', 'speed', 'impl', 'reset', 'lights', 'lift', 'estop', 'mission', 'rth'};
idx = strjoin(cellfun(@(c) ['P.electronics.rcmap.ch.' c], chans, 'UniformOutput', false), ' ');
A(n, 'simulink/Signal Routing/Selector', 'Sel_ch', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', ['[' idx ']'], 'InputPortWidth', '16');
A(n, 'simulink/Signal Routing/Demux', 'Ch', 'Outputs', num2str(numel(chans)));
L(n, 'Sat_norm/1', 'Sel_ch/1', 'ch_norm'); L(n, 'Sel_ch/1', 'Ch/1');
c = @(nm) sprintf('Ch/%d', find(strcmp(chans, nm)));
% ---- sticks: Dead Zone + Gain 1/(1-d) + Polynomial expo [e 0 1-e 0] ----
for ax = {'thr', 'steer'}
    a = ax{1};
    A(n, 'simulink/Discontinuities/Dead Zone', ['DZ_' a], 'LowerValue', '-P.electronics.rcmap.deadband', ...
        'UpperValue', 'P.electronics.rcmap.deadband');
    A(n, 'simulink/Math Operations/Gain', ['DZ_rescale_' a], 'Gain', '1/(1-P.electronics.rcmap.deadband)');
    e = ['P.electronics.rcmap.expo_' a];
    A(n, 'simulink/Math Operations/Polynomial', ['Expo_' a], 'Coefs', sprintf('[%s 0 1-%s 0]', e, e));
    L(n, c(a), ['DZ_' a '/1'], ['x_' a]); L(n, ['DZ_' a '/1'], ['DZ_rescale_' a '/1']);
    L(n, ['DZ_rescale_' a '/1'], ['Expo_' a '/1']);
end
% ---- speed scale s = s_min + (x+1)/2*(1 - s_min) ----
A(n, 'simulink/Math Operations/Bias', 'Spd_p1', 'Bias', '1');
A(n, 'simulink/Math Operations/Gain', 'Spd_half', 'Gain', '(1-P.electronics.rcmap.s_min)/2');
A(n, 'simulink/Math Operations/Bias', 'Spd_min', 'Bias', 'P.electronics.rcmap.s_min');
L(n, c('speed'), 'Spd_p1/1', 'x_speed'); L(n, 'Spd_p1/1', 'Spd_half/1'); L(n, 'Spd_half/1', 'Spd_min/1');
% ---- v_rc = thr*v_max_man*s, w_rc = steer_to_omega_sign*steer*w_max_man*s ----
A(n, 'simulink/Math Operations/Product', 'Thr_s', 'Inputs', '2');
A(n, 'simulink/Math Operations/Gain', 'V_rc', 'Gain', 'P.electronics.lim.v_max_man');
A(n, 'simulink/Math Operations/Product', 'Steer_s', 'Inputs', '2');
A(n, 'simulink/Math Operations/Gain', 'W_rc', 'Gain', 'P.electronics.rcmap.steer_to_omega_sign*P.electronics.lim.w_max_man');
L(n, 'Expo_thr/1', 'Thr_s/1', 'thr'); L(n, 'Spd_min/1', 'Thr_s/2', 'speed_scale'); L(n, 'Thr_s/1', 'V_rc/1');
L(n, 'Expo_steer/1', 'Steer_s/1', 'steer'); L(n, 'Spd_min/1', 'Steer_s/2'); L(n, 'Steer_s/1', 'W_rc/1');
% ---- 3-position decode pos = 1 + (x > -th) + (x > th), Tapped Delay debounce ----
sw3 = {'mode', 1; 'impl', 2; 'lift', 3; 'mission', 4};
for k = 1:size(sw3, 1)
    a = sw3{k, 1};
    CMP(n, ['Pos_lo_' a], '>', '-P.electronics.rcmap.sw_thresh');
    CMP(n, ['Pos_hi_' a], '>', 'P.electronics.rcmap.sw_thresh');
    DTC(n, ['Pos_lo_u8_' a], 'uint8'); DTC(n, ['Pos_hi_u8_' a], 'uint8');
    CST(n, ['Pos_one_' a], '1', 'uint8', '-1');
    A(n, 'simulink/Math Operations/Sum', ['Pos_' a], 'Inputs', '+++', 'OutDataTypeStr', 'uint8');
    L(n, c(a), ['Pos_lo_' a '/1'], ['x_' a]); L(n, c(a), ['Pos_hi_' a '/1']);
    L(n, ['Pos_lo_' a '/1'], ['Pos_lo_u8_' a '/1']); L(n, ['Pos_hi_' a '/1'], ['Pos_hi_u8_' a '/1']);
    L(n, ['Pos_one_' a '/1'], ['Pos_' a '/1']); L(n, ['Pos_lo_u8_' a '/1'], ['Pos_' a '/2']);
    L(n, ['Pos_hi_u8_' a '/1'], ['Pos_' a '/3']);
    DEBOUNCE(n, ['Deb_' a], sprintf('P.electronics.rcmap.debounce_ic(%d)', sw3{k, 2}));
    L(n, ['Pos_' a '/1'], ['Deb_' a '/1'], ['pos_raw_' a]);
end
A(n, 'simulink/Math Operations/Sum', 'Mode_sw', 'Inputs', '+-', 'OutDataTypeStr', 'uint8');
CST(n, 'One_u8', '1', 'uint8', '-1');
L(n, 'Deb_mode/1', 'Mode_sw/1', 'sw_mode'); L(n, 'One_u8/1', 'Mode_sw/2');
% ---- 2-position switches (x > sw2_thresh) ----
for a = {'arm', 'reset', 'lights', 'estop', 'rth'}
    CMP(n, ['On_' a{1}], '>', 'P.electronics.rcmap.sw2_thresh');
    L(n, c(a{1}), ['On_' a{1} '/1'], ['x_' a{1}]);
end
A(n, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Arm_edge', 'vinit', '0');
L(n, 'On_arm/1', 'Arm_edge/1', 'arm');
% ---- remote E-stop CH12: threshold, no debounce (estop_debounce_n = 0), updated only on valid frames (D34) ----
s = ENSUB(n, 'Estop_SH', {'estop_raw'}, {'estop_remote', 'false'});
L(s, 'estop_raw/1', 'estop_remote/1', 'estop_raw');
L(n, 'On_estop/1', 'Estop_SH/1', 'estop_raw');
PICK(n, 'rc_dec/1', 'Sel_dec_valid', {'last_valid'}, {'Estop_SH/enable'});
% ---- sticks_neutral, override (persistence counter) ----
A(n, 'simulink/Math Operations/Abs', 'Abs_thr'); A(n, 'simulink/Math Operations/Abs', 'Abs_steer');
L(n, c('thr'), 'Abs_thr/1'); L(n, c('steer'), 'Abs_steer/1');
CMP(n, 'Neutral_thr', '<', 'P.electronics.rcmap.neutral_tol'); CMP(n, 'Neutral_steer', '<', 'P.electronics.rcmap.neutral_tol');
LOGIC(n, 'Sticks_neutral', 'AND', 2);
L(n, 'Abs_thr/1', 'Neutral_thr/1'); L(n, 'Abs_steer/1', 'Neutral_steer/1');
L(n, 'Neutral_thr/1', 'Sticks_neutral/1'); L(n, 'Neutral_steer/1', 'Sticks_neutral/2');
CMP(n, 'Ovr_thr', '>', 'P.electronics.rcmap.override_tol'); CMP(n, 'Ovr_steer', '>', 'P.electronics.rcmap.override_tol');
LOGIC(n, 'Ovr_cond', 'OR', 2);
L(n, 'Abs_thr/1', 'Ovr_thr/1'); L(n, 'Abs_steer/1', 'Ovr_steer/1');
L(n, 'Ovr_thr/1', 'Ovr_cond/1'); L(n, 'Ovr_steer/1', 'Ovr_cond/2');
LOGIC(n, 'Ovr_not', 'NOT', 1);
L(n, 'Ovr_cond/1', 'Ovr_not/1', 'ovr_cond');
AGE(n, 'Ovr_cnt', 'P.electronics.rcmap.override_n_sat', '0');
L(n, 'Ovr_not/1', 'Ovr_cnt/1');
CMP(n, 'Override', '>', 'round(P.electronics.rcmap.override_t/Ts.ctrl)');
L(n, 'Ovr_cnt/1', 'Override/1', 'ovr_ticks');
% ---- mission CH13 (debounced 3-pos) edges -> START (pos 2) / PAUSE (pos 3); CH14 rise -> RTH ----
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'Mis_chg', 'vinit', 'P.electronics.rcmap.debounce_ic(4)');
CMP(n, 'Mis_pos2', '==', '2'); CMP(n, 'Mis_pos3', '==', '3');
LOGIC(n, 'Evt_start', 'AND', 2); LOGIC(n, 'Evt_pause', 'AND', 2);
L(n, 'Deb_mission/1', 'Mis_chg/1', 'sw_mission'); L(n, 'Deb_mission/1', 'Mis_pos2/1'); L(n, 'Deb_mission/1', 'Mis_pos3/1');
L(n, 'Mis_chg/1', 'Evt_start/1'); L(n, 'Mis_pos2/1', 'Evt_start/2');
L(n, 'Mis_chg/1', 'Evt_pause/1'); L(n, 'Mis_pos3/1', 'Evt_pause/2');
A(n, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Evt_rth', 'vinit', '0');
L(n, 'On_rth/1', 'Evt_rth/1', 'rth');
CST(n, 'Code_start', 'P.electronics.rcmap.aux_code(1)', 'uint8', '-1');
CST(n, 'Code_pause', 'P.electronics.rcmap.aux_code(2)', 'uint8', '-1');
CST(n, 'Code_rth', 'P.electronics.rcmap.aux_code(3)', 'uint8', '-1');
A(n, 'simulink/Discrete/Unit Delay', 'Aux_prev', 'InitialCondition', '0', 'SampleTime', '-1');
A(n, 'simulink/Signal Routing/Switch', 'Aux_rth', 'Criteria', 'u2 ~= 0');
A(n, 'simulink/Signal Routing/Switch', 'Aux_pause', 'Criteria', 'u2 ~= 0');
A(n, 'simulink/Signal Routing/Switch', 'Aux_start', 'Criteria', 'u2 ~= 0');
L(n, 'Code_rth/1', 'Aux_rth/1'); L(n, 'Evt_rth/1', 'Aux_rth/2', 'evt_rth'); L(n, 'Aux_prev/1', 'Aux_rth/3', 'aux_prev');
L(n, 'Code_pause/1', 'Aux_pause/1'); L(n, 'Evt_pause/1', 'Aux_pause/2', 'evt_pause'); L(n, 'Aux_rth/1', 'Aux_pause/3');
L(n, 'Code_start/1', 'Aux_start/1'); L(n, 'Evt_start/1', 'Aux_start/2', 'evt_start'); L(n, 'Aux_pause/1', 'Aux_start/3');
L(n, 'Aux_start/1', 'Aux_prev/1', 'rc_aux_cmd');
LOGIC(n, 'Evt_any', 'OR', 3);
L(n, 'Evt_start/1', 'Evt_any/1'); L(n, 'Evt_pause/1', 'Evt_any/2'); L(n, 'Evt_rth/1', 'Evt_any/3');
DTC(n, 'Evt_u8', 'uint8');
A(n, 'simulink/Math Operations/Sum', 'Seq_sum', 'Inputs', '++', 'OutDataTypeStr', 'uint8', 'SaturateOnIntegerOverflow', 'off');
A(n, 'simulink/Discrete/Unit Delay', 'Seq_prev', 'InitialCondition', '0', 'SampleTime', '-1');
L(n, 'Evt_any/1', 'Evt_u8/1'); L(n, 'Evt_u8/1', 'Seq_sum/1'); L(n, 'Seq_prev/1', 'Seq_sum/2', 'seq_prev');
L(n, 'Seq_sum/1', 'Seq_prev/1', 'rc_aux_seq');
% ---- output: virtual bus rc (the decoder status passed through by Sel_dec_pass, right in front of Rc_BC) ----
pass = {'rc_ok', 'rc_link_lost', 'failsafe', 'frame_lost', 'last_valid', 'lq', 'lq_warn', 'rc_age', 'n_ok', 'n_bad', 'n_skip'};
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_dec_pass', 'OutputSignals', strjoin(pass, ','));
L(n, 'rc_dec/1', 'Sel_dec_pass/1');
BCV(n, 'Rc_BC', [{'ch_norm', 'Sat_norm/1'; 'thr', 'Expo_thr/1'; 'steer', 'Expo_steer/1'; 'speed_scale', 'Spd_min/1'; ...
    'v_rc', 'V_rc/1'; 'w_rc', 'W_rc/1'; 'sw_mode', 'Deb_mode/1'; 'rc_mode_sw', 'Mode_sw/1'; ...
    'sw_impl', 'Deb_impl/1'; 'sw_lift', 'Deb_lift/1'; 'sw_mission', 'Deb_mission/1'; 'arm', 'On_arm/1'; ...
    'arm_req', 'Arm_edge/1'; 'sticks_neutral', 'Sticks_neutral/1'; 'override', 'Override/1'; ...
    'estop_remote', 'Estop_SH/1'; 'reset_req', 'On_reset/1'; 'lights', 'On_lights/1'; 'rc_aux_cmd', 'Aux_start/1'; ...
    'rc_aux_seq', 'Seq_sum/1'}; [pass(:), arrayfun(@(k) sprintf('Sel_dec_pass/%d', k), (1:numel(pass))', ...
    'UniformOutput', false)]]);
OUT(n, 'rc', 1, '');
L(n, 'Rc_BC/1', 'rc/1', 'rc');
set_param(n, 'Description', ['fw_rc_mapping (T08, Task_10, spec 2.7.2, ICD 7, D34). in 1 rc_dec (virtual bus of ' ...
    'fw_sbus_decoder through RT 0.001 -> 0.01). out 1 rc virtual bus (Ts.ctrl): ch_norm single[16] (sat(sign*(raw-mid)/' ...
    '819.5)); thr, steer single (Dead Zone +/-deadband, Gain 1/(1-d), Polynomial [e 0 1-e 0], e = expo_thr / expo_steer); ' ...
    'speed_scale single (s_min + (x7+1)/2*(1-s_min)); v_rc = thr*v_max_man*s [m/s]; w_rc = steer_to_omega_sign*steer*' ...
    'w_max_man*s [rad/s]; sw_mode, sw_impl, sw_lift, sw_mission uint8 1..3 (pos = 1 + (x > -sw_thresh) + (x > sw_thresh), ' ...
    'accepted when the last debounce_n Tapped Delay samples agree); rc_mode_sw uint8 = sw_mode - 1 (0 MANUAL 1 HOLD ' ...
    '2 AUTO); arm (CH6 x > sw2_thresh), arm_req (Detect Rise Positive); sticks_neutral (|x_thr|, |x_steer| < ' ...
    'neutral_tol); override (|x_thr| or |x_steer| > override_tol for > override_t); estop_remote (CH12 x > sw2_thresh, ' ...
    'no debounce, Enabled sample-and-hold updated only while last_valid, i.e. frames with frame_lost = 0 and failsafe = 0); ' ...
    'reset_req (CH9 level); lights (CH10); rc_aux_cmd uint8 (held MissionCmd: 2 START on CH13 edge to pos 2, 3 PAUSE ' ...
    'on edge to pos 3, 6 RTH on CH14 rise) and rc_aux_seq uint8 (+1 per edge); pass-through of rc_ok, rc_link_lost, ' ...
    'failsafe, frame_lost, last_valid, lq, lq_warn, rc_age, n_ok, n_bad, n_skip. Channel indices from ' ...
    'P.electronics.rcmap.ch.']);
end

% ===========================================================================================
% fw_imu_tilt (Task_5, D47)
% ===========================================================================================
function local_fw_imu_tilt(n)
IT = 'Ts.imu';
IN(n, 'imu_meas', 1, 'Bus: Bus_ImuMeas', IT);
IN(n, 'a_track', 2, 'single', IT);
IN(n, 'v_track_LR', 3, 'single', IT, '2');
% mean track speed (centripetal term)
A(n, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(n, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5');
L(n, 'v_track_LR/1', 'V_sum/1', 'v_track_LR'); L(n, 'V_sum/1', 'V_mean/1', 'v_sum');
% gyro z (one Bus Selector of imu_meas per consumer, spec 6.2)
A(n, 'simulink/Signal Routing/Selector', 'Gyro_z', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '3', 'InputPortWidth', '3');
PICK(n, 'imu_meas/1', 'Sel_imu_gyro', {'gyro_B'}, {'Gyro_z/1'}, 'imu_meas');
% linear acceleration [a_track; v_track*gyro_z; 0] (body FLU), scaled by tilt_comp.enable
A(n, 'simulink/Math Operations/Product', 'Centripetal', 'Inputs', '2');
L(n, 'V_mean/1', 'Centripetal/1', 'v_track'); L(n, 'Gyro_z/1', 'Centripetal/2', 'gyro_z');
CST(n, 'Zero_z', '0', 'single', '-1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'A_lin', 'NumInputs', '3');
L(n, 'a_track/1', 'A_lin/1', 'a_track'); L(n, 'Centripetal/1', 'A_lin/2', 'a_cent'); L(n, 'Zero_z/1', 'A_lin/3', 'a_z');
A(n, 'simulink/Math Operations/Gain', 'Comp_enable', 'Gain', 'single(P.electronics.safety.tilt_comp.enable)');
L(n, 'A_lin/1', 'Comp_enable/1', 'a_lin');
A(n, 'simulink/Math Operations/Sum', 'F_comp', 'Inputs', '+-');
PICK(n, 'imu_meas/1', 'Sel_imu_accel', {'accel_B'}, {'F_comp/1'});
L(n, 'Comp_enable/1', 'F_comp/2', 'a_lin_comp');
% Gain -1 directly at the library filter (negative accelerometer convention, ICD 1)
A(n, 'simulink/Math Operations/Gain', 'Neg_conv', 'Gain', '-1');
A(n, 'simulink/Math Operations/Reshape', 'Acc_row', 'OutputDimensionality', 'Customize', 'OutputDimensions', '[1 3]');
A(n, 'simulink/Math Operations/Reshape', 'Gyr_row', 'OutputDimensionality', 'Customize', 'OutputDimensions', '[1 3]');
L(n, 'F_comp/1', 'Neg_conv/1', 'f_comp'); L(n, 'Neg_conv/1', 'Acc_row/1', 'f_neg');
PICK(n, 'imu_meas/1', 'Sel_imu_gyro_cf', {'gyro_B'}, {'Gyr_row/1'});
A(n, 'mspfiltlib/Complementary Filter', 'CF', 'OrientationFormat', 'Rotation matrix', ...
    'AccelerometerGain', 'P.electronics.safety.cf_accel_gain', 'HasMagnetometer', 'off', 'DispRefFrame', 'ENU', ...
    'SimulateUsing', 'Code generation');
L(n, 'Acc_row/1', 'CF/1', 'accel_neg'); L(n, 'Gyr_row/1', 'CF/2', 'gyro_row');
A(n, 'simulink/Sinks/Terminator', 'T_cf_w'); L(n, 'CF/2', 'T_cf_w/1');
% CF output = frame rotation ENU -> body; its transpose is the body -> ENU matrix R of the formulas
A(n, 'simulink/Math Operations/Math Function', 'R_bw', 'Operator', 'transpose');
A(n, 'simulink/Math Operations/Reshape', 'R_vec', 'OutputDimensionality', '1-D array');
A(n, 'simulink/Signal Routing/Selector', 'R_3x', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '[3 6 9]', 'InputPortWidth', '9');
A(n, 'simulink/Signal Routing/Demux', 'R_dmx', 'Outputs', '3');
L(n, 'CF/1', 'R_bw/1', 'R_frame'); L(n, 'R_bw/1', 'R_vec/1', 'R'); L(n, 'R_vec/1', 'R_3x/1'); L(n, 'R_3x/1', 'R_dmx/1');
% roll = atan2(R32, R33), pitch = -asin(R31)
A(n, 'simulink/Math Operations/Trigonometric Function', 'Roll', 'Operator', 'atan2');
L(n, 'R_dmx/2', 'Roll/1', 'R32'); L(n, 'R_dmx/3', 'Roll/2', 'R33');
A(n, 'simulink/Discontinuities/Saturation', 'R31_sat', 'UpperLimit', '1', 'LowerLimit', '-1');
A(n, 'simulink/Math Operations/Trigonometric Function', 'Asin', 'Operator', 'asin');
A(n, 'simulink/Math Operations/Gain', 'Pitch', 'Gain', '-1');
L(n, 'R_dmx/1', 'R31_sat/1', 'R31'); L(n, 'R31_sat/1', 'Asin/1'); L(n, 'Asin/1', 'Pitch/1');
% tilt_gate = |a_track| < gate_a
A(n, 'simulink/Math Operations/Abs', 'Abs_a');
CMP(n, 'Tilt_gate', '<', 'P.electronics.safety.tilt_comp.gate_a');
L(n, 'a_track/1', 'Abs_a/1'); L(n, 'Abs_a/1', 'Tilt_gate/1');
% gyro-z bias: first-order average (tau_bias) while BOTH tracks stand still (max |v_track_L,R| <
% yaw.standstill_v); a pivot turn has mean track speed 0 but turns, so it must not enter the bias
A(n, 'simulink/Math Operations/Abs', 'Abs_v');
A(n, 'simulink/Math Operations/MinMax', 'Vt_max', 'Function', 'max', 'Inputs', '1');
CMP(n, 'Standstill', '<', 'P.electronics.yaw.standstill_v');
L(n, 'v_track_LR/1', 'Abs_v/1'); L(n, 'Abs_v/1', 'Vt_max/1'); L(n, 'Vt_max/1', 'Standstill/1');
s = ENSUB(n, 'Bias_avg', {'gz'}, {'bias', 'single(0)'});
A(s, 'simulink/Discrete/Transfer Fcn First Order', 'LP', 'PoleZ', 'exp(-Ts.imu/P.electronics.yaw.tau_bias)', ...
    'ICPrevOutput', '0');
L(s, 'gz/1', 'LP/1', 'gz'); L(s, 'LP/1', 'bias/1', 'bias');
L(n, 'Gyro_z/1', 'Bias_avg/1');
ENLINE(n, 'Standstill/1', 'Bias_avg', 'standstill');
A(n, 'simulink/Math Operations/Sum', 'Yaw_rate', 'Inputs', '+-');
L(n, 'Gyro_z/1', 'Yaw_rate/1'); L(n, 'Bias_avg/1', 'Yaw_rate/2', 'gyro_bias');
OUT(n, 'roll', 1, 'single'); OUT(n, 'pitch', 2, 'single'); OUT(n, 'tilt_gate', 3, 'boolean');
OUT(n, 'yaw_rate_meas', 4, 'single'); OUT(n, 'gyro_bias', 5, 'single');
% level calibration (commissioning offsets of the installed IMU, P.electronics.imu_cal)
A(n, 'simulink/Math Operations/Bias', 'Roll_cal', 'Bias', '-P.electronics.imu_cal.roll0');
A(n, 'simulink/Math Operations/Bias', 'Pitch_cal', 'Bias', '-P.electronics.imu_cal.pitch0');
L(n, 'Roll/1', 'Roll_cal/1', 'roll_raw'); L(n, 'Pitch/1', 'Pitch_cal/1', 'pitch_raw');
L(n, 'Roll_cal/1', 'roll/1', 'roll'); L(n, 'Pitch_cal/1', 'pitch/1', 'pitch'); L(n, 'Tilt_gate/1', 'tilt_gate/1', 'tilt_gate');
L(n, 'Yaw_rate/1', 'yaw_rate_meas/1', 'yaw_rate_meas'); L(n, 'Bias_avg/1', 'gyro_bias/1');
set_param(n, 'Description', ['fw_imu_tilt (T08, Task_5, D47). in 1 imu_meas Bus_ImuMeas, 2 a_track single m/s^2 ' ...
    '(sens_odom_compute, RT 0.01 -> 0.005), 3 v_track_LR single[2] m/s (encoder track speeds L R, RT 0.01 -> 0.005), all ' ...
    'Ts.imu. out 1 roll, 2 pitch (rad, + nose down), 3 tilt_gate boolean (|a_track| < tilt_comp.gate_a), 4 yaw_rate_meas ' ...
    '(gyro_z - bias), 5 gyro_bias (rad/s). v_track = mean(v_track_LR). Physical accel minus enable*[a_track; v_track*gyro_z; 0] -> Gain -1 -> ' ...
    'mspfiltlib/Complementary Filter (Rotation matrix, AccelerometerGain cf_accel_gain, no magnetometer, ENU, Code ' ...
    'generation); R = transpose(filter output) (body -> ENU), roll = atan2(R32,R33) - imu_cal.roll0, pitch = ' ...
    '-asin(R31) - imu_cal.pitch0 (level calibration of the installed IMU). Gyro-z bias = ' ...
    'Transfer Fcn First Order (pole exp(-Ts/tau_bias)) in an Enabled Subsystem active while BOTH tracks stand still, ' ...
    'max|v_track_L,R| < yaw.standstill_v (a pivot turn, mean track speed 0, is not a standstill).']);
end

% ===========================================================================================
% fw_can1_codec (10 / 50 / 100 ms)
% ===========================================================================================
function local_fw_can1_codec(n)
IN(n, 'esc_fast', 1, 'Bus: Bus_Can1_EscFast', 'Ts.ctrl');
IN(n, 'esc_slow', 2, 'Bus: Bus_Can1_EscSlow', 'Ts.comm');
IN(n, 'impl_rx', 3, 'Bus: Bus_Can1_Impl', 'Ts.comm');
IN(n, 'bms_rx', 4, 'Bus: Bus_Can1_Bms', 'Ts.hk');
IN(n, 'drive_cmd', 5, 'Bus: Bus_DriveCmd', 'Ts.ctrl');
IN(n, 'impl_enable', 6, 'boolean', 'Ts.comm');
IN(n, 'impl_fault_reset', 7, 'boolean', 'Ts.comm');
IN(n, 'impl_speed_rpm', 8, 'single', 'Ts.comm');
IN(n, 'bms_req_flags', 9, 'boolean', 'Ts.comm', '4');
% ---- drive feedback (10 ms; FB2 re-timed 0.05 -> 0.01); one Bus Selector of esc_fast / esc_slow per consumer ----
sides = {'L', 1; 'R', 2};
for k = 1:2
    sd = sides{k, 1};
    RT(n, ['RT_fb2_' sd], 'Ts.ctrl', 'P.ic.Bus_CanFrame8');
    PICK(n, 'esc_slow/1', ['Sel_slow_' sd], {['fb2_' sd]}, {['RT_fb2_' sd '/1']}, 'esc_slow');
    A(n, 'simulink/Ports & Subsystems/Subsystem Reference', ['DFB_' sd], 'ReferencedSubsystem', 'drive_fb_unpack');
    CST(n, ['Node_' sd], num2str(sides{k, 2}), 'uint8', '-1');
    PICK(n, 'esc_fast/1', ['Sel_fast_' sd], {['fb1_' sd], ['fb3_' sd]}, {['DFB_' sd '/1'], ['DFB_' sd '/3']}, 'esc_fast');
    L(n, ['RT_fb2_' sd '/1'], ['DFB_' sd '/2'], ['fb2_' sd '_10ms']);
    L(n, ['Node_' sd '/1'], ['DFB_' sd '/4'], ['node_' sd]);
    % FB1 / FB3 ages (samples) -> drv_age = max * Ts.ctrl
    AGE(n, ['Age1_' sd], 'round(P.electronics.can.age_sat_s/Ts.ctrl)');
    AGE(n, ['Age3_' sd], 'round(P.electronics.can.age_sat_s/Ts.ctrl)');
    PICK(n, 'esc_fast/1', ['V1_' sd], {['fb1_' sd '.valid']}, {['Age1_' sd '/1']});
    PICK(n, 'esc_fast/1', ['V3_' sd], {['fb3_' sd '.valid']}, {['Age3_' sd '/1']});
    A(n, 'simulink/Math Operations/MinMax', ['AgeMax_' sd], 'Function', 'max', 'Inputs', '2');
    L(n, ['Age1_' sd '/1'], ['AgeMax_' sd '/1'], ['age_fb1_' sd]); L(n, ['Age3_' sd '/1'], ['AgeMax_' sd '/2'], ['age_fb3_' sd]);
end
A(n, 'simulink/Signal Routing/Bus Creator', 'Fb_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_DriveFb', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'DFB_L/1', 'Fb_BC/1', 'L'); L(n, 'DFB_R/1', 'Fb_BC/2', 'R');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Age_LR', 'NumInputs', '2');
L(n, 'AgeMax_L/1', 'Age_LR/1', 'age_max_L'); L(n, 'AgeMax_R/1', 'Age_LR/2', 'age_max_R');
DTC(n, 'Age_LR_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Age_LR_Ts', 'Gain', 'Ts.ctrl');
L(n, 'Age_LR/1', 'Age_LR_s/1', 'age_ticks_LR'); L(n, 'Age_LR_s/1', 'Age_LR_Ts/1', 'age_ticks_LR_s');
% ---- drive commands (10 ms); one Bus Selector of drive_cmd per consumer ----
for k = 1:2
    sd = sides{k, 1};
    A(n, 'simulink/Ports & Subsystems/Subsystem Reference', ['DCP_' sd], 'ReferencedSubsystem', 'drive_cmd_pack');
    PICK(n, 'drive_cmd/1', ['Sel_cmd_' sd], {sd}, {['DCP_' sd '/1']}, 'drive_cmd');
    L(n, ['Node_' sd '/1'], ['DCP_' sd '/2']);
end
A(n, 'simulink/Signal Routing/Bus Creator', 'Fast_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_McuFast', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'DCP_L/1', 'Fast_BC/1', 'drv_cmd_L'); L(n, 'DCP_R/1', 'Fast_BC/2', 'drv_cmd_R');
% ---- implement command + BMS request (50 ms), alive +1 per frame ----
A(n, 'simulink/Sources/Counter Free-Running', 'Alive_impl', 'NumBits', '8', 'tsamp', 'Ts.comm');
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'Impl_pack', 'ReferencedSubsystem', 'implcmd_can_pack');
L(n, 'impl_enable/1', 'Impl_pack/1', 'impl_enable'); L(n, 'impl_fault_reset/1', 'Impl_pack/2', 'impl_fault_reset');
L(n, 'Alive_impl/1', 'Impl_pack/3', 'alive_impl'); L(n, 'impl_speed_rpm/1', 'Impl_pack/4', 'impl_speed_rpm');
A(n, 'simulink/Sources/Counter Free-Running', 'Alive_bms', 'NumBits', '8', 'tsamp', 'Ts.comm');
A(n, 'simulink/Signal Routing/Demux', 'Req_dmx', 'Outputs', '4');
L(n, 'bms_req_flags/1', 'Req_dmx/1', 'bms_req_flags');
A(n, 'simulink/Signal Routing/Bus Creator', 'Req_BC', 'Inputs', '5', 'OutDataTypeStr', 'Bus: Bus_BmsReq', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Alive_bms/1', 'Req_BC/1', 'AliveCounter'); L(n, 'Req_dmx/1', 'Req_BC/2', 'HV_Enable_Req');
L(n, 'Req_dmx/2', 'Req_BC/3', 'Charge_Enable_Req'); L(n, 'Req_dmx/3', 'Req_BC/4', 'Fault_Reset_Req');
L(n, 'Req_dmx/4', 'Req_BC/5', 'Shutdown_Req');
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'Bms_req_pack', 'ReferencedSubsystem', 'bmsreq_can_pack');
L(n, 'Req_BC/1', 'Bms_req_pack/1', 'bms_req');
A(n, 'simulink/Signal Routing/Bus Creator', 'Slow_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_McuSlow', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Impl_pack/1', 'Slow_BC/1', 'impl_cmd'); L(n, 'Bms_req_pack/1', 'Slow_BC/2', 'bms_req');
% ---- implement feedback (50 ms) + age / rx_ok ----
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_impl', 'OutputSignals', 'impl_fb');
L(n, 'impl_rx/1', 'Sel_impl/1', 'impl_rx');
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'Impl_unpack', 'ReferencedSubsystem', 'implfb_can_unpack');
L(n, 'Sel_impl/1', 'Impl_unpack/1');
AGE(n, 'Age_impl', 'round(P.electronics.can.age_sat_s/Ts.comm)');
L(n, 'Impl_unpack/9', 'Age_impl/1', 'valid');            % element valid of the virtual bus impl (Impl_BC)
DTC(n, 'Age_impl_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Age_impl_Ts', 'Gain', 'Ts.comm');
L(n, 'Age_impl/1', 'Age_impl_s/1', 'impl_age_ticks'); L(n, 'Age_impl_s/1', 'Age_impl_Ts/1', 'impl_age_ticks_s');
CMP(n, 'Impl_rx_ok', '<=', 'round(P.electronics.safety.impl_timeout/Ts.comm)');
L(n, 'Age_impl/1', 'Impl_rx_ok/1');
BCV(n, 'Impl_BC', {'running', 'Impl_unpack/1'; 'fault', 'Impl_unpack/2'; 'ready', 'Impl_unpack/3'; ...
    'impl_type', 'Impl_unpack/4'; 'speed_rpm', 'Impl_unpack/5'; 'current_A', 'Impl_unpack/6'; ...
    'temp_degC', 'Impl_unpack/7'; 'alive', 'Impl_unpack/8'; 'valid', 'Impl_unpack/9'; 'age', 'Age_impl_Ts/1'; ...
    'rx_ok', 'Impl_rx_ok/1'});
% ---- BMS frames (100 ms): status, ages, valid = all frames younger than bms_timeout ----
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'Bms_unpack', 'ReferencedSubsystem', 'bms_can_unpack');
L(n, 'bms_rx/1', 'Bms_unpack/1', 'bms_rx');
A(n, 'simulink/Sinks/Terminator', 'T_bms_extra'); L(n, 'Bms_unpack/3', 'T_bms_extra/1', 'bms_extra');
AGE(n, 'Age_bms', 'round(P.electronics.can.age_sat_s/Ts.hk)');
L(n, 'Bms_unpack/2', 'Age_bms/1', 'bms_frame_valid');
A(n, 'simulink/Math Operations/MinMax', 'Age_bms_max', 'Function', 'max', 'Inputs', '1');
L(n, 'Age_bms/1', 'Age_bms_max/1', 'bms_age_ticks');
DTC(n, 'Age_bms_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Age_bms_Ts', 'Gain', 'Ts.hk');
L(n, 'Age_bms_max/1', 'Age_bms_s/1', 'bms_age_oldest'); L(n, 'Age_bms_s/1', 'Age_bms_Ts/1', 'bms_age_oldest_s');
CMP(n, 'Bms_fresh', '<=', 'round(P.electronics.safety.bms_timeout/Ts.hk)');
L(n, 'Age_bms_max/1', 'Bms_fresh/1');
A(n, 'simulink/Signal Routing/Bus Assignment', 'Bms_valid', 'AssignedSignals', 'valid');
L(n, 'Bms_unpack/1', 'Bms_valid/1', 'bms_status'); L(n, 'Bms_fresh/1', 'Bms_valid/2', 'bms_fresh');
% ---- outputs ----
OUT(n, 'can1_fast_tx', 1, 'Bus: Bus_Can1_McuFast'); OUT(n, 'can1_slow_tx', 2, 'Bus: Bus_Can1_McuSlow');
OUT(n, 'drive_fb', 3, 'Bus: Bus_DriveFb'); OUT(n, 'drv_age', 4, 'single'); OUT(n, 'impl', 5, '');
OUT(n, 'bms', 6, 'Bus: Bus_BmsStatus'); OUT(n, 'bms_age', 7, 'single'); OUT(n, 'bms_frame_valid', 8, 'boolean');
L(n, 'Fast_BC/1', 'can1_fast_tx/1', 'can1_fast_tx'); L(n, 'Slow_BC/1', 'can1_slow_tx/1', 'can1_slow_tx');
L(n, 'Fb_BC/1', 'drive_fb/1', 'drive_fb'); L(n, 'Age_LR_Ts/1', 'drv_age/1', 'drv_age');
L(n, 'Impl_BC/1', 'impl/1', 'impl'); L(n, 'Bms_valid/1', 'bms/1', 'bms');
L(n, 'Age_bms_Ts/1', 'bms_age/1', 'bms_age'); L(n, 'Bms_unpack/2', 'bms_frame_valid/1');
set_param(n, 'Description', ['fw_can1_codec (T08, ICD 4; multirate, top level of Shaddad_MCU). in 1 esc_fast Bus_Can1_EscFast ' ...
    '0.01, 2 esc_slow Bus_Can1_EscSlow 0.05, 3 impl_rx Bus_Can1_Impl 0.05, 4 bms_rx Bus_Can1_Bms 0.1, 5 drive_cmd ' ...
    'Bus_DriveCmd 0.01, 6 impl_enable boolean 0.05, 7 impl_fault_reset boolean 0.05, 8 impl_speed_rpm single 0.05 ' ...
    '(rpm at blade / rotor), 9 bms_req_flags boolean[4] 0.05 [HV_Enable_Req Charge_Enable_Req Fault_Reset_Req ' ...
    'Shutdown_Req]. out 1 can1_fast_tx Bus_Can1_McuFast 0.01 (drive_cmd_pack x2, node 1/2), 2 can1_slow_tx ' ...
    'Bus_Can1_McuSlow 0.05 (implcmd_can_pack, bmsreq_can_pack; alive = Counter Free-Running +1 per frame), 3 drive_fb ' ...
    'Bus_DriveFb 0.01 (drive_fb_unpack x2 at 0.01, FB2 through RT 0.05 -> 0.01, rx_ok inside), 4 drv_age single[2] ' ...
    '0.01 s (max of FB1 / FB3 age, L R), 5 impl virtual bus 0.05 (implfb_can_unpack: running fault ready impl_type ' ...
    'speed_rpm current_A temp_degC alive valid + age [s] + rx_ok = age <= impl_timeout), 6 bms Bus_BmsStatus 0.1 ' ...
    '(bms_can_unpack; valid := all 7 frame ages <= bms_timeout, Bus Assignment), 7 bms_age single 0.1 (oldest frame), ' ...
    '8 bms_frame_valid boolean[7] 0.1 (fresh this sample). Frame ages count samples (saturating integers).']);
end

% ===========================================================================================
% fw_can2_codec (5 / 10 / 50 / 100 ms)
% ===========================================================================================
function local_fw_can2_codec(n)
IN(n, 'comp_rx', 1, 'Bus: Bus_CanFrame64', 'Ts.comm');
IN(n, 'imu_meas', 2, 'Bus: Bus_ImuMeas', 'Ts.imu');
IN(n, 'odom', 3, 'Bus: Bus_OdomMeas', 'Ts.ctrl');
IN(n, 'status', 4, 'Bus: Bus_McuStatus', 'Ts.comm');
IN(n, 't_ms', 5, 'uint32', 'Ts.comm');
IN(n, 'bms', 6, 'Bus: Bus_BmsStatus', 'Ts.hk');
SSR(n, 'Imu_pack', 'can2_mcu_imu_pack'); L(n, 'imu_meas/1', 'Imu_pack/1', 'imu_meas');
SSR(n, 'Odom_pack', 'can2_mcu_odom_pack'); L(n, 'odom/1', 'Odom_pack/1', 'odom');
SSR(n, 'Status_pack', 'can2_mcu_status_pack'); L(n, 'status/1', 'Status_pack/1', 'status'); L(n, 't_ms/1', 'Status_pack/2', 't_ms');
SSR(n, 'Power_pack', 'can2_mcu_power_pack'); L(n, 'bms/1', 'Power_pack/1', 'bms');
SSR(n, 'Comp_unpack', 'can2_comp_cmd_unpack'); L(n, 'comp_rx/1', 'Comp_unpack/1', 'comp_rx');
% companion freshness: age since the last valid frame with a new seq (first valid frame counts as new); one Bus
% Selector of comp_rx.valid and of gnc_cmd.seq per consumer (spec 6.2)
A(n, 'simulink/Discrete/Unit Delay', 'Seq_prev', 'InitialCondition', '0', 'SampleTime', '-1');
A(n, 'simulink/Signal Routing/Switch', 'Seq_hold', 'Criteria', 'u2 ~= 0');
PICK(n, 'Comp_unpack/1', 'Sel_seq_hold', {'seq'}, {'Seq_hold/1'}, 'gnc_cmd');
PICK(n, 'comp_rx/1', 'Sel_valid_hold', {'valid'}, {'Seq_hold/2'});
L(n, 'Seq_prev/1', 'Seq_hold/3', 'seq_prev');
L(n, 'Seq_hold/1', 'Seq_prev/1', 'seq_held');
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'Seq_new', 'Operator', '~=');
PICK(n, 'Comp_unpack/1', 'Sel_seq_new', {'seq'}, {'Seq_new/1'});
L(n, 'Seq_prev/1', 'Seq_new/2');
A(n, 'simulink/Discrete/Unit Delay', 'Seen_prev', 'InitialCondition', 'false', 'SampleTime', '-1');
LOGIC(n, 'Seen', 'OR', 2);
L(n, 'Seen_prev/1', 'Seen/1', 'seen_prev'); PICK(n, 'comp_rx/1', 'Sel_valid_seen', {'valid'}, {'Seen/2'});
L(n, 'Seen/1', 'Seen_prev/1', 'seen');
LOGIC(n, 'First', 'NOT', 1); L(n, 'Seen_prev/1', 'First/1');
LOGIC(n, 'New_or_first', 'OR', 2); L(n, 'Seq_new/1', 'New_or_first/1', 'seq_new'); L(n, 'First/1', 'New_or_first/2', 'first');
LOGIC(n, 'Fresh', 'AND', 2); L(n, 'New_or_first/1', 'Fresh/1', 'new_or_first');
PICK(n, 'comp_rx/1', 'Sel_valid_fresh', {'valid'}, {'Fresh/2'});
AGE(n, 'Age_comp', 'round(P.electronics.can.age_sat_s/Ts.comm)');
L(n, 'Fresh/1', 'Age_comp/1', 'comp_fresh');
DTC(n, 'Age_comp_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Age_comp_Ts', 'Gain', 'Ts.comm');
L(n, 'Age_comp/1', 'Age_comp_s/1', 'comp_age_ticks'); L(n, 'Age_comp_s/1', 'Age_comp_Ts/1', 'comp_age_ticks_s');
CMP(n, 'Comp_ok', '<=', 'round(P.electronics.safety.comp_timeout/Ts.comm)');
L(n, 'Age_comp/1', 'Comp_ok/1');
OUT(n, 'can2_imu', 1, 'Bus: Bus_CanFrame64'); OUT(n, 'can2_odom', 2, 'Bus: Bus_CanFrame64');
OUT(n, 'can2_status', 3, 'Bus: Bus_CanFrame64'); OUT(n, 'can2_power', 4, 'Bus: Bus_CanFrame64');
OUT(n, 'gnc_cmd', 5, 'Bus: Bus_GncCmd'); OUT(n, 'comp_ok', 6, 'boolean'); OUT(n, 'comp_age', 7, 'single');
L(n, 'Imu_pack/1', 'can2_imu/1', 'can2_imu'); L(n, 'Odom_pack/1', 'can2_odom/1', 'can2_odom');
L(n, 'Status_pack/1', 'can2_status/1', 'can2_status'); L(n, 'Power_pack/1', 'can2_power/1', 'can2_power');
L(n, 'Comp_unpack/1', 'gnc_cmd/1'); L(n, 'Comp_ok/1', 'comp_ok/1', 'comp_ok'); L(n, 'Age_comp_Ts/1', 'comp_age/1', 'comp_age');
set_param(n, 'Description', ['fw_can2_codec (T08, ICD 5; multirate, top level of Shaddad_MCU). in 1 comp_rx Bus_CanFrame64 ' ...
    '0.05 (0x100), 2 imu_meas Bus_ImuMeas 0.005, 3 odom Bus_OdomMeas 0.01, 4 status Bus_McuStatus 0.05 (assembled in ' ...
    'Task_50 from Task_10 signals through fast-to-slow RTs), 5 t_ms uint32 0.05, 6 bms Bus_BmsStatus 0.1. out 1 can2_imu ' ...
    '(0x201, 0.005), 2 can2_odom (0x202, 0.01), 3 can2_status (0x200, 0.05), 4 can2_power (0x203, 0.1) Bus_CanFrame64, ' ...
    '5 gnc_cmd Bus_GncCmd 0.05 (can2_comp_cmd_unpack, Task_50; handed to Task_10 by the deterministic RT = GNC compute ' ...
    'delay, D41), 6 comp_ok boolean 0.05 (age <= comp_timeout), 7 comp_age single 0.05 s (since the last valid frame with ' ...
    'a changed seq; the first valid frame counts; saturates, starts saturated).']);
end

% ===========================================================================================
% fw_adc_scaling (Task_50)
% ===========================================================================================
function local_fw_adc_scaling(n)
IN(n, 'adc_raw', 1, 'uint16', 'Ts.comm', '8');
IN(n, 'mcu_temp_degC', 2, 'single', 'Ts.comm');
DTC(n, 'Cnt_s', 'single');
A(n, 'simulink/Math Operations/Gain', 'Volts', 'Gain', 'P.electronics.adc.vref/(2^P.electronics.adc.nbits-1)');
A(n, 'simulink/Signal Routing/Demux', 'V_dmx', 'Outputs', '8');
A(n, 'simulink/Signal Routing/Demux', 'C_dmx', 'Outputs', '8');
L(n, 'adc_raw/1', 'Cnt_s/1', 'adc_raw'); L(n, 'Cnt_s/1', 'Volts/1', 'counts'); L(n, 'Volts/1', 'V_dmx/1', 'v_adc');
L(n, 'Cnt_s/1', 'C_dmx/1');
k = {'V_pack', 1, 'k_vpack'; 'V_bus', 2, 'k_vlink'; 'V24', 4, 'k_v24'; 'V12', 5, 'k_v12'; 'V5', 6, 'k_v5'};
for j = 1:size(k, 1)                    % lines v_ch<k>: the voltage of ADC channel k
    A(n, 'simulink/Math Operations/Gain', ['K_' k{j, 1}], 'Gain', ['P.electronics.adc.' k{j, 3}]);
    L(n, sprintf('V_dmx/%d', k{j, 2}), ['K_' k{j, 1} '/1'], sprintf('v_ch%d', k{j, 2}));
end
A(n, 'simulink/Math Operations/Bias', 'I_off', 'Bias', '-P.electronics.adc.i_off');
A(n, 'simulink/Math Operations/Gain', 'I_sens', 'Gain', '1/P.electronics.adc.i_sens');
L(n, 'V_dmx/3', 'I_off/1', 'v_ch3'); L(n, 'I_off/1', 'I_sens/1', 'v_ch3_off');
A(n, 'simulink/Lookup Tables/1-D Lookup Table', 'NTC', 'BreakpointsForDimension1', 'P.electronics.adc.ntc_lut_counts', ...
    'Table', 'P.electronics.adc.ntc_lut_K', 'ExtrapMethod', 'Clip', 'OutDataTypeStr', 'single');
L(n, 'C_dmx/7', 'NTC/1', 'ntc_counts');
for j = [1 2 3 4 5 6 8]
    A(n, 'simulink/Sinks/Terminator', sprintf('T_c%d', j));
    L(n, sprintf('C_dmx/%d', j), sprintf('T_c%d/1', j), sprintf('cnt_ch%d', j));
end
A(n, 'simulink/Sinks/Terminator', 'T_v7'); L(n, 'V_dmx/7', 'T_v7/1', 'v_ch7');
A(n, 'simulink/Math Operations/Gain', 'Lift_ratio', 'Gain', '1/P.electronics.adc.vref');
A(n, 'simulink/Discontinuities/Saturation', 'Lift_sat', 'UpperLimit', '1', 'LowerLimit', '0');
L(n, 'V_dmx/8', 'Lift_ratio/1', 'v_ch8'); L(n, 'Lift_ratio/1', 'Lift_sat/1', 'lift_ratio');
A(n, 'simulink/Math Operations/Bias', 'T_mcu_K', 'Bias', '273.15');
L(n, 'mcu_temp_degC/1', 'T_mcu_K/1', 'mcu_temp_degC');
BCV(n, 'Adc_BC', {'V_pack', 'K_V_pack/1'; 'V_bus', 'K_V_bus/1'; 'I_pack', 'I_sens/1'; 'V24', 'K_V24/1'; ...
    'V12', 'K_V12/1'; 'V5', 'K_V5/1'; 'T_encl', 'NTC/1'; 'lift_pos', 'Lift_sat/1'; 'T_mcu', 'T_mcu_K/1'});
OUT(n, 'adc', 1, '');
L(n, 'Adc_BC/1', 'adc/1', 'adc');
set_param(n, 'Description', ['fw_adc_scaling (T08, Task_50, ICD 6 ADC map). in 1 adc_raw uint16[8] counts [Vpack Vbus ' ...
    'Ipack V24 V12 V5 NTC_encl LiftPot], 2 mcu_temp_degC single (Ts.comm). out 1 adc virtual bus (single): V_pack, V_bus, ' ...
    'V24, V12, V5 [V] = counts*vref/(2^nbits-1)*k_*; I_pack [A] = (v - i_off)/i_sens; T_encl [K] = 1-D Lookup ' ...
    'ntc_lut_counts -> ntc_lut_K (clip); lift_pos [0..1] = v/vref; T_mcu [K] = mcu_temp_degC + 273.15. ' ...
    'Parameters P.electronics.adc.']);
end

% ===========================================================================================
% fw_telemetry (Task_10)
% ===========================================================================================
function local_fw_telemetry(n)
CT = 'Ts.ctrl';
IN(n, 'rc', 1, '', CT); IN(n, 'mode', 2, 'uint8', CT); IN(n, 'cmd_src', 3, 'uint8', CT);
IN(n, 'fault_word', 4, 'uint32', CT); IN(n, 'warn_word', 5, 'uint32', CT);
IN(n, 'vc_tel', 6, 'single', CT, '11'); IN(n, 'drive_cmd', 7, 'Bus: Bus_DriveCmd', CT);
IN(n, 'drive_fb', 8, 'Bus: Bus_DriveFb', CT); IN(n, 'odom', 9, 'Bus: Bus_OdomMeas', CT);
IN(n, 'tilt', 10, '', CT); IN(n, 'bms', 11, 'Bus: Bus_BmsStatus', CT); IN(n, 'adc', 12, '', CT);
IN(n, 'comp_age', 13, 'single', CT); IN(n, 'overruns', 14, 'uint32', CT);
% one Bus Selector of each input bus per consumer (spec 6.2)
LOGIC(n, 'Link_ok', 'NOT', 1);
PICK(n, 'rc/1', 'Sel_rc_link', {'rc_link_lost'}, {'Link_ok/1'}, 'rc');
A(n, 'simulink/Signal Routing/Demux', 'Vc_dmx', 'Outputs', '[1 1 1 1 1 1 1 2 2]');
L(n, 'vc_tel/1', 'Vc_dmx/1', 'vc_tel');
pairs = {'W_ref_motor', 'w_ref'; 'Tau_ff', 'tau_ff'; 'Tau_lim_mot', 'tau_lim_mot'; 'Tau_lim_reg', 'tau_lim_reg'};
for j = 1:size(pairs, 1)
    A(n, 'simulink/Signal Routing/Vector Concatenate', pairs{j, 1}, 'NumInputs', '2');
    PICK(n, 'drive_cmd/1', ['Sel_cmd_' pairs{j, 2}], {['L.' pairs{j, 2}], ['R.' pairs{j, 2}]}, ...
        {[pairs{j, 1} '/1'], [pairs{j, 1} '/2']}, 'drive_cmd');
end
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Drive_state', 'NumInputs', '2');
PICK(n, 'drive_fb/1', 'Sel_fb', {'L.state', 'R.state'}, {'Drive_state/1', 'Drive_state/2'}, 'drive_fb');
A(n, 'simulink/Math Operations/Bias', 'T_encl_degC', 'Bias', '-273.15');
PICK(n, 'adc/1', 'Sel_adc', {'T_encl'}, {'T_encl_degC/1'}, 'adc');
% ---- the telemetry signals as one virtual bus tel in the order of their producers (spec 6.3), picked by Sel_tel right
%      in front of the bus-typed Telem_BC in the element order of Bus_FwTelemetry ----
rcs = {'ch_norm', 'failsafe', 'frame_lost', 'lq', 'rc_age', 'n_ok', 'n_bad', 'n_skip'};
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_rc', 'OutputSignals', strjoin(rcs, ','));
L(n, 'rc/1', 'Sel_rc/1');
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_odom', 'OutputSignals', 'vTrack_LR'); L(n, 'odom/1', 'Sel_odom/1', 'odom');
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_tilt', 'OutputSignals', 'roll,pitch,yaw_rate_meas');
L(n, 'tilt/1', 'Sel_tilt/1', 'tilt');
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_bms', 'OutputSignals', 'V_pack,I_pack,SOC,hv_ready');
L(n, 'bms/1', 'Sel_bms/1', 'bms');
vcs = {'v_target', 'w_target', 'v_ref', 'w_ref', 'a_ref', 'alpha_ref', 'w_mix', 'vtrack_ref', 'k_traction'};
tel = [{'rc_link_ok', 'Link_ok/1'}; [rcs(:), arrayfun(@(k) sprintf('Sel_rc/%d', k), (1:8)', 'UniformOutput', false)]; ...
    {'mode', 'mode/1'; 'cmd_src', 'cmd_src/1'; 'fault_word', 'fault_word/1'; 'warn_word', 'warn_word/1'}; ...
    [vcs(:), arrayfun(@(k) sprintf('Vc_dmx/%d', k), (1:9)', 'UniformOutput', false)]; ...
    {'w_ref_motor', 'W_ref_motor/1'; 'tau_ff', 'Tau_ff/1'; 'tau_lim_mot', 'Tau_lim_mot/1'; 'tau_lim_reg', 'Tau_lim_reg/1'; ...
    'drive_state', 'Drive_state/1'; 'vTrack_LR', 'Sel_odom/1'; 'roll', 'Sel_tilt/1'; 'pitch', 'Sel_tilt/2'; ...
    'yaw_rate_meas', 'Sel_tilt/3'; 'V_pack', 'Sel_bms/1'; 'I_pack', 'Sel_bms/2'; 'SOC', 'Sel_bms/3'; ...
    'hv_ready', 'Sel_bms/4'; 'T_encl_degC', 'T_encl_degC/1'; 'comp_age', 'comp_age/1'; 'overruns', 'overruns/1'}];
BCV(n, 'Tel', tel);
% Bus_FwTelemetry element <- element of tel (rc_* and sbus_* from rc, vtrack_meas = odom.vTrack_LR, power_ready =
% bms.hv_ready; every other element has its own name)
ren = {'rc_ch_norm', 'ch_norm'; 'rc_failsafe', 'failsafe'; 'rc_frame_lost', 'frame_lost'; 'rc_lq', 'lq'; ...
    'vtrack_meas', 'vTrack_LR'; 'sbus_ok', 'n_ok'; 'sbus_bad', 'n_bad'; 'sbus_skip', 'n_skip'; 'power_ready', 'hv_ready'};
B = evalin('base', 'Bus_FwTelemetry');
el = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
pk = el;
for k = 1:numel(el)
    r = find(strcmp(ren(:, 1), el{k}), 1);
    if ~isempty(r), pk{k} = ren{r, 2}; end
end
assert(numel(pk) == size(tel, 1) && isempty(setxor(pk, tel(:, 1))), 'build_fw_platform:telem', ...
    'fw_telemetry: tel does not cover the elements of Bus_FwTelemetry');
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_tel', 'OutputSignals', strjoin(pk, ','));
L(n, 'Tel/1', 'Sel_tel/1', 'tel');
A(n, 'simulink/Signal Routing/Bus Creator', 'Telem_BC', 'Inputs', num2str(numel(el)), ...
    'OutDataTypeStr', 'Bus: Bus_FwTelemetry', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(el), L(n, sprintf('Sel_tel/%d', k), sprintf('Telem_BC/%d', k)); end
OUT(n, 'telem', 1, 'Bus: Bus_FwTelemetry');
L(n, 'Telem_BC/1', 'telem/1', 'telem');
set_param(n, 'Description', ['fw_telemetry (T08, Task_10). in 1 rc (virtual bus of fw_rc_mapping), 2 mode uint8, ' ...
    '3 cmd_src uint8, 4 fault_word uint32, 5 warn_word uint32, 6 vc_tel single[11] (fw_vehicle_ctrl: [v_target ' ...
    'w_target v_ref w_ref a_ref alpha_ref w_mix vtrack_ref_L vtrack_ref_R k_tr_L k_tr_R]), 7 drive_cmd Bus_DriveCmd, ' ...
    '8 drive_fb Bus_DriveFb, 9 odom Bus_OdomMeas, 10 tilt (virtual bus: roll pitch tilt_gate yaw_rate_meas gyro_bias ' ...
    'imu_valid), 11 bms Bus_BmsStatus, 12 adc (virtual bus of fw_adc_scaling), 13 comp_age single, 14 overruns uint32 ' ...
    '(all Ts.ctrl). out 1 telem Bus_FwTelemetry (rc_link_ok = NOT rc_link_lost, T_encl_degC = adc.T_encl - 273.15, ' ...
    'power_ready = bms.hv_ready, drive_state = ESC states [L R]).']);
end

% ===========================================================================================
% IO_In / IO_Out variant choices
% ===========================================================================================
function local_io_in_ports(n)
IN(n, 'in1k', 1, 'Bus: Bus_McuIn1k', 'Ts.base'); IN(n, 'in5', 2, 'Bus: Bus_McuIn5', 'Ts.imu');
IN(n, 'in10', 3, 'Bus: Bus_McuIn10', 'Ts.ctrl'); IN(n, 'in50', 4, 'Bus: Bus_McuIn50', 'Ts.comm');
IN(n, 'in100', 5, 'Bus: Bus_McuIn100', 'Ts.hk');
outs = {'sbus', 'Bus: Bus_SbusStream'; 'di_word', 'uint16'; 't_us', 'uint32'; 'reset_flag', 'boolean'; ...
    'fault_inject', 'uint32'; 'imu', 'Bus: Bus_ImuRaw'; 'can1_esc', 'Bus: Bus_Can1_EscFast'; 'busoff', 'boolean'; ...
    'can1_escslow', 'Bus: Bus_Can1_EscSlow'; 'can1_impl', 'Bus: Bus_Can1_Impl'; 'can2_comp', 'Bus: Bus_CanFrame64'; ...
    'adc_raw', 'uint16'; 'mcu_temp_degC', 'single'; 'can1_bms', 'Bus: Bus_Can1_Bms'};
for k = 1:size(outs, 1), OUT(n, outs{k, 1}, k, outs{k, 2}); end
end

function local_io_desc(n, kind)
set_param(n, 'Description', [n ' (T08, Shaddad_MCU IO_In choice ' kind '). in 1 in1k Bus_McuIn1k 0.001, 2 in5 Bus_McuIn5 ' ...
    '0.005, 3 in10 Bus_McuIn10 0.01, 4 in50 Bus_McuIn50 0.05, 5 in100 Bus_McuIn100 0.1. out 1 sbus Bus_SbusStream, ' ...
    '2 di_word uint16, 3 t_us uint32, 4 reset_flag boolean, 5 fault_inject uint32 (0.001); 6 imu Bus_ImuRaw (0.005); ' ...
    '7 can1_esc Bus_Can1_EscFast, 8 busoff boolean[2] [CAN1 CAN2] (0.01); 9 can1_escslow Bus_Can1_EscSlow, 10 can1_impl ' ...
    'Bus_Can1_Impl, 11 can2_comp Bus_CanFrame64, 12 adc_raw uint16[8], 13 mcu_temp_degC single (0.05); 14 can1_bms ' ...
    'Bus_Can1_Bms (0.1).']);
end

function local_fw_io_in_sim(n)
local_io_in_ports(n);
% one Bus Selector of each rate bus per consumer (spec 6.2)
PICK(n, 'in1k/1', 'Sel_1k_sbus', {'sbus'}, {'sbus/1'}, 'in1k');
DTC(n, 'DTC_di', 'uint16'); PICK(n, 'in1k/1', 'Sel_1k_di_word', {'di_word'}, {'DTC_di/1'});
L(n, 'DTC_di/1', 'di_word/1', 'di_word');
DTC(n, 'DTC_t_us', 'uint32'); PICK(n, 'in1k/1', 'Sel_1k_t_us', {'t_us'}, {'DTC_t_us/1'});
L(n, 'DTC_t_us/1', 't_us/1', 't_us');
DTC(n, 'DTC_reset', 'boolean'); PICK(n, 'in1k/1', 'Sel_1k_reset_flag', {'reset_flag'}, {'DTC_reset/1'});
L(n, 'DTC_reset/1', 'reset_flag/1', 'reset_flag');
DTC(n, 'DTC_fi', 'uint32'); PICK(n, 'in1k/1', 'Sel_1k_fault_inject', {'fault_inject'}, {'DTC_fi/1'});
L(n, 'DTC_fi/1', 'fault_inject/1', 'fault_inject');
PICK(n, 'in5/1', 'Sel_5', {'imu'}, {'imu/1'}, 'in5');
PICK(n, 'in10/1', 'Sel_10', {'can1_esc'}, {'can1_esc/1'}, 'in10');
% SIM bus-off: fault_inject b4 (CAN1) / b5 (CAN2) -> busoff[2], RT 0.001 -> 0.01 (D60)
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'BO1_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '16', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'BO2_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '32', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'BO1', 'relop', '~=', 'OutDataTypeStr', 'boolean');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'BO2', 'relop', '~=', 'OutDataTypeStr', 'boolean');
L(n, 'DTC_fi/1', 'BO1_mask/1'); L(n, 'DTC_fi/1', 'BO2_mask/1');
L(n, 'BO1_mask/1', 'BO1/1', 'bo1_bit'); L(n, 'BO2_mask/1', 'BO2/1', 'bo2_bit');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'BO_vec', 'NumInputs', '2');
L(n, 'BO1/1', 'BO_vec/1', 'can1_busoff'); L(n, 'BO2/1', 'BO_vec/2', 'can2_busoff');
RT(n, 'RT_busoff', 'Ts.ctrl', 'false');
L(n, 'BO_vec/1', 'RT_busoff/1', 'busoff_1k'); L(n, 'RT_busoff/1', 'busoff/1', 'busoff');
PICK(n, 'in50/1', 'Sel_50_can1_escslow', {'can1_escslow'}, {'can1_escslow/1'}, 'in50');
PICK(n, 'in50/1', 'Sel_50_can1_impl', {'can1_impl'}, {'can1_impl/1'});
PICK(n, 'in50/1', 'Sel_50_can2_comp', {'can2_comp'}, {'can2_comp/1'});
DTC(n, 'DTC_adc', 'uint16'); PICK(n, 'in50/1', 'Sel_50_adc_raw', {'adc_raw'}, {'DTC_adc/1'});
L(n, 'DTC_adc/1', 'adc_raw/1', 'adc_raw');
DTC(n, 'DTC_tmcu', 'single'); PICK(n, 'in50/1', 'Sel_50_mcu_temp_degC', {'mcu_temp_degC'}, {'DTC_tmcu/1'});
L(n, 'DTC_tmcu/1', 'mcu_temp_degC/1', 'mcu_temp_degC');
PICK(n, 'in100/1', 'Sel_100', {'can1_bms'}, {'can1_bms/1'}, 'in100');
local_io_desc(n, 'SIM (VC_TARGET == 0): pass-through + Data Type Conversion; fault_inject b4/b5 -> busoff (RT to 0.01)');
end

function local_fw_io_in_hw(n)
local_io_in_ports(n);
for p = {'in1k', 'in5', 'in10', 'in50', 'in100'}
    A(n, 'simulink/Sinks/Terminator', ['T_' p{1}]); L(n, [p{1} '/1'], ['T_' p{1} '/1']);
end
H = 'stm32h7xxblockslib/';
% ---- SBUS UART (USART2, RXINV in CubeMX) ----
A(n, [H 'UART//USART Read'], 'SBUS_UART', 'UARTModule', 'USART2', 'DataType', 'uint8', 'DataLength', '32', ...
    'BlockingMode', 'off', 'OutputLength', 'on', 'OutputStatus', 'on', 'SampleTime', 'Ts.base');
DTC(n, 'Uart_len', 'uint8');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Uart_err', 'relop', '~=', 'OutDataTypeStr', 'boolean');
L(n, 'SBUS_UART/2', 'Uart_len/1'); L(n, 'SBUS_UART/3', 'Uart_err/1');
A(n, 'simulink/Signal Routing/Bus Creator', 'Sbus_BC', 'Inputs', '3', 'OutDataTypeStr', 'Bus: Bus_SbusStream', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'SBUS_UART/1', 'Sbus_BC/1', 'bytes'); L(n, 'Uart_len/1', 'Sbus_BC/2', 'len'); L(n, 'Uart_err/1', 'Sbus_BC/3', 'uart_err');
L(n, 'Sbus_BC/1', 'sbus/1', 'sbus');
% ---- DI GPIOF[0:13] -> di_word ----
A(n, [H 'Digital Port Read'], 'DI_GPIOF', 'PortName', 'GPIOF', 'PinNumber', '[0:13]', 'IOAsArray', 'on', 'SampleTime', 'Ts.base');
DTC(n, 'DI_bool', 'boolean');
CST(n, 'DI_pad', 'false(1,16-P.electronics.hw.di_pins)', 'boolean', 'Ts.base');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'DI_bits', 'NumInputs', '2');
A(n, 'commutil2/Bit to Integer Converter', 'DI_word', 'nbits', '16', 'bitOrder', 'LSB first', 'outDtype', 'uint16');
L(n, 'DI_GPIOF/1', 'DI_bool/1'); L(n, 'DI_bool/1', 'DI_bits/1'); L(n, 'DI_pad/1', 'DI_bits/2');
L(n, 'DI_bits/1', 'DI_word/1'); L(n, 'DI_word/1', 'di_word/1', 'di_word');
% ---- t_us = TIM2 CNT (1 MHz, D60) ----
REG(n, 'TIM2_CNT', 'TIM2', 'CNT', 'Read', 'Ts.base');
DTC(n, 'T_us_u32', 'uint32'); L(n, 'TIM2_CNT/1', 'T_us_u32/1'); L(n, 'T_us_u32/1', 't_us/1', 't_us');
% ---- reset_flag = RCC_RSR.IWDG1RSTF read at boot; RMVF written on the second tick ----
REG(n, 'RCC_RSR', 'RCC', 'RSR', 'Read', 'Ts.base');
DTC(n, 'RSR_u32', 'uint32');
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'IWDG1RSTF', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^P.electronics.hw.rcc_rsr_iwdg1rstf_bit', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Rst_seen', 'relop', '~=', 'OutDataTypeStr', 'boolean');
CST(n, 'Boot_false', 'false', 'boolean', 'Ts.base');
A(n, 'simulink/Discrete/Unit Delay', 'First_tick', 'InitialCondition', 'true', 'SampleTime', 'Ts.base');
A(n, 'simulink/Discrete/Unit Delay', 'Second_tick', 'InitialCondition', 'false', 'SampleTime', 'Ts.base');
LOGIC(n, 'Reset_at_boot', 'AND', 2);
L(n, 'RCC_RSR/1', 'RSR_u32/1'); L(n, 'RSR_u32/1', 'IWDG1RSTF/1'); L(n, 'IWDG1RSTF/1', 'Rst_seen/1');
L(n, 'Boot_false/1', 'First_tick/1'); L(n, 'First_tick/1', 'Second_tick/1', 'first_tick');
L(n, 'Rst_seen/1', 'Reset_at_boot/1'); L(n, 'First_tick/1', 'Reset_at_boot/2');
L(n, 'Reset_at_boot/1', 'reset_flag/1', 'reset_flag');
s = ENSUB(n, 'RMVF_write', {}, {});
CST(s, 'RMVF', '2^P.electronics.hw.rcc_rsr_rmvf_bit', 'uint32', '-1');
REG(s, 'RCC_RSR_RMVF', 'RCC', 'RSR', 'Write', '-1', 'RMVF/1');
ENLINE(n, 'Second_tick/1', 'RMVF_write', 'second_tick');
CST(n, 'No_inject', '0', 'uint32', 'Ts.base');
L(n, 'No_inject/1', 'fault_inject/1', 'fault_inject');
% ---- IMU ICM-20948 (I2C1 0x69, 400 kHz, +/-4 g, +/-500 dps, single, 5 ms) ----
A(n, 'stm32IMUsensorslib/ICM20948 IMU Sensor', 'ICM20948', 'I2CModule', 'I2C1', 'I2CAddress', '0x69', ...
    'BitRate', '400000', 'AccelerometerRange', '+/-4g', 'GyroscopeRange', '+/-500 dps', 'DataType', 'single', ...
    'SampleTime', 'Ts.imu');
el = {'Acceleration', 'AngularRate', 'MagneticField', 'Temperature'};
A(n, 'simulink/Signal Routing/Bus Creator', 'Imu_BC', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_ImuRaw', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:4
    A(n, 'simulink/Math Operations/Reshape', ['Imu_vec' num2str(k)], 'OutputDimensionality', '1-D array');
    L(n, sprintf('ICM20948/%d', k), sprintf('Imu_vec%d/1', k));
    L(n, sprintf('Imu_vec%d/1', k), sprintf('Imu_BC/%d', k), el{k});
end
L(n, 'Imu_BC/1', 'imu/1', 'imu');
% ---- FDCAN1 receive: For Iterator (FIFO 0) + ID dispatch to held frame slots ----
slots = {'fb1_L', 'P.drive.can.id_fb1+1', 'Ts.ctrl'; 'fb1_R', 'P.drive.can.id_fb1+2', 'Ts.ctrl'; ...
    'fb3_L', 'P.drive.can.id_fb3+1', 'Ts.ctrl'; 'fb3_R', 'P.drive.can.id_fb3+2', 'Ts.ctrl'; ...
    'fb2_L', 'P.drive.can.id_fb2+1', 'Ts.comm'; 'fb2_R', 'P.drive.can.id_fb2+2', 'Ts.comm'; ...
    'impl_fb', 'P.electronics.can.id.impl_fb', 'Ts.comm'; 'limits', 'P.electronics.can.id.bms_limits', 'Ts.hk'; ...
    'soc', 'P.electronics.can.id.bms_soc', 'Ts.hk'; 'vit', 'P.electronics.can.id.bms_vit', 'Ts.hk'; ...
    'alarm', 'P.electronics.can.id.bms_alarm', 'Ts.hk'; 'cells', 'P.electronics.can.id.bms_cells', 'Ts.hk'; ...
    'state', 'P.electronics.can.id.bms_state', 'Ts.hk'; 'power', 'P.electronics.can.id.bms_power', 'Ts.hk'};
local_can_rx(n, 'CAN1_RX', 'FDCAN1', 'CAN', 8, 'P.electronics.hw.can1_rx_iter', 'Ts.ctrl', slots, 'Bus_CanFrame8');
A(n, 'simulink/Signal Routing/Bus Creator', 'Esc_fast_BC', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_Can1_EscFast', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'Esc_slow_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_EscSlow', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'Impl_BC', 'Inputs', '1', 'OutDataTypeStr', 'Bus: Bus_Can1_Impl', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'Bms_BC', 'Inputs', '7', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
dst = {'Esc_fast_BC/1', 'Esc_fast_BC/2', 'Esc_fast_BC/3', 'Esc_fast_BC/4', 'Esc_slow_BC/1', 'Esc_slow_BC/2', ...
    'Impl_BC/1', 'Bms_BC/1', 'Bms_BC/2', 'Bms_BC/3', 'Bms_BC/4', 'Bms_BC/5', 'Bms_BC/6', 'Bms_BC/7'};
for k = 1:size(slots, 1), L(n, ['Frame_' slots{k, 1} '/1'], dst{k}, slots{k, 1}); end
L(n, 'Esc_fast_BC/1', 'can1_esc/1', 'can1_esc'); L(n, 'Esc_slow_BC/1', 'can1_escslow/1', 'can1_escslow');
L(n, 'Impl_BC/1', 'can1_impl/1', 'can1_impl'); L(n, 'Bms_BC/1', 'can1_bms/1', 'can1_bms');
% ---- FDCAN2 receive (CAN-FD): COMP_CMD slot ----
local_can_rx(n, 'CAN2_RX', 'FDCAN2', 'CAN-FD', 64, 'P.electronics.hw.can2_rx_iter', 'Ts.comm', ...
    {'comp_cmd', 'P.electronics.can.id.comp_cmd', 'Ts.comm'}, 'Bus_CanFrame64');
L(n, 'Frame_comp_cmd/1', 'can2_comp/1', 'can2_comp');
% ---- bus-off: FDCAN1/2 PSR.BO (Task_10, D60) ----
REG(n, 'FDCAN1_PSR', 'FDCAN1', 'PSR', 'Read', 'Ts.ctrl');
REG(n, 'FDCAN2_PSR', 'FDCAN2', 'PSR', 'Read', 'Ts.ctrl');
for k = 1:2
    DTC(n, sprintf('PSR%d_u32', k), 'uint32');
    A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', sprintf('BO%d_bit', k), 'logicop', 'AND', 'UseBitMask', 'on', ...
        'BitMask', '2^P.electronics.hw.fdcan_psr_bo_bit', 'BitMaskRealWorld', 'Stored Integer');
    A(n, 'simulink/Logic and Bit Operations/Compare To Zero', sprintf('BO%d', k), 'relop', '~=', 'OutDataTypeStr', 'boolean');
    L(n, sprintf('FDCAN%d_PSR/1', k), sprintf('PSR%d_u32/1', k)); L(n, sprintf('PSR%d_u32/1', k), sprintf('BO%d_bit/1', k));
    L(n, sprintf('BO%d_bit/1', k), sprintf('BO%d/1', k));
end
A(n, 'simulink/Signal Routing/Vector Concatenate', 'BO_vec', 'NumInputs', '2');
L(n, 'BO1/1', 'BO_vec/1', 'can1_busoff'); L(n, 'BO2/1', 'BO_vec/2', 'can2_busoff');
L(n, 'BO_vec/1', 'busoff/1', 'busoff');
% ---- ADC1 (8 regular conversions, 50 ms) and ADC3 temperature sensor ----
A(n, [H 'Analog to Digital Converter'], 'ADC1_Conv', 'ADCModule', 'ADC1', 'ConversionGroup', 'Regular', ...
    'TriggerMode', 'Trigger and read', 'NumberOfConversions', '8', 'SampleTime', 'Ts.comm');
DTC(n, 'ADC1_u16', 'uint16'); L(n, 'ADC1_Conv/1', 'ADC1_u16/1'); L(n, 'ADC1_u16/1', 'adc_raw/1', 'adc_raw');
A(n, [H 'Analog to Digital Converter'], 'ADC3_TempSensor', 'ADCModule', 'ADC3', 'ConversionGroup', 'Regular', ...
    'TriggerMode', 'Trigger and read', 'NumberOfConversions', '1', 'SampleTime', 'Ts.comm');
DTC(n, 'ADC3_s', 'single');
A(n, 'simulink/Math Operations/Bias', 'TS_c1', 'Bias', '-P.electronics.adc.ts_cal_cnt(1)');
A(n, 'simulink/Math Operations/Gain', 'TS_slope', 'Gain', ...
    'diff(P.electronics.adc.ts_cal_T)/diff(P.electronics.adc.ts_cal_cnt)');
A(n, 'simulink/Math Operations/Bias', 'TS_T1', 'Bias', 'P.electronics.adc.ts_cal_T(1)');
L(n, 'ADC3_TempSensor/1', 'ADC3_s/1'); L(n, 'ADC3_s/1', 'TS_c1/1'); L(n, 'TS_c1/1', 'TS_slope/1'); L(n, 'TS_slope/1', 'TS_T1/1');
L(n, 'TS_T1/1', 'mcu_temp_degC/1', 'mcu_temp_degC');
local_io_desc(n, ['HW (VC_TARGET == 1): stm32h7xxblockslib UART//USART Read USART2, Digital Port Read GPIOF[0:13], ' ...
    'stm32IMUsensorslib ICM20948 (I2C1 0x69), FDCAN Read FDCAN1 / FDCAN2 in For Iterator Subsystems with ID dispatch ' ...
    'to held frame slots (valid = slot counter changed at the slot rate), ADC1 8 conv + ADC3 temperature sensor, ' ...
    'stmutilitieslib Register Read//Write: TIM2 CNT -> t_us, RCC_RSR IWDG1RSTF -> reset_flag (boot, then RMVF), ' ...
    'FDCAN1/2 PSR.BO -> busoff (D60). Pin maps / bit rates are set in the CubeMX project (T17); FDCAN Read outputs ' ...
    '1 Data [64x1] (-> 1-D, bytes 1:8 for the CAN1 slots), 2 Length, 3 Id, 4 Status (stm32cube.blocks.FDCANBlock)']);
end

function local_can_rx(n, name, module, ftype, dlen, iterExpr, ts, slots, busName)
% For Iterator Subsystem reading one FDCAN FIFO; each slot (Compare To Constant on the ID) is an Enabled
% Subsystem that holds data / length and counts receptions. Outside: per slot RT to its rate, valid =
% Detect Change(count), Bus Creator <busName> -> block Frame_<slot>.
fi = [n '/' name];
add_block('simulink/Ports & Subsystems/For Iterator Subsystem', fi);
delete_line(fi, 'In1/1', 'Out1/1'); delete_block([fi '/In1']); delete_block([fi '/Out1']);
set_param(fi, 'SystemSampleTime', ts);
set_param([fi '/For Iterator'], 'IterationSource', 'internal', 'IterationLimit', iterExpr, 'ShowIterationPort', 'off');
A(fi, 'stm32h7xxblockslib/FDCAN Read', 'FDCAN_Read', 'FDCANModule', module, 'BlockMode', 'Receive', ...
    'OperationReadMode', 'Data', 'CANFrameType', ftype, 'ReadSource', 'FIFO 0', 'OutputTypeReadMode', 'Unpacked', ...
    'OutputStatus', 'on', 'SampleTime', '-1');
DTC(fi, 'Rx_data', 'uint8'); DTC(fi, 'Rx_id', 'uint32'); DTC(fi, 'Rx_len', 'uint8');
A(fi, 'simulink/Logic and Bit Operations/Compare To Zero', 'Rx_new', 'relop', '==', 'OutDataTypeStr', 'boolean');
% FDCAN Read (Receive, Unpacked, OutputStatus on) output order of stm32cube.blocks.FDCANBlock (getOutputNames):
% 1 Data uint8 [64x1] (CAN-FD maximum for every frame type), 2 Length uint8, 3 Id uint32, 4 Status.
% Data -> 1-D; a classic CAN frame slot keeps bytes 1:dlen (Bus_CanFrame8.data has 8 elements).
A(fi, 'simulink/Math Operations/Reshape', 'Rx_vec', 'OutputDimensionality', '1-D array');
L(fi, 'FDCAN_Read/1', 'Rx_vec/1');
if dlen < 64
    A(fi, 'simulink/Signal Routing/Selector', 'Rx_bytes', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', sprintf('1:%d', dlen), 'InputPortWidth', '-1');
    L(fi, 'Rx_vec/1', 'Rx_bytes/1'); L(fi, 'Rx_bytes/1', 'Rx_data/1');
else
    L(fi, 'Rx_vec/1', 'Rx_data/1');
end
L(fi, 'FDCAN_Read/2', 'Rx_len/1'); L(fi, 'FDCAN_Read/3', 'Rx_id/1');
L(fi, 'FDCAN_Read/4', 'Rx_new/1');
for k = 1:size(slots, 1)
    sl = slots{k, 1};
    CMP(fi, ['Id_' sl], '==', slots{k, 2});
    LOGIC(fi, ['Hit_' sl], 'AND', 2);
    L(fi, 'Rx_id/1', ['Id_' sl '/1']); L(fi, ['Id_' sl '/1'], ['Hit_' sl '/1']); L(fi, 'Rx_new/1', ['Hit_' sl '/2']);
    s = ENSUB(fi, ['Slot_' sl], {'data', 'len'}, {'data_h', sprintf('zeros(1,%d)', dlen); 'len_h', '0'; 'cnt', '0'});
    L(s, 'data/1', 'data_h/1', 'data'); L(s, 'len/1', 'len_h/1', 'len');
    A(s, 'simulink/Discrete/Unit Delay', 'Cnt_prev', 'InitialCondition', '0', 'SampleTime', '-1');
    CST(s, 'One', '1', 'uint8', '-1');
    A(s, 'simulink/Math Operations/Sum', 'Cnt_inc', 'Inputs', '++', 'OutDataTypeStr', 'uint8', 'SaturateOnIntegerOverflow', 'off');
    L(s, 'Cnt_prev/1', 'Cnt_inc/1', 'cnt_prev'); L(s, 'One/1', 'Cnt_inc/2', 'one'); L(s, 'Cnt_inc/1', 'Cnt_prev/1', 'cnt');
    L(s, 'Cnt_inc/1', 'cnt/1', 'cnt');
    L(fi, 'Rx_data/1', ['Slot_' sl '/1']); L(fi, 'Rx_len/1', ['Slot_' sl '/2']);
    ENLINE(fi, ['Hit_' sl '/1'], ['Slot_' sl], ['hit_' sl]);
    for j = 1:3
        nm = {'data', 'len', 'cnt'};
        OUT(fi, [nm{j} '_' sl], 3*(k-1) + j, '');
        L(fi, sprintf('Slot_%s/%d', sl, j), [nm{j} '_' sl '/1'], [nm{j} '_' sl]);
    end
end
for k = 1:size(slots, 1)
    sl = slots{k, 1}; tsk = slots{k, 3};
    src = cell(1, 3);
    for j = 1:3
        if strcmp(tsk, ts)
            src{j} = sprintf('%s/%d', name, 3*(k-1) + j);
        else
            rt = sprintf('RT_%s_%d', sl, j);
            RT(n, rt, tsk, '0');
            L(n, sprintf('%s/%d', name, 3*(k-1) + j), [rt '/1']);
            src{j} = [rt '/1'];
        end
    end
    A(n, 'simulink/Logic and Bit Operations/Detect Change', ['New_' sl], 'vinit', '0');
    L(n, src{3}, ['New_' sl '/1'], ['cnt_' sl]);
    CST(n, ['Id_' sl], slots{k, 2}, 'uint32', tsk);
    A(n, 'simulink/Signal Routing/Bus Creator', ['Frame_' sl], 'Inputs', '4', 'OutDataTypeStr', ['Bus: ' busName], ...
        'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
    L(n, ['Id_' sl '/1'], ['Frame_' sl '/1'], 'id'); L(n, src{2}, ['Frame_' sl '/2'], 'len');
    L(n, src{1}, ['Frame_' sl '/3'], 'data'); L(n, ['New_' sl '/1'], ['Frame_' sl '/4'], 'valid');
end
end

function local_io_out_ports(n)
ins = {'do_word', 'uint32', 'Ts.base', ''; 'wdg_kick', 'boolean', 'Ts.base', ''; ...
    'can2_imu', 'Bus: Bus_CanFrame64', 'Ts.imu', ''; 'can1_drv', 'Bus: Bus_Can1_McuFast', 'Ts.ctrl', ''; ...
    'can2_odom', 'Bus: Bus_CanFrame64', 'Ts.ctrl', ''; 'pwm_duty', 'single', 'Ts.ctrl', '4'; ...
    'can1_slow', 'Bus: Bus_Can1_McuSlow', 'Ts.comm', ''; 'can2_status', 'Bus: Bus_CanFrame64', 'Ts.comm', ''; ...
    'can2_power', 'Bus: Bus_CanFrame64', 'Ts.hk', ''};
for k = 1:size(ins, 1), IN(n, ins{k, 1}, k, ins{k, 2}, ins{k, 3}, ins{k, 4}); end
outs = {'out1k', 'McuOut1k'; 'out5', 'McuOut5'; 'out10', 'McuOut10'; 'out50', 'McuOut50'; 'out100', 'McuOut100'};
for k = 1:size(outs, 1), OUT(n, outs{k, 1}, k, ['Bus: Bus_' outs{k, 2}]); end
% root buses (identical in both choices)
A(n, 'simulink/Signal Routing/Bus Creator', 'BC_out1k', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_McuOut1k', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'BC_out5', 'Inputs', '1', 'OutDataTypeStr', 'Bus: Bus_McuOut5', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'BC_out10', 'Inputs', '3', 'OutDataTypeStr', 'Bus: Bus_McuOut10', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'BC_out50', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_McuOut50', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(n, 'simulink/Signal Routing/Bus Creator', 'BC_out100', 'Inputs', '1', 'OutDataTypeStr', 'Bus: Bus_McuOut100', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'do_word/1', 'BC_out1k/1', 'do_word'); L(n, 'wdg_kick/1', 'BC_out1k/2', 'wdg_kick');
L(n, 'can2_imu/1', 'BC_out5/1', 'can2_imu');
L(n, 'can1_drv/1', 'BC_out10/1', 'can1_drv'); L(n, 'can2_odom/1', 'BC_out10/2', 'can2_odom'); L(n, 'pwm_duty/1', 'BC_out10/3', 'pwm_duty');
L(n, 'can1_slow/1', 'BC_out50/1', 'can1_slow'); L(n, 'can2_status/1', 'BC_out50/2', 'can2_status');
L(n, 'can2_power/1', 'BC_out100/1', 'can2_power');
for k = 1:size(outs, 1), L(n, ['BC_' outs{k, 1} '/1'], [outs{k, 1} '/1'], outs{k, 1}); end
end

function local_io_out_desc(n, kind)
set_param(n, 'Description', [n ' (T08, Shaddad_MCU IO_Out choice ' kind '). in 1 do_word uint32, 2 wdg_kick boolean ' ...
    '(0.001); 3 can2_imu Bus_CanFrame64 (0.005); 4 can1_drv Bus_Can1_McuFast, 5 can2_odom Bus_CanFrame64, 6 pwm_duty ' ...
    'single[4] (0.01); 7 can1_slow Bus_Can1_McuSlow, 8 can2_status Bus_CanFrame64 (0.05); 9 can2_power Bus_CanFrame64 ' ...
    '(0.1). out 1 out1k Bus_McuOut1k, 2 out5 Bus_McuOut5, 3 out10 Bus_McuOut10, 4 out50 Bus_McuOut50, 5 out100 ' ...
    'Bus_McuOut100 (Bus Creators, identical in both choices).']);
end

function local_fw_io_out_sim(n)
local_io_out_ports(n);
local_io_out_desc(n, 'SIM (VC_TARGET == 0): Bus Creators only');
end

function local_fw_io_out_hw(n)
local_io_out_ports(n);
H = 'stm32h7xxblockslib/';
% ---- DO word -> GPIOG[0:15] + GPIOE[0:3] ----
A(n, 'commutil2/Integer to Bit Converter', 'DO_bits', 'nbits', '32', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
L(n, 'do_word/1', 'DO_bits/1');
A(n, 'simulink/Signal Routing/Selector', 'DO_G', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1:P.electronics.hw.do_g_pins', 'InputPortWidth', '32');
A(n, 'simulink/Signal Routing/Selector', 'DO_E', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', ...
    'P.electronics.hw.do_g_pins+(1:P.electronics.hw.do_e_pins)', 'InputPortWidth', '32');
L(n, 'DO_bits/1', 'DO_G/1', 'do_bits'); L(n, 'DO_bits/1', 'DO_E/1');
A(n, [H 'Digital Port Write'], 'DO_GPIOG', 'PortName', 'GPIOG', 'PinNumber', '[0:15]', 'IOAsArray', 'on');
A(n, [H 'Digital Port Write'], 'DO_GPIOE', 'PortName', 'GPIOE', 'PinNumber', '[0:3]', 'IOAsArray', 'on');
L(n, 'DO_G/1', 'DO_GPIOG/1'); L(n, 'DO_E/1', 'DO_GPIOE/1');
% ---- IWDG1 refresh: KR = 0xAAAA on wdg_kick ----
s = ENSUB(n, 'IWDG1_refresh', {}, {});
CST(s, 'KR_refresh', 'P.electronics.wdg.kr_refresh', 'uint32', '-1');
REG(s, 'IWDG1_KR', 'IWDG1', 'KR', 'Write', '-1', 'KR_refresh/1');
ENLINE(n, 'wdg_kick/1', 'IWDG1_refresh', 'wdg_kick');
% ---- PWM: TIM1 CH1/CH2 (pump, lift; 10 ms), TIM4 CH1/CH2 (fans; RT -> 0.1 s) in percent ----
A(n, 'simulink/Math Operations/Gain', 'Duty_pct', 'Gain', '100');
A(n, 'simulink/Signal Routing/Demux', 'Duty_dmx', 'Outputs', '4');
L(n, 'pwm_duty/1', 'Duty_pct/1'); L(n, 'Duty_pct/1', 'Duty_dmx/1', 'duty_pct');
A(n, [H 'PWM Output'], 'PWM_TIM1', 'TimerModule', 'TIM1', 'DutycycleUnits', 'Percentage', 'EnableChannel1', 'on', ...
    'EnableChannel2', 'on', 'SampleTime', 'Ts.ctrl');
L(n, 'Duty_dmx/1', 'PWM_TIM1/1', 'pump_pct'); L(n, 'Duty_dmx/2', 'PWM_TIM1/2', 'lift_pct');
RT(n, 'RT_fan1', 'Ts.hk', '0'); RT(n, 'RT_fan2', 'Ts.hk', '0');
L(n, 'Duty_dmx/3', 'RT_fan1/1', 'fan1_pct'); L(n, 'Duty_dmx/4', 'RT_fan2/1', 'fan2_pct');
A(n, [H 'PWM Output'], 'PWM_TIM4', 'TimerModule', 'TIM4', 'DutycycleUnits', 'Percentage', 'EnableChannel1', 'on', ...
    'EnableChannel2', 'on', 'SampleTime', 'Ts.hk');
L(n, 'RT_fan1/1', 'PWM_TIM4/1'); L(n, 'RT_fan2/1', 'PWM_TIM4/2');
% ---- FDCAN Write per frame (Raw data, in Enabled Subsystems on valid) ----
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_drv', 'OutputSignals', 'drv_cmd_L,drv_cmd_R');
L(n, 'can1_drv/1', 'Sel_drv/1');
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_slow', 'OutputSignals', 'impl_cmd,bms_req');
L(n, 'can1_slow/1', 'Sel_slow/1');
tx = {'DRV_CMD_L', 'Sel_drv/1', 'FDCAN1', 'CAN', 'P.drive.can.id_cmd+1', 8, 'off'; ...
      'DRV_CMD_R', 'Sel_drv/2', 'FDCAN1', 'CAN', 'P.drive.can.id_cmd+2', 8, 'off'; ...
      'IMPL_CMD', 'Sel_slow/1', 'FDCAN1', 'CAN', 'P.electronics.can.id.impl_cmd', 8, 'off'; ...
      'BMS_REQ', 'Sel_slow/2', 'FDCAN1', 'CAN', 'P.electronics.can.id.bms_req', 8, 'off'; ...
      'MCU_IMU', 'can2_imu/1', 'FDCAN2', 'CAN-FD', 'P.electronics.can.id.mcu_imu', 48, 'on'; ...
      'MCU_ODOM', 'can2_odom/1', 'FDCAN2', 'CAN-FD', 'P.electronics.can.id.mcu_odom', 32, 'on'; ...
      'MCU_STATUS', 'can2_status/1', 'FDCAN2', 'CAN-FD', 'P.electronics.can.id.mcu_status', 48, 'on'; ...
      'MCU_POWER', 'can2_power/1', 'FDCAN2', 'CAN-FD', 'P.electronics.can.id.mcu_power', 64, 'on'};
for k = 1:size(tx, 1)
    nm = tx{k, 1};
    A(n, 'simulink/Signal Routing/Bus Selector', ['Sel_' nm], 'OutputSignals', 'data,valid');
    L(n, tx{k, 2}, ['Sel_' nm '/1']);
    A(n, 'simulink/Signal Routing/Selector', ['Bytes_' nm], 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', sprintf('1:%d', tx{k, 6}), ...
        'InputPortWidth', num2str(max(8, 64*strcmp(tx{k, 4}, 'CAN-FD'))));
    L(n, ['Sel_' nm '/1'], ['Bytes_' nm '/1']);
    s = ENSUB(n, ['TX_' nm], {'data'}, {});
    A(s, [H 'FDCAN Write'], 'FDCAN_Write', 'FDCANModule', tx{k, 3}, 'BlockMode', 'Transmit', ...
        'DataFormatWriteMode', 'Raw data', 'CANFrameType', tx{k, 4}, 'IdentifierTypeVia', 'Dialog', ...
        'IdentifierType', 'Standard (11-bit identifier)', 'IdentifierVia', 'Dialog', 'Identifier', tx{k, 5}, ...
        'DataLengthVia', 'Dialog', 'DataLength', num2str(tx{k, 6}), 'IsBitRateSwitchingEnabled', tx{k, 7});
    L(s, 'data/1', 'FDCAN_Write/1', 'data');
    L(n, ['Bytes_' nm '/1'], ['TX_' nm '/1'], ['bytes_' nm]);
    ENLINE(n, ['Sel_' nm '/2'], ['TX_' nm], '');
end
local_io_out_desc(n, ['HW (VC_TARGET == 1): Integer to Bit (32) -> Digital Port Write GPIOG[0:15] / GPIOE[0:3]; ' ...
    'PWM Output TIM1 CH1/CH2 (0.01) and TIM4 CH1/CH2 (0.1, RT); FDCAN Write per frame (Raw data, Identifier / ' ...
    'DataLength from the dialog, BRS on CAN2) in Enabled Subsystems on valid; stmutilitieslib Register Read//Write ' ...
    'IWDG1 KR = 0xAAAA on wdg_kick. Root buses as in SIM']);
end

% ===========================================================================================
% T09 stubs (final ports)
% ===========================================================================================
function local_stub_desc(n, txt)
set_param(n, 'Description', ['T08 STUB for T09 (final ports; replace the contents, keep the ports). ' txt]);
end

function local_fw_safety_supervisor(n)
CT = 'Ts.ctrl';
IN(n, 'fw10', 1, '', CT); IN(n, 'mode_prev', 2, 'uint8', CT);
A(n, 'simulink/Sinks/Terminator', 'T_fw10'); A(n, 'simulink/Sinks/Terminator', 'T_mode_prev');
L(n, 'fw10/1', 'T_fw10/1', 'fw10'); L(n, 'mode_prev/1', 'T_mode_prev/1', 'mode_prev');
CST(n, 'Fault_word', '0', 'uint32', CT); CST(n, 'Warn_word', '0', 'uint32', CT);
CST(n, 'Fault_cls', 'false(1,3)', 'boolean', CT); CST(n, 'K_der', '1', 'single', CT);
OUT(n, 'fault_word', 1, 'uint32'); OUT(n, 'warn_word', 2, 'uint32'); OUT(n, 'fault_cls', 3, 'boolean'); OUT(n, 'k_der', 4, 'single');
L(n, 'Fault_word/1', 'fault_word/1'); L(n, 'Warn_word/1', 'warn_word/1'); L(n, 'Fault_cls/1', 'fault_cls/1'); L(n, 'K_der/1', 'k_der/1');
local_stub_desc(n, ['fw_safety_supervisor (Task_10, ICD 6 fault word, T09.2). in 1 fw10 = Task_10 platform virtual bus ' ...
    '(elements listed in the Shaddad_MCU Description), 2 mode_prev uint8 (mode of the previous Task_10 sample, Unit ' ...
    'Delay in Task_10, breaks the supervisor <-> mode-manager loop). out 1 fault_word uint32, 2 warn_word uint32, ' ...
    '3 fault_cls boolean[3] [class A active, class B active, class C active], 4 k_der single (min of derates). ' ...
    'Stub: 0, 0, false, 1.']);
end

function local_fw_mode_manager(n)
CT = 'Ts.ctrl';
IN(n, 'fw10', 1, '', CT); IN(n, 'fault_word', 2, 'uint32', CT); IN(n, 'fault_cls', 3, 'boolean', CT, '3');
for p = {'fw10', 'fault_word', 'fault_cls'}
    A(n, 'simulink/Sinks/Terminator', ['T_' p{1}]); L(n, [p{1} '/1'], ['T_' p{1} '/1'], p{1});
end
CST(n, 'Mode', '2', 'uint8', CT); CST(n, 'Cmd_src', '1', 'uint8', CT); CST(n, 'Drive_enable', 'false', 'boolean', CT);
CST(n, 'Rearm_req', 'false', 'boolean', CT); CST(n, 'Bms_req', '[true false false false]', 'boolean', CT);
OUT(n, 'mode', 1, 'uint8'); OUT(n, 'cmd_src', 2, 'uint8'); OUT(n, 'drive_enable', 3, 'boolean');
OUT(n, 'rearm_req', 4, 'boolean'); OUT(n, 'bms_req', 5, 'boolean');
L(n, 'Mode/1', 'mode/1'); L(n, 'Cmd_src/1', 'cmd_src/1'); L(n, 'Drive_enable/1', 'drive_enable/1');
L(n, 'Rearm_req/1', 'rearm_req/1'); L(n, 'Bms_req/1', 'bms_req/1');
local_stub_desc(n, ['fw_mode_manager (Task_10, MATLAB Function mcu_mode_manager.m in T09). in 1 fw10 (Task_10 platform ' ...
    'virtual bus), 2 fault_word uint32, 3 fault_cls boolean[3] [A B C]. out 1 mode uint8 (ShaddadMode code), 2 cmd_src ' ...
    'uint8 (ShaddadCmdSrc), 3 drive_enable boolean, 4 rearm_req boolean (SELFTEST entry and every reset_req edge, D30), ' ...
    '5 bms_req boolean[4] [HV_Enable_Req Charge_Enable_Req Fault_Reset_Req Shutdown_Req] (to BMS_REQ through RT ' ...
    '0.01 -> 0.05). Stub: STANDBY, RC, false, false, [1 0 0 0].']);
end

function local_fw_vehicle_ctrl(n)
CT = 'Ts.ctrl';
IN(n, 'fw10', 1, '', CT); IN(n, 'mode', 2, 'uint8', CT); IN(n, 'cmd_src', 3, 'uint8', CT);
IN(n, 'drive_enable', 4, 'boolean', CT); IN(n, 'fault_word', 5, 'uint32', CT); IN(n, 'k_der', 6, 'single', CT);
for p = {'mode', 'cmd_src', 'fault_word', 'k_der'}
    A(n, 'simulink/Sinks/Terminator', ['T_' p{1}]); L(n, [p{1} '/1'], ['T_' p{1} '/1'], p{1});
end
% stub: open-loop RC pass-through (v_rc, w_rc -> skid mixer -> motor speed setpoints)
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_rc', 'OutputSignals', 'rc.v_rc,rc.w_rc');
L(n, 'fw10/1', 'Sel_rc/1', 'fw10');
A(n, 'simulink/Math Operations/Gain', 'W_half', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2');
A(n, 'simulink/Math Operations/Sum', 'V_L', 'Inputs', '+-'); A(n, 'simulink/Math Operations/Sum', 'V_R', 'Inputs', '++');
L(n, 'Sel_rc/2', 'W_half/1'); L(n, 'Sel_rc/1', 'V_L/1'); L(n, 'W_half/1', 'V_L/2'); L(n, 'Sel_rc/1', 'V_R/1'); L(n, 'W_half/1', 'V_R/2');
A(n, 'simulink/Math Operations/Gain', 'Wm_L', 'Gain', 'P.electronics.drv.k_v2w');
A(n, 'simulink/Math Operations/Gain', 'Wm_R', 'Gain', 'P.electronics.drv.k_v2w');
L(n, 'V_L/1', 'Wm_L/1', 'v_L'); L(n, 'V_R/1', 'Wm_R/1', 'v_R');
A(n, 'simulink/Sources/Counter Free-Running', 'Alive', 'NumBits', '8', 'tsamp', CT);
CST(n, 'Mode_speed', '1', 'uint8', CT); CST(n, 'Zero_s', '0', 'single', CT);
CST(n, 'T_peak', 'P.electronics.drv.T_peak', 'single', CT); CST(n, 'False_b', 'false', 'boolean', CT);
LOGIC(n, 'Park', 'NOT', 1); L(n, 'drive_enable/1', 'Park/1', 'drive_enable');
sides = {'L', 'Wm_L/1'; 'R', 'Wm_R/1'};
for k = 1:2
    sd = sides{k, 1};
    DTC(n, ['W_ref_' sd], 'single'); L(n, sides{k, 2}, ['W_ref_' sd '/1']);
    BCB(n, ['Side_' sd], 'Bus_DriveSideCmd', {'enable', 'drive_enable/1'; 'mode', 'Mode_speed/1'; ...
        'w_ref', ['W_ref_' sd '/1']; 'tau_ff', 'Zero_s/1'; 'tau_lim_mot', 'T_peak/1'; 'tau_lim_reg', 'T_peak/1'; ...
        'park_req', 'Park/1'; 'fault_reset', 'False_b/1'; 'alive', 'Alive/1'});
end
A(n, 'simulink/Signal Routing/Bus Creator', 'Cmd_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_DriveCmd', 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Side_L/1', 'Cmd_BC/1', 'L'); L(n, 'Side_R/1', 'Cmd_BC/2', 'R');
DTC(n, 'V_s', 'single'); DTC(n, 'W_s', 'single'); DTC(n, 'VL_s', 'single'); DTC(n, 'VR_s', 'single');
L(n, 'Sel_rc/1', 'V_s/1'); L(n, 'Sel_rc/2', 'W_s/1'); L(n, 'V_L/1', 'VL_s/1'); L(n, 'V_R/1', 'VR_s/1');
CST(n, 'One_s', '1', 'single', CT);
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Vc_tel', 'NumInputs', '11');
srcs = {'V_s/1', 'W_s/1', 'V_s/1', 'W_s/1', 'Zero_s/1', 'Zero_s/1', 'W_s/1', 'VL_s/1', 'VR_s/1', 'One_s/1', 'One_s/1'};
for k = 1:11, L(n, srcs{k}, sprintf('Vc_tel/%d', k)); end
OUT(n, 'drive_cmd', 1, 'Bus: Bus_DriveCmd'); OUT(n, 'vc_tel', 2, 'single');
L(n, 'Cmd_BC/1', 'drive_cmd/1', 'drive_cmd'); L(n, 'Vc_tel/1', 'vc_tel/1', 'vc_tel');
local_stub_desc(n, ['fw_vehicle_ctrl (Task_10, T09.3). in 1 fw10 (Task_10 platform virtual bus), 2 mode uint8, ' ...
    '3 cmd_src uint8, 4 drive_enable boolean, 5 fault_word uint32, 6 k_der single. out 1 drive_cmd Bus_DriveCmd ' ...
    '(-> drive_cmd_pack), 2 vc_tel single[11] [v_target w_target v_ref w_ref a_ref alpha_ref w_mix vtrack_ref_L ' ...
    'vtrack_ref_R k_tr_L k_tr_R] (-> fw_telemetry; v_ref / w_ref are MCU_STATUS v_applied / w_applied). Stub: open-loop ' ...
    'RC pass-through w_ref = k_v2w*(v_rc -/+ w_rc*chi*B/2), mode SPEED, enable = drive_enable, park_req = NOT ' ...
    'drive_enable, tau limits T_peak, alive Counter Free-Running.']);
end

function local_fw_implement_aux(n)
CM = 'Ts.comm';
IN(n, 'fw50', 1, '', CM);
A(n, 'simulink/Sinks/Terminator', 'T_fw50'); L(n, 'fw50/1', 'T_fw50/1', 'fw50');
CST(n, 'Do_aux', '2^P.electronics.do.companion_pwr_en + 2^P.electronics.do.lidar_pwr_en', 'uint32', CM);
CST(n, 'Pwm', 'zeros(1,4)', 'single', CM); CST(n, 'Impl_en', 'false', 'boolean', CM);
CST(n, 'Impl_rst', 'false', 'boolean', CM); CST(n, 'Impl_rpm', '0', 'single', CM);
OUT(n, 'do_aux', 1, 'uint32'); OUT(n, 'pwm_duty', 2, 'single'); OUT(n, 'impl_enable', 3, 'boolean');
OUT(n, 'impl_fault_reset', 4, 'boolean'); OUT(n, 'impl_speed_rpm', 5, 'single');
L(n, 'Do_aux/1', 'do_aux/1'); L(n, 'Pwm/1', 'pwm_duty/1'); L(n, 'Impl_en/1', 'impl_enable/1');
L(n, 'Impl_rst/1', 'impl_fault_reset/1'); L(n, 'Impl_rpm/1', 'impl_speed_rpm/1');
local_stub_desc(n, ['fw_implement_aux (Task_50, T09.4). in 1 fw50 = Task_50 platform virtual bus (elements listed in ' ...
    'the Shaddad_MCU Description). out 1 do_aux uint32 (DO word bits 2..19 of ICD 6; bits 0/1 are owned by ' ...
    'fw_watchdog_mgr and masked by the platform), 2 pwm_duty single[4] 0..1 [pump lift fan1 fan2] (RT -> 0.01), ' ...
    '3 impl_enable boolean, 4 impl_fault_reset boolean, 5 impl_speed_rpm single (rpm at blade / rotor) -> IMPL_CMD. ' ...
    'Stub: COMPANION_PWR_EN + LIDAR_PWR_EN, zeros, implement off.']);
end

function local_fw_watchdog_mgr(n)
BT = 'Ts.base';
IN(n, 'alive', 1, 'uint8', BT, '3'); IN(n, 'fault_inject', 2, 'uint32', BT); IN(n, 'fault_word', 3, 'uint32', BT);
IN(n, 'mode', 4, 'uint8', BT); IN(n, 'rearm_req', 5, 'boolean', BT);
for p = {'alive', 'fault_inject', 'fault_word', 'mode', 'rearm_req'}
    A(n, 'simulink/Sinks/Terminator', ['T_' p{1}]); L(n, [p{1} '/1'], ['T_' p{1} '/1'], p{1});
end
CST(n, 'Kick', 'true', 'boolean', BT); CST(n, 'Safety_ok', 'true', 'boolean', BT); CST(n, 'Reset_req', 'false', 'boolean', BT);
OUT(n, 'wdg_kick', 1, 'boolean'); OUT(n, 'mcu_safety_ok', 2, 'boolean'); OUT(n, 'safety_reset_req', 3, 'boolean');
L(n, 'Kick/1', 'wdg_kick/1'); L(n, 'Safety_ok/1', 'mcu_safety_ok/1'); L(n, 'Reset_req/1', 'safety_reset_req/1');
local_stub_desc(n, ['fw_watchdog_mgr (Task_1k, T09.5). in 1 alive uint8[3] (task alive counters [Task_10 Task_50 ' ...
    'Task_100] through RT -> 0.001; a stalled task, fault_inject b0/b1/b2, freezes its counter), 2 fault_inject uint32 ' ...
    '(b0-b3 stall / overrun), 3 fault_word uint32 (RT 0.01 -> 0.001), 4 mode uint8 (RT), 5 rearm_req boolean (RT). ' ...
    'out 1 wdg_kick boolean (-> IWDG1 refresh), 2 mcu_safety_ok boolean (DO b0, D30), 3 safety_reset_req boolean ' ...
    '(DO b1 monostable). Stub: true, true, false.']);
end

% ===========================================================================================
% helpers
% ===========================================================================================
function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function L(sys, src, dst, nm)
% signal line src -> dst ('Blk/k'), named (layout spec 6.1, L5) by shaddad_bu('nline'): nm, else the name the source
% port's line already has, else the source block's name ('_k' for output k of a block with several outputs); a Bus
% Selector's output keeps '<element>'
if nargin < 4, nm = ''; end
shaddad_bu('nline', sys, src, dst, nm);
end

function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line src ('Blk/k'; the
% line named nm when given) and feeds its output k to dsts{k} ('Blk/j'; an Enable port as 'Blk/enable')
if nargin < 6, nm = ''; end
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts)
    if endsWith(dsts{k}, '/enable')
        ENLINE(sys, sprintf('%s/%d', blk, k), extractBefore(dsts{k}, '/enable'), '');
    else
        L(sys, sprintf('%s/%d', blk, k), dsts{k});
    end
end
end

function p = IN(sys, name, port, type, ts, dims)
p = A(sys, 'simulink/Sources/In1', name, 'Port', num2str(port));
if ~isempty(type), set_param(p, 'OutDataTypeStr', type); end
if ~isempty(ts), set_param(p, 'SampleTime', ts); end
if nargin >= 6 && ~isempty(dims), set_param(p, 'PortDimensions', dims); end
end

function p = OUT(sys, name, port, type)
p = A(sys, 'simulink/Sinks/Out1', name, 'Port', num2str(port));
if ~isempty(type), set_param(p, 'OutDataTypeStr', type); end
end

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function p = DTC(sys, name, type)
p = A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', type, ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
end

function p = RT(sys, name, ts, ic)
p = A(sys, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', ts, 'InitialCondition', ic);
end

function p = CMP(sys, name, op, c)
p = A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', name, 'relop', op, 'const', c, ...
    'OutDataTypeStr', 'boolean');
end

function p = LOGIC(sys, name, op, nin)
p = A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(nin));
end

function p = SSR(sys, name, ref)
p = A(sys, 'simulink/Ports & Subsystems/Subsystem Reference', name, 'ReferencedSubsystem', ref);
end

function p = REG(sys, name, grp, reg, op, ts, valueSrc)
% stmutilitieslib Register Read//Write placeholder (STM32H743, D60). In this installation the block's
% mask callbacks (targetHardware / opCode / regGrp -> cmsisSvdBlockCallback) fail when the parameters
% are set from a script ('Invalid set access of parameter svdFileInput', device list '<empty>' until the
% STM32 device SVD data of the board setup exist), so the block keeps its library defaults and the
% intended register goes to its Description; T17 selects it in the board setup. For a write, the value
% signal (valueSrc) ends in a Terminator next to the block until then.
p = A(sys, 'stmutilitieslib/Register Read//Write', name);
set_param(p, 'Description', sprintf(['INTENDED (set in the T17 board setup): tgtHW ''STM32H7xx Based'', devID ' ...
    '''STM32H743'', regGrpName ''%s'', regName ''%s'', opCode ''%s''.'], grp, reg, op));
try
    set_param(p, 'SampleTime', ts);
catch %#ok<CTCH> keep the library sample time
end
if nargin >= 7 && ~isempty(valueSrc)
    A(sys, 'simulink/Sinks/Terminator', [name '_value']);
    L(sys, valueSrc, [name '_value/1']);
end
if strcmp(op, 'Write')
    A(sys, 'simulink/Sinks/Terminator', [name '_rd']);
    L(sys, [name '/1'], [name '_rd/1']);
end
end

function BCV(sys, name, map)
% virtual Bus Creator: element names = line names (spec 6.3), shaddad_bu('vbus'): a source whose line already
% carries another name is the error shaddad_bu:vbus
shaddad_bu('vbus', sys, name, map);
end

function p = BCB(sys, name, bus, map)
% nonvirtual Bus Creator with a bus object; map {element, source} in any order
B = evalin('base', bus);
el = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
assert(numel(el) == size(map, 1) && all(ismember(el, map(:, 1))), 'build_fw_platform:bus', ...
    '%s: map does not cover the elements of %s', name, bus);
p = A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(el)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(el)
    src = map{strcmp(map(:, 1), el{k}), 2};
    h = add_line(sys, src, sprintf('%s/%d', name, k), 'autorouting', 'smart');
    local_name_line(h, el{k});
end
end

function local_name_line(h, nm)
% Bus Selector outputs carry their element name; other sources get the line name (an existing
% different name on a branched line is kept, Simulink names the whole line)
src = get_param(h, 'SrcBlockHandle');
if strcmp(get_param(src, 'BlockType'), 'BusSelector'), return, end
cur = get_param(h, 'Name');
if isempty(cur), set_param(h, 'Name', nm); end
end

function s = ENSUB(sys, name, ins, outs)
% Enabled Subsystem (states held); outs = {name, IC} with outputs held when disabled
s = [sys '/' name];
add_block('built-in/SubSystem', s);
add_block('simulink/Ports & Subsystems/Enable', [s '/Enable'], 'StatesWhenEnabling', 'held');
for k = 1:numel(ins), A(s, 'simulink/Sources/In1', ins{k}, 'Port', num2str(k)); end
for k = 1:size(outs, 1)
    A(s, 'simulink/Sinks/Out1', outs{k, 1}, 'Port', num2str(k), 'OutputWhenDisabled', 'held', ...
        'InitialOutput', outs{k, 2});
end
end

function ENLINE(sys, src, sub, nm)
% the line src ('Blk/k') -> Enable port of sub, named as shaddad_bu('nline') names a line (L5): nm when the line
% has no name yet, else the name it has, else the source block's name ('_k' for output k of several); a Bus
% Selector's output keeps '<element>'
ph = get_param([sys '/' sub], 'PortHandles');
sp = strsplit(src, '/');
phs = get_param([sys '/' sp{1}], 'PortHandles');
k = str2double(sp{2});
h = add_line(sys, phs.Outport(k), ph.Enable(1), 'autorouting', 'smart');
if isempty(nm)
    nm = sp{1};
    if numel(phs.Outport) > 1, nm = sprintf('%s_%d', nm, k); end
end
local_name_line(h, nm);
end

function AGE(sys, name, satExpr, icExpr)
% age counter subsystem: in 'fresh' (boolean, scalar or vector) -> out 'ticks' uint16 =
% fresh ? 0 : min(prev + 1, sat); starts at icExpr (default sat = "never seen")
if nargin < 4, icExpr = satExpr; end
s = [sys '/' name];
add_block('built-in/SubSystem', s);
A(s, 'simulink/Sources/In1', 'fresh', 'Port', '1');
A(s, 'simulink/Sinks/Out1', 'ticks', 'Port', '1');
A(s, 'simulink/Discrete/Unit Delay', 'Prev', 'InitialCondition', icExpr, 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'One', 'Value', '1', 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'Sat', 'Value', satExpr, 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'Zero', 'Value', '0', 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Math Operations/Sum', 'Inc', 'Inputs', '++', 'OutDataTypeStr', 'uint16', 'SaturateOnIntegerOverflow', 'on');
A(s, 'simulink/Math Operations/MinMax', 'Lim', 'Function', 'min', 'Inputs', '2');
A(s, 'simulink/Signal Routing/Switch', 'Rst', 'Criteria', 'u2 ~= 0');
L(s, 'Prev/1', 'Inc/1', 'prev'); L(s, 'One/1', 'Inc/2', 'one'); L(s, 'Inc/1', 'Lim/1', 'inc'); L(s, 'Sat/1', 'Lim/2', 'sat');
L(s, 'Zero/1', 'Rst/1', 'zero'); L(s, 'fresh/1', 'Rst/2', 'fresh'); L(s, 'Lim/1', 'Rst/3', 'inc_sat');
L(s, 'Rst/1', 'Prev/1', 'ticks'); L(s, 'Rst/1', 'ticks/1');
end

function DEBOUNCE(sys, name, icExpr)
% 3-position debounce (spec 2.7.2 item 4): Tapped Delay (debounce_n - 1 delays + current) -> MinMax min / max
% -> Relational == -> Switch (accept the new position) / Unit Delay (hold)
s = [sys '/' name];
add_block('built-in/SubSystem', s);
A(s, 'simulink/Sources/In1', 'pos', 'Port', '1');
A(s, 'simulink/Sinks/Out1', 'pos_deb', 'Port', '1');
A(s, 'simulink/Discrete/Tapped Delay', 'Taps', 'NumDelays', 'P.electronics.rcmap.debounce_n-1', ...
    'includeCurrent', 'on', 'vinit', icExpr, 'samptime', '-1');
A(s, 'simulink/Math Operations/MinMax', 'Min', 'Function', 'min', 'Inputs', '1');
A(s, 'simulink/Math Operations/MinMax', 'Max', 'Function', 'max', 'Inputs', '1');
A(s, 'simulink/Logic and Bit Operations/Relational Operator', 'Agree', 'Operator', '==');
A(s, 'simulink/Signal Routing/Switch', 'Accept', 'Criteria', 'u2 ~= 0');
A(s, 'simulink/Discrete/Unit Delay', 'Hold', 'InitialCondition', icExpr, 'SampleTime', '-1');
L(s, 'pos/1', 'Taps/1', 'pos'); L(s, 'Taps/1', 'Min/1', 'taps'); L(s, 'Taps/1', 'Max/1');
L(s, 'Min/1', 'Agree/1', 'taps_min'); L(s, 'Max/1', 'Agree/2', 'taps_max');
L(s, 'pos/1', 'Accept/1'); L(s, 'Agree/1', 'Accept/2', 'agree'); L(s, 'Hold/1', 'Accept/3', 'held');
L(s, 'Accept/1', 'Hold/1', 'pos_deb'); L(s, 'Accept/1', 'pos_deb/1');
end
