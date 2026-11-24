function files = build_can2_codecs(which)
%BUILD_CAN2_CODECS  CAN-FD (CAN2) frame codecs of ICD 5 (owner: electronics, task T08).
%
%   build_can2_codecs()          build all ten Subsystem Reference files
%   build_can2_codecs(names)     build only these (e.g. {'can2_mcu_imu_pack'})
%   files = build_can2_codecs(...)   project-relative paths written
%
%   subsystems/electronics/can2_<msg>_pack.slx    engineering bus -> Byte Pack -> uint8[64] zero padded
%                                                 + id + len + valid -> Bus_CanFrame64
%   subsystems/electronics/can2_<msg>_unpack.slx  Bus_CanFrame64 -> Byte Unpack -> engineering bus
%   <msg> = comp_cmd (0x100, Bus_GncCmd), mcu_status (0x200, Bus_McuStatus), mcu_imu (0x201,
%   Bus_ImuMeas), mcu_odom (0x202, Bus_OdomMeas), mcu_power (0x203, Bus_BmsStatus).
%   Layouts (byte offsets, types, resolutions) are the tables of docs/icd.md section 5; ids and
%   lengths come from P.electronics.can.id / .len, resolutions from P.electronics.can.res.
%   Blocks: mcblib/Signal Management/Byte Pack and Byte Unpack (byteAlign 1, little-endian),
%   Gain + Data Type Conversion (round, saturate) for scaled integers, commutil2 Bit to Integer /
%   Integer to Bit Converters (8 bits, LSB first) for flag bytes. Inherited sample time and
%   single / integer types only (the same files run on the MCU, in Link_MCU_GNC and in companion_io).
%   Unpack specials: element 'valid' = payload valid flag (if the layout carries one) AND frame.valid;
%   Bus_McuStatus.t_mcu = unwrapped t_ms / 1000 (companion side, D41 / T10); Bus_BmsStatus.P_pack =
%   V_pack * I_pack and the fields MCU_POWER does not carry (CVL, DVL, P_chg_lim) = 0;
%   Bus_GncCmd.t_stamp = 0 (not transmitted).
%   Requires P, Ts and Bus_* in the base workspace (shaddad_params).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out each file with
%   shaddad_layout(<file>, 'Recursive', true). Every signal line is named (L5, shaddad_bu('nline'); a Bus Selector's
%   output keeps '<element>'). The root ports stay classic (ICD 5; the MCU, Link_MCU_GNC and companion_io feed them).
%   Inside, one Bus Selector right in front of each consumer (spec 6.2), fed by the bus line x or frame: pack
%   Sel_<slot> for a run of raw slots that go straight into Pack (named after its first slot), for the boolean flags
%   of a flag byte and for a vector slot, Sel_<field> in front of the conversion of one field; unpack Sel_data
%   (Payload) and one Sel_valid* per consumer of frame.valid. Unpack: the decoded fields travel as one virtual bus
%   'decoded' in the frame's slot order (virtual Bus Creator Decoded, spec 6.3), and the Bus Selector Sel_decoded
%   right in front of Bus_BC picks them in the element order of the bus type.

if nargin < 1 || isempty(which), which = {}; end
if ischar(which) || isstring(which), which = cellstr(which); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dirOut = fullfile(root, 'subsystems', 'electronics');
S = local_specs();
files = {};
for k = 1:numel(S)
    for dir = {'pack', 'unpack'}
        name = ['can2_' S(k).msg '_' dir{1}];
        if ~isempty(which) && ~any(strcmp(which, name)), continue, end
        f = fullfile(dirOut, [name '.slx']);
        shaddad_bu('fresh', name, f, 'ssr');
        if strcmp(dir{1}, 'pack'), local_pack(name, S(k)); else, local_unpack(name, S(k)); end
        shaddad_bu('save', name, f);
        close_system(name, 0);
        files{end+1} = fullfile('subsystems', 'electronics', [name '.slx']); %#ok<AGROW>
    end
end
end

% ===========================================================================================
function S = local_specs()
% slot = {type, dims, kind, fields, res}; kinds: raw | scaled | flags | vec | zero | tms
S = struct('msg', {}, 'bus', {}, 'slots', {}, 'desc', {});
sl = @(t, d, k, f, r) struct('type', t, 'dims', d, 'kind', k, 'fields', {f}, 'res', r);
% 0x100 COMP_CMD (32 B), companion -> MCU, Bus_GncCmd
s = [sl('uint8', 1, 'raw', {'seq'}, 1), sl('uint8', 1, 'raw', {'mode_req'}, 1), ...
     sl('uint8', 1, 'raw', {'gnc_state'}, 1), ...
     sl('uint8', 1, 'flags', {'cmd_valid', 'gnc_ready', 'soft_stop_req', 'v_ground_valid', ...
        'mower_enable', 'tiller_enable', 'spray_enable', 'buzzer_cmd'}, 1), ...
     sl('single', 1, 'raw', {'v_cmd'}, 1), sl('single', 1, 'raw', {'omega_cmd'}, 1), ...
     sl('single', 1, 'raw', {'accel_limit'}, 1), sl('single', 1, 'raw', {'v_ground_est'}, 1), ...
     sl('uint16', 1, 'raw', {'spray_section_mask'}, 1), sl('uint8', 1, 'raw', {'beacon_cmd'}, 1), ...
     sl('uint8', 1, 'raw', {'nav_fix'}, 1), ...
     sl('uint16', 1, 'scaled', {'mower_speed_cmd'}, 'P.electronics.can.res.mower_w'), ...
     sl('uint16', 1, 'scaled', {'tiller_speed_cmd'}, 'P.electronics.can.res.tiller_w'), ...
     sl('uint8', 1, 'scaled', {'deck_height_cmd'}, 'P.electronics.can.res.deck_h'), ...
     sl('uint8', 1, 'scaled', {'tiller_depth_cmd'}, 'P.electronics.can.res.till_d'), ...
     sl('uint16', 1, 'scaled', {'spray_flow_cmd'}, 'P.electronics.can.res.spray_flow')];
S(end+1) = struct('msg', 'comp_cmd', 'bus', 'Bus_GncCmd', 'slots', s, 'desc', ...
    ['0x100 COMP_CMD (32 B, 50 ms): 0 u8 seq; 1 u8 mode_req; 2 u8 gnc_state; 3 u8 flags [b0 cmd_valid b1 gnc_ready ' ...
     'b2 soft_stop_req b3 v_ground_valid b4 mower_enable b5 tiller_enable b6 spray_enable b7 buzzer_cmd]; 4 single v_cmd; ' ...
     '8 single omega_cmd; 12 single accel_limit; 16 single v_ground_est; 20 u16 spray_section_mask; 22 u8 beacon_cmd; ' ...
     '23 u8 nav_fix; 24 u16 mower_speed_cmd [0.1 rad/s]; 26 u16 tiller_speed_cmd [0.01 rad/s]; 28 u8 deck_height_cmd ' ...
     '[1 mm]; 29 u8 tiller_depth_cmd [1 mm]; 30 u16 spray_flow_cmd [0.01 L/min]. t_stamp is not transmitted (0).']);
% 0x200 MCU_STATUS (48 B), MCU -> companion, Bus_McuStatus (+ t_ms input on the pack side)
s = [sl('uint32', 1, 'tms', {'t_mcu'}, 1), sl('uint8', 1, 'raw', {'mode'}, 1), ...
     sl('uint8', 1, 'raw', {'cmd_src'}, 1), sl('uint8', 1, 'raw', {'rc_mode_sw'}, 1), ...
     sl('uint8', 1, 'raw', {'rc_aux_cmd'}, 1), sl('uint8', 1, 'raw', {'rc_aux_seq'}, 1), ...
     sl('uint8', 1, 'flags', {'rc_link_ok', 'rc_failsafe', 'estop_active', 'drive_ready', ...
        'power_ready', 'implement_ready', 'charging', 'wdg_reset_seen'}, 1), ...
     sl('uint8', 1, 'raw', {'implement_type'}, 1), sl('uint8', 1, 'raw', {'heartbeat'}, 1), ...
     sl('uint32', 1, 'raw', {'fault_word'}, 1), sl('uint32', 1, 'raw', {'warn_word'}, 1), ...
     sl('uint32', 1, 'raw', {'drive_fault_word'}, 1), sl('single', 2, 'raw', {'v_track_meas'}, 1), ...
     sl('single', 1, 'raw', {'v_applied'}, 1), sl('single', 1, 'raw', {'w_applied'}, 1), ...
     sl('uint16', 1, 'raw', {'gnc_cmd_age_ms'}, 1), ...
     sl('uint8', 1, 'scaled', {'rc_lq'}, 'P.electronics.can.res.rc_lq'), ...
     sl('uint8', 1, 'zero', {}, 1), sl('single', 1, 'raw', {'T_encl'}, 1)];
S(end+1) = struct('msg', 'mcu_status', 'bus', 'Bus_McuStatus', 'slots', s, 'desc', ...
    ['0x200 MCU_STATUS (48 B, 50 ms): 0 u32 t_ms; 4 u8 mode; 5 u8 cmd_src; 6 u8 rc_mode_sw; 7 u8 rc_aux_cmd; ' ...
     '8 u8 rc_aux_seq; 9 u8 flags [b0 rc_link_ok b1 rc_failsafe b2 estop_active b3 drive_ready b4 power_ready ' ...
     'b5 implement_ready b6 charging b7 wdg_reset_seen]; 10 u8 implement_type; 11 u8 heartbeat; 12 u32 fault_word; ' ...
     '16 u32 warn_word; 20 u32 drive_fault_word; 24/28 single v_track_L/R; 32 single v_applied; 36 single w_applied; ' ...
     '40 u16 gnc_cmd_age_ms; 42 u8 rc_lq [1 %]; 43 reserved; 44 single T_encl [K]. Pack: in 1 status Bus_McuStatus ' ...
     '(t_mcu and valid ignored), in 2 t_ms uint32 (MCU millisecond tick). Unpack: t_mcu = unwrapped t_ms / 1000 s, ' ...
     'valid = frame.valid.']);
% 0x201 MCU_IMU (48 B), Bus_ImuMeas
s = [sl('uint32', 1, 'raw', {'t_us'}, 1), sl('uint16', 1, 'raw', {'seq'}, 1), ...
     sl('uint8', 1, 'raw', {'valid'}, 1), sl('uint8', 1, 'zero', {}, 1), ...
     sl('single', 3, 'raw', {'accel_B'}, 1), sl('single', 3, 'raw', {'gyro_B'}, 1), ...
     sl('single', 3, 'raw', {'mag_B'}, 1), sl('single', 1, 'raw', {'temp_degC'}, 1)];
S(end+1) = struct('msg', 'mcu_imu', 'bus', 'Bus_ImuMeas', 'slots', s, 'desc', ...
    ['0x201 MCU_IMU (48 B, 5 ms): 0 u32 t_us; 4 u16 seq; 6 u8 valid; 7 reserved; 8 single acc[3] (body FLU, ' ...
     'physical); 20 single gyr[3]; 32 single mag[3] [uT]; 44 single temp [degC]. Unpack valid = byte 6 AND frame.valid.']);
% 0x202 MCU_ODOM (32 B), Bus_OdomMeas
s = [sl('uint32', 1, 'raw', {'t_us'}, 1), sl('int32', 2, 'raw', {'encCnt_LR'}, 1), ...
     sl('single', 2, 'raw', {'vTrack_LR'}, 1), sl('single', 2, 'raw', {'wMotor_LR'}, 1), ...
     sl('uint8', 1, 'raw', {'valid'}, 1), sl('uint8', 3, 'zero', {}, 1)];
S(end+1) = struct('msg', 'mcu_odom', 'bus', 'Bus_OdomMeas', 'slots', s, 'desc', ...
    ['0x202 MCU_ODOM (32 B, 10 ms): 0 u32 t_us; 4/8 i32 enc_cnt_L/R; 12/16 single v_track_L/R; 20/24 single ' ...
     'w_m_L/R; 28 u8 valid; 29-31 reserved. Unpack valid = byte 28 AND frame.valid.']);
% 0x203 MCU_POWER (64 B), Bus_BmsStatus
s = [sl('single', 13, 'vec', {'V_pack', 'I_pack', 'SOC', 'SOH', 'T_cell_max', 'T_cell_min', 'V_cell_min', ...
        'V_cell_max', 'CCL', 'DCL', 'P_dis_lim', 'E_remaining_Wh', 'Runtime_s'}, 1), ...
     sl('uint32', 1, 'raw', {'alarm_word'}, 1), sl('uint32', 1, 'raw', {'warn_word'}, 1), ...
     sl('uint8', 1, 'raw', {'fault_level'}, 1), sl('uint8', 1, 'raw', {'contactor_state'}, 1), ...
     sl('uint8', 1, 'flags', {'hv_ready', 'hv_opening_warn', 'charging', 'charge_complete', ...
        'bms_critical', 'valid', '', ''}, 1), ...
     sl('uint8', 1, 'raw', {'alive'}, 1)];
S(end+1) = struct('msg', 'mcu_power', 'bus', 'Bus_BmsStatus', 'slots', s, 'desc', ...
    ['0x203 MCU_POWER (64 B, 100 ms, BMS forward): 0..51 single V_pack I_pack SOC SOH T_cell_max T_cell_min ' ...
     'V_cell_min V_cell_max CCL DCL P_dis_lim E_remaining_Wh Runtime_s; 52 u32 alarm_word; 56 u32 warn_word; ' ...
     '60 u8 fault_level; 61 u8 contactor_state; 62 u8 flags [b0 hv_ready b1 hv_opening_warn b2 charging ' ...
     'b3 charge_complete b4 bms_critical b5 valid]; 63 u8 alive. Unpack: P_pack = V_pack*I_pack, CVL = DVL = ' ...
     'P_chg_lim = 0 (not carried), valid = b5 AND frame.valid.']);
end

% ===========================================================================================
function local_pack(n, S)
B = evalin('base', S.bus);
etype = local_etypes(B);
A(n, 'simulink/Sources/In1', 'x', 'Port', '1', 'OutDataTypeStr', ['Bus: ' S.bus], 'SampleTime', '-1');
hasTms = any(strcmp({S.slots.kind}, 'tms'));
if hasTms
    A(n, 'simulink/Sources/In1', 't_ms', 'Port', '2', 'OutDataTypeStr', 'uint32', 'SampleTime', '-1');
end
% one Bus Selector right in front of each consumer, fed by the bus line x (spec 6.2): Sel_<slot> for a run of raw
% slots that go straight into Pack, for the boolean flags of a flag byte and for a vector slot; Sel_<field> in front
% of the conversion of one field
nsl = numel(S.slots);
direct = false(1, nsl);                 % raw slots whose bus element already has the slot's type
for k = 1:nsl
    direct(k) = strcmp(S.slots(k).kind, 'raw') && strcmp(etype.(S.slots(k).fields{1}), S.slots(k).type);
end
types = {S.slots.type};
out = cell(1, nsl);                     % the source of each Pack input
onm = cell(1, nsl);                     % its line name ('' for a Bus Selector output, which keeps '<element>')
nb = 0;
rsel = '';                              % the Bus Selector of the current run of direct raw slots, from slot r0
r0 = 0;
for k = 1:nsl
    s = S.slots(k);
    tag = sprintf('s%02d', k);
    onm{k} = '';
    switch s.kind
        case 'raw'
            f = s.fields{1};
            if direct(k)
                if k == 1 || ~direct(k - 1)     % the first slot of a run
                    j = k;
                    while j < nsl && direct(j + 1), j = j + 1; end
                    rsel = ['Sel_' tag];
                    r0 = k;
                    SEL(n, rsel, 'x', cellfun(@(c) c{1}, {S.slots(k:j).fields}, 'UniformOutput', false));
                end
                out{k} = sprintf('%s/%d', rsel, k - r0 + 1);
            else
                SEL(n, ['Sel_' f], 'x', {f});
                A(n, 'simulink/Signal Attributes/Data Type Conversion', ['DTC_' f], 'OutDataTypeStr', s.type, ...
                    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
                L(n, ['Sel_' f '/1'], ['DTC_' f '/1']);
                out{k} = ['DTC_' f '/1'];
                onm{k} = f;
            end
        case 'scaled'
            f = s.fields{1};
            SEL(n, ['Sel_' f], 'x', {f});
            A(n, 'simulink/Math Operations/Gain', ['Res_' f], 'Gain', ['1/' s.res]);
            A(n, 'simulink/Signal Attributes/Data Type Conversion', ['DTC_' f], 'OutDataTypeStr', s.type, ...
                'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
            L(n, ['Sel_' f '/1'], ['Res_' f '/1']);
            L(n, ['Res_' f '/1'], ['DTC_' f '/1'], [f '_lsb']);
            out{k} = ['DTC_' f '/1'];
            onm{k} = f;
        case 'flags'
            A(n, 'simulink/Signal Routing/Vector Concatenate', ['Bits_' tag], 'NumInputs', '8');
            bf = s.fields(~cellfun(@isempty, s.fields));
            bf = bf(cellfun(@(f) strcmp(etype.(f), 'boolean'), bf));      % boolean flags, straight in
            if ~isempty(bf), SEL(n, ['Sel_' tag], 'x', bf); end
            for j = 1:8
                f = s.fields{j};
                if isempty(f)
                    A(n, 'simulink/Sources/Constant', sprintf('Spare_%s_%d', tag, j), 'Value', 'false', ...
                        'OutDataTypeStr', 'boolean', 'SampleTime', '-1');
                    L(n, sprintf('Spare_%s_%d/1', tag, j), sprintf('Bits_%s/%d', tag, j));
                elseif strcmp(etype.(f), 'boolean')
                    L(n, sprintf('Sel_%s/%d', tag, find(strcmp(bf, f), 1)), sprintf('Bits_%s/%d', tag, j));
                else
                    SEL(n, ['Sel_' f], 'x', {f});
                    A(n, 'simulink/Signal Attributes/Data Type Conversion', ['B_' f], 'OutDataTypeStr', 'boolean');
                    L(n, ['Sel_' f '/1'], ['B_' f '/1']); L(n, ['B_' f '/1'], sprintf('Bits_%s/%d', tag, j), f);
                end
            end
            A(n, 'commutil2/Bit to Integer Converter', ['Flags_' tag], 'nbits', '8', 'bitOrder', 'LSB first', ...
                'outDtype', 'uint8');
            L(n, ['Bits_' tag '/1'], ['Flags_' tag '/1'], ['bits_' tag]);
            out{k} = ['Flags_' tag '/1'];
            onm{k} = ['flags_' tag];
        case 'vec'
            A(n, 'simulink/Signal Routing/Vector Concatenate', ['Vec_' tag], 'NumInputs', num2str(numel(s.fields)));
            SEL(n, ['Sel_' tag], 'x', s.fields);
            for j = 1:numel(s.fields), L(n, sprintf('Sel_%s/%d', tag, j), sprintf('Vec_%s/%d', tag, j)); end
            out{k} = ['Vec_' tag '/1'];
            onm{k} = ['vec_' tag];
        case 'zero'
            A(n, 'simulink/Sources/Constant', ['Rsv_' tag], 'Value', sprintf('zeros(1,%d)', s.dims), ...
                'OutDataTypeStr', s.type, 'SampleTime', '-1');
            out{k} = ['Rsv_' tag '/1'];
            onm{k} = ['rsv_' tag];
        case 'tms'
            out{k} = 't_ms/1';
            onm{k} = 't_ms';
    end
    nb = nb + s.dims*local_size(s.type);
end
A(n, 'mcblib/Signal Management/Byte Pack', 'Pack', 'datatypes', local_cellstr(types), 'byteAlign', '1');
for k = 1:numel(out), L(n, out{k}, sprintf('Pack/%d', k), onm{k}); end
assert(nb == evalin('base', sprintf('P.electronics.can.len.%s', S.msg)), 'build_can2_codecs:len', ...
    '%s: layout %d bytes, P.electronics.can.len.%s differs', S.msg, nb, S.msg);
if nb < 64
    A(n, 'simulink/Sources/Constant', 'Pad', 'Value', sprintf('zeros(1,%d)', 64 - nb), 'OutDataTypeStr', 'uint8', ...
        'SampleTime', '-1');
    A(n, 'simulink/Signal Routing/Vector Concatenate', 'Data64', 'NumInputs', '2');
    L(n, 'Pack/1', 'Data64/1', 'payload'); L(n, 'Pad/1', 'Data64/2', 'pad');
    dsrc = 'Data64/1';
else
    dsrc = 'Pack/1';
end
A(n, 'simulink/Sources/Constant', 'Id', 'Value', ['P.electronics.can.id.' S.msg], 'OutDataTypeStr', 'uint32', 'SampleTime', '-1');
A(n, 'simulink/Sources/Constant', 'Len', 'Value', ['P.electronics.can.len.' S.msg], 'OutDataTypeStr', 'uint8', 'SampleTime', '-1');
A(n, 'simulink/Sources/Constant', 'Valid', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', '-1');
A(n, 'simulink/Signal Routing/Bus Creator', 'Frame_BC', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Id/1', 'Frame_BC/1', 'id'); L(n, 'Len/1', 'Frame_BC/2', 'len'); L(n, dsrc, 'Frame_BC/3', 'data');
L(n, 'Valid/1', 'Frame_BC/4', 'valid');
A(n, 'simulink/Sinks/Out1', 'frame', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_CanFrame64');
L(n, 'Frame_BC/1', 'frame/1', 'frame');
ins = sprintf('in 1 x %s', S.bus);
if hasTms, ins = [ins ', in 2 t_ms uint32']; end
set_param(n, 'Description', sprintf(['%s pack (T08, ICD 5): %s -> out 1 frame Bus_CanFrame64 (id P.electronics.can.id.%s, ' ...
    'len %d, data zero padded to 64 B, valid = true). Inherited rate. Layout: %s'], n, ins, S.msg, nb, S.desc));
end

% ===========================================================================================
function local_unpack(n, S)
B = evalin('base', S.bus);
etype = local_etypes(B);
names = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
A(n, 'simulink/Sources/In1', 'frame', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', 'SampleTime', '-1');
SEL(n, 'Sel_data', 'frame', {'data'});
nb = sum(arrayfun(@(s) s.dims*local_size(s.type), S.slots));
A(n, 'simulink/Signal Routing/Selector', 'Payload', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', sprintf('1:%d', nb), 'InputPortWidth', '64');
L(n, 'Sel_data/1', 'Payload/1');
dimsStr = ['{' strjoin(arrayfun(@(s) num2str(s.dims), S.slots, 'UniformOutput', false), ',') '}'];
A(n, 'mcblib/Signal Management/Byte Unpack', 'Unpack', 'dimensions', dimsStr, ...
    'datatypes', local_cellstr({S.slots.type}), 'byteAlign', '1');
L(n, 'Payload/1', 'Unpack/1', 'payload');
src = struct();                         % element name -> 'blk/port'
ord = {};                               % the elements in the frame's slot order (the inputs of Decoded)
for k = 1:numel(S.slots)
    s = S.slots(k);
    tag = sprintf('s%02d', k);
    up = sprintf('Unpack/%d', k);
    switch s.kind
        case 'raw'
            f = s.fields{1};
            if strcmp(f, 'valid')
                A(n, 'simulink/Signal Attributes/Data Type Conversion', 'DTC_valid', 'OutDataTypeStr', 'boolean');
                L(n, up, 'DTC_valid/1', 'valid_u8');
                src.valid_payload = 'DTC_valid/1';
            elseif strcmp(etype.(f), s.type)
                src.(f) = up;
                ord{end+1} = f; %#ok<AGROW>
            else
                A(n, 'simulink/Signal Attributes/Data Type Conversion', ['DTC_' f], 'OutDataTypeStr', etype.(f));
                L(n, up, ['DTC_' f '/1'], [f '_raw']);
                src.(f) = ['DTC_' f '/1'];
                ord{end+1} = f; %#ok<AGROW>
            end
        case 'scaled'
            f = s.fields{1};
            A(n, 'simulink/Signal Attributes/Data Type Conversion', ['DTC_' f], 'OutDataTypeStr', 'single');
            A(n, 'simulink/Math Operations/Gain', ['Res_' f], 'Gain', s.res, 'OutDataTypeStr', 'single', ...
                'ParamDataTypeStr', 'Inherit: Same as input');
            L(n, up, ['DTC_' f '/1'], [f '_lsb']); L(n, ['DTC_' f '/1'], ['Res_' f '/1']);
            src.(f) = local_cast(n, ['Res_' f '/1'], 'single', etype.(f), f);
            ord{end+1} = f; %#ok<AGROW>
        case 'flags'
            A(n, 'commutil2/Integer to Bit Converter', ['Bits_' tag], 'nbits', '8', 'bitOrder', 'LSB first', ...
                'outDtype', 'boolean');
            A(n, 'simulink/Signal Routing/Demux', ['Dmx_' tag], 'Outputs', '8');
            L(n, up, ['Bits_' tag '/1'], ['flags_' tag]); L(n, ['Bits_' tag '/1'], ['Dmx_' tag '/1'], ['bits_' tag]);
            for j = 1:8
                f = s.fields{j};
                if isempty(f)
                    A(n, 'simulink/Sinks/Terminator', sprintf('T_%s_%d', tag, j));
                    L(n, sprintf('Dmx_%s/%d', tag, j), sprintf('T_%s_%d/1', tag, j), sprintf('spare_%s_%d', tag, j));
                elseif strcmp(f, 'valid')
                    src.valid_payload = sprintf('Dmx_%s/%d', tag, j);
                else
                    src.(f) = local_cast(n, sprintf('Dmx_%s/%d', tag, j), 'boolean', etype.(f), f);
                    ord{end+1} = f; %#ok<AGROW>
                end
            end
        case 'vec'
            A(n, 'simulink/Signal Routing/Demux', ['Dmx_' tag], 'Outputs', num2str(numel(s.fields)));
            L(n, up, ['Dmx_' tag '/1'], ['vec_' tag]);
            for j = 1:numel(s.fields), src.(s.fields{j}) = sprintf('Dmx_%s/%d', tag, j); end
            ord = [ord, s.fields]; %#ok<AGROW>
        case 'zero'
            A(n, 'simulink/Sinks/Terminator', ['Rsv_' tag]);
            L(n, up, ['Rsv_' tag '/1'], ['rsv_' tag]);
        case 'tms'
            src.t_mcu = local_unwrap(n, up);
            ord{end+1} = 't_mcu'; %#ok<AGROW>
    end
end
% element 'valid' (one Bus Selector of frame.valid per consumer)
if any(strcmp(names, 'valid'))
    SEL(n, 'Sel_valid', 'frame', {'valid'});
    if isfield(src, 'valid_payload')
        A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Valid_AND', 'Operator', 'AND', 'Inputs', '2');
        L(n, src.valid_payload, 'Valid_AND/1', 'valid_payload'); L(n, 'Sel_valid/1', 'Valid_AND/2');
        src.valid = 'Valid_AND/1';
    else
        src.valid = 'Sel_valid/1';
    end
    ord{end+1} = 'valid';
end
% specials
if strcmp(S.bus, 'Bus_BmsStatus')
    A(n, 'simulink/Math Operations/Product', 'P_pack_VI', 'Inputs', '2');
    L(n, src.V_pack, 'P_pack_VI/1', 'V_pack'); L(n, src.I_pack, 'P_pack_VI/2', 'I_pack');
    src.P_pack = 'P_pack_VI/1';
    ord{end+1} = 'P_pack';
end
for k = 1:numel(names)
    if ~isfield(src, names{k})
        A(n, 'simulink/Sources/Constant', ['Zero_' names{k}], 'Value', '0', 'OutDataTypeStr', etype.(names{k}), ...
            'SampleTime', '-1');
        src.(names{k}) = ['Zero_' names{k} '/1'];
        ord{end+1} = names{k}; %#ok<AGROW>
    end
end
assert(numel(ord) == numel(names) && isempty(setxor(ord, names)), 'build_can2_codecs:decoded', ...
    '%s: the decoded fields do not cover the elements of %s', n, S.bus);
% the decoded fields as one virtual bus in slot order (spec 6.3), picked right in front of Bus_BC in the bus type's order
shaddad_bu('vbus', n, 'Decoded', [ord(:), cellfun(@(f) src.(f), ord(:), 'UniformOutput', false)]);
A(n, 'simulink/Signal Routing/Bus Selector', 'Sel_decoded', 'OutputSignals', strjoin(names, ','));
L(n, 'Decoded/1', 'Sel_decoded/1', 'decoded');
A(n, 'simulink/Signal Routing/Bus Creator', 'Bus_BC', 'Inputs', num2str(numel(names)), ...
    'OutDataTypeStr', ['Bus: ' S.bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(names), L(n, sprintf('Sel_decoded/%d', k), sprintf('Bus_BC/%d', k)); end
A(n, 'simulink/Sinks/Out1', 'x', 'Port', '1', 'OutDataTypeStr', ['Bus: ' S.bus]);
L(n, 'Bus_BC/1', 'x/1', 'x');
set_param(n, 'Description', sprintf(['%s unpack (T08, ICD 5): in 1 frame Bus_CanFrame64 -> out 1 x %s. ' ...
    'Inherited rate (unwrap / no other state, decodes every sample; data are held upstream while frames ' ...
    'are missing). Layout: %s'], n, S.bus, S.desc));
end

% ===========================================================================================
function p = local_unwrap(n, up)
% t_mcu = (wraps*2^32 + t_ms)/1000 s, wraps counted on valid frames whose t_ms decreased
A(n, 'simulink/Discrete/Unit Delay', 'Prev_tms', 'InitialCondition', '0', 'SampleTime', '-1');
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'Wrapped', 'Operator', '<');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Wrap_valid', 'Operator', 'AND', 'Inputs', '2');
A(n, 'simulink/Signal Routing/Switch', 'Hold_tms', 'Criteria', 'u2 ~= 0');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'Wrap_d', 'OutDataTypeStr', 'double');
A(n, 'simulink/Math Operations/Sum', 'Wraps_sum', 'Inputs', '++');
A(n, 'simulink/Discrete/Unit Delay', 'Prev_wraps', 'InitialCondition', '0', 'SampleTime', '-1');
A(n, 'simulink/Math Operations/Gain', 'Wrap_span', 'Gain', 'P.electronics.can.t_ms_mod');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'Tms_d', 'OutDataTypeStr', 'double');
A(n, 'simulink/Math Operations/Sum', 'Tms_sum', 'Inputs', '++');
A(n, 'simulink/Math Operations/Gain', 'Ms_to_s', 'Gain', '1e-3');
SEL(n, 'Sel_valid_wrap', 'frame', {'valid'}); SEL(n, 'Sel_valid_hold', 'frame', {'valid'});
L(n, up, 'Wrapped/1', 't_ms'); L(n, 'Prev_tms/1', 'Wrapped/2', 't_ms_prev');
L(n, 'Wrapped/1', 'Wrap_valid/1', 'wrapped'); L(n, 'Sel_valid_wrap/1', 'Wrap_valid/2');
L(n, up, 'Hold_tms/1'); L(n, 'Sel_valid_hold/1', 'Hold_tms/2'); L(n, 'Prev_tms/1', 'Hold_tms/3');
L(n, 'Hold_tms/1', 'Prev_tms/1', 't_ms_held');
L(n, 'Wrap_valid/1', 'Wrap_d/1', 'wrap'); L(n, 'Wrap_d/1', 'Wraps_sum/1', 'wrap_d');
L(n, 'Prev_wraps/1', 'Wraps_sum/2', 'wraps_prev');
L(n, 'Wraps_sum/1', 'Prev_wraps/1', 'wraps');
L(n, 'Wraps_sum/1', 'Wrap_span/1');
L(n, up, 'Tms_d/1');
L(n, 'Wrap_span/1', 'Tms_sum/1', 'wrap_ms'); L(n, 'Tms_d/1', 'Tms_sum/2', 't_ms_d');
L(n, 'Tms_sum/1', 'Ms_to_s/1', 't_ms_unwrapped');
p = 'Ms_to_s/1';
end

function p = local_cast(n, s, from, to, f)
if strcmp(from, to), p = s; return, end
A(n, 'simulink/Signal Attributes/Data Type Conversion', ['Cast_' f], 'OutDataTypeStr', to, ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
L(n, s, ['Cast_' f '/1']);
p = ['Cast_' f '/1'];
end

function T = local_etypes(B)
T = struct();
for k = 1:numel(B.Elements), T.(B.Elements(k).Name) = B.Elements(k).DataType; end
end

function b = local_size(t)
switch t
    case {'uint8', 'int8', 'boolean'}, b = 1;
    case {'uint16', 'int16'}, b = 2;
    case {'uint32', 'int32', 'single'}, b = 4;
    otherwise, b = 8;
end
end

function s = local_cellstr(c)
s = ['{' strjoin(cellfun(@(x) ['''' x ''''], c, 'UniformOutput', false), ',') '}'];
end

% ---- generic helpers ------------------------------------------------------------------------
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

function SEL(sys, blk, port, fields)
% one consumer's Bus Selector blk (spec 6.2), fed by the bus line of the root port 'port' (its line named after it)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(fields, ','));
L(sys, [port '/1'], [blk '/1'], port);
end
