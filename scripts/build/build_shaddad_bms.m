function build_shaddad_bms(what)
%BUILD_SHADDAD_BMS  BMS firmware model and BMS CAN codecs (owner: power, T04).
%
%   build_shaddad_bms()           build the four codec SSRs and models/Shaddad_BMS.slx
%   build_shaddad_bms('codecs')   only subsystems/power/{bms_can_pack,bms_can_unpack,bmsreq_can_pack,
%                                 bmsreq_can_unpack}.slx
%   build_shaddad_bms('model')    only models/Shaddad_BMS.slx (codecs must exist)
%
%   Shaddad_BMS (docs/specs/power.md 2.9, 3.9, 4.1, 5.3; build plan T04.5, ICD 3.1 BMS row, ICD 4):
%     root in  1 meas   Bus_BMS_Meas      0.01 s
%              2 can_rx Bus_Can1_McuSlow  0.05 s (slot bms_req = frame 0x3D0 BMS_REQ)
%     root out 1 cmd    Bus_BMS_Cmd       0.01 s (Unit Delay, loop breaker)
%              2 can_tx Bus_Can1_Bms      0.1 s  (frames 0x351 0x355 0x356 0x35A 0x373 0x3C0 0x3C1)
%              3 telem  Bus_BMS_Telemetry 0.1 s
%     IO_In / IO_Out: Variant Subsystems VC_TARGET (0 SIM pass-through, 1 HW stm32blockslib placeholders)
%     BMS_Core (atomic): Meas (0.01) | Protection (0.01) | Limits (0.01) | ModeManager (0.01, MATLAB
%     Function bms_mode_manager.m) | Estimation (0.1) | ChargeCtrl (0.1) | Thermal (0.1) |
%     Balancing (1 s) | FaultWord (0.01) | CmdOut (0.01) | Telemetry (0.1) | BMS_REQ decode (0.05).
%     Its units talk over virtual buses (layout spec 6.3), Out Bus Element ports with picks at the consumers (In Bus
%     Element ports, spec 6.2): m (Meas), prot (Protection), mode (ModeManager), est (Estimation), fw (FaultWord);
%     the other outputs reach at most two inputs of one unit and stay classic lines.
%   Library-block sign convention: batt_sl_lib current + = CHARGE; the buses are + = discharge, so the
%   library blocks get I_lib = -I_pack (Meas). Data type: single throughout (parameters wrapped in single()).
%
%   Codecs (ICD 4; also used by the MCU in T08): Byte Pack / Byte Unpack (byteAlign 1), Gain + Data Type
%   Conversion (Nearest, saturate), Shift Arithmetic / Bitwise Operator (pack) and Extract Bits (unpack).
%     bmsreq_can_pack   in req Bus_BmsReq (0.05)        -> out frame Bus_CanFrame8 (0x3D0)
%     bmsreq_can_unpack in frame Bus_CanFrame8 (0.05)   -> out 1 req Bus_BmsReq (held), 2 valid boolean
%     bms_can_pack      in telem Bus_BMS_Telemetry (0.1) -> out frames Bus_Can1_Bms
%     bms_can_unpack    in frames Bus_Can1_Bms (0.1)    -> out 1 status Bus_BmsStatus (held),
%                       2 frame_valid boolean[7] (limits soc vit alarm cells state power),
%                       3 extra single[9] = [V_bus_V T_batt_K SOE Fan_Cmd SOC_1pct K1_Closed K2_Closed
%                         K4_Closed Balancing_Active] (fields not carried by Bus_BmsStatus)
%     All four keep their classic root ports (the MCU, tb_mcu_closed_loop, tb_pack_bms and tb_power_plant use them), each
%     consumer with its own Bus Selector right in front of it. Inside: F_<slot> picks from its In Bus Element port telem;
%     D_<slot> picks from its In Bus Element port frame and hands out one virtual bus dec, and the seven join in the
%     virtual bus decoded, from which one Bus Selector feeds Status_BC, Valid_mux, Critical and Extra_mux.
%
%   Layout: every file saves through shaddad_bu('save'), which lays out every level of the file with shaddad_layout
%   (spec 7); builders set no positions. Every signal line is named (L5, helper L).

if nargin < 1, what = 'all'; end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
pdir = fullfile(root, 'subsystems', 'power');
if any(strcmp(what, {'all', 'codecs'}))
    local_bmsreq_pack('bmsreq_can_pack', fullfile(pdir, 'bmsreq_can_pack.slx'));
    local_bmsreq_unpack('bmsreq_can_unpack', fullfile(pdir, 'bmsreq_can_unpack.slx'));
    local_bms_pack('bms_can_pack', fullfile(pdir, 'bms_can_pack.slx'));
    local_bms_unpack('bms_can_unpack', fullfile(pdir, 'bms_can_unpack.slx'));
end
if any(strcmp(what, {'all', 'model'}))
    local_model('Shaddad_BMS', fullfile(root, 'models', 'Shaddad_BMS.slx'));
end
end

% ===========================================================================================
%  helpers
% ===========================================================================================
function p = A(sys, lib, n, varargin)
p = shaddad_bu('add', lib, [sys '/' n], varargin{:});
end
function h = L(sys, s, d, nm)
% signal into d ('Blk/k'), named nm (layout spec 6.1, L5). s 'Blk/k': a line, without nm the default of
% shaddad_bu('nline') (the name the source port's line already has, else the source block's name, '_k' for output k of
% a block with several outputs; a Bus Selector's output keeps '<element>'), h its handle. s 'port.element': a pick of
% that element of the In Bus Element port right in front of d (spec 6.2), its line named nm (default the element),
% shaddad_bu('pick'), h -1
if nargin < 4, nm = ''; end
if contains(s, '/')
    h = shaddad_bu('nline', sys, s, d, nm);
else
    shaddad_bu('pick', sys, extractBefore(s, '.'), extractAfter(s, '.'), d, nm);
    h = -1;
end
end
function OB(sys, port, el, s, nm)
% the element el of the Out Bus Element port 'port' of sys (spec 6.3), its line named nm (default el). s 'Blk/k':
% shaddad_bu('outbus'). s 'port.element', a pick handed straight on: the element's block comes first, the port's first
% block from the library, every other one copied from it (spec 13), then the pick right in front of it
if nargin < 5, nm = ''; end
if contains(s, '/')
    shaddad_bu('outbus', sys, port, el, s, nm);
    return
end
b = [sys '/' port '_' el];
first = '';
c = find_system(sys, 'SearchDepth', 1, 'BlockType', 'Outport');
for j = 1:numel(c)
    if strcmp(get_param(c{j}, 'IsBusElementPort'), 'on') && strcmp(get_param(c{j}, 'PortName'), port)
        first = c{j};
        break
    end
end
if isempty(first)
    h = add_block('simulink/Ports & Subsystems/Out Bus Element', b);
    set_param(h, 'PortName', port);
else
    h = add_block(first, b);
end
set_param(h, 'Element', el);
if isempty(nm), nm = el; end
L(sys, s, [port '_' el '/1'], nm);
end
function p = RT(sys, n, ts)
p = A(sys, 'simulink/Signal Attributes/Rate Transition', n, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTime', ts);
end
function p = DTC(sys, n, t)
p = A(sys, 'simulink/Signal Attributes/Data Type Conversion', n, 'OutDataTypeStr', t, 'RndMeth', 'Nearest', ...
    'SaturateOnIntegerOverflow', 'on');
end
function p = CST(sys, n, v, t, ts)
p = A(sys, 'simulink/Sources/Constant', n, 'Value', v, 'OutDataTypeStr', t, 'SampleTime', ts);
end
function s = SUB(sys, n, ins, outs)
% empty virtual subsystem with named ports
s = [sys '/' n];
add_block('simulink/Ports & Subsystems/Subsystem', s);
delete_line(s, 'In1/1', 'Out1/1');
delete_block([s '/In1']); delete_block([s '/Out1']);
for k = 1:numel(ins)
    add_block('simulink/Sources/In1', [s '/' ins{k}], 'Port', num2str(k));
end
for k = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [s '/' outs{k}], 'Port', num2str(k));
end
end
function k = PIN(blk, nm)
% input port number of subsystem blk: its classic Inport named nm, else its In Bus Element port named nm
k = PORTNUM(blk, nm, 'Inport');
if isempty(k), error('build_shaddad_bms:pin', '%s has no input %s', blk, nm); end
end
function k = POUT(blk, nm)
k = PORTNUM(blk, nm, 'Outport');
if isempty(k), error('build_shaddad_bms:pout', '%s has no output %s', blk, nm); end
end
function k = PORTNUM(blk, nm, kind)
k = [];
c = find_system(blk, 'SearchDepth', 1, 'BlockType', kind);
for j = 1:numel(c)
    isBus = strcmp(get_param(c{j}, 'IsBusElementPort'), 'on');
    if (isBus && strcmp(get_param(c{j}, 'PortName'), nm)) || (~isBus && strcmp(get_param(c{j}, 'Name'), nm))
        k = str2double(get_param(c{j}, 'Port'));
        return
    end
end
end
function W(sys, sblk, sport, dblk, dport, nm)
% wire subsystem output (name) -> subsystem input (name) inside sys
if nargin < 6, nm = sport; end
si = sport; di = dport;
if ischar(sport), si = POUT([sys '/' sblk], sport); end
if ischar(dport), di = PIN([sys '/' dblk], dport); end
L(sys, sprintf('%s/%d', sblk, si), sprintf('%s/%d', dblk, di), nm);
end
function BC(sys, n, bus, el)
% nonvirtual Bus Creator, inputs = {elementName, 'Blk/port' or 'port.element'} rows (lines named with the element; a
% Bus Selector's output keeps its '<element>')
A(sys, 'simulink/Signal Routing/Bus Creator', n, 'Inputs', num2str(size(el, 1)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:size(el, 1)
    L(sys, el{k, 2}, [n '/' num2str(k)], el{k, 1});
end
end
function BS(sys, n, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', n, 'OutputSignals', strjoin(sigs, ','));
end
function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line src ('Blk/k', the
% line named nm when given) and feeds its output k to dsts{k} ('Blk/j')
if nargin < 6, nm = ''; end
BS(sys, blk, sigs);
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end
function h = OUTLINE(blk, k)
% the line out of output k of blk
ph = get_param(blk, 'PortHandles');
h = get_param(ph.Outport(k), 'Line');
end
function local_log(h, nm)
% signal logging of an internal BMS signal (0.01 s; appears in logsout of the bench / system)
set_param(get_param(h, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', nm);
end
function sname = BITWORD(sys, n, bits, nbits, ts)
% boolean signals 'Blk/port' at bit positions -> uint32 word (Bit to Integer Converter, LSB first)
% bits: {bitIndex, 'Blk/port'}; unused bits false
sname = n;
% bits: {bitIndex, 'Blk/port' or 'port.element'}
CST(sys, [n '_zero'], 'false', 'boolean', ts);
A(sys, 'simulink/Signal Routing/Vector Concatenate', [n '_cat'], 'NumInputs', num2str(nbits));
for b = 0:nbits-1
    j = find(cellfun(@(x) isequal(x, b), bits(:, 1)), 1);
    if isempty(j)
        L(sys, [n '_zero/1'], sprintf('%s_cat/%d', n, b + 1));
    else
        L(sys, bits{j, 2}, sprintf('%s_cat/%d', n, b + 1));
    end
end
A(sys, 'commutil2/Bit to Integer Converter', n, 'nbits', num2str(nbits), 'bitOrder', 'LSB first', ...
    'outDtype', 'uint32');
L(sys, [n '_cat/1'], [n '/1']);
end

% ===========================================================================================
%  CAN codecs
% ===========================================================================================
function F = local_frames()
% fields: {telemetry element, offset, counts per unit, wire type, Bus_BmsStatus element ('' = extra(k))}
% counts = round((x + offset) * k);  x = counts / k - offset
F = struct('slot', {}, 'id', {}, 'fields', {});
F(end+1) = struct('slot', 'limits', 'id', hex2dec('351'), 'fields', {{ ...
    'V_chg_lim_V', 0, 10, 'uint16', 'CVL'; 'I_chg_lim_A', 0, 10, 'int16', 'CCL'; ...
    'I_dis_lim_A', 0, 10, 'int16', 'DCL'; 'V_dis_lim_V', 0, 10, 'uint16', 'DVL'}});
F(end+1) = struct('slot', 'soc', 'id', hex2dec('355'), 'fields', {{ ...
    'SOC_pack', 0, 100, 'uint16', 'x5'; 'SOH', 0, 100, 'uint16', 'SOH'; ...
    'SOC_pack', 0, 10000, 'uint16', 'SOC'; 'SOE', 0, 10000, 'uint16', 'x3'}});
F(end+1) = struct('slot', 'vit', 'id', hex2dec('356'), 'fields', {{ ...
    'V_pack_V', 0, 100, 'int16', 'V_pack'; 'I_pack_A', 0, 10, 'int16', 'I_pack'; ...
    'T_cell_max_K', -273.15, 10, 'int16', 'x2'; 'V_bus_V', 0, 100, 'uint16', 'x1'}});
F(end+1) = struct('slot', 'alarm', 'id', hex2dec('35A'), 'fields', {{ ...
    'Fault_Word', 0, 1, 'uint32', 'alarm_word'; 'Warning_Word', 0, 1, 'uint32', 'warn_word'}});
F(end+1) = struct('slot', 'cells', 'id', hex2dec('373'), 'fields', {{ ...
    'V_cell_min_V', 0, 1000, 'uint16', 'V_cell_min'; 'V_cell_max_V', 0, 1000, 'uint16', 'V_cell_max'; ...
    'T_cell_min_K', 0, 10, 'uint16', 'T_cell_min'; 'T_cell_max_K', 0, 10, 'uint16', 'T_cell_max'}});
F(end+1) = struct('slot', 'state', 'id', hex2dec('3C0'), 'fields', {{ ...
    'Contactor_State', 0, 1, 'uint8', 'contactor_state'; ...
    'FLAGS', 0, 1, 'uint8', ''; ...
    'Fault_Level', 0, 1, 'uint8', 'fault_level'; 'AliveCounter', 0, 1, 'uint8', 'alive'; ...
    'Runtime_Remaining_s', 0, 1/60, 'uint16', 'Runtime_s'; 'E_remaining_Wh', 0, 0.1, 'uint16', 'E_remaining_Wh'}});
F(end+1) = struct('slot', 'power', 'id', hex2dec('3C1'), 'fields', {{ ...
    'P_dis_lim_W', 0, 0.1, 'uint16', 'P_dis_lim'; 'P_chg_lim_W', 0, 0.1, 'uint16', 'P_chg_lim'; ...
    'P_pack_W', 0, 0.1, 'int16', 'P_pack'; 'Fan_Cmd', 0, 100, 'uint8', 'x4'; 'RESERVED', 0, 1, 'uint8', ''}});
end

function fl = local_state_flags()
% BMS_STATE byte 1: {bit, telemetry element, status element ('' = extra(k))}
fl = {0, 'HV_Ready', 'hv_ready'; 1, 'HV_Opening_Warn', 'hv_opening_warn'; 2, 'K1_Closed', 'x6'; ...
      3, 'K2_Closed', 'x7'; 4, 'K4_Closed', 'x8'; 5, 'Charging_Active', 'charging'; ...
      6, 'Charge_Complete', 'charge_complete'; 7, 'Balancing_Active', 'x9'};
end

function s = local_numstr(v)
s = sprintf('%.15g', v);
end

% ---------------- bmsreq_can_pack ----------------
function local_bmsreq_pack(name, f)
shaddad_bu('fresh', name, f, 'ssr');
ts = 'Ts.can_slow';
A(name, 'simulink/Sources/In1', 'req', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BmsReq', 'SampleTime', ts);
A(name, 'simulink/Sinks/Out1', 'frame', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_CanFrame8');
sig = {'HV_Enable_Req', 'Charge_Enable_Req', 'Fault_Reset_Req', 'Shutdown_Req'};
A(name, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Flags_OR', 'logicop', 'OR', ...
    'UseBitMask', 'off', 'NumInputPorts', '4');
for b = 0:3
    DTC(name, sprintf('b%d_u8', b), 'uint8');
    A(name, 'simulink/Logic and Bit Operations/Shift Arithmetic', sprintf('b%d_shift', b), ...
        'BitShiftNumberSource', 'Dialog', 'BitShiftDirection', 'Left', 'BitShiftNumber', num2str(b));
    PICK(name, 'req/1', ['Sel_' sig{b + 1}], sig(b + 1), {sprintf('b%d_u8/1', b)}, 'req');
    L(name, sprintf('b%d_u8/1', b), sprintf('b%d_shift/1', b));
    L(name, sprintf('b%d_shift/1', b), sprintf('Flags_OR/%d', b + 1));
end
CST(name, 'Reserved', 'zeros(6,1)', 'uint8', ts);
A(name, 'mcblib/Signal Management/Byte Pack', 'Pack', 'datatypes', '{''uint8'',''uint8'',''uint8''}', 'byteAlign', '1');
L(name, 'Flags_OR/1', 'Pack/1', 'flags');
PICK(name, 'req/1', 'Sel_AliveCounter', {'AliveCounter'}, {'Pack/2'}, 'req');
L(name, 'Reserved/1', 'Pack/3', 'reserved');
CST(name, 'ID', 'uint32(hex2dec(''3D0''))', 'uint32', ts);
CST(name, 'LEN', '8', 'uint8', ts);
CST(name, 'VALID', 'true', 'boolean', ts);
BC(name, 'Frame_BC', 'Bus_CanFrame8', {'id', 'ID/1'; 'len', 'LEN/1'; 'data', 'Pack/1'; 'valid', 'VALID/1'});
L(name, 'Frame_BC/1', 'frame/1', 'frame');
set_param(name, 'Description', ['bmsreq_can_pack (T04, ICD 4 0x3D0 BMS_REQ, 50 ms): byte 0 flags b0 HV_Enable_Req, ' ...
    'b1 Charge_Enable_Req, b2 Fault_Reset_Req, b3 Shutdown_Req; byte 1 alive; bytes 2-7 reserved 0. ' ...
    'in req Bus_BmsReq (0.05 s) -> out frame Bus_CanFrame8 (valid = true every sample).']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ---------------- bmsreq_can_unpack ----------------
function local_bmsreq_unpack(name, f)
shaddad_bu('fresh', name, f, 'ssr');
ts = 'Ts.can_slow';
A(name, 'simulink/Sources/In1', 'frame', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', 'SampleTime', ts);
A(name, 'simulink/Sinks/Out1', 'req', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BmsReq');
A(name, 'simulink/Sinks/Out1', 'valid', 'Port', '2', 'OutDataTypeStr', 'boolean');
A(name, 'simulink/Logic and Bit Operations/Compare To Constant', 'ID_match', 'relop', '==', ...
    'const', 'hex2dec(''3D0'')', 'OutDataTypeStr', 'boolean');
A(name, 'simulink/Logic and Bit Operations/Logical Operator', 'Frame_ok', 'Operator', 'AND', 'Inputs', '2');
PICK(name, 'frame/1', 'Sel_id', {'id'}, {'ID_match/1'}, 'frame');
L(name, 'ID_match/1', 'Frame_ok/1', 'id_match');
PICK(name, 'frame/1', 'Sel_valid', {'valid'}, {'Frame_ok/2'}, 'frame');
% decode in an Enabled Subsystem: outputs held while no fresh frame
d = [name '/Decode'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', d);
delete_line(d, 'In1/1', 'Out1/1'); delete_block([d '/In1']); delete_block([d '/Out1']);
A(d, 'simulink/Sources/In1', 'data', 'Port', '1');
A(d, 'simulink/Sinks/Out1', 'req', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BmsReq', ...
    'OutputWhenDisabled', 'held', 'InitialOutput', 'P.ic.Bus_BmsReq');
A(d, 'mcblib/Signal Management/Byte Unpack', 'Unpack', 'dimensions', '{1,1,6}', ...
    'datatypes', '{''uint8'',''uint8'',''uint8''}', 'byteAlign', '1');
L(d, 'data/1', 'Unpack/1', 'data');
A(d, 'simulink/Sinks/Terminator', 'Reserved');
L(d, 'Unpack/3', 'Reserved/1', 'reserved');
nm = {'HV_Enable_Req', 'Charge_Enable_Req', 'Fault_Reset_Req', 'Shutdown_Req'};
for b = 0:3
    A(d, 'simulink/Logic and Bit Operations/Extract Bits', sprintf('bit%d', b), 'bitsToExtract', 'Range of bits', ...
        'bitIdxRange', sprintf('[%d %d]', b, b), 'outScalingMode', 'Treat bit field as an integer');
    A(d, 'simulink/Signal Attributes/Data Type Conversion', [nm{b + 1} '_b'], 'OutDataTypeStr', 'boolean');
    L(d, 'Unpack/1', sprintf('bit%d/1', b), 'flags');
    L(d, sprintf('bit%d/1', b), [nm{b + 1} '_b/1']);
end
BC(d, 'Req_BC', 'Bus_BmsReq', {'AliveCounter', 'Unpack/2'; 'HV_Enable_Req', 'HV_Enable_Req_b/1'; ...
    'Charge_Enable_Req', 'Charge_Enable_Req_b/1'; 'Fault_Reset_Req', 'Fault_Reset_Req_b/1'; 'Shutdown_Req', 'Shutdown_Req_b/1'});
L(d, 'Req_BC/1', 'req/1', 'req');
PICK(name, 'frame/1', 'Sel_data', {'data'}, {'Decode/1'}, 'frame');
L(name, 'Frame_ok/1', 'Decode/Enable', 'frame_ok');
L(name, 'Decode/1', 'req/1', 'req');
L(name, 'Frame_ok/1', 'valid/1', 'frame_ok');
set_param(name, 'Description', ['bmsreq_can_unpack (T04, ICD 4 0x3D0): in frame Bus_CanFrame8 (0.05 s) -> out 1 req ' ...
    'Bus_BmsReq (held while no valid 0x3D0 frame; IC P.ic.Bus_BmsReq), 2 valid (fresh frame this sample).']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ---------------- bms_can_pack ----------------
function local_bms_pack(name, f)
shaddad_bu('fresh', name, f, 'ssr');
ts = 'Ts.hk';
A(name, 'simulink/Sources/In1', 'telem', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Telemetry', 'SampleTime', ts);
A(name, 'simulink/Sinks/Out1', 'frames', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms');
F = local_frames();
fl = local_state_flags();
for j = 1:numel(F)
    s = SUB(name, ['F_' F(j).slot], {}, {'frame'});            % input: the In Bus Element port telem, its picks below
    set_param([s '/frame'], 'OutDataTypeStr', 'Bus: Bus_CanFrame8');
    fd = F(j).fields;
    dts = cell(1, size(fd, 1));
    for k = 1:size(fd, 1)
        tag = sprintf('f%d', k);
        dts{k} = fd{k, 4};
        switch fd{k, 1}
            case 'FLAGS'
                A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', tag, 'logicop', 'OR', ...
                    'UseBitMask', 'off', 'NumInputPorts', num2str(size(fl, 1)));
                for b = 1:size(fl, 1)
                    DTC(s, sprintf('%s_b%d', tag, fl{b, 1}), 'uint8');
                    A(s, 'simulink/Logic and Bit Operations/Shift Arithmetic', sprintf('%s_s%d', tag, fl{b, 1}), ...
                        'BitShiftNumberSource', 'Dialog', 'BitShiftDirection', 'Left', 'BitShiftNumber', num2str(fl{b, 1}));
                    L(s, ['telem.' fl{b, 2}], sprintf('%s_b%d/1', tag, fl{b, 1}), fl{b, 2});
                    L(s, sprintf('%s_b%d/1', tag, fl{b, 1}), sprintf('%s_s%d/1', tag, fl{b, 1}));
                    L(s, sprintf('%s_s%d/1', tag, fl{b, 1}), sprintf('%s/%d', tag, b));
                end
            case 'RESERVED'
                CST(s, tag, '0', 'uint8', ts);
            otherwise
                src = ['telem.' fd{k, 1}];
                if startsWith(fd{k, 4}, 'uint32') || (strcmp(fd{k, 4}, 'uint8') && fd{k, 3} == 1 && fd{k, 2} == 0)
                    DTC(s, tag, fd{k, 4});                         % integer field: no scaling
                    L(s, src, [tag '/1'], fd{k, 1});
                else
                    last = src;
                    if fd{k, 2} ~= 0
                        A(s, 'simulink/Math Operations/Bias', [tag '_off'], 'Bias', ['single(' local_numstr(fd{k, 2}) ')']);
                        L(s, last, [tag '_off/1'], fd{k, 1}); last = [tag '_off/1'];
                    end
                    A(s, 'simulink/Math Operations/Gain', [tag '_k'], 'Gain', ['single(' local_numstr(fd{k, 3}) ')']);
                    if strcmp(last, src), L(s, last, [tag '_k/1'], fd{k, 1}); else, L(s, last, [tag '_k/1']); end
                    % explicit range limit in counts (no integer-overflow diagnostic at the conversion)
                    A(s, 'simulink/Discontinuities/Saturation', [tag '_sat'], ...
                        'UpperLimit', sprintf('single(intmax(''%s''))', fd{k, 4}), ...
                        'LowerLimit', sprintf('single(intmin(''%s''))', fd{k, 4}));
                    L(s, [tag '_k/1'], [tag '_sat/1']);
                    DTC(s, tag, fd{k, 4});
                    L(s, [tag '_sat/1'], [tag '/1']);
                end
        end
    end
    A(s, 'mcblib/Signal Management/Byte Pack', 'Pack', 'datatypes', ['{' strjoin(strcat('''', dts, ''''), ',') '}'], ...
        'byteAlign', '1');
    for k = 1:size(fd, 1)
        L(s, sprintf('f%d/1', k), sprintf('Pack/%d', k));
    end
    CST(s, 'ID', sprintf('uint32(%d)', F(j).id), 'uint32', ts);
    CST(s, 'LEN', '8', 'uint8', ts);
    CST(s, 'VALID', 'true', 'boolean', ts);
    BC(s, 'Frame_BC', 'Bus_CanFrame8', {'id', 'ID/1'; 'len', 'LEN/1'; 'data', 'Pack/1'; 'valid', 'VALID/1'});
    L(s, 'Frame_BC/1', 'frame/1', 'frame');
    L(name, 'telem/1', sprintf('F_%s/1', F(j).slot), 'telem');
end
el = cell(numel(F), 2);
for j = 1:numel(F), el(j, :) = {F(j).slot, sprintf('F_%s/1', F(j).slot)}; end
BC(name, 'Frames_BC', 'Bus_Can1_Bms', el);
L(name, 'Frames_BC/1', 'frames/1', 'frames');
set_param(name, 'Description', ['bms_can_pack (T04, ICD 4, 100 ms): Bus_BMS_Telemetry -> Bus_Can1_Bms 0x351 LIMITS ' ...
    '(CVL, CCL, DCL, DVL), 0x355 SOC, 0x356 VIT (T_batt = T_cell_max), 0x35A ALARM, 0x373 CELLS, 0x3C0 STATE, ' ...
    '0x3C1 POWER; little-endian Byte Pack, round to nearest, saturate.']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ---------------- bms_can_unpack ----------------
function local_bms_unpack(name, f)
shaddad_bu('fresh', name, f, 'ssr');
ts = 'Ts.hk';
A(name, 'simulink/Sources/In1', 'frames', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms', 'SampleTime', ts);
A(name, 'simulink/Sinks/Out1', 'status', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BmsStatus');
A(name, 'simulink/Sinks/Out1', 'frame_valid', 'Port', '2', 'OutDataTypeStr', 'boolean');
A(name, 'simulink/Sinks/Out1', 'extra', 'Port', '3', 'OutDataTypeStr', 'single');
F = local_frames();
fl = local_state_flags();
defs = shaddad_buses('defs');
sd = defs(strcmp({defs.name}, 'Bus_BmsStatus'));
stype = containers.Map(cellfun(@(e) e.name, sd.elems, 'UniformOutput', false), ...
                       cellfun(@(e) e.dtype, sd.elems, 'UniformOutput', false));
src = containers.Map();          % status / extra element -> its path in the virtual bus decoded, <slot>.<element>
for j = 1:numel(F)
    fd = F(j).fields;
    outs = {};
    for k = 1:size(fd, 1)
        if strcmp(fd{k, 1}, 'FLAGS')
            outs = [outs, fl(:, 3)']; %#ok<AGROW>
        elseif ~isempty(fd{k, 5})
            outs{end+1} = fd{k, 5}; %#ok<AGROW>
        end
    end
    s = SUB(name, ['D_' F(j).slot], {}, {});      % the In Bus Element port frame, the Out Bus Element port dec (ok, outs)
    A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'ID_match', 'relop', '==', ...
        'const', sprintf('%d', F(j).id), 'OutDataTypeStr', 'boolean');
    A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Frame_ok', 'Operator', 'AND', 'Inputs', '2');
    L(s, 'frame.id', 'ID_match/1', 'id'); L(s, 'ID_match/1', 'Frame_ok/1', 'id_match'); L(s, 'frame.valid', 'Frame_ok/2', 'valid');
    OB(s, 'dec', 'ok', 'Frame_ok/1', 'frame_ok');
    d = [s '/Decode'];
    add_block('simulink/Ports & Subsystems/Enabled Subsystem', d);
    delete_line(d, 'In1/1', 'Out1/1'); delete_block([d '/In1']); delete_block([d '/Out1']);
    A(d, 'simulink/Sources/In1', 'data', 'Port', '1');
    dts = fd(:, 4)';
    dims = repmat({'1'}, 1, size(fd, 1));
    A(d, 'mcblib/Signal Management/Byte Unpack', 'Unpack', 'dimensions', ['{' strjoin(dims, ',') '}'], ...
        'datatypes', ['{' strjoin(strcat('''', dts, ''''), ',') '}'], 'byteAlign', '1');
    L(d, 'data/1', 'Unpack/1', 'data');
    po = 0;
    for k = 1:size(fd, 1)
        tag = sprintf('f%d', k);
        switch fd{k, 1}
            case 'RESERVED'
                A(d, 'simulink/Sinks/Terminator', [tag '_res']);
                L(d, sprintf('Unpack/%d', k), [tag '_res/1']);
            case 'FLAGS'
                for b = 1:size(fl, 1)
                    po = po + 1;
                    el = fl{b, 3};
                    A(d, 'simulink/Logic and Bit Operations/Extract Bits', sprintf('%s_x%d', tag, fl{b, 1}), ...
                        'bitsToExtract', 'Range of bits', 'bitIdxRange', sprintf('[%d %d]', fl{b, 1}, fl{b, 1}), ...
                        'outScalingMode', 'Treat bit field as an integer');
                    if startsWith(el, 'x'), t = 'single'; else, t = 'boolean'; end
                    A(d, 'simulink/Signal Attributes/Data Type Conversion', sprintf('%s_c%d', tag, fl{b, 1}), 'OutDataTypeStr', t);
                    A(d, 'simulink/Sinks/Out1', el, 'Port', num2str(po), 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
                    L(d, sprintf('Unpack/%d', k), sprintf('%s_x%d/1', tag, fl{b, 1}), 'flags');
                    L(d, sprintf('%s_x%d/1', tag, fl{b, 1}), sprintf('%s_c%d/1', tag, fl{b, 1}));
                    L(d, sprintf('%s_c%d/1', tag, fl{b, 1}), [el '/1'], el);
                end
            otherwise
                if isempty(fd{k, 5}), continue, end
                po = po + 1;
                el = fd{k, 5};
                if startsWith(el, 'x'), t = 'single'; else, t = stype(el); end
                A(d, 'simulink/Sinks/Out1', el, 'Port', num2str(po), 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
                if ~strcmp(t, 'single')
                    A(d, 'simulink/Signal Attributes/Data Type Conversion', tag, 'OutDataTypeStr', t);
                    L(d, sprintf('Unpack/%d', k), [tag '/1']);
                    L(d, [tag '/1'], [el '/1'], el);
                else
                    A(d, 'simulink/Signal Attributes/Data Type Conversion', [tag '_s'], 'OutDataTypeStr', 'single');
                    A(d, 'simulink/Math Operations/Gain', [tag '_k'], 'Gain', ['single(' local_numstr(1/fd{k, 3}) ')']);
                    L(d, sprintf('Unpack/%d', k), [tag '_s/1']);
                    L(d, [tag '_s/1'], [tag '_k/1']);
                    last = [tag '_k/1'];
                    if fd{k, 2} ~= 0
                        A(d, 'simulink/Math Operations/Bias', [tag '_off'], 'Bias', ['single(' local_numstr(-fd{k, 2}) ')']);
                        L(d, last, [tag '_off/1']); last = [tag '_off/1'];
                    end
                    L(d, last, [el '/1'], el);
                end
        end
    end
    L(s, 'frame.data', 'Decode/1', 'data');
    L(s, 'Frame_ok/1', 'Decode/Enable', 'frame_ok');
    for k = 1:numel(outs)
        OB(s, 'dec', outs{k}, sprintf('Decode/%d', k), outs{k});
    end
    PICK(name, 'frames/1', ['Sel_' F(j).slot], {F(j).slot}, {sprintf('D_%s/1', F(j).slot)}, 'frames');
    for k = 1:numel(outs)
        src(outs{k}) = [F(j).slot '.' outs{k}];
    end
end
% the seven decoded frames as one virtual bus (spec 6.3); one Bus Selector right in front of each consumer (spec 6.2)
el = cell(numel(F), 2);
for j = 1:numel(F), el(j, :) = {F(j).slot, sprintf('D_%s/1', F(j).slot)}; end
shaddad_bu('vbus', name, 'Decoded', el);
% all frames fresh + fault_level == 3
A(name, 'simulink/Signal Routing/Mux', 'Valid_mux', 'Inputs', num2str(numel(F)));
A(name, 'simulink/Logic and Bit Operations/Logical Operator', 'All_valid', 'Operator', 'AND', 'Inputs', '1');
dst = arrayfun(@(j) sprintf('Valid_mux/%d', j), 1:numel(F), 'UniformOutput', false);
PICK(name, 'Decoded/1', 'Sel_valid', strcat({F.slot}, '.ok'), dst, 'decoded');
L(name, 'Valid_mux/1', 'All_valid/1', 'frame_valid');
L(name, 'Valid_mux/1', 'frame_valid/1', 'frame_valid');
A(name, 'simulink/Logic and Bit Operations/Compare To Constant', 'Critical', 'relop', '==', 'const', '3', ...
    'OutDataTypeStr', 'boolean');
PICK(name, 'Decoded/1', 'Sel_critical', {src('fault_level')}, {'Critical/1'}, 'decoded');
names = cellfun(@(e) e.name, sd.elems, 'UniformOutput', false);
sel = setdiff(names, {'bms_critical', 'valid'}, 'stable');
el = cell(numel(names), 2);
for k = 1:numel(names)
    switch names{k}
        case 'bms_critical', p = 'Critical/1';
        case 'valid',        p = 'All_valid/1';
        otherwise,           p = sprintf('Sel_status/%d', find(strcmp(sel, names{k})));
    end
    el(k, :) = {names{k}, p};
end
BS(name, 'Sel_status', cellfun(@(x) src(x), sel, 'UniformOutput', false));
L(name, 'Decoded/1', 'Sel_status/1', 'decoded');
BC(name, 'Status_BC', 'Bus_BmsStatus', el);
L(name, 'Status_BC/1', 'status/1', 'status');
A(name, 'simulink/Signal Routing/Mux', 'Extra_mux', 'Inputs', '9');
xs = arrayfun(@(k) src(sprintf('x%d', k)), 1:9, 'UniformOutput', false);
dst = arrayfun(@(k) sprintf('Extra_mux/%d', k), 1:9, 'UniformOutput', false);
PICK(name, 'Decoded/1', 'Sel_extra', xs, dst, 'decoded');
L(name, 'Extra_mux/1', 'extra/1', 'extra');
set_param(name, 'Description', ['bms_can_unpack (T04, ICD 4, 100 ms): Bus_Can1_Bms -> 1 status Bus_BmsStatus (decoded ' ...
    'fields held while a frame is missing; valid = all 7 frames fresh in this sample; bms_critical = Fault_Level == 3), ' ...
    '2 frame_valid boolean[7] (limits soc vit alarm cells state power), 3 extra single[9] = [V_bus_V T_batt_K SOE ' ...
    'Fan_Cmd SOC_1pct K1_Closed K2_Closed K4_Closed Balancing_Active].']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ===========================================================================================
%  Shaddad_BMS model
% ===========================================================================================
function local_model(mdl, f)
for m = {'tb_pack_bms'}
    if bdIsLoaded(m{1}), close_system(m{1}, 0); end
end
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
A(mdl, 'simulink/Sources/In1', 'meas', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Meas', ...
    'SampleTime', 'Ts.bms_fast', 'BusOutputAsStruct', 'on');
A(mdl, 'simulink/Sources/In1', 'can_rx', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_McuSlow', ...
    'SampleTime', 'Ts.can_slow', 'BusOutputAsStruct', 'on');
A(mdl, 'simulink/Sinks/Out1', 'cmd', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Cmd', ...
    'SampleTime', 'Ts.bms_fast', 'BusOutputAsStruct', 'on');
A(mdl, 'simulink/Sinks/Out1', 'can_tx', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms', ...
    'SampleTime', 'Ts.hk', 'BusOutputAsStruct', 'on');
A(mdl, 'simulink/Sinks/Out1', 'telem', 'Port', '3', 'OutDataTypeStr', 'Bus: Bus_BMS_Telemetry', ...
    'SampleTime', 'Ts.hk', 'BusOutputAsStruct', 'on');

local_io_in([mdl '/IO_In']);
core = [mdl '/BMS_Core'];
add_block('simulink/Ports & Subsystems/Subsystem', core);
delete_line(core, 'In1/1', 'Out1/1'); delete_block([core '/In1']); delete_block([core '/Out1']);
set_param(core, 'TreatAsAtomicUnit', 'on');
local_core(core);
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'BMS_CAN_Pack', 'ReferencedSubsystem', 'bms_can_pack');
local_io_out([mdl '/IO_Out']);

L(mdl, 'meas/1', 'IO_In/1', 'meas');
L(mdl, 'can_rx/1', 'IO_In/2', 'can_rx');
L(mdl, 'IO_In/1', 'BMS_Core/1', 'meas_io');
L(mdl, 'IO_In/2', 'BMS_Core/2', 'bms_req_frame');
L(mdl, 'BMS_Core/1', 'IO_Out/1', 'cmd_core');
L(mdl, 'BMS_Core/2', 'BMS_CAN_Pack/1', 'telem');
L(mdl, 'BMS_CAN_Pack/1', 'IO_Out/2', 'frames');
L(mdl, 'IO_Out/1', 'cmd/1', 'cmd');
L(mdl, 'IO_Out/2', 'can_tx/1', 'can_tx');
L(mdl, 'BMS_Core/2', 'telem/1', 'telem');
set_param(mdl, 'Description', ['Shaddad_BMS (T04): BMS ECU firmware (STM32G4 class, single precision). ' ...
    'in 1 meas Bus_BMS_Meas 0.01 s, 2 can_rx Bus_Can1_McuSlow 0.05 s (bms_req 0x3D0); out 1 cmd Bus_BMS_Cmd 0.01 s ' ...
    '(Unit Delay), 2 can_tx Bus_Can1_Bms 0.1 s, 3 telem Bus_BMS_Telemetry 0.1 s. IO_In / IO_Out Variant VC_TARGET ' ...
    '(0 SIM pass-through, 1 HW stm32blockslib). See scripts/build/build_shaddad_bms.m.']);
shaddad_bu('save', mdl, f);
close_system(mdl, 0);
end

% ---------------- IO_In / IO_Out variants ----------------
function local_io_in(s)
add_block('simulink/Ports & Subsystems/Variant Subsystem', s);
ch = find_system(s, 'MatchFilter', @Simulink.match.allVariants, 'SearchDepth', 1);
ch = setdiff(ch, {s});
for k = 1:numel(ch), delete_block(ch{k}); end
add_block('simulink/Sources/In1', [s '/meas'], 'Port', '1');
add_block('simulink/Sources/In1', [s '/can_rx'], 'Port', '2');
add_block('simulink/Sinks/Out1', [s '/meas_io'], 'Port', '1');
add_block('simulink/Sinks/Out1', [s '/bms_req_frame'], 'Port', '2');
% ---- SIM: pass-through (bms_req slot of CAN1 slow) ----
c = SUB(s, 'SIM', {'meas', 'can_rx'}, {'meas_io', 'bms_req_frame'});
set_param([c '/meas'], 'OutDataTypeStr', 'Bus: Bus_BMS_Meas');
set_param([c '/can_rx'], 'OutDataTypeStr', 'Bus: Bus_Can1_McuSlow');
BS(c, 'Sel_req', {'bms_req'});
L(c, 'meas/1', 'meas_io/1', 'meas');
L(c, 'can_rx/1', 'Sel_req/1', 'can_rx');
L(c, 'Sel_req/1', 'bms_req_frame/1', 'bms_req');
set_param(c, 'VariantControl', 'VC_TARGET == 0');
% ---- HW: STM32G4 peripherals (placeholders until T17 configures the board) ----
c = SUB(s, 'HW', {'meas', 'can_rx'}, {'meas_io', 'bms_req_frame'});
A(c, 'simulink/Sinks/Terminator', 'T_meas'); A(c, 'simulink/Sinks/Terminator', 'T_can');
L(c, 'meas/1', 'T_meas/1'); L(c, 'can_rx/1', 'T_can/1');
A(c, 'stm32blockslib/SPI Controller Transfer', 'AFE_SPI');
CST(c, 'AFE_cmd', 'zeros(1,P.power.bms.hw.afe_rx_len)', 'uint8', 'Ts.bms_fast');
L(c, 'AFE_cmd/1', 'AFE_SPI/1');
% afe_decode placeholder (block-based register decode; PEC check deferred to T17, spec 4.2)
d = SUB(c, 'afe_decode', {'rx'}, {'V_cell', 'T_ntc'});
A(d, 'simulink/Signal Attributes/Data Type Conversion', 'rx_u8', 'OutDataTypeStr', 'uint8');
Ns = evalin('base', 'P.power.pack.Ns');
A(d, 'mcblib/Signal Management/Byte Unpack', 'Regs', 'dimensions', sprintf('{%d,%d}', Ns, Ns), ...
    'datatypes', '{''uint16'',''uint16''}', 'byteAlign', '1');
A(d, 'simulink/Signal Attributes/Data Type Conversion', 'V_s', 'OutDataTypeStr', 'single');
A(d, 'simulink/Math Operations/Gain', 'V_lsb', 'Gain', 'single(P.power.bms.hw.afe_lsb_V)');
A(d, 'simulink/Signal Attributes/Data Type Conversion', 'N_s', 'OutDataTypeStr', 'single');
A(d, 'simulink/Lookup Tables/1-D Lookup Table', 'NTC_lut', 'BreakpointsForDimension1', ...
    'single(P.power.bms.hw.ntc_counts)', 'Table', 'single(P.power.bms.hw.ntc_K)', 'ExtrapMethod', 'Clip');
L(d, 'rx/1', 'rx_u8/1'); L(d, 'rx_u8/1', 'Regs/1');
L(d, 'Regs/1', 'V_s/1'); L(d, 'V_s/1', 'V_lsb/1'); L(d, 'V_lsb/1', 'V_cell/1', 'V_cell');
L(d, 'Regs/2', 'N_s/1'); L(d, 'N_s/1', 'NTC_lut/1'); L(d, 'NTC_lut/1', 'T_ntc/1', 'T_ntc');
L(c, 'AFE_SPI/1', 'afe_decode/1', 'afe_rx');
A(c, 'stm32blockslib/Analog to Digital Converter', 'ADC');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'ADC_s', 'OutDataTypeStr', 'single');
A(c, 'simulink/Math Operations/Gain', 'ADC_scale', 'Gain', 'single(P.power.bms.hw.adc_gain)', 'Multiplication', 'Element-wise(K.*u)');
A(c, 'simulink/Math Operations/Bias', 'ADC_offs', 'Bias', 'single(P.power.bms.hw.adc_offset)');
A(c, 'simulink/Signal Routing/Demux', 'ADC_dmx', 'Outputs', '6');
L(c, 'ADC/1', 'ADC_s/1'); L(c, 'ADC_s/1', 'ADC_scale/1'); L(c, 'ADC_scale/1', 'ADC_offs/1'); L(c, 'ADC_offs/1', 'ADC_dmx/1');
% GPIO inputs: Digital Port Read with IOAsArray returns one boolean per pin (not a port word). Pin map of
% the CubeMX project utilities/stm32/shaddad_g474.ioc (T17, stm32_ioc_configure('pinmap','g474')):
% GPIOC[4:11] = K1_FB K2_FB K4_FB DOCK_PRESENT SAFETY_OK SAFETY_HV_OK KEY_ON FUSE_MAIN_OK (Bus_BMS_Meas order).
nDI = evalin('base', 'P.electronics.target.bms.di_n');
assert(nDI == 8, 'build_shaddad_bms:di', 'Bus_BMS_Meas carries 8 GPIO inputs (P.electronics.target.bms.di_n = %d)', nDI);
A(c, 'stm32blockslib/Digital Port Read', 'DI', 'PortName', 'GPIOC', 'PinNumber', sprintf('[4:%d]', 3 + nDI), ...
    'IOAsArray', 'on', 'SampleTime', 'Ts.bms_fast');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'DI_bool', 'OutDataTypeStr', 'boolean');
A(c, 'simulink/Signal Routing/Demux', 'DI_dmx', 'Outputs', num2str(nDI));
L(c, 'DI/1', 'DI_bool/1'); L(c, 'DI_bool/1', 'DI_dmx/1');
CST(c, 'Fuse_none', '0', 'uint8', 'Ts.bms_fast');
BC(c, 'Meas_BC', 'Bus_BMS_Meas', {'V_cell_V', 'afe_decode/1'; 'T_cell_K', 'afe_decode/2'; 'I_pack_A', 'ADC_dmx/1'; ...
    'V_pack_V', 'ADC_dmx/2'; 'V_bus_V', 'ADC_dmx/3'; 'V_dock_V', 'ADC_dmx/4'; 'T_amb_K', 'ADC_dmx/5'; 'T_air_K', 'ADC_dmx/6'; ...
    'K1_fb', 'DI_dmx/1'; 'K2_fb', 'DI_dmx/2'; 'K4_fb', 'DI_dmx/3'; 'Dock_Present', 'DI_dmx/4'; 'Safety_OK', 'DI_dmx/5'; ...
    'Safety_HV_OK', 'DI_dmx/6'; 'Key_On', 'DI_dmx/7'; 'FuseMain_OK', 'DI_dmx/8'; 'Fuse_Blown', 'Fuse_none/1'});
L(c, 'Meas_BC/1', 'meas_io/1', 'meas');
% FDCAN Read (Receive, Unpacked) -> frame slot 0x3D0 BMS_REQ. Outputs (stm32cube.blocks.FDCANBlock, verified
% with getOutputNames): 1 Data uint8 [64x1] (always the CAN-FD maximum: Data length / frame type are not
% receive parameters), 2 Length uint8, 3 Id uint32. A classic frame uses bytes 1:8 (Reshape + Selector).
A(c, 'stm32blockslib/FDCAN Read', 'FDCAN1_Read', 'FDCANModule', 'FDCAN1', 'BlockMode', 'Receive', ...
    'OperationReadMode', 'Data', 'OutputTypeReadMode', 'Unpacked', 'SampleTime', 'Ts.can_slow');
A(c, 'simulink/Math Operations/Reshape', 'rx_vec', 'OutputDimensionality', '1-D array');
A(c, 'simulink/Signal Routing/Selector', 'rx_data8', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1:8', 'InputPortWidth', '-1');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'rx_data', 'OutDataTypeStr', 'uint8');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'rx_len', 'OutDataTypeStr', 'uint8');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'rx_id', 'OutDataTypeStr', 'uint32');
L(c, 'FDCAN1_Read/1', 'rx_vec/1'); L(c, 'rx_vec/1', 'rx_data8/1'); L(c, 'rx_data8/1', 'rx_data/1');
L(c, 'FDCAN1_Read/2', 'rx_len/1'); L(c, 'FDCAN1_Read/3', 'rx_id/1');
% valid = the FIFO frame carries the BMS_REQ identifier (other IDs accepted by the filter are ignored)
A(c, 'simulink/Logic and Bit Operations/Compare To Constant', 'rx_valid', 'relop', '==', ...
    'const', 'P.electronics.can.id.bms_req', 'OutDataTypeStr', 'boolean');
L(c, 'rx_id/1', 'rx_valid/1');
BC(c, 'Frame_BC', 'Bus_CanFrame8', {'id', 'rx_id/1'; 'len', 'rx_len/1'; 'data', 'rx_data/1'; 'valid', 'rx_valid/1'});
L(c, 'Frame_BC/1', 'bms_req_frame/1', 'bms_req');
set_param(c, 'VariantControl', 'VC_TARGET == 1');
set_param(c, 'Description', ['HW IO_In (VC_TARGET == 1): SPI AFE + afe_decode (block-based register decode), ADC1 6 channels, ' ...
    'GPIO aux contacts (Digital Port Read GPIOC[4:11], one boolean per pin -> Demux), FDCAN1 read of 0x3D0 ' ...
    '(Data [64x1] -> bytes 1:8, Length, Id; valid = Id == BMS_REQ). Peripheral settings / pin maps are configured ' ...
    'by T17 (CubeMX, build_targets(''peripherals'')).']);
set_param(s, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

function local_io_out(s)
add_block('simulink/Ports & Subsystems/Variant Subsystem', s);
ch = find_system(s, 'MatchFilter', @Simulink.match.allVariants, 'SearchDepth', 1);
ch = setdiff(ch, {s});
for k = 1:numel(ch), delete_block(ch{k}); end
add_block('simulink/Sources/In1', [s '/cmd_core'], 'Port', '1');
add_block('simulink/Sources/In1', [s '/frames'], 'Port', '2');
add_block('simulink/Sinks/Out1', [s '/cmd'], 'Port', '1');
add_block('simulink/Sinks/Out1', [s '/can_tx'], 'Port', '2');
c = SUB(s, 'SIM', {'cmd_core', 'frames'}, {'cmd', 'can_tx'});
L(c, 'cmd_core/1', 'cmd/1', 'cmd'); L(c, 'frames/1', 'can_tx/1', 'can_tx');
set_param(c, 'VariantControl', 'VC_TARGET == 0');
c = SUB(s, 'HW', {'cmd_core', 'frames'}, {'cmd', 'can_tx'});
set_param([c '/cmd_core'], 'OutDataTypeStr', 'Bus: Bus_BMS_Cmd');
set_param([c '/frames'], 'OutDataTypeStr', 'Bus: Bus_Can1_Bms');
L(c, 'cmd_core/1', 'cmd/1', 'cmd'); L(c, 'frames/1', 'can_tx/1', 'can_tx');
BS(c, 'Sel_cmd', {'K1_Cmd', 'K2_Cmd', 'K4_Cmd', 'Charger_Enable', 'Bal_Cmd'});
L(c, 'cmd_core/1', 'Sel_cmd/1', 'cmd');
% GPIO outputs GPIOB[4 5 6 7 10] = K1_CMD K2_CMD K4_CMD CHARGER_EN BAL_EN (shaddad_g474.ioc, T17): one boolean
% per pin; BAL_EN = any balancing switch requested (the per-cell switches are set through the AFE)
nDO = evalin('base', 'P.electronics.target.bms.do_n');
assert(nDO == 5, 'build_shaddad_bms:do', '5 GPIO outputs expected (P.electronics.target.bms.do_n = %d)', nDO);
A(c, 'simulink/Logic and Bit Operations/Logical Operator', 'Bal_any', 'Operator', 'OR', 'Inputs', '1');
L(c, 'Sel_cmd/5', 'Bal_any/1');
A(c, 'simulink/Signal Routing/Vector Concatenate', 'DO_cat', 'NumInputs', num2str(nDO));
for k = 1:4, L(c, sprintf('Sel_cmd/%d', k), sprintf('DO_cat/%d', k)); end
L(c, 'Bal_any/1', 'DO_cat/5');
A(c, 'simulink/Signal Attributes/Data Type Conversion', 'DO_bool', 'OutDataTypeStr', 'boolean');
A(c, 'stm32blockslib/Digital Port Write', 'DO_contactors', 'PortName', 'GPIOB', 'PinNumber', '[4 5 6 7 10]', ...
    'IOAsArray', 'on');
L(c, 'DO_cat/1', 'DO_bool/1'); L(c, 'DO_bool/1', 'DO_contactors/1');
F = local_frames();
BS(c, 'Sel_frames', {F.slot});
L(c, 'frames/1', 'Sel_frames/1', 'frames');
% FDCAN Write (Raw data, length and identifier from ports): inputs 1 Data uint8[8], 2 Length, 3 Id
% (stm32cube.blocks.FDCANBlock getInputNames)
for j = 1:numel(F)
    BS(c, ['Sel_' F(j).slot], {'data', 'len', 'id'});
    L(c, sprintf('Sel_frames/%d', j), ['Sel_' F(j).slot '/1'], F(j).slot);
    A(c, 'stm32blockslib/FDCAN Write', ['FDCAN1_' F(j).slot], 'FDCANModule', 'FDCAN1', 'BlockMode', 'Transmit', ...
        'DataFormatWriteMode', 'Raw data', 'DataLength', '8', 'CANFrameType', 'CAN');
    for k = 1:3, L(c, sprintf('Sel_%s/%d', F(j).slot, k), sprintf('FDCAN1_%s/%d', F(j).slot, k)); end
end
set_param(c, 'VariantControl', 'VC_TARGET == 1');
set_param(c, 'Description', ['HW IO_Out (VC_TARGET == 1): contactor / charger / balancing GPIO, FDCAN1 raw-data ' ...
    'transmit of the 7 BMS frames (inputs Data, Length, Id; Digital Port Write GPIOB[4 5 6 7 10] = K1 K2 K4 CHARGER_EN ' ...
    'BAL_EN = OR(Bal_Cmd); pin maps from shaddad_g474.ioc, T17). Root outputs keep the SIM buses.']);
set_param(s, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

% ---------------- BMS_Core ----------------
function local_core(core)
add_block('simulink/Sources/In1', [core '/meas'], 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Meas');
add_block('simulink/Sources/In1', [core '/bms_req_frame'], 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_CanFrame8');
add_block('simulink/Sinks/Out1', [core '/cmd'], 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Cmd');
add_block('simulink/Sinks/Out1', [core '/telem'], 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_BMS_Telemetry');
% ---- BMS_REQ decode (0.05 s) -> 0.01 s ----
A(core, 'simulink/Ports & Subsystems/Subsystem Reference', 'BMS_REQ_Decode', 'ReferencedSubsystem', 'bmsreq_can_unpack');
L(core, 'bms_req_frame/1', 'BMS_REQ_Decode/1', 'bms_req_frame');
RT(core, 'RT_req', 'Ts.bms_fast');
RT(core, 'RT_req_valid', 'Ts.bms_fast');
L(core, 'BMS_REQ_Decode/1', 'RT_req/1', 'bms_req');
L(core, 'BMS_REQ_Decode/2', 'RT_req_valid/1', 'bms_req_valid');

local_meas(core);
local_protection(core);
local_limits(core);
local_modemgr(core);
local_estimation(core);
local_chargectrl(core);
local_thermal(core);
local_balancing(core);
local_faultword(core);
local_cmdout(core);
local_telemetry(core);

L(core, 'meas/1', 'Meas/1', 'meas');
% the units' virtual buses (spec 6.3), each to the units that pick from it: m (Meas), mode (ModeManager), prot
% (Protection), est (Estimation), fw (FaultWord)
for c = {'Protection', 'Limits', 'ModeManager', 'Estimation', 'ChargeCtrl', 'Thermal', 'Balancing', 'FaultWord', 'CmdOut', 'Telemetry'}
    W(core, 'Meas', 'm', c{1}, 'm');
end
for c = {'Protection', 'ChargeCtrl', 'Balancing', 'FaultWord', 'CmdOut', 'Telemetry'}
    W(core, 'ModeManager', 'mode', c{1}, 'mode');
end
for c = {'ModeManager', 'FaultWord'}
    W(core, 'Protection', 'prot', c{1}, 'prot');
end
for c = {'Limits', 'FaultWord', 'Telemetry'}
    W(core, 'Estimation', 'est', c{1}, 'est');
end
W(core, 'FaultWord', 'fw', 'Telemetry', 'fw');
L(core, 'RT_req/1', sprintf('ModeManager/%d', PIN([core '/ModeManager'], 'req')), 'req');
L(core, 'RT_req_valid/1', sprintf('ModeManager/%d', PIN([core '/ModeManager'], 'req_valid')), 'req_valid');
% protection <-> limits / balancing
W(core, 'Limits', 'I_chg_lim', 'Protection', 'I_chg_lim');
W(core, 'Limits', 'I_dis_lim', 'Protection', 'I_dis_lim');
W(core, 'Balancing', 'contact_err', 'Protection', 'contact_err');
W(core, 'ChargeCtrl', 'ChargeComplete', 'ModeManager', 'ChargeComplete');
% charge control
W(core, 'Limits', 'I_chg_lim', 'ChargeCtrl', 'I_chg_lim');
% fault word
W(core, 'Limits', 'derate', 'FaultWord', 'derate');
W(core, 'Thermal', 'Fan_Cmd', 'FaultWord', 'Fan_Cmd');
% command output
W(core, 'Balancing', 'Bal_Cmd', 'CmdOut', 'Bal_Cmd');
W(core, 'Thermal', 'Fan_Cmd', 'CmdOut', 'Fan_Cmd');
W(core, 'ChargeCtrl', 'I_req', 'CmdOut', 'I_req');
L(core, sprintf('CmdOut/%d', POUT([core '/CmdOut'], 'cmd')), 'cmd/1', 'cmd');
% telemetry
W(core, 'Limits', 'I_dis_lim', 'Telemetry', 'I_dis_lim');
W(core, 'Limits', 'I_chg_lim', 'Telemetry', 'I_chg_lim');
W(core, 'Balancing', 'Balancing_Active', 'Telemetry', 'Balancing_Active');
W(core, 'Thermal', 'Fan_Cmd', 'Telemetry', 'Fan_Cmd');
W(core, 'CmdOut', 'Bal_Cmd_out', 'Telemetry', 'Bal_Cmd');
L(core, sprintf('Telemetry/%d', POUT([core '/Telemetry'], 'telem')), 'telem/1', 'telem');
end

% ---------------- Meas (0.01 s): conditioning, min/max/index, sign conversion ----------------
function local_meas(core)
s = SUB(core, 'Meas', {}, {});                 % the In Bus Element port meas, the Out Bus Element port m (virtual bus)
A(s, 'dspstat3/Maximum', 'V_max', 'fcn', 'Value and Index', 'indexBase', 'One');
A(s, 'dspstat3/Minimum', 'V_min', 'fcn', 'Value and Index', 'indexBase', 'One');
A(s, 'dspstat3/Maximum', 'T_max', 'fcn', 'Value', 'indexBase', 'One');
A(s, 'dspstat3/Minimum', 'T_min', 'fcn', 'Value', 'indexBase', 'One');
L(s, 'meas.V_cell_V', 'V_max/1', 'V_cell'); L(s, 'meas.V_cell_V', 'V_min/1', 'V_cell');
L(s, 'meas.T_cell_K', 'T_max/1', 'T_cell'); L(s, 'meas.T_cell_K', 'T_min/1', 'T_cell');
DTC(s, 'idx_max', 'uint8'); DTC(s, 'idx_min', 'uint8');
L(s, 'V_max/2', 'idx_max/1'); L(s, 'V_min/2', 'idx_min/1');
A(s, 'simulink/Math Operations/Sum of Elements', 'V_sum');
A(s, 'simulink/Math Operations/Gain', 'V_avg', 'Gain', 'single(1/P.power.pack.Ns)');
L(s, 'meas.V_cell_V', 'V_sum/1', 'V_cell'); L(s, 'V_sum/1', 'V_avg/1');
A(s, 'simulink/Signal Routing/Mux', 'V_mm', 'Inputs', '2');
A(s, 'simulink/Signal Routing/Mux', 'T_mm', 'Inputs', '2');
L(s, 'V_min/1', 'V_mm/1', 'V_min'); L(s, 'V_max/1', 'V_mm/2', 'V_max');
L(s, 'T_min/1', 'T_mm/1', 'T_min'); L(s, 'T_max/1', 'T_mm/2', 'T_max');
A(s, 'simulink/Math Operations/Gain', 'I_lib', 'Gain', 'single(-1)');      % batt_sl_lib: + = charge
L(s, 'meas.I_pack_A', 'I_lib/1', 'I_pack');
A(s, 'simulink/Math Operations/Product', 'P_pack', 'Inputs', '2');
L(s, 'meas.V_pack_V', 'P_pack/1', 'V_pack'); L(s, 'meas.I_pack_A', 'P_pack/2', 'I_pack');
el = {'V_cell', 'meas.V_cell_V'; 'T_cell', 'meas.T_cell_K'; 'I_dis', 'meas.I_pack_A'; 'I_lib', 'I_lib/1'; ...
      'V_pack', 'meas.V_pack_V'; 'V_bus', 'meas.V_bus_V'; 'V_dock', 'meas.V_dock_V'; 'T_amb', 'meas.T_amb_K'; 'T_air', 'meas.T_air_K'; ...
      'V_min', 'V_min/1'; 'V_max', 'V_max/1'; 'V_avg', 'V_avg/1'; 'idx_min', 'idx_min/1'; 'idx_max', 'idx_max/1'; ...
      'T_min', 'T_min/1'; 'T_max', 'T_max/1'; 'V_mm', 'V_mm/1'; 'T_mm', 'T_mm/1'; 'P_pack', 'P_pack/1'; ...
      'K1_fb', 'meas.K1_fb'; 'K2_fb', 'meas.K2_fb'; 'K4_fb', 'meas.K4_fb'; 'Dock', 'meas.Dock_Present'; ...
      'Safety_OK', 'meas.Safety_OK'; 'Safety_HV_OK', 'meas.Safety_HV_OK'; 'Key_On', 'meas.Key_On'; ...
      'FuseMain_OK', 'meas.FuseMain_OK'; 'Fuse_Blown', 'meas.Fuse_Blown'};
for k = 1:size(el, 1), OB(s, 'm', el{k, 1}, el{k, 2}); end
set_param(s, 'Description', 'Measurement conditioning (0.01 s): min/max with index (dspstat3), mean, [min;max] pairs, I_lib = -I_pack (batt_sl_lib + = charge), P_pack.');
end

% ---------------- Protection (0.01 s) ----------------
function local_protection(core)
s = SUB(core, 'Protection', {'I_chg_lim', 'I_dis_lim', 'contact_err'}, {});    % bus ports m, mode; prot (latched, FaultCritical, uv_warn, ov_warn)
b = 'P.power.bms';
VM = sprintf('batt_sl_lib/Protection/Battery Voltage\nMonitoring');
A(s, VM, 'E1_CellV_critical', 'UndervoltageLimit', ['single(' b '.V_uv)'], 'OvervoltageLimit', ['single(' b '.V_ov)'], ...
    'FaultySensorThreshold', ['single(' b '.FaultySensorThreshold)'], ...
    'QualTimeUndervoltage', [b '.qual.UV'], 'DisqualTimeUndervoltage', [b '.disqual'], ...
    'QualTimeOvervoltage', [b '.qual.OV'], 'DisqualTimeOvervoltage', [b '.disqual'], ...
    'QualTimeSensorFaulty', [b '.qual.sensor'], 'DisqualTimeSensorFaulty', [b '.disqual'], 'Ts', '-1', 'DataType', 'single');
A(s, VM, 'E2_CellV_warning', 'UndervoltageLimit', ['single(' b '.V_uv_warn)'], 'OvervoltageLimit', ['single(' b '.V_ov_warn)'], ...
    'FaultySensorThreshold', ['single(' b '.FaultySensorThreshold)'], ...
    'QualTimeUndervoltage', '5', 'DisqualTimeUndervoltage', '5', 'QualTimeOvervoltage', '5', ...
    'DisqualTimeOvervoltage', '5', 'QualTimeSensorFaulty', '5', 'DisqualTimeSensorFaulty', '5', 'Ts', '-1', 'DataType', 'single');
for e = {'E1_CellV_critical', 'E2_CellV_warning'}
    L(s, 'm.V_cell', [e{1} '/1'], 'V_cell'); L(s, 'm.V_pack', [e{1} '/2'], 'V_pack');
end
A(s, 'simulink/Sinks/Terminator', 'E2_sensor_unused'); L(s, 'E2_CellV_warning/3', 'E2_sensor_unused/1');
OB(s, 'prot', 'uv_warn', 'E2_CellV_warning/1', 'uv_warn'); OB(s, 'prot', 'ov_warn', 'E2_CellV_warning/2', 'ov_warn');
% E3 overcurrent: Current (+ charge), ChargingLimit (+), DischargingLimit (-)
A(s, sprintf('batt_sl_lib/Protection/Battery Current\nMonitoring'), 'E3_Overcurrent', ...
    'ErrorLimit', ['single(' b '.OC_ErrorLimit)'], 'QualTime', [b '.qual.OC'], 'DisqualTime', [b '.disqual'], 'Ts', '-1', 'DataType', 'single');
A(s, 'simulink/Math Operations/Gain', 'DCL_lib', 'Gain', 'single(-1)');
L(s, 'm.I_lib', 'E3_Overcurrent/1', 'I_lib');
L(s, 'I_chg_lim/1', 'E3_Overcurrent/2', 'I_chg_lim');
L(s, 'I_dis_lim/1', 'DCL_lib/1', 'I_dis_lim'); L(s, 'DCL_lib/1', 'E3_Overcurrent/3');
% E4 temperature ([min; max])
A(s, sprintf('batt_sl_lib/Protection/Battery Temperature\nMonitoring'), 'E4_Temperature', 'MinMaxFlag', 'on', ...
    'UndertemperatureLimit', ['single(' b '.T_ut)'], 'OvertemperatureLimit', ['single(' b '.T_ot)'], ...
    'QualTimeUndertemp', [b '.qual.OT'], 'DisqualTimeUndertemp', [b '.disqual'], ...
    'QualTimeOvertemp', [b '.qual.OT'], 'DisqualTimeOvertemp', [b '.disqual'], 'Ts', '-1', 'DataType', 'single');
L(s, 'm.T_mm', 'E4_Temperature/1', 'T_mm');
% E5 fault qualification: fuses MAIN TL TR IMP AUX CHG (Fuse_Blown b0-b5), charger OVP
A(s, 'commutil2/Integer to Bit Converter', 'Fuse_bits', 'nbits', '8', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
A(s, 'simulink/Signal Routing/Selector', 'Fuse_sel', 'IndexMode', 'One-based', 'Indices', '1:6', 'InputPortWidth', '8');
A(s, 'batt_sl_lib/Protection/Fault Qualification', 'E5_Fuse_qual', 'QualTime', [b '.qual_aux'], 'DisqualTime', [b '.disqual'], 'Ts', '-1');
L(s, 'm.Fuse_Blown', 'Fuse_bits/1', 'Fuse_Blown'); L(s, 'Fuse_bits/1', 'Fuse_sel/1'); L(s, 'Fuse_sel/1', 'E5_Fuse_qual/1');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'OVP_sym', 'relop', '>', ...
    'const', ['single(' b '.V_dock_ovp)'], 'OutDataTypeStr', 'boolean');
A(s, 'batt_sl_lib/Protection/Fault Qualification', 'E5_OVP_qual', 'QualTime', [b '.qual_aux'], 'DisqualTime', [b '.disqual'], 'Ts', '-1');
L(s, 'm.V_dock', 'OVP_sym/1', 'V_dock'); L(s, 'OVP_sym/1', 'E5_OVP_qual/1');
% E6 contact monitoring (1 s, in Balancing) gated by P.power.bms.contact_enable
CST(s, 'Contact_enable', [b '.contact_enable'], 'boolean', 'Ts.bms_fast');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Contact_err', 'Operator', 'AND', 'Inputs', '2');
L(s, 'contact_err/1', 'Contact_err/1', 'contact_err'); L(s, 'Contact_enable/1', 'Contact_err/2');
% booleans -> active word (bits 0..15) -> latch (reset on Fault_Reset_Req rising edge) -> mask
A(s, 'simulink/Signal Routing/Demux', 'Fuse_dmx', 'Outputs', '6');
L(s, 'E5_Fuse_qual/1', 'Fuse_dmx/1');
bits = {0, 'E1_CellV_critical/2'; 1, 'E1_CellV_critical/1'; 2, 'E3_Overcurrent/1'; 3, 'E4_Temperature/2'; ...
        4, 'E4_Temperature/1'; 5, 'E1_CellV_critical/3'; 8, 'Fuse_dmx/1'; 9, 'Fuse_dmx/2'; 10, 'Fuse_dmx/3'; ...
        11, 'Fuse_dmx/4'; 12, 'Fuse_dmx/5'; 13, 'Fuse_dmx/6'; 14, 'E5_OVP_qual/1'; 15, 'Contact_err/1'};
BITWORD(s, 'Active_word', bits, 32, 'Ts.bms_fast');
A(s, 'simulink/Discrete/Unit Delay', 'Latch_prev', 'InitialCondition', 'uint32(0)', 'SampleTime', 'Ts.bms_fast');
CST(s, 'Word_zero', '0', 'uint32', 'Ts.bms_fast');
A(s, 'simulink/Signal Routing/Switch', 'Latch_reset', 'Criteria', 'u2 ~= 0');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Latch_or', 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '2');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Latch_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', [b '.fault.mask_latched']);
L(s, 'Word_zero/1', 'Latch_reset/1'); L(s, 'mode.reset_edge', 'Latch_reset/2', 'reset_edge'); L(s, 'Latch_prev/1', 'Latch_reset/3');
L(s, 'Active_word/1', 'Latch_or/1', 'active_word'); L(s, 'Latch_reset/1', 'Latch_or/2');
L(s, 'Latch_or/1', 'Latch_mask/1'); h = L(s, 'Latch_mask/1', 'Latch_prev/1', 'latched');
OB(s, 'prot', 'latched', 'Latch_mask/1', 'latched');
local_log(h, 'bms_fault_latched');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Crit_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', [b '.fault.mask_crit_in']);
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Crit_any', 'relop', '~=', 'OutDataTypeStr', 'boolean');
L(s, 'Latch_mask/1', 'Crit_mask/1'); L(s, 'Crit_mask/1', 'Crit_any/1'); OB(s, 'prot', 'FaultCritical', 'Crit_any/1', 'FaultCritical');
shaddad_bu('portorder', s, {'m', 'mode', 'I_chg_lim', 'I_dis_lim', 'contact_err'}, 'Inport');
set_param(s, 'Description', ['Protection (0.01 s): E1 critical / E2 warning voltage monitoring, E3 overcurrent, E4 temperature, ' ...
    'E5 fault qualification (fuses, charger OVP), E6 gate; fault bits latched until a Fault_Reset_Req rising edge with the ' ...
    'symptom gone (library DisqualTime 0 would never clear, so P.power.bms.disqual is used).']);
end

% ---------------- Limits (0.01 s) ----------------
function local_limits(core)
s = SUB(core, 'Limits', {}, {'I_dis_lim', 'I_chg_lim', 'derate'});              % bus ports m, est
b = 'P.power.bms';
A(s, sprintf('batt_sl_lib/Current Management/Battery Discharging\nCurrent Limit'), 'E13_DCL', 'MinMaxFlag', 'on', ...
    'CellUndervoltageLimit', ['single(' b '.V_uv)'], 'R0', ['single(' b '.R0_limit)'], 'T_vec', ['single(' b '.dis.T_vec)'], ...
    'CurrentLimitCoolestCell', ['single(' b '.dis.I_cool)'], 'CurrentLimitHottestCell', ['single(' b '.dis.I_hot)']);
A(s, sprintf('batt_sl_lib/Current Management/Battery Charging\nCurrent Limit'), 'E14_CCL', 'MinMaxFlag', 'on', ...
    'CellOvervoltageLimit', ['single(' b '.V_ov_warn)'], 'R0', ['single(' b '.R0_limit)'], 'T_vec', ['single(' b '.chg.T_vec)'], ...
    'CurrentLimitCoolestCell', ['single(' b '.chg.I_cool)'], 'CurrentLimitHottestCell', ['single(' b '.chg.I_hot)']);
L(s, 'm.V_mm', 'E13_DCL/1', 'V_mm'); L(s, 'm.T_mm', 'E13_DCL/2', 'T_mm');
L(s, 'm.V_mm', 'E14_CCL/1', 'V_mm'); L(s, 'm.T_mm', 'E14_CCL/2', 'T_mm');
% DCL (library: negative) -> magnitude x SOC taper, <= I_dis_peak
A(s, 'simulink/Math Operations/Gain', 'DCL_mag', 'Gain', 'single(-1)');
RT(s, 'RT_SOC', 'Ts.bms_fast');
set_param([s '/RT_SOC'], 'InitialCondition', ['single(min(' b '.est.SOC_init))']);
A(s, 'simulink/Lookup Tables/1-D Lookup Table', 'E22_SOC_taper', 'BreakpointsForDimension1', ['single(' b '.soc_taper.bp)'], ...
    'Table', ['single(' b '.soc_taper.k)'], 'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip');
A(s, 'simulink/Math Operations/Product', 'DCL_taper', 'Inputs', '2');
CST(s, 'I_dis_peak', ['single(' b '.I_dis_peak)'], 'single', 'Ts.bms_fast');
A(s, 'simulink/Math Operations/MinMax', 'DCL_min', 'Function', 'min', 'Inputs', '2');
A(s, 'simulink/Discontinuities/Saturation', 'DCL_pos', 'UpperLimit', 'inf', 'LowerLimit', '0');
L(s, 'E13_DCL/1', 'DCL_mag/1');
L(s, 'est.SOC_min', 'RT_SOC/1', 'SOC_min'); L(s, 'RT_SOC/1', 'E22_SOC_taper/1');
L(s, 'DCL_mag/1', 'DCL_taper/1'); L(s, 'E22_SOC_taper/1', 'DCL_taper/2');
L(s, 'DCL_taper/1', 'DCL_min/1'); L(s, 'I_dis_peak/1', 'DCL_min/2');
L(s, 'DCL_min/1', 'DCL_pos/1'); L(s, 'DCL_pos/1', 'I_dis_lim/1', 'I_dis_lim');
CST(s, 'I_chg_max', ['single(' b '.I_chg_max)'], 'single', 'Ts.bms_fast');
A(s, 'simulink/Math Operations/MinMax', 'CCL_min', 'Function', 'min', 'Inputs', '2');
A(s, 'simulink/Discontinuities/Saturation', 'CCL_pos', 'UpperLimit', 'inf', 'LowerLimit', '0');
L(s, 'E14_CCL/1', 'CCL_min/1'); L(s, 'I_chg_max/1', 'CCL_min/2');
L(s, 'CCL_min/1', 'CCL_pos/1'); L(s, 'CCL_pos/1', 'I_chg_lim/1', 'I_chg_lim');
% derate flag: DCL below the warning level or CCL below the charge maximum
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'DCL_derated', 'relop', '<', ...
    'const', ['single(' b '.warn.I_dis_derate - 1)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'CCL_derated', 'relop', '<', ...
    'const', ['single(' b '.warn.I_chg_derate - 1)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Derate', 'Operator', 'OR', 'Inputs', '2');
L(s, 'DCL_pos/1', 'DCL_derated/1'); L(s, 'CCL_pos/1', 'CCL_derated/1');
L(s, 'DCL_derated/1', 'Derate/1'); L(s, 'CCL_derated/1', 'Derate/2'); L(s, 'Derate/1', 'derate/1', 'derate');
shaddad_bu('portorder', s, {'m', 'est'}, 'Inport');
set_param(s, 'Description', 'Current limits (0.01 s): E13 / E14 (MinMax inputs, temperature tables, voltage headroom), E22 SOC taper on SOC_min, peak / charge maxima. Outputs are magnitudes (A).');
end

% ---------------- ModeManager (0.01 s): MATLAB Function + MCU command timeout ----------------
function local_modemgr(core)
outs = {'K1_Cmd', 'K2_Cmd', 'K4_Cmd', 'Charger_Enable', 'HV_Ready', 'HV_Opening_Warn', 'Charging_Active', ...
        'Charge_Complete', 'Contactor_State', 'Fault_PrechargeFail', 'Fault_Weld', 'PrechargeRetries', ...
        'mcu_timeout', 'reset_edge'};
s = SUB(core, 'ModeManager', {'req_valid', 'ChargeComplete'}, {});              % bus ports m, req, prot; mode (outs)
A(s, 'simulink/Sinks/Terminator', 'req_valid_unused'); L(s, 'req_valid/1', 'req_valid_unused/1', 'req_valid');
% MCU command timeout: BMS_REQ alive frozen for P.power.bms.mcu_cmd_timeout -> HV_Enable_Req treated false
A(s, 'simulink/Logic and Bit Operations/Detect Change', 'Alive_change', 'vinit', '0');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Alive_frozen', 'Operator', 'NOT');
A(s, 'ee_sl_lib/General Control/On-Off Delay', 'Cmd_timeout', 'Td_on', 'P.power.bms.mcu_cmd_timeout', ...
    'Td_off', '0', 'IC', '0', 'Ts', 'Ts.bms_fast');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Timeout_b', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Not_timeout', 'Operator', 'NOT');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'HV_req_eff', 'Operator', 'AND', 'Inputs', '2');
L(s, 'req.AliveCounter', 'Alive_change/1', 'AliveCounter');
L(s, 'Alive_change/1', 'Alive_frozen/1');
L(s, 'Alive_frozen/1', 'Cmd_timeout/1');
L(s, 'Cmd_timeout/1', 'Timeout_b/1');
h = L(s, 'Timeout_b/1', 'Not_timeout/1', 'mcu_timeout');
OB(s, 'mode', 'mcu_timeout', 'Timeout_b/1', 'mcu_timeout');
local_log(h, 'bms_mcu_timeout');
L(s, 'req.HV_Enable_Req', 'HV_req_eff/1', 'HV_Enable_Req'); L(s, 'Not_timeout/1', 'HV_req_eff/2');
% Fault_Reset_Req rising edge (protection latch)
A(s, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Reset_edge', 'vinit', '0');
L(s, 'req.Fault_Reset_Req', 'Reset_edge/1', 'Fault_Reset_Req');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Reset_edge_b', 'OutDataTypeStr', 'boolean');
L(s, 'Reset_edge/1', 'Reset_edge_b/1'); OB(s, 'mode', 'reset_edge', 'Reset_edge_b/1', 'reset_edge');
% MATLAB Function (spec 4.1 table, body subsystems/power/bms_mode_manager.m)
mf = A(s, 'simulink/User-Defined Functions/MATLAB Function', 'BMS_ModeManager');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = sprintf(['function [K1_Cmd, K2_Cmd, K4_Cmd, Charger_Enable, HV_Ready, HV_Opening_Warn, Charging_Active, ' ...
    'Charge_Complete, Contactor_State, Fault_PrechargeFail, Fault_Weld, PrechargeRetries] = fcn(HV_Enable_Req, ' ...
    'Charge_Enable_Req, Fault_Reset_Req, Shutdown_Req, Safety_OK, Dock_Present, FaultCritical, V_pack, V_bus, I_pack, ' ...
    'ChargeComplete, prm)\n%%#codegen\n' ...
    '[K1_Cmd, K2_Cmd, K4_Cmd, Charger_Enable, HV_Ready, HV_Opening_Warn, Charging_Active, Charge_Complete, ' ...
    'Contactor_State, Fault_PrechargeFail, Fault_Weld, PrechargeRetries] = bms_mode_manager(HV_Enable_Req, ' ...
    'Charge_Enable_Req, Fault_Reset_Req, Shutdown_Req, Safety_OK, Dock_Present, FaultCritical, V_pack, V_bus, I_pack, ' ...
    'ChargeComplete, prm);\n']);
cfg.UpdateMethod = 'Discrete';
cfg.SampleTime = 'Ts.bms_fast';
cfg.Description = 'BMS mode manager (docs/specs/power.md 4.1), body bms_mode_manager.m; Stateflow not installed (D1).';
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', mf);
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'prm');
d.Scope = 'Parameter';
d.Tunable = false;
mk = Simulink.Mask.create(mf);
mk.addParameter('Type', 'edit', 'Name', 'prm', 'Prompt', 'Mode manager parameters', 'Value', 'P.power.bms.chart', ...
    'Evaluate', 'on', 'Tunable', 'off');
h = L(s, 'HV_req_eff/1', 'BMS_ModeManager/1', 'HV_Enable_eff');
local_log(h, 'bms_hv_enable_eff');
L(s, 'req.Charge_Enable_Req', 'BMS_ModeManager/2', 'Charge_Enable_Req');
L(s, 'req.Fault_Reset_Req', 'BMS_ModeManager/3', 'Fault_Reset_Req');
L(s, 'req.Shutdown_Req', 'BMS_ModeManager/4', 'Shutdown_Req');
L(s, 'm.Safety_HV_OK', 'BMS_ModeManager/5', 'Safety_HV_OK');
L(s, 'm.Dock', 'BMS_ModeManager/6', 'Dock_Present');
L(s, 'prot.FaultCritical', 'BMS_ModeManager/7', 'FaultCritical');
L(s, 'm.V_pack', 'BMS_ModeManager/8', 'V_pack');
L(s, 'm.V_bus', 'BMS_ModeManager/9', 'V_bus');
L(s, 'm.I_dis', 'BMS_ModeManager/10', 'I_pack');
L(s, 'ChargeComplete/1', 'BMS_ModeManager/11', 'ChargeComplete');
for k = 1:12
    OB(s, 'mode', outs{k}, sprintf('BMS_ModeManager/%d', k), outs{k});
    if strcmp(outs{k}, 'Contactor_State'), local_log(OUTLINE([s '/BMS_ModeManager'], k), 'bms_state'); end
end
shaddad_bu('portorder', s, {'m', 'req', 'req_valid', 'prot', 'ChargeComplete'}, 'Inport');
set_param(s, 'Description', ['Mode manager (0.01 s): MATLAB Function BMS_ModeManager (spec 4.1 transition table, ' ...
    'enum BmsContactorState, after() tick counters, P.power.bms.chart), Safety_OK input = meas.Safety_HV_OK ' ...
    '(stop-category-1 coil supply), MCU command timeout (BMS_REQ alive frozen 0.5 s -> HV_Enable_Req false, ' ...
    'Fault_Word b20), Fault_Reset_Req rising edge for the protection latch.']);
end

% ---------------- Estimation (0.1 s) ----------------
function local_estimation(core)
s = SUB(core, 'Estimation', {}, {});     % bus ports m; est (SOC_cell SOC_pack SOC_cc SOH SOE E_remaining Runtime P_dis_lim P_chg_lim SOC_min CellAH)
sig = {'I_lib', 'V_cell', 'T_cell', 'V_pack', 'P_pack'};
for k = 1:numel(sig)
    RT(s, ['RT_' sig{k}], 'Ts.bms_slow');
    L(s, ['m.' sig{k}], ['RT_' sig{k} '/1'], sig{k});
end
b = 'P.power.bms'; g = 'P.power.group';
% capacity estimate (E10, 1 s) -> rate limit + bounds -> CellAH (0.1 s)
RT(s, 'RT_I_1s', 'Ts.bms_bal'); RT(s, 'RT_SOC_1s', 'Ts.bms_bal');
CST(s, 'AH_init_vec', ['single(' b '.est.AH_init*ones(P.power.pack.Ns,1))'], 'single', 'Ts.bms_bal');
A(s, sprintf('batt_sl_lib/Estimators/Battery Capacity Estimator\n(Least Squares)'), 'E10_Capacity', ...
    'CurrentInputPartition', 'off', 'lsAlgorithm', 'Ordinary least squares', ...
    'SocChangeThreshold', ['single(' b '.cap_est.SocChangeThreshold)'], 'gamma', ['single(' b '.cap_est.gamma)'], ...
    'Ts', [b '.cap_est.Ts'], 'DataType', 'single');
L(s, 'RT_I_lib/1', 'RT_I_1s/1'); L(s, 'RT_I_1s/1', 'E10_Capacity/1');
L(s, 'RT_SOC_1s/1', 'E10_Capacity/2'); L(s, 'AH_init_vec/1', 'E10_Capacity/3');
RT(s, 'RT_AH', 'Ts.bms_slow');
L(s, 'E10_Capacity/1', 'RT_AH/1', 'CellAH_raw');
A(s, 'simulink/Discontinuities/Rate Limiter', 'AH_rate', 'RisingSlewLimit', ['single(' b '.est.AH_rate_lim)'], ...
    'FallingSlewLimit', ['single(-' b '.est.AH_rate_lim)'], 'SampleTimeMode', 'inherited', ...
    'InitialCondition', ['single(' b '.est.AH_init)']);
A(s, 'simulink/Discontinuities/Saturation', 'AH_bounds', 'UpperLimit', ['single(' b '.est.AH_bounds(2))'], ...
    'LowerLimit', ['single(' b '.est.AH_bounds(1))']);
L(s, 'RT_AH/1', 'AH_rate/1'); L(s, 'AH_rate/1', 'AH_bounds/1');
% SOC EKF x16 (one library block; the estimator is a For Each over the 16 cell groups), variable capacity
CST(s, 'SOC_init', ['single(' b '.est.SOC_init(:))'], 'single', 'Ts.bms_slow');
A(s, sprintf('batt_sl_lib/Estimators/SOC Estimator\n(Kalman Filter, Variable Capacity)'), 'E7_SOC_EKF_x16', ...
    'CurrentInputPartition', 'off', 'filterType', 'Extended Kalman filter', 'chargeDynamics', 'One time-constant dynamics', ...
    'Q', ['single(' b '.ekf.Q)'], 'R', ['single(' b '.ekf.R)'], 'P0', ['single(' b '.ekf.P0)'], 'Ts', 'Ts.bms_slow', ...
    'SOC_vec', ['single(' g '.SOC_vec)'], 'T_vec', ['single(' g '.T_vec)'], 'R0_mat', ['single(' g '.R0_mat)'], ...
    'R1_mat', ['single(' g '.R1_mat)'], 'tau1_mat', ['single(' g '.tau1_mat)'], 'V0_mat', ['single(' g '.V0_mat)'], ...
    'DataType', 'single');
L(s, 'RT_I_lib/1', 'E7_SOC_EKF_x16/1'); L(s, 'RT_T_cell/1', 'E7_SOC_EKF_x16/3');
L(s, 'AH_bounds/1', 'E7_SOC_EKF_x16/4', 'CellAH'); L(s, 'SOC_init/1', 'E7_SOC_EKF_x16/5');
% R0 ageing adaptation (D48 EOL robustness, T-PWR-13b). The EKF tables are BOL; an aged cell (R0 x 1.5)
% drops an extra (ratio - 1) x R0 x I, which the EKF would read as an SOC error of up to 0.05 on the flat
% LFP plateau. At every current step |dI| >= dI_min between two 0.1 s samples (mowing pulses, cut /
% non-cut phases) the measured string resistance d(sum V_cell)/dI is compared with the BOL table sum
% (RC branch: 0.1 s / tau1 = 40 s, negligible); the ratio is filtered (gain alpha per step, NVM init
% ratio_init, bounds) and the EKF gets V_cell - (ratio - 1) x R0_tab(SOC(k-1), T) x I_lib.
r = [b '.r0a'];
A(s, 'simulink/Math Operations/Sum of Elements', 'R0a_Vsum');
A(s, 'simulink/Discrete/Difference', 'R0a_dV', 'ICPrevInput', 'single(0)', 'OutDataTypeStr', 'single');
A(s, 'simulink/Discrete/Difference', 'R0a_dI', 'ICPrevInput', 'single(0)', 'OutDataTypeStr', 'single');
A(s, 'simulink/Math Operations/Abs', 'R0a_absdI');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'R0a_step', 'relop', '>=', ...
    'const', ['single(' r '.dI_min)'], 'OutDataTypeStr', 'boolean');
CST(s, 'R0a_one', 'single(1)', 'single', 'Ts.bms_slow');
A(s, 'simulink/Signal Routing/Switch', 'R0a_dI_safe', 'Criteria', 'u2 ~= 0');
A(s, 'simulink/Math Operations/Product', 'R0a_Rstep', 'Inputs', '*/');
A(s, 'simulink/Discrete/Unit Delay', 'R0a_SOC_prev', 'InitialCondition', ['single(' b '.est.SOC_init(:))'], ...
    'SampleTime', 'Ts.bms_slow');
A(s, 'simulink/Lookup Tables/n-D Lookup Table', 'R0a_R0_tab', 'NumberOfTableDimensions', '2', ...
    'BreakpointsForDimension1', ['single(' g '.SOC_vec)'], 'BreakpointsForDimension2', ['single(' g '.T_vec)'], ...
    'Table', ['single(' g '.R0_mat)'], 'InterpMethod', 'Linear point-slope', 'ExtrapMethod', 'Clip', ...
    'OutDataTypeStr', 'single');
A(s, 'simulink/Math Operations/Sum of Elements', 'R0a_Rsum');
A(s, 'simulink/Math Operations/Product', 'R0a_raw', 'Inputs', '*/');
A(s, 'simulink/Discontinuities/Saturation', 'R0a_raw_sat', 'UpperLimit', ['single(' r '.ratio_bounds(2))'], ...
    'LowerLimit', ['single(' r '.ratio_bounds(1))']);
A(s, 'simulink/Discrete/Unit Delay', 'R0a_ratio_prev', 'InitialCondition', ['single(' r '.ratio_init)'], ...
    'SampleTime', 'Ts.bms_slow');
A(s, 'simulink/Math Operations/Sum', 'R0a_err', 'Inputs', '+-');
DTC(s, 'R0a_step_s', 'single');
A(s, 'simulink/Math Operations/Product', 'R0a_gate', 'Inputs', '2');
A(s, 'simulink/Math Operations/Gain', 'R0a_alpha', 'Gain', ['single(' r '.alpha)']);
A(s, 'simulink/Math Operations/Sum', 'R0a_ratio', 'Inputs', '++');
A(s, 'simulink/Math Operations/Bias', 'R0a_excess', 'Bias', 'single(-1)');
A(s, 'simulink/Math Operations/Product', 'R0a_dR', 'Inputs', '2', 'Multiplication', 'Element-wise(.*)');
A(s, 'simulink/Math Operations/Product', 'R0a_dVcorr', 'Inputs', '2', 'Multiplication', 'Element-wise(.*)');
A(s, 'simulink/Math Operations/Sum', 'R0a_V_ekf', 'Inputs', '+-');
L(s, 'RT_V_cell/1', 'R0a_Vsum/1'); L(s, 'R0a_Vsum/1', 'R0a_dV/1');
L(s, 'RT_I_lib/1', 'R0a_dI/1'); L(s, 'R0a_dI/1', 'R0a_absdI/1'); L(s, 'R0a_absdI/1', 'R0a_step/1');
L(s, 'R0a_dI/1', 'R0a_dI_safe/1'); L(s, 'R0a_step/1', 'R0a_dI_safe/2', 'r0_step'); L(s, 'R0a_one/1', 'R0a_dI_safe/3');
L(s, 'R0a_dV/1', 'R0a_Rstep/1'); L(s, 'R0a_dI_safe/1', 'R0a_Rstep/2');
L(s, 'E7_SOC_EKF_x16/1', 'R0a_SOC_prev/1');
L(s, 'R0a_SOC_prev/1', 'R0a_R0_tab/1'); L(s, 'RT_T_cell/1', 'R0a_R0_tab/2');
L(s, 'R0a_R0_tab/1', 'R0a_Rsum/1');
L(s, 'R0a_Rstep/1', 'R0a_raw/1'); L(s, 'R0a_Rsum/1', 'R0a_raw/2'); L(s, 'R0a_raw/1', 'R0a_raw_sat/1');
L(s, 'R0a_raw_sat/1', 'R0a_err/1'); L(s, 'R0a_ratio_prev/1', 'R0a_err/2');
L(s, 'R0a_step/1', 'R0a_step_s/1'); L(s, 'R0a_err/1', 'R0a_gate/1'); L(s, 'R0a_step_s/1', 'R0a_gate/2');
L(s, 'R0a_gate/1', 'R0a_alpha/1');
L(s, 'R0a_ratio_prev/1', 'R0a_ratio/1'); L(s, 'R0a_alpha/1', 'R0a_ratio/2');
hR0 = L(s, 'R0a_ratio/1', 'R0a_ratio_prev/1', 'R0_ratio');
local_log(hR0, 'bms_r0_ratio');
L(s, 'R0a_ratio/1', 'R0a_excess/1'); L(s, 'R0a_excess/1', 'R0a_dR/1'); L(s, 'R0a_R0_tab/1', 'R0a_dR/2');
L(s, 'R0a_dR/1', 'R0a_dVcorr/1'); L(s, 'RT_I_lib/1', 'R0a_dVcorr/2');
L(s, 'RT_V_cell/1', 'R0a_V_ekf/1'); L(s, 'R0a_dVcorr/1', 'R0a_V_ekf/2');
L(s, 'R0a_V_ekf/1', 'E7_SOC_EKF_x16/2', 'V_cell_ekf');
OB(s, 'est', 'SOC_cell', 'E7_SOC_EKF_x16/1', 'SOC_cell');
L(s, 'E7_SOC_EKF_x16/1', 'RT_SOC_1s/1');
% pack-level capacity (mean of groups) -> coulomb counting, SOH, SOE energy capacity
A(s, 'simulink/Math Operations/Sum of Elements', 'AH_sum');
A(s, 'simulink/Math Operations/Gain', 'AH_mean', 'Gain', 'single(1/P.power.pack.Ns)');
L(s, 'AH_bounds/1', 'AH_sum/1'); L(s, 'AH_sum/1', 'AH_mean/1'); OB(s, 'est', 'CellAH', 'AH_mean/1', 'AH_pack');
CST(s, 'SOC_init_mean', ['single(mean(' b '.est.SOC_init))'], 'single', 'Ts.bms_slow');
A(s, sprintf('batt_sl_lib/Estimators/SOC Estimator\n(Coulomb Counting, Variable Capacity)'), 'E8_SOC_CC', ...
    'CurrentInputPartition', 'off', 'Ts', 'Ts.bms_slow', 'DataType', 'single');
L(s, 'RT_I_lib/1', 'E8_SOC_CC/1'); L(s, 'AH_mean/1', 'E8_SOC_CC/2'); L(s, 'SOC_init_mean/1', 'E8_SOC_CC/3');
OB(s, 'est', 'SOC_cc', 'E8_SOC_CC/1', 'SOC_cc');
A(s, 'batt_sl_lib/Estimators/SOH Estimator (Capacity-Based)', 'E11_SOH', 'AH_new', ['single(' b '.soh.AH_new)'], ...
    'AH_eol', ['single(' b '.soh.AH_eol)'], 'DataType', 'single');
L(s, 'AH_mean/1', 'E11_SOH/1'); OB(s, 'est', 'SOH', 'E11_SOH/1', 'SOH');
A(s, 'simulink/Math Operations/Gain', 'E_cap', 'Gain', ['single(' b '.est.V_nom_pack)']);   % Wh = Ah x 51.2 V (D48)
L(s, 'AH_mean/1', 'E_cap/1');
A(s, sprintf('batt_sl_lib/Estimators/SOE Estimator (Energy Counting,\nVariable Energy Capacity)'), 'E12_SOE', ...
    'CurrentInputPartition', 'off', 'Ts', 'Ts.bms_slow', 'DataType', 'single');
L(s, 'RT_I_lib/1', 'E12_SOE/1'); L(s, 'RT_V_pack/1', 'E12_SOE/2'); L(s, 'E_cap/1', 'E12_SOE/3'); L(s, 'SOC_init_mean/1', 'E12_SOE/4');
A(s, 'simulink/Discontinuities/Saturation', 'SOE_sat', 'UpperLimit', 'single(1)', 'LowerLimit', 'single(0)');
L(s, 'E12_SOE/1', 'SOE_sat/1'); OB(s, 'est', 'SOE', 'SOE_sat/1', 'SOE');
% pack SOC (display) = SOC_min / (1 - (SOC_max - SOC_min)), [0, 1]
A(s, 'dspstat3/Minimum', 'SOC_min_blk', 'fcn', 'Value');
A(s, 'dspstat3/Maximum', 'SOC_max_blk', 'fcn', 'Value');
CST(s, 'one', 'single(1)', 'single', 'Ts.bms_slow');
A(s, 'simulink/Math Operations/Sum', 'SOC_den', 'Inputs', '+-+');
A(s, 'simulink/Math Operations/Product', 'SOC_div', 'Inputs', '*/');
A(s, 'simulink/Discontinuities/Saturation', 'SOC_pack_sat', 'UpperLimit', 'single(1)', 'LowerLimit', 'single(0)');
L(s, 'E7_SOC_EKF_x16/1', 'SOC_min_blk/1'); L(s, 'E7_SOC_EKF_x16/1', 'SOC_max_blk/1');
L(s, 'one/1', 'SOC_den/1'); L(s, 'SOC_max_blk/1', 'SOC_den/2'); L(s, 'SOC_min_blk/1', 'SOC_den/3');
L(s, 'SOC_min_blk/1', 'SOC_div/1'); L(s, 'SOC_den/1', 'SOC_div/2'); L(s, 'SOC_div/1', 'SOC_pack_sat/1');
OB(s, 'est', 'SOC_pack', 'SOC_pack_sat/1', 'SOC_pack'); OB(s, 'est', 'SOC_min', 'SOC_min_blk/1', 'SOC_min');
% power capability (E9, group tables, 10 s horizon); library sign: discharging power negative
R10 = ['single(' b '.pwr_est.R_10s)'];
A(s, 'batt_sl_lib/Estimators/Battery Power Estimator', 'E9_Power', 'SOC_vec', ['single(' g '.SOC_vec)'], ...
    'T_vec', ['single(' g '.T_vec)'], 'V0_mat', ['single(' g '.V0_mat)'], 'R0ChargingTimeHorizon', R10, ...
    'R0DischargingTimeHorizon', R10, 'AH', ['single(' g '.AH)'], 'Ns', 'P.power.pack.Ns', 'Np', '1', ...
    'ChargingEfficiency', ['single(' b '.pwr_est.ChargingEfficiency)'], 'vmin', ['single(' b '.pwr_est.vmin)'], ...
    'vmax', ['single(' b '.pwr_est.vmax)'], 'SOCmin', ['single(' b '.pwr_est.SOCmin)'], ...
    'SOCmax', ['single(' b '.pwr_est.SOCmax)'], 'imin', ['single(' b '.pwr_est.imin)'], ...
    'imax', ['single(' b '.pwr_est.imax)'], 'Pmin', ['single(' b '.pwr_est.Pmin)'], 'Pmax', ['single(' b '.pwr_est.Pmax)'], ...
    'TimeHorizon', ['single(' b '.pwr_est.TimeHorizon)'], 'DataType', 'single');
L(s, 'E7_SOC_EKF_x16/1', 'E9_Power/1'); L(s, 'RT_T_cell/1', 'E9_Power/2');
A(s, 'simulink/Math Operations/Abs', 'P_dis_abs');
A(s, 'simulink/Math Operations/Abs', 'P_chg_abs');
L(s, 'E9_Power/1', 'P_dis_abs/1'); OB(s, 'est', 'P_dis_lim', 'P_dis_abs/1', 'P_dis_lim');
L(s, 'E9_Power/2', 'P_chg_abs/1'); OB(s, 'est', 'P_chg_lim', 'P_chg_abs/1', 'P_chg_lim');
% remaining usable energy and runtime (spec 3.9 item 4: Enom x SOH x (SOC_pack - 0.10) / P_avg):
%   E_rem = E_cap x max(SOC_pack - reserve, 0) with E_cap = learned AH x 51.2 V (= Enom x SOH, D48) and the
%   closed-loop (EKF) pack SOC - the open-loop energy-counting SOE keeps any initial-SOC error forever;
%   runtime = 3600 E_rem / max(P_avg, P_min), P_avg = moving average of P_pack over P_avg_window samples
%   at 1 s (600 s = one shift duty period: a 60 s average swings with the cut / non-cut phases and the
%   5-30 s mowing pulses by -20..+100 %, which defeats T-PWR-11 / T-PWR-13b)
A(s, 'simulink/Math Operations/Bias', 'SOC_reserve', 'Bias', ['single(-' b '.soc_reserve_runtime)']);
A(s, 'simulink/Discontinuities/Saturation', 'SOC_use', 'UpperLimit', 'single(1)', 'LowerLimit', 'single(0)');
A(s, 'simulink/Math Operations/Product', 'E_rem', 'Inputs', '2');
L(s, 'SOC_pack_sat/1', 'SOC_reserve/1'); L(s, 'SOC_reserve/1', 'SOC_use/1');
L(s, 'SOC_use/1', 'E_rem/1'); L(s, 'E_cap/1', 'E_rem/2'); OB(s, 'est', 'E_remaining', 'E_rem/1', 'E_remaining');
RT(s, 'RT_P_1s', 'Ts.bms_bal');
A(s, 'dspstat3/Moving Average', 'E20_P_avg', 'Method', 'Sliding window', 'SpecifyWindowLength', 'on', ...
    'WindowLength', [b '.P_avg_window'], 'OverlapLength', [b '.P_avg_window - 1']);
RT(s, 'RT_P_avg', 'Ts.bms_slow');
L(s, 'RT_P_pack/1', 'RT_P_1s/1'); L(s, 'RT_P_1s/1', 'E20_P_avg/1');
A(s, 'simulink/Discontinuities/Saturation', 'P_avg_min', 'UpperLimit', 'inf', 'LowerLimit', ['single(' b '.runtime_P_min)']);
A(s, 'simulink/Math Operations/Product', 'Runtime_div', 'Inputs', '*/');
A(s, 'simulink/Math Operations/Gain', 'Runtime_s', 'Gain', 'single(3600)');
L(s, 'E20_P_avg/1', 'RT_P_avg/1', 'P_avg'); L(s, 'RT_P_avg/1', 'P_avg_min/1');
L(s, 'E_rem/1', 'Runtime_div/1'); L(s, 'P_avg_min/1', 'Runtime_div/2');
L(s, 'Runtime_div/1', 'Runtime_s/1'); OB(s, 'est', 'Runtime', 'Runtime_s/1', 'Runtime');
set_param(s, 'Description', ['Estimation (0.1 s; capacity LS at 1 s): E7 SOC EKF x16 (variable capacity, D48; cell ' ...
    'voltages corrected by the R0 ageing ratio R0a_* learned at current steps), E8 coulomb ' ...
    'counting, E10 capacity (rate limit AH_rate_lim, bounds AH_bounds), E11 SOH, E12 SOE (EnergyCapacity = CellAH x 51.2 V), ' ...
    'E9 power capability, E20 moving average of P_pack (P_avg_window samples at 1 s = 600 s), ' ...
    'E_remaining = E_cap x max(SOC_pack - 0.10, 0), Runtime = 3600 E_remaining / max(P_avg, 200 W).']);
end

% ---------------- ChargeCtrl (0.1 s) ----------------
function local_chargectrl(core)
s = SUB(core, 'ChargeCtrl', {'I_chg_lim'}, {'I_req', 'ChargeComplete'});        % bus ports m, mode
sig = {'V_mm', 'V_max', 'I_lib'};
for k = 1:numel(sig)
    RT(s, ['RT_' sig{k}], 'Ts.bms_slow');
    L(s, ['m.' sig{k}], ['RT_' sig{k} '/1'], sig{k});
end
RT(s, 'RT_active', 'Ts.bms_slow'); RT(s, 'RT_ccl', 'Ts.bms_slow');
L(s, 'mode.Charging_Active', 'RT_active/1', 'Charging_Active'); L(s, 'I_chg_lim/1', 'RT_ccl/1', 'I_chg_lim');
b = 'P.power.bms';
CST(s, 'I_dis_zero', 'single(0)', 'single', 'Ts.bms_slow');
A(s, 'batt_sl_lib/Current Management/Battery CC-CV', 'E15_CCCV', 'MinMaxFlag', 'on', ...
    'MaxCellVoltage', ['single(' b '.V_cv)'], 'Kp', ['single(' b '.cccv.Kp)'], 'Ki', ['single(' b '.cccv.Ki)'], ...
    'Kaw', ['single(' b '.cccv.Kaw)'], 'Kt', ['single(' b '.cccv.Kt)'], 'Ts', 'Ts.bms_slow');
L(s, 'RT_active/1', 'E15_CCCV/1'); L(s, 'RT_V_mm/1', 'E15_CCCV/2'); L(s, 'RT_ccl/1', 'E15_CCCV/3'); L(s, 'I_dis_zero/1', 'E15_CCCV/4');
CST(s, 'I_chg_max', ['single(' b '.I_chg_max)'], 'single', 'Ts.bms_slow');
A(s, 'simulink/Math Operations/MinMax', 'I_req_min', 'Function', 'min', 'Inputs', '3');
A(s, 'simulink/Discontinuities/Saturation', 'I_req_pos', 'UpperLimit', 'inf', 'LowerLimit', 'single(0)');
A(s, 'simulink/Signal Routing/Switch', 'I_req_gate', 'Criteria', 'u2 ~= 0');
L(s, 'E15_CCCV/1', 'I_req_min/1', 'I_cccv'); L(s, 'RT_ccl/1', 'I_req_min/2'); L(s, 'I_chg_max/1', 'I_req_min/3');
L(s, 'I_req_min/1', 'I_req_pos/1');
L(s, 'I_req_pos/1', 'I_req_gate/1'); L(s, 'RT_active/1', 'I_req_gate/2'); L(s, 'I_dis_zero/1', 'I_req_gate/3');
L(s, 'I_req_gate/1', 'I_req/1', 'I_req');
% charge complete: V_max >= 3.545 V and charge current < 10 A for 60 s (E21 On-Off Delay)
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'V_top', 'relop', '>=', ...
    'const', ['single(' b '.charge_done.V)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'I_low', 'relop', '<', ...
    'const', ['single(' b '.charge_done.I)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Done_cond', 'Operator', 'AND', 'Inputs', '3');
A(s, 'ee_sl_lib/General Control/On-Off Delay', 'E21_Done_timer', 'Td_on', [b '.charge_done.t'], 'Td_off', '0', ...
    'IC', '0', 'Ts', 'Ts.bms_slow');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Done_b', 'OutDataTypeStr', 'boolean');
L(s, 'RT_V_max/1', 'V_top/1'); L(s, 'RT_I_lib/1', 'I_low/1');
L(s, 'V_top/1', 'Done_cond/1'); L(s, 'I_low/1', 'Done_cond/2'); L(s, 'RT_active/1', 'Done_cond/3');
L(s, 'Done_cond/1', 'E21_Done_timer/1'); L(s, 'E21_Done_timer/1', 'Done_b/1');
RT(s, 'RT_done', 'Ts.bms_fast');
L(s, 'Done_b/1', 'RT_done/1'); L(s, 'RT_done/1', 'ChargeComplete/1', 'ChargeComplete');
shaddad_bu('portorder', s, {'m', 'mode', 'I_chg_lim'}, 'Inport');
set_param(s, 'Description', 'Charge control (0.1 s): E15 CC-CV on V_max (CV 3.55 V), request = min(CC-CV, I_chg_lim, 100 A) while CHARGING; E21 charge complete (V_max >= 3.545 V and I < 10 A for 60 s).');
end

% ---------------- Thermal (0.1 s): fan control ----------------
function local_thermal(core)
s = SUB(core, 'Thermal', {}, {'Fan_Cmd'});                                      % bus port m
sig = {'T_amb', 'T_air', 'T_mm', 'T_max'};
for k = 1:numel(sig)
    RT(s, ['RT_' sig{k}], 'Ts.bms_slow');
    L(s, ['m.' sig{k}], ['RT_' sig{k} '/1'], sig{k});
end
b = 'P.power.bms';
A(s, 'batt_sl_lib/Thermal Management/Battery Coolant Control', 'E17_Fan_control', 'MinMaxFlag', 'on', ...
    'ControlStrategy', 'ON-OFF control', 'TempOn', 'single(P.power.thermal.TempOn)', 'TempOff', 'single(P.power.thermal.TempOff)', ...
    'c', 'single(0.1)', 'FlowRateStep', 'single(0.2)', 'DataType', 'single');
L(s, 'RT_T_amb/1', 'E17_Fan_control/1'); L(s, 'RT_T_air/1', 'E17_Fan_control/2'); L(s, 'RT_T_mm/1', 'E17_Fan_control/3');
A(s, 'simulink/Sinks/Terminator', 'FlowT_unused'); L(s, 'E17_Fan_control/2', 'FlowT_unused/1');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Fan_on', 'relop', '>', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Math Operations/Sum', 'dT_amb', 'Inputs', '+-');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Cells_hotter', 'relop', '>', ...
    'const', ['single(-' b '.fan.dT_amb)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Forced', 'relop', '>', ...
    'const', ['single(' b '.fan.T_force)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Fan_and', 'Operator', 'AND', 'Inputs', '2');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Fan_or', 'Operator', 'OR', 'Inputs', '2');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Fan_single', 'OutDataTypeStr', 'single');
L(s, 'E17_Fan_control/1', 'Fan_on/1');
L(s, 'RT_T_max/1', 'dT_amb/1'); L(s, 'RT_T_amb/1', 'dT_amb/2'); L(s, 'dT_amb/1', 'Cells_hotter/1');
L(s, 'RT_T_max/1', 'Forced/1');
L(s, 'Fan_on/1', 'Fan_and/1'); L(s, 'Cells_hotter/1', 'Fan_and/2');
L(s, 'Fan_and/1', 'Fan_or/1'); L(s, 'Forced/1', 'Fan_or/2');
L(s, 'Fan_or/1', 'Fan_single/1'); L(s, 'Fan_single/1', 'Fan_Cmd/1', 'Fan_Cmd');
set_param(s, 'Description', 'Fan control (0.1 s): E17 Battery Coolant Control ON-OFF 40 / 35 C on [T_min; T_max], AND T_max > T_amb - 1 K, OR T_max > 50 C.');
end

% ---------------- Balancing (1 s) + cell contact monitoring ----------------
function local_balancing(core)
s = SUB(core, 'Balancing', {}, {'Bal_Cmd', 'Balancing_Active', 'contact_err'});  % bus ports m, mode
sig = {'V_cell', 'V_max', 'I_dis'};
for k = 1:numel(sig)
    RT(s, ['RT_' sig{k}], 'Ts.bms_bal');
    L(s, ['m.' sig{k}], ['RT_' sig{k} '/1'], sig{k});
end
RT(s, 'RT_active', 'Ts.bms_bal');
L(s, 'mode.Charging_Active', 'RT_active/1', 'Charging_Active');
b = 'P.power.bms';
% enable = (CHARGING or REST) and V_max > 3.40 V (LFP imbalance observable only at the top)
A(s, 'simulink/Math Operations/Abs', 'I_abs');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Rest', 'relop', '<', 'const', ['single(' b '.I_rest)'], ...
    'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Chg_or_rest', 'Operator', 'OR', 'Inputs', '2');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Top', 'relop', '>', 'const', ['single(' b '.V_bal_enable)'], ...
    'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Bal_enable', 'Operator', 'AND', 'Inputs', '2');
L(s, 'RT_I_dis/1', 'I_abs/1'); L(s, 'I_abs/1', 'Rest/1');
L(s, 'RT_active/1', 'Chg_or_rest/1'); L(s, 'Rest/1', 'Chg_or_rest/2');
L(s, 'RT_V_max/1', 'Top/1');
L(s, 'Chg_or_rest/1', 'Bal_enable/1'); L(s, 'Top/1', 'Bal_enable/2');
A(s, 'batt_sl_lib/Cell Balancing/Passive Cell Balancing', 'E16_Passive_balancing', 'Threshold', ['single(' b '.bal.Threshold)'], ...
    'HysteresisBand', ['single(' b '.bal.Hyst)'], 'Td_on', [b '.bal.Td_on'], 'Td_off', [b '.bal.Td_off'], ...
    'Ts', 'Ts.bms_bal', 'dataType', 'boolean');
L(s, 'RT_V_cell/1', 'E16_Passive_balancing/1'); L(s, 'Bal_enable/1', 'E16_Passive_balancing/2', 'bal_enable');
L(s, 'E16_Passive_balancing/1', 'Bal_Cmd/1', 'Bal_Cmd');
L(s, 'E16_Passive_balancing/2', 'Balancing_Active/1', 'Balancing_Active');
% E6 cell contact monitoring (1 s), disabled downstream by P.power.bms.contact_enable
A(s, sprintf('batt_sl_lib/Protection/Battery Cell Contact\nMonitoring'), 'E6_Cell_contact', 'Ts', 'Ts.bms_bal', ...
    'Td', '10', 'VoltageChangeDetectionThreshold', 'single(5e-4)', 'VoltageChangePar', 'single(0.02)', ...
    'CumulativeErrorThreshold', 'single(1e-4)', 'QualTime', '20', 'DisqualTime', '0', 'DataType', 'single');
L(s, 'RT_V_cell/1', 'E6_Cell_contact/1');
A(s, 'simulink/Sinks/Terminator', 'E6_symptom'); L(s, 'E6_Cell_contact/2', 'E6_symptom/1');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'E6_any', 'Operator', 'OR', 'Inputs', '1');
RT(s, 'RT_contact', 'Ts.bms_fast');
L(s, 'E6_Cell_contact/1', 'E6_any/1'); L(s, 'E6_any/1', 'RT_contact/1'); L(s, 'RT_contact/1', 'contact_err/1', 'contact_err');
shaddad_bu('portorder', s, {'m', 'mode'}, 'Inport');
set_param(s, 'Description', 'Balancing (1 s): E16 Passive Cell Balancing (10 mV threshold, 3 mV hysteresis) enabled when (CHARGING or REST) and V_max > 3.40 V; E6 cell contact monitoring (gated off by P.power.bms.contact_enable).');
end

% ---------------- FaultWord (0.01 s): fault / warning words and fault level ----------------
function local_faultword(core)
s = SUB(core, 'FaultWord', {'derate', 'Fan_Cmd'}, {});     % bus ports m, prot, mode, est; fw (Fault_Word, Warning_Word, Fault_Level)
b = 'P.power.bms';
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Safety_open', 'Operator', 'NOT');
L(s, 'm.Safety_OK', 'Safety_open/1', 'Safety_OK');
BITWORD(s, 'Aux_word', {6, 'mode.Fault_PrechargeFail'; 7, 'mode.Fault_Weld'; 16, 'Safety_open/1'; 20, 'mode.mcu_timeout'}, 32, 'Ts.bms_fast');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'FW_or', 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '2');
L(s, 'prot.latched', 'FW_or/1', 'latched'); L(s, 'Aux_word/1', 'FW_or/2');
OB(s, 'fw', 'Fault_Word', 'FW_or/1', 'Fault_Word');
% warnings
RT(s, 'RT_SOC', 'Ts.bms_fast'); RT(s, 'RT_fan', 'Ts.bms_fast');
set_param([s '/RT_SOC'], 'InitialCondition', ['single(min(' b '.est.SOC_init))']);
L(s, 'est.SOC_pack', 'RT_SOC/1', 'SOC_pack'); L(s, 'Fan_Cmd/1', 'RT_fan/1', 'Fan_Cmd');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'OT_warn', 'relop', '>', 'const', ['single(' b '.T_warn)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'SOC_low', 'relop', '<', 'const', ['single(' b '.warn.SOC_low)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'SOC_crit', 'relop', '<', 'const', ['single(' b '.warn.SOC_crit)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Math Operations/Sum', 'dV', 'Inputs', '+-');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'dV_big', 'relop', '>', 'const', ['single(' b '.warn.dV_imb)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'At_top', 'relop', '>', 'const', ['single(' b '.warn.V_imb_top)'], 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Imbalance', 'Operator', 'AND', 'Inputs', '2');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Fan_warn', 'relop', '>', 'OutDataTypeStr', 'boolean');
L(s, 'm.T_max', 'OT_warn/1', 'T_max'); L(s, 'RT_SOC/1', 'SOC_low/1'); L(s, 'RT_SOC/1', 'SOC_crit/1');
L(s, 'm.V_max', 'dV/1', 'V_max'); L(s, 'm.V_min', 'dV/2', 'V_min'); L(s, 'dV/1', 'dV_big/1'); L(s, 'm.V_max', 'At_top/1', 'V_max');
L(s, 'dV_big/1', 'Imbalance/1'); L(s, 'At_top/1', 'Imbalance/2'); L(s, 'RT_fan/1', 'Fan_warn/1');
BITWORD(s, 'Warn_word', {0, 'prot.uv_warn'; 1, 'prot.ov_warn'; 2, 'OT_warn/1'; 3, 'SOC_low/1'; 4, 'SOC_crit/1'; ...
    5, 'derate/1'; 6, 'Imbalance/1'; 7, 'Fan_warn/1'}, 32, 'Ts.bms_fast');
OB(s, 'fw', 'Warning_Word', 'Warn_word/1', 'Warning_Word');
% Fault_Level: 3 (HV off) / 2 (derate) / 1 (warning or safety open) / 0
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Lvl3_mask', 'logicop', 'AND', 'UseBitMask', 'on', 'BitMask', [b '.fault.mask_level3']);
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Lvl2_mask', 'logicop', 'AND', 'UseBitMask', 'on', 'BitMask', [b '.fault.mask_level2']);
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Lvl3', 'relop', '~=', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Lvl2', 'relop', '~=', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Warn_any', 'relop', '~=', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Lvl1', 'Operator', 'OR', 'Inputs', '2');
L(s, 'FW_or/1', 'Lvl3_mask/1'); L(s, 'FW_or/1', 'Lvl2_mask/1');
L(s, 'Lvl3_mask/1', 'Lvl3/1'); L(s, 'Lvl2_mask/1', 'Lvl2/1');
L(s, 'Warn_word/1', 'Warn_any/1'); L(s, 'Warn_any/1', 'Lvl1/1'); L(s, 'Safety_open/1', 'Lvl1/2');
for k = 1:3
    DTC(s, sprintf('L%d_u8', k), 'uint8');
    A(s, 'simulink/Math Operations/Gain', sprintf('L%d_w', k), 'Gain', sprintf('uint8(%d)', k), ...
        'OutDataTypeStr', 'uint8', 'ParamDataTypeStr', 'uint8');
    L(s, sprintf('L%d_u8/1', k), sprintf('L%d_w/1', k));
end
L(s, 'Lvl1/1', 'L1_u8/1'); L(s, 'Lvl2/1', 'L2_u8/1'); L(s, 'Lvl3/1', 'L3_u8/1');
A(s, 'simulink/Math Operations/MinMax', 'Level_max', 'Function', 'max', 'Inputs', '3');
for k = 1:3, L(s, sprintf('L%d_w/1', k), sprintf('Level_max/%d', k)); end
OB(s, 'fw', 'Fault_Level', 'Level_max/1', 'Fault_Level');
shaddad_bu('portorder', s, {'m', 'prot', 'mode', 'est', 'derate', 'Fan_Cmd'}, 'Inport');
set_param(s, 'Description', 'Fault_Word = latched protection bits | b6 PRECHARGE_FAIL | b7 K1_WELD | b16 SAFETY_OPEN | b20 MCU_CMD_TIMEOUT (b17-19 LV: no rail measurement at the BMS); Warning_Word b0-b7; Fault_Level 3/2/1/0 from P.power.bms.fault masks.');
end

% ---------------- CmdOut (0.01 s): Bus_BMS_Cmd + Unit Delay (loop breaker) ----------------
function local_cmdout(core)
s = SUB(core, 'CmdOut', {'Bal_Cmd', 'Fan_Cmd', 'I_req'}, {'cmd', 'Bal_Cmd_out'});   % bus ports m, mode
RT(s, 'RT_bal', 'Ts.bms_fast'); RT(s, 'RT_fan', 'Ts.bms_fast'); RT(s, 'RT_ireq', 'Ts.bms_fast');
L(s, 'Bal_Cmd/1', 'RT_bal/1', 'Bal_Cmd'); L(s, 'Fan_Cmd/1', 'RT_fan/1', 'Fan_Cmd'); L(s, 'I_req/1', 'RT_ireq/1', 'I_req');
% balancing switches only while allowed at the 10 ms rate: (CHARGING or REST) and V_max > 3.40 V
% (the 1 s balancing algorithm alone would keep a command up to 1 s into a discharge)
b = 'P.power.bms';
A(s, 'simulink/Math Operations/Abs', 'I_abs');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Rest', 'relop', '<', 'const', ['single(' b '.I_rest)'], ...
    'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Chg_or_rest', 'Operator', 'OR', 'Inputs', '2');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Top', 'relop', '>', 'const', ['single(' b '.V_bal_enable)'], ...
    'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Bal_allowed', 'Operator', 'AND', 'Inputs', '2');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Bal_gate', 'Operator', 'AND', 'Inputs', '2');
L(s, 'm.I_dis', 'I_abs/1', 'I_dis'); L(s, 'I_abs/1', 'Rest/1');
L(s, 'mode.K4_Cmd', 'Chg_or_rest/1', 'K4_Cmd'); L(s, 'Rest/1', 'Chg_or_rest/2');
L(s, 'm.V_max', 'Top/1', 'V_max');
L(s, 'Chg_or_rest/1', 'Bal_allowed/1'); L(s, 'Top/1', 'Bal_allowed/2');
L(s, 'RT_bal/1', 'Bal_gate/1'); L(s, 'Bal_allowed/1', 'Bal_gate/2', 'bal_allowed');
L(s, 'Bal_gate/1', 'Bal_Cmd_out/1', 'Bal_Cmd_out');
CST(s, 'V_req', 'single(P.power.bms.V_cvl)', 'single', 'Ts.bms_fast');
BC(s, 'Cmd_BC', 'Bus_BMS_Cmd', {'K1_Cmd', 'mode.K1_Cmd'; 'K2_Cmd', 'mode.K2_Cmd'; 'K4_Cmd', 'mode.K4_Cmd'; ...
    'Bal_Cmd', 'Bal_gate/1'; 'Fan_Cmd', 'RT_fan/1'; 'Charger_Enable', 'mode.Charger_Enable'; ...
    'Charger_I_Req_A', 'RT_ireq/1'; 'Charger_V_Req_V', 'V_req/1'});
A(s, 'simulink/Discrete/Unit Delay', 'Cmd_delay', 'InitialCondition', 'P.ic.Bus_BMS_Cmd', 'SampleTime', 'Ts.bms_fast');
L(s, 'Cmd_BC/1', 'Cmd_delay/1', 'cmd_now'); L(s, 'Cmd_delay/1', 'cmd/1', 'cmd');
shaddad_bu('portorder', s, {'m', 'mode', 'Bal_Cmd', 'Fan_Cmd', 'I_req'}, 'Inport');
set_param(s, 'Description', ['Bus_BMS_Cmd assembly; balancing gated at 10 ms ((CHARGING (K4 request) or |I| < I_rest) ' ...
    'and V_max > 3.40 V); Unit Delay (0.01 s) = BMS output stage and the plant -> BMS -> plant loop breaker (E24).']);
end

% ---------------- Telemetry (0.1 s) ----------------
function local_telemetry(core)
modeIn = {'Contactor_State', 'HV_Ready', 'HV_Opening_Warn', 'Charging_Active', 'Charge_Complete'};
fwIn = {'Fault_Word', 'Warning_Word', 'Fault_Level'};
limIn = {'I_dis_lim', 'I_chg_lim'};
balIn = {'Bal_Cmd', 'Balancing_Active'};
s = SUB(core, 'Telemetry', [limIn, {'Fan_Cmd'}, balIn], {'telem'});              % bus ports m, mode, est, fw
msig = {'V_pack', 'V_bus', 'I_dis', 'P_pack', 'V_min', 'V_max', 'V_avg', 'idx_min', 'idx_max', 'T_min', 'T_max', ...
        'T_air', 'K1_fb', 'K2_fb', 'K4_fb'};
for k = 1:numel(msig)
    RT(s, ['RT_' msig{k}], 'Ts.hk');
    L(s, ['m.' msig{k}], ['RT_' msig{k} '/1'], msig{k});
end
for k = 1:numel(modeIn)
    RT(s, ['RT_' modeIn{k}], 'Ts.hk');
    L(s, ['mode.' modeIn{k}], ['RT_' modeIn{k} '/1'], modeIn{k});
end
for k = 1:numel(fwIn)
    RT(s, ['RT_' fwIn{k}], 'Ts.hk');
    L(s, ['fw.' fwIn{k}], ['RT_' fwIn{k} '/1'], fwIn{k});
end
for k = 1:numel(limIn)
    RT(s, ['RT_' limIn{k}], 'Ts.hk');
    L(s, [limIn{k} '/1'], ['RT_' limIn{k} '/1'], limIn{k});
end
for k = 1:numel(balIn)
    RT(s, ['RT_' balIn{k}], 'Ts.hk');
    L(s, [balIn{k} '/1'], ['RT_' balIn{k} '/1'], balIn{k});
end
A(s, 'simulink/Sources/Counter Free-Running', 'Alive', 'NumBits', '8', 'tsamp', 'Ts.hk');
DTC(s, 'Alive_u8', 'uint8');
L(s, 'Alive/1', 'Alive_u8/1');
CST(s, 'CVL', 'single(P.power.bms.V_cvl)', 'single', 'Ts.hk');
CST(s, 'DVL', 'single(P.power.bms.V_dvl)', 'single', 'Ts.hk');
el = {'AliveCounter', 'Alive_u8/1'; 'Contactor_State', 'RT_Contactor_State/1'; 'HV_Ready', 'RT_HV_Ready/1'; ...
      'HV_Opening_Warn', 'RT_HV_Opening_Warn/1'; 'K1_Closed', 'RT_K1_fb/1'; 'K2_Closed', 'RT_K2_fb/1'; ...
      'K4_Closed', 'RT_K4_fb/1'; 'Charging_Active', 'RT_Charging_Active/1'; 'Charge_Complete', 'RT_Charge_Complete/1'; ...
      'SOC_pack', 'est.SOC_pack'; 'SOC_cell', 'est.SOC_cell'; 'SOC_cc', 'est.SOC_cc'; 'SOH', 'est.SOH'; 'SOE', 'est.SOE'; ...
      'E_remaining_Wh', 'est.E_remaining'; 'Runtime_Remaining_s', 'est.Runtime'; 'V_pack_V', 'RT_V_pack/1'; ...
      'V_bus_V', 'RT_V_bus/1'; 'I_pack_A', 'RT_I_dis/1'; 'P_pack_W', 'RT_P_pack/1'; 'V_cell_min_V', 'RT_V_min/1'; ...
      'V_cell_max_V', 'RT_V_max/1'; 'V_cell_avg_V', 'RT_V_avg/1'; 'idx_cell_min', 'RT_idx_min/1'; ...
      'idx_cell_max', 'RT_idx_max/1'; 'T_cell_min_K', 'RT_T_min/1'; 'T_cell_max_K', 'RT_T_max/1'; ...
      'T_pack_air_K', 'RT_T_air/1'; 'I_dis_lim_A', 'RT_I_dis_lim/1'; 'I_chg_lim_A', 'RT_I_chg_lim/1'; ...
      'P_dis_lim_W', 'est.P_dis_lim'; 'P_chg_lim_W', 'est.P_chg_lim'; 'V_chg_lim_V', 'CVL/1'; 'V_dis_lim_V', 'DVL/1'; ...
      'Balancing_Active', 'RT_Balancing_Active/1'; 'Balancing_Cmd', 'RT_Bal_Cmd/1'; 'Fan_Cmd', 'Fan_Cmd/1'; ...
      'Fault_Level', 'RT_Fault_Level/1'; 'Fault_Word', 'RT_Fault_Word/1'; 'Warning_Word', 'RT_Warning_Word/1'};
BC(s, 'Telem_BC', 'Bus_BMS_Telemetry', el);
L(s, 'Telem_BC/1', 'telem/1', 'telem');
shaddad_bu('portorder', s, [{'m', 'mode', 'est', 'fw'}, limIn, {'Fan_Cmd'}, balIn], 'Inport');
set_param(s, 'Description', 'Telemetry (0.1 s): Bus_BMS_Telemetry with AliveCounter (+1 per 0.1 s), fault / warning words, contactor state, CVL / DVL.');
end
