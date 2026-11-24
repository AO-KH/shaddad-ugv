function built = build_power_plant(which)
%BUILD_POWER_PLANT  Power distribution, safety chain, LV rails, implements, dock charger, Power_Plant (owner: power, T05).
%
%   build_power_plant()          build every T05 Subsystem Reference file (dependencies first)
%   build_power_plant(names)     build only the listed files
%   built = build_power_plant(...) names of the files written
%
%   Files (subsystems/power/, docs/specs/power.md 2.4-2.8 / 3.3-3.7, build plan T05.1-T05.6):
%     implcmd_can_pack / implcmd_can_unpack   CAN1 0x140 IMPL_CMD codec (MCU -> implement ECU, 50 ms)
%     implfb_can_pack  / implfb_can_unpack    CAN1 0x141 IMPL_FB codec (implement ECU -> MCU, 50 ms)
%     Safety_Chain     hard-wired E-stop loop, reset stretch, SR latch, immediate / cat-1 contacts, coil gating (1 ms)
%     PDU              fuses, shunt, K1/K2/K4, precharge, key switch, C_PDU, cables, BMS supply, GND (only
%                      Electrical Reference of the DC48 network), port power meters
%     LV_Distribution  DC-DC 48->24, 48->12, 12->5 V, 3.3 V LDO, K3 + F_24IMP, rail loads, rail sensors
%     impl_ecu         implement ECU (mask: drv = P.power.impl.<mower|tiller>, impl_type): IMPL_CMD decode,
%                      CAN timeout, ramp, speed PI, 10 ms torque-reference Unit Delay (D36), IMPL_FB
%     Impl_None / Impl_Mower / Impl_Tiller / Impl_Sprayer   VC_IMPL choices with identical ports
%     Dock_Charger     dock contacts + VC_CHG variant (1 BMS-controlled source, 2 batt_lib Charger)
%     Power_Plant      top of the POWER plant (ICD 3.2 Power row)
%   Every numeric block value is a P.power.* (or P.drive.tie / P.structure.impl.sprayer.p_set) or Ts.*
%   expression. No MATLAB Function blocks, no Solver Configuration (test benches add one).
%   Buses (model layout spec 6.2-6.3): Safety_Chain, PDU, LV_Distribution and Dock_Charger take In Bus Element
%   ports (each consumer picks its element right in front of it) and send their outputs as one Out Bus Element
%   bus each (safety, pdu, lv, charger); Power_Plant keeps its ICD 3.2 ports and wires its blocks with these
%   buses, a Bus Selector in front of each consumer that takes single signals. Inside a file, signals that several
%   combiners take in their own operand orders travel as one virtual bus that each combiner picks from right in
%   front of it (LV_Distribution: the rail powers; impl_ecu: the fault flags). Every signal line is named (L5).
%   The CAN codecs, impl_ecu and the implement choices keep their ports. Every file is laid out by
%   shaddad_layout(<file>, 'Recursive', true) and net-checked (local_netcheck) before it is saved.

order = {'implcmd_can_pack', 'implcmd_can_unpack', 'implfb_can_pack', 'implfb_can_unpack', ...
         'Safety_Chain', 'PDU', 'LV_Distribution', 'impl_ecu', 'Impl_None', 'Impl_Mower', 'Impl_Tiller', ...
         'Impl_Sprayer', 'Dock_Charger', 'Power_Plant'};
if nargin < 1 || isempty(which), which = order; end
if ischar(which) || isstring(which), which = cellstr(which); end
bad = setdiff(which, order);
if ~isempty(bad), error('build_power_plant:name', 'Unknown file(s): %s', strjoin(bad, ', ')); end
which = order(ismember(order, which));
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dirp = fullfile(root, 'subsystems', 'power');
built = {};
for k = 1:numel(which)
    nm = which{k};
    f = fullfile(dirp, [nm '.slx']);
    switch nm
        case 'implcmd_can_pack',   local_implcmd_pack(nm, f);
        case 'implcmd_can_unpack', local_implcmd_unpack(nm, f);
        case 'implfb_can_pack',    local_implfb_pack(nm, f);
        case 'implfb_can_unpack',  local_implfb_unpack(nm, f);
        case 'Safety_Chain',       local_safety(nm, f);
        case 'PDU',                local_pdu(nm, f);
        case 'LV_Distribution',    local_lv(nm, f);
        case 'impl_ecu',           local_ecu(nm, f);
        case 'Impl_None',          local_impl_none(nm, f);
        case 'Impl_Mower',         local_impl_motor(nm, f, 'P.power.impl.mower', 1, false);
        case 'Impl_Tiller',        local_impl_motor(nm, f, 'P.power.impl.tiller', 2, true);
        case 'Impl_Sprayer',       local_impl_sprayer(nm, f);
        case 'Dock_Charger',       local_charger(nm, f);
        case 'Power_Plant',        local_plant(nm, f);
    end
    built{end+1} = nm; %#ok<AGROW>
end
end

% ===========================================================================================
%  library paths / helpers
% ===========================================================================================
function s = LIB(k)
switch k
    case 'in',     s = 'simulink/Sources/In1';
    case 'out',    s = 'simulink/Sinks/Out1';
    case 'cp',     s = 'nesl_utility/Connection Port';
    case 'sps',    s = sprintf('nesl_utility/Simulink-PS\nConverter');
    case 'pss',    s = sprintf('nesl_utility/PS-Simulink\nConverter');
    case 'logic',  s = 'simulink/Logic and Bit Operations/Logical Operator';
    case 'cmp',    s = 'simulink/Logic and Bit Operations/Compare To Constant';
    case 'onoff',  s = 'ee_sl_lib/General Control/On-Off Delay';
    case 'res',    s = 'ee_lib/Passive/Resistor';
    case 'cap',    s = 'ee_lib/Passive/Capacitor';
    case 'fuse',   s = 'ee_lib/Switches & Breakers/Fuse';
    case 'relay',  s = 'ee_lib/Switches & Breakers/Relays/SPST Relay';
    case 'switch', s = 'ee_lib/Switches & Breakers/SPST Switch';
    case 'isens',  s = 'ee_lib/Sensors & Transducers/Current Sensor';
    case 'vsens',  s = 'ee_lib/Sensors & Transducers/Voltage Sensor';
    case 'dload',  s = 'ee_lib/Passive/Dynamic Load';
    case 'dcdc',   s = 'ee_lib/Semiconductors & Converters/Converters/DC-DC Converter';
    case 'gnd',    s = 'ee_lib/Connectors & References/Electrical Reference';
    case 'open',   s = 'ee_lib/Connectors & References/Open Circuit';
    case 'mref',   s = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
    case 'ccs',    s = 'fl_lib/Electrical/Electrical Sources/Controlled Current Source';
    case 'ssr',    s = 'simulink/Ports & Subsystems/Subsystem Reference';
    otherwise, error('build_power_plant:lib', 'unknown lib key %s', k);
end
end

function p = A(sys, lib, n, varargin)
% add_block (no position: the layout engine places every block when the file is saved, local_save)
p = shaddad_bu('add', lib, [sys '/' n], varargin{:});
end
function h = L(sys, s, d, nm)
% signal line s -> d ('Blk/k'), named nm (L5); without nm the default name of shaddad_bu('nline')
if nargin < 4, nm = ''; end
h = shaddad_bu('nline', sys, s, d, nm);
end
function PL(sys, a, b)
add_line(sys, a, b, 'autorouting', 'smart');
end
function ph = PH(blk)
ph = get_param(blk, 'PortHandles');
end
function h = CPH(blk)
% physical handle of a Connection Port block (inside the file)
ph = get_param(blk, 'PortHandles'); h = ph.RConn(1);
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
if isempty(t)
    p = A(sys, 'simulink/Sources/Constant', n, 'Value', v, 'SampleTime', ts);
else
    p = A(sys, 'simulink/Sources/Constant', n, 'Value', v, 'OutDataTypeStr', t, 'SampleTime', ts);
end
end
function HEIGHT(b, h)
% set block b's height to h px (its width kept); the engine keeps a block's own height (graphical only)
p = get_param(b, 'Position');
set_param(b, 'Position', [p(1) p(2) p(3) p(2) + h]);
end
function p = LOG(sys, n, op, nin)
% Logical Operator; one with several inputs stands 30 px per input (the pitch of the picks and lines that feed
% it), so that the name labels of its input lines have room (L6); the engine keeps a block's own height
if strcmp(op, 'NOT')
    p = A(sys, LIB('logic'), n, 'Operator', 'NOT');
else
    p = A(sys, LIB('logic'), n, 'Operator', op, 'Inputs', num2str(nin), 'Position', [0 0 30 30*nin]);
end
end
function p = CMP(sys, n, op, c)
p = A(sys, LIB('cmp'), n, 'relop', op, 'const', c, 'OutDataTypeStr', 'boolean');
end
function BS(sys, n, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', n, 'OutputSignals', strjoin(sigs, ','));
end
function SEL(sys, n, src, nm, sigs, dst)
% a Bus Selector n right in front of block dst (spec 6.2): it picks sigs from the bus src ('Blk/k', its line named
% nm), output k into input k of dst
BS(sys, n, sigs); L(sys, src, [n '/1'], nm);
for k = 1:numel(sigs), L(sys, sprintf('%s/%d', n, k), sprintf('%s/%d', dst, k)); end
end
function BC(sys, n, bus, el)
% nonvirtual Bus Creator, rows {elementName, 'Blk/port'}
A(sys, 'simulink/Signal Routing/Bus Creator', n, 'Inputs', num2str(size(el, 1)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:size(el, 1)
    L(sys, el{k, 2}, [n '/' num2str(k)], el{k, 1});
end
end
function BCV(sys, n, el)
% virtual Bus Creator next to its producer (spec 6.3), rows {elementName, 'Blk/port'} (shaddad_bu('vbus'))
shaddad_bu('vbus', sys, n, el);
end
function IN(sys, n, port, dt, ts, dims)
if nargin < 6
    A(sys, LIB('in'), n, 'Port', num2str(port), 'OutDataTypeStr', dt, 'SampleTime', ts);
else
    A(sys, LIB('in'), n, 'Port', num2str(port), 'OutDataTypeStr', dt, 'SampleTime', ts, 'PortDimensions', dims);
end
end
function OUT(sys, n, port, dt)
if nargin < 4
    A(sys, LIB('out'), n, 'Port', num2str(port));
else
    A(sys, LIB('out'), n, 'Port', num2str(port), 'OutDataTypeStr', dt);
end
end
function p = CP(sys, n, port, dom, side)
switch dom
    case 'E', ct = 'Connection: foundation.electrical.electrical';
    case 'M', ct = 'Connection: foundation.mechanical.rotational.rotational';
    case 'T', ct = 'Connection: foundation.thermal.thermal';
end
p = A(sys, LIB('cp'), n, 'Port', num2str(port), 'Side', side, 'ConnectionType', ct);
end
function o = PSS(sys, n, hPS, unit)
% PS-Simulink converter on the physical-signal handle hPS; returns 'n/1'
b = A(sys, LIB('pss'), n, 'Unit', unit);
pb = PH(b); PL(sys, hPS, pb.LConn(1));
o = [n '/1'];
end
function h = SPS(sys, n, src, unit, nm)
% Simulink-PS converter fed by Simulink source 'Blk/port' (line named nm, default as L); returns its physical
% output handle
if nargin < 5, nm = ''; end
b = A(sys, LIB('sps'), n, 'Unit', unit, 'FilteringAndDerivatives', 'zero');
L(sys, src, [n '/1'], nm);
pb = PH(b); h = pb.RConn(1);
end
function o = VSENS(sys, n, hP, hN)
% ee_lib Voltage Sensor between hP (+) and hN (-) -> PS-Simulink 'n' (V); returns 'n/1'
b = A(sys, LIB('vsens'), [n '_sens']);
p = PH(b); PL(sys, p.LConn(1), hP); PL(sys, p.RConn(2), hN);
o = PSS(sys, n, p.RConn(1), 'V');
end
function [o, hP, hN] = ISENS(sys, n)
% ee_lib Current Sensor (current + from its + port hP through the sensor to hN) -> PS-Simulink 'n' (A)
b = A(sys, LIB('isens'), [n '_sens']);
p = PH(b); hP = p.LConn(1); hN = p.RConn(2);
o = PSS(sys, n, p.RConn(1), 'A');
end
function p = PROD(sys, n, a, b)
p = A(sys, 'simulink/Math Operations/Product', n, 'Inputs', '2');
L(sys, a, [n '/1']); L(sys, b, [n '/2']);
end
function p = SUMS(sys, n, signs, srcs)
% Sum of srcs in that operand order; three or more operands: a rectangular Sum (SUMR)
if numel(srcs) >= 3
    p = SUMR(sys, n, signs);
else
    p = A(sys, 'simulink/Math Operations/Sum', n, 'Inputs', signs);
end
for k = 1:numel(srcs), L(sys, srcs{k}, sprintf('%s/%d', n, k)); end
end
function p = SUMR(sys, n, signs)
% a rectangular Sum with signs (no spacers), its inputs 30 px apart down its left side, in operand order, at the
% pitch of the picks and lines that feed it (a round Sum takes its inputs around its circle); IconShape is
% graphical, the operands stay as they are
p = A(sys, 'simulink/Math Operations/Sum', n, 'Inputs', signs, 'IconShape', 'rectangular', ...
    'Position', [0 0 30 30*numel(signs)]);
end
function p = MUX(sys, n, srcs)
p = A(sys, 'simulink/Signal Routing/Mux', n, 'Inputs', num2str(numel(srcs)));
for k = 1:numel(srcs), L(sys, srcs{k}, sprintf('%s/%d', n, k)); end
end
function p = ONOFF(sys, n, ton, toff, ic, ts)
p = A(sys, LIB('onoff'), n, 'Td_on', ton, 'Td_off', toff, 'IC', ic, 'Ts', ts);
end
function sub = SUBSYS(sys, n)
% empty virtual subsystem
sub = [sys '/' n];
add_block('simulink/Ports & Subsystems/Subsystem', sub);
delete_line(sub, 'In1/1', 'Out1/1'); delete_block([sub '/In1']); delete_block([sub '/Out1']);
end
function p = PK(sys, port, el, dst, nm)
% pick the element el of the In Bus Element port 'port' of sys right in front of the input dst ('Blk/k'), its
% line named nm (default el): shaddad_bu('pick')
if nargin < 5, nm = ''; end
p = shaddad_bu('pick', sys, port, el, dst, nm);
end
function OB(sys, port, el, src, nm)
% the element el of the Out Bus Element port 'port' of sys, fed by src ('Blk/k'), its line named nm (default el):
% shaddad_bu('outbus')
if nargin < 5, nm = ''; end
shaddad_bu('outbus', sys, port, el, src, nm);
end
function local_fresh(name, f)
% build rule 3: close the model if open, delete and recreate (Subsystem Reference file)
if bdIsLoaded(name), close_system(name, 0); end
if isfile(f), delete(f); end
new_system(name, 'Subsystem');
end
function local_save(name, f, desc)
% the layout engine at every level of the file (an engine error stops the build), the physical net check, save
set_param(name, 'Description', desc);
shaddad_layout(name, 'Recursive', true);
bad = local_netcheck(name);
if ~isempty(bad)
    error('build_power_plant:net', '%s: physical connection problem before save:\n%s', name, strjoin(bad, newline));
end
save_system(name, f);
close_system(name, 0);
end

function bad = local_netcheck(sys)
% every physical port of every block (all levels) must be connected to at least one other block
b = find_system(sys, 'LookUnderMasks', 'none', 'FollowLinks', 'off', 'MatchFilter', @Simulink.match.allVariants, ...
    'Type', 'block');
bad = {};
for k = 1:numel(b)
    if strcmp(b{k}, sys), continue, end
    par = get_param(b{k}, 'Parent');
    if ~strcmp(par, sys) && strcmp(get_param(par, 'Variant'), 'on'), continue, end   % variant choices connect by name
    pc = get_param(b{k}, 'PortConnectivity');
    me = get_param(b{k}, 'Handle');
    for j = 1:numel(pc)
        if ~(startsWith(pc(j).Type, 'LConn') || startsWith(pc(j).Type, 'RConn')), continue, end
        h = [pc(j).DstBlock(:); pc(j).SrcBlock(:)]; h = h(h > 0);
        if isempty(h) || all(h == me)
            bad{end+1} = sprintf('  %s %s', strrep(b{k}, newline, ' '), pc(j).Type); %#ok<AGROW>
        end
    end
end
end
function v = local_variant(sys, n, ins, outs, conns, choices)
% Variant Subsystem (expression, update diagram) with the same port names in every choice (Simulink maps a
% choice's ports to the parent's by name; the VC_IMPL choices order their inports their own way).
% ins / outs: names; conns: {name, dom, side}; choices: {blockName, referencedSSR ('' = plain subsystem), control}
v = [sys '/' n];
add_block('simulink/Ports & Subsystems/Variant Subsystem', v);
ch = find_system(v, 'MatchFilter', @Simulink.match.allVariants, 'SearchDepth', 1);
ch = setdiff(ch, {v});
for k = 1:numel(ch), delete_block(ch{k}); end
for k = 1:numel(ins), add_block(LIB('in'), [v '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block(LIB('out'), [v '/' outs{k}], 'Port', num2str(k)); end
for k = 1:size(conns, 1)
    CP(v, conns{k, 1}, k, conns{k, 2}, conns{k, 3});
end
for k = 1:size(choices, 1)
    if isempty(choices{k, 2})
        s = SUBSYS(v, choices{k, 1});
        for j = 1:numel(ins), add_block(LIB('in'), [s '/' ins{j}], 'Port', num2str(j)); end
        for j = 1:numel(outs), add_block(LIB('out'), [s '/' outs{j}], 'Port', num2str(j)); end
        for j = 1:size(conns, 1), CP(s, conns{j, 1}, j, conns{j, 2}, conns{j, 3}); end
    else
        add_block(LIB('ssr'), [v '/' choices{k, 1}], 'ReferencedSubsystem', choices{k, 2});
    end
    set_param([v '/' choices{k, 1}], 'VariantControl', choices{k, 3});
end
set_param(v, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

% ===========================================================================================
%  CAN codecs 0x140 IMPL_CMD / 0x141 IMPL_FB (ICD 4; ports kept: two of them sit inside the MCU)
% ===========================================================================================
function local_frame_out(name, idHex, ts)
CST(name, 'ID', sprintf('uint32(hex2dec(''%s''))', idHex), 'uint32', ts);
CST(name, 'LEN', '8', 'uint8', ts);
CST(name, 'VALID', 'true', 'boolean', ts);
BC(name, 'Frame_BC', 'Bus_CanFrame8', {'id', 'ID/1'; 'len', 'LEN/1'; 'data', 'Pack/1'; 'valid', 'VALID/1'});
L(name, 'Frame_BC/1', 'frame/1', 'frame');
end

function local_implcmd_pack(name, f)
local_fresh(name, f);
ts = 'Ts.can_slow';
IN(name, 'enable', 1, 'boolean', ts, '1');
IN(name, 'fault_reset', 2, 'boolean', ts, '1');
IN(name, 'alive', 3, 'uint8', ts, '1');
IN(name, 'speed_rpm', 4, 'single', ts, '1');
OUT(name, 'frame', 1, 'Bus: Bus_CanFrame8');
DTC(name, 'en_u8', 'uint8'); DTC(name, 'rst_u8', 'uint8');
A(name, 'simulink/Logic and Bit Operations/Shift Arithmetic', 'rst_shift', 'BitShiftNumberSource', 'Dialog', ...
    'BitShiftDirection', 'Left', 'BitShiftNumber', '1');
A(name, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Ctrl_OR', 'logicop', 'OR', 'UseBitMask', 'off', ...
    'NumInputPorts', '2');
L(name, 'enable/1', 'en_u8/1', 'enable');
L(name, 'fault_reset/1', 'rst_u8/1', 'fault_reset');
L(name, 'rst_u8/1', 'rst_shift/1', 'fault_reset_u8');
L(name, 'en_u8/1', 'Ctrl_OR/1', 'enable_b0'); L(name, 'rst_shift/1', 'Ctrl_OR/2', 'fault_reset_b1');
DTC(name, 'speed_i16', 'int16');                       % [1 rpm], round to nearest, saturate
L(name, 'speed_rpm/1', 'speed_i16/1', 'speed_rpm');
CST(name, 'Reserved', 'zeros(4,1)', 'uint8', ts);
A(name, 'mcblib/Signal Management/Byte Pack', 'Pack', 'datatypes', '{''uint8'',''uint8'',''int16'',''uint8''}', ...
    'byteAlign', '1');
L(name, 'Ctrl_OR/1', 'Pack/1', 'ctrl'); L(name, 'alive/1', 'Pack/2', 'alive');
L(name, 'speed_i16/1', 'Pack/3', 'speed_cmd'); L(name, 'Reserved/1', 'Pack/4', 'reserved');
local_frame_out(name, '140', ts);
local_save(name, f, ['implcmd_can_pack (T05, ICD 4 0x140 IMPL_CMD, 50 ms): byte 0 ctrl b0 enable, b1 fault_reset; ' ...
    'byte 1 alive; bytes 2-3 int16 speed_cmd [1 rpm at blade / tiller rotor]; bytes 4-7 reserved 0. Inputs (Ts.can_slow): ' ...
    '1 enable boolean, 2 fault_reset boolean, 3 alive uint8, 4 speed_rpm single. Output frame Bus_CanFrame8 (valid = true).']);
end

function local_implcmd_unpack(name, f)
local_fresh(name, f);
ts = 'Ts.can_slow';
IN(name, 'frame', 1, 'Bus: Bus_CanFrame8', ts);
OUT(name, 'enable', 1); OUT(name, 'fault_reset', 2); OUT(name, 'alive', 3); OUT(name, 'speed_rpm', 4);
OUT(name, 'valid', 5, 'boolean');
% the frame's fields, each picked by a Bus Selector right in front of the block that uses it
BS(name, 'Sel', {'id'}); BS(name, 'Valid_sel', {'valid'}); BS(name, 'Data_sel', {'data'});
L(name, 'frame/1', 'Sel/1', 'frame'); L(name, 'frame/1', 'Valid_sel/1'); L(name, 'frame/1', 'Data_sel/1');
CMP(name, 'ID_match', '==', 'hex2dec(''140'')');
LOG(name, 'Frame_ok', 'AND', 2);
L(name, 'Sel/1', 'ID_match/1'); L(name, 'ID_match/1', 'Frame_ok/1', 'id_match'); L(name, 'Valid_sel/1', 'Frame_ok/2');
d = [name '/Decode'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', d);
delete_line(d, 'In1/1', 'Out1/1'); delete_block([d '/In1']); delete_block([d '/Out1']);
A(d, LIB('in'), 'data', 'Port', '1');
on = {'enable', 'boolean'; 'fault_reset', 'boolean'; 'alive', 'uint8'; 'speed_rpm', 'single'};
for k = 1:size(on, 1)
    A(d, LIB('out'), on{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', on{k, 2}, 'OutputWhenDisabled', 'held', ...
        'InitialOutput', '0');
end
A(d, 'mcblib/Signal Management/Byte Unpack', 'Unpack', 'dimensions', '{1,1,1,4}', ...
    'datatypes', '{''uint8'',''uint8'',''int16'',''uint8''}', 'byteAlign', '1');
L(d, 'data/1', 'Unpack/1', 'data');
A(d, 'simulink/Sinks/Terminator', 'Reserved'); L(d, 'Unpack/4', 'Reserved/1', 'reserved');
for b = 0:1
    A(d, 'simulink/Logic and Bit Operations/Extract Bits', sprintf('bit%d', b), 'bitsToExtract', 'Range of bits', ...
        'bitIdxRange', sprintf('[%d %d]', b, b), 'outScalingMode', 'Treat bit field as an integer');
    DTC(d, sprintf('bit%d_b', b), 'boolean');
    L(d, 'Unpack/1', sprintf('bit%d/1', b), 'ctrl');
    L(d, sprintf('bit%d/1', b), sprintf('bit%d_b/1', b), sprintf('%s_bit', on{b + 1, 1}));
end
L(d, 'bit0_b/1', 'enable/1', 'enable'); L(d, 'bit1_b/1', 'fault_reset/1', 'fault_reset');
L(d, 'Unpack/2', 'alive/1', 'alive');
DTC(d, 'speed_single', 'single'); L(d, 'Unpack/3', 'speed_single/1', 'speed_cmd');
L(d, 'speed_single/1', 'speed_rpm/1', 'speed_rpm');
L(name, 'Data_sel/1', 'Decode/1');
L(name, 'Frame_ok/1', 'Decode/Enable', 'frame_ok');
for k = 1:4, L(name, sprintf('Decode/%d', k), sprintf('%s/1', on{k, 1}), on{k, 1}); end
L(name, 'Frame_ok/1', 'valid/1', 'frame_ok');
local_save(name, f, ['implcmd_can_unpack (T05, ICD 4 0x140): in frame Bus_CanFrame8 (Ts.can_slow) -> out 1 enable ' ...
    'boolean, 2 fault_reset boolean, 3 alive uint8, 4 speed_rpm single (rpm at blade / rotor), held while no valid ' ...
    '0x140 frame (IC 0), 5 valid (fresh frame this sample).']);
end

function local_implfb_pack(name, f)
local_fresh(name, f);
ts = 'Ts.can_slow';
nm = {'running', 'boolean'; 'fault', 'boolean'; 'ready', 'boolean'; 'impl_type', 'uint8'; 'speed_rpm', 'single'; ...
      'current_A', 'single'; 'temp_degC', 'single'; 'alive', 'uint8'};
for k = 1:size(nm, 1), IN(name, nm{k, 1}, k, nm{k, 2}, ts, '1'); end
OUT(name, 'frame', 1, 'Bus: Bus_CanFrame8');
A(name, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Status_OR', 'logicop', 'OR', 'UseBitMask', 'off', ...
    'NumInputPorts', '3');
for b = 0:2
    DTC(name, sprintf('b%d_u8', b), 'uint8');
    L(name, sprintf('%s/1', nm{b + 1, 1}), sprintf('b%d_u8/1', b), nm{b + 1, 1});
    if b == 0
        L(name, 'b0_u8/1', 'Status_OR/1', 'running_b0');
    else
        A(name, 'simulink/Logic and Bit Operations/Shift Arithmetic', sprintf('b%d_shift', b), ...
            'BitShiftNumberSource', 'Dialog', 'BitShiftDirection', 'Left', 'BitShiftNumber', num2str(b));
        L(name, sprintf('b%d_u8/1', b), sprintf('b%d_shift/1', b), [nm{b + 1, 1} '_u8']);
        L(name, sprintf('b%d_shift/1', b), sprintf('Status_OR/%d', b + 1), sprintf('%s_b%d', nm{b + 1, 1}, b));
    end
end
DTC(name, 'speed_i16', 'int16');                                     % [1 rpm]
L(name, 'speed_rpm/1', 'speed_i16/1', 'speed_rpm');
A(name, 'simulink/Math Operations/Gain', 'current_k', 'Gain', '10');   % [0.1 A]
DTC(name, 'current_i16', 'int16');
L(name, 'current_A/1', 'current_k/1', 'current_A'); L(name, 'current_k/1', 'current_i16/1', 'current_dA');
A(name, 'simulink/Math Operations/Bias', 'temp_off', 'Bias', '40');    % [1 degC, offset -40]
DTC(name, 'temp_u8', 'uint8');
L(name, 'temp_degC/1', 'temp_off/1', 'temp_degC'); L(name, 'temp_off/1', 'temp_u8/1', 'temp_off40');
A(name, 'mcblib/Signal Management/Byte Pack', 'Pack', 'datatypes', ...
    '{''uint8'',''uint8'',''int16'',''int16'',''uint8'',''uint8''}', 'byteAlign', '1');
HEIGHT([name '/Pack'], 300);           % 50 px per input, so the six lines into it run level (the alive line no
                                       % longer turns up across the temp chain; graphical, fix round 1)
L(name, 'Status_OR/1', 'Pack/1', 'status'); L(name, 'impl_type/1', 'Pack/2', 'impl_type');
L(name, 'speed_i16/1', 'Pack/3', 'speed'); L(name, 'current_i16/1', 'Pack/4', 'current');
L(name, 'temp_u8/1', 'Pack/5', 'temp'); L(name, 'alive/1', 'Pack/6', 'alive');
local_frame_out(name, '141', ts);
local_save(name, f, ['implfb_can_pack (T05, ICD 4 0x141 IMPL_FB, 50 ms): byte 0 status b0 running, b1 fault, b2 ready; ' ...
    'byte 1 impl_type; bytes 2-3 int16 speed [1 rpm]; 4-5 int16 current [0.1 A]; 6 uint8 temp [1 degC, offset -40]; ' ...
    '7 alive. Inputs (Ts.can_slow): 1 running, 2 fault, 3 ready (boolean), 4 impl_type uint8, 5 speed_rpm single, ' ...
    '6 current_A single, 7 temp_degC single, 8 alive uint8. Output frame Bus_CanFrame8.']);
end

function local_implfb_unpack(name, f)
local_fresh(name, f);
ts = 'Ts.can_slow';
IN(name, 'frame', 1, 'Bus: Bus_CanFrame8', ts);
on = {'running', 'boolean'; 'fault', 'boolean'; 'ready', 'boolean'; 'impl_type', 'uint8'; 'speed_rpm', 'single'; ...
      'current_A', 'single'; 'temp_degC', 'single'; 'alive', 'uint8'};
for k = 1:size(on, 1), OUT(name, on{k, 1}, k); end
OUT(name, 'valid', 9, 'boolean');
% the frame's fields, each picked by a Bus Selector right in front of the block that uses it
BS(name, 'Sel', {'id'}); BS(name, 'Valid_sel', {'valid'}); BS(name, 'Data_sel', {'data'});
L(name, 'frame/1', 'Sel/1', 'frame'); L(name, 'frame/1', 'Valid_sel/1'); L(name, 'frame/1', 'Data_sel/1');
CMP(name, 'ID_match', '==', 'hex2dec(''141'')');
LOG(name, 'Frame_ok', 'AND', 2);
L(name, 'Sel/1', 'ID_match/1'); L(name, 'ID_match/1', 'Frame_ok/1', 'id_match'); L(name, 'Valid_sel/1', 'Frame_ok/2');
d = [name '/Decode'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', d);
delete_line(d, 'In1/1', 'Out1/1'); delete_block([d '/In1']); delete_block([d '/Out1']);
A(d, LIB('in'), 'data', 'Port', '1');
for k = 1:size(on, 1)
    A(d, LIB('out'), on{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', on{k, 2}, 'OutputWhenDisabled', 'held', ...
        'InitialOutput', '0');
end
A(d, 'mcblib/Signal Management/Byte Unpack', 'Unpack', 'dimensions', '{1,1,1,1,1,1}', ...
    'datatypes', '{''uint8'',''uint8'',''int16'',''int16'',''uint8'',''uint8''}', 'byteAlign', '1');
HEIGHT([d '/Unpack'], 240);            % 40 px per output, so the alive line clears the temp chain (graphical,
                                       % fix round 1)
L(d, 'data/1', 'Unpack/1', 'data');
for b = 0:2
    A(d, 'simulink/Logic and Bit Operations/Extract Bits', sprintf('bit%d', b), 'bitsToExtract', 'Range of bits', ...
        'bitIdxRange', sprintf('[%d %d]', b, b), 'outScalingMode', 'Treat bit field as an integer');
    DTC(d, sprintf('bit%d_b', b), 'boolean');
    L(d, 'Unpack/1', sprintf('bit%d/1', b), 'status');
    L(d, sprintf('bit%d/1', b), sprintf('bit%d_b/1', b), sprintf('%s_bit', on{b + 1, 1}));
    L(d, sprintf('bit%d_b/1', b), sprintf('%s/1', on{b + 1, 1}), on{b + 1, 1});
end
L(d, 'Unpack/2', 'impl_type/1', 'impl_type');
DTC(d, 'speed_single', 'single'); L(d, 'Unpack/3', 'speed_single/1', 'speed'); L(d, 'speed_single/1', 'speed_rpm/1', 'speed_rpm');
DTC(d, 'current_single', 'single');
A(d, 'simulink/Math Operations/Gain', 'current_k', 'Gain', '0.1');
L(d, 'Unpack/4', 'current_single/1', 'current'); L(d, 'current_single/1', 'current_k/1', 'current_dA');
L(d, 'current_k/1', 'current_A/1', 'current_A');
DTC(d, 'temp_single', 'single');
A(d, 'simulink/Math Operations/Bias', 'temp_off', 'Bias', '-40');
L(d, 'Unpack/5', 'temp_single/1', 'temp'); L(d, 'temp_single/1', 'temp_off/1', 'temp_off40');
L(d, 'temp_off/1', 'temp_degC/1', 'temp_degC');
L(d, 'Unpack/6', 'alive/1', 'alive');
L(name, 'Data_sel/1', 'Decode/1');
L(name, 'Frame_ok/1', 'Decode/Enable', 'frame_ok');
for k = 1:size(on, 1), L(name, sprintf('Decode/%d', k), sprintf('%s/1', on{k, 1}), on{k, 1}); end
L(name, 'Frame_ok/1', 'valid/1', 'frame_ok');
local_save(name, f, ['implfb_can_unpack (T05, ICD 4 0x141): in frame Bus_CanFrame8 (Ts.can_slow) -> out 1 running, ' ...
    '2 fault, 3 ready (boolean), 4 impl_type uint8, 5 speed_rpm, 6 current_A, 7 temp_degC (single), 8 alive uint8 ' ...
    '(held while no valid 0x141 frame, IC 0), 9 valid (fresh frame this sample).']);
end

% ===========================================================================================
%  Safety_Chain (spec 2.5 / 3.4, D30)
% ===========================================================================================
function local_safety(name, f)
local_fresh(name, f);
ts = 'Ts.safety';
% inputs: In Bus Element ports hmi, hw_cmd, bms_cmd, env and flt, each element picked right in front of the block
% that uses it (spec 6.2); output: the bus safety
P = @(port, el, dst, varargin) PK(name, port, el, dst, varargin{:});
% ---- loop: Loop_OK = no E-stop / wireless / bumper trip AND MCU_Safety_OK AND key on (S1) ----
LOG(name, 'EStop_Any', 'OR', 5);
hs = {'estop_front_pressed', 'estop_rear_pressed', 'wireless_estop_tripped', 'bumper_front', 'bumper_rear'};
for k = 1:5, P('hmi', hs{k}, sprintf('EStop_Any/%d', k)); end
LOG(name, 'No_trip', 'NOT');  L(name, 'EStop_Any/1', 'No_trip/1', 'EStop_Any');
LOG(name, 'Key_on', 'NOT');   P('hmi', 'key_off', 'Key_on/1');
LOG(name, 'Loop_OK', 'AND', 3);
L(name, 'No_trip/1', 'Loop_OK/1', 'no_trip'); P('hw_cmd', 'MCU_Safety_OK', 'Loop_OK/2');
L(name, 'Key_on/1', 'Loop_OK/3', 'key_on');
% ---- reset: rising(reset button OR MCU DO1) stretched to t_reset_hold (S2, D30) ----
LOG(name, 'Reset_in', 'OR', 2);
P('hmi', 'reset_btn', 'Reset_in/1'); P('hw_cmd', 'Safety_Reset_Req', 'Reset_in/2');
A(name, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Reset_edge', 'vinit', '0');
L(name, 'Reset_in/1', 'Reset_edge/1', 'reset_request');
ONOFF(name, 'Reset_stretch', '0', 'P.power.safety.t_reset_hold', '0', ts);
L(name, 'Reset_edge/1', 'Reset_stretch/1', 'Reset_edge');
DTC(name, 'Reset_pulse', 'boolean'); L(name, 'Reset_stretch/1', 'Reset_pulse/1', 'reset_stretched');
% ---- safety relay latch (S3): Set = Loop_OK AND Reset_pulse, Reset = NOT Loop_OK, reset dominant ----
LOG(name, 'Set', 'AND', 2);
L(name, 'Loop_OK/1', 'Set/1', 'Loop_OK'); L(name, 'Reset_pulse/1', 'Set/2', 'Reset_pulse');
LOG(name, 'Reset', 'NOT'); L(name, 'Loop_OK/1', 'Reset/1', 'Loop_OK');
A(name, 'ee_sl_lib/General Control/Set-Reset Flip-Flop', 'Relay_latch', 'UndefState', 'Reset', ...
    'Q0', 'P.power.safety.Q0', 'Ts', ts);
L(name, 'Set/1', 'Relay_latch/1', 'latch_set'); L(name, 'Reset/1', 'Relay_latch/2', 'latch_reset');
% ---- contacts (S4 immediate, S5 delayed cat-1) ----
ONOFF(name, 'Immediate_contacts', 'P.power.safety.t_relay_on', 'P.power.safety.t_relay_off', 'P.power.safety.Q0', ts);
ONOFF(name, 'Delayed_contacts', 'P.power.safety.t_relay_on', 'P.power.safety.t_cat1', 'P.power.safety.Q0', ts);
L(name, 'Relay_latch/1', 'Immediate_contacts/1', 'Armed'); L(name, 'Relay_latch/1', 'Delayed_contacts/1', 'Armed');
DTC(name, 'Safety_OK_b', 'boolean'); DTC(name, 'Safety_HV_OK_b', 'boolean'); DTC(name, 'Latched_off_b', 'boolean');
L(name, 'Immediate_contacts/1', 'Safety_OK_b/1', 'immediate_contacts');
L(name, 'Delayed_contacts/1', 'Safety_HV_OK_b/1', 'delayed_contacts');
L(name, 'Relay_latch/2', 'Latched_off_b/1', 'latched_off');
% ---- E-stop channels: Ch1 = NOT front AND NOT rear; Ch2 = Ch1 OR estop_ch2_stuck (fault injection) ----
LOG(name, 'EStop_Ch1_OK', 'NOR', 2); HEIGHT([name '/EStop_Ch1_OK'], 90);   % graphical (fix round 1 trial: -1)
P('hmi', 'estop_front_pressed', 'EStop_Ch1_OK/1'); P('hmi', 'estop_rear_pressed', 'EStop_Ch1_OK/2');
LOG(name, 'EStop_Ch2_OK', 'OR', 2);
L(name, 'EStop_Ch1_OK/1', 'EStop_Ch2_OK/1', 'EStop_Ch1_OK'); P('flt', 'estop_ch2_stuck', 'EStop_Ch2_OK/2');
% ---- dock contact (0.1 s -> 1 ms) and coil gating (S6) ----
RT(name, 'Dock_RT', ts); P('env', 'dock_contact', 'Dock_RT/1');
bs = {'K1_Cmd', 'K2_Cmd', 'K4_Cmd'};
for k = 1:3
    RT(name, [bs{k} '_RT'], ts);
    P('bms_cmd', bs{k}, [bs{k} '_RT/1']);
end
LOG(name, 'K1_coil', 'AND', 2); L(name, 'K1_Cmd_RT/1', 'K1_coil/1', 'K1_Cmd_1ms');
L(name, 'Safety_HV_OK_b/1', 'K1_coil/2', 'Safety_HV_OK');
LOG(name, 'K2_coil', 'AND', 2); L(name, 'K2_Cmd_RT/1', 'K2_coil/1', 'K2_Cmd_1ms'); L(name, 'Safety_HV_OK_b/1', 'K2_coil/2');
LOG(name, 'K3_coil', 'AND', 2); P('hw_cmd', 'ImpPwr24_Enable', 'K3_coil/1'); L(name, 'Safety_OK_b/1', 'K3_coil/2', 'Safety_OK');
LOG(name, 'K4_coil', 'AND', 2); L(name, 'K4_Cmd_RT/1', 'K4_coil/1', 'K4_Cmd_1ms'); L(name, 'Dock_RT/1', 'K4_coil/2', 'dock_present');
MUX(name, 'Coil_mux', {'K1_coil/1', 'K2_coil/1', 'K3_coil/1', 'K4_coil/1'});
% ---- output bus safety: the contacts, the E-stop status (formerly the vector safety_vec), the coil word, key, dock
OB(name, 'safety', 'Safety_OK', 'Safety_OK_b/1');
OB(name, 'safety', 'Safety_HV_OK', 'Safety_HV_OK_b/1');
OB(name, 'safety', 'EStop_Any', 'EStop_Any/1');
OB(name, 'safety', 'EStop_Ch1_OK', 'EStop_Ch1_OK/1');
OB(name, 'safety', 'EStop_Ch2_OK', 'EStop_Ch2_OK/1');
OB(name, 'safety', 'Safety_Latched_Off', 'Latched_off_b/1');
OB(name, 'safety', 'coil', 'Coil_mux/1');
OB(name, 'safety', 'key_on', 'Key_on/1');
OB(name, 'safety', 'dock_present', 'Dock_RT/1');
shaddad_bu('portorder', name, {'hmi', 'hw_cmd', 'bms_cmd', 'env', 'flt'}, 'Inport');
local_save(name, f, ['Safety_Chain (T05, spec 2.5 / 3.4, D30), discrete Ts.safety. Inputs (In Bus Element ports, each ' ...
    'element picked where it is used): 1 hmi Bus_HmiInputs, 2 hw_cmd Bus_PowerHwCmd (DO0 MCU_Safety_OK, DO1 ' ...
    'Safety_Reset_Req, DO2 ImpPwr24_Enable), 3 bms_cmd Bus_BMS_Cmd (K1_Cmd, K2_Cmd, K4_Cmd, 0.01 s), 4 env ' ...
    'Bus_EnvState (dock_contact, 0.1 s), 5 flt Bus_FaultInject (estop_ch2_stuck). Loop_OK = no E-stop / wireless / ' ...
    'bumper trip AND MCU_Safety_OK AND key on; Reset_pulse = On-Off Delay(rising(reset_btn OR DO1), 0, t_reset_hold); ' ...
    'SR latch Set = Loop_OK AND Reset_pulse, Reset = NOT Loop_OK (reset dominant, Q0); Safety_OK = On-Off Delay(t_relay_on, ' ...
    't_relay_off), Safety_HV_OK = On-Off Delay(t_relay_on, t_cat1). Output bus safety (Out Bus Element, all Ts.safety): ' ...
    'Safety_OK, Safety_HV_OK, EStop_Any, EStop_Ch1_OK, EStop_Ch2_OK, Safety_Latched_Off, coil boolean[4] = [K1_Cmd AND ' ...
    'Safety_HV_OK, K2_Cmd AND Safety_HV_OK, ImpPwr24_Enable AND Safety_OK, K4_Cmd AND dock], key_on, dock_present.']);
end

% ===========================================================================================
%  PDU (spec 2.4 / 3.3 D1-D23)
% ===========================================================================================
function local_pdu(name, f)
local_fresh(name, f);
pd = 'P.power.pdu';
cps = {'pack_p', 'Left'; 'pack_n', 'Left'; 'DC48_TracL_p', 'Right'; 'DC48_TracL_n', 'Right'; 'DC48_TracR_p', 'Right'; ...
       'DC48_TracR_n', 'Right'; 'IMP48_p', 'Right'; 'IMP48_n', 'Right'; 'AUX48_p', 'Right'; 'AUX48_n', 'Right'; ...
       'DOCK_p', 'Right'; 'DOCK_n', 'Right'};
H = struct();
for k = 1:size(cps, 1)
    b = CP(name, cps{k, 1}, k, 'E', cps{k, 2});
    H.(cps{k, 1}) = CPH(b);
end
% ---- elements ----
R = @(n, r) PH(A(name, LIB('res'), n, 'R', r));
FUSE = @(n) PH(A(name, LIB('fuse'), n, 'prm', '3', 'I_vec', [pd '.' n '.I_vec'], 't_vec', [pd '.' n '.t_vec'], ...
    'R_closed', [pd '.' n '.R_closed'], 'G_open', [pd '.' n '.G_open'], ...
    'exposemeasurement', 'ee.enum.exposemeasurement.visible'));
RLY = @(n) PH(A(name, LIB('relay'), n, 'block_choices', 'ee.enum.controlport.ps', 'R_closed', [pd '.' n '.R_closed'], ...
    'G_open', [pd '.' n '.G_open'], 'Threshold', '0.5', 'T_brk_C_S', '0', 'T_mak_C_S', '0'));
pRcp = R('R_cab_pack', [pd '.R_cab_pack']);
pFM = FUSE('F_MAIN');
pK1 = RLY('K1'); pK2 = RLY('K2'); pK4 = RLY('K4');
pRpre = R('R_pre', [pd '.R_pre']);
pC = PH(A(name, LIB('cap'), 'C_PDU', 'c', [pd '.C_pdu'], 'r', [pd '.ESR_pdu'], 'g', [pd '.g_bleed'], ...
    'vc_specify', 'on', 'vc_priority', 'High', 'vc', '0'));
pFTL = FUSE('F_TL'); pRTL = R('R_cab_TL', [pd '.R_cab_trac']);
pFTR = FUSE('F_TR'); pRTR = R('R_cab_TR', [pd '.R_cab_trac']);
pFI = FUSE('F_IMP'); pFA = FUSE('F_AUX'); pFC = FUSE('F_CHG');
pKey = PH(A(name, LIB('switch'), 'S_KEY', 'R_closed', [pd '.R_key'], 'G_open', '1e-8', 'Threshold', '0.5'));
pSh = R('R_SHUNT', [pd '.R_shunt']);
pBms = PH(A(name, 'ee_lib/Passive/Constant Power Load', 'BMS_supply', 'load_type', 'ee.enum.dc_ac.dc', ...
    'consumed_power', [pd '.P_bms_supply'], 'min_supply_voltage', '20'));
pG = PH(A(name, LIB('gnd'), 'GND'));
gnd = pG.LConn(1);
% ---- current meters (+ -> -) ----
[iPack, iPp, iPn] = ISENS(name, 'I_pack_m');      % GND -> shunt: + = discharge
[iTL, iTLp, iTLn] = ISENS(name, 'I_TL_m');
[iTR, iTRp, iTRn] = ISENS(name, 'I_TR_m');
[iIMP, iIp, iIn] = ISENS(name, 'I_IMP_m');
[iAUX, iAp, iAn] = ISENS(name, 'I_AUX_m');
[iDOCK, iDp, iDn] = ISENS(name, 'I_DOCK_m');      % DOCK_p -> F_CHG: + = charger current into the bus
[iBMS, iBp, iBn] = ISENS(name, 'I_BMS_m');
% ---- topology (spec 2.4 single-line) ----
P = @(a, b) PL(name, a, b);
P(H.pack_p, pRcp.LConn(1)); P(pRcp.RConn(1), pFM.LConn(1));
batt = pFM.RConn(1);                                            % BATT_P
P(batt, pK1.RConn(1)); P(batt, pK2.RConn(1));
P(pK2.RConn(2), pRpre.LConn(1));
dcbus = pK1.RConn(2);                                           % DCBUS_P
P(pRpre.RConn(1), dcbus);
P(dcbus, pC.LConn(1)); P(pC.RConn(1), gnd);
P(dcbus, pFTL.LConn(1)); P(pFTL.RConn(1), pRTL.LConn(1)); P(pRTL.RConn(1), iTLp); P(iTLn, H.DC48_TracL_p);
P(dcbus, pFTR.LConn(1)); P(pFTR.RConn(1), pRTR.LConn(1)); P(pRTR.RConn(1), iTRp); P(iTRn, H.DC48_TracR_p);
P(dcbus, pFI.LConn(1)); P(pFI.RConn(1), iIp); P(iIn, H.IMP48_p);
P(batt, pKey.LConn(2)); P(pKey.RConn(1), pFA.LConn(1)); P(pFA.RConn(1), iAp); P(iAn, H.AUX48_p);
P(batt, pK4.RConn(1)); P(pK4.RConn(2), pFC.LConn(1)); P(pFC.RConn(1), iDn); P(iDp, H.DOCK_p);
P(batt, iBp); P(iBn, pBms.LConn(1)); P(pBms.RConn(1), gnd);
P(H.pack_n, pSh.LConn(1)); P(pSh.RConn(1), iPn); P(iPp, gnd);
for n = {'DC48_TracL_n', 'DC48_TracR_n', 'IMP48_n', 'AUX48_n', 'DOCK_n'}
    P(H.(n{1}), gnd);
end
% ---- voltage meters ----
vPack = VSENS(name, 'V_pack_m', H.pack_p, H.pack_n);
vBus  = VSENS(name, 'V_bus_m', dcbus, gnd);
vDock = VSENS(name, 'V_dock_m', H.DOCK_p, gnd);
vTL   = VSENS(name, 'V_TL_m', H.DC48_TracL_p, gnd);
vTR   = VSENS(name, 'V_TR_m', H.DC48_TracR_p, gnd);
vIMP  = VSENS(name, 'V_IMP_m', H.IMP48_p, gnd);
vAUX  = VSENS(name, 'V_AUX_m', H.AUX48_p, gnd);
vBatt = VSENS(name, 'V_BATT_m', batt, gnd);
% ---- coils (input bus safety): On-Off Delay (pickup / dropout, D21) -> aux contact + Simulink-PS -> relay (D22) ----
A(name, 'simulink/Signal Routing/Demux', 'Coil_dmx', 'Outputs', '4');
PK(name, 'safety', 'coil', 'Coil_dmx/1');
A(name, 'simulink/Sinks/Terminator', 'K3_here'); L(name, 'Coil_dmx/3', 'K3_here/1', 'K3_coil');
kk = {'K1', 1, pK1; 'K2', 2, pK2; 'K4', 4, pK4};
for k = 1:size(kk, 1)
    c = kk{k, 1};
    ONOFF(name, [c '_delay'], [pd '.' c '.t_on'], [pd '.' c '.t_off'], '0', 'Ts.base');
    L(name, sprintf('Coil_dmx/%d', kk{k, 2}), [c '_delay/1'], [c '_coil']);
    DTC(name, [c '_fb'], 'boolean'); L(name, [c '_delay/1'], [c '_fb/1'], [c '_contact']);
    A(name, 'simulink/Signal Attributes/Data Type Conversion', [c '_dbl'], 'OutDataTypeStr', 'double');
    L(name, [c '_fb/1'], [c '_dbl/1'], [c '_closed']);
    h = SPS(name, [c '_PS'], [c '_dbl/1'], '1', [c '_closed_dbl']);
    pr = kk{k, 3}; PL(name, h, pr.LConn(1));
end
% key switch
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Key_dbl', 'OutDataTypeStr', 'double');
PK(name, 'safety', 'key_on', 'Key_dbl/1');
h = SPS(name, 'Key_PS', 'Key_dbl/1', 'V', 'key_on_dbl'); PL(name, h, pKey.LConn(1));
% ---- fuse states (exposed measurement: 1 = blown) ----
fz = {'F_MAIN', pFM; 'F_TL', pFTL; 'F_TR', pFTR; 'F_IMP', pFI; 'F_AUX', pFA; 'F_CHG', pFC};
fs = cell(1, size(fz, 1));
for k = 1:size(fz, 1)
    pf = fz{k, 2};
    o = PSS(name, [fz{k, 1} '_state'], pf.RConn(2), '1');
    CMP(name, [fz{k, 1} '_ok'], '<', '0.5'); L(name, o, [fz{k, 1} '_ok/1']);
    fs{k} = [fz{k, 1} '_ok/1'];
end
MUX(name, 'Fuse_mux', fs);
% ---- port powers and PDU loss (= pack terminal power + charger power in - all port outputs) ----
PROD(name, 'P_pack', vPack, iPack);
PROD(name, 'P_TL', vTL, iTL); PROD(name, 'P_TR', vTR, iTR); PROD(name, 'P_IMP48', vIMP, iIMP);
PROD(name, 'P_AUX', vAUX, iAUX); PROD(name, 'P_DOCK', vDock, iDOCK); PROD(name, 'P_BMS', vBatt, iBMS);
% P_BMS taller (graphical), so its meter lines come in level (fix round 1 trial: 343 -> 331 crossings, with P_TR at
% 120 px; fix round 3, with the riser order: P_TR at its own height again, 327 -> 324, t6_fix3_sizes.log)
HEIGHT([name '/P_BMS'], 60);
SUMS(name, 'P_loss', '++-----', {'P_pack/1', 'P_DOCK/1', 'P_TL/1', 'P_TR/1', 'P_IMP48/1', 'P_AUX/1', 'P_BMS/1'});
% ---- output bus pdu (Power_Sensors picks its elements) ----
OB(name, 'pdu', 'V_pack', vPack); OB(name, 'pdu', 'V_bus', vBus); OB(name, 'pdu', 'V_dock', vDock);
OB(name, 'pdu', 'I_pack', iPack);
OB(name, 'pdu', 'K1_closed', 'K1_fb/1'); OB(name, 'pdu', 'K2_closed', 'K2_fb/1'); OB(name, 'pdu', 'K4_closed', 'K4_fb/1');
OB(name, 'pdu', 'fuse_ok', 'Fuse_mux/1'); OB(name, 'pdu', 'F_MAIN_ok', 'F_MAIN_ok/1');
OB(name, 'pdu', 'P_trac_L', 'P_TL/1'); OB(name, 'pdu', 'P_trac_R', 'P_TR/1');
OB(name, 'pdu', 'P_chg_bus', 'P_DOCK/1'); OB(name, 'pdu', 'P_pdu_loss', 'P_loss/1'); OB(name, 'pdu', 'P_bms', 'P_BMS/1');
local_save(name, f, ['PDU (T05, spec 2.4 / 3.3 D1-D23). Contains the ONLY Electrical Reference of the DC48 network (GND ' ...
    'after the shunt). Input bus safety (In Bus Element): coil boolean[4] [K1 K2 K3 K4] (Ts.safety; K3 is in ' ...
    'LV_Distribution), key_on boolean. Connection ports pack_p/pack_n (Battery_Pack), DC48_TracL/R_p/n (DRIVE), ' ...
    'IMP48_p/n (implement), AUX48_p/n (LV_Distribution, tapped after F_MAIN and the key switch, before K1), DOCK_p/n ' ...
    '(Dock_Charger, after K4 + F_CHG). Coils through On-Off Delay (P.power.pdu.K*.t_on/t_off) = aux contacts. Output ' ...
    'bus pdu (Out Bus Element, Ts.base): V_pack, V_bus, V_dock (V), I_pack (A, + discharge), K1_closed, K2_closed, ' ...
    'K4_closed (aux contacts), fuse_ok boolean[6] [MAIN TL TR IMP AUX CHG], F_MAIN_ok, P_trac_L, P_trac_R (W into the ' ...
    'DRIVE ports), P_chg_bus (W from DOCK_p into the bus), P_pdu_loss (W = pack terminal power + charger power - port ' ...
    'powers: fuses, shunt, contactors, cables, precharge R, C_PDU bleed), P_bms (BMS supply, W).']);
end

% ===========================================================================================
%  LV_Distribution (spec 2.6 / 3.5 L1-L8)
% ===========================================================================================
function local_lv(name, f)
local_fresh(name, f);
lv = 'P.power.lv';
IN(name, 'lv_demand', 1, 'Bus: Bus_LV_LoadDemand', 'Ts.ctrl');
H.AUX48_p = CPH(CP(name, 'AUX48_p', 1, 'E', 'Left'));
H.AUX48_n = CPH(CP(name, 'AUX48_n', 2, 'E', 'Left'));
H.V24IMP_p = CPH(CP(name, 'V24IMP_p', 3, 'E', 'Right'));
H.V24IMP_n = CPH(CP(name, 'V24IMP_n', 4, 'E', 'Right'));
gnd = H.AUX48_n;
P = @(a, b) PL(name, a, b);
DC = @(n, s) PH(A(name, LIB('dcdc'), n, 'Vref_option', 'ee.enum.converters.voltageref.internal', ...
    'v_ref', [lv '.' s '.v_ref'], 'P_rated', [lv '.' s '.P_rated'], 'droop_enabled', 'false', ...
    'reversible', 'ee.enum.converters.directionality.unidirectional', ...
    'losses_option', 'ee.enum.converters.lossesOption.proportionalToSquaredCurrent', ...
    'P_fixed', [lv '.' s '.P_fixed'], 'eff_rated', [lv '.' s '.eff_rated'], ...
    'model_dynamics', 'ee.enum.converters.dynamics.yes', 'tau', [lv '.' s '.tau'], 'v_ini', '0', ...
    'v1_min', [lv '.' s '.v1_min'], 'R1', [lv '.' s '.R1'], 'R2', [lv '.' s '.R2']));
DL = @(n, vnom) PH(A(name, LIB('dload'), n, 'load_type', 'ee.enum.dc_ac.dc', ...
    'min_supply_voltage', [lv '.load.min_supply_frac*' vnom], 'pow_min', [lv '.load.pow_min'], ...
    'G_reverse', [lv '.load.G_reverse'], 'deltaV', [lv '.load.deltaV']));
p24 = DC('DCDC_24', 'dcdc24'); p12 = DC('DCDC_12', 'dcdc12'); p5 = DC('DCDC_5', 'dcdc5');
l24 = DL('LOAD_24AUX', '24'); l12 = DL('LOAD_12', '12'); l5 = DL('LOAD_5', '5');
ldo = PH(A(name, LIB('ccs'), 'LDO_3V3'));
k3 = PH(A(name, LIB('relay'), 'K3', 'block_choices', 'ee.enum.controlport.ps', 'R_closed', 'P.power.pdu.K3.R_closed', ...
    'G_open', 'P.power.pdu.K3.G_open', 'Threshold', '0.5', 'T_brk_C_S', '0', 'T_mak_C_S', '0'));
f24 = PH(A(name, LIB('fuse'), 'F_24IMP', 'prm', '3', 'I_vec', 'P.power.pdu.F_24IMP.I_vec', 't_vec', 'P.power.pdu.F_24IMP.t_vec', ...
    'R_closed', 'P.power.pdu.F_24IMP.R_closed', 'G_open', 'P.power.pdu.F_24IMP.G_open', ...
    'exposemeasurement', 'ee.enum.exposemeasurement.visible'));
[iIn24, a1, b1] = ISENS(name, 'I_in24_m');  [iIn12, a2, b2] = ISENS(name, 'I_in12_m');
[iO24, a3, b3] = ISENS(name, 'I_out24_m');  [iO12, a4, b4] = ISENS(name, 'I_out12_m');
[iIn5, a5, b5] = ISENS(name, 'I_in5_m');    [iO5, a6, b6] = ISENS(name, 'I_out5_m');
[iK3, a7, b7] = ISENS(name, 'I_k3_m');
% 48 V inputs (common negative = GND, non-isolated)
P(H.AUX48_p, a1); P(b1, p24.LConn(1)); P(p24.LConn(2), gnd); P(p24.RConn(2), gnd);
P(H.AUX48_p, a2); P(b2, p12.LConn(1)); P(p12.LConn(2), gnd); P(p12.RConn(2), gnd);
% 24 V rail: converter -> meter -> V24; aux load, K3 branch to V24_IMP
P(p24.RConn(1), a3); v24 = b3;
P(v24, l24.LConn(1)); P(l24.RConn(1), gnd);
P(v24, a7); P(b7, k3.RConn(1)); P(k3.RConn(2), f24.LConn(1)); P(f24.RConn(1), H.V24IMP_p);
P(H.V24IMP_n, gnd);
% 12 V rail: load, 5 V converter input
P(p12.RConn(1), a4); v12 = b4;
P(v12, l12.LConn(1)); P(l12.RConn(1), gnd);
P(v12, a5); P(b5, p5.LConn(1)); P(p5.LConn(2), gnd); P(p5.RConn(2), gnd);
% 5 V rail: load + 3.3 V LDO current sink (tail on V5, head on GND)
P(p5.RConn(1), a6); v5 = b6;
P(v5, l5.LConn(1)); P(l5.RConn(1), gnd);
P(ldo.RConn(2), v5); P(ldo.LConn(1), gnd);
vAux = VSENS(name, 'V_aux_m', H.AUX48_p, gnd);
v24o = VSENS(name, 'V24_m', v24, gnd);
v12o = VSENS(name, 'V12_m', v12, gnd);
v5o  = VSENS(name, 'V5_m', v5, gnd);
vImp = VSENS(name, 'V24IMP_m', H.V24IMP_p, gnd);
% ---- demands (0.01 s -> Ts.base): P24 = P_24V_W + P_fan*Fan_eff; P12 = P_12V_W + coil holds; P5; LDO ----
RT(name, 'Demand_RT', 'Ts.base'); L(name, 'lv_demand/1', 'Demand_RT/1', 'lv_demand');
dm = {'P24_sel', 'P_24V_W'; 'P12_sel', 'P_12V_W'; 'P5_sel', 'P_5V_W'; 'P3V3_sel', 'P_3V3_W'};   % a pick per consumer
for k = 1:size(dm, 1)
    BS(name, dm{k, 1}, dm(k, 2)); L(name, 'Demand_RT/1', [dm{k, 1} '/1'], 'lv_demand_1ms');
end
A(name, 'simulink/Math Operations/Gain', 'P_fan', 'Gain', 'P.power.thermal.P_fan');
PK(name, 'thermal', 'Fan_eff', 'P_fan/1');
SUMS(name, 'P24_dem', '++', {'P24_sel/1', 'P_fan/1'});
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Coil_dbl', 'OutDataTypeStr', 'double');
PK(name, 'safety', 'coil', 'Coil_dbl/1');
A(name, 'simulink/Math Operations/Gain', 'Coil_hold', 'Gain', [lv '.coil_hold_vec'], 'Multiplication', 'Matrix(K*u)');
L(name, 'Coil_dbl/1', 'Coil_hold/1', 'coil_dbl');
SUMS(name, 'P12_dem', '++', {'P12_sel/1', 'Coil_hold/1'});
A(name, 'simulink/Math Operations/Gain', 'I_ldo_k', 'Gain', ['1/' lv '.ldo33.V_out']);
A(name, 'simulink/Math Operations/Bias', 'I_ldo', 'Bias', [lv '.ldo33.Iq']);
L(name, 'P3V3_sel/1', 'I_ldo_k/1'); L(name, 'I_ldo_k/1', 'I_ldo/1', 'I_ldo_load');
h = SPS(name, 'P24_PS', 'P24_dem/1', 'W', 'P24'); PL(name, h, l24.LConn(2));
h = SPS(name, 'P12_PS', 'P12_dem/1', 'W', 'P12'); PL(name, h, l12.LConn(2));
h = SPS(name, 'P5_PS', 'P5_sel/1', 'W'); PL(name, h, l5.LConn(2));
% LDO input current only while the 5 V rail is up: x sat(V5(k-1) / (V_out + V_dropout), 0, 1); the rail
% voltage feedback passes a 1 ms Unit Delay (D36)
A(name, 'simulink/Discrete/Unit Delay', 'V5_d', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
L(name, v5o, 'V5_d/1', 'V_5V');
A(name, 'simulink/Math Operations/Gain', 'Ldo_gate_k', 'Gain', ['1/(' lv '.ldo33.V_out + ' lv '.ldo33.V_dropout)']);
A(name, 'simulink/Discontinuities/Saturation', 'Ldo_gate', 'UpperLimit', '1', 'LowerLimit', '0');
L(name, 'V5_d/1', 'Ldo_gate_k/1', 'V_5V_d'); L(name, 'Ldo_gate_k/1', 'Ldo_gate/1', 'ldo_gate_raw');
PROD(name, 'I_ldo_g', 'I_ldo/1', 'Ldo_gate/1'); HEIGHT([name '/I_ldo_g'], 80);   % graphical (fix round 1 trial)
h = SPS(name, 'I_ldo_PS', 'I_ldo_g/1', 'A', 'I_ldo'); PL(name, h, ldo.RConn(1));
% ---- K3 relay (coil = ImpPwr24_Enable AND Safety_OK from Safety_Chain), On-Off Delay pickup / dropout ----
A(name, 'simulink/Signal Routing/Demux', 'Coil_dmx', 'Outputs', '4');
PK(name, 'safety', 'coil', 'Coil_dmx/1');
for k = [1 2 4]
    A(name, 'simulink/Sinks/Terminator', sprintf('coil%d_term', k));
    L(name, sprintf('Coil_dmx/%d', k), sprintf('coil%d_term/1', k), sprintf('K%d_coil', k));
end
ONOFF(name, 'K3_delay', 'P.power.pdu.K3.t_on', 'P.power.pdu.K3.t_off', '0', 'Ts.base');
L(name, 'Coil_dmx/3', 'K3_delay/1', 'K3_coil');
DTC(name, 'K3_fb', 'boolean'); L(name, 'K3_delay/1', 'K3_fb/1', 'K3_contact');
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'K3_dbl', 'OutDataTypeStr', 'double');
L(name, 'K3_fb/1', 'K3_dbl/1', 'K3_closed');
h = SPS(name, 'K3_PS', 'K3_dbl/1', '1', 'K3_closed_dbl'); PL(name, h, k3.LConn(1));
o = PSS(name, 'F_24IMP_state', f24.RConn(2), '1');
CMP(name, 'F_24IMP_ok', '<', '0.5'); L(name, o, 'F_24IMP_ok/1');
% ---- 3.3 V status (L8) = max(min(3.3, V5 - V_dropout), 0) ----
A(name, 'simulink/Math Operations/Bias', 'V5_drop', 'Bias', ['-' lv '.ldo33.V_dropout']);
L(name, v5o, 'V5_drop/1');
CST(name, 'V3V3_out', [lv '.ldo33.V_out'], '', 'Ts.base');
A(name, 'simulink/Math Operations/MinMax', 'V3V3_min', 'Function', 'min', 'Inputs', '2');
HEIGHT([name '/V3V3_min'], 60);        % graphical (fix round 1 trial, with I_ldo_g: 285 -> 276 crossings)
L(name, 'V3V3_out/1', 'V3V3_min/1'); L(name, 'V5_drop/1', 'V3V3_min/2');
A(name, 'simulink/Discontinuities/Saturation', 'V3V3', 'UpperLimit', 'inf', 'LowerLimit', '0');
L(name, 'V3V3_min/1', 'V3V3/1');
MUX(name, 'Rails_mux', {v24o, iO24, vImp, v12o, iO12, v5o, 'V3V3/1'});
% rail OK flags: abs(V - V_nom) <= tol * V_nom
rv = {v24o, 1; v12o, 2; v5o, 3; 'V3V3/1', 4};
ok = cell(1, 4);
for k = 1:4
    j = rv{k, 2};
    A(name, 'simulink/Math Operations/Bias', sprintf('Rail%d_dev', j), 'Bias', sprintf('-%s.V_nom(%d)', lv, j));
    A(name, 'simulink/Math Operations/Abs', sprintf('Rail%d_abs', j));
    CMP(name, sprintf('Rail%d_ok', j), '<=', sprintf('%s.rail_ok_tol*%s.V_nom(%d)', lv, lv, j));
    L(name, rv{k, 1}, sprintf('Rail%d_dev/1', j));
    L(name, sprintf('Rail%d_dev/1', j), sprintf('Rail%d_abs/1', j));
    L(name, sprintf('Rail%d_abs/1', j), sprintf('Rail%d_ok/1', j));
    ok{k} = sprintf('Rail%d_ok/1', j);
end
MUX(name, 'Rail_ok_mux', ok);
% ---- powers: rail loads, converter losses, K3 / F_24IMP branch loss ----
PROD(name, 'p_in24', vAux, iIn24); PROD(name, 'p_in12', vAux, iIn12);
PROD(name, 'p_out24', v24o, iO24); PROD(name, 'p_out12', v12o, iO12);
PROD(name, 'p_in5', v12o, iIn5);  PROD(name, 'p_out5', v5o, iO5);
PROD(name, 'p_k3', v24o, iK3);    PROD(name, 'p_v24imp', vImp, iK3);
% two products taller (graphical), so their meter lines come in level (fix round 1 trial: 294 -> 285 crossings)
HEIGHT([name '/p_in5'], 120); HEIGHT([name '/p_v24imp'], 60);
% the eight powers travel as one virtual bus (spec 6.3); each sum and the converter Mux picks its operands right in
% front of it, in its own operand order. The four pick by name, so the bus's input order is free (ruling F2-R1); a
% search over 113 orders found none with fewer crossings than this one (fix round 2, t6_fix2_bcsearch.log)
pw = {'p_in24', 'p_in12', 'p_out24', 'p_out12', 'p_in5', 'p_out5', 'p_k3', 'p_v24imp'};
BCV(name, 'Pow_BC', [pw; strcat(pw, '/1')]');
SUMR(name, 'P_lv_sum', '+-+-+');
SEL(name, 'P_lv_sel', 'Pow_BC/1', 'rail_powers', {'p_out24', 'p_k3', 'p_out12', 'p_in5', 'p_out5'}, 'P_lv_sum');
SUMR(name, 'P_dcdc_sum', '++--+-');
SEL(name, 'P_dcdc_sel', 'Pow_BC/1', 'rail_powers', {'p_in24', 'p_in12', 'p_out24', 'p_out12', 'p_in5', 'p_out5'}, ...
    'P_dcdc_sum');
SUMR(name, 'P_k3_sum', '+-');
SEL(name, 'P_k3_sel', 'Pow_BC/1', 'rail_powers', {'p_k3', 'p_v24imp'}, 'P_k3_sum');
A(name, 'simulink/Signal Routing/Mux', 'P_conv_mux', 'Inputs', '6');
SEL(name, 'P_conv_sel', 'Pow_BC/1', 'rail_powers', {'p_in24', 'p_out24', 'p_in12', 'p_out12', 'p_in5', 'p_out5'}, ...
    'P_conv_mux');
% ---- output bus lv (Power_Sensors picks its elements) ----
OB(name, 'lv', 'rails', 'Rails_mux/1'); OB(name, 'lv', 'rail_ok', 'Rail_ok_mux/1');
OB(name, 'lv', 'K3_closed', 'K3_fb/1'); OB(name, 'lv', 'fuse24_ok', 'F_24IMP_ok/1');
OB(name, 'lv', 'P_lv', 'P_lv_sum/1'); OB(name, 'lv', 'P_dcdc_loss', 'P_dcdc_sum/1');
OB(name, 'lv', 'P_k3_loss', 'P_k3_sum/1'); OB(name, 'lv', 'P_conv', 'P_conv_mux/1');
shaddad_bu('portorder', name, {'lv_demand', 'thermal', 'safety'}, 'Inport');
local_save(name, f, ['LV_Distribution (T05, spec 2.6 / 3.5). AUX48_p/n in (Left), V24IMP_p/n out (Right, K3 + F_24IMP). ' ...
    'ee_lib DC-DC Converter 48->24 V / 48->12 V / 12->5 V (P.power.lv.dcdc*, UVLO v1_min), Dynamic Loads P24 = ' ...
    'P_24V_W + P_fan*Fan_eff, P12 = P_12V_W + energised coil holds, P5 = P_5V_W, 3.3 V LDO = current sink P_3V3_W/3.3 + Iq. ' ...
    'Inputs: 1 lv_demand Bus_LV_LoadDemand (0.01 s), 2 bus thermal (In Bus Element: Fan_eff 0..1, Ts.base, ' ...
    'Pack_Thermal), 3 bus safety (In Bus Element: coil boolean[4] [K1 K2 K3 K4], Ts.safety). Output bus lv (Out Bus ' ...
    'Element, Ts.base): rails[7] = [V_24V I_24V V_24IMP V_12V I_12V V_5V V_3V3], rail_ok boolean[4] (+/-10 % of ' ...
    'P.power.lv.V_nom), K3_closed, fuse24_ok, P_lv (rail loads incl. LDO, coil holds, fans), P_dcdc_loss, P_k3_loss ' ...
    '(K3 + F_24IMP drop) in W, P_conv[6] = [P_in P_out] of the 24 / 12 / 5 V converters (W, diagnostics).']);
end

% ===========================================================================================
%  impl_ecu (implement ECU, masked: drv, impl_type)
% ===========================================================================================
function local_ecu(name, f)
local_fresh(name, f);
ts = 'Ts.impl_ctrl';
ecu = 'P.power.impl.ecu';
IN(name, 'can_rx', 1, 'Bus: Bus_Can1_McuSlow', 'Ts.can_slow');
IN(name, 'Safety_OK', 2, 'boolean', 'Ts.safety');
IN(name, 'w_motor', 3, 'double', 'Ts.base');
IN(name, 'V_dc', 4, 'double', 'Ts.base');
IN(name, 'I_dc', 5, 'double', 'Ts.base');
IN(name, 'P_in', 6, 'double', 'Ts.base');
OUT(name, 'T_ref', 1, 'double'); OUT(name, 'impl_elec', 2, 'Bus: Bus_ImplElecState'); OUT(name, 'can_tx', 3, 'Bus: Bus_Can1_Impl');
% ---- IMPL_CMD decode (0.05 s) and transfer to the 10 ms ECU task ----
BS(name, 'Rx_sel', {'impl_cmd'}); L(name, 'can_rx/1', 'Rx_sel/1', 'can_rx');
A(name, LIB('ssr'), 'Cmd_unpack', 'ReferencedSubsystem', 'implcmd_can_unpack');
L(name, 'Rx_sel/1', 'Cmd_unpack/1');
A(name, 'simulink/Sinks/Terminator', 'Fault_reset_unused'); L(name, 'Cmd_unpack/2', 'Fault_reset_unused/1', 'fault_reset');
rx = {'enable', 1; 'alive', 3; 'speed_rpm', 4; 'valid', 5};
for k = 1:size(rx, 1)
    RT(name, ['RT_' rx{k, 1}], ts);
    L(name, sprintf('Cmd_unpack/%d', rx{k, 2}), ['RT_' rx{k, 1} '/1'], rx{k, 1});
end
for s = {'Safety_OK', 'w_motor', 'V_dc', 'I_dc', 'P_in'}
    RT(name, ['RT_' s{1}], ts); L(name, [s{1} '/1'], ['RT_' s{1} '/1'], s{1});
end
% ---- CAN timeout: age since the last fresh frame (new alive value), reset on reception ----
A(name, 'simulink/Logic and Bit Operations/Detect Change', 'Alive_change', 'vinit', '0');
L(name, 'RT_alive/1', 'Alive_change/1', 'alive_10ms');
LOG(name, 'New_frame', 'AND', 2); L(name, 'Alive_change/1', 'New_frame/1', 'alive_changed');
L(name, 'RT_valid/1', 'New_frame/2', 'valid_10ms');
CST(name, 'One', '1', '', ts);
A(name, 'simulink/Discrete/Discrete-Time Integrator', 'Cmd_age', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1', 'ExternalReset', 'level', 'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', [ecu '.age_max'], 'LowerSaturationLimit', '0', 'SampleTime', ts);
L(name, 'One/1', 'Cmd_age/1', 'one'); L(name, 'New_frame/1', 'Cmd_age/2', 'new_frame');
CMP(name, 'Timeout', '>=', [ecu '.t_can_timeout - 1e-9']); L(name, 'Cmd_age/1', 'Timeout/1', 'cmd_age');
LOG(name, 'Not_timeout', 'NOT'); L(name, 'Timeout/1', 'Not_timeout/1', 'can_timeout');
LOG(name, 'Enable_eff', 'AND', 3);
L(name, 'RT_enable/1', 'Enable_eff/1', 'enable_10ms'); L(name, 'Not_timeout/1', 'Enable_eff/2', 'can_ok');
L(name, 'RT_Safety_OK/1', 'Enable_eff/3', 'Safety_OK_10ms');
% ---- speed reference: rpm -> rad/s at Impl_Shaft, 0 when not enabled / timed out / Safety_OK false; ramp ----
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Spd_dbl', 'OutDataTypeStr', 'double');
A(name, 'simulink/Math Operations/Gain', 'Spd_rad', 'Gain', 'P.power.impl.rpm2rad');
L(name, 'RT_speed_rpm/1', 'Spd_dbl/1', 'speed_rpm_10ms'); L(name, 'Spd_dbl/1', 'Spd_rad/1', 'speed_rpm_dbl');
CST(name, 'Zero', '0', '', ts);
A(name, 'simulink/Signal Routing/Switch', 'Target', 'Criteria', 'u2 ~= 0');
L(name, 'Spd_rad/1', 'Target/1', 'w_cmd'); L(name, 'Enable_eff/1', 'Target/2', 'enable_eff'); L(name, 'Zero/1', 'Target/3', 'zero');
A(name, 'simulink/Discontinuities/Rate Limiter', 'Ramp', 'RisingSlewLimit', 'drv.ramp_up', ...
    'FallingSlewLimit', 'drv.ramp_dn', 'SampleTimeMode', 'inherited', 'InitialCondition', '0');
L(name, 'Target/1', 'Ramp/1', 'w_target');
A(name, 'simulink/Math Operations/Gain', 'Ratio', 'Gain', 'drv.ratio');
L(name, 'Ramp/1', 'Ratio/1', 'w_ref_shaft');
for lg = {'Target', 'ecu_w_target'; 'Enable_eff', 'ecu_enable'}'
    ph = get_param([name '/' lg{1}], 'PortHandles');
    set_param(ph.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', lg{2});
end
A(name, 'simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
L(name, 'Ratio/1', 'Err/1', 'w_ref_motor'); L(name, 'RT_w_motor/1', 'Err/2', 'w_motor_10ms');
% drive active while enabled or still spinning (regenerative braking to standstill after a disable / E-stop);
% integrator reset on the disable edge (spec I11) and held at 0 while the drive is inactive (no creep)
LOG(name, 'Disable', 'NOT'); L(name, 'Enable_eff/1', 'Disable/1');
A(name, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Disable_edge', 'vinit', '0');
L(name, 'Disable/1', 'Disable_edge/1', 'disable');
A(name, 'simulink/Math Operations/Abs', 'W_abs_ctl'); L(name, 'RT_w_motor/1', 'W_abs_ctl/1');
CMP(name, 'Moving', '>', [ecu '.run_frac*drv.w_nom*drv.ratio']); L(name, 'W_abs_ctl/1', 'Moving/1', 'w_motor_abs');
LOG(name, 'Drive_active', 'OR', 2); HEIGHT([name '/Drive_active'], 120);   % graphical (fix round 1 trial: -10)
L(name, 'Enable_eff/1', 'Drive_active/1'); L(name, 'Moving/1', 'Drive_active/2', 'moving');
LOG(name, 'Inactive', 'NOT'); L(name, 'Drive_active/1', 'Inactive/1', 'drive_active');
LOG(name, 'PI_reset', 'OR', 2); L(name, 'Disable_edge/1', 'PI_reset/1', 'disable_edge');
L(name, 'Inactive/1', 'PI_reset/2', 'inactive');
A(name, 'simulink/Discrete/Discrete PID Controller', 'Speed_PI', 'Controller', 'PI', 'P', 'drv.Kp', 'I', 'drv.Ki', ...
    'SampleTime', ts, 'IntegratorMethod', 'Forward Euler', 'LimitOutput', 'on', 'UpperSaturationLimit', 'drv.T_max', ...
    'LowerSaturationLimit', '-drv.T_brake', 'AntiWindupMode', 'clamping', 'ExternalReset', 'level');
L(name, 'Err/1', 'Speed_PI/1', 'w_err'); L(name, 'PI_reset/1', 'Speed_PI/2', 'pi_reset');
A(name, 'simulink/Signal Routing/Switch', 'T_active', 'Criteria', 'u2 ~= 0');
L(name, 'Speed_PI/1', 'T_active/1', 'T_pi'); L(name, 'Drive_active/1', 'T_active/2'); L(name, 'Zero/1', 'T_active/3');
% DC-link over-voltage protection: no regenerative (braking) torque while V_dc > V_ov (an isolated bus, K1 open,
% would otherwise be pumped up by the braking energy)
CMP(name, 'OV_lim', '>', [ecu '.V_ov']); L(name, 'RT_V_dc/1', 'OV_lim/1', 'V_dc_10ms');
A(name, 'simulink/Discontinuities/Saturation', 'T_motoring', 'UpperLimit', 'inf', 'LowerLimit', '0');
L(name, 'T_active/1', 'T_motoring/1', 'T_active');
A(name, 'simulink/Signal Routing/Switch', 'T_ovp', 'Criteria', 'u2 ~= 0');
L(name, 'T_motoring/1', 'T_ovp/1', 'T_motoring'); L(name, 'OV_lim/1', 'T_ovp/2', 'dc_overvoltage'); L(name, 'T_active/1', 'T_ovp/3');
A(name, 'simulink/Discrete/Unit Delay', 'ECU_delay', 'InitialCondition', '0', 'SampleTime', ts);   % D36 compute delay
L(name, 'T_ovp/1', 'ECU_delay/1', 'T_cmd');
L(name, 'ECU_delay/1', 'T_ref/1', 'T_ref');
% ---- status ----
A(name, 'simulink/Math Operations/Gain', 'W_shaft', 'Gain', '1/drv.ratio'); L(name, 'RT_w_motor/1', 'W_shaft/1');
A(name, 'simulink/Math Operations/Abs', 'W_abs'); L(name, 'W_shaft/1', 'W_abs/1', 'w_shaft');
CMP(name, 'Spinning', '>', [ecu '.run_frac*drv.w_nom']); L(name, 'W_abs/1', 'Spinning/1', 'w_shaft_abs');
LOG(name, 'Running', 'AND', 2); L(name, 'Enable_eff/1', 'Running/1'); L(name, 'Spinning/1', 'Running/2', 'spinning');
A(name, 'simulink/Math Operations/Gain', 'Ready_thr', 'Gain', [ecu '.ready_frac']); L(name, 'Target/1', 'Ready_thr/1');
A(name, 'simulink/Logic and Bit Operations/Relational Operator', 'Ready_cmp', 'Operator', '>=', 'OutDataTypeStr', 'boolean');
L(name, 'W_shaft/1', 'Ready_cmp/1'); L(name, 'Ready_thr/1', 'Ready_cmp/2', 'w_ready');
CMP(name, 'Target_pos', '>', '0'); L(name, 'Target/1', 'Target_pos/1');
LOG(name, 'Ready', 'AND', 2); L(name, 'Ready_cmp/1', 'Ready/1', 'at_speed'); L(name, 'Target_pos/1', 'Ready/2', 'target_pos');
% ---- fault code b0 OC b1 OV b2 UV b3 OT b4 CAN_TIMEOUT b5 STALL b6 SAFETY_OFF ----
A(name, 'simulink/Math Operations/Abs', 'I_abs'); L(name, 'RT_I_dc/1', 'I_abs/1', 'I_dc_10ms');
CMP(name, 'OC', '>', [ecu '.I_oc']); L(name, 'I_abs/1', 'OC/1', 'I_dc_abs');
CMP(name, 'OV', '>', [ecu '.V_ov']); L(name, 'RT_V_dc/1', 'OV/1');
CMP(name, 'V_low', '<', [ecu '.V_uv']); L(name, 'RT_V_dc/1', 'V_low/1');
LOG(name, 'UV', 'AND', 2); L(name, 'V_low/1', 'UV/1', 'v_low'); L(name, 'Enable_eff/1', 'UV/2');
CST(name, 'T_motor', [ecu '.T_motor_K'], '', ts);
CMP(name, 'OT', '>', [ecu '.T_ot_K']); L(name, 'T_motor/1', 'OT/1', 'T_motor');
A(name, 'simulink/Math Operations/Abs', 'T_abs'); L(name, 'ECU_delay/1', 'T_abs/1');
CMP(name, 'T_at_lim', '>=', [ecu '.stall_T_frac*drv.T_max']); L(name, 'T_abs/1', 'T_at_lim/1', 'T_ref_abs');
CMP(name, 'W_low', '<', [ecu '.stall_frac*drv.w_nom']); L(name, 'W_abs/1', 'W_low/1');
LOG(name, 'Stall_cond', 'AND', 3);
L(name, 'Enable_eff/1', 'Stall_cond/1'); L(name, 'T_at_lim/1', 'Stall_cond/2', 'T_at_lim');
L(name, 'W_low/1', 'Stall_cond/3', 'w_low');
ONOFF(name, 'Stall_timer', [ecu '.t_stall'], '0', '0', ts);
L(name, 'Stall_cond/1', 'Stall_timer/1', 'stall_cond');
DTC(name, 'STALL', 'boolean'); L(name, 'Stall_timer/1', 'STALL/1', 'stall_timer');
LOG(name, 'SAFETY_OFF', 'NOT'); L(name, 'RT_Safety_OK/1', 'SAFETY_OFF/1');
% the seven fault flags travel as one virtual bus (spec 6.3); the fault word and the fault flag pick theirs right in
% front of them. Both pick by name, so the bus's input order is free (ruling F2-R1): the order with the fewest
% crossings a search found (fix round 3, with the riser order: t6_fix3_bcsearch.log, 116 -> 87, an order no single
% move improves; fix round 2: t6_fix2_bcsearch.log). The fault word keeps its bit order: Fault_bits_sel picks in the
% Mux's operand order (R3)
fl = {'STALL', 'STALL/1'; 'can_timeout', 'Timeout/1'; 'OT', 'OT/1'; 'SAFETY_OFF', 'SAFETY_OFF/1'; 'OV', 'OV/1'; ...
      'UV', 'UV/1'; 'OC', 'OC/1'};
bits = {'OC', 'OV', 'UV', 'OT', 'can_timeout', 'STALL', 'SAFETY_OFF'};   % the fault word's bits 0-6
BCV(name, 'Fault_BC', fl);
A(name, 'simulink/Signal Routing/Mux', 'Fault_bits', 'Inputs', '7');
SEL(name, 'Fault_bits_sel', 'Fault_BC/1', 'fault_flags', bits, 'Fault_bits');
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Fault_bits_dbl', 'OutDataTypeStr', 'double');
A(name, 'simulink/Math Operations/Gain', 'Fault_weights', 'Gain', '2.^(0:6)', 'Multiplication', 'Matrix(K*u)');
DTC(name, 'Fault_Code', 'uint16');
L(name, 'Fault_bits/1', 'Fault_bits_dbl/1', 'fault_bits'); L(name, 'Fault_bits_dbl/1', 'Fault_weights/1', 'fault_bits_dbl');
L(name, 'Fault_weights/1', 'Fault_Code/1', 'fault_word');
LOG(name, 'Impl_Fault', 'OR', 4);
SEL(name, 'Impl_Fault_sel', 'Fault_BC/1', 'fault_flags', {'OC', 'OV', 'OT', 'STALL'}, 'Impl_Fault');
CST(name, 'Type', 'impl_type', 'uint8', ts);
BC(name, 'Elec_BC', 'Bus_ImplElecState', {'Impl_Type', 'Type/1'; 'Impl_Running', 'Running/1'; 'Impl_Fault', 'Impl_Fault/1'; ...
    'Impl_Ready', 'Ready/1'; 'Impl_Speed_radps', 'W_shaft/1'; 'Impl_Torque_Nm', 'ECU_delay/1'; 'Impl_Power_W', 'RT_P_in/1'; ...
    'Impl_Current_A', 'RT_I_dc/1'; 'Impl_Temp_K', 'T_motor/1'; 'Fault_Code', 'Fault_Code/1'});
L(name, 'Elec_BC/1', 'impl_elec/1', 'impl_elec');
local_fb(name, 'Elec_BC/1');
L(name, 'FB_BC/1', 'can_tx/1', 'can_tx');
% ---- mask: drv (implement drive parameter struct), impl_type ----
m = Simulink.Mask.create(name);
m.addParameter('Name', 'drv', 'Prompt', 'Implement drive parameters (P.power.impl.<mower|tiller>)', 'Type', 'edit', ...
    'Value', 'P.power.impl.mower');
m.addParameter('Name', 'impl_type', 'Prompt', 'Implement type code (1 mower, 2 tiller)', 'Type', 'edit', 'Value', '1');
local_save(name, f, ['impl_ecu (T05, build plan T05.5, D36): COTS implement drive controller at Ts.impl_ctrl. Mask: ' ...
    'drv = P.power.impl.<mower|tiller> (Kp, Ki, T_max, T_brake, ramp_up/dn, ratio, w_nom), impl_type. Inputs: 1 can_rx ' ...
    'Bus_Can1_McuSlow (0x140 IMPL_CMD via implcmd_can_unpack), 2 Safety_OK, 3 w_motor rad/s, 4 V_dc V, 5 I_dc A, 6 P_in W ' ...
    '(Ts.base). CAN timeout: age since the last frame with a new alive value >= P.power.impl.ecu.t_can_timeout -> reference 0. ' ...
    'Speed reference (rpm at blade / rotor -> rad/s, 0 when disabled, timed out or Safety_OK false) -> Rate Limiter -> ' ...
    'x ratio -> Discrete PID (PI, clamping, limits [-T_brake, T_max], integrator reset on the disable edge and held while ' ...
    'the drive is inactive; drive active while enabled or still spinning > run_frac x w_nom, else T = 0) -> no braking ' ...
    'torque while V_dc > V_ov (DC-link over-voltage protection) -> Unit Delay (10 ms ECU ' ...
    'compute delay = D36 loop breaker). Outputs: 1 T_ref N m (motor), 2 impl_elec Bus_ImplElecState (0.01 s), 3 can_tx ' ...
    'Bus_Can1_Impl (0x141 IMPL_FB, ready = speed >= ready_frac x target, 0.05 s).']);
end

function local_fb(sys, src)
% IMPL_FB frame (0x141, 0.05 s) from a Bus_ImplElecState signal 'src' at Ts.impl_ctrl -> 'FB_BC/1' (Bus_Can1_Impl)
RT(sys, 'FB_RT', 'Ts.can_slow'); L(sys, src, 'FB_RT/1', 'impl_elec');
sig = {'Impl_Running', 'Impl_Fault', 'Impl_Ready', 'Impl_Type', 'Impl_Speed_radps', 'Impl_Current_A', 'Impl_Temp_K'};
BS(sys, 'FB_sel', sig(1:4)); L(sys, 'FB_RT/1', 'FB_sel/1', 'impl_elec_50ms');      % a pick per consumer
BS(sys, 'FB_speed_sel', sig(5)); L(sys, 'FB_RT/1', 'FB_speed_sel/1');
BS(sys, 'FB_I_sel', sig(6)); L(sys, 'FB_RT/1', 'FB_I_sel/1');
BS(sys, 'FB_T_sel', sig(7)); L(sys, 'FB_RT/1', 'FB_T_sel/1');
A(sys, 'simulink/Math Operations/Gain', 'FB_rpm', 'Gain', '1/P.power.impl.rpm2rad'); L(sys, 'FB_speed_sel/1', 'FB_rpm/1');
A(sys, 'simulink/Signal Attributes/Data Type Conversion', 'FB_rpm_s', 'OutDataTypeStr', 'single');
L(sys, 'FB_rpm/1', 'FB_rpm_s/1', 'speed_rpm');
A(sys, 'simulink/Signal Attributes/Data Type Conversion', 'FB_I_s', 'OutDataTypeStr', 'single'); L(sys, 'FB_I_sel/1', 'FB_I_s/1');
A(sys, 'simulink/Math Operations/Bias', 'FB_degC', 'Bias', '-273.15'); L(sys, 'FB_T_sel/1', 'FB_degC/1');
HEIGHT([sys '/FB_degC'], 60);   % graphical (fix round 2 trial in impl_ecu, t6_fix2_hprobe.log: the temp line
                                % then clears FB_RT's branch to FB_I_sel)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', 'FB_T_s', 'OutDataTypeStr', 'single');
L(sys, 'FB_degC/1', 'FB_T_s/1', 'temp_degC');
A(sys, 'simulink/Sources/Counter Free-Running', 'FB_alive_cnt', 'NumBits', '8', 'tsamp', 'Ts.can_slow');
A(sys, 'simulink/Signal Attributes/Data Type Conversion', 'FB_alive', 'OutDataTypeStr', 'uint8');
L(sys, 'FB_alive_cnt/1', 'FB_alive/1', 'alive_count');
A(sys, LIB('ssr'), 'FB_pack', 'ReferencedSubsystem', 'implfb_can_pack');
src8 = {'FB_sel/1', 'FB_sel/2', 'FB_sel/3', 'FB_sel/4', 'FB_rpm_s/1', 'FB_I_s/1', 'FB_T_s/1', 'FB_alive/1'};
nm8 = {'', '', '', '', 'speed_rpm_s', 'current_A', 'temp_degC_s', 'alive'};
for k = 1:numel(src8), L(sys, src8{k}, sprintf('FB_pack/%d', k), nm8{k}); end
BC(sys, 'FB_BC', 'Bus_Can1_Impl', {'impl_fb', 'FB_pack/1'});
end

% ===========================================================================================
%  Implement choices (VC_IMPL; identical ports, build plan T05.5)
% ===========================================================================================
function H = local_impl_ports(name)
% The choices map to Implement_Loads' ports by name, so their inport order is this builder's: impl_mech, can_rx,
% Safety_OK, hw_cmd has the fewest crossings over the four choices (fix round 1: 158 against 180 for the order of
% Implement_Loads, work/layout/logs/t6_fix1_portsearch_impl.log)
IN(name, 'impl_mech', 1, 'Bus: Bus_ImplMechState', 'Ts.ctrl');
IN(name, 'can_rx', 2, 'Bus: Bus_Can1_McuSlow', 'Ts.can_slow');
IN(name, 'Safety_OK', 3, 'boolean', 'Ts.safety');
IN(name, 'hw_cmd', 4, 'Bus: Bus_PowerHwCmd', 'Ts.base');
OUT(name, 'impl_elec', 1, 'Bus: Bus_ImplElecState'); OUT(name, 'can_tx', 2, 'Bus: Bus_Can1_Impl'); OUT(name, 'P_impl', 3, 'double');
H.IMP48_p = CPH(CP(name, 'IMP48_p', 1, 'E', 'Left'));
H.IMP48_n = CPH(CP(name, 'IMP48_n', 2, 'E', 'Left'));
H.V24IMP_p = CPH(CP(name, 'V24IMP_p', 3, 'E', 'Left'));
H.V24IMP_n = CPH(CP(name, 'V24IMP_n', 4, 'E', 'Left'));
H.Impl_Shaft = CPH(CP(name, 'Impl_Shaft', 5, 'M', 'Right'));
end

function d = local_impl_desc(what)
d = [what ' Ports (identical in every VC_IMPL choice): IMP48_p/n, V24IMP_p/n (electrical, Left), Impl_Shaft ' ...
     '(rotational, Right, omega > 0 = nominal direction); inputs 1 impl_mech Bus_ImplMechState (0.01 s), 2 can_rx ' ...
     'Bus_Can1_McuSlow (0.05 s), 3 Safety_OK boolean (Ts.safety), 4 hw_cmd Bus_PowerHwCmd (Ts.base); outputs ' ...
     '1 impl_elec Bus_ImplElecState (0.01 s), 2 can_tx Bus_Can1_Impl (0x141 IMPL_FB, 0.05 s), 3 P_impl W (IMP48 + ' ...
     'V24IMP electrical input, Ts.base).'];
end

function local_lift_load(name, hP, hN)
% lift actuator electrical load: Dynamic Load lift_P_W x abs(Lift_Duty) on V24IMP
pl = PH(A(name, LIB('dload'), 'LOAD_LIFT', 'load_type', 'ee.enum.dc_ac.dc', 'min_supply_voltage', 'P.power.impl.load_min_V', ...
    'pow_min', 'P.power.lv.load.pow_min', 'G_reverse', 'P.power.lv.load.G_reverse', 'deltaV', 'P.power.lv.load.deltaV'));
PL(name, hP, pl.LConn(1)); PL(name, pl.RConn(1), hN);
A(name, 'simulink/Math Operations/Abs', 'Lift_abs'); L(name, 'Hw_sel/1', 'Lift_abs/1');
A(name, 'simulink/Math Operations/Gain', 'P_lift', 'Gain', 'P.power.impl.lift_P_W'); L(name, 'Lift_abs/1', 'P_lift/1', 'lift_duty_abs');
h = SPS(name, 'P_lift_PS', 'P_lift/1', 'W'); PL(name, h, pl.LConn(2));
end

function local_impl_none(name, f)
local_fresh(name, f);
H = local_impl_ports(name);
for n = {'IMP48_p', 'IMP48_n', 'V24IMP_p', 'V24IMP_n'}
    po = PH(A(name, LIB('open'), ['Open_' n{1}]));
    PL(name, H.(n{1}), po.LConn(1));
end
% shaft: damper to the local reference + network tie (ICD 2, D35)
pd = PH(A(name, 'fl_lib/Mechanical/Rotational Elements/Rotational Damper', 'D_impl', 'D', 'P.power.impl.D_none'));
pr = PH(A(name, LIB('mref'), 'Ref_impl'));
PL(name, H.Impl_Shaft, pd.LConn(1)); PL(name, pd.RConn(1), pr.LConn(1));
pc = PH(A(name, sprintf('fl_lib/Electrical/Electrical Elements/Rotational\nElectromechanical\nConverter'), 'Tie_impl', ...
    'K', 'P.drive.tie.K'));
pt = PH(A(name, LIB('res'), 'Tie_impl_R', 'R', 'P.drive.tie.R'));
PL(name, pc.LConn(2), H.Impl_Shaft); PL(name, pc.RConn(2), pr.LConn(1));
PL(name, pc.RConn(1), H.IMP48_n); PL(name, pc.LConn(1), pt.LConn(1)); PL(name, pt.RConn(1), H.IMP48_n);
for n = {'can_rx', 'hw_cmd', 'Safety_OK', 'impl_mech'}
    A(name, 'simulink/Sinks/Terminator', ['Term_' n{1}]); L(name, [n{1} '/1'], ['Term_' n{1} '/1'], n{1});
end
CST(name, 'Elec_none', 'P.ic.Bus_ImplElecState', 'Bus: Bus_ImplElecState', 'Ts.impl_ctrl');
L(name, 'Elec_none/1', 'impl_elec/1', 'impl_elec');
local_fb(name, 'Elec_none/1');
L(name, 'FB_BC/1', 'can_tx/1', 'can_tx');
CST(name, 'P_zero', '0', 'double', 'Ts.base');
L(name, 'P_zero/1', 'P_impl/1', 'P_impl');
local_save(name, f, local_impl_desc(['Impl_None (VC_IMPL == 0, T05): Open Circuit on IMP48 / V24IMP, Rotational Damper ' ...
    'P.power.impl.D_none + network tie (Rotational Electromechanical Converter P.drive.tie.K, 1 MOhm to IMP48_n, D35) on ' ...
    'Impl_Shaft; IMPL_FB with impl_type 0.']));
end

function local_impl_motor(name, f, drv, typ, gear)
local_fresh(name, f);
H = local_impl_ports(name);
A(name, 'simulink/Sinks/Terminator', 'Term_impl_mech'); L(name, 'impl_mech/1', 'Term_impl_mech/1', 'impl_mech');
BS(name, 'Hw_sel', {'Lift_Duty'}); L(name, 'hw_cmd/1', 'Hw_sel/1', 'hw_cmd');
% ---- 48 V drive: meter, DC-link capacitor, Motor & Drive (System Level), shaft (+ 12:1 gearbox) ----
[i48, ip, in] = ISENS(name, 'I48_m');
PL(name, H.IMP48_p, ip); nodeA = in;
pc = PH(A(name, LIB('cap'), 'C_link', 'c', [drv '.C_link'], 'r', [drv '.C_link_esr'], ...
    'vc_specify', 'on', 'vc_priority', 'High', 'vc', '0'));
PL(name, nodeA, pc.LConn(1)); PL(name, pc.RConn(1), H.IMP48_n);
% DC-link over-voltage clamp (brake chopper / TVS, Zener diode): limits the isolated link when K1 opens while the
% drive is still braking (the 10 ms ECU over-voltage logic plus the 20 ms torque lag are too slow for 5.5 mF)
pz = PH(A(name, 'ee_lib/Semiconductors & Converters/Diode', 'DC_clamp', 'ZenerEnable', 'true', ...
    'BV', 'P.power.impl.dc_clamp.BV', 'Rz', 'P.power.impl.dc_clamp.Rz', 'Goff', '1e-8'));
PL(name, pz.LConn(1), H.IMP48_n); PL(name, pz.RConn(1), nodeA);           % anode on DC-, cathode on DC+
pm = PH(A(name, sprintf('ee_lib/Electromechanical/Motor & Drive\n(System Level)'), 'Motor_Drive', ...
    'torque_speed_param', 'ee.enum.electromech.envelope.tabulated2D', 'w_t', [drv '.w_t_rpm'], 'vdc_t', [drv '.vdc_t'], ...
    'T_t_2D', [drv '.T_t_2D'], 'overtorque_param', 'ee.enum.electromech.overtorque.no', ...
    'eff', [drv '.eff'], 'w_eff', [drv '.w_eff_rpm'], 'T_eff', [drv '.T_eff'], 'Piron', [drv '.Piron'], ...
    'Pbase', [drv '.Pbase'], 'Tc', [drv '.Tc'], 'EnableSwitch', 'false', 'J', [drv '.J_motor'], 'lam', [drv '.lam'], ...
    'speed0', '0'));
% Motor & Drive ports (verified): LConn = [+, Tr (PS in), R rotor], RConn = [-, w (PS out, rad/s), C case]
PL(name, nodeA, pm.LConn(1)); PL(name, pm.RConn(1), H.IMP48_n);
pr = PH(A(name, LIB('mref'), 'Ref_case'));
PL(name, pm.RConn(3), pr.LConn(1));
if gear
    pg = PH(A(name, 'fl_lib/Mechanical/Mechanisms/Gear Box', 'Gear_Box', 'ratio', [drv '.ratio']));
    PL(name, pm.LConn(3), pg.LConn(1)); PL(name, pg.RConn(1), H.Impl_Shaft);     % B = motor, F = rotor (same sense)
else
    PL(name, pm.LConn(3), H.Impl_Shaft);
end
w = PSS(name, 'W_motor', pm.RConn(2), 'rad/s');
v48 = VSENS(name, 'V48_m', nodeA, H.IMP48_n);
% ---- lift load on V24IMP ----
[i24, jp, jn] = ISENS(name, 'I24_m');
PL(name, H.V24IMP_p, jp);
local_lift_load(name, jn, H.V24IMP_n);
v24 = VSENS(name, 'V24_m', jn, H.V24IMP_n);
PROD(name, 'P48', v48, i48); PROD(name, 'P24', v24, i24);
SUMS(name, 'P_sum', '++', {'P48/1', 'P24/1'});
L(name, 'P_sum/1', 'P_impl/1', 'P_impl');
% ---- implement ECU ----
A(name, LIB('ssr'), 'ECU', 'ReferencedSubsystem', 'impl_ecu');
set_param([name '/ECU'], 'drv', drv, 'impl_type', num2str(typ));
L(name, 'can_rx/1', 'ECU/1', 'can_rx'); L(name, 'Safety_OK/1', 'ECU/2', 'Safety_OK');
L(name, w, 'ECU/3', 'w_motor'); L(name, v48, 'ECU/4', 'V_dc'); L(name, i48, 'ECU/5', 'I_dc'); L(name, 'P_sum/1', 'ECU/6');
RT(name, 'Tr_RT', 'Ts.base'); L(name, 'ECU/1', 'Tr_RT/1', 'T_ref');
h = SPS(name, 'Tr_PS', 'Tr_RT/1', 'N*m', 'T_ref_1ms'); PL(name, h, pm.LConn(2));
L(name, 'ECU/2', 'impl_elec/1', 'impl_elec'); L(name, 'ECU/3', 'can_tx/1', 'can_tx');
if gear
    d = ['Impl_Tiller (VC_IMPL == 2, T05): Motor & Drive (System Level) P.power.impl.tiller (tabulated2D envelope, zero ' ...
         'torque at V_dc <= 30 V), DC-link C 1 mF on IMP48, Gear Box ratio 12 (motor -> rotor, same sense) to Impl_Shaft, ' ...
         'impl_ecu (drv = P.power.impl.tiller, type 2), lift load 150 W x |Lift_Duty| on V24IMP.'];
else
    d = ['Impl_Mower (VC_IMPL == 1, T05): Motor & Drive (System Level) P.power.impl.mower (tabulated2D envelope, zero ' ...
         'torque at V_dc <= 30 V), DC-link C 1 mF on IMP48, shaft directly on Impl_Shaft (blade inertia and load in ' ...
         'STRUCTURE), impl_ecu (drv = P.power.impl.mower, type 1), lift load 150 W x |Lift_Duty| on V24IMP.'];
end
local_save(name, f, local_impl_desc(d));
end

function local_impl_sprayer(name, f)
local_fresh(name, f);
sp = 'P.power.impl.sprayer';
ts = 'Ts.impl_ctrl';
H = local_impl_ports(name);
for n = {'IMP48_p', 'IMP48_n'}
    po = PH(A(name, LIB('open'), ['Open_' n{1}]));
    PL(name, H.(n{1}), po.LConn(1));
end
BS(name, 'Hw_sel', {'Lift_Duty'}); L(name, 'hw_cmd/1', 'Hw_sel/1', 'hw_cmd');               % a pick per consumer
BS(name, 'Pump_sel', {'Spray_Pump_Duty'}); L(name, 'hw_cmd/1', 'Pump_sel/1');
BS(name, 'Noz_sel', {'Spray_Nozzle_Cmd'}); L(name, 'hw_cmd/1', 'Noz_sel/1');
% ---- V24IMP: meter -> buck pump driver -> PM DC pump motor; nozzles; lift ----
[i24, jp, jn] = ISENS(name, 'I24_m');
PL(name, H.V24IMP_p, jp); nodeB = jn;
pb = PH(A(name, 'ee_lib/Semiconductors & Converters/Converters/Average-Value DC-DC Converter', 'Pump_driver', ...
    'implementation_option', 'ee.enum.converters.implementation.behavioral', ...
    'converter_type', 'ee.enum.converters.dcdctype.buck', 'control_option', 'ee.enum.converters.controlinput.dutycycle', ...
    'efficiency_option', 'ee.enum.converters.efficiency.constant', 'efficiency_fixed', [sp '.buck_eff']));
% Average-Value DC-DC ports (verified): LConn = [duty PS, in+, in-], RConn = [out+, out-]
PL(name, nodeB, pb.LConn(2)); PL(name, pb.LConn(3), H.V24IMP_n); PL(name, pb.RConn(2), H.V24IMP_n);
[im, kp, kn] = ISENS(name, 'I_motor_m');
pm = PH(A(name, 'ee_lib/Electromechanical/Brushed Motors/DC Motor', 'Pump_motor', 'field_option', 'ee.enum.field_option.pm', ...
    'parameterization', 'ee.enum.dcparam_option.circuit_param', 'Ra', [sp '.Ra'], 'La', [sp '.La'], ...
    'emf_or_torque', 'ee.enum.emf_or_torque.kv', 'Kv', [sp '.Kv'], 'J', [sp '.J'], 'J_unit', 'kg*m^2', 'lam', [sp '.lam'], ...
    'speed0', '0'));
% DC Motor ports (verified): LConn = [+, R rotor], RConn = [-, C case]
PL(name, pb.RConn(1), kp); PL(name, kn, pm.LConn(1)); PL(name, pm.RConn(1), H.V24IMP_n);
pr = PH(A(name, LIB('mref'), 'Ref_case'));
PL(name, pm.LConn(2), H.Impl_Shaft); PL(name, pm.RConn(2), pr.LConn(1));
pw = PH(A(name, sprintf('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational\nMotion Sensor'), 'Pump_speed'));
PL(name, pw.LConn(1), H.Impl_Shaft); PL(name, pw.RConn(1), pr.LConn(1));     % R on the shaft, C on the case reference
w = PSS(name, 'W_pump', pw.RConn(2), 'rad/s');
pa = PH(A(name, 'fl_lib/Physical Signals/Sinks/PS Terminator', 'Angle_term'));
PL(name, pw.RConn(3), pa.LConn(1));
% duty: saturate, driver soft start (inrush limit), -> PS
A(name, 'simulink/Discontinuities/Saturation', 'Duty_sat', 'UpperLimit', '1', 'LowerLimit', '0');
L(name, 'Pump_sel/1', 'Duty_sat/1');
A(name, 'simulink/Discontinuities/Rate Limiter', 'Duty_softstart', 'RisingSlewLimit', [sp '.duty_slew'], ...
    'FallingSlewLimit', '-1e3', 'SampleTimeMode', 'inherited', 'InitialCondition', '0');
L(name, 'Duty_sat/1', 'Duty_softstart/1', 'pump_duty');
h = SPS(name, 'Duty_PS', 'Duty_softstart/1', '1', 'pump_duty_ramped'); PL(name, h, pb.LConn(1));
% nozzle solenoids: nozzle_P_W x sum(Spray_Nozzle_Cmd)
pn = PH(A(name, LIB('dload'), 'LOAD_NOZZLES', 'load_type', 'ee.enum.dc_ac.dc', 'min_supply_voltage', 'P.power.impl.load_min_V', ...
    'pow_min', 'P.power.lv.load.pow_min', 'G_reverse', 'P.power.lv.load.G_reverse', 'deltaV', 'P.power.lv.load.deltaV'));
PL(name, nodeB, pn.LConn(1)); PL(name, pn.RConn(1), H.V24IMP_n);
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Noz_dbl', 'OutDataTypeStr', 'double');
A(name, 'simulink/Math Operations/Sum of Elements', 'Noz_n');
A(name, 'simulink/Math Operations/Gain', 'P_noz', 'Gain', 'P.power.impl.nozzle_P_W');
L(name, 'Noz_sel/1', 'Noz_dbl/1'); L(name, 'Noz_dbl/1', 'Noz_n/1', 'nozzle_cmd_dbl'); L(name, 'Noz_n/1', 'P_noz/1', 'nozzles_on');
h = SPS(name, 'P_noz_PS', 'P_noz/1', 'W'); PL(name, h, pn.LConn(2));
local_lift_load(name, nodeB, H.V24IMP_n);
v24 = VSENS(name, 'V24_m', nodeB, H.V24IMP_n);
PROD(name, 'P24', v24, i24);
L(name, 'P24/1', 'P_impl/1', 'P_impl');
% ---- pump driver status (10 ms) ----
for s = {'Duty_sat/1', 'duty'; w, 'w'; v24, 'V24'; i24, 'I24'; 'P24/1', 'P'; im, 'Im'; 'Safety_OK/1', 'Safety'}'
    RT(name, ['RT_' s{2}], ts); L(name, s{1}, ['RT_' s{2} '/1']);
end
BS(name, 'Mech_sel', {'linePressure'}); L(name, 'impl_mech/1', 'Mech_sel/1', 'impl_mech');
CMP(name, 'Duty_on', '>', [sp '.run_duty']); L(name, 'RT_duty/1', 'Duty_on/1', 'pump_duty_10ms');
CMP(name, 'Spinning', '>', ['P.power.impl.ecu.run_frac*' sp '.w_nom']); L(name, 'RT_w/1', 'Spinning/1', 'w_pump_10ms');
LOG(name, 'Running', 'AND', 2); L(name, 'Duty_on/1', 'Running/1', 'duty_on'); L(name, 'Spinning/1', 'Running/2', 'spinning');
CMP(name, 'Ready', '>', [sp '.p_ready_frac*P.structure.impl.sprayer.p_set']); L(name, 'Mech_sel/1', 'Ready/1');
CMP(name, 'V_low', '<', 'P.power.impl.load_min_V'); L(name, 'RT_V24/1', 'V_low/1', 'V24_10ms');
LOG(name, 'UV', 'AND', 2); L(name, 'V_low/1', 'UV/1', 'v_low'); L(name, 'Duty_on/1', 'UV/2');
LOG(name, 'SAFETY_OFF', 'NOT'); L(name, 'RT_Safety/1', 'SAFETY_OFF/1', 'Safety_OK_10ms');
CST(name, 'No_fault', 'false', 'boolean', ts);
MUX(name, 'Fault_bits', {'No_fault/1', 'No_fault/1', 'UV/1', 'No_fault/1', 'No_fault/1', 'No_fault/1', 'SAFETY_OFF/1'});
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Fault_bits_dbl', 'OutDataTypeStr', 'double');
A(name, 'simulink/Math Operations/Gain', 'Fault_weights', 'Gain', '2.^(0:6)', 'Multiplication', 'Matrix(K*u)');
DTC(name, 'Fault_Code', 'uint16');
L(name, 'Fault_bits/1', 'Fault_bits_dbl/1', 'fault_bits'); L(name, 'Fault_bits_dbl/1', 'Fault_weights/1', 'fault_bits_dbl');
L(name, 'Fault_weights/1', 'Fault_Code/1', 'fault_word');
A(name, 'simulink/Math Operations/Gain', 'Torque', 'Gain', [sp '.Kv/P.power.impl.rpm2rad']);   % Kt = Ke (SI)
L(name, 'RT_Im/1', 'Torque/1', 'I_motor_10ms');
CST(name, 'Type', '3', 'uint8', ts);
CST(name, 'T_motor', 'P.power.impl.ecu.T_motor_K', '', ts);
BC(name, 'Elec_BC', 'Bus_ImplElecState', {'Impl_Type', 'Type/1'; 'Impl_Running', 'Running/1'; 'Impl_Fault', 'No_fault/1'; ...
    'Impl_Ready', 'Ready/1'; 'Impl_Speed_radps', 'RT_w/1'; 'Impl_Torque_Nm', 'Torque/1'; 'Impl_Power_W', 'RT_P/1'; ...
    'Impl_Current_A', 'RT_I24/1'; 'Impl_Temp_K', 'T_motor/1'; 'Fault_Code', 'Fault_Code/1'});
L(name, 'Elec_BC/1', 'impl_elec/1', 'impl_elec');
local_fb(name, 'Elec_BC/1');
L(name, 'FB_BC/1', 'can_tx/1', 'can_tx');
A(name, 'simulink/Sinks/Terminator', 'Term_can_rx'); L(name, 'can_rx/1', 'Term_can_rx/1', 'can_rx');
local_save(name, f, local_impl_desc(['Impl_Sprayer (VC_IMPL == 3, T05): 24 V PM DC pump motor (P.power.impl.sprayer, ' ...
    'shaft on Impl_Shaft) fed by an Average-Value DC-DC buck (duty = hw_cmd.Spray_Pump_Duty with driver soft start ' ...
    'duty_slew) on V24IMP; nozzle solenoids Dynamic Load nozzle_P_W x sum(Spray_Nozzle_Cmd); lift 150 W x |Lift_Duty|; ' ...
    'IMPL_FB from the pump state and impl_mech (ready = linePressure > p_ready_frac x p_set); IMP48 open.']));
end

% ===========================================================================================
%  Dock_Charger (spec 2.8 / 3.7, VC_CHG)
% ===========================================================================================
function local_charger(name, f)
local_fresh(name, f);
cg = 'P.power.charger';
IN(name, 'bms_cmd', 1, 'Bus: Bus_BMS_Cmd', 'Ts.bms_fast');
hp = CPH(CP(name, 'DOCK_p', 1, 'E', 'Left'));
hn = CPH(CP(name, 'DOCK_n', 2, 'E', 'Left'));
% dock contacts: R_dock + switch driven by env.dock_contact (C4)
RT(name, 'Dock_RT', 'Ts.base'); PK(name, 'env', 'dock_contact', 'Dock_RT/1');
A(name, 'simulink/Signal Attributes/Data Type Conversion', 'Dock_dbl', 'OutDataTypeStr', 'double');
L(name, 'Dock_RT/1', 'Dock_dbl/1', 'dock_1ms');
prd = PH(A(name, LIB('res'), 'R_dock', 'R', [cg '.R_dock']));
psw = PH(A(name, LIB('switch'), 'S_DOCK', 'R_closed', '1e-6', 'G_open', '1e-8', 'Threshold', '0.5'));
h = SPS(name, 'Dock_PS', 'Dock_dbl/1', 'V', 'dock_dbl'); PL(name, h, psw.LConn(1));
PL(name, hp, prd.LConn(1)); PL(name, prd.RConn(1), psw.LConn(2));
% bus-side voltage sense (dock sense pins), 1 ms Unit Delay (D36) for the current law / OVP
vs = VSENS(name, 'V_dock_sense', hp, hn);
A(name, 'simulink/Discrete/Unit Delay', 'V_fb', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
L(name, vs, 'V_fb/1', 'V_dock');
% charger variant (VC_CHG)
v = local_variant(name, 'Charger', {'bms_cmd', 'dock', 'V_fb'}, {'I_chg', 'P_grid'}, ...
    {'chg_p', 'E', 'Left'; 'chg_n', 'E', 'Left'}, {'BMS_CTRL', '', 'VC_CHG == 1'; 'CCCV_BLOCK', '', 'VC_CHG == 2'});
local_chg_bms(v);
local_chg_cccv(v);
L(name, 'bms_cmd/1', 'Charger/1', 'bms_cmd'); L(name, 'Dock_RT/1', 'Charger/2'); L(name, 'V_fb/1', 'Charger/3', 'V_fb');
PL(name, psw.RConn(1), shaddad_bu('pport', v, 'chg_p'));
PL(name, shaddad_bu('pport', v, 'chg_n'), hn);
% ---- output bus charger ----
OB(name, 'charger', 'P_grid', 'Charger/2'); OB(name, 'charger', 'I_chg', 'Charger/1');
shaddad_bu('portorder', name, {'bms_cmd', 'env'}, 'Inport');
local_save(name, f, ['Dock_Charger (T05, spec 2.8 / 3.7, D36). Ports DOCK_p/n (Left, to PDU after K4 + F_CHG). Dock contacts ' ...
    'R_dock + SPST switch on dock_contact. Variant Charger (VC_CHG): 1 BMS_CTRL = controlled current source I = dock x ' ...
    'Charger_Enable x min(I_req, I_max, P_max / V_dock(k-1)) -> slew +/-20 A/s -> 0.5 s lag, OVP I = 0 if V_dock(k-1) > ' ...
    'V_ovp (dock-pin voltage through a 1 ms Unit Delay, D36), output diode, 10 kOhm bleed; 2 CCCV_BLOCK = batt_lib/Cyclers/' ...
    'Charger (trigger = dock_contact AND battery voltage > 0.5 V_ovp on the dock pins, i.e. K4 closed) + diode + bleed ' ...
    '(standalone tests without BMS). Inputs: 1 bms_cmd Bus_BMS_Cmd (0.01 s), 2 bus env (In Bus Element: dock_contact ' ...
    'boolean, 0.1 s). Output bus charger (Out Bus Element, Ts.base): P_grid W (= V x I / eta), I_chg A (charger output ' ...
    'current).']);
end

function local_chg_bms(v)
s = [v '/BMS_CTRL'];
cg = 'P.power.charger';
BS(s, 'Cmd_sel', {'Charger_Enable'}); L(s, 'bms_cmd/1', 'Cmd_sel/1', 'bms_cmd');            % a pick per consumer
BS(s, 'Ireq_sel', {'Charger_I_Req_A'}); L(s, 'bms_cmd/1', 'Ireq_sel/1');
RT(s, 'Dock_RT', 'Ts.bms_fast'); L(s, 'dock/1', 'Dock_RT/1', 'dock');
RT(s, 'V_RT', 'Ts.bms_fast'); L(s, 'V_fb/1', 'V_RT/1', 'V_fb');
A(s, 'simulink/Discontinuities/Saturation', 'V_min', 'UpperLimit', 'inf', 'LowerLimit', '1');
CST(s, 'P_max', [cg '.P_max'], '', 'Ts.bms_fast');
A(s, 'simulink/Math Operations/Product', 'I_pmax', 'Inputs', '*/');
L(s, 'V_RT/1', 'V_min/1', 'V_fb_10ms'); L(s, 'P_max/1', 'I_pmax/1'); L(s, 'V_min/1', 'I_pmax/2', 'V_fb_min');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'I_req_dbl', 'OutDataTypeStr', 'double');
L(s, 'Ireq_sel/1', 'I_req_dbl/1');
CST(s, 'I_max', [cg '.I_max'], '', 'Ts.bms_fast');
A(s, 'simulink/Math Operations/MinMax', 'I_law', 'Function', 'min', 'Inputs', '3');
L(s, 'I_req_dbl/1', 'I_law/1', 'I_req'); L(s, 'I_max/1', 'I_law/2'); L(s, 'I_pmax/1', 'I_law/3', 'I_pmax');
LOG(s, 'Enable', 'AND', 2); L(s, 'Cmd_sel/1', 'Enable/1'); L(s, 'Dock_RT/1', 'Enable/2', 'dock_10ms');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Enable_dbl', 'OutDataTypeStr', 'double');
L(s, 'Enable/1', 'Enable_dbl/1', 'charge_enable');
PROD(s, 'I_en', 'I_law/1', 'Enable_dbl/1');
A(s, 'simulink/Discontinuities/Saturation', 'I_pos', 'UpperLimit', 'inf', 'LowerLimit', '0');
L(s, 'I_en/1', 'I_pos/1');
A(s, 'simulink/Discontinuities/Rate Limiter', 'Slew', 'RisingSlewLimit', [cg '.slew'], 'FallingSlewLimit', ['-' cg '.slew'], ...
    'SampleTimeMode', 'inherited', 'InitialCondition', '0');
A(s, 'ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)', 'Lag', 'K', '1', 'T', [cg '.tau'], ...
    'Ts', 'Ts.bms_fast', 'init_option', 'Specify', 'x0', '0');
L(s, 'I_pos/1', 'Slew/1'); L(s, 'Slew/1', 'Lag/1');
RT(s, 'I_RT', 'Ts.base'); L(s, 'Lag/1', 'I_RT/1', 'I_set');
CMP(s, 'OVP_ok', '<', [cg '.V_ovp']); L(s, 'V_fb/1', 'OVP_ok/1');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'OVP_dbl', 'OutDataTypeStr', 'double');
L(s, 'OVP_ok/1', 'OVP_dbl/1', 'ovp_ok');
PROD(s, 'I_cmd', 'I_RT/1', 'OVP_dbl/1');
L(s, 'I_cmd/1', 'I_chg/1', 'I_chg');
% network: source (head = charger +) -> diode -> chg_p; bleed across the source
pcs = PH(A(s, LIB('ccs'), 'Source'));
pd = PH(A(s, 'ee_lib/Semiconductors & Converters/Diode', 'D_out', 'Vf', [cg '.diode_Vf'], 'Ron', [cg '.diode_Ron'], ...
    'Goff', [cg '.diode_Goff']));
pb = PH(A(s, LIB('res'), 'R_bleed', 'R', [cg '.R_bleed']));
hp = CPH([s '/chg_p']); hn = CPH([s '/chg_n']);
PL(s, pcs.LConn(1), pd.LConn(1)); PL(s, pd.RConn(1), hp); PL(s, pcs.RConn(2), hn);
PL(s, pcs.LConn(1), pb.LConn(1)); PL(s, pb.RConn(1), hn);
h = SPS(s, 'I_PS', 'I_cmd/1', 'A'); PL(s, h, pcs.RConn(1));
vo = VSENS(s, 'V_src', pcs.LConn(1), hn);
PROD(s, 'P_out', vo, 'I_cmd/1');
A(s, 'simulink/Math Operations/Gain', 'Grid', 'Gain', ['1/' cg '.eta']); L(s, 'P_out/1', 'Grid/1');
L(s, 'Grid/1', 'P_grid/1', 'P_grid');
end

function local_chg_cccv(v)
s = [v '/CCCV_BLOCK'];
cg = 'P.power.charger';
A(s, 'simulink/Sinks/Terminator', 'Term_cmd'); L(s, 'bms_cmd/1', 'Term_cmd/1', 'bms_cmd');
% trigger = dock contact AND battery voltage detected on the dock pins (K4 closed): the batt_lib Charger starts
% its CC-CV cycle on the rising trigger edge and would otherwise finish at once into the open K4 (bleed only)
CMP(s, 'Batt_detect', '>', ['0.5*' cg '.V_ovp']); L(s, 'V_fb/1', 'Batt_detect/1', 'V_fb');
LOG(s, 'Trigger', 'AND', 2); L(s, 'dock/1', 'Trigger/1', 'dock'); L(s, 'Batt_detect/1', 'Trigger/2', 'batt_detect');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'Trig_dbl', 'OutDataTypeStr', 'double');
L(s, 'Trigger/1', 'Trig_dbl/1', 'trigger');
pc = PH(A(s, 'batt_lib/Cyclers/Charger', 'CCCV', 'charging_current', [cg '.cccv_I'], 'voltage_threshold', [cg '.cccv_Vth'], ...
    'charging_voltage', [cg '.cccv_V'], 'current_threshold', [cg '.cccv_Ith']));
h = SPS(s, 'Trig_PS', 'Trig_dbl/1', '1', 'trigger_dbl'); PL(s, h, pc.LConn(1));
pd = PH(A(s, 'ee_lib/Semiconductors & Converters/Diode', 'D_out', 'Vf', [cg '.diode_Vf'], 'Ron', [cg '.diode_Ron'], ...
    'Goff', [cg '.diode_Goff']));
pb = PH(A(s, LIB('res'), 'R_bleed', 'R', [cg '.R_bleed']));
[io, ip, in] = ISENS(s, 'I_m');
hp = CPH([s '/chg_p']); hn = CPH([s '/chg_n']);
% batt_lib Charger ports: RConn = [+, -]
PL(s, pc.RConn(1), ip); PL(s, in, pd.LConn(1)); PL(s, pd.RConn(1), hp); PL(s, pc.RConn(2), hn);
PL(s, in, pb.LConn(1)); PL(s, pb.RConn(1), hn);
vo = VSENS(s, 'V_src', in, hn);
L(s, io, 'I_chg/1', 'I_chg');
PROD(s, 'P_out', vo, io);
A(s, 'simulink/Math Operations/Gain', 'Grid', 'Gain', ['1/' cg '.eta']); L(s, 'P_out/1', 'Grid/1');
L(s, 'Grid/1', 'P_grid/1', 'P_grid');
end

% ===========================================================================================
%  Power_Plant (ICD 3.2 Power row, build plan T05.1)
% ===========================================================================================
function local_plant(name, f)
local_fresh(name, f);
T = build_stubs('table');
R = T(strcmp({T.name}, 'Power_Plant'));
for k = 1:numel(R.in)
    IN(name, R.in(k).name, k, R.in(k).type, local_ts(R.in(k).ts));
end
for k = 1:numel(R.out)
    OUT(name, R.out(k).name, k, R.out(k).type);
end
H = struct();
for k = 1:numel(R.conn)
    b = A(name, LIB('cp'), R.conn(k).name, 'Port', num2str(k), 'Side', R.conn(k).side, 'ConnectionType', R.conn(k).type);
    H.(R.conn(k).name) = CPH(b);
end
% the root buses go whole into the blocks that pick from them (Pack_Thermal, Safety_Chain, LV_Distribution,
% Dock_Charger, Power_Sensors, Implement_Loads); a block that takes single signals gets a Bus Selector in front
% ---- Battery_Pack (VC_PACK) and its bus pack (a virtual Bus Creator at the producer) ----
v = local_variant(name, 'Battery_Pack', {'Bal_Cmd'}, {'T_cell_true', 'SOC_true', 'V_cell_true'}, ...
    {'pack_p', 'E', 'Right'; 'pack_n', 'E', 'Right'; 'AmbH', 'T', 'Left'}, ...
    {'Grouped', 'Pack_Grouped', 'VC_PACK == 1'; 'Lumped', 'Pack_Lumped', 'VC_PACK == 2'});
BS(name, 'Bms_sel', {'Bal_Cmd'}); L(name, 'bms_cmd/1', 'Bms_sel/1', 'bms_cmd');
L(name, 'Bms_sel/1', 'Battery_Pack/1');
BCV(name, 'Pack_BC', {'T_cell_true', 'Battery_Pack/1'; 'SOC_true', 'Battery_Pack/2'; 'V_cell_true', 'Battery_Pack/3'});
% ---- Pack_Thermal (picks bms_cmd.Fan_Cmd, env.T_amb, env.G_solar, env.dock_contact; sends the bus thermal) ----
A(name, LIB('ssr'), 'Pack_Thermal', 'ReferencedSubsystem', 'Pack_Thermal');
L(name, 'bms_cmd/1', 'Pack_Thermal/1', 'bms_cmd'); L(name, 'env/1', 'Pack_Thermal/2', 'env');
PL(name, shaddad_bu('pport', v, 'AmbH'), shaddad_bu('pport', [name '/Pack_Thermal'], 'AmbH'));
% ---- Safety_Chain (sends the bus safety) ----
A(name, LIB('ssr'), 'Safety_Chain', 'ReferencedSubsystem', 'Safety_Chain');
L(name, 'hmi/1', 'Safety_Chain/1', 'hmi'); L(name, 'hw_cmd/1', 'Safety_Chain/2', 'hw_cmd');
L(name, 'bms_cmd/1', 'Safety_Chain/3', 'bms_cmd'); L(name, 'env/1', 'Safety_Chain/4', 'env');
L(name, 'flt/1', 'Safety_Chain/5', 'flt');
% ---- PDU (picks safety.coil, safety.key_on; sends the bus pdu) ----
A(name, LIB('ssr'), 'PDU', 'ReferencedSubsystem', 'PDU');
L(name, 'Safety_Chain/1', 'PDU/1', 'safety');
PP = @(blk, port) shaddad_bu('pport', [name '/' blk], port);
PL(name, shaddad_bu('pport', v, 'pack_p'), PP('PDU', 'pack_p'));
PL(name, shaddad_bu('pport', v, 'pack_n'), PP('PDU', 'pack_n'));
for c = {'DC48_TracL_p', 'DC48_TracL_n', 'DC48_TracR_p', 'DC48_TracR_n'}
    PL(name, PP('PDU', c{1}), H.(c{1}));
end
% ---- LV_Distribution (picks thermal.Fan_eff, safety.coil; sends the bus lv) ----
A(name, LIB('ssr'), 'LV_Distribution', 'ReferencedSubsystem', 'LV_Distribution');
L(name, 'lv_demand/1', 'LV_Distribution/1', 'lv_demand'); L(name, 'Pack_Thermal/1', 'LV_Distribution/2', 'thermal');
L(name, 'Safety_Chain/1', 'LV_Distribution/3', 'safety');
PL(name, PP('PDU', 'AUX48_p'), PP('LV_Distribution', 'AUX48_p'));
PL(name, PP('PDU', 'AUX48_n'), PP('LV_Distribution', 'AUX48_n'));
% ---- Implement_Loads (VC_IMPL) and its bus impl ----
w = local_variant(name, 'Implement_Loads', {'can_rx', 'hw_cmd', 'Safety_OK', 'impl_mech'}, {'impl_elec', 'can_tx', 'P_impl'}, ...
    {'IMP48_p', 'E', 'Left'; 'IMP48_n', 'E', 'Left'; 'V24IMP_p', 'E', 'Left'; 'V24IMP_n', 'E', 'Left'; 'Impl_Shaft', 'M', 'Right'}, ...
    {'None', 'Impl_None', 'VC_IMPL == 0'; 'Mower', 'Impl_Mower', 'VC_IMPL == 1'; 'Tiller', 'Impl_Tiller', 'VC_IMPL == 2'; ...
     'Sprayer', 'Impl_Sprayer', 'VC_IMPL == 3'});
L(name, 'can_rx/1', 'Implement_Loads/1', 'can_rx'); L(name, 'hw_cmd/1', 'Implement_Loads/2', 'hw_cmd');
BS(name, 'Safety_sel', {'Safety_OK'}); L(name, 'Safety_Chain/1', 'Safety_sel/1', 'safety');
L(name, 'Safety_sel/1', 'Implement_Loads/3');
L(name, 'impl_mech/1', 'Implement_Loads/4', 'impl_mech');
PL(name, PP('PDU', 'IMP48_p'), shaddad_bu('pport', w, 'IMP48_p'));
PL(name, PP('PDU', 'IMP48_n'), shaddad_bu('pport', w, 'IMP48_n'));
PL(name, PP('LV_Distribution', 'V24IMP_p'), shaddad_bu('pport', w, 'V24IMP_p'));
PL(name, PP('LV_Distribution', 'V24IMP_n'), shaddad_bu('pport', w, 'V24IMP_n'));
PL(name, shaddad_bu('pport', w, 'Impl_Shaft'), H.Impl_Shaft);
BCV(name, 'Impl_BC', {'P_impl', 'Implement_Loads/3'});
% ---- Dock_Charger (picks env.dock_contact; sends the bus charger) ----
A(name, LIB('ssr'), 'Dock_Charger', 'ReferencedSubsystem', 'Dock_Charger');
L(name, 'bms_cmd/1', 'Dock_Charger/1', 'bms_cmd'); L(name, 'env/1', 'Dock_Charger/2', 'env');
PL(name, PP('PDU', 'DOCK_p'), PP('Dock_Charger', 'DOCK_p'));
PL(name, PP('PDU', 'DOCK_n'), PP('Dock_Charger', 'DOCK_n'));
% ---- Power_Sensors: one bus per source block, plus the root buses env and flt it picks from ----
A(name, LIB('ssr'), 'Power_Sensors', 'ReferencedSubsystem', 'Power_Sensors');
ps = {'Pack_BC/1', 'pack'; 'Pack_Thermal/1', 'thermal'; 'PDU/1', 'pdu'; 'LV_Distribution/1', 'lv'; ...
      'Safety_Chain/1', 'safety'; 'Impl_BC/1', 'impl'; 'Dock_Charger/1', 'charger'; 'env/1', 'env'; 'flt/1', 'flt'};
for k = 1:size(ps, 1)
    L(name, ps{k, 1}, sprintf('Power_Sensors/%d', k), ps{k, 2});
end
% ---- outputs ----
L(name, 'Power_Sensors/1', 'bms_meas/1', 'bms_meas');
L(name, 'Power_Sensors/2', 'power_status/1', 'power_status');
L(name, 'Implement_Loads/1', 'impl_elec/1', 'impl_elec');
L(name, 'Implement_Loads/2', 'can_tx/1', 'can_tx');
L(name, 'Power_Sensors/3', 'power_truth/1', 'power_truth');
BS(name, 'Safety_out_sel', {'Safety_OK'}); L(name, 'Safety_Chain/1', 'Safety_out_sel/1', 'safety');
L(name, 'Safety_out_sel/1', 'safety_ok/1');
local_save(name, f, ['Power_Plant (T05, build plan T05.1, ICD 3.2 Power row, spec 7.2). Physical ports DC48_TracL_p/n, ' ...
    'DC48_TracR_p/n (to DRIVE), Impl_Shaft (to STRUCTURE). Contents: Battery_Pack (VC_PACK: Pack_Grouped / Pack_Lumped), ' ...
    'Pack_Thermal (dock air on env.dock_contact), PDU (single Electrical Reference), Safety_Chain, LV_Distribution, ' ...
    'Implement_Loads (VC_IMPL 0..3: Impl_None / Impl_Mower / Impl_Tiller / Impl_Sprayer), Dock_Charger (VC_CHG), ' ...
    'Power_Sensors. Buses between them (model layout spec 6.3): pack (Pack_BC), thermal, safety, pdu, lv, impl ' ...
    '(Impl_BC), charger; Power_Sensors takes one bus per source block plus env and flt. No Solver Configuration (the ' ...
    'plant / bench adds it).']);
end

function s = local_ts(ts)
% port-rate expressions of the root ports (Ts.* fields)
switch ts
    case 0.001, s = 'Ts.base';
    case 0.01,  s = 'Ts.ctrl';
    case 0.05,  s = 'Ts.can_slow';
    case 0.1,   s = 'Ts.env';
    otherwise,  s = shaddad_bu('tsexpr', ts);
end
end
