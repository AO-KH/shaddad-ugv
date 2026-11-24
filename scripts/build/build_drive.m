function build_drive(parts, target)
%BUILD_DRIVE  Generate every traction-drive file of build task T06 (owner: drive).
%
%   build_drive()            build all files below (idempotent: each file is deleted and recreated)
%   build_drive(parts)       only the listed parts, e.g. build_drive({'esc','side'})
%
%   Parts (dependency order) and files (subsystems/drive/, all Subsystem References):
%     'codecs'      drive_cmd_pack, drive_cmd_unpack, drive_fb_pack, drive_fb_unpack   (CAN1 DRV frames,
%                   single/int only, inherited rate, reused by Shaddad_MCU; ICD 4)
%     'esc'         drive_esc        ESC firmware model at 1 ms (spec 2.9 / 3.5, MATLAB Function ESC_SM -> esc_sm.m;
%                                    D70: Torque_Limiter/LimitLogic -> esc_torque_limit.m, Feedback/FbPack ->
%                                    esc_feedback.m, each a verbatim copy, see MFB: rebuild after editing them)
%     'gearbox'     drive_gearbox    24:1 planetary, load-dependent mesh loss, no-load drag, J_out (spec 2.8)
%     'brake'       drive_brake      spring brake, elasto-plastic friction with variable normal force (spec 2.7,
%                                    Simscape component +shaddad_drive/elastoplastic_brake.ssc, D64)
%     'thermal'     drive_thermal    3-node thermal network, ESC heat split (spec 2.6)
%     'plant_sys'   drive_plant_sys  Motor & Drive (System Level, thermal) + DC link + aux loads (spec 3.1)
%     'plant_ideal' drive_plant_ideal  speed source + energy-equivalent DC load + network tie (spec 2.11, D35)
%     'plant_foc'   drive_plant_foc  PMSM + average-value inverter + current control (bench-only, D52)
%     'side'        drive_side       one side: ESC + plant variant VC_DRIVE_FID + gearbox + brake + thermal + truth
%     'system'      drive_system     both sides + CAN codecs + rate transitions (ICD 3.2 Drive row)
%
%   Needs P, Ts and Bus_* in the base workspace (runs shaddad_params() when missing).
%   Build plan rules: programmatic only (rule 3), library blocks first (rule 4), no Solver Configuration
%   inside domain files (rule 6), explicit Rate Transitions (rule 7), 1 ms Unit Delays on every ESC ->
%   plant command (rule 8, D36).
%   Layout (model layout spec 6 and 7): no builder sets a position; every file is laid out at every level by
%   shaddad_layout before it is saved (finish). Every signal line is named (L5). Each consumer of a bus picks its
%   elements right in front of it (spec 6.2): a Bus Selector of its own at a classic bus port (drive_fb_pack keeps
%   one Bus Selector for all fields, see local_fb_pack), an In Bus Element pick inside drive_esc, whose blocks
%   Cmd_In and ESC_SM send their outputs as the virtual buses cmd_in and sm (spec 6.3: each sends 3 or more signals
%   to one block). The root ports of every file are unchanged.

if nargin < 1 || isempty(parts)
    parts = {'codecs', 'esc', 'gearbox', 'brake', 'thermal', 'plant_sys', 'plant_ideal', 'plant_foc', 'side', 'system'};
end
if ischar(parts), parts = {parts}; end
if numel(parts) == 1 && strcmp(parts{1}, 'side_foc_body')
    % bench helper: build the drive_side contents with the FOC plant into an existing empty subsystem
    if ~evalin('base', 'exist(''P'',''var'')'), shaddad_params(); end
    local_side_body(target, 'foc');
    return
end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_DriveSideCmd'',''var'')')
    shaddad_params();
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
D = fullfile(root, 'subsystems', 'drive');
for k = 1:numel(parts)
    switch parts{k}
        case 'codecs'
            local_cmd_pack(D); local_cmd_unpack(D); local_fb_pack(D); local_fb_unpack(D);
        case 'esc',         local_esc(D);
        case 'gearbox',     local_gearbox(D);
        case 'brake',       local_brake(D);
        case 'thermal',     local_thermal(D);
        case 'plant_sys',   local_plant_sys(D);
        case 'plant_ideal', local_plant_ideal(D);
        case 'plant_foc',   local_plant_foc(D);
        case 'side',        local_side(D);
        case 'system',      local_system(D);
        otherwise, error('build_drive:part', 'Unknown part %s', parts{k});
    end
end
end

% =============================================================================================
% Generic helpers
% =============================================================================================
function p = A(lib, path, varargin)
% add_block (no position: the layout engine places every block when the file is saved, finish)
p = shaddad_bu('add', lib, path, varargin{:});
end

function L(sys, src, dst, name)
% signal into dst ('Blk/k'), named name (L5). src 'Blk/k': a line, without name the default of shaddad_bu('nline')
% (the name the source port's line already has, else the source block's name, '_k' for output k of a block with
% several outputs). src 'port.element': a pick of that element of the In Bus Element port right in front of dst
% (spec 6.2), its line named name (default the element), shaddad_bu('pick')
if nargin < 4, name = ''; end
if ~contains(src, '/')
    shaddad_bu('pick', sys, extractBefore(src, '.'), extractAfter(src, '.'), dst, name);
else
    shaddad_bu('nline', sys, src, dst, name);
end
end

function OB(sys, port, el, src, name)
% the element el of the Out Bus Element port 'port' of sys, fed by src ('Blk/k'), its line named name (default el)
% (spec 6.3): shaddad_bu('outbus')
if nargin < 5, name = ''; end
shaddad_bu('outbus', sys, port, el, src, name);
end

function SEL(sys, name, src, sigs, dsts)
% a Bus Selector name right in front of its consumer (spec 6.2, a bus at a classic port): it selects sigs from the
% bus src ('Blk/k'), output k into dsts{k} ('Blk/j')
BS(sys, name, sigs);
L(sys, src, [name '/1']);
for k = 1:numel(sigs), L(sys, sprintf('%s/%d', name, k), dsts{k}); end
end

function PORTS(sys, ins)
% number the inputs of the subsystem sys (classic Inports and In Bus Element ports) in the order of ins
shaddad_bu('portorder', sys, ins, 'Inport');
end

function PL(sys, h1, h2)
% physical line by port handles, no autorouting (smart routing can merge branches of stacked blocks)
add_line(sys, h1, h2);
end

function ph = H(blk)
ph = get_param(blk, 'PortHandles');
end

function h = CP(blk, port)
% handle of the physical port of an SSR / subsystem block that belongs to Connection Port 'port'
h = shaddad_bu('pport', blk, port);
end

function h = RC(sys, name)
% the single port handle of a Connection Port block at system level
ph = get_param([sys '/' name], 'PortHandles'); h = ph.RConn(1);
end

function fresh(name, file)
shaddad_bu('fresh', name, file, 'ssr');
end

function finish(name, file, desc)
% the layout engine at every level of the file (model layout spec 7; it keeps every physical node, and an engine
% error stops the build), then save
set_param(name, 'Description', desc);
shaddad_layout(name, 'Recursive', true);
save_system(name, file);
close_system(name, 0);
end

function s = mksub(parent, name, ins, outs)
% empty virtual subsystem with named Inports / Outports (bus element ports come with L picks and OB)
s = [parent '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', s);
c = find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all');
c = setdiff(c, {s});
lns = find_system(s, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
for k = 1:numel(ins), add_block('simulink/Sources/In1', [s '/' ins{k}]); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [s '/' outs{k}]); end
end

function inport(sys, name, k, type, ts)
A('simulink/Sources/In1', [sys '/' name], 'Port', num2str(k), 'OutDataTypeStr', type, 'SampleTime', ts);
end

function outport(sys, name, k, type)
A('simulink/Sinks/Out1', [sys '/' name], 'Port', num2str(k), 'OutDataTypeStr', type);
end

function cport(sys, name, k, side, dom)
switch dom
    case 'E', ct = 'Connection: foundation.electrical.electrical';
    case 'M', ct = 'Connection: foundation.mechanical.rotational.rotational';
    case 'T', ct = 'Connection: foundation.thermal.thermal';
end
A('nesl_utility/Connection Port', [sys '/' name], 'Port', num2str(k), 'Side', side, 'ConnectionType', ct);
end

function b = BS(sys, name, sigs)
b = A('simulink/Signal Routing/Bus Selector', [sys '/' name], 'OutputSignals', strjoin(sigs, ','));
end

function b = BC(sys, name, bus, n)
b = A('simulink/Signal Routing/Bus Creator', [sys '/' name], 'Inputs', num2str(n), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on');
end

function b = DTC(sys, name, type)
b = A('simulink/Signal Attributes/Data Type Conversion', [sys '/' name], 'OutDataTypeStr', type, ...
    'RndMeth', 'Nearest', 'SaturateOnIntegerOverflow', 'on');
end

function b = K(sys, name, g)
b = A('simulink/Math Operations/Gain', [sys '/' name], 'Gain', g);
end

function b = C(sys, name, v, type)
if nargin < 4, b = A('simulink/Sources/Constant', [sys '/' name], 'Value', v);
else, b = A('simulink/Sources/Constant', [sys '/' name], 'Value', v, 'OutDataTypeStr', type);
end
end

function b = SW(sys, name, crit, th)
if nargin < 3, crit = 'u2 ~= 0'; end
if nargin < 4, th = '0'; end
b = A('simulink/Signal Routing/Switch', [sys '/' name], 'Criteria', crit, 'Threshold', th);
end

function b = REL(sys, name, op, const)
b = A('simulink/Logic and Bit Operations/Compare To Constant', [sys '/' name], 'relop', op, 'const', const);
end

function b = LOG(sys, name, op, n)
if nargin < 4, n = 2; end
b = A('simulink/Logic and Bit Operations/Logical Operator', [sys '/' name], 'Operator', op, 'Inputs', num2str(n));
end

function b = SUM(sys, name, signs)
b = A('simulink/Math Operations/Sum', [sys '/' name], 'Inputs', signs);
end

function b = PROD(sys, name, ops)
if nargin < 3, ops = '**'; end
b = A('simulink/Math Operations/Product', [sys '/' name], 'Inputs', ops);
end

function b = MINMAX(sys, name, f, n)
b = A('simulink/Math Operations/MinMax', [sys '/' name], 'Function', f, 'Inputs', num2str(n));
end

function b = LUT1(sys, name, bp, tab)
b = A('simulink/Lookup Tables/1-D Lookup Table', [sys '/' name], 'BreakpointsForDimension1', bp, ...
    'Table', tab, 'ExtrapMethod', 'Clip', 'InterpMethod', 'Linear point-slope');
end

function b = UD(sys, name, ic, ts)
if nargin < 4, ts = '-1'; end
b = A('simulink/Discrete/Unit Delay', [sys '/' name], 'InitialCondition', ic, 'SampleTime', ts);
end

function b = S2PS(sys, name, unit)
% Simulink-PS converter preceded by a deterministic Rate Transition named 'name' (callers wire into it) whose
% output sample time is inherited from the network: a no-op at the 1 ms system rate, the explicit slow-to-fast
% transition into the 5e-5 s network of the FOC bench (rule 7)
A('simulink/Signal Attributes/Rate Transition', [sys '/' name], 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Inherit');
b = A('nesl_utility/Simulink-PS Converter', [sys '/' name '_PS'], 'Unit', unit, 'FilteringAndDerivatives', 'zero');
if strcmp(unit, 'K'), set_param(b, 'AffineConversion', 'on'); end   % absolute temperature (K, not delta K)
L(sys, [name '/1'], [name '_PS/1']);
end

function b = PS2S(sys, name, unit)
b = A('nesl_utility/PS-Simulink Converter', [sys '/' name], 'Unit', unit);
if strcmp(unit, 'K'), set_param(b, 'AffineConversion', 'on'); end   % absolute temperature (K, not delta K)
end

function b = MUX(sys, name, n)
b = A('simulink/Signal Routing/Mux', [sys '/' name], 'Inputs', num2str(n));
end

function b = DEMUX(sys, name, n)
b = A('simulink/Signal Routing/Demux', [sys '/' name], 'Outputs', num2str(n));
end

function b = ZOH(sys, name)
% plant -> 1 ms re-timing: deterministic Rate Transition (no-op at the 1 ms local solver rate, a ZOH for the
% continuous network of cs_SimRef runs, and fast-to-slow for the 5e-5 s FOC bench network)
b = A('simulink/Signal Attributes/Rate Transition', [sys '/' name], 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTime', 'P.drive.Ts_esc');
end

function b = BYTEPACK(sys, name, types)
b = A('mcblib/Signal Management/Byte Pack', [sys '/' name], 'maskPackedDataType', 'uint8', ...
    'datatypes', ['{' strjoin(cellfun(@(t) ['''' t ''''], types, 'UniformOutput', false), ',') '}'], 'byteAlign', '1');
end

function b = BYTEUNPACK(sys, name, types)
b = A('mcblib/Signal Management/Byte Unpack', [sys '/' name], ...
    'dimensions', ['{' strjoin(repmat({'1'}, 1, numel(types)), ',') '}'], ...
    'datatypes', ['{' strjoin(cellfun(@(t) ['''' t ''''], types, 'UniformOutput', false), ',') '}'], 'byteAlign', '1');
end

function b = BITS(sys, name, rng)
b = A('simulink/Logic and Bit Operations/Extract Bits', [sys '/' name], 'bitsToExtract', 'Range of bits', ...
    'bitIdxRange', rng, 'outScalingMode', 'Treat bit field as an integer');
end

function b = PROT(sys, name, prot, cmax)
b = A('mcblib/Protection and Diagnostics/Protection Relay', [sys '/' name], 'Protection', prot, ...
    'CountMaxInputType', 'Specify via dialog', 'CountMax', cmax, 'SamplesForDebounce', '0');
end

function mf = MFB(sys, name, header, fcn, D, desc, outTypes)
% MATLAB Function block name (D70) whose body is one call of the %#codegen file subsystems/drive/<fcn>.m: header is
% the block's own function (its arguments name the ports, in order) with the call. R2026b workaround (as
% build_structure MFB and build_sim_monitors local_foreach): drive_esc sits three Subsystem Reference levels below
% Shaddad_Plant, where a MATLAB Function that calls an external file, or holds parameter data, trips the Stateflow
% JIT relink assertion 'o != nullptr' on the first compile after a load. So the block holds a verbatim copy of the
% file as a local function of the same name (the file stays the single source: after an edit rebuild with
% build_drive({'esc'}); t_mf_equiv/testDriveEscUsesFunctions checks that the copies are current), and every constant
% enters on an input from a Constant block (never the whole P, D58). Integer casts saturate, as the Data Type
% Conversions they replace. outTypes {output, type; ...}: typed outputs (a bus).
mf = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', mf);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
body = strrep(fileread(fullfile(D, [fcn '.m'])), sprintf('\r\n'), newline);
cfg.FunctionScript = [header sprintf(['end\n\n%% ---- verbatim copy of subsystems/drive/%s.m (generated by ' ...
    'build_drive, do not edit here) ----\n'], fcn) body];
cfg.UpdateMethod = 'Inherited';
cfg.SaturateOnIntegerOverflow = true;
cfg.Description = desc;
if nargin > 6
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', mf);
    for k = 1:size(outTypes, 1)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', outTypes{k, 1});
        d.DataType = outTypes{k, 2};
    end
end
end

% =============================================================================================
% CAN codecs (ICD 4: DRV_CMD 0x200+n, DRV_FB1 0x180+n, DRV_FB2 0x280+n, DRV_FB3 0x380+n)
% =============================================================================================
function local_frame_out(s, prefix, idExpr, dataSrc, outName)
% id = uint32(idExpr) + uint32(node), len = 8, valid = true -> Bus_CanFrame8
C(s, [prefix 'IdBase'], idExpr, 'uint32');
DTC(s, [prefix 'NodeU32'], 'uint32');
A('simulink/Math Operations/Sum', [s '/' prefix 'Id'], 'Inputs', '++', 'OutDataTypeStr', 'uint32', ...
    'SaturateOnIntegerOverflow', 'on');
C(s, [prefix 'Len'], 'P.drive.can.len', 'uint8');
C(s, [prefix 'Valid'], 'true', 'boolean');
BC(s, [prefix 'Frame'], 'Bus_CanFrame8', 4);
L(s, 'node/1', [prefix 'NodeU32/1']);
L(s, [prefix 'IdBase/1'], [prefix 'Id/1']);
L(s, [prefix 'NodeU32/1'], [prefix 'Id/2']);
L(s, [prefix 'Id/1'], [prefix 'Frame/1'], 'id');
L(s, [prefix 'Len/1'], [prefix 'Frame/2'], 'len');
L(s, dataSrc, [prefix 'Frame/3'], 'data');
L(s, [prefix 'Valid/1'], [prefix 'Frame/4'], 'valid');
L(s, [prefix 'Frame/1'], [outName '/1'], outName);
end

function local_frame_in(s, prefix, frameIn, idExpr)
% ok = valid && id == idExpr + node; payload held while not ok -> [prefix 'Held'] (uint8[8]); id, valid and data
% each picked by a Bus Selector right in front of its consumer (spec 6.2)
C(s, [prefix 'IdBase'], idExpr, 'uint32');
DTC(s, [prefix 'NodeU32'], 'uint32');
A('simulink/Math Operations/Sum', [s '/' prefix 'IdExp'], 'Inputs', '++', 'OutDataTypeStr', 'uint32', ...
    'SaturateOnIntegerOverflow', 'on');
A('simulink/Logic and Bit Operations/Relational Operator', [s '/' prefix 'IdOk'], 'Operator', '==');
LOG(s, [prefix 'Ok'], 'AND');
SW(s, [prefix 'Held'], 'u2 ~= 0');
UD(s, [prefix 'Last'], 'zeros(8,1)');
L(s, 'node/1', [prefix 'NodeU32/1']);
L(s, [prefix 'IdBase/1'], [prefix 'IdExp/1']);
L(s, [prefix 'NodeU32/1'], [prefix 'IdExp/2']);
SEL(s, [prefix 'SelId'], [frameIn '/1'], {'id'}, {[prefix 'IdOk/1']});
L(s, [prefix 'IdExp/1'], [prefix 'IdOk/2']);
L(s, [prefix 'IdOk/1'], [prefix 'Ok/1']);
SEL(s, [prefix 'SelValid'], [frameIn '/1'], {'valid'}, {[prefix 'Ok/2']});
SEL(s, [prefix 'SelData'], [frameIn '/1'], {'data'}, {[prefix 'Held/1']});
L(s, [prefix 'Ok/1'], [prefix 'Held/2']);
L(s, [prefix 'Last/1'], [prefix 'Held/3']);
L(s, [prefix 'Held/1'], [prefix 'Last/1']);
end

function local_cmd_pack(D)
n = 'drive_cmd_pack'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'cmd', 1, 'Bus: Bus_DriveSideCmd', '-1');
inport(n, 'node', 2, 'uint8', '-1');
outport(n, 'frame', 1, 'Bus: Bus_CanFrame8');
% ctrl word: b0 enable, b1 park_req, b2 fault_reset, b3-4 mode, b8-15 alive; each field picked by a Bus Selector
% right in front of its consumer (spec 6.2)
A('simulink/Logic and Bit Operations/Bitwise Operator', [n '/ModeMask'], 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '3', 'NumInputPorts', '1');
nm = {'b_enable', 'b_park', 'b_reset', 'b_mode', 'b_alive'};
for k = 1:5, DTC(n, nm{k}, 'uint16'); end
MUX(n, 'CtrlMux', 5);
A('simulink/Logic and Bit Operations/Shift Arithmetic', [n '/CtrlShift'], 'BitShiftNumber', '[0 1 2 3 8]', ...
    'BitShiftDirection', 'Left');
A('simulink/Logic and Bit Operations/Bitwise Operator', [n '/Ctrl'], 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '1');
SEL(n, 'Sel_enable', 'cmd/1', {'enable'}, {'b_enable/1'}); SEL(n, 'Sel_park_req', 'cmd/1', {'park_req'}, {'b_park/1'});
SEL(n, 'Sel_fault_reset', 'cmd/1', {'fault_reset'}, {'b_reset/1'});
SEL(n, 'Sel_mode', 'cmd/1', {'mode'}, {'ModeMask/1'}); L(n, 'ModeMask/1', 'b_mode/1');
SEL(n, 'Sel_alive', 'cmd/1', {'alive'}, {'b_alive/1'});
for k = 1:5, L(n, [nm{k} '/1'], ['CtrlMux/' num2str(k)]); end
L(n, 'CtrlMux/1', 'CtrlShift/1', 'ctrl_bits'); L(n, 'CtrlShift/1', 'Ctrl/1', 'ctrl_shifted');
% scaled fields
K(n, 'Kw', 'single(1/P.drive.can.res_w)');       DTC(n, 'w_i16', 'int16');
K(n, 'Kff', 'single(1/P.drive.can.res_tau)');    DTC(n, 'ff_i16', 'int16');
K(n, 'Klm', 'single(1/P.drive.can.res_tau_lim)'); DTC(n, 'lm_u8', 'uint8');
K(n, 'Klr', 'single(1/P.drive.can.res_tau_lim)'); DTC(n, 'lr_u8', 'uint8');
SEL(n, 'Sel_w_ref', 'cmd/1', {'w_ref'}, {'Kw/1'}); L(n, 'Kw/1', 'w_i16/1');
SEL(n, 'Sel_tau_ff', 'cmd/1', {'tau_ff'}, {'Kff/1'}); L(n, 'Kff/1', 'ff_i16/1');
SEL(n, 'Sel_tau_lim_mot', 'cmd/1', {'tau_lim_mot'}, {'Klm/1'}); L(n, 'Klm/1', 'lm_u8/1');
SEL(n, 'Sel_tau_lim_reg', 'cmd/1', {'tau_lim_reg'}, {'Klr/1'}); L(n, 'Klr/1', 'lr_u8/1');
BYTEPACK(n, 'Pack', {'uint16', 'int16', 'int16', 'uint8', 'uint8'});
L(n, 'Ctrl/1', 'Pack/1', 'ctrl'); L(n, 'w_i16/1', 'Pack/2'); L(n, 'ff_i16/1', 'Pack/3');
L(n, 'lm_u8/1', 'Pack/4'); L(n, 'lr_u8/1', 'Pack/5');
local_frame_out(n, '', 'P.drive.can.id_cmd', 'Pack/1', 'frame');
finish(n, f, ['DRV_CMD codec (ICD 4, 0x200+node): Bus_DriveSideCmd -> Bus_CanFrame8. u16 ctrl [b0 enable, b1 park_req, ' ...
    'b2 fault_reset, b3-4 mode, b8-15 alive], i16 w_ref [1 rpm], i16 tau_ff [0.01 N m], u8 tau_lim_mot / tau_lim_reg [0.1 N m]. ' ...
    'Inputs: cmd, node (uint8 1 L / 2 R). Inherited rate, single/int only (reused by Shaddad_MCU). Owner drive (T06).']);
end

function local_cmd_unpack(D)
n = 'drive_cmd_unpack'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'frame', 1, 'Bus: Bus_CanFrame8', '-1');
inport(n, 'node', 2, 'uint8', '-1');
outport(n, 'cmd', 1, 'Bus: Bus_DriveSideCmd');
local_frame_in(n, 'F', 'frame', 'P.drive.can.id_cmd');
BYTEUNPACK(n, 'Unpack', {'uint16', 'int16', 'int16', 'uint8', 'uint8'});
L(n, 'FHeld/1', 'Unpack/1', 'payload');
BITS(n, 'b_enable', '[0 0]'); BITS(n, 'b_park', '[1 1]'); BITS(n, 'b_reset', '[2 2]');
BITS(n, 'b_mode', '[3 4]'); BITS(n, 'b_alive', '[8 15]');
DTC(n, 'enable', 'boolean'); DTC(n, 'park_req', 'boolean'); DTC(n, 'fault_reset', 'boolean');
DTC(n, 'mode', 'uint8'); DTC(n, 'alive', 'uint8');
pairs = {'b_enable', 'enable'; 'b_park', 'park_req'; 'b_reset', 'fault_reset'; 'b_mode', 'mode'; 'b_alive', 'alive'};
for k = 1:5
    L(n, 'Unpack/1', [pairs{k, 1} '/1'], 'ctrl'); L(n, [pairs{k, 1} '/1'], [pairs{k, 2} '/1']);
end
sc = {'w_ref', 'P.drive.can.res_w', 2, 'w_i16'; 'tau_ff', 'P.drive.can.res_tau', 3, 'ff_i16'; ...
      'tau_lim_mot', 'P.drive.can.res_tau_lim', 4, 'lm_u8'; 'tau_lim_reg', 'P.drive.can.res_tau_lim', 5, 'lr_u8'};
for k = 1:4
    DTC(n, [sc{k, 1} '_s'], 'single');
    A('simulink/Math Operations/Gain', [n '/' sc{k, 1}], 'Gain', ['single(' sc{k, 2} ')'], 'OutDataTypeStr', 'single', ...
        'ParamDataTypeStr', 'single');
    L(n, ['Unpack/' num2str(sc{k, 3})], [sc{k, 1} '_s/1'], sc{k, 4}); L(n, [sc{k, 1} '_s/1'], [sc{k, 1} '/1']);
end
BC(n, 'Cmd', 'Bus_DriveSideCmd', 9);
el = {'enable', 'mode', 'w_ref', 'tau_ff', 'tau_lim_mot', 'tau_lim_reg', 'park_req', 'fault_reset', 'alive'};
for k = 1:9, L(n, [el{k} '/1'], ['Cmd/' num2str(k)], el{k}); end
L(n, 'Cmd/1', 'cmd/1', 'cmd');
finish(n, f, ['DRV_CMD decoder (ICD 4): Bus_CanFrame8 -> Bus_DriveSideCmd. A frame with valid = false or an id other than ' ...
    '0x200+node is ignored and the last payload is held (invalid frame = hold last). Inherited rate, single/int only. Owner drive (T06).']);
end

function local_fb_pack(D)
n = 'drive_fb_pack'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'fb', 1, 'Bus: Bus_DriveSideFb', '-1');
inport(n, 'node', 2, 'uint8', '-1');
outport(n, 'fb1', 1, 'Bus: Bus_CanFrame8');
outport(n, 'fb2', 2, 'Bus: Bus_CanFrame8');
outport(n, 'fb3', 3, 'Bus: Bus_CanFrame8');
% one Bus Selector picks every field, in the order of the frames and their byte-pack inputs: the port fb keeps one
% consumer, because a Bus Selector per consumer makes fb branch like node, and the layout engine then runs the
% trunks of the two port lines on one track (a line overlap; the engine offsets the trunks of signal blocks only)
sig = {'status', 'w_m', 'tau_est', 'i_dc', 'v_dc', 'i_q', 'T_motor', 'T_esc', 'tau_avail', 'enc_cnt', 'fault_code', ...
    'alive', 'state'};
BS(n, 'Sel', sig);
L(n, 'fb/1', 'Sel/1');
idx = @(s) num2str(find(strcmp(sig, s)));
% FB1: u16 status, i16 speed [1 rpm], i16 tau_est [0.01 N m], i16 i_dc [0.1 A]
K(n, 'Kw', 'single(1/P.drive.can.res_w)');     DTC(n, 'w_i16', 'int16');
K(n, 'Ktau', 'single(1/P.drive.can.res_tau)'); DTC(n, 'tau_i16', 'int16');
K(n, 'Kidc', 'single(1/P.drive.can.res_i)');   DTC(n, 'idc_i16', 'int16');
L(n, ['Sel/' idx('w_m')], 'Kw/1');       L(n, 'Kw/1', 'w_i16/1');
L(n, ['Sel/' idx('tau_est')], 'Ktau/1'); L(n, 'Ktau/1', 'tau_i16/1');
L(n, ['Sel/' idx('i_dc')], 'Kidc/1');    L(n, 'Kidc/1', 'idc_i16/1');
BYTEPACK(n, 'Pack1', {'uint16', 'int16', 'int16', 'int16'});
L(n, ['Sel/' idx('status')], 'Pack1/1'); L(n, 'w_i16/1', 'Pack1/2'); L(n, 'tau_i16/1', 'Pack1/3'); L(n, 'idc_i16/1', 'Pack1/4');
local_frame_out(n, 'F1', 'P.drive.can.id_fb1', 'Pack1/1', 'fb1');
% FB2: u16 v_dc [0.01 V], i16 i_q [0.1 A], i16 T_motor [0.1 degC], u8 T_esc [1 degC, -40], u8 tau_avail [0.1 N m]
K(n, 'Kv', 'single(1/P.drive.can.res_v)');      DTC(n, 'v_u16', 'uint16');
K(n, 'Kiq', 'single(1/P.drive.can.res_i)');     DTC(n, 'iq_i16', 'int16');
A('simulink/Math Operations/Bias', [n '/Tm_degC'], 'Bias', 'single(-P.drive.can.T0)');
K(n, 'KTm', 'single(1/P.drive.can.res_T)');     DTC(n, 'Tm_i16', 'int16');
A('simulink/Math Operations/Bias', [n '/Te_ofs'], 'Bias', 'single(-P.drive.can.T0 + P.drive.can.T_esc_ofs)');
DTC(n, 'Te_u8', 'uint8');
K(n, 'Kav', 'single(1/P.drive.can.res_tau_lim)'); DTC(n, 'av_u8', 'uint8');
L(n, ['Sel/' idx('v_dc')], 'Kv/1');         L(n, 'Kv/1', 'v_u16/1');
L(n, ['Sel/' idx('i_q')], 'Kiq/1');         L(n, 'Kiq/1', 'iq_i16/1');
L(n, ['Sel/' idx('T_motor')], 'Tm_degC/1'); L(n, 'Tm_degC/1', 'KTm/1'); L(n, 'KTm/1', 'Tm_i16/1');
L(n, ['Sel/' idx('T_esc')], 'Te_ofs/1');    L(n, 'Te_ofs/1', 'Te_u8/1');
L(n, ['Sel/' idx('tau_avail')], 'Kav/1');   L(n, 'Kav/1', 'av_u8/1');
BYTEPACK(n, 'Pack2', {'uint16', 'int16', 'int16', 'uint8', 'uint8'});
L(n, 'v_u16/1', 'Pack2/1'); L(n, 'iq_i16/1', 'Pack2/2'); L(n, 'Tm_i16/1', 'Pack2/3');
L(n, 'Te_u8/1', 'Pack2/4'); L(n, 'av_u8/1', 'Pack2/5');
local_frame_out(n, 'F2', 'P.drive.can.id_fb2', 'Pack2/1', 'fb2');
% FB3: i32 enc_cnt, u16 fault_code, u8 alive, u8 state
BYTEPACK(n, 'Pack3', {'int32', 'uint16', 'uint8', 'uint8'});
L(n, ['Sel/' idx('enc_cnt')], 'Pack3/1'); L(n, ['Sel/' idx('fault_code')], 'Pack3/2');
L(n, ['Sel/' idx('alive')], 'Pack3/3');   L(n, ['Sel/' idx('state')], 'Pack3/4');
local_frame_out(n, 'F3', 'P.drive.can.id_fb3', 'Pack3/1', 'fb3');
finish(n, f, ['DRV_FB codec (ICD 4): Bus_DriveSideFb -> FB1 (0x180+node: u16 status, i16 speed 1 rpm, i16 tau_est 0.01 N m, ' ...
    'i16 i_dc 0.1 A), FB2 (0x280+node: u16 v_dc 0.01 V, i16 i_q 0.1 A, i16 T_motor 0.1 degC, u8 T_esc 1 degC offset -40, ' ...
    'u8 tau_avail 0.1 N m), FB3 (0x380+node: i32 enc_cnt, u16 fault_code, u8 alive, u8 state). Inherited rate (drive_system: ' ...
    'FB1/FB3 at 0.01 s, FB2 rate-transitioned to 0.05 s). Owner drive (T06).']);
end

function local_fb_unpack(D)
n = 'drive_fb_unpack'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'fb1', 1, 'Bus: Bus_CanFrame8', '-1');
inport(n, 'fb2', 2, 'Bus: Bus_CanFrame8', '-1');
inport(n, 'fb3', 3, 'Bus: Bus_CanFrame8', '-1');
inport(n, 'node', 4, 'uint8', '-1');
outport(n, 'fb', 1, 'Bus: Bus_DriveSideFb');
local_frame_in(n, 'F1', 'fb1', 'P.drive.can.id_fb1');
local_frame_in(n, 'F2', 'fb2', 'P.drive.can.id_fb2');
local_frame_in(n, 'F3', 'fb3', 'P.drive.can.id_fb3');
% frame ages (samples since the last accepted frame; caller runs this codec at P.drive.Ts_can)
for fr = {'F1', 'F3'}
    p = fr{1};
    C(n, [p 'Zero'], '0', 'uint8');
    C(n, [p 'One'], '1', 'uint8');
    A('simulink/Math Operations/Sum', [n '/' p 'AgeInc'], 'Inputs', '++', 'OutDataTypeStr', 'uint8', ...
        'SaturateOnIntegerOverflow', 'on');
    SW(n, [p 'Age'], 'u2 ~= 0');
    UD(n, [p 'AgeLast'], 'P.drive.can.n_rx_timeout');
    A('simulink/Logic and Bit Operations/Compare To Constant', [n '/' p 'Fresh'], 'relop', '<', ...
        'const', 'P.drive.can.n_rx_timeout');
    L(n, [p 'Zero/1'], [p 'Age/1']);
    L(n, [p 'Ok/1'], [p 'Age/2']);
    L(n, [p 'AgeLast/1'], [p 'AgeInc/1']); L(n, [p 'One/1'], [p 'AgeInc/2']);
    L(n, [p 'AgeInc/1'], [p 'Age/3']);
    L(n, [p 'Age/1'], [p 'AgeLast/1']);
    L(n, [p 'Age/1'], [p 'Fresh/1']);
end
LOG(n, 'rx_ok', 'AND');
L(n, 'F1Fresh/1', 'rx_ok/1'); L(n, 'F3Fresh/1', 'rx_ok/2');
% FB1
BYTEUNPACK(n, 'Unpack1', {'uint16', 'int16', 'int16', 'int16'});
L(n, 'F1Held/1', 'Unpack1/1', 'payload1');
BITS(n, 'b_brake', '[4 4]'); DTC(n, 'brake_released', 'boolean');
L(n, 'Unpack1/1', 'b_brake/1'); L(n, 'b_brake/1', 'brake_released/1');
sc = {'w_m', 'P.drive.can.res_w', 'Unpack1/2', 'w_i16'; 'tau_est', 'P.drive.can.res_tau', 'Unpack1/3', 'tau_i16'; ...
      'i_dc', 'P.drive.can.res_i', 'Unpack1/4', 'idc_i16'; 'v_dc', 'P.drive.can.res_v', 'Unpack2/1', 'v_u16'; ...
      'i_q', 'P.drive.can.res_i', 'Unpack2/2', 'iq_i16'; 'T_motor_c', 'P.drive.can.res_T', 'Unpack2/3', 'Tm_i16'; ...
      'tau_avail', 'P.drive.can.res_tau_lim', 'Unpack2/5', 'av_u8'};
BYTEUNPACK(n, 'Unpack2', {'uint16', 'int16', 'int16', 'uint8', 'uint8'});
L(n, 'F2Held/1', 'Unpack2/1', 'payload2');
for k = 1:size(sc, 1)
    DTC(n, [sc{k, 1} '_s'], 'single');
    A('simulink/Math Operations/Gain', [n '/' sc{k, 1}], 'Gain', ['single(' sc{k, 2} ')'], 'OutDataTypeStr', 'single', ...
        'ParamDataTypeStr', 'single');
    L(n, sc{k, 3}, [sc{k, 1} '_s/1'], sc{k, 4}); L(n, [sc{k, 1} '_s/1'], [sc{k, 1} '/1']);
end
A('simulink/Math Operations/Bias', [n '/T_motor'], 'Bias', 'single(P.drive.can.T0)');
L(n, 'T_motor_c/1', 'T_motor/1');
DTC(n, 'T_esc_s', 'single');
A('simulink/Math Operations/Bias', [n '/T_esc'], 'Bias', 'single(P.drive.can.T0 - P.drive.can.T_esc_ofs)');
L(n, 'Unpack2/4', 'T_esc_s/1', 'Te_u8'); L(n, 'T_esc_s/1', 'T_esc/1');
% FB3
BYTEUNPACK(n, 'Unpack3', {'int32', 'uint16', 'uint8', 'uint8'});
L(n, 'F3Held/1', 'Unpack3/1', 'payload3');
BC(n, 'Fb', 'Bus_DriveSideFb', 15);
src = {'Unpack3/4', 'state'; 'Unpack1/1', 'status'; 'Unpack3/2', 'fault_code'; 'w_m/1', 'w_m'; 'Unpack3/1', 'enc_cnt'; ...
       'tau_est/1', 'tau_est'; 'tau_avail/1', 'tau_avail'; 'i_q/1', 'i_q'; 'i_dc/1', 'i_dc'; 'v_dc/1', 'v_dc'; ...
       'T_motor/1', 'T_motor'; 'T_esc/1', 'T_esc'; 'brake_released/1', 'brake_released'; 'Unpack3/3', 'alive'; 'rx_ok/1', 'rx_ok'};
for k = 1:15, L(n, src{k, 1}, ['Fb/' num2str(k)], src{k, 2}); end
L(n, 'Fb/1', 'fb/1', 'fb');
finish(n, f, ['DRV_FB decoder (ICD 4): FB1/FB2/FB3 frames -> Bus_DriveSideFb. Each frame payload is held while invalid; ' ...
    'rx_ok = FB1 and FB3 younger than P.drive.can.n_rx_timeout samples (= t_rx_timeout 50 ms when run at P.drive.Ts_can 10 ms); ' ...
    'brake_released = status bit 4. Inputs fb1, fb2 (re-timed to the caller rate), fb3, node. Owner drive (T06).']);
end

% =============================================================================================
% ESC firmware model (spec 2.9 / 3.5, build plan T06.2), 1 ms
% =============================================================================================
function local_esc(D)
n = 'drive_esc'; f = fullfile(D, [n '.slx']); fresh(n, f);
TS = 'P.drive.Ts_esc';
inport(n, 'cmd', 1, 'Bus: Bus_DriveSideCmd', TS);
inport(n, 'sto_ok', 2, 'boolean', TS);
inport(n, 'fault_inj', 3, 'Bus: Bus_DriveSideFaultInj', TS);
inport(n, 'theta_m', 4, 'double', TS);
inport(n, 'v_dc', 5, 'double', TS);
inport(n, 'i_dc', 6, 'double', TS);
inport(n, 'T_w', 7, 'double', TS);
inport(n, 'T_esc', 8, 'double', TS);
outport(n, 'plant_cmd', 1, 'double');
outport(n, 'P_esc', 2, 'double');
outport(n, 'fb', 3, 'Bus: Bus_DriveSideFb');

% ---------------- Cmd_In: CAN command fields, watchdog, fault injection -> bus cmd_in ----------------
% Cmd_In sends 3 or more signals to ESC_SM, Protections and Speed_Ctrl, so its 13 outputs travel as the virtual
% bus cmd_in (Out Bus Element port, spec 6.3); every block picks its elements (spec 6.2)
s = mksub(n, 'Cmd_In', {'cmd', 'fault_inj'}, {});
BS(s, 'SelCmd', {'enable', 'mode', 'w_ref', 'tau_ff', 'tau_lim_mot', 'tau_lim_reg', 'park_req', 'fault_reset', 'alive'});
BS(s, 'SelFi', {'can_drop', 'enc_fail', 'brake_stuck', 'tau_scale', 'T_offset'});
L(s, 'cmd/1', 'SelCmd/1'); L(s, 'fault_inj/1', 'SelFi/1');
OB(s, 'cmd_in', 'enable', 'SelCmd/1');
dbl = {'mode', 2; 'w_ref', 3; 'tau_ff', 4; 'tau_lim_mot', 5; 'tau_lim_reg', 6};
for k = 1:size(dbl, 1)
    DTC(s, ['d_' dbl{k, 1}], 'double');
    L(s, ['SelCmd/' num2str(dbl{k, 2})], ['d_' dbl{k, 1} '/1']); OB(s, 'cmd_in', dbl{k, 1}, ['d_' dbl{k, 1} '/1']);
end
OB(s, 'cmd_in', 'park_req', 'SelCmd/7');
A('simulink/Logic and Bit Operations/Detect Rise Positive', [s '/ResetEdge'], 'vinit', '0');
L(s, 'SelCmd/8', 'ResetEdge/1'); OB(s, 'cmd_in', 'reset_edge', 'ResetEdge/1');
% CAN timeout: Detect Change(alive) resets the Software Watchdog Timer (maxCount 100 at 1 ms)
A('simulink/Logic and Bit Operations/Detect Change', [s '/AliveChange'], 'vinit', '0');
A('mcblib/Protection and Diagnostics/Software Watchdog Timer', [s '/CanWatchdog'], ...
    'maxCountInput', 'Specify via dialog', 'maxCount', 'P.drive.esc.n_can_to');
LOG(s, 'CanTimeout', 'NOT', 1);
L(s, 'SelCmd/9', 'AliveChange/1'); L(s, 'AliveChange/1', 'CanWatchdog/1'); L(s, 'CanWatchdog/1', 'CanTimeout/1');
OB(s, 'cmd_in', 'can_to', 'CanTimeout/1');
A('simulink/Sinks/Terminator', [s '/TermDrop']);
L(s, 'SelFi/1', 'TermDrop/1');
OB(s, 'cmd_in', 'enc_fail', 'SelFi/2'); OB(s, 'cmd_in', 'brake_stuck', 'SelFi/3');
OB(s, 'cmd_in', 'tau_scale', 'SelFi/4'); OB(s, 'cmd_in', 'T_offset', 'SelFi/5');

% ---------------- Speed_Meas: encoder quantisation, 4-sample difference, LPF, counter ----------------
s = mksub(n, 'Speed_Meas', {'theta_m'}, {'w_meas', 'enc_cnt'});
A('simulink/Discontinuities/Quantizer', [s '/Encoder'], 'QuantizationInterval', '2*pi/P.drive.esc.enc_cpr', 'SampleTime', TS);
SW(s, 'EncLost', 'u2 ~= 0');
UD(s, 'ThetaLast', '0', TS);
A('simulink/Discrete/Delay', [s '/Delay4'], 'DelayLength', 'P.drive.esc.n_diff', 'InitialCondition', '0', 'SampleTime', TS);
SUM(s, 'dTheta', '+-');
K(s, 'PerTime', '1/(P.drive.esc.n_diff*P.drive.Ts_esc)');
A('simulink/Discrete/Discrete Transfer Fcn', [s '/SpeedLPF'], 'Numerator', '[1-P.drive.esc.a_wf]', ...
    'Denominator', '[1 -P.drive.esc.a_wf]', 'SampleTime', TS);
L(s, 'theta_m/1', 'Encoder/1');
L(s, 'ThetaLast/1', 'EncLost/1'); L(s, 'cmd_in.enc_fail', 'EncLost/2'); L(s, 'Encoder/1', 'EncLost/3');
L(s, 'EncLost/1', 'ThetaLast/1', 'theta_enc');
L(s, 'EncLost/1', 'dTheta/1'); L(s, 'EncLost/1', 'Delay4/1'); L(s, 'Delay4/1', 'dTheta/2');
L(s, 'dTheta/1', 'PerTime/1'); L(s, 'PerTime/1', 'SpeedLPF/1'); L(s, 'SpeedLPF/1', 'w_meas/1', 'w_meas');
% encoder counter, int32 with wrap: mod(round(theta/q) + 2^31, 2^32) - 2^31
K(s, 'Counts', 'P.drive.esc.enc_cpr/(2*pi)');
A('simulink/Math Operations/Rounding Function', [s '/RoundCnt'], 'Operator', 'round');
A('simulink/Math Operations/Bias', [s '/Ofs'], 'Bias', '2^31');
C(s, 'Mod32', '2^32');
A('simulink/Math Operations/Math Function', [s '/Wrap'], 'Operator', 'mod');
A('simulink/Math Operations/Bias', [s '/Unofs'], 'Bias', '-2^31');
DTC(s, 'CntI32', 'int32');
L(s, 'EncLost/1', 'Counts/1'); L(s, 'Counts/1', 'RoundCnt/1'); L(s, 'RoundCnt/1', 'Ofs/1');
L(s, 'Ofs/1', 'Wrap/1'); L(s, 'Mod32/1', 'Wrap/2'); L(s, 'Wrap/1', 'Unofs/1'); L(s, 'Unofs/1', 'CntI32/1');
L(s, 'CntI32/1', 'enc_cnt/1', 'enc_cnt');
PORTS(s, {'theta_m', 'cmd_in'});

% ---------------- Sensors: winding sensor lag + offset ----------------
s = mksub(n, 'Temp_Sens', {'T_w'}, {'T_mot_meas'});
A('simulink/Discrete/Discrete Transfer Fcn', [s '/WindingLag'], 'Numerator', '[1-P.drive.esc.a_Ts]', ...
    'Denominator', '[1 -P.drive.esc.a_Ts]', 'SampleTime', TS, 'InitialStates', 'P.drive.thermal.T_amb0/(1-P.drive.esc.a_Ts)');
SUM(s, 'AddOfs', '++');
L(s, 'T_w/1', 'WindingLag/1'); L(s, 'WindingLag/1', 'AddOfs/1'); L(s, 'cmd_in.T_offset', 'AddOfs/2');
L(s, 'AddOfs/1', 'T_mot_meas/1', 'T_mot_meas');
PORTS(s, {'T_w', 'cmd_in'});

% ---------------- Torque_Limiter (spec 2.9): MATLAB Function LimitLogic (D70) calling esc_torque_limit.m ----------
% The quadrant limits, the I2t rate and the DERATING flag are one function; the Lookup Tables, the I2t integrator
% and its Relay stay blocks around it (D70), and so do AbsW and ToRpm, the speed in rpm in front of the Lookup Tables
% (inside the function it would close an algebraic loop through them). The integrator (Forward Euler) has no direct
% feedthrough, so LimitLogic -> I2t -> Exhausted -> LimitLogic is no algebraic loop, and I2t and Exhausted see the
% same input at the same step as with the former block network.
s = mksub(n, 'Torque_Limiter', {'w_meas', 'v_dc', 'T_mot_meas', 'T_esc', 'tau_applied'}, ...
    {'up', 'lo', 'tau_avail', 'derating'});
A('simulink/Math Operations/Abs', [s '/AbsW']);
K(s, 'ToRpm', '30/pi');
A('simulink/Lookup Tables/2-D Lookup Table', [s '/EnvPeak'], 'BreakpointsForDimension1', 'P.drive.motor.w_t_rpm', ...
    'BreakpointsForDimension2', 'P.drive.motor.vdc_t', 'Table', 'P.drive.motor.T_pk_2D', 'ExtrapMethod', 'Clip', ...
    'InterpMethod', 'Linear point-slope');
LUT1(s, 'EnvCont', 'P.drive.motor.w_t_rpm', 'P.drive.motor.T_cont_nom');
L(s, 'w_meas/1', 'AbsW/1'); L(s, 'AbsW/1', 'ToRpm/1'); L(s, 'ToRpm/1', 'EnvPeak/1'); L(s, 'v_dc/1', 'EnvPeak/2');
L(s, 'ToRpm/1', 'EnvCont/1');
% I2t: x' = rate from LimitLogic, (r - 1)/k_boost if r > 1 else (r - 1)/t_recover, r = (|tau|/T_cont)^2; x in [0, 1]
A('simulink/Discrete/Discrete-Time Integrator', [s '/I2t'], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1', 'LimitOutput', 'on', 'UpperSaturationLimit', '1', 'LowerSaturationLimit', '0', ...
    'InitialCondition', '0', 'SampleTime', TS);
A('simulink/Discontinuities/Relay', [s '/Exhausted'], 'OnSwitchValue', 'P.drive.esc.x_on', ...
    'OffSwitchValue', 'P.drive.esc.x_off', 'OnOutputValue', '1', 'OffOutputValue', '0', 'SampleTime', TS);
L(s, 'I2t/1', 'Exhausted/1');
% derating factors
LUT1(s, 'f_Tm', 'P.drive.esc.T_mot_derate', '[1 0]');
LUT1(s, 'f_Te', 'P.drive.esc.T_esc_derate', '[1 0]');
LUT1(s, 'f_uv', 'P.drive.esc.V_uv_derate', '[0 1]');
LUT1(s, 'f_ov', 'P.drive.esc.V_ov_derate', '[1 0]');
L(s, 'T_mot_meas/1', 'f_Tm/1'); L(s, 'T_esc/1', 'f_Te/1'); L(s, 'v_dc/1', 'f_uv/1'); L(s, 'v_dc/1', 'f_ov/1');
% the constants of the former Saturation, Gain, Switch and Compare To Constant blocks, on an input (see MFB)
C(s, 'TlPar', ['[P.drive.esc.tau_cont_min, 1/P.drive.esc.k_boost, 1/P.drive.esc.t_recover, P.drive.esc.w_min_dc, ' ...
    'P.drive.esc.I_dc_max_mot, P.drive.esc.I_dc_max_regen, P.drive.esc.w_quad, 1 - P.drive.esc.derate_eps]'], 'double');
MFB(s, 'LimitLogic', sprintf(['function [rate, up, lo, tau_avail, derating] = esc_torque_limit_fcn(tau_lim_mot, ' ...
    'tau_lim_reg, w_meas, v_dc, tau_applied, T_pk, T_cont, exhausted, f_Tm, f_Te, f_uv, f_ov, tlv)\n%%#codegen\n' ...
    '[rate, up, lo, tau_avail, derating] = esc_torque_limit(tau_lim_mot, tau_lim_reg, w_meas, v_dc, tau_applied, ' ...
    'T_pk, T_cont, exhausted, f_Tm, f_Te, f_uv, f_ov, tlv);\n']), 'esc_torque_limit', D, ...
    'Quadrant torque limits, I2t rate and DERATING flag of one ESC: subsystems/drive/esc_torque_limit.m (D70)');
tin = {'cmd_in.tau_lim_mot', 'cmd_in.tau_lim_reg', 'w_meas/1', 'v_dc/1', 'tau_applied/1', 'EnvPeak/1', 'EnvCont/1', ...
    'Exhausted/1', 'f_Tm/1', 'f_Te/1', 'f_uv/1', 'f_ov/1', 'TlPar/1'};
for k = 1:numel(tin), L(s, tin{k}, ['LimitLogic/' num2str(k)]); end
tout = {'I2t/1', 'rate'; 'up/1', 'up'; 'lo/1', 'lo'; 'tau_avail/1', 'tau_avail'; 'derating/1', 'derating'};
for k = 1:size(tout, 1), L(s, ['LimitLogic/' num2str(k)], tout{k, 1}, tout{k, 2}); end
PORTS(s, {'cmd_in', 'w_meas', 'v_dc', 'T_mot_meas', 'T_esc', 'tau_applied'});

% ---------------- Protections (Protection Relay latches, reset by the fault_reset edge) ----------------
s = mksub(n, 'Protections', {'i_dc', 'v_dc', 'w_meas', 'T_mot_meas', 'T_esc', 'duty_applied', 'tau_applied', 'pwr_en'}, ...
    {'fault_cond', 'stall_warn'});
PROT(s, 'OC', 'Overcurrent', 'P.drive.esc.n_oc');     C(s, 'RefOC', 'P.drive.esc.I_oc_fault');
PROT(s, 'OV', 'Overvoltage', 'P.drive.esc.n_ov');     C(s, 'RefOV', 'P.drive.esc.V_ov_fault');
PROT(s, 'UV', 'Undervoltage', 'P.drive.esc.n_uv');    C(s, 'RefUV', 'P.drive.esc.V_uv_fault');
PROT(s, 'OS', 'Overspeed', 'P.drive.esc.n_os');       C(s, 'RefOS', 'P.drive.esc.w_overspeed');
PROT(s, 'MOT_OT', 'Overvoltage', 'P.drive.esc.n_ot'); C(s, 'RefMOT', 'P.drive.esc.T_mot_fault');
PROT(s, 'ESC_OT', 'Overvoltage', 'P.drive.esc.n_ot'); C(s, 'RefESC', 'P.drive.esc.T_esc_fault');
PROT(s, 'ENC', 'Overvoltage', 'P.drive.esc.n_enc');   C(s, 'RefHalf1', '0.5');
PROT(s, 'BRK', 'Overvoltage', 'P.drive.esc.n_brake'); C(s, 'RefHalf2', '0.5');
PROT(s, 'STALL', 'Overvoltage', 'P.drive.esc.n_stall'); C(s, 'RefHalf3', '0.5');
PROT(s, 'TRIP', 'Overvoltage', '1');                  C(s, 'RefHalf4', '0.5');
% undervoltage is a latched fault only when the DC link collapses while the power stage is enabled (DS402: a dead
% DC link in INIT / NOT_READY / READY / STO / FAULT / PARKED is "not ready to switch on", no fault). Revision C:
% revision B armed UV for good after the first v_dc > V_on, so the planned HV removal after a hardware E-stop
% (STO first, then K1/K2 open) latched UV in both ESCs -> MCU DRV_L/R_FAULT (class A) -> no restart (S8a).
% armed = pwr_en(k-1) OR (armed(k-1) AND v_dc < V_on): holds through the n_uv debounce after ESC_SM has already
% dropped to NOT_READY at V_off (T11); pwr_en = ESC_SM torque_en through a 1 ms Unit Delay (loop breaker).
REL(s, 'BelowVon', '<', 'P.drive.esc.V_on');
LOG(s, 'UvHold', 'AND'); LOG(s, 'Armed', 'OR'); UD(s, 'ArmedLast', 'false', TS);
C(s, 'Vnom', 'P.drive.motor.V_nom'); SW(s, 'UvFb', 'u2 ~= 0');
L(s, 'v_dc/1', 'BelowVon/1'); L(s, 'ArmedLast/1', 'UvHold/1'); L(s, 'BelowVon/1', 'UvHold/2');
L(s, 'pwr_en/1', 'Armed/1'); L(s, 'UvHold/1', 'Armed/2'); L(s, 'Armed/1', 'ArmedLast/1');
L(s, 'v_dc/1', 'UvFb/1'); L(s, 'Armed/1', 'UvFb/2'); L(s, 'Vnom/1', 'UvFb/3');
% boolean conditions as doubles
DTC(s, 'EncD', 'double'); L(s, 'cmd_in.enc_fail', 'EncD/1');
REL(s, 'BrkCmd', '>', '0'); LOG(s, 'BrkStuckRel', 'AND'); DTC(s, 'BrkD', 'double');
L(s, 'duty_applied/1', 'BrkCmd/1'); L(s, 'cmd_in.brake_stuck', 'BrkStuckRel/1'); L(s, 'BrkCmd/1', 'BrkStuckRel/2');
L(s, 'BrkStuckRel/1', 'BrkD/1');
% stall: SPEED mode, |tau| > frac*T_peak, |w| < stall_w; warning after t_stall_warn, fault after t_stall_fault
A('simulink/Math Operations/Abs', [s '/AbsTau']); A('simulink/Math Operations/Abs', [s '/AbsW']);
REL(s, 'TauHigh', '>', 'P.drive.esc.stall_tau_frac*P.drive.motor.T_peak');
REL(s, 'WLow', '<', 'P.drive.esc.stall_w');
REL(s, 'SpeedMode', '==', '1');
LOG(s, 'StallCond', 'AND', 3); DTC(s, 'StallD', 'double'); LOG(s, 'NotStall', 'NOT', 1);
L(s, 'tau_applied/1', 'AbsTau/1'); L(s, 'w_meas/1', 'AbsW/1'); L(s, 'AbsTau/1', 'TauHigh/1'); L(s, 'AbsW/1', 'WLow/1');
L(s, 'cmd_in.mode', 'SpeedMode/1');
L(s, 'TauHigh/1', 'StallCond/1'); L(s, 'WLow/1', 'StallCond/2'); L(s, 'SpeedMode/1', 'StallCond/3');
L(s, 'StallCond/1', 'StallD/1'); L(s, 'StallCond/1', 'NotStall/1');
C(s, 'One', '1');
A('simulink/Discrete/Discrete-Time Integrator', [s '/StallTimer'], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'ExternalReset', 'level', 'InitialCondition', '0', 'SampleTime', TS);
REL(s, 'StallWarn', '>=', 'P.drive.esc.t_stall_warn');
L(s, 'One/1', 'StallTimer/1'); L(s, 'NotStall/1', 'StallTimer/2'); L(s, 'StallTimer/1', 'StallWarn/1');
L(s, 'StallWarn/1', 'stall_warn/1', 'stall_warn');
% drive trip by fault injection: tau_scale ~ 0 with enable -> OVERCURRENT
REL(s, 'ScaleZero', '<', 'P.drive.esc.tau_trip'); LOG(s, 'TripCond', 'AND'); DTC(s, 'TripD', 'double');
L(s, 'cmd_in.tau_scale', 'ScaleZero/1'); L(s, 'ScaleZero/1', 'TripCond/1'); L(s, 'cmd_in.enable', 'TripCond/2');
L(s, 'TripCond/1', 'TripD/1');
% relay wiring: Ref_limit, Feedback, Reset (each relay picks the reset edge right in front of it)
rl = {'OC', 'RefOC', 'i_dc/1'; 'OV', 'RefOV', 'v_dc/1'; 'UV', 'RefUV', 'UvFb/1'; 'OS', 'RefOS', 'w_meas/1'; ...
      'MOT_OT', 'RefMOT', 'T_mot_meas/1'; 'ESC_OT', 'RefESC', 'T_esc/1'; 'ENC', 'RefHalf1', 'EncD/1'; ...
      'BRK', 'RefHalf2', 'BrkD/1'; 'STALL', 'RefHalf3', 'StallD/1'; 'TRIP', 'RefHalf4', 'TripD/1'};
for k = 1:size(rl, 1)
    L(s, [rl{k, 2} '/1'], [rl{k, 1} '/1']); L(s, rl{k, 3}, [rl{k, 1} '/2']); L(s, 'cmd_in.reset_edge', [rl{k, 1} '/3']);
end
% fault word bits: 0 OC|TRIP, 1 OV, 2 UV, 3 MOT_OT, 4 ESC_OT, 5 OS, 6 (CAN, in ESC_SM), 7 ENC, 8 BRAKE, 9 STALL, 10 (STO, in ESC_SM)
C(s, 'Zero6', '0'); C(s, 'Zero10', '0');
MUX(s, 'BitsMux', 11);
bsrc = {'OC/1', 'OV/1', 'UV/1', 'MOT_OT/1', 'ESC_OT/1', 'OS/1', 'Zero6/1', 'ENC/1', 'BRK/1', 'STALL/1', 'Zero10/1'};
for k = 1:11, L(s, bsrc{k}, ['BitsMux/' num2str(k)]); end
DTC(s, 'BitsU16', 'uint16');
A('simulink/Logic and Bit Operations/Shift Arithmetic', [s '/BitShift'], 'BitShiftNumber', '0:10', 'BitShiftDirection', 'Left');
A('simulink/Logic and Bit Operations/Bitwise Operator', [s '/Word'], 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '1');
L(s, 'BitsMux/1', 'BitsU16/1'); L(s, 'BitsU16/1', 'BitShift/1'); L(s, 'BitShift/1', 'Word/1');
% drive trip (fault injection tau_scale ~ 0 with enable) reported as OVERCURRENT: Bit Set bit 0 when TRIP latched
A('simulink/Logic and Bit Operations/Bit Set', [s '/SetOC'], 'iBit', '0');
REL(s, 'TripOn', '>', '0.5'); SW(s, 'TripWord', 'u2 ~= 0');
L(s, 'Word/1', 'SetOC/1'); L(s, 'TRIP/1', 'TripOn/1');
L(s, 'SetOC/1', 'TripWord/1'); L(s, 'TripOn/1', 'TripWord/2'); L(s, 'Word/1', 'TripWord/3');
L(s, 'TripWord/1', 'fault_cond/1', 'fault_cond');
PORTS(s, {'cmd_in', 'i_dc', 'v_dc', 'w_meas', 'T_mot_meas', 'T_esc', 'duty_applied', 'tau_applied', 'pwr_en'});

% ---------------- ESC_SM: MATLAB Function (D1) calling esc_sm.m -> bus sm ----------------
% ESC_SM sends 3 or more signals to Speed_Ctrl and Feedback, so its 9 outputs travel as the virtual bus sm (spec 6.3)
s = mksub(n, 'ESC_SM', {'sto_ok', 'v_dc', 'fault_cond', 'w_meas'}, {});
mf = [s '/esc_sm_fcn'];
add_block('simulink/User-Defined Functions/MATLAB Function', mf);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', mf);
ch.Script = sprintf(['function [state, torque_en, brake_rel, duty, hold0, qs, use_speed, fault_word, brake_released] = ' ...
    'esc_sm_fcn(enable, mode, w_ref, tau_ff, park_req, reset_edge, sto_ok, v_dc, can_to, fault_cond, w_meas, smv)\n' ...
    '%%#codegen\n' ...
    '[state, torque_en, brake_rel, duty, hold0, qs, use_speed, fault_word, brake_released] = ' ...
    'esc_sm(enable, mode, w_ref, tau_ff, park_req, reset_edge, sto_ok, v_dc, can_to, fault_cond, w_meas, smv);\n']);
try, ch.SampleTime = TS; catch, end %#ok<CTCH> inherited from the 1 ms inputs otherwise
% State-machine constants P.drive.sm_vec enter on an input (Constant block), not as mask-resolved parameter
% data: drive_esc sits three Subsystem Reference levels below Shaddad_Plant (drive_system -> drive_side ->
% drive_esc), where parameter data intermittently trips the R2026b JIT relink assertion 'o != nullptr'.
C(s, 'SmPar', 'P.drive.sm_vec', 'double');
inn = {'cmd_in.enable', 'cmd_in.mode', 'cmd_in.w_ref', 'cmd_in.tau_ff', 'cmd_in.park_req', 'cmd_in.reset_edge', ...
    'sto_ok/1', 'v_dc/1', 'cmd_in.can_to', 'fault_cond/1', 'w_meas/1', 'SmPar/1'};
for k = 1:numel(inn), L(s, inn{k}, ['esc_sm_fcn/' num2str(k)]); end
outn = {'state', 'torque_en', 'brake_rel', 'duty', 'hold0', 'qs', 'use_speed', 'fault_word', 'brake_released'};
for k = 1:numel(outn), OB(s, 'sm', outn{k}, ['esc_sm_fcn/' num2str(k)]); end
PORTS(s, {'cmd_in', 'sto_ok', 'v_dc', 'fault_cond', 'w_meas'});

% ---------------- Speed_Ctrl: setpoint ramp, speed PI, torque clamp ----------------
s = mksub(n, 'Speed_Ctrl', {'w_meas', 'up', 'lo'}, {'Tr', 'w_cmd', 'lim_active'});
C(s, 'Zero', '0');
SW(s, 'Target', 'u2 ~= 0');                       % hold0 -> 0 else w_ref
LOG(s, 'Active', 'AND');                          % torque_en && use_speed
SW(s, 'RampIn', 'u2 ~= 0');                       % active -> target else w_meas (ramp tracks the speed)
C(s, 'Amax', 'P.drive.esc.a_max'); C(s, 'Aqs', 'P.drive.esc.a_qs'); C(s, 'Atrack', 'P.drive.esc.a_track');
SW(s, 'ASel', 'u2 ~= 0'); SW(s, 'AEff', 'u2 ~= 0'); K(s, 'NegA', '-1');
A('simulink/Discontinuities/Rate Limiter Dynamic', [s '/SetpointRamp']);
L(s, 'Zero/1', 'Target/1'); L(s, 'sm.hold0', 'Target/2'); L(s, 'cmd_in.w_ref', 'Target/3');
L(s, 'sm.torque_en', 'Active/1'); L(s, 'sm.use_speed', 'Active/2');
L(s, 'Target/1', 'RampIn/1'); L(s, 'Active/1', 'RampIn/2'); L(s, 'w_meas/1', 'RampIn/3');
L(s, 'Aqs/1', 'ASel/1'); L(s, 'sm.qs', 'ASel/2'); L(s, 'Amax/1', 'ASel/3');
L(s, 'ASel/1', 'AEff/1'); L(s, 'Active/1', 'AEff/2'); L(s, 'Atrack/1', 'AEff/3'); L(s, 'AEff/1', 'NegA/1');
L(s, 'AEff/1', 'SetpointRamp/1'); L(s, 'RampIn/1', 'SetpointRamp/2'); L(s, 'NegA/1', 'SetpointRamp/3');
% feed-forward only in SPEED operation (not in hold-zero states), torque setpoint in TORQUE mode
LOG(s, 'NotHold', 'NOT', 1); LOG(s, 'FfOn', 'AND');
SW(s, 'FfSel', 'u2 ~= 0');
L(s, 'sm.hold0', 'NotHold/1'); L(s, 'NotHold/1', 'FfOn/1'); L(s, 'sm.use_speed', 'FfOn/2');
% ESC-internal inertia feed-forward of its own ramp in the hold-zero states (ENABLING / DISABLING /
% QUICK_STOP / PARKED, no MCU feed-forward there): tau = J_ff * d(w_ramp)/dt
UD(s, 'RampLast', '0', TS); SUM(s, 'dRamp', '+-'); K(s, 'Jff', 'P.drive.esc.J_ff/P.drive.Ts_esc');
LOG(s, 'IntFfOn', 'AND'); SW(s, 'IntFf', 'u2 ~= 0');
L(s, 'SetpointRamp/1', 'RampLast/1', 'w_ramp'); L(s, 'SetpointRamp/1', 'dRamp/1'); L(s, 'RampLast/1', 'dRamp/2');
L(s, 'dRamp/1', 'Jff/1'); L(s, 'sm.hold0', 'IntFfOn/1'); L(s, 'Active/1', 'IntFfOn/2');
L(s, 'Jff/1', 'IntFf/1'); L(s, 'IntFfOn/1', 'IntFf/2'); L(s, 'Zero/1', 'IntFf/3');
L(s, 'cmd_in.tau_ff', 'FfSel/1'); L(s, 'FfOn/1', 'FfSel/2'); L(s, 'IntFf/1', 'FfSel/3');
SUM(s, 'Err', '+-');
SUM(s, 'PiUp', '+-'); SUM(s, 'PiLo', '+-');
LOG(s, 'PiReset', 'NOT', 1);
A('simulink/Discrete/Discrete PID Controller', [s '/SpeedPI'], 'Controller', 'PI', 'Form', 'Parallel', ...
    'P', 'P.drive.esc.Kp_w', 'I', 'P.drive.esc.Ki_w', 'SampleTime', TS, 'IntegratorMethod', 'Backward Euler', ...
    'LimitOutput', 'on', 'SatLimitsSource', 'external', 'AntiWindupMode', 'clamping', 'ExternalReset', 'level', ...
    'InitialConditionForIntegrator', '0');
L(s, 'SetpointRamp/1', 'Err/1'); L(s, 'w_meas/1', 'Err/2');
L(s, 'up/1', 'PiUp/1'); L(s, 'FfSel/1', 'PiUp/2'); L(s, 'lo/1', 'PiLo/1'); L(s, 'FfSel/1', 'PiLo/2');
L(s, 'Active/1', 'PiReset/1');
L(s, 'Err/1', 'SpeedPI/1'); L(s, 'PiReset/1', 'SpeedPI/2'); L(s, 'PiUp/1', 'SpeedPI/3'); L(s, 'PiLo/1', 'SpeedPI/4');
SUM(s, 'PiPlusFf', '++');
SW(s, 'ModeSel', 'u2 ~= 0');                      % use_speed -> PI + ff else tau_ff (TORQUE mode)
A('simulink/Discontinuities/Saturation Dynamic', [s '/TorqueClamp']);
L(s, 'SpeedPI/1', 'PiPlusFf/1'); L(s, 'FfSel/1', 'PiPlusFf/2');
L(s, 'PiPlusFf/1', 'ModeSel/1'); L(s, 'sm.use_speed', 'ModeSel/2'); L(s, 'cmd_in.tau_ff', 'ModeSel/3');
L(s, 'up/1', 'TorqueClamp/1'); L(s, 'ModeSel/1', 'TorqueClamp/2'); L(s, 'lo/1', 'TorqueClamp/3');
PROD(s, 'Scaled', '**');
SW(s, 'TorqueGate', 'u2 ~= 0');
L(s, 'TorqueClamp/1', 'Scaled/1'); L(s, 'cmd_in.tau_scale', 'Scaled/2');
L(s, 'Scaled/1', 'TorqueGate/1'); L(s, 'sm.torque_en', 'TorqueGate/2'); L(s, 'Zero/1', 'TorqueGate/3');
L(s, 'TorqueGate/1', 'Tr/1', 'Tr');
SW(s, 'WCmd', 'u2 ~= 0');
L(s, 'SetpointRamp/1', 'WCmd/1'); L(s, 'sm.torque_en', 'WCmd/2'); L(s, 'Zero/1', 'WCmd/3'); L(s, 'WCmd/1', 'w_cmd/1', 'w_cmd');
% TORQUE_LIMIT_ACTIVE: commanded torque clipped by the limiter while torque is on
SUM(s, 'Clip', '+-'); A('simulink/Math Operations/Abs', [s '/AbsClip']);
REL(s, 'Clipped', '>', 'P.drive.esc.lim_eps'); LOG(s, 'LimOn', 'AND');
L(s, 'ModeSel/1', 'Clip/1'); L(s, 'TorqueClamp/1', 'Clip/2'); L(s, 'Clip/1', 'AbsClip/1'); L(s, 'AbsClip/1', 'Clipped/1');
L(s, 'Clipped/1', 'LimOn/1'); L(s, 'sm.torque_en', 'LimOn/2'); L(s, 'LimOn/1', 'lim_active/1', 'lim_active');
PORTS(s, {'cmd_in', 'sm', 'w_meas', 'up', 'lo'});

% ---------------- Output stage: 1 ms command delay (D36), estimates ----------------
s = mksub(n, 'Output_Stage', {'Tr', 'w_cmd'}, {'plant_cmd', 'tau_applied', 'duty_applied', 'tau_est', 'i_q', 'P_esc'});
MUX(s, 'CmdMux', 4); DTC(s, 'EnD', 'double');
A('simulink/Discrete/Unit Delay', [s '/CmdDelay'], 'InitialCondition', 'zeros(4,1)', 'SampleTime', TS);
L(s, 'Tr/1', 'CmdMux/1'); L(s, 'sm.duty', 'CmdMux/2'); L(s, 'w_cmd/1', 'CmdMux/3');
L(s, 'sm.torque_en', 'EnD/1'); L(s, 'EnD/1', 'CmdMux/4');
L(s, 'CmdMux/1', 'CmdDelay/1', 'cmd'); L(s, 'CmdDelay/1', 'plant_cmd/1', 'plant_cmd');
DEMUX(s, 'CmdSplit', 4);
L(s, 'CmdDelay/1', 'CmdSplit/1'); L(s, 'CmdSplit/1', 'tau_applied/1', 'tau_applied');
L(s, 'CmdSplit/2', 'duty_applied/1', 'duty_applied');
A('simulink/Sinks/Terminator', [s '/TermW']); L(s, 'CmdSplit/3', 'TermW/1', 'w_cmd_applied');
A('simulink/Sinks/Terminator', [s '/TermEn']); L(s, 'CmdSplit/4', 'TermEn/1', 'en_applied');
A('simulink/Discrete/Discrete Transfer Fcn', [s '/TorqueLag'], 'Numerator', '[1-P.drive.esc.a_Tc]', ...
    'Denominator', '[1 -P.drive.esc.a_Tc]', 'SampleTime', TS);
K(s, 'Iq', '1/P.drive.motor.Kt');
A('simulink/Math Operations/Abs', [s '/AbsIq']);
K(s, 'Irms', '1/sqrt(2)');
A('simulink/Math Operations/Math Function', [s '/Irms2'], 'Operator', 'square');
K(s, 'Pcond', '3*P.drive.esc.R_on'); K(s, 'Psw', 'P.drive.esc.k_sw*P.drive.motor.V_nom');
SUM(s, 'Pesc', '++');
L(s, 'CmdSplit/1', 'TorqueLag/1'); L(s, 'TorqueLag/1', 'tau_est/1', 'tau_est'); L(s, 'TorqueLag/1', 'Iq/1');
L(s, 'Iq/1', 'i_q/1', 'i_q');
L(s, 'Iq/1', 'AbsIq/1'); L(s, 'AbsIq/1', 'Irms/1'); L(s, 'Irms/1', 'Irms2/1'); L(s, 'Irms2/1', 'Pcond/1');
L(s, 'Irms/1', 'Psw/1'); L(s, 'Pcond/1', 'Pesc/1'); L(s, 'Psw/1', 'Pesc/2'); L(s, 'Pesc/1', 'P_esc/1', 'P_esc');
PORTS(s, {'sm', 'Tr', 'w_cmd'});

% ---------------- Feedback: MATLAB Function FbPack (D70) calling esc_feedback.m -> Bus_DriveSideFb ----------------
% The status word (11 bits from the state, the mode and the status inputs), the alive counter and the feedback bus
% are one function; the alive tick Tick (Counter Limited, a state) stays a block in front of it (D70). sto_ok is not
% used (STO_ACTIVE comes from the state and fault-word bit 10): it enters FbPack, as it entered a Terminator before.
s = mksub(n, 'Feedback', {'derating', 'lim_active', 'sto_ok', 'stall_warn', 'w_meas', 'enc_cnt', 'tau_est', 'tau_avail', ...
    'i_q', 'i_dc', 'v_dc', 'T_mot_meas', 'T_esc'}, {'fb'});
% ESC alive counter: +1 per 10 ms (one FB frame)
A('simulink/Sources/Counter Limited', [s '/Tick'], 'uplimit', '256*P.drive.esc.n_alive - 1', 'tsamp', TS);
C(s, 'FbPar', '1/P.drive.esc.n_alive', 'double');   % the gain of the former PerFrame, on an input (see MFB)
MFB(s, 'FbPack', sprintf(['function fb = esc_feedback_fcn(mode, state, brake_released, fault_word, derating, ' ...
    'lim_active, sto_ok, stall_warn, w_meas, enc_cnt, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc, tick, ' ...
    'k_alive)\n%%#codegen\nfb = esc_feedback(mode, state, brake_released, fault_word, derating, lim_active, sto_ok, ' ...
    'stall_warn, w_meas, enc_cnt, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc, tick, k_alive);\n']), ...
    'esc_feedback', D, 'Status word, alive counter and Bus_DriveSideFb of one ESC: subsystems/drive/esc_feedback.m (D70)', ...
    {'fb', 'Bus: Bus_DriveSideFb'});
fin = {'cmd_in.mode', 'sm.state', 'sm.brake_released', 'sm.fault_word', 'derating/1', 'lim_active/1', 'sto_ok/1', ...
    'stall_warn/1', 'w_meas/1', 'enc_cnt/1', 'tau_est/1', 'tau_avail/1', 'i_q/1', 'i_dc/1', 'v_dc/1', 'T_mot_meas/1', ...
    'T_esc/1', 'Tick/1', 'FbPar/1'};
for k = 1:numel(fin), L(s, fin{k}, ['FbPack/' num2str(k)]); end
L(s, 'FbPack/1', 'fb/1', 'fb');
PORTS(s, {'cmd_in', 'sm', 'derating', 'lim_active', 'sto_ok', 'stall_warn', 'w_meas', 'enc_cnt', 'tau_est', 'tau_avail', ...
    'i_q', 'i_dc', 'v_dc', 'T_mot_meas', 'T_esc'});

% ---------------- top-level wiring of drive_esc: the buses cmd_in (Cmd_In) and sm (ESC_SM), picked inside ----------
L(n, 'cmd/1', 'Cmd_In/1'); L(n, 'fault_inj/1', 'Cmd_In/2');
for b = {'Speed_Meas/2', 'Temp_Sens/2', 'Torque_Limiter/1', 'Protections/1', 'ESC_SM/1', 'Speed_Ctrl/1', 'Feedback/1'}
    L(n, 'Cmd_In/1', b{1}, 'cmd_in');
end
L(n, 'theta_m/1', 'Speed_Meas/1'); L(n, 'T_w/1', 'Temp_Sens/1');
% Torque_Limiter in: cmd_in w_meas v_dc T_mot_meas T_esc tau_applied
L(n, 'Speed_Meas/1', 'Torque_Limiter/2', 'w_meas'); L(n, 'v_dc/1', 'Torque_Limiter/3');
L(n, 'Temp_Sens/1', 'Torque_Limiter/4', 'T_mot_meas'); L(n, 'T_esc/1', 'Torque_Limiter/5');
L(n, 'Output_Stage/2', 'Torque_Limiter/6', 'tau_applied');
% Protections in: cmd_in i_dc v_dc w_meas T_mot_meas T_esc duty_applied tau_applied pwr_en
L(n, 'i_dc/1', 'Protections/2'); L(n, 'v_dc/1', 'Protections/3'); L(n, 'Speed_Meas/1', 'Protections/4');
L(n, 'Temp_Sens/1', 'Protections/5'); L(n, 'T_esc/1', 'Protections/6'); L(n, 'Output_Stage/3', 'Protections/7', 'duty_applied');
L(n, 'Output_Stage/2', 'Protections/8');
% power stage enabled (k-1): UV arming, breaks the SM loop; torque_en picked from sm right in front of the delay
UD(n, 'PwrEnLast', 'false', TS);
SEL(n, 'SelPwrEn', 'ESC_SM/1', {'torque_en'}, {'PwrEnLast/1'}); L(n, 'PwrEnLast/1', 'Protections/9', 'pwr_en');
% ESC_SM in: cmd_in sto_ok v_dc fault_cond w_meas -> out sm (state torque_en brake_rel duty hold0 qs use_speed
% fault_word brake_released; brake_rel has no consumer)
L(n, 'sto_ok/1', 'ESC_SM/2'); L(n, 'v_dc/1', 'ESC_SM/3'); L(n, 'Protections/1', 'ESC_SM/4', 'fault_cond');
L(n, 'Speed_Meas/1', 'ESC_SM/5');
for b = {'Speed_Ctrl/2', 'Output_Stage/1', 'Feedback/2'}
    L(n, 'ESC_SM/1', b{1}, 'sm');
end
% Speed_Ctrl in: cmd_in sm w_meas up lo
L(n, 'Speed_Meas/1', 'Speed_Ctrl/3'); L(n, 'Torque_Limiter/1', 'Speed_Ctrl/4', 'up'); L(n, 'Torque_Limiter/2', 'Speed_Ctrl/5', 'lo');
% Output stage in: sm Tr w_cmd
L(n, 'Speed_Ctrl/1', 'Output_Stage/2', 'Tr'); L(n, 'Speed_Ctrl/2', 'Output_Stage/3', 'w_cmd');
L(n, 'Output_Stage/1', 'plant_cmd/1', 'plant_cmd'); L(n, 'Output_Stage/6', 'P_esc/1', 'P_esc');
% Feedback in: cmd_in sm derating lim_active sto_ok stall_warn w_meas enc_cnt tau_est tau_avail i_q i_dc v_dc
%              T_mot_meas T_esc
fbsrc = {'Torque_Limiter/4', 'derating'; 'Speed_Ctrl/3', 'lim_active'; 'sto_ok/1', ''; 'Protections/2', 'stall_warn'; ...
         'Speed_Meas/1', ''; 'Speed_Meas/2', 'enc_cnt'; 'Output_Stage/4', 'tau_est'; 'Torque_Limiter/3', 'tau_avail'; ...
         'Output_Stage/5', 'i_q'; 'i_dc/1', ''; 'v_dc/1', ''; 'Temp_Sens/1', ''; 'T_esc/1', ''};
for k = 1:size(fbsrc, 1), L(n, fbsrc{k, 1}, ['Feedback/' num2str(k + 2)], fbsrc{k, 2}); end
L(n, 'Feedback/1', 'fb/1', 'fb');
finish(n, f, ['Traction ESC firmware model (drive spec 2.9 / 3.5, build plan T06.2), 1 ms. CAN watchdog (Software Watchdog ' ...
    'Timer, 100 ms), DS402 state machine ESC_SM (MATLAB Function -> esc_sm.m, enum EscState, D1), setpoint ramp (Rate Limiter ' ...
    'Dynamic, a_max 420 / a_qs 375 rad/s^2, D33), speed PI (Kp_w 0.13, Ki_w 0.39, clamping, D50), quadrant torque limiter ' ...
    '(2-D envelope, I2t, thermal / voltage / DC-current derating; MATLAB Function LimitLogic -> esc_torque_limit.m, D70), ' ...
    'Protection Relays, status / fault words (status word and feedback bus: MATLAB Function FbPack -> esc_feedback.m, ' ...
    'D70), encoder model. ' ...
    'Every ESC -> plant command (Tr, brake duty, speed command) passes a 1 ms Unit Delay (P.drive.esc.cmd_delay, D36). ' ...
    'plant_cmd = [Tr N m; brake duty; w_cmd rad/s; power-stage enable]. Cmd_In and ESC_SM send their outputs as the ' ...
    'virtual buses cmd_in and sm, picked in front of each consumer. Owner drive (T06).']);
end

% =============================================================================================
% Physical helpers
% =============================================================================================
function h = PSIN(blk)
% the physical-signal input port handle of a Simscape block (first port that accepts a PS line)
ph = H(blk); hs = [ph.LConn ph.RConn];
sys = get_param(blk, 'Parent');
tmp = add_block('nesl_utility/Simulink-PS Converter', [sys '/tmp_T06_probe_sps'], 'MakeNameUnique', 'on');
tp = H(tmp);
h = [];
for k = 1:numel(hs)
    try
        ln = add_line(sys, tp.RConn(1), hs(k));
        delete_line(ln); h = hs(k); break
    catch %#ok<CTCH>
    end
end
delete_block(tmp);
if isempty(h), error('build_drive:psin', 'No physical-signal input on %s', blk); end
end

function b = MREF(sys, name)
b = A(sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference'), [sys '/' name]);
end

function b = EMOT(sys, name)   % Ideal Rotational Motion Sensor with W and A outputs
b = A(sprintf('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational\nMotion Sensor'), [sys '/' name], ...
    'velocity_port', 'true', 'position_port', 'true', 'wrap_angle', 'false');
end

function set_init(blk, var, val, unit)
% Simscape variable initial target (high priority)
set_param(blk, [var '_specify'], 'on', [var '_priority'], 'High', var, val);
if nargin > 3, set_param(blk, [var '_unit'], unit); end
end

% =============================================================================================
% Gearbox (spec 2.8 / 3.2): B = motor side, F = sprocket side
% =============================================================================================
function local_gearbox(D)
n = 'drive_gearbox'; f = fullfile(D, [n '.slx']); fresh(n, f);
cport(n, 'B', 1, 'Left', 'M');
cport(n, 'F', 2, 'Right', 'M');
outport(n, 'gb', 1, 'double');
gb = A('fl_lib/Mechanical/Mechanisms/Gear Box', [n '/GearBox'], 'ratio', 'P.drive.gear.N');
dr = A('fl_lib/Mechanical/Rotational Elements/Rotational Friction', [n '/NoLoadDrag'], 'brkwy_trq', 'P.drive.gear.T_nl_brk', ...
    'Col_trq', 'P.drive.gear.T_nl_C', 'brkwy_vel', 'P.drive.gear.w_nl_brk', 'visc_coef', 'P.drive.gear.c_nl');
jo = A('fl_lib/Mechanical/Rotational Elements/Inertia', [n '/J_out'], 'inertia', 'P.drive.gear.J_out', ...
    'num_ports', 'foundation.enum.numPorts2.one');
ts = A('fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor', [n '/TorqueOut']);
ms = EMOT(n, 'MotionOut');
src = A('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', [n '/MeshLoss']);
cs = MREF(n, 'Case');
pa = A('fl_lib/Physical Signals/Nonlinear Operators/PS Abs', [n '/AbsT']);
pk = A('fl_lib/Physical Signals/Functions/PS Gain', [n '/k_loss'], 'gain', 'P.drive.gear.k_loss');
pw = A('fl_lib/Physical Signals/Functions/PS Gain', [n '/InvWth'], 'gain', '1/P.drive.gear.w_th', 'gain_unit', 's/rad');
pt = A('fl_lib/Physical Signals/Functions/PS Math Function', [n '/Tanh'], 'fcn_choice', 'foundation.enum.math_function.tanh');
pp = A('fl_lib/Physical Signals/Functions/PS Product', [n '/LossProd']);
pn = A('fl_lib/Physical Signals/Functions/PS Gain', [n '/Oppose'], 'gain', '-1');
hB = RC(n, 'B'); hF = RC(n, 'F');
% B node: gear input, drag (C on case)
PL(n, hB, H(gb).LConn(1)); PL(n, hB, H(dr).LConn(1)); PL(n, H(dr).RConn(1), H(cs).LConn(1));
% output node O: gear output, J_out, loss source R, torque sensor R, motion sensor R
hO = H(gb).RConn(1);
PL(n, hO, H(jo).LConn(1)); PL(n, hO, H(ts).LConn(1)); PL(n, hO, H(src).LConn(1)); PL(n, hO, H(ms).LConn(1));
PL(n, H(ts).RConn(1), hF);
PL(n, H(src).RConn(2), H(cs).LConn(1)); PL(n, H(ms).RConn(1), H(cs).LConn(1));
% loss law T_loss = -k_loss*|T_out|*tanh(w_out/w_th)
PL(n, H(ts).RConn(2), H(pa).LConn(1)); PL(n, H(pa).RConn(1), H(pk).LConn(1));
PL(n, H(ms).RConn(2), H(pw).LConn(1)); PL(n, H(pw).RConn(1), H(pt).LConn(1));
PL(n, H(pk).RConn(1), H(pp).LConn(1)); PL(n, H(pt).RConn(1), H(pp).LConn(2));
PL(n, H(pp).RConn(1), H(pn).LConn(1)); PL(n, H(pn).RConn(1), H(src).RConn(1));
% truth: [theta_spr; w_spr; tau_spr]
c1 = PS2S(n, 'theta_spr', 'rad'); c2 = PS2S(n, 'w_spr', 'rad/s'); c3 = PS2S(n, 'tau_spr', 'N*m');
PL(n, H(ms).RConn(3), H(c1).LConn(1)); PL(n, H(ms).RConn(2), H(c2).LConn(1)); PL(n, H(ts).RConn(2), H(c3).LConn(1));
MUX(n, 'GbMux', 3);
L(n, 'theta_spr/1', 'GbMux/1'); L(n, 'w_spr/1', 'GbMux/2'); L(n, 'tau_spr/1', 'GbMux/3'); L(n, 'GbMux/1', 'gb/1', 'gb');
finish(n, f, ['Traction gearbox (spec 2.8): Gear Box N = 24 (B motor side, F sprocket side), load-dependent mesh loss ' ...
    'T_loss = -k_loss |T_out| tanh(w_out / w_th) by an Ideal Torque Source (eta 0.94 both directions), no-load drag ' ...
    '(Rotational Friction) at the input, output inertia J_out. Output gb = [theta_spr; w_spr; tau_spr]. Owner drive (T06).']);
end

% =============================================================================================
% Brake (spec 2.7 / 3.3)
% =============================================================================================
function local_brake(D)
n = 'drive_brake'; f = fullfile(D, [n '.slx']); fresh(n, f);
TS = 'P.drive.Ts_esc';
inport(n, 'rel_cmd', 1, 'double', TS);
inport(n, 'brake_stuck', 2, 'boolean', TS);
cport(n, 'Shaft', 1, 'Left', 'M');
outport(n, 'brk', 1, 'double');
C(n, 'Zero', '0');
SW(n, 'Stuck', 'u2 ~= 0');
A('simulink/Discontinuities/Rate Limiter', [n '/Actuator'], 'RisingSlewLimit', '1/P.drive.brake.t_release', ...
    'FallingSlewLimit', '-1/P.drive.brake.t_engage', 'InitialCondition', '0');
C(n, 'One', '1'); SUM(n, 'Engaged', '+-'); K(n, 'Spring', 'P.drive.brake.F_spring');
sps = S2PS(n, 'NormalForce', 'N');
L(n, 'Zero/1', 'Stuck/1'); L(n, 'brake_stuck/1', 'Stuck/2'); L(n, 'rel_cmd/1', 'Stuck/3');
L(n, 'Stuck/1', 'Actuator/1', 'rel_eff'); L(n, 'One/1', 'Engaged/1'); L(n, 'Actuator/1', 'Engaged/2', 'f_rel');
L(n, 'Engaged/1', 'Spring/1', 'engaged'); L(n, 'Spring/1', 'NormalForce/1', 'F_normal');
% elasto-plastic friction (D64): bristle state, true sticking below mu_s r N (no sign switching at the fixed step)
fr = A('nesl_utility/Simscape Component', [n '/Friction']);
simscape.setBlockComponent(fr, 'shaddad_drive.elastoplastic_brake');
for p = {'r_eff', 'mu_s', 'mu_c', 'w_st', 'k_b', 'c_b', 'w_d', 'T_eps', 'w_eps', 'w_r', 'N_eps'}
    set_param(fr, p{1}, ['P.drive.brake.' p{1}]);
end
wd = A('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference', [n '/Housing']);
pf = H(fr);                                        % LConn [R N], RConn [C T]
PL(n, RC(n, 'Shaft'), pf.LConn(1));
PL(n, pf.RConn(1), H(wd).LConn(1));
PL(n, H(sps).RConn(1), pf.LConn(2));
c1 = PS2S(n, 'tau_brake', 'N*m');
PL(n, pf.RConn(2), H(c1).LConn(1));                % brake torque on the shaft
ZOH(n, 'RT_tau');                                  % network rate -> 1 ms (same rate as the actuator signal)
MUX(n, 'BrkMux', 2);
L(n, 'tau_brake/1', 'RT_tau/1'); L(n, 'RT_tau/1', 'BrkMux/1', 'tau_brake_1ms'); L(n, 'Actuator/1', 'BrkMux/2');
L(n, 'BrkMux/1', 'brk/1', 'brk');
finish(n, f, ['Spring-applied, coil-released parking / emergency brake (spec 2.7, D64): elasto-plastic friction ' ...
    '(Simscape component shaddad_drive.elastoplastic_brake, bristle state) with normal force N = F_spring (1 - f_rel) ' ...
    'between the motor shaft and the housing (Mechanical Rotational Reference): it sticks as a spring k_b below ' ...
    'mu_s r_eff N and slides with the Stribeck curve to mu_c r_eff N. f_rel from the ESC brake command through a ' ...
    'Rate Limiter (40 ms release, 60 ms engage); brake_stuck forces f_rel = 0. Output brk = [tau_brake; f_rel]. ' ...
    'Owner drive (T06).']);
end

% =============================================================================================
% Thermal (spec 2.6 / 3.4): node W = port H (winding), housing, ESC heat sink, ambient
% =============================================================================================
function local_thermal(D)
n = 'drive_thermal'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'P_esc', 1, 'double', 'P.drive.Ts_esc');
inport(n, 'T_amb', 2, 'double', 'P.drive.Ts_esc');
cport(n, 'H', 1, 'Left', 'T');
outport(n, 'T', 1, 'double');
TR = 'fl_lib/Thermal/Thermal Elements/Thermal Resistance';
TM = 'fl_lib/Thermal/Thermal Elements/Thermal Mass';
rwh = A(TR, [n '/R_wh'], 'resistance', 'P.drive.thermal.R_wh');
rhc = A(TR, [n '/R_hc'], 'resistance', 'P.drive.thermal.R_hc');
res = A(TR, [n '/R_esc'], 'resistance', 'P.drive.thermal.R_esc');
mh = A(TM, [n '/C_housing'], 'mass', 'P.drive.thermal.m_h', 'sp_heat', 'P.drive.thermal.cp_h');
me = A(TM, [n '/C_esc'], 'mass', 'P.drive.thermal.m_esc', 'sp_heat', 'P.drive.thermal.cp_esc');
set_init(mh, 'T', 'P.drive.thermal.T_amb0', 'K'); set_init(me, 'T', 'P.drive.thermal.T_amb0', 'K');
cv = A('fl_lib/Thermal/Thermal Elements/Convective Heat Transfer', [n '/Convection'], 'area', 'P.drive.thermal.A_h', ...
    'heat_tr_coeff', 'P.drive.thermal.h_h');
hq = A(sprintf('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow\nRate Source'), [n '/EscHeatSplit']);
tsrc = A(sprintf('fl_lib/Thermal/Thermal Sources/Controlled Temperature\nSource'), [n '/Ambient']);
tref = A('fl_lib/Thermal/Thermal Elements/Thermal Reference', [n '/ThermalRef']);
s1 = A('fl_lib/Thermal/Thermal Sensors/Temperature Sensor', [n '/Sens_W']);
s2 = A('fl_lib/Thermal/Thermal Sensors/Temperature Sensor', [n '/Sens_H']);
s3 = A('fl_lib/Thermal/Thermal Sensors/Temperature Sensor', [n '/Sens_E']);
hW = RC(n, 'H');
% W -> housing
PL(n, hW, H(rwh).LConn(1)); hH = H(rwh).RConn(1);
PL(n, hH, H(mh).LConn(1)); PL(n, hH, H(cv).LConn(1)); PL(n, hH, H(rhc).LConn(1));
% ESC heat share moved from W to E
% Controlled Heat Flow Rate Source polarity (probed in the live session): S > 0 moves heat from the RConn(2)
% port into the LConn(1) port, so W is on RConn(2) and the ESC node E on LConn(1)
PL(n, hW, H(hq).RConn(2)); hE = H(hq).LConn(1);
PL(n, hE, H(me).LConn(1)); PL(n, hE, H(res).LConn(1));
% ambient node AMB = temperature source port A, port B on the reference: T_A - T_B = S (checked in the
% live session: with A on the reference the node B sits at -S)
hA = H(tsrc).LConn(1);
PL(n, H(tsrc).RConn(2), H(tref).LConn(1));
PL(n, H(cv).RConn(1), hA); PL(n, H(rhc).RConn(1), hA); PL(n, H(res).RConn(1), hA);
% sensors (B on the reference)
PL(n, hW, H(s1).LConn(1)); PL(n, hH, H(s2).LConn(1)); PL(n, hE, H(s3).LConn(1));
for b = {s1, s2, s3}, PL(n, H(b{1}).RConn(1), H(tref).LConn(1)); end
% signals
ps1 = S2PS(n, 'Q_esc', 'W'); PL(n, H(ps1).RConn(1), H(hq).RConn(1));
A('simulink/Math Operations/Bias', [n '/Solar'], 'Bias', 'P.drive.thermal.dT_solar');
ps2 = S2PS(n, 'T_air', 'K'); PL(n, H(ps2).RConn(1), H(tsrc).RConn(1));
L(n, 'P_esc/1', 'Q_esc/1'); L(n, 'T_amb/1', 'Solar/1'); L(n, 'Solar/1', 'T_air/1', 'T_air');
o1 = PS2S(n, 'T_w', 'K'); o2 = PS2S(n, 'T_h', 'K'); o3 = PS2S(n, 'T_e', 'K');
PL(n, H(s1).RConn(2), H(o1).LConn(1)); PL(n, H(s2).RConn(2), H(o2).LConn(1)); PL(n, H(s3).RConn(2), H(o3).LConn(1));
MUX(n, 'TMux', 3);
L(n, 'T_w/1', 'TMux/1'); L(n, 'T_h/1', 'TMux/2'); L(n, 'T_e/1', 'TMux/3'); L(n, 'TMux/1', 'T/1', 'T');
finish(n, f, ['Drive thermal network (spec 2.6): winding node W = port H (thermal mass inside the plant choice), R_wh to the ' ...
    'housing (C_h), convection h A and flange R_hc to ambient, ESC heat share P_esc moved from W to the heat sink (C_esc, R_esc) ' ...
    'by a Controlled Heat Flow Rate Source, ambient = Controlled Temperature Source T_amb + dT_solar. Output T = [T_w; T_h; T_esc]. ' ...
    'Owner drive (T06).']);
end

% =============================================================================================
% Plant choices: common ports cmd [Tr; duty; w_cmd; en] -> meas [theta_m; w_m; tau_m; v_dc; i_dc]
% =============================================================================================
function local_plant_ports(n)
inport(n, 'cmd', 1, 'double', 'P.drive.Ts_esc');
outport(n, 'meas', 1, 'double');
cport(n, 'DC_p', 1, 'Left', 'E');
cport(n, 'DC_n', 2, 'Left', 'E');
cport(n, 'Shaft', 3, 'Right', 'M');
cport(n, 'H', 4, 'Right', 'T');
DEMUX(n, 'CmdSplit', 4);
L(n, 'cmd/1', 'CmdSplit/1');
end

function [hP, hN, ci, vs] = local_dc_link(n)
% DC_p -> current sensor -> node P; node N = DC_n; DC link C + ESR, quiescent R_q, bleed; voltage sensor
R = 'fl_lib/Electrical/Electrical Elements/Resistor';
ci = A('ee_lib/Sensors & Transducers/Current Sensor', [n '/I_dc']);
vs = A('ee_lib/Sensors & Transducers/Voltage Sensor', [n '/V_dc']);
cl = A('fl_lib/Electrical/Electrical Elements/Capacitor', [n '/C_link'], 'c', 'P.drive.esc.C_link');
set_init(cl, 'vc', 'P.drive.esc.vc0', 'V');
esr = A(R, [n '/ESR_link'], 'R', 'P.drive.esc.ESR_link');
rq = A(R, [n '/R_quiescent'], 'R', 'P.drive.esc.R_q');
rb = A(R, [n '/R_bleed'], 'R', 'P.drive.esc.R_bleed');
PL(n, RC(n, 'DC_p'), H(ci).LConn(1));
hP = H(ci).RConn(2); hN = RC(n, 'DC_n');
PL(n, hP, H(esr).LConn(1)); PL(n, H(esr).RConn(1), H(cl).LConn(1)); PL(n, H(cl).RConn(1), hN);
PL(n, hP, H(rq).LConn(1)); PL(n, H(rq).RConn(1), hN);
PL(n, hP, H(rb).LConn(1)); PL(n, H(rb).RConn(1), hN);
PL(n, hP, H(vs).LConn(1)); PL(n, H(vs).RConn(2), hN);
end

function local_plant_meas(n, ms, ts, vs, ci)
c1 = PS2S(n, 'theta_m', 'rad'); c2 = PS2S(n, 'w_m', 'rad/s'); c3 = PS2S(n, 'tau_m', 'N*m');
c4 = PS2S(n, 'v_dc', 'V'); c5 = PS2S(n, 'i_dc', 'A');
PL(n, H(ms).RConn(3), H(c1).LConn(1)); PL(n, H(ms).RConn(2), H(c2).LConn(1)); PL(n, H(ts).RConn(2), H(c3).LConn(1));
PL(n, H(vs).RConn(1), H(c4).LConn(1)); PL(n, H(ci).RConn(1), H(c5).LConn(1));
MUX(n, 'MeasMux', 5);
nm = {'theta_m', 'w_m', 'tau_m', 'v_dc', 'i_dc'};
for k = 1:5, L(n, [nm{k} '/1'], ['MeasMux/' num2str(k)]); end
L(n, 'MeasMux/1', 'meas/1', 'meas');
end

function local_coil(n, hP, hN)
% brake coil (fail-safe brake driver on the DC link): R = R_coil / max(duty, duty_min)
vr = A('fl_lib/Electrical/Electrical Elements/Variable Resistor', [n '/BrakeCoil'], 'Rmin', '1');
A('simulink/Discontinuities/Saturation', [n '/DutyFloor'], 'UpperLimit', '1', 'LowerLimit', 'P.drive.brake.duty_min');
C(n, 'R_coil', 'P.drive.brake.R_coil'); PROD(n, 'CoilR', '*/');
sps = S2PS(n, 'R_cmd', 'Ohm');
L(n, 'CmdSplit/2', 'DutyFloor/1', 'duty'); L(n, 'R_coil/1', 'CoilR/1'); L(n, 'DutyFloor/1', 'CoilR/2'); L(n, 'CoilR/1', 'R_cmd/1');
PL(n, H(sps).RConn(1), H(vr).LConn(1));
PL(n, hP, H(vr).LConn(2)); PL(n, H(vr).RConn(1), hN);
end

% =============================================================================================
% Plant SYS (spec 3.1): Motor & Drive (System Level, thermal) + DC link + aux loads
% =============================================================================================
function local_plant_sys(D)
n = 'drive_plant_sys'; f = fullfile(D, [n '.slx']); fresh(n, f);
local_plant_ports(n);
[hP, hN, ci, vs] = local_dc_link(n);
local_coil(n, hP, hN);
md = A(sprintf('ee_lib/Electromechanical/Motor & Drive\n(System Level)'), [n '/MotorDrive'], 'Orientation', 'right');
% thermal block variant (set_param of ComponentPath does not persist on a library link; verified)
simscape.compiler.sli.internal.setActiveVariant(md, 'ee.electromech.motor_and_drive_thermal');
set_param(md, 'torque_speed_param', 'ee.enum.electromech.envelope.tabulated2D');
set_param(md, 'T_t_2D', 'P.drive.motor.T_pk_2D', 'w_t', 'P.drive.motor.w_t_rpm', 'w_t_unit', 'rpm', ...
    'vdc_t', 'P.drive.motor.vdc_t', 'vdc_t_unit', 'V', 'overtorque_param', 'ee.enum.electromech.overtorque.no', ...
    'Tc', 'P.drive.motor.Tc', 'EnableSwitch', 'true');
set_param(md, 'losses_param', ...
    'ee.enum.electromech.motorAndDrive.lossParameterization.TabulatedLossDataAsAFunctionOfSpeedAndTorque');
set_param(md, 'w_eff_vec', 'P.drive.motor.w_eff_rpm', 'w_eff_vec_unit', 'rpm', 'T_eff_vec', 'P.drive.motor.T_eff', ...
    'losses_mat', 'P.drive.motor.loss_T1', 'losses_mat_T2', 'P.drive.motor.loss_T2', ...
    'Tmeas', 'P.drive.motor.Tmeas1', 'Tmeas_unit', 'degC', 'Tmeas2', 'P.drive.motor.Tmeas2', 'Tmeas2_unit', 'degC', ...
    'Res', 'P.drive.motor.Res', 'J', 'P.drive.motor.J', 'lam', 'P.drive.motor.lam', 'speed0', '0', ...
    'thermal_mass', 'P.drive.thermal.C_w', 'initial_temperature', 'P.drive.thermal.T_amb0', 'initial_temperature_unit', 'K');
% Motor & Drive thermal ports with the enable switch (checked in the live session):
% LConn = [+, Tr, R, EN], RConn = [-, w (PS out), H, C]. EN = ESC power-stage enable (risk R6: no DC power
% is drawn with the inverter off, so an open contactor never drives the DC link below zero)
pm = H(md);
cim = A('ee_lib/Sensors & Transducers/Current Sensor', [n '/I_motor']);      % block DC current (through Res)
PL(n, hP, H(cim).LConn(1)); PL(n, H(cim).RConn(2), pm.LConn(1)); PL(n, pm.RConn(1), hN);
sp = S2PS(n, 'Tr', 'N*m'); L(n, 'CmdSplit/1', 'Tr/1', 'Tr'); PL(n, H(sp).RConn(1), pm.LConn(2));
% the EN input switches the drive OFF when > 0.5 (checked in the live session): EN = 1 - enable
C(n, 'One', '1'); SUM(n, 'SwitchOff', '+-');
L(n, 'One/1', 'SwitchOff/1'); L(n, 'CmdSplit/4', 'SwitchOff/2', 'en');
se = S2PS(n, 'Off', '1'); L(n, 'SwitchOff/1', 'Off/1'); PL(n, H(se).RConn(1), pm.LConn(4));
A('simulink/Sinks/Terminator', [n '/TermWcmd']); L(n, 'CmdSplit/3', 'TermWcmd/1', 'w_cmd');
cs = MREF(n, 'Case');
PL(n, pm.RConn(4), H(cs).LConn(1));
PL(n, pm.RConn(3), RC(n, 'H'));
% the block's own speed output w is not used (the shaft speed comes from ShaftMotion): a PS Terminator (no
% equation) ends it, so that no port is left open
ptw = A('fl_lib/Physical Signals/Sinks/PS Terminator', [n '/Term_w']); PL(n, pm.RConn(2), H(ptw).LConn(1));
ts = A('fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor', [n '/ShaftTorque']);
PL(n, pm.LConn(3), H(ts).LConn(1)); PL(n, H(ts).RConn(1), RC(n, 'Shaft'));
ms = EMOT(n, 'ShaftMotion');
PL(n, RC(n, 'Shaft'), H(ms).LConn(1)); PL(n, H(ms).RConn(1), H(cs).LConn(1));
% Winding heat = P_loss only (spec 2.5 / 2.6: P_cu + P_fe + P_esc into node W). The thermal variant of Motor & Drive
% also puts its rotor damping lam w^2 (bearing / seal friction, mechanical, spec 2.5) and the DC-harness loss
% Res i^2 (spec 6.3) into port H; a Controlled Heat Flow Rate Source returns that heat from H to the thermal
% reference (rejected to ambient), so T_w follows the spec network (T5). Energy audit (T13) unaffected.
PSG = 'fl_lib/Physical Signals/Functions/PS Gain'; PSP = 'fl_lib/Physical Signals/Functions/PS Product';
pi2 = A(PSP, [n '/I_sq']); pw2 = A(PSP, [n '/W_sq']);
pgr = A(PSG, [n '/Res_loss'], 'gain', 'P.drive.motor.Res', 'gain_unit', 'Ohm');
pgl = A(PSG, [n '/Damp_loss'], 'gain', 'P.drive.motor.lam', 'gain_unit', 'N*m*s/rad');
pad = A('fl_lib/Physical Signals/Functions/PS Add', [n '/NonWindingHeat']);
hx = A(sprintf('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow\nRate Source'), [n '/HeatToAmbient']);
trf = A('fl_lib/Thermal/Thermal Elements/Thermal Reference', [n '/ThermalRef']);
PL(n, H(cim).RConn(1), H(pi2).LConn(1)); PL(n, H(cim).RConn(1), H(pi2).LConn(2));
PL(n, H(ms).RConn(2), H(pw2).LConn(1)); PL(n, H(ms).RConn(2), H(pw2).LConn(2));
PL(n, H(pi2).RConn(1), H(pgr).LConn(1)); PL(n, H(pw2).RConn(1), H(pgl).LConn(1));
PL(n, H(pgr).RConn(1), H(pad).LConn(1)); PL(n, H(pgl).RConn(1), H(pad).LConn(2));
% source polarity (as in drive_thermal): S > 0 moves heat from RConn(2) (= H) into LConn(1) (= reference)
PL(n, H(pad).RConn(1), H(hx).RConn(1)); PL(n, H(hx).RConn(2), RC(n, 'H')); PL(n, H(hx).LConn(1), H(trf).LConn(1));
local_plant_meas(n, ms, ts, vs, ci);
finish(n, f, ['Drive plant SYS (spec 3.1, VC_DRIVE_FID == 1): Motor & Drive (System Level) thermal variant, tabulated 2-D peak ' ...
    'envelope and loss maps at 25 / 125 degC from P.drive.motor, torque lag Tc; ESC DC link (C_link 2.2 mF + ESR), quiescent ' ...
    'R_q, bleed 10 kOhm, brake-coil Variable Resistor R_coil / duty, DC current / voltage sensors. Thermal port H = winding node ' ...
    '(winding heat = P_loss; rotor damping and DC-harness Res heat of the block returned to the thermal reference). ' ...
    'Enable switch driven by the ESC power-stage enable (risk R6). cmd = [Tr; duty; w_cmd; en], ' ...
    'meas = [theta_m; w_m; tau_m; v_dc; i_dc]. Owner drive (T06).']);
end

% =============================================================================================
% Plant IDEAL (spec 2.11, D35, D52): speed source + energy-equivalent DC load + network tie
% =============================================================================================
function local_plant_ideal(D)
n = 'drive_plant_ideal'; f = fullfile(D, [n '.slx']); fresh(n, f);
local_plant_ports(n);
[hP, hN, ci, vs] = local_dc_link(n);
A('simulink/Sinks/Terminator', [n '/TermTr']); L(n, 'CmdSplit/1', 'TermTr/1', 'Tr');
% speed: first-order lag tau_v + acceleration limit a_max -> Ideal Angular Velocity Source
A('simulink/Discrete/Discrete Transfer Fcn', [n '/SpeedLag'], 'Numerator', '[1-P.drive.ideal.a_v]', ...
    'Denominator', '[1 -P.drive.ideal.a_v]', 'SampleTime', 'P.drive.Ts_esc');
A('simulink/Discontinuities/Rate Limiter', [n '/AccelLimit'], 'RisingSlewLimit', 'P.drive.ideal.a_max', ...
    'FallingSlewLimit', '-P.drive.ideal.a_max', 'InitialCondition', '0');
sw = S2PS(n, 'w_set', 'rad/s');
L(n, 'CmdSplit/3', 'SpeedLag/1', 'w_cmd'); L(n, 'SpeedLag/1', 'AccelLimit/1'); L(n, 'AccelLimit/1', 'w_set/1');
vsrc = A(sprintf('fl_lib/Mechanical/Mechanical Sources/Ideal Angular\nVelocity Source'), [n '/SpeedSource']);
cs = MREF(n, 'Case');
ts = A('fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor', [n '/ShaftTorque']);
PL(n, H(sw).RConn(1), H(vsrc).RConn(1));
PL(n, H(vsrc).LConn(1), H(ts).LConn(1)); PL(n, H(ts).RConn(1), RC(n, 'Shaft')); PL(n, H(vsrc).RConn(2), H(cs).LConn(1));
ms = EMOT(n, 'ShaftMotion');
PL(n, RC(n, 'Shaft'), H(ms).LConn(1)); PL(n, H(ms).RConn(1), H(cs).LConn(1));
% DC load: P = P_mech + k_loss |P_mech| + coil; i = P / max(v, V_min) * sat((v - V_min + V_gate)/V_gate, 0, 1)
PSG = 'fl_lib/Physical Signals/Functions/PS Gain';
pm = A('fl_lib/Physical Signals/Functions/PS Product', [n '/P_mech']);
pab = A('fl_lib/Physical Signals/Nonlinear Operators/PS Abs', [n '/AbsP']);
pkl = A(PSG, [n '/k_loss'], 'gain', 'P.drive.ideal.k_loss');
pad1 = A('fl_lib/Physical Signals/Functions/PS Add', [n '/P_drive']);
pad2 = A('fl_lib/Physical Signals/Functions/PS Add', [n '/P_load']);
pmx = A('fl_lib/Physical Signals/Nonlinear Operators/PS Max', [n '/V_floor']);
pvm = A('fl_lib/Physical Signals/Sources/PS Constant', [n '/V_min'], 'constant', 'P.drive.ideal.V_min', 'constant_unit', 'V');
pdv = A('fl_lib/Physical Signals/Functions/PS Divide', [n '/I_load']);
pg1 = A(PSG, [n '/GateScale'], 'gain', '1/P.drive.ideal.V_gate', 'gain_unit', '1/V');
pgo = A('fl_lib/Physical Signals/Sources/PS Constant', [n '/GateOfs'], 'constant', ...
    '-(P.drive.ideal.V_min - P.drive.ideal.V_gate)/P.drive.ideal.V_gate');
pga = A('fl_lib/Physical Signals/Functions/PS Add', [n '/GateSum']);
pgs = A('fl_lib/Physical Signals/Nonlinear Operators/PS Saturation', [n '/Gate'], 'upper_limit', '1', 'lower_limit', '0');
pig = A('fl_lib/Physical Signals/Functions/PS Product', [n '/I_gated']);
isrc = A('fl_lib/Electrical/Electrical Sources/Controlled Current Source', [n '/DcLoad']);
PL(n, H(ts).RConn(2), H(pm).LConn(1)); PL(n, H(ms).RConn(2), H(pm).LConn(2));
PL(n, H(pm).RConn(1), H(pab).LConn(1)); PL(n, H(pab).RConn(1), H(pkl).LConn(1));
PL(n, H(pm).RConn(1), H(pad1).LConn(1)); PL(n, H(pkl).RConn(1), H(pad1).LConn(2));
% coil power duty * V_nom^2 / R_coil (Simulink, from the delayed ESC duty)
K(n, 'CoilPower', 'P.drive.motor.V_nom^2/P.drive.brake.R_coil'); pc = S2PS(n, 'P_coil', 'W');
L(n, 'CmdSplit/2', 'CoilPower/1', 'duty'); L(n, 'CoilPower/1', 'P_coil/1');
PL(n, H(pad1).RConn(1), H(pad2).LConn(1)); PL(n, H(pc).RConn(1), H(pad2).LConn(2));
PL(n, H(vs).RConn(1), H(pmx).LConn(1)); PL(n, H(pvm).RConn(1), H(pmx).LConn(2));
PL(n, H(pad2).RConn(1), H(pdv).LConn(1)); PL(n, H(pmx).RConn(1), H(pdv).LConn(2));
PL(n, H(vs).RConn(1), H(pg1).LConn(1)); PL(n, H(pg1).RConn(1), H(pga).LConn(1)); PL(n, H(pgo).RConn(1), H(pga).LConn(2));
PL(n, H(pga).RConn(1), H(pgs).LConn(1));
PL(n, H(pdv).RConn(1), H(pig).LConn(1)); PL(n, H(pgs).RConn(1), H(pig).LConn(2));
% power-stage enable (no DC current with the inverter off)
pen = S2PS(n, 'Enable', '1'); L(n, 'CmdSplit/4', 'Enable/1', 'en');
pie = A('fl_lib/Physical Signals/Functions/PS Product', [n '/I_enabled']);
PL(n, H(pig).RConn(1), H(pie).LConn(1)); PL(n, H(pen).RConn(1), H(pie).LConn(2));
PL(n, H(pie).RConn(1), H(isrc).RConn(1));
% Controlled Current Source polarity (probed in the live session): I > 0 leaves the block at LConn(1), so the load
% draws I from DC+ at RConn(2) and returns it to DC- at LConn(1)
PL(n, hN, H(isrc).LConn(1)); PL(n, H(isrc).RConn(2), hP);
% loss heat into the winding node H
hq = A(sprintf('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow\nRate Source'), [n '/LossHeat']);
tref = A('fl_lib/Thermal/Thermal Elements/Thermal Reference', [n '/ThermalRef']);
% heat source polarity: S > 0 moves heat from RConn(2) (reference) into LConn(1) (winding node H)
PL(n, H(pkl).RConn(1), H(hq).RConn(1)); PL(n, H(hq).RConn(2), H(tref).LConn(1)); PL(n, H(hq).LConn(1), RC(n, 'H'));
% network tie (ICD 2, D35): converter R on the shaft, C on the case, - on DC-, + via 1 MOhm thermal resistor to DC-
cv = A(sprintf('fl_lib/Electrical/Electrical Elements/Rotational\nElectromechanical\nConverter'), [n '/Tie'], 'K', 'P.drive.tie.K');
tr = A('ee_lib/Passive/Resistor', [n '/Tie_R']);
simscape.compiler.sli.internal.setActiveVariant(tr, 'ee.passive.resistor_thermal');
set_param(tr, 'R', 'P.drive.tie.R', 'thermal_mass', 'P.drive.thermal.C_w');
set_init(tr, 'temperature', 'P.drive.thermal.T_amb0', 'K');
pcv = H(cv); ptr = H(tr);
PL(n, pcv.LConn(2), RC(n, 'Shaft')); PL(n, pcv.RConn(2), H(cs).LConn(1)); PL(n, pcv.RConn(1), hN);
PL(n, pcv.LConn(1), ptr.LConn(1)); PL(n, ptr.RConn(1), hN); PL(n, ptr.LConn(2), RC(n, 'H'));
local_plant_meas(n, ms, ts, vs, ci);
finish(n, f, ['Drive plant IDEAL (spec 2.11, VC_DRIVE_FID == 3): Ideal Angular Velocity Source following the ESC speed command ' ...
    '(first-order 50 ms + 420 rad/s^2), energy-equivalent DC load P_mech + k_loss |P_mech| + coil (Controlled Current Source), ' ...
    'loss heat into port H, ESC DC link / quiescent / bleed as SYS, and the network tie of ICD 2 (Rotational Electromechanical ' ...
    'Converter K = P.drive.tie.K, 1 MOhm ee.passive.resistor_thermal with its thermal port on H, D35) so the ports equal SYS. ' ...
    'Owner drive (T06).']);
end

function local_plant_foc(D)
% bench-only FOC plant (spec 3.7, D52): same ports as drive_plant_sys; PMSM (thermal) + average-value
% inverter (thermal) + ee_sl_lib PMSM current control at P.drive.Ts_foc; used only in tests/drive/tb_drive_foc
n = 'drive_plant_foc'; f = fullfile(D, [n '.slx']); fresh(n, f);
TSF = 'P.drive.Ts_foc';
local_plant_ports(n);
[hP, hN, ci, vs] = local_dc_link(n);
local_coil(n, hP, hN);
A('simulink/Sinks/Terminator', [n '/TermWcmd']); L(n, 'CmdSplit/3', 'TermWcmd/1', 'w_cmd');
% ---- power stage and machine ----
inv = A('ee_lib/Semiconductors & Converters/Converters/Average-Value Voltage Source Converter (Three-Phase)', [n '/Inverter']);
simscape.compiler.sli.internal.setActiveVariant(inv, 'ee.semiconductors.converters.ideal_voltage_source_converter.abc_thermal');
set_param(inv, 'enable_block_pulses_input', 'true', 'input_option', 'ee.enum.converters.inputOption.ModWave', ...
    'frequency_option', 'ee.enum.converters.frequency.variable', 'losses_parameterization', 'ee.enum.converters.losses.profile', ...
    'Ploss_vect', 'P.drive.foc.Ploss_vect', 'Irms_vect', 'P.drive.foc.Irms_vect', 'Vdc_nom', 'P.drive.motor.V_nom', ...
    'thermal_mass', 'P.drive.foc.thermal_mass_inv');
set_init(inv, 'temperature', 'P.drive.thermal.T_amb0', 'K');
pm = A('ee_lib/Electromechanical/Permanent Magnet/PMSM', [n '/PMSM']);
simscape.compiler.sli.internal.setActiveVariant(pm, 'ee.electromech.pmsm.pm_rotor.pmsm.abc_thermal');
set_param(pm, 'nPolePairs', 'P.drive.motor.p', 'pmflux_param', 'ee.enum.pmflux.fluxlinkage', ...
    'pm_flux_linkage', 'P.drive.motor.lambda_pm', 'stator_param', 'ee.enum.statorparam.LdLqL0', ...
    'Ld', 'P.drive.motor.Ld', 'Lq', 'P.drive.motor.Lq', 'L0', 'P.drive.motor.L0', 'Rs', 'P.drive.motor.Rs', ...
    'axes_param', 'ee.enum.rotorangle.daxis', 'loss_param', 'ee.enum.ironloss.empirical', ...
    'losses_oc', 'P.drive.foc.losses_oc', 'f_losses', 'P.drive.foc.f_losses', 'losses_sc', '[0 0 0]', ...
    'J', 'P.drive.motor.J', 'lam', 'P.drive.motor.lam', 'alpha', 'P.drive.motor.alpha_cu', ...
    'stator_thermal_mass', 'P.drive.foc.C_stator', 'rotor_thermal_mass', 'P.drive.foc.C_rotor');
for th = {'temperatureA', 'temperatureB', 'temperatureC', 'temperatureR'}
    set_init(pm, th{1}, 'P.drive.thermal.T_amb0', 'K');
end
i3 = A('ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)', [n '/I_abc']);
% inverter ports: LConn = [ModWave, ~abc, Enable], RConn = [+, -, H]
% PMSM ports:    LConn = [~abc, n, H_A], RConn = [R, C, H_B, H_C, H_R]
pi_ = H(inv); pp = H(pm); p3 = H(i3);
PL(n, pi_.RConn(1), hP); PL(n, pi_.RConn(2), hN);
PL(n, pi_.LConn(2), p3.LConn(1)); PL(n, p3.RConn(2), pp.LConn(1));
% star point to DC- through 1 MOhm: keeps the machine node voltages defined while the inverter pulses are blocked
rn = A('fl_lib/Electrical/Electrical Elements/Resistor', [n '/R_neutral'], 'R', 'P.drive.tie.R');
PL(n, pp.LConn(2), H(rn).LConn(1)); PL(n, H(rn).RConn(1), hN);
hH = RC(n, 'H');
PL(n, pi_.RConn(3), hH);
PL(n, pp.LConn(3), hH); PL(n, pp.RConn(3), hH); PL(n, pp.RConn(4), hH); PL(n, pp.RConn(5), hH);
cs = MREF(n, 'Case');
ts = A('fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor', [n '/ShaftTorque']);
PL(n, pp.RConn(1), H(ts).LConn(1)); PL(n, H(ts).RConn(1), RC(n, 'Shaft')); PL(n, pp.RConn(2), H(cs).LConn(1));
ms = EMOT(n, 'ShaftMotion');
PL(n, RC(n, 'Shaft'), H(ms).LConn(1)); PL(n, H(ms).RConn(1), H(cs).LConn(1));
% ---- control at Ts_foc (field-oriented current control) ----
RT = 'simulink/Signal Attributes/Rate Transition';
rt = @(nm) A(RT, [n '/' nm], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', TSF);
rt('RT_Tr'); rt('RT_en');
L(n, 'CmdSplit/1', 'RT_Tr/1', 'Tr'); L(n, 'CmdSplit/4', 'RT_en/1', 'en');
fb = {'theta', 'rad', H(ms).RConn(3); 'omega', 'rad/s', H(ms).RConn(2); 'vdc', 'V', H(vs).RConn(1); 'iabc', 'A', p3.RConn(1)};
for k = 1:size(fb, 1)
    c = PS2S(n, ['m_' fb{k, 1}], fb{k, 2}); PL(n, fb{k, 3}, H(c).LConn(1));
    rt(['RT_' fb{k, 1}]); L(n, ['m_' fb{k, 1} '/1'], ['RT_' fb{k, 1} '/1']);
end
K(n, 'ThetaE', 'P.drive.motor.p'); L(n, 'RT_theta/1', 'ThetaE/1');
K(n, 'OmegaE', 'P.drive.motor.p'); L(n, 'RT_omega/1', 'OmegaE/1');
PROD(n, 'TqCmd', '**'); L(n, 'RT_Tr/1', 'TqCmd/1'); L(n, 'RT_en/1', 'TqCmd/2');
rg = A('ee_sl_lib/PMSM Control/PMSM Current Reference Generator', [n '/CurrentRef'], 'RefType', 'Zero d-axis control', ...
    'Vnom', 'P.drive.motor.V_nom', 'Pmax', 'P.drive.foc.P_max', 'Tmax', 'P.drive.motor.T_peak', 'p', 'P.drive.motor.p', ...
    'psim', 'P.drive.motor.lambda_pm', 'Ld', 'P.drive.motor.Ld', 'Lq', 'P.drive.motor.Lq', 'Rs', 'P.drive.motor.Rs', ...
    'Ts', TSF, 'PWMMode', 'SVM: space vector modulation');
L(n, 'TqCmd/1', 'CurrentRef/1'); L(n, 'RT_omega/1', 'CurrentRef/2'); L(n, 'RT_vdc/1', 'CurrentRef/3');
A('simulink/Sinks/Terminator', [n '/TermTqSat']); A('simulink/Sinks/Terminator', [n '/TermTqLim']);
L(n, 'CurrentRef/2', 'TermTqSat/1', 'tq_sat'); L(n, 'CurrentRef/3', 'TermTqLim/1', 'tq_lim');
pk = A('ee_sl_lib/Mathematical Transforms/Park Transform', [n '/Park'], 'AxisAlignment', 'D-axis');
L(n, 'RT_iabc/1', 'Park/1'); L(n, 'ThetaE/1', 'Park/2');
A('simulink/Signal Routing/Selector', [n '/Idq'], 'Indices', '[1 2]', 'InputPortWidth', '3');
L(n, 'Park/1', 'Idq/1');
% decoupling feed-forward: vd = -we Lq iq, vq = we (Ld id + psi)
DEMUX(n, 'IdIq', 2); L(n, 'Idq/1', 'IdIq/1');
K(n, 'LqIq', '-P.drive.motor.Lq'); L(n, 'IdIq/2', 'LqIq/1', 'i_q');
K(n, 'LdId', 'P.drive.motor.Ld'); L(n, 'IdIq/1', 'LdId/1', 'i_d');
A('simulink/Math Operations/Bias', [n '/PlusPsi'], 'Bias', 'P.drive.motor.lambda_pm'); L(n, 'LdId/1', 'PlusPsi/1');
PROD(n, 'Vdff', '**'); PROD(n, 'Vqff', '**');
L(n, 'OmegaE/1', 'Vdff/1'); L(n, 'LqIq/1', 'Vdff/2'); L(n, 'OmegaE/1', 'Vqff/1'); L(n, 'PlusPsi/1', 'Vqff/2');
MUX(n, 'VdqFF', 2); L(n, 'Vdff/1', 'VdqFF/1'); L(n, 'Vqff/1', 'VdqFF/2');
K(n, 'VphMax', '1/sqrt(3)'); L(n, 'RT_vdc/1', 'VphMax/1');
REL(n, 'Off', '<', '0.5'); L(n, 'RT_en/1', 'Off/1');
cc = A('ee_sl_lib/PMSM Control/PMSM Current Controller', [n '/CurrentCtrl'], 'Kp_id', 'P.drive.foc.Kp_i', 'Ki_id', 'P.drive.foc.Ki_i', ...
    'Kp_iq', 'P.drive.foc.Kp_i', 'Ki_iq', 'P.drive.foc.Ki_i', 'Ts', TSF, 'Tsd', TSF, 'AxisPrio', 'Q-axis', 'ActFF', 'on');
L(n, 'CurrentRef/1', 'CurrentCtrl/1', 'idq_ref'); L(n, 'Idq/1', 'CurrentCtrl/2'); L(n, 'VdqFF/1', 'CurrentCtrl/3');
L(n, 'VphMax/1', 'CurrentCtrl/4'); L(n, 'Off/1', 'CurrentCtrl/5');
C(n, 'Zero0', '0'); MUX(n, 'Vdq0', 2);
L(n, 'CurrentCtrl/1', 'Vdq0/1'); L(n, 'Zero0/1', 'Vdq0/2');
ip = A('ee_sl_lib/Mathematical Transforms/Inverse Park Transform', [n '/InvPark'], 'AxisAlignment', 'D-axis');
L(n, 'Vdq0/1', 'InvPark/1'); L(n, 'ThetaE/1', 'InvPark/2');
% min-max zero-sequence injection (SVPWM-equivalent), modulation waves m = v / (Vdc/2)
MINMAX(n, 'Vmax', 'max', 1); MINMAX(n, 'Vmin', 'min', 1);
SUM(n, 'MidSum', '++'); K(n, 'Half', '-0.5'); SUM(n, 'VInj', '++');
L(n, 'InvPark/1', 'Vmax/1'); L(n, 'InvPark/1', 'Vmin/1'); L(n, 'Vmax/1', 'MidSum/1'); L(n, 'Vmin/1', 'MidSum/2');
L(n, 'MidSum/1', 'Half/1'); L(n, 'InvPark/1', 'VInj/1'); L(n, 'Half/1', 'VInj/2');
K(n, 'HalfVdc', '0.5'); A('simulink/Discontinuities/Saturation', [n '/VdcFloor'], 'UpperLimit', 'inf', 'LowerLimit', '1');
PROD(n, 'Mod', '*/'); A('simulink/Discontinuities/Saturation', [n '/ModSat'], 'UpperLimit', '1', 'LowerLimit', '-1');
L(n, 'RT_vdc/1', 'HalfVdc/1'); L(n, 'HalfVdc/1', 'VdcFloor/1');
L(n, 'VInj/1', 'Mod/1'); L(n, 'VdcFloor/1', 'Mod/2'); L(n, 'Mod/1', 'ModSat/1');
smw = S2PS(n, 'ModWave', '1'); L(n, 'ModSat/1', 'ModWave/1'); PL(n, H(smw).RConn(1), pi_.LConn(1));
% inverter enable input (pulses enabled when > 0.5, checked in the live session) = ESC power-stage enable
sbp = S2PS(n, 'PulseEnable', '1'); L(n, 'RT_en/1', 'PulseEnable/1'); PL(n, H(sbp).RConn(1), pi_.LConn(3));
local_plant_meas(n, ms, ts, vs, ci);
finish(n, f, ['Drive plant FOC (spec 3.7, BENCH-ONLY, D52): PMSM (thermal variant, empirical iron losses) + Average-Value ' ...
    'Voltage Source Converter (thermal, loss profile of the SYS map, pulse blocking from the ESC enable) + ee_sl_lib PMSM ' ...
    'Current Reference Generator / Current Controller (Kp 0.25 V/A, Ki 25 V/(A s)) with decoupling feed-forward and min-max ' ...
    'zero-sequence injection at P.drive.Ts_foc (1e-4 s; outside the system rate set). Same ports as drive_plant_sys; used ' ...
    'only in tests/drive/tb_drive_foc with its own Solver Configuration at P.drive.Ts_foc_plant. Owner drive (T06).']);
end

% =============================================================================================
% drive_side (one ESC + motor + gearbox + brake + thermal + truth), mask parameter side
% =============================================================================================
function local_side(D)
n = 'drive_side'; f = fullfile(D, [n '.slx']); fresh(n, f);
local_side_body(n, 'variant');
mo = Simulink.Mask.create(n);
mo.addParameter('Name', 'side', 'Prompt', 'Side (1 = Left / node 1, 2 = Right / node 2)', 'Value', '1', 'Type', 'edit');
finish(n, f, ['One traction drive (spec 7): ESC (drive_esc), plant variant VC_DRIVE_FID (1 SYS drive_plant_sys, 3 IDEAL ' ...
    'drive_plant_ideal; update diagram, identical ports), gearbox, brake, thermal, truth bus with E_dc / E_spr / E_loss ' ...
    'integrals (D40). Mask parameter side (1 L, 2 R) selects the fault-injection sub-bus; no sign parameter. Owner drive (T06).']);
end

function local_side_body(n, plantMode)
% plantMode 'variant': Variant Subsystem VC_DRIVE_FID (SYS | IDEAL); 'foc': drive_plant_foc (bench-only)
TS = 'P.drive.Ts_esc';
inport(n, 'cmd', 1, 'Bus: Bus_DriveSideCmd', TS);
inport(n, 'sto_ok', 2, 'boolean', TS);
inport(n, 'T_amb', 3, 'double', TS);
inport(n, 'fault_inj', 4, 'Bus: Bus_DriveFaultInj', TS);
outport(n, 'fb', 1, 'Bus: Bus_DriveSideFb');
outport(n, 'truth', 2, 'Bus: Bus_DriveSideTruth');
cport(n, 'DC_p', 1, 'Left', 'E');
cport(n, 'DC_n', 2, 'Left', 'E');
cport(n, 'Spr', 3, 'Right', 'M');
% side selects the fault-injection sub-bus (no sign parameter: forward-positive on both sides)
BS(n, 'FaultSides', {'L', 'R'});
C(n, 'Side', 'side');
SW(n, 'FaultSel', 'u2 >= Threshold', '1.5');
L(n, 'fault_inj/1', 'FaultSides/1');
L(n, 'FaultSides/2', 'FaultSel/1'); L(n, 'Side/1', 'FaultSel/2'); L(n, 'FaultSides/1', 'FaultSel/3');
BS(n, 'FaultStuck', {'brake_stuck'});
L(n, 'FaultSel/1', 'FaultStuck/1', 'fault_inj_side');
% blocks
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/ESC'], 'ReferencedSubsystem', 'drive_esc');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Gearbox'], 'ReferencedSubsystem', 'drive_gearbox');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Brake'], 'ReferencedSubsystem', 'drive_brake');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Thermal'], 'ReferencedSubsystem', 'drive_thermal');
% plant fidelity variant (VC_DRIVE_FID: 1 SYS, 3 IDEAL; FOC is bench-only, D52)
vs = [n '/Plant'];
if strcmp(plantMode, 'foc')
    A('simulink/Ports & Subsystems/Subsystem Reference', vs, 'ReferencedSubsystem', 'drive_plant_foc');
else
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
c = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
c = setdiff(c, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
add_block('simulink/Sources/In1', [vs '/cmd']);
add_block('simulink/Sinks/Out1', [vs '/meas']);
cport(vs, 'DC_p', 1, 'Left', 'E'); cport(vs, 'DC_n', 2, 'Left', 'E');
cport(vs, 'Shaft', 3, 'Right', 'M'); cport(vs, 'H', 4, 'Right', 'T');
add_block('simulink/Ports & Subsystems/Subsystem Reference', [vs '/SYS'], 'ReferencedSubsystem', 'drive_plant_sys', ...
    'VariantControl', 'VC_DRIVE_FID == 1');
add_block('simulink/Ports & Subsystems/Subsystem Reference', [vs '/IDEAL'], 'ReferencedSubsystem', 'drive_plant_ideal', ...
    'VariantControl', 'VC_DRIVE_FID == 3');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end
% 1 ms re-timing (Rate Transition) of every plant signal: discrete in fixed (local solver) and variable-step reference runs
ZOH(n, 'ZOH_meas'); ZOH(n, 'ZOH_gb'); ZOH(n, 'ZOH_brk'); ZOH(n, 'ZOH_T');
L(n, 'Plant/1', 'ZOH_meas/1', 'meas'); L(n, 'Gearbox/1', 'ZOH_gb/1', 'gb'); L(n, 'Brake/1', 'ZOH_brk/1', 'brk');
L(n, 'Thermal/1', 'ZOH_T/1', 'T');
DEMUX(n, 'Meas', 5); DEMUX(n, 'Gb', 3); DEMUX(n, 'Brk', 2); DEMUX(n, 'Temp', 3);
L(n, 'ZOH_meas/1', 'Meas/1', 'meas_1ms'); L(n, 'ZOH_gb/1', 'Gb/1', 'gb_1ms'); L(n, 'ZOH_brk/1', 'Brk/1', 'brk_1ms');
L(n, 'ZOH_T/1', 'Temp/1', 'T_1ms');
% Meas: 1 theta_m 2 w_m 3 tau_m 4 v_dc 5 i_dc ; Gb: 1 theta_spr 2 w_spr 3 tau_spr ; Brk: 1 tau_brake 2 f_rel ; Temp: 1 T_w 2 T_h 3 T_esc
% ESC: cmd sto_ok fault_inj theta_m v_dc i_dc T_w T_esc -> plant_cmd, P_esc, fb
L(n, 'cmd/1', 'ESC/1'); L(n, 'sto_ok/1', 'ESC/2'); L(n, 'FaultSel/1', 'ESC/3');
L(n, 'Meas/1', 'ESC/4'); L(n, 'Meas/4', 'ESC/5'); L(n, 'Meas/5', 'ESC/6'); L(n, 'Temp/1', 'ESC/7'); L(n, 'Temp/3', 'ESC/8');
L(n, 'ESC/1', 'Plant/1', 'plant_cmd'); L(n, 'ESC/3', 'fb/1', 'fb');
L(n, 'ESC/2', 'Thermal/1', 'P_esc'); L(n, 'T_amb/1', 'Thermal/2');
% brake release command = coil energised (delayed duty > 0)
DEMUX(n, 'PlantCmd', 4);
L(n, 'ESC/1', 'PlantCmd/1');
A('simulink/Sinks/Terminator', [n '/TermTr']); A('simulink/Sinks/Terminator', [n '/TermW']); A('simulink/Sinks/Terminator', [n '/TermEn']);
L(n, 'PlantCmd/1', 'TermTr/1', 'Tr'); L(n, 'PlantCmd/3', 'TermW/1', 'w_cmd'); L(n, 'PlantCmd/4', 'TermEn/1', 'en');
REL(n, 'CoilOn', '>', '0'); DTC(n, 'RelCmd', 'double');
L(n, 'PlantCmd/2', 'CoilOn/1', 'duty'); L(n, 'CoilOn/1', 'RelCmd/1', 'coil_on'); L(n, 'RelCmd/1', 'Brake/1', 'rel_cmd');
L(n, 'FaultStuck/1', 'Brake/2');
% physical network
PL(n, RC(n, 'DC_p'), CP(vs, 'DC_p')); PL(n, RC(n, 'DC_n'), CP(vs, 'DC_n'));
PL(n, CP(vs, 'Shaft'), CP([n '/Gearbox'], 'B')); PL(n, CP([n '/Brake'], 'Shaft'), CP(vs, 'Shaft'));
PL(n, CP([n '/Gearbox'], 'F'), RC(n, 'Spr'));
PL(n, CP(vs, 'H'), CP([n '/Thermal'], 'H'));
% truth (Bus_DriveSideTruth) and energy integrals at 1 ms (D40)
PROD(n, 'p_dc', '**'); PROD(n, 'p_spr', '**'); PROD(n, 'p_shaft', '**');
SUM(n, 'P_loss', '+-'); SUM(n, 'p_diff', '+-');
L(n, 'Meas/4', 'p_dc/1'); L(n, 'Meas/5', 'p_dc/2');
L(n, 'Gb/3', 'p_spr/1'); L(n, 'Gb/2', 'p_spr/2');
L(n, 'Meas/3', 'p_shaft/1'); L(n, 'Meas/2', 'p_shaft/2');
L(n, 'p_dc/1', 'P_loss/1'); L(n, 'p_shaft/1', 'P_loss/2');
L(n, 'p_dc/1', 'p_diff/1'); L(n, 'p_spr/1', 'p_diff/2');
ig = {'E_dc', 'p_dc/1'; 'E_spr', 'p_spr/1'; 'E_loss', 'p_diff/1'};
for k = 1:3
    A('simulink/Discrete/Discrete-Time Integrator', [n '/' ig{k, 1}], 'IntegratorMethod', 'Integration: Forward Euler', ...
        'gainval', '1', 'InitialCondition', '0', 'SampleTime', TS);
    L(n, ig{k, 2}, [ig{k, 1} '/1']);
end
BC(n, 'Truth', 'Bus_DriveSideTruth', 19);
src = {'Meas/2', 'w_m'; 'Meas/1', 'theta_m'; 'Gb/2', 'w_spr'; 'Gb/1', 'theta_spr'; 'Meas/3', 'tau_m'; 'Gb/3', 'tau_spr'; ...
       'Brk/1', 'tau_brake'; 'Brk/2', 'f_brake_rel'; 'Meas/4', 'v_dc'; 'Meas/5', 'i_dc'; 'p_dc/1', 'p_dc'; 'p_spr/1', 'p_spr'; ...
       'P_loss/1', 'P_loss'; 'Temp/1', 'T_winding'; 'Temp/2', 'T_housing'; 'Temp/3', 'T_esc'; 'E_dc/1', 'E_dc'; ...
       'E_spr/1', 'E_spr'; 'E_loss/1', 'E_loss'};
for k = 1:19, L(n, src{k, 1}, ['Truth/' num2str(k)], src{k, 2}); end
L(n, 'Truth/1', 'truth/1', 'truth');
end

% =============================================================================================
% drive_system (ICD 3.2 Drive row, build plan T06.1)
% =============================================================================================
function local_system(D)
n = 'drive_system'; f = fullfile(D, [n '.slx']); fresh(n, f);
inport(n, 'can_rx', 1, 'Bus: Bus_Can1_McuFast', 'Ts.can_drive');
inport(n, 'sto_ok', 2, 'boolean', 'Ts.base');
inport(n, 'T_amb', 3, 'double', 'Ts.hk');
inport(n, 'fault_inj', 4, 'Bus: Bus_DriveFaultInj', 'Ts.base');
outport(n, 'can_tx_fast', 1, 'Bus: Bus_Can1_EscFast');
outport(n, 'can_tx_slow', 2, 'Bus: Bus_Can1_EscSlow');
outport(n, 'truth', 3, 'Bus: Bus_DriveTruth');
set_param([n '/can_tx_fast'], 'SampleTime', 'Ts.can_drive');
set_param([n '/can_tx_slow'], 'SampleTime', 'Ts.can_fb2');
set_param([n '/truth'], 'SampleTime', 'Ts.base');
cport(n, 'DC48_TracL_p', 1, 'Left', 'E'); cport(n, 'DC48_TracL_n', 2, 'Left', 'E');
cport(n, 'DC48_TracR_p', 3, 'Left', 'E'); cport(n, 'DC48_TracR_n', 4, 'Left', 'E');
cport(n, 'Spr_L', 5, 'Right', 'M'); cport(n, 'Spr_R', 6, 'Right', 'M');
RT = 'simulink/Signal Attributes/Rate Transition';
A(RT, [n '/RT_T_amb'], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.base', ...
    'InitialCondition', 'P.drive.thermal.T_amb0');
L(n, 'T_amb/1', 'RT_T_amb/1');
sides = {'L', 1; 'R', 2};
for k = 1:2
    sd = sides{k, 1}; id = num2str(sides{k, 2});
    C(n, ['Node_' sd], ['P.drive.esc.node_id(' id ')'], 'uint8');
    A('simulink/Ports & Subsystems/Subsystem Reference', [n '/CmdUnpack_' sd], 'ReferencedSubsystem', 'drive_cmd_unpack');
    A(RT, [n '/RT_cmd_' sd], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.esc');
    A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Side_' sd], 'ReferencedSubsystem', 'drive_side');
    set_param([n '/Side_' sd], 'side', id);
    A(RT, [n '/RT_fb_' sd], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.can_drive');
    A('simulink/Ports & Subsystems/Subsystem Reference', [n '/FbPack_' sd], 'ReferencedSubsystem', 'drive_fb_pack');
    A(RT, [n '/RT_fb2_' sd], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.can_fb2');
    % the side's DRV_CMD frame picked by a Bus Selector right in front of its decoder (spec 6.2)
    SEL(n, ['CanSel_' sd], 'can_rx/1', {['drv_cmd_' sd]}, {['CmdUnpack_' sd '/1']});
    L(n, ['Node_' sd '/1'], ['CmdUnpack_' sd '/2']);
    L(n, ['CmdUnpack_' sd '/1'], ['RT_cmd_' sd '/1'], ['cmd_' sd]);
    L(n, ['RT_cmd_' sd '/1'], ['Side_' sd '/1'], ['cmd_' sd '_1ms']);
    L(n, 'sto_ok/1', ['Side_' sd '/2']); L(n, 'RT_T_amb/1', ['Side_' sd '/3'], 'T_amb_1ms');
    L(n, 'fault_inj/1', ['Side_' sd '/4']);
    L(n, ['Side_' sd '/1'], ['RT_fb_' sd '/1'], ['fb_' sd]);
    L(n, ['RT_fb_' sd '/1'], ['FbPack_' sd '/1'], ['fb_' sd '_10ms']); L(n, ['Node_' sd '/1'], ['FbPack_' sd '/2']);
    L(n, ['FbPack_' sd '/2'], ['RT_fb2_' sd '/1'], ['fb2_' sd '_10ms']);
    PL(n, RC(n, ['DC48_Trac' sd '_p']), CP([n '/Side_' sd], 'DC_p'));
    PL(n, RC(n, ['DC48_Trac' sd '_n']), CP([n '/Side_' sd], 'DC_n'));
    PL(n, CP([n '/Side_' sd], 'Spr'), RC(n, ['Spr_' sd]));
end
BC(n, 'EscFast', 'Bus_Can1_EscFast', 4); BC(n, 'EscSlow', 'Bus_Can1_EscSlow', 2); BC(n, 'Truth', 'Bus_DriveTruth', 2);
L(n, 'FbPack_L/1', 'EscFast/1', 'fb1_L'); L(n, 'FbPack_R/1', 'EscFast/2', 'fb1_R');
L(n, 'FbPack_L/3', 'EscFast/3', 'fb3_L'); L(n, 'FbPack_R/3', 'EscFast/4', 'fb3_R');
L(n, 'RT_fb2_L/1', 'EscSlow/1', 'fb2_L'); L(n, 'RT_fb2_R/1', 'EscSlow/2', 'fb2_R');
L(n, 'Side_L/2', 'Truth/1', 'L'); L(n, 'Side_R/2', 'Truth/2', 'R');
L(n, 'EscFast/1', 'can_tx_fast/1', 'can_tx_fast'); L(n, 'EscSlow/1', 'can_tx_slow/1', 'can_tx_slow');
L(n, 'Truth/1', 'truth/1', 'truth');
finish(n, f, ['Traction drive system (ICD 3.2 Drive row, build plan T06.1): DRV_CMD frames 0x201 / 0x202 decoded by ' ...
    'drive_cmd_unpack (invalid frame = hold last), Rate Transition 0.01 -> 0.001, drive_side L / R (mask side), feedback ' ...
    'Rate Transition 0.001 -> 0.01 and drive_fb_pack (FB1 + FB3 at 0.01 s, FB2 at 0.05 s). Physical ports DC48_TracL/R_p/n ' ...
    '(Left), Spr_L / Spr_R (Right). No Solver Configuration, no Electrical Reference (ICD 2). Owner drive (T06).']);
end
