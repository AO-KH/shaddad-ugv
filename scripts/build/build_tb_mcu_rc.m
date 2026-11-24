function build_tb_mcu_rc()
%BUILD_TB_MCU_RC  Bench tests/electronics/tb_mcu_rc.slx: RC_Link_Sim -> Shaddad_MCU (owner: electronics, T08).
%
%   Root inputs: 1 op Bus_OperatorCmd (Ts.ctrl), 2 flt Bus_FaultInject (Ts.ctrl) - external inputs
%   (tests/rc/rc_tb_input). Blocks: RC_Link (SSR RC_Link_Sim, T07) -> MCU_In (Bus Creators of
%   Bus_McuIn1k/5/10/50/100: SBUS stream, constant healthy DI word, t_us clock, fault_inject word as in
%   Shaddad_System/MCU/MCU_In, level IMU, CAN frame slots P.ic.* (no frames), constant ADC counts) ->
%   MCU (Model Shaddad_MCU, Normal). Root outputs (yout): 1 sbus, 2 rc_frame (0.001), 3 out1k (0.001),
%   4 out10 (0.01), 5 telem (0.01). Signals logged inside Shaddad_MCU (logsout): rc_dec (0.001),
%   rc (0.01), fw10 (0.01), tilt (0.005), busoff (0.01). Config Shaddad_Ref (cs_Shaddad).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out the root and MCU_In with
%   shaddad_layout('tb_mcu_rc', 'Recursive', true). Every signal line is named (L5): a line keeps the first name a
%   call gives it (the lines into the bus-typed Bus Creators are named after their elements), and local_name_rest
%   names every line still without a name after its source block ('_k' for output k of a block with several outputs,
%   as shaddad_bu('nline')); a Bus Selector's output keeps '<element>'.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
mdl = 'tb_mcu_rc';
f = fullfile(root, 'tests', 'electronics', [mdl '.slx']);
if ~isfolder(fileparts(f)), mkdir(fileparts(f)); end
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
A(mdl, 'simulink/Sources/In1', 'op', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_OperatorCmd', 'SampleTime', 'Ts.ctrl');
A(mdl, 'simulink/Sources/In1', 'flt', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_FaultInject', 'SampleTime', 'Ts.ctrl');
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Link', 'ReferencedSubsystem', 'RC_Link_Sim');
L(mdl, 'op/1', 'RC_Link/1', 'op'); L(mdl, 'flt/1', 'RC_Link/2', 'flt');
% ---- MCU_In ----
s = [mdl '/MCU_In'];
add_block('built-in/SubSystem', s);
A(s, 'simulink/Sources/In1', 'sbus', 'Port', '1');
A(s, 'simulink/Sources/In1', 'flt', 'Port', '2');
outs = {'in1k', 'McuIn1k', 5; 'in5', 'McuIn5', 1; 'in10', 'McuIn10', 1; 'in50', 'McuIn50', 5; 'in100', 'McuIn100', 1};
for k = 1:size(outs, 1)
    A(s, 'simulink/Sinks/Out1', outs{k, 1}, 'Port', num2str(k));
    A(s, 'simulink/Signal Routing/Bus Creator', ['BC_' outs{k, 1}], 'Inputs', num2str(outs{k, 3}), ...
        'OutDataTypeStr', ['Bus: Bus_' outs{k, 2}], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
    L(s, ['BC_' outs{k, 1} '/1'], [outs{k, 1} '/1'], outs{k, 1});
end
L(s, 'sbus/1', 'BC_in1k/1', 'sbus');
% healthy DI word: E-stop loops closed, safety relay closed, lid closed, key on, HV safety OK
CST(s, 'DI_word', ['2^P.electronics.di.estop_ch1_ok + 2^P.electronics.di.estop_ch2_ok + 2^P.electronics.di.safety_ok' ...
    ' + 2^P.electronics.di.lid_closed + 2^P.electronics.di.key_on + 2^P.electronics.di.safety_hv_ok'], 'uint16', 'Ts.base');
L(s, 'DI_word/1', 'BC_in1k/2', 'di_word');
A(s, 'simulink/Sources/Digital Clock', 'Clock_t', 'SampleTime', 'Ts.base');
A(s, 'simulink/Math Operations/Gain', 'Gain_us', 'Gain', '1e6');
A(s, 'simulink/Math Operations/Rounding Function', 'Round_us', 'Operator', 'round');
A(s, 'simulink/Math Operations/Math Function', 'Mod_2p32', 'Operator', 'mod');
CST(s, 'C_2p32', '2^32', 'double', 'Ts.base');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'DTC_t_us', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'off');
L(s, 'Clock_t/1', 'Gain_us/1'); L(s, 'Gain_us/1', 'Round_us/1'); L(s, 'Round_us/1', 'Mod_2p32/1');
L(s, 'C_2p32/1', 'Mod_2p32/2'); L(s, 'Mod_2p32/1', 'DTC_t_us/1'); L(s, 'DTC_t_us/1', 'BC_in1k/3', 't_us');
CST(s, 'Reset_flag', 'false', 'boolean', 'Ts.base');
L(s, 'Reset_flag/1', 'BC_in1k/4', 'reset_flag');
% fault_inject = task_stall | can1_busoff << 4 | can2_busoff << 5 | (imu_fault == 3) << 6 (0.01 -> 0.001); one pick of
% flt right in front of each consumer (spec 6.2)
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'DTC_stall', 'OutDataTypeStr', 'uint32');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Mask_stall', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '15', 'BitMaskRealWorld', 'Stored Integer');
PICK(s, 'flt/1', 'flt', 'Sel_flt_stall', {'task_stall'}, {'DTC_stall/1'}); L(s, 'DTC_stall/1', 'Mask_stall/1');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'IMU_dropout', 'relop', '==', 'const', '3', 'OutDataTypeStr', 'boolean');
PICK(s, 'flt/1', 'flt', 'Sel_flt_imu', {'imu_fault'}, {'IMU_dropout/1'});
sh = {'can1_busoff', 'Sel_flt_can1', 4; 'can2_busoff', 'Sel_flt_can2', 5; 'imu_dropout', '', 6};
for k = 1:size(sh, 1)
    A(s, 'simulink/Signal Attributes/Data Type Conversion', ['DTC_' sh{k, 1}], 'OutDataTypeStr', 'uint32');
    A(s, 'simulink/Logic and Bit Operations/Shift Arithmetic', ['Shift_' sh{k, 1}], 'BitShiftNumber', num2str(sh{k, 3}), ...
        'BitShiftDirection', 'Left');
    if isempty(sh{k, 2})
        L(s, 'IMU_dropout/1', ['DTC_' sh{k, 1} '/1'], sh{k, 1});
    else
        PICK(s, 'flt/1', 'flt', sh{k, 2}, sh(k, 1), {['DTC_' sh{k, 1} '/1']});
    end
    L(s, ['DTC_' sh{k, 1} '/1'], ['Shift_' sh{k, 1} '/1']);
end
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'OR_fi', 'logicop', 'OR', 'UseBitMask', 'off', 'NumInputPorts', '4');
L(s, 'Mask_stall/1', 'OR_fi/1'); L(s, 'Shift_can1_busoff/1', 'OR_fi/2'); L(s, 'Shift_can2_busoff/1', 'OR_fi/3');
L(s, 'Shift_imu_dropout/1', 'OR_fi/4');
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_fi', 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', 'Ts.base', 'InitialCondition', '0');
L(s, 'OR_fi/1', 'RT_fi/1', 'fault_inject'); L(s, 'RT_fi/1', 'BC_in1k/5', 'fault_inject');
% level IMU at rest (physical specific force +g on z)
A(s, 'simulink/Signal Routing/Bus Creator', 'BC_imu', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_ImuRaw', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
CST(s, 'Acc', '[0 0 9.80665]', 'single', 'Ts.imu'); CST(s, 'Gyr', '[0 0 0]', 'single', 'Ts.imu');
CST(s, 'Mag', '[20 0 -40]', 'single', 'Ts.imu'); CST(s, 'Temp', '35', 'single', 'Ts.imu');
L(s, 'Acc/1', 'BC_imu/1', 'Acceleration'); L(s, 'Gyr/1', 'BC_imu/2', 'AngularRate');
L(s, 'Mag/1', 'BC_imu/3', 'MagneticField'); L(s, 'Temp/1', 'BC_imu/4', 'Temperature');
L(s, 'BC_imu/1', 'BC_in5/1', 'imu');
CST(s, 'Esc_fast', 'P.ic.Bus_Can1_EscFast', 'Bus: Bus_Can1_EscFast', 'Ts.ctrl');
L(s, 'Esc_fast/1', 'BC_in10/1', 'can1_esc');
CST(s, 'Esc_slow', 'P.ic.Bus_Can1_EscSlow', 'Bus: Bus_Can1_EscSlow', 'Ts.comm');
CST(s, 'Impl', 'P.ic.Bus_Can1_Impl', 'Bus: Bus_Can1_Impl', 'Ts.comm');
CST(s, 'Comp', 'P.ic.Bus_CanFrame64', 'Bus: Bus_CanFrame64', 'Ts.comm');
% ADC counts for 51.2 V pack / bus, 0 A, 24 / 12 / 5 V rails, NTC at 318 K, lift pot 0.5
CST(s, 'Adc', ['round([51.2/P.electronics.adc.k_vpack 51.2/P.electronics.adc.k_vlink P.electronics.adc.i_off ' ...
    '24/P.electronics.adc.k_v24 12/P.electronics.adc.k_v12 5/P.electronics.adc.k_v5 0 0.5*P.electronics.adc.vref]' ...
    '/P.electronics.adc.vref*65535) + [0 0 0 0 0 0 round(interp1(P.electronics.adc.ntc_lut_K, ' ...
    'P.electronics.adc.ntc_lut_counts, 318.15)) 0]'], 'uint16', 'Ts.comm');
CST(s, 'T_mcu', '40', 'single', 'Ts.comm');
L(s, 'Esc_slow/1', 'BC_in50/1', 'can1_escslow'); L(s, 'Impl/1', 'BC_in50/2', 'can1_impl');
L(s, 'Comp/1', 'BC_in50/3', 'can2_comp'); L(s, 'Adc/1', 'BC_in50/4', 'adc_raw'); L(s, 'T_mcu/1', 'BC_in50/5', 'mcu_temp_degC');
CST(s, 'Bms', 'P.ic.Bus_Can1_Bms', 'Bus: Bus_Can1_Bms', 'Ts.hk');
L(s, 'Bms/1', 'BC_in100/1', 'can1_bms');
L(mdl, 'RC_Link/1', 'MCU_In/1', 'sbus'); L(mdl, 'flt/1', 'MCU_In/2');
% ---- MCU ----
A(mdl, 'simulink/Ports & Subsystems/Model', 'MCU', 'ModelName', 'Shaddad_MCU', 'SimulationMode', 'Normal');
for k = 1:5, L(mdl, sprintf('MCU_In/%d', k), sprintf('MCU/%d', k), outs{k, 1}); end
% ---- outputs ----
ro = {'sbus', 'RC_Link/1', 'SbusStream'; 'rc_frame', 'RC_Link/2', 'RcFrame'; 'out1k', 'MCU/1', 'McuOut1k'; ...
    'out10', 'MCU/3', 'McuOut10'; 'telem', 'MCU/6', 'FwTelemetry'; 'out50', 'MCU/4', 'McuOut50'};
for k = 1:size(ro, 1)
    A(mdl, 'simulink/Sinks/Out1', ro{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ['Bus: Bus_' ro{k, 3}]);
    L(mdl, ro{k, 2}, [ro{k, 1} '/1'], ro{k, 1});
end
for tk = {2, 'out5'; 5, 'out100'}'
    k = tk{1};
    A(mdl, 'simulink/Sinks/Terminator', sprintf('T_mcu%d', k)); L(mdl, sprintf('MCU/%d', k), sprintf('T_mcu%d/1', k), tk{2});
end
set_param(mdl, 'Description', ['tb_mcu_rc (T08): RC_Link_Sim -> MCU_In -> Shaddad_MCU. in 1 op Bus_OperatorCmd, 2 flt ' ...
    'Bus_FaultInject (0.01, external inputs). out 1 sbus, 2 rc_frame (0.001), 3 out1k, 4 out10 (0.01), 5 telem, ' ...
    '6 out50 (0.05). ' ...
    'Built by scripts/build/build_tb_mcu_rc.m; run with tests/electronics/mcu_tb_run.m.']);
local_name_rest(mdl);
shaddad_bu('save', mdl, f);
shaddad_bu('finalize', mdl, f);
close_system(mdl, 0);
end

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

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function PICK(sys, src, nm, blk, sigs, dsts)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line of src ('Blk/k'),
% which L names nm when it has no name yet, and feeds its output k to dsts{k} ('Blk/j')
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end

function local_name_rest(mdl)
% L5 (layout spec 6.1): every signal line of every level of the bench (shaddad_layout_diagrams) that no call named
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
