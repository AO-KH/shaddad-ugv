function out = elec_harness(op, varargin)
%ELEC_HARNESS  Temporary unit harnesses for the T10 electronics plant / CAN / companion IO (t_elec_plant).
%
%   mdl = elec_harness('build', kind)      create (once per session, never saved) the harness model
%       kind 'plant'    root inputs (ICD 3.2 Electronics) -> Electronics_Plant -> 7 outputs; an independent
%                       elp_hw_frontend instance on the same inputs gives the front-end DI word (out 8)
%            'can1'     CAN1_Net_Sim, root inputs 1..8, outputs 1..6
%            'link'     Link_MCU_GNC, root inputs 1..7, outputs 1..5
%            'comp'     MCU-side packs (T08 can2_*_pack) -> Link_MCU_GNC -> companion_io (SIM) -> outputs 1..5;
%                       companion comp_cmd frame -> Link -> MCU-side can2_comp_cmd_unpack (out 6)
%            'lv'       elp_lv_demand with per-rail energy integrators (outputs 1 lv, 2 P_inbox, 3 E_rail[4] J)
%            'th_auto'  elp_thermal with the firmware fan law (1-D Lookup fan_bp -> fan_duty on both fans)
%            'th_off'   elp_thermal with the fans disabled (duty 0)
%            'hw'       companion_io alone; the model workspace defines VC_TARGET = 1 (HW choice, ROS 2 model)
%   o   = elec_harness('sim', kind, T, ds [, vars])   run; ds = Dataset of root inputs ([] = none);
%         vars = {name, value, ...} set as SimulationInput variables (base workspace untouched)
%   ds  = elec_harness('ds', {name, value}, ...)       Dataset of root inputs in port order
%   s   = elec_harness('bus', busName, t, ovr)         struct of timeseries for a bus port: every element
%         constant at P.ic.<bus>, fields of ovr (struct) override (value: N x dims array or scalar)
%   elec_harness('close')                              close every harness
%   All harnesses use ConfigSetRef Shaddad_Ref (fixed step 1 ms, multitasking); names tmp_T10_h_<kind>.

switch op
    case 'build'
        out = local_build(varargin{1});
    case 'sim'
        [kind, T, ds] = varargin{1:3};
        vars = {}; if numel(varargin) >= 4, vars = varargin{4}; end
        mpar = {}; if numel(varargin) >= 5, mpar = varargin{5}; end
        mdl = local_build(kind);
        in = Simulink.SimulationInput(mdl);
        if ~isempty(ds), in = in.setExternalInput(ds); end
        in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
            'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'off', 'SaveTime', 'off', ...
            'SaveState', 'off', 'LoadInitialState', 'off', mpar{:});
        for k = 1:2:numel(vars), in = in.setVariable(vars{k}, vars{k + 1}); end
        o = sim(in);
        if ~isempty(o.ErrorMessage), error('elec_harness:sim', '%s', o.ErrorMessage); end
        out = o;
    case 'ds'
        ds = Simulink.SimulationData.Dataset;
        for k = 1:numel(varargin), ds = ds.addElement(varargin{k}{2}, varargin{k}{1}); end
        out = ds;
    case 'bus'
        out = local_bus(varargin{:});
    case 'close'
        for k = {'plant', 'can1', 'link', 'comp', 'lv', 'th_auto', 'th_off', 'hw', 'loopft'}
            m = ['tmp_T10_h_' k{1}];
            if bdIsLoaded(m), close_system(m, 0); end
        end
        out = [];
    otherwise
        error('elec_harness:op', 'unknown op %s', op);
end
end

% ===========================================================================================
function s = local_bus(busName, t, ovr)
if nargin < 3, ovr = struct(); end
P = evalin('base', 'P');
B = evalin('base', busName);
ic = P.ic.(busName);
t = t(:);
N = numel(t);
s = struct();
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    if startsWith(e.DataType, 'Bus:')
        sub = strtrim(extractAfter(e.DataType, 'Bus:'));
        o2 = struct(); if isfield(ovr, e.Name), o2 = ovr.(e.Name); end
        s.(e.Name) = local_bus(sub, t, o2);
        continue
    end
    if isfield(ovr, e.Name)
        v = ovr.(e.Name);
        if size(v, 1) == 1 && N > 1, v = repmat(v, N, 1); end
    else
        v = repmat(ic.(e.Name)(:)', N, 1);
    end
    switch e.DataType
        case 'boolean', v = logical(v);
        case 'double',  v = double(v);
        otherwise,      v = cast(v, e.DataType);
    end
    s.(e.Name) = timeseries(v, t);
end
end

function mdl = local_build(kind)
mdl = ['tmp_T10_h_' kind];
if bdIsLoaded(mdl), return, end
new_system(mdl);
if any(strcmp(kind, {'lv', 'th_auto', 'th_off'}))
    % long runs (hours): own fixed-step discrete configuration at the fastest rate of the unit (0.01 s)
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', ...
        'EnableMultiTasking', 'on', 'StartTime', '0');
else
    shaddad_configsets('attach', mdl);
end
feval(['local_' kind], mdl);
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch %#ok<CTCH>
end
end

function local_plant(m)
ins = {'mcu_out1k', 'Bus_McuOut1k', 'Ts.base'; 'mcu_out10', 'Bus_McuOut10', 'Ts.ctrl'; ...
    'power_status', 'Bus_PowerStatus', 'Ts.base'; 'hmi', 'Bus_HmiInputs', 'Ts.base'; 'env', 'Bus_EnvState', 'Ts.hk'; ...
    'impl_mech', 'Bus_ImplMechState', 'Ts.ctrl'; 'flt', 'Bus_FaultInject', 'Ts.base'};
for k = 1:size(ins, 1), IN(m, ins{k, 1}, k, ['Bus: ' ins{k, 2}], ins{k, 3}); end
SSR(m, 'EP', 'Electronics_Plant');
for k = 1:size(ins, 1), L(m, [ins{k, 1} '/1'], sprintf('EP/%d', k)); end
outs = {'elec_hw', 'elec_adc', 'elec_env', 'hw_cmd', 'impl_cmd', 'lv_demand', 'indicators'};
for k = 1:numel(outs)
    A(m, 'simulink/Sinks/Out1', outs{k}, 'Port', num2str(k));
    L(m, sprintf('EP/%d', k), [outs{k} '/1']);
end
SSR(m, 'FE', 'elp_hw_frontend');
L(m, 'power_status/1', 'FE/1'); L(m, 'hmi/1', 'FE/2'); L(m, 'impl_mech/1', 'FE/3');
A(m, 'simulink/Sources/Constant', 'T_fe', 'Value', 'P.electronics.therm.T0', 'SampleTime', 'Ts.hk');
L(m, 'T_fe/1', 'FE/4');
A(m, 'simulink/Sinks/Out1', 'fe_di_word', 'Port', '8'); L(m, 'FE/1', 'fe_di_word/1');
A(m, 'simulink/Sinks/Terminator', 'T_fe_adc'); L(m, 'FE/2', 'T_fe_adc/1');
end

function local_can1(m)
ins = {'mcu_fast', 'Bus_Can1_McuFast', 'Ts.ctrl'; 'mcu_slow', 'Bus_Can1_McuSlow', 'Ts.comm'; ...
    'esc_fast', 'Bus_Can1_EscFast', 'Ts.ctrl'; 'esc_slow', 'Bus_Can1_EscSlow', 'Ts.comm'; ...
    'impl', 'Bus_Can1_Impl', 'Ts.comm'; 'bms', 'Bus_Can1_Bms', 'Ts.hk'; 'flt', 'Bus_FaultInject', 'Ts.ctrl'; ...
    'elec_hw', 'Bus_ElecHwIn', 'Ts.base'};
for k = 1:size(ins, 1), IN(m, ins{k, 1}, k, ['Bus: ' ins{k, 2}], ins{k, 3}); end
SSR(m, 'CAN1', 'CAN1_Net_Sim');
for k = 1:size(ins, 1), L(m, [ins{k, 1} '/1'], sprintf('CAN1/%d', k)); end
for k = 1:6
    A(m, 'simulink/Sinks/Out1', [ins{k, 1} '_rx'], 'Port', num2str(k));
    L(m, sprintf('CAN1/%d', k), [ins{k, 1} '_rx/1']);
end
end

function local_link(m)
ins = {'mcu_imu', 'Bus_CanFrame64', 'Ts.imu'; 'mcu_odom', 'Bus_CanFrame64', 'Ts.ctrl'; ...
    'mcu_status', 'Bus_CanFrame64', 'Ts.comm'; 'mcu_power', 'Bus_CanFrame64', 'Ts.hk'; ...
    'comp_cmd', 'Bus_CanFrame64', 'Ts.comm'; 'flt', 'Bus_FaultInject', 'Ts.ctrl'; 'elec_hw', 'Bus_ElecHwIn', 'Ts.base'};
for k = 1:size(ins, 1), IN(m, ins{k, 1}, k, ['Bus: ' ins{k, 2}], ins{k, 3}); end
SSR(m, 'Link', 'Link_MCU_GNC');
for k = 1:size(ins, 1), L(m, [ins{k, 1} '/1'], sprintf('Link/%d', k)); end
for k = 1:5
    A(m, 'simulink/Sinks/Out1', [ins{k, 1} '_rx'], 'Port', num2str(k));
    L(m, sprintf('Link/%d', k), [ins{k, 1} '_rx/1']);
end
end

function local_comp(m)
% inputs: 1 x_imu Bus_ImuMeas 0.005, 2 x_odom Bus_OdomMeas 0.01, 3 x_status Bus_McuStatus 0.05, 4 t_ms uint32
% 0.05, 5 x_power Bus_BmsStatus 0.1, 6 gnc_cmd Bus_GncCmd 0.05, 7 flt Bus_FaultInject 0.01, 8 elec_hw 0.001
ins = {'x_imu', 'Bus: Bus_ImuMeas', 'Ts.imu'; 'x_odom', 'Bus: Bus_OdomMeas', 'Ts.ctrl'; ...
    'x_status', 'Bus: Bus_McuStatus', 'Ts.comm'; 't_ms', 'uint32', 'Ts.comm'; 'x_power', 'Bus: Bus_BmsStatus', 'Ts.hk'; ...
    'gnc_cmd', 'Bus: Bus_GncCmd', 'Ts.comm'; 'flt', 'Bus: Bus_FaultInject', 'Ts.ctrl'; 'elec_hw', 'Bus: Bus_ElecHwIn', 'Ts.base'};
for k = 1:size(ins, 1), IN(m, ins{k, 1}, k, ins{k, 2}, ins{k, 3}); end
set_param([m '/t_ms'], 'PortDimensions', '1');
SSR(m, 'PackImu', 'can2_mcu_imu_pack');       L(m, 'x_imu/1', 'PackImu/1');
SSR(m, 'PackOdom', 'can2_mcu_odom_pack');     L(m, 'x_odom/1', 'PackOdom/1');
SSR(m, 'PackStatus', 'can2_mcu_status_pack'); L(m, 'x_status/1', 'PackStatus/1'); L(m, 't_ms/1', 'PackStatus/2');
SSR(m, 'PackPower', 'can2_mcu_power_pack');   L(m, 'x_power/1', 'PackPower/1');
SSR(m, 'Link', 'Link_MCU_GNC');
L(m, 'PackImu/1', 'Link/1'); L(m, 'PackOdom/1', 'Link/2'); L(m, 'PackStatus/1', 'Link/3'); L(m, 'PackPower/1', 'Link/4');
L(m, 'flt/1', 'Link/6'); L(m, 'elec_hw/1', 'Link/7');
SSR(m, 'CompIO', 'companion_io');
for k = 1:4, L(m, sprintf('Link/%d', k), sprintf('CompIO/%d', k)); end
L(m, 'gnc_cmd/1', 'CompIO/5');
L(m, 'CompIO/5', 'Link/5');
SSR(m, 'UnpackCmd', 'can2_comp_cmd_unpack');
L(m, 'Link/5', 'UnpackCmd/1');
outs = {'imu_meas', 'CompIO/1'; 'odom_meas', 'CompIO/2'; 'mcu_status', 'CompIO/3'; 'bms_status', 'CompIO/4'; ...
    'comp_frame', 'CompIO/5'; 'gnc_mcu', 'UnpackCmd/1'};
for k = 1:size(outs, 1)
    A(m, 'simulink/Sinks/Out1', outs{k, 1}, 'Port', num2str(k)); L(m, outs{k, 2}, [outs{k, 1} '/1']);
end
end

function local_loopft(m)
% Electronics_Plant + CAN1_Net_Sim + Link_MCU_GNC + companion_io + stub GNC closed through a worst-case
% direct-feedthrough MCU emulation (MCU_FT): out1k = f(in1k) (do_word = di_word, wdg_kick = NOT reset_flag),
% out10 frames = f(ESC frames) and PWM = f(ADC), out50 MCU_STATUS frame = the received COMP_CMD frame.
% Only the T10 loop breakers (DI_Latency, thermal output delay, MCU_STATUS delay) can break the loops.
SSR(m, 'EP', 'Electronics_Plant'); SSR(m, 'CAN1', 'CAN1_Net_Sim'); SSR(m, 'Link', 'Link_MCU_GNC');
SSR(m, 'Comp', 'companion_io');
cst = {'PS', 'P.ic.Bus_PowerStatus', 'Bus_PowerStatus', 'Ts.base'; 'Hmi', 'P.ic.Bus_HmiInputs', 'Bus_HmiInputs', 'Ts.base'; ...
    'Env', 'P.ic.Bus_EnvState', 'Bus_EnvState', 'Ts.hk'; 'Impl', 'P.ic.Bus_ImplMechState', 'Bus_ImplMechState', 'Ts.ctrl'; ...
    'Flt1k', 'P.ic.Bus_FaultInject', 'Bus_FaultInject', 'Ts.base'; 'Flt10', 'P.ic.Bus_FaultInject', 'Bus_FaultInject', 'Ts.ctrl'; ...
    'EscFast', 'P.ic.Bus_Can1_EscFast', 'Bus_Can1_EscFast', 'Ts.ctrl'; 'EscSlow', 'P.ic.Bus_Can1_EscSlow', 'Bus_Can1_EscSlow', 'Ts.comm'; ...
    'ImplFb', 'P.ic.Bus_Can1_Impl', 'Bus_Can1_Impl', 'Ts.comm'; 'Bms', 'P.ic.Bus_Can1_Bms', 'Bus_Can1_Bms', 'Ts.hk'};
for k = 1:size(cst, 1)
    A(m, 'simulink/Sources/Constant', cst{k, 1}, 'Value', cst{k, 2}, 'OutDataTypeStr', ['Bus: ' cst{k, 3}], 'SampleTime', cst{k, 4});
end
% ---- MCU_FT: in 1 elec_hw, 2 elec_adc, 3 esc_fast_rx, 4 comp_rx; out 1 out1k, 2 out5, 3 out10, 4 out50, 5 out100 ----
s = [m '/MCU_FT'];
add_block('built-in/SubSystem', s);
for k = 1:4, A(s, 'simulink/Sources/In1', sprintf('i%d', k), 'Port', num2str(k)); end
on = {'out1k', 'McuOut1k'; 'out5', 'McuOut5'; 'out10', 'McuOut10'; 'out50', 'McuOut50'; 'out100', 'McuOut100'};
for k = 1:5, A(s, 'simulink/Sinks/Out1', on{k, 1}, 'Port', num2str(k)); end
A(s, 'simulink/Signal Routing/Bus Selector', 'SelHw', 'OutputSignals', 'di_word,reset_flag'); L(s, 'i1/1', 'SelHw/1');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'DoWord', 'OutDataTypeStr', 'uint32'); L(s, 'SelHw/1', 'DoWord/1');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Kick', 'Operator', 'NOT', 'Inputs', '1'); L(s, 'SelHw/2', 'Kick/1');
BCB(s, 'BC1k', 'Bus_McuOut1k', {'DoWord/1', 'Kick/1'}); L(s, 'BC1k/1', 'out1k/1');
A(s, 'simulink/Sources/Constant', 'Imu', 'Value', 'P.ic.Bus_CanFrame64', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', 'SampleTime', 'Ts.imu');
BCB(s, 'BC5', 'Bus_McuOut5', {'Imu/1'}); L(s, 'BC5/1', 'out5/1');
A(s, 'simulink/Signal Routing/Bus Selector', 'SelEsc', 'OutputSignals', 'fb1_L,fb1_R'); L(s, 'i3/1', 'SelEsc/1');
BCB(s, 'Drv', 'Bus_Can1_McuFast', {'SelEsc/1', 'SelEsc/2'});
A(s, 'simulink/Signal Routing/Bus Selector', 'SelV', 'OutputSignals', 'valid'); L(s, 'SelEsc/1', 'SelV/1');
A(s, 'simulink/Sources/Constant', 'Odom0', 'Value', 'P.ic.Bus_CanFrame64', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', 'SampleTime', 'Ts.ctrl');
A(s, 'simulink/Signal Routing/Bus Assignment', 'Odom', 'AssignedSignals', 'valid');
L(s, 'Odom0/1', 'Odom/1'); L(s, 'SelV/1', 'Odom/2');
A(s, 'simulink/Signal Routing/Bus Selector', 'SelAdc', 'OutputSignals', 'adc_raw'); L(s, 'i2/1', 'SelAdc/1');
A(s, 'simulink/Signal Routing/Selector', 'Adc4', 'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '1:4', 'InputPortWidth', '8'); L(s, 'SelAdc/1', 'Adc4/1');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'AdcS', 'OutDataTypeStr', 'single'); L(s, 'Adc4/1', 'AdcS/1');
A(s, 'simulink/Math Operations/Gain', 'Duty', 'Gain', '1/65535', 'OutDataTypeStr', 'single'); L(s, 'AdcS/1', 'Duty/1');
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_pwm', 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', 'Ts.ctrl', 'InitialCondition', '0'); L(s, 'Duty/1', 'RT_pwm/1');
BCB(s, 'BC10', 'Bus_McuOut10', {'Drv/1', 'Odom/1', 'RT_pwm/1'}); L(s, 'BC10/1', 'out10/1');
A(s, 'simulink/Sources/Constant', 'Slow', 'Value', 'P.ic.Bus_Can1_McuSlow', 'OutDataTypeStr', 'Bus: Bus_Can1_McuSlow', 'SampleTime', 'Ts.comm');
BCB(s, 'BC50', 'Bus_McuOut50', {'Slow/1', 'i4/1'}); L(s, 'BC50/1', 'out50/1');
A(s, 'simulink/Sources/Constant', 'Pwr', 'Value', 'P.ic.Bus_CanFrame64', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', 'SampleTime', 'Ts.hk');
BCB(s, 'BC100', 'Bus_McuOut100', {'Pwr/1'}); L(s, 'BC100/1', 'out100/1');
% ---- stub GNC: seq = MCU_STATUS heartbeat ----
g = [m '/GNC'];
add_block('built-in/SubSystem', g);
A(g, 'simulink/Sources/In1', 'st', 'Port', '1'); A(g, 'simulink/Sinks/Out1', 'cmd', 'Port', '1');
A(g, 'simulink/Sources/Constant', 'C0', 'Value', 'P.ic.Bus_GncCmd', 'OutDataTypeStr', 'Bus: Bus_GncCmd', 'SampleTime', 'Ts.comm');
A(g, 'simulink/Signal Routing/Bus Selector', 'Hb', 'OutputSignals', 'heartbeat'); L(g, 'st/1', 'Hb/1');
A(g, 'simulink/Signal Routing/Bus Assignment', 'Cmd', 'AssignedSignals', 'seq');
L(g, 'C0/1', 'Cmd/1'); L(g, 'Hb/1', 'Cmd/2'); L(g, 'Cmd/1', 'cmd/1');
% ---- wiring ----
A(m, 'simulink/Signal Routing/Bus Selector', 'S10', 'OutputSignals', 'can1_drv,can2_odom'); L(m, 'MCU_FT/3', 'S10/1');
A(m, 'simulink/Signal Routing/Bus Selector', 'S50', 'OutputSignals', 'can1_slow,can2_status'); L(m, 'MCU_FT/4', 'S50/1');
A(m, 'simulink/Signal Routing/Bus Selector', 'S5', 'OutputSignals', 'can2_imu'); L(m, 'MCU_FT/2', 'S5/1');
A(m, 'simulink/Signal Routing/Bus Selector', 'S100', 'OutputSignals', 'can2_power'); L(m, 'MCU_FT/5', 'S100/1');
L(m, 'MCU_FT/1', 'EP/1'); L(m, 'MCU_FT/3', 'EP/2'); L(m, 'PS/1', 'EP/3'); L(m, 'Hmi/1', 'EP/4'); L(m, 'Env/1', 'EP/5');
L(m, 'Impl/1', 'EP/6'); L(m, 'Flt1k/1', 'EP/7');
L(m, 'S10/1', 'CAN1/1'); L(m, 'S50/1', 'CAN1/2'); L(m, 'EscFast/1', 'CAN1/3'); L(m, 'EscSlow/1', 'CAN1/4');
L(m, 'ImplFb/1', 'CAN1/5'); L(m, 'Bms/1', 'CAN1/6'); L(m, 'Flt10/1', 'CAN1/7'); L(m, 'EP/1', 'CAN1/8');
L(m, 'S5/1', 'Link/1'); L(m, 'S10/2', 'Link/2'); L(m, 'S50/2', 'Link/3'); L(m, 'S100/1', 'Link/4');
L(m, 'Comp/5', 'Link/5'); L(m, 'Flt10/1', 'Link/6'); L(m, 'EP/1', 'Link/7');
for k = 1:4, L(m, sprintf('Link/%d', k), sprintf('Comp/%d', k)); end
L(m, 'Comp/3', 'GNC/1'); L(m, 'GNC/1', 'Comp/5');
L(m, 'EP/1', 'MCU_FT/1'); L(m, 'EP/2', 'MCU_FT/2'); L(m, 'CAN1/3', 'MCU_FT/3'); L(m, 'Link/5', 'MCU_FT/4');
term = {'EP/3', 'EP/4', 'EP/5', 'EP/6', 'EP/7', 'CAN1/1', 'CAN1/2', 'CAN1/4', 'CAN1/5', 'CAN1/6', 'Comp/1', 'Comp/2', 'Comp/4'};
for k = 1:numel(term)
    A(m, 'simulink/Sinks/Terminator', sprintf('T%d', k)); L(m, term{k}, sprintf('T%d/1', k));
end
A(m, 'simulink/Sinks/Out1', 'mcu_status', 'Port', '1'); L(m, 'Comp/3', 'mcu_status/1');
A(m, 'simulink/Sinks/Out1', 'elec_hw', 'Port', '2'); L(m, 'EP/1', 'elec_hw/1');
end

function BCB(sys, name, bus, srcs)
A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(srcs)), 'OutDataTypeStr', ['Bus: ' bus], ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(srcs), L(sys, srcs{k}, sprintf('%s/%d', name, k)); end
end

function local_lv(m)
IN(m, 'do_bits', 1, 'boolean', 'Ts.ctrl', '32');
IN(m, 'pwm', 2, 'double', 'Ts.ctrl', '4');
set_param([m '/do_bits'], 'Interpolate', 'off'); set_param([m '/pwm'], 'Interpolate', 'off');
SSR(m, 'LV', 'elp_lv_demand');
L(m, 'do_bits/1', 'LV/1'); L(m, 'pwm/1', 'LV/2');
A(m, 'simulink/Sinks/Out1', 'lv', 'Port', '1'); L(m, 'LV/1', 'lv/1');
A(m, 'simulink/Sinks/Out1', 'P_inbox', 'Port', '2'); L(m, 'LV/2', 'P_inbox/1');
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel', 'OutputSignals', 'P_24V_W,P_12V_W,P_5V_W,P_3V3_W');
L(m, 'LV/1', 'Sel/1');
A(m, 'simulink/Signal Routing/Vector Concatenate', 'Cat', 'NumInputs', '4');
for k = 1:4, L(m, sprintf('Sel/%d', k), sprintf('Cat/%d', k)); end
A(m, 'simulink/Discrete/Discrete-Time Integrator', 'E', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1.0', 'InitialCondition', '0', 'SampleTime', 'Ts.ctrl');
L(m, 'Cat/1', 'E/1');
A(m, 'simulink/Sinks/Out1', 'E_rail', 'Port', '3'); L(m, 'E/1', 'E_rail/1');
end

function local_th_auto(m), local_th(m, true); end
function local_th_off(m), local_th(m, false); end

function local_th(m, auto)
IN(m, 'P_inbox', 1, 'double', 'Ts.ctrl');
IN(m, 'env', 2, 'Bus: Bus_EnvState', 'Ts.hk');
SSR(m, 'TH', 'elp_thermal');
L(m, 'P_inbox/1', 'TH/1'); L(m, 'env/1', 'TH/3');
if auto
    A(m, 'simulink/Lookup Tables/1-D Lookup Table', 'FanLaw', 'BreakpointsForDimension1', 'P.electronics.aux.fan_bp', ...
        'Table', 'P.electronics.aux.fan_duty', 'ExtrapMethod', 'Clip');
    L(m, 'TH/1', 'FanLaw/1');
    A(m, 'simulink/Signal Attributes/Rate Transition', 'RT', 'Integrity', 'on', 'Deterministic', 'on', ...
        'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', 'Ts.ctrl', 'InitialCondition', '0');
    L(m, 'FanLaw/1', 'RT/1');
    A(m, 'simulink/Sources/Constant', 'Z2', 'Value', '[0 0]', 'SampleTime', 'Ts.ctrl');
    A(m, 'simulink/Signal Routing/Vector Concatenate', 'Pwm', 'NumInputs', '3');
    L(m, 'Z2/1', 'Pwm/1'); L(m, 'RT/1', 'Pwm/2'); L(m, 'RT/1', 'Pwm/3');
    L(m, 'Pwm/1', 'TH/2');
    A(m, 'simulink/Sinks/Out1', 'fan_duty', 'Port', '2'); L(m, 'FanLaw/1', 'fan_duty/1');
else
    A(m, 'simulink/Sources/Constant', 'Pwm', 'Value', 'zeros(1, 4)', 'SampleTime', 'Ts.ctrl');
    L(m, 'Pwm/1', 'TH/2');
end
A(m, 'simulink/Sinks/Out1', 'T_encl', 'Port', '1'); L(m, 'TH/1', 'T_encl/1');
end

function local_hw(m)
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'VC_TARGET', 1);
ins = {'mcu_imu', 'Bus_CanFrame64', 'Ts.imu'; 'mcu_odom', 'Bus_CanFrame64', 'Ts.ctrl'; ...
    'mcu_status', 'Bus_CanFrame64', 'Ts.comm'; 'mcu_power', 'Bus_CanFrame64', 'Ts.hk'; 'gnc_cmd', 'Bus_GncCmd', 'Ts.comm'};
for k = 1:size(ins, 1), IN(m, ins{k, 1}, k, ['Bus: ' ins{k, 2}], ins{k, 3}); end
SSR(m, 'CompIO', 'companion_io');
for k = 1:5, L(m, [ins{k, 1} '/1'], sprintf('CompIO/%d', k)); end
for k = 1:5
    A(m, 'simulink/Sinks/Out1', sprintf('o%d', k), 'Port', num2str(k)); L(m, sprintf('CompIO/%d', k), sprintf('o%d/1', k));
end
end

% ---- helpers ------------------------------------------------------------------------------
function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function L(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'smart');
end

function IN(m, name, port, type, ts, dims)
p = A(m, 'simulink/Sources/In1', name, 'Port', num2str(port), 'OutDataTypeStr', type, 'SampleTime', ts);
if nargin >= 6 && ~isempty(dims), set_param(p, 'PortDimensions', dims); end
end

function SSR(m, name, file)
A(m, 'simulink/Ports & Subsystems/Subsystem Reference', name, 'ReferencedSubsystem', file);
end
