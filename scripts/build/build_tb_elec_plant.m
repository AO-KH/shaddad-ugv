function build_tb_elec_plant()
%BUILD_TB_ELEC_PLANT  Bench tests/electronics/tb_elec_plant.slx (owner: electronics, T10).
%
%   Closed loop Electronics_Plant + Shaddad_MCU with a stub plant, plus the CAN network emulation and the
%   companion IO (build plan T10 acceptance: getAlgebraicLoops = 0):
%     MCU_In (Bus Creators of Bus_McuIn1k..100: elec_hw DI word / reset_flag, t_us clock, idle SBUS stream,
%       level IMU, CAN1 frames from CAN1_Net_Sim, COMP_CMD from Link_MCU_GNC, ADC counts from elec_adc)
%     -> MCU (Model Shaddad_MCU, Normal) -> Electronics_Plant (out1k DO word / wdg_kick, out10 PWM)
%     -> elec_hw / elec_adc back to MCU_In (elec_hw through the 1 ms DI Unit Delay, D36)
%     MCU out10/out50 CAN1 frames -> CAN1_Net_Sim (ESC / implement / BMS frames: P.ic stubs)
%     MCU CAN-FD frames -> Link_MCU_GNC -> companion_io (SIM) -> stub GNC (Bus_GncCmd = P.ic with
%       seq = mcu_status_dec.heartbeat, i.e. a direct MCU_STATUS -> COMP_CMD feedthrough) -> companion_io
%       COMP_CMD frame -> Link_MCU_GNC -> MCU (closes the MCU <-> GNC loop through the MCU_STATUS delay, D41)
%   Stub plant: power_status healthy (E-stop loop closed, relay closed, key on, rails at nominal, 51.2 V),
%   hmi / impl_mech P.ic, env T_amb 298.15 K, G_solar 0, flt P.ic (no faults).
%   Root outputs (yout): 1 elec_hw, 2 hw_cmd, 3 out1k, 4 elec_env, 5 lv_demand, 6 mcu_status_dec, 7 elec_adc.
%   Config Shaddad_Ref (cs_Shaddad).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level with
%   shaddad_layout(mdl, 'Recursive', true). Every signal line is named (L5). Buses (spec 6.3): Stub_Plant sends
%   its six signals as the virtual bus stub (Out Bus Element), picked by Bus Selectors in front of Electronics,
%   CAN1_Net and CAN2_Link; the four CAN1 frames into MCU_In travel as the virtual bus can1_rx (Bus Creator
%   BC_can1_rx next to CAN1_Net), picked in MCU_In by its In Bus Element port can1. Each MCU output bus is picked
%   by a Bus Selector in front of each consumer (spec 6.2).

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
mdl = 'tb_elec_plant';
f = fullfile(root, 'tests', 'electronics', [mdl '.slx']);
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
%% ---- blocks under test ----
A(mdl, 'simulink/Ports & Subsystems/Model', 'MCU', 'ModelName', 'Shaddad_MCU', 'SimulationMode', 'Normal');
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'Electronics', 'ReferencedSubsystem', 'Electronics_Plant');
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'CAN1_Net', 'ReferencedSubsystem', 'CAN1_Net_Sim');
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'CAN2_Link', 'ReferencedSubsystem', 'Link_MCU_GNC');
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'Companion_IO', 'ReferencedSubsystem', 'companion_io');
%% ---- stub plant: out 1 the virtual bus stub (power_status, hmi, env, impl_mech, flt1k, flt10) ----
s = [mdl '/Stub_Plant'];
add_block('built-in/SubSystem', s);
CST(s, 'PS_ic', 'P.ic.Bus_PowerStatus', 'Bus: Bus_PowerStatus', 'Ts.base');
ps = {'V_pack_V', '51.2'; 'V_bus_V', '51.2'; 'V_24V_V', '24'; 'V_12V_V', '12'; 'V_5V_V', '5'; 'V_3V3_V', '3.3'; ...
    'Rail_24V_OK', 'true'; 'Rail_12V_OK', 'true'; 'Rail_5V_OK', 'true'; 'Rail_3V3_OK', 'true'; 'K1_Closed', 'true'; ...
    'K2_Closed', 'true'; 'Safety_OK', 'true'; 'Safety_HV_OK', 'true'; 'EStop_Ch1_OK', 'true'; 'EStop_Ch2_OK', 'true'; ...
    'Key_On', 'true'};
A(s, 'simulink/Signal Routing/Bus Assignment', 'PS', 'AssignedSignals', strjoin(ps(:, 1)', ','));
L(s, 'PS_ic/1', 'PS/1');
for k = 1:size(ps, 1)
    if strcmp(ps{k, 2}, 'true'), ty = 'boolean'; else, ty = 'double'; end
    CST(s, ['C_' ps{k, 1}], ps{k, 2}, ty, 'Ts.base');
    L(s, ['C_' ps{k, 1} '/1'], sprintf('PS/%d', k + 1));
end
shaddad_bu('outbus', s, 'stub', 'power_status', 'PS/1');
CST(s, 'Hmi', 'P.ic.Bus_HmiInputs', 'Bus: Bus_HmiInputs', 'Ts.base');  shaddad_bu('outbus', s, 'stub', 'hmi', 'Hmi/1');
CST(s, 'Env_ic', 'P.ic.Bus_EnvState', 'Bus: Bus_EnvState', 'Ts.hk');
A(s, 'simulink/Signal Routing/Bus Assignment', 'Env', 'AssignedSignals', 'T_amb,G_solar');
CST(s, 'T_amb', '298.15', 'double', 'Ts.hk');  CST(s, 'G_solar', '0', 'double', 'Ts.hk');
L(s, 'Env_ic/1', 'Env/1');  L(s, 'T_amb/1', 'Env/2');  L(s, 'G_solar/1', 'Env/3');
shaddad_bu('outbus', s, 'stub', 'env', 'Env/1');
CST(s, 'Impl', 'P.ic.Bus_ImplMechState', 'Bus: Bus_ImplMechState', 'Ts.ctrl');
shaddad_bu('outbus', s, 'stub', 'impl_mech', 'Impl/1');
CST(s, 'Flt1k', 'P.ic.Bus_FaultInject', 'Bus: Bus_FaultInject', 'Ts.base');  shaddad_bu('outbus', s, 'stub', 'flt1k', 'Flt1k/1');
CST(s, 'Flt10', 'P.ic.Bus_FaultInject', 'Bus: Bus_FaultInject', 'Ts.ctrl');  shaddad_bu('outbus', s, 'stub', 'flt10', 'Flt10/1');
%% ---- stub ESC / implement / BMS frames (no frames: P.ic) ----
CST(mdl, 'EscFast', 'P.ic.Bus_Can1_EscFast', 'Bus: Bus_Can1_EscFast', 'Ts.ctrl');
CST(mdl, 'EscSlow', 'P.ic.Bus_Can1_EscSlow', 'Bus: Bus_Can1_EscSlow', 'Ts.comm');
CST(mdl, 'ImplFb', 'P.ic.Bus_Can1_Impl', 'Bus: Bus_Can1_Impl', 'Ts.comm');
CST(mdl, 'Bms', 'P.ic.Bus_Can1_Bms', 'Bus: Bus_Can1_Bms', 'Ts.hk');
%% ---- MCU_In: in 1 elec_hw, 2 elec_adc, 3 can1 (In Bus Element: the bus can1_rx), 4 comp ----
m = [mdl '/MCU_In'];
add_block('built-in/SubSystem', m);
A(m, 'simulink/Sources/In1', 'elec_hw', 'Port', '1');
A(m, 'simulink/Sources/In1', 'elec_adc', 'Port', '2');
outs = {'in1k', 'McuIn1k', 5; 'in5', 'McuIn5', 1; 'in10', 'McuIn10', 1; 'in50', 'McuIn50', 5; 'in100', 'McuIn100', 1};
for k = 1:size(outs, 1)
    A(m, 'simulink/Sinks/Out1', outs{k, 1}, 'Port', num2str(k));
    A(m, 'simulink/Signal Routing/Bus Creator', ['BC_' outs{k, 1}], 'Inputs', num2str(outs{k, 3}), ...
        'OutDataTypeStr', ['Bus: Bus_' outs{k, 2}], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
    L(m, ['BC_' outs{k, 1} '/1'], [outs{k, 1} '/1'], outs{k, 1});
end
CST(m, 'Sbus', 'P.ic.Bus_SbusStream', 'Bus: Bus_SbusStream', 'Ts.base');  L(m, 'Sbus/1', 'BC_in1k/1', 'sbus');
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_hw', 'OutputSignals', 'di_word,reset_flag');
L(m, 'elec_hw/1', 'Sel_hw/1', 'elec_hw');
L(m, 'Sel_hw/1', 'BC_in1k/2');
A(m, 'simulink/Sources/Digital Clock', 'Clock_t', 'SampleTime', 'Ts.base');
A(m, 'simulink/Math Operations/Gain', 'Gain_us', 'Gain', '1e6');
A(m, 'simulink/Math Operations/Rounding Function', 'Round_us', 'Operator', 'round');
A(m, 'simulink/Signal Attributes/Data Type Conversion', 'DTC_t_us', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'off');
L(m, 'Clock_t/1', 'Gain_us/1', 't');  L(m, 'Gain_us/1', 'Round_us/1', 't_us_raw');  L(m, 'Round_us/1', 'DTC_t_us/1', 't_us_round');
L(m, 'DTC_t_us/1', 'BC_in1k/3', 't_us');
L(m, 'Sel_hw/2', 'BC_in1k/4');
CST(m, 'FaultInj', '0', 'uint32', 'Ts.base');  L(m, 'FaultInj/1', 'BC_in1k/5', 'fault_inject');
A(m, 'simulink/Signal Routing/Bus Creator', 'BC_imu', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_ImuRaw', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
CST(m, 'Acc', '[0 0 9.80665]', 'single', 'Ts.imu');  CST(m, 'Gyr', '[0 0 0]', 'single', 'Ts.imu');
CST(m, 'Mag', '[20 0 -40]', 'single', 'Ts.imu');     CST(m, 'Temp', '35', 'single', 'Ts.imu');
L(m, 'Acc/1', 'BC_imu/1');  L(m, 'Gyr/1', 'BC_imu/2');  L(m, 'Mag/1', 'BC_imu/3');  L(m, 'Temp/1', 'BC_imu/4');
L(m, 'BC_imu/1', 'BC_in5/1', 'imu');
% the CAN1 frames: picks of the In Bus Element port can1 in front of the Bus Creator inputs (layout spec 6.2)
shaddad_bu('pick', m, 'can1', 'esc_fast_rx', 'BC_in10/1', 'can1_esc');
shaddad_bu('pick', m, 'can1', 'esc_slow_rx', 'BC_in50/1', 'can1_escslow');
shaddad_bu('pick', m, 'can1', 'impl_rx', 'BC_in50/2', 'can1_impl');
A(m, 'simulink/Sources/In1', 'comp', 'Port', '4');
L(m, 'comp/1', 'BC_in50/3', 'can2_comp');
A(m, 'simulink/Signal Routing/Bus Selector', 'Sel_adc', 'OutputSignals', 'adc_raw,mcu_temp_degC');
L(m, 'elec_adc/1', 'Sel_adc/1', 'elec_adc');
L(m, 'Sel_adc/1', 'BC_in50/4');  L(m, 'Sel_adc/2', 'BC_in50/5');
shaddad_bu('pick', m, 'can1', 'bms_rx', 'BC_in100/1', 'can1_bms');
shaddad_bu('portorder', m, {'elec_hw', 'elec_adc', 'can1', 'comp'}, 'Inport');
%% ---- stub GNC: Bus_GncCmd = P.ic with seq = MCU_STATUS heartbeat (direct feedthrough) ----
g = [mdl '/GNC_Stub'];
add_block('built-in/SubSystem', g);
A(g, 'simulink/Sources/In1', 'mcu_status', 'Port', '1');
A(g, 'simulink/Sinks/Out1', 'gnc_cmd', 'Port', '1');
CST(g, 'Cmd_ic', 'P.ic.Bus_GncCmd', 'Bus: Bus_GncCmd', 'Ts.comm');
A(g, 'simulink/Signal Routing/Bus Selector', 'Sel_hb', 'OutputSignals', 'heartbeat');
A(g, 'simulink/Signal Routing/Bus Assignment', 'Cmd', 'AssignedSignals', 'seq');
L(g, 'mcu_status/1', 'Sel_hb/1', 'mcu_status');
L(g, 'Cmd_ic/1', 'Cmd/1', 'gnc_cmd_ic');  L(g, 'Sel_hb/1', 'Cmd/2');  L(g, 'Cmd/1', 'gnc_cmd/1', 'gnc_cmd');
%% ---- wiring ----
% CAN1_Net -> MCU_In: the four received frames as the virtual bus can1_rx (layout spec 6.3)
shaddad_bu('vbus', mdl, 'BC_can1_rx', {'esc_fast_rx', 'CAN1_Net/3'; 'esc_slow_rx', 'CAN1_Net/4'; ...
    'impl_rx', 'CAN1_Net/5'; 'bms_rx', 'CAN1_Net/6'});
L(mdl, 'Electronics/1', 'MCU_In/1', 'elec_hw');
L(mdl, 'Electronics/2', 'MCU_In/2', 'elec_adc');
L(mdl, 'BC_can1_rx/1', 'MCU_In/3', 'can1_rx');
L(mdl, 'CAN2_Link/5', 'MCU_In/4', 'comp_cmd_rx');
for k = 1:5, L(mdl, sprintf('MCU_In/%d', k), sprintf('MCU/%d', k), outs{k, 1}); end
% MCU outputs: a Bus Selector in front of each consumer (layout spec 6.2)
SEL_BUS(mdl, 'Sel_can1_drv', 'can1_drv');        L(mdl, 'MCU/3', 'Sel_can1_drv/1', 'out10');
SEL_BUS(mdl, 'Sel_can2_odom', 'can2_odom');      L(mdl, 'MCU/3', 'Sel_can2_odom/1');
SEL_BUS(mdl, 'Sel_can1_slow', 'can1_slow');      L(mdl, 'MCU/4', 'Sel_can1_slow/1', 'out50');
SEL_BUS(mdl, 'Sel_can2_status', 'can2_status');  L(mdl, 'MCU/4', 'Sel_can2_status/1');
SEL_BUS(mdl, 'Sel_can2_imu', 'can2_imu');        L(mdl, 'MCU/2', 'Sel_can2_imu/1', 'out5');
SEL_BUS(mdl, 'Sel_can2_power', 'can2_power');    L(mdl, 'MCU/5', 'Sel_can2_power/1', 'out100');
A(mdl, 'simulink/Sinks/Terminator', 'T_telem');  L(mdl, 'MCU/6', 'T_telem/1', 'telem');
% Electronics_Plant: 1 mcu_out1k, 2 mcu_out10, 3 power_status, 4 hmi, 5 env, 6 impl_mech, 7 flt
L(mdl, 'MCU/1', 'Electronics/1', 'out1k');
L(mdl, 'MCU/3', 'Electronics/2', 'out10');
SEL_BUS(mdl, 'Sel_stub', 'power_status,hmi,env,impl_mech,flt1k');  L(mdl, 'Stub_Plant/1', 'Sel_stub/1', 'stub');
for k = 1:5, L(mdl, sprintf('Sel_stub/%d', k), sprintf('Electronics/%d', k + 2)); end
% CAN1_Net_Sim: 1 mcu_fast, 2 mcu_slow, 3 esc_fast, 4 esc_slow, 5 impl, 6 bms, 7 flt, 8 elec_hw
L(mdl, 'Sel_can1_drv/1', 'CAN1_Net/1');  L(mdl, 'Sel_can1_slow/1', 'CAN1_Net/2');
L(mdl, 'EscFast/1', 'CAN1_Net/3', 'esc_fast');    L(mdl, 'EscSlow/1', 'CAN1_Net/4', 'esc_slow');
L(mdl, 'ImplFb/1', 'CAN1_Net/5', 'impl_fb');      L(mdl, 'Bms/1', 'CAN1_Net/6', 'bms');
SEL_BUS(mdl, 'Sel_flt10', 'flt10');  L(mdl, 'Stub_Plant/1', 'Sel_flt10/1');  L(mdl, 'Sel_flt10/1', 'CAN1_Net/7');
L(mdl, 'Electronics/1', 'CAN1_Net/8');
A(mdl, 'simulink/Sinks/Terminator', 'T_mcu_fast_rx');  L(mdl, 'CAN1_Net/1', 'T_mcu_fast_rx/1', 'mcu_fast_rx');
A(mdl, 'simulink/Sinks/Terminator', 'T_mcu_slow_rx');  L(mdl, 'CAN1_Net/2', 'T_mcu_slow_rx/1', 'mcu_slow_rx');
% Link_MCU_GNC: 1 mcu_imu, 2 mcu_odom, 3 mcu_status, 4 mcu_power, 5 comp_cmd, 6 flt, 7 elec_hw
L(mdl, 'Sel_can2_imu/1', 'CAN2_Link/1');  L(mdl, 'Sel_can2_odom/1', 'CAN2_Link/2');  L(mdl, 'Sel_can2_status/1', 'CAN2_Link/3');
L(mdl, 'Sel_can2_power/1', 'CAN2_Link/4');  L(mdl, 'Companion_IO/5', 'CAN2_Link/5', 'comp_cmd');
SEL_BUS(mdl, 'Sel_flt10_2', 'flt10');  L(mdl, 'Stub_Plant/1', 'Sel_flt10_2/1');  L(mdl, 'Sel_flt10_2/1', 'CAN2_Link/6');
L(mdl, 'Electronics/1', 'CAN2_Link/7');
% companion_io: 1..4 frames, 5 gnc_cmd
rx = {'mcu_imu_rx', 'mcu_odom_rx', 'mcu_status_rx', 'mcu_power_rx'};
for k = 1:4, L(mdl, sprintf('CAN2_Link/%d', k), sprintf('Companion_IO/%d', k), rx{k}); end
L(mdl, 'Companion_IO/3', 'GNC_Stub/1', 'mcu_status_dec');
L(mdl, 'GNC_Stub/1', 'Companion_IO/5', 'gnc_cmd');
tc = {1, 'imu_meas'; 2, 'odom_meas'; 4, 'bms_status'};
for k = 1:size(tc, 1)
    A(mdl, 'simulink/Sinks/Terminator', sprintf('T_comp%d', tc{k, 1}));
    L(mdl, sprintf('Companion_IO/%d', tc{k, 1}), sprintf('T_comp%d/1', tc{k, 1}), tc{k, 2});
end
%% ---- root outputs ----
ro = {'elec_hw', 'Electronics/1', 'ElecHwIn'; 'hw_cmd', 'Electronics/4', 'PowerHwCmd'; 'out1k', 'MCU/1', 'McuOut1k'; ...
    'elec_env', 'Electronics/3', 'ElecEnv'; 'lv_demand', 'Electronics/6', 'LV_LoadDemand'; ...
    'mcu_status_dec', 'Companion_IO/3', 'McuStatus'; 'elec_adc', 'Electronics/2', 'ElecAdc'};
for k = 1:size(ro, 1)
    A(mdl, 'simulink/Sinks/Out1', ro{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ['Bus: Bus_' ro{k, 3}]);
    L(mdl, ro{k, 2}, [ro{k, 1} '/1'], ro{k, 1});
end
te = {5, 'impl_cmd'; 7, 'indicators'};
for k = 1:size(te, 1)
    A(mdl, 'simulink/Sinks/Terminator', sprintf('T_ep%d', te{k, 1}));
    L(mdl, sprintf('Electronics/%d', te{k, 1}), sprintf('T_ep%d/1', te{k, 1}), te{k, 2});
end
set_param(mdl, 'Description', ['tb_elec_plant (T10): closed loop Electronics_Plant + Shaddad_MCU + CAN1_Net_Sim + ' ...
    'Link_MCU_GNC + companion_io with a stub plant and a stub GNC (seq = MCU heartbeat). out 1 elec_hw, 2 hw_cmd, ' ...
    '3 out1k, 4 elec_env, 5 lv_demand, 6 mcu_status_dec, 7 elec_adc. Stub_Plant sends the virtual bus stub; ' ...
    'CAN1_Net''s four received frames reach MCU_In as the virtual bus can1_rx. Built by ' ...
    'scripts/build/build_tb_elec_plant.m; used by tests/electronics/t_elec_plant (getAlgebraicLoops = 0, short ' ...
    'simulation).']);
shaddad_bu('save', mdl, f);
shaddad_bu('finalize', mdl, f);
close_system(mdl, 0);
end

function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function h = L(sys, src, dst, nm)
% a named signal line (layout spec 6.1, L5): shaddad_bu('nline') names the line nm, else keeps the name its source
% port's line already has, else names it after its source block; a Bus Selector's output keeps '<element>'
if nargin < 4, nm = ''; end
h = shaddad_bu('nline', sys, src, dst, nm);
end

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function SEL_BUS(sys, name, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', name, 'OutputSignals', sigs);
end
