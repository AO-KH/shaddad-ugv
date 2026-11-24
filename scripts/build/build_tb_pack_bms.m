function build_tb_pack_bms()
%BUILD_TB_PACK_BMS  Pack + BMS test bench tests/power/tb_pack_bms.slx (owner: power, T04).
%
%   build_tb_pack_bms()   (re)creates the bench (build plan T04.7)
%
%   Contents: Battery_Pack Variant Subsystem (VC_PACK 1 Pack_Grouped / 2 Pack_Lumped), Pack_Thermal,
%   ee_lib SPST Relay contactors K1 (main), K2 (precharge, 22 Ohm), K4 (charge), ee_lib Capacitor 5.5 mF
%   (ESR TB.R_esr, 5 kOhm bleed) as the DRIVE / implement DC-link stub, fl_lib Controlled Current Sources
%   for the bus load profile (gated by the 1 ms-delayed bus voltage, D36) and for the dock charger
%   (I = Charger_Enable x min(I_req, I_max, P_max / V_dock(k-1)), 20 A/s slew, 0.5 s lag, behind K4,
%   OVP), current / voltage sensors, Electrical Reference, Solver Configuration (local Backward Euler
%   P.sim.ssc.Ts, fixed cost), Power_Sensors, Model block Shaddad_BMS, MCU BMS_REQ stimulus through
%   bmsreq_can_pack, coil logic K1/K2 = cmd AND Safety_HV_OK, K4 = cmd AND dock, On-Off Delays
%   P.power.pdu.K*.t_on / t_off (aux contacts).
%   Pack_Thermal and Power_Sensors take buses (model layout spec 6.2-6.3): the bench builds them with virtual
%   Bus Creators from its sensors and stubs (env, the applied fan command, and for Power_Sensors pack, safety,
%   pdu, lv, impl, charger and flt, the element names of Power_Plant's buses). Each consumer of the BMS command
%   bus picks its signals with a Bus Selector right in front of it, and log_fast picks the meters and powers that
%   the pdu bus carries from it. Every signal line is named, and the layout engine places the bench when
%   shaddad_bu saves it.
%   Stimulus: model-workspace struct TB (tests/power/tb_pack_bms_default.m), overridden by tests with
%   SimulationInput.setVariable('TB', TB, 'Workspace', 'tb_pack_bms').
%   Logs: log_fast (1 ms, decimation TB.dec_fast) = [t V_pack V_bus V_dock I_pack I_load I_chg K1 K2 K4
%   P_pack P_trac P_chg_bus P_pdu_loss]; log_cmd (0.01 s, decimation TB.dec_cmd) = [t K1_Cmd K2_Cmd
%   K4_Cmd Charger_Enable I_req Fan_Cmd n_Bal HV_Enable_Req_MCU]; signal logging telem, power_truth
%   (Bus objects, 0.1 s).

mdl = 'tb_pack_bms';
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
f = fullfile(root, 'tests', 'power', [mdl '.slx']);
EL = 'Connection: foundation.electrical.electrical';
TH = 'Connection: foundation.thermal.thermal';
SPS = sprintf('nesl_utility/Simulink-PS\nConverter');
PSS = sprintf('nesl_utility/PS-Simulink\nConverter');
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
hws = get_param(mdl, 'ModelWorkspace');
assignin(hws, 'TB', tb_pack_bms_default());
% StopTime: set per run (SimulationInput 'StopTime'; the config set is the shared reference cs_Shaddad)
A = @(lib, n, varargin) shaddad_bu('add', lib, [mdl '/' n], varargin{:});
L = @(s, d, varargin) shaddad_bu('nline', mdl, s, d, varargin{:});
PL = @(a, b) shaddad_bu('pline', mdl, a, b);
PH = @(b) get_param([mdl '/' b], 'PortHandles');
RT = @(n, ts) A('simulink/Signal Attributes/Rate Transition', n, 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', ts);
LUT = @(n, clk, bp, tab) A('simulink/Lookup Tables/1-D Lookup Table', n, 'BreakpointsForDimension1', bp, 'Table', tab, ...
    'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip', 'Position', [0 0 40 30]);   % 30 px tall like the comparison it feeds
CST = @(n, v, varargin) A('simulink/Sources/Constant', n, 'Value', v, varargin{:});
BCV = @(n, el) shaddad_bu('vbus', mdl, n, el);
SEL = @(n, sigs) A('simulink/Signal Routing/Bus Selector', n, 'OutputSignals', strjoin(sigs, ','));

%% ---- clocks ----
A('simulink/Sources/Digital Clock', 'Clock_1ms', 'SampleTime', 'Ts.base');
A('simulink/Sources/Digital Clock', 'Clock_50ms', 'SampleTime', 'Ts.can_slow');
A('simulink/Sources/Digital Clock', 'Clock_100ms', 'SampleTime', 'Ts.env');
% one clock per group of stimulus tables (fix round 1, ruling F-R3), copies of Clock_1ms (same parameters and
% sample time); the original feeds log_fast's time column
add_block([mdl '/Clock_1ms'], [mdl '/Clock_1ms_tab']);
add_block([mdl '/Clock_1ms'], [mdl '/Clock_1ms_load']);

%% ---- Battery_Pack variant (VC_PACK), outputs in the order of the pack files ----
v = [mdl '/Battery_Pack'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', v);
ch = find_system(v, 'MatchFilter', @Simulink.match.allVariants, 'SearchDepth', 1);
ch = setdiff(ch, {v});
for k = 1:numel(ch), delete_block(ch{k}); end
add_block('simulink/Sources/In1', [v '/Bal_Cmd'], 'Port', '1');
add_block('simulink/Sinks/Out1', [v '/T_cell_true'], 'Port', '1');
add_block('simulink/Sinks/Out1', [v '/SOC_true'], 'Port', '2');
add_block('simulink/Sinks/Out1', [v '/V_cell_true'], 'Port', '3');
add_block('nesl_utility/Connection Port', [v '/pack_p'], 'Port', '1', 'Side', 'Right', 'ConnectionType', EL);
add_block('nesl_utility/Connection Port', [v '/pack_n'], 'Port', '2', 'Side', 'Right', 'ConnectionType', EL);
add_block('nesl_utility/Connection Port', [v '/AmbH'], 'Port', '3', 'Side', 'Left', 'ConnectionType', TH);
add_block('simulink/Ports & Subsystems/Subsystem Reference', [v '/Grouped'], 'ReferencedSubsystem', 'Pack_Grouped');
set_param([v '/Grouped'], 'VariantControl', 'VC_PACK == 1');
add_block('simulink/Ports & Subsystems/Subsystem Reference', [v '/Lumped'], 'ReferencedSubsystem', 'Pack_Lumped');
set_param([v '/Lumped'], 'VariantControl', 'VC_PACK == 2');
set_param(v, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
A('simulink/Ports & Subsystems/Subsystem Reference', 'Pack_Thermal', 'ReferencedSubsystem', 'Pack_Thermal');
PL(shaddad_bu('pport', v, 'AmbH'), shaddad_bu('pport', [mdl '/Pack_Thermal'], 'AmbH'));

%% ---- electrical network ----
CS = 'fl_lib/Electrical/Electrical Sensors/Current Sensor';
VS = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
CCS = 'fl_lib/Electrical/Electrical Sources/Controlled Current Source';
RLY = 'ee_lib/Switches & Breakers/Relays/SPST Relay';
pd = 'P.power.pdu';
A(CS, 'I_shunt');
A(VS, 'V_pack_sens'); A(VS, 'V_bus_sens'); A(VS, 'V_dock_sens');
for k = {'K1', 'K2', 'K4'}
    A(RLY, k{1}, 'R_closed', [pd '.' k{1} '.R_closed'], 'G_open', [pd '.' k{1} '.G_open'], 'Threshold', '0.5');
end
A('ee_lib/Passive/Resistor', 'R_pre', 'R', [pd '.R_pre']);
A('ee_lib/Passive/Capacitor', 'C_bus', 'c', [pd '.C_bus_total_assumed'], 'r', 'TB.R_esr', 'g', [pd '.g_bleed'], ...
    'vc_specify', 'on', 'vc_priority', 'High', 'vc', '0');
A(CCS, 'Load'); A(CCS, 'Charger');
A('ee_lib/Passive/Resistor', 'R_dock_bleed', 'R', 'P.power.charger.R_bleed');
A('fl_lib/Electrical/Electrical Elements/Electrical Reference', 'GND');
A(sprintf('nesl_utility/Solver\nConfiguration'), 'Solver', 'UseLocalSolver', 'on', ...
    'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', 'LocalSolverSampleTime', 'P.sim.ssc.Ts', 'DoFixedCost', 'on', ...
    'MaxNonlinIter', 'P.power.tb.MaxNonlinIter', 'MaxModeIter', 'P.power.tb.MaxModeIter');
hPP = shaddad_bu('pport', v, 'pack_p'); hPN = shaddad_bu('pport', v, 'pack_n');
pI = PH('I_shunt'); pVp = PH('V_pack_sens'); pVb = PH('V_bus_sens'); pVd = PH('V_dock_sens');
pK1 = PH('K1'); pK2 = PH('K2'); pK4 = PH('K4'); pR = PH('R_pre'); pC = PH('C_bus');
pLd = PH('Load'); pCh = PH('Charger'); pRb = PH('R_dock_bleed'); pG = PH('GND'); pSo = PH('Solver');
gnd = pG.LConn(1);
% relays: find the PS control port by domain-checked trial, the other two are the contacts
kc = struct();
for k = {'K1', 'K2', 'K4'}
    A(SPS, [k{1} '_coil_PS'], 'Unit', '1', 'FilteringAndDerivatives', 'zero');
    ps = PH([k{1} '_coil_PS']);
    pk = PH(k{1});
    cand = [pk.LConn, pk.RConn];
    used = local_plink(mdl, ps.RConn(1), cand);
    kc.(k{1}) = cand(setdiff(1:numel(cand), used));
end
% shunt current sensor GND -> pack_n (+ = discharge), pack voltage across the terminals
PL(pI.LConn(1), gnd); PL(pI.RConn(2), hPN);
PL(pVp.LConn(1), hPP); PL(pVp.RConn(2), hPN);
% K1 main, K2 + R_pre precharge, bus capacitor, load
PL(kc.K1(1), hPP); PL(kc.K2(1), hPP); PL(kc.K4(1), hPP);
PL(kc.K2(2), pR.LConn(1));
PL(kc.K1(2), pC.LConn(1)); PL(pR.RConn(1), pC.LConn(1));
PL(pC.RConn(1), gnd);
PL(pLd.RConn(2), pC.LConn(1)); PL(pLd.LConn(1), gnd);           % tail on the bus, head on GND: + = load
PL(pVb.LConn(1), pC.LConn(1)); PL(pVb.RConn(2), gnd);
% K4 -> dock node, charger current source into the dock node, 10 kOhm bleed
PL(kc.K4(2), pRb.LConn(1)); PL(pRb.RConn(1), gnd);
PL(pCh.LConn(1), pRb.LConn(1)); PL(pCh.RConn(2), gnd);          % head on DOCK_P: + = charging current
PL(pVd.LConn(1), pRb.LConn(1)); PL(pVd.RConn(2), gnd);
PL(pSo.RConn(1), gnd);
% sensor outputs
for s = {'I_shunt', 'V_pack_sens', 'V_bus_sens', 'V_dock_sens'}
    A(PSS, [s{1} '_out']);
    pp = PH(s{1}); po = PH([s{1} '_out']);
    PL(pp.RConn(1), po.LConn(1));
end

%% ---- MCU stimulus: BMS_REQ through bmsreq_can_pack (0.05 s) ----
req = {'hv', 'HV_Enable_Req'; 'chg', 'Charge_Enable_Req'; 'rst', 'Fault_Reset_Req'; 'sd', 'Shutdown_Req'};
for k = 1:size(req, 1)
    LUT(['MCU_' req{k, 2}], 'Clock_50ms', ['TB.' req{k, 1} '_t'], ['TB.' req{k, 1} '_v']);
    A('simulink/Logic and Bit Operations/Compare To Zero', ['MCU_' req{k, 2} '_b'], 'relop', '~=', 'OutDataTypeStr', 'boolean', 'Position', [0 0 40 30]);
    L('Clock_50ms/1', ['MCU_' req{k, 2} '/1'], 't_50ms');
    L(['MCU_' req{k, 2} '/1'], ['MCU_' req{k, 2} '_b/1'], [req{k, 2} '_tab']);
end
A('simulink/Sources/Counter Free-Running', 'MCU_alive', 'NumBits', '8', 'tsamp', 'Ts.can_slow');
A('simulink/Signal Attributes/Data Type Conversion', 'MCU_alive_u8', 'OutDataTypeStr', 'uint8');
A('simulink/Logic and Bit Operations/Compare To Constant', 'MCU_alive_run', 'relop', '<', 'const', 'TB.alive_freeze_t', ...
    'OutDataTypeStr', 'boolean');
A('simulink/Signal Routing/Switch', 'MCU_alive_sw', 'Criteria', 'u2 ~= 0');
A('simulink/Discrete/Unit Delay', 'MCU_alive_hold', 'InitialCondition', 'uint8(0)', 'SampleTime', 'Ts.can_slow');
L('MCU_alive/1', 'MCU_alive_u8/1', 'alive_count');
L('Clock_50ms/1', 'MCU_alive_run/1');
L('MCU_alive_u8/1', 'MCU_alive_sw/1', 'alive_count_u8'); L('MCU_alive_run/1', 'MCU_alive_sw/2', 'alive_running');
L('MCU_alive_hold/1', 'MCU_alive_sw/3', 'alive_held');
L('MCU_alive_sw/1', 'MCU_alive_hold/1', 'alive');
A('simulink/Signal Routing/Bus Creator', 'BmsReq_BC', 'Inputs', '5', 'OutDataTypeStr', 'Bus: Bus_BmsReq', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L('MCU_alive_sw/1', 'BmsReq_BC/1', 'AliveCounter');
for k = 1:size(req, 1)
    L(['MCU_' req{k, 2} '_b/1'], sprintf('BmsReq_BC/%d', k + 1), req{k, 2});
end
A('simulink/Ports & Subsystems/Subsystem Reference', 'BMS_REQ_Pack', 'ReferencedSubsystem', 'bmsreq_can_pack');
L('BmsReq_BC/1', 'BMS_REQ_Pack/1', 'bms_req');
A('simulink/Sources/Constant', 'Impl_cmd_none', 'Value', 'P.ic.Bus_CanFrame8', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', ...
    'SampleTime', 'Ts.can_slow');
A('simulink/Signal Routing/Bus Creator', 'CanSlow_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_McuSlow', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L('Impl_cmd_none/1', 'CanSlow_BC/1', 'impl_cmd'); L('BMS_REQ_Pack/1', 'CanSlow_BC/2', 'bms_req');

%% ---- BMS model ----
A('simulink/Ports & Subsystems/Model', 'BMS', 'ModelName', 'Shaddad_BMS', 'SimulationMode', 'Normal');
A('simulink/Ports & Subsystems/Subsystem Reference', 'Power_Sensors', 'ReferencedSubsystem', 'Power_Sensors');
L('Power_Sensors/1', 'BMS/1', 'bms_meas');
L('CanSlow_BC/1', 'BMS/2', 'can_rx');
A('simulink/Sinks/Terminator', 'can_tx_term');
L('BMS/2', 'can_tx_term/1', 'can_tx');
% the command bus bms_cmd: a Bus Selector right in front of each consumer (spec 6.2)
cmd = @(sel, sigs, dst) local_cmd_pick(mdl, sel, sigs, dst);

%% ---- safety / dock stimulus (1 ms) and coil logic ----
LUT('Safety_HV_OK_tab', 'Clock_1ms_tab', 'TB.shv_t', 'TB.shv_v');
% Safety_HV_OK_tab 30 px wide (graphical; fix round 5, ruling F4-R2, t6_fix5_probe2_greedy.log): with the log_cmd
% conversions 90 px tall (below) its line rises 109 px from its port, so its name keeps its label room (106 px; 104
% px at 40 px wide)
set_param([mdl '/Safety_HV_OK_tab'], 'Position', [0 0 30 30]);
LUT('Safety_OK_tab', 'Clock_1ms_tab', 'TB.sok_t', 'TB.sok_v');
LUT('Dock_tab', 'Clock_1ms_tab', 'TB.dock_t', 'TB.dock_v');
LUT('Dock_tab_100ms', 'Clock_100ms', 'TB.dock_t', 'TB.dock_v');
LUT('Fault_tab', 'Clock_1ms_tab', 'TB.flt_t', 'TB.flt_v');
bnm = {'Safety_HV_OK', 'Safety_OK', 'dock_present', 'bms_fault'};
tbs = {'Safety_HV_OK_tab', 'Safety_OK_tab', 'Dock_tab', 'Fault_tab'};
for k = 1:numel(tbs)
    L('Clock_1ms_tab/1', [tbs{k} '/1'], 't_1ms_tab');
    A('simulink/Logic and Bit Operations/Compare To Zero', [tbs{k} '_b'], 'relop', '~=', 'OutDataTypeStr', 'boolean', 'Position', [0 0 40 30]);
    L([tbs{k} '/1'], [tbs{k} '_b/1'], [bnm{k} '_tab']);
end
L('Clock_100ms/1', 'Dock_tab_100ms/1', 't_100ms');
A('simulink/Logic and Bit Operations/Compare To Zero', 'Dock_tab_100ms_b', 'relop', '~=', 'OutDataTypeStr', 'boolean', 'Position', [0 0 40 30]);
L('Dock_tab_100ms/1', 'Dock_tab_100ms_b/1', 'dock_contact_tab');
coil = {'K1', 'K1_Cmd', 'Safety_HV_OK_tab_b'; 'K2', 'K2_Cmd', 'Safety_HV_OK_tab_b'; 'K4', 'K4_Cmd', 'Dock_tab_b'};
for k = 1:size(coil, 1)
    c = coil{k, 1};
    RT([c '_cmd_RT'], 'Ts.base');
    cmd([c '_Cmd_sel'], coil(k, 2), {[c '_cmd_RT/1']});
    A('simulink/Logic and Bit Operations/Logical Operator', [c '_coil'], 'Operator', 'AND', 'Inputs', '2');
    L([c '_cmd_RT/1'], [c '_coil/1'], [coil{k, 2} '_1ms']);
    L([coil{k, 3} '/1'], [c '_coil/2'], bnm{strcmp(tbs, extractBefore(coil{k, 3}, '_b'))});
    A('ee_sl_lib/General Control/On-Off Delay', [c '_delay'], 'Td_on', [pd '.' c '.t_on'], 'Td_off', [pd '.' c '.t_off'], ...
        'IC', '0', 'Ts', 'Ts.base');
    L([c '_coil/1'], [c '_delay/1'], [c '_coil']);
    A('simulink/Signal Attributes/Data Type Conversion', [c '_aux'], 'OutDataTypeStr', 'boolean');
    L([c '_delay/1'], [c '_aux/1'], [c '_contact']);
    A('simulink/Signal Attributes/Data Type Conversion', [c '_aux_dbl'], 'OutDataTypeStr', 'double');
    L([c '_aux/1'], [c '_aux_dbl/1'], [c '_closed']);
    L([c '_aux_dbl/1'], [c '_coil_PS/1'], [c '_closed_dbl']);
end

%% ---- load profile, gated by the 1 ms-delayed bus voltage (D36) ----
LUT('Load_tab', 'Clock_1ms_load', 'TB.load_t', 'TB.load_I');
L('Clock_1ms_load/1', 'Load_tab/1', 't_1ms_load');
A('simulink/Discrete/Unit Delay', 'V_bus_d', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
L('V_bus_sens_out/1', 'V_bus_d/1', 'V_bus');
A('simulink/Math Operations/Bias', 'V_gate_off', 'Bias', '-TB.V_gate(1)');
A('simulink/Math Operations/Gain', 'V_gate_k', 'Gain', '1/(TB.V_gate(2) - TB.V_gate(1))');
A('simulink/Discontinuities/Saturation', 'V_gate', 'UpperLimit', '1', 'LowerLimit', '0');
A('simulink/Math Operations/Product', 'I_load', 'Inputs', '2');
L('V_bus_d/1', 'V_gate_off/1', 'V_bus_d'); L('V_gate_off/1', 'V_gate_k/1', 'V_gate_dV'); L('V_gate_k/1', 'V_gate/1', 'V_gate_raw');
% TB.load_is_power: the table is a DC power demand (W) -> I = P / max(V_bus(k-1), 30 V)
A('simulink/Discontinuities/Saturation', 'V_bus_min', 'UpperLimit', 'inf', 'LowerLimit', '30');
A('simulink/Math Operations/Product', 'I_from_P', 'Inputs', '*/');
A('simulink/Sources/Constant', 'Load_is_power', 'Value', 'TB.load_is_power', 'SampleTime', 'Ts.base');
A('simulink/Signal Routing/Switch', 'Load_mode', 'Criteria', 'u2 ~= 0');
L('V_bus_d/1', 'V_bus_min/1');
L('Load_tab/1', 'I_from_P/1', 'load_tab'); L('V_bus_min/1', 'I_from_P/2', 'V_bus_d_min');
L('I_from_P/1', 'Load_mode/1', 'I_from_P'); L('Load_is_power/1', 'Load_mode/2', 'load_is_power'); L('Load_tab/1', 'Load_mode/3');
L('Load_mode/1', 'I_load/1', 'I_load_req'); L('V_gate/1', 'I_load/2', 'V_gate');
A(SPS, 'Load_PS', 'Unit', 'A', 'FilteringAndDerivatives', 'zero');
L('I_load/1', 'Load_PS/1', 'I_load');
pp = PH('Load_PS'); PL(pp.RConn(1), pLd.RConn(1));

%% ---- dock charger (BMS-controlled current source behind K4); charger controller at 0.01 s ----
% V_dock feedback: 1 ms Unit Delay (D36), then sampled at 0.01 s (fast-to-slow RT)
A('simulink/Discrete/Unit Delay', 'V_dock_d', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
L('V_dock_sens_out/1', 'V_dock_d/1', 'V_dock');
RT('V_dock_RT', 'Ts.bms_fast');
L('V_dock_d/1', 'V_dock_RT/1', 'V_dock_d');
A('simulink/Discontinuities/Saturation', 'V_dock_min', 'UpperLimit', 'inf', 'LowerLimit', '1');
A('simulink/Sources/Constant', 'P_chg_max', 'Value', 'P.power.charger.P_max', 'SampleTime', 'Ts.bms_fast');
A('simulink/Math Operations/Product', 'I_pmax', 'Inputs', '*/');
A('simulink/Signal Attributes/Data Type Conversion', 'I_req_dbl', 'OutDataTypeStr', 'double');
A('simulink/Sources/Constant', 'I_chg_max', 'Value', 'P.power.charger.I_max', 'SampleTime', 'Ts.bms_fast');
A('simulink/Math Operations/MinMax', 'I_chg_law', 'Function', 'min', 'Inputs', '3');
A('simulink/Signal Attributes/Data Type Conversion', 'Chg_en_dbl', 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Product', 'I_chg_en', 'Inputs', '2');
L('V_dock_RT/1', 'V_dock_min/1', 'V_dock_10ms'); L('P_chg_max/1', 'I_pmax/1', 'P_chg_max');
L('V_dock_min/1', 'I_pmax/2', 'V_dock_min');
cmd('I_req_sel', {'Charger_I_Req_A'}, {'I_req_dbl/1'});
L('I_req_dbl/1', 'I_chg_law/1', 'I_req'); L('I_chg_max/1', 'I_chg_law/2', 'I_chg_max'); L('I_pmax/1', 'I_chg_law/3', 'I_pmax');
cmd('Chg_en_sel', {'Charger_Enable'}, {'Chg_en_dbl/1'});
L('I_chg_law/1', 'I_chg_en/1', 'I_chg_law'); L('Chg_en_dbl/1', 'I_chg_en/2', 'charger_enable');
A('simulink/Sources/Constant', 'Chg_force', 'Value', 'TB.chg_force', 'SampleTime', 'Ts.bms_fast');
A('simulink/Sources/Constant', 'Chg_force_I', 'Value', 'TB.chg_force_I', 'SampleTime', 'Ts.bms_fast');
A('simulink/Signal Routing/Switch', 'Chg_force_sw', 'Criteria', 'u2 ~= 0');
L('Chg_force_I/1', 'Chg_force_sw/1', 'chg_force_I'); L('Chg_force/1', 'Chg_force_sw/2', 'chg_force');
L('I_chg_en/1', 'Chg_force_sw/3', 'I_chg_cmd');
A('simulink/Discontinuities/Rate Limiter', 'Chg_slew', 'RisingSlewLimit', 'P.power.charger.slew', ...
    'FallingSlewLimit', '-P.power.charger.slew', 'SampleTimeMode', 'inherited', 'InitialCondition', '0');
A('ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)', 'Chg_lag', 'K', '1', 'T', 'P.power.charger.tau', ...
    'Ts', 'Ts.bms_fast', 'init_option', 'Specify', 'x0', '0');
L('Chg_force_sw/1', 'Chg_slew/1', 'I_chg_target'); L('Chg_slew/1', 'Chg_lag/1', 'I_chg_slewed');
RT('Chg_RT', 'Ts.base');
L('Chg_lag/1', 'Chg_RT/1', 'I_chg_set');
A('simulink/Logic and Bit Operations/Compare To Constant', 'OVP_ok', 'relop', '<', 'const', 'P.power.charger.V_ovp', ...
    'OutDataTypeStr', 'boolean');
A('simulink/Signal Attributes/Data Type Conversion', 'OVP_ok_dbl', 'OutDataTypeStr', 'double');
L('V_dock_d/1', 'OVP_ok/1'); L('OVP_ok/1', 'OVP_ok_dbl/1', 'ovp_ok');
A('simulink/Math Operations/Product', 'I_chg', 'Inputs', '3');
L('Chg_RT/1', 'I_chg/1', 'I_chg_set_1ms'); L('K4_aux_dbl/1', 'I_chg/2'); L('OVP_ok_dbl/1', 'I_chg/3', 'ovp_ok_dbl');
A(SPS, 'Charger_PS', 'Unit', 'A', 'FilteringAndDerivatives', 'zero');
L('I_chg/1', 'Charger_PS/1', 'I_chg');
pp = PH('Charger_PS'); PL(pp.RConn(1), pCh.RConn(1));

%% ---- balancing, fans, thermal inputs ----
cmd('Bal_sel', {'Bal_Cmd'}, {'Battery_Pack/1'});
A('simulink/Sources/Constant', 'Fan_off', 'Value', 'single(0)', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.bms_fast');
A('simulink/Sources/Constant', 'Fan_on', 'Value', 'single(1)', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.bms_fast');
A('simulink/Sources/Constant', 'Fan_mode', 'Value', 'TB.fan_mode', 'SampleTime', 'Ts.bms_fast');
A('simulink/Signal Routing/Multiport Switch', 'Fan_sel', 'Inputs', '3', 'DataPortOrder', 'Zero-based contiguous');
L('Fan_mode/1', 'Fan_sel/1', 'fan_mode');
cmd('Fan_cmd_sel', {'Fan_Cmd'}, {'Fan_sel/2'});
L('Fan_off/1', 'Fan_sel/3', 'fan_off'); L('Fan_on/1', 'Fan_sel/4', 'fan_on');
A('simulink/Sources/Constant', 'T_amb', 'Value', 'TB.T_amb', 'SampleTime', 'Ts.env');
A('simulink/Sources/Constant', 'G_solar', 'Value', 'TB.G_solar', 'SampleTime', 'Ts.env');
% Pack_Thermal picks bms_cmd.Fan_Cmd (here the applied fan command) and env.T_amb, env.G_solar, env.dock_contact
BCV('Fan_BC', {'Fan_Cmd', 'Fan_sel/1'});
L('Fan_BC/1', 'Pack_Thermal/1', 'fan_cmd');
BCV('Env_BC', {'T_amb', 'T_amb/1'; 'G_solar', 'G_solar/1'; 'dock_contact', 'Dock_tab_100ms_b/1'});
L('Env_BC/1', 'Pack_Thermal/2', 'env');

%% ---- powers and the Power_Sensors buses (the element names of Power_Plant's buses; stubs for what the bench
%      does not model) ----
A('simulink/Math Operations/Product', 'P_pack', 'Inputs', '2');
L('V_pack_sens_out/1', 'P_pack/1', 'V_pack'); L('I_shunt_out/1', 'P_pack/2', 'I_pack');
A('simulink/Math Operations/Product', 'P_trac', 'Inputs', '2');
L('V_bus_sens_out/1', 'P_trac/1'); L('I_load/1', 'P_trac/2');
A('simulink/Math Operations/Product', 'P_chg_bus', 'Inputs', '2');
L('V_dock_sens_out/1', 'P_chg_bus/1'); L('I_chg/1', 'P_chg_bus/2');
A('simulink/Math Operations/Sum', 'P_pdu_loss', 'Inputs', '++-');
L('P_pack/1', 'P_pdu_loss/1', 'P_pack'); L('P_chg_bus/1', 'P_pdu_loss/2', 'P_chg_bus'); L('P_trac/1', 'P_pdu_loss/3', 'P_trac_L');
A('simulink/Math Operations/Gain', 'P_grid', 'Gain', '1/P.power.charger.eta');
L('P_chg_bus/1', 'P_grid/1');
z = {'P_trac_R', 'P_bms', 'P_lv', 'P_dcdc_loss', 'P_k3_loss', 'P_impl'};
for k = 1:numel(z), CST(['Zero_' z{k}], '0', 'SampleTime', 'Ts.base'); end
CST('K3_none', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Fuse_ok', 'true(6,1)', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('FuseMain_ok', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Fuse24_ok', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('EStop_none', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Latched_none', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Ch_ok', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Ch2_ok', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Key_on', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
CST('Rails', '[24; 2; 24; 12; 4; 5; 3.3]', 'SampleTime', 'Ts.base');
CST('Rails_ok', 'true(4,1)', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
% the seven stubs of the lv bus 90 px tall (graphical; fix round 2, t6_fix2_greedy.log: 546 -> 531): Lv_BC's fan
% then sets them level with its inputs instead of 1.5 rows below them
for n = {'Rails', 'Rails_ok', 'K3_none', 'Fuse24_ok', 'Zero_P_lv', 'Zero_P_dcdc_loss', 'Zero_P_k3_loss'}
    p = get_param([mdl '/' n{1}], 'Position'); set_param([mdl '/' n{1}], 'Position', [p(1) p(2) p(3) p(2) + 90]);
end
BCV('Pack_BC', {'T_cell_true', 'Battery_Pack/1'; 'SOC_true', 'Battery_Pack/2'; 'V_cell_true', 'Battery_Pack/3'});
BCV('Safety_BC', {'Safety_OK', 'Safety_OK_tab_b/1'; 'Safety_HV_OK', 'Safety_HV_OK_tab_b/1'; 'EStop_Any', 'EStop_none/1'; ...
    'EStop_Ch1_OK', 'Ch_ok/1'; 'EStop_Ch2_OK', 'Ch2_ok/1'; 'Safety_Latched_Off', 'Latched_none/1'; 'key_on', 'Key_on/1'; ...
    'dock_present', 'Dock_tab_b/1'});
BCV('Pdu_BC', {'V_pack', 'V_pack_sens_out/1'; 'V_bus', 'V_bus_sens_out/1'; 'V_dock', 'V_dock_sens_out/1'; ...
    'I_pack', 'I_shunt_out/1'; 'K1_closed', 'K1_aux/1'; 'K2_closed', 'K2_aux/1'; 'K4_closed', 'K4_aux/1'; ...
    'fuse_ok', 'Fuse_ok/1'; 'F_MAIN_ok', 'FuseMain_ok/1'; 'P_trac_L', 'P_trac/1'; 'P_trac_R', 'Zero_P_trac_R/1'; ...
    'P_chg_bus', 'P_chg_bus/1'; 'P_pdu_loss', 'P_pdu_loss/1'; 'P_bms', 'Zero_P_bms/1'});
BCV('Lv_BC', {'rails', 'Rails/1'; 'rail_ok', 'Rails_ok/1'; 'K3_closed', 'K3_none/1'; 'fuse24_ok', 'Fuse24_ok/1'; ...
    'P_lv', 'Zero_P_lv/1'; 'P_dcdc_loss', 'Zero_P_dcdc_loss/1'; 'P_k3_loss', 'Zero_P_k3_loss/1'});
BCV('Impl_BC', {'P_impl', 'Zero_P_impl/1'});
BCV('Charger_BC', {'P_grid', 'P_grid/1'});
BCV('Flt_BC', {'bms_fault', 'Fault_tab_b/1'});
% the env element Power_Sensors picks (T_amb) as a bus of its own, built next to it (routing only; Env_BC feeds
% the distant Pack_Thermal), from a copy of the Constant T_amb (ruling F-R3: in a bench a source may be split per
% consumer group; fix round 2: T_amb's trunk from Env_BC down to Env_ps_BC crossed the stub tables' lines)
add_block([mdl '/T_amb'], [mdl '/T_amb_ps']);
BCV('Env_ps_BC', {'T_amb', 'T_amb_ps/1'});
ps = {'Pack_BC/1', 'pack'; 'Pack_Thermal/1', 'thermal'; 'Pdu_BC/1', 'pdu'; 'Lv_BC/1', 'lv'; 'Safety_BC/1', 'safety'; ...
      'Impl_BC/1', 'impl'; 'Charger_BC/1', 'charger'; 'Env_ps_BC/1', 'env'; 'Flt_BC/1', 'flt'};
for k = 1:size(ps, 1)
    L(ps{k, 1}, sprintf('Power_Sensors/%d', k), ps{k, 2});
end
A('simulink/Sinks/Terminator', 'status_term');
L('Power_Sensors/2', 'status_term/1', 'power_status');
A('simulink/Sinks/Terminator', 'truth_term');
L('Power_Sensors/3', 'truth_term/1', 'power_truth');

%% ---- run termination (TB.stop_*): weakest cell below stop_Vmin, lowest group SOC below stop_SOCmin or
%      above stop_SOCmax, or (stop_on_done) BMS Charge_Complete ----
A('simulink/Math Operations/MinMax', 'Vcell_min_true', 'Function', 'min', 'Inputs', '1');
A('simulink/Math Operations/MinMax', 'SOC_min_true', 'Function', 'min', 'Inputs', '1');
L('Battery_Pack/3', 'Vcell_min_true/1'); L('Battery_Pack/2', 'SOC_min_true/1');
A('simulink/Logic and Bit Operations/Compare To Constant', 'Stop_V', 'relop', '<', 'const', 'TB.stop_Vmin', 'OutDataTypeStr', 'boolean');
A('simulink/Logic and Bit Operations/Compare To Constant', 'Stop_SOC', 'relop', '<', 'const', 'TB.stop_SOCmin', 'OutDataTypeStr', 'boolean');
A('simulink/Logic and Bit Operations/Compare To Constant', 'Stop_SOCmax', 'relop', '>', 'const', 'TB.stop_SOCmax', 'OutDataTypeStr', 'boolean');
L('Vcell_min_true/1', 'Stop_V/1', 'V_cell_min'); L('SOC_min_true/1', 'Stop_SOC/1', 'SOC_min');
L('SOC_min_true/1', 'Stop_SOCmax/1');
A('simulink/Signal Routing/Bus Selector', 'Telem_sel', 'OutputSignals', 'Charge_Complete');
L('BMS/3', 'Telem_sel/1', 'telem');
A('simulink/Sources/Constant', 'Stop_on_done', 'Value', 'TB.stop_on_done', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.hk');
A('simulink/Logic and Bit Operations/Logical Operator', 'Stop_done', 'Operator', 'AND', 'Inputs', '2');
L('Telem_sel/1', 'Stop_done/1'); L('Stop_on_done/1', 'Stop_done/2', 'stop_on_done');
RT('Stop_done_RT', 'Ts.base');
L('Stop_done/1', 'Stop_done_RT/1', 'stop_done');
A('simulink/Logic and Bit Operations/Logical Operator', 'Stop_any', 'Operator', 'OR', 'Inputs', '4', ...
    'Position', [0 0 30 180]);  % 45 px per input (graphical; fix round 5, see lc_dbl1-9, t6_fix5_probe2_greedy.log)
L('Stop_V/1', 'Stop_any/1', 'stop_V'); L('Stop_SOC/1', 'Stop_any/2', 'stop_SOC'); L('Stop_done_RT/1', 'Stop_any/3', 'stop_done_1ms');
L('Stop_SOCmax/1', 'Stop_any/4', 'stop_SOCmax');
A('simulink/Sinks/Stop Simulation', 'Stop');
L('Stop_any/1', 'Stop/1', 'stop_run');

%% ---- logs ----
% the meters and the powers that the pdu bus already carries: a pick from it right in front of the log Mux (spec 6.2)
SEL('Log_meas_sel', {'V_pack', 'V_bus', 'V_dock', 'I_pack'});
SEL('Log_pwr_sel', {'P_trac_L', 'P_chg_bus', 'P_pdu_loss'});
L('Pdu_BC/1', 'Log_meas_sel/1'); L('Pdu_BC/1', 'Log_pwr_sel/1');
% the bench's own signals of log_fast travel as one virtual bus, picked right in front of the log (fix round 1)
ls = {'I_load', 'I_load/1'; 'I_chg', 'I_chg/1'; 'K1_closed_dbl', 'K1_aux_dbl/1'; 'K2_closed_dbl', 'K2_aux_dbl/1'; ...
      'K4_closed_dbl', 'K4_aux_dbl/1'; 'P_pack', 'P_pack/1'};
BCV('Log_stub_BC', ls);
SEL('Log_stub_sel', ls(:, 1)');
L('Log_stub_BC/1', 'Log_stub_sel/1', 'log_stub');
lf = [{'Clock_1ms/1'}, arrayfun(@(k) sprintf('Log_meas_sel/%d', k), 1:4, 'UniformOutput', false), ...
      arrayfun(@(k) sprintf('Log_stub_sel/%d', k), 1:6, 'UniformOutput', false), ...
      arrayfun(@(k) sprintf('Log_pwr_sel/%d', k), 1:3, 'UniformOutput', false)];
A('simulink/Signal Routing/Mux', 'log_fast_mux', 'Inputs', num2str(numel(lf)));
L(lf{1}, 'log_fast_mux/1', 't_1ms');
for k = 2:numel(lf), L(lf{k}, sprintf('log_fast_mux/%d', k)); end
A('simulink/Sinks/To Workspace', 'log_fast', 'VariableName', 'log_fast', 'SaveFormat', 'Array', ...
    'Decimation', 'TB.dec_fast', 'SampleTime', '-1', 'MaxDataPoints', 'inf');
L('log_fast_mux/1', 'log_fast/1', 'log_fast');
A('simulink/Sources/Digital Clock', 'Clock_10ms', 'SampleTime', 'Ts.bms_fast');
A('simulink/Math Operations/Sum of Elements', 'n_bal_sum');
A('simulink/Signal Attributes/Data Type Conversion', 'n_bal_dbl', 'OutDataTypeStr', 'double');
cmd('Bal_log_sel', {'Bal_Cmd'}, {'n_bal_dbl/1'});
L('n_bal_dbl/1', 'n_bal_sum/1', 'Bal_Cmd_dbl');
RT('HV_req_RT', 'Ts.bms_fast');
L('MCU_HV_Enable_Req_b/1', 'HV_req_RT/1');
% log_cmd = [t K1_Cmd K2_Cmd K4_Cmd Charger_Enable I_req Fan_Cmd n_Bal HV_Enable_Req_MCU], each through a double conversion
cs = {'K1_Cmd', 'K2_Cmd', 'K4_Cmd', 'Charger_Enable', 'Charger_I_Req_A', 'Fan_Cmd'};
A('simulink/Signal Routing/Mux', 'log_cmd_mux', 'Inputs', '9');
% the nine conversions 90 px tall, with Stop_any 45 px per input (graphical; fix round 5, ruling F4-R2,
% t6_fix5_probe2_greedy.log, t6_fix5_probe6_why.log): the taller log_cmd column puts the BMS block 150 px and the log
% picks 195 px lower, so bms_cmd's branches into the picks pass below the stop logic and Clock_10ms's line, and the
% can_rx line runs below the MCU stimulus, not across bms_cmd's branches along the top (480 -> 466 crossings, OPEN 8)
for k = 1:9
    A('simulink/Signal Attributes/Data Type Conversion', sprintf('lc_dbl%d', k), 'OutDataTypeStr', 'double');
    p = get_param(sprintf('%s/lc_dbl%d', mdl, k), 'Position');
    set_param(sprintf('%s/lc_dbl%d', mdl, k), 'Position', [p(1) p(2) p(3) p(2) + 90]);
end
L('Clock_10ms/1', 'lc_dbl1/1', 't_10ms');
for k = 1:numel(cs)                                  % a pick right in front of each conversion (L3)
    cmd(['Log_' cs{k} '_sel'], cs(k), {sprintf('lc_dbl%d/1', k + 1)});
end
L('n_bal_sum/1', 'lc_dbl8/1', 'n_Bal'); L('HV_req_RT/1', 'lc_dbl9/1', 'HV_Enable_Req_10ms');
lcn = {'t', 'K1_Cmd', 'K2_Cmd', 'K4_Cmd', 'Charger_Enable', 'I_req', 'Fan_Cmd', 'n_Bal', 'HV_Enable_Req_MCU'};
for k = 1:9
    L(sprintf('lc_dbl%d/1', k), sprintf('log_cmd_mux/%d', k), [lcn{k} '_dbl']);
end
A('simulink/Sinks/To Workspace', 'log_cmd', 'VariableName', 'log_cmd', 'SaveFormat', 'Array', ...
    'Decimation', 'TB.dec_cmd', 'SampleTime', '-1', 'MaxDataPoints', 'inf');
L('log_cmd_mux/1', 'log_cmd/1', 'log_cmd');
% signal logging of the bus outputs
for lg = {'BMS', 3, 'telem'; 'Power_Sensors', 3, 'power_truth'}'
    ph = get_param([mdl '/' lg{1}], 'PortHandles');
    set_param(ph.Outport(lg{2}), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', lg{3});
end
set_param(mdl, 'Description', ['tb_pack_bms (T04 bench, build plan T04.7): see scripts/build/build_tb_pack_bms.m. ' ...
    'Stimulus = model-workspace TB (tests/power/tb_pack_bms_default.m).']);
shaddad_bu('save', mdl, f);
close_system(mdl, 0);
end

function local_cmd_pick(mdl, sel, sigs, dst)
% a Bus Selector sel right in front of the inputs dst (cellstr 'Blk/k'), picking sigs from the BMS command bus
% (BMS output 1, line bms_cmd); its output k feeds dst{k}
shaddad_bu('add', 'simulink/Signal Routing/Bus Selector', [mdl '/' sel], 'OutputSignals', strjoin(sigs, ','));
shaddad_bu('nline', mdl, 'BMS/1', [sel '/1'], 'bms_cmd');
for k = 1:numel(dst)
    shaddad_bu('nline', mdl, sprintf('%s/%d', sel, k), dst{k});
end
end

function k = local_plink(sys, hSrc, cand)
for k = 1:numel(cand)
    try
        add_line(sys, hSrc, cand(k), 'autorouting', 'smart');
        return
    catch %#ok<CTCH> wrong domain
    end
end
error('build_tb_pack_bms:plink', 'no compatible port');
end
