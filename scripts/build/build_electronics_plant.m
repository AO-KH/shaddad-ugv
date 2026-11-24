function build_electronics_plant(which)
%BUILD_ELECTRONICS_PLANT  Electronics plant, CAN network emulation and companion IO (build plan T10).
%
%   build_electronics_plant()            build every file below (idempotent: close, delete, recreate, save)
%   build_electronics_plant(names)       only the listed files (cellstr of base names)
%
%   Files (subsystems/electronics/, Subsystem Reference unless stated):
%     elp_watchdog_reset   IWDG emulation (1 ms): no wdg_kick for > P.electronics.wdg.timeout -> mcu_reset for
%                          t_boot, then a one-tick reset_flag pulse
%     elp_hw_frontend      DI word (ICD 6, commutil2 Bit to Integer Converter, 16 bits, 1 ms) and ADC counts
%                          sat(round(V/k/3.3*65535 + n)) (ZOH 0.05, seeded Random Number noise), mcu_temp
%     elp_actuators        Integer to Bit Converter on do_word; DO / PWM forced 0 during mcu_reset;
%                          Bus_PowerHwCmd (1 ms), Bus_ImplementCmd (0.01), Bus_IndicatorOut (0.1)
%     elp_lv_demand        Bus_LV_LoadDemand from DO / PWM states and P.electronics.load (0.01, D61)
%     elp_thermal          enclosure thermal network (own Solver Configuration, local BE P.sim.ssc.Ts_encl)
%     elp_elec_env         Bus_ElecEnv (0.1): T_imu, T_encl, gnssCorrOK / gnssCorrAge, sensPowerOK
%     Electronics_Plant    ICD 3.2 Electronics row; elec_hw and hw_cmd through 1 ms Unit Delays (D36)
%     CAN1_Net_Sim         CAN1 slot drop (drop mask, MCU bus-off, mcu_reset), data hold, latency (ICD 3.1)
%     Link_MCU_GNC         CAN-FD link: drop on can2_loss / can2_busoff / mcu_reset, MCU_STATUS one-sample
%                          delay (loop breaker, D41), COMP_CMD undelayed
%     companion_io_ros2    MODEL (HW frame IO of companion_io): ros2lib Subscribe /can2/rx, Publish /can2/tx
%                          (std_msgs/UInt8MultiArray, 69 B), ID dispatcher (Compare To Constant + Enabled
%                          Subsystems). A model because ROS 2 message buses do not propagate inside a Subsystem
%                          Reference in R2026b (verified, T12/T13 finding).
%     companion_io         Variant VC_TARGET frame source {SIM root frames | HW Model companion_io_ros2} feeding
%                          the T08 CAN-FD codecs (can2_mcu_*_unpack, can2_comp_cmd_pack); Bus_McuStatus.valid by age
%
%   Needs P, Ts, Bus_* in the base workspace (shaddad_params); refreshes P.electronics from
%   params/param_electronics.m when the T10 fields are missing. No MATLAB Function blocks.
%
%   Layout (model layout spec 6 and 7): the builder places no block; shaddad_bu('save') lays every file out with
%   shaddad_layout(<file>, 'Recursive', true) before it saves it. Every signal line is named (L5: L() calls
%   shaddad_bu('nline'); the inputs of a bus-typed Bus Creator take their element names). The root ports stay
%   classic Inports and Outports, and each consumer of a bus element gets its own Bus Selector right in front of it
%   (spec 6.2); inside a frame slot the frame is the In Bus Element port frame, picked in front of each block.

units = {'elp_watchdog_reset', 'elp_hw_frontend', 'elp_actuators', 'elp_lv_demand', 'elp_thermal', ...
    'elp_elec_env', 'Electronics_Plant', 'CAN1_Net_Sim', 'Link_MCU_GNC', 'companion_io_ros2', 'companion_io'};
if nargin < 1 || isempty(which), which = units; end
which = cellstr(which);
bad = setdiff(which, units);
if ~isempty(bad), error('build_electronics_plant:name', 'Unknown unit(s): %s', strjoin(bad, ', ')); end
local_base();
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dirE = fullfile(root, 'subsystems', 'electronics');
for k = 1:numel(units)
    if ~ismember(units{k}, which), continue, end
    f = fullfile(dirE, [units{k} '.slx']);
    feval(['local_' units{k}], units{k}, f);
end
end

% ===========================================================================================
function local_base()
% base P must carry the T10 parameters (only P.electronics is refreshed, other domains untouched)
ok = evalin('base', ['exist(''P'',''var'') && isfield(P,''electronics'') && isfield(P.electronics.can,''drop_bit'')' ...
    ' && isfield(P.electronics.can,''hw'') && isfield(P.electronics.ntrip,''corr_age_ok'')']);
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'')')
    shaddad_params();
elseif ~ok
    evalin('base', 'P.electronics = param_electronics(P);');
end
end

function local_finish(n, f, desc)
set_param(n, 'Description', desc);
shaddad_bu('save', n, f);
close_system(n, 0);
end

% ===========================================================================================
%  elp_watchdog_reset (1 ms)
% ===========================================================================================
function local_elp_watchdog_reset(n, f)
shaddad_bu('fresh', n, f, 'ssr');
ts = 'Ts.base';
IN(n, 'wdg_kick', 1, 'boolean', ts);
OUT(n, 'mcu_reset', 1, 'boolean');
OUT(n, 'reset_flag', 2, 'boolean');
Nto = 'round(P.electronics.wdg.timeout/Ts.base)';
Nboot = 'round(P.electronics.wdg.t_boot/Ts.base)';
% r: reset tick counter (1..Nboot while mcu_reset), c: ticks since the last kick
UD(n, 'r_z', '0', ts);
UD(n, 'c_z', '0', ts);
CMP(n, 'InReset', '>=', '1');            L(n, 'r_z/1', 'InReset/1', 'r_prev');
LOGIC(n, 'KickOrReset', 'OR', 2);        L(n, 'wdg_kick/1', 'KickOrReset/1', 'wdg_kick');
L(n, 'InReset/1', 'KickOrReset/2', 'in_reset_prev');
A(n, 'simulink/Math Operations/Bias', 'c_inc', 'Bias', '1');  L(n, 'c_z/1', 'c_inc/1', 'c_prev');
C(n, 'Zero_c', '0', 'double', ts);
SW(n, 'c_next');  L(n, 'Zero_c/1', 'c_next/1');  L(n, 'KickOrReset/1', 'c_next/2');  L(n, 'c_inc/1', 'c_next/3');
L(n, 'c_next/1', 'c_z/1', 'c');
CMP(n, 'Timeout', '>', Nto);             L(n, 'c_next/1', 'Timeout/1');
LOGIC(n, 'NotInReset', 'NOT', 1);        L(n, 'InReset/1', 'NotInReset/1');
LOGIC(n, 'Trip', 'AND', 2);              L(n, 'Timeout/1', 'Trip/1', 'timeout');  L(n, 'NotInReset/1', 'Trip/2');
CMP(n, 'BootRunning', '<', Nboot);       L(n, 'r_z/1', 'BootRunning/1');
LOGIC(n, 'Continue', 'AND', 2);          L(n, 'InReset/1', 'Continue/1');  L(n, 'BootRunning/1', 'Continue/2');
A(n, 'simulink/Math Operations/Bias', 'r_inc', 'Bias', '1');  L(n, 'r_z/1', 'r_inc/1');
C(n, 'Zero_r', '0', 'double', ts);
C(n, 'One_r', '1', 'double', ts);
SW(n, 'r_cont');  L(n, 'r_inc/1', 'r_cont/1');  L(n, 'Continue/1', 'r_cont/2');  L(n, 'Zero_r/1', 'r_cont/3');
SW(n, 'r_next');  L(n, 'One_r/1', 'r_next/1');  L(n, 'Trip/1', 'r_next/2', 'trip');  L(n, 'r_cont/1', 'r_next/3');
L(n, 'r_next/1', 'r_z/1', 'r');
CMP(n, 'Reset', '>=', '1');              L(n, 'r_next/1', 'Reset/1');
L(n, 'Reset/1', 'mcu_reset/1', 'mcu_reset');
CMP(n, 'Release', '==', Nboot);          L(n, 'r_z/1', 'Release/1');
L(n, 'Release/1', 'reset_flag/1', 'reset_flag');
local_finish(n, f, sprintf(['elp_watchdog_reset (T10, Electronics_Plant/Watchdog_Reset_Sim, 1 ms). in 1 wdg_kick ' ...
    'boolean 0.001; out 1 mcu_reset boolean, 2 reset_flag boolean (0.001). IWDG1 emulation (electronics 2.9.2): ' ...
    'c = ticks without wdg_kick (cleared by a kick and while in reset); c > round(P.electronics.wdg.timeout/Ts.base) ' ...
    '-> mcu_reset for round(P.electronics.wdg.t_boot/Ts.base) ticks (r counter), then reset_flag is true for ' ...
    'exactly one tick (release tick). Blocks: Unit Delay, Bias, Switch, Compare To Constant, Logical Operator.']));
end

% ===========================================================================================
%  elp_hw_frontend (DI 1 ms, ADC 0.05 s)
% ===========================================================================================
function local_elp_hw_frontend(n, f)
shaddad_bu('fresh', n, f, 'ssr');
IN(n, 'power_status', 1, 'Bus: Bus_PowerStatus', 'Ts.base');
IN(n, 'hmi', 2, 'Bus: Bus_HmiInputs', 'Ts.base');
IN(n, 'impl_mech', 3, 'Bus: Bus_ImplMechState', 'Ts.ctrl');
IN(n, 'T_encl_K', 4, 'double', 'Ts.hk');
OUT(n, 'di_word', 1, 'uint16');
OUT(n, 'elec_adc', 2, 'Bus: Bus_ElecAdc');
% ---- DI word (ICD 6): a Bus Selector in front of each consumer ----
BS(n, 'Sel_ps_di', {'EStop_Ch1_OK', 'EStop_Ch2_OK', 'Safety_OK', 'K1_Closed', 'K2_Closed', 'Key_On', ...
    'Safety_HV_OK', 'Dock_Present'});
L(n, 'power_status/1', 'Sel_ps_di/1', 'power_status');
BS(n, 'Sel_hmi_di', {'bumper_front', 'bumper_rear', 'reset_btn'});
L(n, 'hmi/1', 'Sel_hmi_di/1', 'hmi');
BS(n, 'Sel_hmi_lid', {'lid_open'});  L(n, 'hmi/1', 'Sel_hmi_lid/1');
LOGIC(n, 'LidClosed', 'NOT', 1);  L(n, 'Sel_hmi_lid/1', 'LidClosed/1');
% implement coding plug: ID = VC_IMPL (b8 = ID0, b9 = ID1)
C(n, 'ImplId0', 'mod(VC_IMPL, 2) ~= 0', 'boolean', 'Ts.base');
C(n, 'ImplId1', 'mod(floor(VC_IMPL/2), 2) ~= 0', 'boolean', 'Ts.base');
C(n, 'Spare', 'false(1, 2)', 'boolean', 'Ts.base');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'DI_bits', 'NumInputs', '15');
src = {'Sel_ps_di/1', 'Sel_ps_di/2', 'Sel_ps_di/3', 'Sel_hmi_di/1', 'Sel_hmi_di/2', 'Sel_hmi_di/3', 'Sel_ps_di/4', ...
    'Sel_ps_di/5', 'ImplId0/1', 'ImplId1/1', 'LidClosed/1', 'Sel_ps_di/6', 'Sel_ps_di/7', 'Sel_ps_di/8', 'Spare/1'};
for k = 1:numel(src), L(n, src{k}, sprintf('DI_bits/%d', k)); end
A(n, 'commutil2/Bit to Integer Converter', 'DI_pack', 'nbits', '16', 'bitOrder', 'LSB first', ...
    'signedOutputValues', 'Unsigned', 'outDtype', 'uint16');
L(n, 'DI_bits/1', 'DI_pack/1', 'di_bits');
L(n, 'DI_pack/1', 'di_word/1', 'di_word');
% ---- ADC pin voltages ----
BS(n, 'Sel_ps_adc', {'V_pack_V', 'V_bus_V', 'I_pack_A', 'V_24V_V', 'V_12V_V', 'V_5V_V'});
L(n, 'power_status/1', 'Sel_ps_adc/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'V6', 'NumInputs', '6');
for k = 1:6, L(n, sprintf('Sel_ps_adc/%d', k), sprintf('V6/%d', k)); end
A(n, 'simulink/Discrete/Zero-Order Hold', 'ZOH_adc', 'SampleTime', 'Ts.comm');
L(n, 'V6/1', 'ZOH_adc/1', 'v_meas');
A(n, 'simulink/Math Operations/Gain', 'Divider', 'Multiplication', 'Element-wise(K.*u)', 'Gain', ...
    ['[1/P.electronics.adc.k_vpack 1/P.electronics.adc.k_vlink P.electronics.adc.i_sens ' ...
     '1/P.electronics.adc.k_v24 1/P.electronics.adc.k_v12 1/P.electronics.adc.k_v5]']);
L(n, 'ZOH_adc/1', 'Divider/1');
C(n, 'HallOffset', '[0 0 P.electronics.adc.i_off 0 0 0]', 'double', 'Ts.comm');
A(n, 'simulink/Math Operations/Sum', 'Pin6', 'Inputs', '++');
L(n, 'Divider/1', 'Pin6/1');  L(n, 'HallOffset/1', 'Pin6/2');
% enclosure NTC divider: R = R25 exp(B (1/T - 1/298.15)), V = vref R / (R + R_pu)
RT(n, 'RT_T', 'Ts.comm', 'P.electronics.therm.T0');
L(n, 'T_encl_K/1', 'RT_T/1', 'T_encl_K');
A(n, 'simulink/Math Operations/Math Function', 'InvT', 'Operator', 'reciprocal');  L(n, 'RT_T/1', 'InvT/1');
A(n, 'simulink/Math Operations/Bias', 'dInvT', 'Bias', '-1/298.15');  L(n, 'InvT/1', 'dInvT/1');
A(n, 'simulink/Math Operations/Gain', 'Bcoef', 'Gain', 'P.electronics.adc.ntc_B');  L(n, 'dInvT/1', 'Bcoef/1');
A(n, 'simulink/Math Operations/Math Function', 'ExpB', 'Operator', 'exp');  L(n, 'Bcoef/1', 'ExpB/1');
A(n, 'simulink/Math Operations/Gain', 'R_ntc', 'Gain', 'P.electronics.adc.ntc_R25');  L(n, 'ExpB/1', 'R_ntc/1');
A(n, 'simulink/Math Operations/Bias', 'R_sum', 'Bias', 'P.electronics.adc.ntc_Rpu');  L(n, 'R_ntc/1', 'R_sum/1', 'R_ntc');
A(n, 'simulink/Math Operations/Divide', 'Ratio', 'Inputs', '*/');
L(n, 'R_ntc/1', 'Ratio/1');  L(n, 'R_sum/1', 'Ratio/2');
A(n, 'simulink/Math Operations/Gain', 'V_ntc', 'Gain', 'P.electronics.adc.vref');  L(n, 'Ratio/1', 'V_ntc/1');
% lift potentiometer 0..3.3 V
BS(n, 'Sel_impl', {'liftPos01'});  L(n, 'impl_mech/1', 'Sel_impl/1', 'impl_mech');
RT(n, 'RT_lift', 'Ts.comm', '0');  L(n, 'Sel_impl/1', 'RT_lift/1');
A(n, 'simulink/Math Operations/Gain', 'V_lift', 'Gain', 'P.electronics.adc.vref');  L(n, 'RT_lift/1', 'V_lift/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'V8', 'NumInputs', '3');
L(n, 'Pin6/1', 'V8/1', 'v_pin6');  L(n, 'V_ntc/1', 'V8/2', 'v_ntc');  L(n, 'V_lift/1', 'V8/3', 'v_lift');
% counts = sat(round(V/vref*65535 + n), 0, 65535), n ~ N(0, noise_lsb^2), seeds seed + k
A(n, 'simulink/Math Operations/Gain', 'ToCounts', 'Gain', '(2^P.electronics.adc.nbits - 1)/P.electronics.adc.vref');
L(n, 'V8/1', 'ToCounts/1', 'v_pin');
A(n, 'simulink/Sources/Random Number', 'Noise', 'Mean', 'zeros(1, 8)', 'Variance', 'P.electronics.adc.noise_lsb^2', ...
    'Seed', 'P.electronics.adc.seed + (1:8)', 'SampleTime', 'Ts.comm');
A(n, 'simulink/Math Operations/Sum', 'AddNoise', 'Inputs', '++');
L(n, 'ToCounts/1', 'AddNoise/1');  L(n, 'Noise/1', 'AddNoise/2', 'adc_noise');
A(n, 'simulink/Math Operations/Rounding Function', 'Round', 'Operator', 'round');  L(n, 'AddNoise/1', 'Round/1');
A(n, 'simulink/Discontinuities/Saturation', 'Clip', 'UpperLimit', '2^P.electronics.adc.nbits - 1', 'LowerLimit', '0');
L(n, 'Round/1', 'Clip/1');
DTC(n, 'Counts', 'uint16');  L(n, 'Clip/1', 'Counts/1');
% MCU die temperature = T_encl + dT_mcu
A(n, 'simulink/Math Operations/Bias', 'T_mcu', 'Bias', '-273.15 + P.electronics.therm.dT_mcu');  L(n, 'RT_T/1', 'T_mcu/1');
DTC(n, 'T_mcu_s', 'single');  L(n, 'T_mcu/1', 'T_mcu_s/1');
BC(n, 'BC_adc', 'Bus_ElecAdc', {'Counts/1', 'T_mcu_s/1'});
L(n, 'BC_adc/1', 'elec_adc/1', 'elec_adc');
local_finish(n, f, sprintf(['elp_hw_frontend (T10, Electronics_Plant/HW_Frontend_Sim). in 1 power_status ' ...
    'Bus_PowerStatus 0.001, 2 hmi Bus_HmiInputs 0.001, 3 impl_mech Bus_ImplMechState 0.01, 4 T_encl_K double 0.1; ' ...
    'out 1 di_word uint16 0.001, 2 elec_adc Bus_ElecAdc 0.05. DI (ICD 6): b0 EStop_Ch1_OK, b1 EStop_Ch2_OK, b2 ' ...
    'Safety_OK, b3/b4 bumpers, b5 reset_btn, b6/b7 K1/K2_Closed, b8/b9 implement ID = VC_IMPL, b10 NOT lid_open, ' ...
    'b11 Key_On, b12 Safety_HV_OK, b13 Dock_Present, b14/b15 0 -> commutil2 Bit to Integer Converter (16, LSB ' ...
    'first). ADC (electronics 2.9.5): Zero-Order Hold 0.05; pins [Vpack/21 Vbus/21 1.65+4.1667e-3*I V24/11 ' ...
    'V12/5.7 V5/2 NTC divider (R25 10k, B 3950, 10k pull-up) liftPos01*3.3]; counts = sat(round(V/3.3*65535 + n), ' ...
    '0, 65535), n = Random Number N(0, noise_lsb^2), seeds P.electronics.adc.seed + (1:8). mcu_temp_degC = ' ...
    'T_encl - 273.15 + P.electronics.therm.dT_mcu (single).']));
end

% ===========================================================================================
%  elp_actuators (1 ms DO, 0.01 PWM, 0.1 indicators)
% ===========================================================================================
function local_elp_actuators(n, f)
shaddad_bu('fresh', n, f, 'ssr');
IN(n, 'do_word', 1, 'uint32', 'Ts.base');
IN(n, 'pwm_duty', 2, 'single', 'Ts.ctrl', '4');
IN(n, 'mcu_reset', 3, 'boolean', 'Ts.base');
OUT(n, 'hw_cmd', 1, 'Bus: Bus_PowerHwCmd');
OUT(n, 'impl_cmd', 2, 'Bus: Bus_ImplementCmd');
OUT(n, 'indicators', 3, 'Bus: Bus_IndicatorOut');
OUT(n, 'do_bits', 4, 'boolean');
OUT(n, 'pwm', 5, 'double');
d = @(fld) sprintf('1 + P.electronics.do.%s', fld);
% ---- DO bits, forced 0 while the MCU is held in reset (1 ms) ----
A(n, 'commutil2/Integer to Bit Converter', 'DO_unpack', 'nbits', '32', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
L(n, 'do_word/1', 'DO_unpack/1', 'do_word');
LOGIC(n, 'NotReset', 'NOT', 1);  L(n, 'mcu_reset/1', 'NotReset/1', 'mcu_reset');
LOGIC(n, 'DO_gate', 'AND', 2);   L(n, 'DO_unpack/1', 'DO_gate/1', 'do_raw');  L(n, 'NotReset/1', 'DO_gate/2');
SEL(n, 'DO0', d('mcu_safety_ok'), 32);      L(n, 'DO_gate/1', 'DO0/1', 'do_g');
SEL(n, 'DO1', d('safety_reset_req'), 32);   L(n, 'DO_gate/1', 'DO1/1');
SEL(n, 'DO2', d('impl24_en'), 32);          L(n, 'DO_gate/1', 'DO2/1');
SEL(n, 'Nozzles', [d('nozzle1') ' + (0:7)'], 32);  L(n, 'DO_gate/1', 'Nozzles/1');
% ---- 0.01 s: DO bits, PWM gated ----
RT(n, 'RT_do10', 'Ts.ctrl', 'false');   L(n, 'DO_gate/1', 'RT_do10/1');
L(n, 'RT_do10/1', 'do_bits/1', 'do_bits');
RT(n, 'RT_rst10', 'Ts.ctrl', 'false');  L(n, 'mcu_reset/1', 'RT_rst10/1');
DTC(n, 'PWM_d', 'double');  L(n, 'pwm_duty/1', 'PWM_d/1', 'pwm_duty');
C(n, 'PWM_zero', 'zeros(1, 4)', 'double', 'Ts.ctrl');
SW(n, 'PWM_gate');  L(n, 'PWM_zero/1', 'PWM_gate/1');  L(n, 'RT_rst10/1', 'PWM_gate/2', 'mcu_reset_10');
L(n, 'PWM_d/1', 'PWM_gate/3');
L(n, 'PWM_gate/1', 'pwm/1', 'pwm');
% lift duty = pwm(2) signed by DO18 LIFT_DIR (1 = down = +)
SEL(n, 'PumpDuty', '1', 4);   L(n, 'PWM_gate/1', 'PumpDuty/1');
SEL(n, 'LiftMag', '2', 4);    L(n, 'PWM_gate/1', 'LiftMag/1');
SEL(n, 'LiftDir', d('lift_dir'), 32);  L(n, 'RT_do10/1', 'LiftDir/1');
A(n, 'simulink/Math Operations/Gain', 'LiftNeg', 'Gain', '-1');  L(n, 'LiftMag/1', 'LiftNeg/1');
SW(n, 'LiftDuty');  L(n, 'LiftMag/1', 'LiftDuty/1');  L(n, 'LiftDir/1', 'LiftDuty/2', 'lift_dir');
L(n, 'LiftNeg/1', 'LiftDuty/3');
% ---- Bus_PowerHwCmd (1 ms): PWM values re-gated at 1 ms so all outputs are 0 during the reset ----
RT(n, 'RT_pump1k', 'Ts.base', '0');  L(n, 'PumpDuty/1', 'RT_pump1k/1', 'pump_duty');
RT(n, 'RT_lift1k', 'Ts.base', '0');  L(n, 'LiftDuty/1', 'RT_lift1k/1', 'lift_duty');
C(n, 'Zero1k', '0', 'double', 'Ts.base');
SW(n, 'Pump1k');  L(n, 'Zero1k/1', 'Pump1k/1');  L(n, 'mcu_reset/1', 'Pump1k/2');  L(n, 'RT_pump1k/1', 'Pump1k/3');
SW(n, 'Lift1k');  L(n, 'Zero1k/1', 'Lift1k/1');  L(n, 'mcu_reset/1', 'Lift1k/2');  L(n, 'RT_lift1k/1', 'Lift1k/3');
BC(n, 'BC_hw', 'Bus_PowerHwCmd', {'DO0/1', 'DO1/1', 'DO2/1', 'Pump1k/1', 'Nozzles/1', 'Lift1k/1'});
L(n, 'BC_hw/1', 'hw_cmd/1', 'hw_cmd');
% ---- Bus_ImplementCmd (0.01) ----
SEL(n, 'Nozzles10', [d('nozzle1') ' + (0:7)'], 32);  L(n, 'RT_do10/1', 'Nozzles10/1');
C(n, 'SpeedCmd', '0', 'double', 'Ts.ctrl');
BC(n, 'BC_impl', 'Bus_ImplementCmd', {'Nozzles10/1', 'LiftDuty/1', 'SpeedCmd/1'});
L(n, 'BC_impl/1', 'impl_cmd/1', 'impl_cmd');
% ---- Bus_IndicatorOut (0.1) ----
RT(n, 'RT_do100', 'Ts.hk', 'false');  L(n, 'RT_do10/1', 'RT_do100/1');
RT(n, 'RT_pwm100', 'Ts.hk', '0');     L(n, 'PWM_gate/1', 'RT_pwm100/1');
ind = {'BeaconG', 'beacon_green'; 'BeaconA', 'beacon_amber'; 'BeaconR', 'beacon_red'; 'Buzzer', 'buzzer'; ...
    'Lights', 'work_lights'};
for k = 1:size(ind, 1)
    SEL(n, ind{k, 1}, d(ind{k, 2}), 32);  L(n, 'RT_do100/1', [ind{k, 1} '/1']);
end
SEL(n, 'Fans', '[3 4]', 4);  L(n, 'RT_pwm100/1', 'Fans/1');
BC(n, 'BC_ind', 'Bus_IndicatorOut', {'BeaconG/1', 'BeaconA/1', 'BeaconR/1', 'Buzzer/1', 'Lights/1', 'Fans/1'});
L(n, 'BC_ind/1', 'indicators/1', 'indicators');
local_finish(n, f, sprintf(['elp_actuators (T10, Electronics_Plant/Actuator_Outputs_Sim). in 1 do_word uint32 0.001, ' ...
    '2 pwm_duty single[4] 0.01, 3 mcu_reset boolean 0.001; out 1 hw_cmd Bus_PowerHwCmd 0.001, 2 impl_cmd ' ...
    'Bus_ImplementCmd 0.01, 3 indicators Bus_IndicatorOut 0.1, 4 do_bits boolean[32] 0.01 (gated DO, bit k at ' ...
    'index k+1), 5 pwm double[4] 0.01 (gated). commutil2 Integer to Bit Converter (32, LSB first); DO and PWM ' ...
    'forced 0 while mcu_reset (also re-gated at 1 ms on hw_cmd). Bus_PowerHwCmd: DO0 MCU_Safety_OK, DO1 ' ...
    'Safety_Reset_Req, DO2 ImpPwr24_Enable, pwm(1) Spray_Pump_Duty, DO10..17 Spray_Nozzle_Cmd, Lift_Duty = ' ...
    'pwm(2) signed by DO18 (1 = down = +). Bus_ImplementCmd: nozzleOn = DO10..17, liftDuty, speedCmd 0. ' ...
    'Bus_IndicatorOut: DO3..7, fan_duty = pwm(3:4).']));
end

% ===========================================================================================
%  elp_lv_demand (0.01, D61)
% ===========================================================================================
function local_elp_lv_demand(n, f)
shaddad_bu('fresh', n, f, 'ssr');
ts = 'Ts.ctrl';
IN(n, 'do_bits', 1, 'boolean', ts, '32');
IN(n, 'pwm', 2, 'double', ts, '4');
OUT(n, 'lv_demand', 1, 'Bus: Bus_LV_LoadDemand');
OUT(n, 'P_inbox_W', 2, 'double');
d = @(fld) sprintf('1 + P.electronics.do.%s', fld);
% 24 V: beacon (any colour or buzzer) + 2 work lights + 3D LiDAR (LIDAR_PWR_EN)
SEL(n, 'BeaconBits', ['[' d('beacon_green') ', ' d('beacon_amber') ', ' d('beacon_red') ', ' d('buzzer') ']'], 32);
L(n, 'do_bits/1', 'BeaconBits/1', 'do_bits');
LOGIC(n, 'BeaconOn', 'OR', 1);  L(n, 'BeaconBits/1', 'BeaconOn/1');
SEL(n, 'LightsBit', d('work_lights'), 32);   L(n, 'do_bits/1', 'LightsBit/1');
SEL(n, 'LidarBit', d('lidar_pwr_en'), 32);   L(n, 'do_bits/1', 'LidarBit/1');
SEL(n, 'CompBit', d('companion_pwr_en'), 32);  L(n, 'do_bits/1', 'CompBit/1');
gains = {'BeaconOn', 'P_beacon', 'P.electronics.load.beacon'; 'LightsBit', 'P_lights', '2*P.electronics.load.lights_each'; ...
    'LidarBit', 'P_lidar', 'P.electronics.load.lidar3d'; 'CompBit', 'P_jetson', 'P.electronics.load.jetson(2)'};
for k = 1:size(gains, 1)
    DTC(n, ['D_' gains{k, 1}], 'double');  L(n, [gains{k, 1} '/1'], ['D_' gains{k, 1} '/1']);
    A(n, 'simulink/Math Operations/Gain', gains{k, 2}, 'Gain', gains{k, 3});
    L(n, ['D_' gains{k, 1} '/1'], [gains{k, 2} '/1']);
end
A(n, 'simulink/Math Operations/Sum', 'P24', 'Inputs', '+++');
L(n, 'P_beacon/1', 'P24/1', 'P_beacon');  L(n, 'P_lights/1', 'P24/2', 'P_lights');  L(n, 'P_lidar/1', 'P24/3', 'P_lidar');
% 12 V: MCU board + Jetson (COMPANION_PWR_EN) + modem + cameras + display + enclosure fans
SEL(n, 'FanDuty', '[3 4]', 4);  L(n, 'pwm/1', 'FanDuty/1', 'pwm');
A(n, 'simulink/Math Operations/Sum', 'FanSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(n, 'FanDuty/1', 'FanSum/1');
A(n, 'simulink/Math Operations/Gain', 'P_fans', 'Gain', 'P.electronics.load.fan_each');  L(n, 'FanSum/1', 'P_fans/1');
C(n, 'P12_fixed', ['P.electronics.load.mcu + P.electronics.load.modem + P.electronics.load.cams + ' ...
    'P.electronics.load.display'], 'double', ts);
A(n, 'simulink/Math Operations/Sum', 'P12', 'Inputs', '+++');
L(n, 'P12_fixed/1', 'P12/1');  L(n, 'P_jetson/1', 'P12/2', 'P_jetson');  L(n, 'P_fans/1', 'P12/3', 'P_fans');
% 5 V: RC receiver + GNSS receivers; 3.3 V: IMU + CAN transceiver logic
C(n, 'P5', 'P.electronics.load.rc_rx + P.electronics.load.gnss', 'double', ts);
C(n, 'P3V3', 'P.electronics.load.imu + P.electronics.load.can_xcvr', 'double', ts);
BC(n, 'BC_lv', 'Bus_LV_LoadDemand', {'P24/1', 'P12/1', 'P5/1', 'P3V3/1'});
L(n, 'BC_lv/1', 'lv_demand/1', 'lv_demand');
% in-box heat of the loads (MCU board, Jetson)
C(n, 'Pin_mcu', 'P.electronics.load.mcu*double(P.electronics.load.inbox_mcu)', 'double', ts);
A(n, 'simulink/Math Operations/Gain', 'Pin_jetson', 'Gain', 'double(P.electronics.load.inbox_jetson)');
L(n, 'P_jetson/1', 'Pin_jetson/1');
A(n, 'simulink/Math Operations/Sum', 'P_inbox', 'Inputs', '++');
L(n, 'Pin_mcu/1', 'P_inbox/1');  L(n, 'Pin_jetson/1', 'P_inbox/2');
L(n, 'P_inbox/1', 'P_inbox_W/1', 'P_inbox_W');
local_finish(n, f, sprintf(['elp_lv_demand (T10, Electronics_Plant/LV_Load_Demand, 0.01 s, D61). in 1 do_bits ' ...
    'boolean[32] (gated DO, bit k at index k+1), 2 pwm double[4]; out 1 lv_demand Bus_LV_LoadDemand, 2 P_inbox_W ' ...
    'double (in-box heat of the loads). P.electronics.load: P_24V = beacon*(DO3|DO4|DO5|DO6) + 2*lights_each*DO7 + ' ...
    'lidar3d*DO9 (LIDAR_PWR_EN); P_12V = mcu + jetson(2)*DO8 (COMPANION_PWR_EN) + modem + cams + display + ' ...
    'fan_each*(pwm3 + pwm4); P_5V = rc_rx + gnss; P_3V3 = imu + can_xcvr. P_inbox_W = mcu*inbox_mcu + ' ...
    'jetson(2)*DO8*inbox_jetson.']));
end

% ===========================================================================================
%  elp_thermal (own Simscape thermal network, local BE P.sim.ssc.Ts_encl)
% ===========================================================================================
function local_elp_thermal(n, f)
shaddad_bu('fresh', n, f, 'ssr');
ts = 'Ts.hk';
IN(n, 'P_inbox_W', 1, 'double', 'Ts.ctrl');
IN(n, 'pwm', 2, 'double', 'Ts.ctrl', '4');
IN(n, 'env', 3, 'Bus: Bus_EnvState', ts);
OUT(n, 'T_encl_K', 1, 'double');
RT(n, 'RT_P', ts, '0');    L(n, 'P_inbox_W/1', 'RT_P/1', 'P_inbox_W');
RT(n, 'RT_pwm', ts, '0');  L(n, 'pwm/1', 'RT_pwm/1', 'pwm');
% a Bus Selector in front of each consumer of env: Sel_env (T_amb) -> S_Tamb, Sel_G_solar -> Q_solar
BS(n, 'Sel_env', {'T_amb'});  L(n, 'env/1', 'Sel_env/1', 'env');
BS(n, 'Sel_G_solar', {'G_solar'});  L(n, 'env/1', 'Sel_G_solar/1');
% heat input: in-box loads + P_inbox_extra + alpha_s*A_s*G_solar
A(n, 'simulink/Math Operations/Gain', 'Q_solar', 'Gain', 'P.electronics.therm.alpha_s*P.electronics.therm.A_s');
L(n, 'Sel_G_solar/1', 'Q_solar/1');
C(n, 'Q_extra', 'P.electronics.load.P_inbox_extra', 'double', ts);
A(n, 'simulink/Math Operations/Sum', 'Q_total', 'Inputs', '+++');
L(n, 'RT_P/1', 'Q_total/1');  L(n, 'Q_extra/1', 'Q_total/2');  L(n, 'Q_solar/1', 'Q_total/3', 'Q_solar');
% convection coefficient h = h_nat + h_fan*mean(fan duty)
SEL(n, 'FanDuty', '[3 4]', 4);  L(n, 'RT_pwm/1', 'FanDuty/1');
A(n, 'simulink/Math Operations/Sum', 'FanSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(n, 'FanDuty/1', 'FanSum/1');
A(n, 'simulink/Math Operations/Gain', 'h_fan', 'Gain', '0.5*P.electronics.therm.h_fan');  L(n, 'FanSum/1', 'h_fan/1');
A(n, 'simulink/Math Operations/Bias', 'h_conv', 'Bias', 'P.electronics.therm.h_nat');  L(n, 'h_fan/1', 'h_conv/1');
% ---- Simscape thermal network ----
sps = {'S_Q', 'W'; 'S_h', 'W/(m^2*K)'; 'S_Tamb', 'K'};
for k = 1:size(sps, 1)
    A(n, 'nesl_utility/Simulink-PS Converter', sps{k, 1}, 'Unit', sps{k, 2}, 'FilteringAndDerivatives', 'zero');
end
L(n, 'Q_total/1', 'S_Q/1', 'Q_in');  L(n, 'h_conv/1', 'S_h/1', 'h');  L(n, 'Sel_env/1', 'S_Tamb/1');
A(n, 'fl_lib/Thermal/Thermal Elements/Thermal Mass', 'Enclosure', 'mass', 'P.electronics.therm.mass', ...
    'mass_unit', 'kg', 'sp_heat', 'P.electronics.therm.cp', 'sp_heat_unit', 'J/(kg*K)', 'T_specify', 'on', ...
    'T_priority', 'High', 'T', 'P.electronics.therm.T0', 'T_unit', 'K');
A(n, sprintf('fl_lib/Thermal/Thermal Elements/Convective Heat\nTransfer'), 'Convection', ...
    'thermal_type', 'foundation.enum.constant_variable.variable');
set_param([n '/Convection'], 'area', 'P.electronics.therm.area', 'area_unit', 'm^2', 'min_heat_tr_coeff', '1e-3');
A(n, sprintf('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow\nRate Source'), 'HeatIn');
A(n, sprintf('fl_lib/Thermal/Thermal Sources/Controlled\nTemperature Source'), 'Ambient');
A(n, 'fl_lib/Thermal/Thermal Sensors/Temperature Sensor', 'T_sensor');
for k = 1:3, A(n, 'fl_lib/Thermal/Thermal Elements/Thermal Reference', sprintf('Ref%d', k)); end
A(n, 'nesl_utility/Solver Configuration', 'Solver_Encl', 'UseLocalSolver', 'on', ...
    'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', 'LocalSolverSampleTime', 'P.sim.ssc.Ts_encl', 'DoFixedCost', 'on');
A(n, 'nesl_utility/PS-Simulink Converter', 'PS_T', 'Unit', 'K');
ph = @(b) get_param([n '/' b], 'PortHandles');
M = ph('Enclosure'); Cv = ph('Convection'); Q = ph('HeatIn'); Ta = ph('Ambient'); S = ph('T_sensor');
R1 = ph('Ref1'); R2 = ph('Ref2'); R3 = ph('Ref3'); SC = ph('Solver_Encl');
SQ = ph('S_Q'); SH = ph('S_h'); ST = ph('S_Tamb'); PT = ph('PS_T');
% port order verified in R2026b: Convective [A h | B]; sources [B-side | S, A-side]; sensor [A | B, T]
PL(n, M.LConn(1), Cv.LConn(1));
PL(n, SH.RConn(1), Cv.LConn(2));
PL(n, Cv.RConn(1), Ta.LConn(1));
PL(n, ST.RConn(1), Ta.RConn(1));
PL(n, Ta.RConn(2), R1.LConn(1));
PL(n, SQ.RConn(1), Q.RConn(1));
PL(n, Q.LConn(1), M.LConn(1));
PL(n, Q.RConn(2), R2.LConn(1));
PL(n, S.LConn(1), M.LConn(1));
PL(n, S.RConn(1), R3.LConn(1));
PL(n, S.RConn(2), PT.LConn(1));
PL(n, SC.RConn(1), R1.LConn(1));
% loop breaker: the local-solver network counts as direct feedthrough (D36); T_encl feeds back through
% the MCU (NTC ADC -> fan duty / power enables)
UD(n, 'T_delay', 'P.electronics.therm.T0', ts);
L(n, 'PS_T/1', 'T_delay/1', 'T_encl_ps');
L(n, 'T_delay/1', 'T_encl_K/1', 'T_encl_K');
local_finish(n, f, sprintf(['elp_thermal (T10, Electronics_Plant/Electronics_Thermal, electronics 2.9.4). in 1 ' ...
    'P_inbox_W double 0.01, 2 pwm double[4] 0.01 (fans = pwm 3:4), 3 env Bus_EnvState 0.1; out 1 T_encl_K double ' ...
    '0.1. Own Simscape thermal network with its own Solver Configuration (local Backward Euler, ' ...
    'P.sim.ssc.Ts_encl, fixed cost): Thermal Mass (P.electronics.therm.mass*cp, T0), Convective Heat Transfer ' ...
    '(variable h = h_nat + h_fan*mean(fan duty), area), Controlled Heat Flow Rate Source (P_inbox_W + ' ...
    'P.electronics.load.P_inbox_extra + alpha_s*A_s*G_solar), Controlled Temperature Source (env.T_amb), ' ...
    'Temperature Sensor, Thermal Reference. Inputs re-timed to 0.1 s; T_encl leaves through a 0.1 s Unit Delay ' ...
    '(explicit loop breaker, D36: T_encl -> NTC ADC -> MCU fan duty / power enables -> heat input).']));
end

% ===========================================================================================
%  elp_elec_env (0.1)
% ===========================================================================================
function local_elp_elec_env(n, f)
shaddad_bu('fresh', n, f, 'ssr');
ts = 'Ts.hk';
IN(n, 'T_encl_K', 1, 'double', ts);
IN(n, 'power_status', 2, 'Bus: Bus_PowerStatus', 'Ts.base');
IN(n, 'flt', 3, 'Bus: Bus_FaultInject', 'Ts.base');
IN(n, 'do_bits', 4, 'boolean', 'Ts.ctrl', '32');
OUT(n, 'elec_env', 1, 'Bus: Bus_ElecEnv');
A(n, 'simulink/Math Operations/Bias', 'T_imu', 'Bias', '-273.15 + P.electronics.therm.dT_imu');
L(n, 'T_encl_K/1', 'T_imu/1', 'T_encl_K');
BS(n, 'Sel_ps', {'Rail_24V_OK', 'Rail_12V_OK', 'Rail_5V_OK', 'Rail_3V3_OK'});
L(n, 'power_status/1', 'Sel_ps/1', 'power_status');
BS(n, 'Sel_flt', {'gnss_corr_loss'});  L(n, 'flt/1', 'Sel_flt/1', 'flt');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Cat1k', 'NumInputs', '5');
for k = 1:4, L(n, sprintf('Sel_ps/%d', k), sprintf('Cat1k/%d', k)); end
L(n, 'Sel_flt/1', 'Cat1k/5');
RT(n, 'RT_1k', ts, 'false');  L(n, 'Cat1k/1', 'RT_1k/1', 'rails_corr');
SEL(n, 'Rail24', '1', 5);  SEL(n, 'Rail12', '2', 5);  SEL(n, 'Rail5', '3', 5);  SEL(n, 'Rail3V3', '4', 5);
SEL(n, 'CorrLoss', '5', 5);
for b = {'Rail24', 'Rail12', 'Rail5', 'Rail3V3', 'CorrLoss'}, L(n, 'RT_1k/1', [b{1} '/1']); end
SEL(n, 'LidarEn', '1 + P.electronics.do.lidar_pwr_en', 32);  L(n, 'do_bits/1', 'LidarEn/1', 'do_bits');
RT(n, 'RT_lidar', ts, 'false');  L(n, 'LidarEn/1', 'RT_lidar/1');
% RTK corrections: link up = NOT gnss_corr_loss AND Rail_12V_OK (NTRIP modem on 12 V, D61)
LOGIC(n, 'NotLoss', 'NOT', 1);  L(n, 'CorrLoss/1', 'NotLoss/1');
LOGIC(n, 'CorrOK', 'AND', 2);  L(n, 'NotLoss/1', 'CorrOK/1');  L(n, 'Rail12/1', 'CorrOK/2');
UD(n, 'Age_z', 'P.electronics.ntrip.corr_age_ok', ts);
A(n, 'simulink/Math Operations/Bias', 'Age_inc', 'Bias', 'Ts.hk');  L(n, 'Age_z/1', 'Age_inc/1');
C(n, 'AgeOK', 'P.electronics.ntrip.corr_age_ok', 'double', ts);
SW(n, 'CorrAge');  L(n, 'AgeOK/1', 'CorrAge/1');  L(n, 'CorrOK/1', 'CorrAge/2', 'gnssCorrOK');  L(n, 'Age_inc/1', 'CorrAge/3');
L(n, 'CorrAge/1', 'Age_z/1', 'gnssCorrAge');
% sensor power: b0 IMU (3.3 V), b1 GNSS (5 V), b2 LiDAR (24 V AND LIDAR_PWR_EN)
LOGIC(n, 'LidarPwr', 'AND', 2);  L(n, 'Rail24/1', 'LidarPwr/1');  L(n, 'RT_lidar/1', 'LidarPwr/2');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'PwrBits', 'NumInputs', '3');
L(n, 'Rail3V3/1', 'PwrBits/1');  L(n, 'Rail5/1', 'PwrBits/2');  L(n, 'LidarPwr/1', 'PwrBits/3');
A(n, 'commutil2/Bit to Integer Converter', 'PwrPack', 'nbits', '3', 'bitOrder', 'LSB first', ...
    'signedOutputValues', 'Unsigned', 'outDtype', 'uint8');
L(n, 'PwrBits/1', 'PwrPack/1');
BC(n, 'BC_env', 'Bus_ElecEnv', {'T_imu/1', 'T_encl_K/1', 'CorrOK/1', 'CorrAge/1', 'PwrPack/1'});
L(n, 'BC_env/1', 'elec_env/1', 'elec_env');
local_finish(n, f, sprintf(['elp_elec_env (T10, Electronics_Plant/Elec_Env, 0.1 s). in 1 T_encl_K double 0.1, 2 ' ...
    'power_status Bus_PowerStatus 0.001, 3 flt Bus_FaultInject 0.001, 4 do_bits boolean[32] 0.01; out 1 elec_env ' ...
    'Bus_ElecEnv 0.1. T_imu_degC = T_encl - 273.15 + dT_imu; gnssCorrOK = NOT flt.gnss_corr_loss AND Rail_12V_OK; ' ...
    'gnssCorrAge = P.electronics.ntrip.corr_age_ok while OK, integrates (+0.1 s per sample) while not OK; ' ...
    'sensPowerOK b0 IMU = Rail_3V3_OK, b1 GNSS = Rail_5V_OK, b2 LiDAR = Rail_24V_OK AND LIDAR_PWR_EN (DO9).']));
end

% ===========================================================================================
%  Electronics_Plant (ICD 3.2 Electronics row)
% ===========================================================================================
function local_Electronics_Plant(n, f)
shaddad_bu('fresh', n, f, 'ssr');
ins = {'mcu_out1k', 'Bus_McuOut1k', 'Ts.base'; 'mcu_out10', 'Bus_McuOut10', 'Ts.ctrl'; ...
    'power_status', 'Bus_PowerStatus', 'Ts.base'; 'hmi', 'Bus_HmiInputs', 'Ts.base'; 'env', 'Bus_EnvState', 'Ts.hk'; ...
    'impl_mech', 'Bus_ImplMechState', 'Ts.ctrl'; 'flt', 'Bus_FaultInject', 'Ts.base'};
for k = 1:size(ins, 1), IN(n, ins{k, 1}, k, ['Bus: ' ins{k, 2}], ins{k, 3}); end
outs = {'elec_hw', 'Bus_ElecHwIn'; 'elec_adc', 'Bus_ElecAdc'; 'elec_env', 'Bus_ElecEnv'; 'hw_cmd', 'Bus_PowerHwCmd'; ...
    'impl_cmd', 'Bus_ImplementCmd'; 'lv_demand', 'Bus_LV_LoadDemand'; 'indicators', 'Bus_IndicatorOut'};
for k = 1:size(outs, 1), OUT(n, outs{k, 1}, k, ['Bus: ' outs{k, 2}]); end
SSR(n, 'HW_Frontend_Sim', 'elp_hw_frontend');
SSR(n, 'Watchdog_Reset_Sim', 'elp_watchdog_reset');
SSR(n, 'Actuator_Outputs_Sim', 'elp_actuators');
SSR(n, 'LV_Load_Demand', 'elp_lv_demand');
SSR(n, 'Electronics_Thermal', 'elp_thermal');
SSR(n, 'Elec_Env', 'elp_elec_env');
% a Bus Selector in front of each consumer of an MCU output bus
BS(n, 'Sel_wdg_kick', {'wdg_kick'});  L(n, 'mcu_out1k/1', 'Sel_wdg_kick/1', 'mcu_out1k');
BS(n, 'Sel_do_word', {'do_word'});    L(n, 'mcu_out1k/1', 'Sel_do_word/1');
BS(n, 'Sel_pwm_duty', {'pwm_duty'});  L(n, 'mcu_out10/1', 'Sel_pwm_duty/1', 'mcu_out10');
% front end
L(n, 'power_status/1', 'HW_Frontend_Sim/1', 'power_status');
L(n, 'hmi/1', 'HW_Frontend_Sim/2', 'hmi');
L(n, 'impl_mech/1', 'HW_Frontend_Sim/3', 'impl_mech');
L(n, 'Electronics_Thermal/1', 'HW_Frontend_Sim/4', 'T_encl_K');
L(n, 'HW_Frontend_Sim/2', 'elec_adc/1', 'elec_adc');
% watchdog / reset
L(n, 'Sel_wdg_kick/1', 'Watchdog_Reset_Sim/1');
BC(n, 'BC_elec_hw', 'Bus_ElecHwIn', {'HW_Frontend_Sim/1', 'Watchdog_Reset_Sim/2', 'Watchdog_Reset_Sim/1'});
UD(n, 'DI_Latency', 'P.ic.Bus_ElecHwIn', 'Ts.base');
L(n, 'BC_elec_hw/1', 'DI_Latency/1', 'elec_hw_fe');
L(n, 'DI_Latency/1', 'elec_hw/1', 'elec_hw');
% actuators
L(n, 'Sel_do_word/1', 'Actuator_Outputs_Sim/1');
L(n, 'Sel_pwm_duty/1', 'Actuator_Outputs_Sim/2');
L(n, 'Watchdog_Reset_Sim/1', 'Actuator_Outputs_Sim/3', 'mcu_reset');
UD(n, 'HwCmd_Delay', 'P.ic.Bus_PowerHwCmd', 'Ts.base');
L(n, 'Actuator_Outputs_Sim/1', 'HwCmd_Delay/1', 'hw_cmd_fe');
L(n, 'HwCmd_Delay/1', 'hw_cmd/1', 'hw_cmd');
L(n, 'Actuator_Outputs_Sim/2', 'impl_cmd/1', 'impl_cmd');
L(n, 'Actuator_Outputs_Sim/3', 'indicators/1', 'indicators');
% LV demand, thermal, sensor environment
L(n, 'Actuator_Outputs_Sim/4', 'LV_Load_Demand/1', 'do_bits');
L(n, 'Actuator_Outputs_Sim/5', 'LV_Load_Demand/2', 'pwm');
L(n, 'LV_Load_Demand/1', 'lv_demand/1', 'lv_demand');
L(n, 'LV_Load_Demand/2', 'Electronics_Thermal/1', 'P_inbox_W');
L(n, 'Actuator_Outputs_Sim/5', 'Electronics_Thermal/2');
L(n, 'env/1', 'Electronics_Thermal/3', 'env');
L(n, 'Electronics_Thermal/1', 'Elec_Env/1');
L(n, 'power_status/1', 'Elec_Env/2');
L(n, 'flt/1', 'Elec_Env/3', 'flt');
L(n, 'Actuator_Outputs_Sim/4', 'Elec_Env/4');
L(n, 'Elec_Env/1', 'elec_env/1', 'elec_env');
local_finish(n, f, sprintf(['Electronics_Plant (T10, ICD 3.2 Electronics row, no physical ports). in 1 mcu_out1k ' ...
    'Bus_McuOut1k 0.001, 2 mcu_out10 Bus_McuOut10 0.01, 3 power_status Bus_PowerStatus 0.001, 4 hmi ' ...
    'Bus_HmiInputs 0.001, 5 env Bus_EnvState 0.1, 6 impl_mech Bus_ImplMechState 0.01, 7 flt Bus_FaultInject 0.001; ' ...
    'out 1 elec_hw Bus_ElecHwIn 0.001 (1 ms Unit Delay DI_Latency, IC P.ic.Bus_ElecHwIn: DI sampling latency, ' ...
    'breaks the MCU in1k -> out1k -> watchdog-reset loop, D36), 2 elec_adc Bus_ElecAdc 0.05, 3 elec_env ' ...
    'Bus_ElecEnv 0.1, 4 hw_cmd Bus_PowerHwCmd 0.001 (1 ms Unit Delay HwCmd_Delay, IC P.ic.Bus_PowerHwCmd), 5 ' ...
    'impl_cmd Bus_ImplementCmd 0.01, 6 lv_demand Bus_LV_LoadDemand 0.01, 7 indicators Bus_IndicatorOut 0.1. ' ...
    'Children (SSR): HW_Frontend_Sim elp_hw_frontend, Watchdog_Reset_Sim elp_watchdog_reset, ' ...
    'Actuator_Outputs_Sim elp_actuators, LV_Load_Demand elp_lv_demand, Electronics_Thermal elp_thermal (own ' ...
    'thermal network + Solver Configuration), Elec_Env elp_elec_env. Generated by ' ...
    'scripts/build/build_electronics_plant.m.']));
end

% ===========================================================================================
%  CAN1_Net_Sim (top-level SSR, ICD 3.1)
% ===========================================================================================
function local_CAN1_Net_Sim(n, f)
shaddad_bu('fresh', n, f, 'ssr');
G = {'mcu_fast', 'Bus_Can1_McuFast', 'Ts.ctrl', {'drv_cmd_L', 'drv_cmd_R'}, true; ...
     'mcu_slow', 'Bus_Can1_McuSlow', 'Ts.comm', {'impl_cmd', 'bms_req'}, true; ...
     'esc_fast', 'Bus_Can1_EscFast', 'Ts.ctrl', {'fb1_L', 'fb1_R', 'fb3_L', 'fb3_R'}, false; ...
     'esc_slow', 'Bus_Can1_EscSlow', 'Ts.comm', {'fb2_L', 'fb2_R'}, false; ...
     'impl', 'Bus_Can1_Impl', 'Ts.comm', {'impl_fb'}, false; ...
     'bms', 'Bus_Can1_Bms', 'Ts.hk', {'limits', 'soc', 'vit', 'alarm', 'cells', 'state', 'power'}, false};
for k = 1:size(G, 1), IN(n, G{k, 1}, k, ['Bus: ' G{k, 2}], G{k, 3}); end
IN(n, 'flt', 7, 'Bus: Bus_FaultInject', 'Ts.ctrl');
IN(n, 'elec_hw', 8, 'Bus: Bus_ElecHwIn', 'Ts.base');
for k = 1:size(G, 1), OUT(n, [G{k, 1} '_rx'], k, ['Bus: ' G{k, 2}]); end
% drop sources re-timed to each slot rate (fast-to-slow Rate Transitions); a Bus Selector in front of each
% consumer of flt and elec_hw (layout spec 6.2)
BS(n, 'Sel_rst_10', {'mcu_reset'});  L(n, 'elec_hw/1', 'Sel_rst_10/1', 'elec_hw');
RT(n, 'RT_rst_10', 'Ts.ctrl', 'false');  L(n, 'Sel_rst_10/1', 'RT_rst_10/1');
BS(n, 'Sel_busoff_10', {'can1_busoff'});  L(n, 'flt/1', 'Sel_busoff_10/1', 'flt');
LOGIC(n, 'McuDrop_10', 'OR', 2);  L(n, 'RT_rst_10/1', 'McuDrop_10/1', 'rst_10');  L(n, 'Sel_busoff_10/1', 'McuDrop_10/2');
BS(n, 'Sel_rst_50', {'mcu_reset'});  L(n, 'elec_hw/1', 'Sel_rst_50/1');
RT(n, 'RT_rst_50', 'Ts.comm', 'false');     L(n, 'Sel_rst_50/1', 'RT_rst_50/1');
BS(n, 'Sel_busoff_50', {'can1_busoff'});  L(n, 'flt/1', 'Sel_busoff_50/1');
RT(n, 'RT_busoff_50', 'Ts.comm', 'false');  L(n, 'Sel_busoff_50/1', 'RT_busoff_50/1');
LOGIC(n, 'McuDrop_50', 'OR', 2);  L(n, 'RT_rst_50/1', 'McuDrop_50/1', 'rst_50');
L(n, 'RT_busoff_50/1', 'McuDrop_50/2', 'busoff_50');
BS(n, 'Sel_mask_50', {'can1_drop_mask'});  L(n, 'flt/1', 'Sel_mask_50/1');
RT(n, 'RT_mask_50', 'Ts.comm', '0');   L(n, 'Sel_mask_50/1', 'RT_mask_50/1');
BS(n, 'Sel_mask_100', {'can1_drop_mask'});  L(n, 'flt/1', 'Sel_mask_100/1');
RT(n, 'RT_mask_100', 'Ts.hk', '0');    L(n, 'Sel_mask_100/1', 'RT_mask_100/1');
maskSrc = struct('Ts_comm', 'RT_mask_50/1', 'Ts_hk', 'RT_mask_100/1');   % Ts_ctrl: a pick per Bit block
maskName = struct('Ts_comm', 'mask_50', 'Ts_hk', 'mask_100');
mcuSrc = struct('Ts_ctrl', 'McuDrop_10/1', 'Ts_comm', 'McuDrop_50/1');
mcuName = struct('Ts_ctrl', 'mcu_drop_10', 'Ts_comm', 'mcu_drop_50');
for k = 1:size(G, 1)
    [inName, bus, ts, el, isMcu] = G{k, :};
    tsKey = strrep(ts, '.', '_');
    srcs = cell(1, numel(el));
    for j = 1:numel(el)
        e = el{j};
        bitName = e; if strcmp(inName, 'bms'), bitName = 'bms'; end
        A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', ['Bit_' e], 'logicop', 'AND', 'UseBitMask', 'on', ...
            'BitMask', sprintf('2^P.electronics.can.drop_bit.%s', bitName), 'BitMaskRealWorld', 'Stored Integer');
        if strcmp(tsKey, 'Ts_ctrl')
            BS(n, ['Sel_mask_' e], {'can1_drop_mask'});  L(n, 'flt/1', ['Sel_mask_' e '/1']);
            L(n, ['Sel_mask_' e '/1'], ['Bit_' e '/1']);
        else
            L(n, maskSrc.(tsKey), ['Bit_' e '/1'], maskName.(tsKey));
        end
        A(n, 'simulink/Logic and Bit Operations/Compare To Zero', ['Drop_' e], 'relop', '~=', 'OutDataTypeStr', 'boolean');
        L(n, ['Bit_' e '/1'], ['Drop_' e '/1'], ['bit_' e]);
        drop = ['Drop_' e '/1'];
        if isMcu
            LOGIC(n, ['DropMcu_' e], 'OR', 2);
            L(n, drop, ['DropMcu_' e '/1'], ['mask_drop_' e]);  L(n, mcuSrc.(tsKey), ['DropMcu_' e '/2'], mcuName.(tsKey));
            drop = ['DropMcu_' e '/1'];
        end
        SLOT(n, ['Slot_' e], 'Bus_CanFrame8', ts, 'P.electronics.can.can1_latency_samples', false);
        BS(n, ['Sel_' e], {e});  L(n, [inName '/1'], ['Sel_' e '/1'], inName);
        L(n, ['Sel_' e '/1'], ['Slot_' e '/1']);
        L(n, drop, ['Slot_' e '/2'], ['drop_' e]);
        srcs{j} = ['Slot_' e '/1'];
    end
    BC(n, ['BC_' inName], bus, srcs);
    L(n, ['BC_' inName '/1'], [inName '_rx/1'], [inName '_rx']);
end
local_finish(n, f, sprintf(['CAN1_Net_Sim (T10, top-level SSR of Shaddad_System, ICD 3.1 CAN1_Net). in 1 mcu_fast ' ...
    'Bus_Can1_McuFast 0.01, 2 mcu_slow Bus_Can1_McuSlow 0.05, 3 esc_fast Bus_Can1_EscFast 0.01, 4 esc_slow ' ...
    'Bus_Can1_EscSlow 0.05, 5 impl Bus_Can1_Impl 0.05, 6 bms Bus_Can1_Bms 0.1, 7 flt Bus_FaultInject 0.01, 8 elec_hw ' ...
    'Bus_ElecHwIn 0.001; out 1..6 the same buses delivered (*_rx), same rates. Per frame slot: drop = bit ' ...
    'P.electronics.can.drop_bit.<slot> of flt.can1_drop_mask (b0 DRV_CMD_L b1 DRV_CMD_R b2/b3 FB1_L/R b4/b5 FB3_L/R ' ...
    'b6/b7 FB2_L/R b8 IMPL_CMD b9 BMS_REQ b10 IMPL_FB b11 all BMS frames) OR, for MCU-originated slots (DRV_CMD_L/R, ' ...
    'IMPL_CMD, BMS_REQ), elec_hw.mcu_reset OR flt.can1_busoff (re-timed to the slot rate by Rate Transitions); ' ...
    'dropped -> valid false and data held (Switch + Unit Delay); then Delay of ' ...
    'P.electronics.can.can1_latency_samples (0) samples.']));
end

% ===========================================================================================
%  Link_MCU_GNC (top-level SSR, ICD 3.1 / 5)
% ===========================================================================================
function local_Link_MCU_GNC(n, f)
shaddad_bu('fresh', n, f, 'ssr');
S = {'mcu_imu', 'Ts.imu', ''; 'mcu_odom', 'Ts.ctrl', ''; 'mcu_status', 'Ts.comm', 'P.electronics.can.can2_status_delay'; ...
     'mcu_power', 'Ts.hk', ''; 'comp_cmd', 'Ts.comm', 'P.electronics.can.can2_cmd_delay'};
for k = 1:size(S, 1), IN(n, S{k, 1}, k, 'Bus: Bus_CanFrame64', S{k, 2}); end
IN(n, 'flt', 6, 'Bus: Bus_FaultInject', 'Ts.ctrl');
IN(n, 'elec_hw', 7, 'Bus: Bus_ElecHwIn', 'Ts.base');
for k = 1:size(S, 1), OUT(n, [S{k, 1} '_rx'], k, 'Bus: Bus_CanFrame64'); end
BS(n, 'Sel_flt', {'can2_loss', 'can2_busoff'});  L(n, 'flt/1', 'Sel_flt/1', 'flt');
LOGIC(n, 'LinkDown', 'OR', 2);  L(n, 'Sel_flt/1', 'LinkDown/1', 'can2_loss');  L(n, 'Sel_flt/2', 'LinkDown/2', 'can2_busoff');
rates = {'Ts.imu', 'imu'; 'Ts.ctrl', 'ctrl'; 'Ts.comm', 'comm'; 'Ts.hk', 'hk'};
for k = 1:size(rates, 1)
    [ts, tag] = rates{k, :};
    if strcmp(ts, 'Ts.ctrl')
        ld = 'LinkDown/1';
    else
        RT(n, ['RT_link_' tag], ts, 'false');  L(n, 'LinkDown/1', ['RT_link_' tag '/1'], 'link_down');
        ld = ['RT_link_' tag '/1'];
    end
    % a Bus Selector of mcu_reset in front of each Rate Transition (layout spec 6.2)
    BS(n, ['Sel_rst_' tag], {'mcu_reset'});  L(n, 'elec_hw/1', ['Sel_rst_' tag '/1'], 'elec_hw');
    RT(n, ['RT_rst_' tag], ts, 'false');  L(n, ['Sel_rst_' tag '/1'], ['RT_rst_' tag '/1']);
    LOGIC(n, ['Drop_' tag], 'OR', 2);
    if strcmp(ts, 'Ts.ctrl'), L(n, ld, ['Drop_' tag '/1']); else, L(n, ld, ['Drop_' tag '/1'], ['link_down_' tag]); end
    L(n, ['RT_rst_' tag '/1'], ['Drop_' tag '/2'], ['rst_' tag]);
end
for k = 1:size(S, 1)
    [nm, ts, dly] = S{k, :};
    tag = rates{strcmp(rates(:, 1), ts), 2};
    SLOT(n, ['Slot_' nm], 'Bus_CanFrame64', ts, dly, strcmp(nm, 'mcu_status'));
    L(n, [nm '/1'], ['Slot_' nm '/1'], nm);
    L(n, ['Drop_' tag '/1'], ['Slot_' nm '/2'], ['drop_' tag]);
    L(n, ['Slot_' nm '/1'], [nm '_rx/1'], [nm '_rx']);
end
local_finish(n, f, sprintf(['Link_MCU_GNC (T10, top-level SSR of Shaddad_System, ICD 3.1 CAN2_Link / ICD 5). in 1 ' ...
    'mcu_imu Bus_CanFrame64 0.005, 2 mcu_odom 0.01, 3 mcu_status 0.05, 4 mcu_power 0.1, 5 comp_cmd 0.05, 6 flt ' ...
    'Bus_FaultInject 0.01, 7 elec_hw Bus_ElecHwIn 0.001; out 1 mcu_imu_rx 0.005, 2 mcu_odom_rx 0.01, 3 ' ...
    'mcu_status_rx 0.05, 4 mcu_power_rx 0.1, 5 comp_cmd_rx 0.05 (all Bus_CanFrame64). All five CAN-FD frames are ' ...
    'dropped (valid false, data held: Switch + Unit Delay) on flt.can2_loss OR flt.can2_busoff OR ' ...
    'elec_hw.mcu_reset (Rate Transitions to each frame rate). MCU_STATUS: Delay of ' ...
    'P.electronics.can.can2_status_delay = 1 sample at 0.05 s (IC P.ic.Bus_CanFrame64, PreventDirectFeedthrough: ' ...
    'MCU status assembly delay and MCU <-> GNC loop breaker, D41); COMP_CMD: Delay ' ...
    'P.electronics.can.can2_cmd_delay = 0 (undelayed; the MCU Task_50 -> Task_10 RT is the GNC compute delay).']));
end

% ===========================================================================================
%  companion_io_ros2 (MODEL: HW frame IO of companion_io, D38)
% ===========================================================================================
function local_companion_io_ros2(n, f)
shaddad_bu('fresh', n, f, 'model');
shaddad_configsets('attach', n);
fr = {'mcu_imu', 'Ts.imu', 'mcu_imu'; 'mcu_odom', 'Ts.ctrl', 'mcu_odom'; 'mcu_status', 'Ts.comm', 'mcu_status'; ...
      'mcu_power', 'Ts.hk', 'mcu_power'};
A(n, 'simulink/Sources/In1', 'comp_cmd', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', ...
    'SampleTime', 'Ts.comm', 'BusOutputAsStruct', 'on');
for k = 1:size(fr, 1)
    A(n, 'simulink/Sinks/Out1', fr{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', 'Bus: Bus_CanFrame64', ...
        'SampleTime', fr{k, 2}, 'BusOutputAsStruct', 'on');
end
q = num2str(evalin('base', 'P.electronics.can.hw.qos_depth'));
% ---- RX: /can2/rx -> id / len / data -> ID dispatcher ----
A(n, 'ros2lib/Subscribe', 'Sub_can2_rx', 'topicSource', 'Specify your own', 'topic', '/can2/rx', ...
    'messageType', 'std_msgs/UInt8MultiArray', 'SubscriberMode', 'At sample time', 'QOSHistory', 'Keep last', ...
    'QOSDepth', q);
set_param([n '/Sub_can2_rx'], 'sampleTime', 'Ts.base');
% a Bus Selector of the message in front of each consumer (layout spec 6.2)
BS(n, 'Sel_msg', {'data'});      L(n, 'Sub_can2_rx/2', 'Sel_msg/1', 'can2_rx_msg');
BS(n, 'Sel_msg_2', {'data'});    L(n, 'Sub_can2_rx/2', 'Sel_msg_2/1');
BS(n, 'Sel_msg_3', {'data'});    L(n, 'Sub_can2_rx/2', 'Sel_msg_3/1');
BS(n, 'Sel_msg_len', {'data_SL_Info.CurrentLength'});  L(n, 'Sub_can2_rx/2', 'Sel_msg_len/1');
SEL(n, 'IdBytes', '1:4', -1);    L(n, 'Sel_msg/1', 'IdBytes/1');
SEL(n, 'LenByte', '5', -1);      L(n, 'Sel_msg_2/1', 'LenByte/1');
SEL(n, 'DataBytes', '6:69', -1); L(n, 'Sel_msg_3/1', 'DataBytes/1');
A(n, 'mcblib/Signal Management/Byte Unpack', 'IdUnpack', 'dimensions', '{1}', 'datatypes', '{''uint32''}', ...
    'byteAlign', '1');
L(n, 'IdBytes/1', 'IdUnpack/1');
CMP(n, 'LenOK', '>=', 'P.electronics.can.hw.msg_len');  L(n, 'Sel_msg_len/1', 'LenOK/1');
LOGIC(n, 'MsgOK', 'AND', 2);  L(n, 'Sub_can2_rx/1', 'MsgOK/1', 'IsNew');  L(n, 'LenOK/1', 'MsgOK/2');
for k = 1:size(fr, 1)
    [nm, ts, idName] = fr{k, :};
    CMP(n, ['Id_' nm], '==', sprintf('P.electronics.can.id.%s', idName));  L(n, 'IdUnpack/1', ['Id_' nm '/1'], 'id');
    LOGIC(n, ['New_' nm], 'AND', 2);  L(n, 'MsgOK/1', ['New_' nm '/1']);  L(n, ['Id_' nm '/1'], ['New_' nm '/2']);
    % latch (Enabled Subsystem, outputs held) and a frame counter -> new-frame flag at the frame rate
    es = [n '/Latch_' nm];
    add_block('simulink/Ports & Subsystems/Enabled Subsystem', es);
    delete_line(es, 'In1/1', 'Out1/1');
    set_param([es '/In1'], 'Name', 'id');
    add_block('simulink/Sources/In1', [es '/len'], 'Port', '2');
    add_block('simulink/Sources/In1', [es '/data'], 'Port', '3');
    set_param([es '/Out1'], 'Name', 'id_l', 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
    add_block('simulink/Sinks/Out1', [es '/len_l'], 'Port', '2', 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
    add_block('simulink/Sinks/Out1', [es '/data_l'], 'Port', '3', 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
    L(es, 'id/1', 'id_l/1', 'id');  L(es, 'len/1', 'len_l/1', 'len');  L(es, 'data/1', 'data_l/1', 'data');
    L(n, ['New_' nm '/1'], ['Latch_' nm '/Enable'], ['new_' nm]);
    L(n, 'IdUnpack/1', ['Latch_' nm '/1']);  L(n, 'LenByte/1', ['Latch_' nm '/2'], 'len');
    L(n, 'DataBytes/1', ['Latch_' nm '/3'], 'data');
    DTC(n, ['NewU8_' nm], 'uint8');  L(n, ['New_' nm '/1'], ['NewU8_' nm '/1']);
    UD(n, ['Cnt_z_' nm], '0', 'Ts.base');
    A(n, 'simulink/Math Operations/Sum', ['Cnt_' nm], 'Inputs', '++', 'OutDataTypeStr', 'uint8', ...
        'SaturateOnIntegerOverflow', 'off');
    L(n, ['NewU8_' nm '/1'], ['Cnt_' nm '/1']);  L(n, ['Cnt_z_' nm '/1'], ['Cnt_' nm '/2']);
    L(n, ['Cnt_' nm '/1'], ['Cnt_z_' nm '/1'], ['cnt_' nm]);
    fld = {'id', 'len', 'data'};
    for j = 1:3
        RT(n, sprintf('RT_%s_%d', nm, j), ts, '0');
        L(n, sprintf('Latch_%s/%d', nm, j), sprintf('RT_%s_%d/1', nm, j), [nm '_' fld{j}]);
    end
    RT(n, ['RT_cnt_' nm], ts, '0');  L(n, ['Cnt_' nm '/1'], ['RT_cnt_' nm '/1']);
    A(n, 'simulink/Logic and Bit Operations/Detect Change', ['Fresh_' nm], 'vinit', '0');
    L(n, ['RT_cnt_' nm '/1'], ['Fresh_' nm '/1']);
    BC(n, ['BC_' nm], 'Bus_CanFrame64', {sprintf('RT_%s_1/1', nm), sprintf('RT_%s_2/1', nm), sprintf('RT_%s_3/1', nm), ...
        ['Fresh_' nm '/1']});
    L(n, ['BC_' nm '/1'], [nm '/1'], nm);
end
% ---- TX: COMP_CMD frame -> 69-byte message -> /can2/tx (published only when the frame is valid) ----
% a Bus Selector of the frame in front of each consumer: Sel_tx_id, Sel_tx (len, data), Sel_tx_valid
BS(n, 'Sel_tx_id', {'id'});  L(n, 'comp_cmd/1', 'Sel_tx_id/1', 'comp_cmd');
A(n, 'mcblib/Signal Management/Byte Pack', 'IdPack', 'datatypes', '{''uint32''}', 'byteAlign', '1');
L(n, 'Sel_tx_id/1', 'IdPack/1');
BS(n, 'Sel_tx', {'len', 'data'});  L(n, 'comp_cmd/1', 'Sel_tx/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'TxBytes', 'NumInputs', '3');
L(n, 'IdPack/1', 'TxBytes/1');  L(n, 'Sel_tx/1', 'TxBytes/2');  L(n, 'Sel_tx/2', 'TxBytes/3');
% Blank Message: the mask parameter entityType carries the message type (messageType mirrors it)
A(n, 'ros2lib/Blank Message', 'TxBlank', 'messageClass', 'Message', 'entityType', 'std_msgs/UInt8MultiArray', ...
    'messageType', 'std_msgs/UInt8MultiArray', 'SampleTime', 'Ts.comm');
BS(n, 'Sel_blank', {'data'});  L(n, 'TxBlank/1', 'Sel_blank/1', 'blank_msg');
A(n, 'simulink/Math Operations/Assignment', 'TxData', 'NumberOfDimensions', '1', 'IndexOptions', ...
    'Index vector (dialog)', 'Indices', '1:P.electronics.can.hw.msg_len', 'OutputInitialize', ...
    'Initialize using input port <Y0>');
L(n, 'Sel_blank/1', 'TxData/1');  L(n, 'TxBytes/1', 'TxData/2', 'tx_bytes');
C(n, 'TxLen', 'P.electronics.can.hw.msg_len', 'uint32', 'Ts.comm');
A(n, 'simulink/Signal Routing/Bus Assignment', 'TxMsg', 'AssignedSignals', 'data,data_SL_Info.CurrentLength');
L(n, 'TxBlank/1', 'TxMsg/1');  L(n, 'TxData/1', 'TxMsg/2');  L(n, 'TxLen/1', 'TxMsg/3');
es = [n '/Pub_can2_tx'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', es);
delete_line(es, 'In1/1', 'Out1/1');
delete_block([es '/Out1']);
set_param([es '/In1'], 'Name', 'msg');
add_block('ros2lib/Publish', [es '/Publish'], 'topicSource', 'Specify your own', 'topic', '/can2/tx', ...
    'messageType', 'std_msgs/UInt8MultiArray', 'QOSHistory', 'Keep last', 'QOSDepth', q);
L(es, 'msg/1', 'Publish/1', 'msg');
L(n, 'TxMsg/1', 'Pub_can2_tx/1', 'can2_tx_msg');
BS(n, 'Sel_tx_valid', {'valid'});  L(n, 'comp_cmd/1', 'Sel_tx_valid/1');
L(n, 'Sel_tx_valid/1', 'Pub_can2_tx/Enable');
set_param(n, 'Description', sprintf(['companion_io_ros2 (T10, Model, HW choice of companion_io/FrameIO, ' ...
    'VC_TARGET == 1, D38). in 1 comp_cmd Bus_CanFrame64 0.05; out 1 mcu_imu 0.005, 2 mcu_odom 0.01, 3 ' ...
    'mcu_status 0.05, 4 mcu_power 0.1 (Bus_CanFrame64). RX: ros2lib/Subscribe /can2/rx std_msgs/UInt8MultiArray ' ...
    '(At sample time, Ts.base) -> data(1:4) Byte Unpack uint32 id (LE), data(5) len, data(6:69) payload; IsNew AND ' ...
    'CurrentLength >= 69 AND id == P.electronics.can.id.<frame> enables a latch Enabled Subsystem (held) and a ' ...
    'uint8 frame counter; Rate Transitions to the frame rate; valid = Detect Change of the counter (a new frame ' ...
    'since the previous frame-rate sample). TX: COMP_CMD frame -> Byte Pack id + len + data (69 B) into the ' ...
    'Blank Message data (Assignment) with data_SL_Info.CurrentLength = 69 -> ros2lib/Publish /can2/tx inside an ' ...
    'Enabled Subsystem (frame.valid).\nExternal SocketCAN-FD bridge node contract (not part of the model): runs on ' ...
    'the Jetson, opens can0 as CAN-FD (1 Mbit/s nominal, 2 Mbit/s data, BRS); for every received frame with an ID in ' ...
    '{0x200 MCU_STATUS, 0x201 MCU_IMU, 0x202 MCU_ODOM, 0x203 MCU_POWER} publishes one std_msgs/UInt8MultiArray on ' ...
    '/can2/rx with data = [id u32 little-endian, len u8, payload 64 B zero padded] (69 bytes, layout.dim empty), ' ...
    'QoS reliable / keep last 16, consecutive messages spaced by >= P.electronics.can.hw.min_gap (2 ms; this ' ...
    'subscriber samples the newest message every 1 ms, so pacing guarantees that no frame is overwritten); ' ...
    'subscribes /can2/tx (same layout) and writes each message as one CAN-FD frame (BRS, DLC from len) to can0. ' ...
    'The classic ros2lib CAN Receive / CAN Transmit blocks are limited to 8-byte payloads (MsgLength <= 8, ' ...
    'verified) and cannot carry the CAN-FD frames. A Model (not a Subsystem Reference) because ROS 2 message ' ...
    'buses do not propagate inside a Subsystem Reference in R2026b (verified). Generated by ' ...
    'scripts/build/build_electronics_plant.m.']));
shaddad_bu('save', n, f);
shaddad_bu('finalize', n, f);
close_system(n, 0);
end

% ===========================================================================================
%  companion_io (SSR inside Shaddad_Companion, ICD 3.1 Companion_IO)
% ===========================================================================================
function local_companion_io(n, f)
shaddad_bu('fresh', n, f, 'ssr');
fr = {'mcu_imu', 'Ts.imu', 'can2_mcu_imu_unpack', 'imu_meas', 'Bus_ImuMeas'; ...
      'mcu_odom', 'Ts.ctrl', 'can2_mcu_odom_unpack', 'odom_meas', 'Bus_OdomMeas'; ...
      'mcu_status', 'Ts.comm', 'can2_mcu_status_unpack', 'mcu_status_dec', 'Bus_McuStatus'; ...
      'mcu_power', 'Ts.hk', 'can2_mcu_power_unpack', 'bms_status', 'Bus_BmsStatus'};
for k = 1:size(fr, 1), IN(n, fr{k, 1}, k, 'Bus: Bus_CanFrame64', fr{k, 2}); end
IN(n, 'gnc_cmd', 5, 'Bus: Bus_GncCmd', 'Ts.comm');
for k = 1:size(fr, 1), OUT(n, fr{k, 4}, k, ['Bus: ' fr{k, 5}]); end
OUT(n, 'comp_cmd', 5, 'Bus: Bus_CanFrame64');
% ---- COMP_CMD pack (T08 codec) ----
SSR(n, 'Comp_Pack', 'can2_comp_cmd_pack');
L(n, 'gnc_cmd/1', 'Comp_Pack/1', 'gnc_cmd');
L(n, 'Comp_Pack/1', 'comp_cmd/1', 'comp_cmd');
% ---- frame source: Variant VC_TARGET {SIM root frames | HW ROS 2 (Model companion_io_ros2)} ----
vs = A(n, 'simulink/Ports & Subsystems/Variant Subsystem', 'FrameIO');
local_clearsub(vs);
for k = 1:size(fr, 1)
    add_block('simulink/Sources/In1', [vs '/' fr{k, 1}], 'Port', num2str(k));
    add_block('simulink/Sinks/Out1', [vs '/' fr{k, 1} '_f'], 'Port', num2str(k));
end
add_block('simulink/Sources/In1', [vs '/comp_cmd'], 'Port', '5');
s = A(vs, 'simulink/Ports & Subsystems/Subsystem', 'SIM', 'VariantControl', 'VC_TARGET == 0');
local_clearsub(s);
for k = 1:size(fr, 1)
    add_block('simulink/Sources/In1', [s '/' fr{k, 1}], 'Port', num2str(k));
    add_block('simulink/Sinks/Out1', [s '/' fr{k, 1} '_f'], 'Port', num2str(k));
    L(s, [fr{k, 1} '/1'], [fr{k, 1} '_f/1'], fr{k, 1});
end
add_block('simulink/Sources/In1', [s '/comp_cmd'], 'Port', '5');
add_block('simulink/Sinks/Terminator', [s '/T_comp_cmd']);
L(s, 'comp_cmd/1', 'T_comp_cmd/1', 'comp_cmd');
h = A(vs, 'simulink/Ports & Subsystems/Subsystem', 'HW', 'VariantControl', 'VC_TARGET == 1');
local_clearsub(h);
add_block('simulink/Ports & Subsystems/Model', [h '/CAN2_ROS2'], 'ModelName', 'companion_io_ros2', ...
    'SimulationMode', 'Normal');
for k = 1:size(fr, 1)
    add_block('simulink/Sources/In1', [h '/' fr{k, 1}], 'Port', num2str(k));
    add_block('simulink/Sinks/Terminator', [h '/T_' fr{k, 1}]);
    L(h, [fr{k, 1} '/1'], ['T_' fr{k, 1} '/1'], fr{k, 1});
    add_block('simulink/Sinks/Out1', [h '/' fr{k, 1} '_f'], 'Port', num2str(k));
    if strcmp(fr{k, 1}, 'mcu_status')
        % loop breaker (D36 / D41): GNC -> Comp_Pack -> CAN2_ROS2 (Model, direct feedthrough) -> mcu_status ->
        % GNC input 3 is an algebraic loop without it; SIM breaks it with the MCU_STATUS delay in Link_MCU_GNC
        UD(h, 'Z_mcu_status', 'P.ic.Bus_CanFrame64', 'Ts.gnc');
        L(h, sprintf('CAN2_ROS2/%d', k), 'Z_mcu_status/1', fr{k, 1});
        L(h, 'Z_mcu_status/1', [fr{k, 1} '_f/1'], [fr{k, 1} '_z']);
    else
        L(h, sprintf('CAN2_ROS2/%d', k), [fr{k, 1} '_f/1'], fr{k, 1});
    end
end
add_block('simulink/Sources/In1', [h '/comp_cmd'], 'Port', '5');
A(h, 'simulink/Signal Attributes/Signal Specification', 'Spec_comp_cmd', 'OutDataTypeStr', 'Bus: Bus_CanFrame64', ...
    'SampleTime', 'Ts.comm', 'BusOutputAsStruct', 'on');
L(h, 'comp_cmd/1', 'Spec_comp_cmd/1', 'comp_cmd');
L(h, 'Spec_comp_cmd/1', 'CAN2_ROS2/1', 'comp_cmd');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
for k = 1:size(fr, 1), L(n, [fr{k, 1} '/1'], sprintf('FrameIO/%d', k), fr{k, 1}); end
L(n, 'Comp_Pack/1', 'FrameIO/5');
% ---- CAN-FD unpack codecs (T08) ----
for k = 1:size(fr, 1)
    blk = ['Unpack_' fr{k, 1}];
    SSR(n, blk, fr{k, 3});
    L(n, sprintf('FrameIO/%d', k), [blk '/1'], [fr{k, 1} '_frame']);
    if ~strcmp(fr{k, 1}, 'mcu_status')
        L(n, [blk '/1'], [fr{k, 4} '/1'], fr{k, 4});
    end
end
% ---- Bus_McuStatus.valid = MCU_STATUS frame received within mcu_status_timeout (age in 0.05 s ticks) ----
nTo = 'round(P.electronics.can.mcu_status_timeout/Ts.comm)';
nSat = 'round(P.electronics.can.age_sat_s/Ts.comm)';
BS(n, 'Sel_status_valid', {'valid'});  L(n, 'FrameIO/3', 'Sel_status_valid/1');
UD(n, 'Age_z', nSat, 'Ts.comm');
A(n, 'simulink/Math Operations/Bias', 'Age_inc', 'Bias', '1');  L(n, 'Age_z/1', 'Age_inc/1');
A(n, 'simulink/Discontinuities/Saturation', 'Age_sat', 'UpperLimit', nSat, 'LowerLimit', '0');
L(n, 'Age_inc/1', 'Age_sat/1');
C(n, 'Age_zero', '0', 'double', 'Ts.comm');
SW(n, 'Age');  L(n, 'Age_zero/1', 'Age/1');  L(n, 'Sel_status_valid/1', 'Age/2', 'frame_valid');  L(n, 'Age_sat/1', 'Age/3');
L(n, 'Age/1', 'Age_z/1', 'status_age_ticks');
CMP(n, 'StatusFresh', '<=', nTo);  L(n, 'Age/1', 'StatusFresh/1');
A(n, 'simulink/Signal Routing/Bus Assignment', 'StatusValid', 'AssignedSignals', 'valid');
L(n, 'Unpack_mcu_status/1', 'StatusValid/1', 'mcu_status_raw');  L(n, 'StatusFresh/1', 'StatusValid/2', 'status_fresh');
L(n, 'StatusValid/1', 'mcu_status_dec/1', 'mcu_status_dec');
local_finish(n, f, sprintf(['companion_io (T10, SSR Companion_IO inside Shaddad_Companion, ICD 3.1 / D38). in 1 ' ...
    'mcu_imu Bus_CanFrame64 0.005, 2 mcu_odom 0.01, 3 mcu_status 0.05, 4 mcu_power 0.1, 5 gnc_cmd Bus_GncCmd 0.05; ' ...
    'out 1 imu_meas Bus_ImuMeas 0.005, 2 odom_meas Bus_OdomMeas 0.01, 3 mcu_status_dec Bus_McuStatus 0.05 ' ...
    '(name differs from ICD mcu_status because input 3 has that name), 4 bms_status Bus_BmsStatus 0.1, 5 ' ...
    'comp_cmd Bus_CanFrame64 0.05. FrameIO Variant Subsystem VC_TARGET (update diagram): SIM (0) = the root ' ...
    'frames; HW (1) = Model companion_io_ros2 (ros2lib Subscribe /can2/rx, Publish /can2/tx, ' ...
    'std_msgs/UInt8MultiArray 69 B, ID dispatcher; external SocketCAN-FD bridge contract in that model''s ' ...
    'description), MCU_STATUS through the Unit Delay Z_mcu_status (Ts.gnc, IC P.ic.Bus_CanFrame64: loop breaker ' ...
    'GNC -> Comp_Pack -> CAN2_ROS2 -> mcu_status -> GNC, D36; SIM: Link_MCU_GNC delay). Both feed the same T08 codecs: can2_mcu_imu_unpack, can2_mcu_odom_unpack, ' ...
    'can2_mcu_status_unpack, can2_mcu_power_unpack (P_pack = V*I, CVL / DVL / P_chg_lim = 0) and ' ...
    'can2_comp_cmd_pack (gnc_cmd -> COMP_CMD frame). Bus_McuStatus.valid = no more than ' ...
    'round(P.electronics.can.mcu_status_timeout/Ts.comm) ticks since the last valid MCU_STATUS frame ' ...
    '(false 0.2 s after frames stop; false at boot). Frame data are held upstream (Link_MCU_GNC in SIM, latches in HW).']));
end

% ===========================================================================================
%  helpers
% ===========================================================================================
function local_clearsub(s)
lns = find_system(s, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants), {s});
for k = 1:numel(b), delete_block(b{k}); end
end

function SLOT(sys, name, bus, ts, delayExpr, noFeedthrough)
% one frame slot: drop -> valid false and data held; optional Delay of delayExpr samples. The frame is the In
% Bus Element port frame (input 1), each element picked right in front of the block it feeds (layout spec 6.2);
% laid out at save
s = [sys '/' name];
add_block('built-in/SubSystem', s);
add_block('simulink/Sinks/Out1', [s '/frame_rx'], 'Port', '1');
SW(s, 'Hold');
UD(s, 'Held', '0', ts);
LOGIC(s, 'NotDrop', 'NOT', 1);
LOGIC(s, 'Valid', 'AND', 2);
A(s, 'simulink/Signal Routing/Bus Creator', 'BC', 'Inputs', '4', 'OutDataTypeStr', ['Bus: ' bus], ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
shaddad_bu('pick', s, 'frame', 'id', 'BC/1');
shaddad_bu('pick', s, 'frame', 'len', 'BC/2');
shaddad_bu('pick', s, 'frame', 'data', 'Hold/3');
shaddad_bu('pick', s, 'frame', 'valid', 'Valid/1');
add_block('simulink/Sources/In1', [s '/drop'], 'Port', '2');
L(s, 'Held/1', 'Hold/1', 'data_held');  L(s, 'drop/1', 'Hold/2', 'drop');
L(s, 'Hold/1', 'Held/1', 'data_rx');
L(s, 'drop/1', 'NotDrop/1');
L(s, 'NotDrop/1', 'Valid/2', 'not_drop');
L(s, 'Hold/1', 'BC/3');  L(s, 'Valid/1', 'BC/4', 'valid_rx');
if isempty(delayExpr)
    L(s, 'BC/1', 'frame_rx/1', 'frame_rx');
else
    pdf = 'off'; if noFeedthrough, pdf = 'on'; end
    A(s, 'simulink/Discrete/Delay', 'Latency', 'DelayLength', delayExpr, 'InitialCondition', ['P.ic.' bus], ...
        'SampleTime', ts, 'PreventDirectFeedthrough', pdf);
    L(s, 'BC/1', 'Latency/1', 'frame_link');
    L(s, 'Latency/1', 'frame_rx/1', 'frame_rx');
end
end

function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function h = L(sys, src, dst, nm)
% a named signal line (layout spec 6.1, L5): shaddad_bu('nline') names the line nm, else keeps the name its source
% port's line already has, else names it after its source block ('_k' for output k of a block with several
% outputs); a line out of a Bus Selector keeps R2026b's '<element>'
if nargin < 4, nm = ''; end
h = shaddad_bu('nline', sys, src, dst, nm);
end

function PL(sys, hA, hB)
add_line(sys, hA, hB, 'autorouting', 'smart');
end

function IN(sys, name, port, type, ts, dims)
p = A(sys, 'simulink/Sources/In1', name, 'Port', num2str(port), 'OutDataTypeStr', type, 'SampleTime', ts);
if nargin >= 6 && ~isempty(dims), set_param(p, 'PortDimensions', dims); end
end

function OUT(sys, name, port, type)
A(sys, 'simulink/Sinks/Out1', name, 'Port', num2str(port), 'OutDataTypeStr', type);
end

function SSR(sys, name, file)
A(sys, 'simulink/Ports & Subsystems/Subsystem Reference', name, 'ReferencedSubsystem', file);
end

function RT(sys, name, ts, ic)
A(sys, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', ts, 'InitialCondition', ic);
end

function UD(sys, name, ic, ts)
A(sys, 'simulink/Discrete/Unit Delay', name, 'InitialCondition', ic, 'SampleTime', ts);
end

function BS(sys, name, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', name, 'OutputSignals', strjoin(sigs, ','));
end

function BC(sys, name, bus, srcs)
% nonvirtual Bus Creator of type bus; an input line without a name yet takes its element name (L5)
A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(srcs)), 'OutDataTypeStr', ['Bus: ' bus], ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
B = evalin('base', bus);
for k = 1:numel(srcs)
    ph = get_param([sys '/' extractBefore(srcs{k}, '/')], 'PortHandles');
    old = get_param(ph.Outport(str2double(extractAfter(srcs{k}, '/'))), 'Line');
    if old ~= -1 && ~isempty(get_param(old, 'Name'))
        L(sys, srcs{k}, sprintf('%s/%d', name, k));
    else
        L(sys, srcs{k}, sprintf('%s/%d', name, k), B.Elements(k).Name);
    end
end
end

function C(sys, name, val, type, ts)
A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function DTC(sys, name, type)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', type, 'RndMeth', 'Nearest', ...
    'SaturateOnIntegerOverflow', 'on');
end

function LOGIC(sys, name, op, n)
A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(n));
end

function CMP(sys, name, op, val)
A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', name, 'relop', op, 'const', val, ...
    'OutDataTypeStr', 'boolean');
end

function SW(sys, name)
A(sys, 'simulink/Signal Routing/Switch', name, 'Criteria', 'u2 ~= 0');
end

function SEL(sys, name, idx, width)
A(sys, 'simulink/Signal Routing/Selector', name, 'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', idx, 'InputPortWidth', num2str(width));
end
