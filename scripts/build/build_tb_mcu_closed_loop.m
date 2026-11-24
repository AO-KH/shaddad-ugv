function build_tb_mcu_closed_loop()
%BUILD_TB_MCU_CLOSED_LOOP  Bench tests/electronics/tb_mcu_closed_loop.slx (owner: electronics, T09.6).
%
%   RC_Link_Sim -> Shaddad_MCU (Model, Normal) closed over a simplified plant:
%     Drive_Rx      DRV_CMD frames (out10.can1_drv, drop mask b0/b1) -> drive_cmd_unpack x2 (T06) -> Bus_DriveCmd
%     Plant         per track v_cmd = enable ? w_ref/k_v2w : 0 -> Rate Transition 0.01 -> 0.001 ->
%                   simulink/Discrete/Transfer Fcn First Order (pole exp(-Ts.base/tau_esc), ESC speed loop) ->
%                   robotmobilelib/Differential Drive Kinematic Model ('Wheel Speeds', WheelRadius 1, TrackWidth
%                   chi*B); the yaw-gain factor (input plant(1)) scales the wheel-speed difference, the gyro is the
%                   model's yaw rate (stateDot(3))
%     IMU_Emu       Bus_ImuRaw (0.005): [a_x - g sin(pitch); v*r + g sin(roll) cos(pitch); g cos(roll) cos(pitch)],
%                   gyro [0 0 r], a_x = Discrete Derivative of the mean track speed, pitch / roll from plant(2:3);
%                   magnetometer [20 0 -40] uT + a one-LSB toggle on x every sample (a real IMU is never bit-identical:
%                   sens_acq_imu flags safety.imu_frozen_t of repeated samples, R-05); flt.imu_fault 1 holds the
%                   sample (stuck IMU, as sens_imu_model)
%     ESC_Fb        Bus_DriveSideFb per track (state OPERATION/READY, w_m, encoder counts, tau_avail = T_peak) ->
%                   drive_fb_pack x2 (T06) -> FB1/FB3 (0.01) and FB2 (RT 0.05), drop mask b2..b7
%     BMS_Tx        bms input (Bus_BMS_Telemetry, 0.1) -> bms_can_pack (T04), drop mask b11
%     COMP_Tx       gnc input (Bus_GncCmd, 0.05) with v_ground_est := plant(4)*mean track speed -> can2_comp_cmd_pack
%                   (T08), dropped while flt.can2_loss or flt.can2_busoff
%     Safety_Emu    (0.001) Loop_OK = NOT E-stop AND NOT bumper AND DO0 AND key on; reset pulse = rising(reset_btn OR
%                   DO1) stretched to P.power.safety.t_reset_hold (Tapped Delay + OR); SR latch (reset dominant);
%                   SAFETY_OK / SAFETY_HV_OK = latch through a 1 ms Unit Delay -> DI word (ICD 6)
%     MCU_In        Bus_McuIn1k/5/10/50/100 (fault_inject as Shaddad_System/MCU/MCU_In, constant healthy ADC counts,
%                   no implement frames)
%   Root inputs: 1 op Bus_OperatorCmd, 2 flt Bus_FaultInject, 3 hmi Bus_HmiInputs (Ts.ctrl), 4 gnc Bus_GncCmd
%   (Ts.comm), 5 bms Bus_BMS_Telemetry (Ts.hk; its I_chg_lim_A is the CCL of the MCU torque limiter), 6 plant double[9]
%   (Ts.ctrl) [yaw_gain pitch roll ground_ratio a_run brake_fail impl m f_rr]: a_run (m/s^2) a runaway load that the
%   drives cannot brake and the brakes cannot hold (the tracks accelerate forward at a_run beyond their command),
%   brake_fail b0 L / b1 R an ESC whose brake never reports engaged (bench faults of the D71 stop tests, 2026-11-15);
%   impl the implement coding plug DI IMPL_ID (0 none, 1 mower, 2 tiller, 3 sprayer), m (kg) > 0 the physical plant
%   (Plant/Track_dyn: the speed loops act through the DRV_CMD torque limits on m/2 per track, with the slope load of
%   pitch and the rolling resistance coefficient f_rr; m 0, the default, keeps the first-order plant) (descent task,
%   2026-11-15).
%   Root outputs: 1 out1k, 2 out10, 3 out50, 4 telem (MCU), 5 cmd Bus_DriveCmd (decoded DRV_CMD), 6 rc_frame,
%   7 state double[8] (0.001) [v_L v_R v_mean yaw_rate x y psi relay], 8 di_word uint16 (0.001).
%   Run with tests/electronics/mcu_cl_run.m. Config Shaddad_Ref (cs_Shaddad).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out the root and every
%   subsystem with shaddad_layout('tb_mcu_closed_loop', 'Recursive', true). Every signal line is named (L5): a line
%   keeps the first name a call gives it (a line into a bus-typed Bus Creator, BCB, without a name takes its
%   element's name, and each source feeds at most one input of a Bus Creator, so no two of its inputs share a name),
%   and local_name_rest names every line still without a name after its source block ('_k' for output k of a block
%   with several outputs, as shaddad_bu('nline')); a Bus Selector's output keeps '<element>'. One Bus Selector right
%   in front of each consumer of a bus (spec 6.2; one pick feeds a column of like blocks, e.g. the drop-mask bits).

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
mdl = 'tb_mcu_closed_loop';
f = fullfile(root, 'tests', 'electronics', [mdl '.slx']);
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
ins = {'op', 'Bus_OperatorCmd', 'Ts.ctrl'; 'flt', 'Bus_FaultInject', 'Ts.ctrl'; 'hmi', 'Bus_HmiInputs', 'Ts.ctrl'; ...
    'gnc', 'Bus_GncCmd', 'Ts.comm'; 'bms', 'Bus_BMS_Telemetry', 'Ts.hk'};
for k = 1:size(ins, 1)
    A(mdl, 'simulink/Sources/In1', ins{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ['Bus: ' ins{k, 2}], ...
        'SampleTime', ins{k, 3}, 'BusOutputAsStruct', 'on');
end
A(mdl, 'simulink/Sources/In1', 'plant', 'Port', '6', 'OutDataTypeStr', 'double', 'SampleTime', 'Ts.ctrl', 'PortDimensions', '9');
% ---- RC link (T07) ----
A(mdl, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Link', 'ReferencedSubsystem', 'RC_Link_Sim');
L(mdl, 'op/1', 'RC_Link/1', 'op'); L(mdl, 'flt/1', 'RC_Link/2', 'flt');
RTL(mdl, 'RT_hmi', 'Ts.base', 'P.ic.Bus_HmiInputs', 'hmi/1', '', 'hmi');
RTL(mdl, 'RT_flt_1k', 'Ts.base', 'P.ic.Bus_FaultInject', 'flt/1', '', '');
RTL(mdl, 'RT_plant_5', 'Ts.imu', '[1 0 0 0 0 0 0 0 0]', 'plant/1', '', 'plant');
RTL(mdl, 'RT_plant_50', 'Ts.comm', '[1 0 0 0 0 0 0 0 0]', 'plant/1', '', '');
RTL(mdl, 'RT_plant_1k', 'Ts.base', '[1 0 0 0 0 0 0 0 0]', 'plant/1', '', '');
RTL(mdl, 'RT_flt_50', 'Ts.comm', 'P.ic.Bus_FaultInject', 'flt/1', '', '');
RTL(mdl, 'RT_flt_100', 'Ts.hk', 'P.ic.Bus_FaultInject', 'flt/1', '', '');
% ---- MCU ----
A(mdl, 'simulink/Ports & Subsystems/Model', 'MCU', 'ModelName', 'Shaddad_MCU', 'SimulationMode', 'Normal');
local_safety(mdl);
local_mcu_in(mdl);
local_drive_rx(mdl);
local_plant(mdl);
local_imu(mdl);
local_esc_fb(mdl);
local_bms_tx(mdl);
local_comp_tx(mdl);
% safety emulation (and the implement coding plug, plant(7))
L(mdl, 'RT_hmi/1', 'Safety_Emu/1', 'hmi_1k'); L(mdl, 'RT_flt_1k/1', 'Safety_Emu/2', 'flt_1k'); L(mdl, 'MCU/1', 'Safety_Emu/3', 'out1k');
% MCU inputs
L(mdl, 'RC_Link/1', 'MCU_In/1', 'sbus'); L(mdl, 'Safety_Emu/1', 'MCU_In/2', 'di_word'); L(mdl, 'flt/1', 'MCU_In/3');
L(mdl, 'IMU_Emu/1', 'MCU_In/4', 'imu'); L(mdl, 'ESC_Fb/1', 'MCU_In/5', 'can1_esc'); L(mdl, 'ESC_Fb/2', 'MCU_In/6', 'can1_escslow');
L(mdl, 'COMP_Tx/1', 'MCU_In/7', 'can2_comp'); L(mdl, 'BMS_Tx/1', 'MCU_In/8', 'can1_bms');
mi = {'in1k', 'in5', 'in10', 'in50', 'in100'};
for k = 1:5, L(mdl, sprintf('MCU_In/%d', k), sprintf('MCU/%d', k), mi{k}); end
% plant loop
L(mdl, 'MCU/3', 'Drive_Rx/1', 'out10'); L(mdl, 'flt/1', 'Drive_Rx/2');
L(mdl, 'Drive_Rx/1', 'Plant/1', 'cmd'); L(mdl, 'RT_plant_1k/1', 'Plant/2', 'plant_1k'); L(mdl, 'RT_plant_1k/1', 'Safety_Emu/4');
L(mdl, 'Plant/1', 'IMU_Emu/1', 'v_LR'); L(mdl, 'Plant/2', 'IMU_Emu/2', 'yaw_rate'); L(mdl, 'RT_plant_5/1', 'IMU_Emu/3', 'plant_5');
L(mdl, 'flt/1', 'IMU_Emu/4');
L(mdl, 'Plant/1', 'ESC_Fb/1'); L(mdl, 'Drive_Rx/1', 'ESC_Fb/2'); L(mdl, 'flt/1', 'ESC_Fb/3'); L(mdl, 'RT_flt_50/1', 'ESC_Fb/4', 'flt_50');
L(mdl, 'plant/1', 'ESC_Fb/5');
L(mdl, 'bms/1', 'BMS_Tx/1', 'bms'); L(mdl, 'RT_flt_100/1', 'BMS_Tx/2', 'flt_100');
L(mdl, 'gnc/1', 'COMP_Tx/1', 'gnc'); L(mdl, 'Plant/1', 'COMP_Tx/2'); L(mdl, 'RT_plant_50/1', 'COMP_Tx/3', 'plant_50');
L(mdl, 'RT_flt_50/1', 'COMP_Tx/4');
% ---- outputs ----
ro = {'out1k', 'MCU/1', 'Bus: Bus_McuOut1k'; 'out10', 'MCU/3', 'Bus: Bus_McuOut10'; 'out50', 'MCU/4', 'Bus: Bus_McuOut50'; ...
    'telem', 'MCU/6', 'Bus: Bus_FwTelemetry'; 'cmd', 'Drive_Rx/1', 'Bus: Bus_DriveCmd'; ...
    'rc_frame', 'RC_Link/2', 'Bus: Bus_RcFrame'; 'state', 'Plant/3', 'double'; 'di_word', 'Safety_Emu/1', 'uint16'};
for k = 1:size(ro, 1)
    A(mdl, 'simulink/Sinks/Out1', ro{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ro{k, 3});
    L(mdl, ro{k, 2}, [ro{k, 1} '/1'], ro{k, 1});
end
A(mdl, 'simulink/Sinks/Terminator', 'T_out5'); L(mdl, 'MCU/2', 'T_out5/1', 'out5');
A(mdl, 'simulink/Sinks/Terminator', 'T_out100'); L(mdl, 'MCU/5', 'T_out100/1', 'out100');
L(mdl, 'Safety_Emu/2', 'Plant/3', 'relay');
local_name_rest(mdl);
set_param(mdl, 'Description', ['tb_mcu_closed_loop (T09.6): RC_Link_Sim -> Shaddad_MCU closed over a simplified plant ' ...
    '(first-order ESC speed loops, Differential Drive Kinematic Model with a yaw-gain factor, IMU emulation with ' ...
    'tilt, a one-LSB magnetometer toggle and the stuck fault flt.imu_fault 1), CAN pass-through with the T04/T06/T08 ' ...
    'codecs, constant BMS frames and a safety-chain emulation (SR latch, ' ...
    'reset stretch, 1 ms DI delay). in 1 op Bus_OperatorCmd, 2 flt Bus_FaultInject, 3 hmi Bus_HmiInputs (0.01), 4 gnc ' ...
    'Bus_GncCmd (0.05), 5 bms Bus_BMS_Telemetry (0.1), 6 plant double[9] [yaw_gain pitch roll ground_ratio a_run ' ...
    'brake_fail impl m f_rr] (0.01; a_run runaway load in m/s^2, brake_fail b0 L / b1 R, impl the DI IMPL_ID code, ' ...
    'm > 0 kg the physical plant Track_dyn through the DRV_CMD torque limits with the slope load and f_rr). ' ...
    'out 1 out1k, 2 out10, 3 out50, 4 telem, 5 cmd Bus_DriveCmd, 6 rc_frame, 7 state double[8] [v_L v_R v_mean ' ...
    'yaw_rate x y psi relay] (0.001), 8 di_word uint16 (0.001). Built by scripts/build/build_tb_mcu_closed_loop.m; ' ...
    'run with tests/electronics/mcu_cl_run.m.']);
shaddad_bu('save', mdl, f);
shaddad_bu('finalize', mdl, f);
close_system(mdl, 0);
end

% ===========================================================================================
function local_safety(mdl)
s = SUB(mdl, 'Safety_Emu', {'hmi', 'flt', 'out1k', 'plant'}, {'di_word', 'relay'});
% one pick of hmi right in front of each consumer (spec 6.2); Sel_do feeds the pair of DO bit decoders
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt', 'OutputSignals', 'estop_ch2_stuck'); L(s, 'flt/1', 'Sel_flt/1', 'flt');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_do', 'OutputSignals', 'do_word'); L(s, 'out1k/1', 'Sel_do/1', 'out1k');
LOGIC(s, 'Estop_any', 'OR', 3);
PICK(s, 'hmi/1', 'hmi', 'Sel_hmi_estop', {'estop_front_pressed', 'estop_rear_pressed', 'wireless_estop_tripped'}, ...
    {'Estop_any/1', 'Estop_any/2', 'Estop_any/3'});
LOGIC(s, 'Ch1_ok', 'NOT', 1); L(s, 'Estop_any/1', 'Ch1_ok/1', 'estop_any');
LOGIC(s, 'Ch2_ok', 'OR', 2); L(s, 'Ch1_ok/1', 'Ch2_ok/1', 'ch1_ok'); L(s, 'Sel_flt/1', 'Ch2_ok/2');
LOGIC(s, 'Bumper', 'OR', 2);
PICK(s, 'hmi/1', 'hmi', 'Sel_hmi_bumper', {'bumper_front', 'bumper_rear'}, {'Bumper/1', 'Bumper/2'});
LOGIC(s, 'No_bumper', 'NOT', 1); L(s, 'Bumper/1', 'No_bumper/1', 'bumper');
LOGIC(s, 'Key_on', 'NOT', 1); PICK(s, 'hmi/1', 'hmi', 'Sel_hmi_key', {'key_off'}, {'Key_on/1'});
LOGIC(s, 'Lid_closed', 'NOT', 1); PICK(s, 'hmi/1', 'hmi', 'Sel_hmi_lid', {'lid_open'}, {'Lid_closed/1'});
for b = {'DO0', '0'; 'DO1', '1'}'
    A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', [b{1} '_mask'], 'logicop', 'AND', 'UseBitMask', 'on', ...
        'BitMask', ['2^' b{2}], 'BitMaskRealWorld', 'Stored Integer');
    A(s, 'simulink/Logic and Bit Operations/Compare To Zero', b{1}, 'relop', '~=', 'OutDataTypeStr', 'boolean');
    L(s, 'Sel_do/1', [b{1} '_mask/1']); L(s, [b{1} '_mask/1'], [b{1} '/1']);
end
LOGIC(s, 'Loop_ok', 'AND', 4); L(s, 'Ch1_ok/1', 'Loop_ok/1'); L(s, 'No_bumper/1', 'Loop_ok/2');
L(s, 'DO0/1', 'Loop_ok/3', 'do0'); L(s, 'Key_on/1', 'Loop_ok/4', 'key_on');
LOGIC(s, 'Reset_src', 'OR', 2); PICK(s, 'hmi/1', 'hmi', 'Sel_hmi_reset', {'reset_btn'}, {'Reset_src/1'});
L(s, 'DO1/1', 'Reset_src/2', 'do1');
A(s, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Reset_edge', 'vinit', '0'); L(s, 'Reset_src/1', 'Reset_edge/1');
A(s, 'simulink/Discrete/Tapped Delay', 'Reset_taps', 'NumDelays', 'round(P.power.safety.t_reset_hold/Ts.base) - 1', ...
    'includeCurrent', 'on', 'vinit', '0', 'samptime', '-1');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Reset_pulse', 'Operator', 'OR', 'Inputs', '1');
L(s, 'Reset_edge/1', 'Reset_taps/1', 'reset_edge'); L(s, 'Reset_taps/1', 'Reset_pulse/1');
LOGIC(s, 'Set', 'AND', 2); L(s, 'Loop_ok/1', 'Set/1', 'loop_ok'); L(s, 'Reset_pulse/1', 'Set/2', 'reset_pulse');
A(s, 'simulink/Discrete/Unit Delay', 'Q_prev', 'InitialCondition', 'false', 'SampleTime', '-1');
LOGIC(s, 'Set_or_hold', 'OR', 2); L(s, 'Set/1', 'Set_or_hold/1'); L(s, 'Q_prev/1', 'Set_or_hold/2', 'q_prev');
LOGIC(s, 'Q', 'AND', 2); L(s, 'Set_or_hold/1', 'Q/1'); L(s, 'Loop_ok/1', 'Q/2');      % reset dominant
L(s, 'Q/1', 'Q_prev/1', 'relay');
A(s, 'simulink/Discrete/Unit Delay', 'DI_delay', 'InitialCondition', 'false', 'SampleTime', '-1');
L(s, 'Q/1', 'DI_delay/1');
% DI word (ICD 6): b0 ESTOP_CH1_OK b1 ESTOP_CH2_OK b2 SAFETY_OK b3/b4 bumpers b5 RESET_BTN b8/b9 IMPL_ID0/1 b10
% LID_CLOSED b11 KEY_ON b12 SAFETY_HV_OK (other bits 0: no K1/K2 aux, not docked). IMPL_ID = the implement coding plug,
% plant(7) (0 none, 1 mower, 2 tiller, 3 sprayer; descent task, 2026-11-15: 0 before)
CST(s, 'F2', 'false(1,2)', 'boolean', 'Ts.base'); CST(s, 'F3', 'false(1,3)', 'boolean', 'Ts.base');
A(s, 'simulink/Signal Routing/Selector', 'Impl_sel', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '7', 'InputPortWidth', '9');
L(s, 'plant/1', 'Impl_sel/1', 'plant');
DTC(s, 'Impl_u8', 'uint8'); L(s, 'Impl_sel/1', 'Impl_u8/1', 'impl');
for b = {'Impl_id0', '1'; 'Impl_id1', '2'}'
    A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', [b{1} '_mask'], 'logicop', 'AND', 'UseBitMask', 'on', ...
        'BitMask', b{2}, 'BitMaskRealWorld', 'Stored Integer');
    A(s, 'simulink/Logic and Bit Operations/Compare To Zero', b{1}, 'relop', '~=', 'OutDataTypeStr', 'boolean');
    L(s, 'Impl_u8/1', [b{1} '_mask/1'], 'impl_code'); L(s, [b{1} '_mask/1'], [b{1} '/1']);
end
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Di_bits', 'NumInputs', '13');
SELB(s, 'hmi/1', 'hmi', 'Sel_hmi_di', {'bumper_front', 'bumper_rear', 'reset_btn'});      % DI bits 3-5, inputs 4-6
src = {'Ch1_ok/1', 'Ch2_ok/1', 'DI_delay/1', 'Sel_hmi_di/1', 'Sel_hmi_di/2', 'Sel_hmi_di/3', 'F2/1', 'Impl_id0/1', ...
    'Impl_id1/1', 'Lid_closed/1', 'Key_on/1', 'DI_delay/1', 'F3/1'};
for k = 1:numel(src), L(s, src{k}, sprintf('Di_bits/%d', k)); end
A(s, 'commutil2/Bit to Integer Converter', 'Di_pack', 'nbits', '16', 'bitOrder', 'LSB first', 'outDtype', 'uint16');
L(s, 'Di_bits/1', 'Di_pack/1', 'di_bits'); L(s, 'Di_pack/1', 'di_word/1', 'di_word'); L(s, 'DI_delay/1', 'relay/1', 'safety_ok');
end

function local_mcu_in(mdl)
s = SUB(mdl, 'MCU_In', {'sbus', 'di_word', 'flt', 'imu', 'can1_esc', 'can1_escslow', 'can2_comp', 'can1_bms'}, ...
    {'in1k', 'in5', 'in10', 'in50', 'in100'});
bc = {'in1k', 'McuIn1k', 5; 'in5', 'McuIn5', 1; 'in10', 'McuIn10', 1; 'in50', 'McuIn50', 5; 'in100', 'McuIn100', 1};
for k = 1:size(bc, 1)
    A(s, 'simulink/Signal Routing/Bus Creator', ['BC_' bc{k, 1}], 'Inputs', num2str(bc{k, 3}), ...
        'OutDataTypeStr', ['Bus: Bus_' bc{k, 2}], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
    L(s, ['BC_' bc{k, 1} '/1'], [bc{k, 1} '/1'], bc{k, 1});
end
L(s, 'sbus/1', 'BC_in1k/1', 'sbus'); L(s, 'di_word/1', 'BC_in1k/2', 'di_word');
A(s, 'simulink/Sources/Digital Clock', 'Clock_t', 'SampleTime', 'Ts.base');
A(s, 'simulink/Math Operations/Gain', 'Gain_us', 'Gain', '1e6');
A(s, 'simulink/Math Operations/Rounding Function', 'Round_us', 'Operator', 'round');
A(s, 'simulink/Math Operations/Math Function', 'Mod_2p32', 'Operator', 'mod');
CST(s, 'C_2p32', '2^32', 'double', 'Ts.base');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'DTC_t_us', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'off');
L(s, 'Clock_t/1', 'Gain_us/1'); L(s, 'Gain_us/1', 'Round_us/1'); L(s, 'Round_us/1', 'Mod_2p32/1');
L(s, 'C_2p32/1', 'Mod_2p32/2'); L(s, 'Mod_2p32/1', 'DTC_t_us/1'); L(s, 'DTC_t_us/1', 'BC_in1k/3', 't_us');
CST(s, 'Reset_flag', 'false', 'boolean', 'Ts.base'); L(s, 'Reset_flag/1', 'BC_in1k/4', 'reset_flag');
% fault_inject = task_stall | can1_busoff << 4 | can2_busoff << 5 | (imu_fault == 3) << 6 (as Shaddad_System/MCU/MCU_In);
% one pick of flt right in front of each consumer (spec 6.2)
DTC(s, 'DTC_stall', 'uint32');
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Mask_stall', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '15', 'BitMaskRealWorld', 'Stored Integer');
PICK(s, 'flt/1', 'flt', 'Sel_flt_stall', {'task_stall'}, {'DTC_stall/1'}); L(s, 'DTC_stall/1', 'Mask_stall/1');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'IMU_dropout', 'relop', '==', 'const', '3', 'OutDataTypeStr', 'boolean');
PICK(s, 'flt/1', 'flt', 'Sel_flt_imu', {'imu_fault'}, {'IMU_dropout/1'});
sh = {'can1_busoff', 'Sel_flt_can1', 4; 'can2_busoff', 'Sel_flt_can2', 5; 'imu_dropout', '', 6};
for k = 1:size(sh, 1)
    DTC(s, ['DTC_' sh{k, 1}], 'uint32');
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
RT(s, 'RT_fi', 'Ts.base', '0'); L(s, 'OR_fi/1', 'RT_fi/1', 'fault_inject'); L(s, 'RT_fi/1', 'BC_in1k/5', 'fault_inject');
L(s, 'imu/1', 'BC_in5/1', 'imu'); L(s, 'can1_esc/1', 'BC_in10/1', 'can1_esc');
L(s, 'can1_escslow/1', 'BC_in50/1', 'can1_escslow');
CST(s, 'Impl', 'P.ic.Bus_Can1_Impl', 'Bus: Bus_Can1_Impl', 'Ts.comm'); L(s, 'Impl/1', 'BC_in50/2', 'can1_impl');
L(s, 'can2_comp/1', 'BC_in50/3', 'can2_comp');
% ADC counts for 51.2 V pack / bus, 0 A, 24 / 12 / 5 V rails, NTC at 318 K, lift pot 0.5 (as tb_mcu_rc)
CST(s, 'Adc', ['round([51.2/P.electronics.adc.k_vpack 51.2/P.electronics.adc.k_vlink P.electronics.adc.i_off ' ...
    '24/P.electronics.adc.k_v24 12/P.electronics.adc.k_v12 5/P.electronics.adc.k_v5 0 0.5*P.electronics.adc.vref]' ...
    '/P.electronics.adc.vref*65535) + [0 0 0 0 0 0 round(interp1(P.electronics.adc.ntc_lut_K, ' ...
    'P.electronics.adc.ntc_lut_counts, 318.15)) 0]'], 'uint16', 'Ts.comm');
CST(s, 'T_mcu', '40', 'single', 'Ts.comm');
L(s, 'Adc/1', 'BC_in50/4', 'adc_raw'); L(s, 'T_mcu/1', 'BC_in50/5', 'mcu_temp_degC');
L(s, 'can1_bms/1', 'BC_in100/1', 'can1_bms');
end

function local_drive_rx(mdl)
s = SUB(mdl, 'Drive_Rx', {'out10', 'flt'}, {'cmd'});
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel', 'OutputSignals', 'can1_drv.drv_cmd_L,can1_drv.drv_cmd_R');
L(s, 'out10/1', 'Sel/1', 'out10');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt', 'OutputSignals', 'can1_drop_mask'); L(s, 'flt/1', 'Sel_flt/1', 'flt');
A(s, 'simulink/Signal Routing/Bus Creator', 'Cmd_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_DriveCmd', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for sd = {'L', 1, 'drv_cmd_L'; 'R', 2, 'drv_cmd_R'}'
    DROP(s, ['Drop_' sd{1}], sprintf('Sel/%d', sd{2}), 'Sel_flt/1', sprintf('P.electronics.can.drop_bit.%s', sd{3}));
    A(s, 'simulink/Ports & Subsystems/Subsystem Reference', ['Unpack_' sd{1}], 'ReferencedSubsystem', 'drive_cmd_unpack');
    CST(s, ['Node_' sd{1}], num2str(sd{2}), 'uint8', 'Ts.ctrl');
    L(s, ['Drop_' sd{1} '/1'], ['Unpack_' sd{1} '/1']); L(s, ['Node_' sd{1} '/1'], ['Unpack_' sd{1} '/2']);
    L(s, ['Unpack_' sd{1} '/1'], sprintf('Cmd_BC/%d', sd{2}), sd{1});
end
L(s, 'Cmd_BC/1', 'cmd/1', 'cmd');
end

function local_plant(mdl)
s = SUB(mdl, 'Plant', {'cmd', 'plant', 'relay'}, {'v_LR', 'yaw_rate', 'state'});
A(s, 'simulink/Signal Routing/Demux', 'Plant_dmx', 'Outputs', '9'); L(s, 'plant/1', 'Plant_dmx/1', 'plant');
% runaway load (plant(5) a_run, m/s^2; bench fault of the D71 stop tests, 2026-11-15): a load that the drives cannot
% brake and the brakes cannot hold. While a_run > 0 each track's ESC lag is fed max(v_cmd, v_prev + a_run*lead): the
% input lead that makes the first-order lag accelerate at a_run (lead = Ts/(1 - exp(-Ts/0.05)), forward motion)
A(s, 'simulink/Math Operations/Gain', 'A_run_lead', 'Gain', 'Ts.base/(1 - exp(-Ts.base/0.05))');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Run_on', 'relop', '>', 'OutDataTypeStr', 'boolean');
L(s, 'Plant_dmx/5', 'A_run_lead/1', 'a_run'); L(s, 'Plant_dmx/5', 'Run_on/1');
% one pick of cmd right in front of each consumer (spec 6.2)
for sd = {'L', 1; 'R', 3}'
    A(s, 'simulink/Math Operations/Gain', ['V_cmd_' sd{1}], 'Gain', '1/P.electronics.drv.k_v2w');
    DTC(s, ['V_cmd_d_' sd{1}], 'double');
    CST(s, ['Zero_' sd{1}], '0', 'double', 'Ts.ctrl');
    A(s, 'simulink/Signal Routing/Switch', ['En_' sd{1}], 'Criteria', 'u2 ~= 0');
    PICK(s, 'cmd/1', 'cmd', ['Sel_w_ref_' sd{1}], {[sd{1} '.w_ref']}, {['V_cmd_' sd{1} '/1']});
    L(s, ['V_cmd_' sd{1} '/1'], ['V_cmd_d_' sd{1} '/1']);
    L(s, ['V_cmd_d_' sd{1} '/1'], ['En_' sd{1} '/1']);
    PICK(s, 'cmd/1', 'cmd', ['Sel_enable_' sd{1}], {[sd{1} '.enable']}, {['En_' sd{1} '/2']});
    L(s, ['Zero_' sd{1} '/1'], ['En_' sd{1} '/3']);
    RT(s, ['RT_v_' sd{1}], 'Ts.base', '0'); L(s, ['En_' sd{1} '/1'], ['RT_v_' sd{1} '/1'], ['v_cmd_' sd{1}]);
    A(s, 'simulink/Discrete/Transfer Fcn First Order', ['Esc_' sd{1}], 'PoleZ', 'exp(-Ts.base/0.05)', 'ICPrevOutput', '0');
    A(s, 'simulink/Discrete/Unit Delay', ['V_prev_' sd{1}], 'InitialCondition', '0', 'SampleTime', 'Ts.base');
    A(s, 'simulink/Math Operations/Sum', ['V_run_' sd{1}], 'Inputs', '++');
    A(s, 'simulink/Math Operations/MinMax', ['V_up_' sd{1}], 'Function', 'max', 'Inputs', '2');
    A(s, 'simulink/Signal Routing/Switch', ['V_in_' sd{1}], 'Criteria', 'u2 ~= 0');
    L(s, ['Esc_' sd{1} '/1'], ['V_prev_' sd{1} '/1']); L(s, ['V_prev_' sd{1} '/1'], ['V_run_' sd{1} '/1'], ['v_prev_' sd{1}]);
    L(s, 'A_run_lead/1', ['V_run_' sd{1} '/2'], 'a_run_lead');
    L(s, ['RT_v_' sd{1} '/1'], ['V_up_' sd{1} '/1']); L(s, ['V_run_' sd{1} '/1'], ['V_up_' sd{1} '/2'], ['v_run_' sd{1}]);
    L(s, ['V_up_' sd{1} '/1'], ['V_in_' sd{1} '/1'], ['v_up_' sd{1}]); L(s, 'Run_on/1', ['V_in_' sd{1} '/2'], 'run_on');
    L(s, ['RT_v_' sd{1} '/1'], ['V_in_' sd{1} '/3']);
    L(s, ['V_in_' sd{1} '/1'], ['Esc_' sd{1} '/1'], ['v_in_' sd{1}]);
end
A(s, 'simulink/Signal Routing/Vector Concatenate', 'V_LR', 'NumInputs', '2');
L(s, 'Esc_L/1', 'V_LR/1', 'v_L'); L(s, 'Esc_R/1', 'V_LR/2', 'v_R');
% physical mode (plant(8) mass m > 0; descent task, 2026-11-15): each track's ESC speed loop acts through the decoded
% DRV_CMD torque limits (tau_lim_mot, and tau_lim_reg, which the BMS CCL limits) on m/2, with the slope load g sin(pitch)
% (plant(2), + nose down) and the rolling resistance plant(9) g cos(pitch); the drive off is an ideal brake (the decay to
% 0 of the first-order plant). MATLAB Function Track_dyn -> tests/electronics/mcu_cl_track_dyn.m (state output, no direct
% feedthrough). m 0 (the default) keeps the first-order plant above, so the tests before 2026-11-15 run unchanged
A(s, 'simulink/Signal Routing/Vector Concatenate', 'En_LR', 'NumInputs', '2');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Tlim', 'NumInputs', '4');
PICK(s, 'cmd/1', 'cmd', 'Sel_dyn_en', {'L.enable', 'R.enable'}, {'En_LR/1', 'En_LR/2'});
PICK(s, 'cmd/1', 'cmd', 'Sel_dyn_tlim', {'L.tau_lim_mot', 'R.tau_lim_mot', 'L.tau_lim_reg', 'R.tau_lim_reg'}, ...
    {'Tlim/1', 'Tlim/2', 'Tlim/3', 'Tlim/4'});
RT(s, 'RT_en', 'Ts.base', 'false'); L(s, 'En_LR/1', 'RT_en/1', 'enable_LR');
DTC(s, 'Tlim_d', 'double'); L(s, 'Tlim/1', 'Tlim_d/1', 'tau_lim');
RT(s, 'RT_tlim', 'Ts.base', '[0 0 0 0]'); L(s, 'Tlim_d/1', 'RT_tlim/1', 'tau_lim_d');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'V_cmd_LR', 'NumInputs', '2');
L(s, 'RT_v_L/1', 'V_cmd_LR/1'); L(s, 'RT_v_R/1', 'V_cmd_LR/2');
mf = A(s, 'simulink/User-Defined Functions/MATLAB Function', 'Track_dyn');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = sprintf(['function v = fcn(v_cmd, en, tlim, pl, prm)\n%%#codegen\n' ...
    '%% physical track speeds of the bench (tests/electronics/mcu_cl_track_dyn.m)\n' ...
    'v = mcu_cl_track_dyn(v_cmd, en, tlim, pl, prm);\n']);
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
if isprop(cfg, 'AllowDirectFeedthrough'), cfg.AllowDirectFeedthrough = false; end
CST(s, 'Dyn_prm', '[Ts.base 0.05 P.frames.g P.electronics.drv.k_v2w]', 'double', 'Ts.base');
L(s, 'V_cmd_LR/1', 'Track_dyn/1', 'v_cmd_LR'); L(s, 'RT_en/1', 'Track_dyn/2'); L(s, 'RT_tlim/1', 'Track_dyn/3');
L(s, 'plant/1', 'Track_dyn/4'); L(s, 'Dyn_prm/1', 'Track_dyn/5', 'dyn_prm');
A(s, 'simulink/Signal Routing/Switch', 'V_sel', 'Criteria', 'u2 > Threshold', 'Threshold', '0');
L(s, 'Track_dyn/1', 'V_sel/1', 'v_dyn'); L(s, 'Plant_dmx/8', 'V_sel/2', 'mass'); L(s, 'V_LR/1', 'V_sel/3', 'v_lag');
L(s, 'V_sel/1', 'v_LR/1', 'v_LR');
A(s, 'simulink/Signal Routing/Demux', 'V_sel_dmx', 'Outputs', '2'); L(s, 'V_sel/1', 'V_sel_dmx/1');
% kinematics: wheel speeds [v_mean - g*dv, v_mean + g*dv] (yaw-gain factor g = plant(1)), TrackWidth chi*B
% pitch, roll, ground ratio, brake_fail, the implement code and f_rr are not used here by blocks (Track_dyn reads the
% whole plant vector; IMU_Emu, COMP_Tx, ESC_Fb and Safety_Emu read the others): terminated
A(s, 'simulink/Sinks/Terminator', 'T_pitch'); A(s, 'simulink/Sinks/Terminator', 'T_roll');
A(s, 'simulink/Sinks/Terminator', 'T_ratio'); A(s, 'simulink/Sinks/Terminator', 'T_brake_fail');
A(s, 'simulink/Sinks/Terminator', 'T_impl'); A(s, 'simulink/Sinks/Terminator', 'T_frr');
L(s, 'Plant_dmx/2', 'T_pitch/1', 'pitch'); L(s, 'Plant_dmx/3', 'T_roll/1', 'roll'); L(s, 'Plant_dmx/4', 'T_ratio/1', 'ground_ratio');
L(s, 'Plant_dmx/6', 'T_brake_fail/1', 'brake_fail');
L(s, 'Plant_dmx/7', 'T_impl/1', 'impl'); L(s, 'Plant_dmx/9', 'T_frr/1', 'f_rr');
A(s, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '++'); A(s, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5');
L(s, 'V_sel_dmx/1', 'V_sum/1', 'v_L_sel'); L(s, 'V_sel_dmx/2', 'V_sum/2', 'v_R_sel'); L(s, 'V_sum/1', 'V_mean/1');
A(s, 'simulink/Math Operations/Sum', 'Dv', 'Inputs', '-+'); A(s, 'simulink/Math Operations/Gain', 'Dv_half', 'Gain', '0.5');
L(s, 'V_sel_dmx/1', 'Dv/1'); L(s, 'V_sel_dmx/2', 'Dv/2'); L(s, 'Dv/1', 'Dv_half/1');
A(s, 'simulink/Math Operations/Product', 'Dv_g', 'Inputs', '**'); L(s, 'Dv_half/1', 'Dv_g/1'); L(s, 'Plant_dmx/1', 'Dv_g/2', 'yaw_gain');
A(s, 'simulink/Math Operations/Sum', 'W_L', 'Inputs', '+-'); A(s, 'simulink/Math Operations/Sum', 'W_R', 'Inputs', '++');
L(s, 'V_mean/1', 'W_L/1', 'v_mean'); L(s, 'Dv_g/1', 'W_L/2', 'dv'); L(s, 'V_mean/1', 'W_R/1'); L(s, 'Dv_g/1', 'W_R/2');
A(s, 'robotmobilelib/Differential Drive Kinematic Model', 'Kinematics', 'VehicleInputs', 'Wheel Speeds', ...
    'WheelRadius', '1', 'TrackWidth', 'P.electronics.mix.chi*P.electronics.mix.B', 'SimulateUsing', 'Code generation');
L(s, 'W_L/1', 'Kinematics/1', 'w_L'); L(s, 'W_R/1', 'Kinematics/2', 'w_R');
A(s, 'simulink/Signal Routing/Demux', 'StateDot', 'Outputs', '3'); L(s, 'Kinematics/2', 'StateDot/1', 'stateDot');
A(s, 'simulink/Sinks/Terminator', 'T_xd'); A(s, 'simulink/Sinks/Terminator', 'T_yd');
L(s, 'StateDot/1', 'T_xd/1'); L(s, 'StateDot/2', 'T_yd/1');
RT(s, 'RT_yaw', 'Ts.base', '0'); L(s, 'StateDot/3', 'RT_yaw/1', 'yaw_rate_c');
A(s, 'simulink/Math Operations/Reshape', 'Pose_1d', 'OutputDimensionality', '1-D array');
L(s, 'Kinematics/1', 'Pose_1d/1', 'pose');
RT(s, 'RT_pose', 'Ts.base', '[0 0 0]'); L(s, 'Pose_1d/1', 'RT_pose/1');
L(s, 'RT_yaw/1', 'yaw_rate/1', 'yaw_rate');
DTC(s, 'Relay_d', 'double'); L(s, 'relay/1', 'Relay_d/1', 'relay');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'State', 'NumInputs', '5');
L(s, 'V_sel/1', 'State/1'); L(s, 'V_mean/1', 'State/2'); L(s, 'RT_yaw/1', 'State/3'); L(s, 'RT_pose/1', 'State/4');
L(s, 'Relay_d/1', 'State/5');
L(s, 'State/1', 'state/1', 'state');
end

function local_imu(mdl)
s = SUB(mdl, 'IMU_Emu', {'v_LR', 'yaw_rate', 'plant', 'flt'}, {'imu'});
A(s, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(s, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5');
L(s, 'v_LR/1', 'V_sum/1', 'v_LR'); L(s, 'V_sum/1', 'V_mean/1');
A(s, 'simulink/Discrete/Discrete Derivative', 'Dvdt', 'gainval', '1.0', 'ICPrevScaledInput', '0');
L(s, 'V_mean/1', 'Dvdt/1', 'v_mean');
RT(s, 'RT_a', 'Ts.imu', '0'); RT(s, 'RT_v', 'Ts.imu', '0'); RT(s, 'RT_r', 'Ts.imu', '0');
L(s, 'Dvdt/1', 'RT_a/1', 'a_x'); L(s, 'V_mean/1', 'RT_v/1'); L(s, 'yaw_rate/1', 'RT_r/1', 'yaw_rate');
A(s, 'simulink/Signal Routing/Demux', 'Plant_dmx', 'Outputs', '9'); L(s, 'plant/1', 'Plant_dmx/1', 'plant');
A(s, 'simulink/Sinks/Terminator', 'T_g'); A(s, 'simulink/Sinks/Terminator', 'T_ratio');
A(s, 'simulink/Sinks/Terminator', 'T_a_run'); A(s, 'simulink/Sinks/Terminator', 'T_brake_fail');
A(s, 'simulink/Sinks/Terminator', 'T_impl'); A(s, 'simulink/Sinks/Terminator', 'T_mass');
A(s, 'simulink/Sinks/Terminator', 'T_frr');
L(s, 'Plant_dmx/1', 'T_g/1', 'yaw_gain'); L(s, 'Plant_dmx/4', 'T_ratio/1', 'ground_ratio');
L(s, 'Plant_dmx/5', 'T_a_run/1', 'a_run'); L(s, 'Plant_dmx/6', 'T_brake_fail/1', 'brake_fail');
L(s, 'Plant_dmx/7', 'T_impl/1', 'impl'); L(s, 'Plant_dmx/8', 'T_mass/1', 'mass'); L(s, 'Plant_dmx/9', 'T_frr/1', 'f_rr');
for a = {'pitch', 2; 'roll', 3}'
    A(s, 'simulink/Math Operations/Trigonometric Function', ['Sin_' a{1}], 'Operator', 'sin');
    A(s, 'simulink/Math Operations/Trigonometric Function', ['Cos_' a{1}], 'Operator', 'cos');
    L(s, sprintf('Plant_dmx/%d', a{2}), ['Sin_' a{1} '/1'], a{1}); L(s, sprintf('Plant_dmx/%d', a{2}), ['Cos_' a{1} '/1']);
end
% specific force, body FLU (pitch + = nose down): [a_x - g sin(pitch); v*r + g sin(roll) cos(pitch); g cos(roll) cos(pitch)]
A(s, 'simulink/Math Operations/Gain', 'G_sp', 'Gain', 'P.frames.g'); L(s, 'Sin_pitch/1', 'G_sp/1');
A(s, 'simulink/Math Operations/Sum', 'Fx', 'Inputs', '+-'); L(s, 'RT_a/1', 'Fx/1'); L(s, 'G_sp/1', 'Fx/2');
A(s, 'simulink/Math Operations/Product', 'Vr', 'Inputs', '**'); L(s, 'RT_v/1', 'Vr/1'); L(s, 'RT_r/1', 'Vr/2');
A(s, 'simulink/Math Operations/Product', 'Sr_cp', 'Inputs', '**'); L(s, 'Sin_roll/1', 'Sr_cp/1'); L(s, 'Cos_pitch/1', 'Sr_cp/2');
A(s, 'simulink/Math Operations/Gain', 'G_sr', 'Gain', 'P.frames.g'); L(s, 'Sr_cp/1', 'G_sr/1');
A(s, 'simulink/Math Operations/Sum', 'Fy', 'Inputs', '++'); L(s, 'Vr/1', 'Fy/1'); L(s, 'G_sr/1', 'Fy/2');
A(s, 'simulink/Math Operations/Product', 'Cr_cp', 'Inputs', '**'); L(s, 'Cos_roll/1', 'Cr_cp/1'); L(s, 'Cos_pitch/1', 'Cr_cp/2');
A(s, 'simulink/Math Operations/Gain', 'Fz', 'Gain', 'P.frames.g'); L(s, 'Cr_cp/1', 'Fz/1');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Acc', 'NumInputs', '3');
L(s, 'Fx/1', 'Acc/1', 'f_x'); L(s, 'Fy/1', 'Acc/2', 'f_y'); L(s, 'Fz/1', 'Acc/3', 'f_z');
CST(s, 'Zero2', '[0 0]', 'double', 'Ts.imu');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Gyr', 'NumInputs', '2');
L(s, 'Zero2/1', 'Gyr/1'); L(s, 'RT_r/1', 'Gyr/2');
DTC(s, 'Acc_s', 'single'); DTC(s, 'Gyr_s', 'single');
L(s, 'Acc/1', 'Acc_s/1'); L(s, 'Gyr/1', 'Gyr_s/1');
% magnetometer with a one-LSB toggle on x every sample (P.sensors.imu.mag.Resolution, 0.15 uT on the ICM-20948): a real
% IMU is never bit-identical from one sample to the next, and sens_acq_imu marks the IMU invalid once all 9 raw axes
% repeat for safety.imu_frozen_t (R-05, 2026-11-15); the MCU control does not use the magnetometer
CST(s, 'Mag', '[20 0 -40]', 'single', 'Ts.imu'); CST(s, 'Temp', '35', 'single', 'Ts.imu');
A(s, 'simulink/Sources/Counter Limited', 'Mag_toggle', 'uplimit', '1', 'tsamp', 'Ts.imu');
DTC(s, 'Mag_toggle_s', 'single'); L(s, 'Mag_toggle/1', 'Mag_toggle_s/1', 'toggle');
CST(s, 'Mag_lsb', 'single(P.sensors.imu.mag.Resolution)*[1 0 0]', 'single', 'Ts.imu');
A(s, 'simulink/Math Operations/Product', 'Mag_dither', 'Inputs', '**');
L(s, 'Mag_toggle_s/1', 'Mag_dither/1', 'toggle_s'); L(s, 'Mag_lsb/1', 'Mag_dither/2', 'mag_lsb');
A(s, 'simulink/Math Operations/Sum', 'Mag_d', 'Inputs', '++');
L(s, 'Mag/1', 'Mag_d/1', 'mag'); L(s, 'Mag_dither/1', 'Mag_d/2', 'mag_dither');
% stuck IMU (flt.imu_fault 1, as sens_imu_model): the sample [acc gyro mag] is held (Switch Hold + Unit Delay Last)
RT(s, 'RT_flt', 'Ts.imu', '0');
PICK(s, 'flt/1', 'flt', 'Sel_flt_imu', {'imu_fault'}, {'RT_flt/1'});
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'Stuck', 'relop', '==', 'const', '1', ...
    'OutDataTypeStr', 'boolean');
L(s, 'RT_flt/1', 'Stuck/1', 'imu_fault');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Sample', 'NumInputs', '3');
L(s, 'Acc_s/1', 'Sample/1', 'acc'); L(s, 'Gyr_s/1', 'Sample/2', 'gyr'); L(s, 'Mag_d/1', 'Sample/3', 'mag_d');
A(s, 'simulink/Signal Routing/Switch', 'Hold', 'Criteria', 'u2 ~= 0');
A(s, 'simulink/Discrete/Unit Delay', 'Last', 'InitialCondition', '0', 'SampleTime', 'Ts.imu');
L(s, 'Last/1', 'Hold/1', 'last'); L(s, 'Stuck/1', 'Hold/2', 'stuck'); L(s, 'Sample/1', 'Hold/3', 'sample');
L(s, 'Hold/1', 'Last/1', 'imu9');
A(s, 'simulink/Signal Routing/Demux', 'Imu_dmx', 'Outputs', '[3 3 3]'); L(s, 'Hold/1', 'Imu_dmx/1');
A(s, 'simulink/Signal Routing/Bus Creator', 'BC_imu', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_ImuRaw', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(s, 'Imu_dmx/1', 'BC_imu/1', 'Acceleration'); L(s, 'Imu_dmx/2', 'BC_imu/2', 'AngularRate');
L(s, 'Imu_dmx/3', 'BC_imu/3', 'MagneticField'); L(s, 'Temp/1', 'BC_imu/4', 'Temperature');
L(s, 'BC_imu/1', 'imu/1', 'imu');
end

function local_esc_fb(mdl)
s = SUB(mdl, 'ESC_Fb', {'v_LR', 'cmd', 'flt', 'flt_50', 'plant'}, {'esc_fast', 'esc_slow'});
RT(s, 'RT_v', 'Ts.ctrl', '[0 0]'); L(s, 'v_LR/1', 'RT_v/1', 'v_LR');
A(s, 'simulink/Math Operations/Gain', 'Wm', 'Gain', 'P.electronics.drv.k_v2w'); L(s, 'RT_v/1', 'Wm/1');
% encoder counts: integral of the motor angle at 1 ms (counts = theta*cpr/(2 pi)), wrapped to int32
A(s, 'simulink/Math Operations/Gain', 'Cnt_rate', 'Gain', 'P.electronics.drv.k_v2w*P.electronics.drv.enc_cpr/(2*pi)');
L(s, 'v_LR/1', 'Cnt_rate/1');
A(s, 'simulink/Discrete/Discrete-Time Integrator', 'Cnt', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1.0', 'InitialCondition', '[0 0]', 'SampleTime', 'Ts.base');
L(s, 'Cnt_rate/1', 'Cnt/1', 'count_rate');
A(s, 'simulink/Math Operations/Rounding Function', 'Cnt_floor', 'Operator', 'floor'); L(s, 'Cnt/1', 'Cnt_floor/1', 'counts');
RT(s, 'RT_cnt', 'Ts.ctrl', '[0 0]'); L(s, 'Cnt_floor/1', 'RT_cnt/1');
A(s, 'simulink/Math Operations/Math Function', 'Cnt_mod', 'Operator', 'mod'); CST(s, 'C2p32', '2^32', 'double', 'Ts.ctrl');
L(s, 'RT_cnt/1', 'Cnt_mod/1'); L(s, 'C2p32/1', 'Cnt_mod/2');
CST(s, 'C2p31', '2^31', 'double', 'Ts.ctrl');
A(s, 'simulink/Logic and Bit Operations/Relational Operator', 'Cnt_hi', 'Operator', '>='); L(s, 'Cnt_mod/1', 'Cnt_hi/1'); L(s, 'C2p31/1', 'Cnt_hi/2');
DTC(s, 'Cnt_hi_d', 'double'); L(s, 'Cnt_hi/1', 'Cnt_hi_d/1');
A(s, 'simulink/Math Operations/Gain', 'Cnt_wrap', 'Gain', '2^32'); L(s, 'Cnt_hi_d/1', 'Cnt_wrap/1');
A(s, 'simulink/Math Operations/Sum', 'Cnt_s', 'Inputs', '+-'); L(s, 'Cnt_mod/1', 'Cnt_s/1'); L(s, 'Cnt_wrap/1', 'Cnt_s/2');
DTC(s, 'Cnt_i32', 'int32'); L(s, 'Cnt_s/1', 'Cnt_i32/1');
A(s, 'simulink/Signal Routing/Demux', 'Wm_dmx', 'Outputs', '2'); L(s, 'Wm/1', 'Wm_dmx/1', 'w_m');
A(s, 'simulink/Signal Routing/Demux', 'Cnt_dmx', 'Outputs', '2'); L(s, 'Cnt_i32/1', 'Cnt_dmx/1', 'enc_cnt');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt', 'OutputSignals', 'can1_drop_mask'); L(s, 'flt/1', 'Sel_flt/1', 'flt');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt50', 'OutputSignals', 'can1_drop_mask'); L(s, 'flt_50/1', 'Sel_flt50/1', 'flt_50');
A(s, 'simulink/Sources/Counter Free-Running', 'Alive', 'NumBits', '8', 'tsamp', 'Ts.ctrl');
% the side feedback constants: one Constant per element of Bus_DriveSideFb it feeds (Z_s tau_est, Z_iq i_q, Z_idc i_dc;
% T_K T_motor, T_K_esc T_esc), so that with every line named no two inputs of Fb_L / Fb_R carry one name (duplicated
% names); the same values, types and sample time
CST(s, 'Z_u16', '0', 'uint16', 'Ts.ctrl'); CST(s, 'Z_s', '0', 'single', 'Ts.ctrl'); CST(s, 'T_peak', 'P.electronics.drv.T_peak', 'single', 'Ts.ctrl');
CST(s, 'Z_iq', '0', 'single', 'Ts.ctrl'); CST(s, 'Z_idc', '0', 'single', 'Ts.ctrl');
CST(s, 'V_dc', '51.2', 'single', 'Ts.ctrl'); CST(s, 'T_K', '300', 'single', 'Ts.ctrl'); CST(s, 'Rx_ok', 'true', 'boolean', 'Ts.ctrl');
CST(s, 'T_K_esc', '300', 'single', 'Ts.ctrl');
CST(s, 'St_op', '4', 'uint8', 'Ts.ctrl'); CST(s, 'St_rdy', '2', 'uint8', 'Ts.ctrl'); CST(s, 'St_dis', '5', 'uint8', 'Ts.ctrl');
CST(s, 'Status_op', '2^0 + 2^1 + 2^5', 'uint16', 'Ts.ctrl'); CST(s, 'Status_rdy', '2^0 + 2^5', 'uint16', 'Ts.ctrl');
CST(s, 'Brk_bit', '2^4', 'uint16', 'Ts.ctrl'); CST(s, 'Brk_bit0', '0', 'uint16', 'Ts.ctrl');     % FB1 status b4
CST(s, 'Dis_one', '1', 'uint16', 'Ts.ctrl'); CST(s, 'Dis_zero', '0', 'uint16', 'Ts.ctrl');
% the ESC brake as drive esc_sm (D71, 2026-11-15): run = enable AND NOT park_req (one CAN cycle after the command).
% When the run ends the brake command drops at once (FB1 brake_released false) and the ESC stays DISABLING with its
% torque for n_engage = round(P.drive.brake.t_engage_wait/Ts.ctrl) samples, then READY. Bench fault plant(6)
% brake_fail (b0 L, b1 R): that brake never engages (brake_released true, DISABLING)
A(s, 'simulink/Signal Routing/Selector', 'Brk_fail_sel', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '6', 'InputPortWidth', '9');
L(s, 'plant/1', 'Brk_fail_sel/1', 'plant');
DTC(s, 'Brk_fail_u8', 'uint8'); L(s, 'Brk_fail_sel/1', 'Brk_fail_u8/1', 'brake_fail_d');
A(s, 'simulink/Signal Routing/Bus Creator', 'Fast_BC', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_Can1_EscFast', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
A(s, 'simulink/Signal Routing/Bus Creator', 'Slow_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_EscSlow', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for sd = {'L', 1; 'R', 2}'
    x = sd{1}; j = sd{2};
    % ESC state reported one CAN cycle after the command (explicit loop breakers, rule 8)
    A(s, 'simulink/Discrete/Unit Delay', ['En_dly_' x], 'InitialCondition', 'false', 'SampleTime', '-1');
    PICK(s, 'cmd/1', 'cmd', ['Sel_cmd_' x], {[x '.enable']}, {['En_dly_' x '/1']});      % one pick per consumer
    A(s, 'simulink/Discrete/Unit Delay', ['Park_dly_' x], 'InitialCondition', 'true', 'SampleTime', '-1');
    PICK(s, 'cmd/1', 'cmd', ['Sel_park_' x], {[x '.park_req']}, {['Park_dly_' x '/1']});
    LOGIC(s, ['No_park_' x], 'NOT', 1); L(s, ['Park_dly_' x '/1'], ['No_park_' x '/1'], ['park_req_' x]);
    LOGIC(s, ['Run_' x], 'AND', 2); L(s, ['En_dly_' x '/1'], ['Run_' x '/1'], ['enable_' x]); L(s, ['No_park_' x '/1'], ['Run_' x '/2']);
    % samples since the run ended (uint16, saturating; starts saturated: READY at power-up)
    A(s, 'simulink/Discrete/Unit Delay', ['Dis_prev_' x], 'InitialCondition', '1000', 'SampleTime', 'Ts.ctrl');
    A(s, 'simulink/Math Operations/Sum', ['Dis_inc_' x], 'Inputs', '++', 'OutDataTypeStr', 'uint16', ...
        'SaturateOnIntegerOverflow', 'on');
    A(s, 'simulink/Signal Routing/Switch', ['Dis_cnt_' x], 'Criteria', 'u2 ~= 0');
    L(s, ['Dis_prev_' x '/1'], ['Dis_inc_' x '/1'], ['dis_prev_' x]); L(s, 'Dis_one/1', ['Dis_inc_' x '/2']);
    L(s, 'Dis_zero/1', ['Dis_cnt_' x '/1']); L(s, ['Run_' x '/1'], ['Dis_cnt_' x '/2'], ['run_' x]);
    L(s, ['Dis_inc_' x '/1'], ['Dis_cnt_' x '/3']); L(s, ['Dis_cnt_' x '/1'], ['Dis_prev_' x '/1'], ['dis_cnt_' x]);
    A(s, 'simulink/Logic and Bit Operations/Compare To Constant', ['Engaging_' x], 'relop', '<=', ...
        'const', 'round(P.drive.brake.t_engage_wait/Ts.ctrl)', 'OutDataTypeStr', 'boolean');
    L(s, ['Dis_cnt_' x '/1'], ['Engaging_' x '/1']);
    A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', ['Bfail_bit_' x], 'logicop', 'AND', 'UseBitMask', 'on', ...
        'BitMask', num2str(2^(j - 1)), 'BitMaskRealWorld', 'Stored Integer');
    A(s, 'simulink/Logic and Bit Operations/Compare To Zero', ['Bfail_' x], 'relop', '~=', 'OutDataTypeStr', 'boolean');
    L(s, 'Brk_fail_u8/1', ['Bfail_bit_' x '/1'], 'brake_fail'); L(s, ['Bfail_bit_' x '/1'], ['Bfail_' x '/1']);
    LOGIC(s, ['Dis_on_' x], 'OR', 2); L(s, ['Engaging_' x '/1'], ['Dis_on_' x '/1'], ['engaging_' x]);
    L(s, ['Bfail_' x '/1'], ['Dis_on_' x '/2'], ['brake_fail_' x]);
    A(s, 'simulink/Signal Routing/Switch', ['St_stop_' x], 'Criteria', 'u2 ~= 0');
    L(s, 'St_dis/1', ['St_stop_' x '/1']); L(s, ['Dis_on_' x '/1'], ['St_stop_' x '/2']); L(s, 'St_rdy/1', ['St_stop_' x '/3']);
    A(s, 'simulink/Signal Routing/Switch', ['State_' x], 'Criteria', 'u2 ~= 0');
    L(s, 'St_op/1', ['State_' x '/1']); L(s, ['Run_' x '/1'], ['State_' x '/2']); L(s, ['St_stop_' x '/1'], ['State_' x '/3']);
    LOGIC(s, ['Brk_rel_' x], 'OR', 2); L(s, ['Run_' x '/1'], ['Brk_rel_' x '/1']); L(s, ['Bfail_' x '/1'], ['Brk_rel_' x '/2']);
    A(s, 'simulink/Signal Routing/Switch', ['Status_base_' x], 'Criteria', 'u2 ~= 0');
    L(s, 'Status_op/1', ['Status_base_' x '/1']); L(s, ['Run_' x '/1'], ['Status_base_' x '/2']);
    L(s, 'Status_rdy/1', ['Status_base_' x '/3']);
    A(s, 'simulink/Signal Routing/Switch', ['Brk_st_' x], 'Criteria', 'u2 ~= 0');
    L(s, 'Brk_bit/1', ['Brk_st_' x '/1']); L(s, ['Brk_rel_' x '/1'], ['Brk_st_' x '/2'], ['brake_released_' x]);
    L(s, 'Brk_bit0/1', ['Brk_st_' x '/3']);
    A(s, 'simulink/Math Operations/Sum', ['Status_' x], 'Inputs', '++', 'OutDataTypeStr', 'uint16');
    L(s, ['Status_base_' x '/1'], ['Status_' x '/1'], ['status_base_' x]); L(s, ['Brk_st_' x '/1'], ['Status_' x '/2']);
    DTC(s, ['Wm_s_' x], 'single'); L(s, sprintf('Wm_dmx/%d', j), ['Wm_s_' x '/1']);
    BCB(s, ['Fb_' x], 'Bus_DriveSideFb', {'state', ['State_' x '/1']; 'status', ['Status_' x '/1']; 'fault_code', 'Z_u16/1'; ...
        'w_m', ['Wm_s_' x '/1']; 'enc_cnt', sprintf('Cnt_dmx/%d', j); 'tau_est', 'Z_s/1'; 'tau_avail', 'T_peak/1'; ...
        'i_q', 'Z_iq/1'; 'i_dc', 'Z_idc/1'; 'v_dc', 'V_dc/1'; 'T_motor', 'T_K/1'; 'T_esc', 'T_K_esc/1'; ...
        'brake_released', ['Brk_rel_' x '/1']; 'alive', 'Alive/1'; 'rx_ok', 'Rx_ok/1'});
    A(s, 'simulink/Ports & Subsystems/Subsystem Reference', ['Pack_' x], 'ReferencedSubsystem', 'drive_fb_pack');
    CST(s, ['Node_' x], num2str(j), 'uint8', 'Ts.ctrl');
    L(s, ['Fb_' x '/1'], ['Pack_' x '/1'], ['fb_' x]); L(s, ['Node_' x '/1'], ['Pack_' x '/2']);
    DROP(s, ['Drop_fb1_' x], ['Pack_' x '/1'], 'Sel_flt/1', ['P.electronics.can.drop_bit.fb1_' x]);
    DROP(s, ['Drop_fb3_' x], ['Pack_' x '/3'], 'Sel_flt/1', ['P.electronics.can.drop_bit.fb3_' x]);
    RT(s, ['RT_fb2_' x], 'Ts.comm', 'P.ic.Bus_CanFrame8'); L(s, ['Pack_' x '/2'], ['RT_fb2_' x '/1']);
    DROP(s, ['Drop_fb2_' x], ['RT_fb2_' x '/1'], 'Sel_flt50/1', ['P.electronics.can.drop_bit.fb2_' x]);
    L(s, ['Drop_fb1_' x '/1'], sprintf('Fast_BC/%d', j), ['fb1_' x]); L(s, ['Drop_fb3_' x '/1'], sprintf('Fast_BC/%d', j + 2), ['fb3_' x]);
    L(s, ['Drop_fb2_' x '/1'], sprintf('Slow_BC/%d', j), ['fb2_' x]);
end
L(s, 'Fast_BC/1', 'esc_fast/1', 'esc_fast'); L(s, 'Slow_BC/1', 'esc_slow/1', 'esc_slow');
end

function local_bms_tx(mdl)
s = SUB(mdl, 'BMS_Tx', {'bms', 'flt'}, {'frames'});
A(s, 'simulink/Ports & Subsystems/Subsystem Reference', 'Pack', 'ReferencedSubsystem', 'bms_can_pack');
L(s, 'bms/1', 'Pack/1', 'bms');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt', 'OutputSignals', 'can1_drop_mask'); L(s, 'flt/1', 'Sel_flt/1', 'flt');
slots = {'limits', 'soc', 'vit', 'alarm', 'cells', 'state', 'power'};
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel', 'OutputSignals', strjoin(slots, ','));
L(s, 'Pack/1', 'Sel/1', 'frames_tx');
A(s, 'simulink/Signal Routing/Bus Creator', 'BC', 'Inputs', '7', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:7
    DROP(s, ['Drop_' slots{k}], sprintf('Sel/%d', k), 'Sel_flt/1', 'P.electronics.can.drop_bit.bms');
    L(s, ['Drop_' slots{k} '/1'], sprintf('BC/%d', k), slots{k});
end
L(s, 'BC/1', 'frames/1', 'frames');
end

function local_comp_tx(mdl)
s = SUB(mdl, 'COMP_Tx', {'gnc', 'v_LR', 'plant', 'flt'}, {'frame'});
RT(s, 'RT_v', 'Ts.comm', '[0 0]'); L(s, 'v_LR/1', 'RT_v/1', 'v_LR');
A(s, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(s, 'simulink/Math Operations/Gain', 'V_mean', 'Gain', '0.5'); L(s, 'RT_v/1', 'V_sum/1'); L(s, 'V_sum/1', 'V_mean/1');
A(s, 'simulink/Signal Routing/Selector', 'Ratio', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '4', 'InputPortWidth', '9');
L(s, 'plant/1', 'Ratio/1', 'plant');
A(s, 'simulink/Math Operations/Product', 'V_ground', 'Inputs', '**'); L(s, 'V_mean/1', 'V_ground/1', 'v_mean'); L(s, 'Ratio/1', 'V_ground/2', 'ground_ratio');
DTC(s, 'V_ground_s', 'single'); L(s, 'V_ground/1', 'V_ground_s/1');
A(s, 'simulink/Signal Routing/Bus Assignment', 'Assign', 'AssignedSignals', 'v_ground_est');
L(s, 'gnc/1', 'Assign/1', 'gnc'); L(s, 'V_ground_s/1', 'Assign/2', 'v_ground_est');
A(s, 'simulink/Ports & Subsystems/Subsystem Reference', 'Pack', 'ReferencedSubsystem', 'can2_comp_cmd_pack');
L(s, 'Assign/1', 'Pack/1', 'gnc_cmd');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_flt', 'OutputSignals', 'can2_loss,can2_busoff'); L(s, 'flt/1', 'Sel_flt/1', 'flt');
LOGIC(s, 'Lost', 'OR', 2); L(s, 'Sel_flt/1', 'Lost/1'); L(s, 'Sel_flt/2', 'Lost/2');
LOGIC(s, 'Ok', 'NOT', 1); L(s, 'Lost/1', 'Ok/1', 'can2_lost');
A(s, 'simulink/Signal Routing/Bus Selector', 'Sel_valid', 'OutputSignals', 'valid'); L(s, 'Pack/1', 'Sel_valid/1', 'frame_tx');
LOGIC(s, 'Valid', 'AND', 2); L(s, 'Sel_valid/1', 'Valid/1'); L(s, 'Ok/1', 'Valid/2');
A(s, 'simulink/Signal Routing/Bus Assignment', 'Gate', 'AssignedSignals', 'valid');
L(s, 'Pack/1', 'Gate/1'); L(s, 'Valid/1', 'Gate/2', 'valid');
L(s, 'Gate/1', 'frame/1', 'frame');
end

% ===========================================================================================
% helpers
% ===========================================================================================
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

function SELB(sys, src, nm, blk, sigs)
% a Bus Selector blk that selects sigs from the bus line of src ('Blk/k'), which L names nm when it has no name yet;
% the caller wires its outputs (a pick right in front of its consumer, spec 6.2)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
end

function PICK(sys, src, nm, blk, sigs, dsts)
% one consumer's Bus Selector blk right in front of it (spec 6.2): SELB, then its output k feeds dsts{k} ('Blk/j')
SELB(sys, src, nm, blk, sigs);
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

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function p = DTC(sys, name, type)
p = A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', type, ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
end

function p = LOGIC(sys, name, op, nin)
p = A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(nin));
end

function p = RT(sys, name, ts, ic)
p = A(sys, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', ts, 'InitialCondition', ic);
end

function RTL(sys, name, ts, ic, src, dst, nm)
RT(sys, name, ts, ic);
L(sys, src, [name '/1'], nm);
if ~isempty(dst), L(sys, [name '/1'], dst); end
end

function s = SUB(sys, name, ins, outs)
s = [sys '/' name];
add_block('built-in/SubSystem', s);
for k = 1:numel(ins), A(s, 'simulink/Sources/In1', ins{k}, 'Port', num2str(k)); end
for k = 1:numel(outs), A(s, 'simulink/Sinks/Out1', outs{k}, 'Port', num2str(k)); end
end

function DROP(sys, name, frameSrc, maskSrc, bitExpr)
% CAN frame slot: valid := valid AND NOT (drop mask bit)
A(sys, 'simulink/Logic and Bit Operations/Bitwise Operator', [name '_bit'], 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', ['2^' bitExpr], 'BitMaskRealWorld', 'Stored Integer');
A(sys, 'simulink/Logic and Bit Operations/Compare To Zero', [name '_keep'], 'relop', '==', 'OutDataTypeStr', 'boolean');
L(sys, maskSrc, [name '_bit/1']); L(sys, [name '_bit/1'], [name '_keep/1']);
A(sys, 'simulink/Signal Routing/Bus Selector', [name '_sel'], 'OutputSignals', 'valid');
L(sys, frameSrc, [name '_sel/1']);
LOGIC(sys, [name '_and'], 'AND', 2); L(sys, [name '_sel/1'], [name '_and/1']); L(sys, [name '_keep/1'], [name '_and/2']);
A(sys, 'simulink/Signal Routing/Bus Assignment', name, 'AssignedSignals', 'valid');
L(sys, frameSrc, [name '/1']); L(sys, [name '_and/1'], [name '/2']);
end

function p = BCB(sys, name, bus, map)
% nonvirtual Bus Creator with a bus object; map {element, source} in any order, each source at most once (a source
% feeding two inputs would give them one name); a line into it without a name yet takes its element's name (L)
B = evalin('base', bus);
el = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
assert(numel(el) == size(map, 1) && all(ismember(el, map(:, 1))), 'build_tb_mcu_closed_loop:bus', ...
    '%s: map does not cover the elements of %s', name, bus);
assert(numel(unique(map(:, 2))) == size(map, 1), 'build_tb_mcu_closed_loop:bus', ...
    '%s: a source feeds two inputs (duplicated names)', name);
p = A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(el)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(el)
    L(sys, map{strcmp(map(:, 1), el{k}), 2}, sprintf('%s/%d', name, k), el{k});
end
end
