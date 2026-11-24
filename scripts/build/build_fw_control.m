function files = build_fw_control(which)
%BUILD_FW_CONTROL  MCU vehicle-control units of Shaddad_MCU (owner: electronics, task T09).
%
%   build_fw_control()          build every unit below (Subsystem Reference files, final ports of T08)
%   build_fw_control(names)     build only these units (e.g. {'fw_vehicle_ctrl'})
%   files = build_fw_control(...)   project-relative paths written
%
%   subsystems/electronics/
%     fw_safety_supervisor  Task_10   ICD 6 fault word: 32 conditions, vectorised persistence (Sum + Unit Delay +
%                                     Switch, integer ticks), latch (Logical + Unit Delay, reset by reset_req AND NOT
%                                     cond), commutil2 Bit to Integer (32), class flags, k_der = min of derates; the
%                                     descent-cap trip of bit 16 (subsystem Descent_trip), past the persistence
%     fw_mode_manager       Task_10   MATLAB Function -> subsystems/electronics/mcu_mode_manager.m (ShaddadMode FSM;
%                                     soft stops end in a parking phase and a stop monitor, out stop_ctl, D71)
%     fw_vehicle_ctrl       Task_10   arbitration, descent speed cap (subsystem Descent_cap), lateral limit, shaper
%                                     per axis (constant-decel stop law, D33),
%                                     yaw-rate PI (reset at commanded standstill), skid mixer with desaturation,
%                                     ESC setpoints, torque / power limits, traction control -> Bus_DriveCmd
%     fw_implement_aux      Task_50   rotor, nozzles, pump, lift, beacon, buzzer, lights, fans, power enables
%     fw_watchdog_mgr       Task_1k   task ages, IWDG kick, MCU_SAFETY_OK (DO0), SAFETY_RESET_REQ (DO1, D30)
%   Ports: the T08 stubs (build_fw_platform) fixed them; Shaddad_MCU (build_shaddad_mcu) instantiates the units.
%   Requires P, Ts and Bus_* in the base workspace (shaddad_params). Build rule 3 (docs/build_plan.md).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of each file
%   with shaddad_layout(<file>, 'Recursive', true), the subsystems of fw_vehicle_ctrl, the Enabled Bias_avg and the age
%   counters included. Every signal line is named (L5): a line keeps the first name a call gives it, and local_name_rest
%   names every line still without a name after its source block ('_k' for output k of a block with several outputs, as
%   shaddad_bu('nline')); a Bus Selector's output keeps '<element>'. The root ports of every unit stay classic
%   (Shaddad_MCU, mcu_ctrl_harness and t_mcu_control wire and read them by name and number). Inside, one Bus Selector
%   right in front of each consumer (spec 6.2), fed by the bus line of the port fw10 / fw50; one pick feeds a column of
%   like decoders (the DI word bits, the mode compares of fw_implement_aux).

all = {'fw_safety_supervisor', 'fw_mode_manager', 'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
bad = setdiff(which, all);
assert(isempty(bad), 'build_fw_control:unit', 'Unknown unit(s): %s', strjoin(bad, ', '));
assert(evalin('base', 'exist(''P'',''var'') && isfield(P.electronics.safety, ''mask_rc'')'), ...
    'build_fw_control:params', 'P.electronics lacks the T09 parameters: run shaddad_params first');
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
files = {};
for k = 1:numel(which)
    n = which{k};
    f = fullfile(root, 'subsystems', 'electronics', [n '.slx']);
    shaddad_bu('fresh', n, f, 'ssr');
    feval(['local_' n], n);
    local_name_rest(n);
    shaddad_bu('save', n, f);
    close_system(n, 0);
    files{end+1} = fullfile('subsystems', 'electronics', [n '.slx']); %#ok<AGROW>
end
end

% ===========================================================================================
% fw_safety_supervisor (Task_10)
% ===========================================================================================
function local_fw_safety_supervisor(n)
CT = 'Ts.ctrl'; S = 'P.electronics.safety';
IN(n, 'fw10', 1, '', CT); IN(n, 'mode_prev', 2, 'uint8', CT);
% the previous sample of two more unit outputs (Unit Delays in Task_10, as mode_prev; D71, R-01, 2026-11-15): the mode
% manager's stop monitor stop_ctl(2) (bit 22) and the track speed references vc_tel(8:9) of fw_vehicle_ctrl (bit 16)
IN(n, 'stop_fail_prev', 3, 'boolean', CT); IN(n, 'vtrack_ref_prev', 4, 'single', CT, '2');
B = 'fw10/1';           % the bus line of the port fw10: each consumer picks its fields right in front of it (spec 6.2)
% ---- DI bits (ICD 6) and mode sets of the previous sample ----
SELB(n, B, 'Sel_di_word', {'di_word'});                      % one pick for the seven DI word decoders
di = {'estop_ch1_ok', 'estop_ch2_ok', 'safety_ok', 'bumper_front', 'bumper_rear', 'lid_closed'};
for k = 1:numel(di)
    BIT(n, ['DI_' di{k}], 'Sel_di_word/1', ['P.electronics.di.' di{k}], 'uint16');
end
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Impl_id_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^P.electronics.di.impl_id0 + 2^P.electronics.di.impl_id1', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Impl_fitted', 'relop', '~=', 'OutDataTypeStr', 'boolean');
L(n, 'Sel_di_word/1', 'Impl_id_mask/1'); L(n, 'Impl_id_mask/1', 'Impl_fitted/1');
MODESET(n, 'Act', 'mode_prev/1', 'MANUAL', 'CTRL_STOP');       % MANUAL HOLD AUTO_START AUTO CTRL_STOP
MODESET(n, 'Auto', 'mode_prev/1', 'AUTO_START', 'AUTO');
CMP(n, 'Mon', '>=', 'double(ShaddadMode.STANDBY)'); L(n, 'mode_prev/1', 'Mon/1', 'mode_prev');
% ---- conditions c{bit+1}; cn{bit+1} names a line into Cond that no other call names ----
c = cell(1, 32); cn = cell(1, 32);
LOGIC(n, 'C00_hw_estop', 'NAND', 2); L(n, 'DI_estop_ch1_ok/1', 'C00_hw_estop/1'); L(n, 'DI_estop_ch2_ok/1', 'C00_hw_estop/2');
c{1} = 'C00_hw_estop/1';
SELB(n, B, 'Sel_rc', {'rc.estop_remote', 'rc.rc_link_lost', 'rc.failsafe'});      % bits 1-3, straight into Cond 2-4
c{2} = 'Sel_rc/1'; c{3} = 'Sel_rc/2'; c{4} = 'Sel_rc/3';
% bit 4 COMP_TIMEOUT: COMP_CMD seq unchanged (as seen by Task_10) for comp_timeout, AUTO_START / AUTO only
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'Seq_change', 'vinit', '0');
PICK(n, B, 'Sel_seq', {'gnc_cmd.seq'}, {'Seq_change/1'});
AGE(n, 'Seq_age', 'round(P.electronics.can.age_sat_s/Ts.ctrl)');
L(n, 'Seq_change/1', 'Seq_age/1', 'seq_change');
CMP(n, 'Seq_stale', '>=', ['round(' S '.comp_timeout/Ts.ctrl)']); L(n, 'Seq_age/1', 'Seq_stale/1', 'seq_age');
LOGIC(n, 'C04_comp_timeout', 'AND', 2); L(n, 'Seq_stale/1', 'C04_comp_timeout/1'); L(n, 'Auto/1', 'C04_comp_timeout/2');
c{5} = 'C04_comp_timeout/1';
% bits 5/6 DRV_x_TIMEOUT (FB1 or FB3 age > drv_timeout), monitored from STANDBY on
CMP(n, 'Drv_old', '>', [S '.drv_timeout + Ts.ctrl/2']); PICK(n, B, 'Sel_drv_age', {'drv_age'}, {'Drv_old/1'});
LOGIC(n, 'Drv_tmo', 'AND', 2); L(n, 'Drv_old/1', 'Drv_tmo/1'); L(n, 'Mon/1', 'Drv_tmo/2');
A(n, 'simulink/Signal Routing/Demux', 'Drv_tmo_LR', 'Outputs', '2'); L(n, 'Drv_tmo/1', 'Drv_tmo_LR/1');
c{6} = 'Drv_tmo_LR/1'; c{7} = 'Drv_tmo_LR/2'; cn{6} = 'drv_tmo_L'; cn{7} = 'drv_tmo_R';
% bits 7/8 DRV_x_FAULT: ESC state FAULT or fault_code (STO bit 10 excluded) != 0
for sd = {'L', 8; 'R', 9}'
    CMP(n, ['Esc_fault_' sd{1}], '==', [S '.esc_fault_state']);
    PICK(n, B, ['Sel_state_' sd{1}], {['drive_fb.' sd{1} '.state']}, {['Esc_fault_' sd{1} '/1']});
    A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', ['Fc_mask_' sd{1}], 'logicop', 'AND', 'UseBitMask', 'on', ...
        'BitMask', [S '.drv_fault_mask'], 'BitMaskRealWorld', 'Stored Integer');
    A(n, 'simulink/Logic and Bit Operations/Compare To Zero', ['Fc_on_' sd{1}], 'relop', '~=', 'OutDataTypeStr', 'boolean');
    PICK(n, B, ['Sel_fault_code_' sd{1}], {['drive_fb.' sd{1} '.fault_code']}, {['Fc_mask_' sd{1} '/1']});
    L(n, ['Fc_mask_' sd{1} '/1'], ['Fc_on_' sd{1} '/1']);
    LOGIC(n, sprintf('C%02d_drv_fault_%s', sd{2} - 1, sd{1}), 'OR', 2);
    L(n, ['Esc_fault_' sd{1} '/1'], sprintf('C%02d_drv_fault_%s/1', sd{2} - 1, sd{1}));
    L(n, ['Fc_on_' sd{1} '/1'], sprintf('C%02d_drv_fault_%s/2', sd{2} - 1, sd{1}));
    c{sd{2}} = sprintf('C%02d_drv_fault_%s/1', sd{2} - 1, sd{1});
end
% bit 9 BMS_TIMEOUT (monitored from STANDBY on), bit 10 BMS_ALARM, bit 11 BMS_WARN (fresh data only)
CMP(n, 'Bms_stale', '>', [S '.bms_timeout + Ts.hk/2']); PICK(n, B, 'Sel_bms_age', {'bms_age'}, {'Bms_stale/1'});
LOGIC(n, 'C09_bms_timeout', 'AND', 2); L(n, 'Bms_stale/1', 'C09_bms_timeout/1'); L(n, 'Mon/1', 'C09_bms_timeout/2');
c{10} = 'C09_bms_timeout/1';
CMP(n, 'C10_bms_alarm', '==', '3'); PICK(n, B, 'Sel_fault_level', {'bms.fault_level'}, {'C10_bms_alarm/1'});
c{11} = 'C10_bms_alarm/1';
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Bms_warnword', 'relop', '~=', 'OutDataTypeStr', 'boolean');
PICK(n, B, 'Sel_warn_word', {'bms.warn_word'}, {'Bms_warnword/1'});
CMP(n, 'Bms_dcl_low', '<', [S '.dcl_warn']); PICK(n, B, 'Sel_DCL', {'bms.DCL'}, {'Bms_dcl_low/1'});
LOGIC(n, 'Bms_warn_any', 'OR', 2); L(n, 'Bms_warnword/1', 'Bms_warn_any/1'); L(n, 'Bms_dcl_low/1', 'Bms_warn_any/2');
LOGIC(n, 'Bms_fresh', 'NOT', 1); L(n, 'Bms_stale/1', 'Bms_fresh/1');
LOGIC(n, 'C11_bms_warn', 'AND', 3); L(n, 'Bms_warn_any/1', 'C11_bms_warn/1'); L(n, 'Bms_fresh/1', 'C11_bms_warn/2');
PICK(n, B, 'Sel_bms_valid', {'bms.valid'}, {'C11_bms_warn/3'});
c{12} = 'C11_bms_warn/1';
% bits 12/13 ENC_x_MISMATCH: abs(v_enc - w_m*r_s/N) > enc_mismatch_v (both feedbacks fresh)
A(n, 'simulink/Math Operations/Gain', 'Wm_to_v', 'Gain', '1/P.electronics.drv.k_v2w');
A(n, 'simulink/Math Operations/Sum', 'Enc_diff', 'Inputs', '+-');
A(n, 'simulink/Math Operations/Abs', 'Enc_abs');
CMP(n, 'Enc_bad', '>', 'P.electronics.drv.enc_mismatch_v');
LOGIC(n, 'Enc_mis', 'AND', 2);
PICK(n, B, 'Sel_wMotor_LR', {'odom.wMotor_LR'}, {'Wm_to_v/1'}); PICK(n, B, 'Sel_vTrack_enc', {'odom.vTrack_LR'}, {'Enc_diff/1'});
L(n, 'Wm_to_v/1', 'Enc_diff/2'); L(n, 'Enc_diff/1', 'Enc_abs/1'); L(n, 'Enc_abs/1', 'Enc_bad/1');
L(n, 'Enc_bad/1', 'Enc_mis/1'); PICK(n, B, 'Sel_odom_valid_enc', {'odom.valid'}, {'Enc_mis/2'});
A(n, 'simulink/Signal Routing/Demux', 'Enc_mis_LR', 'Outputs', '2'); L(n, 'Enc_mis/1', 'Enc_mis_LR/1');
c{13} = 'Enc_mis_LR/1'; c{14} = 'Enc_mis_LR/2'; cn{13} = 'enc_mis_L'; cn{14} = 'enc_mis_R';
% bits 14/15 TILT_WARN / TILT_STOP: max(|roll|, |pitch|) > tilt_warn / tilt_stop while IMU_FAULT (bit 24) is clear (its
% value of the previous sample, Imu_ok_prev from Q_prev below: the current one would close an algebraic loop through
% the persistence); an invalid IMU sample holds the persistence timer as tilt_gate does (Gate_closed below). 2026-11-15,
% R-05 (review MCU-06): before, the conditions were ANDed with the per-sample imu_valid, so one invalid sample
% restarted the 0.5 s timer and a steady TILT_STOP could never persist on rough ground (D47)
A(n, 'simulink/Math Operations/Abs', 'Roll_abs'); A(n, 'simulink/Math Operations/Abs', 'Pitch_abs');
A(n, 'simulink/Math Operations/MinMax', 'Tilt_max', 'Function', 'max', 'Inputs', '2');
PICK(n, B, 'Sel_roll', {'tilt.roll'}, {'Roll_abs/1'}); PICK(n, B, 'Sel_pitch', {'tilt.pitch'}, {'Pitch_abs/1'});
L(n, 'Roll_abs/1', 'Tilt_max/1'); L(n, 'Pitch_abs/1', 'Tilt_max/2');
CMP(n, 'Tilt_gt_warn', '>', [S '.tilt_warn']); CMP(n, 'Tilt_gt_stop', '>', [S '.tilt_stop']);
L(n, 'Tilt_max/1', 'Tilt_gt_warn/1', 'tilt'); L(n, 'Tilt_max/1', 'Tilt_gt_stop/1');
LOGIC(n, 'Imu_ok_prev', 'NOT', 1);                          % NOT IMU_FAULT (bit 24) of the previous sample
LOGIC(n, 'C14_tilt_warn', 'AND', 2); L(n, 'Tilt_gt_warn/1', 'C14_tilt_warn/1');
L(n, 'Imu_ok_prev/1', 'C14_tilt_warn/2', 'imu_ok_prev');
LOGIC(n, 'C15_tilt_stop', 'AND', 2); L(n, 'Tilt_gt_stop/1', 'C15_tilt_stop/1'); L(n, 'Imu_ok_prev/1', 'C15_tilt_stop/2');
c{15} = 'C14_tilt_warn/1'; c{16} = 'C15_tilt_stop/1';
% bit 16 OVERSPEED: (i) max |v_track| > v_overspeed; (ii) while driving (mode_prev MANUAL, HOLD, AUTO_START or AUTO) a
% track runs lim.v_follow_dv above its follow reference (runaway, R-01 / DRV-01, 2026-11-15): rf = max(|vtrack_ref_prev|,
% min(rf of the previous sample, |vTrack|)) per track, the reference of the previous sample (input vtrack_ref_prev =
% fw_vehicle_ctrl vc_tel(8:9) through a Unit Delay in Task_10); while the speed lags a falling reference (a regen-limited
% stop, CCL 0) rf follows the speed down, so only a speed that rises again counts (descent task, 2026-11-15: rf =
% |vtrack_ref_prev| before, which tripped on ordinary stops with CCL 0, review finding 1); (iii) the descent-cap trip
% (subsystem Descent_trip below), which also bypasses the persistence (Imm into Active, so a CTRL_STOP parks at once at
% the cap); (i) and (ii) persist t_persist (0.2 s); in CTRL_STOP, ESTOP and the parking the stop monitor (a) (bit 22)
% covers a speed that does not follow
A(n, 'simulink/Math Operations/Abs', 'Vt_abs');
A(n, 'simulink/Math Operations/MinMax', 'Vt_max', 'Function', 'max', 'Inputs', '1');
CMP(n, 'Vt_over', '>', 'P.electronics.lim.v_overspeed');
PICK(n, B, 'Sel_vTrack_over', {'odom.vTrack_LR'}, {'Vt_abs/1'}); L(n, 'Vt_abs/1', 'Vt_max/1'); L(n, 'Vt_max/1', 'Vt_over/1');
A(n, 'simulink/Math Operations/Abs', 'Vref_abs'); L(n, 'vtrack_ref_prev/1', 'Vref_abs/1', 'vtrack_ref_prev');
A(n, 'simulink/Discrete/Unit Delay', 'Rf_prev', 'InitialCondition', '[0 0]', 'SampleTime', '-1');
A(n, 'simulink/Math Operations/MinMax', 'Rf_low', 'Function', 'min', 'Inputs', '2');
A(n, 'simulink/Math Operations/MinMax', 'Rf', 'Function', 'max', 'Inputs', '2');
L(n, 'Rf_prev/1', 'Rf_low/1', 'rf_prev'); L(n, 'Vt_abs/1', 'Rf_low/2');
L(n, 'Vref_abs/1', 'Rf/1', 'vtrack_ref_abs'); L(n, 'Rf_low/1', 'Rf/2', 'rf_low'); L(n, 'Rf/1', 'Rf_prev/1', 'rf');
A(n, 'simulink/Math Operations/Sum', 'Follow_err', 'Inputs', '+-');
L(n, 'Vt_abs/1', 'Follow_err/1'); L(n, 'Rf/1', 'Follow_err/2');
A(n, 'simulink/Math Operations/MinMax', 'Follow_max', 'Function', 'max', 'Inputs', '1');
L(n, 'Follow_err/1', 'Follow_max/1', 'follow_err_LR');
CMP(n, 'Follow_over', '>', 'P.electronics.lim.v_follow_dv'); L(n, 'Follow_max/1', 'Follow_over/1', 'follow_err');
MODESET(n, 'Drv_on', 'mode_prev/1', 'MANUAL', 'AUTO');      % MANUAL HOLD AUTO_START AUTO (the driving modes)
LOGIC(n, 'Follow_on', 'AND', 2); L(n, 'Follow_over/1', 'Follow_on/1'); L(n, 'Drv_on/1', 'Follow_on/2', 'drv_on');
% (iii) descent-cap trip (Agents task descent, the user's decision of 2026-11-15): subsystem Descent_trip, the cap of
% local_descent_cap (trip tables) and local_descent_trip (drive on = mode_prev MANUAL..ESTOP, odometry valid, the
% predicted body speed past the cap of the direction of travel while the reference is steady or rising, or v_follow_dv
% above a falling one; at a hill start, while a brake is engaged and for descent.t_release after both released with the
% reference in the direction of travel at standstill or against it, the measured speed without the prediction)
local_descent_trip(n);
PICK(n, B, 'Sel_descent', {'tilt.pitch', 'tilt.imu_valid', 'di_word', 'impl.impl_type', 'impl.rx_ok', 'adc.lift_pos', ...
    'odom.vTrack_LR', 'a_track', 'odom.valid'}, arrayfun(@(k) sprintf('Descent_trip/%d', k), 1:9, 'UniformOutput', false));
PICK(n, B, 'Sel_brake_rel', {'drive_fb.L.brake_released', 'drive_fb.R.brake_released'}, ...
    {'Descent_trip/12', 'Descent_trip/13'});
L(n, 'mode_prev/1', 'Descent_trip/10'); L(n, 'vtrack_ref_prev/1', 'Descent_trip/11');
LOGIC(n, 'Over_any', 'OR', 3); L(n, 'Vt_over/1', 'Over_any/1'); L(n, 'Follow_on/1', 'Over_any/2', 'follow_on');
L(n, 'Descent_trip/1', 'Over_any/3', 'cap_trip');
LOGIC(n, 'C16_overspeed', 'AND', 2);
L(n, 'Over_any/1', 'C16_overspeed/1'); PICK(n, B, 'Sel_odom_valid_over', {'odom.valid'}, {'C16_overspeed/2'});
c{17} = 'C16_overspeed/1';
% bit 17 HV_LOST (HV_Ready false while ACTIVE), bit 18 HV_OPENING
LOGIC(n, 'Hv_not_ready', 'NOT', 1); PICK(n, B, 'Sel_hv_ready', {'bms.hv_ready'}, {'Hv_not_ready/1'});
LOGIC(n, 'C17_hv_lost', 'AND', 2); L(n, 'Hv_not_ready/1', 'C17_hv_lost/1'); L(n, 'Act/1', 'C17_hv_lost/2');
SELB(n, B, 'Sel_hv_opening', {'bms.hv_opening_warn'});                           % straight into Cond 19
c{18} = 'C17_hv_lost/1'; c{19} = 'Sel_hv_opening/1';
% bit 19 RAIL_UV, bits 20/21 enclosure temperature
CMP(n, 'V24_low', '<', [S '.V24_min']); CMP(n, 'V12_low', '<', [S '.V12_min']); CMP(n, 'V5_low', '<', [S '.V5_min']);
PICK(n, B, 'Sel_V24', {'adc.V24'}, {'V24_low/1'}); PICK(n, B, 'Sel_V12', {'adc.V12'}, {'V12_low/1'});
PICK(n, B, 'Sel_V5', {'adc.V5'}, {'V5_low/1'});
LOGIC(n, 'C19_rail_uv', 'OR', 3); L(n, 'V24_low/1', 'C19_rail_uv/1'); L(n, 'V12_low/1', 'C19_rail_uv/2'); L(n, 'V5_low/1', 'C19_rail_uv/3');
c{20} = 'C19_rail_uv/1';
CMP(n, 'C20_encl_warn', '>', [S '.T_encl_warn']); CMP(n, 'C21_encl_crit', '>', [S '.T_encl_crit']);
PICK(n, B, 'Sel_T_encl_warn', {'adc.T_encl'}, {'C20_encl_warn/1'}); PICK(n, B, 'Sel_T_encl_crit', {'adc.T_encl'}, {'C21_encl_crit/1'});
c{21} = 'C20_encl_warn/1'; c{22} = 'C21_encl_crit/1';
% bit 22 STOP_FAIL (R-01 / D71, 2026-11-15; WDG_RESET_SEEN before, which stays in MCU_STATUS flags b7): the mode
% manager's stop monitor stop_ctl(2) of the previous sample (input stop_fail_prev, Unit Delay in Task_10), class A,
% latched, in maskDO0 (DO0 low = STO backup); bit 23 TASK_OVERRUN (overrun_n consecutive samples), bit 24 IMU_FAULT
c{23} = 'stop_fail_prev/1'; cn{23} = 'stop_fail_prev';                          % straight into Cond 23
A(n, 'simulink/Discrete/Tapped Delay', 'Ovr_taps', 'NumDelays', [S '.overrun_n - 1'], 'includeCurrent', 'on', ...
    'vinit', '0', 'samptime', '-1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'C23_overrun', 'Operator', 'AND', 'Inputs', '1');
PICK(n, B, 'Sel_overrun', {'overrun'}, {'Ovr_taps/1'}); L(n, 'Ovr_taps/1', 'C23_overrun/1', 'overrun_taps');
c{24} = 'C23_overrun/1';
LOGIC(n, 'C24_imu_fault', 'NOT', 1); PICK(n, B, 'Sel_imu_valid_fault', {'tilt.imu_valid'}, {'C24_imu_fault/1'});
c{25} = 'C24_imu_fault/1';
% bits 25/26 CAN bus-off
A(n, 'simulink/Signal Routing/Demux', 'Busoff', 'Outputs', '2'); PICK(n, B, 'Sel_busoff', {'busoff'}, {'Busoff/1'});
c{26} = 'Busoff/1'; c{27} = 'Busoff/2'; cn{26} = 'can1_busoff'; cn{27} = 'can2_busoff';
% bit 27 SAFETY_CHAIN_DISCREPANCY, bit 28 IMPL_FAULT, bit 29 BUMPER, bit 30 LID_OPEN, bit 31 SAFETY_RELAY_OPEN
LOGIC(n, 'C27_discrepancy', 'XOR', 2); L(n, 'DI_estop_ch1_ok/1', 'C27_discrepancy/1'); L(n, 'DI_estop_ch2_ok/1', 'C27_discrepancy/2');
c{28} = 'C27_discrepancy/1';
LOGIC(n, 'Impl_no_rx', 'NOT', 1); PICK(n, B, 'Sel_rx_ok', {'impl.rx_ok'}, {'Impl_no_rx/1'});
LOGIC(n, 'Impl_bad', 'OR', 2); PICK(n, B, 'Sel_impl_fault', {'impl.fault'}, {'Impl_bad/1'}); L(n, 'Impl_no_rx/1', 'Impl_bad/2');
LOGIC(n, 'C28_impl_fault', 'AND', 3); L(n, 'Impl_bad/1', 'C28_impl_fault/1'); L(n, 'Impl_fitted/1', 'C28_impl_fault/2');
L(n, 'Mon/1', 'C28_impl_fault/3');
c{29} = 'C28_impl_fault/1';
LOGIC(n, 'C29_bumper', 'OR', 2); L(n, 'DI_bumper_front/1', 'C29_bumper/1'); L(n, 'DI_bumper_rear/1', 'C29_bumper/2');
c{30} = 'C29_bumper/1';
LOGIC(n, 'C30_lid_open', 'NOT', 1); L(n, 'DI_lid_closed/1', 'C30_lid_open/1');
c{31} = 'C30_lid_open/1';
LOGIC(n, 'Relay_open', 'NOT', 1); L(n, 'DI_safety_ok/1', 'Relay_open/1');
LOGIC(n, 'C31_relay_open', 'AND', 2); L(n, 'Relay_open/1', 'C31_relay_open/1'); L(n, 'Act/1', 'C31_relay_open/2');
c{32} = 'C31_relay_open/1';
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Cond', 'NumInputs', '32');
for k = 1:32, L(n, c{k}, sprintf('Cond/%d', k), cn{k}); end
% ---- vectorised persistence (integer ticks): cnt = cond ? sat(cnt_prev + inc) : 0; persisted = cnt >= N + 1 ----
% inc = 0 for the tilt bits 14/15 while tilt_gate is false (D47) or the IMU sample is invalid (R-05): timer held, else 1
CST(n, 'Gate_bits', 'bitget(sum(2.^[14 15]), 1:32) ~= 0', 'boolean', CT);
LOGIC(n, 'Gate_closed', 'NAND', 2);
PICK(n, B, 'Sel_tilt_gate', {'tilt.tilt_gate', 'tilt.imu_valid'}, {'Gate_closed/1', 'Gate_closed/2'});
LOGIC(n, 'Hold_tick', 'AND', 2); L(n, 'Gate_bits/1', 'Hold_tick/1'); L(n, 'Gate_closed/1', 'Hold_tick/2');
LOGIC(n, 'Count_tick', 'NOT', 1); L(n, 'Hold_tick/1', 'Count_tick/1');
DTC(n, 'Inc', 'uint16'); L(n, 'Count_tick/1', 'Inc/1', 'inc');
A(n, 'simulink/Discrete/Unit Delay', 'Cnt_prev', 'InitialCondition', 'zeros(1,32)', 'SampleTime', '-1');
A(n, 'simulink/Math Operations/Sum', 'Cnt_inc', 'Inputs', '++', 'OutDataTypeStr', 'uint16', 'SaturateOnIntegerOverflow', 'on');
CST(n, 'Cnt_zero', 'zeros(1,32)', 'uint16', CT);
A(n, 'simulink/Signal Routing/Switch', 'Cnt', 'Criteria', 'u2 ~= 0');
L(n, 'Cnt_prev/1', 'Cnt_inc/1', 'cnt_prev'); L(n, 'Inc/1', 'Cnt_inc/2');
L(n, 'Cnt_inc/1', 'Cnt/1'); L(n, 'Cond/1', 'Cnt/2', 'cond'); L(n, 'Cnt_zero/1', 'Cnt/3');
L(n, 'Cnt/1', 'Cnt_prev/1', 'cnt');
CST(n, 'N_persist', ['round(' S '.t_persist/Ts.ctrl) + 1'], 'uint16', CT);
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'Persisted_t', 'Operator', '>=');
L(n, 'Cnt/1', 'Persisted_t/1'); L(n, 'N_persist/1', 'Persisted_t/2');
LOGIC(n, 'Persisted', 'AND', 2); L(n, 'Persisted_t/1', 'Persisted/1'); L(n, 'Cond/1', 'Persisted/2');
% ---- latch: q = persisted OR (latch_mask AND q_prev AND NOT (reset_req AND NOT cond)) ----
CST(n, 'Latch_mask', ['bitget(double(' S '.latch_mask), 1:32) ~= 0'], 'boolean', CT);
LOGIC(n, 'Cond_not', 'NOT', 1); L(n, 'Cond/1', 'Cond_not/1');
LOGIC(n, 'Clear', 'AND', 2); PICK(n, B, 'Sel_reset_req', {'reset_req'}, {'Clear/1'}); L(n, 'Cond_not/1', 'Clear/2');
LOGIC(n, 'Keep', 'NOT', 1); L(n, 'Clear/1', 'Keep/1');
A(n, 'simulink/Discrete/Unit Delay', 'Q_prev', 'InitialCondition', 'false(1,32)', 'SampleTime', '-1');
LOGIC(n, 'Held', 'AND', 3); L(n, 'Latch_mask/1', 'Held/1'); L(n, 'Q_prev/1', 'Held/2', 'q_prev'); L(n, 'Keep/1', 'Held/3');
% the descent-cap trip sets bit 16 in its own sample, past the persistence (descent task): Imm = [0 x 16, cap_trip, 0 x 15]
CST(n, 'Imm_lo', 'false(1,16)', 'boolean', CT); CST(n, 'Imm_hi', 'false(1,15)', 'boolean', CT);
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Imm', 'NumInputs', '3');
L(n, 'Imm_lo/1', 'Imm/1'); L(n, 'Descent_trip/1', 'Imm/2'); L(n, 'Imm_hi/1', 'Imm/3');
LOGIC(n, 'Active', 'OR', 3); L(n, 'Persisted/1', 'Active/1', 'persisted'); L(n, 'Held/1', 'Active/2');
L(n, 'Imm/1', 'Active/3', 'imm');
L(n, 'Active/1', 'Q_prev/1', 'active');
% IMU_FAULT (bit 24) of the previous sample: gates the tilt conditions 14/15 (above)
A(n, 'simulink/Signal Routing/Selector', 'Imu_fault_prev', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1 + 24', 'InputPortWidth', '32');
L(n, 'Q_prev/1', 'Imu_fault_prev/1'); L(n, 'Imu_fault_prev/1', 'Imu_ok_prev/1', 'imu_fault_prev');
% ---- words, classes, derate ----
A(n, 'commutil2/Bit to Integer Converter', 'Fault_pack', 'nbits', '32', 'bitOrder', 'LSB first', 'outDtype', 'uint32');
A(n, 'commutil2/Bit to Integer Converter', 'Warn_pack', 'nbits', '32', 'bitOrder', 'LSB first', 'outDtype', 'uint32');
L(n, 'Active/1', 'Fault_pack/1'); L(n, 'Cond/1', 'Warn_pack/1');
cls = {'A', 'maskA'; 'B', 'maskB'; 'C', 'maskC'};
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Cls', 'NumInputs', '3');
for k = 1:3
    A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', ['Cls_' cls{k, 1} '_mask'], 'logicop', 'AND', ...
        'UseBitMask', 'on', 'BitMask', [S '.' cls{k, 2}], 'BitMaskRealWorld', 'Stored Integer');
    A(n, 'simulink/Logic and Bit Operations/Compare To Zero', ['Cls_' cls{k, 1}], 'relop', '~=', 'OutDataTypeStr', 'boolean');
    L(n, 'Fault_pack/1', ['Cls_' cls{k, 1} '_mask/1'], 'fault_word'); L(n, ['Cls_' cls{k, 1} '_mask/1'], ['Cls_' cls{k, 1} '/1']);
    L(n, ['Cls_' cls{k, 1} '/1'], sprintf('Cls/%d', k));
end
CST(n, 'Derate_vec', [S '.derate_vec'], 'single', CT); CST(n, 'Derate_one', 'ones(1,32)', 'single', CT);
A(n, 'simulink/Signal Routing/Switch', 'Derate_sel', 'Criteria', 'u2 ~= 0');
L(n, 'Derate_vec/1', 'Derate_sel/1'); L(n, 'Active/1', 'Derate_sel/2'); L(n, 'Derate_one/1', 'Derate_sel/3');
A(n, 'simulink/Math Operations/MinMax', 'Derate_bits', 'Function', 'min', 'Inputs', '1');
L(n, 'Derate_sel/1', 'Derate_bits/1');
A(n, 'simulink/Lookup Tables/1-D Lookup Table', 'Derate_therm', 'BreakpointsForDimension1', 'P.electronics.aux.derate_bp', ...
    'Table', 'P.electronics.aux.derate_k', 'ExtrapMethod', 'Clip', 'OutDataTypeStr', 'single', ...
    'TableDataTypeStr', 'single', 'BreakpointsForDimension1DataTypeStr', 'single');
PICK(n, B, 'Sel_T_encl_derate', {'adc.T_encl'}, {'Derate_therm/1'});
A(n, 'simulink/Math Operations/MinMax', 'K_der_min', 'Function', 'min', 'Inputs', '2');
L(n, 'Derate_bits/1', 'K_der_min/1'); L(n, 'Derate_therm/1', 'K_der_min/2');
OUT(n, 'fault_word', 1, 'uint32'); OUT(n, 'warn_word', 2, 'uint32'); OUT(n, 'fault_cls', 3, 'boolean'); OUT(n, 'k_der', 4, 'single');
L(n, 'Fault_pack/1', 'fault_word/1'); L(n, 'Warn_pack/1', 'warn_word/1', 'warn_word');
L(n, 'Cls/1', 'fault_cls/1', 'fault_cls'); L(n, 'K_der_min/1', 'k_der/1', 'k_der');
set_param(n, 'Description', ['fw_safety_supervisor (T09, Task_10, docs/icd.md 6 fault word, spec 2.7.10, D30/D34/D47). ' ...
    'in 1 fw10 (Task_10 platform virtual bus, Shaddad_MCU Description), 2 mode_prev uint8 (mode of the previous ' ...
    'Task_10 sample), 3 stop_fail_prev boolean (fw_mode_manager stop_ctl(2) of the previous sample, D71), 4 ' ...
    'vtrack_ref_prev single[2] (fw_vehicle_ctrl vc_tel(8:9) of the previous sample). out 1 fault_word uint32 (active bits: persisted, latched bits held until reset_req AND NOT cond), ' ...
    '2 warn_word uint32 (instantaneous conditions before persistence / latch), 3 fault_cls boolean[3] [A B C] ' ...
    '(fault_word AND maskA/B/C ~= 0), 4 k_der single = min(derate_vec over the active bits, thermal 1-D Lookup ' ...
    'aux.derate_bp -> derate_k on T_encl). Conditions (bit: detection): 0 DI ESTOP_CH1_OK or CH2_OK low; 1 ' ...
    'rc.estop_remote (valid frames only, D34); 2 rc_link_lost; 3 rc failsafe; 4 COMP_CMD seq unchanged in Task_10 ' ...
    'for comp_timeout while AUTO_START / AUTO; 5/6 drv_age > drv_timeout; 7/8 ESC state FAULT or fault_code AND ' ...
    'drv_fault_mask ~= 0; 9 bms_age > bms_timeout; 10 BMS fault_level 3; 11 BMS warn_word ~= 0 or DCL < dcl_warn ' ...
    '(fresh BMS data); 12/13 |vTrack - wMotor*r_s/N| > enc_mismatch_v; 14/15 max(|roll|,|pitch|) > tilt_warn / ' ...
    'tilt_stop while bit 24 IMU_FAULT of the previous sample is clear, persistence timer held while tilt_gate is ' ...
    'false or imu_valid is false (D47, R-05); 16 max|vTrack| > v_overspeed OR (mode_prev MANUAL..AUTO AND ' ...
    'max(|vTrack| - rf) > lim.v_follow_dv, rf = max(|vtrack_ref_prev|, min(rf of the previous sample, |vTrack|)) ' ...
    'per track: a track runs away from its reference while driving, a regen-limited deceleration does not count, ' ...
    'R-01 / descent) OR the descent-cap trip (subsystem Descent_trip: mode_prev MANUAL..ESTOP, odom valid, |v| + ' ...
    'max(sign(v)*a_track, 0)*t_pred of the mean track speed v past the trip speed of the direction of travel ' ...
    '(descent.trip_fwd / trip_rev by implement row, the mower deck reported raised by adc.lift_pos, and downhill ' ...
    'pitch +/-tilt.pitch, peak-held t_hold, theta_imu ' ...
    'while tilt.imu_valid is false) while the mean reference is steady or rising, or v_follow_dv above a falling ' ...
    'one (falling: more than a_fall*Ts below the sample before, or more than dv_fall above the cap''s speed target ' ...
    'ref_fwd / ref_rev); without the prediction term while drive_fb brake_released is false on a side or true on ' ...
    'both for less than descent.t_release, and the mean reference in the direction of travel is at most ' ...
    'yaw.cmd_still_v (a hill start: the roll-back while the speed loop takes over); set in its own sample past the ' ...
    'persistence, Imm into Active); ' ...
    '17 NOT hv_ready while ACTIVE; 18 hv_opening_warn; 19 V24/V12/V5 below minimum; 20/21 T_encl > warn / crit; 22 ' ...
    'STOP_FAIL = stop_fail_prev (the mode manager stop monitor, D71; class A, latched, in maskDO0); 23 overrun in ' ...
    'overrun_n consecutive samples (Tapped Delay + AND); 24 NOT imu_valid; 25/26 ' ...
    'busoff; 27 ESTOP_CH1_OK XOR CH2_OK; 28 implement fitted (DI IMPL_ID ~= 0) and (IMPL_FB fault or NOT rx_ok); 29 ' ...
    'bumper; 30 NOT LID_CLOSED; 31 NOT SAFETY_OK while ACTIVE (MANUAL..CTRL_STOP). Bits 5, 6, 9 and 28 are monitored ' ...
    'from STANDBY on. Persistence: uint16 tick counter per bit (Sum + Unit Delay + Switch), bit set when ticks >= ' ...
    'round(t_persist/Ts.ctrl) + 1 (the descent-cap trip of bit 16 bypasses it). Parameters P.electronics.safety, ' ...
    'P.electronics.lim. Built by scripts/build/build_fw_control.m.']);
end

% ===========================================================================================
% fw_mode_manager (Task_10)
% ===========================================================================================
function local_fw_mode_manager(n)
CT = 'Ts.ctrl'; S = 'P.electronics.safety';
IN(n, 'fw10', 1, '', CT); IN(n, 'fault_word', 2, 'uint32', CT); IN(n, 'fault_cls', 3, 'boolean', CT, '3');
B = 'fw10/1';           % the bus line of the port fw10: each consumer picks its fields right in front of it (spec 6.2)
SELB(n, B, 'Sel_di_word', {'di_word'});                      % one pick for the four DI word decoders
di = {'key_on', 'safety_ok', 'lid_closed', 'dock_present'};
for k = 1:numel(di)
    BIT(n, ['DI_' di{k}], 'Sel_di_word/1', ['P.electronics.di.' di{k}], 'uint16');
end
% standstill: BOTH tracks under standstill_v (max |vTrack_L,R|, as fw_imu_tilt and Yaw_meas), so a pivot turn (mean
% track speed 0) is no standstill (2026-11-15, R-34, review MCU-05: the mean ended ESTOP.DECEL on its first sample)
A(n, 'simulink/Math Operations/Abs', 'V_abs'); PICK(n, B, 'Sel_vTrack_LR', {'odom.vTrack_LR'}, {'V_abs/1'});
A(n, 'simulink/Math Operations/MinMax', 'V_max', 'Function', 'max', 'Inputs', '1'); L(n, 'V_abs/1', 'V_max/1', 'v_track_abs');
CMP(n, 'Standstill', '<', [S '.standstill_v']); L(n, 'V_max/1', 'Standstill/1', 'v_track_max');
% self-test: rails in range, ESC and BMS frames fresh, IMU valid (an open safety relay does not fail it, D30)
A(n, 'simulink/Math Operations/MinMax', 'Drv_age_max', 'Function', 'max', 'Inputs', '1');
PICK(n, B, 'Sel_drv_age', {'drv_age'}, {'Drv_age_max/1'});
CMP(n, 'Drv_fresh', '<=', [S '.drv_timeout + Ts.ctrl/2']); L(n, 'Drv_age_max/1', 'Drv_fresh/1');
CMP(n, 'Bms_fresh', '<=', [S '.bms_timeout + Ts.hk/2']); PICK(n, B, 'Sel_bms_age', {'bms_age'}, {'Bms_fresh/1'});
CMP(n, 'V24_ok', '>=', [S '.V24_min']); CMP(n, 'V12_ok', '>=', [S '.V12_min']); CMP(n, 'V5_ok', '>=', [S '.V5_min']);
PICK(n, B, 'Sel_V24', {'adc.V24'}, {'V24_ok/1'}); PICK(n, B, 'Sel_V12', {'adc.V12'}, {'V12_ok/1'});
PICK(n, B, 'Sel_V5', {'adc.V5'}, {'V5_ok/1'});
LOGIC(n, 'Selftest_ok', 'AND', 6);
L(n, 'Drv_fresh/1', 'Selftest_ok/1'); L(n, 'Bms_fresh/1', 'Selftest_ok/2');
PICK(n, B, 'Sel_imu_valid', {'tilt.imu_valid'}, {'Selftest_ok/3'});
L(n, 'V24_ok/1', 'Selftest_ok/4'); L(n, 'V12_ok/1', 'Selftest_ok/5'); L(n, 'V5_ok/1', 'Selftest_ok/6');
% brakes_ok (R-01 / D71, 2026-11-15): both ESCs report the brake engaged with the torque off, on fresh frames: FB1
% brake_released false AND ESC state READY / NOT_READY / FAULT / STO (esc_sm drops brake_released with the brake
% command and keeps the torque on n_engage longer in DISABLING, so a state in ENABLING..QUICK_STOP is no confirmation)
% AND rx_ok, both sides; Drv_fresh (drv_age <= drv_timeout, both sides) as for the self-test
SELB(n, B, 'Sel_fb_brake', {'drive_fb.L.brake_released', 'drive_fb.R.brake_released'});
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Brk_LR', 'NumInputs', '2');
L(n, 'Sel_fb_brake/1', 'Brk_LR/1'); L(n, 'Sel_fb_brake/2', 'Brk_LR/2');
LOGIC(n, 'Brk_engaged', 'NOT', 1); L(n, 'Brk_LR/1', 'Brk_engaged/1', 'brake_released_LR');
SELB(n, B, 'Sel_fb_state', {'drive_fb.L.state', 'drive_fb.R.state'});
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Esc_st_LR', 'NumInputs', '2');
L(n, 'Sel_fb_state/1', 'Esc_st_LR/1'); L(n, 'Sel_fb_state/2', 'Esc_st_LR/2');
CMP(n, 'Esc_off_lo', '<=', 'double(EscState.READY)'); CMP(n, 'Esc_off_hi', '>=', 'double(EscState.FAULT)');
L(n, 'Esc_st_LR/1', 'Esc_off_lo/1', 'esc_state_LR'); L(n, 'Esc_st_LR/1', 'Esc_off_hi/1');
LOGIC(n, 'Esc_off', 'OR', 2); L(n, 'Esc_off_lo/1', 'Esc_off/1'); L(n, 'Esc_off_hi/1', 'Esc_off/2');
SELB(n, B, 'Sel_fb_rx', {'drive_fb.L.rx_ok', 'drive_fb.R.rx_ok'});
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Rx_LR', 'NumInputs', '2');
L(n, 'Sel_fb_rx/1', 'Rx_LR/1'); L(n, 'Sel_fb_rx/2', 'Rx_LR/2');
LOGIC(n, 'Brk_side_ok', 'AND', 3);
L(n, 'Brk_engaged/1', 'Brk_side_ok/1', 'brake_engaged_LR'); L(n, 'Esc_off/1', 'Brk_side_ok/2', 'esc_off_LR');
L(n, 'Rx_LR/1', 'Brk_side_ok/3', 'rx_ok_LR');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Brk_both', 'Operator', 'AND', 'Inputs', '1');
L(n, 'Brk_side_ok/1', 'Brk_both/1', 'brake_ok_LR');
LOGIC(n, 'Brakes_ok', 'AND', 2); L(n, 'Brk_both/1', 'Brakes_ok/1', 'brake_ok_both'); L(n, 'Drv_fresh/1', 'Brakes_ok/2');
% constants on inputs (no parameter data): tick counts, masks, flags, stop-monitor values (single)
CST(n, 'Prm_n', ['round([' S '.selftest_t ' S '.selftest_timeout ' S '.auto_start_warn_t ' S '.standstill_t ' ...
    S '.ctrlstop_timeout P.electronics.shaper.estop_tmax ' S '.stopfail_t0 ' S '.stopfail_brake_t ' ...
    S '.stopfail_move_t]/Ts.ctrl)'], 'uint16', CT);
CST(n, 'Prm_mask', ['[' S '.maskA ' S '.maskB ' S '.mask_rc]'], 'uint32', CT);
CST(n, 'Prm_flag', ['[' S '.rc_required_in_auto ' S '.rearm.auto_selftest]'], 'boolean', CT);
CST(n, 'Prm_s', ['[P.electronics.shaper.a_ctrlstop P.electronics.shaper.a_estop ' S '.ctrlstop_t_extra ' ...
    S '.stopfail_dv ' S '.stopfail_rate ' S '.standstill_v 1/Ts.ctrl]'], 'single', CT);
args = {'rc_ok', 'sw_mode', 'arm', 'arm_req', 'sticks_neutral', 'override', 'reset_req', 'reset_evt', 'fault_word', ...
    'fault_cls', 'comp_ok', 'gnc_ready', 'mode_req', 'hv_ready', 'key_on', 'safety_ok', 'lid_closed', 'dock_present', ...
    'standstill', 'v_max', 'brakes_ok', 'selftest_ok', 'prm_n', 'prm_mask', 'prm_flag', 'prm_s'};
mf = A(n, 'simulink/User-Defined Functions/MATLAB Function', 'Mode_FSM');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = sprintf(['function [mode, cmd_src, drive_enable, rearm_req, hv_req, chg_req, rst_req, off_req, ' ...
    'stop_ctl] = fcn(%s)\n' ...
    '%%#codegen\n' ...
    '%% body: subsystems/electronics/mcu_mode_manager.m (ShaddadMode FSM, T09); bms_req leaves as 4 scalars\n' ...
    '[mode, cmd_src, drive_enable, rearm_req, bms_req, stop_ctl] = mcu_mode_manager(%s);\n' ...
    'hv_req = bms_req(1); chg_req = bms_req(2); rst_req = bms_req(3); off_req = bms_req(4);\n'], ...
    strjoin(args, ', '), strjoin(args, ', '));
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = true;
cfg.Description = 'MCU mode manager FSM (T09). Constants arrive on inputs (no parameter data).';
% the bus fields of the FSM in two picks right in front of it, inputs 1-8 and 11-14 (fault_word, fault_cls enter 9, 10)
SELB(n, B, 'Sel_fsm_rc', {'rc.rc_ok', 'rc.sw_mode', 'rc.arm', 'rc.arm_req', 'rc.sticks_neutral', 'rc.override', ...
    'reset_req', 'reset_evt'});
SELB(n, B, 'Sel_fsm_gnc', {'comp_ok', 'gnc_cmd.gnc_ready', 'gnc_cmd.mode_req', 'bms.hv_ready'});
src = {'Sel_fsm_rc/1', 'Sel_fsm_rc/2', 'Sel_fsm_rc/3', 'Sel_fsm_rc/4', 'Sel_fsm_rc/5', 'Sel_fsm_rc/6', ...
    'Sel_fsm_rc/7', 'Sel_fsm_rc/8', 'fault_word/1', 'fault_cls/1', 'Sel_fsm_gnc/1', 'Sel_fsm_gnc/2', ...
    'Sel_fsm_gnc/3', 'Sel_fsm_gnc/4', 'DI_key_on/1', 'DI_safety_ok/1', 'DI_lid_closed/1', 'DI_dock_present/1', ...
    'Standstill/1', 'V_max/1', 'Brakes_ok/1', 'Selftest_ok/1', 'Prm_n/1', 'Prm_mask/1', 'Prm_flag/1', 'Prm_s/1'};
for k = 1:numel(src), L(n, src{k}, sprintf('Mode_FSM/%d', k), args{k}); end
OUT(n, 'mode', 1, 'uint8'); OUT(n, 'cmd_src', 2, 'uint8'); OUT(n, 'drive_enable', 3, 'boolean');
OUT(n, 'rearm_req', 4, 'boolean'); OUT(n, 'bms_req', 5, 'boolean'); OUT(n, 'stop_ctl', 6, 'boolean');
nm = {'mode', 'cmd_src', 'drive_enable', 'rearm_req'};
for k = 1:4, L(n, sprintf('Mode_FSM/%d', k), [nm{k} '/1'], nm{k}); end
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Bms_req', 'NumInputs', '4');      % 1-D boolean[4]
nmq = {'hv_enable_req', 'charge_enable_req', 'fault_reset_req', 'shutdown_req'};
for k = 1:4, L(n, sprintf('Mode_FSM/%d', k + 4), sprintf('Bms_req/%d', k), nmq{k}); end
L(n, 'Bms_req/1', 'bms_req/1', 'bms_req');
L(n, 'Mode_FSM/9', 'stop_ctl/1', 'stop_ctl');                % boolean[2] [limits_hold stop_fail] (D71)
set_param(n, 'Description', ['fw_mode_manager (T09, Task_10, spec 2.7.3 with D18/D20/D21/D30/D34/D44/D71). in 1 ' ...
    'fw10 (Task_10 platform virtual bus), 2 fault_word uint32, 3 fault_cls boolean[3] [A B C] (fw_safety_supervisor, ' ...
    'same sample). out 1 mode uint8 (ShaddadMode), 2 cmd_src uint8 (ShaddadCmdSrc: RC in MANUAL, AUTO in AUTO, else ' ...
    'NONE), 3 drive_enable boolean (MANUAL HOLD AUTO_START AUTO, CTRL_STOP and ESTOP.DECEL while decelerating; off in ' ...
    'the parking phase), 4 rearm_req boolean (one sample on SELFTEST entry and on every reset_req rising edge, D30), ' ...
    '5 bms_req boolean[4] [HV_Enable_Req = key on AND NOT class A AND mode not BOOT/FAULT, Charge_Enable_Req = key on ' ...
    'AND DOCK_PRESENT AND STANDBY, Fault_Reset_Req = reset_req, Shutdown_Req = key off], 6 stop_ctl boolean[2] ' ...
    '[limits_hold (the parking phase of CTRL_STOP / ESTOP: park_req with the torque limits held until both brakes ' ...
    'confirm, to fw_vehicle_ctrl) stop_fail (the stop monitor, to fault bit 22 STOP_FAIL through the one-sample ' ...
    'delay Stop_fail_prev into fw_safety_supervisor input 3)] (D71, R-01, 2026-11-15). MATLAB Function Mode_FSM -> subsystems/electronics/' ...
    'mcu_mode_manager.m (transition table in its header); standstill = max(|odom.vTrack_LR|) < safety.standstill_v ' ...
    '(both tracks: a pivot turn is no standstill, R-34), v_max = that maximum; brakes_ok = both sides FB1 ' ...
    'brake_released false AND ESC state <= READY or >= FAULT AND rx_ok, with drv_age fresh; ' ...
    'selftest_ok = ESC frames fresh (drv_age), BMS fresh (bms_age), IMU valid, V24/V12/V5 >= minimum; tick counts ' ...
    '[selftest_t selftest_timeout auto_start_warn_t standstill_t ctrlstop_timeout shaper.estop_tmax stopfail_t0 ' ...
    'stopfail_brake_t stopfail_move_t]/Ts.ctrl, masks [maskA maskB mask_rc], flags [rc_required_in_auto ' ...
    'rearm.auto_selftest] and single values [a_ctrlstop a_estop ctrlstop_t_extra stopfail_dv stopfail_rate ' ...
    'standstill_v 1/Ts.ctrl] arrive on inputs. Built by scripts/build/build_fw_control.m.']);
end

% ===========================================================================================
% fw_vehicle_ctrl (Task_10)
% ===========================================================================================
function local_fw_vehicle_ctrl(n)
CT = 'Ts.ctrl'; SH = 'P.electronics.shaper'; LIM = 'P.electronics.lim';
IN(n, 'fw10', 1, '', CT); IN(n, 'mode', 2, 'uint8', CT); IN(n, 'cmd_src', 3, 'uint8', CT);
IN(n, 'drive_enable', 4, 'boolean', CT); IN(n, 'fault_word', 5, 'uint32', CT); IN(n, 'k_der', 6, 'single', CT);
IN(n, 'limits_hold', 7, 'boolean', CT);                     % parking phase: limits kept with the drive off (D71)
B = 'fw10/1';           % the bus line of the port fw10: each consumer picks its fields right in front of it (spec 6.2)
% ---- yaw rate for the control loop: gyro z re-biased with a bias average gated on BOTH tracks still AND on the
% commanded standstill (2026-11-14, S4 stop 3) ----
% (fw_imu_tilt gates its bias on BOTH tracks still since 2026-10-27; before, it used the MEAN track speed, which
% let pivot turns enter the bias. This re-bias of gyro_z = yaw_rate_meas + gyro_bias stays as a guard.) The measured
% gate alone admits a slow pivot with both tracks under standstill_v: in the S4 GNC pauses a residual Yaw_PI
% integrator held one, the average absorbed the turn, the re-biased rate read 0 and the PI never unwound. So the
% average also needs cmd_still (subsystem Cmd_still below) of the previous step (Unit Delay Cmd_still_z): w_ref
% follows this yaw rate through the Shaper_w reset (x_meas), so the current cmd_still would close an algebraic loop
% through the enable of Bias_avg
g = SUB(n, 'Yaw_meas', {'yaw_rate_meas', 'gyro_bias', 'v_track_LR', 'cmd_still'}, {'yaw_rate'});
A(g, 'simulink/Math Operations/Sum', 'Gyro_z', 'Inputs', '++');
L(g, 'yaw_rate_meas/1', 'Gyro_z/1', 'yaw_rate_meas'); L(g, 'gyro_bias/1', 'Gyro_z/2', 'gyro_bias');
A(g, 'simulink/Math Operations/Abs', 'Vt_abs'); L(g, 'v_track_LR/1', 'Vt_abs/1', 'v_track_LR');
A(g, 'simulink/Math Operations/MinMax', 'Vt_max', 'Function', 'max', 'Inputs', '1'); L(g, 'Vt_abs/1', 'Vt_max/1');
CMP(g, 'Still', '<', 'P.electronics.yaw.standstill_v'); L(g, 'Vt_max/1', 'Still/1');
A(g, 'simulink/Discrete/Unit Delay', 'Cmd_still_z', 'InitialCondition', '0', 'SampleTime', '-1');
L(g, 'cmd_still/1', 'Cmd_still_z/1', 'cmd_still');
LOGIC(g, 'Bias_en', 'AND', 2); L(g, 'Still/1', 'Bias_en/1', 'still'); L(g, 'Cmd_still_z/1', 'Bias_en/2', 'cmd_still_prev');
b = [g '/Bias_avg'];
add_block('built-in/SubSystem', b);
add_block('simulink/Ports & Subsystems/Enable', [b '/Enable'], 'StatesWhenEnabling', 'held');
A(b, 'simulink/Sources/In1', 'gyro_z', 'Port', '1');
A(b, 'simulink/Sinks/Out1', 'bias', 'Port', '1', 'OutputWhenDisabled', 'held', 'InitialOutput', '0');
A(b, 'simulink/Discrete/Transfer Fcn First Order', 'Avg', 'PoleZ', 'exp(-Ts.ctrl/P.electronics.yaw.tau_bias)', 'ICPrevOutput', '0');
L(b, 'gyro_z/1', 'Avg/1', 'gyro_z'); L(b, 'Avg/1', 'bias/1', 'bias');
L(g, 'Gyro_z/1', 'Bias_avg/1', 'gyro_z');
ph = get_param(b, 'PortHandles'); ps = get_param([g '/Bias_en'], 'PortHandles');
h = add_line(g, ps.Outport(1), ph.Enable(1), 'autorouting', 'smart');
set_param(h, 'Name', 'bias_en');
A(g, 'simulink/Math Operations/Sum', 'Yaw_rate', 'Inputs', '+-');
L(g, 'Gyro_z/1', 'Yaw_rate/1'); L(g, 'Bias_avg/1', 'Yaw_rate/2', 'bias_still');
L(g, 'Yaw_rate/1', 'yaw_rate/1', 'yaw_rate');
PICK(n, B, 'Sel_yaw_meas', {'tilt.yaw_rate_meas', 'tilt.gyro_bias', 'odom.vTrack_LR'}, {'Yaw_meas/1', 'Yaw_meas/2', 'Yaw_meas/3'});
% the control loop uses the re-biased yaw rate Yaw_meas/1 (the shaper reset of the yaw axis, Yaw_PI, Traction)
% ---- profile index p: 1 MANUAL / HOLD / other, 2 AUTO_START / AUTO, 3 CTRL_STOP, 4 ESTOP (ESTOP.DECEL) ----
MODESET(n, 'Is_auto', 'mode/1', 'AUTO_START', 'AUTO');
CMP(n, 'Is_ctrlstop', '==', 'double(ShaddadMode.CTRL_STOP)'); CMP(n, 'Is_estop', '==', 'double(ShaddadMode.ESTOP)');
L(n, 'mode/1', 'Is_ctrlstop/1', 'mode'); L(n, 'mode/1', 'Is_estop/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Prof_flags', 'NumInputs', '3');
L(n, 'Is_auto/1', 'Prof_flags/1'); L(n, 'Is_ctrlstop/1', 'Prof_flags/2'); L(n, 'Is_estop/1', 'Prof_flags/3');
DTC(n, 'Prof_u8', 'uint8'); L(n, 'Prof_flags/1', 'Prof_u8/1');
CST(n, 'Prof_w8', '[1 2 3]', 'uint8', CT);
A(n, 'simulink/Math Operations/Product', 'Prof_wt', 'Inputs', '**', 'OutDataTypeStr', 'uint8');
L(n, 'Prof_u8/1', 'Prof_wt/1'); L(n, 'Prof_w8/1', 'Prof_wt/2');
A(n, 'simulink/Math Operations/Sum', 'Prof_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions', 'OutDataTypeStr', 'uint8');
CST(n, 'One_u8', '1', 'uint8', CT);
A(n, 'simulink/Math Operations/Sum', 'Prof_idx', 'Inputs', '++', 'OutDataTypeStr', 'uint8');
L(n, 'Prof_wt/1', 'Prof_sum/1'); L(n, 'Prof_sum/1', 'Prof_idx/1'); L(n, 'One_u8/1', 'Prof_idx/2');
CMP(n, 'Is_stop', '>=', '3'); L(n, 'Prof_idx/1', 'Is_stop/1', 'profile');
% AUTO acceleration cap from the companion (accel_limit > 0 caps a_acc_auto)
A(n, 'simulink/Math Operations/MinMax', 'Cap_min', 'Function', 'min', 'Inputs', '2');
CST(n, 'A_acc_auto', [SH '.a_acc_auto'], 'single', CT);
PICK(n, B, 'Sel_accel_cap', {'gnc_cmd.accel_limit'}, {'Cap_min/1'}); L(n, 'A_acc_auto/1', 'Cap_min/2');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Cap_on', 'relop', '>', 'OutDataTypeStr', 'boolean');
PICK(n, B, 'Sel_accel_on', {'gnc_cmd.accel_limit'}, {'Cap_on/1'});
A(n, 'simulink/Signal Routing/Switch', 'A_acc_auto_eff', 'Criteria', 'u2 ~= 0');
L(n, 'Cap_min/1', 'A_acc_auto_eff/1'); L(n, 'Cap_on/1', 'A_acc_auto_eff/2'); L(n, 'A_acc_auto/1', 'A_acc_auto_eff/3');
% profile vectors [a_acc a_dec jerk K a_stop] per axis
CST(n, 'Pv_man', ['[' SH '.a_acc_man ' SH '.a_dec_man ' SH '.j_max ' SH '.K_v 0]'], 'single', CT);
CST(n, 'Pv_auto_rest', ['[' SH '.a_dec_auto ' SH '.j_max ' SH '.K_v 0]'], 'single', CT);
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Pv_auto', 'NumInputs', '2');
L(n, 'A_acc_auto_eff/1', 'Pv_auto/1', 'a_acc_auto'); L(n, 'Pv_auto_rest/1', 'Pv_auto/2');
CST(n, 'Pv_ctrl', ['[' SH '.a_ctrlstop ' SH '.a_ctrlstop ' SH '.j_stop ' SH '.K_v ' SH '.a_ctrlstop]'], 'single', CT);
CST(n, 'Pv_estop', ['[' SH '.a_estop ' SH '.a_estop ' SH '.j_stop ' SH '.K_v ' SH '.a_estop]'], 'single', CT);
CST(n, 'Pw_man', ['[' SH '.alpha_max ' SH '.alpha_max ' SH '.jw_max ' SH '.K_w 0]'], 'single', CT);
CST(n, 'Pw_auto', ['[' SH '.alpha_auto ' SH '.alpha_auto ' SH '.jw_auto ' SH '.K_w_auto 0]'], 'single', CT);
CST(n, 'Pw_stop', ['[' SH '.alpha_stop ' SH '.alpha_stop ' SH '.jw_stop ' SH '.K_w ' SH '.alpha_stop]'], 'single', CT);
for ax = {'v', {'Pv_man/1', 'Pv_auto/1', 'Pv_ctrl/1', 'Pv_estop/1'}; 'w', {'Pw_man/1', 'Pw_auto/1', 'Pw_stop/1', 'Pw_stop/1'}}'
    m = ['Prof_' ax{1}];
    A(n, 'simulink/Signal Routing/Multiport Switch', m, 'DataPortOrder', 'One-based contiguous', 'Inputs', '4', ...
        'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
    L(n, 'Prof_idx/1', [m '/1']);
    for k = 1:4, L(n, ax{2}{k}, sprintf('%s/%d', m, k + 1)); end
end
% ---- arbitration (Multiport Switch on cmd_src), derate, lateral-acceleration limit ----
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Cmd_rc', 'NumInputs', '2');
PICK(n, B, 'Sel_rc', {'rc.v_rc', 'rc.w_rc'}, {'Cmd_rc/1', 'Cmd_rc/2'});
A(n, 'simulink/Discontinuities/Saturation', 'V_auto_sat', 'UpperLimit', [LIM '.v_max_auto'], 'LowerLimit', ['-' LIM '.v_rev_auto']);
A(n, 'simulink/Discontinuities/Saturation', 'W_auto_sat', 'UpperLimit', [LIM '.w_max_auto'], 'LowerLimit', ['-' LIM '.w_max_auto']);
PICK(n, B, 'Sel_v_cmd', {'gnc_cmd.v_cmd'}, {'V_auto_sat/1'}); PICK(n, B, 'Sel_omega_cmd', {'gnc_cmd.omega_cmd'}, {'W_auto_sat/1'});
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Cmd_auto_raw', 'NumInputs', '2');
L(n, 'V_auto_sat/1', 'Cmd_auto_raw/1'); L(n, 'W_auto_sat/1', 'Cmd_auto_raw/2');
LOGIC(n, 'No_soft_stop', 'NOT', 1); PICK(n, B, 'Sel_soft_stop', {'gnc_cmd.soft_stop_req'}, {'No_soft_stop/1'});
LOGIC(n, 'Auto_valid', 'AND', 2); PICK(n, B, 'Sel_cmd_valid', {'gnc_cmd.cmd_valid'}, {'Auto_valid/1'});
L(n, 'No_soft_stop/1', 'Auto_valid/2');
CST(n, 'Zero2', 'zeros(1,2)', 'single', CT);
A(n, 'simulink/Signal Routing/Switch', 'Cmd_auto', 'Criteria', 'u2 ~= 0');
L(n, 'Cmd_auto_raw/1', 'Cmd_auto/1'); L(n, 'Auto_valid/1', 'Cmd_auto/2'); L(n, 'Zero2/1', 'Cmd_auto/3');
A(n, 'simulink/Signal Routing/Multiport Switch', 'Arbitration', 'DataPortOrder', 'Specify indices', ...
    'DataPortIndices', '{1,2}', 'DataPortForDefault', 'Additional data port', 'DiagnosticForDefault', 'None');
L(n, 'cmd_src/1', 'Arbitration/1', 'cmd_src'); L(n, 'Cmd_rc/1', 'Arbitration/2'); L(n, 'Cmd_auto/1', 'Arbitration/3');
L(n, 'Zero2/1', 'Arbitration/4');
A(n, 'simulink/Signal Routing/Demux', 'Cmd_vw', 'Outputs', '2'); L(n, 'Arbitration/1', 'Cmd_vw/1', 'cmd_sel');
A(n, 'simulink/Math Operations/Gain', 'V_lim', 'Gain', [LIM '.v_track_max']);
L(n, 'k_der/1', 'V_lim/1', 'k_der');
% descent speed cap (Agents task descent; the user's decision of 2026-11-15: always on, in every mode, by slope and
% fitted implement): subsystem Descent_cap (local_descent_cap, ref tables: downhill pitch +/-tilt.pitch peak-held t_hold,
% theta_imu while tilt.imu_valid is false, the implement row of the coding plug DI IMPL_ID, an unknown implement the most
% restrictive) gives [ref_fwd ref_rev] = cap - descent.dv; v_target = sat(v_sel, -min(k_der*v_track_max, ref_rev),
% min(k_der*v_track_max, ref_fwd)) (the supervisor trips OVERSPEED at the cap); tiller fitted: ref_rev = 0, no reversing
c = SUB(n, 'Descent_cap', {'pitch', 'imu_valid', 'di_word', 'impl_type', 'impl_rx_ok', 'lift_pos'}, {'ref_fwd', 'ref_rev'});
src = local_descent_cap(c, {'ref'});
L(c, src.ref{1}, 'ref_fwd/1', 'ref_fwd'); L(c, src.ref{2}, 'ref_rev/1', 'ref_rev');
PICK(n, B, 'Sel_descent', {'tilt.pitch', 'tilt.imu_valid', 'di_word', 'impl.impl_type', 'impl.rx_ok', 'adc.lift_pos'}, ...
    arrayfun(@(k) sprintf('Descent_cap/%d', k), 1:6, 'UniformOutput', false));
A(n, 'simulink/Math Operations/MinMax', 'V_up', 'Function', 'min', 'Inputs', '2');
A(n, 'simulink/Math Operations/MinMax', 'V_dn', 'Function', 'min', 'Inputs', '2');
L(n, 'V_lim/1', 'V_up/1', 'v_lim'); L(n, 'Descent_cap/1', 'V_up/2', 'ref_fwd');
L(n, 'V_lim/1', 'V_dn/1'); L(n, 'Descent_cap/2', 'V_dn/2', 'ref_rev');
A(n, 'simulink/Math Operations/Gain', 'V_lim_neg', 'Gain', '-1'); L(n, 'V_dn/1', 'V_lim_neg/1', 'v_dn');
A(n, 'simulink/Discontinuities/Saturation Dynamic', 'V_target');
L(n, 'V_up/1', 'V_target/1', 'v_up'); L(n, 'Cmd_vw/1', 'V_target/2', 'v_sel'); L(n, 'V_lim_neg/1', 'V_target/3');
% |w| <= min(k_der*w_max, a_lat_max / max(|v_ref|, 0.1))
A(n, 'simulink/Math Operations/Gain', 'W_lim_der', 'Gain', ['max(' LIM '.w_max_man, ' LIM '.w_max_auto)']);
L(n, 'k_der/1', 'W_lim_der/1');
A(n, 'simulink/Math Operations/Abs', 'Vref_abs');
CST(n, 'V_lat_min', [LIM '.v_lat_min'], 'single', CT);
A(n, 'simulink/Math Operations/MinMax', 'Vref_floor', 'Function', 'max', 'Inputs', '2');
CST(n, 'A_lat', [LIM '.a_lat_max'], 'single', CT);
A(n, 'simulink/Math Operations/Divide', 'W_lat');
A(n, 'simulink/Math Operations/MinMax', 'W_lim', 'Function', 'min', 'Inputs', '2');
A(n, 'simulink/Math Operations/Gain', 'W_lim_neg', 'Gain', '-1');
L(n, 'Vref_abs/1', 'Vref_floor/1'); L(n, 'V_lat_min/1', 'Vref_floor/2');
L(n, 'A_lat/1', 'W_lat/1'); L(n, 'Vref_floor/1', 'W_lat/2');
L(n, 'W_lim_der/1', 'W_lim/1'); L(n, 'W_lat/1', 'W_lim/2'); L(n, 'W_lim/1', 'W_lim_neg/1');
A(n, 'simulink/Discontinuities/Saturation Dynamic', 'W_target');
L(n, 'W_lim/1', 'W_target/1', 'w_lim'); L(n, 'Cmd_vw/2', 'W_target/2', 'w_sel'); L(n, 'W_lim_neg/1', 'W_target/3');
% ---- shapers ----
local_shaper(n, 'Shaper_v');
local_shaper(n, 'Shaper_w');
L(n, 'V_target/1', 'Shaper_v/1', 'v_target'); PICK(n, B, 'Sel_v_track', {'v_track'}, {'Shaper_v/2'});
L(n, 'drive_enable/1', 'Shaper_v/3', 'drive_enable'); L(n, 'Prof_v/1', 'Shaper_v/4', 'prof_v'); L(n, 'Is_stop/1', 'Shaper_v/5', 'is_stop');
L(n, 'W_target/1', 'Shaper_w/1', 'w_target'); L(n, 'Yaw_meas/1', 'Shaper_w/2', 'yaw_rate_meas');
L(n, 'drive_enable/1', 'Shaper_w/3'); L(n, 'Prof_w/1', 'Shaper_w/4', 'prof_w'); L(n, 'Is_stop/1', 'Shaper_w/5');
L(n, 'Shaper_v/1', 'Vref_abs/1', 'v_ref');
% ---- commanded standstill (2026-11-14, S4 stop 3): both targets exactly 0 (the arbitration passes a zero command
% unchanged: RC deadband, Zero2, COMP_CMD v_cmd = omega_cmd = 0) and both shaped references under yaw.cmd_still_v /
% yaw.cmd_still_w (they decay exponentially to 0), or the drive disabled (the shapers then follow the measured v_track
% and yaw rate, so their values are no command). It resets Yaw_PI and, with the measured standstill, enables the
% gyro re-bias of Yaw_meas ----
c = SUB(n, 'Cmd_still', {'v_target', 'w_target', 'v_ref', 'w_ref', 'drive_enable'}, {'cmd_still'});
A(c, 'simulink/Logic and Bit Operations/Compare To Zero', 'Vt_zero', 'relop', '==', 'OutDataTypeStr', 'boolean');
A(c, 'simulink/Logic and Bit Operations/Compare To Zero', 'Wt_zero', 'relop', '==', 'OutDataTypeStr', 'boolean');
L(c, 'v_target/1', 'Vt_zero/1', 'v_target'); L(c, 'w_target/1', 'Wt_zero/1', 'w_target');
A(c, 'simulink/Math Operations/Abs', 'Vr_abs'); A(c, 'simulink/Math Operations/Abs', 'Wr_abs');
L(c, 'v_ref/1', 'Vr_abs/1', 'v_ref'); L(c, 'w_ref/1', 'Wr_abs/1', 'w_ref');
CMP(c, 'Vr_small', '<', 'P.electronics.yaw.cmd_still_v'); CMP(c, 'Wr_small', '<', 'P.electronics.yaw.cmd_still_w');
L(c, 'Vr_abs/1', 'Vr_small/1'); L(c, 'Wr_abs/1', 'Wr_small/1');
LOGIC(c, 'Cmd_zero', 'AND', 4);
L(c, 'Vt_zero/1', 'Cmd_zero/1'); L(c, 'Wt_zero/1', 'Cmd_zero/2'); L(c, 'Vr_small/1', 'Cmd_zero/3'); L(c, 'Wr_small/1', 'Cmd_zero/4');
LOGIC(c, 'Drive_off', 'NOT', 1); L(c, 'drive_enable/1', 'Drive_off/1', 'drive_enable');
LOGIC(c, 'Standstill', 'OR', 2); L(c, 'Cmd_zero/1', 'Standstill/1', 'cmd_zero'); L(c, 'Drive_off/1', 'Standstill/2', 'drive_off');
L(c, 'Standstill/1', 'cmd_still/1', 'cmd_still');
L(n, 'V_target/1', 'Cmd_still/1'); L(n, 'W_target/1', 'Cmd_still/2'); L(n, 'Shaper_v/1', 'Cmd_still/3');
L(n, 'Shaper_w/1', 'Cmd_still/4', 'w_ref'); L(n, 'drive_enable/1', 'Cmd_still/5');
L(n, 'Cmd_still/1', 'Yaw_meas/4', 'cmd_still');
% ---- yaw-rate PI on the bias-corrected gyro (D2): w_mix = w_ref + u. Level reset (integrator 0) and u = 0 while the
% drive is off, on an invalid IMU sample (tilt.imu_valid false; 2026-11-15, R-05, review MCU-06: before, only the
% persisted IMU_FAULT bit 24 reset it, so the loop acted on invalid gyro data for 0.5 s), with yaw.enable false and
% at commanded standstill (a reset, not a bleed: no residual integrator may survive into a pause, S4 stop 3) ----
y = SUB(n, 'Yaw_PI', {'w_ref', 'yaw_meas', 'enable', 'imu_valid', 'cmd_still'}, {'u'});
A(y, 'simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
L(y, 'w_ref/1', 'Err/1', 'w_ref'); L(y, 'yaw_meas/1', 'Err/2', 'yaw_meas');
LOGIC(y, 'Imu_bad', 'NOT', 1); L(y, 'imu_valid/1', 'Imu_bad/1', 'imu_valid');
CST(y, 'Yaw_en', 'P.electronics.yaw.enable', 'boolean', CT);
LOGIC(y, 'Yaw_dis', 'NOT', 1); L(y, 'Yaw_en/1', 'Yaw_dis/1');
LOGIC(y, 'Drive_off', 'NOT', 1); L(y, 'enable/1', 'Drive_off/1', 'enable');
LOGIC(y, 'Off', 'OR', 4); L(y, 'Drive_off/1', 'Off/1'); L(y, 'Imu_bad/1', 'Off/2', 'imu_bad'); L(y, 'Yaw_dis/1', 'Off/3');
L(y, 'cmd_still/1', 'Off/4', 'cmd_still');
A(y, 'simulink/Discrete/Discrete PID Controller', 'PI', 'Controller', 'PI', 'SampleTime', CT, ...
    'P', 'P.electronics.yaw.Kp', 'I', 'P.electronics.yaw.Ki', 'IntegratorMethod', 'Forward Euler', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'P.electronics.yaw.u_max', 'LowerSaturationLimit', '-P.electronics.yaw.u_max', ...
    'AntiWindupMode', 'clamping', 'ExternalReset', 'level');
L(y, 'Err/1', 'PI/1', 'e_w'); L(y, 'Off/1', 'PI/2', 'pi_reset');
CST(y, 'Zero', '0', 'single', CT);
A(y, 'simulink/Signal Routing/Switch', 'U_eff', 'Criteria', 'u2 ~= 0');
L(y, 'Zero/1', 'U_eff/1'); L(y, 'Off/1', 'U_eff/2'); L(y, 'PI/1', 'U_eff/3', 'u_pi');
L(y, 'U_eff/1', 'u/1', 'u');
L(n, 'Shaper_w/1', 'Yaw_PI/1', 'w_ref'); L(n, 'Yaw_meas/1', 'Yaw_PI/2');
L(n, 'drive_enable/1', 'Yaw_PI/3'); PICK(n, B, 'Sel_imu_valid', {'tilt.imu_valid'}, {'Yaw_PI/4'});
L(n, 'Cmd_still/1', 'Yaw_PI/5');
% the root input fault_word (port 5) has had no consumer since the yaw PI resets on imu_valid (2026-11-15): it stays,
% because Shaddad_MCU and mcu_ctrl_harness wire the ports by number (T08 port list), and ends in a Terminator
A(n, 'simulink/Sinks/Terminator', 'T_fault_word'); L(n, 'fault_word/1', 'T_fault_word/1', 'fault_word');
A(n, 'simulink/Math Operations/Sum', 'W_mix', 'Inputs', '++');
L(n, 'Shaper_w/1', 'W_mix/1'); L(n, 'Yaw_PI/1', 'W_mix/2', 'u_yaw');
% ---- skid mixer with curvature-preserving desaturation (spec 2.7.6, chi from P.electronics.mix, D39) ----
m = SUB(n, 'Mixer', {'v_ref', 'w_mix', 'a_ref', 'alpha_ref'}, {'vtrack_ref', 'atrack_ref'});
A(m, 'simulink/Math Operations/Gain', 'Half_w', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2');
A(m, 'simulink/Math Operations/Gain', 'Half_a', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2');
L(m, 'w_mix/1', 'Half_w/1', 'w_mix'); L(m, 'alpha_ref/1', 'Half_a/1', 'alpha_ref');
for q = {'v', 'v_ref/1', 'Half_w/1'; 'a', 'a_ref/1', 'Half_a/1'}'
    A(m, 'simulink/Math Operations/Sum', [q{1} '_L'], 'Inputs', '+-'); A(m, 'simulink/Math Operations/Sum', [q{1} '_R'], 'Inputs', '++');
    L(m, q{2}, [q{1} '_L/1']); L(m, q{3}, [q{1} '_L/2']); L(m, q{2}, [q{1} '_R/1']); L(m, q{3}, [q{1} '_R/2']);
    A(m, 'simulink/Signal Routing/Vector Concatenate', [q{1} '_LR'], 'NumInputs', '2');
    L(m, [q{1} '_L/1'], [q{1} '_LR/1']); L(m, [q{1} '_R/1'], [q{1} '_LR/2']);
end
A(m, 'simulink/Math Operations/Abs', 'V_abs'); A(m, 'simulink/Math Operations/MinMax', 'V_peak', 'Function', 'max', 'Inputs', '1');
A(m, 'simulink/Math Operations/Gain', 'V_ratio', 'Gain', ['1/' LIM '.v_track_max']);
CST(m, 'One', '1', 'single', CT);
A(m, 'simulink/Math Operations/MinMax', 'K_desat', 'Function', 'max', 'Inputs', '2');
L(m, 'v_LR/1', 'V_abs/1', 'v_LR'); L(m, 'V_abs/1', 'V_peak/1'); L(m, 'V_peak/1', 'V_ratio/1');
L(m, 'V_ratio/1', 'K_desat/1'); L(m, 'One/1', 'K_desat/2');
A(m, 'simulink/Math Operations/Product', 'V_desat', 'Inputs', '*/'); A(m, 'simulink/Math Operations/Product', 'A_desat', 'Inputs', '*/');
L(m, 'v_LR/1', 'V_desat/1'); L(m, 'K_desat/1', 'V_desat/2', 'k_desat'); L(m, 'a_LR/1', 'A_desat/1', 'a_LR'); L(m, 'K_desat/1', 'A_desat/2');
L(m, 'V_desat/1', 'vtrack_ref/1', 'vtrack_ref'); L(m, 'A_desat/1', 'atrack_ref/1', 'atrack_ref');
L(n, 'Shaper_v/1', 'Mixer/1'); L(n, 'W_mix/1', 'Mixer/2', 'w_mix'); L(n, 'Shaper_v/2', 'Mixer/3', 'a_ref');
L(n, 'Shaper_w/2', 'Mixer/4', 'alpha_ref');
% ---- ESC setpoints: w_ref = k_v2w*v_track_ref, tau_ff = Kff_a*a_track_ref ----
A(n, 'simulink/Math Operations/Gain', 'W_ref_motor', 'Gain', 'P.electronics.drv.k_v2w');
A(n, 'simulink/Discontinuities/Saturation', 'W_ref_sat', 'UpperLimit', 'P.electronics.drv.w_ref_max', ...
    'LowerLimit', '-P.electronics.drv.w_ref_max');
A(n, 'simulink/Math Operations/Gain', 'Tau_ff', 'Gain', 'P.electronics.drv.Kff_a');
A(n, 'simulink/Discontinuities/Saturation', 'Tau_ff_sat', 'UpperLimit', 'P.electronics.drv.T_peak', ...
    'LowerLimit', '-P.electronics.drv.T_peak');
L(n, 'Mixer/1', 'W_ref_motor/1', 'vtrack_ref'); L(n, 'W_ref_motor/1', 'W_ref_sat/1');
L(n, 'Mixer/2', 'Tau_ff/1', 'atrack_ref'); L(n, 'Tau_ff/1', 'Tau_ff_sat/1');
% ---- traction control (slip relay + rate limiter, only with v_ground_valid) ----
t = SUB(n, 'Traction', {'v_track_LR', 'v_ground', 'v_ground_valid', 'yaw_rate'}, {'k_tr'});
A(t, 'simulink/Math Operations/Gain', 'Yaw_lever', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2*[-1 1]', ...
    'Multiplication', 'Element-wise(K.*u)');
A(t, 'simulink/Math Operations/Sum', 'V_body', 'Inputs', '++');
L(t, 'yaw_rate/1', 'Yaw_lever/1', 'yaw_rate'); L(t, 'v_ground/1', 'V_body/1', 'v_ground'); L(t, 'Yaw_lever/1', 'V_body/2');
A(t, 'simulink/Math Operations/Sum', 'Dv', 'Inputs', '+-');
A(t, 'simulink/Math Operations/Sign', 'Dir');
A(t, 'simulink/Math Operations/Abs', 'Vt_abs');
CST(t, 'V_min', 'P.electronics.trac.v_min', 'single', CT);
A(t, 'simulink/Math Operations/MinMax', 'Den', 'Function', 'max', 'Inputs', '2');
A(t, 'simulink/Math Operations/Product', 'Slip', 'Inputs', '**/');
L(t, 'v_track_LR/1', 'Dv/1', 'v_track_LR'); L(t, 'V_body/1', 'Dv/2', 'v_body');
L(t, 'v_track_LR/1', 'Dir/1'); L(t, 'v_track_LR/1', 'Vt_abs/1'); L(t, 'Vt_abs/1', 'Den/1'); L(t, 'V_min/1', 'Den/2');
L(t, 'Dv/1', 'Slip/1'); L(t, 'Dir/1', 'Slip/2'); L(t, 'Den/1', 'Slip/3');
A(t, 'simulink/Discontinuities/Relay', 'Slip_relay', 'OnSwitchValue', 'P.electronics.trac.s_on', ...
    'OffSwitchValue', 'P.electronics.trac.s_off', 'OnOutputValue', 'P.electronics.trac.k_min', 'OffOutputValue', '1', ...
    'OutDataTypeStr', 'single', 'SampleTime', '-1');
L(t, 'Slip/1', 'Slip_relay/1', 'slip');
CST(t, 'Trac_en', 'P.electronics.trac.enable', 'boolean', CT);
LOGIC(t, 'Active', 'AND', 2); L(t, 'v_ground_valid/1', 'Active/1', 'v_ground_valid'); L(t, 'Trac_en/1', 'Active/2');
CST(t, 'One2', 'ones(1,2)', 'single', CT);
A(t, 'simulink/Signal Routing/Switch', 'K_target', 'Criteria', 'u2 ~= 0');
L(t, 'Slip_relay/1', 'K_target/1'); L(t, 'Active/1', 'K_target/2'); L(t, 'One2/1', 'K_target/3');
A(t, 'simulink/Discontinuities/Rate Limiter', 'K_rate', 'RisingSlewLimit', 'P.electronics.trac.k_up', ...
    'FallingSlewLimit', '-P.electronics.trac.k_dn', 'SampleTimeMode', 'inherited', 'InitialCondition', '1');
L(t, 'K_target/1', 'K_rate/1', 'k_target'); L(t, 'K_rate/1', 'k_tr/1', 'k_tr');
PICK(n, B, 'Sel_traction', {'odom.vTrack_LR', 'gnc_cmd.v_ground_est', 'gnc_cmd.v_ground_valid'}, ...
    {'Traction/1', 'Traction/2', 'Traction/3'});
L(n, 'Yaw_meas/1', 'Traction/4');
% ---- torque / power limits (spec 2.7.9, D2: tau_lim_mot / tau_lim_reg) ----
q = SUB(n, 'Torque_Limits', {'V_pack', 'DCL', 'CCL', 'w_m', 'tau_avail', 'k_tr', 'k_der', 'enable'}, {'tau_lim_mot', 'tau_lim_reg'});
TL = 'P.electronics.tlim'; DR = 'P.electronics.drv';
A(q, 'simulink/Math Operations/Product', 'VxDCL', 'Inputs', '**'); A(q, 'simulink/Math Operations/Product', 'VxCCL', 'Inputs', '**');
L(q, 'V_pack/1', 'VxDCL/1', 'V_pack'); L(q, 'DCL/1', 'VxDCL/2', 'DCL'); L(q, 'V_pack/1', 'VxCCL/1'); L(q, 'CCL/1', 'VxCCL/2', 'CCL');
A(q, 'simulink/Math Operations/Gain', 'Margin_d', 'Gain', [TL '.dis_margin']); A(q, 'simulink/Math Operations/Gain', 'Margin_c', 'Gain', [TL '.dis_margin']);
CST(q, 'P_aux', [TL '.P_aux_est'], 'single', CT);
A(q, 'simulink/Math Operations/Sum', 'P_dis', 'Inputs', '+-'); A(q, 'simulink/Math Operations/Sum', 'P_chg', 'Inputs', '++');
L(q, 'VxDCL/1', 'Margin_d/1'); L(q, 'Margin_d/1', 'P_dis/1'); L(q, 'P_aux/1', 'P_dis/2');
L(q, 'VxCCL/1', 'Margin_c/1'); L(q, 'Margin_c/1', 'P_chg/1'); L(q, 'P_aux/1', 'P_chg/2');
A(q, 'simulink/Math Operations/Abs', 'Wm_abs'); CST(q, 'W_min', [DR '.w_min'], 'single', CT);
A(q, 'simulink/Math Operations/MinMax', 'W_star', 'Function', 'max', 'Inputs', '2');
L(q, 'w_m/1', 'Wm_abs/1', 'w_m'); L(q, 'Wm_abs/1', 'W_star/1'); L(q, 'W_min/1', 'W_star/2');
A(q, 'simulink/Math Operations/Gain', 'Half_eta', 'Gain', [DR '.eta_drv/2']);
A(q, 'simulink/Math Operations/Gain', 'Half_over_eta', 'Gain', ['1/(2*' DR '.eta_drv)']);
A(q, 'simulink/Math Operations/Product', 'T_pow', 'Inputs', '*/'); A(q, 'simulink/Math Operations/Product', 'T_regen', 'Inputs', '*/');
L(q, 'P_dis/1', 'Half_eta/1', 'P_dis'); L(q, 'Half_eta/1', 'T_pow/1'); L(q, 'W_star/1', 'T_pow/2', 'w_star');
L(q, 'P_chg/1', 'Half_over_eta/1', 'P_chg'); L(q, 'Half_over_eta/1', 'T_regen/1'); L(q, 'W_star/1', 'T_regen/2');
CST(q, 'T_peak', [DR '.T_peak'], 'single', CT);
A(q, 'simulink/Math Operations/MinMax', 'Mot_min', 'Function', 'min', 'Inputs', '3');
L(q, 'T_peak/1', 'Mot_min/1'); L(q, 'tau_avail/1', 'Mot_min/2', 'tau_avail'); L(q, 'T_pow/1', 'Mot_min/3', 'T_pow');
A(q, 'simulink/Math Operations/Product', 'Mot_scaled', 'Inputs', '***');
L(q, 'Mot_min/1', 'Mot_scaled/1'); L(q, 'k_tr/1', 'Mot_scaled/2', 'k_tr'); L(q, 'k_der/1', 'Mot_scaled/3', 'k_der');
A(q, 'simulink/Math Operations/MinMax', 'Reg_min', 'Function', 'min', 'Inputs', '2');
L(q, 'T_peak/1', 'Reg_min/1'); L(q, 'T_regen/1', 'Reg_min/2', 'T_regen');
CST(q, 'Zero2', 'zeros(1,2)', 'single', CT);
for k = {'Mot', 'Mot_scaled/1', 'tau_lim_mot'; 'Reg', 'Reg_min/1', 'tau_lim_reg'}'
    A(q, 'simulink/Discontinuities/Saturation', [k{1} '_sat'], 'UpperLimit', [DR '.T_peak'], 'LowerLimit', '0');
    A(q, 'simulink/Signal Routing/Switch', [k{1} '_en'], 'Criteria', 'u2 ~= 0');
    L(q, k{2}, [k{1} '_sat/1']); L(q, [k{1} '_sat/1'], [k{1} '_en/1']); L(q, 'enable/1', [k{1} '_en/2'], 'enable');
    L(q, 'Zero2/1', [k{1} '_en/3']); L(q, [k{1} '_en/1'], [k{3} '/1'], k{3});
end
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Wm_LR', 'NumInputs', '2');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Tau_avail_LR', 'NumInputs', '2');
PICK(n, B, 'Sel_w_m', {'drive_fb.L.w_m', 'drive_fb.R.w_m'}, {'Wm_LR/1', 'Wm_LR/2'});
PICK(n, B, 'Sel_tau_avail', {'drive_fb.L.tau_avail', 'drive_fb.R.tau_avail'}, {'Tau_avail_LR/1', 'Tau_avail_LR/2'});
PICK(n, B, 'Sel_bms', {'bms.V_pack', 'bms.DCL', 'bms.CCL'}, {'Torque_Limits/1', 'Torque_Limits/2', 'Torque_Limits/3'});
L(n, 'Wm_LR/1', 'Torque_Limits/4', 'w_m');
L(n, 'Tau_avail_LR/1', 'Torque_Limits/5', 'tau_avail'); L(n, 'Traction/1', 'Torque_Limits/6', 'k_tr');
L(n, 'k_der/1', 'Torque_Limits/7');
% the limits are on while the drive is enabled or in the parking phase of a stop (limits_hold, mode manager stop_ctl(1):
% park_req = NOT drive_enable with the limits kept until both brakes confirm, D71, R-01); 0 otherwise as before
LOGIC(n, 'Lim_en', 'OR', 2); L(n, 'drive_enable/1', 'Lim_en/1'); L(n, 'limits_hold/1', 'Lim_en/2', 'limits_hold');
L(n, 'Lim_en/1', 'Torque_Limits/8', 'limits_on');
% ---- Bus_DriveCmd ----
for v = {'W_ref_sat/1', 'Wr', 'w_ref_LR'; 'Tau_ff_sat/1', 'Tf', 'tau_ff_LR'; 'Torque_Limits/1', 'Tm', 'tau_lim_mot_LR'; ...
        'Torque_Limits/2', 'Tg', 'tau_lim_reg_LR'}'
    A(n, 'simulink/Signal Routing/Demux', ['Dmx_' v{2}], 'Outputs', '2'); L(n, v{1}, ['Dmx_' v{2} '/1'], v{3});
end
LOGIC(n, 'Park', 'NOT', 1); L(n, 'drive_enable/1', 'Park/1');
CST(n, 'Mode_speed', '1', 'uint8', CT);
A(n, 'simulink/Sources/Counter Free-Running', 'Alive', 'NumBits', '8', 'tsamp', CT);
for sd = {'L', 1; 'R', 2}'
    SELB(n, B, ['Sel_reset_' sd{1}], {'reset_req'});      % fault_reset, right in front of Side_<side>
    BCB(n, ['Side_' sd{1}], 'Bus_DriveSideCmd', {'enable', 'drive_enable/1'; 'mode', 'Mode_speed/1'; ...
        'w_ref', sprintf('Dmx_Wr/%d', sd{2}); 'tau_ff', sprintf('Dmx_Tf/%d', sd{2}); ...
        'tau_lim_mot', sprintf('Dmx_Tm/%d', sd{2}); 'tau_lim_reg', sprintf('Dmx_Tg/%d', sd{2}); ...
        'park_req', 'Park/1'; 'fault_reset', ['Sel_reset_' sd{1} '/1']; 'alive', 'Alive/1'});
end
A(n, 'simulink/Signal Routing/Bus Creator', 'Cmd_BC', 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_DriveCmd', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Side_L/1', 'Cmd_BC/1', 'L'); L(n, 'Side_R/1', 'Cmd_BC/2', 'R');
% ---- telemetry vector ----
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Vc_tel', 'NumInputs', '9');
srcs = {'V_target/1', 'W_target/1', 'Shaper_v/1', 'Shaper_w/1', 'Shaper_v/2', 'Shaper_w/2', 'W_mix/1', 'Mixer/1', 'Traction/1'};
for k = 1:numel(srcs), L(n, srcs{k}, sprintf('Vc_tel/%d', k)); end
OUT(n, 'drive_cmd', 1, 'Bus: Bus_DriveCmd'); OUT(n, 'vc_tel', 2, 'single');
L(n, 'Cmd_BC/1', 'drive_cmd/1', 'drive_cmd'); L(n, 'Vc_tel/1', 'vc_tel/1', 'vc_tel');
set_param(n, 'Description', ['fw_vehicle_ctrl (T09, Task_10, spec 2.7.4-2.7.9 with D2/D33/D39/D41/D71). in 1 fw10 ' ...
    '(Task_10 platform virtual bus), 2 mode uint8, 3 cmd_src uint8, 4 drive_enable boolean, 5 fault_word uint32 ' ...
    '(terminated: no consumer since 2026-11-15), 6 k_der single, 7 limits_hold boolean (mode manager stop_ctl(1): ' ...
    'the parking phase of a stop, D71). out 1 drive_cmd Bus_DriveCmd (-> drive_cmd_pack: enable = drive_enable, ' ...
    'mode 1 SPEED, w_ref = ' ...
    'k_v2w*v_track_ref (sat w_ref_max), tau_ff = Kff_a*a_track_ref (sat T_peak), tau_lim_mot = min(T_peak, tau_avail, ' ...
    '(P_dis/2)*eta_drv/w*)*k_tr*k_der, tau_lim_reg = min(T_peak, (P_chg/2)/(eta_drv*w*)), P_dis = dis_margin*V_pack*DCL ' ...
    '- P_aux_est, P_chg = dis_margin*V_pack*CCL + P_aux_est, w* = max(|w_m|, w_min), limits 0 unless drive_enable OR ' ...
    'limits_hold, ' ...
    'park_req = NOT drive_enable, fault_reset = reset_req, alive Counter Free-Running), 2 vc_tel single[11] [v_target ' ...
    'w_target v_ref w_ref a_ref alpha_ref w_mix vtrack_ref_L vtrack_ref_R k_tr_L k_tr_R]. Arbitration: Multiport ' ...
    'Switch on cmd_src (1 RC [v_rc w_rc], 2 AUTO [sat(v_cmd, -v_rev_auto, v_max_auto) sat(omega_cmd, +/-w_max_auto)] ' ...
    'when cmd_valid AND NOT soft_stop_req, default 0); v_target = sat(v_sel, -min(k_der*v_track_max, ref_rev), ' ...
    'min(k_der*v_track_max, ref_fwd)), [ref_fwd ref_rev] the descent speed cap minus descent.dv (subsystem ' ...
    'Descent_cap: descent.ref_fwd / ref_rev by implement row (DI IMPL_ID; the mower with adc.lift_pos >= ' ...
    'descent.lift_raised: the raised-deck row; IMPL_FB of another type = unknown, the most restrictive row) and ' ...
    'downhill pitch +/-tilt.pitch in deg, peak-held t_hold (Tapped Delay + MinMax), ' ...
    'theta_imu while tilt.imu_valid is false; 2-D Lookup Table Flat on -pitch: the steeper breakpoint); w_target = sat(+/-min(' ...
    'k_der*max(w_max_man, w_max_auto), a_lat_max/max(|v_ref|, v_lat_min))). Shaper per axis (subsystems Shaper_v / ' ...
    'Shaper_w): profile [a_acc a_dec jerk K a_stop] by mode (1 MANUAL/HOLD, 2 AUTO_START/AUTO with a_acc capped by ' ...
    'gnc accel_limit > 0 and the D41 yaw profile, 3 CTRL_STOP, 4 ESTOP); a_dem = sat(K*(x_t - x_ref), -a_dn, a_up) ' ...
    'direction-aware, stop profiles a_dem = -sign(x_ref)*min(a_stop, sqrt(2*jerk*|x_ref|), |x_ref|/Ts) (D33; the last ' ...
    'term lands x_ref on 0); Rate Limiter Dynamic +/-jerk; Discrete-Time Integrator Forward Euler with level reset to ' ...
    'the measured value (v_track, yaw_rate_meas) while drive_enable is false. Yaw-rate loop: Discrete PID Controller ' ...
    'PI (yaw.Kp/Ki, +/-u_max, clamping, level reset and output 0 while disabled, on an invalid IMU sample ' ...
    '(tilt.imu_valid false, R-05), ' ...
    'yaw.enable false or at commanded standstill); w_mix = w_ref + u. Commanded standstill (subsystem Cmd_still, ' ...
    '2026-11-14): v_target = w_target = 0 AND |v_ref| < yaw.cmd_still_v AND |w_ref| < yaw.cmd_still_w, OR ' ...
    'drive_enable false. The loop (and the yaw shaper reset and traction) use the yaw rate of ' ...
    'subsystem Yaw_meas: gyro_z = tilt.yaw_rate_meas + tilt.gyro_bias re-biased by a first-order average (pole ' ...
    'exp(-Ts/yaw.tau_bias)) enabled only while BOTH |vTrack_L,R| < yaw.standstill_v AND the commanded standstill ' ...
    'of the previous step (Unit Delay: w_ref follows this rate through the Shaper_w reset), so neither a pivot ' ...
    'turn (mean track speed 0) nor a slow pivot under the standstill gate enters the bias. Skid mixer v_L,R = ' ...
    'v_ref -/+ w_mix*chi*B/2, a_L,R = a_ref -/+ alpha_ref*chi*B/2, ' ...
    'both divided by k = max(1, max|v_L,R|/v_track_max). Traction: s = (v_track - v_body)*sign(v_track)/max(|v_track|, ' ...
    'v_min), v_body = v_ground_est -/+ yaw_rate*chi*B/2, Relay s_on/s_off -> k_min/1 while v_ground_valid, Rate Limiter ' ...
    '+k_up/-k_dn. Parameters P.electronics.{lim, shaper, yaw, mix, drv, tlim, trac}. Built by ' ...
    'scripts/build/build_fw_control.m.']);
end

function src = local_descent_cap(s, kinds)
% descent speed cap blocks inside the subsystem s (Agents task descent, 2026-11-15), whose In ports pitch (rad, + nose
% down), imu_valid, di_word, impl_type, impl_rx_ok and lift_pos (adc.lift_pos, 0..1) exist; kinds lists the tables
% P.electronics.lim.descent.<kind>_fwd / <kind>_rev (rows implement code 0..3, 4 = unknown, 5 = the mower with its deck
% reported raised; columns theta_bp) to look up on the one pitch window and row:
% {'ref'} in fw_vehicle_ctrl/Descent_cap, {'trip', 'ref'} in fw_safety_supervisor/Descent_trip. Returns src.<kind> =
% {forward, reverse} sources 'Cap_<kind>_f/1', 'Cap_<kind>_r/1'.
%   downhill pitch in the direction of travel th = [+pitch -pitch] (deg), theta_imu in both while the IMU sample is
%   invalid, each the largest of the last round(t_hold/Ts.ctrl) samples (Tapped Delay, vinit theta_imu, + MinMax);
%   column = the first breakpoint at or above th (the steeper one): 2-D Lookup Table, Flat, on -th against
%   -fliplr(theta_bp), Clip (above the last breakpoint the last, at or below 0 the first); row = the coding plug DI
%   IMPL_ID1:IMPL_ID0 (VC_IMPL, as fw_implement_aux), 5 for the mower with lift_pos >= descent.lift_raised (the deck
%   reported raised), 4 (the smallest entries of all rows) when IMPL_FB is received (impl_rx_ok) with another impl_type
CT = 'Ts.ctrl'; D = 'P.electronics.lim.descent';
A(s, 'simulink/Math Operations/Gain', 'Pitch_deg', 'Gain', '180/pi');
A(s, 'simulink/Math Operations/Gain', 'Pitch_deg_rev', 'Gain', '-180/pi');
L(s, 'pitch/1', 'Pitch_deg/1', 'pitch'); L(s, 'pitch/1', 'Pitch_deg_rev/1');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'Th_raw', 'NumInputs', '2');
L(s, 'Pitch_deg/1', 'Th_raw/1', 'th_fwd'); L(s, 'Pitch_deg_rev/1', 'Th_raw/2', 'th_rev');
CST(s, 'Th_imu', [D '.theta_imu*[1 1]'], 'single', CT);
A(s, 'simulink/Signal Routing/Switch', 'Th_valid', 'Criteria', 'u2 ~= 0');
L(s, 'Th_raw/1', 'Th_valid/1', 'th_raw'); L(s, 'imu_valid/1', 'Th_valid/2', 'imu_valid');
L(s, 'Th_imu/1', 'Th_valid/3', 'th_imu');
A(s, 'simulink/Signal Routing/Demux', 'Th_dmx', 'Outputs', '2'); L(s, 'Th_valid/1', 'Th_dmx/1', 'th');
% the implement row
A(s, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Impl_id_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^P.electronics.di.impl_id0 + 2^P.electronics.di.impl_id1', 'BitMaskRealWorld', 'Stored Integer');
A(s, 'simulink/Logic and Bit Operations/Shift Arithmetic', 'Impl_code', 'BitShiftNumber', 'P.electronics.di.impl_id0', ...
    'BitShiftDirection', 'Right');
L(s, 'di_word/1', 'Impl_id_mask/1', 'di_word'); L(s, 'Impl_id_mask/1', 'Impl_code/1', 'impl_id');
DTC(s, 'Impl_code_u8', 'uint8'); L(s, 'Impl_code/1', 'Impl_code_u8/1', 'impl_code');
A(s, 'simulink/Logic and Bit Operations/Relational Operator', 'Impl_other', 'Operator', '~=');
L(s, 'impl_type/1', 'Impl_other/1', 'impl_type'); L(s, 'Impl_code_u8/1', 'Impl_other/2', 'impl_code_u8');
LOGIC(s, 'Impl_unknown', 'AND', 2);
L(s, 'Impl_other/1', 'Impl_unknown/1', 'impl_other'); L(s, 'impl_rx_ok/1', 'Impl_unknown/2', 'impl_rx_ok');
DTC(s, 'Impl_code_s', 'single'); L(s, 'Impl_code/1', 'Impl_code_s/1');
% the mower with its deck reported raised (lift potentiometer at or above descent.lift_raised): row 5, the *_raised rows
% (the user, 2026-11-15: a backstop; the GNC keeps the deck down whenever the robot moves)
CMP(s, 'Is_mower', '==', 'double(ImplementType.MOWER)'); L(s, 'Impl_code_u8/1', 'Is_mower/1');
CMP(s, 'Deck_up', '>=', [D '.lift_raised']); L(s, 'lift_pos/1', 'Deck_up/1', 'lift_pos');
LOGIC(s, 'Mower_raised', 'AND', 2); L(s, 'Is_mower/1', 'Mower_raised/1', 'is_mower'); L(s, 'Deck_up/1', 'Mower_raised/2', 'deck_up');
CST(s, 'Row_raised_c', '5', 'single', CT);
A(s, 'simulink/Signal Routing/Switch', 'Row_raised', 'Criteria', 'u2 ~= 0');
L(s, 'Row_raised_c/1', 'Row_raised/1', 'row_raised_c'); L(s, 'Mower_raised/1', 'Row_raised/2', 'mower_raised');
L(s, 'Impl_code_s/1', 'Row_raised/3', 'impl_code_s');
CST(s, 'Row_unknown', '4', 'single', CT);
A(s, 'simulink/Signal Routing/Switch', 'Row', 'Criteria', 'u2 ~= 0');
L(s, 'Row_unknown/1', 'Row/1', 'row_unknown'); L(s, 'Impl_unknown/1', 'Row/2', 'impl_unknown');
L(s, 'Row_raised/1', 'Row/3', 'row_known');
% per direction: peak hold, then the tables
src = struct();
for q = {'f', 1, 'fwd'; 'r', 2, 'rev'}'
    x = q{1};
    A(s, 'simulink/Discrete/Tapped Delay', ['Th_' x '_taps'], 'NumDelays', ['round(' D '.t_hold/Ts.ctrl) - 1'], ...
        'includeCurrent', 'on', 'vinit', [D '.theta_imu'], 'samptime', '-1');
    A(s, 'simulink/Math Operations/MinMax', ['Th_' x '_peak'], 'Function', 'max', 'Inputs', '1');
    A(s, 'simulink/Math Operations/Gain', ['Th_' x '_neg'], 'Gain', '-1');
    L(s, sprintf('Th_dmx/%d', q{2}), ['Th_' x '_taps/1'], ['th_' x]);
    L(s, ['Th_' x '_taps/1'], ['Th_' x '_peak/1'], ['th_' x '_taps']);
    L(s, ['Th_' x '_peak/1'], ['Th_' x '_neg/1'], ['th_' x '_peak']);
    for k = 1:numel(kinds)
        b = ['Cap_' kinds{k} '_' x];
        A(s, 'simulink/Lookup Tables/2-D Lookup Table', b, 'BreakpointsForDimension1', '0:5', ...
            'BreakpointsForDimension2', ['-fliplr(' D '.theta_bp)'], 'Table', [D '.' kinds{k} '_' q{3} '(:, end:-1:1)'], ...
            'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip', 'OutDataTypeStr', 'single', 'TableDataTypeStr', 'single', ...
            'BreakpointsForDimension1DataTypeStr', 'single', 'BreakpointsForDimension2DataTypeStr', 'single');
        L(s, 'Row/1', [b '/1'], 'row'); L(s, ['Th_' x '_neg/1'], [b '/2'], ['th_' x '_neg']);
        src.(kinds{k}){q{2}} = [b '/1'];
    end
end
end

function local_descent_trip(n)
% subsystem Descent_trip of fw_safety_supervisor (Agents task descent, 2026-11-15): in 1 pitch, 2 imu_valid, 3 di_word,
% 4 impl_type, 5 impl_rx_ok, 6 lift_pos (adc.lift_pos), 7 v_LR (odom.vTrack_LR), 8 a_track, 9 odom_valid, 10 mode_prev,
% 11 vref_LR (vtrack_ref_prev); out 1 cap_trip (boolean) =
%   drive on (mode_prev MANUAL..ESTOP) AND odom_valid AND |v| + max(d*a_track, 0)*t_pred > trip_d AND (the reference is
%   steady or rising OR |v| - d*r > v_follow_dv), v = the mean of v_LR (the body speed), d = sign(v), r = the mean of
%   vref_LR, trip_d = the forward (v > 0) or the reverse trip speed of local_descent_cap; the reference falls (a
%   commanded deceleration) when d*r is more than a_fall*Ts.ctrl below d*r of the sample before, or more than dv_fall
%   above the speed target of the cap ref_d (fw_vehicle_ctrl's limit, the same tables, so the cap pulls it down: in the
%   sample the cap drops the reference of the previous sample has not started to fall). A runaway past the cap (the drive
%   cannot brake: CCL 0) trips at the cap; a regen deceleration from above a cap that has just dropped (a crest, a
%   steeper slope) follows its falling reference and does not trip.
%   Hill start (descent task, step 9, 2026-11-16; structure step 8 point 2, the S2 climb replay): in 12 brake_rel_L, 13
%   brake_rel_R (drive_fb brake_released); the prediction term is 0 while a brake is engaged or both have been released
%   for less than round(descent.t_release/Ts.ctrl) samples (Rel_age), AND d*r <= yaw.cmd_still_v (the reference held
%   at standstill, or against the motion: a stick pushed uphill during the roll-back). There the drive pushes against a
%   motion from rest: a held roll-back stays far below the cap, and a real runaway still trips on the measured speed
CT = 'Ts.ctrl'; D = 'P.electronics.lim.descent';
c = SUB(n, 'Descent_trip', {'pitch', 'imu_valid', 'di_word', 'impl_type', 'impl_rx_ok', 'lift_pos', 'v_LR', ...
    'a_track', 'odom_valid', 'mode_prev', 'vref_LR', 'brake_rel_L', 'brake_rel_R'}, {'cap_trip'});
src = local_descent_cap(c, {'trip', 'ref'});
cf = src.trip{1}; cr = src.trip{2};
% body speed and reference (means of the two tracks), the direction of travel
A(c, 'simulink/Math Operations/Sum', 'V_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(c, 'simulink/Math Operations/Gain', 'V_m', 'Gain', '0.5');
L(c, 'v_LR/1', 'V_sum/1', 'v_LR'); L(c, 'V_sum/1', 'V_m/1', 'v_sum');
A(c, 'simulink/Math Operations/Sum', 'R_sum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
A(c, 'simulink/Math Operations/Gain', 'R_m', 'Gain', '0.5');
L(c, 'vref_LR/1', 'R_sum/1', 'vref_LR'); L(c, 'R_sum/1', 'R_m/1', 'r_sum');
A(c, 'simulink/Math Operations/Sign', 'Dir'); L(c, 'V_m/1', 'Dir/1', 'v_m');
A(c, 'simulink/Math Operations/Abs', 'V_abs'); L(c, 'V_m/1', 'V_abs/1');
A(c, 'simulink/Logic and Bit Operations/Compare To Zero', 'Fwd', 'relop', '>', 'OutDataTypeStr', 'boolean');
L(c, 'V_m/1', 'Fwd/1');
A(c, 'simulink/Signal Routing/Switch', 'Cap_dir', 'Criteria', 'u2 ~= 0');
L(c, cf, 'Cap_dir/1', 'trip_fwd'); L(c, 'Fwd/1', 'Cap_dir/2', 'fwd'); L(c, cr, 'Cap_dir/3', 'trip_rev');
% the reference in the direction of travel, now and one sample before: falling by more than a_fall*Ts
A(c, 'simulink/Math Operations/Product', 'Ref_d', 'Inputs', '**');
L(c, 'Dir/1', 'Ref_d/1', 'dir'); L(c, 'R_m/1', 'Ref_d/2', 'r_m');
A(c, 'simulink/Discrete/Unit Delay', 'R_m_prev', 'InitialCondition', '0', 'SampleTime', '-1');
L(c, 'R_m/1', 'R_m_prev/1');
A(c, 'simulink/Math Operations/Product', 'Ref_d_prev', 'Inputs', '**');
L(c, 'Dir/1', 'Ref_d_prev/1'); L(c, 'R_m_prev/1', 'Ref_d_prev/2', 'r_m_prev');
CST(c, 'Eps_fall', [D '.a_fall*Ts.ctrl'], 'single', CT);
A(c, 'simulink/Math Operations/Sum', 'Fall_lim', 'Inputs', '+-');
L(c, 'Ref_d_prev/1', 'Fall_lim/1', 'ref_prev'); L(c, 'Eps_fall/1', 'Fall_lim/2', 'eps_fall');
A(c, 'simulink/Logic and Bit Operations/Relational Operator', 'Falling', 'Operator', '<');
L(c, 'Ref_d/1', 'Falling/1', 'ref'); L(c, 'Fall_lim/1', 'Falling/2', 'fall_lim');
% ... or the reference above the cap's speed target (the vehicle control pulls it down)
A(c, 'simulink/Signal Routing/Switch', 'Ref_cap_dir', 'Criteria', 'u2 ~= 0');
L(c, src.ref{1}, 'Ref_cap_dir/1', 'ref_cap_fwd'); L(c, 'Fwd/1', 'Ref_cap_dir/2'); L(c, src.ref{2}, 'Ref_cap_dir/3', 'ref_cap_rev');
A(c, 'simulink/Math Operations/Sum', 'Ref_above', 'Inputs', '+-');
L(c, 'Ref_d/1', 'Ref_above/1'); L(c, 'Ref_cap_dir/1', 'Ref_above/2', 'ref_cap');
CMP(c, 'Cap_pulls', '>', [D '.dv_fall']); L(c, 'Ref_above/1', 'Cap_pulls/1', 'ref_above');
LOGIC(c, 'Decel', 'OR', 2); L(c, 'Falling/1', 'Decel/1', 'falling'); L(c, 'Cap_pulls/1', 'Decel/2', 'cap_pulls');
LOGIC(c, 'Steady', 'NOT', 1); L(c, 'Decel/1', 'Steady/1', 'decel');
A(c, 'simulink/Math Operations/Sum', 'Lag', 'Inputs', '+-'); L(c, 'V_abs/1', 'Lag/1', 'v'); L(c, 'Ref_d/1', 'Lag/2');
CMP(c, 'Lag_big', '>', 'P.electronics.lim.v_follow_dv'); L(c, 'Lag/1', 'Lag_big/1', 'lag');
LOGIC(c, 'No_follow', 'OR', 2); L(c, 'Steady/1', 'No_follow/1', 'steady'); L(c, 'Lag_big/1', 'No_follow/2', 'lag_big');
% the predicted speed past the trip speed
A(c, 'simulink/Math Operations/Product', 'A_d', 'Inputs', '**'); L(c, 'Dir/1', 'A_d/1'); L(c, 'a_track/1', 'A_d/2', 'a_track');
CST(c, 'Zero', '0', 'single', CT);
A(c, 'simulink/Math Operations/MinMax', 'A_pos', 'Function', 'max', 'Inputs', '2');
L(c, 'A_d/1', 'A_pos/1', 'a_dir'); L(c, 'Zero/1', 'A_pos/2', 'zero');
A(c, 'simulink/Math Operations/Gain', 'Dv_pred', 'Gain', [D '.t_pred']); L(c, 'A_pos/1', 'Dv_pred/1', 'a_pos');
% ... without the prediction at a hill start (a brake engaged, or both released less than t_release ago, and the
% reference in the direction of travel at standstill or against the motion)
LOGIC(c, 'Released', 'AND', 2);
L(c, 'brake_rel_L/1', 'Released/1', 'brake_rel_L'); L(c, 'brake_rel_R/1', 'Released/2', 'brake_rel_R');
LOGIC(c, 'Braked', 'NOT', 1); L(c, 'Released/1', 'Braked/1', 'released');
AGE(c, 'Rel_age', ['round(' D '.t_release/Ts.ctrl)'], '0'); L(c, 'Braked/1', 'Rel_age/1', 'braked');
CMP(c, 'Rel_recent', '<', ['round(' D '.t_release/Ts.ctrl)']); L(c, 'Rel_age/1', 'Rel_recent/1', 'rel_age');
CMP(c, 'Ref_still', '<=', 'P.electronics.yaw.cmd_still_v'); L(c, 'Ref_d/1', 'Ref_still/1');
LOGIC(c, 'Hill_start', 'AND', 2);
L(c, 'Rel_recent/1', 'Hill_start/1', 'rel_recent'); L(c, 'Ref_still/1', 'Hill_start/2', 'ref_still');
A(c, 'simulink/Signal Routing/Switch', 'Dv_used', 'Criteria', 'u2 ~= 0');
L(c, 'Zero/1', 'Dv_used/1'); L(c, 'Hill_start/1', 'Dv_used/2', 'hill_start'); L(c, 'Dv_pred/1', 'Dv_used/3', 'dv_pred');
A(c, 'simulink/Math Operations/Sum', 'V_pred', 'Inputs', '++'); L(c, 'V_abs/1', 'V_pred/1'); L(c, 'Dv_used/1', 'V_pred/2', 'dv_used');
A(c, 'simulink/Logic and Bit Operations/Relational Operator', 'Past_cap', 'Operator', '>');
L(c, 'V_pred/1', 'Past_cap/1', 'v_pred'); L(c, 'Cap_dir/1', 'Past_cap/2', 'trip_dir');
MODESET(c, 'Drive_on', 'mode_prev/1', 'MANUAL', 'ESTOP');      % MANUAL HOLD AUTO_START AUTO CTRL_STOP ESTOP
LOGIC(c, 'Trip', 'AND', 4);
L(c, 'Past_cap/1', 'Trip/1', 'past_cap'); L(c, 'No_follow/1', 'Trip/2', 'no_follow'); L(c, 'Drive_on/1', 'Trip/3', 'drive_on');
L(c, 'odom_valid/1', 'Trip/4', 'odom_valid');
L(c, 'Trip/1', 'cap_trip/1', 'cap_trip');
end

function local_shaper(sys, name)
% reference shaper of one axis: in x_t, x_meas, en, prof [a_acc a_dec jerk K a_stop], is_stop; out x_ref, a_ref
s = SUB(sys, name, {'x_t', 'x_meas', 'en', 'prof', 'is_stop'}, {'x_ref', 'a_ref'});
A(s, 'simulink/Signal Routing/Demux', 'Prof', 'Outputs', '5'); L(s, 'prof/1', 'Prof/1', 'prof');
A(s, 'simulink/Discrete/Discrete-Time Integrator', 'X_ref', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1.0', 'ExternalReset', 'level', 'InitialConditionSource', 'external', 'SampleTime', '-1');
LOGIC(s, 'Reset', 'NOT', 1); L(s, 'en/1', 'Reset/1', 'en');
L(s, 'Reset/1', 'X_ref/2', 'track_meas'); L(s, 'x_meas/1', 'X_ref/3', 'x_meas');
% tracking law (direction-aware limits)
A(s, 'simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
L(s, 'x_t/1', 'Err/1', 'x_t'); L(s, 'X_ref/1', 'Err/2', 'x_ref');
A(s, 'simulink/Math Operations/Product', 'K_err', 'Inputs', '**'); L(s, 'Prof/4', 'K_err/1', 'K'); L(s, 'Err/1', 'K_err/2', 'e');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Fwd_ge', 'relop', '>=', 'OutDataTypeStr', 'boolean');
A(s, 'simulink/Logic and Bit Operations/Compare To Zero', 'Fwd_gt', 'relop', '>', 'OutDataTypeStr', 'boolean');
L(s, 'X_ref/1', 'Fwd_ge/1'); L(s, 'X_ref/1', 'Fwd_gt/1');
A(s, 'simulink/Signal Routing/Switch', 'A_up', 'Criteria', 'u2 ~= 0');
A(s, 'simulink/Signal Routing/Switch', 'A_dn', 'Criteria', 'u2 ~= 0');
L(s, 'Prof/1', 'A_up/1', 'a_acc'); L(s, 'Fwd_ge/1', 'A_up/2'); L(s, 'Prof/2', 'A_up/3', 'a_dec');
L(s, 'Prof/2', 'A_dn/1'); L(s, 'Fwd_gt/1', 'A_dn/2'); L(s, 'Prof/1', 'A_dn/3');
A(s, 'simulink/Math Operations/Gain', 'A_dn_neg', 'Gain', '-1'); L(s, 'A_dn/1', 'A_dn_neg/1');
A(s, 'simulink/Discontinuities/Saturation Dynamic', 'A_track');
L(s, 'A_up/1', 'A_track/1', 'a_up'); L(s, 'K_err/1', 'A_track/2'); L(s, 'A_dn_neg/1', 'A_track/3');
% constant-deceleration stop law (D33): -sign(x)*min(a_stop, sqrt(2*jerk*|x|), |x|/Ts)
A(s, 'simulink/Math Operations/Abs', 'X_abs'); L(s, 'X_ref/1', 'X_abs/1');
A(s, 'simulink/Math Operations/Gain', 'Two_x', 'Gain', '2'); L(s, 'X_abs/1', 'Two_x/1', 'x_abs');
A(s, 'simulink/Math Operations/Product', 'J_x', 'Inputs', '**'); L(s, 'Two_x/1', 'J_x/1'); L(s, 'Prof/3', 'J_x/2', 'jerk');
A(s, 'simulink/Math Operations/Sqrt', 'Sqrt_jx'); L(s, 'J_x/1', 'Sqrt_jx/1');
A(s, 'simulink/Math Operations/Gain', 'Land', 'Gain', '1/Ts.ctrl'); L(s, 'X_abs/1', 'Land/1');
A(s, 'simulink/Math Operations/MinMax', 'Stop_mag', 'Function', 'min', 'Inputs', '3');
L(s, 'Prof/5', 'Stop_mag/1', 'a_stop'); L(s, 'Sqrt_jx/1', 'Stop_mag/2'); L(s, 'Land/1', 'Stop_mag/3');
A(s, 'simulink/Math Operations/Sign', 'X_sign'); L(s, 'X_ref/1', 'X_sign/1');
A(s, 'simulink/Math Operations/Product', 'Stop_dir', 'Inputs', '**'); L(s, 'X_sign/1', 'Stop_dir/1'); L(s, 'Stop_mag/1', 'Stop_dir/2');
A(s, 'simulink/Math Operations/Gain', 'A_stop', 'Gain', '-1'); L(s, 'Stop_dir/1', 'A_stop/1');
A(s, 'simulink/Signal Routing/Switch', 'A_dem', 'Criteria', 'u2 ~= 0');
L(s, 'A_stop/1', 'A_dem/1', 'a_stop_dem'); L(s, 'is_stop/1', 'A_dem/2', 'is_stop'); L(s, 'A_track/1', 'A_dem/3', 'a_track_dem');
CST(s, 'Zero', '0', 'single', 'Ts.ctrl');
A(s, 'simulink/Signal Routing/Switch', 'A_dem_en', 'Criteria', 'u2 ~= 0');
L(s, 'A_dem/1', 'A_dem_en/1', 'a_dem'); L(s, 'en/1', 'A_dem_en/2'); L(s, 'Zero/1', 'A_dem_en/3');
% jerk limit and integration
A(s, 'simulink/Math Operations/Gain', 'J_neg', 'Gain', '-1'); L(s, 'Prof/3', 'J_neg/1');
A(s, 'simulink/Discontinuities/Rate Limiter Dynamic', 'Jerk_lim');
L(s, 'Prof/3', 'Jerk_lim/1'); L(s, 'A_dem_en/1', 'Jerk_lim/2'); L(s, 'J_neg/1', 'Jerk_lim/3');
L(s, 'Jerk_lim/1', 'X_ref/1', 'a_ref');
L(s, 'X_ref/1', 'x_ref/1'); L(s, 'Jerk_lim/1', 'a_ref/1');
end

% ===========================================================================================
% fw_implement_aux (Task_50)
% ===========================================================================================
function local_fw_implement_aux(n)
CM = 'Ts.comm'; X = 'P.electronics.aux';
IN(n, 'fw50', 1, '', CM);
B = 'fw50/1';           % the bus line of the port fw50: each consumer picks its fields right in front of it (spec 6.2)
% ---- implement type from the coding plug (DI IMPL_ID0/1 = VC_IMPL), modes ----
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Id_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', '2^P.electronics.di.impl_id0 + 2^P.electronics.di.impl_id1', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Shift Arithmetic', 'Id_shift', 'BitShiftNumber', 'P.electronics.di.impl_id0', ...
    'BitShiftDirection', 'Right');
PICK(n, B, 'Sel_impl_id', {'di_word'}, {'Id_mask/1'}); L(n, 'Id_mask/1', 'Id_shift/1');
CMP(n, 'Is_mower', '==', 'double(ImplementType.MOWER)'); CMP(n, 'Is_tiller', '==', 'double(ImplementType.TILLER)');
CMP(n, 'Is_sprayer', '==', 'double(ImplementType.SPRAYER)'); CMP(n, 'Fitted', '~=', '0');
for k = {'Is_mower', 'Is_tiller', 'Is_sprayer', 'Fitted'}, L(n, 'Id_shift/1', [k{1} '/1'], 'impl_id'); end
LOGIC(n, 'Rotor_type', 'OR', 2); L(n, 'Is_mower/1', 'Rotor_type/1'); L(n, 'Is_tiller/1', 'Rotor_type/2');
md = {'Manual', 'MANUAL'; 'Auto', 'AUTO'; 'Standby', 'STANDBY'; 'Hold', 'HOLD'; 'Auto_start', 'AUTO_START'; ...
    'Ctrl_stop', 'CTRL_STOP'; 'Estop', 'ESTOP'; 'Fault', 'FAULT'; 'Selftest', 'SELFTEST'};
SELB(n, B, 'Sel_mode', {'st.mode'});                         % one pick for the ten mode compares
for k = 1:size(md, 1)
    CMP(n, ['M_' md{k, 1}], '==', ['double(ShaddadMode.' md{k, 2} ')']); L(n, 'Sel_mode/1', ['M_' md{k, 1} '/1']);
end
CMP(n, 'Not_boot', '~=', 'double(ShaddadMode.BOOT)'); L(n, 'Sel_mode/1', 'Not_boot/1');
SELB(n, B, 'Sel_fault_word', {'st.fault_word'});
BIT(n, 'Impl_fault', 'Sel_fault_word/1', '28', 'uint32');
LOGIC(n, 'Impl_ok', 'NOT', 1); L(n, 'Impl_fault/1', 'Impl_ok/1', 'impl_fault');
CMP(n, 'Sw_on', '>=', '2'); CMP(n, 'Sw_auto', '==', '3');
PICK(n, B, 'Sel_sw_on', {'st.rc.sw_impl'}, {'Sw_on/1'}); PICK(n, B, 'Sel_sw_auto', {'st.rc.sw_impl'}, {'Sw_auto/1'});
% ---- rotor (mower / tiller) -> IMPL_CMD ----
LOGIC(n, 'Gnc_mow', 'AND', 2); L(n, 'Is_mower/1', 'Gnc_mow/1');
PICK(n, B, 'Sel_mower_enable', {'gnc_cmd.mower_enable'}, {'Gnc_mow/2'});
LOGIC(n, 'Gnc_till', 'AND', 2); L(n, 'Is_tiller/1', 'Gnc_till/1');
PICK(n, B, 'Sel_tiller_enable', {'gnc_cmd.tiller_enable'}, {'Gnc_till/2'});
LOGIC(n, 'Gnc_rotor', 'OR', 2); L(n, 'Gnc_mow/1', 'Gnc_rotor/1'); L(n, 'Gnc_till/1', 'Gnc_rotor/2');
LOGIC(n, 'Auto_rotor', 'AND', 2); L(n, 'M_Auto/1', 'Auto_rotor/1'); L(n, 'Gnc_rotor/1', 'Auto_rotor/2');
LOGIC(n, 'Mode_rotor', 'OR', 2); L(n, 'M_Manual/1', 'Mode_rotor/1'); L(n, 'Auto_rotor/1', 'Mode_rotor/2');
LOGIC(n, 'Rotor_en', 'AND', 4); L(n, 'Rotor_type/1', 'Rotor_en/1'); L(n, 'Sw_on/1', 'Rotor_en/2');
L(n, 'Impl_ok/1', 'Rotor_en/3'); L(n, 'Mode_rotor/1', 'Rotor_en/4');
CST(n, 'W_mow_def', [X '.mower_w_default'], 'single', CM); CST(n, 'W_till_def', [X '.tiller_w_default'], 'single', CM);
A(n, 'simulink/Signal Routing/Switch', 'W_def', 'Criteria', 'u2 ~= 0');
L(n, 'W_mow_def/1', 'W_def/1'); L(n, 'Is_mower/1', 'W_def/2'); L(n, 'W_till_def/1', 'W_def/3');
A(n, 'simulink/Signal Routing/Switch', 'W_gnc', 'Criteria', 'u2 ~= 0');
PICK(n, B, 'Sel_mower_speed', {'gnc_cmd.mower_speed_cmd'}, {'W_gnc/1'}); L(n, 'Is_mower/1', 'W_gnc/2');
PICK(n, B, 'Sel_tiller_speed', {'gnc_cmd.tiller_speed_cmd'}, {'W_gnc/3'});
A(n, 'simulink/Signal Routing/Switch', 'W_cmd', 'Criteria', 'u2 ~= 0');
L(n, 'W_gnc/1', 'W_cmd/1'); L(n, 'M_Auto/1', 'W_cmd/2'); L(n, 'W_def/1', 'W_cmd/3');
CST(n, 'Zero', '0', 'single', CM);
A(n, 'simulink/Signal Routing/Switch', 'W_on', 'Criteria', 'u2 ~= 0');
L(n, 'W_cmd/1', 'W_on/1', 'w_impl'); L(n, 'Rotor_en/1', 'W_on/2', 'rotor_en'); L(n, 'Zero/1', 'W_on/3');
A(n, 'simulink/Math Operations/Gain', 'To_rpm', 'Gain', '30/pi'); L(n, 'W_on/1', 'To_rpm/1');
SELB(n, B, 'Sel_reset_btn', {'di_word'});
BIT(n, 'Di_reset', 'Sel_reset_btn/1', 'P.electronics.di.reset_btn', 'uint16');
LOGIC(n, 'Impl_reset', 'OR', 2); PICK(n, B, 'Sel_rc_reset', {'st.rc.reset_req'}, {'Impl_reset/1'});
L(n, 'Di_reset/1', 'Impl_reset/2');
% ---- sprayer: nozzles (DO b10..b17) and pump duty ----
A(n, 'simulink/Math Operations/Abs', 'V_abs'); PICK(n, B, 'Sel_v_track', {'st.v_track'}, {'V_abs/1'});
CMP(n, 'Moving_spray', '>', [X '.spray_v_min']); L(n, 'V_abs/1', 'Moving_spray/1');
CMP(n, 'Moving', '>', [X '.moving_v']); L(n, 'V_abs/1', 'Moving/1');
LOGIC(n, 'Man_spray', 'AND', 2); L(n, 'M_Manual/1', 'Man_spray/1'); L(n, 'Sw_on/1', 'Man_spray/2');
LOGIC(n, 'Auto_spray', 'AND', 3); L(n, 'M_Auto/1', 'Auto_spray/1'); L(n, 'Sw_auto/1', 'Auto_spray/2');
PICK(n, B, 'Sel_spray_enable', {'gnc_cmd.spray_enable'}, {'Auto_spray/3'});
LOGIC(n, 'Spray_mode', 'OR', 2); L(n, 'Man_spray/1', 'Spray_mode/1'); L(n, 'Auto_spray/1', 'Spray_mode/2');
LOGIC(n, 'Spray_on', 'AND', 3); L(n, 'Is_sprayer/1', 'Spray_on/1'); L(n, 'Moving_spray/1', 'Spray_on/2'); L(n, 'Spray_mode/1', 'Spray_on/3');
CST(n, 'Mask_all', '255', 'uint16', CM); CST(n, 'Mask_none', '0', 'uint16', CM);
A(n, 'simulink/Signal Routing/Switch', 'Mask_src', 'Criteria', 'u2 ~= 0');
PICK(n, B, 'Sel_section_mask', {'gnc_cmd.spray_section_mask'}, {'Mask_src/1'});
L(n, 'M_Auto/1', 'Mask_src/2'); L(n, 'Mask_all/1', 'Mask_src/3');
A(n, 'simulink/Signal Routing/Switch', 'Mask_on', 'Criteria', 'u2 ~= 0');
L(n, 'Mask_src/1', 'Mask_on/1'); L(n, 'Spray_on/1', 'Mask_on/2', 'spray_on'); L(n, 'Mask_none/1', 'Mask_on/3');
A(n, 'commutil2/Integer to Bit Converter', 'Mask_bits', 'nbits', '16', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
A(n, 'simulink/Signal Routing/Selector', 'Nozzles', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1:8', 'InputPortWidth', '16');
L(n, 'Mask_on/1', 'Mask_bits/1', 'nozzle_mask'); L(n, 'Mask_bits/1', 'Nozzles/1');
A(n, 'simulink/Math Operations/Gain', 'Flow_duty', 'Gain', ['1/' X '.pump_flow_full']);
A(n, 'simulink/Discontinuities/Saturation', 'Flow_sat', 'UpperLimit', '1', 'LowerLimit', '0');
PICK(n, B, 'Sel_flow_cmd', {'gnc_cmd.spray_flow_cmd'}, {'Flow_duty/1'}); L(n, 'Flow_duty/1', 'Flow_sat/1');
CST(n, 'Pump_man', [X '.pump_duty_man'], 'single', CM);
A(n, 'simulink/Signal Routing/Switch', 'Pump_src', 'Criteria', 'u2 ~= 0');
L(n, 'Flow_sat/1', 'Pump_src/1'); L(n, 'M_Auto/1', 'Pump_src/2'); L(n, 'Pump_man/1', 'Pump_src/3');
A(n, 'simulink/Signal Routing/Switch', 'Pump', 'Criteria', 'u2 ~= 0');
L(n, 'Pump_src/1', 'Pump/1'); L(n, 'Spray_on/1', 'Pump/2'); L(n, 'Zero/1', 'Pump/3');
% ---- lift (PWM 2 magnitude, DO b18 LIFT_DIR = down) ----
% MANUAL: CH11 UP (+) / HOLD / DOWN (-) at lift_duty_max; AUTO: duty = sat(lift_Kp*(h_cmd - h_meas)) with
% h_meas from the lift potentiometer (mower: deck height h_lo + x*(h_hi - h_lo); tiller: -d_max*(1 - x)),
% h_cmd = deck_height_cmd / -tiller_depth_cmd (companion), *_default without a fresh companion command
CMP(n, 'Lift_up', '==', '1'); CMP(n, 'Lift_down', '==', '3');
PICK(n, B, 'Sel_lift_up', {'st.rc.sw_lift'}, {'Lift_up/1'}); PICK(n, B, 'Sel_lift_down', {'st.rc.sw_lift'}, {'Lift_down/1'});
DTC(n, 'Up_s', 'single'); DTC(n, 'Down_s', 'single');
L(n, 'Lift_up/1', 'Up_s/1'); L(n, 'Lift_down/1', 'Down_s/1');
A(n, 'simulink/Math Operations/Sum', 'Lift_dir_man', 'Inputs', '+-'); L(n, 'Up_s/1', 'Lift_dir_man/1'); L(n, 'Down_s/1', 'Lift_dir_man/2');
A(n, 'simulink/Math Operations/Gain', 'Duty_man', 'Gain', [X '.lift_duty_max']); L(n, 'Lift_dir_man/1', 'Duty_man/1');
A(n, 'simulink/Math Operations/Gain', 'H_mow', 'Gain', [X '.lift_h_range(2) - ' X '.lift_h_range(1)']);
CST(n, 'H_mow0', [X '.lift_h_range(1)'], 'single', CM);
A(n, 'simulink/Math Operations/Sum', 'H_meas_mow', 'Inputs', '++');
PICK(n, B, 'Sel_lift_pos_mow', {'adc.lift_pos'}, {'H_mow/1'}); L(n, 'H_mow/1', 'H_meas_mow/1'); L(n, 'H_mow0/1', 'H_meas_mow/2');
CST(n, 'One', '1', 'single', CM);
A(n, 'simulink/Math Operations/Sum', 'X_inv', 'Inputs', '+-'); L(n, 'One/1', 'X_inv/1');
PICK(n, B, 'Sel_lift_pos_till', {'adc.lift_pos'}, {'X_inv/2'});
A(n, 'simulink/Math Operations/Gain', 'H_meas_till', 'Gain', ['-' X '.till_d_max']); L(n, 'X_inv/1', 'H_meas_till/1');
A(n, 'simulink/Signal Routing/Switch', 'H_meas', 'Criteria', 'u2 ~= 0');
L(n, 'H_meas_mow/1', 'H_meas/1'); L(n, 'Is_mower/1', 'H_meas/2'); L(n, 'H_meas_till/1', 'H_meas/3');
A(n, 'simulink/Math Operations/Gain', 'H_cmd_till_gnc', 'Gain', '-1');
PICK(n, B, 'Sel_depth_cmd', {'gnc_cmd.tiller_depth_cmd'}, {'H_cmd_till_gnc/1'});
CST(n, 'H_mow_def', [X '.deck_h_default'], 'single', CM); CST(n, 'H_till_def', ['-' X '.till_d_default'], 'single', CM);
A(n, 'simulink/Signal Routing/Switch', 'H_gnc', 'Criteria', 'u2 ~= 0');
PICK(n, B, 'Sel_deck_cmd', {'gnc_cmd.deck_height_cmd'}, {'H_gnc/1'}); L(n, 'Is_mower/1', 'H_gnc/2');
L(n, 'H_cmd_till_gnc/1', 'H_gnc/3');
A(n, 'simulink/Signal Routing/Switch', 'H_def', 'Criteria', 'u2 ~= 0');
L(n, 'H_mow_def/1', 'H_def/1'); L(n, 'Is_mower/1', 'H_def/2'); L(n, 'H_till_def/1', 'H_def/3');
A(n, 'simulink/Signal Routing/Switch', 'H_cmd', 'Criteria', 'u2 ~= 0');
L(n, 'H_gnc/1', 'H_cmd/1'); PICK(n, B, 'Sel_comp_ok', {'comp_ok'}, {'H_cmd/2'}); L(n, 'H_def/1', 'H_cmd/3');
A(n, 'simulink/Math Operations/Sum', 'H_err', 'Inputs', '+-'); L(n, 'H_cmd/1', 'H_err/1', 'h_cmd'); L(n, 'H_meas/1', 'H_err/2', 'h_meas');
A(n, 'simulink/Math Operations/Gain', 'H_Kp', 'Gain', [X '.lift_Kp']); L(n, 'H_err/1', 'H_Kp/1');
A(n, 'simulink/Discontinuities/Saturation', 'Duty_auto', 'UpperLimit', [X '.lift_duty_max'], 'LowerLimit', ['-' X '.lift_duty_max']);
L(n, 'H_Kp/1', 'Duty_auto/1');
A(n, 'simulink/Signal Routing/Switch', 'Duty_a0', 'Criteria', 'u2 ~= 0');
L(n, 'Duty_auto/1', 'Duty_a0/1'); L(n, 'M_Auto/1', 'Duty_a0/2'); L(n, 'Zero/1', 'Duty_a0/3');
A(n, 'simulink/Signal Routing/Switch', 'Duty_mode', 'Criteria', 'u2 ~= 0');
L(n, 'Duty_man/1', 'Duty_mode/1'); L(n, 'M_Manual/1', 'Duty_mode/2'); L(n, 'Duty_a0/1', 'Duty_mode/3');
A(n, 'simulink/Signal Routing/Switch', 'Duty_fit', 'Criteria', 'u2 ~= 0');
L(n, 'Duty_mode/1', 'Duty_fit/1', 'lift_duty_req'); L(n, 'Rotor_type/1', 'Duty_fit/2'); L(n, 'Zero/1', 'Duty_fit/3');
% end stops: no UP at lift_pos >= liftpot_lim(2), no DOWN at lift_pos <= liftpot_lim(1)
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Dir_up', 'relop', '>', 'OutDataTypeStr', 'boolean');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Dir_down', 'relop', '<', 'OutDataTypeStr', 'boolean');
L(n, 'Duty_fit/1', 'Dir_up/1'); L(n, 'Duty_fit/1', 'Dir_down/1');
CMP(n, 'At_top', '>=', 'P.electronics.adc.liftpot_lim(2)'); CMP(n, 'At_bottom', '<=', 'P.electronics.adc.liftpot_lim(1)');
PICK(n, B, 'Sel_lift_pos_top', {'adc.lift_pos'}, {'At_top/1'}); PICK(n, B, 'Sel_lift_pos_bottom', {'adc.lift_pos'}, {'At_bottom/1'});
LOGIC(n, 'Stop_up', 'AND', 2); L(n, 'Dir_up/1', 'Stop_up/1'); L(n, 'At_top/1', 'Stop_up/2');
LOGIC(n, 'Stop_down', 'AND', 2); L(n, 'Dir_down/1', 'Stop_down/1'); L(n, 'At_bottom/1', 'Stop_down/2');
LOGIC(n, 'End_stop', 'OR', 2); L(n, 'Stop_up/1', 'End_stop/1'); L(n, 'Stop_down/1', 'End_stop/2');
A(n, 'simulink/Signal Routing/Switch', 'Duty_lift', 'Criteria', 'u2 ~= 0');
L(n, 'Zero/1', 'Duty_lift/1'); L(n, 'End_stop/1', 'Duty_lift/2', 'end_stop'); L(n, 'Duty_fit/1', 'Duty_lift/3');
A(n, 'simulink/Math Operations/Abs', 'Lift_pwm'); L(n, 'Duty_lift/1', 'Lift_pwm/1', 'lift_duty');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Lift_dir', 'relop', '<', 'OutDataTypeStr', 'boolean');
L(n, 'Duty_lift/1', 'Lift_dir/1');
% ---- fans (enclosure NTC) ----
A(n, 'simulink/Lookup Tables/1-D Lookup Table', 'Fan_duty', 'BreakpointsForDimension1', [X '.fan_bp'], ...
    'Table', [X '.fan_duty'], 'ExtrapMethod', 'Clip', 'OutDataTypeStr', 'single', 'TableDataTypeStr', 'single', ...
    'BreakpointsForDimension1DataTypeStr', 'single');
PICK(n, B, 'Sel_T_encl', {'adc.T_encl'}, {'Fan_duty/1'});
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Pwm', 'NumInputs', '4');
L(n, 'Pump/1', 'Pwm/1', 'pump_duty'); L(n, 'Lift_pwm/1', 'Pwm/2'); L(n, 'Fan_duty/1', 'Pwm/3', 'fan_duty'); L(n, 'Fan_duty/1', 'Pwm/4');
% ---- beacon / buzzer timing (counters at Ts.comm) ----
A(n, 'simulink/Sources/Counter Limited', 'Blink_slow', 'uplimit', ['round(1/(' X '.blink_f*Ts.comm)) - 1'], 'tsamp', CM);
A(n, 'simulink/Sources/Counter Limited', 'Blink_fast', 'uplimit', ['round(1/(' X '.blink_fast*Ts.comm)) - 1'], 'tsamp', CM);
A(n, 'simulink/Sources/Counter Limited', 'Beep_cnt', 'uplimit', ['round(' X '.buzz_period/Ts.comm) - 1'], 'tsamp', CM);
CMP(n, 'On_slow', '<', ['round(1/(' X '.blink_f*Ts.comm))/2']); CMP(n, 'On_fast', '<', ['round(1/(' X '.blink_fast*Ts.comm))/2']);
CMP(n, 'Beep_on', '<', ['round(' X '.buzz_on/Ts.comm)']);
L(n, 'Blink_slow/1', 'On_slow/1'); L(n, 'Blink_fast/1', 'On_fast/1'); L(n, 'Beep_cnt/1', 'Beep_on/1');
% green: AUTO steady, AUTO_START 2 Hz; amber: MANUAL / HOLD steady, STANDBY 1 Hz, CTRL_STOP 2 Hz;
% red: ESTOP steady, FAULT 2 Hz; SELFTEST: all three (lamp test)
LOGIC(n, 'G_blink', 'AND', 2); L(n, 'M_Auto_start/1', 'G_blink/1'); L(n, 'On_fast/1', 'G_blink/2');
LOGIC(n, 'Green', 'OR', 3); L(n, 'M_Auto/1', 'Green/1'); L(n, 'G_blink/1', 'Green/2'); L(n, 'M_Selftest/1', 'Green/3');
LOGIC(n, 'A_sb', 'AND', 2); L(n, 'M_Standby/1', 'A_sb/1'); L(n, 'On_slow/1', 'A_sb/2');
LOGIC(n, 'A_cs', 'AND', 2); L(n, 'M_Ctrl_stop/1', 'A_cs/1'); L(n, 'On_fast/1', 'A_cs/2');
LOGIC(n, 'Amber', 'OR', 5); L(n, 'M_Manual/1', 'Amber/1'); L(n, 'M_Hold/1', 'Amber/2'); L(n, 'A_sb/1', 'Amber/3');
L(n, 'A_cs/1', 'Amber/4'); L(n, 'M_Selftest/1', 'Amber/5');
LOGIC(n, 'R_blink', 'AND', 2); L(n, 'M_Fault/1', 'R_blink/1'); L(n, 'On_fast/1', 'R_blink/2');
LOGIC(n, 'Red', 'OR', 3); L(n, 'M_Estop/1', 'Red/1'); L(n, 'R_blink/1', 'Red/2'); L(n, 'M_Selftest/1', 'Red/3');
% buzzer: AUTO_START continuous; AUTO while moving 0.1 s every 2 s; GNC buzzer_cmd in AUTO
LOGIC(n, 'Beep_auto', 'AND', 3); L(n, 'M_Auto/1', 'Beep_auto/1'); L(n, 'Moving/1', 'Beep_auto/2'); L(n, 'Beep_on/1', 'Beep_auto/3');
LOGIC(n, 'Gnc_buzz', 'AND', 2); L(n, 'M_Auto/1', 'Gnc_buzz/1');
PICK(n, B, 'Sel_buzzer_cmd', {'gnc_cmd.buzzer_cmd'}, {'Gnc_buzz/2'});
LOGIC(n, 'Buzzer', 'OR', 3); L(n, 'M_Auto_start/1', 'Buzzer/1'); L(n, 'Beep_auto/1', 'Buzzer/2'); L(n, 'Gnc_buzz/1', 'Buzzer/3');
% K3 (IMPL24_EN), power enables after BOOT
LOGIC(n, 'K3', 'AND', 2); L(n, 'Fitted/1', 'K3/1'); L(n, 'Not_boot/1', 'K3/2');
% ---- DO word bits 2..19 (ICD 6; bits 0/1 are owned by fw_watchdog_mgr) ----
CST(n, 'B01', 'false(1,2)', 'boolean', CM); CST(n, 'B19_31', 'false(1,13)', 'boolean', CM);
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Do_bits', 'NumInputs', '12');
SELB(n, B, 'Sel_lights', {'st.rc.lights'});                 % b7 work lights, straight into Do_bits 7
srcs = {'B01/1', 'K3/1', 'Green/1', 'Amber/1', 'Red/1', 'Buzzer/1', 'Sel_lights/1', 'Not_boot/1', 'Not_boot/1', ...
    'Nozzles/1', 'Lift_dir/1', 'B19_31/1'};
for k = 1:numel(srcs), L(n, srcs{k}, sprintf('Do_bits/%d', k)); end
A(n, 'commutil2/Bit to Integer Converter', 'Do_pack', 'nbits', '32', 'bitOrder', 'LSB first', 'outDtype', 'uint32');
L(n, 'Do_bits/1', 'Do_pack/1', 'do_bits');
OUT(n, 'do_aux', 1, 'uint32'); OUT(n, 'pwm_duty', 2, 'single'); OUT(n, 'impl_enable', 3, 'boolean');
OUT(n, 'impl_fault_reset', 4, 'boolean'); OUT(n, 'impl_speed_rpm', 5, 'single');
L(n, 'Do_pack/1', 'do_aux/1', 'do_aux'); L(n, 'Pwm/1', 'pwm_duty/1', 'pwm_duty'); L(n, 'Rotor_en/1', 'impl_enable/1');
L(n, 'Impl_reset/1', 'impl_fault_reset/1', 'impl_fault_reset'); L(n, 'To_rpm/1', 'impl_speed_rpm/1', 'impl_speed_rpm');
set_param(n, 'Description', ['fw_implement_aux (T09, Task_50, spec 2.7.12, ICD 6). in 1 fw50 (Task_50 platform virtual ' ...
    'bus, Shaddad_MCU Description). out 1 do_aux uint32 (DO bits 2..19: b2 IMPL24_EN = implement fitted AND mode ' ...
    '~= BOOT, b3 green (AUTO steady, AUTO_START 2 Hz), b4 amber (MANUAL / HOLD steady, STANDBY 1 Hz, CTRL_STOP 2 Hz), ' ...
    'b5 red (ESTOP steady, FAULT 2 Hz), all three in SELFTEST, b6 buzzer (AUTO_START continuous, AUTO moving 0.1 s ' ...
    'every 2 s, GNC buzzer_cmd in AUTO), b7 work lights (CH10), b8 COMPANION_PWR_EN / b9 LIDAR_PWR_EN after BOOT, ' ...
    'b10..b17 nozzles, b18 LIFT_DIR (1 = down); commutil2 Bit to Integer 32), 2 pwm_duty single[4] [pump lift fan1 ' ...
    'fan2], 3 impl_enable, 4 impl_fault_reset (CH9 OR DI RESET_BTN), 5 impl_speed_rpm (rpm at blade / rotor). ' ...
    'Implement type = DI IMPL_ID (coding plug = VC_IMPL). Rotor: mower / tiller fitted AND sw_impl >= 2 AND NOT ' ...
    'IMPL_FAULT (bit 28) AND (MANUAL with aux.mower_w_default / tiller_w_default, or AUTO with the GNC enable and ' ...
    'speed). Sprayer: |v_track| > spray_v_min AND (MANUAL sw_impl >= 2: all 8 nozzles, pump_duty_man; AUTO sw_impl 3 ' ...
    'AND spray_enable: spray_section_mask, pump duty = spray_flow_cmd/pump_flow_full). Lift: MANUAL CH11 UP/HOLD/DOWN ' ...
    '+/-lift_duty_max; AUTO duty = sat(lift_Kp*(h_cmd - h_meas), +/-lift_duty_max), h_meas from the lift pot (mower ' ...
    'lift_h_range, tiller -till_d_max*(1 - x)), h_cmd = deck_height_cmd / -tiller_depth_cmd (deck_h_default / ' ...
    'till_d_default without comp_ok); end stops at adc.liftpot_lim. Fans: 1-D Lookup fan_bp -> fan_duty on T_encl. ' ...
    'Parameters P.electronics.aux. Built by scripts/build/build_fw_control.m.']);
end

% ===========================================================================================
% fw_watchdog_mgr (Task_1k)
% ===========================================================================================
function local_fw_watchdog_mgr(n)
BT = 'Ts.base'; W = 'P.electronics.wdg'; R = 'P.electronics.safety.rearm';
IN(n, 'alive', 1, 'uint8', BT, '3'); IN(n, 'fault_inject', 2, 'uint32', BT); IN(n, 'fault_word', 3, 'uint32', BT);
IN(n, 'mode', 4, 'uint8', BT); IN(n, 'rearm_req', 5, 'boolean', BT);
% ---- task ages (ms ticks since the alive counter last changed) ----
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'Alive_change', 'vinit', '0');
L(n, 'alive/1', 'Alive_change/1', 'alive');
AGE(n, 'Age', '65535', '0');
L(n, 'Alive_change/1', 'Age/1', 'alive_change');
CST(n, 'Age_lim', ['round(' W '.age_lim/Ts.base)'], 'uint16', BT);
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'Age_ok', 'Operator', '<');
L(n, 'Age/1', 'Age_ok/1', 'age_ms'); L(n, 'Age_lim/1', 'Age_ok/2');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Healthy', 'Operator', 'AND', 'Inputs', '1');
L(n, 'Age_ok/1', 'Healthy/1');
BIT(n, 'Stall_1k', 'fault_inject/1', [W '.stall_1k_bit'], 'uint32');
LOGIC(n, 'No_stall', 'NOT', 1); L(n, 'Stall_1k/1', 'No_stall/1', 'stall_1k');
LOGIC(n, 'Kick', 'AND', 2); L(n, 'Healthy/1', 'Kick/1', 'healthy'); L(n, 'No_stall/1', 'Kick/2');
% ---- MCU_SAFETY_OK (DO0) = healthy AND (fault_word AND maskDO0) == 0 AND mode ~= BOOT (D30) ----
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'Do0_mask', 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', 'P.electronics.safety.maskDO0', 'BitMaskRealWorld', 'Stored Integer');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'Internal_ok', 'relop', '==', 'OutDataTypeStr', 'boolean');
L(n, 'fault_word/1', 'Do0_mask/1', 'fault_word'); L(n, 'Do0_mask/1', 'Internal_ok/1');
CMP(n, 'Not_boot', '~=', 'double(ShaddadMode.BOOT)'); L(n, 'mode/1', 'Not_boot/1', 'mode');
LOGIC(n, 'DO0', 'AND', 3); L(n, 'Healthy/1', 'DO0/1'); L(n, 'Internal_ok/1', 'DO0/2'); L(n, 'Not_boot/1', 'DO0/3');
% ---- SAFETY_RESET_REQ (DO1): rearm_req armed until DO0 has been high for t_do0_settle, then a t_pulse monostable ----
A(n, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Rearm_rise', 'vinit', '0');
L(n, 'rearm_req/1', 'Rearm_rise/1', 'rearm_req');
AGE(n, 'Do0_low_age', '65535', '0');                % ticks since DO0 was last low
LOGIC(n, 'Do0_low', 'NOT', 1); L(n, 'DO0/1', 'Do0_low/1', 'do0');
L(n, 'Do0_low/1', 'Do0_low_age/1');
CMP(n, 'Settled', '>', ['round(' R '.t_do0_settle/Ts.base)']); L(n, 'Do0_low_age/1', 'Settled/1', 'do0_high_ms');
A(n, 'simulink/Discrete/Unit Delay', 'Armed_prev', 'InitialCondition', 'false', 'SampleTime', '-1');
LOGIC(n, 'Armed', 'OR', 2); L(n, 'Rearm_rise/1', 'Armed/1'); L(n, 'Armed_prev/1', 'Armed/2', 'armed_prev');
LOGIC(n, 'Fire', 'AND', 2); L(n, 'Armed/1', 'Fire/1', 'armed'); L(n, 'Settled/1', 'Fire/2');
LOGIC(n, 'Not_fire', 'NOT', 1); L(n, 'Fire/1', 'Not_fire/1', 'fire');
LOGIC(n, 'Armed_next', 'AND', 2); L(n, 'Armed/1', 'Armed_next/1'); L(n, 'Not_fire/1', 'Armed_next/2');
L(n, 'Armed_next/1', 'Armed_prev/1');
A(n, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'Fire_rise', 'vinit', '0'); L(n, 'Fire/1', 'Fire_rise/1');
CST(n, 'Tick', '1', 'single', BT);
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'Pulse_ticks', 'IntegratorMethod', 'Accumulation: Forward Euler', ...
    'gainval', '1.0', 'ExternalReset', 'rising', 'InitialConditionSource', 'internal', 'InitialCondition', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', ['round(' R '.t_pulse/Ts.base) + 1'], 'LowerSaturationLimit', '0', ...
    'OutDataTypeStr', 'single', 'SampleTime', '-1');
L(n, 'Tick/1', 'Pulse_ticks/1'); L(n, 'Fire_rise/1', 'Pulse_ticks/2', 'fire_rise');
A(n, 'simulink/Discrete/Unit Delay', 'Started_prev', 'InitialCondition', 'false', 'SampleTime', '-1');
LOGIC(n, 'Started', 'OR', 2); L(n, 'Fire/1', 'Started/1'); L(n, 'Started_prev/1', 'Started/2');
L(n, 'Started/1', 'Started_prev/1', 'started');
CMP(n, 'In_pulse', '<', ['round(' R '.t_pulse/Ts.base)']); L(n, 'Pulse_ticks/1', 'In_pulse/1', 'pulse_ticks');
LOGIC(n, 'DO1', 'AND', 2); L(n, 'Started/1', 'DO1/1'); L(n, 'In_pulse/1', 'DO1/2');
OUT(n, 'wdg_kick', 1, 'boolean'); OUT(n, 'mcu_safety_ok', 2, 'boolean'); OUT(n, 'safety_reset_req', 3, 'boolean');
L(n, 'Kick/1', 'wdg_kick/1', 'wdg_kick'); L(n, 'DO0/1', 'mcu_safety_ok/1'); L(n, 'DO1/1', 'safety_reset_req/1', 'do1');
set_param(n, 'Description', ['fw_watchdog_mgr (T09, Task_1k, spec 2.7.11, ICD 6, D30). in 1 alive uint8[3] (alive ' ...
    'counters [Task_10 Task_50 Task_100] through RT -> 0.001), 2 fault_inject uint32 (SIM: bit wdg.stall_1k_bit ' ...
    'emulates a hang of the kick), 3 fault_word uint32, 4 mode uint8, 5 rearm_req boolean (all RT -> 0.001). out 1 ' ...
    'wdg_kick = all task ages < round(wdg.age_lim/Ts.base) [Task_10 Task_50 Task_100] AND NOT stall bit (Detect ' ...
    'Change -> uint16 age counters), 2 mcu_safety_ok (DO0) = ages healthy AND (fault_word AND safety.maskDO0) == 0 ' ...
    'AND mode ~= BOOT, 3 safety_reset_req (DO1): a rearm_req rising edge arms the request until DO0 has been high ' ...
    'for safety.rearm.t_do0_settle, then a safety.rearm.t_pulse monostable (Detect Rise Positive -> Discrete-Time ' ...
    'Integrator tick counter with rising reset -> Compare To Constant). Built by scripts/build/build_fw_control.m.']);
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

function SELB(sys, bus, blk, sigs)
% a Bus Selector blk that selects sigs from the bus line bus ('Blk/k', the line named after the block, e.g. fw10);
% the caller wires its outputs (a pick right in front of its consumer, spec 6.2, or one pick for a column of like blocks)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, bus, [blk '/1'], extractBefore(bus, '/'));
end

function PICK(sys, bus, blk, sigs, dsts)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line bus ('Blk/k') and
% feeds its output k to dsts{k} ('Blk/j')
SELB(sys, bus, blk, sigs);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end

function local_name_rest(n)
% L5 (layout spec 6.1): every signal line of every level of file n (shaddad_layout_diagrams) that no call named takes its
% source block's name, '_k' for output k of a block with several outputs, as shaddad_bu('nline') names a line; a line out
% of a Bus Selector keeps the '<element>' name R2026b gives it
d = shaddad_layout_diagrams(n);
for q = 1:numel(d)
    b = find_system(d{q}, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'Type', 'block');
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

function p = IN(sys, name, port, type, ts, dims)
p = A(sys, 'simulink/Sources/In1', name, 'Port', num2str(port));
if ~isempty(type), set_param(p, 'OutDataTypeStr', type); end
if ~isempty(ts), set_param(p, 'SampleTime', ts); end
if nargin >= 6 && ~isempty(dims), set_param(p, 'PortDimensions', dims); end
end

function p = OUT(sys, name, port, type)
p = A(sys, 'simulink/Sinks/Out1', name, 'Port', num2str(port));
if ~isempty(type), set_param(p, 'OutDataTypeStr', type); end
end

function p = CST(sys, name, val, type, ts)
p = A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', type, 'SampleTime', ts);
end

function p = DTC(sys, name, type)
p = A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', type, ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
end

function p = CMP(sys, name, op, c)
p = A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', name, 'relop', op, 'const', c, ...
    'OutDataTypeStr', 'boolean');
end

function p = LOGIC(sys, name, op, nin)
p = A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(nin));
end

function BIT(sys, name, src, bitExpr, ~)
% boolean = (word AND 2^bit) ~= 0
A(sys, 'simulink/Logic and Bit Operations/Bitwise Operator', [name '_mask'], 'logicop', 'AND', 'UseBitMask', 'on', ...
    'BitMask', ['2^(' bitExpr ')'], 'BitMaskRealWorld', 'Stored Integer');
A(sys, 'simulink/Logic and Bit Operations/Compare To Zero', name, 'relop', '~=', 'OutDataTypeStr', 'boolean');
L(sys, src, [name '_mask/1']); L(sys, [name '_mask/1'], [name '/1']);
end

function MODESET(sys, name, src, lo, hi)
% boolean = ShaddadMode.lo <= mode <= ShaddadMode.hi
CMP(sys, [name '_lo'], '>=', ['double(ShaddadMode.' lo ')']);
CMP(sys, [name '_hi'], '<=', ['double(ShaddadMode.' hi ')']);
LOGIC(sys, name, 'AND', 2);
L(sys, src, [name '_lo/1']); L(sys, src, [name '_hi/1']);
L(sys, [name '_lo/1'], [name '/1']); L(sys, [name '_hi/1'], [name '/2']);
end

function s = SUB(sys, name, ins, outs)
s = [sys '/' name];
add_block('built-in/SubSystem', s);
for k = 1:numel(ins), A(s, 'simulink/Sources/In1', ins{k}, 'Port', num2str(k)); end
for k = 1:numel(outs), A(s, 'simulink/Sinks/Out1', outs{k}, 'Port', num2str(k)); end
end

function p = BCB(sys, name, bus, map)
% nonvirtual Bus Creator with a bus object; map {element, source} in any order
B = evalin('base', bus);
el = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
assert(numel(el) == size(map, 1) && all(ismember(el, map(:, 1))), 'build_fw_control:bus', ...
    '%s: map does not cover the elements of %s', name, bus);
p = A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(el)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(el)
    L(sys, map{strcmp(map(:, 1), el{k}), 2}, sprintf('%s/%d', name, k), el{k});
end
end

function AGE(sys, name, satExpr, icExpr)
% age counter subsystem: in 'fresh' (boolean, scalar or vector) -> out 'ticks' uint16 =
% fresh ? 0 : min(prev + 1, sat); starts at icExpr (default sat = "never seen")
if nargin < 4, icExpr = satExpr; end
s = SUB(sys, name, {'fresh'}, {'ticks'});
A(s, 'simulink/Discrete/Unit Delay', 'Prev', 'InitialCondition', icExpr, 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'One', 'Value', '1', 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'Sat', 'Value', satExpr, 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Sources/Constant', 'Zero', 'Value', '0', 'OutDataTypeStr', 'uint16', 'SampleTime', '-1');
A(s, 'simulink/Math Operations/Sum', 'Inc', 'Inputs', '++', 'OutDataTypeStr', 'uint16', 'SaturateOnIntegerOverflow', 'on');
A(s, 'simulink/Math Operations/MinMax', 'Lim', 'Function', 'min', 'Inputs', '2');
A(s, 'simulink/Signal Routing/Switch', 'Rst', 'Criteria', 'u2 ~= 0');
L(s, 'Prev/1', 'Inc/1', 'prev'); L(s, 'One/1', 'Inc/2', 'one'); L(s, 'Inc/1', 'Lim/1', 'inc'); L(s, 'Sat/1', 'Lim/2', 'sat');
L(s, 'Zero/1', 'Rst/1', 'zero'); L(s, 'fresh/1', 'Rst/2', 'fresh'); L(s, 'Lim/1', 'Rst/3', 'inc_sat');
L(s, 'Rst/1', 'Prev/1', 'ticks'); L(s, 'Rst/1', 'ticks/1');
end
