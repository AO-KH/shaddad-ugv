function [mode, cmd_src, drive_enable, rearm_req, bms_req, stop_ctl] = mcu_mode_manager(rc_ok, sw_mode, arm, ...
    arm_req, sticks_neutral, override, reset_req, reset_evt, fault_word, fault_cls, comp_ok, gnc_ready, mode_req, ...
    hv_ready, key_on, safety_ok, lid_closed, dock_present, standstill, v_max, brakes_ok, selftest_ok, prm_n, ...
    prm_mask, prm_flag, prm_s)
%MCU_MODE_MANAGER  STM32 MCU mode manager FSM (Shaddad_MCU/Task_10/Mode_Manager, T09; Stateflow not installed).
%#codegen
%
%   Called once per Task_10 step (0.01 s) by the MATLAB Function block of fw_mode_manager.slx. States are
%   ShaddadMode codes (uint8, docs/icd.md 9); all timers are tick counts of the 0.01 s task.
%
%   Inputs (all from blocks of fw_mode_manager; constants arrive on inputs, no parameter data):
%     rc_ok, sw_mode (1 MANUAL 2 HOLD 3 AUTO), arm, arm_req (rising edge of arm), sticks_neutral, override
%                         fw_rc_mapping (ICD 7)
%     reset_req           DI RESET_BTN OR RC CH9 (level); reset_evt = watchdog-reset event (-> BOOT)
%     fault_word          supervisor fault word (ICD 6) of this sample; fault_cls boolean[3] [A B C]
%     comp_ok, gnc_ready, mode_req (GncModeReq)      companion (COMP_CMD through the Task_50 -> Task_10 RT)
%     hv_ready            BMS HV_Ready (power_ready, D4)
%     key_on, safety_ok, lid_closed, dock_present     DI word bits (ICD 6)
%     standstill          max(|v_track_L|, |v_track_R|) < P.electronics.safety.standstill_v (both tracks, so a
%                         pivot turn is no standstill; R-34, 2026-11-15)
%     v_max   single      max(|v_track_L|, |v_track_R|) (m/s): the faster track
%     brakes_ok           both ESCs report the brake engaged with the torque off: FB1 brake_released false, ESC
%                         state READY / NOT_READY / FAULT / STO (not ENABLING..QUICK_STOP), rx_ok, drv_age fresh
%     selftest_ok         rails in range, ESC and BMS frames fresh, IMU valid
%     prm_n  uint16[9]    [N_selftest N_selftest_timeout N_auto_start N_standstill N_ctrlstop_max N_estop_decel
%                         N_sf_t0 N_sf_brake N_sf_move] ticks
%     prm_mask uint32[3]  [maskA maskB mask_rc] (mask_rc = RC_LINK_LOST | RC_FAILSAFE)
%     prm_flag boolean[2] [rc_required_in_auto auto_rearm_on_selftest]
%     prm_s  single[7]    [a_ctrlstop a_estop ctrlstop_t_extra stopfail_dv stopfail_rate standstill_v ticks_per_s]
%   Outputs: mode (ShaddadMode code), cmd_src (ShaddadCmdSrc code), drive_enable, rearm_req (one-sample pulse:
%   SELFTEST entry and every reset_req rising edge, D30), bms_req boolean[4] [HV_Enable_Req Charge_Enable_Req
%   Fault_Reset_Req Shutdown_Req] (BMS_REQ, D4), stop_ctl boolean[2] [limits_hold stop_fail]:
%     limits_hold   the parking phase of CTRL_STOP or ESTOP: the drive is off (park_req = NOT drive_enable) but the
%                   torque limits stay on until both brakes confirm (fw_vehicle_ctrl, R-01 / D71)
%     stop_fail     the stop monitor of this sample (fault bit 22 STOP_FAIL one sample later, class A, in maskDO0):
%                   (a) in CTRL_STOP, ESTOP.DECEL or an ESTOP park the faster track rises stopfail_dv above v0, or
%                       from N_sf_t0 after entry stays above max(v0 - stopfail_rate*a*t, standstill_v), a =
%                       a_ctrlstop / a_estop, v0 = v_max at entry;
%                   (b) the brakes do not confirm N_sf_brake after the park request;
%                   (c) motion while stopped: v_max > standstill_v for N_sf_move in STANDBY, or in ESTOP.SAFE or
%                       FAULT (both can be entered at speed) once standstill was seen there
%
%   Transitions (priority order; docs/specs/electronics.md 2.7.3 with D18, D20, D21, D30, D34, D44, D71):
%     reset_evt -> BOOT;  BOOT -> SELFTEST after one tick (entry: rearm_req)
%     HW E-stop (bit 0)                       any state except ESTOP -> ESTOP.SAFE (drive off)
%     remote E-stop (bit 1)                   SELFTEST/STANDBY/ACTIVE/CTRL_STOP -> ESTOP (DECEL when the drive
%                                             was enabled, PARK from the CTRL_STOP parking or when the drive was
%                                             enabled with OVERSPEED set, else SAFE)
%     class A (except bit 0)                  any state except ESTOP / FAULT -> FAULT
%     SELFTEST -> STANDBY after N_selftest when selftest_ok; -> FAULT after N_selftest_timeout
%     STANDBY -> MANUAL      rc_ok, arm_req, sticks_neutral, hv_ready, DI SAFETY_OK, sw_mode 1, no class A/B
%     STANDBY -> AUTO_START  rc_ok, arm_req, hv_ready, DI SAFETY_OK, sw_mode 3, comp_ok, gnc_ready, lid closed,
%                            no class A/B (D20)
%     ACTIVE (MANUAL HOLD AUTO_START AUTO) -> CTRL_STOP on class B, NOT arm, and NOT rc_ok unless
%                            (AUTO and NOT rc_required_in_auto) (D44; RC bits 2/3 and the RC inputs are then ignored)
%     AUTO_START/AUTO -> CTRL_STOP on NOT comp_ok or mode_req FAULT_STOP
%     MANUAL -> HOLD (sw 2), MANUAL -> AUTO_START on a change to sw 3 when ready (else HOLD),
%     HOLD -> MANUAL (sw 1), HOLD -> AUTO_START (sw 3 and ready), AUTO_START -> AUTO after N_auto_start (sw 3),
%     AUTO_START/AUTO -> MANUAL on sw 1 or override, -> HOLD on sw 2
%     CTRL_STOP: decelerates (drive enabled) until standstill held N_standstill or the time-out
%                min(N_ctrlstop_max, (v0/a_ctrlstop + ctrlstop_t_extra)*ticks_per_s); entered with OVERSPEED
%                (bit 16) set it skips the deceleration, and OVERSPEED set during it ends it at once (the
%                descent-cap trip, descent task 2026-11-15); then parks (drive off, park_req, limits held) and
%                -> STANDBY once brakes_ok (D71)
%     ESTOP.DECEL -> ESTOP park on standstill, after N_estop_decel (1.2 s) or on OVERSPEED; park -> ESTOP.SAFE
%                once brakes_ok
%     ESTOP (SAFE) -> STANDBY on a reset_req edge with bits 0/1 cleared and standstill
%     FAULT -> STANDBY on a reset_req edge with no class A / B and standstill
%   A new arm_req (arm off -> on) is needed after every stop.

M_BOOT = uint8(ShaddadMode.BOOT);        M_SELFTEST = uint8(ShaddadMode.SELFTEST);
M_STANDBY = uint8(ShaddadMode.STANDBY);  M_MANUAL = uint8(ShaddadMode.MANUAL);
M_HOLD = uint8(ShaddadMode.HOLD);        M_AUTO_START = uint8(ShaddadMode.AUTO_START);
M_AUTO = uint8(ShaddadMode.AUTO);        M_CTRL_STOP = uint8(ShaddadMode.CTRL_STOP);
M_ESTOP = uint8(ShaddadMode.ESTOP);      M_FAULT = uint8(ShaddadMode.FAULT);
FAULT_STOP = uint8(GncModeReq.FAULT_STOP);
BIT_HW_ESTOP = uint32(1);                % fault bit 0
BIT_RC_ESTOP = uint32(2);                % fault bit 1
BIT_OVERSPEED = uint32(65536);           % fault bit 16

persistent st tmr hold decel rst_prev sw_prev sw_held arm_held init_done park park_tmr v0 n_cs move_cnt still_seen
first = false;
if isempty(init_done)
    st = M_BOOT; tmr = uint16(0); hold = uint16(0); decel = false; rst_prev = false;
    sw_prev = uint8(2); sw_held = uint8(2); arm_held = false; init_done = true;
    park = false; park_tmr = uint16(0); v0 = single(0); n_cs = uint16(0); move_cnt = uint16(0); still_seen = false;
    first = true;                                     % power-up: this call is the BOOT entry tick
end

N_selftest = prm_n(1); N_selftest_to = prm_n(2); N_auto_start = prm_n(3);
N_standstill = prm_n(4); N_ctrlstop_max = prm_n(5); N_estop_decel = prm_n(6);
N_sf_t0 = prm_n(7); N_sf_brake = prm_n(8); N_sf_move = prm_n(9);
maskA = prm_mask(1); maskB = prm_mask(2); mask_rc = prm_mask(3);
rc_required = prm_flag(1); auto_rearm = prm_flag(2);
a_ctrlstop = prm_s(1); a_estop = prm_s(2); t_extra = prm_s(3); sf_dv = prm_s(4); sf_rate = prm_s(5);
v_still = prm_s(6); f_tick = prm_s(7);

% ---- derived inputs ----
rst_edge = reset_req && ~rst_prev;
estop_hw = bitand(fault_word, BIT_HW_ESTOP) ~= 0;
estop_rc = bitand(fault_word, BIT_RC_ESTOP) ~= 0;
overspeed = bitand(fault_word, BIT_OVERSPEED) ~= 0;
clsA = bitand(fault_word, maskA) ~= 0;
clsA_ex = bitand(fault_word, bitand(maskA, bitcmp(BIT_HW_ESTOP))) ~= 0;
rc_waived = (st == M_AUTO) && ~rc_required;          % AUTO without RC supervision (D44)
wordB = bitand(fault_word, maskB);
if rc_waived
    wordB = bitand(wordB, bitcmp(mask_rc));
end
clsB = wordB ~= 0;
% RC inputs are used only while the link is good (held otherwise; failsafe presets never act)
if rc_ok
    sw_held = uint8(sw_mode); arm_held = logical(arm);
end
sw = sw_held;
arm_eff = arm_held;
ovr = override && rc_ok;
arm_edge = arm_req && rc_ok;
sw_changed = sw ~= sw_prev;
auto_ready = comp_ok && gnc_ready && lid_closed;
active = st == M_MANUAL || st == M_HOLD || st == M_AUTO_START || st == M_AUTO;

if ~first
    tmr = tmr + 1;                                    % ticks since entry of st (uint16, saturates)
end
nxt = st;
park_nxt = park;                                      % parking phase of CTRL_STOP / ESTOP (D71)
rearm = false;
if st == M_ESTOP && estop_hw
    decel = false; park_nxt = false;                  % a hardware E-stop ends ESTOP.DECEL and the park at once
end
if reset_evt
    nxt = M_BOOT;
elseif st == M_BOOT
    if tmr >= 1                                       % one tick in BOOT
        nxt = M_SELFTEST;
        rearm = auto_rearm;                           % automatic re-arm on SELFTEST entry (D30)
    end
elseif estop_hw && st ~= M_ESTOP
    nxt = M_ESTOP; decel = false; park_nxt = false;
elseif estop_rc && (st == M_SELFTEST || st == M_STANDBY || active || st == M_CTRL_STOP)
    nxt = M_ESTOP;
    moving = active || (st == M_CTRL_STOP && ~park);  % a moving drive decelerates (ESTOP.DECEL) unless OVERSPEED is
    decel = moving && ~overspeed;                     % set: then it parks at once (descent task, as the CTRL_STOP)
    park_nxt = (st == M_CTRL_STOP && park) || (moving && overspeed);   % the CTRL_STOP parking goes on as the ESTOP park
elseif clsA_ex && st ~= M_ESTOP && st ~= M_FAULT
    nxt = M_FAULT;
elseif st == M_SELFTEST
    if tmr >= N_selftest && selftest_ok
        nxt = M_STANDBY;
    elseif tmr >= N_selftest_to
        nxt = M_FAULT;
    end
elseif st == M_STANDBY
    if arm_edge && hv_ready && safety_ok && ~clsA && ~clsB
        if sw == 1 && sticks_neutral
            nxt = M_MANUAL;
        elseif sw == 3 && auto_ready
            nxt = M_AUTO_START;
        end
    end
elseif active
    if clsB || ~arm_eff || (~rc_ok && ~rc_waived)
        nxt = M_CTRL_STOP;
    elseif (st == M_AUTO_START || st == M_AUTO) && (~comp_ok || mode_req == FAULT_STOP)
        nxt = M_CTRL_STOP;
    elseif st == M_MANUAL
        if sw == 2
            nxt = M_HOLD;
        elseif sw == 3 && sw_changed
            if auto_ready
                nxt = M_AUTO_START;
            else
                nxt = M_HOLD;
            end
        end
    elseif st == M_HOLD
        if sw == 1
            nxt = M_MANUAL;
        elseif sw == 3 && auto_ready
            nxt = M_AUTO_START;
        end
    else                                              % AUTO_START, AUTO
        if sw == 1 || ovr
            nxt = M_MANUAL;
        elseif sw == 2
            nxt = M_HOLD;
        elseif st == M_AUTO_START && tmr >= N_auto_start && sw == 3
            nxt = M_AUTO;
        end
    end
    if nxt == M_CTRL_STOP
        park_nxt = overspeed;                         % a runaway (OVERSPEED) skips the regen deceleration (D71)
    end
elseif st == M_CTRL_STOP
    if ~park
        if standstill
            hold = hold + 1;
        else
            hold = uint16(0);
        end
        if hold >= N_standstill || tmr >= n_cs || overspeed
            park_nxt = true; park_tmr = uint16(0);    % end of the deceleration: park on the brakes (D71); OVERSPEED
        end                                           % during it (the descent-cap trip: the speed runs past the cap)
                                                      % parks at once (descent task, 2026-11-15)
    end
    if park_nxt && brakes_ok
        nxt = M_STANDBY;                              % both brakes confirmed: the limits go to 0 in STANDBY
    end
elseif st == M_ESTOP
    if decel && (standstill || tmr >= N_estop_decel || overspeed)
        decel = false;                                % ESTOP.DECEL -> ESTOP park (drive off, limits held); OVERSPEED
                                                      % during the deceleration parks at once (descent task)
        park_nxt = true; park_tmr = uint16(0);
    end
    if park_nxt && brakes_ok
        park_nxt = false;                             % ESTOP park -> ESTOP.SAFE
    end
    if rst_edge && ~decel && ~park_nxt && ~estop_hw && ~estop_rc && standstill
        nxt = M_STANDBY;
    end
elseif st == M_FAULT
    if rst_edge && ~clsA && ~clsB && standstill
        nxt = M_STANDBY;
    end
else
    nxt = M_FAULT;                                    % unknown code (defensive)
end
if rst_edge
    rearm = true;                                     % every reset_req edge re-issues SAFETY_RESET_REQ (D30)
end
if nxt ~= M_CTRL_STOP && nxt ~= M_ESTOP
    park_nxt = false;
end

% ---- state update (timers restart on every transition) ----
if nxt ~= st
    tmr = uint16(0); hold = uint16(0); move_cnt = uint16(0); still_seen = false;
    if ~(park && park_nxt)
        park_tmr = uint16(0);                         % a park that goes on (CTRL_STOP -> ESTOP) keeps its timer
    end
    if nxt ~= M_ESTOP
        decel = false;
    end
    if nxt == M_CTRL_STOP || nxt == M_ESTOP
        v0 = v_max;                                   % the faster track at entry (stop monitor, time-out)
        n_cs = uint16(min(single(N_ctrlstop_max), round((v0/a_ctrlstop + t_extra)*f_tick)));
    end
end
st = nxt;
park = park_nxt;
rst_prev = reset_req;
sw_prev = sw;

% ---- stop monitor (fault bit 22 STOP_FAIL one sample later, D71) ----
sf = false;
if (st == M_CTRL_STOP) || (st == M_ESTOP && (decel || park))
    % (a) the speed does not fall
    if st == M_CTRL_STOP, a_stop = a_ctrlstop; else, a_stop = a_estop; end
    bound = max(v0 - sf_rate*a_stop*single(tmr)/f_tick, v_still);
    sf = (v_max > v0 + sf_dv) || (tmr >= N_sf_t0 && v_max > bound);
end
if park
    sf = sf || park_tmr >= N_sf_brake;                % (b) the brakes do not confirm
    park_tmr = park_tmr + 1;
end
stopped = st == M_STANDBY || st == M_FAULT || (st == M_ESTOP && ~decel && ~park);
if stopped
    % armed at once in STANDBY (entered at rest, or from a park whose brakes confirmed: motion then is a brake that
    % does not hold); in FAULT and ESTOP.SAFE, which can be entered at speed, once standstill was seen there
    still_seen = still_seen || v_max <= v_still || st == M_STANDBY;
    if still_seen && v_max > v_still
        move_cnt = move_cnt + 1;
    else
        move_cnt = uint16(0);
    end
    sf = sf || move_cnt >= N_sf_move;                 % (c) motion while stopped
end

% ---- outputs ----
mode = st;
if st == M_MANUAL
    cmd_src = uint8(ShaddadCmdSrc.RC);
elseif st == M_AUTO
    cmd_src = uint8(ShaddadCmdSrc.AUTO);
else
    cmd_src = uint8(ShaddadCmdSrc.NONE);
end
drive_enable = st == M_MANUAL || st == M_HOLD || st == M_AUTO_START || st == M_AUTO || ...
    (st == M_CTRL_STOP && ~park) || (st == M_ESTOP && decel);
rearm_req = rearm;
hv_req = key_on && ~fault_cls(1) && st ~= M_BOOT && st ~= M_FAULT;
chg_req = key_on && dock_present && st == M_STANDBY;
bms_req = [hv_req, chg_req, reset_req, ~key_on];
stop_ctl = [(st == M_CTRL_STOP || st == M_ESTOP) && park, sf];
end
