function fsm = gnc_missionFSM(nav, mcu, bms, mcmd, obs, pe, plan, rp, teb, socRth, prm) %#codegen
%GNC_MISSIONFSM  GNC mission manager: top-level autonomy state machine (MATLAB Function, 0.05 s).
%
%   fsm = gnc_missionFSM(nav, mcu, bms, mcmd, obs, pe, plan, rp, teb, socRth, prm)
%     nav     Bus_NavCond      pose [x y psi], v, pitch, roll, sigma_pos, nav_ok, nav_degraded
%     mcu     Bus_McuStatus    rc_mode_sw (0 MANUAL / 1 HOLD / 2 AUTO), rc_link_ok, estop_active, drive_ready,
%                              implement_ready, charging, heartbeat, v_applied, valid
%     bms     Bus_BmsStatus    SOC, bms_critical, valid
%     mcmd    Bus_MissionCmd   conditioned: cmd = the new MissionCmd in the step it arrives, else NONE
%     obs     Bus_ObsFlags     d_obs (Inf = clear), blocked_vfh, person_near, scan_valid (Bus_Perception.scanValid; a
%                              blank scan reads as clear: d_obs Inf, empty VFH, so the FSM holds on a lost scan)
%     pe      Bus_PathError    delayed one step (Unit Delay, loop breaker)
%     plan    Bus_MissionPlan  MissionSource constant (segment table, path rows for goals, geofence)
%     rp      Bus_ReplanResult delayed one step (ok, ver, fail_count)
%     teb     double[4]        TEB SolnInfo [ExitFlag LastFeasibleIdx HasReachedGoal TrajectoryCost], delayed;
%                              ExitFlag -1 = no TEB (VC_TEB = 0: VFH-steered pursuit on a Hybrid A* detour)
%     socRth  double           return-to-home SOC threshold (block math, spec 2.8)
%     prm     P.gnc.fsm        non-tunable parameter sub-struct
%     fsm     Bus_FsmOut
%
%   Why a MATLAB Function (docs/specs/gnc.md 4.1, F1): Stateflow is not installed; 7 states, 10 sub-modes
%   and about 30 timed, guarded transitions. States are Simulink enumerations (GncState, AutoSub,
%   HoldReason) held in persistent variables; the code is an ordered transition table (first match
%   fires, G1/G2 first) so it ports 1:1 to a chart. fsm.trans_id reports the transition of the step.
%
%   Timers: a timer is 0 in the first step its condition holds and grows by Ts while it holds; the
%   transition fires at timer >= T, i.e. T after the first step in which the condition was seen.
%
%   Transition table (spec 4.1 with the revision-B changes of build plan T15):
%     G1  any ~FAULT : estop | MCU link lost (heartbeat age >= t_mcu_timeout or ~valid, after the first
%                      valid frame) | bms_critical | in {AUTO, RTH, HOLD}: nav lost >= t_nav_lost,
%                      geofence breach >= t_geofence, stuck >= t_stuck, tilt, |e_y| > xte_fault for t_xte
%                      -> FAULT (latched fault_word bits)
%     G2  ~{FAULT, MANUAL} : rc MANUAL & rc_link_ok -> MANUAL (missionActive remembered)
%     T1  IDLE : START & rc AUTO & nav_ok & plan.valid & SOC > soc_start_min & mcu.drive_ready -> AUTO
%                (seg 1, ALIGN if |e_psi| > align_enter on a pivot segment)
%     T2  IDLE : RTH & rc AUTO & nav_ok & drive_ready -> RTH (GLOBAL replan to the dock-approach start)
%     T3  MANUAL : rc ~MANUAL -> PAUSE (OPERATOR) if missionActive else IDLE
%     T4  MANUAL : ~rc_link_ok -> PAUSE (RC_LOSS) if missionActive else IDLE
%     T5  AUTO : d_obs < d_stop (not in BYPASS / bypass rejoin / dock) | VFH blocked for t_vfh |
%                person_near | scan lost (invalid for more than protNStale scans, t_scan_lost) -> OBSTACLE_HOLD
%                (hist AUTO, s_obs); HoldReason SENSOR_LOST (scan lost) before PERSON, VFH_BLOCKED, OBSTACLE
%     T6  AUTO : PAUSE | rc HOLD -> PAUSE (OPERATOR)
%     36  AUTO / RTH / HOLD : ~drive_ready for t_drive_lost (a shorter drop is the ESC unpark through ENABLING,
%                      drive spec 2.9) | ~drive_ready & ~rc_link_ok (at once) -> PAUSE, HoldReason RC_LOSS when
%                      ~rc_link_ok, else OPERATOR (the MCU owns the safety reaction: RC loss D44, E-stop, CTRL_STOP;
%                      E-stop and a lost MCU link are G1)
%     T7  AUTO / RTH : nav_degraded >= t_nav_degr -> PAUSE (NAV_DEGRADED)
%     T8  AUTO : SOC < socRth (before the final transit) | RTH -> RTH
%     T9  AUTO : ABORT -> IDLE
%     35  AUTO : implement not ready >= t_impl_ready -> PAUSE (IMPLEMENT_NOT_READY); v_cap 0 while waiting
%     S1  AUTO : s >= seg length - seg_end_tol -> seg + 1 (T10 after the last driven segment)
%     T10 AUTO : mission done -> RTH/DOCKED (last segment = DOCK) | RTH (end_action RTH) | IDLE
%     S2  ALIGN : |e_psi| < align_exit (or tilt > align_tilt) -> sub(seg type)
%     S3  BYPASS : end of the detour path reached & d_obs > d_release -> sub(seg type), path MISSION
%     S4  BYPASS (TEB) : ExitFlag 1 or LastFeasibleIdx < teb_lfi_min for teb_t_bad -> REJOIN (LOCAL replan)
%     S5  REJOIN : end of the replanned path -> sub(seg type), path MISSION
%         REJOIN (not a bypass rejoin): a failed LOCAL plan is followed by a GLOBAL plan (no lane mask) to the same
%         goal; only a failed GLOBAL plan counts as a try (retries tries, goal rejoin_off further each, then PAUSE
%         PLANNER_FAIL). BYPASS, its TEB rejoin and a rejoin resumed from a paused bypass (laneOnly) stay LOCAL
%         (lane half-width, spec 8 T7)
%     H1  HOLD : clear (d_obs > d_release, ~blocked, ~person, scan valid) for t_clear_eff -> hist (AUTO or RTH);
%                never on an invalid scan
%     H2  HOLD : t_hold >= t_bypass & seg.allow_bypass & hist AUTO on ROW & ~person & scan valid for t_clear_eff
%                -> AUTO/BYPASS (LOCAL Hybrid A* detour to the lane point x_front + d_obs + rejoin_off past the
%                hold point; followed by TEB (VC_TEB 1) or the VFH-steered pursuit (VC_TEB 0)). Not with TEB when
%                the fitted implement may not reverse (prm.no_rev, the tiller): TEB plans reverse motion; the robot
%                stays in the hold (H1 when clear, H3 PAUSE OBST_TIMEOUT); the VFH-steered pursuit drives forward only
%     H3  HOLD : t_hold >= t_hold_max -> PAUSE (OBST_TIMEOUT; SENSOR_LOST while the scan is lost); H4 (mower off
%                after t_mower_off) is applied by gnc_ImplementManager from fsm.t_hold
%     P1  PAUSE : RESUME & rc AUTO & nav_ok -> AUTO or RTH (re-plan). AUTO: on the lane (|e_y| < resume_xte) with
%                 |e_psi| >= resume_epsi on a pivot segment (seg column 10, tilt <= align_tilt, not the dock
%                 approach) -> ALIGN (pivot in place, then S2); else REJOIN with a LOCAL replan when |e_y| >=
%                 resume_xte or |e_psi| >= resume_epsi (or paused in BYPASS / REJOIN)
%     P2  PAUSE (NAV) : auto_resume_nav & nav ok and not degraded for t_nav_rec -> AUTO
%     P3  PAUSE : (RTH | SOC < socRth) & rc AUTO & nav_ok & drive_ready -> RTH
%     P4  PAUSE : ABORT -> IDLE
%     R1  RTH_PLANNING : replan ok -> RTH_TRANSIT (path REPLAN, window B = dock segment)
%     R2  RTH_PLANNING : retries replans failed -> PAUSE (PLANNER_FAIL)
%     R3  RTH_TRANSIT : end of the replanned path -> DOCK_APPROACH (seg = dock_seg, row tracker, v_dock)
%     R4  DOCK_APPROACH : end of the dock segment (after an in-place heading trim with the ALIGN tracker when
%                         |e_psi| > dock_tol_yaw, until |e_psi| < dock_tol_yaw / 2) -> DOCKED (v = 0); the same trim
%                         ends a mission whose last segment is the dock approach (T10)
%     R5  DOCKED : mcu.charging | t_docked elapsed -> IDLE
%     R6  RTH : obstacle conditions of T5 (a lost scan too) -> OBSTACLE_HOLD (hist RTH)
%     F1  FAULT : RESET_FAULT & no active fault condition & ~estop & rc ~AUTO -> IDLE (fault_word cleared)
%   Outputs: mode_req IDLE STOP, MANUAL MANUAL_ACK, AUTO/RTH AUTO, PAUSE/HOLD HOLD, FAULT FAULT_STOP;
%   cmd_valid only in AUTO and RTH (not RTH_PLANNING / DOCKED); gnc_ready = nav_ok & plan.valid & no fault.
%   hold_reason in OBSTACLE_HOLD: SENSOR_LOST while the scan is lost, else the reason the hold was entered with.
%   impl_allow (spec 2.9): AUTO on ROW / TURN, or an OBSTACLE_HOLD entered there until person_near is seen or the
%   scan is lost in it; from then false for the rest of that hold (latch personSeen, renewed at each hold entry).
%   Mower deck (the user, 2026-11-15: down whenever the robot moves): the deck counts as down after t_deck of AUTO /
%   RTH since power-up (deck_t0) or the last MANUAL (no deck report reaches the GNC); until then, in AUTO / RTH, the
%   STOP tracker and v_cap 0 at standstill (hold_reason IMPLEMENT_NOT_READY), and v_cap <= the deck-raised descent
%   cap while still moving.

persistent S
if isempty(S)
    S = local_init(prm);
end
Ts = prm.Ts;
tid = uint8(0);
cmd = mcmd.cmd;
scanOk = obs.scan_valid;                    % Bus_Perception.scanValid through LocalAvoidance (critfix R-02)
cStart = cmd == uint8(MissionCmd.START);
cPause = cmd == uint8(MissionCmd.PAUSE);
cResume = cmd == uint8(MissionCmd.RESUME);
cAbort = cmd == uint8(MissionCmd.ABORT);
cRth = cmd == uint8(MissionCmd.RTH);
cReset = cmd == uint8(MissionCmd.RESET_FAULT);
swManual = mcu.rc_mode_sw == uint8(0);
swHold = mcu.rc_mode_sw == uint8(1);
swAuto = mcu.rc_mode_sw == uint8(2);
nSeg = double(plan.n_seg);
lastDrv = max(1, nSeg - 1);                  % the last plan row is the TERMINAL extension (never driven)
if S.planKey ~= nSeg
    S.planKey = nSeg;
    S.lastWork = local_lastWork(plan, lastDrv);
end
socOk = bms.valid && double(bms.SOC) > prm.soc_start_min;
socLow = bms.valid && double(bms.SOC) < socRth;

%% ---- supervision (fault conditions, timers advance every step) ----
st = S.st;
inMotion = st == GncState.AUTO_MISSION || st == GncState.RETURN_TO_HOME || st == GncState.OBSTACLE_HOLD;
% MCU link: heartbeat age (counted from the first valid frame) or frame not fresh
if mcu.valid, S.linkSeen = true; end
if mcu.heartbeat ~= S.hb || ~S.linkSeen
    S.hb = mcu.heartbeat;
    S.hbAge = 0;
else
    S.hbAge = S.hbAge + Ts;
end
mcuLost = S.linkSeen && (~mcu.valid || S.hbAge >= prm.t_mcu_timeout - 1e-9);
[S.tNavLost, navLost] = local_timer(S.tNavLost, inMotion && ~nav.nav_ok, prm.t_nav_lost, Ts);
inside = local_inside(plan, nav.pose(1), nav.pose(2));
[S.tGeo, geoBreach] = local_timer(S.tGeo, inMotion && ~inside, prm.t_geofence, Ts);
stuckCond = (st == GncState.AUTO_MISSION || st == GncState.RETURN_TO_HOME) && S.cmdValid && ...
    abs(double(mcu.v_applied)) >= prm.v_stuck_cmd && abs(nav.v) < prm.v_stuck_meas;
[S.tStuck, stuck] = local_timer(S.tStuck, stuckCond, prm.t_stuck, Ts);
tilt = inMotion && (abs(nav.pitch) > prm.pitch_fault || abs(nav.roll) > prm.roll_fault);
trkSub = S.sub == AutoSub.TRANSIT || S.sub == AutoSub.ROW || S.sub == AutoSub.TURN || ...
    S.sub == AutoSub.RTH_TRANSIT || S.sub == AutoSub.DOCK_APPROACH;
xteCond = (st == GncState.AUTO_MISSION || st == GncState.RETURN_TO_HOME) && trkSub && ...
    (~pe.valid || abs(pe.xte) > prm.xte_fault);
[S.tXte, xteFault] = local_timer(S.tXte, xteCond, prm.t_xte, Ts);
f = uint32(0);
f = local_bit(f, 0, navLost);
f = local_bit(f, 2, mcuLost);
f = local_bit(f, 3, xteFault);
f = local_bit(f, 4, stuck);
f = local_bit(f, 5, tilt);
f = local_bit(f, 6, bms.bms_critical);
f = local_bit(f, 9, geoBreach);
f = local_bit(f, 11, mcu.estop_active);
% obstacle conditions (T5 / R6)
[S.tVfh, vfhBlocked] = local_timer(S.tVfh, obs.blocked_vfh, prm.t_vfh, Ts);
dObsOn = ~(S.sub == AutoSub.BYPASS || S.sub == AutoSub.DOCK_APPROACH || S.sub == AutoSub.DOCKED || ...
    (S.sub == AutoSub.REJOIN && S.bypassRejoin));
% scan validity (R-02, NAV-01): the scan is lost after more than P.sensors.per.protNStale consecutive invalid scans
% (t_scan_lost, the protective field's stale rule, which raises protectiveStop in the same step). A lost scan holds
% the robot (T5 / R6), is the hold reason before PERSON and sets the implement latch; a missed scan is none of these.
% H1 needs a valid scan, H2 one valid for t_clear_eff (no detour planned on the first scan after a loss)
[S.tScan, S.scanLost] = local_timer(S.tScan, ~scanOk, prm.t_scan_lost, Ts);
[S.tScanOk, scanSteady] = local_timer(S.tScanOk, scanOk, prm.t_clear_eff, Ts);
obsHit = (dObsOn && obs.d_obs < prm.d_stop) || vfhBlocked || obs.person_near || S.scanLost;
% nav degraded (T7) and recovery (P2)
[S.tNavDeg, navDeg] = local_timer(S.tNavDeg, (st == GncState.AUTO_MISSION || st == GncState.RETURN_TO_HOME) && ...
    nav.nav_degraded, prm.t_nav_degr, Ts);
[S.tNavRec, navRec] = local_timer(S.tNavRec, st == GncState.PAUSE && nav.nav_ok && ~nav.nav_degraded, ...
    prm.t_nav_rec, Ts);
% drive readiness (T36): drive_ready drops for about 0.1 s each time a parked ESC unparks (OPERATION/PARKED ->
% ENABLING -> OPERATION, drive spec 2.9; parked ESCs report OPERATION, so drive_ready holds while parked). Only a
% drop of t_drive_lost or more is a lost drive; with the RC link lost the MCU has stopped already (D44): at once
[S.tDrive, driveLost] = local_timer(S.tDrive, ~mcu.drive_ready, prm.t_drive_lost, Ts);
driveOff = ~mcu.drive_ready && (driveLost || ~mcu.rc_link_ok);
% replan result of the pending request (rp is delayed one step)
rpOk = false;  rpFail = false;
if S.rpPending
    if rp.ok && rp.ver ~= S.rpVer0
        rpOk = true;  S.rpPending = false;
    elseif rp.fail_count ~= S.rpFail0
        rpFail = true;  S.rpPending = false;
    end
end
% implement readiness (spec 2.9): no motion until mcu.implement_ready on implement segments
implNeeded = st == GncState.AUTO_MISSION && plan.implement_type >= uint8(1) && plan.implement_type <= uint8(3) && ...
    local_segCol(plan, S.seg, 6) > 0 && (S.sub == AutoSub.ROW || S.sub == AutoSub.TURN);
S.implWait = implNeeded && ~mcu.implement_ready;
[S.tImpl, implTimeout] = local_timer(S.tImpl, S.implWait, prm.t_impl_ready, Ts);
% mower deck (the user, 2026-11-15: down whenever the robot moves). No deck report reaches the GNC: the deck counts as down
% once the MCU has driven it to deck_height_cmd for t_deck. The MCU moves it toward the GNC's command only in its AUTO mode
% (requested in AUTO_MISSION / RETURN_TO_HOME); the operator may move it in MANUAL; its height at power-up is unknown
% (deck_t0, normally 0). PAUSE / OBSTACLE_HOLD (MCU HOLD) leave it where it is
if st == GncState.MANUAL
    S.tDeck = 0;
elseif st == GncState.AUTO_MISSION || st == GncState.RETURN_TO_HOME
    S.tDeck = min(S.tDeck + Ts, prm.t_deck);
end
S.deckUp = plan.implement_type == uint8(1) && S.tDeck < prm.t_deck - 1e-9;

%% ---- transition table ----
if f ~= uint32(0) && st ~= GncState.FAULT
    % G1
    S = local_enterFault(S, f);
    tid = uint8(1);
elseif st ~= GncState.FAULT && st ~= GncState.MANUAL && swManual && mcu.rc_link_ok
    % G2
    if st ~= GncState.IDLE
        S.missionActive = true;
        if st == GncState.OBSTACLE_HOLD
            S.resumeState = S.hist;
        elseif st ~= GncState.PAUSE
            S.resumeState = st;
        end
    end
    S.st = GncState.MANUAL;
    tid = uint8(2);
else
    switch st
        case GncState.IDLE
            autoGo = prm.auto_start && ~S.missionDone && swAuto && nav.nav_ok && plan.valid && socOk && mcu.drive_ready;
            if (cStart || autoGo) && swAuto && nav.nav_ok && plan.valid && socOk && mcu.drive_ready
                % T1
                S = local_startMission(S, plan, nav, prm);
                tid = uint8(11);
            elseif cRth && swAuto && nav.nav_ok && mcu.drive_ready && plan.valid
                % T2
                S = local_requestRth(S, plan, rp, prm);
                tid = uint8(12);
            end

        case GncState.MANUAL
            if ~swManual
                % T3
                S = local_leaveManual(S, HoldReason.OPERATOR);
                tid = uint8(13);
            elseif ~mcu.rc_link_ok
                % T4
                S = local_leaveManual(S, HoldReason.RC_LOSS);
                tid = uint8(14);
            end

        case GncState.AUTO_MISSION
            if obsHit
                % T5
                S = local_enterHold(S, GncState.AUTO_MISSION, pe, obs, vfhBlocked);
                tid = uint8(15);
            elseif cPause || swHold
                % T6
                S = local_enterPause(S, GncState.AUTO_MISSION, HoldReason.OPERATOR);
                tid = uint8(16);
            elseif driveOff
                % 36
                S = local_enterPause(S, GncState.AUTO_MISSION, local_rcReason(mcu));
                tid = uint8(36);
            elseif navDeg
                % T7
                S = local_enterPause(S, GncState.AUTO_MISSION, HoldReason.NAV_DEGRADED);
                S.autoResume = prm.auto_resume_nav;
                tid = uint8(17);
            elseif cRth || (socLow && S.seg <= S.lastWork)
                % T8
                S = local_requestRth(S, plan, rp, prm);
                tid = uint8(18);
            elseif cAbort
                % T9
                S = local_toIdle(S);
                tid = uint8(19);
            elseif implTimeout
                S = local_enterPause(S, GncState.AUTO_MISSION, HoldReason.IMPLEMENT_NOT_READY);
                tid = uint8(35);
            else
                [S, tid] = local_autoSub(S, plan, nav, pe, obs, rp, teb, rpOk, rpFail, lastDrv, prm);
            end

        case GncState.PAUSE
            if cResume && swAuto && nav.nav_ok
                % P1
                S = local_resume(S, plan, nav, pe, rp, prm);
                tid = uint8(41);
            elseif S.pauseReason == HoldReason.NAV_DEGRADED && S.autoResume && navRec
                % P2
                S = local_resume(S, plan, nav, pe, rp, prm);
                tid = uint8(42);
            elseif (cRth || (socLow && S.resumeState == GncState.AUTO_MISSION && S.seg <= S.lastWork)) && ...
                    swAuto && nav.nav_ok && mcu.drive_ready && plan.valid
                % P3
                S = local_requestRth(S, plan, rp, prm);
                tid = uint8(43);
            elseif cAbort
                % P4
                S = local_toIdle(S);
                tid = uint8(44);
            end

        case GncState.OBSTACLE_HOLD
            S.tHold = S.tHold + Ts;
            % implement latch (spec 2.9, R-03): once person_near was seen (or the scan was lost) in the hold,
            % impl_allow stays false until the hold ends
            S.personSeen = S.personSeen || obs.person_near || S.scanLost;
            % never released on an invalid scan: a blank scan reads as clear (R-02)
            clr = obs.d_obs > prm.d_release && ~obs.blocked_vfh && ~obs.person_near && scanOk;
            [S.tClr, clrDone] = local_timer(S.tClr, clr, prm.t_clear_eff, Ts);
            % no TEB bypass with an implement that may not reverse (the tiller, prm.no_rev): TEB plans reverse motion
            % (MaxReverseVelocity NaN); the robot holds (H1) or pauses (H3). The VFH-steered pursuit (VC_TEB = 0,
            % teb(1) = -1) drives forward only and keeps the bypass
            byp = S.hist == GncState.AUTO_MISSION && S.histSub == AutoSub.ROW && ~obs.person_near && scanSteady && ...
                local_segCol(plan, S.seg, 7) > 0 && ~(local_noRev(plan, prm) && teb(1) ~= -1);
            if clrDone
                % H1
                S.st = S.hist;  S.sub = S.histSub;  S.holdReason = HoldReason.NONE;
                tid = uint8(31);
            elseif S.tHold >= prm.t_bypass - 1e-9 && byp
                % H2
                S = local_startBypass(S, plan, teb, rp, prm);
                tid = uint8(32);
            elseif S.tHold >= prm.t_hold_max - 1e-9
                % H3 (SENSOR_LOST while the scan is lost)
                if S.scanLost
                    S = local_enterPause(S, S.hist, HoldReason.SENSOR_LOST);
                else
                    S = local_enterPause(S, S.hist, HoldReason.OBST_TIMEOUT);
                end
                tid = uint8(33);
            elseif cPause || swHold
                S = local_enterPause(S, S.hist, HoldReason.OPERATOR);
                tid = uint8(16);
            elseif driveOff
                S = local_enterPause(S, S.hist, local_rcReason(mcu));
                tid = uint8(36);
            elseif cAbort
                S = local_toIdle(S);
                tid = uint8(19);
            end

        case GncState.RETURN_TO_HOME
            if obsHit && S.sub ~= AutoSub.RTH_PLANNING && S.sub ~= AutoSub.DOCKED
                % R6
                S = local_enterHold(S, GncState.RETURN_TO_HOME, pe, obs, vfhBlocked);
                tid = uint8(56);
            elseif (cPause || swHold) && S.sub ~= AutoSub.DOCKED
                S = local_enterPause(S, GncState.RETURN_TO_HOME, HoldReason.OPERATOR);
                tid = uint8(16);
            elseif driveOff && S.sub ~= AutoSub.DOCKED
                S = local_enterPause(S, GncState.RETURN_TO_HOME, local_rcReason(mcu));
                tid = uint8(36);
            elseif navDeg && S.sub ~= AutoSub.DOCKED
                S = local_enterPause(S, GncState.RETURN_TO_HOME, HoldReason.NAV_DEGRADED);
                S.autoResume = prm.auto_resume_nav;
                tid = uint8(17);
            elseif cAbort
                S = local_toIdle(S);
                tid = uint8(19);
            else
                [S, tid] = local_rthSub(S, plan, mcu, pe, rpOk, rpFail, rp, prm);
            end

        case GncState.FAULT
            fNow = mcu.estop_active || mcuLost || bms.bms_critical || ...
                abs(nav.pitch) > prm.pitch_fault || abs(nav.roll) > prm.roll_fault;
            if cReset && ~fNow && ~swAuto
                % F1
                S = local_toIdle(S);
                S.faultWord = uint32(0);
                tid = uint8(61);
            else
                S.faultWord = bitor(S.faultWord, f);
            end

        otherwise
            S.st = GncState.IDLE;
    end
end

%% ---- outputs ----
fsm = local_pack(S, plan, nav, tid, prm);
S.cmdValid = fsm.cmd_valid;
end

% =============================================================================================
%  state helpers
% =============================================================================================
function S = local_init(prm)
S = struct('st', GncState.IDLE, 'sub', AutoSub.NONE, 'hist', GncState.AUTO_MISSION, 'histSub', AutoSub.NONE, ...
    'resumeState', GncState.AUTO_MISSION, 'seg', 1, 'nxt', 2, 'pathSrc', PathSrc.MISSION, ...
    'holdReason', HoldReason.NONE, 'pauseReason', HoldReason.NONE, 'faultWord', uint32(0), ...
    'missionActive', false, 'missionDone', false, 'autoResume', false, 'cmdValid', false, ...
    'linkSeen', false, 'hb', uint8(0), 'hbAge', 0, ...
    'tNavLost', -1, 'tGeo', -1, 'tStuck', -1, 'tXte', -1, 'tVfh', -1, 'tNavDeg', -1, 'tNavRec', -1, 'tDrive', -1, ...
    'tImpl', -1, 'tClr', -1, 'tTeb', -1, 'tHold', 0, 'tDocked', 0, 'implWait', false, ...
    'tScan', -1, 'tScanOk', -1, 'scanLost', false, 'personSeen', false, ...
    'sObs', 0, 'dObsHold', 0, ...
    'replanReq', false, 'replanGoal', zeros(3, 1), 'replanKind', uint8(0), 'rpPending', false, ...
    'rpVer0', uint16(0), 'rpFail0', uint8(0), 'tries', 0, 'goalS', 0, ...
    'bypassRejoin', false, 'laneOnly', false, 'tebMode', true, 'srcAge', 0, 'planKey', -1, 'lastWork', 1, ...
    'dockTrim', false, 'tDeck', min(max(prm.deck_t0, 0), prm.t_deck), 'deckUp', false);
end

function [t, fire] = local_timer(t, cond, T, Ts)
if cond
    if t < 0
        t = 0;
    else
        t = t + Ts;
    end
else
    t = -1;
end
fire = cond && t >= T - 1e-9;
end

function f = local_bit(f, b, c)
if c
    f = bitor(f, bitshift(uint32(1), b));
end
end

function r = local_rcReason(mcu)
if mcu.rc_link_ok
    r = HoldReason.OPERATOR;
else
    r = HoldReason.RC_LOSS;
end
end

function S = local_enterFault(S, f)
S.st = GncState.FAULT;
S.sub = AutoSub.NONE;
S.faultWord = bitor(S.faultWord, f);
S.pathSrc = PathSrc.MISSION;
S.rpPending = false;
S.missionActive = false;
S.holdReason = HoldReason.NONE;
end

function S = local_toIdle(S)
S.st = GncState.IDLE;
S.sub = AutoSub.NONE;
S.pathSrc = PathSrc.MISSION;
S.rpPending = false;
S.missionActive = false;
S.holdReason = HoldReason.NONE;
S.pauseReason = HoldReason.NONE;
S.bypassRejoin = false;
S.seg = 1;
end

function S = local_leaveManual(S, reason)
if S.missionActive
    S.st = GncState.PAUSE;
    S.pauseReason = reason;
    S.holdReason = reason;
    S.autoResume = false;
else
    S = local_toIdle(S);
    if reason == HoldReason.RC_LOSS
        S.holdReason = reason;               % warning: RC lost while in MANUAL (MCU failsafe brakes)
    end
end
end

function S = local_enterPause(S, from, reason)
S.resumeState = from;
S.st = GncState.PAUSE;
S.pauseReason = reason;
S.holdReason = reason;
S.autoResume = false;
S.missionActive = true;
S.rpPending = false;
S.tNavRec = -1;
end

function S = local_enterHold(S, from, pe, obs, vfhBlocked)
S.hist = from;
S.histSub = S.sub;
S.st = GncState.OBSTACLE_HOLD;
S.tHold = 0;
S.tClr = -1;
S.sObs = pe.s;
S.dObsHold = obs.d_obs;
if ~isfinite(S.dObsHold), S.dObsHold = 0; end
S.personSeen = obs.person_near || S.scanLost;     % implement latch (spec 2.9)
if S.scanLost                                    % before PERSON: Perception's protectiveStop is true on a lost scan
    S.holdReason = HoldReason.SENSOR_LOST;
elseif obs.person_near
    S.holdReason = HoldReason.PERSON;
elseif vfhBlocked
    S.holdReason = HoldReason.VFH_BLOCKED;
else
    S.holdReason = HoldReason.OBSTACLE;
end
end

function S = local_startMission(S, plan, nav, prm)
S.st = GncState.AUTO_MISSION;
S.missionActive = true;
S.missionDone = false;
S.faultWord = uint32(0);
S.pathSrc = PathSrc.MISSION;
S.bypassRejoin = false;
S.holdReason = HoldReason.NONE;
S.pauseReason = HoldReason.NONE;
S = local_enterSeg(S, plan, nav, 1, prm);
end

function S = local_enterSeg(S, plan, nav, k, prm)
% new mission segment k: sub-mode from its type, ALIGN when the heading error at its start exceeds
% align_enter on a segment that allows pivoting and the tilt allows it (D53)
S.seg = k;
S.dockTrim = false;
S.nxt = min(k + 1, max(1, double(plan.n_seg)));
S.pathSrc = PathSrc.MISSION;
S.srcAge = 0;
S.sub = local_subOf(local_segCol(plan, k, 1));
row = (k - 1)*prm.N_raw + 1;
th0 = double(plan.path(row, 3))*pi/180;
epsi0 = local_wrap(nav.pose(3) - th0);
tiltOk = max(abs(nav.pitch), abs(nav.roll)) <= prm.align_tilt;
if abs(epsi0) > prm.align_enter && local_segCol(plan, k, 10) > 0 && tiltOk && ...
        S.sub ~= AutoSub.DOCK_APPROACH
    S.sub = AutoSub.ALIGN;
end
end

function s = local_subOf(type)
switch round(type)
    case 1
        s = AutoSub.TRANSIT;
    case 2
        s = AutoSub.ROW;
    case 3
        s = AutoSub.TURN;
    case 4
        s = AutoSub.DOCK_APPROACH;
    otherwise
        s = AutoSub.TRANSIT;
end
end

function v = local_segCol(plan, k, c)
k = min(max(1, round(k)), size(plan.seg, 1));
v = double(plan.seg(k, c));
end

function nr = local_noRev(plan, prm)
% the fitted implement may not reverse at any slope (prm.no_rev per implement_type 0..3 from P.structure.descent.v_rev;
% the tiller, the user 2026-11-15); an unknown implement counts as one that may not
k = double(plan.implement_type) + 1;
nr = true;
if k >= 1 && k <= numel(prm.no_rev)
    nr = prm.no_rev(k) > 0;
end
end

function L = local_lastWork(plan, lastDrv)
% last ROW / TURN segment: the SOC RTH trigger is not used once the robot is on its final transit home
L = 1;
for k = 1:lastDrv
    t = double(plan.seg(k, 1));
    if t == 2 || t == 3
        L = k;
    end
end
end

function S = local_request(S, rp, goal, kind)
S.replanReq = ~S.replanReq;
S.replanGoal = goal(:);
S.replanKind = uint8(kind);
S.rpPending = true;
S.rpVer0 = rp.ver;
S.rpFail0 = rp.fail_count;
end

function S = local_requestRth(S, plan, rp, prm)
S.st = GncState.RETURN_TO_HOME;
S.sub = AutoSub.RTH_PLANNING;
S.missionActive = true;
S.pathSrc = PathSrc.MISSION;
S.bypassRejoin = false;
S.holdReason = HoldReason.NONE;
S.tries = 0;
S = local_request(S, rp, local_dockGoal(plan, prm), prm.kind_global);
end

function g = local_dockGoal(plan, prm)
% dock-approach start = first pose of the dock segment (home when the plan has no dock segment)
k = double(plan.dock_seg);
if k >= 1
    row = (k - 1)*prm.N_raw + 1;
    g = [double(plan.path(row, 1)); double(plan.path(row, 2)); double(plan.path(row, 3))*pi/180];
else
    g = plan.home(:);
end
end

function g = local_pathPoint(plan, k, s, lastDrv, prm)
% pose at arc length s along mission segment k (continuing into the following segments)
N = prm.N_raw;
k = min(max(1, k), lastDrv);
L = local_segCol(plan, k, 3);
while s > L && k < lastDrv
    s = s - L;
    k = k + 1;
    L = local_segCol(plan, k, 3);
end
u = min(max(s/max(L, 1e-6), 0), 1)*(N - 1);
i = min(floor(u), N - 2);
w = u - i;
r = (k - 1)*N + i + 1;
x = (1 - w)*double(plan.path(r, 1)) + w*double(plan.path(r + 1, 1));
y = (1 - w)*double(plan.path(r, 2)) + w*double(plan.path(r + 1, 2));
th1 = double(plan.path(r, 3))*pi/180;
th2 = double(plan.path(r + 1, 3))*pi/180;
g = [x; y; local_wrap(th1 + w*local_wrap(th2 - th1))];
end

function S = local_startBypass(S, plan, teb, rp, prm)
% H2: LOCAL Hybrid A* detour to the lane point rejoin_off past the obstacle face, followed by TEB
% (teb(1) ~= -1) or by the VFH-steered pursuit (VC_TEB = 0)
S.st = GncState.AUTO_MISSION;
S.sub = AutoSub.BYPASS;
S.holdReason = HoldReason.NONE;
S.tebMode = teb(1) ~= -1;
S.bypassRejoin = true;
S.tries = 0;
S.goalS = S.sObs + S.dObsHold + prm.x_front + prm.rejoin_off;
S.tTeb = -1;
lastDrv = max(1, double(plan.n_seg) - 1);
S = local_request(S, rp, local_pathPoint(plan, S.seg, S.goalS, lastDrv, prm), prm.kind_local);
end

function S = local_resume(S, plan, nav, pe, rp, prm)
S.holdReason = HoldReason.NONE;
S.pauseReason = HoldReason.NONE;
if S.resumeState == GncState.RETURN_TO_HOME
    if S.sub == AutoSub.DOCK_APPROACH || S.sub == AutoSub.DOCKED
        S.st = GncState.RETURN_TO_HOME;
    else
        S = local_requestRth(S, plan, rp, prm);
    end
    return
end
S.st = GncState.AUTO_MISSION;
onLane = pe.valid && abs(pe.xte) < prm.resume_xte;
headOff = abs(pe.epsi) >= prm.resume_epsi;
onReplan = S.sub == AutoSub.REJOIN || S.sub == AutoSub.BYPASS;   % pe may refer to the detour path
tiltOk = max(abs(nav.pitch), abs(nav.roll)) <= prm.align_tilt;
if onLane && headOff && ~onReplan && local_segCol(plan, S.seg, 10) > 0 && tiltOk && ...
        S.sub ~= AutoSub.DOCK_APPROACH
    % on the lane but facing away from it: pivot in place (ALIGN, then S2 back to the segment's sub-mode). The
    % forward-only LOCAL rejoin cannot turn the robot round inside the lane mask (S4 rejoin at 458.8 s, 175 deg)
    S.sub = AutoSub.ALIGN;
    S.pathSrc = PathSrc.MISSION;
    S.bypassRejoin = false;
elseif ~onLane || headOff || onReplan
    lastDrv = max(1, double(plan.n_seg) - 1);
    % a bypass interrupted by a pause (paused in BYPASS, in its TEB rejoin, or in a rejoin resumed from one) keeps
    % its rejoin in the lane: no GLOBAL fallback (lane half-width, spec 8 T7)
    S.laneOnly = S.sub == AutoSub.BYPASS || (S.sub == AutoSub.REJOIN && (S.bypassRejoin || S.laneOnly));
    S.sub = AutoSub.REJOIN;
    S.bypassRejoin = false;
    S.pathSrc = PathSrc.MISSION;
    S.tries = 0;
    S.goalS = max(pe.s, 0) + prm.rejoin_off;
    S = local_request(S, rp, local_pathPoint(plan, S.seg, S.goalS, lastDrv, prm), prm.kind_local);
end
end

% =============================================================================================
%  AUTO sub-modes (S1-S5, T10)
% =============================================================================================
function [S, tid] = local_autoSub(S, plan, nav, pe, obs, rp, teb, rpOk, rpFail, lastDrv, prm)
tid = uint8(0);
S.srcAge = S.srcAge + 1;
segLen = local_segCol(plan, S.seg, 3);
atSegEnd = S.pathSrc == PathSrc.MISSION && S.srcAge >= 2 && pe.valid && pe.s >= segLen - prm.seg_end_tol;
atRpEnd = S.pathSrc == PathSrc.REPLAN && S.srcAge >= 2 && pe.valid && pe.s_rem < prm.replan_end_tol;
switch S.sub
    case AutoSub.ALIGN
        tilted = max(abs(nav.pitch), abs(nav.roll)) > prm.align_tilt;
        if abs(pe.epsi) < prm.align_exit || tilted
            % S2
            S.sub = local_subOf(local_segCol(plan, S.seg, 1));
            tid = uint8(22);
        end
    case {AutoSub.BYPASS, AutoSub.REJOIN}
        if S.pathSrc == PathSrc.MISSION
            % waiting for the detour / rejoin path
            if rpOk
                S.pathSrc = PathSrc.REPLAN;
                S.srcAge = 0;
                S.nxt = min(S.seg + 1, max(1, double(plan.n_seg)));
                S.tTeb = -1;
            elseif rpFail && S.sub == AutoSub.REJOIN && ~S.bypassRejoin && ~S.laneOnly && ...
                    S.replanKind == uint8(prm.kind_local)
                % LOCAL rejoin failed (forward-only in the lane mask: e.g. facing away from the lane): GLOBAL plan
                % to the same goal (no lane mask, R_min) before the try counts; never for a bypass (laneOnly)
                S = local_request(S, rp, S.replanGoal, prm.kind_global);
            elseif rpFail
                S.tries = S.tries + 1;
                if S.tries >= prm.retries
                    S = local_enterPause(S, GncState.AUTO_MISSION, HoldReason.PLANNER_FAIL);
                    tid = uint8(24);
                else
                    S.goalS = S.goalS + prm.rejoin_off;
                    S = local_request(S, rp, local_pathPoint(plan, S.seg, S.goalS, lastDrv, prm), prm.kind_local);
                end
            end
        else
            tebBad = S.sub == AutoSub.BYPASS && S.tebMode && S.srcAge >= 2 && ...
                (teb(1) == 1 || teb(2) < prm.teb_lfi_min);
            [S.tTeb, tebFail] = local_timer(S.tTeb, tebBad, prm.teb_t_bad, prm.Ts);
            if atRpEnd && (S.sub == AutoSub.REJOIN || obs.d_obs > prm.d_release)
                % S3 (BYPASS) / S5 (REJOIN)
                if S.sub == AutoSub.BYPASS, tid = uint8(23); else, tid = uint8(25); end
                S.pathSrc = PathSrc.MISSION;
                S.srcAge = 0;
                S.nxt = min(S.seg + 1, max(1, double(plan.n_seg)));
                S.sub = local_subOf(local_segCol(plan, S.seg, 1));
                S.bypassRejoin = false;
            elseif tebFail
                % S4: TEB infeasible -> REJOIN on a new LOCAL detour from the current pose
                S.sub = AutoSub.REJOIN;
                S.pathSrc = PathSrc.MISSION;
                S.srcAge = 0;
                S.tries = 0;
                S.goalS = S.goalS + prm.rejoin_off;
                S = local_request(S, rp, local_pathPoint(plan, S.seg, S.goalS, lastDrv, prm), prm.kind_local);
                tid = uint8(24);
            end
        end
    otherwise
        if S.sub == AutoSub.DOCK_APPROACH
            [S, atSegEnd] = local_dockTrim(S, atSegEnd, pe, prm);
        end
        if atSegEnd
            % S1
            if S.seg >= lastDrv
                % T10: mission complete
                S.missionDone = true;
                if round(local_segCol(plan, S.seg, 1)) == 4
                    S.st = GncState.RETURN_TO_HOME;
                    S.sub = AutoSub.DOCKED;
                    S.tDocked = 0;
                elseif plan.end_action == uint8(1) && plan.dock_seg >= uint16(1)
                    S = local_requestRth(S, plan, rp, prm);
                else
                    S = local_toIdle(S);
                    S.missionDone = true;
                end
                tid = uint8(20);
            else
                S = local_enterSeg(S, plan, nav, S.seg + 1, prm);
                tid = uint8(21);
            end
        end
end
end

% =============================================================================================
%  RETURN_TO_HOME sub-modes (R1-R5)
% =============================================================================================
function [S, tid] = local_rthSub(S, plan, mcu, pe, rpOk, rpFail, rp, prm)
tid = uint8(0);
S.srcAge = S.srcAge + 1;
switch S.sub
    case AutoSub.RTH_PLANNING
        if rpOk
            % R1
            S.sub = AutoSub.RTH_TRANSIT;
            S.pathSrc = PathSrc.REPLAN;
            S.srcAge = 0;
            if plan.dock_seg >= uint16(1)
                S.nxt = double(plan.dock_seg);
                % the window segment index of the RTH path is the transit before the dock approach, so the
                % governor / profiler use its transit speed and end at the dock-approach speed
                kd = double(plan.dock_seg) - 1;
                if kd >= 1 && round(local_segCol(plan, kd, 1)) == 1
                    S.seg = kd;
                end
            end
            tid = uint8(51);
        elseif rpFail
            S.tries = S.tries + 1;
            if S.tries >= prm.retries
                % R2
                S = local_enterPause(S, GncState.RETURN_TO_HOME, HoldReason.PLANNER_FAIL);
                tid = uint8(52);
            else
                S = local_request(S, rp, local_dockGoal(plan, prm), prm.kind_global);
            end
        end
    case AutoSub.RTH_TRANSIT
        if S.srcAge >= 2 && pe.valid && pe.s_rem < prm.replan_end_tol
            % R3
            S.pathSrc = PathSrc.MISSION;
            S.srcAge = 0;
            if plan.dock_seg >= uint16(1)
                S.seg = double(plan.dock_seg);
                S.nxt = min(S.seg + 1, max(1, double(plan.n_seg)));
                S.sub = AutoSub.DOCK_APPROACH;
                S.dockTrim = false;
            else
                S.sub = AutoSub.DOCKED;
                S.tDocked = 0;
            end
            tid = uint8(53);
        end
    case AutoSub.DOCK_APPROACH
        atEnd = S.srcAge >= 2 && pe.valid && pe.s >= local_segCol(plan, S.seg, 3) - prm.seg_end_tol;
        [S, done] = local_dockTrim(S, atEnd, pe, prm);
        if done
            % R4
            S.sub = AutoSub.DOCKED;
            S.tDocked = 0;
            tid = uint8(54);
        end
    case AutoSub.DOCKED
        S.tDocked = S.tDocked + prm.Ts;
        if mcu.charging || S.tDocked >= prm.t_docked - 1e-9
            % R5
            S = local_toIdle(S);
            S.missionDone = true;
            tid = uint8(55);
        end
    otherwise
        S = local_requestRth(S, plan, rp, prm);
end
end

function [S, done] = local_dockTrim(S, atEnd, pe, prm)
% end of the dock-approach segment: when the heading error exceeds dock_tol_yaw the robot pivots in place
% (ALIGN tracker) until it is below dock_tol_yaw / 2, then the dock is reached (R4 / T10)
done = false;
if ~atEnd
    return
end
if S.dockTrim
    done = abs(pe.epsi) < prm.dock_tol_yaw/2;
elseif abs(pe.epsi) > prm.dock_tol_yaw
    S.dockTrim = true;
else
    done = true;
end
if done
    S.dockTrim = false;
end
end

% =============================================================================================
%  outputs
% =============================================================================================
function fsm = local_pack(S, plan, nav, tid, prm)
st = S.st;
sub = S.sub;
isAuto = st == GncState.AUTO_MISSION;
isRth = st == GncState.RETURN_TO_HOME;
% tracker and mode speed cap
trk = TrackerSel.STOP;
vcap = 0;
if isAuto || isRth
    switch sub
        case AutoSub.ALIGN
            trk = TrackerSel.ALIGN;
        case AutoSub.TRANSIT
            trk = TrackerSel.PP;  vcap = prm.v_transit;
        case AutoSub.ROW
            trk = TrackerSel.ROW; vcap = prm.v_row;
        case AutoSub.TURN
            trk = TrackerSel.PP;  vcap = prm.v_turn;
        case {AutoSub.BYPASS, AutoSub.REJOIN}
            if S.pathSrc == PathSrc.REPLAN
                if S.bypassRejoin
                    trk = TrackerSel.TEB;
                else
                    trk = TrackerSel.PP;
                end
                vcap = prm.v_bypass;
            end
        case AutoSub.RTH_TRANSIT
            trk = TrackerSel.PP;  vcap = prm.v_transit;
        case AutoSub.DOCK_APPROACH
            trk = TrackerSel.ROW; vcap = prm.v_dock;
            if S.dockTrim, trk = TrackerSel.ALIGN; end
        otherwise
            trk = TrackerSel.STOP;
    end
end
if S.implWait
    vcap = 0;
end
% mower deck not yet down (deckUp): at standstill no motion at all, no pivot either (STOP tracker, v_cap 0), while the
% MCU lowers it; already moving, the deck-raised rows of the descent cap (as the MCU's backstop) bound the governor
% through v_cap until the robot is still
deckWait = S.deckUp && (isAuto || isRth);
if deckWait
    if abs(nav.v) <= prm.v_stuck_meas
        trk = TrackerSel.STOP;
        vcap = 0;
    else
        vcap = min(vcap, gnc_descentCap(nav.pitch, plan.implement_type, false, prm.desc, true));
    end
end
cmdValid = isAuto || (isRth && sub ~= AutoSub.RTH_PLANNING && sub ~= AutoSub.DOCKED);
switch st
    case GncState.IDLE
        mreq = GncModeReq.STOP;
    case GncState.MANUAL
        mreq = GncModeReq.MANUAL_ACK;
    case {GncState.AUTO_MISSION, GncState.RETURN_TO_HOME}
        mreq = GncModeReq.AUTO;
    case {GncState.PAUSE, GncState.OBSTACLE_HOLD}
        mreq = GncModeReq.HOLD;
    otherwise
        mreq = GncModeReq.FAULT_STOP;
end
implAllow = (isAuto && (sub == AutoSub.ROW || sub == AutoSub.TURN)) || ...
    (st == GncState.OBSTACLE_HOLD && S.hist == GncState.AUTO_MISSION && ...
     (S.histSub == AutoSub.ROW || S.histSub == AutoSub.TURN) && ~S.personSeen);
subOut = sub;
if st == GncState.OBSTACLE_HOLD
    subOut = S.histSub;
end
holdR = S.holdReason;
if st == GncState.OBSTACLE_HOLD && S.scanLost
    holdR = HoldReason.SENSOR_LOST;              % while the scan is lost, whatever started the hold
end
if (S.implWait && isAuto) || deckWait
    holdR = HoldReason.IMPLEMENT_NOT_READY;
end
pauseR = HoldReason.NONE;
if st == GncState.PAUSE
    pauseR = S.pauseReason;
end
nSeg = max(1, double(plan.n_seg));
nxt = min(S.seg + 1, nSeg);
if S.pathSrc == PathSrc.REPLAN || isRth
    nxt = min(max(1, S.nxt), nSeg);
end
fsm = struct( ...
    'state', uint8(st), ...
    'sub', uint8(subOut), ...
    'seg_idx', uint16(min(max(1, S.seg), nSeg)), ...
    'nxt_idx', uint16(nxt), ...
    'path_src', uint8(S.pathSrc), ...
    'trk_sel', uint8(trk), ...
    'mode_req', uint8(mreq), ...
    'cmd_valid', cmdValid, ...
    'gnc_ready', nav.nav_ok && plan.valid && S.faultWord == uint32(0) && st ~= GncState.FAULT, ...
    'impl_allow', implAllow, ...
    'replan_req', S.replanReq, ...
    'replan_goal', S.replanGoal, ...
    'replan_kind', S.replanKind, ...
    'hold_reason', uint8(holdR), ...
    'pause_reason', uint8(pauseR), ...
    'mission_done', S.missionDone, ...
    'teb_enable', isAuto && (sub == AutoSub.BYPASS || (sub == AutoSub.REJOIN && S.bypassRejoin)) && ...
        S.pathSrc == PathSrc.REPLAN && S.tebMode, ...
    'v_cap', vcap, ...
    's_obs', S.sObs, ...
    't_hold', S.tHold*double(st == GncState.OBSTACLE_HOLD), ...
    'fault_word', S.faultWord, ...
    'trans_id', tid);
end

% =============================================================================================
%  geometry
% =============================================================================================
function in = local_inside(plan, x, y)
% point-in-polygon (crossing number) on the geofence plan.boundary(1:n_bnd, :); no geofence below 3 vertices
nb = double(plan.n_bnd);
in = true;
if nb < 3
    return
end
c = false;
j = nb;
for i = 1:nb
    xi = plan.boundary(i, 1); yi = plan.boundary(i, 2);
    xj = plan.boundary(j, 1); yj = plan.boundary(j, 2);
    if (yi > y) ~= (yj > y)
        xc = (xj - xi)*(y - yi)/(yj - yi) + xi;
        if x < xc
            c = ~c;
        end
    end
    j = i;
end
in = c;
end

function a = local_wrap(a)
a = a - 2*pi*floor((a + pi)/(2*pi));
end
