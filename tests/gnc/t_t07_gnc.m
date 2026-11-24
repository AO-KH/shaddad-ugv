classdef t_t07_gnc < matlab.unittest.TestCase
%T_T07_GNC  Card T-07 step 7 (GNC, pass B): mode-logic tests of the mission state machine (gnc_missionFSM.m) for the
%   transitions that no test exercised before (pass A, findings T07-GNC-04 and T07-GNC-05), and the VFH-blocked hold.
%
%   runtests('t_t07_gnc')
%
%   Each test drives the state machine through the scripted driver tests/gnc/gnc_fsm_driver.m (MATLAB level, 0.05 s
%   steps, the path error emulated) and checks the state, the sub-mode, the reason and Bus_FsmOut.trans_id of the
%   transition. Two transitions cannot fire with the plans gnc_compileMission makes (finding T07-GNC-06): S2c (turn
%   segments never allow a pivot) and T10r (a plan with a dock always ends with its dock approach); their tests edit
%   one column of the compiled plan to reach the code. Card T-07 step 7, the GNC engineer.

    properties
        P
        plan      % t_gnc_mission's scenario plan: two 30 m rows, a turn, a dock, end action RTH, bypass allowed
        planTr    % the same with home 6 m before the first row (inside world 1's geofence): it begins with a transit
        planND    % the same rows with no dock (end action STOP)
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.P = shaddad_params('Assign', false);
            tc.plan = t_gnc_mission.fsmPlan(tc.P);
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 30 0; 30 4 0 4], 'home_enu', [-6 0 0], ...
                'speed_work', 0.8, 'turn_radius_m', 2, 'skip_rows', 0, 'dock_enu', [-4 -4 0], 'end_action', 'RTH', ...
                'allow_bypass', true, 'lane_halfwidth_m', 2.5);
            tc.planTr = gnc_compileMission(tc.P, S, 'Validate', false);
            S = rmfield(S, {'dock_enu'});
            S.home_enu = [0 0 0];
            S.end_action = 'STOP';
            tc.planND = gnc_compileMission(tc.P, S, 'Validate', false);
        end
    end

    methods (Test, TestTags = {'T07'})
        function testT3iManualToIdle(tc)
            % T3i: the switch away from MANUAL with no mission active -> IDLE (trans_id 13)
            F = gnc_fsm_driver(tc.P, tc.plan);
            F.set('mcu.rc_mode_sw', uint8(0));  F.run(2);
            tc.verifyEqual(F.st, GncState.MANUAL, 'G2 from IDLE');
            F.set('mcu.rc_mode_sw', uint8(2));  F.run(1);
            tc.verifyEqual(F.st, GncState.IDLE);
            tc.verifyEqual(double(F.out.trans_id), 13);
            tc.verifyLast(F, GncState.MANUAL, GncState.IDLE, 13);
        end

        function testT10rEndRth(tc)
            % T10r: the end of the last driven segment, not a dock approach, with end_action RTH and a dock segment
            % -> RETURN_TO_HOME (RTH_PLANNING), trans_id 20. Compiled plans end with the dock approach (T07-GNC-06), so
            % the test gives the plan's last driven segment the type ROW.
            plan = tc.plan;
            last = double(plan.n_seg) - 1;
            tc.assertEqual(double(plan.seg(last, 1)), double(SegType.DOCK), 'the compiled plan ends with its dock approach');
            plan.seg(last, 1) = double(SegType.ROW);
            F = gnc_fsm_driver(tc.P, plan);
            F.startAuto();
            F.drive(8000, @(f) f.out.trans_id == 20);
            tc.verifyEqual(double(F.out.trans_id), 20);
            tc.verifyEqual(F.st, GncState.RETURN_TO_HOME);
            tc.verifyEqual(F.sub, AutoSub.RTH_PLANNING);
            tc.verifyTrue(logical(F.out.mission_done));
        end

        function testR7P1rPauseResumeRth(tc)
            % R7: the operator's PAUSE in RETURN_TO_HOME -> PAUSE (OPERATOR, 16); P1r: RESUME -> RETURN_TO_HOME, the
            % return planned again away from the dock approach (41)
            F = tc.inRthTransit(tc.plan);
            F.cmd(MissionCmd.PAUSE);  F.run(1);
            tc.verifyEqual(F.st, GncState.PAUSE);
            tc.verifyEqual(double(F.out.pause_reason), double(HoldReason.OPERATOR));
            tc.verifyLast(F, GncState.RETURN_TO_HOME, GncState.PAUSE, 16);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            tc.verifyEqual(F.st, GncState.RETURN_TO_HOME);
            tc.verifyEqual(F.sub, AutoSub.RTH_PLANNING);
            tc.verifyLast(F, GncState.PAUSE, GncState.RETURN_TO_HOME, 41);
        end

        function testR8P2rNavPauseResumeRth(tc)
            % R8: nav_degraded for 2.0 s in RETURN_TO_HOME -> PAUSE (NAV_DEGRADED, 17); P2r: 5 s of good navigation ->
            % RETURN_TO_HOME (42)
            F = tc.inRthTransit(tc.plan);
            F.set('nav.nav_degraded', true);
            n = F.runUntil(@(f) f.st == GncState.PAUSE, 100);
            tc.verifyEqual(n*0.05, tc.P.gnc.fsm.t_nav_degr, 'AbsTol', 0.051, 'R8 after t_nav_degr');
            tc.verifyEqual(double(F.out.pause_reason), double(HoldReason.NAV_DEGRADED));
            tc.verifyLast(F, GncState.RETURN_TO_HOME, GncState.PAUSE, 17);
            F.set('nav.nav_degraded', false);
            n = F.runUntil(@(f) f.st ~= GncState.PAUSE, 200);
            tc.verifyEqual(n*0.05, tc.P.gnc.fsm.t_nav_rec, 'AbsTol', 0.051, 'P2r after t_nav_rec');
            tc.verifyEqual(F.st, GncState.RETURN_TO_HOME);
            tc.verifyLast(F, GncState.PAUSE, GncState.RETURN_TO_HOME, 42);
        end

        function testH6H9HoldPauseAbort(tc)
            % H6: the operator's PAUSE in OBSTACLE_HOLD -> PAUSE (OPERATOR, 16); H9: ABORT in OBSTACLE_HOLD -> IDLE (19)
            F = tc.inHold(tc.plan);
            F.cmd(MissionCmd.PAUSE);  F.run(1);
            tc.verifyEqual(F.st, GncState.PAUSE);
            tc.verifyEqual(double(F.out.pause_reason), double(HoldReason.OPERATOR));
            tc.verifyLast(F, GncState.OBSTACLE_HOLD, GncState.PAUSE, 16);
            F = tc.inHold(tc.plan);
            F.cmd(MissionCmd.ABORT);  F.run(1);
            tc.verifyEqual(F.st, GncState.IDLE);
            tc.verifyLast(F, GncState.OBSTACLE_HOLD, GncState.IDLE, 19);
        end

        function testR9RthAbort(tc)
            % R9: ABORT in RETURN_TO_HOME -> IDLE (19)
            F = tc.inRthTransit(tc.plan);
            F.cmd(MissionCmd.ABORT);  F.run(1);
            tc.verifyEqual(F.st, GncState.IDLE);
            tc.verifyEqual(F.sub, AutoSub.NONE);
            tc.verifyLast(F, GncState.RETURN_TO_HOME, GncState.IDLE, 19);
        end

        function testS2aAlignToTransit(tc)
            % SE5 then S2a: the mission begins with a transit facing away from it -> ALIGN; |e_psi| below align_exit
            % -> TRANSIT (22)
            plan = tc.planTr;
            tc.assertEqual(double(plan.seg(1, 1)), double(SegType.TRANSIT), 'the plan begins with a transit');
            tc.assertGreaterThan(double(plan.seg(1, 10)), 0, 'the transit allows a pivot');
            F = gnc_fsm_driver(tc.P, plan);
            F.set('nav.pose', [-6; 0; pi]);  F.set('pe.epsi', pi);
            F.startAuto();
            F.run(1);
            tc.verifyEqual(F.sub, AutoSub.ALIGN, 'ALIGN at the start');
            F.set('pe.epsi', 0);  F.run(1);
            tc.verifyEqual(F.sub, AutoSub.TRANSIT);
            tc.verifyEqual(double(F.out.trans_id), 22);
        end

        function testS2cAlignToTurn(tc)
            % S2c: ALIGN ends into a TURN (22). Turn segments never allow a pivot (T07-GNC-06): the test gives the
            % turn segment's column 10 the value 1 and points the robot away from the turn at its start
            plan = tc.plan;
            kT = find(plan.seg(1:plan.n_seg, 1) == double(SegType.TURN), 1);
            tc.assertNotEmpty(kT, 'the plan has a turn');
            tc.assertEqual(double(plan.seg(kT, 10)), 0, 'compiled turns allow no pivot');
            plan.seg(kT, 10) = 1;
            F = gnc_fsm_driver(tc.P, plan);
            F.startAuto();
            F.drive(4000, @(f) double(f.out.seg_idx) == kT - 1);
            F.set('nav.pose', [0; 0; pi]);  F.set('pe.epsi', pi);
            F.drive(4000, @(f) f.sub == AutoSub.ALIGN);
            tc.verifyEqual(F.sub, AutoSub.ALIGN);
            tc.verifyEqual(double(F.out.seg_idx), kT);
            F.set('pe.epsi', 0);  F.set('nav.pose', [0; 0; 0]);  F.run(1);
            tc.verifyEqual(F.sub, AutoSub.TURN);
            tc.verifyEqual(double(F.out.trans_id), 22);
        end

        function testS5aRejoinToTransit(tc)
            % P1j then S5a: a resume off the lane on a transit -> REJOIN; the end of the replanned path -> TRANSIT (25)
            F = gnc_fsm_driver(tc.P, tc.planTr);
            F.set('nav.pose', [-6; 0; 0]);
            F.startAuto();
            tc.assertEqual(F.sub, AutoSub.TRANSIT);
            F.run(5);
            tc.rejoin(F, AutoSub.TRANSIT);
        end

        function testS5cRejoinToTurn(tc)
            % S5c: a resume off the lane on a turn -> REJOIN -> TURN (25)
            F = gnc_fsm_driver(tc.P, tc.plan);
            F.startAuto();
            F.drive(4000, @(f) f.sub == AutoSub.TURN);
            tc.assertEqual(F.sub, AutoSub.TURN);
            F.run(5);
            tc.rejoin(F, AutoSub.TURN);
        end

        function testS5dRejoinToDock(tc)
            % S5d: a resume off the lane on the dock approach -> REJOIN -> DOCK_APPROACH (25)
            F = gnc_fsm_driver(tc.P, tc.plan);
            F.startAuto();
            F.drive(8000, @(f) f.sub == AutoSub.DOCK_APPROACH);
            tc.assertEqual(F.sub, AutoSub.DOCK_APPROACH);
            F.run(5);
            tc.rejoin(F, AutoSub.DOCK_APPROACH);
        end

        function testR3dRthNoDock(tc)
            % T2, R1, then R3d: a return in a plan with no dock segment ends DOCKED at home (53); R5 -> IDLE
            F = gnc_fsm_driver(tc.P, tc.planND);
            tc.assertEqual(double(tc.planND.dock_seg), 0, 'no dock segment');
            F.cmd(MissionCmd.RTH);  F.run(2);
            tc.verifyEqual(F.st, GncState.RETURN_TO_HOME);
            F.rpAnswer(true);
            tc.verifyEqual(F.sub, AutoSub.RTH_TRANSIT);
            F.drive(2000, @(f) f.sub == AutoSub.DOCKED || f.st ~= GncState.RETURN_TO_HOME);
            tc.verifyEqual(F.sub, AutoSub.DOCKED);
            tc.verifyTrue(any(F.log(:, 3) == 53), 'R3d fired (trans_id 53)');
            F.run(round(tc.P.gnc.fsm.t_docked/0.05) + 2);
            tc.verifyEqual(F.st, GncState.IDLE, 'R5 after t_docked');
        end

        function testHR3VfhBlockedHold(tc)
            % T5 with the VFH blocked (T07-GNC-04): blocked_vfh for t_vfh 0.2 s on a ROW -> OBSTACLE_HOLD with hold
            % reason VFH_BLOCKED (15); and in RETURN_TO_HOME -> OBSTACLE_HOLD (R6, 56)
            F = gnc_fsm_driver(tc.P, tc.plan);
            F.startAuto();
            F.drive(4000, @(f) f.sub == AutoSub.ROW);
            F.run(10);
            F.set('obs.blocked_vfh', true);
            n = F.runUntil(@(f) f.st == GncState.OBSTACLE_HOLD, 40);
            tc.verifyEqual(n*0.05, tc.P.gnc.fsm.t_vfh, 'AbsTol', 0.051, 'the hold after t_vfh');
            tc.verifyEqual(double(F.out.hold_reason), double(HoldReason.VFH_BLOCKED));
            tc.verifyLast(F, GncState.AUTO_MISSION, GncState.OBSTACLE_HOLD, 15);
            F = tc.inRthTransit(tc.plan);
            F.set('obs.blocked_vfh', true);
            F.runUntil(@(f) f.st == GncState.OBSTACLE_HOLD, 40);
            tc.verifyEqual(F.st, GncState.OBSTACLE_HOLD);
            tc.verifyEqual(double(F.out.hold_reason), double(HoldReason.VFH_BLOCKED));
            tc.verifyLast(F, GncState.RETURN_TO_HOME, GncState.OBSTACLE_HOLD, 56);
        end

        function testPR6ScanLostPause(tc)
            % H3 with the scan lost: a hold for a lost scan that lasts t_hold_max 120 s -> PAUSE with pause reason
            % SENSOR_LOST (33)
            F = gnc_fsm_driver(tc.P, tc.plan);
            F.startAuto();
            F.drive(4000, @(f) f.sub == AutoSub.ROW);
            F.run(10);
            F.set('scanOk', false);
            F.runUntil(@(f) f.st == GncState.OBSTACLE_HOLD, 40);
            tc.verifyEqual(double(F.out.hold_reason), double(HoldReason.SENSOR_LOST));
            n = F.runUntil(@(f) f.st ~= GncState.OBSTACLE_HOLD, 3000);
            tc.verifyEqual(n*0.05, tc.P.gnc.fsm.t_hold_max, 'AbsTol', 0.051, 'H3 after t_hold_max');
            tc.verifyEqual(F.st, GncState.PAUSE);
            tc.verifyEqual(double(F.out.pause_reason), double(HoldReason.SENSOR_LOST));
            tc.verifyLast(F, GncState.OBSTACLE_HOLD, GncState.PAUSE, 33);
        end
    end

    methods
        function F = inRthTransit(tc, plan)
            % a mission started, then the operator's RTH, the plan home received: RETURN_TO_HOME / RTH_TRANSIT
            F = gnc_fsm_driver(tc.P, plan);
            F.startAuto();
            F.drive(20);
            F.cmd(MissionCmd.RTH);  F.run(2);
            tc.assertEqual(F.st, GncState.RETURN_TO_HOME);
            F.rpAnswer(true);
            tc.assertEqual(F.sub, AutoSub.RTH_TRANSIT);
            F.run(5);
        end

        function F = inHold(tc, plan)
            % a mission started, an obstacle 1.0 m ahead on the first row: OBSTACLE_HOLD
            F = gnc_fsm_driver(tc.P, plan);
            F.startAuto();
            F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(2);
            tc.assertEqual(F.st, GncState.OBSTACLE_HOLD);
        end

        function rejoin(tc, F, sub)
            % the operator's PAUSE, a resume 0.5 m off the lane -> REJOIN (P1j), the replanned path driven to its end
            % -> the segment's own sub-mode (S5, trans_id 25)
            F.cmd(MissionCmd.PAUSE);  F.run(2);
            tc.assertEqual(F.st, GncState.PAUSE);
            F.set('pe.xte', 0.5);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            tc.verifyEqual(F.sub, AutoSub.REJOIN, 'P1j: REJOIN');
            F.set('pe.xte', 0);
            F.rpAnswer(true);
            F.drive(600, @(f) f.sub ~= AutoSub.REJOIN);
            tc.verifyEqual(F.sub, sub);
            tc.verifyTrue(any(F.log(:, 3) == 25), 'S5 fired (trans_id 25)');
        end

        function verifyLast(tc, F, from, to, tid)
            % the last change of state in the driver log went from 'from' to 'to' with transition id tid
            L = F.log;
            k = find(L(2:end, 1) ~= L(1:end - 1, 1), 1, 'last') + 1;
            tc.verifyNotEmpty(k, 'a change of state');
            if isempty(k), return, end
            tc.verifyEqual([L(k - 1, 1) L(k, 1) L(k, 3)], [double(from) double(to) tid], ...
                'the last change of state: [from to trans_id]');
        end
    end
end
