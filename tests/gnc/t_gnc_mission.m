classdef t_gnc_mission < matlab.unittest.TestCase
%T_GNC_MISSION  Acceptance tests of build task T15 (docs/build_plan.md T15, docs/specs/gnc.md 2.6, 2.8, 2.9, 4.1,
%   4.4, 8: T6-T18, T20).
%
%   Bench tests/gnc/tb_gnc_mission.slx (scripts/build/build_gnc_mission.m): Shaddad_GNC (Normal) + kinematic plant
%   with 0.1 s dead time, MCU AUTO shaper and v / yaw-rate lags, RTK noise, scenario tables (MCU, mission command,
%   BMS, nav, plant), harness LiDAR gnc_harnessLidar (rangeSensor on P.world.occ + dynamic obstacles). Runs go
%   through tests/gnc/gnc_mission_run.m (Simulink.SimulationInput.setVariable only: the shared base workspace is
%   never modified; it must hold Ts, Bus_* and cs_SimFixed). FSM fields are read at 0.05 s from the signal 'fsm'
%   logged inside Shaddad_GNC/MissionManager. Every measured number is printed as 'T15 MEASURED <test>: ...'.
%   T11 drives gnc_missionFSM directly (MATLAB) through scripted scenarios and logs the transition matrix.
%   Accelerator (T20 >= 50x real time) and code generation are BLOCKED: no C compiler / Simulink Coder (D31).

    properties
        P1      % world 1 flat_test
    end

    methods (TestClassSetup)
        function setupBase(tc)
            if ~evalin('base', 'exist(''Ts'',''var'') && exist(''Bus_FsmOut'',''var'') && exist(''VC_PHASE'',''var'')')
                shaddad_params();
            end
            if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
                shaddad_configsets('fixed');
            end
            tc.P1 = shaddad_params('Assign', false);
            tc.applyFixture(matlab.unittest.fixtures.SuppressedWarningsFixture( ...
                {'Simulink:blocks:BusSelectorOutSignalNameUpdate', 'Simulink:Engine:SaveWithParameterizedLinks_Warning', ...
                 'gnc_compileMission:check', 'gnc_compileMission:speed', 'Simulink:Engine:UINotUpdatedDuringRTW'}));
            root = char(currentProject().RootFolder);
            if ~isfile(fullfile(root, 'tests', 'gnc', 'tb_gnc_mission.slx')), build_gnc_mission('tb_gnc_mission'); end
        end
    end

    methods (TestClassTeardown)
        function closeModels(~)
            for m = {'tb_gnc_mission'}
                if bdIsLoaded(m{1}), close_system(m{1}, 0); end
            end
        end
    end

    %% ================================ files / interfaces =====================================
    methods (Test)
        function testSsrPortContract(tc)
            % the five T15 Subsystem References keep the T14 port contract and are no longer stubs
            T = build_shaddad_gnc('ports');
            for n = {'gnc_MissionManager', 'gnc_LocalAvoidance', 'gnc_GlobalReplanner', 'gnc_ImplementManager', 'gnc_StatusMetrics'}
                wasLoaded = bdIsLoaded(n{1});
                load_system(n{1});
                for d = {'Inport', 'Outport'}
                    b = find_system(n{1}, 'SearchDepth', 1, 'BlockType', d{1});
                    [~, o] = sort(cellfun(@(x) str2double(get_param(x, 'Port')), b));
                    if strcmp(d{1}, 'Inport'), ref = T.(n{1}).in; else, ref = T.(n{1}).out; end
                    tc.verifyEqual(reshape(get_param(b(o), 'Name'), 1, []), {ref.name}, [n{1} ' ' d{1} ' names']);
                    tc.verifyEqual(reshape(get_param(b(o), 'OutDataTypeStr'), 1, []), {ref.type}, [n{1} ' ' d{1} ' types']);
                end
                tc.verifyFalse(startsWith(strtrim(get_param(n{1}, 'Description')), 'SHADDAD_GNC_STUB'), n{1});
                if ~wasLoaded, close_system(n{1}, 0); end
            end
            % MATLAB Function blocks of the task (only where no block exists, build plan T15)
            mf = {'gnc_MissionManager/MissionFSM', 'gnc_GlobalReplanner/Replan/HybridAStar'};
            for k = 1:numel(mf)
                load_system(extractBefore(mf{k}, '/'));
                tc.verifyTrue(contains(get_param(mf{k}, 'MATLABFunctionConfiguration').FunctionScript, ...
                    {'gnc_missionFSM', 'gnc_globalReplanner'}), mf{k});
            end
            tc.report('ssr_ports', '5 T15 Subsystem References match build_shaddad_gnc(''ports''), none is a stub');
        end

        function testInterfacesAndLoops(tc)
            n = shaddad_check_interfaces('Files', {'Shaddad_GNC', 'gnc_MissionManager', 'gnc_LocalAvoidance', ...
                'gnc_GlobalReplanner', 'gnc_ImplementManager', 'gnc_StatusMetrics'}, 'Quiet', true);
            tc.verifyEqual(n, 0);
            if ~evalin('base', 'exist(''mpcRowObj'',''var'')'), evalin('base', 'gnc_designMPC(P);'); end
            msg = {};
            for vt = [1 0]
                m = 'tb_gnc_mission';
                load_system(m);
                in = Simulink.SimulationInput(m);
                P = tc.P1;
                in = in.setVariable('P', P); in = in.setVariable('VC_TEB', vt);
                % compile through a 0-length simulation (applies the variables) and count algebraic loops
                in = in.setModelParameter('StopTime', '0');
                sim(in);
                assignin('base', 'tmp_t15_vc', evalin('base', 'VC_TEB'));
                try
                    evalin('base', sprintf('VC_TEB = %d;', vt));
                    loops = Simulink.BlockDiagram.getAlgebraicLoops(m);
                    evalin('base', 'VC_TEB = tmp_t15_vc; clear tmp_t15_vc');
                catch err
                    evalin('base', 'VC_TEB = tmp_t15_vc; clear tmp_t15_vc');
                    rethrow(err);
                end
                tc.verifyEmpty(loops, sprintf('algebraic loops with VC_TEB = %d', vt));
                msg{end+1} = sprintf('VC_TEB %d: %d loops', vt, numel(loops)); %#ok<AGROW>
            end
            tc.report('interfaces', sprintf('check_interfaces %d; bench %s', n, strjoin(msg, ', ')));
        end

        function testCodegenCompanion(tc)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
        end

        %% ================================ readiness / start ===================================
        function testReadyAndStartGating(tc)
            % gnc_ready true in IDLE with valid plan and nav; START ignored without drive_ready; HoldReason
            % RC_LOSS when drive_ready drops with rc_link_ok false (D44)
            plan = tc.rowPlan(40, 0.8);
            mcu = [0 2 1 0 0 0 0 0; 4 2 1 1 0 0 0 0; 12 2 0 0 0 0 0 0];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 15, 'mcu', mcu, 'cmd', [2 2; 6 2]);
            i2 = r.tf >= 2.5 & r.tf < 6;
            readyIdle = all(r.f_gnc_ready(r.tf >= 0.5 & r.tf < 6) == 1);
            ignored = all(r.f_state(i2) == double(GncState.IDLE));
            tStart = tc.first(r, r.f_state == double(GncState.AUTO_MISSION));
            tPause = tc.first(r, r.tf > 12 & r.f_state == double(GncState.PAUSE));
            hr = r.f_pause_reason(find(r.tf >= tPause, 1));
            tc.report('ready_start', sprintf(['gnc_ready in IDLE %d; START at 2 s without drive_ready ignored %d; START at 6 s ' ...
                'with drive_ready -> AUTO at %.2f s; drive_ready + rc_link drop at 12 s -> PAUSE at %.2f s, reason %d (RC_LOSS %d)'], ...
                readyIdle, ignored, tStart, tPause, hr, double(HoldReason.RC_LOSS)));
            tc.verifyTrue(readyIdle);
            tc.verifyTrue(ignored);
            tc.verifyGreaterThan(tStart, 6);
            tc.verifyLessThan(tStart, 6.5);
            tc.verifyEqual(hr, double(HoldReason.RC_LOSS));
            tc.verifyLessThanOrEqual(tPause - 12, 0.1);
        end

        %% ================================ T6 static box =======================================
        function testT6StaticBox(tc)
            plan = tc.rowPlan(40, 0.8, 'MOWER');
            box = [2 10.25 0 0.5 0.5 0];
            r0 = gnc_mission_run(tc.P1, plan, 'StopTime', 20, 'cmd', [1 2], 'dyn', [box 0 1e5]);
            tH0 = tc.first(r0, r0.f_state == double(GncState.OBSTACLE_HOLD));
            tOff = round((tH0 + 5)*10)/10;                          % removed 5 s after the hold (scan grid)
            r = gnc_mission_run(tc.P1, plan, 'StopTime', tOff + 8, 'cmd', [1 2], 'dyn', [box 0 tOff]);
            tD = r.tc(find(r.d_obs < tc.P1.gnc.obs.d_stop, 1));
            tH = tc.first(r, r.f_state == double(GncState.OBSTACLE_HOLD));
            tM = r.tc(find(r.tc > tH & r.mower_enable == 0, 1));
            tR = tc.first(r, r.tf > tH & r.f_state == double(GncState.AUTO_MISSION));
            front = 10.0 - (r.truth(:, 1) + tc.P1.gnc.geom.x_front);
            dStop = front(find(r.t >= tOff, 1));
            dMin = min(front(r.t < tOff));
            tc.report('T6', sprintf(['d_obs < 1.5 at %.2f s, OBSTACLE_HOLD at %.2f s (+%.2f s); stop front-to-box %.3f m; ' ...
                'min clearance %.3f m; mower off %.2f s after hold; box removed %.1f s, resume %.2f s after removal'], ...
                tD, tH, tH - tD, dStop, dMin, tM - tH, tOff, tR - tOff));
            tc.verifyLessThanOrEqual(tH - tD, 0.1 + 1e-9);
            tc.verifyGreaterThanOrEqual(dStop, 1.2);
            tc.verifyGreaterThan(dMin, 1.0);
            tc.verifyLessThanOrEqual(abs(tM - tH - 3), 0.05 + 1e-9);
            tc.verifyLessThanOrEqual(abs(tR - tOff - 2.0), 0.1 + 1e-9);
        end

        function testT6SprayOff(tc)
            % sprayer: spray off within 0.05 s of OBSTACLE_HOLD; section mask = vision mask AND enable; Q law
            plan = tc.rowPlan(40, 0.8, 'SPRAYER');
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 20, 'cmd', [1 2], 'dyn', [2 10.25 0 0.5 0.5 0 0 1e5], ...
                'mcu', [0 2 1 1 0 0 2 0], 'Spray', [165 0.2]);
            tH = tc.first(r, r.f_state == double(GncState.OBSTACLE_HOLD));
            iOn = find(r.spray_enable > 0, 1);
            k = find(r.tc < tH & r.spray_enable > 0, 1, 'last');
            tOff = r.tc(find(r.tc >= tH & r.spray_enable == 0, 1));
            v = r.truth(round(r.tc(k)/0.01) + 1, 4);
            Q = tc.P1.gnc.impl.R_app*interp1(r.t, r.truth(:, 4), r.tc(k))*tc.P1.gnc.impl.boom_w*0.006;
            tc.report('T6_spray', sprintf(['spray on at %.2f s (mask %d of vision 165), hold %.2f s, spray off %.3f s after hold; ' ...
                'flow %.3f L/min at v %.3f (law %.3f)'], r.tc(iOn), r.spray_section_mask(iOn), tH, tOff - tH, ...
                r.spray_flow_cmd(k), v, Q));
            tc.verifyLessThanOrEqual(tOff - tH, 0.05 + 1e-9);
            tc.verifyEqual(r.spray_section_mask(iOn), 165);
            tc.verifyEqual(r.spray_flow_cmd(k), tc.P1.gnc.impl.R_app*tc.P1.gnc.impl.boom_w*0.006*interp1(r.tc, ...
                tc.navV(r), r.tc(k)), 'RelTol', 0.02);
        end

        %% ================================ T7 bypass ===========================================
        function testT7BypassTeb(tc)
            m = tc.bypassRun(1);
            tc.verifyBypass(m, 'T7_TEB');
            % TEB wall time per call on the 10 x 10 m window (target <= 60 ms, D56)
            ms = tc.tebCallTime(m.r);
            tc.report('T7_TEB_time', sprintf('TEB wall time per call (10 x 10 m at 5 cells/m, interpreted): median %.1f ms, max %.1f ms (target <= 60 ms)', ...
                median(ms), max(ms)));
            tc.verifyLessThanOrEqual(median(ms), 60);
        end

        function testT7BypassNoTeb(tc)
            m = tc.bypassRun(0);
            tc.verifyBypass(m, 'T7_noTEB');
        end

        %% ================================ T8 - T10 missions ===================================
        function testT8Greenhouse(tc)
            P = shaddad_params('World', 4, 'Assign', false);
            [plan, info] = gnc_compileMission(P, 'greenhouse_std');
            r = gnc_mission_run(P, plan, 'StopTime', 850, 'cmd', [1 2], 'Lidar', 1);
            tDone = tc.first(r, r.f_mission_done > 0);
            tIdle = tc.first(r, r.tf > 5 & r.f_state == double(GncState.IDLE));
            holds = sum(diff(r.f_state == double(GncState.OBSTACLE_HOLD)) > 0);
            [cov, ovl] = tc.coverage(r, plan, P, info);
            ovlPlan = tc.coverage(tc.idealRows(plan), plan, P, info, 'overlap');   % ROW passes on the planned lanes
            [xr, xm] = tc.rowXte(r, plan);
            [hr, hp] = tc.rowHeading(r);
            geo = any(bitand(uint32(r.f_fault_word), uint32(2^9))) || ~all(tc.insideGeofence(r, plan));
            tc.report('T8', sprintf(['greenhouse 9.6 x 37.5 m (%d lanes, mower): mission done at %.1f s = %.2f min (compiler ' ...
                'estimate %.2f min, spec band 22 +/- 3 min), DOCKED -> IDLE %.1f s; obstacle holds %d; coverage %.2f %% of ' ...
                'the inset floor; overlap of the planned ROW passes %.1f %%, of the driven ROW passes %.1f %% (with headland ' ...
                'turns %.1f %%; the T14 row tracker yaws %.2f deg RMS / %.2f deg p99 on the rows, which sweeps the deck 0.9 m ' ...
                'ahead sideways); row XTE RMS %.4f m (max %.3f m); geofence violation %d; nav_cmd navMode %d; wall %.0f s'], ...
                info.n_lanes, tDone, tDone/60, info.t_est_s/60, tIdle, holds, 100*cov, 100*ovlPlan, 100*ovl(1), 100*ovl(2), ...
                hr, hp, xr, xm, geo, max(r.nav_mode), r.wall));
            tc.verifyFalse(isnan(tDone));
            tc.verifyLessThanOrEqual(tDone/60, 25);
            tc.verifyLessThanOrEqual(abs(tDone - info.t_est_s)/info.t_est_s, 0.15);
            tc.verifyGreaterThanOrEqual(cov, 0.98);
            tc.verifyLessThanOrEqual(ovlPlan, 0.15);
            tc.verifyLessThan(xr, 0.03);
            tc.verifyFalse(geo);
            tc.verifyEqual(holds, 0);
            tc.verifyEqual(max(r.nav_mode), tc.P1.gnc.fsm.nav_mode_gh);
        end

        function testT9PalmT20(tc)
            % palm patrol (palm_demo: 6 lanes x 104 m, 1.0 m/s, D46) + the T20 long-run checks on the same run
            P = shaddad_params('World', 3, 'Assign', false);
            [plan, info] = gnc_compileMission(P, 'palm_demo');
            r = gnc_mission_run(P, plan, 'StopTime', 1100, 'cmd', [1 2]);
            tDone = tc.first(r, r.f_mission_done > 0);
            [xr, xm] = tc.rowXte(r, plan);
            % turn speed: commanded v on TURN segments, and the true speed over the middle half of every turn
            % (the first metres carry the 0.1 s dead time + drive lag of the deceleration from the row speed)
            turn = r.f_sub == double(AutoSub.TURN) & r.f_state == double(GncState.AUTO_MISSION);
            vCmdTurn = max(interp1(r.tc, r.v_cmd, r.tf(turn), 'previous'));
            vTurn = 0;  vEntry = 0;
            for k = unique(r.f_seg_idx(turn))'
                tt = r.tf(turn & r.f_seg_idx == k);
                vv = interp1(r.t, r.truth(:, 4), tt);
                q = round(numel(tt)/4);
                vTurn = max(vTurn, max(vv(q + 1:end - q)));
                vEntry = max(vEntry, max(vv));
            end
            seqOk = all(mod(diff(r.seq), 256) == 1);
            tc.report('T9', sprintf(['palm_demo %d lanes x %.0f m at %.1f m/s: mission done %.1f s vs estimate %.1f s (%+.1f %%); row XTE RMS ' ...
                '%.4f m (max %.3f); turns: commanded v <= %.3f m/s, true speed mid-turn <= %.3f m/s (entry peak %.3f); wall %.0f s ' ...
                '(%.1fx real time, Normal mode)'], info.n_lanes, plan.seg(2, 3), plan.speed_work, tDone, info.t_est_s, ...
                100*(tDone - info.t_est_s)/info.t_est_s, xr, xm, vCmdTurn, vTurn, vEntry, r.wall, r.t(end)/r.wall));
            tc.report('T20', sprintf(['long-run checks on the %.0f s palm run: Bus_GncCmd finite %d, seq heartbeat +1 every 0.05 s %d; ' ...
                '4 h Accelerator run BLOCKED (no C compiler, D31); Normal mode ran at %.1fx real time'], r.t(end), r.cmd_finite, ...
                seqOk, r.t(end)/r.wall));
            tc.verifyFalse(isnan(tDone));
            tc.verifyLessThanOrEqual(abs(tDone - info.t_est_s)/info.t_est_s, 0.05);
            tc.verifyLessThan(xr, 0.05);
            tc.verifyLessThanOrEqual(vCmdTurn, 0.8 + 1e-6);
            tc.verifyLessThanOrEqual(vTurn, 0.8 + 1e-6);
            tc.verifyTrue(r.cmd_finite);
            tc.verifyTrue(seqOk);
        end

        function testT20Accelerator(tc)
            cc = mex.getCompilerConfigurations('C', 'Installed');
            tc.assumeTrue(~isempty(cc), 'BLOCKED: no C compiler for Accelerator mode (D31); T20 long-run checks run in testT9PalmT20');
        end

        function testT10Solar(tc)
            P = shaddad_params('World', 5, 'Assign', false);
            [plan, info] = gnc_compileMission(P, 'solar_demo');
            r = gnc_mission_run(P, plan, 'StopTime', 700, 'cmd', [1 2]);
            tDone = tc.first(r, r.f_mission_done > 0);
            holds = sum(diff(r.f_state == double(GncState.OBSTACLE_HOLD)) > 0);
            W = P.world.obstacles;
            piers = W.cyl(1:W.nCyl, :);  piers = piers(piers(:, 6) == 3, :);
            dmin = gnc_mission_metrics(r, 'clearance', [ones(size(piers, 1), 1) piers(:, 1:3) zeros(size(piers, 1), 2)]);
            nRow = sum(plan.seg(1:plan.n_seg, 1) == SegType.ROW);
            rowsDone = numel(unique(r.f_seg_idx(r.f_sub == double(AutoSub.ROW))));
            tc.report('T10', sprintf(['solar_demo %d aisles: mission done %.1f s (estimate %.1f s); min body distance to piers ' ...
                '%.3f m; obstacle holds %d; ROW segments driven %d of %d; wall %.0f s'], info.n_lanes, tDone, info.t_est_s, ...
                dmin, holds, rowsDone, nRow, r.wall));
            tc.verifyFalse(isnan(tDone));
            tc.verifyGreaterThanOrEqual(dmin, 0.30);
            tc.verifyEqual(holds, 0);
            tc.verifyEqual(rowsDone, nRow);
        end

        %% ================================ T11 mode logic ======================================
        function testT11ModeLogic(tc)
            [ids, M, chk] = t_gnc_mission.fsmScenarios(tc.P1);
            need = [1 2 11:20 21:25 31:33 35 36 41:44 51:56 61];
            missing = setdiff(need, ids);
            names = {'IDLE', 'MANUAL', 'AUTO', 'PAUSE', 'HOLD', 'RTH', 'FAULT'};
            rows = cell(1, 7);
            for i = 1:7
                rows{i} = sprintf('%s:%s', names{i}, mat2str(M(i, :)));
            end
            tc.report('T11', sprintf(['transition IDs fired %s; missing %s; state transition matrix (from row to column, ' ...
                'IDLE MANUAL AUTO PAUSE HOLD RTH FAULT) %s; checks: %s'], mat2str(ids), mat2str(missing), strjoin(rows, ' '), ...
                strjoin(fieldnames(chk)', ' ')));
            tc.verifyEmpty(missing);
            f = fieldnames(chk);
            for k = 1:numel(f)
                tc.verifyTrue(chk.(f{k}), f{k});
            end
        end

        function testT11ClosedLoop(tc)
            % RC MANUAL during AUTO -> MANUAL within 1 step, cmd_valid false; RC back to AUTO -> PAUSE; RESUME ->
            % AUTO with REJOIN when |e_y| > 0.3 m (pose offset); ABORT -> IDLE; E-stop -> FAULT within 1 step;
            % RESET_FAULT refused while rc_sw == AUTO
            plan = tc.rowPlan(60, 0.8);
            mcu = [0 2 1 1 0 0 0 0; 8 0 1 1 0 0 0 0; 12 2 1 1 0 0 0 0; 30 2 1 1 1 0 0 0; 31 2 1 1 0 0 0 0; 34 1 1 1 0 0 0 0];
            nav = [0 1 0 4 0 0; 12 1 0 4 0 0.6; 22 1 0 4 0 0];
            cmd = [1 2; 14 4; 26 5; 27 2; 32 7; 35 7];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 37, 'mcu', mcu, 'nav', nav, 'cmd', cmd);
            tMan = tc.first(r, r.tf >= 8 & r.f_state == double(GncState.MANUAL));
            cvMan = r.f_cmd_valid(find(r.tf >= tMan, 1));
            tPau = tc.first(r, r.tf >= 12 & r.f_state == double(GncState.PAUSE));
            subRes = r.f_sub(find(r.tf >= 14.25, 1));
            rejoin = any(r.f_sub(r.tf >= 14 & r.tf < 26) == double(AutoSub.REJOIN));
            tIdle = tc.first(r, r.tf >= 26 & r.f_state == double(GncState.IDLE));
            tFault = tc.first(r, r.tf >= 30 & r.f_state == double(GncState.FAULT));
            stayFault = all(r.f_state(r.tf >= 32.3 & r.tf < 35) == double(GncState.FAULT));
            tReset = tc.first(r, r.tf >= 35 & r.f_state == double(GncState.IDLE));
            tc.report('T11_loop', sprintf(['rc MANUAL at 8.00 -> MANUAL %.2f s (cmd_valid %d); rc AUTO at 12.00 -> PAUSE %.2f s; ' ...
                'RESUME (e_y 0.6 m) -> sub %d (REJOIN %d) %d; ABORT -> IDLE %.2f s; E-stop at 30.00 -> FAULT %.2f s; ' ...
                'RESET_FAULT with rc AUTO refused %d, with rc HOLD -> IDLE %.2f s'], tMan, cvMan, tPau, subRes, ...
                double(AutoSub.REJOIN), rejoin, tIdle, tFault, stayFault, tReset));
            tc.verifyLessThanOrEqual(tMan - 8, 0.05 + 1e-9);
            tc.verifyEqual(cvMan, 0);
            tc.verifyLessThanOrEqual(tPau - 12, 0.05 + 1e-9);
            tc.verifyTrue(rejoin);
            tc.verifyFalse(isnan(tIdle));
            tc.verifyLessThanOrEqual(tFault - 30, 0.05 + 1e-9);
            tc.verifyTrue(stayFault);
            tc.verifyFalse(isnan(tReset));
        end

        %% ================================ S4 fixes (2026-11-14) ===============================
        function testT36DriveReadyDebounce(tc)
            % T36: drive_ready drops of 0.1 and 0.3 s (a parked ESC unparks through ENABLING, drive spec 2.9) do not
            % pause; a drop of t_drive_lost pauses (OPERATOR) in AUTO, OBSTACLE_HOLD and RTH; with the RC link lost
            % at once (RC_LOSS, D44). Scripted FSM, then a closed-loop bench run with the same drops.
            P = tc.P1;  Tl = P.gnc.fsm.t_drive_lost;  Ts = P.gnc.Ts;
            F = t_gnc_mission.fsmDriver(P, t_gnc_mission.fsmPlan(P));
            nDrop = @(f) f.runUntil(@(g) g.st == GncState.PAUSE, 100);   % steps after the first drop step
            % AUTO
            F.reset();  F.startAuto();  F.drive(20);
            k0 = size(F.log, 1);
            for n = [2 6]
                F.set('mcu.drive_ready', false);  F.run(n);  F.set('mcu.drive_ready', true);  F.run(4);
            end
            autoShort = all(F.log(k0 + 1:end, 1) == double(GncState.AUTO_MISSION));
            F.set('mcu.drive_ready', false);  nA = nDrop(F);
            autoLong = F.out.pause_reason == uint8(HoldReason.OPERATOR) && F.out.trans_id == 36;
            % OBSTACLE_HOLD
            F.set('mcu.drive_ready', true);  F.cmd(MissionCmd.RESUME);  F.run(2);
            F.set('obs.d_obs', 1.0);  F.run(2);
            inHold = F.st == GncState.OBSTACLE_HOLD;
            k0 = size(F.log, 1);
            F.set('mcu.drive_ready', false);  F.run(6);  F.set('mcu.drive_ready', true);  F.run(2);
            holdShort = inHold && all(F.log(k0 + 1:end, 1) == double(GncState.OBSTACLE_HOLD));
            F.set('mcu.drive_ready', false);  nH = nDrop(F);
            holdLong = F.out.pause_reason == uint8(HoldReason.OPERATOR) && F.out.trans_id == 36;
            % RETURN_TO_HOME (transit on the replanned path)
            F.set('mcu.drive_ready', true);  F.set('obs.d_obs', inf);  F.cmd(MissionCmd.RESUME);  F.run(2);
            F.cmd(MissionCmd.RTH);  F.run(1);  F.rpAnswer(true, 6);  F.run(3);
            inRth = F.st == GncState.RETURN_TO_HOME && F.sub == AutoSub.RTH_TRANSIT;
            k0 = size(F.log, 1);
            F.set('mcu.drive_ready', false);  F.run(6);  F.set('mcu.drive_ready', true);  F.run(2);
            rthShort = inRth && all(F.log(k0 + 1:end, 1) == double(GncState.RETURN_TO_HOME));
            F.set('mcu.drive_ready', false);  nR = nDrop(F);
            rthLong = F.out.pause_reason == uint8(HoldReason.OPERATOR) && F.out.trans_id == 36;
            % RC link lost with the drive: at once
            F.reset();  F.startAuto();  F.drive(20);
            F.set('mcu.rc_link_ok', false);  F.set('mcu.drive_ready', false);  nRc = nDrop(F);
            rcAtOnce = nRc == 0 && F.out.pause_reason == uint8(HoldReason.RC_LOSS);
            % closed loop (bench: drive_ready also gates the plant): drops of 0.1 s at 6 s and 0.3 s at 10 s, a lost
            % drive from 16 s to 18 s, RESUME at 19 s
            plan = tc.rowPlan(60, 0.8);
            mcu = [0 2 1 1 0 0 0 0; 6 2 1 0 0 0 0 0; 6.1 2 1 1 0 0 0 0; 10 2 1 0 0 0 0 0; 10.3 2 1 1 0 0 0 0; ...
                   16 2 1 0 0 0 0 0; 18 2 1 1 0 0 0 0];
            r = gnc_mission_run(P, plan, 'StopTime', 22, 'mcu', mcu, 'cmd', [1 2; 19 4]);
            tA = tc.first(r, r.f_state == double(GncState.AUTO_MISSION));
            noPause = all(r.f_state(r.tf > tA + 0.1 & r.tf < 16) == double(GncState.AUTO_MISSION));
            tP = tc.first(r, r.tf >= 16 & r.f_state == double(GncState.PAUSE));
            reason = r.f_pause_reason(find(r.tf >= tP, 1));
            tR = tc.first(r, r.tf >= 19 & r.f_state == double(GncState.AUTO_MISSION));
            x = @(t) interp1(r.t, r.truth(:, 1), t);
            dx1 = x(15.9) - x(10.4);  dx2 = x(22) - x(19.5);
            tc.report('T36', sprintf(['t_drive_lost %.2f s. Scripted: AUTO drops 0.1 / 0.3 s no pause %d, long drop -> PAUSE ' ...
                'after %.2f s (OPERATOR, 36) %d; OBSTACLE_HOLD 0.3 s %d, long %.2f s %d; RTH_TRANSIT 0.3 s %d, long %.2f s %d; ' ...
                'RC link lost -> PAUSE RC_LOSS after %.2f s %d. Bench: AUTO %.2f s, no pause through the 0.1 / 0.3 s drops %d; ' ...
                'drive lost at 16.00 -> PAUSE %.2f s (+%.2f), reason %d (OPERATOR %d); RESUME 19.00 -> AUTO %.2f s; x %.2f m ' ...
                'driven 10.4-15.9 s, %.2f m after the resume'], Tl, autoShort, nA*Ts, autoLong, holdShort, nH*Ts, holdLong, ...
                rthShort, nR*Ts, rthLong, nRc*Ts, rcAtOnce, tA, noPause, tP, tP - 16, reason, double(HoldReason.OPERATOR), tR, ...
                dx1, dx2));
            tc.verifyTrue(autoShort);  tc.verifyTrue(holdShort);  tc.verifyTrue(rthShort);
            for n = [nA nH nR]
                tc.verifyLessThan(abs(n*Ts - Tl), Ts/2);
            end
            tc.verifyTrue(autoLong);  tc.verifyTrue(holdLong);  tc.verifyTrue(rthLong);
            tc.verifyTrue(rcAtOnce);
            tc.verifyTrue(noPause);
            tc.verifyLessThanOrEqual(abs(tP - 16 - Tl), 0.1 + 1e-9);
            tc.verifyEqual(reason, double(HoldReason.OPERATOR));
            tc.verifyLessThanOrEqual(tR - 19, 0.25 + 1e-9);          % Bus_MissionCmd at 0.2 s (START: 1.00 -> 1.20 s)
            tc.verifyGreaterThan(dx1, 3);
            tc.verifyGreaterThan(dx2, 0.5);
        end

        function testResumeAlignHeading(tc)
            % P1 / P2: a resume on the lane facing 175 deg away from it (the S4 rejoin at 458.8 s) pivots in place
            % (ALIGN, no replan) and then goes on along the lane (S2, S1); off the lane, tilted or on a TURN segment
            % (no pivot, seg column 10) it rejoins (REJOIN, LOCAL replan); below resume_epsi it just continues
            P = tc.P1;  A = P.gnc.align;  Ts = P.gnc.Ts;  e0 = deg2rad(175);
            F = t_gnc_mission.fsmDriver(P, t_gnc_mission.fsmPlan(P));
            % (a) on the lane (0.22 m), 175 deg off, ROW: ALIGN, the pivot (e_psi follows w = sat(-K e_psi, w_max)), ROW
            F.reset();  F.startAuto();  F.drive(40);
            seg0 = double(F.out.seg_idx);  sub0 = F.sub;
            req0 = t_gnc_mission.pauseAndSet(F, 0.22, e0);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            al = F.st == GncState.AUTO_MISSION && F.sub == AutoSub.ALIGN && F.out.trk_sel == uint8(TrackerSel.ALIGN) && ...
                F.out.v_cap == 0 && F.out.cmd_valid && F.out.replan_req == req0;
            F.hold = true;  k = 0;
            while F.sub == AutoSub.ALIGN && k < 400
                e = double(F.pe.epsi);
                F.set('pe.epsi', e + max(-A.w_max, min(A.w_max, -A.K*e))*Ts);
                F.step();  k = k + 1;
            end
            F.hold = false;
            back = F.sub == sub0 && F.out.trans_id == 22 && double(F.out.seg_idx) == seg0 && F.out.replan_req == req0;
            eEnd = rad2deg(abs(double(F.pe.epsi)));
            F.drive(2000, @(f) double(f.out.seg_idx) ~= seg0);
            onward = double(F.out.seg_idx) == seg0 + 1 && F.st == GncState.AUTO_MISSION;
            % (b) P2 (auto resume after NAV_DEGRADED) with the same pose
            F.reset();  F.startAuto();  F.drive(40);
            F.set('nav.nav_degraded', true);  F.run(round(2.05/0.05));
            navPause = F.st == GncState.PAUSE;
            F.set('pe.xte', 0.22);  F.set('pe.epsi', e0);  F.set('nav.nav_degraded', false);
            F.runUntil(@(f) f.st == GncState.AUTO_MISSION, 200);
            p2 = navPause && F.sub == AutoSub.ALIGN && F.out.trans_id == 42;
            % (c) off the lane, (d) tilted, (e) TURN segment: REJOIN; (f) 20 deg: continue
            F.reset();  F.startAuto();  F.drive(40);  t_gnc_mission.pauseAndSet(F, 0.45, e0);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            offLane = F.sub == AutoSub.REJOIN && F.out.replan_kind == uint8(P.gnc.fsm.kind_local);
            F.reset();  F.startAuto();  F.drive(40);  t_gnc_mission.pauseAndSet(F, 0.10, e0);  F.set('nav.pitch', 0.25);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            tilted = F.sub == AutoSub.REJOIN;
            F.reset();  F.startAuto();  F.drive(4000, @(f) f.sub == AutoSub.TURN);  F.drive(10);
            onTurn = F.sub == AutoSub.TURN;
            t_gnc_mission.pauseAndSet(F, 0.10, e0);  F.cmd(MissionCmd.RESUME);  F.run(1);
            turnSeg = onTurn && F.sub == AutoSub.REJOIN;
            F.reset();  F.startAuto();  F.drive(40);  subF = F.sub;  req0 = t_gnc_mission.pauseAndSet(F, 0.10, deg2rad(20));
            F.cmd(MissionCmd.RESUME);  F.run(1);
            small = F.sub == subF && F.out.replan_req == req0;
            tc.report('resume_align', sprintf(['resume on the lane (e_y 0.22 m, e_psi 175 deg, ROW): ALIGN, trk ALIGN, v_cap 0, ' ...
                'no replan %d; pivot at the ALIGN law %.2f s to %.1f deg, S2 back to the ROW %d, on to the next segment %d; ' ...
                'P2 after NAV_DEGRADED -> ALIGN %d; REJOIN when off the lane (0.45 m) %d, tilted (14 deg) %d, on a TURN %d; ' ...
                'e_psi 20 deg continues on the ROW %d'], al, k*Ts, eEnd, back, onward, p2, offLane, tilted, turnSeg, small));
            tc.verifyTrue(al);  tc.verifyTrue(back);  tc.verifyTrue(onward);
            tc.verifyLessThan(eEnd, rad2deg(A.exit));
            tc.verifyTrue(p2);
            tc.verifyTrue(offLane);  tc.verifyTrue(tilted);  tc.verifyTrue(turnSeg);  tc.verifyTrue(small);
        end

        function testRejoinGlobalFallback(tc)
            % REJOIN after a resume: a failed LOCAL plan is followed by a GLOBAL plan to the same goal, and only a failed
            % GLOBAL plan counts as a try (PLANNER_FAIL after retries LOCAL + GLOBAL pairs, the goal rejoin_off further
            % for each pair); a bypass rejoin stays LOCAL. Planner on the S4 geometry: greenhouse lane 5, 0.22 m left
            % of the centre and 175 deg off its heading: LOCAL fails, GLOBAL succeeds and ends on the lane.
            P = tc.P1;  Fp = P.gnc.fsm;
            F = t_gnc_mission.fsmDriver(P, t_gnc_mission.fsmPlan(P));
            % (a) LOCAL fails, GLOBAL succeeds: REPLAN path, S5 back on the ROW
            F.reset();  F.startAuto();  F.drive(40);  t_gnc_mission.pauseAndSet(F, 0.45, 0);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            k1 = double(F.out.replan_kind);  g1 = double(F.out.replan_goal);
            F.set('pe.xte', 0);  F.rpAnswer(false);
            k2 = double(F.out.replan_kind);  g2 = double(F.out.replan_goal);
            fallback = F.sub == AutoSub.REJOIN && k1 == Fp.kind_local && k2 == Fp.kind_global && max(abs(g1 - g2)) < 1e-9;
            F.rpAnswer(true);
            src = F.out.path_src;
            F.drive(400, @(f) f.sub ~= AutoSub.REJOIN);
            rejoined = src == uint8(PathSrc.REPLAN) && F.sub == AutoSub.ROW && F.st == GncState.AUTO_MISSION;
            % (b) every plan fails
            F.reset();  F.startAuto();  F.drive(40);  t_gnc_mission.pauseAndSet(F, 0.45, 0);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            kinds = double(F.out.replan_kind);  gx = double(F.out.replan_goal(1));  nF = 0;
            while F.st == GncState.AUTO_MISSION && nF < 10
                F.rpAnswer(false);  nF = nF + 1;
                if F.st == GncState.AUTO_MISSION
                    kinds(end + 1) = double(F.out.replan_kind);  gx(end + 1) = double(F.out.replan_goal(1)); %#ok<AGROW>
                end
            end
            pf = F.st == GncState.PAUSE && F.out.pause_reason == uint8(HoldReason.PLANNER_FAIL);
            pairs = isequal(kinds, repmat([Fp.kind_local Fp.kind_global], 1, Fp.retries)) && nF == 2*Fp.retries && ...
                all(abs(gx(2:2:end) - gx(1:2:end)) < 1e-9) && all(abs(diff(gx(1:2:end)) - Fp.rejoin_off) < 1e-6);
            % (c) bypass (TEB infeasible -> REJOIN, S4): LOCAL only, PLANNER_FAIL after retries failures
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(round(10.2/0.05));  F.set('obs.d_obs', inf);
            F.rpAnswer(true);  F.run(3);  F.set('teb', [1; 1; 0; 1]);  F.run(round(3.5/0.05));
            inRejoin = F.sub == AutoSub.REJOIN;
            F.set('teb', [0; 12; 0; 1]);
            kb = double(F.out.replan_kind);  nB = 0;
            while F.st == GncState.AUTO_MISSION && nB < 10
                F.rpAnswer(false);  nB = nB + 1;
                if F.st == GncState.AUTO_MISSION, kb(end + 1) = double(F.out.replan_kind); end %#ok<AGROW>
            end
            bypassLocal = inRejoin && all(kb == Fp.kind_local) && nB == Fp.retries && ...
                F.out.pause_reason == uint8(HoldReason.PLANNER_FAIL);
            % (e) a bypass interrupted by a pause on its detour (operator PAUSE in BYPASS on the REPLAN path), resumed: the
            %     REJOIN stays LOCAL (lane half-width, spec 8 T7), also after a second pause during that rejoin (the
            %     request outstanding at that pause is answered while the FSM is paused, which ignores it)
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(round(10.2/0.05));  F.set('obs.d_obs', inf);
            F.rpAnswer(true);  F.run(10);
            onDetour = F.sub == AutoSub.BYPASS && F.out.path_src == uint8(PathSrc.REPLAN);
            F.cmd(MissionCmd.PAUSE);  F.run(2);  F.cmd(MissionCmd.RESUME);  F.run(1);
            resumedRejoin = F.sub == AutoSub.REJOIN;
            ke = double(F.out.replan_kind);
            F.rpAnswer(false);  ke(end + 1) = double(F.out.replan_kind);
            F.cmd(MissionCmd.PAUSE);  F.run(2);  F.rpAnswer(false);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            [k2, nE] = t_gnc_mission.failAll(F, 10);
            ke = [ke k2];
            pausedBypass = onDetour && resumedRejoin && all(ke == Fp.kind_local) && nE == Fp.retries && ...
                F.out.pause_reason == uint8(HoldReason.PLANNER_FAIL);
            % (f) a pause in the TEB rejoin of a bypass (S4), resumed: LOCAL only
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(round(10.2/0.05));  F.set('obs.d_obs', inf);
            F.rpAnswer(true);  F.run(3);  F.set('teb', [1; 1; 0; 1]);  F.run(round(3.5/0.05));
            inTebRejoin = F.sub == AutoSub.REJOIN;
            F.set('teb', [0; 12; 0; 1]);
            F.cmd(MissionCmd.PAUSE);  F.run(2);  F.rpAnswer(false);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            [kf, nT] = t_gnc_mission.failAll(F, 10);
            pausedTeb = inTebRejoin && all(kf == Fp.kind_local) && nT == Fp.retries && ...
                F.out.pause_reason == uint8(HoldReason.PLANNER_FAIL);
            % (d) the planner on greenhouse lane 5 (the S4 pose of the 459 s call)
            [okL, okG, dEnd, info] = t_gnc_mission.s4RejoinPlans();
            tc.report('rejoin_global', sprintf(['LOCAL rejoin fails -> GLOBAL to the same goal %d, GLOBAL ok -> REPLAN path, S5 ' ...
                'back on the ROW %d; all plans fail: kinds %s, %d failures -> PLANNER_FAIL %d (goals +%.1f m per pair %d); ' ...
                'bypass rejoin: kinds %s, %d failures -> PLANNER_FAIL %d; bypass paused on its detour and resumed (twice): ' ...
                'kinds %s, %d failures after the second resume -> PLANNER_FAIL %d; TEB rejoin paused and resumed: kinds %s, ' ...
                '%d failures -> PLANNER_FAIL %d. Greenhouse lane %d, start %s (%s): LOCAL ok %d ' ...
                '(%.2f s), GLOBAL ok %d (%.2f s, %.1f m), path end %.3f m / %.1f deg from the goal'], fallback, rejoined, ...
                mat2str(kinds), nF, pf, Fp.rejoin_off, pairs, mat2str(kb), nB, bypassLocal, mat2str(ke), nE, pausedBypass, ...
                mat2str(kf), nT, pausedTeb, info.seg, mat2str(info.start, 4), ...
                info.desc, okL, info.tL, okG, info.tG, info.len, dEnd(1), dEnd(2)));
            tc.verifyTrue(fallback);  tc.verifyTrue(rejoined);
            tc.verifyTrue(pf);  tc.verifyTrue(pairs);
            tc.verifyTrue(bypassLocal);
            tc.verifyTrue(pausedBypass);  tc.verifyTrue(pausedTeb);
            tc.verifyFalse(okL);  tc.verifyTrue(okG);
            tc.verifyLessThan(dEnd(1), 0.10);  tc.verifyLessThan(dEnd(2), 5);
        end

        %% ================================ T12 SOC -> RTH -> dock ==============================
        function testT12SocRth(tc)
            P = tc.P1;
            % four 40 m rows at 0.25 m/s (clear of the world-1 test obstacles), dock north of the last row; SOC ramps
            % 0.40 -> 0.18 in 673 s and keeps falling at that rate (linear in time, about 7x the real drain at work speed),
            % so RTH triggers on the last row and the reserve above 0.18 must cover the trip to the dock
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 20 40 20; 40 25 0 25; 0 30 40 30; 40 35 0 35], ...
                'home_enu', [0 20 0], 'speed_work', 0.25, 'turn_radius_m', 2.5, 'skip_rows', 0, 'dock_enu', [6 40 0], ...
                'end_action', 'RTH');
            plan = gnc_compileMission(P, S, 'Validate', false);
            bms = [0 0.40 0; 673 0.18 0; 2000 0.18 - 0.22/673*1327 0];
            r = gnc_mission_run(P, plan, 'StopTime', 700, 'cmd', [1 2], 'bms', bms, 'mcu', [0 2 1 1 0 2 0 0]);
            iR = find(r.f_trans_id == 18, 1);
            soc = @(t) interp1(bms(:, 1), bms(:, 2), t);
            socRthT = interp1(r.ts, r.s_soc_rth, r.tf(iR), 'previous');
            tDock = tc.first(r, r.f_sub == double(AutoSub.DOCKED));
            tIdle = tc.first(r, r.tf > tDock & r.f_state == double(GncState.IDLE));
            kD = find(r.t >= tDock, 1);
            p = r.truth(kD, 1:3);
            dk = plan.dock(:)';
            ey = -(p(1) - dk(1))*sin(dk(3)) + (p(2) - dk(2))*cos(dk(3));
            ep = rad2deg(angle(exp(1i*(p(3) - dk(3)))));
            % soc_rth block math at d_home = 500 m (home placed 500 m from the robot, IDLE run)
            plan5 = plan;  plan5.home = [-500; 0; 0];
            r5 = gnc_mission_run(P, plan5, 'StopTime', 1, 'x0', [0 0 0]);
            s500 = r5.s_soc_rth(end);
            tc.report('T12', sprintf(['RTH (T8) at %.2f s with SOC %.4f vs soc_rth %.4f (diff %+.4f); DOCKED at %.2f s with SOC %.4f; ' ...
                'dock pose error e_y %.4f m, e_psi %.2f deg; DOCKED -> IDLE (charging) %.2f s; soc_rth(500 m) = %.4f'], ...
                r.tf(iR), soc(r.tf(iR)), socRthT, soc(r.tf(iR)) - socRthT, tDock, soc(tDock), ey, ep, tIdle, s500));
            tc.verifyLessThanOrEqual(abs(soc(r.tf(iR)) - socRthT), 0.005);
            tc.verifyGreaterThanOrEqual(soc(tDock), 0.18);
            tc.verifyLessThan(abs(ey), 0.03);
            tc.verifyLessThan(abs(ep), 2);
            tc.verifyFalse(isnan(tIdle));
            F = tc.P.gnc.fsm;                           % soc_rth = soc_reserve + k_tort e_m k_margin d / E_nom (spec 2.8)
            tc.verifyEqual(s500, F.soc_reserve + F.k_tort*500*F.e_m*F.k_margin/F.E_nom_Wh, 'AbsTol', 1e-4);
        end

        %% ================================ T13 - T17 faults ====================================
        function testT13RtkDegradation(tc)
            plan = tc.rowPlan(80, 0.8);
            nav = [0 1 0 4 0 0; 15 1 0.15 3 0 0; 25 1 0 4 0 0];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 34, 'cmd', [1 2], 'nav', nav);
            k15 = find(r.tc >= 15, 1);
            vNav = r.v_limits(k15, 8);
            tP = tc.first(r, r.tf >= 15 & r.f_state == double(GncState.PAUSE));
            reason = r.f_pause_reason(find(r.tf >= tP, 1));
            tA = tc.first(r, r.tf >= 25 & r.f_state == double(GncState.AUTO_MISSION));
            tv = r.tc(find(r.tc >= 15 & r.v_cmd <= 0.5 + 1e-6, 1));
            tc.report('T13', sprintf(['fix -> float (sigma 0.15) at 15.00 s: governor nav limit %.3f m/s in the same step, v_cmd <= 0.5 at ' ...
                '%.2f s; PAUSE (reason %d) at %.2f s (+%.2f); fixed again at 25.00: AUTO at %.2f s (+%.2f)'], vNav, tv, reason, ...
                tP, tP - 15, tA, tA - 25));
            tc.verifyLessThanOrEqual(vNav, 0.5);
            tc.verifyEqual(reason, double(HoldReason.NAV_DEGRADED));
            tc.verifyLessThanOrEqual(abs(tP - 15 - 2.0), 0.1 + 1e-9);
            tc.verifyLessThanOrEqual(abs(tA - 25 - 5.0), 0.1 + 1e-9);
        end

        function testT7NavSigmaRule(tc)
            % T7 sigma term of nav_degraded on gnc_InputConditioning alone (gnc_ic_run; S4 fix 2026-11-14, docs/specs/
            % gnc.md 4.1 T7 and 8 T13). The covariance is built from the sigmas along / across a lane at heading psi
            % (laneProf), so the expected relay inputs are those sigmas, not the model's formula; the expectation
            % (relayRef, pe one 0.05 s step late) must match nav_degraded at every 0.05 s step:
            % (a) lane along East: across 0.105-0.145 m (off holds), a ramp up (on at the first step >= 0.15),
            %     0.15 +/- 0.003 (on holds), 0.105-0.145 (on holds), a ramp down (off at the first step <= 0.10),
            %     0.10 +/- 0.003 and 0.105-0.145 (off holds): one rise and one fall, no chatter;
            % (b) lane at 30 deg, the ellipse along it (along 0.24, across 0.05 m): clear, although max(posStd(1:2))
            %     is 0.21 m (the old 0.10 m rule) and the projection without covPose(1,2) 0.152 m; then the across
            %     sigma ramped: on where it reaches 0.15 m, off where it falls to 0.10 m;
            % (c) lane at 120 deg, the along sigma ramped 0.10 -> 0.30 -> 0.10 m (across 0.05): on at 0.25, off at 0.20;
            % (d) lane at 90 deg (along 0.16 m = N, across 0.05 m = E): clear with a valid path error, degraded while
            %     pe.valid is false (both sigmas are max(posStd(1:2)) = 0.16 m)
            F = tc.P1.gnc.fsm;
            dt = 0.01;
            sq = @(t, a) a*(2*mod(floor(t/0.25), 2) - 1);              % +-a, every 0.25 s
            tri = @(t) 0.105 + 0.04*(1 - abs(2*mod(t, 1) - 1));         % 0.105-0.145 m, 1 s period
            C = {};
            t = (0:dt:36)';  sc = 0.05*ones(size(t));
            m = t >= 2 & t < 8;    sc(m) = tri(t(m));
            m = t >= 8 & t < 10;   sc(m) = 0.1215 + 0.02*(t(m) - 8);
            m = t >= 10 & t < 16;  sc(m) = 0.150 + sq(t(m), 0.003);
            m = t >= 16 & t < 22;  sc(m) = tri(t(m));
            m = t >= 22 & t < 24;  sc(m) = 0.1215 - 0.02*(t(m) - 22);
            m = t >= 24 & t < 30;  sc(m) = 0.100 + sq(t(m), 0.003);
            m = t >= 30;           sc(m) = tri(t(m));
            C{end+1} = struct('id', 'a', 'psi', 0, 'p', t_gnc_mission.laneProf(t, 0, 0.05*ones(size(t)), sc, true(size(t))), ...
                'up', 9.45, 'dn', 23.10);
            t = (0:dt:30)';  sa = 0.05*ones(size(t));  sc = sa;
            sa(t >= 1 & t < 10) = 0.24;  sa(t >= 10) = 0.12;
            m = t >= 10 & t < 20;  sc(m) = 0.0512 + 0.015*(t(m) - 10);
            m = t >= 20;           sc(m) = 0.2012 - 0.015*(t(m) - 20);
            C{end+1} = struct('id', 'b', 'psi', pi/6, 'p', t_gnc_mission.laneProf(t, pi/6, sa, sc, true(size(t))), ...
                'up', 16.60, 'dn', 26.75);
            t = (0:dt:21)';  sa = 0.05*ones(size(t));  sc = sa;
            m = t >= 1 & t < 11;  sa(m) = 0.1012 + 0.02*(t(m) - 1);
            m = t >= 11;          sa(m) = 0.3012 - 0.02*(t(m) - 11);
            C{end+1} = struct('id', 'c', 'psi', 2*pi/3, 'p', t_gnc_mission.laneProf(t, 2*pi/3, sa, sc, true(size(t))), ...
                'up', 8.45, 'dn', 16.10);
            t = (0:dt:15)';  sa = 0.05*ones(size(t));  sc = sa;  sa(t >= 1) = 0.16;
            C{end+1} = struct('id', 'd', 'psi', pi/2, 'p', t_gnc_mission.laneProf(t, pi/2, sa, sc, ~(t >= 5 & t < 10)), ...
                'up', 5.05, 'dn', 10.05);
            msg = {};
            for k = 1:numel(C)
                c = C{k};  p = c.p;
                r = gnc_ic_run(tc.P1, p);
                i = round(r.t/dt) + 1;                              % profile samples at the 0.05 s steps
                vz = p.valid([1; i(1:end - 1)]);  vz(1) = false;   % pe_z: previous step; P.ic.Bus_PathError at t = 0
                smax = max(p.sE(i), p.sN(i));
                xc = p.sc(i);  xa = p.sa(i);
                xc(~vz) = smax(~vz);  xa(~vz) = smax(~vz);
                e = t_gnc_mission.relayRef(xc, F.sigma_pause, F.sigma_degr) | ...
                    t_gnc_mission.relayRef(xa, F.sigma_along, F.sigma_along_off);
                d = diff([false; r.nav_degraded(:)]);
                up = r.t(d == 1)';  dn = r.t(d == -1)';
                msg{end+1} = sprintf('(%s) lane %g deg: on at %s s, off at %s s, %d of %d steps as expected', c.id, ...
                    round(c.psi*180/pi), mat2str(up, 4), mat2str(dn, 4), nnz(r.nav_degraded(:) == e), numel(e)); %#ok<AGROW>
                tc.verifyEqual(r.nav_degraded(:), e, sprintf('(%s) nav_degraded at every 0.05 s step', c.id));
                tc.verifyEqual(up, c.up, 'AbsTol', 1e-6, sprintf('(%s) switch-on times', c.id));
                tc.verifyEqual(dn, c.dn, 'AbsTol', 1e-6, sprintf('(%s) switch-off times', c.id));
                tc.verifyTrue(all(r.nav_ok), sprintf('(%s) nav_ok (max(posStd(1:2)) < sigma_lost)', c.id));
                tc.verifyEqual(r.sigma_pos, smax, 'AbsTol', 1e-12, sprintf('(%s) sigma_pos = max(posStd(1:2))', c.id));
                if c.id == 'b'
                    m = r.t >= 1 & r.t < 10;
                    s = sin(c.psi);  co = cos(c.psi);
                    dg = sqrt(s^2*p.sE(i(m)).^2 + co^2*p.sN(i(m)).^2);
                    msg{end+1} = sprintf('(b) 1-10 s: max(posStd(1:2)) %.3f m, without covPose(1,2) %.3f m, not degraded %d', ...
                        min(smax(m)), min(dg), ~any(r.nav_degraded(m))); %#ok<AGROW>
                    tc.verifyGreaterThan(min(smax(m)), F.sigma_degr, '(b) the old rule would pause');
                    tc.verifyGreaterThan(min(dg), F.sigma_pause, '(b) a projection without covPose(1,2) would pause');
                    tc.verifyFalse(any(r.nav_degraded(m)), '(b) along 0.24 / across 0.05 m is clear');
                end
            end
            tc.report('T7_sigma_rule', strjoin(msg, '; '));
        end

        function testT7NavSigmaPause(tc)
            % T7 and P2 through the mission bench with the lane-frame sigma rule (S4 fix 2026-11-14): RTK FIXED
            % (quality 4) throughout, so only the sigma term acts; the bench feeds posStd(1) = posStd(2) = sigma and
            % covPose 0, so the cross-track sigma is sigma on the row. sigma 0.105 / 0.145 m in turn every 0.5 s from
            % 10 to 20 s: no pause (the old max(posStd(1:2)) > 0.10 m rule paused 2 s after 10 s); 0.16 m from 20 s:
            % PAUSE (NAV_DEGRADED) at 22.0 +/- 0.1 s; 0.105 / 0.145 m from 24 to 34 s: no resume (hysteresis);
            % nominal sigma from 34 s: automatic resume (P2) at 39.0 +/- 0.1 s
            plan = tc.rowPlan(80, 0.8);
            band = @(tb) [tb, ones(size(tb)), 0.125 + 0.02*(-1).^(1:numel(tb))', 4*ones(size(tb)), zeros(numel(tb), 2)];
            nav = [0 1 0 4 0 0; band((10:0.5:19.5)'); 20 1 0.16 4 0 0; band((24:0.5:33.5)'); 34 1 0 4 0 0];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 42, 'cmd', [1 2], 'nav', nav);
            isP = r.f_state(:) == double(GncState.PAUSE);
            kP = find(diff([false; isP]) == 1);
            tP = r.tf(kP);
            tA = tc.first(r, r.tf > 22 & r.f_state == double(GncState.AUTO_MISSION));
            auto1 = all(r.f_state(r.tf >= 10 & r.tf < 20) == double(GncState.AUTO_MISSION));
            held = all(isP(r.tf >= 22.5 & r.tf < 38.8));
            reason = NaN;
            if ~isempty(kP), reason = r.f_pause_reason(kP(1)); end
            tc.report('T7_sigma_pause', sprintf(['RTK FIXED; sigma 0.105/0.145 m 10-20 s: AUTO throughout %d; 0.16 m from ' ...
                '20 s: %d pause(s) at %s s, reason %d (NAV_DEGRADED %d); 0.105/0.145 m 24-34 s: paused throughout %d; ' ...
                'nominal from 34 s: AUTO at %.2f s (+%.2f)'], auto1, numel(kP), mat2str(tP(:)', 4), reason, ...
                double(HoldReason.NAV_DEGRADED), held, tA, tA - 34));
            tc.verifyTrue(auto1, 'no pause while the sigma stays between sigma_degr and sigma_pause');
            tc.assertNumElements(tP, 1);
            tc.verifyEqual(reason, double(HoldReason.NAV_DEGRADED));
            tc.verifyLessThanOrEqual(abs(tP - 22.0), 0.1 + 1e-9);
            tc.verifyTrue(held, 'no resume while the sigma stays between sigma_degr and sigma_pause');
            tc.verifyLessThanOrEqual(abs(tA - 39.0), 0.1 + 1e-9);
        end

        function testT14NavInvalid(tc)
            plan = tc.rowPlan(60, 0.8);
            nav = [0 1 0 4 0 0; 10 0 0 4 0 0; 10.4 1 0 4 0 0];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 13, 'cmd', [1 2], 'nav', nav);
            tF = tc.first(r, r.f_state == double(GncState.FAULT));
            k = find(r.tc >= tF, 1);
            fw = r.f_fault_word(find(r.tf >= tF, 1));
            tc.report('T14', sprintf('nav_valid false 10.0-10.4 s: FAULT at %.2f s (+%.2f), fault_word 0x%X, v_cmd %.3f and cmd_valid %d in that step', ...
                tF, tF - 10, fw, r.v_cmd(k), r.cmd_valid(k)));
            tc.verifyLessThanOrEqual(abs(tF - 10 - 0.30), 0.05 + 1e-9);
            tc.verifyTrue(bitand(uint32(fw), uint32(1)) > 0);
            tc.verifyEqual(r.v_cmd(k), 0);
            tc.verifyEqual(r.cmd_valid(k), 0);
        end

        function testT15McuFrozen(tc)
            plan = tc.rowPlan(60, 0.8);
            mcu = [0 2 1 1 0 0 0 0; 10 2 1 1 0 0 0 1];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 12, 'cmd', [1 2], 'mcu', mcu);
            tF = tc.first(r, r.f_state == double(GncState.FAULT));
            fw = r.f_fault_word(find(r.tf >= tF, 1));
            tc.report('T15', sprintf('MCU status frozen at 10.00 s: FAULT at %.2f s (+%.2f), fault_word 0x%X', tF, tF - 10, fw));
            tc.verifyLessThanOrEqual(abs(tF - 10 - 0.20), 0.05 + 1e-9);
            tc.verifyTrue(bitand(uint32(fw), uint32(4)) > 0);
        end

        function testT16Geofence(tc)
            plan = tc.rowPlan(60, 0.8);
            b = plan.boundary(1:plan.n_bnd, :);
            yMin = min(b(:, 2));                                    % row along y = 0: push south
            dy = -(0 - yMin) - 0.6;
            nav = [0 1 0 4 0 0; 10 1 0 4 0 dy];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 12, 'cmd', [1 2], 'nav', nav);
            tF = tc.first(r, r.f_state == double(GncState.FAULT));
            fw = r.f_fault_word(find(r.tf >= tF, 1));
            tc.report('T16', sprintf('pose pushed %.2f m south (0.6 m outside the grown geofence) at 10.00 s: FAULT at %.2f s (+%.2f), fault_word 0x%X', ...
                -dy, tF, tF - 10, fw));
            tc.verifyLessThanOrEqual(tF - 10, 0.5);
            tc.verifyTrue(bitand(uint32(fw), uint32(2^9)) > 0);
        end

        function testT17Stuck(tc)
            plan = tc.rowPlan(60, 0.5);
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 22, 'cmd', [1 2], 'plant', [0 1; 12 0]);
            tF = tc.first(r, r.f_state == double(GncState.FAULT));
            fw = r.f_fault_word(find(r.tf >= tF, 1));
            vc = r.v_cmd(find(r.tc >= 11.9, 1));
            tc.report('T17', sprintf('plant speed forced to 0 at 12.00 s with v_cmd %.2f m/s: FAULT at %.2f s (+%.2f), fault_word 0x%X', ...
                vc, tF, tF - 12, fw));
            tc.verifyLessThanOrEqual(abs(tF - 12 - 5.0), 0.1 + 1e-9);
            tc.verifyTrue(bitand(uint32(fw), uint32(2^4)) > 0);
        end

        %% ================================ T18 Hybrid A* =======================================
        function testT18HybridAStar(tc)
            [ok, clr, tPlan, ratio, info] = t_gnc_mission.hastarStatic(tc.P1);
            tc.report('T18', sprintf(['GLOBAL Hybrid A* on a 100 x 100 m static map with %d keep-outs: success %d, path %.1f m vs ' ...
                'shortest grid path %.1f m (ratio %.3f), min clearance to the keep-outs %.3f m (r_infl %.2f), t_plan %.2f s (MATLAB, ' ...
                'interpreted); LOCAL lane-masked detour ok %d, max lane deviation %.2f m (half-width %.1f); world 2 U-turn plan ok %d, ' ...
                'U-turn (heading 30..150 deg) from x %.2f m (ramp 20 deg for x 10..30 m), max heading deviation from the fall line on ' ...
                'terrain > 10 deg %.1f deg (tolerance %.1f)'], info.nKeep, ok, info.len, info.lenGrid, ratio, clr, ...
                tc.P1.gnc.hastar.r_infl, tPlan, info.localOk, info.localDev, info.half, info.slopeOk, info.uturnX, info.slopeDev, info.slopeTol));
            tc.verifyTrue(ok);
            tc.verifyGreaterThanOrEqual(clr, tc.P1.gnc.hastar.r_infl - 0.1/2);
            tc.verifyLessThan(tPlan, 1.5);
            tc.verifyLessThanOrEqual(ratio, 1.3);
            tc.verifyTrue(info.localOk);
            tc.verifyLessThanOrEqual(info.localDev, info.half + 1e-6);
            tc.verifyTrue(info.slopeOk);
            tc.verifyLessThanOrEqual(info.slopeDev, info.slopeTol + 0.5);
            tc.verifyGreaterThan(info.uturnX, 30);
        end

        %% ================================ implement / status ==================================
        function testTillerAndStatus(tc)
            % tiller lowered 0.3 m into the ROW and raised 0.5 m before its end; status: xte_rms (10 s moving RMS on ROW),
            % progress monotonic to 1, dist_home, nav_cmd AUTO for a non-greenhouse plan
            plan = tc.rowPlan(20, 0.6, 'TILLER');
            r = gnc_mission_run(tc.P1, plan, 'StopTime', 50, 'cmd', [1 2], 'mcu', [0 2 1 1 0 0 2 0]);
            low = r.tiller_depth_cmd > 0;
            x = interp1(r.t, r.truth(:, 1), r.tc);
            xLow = x(find(low, 1));  xUp = x(find(low, 1, 'last'));
            rmsS = r.s_xte_rms(r.s_sub_mode == double(AutoSub.ROW));
            prog = r.s_progress;
            tc.report('impl_status', sprintf(['tiller lowered at x %.3f m (lag %.2f), raised after x %.3f m (row end 20 m, lead %.2f); ' ...
                'status: xte_rms on ROW %.4f m, progress %.3f -> %.3f (monotonic %d), navMode %d'], xLow, ...
                tc.P1.gnc.impl.till_lower_lag, xUp, tc.P1.gnc.impl.till_lift_lead, rmsS(end), prog(1), prog(end), ...
                all(diff(prog) >= -0.005), max(r.nav_mode)));
            tc.verifyEqual(xLow, tc.P1.gnc.impl.till_lower_lag, 'AbsTol', 0.06);
            tc.verifyEqual(xUp, 20 - tc.P1.gnc.impl.till_lift_lead, 'AbsTol', 0.06);
            tc.verifyLessThan(rmsS(end), 0.03);
            tc.verifyEqual(prog(end), 1, 'AbsTol', 1e-6);
            tc.verifyTrue(all(diff(prog) >= -0.005));               % RTK noise moves s by a few mm at standstill
            tc.verifyEqual(max(r.nav_mode), 0);
        end

        %% ================================ critical fixes (2026-11-15) =========================
        function testPersonHoldMowerOff(tc)
            % R-03 (GNC-02, spec 2.9): the blades stay off from the first PERSON hold until the hold ends, also when the
            % person steps back to the deck front, out of the protective field that shrank with the speed; they restart
            % with H1. GNC-02's bench case (MOWER row 0.5 m/s, 0.3 m circle switched on 1.5 m ahead) plus the step back
            m = tc.personHoldRun('MOWER');
            tc.verifyPersonHold(m, 'person_hold_mower');
        end

        function testPersonHoldTillerOff(tc)
            % R-03: the same for the tiller rotor (tiller_enable); the tiller stays raised inside the hold
            m = tc.personHoldRun('TILLER');
            tc.verifyPersonHold(m, 'person_hold_tiller');
            tc.verifyEqual(max(m.r.tiller_depth_cmd(m.iHold)), 0, 'tiller raised inside the hold');
        end

        function testScanLostOnRow(tc)
            % R-02 (NAV-01, SYS-02): a LiDAR loss on a ROW (scanValid false; the blank scan reads as clear: d_obs Inf, empty
            % VFH, protectiveStop false) stops the robot in OBSTACLE_HOLD with HoldReason SENSOR_LOST, mower off; the hold
            % lasts while the scan is invalid; with the scan back the normal hold logic resumes: H1 t_clear later
            plan = tc.rowPlan(40, 0.8, 'MOWER');
            t1 = 8;  t2 = 14;
            % first protNStale missed scans from tD (no loss: P.sensors.per.protNStale, the field's stale rule), then the
            % loss from t1 to t2; the table times of the short dropout lie between the 0.1 s scan samples
            nS = tc.P1.sensors.per.protNStale;  tsS = tc.P1.sensors.lidar.Ts;  tD = 5;
            r = gnc_mission_run(tc.P1, plan, 'StopTime', t2 + 6, 'cmd', [1 2], 'mcu', [0 2 1 1 0 0 2 0], ...
                'scan', [0 1; tD - tsS/2 0; tD + (nS - 0.5)*tsS 1; t1 0; t2 1]);
            HOLD = double(GncState.OBSTACLE_HOLD);  SL = double(HoldReason.SENSOR_LOST);
            lag = 0.1 + tc.P1.gnc.fsm.t_scan_lost;                   % RT_scan (one 0.1 s scan) + debounce
            staleNoHold = all(r.f_state(r.tf >= tD - 1e-9 & r.tf < t1 - 1e-9) == double(GncState.AUTO_MISSION)) && ...
                all(r.mower_enable(r.tc >= tD - 1e-9 & r.tc < t1 - 1e-9) == 1);
            tH = tc.first(r, r.tf >= t1 & r.f_state == HOLD);
            tR = tc.first(r, r.tf > tH & r.f_state ~= HOLD);
            hrH = r.f_hold_reason(find(r.tf >= tH - 1e-9, 1));
            iBlind = r.tf >= tH - 1e-9 & r.tf < t2 + 0.1 - 1e-9;
            held = all(r.f_state(iBlind) == HOLD);
            reasonBlind = unique(r.f_hold_reason(iBlind))';
            vBlind = max(abs(r.truth(r.t >= tH + 3 & r.t < t2, 4)));    % after the stop (drive lag tau_v 0.35 s)
            mowBefore = r.mower_enable(find(r.tc < t1, 1, 'last'));
            mowHold = max(r.mower_enable(r.tc >= tH - 1e-9 & r.tc < tR - 1e-9));
            trR = r.f_trans_id(find(r.tf >= tR - 1e-9, 1));
            kb = find(r.tc >= tR - 1e-9 & r.mower_enable > 0, 1);
            if isempty(kb), tMow = NaN; else, tMow = r.tc(kb); end
            vAfter = max(r.truth(r.t > tR + 3, 4));
            tc.report('scan_lost_row', sprintf(['%d missed scans at %.1f s (protNStale %d): no hold, mower on %d; scan ' ...
                'invalid %.1f-%.1f s: OBSTACLE_HOLD at %.2f s (+%.2f s, RT + t_scan_lost %.2f s), reason %d (SENSOR_LOST %d); ' ...
                'held while blind %d, reasons %s, speed while blind <= %.3f m/s; mower %d before, max %d in the hold; ' ...
                'release at %.2f s (transition %d, %.2f s after the scan came back), mower back %.2f s, speed after %.2f ' ...
                'm/s'], nS, tD, nS, staleNoHold, t1, t2, tH, tH - t1, lag, hrH, SL, held, mat2str(reasonBlind), vBlind, ...
                mowBefore, mowHold, tR, trR, tR - t2, tMow, vAfter));
            tc.verifyTrue(staleNoHold, 'protNStale missed scans: no hold, mower on');
            tc.verifyLessThanOrEqual(tH - t1, lag + 0.05 + 1e-9);
            tc.verifyEqual(hrH, SL);
            tc.verifyTrue(held, 'no release while the scan is invalid');
            tc.verifyEqual(reasonBlind, SL);
            tc.verifyLessThan(vBlind, 0.02);
            tc.verifyEqual(mowBefore, 1);
            tc.verifyEqual(mowHold, 0, 'mower off for the whole SENSOR_LOST hold');
            tc.verifyEqual(trR, 31, 'released by H1');
            tc.verifyLessThanOrEqual(abs(tR - t2 - tc.P1.gnc.obs.t_clear), 0.1 + 1e-9);
            tc.verifyLessThanOrEqual(tMow - tR, 0.05 + 1e-9);
            tc.verifyGreaterThan(vAfter, 0.5);
        end

        function testScanLostInHold(tc)
            % R-02: a scan lost during an OBSTACLE_HOLD keeps the hold (before the fix the blank scan read as clear and the
            % hold released t_clear after the loss) and shows SENSOR_LOST; with the scan back the box is seen again, the
            % entry reason OBSTACLE returns and the hold releases t_clear after the box is removed (H1, as T6)
            plan = tc.rowPlan(40, 0.8);
            box = [2 10.25 0 0.5 0.5 0];
            r0 = gnc_mission_run(tc.P1, plan, 'StopTime', 16, 'cmd', [1 2], 'dyn', [box 0 1e5]);
            HOLD = double(GncState.OBSTACLE_HOLD);  SL = double(HoldReason.SENSOR_LOST);
            tH0 = tc.first(r0, r0.f_state == HOLD);
            OB = r0.f_hold_reason(find(r0.tf >= tH0 - 1e-9, 1));     % entry reason of the box hold (OBSTACLE)
            g = @(x) round(x*10)/10;                                  % on the 0.1 s scan grid
            tS1 = g(tH0 + 1.5);  tS2 = g(tH0 + 4.5);  tOff = g(tH0 + 6);
            r = gnc_mission_run(tc.P1, plan, 'StopTime', tOff + 5, 'cmd', [1 2], 'dyn', [box 0 tOff], ...
                'scan', [0 1; tS1 0; tS2 1]);
            lag = 0.1 + tc.P1.gnc.fsm.t_scan_lost;
            tH = tc.first(r, r.f_state == HOLD);
            tR = tc.first(r, r.tf > tH & r.f_state ~= HOLD);
            tSL = tc.first(r, r.tf > tH & r.f_hold_reason == SL);
            tOB = tc.first(r, r.tf > tSL & r.f_hold_reason == OB);
            rs = @(a, b) unique(r.f_hold_reason(r.tf >= a - 1e-9 & r.tf < b - 1e-9))';
            held = all(r.f_state(r.tf >= tH - 1e-9 & r.tf < tR - 1e-9) == HOLD);
            trR = r.f_trans_id(find(r.tf >= tR - 1e-9, 1));
            tc.report('scan_lost_hold', sprintf(['box hold at %.2f s (reference run %.2f s, reason %d); scan invalid %.1f-%.1f ' ...
                's: reason SENSOR_LOST from %.2f s (+%.2f s), entry reason again from %.2f s (+%.2f s after the scan came ' ...
                'back); reasons before / blind / back %s / %s / %s; held %d; box removed %.1f s, release %.2f s (transition ' ...
                '%d, +%.2f s)'], tH, tH0, OB, tS1, tS2, tSL, tSL - tS1, tOB, tOB - tS2, mat2str(rs(tH, tSL)), ...
                mat2str(rs(tSL, tOB)), mat2str(rs(tOB, tR)), held, tOff, tR, trR, tR - tOff));
            tc.verifyEqual(OB, double(HoldReason.OBSTACLE), 'the box hold is an OBSTACLE hold');
            tc.verifyEqual(tH, tH0, 'AbsTol', 1e-9);
            tc.verifyLessThanOrEqual(tSL - tS1, lag + 0.05 + 1e-9);
            tc.verifyLessThanOrEqual(abs(tOB - tS2 - 0.1), 0.05 + 1e-9);
            tc.verifyEqual(rs(tH, tSL), OB);
            tc.verifyEqual(rs(tSL, tOB), SL);
            tc.verifyEqual(rs(tOB, tR), OB);
            tc.verifyTrue(held, 'the hold lasts through the scan loss and while the box is there');
            tc.verifyEqual(trR, 31, 'released by H1');
            tc.verifyLessThanOrEqual(abs(tR - tOff - tc.P1.gnc.obs.t_clear), 0.1 + 1e-9);
        end

        function testScanLostFsm(tc)
            % FSM level (gnc_missionFSM through gnc_fsm_driver, scripted): a scan invalid for t_scan_lost on a ROW -> T5
            % OBSTACLE_HOLD SENSOR_LOST, shown also with person_near; no H1 and no H2 (bypass allowed, t_bypass passed)
            % while it lasts; with the scan back H1 after t_clear_eff, or H2 once the scan has been valid t_clear_eff; H3 ->
            % PAUSE SENSOR_LOST; R6 in RTH. Latch (spec 2.9): impl_allow false for the rest of a hold once person_near was
            % seen, true again after H1; a later hold without a person keeps it
            P = tc.P1;  pr = P.gnc.fsm;  SL = uint8(HoldReason.SENSOR_LOST);
            F = t_gnc_mission.fsmDriver(P, t_gnc_mission.fsmPlan(P));
            c = struct();
            % 0 debounce (the protective field's stale rule, read from P.sensors.per.protNStale): protNStale missed scans
            % are no loss: no hold on the ROW; in an OBSTACLE hold no SENSOR_LOST and no implement latch
            c.t_scan_lost_from_protNStale = abs(pr.t_scan_lost - P.sensors.per.protNStale*P.sensors.lidar.Ts) < 1e-12;
            nStale = round(pr.t_scan_lost/pr.Ts);           % FSM steps of protNStale invalid 0.1 s scans
            F.reset();  F.startAuto();  F.drive(40);
            F.set('scanOk', false);  F.run(nStale);  F.set('scanOk', true);  F.run(2);
            c.stale_scans_no_hold = F.st == GncState.AUTO_MISSION && F.sub == AutoSub.ROW;
            F.set('obs.d_obs', 1.0);  F.run(2);
            F.set('scanOk', false);  F.run(nStale);
            c.stale_scans_no_sensor_lost = F.st == GncState.OBSTACLE_HOLD && ...
                F.out.hold_reason == uint8(HoldReason.OBSTACLE) && F.out.impl_allow;
            % 1 scan lost on the ROW, then 30 s blind on a clear-looking (blank) scan
            F.reset();  F.startAuto();  F.drive(40);
            c.on_row = F.st == GncState.AUTO_MISSION && F.sub == AutoSub.ROW && F.out.impl_allow;
            F.set('scanOk', false);
            n = F.runUntil(@(f) f.st == GncState.OBSTACLE_HOLD, 20);
            c.hold_after_t_scan_lost = abs(n*pr.Ts - pr.t_scan_lost) < 1e-9 && F.out.hold_reason == SL;
            c.stopped_impl_off = ~F.out.impl_allow && ~F.out.cmd_valid && F.out.mode_req == uint8(GncModeReq.HOLD);
            F.run(round(30/pr.Ts));
            c.blind_no_release = F.st == GncState.OBSTACLE_HOLD && F.out.hold_reason == SL;
            F.set('obs.person_near', true);  F.run(2);
            c.sensor_lost_over_person = F.st == GncState.OBSTACLE_HOLD && F.out.hold_reason == SL;
            % 2 scan back, corridor clear: H1 after t_clear_eff (H2 waits for a scan valid t_clear_eff)
            F.set('obs.person_near', false);  F.set('scanOk', true);
            n = F.runUntil(@(f) f.st ~= GncState.OBSTACLE_HOLD, 200);
            c.h1_after_scan_back = F.st == GncState.AUTO_MISSION && F.sub == AutoSub.ROW && ...
                abs(n*pr.Ts - pr.t_clear_eff) < 1e-9 && F.out.trans_id == uint8(31) && F.out.impl_allow;
            % 3 scan lost for 12 s, back with an obstacle in the corridor: H2 once the scan has been valid t_clear_eff
            F.drive(20);
            F.set('scanOk', false);  F.run(round(12/pr.Ts));
            F.set('obs.d_obs', 1.0);  F.set('scanOk', true);
            n = F.runUntil(@(f) f.sub == AutoSub.BYPASS, 200);
            c.h2_after_valid_scan = F.st == GncState.AUTO_MISSION && abs(n*pr.Ts - pr.t_clear_eff) < 1e-9;
            % 4 a blind hold of t_hold_max -> PAUSE SENSOR_LOST (H3)
            F.reset();  F.startAuto();  F.drive(40);
            F.set('scanOk', false);  F.run(round((pr.t_hold_max + 0.5)/pr.Ts));
            c.h3_pause_sensor_lost = F.st == GncState.PAUSE && F.out.pause_reason == SL;
            % 5 R6: scan lost on the RTH transit
            F.reset();  F.startAuto();  F.drive(20);
            F.cmd(MissionCmd.RTH);  F.run(1);  F.rpAnswer(true, 6);  F.run(3);
            c.rth_transit = F.st == GncState.RETURN_TO_HOME && F.sub == AutoSub.RTH_TRANSIT;
            F.set('scanOk', false);  F.run(round(pr.t_scan_lost/pr.Ts) + 1);
            c.r6_sensor_lost = F.st == GncState.OBSTACLE_HOLD && F.out.hold_reason == SL;
            % 6 latch: PERSON hold with an obstacle left in the corridor, the field loses the person after 0.5 s
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.person_near', true);  F.set('obs.d_obs', 1.0);  F.run(1);
            c.person_hold_impl_off = F.st == GncState.OBSTACLE_HOLD && ...
                F.out.hold_reason == uint8(HoldReason.PERSON) && ~F.out.impl_allow;
            F.run(round(0.5/pr.Ts));  F.set('obs.person_near', false);  F.run(round(3/pr.Ts));
            c.latched_after_person_left = F.st == GncState.OBSTACLE_HOLD && ~F.out.impl_allow;
            F.set('obs.d_obs', inf);
            F.runUntil(@(f) f.st ~= GncState.OBSTACLE_HOLD, 200);
            c.latch_cleared_by_h1 = F.st == GncState.AUTO_MISSION && F.out.impl_allow;
            F.drive(10);  F.set('obs.d_obs', 1.0);  F.run(2);
            c.obstacle_hold_keeps_impl = F.st == GncState.OBSTACLE_HOLD && ...
                F.out.hold_reason == uint8(HoldReason.OBSTACLE) && F.out.impl_allow;
            % 7 Perception failing safe (sensors-nav): the field holds its last result for protNStale invalid scans and
            % raises protectiveStop in the step the GNC counts the scan lost; the hold is a SENSOR_LOST hold (checked
            % before PERSON) and stays one until it ends
            F.reset();  F.startAuto();  F.drive(40);
            F.set('scanOk', false);  F.run(nStale);
            c.fail_safe_no_hold_while_stale = F.st == GncState.AUTO_MISSION;
            F.set('obs.person_near', true);  F.run(1);
            c.fail_safe_entry_sensor_lost = F.st == GncState.OBSTACLE_HOLD && F.out.hold_reason == SL && ...
                ~F.out.impl_allow;
            F.set('scanOk', true);  F.set('obs.person_near', false);  F.run(2);
            c.fail_safe_reason_kept = F.st == GncState.OBSTACLE_HOLD && F.out.hold_reason == SL;
            f = fieldnames(c);
            ok = cellfun(@(x) c.(x), f);
            tc.report('scan_lost_fsm', sprintf('t_scan_lost %.2f s, t_clear_eff %.2f s; checks passed %d of %d; failed: %s', ...
                pr.t_scan_lost, pr.t_clear_eff, sum(ok), numel(ok), strjoin(f(~ok)', ' ')));
            for k = 1:numel(f)
                tc.verifyTrue(c.(f{k}), f{k});
            end
        end

        %% ================================ downhill speed cap (Agents task descent, 2026-11-15) ====
        function testDescentCapRule(tc)
            % the cap of one pitch (P.gnc.desc from P.structure.descent: rows implement_type + 1, columns theta_bp of
            % measured downhill pitch in the direction of travel), less the MCU's target margin D.dv: between breakpoints
            % the steeper one, a breakpoint its own value (also deg2rad(20)), level or uphill the first column, above the
            % last breakpoint its value (0, TILT_WARN), an unknown implement the lowest row; the tiller may not reverse at
            % any slope. Every expected value is read from the table (descRef: the rule on its deg axis, not
            % gnc_descentCap): named cases, then a sweep over every breakpoint, every bin's middle, just steeper than each
            % breakpoint, uphill and above the last, for every row (deck-raised rows too), forward and reverse.
            % CommandShaping's envelope (gnc_descentEnvelope) scales v to the cap and omega with it, and leaves a pivot
            % and an allowed command alone
            D = tc.P1.gnc.desc;  Sd = tc.P1.structure.descent;  d = pi/180;  ref = t_gnc_mission.descRef(tc.P1);
            bp = Sd.theta_bp;                                       % deg
            cf = @(th, im) gnc_descentCap(th*d, im, false, D);
            cr = @(th, im) gnc_descentCap(th*d, im, true, D);
            rf = @(th, im) ref(th, im, false);
            rr = @(th, im) ref(th, im, true);
            % between: 12 / 5.5 deg sprayer, 17 deg none, 20.25 deg sprayer and 22.5 deg none (step-8 columns); beyond:
            % above the last breakpoint, none
            got = struct('sprayer', [cf(10, 3) cf(15, 3) cf(20, 3)], 'none', [cf(10, 0) cf(15, 0) cf(20, 0)], ...
                'between', [cf(12, 3) cf(5.5, 3) cf(17, 0) cf(20.25, 3) cf(22.5, 0)], 'beyond', cf(bp(end) + 1, 0), ...
                'exact20', gnc_descentCap(deg2rad(20), 3, false, D), 'level_uphill', [cf(0, 3) cf(-15, 3)], ...
                'reverse', [cr(20, 3) cr(15, 3) cr(0, 2) cr(-10, 2) cr(15, 2)], 'unknown', cf(15, 7));
            want = struct('sprayer', [rf(10, 3) rf(15, 3) rf(20, 3)], 'none', [rf(10, 0) rf(15, 0) rf(20, 0)], ...
                'between', [rf(12, 3) rf(5.5, 3) rf(17, 0) rf(20.25, 3) rf(22.5, 0)], 'beyond', rf(bp(end) + 1, 0), ...
                'exact20', rf(20, 3), 'level_uphill', [rf(0, 3) rf(-15, 3)], ...
                'reverse', [rr(20, 3) rr(15, 3) rr(0, 2) rr(-10, 2) rr(15, 2)], 'unknown', rf(15, 7));
            % sweep: every breakpoint, every bin's middle, just steeper than each breakpoint (the next column), uphill
            % and above the last; implement_type 0..3 and an unknown one (7), the mower also with the deck raised
            thS = [bp, (bp(1:end-1) + bp(2:end))/2, bp(1:end-1) + 1e-4, -bp(2), bp(end) + 1];
            nS = 0;  bad = {};
            for th = thS
                for im = [0 1 2 3 7]
                    for rv = [false true]
                        for rz = [false true]
                            if rz && im ~= 1, continue; end
                            g = gnc_descentCap(th*d, im, rv, D, rz);  w = ref(th, im, rv, rz);
                            nS = nS + 1;
                            if abs(g - w) > 1e-12
                                s = sprintf('%g deg impl %d rev %d raised %d: %g, table %g', th, im, rv, rz, g, w);
                                bad{end + 1} = s; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
            % envelope: tiller reversing on level ground, sprayer forward on 15 deg, no implement reversing, tiller pivot,
            % sprayer reversing down 20 deg (nose up); expected: v scaled to the table's cap, omega by the same factor
            E = [-0.3 0.2 0 2; 1.0 0.5 15*d 3; -0.4 0.1 0 0; 0 0.5 0 2; -0.5 0.2 -20*d 3];
            env = zeros(size(E, 1), 2);
            for k = 1:size(E, 1)
                [env(k, 1), env(k, 2)] = gnc_descentEnvelope(E(k, 1), E(k, 2), E(k, 3), uint8(E(k, 4)), D);
            end
            kT = min(1, rr(0, 2)/0.3);  kS = min(1, rf(15, 3)/1.0);  kN = min(1, rr(0, 0)/0.4);  kR = min(1, rr(20, 3)/0.5);
            wantEnv = [-0.3*kT 0.2*kT; 1.0*kS 0.5*kS; -0.4*kN 0.1*kN; 0 0.5; -0.5*kR 0.2*kR];
            tc.report('descent_cap_rule', sprintf(['margin dv %.2f m/s; %d breakpoints %s deg; sprayer forward 10/15/20 deg ' ...
                '%s m/s, none %s; 12 / 5.5 deg sprayer, 17 deg none, 20.25 deg sprayer, 22.5 deg none %s; above the last ' ...
                'breakpoint none %.2f; deg2rad(20) sprayer %.2f; level / uphill sprayer %s; reverse sprayer 20 / 15 deg, ' ...
                'tiller 0 / -10 / 15 deg %s; unknown implement at 15 deg %.2f; sweep %d pairs, %d off the table; envelope ' ...
                '[v w] %s; no_rev %s, look-ahead %.2f m (%d samples), hold %d steps'], D.dv, numel(bp), mat2str(bp), ...
                mat2str(got.sprayer), mat2str(got.none), mat2str(got.between), got.beyond, got.exact20, ...
                mat2str(got.level_uphill), mat2str(got.reverse), got.unknown, nS, numel(bad), mat2str(env, 3), ...
                mat2str(D.no_rev), D.d_look, D.n_look, D.n_hold));
            f = fieldnames(want);
            for k = 1:numel(f)
                tc.verifyEqual(got.(f{k}), want.(f{k}), 'AbsTol', 1e-12, f{k});
            end
            tc.verifyEmpty(bad, strjoin(bad, '; '));
            tc.verifyEqual(env, wantEnv, 'AbsTol', 1e-12);
            tc.verifyEqual(D.no_rev, [0 0 1 0]);    % the user's rule, not a table value: only the tiller may not reverse
            % the table's form the rule relies on: one column per breakpoint, rising breakpoints (first at or above), and
            % no row rising with pitch (the cap of the held peak pitch is then the lowest cap of the hold window)
            nb = numel(bp);
            tc.verifyEqual([size(Sd.v_fwd, 2) size(Sd.v_rev, 2) numel(Sd.v_fwd_raised) numel(Sd.v_rev_raised)], ...
                nb*ones(1, 4), 'one column per breakpoint');
            tc.verifyTrue(all(diff(bp) > 0), 'theta_bp rises');
            tc.verifyTrue(all(all(diff([Sd.v_fwd; Sd.v_rev; Sd.v_fwd_raised; Sd.v_rev_raised], 1, 2) <= 0)), ...
                'no row rises with pitch');
        end

        function testDescentGovernorCap(tc)
            % SpeedGovernor at the cap: a straight 30 m row at v_work 1.2 m/s (= v_max), constant nav pitch (bench tilt,
            % + nose down = downhill) on the flat DEM of world 1. The v_pitch limit (v_limits 6) is min(LUT(|pitch|), cap
            % - dv) and bounds v_cmd; the expected cap is read from the table P.structure.descent (descRef: the steeper
            % breakpoint, less the MCU's target margin dv). The sprayer (full tank) at 10 / 15 / 20 deg, and at 20.25 /
            % 21.2 deg in the step-8 columns (20.5 / 21.5 deg); no implement at 10 / 15 / 20 / 22 deg: the lower of the
            % LUT and the cap is the limit
            P = tc.P1;  d = pi/180;  ref = t_gnc_mission.descRef(P);
            cases = {'SPRAYER', 10; 'SPRAYER', 15; 'SPRAYER', 20; 'SPRAYER', 20.25; 'SPRAYER', 21.2; 'NONE', 10; ...
                'NONE', 15; 'NONE', 20; 'NONE', 22};
            n = size(cases, 1);
            [lim, want, lut, cap, vmax, vend] = deal(zeros(n, 1));
            for k = 1:n
                th = cases{k, 2}*d;
                if strcmp(cases{k, 1}, 'NONE')
                    plan = tc.rowPlan(30, 1.2);
                else
                    plan = tc.rowPlan(30, 1.2, cases{k, 1});
                end
                r = gnc_mission_run(P, plan, 'StopTime', 8, 'cmd', [1 2], 'mcu', [0 2 1 1 0 0 2 0], 'Tilt', [th 0]);
                lut(k) = interp1(P.gnc.spd.pitch_x, P.gnc.spd.pitch_v, th);
                cap(k) = ref(cases{k, 2}, double(plan.implement_type), false);
                want(k) = min(lut(k), cap(k));
                lim(k) = r.v_limits(end, 6);
                vmax(k) = max(r.v_cmd);
                vend(k) = r.v_cmd(end);
            end
            txt = strjoin(arrayfun(@(k) sprintf('%s %g deg: LUT %.2f, cap %.2f, v_pitch %.3f, v_cmd max %.3f / end %.3f', ...
                cases{k, 1}, cases{k, 2}, lut(k), cap(k), lim(k), vmax(k), vend(k)), 1:n, 'UniformOutput', false), '; ');
            tc.report('descent_governor_cap', txt);
            vm = min(want, P.gnc.spd.v_max);
            tc.verifyEqual(lim, want, 'AbsTol', 1e-5);              % v_limits is single
            tc.verifyLessThanOrEqual(vmax, vm + 1e-5);
            tc.verifyGreaterThanOrEqual(vend, vm - 0.01);           % the limit is reached: it binds
        end

        function testDescentLookAhead(tc)
            % look-ahead (gnc_descentLimit): world 2, the sprayer (full tank) driving west from the plateau at x = 36 m down
            % the 20 deg ramp (top fillet x 31.5 -> 28.5) at v_work 1.0 m/s. The bench's nav pitch stays 0 (constant tilt),
            % so only the DEM look-ahead along the planned path can slow the robot. The predicted body pitch (DEM chord
            % over the contact length) passes 10 deg at X10 and 15 deg at X15 (20 deg on the ramp). The planned caps are
            % read from the table P.structure.descent at that chord pitch (descRef: the steeper breakpoint, less the MCU's
            % target margin dv): V10 just past X10, V15 just past X15, VR the ramp's steepest chord. The truth speed is
            % down to V10 at X10 and to V15 at X15 (within 0.02 m/s), the slow-down starts at least 1 m before X10, and
            % the robot goes on down the ramp at VR, in AUTO, without a stop (no stop-go). StopTime 30 s (26 s with the
            % step-6 table): at the step-8 ramp cap the run still ends about 2.5 m past X15
            P2 = shaddad_params('World', 2, 'Assign', false);
            S = struct('mission_type', 'GOTO', 'rows_enu', [36 0 2 0], 'home_enu', [36 0 pi], 'speed_work', 1.0, ...
                'implement', struct('type', 'SPRAYER', 'speed_rpm', 2800, 'height_m', 0.06));
            plan = gnc_compileMission(P2, S, 'Validate', false);
            r = gnc_mission_run(P2, plan, 'StopTime', 30, 'cmd', [1 2], 'mcu', [0 2 1 1 0 0 2 0]);
            D = P2.gnc.desc;  T = P2.world.terrain;  ref = t_gnc_mission.descRef(P2);
            Hi = griddedInterpolant({T.xE, T.yN}, T.H, 'linear', 'nearest');
            xs = (36:-0.005:20)';
            th = atan((Hi(xs + D.L_c/2, 0*xs) - Hi(xs - D.L_c/2, 0*xs))/D.L_c);   % driving west: rear end at x + L_c/2
            i10 = find(th > 10*pi/180 + D.tol, 1);  i15 = find(th > 15*pi/180 + D.tol, 1);
            x10 = xs(i10);  x15 = xs(i15);
            V = [NaN NaN NaN];                                      % NaN (fails) when the DEM has no such pitch
            if ~isempty(i10) && ~isempty(i15)
                V = [ref(th(i10)*180/pi, 3, false) ref(th(i15)*180/pi, 3, false) ref(max(th)*180/pi, 3, false)];
            end
            x = r.truth(:, 1);  v = r.truth(:, 4);
            k10 = find(x <= x10, 1);  k15 = find(x <= x15, 1);
            v10 = NaN;  v15 = NaN;                                  % NaN (fails) when the robot never got there
            if ~isempty(k10), v10 = v(k10); end
            if ~isempty(k15), v15 = v(k15); end
            kw = find(r.v_gov >= S.speed_work - 1e-6, 1);           % work speed reached
            ks = [];
            if ~isempty(kw), ks = kw - 1 + find(r.v_gov(kw:end) < S.speed_work - 0.02, 1); end
            xSlow = NaN;
            if ~isempty(ks), xSlow = interp1(r.t, x, r.tc(ks)); end
            onRamp = x < x15 - 0.5;                                 % past the fillet's 15 deg point
            auto = all(r.f_state(r.tf > 2) == double(GncState.AUTO_MISSION));
            tc.report('descent_look_ahead', sprintf(['world 2 sprayer at %.1f m/s: X10 %.2f m (cap %.2f), X15 %.2f m (cap ' ...
                '%.2f), ramp chord up to %.4f deg (cap %.2f); truth speed %.3f m/s at X10, %.3f m/s at X15; v_gov below the ' ...
                'work speed from x %.2f m (%.2f m before X10); on the ramp (to x %.2f m) speed %.3f to %.3f m/s, v_pitch ' ...
                'limit at the end %.3f; AUTO throughout %d'], S.speed_work, x10, V(1), x15, V(2), max(th)*180/pi, V(3), ...
                v10, v15, xSlow, xSlow - x10, x(end), min(v(onRamp)), max(v(onRamp)), r.v_limits(end, 6), auto));
            tc.verifyLessThan(x(end), x15 - 2, 'the run reaches the ramp');
            tc.verifyLessThanOrEqual(v10, V(1) + 0.02);
            tc.verifyLessThanOrEqual(v15, V(2) + 0.02);
            tc.verifyGreaterThanOrEqual(xSlow - x10, 1.0);
            tc.verifyGreaterThanOrEqual(min(v(onRamp)), V(3) - 0.05, 'no stop on the ramp');
            tc.verifyLessThanOrEqual(max(v(onRamp)), V(3) + 0.02);
            tc.verifyTrue(auto);
        end

        function testTillerNoReverse(tc)
            % the user (2026-11-15): no reversing with the tiller fitted. The GNC commands reverse only through TEB (BYPASS
            % and its rejoin; MaxReverseVelocity NaN lets TEB plan reverse at up to MaxVelocity): docking, rejoins and RTH
            % follow forward-only Hybrid A* or mission paths with Pure Pursuit / the row tracker (v_gov >= 0), ALIGN pivots
            % (v = 0), and there is no recovery manoeuvre (stuck -> FAULT). With the tiller (P.gnc.fsm.no_rev) H2 starts
            % no TEB bypass: the robot holds (H1 once clear) and pauses with OBST_TIMEOUT after t_hold_max (H3); the
            % VFH-steered pursuit (VC_TEB 0, teb(1) = -1) drives forward only and keeps the bypass. (a) FSM level (b) mission
            % bench: the T7 bypass scenario (world 3, obstacle on the lane, bypass allowed) with the tiller and TEB: no
            % BYPASS, no negative v_cmd or MCU speed reference, PAUSE OBST_TIMEOUT t_hold_max after the hold (shortened to
            % 20 s); (c) reported only: the same scenario with no implement (TEB bypass), its lowest v_cmd
            P = tc.P1;  pr = P.gnc.fsm;  BYP = double(AutoSub.BYPASS);  AUTO = double(GncState.AUTO_MISSION);
            OT = uint8(HoldReason.OBST_TIMEOUT);
            plan = t_gnc_mission.fsmPlan(P);
            planT = plan;  planT.implement_type = uint8(ImplementType.TILLER);
            nB = round((pr.t_bypass + 2)/pr.Ts);
            c = struct();
            % (a) no implement, TEB: H2 after t_bypass
            F = t_gnc_mission.fsmDriver(P, plan);
            F.startAuto();  F.drive(40);  F.set('obs.d_obs', 1.0);
            c.none_teb_bypass = F.runUntil(@(f) f.sub == AutoSub.BYPASS && f.st == GncState.AUTO_MISSION, nB) >= 0;
            % tiller, TEB: the hold goes on past t_bypass, then H3 PAUSE OBST_TIMEOUT
            F = t_gnc_mission.fsmDriver(P, planT);
            F.startAuto();  F.drive(40);  F.set('obs.d_obs', 1.0);  F.run(2);
            c.tiller_teb_hold = F.st == GncState.OBSTACLE_HOLD;
            F.run(nB);
            c.tiller_teb_no_bypass = F.st == GncState.OBSTACLE_HOLD;
            F.runUntil(@(f) f.st ~= GncState.OBSTACLE_HOLD, round(pr.t_hold_max/pr.Ts));
            c.tiller_teb_h3_pause = F.st == GncState.PAUSE && F.out.pause_reason == OT && F.out.trans_id == 33;
            % tiller, VFH-steered pursuit (VC_TEB 0): H2 after t_bypass
            F = t_gnc_mission.fsmDriver(P, planT);
            F.set('teb', [-1; 0; 0; 0]);
            F.startAuto();  F.drive(40);  F.set('obs.d_obs', 1.0);
            c.tiller_vfh_bypass = F.runUntil(@(f) f.sub == AutoSub.BYPASS && f.st == GncState.AUTO_MISSION, nB) >= 0;
            % (b) mission bench, tiller, TEB
            P3 = shaddad_params('World', 3, 'Assign', false);
            P3.gnc.fsm.t_hold_max = 20;
            S = struct('mission_type', 'GOTO', 'rows_enu', [16 20 16 80], 'home_enu', [16 20 pi/2], 'speed_work', 1.0, ...
                'allow_bypass', true, 'lane_halfwidth_m', 2.5, 'implement', struct('type', 'TILLER', 'speed_rpm', 250, ...
                'height_m', 0.06));
            planB = gnc_compileMission(P3, S, 'Validate', false);
            r = gnc_mission_run(P3, planB, 'StopTime', 38, 'cmd', [1 2], 'dyn', [1 16 35 0.5 0 0 0 1e5], 'Teb', 1, ...
                'mcu', [0 2 1 1 0 0 2 0]);
            tH = tc.first(r, r.f_state == double(GncState.OBSTACLE_HOLD));
            tP = tc.first(r, r.tf > tH & r.f_state == double(GncState.PAUSE));
            kP = find(r.tf >= tP, 1);
            if isempty(kP), kP = numel(r.tf); end                   % no pause: the checks below fail
            c.bench_tiller_plan = planB.implement_type == uint8(ImplementType.TILLER);
            c.bench_hold = isfinite(tH);
            c.bench_no_bypass = ~any(r.f_sub == BYP & r.f_state == AUTO) && ~any(r.f_teb_enable > 0);
            c.bench_pause_obst_timeout = abs(tP - tH - P3.gnc.fsm.t_hold_max) <= 0.1 + 1e-9 && ...
                r.f_pause_reason(kP) == double(OT);
            c.bench_no_reverse = min(r.v_cmd) >= 0 && min(r.shaped(:, 1)) >= -1e-9;
            % (c) reported only: no implement, TEB bypass in the same scenario (testT7BypassTeb's run)
            m = tc.bypassRun(1);
            f = fieldnames(c);
            ok = cellfun(@(x) c.(x), f);
            tc.report('tiller_no_reverse', sprintf(['FSM and bench checks passed %d of %d (failed: %s); bench tiller + TEB: hold ' ...
                '%.2f s, PAUSE %.2f s (+%.2f, reason %d), BYPASS steps %d, min v_cmd %.4f m/s, min MCU v_ref %.4f m/s; ' ...
                'reported only, no implement + TEB: BYPASS %.2f s, min v_cmd %.4f m/s (%d steps below -0.01 m/s)'], sum(ok), ...
                numel(ok), strjoin(f(~ok)', ' '), tH, tP, tP - tH, r.f_pause_reason(kP), sum(r.f_sub == BYP), min(r.v_cmd), ...
                min(r.shaped(:, 1)), m.tB, min(m.r.v_cmd), sum(m.r.v_cmd < -0.01)));
            for k = 1:numel(f)
                tc.verifyTrue(c.(f{k}), f{k});
            end
        end

        function testMowerDeckDownWhenMoving(tc)
            % the user (2026-11-15): the mower deck is down whenever the robot moves (raised, the robot is nose-heavy). The
            % GNC commands the deck to impl.mower_h in every state and never raises it; with no deck report it counts the
            % deck down after t_deck of AUTO since power-up or the last MANUAL, and until then the robot does not move (STOP
            % tracker, v_cap 0, IMPLEMENT_NOT_READY); already moving, the deck-raised rows of P.structure.descent bound
            % v_cap. P.gnc.desc is read from P.structure.descent (no copied numbers). (a) FSM level: mower start, a MANUAL
            % spell, moving with the deck up, no implement. (b) Mission bench, mower row with the deck raised at power-up
            % ('DeckDown' false): the deck height as the MCU moves it (fw_vehicle_ctrl's lift law in its AUTO mode: rate
            % h_rate*sat(lift_Kp*(h_cmd - h), +-lift_duty_max), lag h_tau, from the top stop), replayed on the logged GNC
            % states: the robot never moves or turns while the deck is more than deck_tol above its command
            P = tc.P1;  pr = P.gnc.fsm;  Ts = pr.Ts;  D = P.gnc.desc;  Sd = P.structure.descent;
            ref = t_gnc_mission.descRef(P);
            STOP = uint8(TrackerSel.STOP);  INR = uint8(HoldReason.IMPLEMENT_NOT_READY);
            plan = t_gnc_mission.fsmPlan(P);
            planM = plan;  planM.implement_type = uint8(ImplementType.MOWER);
            nD = ceil(pr.t_deck/Ts - 1e-9);
            c = struct();
            c.desc_read_from_structure = isequal(D.v_fwd, Sd.v_fwd) && isequal(D.v_rev, Sd.v_rev) && ...
                isequal(D.v_fwd_raised, Sd.v_fwd_raised) && isequal(D.v_rev_raised, Sd.v_rev_raised) && ...
                max(abs(D.theta_bp - Sd.theta_bp*pi/180)) < 1e-12;
            % (a) mower, deck height unknown at power-up (deck_t0 = 0, the product default)
            F = t_gnc_mission.fsmDriver(P, planM);
            F.startAuto();
            n = F.runUntil(@(f) f.out.trk_sel ~= STOP, 2*nD);
            c.start_waits_t_deck = n + 1 == nD;
            F.reset();  F.startAuto();  F.run(1);
            c.wait_no_motion = F.st == GncState.AUTO_MISSION && F.out.trk_sel == STOP && F.out.v_cap == 0 && ...
                F.out.cmd_valid && F.out.hold_reason == INR && F.out.mode_req == uint8(GncModeReq.AUTO);
            F.run(nD);
            c.then_row = F.out.trk_sel == uint8(TrackerSel.ROW) && F.out.v_cap > 0 && F.out.hold_reason == 0;
            % a MANUAL spell: the operator may have moved the deck; after RESUME the robot waits t_deck again
            F.set('mcu.rc_mode_sw', uint8(0));  F.run(2);
            c.manual = F.st == GncState.MANUAL;
            F.set('mcu.rc_mode_sw', uint8(2));  F.run(2);
            F.cmd(MissionCmd.RESUME);  F.run(1);
            c.resume_waits = F.st == GncState.AUTO_MISSION && F.out.trk_sel == STOP && F.out.v_cap == 0;
            n = F.runUntil(@(f) f.out.trk_sel ~= STOP, 2*nD);
            c.resume_waits_t_deck = n + 1 == nD;
            % a pause keeps the deck where it is: no new wait after RESUME
            F.cmd(MissionCmd.PAUSE);  F.run(3);  F.cmd(MissionCmd.RESUME);  F.run(2);
            c.pause_no_wait = F.st == GncState.AUTO_MISSION && F.out.trk_sel ~= STOP;
            % moving with the deck up (only a deck report could show it): the deck-raised rows less dv bound v_cap
            F = t_gnc_mission.fsmDriver(P, planM);
            F.startAuto();  F.set('nav.v', 0.5);  F.run(1);
            c.moving_raised_level = F.out.trk_sel ~= STOP && abs(F.out.v_cap - ref(0, 1, false, true)) < 1e-12;
            F.set('nav.pitch', 10*pi/180);  F.run(1);
            c.moving_raised_10deg = F.out.v_cap == ref(10, 1, false, true);   % the table's deck-raised row, less dv
            F.set('nav.v', 0);  F.run(1);
            c.still_then_stop = F.out.trk_sel == STOP && F.out.v_cap == 0;
            % no implement: no deck wait
            F = t_gnc_mission.fsmDriver(P, plan);
            F.startAuto();  F.run(1);
            c.none_no_wait = F.out.trk_sel == uint8(TrackerSel.ROW) && F.out.v_cap > 0;
            % (b) mission bench, deck raised at power-up
            planB = tc.rowPlan(20, 0.8, 'MOWER');
            r = gnc_mission_run(P, planB, 'StopTime', 16, 'cmd', [1 2], 'mcu', [0 2 1 1 0 0 2 0], 'DeckDown', false);
            mw = P.structure.impl.mower;  ax = P.electronics.aux;
            t = r.t;
            auto = interp1(r.tf, double(r.f_state == double(GncState.AUTO_MISSION) | ...
                r.f_state == double(GncState.RETURN_TO_HOME)), t, 'previous', 0) > 0;
            hcmd = interp1(r.tc, r.deck_height_cmd, t, 'previous', P.gnc.impl.mower_h);
            x = mw.h_range(2);  h = zeros(size(t));  h(1) = x;    % x: lift actuator (potentiometer), h: deck, lagged
            for k = 2:numel(t)
                dt = t(k) - t(k - 1);
                if auto(k - 1)
                    duty = min(max(ax.lift_Kp*(hcmd(k - 1) - x), -ax.lift_duty_max), ax.lift_duty_max);   % + = up
                    x = min(max(x + mw.h_rate*duty*dt, mw.h_range(1)), mw.h_range(2));
                end
                h(k) = h(k - 1) + (x - h(k - 1))*(1 - exp(-dt/mw.h_tau));
            end
            raised = h > hcmd + pr.deck_tol;
            moving = abs(r.truth(:, 4)) > 1e-3 | abs(r.truth(:, 5)) > 1e-3;
            tAuto = tc.first(r, r.f_state == double(GncState.AUTO_MISSION));
            kMove = find(moving, 1);  kDown = find(~raised & t > 0, 1);
            tMove = NaN;  tDown = NaN;
            if ~isempty(kMove), tMove = t(kMove); end
            if ~isempty(kDown), tDown = t(kDown); end
            c.bench_deck_cmd_down = all(abs(double(r.deck_height_cmd) - P.gnc.impl.mower_h) < 1e-6);
            c.bench_never_moves_raised = ~any(raised & moving);
            c.bench_moves_after = r.truth(end, 1) > 1.0;
            f = fieldnames(c);
            ok = cellfun(@(x) c.(x), f);
            tc.report('mower_deck_down', sprintf(['t_deck %.2f s (lift %.3f m/s, from the top stop %.2f m to within %.2f m of %.2f m), ' ...
                'deck-raised cap less dv: level %.2f m/s, 10 deg %.2f m/s; checks passed %d of %d (failed: %s); bench: AUTO ' ...
                '%.2f s, deck within %.2f m of its command %.2f s, first motion %.2f s (%.2f s after AUTO), x at the end %.2f m'], ...
                pr.t_deck, ax.lift_duty_max*mw.h_rate, mw.h_range(2), pr.deck_tol, P.gnc.impl.mower_h, ...
                ref(0, 1, false, true), ref(10, 1, false, true), sum(ok), numel(ok), ...
                strjoin(f(~ok)', ' '), tAuto, pr.deck_tol, tDown, tMove, tMove - tAuto, r.truth(end, 1)));
            for k = 1:numel(f)
                tc.verifyTrue(c.(f{k}), f{k});
            end
        end
    end

    %% ================================ helpers =================================================
    methods
        function report(~, name, txt)
            fprintf('T15 MEASURED %s: %s\n', name, txt);
        end

        function plan = rowPlan(tc, len, v, impl)
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 len 0], 'home_enu', [0 0 0], 'speed_work', v);
            if nargin >= 4
                S.implement = struct('type', impl, 'speed_rpm', 2800, 'height_m', 0.06);
            end
            plan = gnc_compileMission(tc.P1, S, 'Validate', false);
        end

        function t = first(~, r, mask)
            i = find(mask, 1);
            if isempty(i), t = NaN; else, t = r.tf(i); end
        end

        function v = navV(~, r)
            v = interp1(r.t, r.truth(:, 4), r.tc);
        end

        function m = bypassRun(tc, vcTeb)
            P = shaddad_params('World', 3, 'Assign', false);
            S = struct('mission_type', 'GOTO', 'rows_enu', [16 20 16 80], 'home_enu', [16 20 pi/2], 'speed_work', 1.0, ...
                'allow_bypass', true, 'lane_halfwidth_m', 2.5);
            plan = gnc_compileMission(P, S, 'Validate', false);
            dyn = [1 16 35 0.5 0 0 0 1e5];                          % persistent 1 m obstacle on the lane centre
            r = gnc_mission_run(P, plan, 'StopTime', 90, 'cmd', [1 2], 'dyn', dyn, 'Teb', vcTeb);
            m.r = r;
            m.tH = tc.first(r, r.f_state == double(GncState.OBSTACLE_HOLD));
            m.tB = tc.first(r, r.f_sub == double(AutoSub.BYPASS) & r.f_state == double(GncState.AUTO_MISSION));
            m.tEnd = tc.first(r, r.tf > m.tB & r.f_sub == double(AutoSub.ROW) & r.f_state == double(GncState.AUTO_MISSION));
            m.clear = gnc_mission_metrics(r, 'clearance', dyn);
            e = r.truth(:, 1) - 16;
            m.dev = max(abs(e));
            k5 = find(r.truth(:, 2) >= 35.5 + 5, 1);
            m.xte5 = max(abs(e(k5:end)));
            m.half = 2.5;
        end

        function verifyBypass(tc, m, name)
            tc.report(name, sprintf(['hold %.2f s, BYPASS %.2f s (+%.2f), back on ROW %.2f s (bypass %.1f s); footprint ' ...
                'clearance %.3f m; max lateral deviation %.2f m (half-width %.1f); |XTE| from 5 m past the obstacle <= %.3f m; wall %.0f s'], ...
                m.tH, m.tB, m.tB - m.tH, m.tEnd, m.tEnd - m.tB, m.clear, m.dev, m.half, m.xte5, m.r.wall));
            tc.verifyLessThanOrEqual(abs(m.tB - m.tH - 10), 0.1 + 1e-9);
            tc.verifyGreaterThanOrEqual(m.clear, 0.3);
            tc.verifyLessThanOrEqual(m.dev, m.half);
            tc.verifyLessThan(m.xte5, 0.10);
            tc.verifyLessThan(m.tEnd - m.tB, 40);
        end

        function m = personHoldRun(tc, impl)
            % GNC-02 bench case (R-03): implement row at 0.5 m/s, MCU implement_ready true, a 0.3 m circle (the person)
            % switched on 1.5 m ahead of base_link at t_on. On its own the circle stays inside the standstill protective
            % field (protXF .. protXF + protL0), so 1.5 s later, the robot stopped, the person steps back 0.5 m to the deck
            % front, out of the field, and leaves at t_on + 6
            plan = tc.rowPlan(40, 0.5, impl);
            mcu = [0 2 1 1 0 0 2 0];
            t_on = 8;  t_back = t_on + 1.5;  t_gone = t_on + 6;
            r0 = gnc_mission_run(tc.P1, plan, 'StopTime', t_on, 'cmd', [1 2], 'mcu', mcu);
            x_on = interp1(r0.t, r0.truth(:, 1), t_on);
            dyn = [1 x_on + 1.5 0 0.3 0 0 t_on t_back; 1 x_on + 2.0 0 0.3 0 0 t_back t_gone];
            r = gnc_mission_run(tc.P1, plan, 'StopTime', t_gone + 5, 'cmd', [1 2], 'mcu', mcu, 'dyn', dyn);
            HOLD = double(GncState.OBSTACLE_HOLD);
            m.r = r;  m.impl = impl;  m.t_on = t_on;  m.t_back = t_back;  m.t_gone = t_gone;
            if strcmp(impl, 'MOWER'), m.en = r.mower_enable; else, m.en = r.tiller_enable; end
            m.tP = tc.first(r, r.f_state == HOLD & r.f_hold_reason == double(HoldReason.PERSON));
            m.tE = tc.first(r, r.tf > m.tP & r.f_state ~= HOLD);
            m.trans = r.f_trans_id(find(r.tf >= m.tE - 1e-9, 1));
            m.iHold = r.tc >= m.tP - 1e-9 & r.tc < m.tE - 1e-9;
            m.onInHold = sum(m.en(m.iHold) > 0)*tc.P1.gnc.Ts;
            m.enBefore = m.en(find(r.tc < m.tP - 1e-9, 1, 'last'));
            kf = find(r.to > m.tP & r.to < m.tE & r.o_person == 0, 1);
            if isempty(kf), m.tFall = NaN; else, m.tFall = r.to(kf); end
            kb = find(r.tc >= m.tE - 1e-9 & m.en > 0, 1);
            if isempty(kb), m.tBack = NaN; else, m.tBack = r.tc(kb); end
            m.edge = x_on + 1.7 - interp1(r.t, r.truth(:, 1), t_back);  % person edge ahead of base_link after the step back
            m.fieldEnd = tc.P1.sensors.per.protXF + tc.P1.sensors.per.protL0;   % standstill field end
        end

        function verifyPersonHold(tc, m, name)
            tc.report(name, sprintf(['%s enable before the person %d; PERSON hold at %.2f s (%.2f s after the person ' ...
                'appeared); the person steps back at %.1f s to %.3f m ahead of base_link (standstill field ends at %.2f m), ' ...
                'person_near off at %.2f s (%.2f s into the hold); implement on inside the hold %.2f s of %.2f s; hold end ' ...
                '%.2f s (transition %d, %.2f s after the person left); implement back on %.2f s'], m.impl, m.enBefore, m.tP, ...
                m.tP - m.t_on, m.t_back, m.edge, m.fieldEnd, m.tFall, m.tFall - m.tP, m.onInHold, m.tE - m.tP, m.tE, ...
                m.trans, m.tE - m.t_gone, m.tBack));
            tc.verifyEqual(m.enBefore, 1, 'implement on before the person');
            tc.verifyLessThanOrEqual(m.tP - m.t_on, 0.2 + 1e-9, 'PERSON hold within two scans');
            % the case is relevant: the field loses the person inside the hold, before H4 (t_mower_off) would stop the
            % implement anyway
            tc.verifyGreaterThan(m.edge, m.fieldEnd, 'the person stands beyond the standstill field');
            tc.verifyLessThan(m.tFall - m.tP, tc.P1.gnc.obs.t_mower_off, 'person_near falls inside the hold, before H4');
            tc.verifyEqual(m.onInHold, 0, 'implement off from the first PERSON hold until the hold ends');
            tc.verifyEqual(m.trans, 31, 'the hold ends by H1');
            tc.verifyLessThanOrEqual(abs(m.tE - m.t_gone - tc.P1.gnc.obs.t_clear), 0.1 + 1e-9, 'H1 t_clear after the person left');
            tc.verifyLessThanOrEqual(m.tBack - m.tE, 0.05 + 1e-9, 'implement back on with the hold end');
        end

        function ms = tebCallTime(~, r)
            % wall time per TEB call on the bench window: the TEB System object of the block with a 10 x 10 m
            % crop of the harness map at the bypass start (5 cells/m) and the detour reference
            P = r.P;  t = P.gnc.teb;
            k = find(r.tf >= r.tf(find(r.f_teb_enable > 0, 1)), 1);
            pose = interp1(r.t, r.truth(:, 1:3), r.tf(k));
            prm = struct('res', P.world.occ.res, 'origin', P.world.occ.origin, 'rmin', P.sensors.lidar.rMin, ...
                'rmax', P.gnc.tb.lidar_rmax, 'nBeams', P.gnc.obs.n_beams, 'on', 1, 'protXF', P.sensors.per.protXF, ...
                'protXR', P.sensors.per.protXR, 'protL0', P.sensors.per.protL0, 'protKv', P.sensors.per.protKv, ...
                'protW', P.sensors.per.protW, 'protStatic', 0, 'mapN', P.gnc.map.local_N, 'mapRes', P.gnc.map.local_res);
            clear gnc_harnessLidar
            [~, ~, ~, ~, occL, gl] = gnc_harnessLidar(pose(:), 0, 0, P.world.occ.grid, P.gnc.tb.dyn, prm);
            nc = t.map_window*P.gnc.map.local_res;  c0 = (P.gnc.map.local_N - nc)/2;
            c = occL(c0 + (1:nc), c0 + (1:nc));
            c = c(1:2:end, 1:2:end) | c(2:2:end, 1:2:end) | c(1:2:end, 2:2:end) | c(2:2:end, 2:2:end);
            MI = struct('Resolution', P.gnc.map.local_res/t.map_decim, 'GridLocationInWorld', gl + c0/P.gnc.map.local_res, ...
                'GridSize', size(c), 'OccupancyMatrix', c, 'GridOriginInLocal', -[1 1]*t.map_window/2);
            kv = find(r.rp_ver == max(r.rp_ver), 1);
            rp = squeeze(r.rp_path(:, :, kv));  if size(rp, 1) ~= 100, rp = rp'; end
            ref = [rp(:, 1:2) deg2rad(rp(:, 3))];
            T = nav.slalgs.internal.TimedElasticBand('ControlMode', "Path Following with Obstacle Avoidance", 'SampleTime', t.Ts, ...
                'LookAheadTime', t.LookAheadTime, 'GoalTolerance', t.GoalTolerance, 'MaxVelocity', t.MaxVelocity, ...
                'MaxAcceleration', t.MaxAcceleration, 'MinTurningRadius', t.MinTurningRadius, 'ReferenceDeltaTime', ...
                t.ReferenceDeltaTime, 'NumIteration', t.NumIteration, 'Time', t.Time, 'Smoothness', t.Smoothness, ...
                'Obstacle', t.Obstacle, 'Dimension', t.Dimension, 'ObstacleSafetyMargin', t.ObstacleSafetyMargin);
            T(pose(:), [0; 0], MI, ref);                           % first call (setup) not timed
            ms = zeros(20, 1);
            for k = 1:20
                t0 = tic;
                T(pose(:), [0.3; 0], MI, ref);
                ms(k) = 1e3*toc(t0);
            end
        end

        function [cov, ovl] = coverage(~, r, plan, P, info, what)
            % mower-deck footprint raster (cov.raster) over the floor inset by c_in, while mower_enable; overlap =
            % share of the cells mowed by the ROW passes that more than one pass mowed (sidelap), and the same share
            % counting the headland turns (the blades run in turns, spec 2.9)
            res = P.gnc.cov.raster;
            B = P.world.boundary;  %#ok<NASGU>
            xl = [0 9.6] + info.c_in*[1 -1];  yl = [0 37.5] + info.c_in*[1 -1];
            nx = round(diff(xl)/res);  ny = round(diff(yl)/res);
            cnt = zeros(ny, nx);  cntRow = cnt;  last = zeros(ny, nx);
            w = double(plan.swath);  dx = P.gnc.cov.mower_x;  dl = P.gnc.cov.mower_len;
            on = interp1(r.tc, r.mower_enable, r.t, 'previous', 0) > 0;
            seg = interp1(r.tf, r.f_seg_idx, r.t, 'previous', 0);
            kk = find(on)';
            kk = kk(1:5:end);                                       % 0.05 s: <= 0.04 m of travel per sample
            ext = hypot(dx + dl/2, w/2);
            nb = ceil(ext/res) + 1;
            for k = kk
                p = r.truth(k, 1:3);
                c = [cos(p(3)) sin(p(3))];
                % cells whose centres lie inside the deck rectangle (no raster dilation)
                j0 = floor((p(1) - xl(1))/res) + 1;  i0 = floor((p(2) - yl(1))/res) + 1;
                [J, I] = meshgrid(j0 + (-nb:nb), i0 + (-nb:nb));
                ok = I >= 1 & I <= ny & J >= 1 & J <= nx;
                I = I(ok);  J = J(ok);
                xc = xl(1) + (J - 0.5)*res - p(1);  yc = yl(1) + (I - 0.5)*res - p(2);
                lx = xc*c(1) + yc*c(2);  ly = -xc*c(2) + yc*c(1);
                in = abs(lx - dx) <= dl/2 & abs(ly) <= w/2;
                idx = sub2ind([ny nx], I(in), J(in));
                newSeg = last(idx) ~= seg(k);
                cnt(idx(newSeg)) = cnt(idx(newSeg)) + 1;
                if seg(k) >= 1 && plan.seg(seg(k), 1) == SegType.ROW
                    cntRow(idx(newSeg)) = cntRow(idx(newSeg)) + 1;
                end
                last(idx) = seg(k);
            end
            cov = nnz(cnt)/numel(cnt);
            ovl = [nnz(cntRow > 1)/max(1, nnz(cntRow)), nnz(cnt > 1)/max(1, nnz(cnt))];
            if nargin >= 6 && strcmp(what, 'overlap'), cov = ovl(1); end
        end

        function r = idealRows(~, plan)
            % the planned ROW passes as a 'run' (raw lane poses every 0.02 m, mower on) for the coverage raster
            X = [];  sg = [];
            for k = find(plan.seg(1:plan.n_seg, 1) == SegType.ROW)'
                q = double(plan.path((k - 1)*100 + (1:100), :));
                s = [0; cumsum(hypot(diff(q(:, 1)), diff(q(:, 2))))];
                ss = (0:0.02:s(end))';
                th = unwrap(deg2rad(q(:, 3)));
                X = [X; interp1(s, q(:, 1), ss) interp1(s, q(:, 2), ss) interp1(s, th, ss)]; %#ok<AGROW>
                sg = [sg; k*ones(numel(ss), 1)]; %#ok<AGROW>
            end
            n = size(X, 1);
            r = struct('t', (1:n)'*0.01, 'truth', [X zeros(n, 2)]);
            r.tc = r.t;  r.tf = r.t;  r.mower_enable = ones(n, 1);  r.f_seg_idx = sg;
        end

        function [hr, hp] = rowHeading(~, r)
            % heading error of the true pose to the lane direction on ROW segments (lanes of the plan are straight
            % in their middle part: first / last 3 m excluded), RMS and 99th percentile in degrees
            plan = r.plan;  e = [];
            for k = find(plan.seg(1:plan.n_seg, 1) == SegType.ROW)'
                q = double(plan.path((k - 1)*100 + (1:100), 1:2));
                u = q(end, :) - q(1, :);  Lk = norm(u);  u = u/Lk;
                m = r.f_seg_idx == k & r.f_sub == double(AutoSub.ROW);
                if ~any(m), continue; end
                p = interp1(r.t, r.truth(:, 1:3), r.tf(m));
                sAl = (p(:, 1:2) - q(1, :))*u';
                keep = sAl > 3 & sAl < Lk - 3;
                e = [e; angle(exp(1i*(p(keep, 3) - atan2(u(2), u(1)))))]; %#ok<AGROW>
            end
            hr = rad2deg(sqrt(mean(e.^2)));  hp = rad2deg(prctile(abs(e), 99));
        end

        function [xr, xm] = rowXte(~, r, plan)
            % truth cross-track error on ROW segments (sub ROW, 0.05 s grid), RMS and max
            e = [];
            rows = find(plan.seg(1:plan.n_seg, 1) == SegType.ROW);
            for k = rows'
                mask = r.f_seg_idx == k & r.f_sub == double(AutoSub.ROW) & r.f_state == double(GncState.AUTO_MISSION);
                if ~any(mask), continue; end
                q = double(plan.path((k - 1)*100 + (1:100), 1:2));
                tt = r.tf(mask);
                p = interp1(r.t, r.truth(:, 1:2), tt);
                % signed distance to the raw lane polyline (lanes may start with an S-curve lead-in)
                a = q(1:end - 1, :);  u = diff(q);  L2 = sum(u.^2, 2)';
                t = ((p(:, 1) - a(:, 1)').*u(:, 1)' + (p(:, 2) - a(:, 2)').*u(:, 2)')./L2;
                t = min(max(t, 0), 1);
                dx = p(:, 1) - (a(:, 1)' + t.*u(:, 1)');  dy = p(:, 2) - (a(:, 2)' + t.*u(:, 2)');
                [dmin, j] = min(hypot(dx, dy), [], 2);
                sg = sign(u(j, 1).*dy(sub2ind(size(dy), (1:numel(j))', j)) - u(j, 2).*dx(sub2ind(size(dx), (1:numel(j))', j)));
                sAl = [0; cumsum(sqrt(L2'))];
                sP = sAl(j) + t(sub2ind(size(t), (1:numel(j))', j)).*sqrt(L2(j))';
                keep = sP > 2 & sP < sAl(end) - 1;                  % settled part of the lane
                e = [e; sg(keep).*dmin(keep)]; %#ok<AGROW>
            end
            xr = sqrt(mean(e.^2));  xm = max(abs(e));
        end

        function in = insideGeofence(~, r, plan)
            b = plan.boundary(1:plan.n_bnd, :);
            in = inpolygon(r.truth(:, 1), r.truth(:, 2), b(:, 1), b(:, 2));
        end
    end

    methods (Static)
        function f = descRef(P)
            % expected downhill cap read straight from the table P.structure.descent, by its rule on the table's deg
            % axis (a reference for the descent tests, independent of gnc_descentCap): f(th, impl, rev) or
            % f(th, impl, rev, raised) = max(0, c - P.gnc.desc.dv), c the entry of the first breakpoint at or above th
            % deg (within P.gnc.desc.tol; level or uphill the first, above the last the last), row implement_type + 1
            % (any other implement: the lowest entry of the column; raised true with the mower: the deck-raised rows)
            Sd = P.structure.descent;  dv = P.gnc.desc.dv;  tl = P.gnc.desc.tol*180/pi;
            f = @(th, impl, rev, varargin) t_gnc_mission.descRefAt(Sd, dv, tl, th, impl, rev, ...
                ~isempty(varargin) && varargin{1});
        end

        function v = descRefAt(Sd, dv, tl, th, impl, rev, raised)
            % descRef at one pitch th (deg), implement_type impl, direction rev, deck raised
            j = find(Sd.theta_bp >= th - tl, 1);
            if isempty(j), j = numel(Sd.theta_bp); end
            if raised && impl == 1
                R = Sd.v_fwd_raised;
                if rev, R = Sd.v_rev_raised; end
            else
                R = Sd.v_fwd;
                if rev, R = Sd.v_rev; end
                if impl >= 0 && impl < size(R, 1) && impl == round(impl)
                    R = R(impl + 1, :);
                else
                    R = min(R, [], 1);
                end
            end
            v = max(0, R(j) - dv);
        end

        function p = laneProf(t, psi, sa, sc, valid)
            % gnc_ic_run profile of a position covariance whose axes lie along / across a lane at heading psi (ENU, CCW
            % from East): sigma sa along t = [cos psi; sin psi], sc across n = [-sin psi; cos psi] (testT7NavSigmaRule)
            c = cos(psi);  s = sin(psi);
            p = struct('t', t, 'sE', sqrt(c^2*sa.^2 + s^2*sc.^2), 'sN', sqrt(s^2*sa.^2 + c^2*sc.^2), ...
                'cEN', c*s*(sa.^2 - sc.^2), 'psi', psi*ones(size(t)), 'valid', logical(valid(:)), 'sa', sa, 'sc', sc);
        end

        function y = relayRef(x, on, off)
            % the Simulink Relay as measured (work/agentic/logs/20261114_s4fix_t7_explore2.log): on at x >= on, off at
            % x <= off, else the previous state; off at the start below the on point
            y = false(size(x));  s = false;
            for k = 1:numel(x)
                if x(k) >= on, s = true; elseif x(k) <= off, s = false; end
                y(k) = s;
            end
        end

        function [ids, M, chk] = fsmScenarios(P)
            % scripted unit scenarios of gnc_missionFSM (T11): every transition of spec 4.1, logged matrix
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 30 0; 30 4 0 4], 'home_enu', [0 0 0], 'speed_work', 0.8, ...
                'turn_radius_m', 2, 'skip_rows', 0, 'dock_enu', [-4 -4 0], 'end_action', 'RTH', 'allow_bypass', true, ...
                'lane_halfwidth_m', 2.5);
            plan = gnc_compileMission(P, S, 'Validate', false);
            Sm = S;  Sm.implement = struct('type', 'MOWER', 'speed_rpm', 2800, 'height_m', 0.06);
            planM = gnc_compileMission(P, Sm, 'Validate', false);
            F = t_gnc_mission.fsmDriver(P, plan);
            ids = [];  M = zeros(7);  chk = struct();
            % A: START ignored without drive_ready, START, ALIGN, segments to the dock, DOCKED, charging -> IDLE
            F.reset();  F.set('mcu.drive_ready', false);  F.set('nav.pose', [0; 0; pi]);  F.set('pe.epsi', pi);
            F.run(4);  F.cmd(MissionCmd.START);  F.run(3);
            chk.start_needs_drive_ready = F.st == GncState.IDLE;
            chk.gnc_ready_idle = F.out.gnc_ready;
            F.set('mcu.drive_ready', true);  F.cmd(MissionCmd.START);  F.run(2);
            chk.align_on_start = F.sub == AutoSub.ALIGN;
            F.set('nav.pose', [0; 0; 0]);  F.set('pe.epsi', 0);  F.run(3);
            F.drive(4000, @(f) f.st ~= GncState.AUTO_MISSION && f.st ~= GncState.RETURN_TO_HOME || f.sub == AutoSub.DOCKED);
            chk.docked_after_mission = F.sub == AutoSub.DOCKED;
            F.set('mcu.charging', true);  F.run(3);
            chk.docked_to_idle = F.st == GncState.IDLE;
            [ids, M] = F.collect(ids, M);
            % B: obstacle hold, release, bypass (TEB), S3, H3 timeout, resume with rejoin, S5
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(2);
            chk.hold = F.st == GncState.OBSTACLE_HOLD;
            F.set('obs.d_obs', inf);  F.run(round(2.2/0.05));
            chk.release = F.st == GncState.AUTO_MISSION;
            F.set('obs.d_obs', 1.0);  F.run(round(10.2/0.05));
            chk.bypass = F.sub == AutoSub.BYPASS;
            F.set('obs.d_obs', inf);  F.rpAnswer(true);  F.drive(400, @(f) f.sub ~= AutoSub.BYPASS);
            chk.bypass_done = F.sub == AutoSub.ROW;
            F.set('obs.d_obs', 1.0);  F.set('obs.person_near', true);  F.run(round(121/0.05));
            chk.hold_timeout = F.st == GncState.PAUSE && F.out.pause_reason == uint8(HoldReason.OBST_TIMEOUT);
            F.set('obs.d_obs', inf);  F.set('obs.person_near', false);  F.set('pe.xte', 0.5);
            F.cmd(MissionCmd.RESUME);  F.run(2);
            chk.resume_rejoin = F.sub == AutoSub.REJOIN;
            F.set('pe.xte', 0);  F.rpAnswer(true);  F.drive(400, @(f) f.sub ~= AutoSub.REJOIN);
            chk.rejoin_done = F.sub == AutoSub.ROW;
            [ids, M] = F.collect(ids, M);
            % B2: bypass with TEB failing (S4), replan failures -> PAUSE PLANNER_FAIL (S4 / R2-like)
            F.reset();  F.startAuto();  F.drive(40);
            F.set('obs.d_obs', 1.0);  F.run(round(10.2/0.05));  F.set('obs.d_obs', inf);
            F.rpAnswer(true);  F.run(3);  F.set('teb', [1; 1; 0; 1]);  F.run(round(3.5/0.05));
            chk.teb_fail_rejoin = F.sub == AutoSub.REJOIN;
            F.set('teb', [0; 12; 0; 1]);
            for k = 1:3, F.rpAnswer(false); end
            chk.planner_fail = F.st == GncState.PAUSE && F.out.pause_reason == uint8(HoldReason.PLANNER_FAIL);
            F.cmd(MissionCmd.ABORT);  F.run(2);
            [ids, M] = F.collect(ids, M);
            % C: manual, RC back to AUTO -> PAUSE, ABORT; MANUAL from IDLE and RC link lost (T4)
            F.reset();  F.startAuto();  F.drive(20);
            F.set('mcu.rc_mode_sw', uint8(0));  F.run(1);
            chk.manual_one_step = F.st == GncState.MANUAL && ~F.out.cmd_valid;
            F.set('mcu.rc_mode_sw', uint8(2));  F.run(1);
            chk.manual_to_pause = F.st == GncState.PAUSE;
            F.cmd(MissionCmd.ABORT);  F.run(1);
            chk.abort_idle = F.st == GncState.IDLE;
            F.set('mcu.rc_mode_sw', uint8(0));  F.run(2);  F.set('mcu.rc_link_ok', false);  F.run(2);
            chk.manual_link_lost = F.st == GncState.IDLE;
            F.set('mcu.rc_link_ok', true);  F.set('mcu.rc_mode_sw', uint8(2));  F.run(1);
            % T4 with an active mission -> PAUSE (RC_LOSS)
            F.startAuto();  F.drive(10);  F.set('mcu.rc_mode_sw', uint8(0));  F.run(1);
            F.set('mcu.rc_link_ok', false);  F.run(1);
            chk.manual_link_lost_pause = F.st == GncState.PAUSE && F.out.pause_reason == uint8(HoldReason.RC_LOSS);
            [ids, M] = F.collect(ids, M);
            % D: E-stop -> FAULT in one step; RESET refused with rc AUTO, accepted with rc HOLD
            F.reset();  F.startAuto();  F.drive(20);
            F.set('mcu.estop_active', true);  F.run(1);
            chk.estop_fault_one_step = F.st == GncState.FAULT && ~F.out.cmd_valid;
            F.set('mcu.estop_active', false);  F.cmd(MissionCmd.RESET_FAULT);  F.run(2);
            chk.reset_refused_auto = F.st == GncState.FAULT;
            F.set('mcu.rc_mode_sw', uint8(1));  F.cmd(MissionCmd.RESET_FAULT);  F.run(2);
            chk.reset_ok = F.st == GncState.IDLE && F.out.fault_word == 0;
            F.set('mcu.rc_mode_sw', uint8(2));
            [ids, M] = F.collect(ids, M);
            % E: nav degraded -> PAUSE (T7) -> auto resume (P2); operator PAUSE (T6) -> RESUME (P1); ABORT (T9)
            F.reset();  F.startAuto();  F.drive(20);
            F.set('nav.nav_degraded', true);  F.run(round(2.05/0.05));
            chk.nav_pause = F.st == GncState.PAUSE;
            F.set('nav.nav_degraded', false);  F.run(round(5.05/0.05));
            chk.nav_resume = F.st == GncState.AUTO_MISSION;
            F.cmd(MissionCmd.PAUSE);  F.run(1);  F.cmd(MissionCmd.RESUME);  F.run(1);
            chk.pause_resume = F.st == GncState.AUTO_MISSION;
            F.cmd(MissionCmd.ABORT);  F.run(1);
            [ids, M] = F.collect(ids, M);
            % F: SOC -> RTH (T8), R1, R6 (obstacle in RTH), H1 back to RTH, R3, R4, R5; R2 via failures; P3; T2
            F.reset();  F.startAuto();  F.drive(20);
            F.set('bms.SOC', single(0.20));  F.run(1);
            chk.soc_rth = F.st == GncState.RETURN_TO_HOME;
            F.set('bms.SOC', single(0.8));  F.rpAnswer(true, 6);  F.run(3);
            F.set('obs.d_obs', 1.0);  F.run(2);
            chk.rth_hold = F.st == GncState.OBSTACLE_HOLD;
            F.set('obs.d_obs', inf);  F.run(round(2.1/0.05));
            chk.rth_back = F.st == GncState.RETURN_TO_HOME;
            F.drive(2000, @(f) f.sub == AutoSub.DOCKED || f.st ~= GncState.RETURN_TO_HOME);
            chk.rth_docked = F.sub == AutoSub.DOCKED;
            F.run(round(2.1/0.05));
            chk.docked_timeout_idle = F.st == GncState.IDLE;
            F.cmd(MissionCmd.RTH);  F.run(2);
            chk.idle_rth = F.st == GncState.RETURN_TO_HOME;
            for k = 1:3, F.rpAnswer(false); end
            chk.rth_planner_fail = F.st == GncState.PAUSE;
            F.cmd(MissionCmd.RTH);  F.run(2);
            chk.pause_rth = F.st == GncState.RETURN_TO_HOME;
            [ids, M] = F.collect(ids, M);
            % G: drive_ready drop with RC link lost -> PAUSE RC_LOSS (36); implement not ready -> PAUSE (35)
            F.reset();  F.startAuto();  F.drive(20);
            F.set('mcu.rc_link_ok', false);  F.set('mcu.drive_ready', false);  F.run(1);
            chk.rc_loss = F.st == GncState.PAUSE && F.out.pause_reason == uint8(HoldReason.RC_LOSS);
            [ids, M] = F.collect(ids, M);
            Fm = t_gnc_mission.fsmDriver(P, planM);
            Fm.reset();  Fm.set('mcu.implement_ready', false);  Fm.startAuto();  Fm.run(round(10.1/0.05));
            chk.impl_timeout = Fm.st == GncState.PAUSE && Fm.out.pause_reason == uint8(HoldReason.IMPLEMENT_NOT_READY);
            [ids, M] = Fm.collect(ids, M);
            % H: faults in motion (nav lost, MCU heartbeat, geofence, tilt, XTE, stuck, BMS critical)
            % freeze: the heartbeat stops changing from this step, i.e. its last change was one step earlier (0.20 s age)
            G = {'nav.nav_ok', false, 0.30, 0; 'freeze', true, 0.15, 2; 'nav.pose', [0; -60; 0], 0.20, 9; ...
                 'nav.pitch', 0.5, 0, 5; 'pe.xte', 0.6, 3.0, 3; 'bms.bms_critical', true, 0, 6};
            for g = 1:size(G, 1)
                F.reset();  F.startAuto();  F.drive(10);
                F.set(G{g, 1}, G{g, 2});
                n = F.runUntil(@(f) f.st == GncState.FAULT, 200);
                chk.(sprintf('fault_%d', G{g, 4})) = abs(n*0.05 - G{g, 3}) < 0.025 + 1e-9 && ...
                    bitand(F.out.fault_word, uint32(2^G{g, 4})) > 0;
                [ids, M] = F.collect(ids, M);
            end
            F.reset();  F.startAuto();  F.drive(10);
            F.set('mcu.v_applied', single(0.5));  F.set('nav.v', 0);  F.hold = true;
            n = F.runUntil(@(f) f.st == GncState.FAULT, 200);
            chk.fault_4 = abs(n*0.05 - 5.0) < 0.025 + 1e-9;
            [ids, M] = F.collect(ids, M);
            ids = unique(ids);
        end

        function F = fsmDriver(P, plan)
            F = gnc_fsm_driver(P, plan);
        end

        function plan = fsmPlan(P)
            % the plan of fsmScenarios: two 30 m rows and a turn, dock, end action RTH, bypass allowed, 2.5 m lanes
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 30 0; 30 4 0 4], 'home_enu', [0 0 0], 'speed_work', 0.8, ...
                'turn_radius_m', 2, 'skip_rows', 0, 'dock_enu', [-4 -4 0], 'end_action', 'RTH', 'allow_bypass', true, ...
                'lane_halfwidth_m', 2.5);
            plan = gnc_compileMission(P, S, 'Validate', false);
        end

        function [kinds, n] = failAll(F, nmax)
            % answer every replan request of a waiting BYPASS / REJOIN with a failure until the FSM leaves AUTO; kinds =
            % replan_kind of the pending request and of each new one, n = failures answered
            kinds = double(F.out.replan_kind);  n = 0;
            while F.st == GncState.AUTO_MISSION && n < nmax
                F.rpAnswer(false);  n = n + 1;
                if F.st == GncState.AUTO_MISSION, kinds(end + 1) = double(F.out.replan_kind); end %#ok<AGROW>
            end
        end

        function req0 = pauseAndSet(F, xte, epsi)
            % operator PAUSE, then the path error the robot has when it is resumed; req0 = replan_req before the resume
            F.cmd(MissionCmd.PAUSE);  F.run(2);
            F.set('pe.xte', xte);  F.set('pe.epsi', epsi);
            req0 = F.out.replan_req;
        end

        function [okL, okG, dEnd, info] = s4RejoinPlans()
            % gnc_globalReplanner on the S4 rejoin geometry (full S4 run of 2026-11-13, call at 459 s): greenhouse_std in
            % world 4, its second ROW segment (lane 5, heading -90 deg), 19.26 m along it, 0.22 m left of its centre and
            % heading 175 deg off it; goal rejoin_off further along the lane (the FSM's REJOIN goal); static grid only
            P4 = shaddad_params('World', 4, 'Assign', false);
            plan = gnc_compileMission(P4, 'greenhouse_std');
            hp = P4.gnc.hastar;  N = hp.N_raw;  W = P4.world;
            rows = find(plan.seg(1:plan.n_seg, 1) == SegType.ROW);
            k = rows(2);
            q = double(plan.path((k - 1)*N + (1:N), :));
            s = [0; cumsum(hypot(diff(q(:, 1)), diff(q(:, 2))))];
            at = @(sq) [interp1(s, q(:, 1), sq), interp1(s, q(:, 2), sq)];
            th = deg2rad(q(1, 3));                                  % straight lane
            start = [at(19.26) + 0.22*[-sin(th) cos(th)], angle(exp(1i*(th + deg2rad(175))))];
            g = [at(19.26 + hp.rejoin_off), th];
            map = P4.ic.Bus_TebMapInfo;
            terr = struct('xE', W.terrain.xE, 'yN', W.terrain.yN, 'dHdE', W.terrain.dHdE, 'dHdN', W.terrain.dHdN);
            clear gnc_globalReplanner
            rl = gnc_globalReplanner(g(:), uint8(hp.kind_local), uint16(k), start(:), map, plan, W.occ.grid, W.occ.res, ...
                W.occ.origin, terr, hp);
            rg = gnc_globalReplanner(g(:), uint8(hp.kind_global), uint16(k), start(:), map, plan, W.occ.grid, W.occ.res, ...
                W.occ.origin, terr, hp);
            okL = rl.ok;  okG = rg.ok;
            pend = rg.path(end, :);
            dEnd = [hypot(pend(1) - g(1), pend(2) - g(2)), abs(rad2deg(angle(exp(1i*(deg2rad(pend(3)) - g(3))))))];
            info = struct('seg', k, 'start', [start(1:2) rad2deg(start(3))], 'desc', 'x y deg', 'tL', rl.t_plan, ...
                'tG', rg.t_plan, 'len', sum(hypot(diff(rg.path(:, 1)), diff(rg.path(:, 2)))));
        end

        function [ok, clr, tPlan, ratio, info] = hastarStatic(P)
            % T18: GLOBAL plan on a 100 x 100 m static map at 10 cells/m with keep-outs; comparison with the
            % shortest 8-connected grid path on the same inflated map (plannerAStarGrid)
            res = 10;  N = 1000;
            occ = false(N);
            [X, Y] = meshgrid(((1:N) - 0.5)/res, 100 - ((1:N)' - 0.5)/res);
            keep = [30 50 6; 55 30 5; 60 70 8; 80 50 4; 40 15 3; 20 80 5];
            for k = 1:size(keep, 1)
                occ = occ | hypot(X - keep(k, 1), Y - keep(k, 2)) <= keep(k, 3);
            end
            occ(:, [1:5 end-4:end]) = true;  occ([1:5 end-4:end], :) = true;
            mp = P.ic.Bus_MissionPlan;               % (not "plan": plannerAStarGrid.plan is called below)
            map = P.ic.Bus_TebMapInfo;
            start = [10 40 0];  goal = [92 62 pi/2];
            prm = P.gnc.hastar;
            clear gnc_globalReplanner
            flat = struct('xE', [0 100], 'yN', [0 100], 'dHdE', zeros(2), 'dHdN', zeros(2));
            rp = gnc_globalReplanner(goal, prm.kind_global, uint16(1), start, map, mp, occ, res, [0 0], flat, prm);
            ok = rp.ok;  tPlan = rp.t_plan;
            p = rp.path(:, 1:2);
            % clearance of the centre line to the keep-out discs
            d = inf;
            for k = 1:size(keep, 1)
                d = min(d, min(hypot(p(:, 1) - keep(k, 1), p(:, 2) - keep(k, 2)) - keep(k, 3)));
            end
            clr = d;
            info.len = sum(hypot(diff(p(:, 1)), diff(p(:, 2))));
            bm = binaryOccupancyMap(occ, res);  inflate(bm, prm.r_infl);
            ga = plannerAStarGrid(bm);
            gp = plan(ga, world2grid(bm, start(1:2)), world2grid(bm, goal(1:2)));
            gw = grid2world(bm, gp);
            info.lenGrid = sum(hypot(diff(gw(:, 1)), diff(gw(:, 2))));
            ratio = info.len/info.lenGrid;
            info.nKeep = size(keep, 1);
            % LOCAL: lane-masked detour around a 1 m obstacle on a lane (perception grid 30 x 30 m)
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 40 0], 'home_enu', [0 0 0], 'speed_work', 1.0, ...
                'allow_bypass', true, 'lane_halfwidth_m', 2.5);
            pl = gnc_compileMission(P, S, 'Validate', false);
            m = P.ic.Bus_TebMapInfo;
            m.Resolution = 10;  m.GridSize = [300 300];  m.GridOriginInLocal = [-15 -15];  m.GridLocationInWorld = [-5 -15];
            [mx, my] = meshgrid(-5 + ((1:300) - 0.5)/10, 15 - ((1:300)' - 0.5)/10);
            m.OccupancyMatrix = hypot(mx - 12.5, my) <= 0.5;
            clear gnc_globalReplanner
            rl = gnc_globalReplanner([18.7 0 0], prm.kind_local, uint16(1), [10 0 0], m, pl, false(10), 10, [-100 -100], ...
                flat, prm);
            info.localOk = rl.ok;
            info.localDev = max(abs(rl.path(:, 2)));
            info.half = 2.5;
            % slope (D53): world 2 has a 20 deg ramp for x in [10, 30] m; a GLOBAL plan that must reverse its heading
            % (start heading East at x = 15 m, goal heading West at x = 28 m) may not turn on the ramp
            P2 = shaddad_params('World', 2, 'Assign', false);
            T = P2.world.terrain;
            terr = struct('xE', T.xE, 'yN', T.yN, 'dHdE', T.dHdE, 'dHdN', T.dHdN);
            clear gnc_globalReplanner
            rs = gnc_globalReplanner([28 4 pi], prm.kind_global, uint16(1), [15 0 0], P2.ic.Bus_TebMapInfo, P2.ic.Bus_MissionPlan, ...
                P2.world.occ.grid, P2.world.occ.res, P2.world.occ.origin, terr, P2.gnc.hastar);
            nb = max(1, round(P2.gnc.hastar.slope_base/mean(diff(T.xE))));     % DEM slope over the footprint length
            kb = ones(nb)/nb^2;
            gx = griddedInterpolant({T.xE, T.yN}, conv2(T.dHdE, kb, 'same'), 'linear', 'nearest');
            gy = griddedInterpolant({T.xE, T.yN}, conv2(T.dHdN, kb, 'same'), 'linear', 'nearest');
            q = rs.path;
            sl = atan(hypot(gx(q(:, 1), q(:, 2)), gy(q(:, 1), q(:, 2))));
            fall = atan2(gy(q(:, 1), q(:, 2)), gx(q(:, 1), q(:, 2)));
            onSteep = sl > P2.gnc.hastar.slope_turn_max & hypot(q(:, 1) - 15, q(:, 2)) > P2.gnc.hastar.R_min;
            dev = asind(abs(sin(deg2rad(q(:, 3)) - fall)));    % heading deviation from the fall line
            info.slopeOk = rs.ok;
            info.slopeDev = max([0; dev(onSteep)]);
            info.slopeTol = rad2deg(P2.gnc.hastar.slope_head_tol);
            info.slopeTurnX = [min(q(q(:, 1) > 25, 1)) max(q(:, 1))];   % the U-turn lies beyond the ramp end (x = 30)
            info.uturnX = min(q(abs(q(:, 3)) > 30 & abs(q(:, 3)) < 150, 1));
        end
    end
end
