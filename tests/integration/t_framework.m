classdef t_framework < matlab.unittest.TestCase
    %T_FRAMEWORK  Integration framework tests T-INT-01..12 (build plan T16.6, integration spec 9.1).
    %
    %   Tags: 'Fast' (unit benches and offline checks, part of run_regression('Fast')),
    %         'Integration' (top-model compiles, rate audit, rebuilds; minutes),
    %         'System' (variant matrix, determinism, fixed vs reference solver, speed, restart; long).
    %   System results that need full scenario runs (T-INT-04 IMU / GNSS / pitch, T-INT-06, T-INT-08) read
    %   the latest run_scenario results in work/results (scn_latest); they are Incomplete when absent.
    %   Code generation and accelerator items are BLOCKED (no Simulink Coder, no C compiler, D31).

    properties
        P
    end

    properties (TestParameter)
        % T-INT-10 covering array: [VC_PHASE VC_DRIVE_FID VC_IMPL VC_PACK VC_VIZ VC_TEB VC_OPSRC] (every value
        % of every control at least once; phase x drive x pack x opsrc pairs covered)
        vm = struct('r1', [1 1 0 1 0 1 1], 'r2', [1 3 1 2 2 1 2], 'r3', [1 1 2 2 1 1 3], 'r4', [1 3 3 1 0 1 1], ...
                    'r5', [2 1 0 2 0 0 2], 'r6', [2 3 1 1 1 1 1], 'r7', [2 1 2 1 2 0 3], 'r8', [2 3 3 2 0 1 2]);
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_startup('quiet'); end
            tc.P = t_framework.freshP();
        end
    end

    methods (Static)
        function P = freshP()
            % base P with the current integration fields (param_sim) merged in
            P = evalin('base', 'P');
            S = param_sim();
            P.sim = S.sim;  P.scen = S.scen;  P.viz = S.viz;  P.log = S.log;
        end

        function [o, kpi] = runMon(P, spec, T)
            % Sim_Monitors unit bench (tests/integration/tb_sim_monitors.slx)
            if ~bdIsLoaded('tb_sim_monitors'), load_system('tb_sim_monitors'); end
            in = Simulink.SimulationInput('tb_sim_monitors');
            in = in.setVariable('P', P).setExternalInput(mon_tb_input(P, spec, T)).setModelParameter('StopTime', num2str(T), ...
                'CaptureErrors', 'on');
            o = sim(in);
            kpi = [];
            try
                kpi = o.yout.get('kpi').Values;
            catch
            end
        end

        function sep = oracle(fp, obs, kind)
            % robotics collision oracle (same algorithm as robotslcollision 'Check Collision'): footprint
            % box P.sim.mon.footprint x 0.9 m at fp = [x y psi], obstacle 'cyl' [x y r] or 'box' [x y yaw hx hy]
            fpLW = evalin('base', 'P.sim.mon.footprint');
            B = collisionBox(fpLW(1), fpLW(2), 0.9);
            B.Pose = trvec2tform([fp(1) fp(2) 1])*eul2tform([fp(3) 0 0]);
            if strcmp(kind, 'cyl')
                O = collisionCylinder(obs(3), 2);
                O.Pose = trvec2tform([obs(1) obs(2) 1]);
            else
                O = collisionBox(2*obs(4), 2*obs(5), 2);
                O.Pose = trvec2tform([obs(1) obs(2) 1])*eul2tform([obs(3) 0 0]);
            end
            [c, sep] = checkCollision(B, O);
            if c, sep = 0; end
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'Fast'})
        function testMonitorSepDist(tc)
            % T-INT-12: footprint box at the origin vs cylinder r = 0.3 m at (1.5, 0): SepDist 1.5 - L/2 - 0.3 +/- 0.01
            P = tc.P;
            P.world.obstacles.cyl = [1.5 0 0.3 2 0 1];  P.world.obstacles.nCyl = 1;
            spec.power_truth.SOC_pack_true = [0 0.9];  spec.power_truth.T_cell_max_K = [0 300];
            [o, k] = t_framework.runMon(P, spec, 1);
            tc.assertEmpty(o.ErrorMessage);
            tc.verifyEqual(k.clear_min.Data(end), 1.5 - P.sim.mon.footprint(1)/2 - 0.3, 'AbsTol', 0.01);
            tc.verifyFalse(any(k.collision.Data));
            tc.verifyEqual(double(k.stop_reason.Data(end)), 0);
            ref = t_framework.oracle([0 0 0], [1.5 0 0.3], 'cyl');
            tc.verifyEqual(k.clear_min.Data(end), ref, 'AbsTol', 1e-3);
        end

        function testMonitorCollisionStop(tc)
            % T-INT-12: overlapping -> CollStatus = 1 and Stop Simulation within 0.1 s (stop_reason 4)
            P = tc.P;
            xc = P.sim.mon.footprint(1)/2 + 0.3 - 0.2;        % 0.2 m overlap at 0.5 s
            P.world.obstacles.cyl = [xc 0 0.3 2 0 1];  P.world.obstacles.nCyl = 1;
            spec.power_truth.SOC_pack_true = [0 0.9];
            spec.truth.pose2D = [0 -3 0 0; 0.5 0 0 0];       % footprint jumps onto the cylinder at 0.5 s
            [o, k] = t_framework.runMon(P, spec, 2);
            tc.assertEmpty(o.ErrorMessage);
            tc.verifyTrue(any(k.collision.Data));
            tc.verifyEqual(double(k.stop_reason.Data(end)), 4);
            tc.verifyLessThanOrEqual(k.stop_reason.Time(end), 0.5 + 0.1 + 1e-9);
            tc.verifyEqual(double(k.n_coll.Data(end)), 1);
        end

        function testMonitorNaNAssertion(tc)
            % T-INT-12: NaN injected into truth -> the assertion stops the run with stop_reason = 5
            P = tc.P;
            spec.power_truth.SOC_pack_true = [0 0.9];
            spec.truth.vel_W = [0 0 0 0; 0.7 NaN 0 0];
            [o, k] = t_framework.runMon(P, spec, 3);
            tc.verifyNotEmpty(o.ErrorMessage, 'the NaN assertion must stop the run');
            tc.verifyTrue(contains(o.ErrorMessage, 'NaN_Assertion') || contains(lower(o.ErrorMessage), 'assertion'));
            tc.verifyEqual(double(k.stop_reason.Data(end)), 5);
            tc.verifyLessThan(k.stop_reason.Time(end), 1.0);
        end

        function testMonitorBoxOracle(tc)
            % T-INT-12 / D42: oriented boxes and film segments against the robotics collision oracle
            P = tc.P;
            rng(20261022, 'twister');
            fp = [1.0 2.0 0.4];
            for k = 1:6
                bx = [fp(1) + 3*(rand - 0.5), fp(2) + 3*(rand - 0.5), 0, 0.3 + rand, 0.2 + rand, 1, pi*(rand - 0.5), 5, k];
                P.world.obstacles.box = bx;  P.world.obstacles.nBox = 1;
                P.world.obstacles.nCyl = 0;  P.world.obstacles.nFilm = 0;
                spec = struct();
                spec.power_truth.SOC_pack_true = [0 0.9];
                spec.truth.pose2D = [0 fp];
                [o, kp] = t_framework.runMon(P, spec, 0.3);
                tc.assertEmpty(o.ErrorMessage);
                L = sim_monitor_obstacles(P.world, 'box', P.sim.mon);
                ref = t_framework.oracle(fp, L(1, :), 'box');
                sep = o.logsout.get('mon_sep_box').Values.Data(end);
                if ref > 0
                    tc.verifyEqual(sep, ref, 'AbsTol', 2e-3, sprintf('box %d', k));
                else
                    tc.verifyLessThanOrEqual(sep, 0, sprintf('box %d overlaps', k));
                end
                tc.verifyEqual(kp.collision.Data(end), sep <= 0);
            end
            % film segment -> 0.02 m thick box
            P.world.obstacles.nBox = 0;
            P.world.obstacles.film = [0 4 6 4];  P.world.obstacles.nFilm = 1;
            spec = struct();  spec.power_truth.SOC_pack_true = [0 0.9];  spec.truth.pose2D = [0 3 3 0];
            o = t_framework.runMon(P, spec, 0.3);
            sep = o.logsout.get('mon_sep_box').Values.Data(end);
            tc.verifyEqual(sep, 4 - 0.01 - 3 - 0.55, 'AbsTol', 1e-6);
        end

        function testNearestKBruteForce(tc)
            % T-INT-11 (monitor part; terrain grades / soil are T02's structure_env tests, D10): with 60
            % random cylinders the nearest-K selection + For Each give the brute-force minimum separation
            P = tc.P;
            rng(7, 'twister');
            n = 60;
            C = [40*rand(n, 2) - 20, 0.1 + 0.4*rand(n, 1), 2*ones(n, 1), zeros(n, 1), ones(n, 1)];
            P.world.obstacles.cyl = C;  P.world.obstacles.nCyl = n;
            fp = [0.5 -0.3 0.7];
            spec.power_truth.SOC_pack_true = [0 0.9];  spec.truth.pose2D = [0 fp];
            P.scen.stop_on_collision = false;
            o = t_framework.runMon(P, spec, 0.3);
            tc.assertEmpty(o.ErrorMessage);
            ref = inf;
            for k = 1:n
                ref = min(ref, t_framework.oracle(fp, C(k, 1:3), 'cyl'));
            end
            sep = o.logsout.get('mon_sep_cyl').Values.Data(end);
            if ref > 0
                tc.verifyEqual(sep, ref, 'AbsTol', 2e-3);
            else
                tc.verifyLessThanOrEqual(sep, 0);
            end
            % the K = 12 rows sorted by the metric |c - p| - r are the 12 smallest
            L = sim_monitor_obstacles(P.world, 'cyl', P.sim.mon);
            m = hypot(L(:, 1) - fp(1), L(:, 2) - fp(2)) - L(:, 3);
            [~, ord] = sort(m);
            tc.verifyEqual(numel(unique(ord(1:12))), 12);
        end

        function testKinematicOracle(tc)
            % T-INT-04 (E1): v_L = v_R = 1 m/s for 10 s from psi0 = 0 -> x = +10 m +/- 2 %, |y| < 0.05 m;
            % psi0 = +90 deg -> y = +10 m; v_R > v_L -> psi increasing (omega > 0, turning left)
            if ~bdIsLoaded('tb_frames'), load_system('tb_frames'); end
            P = tc.P;
            r = P.drive.gear.r_spr;
            run = @(w, x0) sim(Simulink.SimulationInput('tb_frames').setVariable('P', P) ...
                .setVariable('tb_frames_w', w).setVariable('tb_frames_x0', x0).setModelParameter('StopTime', '10'));
            last = @(o) local_last_state(o.yout.get('state').Values);
            s = last(run([1 1]/r, [0; 0; 0]));
            tc.verifyEqual(s(1), 10, 'RelTol', 0.02);
            tc.verifyLessThan(abs(s(2)), 0.05);
            s = last(run([1 1]/r, [0; 0; pi/2]));
            tc.verifyEqual(s(2), 10, 'RelTol', 0.02);
            s = last(run([0.9 1.1]/r, [0; 0; 0]));
            tc.verifyGreaterThan(s(3), 0);
        end

        function testEvalCriteria(tc)
            S.a.b = struct('t', (0:0.1:1)', 'v', (0:0.1:1)');
            crit = struct('id', {'x1', 'x2', 'x3', 'x4'}, 'desc', '', 'signal', '', ...
                'metric', {@(S) S.a.b.v(end), @(S) S.a.b.v(end), @(S) error('boom'), @(S) 1}, ...
                'op', {'<=', 'in', '<', '>'}, 'limit', {1, [0.5 0.9], 1, 0}, 'unit', '', 'blocked', {'', '', '', 'BLOCKED: x'});
            r = eval_criteria(S, crit, 'Print', false);
            tc.verifyEqual({r.status}, {'PASS', 'FAIL', 'ERROR', 'BLOCKED'});
            tc.verifyEqual(scn_metrics('first', S.a.b, @(v) v >= 0.5, 0), 0.5, 'AbsTol', 1e-12);
            tc.verifyEqual(scn_metrics('pulse', S.a.b, @(v) v >= 0.3 & v < 0.65, 0), 0.4, 'AbsTol', 1e-12);
            tc.verifyEqual(scn_metrics('at', S.a.b, 0.55), 0.5, 'AbsTol', 1e-12);
        end

        function testKpiCoverageXte(tc)
            % synthetic straight lane: full coverage of a 1.0 m wide strip, XTE = constant offset
            P = tc.P;
            plan = P.ic.Bus_MissionPlan;
            plan.valid = true;  plan.n_seg = uint16(1);
            nr = size(plan.path, 1)/size(plan.seg, 1);
            plan.path(1:nr, 1) = single(5);  plan.path(1:nr, 2) = single(linspace(0, 20, nr));
            plan.seg(1, 1) = single(2);  plan.seg(1, 6) = single(1);
            t = (0:0.01:40)';
            S.truth.pose2D = struct('t', t, 'v', [5.03*ones(size(t)), 0.5*t, pi/2*ones(size(t))]);
            S.gnc_status.seg_idx = struct('t', t, 'v', uint16(ones(size(t))));
            S.gnc_status.gnc_state = struct('t', t, 'v', uint8(2*ones(size(t))));
            area = [4.6 1; 5.4 1; 5.4 19; 4.6 19];
            c = kpi_coverage(S, P, plan, area, 'Swath', 1.0);
            tc.verifyGreaterThan(c, 0.99);
            [x, info] = kpi_truth_xte(S, plan);
            tc.verifyEqual(info.rms, 0.03, 'AbsTol', 1e-6);
            tc.verifyEqual(numel(x), info.n);
        end

        function testBirdsEyeOffline(tc)
            P = tc.P;
            b = shaddad.BirdsEyeView('MapImage', P.world.occ.grid, 'MapResolution', P.world.occ.res, ...
                'MapOrigin', P.world.occ.origin, 'Visible', false, 'PathXY', [0 0; 10 0]);
            for k = 1:3
                b([k; 0; 0], [k; 0.1; 0], single(4*ones(360, 1)), single(linspace(-pi, pi, 360)'), uint8(3), 0.9);
            end
            tc.verifyEqual(b.frameCount, 3);
            f = b.figureHandle;
            tc.verifyTrue(isvalid(f));
            release(b);  close(f);
            xy = shaddad.BirdsEyeView.planPath(P.ic.Bus_MissionPlan);
            tc.verifyEqual(size(xy, 2), 2);
        end

        function testScenarioInputs(tc)
            % every scenario script builds a SimulationInput on Shaddad_System and well-formed criteria
            ids = {'S0', {}; 'S1', {}; 'S2', {}; 'S6', {'Variant', 'a'}; 'S6', {'Variant', 'b'}; 'S6', {'Variant', 'c'}; ...
                   'S8', {'Variant', 'a'}; 'S8', {'Variant', 'b'}; 'S7', {'Variant', 'farm'}};
            root = char(currentProject().RootFolder);
            for k = 1:size(ids, 1)
                f = dir(fullfile(root, 'scripts', 'scenarios', ['scn_' ids{k, 1} '_*.m']));
                tc.assertNotEmpty(f, ids{k, 1});
                [~, fn] = fileparts(f(1).name);
                [in, crit] = feval(fn, tc.P, ids{k, 2}{:});
                tc.verifyClass(in, 'Simulink.SimulationInput');
                tc.verifyEqual(in.ModelName, 'Shaddad_System');
                tc.verifyNotEmpty(crit);
                tc.verifyTrue(all(arrayfun(@(c) isa(c.metric, 'function_handle'), crit)));
                v = in.Variables;  nm = {v.Name};
                tc.verifyTrue(all(ismember({'P', 'Ts', 'VC_PHASE', 'VC_IMPL'}, nm)), ids{k, 1});
                ds = in.ExternalInput;
                tc.verifyEqual(ds.getElementNames, {'ScenarioIn'});
                op = ds.getElement(1).op;
                if ~strcmp(ids{k, 1}, 'S0')
                    tc.verifyEqual(double(op.tx_power_on.Data(end)), 1, [ids{k, 1} ': transmitter must be on (T07)']);
                end
            end
        end

        function testInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'Sim_Monitors', 'Viz_Logging', 'Shaddad_System'}, 'Quiet', true);
            tc.verifyEqual(n, 0);
        end

        function testLoggingManifest(tc)
            % T-INT-06 (manifest part): every manifest port logs under its name; missions decimate to 0.1 s
            L = scn_logging('table');
            if ~bdIsLoaded('Shaddad_System'), load_system('Shaddad_System'); end
            for k = 1:size(L, 1)
                ph = get_param(['Shaddad_System/' L{k, 2}], 'PortHandles');
                p = ph.Outport(L{k, 3});
                tc.verifyEqual(get_param(p, 'DataLogging'), 'on', L{k, 1});
                tc.verifyEqual(get_param(p, 'DataLoggingName'), L{k, 1});
            end
            mi = scn_logging('override', tc.P, 4*3600);
            dec = arrayfun(@(s) s.LoggingInfo.Decimation, mi.Signals(1:size(L, 1)));
            tc.verifyEqual(dec(:), max(1, round(tc.P.log.Ts_mission./cell2mat(L(:, 5)))));
            tc.verifyEmpty(scn_logging('override', tc.P, 120));
        end

        function testJoystickPlaceholder(tc)
            % D62: 2 s smoke test of vrlib/Joystick Input; when it fails, VC_OPSRC 3 is a Constant placeholder
            m = 'tmp_T16_joysmoke';
            if bdIsLoaded(m), close_system(m, 0); end
            new_system(m);
            cl = onCleanup(@() close_system(m, 0));
            ok = true;
            try
                load_system('vrlib');
                add_block('vrlib/Joystick Input', [m '/J']);
                ph = get_param([m '/J'], 'PortHandles');
                for k = 1:numel(ph.Outport)
                    add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', m, k));
                    add_line(m, sprintf('J/%d', k), sprintf('T%d/1', k));
                end
                sim(m, 'StopTime', '2', 'SolverType', 'Fixed-step', 'FixedStep', '0.01');
            catch
                ok = false;
            end
            if ~bdIsLoaded('Shaddad_System'), load_system('Shaddad_System'); end
            j = get_param('Shaddad_System/Operator_Console/Joystick', 'BlockType');
            if ok
                tc.verifyFalse(strcmp(j, 'Constant'), 'smoke test passed: choice 3 should map vrlib/Joystick Input');
            else
                tc.verifyEqual(j, 'Constant', 'smoke test failed: choice 3 must stay a Constant placeholder');
            end
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'Integration'})
        function testSystemUpdate(tc)
            % T-INT-01: update of Shaddad_System for VC_PHASE 1 and 2 < 180 s with 0 algebraic loops
            % (AlgebraicLoopMsg = error, so a successful update proves 0 loops) and 0 implicit RTs
            % (AutoInsertRateTranBlk = off, MultiTaskRateTransMsg = error)
            cs = evalin('base', 'cs_Shaddad');
            tc.verifyEqual(get_param(cs, 'AlgebraicLoopMsg'), 'error');
            tc.verifyEqual(get_param(cs, 'AutoInsertRateTranBlk'), 'off');
            if ~bdIsLoaded('Shaddad_System'), load_system('Shaddad_System'); end
            ph0 = evalin('base', 'VC_PHASE');
            cl = onCleanup(@() assignin('base', 'VC_PHASE', ph0));
            for ph = [1 2]
                assignin('base', 'VC_PHASE', ph);
                t0 = tic;
                ok = false;
                for attempt = 1:2
                    try
                        evalc('set_param(''Shaddad_System'', ''SimulationCommand'', ''update'')');
                        ok = true;
                        break
                    catch tmp_tf_err
                        msg = tmp_tf_err.message;
                        if ~contains(msg, 'o != nullptr'), break, end
                    end
                end
                dt = toc(t0);
                tc.verifyTrue(ok, sprintf('VC_PHASE %d update failed', ph));
                tc.verifyLessThan(dt, 180, sprintf('VC_PHASE %d update took %.0f s', ph, dt));
            end
        end

        function testRateAudit(tc)
            % T-INT-05: compiled rates subset of P.sim.Ts_compiled_allowed, 0.007/0.014 only in RC_Link_Sim
            rep = shaddad_rate_audit('Shaddad_System');
            tc.verifyEmpty(rep.bad);
            % the Operator_RC component (the virtual subsystem around the RC_Link_Sim instance, layout spec 5)
            % reports the instance's internal frame rate; the instance and its blocks lie inside the audit's
            % RCPath <top>/Operator_RC, so only blocks outside it count
            out = rep.rc_outside;
            if ~isempty(out)
                keep = ~strcmp(out(:, 1), 'Shaddad_System/Operator_RC');
                out = out(keep, :);
            end
            tc.verifyEmpty(out);
            tc.verifyTrue(all(strcmp(rep.autoRT, 'off')));
        end

        function testBuildIdempotent(tc)
            % the T16 build scripts regenerate their files at the exact paths
            root = char(currentProject().RootFolder);
            files = {fullfile(root, 'subsystems', 'integration', 'Sim_Monitors.slx'), @build_sim_monitors; ...
                     fullfile(root, 'subsystems', 'integration', 'Viz_Logging.slx'), @build_viz_logging; ...
                     fullfile(root, 'tests', 'integration', 'tb_sim_monitors.slx'), @build_tb_sim_monitors; ...
                     fullfile(root, 'tests', 'integration', 'tb_frames.slx'), @build_tb_frames};
            for k = 1:size(files, 1)
                d0 = dir(files{k, 1});
                files{k, 2}();
                d1 = dir(files{k, 1});
                tc.verifyNotEmpty(d1, files{k, 1});
                tc.verifyGreaterThanOrEqual(d1.datenum, d0.datenum);
            end
            [~, nm] = fileparts(files{1, 1});
            tc.verifyNotEqual(get_param(nm, 'Description'), 'SHADDAD_STUB');
            tc.verifyEqual(shaddad_check_interfaces('Files', {'Sim_Monitors', 'Viz_Logging'}, 'Quiet', true), 0);
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'System'})
        function testVariantMatrix(tc, vm)
            % T-INT-10: one row of the covering array of VC_PHASE x VC_DRIVE_FID {1,3} x VC_IMPL 0..3 x VC_PACK
            % x VC_VIZ x VC_TEB x VC_OPSRC {1,2,3 (placeholder)} compiles (update diagram, 0 algebraic loops)
            names = {'VC_PHASE', 'VC_DRIVE_FID', 'VC_IMPL', 'VC_PACK', 'VC_VIZ', 'VC_TEB', 'VC_OPSRC'};
            old = cellfun(@(n) evalin('base', n), names);
            cl = onCleanup(@() cellfun(@(n, v) assignin('base', n, v), names, num2cell(old)));
            if ~bdIsLoaded('Shaddad_System'), load_system('Shaddad_System'); end
            for c = 1:numel(names), assignin('base', names{c}, vm(c)); end
            ok = false;  msg = '';
            for attempt = 1:2
                try
                    evalc('set_param(''Shaddad_System'', ''SimulationCommand'', ''update'')');
                    ok = true;  break
                catch tmp_tf_err
                    msg = tmp_tf_err.message;
                    for q = 1:min(3, numel(tmp_tf_err.cause))
                        msg = [msg ' | ' tmp_tf_err.cause{q}.message]; %#ok<AGROW>
                    end
                    if ~contains(msg, 'o != nullptr'), break, end
                end
            end
            tc.verifyTrue(ok, sprintf('%s: %s', mat2str(vm), msg));
        end

        function testDeterminism(tc)
            % T-INT-07: two identical runs give identical logged pose (S0, 10 s); parsim vs serial is
            % BLOCKED (no memory for parallel workers in the shared session)
            r1 = run_scenario('S0', 'Save', false, 'KeepOut', false);
            r2 = run_scenario('S0', 'Save', false, 'KeepOut', false);
            tc.assertTrue(r1.ok && r2.ok);
            d = max(abs(r1.S.truth.pos_W.v(:) - r2.S.truth.pos_W.v(:)));
            tc.verifyEqual(d, 0);
            tc.verifyEqual(max(abs(double(r1.S.telem.mode.v(:)) - double(r2.S.telem.mode.v(:)))), 0);
        end

        function testFixedVsRef(tc)
            % T-INT-02: fixed (ode3 + local BE solvers) vs reference (daessc, local solvers off) on S1
            % (first 20 s: arm, 1.2 m/s step, steady): RMS speed difference <= 1 % of max |v|
            % precondition: the robot moves in S1 (otherwise both runs stand still and the comparison is void)
            r1 = scn_latest('S1');
            if ~isempty(r1)
                vmax = max(abs(double(r1.S.truth.speed_fwd.v)));
                tc.assertGreaterThan(vmax, 0.5, sprintf(['S1 precondition: the robot does not move (max |v| = %.3g m/s; ' ...
                    'ESC tau_avail / MCU tau_lim_mot zero fixed point, see the T16 report) - comparison void'], vmax));
            end
            rf = run_scenario('S1', 'StopTime', 20, 'Save', false, 'KeepOut', false);
            rr = run_scenario('S1', 'StopTime', 20, 'Solver', 'ref', 'Save', false, 'KeepOut', false);
            tc.assertTrue(rf.ok, rf.msg);
            tc.assertTrue(rr.ok, rr.msg);
            t = (0:0.01:20)';
            vf = scn_metrics('resample', rf.S.truth.speed_fwd, t);
            vr = scn_metrics('resample', rr.S.truth.speed_fwd, t);
            tc.verifyLessThanOrEqual(sqrt(mean((vf - vr).^2)), 0.01*max(abs(vf)));
        end

        function testSpeed(tc)
            % T-INT-03: RTF targets need accelerator (C compiler): BLOCKED; the Normal-mode RTF and the
            % Simulink Profiler report of a short S1 run are produced for the record
            r = run_scenario('S1', 'StopTime', 10, 'Profile', true, 'Save', false, 'KeepOut', false);
            tc.verifyTrue(r.ok, r.msg);
            tc.verifyTrue(isfile(r.profile_file));
            fprintf('T-INT-03: Normal-mode RTF %.3f (S1 first 10 s)\n', r.rtf);
            tc.assumeFalse(isempty(mex.getCompilerConfigurations('C', 'Selected')), ...
                sprintf('BLOCKED: no C compiler (accelerator); Normal-mode RTF %.3f', r.rtf));
        end

        function testOperatingPointRestart(tc)
            % T-INT-09: save the operating point and resume. The mspsensorlib IMU System object
            % (fusion.simulink.imuSensor in Shaddad_SensorModels/IMU) does not support SimState save:
            % reported to T11, test incomplete when the save fails for that reason
            P = tc.P;
            in = scn_base_input(P, 'op', 'StopTime', 0.2);
            in = in.setModelParameter('SaveFinalState', 'on', 'SaveOperatingPoint', 'on', 'FinalStateName', 'xFinal');
            try
                o = sim(in);
                msg = o.ErrorMessage;
            catch tmp_tf_err
                msg = tmp_tf_err.message;
            end
            tc.assumeFalse(contains(msg, 'imuSensor') || contains(msg, 'SimState'), ...
                ['BLOCKED (T11 IMU model): ' msg]);
            tc.verifyEmpty(msg);
        end

        function testLoggingAndEnergyFromResults(tc)
            % T-INT-06 (signals present in an S1 run, 1 kHz signals decimated to 0.01 s) and T-INT-08
            % (eps_el, eps_mech <= 1 % on S1; S3 BLOCKED by its run time without accelerator)
            r = scn_latest('S1');
            tc.assumeNotEmpty(r, 'no saved S1 result (run_scenario(''S1''))');
            L = scn_logging('table');
            for k = 1:size(L, 1)
                if startsWith(L{k, 1}, {'nav_', 'gnc_'}), continue, end     % Phase-1 stub outputs (constants)
                tc.verifyTrue(isfield(r.S, L{k, 1}), L{k, 1});
            end
            dt = median(diff(r.S.truth.pos_W.t));
            tc.verifyEqual(dt, 0.01, 'AbsTol', 1e-9);
            tc.verifyLessThanOrEqual(r.S.kpi.eps_el.v(end), 0.01);
            tc.verifyLessThanOrEqual(r.S.kpi.eps_mech.v(end), 0.01);
        end

        function testConventionsFromResults(tc)
            % T-INT-04 (sensor part): IMU at rest z = +9.79 m/s^2 (physical specific force on Bus_ImuRaw,
            % D12) from S0; GNSS course with psi = 0 = 90 +/- 1 deg from S1; pitch on the ramp from S2
            r0 = scn_latest('S0');
            tc.assumeNotEmpty(r0, 'no saved S0 result');
            % bus convention (exact): truth specific force at rest, level = [0 0 +g] (D12, physical)
            sf = double(r0.S.truth.specForce_B.v);
            tc.verifyEqual(mean(sf(end - 99:end, 3)), tc.P.frames.g, 'AbsTol', 0.05);
            % sensor output: Bus_ImuRaw.Acceleration z positive (physical convention); the value carries the
            % modelled bias and temperature drift (ICM-20948 datasheet coefficients, D49; IMU at about
            % 50 degC in S0), so the convention check uses a +/-0.5 m/s^2 envelope
            az = double(r0.S.imu_raw.Acceleration.v(:, 3));
            fprintf('T-INT-04: Bus_ImuRaw acc z at rest %.4f m/s^2 (truth %.4f)\n', mean(az(end - 99:end)), ...
                mean(sf(end - 99:end, 3)));
            tc.verifyEqual(mean(az(end - 99:end)), tc.P.frames.g, 'AbsTol', 0.5);
            % GNSS moving-base heading with psi = 0 (facing East): compass 90 +/- 1 deg
            g = r0.S.gnss_raw;
            ok = double(g.headingValid.v(:)) ~= 0 & g.headingValid.t > 2;
            tc.assumeTrue(any(ok), 'no valid GNSS heading in S0');
            hd = double(g.heading_deg.v(ok, 1));
            tc.verifyEqual(median(mod(hd, 360)), 90, 'AbsTol', 1);
            r2 = scn_latest('S2');
            tc.assumeNotEmpty(r2, 'no saved S2 result');
            st = r2.eval(strcmp({r2.eval.id}, 'S2.pitch')).status;
            tc.verifyEqual(st, 'PASS');
        end
    end
end

function s = local_last_state(ts)
% last [x y theta] sample of a logged 3-element state, whatever the storage orientation
d = squeeze(ts.Data);
if size(d, 2) ~= 3 && size(d, 1) == 3, d = d.'; end
s = double(d(end, :));
end
