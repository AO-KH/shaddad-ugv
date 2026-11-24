classdef (TestTags = {'Long'}) t_row_test_system < matlab.unittest.TestCase
    %T_ROW_TEST_SYSTEM  RC row test in the system model: the RowDriver operator choice (VC_OPSRC 4) and scenario R1
    %   (feature 2026-10-27 RC row test, Task 8; tag 'Long').
    %
    %   Spec: docs/superpowers/specs/2026-10-27-rc-row-test-design.md, sections 7 and 8. Each test runs Shaddad_System
    %   once through run_scenario('R1', 'Args', ...) with short inputs (2 rows, 1 minute of driving after arming, 30 degC,
    %   sprayer): full detail (testShortRun) and the fast preset (testFastPreset), each about 5 to 9 min in Normal mode
    %   with the compile. The logs go to work/results/R1_<time>.mat, as for every run_scenario run. Checked on each run:
    %   the MCU reached MANUAL (the driver armed it through the RC chain), at least one row completed on the true path
    %   (row_test_count_rows), the run ended before its stop time through the driver's ended -> Stop Simulation after the
    %   full driving time in MANUAL, the cells started at the ambient temperature (BMS T_cell_max_K within 1 K of
    %   303.15 K), and the implement channel CH8 was in its ON position only while the robot was on a row. Task 11:
    %   testTankEmpty runs the fast preset with a part-filled tank (scenario option 'TankFill' 0.05 = 2 L, about 25 s
    %   of spraying) and checks that the driver switches the pump off when the tank is empty: CH8 ON from the start of
    %   row 1 until the tank (from truth.mass) is down to tank_empty_L, then OFF while the robot drives on along row 1,
    %   the pump PWM 0 from then on, at least one row, and the run ended "time up" (row_test_summary).
    %   Run: runtests('tests/rc/t_row_test_system.m')     one run: add 'ProcedureName', 'testShortRun'

    properties (Constant)
        Inputs = {'Rows', 2, 'Minutes', 1, 'Ambient', 30, 'Sprayer', true};   % the short run of every test
        T_amb_K = 303.15                                                      % 30 degC
        Ext = 0.5    % m, CH8 may stay ON this far beyond a row end: the driver switches on what it sees 0.3 s late
                     % (t_react), and the RC link and the MCU decode add tens of ms (0.5 m covers that at 1 m/s)
    end

    methods (Test)
        function testShortRun(tc)
            res = tc.runR1(false);
            tc.verifyRun(res);
            vc = struct('VC_PHASE', 1, 'VC_IMPL', 3, 'VC_VIZ', 0, 'VC_OPSRC', 4);
            for f = fieldnames(vc)'
                tc.verifyEqual(t_row_test_system.vcValue(res, f{1}), vc.(f{1}), f{1});
            end
        end

        function testFastPreset(tc)
            res = tc.runR1(true);
            tc.verifyRun(res);
            E = getfield(param_sim(), 'sim', 'endurance');
            tc.verifyEqual(t_row_test_system.vcValue(res, 'VC_PACK'), 2, 'fast: lumped pack');
            for f = {'VC_PACK', 'VC_DRIVE_FID', 'VC_STRUCT_FID'}
                tc.verifyEqual(t_row_test_system.vcValue(res, f{1}), E.(f{1}), ['fast: P.sim.endurance.' f{1}]);
            end
        end

        function testTankEmpty(tc)
            % 2 L in the tank (TankFill 0.05): the driver switches the pump off when the tank is empty, in row 1
            res = tc.runR1(true, 'TankFill', 0.05);
            tc.verifyRun(res);
            S = res.S;
            P = t_row_test_system.vcValue(res, 'P');
            R = P.scen.rowtest;                  % the settings of the run (tank_empty_L)
            rows = row_test_rows(R);
            sp = P.structure.impl.sprayer;
            tc.verifyEqual(1e3*sp.fill0*sp.V_max, 2, 'AbsTol', 1e-9, 'TankFill 0.05: 2 L of 40 L at the start');
            % the tank volume from the true mass: the liquid is all the robot loses (1e3/rho L per kg)
            m = S.truth.mass;
            V = 1e3*sp.fill0*sp.V_max - 1e3*(double(m.v(1)) - double(m.v(:)))/sp.rho_liq;
            i_mt = find(V <= R.tank_empty_L, 1);
            tc.assertNotEmpty(i_mt, sprintf('the tank never got down to tank_empty_L (%.3f L left)', V(end)));
            t_mt = m.t(i_mt);
            % CH8 (valid RC frames): ON from the start of row 1 until the tank is empty, OFF for the rest of the run
            ch = S.telem.rc_ch_norm;
            ok = S.telem.rc_link_ok.v(:) ~= 0;
            on = double(ch.v(:, 8)) > -0.5 & ok;
            k_on = find(on, 1);
            tc.assertNotEmpty(k_on, 'CH8 never ON');
            k_off = k_on - 1 + find(~on(k_on:end) & ok(k_on:end), 1);
            tc.assertNotEmpty(k_off, 'CH8 never OFF again after the row start');
            t_on = ch.t(k_on); t_off = ch.t(k_off);
            [tp, ip] = unique(S.truth.pose2D.t);
            xy = @(tq) interp1(tp, S.truth.pose2D.v(ip, 1:2), tq);
            p_on = xy(t_on); p_off = xy(t_off);
            tc.verifyTrue(p_on(1) > -tc.Ext && p_on(1) < 1 && abs(p_on(2)) <= R.row_tol, ...
                sprintf('CH8 ON at %.2f s at the start of row 1: robot at (%.2f, %.2f) m', t_on, p_on));
            tc.verifyEqual(t_off, t_mt, 'AbsTol', 0.2, sprintf(['CH8 OFF when the tank is down to %.2f L ' ...
                '(the driver sees it one 10 ms step late, then the RC link and the MCU decode)'], R.tank_empty_L));
            in_row1 = t_row_test_system.onRow(p_off, rows(1, :), R.row_tol, 0) && p_off(1) < R.L_row - R.d_slow;
            tc.verifyTrue(in_row1, sprintf('CH8 OFF at %.2f s while the robot is still in row 1: (%.2f, %.2f) m', ...
                t_off, p_off));
            tc.verifyFalse(any(on(k_off:end)), 'CH8 ON again after the tank was empty');
            % the pump: PWM duty (MCU, Task_50) 0 from then on; it ran before
            pw = S.mcu_out10.pwm_duty;
            pump = double(pw.v(:, 1));
            tc.verifyTrue(any(pump(pw.t < t_off) > 0), 'the pump never ran');
            late = pw.t > t_off + 0.25;          % the MCU's Task_50 (50 ms) and its rate transitions (0.10 s measured)
            tc.verifyTrue(any(late) && all(pump(late) == 0), sprintf('pump PWM not 0 after CH8 OFF at %.2f s', t_off));
            % the liquid left: the pump stopped before the tank ran dry, and no liquid was used after it
            after = m.t > t_off + 1;
            tc.verifyTrue(V(end) > 0 && V(end) <= R.tank_empty_L, sprintf('%.3f L left at the end', V(end)));
            tc.verifyLessThan(max(double(m.v(after))) - min(double(m.v(after))), 1e-6, 'truth.mass still falling');
            % the robot drove on to the end of row 1; rows completed; how the run ended
            x_after = S.truth.pose2D.v(S.truth.pose2D.t > t_off, 1);
            tc.verifyGreaterThan(max(x_after), R.L_row - 1, 'the robot did not drive on to the end of row 1');
            run = struct('wall_s', res.wall_s, 'rtf', res.rtf, 't_end', res.t_end, 'file', res.file, 'msg', res.msg);
            [~, M] = row_test_summary(S, run, R, rows);
            tc.verifyGreaterThanOrEqual(M.rows_done, 1, 'no row completed');
            tc.verifyEqual(M.end_reason, 'time up');
            t_p0 = [pw.t(find(pw.t >= t_off & pump == 0, 1)); NaN];
            fprintf(['R1 tank: down to %.2f L at %.2f s at (%.2f, %.2f) m, CH8 ON %.2f..%.2f s, pump PWM 0 from ' ...
                '%.2f s, %.3f L left at the end, rows %d, %s\n'], R.tank_empty_L, t_mt, xy(t_mt), t_on, t_off, ...
                t_p0(1), V(end), M.rows_done, M.end_reason);
        end
    end

    methods
        function res = runR1(tc, fast, varargin)
            % one short R1 run (further scenario arguments in varargin); R1 must exist: without its scenario file
            % run_scenario runs a default 10 s scenario
            tc.assertEqual(exist('scn_R1_row_test', 'file'), 2, ...
                'scripts/scenarios/scn_R1_row_test.m missing (run_scenario would run a default 10 s scenario)');
            ws = warning('off', 'all');          % the models' own warnings (parameter precision loss, counter wrap, ...)
            cw = onCleanup(@() warning(ws));     % stay out of the test output, as in the Phase 1 harness (phase1_run_case)
            res = run_scenario('R1', 'Args', [tc.Inputs, {'Fast', fast}, varargin]);
            clear cw
            tc.assertTrue(logical(res.ok), ['R1 ended with an error: ' char(res.msg)]);
            tc.assertNotEmpty(res.S, 'R1: nothing logged');
        end

        function verifyRun(tc, res)
            S = res.S;
            R = param_rowtest();
            R.n_rows = 2; R.minutes = 1; R.T_amb_C = 30; R.sprayer = 1;      % tc.Inputs
            rows = row_test_rows(R);
            % the driver armed the robot through the RC chain: MCU in MANUAL
            mode = S.telem.mode;
            man = mode.v == double(ShaddadMode.MANUAL);
            tc.assertTrue(any(man), 'telem.mode never reached MANUAL (3)');
            % rows completed, counted on the true path
            pose = S.truth.pose2D;
            n = row_test_count_rows(pose.t, pose.v(:, 1:2), rows, R.row_tol, R.row_frac);
            tc.verifyGreaterThanOrEqual(n, 1, 'no row completed on the true path');
            % the run ended through the driver (ended -> Stop Simulation) before the stop time, after the whole driving
            % time in MANUAL (no abort)
            t_man = mode.t(find(man, 1));
            t_off = mode.t(find(man, 1, 'last'));
            tc.verifyEqual(res.stop_time, R.t_arm + 60*R.minutes + 15, 'AbsTol', 1e-9, 'stop time t_arm + 60 x minutes + 15 s');
            tc.verifyLessThan(res.t_end, res.stop_time, 'the run was not ended by the driver before the stop time');
            tc.verifyGreaterThanOrEqual(t_off - t_man, 60*R.minutes - 0.05, ...
                sprintf('MANUAL only from %.2f to %.2f s: the driver did not drive the whole minute', t_man, t_off));
            % the cells started at the ambient temperature
            T0 = double(S.bms_telem.T_cell_max_K.v(1));
            tc.verifyEqual(T0, tc.T_amb_K, 'AbsTol', 1, 'BMS T_cell_max_K at the start: the ambient temperature');
            % the implement switch CH8 (OFF -1, ON 0) was ON only while the robot was on a row
            ch = S.telem.rc_ch_norm;
            on = double(ch.v(:, 8)) > -0.5 & S.telem.rc_link_ok.v(:) ~= 0;   % valid RC frames only
            tc.verifyTrue(any(on), 'sprayer run: CH8 never ON');
            [tp, ip] = unique(pose.t);
            t_on = ch.t(on);
            xy = interp1(tp, pose.v(ip, 1:2), min(max(t_on, tp(1)), tp(end)));   % true position at those samples
            off_row = ~t_row_test_system.onRow(xy, rows, R.row_tol, tc.Ext);
            msg = 'CH8 never ON off the rows';
            if any(off_row)
                k = find(off_row, 1);
                msg = sprintf('CH8 ON off the rows at %d samples, first at t = %.2f s, robot at (%.2f, %.2f) m', ...
                    nnz(off_row), t_on(k), xy(k, 1), xy(k, 2));
            end
            tc.verifyFalse(any(off_row), msg);
            fprintf(['R1 %s: t_end %.2f s of %.0f s, MANUAL %.2f..%.2f s, rows counted %d, T_cell_max(1) %.2f K, ' ...
                'CH8 ON %.1f s, log %s\n'], t_row_test_system.fidelity(res), res.t_end, res.stop_time, t_man, t_off, n, ...
                T0, nnz(on)*mean(diff(ch.t)), res.file);
        end
    end

    methods (Static)
        function v = vcValue(res, name)
            % value of a variable of the run's SimulationInput (the variant controls)
            vars = res.in.Variables;
            v = vars(strcmp({vars.Name}, name)).Value;
        end

        function s = fidelity(res)
            s = 'full';
            if t_row_test_system.vcValue(res, 'VC_PACK') == 2, s = 'fast'; end
        end

        function on = onRow(xy, rows, tol, ext)
            % true where a position lies within tol of a row's centre line, between its ends extended by ext
            on = false(size(xy, 1), 1);
            for r = 1:size(rows, 1)
                a = rows(r, 1:2);
                d = rows(r, 3:4) - a;
                L = hypot(d(1), d(2));
                u = d/L;
                px = xy(:, 1) - a(1);
                py = xy(:, 2) - a(2);
                s = px*u(1) + py*u(2);                    % along the row from its first end
                on = on | (s >= -ext & s <= L + ext & abs(py*u(1) - px*u(2)) <= tol);
            end
        end
    end
end
