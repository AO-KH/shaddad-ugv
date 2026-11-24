classdef t_row_driver < matlab.unittest.TestCase
    %T_ROW_DRIVER  RC row test: settings, geometry, driver parameters, row counting, driver, run summary and field page (feature 2026-10-27 RC row test).
    %
    %   Spec: docs/superpowers/specs/2026-10-27-rc-row-test-design.md. Task 1 covers the settings struct
    %   (param_rowtest) and the row geometry (row_test_rows); Task 2 the driver parameter struct (row_test_prm)
    %   and the row counter (row_test_count_rows); Task 3 the driver (row_driver_init, row_driver_step) on its
    %   kinematic bench (tests/rc/row_driver_bench.m, through the helper tc.bench); Task 4 the run summary
    %   (row_test_summary) on a synthetic run log (the helper tc.synthRun); Task 5 the field page
    %   (row_test_field_page) on the same log; Task 6 the inputs check (row_test_inputs), the start window
    %   (row_test_window, built hidden, its buttons pressed through their callbacks: no uiwait), the launcher
    %   (phase1_row_test, dry runs only: no MATLAB is started) and the worker's error path (phase1_row_run); those
    %   tests set work/rowtest.lock and work/rowtest_last.mat aside and remove the run folders they create (the
    %   helper tc.keepRunFiles); Task 7 the field world (param_world(7), row_field, against the contract of world 1
    %   and world_check) and the row-test settings and driver parameters in P (P.scen.rowtest, P.scen.rowdrv; P
    %   numeric only, D58); Task 9 plot_run's 'Folder' and 'Fig' options (on the newest D1 log in work/results, saved
    %   into a temporary folder; skipped when there is no D1 log) and the field map edge of its path tile (on the
    %   synthetic log, the helpers tc.plotRunLog and tc.fieldEdge); the final fixes plot_run's 'Note' option (the
    %   fidelity in the title of every page, on the synthetic log); Task 11 the sprayer tank: the driver switches the
    %   pump off once the tank is down to tank_empty_L (on the bench with a tank, tc.bench's tank0, and on direct
    %   row_driver_step calls); later tasks add their tests to methods (Test).
    %   Run: runtests('tests/rc/t_row_driver.m')

    properties   %#ok<*PROP> the tests also use local variables P, R, prm (tc.R / tc.prm are the plan's names)
        P      % shaddad_params('Assign', false), loaded once for the class
        R      % row-test settings of the last tc.bench call (param_rowtest defaults before the first call)
        prm    % driver parameters of the last tc.bench call (row_test_prm(P, R))
    end

    methods (TestClassSetup)
        function loadParams(tc)
            tc.P = shaddad_params('Assign', false);
            tc.R = param_rowtest();
            tc.prm = row_test_prm(tc.P, tc.R);
        end
    end

    methods (Test)
        function testDefaultsNumeric(tc)
            R = param_rowtest(); f = fieldnames(R);
            tc.verifyTrue(all(cellfun(@(k) isnumeric(R.(k)), f)), 'D58: numeric only');
            tc.verifyTrue(all(cellfun(@(k) isa(R.(k), 'double'), f)), 'all fields double');
        end

        function testRowsGeometry(tc)
            R = param_rowtest(); R.n_rows = 4;
            rows = row_test_rows(R);
            tc.verifyEqual(rows, [0 0 50 0 1; 0 3 50 3 2; 0 6 50 6 3; 0 9 50 9 4]);
        end

        function testStickTables(tc)
            P = shaddad_params('Assign', false); prm = row_test_prm(P, param_rowtest());
            tc.verifyTrue(all(diff(prm.thr_x) > 0) && all(diff(prm.steer_x) ~= 0));
            tc.verifyEqual(interp1(prm.thr_x, prm.thr_v, scn_rc_stick(P, 1.0)), 1.0, 'RelTol', 0.01);
            tc.verifyEqual(prm.n_delay, 30); tc.verifyEqual(prm.t_drive, 600);
        end

        function testSteerTableLookups(tc)    % the steering table works both ways (steer_x runs against steer_w: RC sign)
            P = shaddad_params('Assign', false); prm = row_test_prm(P, param_rowtest());
            tc.verifyTrue(all(diff(prm.thr_v) > 0) && all(diff(prm.steer_w) > 0), 'tables are sorted by the target');
            tc.verifyTrue(all(diff(prm.steer_x) > 0) || all(diff(prm.steer_x) < 0), 'strictly monotonic stick');
            for w = [-0.6 -0.2 0.2 0.6]
                x = scn_rc_stick(P, w, 'steer');
                tc.verifyEqual(interp1(prm.steer_x, prm.steer_w, x), w, 'AbsTol', 0.005);   % stick -> yaw rate
                tc.verifyEqual(interp1(prm.steer_w, prm.steer_x, w), x, 'AbsTol', 0.005);   % yaw rate -> stick
            end
        end

        function testPrmFields(tc)            % fixed field set and sizes (codegen), values from P and R
            P = shaddad_params('Assign', false); R = param_rowtest();
            prm = row_test_prm(P, R); f = fieldnames(prm);
            tc.verifyTrue(all(cellfun(@(k) isa(prm.(k), 'double'), f)), 'doubles only');
            drv = {'n_rows', 'sprayer', 'L_row', 'spacing', 'headland', 'v_row', 'v_turn', 'd_slow', 'L_look', ...
                't_react', 't_stick', 'wobble', 'f_wobble', 't_tx', 't_arm', 't_arm_wait', 't_abort', 't_end_hold', ...
                't_disarm', 'tank_empty_L'};
            for k = 1:numel(drv)
                tc.verifyEqual(prm.(drv{k}), R.(drv{k}), drv{k});
            end
            tc.verifyEqual(prm.Ts, P.sim.Ts_scen);
            tc.verifyEqual(prm.dx_max, P.sim.Ts_scen/R.t_stick, 'AbsTol', 1e-12);
            tc.verifyEqual(prm.start, [-R.headland/2 0 0]);
            tc.verifyEqual(prm.thr_v([1 end]), [0 P.electronics.lim.v_max_man]);
            tc.verifyEqual(prm.steer_w([1 end]), P.electronics.lim.w_max_man*[-1 1]);
            tc.verifyEqual([numel(prm.thr_v) numel(prm.thr_x) numel(prm.steer_w) numel(prm.steer_x)], [21 21 41 41]);
            % another input (logical sprayer, other rows, minutes and reaction time): the same fields, the same sizes
            R2 = R; R2.n_rows = 3; R2.minutes = 2; R2.sprayer = true; R2.fast = true; R2.T_amb_C = 20; R2.t_react = 0.29;
            prm2 = row_test_prm(P, R2);
            tc.verifyEqual(fieldnames(prm2), f);
            tc.verifyEqual(structfun(@size, prm2, 'UniformOutput', false), structfun(@size, prm, 'UniformOutput', false));
            tc.verifyTrue(isa(prm2.sprayer, 'double') && prm2.sprayer == 1);
            tc.verifyEqual([prm2.n_rows prm2.t_drive prm2.n_delay], [3 120 29]);   % 0.29/0.01 is 28.999999999999996: round, not floor
        end

        function testCountRows(tc)            % a row counts when one contiguous run within tol covers >= frac of its length
            R = param_rowtest(); R.n_rows = 4; rows = row_test_rows(R);     % rows at y = 0 3 6 9 m, 50 m long
            dt = 0.1; x = (0:500)'*dt; n1 = numel(x); z = ones(n1, 1);      % 1 m/s, a sample every 0.1 s
            t = (0:n1 - 1)'*dt;
            % rows 1 (east) and 2 (west), full length, one after the other: 2 rows, each ending at its last sample
            [n, t_done] = row_test_count_rows([t; t + n1*dt], [x, 0*z; flipud(x), 3*z], rows, 0.5, 0.95);
            tc.verifyEqual(n, 2);
            tc.verifyEqual(t_done, [t(end); t(end) + n1*dt], 'AbsTol', 1e-9);
            % 0.6 m off row 3 (outside tol) over the full length, and 90 % of row 4: nothing counts
            tc.verifyEqual(row_test_count_rows(t, [x, 6.6*z], rows, 0.5, 0.95), 0);
            tc.verifyEqual(row_test_count_rows(t(1:451), [x(1:451), 9*z(1:451)], rows, 0.5, 0.95), 0);
            % just inside the limits: 0.4 m off row 3, and 96 % of row 4
            tc.verifyEqual(row_test_count_rows(t, [x, 6.4*z], rows, 0.5, 0.95), 1);
            tc.verifyEqual(row_test_count_rows(t(1:481), [x(1:481), 9*z(1:481)], rows, 0.5, 0.95), 1);
            % 5 m of headland before and after row 1: the pass ends where the robot leaves the row (end + tol)
            xh = (-50:550)'*dt;
            [n, t_done] = row_test_count_rows((0:numel(xh) - 1)'*dt, [xh, 0*xh], rows, 0.5, 0.95);
            tc.verifyEqual([n, t_done], [1, 55.5], 'AbsTol', 0.2);
        end

        function testCountRowsPasses(tc)      % every pass counts once, t_done sorted; a detour splits the run
            R = param_rowtest(); R.n_rows = 4; rows = row_test_rows(R);
            dt = 0.1; x = (0:500)'*dt; n1 = numel(x); z = ones(n1, 1); t = (0:4*n1 - 1)'*dt;
            % rows 1, 2, 1, 2 (east, west, east, west): four passes
            [n, t_done] = row_test_count_rows(t, [x, 0*z; flipud(x), 3*z; x, 0*z; flipud(x), 3*z], rows, 0.5, 0.95);
            tc.verifyEqual(n, 4);
            tc.verifyEqual(t_done, t(n1*(1:4)), 'AbsTol', 1e-9);
            % one pass of row 1 with a 1 m detour over 10 m in the middle: two runs of 40 %, none counts
            y = 0*z; y(201:300) = 1;
            tc.verifyEqual(row_test_count_rows(t(1:n1), [x, y], rows, 0.5, 0.95), 0);
            % a wobble of 0.3 m stays inside tol: counts
            tc.verifyEqual(row_test_count_rows(t(1:n1), [x, 0.3*sin(x)], rows, 0.5, 0.95), 1);
        end

        function testArmsThenDrives(tc)   % tx on, arm at 2 s, no stick before MANUAL
            B = tc.bench(4, 10, 0.5); i = B.t < 2;
            tc.verifyTrue(all(B.tx(B.t >= 0.5)) && all(B.arm(B.t >= 2.01)) && all(B.thr(i) == 0));
        end

        function testStaysOnRows(tc)      % |y - y_row| <= 0.3 m on straights, 2 passes over 4 rows
            B = tc.bench(4, 620, 10); on = B.phase == 2 & B.pose(:,1) > 5 & B.pose(:,1) < 45;
            tc.verifyLessThanOrEqual(max(abs(B.pose(on,2) - 3*round(B.pose(on,2)/3))), 0.3);
            tc.verifyGreaterThanOrEqual(row_test_count_rows(B.t, B.pose(:,1:2), row_test_rows(tc.R), 0.5, 0.95), 8);
        end

        function testOddRowsSecondPass(tc)   % 3 rows: the second pass drives row 1 west from x = 50; both passes count
            B = tc.bench(3, 600, 5); k = find(B.phase == 4, 1, 'last');
            tc.verifyGreaterThan(B.pose(k + find(B.phase(k+1:end) == 2, 1), 1), 40);
            rows = row_test_rows(tc.R);
            [n, t_done] = row_test_count_rows(B.t, B.pose(:,1:2), rows(1, :), 0.5, 0.95);
            tc.verifyEqual(n, 2, 'row 1 counts on the first and on the second pass');
            x = interp1(B.t, B.pose(:,1), t_done);    % where each pass ended: past the east end, then the west end
            tc.verifyTrue(numel(x) == 2 && x(1) > 45 && x(2) < 5, ...
                sprintf('row 1 east, then west: the passes ended at x = %s m', mat2str(x', 3)));
        end

        function testSprayerSwitch(tc)    % pump only along rows
            B = tc.bench(4, 300, 3, true);
            tc.verifyTrue(all(B.impl(B.phase == 2) == 2) && all(B.impl(B.phase ~= 2) == 1));
        end

        function testTimeEnd(tc)          % neutral, disarm, ended within ~5 s after the time
            B = tc.bench(4, 140, 2); te = B.ended_t;
            tc.verifyGreaterThan(te, 2 + 120); tc.verifyLessThan(te, 2 + 120 + 8);
        end

        function testAbortOnFault(tc)     % MCU FAULT at 30 s -> neutral, ended 3.5 s later, no re-arm
            B = tc.bench(4, 300, 1, false, 30);
            tc.verifyTrue(all(B.thr(B.t > 31) == 0) && all(B.arm(B.t > 31) == B.arm(end)));
            tc.verifyEqual(B.ended_t, 33.5, 'AbsTol', 0.2);
        end

        function testDeterministic(tc)
            a = tc.bench(4, 60, 1); b = tc.bench(4, 60, 1); tc.verifyEqual(a.pose, b.pose);
        end

        function testStickRate(tc)
            B = tc.bench(4, 120, 2); tc.verifyLessThanOrEqual(max(abs(diff([B.thr B.steer]))), tc.prm.dx_max + 1e-12);
        end

        function testOperatorCmdFields(tc)    % op = Bus_OperatorCmd (P.scen.dash field order), safe values at the start
            s = row_driver_init(tc.prm); f0 = fieldnames(s); z0 = struct2cell(structfun(@size, s, 'UniformOutput', false));
            [op, s, ended] = row_driver_step(s, tc.prm.start, 0, 2, 0, tc.prm);
            tc.verifyEqual(fieldnames(op), fieldnames(tc.P.scen.dash), 'field order of Bus_OperatorCmd');
            tc.verifyTrue(all(structfun(@(x) isa(x, 'double') && isscalar(x), op)) && isa(ended, 'double'));
            safe = tc.P.scen.dash; safe.tx_power_on = 0;   % t = 0: transmitter still off, disarmed, sticks centred
            tc.verifyEqual(op, safe);
            tc.verifyEqual(ended, 0);
            tc.verifyEqual(fieldnames(s), f0, 'fixed state fields (codegen)');
            tc.verifyEqual(struct2cell(structfun(@size, s, 'UniformOutput', false)), z0, 'fixed state sizes (codegen)');
        end

        function testArmTimeout(tc)       % the MCU never reaches MANUAL: abort t_arm_wait after arming, ended 3 s later
            prm = tc.prm; s = row_driver_init(prm); n = round(20/prm.Ts);
            ph = zeros(n, 1); thr = ph; en = ph;
            for k = 1:n
                [op, s, en(k)] = row_driver_step(s, prm.start, 0, 2, 0, prm);
                ph(k) = s.phase; thr(k) = op.stick_thr;
            end
            t = (0:n - 1)'*prm.Ts;
            tc.verifyEqual(t(find(ph == 8, 1)), prm.t_arm + prm.t_arm_wait, 'AbsTol', 0.02);
            tc.verifyEqual(t(find(en == 1, 1)), prm.t_arm + prm.t_arm_wait + 3, 'AbsTol', 0.02);
            tc.verifyFalse(any(ismember(ph, [2 3 4])), 'never drives');
            tc.verifyTrue(all(thr == 0));
        end

        function testReactionDelay(tc)    % pose seen n_delay samples late: the U-turn starts 0.3 s after the row end
            B = tc.bench(4, 70, 1);
            i1 = find(B.phase == 2 & B.pose(:,1) > 50, 1); i2 = find(B.phase == 3, 1);
            tc.verifyEqual(B.t(i2) - B.t(i1), tc.prm.n_delay*tc.prm.Ts, 'AbsTol', 1.5*tc.prm.Ts);
        end

        function testEndSequence(tc)      % STOP t_drive after MANUAL, DISARM after standing t_end_hold, ENDED t_disarm later
            B = tc.bench(4, 140, 2); t = B.t; p = tc.prm; tol = 1.5*p.Ts;
            i_drv = find(ismember(B.phase, [2 3 4]), 1);   % first drive: the step MANUAL was seen
            i_stop = find(B.phase == 5, 1); i_dis = find(B.phase == 6, 1); i_end = find(B.phase == 7, 1);
            i_v = i_stop - 1 + find(B.v(i_stop:end) < 0.05, 1);   % the robot is below 0.05 m/s from here on
            tc.verifyEqual(t(i_stop) - t(i_drv), p.t_drive, 'AbsTol', tol, 'STOP t_drive after MANUAL');
            tc.verifyEqual(t(i_dis) - t(i_v), p.n_delay*p.Ts + p.t_end_hold, 'AbsTol', tol, 'DISARM after standing t_end_hold');
            tc.verifyEqual(t(i_end) - t(i_dis), p.t_disarm, 'AbsTol', tol, 'ENDED t_disarm after DISARM');
            tc.verifyTrue(all(B.arm(i_drv:i_dis - 1) == 1) && all(B.arm(i_dis:end) == 0), 'armed until DISARM');
            k = t > t(i_stop) + p.t_stick;                      % a full stick travel after STOP
            tc.verifyTrue(all(B.thr(k) == 0) && all(B.steer(k) == 0), 'sticks neutral');
        end

        function testUTurns(tc)           % one left and one right U-turn: at v_turn, radius spacing/2 around the row end
            B = tc.bench(4, 145, 2.25); R = tc.R; tu = B.phase == 3;
            i0 = find(diff([0; tu]) == 1); i1 = find(diff([tu; 0]) == -1);
            tc.verifyEqual(numel(i0), 2, 'two U-turns');
            tc.verifyLessThanOrEqual(max(B.v(tu)), tc.prm.v_turn + 0.02, 'slowed to the turn speed');
            for j = 1:numel(i0)
                k = i0(j):i1(j);
                xc = R.L_row*(B.pose(i0(j), 1) > R.L_row/2);                       % row end
                yc = R.spacing*round(B.pose(i0(j), 2)/R.spacing) + R.spacing/2;    % half way to the next row
                r = hypot(B.pose(k, 1) - xc, B.pose(k, 2) - yc);
                tc.verifyLessThanOrEqual(max(abs(r - R.spacing/2)), 0.2, sprintf('radius of U-turn %d', j));
            end
        end

        function testWobble(tc)           % held on a row line: throttle = stick for v_row, steering = the unsteadiness alone
            p = tc.prm; s = row_driver_init(p); n = round(30/p.Ts); t = (0:n - 1)'*p.Ts;
            st = zeros(n, 1); th = st; ph = st;
            for k = 1:n                   % robot at x = 10 m on row 1's line, 1 m/s; MCU MANUAL from t_arm
                [op, s] = row_driver_step(s, [10 0 0], 1, 2 + (t(k) >= p.t_arm), 0, p);
                st(k) = op.stick_steer; th(k) = op.stick_thr; ph(k) = s.phase;
            end
            i = t >= 5;
            wb = p.wobble*(sin(2*pi*p.f_wobble(1)*t) + sin(2*pi*p.f_wobble(2)*t + 1))/2;
            tc.verifyEqual(ph(i), 2*ones(nnz(i), 1), 'ROW');
            tc.verifyEqual(st(i), wb(i), 'AbsTol', 1e-9);
            tc.verifyEqual(th(i), interp1(p.thr_v, p.thr_x, p.v_row)*ones(nnz(i), 1), 'AbsTol', 1e-12);
        end

        function testTankEmptyPumpOff(tc)     % 2 L in the tank: CH8 ON in row 1 until tank_empty_L, then OFF for good
            B = tc.bench(4, 200, 3, true, Inf, 2); p = tc.prm; R = tc.R;   % 2 L: 23.75 s down to 0.1 L at 4.8 L/min
            F = tc.bench(4, 200, 3, true);                                  % the same run, a tank that never empties
            i_on = find(B.impl == 2, 1);                                    % the start of row 1
            i_mt = find(B.tank <= p.tank_empty_L, 1);                       % the first step the driver sees it empty
            tc.assertTrue(~isempty(i_on) && ~isempty(i_mt) && i_mt > i_on, 'CH8 ON, then the tank empty');
            tc.verifyEqual(i_on, find(F.impl == 2, 1), 'ON where row 1 starts, as with a full tank');
            tc.verifyTrue(B.phase(i_on) == 2 && abs(B.pose(i_on, 1)) < 1 && abs(B.pose(i_on, 2)) < R.row_tol, ...
                sprintf('ON at the start of row 1: (%.2f, %.2f) m', B.pose(i_on, 1:2)));
            tc.verifyTrue(all(B.impl(i_on:i_mt - 1) == 2), 'ON from the start of row 1 until the tank is empty');
            tc.verifyTrue(all(B.impl(i_mt:end) == 1), 'OFF from then on, for the rest of the run');
            tc.verifyEqual(B.t(i_mt) - B.t(i_on), (2 - p.tank_empty_L)/(4.8/60), 'AbsTol', 1.5*p.Ts, ...
                'ON for (tank0 - tank_empty_L)/drain');
            xy = B.pose(i_mt, 1:2);
            tc.verifyTrue(B.phase(i_mt) == 2 && xy(1) > 5 && xy(1) < R.L_row - 5 && abs(xy(2)) < R.row_tol, ...
                sprintf('the tank runs empty along row 1: (%.2f, %.2f) m', xy));
            tc.verifyTrue(any(F.impl(i_mt + 1:end) == 2), 'with a full tank CH8 stays ON there');
            tc.verifyEqual(B.tank(end), B.tank(i_mt), 'no liquid used once CH8 is OFF');
            % the robot keeps driving and the rows are still counted: the run of the full tank, but for CH8
            tc.verifyEqual(rmfield(B, {'impl', 'tank'}), rmfield(F, {'impl', 'tank'}), 'the same driving');
            n = row_test_count_rows(B.t, B.pose(:, 1:2), row_test_rows(R), R.row_tol, R.row_frac);
            tc.verifyGreaterThanOrEqual(n, 2, 'rows completed after the tank ran empty');
        end

        function testTankStepRule(tc)         % direct calls: ON in ROW with the sprayer while tank_L > tank_empty_L
            R = param_rowtest(); R.sprayer = 1; p = row_test_prm(tc.P, R); e = p.tank_empty_L;
            n = round(10/p.Ts); t = (0:n - 1)'*p.Ts;
            % tank_L: 0 L up to 1 s (the Unit Delay's initial value at t = 0), then falling as a tank does: full, just
            % above tank_empty_L, at it, half of it, empty
            tank = zeros(n, 1); tank(t >= 1) = 40;
            tank(t >= 5) = e + 1e-9; tank(t >= 6) = e; tank(t >= 7) = e/2; tank(t >= 8) = 0;
            s = row_driver_init(p); s40 = s; impl = zeros(n, 1); ph = impl; same = true(n, 1);
            for k = 1:n                       % robot at x = 10 m on row 1's line, 1 m/s; MCU MANUAL from t_arm
                m = 2 + (t(k) >= p.t_arm);
                [op, s, en] = row_driver_step(s, [10 0 0], 1, m, tank(k), p);
                [op40, s40, en40] = row_driver_step(s40, [10 0 0], 1, m, 40, p);   % the same driver, a full tank
                impl(k) = op.sw_impl; ph(k) = s.phase;
                op.sw_impl = op40.sw_impl;
                same(k) = isequal(op, op40) && isequal(s, s40) && en == en40;
            end
            row = ph == 2;
            tc.assertTrue(any(row & t < 5), 'ROW reached while the tank is full');
            tc.verifyEqual(impl, 1 + double(row & tank > e), 'ON (2) exactly in ROW with more than tank_empty_L');
            tc.verifyTrue(all(impl(row & t < 6) == 2), ...
                '0 L before the row does not keep it OFF; ON just above tank_empty_L');
            tc.verifyTrue(all(impl(t >= 6) == 1), 'OFF at tank_empty_L and below');
            tc.verifyTrue(all(same), 'the sticks, the other switches and the driver state do not depend on tank_L');
        end

        function testTankSprayerOff(tc)       % no sprayer: CH8 OFF whatever tank_L; the bench tank is never drained
            R = param_rowtest(); R.sprayer = 0; p = row_test_prm(tc.P, R);
            n = round(6/p.Ts); t = (0:n - 1)'*p.Ts;
            for lv = [0 p.tank_empty_L 2 40 Inf]
                s = row_driver_init(p); impl = zeros(n, 1);
                for k = 1:n                   % robot on row 1's line, MCU MANUAL from t_arm: ROW from about 2 s on
                    [op, s] = row_driver_step(s, [10 0 0], 1, 2 + (t(k) >= p.t_arm), lv, p);
                    impl(k) = op.sw_impl;
                end
                tc.verifyTrue(s.phase == 2 && all(impl == 1), ...
                    sprintf('tank_L %g L: CH8 OFF all along (ROW at the end)', lv));
            end
            B = tc.bench(4, 150, 2, false, Inf, 2); F = tc.bench(4, 150, 2, false);
            tc.verifyTrue(all(B.impl == 1) && all(B.tank == 2), 'bench, 2 L in the tank: CH8 OFF, none used');
            tc.verifyEqual(rmfield(B, 'tank'), rmfield(F, 'tank'), ...
                'bench: the same run as with a tank that never empties');
        end

        function testTankFullAsBefore(tc)     % a full 40 L tank outlasts 3 minutes: the bench run without a tank
            F = tc.bench(4, 200, 3, true);                 % the default tank (Inf): what every other bench test sees
            B = tc.bench(4, 200, 3, true, Inf, 40);        % full: 3 min of driving, at most 14.4 L sprayed
            tc.verifyTrue(all(isinf(F.tank)), 'the default tank never empties');
            tc.verifyEqual(rmfield(B, 'tank'), rmfield(F, 'tank'), 'a full tank: the same run, field for field');
            tc.verifyTrue(all(B.impl(B.phase == 2) == 2) && all(B.impl(B.phase ~= 2) == 1), ...
                'CH8 ON exactly along the rows');
            tc.verifyEqual(40 - B.tank(end), nnz(B.impl == 2)*tc.prm.Ts*4.8/60, 'AbsTol', 1e-9, ...
                'liquid used only while CH8 is ON, 4.8 L/min');
        end

        function testSummaryEndReason(tc)     % how the run ended: time up, SoC floor, MCU left MANUAL, error of the run
            [S, run, cfg, rows] = tc.synthRun();                                    % 7 min asked, MANUAL ends 3 s after the time
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.end_reason, 'time up');
            Sb = S; Sb.telem.mode.v(Sb.telem.mode.t >= 20 & Sb.telem.mode.t < 20.3) = 4;   % 0.3 s in HOLD, then MANUAL again
            [~, M] = row_test_summary(Sb, run, cfg, rows);
            tc.verifyEqual(M.end_reason, 'time up', 'a short blip out of MANUAL does not end the run');
            tc.verifyEqual(M.t_drive, 420, 'AbsTol', 1e-9);
            [S, run, cfg, rows] = tc.synthRun(struct('soc_stop', 30));              % Sim_Monitors stops the run at 30 s: stop_reason 3
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.end_reason, 'battery at the SoC floor');
            S.kpi.stop_reason.v(:) = 0;                                             % the same log without the reason: it just ends early
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.end_reason, 'run ended early at 30.0 s (stop_reason 0)', 'not "time up" before the time is up');
            [S, run, cfg, rows] = tc.synthRun(struct('exit', [40 9], 'bms_fault', [39.9 4]));   % MCU FAULT (9) at 40 s, BMS Fault_Word 4
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyTrue(startsWith(M.end_reason, 'MCU FAULT at 40'), M.end_reason);
            tc.verifyEqual(M.bms_fault_bits, 4);
            run.msg = 'Assertion failed in block Sim_Monitors/NaN_Assertion';       % an error of the run comes first
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.end_reason, 'simulation error: Assertion failed in block Sim_Monitors/NaN_Assertion');
        end

        function testSummaryFields(tc)        % field set and order of M: the field page and the worker read it
            [S, run, cfg, rows] = tc.synthRun();
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(fieldnames(M)', {'rows_done', 'passes', 't_done', 'E_row_Wh', 't_drive', 'dist', 'v_mean', 'E_Wh', ...
                'Wh_per_row', 'Wh_per_row_run', 'Wh_per_km', 'soc_start', 'soc_end', 'I_peak', 'T_cell_max_C', ...
                'T_cell_bms_max_C', 'T_winding_max_C', 'T_esc_max_C', 'T_encl_max_C', 'dy_max', 'mcu_fault_bits', ...
                'mcu_warn_bits', 'bms_fault_bits', 'end_reason', 'fidelity'});
        end

        function testSummaryRows(tc)          % rows counted from the true path: repeats counted, field covered = least count of a row
            [S, run, cfg, rows] = tc.synthRun();                                    % rows 1-3 twice, row 4 once
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual([M.rows_done M.passes size(M.t_done, 1) size(M.E_row_Wh, 1)], [7 1 7 7]);
            tc.verifyTrue(issorted(M.t_done) && size(M.t_done, 2) == 1);
            tc.verifyEqual(M.t_done(1), 55.5, 'AbsTol', 0.5);    % armed at 2 s, 3 m run-in, 50 m row, 0.5 m band: 55.5 s at 1 m/s
            tc.verifyEqual(M.E_row_Wh, diff([2; M.t_done])*0.1, 'AbsTol', 1e-9, ...
                'Wh between completions (the first from MANUAL at 2 s), 360 W = 0.1 Wh/s');
            rows(2, [2 4]) = rows(2, [2 4]) + 1;                                    % row 2 1 m further north: the robot never drives it
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual([M.rows_done M.passes], [5 0], 'a row never completed: the field was not covered once');
            [S, run, cfg, rows] = tc.synthRun(struct('minutes', 8));                % the whole path: 8 rows, the field twice
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual([M.rows_done M.passes], [8 2]);
        end

        function testSummaryFigures(tc)       % the figures of a clean run against what the synthetic log holds
            [S, run, cfg, rows] = tc.synthRun();      % 4 rows, 7 min: 1 m/s, 360 W, SoC 0.95 - 1e-4 t, 430 s log, 0.25 m weaving
            [~, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.t_drive, 420, 'AbsTol', 1e-9, 'MANUAL for 423 s, but the time asked is 420 s');
            tc.verifyEqual([M.dist M.v_mean], [420 1], 'AbsTol', 1e-9);
            tc.verifyEqual([M.E_Wh M.Wh_per_row_run M.Wh_per_km], [43 43/7 43/0.42], 'AbsTol', 1e-9);   % kpi.E_batt, J
            tc.verifyEqual(M.Wh_per_row, mean(M.E_row_Wh), 'AbsTol', 1e-12, ...
                'the mean of the per-row energies (the field page bars)');
            tc.verifyEqual(M.Wh_per_row, (397.5 - 2)*0.1/7, 'AbsTol', 0.01, ...
                '360 W from MANUAL at 2 s to the 7th completion (about 397.5 s), over 7 rows: not the whole run');
            tc.verifyEqual([M.soc_start M.soc_end], [95 90.7], 'AbsTol', 1e-9);                      % per cent
            tc.verifyEqual(M.I_peak, 16, 'AbsTol', 1e-9, 'largest |I| (the regeneration peak), not the precharge inrush of 30 A at 0.5 s');
            tc.verifyEqual([M.T_cell_max_C M.T_cell_bms_max_C M.T_winding_max_C M.T_esc_max_C], ...
                [34.86 36.72 53.6 50.16], 'AbsTol', 1e-3);   % degC: true cells, BMS reading, the hotter side
            tc.verifyEqual(M.T_encl_max_C, 49.3, 'AbsTol', 1e-3, 'not the start-up values of the enclosure sensor (-273 and 120 degC)');
            tc.verifyEqual(M.dy_max, 0.25, 'AbsTol', 0.01, 'along the rows only, not the headland connectors 1.5 m out');
            tc.verifyEqual([M.mcu_fault_bits M.mcu_warn_bits M.bms_fault_bits], [0 0 0], 'start-up bits are not faults of the run');
            tc.verifyEqual(M.fidelity, 'full');
            for k = 1:2                           % each cell line from its own signal: n/a when that one is missing
                Sk = S;
                if k == 1, Sk.power_truth = rmfield(Sk.power_truth, 'T_cell_max_K');
                else, Sk.bms_telem = rmfield(Sk.bms_telem, 'T_cell_max_K'); end
                [txt, M] = row_test_summary(Sk, run, cfg, rows);
                tc.verifyEqual(isnan([M.T_cell_max_C M.T_cell_bms_max_C]), [k == 1, k == 2], ...
                    sprintf('case %d: NaN', k));
                want = {'34.9 degC', '36.7 degC'}; want{k} = 'n/a';
                has = @(label, value) ~isempty(regexp(txt, ['\n  ' label '[ ]+' regexptranslate('escape', value) ...
                    '\n'], 'once'));
                tc.verifyTrue(has('Cell', want{1}) && has('Cell \(BMS reading\)', want{2}), ...
                    sprintf('case %d: the cell lines', k));
            end
        end

        function testSummaryBits(tc)          % fault and warning bits that were set at any time after the start-up
            [S, run, cfg, rows] = tc.synthRun();
            S.telem.fault_word.v(S.telem.fault_word.t >= 100 & S.telem.fault_word.t < 110) = 1024;   % bit 10
            S.telem.fault_word.v(S.telem.fault_word.t >= 200 & S.telem.fault_word.t < 210) = 1;      % bit 0, a second event
            S.telem.warn_word.v(S.telem.warn_word.t >= 150) = 8;                                     % bit 3
            S.bms_telem.Fault_Word.v(S.bms_telem.Fault_Word.t >= 300) = 4;                           % bit 2
            S.bms_telem.Fault_Word.v(S.bms_telem.Fault_Word.t == 400) = 16;                          % bit 4, one sample
            [txt, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual([M.mcu_fault_bits M.mcu_warn_bits M.bms_fault_bits], [1025 8 20]);
            tc.verifyTrue(contains(txt, '0x00000401 (bits 0, 10)') && contains(txt, '0x00000008 (bit 3)') ...
                && contains(txt, '0x00000014 (bits 2, 4)'), 'the text gives the word and the bit numbers');
        end

        function testSummaryText(tc)          % the text: inputs, figures, how it ended, log and timing
            [S, run, cfg, rows] = tc.synthRun();
            txt = row_test_summary(S, run, cfg, rows);
            tc.verifyClass(txt, 'char'); tc.verifySize(txt, [1 numel(txt)]);
            has = @(label, value) ~isempty(regexp(txt, [label '[ ]+' regexptranslate('escape', value) '(\n|$)'], 'once'));
            tc.verifyTrue(has('Run log', run.file) && has('Ended', 'time up'));
            tc.verifyTrue(has('Rows', '4') && has('Driving time asked', '7 min') && has('Ambient temperature', '30 degC') ...
                && has('Sprayer', 'on'), 'inputs');
            tc.verifyTrue(has('Rows completed', '7 (repeats counted)') && has('Field covered', '1 time') ...
                && has('Driving time', '7 min 00 s') && has('Distance', '420.0 m') && has('Mean speed', '1.00 m/s'), 'rows and driving');
            tc.verifyTrue(has('Energy from the pack', '43.0 Wh') && has('Energy per row', '5.65 Wh') ...
                && has('Energy per row, whole run', '6.14 Wh (total / rows, incl. start-up, returns, last pass)') ...
                && has('Energy per km', '102.4 Wh/km') && has('State of charge', '95.0 % at the start, 90.7 % at the end') ...
                && has('Peak pack current', '16.0 A'), 'battery');
            tc.verifyTrue(has('Cell', '34.9 degC') && has('Cell \(BMS reading\)', '36.7 degC') ...
                && has('Motor winding', '53.6 degC') && has('Motor controller', '50.2 degC') ...
                && has('Enclosure \(MCU sensor\)', '49.3 degC') ...
                && has('Largest distance from a row line', '0.25 m'), 'temperatures and row keeping');
            tc.verifyTrue(has('MCU fault bits', 'none') && has('MCU warning bits', 'none') && has('BMS fault bits', 'none'), 'faults');
            tc.verifyTrue(has('Simulated time', '7 min 10 s') && has('Wall-clock time', '1 h 02 min') && has('Real-time factor', '0.13'), 'timing');
            tc.verifyFalse(contains(txt, 'NaN') || contains(txt, 'n/a'), 'a clean run has every figure');
            tc.verifyTrue(contains(txt, 'full detail'));
            cfg.fast = 1; cfg.sprayer = 0;
            [txt, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyEqual(M.fidelity, 'fast');
            tc.verifyTrue(contains(txt, 'fast (lumped pack') && ~isempty(regexp(txt, 'Sprayer[ ]+off', 'once')), 'fidelity and sprayer input');
        end

        function testSummaryNoArming(tc)      % the MCU never reaches MANUAL: nothing driven, the reason says so
            [S, run, cfg, rows] = tc.synthRun();
            S.telem.mode.v(S.telem.mode.t >= 2) = 2;                                % STANDBY from 1 s to the end
            S.truth.pose2D.v = repmat(S.truth.pose2D.v(1, :), numel(S.truth.pose2D.t), 1);   % the robot stands at its start pose
            [txt, M] = row_test_summary(S, run, cfg, rows);
            tc.verifyTrue(startsWith(M.end_reason, 'MCU never reached MANUAL (STANDBY at 430'), M.end_reason);
            tc.verifyEqual([M.rows_done M.passes M.t_drive], [0 0 0]);
            tc.verifyEmpty(M.t_done); tc.verifyEmpty(M.E_row_Wh);
            tc.verifyTrue(isnan(M.v_mean) && isnan(M.Wh_per_row) && isnan(M.Wh_per_row_run), 'no division by zero');
            tc.verifyTrue(contains(txt, 'MCU never reached MANUAL'));
        end

        function testSummaryEmptyLog(tc)      % the run failed before it logged anything: the error text, no exception
            [~, run, cfg, rows] = tc.synthRun();
            run.msg = sprintf('Error in <a href="matlab:open_and_hilite_hyperlink (''Sys/Blk'',''error'')">Sys/Blk</a>: no\nmodel');
            run.rtf = Inf;                                                          % run_scenario: t_end / exec_s without an exec_s
            for S = {struct(), []}
                [txt, M] = row_test_summary(S{1}, run, cfg, rows);
                tc.verifyEqual(M.end_reason, 'simulation error: Error in Sys/Blk: no model', 'one line, no hyperlink markup');
                tc.verifyEqual([M.rows_done M.passes M.t_drive], [0 0 0]);
                tc.verifyTrue(isnan(M.dist) && isnan(M.E_Wh) && isnan(M.I_peak) && isnan(M.mcu_fault_bits), 'no data is NaN');
                tc.verifyTrue(contains(txt, 'simulation error: Error in Sys/Blk: no model') && contains(txt, 'n/a'));
                tc.verifyNotEmpty(regexp(txt, 'Real-time factor[ ]+n/a', 'once'), 'Inf is no figure');
            end
        end

        function testFieldPage(tc)            % 2 x 2 tiles from the synthetic log: each tile shows what the summary holds
            [S, run, cfg, rows] = tc.synthRun();
            [~, M] = row_test_summary(S, run, cfg, rows);
            f = row_test_field_page(S, M, rows, 'Visible', 'off', 'Name', 'R1_20261028_101500', 'Cfg', cfg);
            tc.addTeardown(@close, f);
            tc.verifyEqual(char(f.Visible), 'off');
            tc.verifyNumElements(findall(f, 'Type', 'axes'), 4);
            ttl = findobj(f, 'Type', 'tiledlayout').Title.String;
            for part = {'R1_20261028_101500', 'full', '4 rows', '7 min', '30 degC', 'sprayer on'}
                tc.verifySubstring(ttl, part{1}, 'title: log name, fidelity and inputs');
            end
            pose = S.truth.pose2D;
            ax = findobj(f, 'Tag', 'path');                                         % (a) the rows and the path coloured by time
            sc = findobj(ax, 'Type', 'scatter');
            tc.verifyEqual([sc.XData(:) sc.YData(:) sc.CData(:)], [pose.v(:, 1:2) pose.t], 'the true path, colour = time');
            rl = findobj(ax, 'Tag', 'row');
            tc.verifyEqual(sortrows([vertcat(rl.XData) vertcat(rl.YData)]), sortrows(rows(:, [1 3 2 4])), 'the four rows');
            st = findobj(findobj(f, 'Tag', 'rows_done'), 'Type', 'stair');          % (b) rows completed against time
            [~, k] = ismember(M.t_done, st.XData);
            tc.verifyEqual(st.YData(k), 1:M.rows_done, 'the count is k at the k-th completion');
            ln = findobj(findobj(f, 'Tag', 'offset'), 'Type', 'line');              % (c) distance from the row line
            tc.verifyEqual(ln.XData(:), pose.t);
            tc.verifyEqual(max(abs(ln.YData)), M.dy_max, 'AbsTol', 1e-12, 'the largest distance of the summary');
            bb = findobj(findobj(f, 'Tag', 'energy'), 'Type', 'bar');               % (d) energy per row
            tc.verifyEqual(bb.YData(:), M.E_row_Wh);
            cfg.fast = 1;                                                           % no log name, no inputs: the title shows what it has
            [~, M] = row_test_summary(S, run, cfg, rows);
            f = row_test_field_page(S, M, rows, 'Visible', 'off');
            tc.addTeardown(@close, f);
            tc.verifySubstring(findobj(f, 'Type', 'tiledlayout').Title.String, 'fast');
        end

        function testFieldPageOffset(tc)      % tile (c): signed distance to the nearest row line, + = left of the row direction
            [S, run, cfg, rows] = tc.synthRun();
            % 0.3 m north of row 1 | 0.2 m south of row 2 | 3 m before the rows start | past their ends | 1.4 m north of row 1
            % (row 2 is 1.6 m away) | 0.1 m south of row 4
            S.truth.pose2D = struct('t', (0:5)'/10, 'v', [10 0.3 0; 20 2.8 0; -3 0 0; 51 3 0; 25 1.4 0; 40 8.9 0]);
            [~, M] = row_test_summary(S, run, cfg, rows);
            f = row_test_field_page(S, M, rows, 'Visible', 'off');
            tc.addTeardown(@close, f);
            ln = findobj(findobj(f, 'Tag', 'offset'), 'Type', 'line');
            tc.verifyEqual(ln.YData(:), [0.3; -0.2; NaN; NaN; 1.4; -0.1], 'AbsTol', 1e-12, 'NaN where the robot is not along a row');
            tc.verifyEqual(M.dy_max, 1.4, 'AbsTol', 1e-12);
            tc.verifyEqual(max(abs(ln.YData)), M.dy_max, 'AbsTol', 1e-12, 'the page and the summary count the same samples');
        end

        function testFieldPageIncomplete(tc)  % no energy figure for the last row, no row completed, nothing logged: a page, no error
            [S, run, cfg, rows] = tc.synthRun();
            [~, M] = row_test_summary(S, run, cfg, rows);
            M.E_row_Wh(end) = NaN;                % the kpi log ended before the last completion (header of row_test_summary)
            f = row_test_field_page(S, M, rows, 'Visible', 'off');
            tc.addTeardown(@close, f);
            ax = findobj(f, 'Tag', 'energy');
            tc.verifyEqual(findobj(ax, 'Type', 'bar').YData(:), M.E_row_Wh);
            tc.verifyNumElements(findobj(ax, 'Type', 'text', 'String', 'n/a'), 1, 'the bar without a figure is marked');
            Sn = S;                               % the MCU never reached MANUAL: the robot stands at its start, no row is done
            Sn.telem.mode.v(Sn.telem.mode.t >= 2) = 2;
            Sn.truth.pose2D.v = repmat(Sn.truth.pose2D.v(1, :), numel(Sn.truth.pose2D.t), 1);
            for Sk = {Sn, struct(), []}
                [~, M] = row_test_summary(Sk{1}, run, cfg, rows);
                f = row_test_field_page(Sk{1}, M, rows, 'Visible', 'off');
                tc.addTeardown(@close, f);
                tc.verifyNumElements(findall(f, 'Type', 'axes'), 4);
                for tag = {'rows_done', 'offset', 'energy'}
                    tc.verifyNotEmpty(findobj(findobj(f, 'Tag', tag{1}), 'Type', 'text'), ['a note in the tile ' tag{1}]);
                end
            end
        end

        function testInputValidation(tc)      % the five inputs against the limits of param_rowtest; msg names the input and its range
            R = param_rowtest();
            [ok, msg, cfg] = row_test_inputs(R, R);
            tc.verifyTrue(ok, 'the defaults are accepted');
            tc.verifyEmpty(msg);
            tc.verifyEqual(cfg, R);
            bad = {'n_rows', 1, 'Rows', '2 to 30'; 'n_rows', 31, 'Rows', '2 to 30'; 'n_rows', 4.5, 'Rows', '2 to 30'; ...
                'minutes', 0, 'Minutes', '1 to 240'; 'minutes', 241, 'Minutes', '1 to 240'; 'minutes', NaN, 'Minutes', '1 to 240'; ...
                'T_amb_C', 60, 'Ambient', '-10 to 55'; 'sprayer', 2, 'Sprayer', 'true or false'; 'fast', [], 'Fast', 'true or false'};
            for k = 1:size(bad, 1)
                c = R; c.(bad{k, 1}) = bad{k, 2};
                [ok, msg, ~, est] = row_test_inputs(c, R);
                lbl = sprintf('%s = %s', bad{k, 1}, mat2str(bad{k, 2}));
                tc.verifyFalse(ok, lbl);
                tc.verifySubstring(msg, bad{k, 3}, [lbl ': the input']);
                tc.verifySubstring(msg, bad{k, 4}, [lbl ': its range']);
                tc.verifyTrue(isnan(est), [lbl ': no estimate']);
            end
            c = R; c.n_rows = 30; c.minutes = 240; c.T_amb_C = -10; c.sprayer = true; c.fast = true;   % the limits; check boxes give logicals
            [ok, ~, cfg] = row_test_inputs(c, R);
            tc.verifyTrue(ok, 'the limits are inside the ranges');
            tc.verifyTrue(isa(cfg.sprayer, 'double') && cfg.sprayer == 1 && isa(cfg.fast, 'double') && cfg.fast == 1, ...
                'sprayer and fast stored as double 0 / 1 (D58)');
            R.rtf = [1 2 4 8];                    % distinct: param_rowtest gives full and full + implement 0.5 both
            for k = 1:4                           % estimate: compile + the minutes at the real-time factor of the case
                c = R; c.minutes = 60; c.sprayer = mod(k - 1, 2); c.fast = k > 2;   % full, full + implement, fast, fast + implement
                [~, ~, ~, est] = row_test_inputs(c, R);
                tc.verifyEqual(est, R.t_compile/60 + 60/R.rtf(k), 'AbsTol', 1e-12, sprintf('estimate, case %d', k));
            end
        end

        function testDryRunJob(tc)            % a dry run: the run folder with inputs.mat, the background command, the estimate; nothing starts
            tc.keepRunFiles();
            [job, txt] = tc.launch('Rows', 4, 'Minutes', 5, 'Ambient', 30, 'Sprayer', true, 'Fast', false, 'DryRun', true);
            root = char(currentProject().RootFolder);
            work = fullfile(root, 'work');
            tc.verifyEqual(fieldnames(job)', {'folder', 'cfg', 'est_min', 'cmd'});
            tc.verifyMatches(job.folder, ['^' regexptranslate('escape', fullfile(work, 'reports', 'rowtest_')) '\d{8}_\d{6}$']);
            tc.verifyTrue(isfile(fullfile(job.folder, 'inputs.mat')), 'the folder with inputs.mat');
            c = param_rowtest(); c.n_rows = 4; c.minutes = 5; c.T_amb_C = 30; c.sprayer = 1; c.fast = 0;
            tc.verifyEqual(job.cfg, c, 'param_rowtest with the five inputs, numeric (D58)');
            tc.verifyEqual(load(fullfile(job.folder, 'inputs.mat')).cfg, c, 'inputs.mat holds cfg');
            tc.verifyEqual(job.est_min, c.t_compile/60 + 5/c.rtf(2), 'AbsTol', 1e-9, ...
                'compile + 5 min at the real-time factor of full detail + implement (param_rowtest)');
            runcmd = fullfile(job.folder, 'run.cmd');      % the start line runs run.cmd: the -batch line, then the done.txt guard
            tc.verifyEqual(job.cmd, sprintf('start "Shaddad row test" /min "%s"', runcmd), 'the start line');
            rc = fileread(runcmd);
            tc.verifyTrue(startsWith(rc, ['@echo off' char([13 10])]), 'a batch file: @echo off first (no byte order mark), CR LF');
            batch = sprintf('"%s" -batch "cd(''%s''); openProject(''Shaddad_UGV.prj''); phase1_row_run(''%s'')"', ...
                fullfile(matlabroot, 'bin', 'matlab.exe'), root, job.folder);
            guard = {['if exist "' fullfile(job.folder, 'done.txt') '" exit %code%'], ...
                'echo error: the background MATLAB ended (exit %code%) before the row test finished', ...
                ['if /i "%lock%"=="folder  ' job.folder '" del "' fullfile(work, 'rowtest.lock') '"'], ...
                ['>"' fullfile(job.folder, 'done.txt') '" echo finished']};
            for s = [{batch} guard]
                tc.verifySubstring(rc, s{1}, 'run.cmd: the -batch line and the done.txt guard');
            end
            tc.verifyLessThan(strfind(rc, batch), strfind(rc, guard{1}), 'the guard runs after MATLAB');
            tc.verifySubstring(txt, job.folder, 'the folder is printed');
            tc.verifyFalse(isfile(fullfile(work, 'rowtest.lock')), 'a dry run starts nothing and takes no lock');
            last = struct('n_rows', 4, 'minutes', 5, 'T_amb_C', 30, 'sprayer', 1, 'fast', 0);
            tc.verifyEqual(load(fullfile(work, 'rowtest_last.mat')).last, last, 'the inputs are remembered');
            job2 = tc.launch('Window', false, 'DryRun', true);   % no inputs, no window: the last inputs
            tc.verifyEqual(job2.cfg, c, 'the missing inputs come from work/rowtest_last.mat');
            tc.verifyNotEqual(job2.folder, job.folder, 'a folder of its own, also within the same second');
        end

        function testFastOptional(tc)         % a scripted run may leave Fast out (off): the window only when rows, minutes, ambient or sprayer is missing
            tc.keepRunFiles();
            lastfile = fullfile(char(currentProject().RootFolder), 'work', 'rowtest_last.mat');
            last = struct('n_rows', 9, 'minutes', 7, 'T_amb_C', 20, 'sprayer', 0, 'fast', 1);   % the last start was fast
            save(lastfile, 'last');
            job = tc.launch('Rows', 8, 'Minutes', 20, 'Ambient', 40, 'Sprayer', true, 'DryRun', true);   % the spec's example
            tc.assertNotEmpty(job, 'no window without Fast');
            tc.verifyEqual([job.cfg.n_rows job.cfg.minutes job.cfg.T_amb_C job.cfg.sprayer job.cfg.fast], [8 20 40 1 0], ...
                'Fast off when not given, whatever the last start used');
            save(lastfile, 'last');
            job = tc.launch('Window', false, 'DryRun', true);
            tc.assertNotEmpty(job);
            tc.verifyEqual([job.cfg.n_rows job.cfg.minutes job.cfg.T_amb_C job.cfg.sprayer job.cfg.fast], [9 7 20 0 0], ...
                'no window: the last inputs, but Fast off');
        end

        function testSecondRunRefused(tc)     % a lock whose folder has no done.txt and is younger than 3 x est_min: refused unless Force
            tc.keepRunFiles();
            args = {'Rows', 4, 'Minutes', 5, 'Ambient', 30, 'Sprayer', true, 'Fast', false, 'DryRun', true};
            run1 = tc.launch(args{:});            % the folder of a run still going (a dry run takes no lock: written here)
            lock = fullfile(char(currentProject().RootFolder), 'work', 'rowtest.lock');
            tc.writeLock(lock, run1.folder, datetime('now'), run1.est_min);
            try
                tc.launch(args{:});
                tc.verifyFail('a second row test while the first one runs');
            catch e
                tc.verifyEqual(e.identifier, 'phase1_row_test:busy');
                tc.verifySubstring(e.message, run1.folder, 'the message names the running folder');
            end
            tc.verifyNotEmpty(tc.launch(args{:}, 'Force', true), 'Force: accepted all the same');
            tc.writeLock(lock, run1.folder, datetime('now') - minutes(3*run1.est_min + 1), run1.est_min);
            tc.verifyNotEmpty(tc.launch(args{:}), 'started more than 3 x est_min ago: a stale lock');
            tc.writeLock(lock, run1.folder, datetime('now'), run1.est_min);
            writelines('finished', fullfile(run1.folder, 'done.txt'));
            tc.verifyNotEmpty(tc.launch(args{:}), 'done.txt in the folder: that run has finished');
        end

        function testWorkerError(tc)          % the worker on bad inputs: summary.txt with the error, done.txt; lock, diary, warnings
            tc.keepRunFiles();
            args = {'Rows', 4, 'Minutes', 5, 'Ambient', 30, 'Sprayer', true, 'Fast', false, 'DryRun', true};
            a = tc.launch(args{:}); b = tc.launch(args{:});
            for d = {a.folder, b.folder}          % Rows = 0 in inputs.mat
                f = fullfile(d{1}, 'inputs.mat');
                s = load(f); s.cfg.n_rows = 0; save(f, '-struct', 's');
            end
            tc.assertFalse(row_test_inputs(s.cfg, param_rowtest()), 'the worker must stop at the inputs check, before any simulation');
            lock = fullfile(char(currentProject().RootFolder), 'work', 'rowtest.lock');
            tc.writeLock(lock, a.folder, datetime('now'), a.est_min);
            lock_txt = fileread(lock);
            dy = {get(0, 'Diary'), get(0, 'DiaryFile')}; ws = warning;
            evalc('phase1_row_run(b.folder)');
            tc.verifyEqual(fileread(lock), lock_txt, 'the lock of another run (Force) stays');
            evalc('phase1_row_run(a.folder)');
            tc.verifyFalse(isfile(lock), 'the run removes its own lock');
            for d = {a.folder, b.folder}
                txt = fileread(fullfile(d{1}, 'summary.txt'));
                tc.verifySubstring(txt, 'error');
                tc.verifySubstring(txt, 'Rows', 'the message of the inputs check');
                tc.verifyTrue(isfile(fullfile(d{1}, 'done.txt')), 'done.txt on the error path');
                tc.verifyTrue(isfile(fullfile(d{1}, 'progress.log')), 'the progress log (diary)');
            end
            tc.verifyEqual({get(0, 'Diary'), get(0, 'DiaryFile')}, dy, 'the diary is back as it was');
            tc.verifyEqual(warning, ws, 'the warning states are back');
        end

        function testWindow(tc)               % the start window, hidden: pre-filled, the estimate follows the inputs, Run checks them
            R = param_rowtest();
            c = R; c.n_rows = 4; c.minutes = 5; c.T_amb_C = 30; c.sprayer = 1;
            f = row_test_window(c, R, 'Visible', 'off');
            tc.addTeardown(@delete, f);
            h = @(tag) findobj(f, 'Tag', tag);
            tc.verifyEqual(char(f.Visible), 'off');
            tc.verifyEqual([numel(findobj(f, 'Type', 'uinumericeditfield')) numel(findobj(f, 'Type', 'uicheckbox')) ...
                numel(findobj(f, 'Type', 'uibutton'))], [3 2 2], '3 numeric fields, 2 check boxes, Run and Cancel');
            tc.verifyEqual([h('rows').Value h('minutes').Value h('ambient').Value h('sprayer').Value h('fast').Value], [4 5 30 1 0]);
            about = @(k) local_about(R.t_compile/60 + 5/R.rtf(k));   % the window's text for 5 min, case k of R.rtf
            tc.verifySubstring(h('estimate').Text, about(2), 'compile + 5 min, full detail with the sprayer');
            fast = h('fast'); fast.Value = true; fast.ValueChangedFcn(fast, []);
            tc.verifySubstring(h('estimate').Text, about(4), 'compile + 5 min, fast with the sprayer');
            rows = h('rows'); rows.Value = 1; rows.ValueChangedFcn(rows, []);
            tc.verifySubstring(h('estimate').Text, 'Rows', 'what is wrong instead of an estimate');
            go = h('run'); go.ButtonPushedFcn(go, []);
            tc.verifyEqual(f.UserData.choice, '', 'invalid inputs: the window stays');
            rows.Value = 4; go.ButtonPushedFcn(go, []);
            tc.verifyEqual(f.UserData.choice, 'run');
            c.fast = 1;
            tc.verifyEqual(f.UserData.cfg, c, 'the inputs as row_test_inputs returns them');
            g = row_test_window(R, R, 'Visible', 'off');
            tc.addTeardown(@delete, g);
            cancel = findobj(g, 'Tag', 'cancel'); cancel.ButtonPushedFcn(cancel, []);
            tc.verifyEqual(g.UserData.choice, 'cancel');
            g.UserData.choice = ''; g.CloseRequestFcn(g, []);
            tc.verifyEqual(g.UserData.choice, 'cancel', 'closing the window is Cancel');
            tc.verifyTrue(isvalid(g), 'the caller deletes the window after uiwait');
        end

        function testWorld7Contract(tc)       % world 7 row_field: the contract of world 1, flat farm soil, no obstacles, 241 x 241 grid sized to the rows
            W1 = param_world(1, 9.81);
            % res = max(0.25, span/240) rounded up to a multiple of 1/64 m (grid steps exact to the last bit)
            R = param_rowtest();                          % 6 rows of 50 m: span 50 + 2*(6 + 5) = 72 m, 72/240 = 0.3 -> res 0.3125 m
            R30 = R; R30.n_rows = 30;                     % the most rows: the 87 m across the rows set the span, 109 m: 109/240 -> res 0.46875 m
            Rs = R; Rs.n_rows = 2; Rs.L_row = 30;         % a small field: span 52 m, res stays at 0.25 m (as worlds 1-2)
            tc.verifyEqual(param_world(7, 9.81), param_world(7, 9.81, R), 'without R: the param_rowtest defaults');
            for c = {R, 0.3125; R30, 0.46875; Rs, 0.25}'
                [Rk, res] = c{:};
                lbl = sprintf('%d rows of %g m', Rk.n_rows, Rk.L_row);
                W = param_world(7, 9.81, Rk);
                tc.verifyEqual(fieldnames(W), fieldnames(W1), [lbl ': the field names of world 1']);
                [ok, msg] = world_check(W, 'Psens', tc.P.sensors);     % sub-struct fields, numeric only, sizes, lists, occ, poses
                tc.verifyTrue(ok, [lbl ': world_check: ' strjoin(msg, '; ')]);
                tc.verifyEqual(W.id, 7, lbl);
                tc.verifyEqual([numel(W.terrain.xE) numel(W.terrain.yN)], [241 241], [lbl ': grid size']);
                for ax = {'xE', 'yN'}                     % the plant's 2-D Lookup Tables on P.world use 'Evenly spaced points'
                    n_step = numel(unique(diff(W.terrain.(ax{1}))));
                    tc.verifyEqual(n_step, 1, sprintf(['%s: terrain.%s has %d distinct steps; it must have exactly one ' ...
                        '(evenly spaced to the last bit, as the 2-D Lookup Tables need)'], lbl, ax{1}, n_step));
                end
                tc.verifyEqual(W.terrain.res, res, 'AbsTol', 1e-12, [lbl ': res = max(0.25, span/240) rounded up to 1/64 m']);
                tc.verifyEqual(W.rows, row_test_rows(Rk), [lbl ': rows']);
                tc.verifyEqual(W.nRows, Rk.n_rows, [lbl ': nRows']);
                m = Rk.headland + Rk.margin;              % the field (the row lines) with a headland and the margin on every side
                fx = [min(W.rows(:, 1)) - m, max(W.rows(:, 3)) + m]; fy = [min(W.rows(:, 2)) - m, max(W.rows(:, 4)) + m];
                tc.verifyTrue(W.xlim(1) <= fx(1) + 1e-9 && W.xlim(2) >= fx(2) - 1e-9 && W.ylim(1) <= fy(1) + 1e-9 ...
                    && W.ylim(2) >= fy(2) - 1e-9, sprintf('%s: the field %s x %s inside xlim / ylim', lbl, mat2str(fx), mat2str(fy)));
                tc.verifyEqual(W.boundary, [W.xlim(1) + 1, W.ylim(1) + 1; W.xlim(2) - 1, W.ylim(1) + 1; ...
                    W.xlim(2) - 1, W.ylim(2) - 1; W.xlim(1) + 1, W.ylim(2) - 1], 'AbsTol', 1e-12, [lbl ': boundary = the grid edge minus 1 m']);
                tc.verifyEqual(W.start, [-Rk.headland/2 0 0], [lbl ': start half a headland before row 1, heading east']);
                tc.verifyEqual(W.start, row_test_prm(tc.P, Rk).start, [lbl ': the start pose of the driver']);
                tc.verifyEqual(W.dock.pose, [-Rk.headland 0 0], [lbl ': the dock at the outer edge of the headland, off the driven path']);
                % the ground: flat, farm soil (class 2) everywhere, no obstacles; env, site and soil table as world 1
                tc.verifyEqual([W.terrain.H W.terrain.dHdE W.terrain.dHdN], zeros(241, 3*241), [lbl ': flat']);
                tc.verifyEqual(W.soilId, 2*ones(241), [lbl ': farm soil (class 2) everywhere']);
                p = fieldnames(W.soil); t2 = W.soilTable(W.soilTable(:, 1) == 2, :);
                for j = 1:numel(p)
                    tc.verifyEqual(W.soil.(p{j}), t2(j + 1)*ones(241), [lbl ': soil.' p{j} ' of class 2']);
                end
                tc.verifyEqual([W.obstacles.nCyl W.obstacles.nBox W.obstacles.nFilm W.nCanopy W.gnssZones.n], zeros(1, 5), ...
                    [lbl ': no obstacles, canopy or GNSS zones (dummy rows, counts 0)']);
                tc.verifyFalse(any(W.occ.grid(:)), [lbl ': empty occupancy grid']);
                tc.verifyEqual(W.occ.res, 10, [lbl ': occupancy grid at 10 cells/m']);
                tc.verifyEqual({W.env, W.lla0, W.soilTable, W.soilDefault}, {W1.env, W1.lla0, W1.soilTable, W1.soilDefault}, ...
                    [lbl ': env, site and soil table as world 1']);
            end
        end

        function testParamsNumeric(tc)        % shaddad_params('World', 7): P numeric / logical only (D58); P.scen.rowtest and P.scen.rowdrv
            [P, ~, ~, M] = shaddad_params('World', 7, 'Assign', false);
            tc.verifyEmpty(local_nonnumeric(P, 'P'), 'D58: every leaf of P numeric or logical (docs/architecture.md)');
            tc.verifyEqual([P.world.id P.scen.world_id], [7 7]);
            tc.verifyEqual(P.scen.rowtest, param_rowtest(), 'the settings: param_rowtest');
            tc.verifyEqual(P.world, param_world(7, P.frames.g, P.scen.rowtest), 'world 7 sized from P.scen.rowtest');
            tc.verifyEqual(M.world_names{7}, 'row_field');
            tc.verifyEqual(P.scen.rowdrv, row_test_prm(P, P.scen.rowtest), 'the driver parameters: row_test_prm');
            tc.verifyTrue(all(structfun(@(x) isa(x, 'double'), P.scen.rowdrv)), 'P.scen.rowdrv: doubles only');
            % every world gets them, so the model compiles with any operator source: world 1 of the class setup
            tc.verifyEmpty(local_nonnumeric(tc.P, 'P'), 'D58, world 1');
            tc.verifyEqual(tc.P.scen.rowtest, P.scen.rowtest, 'world 1: the same settings');
            tc.verifyEqual(tc.P.scen.rowdrv, P.scen.rowdrv, 'world 1: the same driver parameters');
        end

        function testPlotRunFolderFig(tc)     % Save, Folder, Fig: 6 PNG + 6 FIG in that folder; each FIG opens visible
            L = dir(fullfile(char(currentProject().RootFolder), 'work', 'results', 'D1_*.mat'));
            tc.assumeNotEmpty(L, 'no D1 log in work/results (phase1_test_drive writes one)');
            [~, i] = max([L.datenum]);
            file = fullfile(L(i).folder, L(i).name);                               %#ok<NASGU> evalc below
            fx = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tmp = fullfile(fx.Folder, 'pages');   % not there yet: plot_run makes it
            args = {'Save', true, 'Folder', tmp, 'Fig', true, 'Visible', 'off', 'Note', 'full detail'};  %#ok<NASGU>
            [txt, figs] = evalc('plot_run(file, args{:})');                        % args: as phase1_row_run calls it
            tc.addTeardown(@close, figs);
            pages = {'drive', 'tracks', 'battery', 'power', 'rc', 'nav'};
            names = arrayfun(@(k) sprintf('%d_%s', k, pages{k}), 1:6, 'UniformOutput', false);
            for ext = {'.png', '.fig'}
                d = dir(fullfile(tmp, ['*' ext{1}]));
                tc.verifyEqual(sort({d.name}), strcat(names, ext{1}), ['the six pages as ' ext{1}]);
            end
            tc.verifySubstring(txt, ['Plots saved in ' tmp], 'the folder is printed');
            for k = 1:6                           % the pages returned stay as drawn; the saved ones open visible
                tc.verifyEqual(char(figs(k).Visible), 'off', [names{k} ': the page returned is still hidden']);
                tc.verifyEmpty(figs(k).CreateFcn, [names{k} ': the CreateFcn of the page returned is put back']);
                g = openfig(fullfile(tmp, [names{k} '.fig']));
                shut = onCleanup(@() close(g));   % closed also when a check below fails
                tc.verifyEqual(char(g.Visible), 'on', [names{k} '.fig opens visible (drawn with Visible off)']);
                tc.verifyTrue(endsWith(findobj(g, 'Type', 'tiledlayout').Title.String, '  |  full detail'), ...
                    [names{k} '.fig: the note at the end of its title']);
                clear shut
            end
        end

        function testPlotRunSaveOnly(tc)      % Folder and Fig act only with Save: without it nothing is written
            fx = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tmp = fullfile(fx.Folder, 'pages');
            f = plot_run(tc.plotRunLog(), 'Pages', {'drive'}, 'Folder', tmp, 'Fig', true, 'Visible', 'off');
            tc.addTeardown(@close, f);
            tc.verifyFalse(isfolder(tmp), 'no folder made, no file written');
        end

        function testPlotRunFieldEdge(tc)     % path tile: world 7's edge from the logged map, other worlds as before
            R = param_rowtest(); R.n_rows = 2;
            occ = param_world(7, 9.81, R).occ;    % the map an R1 run with 2 rows logs (run_scenario: res.viz.occ)
            lim = occ.origin' + [0 size(occ.grid, 2); 0 size(occ.grid, 1)]/occ.res;   % its [xlim; ylim]
            W7 = param_world(7, 9.81);
            tc.assertNotEqual(lim, [W7.xlim; W7.ylim], 'the map of 2 rows differs from the world 7 defaults (6 rows)');
            [edge, ax] = tc.fieldEdge(tc.plotRunLog(struct('occ', occ, 'world_id', 7)));
            tc.assertNumElements(edge, 1, 'world 7: one field map edge');
            tc.verifyEqual([edge.XData; edge.YData], [lim(1, [1 2 2 1 1]); lim(2, [1 1 2 2 1])], 'AbsTol', 1e-9, ...
                'world 7: the edge of the logged map, not of the default rows');
            tc.verifyEmpty(findall(ax, 'Type', 'rectangle', '-or', 'Type', 'patch'), 'world 7: no obstacles');
            W1 = param_world(1, 9.81);            % world 1 with the same map logged: world 1's edge and obstacles
            [edge, ax] = tc.fieldEdge(tc.plotRunLog(struct('occ', occ, 'world_id', 1)));
            tc.assertNumElements(edge, 1, 'world 1: one field map edge');
            tc.verifyEqual([edge.XData; edge.YData], [W1.xlim([1 2 2 1 1]); W1.ylim([1 1 2 2 1])], ...
                'world 1: its own edge, as before');
            tc.verifyEqual([numel(findall(ax, 'Type', 'rectangle')) numel(findall(ax, 'Type', 'patch'))], ...
                [W1.obstacles.nCyl W1.obstacles.nBox], 'world 1: its cylinders and boxes, as before');
            [edge, ax] = tc.fieldEdge(tc.plotRunLog());   % no viz (a log from before run_scenario stored it): no field
            tc.verifyNotEmpty(ax, 'the path tile');
            tc.verifyEmpty(edge, 'no viz: no field map edge');
        end

        function testPlotRunNote(tc)          % Note: added to every page title; without it the title is as before
            run = tc.plotRunLog();
            p = run.S.truth.pos_W.v;              % the title: log name, end of the MCU mode log, true path length
            base = sprintf('run  |  %.0f s, %.1f m', run.S.telem.mode.t(end), sum(vecnorm(diff(p(:, 1:2)), 2, 2)));
            names = {'Commands and response', 'Tracks and motors', 'Battery and energy', ...
                'Power electronics, safety chain and temperatures', 'RC link and MCU health', ...
                'Navigation and vehicle dynamics'};
            f = plot_run(run, 'Visible', 'off', 'Note', 'full detail');    % the fidelity, as the worker gives it
            tc.addTeardown(@close, f);
            tc.assertNumElements(f, 6);
            for k = 1:6
                tc.verifyEqual(findobj(f(k), 'Type', 'tiledlayout').Title.String, ...
                    sprintf('%s  -  %s  |  full detail', names{k}, base), ...
                    sprintf('page %d: the note after the log figures', k));
            end
            g = plot_run(run, 'Pages', {'drive'}, 'Visible', 'off');
            tc.addTeardown(@close, g);
            tc.verifyEqual(findobj(g, 'Type', 'tiledlayout').Title.String, sprintf('%s  -  %s', names{1}, base), ...
                'no Note: the title as before');
        end
    end

    methods (Static)
        function [S, run, cfg, rows] = synthRun(o)
            % [S, run, cfg, rows] = tc.synthRun(o): a synthetic run log for the summary and field page tests, in the layout of run_scenario
            % (S.<group>.<signal>.t / .v, the groups at their own rates: telem, truth, drive_truth 10 Hz; kpi, power_truth,
            % bms_telem 2 Hz), with run, cfg (param_rowtest with 30 degC, sprayer on, full detail) and rows (row_test_rows).
            % The robot is armed (MANUAL) at 2 s, drives 4 rows at 1 m/s (50 m rows, 3 m apart) east and west in turn through
            % straight headland connectors 1.5 m beyond the row ends, back to row 1 after the last row (2 passes of the path),
            % weaves 0.25 m about the row lines, stands from the time asked on; the pack gives 360 W, the SoC falls from 0.95.
            % The pack current is 7.5 A with a peak of 14.2 A at 100 s and a regeneration peak of -16 A at 150 s. The
            % true cell maximum rises from 34 degC (2 mK/s), the BMS reading of it from 35 degC (4 mK/s). The first
            % samples hold the start-up values the summary must ignore: RC link / warning / BMS bits, an enclosure
            % temperature of -273.15 and 120 degC, an inrush of 30 A at 0.5 s.
            %   o (struct, every field optional):
            %     minutes    driving time asked (7: rows 1-3 twice, row 4 once; 8 drives the whole path, then stands in row 4)
            %     exit       [t mode] the MCU leaves MANUAL for mode (ShaddadMode code) at t s; the log ends 3.5 s later
            %     soc_stop   t: Sim_Monitors stops the run at t s (kpi.stop_reason 3), the MCU stays in MANUAL
            %     bms_fault  [t word] BMS Fault_Word from t s on
            %   Without exit and soc_stop the MCU leaves MANUAL (to STANDBY) 3 s after the time, the log ends 5 s later.
            if nargin < 1, o = struct(); end
            d = struct('minutes', 7, 'exit', [], 'soc_stop', [], 'bms_fault', []);
            f = fieldnames(o);
            for k = 1:numel(f), d.(f{k}) = o.(f{k}); end
            cfg = param_rowtest(); cfg.n_rows = 4; cfg.minutes = d.minutes; cfg.T_amb_C = 30; cfg.sprayer = 1; cfg.fast = 0;
            rows = row_test_rows(cfg);
            % path (even number of rows): start pose, then per row its two ends and the connector to the next row
            W = [-cfg.headland/2 0];
            for p = 1:2
                for k = 1:cfg.n_rows
                    y = cfg.spacing*(k - 1);
                    if mod(k, 2) == 1, x = [0 cfg.L_row]; xh = cfg.L_row + cfg.spacing/2; else, x = [cfg.L_row 0]; xh = -cfg.spacing/2; end
                    W = [W; x(1) y; x(2) y]; %#ok<AGROW>
                    if k < cfg.n_rows, W = [W; xh y; xh y + cfg.spacing]; end %#ok<AGROW>
                end
                if p < 2, W = [W; xh y; xh 0]; end %#ok<AGROW>
            end
            s = [0; cumsum(hypot(diff(W(:, 1)), diff(W(:, 2))))];    % distance along the path, m = s at 1 m/s
            % times: MANUAL from 2 s, sticks released at t_stop, the MCU leaves MANUAL at T_exit, the log ends at t_end
            t_man = 2; t_stop = t_man + 60*d.minutes;
            T_exit = t_stop + 3; m_exit = 2; t_end = T_exit + 5;
            if ~isempty(d.exit), T_exit = d.exit(1); m_exit = d.exit(2); t_end = T_exit + 3.5; end
            if ~isempty(d.soc_stop), t_end = d.soc_stop; T_exit = Inf; end
            tf = (0:round(t_end/0.1))'*0.1;
            ts = (0:round(t_end/0.5))'*0.5;
            sig = @(t, x) struct('t', t, 'v', x);
            % fast group: MCU telemetry, true pose, drive temperatures
            mode = zeros(size(tf), 'uint8'); mode(tf >= 0.1) = 1; mode(tf >= 1) = 2; mode(tf >= t_man) = 3; mode(tf >= T_exit) = m_exit;
            xy = interp1(s, W, min(max(min(tf, t_stop) - t_man, 0), s(end)));
            xy(:, 2) = xy(:, 2) + 0.25*sin(2*pi*tf/20);
            Te = single(45 + 0.01*tf); Te(1) = -273.15; Te(2) = 120;
            S.telem.mode = sig(tf, mode);
            S.telem.fault_word = sig(tf, uint32(4*(tf < 0.5)));
            S.telem.warn_word = sig(tf, uint32(524292*(tf < 0.15)));
            S.telem.T_encl_degC = sig(tf, Te);
            S.truth.pose2D = sig(tf, [xy, zeros(size(tf))]);
            kw = struct('L', 0.02, 'R', 0.015); ke = struct('L', 0.01, 'R', 0.012);   % K/s: the left winding and the right controller run hotter
            for side = 'LR'
                S.drive_truth.(side).T_winding = sig(tf, 318.15 + kw.(side)*tf);
                S.drive_truth.(side).T_esc = sig(tf, 318.15 + ke.(side)*tf);
            end
            % slow group: KPIs, pack, BMS
            sr = zeros(size(ts), 'uint8');
            if ~isempty(d.soc_stop), sr(ts >= d.soc_stop - 0.5) = 3; end
            fw = uint32(65536*(ts < 0.1));
            if ~isempty(d.bms_fault), fw(ts >= d.bms_fault(1)) = d.bms_fault(2); end
            Ip = 7.5*ones(size(ts)); Ip(2) = 30; Ip(ts == 100) = 14.2; Ip(ts == 150) = -16;
            S.kpi.dist = sig(ts, min(max(min(ts, t_stop) - t_man, 0), s(end)));
            S.kpi.E_batt = sig(ts, 360*ts);
            S.kpi.stop_reason = sig(ts, sr);
            S.power_truth.SOC_pack_true = sig(ts, 0.95 - 1e-4*ts);
            S.power_truth.I_pack_A = sig(ts, Ip);
            S.power_truth.T_cell_max_K = sig(ts, 307.15 + 0.002*ts);   % the true cells: cooler than the BMS reads them
            S.bms_telem.T_cell_max_K = sig(ts, single(308.15 + 0.004*ts));
            S.bms_telem.Fault_Word = sig(ts, fw);
            run = struct('wall_s', 3725, 'rtf', 0.13, 't_end', t_end, 'file', 'C:\work\results\R1_20261028_101500.mat', 'msg', '');
        end

        function run = plotRunLog(viz)
            % run = tc.plotRunLog(viz): the synthetic log of tc.synthRun as a run for plot_run (run.S), with the
            % true position truth.pos_W (from truth.pose2D: the path tile and its field need it) and, when given,
            % viz as run_scenario stores it (occ, world_id)
            S = t_row_driver.synthRun(); p = S.truth.pose2D;
            S.truth.pos_W = struct('t', p.t, 'v', [p.v(:, 1:2), zeros(numel(p.t), 1)]);
            run = struct('S', S);
            if nargin > 0, run.viz = viz; end
        end
    end

    methods
        function B = bench(tc, nRows, T, minutes, sprayer, faultAt, tank0)
            % B = tc.bench(nRows, T, minutes, sprayer, faultAt, tank0): param_rowtest with n_rows, minutes and sprayer
            % set, prm from row_test_prm, row_driver_bench for T seconds (MCU FAULT from faultAt s on, default none;
            % tank0 L in the sprayer tank at the start, default Inf: a tank that never empties). Keeps R and prm in
            % tc.R / tc.prm.
            if nargin < 5, sprayer = false; end
            if nargin < 6, faultAt = Inf; end
            if nargin < 7, tank0 = Inf; end
            R = param_rowtest(); R.n_rows = nRows; R.minutes = minutes; R.sprayer = double(sprayer);
            tc.R = R;
            tc.prm = row_test_prm(tc.P, R);
            B = row_driver_bench(tc.prm, T, struct('fault_at', faultAt, 'tank0', tank0));
        end

        function keepRunFiles(tc)
            % tc.keepRunFiles(): work/rowtest.lock and work/rowtest_last.mat are taken away for this test and put back as they
            % were after it (a row test running in the background keeps its lock); the run folders work/reports/rowtest_*
            % that appear during the test are removed (no other process prepares a row test meanwhile).
            work = fullfile(char(currentProject().RootFolder), 'work');
            for name = {'rowtest.lock', 'rowtest_last.mat'}
                p = fullfile(work, name{1});
                had = isfile(p); bytes = zeros(0, 1, 'uint8');
                if had
                    fid = fopen(p, 'r'); bytes = fread(fid, Inf, '*uint8'); fclose(fid);
                    delete(p);
                end
                tc.addTeardown(@local_put_back, p, had, bytes);
            end
            reports = fullfile(work, 'reports');
            d0 = dir(fullfile(reports, 'rowtest_*'));
            tc.addTeardown(@local_remove_new, reports, {d0.name});
        end

        function [job, txt] = launch(~, varargin)
            % [job, txt] = tc.launch(...): phase1_row_test(...), its Command Window text captured in txt. A start window that
            % opens by mistake (the launch then waits in uiwait) is deleted after 20 s, so no test can block MATLAB: the
            % launch then returns [] as for Cancel.
            t = timer('Name', 't_row_driver_launch', 'StartDelay', 20, ...
                'TimerFcn', @(~, ~) delete(findall(groot, 'Tag', 'row_test_window')));
            c = onCleanup(@() local_end_timer(t));
            start(t);
            [txt, job] = evalc('phase1_row_test(varargin{:})');
        end

        function writeLock(~, lock, folder, t0, est_min)
            % tc.writeLock(lock, folder, t0, est_min): work/rowtest.lock as phase1_row_test writes it when it starts a run
            writelines({['folder  ' folder]; ['start   ' char(t0, 'yyyy-MM-dd HH:mm:ss')]; sprintf('est_min %.6g', est_min)}, lock);
        end

        function [edge, ax] = fieldEdge(tc, run)
            % [edge, ax] = tc.fieldEdge(run): the drive page of plot_run(run), hidden and closed after the test:
            % its path tile ax (the tile with the start marker) and there the line 'field map edge' (empty when
            % no field is drawn)
            f = plot_run(run, 'Pages', {'drive'}, 'Visible', 'off');
            tc.addTeardown(@close, f);
            ax = ancestor(findobj(f, 'Type', 'line', 'DisplayName', 'start'), 'axes');
            edge = findobj(ax, 'Type', 'line', 'DisplayName', 'field map edge');
        end
    end
end

% -------------------------------------------------------------------------------------------
function local_put_back(p, had, bytes)
% teardown of keepRunFiles: the file as it was before the test, or no file
if had
    fid = fopen(p, 'w'); fwrite(fid, bytes); fclose(fid);
elseif isfile(p)
    delete(p);
end
end

function local_end_timer(t)
% cleanup of launch: its guard timer stopped and deleted
stop(t);
delete(t);
end

function local_remove_new(reports, before)
% teardown of keepRunFiles: remove the run folders that were not there before the test
d = dir(fullfile(reports, 'rowtest_*'));
for k = find([d.isdir] & ~ismember({d.name}, before))
    rmdir(fullfile(reports, d(k).name), 's');
end
end

function bad = local_nonnumeric(S, nm)
% testParamsNumeric: the leaves of the struct S (named nm) that are not numeric or logical, or not real (D58)
bad = {};
f = fieldnames(S);
for k = 1:numel(f)
    v = S.(f{k});
    if isstruct(v)
        for j = 1:numel(v), bad = [bad, local_nonnumeric(v(j), [nm '.' f{k}])]; end %#ok<AGROW>
    elseif ~(isnumeric(v) || islogical(v)) || ~isreal(v)
        bad{end+1} = sprintf('%s.%s (%s)', nm, f{k}, class(v)); %#ok<AGROW>
    end
end
end

function s = local_about(est)
% testWindow: an estimate of est min as the start window writes it (whole minutes): 'about 44 min', 'about 1 h 05 min'
m = round(est);
if m < 60, s = sprintf('about %d min', m); else, s = sprintf('about %d h %02d min', floor(m/60), mod(m, 60)); end
end
