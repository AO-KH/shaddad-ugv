classdef t_power_plant < matlab.unittest.TestCase
%T_POWER_PLANT  T05 acceptance tests: Power_Plant, PDU, safety chain (D30 re-arm), LV rails, implements + ECU,
%   dock charger, energy audit and the 24 h shift cycle (owner: power, T05; build plan T05 acceptance).
%
%   runtests('t_power_plant')
%   Benches: tests/power/tb_power_plant.slx, tb_shift_cycle.slx, tb_lv_rails.slx, tb_impl_codec.slx
%   (scripts/build/build_tb_power_plant.m). Short runs go through pp_run (independent of the shared base
%   workspace); the long cases (1 h audit, 1.5 x F_TL overload, 2 x 24 h shift cycle) through the content-keyed cache
%   pp_case (work/T05). Measured numbers are written to the test log (verbosity 1) and to work/T05/results.mat.

    properties (Constant)
        VC0 = struct('VC_PACK', 2, 'VC_IMPL', 0, 'VC_CHG', 1)
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''Bus_PowerStatus'', ''var'')'), shaddad_buses(); end
            % bench builds (idempotent); P.power in base refreshed from params/param_power.m (own domain only)
            if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_params(); end
            evalin('base', 'P.power = param_power();');
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            for b = {'tb_power_plant', 'tb_shift_cycle', 'tb_lv_rails', 'tb_impl_codec'}
                if ~isfile(fullfile(root, 'tests', 'power', [b{1} '.slx']))
                    build_tb_power_plant(b);
                end
            end
            tc.addTeardown(@() t_power_plant.close_benches());
        end
    end

    methods (Static)
        function close_benches()
            for b = {'tb_power_plant', 'tb_shift_cycle', 'tb_lv_rails', 'tb_impl_codec'}
                if bdIsLoaded(b{1}), close_system(b{1}, 0); end
            end
        end
        function t = t_first(t_vec, cond, t_from)
            % first time >= t_from where cond holds (NaN if never)
            if nargin < 3, t_from = -inf; end
            k = find(cond(:) & t_vec(:) >= t_from, 1);
            if isempty(k), t = NaN; else, t = t_vec(k); end
        end
        function record(name, m)
            % keep the measured numbers (work/T05/results.mat, struct field per test)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            d = fullfile(root, 'work', 'T05');
            if ~isfolder(d), mkdir(d); end
            f = fullfile(d, 'results.mat');
            S = struct();
            if isfile(f), S = load(f); end
            S.(name) = m;
            save(f, '-struct', 'S');
        end
        function P = pm_q0(P, q)
            P.power.safety.Q0 = q;
        end
    end

    methods (Test)
        % ================================================================== structure / interfaces
        function interfaces(tc)
            n = shaddad_check_interfaces('Files', {'Power_Plant'}, 'Quiet', true);
            tc.verifyEqual(n, 0, 'Power_Plant root ports differ from the ICD 3.2 contract');
        end

        function no_matlab_function_blocks(tc)
            % Power_Plant contains every T05 Subsystem Reference (all variant choices are searched)
            files = {'Power_Plant'};
            nMF = 0; nSolver = 0; nRef = 0;
            for k = 1:numel(files)
                wl = bdIsLoaded(files{k});
                load_system(files{k});
                nMF = nMF + numel(find_system(files{k}, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants, ...
                    'SFBlockType', 'MATLAB Function'));
                nSolver = nSolver + numel(find_system(files{k}, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants, ...
                    'ReferenceBlock', sprintf('nesl_utility/Solver\nConfiguration')));
                nRef = nRef + numel(find_system(files{k}, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants, ...
                    'ReferenceBlock', 'fl_lib/Electrical/Electrical Elements/Electrical Reference'));   % ee_lib links here
                if ~wl && ~any(strcmp(files{k}, {'Power_Plant'})), close_system(files{k}, 0); end
            end
            tc.verifyEqual(nMF, 0, 'T05 files contain MATLAB Function blocks');
            tc.verifyEqual(nSolver, 0, 'T05 domain files must not contain a Solver Configuration');
            tc.verifyEqual(nRef, 1, 'exactly one Electrical Reference (PDU GND) expected in the T05 files');
        end

        function compiles_all_variants(tc)
            % Power_Plant compiles for VC_IMPL 0..3 (no isolated network), Grouped pack and the CCCV charger
            cases = {struct('VC_PACK', 2, 'VC_IMPL', 0, 'VC_CHG', 1), struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), ...
                     struct('VC_PACK', 2, 'VC_IMPL', 2, 'VC_CHG', 1), struct('VC_PACK', 2, 'VC_IMPL', 3, 'VC_CHG', 1), ...
                     struct('VC_PACK', 1, 'VC_IMPL', 1, 'VC_CHG', 1), struct('VC_PACK', 2, 'VC_IMPL', 0, 'VC_CHG', 2)};
            ok = false(1, numel(cases)); msg = cell(1, numel(cases));
            for k = 1:numel(cases)
                R = pp_run('tb_power_plant', [], 'VC', cases{k}, 'CompileOnly', true);
                ok(k) = R.ok; msg{k} = R.err;
                tc.log(1, sprintf('compile VC_PACK %d VC_IMPL %d VC_CHG %d: %d (%.1f s)', cases{k}.VC_PACK, ...
                    cases{k}.VC_IMPL, cases{k}.VC_CHG, R.ok, R.wall));
            end
            t_power_plant.record('compile', struct('ok', ok));
            for k = 1:numel(cases)
                tc.verifyTrue(ok(k), sprintf('compile case %d failed: %s', k, msg{k}));
            end
        end

        function algebraic_loops(tc)
            % getAlgebraicLoops = 0 on both benches (base workspace: P.power refreshed in the class setup)
            evalin('base', 'P.power = param_power();');
            vcs = {'VC_PACK', 'VC_IMPL', 'VC_CHG'};
            old = cellfun(@(v) evalin('base', v), vcs);
            c = onCleanup(@() cellfun(@(v, x) assignin('base', v, x), vcs, num2cell(old)));
            n = zeros(1, 3);
            sets = {[2 1 1], [2 3 1], [2 0 1]};
            mdls = {'tb_power_plant', 'tb_power_plant', 'tb_shift_cycle'};
            for k = 1:3
                for j = 1:3, assignin('base', vcs{j}, sets{k}(j)); end
                load_system(mdls{k});
                loops = Simulink.BlockDiagram.getAlgebraicLoops(mdls{k});
                n(k) = numel(loops);
                tc.log(1, sprintf('%s (VC_IMPL %d): %d algebraic loops', mdls{k}, sets{k}(2), n(k)));
            end
            t_power_plant.record('algebraic_loops', struct('n', n));
            tc.verifyEqual(n, zeros(1, 3));
        end

        % ================================================================== CAN codecs
        function impl_codec_round_trip(tc)
            rs = RandStream('mt19937ar', 'Seed', 5);
            N = 1000; t = (0:N-1)' * 0.05;
            sg = @(x) double(single(x));                        % codec inputs are single on the MCU
            cmd = [t, rand(rs, N, 1) > 0.5, rand(rs, N, 1) > 0.5, floor(256 * rand(rs, N, 1)), sg(10000 * rand(rs, N, 1) - 5000)];
            fb = [t, rand(rs, N, 3) > 0.5, floor(4 * rand(rs, N, 1)), sg(10000 * rand(rs, N, 1) - 5000), ...
                  sg(600 * rand(rs, N, 1) - 300), sg(255 * rand(rs, N, 1) - 40), floor(256 * rand(rs, N, 1))];
            TB = struct('cmd', cmd, 'fb', fb);
            R = pp_run('tb_impl_codec', TB, 'StopTime', t(end));
            co = R.cmd_out; fo = R.fb_out;                     % one row per 0.05 s sample (no codec delay)
            m = min(size(co, 1), N);
            co = co(1:m, :); fo = fo(1:m, :); cmd = cmd(1:m, :); fb = fb(1:m, :);
            e.cmd_bool = max(max(abs(co(:, 1:2) - cmd(:, 2:3))));
            e.cmd_alive = max(abs(co(:, 3) - cmd(:, 4)));
            e.cmd_speed_lsb = max(abs(co(:, 4) - cmd(:, 5))) / 1;
            e.cmd_valid = min(co(:, 5));
            e.fb_bool = max(max(abs(fo(:, 1:3) - fb(:, 2:4))));
            e.fb_type = max(abs(fo(:, 4) - fb(:, 5)));
            e.fb_speed_lsb = max(abs(fo(:, 5) - fb(:, 6))) / 1;
            e.fb_current_lsb = max(abs(fo(:, 6) - fb(:, 7))) / 0.1;
            e.fb_temp_lsb = max(abs(fo(:, 7) - fb(:, 8))) / 1;
            e.fb_alive = max(abs(fo(:, 8) - fb(:, 9)));
            e.fb_valid = min(fo(:, 9));
            e.n = m;
            tc.log(1, sprintf(['IMPL codec round trip (%d samples): speed_cmd %.3f LSB, FB speed %.3f LSB, current %.3f LSB, ' ...
                'temp %.3f LSB, booleans / alive / type exact: %d'], m, e.cmd_speed_lsb, e.fb_speed_lsb, e.fb_current_lsb, ...
                e.fb_temp_lsb, e.cmd_bool + e.cmd_alive + e.fb_bool + e.fb_type + e.fb_alive == 0));
            t_power_plant.record('codec', e);
            tc.verifyEqual(m, N);
            tc.verifyLessThanOrEqual([e.cmd_speed_lsb e.fb_speed_lsb e.fb_current_lsb e.fb_temp_lsb], 0.5 + 1e-3);
            tc.verifyEqual([e.cmd_bool e.cmd_alive e.fb_bool e.fb_type e.fb_alive], zeros(1, 5));
            tc.verifyEqual([e.cmd_valid e.fb_valid], [1 1]);
        end

        % ================================================================== safety chain (D30)
        function rearm_powerup(tc)
            % ICD 6: DO0 rises at 10 ms, automatic 50 ms DO1 pulse 20 ms later -> Safety_OK / Safety_HV_OK <= 0.1 s
            F = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 0.25, 'PMod', @(P) t_power_plant.pm_q0(P, 0));
            f = R.log_fast; t = f(:, F.t);
            m.t_SOK = t_power_plant.t_first(t, f(:, F.Safety_OK) > 0.5);
            m.t_SHV = t_power_plant.t_first(t, f(:, F.Safety_HV_OK) > 0.5);
            m.open_before_DO1 = all(f(t < 0.030, F.Safety_OK) < 0.5) && all(f(t < 0.030, F.Safety_HV_OK) < 0.5);
            tc.log(1, sprintf('power-up re-arm (Q0 = 0): Safety_OK at %.3f s, Safety_HV_OK at %.3f s (<= 0.1 s)', m.t_SOK, m.t_SHV));
            t_power_plant.record('rearm_powerup', m);
            tc.verifyTrue(m.open_before_DO1, 'relay must be open before the DO1 pulse');
            tc.verifyLessThanOrEqual([m.t_SOK m.t_SHV], 0.1);
        end

        function rearm_button_before_do0(tc)
            % reset button pressed 5 ms before DO0 rises (no DO1 pulse) -> relay still closes (50 ms stretch)
            F = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            TB.do1_t = [0 1e7]; TB.do1_v = [0 0];
            TB.reset_t = [0 0.005 0.015 1e7]; TB.reset_v = [0 1 0 0];
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 0.2, 'PMod', @(P) t_power_plant.pm_q0(P, 0));
            f = R.log_fast; t = f(:, F.t);
            m.t_SOK = t_power_plant.t_first(t, f(:, F.Safety_OK) > 0.5);
            m.t_latch = t_power_plant.t_first(t, f(:, F.Latched_off) < 0.5);
            tc.log(1, sprintf('button 5 ms before DO0: latch set at %.3f s, Safety_OK at %.3f s', m.t_latch, m.t_SOK));
            t_power_plant.record('rearm_button', m);
            tc.verifyLessThanOrEqual(m.t_SOK, 0.1, 'relay must close through the stretched reset edge');
        end

        function rearm_blocked_by_estop(tc)
            % DO1 pulse while an E-stop is pressed -> relay stays open (also after the release, until a new reset)
            F = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            TB.estopF_t = [0 0.3 1e7]; TB.estopF_v = [1 0 0];
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 0.5, 'PMod', @(P) t_power_plant.pm_q0(P, 0));
            f = R.log_fast;
            m.max_SOK = max(f(:, F.Safety_OK)); m.max_SHV = max(f(:, F.Safety_HV_OK));
            tc.log(1, sprintf('DO1 pulse with E-stop pressed: max Safety_OK %d, Safety_HV_OK %d (0 expected)', m.max_SOK, m.max_SHV));
            t_power_plant.record('rearm_estop', m);
            tc.verifyEqual([m.max_SOK m.max_SHV], [0 0]);
        end

        function estop_ch2_stuck(tc)
            % estop_ch2_stuck -> EStop_Ch1_OK != EStop_Ch2_OK while pressed (and both false without the fault)
            F = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            TB.estopF_t = [0 0.2 0.4 1e7]; TB.estopF_v = [0 1 0 0];
            TB.ch2_t = [0 1e7]; TB.ch2_v = [1 1];
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 0.5);
            f = R.log_fast; t = f(:, F.t); pr = t > 0.205 & t < 0.395;
            m.discrepancy = all(f(pr, F.Ch1_OK) == 0 & f(pr, F.Ch2_OK) == 1);
            TB.ch2_v = [0 0];
            R2 = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 0.5);
            f2 = R2.log_fast;
            m.both_open_without_fault = all(f2(pr, F.Ch1_OK) == 0 & f2(pr, F.Ch2_OK) == 0);
            tc.log(1, sprintf('E-stop pressed with ch2 stuck: Ch1 0 / Ch2 1 = %d; without fault both 0 = %d', ...
                m.discrepancy, m.both_open_without_fault));
            t_power_plant.record('estop_ch2', m);
            tc.verifyTrue(m.discrepancy);
            tc.verifyTrue(m.both_open_without_fault);
        end

        % ================================================================== precharge
        function precharge_T03(tc)
            % HV_Enable_Req at 1.0 s into 5.5 mF (2 x 2.2 mF DRIVE stubs + 1 mF mower link + 0.1 mF PDU)
            % The 0.42 s of the spec hand calculation (tau = 22 Ohm x 5.5 mF) runs from the start of the precharge
            % (K2 contact closed). The request -> K2 latency of the BMS / CAN chain (MCU frame 50 ms, BMS 0.05 -> 0.01
            % RT, BMS output Unit Delay, 0.01 -> 0.001 RT, K2 pickup) is reported separately (T04 interface note).
            [F, S, B] = pp_log_cols();
            TB = tb_power_plant_default();
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 2.5);
            f = R.log_fast; t = f(:, F.t); b = R.log_bms; s = R.log_slow;
            m.t_K2 = t_power_plant.t_first(t, f(:, F.K2) > 0.5);
            m.t_97 = t_power_plant.t_first(t, f(:, F.V_bus) >= 0.97 * f(:, F.V_pack), 1.0);
            m.dt_97 = m.t_97 - m.t_K2;
            m.dt_97_from_mcu_request = m.t_97 - 1.0;
            m.t_K2cmd = t_power_plant.t_first(s(:, S.t), s(:, S.K2_Cmd) > 0.5);
            m.t_K1 = t_power_plant.t_first(t, f(:, F.K1) > 0.5);
            m.t_K2_open = t_power_plant.t_first(t, f(:, F.K2) < 0.5, m.t_K2);
            m.dK2open_contacts = m.t_K2_open - m.t_K1;
            m.dK2open = t_power_plant.t_first(s(:, S.t), s(:, S.K2_Cmd) < 0.5, m.t_K2cmd) - ...
                        t_power_plant.t_first(s(:, S.t), s(:, S.K1_Cmd) > 0.5);
            k1 = find(t >= m.t_K1, 1);
            Ibase = f(k1 - 1, F.I_pack);
            w = t >= m.t_K1 & t < m.t_K1 + 0.02;
            m.I2t = sum((f(w, F.I_pack) - Ibase).^2) * 1e-3;
            m.fuse_blown = max(f(:, F.Fuse_Blown));
            m.t_HV_Ready = t_power_plant.t_first(b(:, B.t), b(:, B.HV_Ready) > 0.5);
            tc.log(1, sprintf(['T-PWR-03: MCU request 1.000 s, K2_Cmd %.3f s, K2 closes %.3f s; V_bus >= 0.97 V_pack %.3f s ' ...
                'after K2 closed (0.42 +/- 0.05; %.3f s after the MCU request), K1 %.3f s, K2 command off %.0f ms after K1 ' ...
                'command (contacts %.0f ms), K1 I^2t %.3f A^2s (BE 1 ms), fuses %d, HV_Ready %.2f s (<= 1.60)'], ...
                m.t_K2cmd, m.t_K2, m.dt_97, m.dt_97_from_mcu_request, m.t_K1, 1e3 * m.dK2open, 1e3 * m.dK2open_contacts, ...
                m.I2t, m.fuse_blown, m.t_HV_Ready));
            t_power_plant.record('precharge', m);
            tc.verifyEqual(m.dt_97, 0.42, 'AbsTol', 0.05);
            tc.verifyGreaterThan(m.t_K1, m.t_97 - 1e-3);
            tc.verifyEqual(m.dK2open, 0.05, 'AbsTol', 0.01);
            tc.verifyLessThanOrEqual(m.I2t, 2);
            tc.verifyEqual(m.fuse_blown, 0);
            tc.verifyLessThanOrEqual(m.t_HV_Ready, 1.60 + 1e-6);
        end

        function precharge_fail_T03b(tc)
            % shorted bus (0.1 Ohm across DC48_TracL): PRE_FAIL 1.5 +/- 0.02 s, three retries, FAULT b6, K1 never
            [F, ~, B, T] = pp_log_cols();
            TB = tb_power_plant_default();
            TB.short_t = [0 1e7]; TB.short_v = [1 1]; TB.R_short = 0.1;
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 18);
            f = R.log_fast; t = f(:, F.t); b = R.log_bms; tr = R.log_truth;
            k2 = f(:, F.K2) > 0.5;
            on = find(diff([0; k2]) == 1); off = find(diff([k2; 0]) == -1);
            m.n_attempts = numel(on);
            m.K2_closed_s = (t(off) + 1e-3 - t(on))';
            m.K1_ever = max(f(:, F.K1));
            m.final_state = b(end, B.Contactor_State);
            m.fault_b6 = bitget(uint32(b(end, B.Fault_Word)), 7);
            Ep = tr(:, T.E_pdu_loss_Wh) * 3600;
            m.E_pre_J = zeros(1, numel(on));
            for k = 1:numel(on)
                m.E_pre_J(k) = interp1(tr(:, T.t), Ep, t(off(k)) + 0.1) - interp1(tr(:, T.t), Ep, t(on(k)) - 0.1);
            end
            tc.log(1, sprintf(['T-PWR-03b: %d precharge attempts, K2 closed %s s (1.5 +/- 0.02), K1 closed %d, final state %d ' ...
                '(10 FAULT), Fault_Word b6 %d, R_pre energy per attempt %s J (<= 200)'], m.n_attempts, mat2str(m.K2_closed_s, 4), ...
                m.K1_ever, m.final_state, m.fault_b6, mat2str(round(m.E_pre_J))));
            t_power_plant.record('precharge_fail', m);
            tc.verifyEqual(m.n_attempts, 3);
            tc.verifyEqual(m.K2_closed_s, 1.5 * ones(1, m.n_attempts), 'AbsTol', 0.02);
            tc.verifyEqual(m.K1_ever, 0);
            tc.verifyEqual(m.final_state, 10);
            tc.verifyEqual(double(m.fault_b6), 1);
            tc.verifyLessThanOrEqual(m.E_pre_J, 200);
        end

        % ================================================================== E-stop (T-PWR-04)
        function estop_T04(tc)
            [F, ~, B] = pp_log_cols();
            t0 = 3.0;
            TB = tb_power_plant_default();
            TB.k3_t = [0 0.5 1e7]; TB.k3_v = [0 1 1];
            TB.loadL_t = [0 2 6 1e7]; TB.loadL_v = [0 50 0 0]; TB.loadR_t = TB.loadL_t; TB.loadR_v = TB.loadL_v;
            TB.estopF_t = [0 t0 6.0 1e7]; TB.estopF_v = [0 1 0 0];
            TB.reset_t = [0 7.0 7.1 1e7]; TB.reset_v = [0 1 0 0];
            % MCU keeps HV_Enable_Req through the stop-category-1 delay (K1 opens on Safety_HV_OK), drops it after
            % K1 is open and raises it again after the reset (BMS re-arm needs a falling + rising edge)
            TB.hv_t = [0 1.0 t0 + 2.0 7.5 1e7]; TB.hv_v = [0 1 0 1 1];
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 9.0);
            f = R.log_fast; t = f(:, F.t); b = R.log_bms;
            m.I_before = mean(f(t > t0 - 0.5 & t < t0, F.I_pack));
            m.dt_SOK = t_power_plant.t_first(t, f(:, F.Safety_OK) < 0.5, t0) - t0;
            m.dt_K3 = t_power_plant.t_first(t, f(:, F.K3) < 0.5, t0) - t0;
            m.dt_K1 = t_power_plant.t_first(t, f(:, F.K1) < 0.5, t0) - t0;
            m.I_after = max(f(t > t0 + m.dt_K1 + 0.1 & t < 6.0, F.I_pack));
            m.reclose_on_release = max([f(t > 6.0 & t < 7.0, F.K1); f(t > 6.0 & t < 7.0, F.Safety_OK)]);
            tH = t_power_plant.t_first(b(:, B.t), b(:, B.HV_Ready) > 0.5, 7.5);
            m.dt_HV_Ready = tH - 7.5;
            tc.log(1, sprintf(['T-PWR-04: I_pack %.0f A before; Safety_OK false after %.0f ms (<= 20), K3 open after %.0f ms (<= 30), ' ...
                'K1 open after %.3f s (1.5 +/- 0.03), then I_pack <= %.2f A (< 6), re-close on release %d, HV_Ready %.2f s ' ...
                'after reset + HV edge (<= 1.0)'], m.I_before, 1e3 * m.dt_SOK, 1e3 * m.dt_K3, m.dt_K1, m.I_after, ...
                m.reclose_on_release, m.dt_HV_Ready));
            t_power_plant.record('estop', m);
            tc.verifyGreaterThan(m.I_before, 90);
            tc.verifyLessThanOrEqual(m.dt_SOK, 0.020 + 1e-9);
            tc.verifyLessThanOrEqual(m.dt_K3, 0.030 + 1e-9);
            tc.verifyEqual(m.dt_K1, 1.5, 'AbsTol', 0.03);
            tc.verifyLessThan(m.I_after, 6);
            tc.verifyEqual(m.reclose_on_release, 0);
            tc.verifyLessThanOrEqual(m.dt_HV_Ready, 1.0);
        end

        % ================================================================== LV rails (T-PWR-05)
        function lv_steps_T05(tc)
            TB = local_lv_tb();
            TB.P12_t = [0 0.5 2.0 1e7]; TB.P12_v = [0 150 300 300];
            TB.P24_t = [0 1.0 1e7];     TB.P24_v = [0 600 600];
            TB.P5_t = [0 1.5 1e7];      TB.P5_v = [0 15 15];
            TB.P3_t = [0 1e7];          TB.P3_v = [0.6 0.6];
            R = pp_run('tb_lv_rails', TB, 'StopTime', 2.6);
            g = R.log_lv; t = g(:, 1);
            Vn = [24 12 5]; col = [4 7 9]; tStep = [1.0 0.5 1.5];
            for k = 1:3
                v = g(:, col(k));
                w = t >= tStep(k) & t < tStep(k) + 0.05;
                m.dev_transient(k) = max(abs(v(w) - Vn(k))) / Vn(k);
                kk = find(w & abs(v - Vn(k)) > 0.02 * Vn(k), 1, 'last');
                if isempty(kk), m.t_recover(k) = 0; else, m.t_recover(k) = t(kk) - tStep(k) + 1e-3; end
                ws = t > 2.4 & t < 2.6;
                m.dev_steady(k) = max(abs(v(ws) - Vn(k))) / Vn(k);
            end
            w24 = t > 1.9 & t < 2.0; w12 = t > 2.5 & t < 2.6;
            % log_lv columns 20..25 = [P_in24 P_out24 P_in12 P_out12 P_in5 P_out5]
            m.eff24 = mean(g(w24, 21)) / mean(g(w24, 20));
            m.eff12 = mean(g(w12, 23)) / mean(g(w12, 22));
            m.P24_out = mean(g(w24, 21)); m.P12_out = mean(g(w12, 23));
            v5 = g(:, 9); v33 = g(:, 10);
            m.v3v3_rule = all(v33(v5 >= 3.6) >= 3.2);
            tc.log(1, sprintf(['T-PWR-05: steady dev [24 12 5] %s %% (<= 1), transient %s %% (<= 8), recovery %s ms (<= 25), ' ...
                'eff 24 V %.2f %% at %.0f W (94 +/- 1), 12 V %.2f %% at %.0f W (93 +/- 1), V_3V3 rule %d'], ...
                mat2str(100 * m.dev_steady, 3), mat2str(100 * m.dev_transient, 3), mat2str(1e3 * m.t_recover, 3), ...
                100 * m.eff24, m.P24_out, 100 * m.eff12, m.P12_out, m.v3v3_rule));
            t_power_plant.record('lv_steps', m);
            tc.verifyLessThanOrEqual(m.dev_steady, 0.01);
            tc.verifyLessThanOrEqual(m.dev_transient, 0.08);
            tc.verifyLessThanOrEqual(m.t_recover, 0.025);
            tc.verifyEqual(m.eff24, 0.94, 'AbsTol', 0.01);
            tc.verifyEqual(m.eff12, 0.93, 'AbsTol', 0.01);
            tc.verifyTrue(m.v3v3_rule);
        end

        function lv_uvlo_rail_ok(tc)
            % AUX48 ramp 52 -> 30 V: rails off below v1_min 36 V; Rail_*_OK false below 90 % of nominal
            TB = local_lv_tb();
            TB.vaux_t = [0 0.5 2.5 1e7]; TB.vaux_v = [52 52 30 30];
            R = pp_run('tb_lv_rails', TB, 'StopTime', 3.0);
            g = R.log_lv; t = g(:, 1);
            Vn = [24 12 5 3.3]; col = [4 7 9 10]; okc = [11 12 13 14];
            consistent = true(1, 4); below_false = true(1, 4);
            for k = 1:4
                v = g(:, col(k)); ok = g(:, okc(k)) > 0.5;
                consistent(k) = all(ok == (abs(v - Vn(k)) <= 0.1 * Vn(k) + 1e-9));
                below_false(k) = ~any(ok(v < 0.9 * Vn(k)));
            end
            tOff = t_power_plant.t_first(t, g(:, 2) < 36);
            late = t > tOff + 0.1;
            m.t_vaux_36 = tOff;
            m.V24_after = max(g(late, 4)); m.V12_after = max(g(late, 7)); m.V5_after = max(g(late, 9));
            m.consistent = consistent; m.below_false = below_false;
            m.v3v3_rule = all(g(g(:, 9) >= 3.6, 10) >= 3.2);
            tc.log(1, sprintf(['UVLO: V_aux < 36 V at %.2f s; 0.1 s later max V24 %.2f, V12 %.2f, V5 %.2f V; Rail OK == within ' ...
                '+/-10 %%: %s; false below 90 %%: %s; V_3V3 rule %d'], m.t_vaux_36, m.V24_after, m.V12_after, m.V5_after, ...
                mat2str(consistent), mat2str(below_false), m.v3v3_rule));
            t_power_plant.record('lv_uvlo', m);
            tc.verifyTrue(all(below_false));
            tc.verifyTrue(all(consistent));
            tc.verifyLessThan([m.V24_after m.V12_after], [0.5 * 24 0.5 * 12]);
            tc.verifyTrue(m.v3v3_rule);
        end

        % ================================================================== implements
        function mower_T14(tc)
            % mower: 90 % of 293.2 rad/s <= 3.5 s (k_aero 1.19e-5, J 0.05); after E-stop blade < 10 % in <= 3.0 s
            [F, S] = pp_log_cols();
            TB = tb_power_plant_default();
            TB.ien_t = [0 2 1e7]; TB.ien_v = [0 1 1]; TB.ispd_t = [0 2 1e7]; TB.ispd_v = [0 2800 2800];
            TB.estopF_t = [0 7 1e7]; TB.estopF_v = [0 1 1];
            TB.J_impl = 0.05; TB.k_aero = 1.19e-5;
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 10.5);
            f = R.log_fast; t = f(:, F.t); w = f(:, F.w_impl); s = R.log_slow;
            wn = 293.2;
            m.t90 = t_power_plant.t_first(t, w >= 0.9 * wn, 2) - 2;
            m.w_steady = mean(w(t > 6.5 & t < 7));
            m.t_stop10 = t_power_plant.t_first(t, w < 0.1 * wn, 7) - 7;
            m.I_regen_min = min(f(t > 7 & t < 8.5, F.I_pack));
            m.I_imp_peak = max(abs(s(:, S.Impl_Current)));
            m.P_steady_noload = mean(s(s(:, 1) > 6.5 & s(:, 1) < 7, S.Impl_Power));
            m.ready = max(s(s(:, 1) > 6.5 & s(:, 1) < 7, S.Impl_Ready));
            tc.log(1, sprintf(['T-PWR-14: 90 %% speed after %.2f s (<= 3.5), steady %.1f rad/s, P %.0f W (windage only), ' ...
                'I_imp peak %.1f A (<= 45), after E-stop < 10 %% in %.2f s (<= 3.0), regen pack current down to %.1f A'], ...
                m.t90, m.w_steady, m.P_steady_noload, m.I_imp_peak, m.t_stop10, m.I_regen_min));
            t_power_plant.record('mower', m);
            tc.verifyLessThanOrEqual(m.t90, 3.5);
            tc.verifyLessThanOrEqual(m.t_stop10, 3.0);
            tc.verifyEqual(m.w_steady, wn, 'RelTol', 0.02);
            tc.verifyEqual(m.ready, 1);
            tc.verifyLessThanOrEqual(m.I_imp_peak, 45);
            tc.verifyLessThan(m.I_regen_min, 0, 'regenerative braking current must flow back into the pack');
        end

        function ecu_can_timeout(tc)
            % implement ECU speed reference -> 0 at 0.2 +/- 0.02 s after the IMPL_CMD frames stop
            TB = tb_power_plant_default();
            TB.ien_t = [0 2 1e7]; TB.ien_v = [0 1 1]; TB.ispd_t = [0 2 1e7]; TB.ispd_v = [0 2800 2800];
            TB.icmd_stop_t = 5.0;
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 5.6);
            e = R.ecu.ecu_w_target;
            m.t_ref0 = t_power_plant.t_first(e(:, 1), e(:, 2) == 0, 3.0);
            m.dt = m.t_ref0 - TB.icmd_stop_t;
            tc.log(1, sprintf('ECU CAN timeout: last IMPL_CMD frame at %.2f s, reference 0 at %.3f s = %.3f s after the frames stop', ...
                TB.icmd_stop_t - 0.05, m.t_ref0, m.dt));
            t_power_plant.record('ecu_timeout', m);
            tc.verifyEqual(m.dt, 0.2, 'AbsTol', 0.02);
        end

        function tiller_T15(tc)
            % tiller: rotor (Impl_Shaft, bench sensor) = motor / 12 (+/- 0.5 %); LF step 1 -> 2
            [F, S] = pp_log_cols();
            TB = tb_power_plant_default();
            TB.ien_t = [0 2 1e7]; TB.ien_v = [0 1 1]; TB.ispd_t = [0 2 1e7]; TB.ispd_v = [0 250 250];
            TB.J_impl = 0.35; TB.k_aero = 0; TB.T_c = 0.4 * 53; TB.k_lf = 0.6 * 53 / 26.18;
            TB.lf_t = [0 8 1e7]; TB.lf_v = [1 2 2];
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 2, 'VC_CHG', 1), 'StopTime', 10);
            f = R.log_fast; s = R.log_slow;
            wr = interp1(f(:, F.t), f(:, F.w_impl), s(:, S.t));
            % steady windows at LF 1 and LF 2 (the bench rotor speed passes a 1 ms Unit Delay)
            sel = ((s(:, S.t) > 6 & s(:, S.t) < 8) | (s(:, S.t) > 9.5)) & abs(s(:, S.Impl_Speed)) > 1;
            ratio_err = max(abs(wr(sel) ./ s(sel, S.Impl_Speed) - 1));
            m.ratio_err = ratio_err;
            m.w_LF1 = mean(wr(s(:, 1) > 7 & s(:, 1) < 8));
            m.P_LF1 = mean(s(s(:, 1) > 7 & s(:, 1) < 8, S.Impl_Power));
            m.dip = 1 - min(wr(s(:, 1) > 8 & s(:, 1) < 10)) / m.w_LF1;
            tc.log(1, sprintf(['T-PWR-15: rotor speed / (motor speed / 12) - 1 <= %.2e (<= 0.5 %%), rotor %.2f rad/s, P(LF 1) ' ...
                '%.2f kW (spec 8: 1.6 +/- 0.2), speed dip at LF 1 -> 2 %.1f %% (spec 8: <= 15 %%, information: bench rotor ' ...
                'J 0.35 kg m^2 + soil torque stub, spec PI gains)'], m.ratio_err, m.w_LF1, m.P_LF1 / 1e3, 100 * m.dip));
            t_power_plant.record('tiller', m);
            tc.verifyLessThanOrEqual(m.ratio_err, 0.005);
            tc.verifyEqual(m.w_LF1, 26.18, 'RelTol', 0.02);
        end

        function sprayer_T16(tc)
            % sprayer: duty 0 -> 1, all 8 nozzles: pump >= 90 % nominal within 1 s, I_24IMP <= 13 A, V24_IMP >= 23 V
            [F, S] = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            TB.k3_t = [0 0.5 1e7]; TB.k3_v = [0 1 1];
            TB.duty_t = [0 2 1e7]; TB.duty_v = [0 1 1]; TB.noz_t = [0 2 1e7]; TB.noz_v = [0 8 8];
            TB.J_impl = 1e-6; TB.k_aero = 4.9e-6; TB.p_line = 1;           % pump inertia is in the DC Motor J (spec I13)
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 3, 'VC_CHG', 1), 'StopTime', 4);
            f = R.log_fast; s = R.log_slow; t = f(:, F.t);
            wn = 314.16;
            m.t90 = t_power_plant.t_first(t, f(:, F.w_impl) >= 0.9 * wn, 2) - 2;
            m.I24imp_max = max(s(:, S.Impl_Current));
            m.V24imp_min = min(f(t > 2, F.V_24IMP));
            m.ready = s(end, S.Impl_Ready);
            % information: with the STRUCTURE pump inertia (P.structure.impl.sprayer.J) on Impl_Shaft
            TB.J_impl = 0.002;
            R2 = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 3, 'VC_CHG', 1), 'StopTime', 4);
            m.I24imp_max_structJ = max(R2.log_slow(:, S.Impl_Current));
            m.t90_structJ = t_power_plant.t_first(R2.log_fast(:, F.t), R2.log_fast(:, F.w_impl) >= 0.9 * wn, 2) - 2;
            tc.log(1, sprintf(['T-PWR-16: pump 90 %% after %.2f s (<= 1.0), I_24IMP max %.1f A (<= 13), V24_IMP min %.2f V ' ...
                '(>= 23.0), ready %d; with the structure pump inertia 0.002 kg m^2: %.2f s, %.1f A (information)'], ...
                m.t90, m.I24imp_max, m.V24imp_min, m.ready, m.t90_structJ, m.I24imp_max_structJ));
            t_power_plant.record('sprayer', m);
            tc.verifyLessThanOrEqual(m.t90, 1.0);
            tc.verifyLessThanOrEqual(m.I24imp_max, 13);
            tc.verifyGreaterThanOrEqual(m.V24imp_min, 23.0);
            tc.verifyEqual(m.ready, 1);
        end

        function dead_bus_T18(tc)
            % implement and DRIVE stubs commanded while K1 is open: no solver error, bus current <= 1 A, no torque
            [F, S] = pp_log_cols();
            TB = tb_power_plant_default(); TB.hv_v = [0 0 0];
            TB.ien_t = [0 1 1e7]; TB.ien_v = [0 1 1]; TB.ispd_t = [0 1 1e7]; TB.ispd_v = [0 2800 2800];
            TB.loadL_t = [0 1 1e7]; TB.loadL_v = [0 50 50]; TB.loadR_t = TB.loadL_t; TB.loadR_v = TB.loadL_v;
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 3);
            f = R.log_fast; s = R.log_slow;
            m.I_bus_max = max(abs(f(:, F.I_loadL)) + abs(f(:, F.I_loadR))) + max(abs(s(:, S.Impl_Current)));
            m.w_max = max(abs(f(:, F.w_impl)));
            m.V_bus_max = max(f(:, F.V_bus));
            m.K1 = max(f(:, F.K1));
            tc.log(1, sprintf('T-PWR-18: K1 %d, V_bus max %.2f V, bus current max %.3f A (<= 1), blade speed max %.4f rad/s', ...
                m.K1, m.V_bus_max, m.I_bus_max, m.w_max));
            t_power_plant.record('dead_bus', m);
            tc.verifyEqual(m.K1, 0);
            tc.verifyLessThanOrEqual(m.I_bus_max, 1);
            tc.verifyLessThan(m.w_max, 0.1);
        end

        function dead_bus_coastdown(tc)
            % mower spinning, then HV off: the DC bus decays to ~0 V through the bleed while the blade still coasts
            % (no solver error / assertion of the Motor & Drive block on a dead bus)
            F = pp_log_cols();
            TB = tb_power_plant_default();
            TB.hv_t = [0 1 6 1e7]; TB.hv_v = [0 1 0 0];
            TB.ien_t = [0 2 6 1e7]; TB.ien_v = [0 1 0 0]; TB.ispd_t = [0 2 1e7]; TB.ispd_v = [0 2800 2800];
            TB.dec_fast = 100; TB.dec_slow = 10;
            R = pp_run('tb_power_plant', TB, 'VC', struct('VC_PACK', 2, 'VC_IMPL', 1, 'VC_CHG', 1), 'StopTime', 120);
            f = R.log_fast;
            m.T_end = R.T_end;
            m.V_bus_end = f(end, F.V_bus);
            m.V_bus_max_K1open = max(f(f(:, F.t) > 6.5, F.V_bus));
            m.w_end = f(end, F.w_impl);
            m.K1_end = f(end, F.K1);
            clampBV = getfield(param_power(), 'impl', 'dc_clamp', 'BV');
            tc.log(1, sprintf(['dead-bus coast-down: K1 %d, isolated bus held <= %.1f V (ECU over-voltage logic + %.0f V DC-link ' ...
                'clamp) while braking, then V_bus %.2f V with the blade still at %.1f rad/s at %.0f s (no solver error)'], ...
                m.K1_end, m.V_bus_max_K1open, clampBV, m.V_bus_end, m.w_end, m.T_end));
            t_power_plant.record('dead_bus_coast', m);
            tc.verifyEqual(m.T_end, 120, 'AbsTol', 1e-6);
            tc.verifyLessThan(m.V_bus_end, 30, 'bus must decay into the zero-torque region');
            tc.verifyGreaterThan(m.w_end, 1);
            tc.verifyLessThanOrEqual(m.V_bus_max_K1open, clampBV + 2, 'regenerative braking must not pump the isolated bus');
        end

        % ================================================================== short circuit / overload (T-PWR-06 / 06b)
        function short_T06(tc)
            % 5 mOhm short at DC48_TracL with Ts.local = Ts.base = 1e-4 s: F_TL <= 10 ms, F_MAIN intact, b1, BMS b9 <= 0.2 s
            [F, ~, B] = pp_log_cols();
            TB = tb_power_plant_default();
            TB.hv_t = [0 0.2 1e7]; TB.short_t = [0 1.0 1e7]; TB.short_v = [0 1 1]; TB.R_short = 5e-3;
            R = pp_run('tb_power_plant', TB, 'VC', t_power_plant.VC0, 'StopTime', 1.3, 'Mode', 'fault');
            f = R.log_fast; t = f(:, F.t); b = R.log_bms;
            fb = uint8(f(:, F.Fuse_Blown));
            m.t_TL = t_power_plant.t_first(t, bitget(fb, 2) > 0, 1.0) - 1.0;
            m.main_intact = ~any(bitget(fb, 1));
            m.fuse_word = double(fb(end));
            m.I_peak = max(f(:, F.I_pack));
            m.t_b9 = t_power_plant.t_first(b(:, B.t), bitget(uint32(b(:, B.Fault_Word)), 10) > 0, 1.0) - 1.0;
            m.HV_ready_before = b(find(b(:, 1) <= 0.95, 1, 'last'), B.HV_Ready);
            tc.log(1, sprintf(['T-PWR-06: F_TL blown %.1f ms after the short (<= 10), F_MAIN intact %d, Fuse_Blown %d (b1), peak ' ...
                'pack current %.0f A, BMS Fault_Word b9 within %.2f s (<= 0.2, 0.1 s telemetry)'], 1e3 * m.t_TL, ...
                m.main_intact, m.fuse_word, m.I_peak, m.t_b9));
            t_power_plant.record('short', m);
            tc.verifyEqual(m.HV_ready_before, 1);
            tc.verifyLessThanOrEqual(m.t_TL, 0.010);
            tc.verifyTrue(m.main_intact);
            tc.verifyEqual(m.fuse_word, 2);
            tc.verifyLessThanOrEqual(m.t_b9, 0.2 + 1e-9);
        end

        function overload_T06b(tc)
            % 1.5 x F_TL rating on DC48_TracL (pp_case_def): F_TL blows within 150-300 s, F_MAIN intact (Ts.local 1e-4 s, 0.1 s log)
            F = pp_log_cols();
            R = pp_case('overload_06b');
            f = R.log_fast; t = f(:, F.t);
            fb = uint8(f(:, F.Fuse_Blown));
            m.t_TL = t_power_plant.t_first(t, bitget(fb, 2) > 0) - 2;
            m.main_intact = ~any(bitget(fb, 1));
            m.I_load = mean(f(t > 10 & t < 100, F.I_loadL));
            tc.log(1, sprintf('T-PWR-06b: %.0f A on TracL, F_TL blows after %.1f s (150..300), F_MAIN intact %d', m.I_load, m.t_TL, m.main_intact));
            t_power_plant.record('overload', m);
            tc.verifyGreaterThanOrEqual(m.t_TL, 150);
            tc.verifyLessThanOrEqual(m.t_TL, 300);
            tc.verifyTrue(m.main_intact);
        end

        % ================================================================== energy audit (D40) and shift cycle (T-PWR-20)
        function energy_audit_1h(tc)
            [~, ~, ~, T] = pp_log_cols();
            R = pp_case('audit_1h');
            tr = R.log_truth;
            num = (tr(:, T.E_out_Wh) - tr(:, T.E_in_Wh)) + tr(:, T.E_chg_bus_Wh) - (tr(:, T.E_trac_dc_Wh) + ...
                tr(:, T.E_impl_dc_Wh) + tr(:, T.E_lv_Wh) + tr(:, T.E_dcdc_loss_Wh) + tr(:, T.E_pdu_loss_Wh));
            den = max(tr(:, T.E_out_Wh), 1);
            m.eps_el_end = abs(num(end)) / den(end);
            m.eps_el_max = max(abs(num) ./ den);
            m.E = struct('out', tr(end, T.E_out_Wh), 'in', tr(end, T.E_in_Wh), 'chg_bus', tr(end, T.E_chg_bus_Wh), ...
                'trac', tr(end, T.E_trac_dc_Wh), 'impl', tr(end, T.E_impl_dc_Wh), 'lv', tr(end, T.E_lv_Wh), ...
                'dcdc', tr(end, T.E_dcdc_loss_Wh), 'pdu', tr(end, T.E_pdu_loss_Wh));
            m.rtf = R.rtf;
            tc.log(1, sprintf(['energy audit 1 h: E_out %.1f, E_in %.1f, E_chg_bus %.1f, E_trac %.1f, E_impl %.1f, E_lv %.1f, ' ...
                'E_dcdc %.1f, E_pdu %.2f Wh; eps_el %.2e at 1 h, max %.2e (<= 0.5 %%); bench %.1fx real time'], ...
                m.E.out, m.E.in, m.E.chg_bus, m.E.trac, m.E.impl, m.E.lv, m.E.dcdc, m.E.pdu, m.eps_el_end, m.eps_el_max, m.rtf));
            t_power_plant.record('audit', m);
            tc.verifyGreaterThan(R.T_end, 3600 - 1e-6);
            tc.verifyGreaterThan([m.E.trac m.E.impl m.E.lv m.E.chg_bus], 0, 'every energy path must be exercised');
            tc.verifyLessThanOrEqual(m.eps_el_end, 0.005);
            tc.verifyLessThanOrEqual(m.eps_el_max, 0.005);
        end

        function shift_cycle_T20_45C(tc)
            m = t_power_plant.eval_shift(tc, pp_case('shift_45'), 'shift_45');
            t_power_plant.verify_shift(tc, m);
        end

        function shift_cycle_T20_50C(tc)
            m = t_power_plant.eval_shift(tc, pp_case('shift_50'), 'shift_50');
            t_power_plant.verify_shift(tc, m);
        end

        function shift_bench_speed(tc)
            % T-PWR-20 bench >= 100x real time (mission mode, Lumped pack); compiler state reported (D56)
            R1 = pp_case('shift_45', 'CacheOnly', true); R2 = pp_case('shift_50', 'CacheOnly', true);
            tc.assumeFalse(isempty(R1) || isempty(R2), 'shift-cycle cases not run yet');
            cc = mex.getCompilerConfigurations('C');
            m.rtf = [R1.rtf R2.rtf]; m.compiler = ~isempty(cc);
            tc.log(1, sprintf('tb_shift_cycle: %.1fx / %.1fx real time (>= 100); C compiler for Accelerator: %d', m.rtf, m.compiler));
            t_power_plant.record('shift_speed', m);
            tc.verifyGreaterThanOrEqual(m.rtf, 100);
        end
    end

    methods (Static)
        function m = eval_shift(tc, R, name)
            [~, ~, B, T] = pp_log_cols();
            TB = tb_power_plant_default('shift');
            b = R.log_bms; tr = R.log_truth;
            sh = TB.shift;
            m.T_end = R.T_end; m.rtf = R.rtf;
            for k = 1:sh.n
                ts = sh.t_start(k); te = ts + sh.t_shift;
                w = b(:, B.t) >= ts & b(:, B.t) <= te;
                m.hv_through(k) = all(b(w, B.HV_Ready) > 0.5);
                m.soc_start(k) = interp1(tr(:, T.t), tr(:, T.SOC_pack_true), ts);
                m.soc_end(k) = interp1(tr(:, T.t), tr(:, T.SOC_pack_true), te);
                m.soc_min_group_end(k) = interp1(tr(:, T.t), tr(:, T.SOC_true_min), te);
                m.complete(k) = m.hv_through(k) && te <= R.T_end;
            end
            m.n_complete = sum(m.complete);
            m.fault_level_max = max(b(:, B.Fault_Level));
            crit = uint32(sum(2.^[0 1 2 3 4 5 6 7 8 14 15]));
            m.crit_fault = any(bitand(uint32(b(:, B.Fault_Word)), crit) > 0);
            m.T_cell_max_C = max(tr(:, T.T_cell_max_K)) - 273.15;
            m.charge_done = sum(diff([0; b(:, B.Charge_Complete) > 0.5]) == 1);
            tc.log(1, sprintf(['T-PWR-20 %s: %d / 3 shifts complete (HV through the 4 h duty), SOC_pack_true start %s end %s ' ...
                '(>= 0.215), BMS fault level max %d, critical faults %d, T_cell max %.1f C (<= 55), charge completions %d, ' ...
                '%.1fx real time'], name, m.n_complete, mat2str(m.soc_start, 3), mat2str(m.soc_end, 3), m.fault_level_max, ...
                m.crit_fault, m.T_cell_max_C, m.charge_done, m.rtf));
            t_power_plant.record(name, m);
        end

        function verify_shift(tc, m)
            tc.verifyGreaterThanOrEqual(m.n_complete, 3);
            tc.verifyGreaterThanOrEqual(m.soc_end, param_power().budget.soc_window(2));
            tc.verifyLessThan(m.fault_level_max, 3);
            tc.verifyFalse(m.crit_fault);
            tc.verifyLessThanOrEqual(m.T_cell_max_C, 55);
        end
    end
end

function TB = local_lv_tb()
TB = struct('vaux_t', [0 1e7], 'vaux_v', [52 52], 'P24_t', [0 1e7], 'P24_v', [53 53], 'P12_t', [0 1e7], 'P12_v', [99 99], ...
    'P5_t', [0 1e7], 'P5_v', [2.5 2.5], 'P3_t', [0 1e7], 'P3_v', [0.6 0.6], 'fan_t', [0 1e7], 'fan_v', [0 0], ...
    'k3_t', [0 1e7], 'k3_v', [0 0], 'imp_R', 1e3, 'MaxNonlinIter', 3, 'MaxModeIter', 2);
end
