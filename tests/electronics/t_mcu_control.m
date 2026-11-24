classdef t_mcu_control < matlab.unittest.TestCase
%T_MCU_CONTROL  Acceptance tests of build task T09 (MCU vehicle control, mode manager, safety supervisor,
%   re-arm, implement / aux, watchdog; docs/build_plan.md T09).
%
%   runtests('tests/electronics/t_mcu_control.m')
%   Unit harnesses: mcu_ctrl_harness (temporary, unsaved models: 'sup', 'ctrl', 'aux', 'wdg').
%   Closed-loop bench: tests/electronics/tb_mcu_closed_loop.slx (scripts/build/build_tb_mcu_closed_loop.m),
%   run with mcu_cl_run; the nominal scenarios run once per session in one fast-restart batch (P.rc.p_loss = 0).
%   Code generation is BLOCKED while Simulink Coder is missing (D31). The rebuild check (about 1 min) runs
%   only with the environment variable T09_LONG=1.

    properties (Constant)
        Units = {'fw_safety_supervisor', 'fw_mode_manager', 'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
        Bench = 'tb_mcu_closed_loop';
    end

    properties
        Root
        Cache          % containers.Map (handle): bench results shared by the test methods
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_McuIn1k'',''var'')')
                shaddad_params();
            end
            if ~evalin('base', 'isfield(P.electronics.safety, ''mask_rc'')')
                evalin('base', 'P.electronics = param_electronics(P);');
            end
            tc.Root = currentProject().RootFolder;
            tc.Cache = containers.Map();
            wasMcu = bdIsLoaded('Shaddad_MCU'); wasTb = bdIsLoaded(t_mcu_control.Bench);
            tc.addTeardown(@() t_mcu_control.closeIfNew(t_mcu_control.Bench, wasTb));
            tc.addTeardown(@() t_mcu_control.closeIfNew('Shaddad_MCU', wasMcu));
            tc.addTeardown(@() mcu_ctrl_harness('close'));
            tc.addTeardown(@() clear('mcu_mode_manager'));
        end
    end

    methods (Static)
        function closeIfNew(m, was)
            if ~was && bdIsLoaded(m), close_system(m, 0); end
        end

        function c = changes(t, x)
            % [t x] at the start and at every change of x
            x = double(x(:));
            k = [1; find(diff(x) ~= 0) + 1];
            c = [t(k) x(k)];
        end

        function tf = firstTime(t, cond, t0)
            % first time >= t0 where cond holds (NaN if never)
            if nargin < 3, t0 = -inf; end
            k = find(t >= t0 - 1e-9 & cond(:), 1);
            if isempty(k), tf = NaN; else, tf = t(k); end
        end

        function x = stick(y, e)
            % stick position whose deadband + expo output is y (spec 2.7.2 / ICD 7)
            P = evalin('base', 'P');
            d = P.electronics.rcmap.deadband;
            xd = fzero(@(x) e*x.^3 + (1 - e)*x - abs(y), [0 1]);
            x = sign(y)*(xd*(1 - d) + d);
        end

        function P0 = nominalP()
            P0 = evalin('base', 'P');
            P0.rc.p_loss = 0;
            P0.electronics.imu_cal = struct('roll0', 0, 'pitch0', 0);   % IMU_Emu is an ideal IMU: no level calibration
        end

        function [names, T, tabs] = scenarios()
            % nominal bench scenarios (one fast-restart batch, P.rc.p_loss = 0)
            P = evalin('base', 'P');
            thr1 = t_mcu_control.stick(1.0/P.electronics.lim.v_max_man, P.electronics.rcmap.expo_thr);   % 1.0 m/s
            arm = [1.5 1];
            auto = @() struct('op', struct('sw_mode', 3, 'sw_arm', arm), ...
                'gnc', struct('cmd_valid', [5.0 1], 'v_cmd', [5.0 0.8]));
            s = struct();
            s.manual = {8, struct('op', struct('sw_arm', arm, 'stick_thr', [2.0 0.5]))};
            rng(24);                                                   % 100 settled throttle steps, random TX phases (D37)
            ev = 4 + cumsum(1.5 + 0.3*rand(100, 1)); val = 0.5 + 0.05*mod((1:100)', 2);
            s.latency = {ev(end) + 1.5, struct('op', struct('sw_arm', arm, 'stick_thr', [2.0 0.5; round(ev, 2) val]))};
            s.pivot = {9, struct('op', struct('sw_arm', arm, 'stick_steer', [2.0 1.0; 6.0 0], 'stick_thr', [7.0 0.5]))};
            ya = auto();
            ya.gnc.omega_cmd = [8.0 0.3];
            ya.op.stick_thr = [13.0 0.5];                              % operator override in AUTO
            ya.plant = [1 0.7 0 0 1];                                  % plant yaw gain 0.7
            s.yaw_auto = {15, ya};
            ct = auto(); ct.gnc_seq_stop = 7.0;
            s.comp_timeout = {10, ct};
            s.ctrl_stop = {10, struct('op', struct('sw_arm', [arm; 6.0 0], 'stick_thr', [2.0 thr1]))};
            s.estop_rc = {11, struct('op', struct('sw_arm', arm, 'stick_thr', [2.0 1.0], 'sw_estop', [6.0 1; 9.0 0], ...
                'btn_reset', [9.5 0; 9.6 1; 9.8 0]))};
            ra = auto(); ra.flt = struct('rc_link_loss', [7.0 1]);
            s.rc_auto_req = {10, ra};
            t10 = (0:round(20/0.01))'*0.01;
            pitch = deg2rad(min(max(18 + (t10 - 3), 18), 32));         % 1 deg/s from 3 s
            s.tilt_static = {20, struct('plant', [ones(size(t10)) pitch zeros(size(t10)) ones(size(t10))])};
            s.tilt_ramp_start = {20, struct('op', struct('sw_arm', arm, 'stick_thr', [3.0 0.8; 8.0 0; 11.0 0.8; 15.0 0]), ...
                'plant', [1 deg2rad(-20) 0 1])};
            s.wdg_stall = {5, struct('flt', struct('task_stall', [3.0 2]))};
            s.fb_loss = {7, struct('op', struct('sw_arm', arm, 'stick_thr', [2.0 0.5]), ...
                'flt', struct('can1_drop_mask', [5.0 2^P.electronics.can.drop_bit.fb1_L + 2^P.electronics.can.drop_bit.fb3_L]))};
            s.slip = {8, struct('op', struct('sw_arm', arm, 'stick_thr', [2.0 thr1]), 'gnc', struct('v_ground_valid', 1), ...
                'plant', [0 1 0 0 1; 6.0 1 0 0 0.5])};
            s.estop_powerup = {5, struct('hmi', struct('estop_front_pressed', [0 1; 2.0 0], 'reset_btn', [2.5 1; 2.7 0]))};
            names = fieldnames(s)';
            T = cellfun(@(n) s.(n){1}, names);
            tabs = cellfun(@(n) s.(n){2}, names, 'UniformOutput', false);
        end
    end

    methods
        function r = bench(tc, name)
            % result of a nominal bench scenario (the whole batch runs on the first request)
            if ~isKey(tc.Cache, 'nominal')
                [names, T, tabs] = t_mcu_control.scenarios();
                t0 = tic;
                rr = mcu_cl_run(T, tabs, t_mcu_control.nominalP());
                tc.log(1, sprintf('Nominal bench batch: %d scenarios, %.0f s simulated in %.0f s', numel(names), sum(T), toc(t0)));
                m = containers.Map();
                for k = 1:numel(names), m(names{k}) = rr{k}; end
                tc.Cache('nominal') = m;
            end
            m = tc.Cache('nominal');
            r = m(name);
        end

        function [mode, y] = mmRun(~, seq)
            % run mcu_mode_manager.m directly: seq = struct array of input overrides, one per 0.01 s tick
            clear('mcu_mode_manager');
            P = evalin('base', 'P'); S = P.electronics.safety;
            d = struct('rc_ok', true, 'sw_mode', uint8(1), 'arm', false, 'arm_req', false, 'sticks_neutral', true, ...
                'override', false, 'reset_req', false, 'reset_evt', false, 'fault_word', uint32(0), ...
                'fault_cls', false(1, 3), 'comp_ok', true, 'gnc_ready', true, 'mode_req', uint8(2), 'hv_ready', true, ...
                'key_on', true, 'safety_ok', true, 'lid_closed', true, 'dock_present', false, 'standstill', true, ...
                'v_max', 0, 'brakes_ok', true, 'selftest_ok', true);
            prm_n = uint16(round([S.selftest_t S.selftest_timeout S.auto_start_warn_t S.standstill_t ...
                S.ctrlstop_timeout P.electronics.shaper.estop_tmax S.stopfail_t0 S.stopfail_brake_t S.stopfail_move_t]/0.01));
            prm_mask = uint32([S.maskA S.maskB S.mask_rc]);
            prm_flag = logical([S.rc_required_in_auto S.rearm.auto_selftest]);
            prm_s = single([P.electronics.shaper.a_ctrlstop P.electronics.shaper.a_estop S.ctrlstop_t_extra ...
                S.stopfail_dv S.stopfail_rate S.standstill_v 1/0.01]);
            n = numel(seq);
            mode = zeros(n, 1); y = struct('cmd_src', zeros(n, 1), 'drive_enable', false(n, 1), 'rearm_req', false(n, 1), ...
                'bms_req', false(n, 4), 'stop_ctl', false(n, 2));
            for k = 1:n
                in = d;
                f = fieldnames(seq(k));
                for j = 1:numel(f)
                    if ~isempty(seq(k).(f{j})), in.(f{j}) = seq(k).(f{j}); end
                end
                if isfield(seq(k), 'prm_flag') && ~isempty(seq(k).prm_flag), pf = seq(k).prm_flag; else, pf = prm_flag; end
                [m, c, de, rr, br, sc] = mcu_mode_manager(logical(in.rc_ok), uint8(in.sw_mode), logical(in.arm), ...
                    logical(in.arm_req), logical(in.sticks_neutral), logical(in.override), logical(in.reset_req), ...
                    logical(in.reset_evt), uint32(in.fault_word), logical(in.fault_cls), logical(in.comp_ok), ...
                    logical(in.gnc_ready), uint8(in.mode_req), logical(in.hv_ready), logical(in.key_on), ...
                    logical(in.safety_ok), logical(in.lid_closed), logical(in.dock_present), logical(in.standstill), ...
                    single(in.v_max), logical(in.brakes_ok), logical(in.selftest_ok), prm_n, prm_mask, pf, prm_s);
                mode(k) = double(m); y.cmd_src(k) = double(c); y.drive_enable(k) = de; y.rearm_req(k) = rr;
                y.bms_req(k, :) = br; y.stop_ctl(k, :) = sc;
            end
            clear('mcu_mode_manager');
        end
    end

    methods (Test)
        % ================================================================== files, ports, structure
        function testFilesAndDescriptions(tc)
            r = tc.Root;
            f = [cellfun(@(u) ['subsystems/electronics/' u '.slx'], tc.Units, 'UniformOutput', false), ...
                {'subsystems/electronics/mcu_mode_manager.m', 'scripts/build/build_fw_control.m', ...
                'scripts/build/build_tb_mcu_closed_loop.m', 'tests/electronics/tb_mcu_closed_loop.slx'}];
            for k = 1:numel(f)
                tc.verifyTrue(isfile(fullfile(r, f{k})), f{k});
            end
            for k = 1:numel(tc.Units)
                d = Simulink.MDLInfo(fullfile(r, 'subsystems', 'electronics', [tc.Units{k} '.slx'])).Description;
                tc.verifyTrue(startsWith(d, [tc.Units{k} ' (T09']), [tc.Units{k} ': T09 description']);
                tc.verifyFalse(contains(d, 'STUB'), [tc.Units{k} ': still a stub']);
            end
            txt = fileread(fullfile(r, 'subsystems', 'electronics', 'mcu_mode_manager.m'));
            tc.verifyTrue(contains(txt, '%#codegen'), 'mcu_mode_manager.m: %#codegen');
            tc.verifyTrue(contains(txt, 'ShaddadMode.'), 'mcu_mode_manager.m uses the ShaddadMode enum');
            tc.verifyEqual(checkcode(fullfile(r, 'subsystems', 'electronics', 'mcu_mode_manager.m'), '-struct'), ...
                struct('message', {}, 'fix', {}, 'line', {}, 'column', {}), 'checkcode messages');
        end

        function testPortsAsT08Stubs(tc)
            % final ports fixed by the T08 stubs (names, order, data types); D71 (2026-11-15, lead-approved R-01
            % interface changes): fw_mode_manager out 6 stop_ctl, fw_vehicle_ctrl in 7 limits_hold,
            % fw_safety_supervisor in 3 stop_fail_prev, in 4 vtrack_ref_prev
            ports = struct( ...
                'fw_safety_supervisor', {{{'fw10', 'mode_prev', 'stop_fail_prev', 'vtrack_ref_prev'}, {'fault_word', 'warn_word', 'fault_cls', 'k_der'}}}, ...
                'fw_mode_manager', {{{'fw10', 'fault_word', 'fault_cls'}, {'mode', 'cmd_src', 'drive_enable', 'rearm_req', 'bms_req', 'stop_ctl'}}}, ...
                'fw_vehicle_ctrl', {{{'fw10', 'mode', 'cmd_src', 'drive_enable', 'fault_word', 'k_der', 'limits_hold'}, {'drive_cmd', 'vc_tel'}}}, ...
                'fw_implement_aux', {{{'fw50'}, {'do_aux', 'pwm_duty', 'impl_enable', 'impl_fault_reset', 'impl_speed_rpm'}}}, ...
                'fw_watchdog_mgr', {{{'alive', 'fault_inject', 'fault_word', 'mode', 'rearm_req'}, {'wdg_kick', 'mcu_safety_ok', 'safety_reset_req'}}});
            types = struct('fw_safety_supervisor', {{{'Inherit: auto', 'uint8', 'boolean', 'single'}, {'uint32', 'uint32', 'boolean', 'single'}}}, ...
                'fw_mode_manager', {{{'Inherit: auto', 'uint32', 'boolean'}, {'uint8', 'uint8', 'boolean', 'boolean', 'boolean', 'boolean'}}}, ...
                'fw_vehicle_ctrl', {{{'Inherit: auto', 'uint8', 'uint8', 'boolean', 'uint32', 'single', 'boolean'}, {'Bus: Bus_DriveCmd', 'single'}}}, ...
                'fw_implement_aux', {{{'Inherit: auto'}, {'uint32', 'single', 'boolean', 'boolean', 'single'}}}, ...
                'fw_watchdog_mgr', {{{'uint8', 'uint32', 'uint32', 'uint8', 'boolean'}, {'boolean', 'boolean', 'boolean'}}});
            for s = fieldnames(ports)'
                w = bdIsLoaded(s{1}); load_system(s{1});
                ins = find_system(s{1}, 'SearchDepth', 1, 'BlockType', 'Inport');
                outs = find_system(s{1}, 'SearchDepth', 1, 'BlockType', 'Outport');
                [~, io] = sort(str2double(get_param(ins, 'Port'))); [~, oo] = sort(str2double(get_param(outs, 'Port')));
                tc.verifyEqual(get_param(ins(io), 'Name')', ports.(s{1}){1}, [s{1} ' inputs']);
                tc.verifyEqual(get_param(outs(oo), 'Name')', ports.(s{1}){2}, [s{1} ' outputs']);
                tc.verifyEqual(get_param(ins(io), 'OutDataTypeStr')', types.(s{1}){1}, [s{1} ' input types']);
                tc.verifyEqual(get_param(outs(oo), 'OutDataTypeStr')', types.(s{1}){2}, [s{1} ' output types']);
                if ~w, close_system(s{1}, 0); end
            end
        end

        function testBlocksFirst(tc)
            % one MATLAB Function block (the mode-manager FSM), no Stateflow chart, the named library blocks
            nMf = 0;
            need = struct('fw_safety_supervisor', {{'Bit to Integer Converter', 'Tapped Delay', '1-D Lookup Table'}}, ...
                'fw_vehicle_ctrl', {{'Discrete PID Controller', 'Rate Limiter Dynamic', 'Saturation Dynamic', ...
                'Discrete-Time Integrator', 'Multiport Switch', 'Relay', 'Rate Limiter', 'Counter Free-Running', 'Sqrt', 'Sign'}}, ...
                'fw_implement_aux', {{'1-D Lookup Table', 'Counter Limited', 'Bit to Integer Converter'}}, ...
                'fw_watchdog_mgr', {{'Detect Change', 'Detect Rise Positive', 'Discrete-Time Integrator'}});
            for k = 1:numel(tc.Units)
                u = tc.Units{k};
                w = bdIsLoaded(u); load_system(u);
                b = find_system(u, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
                sfb = find_system(u, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SFBlockType', 'MATLAB Function');
                charts = find_system(u, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SFBlockType', 'Chart');
                nMf = nMf + numel(sfb);
                tc.verifyEmpty(charts, [u ': Stateflow chart']);
                if isfield(need, u)
                    rb = get_param(b, 'ReferenceBlock'); rb = rb(~cellfun(@isempty, rb));
                    nm = strrep(regexprep(rb, '^.*/', ''), newline, ' ');
                    bt = get_param(b, 'BlockType');
                    builtin = containers.Map({'Discrete-Time Integrator', '1-D Lookup Table', 'Multiport Switch', 'Sign', ...
                        'Rate Limiter', 'Relay', 'Sqrt'}, {'DiscreteIntegrator', 'Lookup_n-D', 'MultiPortSwitch', 'Signum', ...
                        'RateLimiter', 'Relay', 'Sqrt'});
                    for j = 1:numel(need.(u))
                        want = need.(u){j};
                        hit = any(strcmp(nm, want));
                        if strcmp(want, 'Discrete PID Controller')     % library link slpidlib/PID Controller, discrete time
                            pid = b(strcmp(get_param(b, 'ReferenceBlock'), 'slpidlib/PID Controller'));
                            hit = hit || (~isempty(pid) && strcmp(get_param(pid{1}, 'TimeDomain'), 'Discrete-time'));
                        end
                        if isKey(builtin, want), hit = hit || any(strcmp(bt, builtin(want))); end
                        tc.verifyTrue(hit, sprintf('%s uses %s', u, want));
                    end
                end
                if ~w, close_system(u, 0); end
            end
            tc.verifyEqual(nMf, 1, 'exactly one MATLAB Function block (fw_mode_manager/Mode_FSM)');
            load_system('fw_mode_manager');
            cfg = get_param('fw_mode_manager/Mode_FSM', 'MATLABFunctionConfiguration');
            tc.verifyTrue(contains(cfg.FunctionScript, 'mcu_mode_manager('), 'Mode_FSM calls mcu_mode_manager.m');
        end

        function testShaddadMcuUpdate(tc)
            % Shaddad_MCU with the T09 units: update, no algebraic loop, ICD ports, allowed compiled rates
            m = 'Shaddad_MCU';
            load_system(m);
            msg = '';
            for attempt = 1:2                      % R2026b JIT relink assertion on a first compile (lesson)
                try
                    set_param(m, 'SimulationCommand', 'update'); msg = ''; break
                catch e
                    msg = getReport(e, 'extended', 'hyperlinks', 'off');
                end
            end
            tc.assertEmpty(msg, msg);
            tc.verifyEqual(numel(Simulink.BlockDiagram.getAlgebraicLoops(m)), 0, 'Shaddad_MCU algebraic loops');
            tc.verifyEqual(shaddad_check_interfaces('Files', {'Shaddad_MCU'}, 'Quiet', true), 0, 'Shaddad_MCU ICD ports');
            P = evalin('base', 'P');
            feval(m, [], [], [], 'compile');
            c = onCleanup(@() feval(m, [], [], [], 'term'));
            bad = {};
            for u = {'Safety_Supervisor', 'Mode_Manager', 'Vehicle_Ctrl'}
                bad = [bad, t_mcu_control.badRates([m '/Task_10/' u{1}], P)]; %#ok<AGROW>
            end
            bad = [bad, t_mcu_control.badRates([m '/Task_50/Implement_Aux'], P), t_mcu_control.badRates([m '/Task_1k/Watchdog_Mgr'], P)];
            tc.verifyEmpty(bad, strjoin(bad, newline));
            clear c
        end

        function testBenchLoops(tc)
            m = t_mcu_control.Bench;
            load_system(m);
            set_param(m, 'SimulationCommand', 'update');
            tc.verifyEqual(numel(Simulink.BlockDiagram.getAlgebraicLoops(m)), 0, 'tb_mcu_closed_loop algebraic loops');
            tc.verifyEqual(get_param([m '/MCU'], 'ModelName'), 'Shaddad_MCU');
            tc.verifyEqual(get_param([m '/Plant/Kinematics'], 'VehicleInputs'), 'Wheel Speeds');
        end

        function testRebuildIdempotent(tc)
            tc.assumeTrue(strcmp(getenv('T09_LONG'), '1'), 'Long check (about 1 min): set T09_LONG=1');
            before = cellfun(@(u) t_mcu_control.signature(u), tc.Units, 'UniformOutput', false);
            build_fw_control();
            after = cellfun(@(u) t_mcu_control.signature(u), tc.Units, 'UniformOutput', false);
            for k = 1:numel(tc.Units)
                tc.verifyEqual(after{k}, before{k}, [tc.Units{k} ': rebuild changed the block set']);
            end
        end

        % ================================================================== vehicle control (unit harness)
        function testTEL09_Mixer(tc)
            P = evalin('base', 'P');
            h = P.electronics.mix.chi*P.electronics.mix.B/2;
            tc.verifyEqual(P.electronics.mix.chi, P.structure.track.chi_nom, 'chi from structure (D39)');
            tc.verifyEqual(h, P.structure.track.chi_nom*P.structure.geom.B/2, 'AbsTol', 1e-12);
            u1 = struct('rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0], 'rc_v_rc', [2.0 0.6], 'rc_w_rc', [2.0 0.5], ...
                'tilt_yaw_rate_meas', [2.0 0.5]);
            u2 = u1; u2.rc_v_rc = [2.0 2.0]; u2.rc_w_rc = [2.0 1.0]; u2.tilt_yaw_rate_meas = [2.0 0.5];
            rr = mcu_ctrl_harness('sim', 'ctrl', 6, {u1, u2});
            for j = 1:2
                v = rr{j}.vc_tel; hs = double(single(h));
                vL = v(:, 3) - v(:, 7)*hs; vR = v(:, 3) + v(:, 7)*hs;
                k = max(1, max(abs([vL vR]), [], 2)/P.electronics.lim.v_track_max);
                tc.verifyEqual(v(:, 8), vL./k, 'AbsTol', 1e-6, 'v_L = (v_ref - w_mix*chi*B/2)/k');
                tc.verifyEqual(v(:, 9), vR./k, 'AbsTol', 1e-6, 'v_R = (v_ref + w_mix*chi*B/2)/k');
                tc.verifyLessThanOrEqual(max(abs(v(:, 8:9)), [], 'all'), P.electronics.lim.v_track_max + 1e-6);
                tc.log(1, sprintf('T-EL-09 case %d: chi*B/2 = %.4f (chi_nom %.4f), max |v_track_ref| %.3f, max k %.3f', ...
                    j, h, P.electronics.mix.chi, max(abs(v(:, 8:9)), [], 'all'), max(k)));
            end
            v = rr{2}.vc_tel(end, :);
            tc.verifyEqual((v(9) - v(8))/(v(9) + v(8)), (2*v(7)*h)/(2*v(3)), 'RelTol', 1e-5, 'desaturation keeps the curvature');
            tc.verifyEqual(rr{2}.drive_cmd.L.w_ref(end), v(8)*P.electronics.drv.k_v2w, 'RelTol', 1e-5, 'w_ref = k_v2w*v_track_ref');
        end

        function testTEL10_Shaper(tc)
            P = evalin('base', 'P'); SH = P.electronics.shaper;
            arm = struct('rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0]);
            u1 = arm; u1.rc_v_rc = [2.0 1.0];                              % MANUAL speed step
            u2 = arm; u2.rc_w_rc = [2.0 0.5]; u2.rc_v_rc = [2.0 1.0];       % MANUAL yaw step (lateral limit at 1 m/s: 1 rad/s)
            rr = mcu_ctrl_harness('sim', 'ctrl', 6, {u1, u2});
            r = rr{1}; t = r.t; k = t >= 2.0;
            v = r.vc_tel(:, 3); a = r.vc_tel(:, 5); j = [0; diff(a)]/0.01;
            t98 = t_mcu_control.firstTime(t, v >= 0.98*1.0, 2.0) - 2.0;
            tc.log(1, sprintf('T-EL-10: a_max %.4f m/s^2, jerk_max %.4f m/s^3, 98 %% at %.2f s, a_min %.4f', ...
                max(a(k)), max(abs(j(k))), t98, min(a(k))));
            tc.verifyLessThanOrEqual(max(a(k)), SH.a_acc_man + 1e-3);
            tc.verifyLessThanOrEqual(max(abs(j(k))), SH.j_max + 1e-2);
            tc.verifyLessThanOrEqual(t98, 1.9);
            tc.verifyGreaterThanOrEqual(min(v(k)), -1e-6); tc.verifyLessThanOrEqual(max(v), 1.0 + 1e-4, 'no overshoot');
            r = rr{2}; w = r.vc_tel(:, 4); al = r.vc_tel(:, 6); jw = [0; diff(al)]/0.01;
            tc.verifyLessThanOrEqual(max(abs(al)), SH.alpha_max + 1e-3, 'yaw acceleration');
            tc.verifyLessThanOrEqual(max(abs(jw)), SH.jw_max + 1e-2, 'yaw jerk');
            tc.verifyEqual(w(end), 0.5, 'AbsTol', 1e-3);
        end

        function testTEL11_TorqueLimiter(tc)
            P = evalin('base', 'P'); D = P.electronics.drv; TL = P.electronics.tlim; S = P.electronics.safety;
            d = mcu_ctrl_harness('defaults', 'ctrl');
            fb = d.drive_fb; fb.L.w_m = 300; fb.R.w_m = -300;
            Vb = P.power.pack.V_nom; Dw = 0.8*P.electronics.safety.dcl_warn;   % 13S pack nominal; a DCL below dcl_warn
            bms = d.bms; bms.V_pack = Vb; bms.DCL = 100; bms.CCL = 100;
            u = struct('rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0], 'drive_fb', fb, 'bms', bms);
            bms2 = bms; bms2.DCL = Dw; u2 = u; u2.bms = bms2;              % BMS_WARN (DCL < dcl_warn): info only, no derate (D65)
            fb3 = fb; fb3.L.w_m = 0; fb3.R.w_m = 0; u3 = u; u3.drive_fb = fb3;
            bms4 = bms; bms4.CCL = 0; u4 = u; u4.bms = bms4;                % CCL 0: regen limited, traction not (D2)
            tI = 2.0; u5 = u; u5.tilt_imu_valid = [tI 0];                  % IMU invalid after the arm edge: IMU_FAULT (bit 24, C)
            rr = mcu_ctrl_harness('sim', 'ctrl', 3, {u, u2, u3, u4, u5});
            r = rr{1};
            Pdis = TL.dis_margin*Vb*100 - TL.P_aux_est; Pchg = TL.dis_margin*Vb*100 + TL.P_aux_est;
            mot = min([D.T_peak, Pdis/2*D.eta_drv/300]); reg = min([D.T_peak, Pchg/2/(D.eta_drv*300)]);
            tc.log(1, sprintf('T-EL-11: tau_lim_mot L %.4f R %.4f (expected %.4f +/- 1 %%), tau_lim_reg %.4f (%.4f)', ...
                r.drive_cmd.L.tau_lim_mot(end), r.drive_cmd.R.tau_lim_mot(end), mot, r.drive_cmd.L.tau_lim_reg(end), reg));
            tc.verifyEqual(r.drive_cmd.R.tau_lim_mot(end), mot, 'RelTol', 0.01);
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(end), mot, 'RelTol', 1e-5);
            tc.verifyEqual(r.drive_cmd.L.tau_lim_reg(end), reg, 'RelTol', 1e-5);
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(r.t < 1.5), zeros(sum(r.t < 1.5), 1), 'limits 0 while disabled');
            r = rr{2};                                                     % D65: expect 1 itself, so a return of the 0.5 fails
            Pdis2 = TL.dis_margin*Vb*Dw - TL.P_aux_est; mot2 = min(D.T_peak, Pdis2/2*D.eta_drv/300);
            tc.log(1, sprintf('T-EL-11 BMS_WARN: k_der %.4f, tau_lim_mot L %.4f (expected %.4f)', ...
                r.k_der(end), r.drive_cmd.L.tau_lim_mot(end), mot2));
            tc.verifyTrue(bitand(double(r.fault_word(end)), 2^11) ~= 0, 'BMS_WARN (bit 11) set by DCL < dcl_warn');
            tc.verifyEqual(logical(r.fault_cls(end, :)), false(1, 3), 'bit 11 is info only: no class A/B/C (D65)');
            tc.verifyEqual(double(r.k_der(end)), 1, 'AbsTol', 1e-6, 'BMS_WARN does not derate (D65)');
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(end), mot2, 'RelTol', 1e-5, 'the DCL limit alone, no BMS_WARN factor (D65)');
            r = rr{3};
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(end), D.T_peak, 'RelTol', 1e-6, 'w* = w_min at standstill -> T_peak');
            r = rr{4};
            Pchg4 = TL.dis_margin*Vb*0 + TL.P_aux_est;
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(end), mot, 'RelTol', 1e-5, 'CCL 0 leaves the motoring limit (D2)');
            tc.verifyEqual(r.drive_cmd.L.tau_lim_reg(end), min(D.T_peak, Pchg4/2/(D.eta_drv*300)), 'RelTol', 1e-5, ...
                'CCL 0 limits regen to T_regen (D2)');
            r = rr{5};                                                     % a class-C derate that still exists reaches the drives
            tD = t_mcu_control.firstTime(r.t, r.k_der < 1);
            tc.log(1, sprintf('T-EL-11 IMU_FAULT: k_der %.4f from %.2f s, tau_lim_mot L %.4f (expected %.4f)', ...
                r.k_der(end), tD, r.drive_cmd.L.tau_lim_mot(end), S.derate_imu*mot));
            tc.verifyTrue(all(double(r.mode(r.t >= tI)) == double(ShaddadMode.MANUAL)), 'class C does not stop: MANUAL stays');
            tc.verifyTrue(bitand(double(r.fault_word(end)), 2^24) ~= 0, 'IMU_FAULT (bit 24) set');
            tc.verifyEqual(logical(r.fault_cls(end, :)), [false false true], 'IMU_FAULT is class C');
            tc.verifyEqual(tD, tI + S.t_persist(1 + 24), 'AbsTol', 0.01 + 1e-9, 'k_der drops once bit 24 persists (0.5 s)');
            tc.verifyEqual(double(r.k_der(end)), S.derate_imu, 'AbsTol', 1e-6, 'k_der = derate_imu');
            tc.verifyEqual(r.drive_cmd.L.tau_lim_mot(end), S.derate_imu*mot, 'RelTol', 1e-5, ...
                'tau_lim_mot = derate_imu*min(T_peak, T_pow) at the case 1 point');
        end

        % ================================================================== safety supervisor (unit harness)
        function testTEL13_FaultBits(tc)
            P = evalin('base', 'P'); S = P.electronics.safety; di = P.electronics.di;
            d = mcu_ctrl_harness('defaults', 'sup');
            t0 = 1.0; t1 = 7.0; tr = 7.5; T = 8.5;
            dw = d.di_word;
            u = cell(1, 32); expBits = cell(1, 32); tExp = zeros(1, 32);
            for b = 0:31
                x = struct('mode_prev', 2, 'reset_req', [0 0; tr 1; tr + 0.05 0]);
                tp = S.t_persist(b + 1); extra = [];
                switch b
                    case 0,  x.di_word = [0 dw; t0 dw - 2^di.estop_ch1_ok - 2^di.estop_ch2_ok; t1 dw];
                    case 1,  x.rc_estop_remote = [t0 1; t1 0];
                    case 2,  x.rc_rc_link_lost = [t0 1; t1 0];
                    case 3,  x.rc_failsafe = [t0 1; t1 0];
                    case 4
                        x.mode_prev = 6;                                    % AUTO
                        g = d.gnc_cmd; tt = [(0:0.05:t0 - 0.05)'; (t1:0.05:T)'];  % frozen between t0 - 0.05 and t1
                        g.seq = [tt mod((1:numel(tt))', 256)];
                        x.gnc_cmd = g; tp = S.comp_timeout - 0.05;           % seq last changed at t0 - 0.05
                    case {5, 6}, a = [0 0]; a(b - 4) = 0.1; x.drv_age = [t0 a; t1 0 0];
                    case 7,  f = d.drive_fb; f.L.state = [t0 S.esc_fault_state; t1 2]; x.drive_fb = f;
                    case 8,  f = d.drive_fb; f.R.fault_code = [t0 1; t1 0]; x.drive_fb = f;
                    case 9,  x.bms_age = [t0 1.0; t1 0];
                    case 10, q = d.bms; q.fault_level = [t0 3; t1 0]; x.bms = q;
                    case 11, q = d.bms; q.warn_word = [t0 1; t1 0]; x.bms = q;
                    case {12, 13}, o = d.odom; vv = [0 0]; vv(b - 11) = 1.0; o.vTrack_LR = [t0 vv; t1 0 0]; x.odom = o;
                    case 14, x.tilt_pitch = [t0 S.tilt_warn + deg2rad(1); t1 0];
                    case 15, x.tilt_pitch = [t0 S.tilt_stop + deg2rad(1); t1 0]; extra = 14;
                    case 16
                        vo = 1.05*P.electronics.lim.v_overspeed;
                        o = d.odom; o.vTrack_LR = [t0 vo vo; t1 0 0];
                        o.wMotor_LR = [t0 vo*P.electronics.drv.k_v2w*[1 1]; t1 0 0]; x.odom = o;
                    case 17, x.mode_prev = 3; q = d.bms; q.hv_ready = [t0 0; t1 1]; x.bms = q;
                    case 18, q = d.bms; q.hv_opening_warn = [t0 1; t1 0]; x.bms = q;
                    case 19, x.adc_V24 = [t0 20; t1 24];
                    case 20, x.adc_T_encl = [t0 345; t1 318.15];
                    case 21, x.adc_T_encl = [t0 355; t1 318.15]; extra = 20;
                    case 22, x.stop_fail_prev = [t0 1; t1 0];          % STOP_FAIL (D71; WDG_RESET_SEEN before 2026-11-15)
                    case 23, x.overrun = [t0 1; t1 0]; tp = (S.overrun_n - 1)*0.01;
                    case 24, x.tilt_imu_valid = [t0 0; t1 1];
                    case {25, 26}, bb = [0 0]; bb(b - 24) = 1; x.busoff = [t0 bb; t1 0 0];
                    case 27, x.di_word = [0 dw; t0 dw - 2^di.estop_ch2_ok; t1 dw]; extra = 0;
                    case 28, x.di_word = dw + 2^di.impl_id0; x.impl_rx_ok = [0 1; t0 0; t1 1];
                    case 29, x.di_word = [0 dw; t0 dw + 2^di.bumper_front; t1 dw];
                    case 30, x.di_word = [0 dw; t0 dw - 2^di.lid_closed; t1 dw];
                    case 31, x.mode_prev = 3; x.di_word = [0 dw; t0 dw - 2^di.safety_ok; t1 dw];
                end
                u{b + 1} = x; expBits{b + 1} = [b extra]; tExp(b + 1) = t0 + tp;
            end
            rr = mcu_ctrl_harness('sim', 'sup', T, u);
            latched = bitget(double(S.latch_mask), 1:32) ~= 0;
            cls = [bitget(double(S.maskA), 1:32); bitget(double(S.maskB), 1:32); bitget(double(S.maskC), 1:32)] ~= 0;
            rep = {};
            for b = 0:31
                r = rr{b + 1}; t = r.t; fw = double(r.fault_word);
                on_b = bitand(fw, 2^b) ~= 0;
                tset = t_mcu_control.firstTime(t, on_b, 0);
                tc.verifyEqual(tset, tExp(b + 1), 'AbsTol', 0.01 + 1e-9, sprintf('bit %d set time', b));
                % only the expected bits while active (before the condition ends)
                kA = find(t >= tExp(b + 1) + 0.6 & t < t1, 1);
                if b == 20 || b == 21, kA = find(t >= tExp(b + 1) + 0.2 & t < t1, 1); end
                if ~isempty(kA)
                    expw = sum(2.^expBits{b + 1});
                    tc.verifyEqual(fw(kA), expw, sprintf('bit %d: fault word while active', b));
                    ec = any(cls(:, 1 + expBits{b + 1}), 2)';
                    tc.verifyEqual(logical(r.fault_cls(kA, :)), ec, sprintf('bit %d: class flags [A B C]', b));
                end
                % latched bits stay after the condition ends until reset_req, others clear
                kAfter = find(t >= t1 + 0.1 & t < tr, 1); kReset = find(t >= tr + 0.1, 1);
                tc.verifyEqual(on_b(kAfter), latched(b + 1), sprintf('bit %d latch behaviour', b));
                tc.verifyFalse(on_b(kReset), sprintf('bit %d cleared after reset_req', b));
                rep{end + 1} = sprintf('%d:%.2f', b, tset - t0); %#ok<AGROW>
            end
            tc.log(1, ['T-EL-13 bit: set delay [s] = ' strjoin(rep, ' ')]);
            % bit 17 HV_LOST at 0.2 s; STO fault code (bit 10) does not make DRV_FAULT
            tc.verifyEqual(t_mcu_control.firstTime(rr{18}.t, bitand(double(rr{18}.fault_word), 2^17) ~= 0) - t0, 0.2, 'AbsTol', 0.01 + 1e-9);
            f = d.drive_fb; f.L.fault_code = [t0 2^10]; f.L.state = [t0 8];
            r = mcu_ctrl_harness('sim', 'sup', 2, struct('drive_fb', f));
            tc.verifyEqual(max(double(r.fault_word)), 0, 'ESC STO (state 8, fault bit 10) is not DRV_FAULT');
        end

        function testTiltGateAndDerate(tc)
            % D47: the tilt persistence is held while tilt_gate is false; k_der = min of the active derates
            P = evalin('base', 'P'); S = P.electronics.safety;
            u1 = struct('tilt_pitch', [1.0 S.tilt_warn + deg2rad(1)], 'tilt_tilt_gate', [0 1; 1.2 0; 1.5 1]);   % 0.3 s gated
            u2 = struct('tilt_pitch', [1.0 S.tilt_warn + deg2rad(1)], 'tilt_imu_valid', [1.0 0], 'adc_T_encl', [0 345]);
            rr = mcu_ctrl_harness('sim', 'sup', 8, {u1, u2});
            t14 = t_mcu_control.firstTime(rr{1}.t, bitand(double(rr{1}.fault_word), 2^14) ~= 0);
            tc.verifyEqual(t14, 1.0 + S.t_tilt + 0.3, 'AbsTol', 0.01 + 1e-9, 'timer held while the gate is closed');
            tc.verifyEqual(double(rr{1}.k_der(end)), S.derate_tilt, 'AbsTol', 1e-6);
            % IMU invalid (derate 0.25) + enclosure warm (thermal LUT 0.5 at 70 degC + bit 20 after 5 s)
            r = rr{2};
            tc.verifyEqual(double(r.k_der(find(r.t >= 0.5, 1))), interp1(P.electronics.aux.derate_bp, P.electronics.aux.derate_k, 345), 'AbsTol', 1e-5);
            tc.verifyEqual(double(r.k_der(end)), S.derate_imu, 'AbsTol', 1e-6);
            tc.verifyFalse(any(bitand(double(r.fault_word), 2^14) ~= 0), 'no tilt fault with the IMU invalid');
        end

        % ================================================================== mode manager
        function testTEL14_Transitions(tc)
            P = evalin('base', 'P'); S = P.electronics.safety;
            B = @(b) uint32(sum(2.^b));
            M = @(s) double(ShaddadMode.(s));
            seq = repmat(struct('arm', [], 'arm_req', [], 'sw_mode', [], 'fault_word', [], 'fault_cls', [], 'reset_req', [], ...
                'standstill', [], 'override', [], 'rc_ok', [], 'comp_ok', [], 'mode_req', [], 'reset_evt', [], ...
                'selftest_ok', [], 'prm_flag', []), 1, 3000);
            put = @(s, k, f, v) local_set(s, k, f, v);
            % power-up: BOOT 1 tick -> SELFTEST (rearm) -> STANDBY after selftest_t
            % 0..110: still; 120 arm edge in MANUAL position -> MANUAL; 200 sw 2 -> HOLD; 250 sw 1 -> MANUAL
            seq = put(seq, 120:2999, 'arm', true); seq = put(seq, 120, 'arm_req', true);
            seq = put(seq, 200:249, 'sw_mode', uint8(2));
            % 300 sw 3 -> AUTO_START; 600 -> AUTO (3 s); 650 override -> MANUAL; 700 sw 2 -> HOLD; 720 sw 3 -> AUTO_START
            seq = put(seq, 300:649, 'sw_mode', uint8(3)); seq = put(seq, 650, 'override', true);
            seq = put(seq, 700:719, 'sw_mode', uint8(2)); seq = put(seq, 720:799, 'sw_mode', uint8(3));
            % 760 comp lost -> CTRL_STOP; standstill 0.5 s -> STANDBY
            seq = put(seq, 760:779, 'comp_ok', false);
            seq = put(seq, 800:2999, 'sw_mode', uint8(1));
            % 900 arm edge -> MANUAL; 950 class B (RC_LINK_LOST) -> CTRL_STOP -> STANDBY
            seq = put(seq, 900, 'arm_req', true); seq = put(seq, 950:960, 'fault_word', B(2));
            % 1100 arm edge -> MANUAL; 1150 remote E-stop -> ESTOP (DECEL, moving) -> SAFE; 1300 reset -> STANDBY
            seq = put(seq, 1100, 'arm_req', true); seq = put(seq, 1150:1160, 'standstill', false);
            seq = put(seq, 1150:1250, 'fault_word', B(1)); seq = put(seq, 1300, 'reset_req', true);
            % 1400 arm -> MANUAL; 1450 class A (DRV_L_TIMEOUT) -> FAULT; 1600 reset (cleared) -> STANDBY
            seq = put(seq, 1400, 'arm_req', true); seq = put(seq, 1450:1550, 'fault_word', B(5));
            seq = put(seq, 1450:1550, 'fault_cls', [true false false]); seq = put(seq, 1600, 'reset_req', true);
            % 1700 HW E-stop in STANDBY -> ESTOP (SAFE); 1800 reset -> STANDBY
            seq = put(seq, 1700:1750, 'fault_word', B(0)); seq = put(seq, 1700:1750, 'fault_cls', [true false false]);
            seq = put(seq, 1800, 'reset_req', true);
            % 1900 arm in AUTO position -> AUTO_START; 1950 sw 1 -> MANUAL; 2000 arm off -> CTRL_STOP
            seq = put(seq, 1900:1949, 'sw_mode', uint8(3)); seq = put(seq, 1900, 'arm_req', true);
            seq = put(seq, 2000:2099, 'arm', false);
            % 2100 arm -> MANUAL (sw 1), 2150 sw 3 -> AUTO_START, 2450 AUTO; 2500 GNC FAULT_STOP -> CTRL_STOP
            seq = put(seq, 2100, 'arm_req', true); seq = put(seq, 2150:2999, 'sw_mode', uint8(3));
            seq = put(seq, 2500:2510, 'mode_req', uint8(GncModeReq.FAULT_STOP));
            % 2700 watchdog reset event -> BOOT -> SELFTEST
            seq = put(seq, 2700, 'reset_evt', true);
            [mode, y] = tc.mmRun(seq);
            c = t_mcu_control.changes((0:numel(mode) - 1)'*0.01, mode);
            tc.log(1, ['T-EL-14 mode sequence (t, mode): ' mat2str(c', 4)]);
            want = [M('BOOT') M('SELFTEST') M('STANDBY') M('MANUAL') M('HOLD') M('MANUAL') M('AUTO_START') M('AUTO') ...
                M('MANUAL') M('HOLD') M('AUTO_START') M('CTRL_STOP') M('STANDBY') M('MANUAL') M('CTRL_STOP') M('STANDBY') ...
                M('MANUAL') M('ESTOP') M('STANDBY') M('MANUAL') M('FAULT') M('STANDBY') M('ESTOP') M('STANDBY') ...
                M('AUTO_START') M('MANUAL') M('CTRL_STOP') M('STANDBY') M('MANUAL') M('AUTO_START') M('AUTO') ...
                M('CTRL_STOP') M('STANDBY') M('BOOT') M('SELFTEST') M('STANDBY')];
            tc.verifyEqual(c(:, 2)', want, 'mode sequence');
            tk = @(s, after) c(find(c(:, 2) == M(s) & c(:, 1) >= after, 1), 1);
            tc.verifyEqual(tk('SELFTEST', 0), 0.01, 'AbsTol', 1e-9, 'BOOT -> SELFTEST after one tick');
            tc.verifyEqual(tk('STANDBY', 0), 0.01 + S.selftest_t, 'AbsTol', 1e-9, 'SELFTEST -> STANDBY after selftest_t');
            tc.verifyEqual(tk('AUTO', 0) - tk('AUTO_START', 0), S.auto_start_warn_t, 'AbsTol', 1e-9, 'AUTO_START 3 s');
            tc.verifyTrue(y.rearm_req(2), 'rearm_req on SELFTEST entry');
            tc.verifyEqual(find(y.rearm_req)', [2 1300 1600 1800 2701], 'rearm_req: SELFTEST entries and reset_req edges');
            % ESTOP from a moving MANUAL keeps the drive enabled (ESTOP.DECEL) until standstill
            kE = round(tk('ESTOP', 11)/0.01) + 1;
            tc.verifyTrue(all(y.drive_enable(kE:1160)), 'ESTOP.DECEL keeps the drive enabled');
            tc.verifyFalse(any(y.drive_enable(1162:1299)), 'ESTOP.SAFE disables the drive');
            tc.verifyFalse(any(y.drive_enable(1701:1799)), 'HW E-stop -> ESTOP.SAFE');
            % cmd_src
            act = ismember(mode, [M('MANUAL') M('AUTO')]);
            tc.verifyEqual(y.cmd_src(mode == M('MANUAL')), ones(sum(mode == M('MANUAL')), 1));
            tc.verifyEqual(y.cmd_src(mode == M('AUTO')), 2*ones(sum(mode == M('AUTO')), 1));
            tc.verifyEqual(y.cmd_src(~act), zeros(sum(~act), 1));
            % BMS requests: HV enable outside BOOT / FAULT and without class A
            tc.verifyFalse(y.bms_req(1, 1)); tc.verifyTrue(y.bms_req(200, 1));
            tc.verifyFalse(any(y.bms_req(mode == M('FAULT'), 1)));
            rst = arrayfun(@(s) ~isempty(s.reset_req) && s.reset_req, seq)';
            tc.verifyEqual(y.bms_req(:, 3), rst, 'Fault_Reset_Req = reset_req');
            tc.verifyFalse(any(y.bms_req(:, 2)) || any(y.bms_req(:, 4)), 'no charge / shutdown request (not docked, key on)');
        end

        function testTEL14_Guards(tc)
            % entry guards and the remaining transitions (direct FSM calls, 0.01 s ticks)
            M = @(s) double(ShaddadMode.(s));
            B = @(b) uint32(sum(2.^b));
            P = evalin('base', 'P'); S = P.electronics.safety;
            % (a) SELFTEST timeout -> FAULT, reset -> STANDBY
            seq = t_mcu_control.seqOf(260, {1:260, 'selftest_ok', false; 240, 'reset_req', true});
            mode = tc.mmRun(seq);
            tc.verifyEqual(t_mcu_control.changes((0:259)'*0.01, mode)', ...
                [0 0.01 0.01 + S.selftest_timeout 2.39; M('BOOT') M('SELFTEST') M('FAULT') M('STANDBY')], 'AbsTol', 1e-9);
            % (b) arming guards in STANDBY: each blocked arm edge leaves STANDBY, the last one arms
            g = {'sticks_neutral', false; 'hv_ready', false; 'safety_ok', false; 'fault_word', B(2); 'rc_ok', false};
            spec = {120:500, 'arm', true};
            for k = 1:size(g, 1)
                t = 110 + 30*k;
                spec(end + 1, :) = {t, 'arm_req', true}; %#ok<AGROW>
                spec(end + 1, :) = {t, g{k, 1}, g{k, 2}}; %#ok<AGROW>
            end
            % AUTO position: lid open, GNC not ready, companion stale, then HOLD position (not armable)
            ga = {'lid_closed', false; 'gnc_ready', false; 'comp_ok', false};
            for k = 1:size(ga, 1)
                t = 300 + 20*k;
                spec(end + 1, :) = {t, 'arm_req', true}; spec(end + 1, :) = {t, ga{k, 1}, ga{k, 2}}; %#ok<AGROW>
            end
            spec = [spec; {300:390, 'sw_mode', uint8(3); 395:399, 'sw_mode', uint8(2); 397, 'arm_req', true; 450, 'arm_req', true}];
            [mode, y] = tc.mmRun(t_mcu_control.seqOf(500, spec));
            tc.verifyTrue(all(mode(103:449) == M('STANDBY')), 'blocked arm edges keep STANDBY');
            tc.verifyEqual(mode(450), M('MANUAL'), 'arm edge with every guard met');
            tc.verifyFalse(any(y.drive_enable(1:449)));
            % (c) AUTO -> HOLD (sw 2) -> AUTO_START -> AUTO; RC loss -> CTRL_STOP (required) / ignored (not required)
            base = {120:900, 'arm', true; 120, 'arm_req', true; 120:900, 'sw_mode', uint8(3); 450:459, 'sw_mode', uint8(2)};
            loss = {800:900, 'rc_ok', false; 800:900, 'fault_word', B([2 3]); 800:900, 'arm', false; 800:900, 'sw_mode', uint8(2)};
            [m1] = tc.mmRun(t_mcu_control.seqOf(900, [base; loss]));
            c1 = t_mcu_control.changes((0:899)'*0.01, m1);
            tc.verifyEqual(c1(:, 2)', [M('BOOT') M('SELFTEST') M('STANDBY') M('AUTO_START') M('AUTO') M('HOLD') ...
                M('AUTO_START') M('AUTO') M('CTRL_STOP') M('STANDBY')], 'rc_required_in_auto = true');
            tc.verifyEqual(c1(9, 1), 7.99, 'AbsTol', 1e-9, 'RC loss in AUTO -> CTRL_STOP at once');
            nf = [base; loss; {1:900, 'prm_flag', logical([false S.rearm.auto_selftest])}];
            m2 = tc.mmRun(t_mcu_control.seqOf(900, nf));
            tc.verifyEqual(m2(end), M('AUTO'), 'rc_required_in_auto = false: RC inputs and RC bits ignored in AUTO');
            % (d) remote E-stop from CTRL_STOP (moving) -> ESTOP.DECEL; HW E-stop -> ESTOP.SAFE at once;
            %     remote E-stop in FAULT stays FAULT
            spec = {120:900, 'arm', true; 120, 'arm_req', true; 150:900, 'standstill', false; 200:900, 'arm', false; ...
                250:300, 'fault_word', B(1); 280:300, 'fault_word', B([0 1])};
            [m3, y3] = tc.mmRun(t_mcu_control.seqOf(400, spec));
            tc.verifyEqual(m3([199 200 250]), [M('MANUAL'); M('CTRL_STOP'); M('ESTOP')]);
            tc.verifyTrue(all(y3.drive_enable(250:279)), 'ESTOP.DECEL from CTRL_STOP');
            tc.verifyFalse(any(y3.drive_enable(280:400)), 'HW E-stop ends ESTOP.DECEL');
            spec = {120:900, 'arm', true; 120, 'arm_req', true; 150:400, 'fault_word', B(29); 200:300, 'fault_word', B([1 29])};
            m4 = tc.mmRun(t_mcu_control.seqOf(400, spec));
            tc.verifyTrue(all(m4(150:400) == M('FAULT')), 'bumper -> FAULT; remote E-stop does not leave FAULT');
            % (e) ESTOP.DECEL ends after estop_tmax without standstill
            spec = {120:900, 'arm', true; 120, 'arm_req', true; 1:900, 'standstill', false; 200:210, 'fault_word', B(1)};
            [m5, y5] = tc.mmRun(t_mcu_control.seqOf(400, spec));
            kE = find(m5 == M('ESTOP'), 1);
            tc.verifyEqual(find(~y5.drive_enable(kE:end), 1) - 1, round(P.electronics.shaper.estop_tmax/0.01), 'ESTOP.DECEL 1.2 s');
            % (f) CTRL_STOP without standstill leaves after ctrlstop_timeout: the time-out is min(ctrlstop_timeout,
            %     v0/a_ctrlstop + ctrlstop_t_extra) since D71, so v0 (v_max at entry) 5 m/s keeps this case on the 5 s cap
            %     (testTEL52_CtrlStopTimeoutV0 tests the v0 term)
            spec = {120:2000, 'arm', true; 120, 'arm_req', true; 1:2000, 'standstill', false; 1:2000, 'v_max', 5; ...
                200:2000, 'arm', false};
            m6 = tc.mmRun(t_mcu_control.seqOf(1000, spec));
            kC = find(m6 == M('CTRL_STOP'), 1); kS = find(m6 == M('STANDBY') & (1:1000)' > kC, 1);
            tc.verifyEqual(kS - kC, round(S.ctrlstop_timeout/0.01), 'CTRL_STOP timeout');
        end

        function testTEL14_NeverActiveWithoutArm(tc)
            % random inputs (20 000 ticks): ACTIVE is entered only from STANDBY on a tick with arm_req
            M = @(s) double(ShaddadMode.(s));
            rng(14);
            n = 20000;
            seq = repmat(struct('arm', [], 'arm_req', [], 'sw_mode', [], 'fault_word', [], 'reset_req', [], ...
                'standstill', [], 'override', [], 'rc_ok', [], 'comp_ok', [], 'gnc_ready', [], 'mode_req', [], ...
                'safety_ok', [], 'hv_ready', [], 'sticks_neutral', [], 'lid_closed', []), 1, n);
            hold = @(p) cumsum(rand(n, 1) < p);                     % piecewise-constant random processes
            sw = uint8(1 + mod(hold(0.01), 3)); arm = mod(hold(0.01), 2) == 1;
            armReq = [false; diff(arm) > 0];
            bits = [1 2 3 4 5 9 10 15 17 31];
            fwk = mod(hold(0.004), 8);
            for k = 1:n
                seq(k).sw_mode = sw(k); seq(k).arm = arm(k); seq(k).arm_req = armReq(k);
                if fwk(k) >= 6, seq(k).fault_word = uint32(2^bits(1 + mod(fwk(k) + k, numel(bits)))); else, seq(k).fault_word = uint32(0); end
                seq(k).reset_req = rand < 0.01; seq(k).standstill = rand < 0.8; seq(k).override = rand < 0.002;
                seq(k).rc_ok = rand < 0.97; seq(k).comp_ok = rand < 0.95; seq(k).gnc_ready = rand < 0.9;
                seq(k).safety_ok = rand < 0.95; seq(k).hv_ready = rand < 0.95; seq(k).sticks_neutral = rand < 0.7;
                seq(k).lid_closed = rand < 0.95;
            end
            [mode, y] = tc.mmRun(seq);
            act = ismember(mode, [M('MANUAL') M('HOLD') M('AUTO_START') M('AUTO')]);
            entry = find(act(2:end) & ~act(1:end-1)) + 1;
            prev = mode(entry - 1);
            tc.verifyTrue(all(prev == M('STANDBY')), 'ACTIVE entered only from STANDBY');
            tc.verifyTrue(all(armReq(entry)), 'ACTIVE entered only on an arm_req tick');
            tc.verifyTrue(all(y.drive_enable(act)), 'drive enabled in ACTIVE');
            tc.verifyFalse(any(y.drive_enable(ismember(mode, [M('STANDBY') M('FAULT') M('BOOT') M('SELFTEST')]))));
            tc.log(1, sprintf('T-EL-14 fuzz: %d ACTIVE entries, modes visited %s', numel(entry), mat2str(unique(mode)')));
            tc.verifyGreaterThanOrEqual(numel(entry), 10);
        end

        function testModeManagerBlock(tc)
            % block-level spot check (fw_safety_supervisor -> fw_mode_manager -> fw_vehicle_ctrl chain harness)
            u = struct('rc_arm', [1.5 1; 4.0 0], 'rc_arm_req', [1.5 1; 1.51 0], 'rc_v_rc', [2.0 0.5], ...
                'reset_req', [0 0], 'di_word', []);
            u = rmfield(u, {'reset_req', 'di_word'});
            r = mcu_ctrl_harness('sim', 'ctrl', 5, u);
            c = t_mcu_control.changes(r.t, r.mode);
            M = @(s) double(ShaddadMode.(s));
            tc.verifyEqual(c(:, 2)', [M('BOOT') M('SELFTEST') M('STANDBY') M('MANUAL') M('CTRL_STOP') M('STANDBY')]);
            tc.verifyEqual(c(2:4, 1)', [0.01 1.01 1.5], 'AbsTol', 1e-9);
            tc.verifyTrue(r.rearm_req(2), 'rearm_req at SELFTEST entry');
            tc.verifyEqual(r.drive_cmd.L.enable, r.drive_enable, 'DRV_CMD enable = drive_enable');
            tc.verifyEqual(r.drive_cmd.L.park_req, ~r.drive_enable, 'park_req = NOT drive_enable');
            tc.verifyEqual(double(r.bms_req(end, :)), [1 0 0 0], 'BMS_REQ: HV enable');
            al = double(r.drive_cmd.L.alive);
            tc.verifyTrue(all(mod(diff(al), 256) == 1), 'DRV_CMD alive +1 per frame');
            tc.verifyTrue(all(r.drive_cmd.L.mode == 1), 'DRV_CMD mode SPEED');
        end

        function testAutoArbitration(tc)
            % AUTO: v / omega saturations, lateral-acceleration limit, GNC accel_limit cap, cmd_valid / soft stop
            P = evalin('base', 'P'); LIM = P.electronics.lim;
            d = mcu_ctrl_harness('defaults', 'ctrl');
            g = d.gnc_cmd; tt = (0:0.05:16)';
            g.seq = [tt mod((1:numel(tt))', 256)]; g.gnc_ready = 1; g.accel_limit = 0.3;
            g.cmd_valid = [5.0 1; 12.0 0; 12.5 1]; g.v_cmd = [5.0 2.0; 13.5 -1.0]; g.omega_cmd = [5.0 2.0; 13.5 0];
            g.soft_stop_req = [15.0 1];
            u = struct('rc_sw_mode', 3, 'rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0], 'comp_ok', 1, 'gnc_cmd', g);
            r = mcu_ctrl_harness('sim', 'ctrl', 16, u);
            M = @(s) double(ShaddadMode.(s));
            k = @(x) find(r.t >= x, 1);
            c = t_mcu_control.changes(r.t, r.mode);
            tc.verifyEqual(c(:, 2)', [M('BOOT') M('SELFTEST') M('STANDBY') M('AUTO_START') M('AUTO')], 'AUTO reached');
            vc = r.vc_tel; ka = r.t >= 4.6;
            tc.verifyEqual(vc(k(11.9), 1), LIM.v_max_auto, 'AbsTol', 1e-6, 'v_cmd saturated at v_max_auto');
            tc.verifyEqual(vc(k(11.9), 2), min(LIM.w_max_auto, LIM.a_lat_max/abs(vc(k(11.9) - 1, 3))), 'RelTol', 1e-3, ...
                'omega limited by w_max_auto and a_lat_max/|v_ref|');
            kf = r.t >= 5.0 & r.t < 12.0;                                  % forward start from standstill to 1.5 m/s
            tc.verifyLessThanOrEqual(max(vc(kf, 5)), 0.3 + 1e-3, 'GNC accel_limit caps the AUTO acceleration');
            tc.verifyGreaterThanOrEqual(max(vc(kf, 5)), 0.3 - 1e-3);
            kd = r.t >= 12.0 & r.t < 12.5;                                 % cmd_valid false: deceleration not capped
            tc.verifyLessThan(min(vc(kd, 5)), -0.3 - 1e-3, 'deceleration uses a_dec_auto (not the cap)');
            tc.verifyEqual(vc(k(12.05), 1:2), [0 0], 'cmd_valid false -> zero targets');
            tc.verifyEqual(vc(k(14.0), 1), -LIM.v_rev_auto, 'AbsTol', 1e-6, 'reverse saturated at v_rev_auto');
            tc.verifyEqual(vc(k(15.05), 1:2), [0 0], 'soft_stop_req -> zero targets');
            tc.verifyEqual(r.mode(end), uint8(M('AUTO')), 'MCU stays AUTO (GNC holds zero speed, D20)');
            tc.verifyTrue(all(r.cmd_src(ka) == 2), 'cmd_src AUTO');
        end

        % ================================================================== watchdog / re-arm (unit harness)
        function testWatchdogUnit(tc)
            P = evalin('base', 'P'); W = P.electronics.wdg;
            t = (0:4000)'*0.001;
            al = mod([floor(t/0.01) floor(t/0.05) floor(t/0.1)], 256);
            al50 = al; al50(t >= 2.0, 2) = al(find(t >= 2.0, 1) - 1, 2);         % Task_50 counter frozen from 2.0 s
            base = struct('alive', [t al], 'mode', [0 0; 0.02 1]);
            u1 = base; u1.rearm_req = [0.02 1; 0.03 0; 1.0 1; 1.01 0];
            u2 = base; u2.alive = [t al50];
            u3 = base; u3.fault_inject = [1.0 2^W.stall_1k_bit; 1.2 0];
            u4 = base; u4.fault_word = [1.0 2^5; 1.5 2^0; 2.0 2^23; 2.5 2^31; 3.0 0]; u4.mode = [0 0; 0.02 1; 3.5 0];
            rr = mcu_ctrl_harness('sim', 'wdg', 4, {u1, u2, u3, u4});
            r = rr{1};
            tDo0 = t_mcu_control.firstTime(r.t, r.mcu_safety_ok);
            rise = r.t(find(diff(r.safety_reset_req) > 0) + 1); fall = r.t(find(diff(r.safety_reset_req) < 0) + 1);
            tc.verifyTrue(all(r.wdg_kick), 'healthy tasks: kick every tick');
            tc.verifyEqual(tDo0, 0.02, 'AbsTol', 1e-9, 'DO0 with mode ~= BOOT');
            tc.verifyEqual(rise', [tDo0 + 0.02, 1.0], 'AbsTol', 1e-9, 'DO1 after t_do0_settle; immediate when DO0 is settled');
            tc.verifyEqual((fall - rise)', [0.05 0.05], 'AbsTol', 1e-9, 'DO1 pulse t_pulse');
            r = rr{2};
            lastChange = 2.0 - 0.05 + 0.0;                                  % last visible counter change (1.95 s)
            tStop = t_mcu_control.firstTime(r.t, ~r.wdg_kick);
            tc.verifyEqual(tStop - lastChange, W.age_lim(2), 'AbsTol', 1e-9, 'kick stops age_lim(2) after the last change');
            tc.verifyFalse(r.mcu_safety_ok(end), 'unhealthy watchdog drops DO0');
            r = rr{3};
            tc.verifyFalse(any(r.wdg_kick(r.t >= 1.0 & r.t < 1.2)), 'stall bit stops the kick');
            tc.verifyTrue(all(r.wdg_kick(r.t >= 1.2)));
            r = rr{4}; k = @(x) find(r.t >= x, 1);
            tc.verifyFalse(r.mcu_safety_ok(k(1.2)), 'maskDO0 bit 5 drops DO0');
            tc.verifyTrue(r.mcu_safety_ok(k(1.7)), 'bit 0 (hardware chain) keeps DO0 (D30)');
            tc.verifyFalse(r.mcu_safety_ok(k(2.2)), 'bit 23 drops DO0');
            tc.verifyTrue(r.mcu_safety_ok(k(2.7)), 'bit 31 keeps DO0 (D30)');
            tc.verifyFalse(r.mcu_safety_ok(k(3.7)), 'BOOT drops DO0');
        end

        % ================================================================== implement / aux (unit harness)
        function testImplementAuxUnit(tc)
            P = evalin('base', 'P'); X = P.electronics.aux; di = P.electronics.di; DO = P.electronics.do;
            d = mcu_ctrl_harness('defaults', 'aux');
            M = @(s) double(ShaddadMode.(s));
            bit = @(w, b) bitand(double(w), 2^b) ~= 0;
            % beacon / buzzer per mode (4 s each)
            modes = {'STANDBY', 'MANUAL', 'AUTO_START', 'AUTO', 'ESTOP', 'FAULT', 'BOOT'};
            u = cell(1, numel(modes));
            for k = 1:numel(modes), u{k} = struct('st_mode', M(modes{k}), 'st_v_track', 0.5); end
            % mower MANUAL (sw_impl 2), tiller AUTO with GNC, sprayer MANUAL / AUTO, implement fault, lift
            g = d.gnc_cmd; g.tiller_enable = 1; g.tiller_speed_cmd = 20; g.tiller_depth_cmd = 0.05;
            u{end + 1} = struct('st_mode', M('MANUAL'), 'st_rc_sw_impl', 2, 'di_word', d.di_word + 2^di.impl_id0, ...
                'st_rc_sw_lift', [0 1; 2 3], 'adc_lift_pos', [0 0.5; 1.5 0.99; 2.5 0.5; 3.5 0.01]);
            u{end + 1} = struct('st_mode', M('AUTO'), 'st_rc_sw_impl', 2, 'di_word', d.di_word + 2^di.impl_id1, ...
                'gnc_cmd', g, 'adc_lift_pos', 0.5);
            gs = d.gnc_cmd; gs.spray_enable = 1; gs.spray_section_mask = 5; gs.spray_flow_cmd = 4;
            u{end + 1} = struct('st_mode', [0 M('MANUAL'); 2 M('AUTO')], 'st_rc_sw_impl', [0 2; 2 3], 'st_v_track', [0 0.1; 1 0.5], ...
                'di_word', d.di_word + 2^di.impl_id0 + 2^di.impl_id1, 'gnc_cmd', gs);
            u{end + 1} = struct('st_mode', M('MANUAL'), 'st_rc_sw_impl', 2, 'di_word', d.di_word + 2^di.impl_id0, ...
                'st_fault_word', [1 2^28], 'adc_T_encl', 330);
            rr = mcu_ctrl_harness('sim', 'aux', 4, u);
            for k = 1:numel(modes)
                r = rr{k}; w = r.do_aux(r.t >= 1); G = bit(w, DO.beacon_green); Am = bit(w, DO.beacon_amber);
                Rd = bit(w, DO.beacon_red); Bz = bit(w, DO.buzzer);
                frac = @(x) mean(x); nTog = @(x) sum(diff(x) ~= 0)/3;          % toggles per second
                switch modes{k}
                    case 'STANDBY', tc.verifyEqual([frac(G) frac(Am) frac(Rd)], [0 0.5 0], 'AbsTol', 0.05); tc.verifyEqual(nTog(Am), 2, 'AbsTol', 0.4);
                    case 'MANUAL', tc.verifyEqual([frac(G) frac(Am) frac(Rd)], [0 1 0]);
                    case 'AUTO_START', tc.verifyEqual([frac(G) frac(Am) frac(Rd)], [0.5 0 0], 'AbsTol', 0.05);
                        tc.verifyEqual(nTog(G), 4, 'AbsTol', 0.4); tc.verifyTrue(all(Bz), 'AUTO_START buzzer');
                    case 'AUTO', tc.verifyEqual([frac(G) frac(Am) frac(Rd)], [1 0 0]);
                        tc.verifyEqual(frac(Bz), X.buzz_on/X.buzz_period, 'AbsTol', 0.02, 'AUTO moving beep 0.1 s / 2 s');
                    case 'ESTOP', tc.verifyEqual([frac(G) frac(Am) frac(Rd)], [0 0 1]);
                    case 'FAULT', tc.verifyEqual(frac(Rd), 0.5, 'AbsTol', 0.05); tc.verifyEqual(nTog(Rd), 4, 'AbsTol', 0.4);
                    case 'BOOT', tc.verifyEqual(double(w), zeros(size(w)), 'all DO off in BOOT');
                end
                if ~strcmp(modes{k}, 'BOOT')
                    tc.verifyTrue(all(bit(w, DO.companion_pwr_en) & bit(w, DO.lidar_pwr_en)), [modes{k} ': power enables after BOOT']);
                end
                tc.verifyFalse(any(bit(w, DO.impl24_en)), 'K3 off without an implement');
            end
            r = rr{numel(modes) + 1}; k = @(x) find(r.t >= x, 1);           % mower MANUAL
            tc.verifyTrue(all(r.impl_enable(r.t > 0.1))); tc.verifyEqual(r.impl_speed_rpm(end), X.mower_w_default*30/pi, 'RelTol', 1e-5);
            tc.verifyTrue(bit(r.do_aux(end), DO.impl24_en), 'K3 with an implement');
            tc.verifyEqual(r.pwm_duty(k(1.0), 2), X.lift_duty_max, 'AbsTol', 1e-6); tc.verifyFalse(bit(r.do_aux(k(1.0)), DO.lift_dir), 'UP');
            tc.verifyEqual(r.pwm_duty(k(1.8), 2), 0, 'end stop at the top');
            tc.verifyEqual(r.pwm_duty(k(3.0), 2), X.lift_duty_max, 'AbsTol', 1e-6); tc.verifyTrue(bit(r.do_aux(k(3.0)), DO.lift_dir), 'DOWN');
            tc.verifyEqual(r.pwm_duty(k(3.8), 2), 0, 'end stop at the bottom');
            r = rr{numel(modes) + 2};                                          % tiller AUTO
            tc.verifyTrue(r.impl_enable(end)); tc.verifyEqual(r.impl_speed_rpm(end), 20*30/pi, 'RelTol', 1e-5);
            hm = -X.till_d_max*(1 - 0.5); duty = max(-X.lift_duty_max, min(X.lift_duty_max, X.lift_Kp*(-0.05 - hm)));
            tc.verifyEqual(r.pwm_duty(end, 2), abs(duty), 'AbsTol', 1e-5, 'AUTO height loop'); tc.verifyEqual(bit(r.do_aux(end), DO.lift_dir), duty < 0);
            r = rr{numel(modes) + 3}; k = @(x) find(r.t >= x, 1);              % sprayer
            noz = @(w) bitand(bitshift(double(w), -DO.nozzle1), 255);
            tc.verifyEqual(noz(r.do_aux(k(0.5))), 0, 'no spray below spray_v_min');
            tc.verifyEqual(noz(r.do_aux(k(1.5))), 255, 'MANUAL: all nozzles'); tc.verifyEqual(r.pwm_duty(k(1.5), 1), X.pump_duty_man, 'AbsTol', 1e-6);
            tc.verifyEqual(noz(r.do_aux(k(3.0))), 5, 'AUTO: section mask');
            tc.verifyEqual(r.pwm_duty(k(3.0), 1), 4/X.pump_flow_full, 'AbsTol', 1e-5, 'AUTO pump duty = flow/flow_full');
            tc.verifyFalse(any(r.impl_enable), 'no rotor on a sprayer');
            r = rr{numel(modes) + 4};                                          % implement fault, fans
            tc.verifyTrue(r.impl_enable(find(r.t >= 0.5, 1))); tc.verifyFalse(any(r.impl_enable(r.t >= 1.05)), 'IMPL_FAULT stops the rotor');
            fan = interp1(X.fan_bp, X.fan_duty, min(max(330, X.fan_bp(1)), X.fan_bp(end)));
            tc.verifyEqual(r.pwm_duty(end, 3:4), [fan fan], 'AbsTol', 1e-5, 'fan duty from the enclosure NTC');
        end

        % ================================================================== closed loop (bench)
        function testTEL40_PowerUp(tc)
            P = evalin('base', 'P');
            r = tc.bench('manual');
            tDo0 = t_mcu_control.firstTime(r.t1, r.do0);
            rise = r.t1(find(diff(r.do1) > 0) + 1); fall = r.t1(find(diff(r.do1) < 0) + 1);
            tRelay = t_mcu_control.firstTime(r.t1, r.state(:, 8) > 0.5);
            tStb = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == double(ShaddadMode.STANDBY));
            A = bitand(double(r.telem.fault_word), double(P.electronics.safety.maskA)) ~= 0;
            tc.log(1, sprintf('T-EL-40: DO0 %.3f s, DO1 %.3f..%.3f s, relay %.3f s, STANDBY %.2f s, class-A samples %d', ...
                tDo0, rise(1), fall(1), tRelay, tStb, sum(A)));
            tc.verifyLessThanOrEqual(tDo0, 0.030 + 1e-9);
            tc.verifyEqual(rise(1) - tDo0, 0.020, 'AbsTol', 0.002 + 1e-9);
            tc.verifyEqual(fall(1) - rise(1), 0.050, 'AbsTol', 0.002 + 1e-9);
            tc.verifyNumElements(rise, 1, 'one automatic DO1 pulse');
            tc.verifyLessThanOrEqual(tRelay, 0.1);
            tc.verifyLessThanOrEqual(tStb, 1.5);
            tc.verifyFalse(any(A), 'no class-A bit');
            tc.verifyTrue(all(r.do0(r.t1 >= tDo0)), 'DO0 stays high');
            tc.verifyTrue(all(r.wdg_kick), 'watchdog kicked every 1 ms');
        end

        function testTEL40_EstopAtPowerUp(tc)
            r = tc.bench('estop_powerup');
            M = @(s) double(ShaddadMode.(s));
            c = t_mcu_control.changes(r.t10, r.telem.mode);
            rise = r.t1(find(diff(r.do1) > 0) + 1);
            tc.log(1, ['T-EL-40 E-stop at power-up, modes: ' mat2str(c', 4) ', DO1 rises ' mat2str(rise', 4)]);
            tc.verifyEqual(c(:, 2)', [M('BOOT') M('SELFTEST') M('ESTOP') M('STANDBY')]);
            tc.verifyTrue(all(r.do0(r.t1 >= 0.03)), 'DO0 stays high with the E-stop pressed (D30)');
            tc.verifyFalse(any(r.state(r.t1 < 2.5, 8) > 0.5), 'relay open while pressed / before reset');
            tc.verifyEqual(numel(rise), 2, 'DO1 re-issued on the reset');
            tc.verifyGreaterThanOrEqual(rise(2), 2.5); tc.verifyLessThanOrEqual(rise(2), 2.6);
            tc.verifyTrue(r.state(end, 8) > 0.5, 'relay closed after release + reset');
            tc.verifyEqual(c(end, 1), 2.51, 'AbsTol', 0.02, 'STANDBY after the reset');
            tc.verifyTrue(bitand(double(r.telem.fault_word(find(r.t10 >= 1.0, 1))), 1) ~= 0, 'HW_ESTOP latched');
        end

        function testTEL24_ThrottleAndLatency(tc)
            P = evalin('base', 'P');
            r = tc.bench('manual');
            k = find(r.t10 >= 7.0, 1);
            vt = r.telem.v_target(k); vs = mean(r.state(r.t1 >= 6 & r.t1 <= 8, 3));
            tc.log(1, sprintf('T-EL-24: v_target %.4f m/s, steady %.4f m/s', vt, vs));
            vExp = 0.4*P.electronics.lim.v_max_man;                % half throttle through the expo curve
            tc.verifyEqual(vt, vExp, 'AbsTol', 0.01);
            tc.verifyEqual(vs, vExp, 'AbsTol', 0.03);
            % TX frame sample -> DRV_CMD change (D37 window 18..28 ms) over 100 settled throttle steps; the first
            % response of the shaper is the torque feed-forward field (tau_ff = Kff_a*a_ref, bytes 5..6)
            r = tc.bench('latency');
            c2 = double(r.rc_frame.ch_raw(:, P.electronics.rcmap.ch.thr));
            kk = find(diff(c2) ~= 0) + 1; kk = kk(r.t1(kk) > 3.5);
            tTx = r.t1(kk) - P.rc.frame_period;
            tff = double(typecast(reshape(uint8(r.drvL(:, 5:6))', [], 1), 'int16'));
            tD = r.t10(find(diff(tff) ~= 0) + 1);
            lat = arrayfun(@(x) tD(find(tD >= x - 1e-9, 1)) - x, tTx);
            quiet = arrayfun(@(x) ~any(tD > x - 0.2 & tD < x - 1e-9), tTx);   % settled before each step
            tc.log(1, sprintf('T-EL-24 / D37: %d steps (%d settled), TX sample -> DRV_CMD change %.0f..%.0f ms (mean %.1f)', ...
                numel(lat), sum(quiet), 1e3*min(lat), 1e3*max(lat), 1e3*mean(lat)));
            tc.verifyEqual(numel(lat), 100);
            tc.verifyTrue(all(quiet), 'DRV_CMD settled before every step');
            tc.verifyGreaterThanOrEqual(min(lat), 0.018 - 1e-9);
            tc.verifyLessThanOrEqual(max(lat), 0.028 + 1e-9);
        end

        function testTEL25_PivotAndMixer(tc)
            P = evalin('base', 'P');
            h = P.electronics.mix.chi*P.electronics.mix.B/2;
            r = tc.bench('pivot');
            k = r.t1 >= 5.0 & r.t1 <= 6.0;
            vL = mean(r.state(k, 1)); vR = mean(r.state(k, 2));
            tc.log(1, sprintf('T-EL-25: v_L %+.4f, v_R %+.4f m/s (chi*B/2 = %.4f with chi_nom %.4f from T03)', vL, vR, h, P.electronics.mix.chi));
            tc.verifyEqual(vL, h, 'AbsTol', 0.01); tc.verifyEqual(vR, -h, 'AbsTol', 0.01);
            % T-EL-09 identity on every closed-loop sample
            tl = r.telem; hs = double(single(h));
            vLr = tl.v_ref - tl.w_mix*hs; vRr = tl.v_ref + tl.w_mix*hs; kd = max(1, max(abs([vLr vRr]), [], 2)/P.electronics.lim.v_track_max);
            tc.verifyEqual(tl.vtrack_ref, [vLr vRr]./kd, 'AbsTol', 1e-6);
            % after the pivot the re-biased yaw rate stays unbiased: straight drive has no yaw
            kEnd = r.t1 >= 8.5;
            tc.verifyLessThan(max(abs(r.state(kEnd, 4))), 0.01, 'straight after the pivot');
        end

        function testYawLoop(tc)
            % AUTO (shaper K_w_auto, D41): omega step 0.3 rad/s at 0.8 m/s, plant yaw gain 0.7
            r = tc.bench('yaw_auto');
            ts = t_mcu_control.firstTime(r.t10, abs(r.telem.w_target) > 0.01);
            fin = r.telem.w_target(find(r.t10 >= 12.5, 1));
            y = r.state(:, 4); k = r.t1 >= ts & r.t1 <= 12.9;
            t63 = t_mcu_control.firstTime(r.t1, y >= 0.632*fin, ts) - ts;
            os = max(y(k))/fin - 1; err = abs(mean(y(r.t1 >= 12.5 & r.t1 <= 12.9))/fin - 1);
            tc.log(1, sprintf('Yaw loop: step %.3f rad/s at %.2f s (v %.3f m/s), 63 %% at %.3f s, steady error %.2f %%, overshoot %.2f %%', ...
                fin, ts, mean(r.state(r.t1 >= 7.9 & r.t1 <= 8.0, 3)), t63, 100*err, 100*os));
            tc.verifyEqual(fin, 0.3, 'AbsTol', 1e-6);
            tc.verifyEqual(mean(r.state(r.t1 >= 7.9 & r.t1 <= 8.0, 3)), 0.8, 'AbsTol', 0.01);
            tc.verifyEqual(t63, 0.25, 'AbsTol', 0.1);
            tc.verifyLessThan(err, 0.03);
            tc.verifyLessThan(os, 0.15);
        end

        function testTEL43_YawPauseNoCreep(tc)
            % S4 stop 3 (2026-11-14): AUTO pause after a turn. The turn (0.3 rad/s at 0.5 m/s, plant yaw gain 0.7)
            % leaves the yaw PI with an integrator; then v_cmd = omega_cmd = 0 for 66 s with cmd_valid, mode AUTO.
            % Before the commanded-standstill gate the tracks pivoted under yaw.standstill_v, the gyro re-bias
            % averages absorbed the turn, the re-biased rate read 0 and the PI never unwound. Pass: vtrack_ref 0 and
            % no heading creep in the pause, and no bias average learns a turn: the telemetry yaw rate (fw_imu_tilt)
            % matches the true rate in the pause, and in the straight drive after it the yaw PI holds the re-biased
            % rate of Yaw_meas at w_ref = 0, so the true yaw rate there is that loop's bias error (ideal bench gyro)
            P = t_mcu_control.nominalP();
            tStop = 9.0; tRes = 75.0; T = 81.0;
            s = struct('op', struct('sw_mode', 3, 'sw_arm', [1.5 1]), ...
                'gnc', struct('cmd_valid', [5.0 1], 'v_cmd', [5.0 0.5; tStop 0; tRes 0.3], 'omega_cmd', [6.0 0.3; tStop 0]), ...
                'plant', [1 0.7 0 0 1]);
            r = mcu_cl_run(T, s, P);
            tl = r.telem; t10 = r.t10; t1 = r.t1;
            u = tl.w_mix - tl.w_ref;                                        % yaw PI output (w_mix = w_ref + u)
            uTurn = u(find(t10 < tStop, 1, 'last'));
            tZ = t_mcu_control.firstTime(t10, tl.v_target == 0 & tl.w_target == 0, tStop);
            kH = t10 >= tZ + 5 & t10 < tRes; kH1 = t1 >= tZ + 5 & t1 < tRes;   % the last 60 s of the pause
            psi = rad2deg(unwrap(r.state(:, 7))); psi0 = psi(find(kH1, 1));
            dPsi = max(abs(psi(kH1) - psi0));
            vtr = max(abs(tl.vtrack_ref(kH, :)), [], 'all');
            kB = t10 >= tRes - 10 & t10 < tRes; kB1 = t1 >= tRes - 10 & t1 < tRes;
            eTilt = abs(mean(tl.yaw_rate_meas(kB)) - mean(r.state(kB1, 4)));
            eLoop = abs(mean(r.state(t1 >= T - 2, 4)));
            tc.log(1, sprintf(['T-EL-43: u at the end of the turn %+.4f rad/s; pause from %.2f s: mean u %+.5f rad/s, ' ...
                'max |vtrack_ref| %.2e m/s, heading change %.3f deg over %.1f s; bias error: telemetry %.5f rad/s, loop ' ...
                '(yaw rate driving straight after the pause) %.5f rad/s'], uTurn, tZ, mean(u(kH)), vtr, dPsi, ...
                t10(find(kH, 1, 'last')) - t10(find(kH, 1)), eTilt, eLoop));
            tc.verifyTrue(all(double(tl.mode(kH)) == double(ShaddadMode.AUTO)), 'AUTO throughout the pause');
            tc.verifyGreaterThan(abs(uTurn), 0.05, 'the turn leaves the yaw PI with an integrator');
            tc.verifyLessThan(vtr, 1e-4, 'vtrack_ref 0 at commanded standstill (m/s)');
            tc.verifyLessThan(dPsi, 0.5, 'heading change over the last 60 s of the pause (deg)');
            tc.verifyLessThan(eTilt, 0.002, 'telemetry yaw-rate error in the pause (fw_imu_tilt bias, rad/s)');
            tc.verifyLessThan(eLoop, 0.002, 'yaw rate driving straight after the pause (Yaw_meas bias error, rad/s)');
        end

        function testTEL45_PivotEstopDecel(tc)
            % R-34 (review MCU-05, 2026-11-15): remote E-stop during a pivot turn. The mode manager's standstill is
            % max(|v_L|, |v_R|) < standstill_v; with |mean(v_L, v_R)|, which is 0 in a pivot, ESTOP.DECEL ended on its
            % first sample and the drive went off with the tracks at chi*B/2 = 0.42 m/s. Pass: ESTOP.DECEL keeps the
            % drive enabled while the yaw axis follows the stop profile (|alpha_ref| <= alpha_stop), and the drive goes
            % off only with both tracks under standstill_v, within estop_tmax; ESTOP stays latched
            P = t_mcu_control.nominalP(); S = P.electronics.safety; SH = P.electronics.shaper;
            M = @(s) double(ShaddadMode.(s));
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_steer', [2.0 1.0], 'sw_estop', [4.0 1]));
            r = mcu_cl_run(6, s, P);
            t10 = r.t10; md = double(r.telem.mode);
            tE = t_mcu_control.firstTime(t10, md == M('ESTOP'), 3.9);
            tOff = t_mcu_control.firstTime(t10, ~r.cmd.L.enable & ~r.cmd.R.enable, tE);
            vmax = max(abs(r.state(:, 1:2)), [], 2);                          % both tracks, 1 ms
            vAt = @(t) vmax(find(r.t1 <= t + 1e-9, 1, 'last'));
            w0 = abs(r.telem.w_ref(find(t10 < tE, 1, 'last')));
            kD = t10 >= tE & t10 < tOff;
            aMax = max(abs(r.telem.alpha_ref(kD)));
            tc.log(1, sprintf(['T-EL-45: pivot w_ref %.3f rad/s, max |v_track| %.3f m/s at ESTOP (%.2f s); drive off ' ...
                '%.3f s later (w0/alpha_stop %.3f s), max |v_track| then %.4f m/s; max |alpha_ref| in DECEL %.2f rad/s^2'], ...
                w0, vAt(tE), tE, tOff - tE, w0/SH.alpha_stop, vAt(tOff), aMax));
            tc.verifyGreaterThan(vAt(tE), 0.3, 'the tracks pivot at the E-stop (m/s)');
            tc.verifyGreaterThan(tOff - tE, 0.5*w0/SH.alpha_stop, 'ESTOP.DECEL follows the stop profile: drive not off at once (s)');
            tc.verifyLessThanOrEqual(tOff - tE, SH.estop_tmax + 0.02, 'drive off by estop_tmax (s)');
            tc.verifyLessThan(vAt(tOff), S.standstill_v + 0.01, 'drive off only with both tracks under standstill_v (m/s)');
            tc.verifyLessThanOrEqual(aMax, SH.alpha_stop + 1e-3, 'yaw stop profile |alpha_ref| <= alpha_stop (rad/s^2)');
            tc.verifyTrue(all(md(t10 >= tE) == M('ESTOP')), 'ESTOP latched');
        end

        function testTEL46_TiltHoldOnInvalidImu(tc)
            % R-05 (review MCU-06, 2026-11-15). (a) Supervisor harness: tilt above tilt_stop from t0 with imu_valid
            % false for one sample every 0.4 s. An invalid sample holds the tilt timers (as tilt_gate, D47) and the tilt
            % bits are suppressed only once IMU_FAULT (bit 24) persists; before, each invalid sample restarted the 0.5 s
            % timer, so TILT_STOP never set. Pass: bit 15 (and 14) set after t_persist plus the held samples, bit 24
            % never. (b) Control harness: the yaw PI resets on the instantaneous imu_valid: wound up to u_max, one
            % invalid sample gives u = 0 and restarts the integrator (before, only the persisted bit 24 reset it)
            P = evalin('base', 'P'); S = P.electronics.safety; Y = P.electronics.yaw;
            t0 = 1.0; T = 4.0; drops = (1.3:0.4:3.7)';
            tab = zeros(2*numel(drops), 2);
            tab(1:2:end, :) = [drops zeros(size(drops))]; tab(2:2:end, :) = [drops + 0.01 ones(size(drops))];
            u = struct('tilt_pitch', [t0 S.tilt_stop + deg2rad(2)], 'tilt_imu_valid', [0 1; tab]);
            r = mcu_ctrl_harness('sim', 'sup', T, u);
            fw = double(r.fault_word); t = r.t;
            valid = ~ismember(round(t*100), round(drops*100));
            k0 = find(t >= t0 - 1e-9, 1); N = round(S.t_persist(1 + 15)/0.01) + 1;   % ticks, as Persisted_t
            tExp = t(k0 - 1 + find(cumsum(valid(k0:end)) >= N, 1));                  % only the valid samples count
            t14 = t_mcu_control.firstTime(t, bitand(fw, 2^14) ~= 0); t15 = t_mcu_control.firstTime(t, bitand(fw, 2^15) ~= 0);
            tc.log(1, sprintf(['T-EL-46 (a): tilt %.1f deg from %.2f s, imu_valid false one sample every 0.4 s: TILT_WARN ' ...
                'at %.2f s, TILT_STOP at %.2f s (expected %.2f s = t0 + t_persist + held samples), IMU_FAULT samples %d'], ...
                rad2deg(S.tilt_stop) + 2, t0, t14, t15, tExp, sum(bitand(fw, 2^24) ~= 0)));
            tc.verifyEqual(t15, tExp, 'AbsTol', 0.01 + 1e-9, 'TILT_STOP (bit 15) persists through single invalid samples');
            tc.verifyEqual(t14, tExp, 'AbsTol', 0.01 + 1e-9, 'TILT_WARN (bit 14) as well');
            tc.verifyFalse(any(bitand(fw, 2^24) ~= 0), 'no IMU_FAULT from single invalid samples');
            % (b) MANUAL from 1.5 s, yaw-rate demand 0.5 rad/s from 2.0 s with the measured rate 0: u winds up to u_max
            tI = 3.0;
            u2 = struct('rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0], 'rc_w_rc', [2.0 0.5], ...
                'tilt_imu_valid', [0 1; tI 0; tI + 0.01 1]);
            r2 = mcu_ctrl_harness('sim', 'ctrl', 3.5, u2);
            uPI = r2.vc_tel(:, 7) - r2.vc_tel(:, 4);                                % w_mix - w_ref
            kI = find(r2.t >= tI - 1e-9, 1);
            tc.log(1, sprintf('T-EL-46 (b): yaw PI u before the invalid sample %.4f, in it %.4f, after it %.4f rad/s (u_max %.2f)', ...
                uPI(kI - 1), uPI(kI), uPI(kI + 1), Y.u_max));
            tc.verifyEqual(double(r2.mode(kI)), double(ShaddadMode.MANUAL), 'MANUAL at the invalid sample');
            tc.verifyGreaterThan(uPI(kI - 1), 0.9*Y.u_max, 'yaw PI wound up before the invalid sample');
            tc.verifyEqual(uPI(kI), 0, 'AbsTol', 1e-6, 'yaw PI output 0 on the invalid IMU sample');
            tc.verifyLessThan(uPI(kI + 1), 0.9*Y.u_max, 'the integrator restarts from 0 after it');
        end

        function testTEL47_StuckImuFault(tc)
            % R-05 (review MCU-03 / NAV-03, 2026-11-15): sens_acq_imu (Task_5) marks imu_meas invalid once all 9 raw
            % axes have repeated bit for bit for safety.imu_frozen_t, so a stuck IMU gives IMU_FAULT (bit 24, class C:
            % derate_imu, yaw PI off). Bench: IMU_Emu holds its sample while flt.imu_fault = 1 (stuck, as the plant
            % sens_imu_model); its one-LSB magnetometer toggle keeps the healthy sample fresh at rest. Pass: no invalid
            % sample at rest before the fault (self-test passes: STANDBY), invalid imu_frozen_t after the IMU sticks,
            % IMU_FAULT t_persist later and while stuck; class C: STANDBY stays
            P = t_mcu_control.nominalP(); S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            tF = 3.0; T = 5.0;
            r = mcu_cl_run(T, struct('flt', struct('imu_fault', [0 0; tF 1])), P);
            t10 = r.t10; ww = double(r.telem.warn_word); fw = double(r.telem.fault_word);
            tInv = t_mcu_control.firstTime(t10, bitand(ww, 2^24) ~= 0, tF);         % condition 24 = NOT imu_valid
            tBit = t_mcu_control.firstTime(t10, bitand(fw, 2^24) ~= 0, tF);
            tStb = t_mcu_control.firstTime(t10, double(r.telem.mode) == M('STANDBY'));
            kRest = t10 >= 0.05 & t10 < tF;                                  % after the power-up samples (RT initial values)
            tc.log(1, sprintf(['T-EL-47: STANDBY at %.2f s; IMU stuck from %.2f s: imu_valid false at %.3f s (+%.3f s, ' ...
                'imu_frozen_t %.3f s), IMU_FAULT at %.3f s (+%.3f s after invalid); invalid samples at rest before the ' ...
                'fault %d'], tStb, tF, tInv, tInv - tF, S.imu_frozen_t, tBit, tBit - tInv, sum(bitand(ww(kRest), 2^24) ~= 0)));
            tc.verifyLessThanOrEqual(tStb, 1.5, 'self-test passes with the healthy bench IMU (STANDBY, s)');
            tc.verifyFalse(any(bitand(ww(kRest), 2^24) ~= 0), 'IMU valid at rest before the fault (no false frozen flag)');
            tc.verifyEqual(tInv - tF, S.imu_frozen_t, 'AbsTol', 0.02, 'imu_valid false imu_frozen_t after the IMU sticks (s)');
            tc.verifyEqual(tBit - tInv, S.t_persist(1 + 24), 'AbsTol', 0.01 + 1e-9, 'IMU_FAULT (bit 24) t_persist later (s)');
            tc.verifyTrue(all(bitand(fw(t10 >= tBit), 2^24) ~= 0), 'IMU_FAULT stays while the IMU is stuck');
            tc.verifyTrue(all(double(r.telem.mode(t10 >= tStb)) == M('STANDBY')), 'class C: no mode change');
        end

        % ================================================================== soft-stop policy D71 (R-01, 2026-11-15)
        function testTEL48_ClassBStopParks(tc)
            % (i) a class-B stop (RC link lost while driving, bit 2): CTRL_STOP decelerates, then parks: DRV_CMD
            % enable 0, park_req 1 with the torque limits held until both ESCs report the brake engaged with the
            % torque off (bench ESC_Fb as esc_sm: n_engage), then STANDBY with the limits 0; no STOP_FAIL, DO0 high
            P = t_mcu_control.nominalP(); S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [2.0 0.5]), 'flt', struct('rc_link_loss', [5.0 1]));
            r = mcu_cl_run(8, s, P);
            t10 = r.t10; md = double(r.telem.mode); c = r.cmd;
            off = ~c.L.enable & ~c.R.enable; park = c.L.park_req & c.R.park_req;
            lim = [c.L.tau_lim_mot c.R.tau_lim_mot c.L.tau_lim_reg c.R.tau_lim_reg];
            tC = t_mcu_control.firstTime(t10, md == M('CTRL_STOP'), 4.9);
            tP = t_mcu_control.firstTime(t10, off & park, tC);                     % parking starts (decoded DRV_CMD)
            tS = t_mcu_control.firstTime(t10, md == M('STANDBY'), tC);
            kP = t10 >= tP & t10 < tS - 0.011; kS = t10 >= tS + 0.02;
            kB = find(t10 < tP, 1, 'last');
            vmax = max(abs(r.state(:, 1:2)), [], 2); vP = vmax(find(r.t1 <= tP + 1e-9, 1, 'last'));
            fw = double(r.telem.fault_word);
            tc.log(1, sprintf(['T-EL-48: CTRL_STOP at %.2f s (RC link lost), parking from %.2f s (|v| %.4f m/s) to ' ...
                'STANDBY at %.2f s (%.3f s); limits held [mot L R, reg L R] %s N m (before %s), after STANDBY max %.3g'], ...
                tC, tP, vP, tS, tS - tP, mat2str(min(lim(kP, :), [], 1), 4), mat2str(lim(kB, :), 4), max(lim(kS, :), [], 'all')));
            tc.verifyTrue(all(md(t10 >= tC & t10 < tS) == M('CTRL_STOP')), 'CTRL_STOP until both brakes confirm');
            tc.verifyLessThan(vP, S.standstill_v, 'parking after standstill (m/s)');
            tc.verifyGreaterThanOrEqual(tS - tP, P.drive.brake.t_engage_wait, 'parking lasts the ESC engage time (s)');
            tc.verifyLessThan(tS - tP, S.stopfail_brake_t, 'brakes confirmed before the STOP_FAIL time-out (s)');
            tc.verifyTrue(any(kP) && all(lim(kP, :) > 0, 'all'), 'limits held in the parking phase');
            tc.verifyEqual(lim(find(kP, 1), :), lim(kB, :), 'RelTol', 1e-3, 'held at the values of the stop');
            tc.verifyTrue(all(lim(kS, :) == 0, 'all'), 'limits 0 in STANDBY');
            tc.verifyTrue(all(park(t10 >= tP)), 'park_req from the parking phase on');
            tc.verifyFalse(any(bitand(fw, 2^22) ~= 0), 'no STOP_FAIL');
            tc.verifyTrue(all(r.do0(r.t1 >= 0.03)), 'DO0 high: no STO in a normal stop (HV stays on)');
        end

        function testTEL49_StopNotDecelerating(tc)
            % (ii) a stop whose speed does not fall (bench: regen blocked, plant(5) a_run 0.01 m/s^2 from the stop):
            % STOP_FAIL (a) at stopfail_t0 after the CTRL_STOP entry, class A: DO0 low (relay -> STO), FAULT. No
            % OVERSPEED: its speed-follow condition is gated to the driving modes (MANUAL..AUTO), the monitor covers it
            P = t_mcu_control.nominalP(); S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            thr1 = t_mcu_control.stick(1.0/P.electronics.lim.v_max_man, P.electronics.rcmap.expo_thr);
            s = struct('op', struct('sw_arm', [1.5 1; 5.0 0], 'stick_thr', [2.0 thr1]), ...
                'plant', [0 1 0 0 1 0 0; 5.0 1 0 0 1 0.01 0]);
            r = mcu_cl_run(7, s, P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word);
            tC = t_mcu_control.firstTime(t10, md == M('CTRL_STOP'), 4.9);
            tF = t_mcu_control.firstTime(t10, bitand(fw, 2^22) ~= 0, tC);
            t16 = t_mcu_control.firstTime(t10, bitand(fw, 2^16) ~= 0, tC);
            tD = t_mcu_control.firstTime(r.t1, ~r.do0, tC);
            tR = t_mcu_control.firstTime(r.t1, r.state(:, 8) < 0.5, tC);
            tFa = t_mcu_control.firstTime(t10, md == M('FAULT'), tC);
            v = r.state(:, 3); v0 = v(find(r.t1 <= tC + 1e-9, 1, 'last')); vF = v(find(r.t1 <= tF + 1e-9, 1, 'last'));
            tc.log(1, sprintf(['T-EL-49: CTRL_STOP at %.2f s from %.3f m/s, speed held (%.3f m/s at STOP_FAIL); STOP_FAIL ' ...
                'at +%.3f s, OVERSPEED at +%.3f s, DO0 low at +%.3f s, relay open at +%.3f s, FAULT at +%.3f s'], ...
                tC, v0, vF, tF - tC, t16 - tC, tD - tC, tR - tC, tFa - tC));
            tc.verifyGreaterThan(vF, 0.9*v0, 'the speed did not fall');
            tc.verifyEqual(tF - tC, S.stopfail_t0 + 0.01, 'AbsTol', 0.03, 'STOP_FAIL at stopfail_t0 (+ one sample)');
            tc.verifyLessThanOrEqual(tD - tF, 0.015, 'DO0 low with STOP_FAIL (maskDO0)');
            tc.verifyLessThanOrEqual(tR - tD, 0.03, 'the safety relay opens (STO)');
            tc.verifyLessThanOrEqual(tFa - tF, 0.011, 'class A -> FAULT');
            tc.verifyTrue(all(bitand(fw(t10 >= tF), 2^22) ~= 0), 'STOP_FAIL latched');
            tc.verifyTrue(isnan(t16), 'no OVERSPEED in CTRL_STOP (speed-follow only while driving)');
        end

        function testTEL50_BrakeNeverConfirms(tc)
            % (iii) the left brake never reports engaged (bench plant(6) brake_fail b0): the parking phase after a
            % controlled stop does not end; STOP_FAIL (b) stopfail_brake_t after the park request, DO0 low, FAULT
            P = t_mcu_control.nominalP(); S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            s = struct('op', struct('sw_arm', [1.5 1; 4.0 0], 'stick_thr', [2.0 0.5]), 'plant', [1 0 0 1 0 1]);
            r = mcu_cl_run(7, s, P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word); c = r.cmd;
            off = ~c.L.enable & ~c.R.enable; park = c.L.park_req & c.R.park_req;
            lim = [c.L.tau_lim_mot c.R.tau_lim_mot c.L.tau_lim_reg c.R.tau_lim_reg];
            tC = t_mcu_control.firstTime(t10, md == M('CTRL_STOP'), 3.9);
            tP = t_mcu_control.firstTime(t10, off & park, tC);
            tF = t_mcu_control.firstTime(t10, bitand(fw, 2^22) ~= 0, tC);
            tD = t_mcu_control.firstTime(r.t1, ~r.do0, tC);
            tFa = t_mcu_control.firstTime(t10, md == M('FAULT'), tC);
            kP = t10 >= tP & t10 < tF - 0.011;
            tc.log(1, sprintf(['T-EL-50: CTRL_STOP at %.2f s, parking from %.2f s, STOP_FAIL at +%.3f s after the park ' ...
                'request (stopfail_brake_t %.2f s), DO0 low at +%.3f s, FAULT at +%.3f s; min held limit %.3f N m'], tC, tP, ...
                tF - tP, S.stopfail_brake_t, tD - tP, tFa - tP, min(lim(kP, :), [], 'all')));
            tc.verifyFalse(any(md(t10 > tC & t10 < tF) == M('STANDBY')), 'no STANDBY: the brake never confirms');
            tc.verifyEqual(tF - tP, S.stopfail_brake_t + 0.01, 'AbsTol', 0.03, 'STOP_FAIL (b) at stopfail_brake_t');
            tc.verifyTrue(all(lim(kP, :) > 0, 'all'), 'limits held while waiting for the brakes');
            tc.verifyLessThanOrEqual(tD - tF, 0.015, 'DO0 low with STOP_FAIL');
            tc.verifyLessThanOrEqual(tFa - tF, 0.011, 'class A -> FAULT');
        end

        function testTEL51_MotionInStandby(tc)
            % (iv) motion while stopped: STANDBY at rest, then a runaway load (plant(5) a_run 0.5 m/s^2 from 3 s, the
            % brakes cannot hold it): STOP_FAIL (c) stopfail_move_t after the faster track passes standstill_v,
            % DO0 low, STANDBY -> FAULT
            P = t_mcu_control.nominalP(); S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            r = mcu_cl_run(5, struct('plant', [0 1 0 0 1 0 0; 3.0 1 0 0 1 0.5 0]), P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word);
            vm = abs(r.telem.vtrack_meas); vmax = max(vm, [], 2);
            tM = t_mcu_control.firstTime(t10, vmax > S.standstill_v, 2.9);         % the MCU sees the motion
            tF = t_mcu_control.firstTime(t10, bitand(fw, 2^22) ~= 0);
            tD = t_mcu_control.firstTime(r.t1, ~r.do0, 2.9);
            tFa = t_mcu_control.firstTime(t10, md == M('FAULT'), 2.9);
            tc.log(1, sprintf(['T-EL-51: runaway from 3.00 s in STANDBY: encoder speed > standstill_v at %.2f s, STOP_FAIL ' ...
                'at %.2f s (+%.3f s), DO0 low at %.3f s, FAULT at %.2f s'], tM, tF, tF - tM, tD, tFa));
            tc.verifyTrue(all(md(t10 >= 1.1 & t10 < tF) == M('STANDBY')), 'STANDBY until STOP_FAIL');
            tc.verifyEqual(tF - tM, S.stopfail_move_t + 0.01, 'AbsTol', 0.03, 'STOP_FAIL (c) after stopfail_move_t');
            tc.verifyLessThanOrEqual(tD - tF, 0.015, 'DO0 low with STOP_FAIL');
            tc.verifyLessThanOrEqual(tFa - tF, 0.011, 'class A -> FAULT');
        end

        function testTEL52_CtrlStopTimeoutV0(tc)
            % (v) the CTRL_STOP time-out from v0 (direct FSM calls, 0.01 s ticks): v0 = v_max at entry; the deceleration
            % ends at min(ctrlstop_timeout, v0/a_ctrlstop + ctrlstop_t_extra) when standstill is not held (a creep above
            % standstill_v), then the parking phase (drive off, limits_hold) lasts until brakes_ok, then STANDBY; the
            % speed falls at the profile rate, so no stop_fail. testTEL14_Guards (f) covers the 5 s cap (v0 5 m/s)
            P = evalin('base', 'P'); S = P.electronics.safety; a = P.electronics.shaper.a_ctrlstop;
            M = @(s) double(ShaddadMode.(s));
            for v0 = [1.4 1.2]
                kC = 200; nTo = round((v0/a + S.ctrlstop_t_extra)/0.01); kBr = kC + nTo + 10;   % brakes confirm 10 ticks later
                vt = @(k) (k < kC + nTo)*max(v0 - (k - kC)*0.01*a, 0.08);          % profile, a creep of 0.08 m/s, then
                                                                                    % still on the brakes (parking)
                spec = {120:900, 'arm', true; 120, 'arm_req', true; 1:900, 'standstill', false; 1:199, 'v_max', v0; ...
                    200:900, 'arm', false; 1:kBr, 'brakes_ok', false};
                for k = 200:900, spec(end + 1, :) = {k, 'v_max', vt(k)}; end %#ok<AGROW>
                [mode, y] = tc.mmRun(t_mcu_control.seqOf(900, spec));
                kD = kC - 1 + find(~y.drive_enable(kC:end), 1);                      % end of the deceleration
                kS = find(mode == M('STANDBY') & (1:900)' > kC, 1);
                tc.log(1, sprintf(['T-EL-52 v0 %.1f m/s: CTRL_STOP at tick %d, deceleration ends after %d ticks (expected ' ...
                    'min(%d, %d)), parking %d ticks, STANDBY at tick %d, stop_fail samples %d'], v0, kC, kD - kC, ...
                    round(S.ctrlstop_timeout/0.01), nTo, sum(y.stop_ctl(kD:kS - 1, 1)), kS, sum(y.stop_ctl(kC:kS, 2))));
                tc.verifyEqual(mode(kC), M('CTRL_STOP'));
                tc.verifyEqual(kD - kC, min(round(S.ctrlstop_timeout/0.01), nTo), sprintf('time-out from v0 %.1f m/s', v0));
                tc.verifyTrue(all(y.stop_ctl(kD:kS - 1, 1)) && all(mode(kD:kS - 1) == M('CTRL_STOP')), 'parking: limits_hold');
                tc.verifyEqual(kS, kBr + 1, 'STANDBY once the brakes confirm');
                tc.verifyFalse(any(y.stop_ctl(kC:kS, 2)), 'no stop_fail: the speed fell at the profile rate');
            end
        end

        function testTEL53_RunawayParksAtOnce(tc)
            % (vi) a runaway while driving (DRV-01, bench plant(5) a_run 1.0 m/s^2 from 5 s, beyond the brakes): the track
            % runs above its reference, OVERSPEED (bit 16) by the speed-follow condition below v_overspeed; the CTRL_STOP
            % entered with bit 16 skips the regen deceleration and parks at once (enable 0, park_req 1, limits held from
            % its first sample). Then the brakes cannot hold the bench runaway: STOP_FAIL ((a) in the park or (c) in
            % STANDBY) -> STO. The nominal bench scenarios (hard decelerations, E-stops, pivots, turns) set neither bit
            % 16 nor bit 22; the logged follow error is that of the driving modes (the gate of the condition, mode of
            % the previous sample MANUAL..AUTO) and, in brackets, of every sample
            P = t_mcu_control.nominalP(); L = P.electronics.lim;
            M = @(s) double(ShaddadMode.(s));
            thr = t_mcu_control.stick(0.8/L.v_max_man, P.electronics.rcmap.expo_thr);
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [2.0 thr]), 'plant', [0 1 0 0 1 0 0; 5.0 1 0 0 1 1.0 0]);
            r = mcu_cl_run(7, s, P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word); c = r.cmd;
            lim = [c.L.tau_lim_mot c.R.tau_lim_mot c.L.tau_lim_reg c.R.tau_lim_reg];
            t16 = t_mcu_control.firstTime(t10, bitand(fw, 2^16) ~= 0, 4.9);
            tC = t_mcu_control.firstTime(t10, md == M('CTRL_STOP'), 4.9);
            tF = t_mcu_control.firstTime(t10, bitand(fw, 2^22) ~= 0, tC);
            vmax = max(abs(r.state(:, 1:2)), [], 2); v16 = vmax(find(r.t1 <= t16 + 1e-9, 1, 'last'));
            kC = t10 >= tC + 0.0105 & md == M('CTRL_STOP');                          % the decoded DRV_CMD of CTRL_STOP
            tc.log(1, sprintf(['T-EL-53: runaway from 5.00 s at 0.8 m/s: OVERSPEED at %.2f s with %.3f m/s (v_overspeed ' ...
                '%.2f), CTRL_STOP at %.2f s parks at once (%d samples, min held limit %.3f N m), STOP_FAIL at +%.3f s'], ...
                t16, v16, L.v_overspeed, tC, sum(kC), min(lim(kC, :), [], 'all'), tF - tC));
            tc.verifyLessThan(v16, L.v_overspeed, 'OVERSPEED from the speed-follow condition (below v_overspeed)');
            tc.verifyLessThanOrEqual(tC - t16, 0.011, 'class B -> CTRL_STOP');
            tc.verifyTrue(any(kC), 'CTRL_STOP samples');
            tc.verifyTrue(all(~c.L.enable(kC) & ~c.R.enable(kC) & c.L.park_req(kC) & c.R.park_req(kC)), ...
                'no regen deceleration: park at once');
            tc.verifyTrue(all(lim(kC, :) > 0, 'all'), 'limits held in the park');
            tc.verifyTrue(isfinite(tF), 'STOP_FAIL when the brakes cannot hold the runaway (STO backup)');
            % no false trips in the nominal closed-loop scenarios; the largest speed-follow error, from telemetry
            [names, ~, ~] = t_mcu_control.scenarios();
            rep = {};
            for k = 1:numel(names)
                rk = tc.bench(names{k});
                f = double(rk.telem.fault_word);
                e = max(abs(rk.telem.vtrack_meas(2:end, :)) - abs(rk.telem.vtrack_ref(1:end - 1, :)), [], 2);
                md = double(rk.telem.mode(1:end - 1));
                drv = md >= M('MANUAL') & md <= M('AUTO');
                rep{end + 1} = sprintf('%s %.3f (%.3f)', names{k}, max([e(drv); -inf]), max(e)); %#ok<AGROW>
                tc.verifyFalse(any(bitand(f, 2^16 + 2^22) ~= 0), [names{k} ': no OVERSPEED / STOP_FAIL']);
            end
            tc.log(1, ['T-EL-53 nominal scenarios, max(|vtrack_meas| - |vtrack_ref of the previous sample|) m/s while ' ...
                'driving (every sample): ' ...
                strjoin(rep, ', ') sprintf(' (limit %.2f)', L.v_follow_dv)]);
        end

        % ================================================================== descent speed cap (Agents task descent, 2026-11-15)
        function testTEL54_DescentCapPerImplement(tc)
            % The always-on downhill speed cap (the user's decision of 2026-11-15): fw_vehicle_ctrl limits the speed
            % target to the P.structure.descent cap minus lim.descent.dv (the top-speed entries 1.41 m/s: no cap below
            % the top speed), by the downhill pitch in the direction of travel (+pitch forward, -pitch in reverse,
            % peak-held t_hold) and the implement row (DI IMPL_ID coding plug). Control harness, MANUAL from 1.5 s, full
            % stick (v_rc +/-v_max_man) from 2.0 s, downhill pitch 0 3 7 12 17 22 27 deg in 0.5 s steps from 2.0 s
            % (between breakpoints the steeper column applies), back to 0 at 5.5 s (the peak hold keeps the 27 deg cap
            % until 6.5 s); every implement row (0 none, 1 mower, 2 tiller, 3 sprayer) and direction. Also: an invalid
            % IMU sample (sprayer, level) gives the 20 deg column; an unknown implement (plug none, IMPL_FB sprayer) the
            % most restrictive entries (forward the sprayer's, reverse 0: no reversing)
            P = evalin('base', 'P'); L = P.electronics.lim; D = L.descent; di = P.electronics.di;
            d = mcu_ctrl_harness('defaults', 'ctrl');
            th = [0 3 7 12 17 22 27]; tS = 2.0 + 0.5*(0:numel(th) - 1);
            arm = {'rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0]};
            u = {}; key = zeros(0, 2);
            for code = 0:3
                for dir = [1 -1]
                    u{end + 1} = struct(arm{:}, 'rc_v_rc', [2.0 dir*L.v_max_man], ...
                        'tilt_pitch', [0 0; tS' dir*deg2rad(th'); 5.5 0], 'di_word', d.di_word + code*2^di.impl_id0); %#ok<AGROW>
                    key(end + 1, :) = [code dir]; %#ok<AGROW>
                end
            end
            u{end + 1} = struct(arm{:}, 'rc_v_rc', [2.0 L.v_max_man], 'di_word', d.di_word + 3*2^di.impl_id0, ...
                'tilt_imu_valid', [0 1; 3.0 0]);
            for dir = [1 -1]
                u{end + 1} = struct(arm{:}, 'rc_v_rc', [2.0 dir*L.v_max_man], ...
                    'tilt_pitch', [0 0; 2.5 dir*deg2rad(12)], 'impl_rx_ok', 1, 'impl_impl_type', 3); %#ok<AGROW>
            end
            rr = mcu_ctrl_harness('sim', 'ctrl', 7.0, u);
            vmax = L.v_max_man; nm = {'none', 'mower', 'tiller', 'sprayer'}; rep = {};
            at = @(r, t) find(r.t >= t - 1e-9, 1);
            for q = 1:size(key, 1)
                r = rr{q}; code = key(q, 1); dir = key(q, 2); vt = double(r.vc_tel(:, 1));
                tc.verifyEqual(double(r.mode(at(r, 2.0))), double(ShaddadMode.MANUAL), 'MANUAL');
                got = arrayfun(@(t) vt(at(r, t + 0.45)), tS);
                want = arrayfun(@(x) dir*local_descent_ref(D, code, dir, x, vmax), th);
                txt = sprintf('%s %s', nm{code + 1}, char('f'*(dir > 0) + 'r'*(dir < 0)));
                tc.verifyEqual(got, want, 'AbsTol', 1e-5, [txt ': speed target by downhill pitch']);
                tc.verifyEqual(vt(at(r, 6.3)), dir*local_descent_ref(D, code, dir, 27, vmax), 'AbsTol', 1e-5, ...
                    [txt ': peak hold 0.8 s after the pitch fell to 0']);
                tc.verifyEqual(vt(at(r, 6.8)), dir*local_descent_ref(D, code, dir, 0, vmax), 'AbsTol', 1e-5, ...
                    [txt ': released after t_hold']);
                rep{end + 1} = sprintf('%s %s', txt, mat2str(abs(got), 3)); %#ok<AGROW>
            end
            r = rr{size(key, 1) + 1}; vt = double(r.vc_tel(:, 1));
            tc.verifyEqual(vt(at(r, 2.9)), local_descent_ref(D, 3, 1, 0, vmax), 'AbsTol', 1e-5, 'sprayer level, IMU valid');
            tc.verifyEqual(vt(at(r, 3.3)), local_descent_ref(D, 3, 1, 20, vmax), 'AbsTol', 1e-5, ...
                'IMU invalid: the 20 deg column');
            tc.verifyEqual(vt(at(r, 3.9)), local_descent_ref(D, 3, 1, 20, vmax), 'AbsTol', 1e-5, ...
                'IMU invalid with IMU_FAULT (derate 0.25 leaves the lower cap)');
            imu = [vt(at(r, 2.9)) vt(at(r, 3.3)) vt(at(r, 3.9))];
            ru = rr{end - 1}; rv = rr{end};
            unk = [double(ru.vc_tel(at(ru, 2.3), 1)) double(ru.vc_tel(at(ru, 3.0), 1)) ...
                double(rv.vc_tel(at(rv, 2.3), 1)) double(rv.vc_tel(at(rv, 3.0), 1))];
            tc.verifyEqual(unk, [local_descent_ref(D, 'min', 1, 0, vmax) local_descent_ref(D, 'min', 1, 12, vmax) ...
                -local_descent_ref(D, 'min', -1, 0, vmax) -local_descent_ref(D, 'min', -1, 12, vmax)], 'AbsTol', 1e-5, ...
                'unknown implement: the smallest entry of every row');
            tc.log(1, sprintf(['T-EL-54 speed target |m/s| at downhill pitch %s deg (f forward, r reverse): %s; ' ...
                'sprayer, IMU invalid at 3.0 s: %s; unknown implement (fwd 0/12 deg, rev 0/12 deg): %s'], mat2str(th), ...
                strjoin(rep, '; '), mat2str(imu, 3), mat2str(unk, 3)));
        end

        function testTEL55_DescentCapTrip(tc)
            % The trip at the cap (fault bit 16 OVERSPEED, in its own sample) and the park (descent task, 2026-11-15).
            % (a) Supervisor harness, MANUAL, sprayer on 19.5 deg (the 20 deg column, cap 0.30 m/s): the speed runs away
            % at 1 m/s^2 from the steady reference cap - dv: bit 16 sets in the first sample where v + a_track*t_pred
            % passes the cap (the speed-follow term alone needs 0.2 m/s and 0.2 s more), warn bit 16 with it, latched
            % until reset_req. (b) A cap that drops below the speed with regen (a crest: level, then 19.5 deg at 1 s): the
            % reference falls at 1.2 m/s^2 and the speed follows it 0.08 m/s behind, above the cap, then settles under
            % it: no trip. (c) As (a) in STANDBY: no trip (the drive gate). (d) Mode manager (direct calls): bit 16
            % during the CTRL_STOP deceleration parks at once
            % (drive off, limits_hold) and STANDBY follows the brakes; bit 16 during ESTOP.DECEL parks at once, ESTOP.SAFE
            % follows the brakes; a remote E-stop with bit 16 already set parks without a deceleration
            P = evalin('base', 'P'); L = P.electronics.lim; D = L.descent; di = P.electronics.di;
            kv = P.electronics.drv.k_v2w;
            d = mcu_ctrl_harness('defaults', 'sup');
            cap = D.v_fwd(4, local_descent_col(D, 19.5)); ref = cap - D.dv;   % sprayer, 19.5 deg: the 20 deg column
            t0 = 1.0; tt = (t0:0.01:1.4)';
            base = struct('mode_prev', 3, 'tilt_pitch', deg2rad(19.5), 'di_word', d.di_word + 3*2^di.impl_id0, ...
                'reset_req', [0 0; 2.0 1; 2.05 0]);
            va = ref + 0.005 + 1.0*(tt - t0);                           % (a) runaway at 1 m/s^2, back to rest at 1.41 s
            ua = base; o = d.odom;
            o.vTrack_LR = [0 ref ref; tt va va; 1.41 0 0]; o.wMotor_LR = [0 kv*[ref ref]; tt kv*[va va]; 1.41 0 0];
            ua.odom = o; ua.vtrack_ref_prev = [0 ref ref; 1.41 0 0]; ua.a_track = [0 0; t0 1.0; 1.41 0];
            rb = max(0.60 - 1.2*(tt - t0), ref); vb = rb + 0.08*(rb > ref) + 0.02*(rb <= ref);   % (b)
            ub = base; o = d.odom;
            o.vTrack_LR = [0 0.68 0.68; tt vb vb]; o.wMotor_LR = [0 kv*[0.68 0.68]; tt kv*[vb vb]];
            ub.odom = o; ub.vtrack_ref_prev = [0 0.60 0.60; tt rb rb]; ub.a_track = [0 0; t0 -1.2; t0 + 0.34 0];
            ub.tilt_pitch = [0 0; t0 deg2rad(19.5)];                    % level until the crest at t0
            uc = ua; uc.mode_prev = 2;                                  % (c)
            rr = mcu_ctrl_harness('sim', 'sup', 2.5, {ua, ub, uc});
            b16 = @(r) bitand(double(r.fault_word), 2^16) ~= 0; w16 = @(r) bitand(double(r.warn_word), 2^16) ~= 0;
            r = rr{1}; t = r.t; on = b16(r);
            kv0 = find(va + 1.0*D.t_pred > cap, 1); tExp = tt(kv0);       % the first sample of v + a*t_pred past the cap
            tSet = t_mcu_control.firstTime(t, on); tW = t_mcu_control.firstTime(t, w16(r));
            tc.verifyEqual(tSet, tExp, 'AbsTol', 1e-6, '(a) bit 16 in the sample the predicted speed passes the cap');
            tc.verifyEqual(tW, tExp, 'AbsTol', 1e-6, '(a) warn bit 16 with it');
            tc.verifyTrue(all(on(t >= tSet & t < 2.0)), '(a) latched after the speed fell (class B, until reset)');
            tc.verifyFalse(any(on(t >= 2.1)), '(a) cleared by reset_req with the condition gone');
            tc.verifyFalse(any(b16(rr{2})), '(b) no trip: the speed follows a falling reference with regen');
            tc.verifyFalse(any(b16(rr{3})), '(c) no trip in STANDBY');
            vSet = va(kv0);
            % (d) mode manager
            M = @(s) double(ShaddadMode.(s)); B16 = uint32(2^16);
            sp = {120:600, 'arm', true; 120, 'arm_req', true; 1:600, 'standstill', false; 1:600, 'v_max', 1.0};
            [m1, y1] = tc.mmRun(t_mcu_control.seqOf(600, [sp; {200:600, 'arm', false; 230:600, 'fault_word', B16; ...
                1:239, 'brakes_ok', false}]));
            [m2, y2] = tc.mmRun(t_mcu_control.seqOf(600, [sp; {200:600, 'fault_word', uint32(2); ...
                230:600, 'fault_word', uint32(2) + B16; 1:239, 'brakes_ok', false}]));
            [m3, y3] = tc.mmRun(t_mcu_control.seqOf(600, [sp; {200:600, 'fault_word', uint32(2) + B16; ...
                1:209, 'brakes_ok', false}]));
            tc.verifyTrue(all(m1(200:239) == M('CTRL_STOP')) && all(y1.drive_enable(200:229)), ...
                '(d1) CTRL_STOP decelerates until bit 16');
            tc.verifyTrue(~any(y1.drive_enable(230:239)) && all(y1.stop_ctl(230:239, 1)), ...
                '(d1) bit 16 in the deceleration: park at once, limits held');
            tc.verifyEqual(m1(240), M('STANDBY'), '(d1) STANDBY once the brakes confirm');
            tc.verifyTrue(all(m2(200:250) == M('ESTOP')) && all(y2.drive_enable(200:229)), '(d2) ESTOP.DECEL until bit 16');
            tc.verifyTrue(~any(y2.drive_enable(230:239)) && all(y2.stop_ctl(230:239, 1)), ...
                '(d2) bit 16 in ESTOP.DECEL: park at once, limits held');
            tc.verifyFalse(any(y2.stop_ctl(240:250, 1)), '(d2) ESTOP.SAFE once the brakes confirm');
            tc.verifyTrue(~any(y3.drive_enable(200:209)) && all(y3.stop_ctl(200:209, 1)) && all(m3(200:250) == M('ESTOP')), ...
                '(d3) remote E-stop with bit 16 set: park at once, no deceleration');
            tc.log(1, sprintf(['T-EL-55 (a) sprayer, 20 deg, cap %.2f m/s, reference %.2f m/s, runaway at 1 m/s^2: bit 16 at ' ...
                '%.2f s with v %.3f m/s (v + a*t_pred %.3f); (b) a crest at 1.2 m/s^2, 0.08 m/s lag: bit 16 samples %d; (c) ' ...
                'STANDBY: %d; (d) park at tick 230 (CTRL_STOP decel %d ticks, ESTOP.DECEL %d ticks), d3 drive_enable ' ...
                'samples %d'], cap, ref, tSet, vSet, vSet + D.t_pred, sum(b16(rr{2})), sum(b16(rr{3})), ...
                sum(y1.drive_enable(200:239)), sum(y2.drive_enable(200:239)), sum(y3.drive_enable(200:209))));
        end

        function testTEL56_CclZeroManualStop(tc)
            % Review finding 1 (2026-11-15): with the battery charge limit at 0 (CCL 0, regen only the P_aux_est
            % allowance) a MANUAL stop from 1.4 m/s lags its falling reference. Bench with the physical plant (226 kg,
            % no implement; f_rr 0.05; the decoded DRV_CMD torque limits), on the flat and on a 4 deg descent (the 5 deg
            % column: the bench pitch estimate errs by about 1 deg at the end of an acceleration, T-EL-33, and above 5
            % deg the 10 deg column applies): stick full forward from 2 s, released at 6 s. Pass: no OVERSPEED, the mode stays MANUAL, the speed only falls and the robot stops. The
            % log gives when the speed-follow rule before the descent task (|v| - |reference| > v_follow_dv for
            % t_persist) would have latched OVERSPEED
            P = t_mcu_control.nominalP(); L = P.electronics.lim; S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            pit = [0 4]; tabs = cell(1, 2);
            for k = 1:2
                tabs{k} = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [2.0 1.0; 6.0 0]), ...
                    'bms', struct('I_chg_lim_A', 0), 'plant', [1 deg2rad(pit(k)) 0 1 0 0 0 226 0.05]);
            end
            rr = mcu_cl_run(15, tabs, P);
            rep = {};
            for k = 1:2
                r = rr{k}; t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word);
                v = r.state(:, 3); t1 = r.t1;
                v0 = v(find(t1 <= 6.0, 1, 'last'));
                kR = t1 >= 6.0;
                rise = max(v(kR) - cummin(v(kR)));
                tStop = t_mcu_control.firstTime(t1, abs(v) < S.standstill_v, 6.0);
                lag = max(abs(r.telem.vtrack_meas(2:end, :)) - abs(r.telem.vtrack_ref(1:end - 1, :)), [], 2);
                tOld = local_old_follow_trip(t10, r.telem.vtrack_meas, r.telem.vtrack_ref, md, L.v_follow_dv, ...
                    round(S.t_persist(1 + 16)/0.01) + 1);
                kv = t10 > 6.0 & t10 < 7.0; reg = max(double(r.telem.tau_lim_reg(kv, :)), [], 'all');
                txt = sprintf('%.1f deg', pit(k));
                tc.verifyGreaterThan(v0, 1.38, [txt ': 1.4 m/s reached']);
                tc.verifyFalse(any(bitand(fw, 2^16 + 2^22) ~= 0), [txt ': no OVERSPEED, no STOP_FAIL']);
                tc.verifyTrue(all(md(t10 >= 1.6) == M('MANUAL')), [txt ': MANUAL to the end (a normal stop)']);
                tc.verifyLessThan(rise, 0.01, [txt ': the speed only falls after the release (m/s)']);
                tc.verifyTrue(isfinite(tStop), [txt ': the robot stops']);
                tc.verifyLessThan(reg, 1.0, [txt ': CCL 0 limits the regen torque (N m)']);
                rep{end + 1} = sprintf(['%s: v0 %.3f m/s, stop %.2f s after the release, largest lag %.3f m/s, tau_lim_reg ' ...
                    '<= %.2f N m in the first second, the pre-descent rule would latch OVERSPEED at %.2f s'], txt, v0, ...
                    tStop - 6.0, max(lag(t10(2:end) > 6.0)), reg, tOld); %#ok<AGROW>
            end
            tc.log(1, ['T-EL-56 CCL 0 MANUAL stop from 1.4 m/s: ' strjoin(rep, '; ')]);
        end

        function testTEL57_CclZeroAutoStop(tc)
            % As T-EL-56 in AUTO: the companion commands v_max_auto from 5 s and 0 at 9 s (a person or obstacle stop), CCL
            % 0, physical plant (226 kg, f_rr 0.05), on the flat and on 4 deg. Pass: no OVERSPEED, the mode stays AUTO,
            % the speed only falls and the robot stops
            P = t_mcu_control.nominalP(); L = P.electronics.lim; S = P.electronics.safety;
            M = @(s) double(ShaddadMode.(s));
            pit = [0 4]; tabs = cell(1, 2);
            for k = 1:2
                tabs{k} = struct('op', struct('sw_mode', 3, 'sw_arm', [1.5 1]), ...
                    'gnc', struct('cmd_valid', [5.0 1], 'v_cmd', [5.0 L.v_max_auto; 9.0 0]), ...
                    'bms', struct('I_chg_lim_A', 0), 'plant', [1 deg2rad(pit(k)) 0 1 0 0 0 226 0.05]);
            end
            rr = mcu_cl_run(17, tabs, P);
            rep = {};
            for k = 1:2
                r = rr{k}; t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word);
                v = r.state(:, 3); t1 = r.t1;
                v0 = v(find(t1 <= 9.0, 1, 'last'));
                kR = t1 >= 9.05;
                rise = max(v(kR) - cummin(v(kR)));
                tStop = t_mcu_control.firstTime(t1, abs(v) < S.standstill_v, 9.0);
                tOld = local_old_follow_trip(t10, r.telem.vtrack_meas, r.telem.vtrack_ref, md, L.v_follow_dv, ...
                    round(S.t_persist(1 + 16)/0.01) + 1);
                txt = sprintf('%.1f deg', pit(k));
                tc.verifyEqual(v0, L.v_max_auto, 'AbsTol', 0.03, [txt ': v_max_auto reached in AUTO']);
                tc.verifyFalse(any(bitand(fw, 2^16 + 2^22) ~= 0), [txt ': no OVERSPEED, no STOP_FAIL']);
                tc.verifyTrue(all(md(t10 >= 4.6) == M('AUTO')), [txt ': AUTO to the end']);
                tc.verifyLessThan(rise, 0.01, [txt ': the speed only falls after the stop command (m/s)']);
                tc.verifyTrue(isfinite(tStop), [txt ': the robot stops']);
                rep{end + 1} = sprintf('%s: v0 %.3f m/s, stop %.2f s after the command, pre-descent rule OVERSPEED at %.2f s', ...
                    txt, v0, tStop - 9.0, tOld); %#ok<AGROW>
            end
            tc.log(1, ['T-EL-57 CCL 0 AUTO stop: ' strjoin(rep, '; ')]);
        end

        function testTEL58_CclZeroSprayerDescent(tc)
            % The S2c case (descent task): the sprayer with a full tank (316 kg) on a 14.5 deg descent (the 15 deg
            % column; descent step 9, 2026-11-16: with the 12-breakpoint table of structure step 8 the 20 deg column gives
            % a target of 0.15 m/s, which the CCL 0 drive holds on 19.5 deg (max 0.146 m/s, no trip, no stop-go), and
            % CR-001 limits the full-tank sprayer to about 18.5 deg) with CCL 0 (bench, physical plant, f_rr 0.05). The
            % cap is 0.70 m/s, the speed target 0.60 m/s; the drive can hold only about 0.23 m/s on 14.5 deg (the regen
            % allowance), so the robot runs away past the target. Stop-go: stick forward at 2.0 s,
            % back at 3.0 s, reset (CH9) at 3.5 s, arm off/on at 4.0/4.5 s, stick forward again at 5.0 s. Pass, both
            % cycles: the speed target never above cap - dv; OVERSPEED (descent-cap trip) parks at once with the true
            % (bench) speed at or below the cap, the speed never above it (1 mm/s numerical tolerance), the brakes confirm
            % (STANDBY), no STOP_FAIL, DO0 high; the robot moves downhill in each cycle
            P = t_mcu_control.nominalP(); D = P.electronics.lim.descent;
            M = @(s) double(ShaddadMode.(s));
            th = 14.5; cap = D.v_fwd(4, local_descent_col(D, th)); ref = cap - D.dv;
            s = struct('op', struct('sw_arm', [1.5 1; 4.0 0; 4.5 1], 'stick_thr', [2.0 1.0; 3.0 0; 5.0 1.0; 6.0 0], ...
                'btn_reset', [3.5 1; 3.7 0]), 'bms', struct('I_chg_lim_A', 0), ...
                'plant', [1 deg2rad(th) 0 1 0 0 3 316 0.05]);
            r = mcu_cl_run(7.5, s, P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word); c = r.cmd;
            v = r.state(:, 3); t1 = r.t1;
            man = md == M('MANUAL');
            tc.verifyLessThanOrEqual(max(double(r.telem.v_target(man))), ref + 1e-5, 'speed target at most cap - dv');
            tc.verifyFalse(any(bitand(fw, 2^22) ~= 0), 'no STOP_FAIL');
            tc.verifyTrue(all(r.do0(t1 >= 0.03)), 'DO0 high (no STO)');
            rep = {};
            for w = [2.0 3.4; 5.0 7.4]'
                tC = t_mcu_control.firstTime(t10, md == M('CTRL_STOP'), w(1));
                tP = t_mcu_control.firstTime(t10, ~c.L.enable & ~c.R.enable & c.L.park_req & c.R.park_req, w(1));
                tS = t_mcu_control.firstTime(t10, md == M('STANDBY'), tC);
                vP = v(find(t1 <= tP + 1e-9, 1, 'last')); kw = t1 >= w(1) & t1 <= w(2);
                vm = max(v(kw)); vOdo = max(r.telem.vtrack_meas(t10 <= tC + 1e-9 & t10 >= tC - 0.0101, :), [], 'all');
                x = cumsum(v(kw))*0.001;
                tc.verifyTrue(isfinite(tC) && bitand(double(fw(t10 == tC)), 2^16) ~= 0, ...
                    sprintf('cycle %.0f s: CTRL_STOP with OVERSPEED', w(1)));
                tc.verifyLessThanOrEqual(tP - tC, 0.011, sprintf('cycle %.0f s: park at once', w(1)));
                tc.verifyLessThanOrEqual(vP, cap + 1e-3, sprintf('cycle %.0f s: the park starts at or below the cap', w(1)));
                tc.verifyLessThanOrEqual(vm, cap + 1e-3, sprintf('cycle %.0f s: the speed stays at or below the cap', w(1)));
                tc.verifyTrue(isfinite(tS), sprintf('cycle %.0f s: STANDBY on the brakes', w(1)));
                tc.verifyGreaterThan(x(end), 0.01, sprintf('cycle %.0f s: the robot moved downhill', w(1)));
                rep{end + 1} = sprintf(['cycle from %.1f s: OVERSPEED / CTRL_STOP at %.2f s (encoder speed %.3f m/s), park ' ...
                    'request at %.2f s with the true speed %.3f m/s, max %.3f m/s, STANDBY at %.2f s, %.3f m'], w(1), tC, ...
                    vOdo, tP, vP, vm, tS, x(end)); %#ok<AGROW>
            end
            tc.verifyTrue(any(md(t10 > 4.5 & t10 < 5.0) == M('MANUAL')), 'reset and re-arm: MANUAL again');
            tc.log(1, sprintf('T-EL-58 sprayer, %.1f deg, CCL 0, cap %.2f m/s, speed target %.2f m/s: %s', th, cap, ref, ...
                strjoin(rep, '; ')));
        end

        function testTEL59_TillerNoReverse(tc)
            % No reversing with the tiller fitted (the user's decision of 2026-11-15; P.structure.descent v_rev row 3 = 0):
            % bench, level, coding plug tiller (plant(7) 2), MANUAL: full stick back from 2 s to 4 s, then half forward
            % from 5 s. Pass: the speed target stays >= 0 and the robot does not move back, forward driving works; what the
            % operator sees: MANUAL throughout, no fault or warning bit from the cap (bit 28 IMPL_FAULT only: the bench
            % sends no IMPL_FB frames), v_applied (MCU_STATUS, = v_ref) 0 while the stick is back
            P = t_mcu_control.nominalP();
            M = @(s) double(ShaddadMode.(s));
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [2.0 -1.0; 4.0 0; 5.0 0.5; 7.0 0]), ...
                'plant', [1 0 0 1 0 0 2 0 0]);
            r = mcu_cl_run(8, s, P);
            t10 = r.t10; md = double(r.telem.mode); fw = double(r.telem.fault_word); ww = double(r.telem.warn_word);
            v = r.state(:, 3); t1 = r.t1; kB = t10 >= 2.0 & t10 < 4.0;
            other = @(x) bitand(x, bitcmp(uint32(2^28), 'uint32')) ~= 0;
            tc.verifyTrue(all(md(t10 >= 1.6) == M('MANUAL')), 'MANUAL throughout');
            tc.verifyEqual(max(abs(double(r.telem.v_target(kB)))), 0, 'AbsTol', 1e-6, 'speed target 0 with the stick back');
            tc.verifyEqual(max(abs(double(r.telem.v_ref(kB)))), 0, 'AbsTol', 1e-6, 'v_applied (v_ref) 0');
            tc.verifyGreaterThan(min(v), -0.005, 'no reverse motion (m/s)');
            tc.verifyGreaterThan(v(find(t1 <= 6.9, 1, 'last')), 0.3, 'forward driving works (m/s)');
            kM = t10 >= 1.6;                                            % MANUAL (no power-up frame timeouts)
            tc.verifyFalse(any(other(uint32(fw(kM)))) || any(other(uint32(ww(kM)))), 'no fault or warning bit but IMPL_FAULT');
            tc.log(1, sprintf(['T-EL-59 tiller, stick back 2-4 s: mode %s, speed target max |%.3g| m/s, v_ref max |%.3g| ' ...
                'm/s, min speed %.4f m/s; forward at 6.9 s %.3f m/s; fault words seen %s, warn words %s'], ...
                mat2str(unique(md(t10 >= 1.6))'), max(abs(double(r.telem.v_target(kB)))), ...
                max(abs(double(r.telem.v_ref(kB)))), min(v), v(find(t1 <= 6.9, 1, 'last')), ...
                mat2str(unique(fw(t10 >= 1.6))'), mat2str(unique(ww(t10 >= 1.6))')));
        end

        function testTEL60_MowerDeckRaised(tc)
            % The raised-deck backstop (the user, 2026-11-15: the GNC keeps the mower deck down whenever the robot moves;
            % the MCU applies P.structure.descent v_fwd_raised / v_rev_raised if the deck is reported up): the mower
            % (coding plug 1) with the lift potentiometer adc.lift_pos at or above lim.descent.lift_raised (deck height
            % h_raised) takes the raised-deck rows. Control harness, MANUAL, full stick from 2.0 s; lift_pos 0.5 (the bench
            % default) then 0.9 from 3.0 s; downhill pitch 0, then 7 deg (the 10 deg column) from 4.0 s; mower forward and
            % reverse, and the sprayer at the same lift position, which keeps its own row (the backstop is the mower's)
            P = evalin('base', 'P'); L = P.electronics.lim; D = L.descent; di = P.electronics.di;
            d = mcu_ctrl_harness('defaults', 'ctrl');
            arm = {'rc_arm', [1.5 1], 'rc_arm_req', [1.5 1; 1.51 0]};
            key = {1, 1; 1, -1; 3, 1}; u = cell(1, size(key, 1));
            for q = 1:size(key, 1)
                [code, dir] = key{q, :};
                u{q} = struct(arm{:}, 'rc_v_rc', [2.0 dir*L.v_max_man], 'di_word', d.di_word + code*2^di.impl_id0, ...
                    'adc_lift_pos', [0 0.5; 3.0 0.9], 'tilt_pitch', [0 0; 4.0 dir*deg2rad(7)]);
            end
            rr = mcu_ctrl_harness('sim', 'ctrl', 5.0, u);
            at = @(r, t) find(r.t >= t - 1e-9, 1); vmax = L.v_max_man; rep = {};
            tc.verifyLessThan(0.5, D.lift_raised, 'the bench / harness lift position 0.5 is a lowered deck');
            tc.verifyGreaterThanOrEqual(0.9, D.lift_raised, 'lift position 0.9 is a raised deck');
            for q = 1:size(key, 1)
                [code, dir] = key{q, :}; r = rr{q}; vt = double(r.vc_tel(:, 1));
                if code == 1, up = 'raised'; else, up = code; end
                got = [vt(at(r, 2.9)) vt(at(r, 3.4)) vt(at(r, 4.4))];
                want = dir*[local_descent_ref(D, code, dir, 0, vmax) local_descent_ref(D, up, dir, 0, vmax) ...
                    local_descent_ref(D, up, dir, 7, vmax)];
                txt = sprintf('plug %d %s', code, char('f'*(dir > 0) + 'r'*(dir < 0)));
                tc.verifyEqual(double(r.mode(at(r, 2.9))), double(ShaddadMode.MANUAL), [txt ': MANUAL']);
                tc.verifyEqual(got, want, 'AbsTol', 1e-5, [txt ': deck down 0 deg, raised 0 deg, raised 7 deg']);
                rep{end + 1} = sprintf('%s %s', txt, mat2str(got, 3)); %#ok<AGROW>
            end
            tc.log(1, sprintf(['T-EL-60 speed target m/s [deck down 0 deg, deck raised 0 deg, raised 7 deg], lift_raised ' ...
                '%.3f (deck %.3f m): %s'], D.lift_raised, D.h_raised, strjoin(rep, '; ')));
        end

        function testTEL61_HillStartRollback(tc)
            % The hill-start false trip (descent task, step 9, 2026-11-16; structure step 8 point 2, the integration S2
            % climb replay): at a brake release on a steep climb the robot rolls back while the speed loop takes over,
            % and the trip prediction |v| + max(a_track*sign(v), 0)*t_pred passed the reverse trip (S2: 0.175 m/s
            % against 0.15 m/s at a measured 20.5 deg): OVERSPEED latched for nothing. Descent_trip now drops the
            % prediction while a brake is engaged and for lim.descent.t_release after both released, while the reference
            % in the direction of travel is at most yaw.cmd_still_v; the trip on the measured speed stays. Bench,
            % physical plant, sprayer with a full tank (316 kg, f_rr 0.05) on a 20.2 deg climb (reverse downhill pitch:
            % the 20.5 deg column), the ESCs without a hold-torque preload (plant(6) b2: the hold force builds up after
            % each brake release, about the 0.09 m/s roll-back of the HiFi S2 hill start); arm at 1.5 s, stick 0, then
            % half forward from 3.0 s (the climb). (a) Normal charge current: no OVERSPEED (fault or warn bit 16), MANUAL
            % throughout, the roll-back is held and the robot climbs. (c) Control, no regeneration at all (CCL 0 and
            % tlim.P_aux_est 0): the drive cannot hold the roll-back, a real runaway inside the window: OVERSPEED on the
            % measured speed, the park at once, the brakes confirm (STANDBY), no STOP_FAIL. The log gives the speeds for
            % the safety argument. (The bench roll-back is gentler than the HiFi one: with t_release 0, the rule before,
            % it did not trip either, 2026-11-16; the false trip itself is the S2 replay's)
            P = t_mcu_control.nominalP(); D = P.electronics.lim.descent;
            M = @(s) double(ShaddadMode.(s)); b16 = @(x) bitand(double(x), 2^16) ~= 0;
            row = 4; th = 20.2;                                         % sprayer row (implement code 3)
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [3.0 0.5]), 'plant', [1 -deg2rad(th) 0 1 0 4 3 316 0.05]);
            sc = s; sc.bms = struct('I_chg_lim_A', 0);
            Pc = P; Pc.electronics.tlim.P_aux_est = 0;
            rr = {mcu_cl_run(4.5, s, P), mcu_cl_run(3.0, sc, Pc)};
            nh = round(D.t_hold/0.01); rep = cell(1, 2); x = cell(1, 2);
            for q = 1:2
                r = rr{q}; t10 = r.t10; c = r.cmd; t1 = r.t1; v = r.state(:, 3);
                rel = c.L.enable & c.R.enable & ~c.L.park_req & ~c.R.park_req;
                tR = t_mcu_control.firstTime(t10, rel, 1.4);             % the release command
                vm = mean(double(r.telem.vtrack_meas), 2);
                hp = movmax(-rad2deg(double(r.telem.pitch)), [nh - 1, 0]);   % held reverse downhill pitch
                kW = t10 >= tR & t10 <= tR + D.t_release;
                trip = D.trip_rev(row, local_descent_col(D, max(hp(kW))));
                fw = double(r.telem.fault_word); ww = double(r.telem.warn_word); md = double(r.telem.mode);
                t16 = t_mcu_control.firstTime(t10, b16(fw), tR); k16 = find(t10 == t16, 1);
                x{q} = struct('tR', tR, 'vmin', min(v(t1 >= tR & t1 <= tR + 1)), 'vmmin', min(vm(kW)), 'hp', max(hp(kW)), ...
                    'trip', trip, 't16', t16, 'v16', abs(vm(max([k16 1]))), 'md', md, 'fw', fw, 'ww', ww, 't10', t10);
                rep{q} = sprintf(['release %.2f s, held reverse pitch %.2f deg (trip %.2f m/s), roll-back true %.3f / ' ...
                    'encoder %.3f m/s, OVERSPEED at %+.2f s (encoder %.3f m/s)'], tR, x{q}.hp, trip, x{q}.vmin, ...
                    x{q}.vmmin, t16 - tR, x{q}.v16);
            end
            a = x{1}; r = rr{1};
            tc.verifyLessThan(a.vmin, -0.05, '(a) the brake release rolls the robot back (m/s)');
            tc.verifyFalse(any(b16(a.fw) | b16(a.ww)), '(a) no OVERSPEED: fault or warn bit 16');
            tc.verifyTrue(all(a.md(a.t10 >= 1.6) == M('MANUAL')), '(a) MANUAL throughout');
            tc.verifyGreaterThan(r.state(find(r.t1 <= 4.4, 1, 'last'), 3), 0.2, '(a) the robot climbs (m/s)');
            k = x{2}; r = rr{2}; c = r.cmd; v = r.state(:, 3); t1 = r.t1;
            tC = t_mcu_control.firstTime(k.t10, k.md == M('CTRL_STOP'), k.tR);
            tP = t_mcu_control.firstTime(k.t10, ~c.L.enable & ~c.R.enable & c.L.park_req & c.R.park_req, k.tR);
            tS = t_mcu_control.firstTime(k.t10, k.md == M('STANDBY'), tC);
            vP = abs(v(find(t1 <= tP + 1e-9, 1, 'last'))); vX = max(abs(v(t1 >= k.tR & t1 <= tS)));
            tc.verifyTrue(k.t16 <= k.tR + D.t_release, '(c) a real runaway in the window trips');
            tc.verifyGreaterThan(k.v16, k.trip - 1e-6, '(c) ... on the measured speed (the encoder speed past the trip)');
            tc.verifyLessThanOrEqual(tP - tC, 0.011, '(c) park at once');
            tc.verifyTrue(isfinite(tS), '(c) STANDBY on the brakes');
            tc.verifyFalse(any(bitand(k.fw, 2^22) ~= 0), '(c) no STOP_FAIL');
            tc.log(1, sprintf(['T-EL-61 sprayer 316 kg, %.1f deg climb, t_release %.2f s: (a) normal CCL: %s; (c) no ' ...
                'regen: %s; park request %+.3f s with the true speed %.3f m/s, max %.3f m/s until STANDBY (%+.2f s)'], ...
                th, D.t_release, rep{1}, rep{2}, tP - k.tR, vP, vX, tS - k.tR));
        end

        function testTEL30_31_AutoStartOverride(tc)
            P = evalin('base', 'P');
            r = tc.bench('yaw_auto');
            M = @(s) double(ShaddadMode.(s));
            c = t_mcu_control.changes(r.t10, r.telem.mode);
            tAS = c(find(c(:, 2) == M('AUTO_START'), 1), 1); tA = c(find(c(:, 2) == M('AUTO'), 1), 1);
            tc.log(1, ['T-EL-30/31 modes: ' mat2str(c', 4)]);
            tc.verifyEqual(tA - tAS, 3.0, 'AbsTol', 0.01, 'T-EL-30 AUTO_START 3.0 s');
            % buzzer during AUTO_START (DO b6, 1 kHz DO word)
            bz = bitand(r.do_word, 2^P.electronics.do.buzzer) ~= 0;
            tc.verifyTrue(all(bz(r.t1 >= tAS + 0.15 & r.t1 < tA)), 'AUTO_START buzzer (Task_10 -> Task_50 -> DO word RTs)');
            % override: stick deflection seen by Task_10 (|thr| > override_tol) -> MANUAL
            tO = t_mcu_control.firstTime(r.t10, abs(r.rc.thr) > P.electronics.rcmap.override_tol, 12.9);
            tM = c(find(c(:, 2) == M('MANUAL') & c(:, 1) > 12.9, 1), 1);
            tc.log(1, sprintf('T-EL-31: deflection at %.2f s -> MANUAL at %.2f s (%.3f s)', tO, tM, tM - tO));
            tc.verifyLessThanOrEqual(tM - tO, 0.21 + 1e-9);
            tc.verifyGreaterThanOrEqual(tM - tO, P.electronics.rcmap.override_t - 0.01 - 1e-9, 'override persistence');
        end

        function testTEL32_CompanionTimeout(tc)
            r = tc.bench('comp_timeout');
            tS = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == double(ShaddadMode.CTRL_STOP));
            tc.log(1, sprintf('T-EL-32: COMP_CMD seq frozen from 7.00 s (first stale frame) -> CTRL_STOP at %.2f s (%.3f s)', tS, tS - 7.0));
            tc.verifyEqual(tS - 7.0, 0.20, 'AbsTol', 0.02);
            tc.verifyTrue(bitand(double(r.telem.fault_word(find(r.t10 >= tS, 1))), 2^4) ~= 0, 'COMP_TIMEOUT bit');
            tc.verifyLessThan(abs(r.state(end, 3)), 0.02, 'stopped');
        end

        function testTEL42_CtrlStopNoTail(tc)
            P = evalin('base', 'P');
            r = tc.bench('ctrl_stop');
            tS = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == double(ShaddadMode.CTRL_STOP));
            v = r.state(:, 3); v0 = v(find(r.t1 >= tS, 1));
            tz = t_mcu_control.firstTime(r.t1, abs(v) < 0.02, tS) - tS;
            a = movmean([0; diff(v)]/0.001, 100);
            amax = max(-a(r.t1 > tS));
            tc.log(1, sprintf('T-EL-42: CTRL_STOP from %.3f m/s, |v| < 0.02 after %.3f s (limit %.3f), max decel (100 ms) %.3f m/s^2', ...
                v0, tz, v0/P.electronics.shaper.a_ctrlstop + 0.25, amax));
            tc.verifyEqual(v0, 1.0, 'AbsTol', 0.02);
            tc.verifyLessThanOrEqual(tz, v0/P.electronics.shaper.a_ctrlstop + 0.25);
            tc.verifyLessThanOrEqual(amax, 1.05);
            tc.verifyFalse(any(r.state(r.t1 > tS + tz + 0.2, 3) < -0.01), 'no reversal');
            c = t_mcu_control.changes(r.t10, r.telem.mode);
            tc.verifyEqual(c(end, 2), double(ShaddadMode.STANDBY), 'STANDBY after the controlled stop');
        end

        function testTEL28_RemoteEstop(tc)
            P = evalin('base', 'P');
            r = tc.bench('estop_rc');
            M = @(s) double(ShaddadMode.(s));
            tE = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == M('ESTOP'));
            v = r.state(:, 3); v0 = v(find(r.t1 >= tE, 1));
            % constant-deceleration phase: from 0.3 s after ESTOP until v < 0.3 m/s (linear fit)
            k = r.t1 >= tE + 0.3 & v > 0.3;
            p = polyfit(r.t1(k), v(k), 1);
            tPark = t_mcu_control.firstTime(r.t10, r.cmd.L.park_req, tE) - tE;
            tc.log(1, sprintf('T-EL-28: ESTOP at %.2f s from %.3f m/s, constant-phase decel %.3f m/s^2, park after %.3f s', ...
                tE, v0, -p(1), tPark));
            tc.verifyEqual(v0, P.electronics.lim.v_max_man, 'AbsTol', 0.03);
            tc.verifyEqual(-p(1), P.electronics.shaper.a_estop, 'AbsTol', 0.1);
            tc.verifyLessThanOrEqual(tPark, 1.2 + 1e-6);
            tc.verifyTrue(all(double(r.telem.mode(r.t10 >= tE & r.t10 < 9.6)) == M('ESTOP')), 'ESTOP latched (CH12 back to RUN at 9.0 s)');
            c = t_mcu_control.changes(r.t10, r.telem.mode);
            tc.verifyEqual(c(end, 2), M('STANDBY'), 'reset (CH9) -> STANDBY');
            tc.verifyEqual(c(end, 1), 9.62, 'AbsTol', 0.04);
        end

        function testTEL26_Failsafe(tc)
            M = @(s) double(ShaddadMode.(s));
            P = t_mcu_control.nominalP(); P.rc.rx_failsafe_time = 0.02;      % failsafe frames right after the RF loss
            thr = 0.6;
            s = struct('op', struct('sw_arm', [1.5 1; 7.5 0; 8.0 1], 'stick_thr', [2.0 thr; 6.5 0]), ...
                'flt', struct('rc_link_loss', [5.0 1; 6.0 0]));
            r = mcu_cl_run(10, s, P);
            tF = t_mcu_control.firstTime(r.t1, r.dec.frame_new & r.dec.failsafe, 4.9);
            tS = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == M('CTRL_STOP'));
            v = r.state(:, 3); a = movmean([0; diff(v)]/0.001, 100);
            amax = max(-a(r.t1 > tS));
            c = t_mcu_control.changes(r.t10, r.telem.mode);
            tc.log(1, sprintf('T-EL-26: first failsafe frame %.3f s -> CTRL_STOP %.2f s (%.0f ms), max decel %.3f m/s^2, modes %s', ...
                tF, tS, 1e3*(tS - tF), amax, mat2str(c', 4)));
            tc.verifyLessThanOrEqual(tS - tF, 0.020 + 1e-9);
            tc.verifyLessThanOrEqual(amax, 1.05);
            tc.verifyFalse(any(double(r.telem.mode) == M('ESTOP')), 'ESTOP never entered');
            % link back at 6 s, arm still on: no motion until arm off -> on (7.5 / 8.0 s)
            tc.verifyLessThan(max(abs(v(r.t1 > 6.8 & r.t1 < 8.0))), 0.01, 'no motion before the new arm edge');
            tc.verifyEqual(c(end, 2), M('MANUAL'), 'MANUAL again after arm off -> on');
            tc.verifyGreaterThan(c(end, 1), 8.0);
        end

        function testTEL27_NoPulses(tc)
            P = t_mcu_control.nominalP(); P.rc.rx_failsafe_mode = 2; P.rc.rx_failsafe_time = 0.02;   % NO_PULSES
            s = struct('op', struct('sw_arm', [1.5 1], 'stick_thr', [2.0 0.3]), 'flt', struct('rc_link_loss', [5.0 1]));
            r = mcu_cl_run(6, s, P);
            fv = r.dec.frame_new & ~r.dec.frame_lost & ~r.dec.failsafe;
            tLast = r.t1(find(r.t1 < 5.2 & fv, 1, 'last'));
            tLost = t_mcu_control.firstTime(r.t1, r.dec.rc_link_lost, 4.9);
            tBit = t_mcu_control.firstTime(r.t10, bitand(double(r.telem.fault_word), 4) ~= 0, 4.9);
            tc.log(1, sprintf('T-EL-27: last valid frame %.3f s, RC_LINK_LOST %.3f s (%.1f ms), fault bit 2 at %.2f s, frames after: %d', ...
                tLast, tLost, 1e3*(tLost - tLast), tBit, sum(r.dec.frame_new(r.t1 > 5.1))));
            tc.verifyEqual(tLost - tLast, 0.100, 'AbsTol', 0.002 + 1e-9);
            tc.verifyEqual(sum(r.dec.frame_new(r.t1 > 5.1)), 0, 'NO_PULSES: no frames');
            tc.verifyLessThanOrEqual(tBit - tLost, 0.01 + 1e-9);
            tc.verifyEqual(double(r.telem.mode(end)), double(ShaddadMode.CTRL_STOP), 'class B stop');
        end

        function testTEL41_RcLossInAuto(tc)
            M = @(s) double(ShaddadMode.(s));
            r = tc.bench('rc_auto_req');
            c1 = t_mcu_control.changes(r.t10, r.telem.mode);
            P = t_mcu_control.nominalP(); P.electronics.safety.rc_required_in_auto = false;
            [names, ~, tabs] = t_mcu_control.scenarios();
            r2 = mcu_cl_run(10, tabs{strcmp(names, 'rc_auto_req')}, P);
            c2 = t_mcu_control.changes(r2.t10, r2.telem.mode);
            tc.log(1, ['T-EL-41 rc_required_in_auto true: ' mat2str(c1', 4) '; false: ' mat2str(c2', 4)]);
            tc.verifyEqual(c1(end - 1, 2), M('CTRL_STOP'), 'required: CTRL_STOP');
            tc.verifyLessThanOrEqual(c1(end - 1, 1) - 7.0, 0.13 + 1e-9, 'RF loss -> CTRL_STOP <= 0.13 s (D37)');
            tc.verifyEqual(c2(end, 2), M('AUTO'), 'not required: stays AUTO');
            tc.verifyTrue(bitand(double(r2.telem.fault_word(end)), 2^3) ~= 0, 'failsafe frames arrive (RC_FAILSAFE shown)');
            tc.verifyEqual(r2.state(end, 3), 0.8, 'AbsTol', 0.02, 'keeps driving');
        end

        function testTEL33_Tilt(tc)
            P = evalin('base', 'P'); S = P.electronics.safety;
            r = tc.bench('tilt_static');
            fw = double(r.telem.fault_word);
            pitchTrue = @(t) 18 + max(t - 3, 0);                           % deg
            t14 = t_mcu_control.firstTime(r.t10, bitand(fw, 2^14) ~= 0); t15 = t_mcu_control.firstTime(r.t10, bitand(fw, 2^15) ~= 0);
            w = pitchTrue(t14 - S.t_tilt); st = pitchTrue(t15 - S.t_tilt);
            tc.log(1, sprintf('T-EL-33: TILT_WARN set at %.2f s (pitch %.2f deg 0.5 s before), TILT_STOP at %.2f s (%.2f deg)', t14, w, t15, st));
            tc.verifyEqual(w, rad2deg(S.tilt_warn), 'AbsTol', 1); tc.verifyEqual(st, rad2deg(S.tilt_stop), 'AbsTol', 1);
            r = tc.bench('tilt_ramp_start');
            fw = double(r.telem.fault_word);
            a = r.telem.a_ref; pe = rad2deg(abs(r.telem.pitch));
            tc.log(1, sprintf('T-EL-33 ramp starts (20 deg, a_ref up to %.2f m/s^2): max |pitch est| %.2f deg, TILT_WARN samples %d', ...
                max(a), max(pe(r.t10 > 2)), sum(bitand(fw, 2^14) ~= 0)));
            tc.verifyGreaterThanOrEqual(max(a), P.electronics.shaper.a_acc_man - 1e-3, '0.8 m/s^2 starts');
            tc.verifyFalse(any(bitand(fw, 2^14) ~= 0), 'no TILT_WARN during starts on a 20 deg ramp');
            tc.verifyEqual(double(r.telem.mode(end)), double(ShaddadMode.MANUAL));
        end

        function testTEL34_WatchdogStall(tc)
            r = tc.bench('wdg_stall');
            tStop = t_mcu_control.firstTime(r.t1, ~r.wdg_kick, 2.9);
            tc.log(1, sprintf('T-EL-34: Task_50 stall injected at 3.000 s -> wdg_kick stops at %.3f s (%.0f ms)', tStop, 1e3*(tStop - 3.0)));
            tc.verifyLessThanOrEqual(tStop - 3.0, 0.110 + 1e-9);
            tc.verifyTrue(all(r.wdg_kick(r.t1 < 3.0)));
            tc.verifyFalse(any(r.wdg_kick(r.t1 > tStop)), 'no kick while stalled (IWDG resets the MCU)');
            tc.verifyFalse(r.do0(end), 'DO0 drops with the unhealthy watchdog');
        end

        function testTEL35_FeedbackLoss(tc)
            M = @(s) double(ShaddadMode.(s));
            r = tc.bench('fb_loss');
            tB = t_mcu_control.firstTime(r.t10, bitand(double(r.telem.fault_word), 2^5) ~= 0);
            tF = t_mcu_control.firstTime(r.t10, double(r.telem.mode) == M('FAULT'));
            tEn = t_mcu_control.firstTime(r.t10, ~r.cmd.L.enable & ~r.cmd.R.enable, 4.9);
            tc.log(1, sprintf('T-EL-35: FB1/FB3_L lost at 5.00 s -> DRV_L_TIMEOUT %.2f s, FAULT %.2f s, enables false %.2f s', tB, tF, tEn));
            tc.verifyEqual(tB - 5.0, 0.050, 'AbsTol', 0.010 + 1e-9);
            tc.verifyEqual(tF, tB, 'AbsTol', 1e-9);
            tc.verifyLessThanOrEqual(tEn - tB, 0.010 + 1e-9);
            tc.verifyFalse(r.do0(end), 'DO0 drops (bit 5 in maskDO0, D30)');
        end

        function testTEL38_Traction(tc)
            r = tc.bench('slip');
            k = r.telem.k_traction;
            t05 = t_mcu_control.firstTime(r.t10, all(k <= 0.5 + 1e-4, 2), 6.0) - 6.0;
            tc.log(1, sprintf('T-EL-38: 50 %% slip from 6.00 s -> k_tr 0.5 after %.2f s; tau_lim_mot %.2f N m', t05, r.telem.tau_lim_mot(end, 1)));
            tc.verifyLessThanOrEqual(t05, 0.3 + 1e-9);
            tc.verifyTrue(all(k(r.t10 < 6.0, :) == 1, 'all'), 'k_tr 1 without slip');
            tc.verifyEqual(r.telem.tau_lim_mot(end, :), 0.5*r.telem.tau_lim_mot(find(r.t10 >= 5.9, 1), :), 'RelTol', 1e-3, 'k_tr scales the motoring limit');
        end

        % ================================================================== code generation
        function testCodegenBlocked(tc)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
            evalin('base', 'VC_TARGET = 0;');
            slbuild('Shaddad_MCU');
        end
    end

    methods (Static)
        function bad = badRates(sys, P)
            bad = {};
            blk = find_system(sys, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
            for k = 1:numel(blk)
                try
                    st = get_param(blk{k}, 'CompiledSampleTime');
                catch
                    continue
                end
                if ~iscell(st), st = {st}; end
                for j = 1:numel(st)
                    v = st{j}(1);
                    if ~(any(abs(v - P.sim.Ts_compiled_allowed) < 1e-12) || v == -1 || isinf(v) || isnan(v))
                        bad{end + 1} = sprintf('%s: %g', blk{k}, v); %#ok<AGROW>
                    end
                end
            end
        end

        function seq = seqOf(n, spec)
            % 1 x n struct array of FSM input overrides; spec rows {tick indices, field, value} applied in order
            f = unique(spec(:, 2))';
            c = [f; repmat({[]}, 1, numel(f))];
            seq = repmat(struct(c{:}), 1, n);
            for r = 1:size(spec, 1)
                for k = spec{r, 1}
                    if k <= n, seq(k).(spec{r, 2}) = spec{r, 3}; end
                end
            end
        end

        function s = signature(u)
            w = bdIsLoaded(u); load_system(u);
            b = find_system(u, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
            s = sort(strcat(b, '|', get_param(b, 'BlockType')));
            if ~w, close_system(u, 0); end
        end
    end
end

function s = local_set(s, k, f, v)
for j = k
    s(j).(f) = v;
end
end

function v = local_descent_ref(D, code, dir, thd, vmax)
% speed target magnitude by the descent cap rule (P.electronics.lim.descent from P.structure.descent): direction dir
% (+1 forward v_fwd, -1 reverse v_rev), implement row code + 1 ('raised': the mower with its deck raised, v_*_raised;
% 'min': an unknown implement, the smallest entry of all rows, the raised deck included), downhill pitch thd (deg): the
% first breakpoint at or above it (above the last: the last); an entry at or above v_top: v_top + dv_top (the trip
% speed: no cap below the top speed), else max(entry - dv, 0); then at most vmax (the stick)
if dir > 0, tab = D.v_fwd; rz = D.v_fwd_raised; else, tab = D.v_rev; rz = D.v_rev_raised; end
if strcmp(code, 'min'), row = min([tab; rz], [], 1);
elseif strcmp(code, 'raised'), row = rz;
else, row = tab(code + 1, :); end
j = find(thd <= D.theta_bp, 1);
if isempty(j), j = numel(D.theta_bp); end
cap = row(j);
if cap >= D.v_top - 1e-4, ref = D.v_top + D.dv_top; else, ref = max(cap - D.dv, 0); end
v = min(vmax, ref);
end

function j = local_descent_col(D, thd)
% the column of the descent tables for the downhill pitch thd (deg): the first breakpoint at or above it (above the
% last: the last), as the MCU lookups (2-D Lookup Table, Flat, on the negated axis)
j = find(thd <= D.theta_bp, 1);
if isempty(j), j = numel(D.theta_bp); end
end

function t = local_old_follow_trip(t10, vm, vr, md, dv, n)
% the time the speed-follow rule before the descent task (max(|vtrack_meas| - |vtrack_ref of the previous sample|) >
% dv for n samples, mode of the previous sample MANUAL..AUTO) would have set OVERSPEED on these signals (NaN if never)
e = [-inf; max(abs(vm(2:end, :)) - abs(vr(1:end - 1, :)), [], 2)];
drv = [false; md(1:end - 1) >= double(ShaddadMode.MANUAL) & md(1:end - 1) <= double(ShaddadMode.AUTO)];
c = e > dv & drv;
k = 0; t = NaN;
for j = 1:numel(c)
    if c(j), k = k + 1; else, k = 0; end
    if k >= n, t = t10(j); return, end
end
end
