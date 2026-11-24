classdef t_mcu_platform < matlab.unittest.TestCase
%T_MCU_PLATFORM  Acceptance tests of build task T08 (MCU firmware platform, docs/build_plan.md T08).
%
%   runtests('tests/electronics/t_mcu_platform.m')
%   Bench: tests/electronics/tb_mcu_rc.slx (RC_Link_Sim -> Shaddad_MCU, scripts/build/build_tb_mcu_rc.m),
%   run with mcu_tb_run; unit harnesses: mcu_harness (temporary, unsaved models).
%   Long checks (Shaddad_System update, HW variant compile) run only with the environment variable
%   T08_LONG=1. Code generation is BLOCKED while Simulink Coder is missing (D31).

    properties (Constant)
        Units = {'fw_io_in_sim', 'fw_io_in_hw', 'fw_io_out_sim', 'fw_io_out_hw', 'fw_sbus_decoder', 'fw_rc_mapping', ...
            'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', 'fw_adc_scaling', 'fw_telemetry'};
        Stubs = {'fw_mode_manager', 'fw_safety_supervisor', 'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
        Codecs = {'comp_cmd', 'mcu_status', 'mcu_imu', 'mcu_odom', 'mcu_power'};
    end

    properties
        Root
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_McuIn1k'',''var'')')
                shaddad_params();
            end
            tc.Root = currentProject().RootFolder;
            wasMcu = bdIsLoaded('Shaddad_MCU'); wasTb = bdIsLoaded('tb_mcu_rc');
            tc.addTeardown(@() t_mcu_platform.closeIfNew('tb_mcu_rc', wasTb));
            tc.addTeardown(@() t_mcu_platform.closeIfNew('Shaddad_MCU', wasMcu));
            tc.addTeardown(@() mcu_harness('close'));
        end
    end

    methods (Static)
        function closeIfNew(m, was)
            if ~was && bdIsLoaded(m), close_system(m, 0); end
        end

        function op = randomOp(T, seed)
            % random operator input on the 0.01 s grid (all sticks / switches change every sample)
            rng(seed);
            n = round(T/0.01) + 1;
            op = struct();
            for f = {'stick_thr', 'stick_steer', 'stick_aux_lv', 'stick_aux_lh', 'knob_speed'}, op.(f{1}) = 2*rand(n, 1) - 1; end
            for f = {'sw_mode', 'sw_impl', 'sw_lift', 'sw_mission'}, op.(f{1}) = randi(3, n, 1); end
            for f = {'sw_arm', 'sw_lights', 'sw_estop', 'btn_reset', 'btn_rth'}, op.(f{1}) = randi([0 1], n, 1); end
        end

        function [t, idx] = edges(x, t)
            idx = find(diff(double(x(:))) ~= 0) + 1;
            t = t(idx);
        end
    end

    methods (Test)
        % ------------------------------------------------------------------ files, ports, structure
        function testFilesAndBuildOutputs(tc)
            r = tc.Root;
            f = [{'models/Shaddad_MCU.slx', 'tests/electronics/tb_mcu_rc.slx', 'subsystems/electronics/sbus_frame_sync.m'}, ...
                cellfun(@(u) ['subsystems/electronics/' u '.slx'], [tc.Units, tc.Stubs], 'UniformOutput', false)];
            for m = tc.Codecs
                f = [f, {['subsystems/electronics/can2_' m{1} '_pack.slx'], ['subsystems/electronics/can2_' m{1} '_unpack.slx']}]; %#ok<AGROW>
            end
            for k = 1:numel(f)
                tc.verifyTrue(isfile(fullfile(r, f{k})), f{k});
                if endsWith(f{k}, '.slx')
                    d = Simulink.MDLInfo(fullfile(r, f{k})).Description;
                    tc.verifyFalse(contains(d, 'SHADDAD_STUB'), [f{k} ' is still a T01 stub']);
                    tc.verifyGreaterThan(numel(d), 40, [f{k} ' has no port description']);
                end
            end
            for s = tc.Stubs
                d = Simulink.MDLInfo(fullfile(r, 'subsystems', 'electronics', [s{1} '.slx'])).Description;
                tc.verifyTrue(startsWith(d, 'T08 STUB for T09'), [s{1} ': stub description']);
            end
            txt = fileread(fullfile(r, 'subsystems', 'electronics', 'sbus_frame_sync.m'));
            tc.verifyTrue(contains(txt, '%#codegen'), 'sbus_frame_sync.m: %#codegen');
        end

        function testInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'Shaddad_MCU'}, 'Quiet', true);
            tc.verifyEqual(n, 0, 'Shaddad_MCU root ports differ from ICD 3.1');
            % final ports of the T09 stubs (names and order); D71 (2026-11-15, lead-approved R-01 interface changes):
            % fw_mode_manager out 6 stop_ctl, fw_vehicle_ctrl in 7 limits_hold, fw_safety_supervisor in 3
            % stop_fail_prev, in 4 vtrack_ref_prev
            ports = struct( ...
                'fw_safety_supervisor', {{{'fw10', 'mode_prev', 'stop_fail_prev', 'vtrack_ref_prev'}, {'fault_word', 'warn_word', 'fault_cls', 'k_der'}}}, ...
                'fw_mode_manager', {{{'fw10', 'fault_word', 'fault_cls'}, {'mode', 'cmd_src', 'drive_enable', 'rearm_req', 'bms_req', 'stop_ctl'}}}, ...
                'fw_vehicle_ctrl', {{{'fw10', 'mode', 'cmd_src', 'drive_enable', 'fault_word', 'k_der', 'limits_hold'}, {'drive_cmd', 'vc_tel'}}}, ...
                'fw_implement_aux', {{{'fw50'}, {'do_aux', 'pwm_duty', 'impl_enable', 'impl_fault_reset', 'impl_speed_rpm'}}}, ...
                'fw_watchdog_mgr', {{{'alive', 'fault_inject', 'fault_word', 'mode', 'rearm_req'}, {'wdg_kick', 'mcu_safety_ok', 'safety_reset_req'}}});
            for s = fieldnames(ports)'
                w = bdIsLoaded(s{1}); load_system(s{1});
                ins = find_system(s{1}, 'SearchDepth', 1, 'BlockType', 'Inport');
                outs = find_system(s{1}, 'SearchDepth', 1, 'BlockType', 'Outport');
                [~, io] = sort(str2double(get_param(ins, 'Port'))); [~, oo] = sort(str2double(get_param(outs, 'Port')));
                tc.verifyEqual(get_param(ins(io), 'Name')', ports.(s{1}){1}, [s{1} ' inputs']);
                tc.verifyEqual(get_param(outs(oo), 'Name')', ports.(s{1}){2}, [s{1} ' outputs']);
                if ~w, close_system(s{1}, 0); end
            end
        end

        function testModelStructure(tc)
            m = 'Shaddad_MCU';
            load_system(m);
            ts = {'Task_1k', 'Ts.base'; 'Task_5', 'Ts.imu'; 'Task_10', 'Ts.ctrl'; 'Task_50', 'Ts.comm'; 'Task_100', 'Ts.hk'};
            for k = 1:size(ts, 1)
                b = [m '/' ts{k, 1}];
                tc.verifyEqual(get_param(b, 'TreatAsAtomicUnit'), 'on', [ts{k, 1} ' atomic']);
                tc.verifyEqual(get_param(b, 'SystemSampleTime'), ts{k, 2}, [ts{k, 1} ' rate']);
            end
            for v = {'IO_In', 'IO_Out'}
                b = [m '/' v{1}];
                tc.verifyEqual(get_param(b, 'Variant'), 'on');
                tc.verifyEqual(get_param(b, 'VariantActivationTime'), 'update diagram');
                tc.verifyEqual(get_param([b '/SIM'], 'VariantControl'), 'VC_TARGET == 0');
                tc.verifyEqual(get_param([b '/HW'], 'VariantControl'), 'VC_TARGET == 1');
            end
            rts = find_system(m, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'RateTransition');
            tc.verifyGreaterThan(numel(rts), 20);
            tc.verifyTrue(all(strcmp(get_param(rts, 'Integrity'), 'on')) && all(strcmp(get_param(rts, 'Deterministic'), 'on')), ...
                'every Rate Transition: Integrity and Deterministic on');
            mf = find_system(m, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SFBlockType', 'MATLAB Function');
            tc.verifyEqual(numel(mf), 1, 'only the SBUS frame-sync MATLAB Function (build plan T08.2)');
            tc.verifyTrue(endsWith(mf{1}, 'Frame_Sync'));
            hw = {'fw_io_in_hw', {'SBUS_UART', 'DI_GPIOF', 'ICM20948', 'ADC1_Conv', 'ADC3_TempSensor', 'TIM2_CNT', 'RCC_RSR', ...
                    'FDCAN1_PSR', 'FDCAN2_PSR', 'CAN1_RX/FDCAN_Read', 'CAN2_RX/FDCAN_Read'}; ...
                'fw_io_out_hw', {'DO_GPIOG', 'DO_GPIOE', 'PWM_TIM1', 'PWM_TIM4', 'IWDG1_refresh/IWDG1_KR', 'TX_DRV_CMD_L/FDCAN_Write', ...
                    'TX_MCU_POWER/FDCAN_Write'}};
            for k = 1:size(hw, 1)
                w = bdIsLoaded(hw{k, 1}); load_system(hw{k, 1});
                for j = 1:numel(hw{k, 2})
                    tc.verifyGreaterThan(getSimulinkBlockHandle([hw{k, 1} '/' hw{k, 2}{j}]), 0, [hw{k, 1} '/' hw{k, 2}{j}]);
                end
                if k == 1
                    tc.verifyEqual(get_param([hw{k, 1} '/SBUS_UART'], 'UARTModule'), 'USART2');
                    tc.verifyEqual(get_param([hw{k, 1} '/SBUS_UART'], 'DataLength'), '32');
                    tc.verifyEqual(get_param([hw{k, 1} '/ICM20948'], 'I2CAddress'), '0x69');
                    tc.verifyEqual(get_param([hw{k, 1} '/ICM20948'], 'AccelerometerRange'), '+/-4g');
                    tc.verifyEqual(get_param([hw{k, 1} '/ICM20948'], 'GyroscopeRange'), '+/-500 dps');
                end
                if ~w, close_system(hw{k, 1}, 0); end
            end
        end

        function testUpdateRatesLoops(tc)
            m = 'Shaddad_MCU';
            load_system(m);
            set_param(m, 'SimulationCommand', 'update');
            tc.verifyEqual(numel(Simulink.BlockDiagram.getAlgebraicLoops(m)), 0, 'Shaddad_MCU algebraic loops');
            load_system('tb_mcu_rc');
            tc.verifyEqual(numel(Simulink.BlockDiagram.getAlgebraicLoops('tb_mcu_rc')), 0, 'tb_mcu_rc algebraic loops');
            % compiled rates of every block inside Shaddad_MCU are in the allowed set
            P = evalin('base', 'P');
            feval(m, [], [], [], 'compile');
            c = onCleanup(@() feval(m, [], [], [], 'term'));
            blk = find_system(m, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
            bad = {};
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
                        bad{end+1} = sprintf('%s: %g', blk{k}, v); %#ok<AGROW>
                    end
                end
            end
            tc.verifyEmpty(bad, strjoin(bad, newline));
        end

        function testSystemUpdate(tc)
            tc.assumeTrue(strcmp(getenv('T08_LONG'), '1'), 'Long check (about 4 min): set T08_LONG=1');
            tc.assumeEqual(evalin('base', 'VC_TARGET'), 0);
            w = bdIsLoaded('Shaddad_System');
            load_system('Shaddad_System');
            if ~w, tc.addTeardown(@() close_system('Shaddad_System', 0)); end
            msg = '';
            for attempt = 1:2                      % R2026b Stateflow JIT relink assertion on the first compile
                try
                    set_param('Shaddad_System', 'SimulationCommand', 'update');
                    msg = ''; break
                catch e
                    msg = getReport(e, 'extended', 'hyperlinks', 'off');
                end
            end
            if ~isempty(msg)
                mcuInvolved = contains(msg, 'Shaddad_System/MCU') || contains(msg, 'Shaddad_MCU');
                tc.verifyFalse(mcuInvolved, ['Shaddad_MCU involved in the Shaddad_System update error: ' msg]);
                tc.assumeFail(['BLOCKED: Shaddad_System update fails outside Shaddad_MCU: ' msg(1:min(400, end))]);
            end
        end

        % ------------------------------------------------------------------ SBUS decoder (bench)
        function testTEL01_BitExact2000(tc)
            P = evalin('base', 'P'); P.rc.p_loss = 0;
            T = 28.6;
            r = mcu_tb_run(T, t_mcu_platform.randomOp(T, 801), struct(), P);
            a = mcu_sbus_analyze(r);
            tc.log(1, sprintf('T-EL-01: %d frames, wrong %d, missed %d, spurious %d, frame_new - last byte %d..%d ms', ...
                a.nFrameNew, a.nWrong, a.nMissed, a.nSpurious, min(a.lat_lastbyte_ms), max(a.lat_lastbyte_ms)));
            tc.verifyGreaterThanOrEqual(a.nFrameNew, 2000);
            tc.verifyEqual(a.nWrong, 0); tc.verifyEqual(a.nMissed, 0); tc.verifyEqual(a.nSpurious, 0);
            tc.verifyEqual(a.nFrameNew, a.nBurst);
            % frame_new at the 1 ms tick that delivers the last byte: <= 1 ms after its line completion (<= 4.0 ms)
            tc.verifyLessThanOrEqual(max(a.lat_lastbyte_ms) + 1, 4.0);
            tc.verifyEqual(double(r.dec.n_bad(end)), 0);
        end

        function testTEL02_PhaseDriftDrops(tc)
            T = 28.6;
            op = t_mcu_platform.randomOp(T, 802);
            cases = [500 0.63e-3; -500 0.12e-3];
            for k = 1:2
                P = evalin('base', 'P'); P.rc.p_loss = 0;
                P.rc.uart.p_drop = 1/(50*25);                     % one dropped byte per 50 frames
                P.rc.seed_drop = P.rc.seed_drop + 17;             % realisation with 40 drops in 2042 frames (1/51)
                P.rc.uart.drift_ppm = cases(k, 1); P.rc.uart.phase0 = cases(k, 2);
                r = mcu_tb_run(T, op, struct(), P);
                a = mcu_sbus_analyze(r);
                okFrac = a.nFrameNew/a.nBurst;
                tc.log(1, sprintf(['T-EL-02 drift %+d ppm phase %.2f ms: frames %d, damaged %d (%.2f %%), n_ok %.2f %%, ' ...
                    'wrong %d, missed %d, resync %d, payloads with 0x0F %d / 0x00 %d'], cases(k, 1), 1e3*cases(k, 2), ...
                    a.nBurst, a.nDamaged, 100*a.nDamaged/a.nBurst, 100*okFrac, a.nWrong, a.nMissed, a.resync_max, a.n0F, a.n00));
                tc.verifyGreaterThan(a.n0F, 0); tc.verifyGreaterThan(a.n00, 0);
                tc.verifyGreaterThan(a.nDamaged, 0);
                tc.verifyEqual(a.nWrong, 0, 'wrong frames');
                tc.verifyEqual(a.nSpurious, 0);
                tc.verifyEqual(a.nMissed, 0, 'intact frames lost (collateral loss)');
                tc.verifyLessThanOrEqual(a.resync_max, 1, 'resync <= 1 frame');
                tc.verifyGreaterThanOrEqual(okFrac, 0.98 - 1e-12, 'n_ok >= 98 %');
            end
        end

        function testTEL03_ByteErrors(tc)
            % SBUS stream -> fw_sbus_decoder (decchain harness): no corrupted frame may be accepted (S6c).
            % Chunks [9 8 8] per frame, one frame every 14 ms. Damaged frames: bit flip + uart_err in chunk 1
            % (header chunk: the revision-B defect), chunk 2, chunk 3, corrupted header, uart_err without a
            % visible flip, channel out of range (high / low, clean UART), trailing byte after the footer.
            M = 40; rng(803);
            ch = 172 + floor(rand(M, 16)*(1811 - 172 + 1));
            ch(15, 3) = 2047; ch(18, 1) = 100;                       % implausible channels, no UART error
            fr = sbus_ref_encode(ch, false(M, 4), zeros(M, 1));
            st = 5 + 14*(0:M-1)'; N = st(end) + 20;
            [b, l, er] = mcu_sbus_stream(fr, st, N);
            flip = [3 3 2; 6 12 7; 9 22 4; 12 1 0];                  % frame, byte, bit
            for i = 1:size(flip, 1)
                m = flip(i, 1); j = flip(i, 2);
                c = 1 + (j > 9) + (j > 17);                         % chunk of byte j ([9 8 8])
                kk = j - [0 9 17]; kk = kk(c);                      % position inside the chunk
                b(st(m) + c - 1, kk) = bitxor(b(st(m) + c - 1, kk), uint8(2^flip(i, 3)));
                er(st(m) + c - 1) = true;
            end
            er(st(24) + 1) = true;                                  % flagged chunk, bit flip not visible
            l(st(21) + 2) = l(st(21) + 2) + 1;                      % trailing byte 0x00 after the footer
            bad = [3 6 9 12 15 18 21 24];
            t = (0:N-1)'*1e-3;
            ds = mcu_harness('ds', {'bytes', timeseries(b, t)}, {'len', timeseries(l, t)}, {'uart_err', timeseries(er, t)});
            o = mcu_harness('sim', 'decchain', t(end), ds);
            dec = o.logsout.get('rc_dec').Values;
            fn = find(squeeze(dec.frame_new.Data)); tf = dec.frame_new.Time(fn);
            cr = double(squeeze(dec.ch_raw.Data)); if size(cr, 1) == 16 && size(cr, 2) ~= 16, cr = cr.'; end
            good = setdiff(1:M, bad);
            tLast = (st + 1)*1e-3;                                  % time of the tick with the last byte
            tc.verifyEqual(numel(fn), numel(good), 'frame_new count = intact frames');
            acc = arrayfun(@(x) find(abs(tLast - x) < 1e-9, 1), tf);
            tc.verifyEqual(acc(:), good(:), 'accepted frames = intact frames, on the tick of their last byte');
            nWrong = 0;
            for i = 1:numel(fn)
                if ~isequal(cr(fn(i), :), ch(acc(i), :)), nWrong = nWrong + 1; end
            end
            tc.verifyEqual(nWrong, 0, 'accepted channels equal the transmitted channels');
            nb = double(squeeze(dec.n_bad.Data));
            tc.verifyEqual(nb(end), numel(bad), 'n_bad = damaged frames (one count per frame)');
            tc.log(1, sprintf('T-EL-03: %d frames, %d damaged rejected (n_bad %d), %d accepted, %d wrong', ...
                M, numel(bad), nb(end), numel(fn), nWrong));
        end

        function testTEL04_Sbus2Footers(tc)
            for ft = [4 20 36 52 1]
                P = evalin('base', 'P'); P.rc.p_loss = 0; P.electronics.sbus.footer = ft;
                r = mcu_tb_run(1.0, struct('stick_thr', [0 0.3]), struct(), P);
                a = mcu_sbus_analyze(r);
                nTx = a.nBurst;
                if ft == 1     % invalid footer: every frame rejected
                    tc.verifyEqual(double(r.dec.n_ok(end)), 0, 'footer 0x01 must be rejected');
                    tc.verifyEqual(double(r.dec.n_bad(end)), nTx);
                else
                    tc.verifyEqual(double(r.dec.n_ok(end)), nTx, sprintf('footer 0x%02X', ft));
                    tc.verifyEqual(a.nWrong, 0);
                end
                tc.log(1, sprintf('T-EL-04 footer 0x%02X: frames %d, n_ok %d, n_bad %d', ft, nTx, r.dec.n_ok(end), r.dec.n_bad(end)));
            end
        end

        function testTEL05_LinkLost(tc)
            P = evalin('base', 'P'); P.rc.p_loss = 0;
            r = mcu_tb_run(1.6, struct('tx_power_on', [0 1; 1.0 0]), struct(), P);
            t = r.t1;
            fv = find(r.dec.frame_valid);
            tLast = t(fv(find(t(fv) < 1.1, 1, 'last')));
            kLost = find(r.dec.rc_link_lost & t > tLast, 1);
            dt = t(kLost) - tLast;
            tc.log(1, sprintf('T-EL-05: last valid frame %.3f s, rc_link_lost %.3f s -> %.1f ms', tLast, t(kLost), 1e3*dt));
            tc.verifyEqual(dt, 0.100, 'AbsTol', 0.002 + 1e-9);
            tc.verifyFalse(any(r.dec.rc_ok(kLost:end)));
            % before the loss the link was up continuously after the first valid frame
            k1 = fv(1);
            tc.verifyFalse(any(r.dec.rc_link_lost(k1:find(t >= 1.0, 1))));
        end

        function testTEL06_LinkQuality(tc)
            P = evalin('base', 'P'); P.rc.p_loss = 0.2;
            r = mcu_tb_run(60, struct(), struct(), P);
            w = r.t1 >= 10;
            lq = mean(r.dec.lq(w));
            fn = find(r.dec.frame_new & w);
            lossFrac = mean(r.dec.frame_lost(fn));
            tc.log(1, sprintf('T-EL-06: p_loss 0.2 -> mean LQ %.4f (10..60 s), lost-frame fraction %.4f, telem rc_lq %.4f', ...
                lq, lossFrac, mean(r.telem.rc_lq(r.t10 >= 10))));
            tc.verifyEqual(lq, 0.80, 'AbsTol', 0.03);
            tc.verifyEqual(mean(r.telem.rc_lq(r.t10 >= 10)), lq, 'AbsTol', 0.01);
        end

        % ------------------------------------------------------------------ RC mapping
        function testTEL07_Expo(tc)
            raw = (172:1811)'; N = numel(raw); t = (0:N-1)'*0.01;
            ch = repmat(uint16(992), N, 16); ch(:, 1) = uint16(raw); ch(:, 2) = uint16(flipud(raw));
            ds = mcu_harness('ds', {'ch_raw', timeseries(ch, t)}, {'frame_lost', timeseries(false(N, 1), t)}, ...
                {'failsafe', timeseries(false(N, 1), t)}, {'last_valid', timeseries(true(N, 1), t)});
            o = mcu_harness('sim', 'rcmap', t(end), ds);
            rc = o.logsout.get('rc').Values;
            P = evalin('base', 'P'); R = P.electronics.rcmap;
            y = @(x, e) e*x.^3 + (1 - e)*x;
            db = @(x) sign(x).*max(abs(x) - R.deadband, 0)/(1 - R.deadband);
            x1 = min(max(R.sign(1)*(double(ch(:, 1)) - R.raw_mid(1))/R.raw_half, -1), 1);
            x2 = min(max(R.sign(2)*(double(ch(:, 2)) - R.raw_mid(2))/R.raw_half, -1), 1);
            e1 = max(abs(double(squeeze(rc.steer.Data)) - y(db(x1), R.expo_steer)));
            e2 = max(abs(double(squeeze(rc.thr.Data)) - y(db(x2), R.expo_thr)));
            % speed scale, sign convention and v / w at full stick
            tc.log(1, sprintf('T-EL-07: max expo error steer %.3g, thr %.3g over %d raw values', e1, e2, N));
            tc.verifyLessThan(e1, 1e-6); tc.verifyLessThan(e2, 1e-6);
            s = double(squeeze(rc.speed_scale.Data)); w = double(squeeze(rc.w_rc.Data)); st = double(squeeze(rc.steer.Data));
            tc.verifyEqual(s, (R.s_min + 0.5*(1 - R.s_min))*ones(N, 1), 'AbsTol', 1e-6);
            tc.verifyEqual(w, R.steer_to_omega_sign*P.electronics.lim.w_max_man*st.*s, 'AbsTol', 1e-5);
        end

        function testTEL08_Debounce(tc)
            % (a) harness: glitches of 1..3 Task_10 samples rejected, a step accepted after debounce_n samples agree
            P = evalin('base', 'P'); n = P.electronics.rcmap.debounce_n;
            N = 120; t = (0:N-1)'*0.01;
            ch = repmat(uint16(992), N, 16); md = repmat(uint16(172), N, 1);     % mode pos 1 (MANUAL)
            md(11) = 1811; md(31:32) = 1811; md(51:53) = 1811; md(81:end) = 992;   % glitches 1/2/3 samples, step to pos 2
            ch(:, P.electronics.rcmap.ch.mode) = md;
            ds = mcu_harness('ds', {'ch_raw', timeseries(ch, t)}, {'frame_lost', timeseries(false(N, 1), t)}, ...
                {'failsafe', timeseries(false(N, 1), t)}, {'last_valid', timeseries(true(N, 1), t)});
            o = mcu_harness('sim', 'rcmap', t(end), ds);
            sw = double(squeeze(o.logsout.get('rc').Values.sw_mode.Data));
            tc.verifyFalse(any(sw(1:80) == 3), 'glitch accepted');
            tc.verifyTrue(all(sw(n:80) == 1), 'MANUAL accepted after debounce_n samples, held through the glitches');
            k = 80 + find(sw(81:end) == 2, 1);
            tc.verifyEqual((k - 81)*0.01, (n - 1)*0.01, 'AbsTol', 1e-9, 'step accepted when debounce_n samples agree');
            % (b) bench: TX switch steps; frame carrying the new position -> debounced position 40 +/- 10 ms
            Pb = P; Pb.rc.p_loss = 0;
            tev = (0.5:0.37:7.5)'; val = 1 + mod((1:numel(tev))', 3);   % steps 2, 3, 1, 2, ... (each a change)
            tab = [0 1];
            for j = 1:numel(tev)
                tab = [tab; tev(j) val(j)]; %#ok<AGROW>
                if j < numel(tev)
                    g = mod(val(j), 3) + 1;                          % other position for one 10 ms operator sample
                    tab = [tab; tev(j) + 0.15 g; tev(j) + 0.16 val(j)]; %#ok<AGROW>
                end
            end
            r = mcu_tb_run(8.0, struct('sw_mode', tab), struct(), Pb);
            raw5 = double(r.dec.ch_raw(:, P.electronics.rcmap.ch.mode));
            posRaw = 1 + (raw5 > 992 - 0.5*819.5) + (raw5 > 992 + 0.5*819.5);
            fnew = find(r.dec.frame_new);
            swd = double(r.rc.sw_mode);
            [tStep, iStep] = t_mcu_platform.edges(swd, r.t10);
            first = tStep < 0.4;                                     % debounced IC (HOLD) -> first TX position
            tStep = tStep(~first); iStep = iStep(~first);
            lat = nan(numel(tStep), 1);
            for j = 1:numel(tStep)
                newPos = swd(iStep(j));
                % first frame carrying newPos in the run of frames that led to acceptance
                kf = fnew(r.t1(fnew) <= tStep(j) + 1e-9);
                kk = numel(kf);
                while kk > 1 && posRaw(kf(kk - 1)) == newPos, kk = kk - 1; end
                lat(j) = tStep(j) - r.t1(kf(kk));
            end
            nGl = sum(diff(tab(:, 1)) < 0.02);
            tc.log(1, sprintf('T-EL-08: %d steps accepted after %.0f..%.0f ms (frame -> debounced), %d one-sample operator glitches', ...
                numel(tStep), 1e3*min(lat), 1e3*max(lat), nGl));
            tc.verifyEqual(numel(tStep), numel(tev), 'every step accepted once, no glitch accepted');
            tc.verifyEqual(swd(iStep), val, 'accepted positions');
            tc.verifyGreaterThanOrEqual(min(lat), 0.030 - 1e-9); tc.verifyLessThanOrEqual(max(lat), 0.050 + 1e-9);
        end

        function testRemoteEstopValidFramesOnly(tc)
            % SBUS stream -> fw_sbus_decoder -> RT -> fw_rc_mapping: CH12 = STOP in failsafe / lost frames (forced)
            M = 20; ch = repmat(992, M, 16); fl = false(M, 4);
            ch(:, 12) = 172; ch(11:16, 12) = 1811;
            fl(11, 4) = true; fl(12, 3) = true; fl(13:15, 4) = true; fl(17, 4) = true;   % 17: failsafe with RUN
            fr = sbus_ref_encode(ch, fl, zeros(M, 1));
            st = 5 + 14*(0:M-1)'; N = st(end) + 20;
            [b, l, er] = mcu_sbus_stream(fr, st, N);
            t = (0:N-1)'*1e-3;
            ds = mcu_harness('ds', {'bytes', timeseries(b, t)}, {'len', timeseries(l, t)}, {'uart_err', timeseries(er, t)});
            o = mcu_harness('sim', 'decchain', t(end), ds);
            dec = o.logsout.get('rc_dec').Values; rc = o.logsout.get('rc').Values;
            fn = find(squeeze(dec.frame_new.Data)); tf = dec.frame_new.Time(fn);
            es = squeeze(rc.estop_remote.Data); t10 = rc.estop_remote.Time;
            tc.verifyEqual(numel(fn), M);
            tc.verifyFalse(any(es(t10 < tf(16))), 'STOP in failsafe / frame-lost frames changed the remote E-stop');
            kOn = find(es, 1);
            tc.verifyGreaterThanOrEqual(t10(kOn), tf(16) - 1e-9);
            tc.verifyLessThan(t10(kOn) - tf(16), 0.010 + 1e-9, 'valid STOP frame -> E-stop within one Task_10 sample');
            tc.verifyTrue(all(es(t10 >= t10(kOn) & t10 < tf(18))), 'held through the failsafe frame 17 (RUN)');
            tc.verifyFalse(any(es(t10 >= tf(18) + 0.010)), 'valid RUN frame releases the remote E-stop');
            tc.log(1, sprintf('Remote E-stop: valid STOP frame at %.3f s -> E-stop at %.3f s; failsafe / lost STOP frames ignored', ...
                tf(16), t10(kOn)));
        end

        % ------------------------------------------------------------------ RC -> DRV_CMD latency (D37)
        function testD37_Latency(tc)
            rng(37);
            nev = 200;
            iv = round((0.25 + 0.1*rand(nev, 1))/0.01)*0.01;
            tev = 0.3 + cumsum(iv);
            val = mod((1:nev)', 2)*0.5;
            P = evalin('base', 'P'); P.rc.p_loss = 0;
            r = mcu_tb_run(tev(end) + 0.3, struct('stick_thr', [0 0; tev val]), struct(), P);
            c2 = double(r.ch_raw_tx(:, P.electronics.rcmap.ch.thr));
            k = find(diff(c2) ~= 0) + 1;
            k = k(c2(k - 1) ~= 0);                                 % ignore the first frame after the IC
            tTx = r.t1(k) - P.rc.frame_period;                     % rc_frame is aligned with the release (k+1)*Tf
            w = double(typecast(reshape(r.drvL(:, 3:4)', [], 1), 'int16'));
            tDrv = r.t10(find(diff(w) ~= 0) + 1);
            lat = arrayfun(@(x) tDrv(find(tDrv >= x - 1e-9, 1)) - x, tTx);
            tc.log(1, sprintf('D37: %d TX samples -> DRV_CMD change %.0f..%.0f ms (mean %.1f)', numel(lat), ...
                1e3*min(lat), 1e3*max(lat), 1e3*mean(lat)));
            tc.verifyEqual(numel(lat), nev);
            tc.verifyGreaterThanOrEqual(min(lat), 0.018 - 1e-9); tc.verifyLessThanOrEqual(max(lat), 0.028 + 1e-9);
        end

        % ------------------------------------------------------------------ CAN codecs (T-EL-15)
        function testTEL15_Can2RoundTrip(tc)
            for m = tc.Codecs
                r = mcu_can2_roundtrip(m{1}, 400);
                fn = setdiff(fieldnames(r), {'frame'});
                bad = fn(cellfun(@(f) ~r.(f).ok, fn));
                tc.verifyEmpty(bad, sprintf('can2_%s round trip: %s', m{1}, strjoin(bad', ', ')));
                tc.verifyTrue(r.frame.id_ok && r.frame.len_ok && r.frame.pad_zero && r.frame.valid, ['can2_' m{1} ' frame header']);
                sc = fn(cellfun(@(f) r.(f).tol > 0 && r.(f).tol < 1, fn));
                tc.log(1, sprintf('T-EL-15 can2_%s: %d fields OK (singles / integers bit-exact; scaled: %s)', m{1}, numel(fn), ...
                    strjoin(cellfun(@(f) sprintf('%s %.3g <= %.3g', f, r.(f).maxErr, r.(f).tol), sc, 'UniformOutput', false)', ', ')));
            end
        end

        function testTEL15_Can1RoundTrip(tc)
            r = mcu_can1_roundtrip(400);
            fn = fieldnames(r);
            bad = fn(cellfun(@(f) ~r.(f).ok, fn));
            tc.verifyEmpty(bad, ['CAN1 round trip: ' strjoin(bad', ', ')]);
            tc.log(1, sprintf('T-EL-15 CAN1: %d checks OK (drive cmd / fb, implement cmd / fb, BMS request / frames); BMS valid drops %.2f s after the last frame', ...
                numel(fn), r.bms_timeout.maxErr));
        end

        % ------------------------------------------------------------------ IMU tilt (D47)
        function testTiltStatic(tc)
            r = mcu_tilt_case(atan2(1.700, 9.639), @(t) 0, 5);
            tc.log(1, sprintf('Tilt: accel [-1.700 0 9.639] -> pitch %.4f rad, roll %.2g', r.pitch(end), r.roll(end)));
            tc.verifyEqual(r.pitch(end), 0.1745, 'AbsTol', 0.005);
            tc.verifyEqual(r.roll(end), 0, 'AbsTol', 1e-3);
        end

        function testTiltRampAcceleration(tc)
            th = -20*pi/180;                                         % climbing a 20 deg ramp (nose up)
            prof = @(t) (t >= 3).*min(0.8, 5*max(t - 3, 0));        % 0.8 m/s^2 with the 5 m/s^3 MCU jerk limit
            r = mcu_tilt_case(th, prof, 6.5);
            e = rad2deg(r.pitch - th);
            P = evalin('base', 'P'); P.electronics.safety.tilt_comp.enable = false;
            ru = mcu_tilt_case(th, prof, 6.5, P);
            eu = rad2deg(ru.pitch - th);
            wSus = r.t5 >= 3.5;                                      % sustained acceleration (onset transient excluded)
            tc.log(1, sprintf(['Tilt ramp 20 deg, 0.8 m/s^2: compensated error %.2f deg (end), max %.2f deg after the first ' ...
                '0.5 s, onset peak %.2f deg; uncompensated %.2f deg; a_track %.3f m/s^2; tilt_gate false during accel %d'], ...
                abs(e(end)), max(abs(e(wSus))), max(abs(e(r.t5 >= 3))), abs(eu(end)), r.a_track(end), ~any(r.tilt_gate(wSus))));
            tc.verifyLessThanOrEqual(max(abs(e(wSus))), 1.0);
            tc.verifyGreaterThan(abs(eu(end)), 4.0);
            tc.verifyEqual(r.a_track(end), 0.8, 'AbsTol', 0.01);
            tc.verifyFalse(any(r.tilt_gate(wSus)), 'tilt_gate must be false while |a_track| >= gate_a');
            tc.verifyTrue(all(r.tilt_gate(r.t5 > 1 & r.t5 < 3)), 'tilt_gate true at rest');
        end

        function testGyroBias(tc)
            b = 0.02;
            r = mcu_tilt_case(0, @(t) (t >= 10)*0.5, 12, [], b);
            k10 = find(r.t5 >= 10, 1);
            P = evalin('base', 'P');
            expect = b*(1 - exp(-10/P.electronics.yaw.tau_bias));
            tc.verifyEqual(r.gyro_bias(k10), expect, 'AbsTol', 1e-3);
            tc.verifyEqual(r.yaw_rate_meas(k10), b - r.gyro_bias(k10), 'AbsTol', 1e-6);
            tc.verifyEqual(r.gyro_bias(end), r.gyro_bias(find(r.t5 >= 10.2, 1)), 'AbsTol', 1e-6, 'bias held while moving');
        end

        function testGyroBiasPivot(tc)
            % a pivot turn (tracks counter-rotating, mean track speed 0) is not a standstill: the turn rate
            % must not enter the gyro bias (live drive 2026-10-27: -0.4 rad/s yaw-rate error after pivots)
            b = 0.02;  w = 0.5;
            r = mcu_tilt_case(0, @(t) 0, 12, [], b, [5 10 0.3 w]);
            k5 = find(r.t5 >= 5, 1);  k10 = find(r.t5 >= 9.95, 1);
            tc.log(1, sprintf('Pivot 5-10 s, w = %.2f rad/s: gyro_bias %.4f -> %.4f, yaw_rate_meas at 9.95 s %.4f', ...
                w, r.gyro_bias(k5), r.gyro_bias(k10), r.yaw_rate_meas(k10)));
            tc.verifyEqual(r.gyro_bias(k10), r.gyro_bias(k5), 'AbsTol', 5e-3, 'bias held during the pivot');
            tc.verifyEqual(r.yaw_rate_meas(k10), w, 'AbsTol', 0.02, 'yaw rate = turn rate during the pivot');
        end

        % ------------------------------------------------------------------ platform signals (bench)
        function testBusOffInjection(tc)
            P = evalin('base', 'P'); P.rc.p_loss = 0;
            r = mcu_tb_run(1.5, struct(), struct('can1_busoff', [0 0; 0.5 1], 'can2_busoff', [0 0; 1.0 1]), P);
            b = logical(r.busoff);
            t1 = r.t10(find(b(:, 1), 1)); t2 = r.t10(find(b(:, 2), 1));
            tc.log(1, sprintf('Bus-off (SIM fault_inject b4/b5): CAN1 bit at %.3f s, CAN2 bit at %.3f s', t1, t2));
            tc.verifyFalse(any(b(r.t10 < 0.5, 1))); tc.verifyFalse(any(b(r.t10 < 1.0, 2)));
            tc.verifyLessThanOrEqual(t1 - 0.5, 0.020 + 1e-9); tc.verifyLessThanOrEqual(t2 - 1.0, 0.020 + 1e-9);
            tc.verifyTrue(all(b(r.t10 >= 0.53, 1))); tc.verifyTrue(all(b(r.t10 >= 1.03, 2)));
        end

        function testTelemetryStatusFrames(tc)
            P = evalin('base', 'P'); P.rc.p_loss = 0;
            r = mcu_tb_run(1.5, struct('stick_thr', [0 0; 0.5 0.5], 'sw_mode', [0 3]), struct(), P);
            k = numel(r.t10);
            tc.verifyEqual(double(r.telem.mode(k)), 2);                         % stub mode manager: STANDBY
            tc.verifyEqual(double(r.telem.rc_ch_norm(k, :)), double(r.rc.ch_norm(k, :)), 'AbsTol', 1e-6);
            tc.verifyEqual(double(r.telem.sbus_ok(k)), double(r.dec.n_ok(find(r.t1 <= r.t10(k) + 1e-9, 1, 'last'))));
            tc.verifyTrue(logical(r.telem.rc_link_ok(k)));
            % MCU_STATUS (0x200) payload decoded by hand: t_ms, mode, rc_mode_sw (AUTO = 2), flags b0 rc_link_ok, rc_lq
            d = r.status(end, :);
            tms = double(typecast(uint8(d(1:4)), 'uint32'));
            tc.verifyEqual(tms, round(1e3*r.t50(end)), 'AbsTol', 1, 't_ms');
            tc.verifyEqual(double(d(5)), 2); tc.verifyEqual(double(d(7)), 2);
            tc.verifyEqual(bitand(double(d(10)), 1), 1, 'rc_link_ok flag');
            tc.verifyEqual(double(d(43))/100, r.telem.rc_lq(end), 'AbsTol', 0.006);
            tc.verifyEqual(double(r.status(end, 12)) - double(r.status(end - 1, 12)), 1, 'heartbeat +1 per frame');
            % BMS_REQ alive +1 per 50 ms frame (byte 1)
            tc.verifyTrue(all(mod(diff(double(r.bmsreq(:, 2))), 256) == 1));
            % DO word: stub watchdog DO0 = 1, DO1 = 0, implement stub bits 8 and 9
            tc.verifyEqual(r.do_word(end), 1 + 2^8 + 2^9);
        end

        function testAdcScaling(tc)
            P = evalin('base', 'P'); a = P.electronics.adc;
            Tk = 318.15;
            cnt = round([51.2/a.k_vpack 51.2/a.k_vlink a.i_off + 10*a.i_sens 24/a.k_v24 12/a.k_v12 5/a.k_v5 0 0.5*a.vref] ...
                /a.vref*65535);
            cnt(7) = round(interp1(a.ntc_lut_K, a.ntc_lut_counts, Tk));
            t = (0:0.05:0.2)';
            ds = mcu_harness('ds', {'adc_raw', timeseries(repmat(uint16(cnt), numel(t), 1), t)}, ...
                {'mcu_temp', timeseries(single(40*ones(size(t))), t)});
            o = mcu_harness('sim', 'adc', t(end), ds);
            blk = arrayfun(@(k) o.yout{k}.BlockPath.getBlock(1), 1:o.yout.numElements, 'UniformOutput', false);
            g = @(n) double(o.yout{find(endsWith(blk, ['/' n]), 1)}.Values.Data(end));
            lsb = a.vref/65535;
            tc.verifyEqual(g('V_pack'), 51.2, 'AbsTol', a.k_vpack*lsb);
            tc.verifyEqual(g('I_pack'), 10, 'AbsTol', lsb/a.i_sens);
            tc.verifyEqual(g('V24'), 24, 'AbsTol', a.k_v24*lsb); tc.verifyEqual(g('V5'), 5, 'AbsTol', a.k_v5*lsb);
            tc.verifyEqual(g('T_encl'), Tk, 'AbsTol', 0.2);
            tc.verifyEqual(g('lift_pos'), 0.5, 'AbsTol', 1e-4); tc.verifyEqual(g('T_mcu'), 313.15, 'AbsTol', 1e-3);
        end

        % ------------------------------------------------------------------ code generation / HW
        function testCodegenBlocked(tc)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
            vc = evalin('base', 'VC_TARGET');
            c = onCleanup(@() assignin('base', 'VC_TARGET', vc));
            assignin('base', 'VC_TARGET', 1);
            shaddad_configsets('codegen');
            load_system('Shaddad_MCU');
            ref = getConfigSet('Shaddad_MCU', 'Shaddad_Ref'); src = ref.SourceName;
            c2 = onCleanup(@() set_param(ref, 'SourceName', src));
            ref.SourceName = 'cs_GenARM'; ref.refresh;
            slbuild('Shaddad_MCU');
        end

        function testHwVariantCompile(tc)
            tc.assumeTrue(strcmp(getenv('T08_LONG'), '1'), ['Long check (about 5 min): set T08_LONG=1. Measured 2026-10-22: ' ...
                'BLOCKED - Register Read//Write needs an STM32CubeCLT installation (D31 / T17)']);
            vc = evalin('base', 'VC_TARGET');
            c = onCleanup(@() assignin('base', 'VC_TARGET', vc));
            assignin('base', 'VC_TARGET', 1);
            load_system('Shaddad_MCU');
            try
                set_param('Shaddad_MCU', 'SimulationCommand', 'update');
            catch e
                msg = getReport(e, 'extended', 'hyperlinks', 'off');
                tc.assumeFalse(contains(msg, 'STM32CubeCLT'), ['BLOCKED: ' msg(1:min(300, end))]);
                tc.verifyFail(msg);
            end
        end
    end
end
