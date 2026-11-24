classdef t_t07_mcu < matlab.unittest.TestCase
%T_T07_MCU  Card T-07 step 7: the MCU firmware's missing tests (T-07 pass B) and the run of finding T07-MODES-01.
%
%   runtests('t_t07_mcu')
%
%   testModes01_DrvCmdTimeoutReset   Shaddad_System: the robot armed in MANUAL, DRV_CMD_L alone dropped for 0.2 s
%                                    (ScenarioIn.flt.can1_drop_mask bit 0), then the RC reset (CH9) and a new arm.
%                                    It states the expected safe behaviour (REQ-SYS-011, -002, -064, -078: a fault
%                                    stops the robot; once its cause is gone, a deliberate reset from valid RC frames
%                                    and a new arm command bring it back) and records what the run does in
%                                    work/team/t07/logs/t07_modes01_run.json. If the robot stays in FAULT with the
%                                    loop open, it fails with a message that cites T07-MODES-01 (a finding, not a fix).
%   testMissionCommands_CH13_CH14    fw_rc_mapping (mcu_harness 'rcmap'): CH13 to position 2 gives START and to 3
%                                    PAUSE after the debounce, CH14's rise gives RTH; a glitch and the return to
%                                    position 1 give none; rc_aux_cmd holds the last code, rc_aux_seq counts events.
%   testStopS6 / S7 / S11 / S12      mcu_mode_manager.m stepped directly (as t_mcu_control/mmRun): the stop phases of
%                                    FSM-MCU-STOP, observed on the outputs (drive_enable: a deceleration phase;
%                                    stop_ctl(1): a park; neither in ESTOP: ESTOP.SAFE).
%   testModeM23_DefensiveDefault     FSM-MCU-MODE M23 cannot be reached through the inputs (the state is persistent);
%                                    checked statically: the default branch goes to FAULT and every other state has a
%                                    branch of its own.
%   testMcuStatusInFirmware          fw_can2_codec as built (in-memory harness of t_t07_mcu_sil): the status bus and
%                                    t_ms to the can2_status frame, every field against the ICD section 5 table of
%                                    t_t07_frames, the identifier, the length and the zero padding.
%   No model file is saved; the harness models are closed with close_system(name, 0).

    properties
        Root
    end

    properties (Constant)
        DT = 0.01;                  % the mode manager's tick (Task_10)
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.Root = char(currentProject().RootFolder);
            if ~evalin('base', 'exist(''P'', ''var'') && exist(''Ts'', ''var'') && exist(''Bus_McuIn10'', ''var'')')
                shaddad_params();
            end
        end
    end

    methods (Test, TestTags = {'T07'})
        % ================================================================== T07-MODES-01, a system run
        function testModes01_DrvCmdTimeoutReset(tc)
            P = evalin('base', 'P');
            if P.world.id ~= 1
                P = shaddad_params('World', 1, 'Implement', 0, 'Phase', 1, 'Assign', false);
            end
            P = scn_integration_overrides(P);
            T = 11.0; tDrop = 4.0; tReset = 6.0;
            tab = scn_rc_tab(0.5, 2.0);                                 % TX on 0.5 s, arm 2.0 s: MANUAL, standing
            tab.flt.can1_drop_mask = [0 0; tDrop 2^P.electronics.can.drop_bit.drv_cmd_L; tDrop + 0.2 0];
            tab.op.btn_reset = [0 0; tReset 1; tReset + 0.3 0];        % RC CH9 reset, 0.3 s
            tab.op.sw_arm = [0 0; 2.0 1; 7.0 0; 8.0 1];                % a new arm command after the reset
            res = run_scenario('T07M1', 'Input', tab, 'StopTime', T, 'P', P, 'Save', false);
            tc.assertTrue(res.ok && ~isempty(res.S), ['the Shaddad_System run failed: ' res.msg]);
            S = res.S;
            m = S.telem.mode; fw = S.telem.fault_word; dw = S.mcu_out1k.do_word;
            at = @(x, t) double(x.v(find(x.t <= t + 1e-9, 1, 'last'), :));
            bit = @(x, b) bitand(uint32(x), uint32(2^b)) ~= 0;
            kF = find(bit(fw.v(:, 1), 7) & fw.t >= tDrop, 1);
            rec = struct('id', 'T07-MODES-01', 'run', 'Shaddad_System, normal mode', 'stimulus', sprintf(['TX on ' ...
                '0.5 s, arm 2.0 s (MANUAL, standing), DRV_CMD_L dropped %.1f to %.1f s (can1_drop_mask bit 0), RC ' ...
                'reset (CH9) %.1f to %.1f s, arm off 7.0 s and on 8.0 s, %.0f s'], tDrop, tDrop + 0.2, tReset, ...
                tReset + 0.3, T), 'wall_s', round(res.wall_s, 1), ...
                'mode_before_drop', at(m, tDrop - 0.1), 'drv_l_fault_set_s', NaN, ...
                'mode_after_drop', at(m, tReset - 0.1), 'mode_end', at(m, T - 0.05), ...
                'fault_word_end', at(fw, T - 0.05), 'drv_l_fault_end', bit(at(fw, T - 0.05), 7), ...
                'do0_before_drop', bitand(at(dw, tDrop - 0.1), 1), 'do0_end', bitand(at(dw, T - 0.05), 1), ...
                'safety_ok_end', NaN, 'drive_state_end', [], 'mode_changes', [], 'note', '');
            if ~isempty(kF), rec.drv_l_fault_set_s = fw.t(kF); end
            if isfield(S, 'power_status') && isfield(S.power_status, 'Safety_OK')
                rec.safety_ok_end = at(S.power_status.Safety_OK, T - 0.05);
            end
            if isfield(S.telem, 'drive_state'), rec.drive_state_end = at(S.telem.drive_state, T - 0.05); end
            ch = [1; find(diff(double(m.v(:, 1))) ~= 0) + 1];
            rec.mode_changes = [m.t(ch), double(m.v(ch, 1))];
            % the expected safe reaction to the time-out (the fault stops the robot)
            tc.assertEqual(rec.mode_before_drop, 3, 'precondition: MANUAL (armed) before the drop');
            tc.verifyFalse(isnan(rec.drv_l_fault_set_s), 'DRV_L_FAULT (bit 7) after DRV_CMD_L was lost');
            tc.verifyEqual(rec.mode_after_drop, 9, 'FAULT after the drive time-out (class A)');
            % the expected recovery: cause gone, a deliberate reset from valid RC frames and a new arm (REQ-SYS-011,
            % -002, -064, -078)
            ok = ~rec.drv_l_fault_end && rec.do0_end == 1 && any(rec.mode_end == [2 3]);
            rec.note = sprintf(['After the reset at %.1f s and the new arm at 8.0 s: mode %d, DRV_L_FAULT %d, DO0 %d, ' ...
                'Safety_OK %g, drive states %s.'], tReset, rec.mode_end, rec.drv_l_fault_end, rec.do0_end, ...
                rec.safety_ok_end, mat2str(rec.drive_state_end));
            if ~ok
                rec.note = ['T07-MODES-01 CONFIRMED by the run: ' rec.note ' The robot stays stopped (fail-safe) but ' ...
                    'the RC reset does not recover it.'];
            end
            tc.writeJson(fullfile(tc.Root, 'work', 'team', 't07', 'logs', 't07_modes01_run.json'), rec);
            tc.log(1, rec.note);
            tc.verifyTrue(ok, ['T07-MODES-01 (finding, not fixed here): ' rec.note ' Expected: once DRV_CMD_L ' ...
                'is back, the RC reset clears DRV_L_FAULT, DO0 closes the loop and the new arm gives MANUAL ' ...
                '(REQ-SYS-011, -002, -064, -078).']);
        end

        % ================================================================== MCU-02-F6: mission commands
        function testMissionCommands_CH13_CH14(tc)
            P = evalin('base', 'P'); R = P.electronics.rcmap; n = R.debounce_n;
            N = 300; t = (0:N-1)'*tc.DT;
            ch = repmat(uint16(992), N, 16);
            mis = repmat(uint16(172), N, 1);                           % position 1
            mis(t >= 0.30 & t < 0.31) = 1811;                          % a one-sample glitch to position 3
            mis(t >= 0.50) = 992;                                      % position 2: START
            mis(t >= 1.20) = 1811;                                     % position 3: PAUSE
            mis(t >= 1.80) = 172;                                      % back to position 1: no event
            mis(t >= 2.20) = 992;                                      % position 2 again: START
            rth = repmat(uint16(172), N, 1); rth(t >= 1.50 & t < 1.70) = 1811;   % CH14 pulse: RTH on the rise
            ch(:, R.ch.mission) = mis; ch(:, R.ch.rth) = rth;
            ds = mcu_harness('ds', {'ch_raw', timeseries(ch, t)}, {'frame_lost', timeseries(false(N, 1), t)}, ...
                {'failsafe', timeseries(false(N, 1), t)}, {'last_valid', timeseries(true(N, 1), t)});
            tc.addTeardown(@() mcu_harness('close'));
            o = mcu_harness('sim', 'rcmap', t(end), ds);
            rc = o.logsout.get('rc').Values;
            cmd = double(squeeze(rc.rc_aux_cmd.Data)); seq = double(squeeze(rc.rc_aux_seq.Data));
            tt = rc.rc_aux_seq.Time;
            ev = find(diff([seq(1); seq]) ~= 0);
            exp = [0.50 + (n - 1)*tc.DT, R.aux_code(1); 1.20 + (n - 1)*tc.DT, R.aux_code(2); 1.50, R.aux_code(3); ...
                2.20 + (n - 1)*tc.DT, R.aux_code(1)];
            got = [tt(ev), cmd(ev)];
            tc.log(1, sprintf('mission events at %s s, codes %s (expected %s, %s)', mat2str(got(:, 1)', 4), ...
                mat2str(got(:, 2)'), mat2str(exp(:, 1)', 4), mat2str(exp(:, 2)')));
            tc.verifyEqual(size(got, 1), 4, 'four events: START, PAUSE, RTH, START; none for the glitch or position 1');
            if size(got, 1) == 4
                tc.verifyEqual(got(:, 1), exp(:, 1), 'AbsTol', tc.DT + 1e-9, 'event times (debounce on CH13)');
                tc.verifyEqual(got(:, 2), exp(:, 2), 'rc_aux_cmd = the code of the last event');
            end
            tc.verifyEqual(seq(end) - seq(1), 4, 'rc_aux_seq counts the events');
            tc.verifyEqual(cmd(end), R.aux_code(1), 'rc_aux_cmd holds the last code');
        end

        % ================================================================== FSM-MCU-STOP S6, S7, S11, S12
        function testStopS6_RemoteEstopOverspeedParks(tc)
            % a remote E-stop while moving with OVERSPEED set parks at once: ESTOP without a deceleration phase
            tE = 1.6; tB = 2.0;
            Y = tc.mm(2.4, struct('fault_word', [0 0; tE 2 + 2^16], 'standstill', [0 1; tE 0], 'v_max', [0 0; tE 1.0], ...
                'brakes_ok', [0 1; tE 0; tB 1]));
            tc.verifyEqual(Y.mode(Y.k(tE - 0.01)), 3, 'MANUAL before the E-stop');
            kE = find(Y.mode == 8, 1);
            tc.assertNotEmpty(kE, 'ESTOP after the remote E-stop');
            tc.verifyTrue(Y.park(kE), 'S6: ESTOP entered in the park (ESTOP_PARK)');
            tc.verifyFalse(any(Y.de(Y.mode == 8)), 'S6: no deceleration phase (drive_enable never true in ESTOP)');
            tc.verifyFalse(Y.park(Y.k(tB + 0.05)), 'ESTOP_SAFE once the brakes confirm');
            tc.verifyEqual(Y.mode(end), 8, 'ESTOP held');
        end

        function testStopS7_RemoteEstopDuringCtrlPark(tc)
            % a remote E-stop during the CTRL_STOP parking: ESTOP goes on parking, and the park keeps its timer
            P = evalin('base', 'P'); nB = round(P.electronics.safety.stopfail_brake_t/tc.DT);
            tB = 1.6; tE = 2.4;
            Y = tc.mm(3.5, struct('fault_word', [0 0; tB 2^9; tE 2^9 + 2], 'brakes_ok', [0 1; tB 0]));
            kC = find(Y.mode == 7, 1);
            tc.assertNotEmpty(kC, 'CTRL_STOP after the class-B bit');
            kP = find(Y.mode == 7 & Y.park, 1);
            tc.assertNotEmpty(kP, 'CTRL_STOP parks after standstill_t');
            kE = find(Y.mode == 8, 1);
            tc.assertNotEmpty(kE, 'ESTOP after the remote E-stop');
            tc.verifyTrue(Y.park(kE) && ~Y.de(kE), 'S7: ESTOP entered in the park (ESTOP_PARK), no deceleration');
            kF = find(Y.sf, 1);
            tc.assertNotEmpty(kF, 'STOP_FAIL (brakes not confirmed) during the park');
            tc.log(1, sprintf('S7: park at %.2f s, ESTOP at %.2f s, STOP_FAIL at %.2f s (brake time-out %.2f s)', ...
                Y.t(kP), Y.t(kE), Y.t(kF), nB*tc.DT));
            tc.verifyEqual(kF - kP, nB, 'AbsTol', 2, 'S7: the park keeps its timer across CTRL_STOP -> ESTOP');
        end

        function testStopS11_HwEstopEndsDecel(tc)
            % a hardware E-stop during ESTOP.DECEL ends the deceleration at once: ESTOP_SAFE
            tE = 1.6; tH = 1.8;
            Y = tc.mm(2.2, struct('fault_word', [0 0; tE 2; tH 3], 'standstill', [0 1; tE 0], 'v_max', [0 0; tE 1.0]));
            kE = find(Y.mode == 8, 1);
            tc.assertNotEmpty(kE, 'ESTOP after the remote E-stop');
            tc.verifyTrue(all(Y.de(kE:Y.k(tH - 0.01))), 'ESTOP_DECEL until the hardware E-stop');
            kH = Y.k(tH);
            tc.verifyFalse(Y.de(kH) || Y.park(kH), 'S11: ESTOP_SAFE at the hardware E-stop (no deceleration, no park)');
            tc.verifyFalse(any(Y.de(kH:end) | Y.park(kH:end)), 'ESTOP_SAFE held');
        end

        function testStopS12_HwEstopEndsPark(tc)
            % a hardware E-stop during the ESTOP park ends the park at once, brakes confirmed or not: ESTOP_SAFE
            tE = 1.6; tH = 1.9;
            Y = tc.mm(2.3, struct('fault_word', [0 0; tE 2 + 2^16; tH 3 + 2^16], 'standstill', [0 1; tE 0], ...
                'v_max', [0 0; tE 1.0], 'brakes_ok', [0 1; tE 0]));
            kE = find(Y.mode == 8, 1);
            tc.assertNotEmpty(kE, 'ESTOP after the remote E-stop');
            tc.verifyTrue(all(Y.park(kE:Y.k(tH - 0.01))), 'ESTOP_PARK until the hardware E-stop (brakes not confirmed)');
            kH = Y.k(tH);
            tc.verifyFalse(Y.park(kH) || Y.de(kH), 'S12: ESTOP_SAFE at the hardware E-stop, the brakes unconfirmed');
        end

        % ================================================================== FSM-MCU-MODE M23
        function testModeM23_DefensiveDefault(tc)
            % M23 (an unknown state code to FAULT) cannot be driven: the state is a persistent variable that only the
            % FSM writes. Checked statically: the default branch goes to FAULT, and every ShaddadMode state has a
            % branch of its own before it.
            src = fileread(fullfile(tc.Root, 'subsystems', 'electronics', 'mcu_mode_manager.m'));
            tc.verifyNotEmpty(regexp(src, 'else\s*\n\s*nxt = M_FAULT;\s*%\s*unknown code \(defensive\)', 'once'), ...
                'M23: the default branch sets nxt = M_FAULT');
            body = src(1:regexp(src, 'nxt = M_FAULT;\s*%\s*unknown code', 'once'));
            for s = {'M_BOOT', 'M_SELFTEST', 'M_STANDBY', 'M_MANUAL', 'M_HOLD', 'M_AUTO_START', 'M_AUTO', ...
                    'M_CTRL_STOP', 'M_ESTOP', 'M_FAULT'}
                tc.verifyNotEmpty(regexp(body, ['st == ' s{1} '\>'], 'once'), ['a branch tests ' s{1}]);
            end
            tc.verifyEqual(numel(enumeration('ShaddadMode')), 10, 'ShaddadMode has the 10 states the FSM handles');
        end

        % ================================================================== MCU_STATUS from the firmware
        function testMcuStatusInFirmware(tc)
            u = 'fw_can2_codec';
            shaddad_configsets('codegen');
            ctx = t_t07_mcu_sil.context({u});
            tc.assertEmpty(ctx.error, ctx.error);
            m = t_t07_mcu_sil.harness(u, ctx.units.(u));
            tc.addTeardown(@() t_t07_mcu_sil.closeQuiet({m, u}));
            L = t_t07_frames.can2Layout('mcu_status');
            ov = L.ovr;
            for k = fieldnames(ov)', ov.(k{1}) = ov.(k{1})(:); end
            ts = evalin('base', get_param([m '/status'], 'SampleTime'));
            N = 12; t = (0:N-1)'*ts;
            X = t_t07_frames.busStim(L.bus, t, ov);
            tms = uint32([0 10 20 1000 2^31 2^32-20 2^32-10 2^32-1 30 40 50 60])';
            sc = t_t07_mcu_sil.newScenario(t(end), 'MCU_STATUS fields of t_t07_frames', []);
            fn = fieldnames(X);
            for k = 1:numel(fn)
                sc = t_t07_mcu_sil.ovSet(sc, ['status.' fn{k}], struct('series', double(X.(fn{k}).Data)));
            end
            tc.assertEqual(evalin('base', get_param([m '/t_ms'], 'SampleTime')), ts, 't_ms at the rate of status');
            tmsS = double(tms);
            sc = t_t07_mcu_sil.ovSet(sc, 't_ms', struct('series', tmsS));
            ds = t_t07_mcu_sil.inputs(m, sc);
            R = t_t07_mcu_sil.simulate(m, ds, t(end), 'normal');
            fr = [];
            for k = 1:numElements(R.y)
                e = R.y.getElement(k);
                if strcmp(get_param(e.BlockPath.getBlock(1), 'Name'), 'can2_status'), fr = e.Values; end
            end
            tc.assertNotEmpty(fr, 'the can2_status output was logged');
            keep = abs(fr.id.Time - t') < 1e-9; keep = any(keep, 2);
            id = double(squeeze(fr.id.Data)); ln = double(squeeze(fr.len.Data)); D = t_t07_frames.rows(fr.data.Data, 64);
            id = id(keep); ln = ln(keep); D = D(keep, :);
            tc.verifyEqual(id, repmat(L.id, N, 1), 'MCU_STATUS identifier (ICD 5)');
            tc.verifyEqual(ln, repmat(L.len, N, 1), 'MCU_STATUS length (ICD 5)');
            tc.verifyEqual(double(D(:, L.len+1:end)), zeros(N, 64 - L.len), 'padding after the length is 0');
            nBad = 0; rep = {};
            for k = 1:size(L.f, 1)
                [fld, off, ty, el, ix, res, kind] = L.f{k, :};
                got = t_t07_frames.rd(D, off, ty);
                switch kind
                    case 'tms'
                        want = double(tmsS(1:N));
                    case 'zero'
                        want = zeros(N, 1);
                    case 'flags'
                        want = zeros(N, 1);
                        for b = 1:numel(el)
                            if ~isempty(el{b}), want = want + 2^(b - 1)*double(X.(el{b}).Data(:, 1)); end
                        end
                    otherwise
                        v = X.(el).Data; v = v(:, ix);
                        if strcmp(kind, 'scaled')
                            want = double(t_t07_frames.rnd(single(single(v)/single(res)), ty));
                        elseif strcmp(ty, 'single')
                            want = double(typecast(single(v), 'uint32')); got = double(typecast(single(got), 'uint32'));
                        else
                            want = double(v);
                        end
                end
                bad = find(got(:) ~= want(:));
                nBad = nBad + numel(bad);
                if ~isempty(bad), rep{end+1} = sprintf('%s (byte %d) samples %s', fld, off, mat2str(bad')); end %#ok<AGROW>
            end
            tc.log(1, sprintf('MCU_STATUS through fw_can2_codec: %d mismatches against ICD 5 in %d frames %s', nBad, N, ...
                strjoin(rep, '; ')));
            tc.verifyEqual(nBad, 0, strjoin(rep, newline));
        end
    end

    methods
        function Y = mm(tc, T, tab)
        %MM  Step mcu_mode_manager.m from power-up for T s (0.01 s ticks): healthy inputs, armed at 1.3 s (MANUAL),
        %   with the ZOH tables of tab ([t value] rows, per input) applied on top. Y: .t .k(t) .mode .de (drive_enable)
        %   .park (stop_ctl(1)) .sf (stop_ctl(2)).
            P = evalin('base', 'P'); S = P.electronics.safety; dt = tc.DT;
            n = round(T/dt) + 1; t = (0:n-1)'*dt;
            d = struct('rc_ok', 1, 'sw_mode', 1, 'arm', [0 0; 1.3 1], 'arm_req', [0 0; 1.3 1; 1.31 0], ...
                'sticks_neutral', 1, 'override', 0, 'reset_req', 0, 'reset_evt', 0, 'fault_word', 0, 'comp_ok', 1, ...
                'gnc_ready', 1, 'mode_req', 2, 'hv_ready', 1, 'key_on', 1, 'safety_ok', 1, 'lid_closed', 1, ...
                'dock_present', 0, 'standstill', 1, 'v_max', 0, 'brakes_ok', 1, 'selftest_ok', 1);
            f = fieldnames(tab);
            for j = 1:numel(f), d.(f{j}) = tab.(f{j}); end
            g = structfun(@(v) t_t07_mcu_sil.series(v, t, 1), d, 'UniformOutput', false);
            prm_n = uint16(round([S.selftest_t S.selftest_timeout S.auto_start_warn_t S.standstill_t ...
                S.ctrlstop_timeout P.electronics.shaper.estop_tmax S.stopfail_t0 S.stopfail_brake_t S.stopfail_move_t]/dt));
            prm_mask = uint32([S.maskA S.maskB S.mask_rc]);
            prm_flag = logical([S.rc_required_in_auto S.rearm.auto_selftest]);
            prm_s = single([P.electronics.shaper.a_ctrlstop P.electronics.shaper.a_estop S.ctrlstop_t_extra ...
                S.stopfail_dv S.stopfail_rate S.standstill_v 1/dt]);
            clear('mcu_mode_manager');
            Y = struct('t', t, 'mode', zeros(n, 1), 'de', false(n, 1), 'park', false(n, 1), 'sf', false(n, 1));
            for k = 1:n
                [m, ~, de, ~, ~, sc] = mcu_mode_manager(g.rc_ok(k) ~= 0, uint8(g.sw_mode(k)), g.arm(k) ~= 0, ...
                    g.arm_req(k) ~= 0, g.sticks_neutral(k) ~= 0, g.override(k) ~= 0, g.reset_req(k) ~= 0, ...
                    g.reset_evt(k) ~= 0, uint32(g.fault_word(k)), false(1, 3), g.comp_ok(k) ~= 0, g.gnc_ready(k) ~= 0, ...
                    uint8(g.mode_req(k)), g.hv_ready(k) ~= 0, g.key_on(k) ~= 0, g.safety_ok(k) ~= 0, ...
                    g.lid_closed(k) ~= 0, g.dock_present(k) ~= 0, g.standstill(k) ~= 0, single(g.v_max(k)), ...
                    g.brakes_ok(k) ~= 0, g.selftest_ok(k) ~= 0, prm_n, prm_mask, prm_flag, prm_s);
                Y.mode(k) = double(m); Y.de(k) = de; Y.park(k) = sc(1); Y.sf(k) = sc(2);
            end
            clear('mcu_mode_manager');
            Y.k = @(tq) find(t <= tq + 1e-9, 1, 'last');
        end

        function writeJson(~, file, rec)
            fid = fopen(file, 'w');
            fprintf(fid, '%s\n', jsonencode(rec, 'PrettyPrint', true));
            fclose(fid);
        end
    end
end
