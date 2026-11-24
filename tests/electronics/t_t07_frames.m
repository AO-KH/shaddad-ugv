classdef t_t07_frames < matlab.unittest.TestCase
%T_T07_FRAMES  Card T-07 step 2: the CAN1 and CAN2 frames from sender to receiver, against the ICD byte layouts.
%
%   Each frame is packed by its sender's codec unit and unpacked by its receiver's codec unit: the same
%   Subsystem Reference files that run in the MCU firmware (fw_can1_codec embeds implcmd_can_pack,
%   implfb_can_unpack, bmsreq_can_pack, bms_can_unpack, drive_cmd_pack and drive_fb_unpack; fw_can2_codec the
%   can2_* units), in the BMS firmware (bms_can_pack, bmsreq_can_unpack), in the implement ECU
%   (implcmd_can_unpack, implfb_can_pack) and on the companion (companion_io: the can2_* units). For every frame:
%     - the byte image is decoded with this class's own copy of the docs/icd.md tables (sections 4 and 5:
%       byte offsets, types, scales, offsets, little-endian, flag bits) and compared with the sender's inputs,
%       for typical, minimum, maximum and out-of-range values (out-of-range values saturate);
%     - the identifier, the length and the padding bytes are checked;
%     - the receiver's outputs are compared with the decoded values;
%     - a missing frame (valid false), a frame with a foreign identifier and a short frame (length 4) are fed
%       to the receivers of the single-frame CAN1 codecs and of COMP_CMD, and their reaction is checked: the
%       valid output and the hold of the last good values. The length is logged, since no receiver is
%       specified to check it (finding T07-FRM-01 of the frames inventory).
%   Not here: the DRV frames (the same drive codec units, byte image against ICD 4 in t_drive/testT16_codec),
%   SBUS (t_mcu_platform TEL01 to TEL06, t_rc_link) and the time-outs of the firmware (t_mcu_control).
%   Harness models are built in memory and never saved (tmp_T07_*). Requires P, Ts and the Bus_* objects in base.

    properties (TestParameter)
        msg = {'comp_cmd', 'mcu_status', 'mcu_imu', 'mcu_odom', 'mcu_power'};
    end

    properties
        P
    end

    properties (Constant)
        NG = 8;         % normal samples; then 9 drop, 10 foreign identifier, 11 short (length 4), 12 normal again
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.assumeEqual(evalin('base', 'exist(''P'', ''var'')'), 1, 'P, Ts and the buses must be in base');
            tc.P = evalin('base', 'P');
            tc.addTeardown(@t_t07_frames.closeAll);
        end
    end

    methods (Test, TestTags = {'T07'})
        % ------------------------------------------------------------------ CAN1 0x140 IMPL_CMD
        function testImplCmdFrame(tc)
            N = tc.NG + 4; ts = evalin('base', 'Ts.can_slow'); t = (0:N-1)'*ts;
            en = logical([1 0 1 0 1 1 0 1 0 1 1 0])'; rst = logical([0 1 1 0 0 1 0 1 1 0 1 0])';
            al = uint8([0 1 254 255 0 7 8 9 10 11 12 13])';
            rpm = single([2800.4 -32768 32767 40000 -40000 -1234.6 0.3 100.7 500.2 600.2 700.2 800.2])';
            o = t_t07_frames.runPair('implcmd', 'implcmd_can_pack', ...
                {'enable', 'boolean', en; 'fault_reset', 'boolean', rst; 'alive', 'uint8', al; 'speed_rpm', 'single', rpm}, ...
                'implcmd_can_unpack', {'enable', 'fault_reset', 'alive', 'speed_rpm', 'valid'}, 'Bus_CanFrame8', ts, t);
            raw = t_t07_frames.rndn(rpm, 'int16');
            ctrl = uint8(en) + 2*uint8(rst);
            exp = [ctrl, al, t_t07_frames.le(raw, 'int16'), zeros(N, 4, 'uint8')];
            tc.checkTx(o, hex2dec('140'), 8, exp, 'IMPL_CMD');
            ref = struct('enable', en, 'fault_reset', rst, 'alive', double(al), 'speed_rpm', double(raw));
            tc.checkRx(o, ref, {'enable', 'fault_reset', 'alive', 'speed_rpm'}, 0, 'IMPL_CMD at the implement ECU');
        end

        % ------------------------------------------------------------------ CAN1 0x141 IMPL_FB
        function testImplFbFrame(tc)
            N = tc.NG + 4; ts = evalin('base', 'Ts.can_slow'); t = (0:N-1)'*ts;
            run_ = logical([1 0 1 0 1 1 0 1 0 1 1 0])'; flt = logical([0 1 1 0 0 1 0 1 1 0 1 0])';
            rdy = logical([1 1 0 0 1 0 1 1 0 0 1 1])';
            typ = uint8([1 0 3 255 2 1 0 3 1 2 3 1])'; al = uint8([0 1 254 255 0 7 8 9 10 11 12 13])';
            spd = single([2800.4 -32768 32767 40000 -40000 -1234.6 0.3 100.7 500.2 600.2 700.2 800.2])';
            cur = single([12.34 -3276.8 3276.7 4000 -4000 -0.04 0 55.56 1.2 2.3 3.4 4.5])';
            tmp = single([45 -40 215 300 -60 25.4 0 100.6 30.2 31.2 32.2 33.2])';
            o = t_t07_frames.runPair('implfb', 'implfb_can_pack', ...
                {'running', 'boolean', run_; 'fault', 'boolean', flt; 'ready', 'boolean', rdy; 'impl_type', 'uint8', typ; ...
                 'speed_rpm', 'single', spd; 'current_A', 'single', cur; 'temp_degC', 'single', tmp; 'alive', 'uint8', al}, ...
                'implfb_can_unpack', {'running', 'fault', 'ready', 'impl_type', 'speed_rpm', 'current_A', 'temp_degC', ...
                'alive', 'valid'}, 'Bus_CanFrame8', ts, t);
            rs = t_t07_frames.rndn(spd, 'int16');
            rc = t_t07_frames.rndn(single(cur*single(10)), 'int16');
            rt = t_t07_frames.rndn(single(tmp + single(40)), 'uint8');
            st = uint8(run_) + 2*uint8(flt) + 4*uint8(rdy);
            exp = [st, typ, t_t07_frames.le(rs, 'int16'), t_t07_frames.le(rc, 'int16'), uint8(rt), al];
            tc.checkTx(o, hex2dec('141'), 8, exp, 'IMPL_FB');
            ref = struct('running', run_, 'fault', flt, 'ready', rdy, 'impl_type', double(typ), 'speed_rpm', double(rs), ...
                'current_A', double(single(rc)*single(0.1)), 'temp_degC', double(single(rt) - single(40)), 'alive', double(al));
            tc.checkRx(o, ref, fieldnames(ref)', 2e-6, 'IMPL_FB at the MCU');
        end

        % ------------------------------------------------------------------ CAN1 0x3D0 BMS_REQ
        function testBmsReqFrame(tc)
            N = tc.NG + 4; ts = evalin('base', 'Ts.can_slow'); t = (0:N-1)'*ts;
            al = uint8([0 1 254 255 0 7 8 9 10 11 12 13])';
            f = logical(dec2bin(mod([0 15 5 10 1 2 4 8 3 12 6 9], 16), 4) - '0');     % all four bits in turn
            req = struct('AliveCounter', timeseries(al, t), 'HV_Enable_Req', timeseries(f(:, 4), t), ...
                'Charge_Enable_Req', timeseries(f(:, 3), t), 'Fault_Reset_Req', timeseries(f(:, 2), t), ...
                'Shutdown_Req', timeseries(f(:, 1), t));
            o = t_t07_frames.runPair('bmsreq', 'bmsreq_can_pack', {'req', 'Bus: Bus_BmsReq', req}, ...
                'bmsreq_can_unpack', {'req', 'valid'}, 'Bus_CanFrame8', ts, t);
            flags = uint8(f(:, 4)) + 2*uint8(f(:, 3)) + 4*uint8(f(:, 2)) + 8*uint8(f(:, 1));
            exp = [flags, al, zeros(N, 6, 'uint8')];
            tc.checkTx(o, hex2dec('3D0'), 8, exp, 'BMS_REQ');
            R = o.req;
            ref = struct('AliveCounter', double(al), 'HV_Enable_Req', f(:, 4), 'Charge_Enable_Req', f(:, 3), ...
                'Fault_Reset_Req', f(:, 2), 'Shutdown_Req', f(:, 1));
            o2 = struct('valid', o.valid);
            for k = fieldnames(ref)', o2.(k{1}) = R.(k{1}); end
            o2.frame_tx = o.frame_tx; o2.frame_rx = o.frame_rx;
            tc.checkRx(o2, ref, fieldnames(ref)', 0, 'BMS_REQ at the BMS');
        end

        % ------------------------------------------------------------------ CAN1 0x351 .. 0x3C1 BMS frames
        function testBmsFrames(tc)
            N = tc.NG; ts = evalin('base', 'Ts.hk'); t = (0:N-1)'*ts;
            ov = struct( ...
                'V_chg_lim_V', [54.6 0 6553.5 7000 -5 48.04 39.5 50.25], 'I_chg_lim_A', [20 -3276.8 3276.7 4000 -4000 -12.34 0 5.55], ...
                'I_dis_lim_A', [150 -3276.8 3276.7 4000 -4000 -12.34 0 75.25], 'V_dis_lim_V', [39 0 6553.5 7000 -5 40.04 42.5 44.25], ...
                'SOC_pack', [0.513 0 655.35 700 -0.1 1 0.2 0.754], 'SOH', [0.95 0 655.35 700 -0.1 0.8 0.5 0.904], ...
                'SOE', [0.6 0 6.5535 7 -0.1 0.3 0.42 0.9], 'V_pack_V', [47.5 -327.68 327.67 400 -400 0 50.004 52.444], ...
                'I_pack_A', [-12.3 -3276.8 3276.7 4000 -4000 150 0 33.33], 'V_bus_V', [48.1 0 655.35 700 -1 0 49.004 51.444], ...
                'T_cell_max_K', [318.15 1 6553.5 7000 -5 300.04 280 333.33], 'T_cell_min_K', [298.15 0 6553.5 7000 -5 290.04 275 310.33], ...
                'V_cell_min_V', [3.2 0 65.535 70 -0.1 3.0004 2.9 3.3333], 'V_cell_max_V', [3.6 0 65.535 70 -0.1 3.4004 4.1 4.2222], ...
                'Runtime_Remaining_s', [3600 0 3932100 4e6 -60 1234.5 59 61], 'E_remaining_Wh', [2500 0 655350 7e5 -10 1234.5 4 6], ...
                'P_dis_lim_W', [5000 0 655350 7e5 -10 1234.5 4 6], 'P_chg_lim_W', [1200 0 655350 7e5 -10 4321.5 4 6], ...
                'P_pack_W', [-3000 -327680 327670 4e5 -4e5 1234.5 -4 6], 'Fan_Cmd', [0.5 0 2.55 3 -0.1 0.304 1 0.754]);
            fn = fieldnames(ov);
            for k = 1:numel(fn), ov.(fn{k}) = ov.(fn{k})(:); end
            tel = t_t07_frames.busStim('Bus_BMS_Telemetry', t, ov);
            o = t_t07_frames.runBms(tel, ts, t);
            X = @(nm) double(tel.(nm).Data(:, 1));
            % the ICD 4 rows: {slot, id, {field, byte offset, wire type, telemetry element, offset, counts per unit}}
            F = { ...
                'limits', '351', {'CVL', 0, 'uint16', 'V_chg_lim_V', 0, 10; 'CCL', 2, 'int16', 'I_chg_lim_A', 0, 10; ...
                                  'DCL', 4, 'int16', 'I_dis_lim_A', 0, 10; 'DVL', 6, 'uint16', 'V_dis_lim_V', 0, 10}; ...
                'soc', '355', {'SOC', 0, 'uint16', 'SOC_pack', 0, 100; 'SOH', 2, 'uint16', 'SOH', 0, 100; ...
                               'SOC_hires', 4, 'uint16', 'SOC_pack', 0, 10000; 'SOE', 6, 'uint16', 'SOE', 0, 10000}; ...
                'vit', '356', {'V_pack', 0, 'int16', 'V_pack_V', 0, 100; 'I_pack', 2, 'int16', 'I_pack_A', 0, 10; ...
                               'T_batt', 4, 'int16', 'T_cell_max_K', -273.15, 10; 'V_bus', 6, 'uint16', 'V_bus_V', 0, 100}; ...
                'alarm', '35A', {'Fault_Word', 0, 'uint32', 'Fault_Word', 0, 1; 'Warning_Word', 4, 'uint32', 'Warning_Word', 0, 1}; ...
                'cells', '373', {'Vcell_min', 0, 'uint16', 'V_cell_min_V', 0, 1000; 'Vcell_max', 2, 'uint16', 'V_cell_max_V', 0, 1000; ...
                                 'Tcell_min', 4, 'uint16', 'T_cell_min_K', 0, 10; 'Tcell_max', 6, 'uint16', 'T_cell_max_K', 0, 10}; ...
                'state', '3C0', {'Contactor_State', 0, 'uint8', 'Contactor_State', 0, 1; 'Fault_Level', 2, 'uint8', 'Fault_Level', 0, 1; ...
                                 'alive', 3, 'uint8', 'AliveCounter', 0, 1; 'Runtime_Remaining', 4, 'uint16', 'Runtime_Remaining_s', 0, 1/60; ...
                                 'E_remaining', 6, 'uint16', 'E_remaining_Wh', 0, 0.1}; ...
                'power', '3C1', {'P_dis_lim', 0, 'uint16', 'P_dis_lim_W', 0, 0.1; 'P_chg_lim', 2, 'uint16', 'P_chg_lim_W', 0, 0.1; ...
                                 'P_pack', 4, 'int16', 'P_pack_W', 0, 0.1; 'Fan_Cmd', 6, 'uint8', 'Fan_Cmd', 0, 100}};
            nBad = 0; rep = {};
            raw = struct();
            for j = 1:size(F, 1)
                fr = o.frames.(F{j, 1});
                id = double(squeeze(fr.id.Data)); ln = double(squeeze(fr.len.Data)); D = t_t07_frames.rows(fr.data.Data, 8);
                tc.verifyEqual(id(1:N), repmat(hex2dec(F{j, 2}), N, 1), ['BMS ' F{j, 1} ' identifier']);
                tc.verifyEqual(ln(1:N), repmat(8, N, 1), ['BMS ' F{j, 1} ' length']);
                fd = F{j, 3};
                for k = 1:size(fd, 1)
                    got = t_t07_frames.rd(D, fd{k, 2}, fd{k, 3});
                    if strcmp(fd{k, 3}, 'uint32')
                        want = X(fd{k, 4});
                    else
                        want = double(t_t07_frames.rndn(single((single(X(fd{k, 4})) + single(fd{k, 5}))*single(fd{k, 6})), fd{k, 3}));
                    end
                    bad = find(got(1:N) ~= want(1:N));
                    nBad = nBad + numel(bad);
                    if ~isempty(bad), rep{end+1} = sprintf('%s.%s samples %s: %s against %s', F{j, 1}, fd{k, 1}, ...
                            mat2str(bad'), mat2str(got(bad)'), mat2str(want(bad)')); end %#ok<AGROW>
                    raw.(fd{k, 1}) = got;
                end
            end
            % BMS_STATE byte 1 flags and BMS_POWER byte 7 reserved
            Ds = t_t07_frames.rows(o.frames.state.data.Data, 8);
            fl = {'HV_Ready', 'HV_Opening_Warn', 'K1_Closed', 'K2_Closed', 'K4_Closed', 'Charging_Active', 'Charge_Complete', ...
                'Balancing_Active'};
            for b = 0:7
                got = bitget(Ds(1:N, 2), b + 1); want = double(tel.(fl{b + 1}).Data(:, 1));
                nb = sum(double(got) ~= want); nBad = nBad + nb;
                if nb, rep{end+1} = sprintf('state flags bit %d (%s) wrong in %d samples', b, fl{b + 1}, nb); end %#ok<AGROW>
            end
            Dp = t_t07_frames.rows(o.frames.power.data.Data, 8);
            tc.verifyEqual(double(Dp(1:N, 8)), zeros(N, 1), 'BMS_POWER byte 7 reserved 0');
            tc.log(1, sprintf('T07 BMS frames: %d byte-image mismatches against ICD 4 in 7 frames x %d samples %s', nBad, N, ...
                strjoin(rep, '; ')));
            tc.verifyEqual(nBad, 0, strjoin(rep, newline));
            % the receiver (bms_can_unpack, in fw_can1_codec): every Bus_BmsStatus element the frames carry
            S = o.status;
            g = @(nm) double(squeeze(S.(nm).Data));
            chk = {'V_pack', raw.V_pack/100; 'I_pack', raw.I_pack/10; 'CVL', raw.CVL/10; 'CCL', raw.CCL/10; 'DCL', raw.DCL/10; ...
                'DVL', raw.DVL/10; 'SOC', raw.SOC_hires/10000; 'SOH', raw.SOH/100; 'V_cell_min', raw.Vcell_min/1000; ...
                'V_cell_max', raw.Vcell_max/1000; 'T_cell_min', raw.Tcell_min/10; 'T_cell_max', raw.Tcell_max/10; ...
                'alarm_word', raw.Fault_Word; 'warn_word', raw.Warning_Word; 'contactor_state', raw.Contactor_State; ...
                'fault_level', raw.Fault_Level; 'alive', raw.alive; 'Runtime_s', raw.Runtime_Remaining*60; ...
                'E_remaining_Wh', raw.E_remaining*10; 'P_dis_lim', raw.P_dis_lim*10; 'P_chg_lim', raw.P_chg_lim*10; ...
                'P_pack', raw.P_pack*10; 'hv_ready', double(bitget(Ds(:, 2), 1)); 'hv_opening_warn', double(bitget(Ds(:, 2), 2)); ...
                'charging', double(bitget(Ds(:, 2), 6)); 'charge_complete', double(bitget(Ds(:, 2), 7))};
            rb = {};
            for k = 1:size(chk, 1)
                y = g(chk{k, 1}); r = chk{k, 2};
                e = abs(y(1:N) - r(1:N)); tol = 2e-6*max(1, abs(r(1:N)));
                if any(e > tol), rb{end+1} = sprintf('%s max err %.3g', chk{k, 1}, max(e)); end %#ok<AGROW>
            end
            fv = logical(squeeze(o.frame_valid.Data));
            tc.log(1, sprintf('T07 BMS receiver: %d of %d elements off %s; frame_valid all %d', numel(rb), size(chk, 1), ...
                strjoin(rb, ', '), all(fv(1:N))));
            tc.verifyEmpty(rb, strjoin(rb, ', '));
            tc.verifyTrue(all(fv(1:N)), 'frame_valid with all seven frames fresh');
        end

        % ------------------------------------------------------------------ CAN2 0x100 .. 0x203
        function testCan2Frame(tc, msg)
            N = tc.NG + 4; ts = evalin('base', 'Ts.ctrl'); t = (0:N-1)'*ts;
            L = t_t07_frames.can2Layout(msg);
            ov = L.ovr;
            for k = fieldnames(ov)', ov.(k{1}) = ov.(k{1})(:); end
            X = t_t07_frames.busStim(L.bus, t, ov);
            ins = {'x', ['Bus: ' L.bus], X};
            tms = uint32([0 10 20 1000 2^31 2^32-20 2^32-10 2^32-1 30 40 50 60])';
            if strcmp(msg, 'mcu_status'), ins(end+1, :) = {'t_ms', 'uint32', tms}; end
            o = t_t07_frames.runPair(['can2_' msg], ['can2_' msg '_pack'], ins, ['can2_' msg '_unpack'], {'y'}, ...
                'Bus_CanFrame64', ts, t);
            fr = o.frame_tx;
            id = double(squeeze(fr.id.Data)); ln = double(squeeze(fr.len.Data)); D = t_t07_frames.rows(fr.data.Data, 64);
            tc.verifyEqual(id(1:N), repmat(L.id, N, 1), [msg ' identifier']);
            tc.verifyEqual(ln(1:N), repmat(L.len, N, 1), [msg ' length']);
            tc.verifyEqual(double(D(1:N, L.len+1:end)), zeros(N, 64 - L.len), [msg ' padding bytes after the length are 0']);
            nBad = 0; rep = {}; Y = o.y;
            ng = tc.NG;
            for k = 1:size(L.f, 1)
                [fld, off, ty, el, ix, res, kind] = L.f{k, :};
                got = t_t07_frames.rd(D, off, ty);
                switch kind
                    case 'tms'
                        want = double(tms);
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
                bad = find(got(1:ng) ~= want(1:ng));
                nBad = nBad + numel(bad);
                if ~isempty(bad), rep{end+1} = sprintf('%s (byte %d) samples %s', fld, off, mat2str(bad')); end %#ok<AGROW>
                % the receiver's output for this field (normal samples)
                if any(strcmp(kind, {'raw', 'scaled'})) && ~isempty(el)
                    y = double(Y.(el).Data); if size(y, 2) ~= size(X.(el).Data, 2), y = t_t07_frames.rows(Y.(el).Data, size(X.(el).Data, 2)); end
                    y = y(:, ix);
                    if strcmp(kind, 'scaled')
                        r = double(single(want)*single(res)); e = abs(y(1:ng) - r(1:ng)); tolr = 2e-6*max(1, abs(r(1:ng)));
                        okr = all(e <= tolr);
                    elseif strcmp(ty, 'single')
                        okr = isequal(typecast(single(y(1:ng)), 'uint32'), typecast(single(X.(el).Data(1:ng, ix)), 'uint32'));
                    else
                        okr = isequal(y(1:ng), double(X.(el).Data(1:ng, ix)));
                    end
                    if ~okr, rep{end+1} = sprintf('receiver %s', el); nBad = nBad + 1; end %#ok<AGROW>
                elseif strcmp(kind, 'flags')
                    for b = 1:numel(el)
                        if isempty(el{b}) || ~isfield(Y, el{b}) || strcmp(el{b}, 'valid'), continue, end
                        yb = double(squeeze(Y.(el{b}).Data)); xb = double(X.(el{b}).Data(:, 1));
                        if ~isequal(yb(1:ng), xb(1:ng)), rep{end+1} = sprintf('receiver %s', el{b}); nBad = nBad + 1; end %#ok<AGROW>
                    end
                end
            end
            tc.log(1, sprintf('T07 %s (0x%X, %d B): %d mismatches against ICD 5 and at the receiver in %d samples %s', msg, L.id, ...
                L.len, nBad, ng, strjoin(rep, '; ')));
            tc.verifyEqual(nBad, 0, strjoin(rep, newline));
            % missing, foreign-identifier and short frames at the receiver
            tc.badFrames(o, msg, L);
        end

        % ------------------------------------------------------------------ the receivers' reaction to bad frames
        function testBadFramesHeld(tc)
            % IMPL_CMD at the implement ECU, IMPL_FB at the MCU, BMS_REQ at the BMS: a missing frame (valid false) and a
            % frame with a foreign identifier must leave the receiver's valid output false and its outputs held at the
            % last good frame; a short frame is logged (no receiver is specified to check the length)
            N = tc.NG + 4; ts = evalin('base', 'Ts.can_slow'); t = (0:N-1)'*ts;
            al = uint8(1:N)'; rpm = single(100*(1:N) + 0.2)';
            o = t_t07_frames.runPair('implcmd', 'implcmd_can_pack', ...
                {'enable', 'boolean', true(N, 1); 'fault_reset', 'boolean', false(N, 1); 'alive', 'uint8', al; ...
                 'speed_rpm', 'single', rpm}, 'implcmd_can_unpack', {'enable', 'fault_reset', 'alive', 'speed_rpm', 'valid'}, ...
                'Bus_CanFrame8', ts, t);
            tc.heldCheck(o, {'alive', 'speed_rpm'}, 'IMPL_CMD at the implement ECU');
            typ = uint8(mod(1:N, 4))';
            o = t_t07_frames.runPair('implfb', 'implfb_can_pack', ...
                {'running', 'boolean', true(N, 1); 'fault', 'boolean', false(N, 1); 'ready', 'boolean', true(N, 1); ...
                 'impl_type', 'uint8', typ; 'speed_rpm', 'single', rpm; 'current_A', 'single', single(1:N)'; ...
                 'temp_degC', 'single', single(20 + (1:N))'; 'alive', 'uint8', al}, 'implfb_can_unpack', ...
                {'running', 'fault', 'ready', 'impl_type', 'speed_rpm', 'current_A', 'temp_degC', 'alive', 'valid'}, ...
                'Bus_CanFrame8', ts, t);
            tc.heldCheck(o, {'alive', 'speed_rpm', 'temp_degC'}, 'IMPL_FB at the MCU');
            req = struct('AliveCounter', timeseries(al, t), 'HV_Enable_Req', timeseries(true(N, 1), t), ...
                'Charge_Enable_Req', timeseries(false(N, 1), t), 'Fault_Reset_Req', timeseries(false(N, 1), t), ...
                'Shutdown_Req', timeseries(false(N, 1), t));
            o = t_t07_frames.runPair('bmsreq', 'bmsreq_can_pack', {'req', 'Bus: Bus_BmsReq', req}, ...
                'bmsreq_can_unpack', {'req', 'valid'}, 'Bus_CanFrame8', ts, t);
            o.alive = o.req.AliveCounter;
            tc.heldCheck(o, {'alive'}, 'BMS_REQ at the BMS');
        end
    end

    % ======================================================================================================
    methods
        function checkTx(tc, o, id, len, exp, what)
            % the sender's frame, normal samples: identifier, length, valid and the eight data bytes
            n = tc.NG; fr = o.frame_tx;
            tc.verifyEqual(double(squeeze(fr.id.Data(1:n))), repmat(id, n, 1), [what ' identifier']);
            tc.verifyEqual(double(squeeze(fr.len.Data(1:n))), repmat(len, n, 1), [what ' length']);
            tc.verifyTrue(all(squeeze(fr.valid.Data(1:n))), [what ' valid']);
            D = t_t07_frames.rows(fr.data.Data, 8);
            bad = find(any(D(1:n, :) ~= exp(1:n, :), 2));
            tc.log(1, sprintf('T07 %s (0x%X): byte image against ICD 4, %d of %d samples differ %s', what, id, numel(bad), n, ...
                mat2str(double(D(bad, :)))));
            tc.verifyEmpty(bad, sprintf('%s: samples %s differ from the ICD 4 layout', what, mat2str(bad')));
        end

        function checkRx(tc, o, ref, names, tol, what)
            % the receiver's outputs, normal samples, against the values the ICD layout decodes
            n = tc.NG; rep = {};
            for k = 1:numel(names)
                y = double(squeeze(o.(names{k}).Data)); r = double(ref.(names{k})(:));
                e = abs(y(1:n) - r(1:n));
                if any(e > tol*max(1, abs(r(1:n)))), rep{end+1} = sprintf('%s %s against %s', names{k}, mat2str(y(1:n)', 6), ...
                        mat2str(r(1:n)', 6)); end %#ok<AGROW>
            end
            v = logical(squeeze(o.valid.Data));
            tc.log(1, sprintf('T07 %s: %d of %d outputs off %s; valid %s', what, numel(rep), numel(names), strjoin(rep, '; '), ...
                mat2str(v(1:n)')));
            tc.verifyEmpty(rep, strjoin(rep, newline));
            tc.verifyTrue(all(v(1:n)), [what ': valid on every good frame']);
        end

        function heldCheck(tc, o, names, what)
            % samples 9 (missing), 10 (foreign identifier): valid false and the outputs of sample 8 held;
            % sample 11 (short): logged; sample 12: a good frame again
            n = tc.NG; v = logical(squeeze(o.valid.Data));
            tc.verifyFalse(v(n + 1), [what ': valid false on a missing frame']);
            tc.verifyFalse(v(n + 2), [what ': valid false on a foreign identifier']);
            tc.verifyTrue(v(n + 4), [what ': valid again on the next good frame']);
            short = {};
            for k = 1:numel(names)
                y = double(squeeze(o.(names{k}).Data));
                tc.verifyEqual(y(n + 1), y(n), [what ': ' names{k} ' held on a missing frame']);
                tc.verifyEqual(y(n + 2), y(n), [what ': ' names{k} ' held on a foreign identifier']);
                tc.verifyNotEqual(y(n + 4), y(n), [what ': ' names{k} ' updated by the next good frame']);
                short{end+1} = sprintf('%s %s', names{k}, tern(y(n + 3) == y(n), 'held', 'updated')); %#ok<AGROW>
            end
            tc.log(1, sprintf('T07 %s: missing frame valid %d, foreign id valid %d, short frame (length 4) valid %d with %s', ...
                what, v(n + 1), v(n + 2), v(n + 3), strjoin(short, ', ')));
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
        end

        function badFrames(tc, o, msg, L)
            % CAN2: a missing frame must clear the receiver's valid (where the bus has one); identifier and length logged
            n = tc.NG; Y = o.y; rep = {};
            if isfield(Y, 'valid')
                v = logical(squeeze(Y.valid.Data));
                tc.verifyFalse(v(n + 1), [msg ': valid false on a missing frame']);
                rep{end+1} = sprintf('valid: good %d, missing %d, foreign id %d, short %d, good again %d', v(n), v(n + 1), ...
                    v(n + 2), v(n + 3), v(n + 4));
            end
            % one carried field: is it held, zeroed or decoded on the bad samples?
            el = L.probe; y = double(squeeze(Y.(el).Data)); y = y(:, 1);
            rep{end+1} = sprintf('%s on samples 8..12: %s', el, mat2str(y(n:n + 4)', 6));
            tc.log(1, sprintf('T07 %s receiver on bad frames: %s', msg, strjoin(rep, '; ')));
        end
    end

    % ======================================================================================================
    methods (Static)
        function L = can2Layout(msg)
            % docs/icd.md section 5, field by field: {ICD field, byte offset, wire type, bus element (or flag list),
            % element index, resolution, kind}; kinds: raw, scaled, flags, zero, tms
            P = evalin('base', 'P'); r = P.electronics.can.res;
            switch msg
                case 'comp_cmd'
                    L.id = hex2dec('100'); L.len = 32; L.bus = 'Bus_GncCmd'; L.probe = 'seq';
                    L.f = {'seq', 0, 'uint8', 'seq', 1, 1, 'raw'; 'mode_req', 1, 'uint8', 'mode_req', 1, 1, 'raw'; ...
                        'gnc_state', 2, 'uint8', 'gnc_state', 1, 1, 'raw'; ...
                        'flags', 3, 'uint8', {'cmd_valid', 'gnc_ready', 'soft_stop_req', 'v_ground_valid', 'mower_enable', ...
                        'tiller_enable', 'spray_enable', 'buzzer_cmd'}, 1, 1, 'flags'; ...
                        'v_cmd', 4, 'single', 'v_cmd', 1, 1, 'raw'; 'omega_cmd', 8, 'single', 'omega_cmd', 1, 1, 'raw'; ...
                        'accel_limit', 12, 'single', 'accel_limit', 1, 1, 'raw'; 'v_ground_est', 16, 'single', 'v_ground_est', 1, 1, 'raw'; ...
                        'spray_section_mask', 20, 'uint16', 'spray_section_mask', 1, 1, 'raw'; 'beacon_cmd', 22, 'uint8', 'beacon_cmd', 1, 1, 'raw'; ...
                        'nav_fix', 23, 'uint8', 'nav_fix', 1, 1, 'raw'; ...
                        'mower_speed_cmd', 24, 'uint16', 'mower_speed_cmd', 1, r.mower_w, 'scaled'; ...
                        'tiller_speed_cmd', 26, 'uint16', 'tiller_speed_cmd', 1, r.tiller_w, 'scaled'; ...
                        'deck_height_cmd', 28, 'uint8', 'deck_height_cmd', 1, r.deck_h, 'scaled'; ...
                        'tiller_depth_cmd', 29, 'uint8', 'tiller_depth_cmd', 1, r.till_d, 'scaled'; ...
                        'spray_flow_cmd', 30, 'uint16', 'spray_flow_cmd', 1, r.spray_flow, 'scaled'};
                    L.ovr = struct('mower_speed_cmd', [293.2 0 6553.5 7000 -5 100.04 26.18 0.06 1 2 3 4], ...
                        'tiller_speed_cmd', [26.18 0 655.35 700 -5 10.004 1 2.5 1 2 3 4], ...
                        'deck_height_cmd', [0.06 0 0.255 0.3 -0.01 0.0304 0.12 0.1 0.01 0.02 0.03 0.04], ...
                        'tiller_depth_cmd', [0.08 0 0.255 0.3 -0.01 0.0404 0.12 0.1 0.01 0.02 0.03 0.04], ...
                        'spray_flow_cmd', [8.01 0 655.35 700 -1 1.004 2.5 3.25 1 2 3 4]);
                case 'mcu_status'
                    L.id = hex2dec('200'); L.len = 48; L.bus = 'Bus_McuStatus'; L.probe = 'heartbeat';
                    L.f = {'t_ms', 0, 'uint32', '', 1, 1, 'tms'; 'mode', 4, 'uint8', 'mode', 1, 1, 'raw'; ...
                        'cmd_src', 5, 'uint8', 'cmd_src', 1, 1, 'raw'; 'rc_mode_sw', 6, 'uint8', 'rc_mode_sw', 1, 1, 'raw'; ...
                        'rc_aux_cmd', 7, 'uint8', 'rc_aux_cmd', 1, 1, 'raw'; 'rc_aux_seq', 8, 'uint8', 'rc_aux_seq', 1, 1, 'raw'; ...
                        'flags', 9, 'uint8', {'rc_link_ok', 'rc_failsafe', 'estop_active', 'drive_ready', 'power_ready', ...
                        'implement_ready', 'charging', 'wdg_reset_seen'}, 1, 1, 'flags'; ...
                        'implement_type', 10, 'uint8', 'implement_type', 1, 1, 'raw'; 'heartbeat', 11, 'uint8', 'heartbeat', 1, 1, 'raw'; ...
                        'fault_word', 12, 'uint32', 'fault_word', 1, 1, 'raw'; 'warn_word', 16, 'uint32', 'warn_word', 1, 1, 'raw'; ...
                        'drive_fault_word', 20, 'uint32', 'drive_fault_word', 1, 1, 'raw'; ...
                        'v_track_L', 24, 'single', 'v_track_meas', 1, 1, 'raw'; 'v_track_R', 28, 'single', 'v_track_meas', 2, 1, 'raw'; ...
                        'v_applied', 32, 'single', 'v_applied', 1, 1, 'raw'; 'w_applied', 36, 'single', 'w_applied', 1, 1, 'raw'; ...
                        'gnc_cmd_age_ms', 40, 'uint16', 'gnc_cmd_age_ms', 1, 1, 'raw'; ...
                        'rc_lq', 42, 'uint8', 'rc_lq', 1, r.rc_lq, 'scaled'; 'reserved', 43, 'uint8', '', 1, 1, 'zero'; ...
                        'T_encl', 44, 'single', 'T_encl', 1, 1, 'raw'};
                    L.ovr = struct('rc_lq', [0.87 0 2.55 3 -0.1 0.504 1 0.33 0.1 0.2 0.3 0.4]);
                case 'mcu_imu'
                    L.id = hex2dec('201'); L.len = 48; L.bus = 'Bus_ImuMeas'; L.probe = 'seq';
                    L.f = {'t_us', 0, 'uint32', 't_us', 1, 1, 'raw'; 'seq', 4, 'uint16', 'seq', 1, 1, 'raw'; ...
                        'valid', 6, 'uint8', 'valid', 1, 1, 'raw'; 'reserved', 7, 'uint8', '', 1, 1, 'zero'};
                    for j = 1:3
                        L.f(end+1, :) = {sprintf('acc[%d]', j), 8 + 4*(j - 1), 'single', 'accel_B', j, 1, 'raw'};
                        L.f(end+1, :) = {sprintf('gyr[%d]', j), 20 + 4*(j - 1), 'single', 'gyro_B', j, 1, 'raw'};
                        L.f(end+1, :) = {sprintf('mag[%d]', j), 32 + 4*(j - 1), 'single', 'mag_B', j, 1, 'raw'};
                    end
                    L.f(end+1, :) = {'temp', 44, 'single', 'temp_degC', 1, 1, 'raw'};
                    L.ovr = struct();
                case 'mcu_odom'
                    L.id = hex2dec('202'); L.len = 32; L.bus = 'Bus_OdomMeas'; L.probe = 't_us';
                    L.f = {'t_us', 0, 'uint32', 't_us', 1, 1, 'raw'; 'enc_cnt_L', 4, 'int32', 'encCnt_LR', 1, 1, 'raw'; ...
                        'enc_cnt_R', 8, 'int32', 'encCnt_LR', 2, 1, 'raw'; 'v_track_L', 12, 'single', 'vTrack_LR', 1, 1, 'raw'; ...
                        'v_track_R', 16, 'single', 'vTrack_LR', 2, 1, 'raw'; 'w_m_L', 20, 'single', 'wMotor_LR', 1, 1, 'raw'; ...
                        'w_m_R', 24, 'single', 'wMotor_LR', 2, 1, 'raw'; 'valid', 28, 'uint8', 'valid', 1, 1, 'raw'; ...
                        'reserved29', 29, 'uint8', '', 1, 1, 'zero'; 'reserved30', 30, 'uint8', '', 1, 1, 'zero'; ...
                        'reserved31', 31, 'uint8', '', 1, 1, 'zero'};
                    L.ovr = struct();
                case 'mcu_power'
                    L.id = hex2dec('203'); L.len = 64; L.bus = 'Bus_BmsStatus'; L.probe = 'alive';
                    s13 = {'V_pack', 'I_pack', 'SOC', 'SOH', 'T_cell_max', 'T_cell_min', 'V_cell_min', 'V_cell_max', 'CCL', 'DCL', ...
                        'P_dis_lim', 'E_remaining_Wh', 'Runtime_s'};
                    L.f = cell(0, 7);
                    for j = 1:13, L.f(end+1, :) = {s13{j}, 4*(j - 1), 'single', s13{j}, 1, 1, 'raw'}; end
                    L.f = [L.f; {'alarm_word', 52, 'uint32', 'alarm_word', 1, 1, 'raw'; 'warn_word', 56, 'uint32', 'warn_word', 1, 1, 'raw'; ...
                        'fault_level', 60, 'uint8', 'fault_level', 1, 1, 'raw'; 'contactor_state', 61, 'uint8', 'contactor_state', 1, 1, 'raw'; ...
                        'flags', 62, 'uint8', {'hv_ready', 'hv_opening_warn', 'charging', 'charge_complete', 'bms_critical', 'valid', '', ''}, ...
                        1, 1, 'flags'; 'alive', 63, 'uint8', 'alive', 1, 1, 'raw'}];
                    L.ovr = struct();
            end
        end

        function o = runPair(tag, packUnit, ins, unpackUnit, outs, frameBus, ts, t)
            % harness: root inputs -> pack unit -> frame -> modifier (drop, foreign id, short) -> unpack unit -> outputs.
            % The bad samples are the last four but one: 9 drop, 10 identifier + 1, 11 length 4; 12 good.
            m = ['tmp_T07_' tag];
            N = numel(t); ng = t_t07_frames.NG;
            t_t07_frames.buildPair(m, packUnit, ins, unpackUnit, outs, frameBus, ts);
            ds = Simulink.SimulationData.Dataset;
            for k = 1:size(ins, 1)
                v = ins{k, 3};
                if ~isstruct(v), v = timeseries(v, t); end
                ds = ds.addElement(v, ins{k, 1});
            end
            drop = false(N, 1); drop(ng + 1) = true;
            idadd = zeros(N, 1, 'uint32'); idadd(ng + 2) = 1;
            lenset = zeros(N, 1, 'uint8'); lenset(ng + 3) = 4;
            ds = ds.addElement(timeseries(drop, t), 'drop');
            ds = ds.addElement(timeseries(idadd, t), 'id_add');
            ds = ds.addElement(timeseries(lenset, t), 'len_set');
            in = Simulink.SimulationInput(m);
            in = in.setExternalInput(ds);
            in = in.setModelParameter('StopTime', sprintf('%.6f', t(end)), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
            r = sim(in);
            if ~isempty(r.ErrorMessage), error('t_t07_frames:sim', '%s', r.ErrorMessage); end
            o = struct();
            for k = 1:numel(outs), o.(outs{k}) = t_t07_frames.logged(r.yout, [m '/o_' outs{k}]); end
            o.frame_tx = t_t07_frames.logged(r.yout, [m '/frame_tx']);
            o.frame_rx = t_t07_frames.logged(r.yout, [m '/frame_rx']);
        end

        function v = logged(Y, blk)
            % the values logged at the root outport blk (the Dataset elements carry signal names, not block names)
            for k = 1:Y.numElements
                e = Y.getElement(k);
                if strcmp(e.BlockPath.getBlock(1), blk), v = e.Values; return, end
            end
            error('t_t07_frames:logged', 'no logged output for %s', blk);
        end

        function buildPair(m, packUnit, ins, unpackUnit, outs, frameBus, ts)
            if bdIsLoaded(m), close_system(m, 0); end
            new_system(m);
            shaddad_configsets('attach', m);
            tsS = num2str(ts, 17);
            for k = 1:size(ins, 1)
                p = add_block('simulink/Sources/In1', [m '/' ins{k, 1}], 'Port', num2str(k), 'OutDataTypeStr', ins{k, 2}, ...
                    'SampleTime', tsS);
                if ~startsWith(ins{k, 2}, 'Bus:'), set_param(p, 'PortDimensions', '1'); end
            end
            nI = size(ins, 1);
            add_block('simulink/Sources/In1', [m '/drop'], 'Port', num2str(nI + 1), 'OutDataTypeStr', 'boolean', 'SampleTime', tsS);
            add_block('simulink/Sources/In1', [m '/id_add'], 'Port', num2str(nI + 2), 'OutDataTypeStr', 'uint32', 'SampleTime', tsS);
            add_block('simulink/Sources/In1', [m '/len_set'], 'Port', num2str(nI + 3), 'OutDataTypeStr', 'uint8', 'SampleTime', tsS);
            add_block('simulink/Ports & Subsystems/Subsystem Reference', [m '/Pack'], 'ReferencedSubsystem', packUnit);
            add_block('simulink/Ports & Subsystems/Subsystem Reference', [m '/Unpack'], 'ReferencedSubsystem', unpackUnit);
            for k = 1:nI, add_line(m, [ins{k, 1} '/1'], sprintf('Pack/%d', k), 'autorouting', 'smart'); end
            % the frame modifier
            add_block('simulink/Signal Routing/Bus Selector', [m '/Sel'], 'OutputSignals', 'id,len,data,valid');
            add_line(m, 'Pack/1', 'Sel/1', 'autorouting', 'smart');
            add_block('simulink/Math Operations/Sum', [m '/Id'], 'Inputs', '++', 'OutDataTypeStr', 'uint32');
            add_line(m, 'Sel/1', 'Id/1'); add_line(m, 'id_add/1', 'Id/2');
            add_block('simulink/Signal Routing/Switch', [m '/Len'], 'Criteria', 'u2 > Threshold', 'Threshold', '0');
            add_line(m, 'len_set/1', 'Len/1'); add_line(m, 'len_set/1', 'Len/2'); add_line(m, 'Sel/2', 'Len/3');
            add_block('simulink/Logic and Bit Operations/Logical Operator', [m '/NotDrop'], 'Operator', 'NOT', 'Inputs', '1');
            add_block('simulink/Logic and Bit Operations/Logical Operator', [m '/Valid'], 'Operator', 'AND', 'Inputs', '2');
            add_line(m, 'drop/1', 'NotDrop/1'); add_line(m, 'Sel/4', 'Valid/1'); add_line(m, 'NotDrop/1', 'Valid/2');
            add_block('simulink/Signal Routing/Bus Creator', [m '/Frame'], 'Inputs', '4', 'OutDataTypeStr', ['Bus: ' frameBus], ...
                'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
            h = add_line(m, 'Id/1', 'Frame/1'); set_param(h, 'Name', 'id');          % the Bus Creator takes the element
            h = add_line(m, 'Len/1', 'Frame/2'); set_param(h, 'Name', 'len');        % names from its input lines
            add_line(m, 'Sel/3', 'Frame/3');
            h = add_line(m, 'Valid/1', 'Frame/4'); set_param(h, 'Name', 'valid');
            add_line(m, 'Frame/1', 'Unpack/1', 'autorouting', 'smart');
            % outputs: the receiver's, then the two frames
            for k = 1:numel(outs)               % o_<name>: an output may share its name with an input
                add_block('simulink/Sinks/Out1', [m '/o_' outs{k}], 'Port', num2str(k));
                add_line(m, sprintf('Unpack/%d', k), ['o_' outs{k} '/1'], 'autorouting', 'smart');
            end
            add_block('simulink/Sinks/Out1', [m '/frame_tx'], 'Port', num2str(numel(outs) + 1), 'OutDataTypeStr', ['Bus: ' frameBus]);
            add_block('simulink/Sinks/Out1', [m '/frame_rx'], 'Port', num2str(numel(outs) + 2), 'OutDataTypeStr', ['Bus: ' frameBus]);
            add_line(m, 'Pack/1', 'frame_tx/1', 'autorouting', 'smart'); add_line(m, 'Frame/1', 'frame_rx/1', 'autorouting', 'smart');
            % any unpack output beyond the ones named is terminated
            ph = get_param([m '/Unpack'], 'PortHandles');
            for k = numel(outs) + 1:numel(ph.Outport)
                add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', m, k));
                add_line(m, sprintf('Unpack/%d', k), sprintf('T%d/1', k), 'autorouting', 'smart');
            end
        end

        function o = runBms(tel, ts, t)
            % bms_can_pack (the BMS firmware) -> Bus_Can1_Bms -> bms_can_unpack (the MCU firmware)
            m = 'tmp_T07_bms';
            if bdIsLoaded(m), close_system(m, 0); end
            new_system(m);
            shaddad_configsets('attach', m);
            add_block('simulink/Sources/In1', [m '/telem'], 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Telemetry', ...
                'SampleTime', num2str(ts, 17));
            add_block('simulink/Ports & Subsystems/Subsystem Reference', [m '/Pack'], 'ReferencedSubsystem', 'bms_can_pack');
            add_block('simulink/Ports & Subsystems/Subsystem Reference', [m '/Unpack'], 'ReferencedSubsystem', 'bms_can_unpack');
            add_line(m, 'telem/1', 'Pack/1'); add_line(m, 'Pack/1', 'Unpack/1');
            nm = {'status', 'frame_valid', 'extra'};
            for k = 1:3
                add_block('simulink/Sinks/Out1', [m '/' nm{k}], 'Port', num2str(k));
                add_line(m, sprintf('Unpack/%d', k), [nm{k} '/1']);
            end
            add_block('simulink/Sinks/Out1', [m '/frames'], 'Port', '4', 'OutDataTypeStr', 'Bus: Bus_Can1_Bms');
            add_line(m, 'Pack/1', 'frames/1');
            ds = Simulink.SimulationData.Dataset; ds = ds.addElement(tel, 'telem');
            in = Simulink.SimulationInput(m);
            in = in.setExternalInput(ds);
            in = in.setModelParameter('StopTime', sprintf('%.6f', t(end)), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
            r = sim(in);
            if ~isempty(r.ErrorMessage), error('t_t07_frames:sim', '%s', r.ErrorMessage); end
            o = struct();
            for k = {'status', 'frame_valid', 'extra', 'frames'}, o.(k{1}) = t_t07_frames.logged(r.yout, [m '/' k{1}]); end
        end

        function s = busStim(busName, t, ovr)
            % a struct of timeseries for every element of the bus: a deterministic pattern per type (minimum,
            % maximum, zero, edge and middle values), with the elements named in ovr set to those columns
            B = evalin('base', busName); N = numel(t); s = struct();
            for k = 1:numel(B.Elements)
                e = B.Elements(k); d = prod(e.Dimensions);
                v = t_t07_frames.pattern(e.DataType, N, d, k);
                if isfield(ovr, e.Name)
                    v = cast(repmat(ovr.(e.Name)(:), 1, d), e.DataType);
                end
                s.(e.Name) = timeseries(v, t);
            end
        end

        function v = pattern(type, N, d, k)
            i = (0:N-1)'; j = 0:d-1; idx = mod(i + j + k, 8) + 1;
            switch type
                case 'boolean'
                    v = logical(mod(i + j + k, 2));
                case 'single'
                    b = single([1.5 -2.25 1000.125 -0.0078125 3.4e38 -3.4e38 1.0e-40 0]);
                    v = b(idx);
                case 'double'
                    v = double(i + 10*j + k);
                otherwise
                    lo = double(intmin(type)); hi = double(intmax(type));
                    b = [lo hi 0 1 floor((lo + hi)/2) hi - 1 lo + 1 37];
                    v = cast(b(idx), type);
            end
            if d == 1, v = v(:); end
        end

        function r = rnd(x, type)
            % the CAN2 codecs' conversion (Data Type Conversion RndMeth Round): nearest, ties away from zero; saturate
            r = double(x); r = round(r);
            r = min(max(r, double(intmin(type))), double(intmax(type)));
            r = cast(r, type);
        end

        function r = rndn(x, type)
            % the CAN1 codecs' conversion (RndMeth Nearest): nearest, ties toward +inf; saturate. It differs from rnd
            % only on an exact negative tie (finding T07-FRM-04)
            r = floor(double(x) + 0.5);
            r = min(max(r, double(intmin(type))), double(intmax(type)));
            r = cast(r, type);
        end

        function b = le(v, type)
            % little-endian bytes of a column of integers, one row per sample
            v = cast(v(:), type); n = numel(typecast(v(1), 'uint8'));
            b = reshape(typecast(v, 'uint8'), n, []).';
        end

        function v = rd(D, off, type)
            % the field at byte offset off of each row of D, little-endian
            n = numel(typecast(zeros(1, 1, type), 'uint8'));
            B = uint8(D(:, off + 1:off + n)).';
            v = double(typecast(B(:), type));
        end

        function D = rows(data, width)
            D = squeeze(data);
            if size(D, 2) ~= width && size(D, 1) == width, D = D.'; end
            D = double(D);
        end

        function closeAll()
            for m = {'tmp_T07_implcmd', 'tmp_T07_implfb', 'tmp_T07_bmsreq', 'tmp_T07_bms', 'tmp_T07_can2_comp_cmd', ...
                    'tmp_T07_can2_mcu_status', 'tmp_T07_can2_mcu_imu', 'tmp_T07_can2_mcu_odom', 'tmp_T07_can2_mcu_power'}
                if bdIsLoaded(m{1}), close_system(m{1}, 0); end
            end
        end
    end
end
