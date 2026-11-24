classdef t_rc_link < matlab.unittest.TestCase
    %T_RC_LINK  T07 acceptance tests of the RC link simulation (build plan T07; owner electronics / rc).
    %
    %   Benches (built by scripts/build/build_rc_link.m):
    %     tests/rc/tb_rc_link.slx           root op / flt (0.01 s, external input) -> RC_Link_Sim
    %     tests/rc/tb_rc_sbus_encoder.slx   random full-range Bus_RcFrame -> rc_sbus_encoder
    %     tests/rc/tb_rc_link_speed.slx     constant op / flt -> RC_Link_Sim -> root Outports (speed)
    %     tests/rc/tb_rc_uart_line.slx      rc_uart_line_sim.slx unit (bytes 1..25 every frame period)
    %   Helpers: rc_tb_input (scenario Dataset), rc_tb_run (simulate + flatten outputs).
    %   Parameter variants are passed per run with Simulink.SimulationInput.setVariable('P', ...), so
    %   the shared base workspace is never modified. Measured values are written to the test log
    %   (verbosity 1) and collected in the static property-free struct returned by t_rc_link.results.
    %
    %   Timing conventions: rc_frame / sbus are the 1 ms ports; a receiver frame sampled at the frame
    %   time k*Tf leaves through the deterministic frame -> 1 ms Rate Transition at k*Tf + Tf (one-frame
    %   radio latency, D37), so "frame time" = port time - Tf. A 0.01 s scenario edge at t reaches the
    %   1 ms rate at t + 0.01 (deterministic 0.01 -> 0.001 RT, D32).

    properties
        P       % base P with P.rc refreshed from params/param_rc.m
        Tf      % SBUS frame period (s)
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_RcFrame'',''var'')')
                shaddad_params();
            end
            P = evalin('base', 'P');
            P.rc = param_rc();
            tc.P = P;
            tc.Tf = P.rc.frame_period;
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            need = {'tests/rc/tb_rc_link.slx', 'tests/rc/tb_rc_sbus_encoder.slx', 'tests/rc/tb_rc_link_speed.slx', ...
                'tests/rc/tb_rc_uart_line.slx', 'subsystems/rc/RC_Link_Sim.slx'};
            if ~all(cellfun(@(f) isfile(fullfile(root, f)), need))
                build_rc_link();
            end
            if ~evalin('base', 'isfield(P.rc, ''tx_perm'')')
                evalin('base', 'P.rc = param_rc();');
            end
            tc.addTeardown(@t_rc_link.closeModels);
        end
    end

    methods (Static)
        function closeModels()
            for n = {'tb_rc_link', 'tb_rc_sbus_encoder', 'tb_rc_link_speed', 'tb_rc_uart_line'}
                if bdIsLoaded(n{1})
                    try                                  % still compiled after a failed test
                        feval(n{1}, [], [], [], 'term');
                    catch
                    end
                    close_system(n{1}, 0);
                end
            end
        end
    end

    methods (Test)
        % ------------------------------------------------------------------ reference codec
        function ref_codec_known_vectors(tc)
            b = sbus_ref_encode(992*ones(1, 16));
            pat = uint8([224 3 31 248 192 7 62 240 129 15 124]);       % 0xE0 03 1F F8 C0 07 3E F0 81 0F 7C
            tc.verifyEqual(b, [uint8(15), pat, pat, uint8(0), uint8(0)], 'centre frame');
            b = sbus_ref_encode(2047*ones(1, 16), [1 1 1 1]);
            tc.verifyEqual(b, [uint8(15), uint8(255*ones(1, 22)), uint8(15), uint8(0)], 'all ones');
            b = sbus_ref_encode([1 zeros(1, 15)]);
            tc.verifyEqual(b(2:4), uint8([1 0 0]), 'CH1 LSB is bit 0 of byte 1');
            b = sbus_ref_encode([0 1 zeros(1, 14)]);
            tc.verifyEqual(b(2:4), uint8([0 8 0]), 'CH2 LSB is bit 11 = byte 2 bit 3');
        end

        function ref_codec_roundtrip(tc)
            s = rng(7101); c = onCleanup(@() rng(s));
            N = 5000;
            ch = randi([0 2047], N, 16);
            fl = rand(N, 4) > 0.5;
            ftr = [0 4 20 36 52]; ftr = ftr(randi(5, N, 1)).';
            b = sbus_ref_encode(ch, fl, ftr);
            [ch2, f2, ok] = sbus_ref_decode(b);
            tc.verifyEqual(double(ch2), ch, 'channels');
            tc.verifyEqual([f2.ch17 f2.ch18 f2.frame_lost f2.failsafe], fl, 'flags');
            tc.verifyTrue(all(ok), 'valid SBUS / SBUS2 footers accepted');
            bad = b(1:3, :); bad(1, 1) = 14; bad(2, 25) = 1; bad(3, 24) = bitor(bad(3, 24), 16);
            [~, ~, okb] = sbus_ref_decode(bad);
            tc.verifyFalse(any(okb), 'bad header / footer / reserved flag bit rejected');
        end

        % ------------------------------------------------------------------ encoder unit bench
        function encoder_bit_exact_full_range(tc)
            N = 2000;
            T = (N + 2)*tc.Tf;
            mdl = 'tb_rc_sbus_encoder';
            if ~bdIsLoaded(mdl), load_system(mdl); end
            in = Simulink.SimulationInput(mdl);
            in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
            out = sim(in);
            y = out.yout;
            fb = y.get('frame_bytes').Values; fn = y.get('frame_new').Values; fr = y.get('frame').Values;
            t1 = fb.Time; B = local_rows(fb.Data, numel(t1)); newk = find(logical(fn.Data(:)));
            tF = fr.ch_raw.Time; C = double(local_rows(fr.ch_raw.Data, numel(tF)));
            FL = [fr.ch17.Data(:) fr.ch18.Data(:) fr.frame_lost.Data(:) fr.failsafe.Data(:)];
            % frame released at 1 ms tick t = frame time + Tf
            kF = round((t1(newk) - tc.Tf)/tc.Tf) + 1;
            okk = kF >= 1 & kF <= numel(tF);
            newk = newk(okk); kF = kF(okk);
            ref = sbus_ref_encode(C(kF, :), FL(kF, :));
            nBad = sum(any(B(newk, :) ~= ref, 2));
            [chd, fld] = sbus_ref_decode(B(newk, :));
            nBadDec = sum(any(double(chd) ~= C(kF, :), 2) | any([fld.ch17 fld.ch18 fld.frame_lost fld.failsafe] ~= FL(kF, :), 2));
            tc.log(1, sprintf('encoder bench: %d frames (channels 0..2047, random flags), %d byte mismatches vs sbus_ref_encode, %d decode mismatches', ...
                numel(newk), nBad, nBadDec));
            tc.verifyGreaterThanOrEqual(numel(newk), N, 'frames compared');
            tc.verifyEqual(nBad, 0, 'encoder bytes == sbus_ref_encode');
            tc.verifyEqual(nBadDec, 0, 'sbus_ref_decode(encoder bytes) == input frame');
            tc.verifyEqual(max(C(:)), 2047); tc.verifyEqual(min(C(:)), 0);
        end

        % ------------------------------------------------------------------ RC_Link_Sim end to end
        function link_2000_random_frames(tc)
            % random operator commands, p_loss 0.2 (flags exercised): every SBUS frame on the line
            % decodes (sbus_ref_decode) bit-exactly to the receiver frame; frame period; loss fraction
            s = rng(7102); c = onCleanup(@() rng(s));
            T = 28.2;
            n = round(T/0.01) + 1;
            op = struct();
            for f = {'stick_thr', 'stick_steer', 'stick_aux_lv', 'stick_aux_lh', 'knob_speed'}
                op.(f{1}) = 2*rand(n, 1) - 1;
            end
            for f = {'sw_mode', 'sw_impl', 'sw_lift', 'sw_mission'}, op.(f{1}) = randi([0 3], n, 1); end
            for f = {'sw_arm', 'sw_lights', 'sw_estop', 'btn_reset', 'btn_rth'}, op.(f{1}) = randi([0 1], n, 1); end
            P = tc.P; P.rc.p_loss = 0.2;
            r = rc_tb_run(T, op, struct(), P);
            G = local_groups(r);
            % frames with 25 bytes, compared with the receiver frame (rc_frame port) at the first-byte tick
            i0 = G.start(G.n == 25);
            F = G.frames(G.n == 25, :);
            FL = [r.ch17(i0) r.ch18(i0) r.frame_lost(i0) r.failsafe(i0)];
            [chd, fld, okd] = sbus_ref_decode(F);
            badDec = any(chd ~= r.ch_raw(i0, :), 2) | any([fld.ch17 fld.ch18 fld.frame_lost fld.failsafe] ~= FL, 2);
            badEnc = any(F ~= sbus_ref_encode(double(r.ch_raw(i0, :)), FL), 2);
            per = diff(r.t(i0));
            Tmean = (r.t(i0(end)) - r.t(i0(1)))/(numel(i0) - 1);
            lostFrac = mean(fld.frame_lost(2:end));          % first frame: TX off in the RT initial condition
            prop = double(r.ch_raw(i0, 1:14));
            tc.log(1, sprintf(['link: %d frames (%d groups, %d not 25 bytes), %d decode mismatches, %d encode mismatches, ' ...
                '%d invalid; frame period mean %.6f ms (first-byte ticks, min %.0f max %.0f ms); frame_lost fraction %.4f ' ...
                '(p_loss 0.2); raw range %d..%d'], numel(i0), numel(G.n), sum(G.n ~= 25), sum(badDec), sum(badEnc), ...
                sum(~okd), 1e3*Tmean, 1e3*min(per), 1e3*max(per), lostFrac, min(prop(:)), max(prop(:))));
            tc.verifyGreaterThanOrEqual(numel(i0), 2000, '>= 2000 frames');
            tc.verifyEqual(sum(G.n ~= 25), 0, 'every frame is 25 bytes on the line');
            tc.verifyEqual(sum(badDec), 0, 'bit-exact through sbus_ref_decode');
            tc.verifyEqual(sum(badEnc), 0, 'bit-exact vs sbus_ref_encode');
            tc.verifyTrue(all(okd), 'header / footer / reserved bits valid');
            tc.verifyLessThanOrEqual(abs(1e3*Tmean - 14.000), 0.001, 'frame period 14.000 +/- 0.001 ms');
            tc.verifyLessThanOrEqual(abs(lostFrac - 0.20), 0.03, 'frame_lost fraction 0.20 +/- 0.03');
            tc.verifyGreaterThanOrEqual(min(prop(:)), 172); tc.verifyLessThanOrEqual(max(prop(:)), 1811);
        end

        function bytes_per_tick(tc)
            % (1) aligned frame start (phase0 = 0, no drift): 8 / 8 / 9 bytes per 1 ms tick.
            % (2) default P.rc.uart (phase0 0.37 ms, 200 ppm drift) over more than one full wrap of the
            %     drifting sub-ms phase: a 3 ms frame that starts inside a tick spans 4 ticks, so the
            %     chunks are partial / 8-9 / 8-9 / partial. Every tick of every frame must carry exactly
            %     the reference count of bytes completed in it (byte i of release n completes at
            %     n*Tf + ph(n) + i*Tb, ph(n) by the sbus_uart_line_sim recursion), 25 bytes per frame,
            %     <= 9 per tick, 8 or 9 on every tick that lies fully inside a frame, none between frames.
            P = tc.P; P.rc.p_loss = 0; P.rc.uart.phase0 = 0; P.rc.uart.drift_ppm = 0;
            r = rc_tb_run(1.0, struct(), struct(), P);
            G = local_groups(r);
            pat = local_patterns(r, G);
            okPat = cellfun(@(p) isequal(p, [8 8 9]), pat);
            C1 = local_chunk_check(r, P.rc.uart, tc.Tf);
            U = tc.P.rc.uart;
            T2 = local_wrap_time(U, tc.Tf);
            r2 = rc_tb_run(T2, struct(), struct(), tc.P);
            G2 = local_groups(r2);
            pat2 = local_patterns(r2, G2);
            C2 = local_chunk_check(r2, U, tc.Tf);
            inner = cellfun(@(p) p(2:end-1), pat2(cellfun(@numel, pat2) >= 3), 'UniformOutput', false);
            inner = [inner{:}];
            s2 = cellfun(@(p) mat2str(p), pat2, 'UniformOutput', false);
            [u, ~, j] = unique(s2);
            cnt = accumarray(j(:), 1);
            tc.log(1, sprintf(['bytes/tick phase0 = 0, drift 0: %d of %d frames [8 8 9], %d of %d releases equal the reference ' ...
                'chunking. Default phase0 %.2f ms, drift %g ppm, %.1f s (%d releases, phase %.3f..%.3f ms, %d wrap(s)): ' ...
                '%d of %d releases equal the reference chunking (%d at a phase-wrap edge), %d bytes between frames, ' ...
                'full ticks inside a frame carry %s bytes; patterns %s'], sum(okPat), numel(pat), sum(C1.match), numel(C1.match), ...
                1e3*U.phase0, U.drift_ppm, T2, numel(C2.match), 1e3*min(C2.ph), 1e3*max(C2.ph), sum(diff(C2.ph) < 0), ...
                sum(C2.match), numel(C2.match), sum(C2.alt), C2.idle_bytes, mat2str(unique(inner)), ...
                strjoin(strcat(u(:)', 'x', arrayfun(@num2str, cnt(:)', 'UniformOutput', false)), ', ')));
            tc.verifyTrue(all(okPat), 'bytes per tick 8/8/9 with an aligned frame start');
            tc.verifyTrue(all(C1.match), 'aligned start: chunks == reference');
            tc.verifyGreaterThanOrEqual(sum(diff(C2.ph) < 0), 1, 'default run covers a full wrap of the sub-ms phase');
            tc.verifyTrue(all(C2.match), 'default phase: every chunk == reference chunking of t0 + i*Tb');
            tc.verifyEqual(C2.idle_bytes, 0, 'no bytes between frames');
            tc.verifyTrue(all(G2.n == 25), '25 bytes per frame (default phase)');
            tc.verifyLessThanOrEqual(max(cellfun(@max, pat2)), 9, 'never more than 9 bytes per tick (100 kbit/s, 12 bit)');
            tc.verifyTrue(all(inner == 8 | inner == 9), '8 or 9 bytes on every full tick inside a frame');
        end

        function tx_to_first_byte_latency(tc)
            % TX frame sample k*Tf -> first SBUS byte of that frame, for EVERY frame over more than one full
            % wrap of the drifting sub-ms phase (default P.rc.uart). A strictly increasing throttle ramp gives
            % every TX frame its own CH2 value, so each burst on the line is matched to the TX frame that
            % sampled it (TX frame k samples the 0.01 s input j = floor(k*Tf/0.01) - 1, D32 input RT).
            %   line level: u = first-byte completion time - k*Tf. Byte i completes at k*Tf + u + (i-1)*Tb and
            %     is counted on the first 1 ms tick at or after it, so the observed per-tick counts bound u to
            %     (lo, hi]. Criterion: (lo, hi] inside 14 ms + phase0 +/- 1 ms; it must also contain the
            %     reference Tf + ph + Tb (ph = the drifting sub-ms phase of that release).
            %   port level (1 ms ticks): first tick with bytes - k*Tf = ceil(u / 1 ms), i.e. 15 or 16 ms;
            %     must equal the reference for every frame and lie in [Tf, Tf + 2 ms].
            P = tc.P; P.rc.p_loss = 0;
            U = P.rc.uart; Tf = tc.Tf; Tms = 1e-3;
            T = local_wrap_time(U, Tf);
            tt = (0:round(T/0.01))'*0.01;
            op.stick_thr = -1 + 2*tt/T;                        % >= 2 raw counts per 0.01 s sample
            r = rc_tb_run(T, op, struct(), P);
            G = local_groups(r);
            ok25 = G.n == 25;
            [chd, fld] = sbus_ref_decode(G.frames(ok25, :));
            gSt = G.start(ok25); gEn = G.stop(ok25);
            thr = double(chd(:, P.rc.ch_map.thr));
            raw = @(x) min(max(P.rc.raw_mid + round(P.rc.raw_half*x), P.rc.raw_min), P.rc.raw_max);
            kAll = (1:floor((T - 6e-3)/Tf) - 1)';             % TX frames whose burst ends inside the run
            jIn = floor(kAll*Tf/0.01 + 1e-9) - 1;              % 0.01 s input sample seen by TX frame k
            rawTx = raw(op.stick_thr(jIn + 1));
            ph = local_ref_phase(U, Tf, kAll(end) + 1);         % release k + 1 carries TX frame k
            nK = numel(kAll);
            nHit = zeros(nK, 1); latP = nan(nK, 1); lo = nan(nK, 1); hi = nan(nK, 1);
            okRef = false(nK, 1); okPort = false(nK, 1); edge = false(nK, 1);
            for q = 1:nK
                k = kAll(q);
                g = find(thr == rawTx(q) & ~fld.frame_lost);
                nHit(q) = numel(g);
                if nHit(q) ~= 1, continue, end
                rows = (gSt(g) - 1):gEn(g);                     % the idle tick before the burst, then the burst
                N = cumsum(r.len(rows)); N = N - N(1);
                [lo(q), hi(q)] = local_first_byte_bounds(r.t(rows) - k*Tf, N, U.Tb);
                latP(q) = r.t(gSt(g)) - k*Tf;
                cand = local_phase_candidates(ph(k + 1));
                edge(q) = numel(cand) > 1;
                uRef = Tf + cand + U.Tb;
                okRef(q) = any(uRef > lo(q) - 1e-9 & uRef <= hi(q) + 1e-9);
                okPort(q) = any(abs(latP(q) - ceil((uRef - 1e-9)/Tms)*Tms) < 1e-9);
            end
            phK = ph(kAll + 1);
            nom = Tf + U.phase0;
            n15 = sum(abs(latP - 15e-3) < 1e-9); n16 = sum(abs(latP - 16e-3) < 1e-9);
            dev = latP - nom; [~, iw] = max(abs(dev));
            tc.log(1, sprintf(['TX sample -> first byte, %d frames over %.1f s (phase %.3f..%.3f ms, %d wrap(s), %d at a ' ...
                'wrap edge): line level u in %.3f..%.3f ms (criterion 14 ms + phase0 %.2f ms +/- 1 = %.2f..%.2f ms; ' ...
                'widest bound %.3f ms; %d of %d contain the reference Tf + ph + Tb); port (1 ms tick) %s ms: 15 ms x%d, ' ...
                '16 ms x%d, %d of %d equal ceil(u); port vs 14 ms + phase0: worst %+.2f ms'], nK, T, 1e3*min(phK), ...
                1e3*max(phK), sum(diff(phK) < 0), sum(edge), 1e3*min(lo), 1e3*max(hi), 1e3*U.phase0, 1e3*(nom - Tms), ...
                1e3*(nom + Tms), 1e3*max(hi - lo), sum(okRef), nK, mat2str(1e3*unique(round(latP/Tms)*Tms)'), n15, n16, ...
                sum(okPort), nK, 1e3*dev(iw)));
            tc.verifyEqual(nHit, ones(nK, 1), 'every TX frame appears exactly once on the line');
            tc.verifyGreaterThanOrEqual(nK, ceil(1e-3/(Tf*U.drift_ppm*1e-6)), 'frames cover a full wrap of the phase');
            tc.verifyGreaterThanOrEqual(sum(diff(phK) < 0), 1, 'the phase wraps inside the run');
            tc.verifyTrue(all(hi > lo), 'per-tick counts consistent with bytes every Tb');
            tc.verifyGreaterThanOrEqual(min(lo), nom - Tms - 1e-9, 'first byte >= 14 ms + phase0 - 1 ms (line level, every frame)');
            tc.verifyLessThanOrEqual(max(hi), nom + Tms + 1e-9, 'first byte <= 14 ms + phase0 + 1 ms (line level, every frame)');
            tc.verifyTrue(all(okRef), 'line-level first byte = Tf + phase + Tb (every frame)');
            tc.verifyTrue(all(okPort), 'port first-byte tick = ceil((Tf + phase + Tb) / 1 ms) (every frame)');
            tc.verifyGreaterThanOrEqual(min(latP), Tf - 1e-9, 'port latency >= Tf');
            tc.verifyLessThanOrEqual(max(latP), Tf + 2*Tms + 1e-9, 'port latency <= Tf + 2 ms');
        end

        % ------------------------------------------------------------------ transmitter mapping
        function transmitter_mapping(tc)
            P = tc.P; P.rc.p_loss = 0;
            seg = 0.1;
            sticks = [1 -1 0];
            sw3 = [1 2 3 0];
            sw2 = [0 1 0];
            op = struct();
            nS = numel(sticks); n3 = numel(sw3); n2 = numel(sw2);
            tS = (0:nS-1)'*seg;
            for f = {'stick_thr', 'stick_steer', 'stick_aux_lv', 'stick_aux_lh', 'knob_speed'}
                op.(f{1}) = [tS sticks(:)];
            end
            t3 = (0:n3-1)'*seg;
            for f = {'sw_mode', 'sw_impl', 'sw_lift', 'sw_mission'}, op.(f{1}) = [t3 sw3(:)]; end
            t2 = (0:n2-1)'*seg;
            for f = {'sw_arm', 'sw_lights', 'sw_estop', 'btn_reset', 'btn_rth'}, op.(f{1}) = [t2 sw2(:)]; end
            r = rc_tb_run(0.5, op, struct(), P);
            at = @(t) find(abs(r.t - t) < 1e-9, 1);              % sample near the end of each segment
            M = P.rc.ch_map;
            stickCh = [M.steer M.thr M.aux_lv M.aux_lh M.speed];
            sw3Ch = [M.mode M.impl M.lift M.mission];
            sw2Ch = [M.arm M.lights M.estop M.reset M.rth];
            gotS = zeros(nS, 5); got3 = zeros(n3, 4); got2 = zeros(n2, 5);
            for k = 1:nS, gotS(k, :) = double(r.ch_raw(at((k - 1)*seg + 0.08), stickCh)); end
            for k = 1:n3, got3(k, :) = double(r.ch_raw(at((k - 1)*seg + 0.08), sw3Ch)); end
            for k = 1:n2, got2(k, :) = double(r.ch_raw(at((k - 1)*seg + 0.08), sw2Ch)); end
            spare = double(r.ch_raw(at(0.4), 15:16));
            tc.log(1, sprintf('sticks +1/-1/0 -> %s; CH5 sw_mode 1/2/3/0 -> %s; 2-pos 0/1/0 -> %s; spare %s', ...
                mat2str(gotS(:, 2)'), mat2str(got3(:, 1)'), mat2str(got2(:, 1)'), mat2str(spare)));
            tc.verifyEqual(gotS, repmat([1811; 172; 992], 1, 5), 'stick +1/-1/0 -> 1811/172/992 (CH1-4, CH7)');
            tc.verifyEqual(got3, repmat([172; 992; 1811; 172], 1, 4), 'sw 1/2/3 (0) -> 172/992/1811 (172) on CH5/8/11/13');
            tc.verifyEqual(got2, repmat([172; 1811; 172], 1, 5), '2-position 0/1 -> 172/1811 on CH6/10/12/9/14');
            tc.verifyEqual(spare, [992 992], 'spare channels at centre');
        end

        % ------------------------------------------------------------------ receiver failsafe
        function failsafe_modes(tc)
            % rc_link_loss from 2.0 s: frame_lost from the next frame, failsafe at 3.0 +/- 0.014 s,
            % mode 1 FLAG_PRESET (CH12 = 172), mode 2 NO_PULSES (no bytes), mode 3 HOLD
            op = struct('stick_thr', 0.5, 'stick_steer', -0.3, 'sw_mode', 3, 'sw_arm', 1, 'sw_estop', 1, 'sw_impl', 2);
            op = structfun(@(x) [0 x], op, 'UniformOutput', false);
            flt.rc_link_loss = [0 0; 2.0 1];
            T = 3.6;
            M = tc.P.rc.ch_map;
            res = struct();
            for mode = 1:3
                P = tc.P; P.rc.p_loss = 0; P.rc.rx_failsafe_mode = mode;
                r = rc_tb_run(T, op, flt, P);
                tFrame = r.t - tc.Tf;                            % frame time of the rc_frame sample
                valid = r.t >= tc.Tf;
                iLost = find(valid & r.frame_lost & r.t > 0.1, 1);
                if mode == 3                                     % HOLD: failsafe not visible in the flags
                    iFs = find(r.t >= 3.1 - 1e-9, 1);
                else
                    iFs = find(valid & r.t > 0.1 & (r.failsafe | ~r.sent), 1);
                end
                held = double(r.ch_raw(find(r.t < 1.9, 1, 'last'), :));
                res(mode).t_lost = tFrame(iLost);
                res(mode).t_fs = tFrame(iFs);
                after = r.t >= r.t(iFs);
                G = local_groups(r);
                lastByte = r.t(find(r.len > 0, 1, 'last'));
                res(mode).last_byte = lastByte;
                fsCh = double(r.ch_raw(after, :));
                switch mode
                    case 1
                        tc.verifyTrue(all(r.failsafe(after) & r.frame_lost(after) & r.sent(after)), 'mode 1: failsafe + lost flags, frames continue');
                        tc.verifyEqual(unique(fsCh(:, M.estop)), 172, 'mode 1: CH12 = 172 (RUN, D34)');
                        tc.verifyEqual(fsCh(end, :), double(tc.P.rc.failsafe_raw), 'mode 1: preset channels');
                        % the failsafe frames on the line carry the failsafe bit and CH12 = 172
                        F = G.frames(G.n == 25 & r.t(G.start) > r.t(iFs) + 0.005, :);
                        [chd, fld] = sbus_ref_decode(F);
                        tc.verifyTrue(~isempty(F) && all(fld.failsafe) && all(fld.frame_lost), 'mode 1: SBUS flags');
                        tc.verifyEqual(unique(double(chd(:, M.estop))), 172, 'mode 1: SBUS CH12 = 172');
                    case 2
                        tc.verifyFalse(any(r.sent(after)), 'mode 2: no frames sent after failsafe');
                        tc.verifyFalse(any(r.len(r.t > r.t(iFs)) > 0), 'mode 2: no bytes after the failsafe frame');
                    case 3
                        tc.verifyFalse(any(r.failsafe(after)), 'mode 3: no failsafe flag');
                        tc.verifyTrue(all(r.frame_lost(after) & r.sent(after)), 'mode 3: lost bits only, frames continue');
                        tc.verifyEqual(fsCh, repmat(held, sum(after), 1), 'mode 3: held channels');
                end
                if mode ~= 3
                    tc.verifyLessThanOrEqual(abs(res(mode).t_fs - 3.0), 0.014 + 1e-9, sprintf('mode %d: failsafe at 3.0 +/- 0.014 s', mode));
                end
                tc.verifyFalse(any(r.frame_lost(valid & r.t > 0.1 & tFrame < 2.0)), 'no lost frame before the loss');
                % first lost frame = first frame sampled after the loss reaches the 1 ms rate (2.0 + 0.01 s)
                tc.verifyEqual(res(mode).t_lost, ceil((2.01 - 1e-9)/tc.Tf)*tc.Tf, 'AbsTol', 1e-9, 'frame_lost from the next frame');
            end
            tc.log(1, sprintf(['link loss at 2.000 s (scenario; 2.010 s at 1 ms): first lost frame %.3f s; failsafe frame time ' ...
                'mode1 %.3f / mode2 %.3f s (port +%.0f ms; mode 3 HOLD shows lost bits only); mode 2 last SBUS byte at %.3f s (port time)'], ...
                res(1).t_lost, res(1).t_fs, res(2).t_fs, 1e3*tc.Tf, res(2).last_byte));
        end

        function line_silent(tc)
            % rc_line_silent from 1.98 s (scenario) = 1.990 s at 1 ms, inside a byte burst
            P = tc.P; P.rc.p_loss = 0;
            flt.rc_line_silent = [0 0; 1.98 1; 2.5 0];
            r0 = rc_tb_run(3.0, struct(), struct(), P);
            r = rc_tb_run(3.0, struct(), flt, P);
            tOn = 1.99; tOff = 2.51;
            k = find(abs(r.t - tOn) < 1e-9);
            sil = r.t >= tOn - 1e-9 & r.t < tOff - 1e-9;
            firstZero = r.t(find(r.t >= tOn - 1e-9 & r.len == 0, 1)) - tOn;
            resume = r.t(find(r.t > tOff & r.len > 0, 1));
            tc.log(1, sprintf(['line silent at 1.990 s (1 ms rate; scenario 1.980 s): unsilenced chunk at that tick %d bytes, ' ...
                'silenced %d bytes (len = 0 after %.0f ms); bytes during silence %d; resume at %.3f s'], r0.len(k), r.len(k), ...
                1e3*firstZero, sum(r.len(sil)), resume));
            tc.verifyGreaterThan(r0.len(k), 0, 'reference: the tick is inside a byte burst');
            tc.verifyEqual(r.len(k), 0, 'len = 0 at the first 1 ms tick of rc_line_silent (within 1 ms)');
            tc.verifyEqual(sum(r.len(sil)), 0, 'no bytes while silent');
            tc.verifyFalse(any(r.uart_err(sil)), 'no uart_err while silent');
            tc.verifyFalse(any(r.sent(r.t >= 2.03 & r.t < 2.5)), 'receiver: sent = false while silent (frame path)');
            tc.verifyNotEmpty(resume, 'bytes resume after the silence');
        end

        function byte_errors(tc)
            % byte error rate 1e-3: chunks flagged uart_err == chunks containing an injected (corrupted) byte
            P = tc.P; P.rc.p_loss = 0;
            T = 30;
            r0 = rc_tb_run(T, struct(), struct(), P);
            r = rc_tb_run(T, struct(), struct('rc_byte_err_rate', [0 1e-3]), P);
            tc.assertEqual(r.len, r0.len, 'same byte timing');
            diffB = r.bytes ~= r0.bytes;
            nInj = sum(diffB(:));
            injTick = any(diffB, 2);
            nFlag = sum(r.uart_err);
            nBytes = sum(r.len);
            tc.log(1, sprintf('byte errors: %d bytes on the line, %d corrupted (rate %.2e), %d chunks with errors, %d uart_err flags, %d mismatched chunks', ...
                nBytes, nInj, nInj/nBytes, sum(injTick), nFlag, sum(injTick ~= r.uart_err)));
            tc.verifyGreaterThan(nInj, 20, 'errors injected');
            tc.verifyEqual(r.uart_err, injTick, 'uart_err exactly on chunks with an injected error');
            tc.verifyEqual(nFlag, sum(injTick), 'uart_err count = injected-chunk count');
            tc.verifyEqual(sum(r0.uart_err), 0, 'no uart_err without injection');
            tc.verifyLessThanOrEqual(abs(nInj/nBytes - 1e-3), 4*sqrt(1e-3/nBytes), 'error rate');
        end

        function uart_line_unit(tc)
            % rc_uart_line_sim.slx alone: bytes 1..25 per release, in order, 25 per frame, <= 9 per tick,
            % first byte one tick after the release (phase0 + Tb < 1 ms)
            mdl = 'tb_rc_uart_line';
            if ~bdIsLoaded(mdl), load_system(mdl); end
            T = 1.0;
            in = Simulink.SimulationInput(mdl);
            in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
            out = sim(in);
            sb = out.yout.get('sbus').Values;
            r.t = sb.len.Time; r.len = double(sb.len.Data(:)); r.bytes = local_rows(sb.bytes.Data, numel(r.t));
            G = local_groups(r);
            nRel = numel(0:tc.Tf:T - 4e-3);
            firstLag = r.t(G.start(1:min(5, end)))' - (0:min(5, numel(G.start)) - 1)*tc.Tf;
            tc.log(1, sprintf('rc_uart_line_sim unit: %d frames released, %d bursts, sizes %s, first-byte lag %s ms', ...
                nRel, numel(G.n), mat2str(unique(G.n)'), mat2str(1e3*firstLag, 3)));
            tc.verifyEqual(numel(G.n), nRel, 'one burst per release');
            tc.verifyTrue(all(G.n == 25), '25 bytes per frame');
            tc.verifyEqual(double(G.frames), repmat(1:25, numel(G.n), 1), 'byte order preserved');
            tc.verifyLessThanOrEqual(max(r.len), 9, '<= 9 bytes per 1 ms tick');
            tc.verifyEqual(firstLag, 1e-3*ones(size(firstLag)), 'AbsTol', 1e-9, 'first byte one tick after the release');
        end

        % ------------------------------------------------------------------ structure / rates
        function interface_and_structure(tc)
            n = shaddad_check_interfaces('Files', {'RC_Link_Sim'}, 'Quiet', true);
            tc.verifyEqual(n, 0, 'shaddad_check_interfaces RC_Link_Sim');
            mdl = 'tb_rc_link';
            if ~bdIsLoaded(mdl), load_system(mdl); end
            rc = [mdl '/RC_Link'];
            mf = find_system(rc, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SFBlockType', 'MATLAB Function');
            ch = find_system(rc, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SFBlockType', 'Chart');
            tc.verifyEqual(numel(mf), 1, 'exactly one MATLAB Function');
            if isscalar(mf)
                cfg = get_param(mf{1}, 'MATLABFunctionConfiguration');
                tc.verifyTrue(contains(cfg.FunctionScript, 'sbus_uart_line_sim('), 'the MATLAB Function calls sbus_uart_line_sim');
            end
            tc.verifyEmpty(ch, 'no Stateflow chart');
            % RC_Link_Sim/UART_Line_Sim (inline) == subsystems/rc/rc_uart_line_sim.slx (same builder)
            wasU = bdIsLoaded('rc_uart_line_sim');
            if ~wasU, load_system('rc_uart_line_sim'); end
            sF = local_signature('rc_uart_line_sim');
            sI = local_signature([rc '/UART_Line_Sim']);
            if ~wasU && ~bdIsLoaded('tb_rc_uart_line'), close_system('rc_uart_line_sim', 0); end
            tc.verifyEqual(sI, sF, 'inline UART_Line_Sim identical to rc_uart_line_sim.slx (blocks, parameters, script)');
            loops = Simulink.BlockDiagram.getAlgebraicLoops(mdl);
            tc.verifyEmpty(loops, 'no algebraic loops in tb_rc_link');
            % compiled rates: Rate Transitions integer ratio, outputs 0.001, rates in the allowed set
            evalc('feval(mdl, [], [], [], ''compile'')');
            cl = onCleanup(@() feval(mdl, [], [], [], 'term')); %#ok<FVAL> model command form
            rts = find_system(rc, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'RateTransition');
            ratio = zeros(numel(rts), 1); desc = cell(numel(rts), 1);
            for k = 1:numel(rts)
                ph = get_param(rts{k}, 'PortHandles');
                ti = get_param(ph.Inport(1), 'CompiledSampleTime'); to = get_param(ph.Outport(1), 'CompiledSampleTime');
                ratio(k) = max(ti(1), to(1))/min(ti(1), to(1));
                desc{k} = sprintf('%s %g->%g', get_param(rts{k}, 'Name'), ti(1), to(1));
            end
            ph = get_param(rc, 'PortHandles');
            tOut = arrayfun(@(h) local_first(get_param(h, 'CompiledSampleTime')), ph.Outport);
            blks = find_system(rc, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
            rates = [];
            for k = 1:numel(blks)
                try
                    st = get_param(blks{k}, 'CompiledSampleTime');
                catch
                    continue                             % blocks without a compiled sample time
                end
                if ~iscell(st), st = {st}; end
                for j = 1:numel(st)
                    if ~isempty(st{j}) && st{j}(1) >= 0, rates(end+1) = st{j}(1); end %#ok<AGROW>
                end
            end
            rates = unique(rates);
            allowed = tc.P.sim.Ts_compiled_allowed;
            notAllowed = rates(~arrayfun(@(x) any(abs(x - allowed) < 1e-12 | (isinf(x) & isinf(allowed))), rates));
            tc.log(1, sprintf('Rate Transitions: %s; output port rates %s; compiled rates inside RC_Link_Sim %s', ...
                strjoin(desc', ', '), mat2str(tOut), mat2str(rates)));
            tc.verifyEqual(abs(ratio - round(ratio)) < 1e-9, true(size(ratio)), 'integer RT ratios');
            tc.verifyEqual(tOut, [1e-3 1e-3], 'AbsTol', 1e-12, 'RC_Link_Sim exports only 0.001 ports');
            tc.verifyEmpty(notAllowed, 'compiled rates in P.sim.Ts_compiled_allowed');
        end

        function real_time_factor(tc)
            % tests/rc/tb_rc_link_speed (constant op with TX on / flt, no logging), Accelerator (D56).
            % RC_Link_Sim cost = bench - same bench with RC_Link commented out (engine fixed cost per step).
            mdl = 'tb_rc_link_speed';
            if ~bdIsLoaded(mdl), load_system(mdl); end
            T = 60;
            in = Simulink.SimulationInput(mdl);
            in = in.setModelParameter('StopTime', sprintf('%g', T), 'SimulationMode', 'accelerator', ...
                'SaveOutput', 'off', 'SaveTime', 'off', 'SignalLogging', 'off');
            w = @() local_best(in, 3);
            tFull = w();
            set_param([mdl '/RC_Link'], 'Commented', 'on');
            cl = onCleanup(@() set_param([mdl '/RC_Link'], 'Commented', 'off'));
            tFloor = w();
            set_param([mdl '/RC_Link'], 'Commented', 'off');
            inN = in.setModelParameter('SimulationMode', 'normal');
            tNorm = local_best(inN, 2);
            rtfMarg = T/max(tFull - tFloor, eps);
            tc.log(1, sprintf(['RTF: Accelerator bench %.0fx (%.2f us/step), engine floor %.0fx (%.2f us/step), RC_Link_Sim ' ...
                'marginal %.0fx (%.2f us/step); Normal bench %.0fx'], T/tFull, 1e3*tFull/T, T/tFloor, 1e3*tFloor/T, ...
                rtfMarg, 1e3*(tFull - tFloor)/T, T/tNorm));
            tc.verifyGreaterThanOrEqual(rtfMarg, 200, 'RC_Link_Sim >= 200x real time (marginal cost, Accelerator)');
        end
    end
end

% =============================================================================================
function A = local_rows(D, n)
if size(D, 1) == n && ismatrix(D), A = D; else, A = reshape(D, [], n).'; end
end

function G = local_groups(r)
% contiguous byte bursts on the line (frames are separated by >= 9 idle ticks)
act = r.len > 0;
st = find(act & ~[false; act(1:end-1)]);
en = find(act & ~[act(2:end); false]);
G.start = st; G.stop = en; G.n = zeros(numel(st), 1); G.frames = zeros(numel(st), 25, 'uint8');
for g = 1:numel(st)
    rows = st(g):en(g);
    b = [];
    for k = rows, b = [b, r.bytes(k, 1:r.len(k))]; end %#ok<AGROW>
    G.n(g) = numel(b);
    if numel(b) == 25, G.frames(g, :) = b; end
end
end

function pat = local_patterns(r, G)
pat = cell(numel(G.start), 1);
for g = 1:numel(G.start), pat{g} = r.len(G.start(g):G.stop(g)).'; end
end

function T = local_wrap_time(U, Tf)
% run length (s) covering more than one full wrap of the drifting sub-ms frame-start phase
% (default 200 ppm: 1 ms / 2.8 us = 358 frames = 5.0 s -> 6.0 s)
step = Tf*abs(U.drift_ppm)*1e-6;
if step > 0, nWrap = ceil(1e-3/step); else, nWrap = 0; end
T = max(2, ceil((nWrap + 40)*Tf/0.5)*0.5);
end

function ph = local_ref_phase(U, Tf, N)
% sub-ms frame-start phase of releases 1..N: the recursion of sbus_uart_line_sim (same operations)
ph = zeros(N, 1);
p = U.phase0;
for n = 1:N
    p = mod(p + Tf*U.drift_ppm*1e-6, 1e-3);
    ph(n) = p;
end
end

function cand = local_phase_candidates(p)
% the phase and, when it sits on the 1 ms wrap (exactly 0 or 1 ms up to round-off), its other branch:
% the recursion may land just below 1 ms or just above 0 there (identical byte spacing, one tick apart)
cand = p;
if p < 1e-9, cand(end+1) = p + 1e-3; end
if p > 1e-3 - 1e-9, cand(end+1) = p - 1e-3; end
end

function c = local_ref_chunks(p, Tb)
% bytes counted on ticks 1..5 ms after a release with phase p (byte i completes at p + i*Tb and is
% counted on the first tick at or after it, 1 ns tolerance as in sbus_uart_line_sim)
N = arrayfun(@(tau) sum(p + (1:25)*Tb <= tau + 1e-9), (1:5)*1e-3);
c = diff([0 N]);
end

function C = local_chunk_check(r, U, Tf)
% per-tick byte counts r.len vs the reference chunking of every release n (released at n*Tf); rows of
% r are the 1 ms ticks 0, 1, 2, ... ms. C.match(n): release n equals the reference (C.alt: with the
% other branch of a wrap-edge phase); C.idle_bytes: bytes on ticks outside every release window.
ms = round(r.t/1e-3);
assert(isequal(ms(:), (0:numel(ms) - 1)'), 't_rc_link:ticks', 'rows are not consecutive 1 ms ticks');
tf = round(Tf/1e-3);
nRel = floor((ms(end) - 5)/tf);
C.ph = local_ref_phase(U, Tf, nRel);
C.match = false(nRel, 1); C.alt = false(nRel, 1);
covered = false(numel(ms), 1);
for n = 1:nRel
    rows = n*tf + (1:5) + 1;
    covered(rows) = true;
    got = r.len(rows).';
    cand = local_phase_candidates(C.ph(n));
    for c = 1:numel(cand)
        if isequal(got, local_ref_chunks(cand(c), U.Tb))
            C.match(n) = true; C.alt(n) = c > 1;
            break
        end
    end
end
covered(ms > nRel*tf + 5) = true;                    % tail after the last complete window
C.idle_bytes = sum(r.len(~covered));
end

function [lo, hi] = local_first_byte_bounds(tau, N, Tb)
% bounds lo < u <= hi on the first-byte completion time u (origin of tau) from the cumulative byte
% counts N(tau) on 1 ms ticks tau: byte i completes at u + (i-1)*Tb, counted on the first tick at or after
lo = -inf; hi = inf;
for m = 1:numel(tau)
    n = N(m);
    if n < 25, lo = max(lo, tau(m) - n*Tb); end       % byte n+1 not yet complete: u + n*Tb > tau
    if n > 0, hi = min(hi, tau(m) - (n - 1)*Tb); end  % byte n complete: u + (n-1)*Tb <= tau
end
end

function s = local_signature(root)
% sorted text signature of every block below root: relative path, BlockType, dialog parameter values
% (character values only) and MATLAB Function scripts
blks = find_system(root, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
blks = setdiff(blks, {root});
s = cell(numel(blks), 1);
for k = 1:numel(blks)
    b = blks{k};
    rel = b(numel(root) + 2:end);
    dps = get_param(b, 'DialogParameters');
    if isstruct(dps), dp = fieldnames(dps); else, dp = {}; end
    vals = {};
    for j = 1:numel(dp)
        try
            v = get_param(b, dp{j});
        catch
            continue
        end
        if ischar(v), vals{end+1} = [dp{j} '=' v]; end %#ok<AGROW>
    end
    if strcmp(get_param(b, 'BlockType'), 'SubSystem') && strcmp(get_param(b, 'SFBlockType'), 'MATLAB Function')
        c = get_param(b, 'MATLABFunctionConfiguration');
        vals{end+1} = ['script=' c.FunctionScript]; %#ok<AGROW>
    end
    s{k} = strjoin([{rel, get_param(b, 'BlockType')}, vals], '|');
end
s = sort(s);
end

function v = local_first(x)
if iscell(x), x = x{1}; end
v = x(1);
end

function t = local_best(in, n)
t = inf;
for k = 1:n
    out = sim(in);
    t = min(t, out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime);
end
end
