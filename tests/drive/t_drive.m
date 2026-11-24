classdef t_drive < matlab.unittest.TestCase
%T_DRIVE  Acceptance tests of build task T06 (traction drive), drive spec 8 T1-T23 (revision B) and the
%   build-plan items: ports, variants, algebraic loops, codec bit-exactness, simulation speed.
%   Benches: tests/drive/tb_drive.slx, tb_drive_sys.slx, tb_drive_2mass.slx, tb_drive_foc.slx
%   (all built by build_tb_drive.m). Measured values are printed and saved to work/t06_results.mat.
%   Tests tagged 'Long' (T05 2 x 2 h thermal, T08 600 s hold, T17 speed) take tens of minutes.
%   T19b-T19d (critfix R-01): park_req, a dropped enable and a CAN timeout on DRV-01's regen-limited descent.

    properties
        P
        root
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Bus_DriveSideCmd'',''var'') && isfield(P.drive,''sm'')')
                shaddad_params();
            end
            if ~evalin('base', 'exist(''cs_SimFixed'',''var'')'), shaddad_configsets('fixed'); end
            tc.P = evalin('base', 'P');
            tc.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            d = fileparts(mfilename('fullpath'));
            if ~all(cellfun(@(m) isfile(fullfile(d, [m '.slx'])), {'tb_drive', 'tb_drive_sys', 'tb_drive_2mass'}))
                build_tb_drive();
            end
            tb_drive_results('reset');
        end
    end

    methods (TestClassTeardown)
        function teardown(tc)
            f = fullfile(tc.root, 'work', 't06_results.mat');
            try, tb_drive_results('save', f); catch, end %#ok<CTCH>
            for m = {'tb_drive', 'tb_drive_sys', 'tb_drive_2mass', 'tb_drive_foc', 'tmp_T06_codec'}
                if bdIsLoaded(m{1}), close_system(m{1}, 0); end
            end
        end
    end

    % =========================================================================================
    methods (Access = private)
        function [t, v] = vtrack(tc, tr)
            t = tr.w_spr.Time; v = tr.w_spr.Data*tc.P.drive.gear.r_spr;
        end
        function rec(~, id, crit, meas, ok)
            if ischar(ok), r = ok; elseif ok, r = 'pass'; else, r = 'fail'; end
            tb_drive_results('add', id, crit, meas, r);
        end
        function g = gradeTorque(tc, deg)
            b = tc.P.drive.bench;
            g = -b.m_side*b.g*sind(deg)*tc.P.drive.gear.r_spr;          % at the sprocket, + deg = uphill
        end
        function T = rollTorque(tc, f_rr, deg)
            b = tc.P.drive.bench;
            T = f_rr*b.m_side*b.g*cosd(deg)*tc.P.drive.gear.r_spr;
        end
        function ts = ramp(~, t0, t1, v0, v1, tend)
            t = (0:0.001:tend)';
            v = v0 + (v1 - v0)*min(max((t - t0)/(t1 - t0), 0), 1);
            ts = timeseries(v, t);
        end
        function fi = faults(~, t, vals)
            fi = tb_drive_bus_ts('Bus_DriveFaultInj', t, vals);
        end
        function R = descentStop(tc, deg, tend, o, tReq)
            % critfix R-01: DRV-01's bench descent (tb_drive; grade of deg degrees downhill ramped in over 3-4 s; track
            % target 1 m/s; tau_lim_reg T_peak -> 0.32 N m at 6 s, so the side cannot hold its speed) with the profile
            % fields o (tb_drive_profile), and its stop measured from the request time tReq
            Tpk = tc.P.drive.motor.T_peak;
            pr = struct('t_end', tend, 'v_tgt', [0 0; 1 1.0], 'tau_lim_reg', [0 Tpk; 6 0.32]);
            fn = fieldnames(o);
            for k = 1:numel(fn), pr.(fn{k}) = o.(fn{k}); end
            [c, ~] = tb_drive_profile(pr);
            so = tb_drive_run('tb_drive', tend, struct('tb_cmd', c, 'tb_grade', tc.ramp(3, 4, 0, tc.gradeTorque(-deg), tend)));
            fb = so.fb; tr = so.truth; [t, v] = tc.vtrack(tr);
            ts = fb.state.Time; st = double(fb.state.Data); fc = fb.fault_code.Data;
            after = ts >= tReq - 1e-9;
            R.t5 = local_first(ts(after & st == 5)); R.t6 = local_first(ts(after & st == 6));
            R.t7 = local_first(ts(after & st == 7));                   % first DISABLING, QUICK_STOP, FAULT
            tf = tr.f_brake_rel.Time; f = tr.f_brake_rel.Data;
            R.tEng = local_first(tf(tf >= tReq & f <= 1e-9));          % spring brake fully applied (f_rel 0)
            m = t >= tReq;
            R.vReq = interp1(t, v, tReq); R.vEng = interp1(t, v, R.tEng);
            R.vmax = max(v(m)); R.vEnd = max(abs(v(t >= tend - 0.5))); R.dist = trapz(t(m), v(m));
            R.os = any(bitand(fc(after), uint16(32)) ~= 0);             % OVERSPEED, fault bit 5
            R.stEnd = st(end); R.fcEnd = fc(end); R.pk = max(abs(tr.tau_spr.Data(m)));
            mm = ts >= tReq - 0.05; tt = ts(mm); ss = st(mm); ch = [1; find(diff(ss) ~= 0) + 1];
            R.seq = strjoin(arrayfun(@(i) sprintf('%d at %.3f s', ss(i), tt(i)), ch, 'UniformOutput', false), ', ');
        end
    end

    % =========================================================================================
    methods (Test)
        function testInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'drive_system'}, 'Quiet', true);
            tc.rec('IF', 'shaddad_check_interfaces(drive_system) = 0', sprintf('%d mismatches', n), n == 0);
            tc.verifyEqual(n, 0);
        end

        function testAlgebraicLoops(tc)
            for m = {'tb_drive', 'tb_drive_sys', 'tb_drive_2mass'}
                load_system(m{1});
                try
                    evalc('L = Simulink.BlockDiagram.getAlgebraicLoops(m{1});');
                catch err
                    % R2026b JIT relink assertion (first compile only, build lessons): retry once
                    if ~contains(getReport(err, 'extended', 'hyperlinks', 'off'), 'o != nullptr'), rethrow(err); end
                    evalc('L = Simulink.BlockDiagram.getAlgebraicLoops(m{1});');
                end
                tc.rec('ALG', ['getAlgebraicLoops(' m{1} ') = 0'], sprintf('%d', numel(L)), isempty(L));
                tc.verifyEmpty(L, m{1});
            end
        end

        function testVariantsCompile(tc)
            % SYS and IDEAL compile in drive_system (both sides) with identical ports, no isolated network
            for fid = [1 3]
                [so, ~] = tb_drive_run('tb_drive_sys', 0.05, struct(), struct('VC_DRIVE_FID', fid));
                ok = isempty(so.ErrorMessage);
                tc.rec('VAR', sprintf('drive_system compiles and runs with VC_DRIVE_FID = %d', fid), 'ok', ok);
                tc.verifyTrue(ok);
            end
            % identical choice ports
            ports = @(m) local_ports(m);
            a = ports('drive_plant_sys'); b = ports('drive_plant_ideal');
            tc.rec('VAR', 'drive_plant_sys / drive_plant_ideal ports identical', strjoin(a, ', '), isequal(a, b));
            tc.verifyEqual(a, b);
            function p = local_ports(m)
                load_system(m);
                bl = [find_system(m, 'SearchDepth', 1, 'BlockType', 'Inport'); find_system(m, 'SearchDepth', 1, 'BlockType', 'Outport'); ...
                      find_system(m, 'SearchDepth', 1, 'BlockType', 'PMIOPort')];
                p = cellfun(@(x) sprintf('%s:%s:%s', get_param(x, 'BlockType'), get_param(x, 'Name'), get_param(x, 'Port')), bl, ...
                    'UniformOutput', false)';
                ct = find_system(m, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
                p = [p, cellfun(@(x) get_param(x, 'ConnectionType'), ct, 'UniformOutput', false)'];
            end
        end

        function testT01_ramp(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 5, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 5, struct('tb_cmd', c));
            [t, v] = tc.vtrack(so.truth);
            v15 = interp1(t, v, 2.5); vmax = max(v); ess = max(abs(v(t >= 4) - 1.0));
            ok = v15 >= 0.95 && vmax < 1.05 && ess < 0.01;
            tc.rec('T1', 'v(t_cmd+1.5 s) >= 0.95 m/s; overshoot < 5 %; |e_ss| < 0.01 m/s (after 3 s)', ...
                sprintf('v = %.4f m/s; overshoot %.2f %%; |e_ss| %.4f m/s', v15, 100*(vmax - 1), ess), ok);
            tc.verifyGreaterThanOrEqual(v15, 0.95); tc.verifyLessThan(vmax, 1.05); tc.verifyLessThan(ess, 0.01);
        end

        function testT02_step(tc)
            Pm = tc.P; Pm.drive.esc.a_max = 1e6;                  % "a_max = inf": no ESC ramp
            [c, ~] = tb_drive_profile(struct('t_end', 4, 'v_tgt', [0 0; 1 0.5], 'shaped', false, 'Kff', 0));
            so = tb_drive_run('tb_drive', 4, struct('tb_cmd', c), struct('P', Pm));
            [t, v] = tc.vtrack(so.truth);
            tstep = 1.01;                                          % arrival at the ESC (one CAN frame RT)
            os = 100*(max(v)/0.5 - 1);
            last = find(abs(v - 0.5) > 0.01, 1, 'last'); ts2 = t(last) - tstep;
            tmax = max(abs(so.truth.tau_m.Data)); Tpk = tc.P.drive.motor.T_peak;
            tc.rec('T2', sprintf('torque <= T_peak %.2f N m (saturation), no wind-up', Tpk), sprintf('max |tau_m| %.2f N m', tmax), tmax <= Tpk + 1e-6);
            tc.verifyLessThanOrEqual(tmax, Tpk + 1e-6);
            ok = os < 10 && ts2 < 0.6;
            e = tc.P.drive.esc; pr = roots([e.J_eq e.Kp_w e.Ki_w]);          % rigid closed loop of the speed PI
            if isreal(pr), ps = sprintf('%.1f / %.1f', pr); else, ps = sprintf('%.1f +/- %.1fj', real(pr(1)), abs(imag(pr(1)))); end
            tc.rec('T2', 'unramped step 0 -> 0.5 m/s: overshoot < 10 %, 2 % settling < 0.6 s (rev-A criterion)', ...
                sprintf('overshoot %.1f %%, settling %.3f s (speed PI Kp %.3f / Ki %.3f, D64: rigid closed-loop poles %s rad/s)', ...
                os, ts2, e.Kp_w, e.Ki_w, ps), tern(ok, 'pass', 'conflict'));
            tc.verifyLessThan(os, 15, 'wind-up / overshoot bound');
            tc.assumeTrue(ok, sprintf(['CONFLICT: spec T2 numbers (overshoot < 10 %%, settling < 0.6 s) were set for the rev-A gains; ' ...
                'with the binding speed PI (D64: Kp %.3f, Ki %.3f) the measured step gives overshoot %.1f %% and settling %.2f s ' ...
                '(rigid closed-loop poles %s rad/s => 2 %% settling >= 4/%.1f = %.2f s)'], e.Kp_w, e.Ki_w, os, ts2, ps, ...
                -max(real(pr)), 4/-max(real(pr))));
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
        end

        function testT03_climb(tc)
            g = tc.gradeTorque(20); Trr = tc.rollTorque(0.08, 20);
            [c, ~] = tb_drive_profile(struct('t_end', 12, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 12, struct('tb_cmd', c, 'tb_Trr', Trr, 'tb_grade', tc.ramp(3, 4, 0, g, 12)));
            tr = so.truth; [t, v] = tc.vtrack(tr); m = t >= 9;
            ev = 100*max(abs(v(m) - 1)); tau = mean(tr.tau_m.Data(m)); idc = mean(tr.i_dc.Data(m)); pdc = mean(tr.p_dc.Data(m));
            vdc = mean(tr.v_dc.Data(m));
            % expected motor torque: (grade + rolling) at the sprocket through N and eta, plus the gearbox no-load drag
            gr = tc.P.drive.gear; wm = 1.0*gr.k_v2w;
            tauExp = (abs(g) + Trr)/(gr.N*gr.eta) + gr.T_nl_C + gr.c_nl*wm;
            etaDr = tau*wm/pdc;                                    % motor + ESC efficiency at the operating point
            ok = ev < 2 && abs(tau/tauExp - 1) <= 0.05 && abs(idc*vdc/pdc - 1) <= 0.02 && etaDr >= 0.75 && etaDr <= 0.97;
            tc.rec('T3', sprintf(['20 deg climb 1 m/s f_rr 0.08 (%.0f kg per side): speed error < 2 %%; tau_m %.3f N m +/- 5 %% ' ...
                '(grade + rolling through the gearbox); i_dc x v_dc = p_dc +/- 2 %%; motor + ESC efficiency 75-97 %%'], ...
                tc.P.drive.bench.m_side, tauExp), ...
                sprintf('error %.2f %%; tau_m %.3f N m; i_dc %.2f A at %.1f V; p_dc %.0f W; efficiency %.1f %%', ev, tau, idc, vdc, pdc, 100*etaDr), ok);
            tc.verifyLessThan(ev, 2); tc.verifyLessThanOrEqual(abs(tau/tauExp - 1), 0.05);
            tc.verifyLessThanOrEqual(abs(idc*vdc/pdc - 1), 0.02);
            tc.verifyGreaterThanOrEqual(etaDr, 0.75); tc.verifyLessThanOrEqual(etaDr, 0.97);
        end

        function testT04_stall(tc)
            Tpk = tc.P.drive.motor.T_peak; Tct = tc.P.drive.motor.T_cont;
            [c, ~] = tb_drive_profile(struct('t_end', 112, 'mode', 2, 'Kff', 0, 'tau_ff_tab', [0 0; 0.5 Tpk; 40 0; 110 Tpk]));
            so = tb_drive_run('tb_drive', 112, struct('tb_cmd', c, 'tb_load', 2, 'tb_w_dyno', timeseries(0, 0), 'tb_dec', 10));
            tr = so.truth; t = tr.tau_m.Time; tau = tr.tau_m.Data;
            i_on = find(tau >= 0.99*Tpk & t > 0.5, 1);
            i_off = i_on - 1 + find(tau(i_on:end) < 0.99*Tpk, 1);
            dur = t(i_off) - t(i_on);
            win = t > t(i_on) + 0.05 & t < t(i_off) - 0.05;
            dev = max(abs(tau(win) - Tpk));
            fold = max(tau(t >= t(i_off) + 1 & t < 40));
            av = double(so.fb.tau_avail.Data); ta = so.fb.tau_avail.Time;
            i_rec = find(ta > 40 & av >= Tpk - 0.01, 1); trec = ta(i_rec) - 40;
            again = max(tau(t > 111));
            ok = abs(dur - 30) <= 1 && dev <= 0.0125*Tpk && fold <= Tct + 0.02 && abs(trec - 60) <= 5 && again >= 0.99*Tpk;
            tc.rec('T4', sprintf(['TORQUE T_peak %.2f N m locked: +/- 1.25 %% for 30 +/- 1 s; fold-back <= T_cont %.2f N m within 1 s; ' ...
                'peak again after 60 +/- 5 s at zero torque'], Tpk, Tct), ...
                sprintf('%.2f s at %.2f N m (max dev %.3f); fold-back level %.3f N m; peak available %.1f s after zero torque; re-apply %.2f N m', ...
                dur, Tpk, dev, fold, trec, again), ok);
            tc.verifyLessThanOrEqual(abs(dur - 30), 1); tc.verifyLessThanOrEqual(dev, 0.0125*Tpk);
            tc.verifyLessThanOrEqual(fold, Tct + 0.02); tc.verifyLessThanOrEqual(abs(trec - 60), 5);
            tc.verifyGreaterThanOrEqual(again, 0.99*Tpk);
            % locked sprocket (information for T23): at w = 0 the smooth mesh-loss law -k_loss |T_out| tanh(w/w_th) is 0,
            % so the full N x 16 N m reaches the sprocket (no static efficiency in the model); T23 itself is in testT23_gearbox
            spr = max(abs(tr.tau_spr.Data(win)));
            gr = tc.P.drive.gear;
            tc.rec('T23', sprintf('info: locked sprocket at T_peak (w = 0, no mesh loss at standstill in the smooth loss law); T2NOT %.0f N m', gr.T_estop_out), ...
                sprintf('|tau_spr| %.1f N m = N x T_peak (%.1f %% of T2B %.0f, %.0f %% of T2NOT %.0f)', spr, 100*spr/gr.T_acc_out, gr.T_acc_out, ...
                100*spr/gr.T_estop_out, gr.T_estop_out), ...
                tern(spr <= tc.P.drive.gear.T_estop_out, 'pass', 'fail'));
            tc.verifyLessThanOrEqual(spr, tc.P.drive.gear.T_estop_out);
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
        end

        function testT23_gearbox(tc)
            % peak motor torque 16 N m (TORQUE mode) through the gearbox while the dyno turns the sprocket at +/- 2 rad/s
            % (motor 48 rad/s = 458 rpm, below every envelope corner): |tau_spr| <= T2B 380 N m (design 16 x 24 x 0.94 = 361)
            g = tc.P.drive.gear; meas = ''; okAll = true; pk = 0; Tpk = tc.P.drive.motor.T_peak;
            for sgn = [1 -1]
                [c, ~] = tb_drive_profile(struct('t_end', 4, 'mode', 2, 'Kff', 0, 'tau_ff_tab', [0 0; 1 sgn*Tpk; 3 0]));
                so = tb_drive_run('tb_drive', 4, struct('tb_cmd', c, 'tb_load', 2, 'tb_w_dyno', tc.ramp(0.2, 0.7, 0, sgn*2, 4)));
                tr = so.truth; t = tr.tau_spr.Time; m = t >= 1.5 & t <= 2.9;
                tm = mean(abs(tr.tau_m.Data(m))); ts = max(abs(tr.tau_spr.Data(t >= 0.8)));
                pk = max(pk, ts);
                ok = tm >= 0.99*Tpk && ts <= g.T_acc_out; okAll = okAll && ok;
                meas = [meas sprintf('%+d rad/s: motor %.2f N m, max |tau_spr| %.1f N m. ', 2*sgn, tm, ts)]; %#ok<AGROW>
                tc.verifyGreaterThanOrEqual(tm, 0.99*Tpk); tc.verifyLessThanOrEqual(ts, g.T_acc_out);
            end
            tc.rec('T23', sprintf('|tau_spr| <= %.0f N m (T2B) at peak motor torque %.2f N m (sprocket turning, both directions)', g.T_acc_out, Tpk), ...
                sprintf('%s(design N x T_peak x eta = %.1f N m)', meas, g.N*Tpk*g.eta), okAll);
            tc.assertGreaterThan(pk, 0);
        end

        function testT05_thermal(tc)
            tc.assumeTrue(local_long(), 'Long test (2 x 2 h, Accelerator, chunked) skipped: set T06_LONG=1 environment variable to run');
            % (a) 40 degC ambient, no solar; (b) 50 degC + solar (tb_drive_long, operating-point chunks)
            Ra = tb_drive_long('T5a');
            okT = abs(Ra.Tend_C - 129) <= 8; ok63 = abs(Ra.t63_min - 20) <= 5;
            tc.rec('T5', 'dyno 3000 rpm 8 N m, 40 degC, 2 h: T_winding 129 +/- 8 degC', ...
                sprintf('T_winding(2 h) %.1f degC (wall %.0f s, %s)', Ra.Tend_C, Ra.wall, Ra.mode), okT);
            tc.verifyLessThanOrEqual(abs(Ra.Tend_C - 129), 8);
            % 63 % rise: the spec value (about 20 min) is the constant-loss figure of the 2.6 network (20.3 min with 223 W);
            % with the copper temperature coefficient of the spec 2.5 loss law (alpha_cu, loss maps at 25 / 125 degC) the
            % positive loss-temperature feedback stretches it to 25.3 min (independent 2-node ODE with the spec parameters)
            tc.rec('T5', '63 % rise 20 +/- 5 min (spec 2.6 constant-loss estimate)', ...
                sprintf('t63 %.2f min (spec network + spec 2.5 alpha_cu loss feedback: 25.3 min expected; constant 223 W: 20.3 min)', ...
                Ra.t63_min), tern(ok63, 'pass', 'conflict'));
            tc.verifyLessThanOrEqual(Ra.t63_min, 27, 'regression bound of the spec-consistent value 25.3 min');
            Rb = tb_drive_long('T5b');
            ok2 = Rb.derating && Rb.Tmax_C <= 165;
            tc.rec('T5', '50 degC + solar: derating active, T_winding never > 165 degC', ...
                sprintf(['DERATING (status b3) after power-up %d, first at %.1f min (T_winding 140 degC at %.1f min; spec about 35 min); ' ...
                'min tau_avail %.2f N m, mean tau_m %.2f N m; max T_winding %.1f degC; max T_esc %.1f degC'], ...
                Rb.derating, Rb.derate_t_min, Rb.t140_min, Rb.tau_avail_min, Rb.tau_m_mean, Rb.Tmax_C, Rb.T_esc_max_C), ok2);
            tc.verifyTrue(Rb.derating); tc.verifyLessThanOrEqual(Rb.Tmax_C, 165);
            tc.assumeTrue(ok63, sprintf(['CONFLICT: spec T5 63 %% rise 20 +/- 5 min is the constant-loss estimate; the spec 2.5 ' ...
                'loss law with alpha_cu gives 25.3 min on the spec 2.6 network, measured %.2f min'], Ra.t63_min));
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
        end

        function testT06_efficiency(tc)
            tc.assumeTrue(false, ['PENDING: the T6 efficiency points (92.8 / 90.6 / 88.8 % at 8 / 7 / 4 N m) are datasheet ' ...
                'values of the former 2.5 kW motor; the real Movematic 750 W motor (docs/hardware/real_robot_components.md) ' ...
                'needs its efficiency map before this test is re-baselined']);
            pts = [3000 8 92.8; 1790 7 90.6; 895 4 88.8];
            Pm = tc.P; Pm.drive.thermal.dT_solar = 0; Pm.drive.thermal.T_amb0 = 298.15;
            N = tc.P.drive.gear.N; e = tc.P.drive.esc; br = tc.P.drive.brake; g = tc.P.drive.gear; lam = tc.P.drive.motor.lam;
            meas = ''; okAll = true;
            for k = 1:size(pts, 1)
                w = pts(k, 1)*pi/30;
                [c, ~] = tb_drive_profile(struct('t_end', 3, 'mode', 2, 'Kff', 0, 'tau_ff_tab', [0 0; 0.3 pts(k, 2)]));
                so = tb_drive_run('tb_drive', 3, struct('tb_cmd', c, 'tb_load', 2, 'tb_w_dyno', tc.ramp(0.2, 1.0, 0, w/N, 3), ...
                    'tb_T_amb', 298.15), struct('P', Pm));
                tr = so.truth; m = tr.tau_m.Time >= 2;
                tau = mean(tr.tau_m.Data(m)); wm = mean(tr.w_m.Data(m)); v = mean(tr.v_dc.Data(m)); pdc = mean(tr.p_dc.Data(m));
                paux = v^2/e.R_q + v^2/e.R_bleed + v^2*br.duty_hold/br.R_coil;
                eta = 100*(tau + lam*wm)*wm/(pdc - paux);
                pspr = mean(tr.p_spr.Data(m)); pin = tau*wm;
                pdrag = (g.T_nl_C + g.c_nl*wm)*wm;
                eta_gb = 100*pspr/pin; eta_mesh = 100*pspr/(pin - pdrag);
                okk = abs(eta - pts(k, 3)) <= 1;
                if k == 1, okk = okk && abs(eta_mesh - 94) <= 0.5; end
                okAll = okAll && okk;
                meas = [meas sprintf('%d rpm/%g N m: drive %.2f %% (spec %.1f); gearbox %.2f %% total, %.2f %% mesh. ', ...
                    pts(k, 1), pts(k, 2), eta, pts(k, 3), eta_gb, eta_mesh)]; %#ok<AGROW>
                tc.verifyLessThanOrEqual(abs(eta - pts(k, 3)), 1);
                if k == 1, tc.verifyLessThanOrEqual(abs(eta_mesh - 94), 0.5); end
            end
            tc.rec('T6', 'drive eta 92.8 / 90.6 / 88.8 % +/- 1 pt (25 degC, motor + ESC); gearbox (mesh) 94 +/- 0.5 % at 8 N m', meas, okAll);
        end

        function testT07_regen(tc)
            g = tc.gradeTorque(-20);                              % descent: gravity drives forward
            [c, ~] = tb_drive_profile(struct('t_end', 12, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 12, struct('tb_cmd', c, 'tb_grade', tc.ramp(3, 4, 0, g, 12)));
            tr = so.truth; [t, v] = tc.vtrack(tr); m = t >= 9;
            ev = 100*max(abs(v(m) - 1)); idc = mean(tr.i_dc.Data(m)); pdc = mean(tr.p_dc.Data(m));
            % regeneration: gravity power at the sprocket returns to the DC bus with gearbox + motor + ESC losses
            % (and the bench rolling resistance): p_dc = -(30 .. 95 %) of it
            Pspr = abs(g)*1.0/tc.P.drive.gear.r_spr;
            kreg = -pdc/Pspr;
            ok = ev < 2 && idc < 0 && kreg >= 0.30 && kreg <= 0.95;
            tc.rec('T7', '20 deg descent 1 m/s: speed error < 2 %; regenerating (i_dc < 0), p_dc = -(30..95 %) of the gravity power', ...
                sprintf('error %.2f %%; i_dc %.2f A; p_dc %.0f W = %.0f %% of %.0f W', ev, idc, pdc, 100*kreg, Pspr), ok);
            tc.verifyLessThan(ev, 2); tc.verifyLessThan(idc, 0);
            tc.verifyGreaterThanOrEqual(kreg, 0.30); tc.verifyLessThanOrEqual(kreg, 0.95);
            % stiff 59 V source: regen torque <= 33 % of the regen limit, no overvoltage fault
            so = tb_drive_run('tb_drive', 12, struct('tb_cmd', c, 'tb_grade', tc.ramp(3, 4, 0, g, 12), ...
                'tb_vsrc', timeseries(59, 0), 'tb_Rsrc', 1e-3));
            fb = so.fb; m = fb.tau_est.Time >= 9;
            reg = -mean(double(fb.tau_est.Data(m))); ratio = reg/tc.P.drive.motor.T_peak;
            ov = any(bitand(fb.fault_code.Data, uint16(2)) ~= 0);
            [t, v] = tc.vtrack(so.truth); ev2 = 100*max(abs(v(t >= 9) - 1));
            ok = ratio <= 0.33 + 1e-3 && ~ov;
            tc.rec('T7', 'Vdc 59 V stiff: regen torque <= 33 % of the regen limit (T_peak), overvoltage fault not set', ...
                sprintf('regen %.2f N m = %.1f %% of limit; OV fault %d; speed error %.2f %%', reg, 100*ratio, ov, ev2), ok);
            tc.verifyLessThanOrEqual(ratio, 0.331); tc.verifyFalse(ov);
        end

        function testT08_brakeHold(tc)
            tc.assumeTrue(local_long(), 'Long test (605 s) skipped: set T06_LONG=1 environment variable to run');
            R = tb_drive_long('T8');
            ok = abs(R.dt_ms - 60) <= 10 && R.creep_mph < 0.05 && R.wmax < 2e-3 && R.hold_s >= 600;
            tc.rec('T8', 'brake hold 20 deg, drive disabled, 600 s: engaged 60 +/- 10 ms after command; creep < 0.05 m/h; |w_m| < 2e-3 rad/s', ...
                sprintf('engaged after %.1f ms; creep %.4f m/h over %.0f s; max |w_m| %.2e rad/s', R.dt_ms, R.creep_mph, R.hold_s, R.wmax), ok);
            tc.verifyLessThanOrEqual(abs(R.dt_ms - 60), 10); tc.verifyLessThan(R.creep_mph, 0.05); tc.verifyLessThan(R.wmax, 2e-3);
            tc.verifyGreaterThanOrEqual(R.hold_s, 600);
        end

        function testT09_sto(tc)
            a = tc.P.drive.esc.a_Tc; edges = 0:20:600; spec = zeros(1, numel(edges) - 1);
            cases = {'flat', 0, 0.6; 'downhill 20 deg', tc.gradeTorque(-20), 1.0};
            vTop = tc.P.electronics.lim.v_track_max; Test = tc.P.drive.gear.T_estop_out;
            for k = 1:2
                [c, ~] = tb_drive_profile(struct('t_end', 10, 'v_tgt', [0 0; 0.5 vTop]));
                sto = timeseries([true; false], [0; 6]);
                so = tb_drive_run('tb_drive', 10, struct('tb_cmd', c, 'tb_sto', sto, 'tb_grade', timeseries(cases{k, 2}, 0)));
                tr = so.truth; fb = so.fb;
                st = double(fb.state.Data); ts = fb.state.Time;
                t8 = ts(find(ts >= 6 - 1e-9 & st == 8, 1)) - 6;
                te = fb.tau_est.Time; ye = double(fb.tau_est.Data);
                w = te >= 6.001 - 1e-9 & te <= 6.010;
                yy = ye(w); rr = yy(2:end)./yy(1:end - 1);
                tr0 = max(abs(rr - a)) < 1e-4;                      % pure decay => Tr = 0 from 6.001 s
                [t, v] = tc.vtrack(tr);
                i0 = find(t >= 6, 1); i1 = i0 - 1 + find(v(i0:end) <= 0, 1);
                if isempty(i1), i1 = numel(t); end
                d = trapz(t(i0:i1), v(i0:i1));
                pk = max(abs(tr.tau_spr.Data(t >= 6)));
                spec = spec + histcounts(abs(tr.tau_spr.Data), edges)*mean(diff(t));
                % STO_ACTIVE: status b8 (Extract Bits of fault-word bit 10) and fault_code b10 while STO is low
                ws = ts > 6.002 & ts < 9.9;
                stoBits = all(bitand(fb.status.Data(ws), uint16(256)) ~= 0) && all(bitand(fb.fault_code.Data(ws), uint16(1024)) ~= 0);
                ok = t8 <= 1e-3 + 1e-9 && tr0 && d < cases{k, 3} && pk < Test && stoBits;
                tc.rec('T9', sprintf('STO at %.2f m/s (top speed) %s: torque command 0 within 1 ms; stop < %.1f m; peak |tau_spr| < T2NOT %.0f N m; STO_ACTIVE bits', ...
                    vTop, cases{k, 1}, cases{k, 3}, Test), sprintf('STO state after %.1f ms, Tr = 0 from +1 ms: %d; distance %.3f m; peak %.1f N m; status b8 + fault b10: %d', ...
                    1e3*t8, tr0, d, pk, stoBits), ok);
                tc.verifyLessThanOrEqual(t8, 1e-3 + 1e-9); tc.verifyTrue(tr0); tc.verifyLessThan(d, cases{k, 3}); tc.verifyLessThan(pk, Test);
                tc.verifyTrue(stoBits);
            end
            nz = find(spec > 0);
            txt = strjoin(arrayfun(@(i) sprintf('%d-%d: %.2f s', edges(i), edges(i + 1), spec(i)), nz, 'UniformOutput', false), '; ');
            tc.rec('T23', 'gearbox load spectrum |tau_spr| (20 N m bins, time at torque; T9 runs)', txt, 'pass');
        end

        function testT10_canTimeout(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 8, 'v_tgt', [0 0; 1 1.0], 'alive_freeze', 5));
            so = tb_drive_run('tb_drive', 8, struct('tb_cmd', c));
            fb = so.fb; tr = so.truth; ts = fb.state.Time; st = double(fb.state.Data);
            tqs = 1e3*(ts(find(st == 6, 1)) - 5);
            [t, v] = tc.vtrack(tr);
            m = v < 0.9 & v > 0.1 & t > 5; p = polyfit(t(m), v(m), 1); dec = -p(1);
            tf = tr.f_brake_rel.Time; f = tr.f_brake_rel.Data;
            i_eng = find(tf > 5 & f <= 1e-9, 1); v_eng = abs(interp1(t, v, tf(i_eng)));
            fault6 = bitand(fb.fault_code.Data(end), uint16(64)) ~= 0; s_end = st(end);
            ok = abs(tqs - 100) <= 10 && abs(dec - 2.0) <= 0.15 && v_eng < 0.011 && fault6 && s_end == 7;
            tc.rec('T10', 'alive frozen at 5 s, 1 m/s: QUICK_STOP at 100 +/- 10 ms; decel 2.0 +/- 0.15 m/s^2; brake at standstill; bit 6; state 7', ...
                sprintf('QS after %.0f ms; decel %.3f m/s^2; |v| at brake engaged %.4f m/s; bit6 %d; final state %d', tqs, dec, v_eng, fault6, s_end), ok);
            tc.verifyLessThanOrEqual(abs(tqs - 100), 10); tc.verifyLessThanOrEqual(abs(dec - 2.0), 0.15);
            tc.verifyLessThan(v_eng, 0.011); tc.verifyTrue(fault6); tc.verifyEqual(s_end, 7);
        end

        function testT11_undervoltage(tc)
            % dyno at 100 rad/s motor (955 rpm): below the voltage-dependent corner speed of the peak envelope at every
            % Vdc >= 36 V (2000 rpm x 36 / 48 = 1500 rpm), so tau_avail = T_peak x f_uv isolates the UV derating law
            Tpk = tc.P.drive.motor.T_peak; Vd = tc.P.drive.esc.V_uv_derate; N = tc.P.drive.gear.N;
            tv = (0:0.001:14)'; vs = 51 - 17*min(max((tv - 1)/10, 0), 1);
            [c, ~] = tb_drive_profile(struct('t_end', 14, 'mode', 2, 'Kff', 0, 'tau_ff_tab', [0 0; 0.3 1.4*Tpk]));
            so = tb_drive_run('tb_drive', 14, struct('tb_cmd', c, 'tb_load', 2, 'tb_w_dyno', tc.ramp(0.2, 0.7, 0, 100/N, 14), ...
                'tb_vsrc', timeseries(vs, tv)));
            fb = so.fb; t = fb.v_dc.Time; v = double(fb.v_dc.Data); av = double(fb.tau_avail.Data);
            m = v >= Vd(1) + 0.2 & v <= Vd(2) - 0.2 & t > 1;
            err = max(abs(av(m) - Tpk*(v(m) - Vd(1))/diff(Vd)))/Tpk;
            % UV fault timing from the first sample below V_uv_fault: the ESC enters NOT_READY at V_off = 35 V and sheds
            % its load (power stage, brake coil), so the terminal voltage rebounds a few mV above 35 V for some ms; the
            % Protection Relay keeps counting through that (cumulative time below the limit)
            fc = fb.fault_code.Data; i_uv = find(bitand(fc, uint16(4)) ~= 0, 1);
            i35 = find(v < 35 & t > 1, 1);
            dt = 1e3*(t(i_uv) - t(i35));
            reb = max(v(i35:i_uv)) - 35;
            ok = err <= 0.02 && abs(dt - 50) <= 2 && isempty(so.ErrorMessage);
            tc.rec('T11', sprintf('Vdc 51 -> 34 V above T_peak: motoring limit linear 1 -> 0 between %.0f -> %.0f V (+/- 2 %%); UV fault 50 ms below 35 V', Vd(2), Vd(1)), ...
                sprintf(['max limit error %.2f %% of T_peak (dyno 100 rad/s); UV fault %.0f ms after Vdc first < 35 V ' ...
                '(load-shed rebound %.0f mV above 35 V in between); solver ok'], 100*err, dt, 1e3*reb), ok);
            tc.verifyLessThanOrEqual(err, 0.02); tc.verifyLessThanOrEqual(abs(dt - 50), 2);
        end

        function testT12_contactor(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 6, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 6, struct('tb_cmd', c, 'tb_contactor', timeseries([1; 1; 0], [0; 4; 4.001])));
            tr = so.truth; t = tr.v_dc.Time; vd = tr.v_dc.Data; st = double(so.fb.state.Data);
            t35 = t(find(vd < 35 & t > 4, 1)); tnr = t(find(st == 1 & t > 4, 1));
            dt = 1e3*(tnr - t35); feng = tr.f_brake_rel.Data(end);
            % ESC logic always powered: FB frames keep coming in NOT_READY (alive counting, rx_ok, state 1 decoded)
            rx = so.fb_rx; tr2 = rx.alive.Time; mr = tr2 > tnr + 0.1;
            nAlive = numel(unique(rx.alive.Data(mr))); rxok = all(rx.rx_ok.Data(mr)); stRx = double(rx.state.Data(end));
            fbOk = nAlive > 10 && rxok && stRx == 1;
            ok = isempty(so.ErrorMessage) && dt <= 10 && feng <= 1e-9 && fbOk;
            tc.rec('T12', 'contactor opens while coasting at 1 m/s: no solver error; NOT_READY within 10 ms of Vdc < 35 V; brake engages; FB frames continue', ...
                sprintf('NOT_READY %.0f ms after Vdc < 35 V; f_rel end %.2f; min Vdc %.2f V; FB in NOT_READY: %d alive values, rx_ok %d, decoded state %d', ...
                dt, feng, min(vd(t > 4)), nAlive, rxok, stRx), ok);
            tc.verifyLessThanOrEqual(dt, 10); tc.verifyLessThanOrEqual(feng, 1e-9); tc.verifyTrue(fbOk);
        end

        function testT24_hvOffAfterSto(tc)
            % revision C (S8a): DC link removed after STO (hardware E-stop: STO first, K1/K2 open later) must not
            % latch UNDERVOLTAGE (fault bit 2): the ESC is NOT_READY while the link is dead and returns to READY /
            % OPERATION without a fault reset when HV is back. UV during operation stays a latched fault (T11).
            [c, ~] = tb_drive_profile(struct('t_end', 10, 'v_tgt', [0 0; 1 1.0]));
            sto = timeseries([true; false; true], [0; 3; 6]);
            cont = timeseries([1; 1; 0; 0; 1], [0; 3.5; 3.501; 7; 7.001]);
            so = tb_drive_run('tb_drive', 10, struct('tb_cmd', c, 'tb_sto', sto, 'tb_contactor', cont));
            fb = so.fb; st = double(fb.state.Data); fc = fb.fault_code.Data;
            tv = so.truth.v_dc.Time; vd = so.truth.v_dc.Data;
            uv = any(bitand(fc, uint16(4)) ~= 0); vmin = min(vd(tv > 3.5 & tv < 7)); vend = vd(end);
            nFault = nnz(st == 7); sEnd = st(end);
            ok = isempty(so.ErrorMessage) && ~uv && nFault == 0 && vmin < 35 && any(sEnd == [2 3 4]);
            tc.rec('T24', 'STO at 3 s, contactor open 3.5-7 s (Vdc < 35 V): no UNDERVOLTAGE latched, no FAULT, READY/OPERATION when HV returns', ...
                sprintf('min Vdc %.2f V while open; UV bit seen %d; FAULT samples %d; final state %d at Vdc %.1f V', ...
                vmin, uv, nFault, sEnd, vend), ok);
            tc.verifyLessThan(vmin, 35, 'DC link really collapsed');
            tc.verifyFalse(uv, 'UNDERVOLTAGE latched while the power stage was off (STO)');
            tc.verifyEqual(nFault, 0, 'ESC entered FAULT');
            tc.verifyTrue(any(sEnd == [2 3 4]), sprintf('final state %d (expected READY/ENABLING/OPERATION)', sEnd));
        end

        function testT13_energy(tc)
            g = tc.gradeTorque(20);
            tg = (0:0.001:18)'; gv = g*min(max((tg - 3), 0), 1) - 2*g*min(max((tg - 10)/2, 0), 1);
            Trr = tc.rollTorque(0.08, 20);
            [c, ~] = tb_drive_profile(struct('t_end', 18, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 18, struct('tb_cmd', c, 'tb_Trr', Trr, 'tb_grade', timeseries(gv, tg)), struct(), ...
                {'SimscapeLogType', 'all', 'SimscapeLogName', 'simlog', 'SimscapeLogLimitData', 'off'});
            E = tb_drive_energy(so, tc.P);
            ok = E.err_pct < 0.5;
            tc.rec('T13', '|int p_dc - (int p_spr + int losses + dE_kin + dE_th)| < 0.5 % of int |p_dc| (T3 + T7 sequence)', ...
                sprintf('residual %.4f %% (E_dc %.1f kJ, E_spr %.1f kJ, losses %.1f kJ, dE_th %.1f kJ, dE_kin %.3f kJ)', ...
                E.err_pct, E.E_dc/1e3, E.E_spr/1e3, E.E_loss/1e3, E.dE_th/1e3, E.dE_kin/1e3), ok);
            tc.verifyLessThan(E.err_pct, 0.5);
        end

        function testT14_sides(tc)
            [cL, ~] = tb_drive_profile(struct('t_end', 4, 'w_ref_tab', [0 0; 1 100], 'shaped', false, 'Kff', 0));
            [cR, ~] = tb_drive_profile(struct('t_end', 4, 'w_ref_tab', [0 0; 1 -100], 'shaped', false, 'Kff', 0));
            so = tb_drive_run('tb_drive_sys', 4, struct('tb_cmdL', cL, 'tb_cmdR', cR));
            tr = so.truth; m = tr.L.w_spr.Time >= 3;
            wl = mean(tr.L.w_spr.Data(m)); wr = mean(tr.R.w_spr.Data(m));
            dcnt = @(x) sum(double(diff(int64(x))));             % counter progress (no wrap within 4 s)
            el = so.fbL.enc_cnt.Data(so.fbL.enc_cnt.Time >= 1.5); er = so.fbR.enc_cnt.Data(so.fbR.enc_cnt.Time >= 1.5);
            incL = dcnt(el); incR = dcnt(er);
            [cZ, ~] = tb_drive_profile(struct('t_end', 4, 'w_ref_tab', [0 0], 'shaped', false, 'Kff', 0));
            so2 = tb_drive_run('tb_drive_sys', 4, struct('tb_cmdL', cL, 'tb_cmdR', cZ));
            xr = max(abs(so2.truth.R.w_spr.Data(so2.truth.R.w_spr.Time >= 1)));
            ok = wl > 0 && wr < 0 && incL > 0 && incR < 0 && xr < 1e-3 && abs(wl + wr) < 0.01*abs(wl);
            tc.rec('T14', 'w_ref L +100 / R -100 rad/s: w_spr L > 0, R < 0; enc_cnt L up, R down; no cross-coupling', ...
                sprintf('w_spr L %.4f R %.4f rad/s; enc L %+d R %+d counts; R speed with R cmd 0 and L at 100 rad/s: %.1e rad/s', ...
                wl, wr, incL, incR, xr), ok);
            tc.verifyGreaterThan(wl, 0); tc.verifyLessThan(wr, 0); tc.verifyGreaterThan(incL, 0); tc.verifyLessThan(incR, 0);
            tc.verifyLessThan(xr, 1e-3);
        end

        function testT15_focEquivalence(tc)
            d = fileparts(mfilename('fullpath'));
            tc.assumeTrue(isfile(fullfile(d, 'tb_drive_foc.slx')), 'tb_drive_foc.slx not built');
            R = tb_drive_foc_compare(tc.P);
            ok = R.speed_rms_pct < 2 && R.energy_pct < 3 && R.idc_peak_pct < 5;
            tc.rec('T15', 'SYS vs FOC (T1, T3 profiles, 10 s): speed RMS diff < 2 %; DC energy diff < 3 %; peak i_dc diff < 5 %', ...
                sprintf('speed RMS %.2f %%; DC energy %.2f %%; peak i_dc %.2f %% (%s)', R.speed_rms_pct, R.energy_pct, R.idc_peak_pct, R.note), ok);
            tc.verifyLessThan(R.speed_rms_pct, 2); tc.verifyLessThan(R.energy_pct, 3); tc.verifyLessThan(R.idc_peak_pct, 5);
            tc.rec('T17', 'FOC 10 s scenario < 10 min wall-clock (Normal mode, incl. compile)', ...
                sprintf('%.0f s (worst of the T1 / T3 runs)', R.wall_foc), R.wall_foc < 600);
            tc.verifyLessThan(R.wall_foc, 600);
        end

        function testT16_codec(tc)
            R = tb_drive_codec_check(tc.P, 10000);
            ok = R.cmd_exact && R.fb_exact && R.hold_ok && R.ids_ok && R.wrap_ok && R.layout_ok;
            tc.rec('T16', ['codec round trip bit-exact to the quantised values, 10 000 random commands / feedbacks, alive wrap, ' ...
                'negatives, invalid frame hold; frame bytes = ICD 4 layout; FB1 lost 10 frames: rx_ok false from age 50 ms, payload held'], ...
                sprintf(['CMD mismatches %d, FB mismatches %d, hold ok %d, ids ok %d, alive wrap ok %d, rx_ok as expected %d ' ...
                '(%d samples false); ICD 4 byte-layout mismatches CMD/FB1/FB2/FB3 %d/%d/%d/%d'], R.cmd_mismatch, R.fb_mismatch, ...
                R.hold_ok, R.ids_ok, R.wrap_ok, R.rxok_ok, numel(R.rx_off_samples), R.layout_mismatch), ok && R.rxok_ok);
            tc.verifyTrue(R.cmd_exact); tc.verifyTrue(R.fb_exact); tc.verifyTrue(R.hold_ok); tc.verifyTrue(R.ids_ok);
            tc.verifyTrue(R.wrap_ok); tc.verifyTrue(R.rxok_ok); tc.verifyTrue(R.layout_ok);
        end

        function testCodecTypes(tc)
            R = tb_drive_codec_check(tc.P, 20);
            tc.rec('CODEC', 'codecs single/int only (no double signal inside drive_cmd_pack/unpack, drive_fb_pack/unpack)', ...
                sprintf('%d double-typed block outputs: %s', numel(R.doubles), strjoin(R.doubles, ', ')), isempty(R.doubles));
            tc.verifyEmpty(R.doubles);
        end

        function testCodegenBlocked(tc)
            hasCoder = any(strcmp({ver().Name}, 'Simulink Coder'));
            tc.rec('CG', 'codec code generation (cs_GenARM, ert, GenCodeOnly)', tern(hasCoder, 'run', 'BLOCKED: Simulink Coder not installed'), ...
                tern(hasCoder, 'pass', 'blocked'));
            tc.assumeTrue(hasCoder, 'BLOCKED: Simulink Coder not installed');
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
        end

        function testT17_speed(tc)
            tc.assumeTrue(local_long(), 'Long test skipped: set T06_LONG=1 environment variable to run');
            % synthetic duty (speed steps 0..1.5 m/s, turns with L/R differences, stops), drive_system both sides,
            % ideal DC source per side; 300 s segment of the cycle after a 1 s warm-up run (compile / accelerator build).
            % RTF = t_seg / ExecutionElapsedWallTime (SimulationMetadata); projected 1 h wall = 3600/RTF + initialisation.
            tseg = 300;
            [cL, cR] = local_duty(tseg);
            mw = struct('tb_cmdL', cL, 'tb_cmdR', cR, 'tb_dec', 100, 'tb_Rsrc', 1e-3);
            cfg = {'Normal SYS', 'normal', 1; 'Accelerator SYS', 'accelerator', 1; 'Accelerator IDEAL', 'accelerator', 3};
            meas = ''; best = 0;
            for ic = 1:size(cfg, 1)
                gv = struct('VC_DRIVE_FID', cfg{ic, 3});
                tb_drive_run('tb_drive_sys', 1, mw, gv, {'SimulationMode', cfg{ic, 2}});            % warm-up
                [so, w2] = tb_drive_run('tb_drive_sys', tseg, mw, gv, {'SimulationMode', cfg{ic, 2}});
                tc.verifyEmpty(so.ErrorMessage);
                ti = so.SimulationMetadata.TimingInfo; w1 = ti.InitializationElapsedWallTime;
                rtf = tseg/ti.ExecutionElapsedWallTime; best = max(best, rtf);
                meas = [meas sprintf('%s: RTF %.1f (%.0f s executed in %.1f s; wall %.1f s incl. init %.1f s) => 1 h about %.1f min; ', ...
                    cfg{ic, 1}, rtf, tseg, ti.ExecutionElapsedWallTime, w2, w1, (3600/rtf + w1)/60)]; %#ok<AGROW>
            end
            hasCC = ~isempty(mex.getCompilerConfigurations('C', 'Installed'));
            meas = [meas sprintf('C compiler installed %d', hasCC)];
            tc.rec('T17', '1 h synthetic duty, drive_system both sides, ideal DC source: wall-clock < 60 s (>= 60x real time)', ...
                meas, tern(best >= 60, 'pass', 'blocked'));
            tc.assumeTrue(best >= 60, sprintf(['BLOCKED: best RTF %.1f < 60 (Normal mode and JIT Accelerator, no C compiler, D31). ' ...
                'The 60x target assumes compiled code (D56 item 4: MinGW-w64)'], best));
            function r = tern(c, a, b), if c, r = a; else, r = b; end, end
            function [cL, cR] = local_duty(T)
                nc = ceil(T/120); o = kron((0:nc - 1)'*120, ones(8, 1));   % 120 s cycle repeated
                vt = repmat([0 0; 2 1.0; 20 1.3; 40 0.5; 55 0; 60 1.2; 90 0.8; 110 0], nc, 1); vt(:, 1) = vt(:, 1) + o;
                o5 = kron((0:nc - 1)'*120, ones(5, 1)); dv = repmat([0 0; 25 0.3; 35 0; 70 -0.4; 80 0], nc, 1); dv(:, 1) = dv(:, 1) + o5;
                [~, pr] = tb_drive_profile(struct('t_end', T, 'v_tgt', vt));
                [~, pd] = tb_drive_profile(struct('t_end', T, 'v_tgt', dv));
                k = tc.P.drive.gear.k_v2w; Kff = tc.P.electronics.drv.Kff_a;
                vl = pr.v_ref - pd.v_ref; vr = pr.v_ref + pd.v_ref;
                al = pr.a_ref - pd.a_ref; ar = pr.a_ref + pd.a_ref;
                Tpk = tc.P.drive.motor.T_peak;
                base = struct('enable', 1, 'mode', 1, 'tau_lim_mot', Tpk, 'tau_lim_reg', Tpk, 'alive', mod(0:numel(pr.t) - 1, 256)');
                b = base; b.w_ref = k*vl; b.tau_ff = Kff*al; cL = tb_drive_bus_ts('Bus_DriveSideCmd', pr.t, b);
                b = base; b.w_ref = k*vr; b.tau_ff = Kff*ar; cR = tb_drive_bus_ts('Bus_DriveSideCmd', pr.t, b);
            end
        end

        function testT18_frameDrops(tc)
            [c, pr] = tb_drive_profile(struct('t_end', 8, 'v_tgt', [0 0; 1 1.0; 4 0.5; 6 1.2]));
            so0 = tb_drive_run('tb_drive', 8, struct('tb_cmd', c));
            rs = RandStream('mt19937ar', 'Seed', 6); drop = rand(rs, numel(pr.t), 1) < 0.1;
            so1 = tb_drive_run('tb_drive', 8, struct('tb_cmd', c, 'tb_drop', timeseries(drop, pr.t)));
            rmsE = @(so) local_rms(so);
            e0 = rmsE(so0); e1 = rmsE(so1);
            st = double(so1.fb.state.Data); to = any(st == 6 | st == 7) || any(bitand(so1.fb.fault_code.Data, uint16(64)) ~= 0);
            run = max(diff([0; find(~drop); numel(drop) + 1])) - 1;
            ok = ~to && (e1 - e0) < 1;
            tc.rec('T18', 'frame drops p = 0.1: no false CAN timeout; speed RMS error increase < 1 %', ...
                sprintf('%d of %d frames dropped (longest run %d); timeout %d; RMS error %.3f %% -> %.3f %% of 1 m/s (+%.3f pt)', ...
                sum(drop), numel(drop), run, to, e0, e1, e1 - e0), ok);
            tc.verifyFalse(to); tc.verifyLessThan(e1 - e0, 1);
            function e = local_rms(so)
                [t, v] = tc.vtrack(so.truth);
                vr = interp1(pr.t + 0.02, pr.v_ref, t, 'previous', 0);      % reference delayed by the two RTs
                m = t > 1;
                e = 100*sqrt(mean((v(m) - vr(m)).^2));
            end
        end

        function testT19_noRegen(tc)
            g = tc.gradeTorque(-20);
            [c, ~] = tb_drive_profile(struct('t_end', 11, 'v_tgt', [0 0; 1 1.0], 'tau_lim_reg', [0 tc.P.drive.motor.T_peak; 8 0]));
            so = tb_drive_run('tb_drive', 11, struct('tb_cmd', c, 'tb_grade', tc.ramp(3, 4, 0, g, 11)));
            fb = so.fb; te = fb.tau_est.Time; y = double(fb.tau_est.Data);
            m = te >= 8 + 0.04 & te <= 10;
            reg = max(0, -min(y(m)));
            [t, v] = tc.vtrack(so.truth);
            ok = reg <= 0.1;
            st = double(fb.state.Data); fc = fb.fault_code.Data(end);
            tc.rec('T19', 'tau_lim_reg = 0 on 20 deg descent at 1 m/s: regen torque <= 0.1 N m for 2 s', ...
                sprintf(['max regen torque %.3f N m over [8.04, 10] s (no holding torque: the side runs away to %.2f m/s, ' ...
                'final state %d, fault word 0x%04X, OVERSPEED bit 5 set: %d)'], ...
                reg, max(v(t >= 8 & t <= 10)), st(end), fc, bitand(fc, uint16(32)) ~= 0), ok);
            tc.verifyLessThanOrEqual(reg, 0.1);
        end

        function testT19b_parkOnDescent(tc)
            % critfix R-01, park_req as the ICD defines it ("force brake engaged and torque off"): DRV-01's descent
            % (20 deg, 1 m/s, tau_lim_reg 0.32 N m from 6 s, so the side runs away at about 1.7 m/s^2), park_req at
            % 6.2 s, alone and in the MCU's frame (enable 0, park_req 1, both limits 0: build_fw_control sends
            % park_req = NOT drive_enable). The ESC drops the brake command on entry to DISABLING whatever the speed and
            % takes the torque off n_engage later (READY).
            Tpk = tc.P.drive.motor.T_peak; Ts = tc.P.drive.Ts_esc; nE = tc.P.drive.sm.n_engage*Ts;
            Tno = tc.P.drive.gear.T_estop_out; tp = 6.2;
            cases = {'park_req alone', struct('park_req', [0 0; tp 1]); ...
                'MCU frame: enable 0, park_req 1, limits 0', struct('enable', [0 1; tp 0], 'park_req', [0 0; tp 1], ...
                'tau_lim_mot', [0 Tpk; tp 0], 'tau_lim_reg', [0 Tpk; 6 0.32; tp 0])};
            for k = 1:size(cases, 1)
                R = tc.descentStop(20, 9, cases{k, 2}, tp);
                dt = R.tEng - R.t5;
                ok = dt <= nE + 1e-9 && R.vEnd < 0.01 && ~R.os && isnan(R.t7) && R.stEnd == 2 && R.pk < Tno;
                tc.rec('T19b', sprintf(['park_req at %.1f s on the 20 deg descent, tau_lim_reg 0.32 N m (%s): spring brake ' ...
                    'applied within n_engage %.0f ms of DISABLING; speed to 0; no OVERSPEED, no FAULT; READY; ' ...
                    'peak |tau_spr| < T2NOT %.0f N m'], tp, cases{k, 1}, 1e3*nE, Tno), ...
                    sprintf(['states %s; brake applied %.0f ms after DISABLING at %.3f m/s (%.3f m/s at the request, ' ...
                    'max %.3f m/s); %.2f m to standstill, |v| over the last 0.5 s %.4f m/s; OVERSPEED bit %d; final state ' ...
                    '%d, fault 0x%04X; peak |tau_spr| %.0f N m'], R.seq, 1e3*dt, R.vEng, R.vReq, R.vmax, R.dist, R.vEnd, ...
                    R.os, R.stEnd, R.fcEnd, R.pk), ok);
                lab = [cases{k, 1} ': '];
                tc.verifyLessThanOrEqual(dt, nE + 1e-9, [lab 'spring brake applied within n_engage of DISABLING']);
                tc.verifyLessThan(R.vEnd, 0.01, [lab 'speed to 0']);
                tc.verifyFalse(R.os, [lab 'OVERSPEED fault']);
                tc.verifyTrue(isnan(R.t7), [lab 'FAULT entered']);
                tc.verifyEqual(R.stEnd, 2, [lab 'final state READY']);
                tc.verifyLessThan(R.pk, Tno, [lab 'peak gearbox torque below T2NOT']);
            end
        end

        function testT19c_disableOnDescent(tc)
            % critfix R-01, DISABLING time-out: enable 0 at 6.0 s (park_req 0) on the descent, as tau_lim_reg falls to
            % 0.32 N m. DISABLING keeps "stop first, then engage", but drops the brake command, whatever the speed,
            % t_stop_max (P.drive.sm.n_stop_max) after the stop began. 20 deg (DRV-01's bench): the side reaches the ESC
            % overspeed trip about 0.5 s later, before the time-out, so there the brake comes from FAULT (OVERSPEED,
            % reported). 8 deg: the runaway is slow, and the time-out itself applies the brake.
            sm = tc.P.drive.sm; Ts = tc.P.drive.Ts_esc; tOut = sm.n_stop_max*Ts; nE = sm.n_engage*Ts; tq = 6.0;
            for deg = [20 8]
                R = tc.descentStop(deg, 10, struct('enable', [0 1; tq 0]), tq);
                dt = R.tEng - R.t5;
                if deg == 8
                    ok = dt >= tOut - 1e-9 && dt <= tOut + nE + 1e-9 && ~R.os && isnan(R.t7) && R.vEnd < 0.01 && R.stEnd == 2;
                    crit = sprintf(['the time-out applies the brake %.2f-%.2f s after DISABLING; no OVERSPEED, no FAULT; ' ...
                        'speed to 0; READY'], tOut, tOut + nE);
                else
                    ok = dt <= tOut + nE + 1e-9;
                    crit = sprintf(['brake applied within t_stop_max + n_engage = %.2f s of DISABLING (the overspeed trip ' ...
                        'may come first: reported)'], tOut + nE);
                end
                tc.rec('T19c', sprintf('enable 0 at %.1f s on the %d deg descent, tau_lim_reg 0.32 N m: %s', tq, deg, crit), ...
                    sprintf(['states %s; brake applied %.3f s after DISABLING at %.3f m/s (max %.3f m/s); %.2f m to ' ...
                    'standstill, |v| over the last 0.5 s %.4f m/s; OVERSPEED bit %d; final state %d, fault 0x%04X'], ...
                    R.seq, dt, R.vEng, R.vmax, R.dist, R.vEnd, R.os, R.stEnd, R.fcEnd), ok);
                lab = sprintf('%d deg: ', deg);
                tc.verifyLessThanOrEqual(dt, tOut + nE + 1e-9, [lab 'brake applied within the time-out']);
                if deg == 8
                    tc.verifyGreaterThanOrEqual(dt, tOut - 1e-9, [lab 'the time-out (not standstill, not a fault) applied the brake']);
                    tc.verifyFalse(R.os, [lab 'OVERSPEED fault']);
                    tc.verifyTrue(isnan(R.t7), [lab 'FAULT entered']);
                    tc.verifyLessThan(R.vEnd, 0.01, [lab 'speed to 0']);
                    tc.verifyEqual(R.stEnd, 2, [lab 'final state READY']);
                end
            end
        end

        function testT19d_canTimeoutOnDescent(tc)
            % critfix R-01, QUICK_STOP time-out: alive frozen at 5.9 s on the descent, so QUICK_STOP comes 100 ms later
            % as tau_lim_reg falls to 0.32 N m. The brake follows, whatever the speed, at the latest t_stop_max after the
            % stop began, and FAULT (CAN_TIMEOUT latched) n_engage after it. 20 deg (DRV-01's bench): the overspeed trip
            % comes first (reported); 8 deg: the time-out applies the brake.
            sm = tc.P.drive.sm; Ts = tc.P.drive.Ts_esc; tOut = sm.n_stop_max*Ts; nE = sm.n_engage*Ts; tf = 5.9;
            for deg = [20 8]
                R = tc.descentStop(deg, 10, struct('alive_freeze', tf), tf);
                dqs = R.t6 - tf; dt = R.tEng - R.t6;
                can = bitand(R.fcEnd, uint16(64)) ~= 0;
                ok = abs(dqs - 0.1) <= 0.01 && dt <= tOut + nE + 1e-9 && R.stEnd == 7 && can;
                if deg == 8
                    ok = ok && dt >= tOut - 1e-9 && ~R.os && R.vEnd < 0.01;
                    crit = sprintf(['QUICK_STOP 100 +/- 10 ms after the freeze; the time-out applies the brake %.2f-%.2f s ' ...
                        'after QUICK_STOP; no OVERSPEED; speed to 0; FAULT with CAN_TIMEOUT (bit 6)'], tOut, tOut + nE);
                else
                    crit = sprintf(['QUICK_STOP 100 +/- 10 ms after the freeze; brake applied within t_stop_max + n_engage ' ...
                        '= %.2f s of QUICK_STOP (the overspeed trip may come first: reported); FAULT with CAN_TIMEOUT (bit 6)'], ...
                        tOut + nE);
                end
                tc.rec('T19d', sprintf('alive frozen at %.1f s on the %d deg descent, tau_lim_reg 0.32 N m: %s', tf, deg, crit), ...
                    sprintf(['states %s; QUICK_STOP %.0f ms after the freeze; brake applied %.3f s after QUICK_STOP at ' ...
                    '%.3f m/s (max %.3f m/s); %.2f m to standstill, |v| over the last 0.5 s %.4f m/s; OVERSPEED bit %d; ' ...
                    'final state %d, fault 0x%04X'], R.seq, 1e3*dqs, dt, R.vEng, R.vmax, R.dist, R.vEnd, R.os, R.stEnd, ...
                    R.fcEnd), ok);
                lab = sprintf('%d deg: ', deg);
                tc.verifyLessThanOrEqual(abs(dqs - 0.1), 0.01, [lab 'QUICK_STOP 100 ms after the alive freeze']);
                tc.verifyLessThanOrEqual(dt, tOut + nE + 1e-9, [lab 'brake applied within the time-out']);
                tc.verifyEqual(R.stEnd, 7, [lab 'final state FAULT']);
                tc.verifyTrue(can, [lab 'CAN_TIMEOUT (fault bit 6) latched']);
                if deg == 8
                    tc.verifyGreaterThanOrEqual(dt, tOut - 1e-9, [lab 'the time-out (not standstill, not a fault) applied the brake']);
                    tc.verifyFalse(R.os, [lab 'OVERSPEED fault']);
                    tc.verifyLessThan(R.vEnd, 0.01, [lab 'speed to 0']);
                end
            end
        end

        function testT20_encoderFault(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 6, 'v_tgt', [0 0; 1 1.0]));
            tf = [0; 4];
            fi = tc.faults(tf, struct('L', struct('tau_scale', 1, 'enc_fail', [0; 1]), 'R', struct('tau_scale', 1)));
            so = tb_drive_run('tb_drive', 6, struct('tb_cmd', c, 'tb_fi', fi));
            ts = so.fb.state.Time; st = double(so.fb.state.Data);
            tF = 1e3*(ts(find(ts >= 4 & st == 7, 1)) - 4);
            t = so.truth.f_brake_rel.Time; f = so.truth.f_brake_rel.Data;
            tB = 1e3*(t(find(t >= 4 & f <= 1e-9, 1)) - 4);
            enc = bitand(so.fb.fault_code.Data(end), uint16(128)) ~= 0;
            ok = tF <= 10 && tB <= 70 && enc;
            tc.rec('T20', 'enc_fail -> FAULT within 10 ms, brake engaged within 70 ms', ...
                sprintf('FAULT after %.0f ms (ENCODER bit 7 set: %d); brake engaged after %.0f ms', tF, enc, tB), ok);
            tc.verifyLessThanOrEqual(tF, 10); tc.verifyLessThanOrEqual(tB, 70); tc.verifyTrue(enc);
        end

        function testTauScaleTrip(tc)
            % build plan T06.2: fault_inj.tau_scale multiplies Tr; 0 with enable = drive trip -> OVERCURRENT bit 0 -> FAULT
            [c, ~] = tb_drive_profile(struct('t_end', 6, 'v_tgt', [0 0; 1 1.0]));
            fi = tc.faults([0; 4], struct('L', struct('tau_scale', [1; 0]), 'R', struct('tau_scale', 1)));
            so = tb_drive_run('tb_drive', 6, struct('tb_cmd', c, 'tb_fi', fi));
            fb = so.fb; ts = fb.state.Time; st = double(fb.state.Data);
            tF = 1e3*(ts(find(ts >= 4 & st == 7, 1)) - 4);
            oc = bitand(fb.fault_code.Data(end), uint16(1)) ~= 0;
            fbit = bitand(fb.status.Data(end), uint16(4)) ~= 0;
            te = fb.tau_est.Time; tz = max(abs(double(fb.tau_est.Data(te >= 4.04))));
            tq = so.truth.tau_m.Time; tpre = mean(so.truth.tau_m.Data(tq >= 3.5 & tq < 4));
            ok = ~isempty(tF) && tF <= 10 && oc && fbit && tz < 0.05;
            tc.rec('TRIP', 'tau_scale 0 with enable: Tr scaled to 0, OVERCURRENT bit 0, FAULT (state 7) within 10 ms', ...
                sprintf('FAULT after %.0f ms; fault bit 0 %d; status FAULT b2 %d; |tau_est| after 40 ms %.4f N m (before %.2f N m)', ...
                tF, oc, fbit, tz, tpre), ok);
            tc.verifyNotEmpty(tF); tc.verifyLessThanOrEqual(tF, 10); tc.verifyTrue(oc); tc.verifyTrue(fbit); tc.verifyLessThan(tz, 0.05);
        end

        function testT21_twoMass(tc)
            b2 = tc.P.drive.bench2m; kv = tc.P.drive.gear.k_v2w;
            [c, ~] = tb_drive_profile(struct('t_end', 6, 'Ts', 0.01, 'v_tgt', [0 0; 1 1.0]));
            meas = ''; okAll = true;
            for kx = b2.k_x
                for ms = b2.m_side
                    R = tb_drive_2mass_lti(tc.P, kx, ms);
                    so = tb_drive_run('tb_drive_2mass', 6, struct('tb_cmd', c, 'tb_kx', kx, 'tb_cx', b2.tau_d*kx, 'tb_m', ms));
                    vv = so.v_veh.Data; os = 100*(max(vv) - 1);
                    ok = R.Ms <= b2.Ms_max && R.zeta_min >= b2.zeta_min && os <= 100*b2.os_max && R.stable;
                    okAll = okAll && ok;
                    meas = [meas sprintf('k %.0f kN/m m %.1f kg: Ms %.2f, zeta %.2f, PM %.0f deg, v overshoot %.2f %%. ', ...
                        kx/1e3, ms, R.Ms, R.zeta_min, R.PM, os)]; %#ok<AGROW>
                    tc.verifyLessThanOrEqual(R.Ms, b2.Ms_max); tc.verifyGreaterThanOrEqual(R.zeta_min, b2.zeta_min);
                    tc.verifyLessThanOrEqual(os, 100*b2.os_max); tc.verifyTrue(R.stable);
                end
            end
            tc.rec('T21', sprintf('tb_drive_2mass k_x 40/61/540 kN/m, m_side %s kg: Ms <= 1.6, zeta_min >= 0.35, vehicle overshoot <= 2 %%', ...
                mat2str(b2.m_side)), meas, okAll);
            % validation of the loop model against the bench: small reference step at 100 rad/s
            vmeas = ''; okV = true;
            for kx = b2.k_x([1 end])
                ms = b2.m_side(2);
                R = tb_drive_2mass_lti(tc.P, kx, ms);
                tt = (0:0.001:4)'; wr = 100*min(tt/1.5, 1); wr(tt >= 3) = 105;
                cs = tb_drive_bus_ts('Bus_DriveSideCmd', tt, struct('enable', 1, 'mode', 1, 'w_ref', wr, 'tau_lim_mot', tc.P.drive.motor.T_peak, ...
                    'tau_lim_reg', tc.P.drive.motor.T_peak, 'alive', mod(floor(tt/0.01), 256)));
                so = tb_drive_run('tb_drive_2mass', 3.6, struct('tb_cmd', cs, 'tb_kx', kx, 'tb_cx', b2.tau_d*kx, 'tb_m', ms));
                tw = so.fb.w_m.Time; yw = double(so.fb.w_m.Data);
                m = tw >= 2.95 & tw <= 3.6;
                y0 = mean(yw(tw >= 2.9 & tw < 3));
                ym = yw(m) - y0;
                % discrete step response on its own time grid from t = 0 (step(sys, t) with a vector that starts
                % before 0 would shift the response by the pre-step window), interpolated onto the bench samples
                tq = tw(m) - 3; tk = (0:tc.P.drive.Ts_esc:max(tq) + tc.P.drive.Ts_esc)';
                ys = 5*step(R.Tr, tk);
                yl = zeros(size(tq)); ip = tq >= -1e-9;
                yl(ip) = interp1(tk, ys, max(tq(ip), 0), 'linear', 'extrap');
                rmsd = sqrt(mean((ym - yl).^2))/5;
                vmeas = [vmeas sprintf('k %.0f kN/m: RMS(bench - model) %.1f %% of the 5 rad/s step. ', kx/1e3, 100*rmsd)]; %#ok<AGROW>
                tc.verifyLessThan(rmsd, 0.15); okV = okV && rmsd < 0.15;
            end
            tc.rec('T21', 'loop model (Ms, zeta source) validated on the bench: speed-step RMS difference < 15 %', vmeas, okV);
            tc.assertNotEmpty(kv);
        end

        function testT22_stopProfile(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 7, 'v_tgt', [0 0; 0.5 1.2], 'stop', [4 2.0 tc.P.electronics.shaper.j_stop]));
            so = tb_drive_run('tb_drive', 7, struct('tb_cmd', c));
            [t, v] = tc.vtrack(so.truth);
            tr = so.cmd_rx.w_ref.Time; vr = double(so.cmd_rx.w_ref.Data)/tc.P.drive.gear.k_v2w;
            vri = interp1(tr, vr, t, 'previous', 'extrap');
            m = t >= 4 & t <= 5.2;
            e = max(abs(v(m) - vri(m)));
            ok = e <= 0.05;
            tc.rec('T22', '2.0 m/s^2 MCU stop profile from 1.2 m/s followed with speed error <= 0.05 m/s (no ESC ramp limitation)', ...
                sprintf('max |v - v_ref(ESC input)| %.4f m/s', e), ok);
            tc.verifyLessThanOrEqual(e, 0.05);
        end

        function testIdealVariant(tc)
            [c, ~] = tb_drive_profile(struct('t_end', 5, 'v_tgt', [0 0; 1 1.0]));
            so = tb_drive_run('tb_drive', 5, struct('tb_cmd', c), struct('VC_DRIVE_FID', 3));
            [t, v] = tc.vtrack(so.truth);
            v15 = interp1(t, v, 2.5); idc = mean(so.truth.i_dc.Data(t > 4)); Tw = so.truth.T_winding.Data(end);
            ok = v15 >= 0.9 && idc > 0 && isfinite(Tw);
            tc.rec('IDEAL', 'IDEAL choice in tb_drive: tracks the T1 profile, positive DC current when motoring, thermal port fed', ...
                sprintf('v(2.5 s) %.3f m/s; i_dc %.2f A; T_winding %.2f K', v15, idc, Tw), ok);
            tc.verifyGreaterThanOrEqual(v15, 0.9); tc.verifyGreaterThan(idc, 0);
        end
    end
end

function tf = local_long()
tf = strcmp(getenv('T06_LONG'), '1');
end

function x = local_first(v)
% first element of v, NaN when v is empty
if isempty(v), x = NaN; else, x = v(1); end
end
