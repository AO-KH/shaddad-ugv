classdef t_t07_nav < matlab.unittest.TestCase
%T_T07_NAV  Card T-07 step 7: the MIL unit tests of the transitions that had none in the step 5 inventory
%   (work/team/t07/inv/nav.json): the navigation mode manager's N5 to N8 (subsystems/sensors/sens_nav_mode.m),
%   the D89 health flag navValid (T-NAV-14, card G1-02), and every transition of the GNSS fix chain, G1 to G9, and of
%   the moving-base heading, H1 to H3 (subsystems/sensors/sens_gnss_fixchain.m).
%
%   runtests('t_t07_nav')
%
%   The functions run as MATLAB code with their persistent state cleared before each case, scripted like
%   t_nav/tModeUnit: the mode manager at its 0.01 s step with P.sensors.navmode, the fix chain at its 0.1 s step with
%   sens_gnss_fixchain_prm(P.sensors.gnss) and its four draws u forced to each side of a branch (a draw 1e-6 below the
%   step probability p = 1 - exp(-lambda dt) takes the transition, 1e-6 above does not). The values come from P (D114:
%   these are checks of the model's own logic). Timers are checked to two steps (the time is a sum of 0.01 s steps).

    properties
        P
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.P = shaddad_params('Assign', false);
        end
    end

    methods (TestMethodTeardown)
        function clearState(~)
            clear sens_nav_mode sens_gnss_fixchain
        end
    end

    methods (Static)
        function m(id, fmt, varargin)
            fprintf('MEASURED %s: %s\n', id, sprintf(fmt, varargin{:}));
        end

        function p = pr(lam, dt)
            p = 1 - exp(-lam*dt);                 % as sens_gnss_fixchain/local_p
        end
    end

    methods (Access = private)
        function [f, M] = modeFn(tc, M)
        %MODEFN  Handle of one step of sens_nav_mode, persistent state cleared: f(t, navMode, fix, avail, en, Pd)
        %   with the IMU valid, standing still, low pack current, the dual-antenna heading valid, and nis = 1.
            if nargin < 2, M = tc.P.sensors.navmode; end
            clear sens_nav_mode
            f = @(t, cmd, fix, av, en, Pd) sens_nav_mode(t, uint8(cmd), false, true, true, true, true, uint8(fix), ...
                av, en, ones(5, 1), Pd, M);
        end

        function [t, st] = toRun(tc, f, cmd)
        %TORUN  INIT -> ALIGN (2.5 s, no measurement) -> RUN_GNSS (N3) on one accepted RTK-fixed epoch with a small P.
            t = 0;  no = false(5, 1);  g = [true; true; false; true; false];
            for k = 1:250, t = t + 0.01; f(t, 0, 4, no, no, [1e4 1e4 9.87]); end
            t = t + 0.01;
            st = f(t, cmd, 4, g, g, [1e-4 1e-4 1e-5]);
            tc.assertEqual(st, uint8(2), 'RUN_GNSS after the alignment');
        end

        function c = chain(tc, s0)
        %CHAIN  Fix-chain constants with the initial state s0 (c(2) = initFix), persistent state cleared.
            c = sens_gnss_fixchain_prm(tc.P.sensors.gnss);
            c(2) = s0;
            clear sens_gnss_fixchain
        end
    end

    methods (Test, TestTags = {'T07'})
        % ------------------------------------------------------------------ navigation mode manager
        function tModeN5_GnssToLidar(tc)
            % N5 RUN_GNSS -> RUN_LIDAR: (a) on request (navMode 2); (b) in AUTO on a mapped site after tLidarSwitch
            % without an RTK epoch; (c) not without a prior map
            M = tc.P.sensors.navmode;  Ps = [1e-4 1e-4 1e-5];  g = [true; true; false; true; false];
            f = tc.modeFn();
            t = tc.toRun(f, 0);
            [st, ~, ~, ~, ~, lo] = f(t + 0.01, 2, 4, g, g, Ps);
            tc.verifyEqual(st, uint8(3), 'N5 on request: RUN_LIDAR at the next step');
            tc.verifyTrue(lo, 'lidarLocOn in RUN_LIDAR');
            for M0 = [M, setfield(M, 'priorMap', 0)] %#ok<SFLD>
                f = tc.modeFn(M0);
                t = tc.toRun(f, 0);  t0 = t;  tSw = NaN;  stBefore = true;
                for k = 1:700
                    t = t + 0.01;
                    st = f(t, 0, 1, g, g, Ps);                    % accepted SINGLE epochs: no RTK
                    if st == 3, tSw = t - t0; break, end
                    stBefore = stBefore && st == 2;
                end
                if M0.priorMap > 0
                    t_t07_nav.m('T07-N5', 'AUTO, no RTK epoch: RUN_LIDAR after %.2f s (tLidarSwitch %.1f s)', tSw, M0.tLidarSwitch);
                    tc.verifyEqual(tSw, M0.tLidarSwitch, 'AbsTol', 2*M0.dt, 'N5 in AUTO after tLidarSwitch without RTK');
                    tc.verifyTrue(stBefore, 'RUN_GNSS until then');
                else
                    tc.verifyTrue(isnan(tSw) && stBefore, 'no prior map: RUN_GNSS kept for 7 s without RTK');
                end
            end
        end

        function tModeN6_LidarToDeadReckoning(tc)
            % N6 RUN_LIDAR -> DEAD_RECKONING after drTimeout without an accepted GNSS position or scan match;
            % lidarLocOn stays on (a prior map, navMode 2) and quality is DR
            M = tc.P.sensors.navmode;  Ps = [1e-4 1e-4 1e-5];  g = [true; true; false; true; false];
            od = [false; false; false; true; false];
            f = tc.modeFn();
            t = tc.toRun(f, 0);
            t = t + 0.01;  st = f(t, 2, 4, g, g, Ps);
            tc.assertEqual(st, uint8(3), 'RUN_LIDAR');
            t0 = t;  tDr = NaN;  stBefore = true;
            for k = 1:400
                t = t + 0.01;
                [st, q, nv, ~, ~, lo] = f(t, 2, 4, od, od, Ps);  % odometry only
                if st == 4, tDr = t - t0; break, end
                stBefore = stBefore && st == 3;
            end
            t_t07_nav.m('T07-N6', 'RUN_LIDAR without an absolute update: DEAD_RECKONING after %.2f s (drTimeout %.1f s); quality %d, lidarLocOn %d, navValid %d', ...
                tDr, M.drTimeout, q, lo, nv);
            tc.verifyEqual(tDr, M.drTimeout, 'AbsTol', 2*M.dt, 'N6 after drTimeout');
            tc.verifyTrue(stBefore, 'RUN_LIDAR until then');
            tc.verifyEqual(q, uint8(1), 'quality DR');
            tc.verifyTrue(lo, 'lidarLocOn kept in DEAD_RECKONING with a prior map and navMode 2');
            tc.verifyTrue(nv, 'navValid (tSinceAbs < tAbsMax)');
        end

        function tModeN7_LidarToGnss(tc)
            % N7 RUN_LIDAR -> RUN_GNSS: (a) on request (navMode 1); (b) in AUTO after tGnssReturn of RTK fixed, an RTK
            % float epoch not counting
            M = tc.P.sensors.navmode;  Ps = [1e-4 1e-4 1e-5];  g = [true; true; false; true; false];
            sm = [false; false; false; true; true];
            f = tc.modeFn();
            t = tc.toRun(f, 0);
            t = t + 0.01;  st = f(t, 2, 4, g, g, Ps);
            tc.assertEqual(st, uint8(3), 'RUN_LIDAR');
            t = t + 0.01;  st = f(t, 2, 1, sm, sm, Ps);
            tc.verifyEqual(st, uint8(3), 'RUN_LIDAR held on scan matches');
            t = t + 0.01;  [st, ~, ~, ~, ~, lo] = f(t, 1, 1, sm, sm, Ps);
            tc.verifyEqual(st, uint8(2), 'N7 on request: RUN_GNSS at the next step');
            tc.verifyFalse(lo, 'lidarLocOn off in RUN_GNSS');
            % (b) AUTO: RUN_LIDAR after 5 s without RTK, then 3 s of RTK float, then RTK fixed
            f = tc.modeFn();
            t = tc.toRun(f, 0);
            for k = 1:520, t = t + 0.01; st = f(t, 0, 1, g, g, Ps); end
            tc.assertEqual(st, uint8(3), 'RUN_LIDAR in AUTO');
            fl = true;
            for k = 1:300, t = t + 0.01; st = f(t, 0, 5, g, g, Ps); fl = fl && st == 3; end
            tc.verifyTrue(fl, 'RTK float for 3 s: RUN_LIDAR kept');
            t0 = t;  tRet = NaN;  stBefore = true;
            for k = 1:400
                t = t + 0.01;
                st = f(t, 0, 4, g, g, Ps);
                if st == 2, tRet = t - t0; break, end
                stBefore = stBefore && st == 3;
            end
            t_t07_nav.m('T07-N7', 'AUTO in RUN_LIDAR: RTK float 3 s kept RUN_LIDAR %d; RTK fixed: RUN_GNSS after %.2f s (tGnssReturn %.1f s)', ...
                fl, tRet, M.tGnssReturn);
            tc.verifyEqual(tRet, M.tGnssReturn, 'AbsTol', 2*M.dt, 'N7 in AUTO after tGnssReturn of RTK fixed');
            tc.verifyTrue(stBefore, 'RUN_LIDAR until then');
        end

        function tModeN8_DeadReckoningToLidar(tc)
            % N8 DEAD_RECKONING -> RUN_LIDAR on an accepted scan match (fresh, navMode not GNSS), quality
            % LIDAR_LOCALIZED; with navMode 1 the same match gives RUN_GNSS (N9)
            M = tc.P.sensors.navmode;  Ps = [1e-4 1e-4 1e-5];  g = [true; true; false; true; false];
            od = [false; false; false; true; false];  sm = [false; false; false; true; true];
            for cmd = [2 1]
                f = tc.modeFn();
                t = tc.toRun(f, 0);
                t = t + 0.01;  f(t, cmd, 4, g, g, Ps);
                for k = 1:250, t = t + 0.01; st = f(t, cmd, 4, od, od, Ps); end
                tc.assertEqual(st, uint8(4), sprintf('DEAD_RECKONING (navMode %d)', cmd));
                t = t + 0.01;
                [st, q, nv, ~, ~, lo, ~, tsa] = f(t, cmd, 4, sm, sm, Ps);
                if cmd == 2
                    t_t07_nav.m('T07-N8', 'DEAD_RECKONING, one accepted scan match: state %d, quality %d, navValid %d, lidarLocOn %d, tSinceAbs %.2f s', ...
                        st, q, nv, lo, tsa);
                    tc.verifyEqual(st, uint8(3), 'N8: RUN_LIDAR');
                    tc.verifyEqual(q, uint8(5), 'quality LIDAR_LOCALIZED');
                    tc.verifyTrue(lo && nv);
                    tc.verifyEqual(tsa, 0, 'AbsTol', 1e-12);
                else
                    tc.verifyEqual(st, uint8(2), 'navMode 1: the scan match gives RUN_GNSS (N9), not RUN_LIDAR');
                    tc.verifyFalse(lo);
                end
            end
        end

        function tNAV14_NavValidHealthFlag(tc)
            % D89 (T-NAV-14, card G1-02): navValid needs the position sigma below sigPosValid (2 m) and an absolute
            % update within tAbsMax (10 s); quality INVALID when it is false
            M = tc.P.sensors.navmode;  g = [true; true; false; true; false];  od = [false; false; false; true; false];
            f = tc.modeFn();
            t = tc.toRun(f, 0);
            s2 = M.sigPosValid^2;
            P3 = [s2*(1 - 1e-3), s2*(1 - 1e-3), 1e-5;  s2, 1e-4, 1e-5;  1e-4, s2*(1 + 1e-3), 1e-5];
            nv = false(1, 3);  q = zeros(1, 3);  st = zeros(1, 3);
            for j = 1:3
                t = t + 0.01;
                [st(j), q(j), nv(j)] = f(t, 0, 4, g, g, P3(j, :));
            end
            t_t07_nav.m('T-NAV-14', 'RUN_GNSS: navValid %d at sigma %.4f m, %d at %.4f m, %d at %.4f m (sigPosValid %.1f m); quality %s', ...
                nv(1), sqrt(P3(1, 1)), nv(2), M.sigPosValid, nv(3), sqrt(P3(3, 2)), M.sigPosValid, mat2str(q));
            tc.verifyEqual(st, [2 2 2], 'RUN_GNSS kept (the sigma does not change the state)');
            tc.verifyEqual(nv, [true false false], 'navValid below 2 m only');
            tc.verifyEqual(q, [4 0 0], 'quality RTK_FIXED, then INVALID');
            % no absolute update: DEAD_RECKONING after 2 s, navValid until tAbsMax
            t = t + 0.01;  f(t, 0, 4, g, g, [1e-4 1e-4 1e-5]);  t0 = t;  tInv = NaN;  qDr = NaN;
            for k = 1:1200
                t = t + 0.01;
                [st1, q1, nv1] = f(t, 0, 4, od, od, [1e-4 1e-4 1e-5]);
                if st1 == 4 && isnan(qDr), qDr = double(q1); end
                if ~nv1, tInv = t - t0; break, end
            end
            t_t07_nav.m('T-NAV-14', 'no absolute update: quality %d in DEAD_RECKONING, navValid false after %.2f s (tAbsMax %.1f s), quality %d', ...
                qDr, tInv, M.tAbsMax, q1);
            tc.verifyEqual(qDr, 1, 'quality DR while navValid');
            tc.verifyEqual(tInv, M.tAbsMax, 'AbsTol', 2*M.dt, 'navValid false after tAbsMax without an absolute update');
            tc.verifyEqual(st1, uint8(4), 'still DEAD_RECKONING');
            tc.verifyEqual(q1, uint8(0), 'quality INVALID');
        end

        % ------------------------------------------------------------------ GNSS fix chain (plant)
        function tFixG1_Outage(tc)
            % G1 any -> NOFIX on a forced outage (and no outlier); afterwards the chain re-acquires through SINGLE (G9)
            for s0 = [4 5 1]
                c = tc.chain(s0);
                [s, ~, o] = sens_gnss_fixchain(0.5, [1 1 1 0], true, 1, true, false, c);
                tc.verifyEqual(s, uint8(0), sprintf('G1 from %d: NOFIX', s0));
                tc.verifyFalse(o, 'no outlier in an outage');
                s = sens_gnss_fixchain(0.5, [0 1 1 1], true, 1, false, false, c);
                tc.verifyEqual(s, uint8(1), 'after the outage: SINGLE (G9)');
            end
        end

        function tFixG2_FixedToFloat(tc)
            % G2 RTK_FIXED -> RTK_FLOAT: corrections older than corrAgeFixLimit, forceFloat, or the draw u(1)
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;  u1 = [1 1 1 1];
            pF = t_t07_nav.pr(m.l_fx_fl0 + m.l_fx_fl1*(1 - sv)^2, G.dt);
            cases = {G.corrAgeFixLimit + 0.5, false, 1, 5;  G.corrAgeFixLimit - 0.5, false, 1, 4;  1, true, 1, 5;  ...
                     1, false, pF*(1 - 1e-6), 5;  1, false, pF*(1 + 1e-6), 4};
            for k = 1:size(cases, 1)
                c = tc.chain(4);
                u = u1;  u(1) = cases{k, 3};
                s = sens_gnss_fixchain(sv, u, true, cases{k, 1}, false, cases{k, 2}, c);
                tc.verifyEqual(s, uint8(cases{k, 4}), sprintf('G2 case %d: corrAge %.1f, forceFloat %d, u1 %.6g', k, ...
                    cases{k, 1}, cases{k, 2}, cases{k, 3}));
            end
            t_t07_nav.m('T07-G2', 'FIXED -> FLOAT step probability %.5f at svf %.1f', pF, sv);
        end

        function tFixG3_FixedToNofix(tc)
            % G3 RTK_FIXED -> NOFIX: the draw u(2) below p(l_nofix (svf_nofix - svf)); none at svf >= svf_nofix
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.1;
            pN = t_t07_nav.pr(m.l_nofix*(m.svf_nofix - sv), G.dt);
            for k = 1:3
                c = tc.chain(4);
                u2 = [pN*(1 - 1e-6), pN*(1 + 1e-6), 0];  svk = [sv sv m.svf_nofix];  want = [0 4 4];
                s = sens_gnss_fixchain(svk(k), [1 u2(k) 1 1], true, 1, false, false, c);
                tc.verifyEqual(s, uint8(want(k)), sprintf('G3 case %d: svf %.2f, u2 %.6g', k, svk(k), u2(k)));
            end
        end

        function tFixG4_FloatToFixed(tc)
            % G4 RTK_FLOAT -> RTK_FIXED: the draw u(1) with the corrections up and fresh and no forceFloat
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pX = t_t07_nav.pr(m.l_fl_fx*sv^3, G.dt);
            % {u1, corrOK, corrAge, forceFloat, expected}
            cases = {pX*(1 - 1e-6), true, 1, false, 4;  pX*(1 + 1e-6), true, 1, false, 5;  0, false, 1, false, 5;  ...
                     0, true, G.corrAgeFixLimit, false, 5;  0, true, 1, true, 5};
            for k = 1:size(cases, 1)
                c = tc.chain(5);
                s = sens_gnss_fixchain(sv, [cases{k, 1} 1 1 1], cases{k, 2}, cases{k, 3}, false, cases{k, 4}, c);
                tc.verifyEqual(s, uint8(cases{k, 5}), sprintf('G4 case %d', k));
            end
        end

        function tFixG5_FloatToSingle(tc)
            % G5 RTK_FLOAT -> SINGLE: corrections older than corrAgeFloatLimit, or the draw u(1) above 1 - p
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pS = t_t07_nav.pr(m.l_fl_sg*(1 - sv)^3, G.dt);
            cases = {0.5, G.corrAgeFloatLimit + 0.5, 1;  0.5, G.corrAgeFloatLimit - 0.5, 5;  ...
                     1 - pS*(1 - 1e-6), 1, 1;  1 - pS*(1 + 1e-6), 1, 5};
            for k = 1:size(cases, 1)
                c = tc.chain(5);
                s = sens_gnss_fixchain(sv, [cases{k, 1} 1 1 1], true, cases{k, 2}, false, false, c);
                tc.verifyEqual(s, uint8(cases{k, 3}), sprintf('G5 case %d', k));
            end
        end

        function tFixG6_FloatToNofix(tc)
            % G6 RTK_FLOAT -> NOFIX: the draw u(2) below p(lamNF) when neither G4 nor G5 fires
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.1;
            pN = t_t07_nav.pr(m.l_nofix*(m.svf_nofix - sv), G.dt);
            for k = 1:2
                c = tc.chain(5);
                u2 = [pN*(1 - 1e-6), pN*(1 + 1e-6)];  want = [0 5];
                s = sens_gnss_fixchain(sv, [0.5 u2(k) 1 1], true, 1, false, false, c);
                tc.verifyEqual(s, uint8(want(k)), sprintf('G6 case %d', k));
            end
        end

        function tFixG7_SingleToFloat(tc)
            % G7 SINGLE -> RTK_FLOAT: the draw u(1) with the corrections up and younger than corrAgeFloatLimit
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pF = t_t07_nav.pr(m.l_sg_fl*sv^2, G.dt);
            cases = {pF*(1 - 1e-6), true, 1, 5;  pF*(1 + 1e-6), true, 1, 1;  0, false, 1, 1;  ...
                     0, true, G.corrAgeFloatLimit, 1};
            for k = 1:size(cases, 1)
                c = tc.chain(1);
                s = sens_gnss_fixchain(sv, [cases{k, 1} 1 1 1], cases{k, 2}, cases{k, 3}, false, false, c);
                tc.verifyEqual(s, uint8(cases{k, 4}), sprintf('G7 case %d', k));
            end
        end

        function tFixG8_SingleToNofix(tc)
            % G8 SINGLE -> NOFIX: the draw u(2) below p(lamNF)
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.1;
            pN = t_t07_nav.pr(m.l_nofix*(m.svf_nofix - sv), G.dt);
            for k = 1:2
                c = tc.chain(1);
                u2 = [pN*(1 - 1e-6), pN*(1 + 1e-6)];  want = [0 1];
                s = sens_gnss_fixchain(sv, [1 u2(k) 1 1], true, 1, false, false, c);
                tc.verifyEqual(s, uint8(want(k)), sprintf('G8 case %d', k));
            end
        end

        function tFixG9_NofixToSingle(tc)
            % G9 NOFIX -> SINGLE: the draw u(1) below p(l_nf_sg1 svf + l_nf_sg0); never straight to an RTK state
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pS = t_t07_nav.pr(m.l_nf_sg1*sv + m.l_nf_sg0, G.dt);
            for k = 1:2
                c = tc.chain(0);
                u1 = [pS*(1 - 1e-6), pS*(1 + 1e-6)];  want = [1 0];
                s = sens_gnss_fixchain(sv, [u1(k) 0 1 1], true, 1, false, false, c);
                tc.verifyEqual(s, uint8(want(k)), sprintf('G9 case %d', k));
            end
            t_t07_nav.m('T07-G9', 'NOFIX -> SINGLE step probability %.5f at svf %.1f', pS, sv);
        end

        % ------------------------------------------------------------------ moving-base heading (plant)
        function tRelposH1_Lose(tc)
            % H1 RELPOS_VALID -> RELPOS_INVALID: the draw u(3) below p(h_lose0 + h_lose1 (1 - svf)^2)
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pL = t_t07_nav.pr(m.h_lose0 + m.h_lose1*(1 - sv)^2, G.dt);
            for k = 1:2
                c = tc.chain(4);
                [~, h0] = sens_gnss_fixchain(sv, [1 1 0 1], true, 1, false, false, c);
                tc.assertTrue(h0, 'RELPOS valid first');
                u3 = [pL*(1 - 1e-6), pL*(1 + 1e-6)];  want = [false true];
                [~, h] = sens_gnss_fixchain(sv, [1 1 u3(k) 1], true, 1, false, false, c);
                tc.verifyEqual(h, want(k), sprintf('H1 case %d', k));
            end
        end

        function tRelposH2_Gain(tc)
            % H2 RELPOS_INVALID -> RELPOS_VALID: the draw u(3) below p(h_gain svf^2); relposFixed only with a fix
            G = tc.P.sensors.gnss;  m = G.markov;  sv = 0.5;
            pG = t_t07_nav.pr(m.h_gain*sv^2, G.dt);
            for k = 1:2
                c = tc.chain(4);
                u3 = [pG*(1 - 1e-6), pG*(1 + 1e-6)];  want = [true false];
                [~, h] = sens_gnss_fixchain(sv, [1 1 u3(k) 1], true, 1, false, false, c);
                tc.verifyEqual(h, want(k), sprintf('H2 case %d', k));
            end
            c = tc.chain(0);
            [s, h] = sens_gnss_fixchain(sv, [1 1 0 1], true, 1, false, false, c);
            tc.verifyEqual(s, uint8(0));
            tc.verifyFalse(h, 'heading gained in NOFIX: relposFixed false (needs fixType >= 1)');
            [s, h] = sens_gnss_fixchain(sv, [0 1 1 1], true, 1, false, false, c);
            tc.verifyEqual(s, uint8(1));
            tc.verifyTrue(h, 'with the SINGLE fix: relposFixed true (the heading state was kept)');
        end

        function tRelposH3_Outage(tc)
            % H3 any -> RELPOS_INVALID on a forced outage; not regained until a gain draw
            c = tc.chain(4);
            [~, h0] = sens_gnss_fixchain(0.5, [1 1 0 1], true, 1, false, false, c);
            tc.assertTrue(h0, 'RELPOS valid first');
            [s, h] = sens_gnss_fixchain(0.5, [1 1 1 1], true, 1, true, false, c);
            tc.verifyEqual(s, uint8(0));
            tc.verifyFalse(h, 'H3: relposFixed false in the outage');
            [s, h] = sens_gnss_fixchain(0.5, [0 1 1 1], true, 1, false, false, c);
            tc.verifyEqual(s, uint8(1));
            tc.verifyFalse(h, 'after the outage: invalid until a gain draw');
        end
    end
end
