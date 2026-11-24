classdef t_gnc_tracking < matlab.unittest.TestCase
%T_GNC_TRACKING  Acceptance tests of build task T14 (docs/build_plan.md T14, docs/specs/gnc.md 8).
%
%   Bench tests/gnc/tb_gnc.slx (scripts/build/build_tb_gnc.m): kinematic plant, tau_v 0.35 s, tau_r 0.30 s,
%   0.1 s input dead time + MCU AUTO shaper, RTK noise 1.5 cm / 0.3 deg. Scenario data reach the bench only
%   through Simulink.SimulationInput.setVariable (P, VC_ROW_TRACKER, mpcRowObj): the shared base workspace
%   is never modified (it must hold Ts, Bus_* and cs_SimFixed, created by shaddad_params / configsets when
%   missing). Cross-track errors are TRUTH errors (truth position against the compiled raw path).
%   Every measured number is printed as 'T14 MEASURED <test>: ...' for the build report.
%
%   T1 straight 60 m row 0.8 m/s: XTE RMS < 2 cm, max < 6 cm, |mean| < 1 cm (settled, t > 10 s)
%   T2 row entry with a 0.3 m lateral offset at 0.8 m/s: |XTE| < 5 cm within 4 s, overshoot < 5 cm
%   T3 side-slope drift 0.03 m/s: MPC |mean| < 1 cm and RMS < 2 cm; Stanley |mean| < 4.5 cm; PP reported
%   T4 headland skip-row turn R 0.81 m: max |XTE| in the turn < 15 cm, |XTE| < 5 cm within 2 m after row
%      entry, peak |omega| <= 1.2 rad/s, |v| + |omega| chi B/2 <= 2.0 m/s (with the identified P chi B/2
%      and with the build-plan value 0.595), v at the apex <= 0.5 m/s
%   T5 speed step 0 -> 1 m/s: v_cmd reaches 1.0 in 2.0 +/- 0.15 s, no overshoot, |v - v_cmd| < 0.05 m/s
%      after 3 s, deceleration <= 1.0 m/s^2
%   envelope: v_cmd <= 1.5 m/s with 2.0 m/s requested; omega <= 0.2 rad/s at 20 deg tilt
%   compiler: greenhouse_std.json 10-11 lanes, length within +/-5 % of analytic, turn radii >= R_min,
%      estimated time <= 22 min, outer-lane body edge >= 0.35 m from the wall / post line, compile < 10 s
%   Shaddad_GNC: interface check 0, Model block with 6 inputs updates, 0 algebraic loops, every GNC
%      Subsystem Reference matches build_shaddad_gnc('ports') (contract for T15); code
%      generation with cs_Companion BLOCKED while Simulink Coder is missing (D31)

    properties
        P
    end

    methods (TestClassSetup)
        function setupBase(tc)
            if ~evalin('base', 'exist(''Ts'',''var'') && exist(''Bus_GncCmd'',''var'') && exist(''VC_PHASE'',''var'')')
                shaddad_params();
            end
            if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
                shaddad_configsets('fixed');
            end
            tc.P = shaddad_params('Assign', false);
            tc.applyFixture(matlab.unittest.fixtures.SuppressedWarningsFixture( ...
                {'Simulink:blocks:BusSelectorOutSignalNameUpdate', 'Simulink:Engine:SaveWithParameterizedLinks_Warning', ...
                 'gnc_compileMission:check', 'gnc_compileMission:speed'}));
            root = char(currentProject().RootFolder);
            if ~isfile(fullfile(root, 'models', 'Shaddad_GNC.slx')), build_shaddad_gnc(); end
            if ~isfile(fullfile(root, 'tests', 'gnc', 'tb_gnc.slx')), build_tb_gnc(); end
        end
    end

    methods (TestClassTeardown)
        function closeModels(~)
            for m = {'tb_gnc', 'Shaddad_GNC'}
                if bdIsLoaded(m{1}), close_system(m{1}, 0); end
            end
        end
    end

    %% ================================ compiler =============================================
    methods (Test)
        function testCompilerGreenhouse(tc)
            [plan, info] = gnc_compileMission(tc.P, 'greenhouse_std');
            tc.report('compiler_greenhouse', sprintf(['lanes %d (planner %d), spacing %.3f m, c_in %.3f m, segments %d, ' ...
                'order %s, rows %.1f m + turns %.1f m = %.1f m vs analytic %.1f m (ratio %.4f), R_min turns %.3f m (R %.2f), ' ...
                't_est %.1f min, body edge to wall %.3f m, clearance body+deck %.3f m (posts %.3f, walls %.3f), ' ...
                'lead-in shifts %s, coverage %.1f %%, compile %.2f s'], ...
                info.n_lanes, info.cov_planner.n_planner, info.lane_spacing, info.c_in, info.n_seg, mat2str(info.order), ...
                info.len_rows, info.len_turns, info.len_rows + info.len_turns, info.len_analytic, info.len_ratio, ...
                info.R_min_turns, info.R_req, info.t_est_s/60, info.body_edge_to_wall, info.clear.min, info.clear.min_cyl, ...
                info.clear.min_wall, mat2str(info.lead_in), 100*info.coverage, info.t_compile));
            tc.verifyTrue(plan.valid);
            tc.verifyGreaterThanOrEqual(info.n_lanes, 10);
            tc.verifyLessThanOrEqual(info.n_lanes, 11);
            tc.verifyLessThanOrEqual(abs(info.len_ratio - 1), 0.05);
            tc.verifyGreaterThanOrEqual(info.R_min_turns, info.R_req*(1 - 1e-3));
            tc.verifyLessThanOrEqual(info.t_est_s, 22*60);
            tc.verifyGreaterThanOrEqual(info.body_edge_to_wall, 0.35);
            tc.verifyLessThan(info.t_compile, 10);
            cv = tc.P.gnc.cov;                          % D54 inset (0.40 m for the 0.78 m wide real robot)
            tc.verifyEqual(abs(info.c_in - (cv.gh_post_edge + cv.gh_c_clear + cv.gh_c_xte + max(0, tc.P.gnc.geom.W/2 - cv.gh_swath/2))) < 1e-9, true);
            tc.verifyTrue(info.geofence_ok);
            tc.verifyEqual(double(plan.n_seg), info.n_seg);
            tc.verifyEqual(plan.datum_lla(:)', tc.P.world.lla0, 'AbsTol', 1e-12);
            % S4 / D42: footprint (body + mower deck) clearance >= 0.15 m to posts and walls along the whole plan
            tc.verifyGreaterThanOrEqual(info.clear.min, tc.P.gnc.cov.clear_min);
            tc.verifyGreaterThanOrEqual(info.coverage, 0.98);
            tc.verifyEmpty(info.warnings);
            % every segment has exactly N_raw = 100 poses, consecutive segments join (path continuity)
            N = tc.P.gnc.path.N_raw;
            for k = 1:info.n_seg - 1
                tc.verifyLessThan(norm(double(plan.path(k*N, 1:2) - plan.path(k*N + 1, 1:2))), 0.02, sprintf('seg %d/%d', k, k + 1));
            end
        end

        function testCompilerSlopeTurn(tc)
            % D53: no headland turn where the DEM slope exceeds cov.slope_turn_max. World 2 has a 20 deg ramp
            % for x in [10, 30] m: rows ending at x = 14 m must be shortened until the turn is below 10 deg.
            S = struct('mission_type', 'GOTO', 'world_id', 2, 'rows_enu', [0 0 14 0; 14 4 0 4], 'home_enu', [-3 0 0], ...
                'turn_radius_m', 1.5, 'skip_rows', 0, 'speed_work', 0.8);
            [plan, info] = gnc_compileMission(tc.P, S, 'Validate', false);
            W = param_world(2, tc.P.frames.g);  T = W.terrain;
            gx = griddedInterpolant({T.xE, T.yN}, T.dHdE, 'linear', 'nearest');
            gy = griddedInterpolant({T.xE, T.yN}, T.dHdN, 'linear', 'nearest');
            iT = find(plan.seg(1:plan.n_seg, 1) == SegType.TURN);
            xy = double(plan.path((iT - 1)*100 + (1:100), 1:2));
            sl = max(atan(hypot(gx(xy(:, 1), xy(:, 2)), gy(xy(:, 1), xy(:, 2)))));
            iR = find(plan.seg(1:plan.n_seg, 1) == SegType.ROW);
            tc.report('compiler_slope', sprintf(['world 2 (20 deg ramp x 10..30 m): %d turn, max slope on the turn %.2f deg ' ...
                '(limit %.1f), row ends x %.2f / %.2f m (requested 14), warnings: %s'], numel(iT), rad2deg(sl), ...
                rad2deg(tc.P.gnc.cov.slope_turn_max), plan.path((iR(1) - 1)*100 + 100, 1), plan.path((iR(2) - 1)*100 + 1, 1), ...
                strjoin(info.warnings, '; ')));
            tc.verifyNumElements(iT, 1);
            tc.verifyLessThanOrEqual(sl, tc.P.gnc.cov.slope_turn_max + 1e-9);
            tc.verifyLessThan(plan.path((iR(1) - 1)*100 + 100, 1), single(14));
            tc.verifyTrue(any(contains(info.warnings, 'D53')));
        end

        function testCompilerPalmSolar(tc)
            for nm = {'palm_demo', 'solar_demo'}
                [plan, info] = gnc_compileMission(tc.P, nm{1});
                tc.report(['compiler_' nm{1}], sprintf(['lanes %d, segments %d, length %.1f m, R_min turns %.2f m, ' ...
                    't_est %.1f min, clearance %.2f m, geofence %d, compile %.2f s'], info.n_lanes, info.n_seg, ...
                    info.len_total, info.R_min_turns, info.t_est_s/60, info.clear.min, info.geofence_ok, info.t_compile));
                tc.verifyTrue(plan.valid);
                tc.verifyTrue(info.geofence_ok, nm{1});
                tc.verifyGreaterThanOrEqual(info.R_min_turns, info.R_req*(1 - 1e-3), nm{1});
                tc.verifyGreaterThanOrEqual(info.clear.min, tc.P.gnc.cov.clear_min, nm{1});
                tc.verifyLessThan(info.t_compile, 10, nm{1});
                tc.verifyEqual(plan.seg(plan.dock_seg, 1), single(SegType.DOCK), nm{1});
            end
        end

        function testMpcDesign(tc)
            obj = gnc_designMPC(tc.P, 'Assign', false);
            tc.verifyClass(obj, 'mpc');
            tc.verifyEqual(size(obj.Model.Plant.A), [5 5]);          % [e_y e_psi r u(k-1) u(k-2)]
            mdl = gnc_mpcPlantUpdate(0.8, tc.P.gnc.mpc);
            % two-sample MV dead time: r_cmd(k) first reaches r at k+3
            x = zeros(5, 1); u = [1; 0; 0; 0; 0];
            for k = 1:3
                x = mdl.Plant.A*x + mdl.Plant.B*u*(k == 1);
                if k < 3, tc.verifyEqual(x(3), 0, 'AbsTol', 1e-12); end
            end
            tc.verifyGreaterThan(x(3), 0);
        end

        %% ============================ model / interface =====================================
        function testInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'Shaddad_GNC'}, 'Quiet', true);
            tc.verifyEqual(n, 0);
            load_system('tb_gnc');
            ph = get_param('tb_gnc/GNC', 'PortHandles');
            tc.verifyEqual(numel(ph.Inport), 6);
            tc.verifyEqual(numel(ph.Outport), 4);
            % compiled with the base workspace (P with an invalid plan -> dummy path); mpcRowObj is created
            % there when missing (gnc_designMPC is the only GNC base write, as in shaddad_startup)
            if ~evalin('base', 'exist(''mpcRowObj'',''var'')'), evalin('base', 'gnc_designMPC(P);'); end
            t0 = tic;
            evalc('set_param(''tb_gnc'', ''SimulationCommand'', ''update'')');
            tUpd = toc(t0);
            loops = Simulink.BlockDiagram.getAlgebraicLoops('tb_gnc');
            tc.report('interfaces', sprintf('check_interfaces %d, GNC Model block %d in / %d out, update %.1f s, algebraic loops %d', ...
                n, numel(ph.Inport), numel(ph.Outport), tUpd, numel(loops)));
            tc.verifyEmpty(loops);
            close_system('tb_gnc', 0);
        end

        function testSsrPortContract(tc)
            % every GNC Subsystem Reference has the ports of build_shaddad_gnc('ports') (names, order, data
            % types): the contract the T15 replacements of the stubs must keep
            T = build_shaddad_gnc('ports');
            fn = fieldnames(T);
            nStub = 0;
            for k = 1:numel(fn)
                n = fn{k};
                wasLoaded = bdIsLoaded(n);
                load_system(n);
                for d = {'Inport', 'Outport'}
                    b = find_system(n, 'SearchDepth', 1, 'BlockType', d{1});
                    [~, o] = sort(cellfun(@(x) str2double(get_param(x, 'Port')), b));
                    if strcmp(d{1}, 'Inport'), ref = T.(n).in; else, ref = T.(n).out; end
                    tc.verifyEqual(reshape(get_param(b(o), 'Name'), 1, []), {ref.name}, [n ' ' d{1} ' names']);
                    tc.verifyEqual(reshape(get_param(b(o), 'OutDataTypeStr'), 1, []), {ref.type}, [n ' ' d{1} ' types']);
                end
                nStub = nStub + startsWith(strtrim(get_param(n, 'Description')), 'SHADDAD_GNC_STUB');
                if ~wasLoaded, close_system(n, 0); end
            end
            tc.report('ssr_ports', sprintf('%d GNC Subsystem References match the port table (%d still T14 stubs)', ...
                numel(fn), nStub));
        end

        function testCodegenCompanion(tc)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
            evalin('base', 'shaddad_configsets(''fixed'');');
            tc.verifyTrue(evalin('base', 'exist(''cs_Companion'',''var'')') > 0);
        end

        %% =================================== tracking =======================================
        function testT1StraightRow(tc)
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 60 0], 'home_enu', [0 0 0], 'speed_work', 0.8);
            r = gnc_tb_run(tc.P, S, 'StopTime', 82, 'Tracker', 1);
            e = r.truth(:, 2);                                     % row along +x: XTE = y
            w = r.t > 10 & r.truth(:, 1) < 56;
            m = struct('rms', sqrt(mean(e(w).^2)), 'max', max(abs(e(w))), 'mean', mean(e(w)));
            psiRms = sqrt(mean(r.truth(w, 3).^2));
            tc.report('T1', sprintf('MPC 0.8 m/s: XTE RMS %.4f m, max %.4f m, mean %.4f m (settled t > 10 s, x < 56 m), heading RMS %.2f deg, wall %.1f s', ...
                m.rms, m.max, m.mean, rad2deg(psiRms), r.wall));
            tc.verifyLessThan(m.rms, 0.02);
            tc.verifyLessThan(m.max, 0.06);
            tc.verifyLessThan(abs(m.mean), 0.01);
            tc.verifyLessThanOrEqual(max(r.v_cmd), tc.P.gnc.spd.v_max + 1e-6);
        end

        function testT2RowEntry(tc)
            % two ROW segments 0.3 m apart: at the switch the row tracker sees a 0.3 m step in e_y at 0.8 m/s
            s1 = struct('type', 'ROW', 'xy', [-12 0.3; 0 0.3], 'v', 0.8);
            s2 = struct('type', 'ROW', 'xy', [0 0; 40 0], 'v', 0.8);
            S = struct('mission_type', 'GOTO', 'segments', {{s1, s2}}, 'home_enu', [-12 0.3 0], 'speed_work', 0.8);
            r = gnc_tb_run(tc.P, S, 'StopTime', 32, 'Tracker', 1);
            seg = interp1(r.ts, r.seg_idx, r.t, 'previous', 'extrap');
            i0 = find(seg >= 2, 1);
            t0 = r.t(i0);  e = r.truth(:, 2);
            after = r.t >= t0 & r.truth(:, 1) < 36;
            iLast = find(after & abs(e) >= 0.05, 1, 'last');
            tSettle = r.t(iLast + 1) - t0;
            over = max(0, -min(e(after)));
            wlim = min(interp1(tc.P.gnc.cmd.w_max_v, tc.P.gnc.cmd.w_max, min(abs(r.v_cmd), 2)), tc.P.gnc.mpc.u_max);
            wsat = local_longest(abs(r.w_cmd) >= wlim - 1e-3, 0.05);     % omega at its bound
            tc.report('T2', sprintf(['entry at t %.2f s, x %.2f m, e_y %.3f m, v %.3f m/s: |XTE| < 5 cm %.2f s after entry, ' ...
                'overshoot %.3f m, longest omega saturation %.2f s, wall %.1f s'], t0, r.truth(i0, 1), e(i0), r.truth(i0, 4), ...
                tSettle, over, wsat, r.wall));
            tc.verifyGreaterThan(e(i0), 0.25);
            tc.verifyLessThan(tSettle, 4);
            tc.verifyLessThan(over, 0.05);
            tc.verifyLessThanOrEqual(wsat, 0.5);
        end

        function testT3DriftMPC(tc)
            m = tc.driftRun(1, 'ROW');
            tc.verifyLessThan(abs(m.mean), 0.01);
            tc.verifyLessThan(m.rms, 0.02);
        end

        function testT3DriftStanley(tc)
            m = tc.driftRun(2, 'ROW');
            tc.verifyLessThan(abs(m.mean), 0.045);
        end

        function testT3DriftPurePursuit(tc)
            m = tc.driftRun(1, 'TRANSIT');          % TRANSIT segment -> Pure Pursuit (reported only)
            tc.verifyTrue(isfinite(m.mean));
        end

        function testT4HeadlandTurn(tc)
            S = tc.turnMission();
            r = gnc_tb_run(tc.P, S, 'StopTime', 62, 'Tracker', 1);
            c = tc.P.gnc.cmd.chi*tc.P.gnc.cmd.B/2;               % identified chi_nom (T03, D39) * B/2
            env = abs(r.v_cmd) + abs(r.w_cmd)*c;
            cN = 1.40*tc.P.gnc.cmd.B/2;                         % chi 1.40 (build-plan number) on the real gauge
            env595 = abs(r.v_cmd) + abs(r.w_cmd)*cN;
            seg = interp1(r.ts, r.seg_idx, r.t, 'previous', 'extrap');
            inTurn = seg == 2;
            eTurn = t_gnc_tracking.xte(r.plan, 2, r.truth(inTurn, 1:2));
            % apex: closest approach to the turn's farthest point from the lane line
            tp = double(r.plan.path(101:200, 1:2));
            [~, ia] = max(tp(:, 1));
            [~, ja] = min(hypot(r.truth(:, 1) - tp(ia, 1), r.truth(:, 2) - tp(ia, 2)));
            vApex = r.truth(ja, 4);
            % second row: |XTE| < 5 cm within 2 m after entry
            i3 = find(seg == 3, 1);
            row3 = seg == 3 & r.truth(:, 1) > 1.0;
            e3 = t_gnc_tracking.xte(r.plan, 3, r.truth(row3, 1:2));
            d3 = r.truth(i3, 1) - r.truth(row3, 1);                % distance travelled along -x since entry
            bad = find(abs(e3) >= 0.05, 1, 'last');
            if isempty(bad), dSettle = 0; else, dSettle = d3(bad); end
            tc.report('T4', sprintf(['turn R %.2f m (length %.2f m): max |XTE| %.3f m, peak |omega_cmd| %.3f rad/s, ' ...
                'max |v|+|w|chiB/2 %.3f m/s (P chiB/2 %.4f) / %.3f m/s (chi 1.40), ' ...
                'v at apex %.3f m/s, max v in turn %.3f m/s; row entry: |XTE| < 5 cm after %.2f m (max |XTE| on row %.3f m), wall %.1f s'], ...
                tc.P.gnc.cov.gh_R, r.plan.seg(2, 3), max(abs(eTurn)), max(abs(r.w_cmd)), max(env), c, max(env595), vApex, ...
                max(r.truth(inTurn, 4)), dSettle, max(abs(e3)), r.wall));
            tc.verifyLessThan(max(abs(eTurn)), 0.15);
            tc.verifyLessThanOrEqual(max(abs(r.w_cmd)), tc.P.gnc.cmd.w_max(1) + 1e-6);
            tc.verifyLessThanOrEqual(max(env), tc.P.gnc.cmd.v_track_max + 1e-6);
            tc.verifyLessThanOrEqual(max(env595), 1.05*tc.P.gnc.cmd.v_track_max + 1e-6);
            tc.verifyLessThanOrEqual(vApex, 0.5 + 1e-3);
            tc.verifyLessThanOrEqual(dSettle, 2.0);
        end

        function testT5SpeedStep(tc)
            s1 = struct('type', 'TRANSIT', 'xy', [0 0; 30 0], 'v', 1.0);
            S = struct('mission_type', 'GOTO', 'segments', {{s1}}, 'home_enu', [0 0 0], 'speed_transit', 1.0);
            r = gnc_tb_run(tc.P, S, 'StopTime', 40, 'Tracker', 1);
            i0 = find(r.v_cmd > 0, 1);  t0 = r.tc(i0) - 0.05;        % last zero sample = step time
            i1 = find(r.v_cmd >= 1.0 - 1e-6, 1);
            tRise = r.tc(i1) - t0;
            over = max(r.v_cmd) - 1.0;
            vm = interp1(r.t, r.truth(:, 4), r.tc);
            cruise = r.tc >= t0 + 3 & r.v_cmd >= 1.0 - 1e-6;
            dv = max(abs(vm(cruise) - r.v_cmd(cruise)));
            dec = -min(diff(r.v_cmd)/0.05);
            tc.report('T5', sprintf(['v_cmd 0 -> 1.0 m/s in %.3f s, overshoot %.4f m/s, max |v - v_cmd| after 3 s %.4f m/s, ' ...
                'max deceleration of v_cmd %.3f m/s^2, max v_cmd %.3f m/s, wall %.1f s'], tRise, over, dv, dec, max(r.v_cmd), r.wall));
            tc.verifyLessThanOrEqual(abs(tRise - 2.0), 0.15);
            tc.verifyLessThanOrEqual(over, 1e-6);
            tc.verifyLessThan(dv, 0.05);
            tc.verifyLessThanOrEqual(dec, 1.0 + 1e-6);
        end

        function testEnvelopeVmax(tc)
            % 2.0 m/s requested on a straight transit (compiler clips to v_max, governor saturates at v_max)
            s1 = struct('type', 'TRANSIT', 'xy', [0 0; 40 0], 'v', 2.0);
            S = struct('mission_type', 'GOTO', 'segments', {{s1}}, 'home_enu', [0 0 0], 'speed_transit', 2.0);
            [plan, ~] = gnc_compileMission(tc.P, S, 'Validate', false);
            plan.seg(1, 4) = single(2.0);                       % bypass the compiler clip: test the model envelope
            r = gnc_tb_run(tc.P, S, 'StopTime', 25, 'Plan', plan);
            tc.report('envelope_vmax', sprintf('segment speed 2.0 m/s requested: max v_cmd %.4f m/s (limit %.2f)', ...
                max(r.v_cmd), tc.P.gnc.spd.v_max));
            tc.verifyLessThanOrEqual(max(r.v_cmd), tc.P.gnc.spd.v_max + 1e-6);
        end

        function testTiltOmegaLimit(tc)
            r = gnc_tb_run(tc.P, tc.turnMission(), 'StopTime', 62, 'tilt', [deg2rad(20) 0]);
            tc.report('tilt_omega', sprintf('pitch 20 deg: max |omega_cmd| %.4f rad/s (limit 0.2), max v_cmd %.3f m/s', ...
                max(abs(r.w_cmd)), max(r.v_cmd)));
            tc.verifyLessThanOrEqual(max(abs(r.w_cmd)), 0.2 + 1e-6);
        end
    end

    %% ================================ helpers ===============================================
    methods
        function report(~, name, txt)
            fprintf('T14 MEASURED %s: %s\n', name, txt);
        end

        function m = driftRun(tc, tracker, type)
            s1 = struct('type', type, 'xy', [0 0; 60 0], 'v', 0.8);
            S = struct('mission_type', 'GOTO', 'segments', {{s1}}, 'home_enu', [0 0 0], 'speed_transit', 0.8, 'speed_work', 0.8);
            r = gnc_tb_run(tc.P, S, 'StopTime', 82, 'Tracker', tracker, 'drift', 0.03);
            e = r.truth(:, 2);
            w = r.t > 15 & r.truth(:, 1) < 56;
            m = struct('rms', sqrt(mean(e(w).^2)), 'max', max(abs(e(w))), 'mean', mean(e(w)));
            names = {'MPC', 'Stanley'};  nm = names{tracker};
            if strcmp(type, 'TRANSIT'), nm = 'PurePursuit'; end
            tc.report(['T3_' nm], sprintf('drift 0.03 m/s, 0.8 m/s: XTE mean %.4f m, RMS %.4f m, max %.4f m (t > 15 s, x < 56 m), wall %.1f s', ...
                m.mean, m.rms, m.max, r.wall));
        end

        function S = turnMission(tc)
            d = 2*0.856;                                         % greenhouse skip-row spacing (2 x 0.856 m)
            S = struct('mission_type', 'GOTO', 'rows_enu', [0 0 12 0; 12 d 0 d], 'home_enu', [0 0 0], ...
                'speed_work', tc.P.gnc.spd.v_work_greenhouse, 'turn_radius_m', tc.P.gnc.cov.gh_R, 'skip_rows', 0, ...
                'speed_turn_max', tc.P.gnc.spd.v_turn_max);
        end
    end

    methods (Static)
        function e = xte(plan, k, q)
            % signed distance (+ left) of the points q to the raw polyline of plan segment k
            pts = double(plan.path((k-1)*100 + (1:100), 1:2));
            e = zeros(size(q, 1), 1);
            for i = 1:size(q, 1)
                a = pts(1:end-1, :);  ab = pts(2:end, :) - a;
                L2 = max(sum(ab.^2, 2), 1e-12);
                t = min(max(((q(i, 1) - a(:, 1)).*ab(:, 1) + (q(i, 2) - a(:, 2)).*ab(:, 2))./L2, 0), 1);
                px = a(:, 1) + t.*ab(:, 1);  py = a(:, 2) + t.*ab(:, 2);
                d = hypot(q(i, 1) - px, q(i, 2) - py);
                [dm, j] = min(d);
                e(i) = sign(ab(j, 1)*(q(i, 2) - py(j)) - ab(j, 2)*(q(i, 1) - px(j)))*dm;
            end
        end
    end
end

function L = local_longest(mask, dt)
% longest run of true samples in mask, in seconds
L = 0; run = 0;
for k = 1:numel(mask)
    if mask(k), run = run + 1; else, run = 0; end
    L = max(L, run);
end
L = L*dt;
end
