classdef t_world < matlab.unittest.TestCase
    %T_WORLD  Acceptance tests of build task T02 (world, terrain and environment; owner structure).
    %
    %   results = runtests('tests/structure/t_world.m')        everything (about 1-2 min)
    %   runtests('t_world', 'Tag', 'World')    P.world builders, checks, geometry (< 30 s)
    %   runtests('t_world', 'Tag', 'Bench')    structure_env bench tb_structure_env (incl. 24 h diurnal run)
    %   runtests('t_world', 'Tag', 'Build')    rerun build_structure_env / build_tb_structure_env + port check
    %
    %   Criteria (docs/build_plan.md T02 "Acceptance"): world_build ids 3-6 < 30 s each; world_check passes
    %   ids 1-6 (gnssSvf / gnssZoneId sized like the terrain grid); palm_farm memory < 40 MB, nCyl = 91,
    %   nRows = 6, mean dHdN = 0.005 +/- 0.0005, residual roughness std 0.015 +/- 0.003 m, gradient error
    %   vs numeric < 1e-9, occupancy cells within +/-5 % of sum(pi r^2)*100; greenhouse inner area and zone
    %   area 360 +/- 0.5 m^2, post spacing 3.0 +/- 0.01 m, gnssSvf 0.12 inside / 1 outside; solar 5 tables,
    %   pier spacing 6.0 m, zBase 0.8, gnssZoneId 2 under tables / 3 between rows; structure_env bench:
    %   T_amb 318.15 constant, override 300 K within one 0.1 s sample, diurnal min 302.15 / max 320.15
    %   +/- 0.05 K at 15:00 +/- 1 min, dock contact true at the dock pose and false at 0.06 m offset or
    %   0.1 m/s, G under canopy = 0.6 G; 0 algebraic loops, compiled rates in P.sim.Ts_compiled_allowed.
    %   Measured numbers are written to the test diagnostics (log) of every test.
    %   The shared base workspace is not modified: P / Ts / tbe go to the bench by SimulationInput.setVariable.

    properties
        Worlds cell         % P.world of ids 1..6
        Tb double           % build time per id [s]
        P1 struct           % P of world 1 (flat_test)
        P3 struct           % P of world 3 (palm_farm)
        Ts struct
        Loaded cell
    end

    properties (Constant)
        Bench = 'tb_structure_env';
    end

    methods (TestClassSetup)
        function setupData(tc)
            if ~evalin('base', 'exist(''Bus_EnvState'',''var'') && exist(''Bus_EnvOverride'',''var'') && exist(''cs_SimFixed'',''var'')')
                shaddad_startup('quiet');
            end
            tc.Loaded = find_system('type', 'block_diagram');
            tc.addTeardown(@() t_world.closeNew(tc.Loaded));
            [tc.P1, tc.Ts] = shaddad_params('World', 1, 'Assign', false);
            tc.P3 = shaddad_params('World', 3, 'Assign', false);
            g = tc.P1.frames.g;
            tc.Worlds = cell(1, 6);  tc.Tb = zeros(1, 6);
            for id = 1:6
                t0 = tic;
                tc.Worlds{id} = param_world(id, g);
                tc.Tb(id) = toc(t0);
            end
        end
    end

    methods (Static)
        function closeNew(loaded)
            % close only this task's diagrams (bench, structure_env) that were not loaded at setup;
            % structure_env stays loaded while another loaded model references it (shared session)
            if bdIsLoaded(t_world.Bench) && ~any(strcmp(loaded, t_world.Bench))
                try, close_system(t_world.Bench, 0); catch, end %#ok<CTCH>
            end
            if bdIsLoaded('structure_env') && ~any(strcmp(loaded, 'structure_env')) && isempty(t_world.envUsers())
                try, close_system('structure_env', 0); catch, end %#ok<CTCH>
            end
        end

        function u = envUsers()
            % loaded models / subsystem files (not libraries, not the bench) that reference structure_env
            u = {};
            bd = find_system('type', 'block_diagram');
            for k = 1:numel(bd)
                if any(strcmp(bd{k}, {t_world.Bench, 'structure_env'})) || ...
                        strcmp(get_param(bd{k}, 'BlockDiagramType'), 'library')
                    continue
                end
                try
                    if ~isempty(find_system(bd{k}, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
                            'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem', ...
                            'ReferencedSubsystem', 'structure_env'))
                        u{end+1} = bd{k}; %#ok<AGROW>
                    end
                catch %#ok<CTCH>
                end
            end
        end

        function [X, Y] = grid(W)
            [X, Y] = ndgrid(W.terrain.xE, W.terrain.yN);
        end
    end

    %% =========================================================================================
    methods (Test, TestTags = {'World'})
        function testBuildTime(tc)
            for id = 3:6
                tc.log(1, sprintf('world_build id %d: %.2f s (criterion < 30 s)', id, tc.Tb(id)));
                tc.verifyLessThan(tc.Tb(id), 30, sprintf('world %d build time', id));
            end
        end

        function testWorldCheckAllIds(tc)
            for id = 1:6
                W = tc.Worlds{id};
                [ok, msg] = world_check(W, 'Psens', tc.P1.sensors);
                tc.verifyTrue(ok, sprintf('world %d: %s', id, strjoin(msg, '; ')));
                sz = [numel(W.terrain.xE) numel(W.terrain.yN)];
                tc.verifySize(W.gnssSvf, sz, sprintf('world %d gnssSvf size', id));
                tc.verifySize(W.gnssZoneId, sz, sprintf('world %d gnssZoneId size', id));
                tc.verifyEqual(W.id, id);
                tc.verifyEqual(fieldnames(W), fieldnames(tc.Worlds{1}), 'field contract / order');
                tc.log(1, sprintf('world %d: check ok, grid %d x %d, occ %d x %d', id, sz, size(W.occ.grid)));
            end
        end

        function testSingleSourceWithSensors(tc)
            S = tc.P1.sensors;
            D = world_gnss_grids('defaults');
            for f = fieldnames(D)'
                tc.verifyEqual(D.(f{1}), S.gnss.svf.(f{1}), ['world_gnss_grids default ' f{1} ' ~= P.sensors.gnss.svf']);
            end
            O = world_occupancy('defaults');
            tc.verifyEqual(O.zBand, S.lidar.zBand, 'occupancy height band ~= P.sensors.lidar.zBand');
            tc.verifyEqual(O.res, S.lidar.mapRes, 'occupancy resolution ~= P.sensors.lidar.mapRes');
            tc.verifyEqual(tc.P3.world.gnssSvf, world_gnss_grids(tc.P3.world, tc.P3.sensors), 'P.world.gnssSvf (palm)');
            tc.verifyEqual(tc.P3.sensors.env.nCyl, 91, 'P.sensors.env derived from P.world');
        end

        function testParamsAssemblyAllWorlds(tc)
            for id = 3:6
                P = shaddad_params('World', id, 'Assign', false);
                tc.verifyEqual(P.world.id, id);
                tc.verifyEqual(P.scen.world_id, id);
                tc.verifyTrue(world_check(P.world, 'Psens', P.sensors), sprintf('world %d in P', id));
                tc.verifyTrue(all(isfinite(P.structure.init.xme0(:))), sprintf('world %d initial state', id));
            end
        end

        function testDeterministic(tc)
            for id = 3:6
                W2 = param_world(id, tc.P1.frames.g);
                tc.verifyTrue(isequal(W2, tc.Worlds{id}), sprintf('world %d not reproducible', id));
            end
        end

        function testPalmFarm(tc)
            W = tc.Worlds{3};
            s = whos('W');
            mb = s.bytes/2^20;
            tc.log(1, sprintf('palm_farm memory %.1f MB (< 40)', mb));
            tc.verifyLessThan(mb, 40);
            O = W.obstacles;
            tc.verifyEqual(O.nCyl, 91);
            tc.verifyEqual(W.nRows, 6);
            c = O.cyl(1:O.nCyl, :);
            tc.verifyEqual(unique(c(:,1))', 8*(0:6) + 4, 'palm rows x = 8k+4');
            tc.verifyEqual(unique(c(:,2))', 8*(0:12) + 12, 'palms y = 8j+12');
            tc.verifyTrue(all(c(:,3) >= 0.30 & c(:,3) <= 0.40), 'trunk r U(0.30,0.40)');
            tc.verifyTrue(all(c(:,4) == 6 & c(:,5) == 0 & c(:,6) == 1), 'trunk h 6, zBase 0, class 1');
            tc.verifyEqual(W.nCanopy, 91);
            R = W.canopy(1:W.nCanopy, 3);
            tc.verifyTrue(all(R >= 3.0 & R <= 4.0), 'canopy R U(3,4)');
            tc.verifyEqual(W.rows(:, [1 3])', repmat(8*(0:5) + 8, 2, 1), 'aisles x = 8k+8');
            tc.verifyEqual(W.rows(:, [2 4]), repmat([8 112], 6, 1), 'aisles y 8..112');
            tc.verifyEqual(W.boundary, [-8 -8; 72 -8; 72 128; -8 128]);
            tc.verifyEqual(W.start, [-5 0 pi/2]);
            tc.verifyEqual(W.dock.pose, [-6 -5 pi/2]);
            tc.verifyEqual(W.xlim, [-10 74]);  tc.verifyEqual(W.ylim, [-10 130]);
            % terrain
            mN = mean(W.terrain.dHdN(:));
            [X, Y] = t_world.grid(W);
            far = true(size(X));
            for k = 1:O.nCyl
                far = far & hypot(X - c(k,1), Y - c(k,2)) > 2.5;      % outside the basin berms
            end
            r = W.terrain.H - 0.005*Y;
            sr = std(r(far));
            [gN, gE] = gradient(W.terrain.H, W.terrain.res);
            eG = max(abs([gN(:) - W.terrain.dHdN(:); gE(:) - W.terrain.dHdE(:)]));
            nOcc = nnz(W.occ.grid);  aOcc = sum(pi*c(:,3).^2)*W.occ.res^2;
            tc.log(1, sprintf(['palm_farm: mean dHdN %.6f (0.005 +/- 0.0005), roughness std %.5f m ' ...
                '(0.015 +/- 0.003), gradient error %.3g (< 1e-9), occupancy %d cells vs %.1f (%+.2f %%, +/-5 %%)'], ...
                mN, sr, eG, nOcc, aOcc, 100*(nOcc/aOcc - 1)));
            tc.verifyEqual(mN, 0.005, 'AbsTol', 0.0005);
            tc.verifyEqual(sr, 0.015, 'AbsTol', 0.003);
            tc.verifyLessThan(eG, 1e-9);
            tc.verifyEqual(nOcc/aOcc, 1, 'AbsTol', 0.05);
            % basins, berm, soil, vegetation, shade
            [~, info] = world_build(W, 3);
            Hb = info.comp.H_berm;
            tc.verifyEqual(max(Hb(:)), 0.15, 'AbsTol', 1e-3, 'berm height');
            i0 = find(abs(W.terrain.xE - c(1,1)) < 1e-9);  j0 = find(abs(W.terrain.yN - c(1,2)) < 1e-9);
            tc.verifyEqual(Hb(i0 + 6, j0), 0.15, 'AbsTol', 1e-9, 'berm crest at r = 1.5 m');
            tc.verifyEqual(W.soilId(i0, j0), 3, 'wet loam in the basin');
            tc.verifyEqual(W.shade(i0, j0), 0.6, 'shade under canopy');
            tc.verifyEqual(W.veg.rho(i0, j0), 2.0, 'veg under canopy');
            tc.verifyEqual(W.gnssSvf(i0, j0), tc.P1.sensors.gnss.svf.canopyAtt, 'AbsTol', 1e-12, 'svf under canopy');
            fs = nnz(W.soilId == 1)/numel(W.soilId);
            tc.log(1, sprintf('palm_farm: dry-sand fraction %.3f (10 %% discs outside the basins), %d discs', fs, info.comp.nSandDiscs));
            tc.verifyGreaterThan(fs, 0.08);  tc.verifyLessThan(fs, 0.12);
            tc.verifyTrue(all(ismember(W.soilId(:), [1 2 3])), 'palm_farm soils 1..3 only');
        end

        function testGreenhouse(tc)
            W = tc.Worlds{4};
            O = W.obstacles;
            b = O.box(1:O.nBox, :);
            tc.verifyEqual(O.nBox, 5);
            tc.verifyTrue(all(b(:,8) == 4 & b(:,6) == 3 & b(:,3) == 0), 'wall boxes class 4, 3 m high');
            tc.verifyTrue(all(min(b(:,4:5), [], 2) == 0.1), 'wall thickness 0.1 m');
            % inner faces of the wall boxes
            xW = max(b(b(:,1) < 4.8 & b(:,5) > 1, 1) + b(b(:,1) < 4.8 & b(:,5) > 1, 4)/2);
            xEa = min(b(b(:,1) > 4.8 & b(:,5) > 1, 1) - b(b(:,1) > 4.8 & b(:,5) > 1, 4)/2);
            yS = max(b(b(:,2) < 18 & b(:,4) > 1, 2) + b(b(:,2) < 18 & b(:,4) > 1, 5)/2);
            yNo = min(b(b(:,2) > 18 & b(:,4) > 1, 2) - b(b(:,2) > 18 & b(:,4) > 1, 5)/2);
            aIn = (xEa - xW)*(yNo - yS);
            Z = W.gnssZones;
            aZ = polyarea(Z.X(1, 1:Z.nv(1)), Z.Y(1, 1:Z.nv(1)));
            p = O.cyl(1:O.nCyl, :);
            dW = diff(sort(p(abs(p(:,1) - 0.15) < 1e-9, 2)));
            dE = diff(sort(p(abs(p(:,1) - 9.45) < 1e-9, 2)));
            % door: gap in the south wall
            sw = b(b(:,2) < 0, :);
            gap = [max(sw(sw(:,1) < 4.8, 1) + sw(sw(:,1) < 4.8, 4)/2), min(sw(sw(:,1) > 4.8, 1) - sw(sw(:,1) > 4.8, 4)/2)];
            [X, Y] = t_world.grid(W);
            in  = X > 0 & X < 9.6 & Y > 0 & Y < 37.5;
            out = X < 0 | X > 9.6 | Y < 0 | Y > 37.5;
            tc.log(1, sprintf(['greenhouse: inner area %.3f m^2, zone area %.3f m^2 (360 +/- 0.5), post spacing ' ...
                '[%.4f %.4f] m (3.0 +/- 0.01), door x %.2f..%.2f, svf inside %s outside %s'], aIn, aZ, ...
                min([dW; dE]), max([dW; dE]), gap, mat2str(unique(W.gnssSvf(in))'), mat2str(unique(W.gnssSvf(out))')));
            tc.verifyEqual(aIn, 360, 'AbsTol', 0.5);
            tc.verifyEqual(aZ, 360, 'AbsTol', 0.5);
            tc.verifyEqual(numel(dW), 12);  tc.verifyEqual(numel(dE), 12);
            tc.verifyEqual([dW; dE], 3.0*ones(24, 1), 'AbsTol', 0.01);
            tc.verifyTrue(all(p(:,3) == 0.05 & p(:,6) == 2), 'posts r 0.05 class 2');
            tc.verifyEqual(gap, [3.3 6.3], 'AbsTol', 1e-9, 'door x 3.3..6.3');
            tc.verifyEqual(O.nFilm, 5, 'film segments');
            tc.verifyEqual(Z.n, 1);  tc.verifyEqual(Z.svf(1), 0.12);  tc.verifyEqual(Z.type(1), 1);
            tc.verifyTrue(all(W.gnssSvf(in) == 0.12), 'gnssSvf 0.12 inside');
            tc.verifyTrue(all(W.gnssSvf(out) == 1), 'gnssSvf 1 outside');
            tc.verifyTrue(all(W.gnssZoneId(in) == 1) && all(W.gnssZoneId(out) == 0), 'gnssZoneId');
            tc.verifyTrue(all(W.soilId(in) == 4) && all(W.soilId(out) == 2), 'soil 4 inside, 2 outside');
            tc.verifyTrue(all(W.shade(in) == 0.6) && all(W.veg.rho(in) == 1.5), 'shade 0.6 / veg 1.5 inside');
            tc.verifyEqual(W.start, [4.8 -2 pi/2]);  tc.verifyEqual(W.dock.pose, [4.8 -4 pi/2]);
            tc.verifyEqual(W.nRows, 1);
            % the door is free in the occupancy grid: row of the south-wall centre line y = -0.05,
            % cell centres x 3.45..6.15 free, x 0.05..3.15 occupied (row 1 = max y)
            oc = W.occ;
            ir = size(oc.grid, 1) - floor((-0.05 - oc.origin(2))*oc.res);
            col = @(x) round((x - oc.origin(1))*oc.res + 0.5);
            tc.verifyFalse(any(oc.grid(ir, col(3.45:0.1:6.15))), 'door blocked in occ');
            tc.verifyTrue(all(oc.grid(ir, col(0.05:0.1:3.15))), 'south wall missing in occ');
        end

        function testSolarFarm(tc)
            W = tc.Worlds{5};
            O = W.obstacles;
            b = O.box(1:O.nBox, :);
            tc.verifyEqual(O.nBox, 5, 'tables');
            tc.verifyTrue(all(b(:,8) == 6 & b(:,3) == 0.8 & b(:,6) == 1.5 & b(:,5) == 4 & b(:,4) == 40), ...
                'tables class 6, zBase 0.8, lz 1.5, 40 x 4 m');
            tc.verifyEqual(b(:,2)', 7*(1:5), 'table centres y = 7k');
            p = O.cyl(1:O.nCyl, :);
            tc.verifyEqual(O.nCyl, 70);
            tc.verifyTrue(all(p(:,6) == 3 & p(:,3) == 0.08), 'piers class 3 r 0.08');
            dx = [];
            for y = unique(p(:,2))'
                dx = [dx; diff(sort(p(p(:,2) == y, 1)))]; %#ok<AGROW>
            end
            [X, Y] = t_world.grid(W);
            yc = 7*(1:5);
            under = false(size(X));  betw = false(size(X));
            for k = 1:5, under = under | (X > 0 & X < 40 & abs(Y - yc(k)) < 2); end
            for k = 1:4, betw = betw | (X > 0 & X < 40 & Y > yc(k) + 2 & Y < yc(k+1) - 2); end
            tc.log(1, sprintf(['solar_farm: %d tables, pier spacing %.4f..%.4f m, zBase %s, gnssZoneId under %s ' ...
                'between %s, svf under %s between %s'], O.nBox, min(dx), max(dx), mat2str(unique(b(:,3))'), ...
                mat2str(unique(W.gnssZoneId(under))'), mat2str(unique(W.gnssZoneId(betw))'), ...
                mat2str(unique(W.gnssSvf(under))'), mat2str(unique(W.gnssSvf(betw))')));
            tc.verifyEqual(dx, 6.0*ones(size(dx)), 'AbsTol', 1e-9);
            off = abs(p(:,2) - round(p(:,2)/7)*7);
            tc.verifyEqual(off, 1.4*ones(size(off)), 'AbsTol', 1e-9, 'piers at yc +/- 1.4');
            tc.verifyTrue(all(W.gnssZoneId(under) == 2), 'zone 2 under tables');
            tc.verifyTrue(all(W.gnssZoneId(betw) == 3), 'zone 3 between rows');
            tc.verifyTrue(all(W.gnssSvf(under) == 0.45) && all(W.gnssSvf(betw) == 0.75), 'svf 0.45 / 0.75');
            tc.verifyTrue(all(W.shade(under) == 0.2) && all(W.shade(~under & (X < 0 | X > 40)) == 1), 'shade');
            tc.verifyTrue(all(W.veg.rho(:) == 2.0) && all(W.soilId(:) == 2), 'veg 2.0, soil 2');
            tc.verifyEqual(W.rows(:, 2)', 7*(0:5) + 3.5, 'aisles y = 7k + 3.5');
            tc.verifyEqual(W.nRows, 6);
        end

        function testSoilStrips(tc)
            W = tc.Worlds{6};
            tc.verifyEqual([W.xlim(2) - W.xlim(1), W.ylim(2) - W.ylim(1)], [60 60], '60 x 60 m');
            tc.verifyTrue(all(W.terrain.H(:) == 0), 'flat');
            [X, Y] = t_world.grid(W);
            for k = 1:5
                m = Y > 10*(k-1) + 0.5 & Y < 10*k - 0.5;
                tc.verifyTrue(all(W.soilId(m) == k), sprintf('strip %d', k));
                tc.verifyEqual(W.soil.muPeak(m & X > 0 & X < 40), W.soilTable(k, 2)*ones(nnz(m & X > 0 & X < 40), 1), ...
                    'AbsTol', 1e-12, sprintf('strip %d muPeak', k));
            end
            tc.verifyEqual(W.nRows, 5);
            tc.verifyEqual(W.rows(:, 2)', 10*(1:5) - 5);
        end

        function testSoilBlending(tc)
            % property grids are the 3x3 box blend of the raw table values (edge replicated)
            W = tc.Worlds{3};
            raw = reshape(W.soilTable(W.soilId(:), 2), size(W.soilId));
            Ap = raw([1 1:end end], [1 1:end end]);
            B = conv2(Ap, ones(3)/9, 'valid');
            tc.verifyEqual(W.soil.muPeak, B, 'AbsTol', 1e-12);
            tc.verifyGreaterThan(numel(unique(W.soil.muPeak(:))), 3, 'blend produces intermediate values');
        end

        function testPlot(tc)
            f = world_plot(tc.Worlds{3}, 'Visible', 'off');
            tc.addTeardown(@() close(f));
            tc.verifyTrue(isgraphics(f, 'figure'));
        end
    end

    %% =========================================================================================
    methods (Test, TestTags = {'Bench'})
        function testBenchConstantTamb(tc)
            y = tc.simBench(tc.P1, struct(), 2);
            T = y{1}.Values.Data;
            tc.log(1, sprintf('T_amb min %.4f max %.4f K (318.15 constant), T_sky %.2f, T_ground %.2f, G %.1f', ...
                min(T), max(T), y{2}.Values.Data(end), y{3}.Values.Data(end), y{4}.Values.Data(end)));
            tc.verifyEqual(T, 318.15*ones(size(T)), 'AbsTol', 1e-12);
            tc.verifyEqual(y{2}.Values.Data(end), 318.15 + tc.P1.world.env.T_sky_offset, 'AbsTol', 1e-12);
            tc.verifyEqual(y{3}.Values.Data(end), 318.15 + tc.P1.world.env.T_ground_offset, 'AbsTol', 1e-12);
            tc.verifyEqual(y{4}.Values.Data(end), tc.P1.world.env.G_solar, 'AbsTol', 1e-12);
        end

        function testBenchOverride(tc)
            t_ovr = 1.0;
            y = tc.simBench(tc.P1, struct('use_T', 1, 'T_ovr', 300, 't_ovr', t_ovr), 2);
            T = y{1}.Values.Data;  t = y{1}.Values.Time;
            k = find(abs(T - 300) < 1e-9, 1);
            dt = t(k) - t_ovr;
            tc.log(1, sprintf('override 300 K applied at t_ovr = %.1f s, reflected at t = %.2f s (delay %.2f s, <= 0.1 s)', ...
                t_ovr, t(k), dt));
            tc.verifyLessThanOrEqual(dt, 0.1 + 1e-9);
            tc.verifyTrue(all(abs(T(t >= t_ovr + 0.1 - 1e-9) - 300) < 1e-9), 'stays at 300 K');
            tc.verifyTrue(all(abs(T(t < t_ovr - 1e-9) - 318.15) < 1e-9), 'base before the override');
        end

        function testBenchDock(tc)
            P = tc.P1;  d = P.world.dock;
            cases = { 'at dock pose',      d.pose,                   0,    false, true
                      '0.06 m offset x',   d.pose + [0.06 0 0],      0,    false, false
                      '0.06 m offset y',   d.pose + [0 0.06 0],      0,    false, false
                      '0.1 m/s',           d.pose,                   0.1,  false, false
                      'yaw + 4 deg',       d.pose + [0 0 4*pi/180],  0,    false, false
                      'yaw + 2*pi',        d.pose + [0 0 2*pi],      0,    false, true
                      '0.04 m offset',     d.pose + [0.04 0 0],      0,    false, true
                      'reverse -0.1 m/s',  d.pose,                   -0.1, false, false
                      'force, 0.06 m',     d.pose + [0.06 0 0],      0.1,  true,  true };
            for k = 1:size(cases, 1)
                [nm, pose, v, force, expect] = cases{k, :};
                y = tc.simBench(P, struct('pose', pose, 'speed', v, 'dock_force', force), 0.5);
                c = y{6}.Values.Data;  dd = y{7}.Values.Data(end);
                tc.log(1, sprintf('dock: %-17s dist %.4f m -> contact %d (expected %d)', nm, dd, c(end), expect));
                tc.verifyEqual(logical(c(:)'), repmat(expect, 1, numel(c)), nm);
            end
        end

        function testBenchCanopyAndSoil(tc)
            P = tc.P3;
            c = P.world.canopy(1, :);
            y = tc.simBench(P, struct('pose', [c(1:2) 0]), 0.3);
            G = y{4}.Values.Data(end);
            yo = tc.simBench(P, struct('pose', P.world.start), 0.3);
            Go = yo{4}.Values.Data(end);
            yG = tc.simBench(P, struct('pose', [c(1:2) 0], 'use_G', true, 'G_ovr', 500), 0.3);
            tc.log(1, sprintf('palm_farm: G under canopy %.2f W/m^2, open %.2f (ratio %.4f, 0.6), override 500 under canopy -> %.2f', ...
                G, Go, G/Go, yG{4}.Values.Data(end)));
            tc.verifyEqual(G, 0.6*Go, 'AbsTol', 1e-9);
            tc.verifyEqual(Go, P.world.env.G_solar, 'AbsTol', 1e-9);
            tc.verifyEqual(yG{4}.Values.Data(end), 0.6*500, 'AbsTol', 1e-9);
            tc.verifyEqual(y{3}.Values.Data(end), y{1}.Values.Data(end) + P.world.env.T_ground_offset*G/1000, 'AbsTol', 1e-9);
            tc.verifyEqual(double(y{5}.Values.Data(end)), 3, 'wet loam at the palm');
            % flat_test soil strips (asphalt y 20..26, sand y 30..36) and the soil override
            P1 = tc.P1;
            ya = tc.simBench(P1, struct('pose', [10 22 0]), 0.3);
            ys = tc.simBench(P1, struct('pose', [10 33 0]), 0.3);
            yv = tc.simBench(P1, struct('pose', [10 33 0], 'use_soil', true, 'soil_ovr', uint8(4)), 0.3);
            sid = double([ya{5}.Values.Data(end), ys{5}.Values.Data(end), yv{5}.Values.Data(end)]);
            tc.log(1, sprintf('flat_test soil_id asphalt %d, sand %d, override 4 -> %d (class %s)', sid, class(ya{5}.Values.Data)));
            tc.verifyEqual(sid, [5 1 4]);
            tc.verifyClass(ya{5}.Values.Data, 'uint8');
            tc.verifyClass(ya{6}.Values.Data, 'logical');
        end

        function testBenchDiurnal(tc)
            P = tc.P1;
            P.world.env.use_diurnal = true;
            y = tc.simBench(P, struct(), 86400, {'Decimation', '10'});
            T = y{1}.Values;  G = y{4}.Values;
            [mx, imx] = max(T.Data);  [mn, imn] = min(T.Data);
            h0 = P.world.env.diurnal.t0_hour;
            hMax = h0 + T.Time(imx)/3600;  hMin = mod(h0 + T.Time(imn)/3600, 24);
            [gmx, ig] = max(G.Data);
            tc.log(1, sprintf(['diurnal: T max %.4f K at %02d:%05.2f, T min %.4f K at %02d:%05.2f ' ...
                '(criteria 320.15 / 302.15 +/- 0.05 K, max at 15:00 +/- 1 min); G max %.1f W/m^2 at %.3f h'], ...
                mx, floor(hMax), 60*mod(hMax, 1), mn, floor(hMin), 60*mod(hMin, 1), gmx, h0 + G.Time(ig)/3600));
            tc.verifyEqual(mx, 320.15, 'AbsTol', 0.05);
            tc.verifyEqual(mn, 302.15, 'AbsTol', 0.05);
            tc.verifyEqual(hMax, 15, 'AbsTol', 1/60);
            tc.verifyGreaterThanOrEqual(min(G.Data), 0);
            tc.verifyEqual(gmx, P.world.env.diurnal.G_pk, 'AbsTol', 1e-6);
        end

        function testBenchLoopsAndRates(tc)
            mdl = t_world.Bench;
            if ~bdIsLoaded(mdl), load_system(mdl); end
            % compile with the base P / Ts (the bench reads them like every model of the project)
            tc.assumeTrue(evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'')'), 'P / Ts not in base');
            nLoops = -1;
            evalc('nLoops = numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));');
            ts = Simulink.BlockDiagram.getSampleTimes(mdl);
            v = [];
            for k = 1:numel(ts)
                if isnumeric(ts(k).Value) && ~isempty(ts(k).Value), v(end+1) = ts(k).Value(1); end %#ok<AGROW>
            end
            allowed = evalin('base', 'P.sim.Ts_compiled_allowed');
            bad = v(~arrayfun(@(x) any(abs(x - allowed) < 1e-12 | (isinf(x) & isinf(allowed))), v));
            tc.log(1, sprintf('tb_structure_env: %d algebraic loops, compiled rates %s', nLoops, mat2str(unique(v))));
            tc.verifyEqual(nLoops, 0);
            tc.verifyEmpty(bad, 'compiled rates outside P.sim.Ts_compiled_allowed');
        end
    end

    %% =========================================================================================
    methods (Test, TestTags = {'Build'})
        function testBuildScriptsAndPorts(tc)
            users = t_world.envUsers();
            tc.assumeEmpty(users, sprintf(['structure_env is referenced by loaded model(s) of another task (%s); ' ...
                'rebuild skipped'], strjoin(users, ', ')));
            if bdIsLoaded(t_world.Bench), close_system(t_world.Bench, 0); end
            t0 = tic;  build_structure_env();  tEnv = toc(t0);
            t0 = tic;  build_tb_structure_env();  tTb = toc(t0);
            tc.log(1, sprintf('build_structure_env %.1f s, build_tb_structure_env %.1f s', tEnv, tTb));
            load_system('structure_env');
            tc.addTeardown(@() t_world.closeNew(tc.Loaded));
            ins = find_system('structure_env', 'SearchDepth', 1, 'BlockType', 'Inport');
            outs = find_system('structure_env', 'SearchDepth', 1, 'BlockType', 'Outport');
            pin = get_param(ins, 'Port');  [~, o] = sort(str2double(pin));  ins = ins(o);
            tc.verifyEqual(get_param(ins, 'Name')', {'pose2D', 'speed', 'env_ovr'});
            tc.verifyEqual(get_param(ins, 'OutDataTypeStr')', {'double', 'double', 'Bus: Bus_EnvOverride'});
            tc.verifyEqual(get_param(ins, 'SampleTime')', {'0', '0', 'Ts.env'});
            tc.verifyEqual(get_param(ins{1}, 'PortDimensions'), '3');
            tc.verifyEqual(get_param(outs, 'Name'), {'env'});
            tc.verifyEqual(get_param(outs{1}, 'OutDataTypeStr'), 'Bus: Bus_EnvState');
            mf = find_system('structure_env', 'LookUnderMasks', 'all', 'SFBlockType', 'MATLAB Function');
            tc.verifyEmpty(mf, 'structure_env must contain library blocks only');
            tc.verifyEmpty(find_system('structure_env', 'BlockType', 'SimscapeBlock'), 'no Simscape blocks');
        end
    end

    %% =========================================================================================
    methods
        function y = simBench(tc, P, ovr, stopTime, extra)
            %SIMBENCH  Simulate tb_structure_env with P, Ts and tbe fields overridden by ovr.
            if nargin < 5, extra = {}; end
            mdl = t_world.Bench;
            if ~bdIsLoaded(mdl), load_system(mdl); end
            tbe = struct('pose', [0 0 0], 'speed', 0, 'use_T', 0, 'T_ovr', 300, 't_ovr', 1.0, ...
                         'use_G', false, 'G_ovr', 500, 'use_soil', false, 'soil_ovr', uint8(5), 'dock_force', false);
            for f = fieldnames(ovr)'
                tbe.(f{1}) = ovr.(f{1});
            end
            in = Simulink.SimulationInput(mdl);
            in = in.setVariable('P', P).setVariable('Ts', tc.Ts).setVariable('tbe', tbe, 'Workspace', mdl);
            in = in.setModelParameter('StopTime', num2str(stopTime), extra{:});
            out = sim(in);
            tc.assertEmpty(out.ErrorMessage, 'bench simulation error');
            y = out.yout;
        end
    end
end
