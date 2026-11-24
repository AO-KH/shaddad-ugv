classdef t_perception < matlab.unittest.TestCase
%T_PERCEPTION  Build task T13 acceptance tests: Shaddad_Perception, sens_scan_filter, sens_acq_lidar, F4 / F8-F11.
%
%   runtests('tests/sensors/t_perception.m')
%   Benches: tests/sensors/tb_perception.slx (Model Shaddad_Perception driven by recorded FAST2D scans,
%   per_scan_gen / per_tb_input / per_tb_run), tb_per_scanfilter.slx (bit-exact scan filter), tb_per_tebvfh.slx +
%   tb_per_tebvfh_top.slx (T-PER-05). Built by scripts/build/build_perception.m and build_tb_perception.m.
%   The base workspace is not modified (P goes into every run with SimulationInput.setVariable; the HW / FULL3D
%   compile checks set VC_TARGET / VC_LIDAR in base for one update and restore them; the FULL3D cloud test
%   rebuilds sens_uavScene for its world and restores it). Every measured number is printed.

    properties
        W = struct()     % parameter sets per world (shaddad_params(..., 'Assign', false))
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_Perception'',''var'')')
                shaddad_params();
            end
            for w = [1 3 4 6]
                tc.W.(sprintf('w%d', w)) = shaddad_params('World', w, 'Assign', false);
            end
            root = char(currentProject().RootFolder);
            files = {'models/Shaddad_Perception.slx', 'subsystems/sensors/sens_scan_filter.slx', ...
                     'subsystems/sensors/sens_acq_lidar.slx', 'tests/sensors/tb_perception.slx', ...
                     'tests/sensors/tb_per_scanfilter.slx', 'tests/sensors/tb_per_tebvfh.slx', ...
                     'tests/sensors/tb_per_tebvfh_top.slx'};
            for k = 1:numel(files)
                tc.assertTrue(isfile(fullfile(root, files{k})), ['missing ' files{k} ' (run build_perception / build_tb_perception)']);
            end
        end
    end

    methods (Static)
        function m(id, fmt, varargin)
            fprintf('[%s] %s\n', id, sprintf(fmt, varargin{:}));
        end
    end

    methods (Test)
        %% ---------------- first step: scan matching timing on the greenhouse aisle ----------------
        function tSM00_ScanMatchTiming(tc)
            % matchScansGrid with the P settings (D55) <= 40 ms per call interpreted; pose within 0.1 m / 1 deg
            P = tc.W.w4;  per = P.sensors.per;  L = P.sensors.lidar;
            rs = RandStream('mt19937ar', 'Seed', 21);
            K = 20;
            t = (0:K-1)'*0.1;
            yv = 6 + 26*rand(rs, K, 1);  xv = 4.8 + 0.6*(rand(rs, K, 1) - 0.5);  pv = pi/2 + 0.05*(rand(rs, K, 1) - 0.5);
            S = per_scan_gen(P, t, [xv yv pv], 'seed', 21);
            grid = sens_per_prior_map(P.world, per);
            c = sens_per_prm(P, 'sm');
            clear sens_scanmatch_loc
            e = zeros(K, 3);  tMatch = zeros(K, 1);  tCall = zeros(K, 1);  nPts = zeros(K, 1);  sAl = zeros(K, 1);  vOK = false(K, 1);
            mp = binaryOccupancyMap(grid, P.world.occ.res);  mp.GridLocationInWorld = P.world.occ.origin;
            rsn = rangeSensor('Range', [L.rMin per.smMaxRange], 'HorizontalAngle', [-pi pi - pi/180], ...
                'HorizontalAngleResolution', pi/180, 'RangeNoise', 0);
            for k = 1:K
                pTrue = [xv(k) yv(k) pv(k)];
                pPred = pTrue + [0.1*randn(rs, 1, 2) 0.01*randn(rs)];
                % matchScansGrid alone (interpreted), as in the D55 measurement
                th = pPred(3);  pLp = [pPred(1:2) + L.r(1)*[cos(th) sin(th)], th];
                [rr, aa] = rsn(pLp, mp);
                cur = lidarScan(double(S.ranges(k, :)'), double(S.angles(:)));  ref = lidarScan(rr, aa);
                t0 = tic;
                matchScansGrid(cur, ref, 'InitialPose', [0 0 0], 'Resolution', per.smRes, 'MaxRange', per.smMaxRange, ...
                    'TranslationSearchRange', per.smTrans, 'RotationSearchRange', per.smRot);
                tMatch(k) = toc(t0);
                % complete F11 function (rangeSensor + matchScansGrid + pose / covariance), interpreted
                rF = sens_scan_filter_ref(S.ranges(k, :)', S.angles(:), P);
                t0 = tic;
                sm = sens_scanmatch_loc(pPred, rF, S.angles(:), t(k), grid, c);
                tCall(k) = toc(t0);
                e(k, :) = [sm.pose(1:2)' - pTrue(1:2), angdiff(pTrue(3), sm.pose(3))];
                nPts(k) = double(sm.nPts);  sAl(k) = sqrt(sm.cov(2, 2));  vOK(k) = sm.valid;
            end
            tm = median(tMatch(2:end));  tf = median(tCall(2:end));
            cross = abs(e(:, 1));  along = abs(e(:, 2));  head = abs(e(:, 3))*180/pi;
            tc.m('T13-SM-TIMING', ['greenhouse aisle, %d scans, %.0f points/scan (%.0f within %g m): matchScansGrid %.1f ms ' ...
                'median (max %.1f; D55 29 ms), full sens_scanmatch_loc %.1f ms; |error| cross-aisle max %.3f m, heading max ' ...
                '%.2f deg, along-aisle median %.3f m / max %.3f m (reported along-aisle std %.2f m), within 0.1 m / 1 deg: ' ...
                '%d of %d (cross + heading %d of %d); valid %d of %d'], K, mean(sum(~isnan(S.ranges), 2)), mean(nPts), ...
                per.smMaxRange, 1e3*tm, 1e3*max(tMatch(2:end)), 1e3*tf, max(cross), max(head), median(along), max(along), ...
                median(sAl), nnz(hypot(e(:, 1), e(:, 2)) < 0.1 & head < 1), K, nnz(cross < 0.1 & head < 1), K, nnz(vOK), K);
            tc.verifyLessThanOrEqual(tm, 0.040, 'matchScansGrid <= 40 ms per call (interpreted)');
            tc.verifyLessThan(max(cross), 0.1, 'cross-aisle error < 0.1 m');
            tc.verifyLessThan(max(head), 1.0, 'heading error < 1 deg');
            % along the aisle the film walls carry no information (corridor degeneracy, spec 3.7): the error is
            % checked against the reported covariance instead of 0.1 m
            tc.verifyLessThan(max(along./sAl), 3, 'along-aisle error within 3 reported sigma');
        end

        function tSM01_AlongAisleHonesty(tc)
            % S4 nav fix round 2 (2026-11-14): on logged S4 greenhouse-aisle scans (closed-loop run, the filtered scan
            % each match used, tests/sensors/per_sm_aisle_scans.mat) the along-aisle sigma of sens_scanmatch_loc is
            % honest. From the start the closed loop used and from the true pose: the sigma reports the aisle's
            % information (not the 0.5-0.6 m sampling window of matchScansGrid's covariance), the per-scan NEES along
            % the aisle is at most 2 (the EKF's smInfl 2 and the correlation of consecutive matches take the rest),
            % at least 95 % of the errors lie within 3 sigma, and the cross-aisle error stays small. The capture of a
            % start 0.3 m behind / ahead of the truth along the aisle is printed (the start guard in sens_nav_gate
            % covers the second).
            P = tc.W.w4;
            root = char(currentProject().RootFolder);
            F = load(fullfile(root, 'tests', 'sensors', 'per_sm_aisle_scans.mat'));
            grid = sens_per_prior_map(P.world, P.sensors.per);
            c = sens_per_prm(P, 'sm');
            ang = single(P.sensors.lidar.angles(:));
            K = size(F.ranges, 1);
            clear sens_scanmatch_loc
            starts = {'logged start', F.start; 'true pose', F.pose; ...
                      '0.3 m behind', F.pose - 0.3*[cos(F.pose(:, 3)) sin(F.pose(:, 3)) zeros(K, 1)]; ...
                      '0.3 m ahead', F.pose + 0.3*[cos(F.pose(:, 3)) sin(F.pose(:, 3)) zeros(K, 1)]};
            res = cell(size(starts, 1), 1);
            for s = 1:size(starts, 1)
                E = nan(K, 5);                                     % [along err, along sigma, cross err, cross sigma, valid]
                for k = 1:K
                    sm = sens_scanmatch_loc(starts{s, 2}(k, :), double(F.ranges(k, :)'), ang, F.t(k), grid, c);
                    th = F.pose(k, 3);  u = [cos(th); sin(th)];  n = [-sin(th); cos(th)];
                    e = sm.pose(1:2) - F.pose(k, 1:2)';
                    Cxy = sm.cov(1:2, 1:2);
                    E(k, :) = [u'*e, sqrt(u'*Cxy*u), n'*e, sqrt(n'*Cxy*n), double(sm.valid)];
                end
                res{s} = E;
                v = E(:, 5) > 0.5;
                tc.m('T13-SM01', ['%s, %d logged aisle scans (y %.1f-%.1f m): valid %d; along-aisle error mean %+.3f rms ' ...
                    '%.3f m, sigma median %.3f m, NEES %.2f, within 3 sigma %.3f; cross-aisle rms %.3f m, sigma median %.3f m'], ...
                    starts{s, 1}, K, min(F.pose(:, 2)), max(F.pose(:, 2)), nnz(v), mean(E(v, 1)), rms(E(v, 1)), ...
                    median(E(v, 2)), mean(E(v, 1).^2./E(v, 2).^2), mean(abs(E(v, 1)) < 3*E(v, 2)), rms(E(v, 3)), median(E(v, 4)));
            end
            for s = 1:2
                E = res{s};  v = E(:, 5) > 0.5;
                tc.verifyGreaterThan(mean(v), 0.9, [starts{s, 1} ': valid > 90 %']);
                tc.verifyLessThan(median(E(v, 2)), 0.3, [starts{s, 1} ': the along-aisle sigma reports the aisle''s ' ...
                    'information (sampling window 0.5-0.6 m)']);
                tc.verifyLessThanOrEqual(mean(E(v, 1).^2./E(v, 2).^2), 2, [starts{s, 1} ': along-aisle NEES per scan <= 2']);
                tc.verifyGreaterThanOrEqual(mean(abs(E(v, 1)) < 3*E(v, 2)), 0.95, [starts{s, 1} ': >= 95 % within 3 sigma']);
                tc.verifyLessThan(rms(E(v, 3)), 0.06, [starts{s, 1} ': cross-aisle error rms < 0.06 m']);
            end
        end

        %% ---------------- interfaces, variants, loops ----------------
        function tInterfaces(tc)
            [n, rep] = shaddad_check_interfaces('Files', {'Shaddad_Perception'}, 'Quiet', true);
            tc.m('T13-IF', 'shaddad_check_interfaces Shaddad_Perception: %d mismatches %s', n, strjoin([rep.mismatch], '; '));
            tc.verifyEqual(n, 0, 'interface mismatches');
            mdl = 'Shaddad_Perception';
            wasLoaded = bdIsLoaded(mdl);
            if ~wasLoaded, load_system(mdl); end
            cl = onCleanup(@() local_close(mdl, wasLoaded));
            lp = Simulink.BlockDiagram.getAlgebraicLoops(mdl);
            tc.m('T13-IF', 'algebraic loops %d', numel(lp));
            tc.verifyEmpty(lp, 'algebraic loops');
            % compiled port rates
            evalc('feval(mdl, [], [], [], ''compile'')');
            ph = get_param([mdl '/perception'], 'PortHandles');  ts1 = get_param(ph.Inport, 'CompiledSampleTime');
            ph = get_param([mdl '/scanmatch'], 'PortHandles');   ts2 = get_param(ph.Inport, 'CompiledSampleTime');
            evalc('feval(mdl, [], [], [], ''term'')');
            tc.m('T13-IF', 'compiled sample times: perception %s, scanmatch %s', mat2str(ts1), mat2str(ts2));
            tc.verifyEqual(ts1(1), 0.1, 'AbsTol', 1e-12);  tc.verifyEqual(ts2(1), 0.2, 'AbsTol', 1e-12);
            % every variant choice compiles (update diagram activation)
            for v = {{'VC_TARGET', 1}, {'VC_LIDAR', 2}}
                nm = v{1}{1};  old = evalin('base', nm);
                assignin('base', nm, v{1}{2});
                rs = onCleanup(@() assignin('base', nm, old));
                ok = true;  msg = '';
                try
                    set_param(mdl, 'SimulationCommand', 'update');
                catch e
                    ok = false;  msg = e.message;
                end
                clear rs
                tc.m('T13-IF', 'update diagram with %s = %d: %s %s', nm, v{1}{2}, mat2str(ok), msg);
                tc.verifyTrue(ok, sprintf('compile with %s = %d', nm, v{1}{2}));
            end
            % MATLAB Function blocks: only where listed, at most one Subsystem Reference level below the model
            mf = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, ...
                'SFBlockType', 'MATLAB Function');
            nm = cellfun(@(p) get_param(p, 'Name'), mf, 'UniformOutput', false);
            tc.m('T13-IF', 'MATLAB Function blocks: %s', strjoin(unique(nm), ', '));
            tc.verifyEqual(sort(unique(nm(:)))', sort({'Cloud2Scan', 'OccMap', 'Obstacles', 'RowDetect', 'ScanMatchLoc'}));
            sf = find_system('sens_scan_filter', 'LookUnderMasks', 'all', 'SFBlockType', 'MATLAB Function');
            tc.verifyEmpty(sf, 'sens_scan_filter is built from blocks only');
        end

        %% ---------------- block-based scan filter == reference, bit for bit ----------------
        function tScanFilterBitExact(tc)
            P3 = tc.W.w3;  P4 = tc.W.w4;
            t = (0:49)'*0.1;
            S1 = per_scan_gen(P3, t, [16 + 0*t, 20 + t, pi/2 + 0*t], 'dust', 0.6, 'seed', 11);
            S2 = per_scan_gen(P4, t, [4.8 + 0*t, 5 + 0.5*t, pi/2 + 0*t], 'dust', 0.3, 'seed', 12);
            rg = [S1.ranges; S2.ranges];
            rg(3, 1:5) = single(0.6);  rg(4, 175:185) = single(0.2);      % self-box hits
            [Y, R] = per_sf_run(P3, rg, S1.angles);
            fn = fieldnames(R);
            nd = 0;
            for k = 1:numel(fn)
                a = Y.(fn{k});  b = R.(fn{k});
                tc.verifyEqual(size(a), size(b));
                tc.verifyEqual(class(a), class(b));
                if islogical(a)
                    d = nnz(a ~= b);
                else
                    bitsA = typecast(a(:), local_uint(a));  bitsB = typecast(b(:), local_uint(b));
                    bothNaN = isnan(a(:)) & isnan(b(:));
                    d = nnz(bitsA ~= bitsB & ~bothNaN);
                end
                nd = nd + d;
                tc.verifyEqual(d, 0, sprintf('%s differs from the reference', fn{k}));
            end
            tc.m('T13-SF', ['100 scans (50 palm dust 0.6, 50 greenhouse dust 0.3): %d differing elements over 6 outputs ' ...
                '(rangesF, rangesB, anglesB, xB, yB, keep); kept %d of %d beams, removed returns %d'], nd, nnz(R.keep), ...
                numel(R.keep), nnz(~isnan(rg) & ~R.keep));
            tc.verifyGreaterThan(nnz(~isnan(rg) & ~R.keep), 50, 'dust / self-box removals exercised');
        end

        %% ---------------- occupancy grid function == occupancyMap / insertRay ----------------
        function tOccMapReference(tc)
            P = local_wall_world(tc.W.w6);
            t = (0:250)'*0.1;  x = 5 + max(0, t - 5);
            S = per_scan_gen(P, t, [x, 0*t, 0*t], 'seed', 6);
            [pR, gR, oR] = sens_occmap_ref(S, P);
            clear sens_occmap_update
            c = sens_per_prm(P, 'occ');
            K = numel(t);  dP = zeros(K, 1);  agr = zeros(K, 1);  sh = zeros(K, 2);
            for k = 1:K
                rF = sens_scan_filter_ref(S.ranges(k, :)', S.angles(:), P);
                [mp, p] = sens_occmap_update(S.poseL(k, :), S.ranges(k, :)', rF, S.angles(:), true, c);
                sh(k, :) = round((mp.GridLocationInWorld - gR(k, :))*P.sensors.per.mapRes);
                cs = max(1, 1 - sh(k, 1)):min(300, 300 - sh(k, 1));  rs = max(1, 1 + sh(k, 2)):min(300, 300 + sh(k, 2));
                E = abs(p(rs, cs) - pR(rs - sh(k, 2), cs + sh(k, 1), k));
                A = mp.OccupancyMatrix(rs, cs);  B = oR(rs - sh(k, 2), cs + sh(k, 1), k);
                dP(k) = mean(E(:));  agr(k) = mean(A(:) == B(:));
            end
            tc.m('T13-OCC', ['own rolling log-odds grid vs occupancyMap / insertRay (MATLAB), wall scene 25 s: window ' ...
                'offset max %d cells, mean |dp| max %.5f, occupancy agreement min %.5f'], max(abs(sh(:))), max(dP), min(agr));
            tc.verifyEqual(max(abs(sh(:))), 0, 'same window placement as occupancyMap.move');
            tc.verifyLessThan(max(dP), 0.002, 'probabilities equal to the occupancyMap reference');
            tc.verifyGreaterThan(min(agr), 0.999, 'occupied / free classification equal to the reference');
        end

        %% ---------------- T-PER-01 obstacle 0.3 m box, open field, 1 m/s ----------------
        function tPER01_Obstacle(tc)
            P = tc.W.w6;
            P.world.obstacles.box(1, :) = [33 20.1 0 0.3 0.3 0.5 0 5 1];  P.world.obstacles.nBox = 1;
            P.world.occ = world_occupancy(P.world);
            t = (0:250)'*0.1;
            tn = (0:2500)'*0.01;
            nav = struct('t', tn, 'pos', [5 + tn, 20 + 0*tn, 0*tn], 'eul', zeros(numel(tn), 3), 'vel_B', repmat([1 0 0], numel(tn), 1));
            for dust = [0 0.5]
                S = per_scan_gen(P, t, [5 + t, 20 + 0*t, 0*t], 'dust', dust, 'seed', 3 + 10*dust);
                R = per_tb_run(P, per_tb_input(P, S, nav), 25);
                pr = R.per;  N = numel(pr.t);
                det = false(N, 1);  err = nan(N, 1);  fa = zeros(N, 1);  rng = zeros(N, 1);
                for k = 1:N
                    tru = [33 - (5 + pr.t(k)), 0.1];  rng(k) = hypot(tru(1), tru(2));
                    n = double(pr.obsCount(k));  op = double(squeeze(pr.obsPos_B(:, :, k)));
                    d = hypot(op(1:n, 1) - tru(1), op(1:n, 2) - tru(2));
                    if any(d < 0.5), det(k) = true;  err(k) = min(d); end
                    fa(k) = nnz(d >= 0.5);
                end
                in = rng <= 20 & rng >= 2;
                pd = mean(det(in));  ee = err(in & det);
                tc.m('T-PER-01', ['dust %.1f: detection %.4f over %d scans within 20 m (box range 2..20 m), position ' ...
                    'error rms %.3f m, 95th pct %.3f m, max %.3f m; false obstacles %.3f per scan'], dust, pd, nnz(in), ...
                    sqrt(mean(ee.^2)), prctile(ee, 95), max(ee), mean(fa));
                tc.verifyGreaterThan(pd, 0.95, 'detection probability within 20 m');
                tc.verifyLessThan(sqrt(mean(ee.^2)), 0.10, 'position error (rms) < 0.10 m');
                if dust > 0
                    tc.verifyLessThan(mean(fa), 0.1, 'false obstacles per scan at dust 0.5');
                end
            end
        end

        %% ---------------- T-PER-02 protective field: person enters at 1 m/s ----------------
        function tPER02_ProtectiveField(tc)
            P = tc.W.w6;
            T = 6;  v = 0.5;  rP = 0.25;
            t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            obs = @(tt) [8.6, 24.95 - tt, rP];                            % person crossing at 1 m/s
            S = per_scan_gen(P, t, [5 + v*t, 20 + 0*t, 0*t], 'obs', obs, 'seed', 13);
            nav = struct('t', tn, 'pos', [5 + v*tn, 20 + 0*tn, 0*tn], 'eul', zeros(numel(tn), 3), 'vel_B', repmat([v 0 0], numel(tn), 1));
            R = per_tb_run(P, per_tb_input(P, S, nav), T);
            per = P.sensors.per;
            Lf = per.protL0 + per.protKv*v;
            tf = (0:1e-4:T)';  xb = 8.6 - (5 + v*tf);  yb = 24.95 - tf - 20;
            qx = min(max(xb, per.protXF), per.protXF + Lf);  qy = min(max(yb, -per.protW/2), per.protW/2);
            tIn = tf(find(hypot(xb - qx, yb - qy) <= rP, 1));
            ks = find(R.per.protectiveStop, 1);  tStop = R.per.t(ks);
            early = any(R.per.protectiveStop(R.per.t < tIn - 1e-9));
            tc.m('T-PER-02', ['person r %.2f m crossing at 1 m/s, robot %.1f m/s (field %.2f x %.2f m): enters the field at ' ...
                '%.4f s, protectiveStop at %.2f s -> %.3f s; protFieldLen %.3f m; stop before entry %d'], rP, v, Lf, per.protW, ...
                tIn, tStop, tStop - tIn, R.per.protFieldLen(end), early);
            tc.verifyLessThanOrEqual(tStop - tIn, 0.15 + 1e-9, 'protectiveStop within one scan (<= 0.15 s)');
            tc.verifyFalse(early, 'no stop before the person enters');
            tc.verifyEqual(double(R.per.protFieldLen(end)), Lf, 'AbsTol', 1e-6);
            % reversing: rear field, a person behind the robot
            % reversing at 0.5 m/s towards a standing person behind the robot: rear field
            T2 = 3;  t2 = (0:T2*10)'*0.1;  tn2 = (0:T2*100)'*0.01;
            S2 = per_scan_gen(P, t2, [5 - v*t2, 20 + 0*t2, 0*t2], 'obs', @(tt) [2.0, 20.2, rP], 'seed', 14);
            nav2 = struct('t', tn2, 'pos', [5 - v*tn2, 20 + 0*tn2, 0*tn2], 'eul', zeros(numel(tn2), 3), ...
                'vel_B', repmat([-v 0 0], numel(tn2), 1));
            R2 = per_tb_run(P, per_tb_input(P, S2, nav2), T2);
            tIn2 = (per.protXR - Lf - (2.0 + rP - 5))/v;                      % person surface reaches the rear field end
            st2 = logical(R2.per.protectiveStop(:));  tt2 = R2.per.t(:);
            k2 = find(st2, 1);
            tc.m('T-PER-02', 'reversing: person enters the rear field at %.3f s, protectiveStop at %.2f s, stop before %d', ...
                tIn2, tt2(k2), any(st2(tt2 < tIn2 - 1e-9)));
            tc.verifyLessThanOrEqual(tt2(k2) - tIn2, 0.15 + 1e-9, 'rear field stop within one scan');
            tc.verifyFalse(any(st2(tt2 < tIn2 - 1e-9)), 'no rear stop before the person enters');
            tc.verifyTrue(all(st2(tt2 > tIn2 + 0.15 & tt2 < T2)), 'rear stop held while the person is in the field');
        end

        %% ---------------- R-03 / NAV-02: the protective field starts at the fitted implement ----------------
        function tPER02b_ImplementField(tc)
            % critfix R-03: the field limits come from the fitted implement (P.structure.impl with the setting
            % P.structure.cfg.implement = VC_IMPL); mower deck front 1.20 m: a point on the centre line 1.3 m ahead
            % stops the robot at standstill (it lay beyond the body-front field 0.55 .. 1.05 m); both fields are on
            % while |v| < protVBoth, so at v = -0.001 and +0.001 m/s a point 1.3 m ahead and one 0.8 m behind stop;
            % at 0.3 m/s only the field in the direction of travel is on
            P = shaddad_params('World', 6, 'Implement', 1, 'Assign', false);  per = P.sensors.per;
            Pt = shaddad_params('World', 6, 'Implement', 2, 'Assign', false);
            m = P.structure.impl.mower;  ti = Pt.structure.impl.tiller;  p0 = tc.W.w6.sensors.per;
            tc.m('R-03', ['protXF / protXR: none %.3f / %.3f m, mower %.3f / %.3f m, tiller %.3f / %.3f m; protVBoth %.2f m/s; ' ...
                'standstill front field with the mower %.2f .. %.2f m (body front %.2f .. %.2f m before)'], p0.protXF, p0.protXR, ...
                per.protXF, per.protXR, Pt.sensors.per.protXF, Pt.sensors.per.protXR, per.protVBoth, per.protXF, ...
                per.protXF + per.protL0, p0.selfBox(2), p0.selfBox(2) + per.protL0);
            tc.verifyEqual([p0.protXF p0.protXR], p0.selfBox([2 1]), 'AbsTol', 1e-12, 'no implement: the body');
            tc.verifyEqual([per.protXF per.protXR], [m.pos(1) + m.dims(1)/2, per.selfBox(1)], 'AbsTol', 1e-12, ...
                'mower: front field from the deck front, rear field from the body');
            tc.verifyEqual([Pt.sensors.per.protXF Pt.sensors.per.protXR], [per.selfBox(2), ti.pos(1) - ti.dims(1)/2], ...
                'AbsTol', 1e-12, 'tiller: rear field from the tiller rear');
            x0 = [5 20 0];  t = (0:10)'*0.1;  tn = (0:100)'*0.01;  r = 0.05;
            % {surface point on the centre line [m], v [m/s], stop expected}
            cases = {1.30, 0, true; 1.30, -0.001, true; 1.30, 0.001, true; -0.80, 0.001, true; -0.80, -0.001, true; ...
                     1.30, -0.3, false; -0.80, 0.3, false};
            for k = 1:size(cases, 1)
                xs = cases{k, 1};  v = cases{k, 2};
                S = per_scan_gen(P, t, repmat(x0, numel(t), 1), 'obs', @(tt) [x0(1) + xs + sign(xs)*r, x0(2), r], 'seed', 40 + k);
                nav = struct('t', tn, 'pos', repmat([x0(1:2) 0], numel(tn), 1), 'eul', zeros(numel(tn), 3), ...
                    'vel_B', repmat([v 0 0], numel(tn), 1));
                R = per_tb_run(P, per_tb_input(P, S, nav), 1);
                ps = logical(R.per.protectiveStop(:));
                L = per.protL0 + per.protKv*abs(v);
                tc.m('R-03', ['mower, point on the centre line at %+.2f m, v %+.3f m/s: protectiveStop %s (front field %.2f .. ' ...
                    '%.2f m, rear field %.2f .. %.2f m)'], xs, v, char('0' + ps'), per.protXF, per.protXF + L, ...
                    per.protXR - L, per.protXR);
                if cases{k, 3}
                    tc.verifyTrue(all(ps), sprintf('point at %+.2f m, v %+.3f m/s stops', xs, v));
                else
                    tc.verifyFalse(any(ps), sprintf('point at %+.2f m, v %+.3f m/s: that field is off', xs, v));
                end
            end
        end

        %% ---------------- R-02 / NAV-01: a lost or stale scan stops (fail safe) ----------------
        function tPER07_ScanLost(tc)
            % critfix R-02: NAV-01's case (a blank scan, all NaN and valid false, 2 s at 1 m/s) gives protectiveStop; an open
            % field (no return, valid) does not stop; a scan lost 1.0 .. 1.9 s stops once the last valid scan is older than
            % protNStale scans (two scan periods) and releases at the first valid scan; a single invalid scan does not
            % stop; without a valid scan at the start the field stops until the first one; a person in the field stays a
            % stop when the scan goes blank (the field result of the last valid scan is held, then the stale stop)
            P = shaddad_params('Assign', false);  per = P.sensors.per;
            ang = single(P.sensors.lidar.angles);
            nav = @(tn, v, x0) struct('t', tn, 'pos', repmat([x0(1:2) 0], numel(tn), 1), 'eul', repmat([x0(3) 0 0], numel(tn), 1), ...
                'vel_B', repmat([v 0 0], numel(tn), 1));
            t = (0:20)'*0.1;  tn = (0:200)'*0.01;
            S = struct('t', t, 'ranges', NaN(numel(t), 360, 'single'), 'angles', ang, 'valid', false(numel(t), 1));
            R = per_tb_run(P, per_tb_input(P, S, nav(tn, 1, [0 0 0])), 2);
            ps = logical(R.per.protectiveStop(:));
            tc.m('R-02', 'NAV-01 blank scan 2 s at 1 m/s: scanValid any %d; protectiveStop any %d, all %d (%s)', ...
                any(R.per.scanValid), any(ps), all(ps), char('0' + ps'));
            tc.verifyTrue(any(ps), 'NAV-01: a blank scan gives protectiveStop');
            tc.verifyTrue(all(ps), 'no valid scan from the start: protectiveStop at every scan');
            % open field; lost 1.0 .. 1.9 s; one invalid scan at 2.5 s
            t = (0:30)'*0.1;  tn = (0:300)'*0.01;  tLost = 1.0;
            v = true(numel(t), 1);  v(t > tLost - 0.05 & t < 1.95) = false;  v(abs(t - 2.5) < 1e-6) = false;
            S = struct('t', t, 'ranges', NaN(numel(t), 360, 'single'), 'angles', ang, 'valid', v);
            R = per_tb_run(P, per_tb_input(P, S, nav(tn, 1, [0 0 0])), 3);
            ps = logical(R.per.protectiveStop(:));  tt = R.per.t(:);
            kS = find(ps & tt > tLost - 1e-9, 1);  kR = find(~ps & tt > tt(kS), 1);
            tc.m('R-02', ['open field, scan lost %.1f .. 1.9 s, invalid at 2.5 s (protNStale %d): protectiveStop %s; stop at %.1f s ' ...
                '(last valid scan %.1f s old), released at %.1f s'], tLost, per.protNStale, char('0' + ps'), tt(kS), ...
                tt(kS) - (tLost - 0.1), tt(kR));
            tc.verifyFalse(any(ps(tt < tLost - 1e-9)), 'no stop on an open field with valid scans (none at start-up)');
            tc.verifyEqual(tt(kS) - (tLost - 0.1), (per.protNStale + 1)*0.1, 'AbsTol', 1e-6, ...
                'stop once the last valid scan is older than protNStale scans');
            tc.verifyTrue(all(ps(tt >= tt(kS) - 1e-9 & tt < 1.95)), 'stop held while the scan is lost');
            tc.verifyEqual(tt(kR), 2.0, 'AbsTol', 1e-6, 'released by the first valid scan');
            tc.verifyFalse(any(ps(tt > 1.95)), 'a single invalid scan does not stop');
            % start-up without a valid scan until 0.5 s
            S.valid = t > 0.45;
            R = per_tb_run(P, per_tb_input(P, S, nav(tn, 0, [0 0 0])), 3);
            ps = logical(R.per.protectiveStop(:));  tt = R.per.t(:);
            tc.m('R-02', 'start-up, first valid scan at 0.5 s: protectiveStop %s (start-up stop %.1f s)', char('0' + ps'), ...
                tt(find(~ps, 1)) - tt(1));
            tc.verifyTrue(all(ps(tt < 0.45)), 'stop from the start until the first valid scan');
            tc.verifyFalse(any(ps(tt > 0.45)), 'released by the first valid scan');
            % a person in the standstill field (surface 0.9 m ahead), the scan blank 1.0 .. 1.9 s
            P6 = tc.W.w6;  x0 = [5 20 0];
            S = per_scan_gen(P6, t, repmat(x0, numel(t), 1), 'obs', @(tt) [x0(1) + 1.1, x0(2), 0.2], 'seed', 31);
            lost = t > tLost - 0.05 & t < 1.95;
            S.ranges(lost, :) = NaN;  S.valid(lost) = false;
            R = per_tb_run(P6, per_tb_input(P6, S, nav(tn, 0, x0)), 3);
            ps = logical(R.per.protectiveStop(:));
            tc.m('R-02', 'person 0.9 m ahead at rest, scan blank %.1f .. 1.9 s: protectiveStop %s', tLost, char('0' + ps'));
            tc.verifyTrue(all(ps), 'the stop is held through the blank scans');
        end

        %% ---------------- T-PER-03 row detection on a palm aisle ----------------
        function tPER03_RowDetection(tc)
            P = tc.W.w3;
            T = 80;
            t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            pf = @(tt) [16 + 0.3*sin(2*pi*tt/20), 14 + 1.0*tt, pi/2 + 3*pi/180*sin(2*pi*tt/13)];
            S = per_scan_gen(P, t, pf(t), 'seed', 5);
            pn = pf(tn);
            nav = struct('t', tn, 'pos', [pn(:, 1:2) 0*tn], 'eul', [pn(:, 3) zeros(numel(tn), 2)], 'vel_B', repmat([1 0 0], numel(tn), 1));
            R = per_tb_run(P, per_tb_input(P, S, nav), T);
            row = R.per.row;  ps = pf(R.per.t);
            interior = ps(:, 2) > 18 & ps(:, 2) < 100;                      % trunks of the aisle ahead and behind
            v = logical(row.valid(:)) & interior;
            eL = double(row.lateralOffset(v)) - (-(ps(v, 1) - 16));          % heading north: left = smaller x
            eH = double(row.headingErr(v)) - (ps(v, 3) - pi/2);
            tc.m('T-PER-03', ['palm aisle x = 16 (trunk rows x = 12 / 20, 8 m spacing), 1 m/s, lateral +/-0.3 m, heading ' ...
                '+/-3 deg: valid %.4f of %d interior scans, lateral offset rms %.4f m, heading error rms %.3f deg, distances ' ...
                'L %.2f / R %.2f m, fitRms %.3f m (row window x [%g %g] m)'], mean(v(interior)), nnz(interior), sqrt(mean(eL.^2)), ...
                sqrt(mean(eH.^2))*180/pi, mean(row.distLeft(v)), mean(row.distRight(v)), mean(row.fitRms(v)), P.sensors.per.rowXWin);
            tc.verifyGreaterThan(mean(v(interior)), 0.90, 'valid > 90 % of interior scans');
            tc.verifyLessThan(sqrt(mean(eL.^2)), 0.10, 'lateral offset rms < 0.10 m');
            tc.verifyLessThan(sqrt(mean(eH.^2))*180/pi, 1.5, 'heading error rms < 1.5 deg');
            % open field: no row
            P6 = tc.W.w6;  t2 = (0:20)'*0.1;  tn2 = (0:200)'*0.01;
            S6 = per_scan_gen(P6, t2, [5 + 0*t2, 20 + 0*t2, 0*t2], 'seed', 7);
            nav6 = struct('t', tn2, 'pos', repmat([5 20 0], numel(tn2), 1), 'eul', zeros(numel(tn2), 3));
            R6 = per_tb_run(P6, per_tb_input(P6, S6, nav6), 2);
            tc.verifyFalse(any(R6.per.row.valid), 'no row in the open field');
        end

        %% ---------------- T-PER-04 local map: 5 s stationary 3 m from a wall, then 20 m ----------------
        function tPER04_LocalMap(tc)
            P = local_wall_world(tc.W.w6);
            P.world.occ = world_occupancy(P.world);
            t = (0:250)'*0.1;  x = 5 + max(0, t - 5);
            S = per_scan_gen(P, t, [x, 0*t, 0*t], 'seed', 6);
            % probabilities (function in MATLAB execution, same inputs as the block)
            clear sens_occmap_update
            c = sens_per_prm(P, 'occ');
            for k = 1:51
                rF = sens_scan_filter_ref(S.ranges(k, :)', S.angles(:), P);
                [mp, p] = sens_occmap_update(S.poseL(k, :), S.ranges(k, :)', rF, S.angles(:), true, c);
            end
            gl = mp.GridLocationInWorld;
            xc = gl(1) + ((1:300) - 0.5)/10;  yc = gl(2) + 30 - ((1:300)' - 0.5)/10;
            % wall section facing the LiDAR (3 m away): within +/-3 m along the wall (incidence <= 45 deg); FAST2D box
            % returns lie on the world cell grid, so the face y = 3.0 is a cell boundary and the face cell of a column
            % is the more probable of the two rows next to it
            pwc = @(span) max(p(yc > 2.9 & yc < 3.0, abs(xc - 5.55) <= span), p(yc > 3.0 & yc < 3.1, abs(xc - 5.55) <= span));
            pw = pwc(3);  pw4 = pwc(4);
            pf = p(yc > 0.5 & yc < 2.75, abs(xc - 5.55) <= 4);
            tc.m('T-PER-04', ['after 5 s stationary 3 m from the wall: wall cells p min %.3f (mean %.3f) over %d columns ' ...
                '(+/-3 m; +/-4 m: min %.3f), free cells p max %.3f (mean %.3f)'], min(pw), mean(pw), numel(pw), min(pw4), ...
                max(pf(:)), mean(pf(:)));
            tc.verifyGreaterThan(min(pw), 0.9, 'wall cells p > 0.9');
            tc.verifyLessThan(max(pf(:)), 0.3, 'free cells p < 0.3');
            % the model: rolling map after the 20 m drive vs the truth raster
            tn = (0:2500)'*0.01;  xn = 5 + max(0, tn - 5);
            nav = struct('t', tn, 'pos', [xn, 0*tn, 0*tn], 'eul', zeros(numel(tn), 3), 'vel_B', [double(tn > 5) zeros(numel(tn), 2)]);
            R = per_tb_run(P, per_tb_input(P, S, nav), 25, 'Map', true);
            mp = R.per.map;  k = numel(R.per.t);
            gl = reshape(mp.GridLocationInWorld(:, :, k), 1, 2);  A = mp.OccupancyMatrix(:, :, k);
            xc = gl(1) + ((1:300) - 0.5)/10;  yc = gl(2) + 30 - ((1:300)' - 0.5)/10;
            xL = 25 + P.sensors.lidar.r(1);
            win = repmat(xc >= xL - 15 & xc <= xL + 5, 300, 1);
            % truth raster: the visible face (P.world.occ cells of the wall nearest to the robot) in the window
            occT = P.world.occ;
            [XC, YC] = meshgrid(xc, yc);
            jc = floor((XC - occT.origin(1))*occT.res) + 1;  ic = size(occT.grid, 1) - floor((YC - occT.origin(2))*occT.res);
            inW = ic >= 1 & ic <= size(occT.grid, 1) & jc >= 1 & jc <= size(occT.grid, 2);
            T = false(size(XC));
            T(inW) = occT.grid(sub2ind(size(occT.grid), ic(inW), jc(inW)));
            Tface = T & ~[T(2:end, :); false(1, 300)];                    % wall cell whose lower-y neighbour is free
            B = Tface & win;  A = A & win;
            Bd = conv2(double(B), ones(3), 'same') > 0;  Ad = conv2(double(A), ones(3), 'same') > 0;
            found = nnz(B & Ad);  fp = nnz(A & ~Bd);
            iou = found/(nnz(B) + fp);
            tc.m('T-PER-04', ['after the 20 m drive (map window %s): truth face cells %d, found %d, map occupied %d, ' ...
                'false %d -> IoU (1-cell tolerance) %.3f; strict IoU %.3f'], mat2str(gl), nnz(B), found, nnz(A), fp, iou, ...
                nnz(A & B)/nnz(A | B));
            tc.verifyGreaterThan(iou, 0.8, 'no tearing after the rolling move (IoU > 0.8)');
            tc.verifyEqual(gl, [floor(xL*10) - 150, -150]/10, 'AbsTol', 1e-9, 'window centred on the LiDAR');
            tc.verifyEqual(double(reshape(mp.GridSize(:, :, k), 1, 2)), [300 300]);
            tc.verifyEqual(double(reshape(mp.GridOriginInLocal(:, :, k), 1, 2)), [-15 -15]);
        end

        %% ---------------- T-PER-05 TEB / VFH compatibility ----------------
        function tPER05_TebVfh(tc)
            P = tc.W.w1;
            T = 2;  t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            S = per_scan_gen(P, t, repmat([20 10 0], numel(t), 1), 'seed', 2);
            nav = struct('t', tn, 'pos', repmat([20 10 0], numel(tn), 1), 'eul', zeros(numel(tn), 3), 'vel_B', repmat([0.5 0 0], numel(tn), 1));
            ds = per_tb_input(P, S, nav);
            mdl = 'tb_per_tebvfh_top';
            if ~bdIsLoaded(mdl), load_system(mdl); end
            in = Simulink.SimulationInput(mdl);
            in = in.setExternalInput(ds);  in = in.setVariable('P', P);
            in = in.setModelParameter('StopTime', num2str(T));
            % Normal mode: Bus_Perception.map -> TEB, ranges_B / angles_B -> VFH with 0 errors
            inN = in.setBlockParameter([mdl '/TebVfh'], 'SimulationMode', 'Normal');
            out = sim(inN);
            tc.verifyEmpty(out.ErrorMessage);
            vc = out.yout{1}.Values.Data;  sd = out.yout{2}.Values.Data;
            tc.m('T-PER-05', ['Shaddad_Perception -> tb_per_tebvfh (Model block, Normal): 0 errors; TEB velcmds %s per ' ...
                'step, last VFH steering %.3f rad'], mat2str(size(vc, [1 2])), sd(end));
            % VFH and NaN ranges (controllerVFH): NaN treated like no obstacle?
            vfh = controllerVFH('UseLidarScan', false, 'DistanceLimits', [0.05 9]);
            a = -pi + (0:359)'*pi/180;  r = NaN(360, 1);  r(175:187) = 1.5;
            s1 = vfh(r, a, 0);  release(vfh);  r2 = r;  r2(isnan(r2)) = 9;  s2 = vfh(r2, a, 0);
            release(vfh);  s3 = vfh(NaN(360, 1), a, 0);
            tc.m('T-PER-05', 'VFH ignores NaN ranges: steering with NaN %.4f rad, NaN -> 9 m %.4f rad, all NaN %.4f rad', s1, s2, s3);
            tc.verifyEqual(s1, s2, 'VFH: NaN equivalent to a free 9 m range');
            % TEB time per call on a 10 x 10 m window at 5 cells/m (controllerTEB, interpreted)
            m = binaryOccupancyMap(10, 10, 5);  setOccupancy(m, [6 5; 6 5.2; 6 4.8], true);
            teb = controllerTEB([1 5 0; 3 5 0; 5 5 0; 7 5 0; 9 5 0], m);
            tt = zeros(1, 10);
            for k = 1:10, t0 = tic; teb([1 5 0], [0.5 0]); tt(k) = toc(t0); end
            tc.m('T-PER-05', 'TEB time per call, 10 x 10 m at 5 cells/m: first %.1f ms, median %.1f ms, max %.1f ms (calls 2-10)', ...
                1e3*tt(1), 1e3*median(tt(2:end)), 1e3*max(tt(2:end)));
            % the test model referenced as a Model block in Accelerator mode needs a C compiler
            cc = mex.getCompilerConfigurations('C', 'Selected');
            tc.m('T-PER-05', 'Accelerator-mode reference: C compiler %s', mat2str(~isempty(cc)));
            tc.assumeNotEmpty(cc, 'BLOCKED: no C compiler installed (Model block in Accelerator mode cannot be built)');
            outA = sim(in);
            tc.verifyEmpty(outA.ErrorMessage);
        end

        %% ---------------- scan matching in the greenhouse aisle (model) ----------------
        function tScanMatchGreenhouse(tc)
            P = tc.W.w4;
            T = 30;  t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            pf = @(tt) [4.8 + 0.2*sin(2*pi*tt/15), 4 + 0.6*tt, pi/2 + 0.03*sin(2*pi*tt/11)];
            S = per_scan_gen(P, t, pf(t), 'seed', 4);
            pn = pf(tn);
            rs = RandStream('mt19937ar', 'Seed', 9);
            e0 = [0.05*randn(rs, 1, 2) 0.005*randn(rs)];                     % prediction error of the nav pose
            nav = struct('t', tn, 'pos', [pn(:, 1:2) + e0(1:2), 0*tn], 'eul', [pn(:, 3) + e0(3) zeros(numel(tn), 2)], ...
                'vel_B', repmat([0.6 0 0], numel(tn), 1), 'lidarLocActive', true(numel(tn), 1));
            R = per_tb_run(P, per_tb_input(P, S, nav), T);
            sm = R.sm;  att = sm.t > 0;                                     % attempts (first sample = IC)
            v = logical(sm.valid(:)) & att;
            ps = squeeze(sm.pose);  if size(ps, 2) ~= 3, ps = ps'; end
            tr = pf(sm.t);  e = ps - tr;  e(:, 3) = angdiff(tr(:, 3), ps(:, 3));
            C = sm.cov;  vAl = squeeze(C(2, 2, v));  vCr = squeeze(C(1, 1, v));
            tc.m('T13-SM', ['greenhouse aisle 30 s at 0.6 m/s, lidarLocActive: valid %.4f of %d attempts; error rms cross ' ...
                '%.3f m, along %.3f m, heading %.2f deg; variance along %.4f m^2 > cross %.4f m^2 (mean); nPts %.0f'], ...
                mean(v(att)), nnz(att), sqrt(mean(e(v, 1).^2)), sqrt(mean(e(v, 2).^2)), sqrt(mean(e(v, 3).^2))*180/pi, ...
                mean(vAl), mean(vCr), mean(double(sm.nPts(v))));
            tc.verifyGreaterThan(mean(v(att)), 0.80, 'scan matching valid > 80 % of attempts');
            tc.verifyGreaterThan(mean(vAl), mean(vCr), 'along-aisle variance > cross-aisle variance');
            tc.verifyGreaterThan(mean(vAl > vCr), 0.9, 'along > cross in > 90 % of the valid matches');
            % perception.scanMatch holds the latest 0.2 s result at 0.1 s
            tc.verifyTrue(any(R.per.scanMatch.valid), 'perception.scanMatch carries the result');
            % disabled -> invalid
            nav.lidarLocActive(:) = false;
            R0 = per_tb_run(P, per_tb_input(P, S, nav), 2);
            tc.verifyFalse(any(R0.sm.valid), 'no scan matching while lidarLocActive is false');
        end

        %% ---------------- compiled timing ----------------
        function tCompiledTiming(tc)
            % marginal execution time per 0.1 s step (Normal mode: MATLAB Function blocks compiled by the JIT; no C
            % compiler for Accelerator / SIL), difference of a 25 s and a 5 s run (initialisation removed)
            cases = {'open field, all beams no return (worst case occupancy update)', tc.W.w3, 'free', false; ...
                     'palm aisle 1 m/s', tc.W.w3, 'palm', false; ...
                     'greenhouse aisle 0.6 m/s, scan matching off', tc.W.w4, 'gh', false; ...
                     'greenhouse aisle 0.6 m/s, scan matching on', tc.W.w4, 'gh', true};
            ms = zeros(size(cases, 1), 1);
            for c = 1:size(cases, 1)
                ex = zeros(1, 2);  Tv = [5 25];
                for j = 1:2
                    [ds, P] = local_timing_input(cases{c, 2}, cases{c, 3}, cases{c, 4}, Tv(j));
                    R = per_tb_run(P, ds, Tv(j));
                    ex(j) = R.exec;
                end
                ms(c) = 1e3*(ex(2) - ex(1))/200;
                tc.m('T13-TIME', '%s: %.2f ms per 0.1 s step (exec %.2f s / %.2f s for 5 / 25 s)', cases{c, 1}, ms(c), ex(1), ex(2));
            end
            smCall = 2*(ms(4) - ms(3));
            tc.m('T13-TIME', 'perception <= %.2f ms per 0.1 s step (budget 20); scan match %.1f ms per 0.2 s call (budget 40)', ...
                max(ms(1:3)), smCall);
            tc.verifyLessThanOrEqual(max(ms(1:3)), 20, 'perception <= 20 ms per 0.1 s step');
            tc.verifyLessThanOrEqual(smCall, 40, 'scan match <= 40 ms per 0.2 s');
        end

        %% ---------------- FULL3D: cloud -> 2-D scan (F4) ----------------
        function tFull3dCloud2Scan(tc)
            P = tc.W.w3;
            pose = [8 40 0 pi/2 0 0];
            pose(3) = interp2(P.world.terrain.yN, P.world.terrain.xE, P.world.terrain.H, pose(2), pose(1));
            bk = evalin('base', 'sens_uavScene');
            cl = onCleanup(@() assignin('base', 'sens_uavScene', bk));
            tbWas = bdIsLoaded('tb_sens_plant');                          % T11 bench: closed again if loaded here
            cl2 = onCleanup(@() local_close('tb_sens_plant', tbWas));
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(P));
            R3 = sens_tb_run(P, struct('T', 1.05, 'pose', @(t) repmat(pose, numel(t), 1)), 'VC_LIDAR', 2, 'cloudN', 10);
            xyz = R3.cloud.xyz(:, :, :, end);
            c = sens_per_prm(P, 'c2s');
            [r3, v3, n3] = sens_lidar_cloud2scan(xyz, P.sensors.lidar.full3d.nAz, [0 0], true, c);
            t = (0:19)'*0.1;
            S = per_scan_gen(P, t, repmat(pose([1 2 4]), numel(t), 1), 'seed', 8);
            r2 = median(S.ranges, 1, 'omitnan')';
            trunk = isfinite(r2) & isfinite(r3) & r2 < 30;
            d = abs(r2(trunk) - double(r3(trunk)));
            tc.m('T13-C2S', ['palm aisle FULL3D cloud (31 x 360) -> sens_lidar_cloud2scan: valid %d, band points %d, trunk ' ...
                'beams %d, median |FAST2D - cloud2scan| %.4f m'], v3, n3, nnz(trunk), median(d));
            tc.verifyTrue(v3, 'scan valid');
            tc.verifyGreaterThan(nnz(trunk), 5, 'trunk beams in both');
            tc.verifyLessThan(median(d), 0.05, 'median range difference on trunk beams');
            % 10 deg slope: ground in the 2-D scan with the body-frame band (cloudLevel 0) and levelled (1)
            Pf = tc.W.w1;  th = 10*pi/180;
            [~, Y] = ndgrid(Pf.world.terrain.xE, Pf.world.terrain.yN);
            Pf.world.terrain.H = tan(th)*Y;
            Pf.world.obstacles.nCyl = 0;  Pf.world.obstacles.nBox = 0;
            poseS = [0 0 0 pi/2 -th 0];
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(Pf));
            R4 = sens_tb_run(Pf, struct('T', 1.05, 'pose', @(t) repmat(poseS, numel(t), 1)), 'VC_LIDAR', 2, 'cloudN', 10);
            xyz4 = R4.cloud.xyz(:, :, :, end);
            cf = sens_per_prm(Pf, 'c2s');
            [r40, ~, n40] = sens_lidar_cloud2scan(xyz4, Pf.sensors.lidar.full3d.nAz, [0 -th], true, cf);
            cf(16) = 1;
            [r41, ~, n41] = sens_lidar_cloud2scan(xyz4, Pf.sensors.lidar.full3d.nAz, [0 -th], true, cf);
            tc.m('T13-C2S', ['10 deg slope, no obstacles: returns in the 2-D scan %d beams / %d band points with the ' ...
                'body-frame band (cloudLevel 0), %d beams / %d points levelled with roll / pitch (cloudLevel 1)'], ...
                nnz(isfinite(r40)), n40, nnz(isfinite(r41)), n41);
            tc.verifyLessThan(nnz(isfinite(r40))/360, 0.01, 'ground returns < 1 % of the beams (body-frame band)');
            % the model with VC_LIDAR = 2 runs on the cloud sequence
            nc = size(R3.cloud.xyz, 4);
            C = struct('t', (0:nc-1)'*0.1, 'xyz', R3.cloud.xyz, 'nAz', repmat(uint16(P.sensors.lidar.full3d.nAz), nc, 1), ...
                'valid', true(nc, 1));
            tn = (0:(nc-1)*10)'*0.01;
            nav = struct('t', tn, 'pos', repmat(pose(1:3), numel(tn), 1), 'eul', repmat(pose(4:6), numel(tn), 1));
            R = per_tb_run(P, per_tb_input(P, [], nav, C), (nc-1)*0.1, 'VC_LIDAR', 2);
            tc.m('T13-C2S', 'Shaddad_Perception with VC_LIDAR = 2: scanValid %d of %d, obstacles in the last scan %d', ...
                nnz(R.per.scanValid), numel(R.per.scanValid), R.per.obsCount(end));
            tc.verifyTrue(all(R.per.scanValid(2:end)), 'scans valid from the FULL3D cloud');
            tc.verifyGreaterThan(double(R.per.obsCount(end)), 2, 'trunks detected from the FULL3D cloud');
        end

        %% ---------------- sector minima and obstacle list against the scan ----------------
        function tSectorsAndObstacles(tc)
            P = tc.W.w1;
            T = 1;  t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            S = per_scan_gen(P, t, repmat([20 10 0.3], numel(t), 1), 'seed', 15);
            nav = struct('t', tn, 'pos', repmat([20 10 0], numel(tn), 1), 'eul', repmat([0.3 0 0], numel(tn), 1));
            R = per_tb_run(P, per_tb_input(P, S, nav), T);
            k = numel(R.per.t);
            rB = double(R.per.ranges_B(k, :))';
            [~, rBref] = sens_scan_filter_ref(S.ranges(k, :)', S.angles(:), P);
            idx = reshape(P.sensors.per.secIdx, P.sensors.per.secLen, P.sensors.per.nSectors);
            ref = min(reshape(double(rBref(idx)), size(idx)), [], 1, 'omitnan');
            got = double(R.per.minSectorRange(k, :));
            tc.m('T13-SEC', 'sector minima %s (reference %s)', mat2str(got, 4), mat2str(ref, 4));
            tc.verifyEqual(isnan(got), isnan(ref), 'empty sectors = NaN');
            tc.verifyEqual(got(~isnan(got)), ref(~isnan(ref)), 'AbsTol', 1e-6, 'sector minima');
            tc.verifyTrue(isequaln(rB, double(rBref)), 'ranges_B = scan filter output');
            % obstacles: cylinder (25, 10) r 0.35, box (40, 40) 0.6 m; rotate to base_link (yaw 0.3)
            Rz = [cos(0.3) sin(0.3); -sin(0.3) cos(0.3)];
            tru = (Rz*([25 10; 40 40] - [20 10])')';
            n = double(R.per.obsCount(k));  op = double(squeeze(R.per.obsPos_B(:, :, k)));
            d1 = min(hypot(op(1:n, 1) - tru(1, 1), op(1:n, 2) - tru(1, 2)));
            d2 = min(hypot(op(1:n, 1) - tru(2, 1), op(1:n, 2) - tru(2, 2)));
            rad = double(R.per.obsRadius(k, 1:n));
            tc.m('T13-OBS', 'obsCount %d; cylinder centre error %.3f m (radius %.3f, true 0.35), box centre error %.3f m; ranges sorted %d', ...
                n, d1, rad(1), d2, issorted(double(R.per.obsRange(k, 1:n))));
            tc.verifyLessThan(d1, 0.10);  tc.verifyLessThan(d2, 0.25);
            tc.verifyTrue(issorted(double(R.per.obsRange(k, 1:n))), 'obstacles sorted by range');
            tc.verifyEqual(abs(rad(1) - 0.35) < 0.1, true, 'cylinder radius');
        end

        %% ---------------- obstacle table: more clusters than the table holds ----------------
        function tObstacleTableFull(tc)
            % 90 separate 2-beam clusters in angular order with scattered ranges: the 20 nearest must be reported
            P = tc.W.w1;
            r = NaN(360, 1, 'single');
            rr = 3 + mod((1:90)'*7.3, 30);
            for i = 1:90, r(4*i - 3:4*i - 2) = single(rr(i)); end
            ang = single(P.sensors.lidar.angles(:));
            [rF, rB, ~, xB, yB] = sens_scan_filter_ref(r, ang, P);
            [nO, ~, ~, oR, ~, nCl] = sens_obstacle_extract(rF, ang, xB, yB, sens_per_prm(P, 'obs'));
            cm = zeros(90, 1);
            for i = 1:90, cm(i) = min(double(rB(4*i - 3:4*i - 2))); end
            cm = sort(cm);
            tc.m('T13-OBS', '90 clusters: table %d, obsCount %d, 20th range %.3f (expected %.3f)', nCl, nO, oR(20), cm(20));
            tc.verifyEqual(double(nO), 20);
            tc.verifyEqual(double(oR(:)), cm(1:20), 'AbsTol', 1e-4, 'the 20 nearest clusters, sorted');
        end

        %% ---------------- code generation ----------------
        function tCodegen(tc)
            hasCoder = any(strcmp({ver().Name}, 'Simulink Coder'));
            tc.m('T13-CG', 'Simulink Coder installed: %d', hasCoder);
            tc.assumeTrue(hasCoder, 'BLOCKED: Simulink Coder not installed');
            shaddad_configsets('codegen');
            mdl = 'Shaddad_Perception';
            load_system(mdl);
            cs = evalin('base', 'cs_Companion');
            ref = getConfigSet(mdl, 'Shaddad_Ref');  old = ref.SourceName;
            ref.SourceName = cs.Name;  c = onCleanup(@() set(ref, 'SourceName', old));
            slbuild(mdl);
        end
    end
end

% ============================================================================================
function local_close(mdl, wasLoaded)
if ~wasLoaded && bdIsLoaded(mdl), close_system(mdl, 0); end
end

function u = local_uint(a)
if isa(a, 'single'), u = 'uint32'; else, u = 'uint64'; end
end

function P = local_wall_world(P)
% open field (soil_strips, no obstacles) with a 50 m wall along x, face at y = 3.0 (box y 3.0 .. 3.2)
P.world.obstacles.box(1, :) = [20 3.1 0 50 0.2 1.0 0 5 1];
P.world.obstacles.nBox = 1;
end

function [ds, P] = local_timing_input(P, kind, smOn, T)
t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
switch kind
    case 'free'
        S = struct('t', t, 'ranges', NaN(numel(t), 360, 'single'), 'angles', single(P.sensors.lidar.angles), ...
                   'valid', true(numel(t), 1));
        pn = [16 + 0*tn, 20 + tn, pi/2 + 0*tn];
    case 'palm'
        S = per_scan_gen(P, t, [16 + 0*t, 20 + t, pi/2 + 0*t], 'seed', 3);
        pn = [16 + 0*tn, 20 + tn, pi/2 + 0*tn];
    otherwise
        pf = @(tt) [4.8 + 0.2*sin(2*pi*tt/15), 4 + 0.6*tt, pi/2 + 0.03*sin(2*pi*tt/11)];
        S = per_scan_gen(P, t, pf(t), 'seed', 4);
        pn = pf(tn) + [0.05 -0.04 0.005];
end
nav = struct('t', tn, 'pos', [pn(:, 1:2) 0*tn], 'eul', [pn(:, 3) zeros(numel(tn), 2)], ...
    'vel_B', repmat([1 0 0], numel(tn), 1), 'lidarLocActive', smOn & true(numel(tn), 1));
ds = per_tb_input(P, S, nav);
end
