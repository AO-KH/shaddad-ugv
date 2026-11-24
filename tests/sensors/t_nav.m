classdef t_nav < matlab.unittest.TestCase
%T_NAV  Acceptance tests of build task T12 (navigation fusion, models/Shaddad_NavFusion.slx).
%
%   runtests('tests/sensors/t_nav.m')                                     all tests (about 20-25 min)
%   runtests('tests/sensors/t_nav.m', 'ProcedureName', 'tNAV01_StaticAlignment')
%   runtests('tests/sensors/t_nav.m', 'Tag', 'fast')                      unit / compile tests only
%
%   Bench: tests/sensors/tb_nav.slx (scripts/build/build_tb_nav.m): Differential Drive Kinematic Model truth
%   (+ slip, side slip, slope) -> Shaddad_SensorModels (T11) -> sens_acq_imu + sens_odom_compute (T11) ->
%   Shaddad_NavFusion; scan matching is emulated from the truth pose (Shaddad_Perception is T13). Inputs from
%   tests/sensors/nav_tb_input.m, runs by nav_tb_run.m, errors by nav_tb_metrics.m, paths by nav_tb_path.m.
%   P is created with shaddad_params(..., 'Assign', false) and passed with SimulationInput.setVariable (the base
%   workspace is not modified). Bench conventions: RTK receiver warm start (P.sensors.gnss.initFix = 4) unless
%   the scenario says otherwise, IMU at its 25 degC calibration temperature, GNSS velocity noise added on the
%   bench side (nav_tb_run 'GpsVelNoise' 'bench', see there). Every measured number is printed as
%   'MEASURED <id>: ...'. Criteria: docs/build_plan.md T12 and docs/specs/sensors.md section 9 (T-NAV-01..11).

    properties
        P1      % flat_test world
        P3      % palm_farm
        P4      % greenhouse
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', ['exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_NavState'',''var'') && ' ...
                    'exist(''cs_SimFixed'',''var'') && exist(''cs_Shaddad'',''var'') && exist(''VC_TARGET'',''var'')'])
                shaddad_params();  shaddad_configsets('fixed');
            end
            if ~evalin('base', 'isfield(P.sensors, ''navgate'') && isfield(P.sensors.navgate, ''kRpos'') && isfield(P.sensors.nav, ''tauMaxSm'')')
                % base P older than param_sensors.m (another task assigned it): refresh with the same options
                w = evalin('base', 'P.scen.world_id');
                shaddad_params('World', w, 'Phase', evalin('base', 'VC_PHASE'), 'Target', evalin('base', 'VC_TARGET'), ...
                    'Implement', evalin('base', 'VC_IMPL'));
            end
            tc.P1 = t_nav.warm(shaddad_params('Assign', false));
            tc.P3 = t_nav.warm(shaddad_params('World', 3, 'Assign', false));
            tc.P4 = shaddad_params('World', 4, 'Assign', false);
            root = char(currentProject().RootFolder);
            if ~isfile(fullfile(root, 'tests', 'sensors', 'tb_nav.slx'))
                build_tb_nav();
            end
            if ~bdIsLoaded('tb_nav'), load_system('tb_nav'); end
        end
    end

    methods (TestClassTeardown)
        function teardown(~)
            if bdIsLoaded('tb_nav'), close_system('tb_nav', 0); end
        end
    end

    methods (Access = private)
        function m(tc, id, txt, varargin)
            s = sprintf(['MEASURED %s: ' txt], id, varargin{:});
            fprintf('%s\n', s);
            tc.log(1, s);
        end
    end

    methods (Static, Access = private)
        function P = warm(P)
            P.sensors.gnss.initFix = 4;      % RTK receiver already fixed (warm start)
        end
        function b = base(P, s, chi)
            % prof base row [v r vy sL sR chi gx gy]
            if nargin < 3, chi = P.structure.track.chi_nom; end
            b = [0 0 0 s s chi 0 0];
        end
        function a = wrap(a)
            a = atan2(sin(a), cos(a));
        end
        function k = idx(t, tq)
            [~, k] = min(abs(t - tq));
        end
    end

    % =========================================================================================
    %  structure, compile, interfaces (fast)
    % =========================================================================================
    methods (Test, TestTags = {'fast'})
        function tFilesBuilt(tc)
            root = char(currentProject().RootFolder);
            f = {fullfile(root, 'models', 'Shaddad_NavFusion.slx'), fullfile(root, 'subsystems', 'sensors', 'sens_acq_gnss.slx'), ...
                 fullfile(root, 'tests', 'sensors', 'tb_nav.slx')};
            for k = 1:numel(f)
                tc.verifyTrue(isfile(f{k}), f{k});
            end
            info = Simulink.MDLInfo(f{1});
            tc.verifyFalse(contains(info.Description, 'SHADDAD_STUB'), 'stub Description cleared');
            fn = {'sens_nav_gate', 'sens_nav_mode', 'sens_ekf_f', 'sens_ekf_dfdx', 'sens_ekf_h_gnsspos', 'sens_ekf_h_gnssvel', ...
                  'sens_ekf_h_heading', 'sens_ekf_h_odom', 'sens_ekf_h_lidarpose', 'sens_ekf_dh_gnsspos', 'sens_ekf_dh_gnssvel', ...
                  'sens_ekf_dh_heading', 'sens_ekf_dh_odom', 'sens_ekf_dh_lidarpose', 'sens_nav_ekf'};
            for k = 1:numel(fn)
                tc.verifyEqual(exist(fn{k}, 'file'), 2, fn{k});
            end
        end

        function tInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'Shaddad_NavFusion'}, 'Quiet', true);
            tc.m('IFACE', 'shaddad_check_interfaces(Shaddad_NavFusion) mismatches %d', n);
            tc.verifyEqual(n, 0);
        end

        function tCompileMultitasking(tc)
            % compile with EnableMultiTasking 'on' and the default MultiTaskDSMMsg ('error'); NavState rate 0.01 s
            mdl = 'Shaddad_NavFusion';
            wasLoaded = bdIsLoaded(mdl);
            if ~wasLoaded, load_system(mdl); end
            c = onCleanup(@() local_close(mdl, wasLoaded));
            cs = getActiveConfigSet(mdl);
            mt = get_param(cs, 'EnableMultiTasking');  dsm = get_param(cs, 'MultiTaskDSMMsg');
            tc.m('COMPILE', 'config %s: EnableMultiTasking %s, MultiTaskDSMMsg %s', get_param(cs, 'Name'), mt, dsm);
            tc.verifyEqual(mt, 'on');
            tc.verifyEqual(dsm, 'error');
            try
                evalc('set_param(mdl, ''SimulationCommand'', ''update'')');
                evalc('feval(mdl, [], [], [], ''compile'')');
            catch e
                tc.verifyFail(e.message);
                return
            end
            try
                ph = get_param([mdl '/nav_state'], 'PortHandles');
                ts = get_param(ph.Inport(1), 'CompiledSampleTime');
                ph2 = get_param([mdl '/nav_diag'], 'PortHandles');
                ts2 = get_param(ph2.Inport(1), 'CompiledSampleTime');
            catch e
                ts = NaN;  ts2 = NaN;
            end
            evalc('feval(mdl, [], [], [], ''term'')');
            tc.m('COMPILE', 'nav_state compiled sample time %s, nav_diag %s', mat2str(ts), mat2str(ts2));
            tc.verifyEqual(ts(1), 0.01, 'AbsTol', 1e-12);
            tc.verifyEqual(ts2(1), 0.01, 'AbsTol', 1e-12);
        end

        function tAlgebraicLoops(tc)
            for mdl = {'Shaddad_NavFusion', 'tb_nav'}
                wasLoaded = bdIsLoaded(mdl{1});
                if ~wasLoaded, load_system(mdl{1}); end
                evalc('loops = Simulink.BlockDiagram.getAlgebraicLoops(mdl{1});');
                if ~wasLoaded, close_system(mdl{1}, 0); end
                tc.m('ALGLOOPS', '%s algebraic loops %d', mdl{1}, numel(loops));
                tc.verifyEmpty(loops, mdl{1});
            end
        end

        function tBlocks(tc)
            % library blocks required by the build plan are present in the model
            mdl = 'Shaddad_NavFusion';
            wasLoaded = bdIsLoaded(mdl);
            if ~wasLoaded, load_system(mdl); end
            c = onCleanup(@() local_close(mdl, wasLoaded));
            refs = {'mspfiltlib/IMU Filter', 'mspfiltlib/ecompass', 'robotcorelib/Coordinate Transformation Conversion'};
            for k = 1:numel(refs)
                b = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'ReferenceBlock', refs{k});
                tc.verifyNotEmpty(b, refs{k});
            end
            % aerolibtransform2 LLA <-> flat Earth blocks (copied unlinked from the library)
            tc.verifyEqual(get_param([mdl '/NavMeas/LLA2Flat'], 'LL0'), 'P.sensors.refLLA(1:2)');
            tc.verifyEqual(get_param([mdl '/NavOutput/Flat2LLA'], 'LL0'), 'P.sensors.refLLA(1:2)');
            tc.verifyEqual(get_param([mdl '/AttitudeFilter/NegAcc'], 'Gain'), '-1', 'Gain -1 before IMU Filter / ecompass');
            rs = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem', 'Name', 'NavEKF');
            rp = find_system(rs{1}, 'SearchDepth', 1, 'BlockType', 'ResetPort');
            tc.verifyEqual(get_param(rp{1}, 'ResetTriggerType'), 'rising', 'EKF inside a Resettable Subsystem');
            ms = find_system(rs{1}, 'SearchDepth', 1, 'BlockType', 'MATLABSystem');
            tc.verifyEqual(get_param(ms{1}, 'System'), 'sens_nav_ekf');
            vs = [mdl '/IO_In'];
            tc.verifyEqual(get_param(vs, 'Variant'), 'on');
            tc.verifyEqual(get_param(vs, 'VariantActivationTime'), 'update diagram');
            tc.verifyEqual(get_param([vs '/SIM'], 'VariantControl'), 'VC_TARGET == 0');
            tc.verifyEqual(get_param([vs '/HW'], 'VariantControl'), 'VC_TARGET == 1');
            mf = find_system(mdl, 'SearchDepth', 1, 'SFBlockType', 'MATLAB Function');
            tc.m('BLOCKS', 'MATLAB Function blocks: %s', strjoin(get_param(mf, 'Name'), ', '));
            tc.verifyEqual(sort(get_param(mf, 'Name')), {'NavGate'; 'NavModeManager'});
        end

        function tJacobians(tc)
            x = [3; -2; 0.5; 0.7; 1.1; 0.05; 0.01; 0.02; 0.04; 0.07; 1.35];
            u = tc.P1.sensors.ekf.u0;  u(1:4) = [0.3; 0.2; sin(0.1); cos(0.1)];
            u1 = [-0.4; 0.02; 1.05; 0.05; 0.1; 0.18; 0.3];
            f = {@(x) sens_ekf_f(x, u), @(x) sens_ekf_dfdx(x, u); ...
                 @(x) sens_ekf_h_gnsspos(x, u1), @(x) sens_ekf_dh_gnsspos(x, u1); ...
                 @(x) sens_ekf_h_gnssvel(x, u1), @(x) sens_ekf_dh_gnssvel(x, u1); ...
                 @(x) sens_ekf_h_heading(x, [0.18; 0.3]), @(x) sens_ekf_dh_heading(x, [0.18; 0.3]); ...
                 @(x) sens_ekf_h_odom(x, [0.3; 0.85]), @(x) sens_ekf_dh_odom(x, [0.3; 0.85]); ...
                 @(x) sens_ekf_h_lidarpose(x, [0.25; 0.3]), @(x) sens_ekf_dh_lidarpose(x, [0.25; 0.3])};
            for k = 1:size(f, 1)
                H = f{k, 2}(x);  Hn = zeros(size(H));
                for j = 1:11
                    e = zeros(11, 1);  e(j) = 1e-6;
                    Hn(:, j) = (f{k, 1}(x + e) - f{k, 1}(x - e))/2e-6;
                end
                tc.verifyLessThan(max(abs(H(:) - Hn(:))), 1e-6, sprintf('Jacobian %d', k));
            end
            % tau = 0 reduces the measurement functions to the spec models
            tc.verifyEqual(sens_ekf_h_lidarpose(x, [0; 0.3]), x([1 2 4]), 'AbsTol', 1e-15);
            tc.verifyEqual(sens_ekf_h_heading(x, [0; 0.3]), x(4), 'AbsTol', 1e-15);
            % clamps of the slip / gauge states come from P (u(13:16))
            xc = x;  xc(9) = 0.9;  xc(11) = 3;
            xn = sens_ekf_f(xc, u);
            tc.verifyLessThanOrEqual(xn(9), tc.P1.sensors.ekf.slip_clamp(2));
            tc.verifyLessThanOrEqual(xn(11), tc.P1.sensors.ekf.chi_clamp(2));
        end

        function tEkfSystemObjectReset(tc)
            % R6 fallback: the System object restores x0 / P0 on reset (the Resettable Subsystem calls resetImpl)
            E = tc.P1.sensors.ekf;
            o = sens_nav_ekf('InitialState', E.x0, 'InitialStateCovariance', E.P0);
            z7 = zeros(7, 1);
            args = @(en5) {E.u0, E.Qd, zeros(3,1), false, eye(3), z7, zeros(2,1), false, eye(2), z7, 0, false, 1, [0;0], ...
                           [0;0], false, eye(2), [0; 0.85], [1; 2; 0.3], en5, 0.01*eye(3), [0; 0], false};
            a = args(true);
            [x1, Pa] = o(a{:});
            for k = 1:20, a = args(mod(k, 5) == 0); o(a{:}); end
            reset(o);
            a = args(true);
            [x2, P2] = o(a{:});
            tc.m('RESET', 'System object reset: max |x - x(first step)| %.3g, max |P - P(first step)| %.3g', ...
                 max(abs(x2 - x1)), max(abs(P2(:) - Pa(:))));
            tc.verifyEqual(x2, x1, 'AbsTol', 1e-12);
            tc.verifyEqual(P2, Pa, 'AbsTol', 1e-12);
            release(o);
        end

        function tGateUnit(tc)
            % chi-square rejection of an outlier, forced acceptance after Nmax rejections, re-fix bypass
            G = tc.P1.sensors.navgate;  E = tc.P1.sensors.ekf;
            clear sens_nav_gate
            c = onCleanup(@() clear('sens_nav_gate'));
            x = E.x0;  x(1:2) = [10; 5];  Pm = diag([1e-4 1e-4 1e-3 1e-5 1e-3 1e-3 1e-8 1e-4 1e-4 1e-4 1e-3]);
            u1 = [tc.P1.sensors.gnss.rAnt1(:); 0; 0; 0; 0];
            l = sens_ekf_h_gnsspos(x, u1) - x(1:3);
            z = x(1:3) + l;
            call = @(z1, fix, av) sens_nav_gate(x, Pm, E.u0, 0*E.Qd, false, z1, [0;0], 0, [0;0], [0;0;0], av, ...
                [0.015 0.03 0.02], 0.01, eye(3), u1, u1, [0;0], [0;0.85], [0;0], uint8(fix), G);
            av = [true; false; false; false; false];
            for k = 1:G.nBypass, call(z, 4, av); end      % first fix after start: bypass epochs
            en = call(z, 4, av);  tc.verifyTrue(en(1), 'consistent RTK epoch accepted');
            en = call(z + [3; 0; 0], 4, av);  tc.verifyFalse(en(1), '3 m outlier rejected');
            for k = 1:G.Nmax(1) - 1, call(z + [3; 0; 0], 4, av); end
            [en, R1] = call(z + [3; 0; 0], 4, av);
            tc.verifyTrue(en(1), 'forced acceptance after Nmax consecutive rejections');
            tc.verifyEqual(R1(1, 1), (G.kRpos(3)*0.015)^2*G.kForce(1), 'RelTol', 1e-12, 'forced R x kForce');
            % FLOAT epochs then a jump back to FIXED: bypass of the gate for nBypass epochs
            for k = 1:3, call(z + [0.5; 0; 0], 5, av); end
            en = call(z + [1.0; 0; 0], 4, av);
            tc.verifyTrue(en(1), 'first RTK FIXED epoch after FLOAT bypasses the gate');
            [~, R1] = call(z, 5, av);
            tc.verifyEqual(R1(1, 1), (G.kRpos(2)*0.015)^2, 'RelTol', 1e-12, 'FLOAT position R inflated by kRpos');
        end

        function tGateVelAndSmGuard(tc)
            % S4 nav fix round 2: GNSS velocity R inflated by kRvel without RTK (the gate keeps kR); start guard on
            % the weak axis of a scan match (an aisle along N): a match that did not move from an uncertain start
            % gets no more weight than the prior along that axis, a moved match or a confident start keeps its R
            G = tc.P1.sensors.navgate;  E = tc.P1.sensors.ekf;
            clear sens_nav_gate
            c = onCleanup(@() clear('sens_nav_gate'));
            x = E.x0;  x(1:2) = [4.8; 10];  x(4) = pi/2;  x(5) = 0.5;
            u2 = [tc.P1.sensors.gnss.rAnt1(:); 0; 0; 0; 0];  u5 = [0; 0];
            zv = sens_ekf_h_gnssvel(x, u2);  zp = sens_ekf_h_lidarpose(x, u5);
            C5 = diag([0.04^2 0.15^2 (0.5*pi/180)^2]);            % aisle along N: weak axis N, 0.15 m
            call = @(Pm, z5, fix, av) sens_nav_gate(x, Pm, E.u0, 0*E.Qd, false, [0; 0; 0], zv, 0, [0; 0], z5, av, ...
                [1.2 2.5 0.05], 0.01, C5, u2, u2, [0; 0], [0; 0.85], u5, uint8(fix), G);
            % GNSS velocity: R2 = (kRvel(fix) sAcc)^2, FIXED unchanged
            av = [false; true; false; false; false];
            Pm = diag([1e-2 1e-2 1e-3 1e-4 1e-3 1e-3 1e-8 1e-4 1e-4 1e-4 1e-3]);
            [en, ~, R2] = call(Pm, zp, 1, av);
            tc.verifyTrue(en(2), 'consistent SINGLE velocity accepted');
            tc.verifyEqual(R2(1, 1), (G.kRvel(1)*0.05)^2, 'RelTol', 1e-12, 'SINGLE velocity R inflated by kRvel(1)');
            [~, ~, R2] = call(Pm, zp, 5, av);
            tc.verifyEqual(R2(1, 1), (G.kRvel(2)*0.05)^2, 'RelTol', 1e-12, 'FLOAT velocity R inflated by kRvel(2)');
            [~, ~, R2] = call(Pm, zp, 4, av);
            tc.verifyEqual(R2(1, 1), (G.kR*0.05)^2, 'RelTol', 1e-12, 'FIXED velocity R = kR sAcc (unchanged)');
            % scan match, weak axis N
            av = [false; false; false; false; true];
            R5n = C5(2, 2)*G.smInfl;                              % weak-axis R5 without the guard
            Pu = Pm;  Pu(2, 2) = 0.30^2;                          % start uncertain along N (0.30 m > smCap)
            [en, ~, ~, ~, ~, R5] = call(Pu, zp + [0; 0.02; 0], 1, av);
            tc.verifyTrue(en(5), 'match accepted');
            tc.verifyEqual(R5(2, 2), Pu(2, 2), 'RelTol', 1e-4, 'uncertain start, no move: weak-axis R5 = prior variance');
            tc.verifyEqual(R5(1, 1), C5(1, 1)*G.smInfl, 'RelTol', 1e-9, 'cross-aisle R5 unchanged');
            [~, ~, ~, ~, ~, R5] = call(Pu, zp + [0; 0.25; 0], 1, av);
            tc.verifyEqual(R5(2, 2), R5n, 'RelTol', 1e-9, 'uncertain start, the match moved: R5 unchanged');
            [~, ~, ~, ~, ~, R5] = call(Pm, zp + [0; 0.02; 0], 1, av);
            tc.verifyEqual(R5(2, 2), R5n, 'RelTol', 1e-9, 'confident start: R5 unchanged');
            C5i = C5;  C5i(2, 2) = C5(1, 1);                      % isotropic match: no weak axis
            calli = @(Pm, z5) sens_nav_gate(x, Pm, E.u0, 0*E.Qd, false, [0; 0; 0], zv, 0, [0; 0], z5, av, ...
                [1.2 2.5 0.05], 0.01, C5i, u2, u2, [0; 0], [0; 0.85], u5, uint8(1), G);
            [~, ~, ~, ~, ~, R5] = calli(Pu, zp + [0; 0.02; 0]);
            tc.verifyEqual(R5(2, 2), C5i(2, 2)*G.smInfl, 'RelTol', 1e-9, 'no weak axis: R5 unchanged');
            tc.m('T-NAV-GUARD', 'kRvel %s; weak-axis R5 sigma: %.3f m (no guard), %.3f m (uncertain start, no move)', ...
                mat2str(G.kRvel), sqrt(R5n), 0.30);
        end

        function tModeUnit(tc)
            % INIT -> ALIGN -> RUN_GNSS -> DEAD_RECKONING -> RUN_GNSS; FAULT reset pulse; navReset; DR_TEST mask
            M = tc.P1.sensors.navmode;
            clear sens_nav_mode
            c = onCleanup(@() clear('sens_nav_mode'));
            Pbig = [1e4 1e4 9.87];  Psmall = [1e-4 1e-4 1e-5];
            f = @(t, cmd, rst, en, nis, Pd) sens_nav_mode(t, uint8(cmd), rst, true, true, true, true, uint8(4), en, en, nis, Pd, M);
            no = false(5, 1);  g = [true; true; false; true; false];
            t = 0;  st = [];
            for k = 1:250, t = t + 0.01; st = f(t, 0, false, no, zeros(5,1), Pbig); end
            tc.verifyEqual(st, uint8(1), 'ALIGN after 2 s of valid IMU');
            [st, ~, nv, al] = f(t + 0.01, 0, false, g, ones(5,1), Psmall);  t = t + 0.01;
            tc.verifyEqual(st, uint8(2), 'RUN_GNSS once aligned');
            tc.verifyTrue(al);  tc.verifyTrue(nv);
            for k = 1:250, t = t + 0.01; st = f(t, 0, false, [false; false; false; true; false], ones(5,1), Psmall); end
            tc.verifyEqual(st, uint8(4), 'DEAD_RECKONING after 2 s without an absolute update');
            st = f(t + 0.01, 0, false, g, ones(5,1), Psmall);  t = t + 0.01;
            tc.verifyEqual(st, uint8(2), 'back to RUN_GNSS');
            % FAULT: normalised NIS > 3 for > 5 s
            rstSeen = false;
            for k = 1:900
                t = t + 0.01;
                [st, ~, ~, ~, ~, ~, rstOut] = f(t, 0, false, g, 20*ones(5,1), Psmall);
                rstSeen = rstSeen || rstOut;
                if rstOut, break, end
            end
            tc.verifyTrue(rstSeen, 'FAULT issues a reset pulse');
            tc.verifyEqual(st, uint8(5));
            % navReset edge
            [st, ~, nv, al, ~, ~, rstOut, ~, ~, nr] = f(t + 0.01, 0, true, no, zeros(5,1), Pbig);
            tc.verifyEqual(st, uint8(0));  tc.verifyFalse(nv);  tc.verifyFalse(al);  tc.verifyTrue(rstOut);
            tc.verifyEqual(nr, uint16(2), 'reset counter (FAULT + navReset)');
        end

        function tModeUnitImuLoss(tc)
            % critfix R-05 / NAV-03: the IMU-invalid time counts in every state; after tImuLossInv navValid is false and
            % quality INVALID; after tImuLossFault (in ALIGN, RUN_* or DEAD_RECKONING) FAULT with one reset pulse; FAULT
            % holds while the IMU stays invalid; when it is valid again a second reset pulse and INIT; an IMU invalid
            % in INIT (start-up) is no fault
            M = tc.P1.sensors.navmode;
            clear sens_nav_mode
            c = onCleanup(@() clear('sens_nav_mode'));
            Pbig = [1e4 1e4 9.87];  Psmall = [1e-4 1e-4 1e-5];  no = false(5, 1);
            g = [true; true; false; true; false];  dr = [false; false; false; true; false];
            f = @(t, imu, en, Pd) sens_nav_mode(t, uint8(0), false, imu, true, true, true, uint8(4), en, en, ones(5, 1), Pd, M);
            t = 0;  rs = false;
            for k = 1:150, t = t + 0.01; [st, ~, ~, ~, ~, ~, r] = f(t, false, no, Pbig); rs = rs || r; end
            tc.verifyEqual(st, uint8(0), 'INIT while the IMU is invalid from the start (1.5 s)');
            tc.verifyFalse(rs, 'no FAULT and no reset from INIT');
            for k = 1:250, t = t + 0.01; f(t, true, no, Pbig); end
            [st, ~, nv] = f(t + 0.01, true, g, Psmall);  t = t + 0.01;
            tc.verifyEqual(st, uint8(2), 'RUN_GNSS');  tc.verifyTrue(nv);
            % a loss shorter than tImuLossInv keeps navValid
            nS = round(M.tImuLossInv/M.dt) - 1;  nvS = true;
            for k = 1:nS, t = t + 0.01; [~, ~, nv] = f(t, false, g, Psmall); nvS = nvS && nv; end
            [~, ~, nv] = f(t + 0.01, true, g, Psmall);  t = t + 0.01;
            tc.verifyTrue(nvS && nv, sprintf('navValid through a %.2f s loss', nS*M.dt));
            for s0 = [2 4]
                % run in RUN_GNSS (2), or 2.5 s without an absolute update -> DEAD_RECKONING (4)
                if s0 == 4
                    for k = 1:250, t = t + 0.01; st = f(t, true, dr, Psmall); end
                    en = dr;
                else
                    en = g;
                end
                tc.verifyEqual(st, uint8(s0), sprintf('state %d before the loss', s0));
                t0 = t;  tI = NaN;  tF = NaN;  qI = NaN;  nR = 0;  alF = true;
                for k = 1:200
                    t = t + 0.01;
                    [st, q, nv, al, ~, ~, r] = f(t, false, en, Psmall);
                    if isnan(tI) && ~nv, tI = t - t0;  qI = double(q); end
                    if isnan(tF) && st == 5, tF = t - t0;  alF = al; end
                    nR = nR + r;
                end
                stL = st;
                [st1, ~, nv1, ~, ~, ~, r1] = f(t + 0.01, true, en, Psmall);  t = t + 0.01;
                tc.m('T-NAV-MODE', ['IMU lost in state %d: navValid false after %.2f s (quality %d), FAULT after %.2f s (aligned ' ...
                    '%d), %d reset pulse(s) during the loss, state %d after 2 s; IMU valid again: state %d, reset %d, navValid %d'], ...
                    s0, tI, qI, tF, alF, nR, stL, st1, r1, nv1);
                tc.verifyEqual(tI, M.tImuLossInv, 'AbsTol', 1e-9, 'navValid false after tImuLossInv');
                tc.verifyEqual(qI, 0, 'quality INVALID');
                tc.verifyEqual(tF, M.tImuLossFault, 'AbsTol', 1e-9, 'FAULT after tImuLossFault');
                tc.verifyFalse(alF, 'not aligned in FAULT');
                tc.verifyEqual(nR, 1, 'one reset pulse at the FAULT');
                tc.verifyEqual(stL, uint8(5), 'FAULT while the IMU stays invalid');
                tc.verifyEqual(st1, uint8(0), 'INIT when the IMU is valid again');
                tc.verifyTrue(r1, 'reset pulse when the IMU is valid again');
                tc.verifyFalse(nv1);
                % re-align: 2 s of valid IMU (INIT -> ALIGN), then RUN_GNSS
                for k = 1:250, t = t + 0.01; f(t, true, no, Pbig); end
                st = f(t + 0.01, true, g, Psmall);  t = t + 0.01;
                tc.verifyEqual(st, uint8(2), 'aligned again after the IMU returned');
            end
        end
    end

    % =========================================================================================
    %  bench scenarios (sensors spec section 9, T-NAV-01..11, D49 slip hold, S7-style outage)
    % =========================================================================================
    methods (Test, TestTags = {'bench'})
        function tNAV01_StaticAlignment(tc)
            R = nav_tb_run(tc.P1, struct('T', 30), struct('pose0', [5 3 0.6]));
            n = R.nav;
            tv = n.time(find(n.navValid, 1));
            k10 = t_nav.idx(n.time, 10);
            sp = max(n.posStd(k10, 1:2));  sy = n.yawStd(k10)*180/pi;
            w = n.time >= 29 & n.time <= 30;
            bgErr = abs(mean(n.yawRate(w)));     % true yaw rate is 0: residual = gyro bias error
            tc.m('T-NAV-01', 'navValid at %.2f s; at 10 s sigma_pos %.4f m, sigma_psi %.3f deg; bias error at 30 s %.2e rad/s (wall %.0f s)', ...
                 tv, sp, sy, bgErr, R.wall);
            tc.verifyLessThanOrEqual(tv, 15, 'navValid within 15 s');
            tc.verifyLessThan(sp, 0.03);
            tc.verifyLessThan(sy, 0.8);
            tc.verifyLessThan(bgErr, 0.002);
        end

        function tNAV02_LawnMower(tc)
            P = tc.P1;  T = 600;
            seg = {{'wait', 10}, {'speed', 1, 2}};
            for k = 1:12
                seg = [seg, {{'straight', 40}, {'turn', (-1)^(k+1)*pi, 0.5}}]; %#ok<AGROW>
            end
            kin = nav_tb_path([seg, {{'straight', 40}}], t_nav.base(P, 0.03));
            R = nav_tb_run(P, struct('T', T, 'kin', kin), struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R, [20 T]);
            tc.m('T-NAV-02', ['10 min lawn-mower 1 m/s RTK: horizontal RMS %.4f m, max %.4f m; yaw RMS %.3f deg; ' ...
                 'speed RMS %.4f m/s; mean 3-DOF NEES %.2f (wall %.0f s)'], M.rmsH, M.maxH, M.rmsYaw*180/pi, M.rmsV, ...
                 M.meanNees3, R.wall);
            tc.verifyLessThan(M.rmsH, 0.03);
            tc.verifyLessThan(M.maxH, 0.08);
            tc.verifyLessThan(M.rmsYaw*180/pi, 0.5);
            tc.verifyLessThan(M.rmsV, 0.03);
            tc.verifyGreaterThanOrEqual(M.meanNees3, 1.5);
            tc.verifyLessThanOrEqual(M.meanNees3, 4.5);
        end

        function tNAV03_SlipStepAndGauge(tc)
            P = tc.P1;  chiT = 1.5;          % true gauge factor differs from the prior chi0 = chi_nom
            seg = {{'wait', 10}, {'speed', 1, 2}, {'straight', 48}, {'set', 4, 0.15, 0.01}, {'set', 5, 0.15, 0.01}, ...
                   {'straight', 30}};
            for k = 1:8
                seg = [seg, {{'turn', pi/2, 0.5}, {'straight', 3}, {'turn', -pi/2, 0.5}, {'straight', 3}}]; %#ok<AGROW>
            end
            kin = nav_tb_path(seg, t_nav.base(P, 0.03, chiT));
            T = min(kin(end, 1), 185);
            R = nav_tb_run(P, struct('T', T, 'kin', kin), struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R);  t = M.t;
            tStep = 60.02;  tTurn = 90.04;
            w = t >= tStep + 10 & t <= tTurn;
            eS = max(max(abs(M.eSlip(w, :))));
            wc = t >= tTurn + 60 & t <= T;
            eC = max(abs(M.eChi(wc)));
            tc.m('T-NAV-03', 'slip step 0.03 -> 0.15: max |s_hat - s| %.4f in [%.0f, %.0f] s; chi %.2f: max |chi_hat - chi| %.3f 60 s after turning starts', ...
                 eS, tStep + 10, tTurn, chiT, eC);
            tc.verifyLessThan(eS, 0.02);
            tc.verifyLessThan(eC, 0.1);
        end

        function tNAV04_PalmRow(tc)
            % palm_farm: lane x = 14 m, 2 m from the trunk line x = 12 m, under the canopy edge (sky view 0.45 - 1.0 along
            % the row; the aisle centre x = 16 m is open sky in the world data, P.world.gnssSvf = 1), from the south
            % headland to the north one; the rows span y 8..112 m (104 m, shorter than the 160 m of the spec)
            P = tc.P3;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 131}}, t_nav.base(P, 0.04));
            R = nav_tb_run(P, struct('T', kin(end, 1), 'kin', kin), struct('pose0', [14 -8 pi/2]));
            M = nav_tb_metrics(R);
            y = R.truth(1:numel(M.t), 3);
            w = y >= 8 & y <= 112;
            rmsX = sqrt(mean(M.eCross(w).^2));  maxX = max(abs(M.eCross(w)));  rmsY = sqrt(mean(M.eYaw(w).^2))*180/pi;
            fx = double(R.gnss.fixType);  yg = interp1(M.t, y, R.gnss.time, 'nearest', 'extrap');  wg = yg >= 8 & yg <= 112;
            nRst = double(R.nav.resetCount(end));  nFault = sum(R.diag.modeState == 5);
            tc.m('T-NAV-04', ['palm row %.0f m (in row %.0f m): cross-track RMS %.3f m, max %.3f m; yaw RMS %.2f deg; ' ...
                 'resets %d, FAULT samples %d; fix fractions in row FIXED %.2f FLOAT %.2f SINGLE %.2f NOFIX %.2f'], ...
                 max(y) - min(y), 104, rmsX, maxX, rmsY, nRst, nFault, mean(fx(wg) == 4), mean(fx(wg) == 5), ...
                 mean(fx(wg) == 1), mean(fx(wg) == 0));
            tc.verifyLessThan(rmsX, 0.10);
            tc.verifyLessThan(maxX, 0.25);
            tc.verifyLessThan(rmsY, 1.0);
            tc.verifyEqual(nRst, 0);
            tc.verifyEqual(nFault, 0);
        end

        function tNAV05_GnssOutageTurn(tc)
            P = tc.P1;  t0 = 60;  t1 = 90;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 58}, {'turn', pi/2, 0.5}, {'straight', 55}}, ...
                              t_nav.base(P, 0.04));
            prof = struct('T', 125, 'kin', kin, 'flt', {{'gnss_outage', [0 0; t0 1; t1 0]}});
            R = nav_tb_run(P, prof, struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R);  t = M.t;
            k0 = t_nav.idx(t, t0);  k1 = t_nav.idx(t, t1);
            eEnd = M.eH(k1);  dYaw = abs(t_nav.wrap(M.eYaw(k1) - M.eYaw(k0)))*180/pi;
            g = R.gnss;  kf = find(g.time > t1 & g.fixType == 4 & g.newData, 1);
            tFix = g.time(kf);
            k50 = find(t >= tFix & M.eH < 0.05, 1);
            tBack = t(k50) - tFix;
            tc.m('T-NAV-05', '30 s outage incl. 90 deg turn: position error at end %.3f m, yaw drift %.2f deg; RTK re-fix at %.1f s, < 0.05 m after %.2f s', ...
                 eEnd, dYaw, tFix, tBack);
            tc.verifyLessThan(eEnd, 0.5);
            tc.verifyLessThan(dYaw, 2);
            tc.verifyLessThanOrEqual(tBack, 5);
        end

        function tNAV06_OutlierRejection(tc)
            P = tc.P1;
            P.sensors.gnss.pOutlier = 0.02;  P.sensors.gnss.svfOutlier = 1.1;   % multipath outliers also in open sky
            P.sensors.gnss.initFix = 5;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 40}, {'turn', pi, 0.5}, {'straight', 40}, ...
                               {'turn', -pi, 0.5}, {'straight', 40}}, t_nav.base(P, 0.03));
            T = kin(end, 1);
            prof = struct('T', T, 'kin', kin, 'flt', {{'gnss_force_float', [0 1]}});
            R = nav_tb_run(P, prof, struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R);
            d = R.diag;  g = R.gnss;  sd = R.sdiag;
            ev = find(d.nis(:, 1) > 0 & d.time > 15);           % evaluated GNSS position epochs
            tm = d.time(ev) - d.gnssLatency(ev);                 % time of validity of the epoch
            [~, kg] = min(abs(g.t_meas(:).' - tm(:)), [], 2);   % epoch of each evaluated sample
            out = logical(sd.outlier(kg));  rej = ~d.enable(ev, 1);
            spk = zeros(sum(out), 1);  io = ev(out);
            for k = 1:numel(io)
                a = max(io(k) - 1, 1);  b = min(io(k) + 20, numel(M.eH));
                spk(k) = max(abs(M.eH(a:b) - M.eH(a)));
            end
            tc.m('T-NAV-06', 'FLOAT, outliers 2-5 m at %.1f %%/epoch: %d outlier epochs, %.1f %% rejected; %d clean epochs, %.2f %% rejected; max spike %.3f m', ...
                 100*P.sensors.gnss.pOutlier, sum(out), 100*mean(rej(out)), sum(~out), 100*mean(rej(~out)), max([spk; 0]));
            tc.verifyGreaterThanOrEqual(sum(out), 10, 'enough outliers for the statistics');
            tc.verifyGreaterThanOrEqual(mean(rej(out)), 0.95);
            tc.verifyLessThan(mean(rej(~out)), 0.01);
            tc.verifyLessThan(max([spk; 0]), 0.10);
        end

        function tNAV07_GreenhouseLidar(tc)
            % greenhouse coverage with LiDAR scan-to-map localization (emulated matches, 3 cm prior-map error),
            % navMode 2 LIDAR_PRIOR_MAP, GNSS cold start under the roof (sky view 0.12), heading from the ecompass at
            % standstill with motors off. Shortened from 22 min to 4 legs (240 s) for the session time limit.
            P = tc.P4;  v = 0.6;  L = 31;
            seg = {{'wait', 10}, {'speed', v, 1}, {'straight', L/v - 1}, {'turn', -pi, 0.5}, {'straight', L/v}, ...
                   {'turn', pi, 0.5}, {'straight', L/v}, {'turn', pi, 0.5}, {'straight', L/v - 1}, {'speed', 0, 1}};
            kin = nav_tb_path(seg, t_nav.base(P, 0.03));
            T = kin(end, 1);
            prof = struct('T', T, 'kin', kin, 'sm', [0 1 0.03 0.5*pi/180 0.1], ...
                          'navcmd', {{'navMode', [0 2]}});
            R = nav_tb_run(P, prof, struct('pose0', [2.0 3.0 pi/2]));
            M = nav_tb_metrics(R, [15 T]);
            d = R.diag;
            avS = d.nis(:, 5) > 0;
            accS = sum(d.enable(:, 5))/max(1, sum(avS));
            smValid = mean(R.sm.valid(R.sm.time > 15));
            nees = M.meanNees3;
            tc.m('T-NAV-07', ['greenhouse %.0f s coverage 0.6 m/s, LiDAR localization: horizontal RMS %.3f m, max %.3f m; ' ...
                 'yaw RMS %.2f deg; scan matches valid %.1f %% of attempts, %.1f %% of the valid ones accepted; mean NEES %.2f'], ...
                 T, M.rmsH, M.maxH, M.rmsYaw*180/pi, 100*smValid, 100*accS, nees);
            tc.verifyLessThan(M.rmsH, 0.10);
            tc.verifyLessThan(M.maxH, 0.25);
            tc.verifyLessThan(M.rmsYaw*180/pi, 1.5);
            tc.verifyGreaterThan(smValid, 0.8);
            tc.verifyGreaterThan(accS, 0.95);
            tc.verifyLessThan(nees, 3*3, 'NEES < 3x nominal (3 DOF)');
        end

        function tNAV08_LatencyCompensation(tc)
            P = tc.P1;
            kin = nav_tb_path({{'wait', 10}, {'speed', 2, 3}, {'straight', 27}}, t_nav.base(P, 0.03));
            R = nav_tb_run(P, struct('T', 40, 'kin', kin), struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R, [20 40]);
            b = mean(M.eAlong(M.win));
            P0 = P;  P0.sensors.nav.tauMax = 0;            % reference: latency compensation off
            R0 = nav_tb_run(P0, struct('T', 40, 'kin', kin), struct('pose0', [0 0 0]));
            M0 = nav_tb_metrics(R0, [20 40]);
            b0 = mean(M0.eAlong(M0.win));
            tc.m('T-NAV-08', '2 m/s straight, epoch age %.2f s (0.08 s receiver + 0.1 s rate transition): along-track bias %.4f m compensated, %.3f m uncompensated', ...
                 median(R.diag.gnssLatency(R.diag.nis(:, 1) > 0)), b, b0);
            tc.verifyLessThan(abs(b), 0.02);
            tc.verifyGreaterThan(abs(b0), 0.1, 'reference without compensation shows the latency error');
        end

        function tNAV09_NavReset(tc)
            P = tc.P1;  tR = 120;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 140}}, t_nav.base(P, 0.03));
            prof = struct('T', 140, 'kin', kin, 'navcmd', {{'navReset', [0 0; tR 1; tR + 0.05 0]}});
            R = nav_tb_run(P, prof, struct('pose0', [0 0 0]));
            d = R.diag;  n = R.nav;
            k = find(double(n.resetCount) >= 1, 1);
            E = P.sensors.ekf;
            ex = max(abs(d.x(k, :).' - E.x0));  eP = max(abs(d.P_diag(k, :).' - diag(E.P0)));
            enk = any(d.enable(k, :));
            kv = find(n.time > n.time(k) & n.navValid, 1);
            tBack = n.time(kv) - n.time(k);
            tc.m('T-NAV-09', ['navReset at %.2f s: reset sample %.2f s, corrections in that sample %d, max |x - x0| %.3g, ' ...
                 'max |diag P - diag P0| %.3g; navValid %d at reset, true again after %.2f s'], tR, n.time(k), enk, ex, eP, ...
                 n.navValid(k), tBack);
            tc.verifyFalse(enk, 'no correction coincides with the reset sample');
            tc.verifyLessThan(ex, 1e-9, 'states return to x0 within one sample');
            tc.verifyLessThan(eP, 1e-9, 'covariance returns to P0 within one sample');
            tc.verifyFalse(logical(n.navValid(k)));
            tc.verifyLessThanOrEqual(tBack, 15);
        end

        function tNAV10_EcompassInit(tc)
            % no dual-antenna heading (RELPOS never valid), stationary, motors off
            P = tc.P1;  P.sensors.gnss.markov.h_gain = 0;  psi0 = 1.0;
            prof = struct('T', 25, 'bms', {{'I_pack', [0 0]}});
            R = nav_tb_run(P, prof, struct('pose0', [5 3 psi0]));
            M = nav_tb_metrics(R);
            kv = find(R.nav.navValid, 1);
            e1 = abs(M.eYaw(kv))*180/pi;  e2 = abs(M.eYaw(end))*180/pi;
            % declination sign: ideal magnetometer -> the residual must be far below the 2 x 2.93 deg of a sign error
            Pi = P;  Pi.sensors.imu.mag.ConstantBias = [0 0 0];  Pi.sensors.imu.mag.BiasInstability = 0;
            Pi.sensors.imu.kMagI = [0 0 0];
            Ri = nav_tb_run(Pi, prof, struct('pose0', [5 3 psi0]));
            Mi = nav_tb_metrics(Ri);
            ei = abs(Mi.eYaw(end))*180/pi;
            tc.m('T-NAV-10', ['ecompass init (no dual antenna): navValid at %.2f s, yaw error %.2f deg at navValid, %.2f deg at 25 s; ' ...
                 'ideal magnetometer %.2f deg (declination %.2f deg)'], R.nav.time(kv), e1, e2, ei, P.sensors.magDecl);
            tc.verifyNotEmpty(kv, 'aligned with the ecompass heading');
            tc.verifyLessThan(e1, 8);
            tc.verifyLessThan(e2, 8);
            tc.verifyLessThan(ei, P.sensors.magDecl/2, 'declination subtracted with the right sign');
        end

        function tNAV11_Slope(tc)
            % 20 deg slope: climb along +x, then side-slope traverse (heading north) with downhill skid vy = 0.05 m/s
            P = tc.P1;  gx = tand(20);
            b = t_nav.base(P, 0.05);  b(7) = gx;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 20}, {'turn', pi/2, 0.5}, ...
                               {'set', 3, 0.05, 1}, {'straight', 24}}, b);
            T = kin(end, 1);
            R = nav_tb_run(P, struct('T', T, 'kin', kin), struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R, [15 T]);  t = M.t;
            tTurn = 32;  tTr = tTurn + pi + 2;
            wC = t >= 17 & t <= tTurn;  wT = t >= tTr + 1 & t <= T;
            % criteria "pitch error < 1.0 deg" and "vy within 0.03 m/s" are evaluated as the RMS over the steady
            % climb / traverse windows (the max, which includes the 90 deg turn transient, is reported)
            rP = @(w) sqrt(mean(M.ePitch(w).^2))*180/pi;  mP = @(w) max(abs(M.ePitch(w)))*180/pi;
            pC = rP(wC);  pT = rP(wT);
            vyR = sqrt(mean(M.eVy(wT).^2));  vyM = mean(M.eVy(wT));  vyX = max(abs(M.eVy(wT)));
            rmsP3 = sqrt(mean(sum(M.ePos(M.win, :).^2, 2)));
            tc.m('T-NAV-11', ['20 deg slope: pitch error RMS %.2f deg climbing (max %.2f), %.2f deg traversing (max %.2f, roll ' ...
                 '%.1f deg); side-slip vy %.2f m/s: error RMS %.4f m/s, mean %.4f, max %.4f; position RMS %.4f m (horizontal %.4f m)'], ...
                 pC, mP(wC), pT, mP(wT), mean(R.truth(wT, 7))*180/pi, 0.05, vyR, vyM, vyX, rmsP3, M.rmsH);
            tc.verifyLessThan(max(pC, pT), 1.0);
            tc.verifyLessThan(vyR, 0.03);
            tc.verifyLessThan(rmsP3, 0.05);
        end

        function tSlipHoldOutageFarm(tc)
            % D49 and S7-style 30 s GNSS outage at 1.0 m/s on a straight 88 m aisle, farm soil (slip 0.04), then 10 s FLOAT
            [dS, eA] = t_nav.s7(tc.P1, 0.04);
            tc.m('S7-FARM', 'slip drift during the 30 s outage %.4f (L) %.4f (R); along-track error at outage end %.3f m', dS, eA);
            tc.verifyLessThan(max(dS), 0.003);
            tc.verifyLessThanOrEqual(abs(eA), 0.5);
        end

        function tS7OutageWetSoil(tc)
            % S7-style 30 s outage, wet loam (slip 0.12)
            [dS, eA] = t_nav.s7(tc.P1, 0.12);
            tc.m('S7-WET', 'slip drift during the 30 s outage %.4f (L) %.4f (R); along-track error at outage end %.3f m', dS, eA);
            tc.verifyLessThan(max(dS), 0.003);
            tc.verifyLessThanOrEqual(abs(eA), 0.5);
        end

        function tNAV13_ImuLoss(tc)
            % critfix R-05 / NAV-03 (the review's bench command): greenhouse, LiDAR mode, 0.6 m/s, from tL to the end an IMU
            % dropout (imu_fault 3: sens_acq_imu valid false at once) or a stuck IMU (imu_fault 1: sens_acq_imu valid false
            % once all 9 raw axes have repeated for P.electronics.safety.imu_frozen_t, the MCU's frozen-IMU check). navValid
            % falls within tImuLossInv of the invalid IMU (plus the 0.005 -> 0.01 s rate transition), quality INVALID,
            % FAULT after tImuLossFault with one reset pulse, FAULT to the end; before the loss (at rest and moving) the IMU
            % is valid and the solution valid
            P = tc.P4;  M = P.sensors.navmode;  tL = 20;  dly = 0.02;
            kin = nav_tb_path({{'wait', 10}, {'speed', 0.6, 1}, {'straight', 15}, {'turn', pi, 0.5}, {'straight', 15}}, ...
                t_nav.base(P, 0.03));
            T = kin(end, 1);
            for fk = [3 1]
                if fk == 3, nm = 'dropout';  d0 = 0; else, nm = 'stuck';  d0 = P.electronics.safety.imu_frozen_t; end
                R = nav_tb_run(P, struct('T', T, 'kin', kin, 'sm', [0 1 0.03 0.5*pi/180 0.1], 'navcmd', {{'navMode', [0 2]}}, ...
                    'flt', {{'imu_fault', [0 0; tL fk]}}), struct('pose0', [2.0 3.0 pi/2]));
                n = R.nav;  t = n.time(:);  nv = logical(n.navValid(:));  q = double(n.quality(:));
                ms = double(R.diag.modeState(:));  N = min(numel(t), numel(ms));
                t = t(1:N);  nv = nv(1:N);  q = q(1:N);  ms = ms(1:N);
                Mt = nav_tb_metrics(R);  eYaw = Mt.eYaw(1:min(N, end));
                pre = t > 12 & t < tL;
                kI = find(t > tL & ~nv, 1);  kF = find(t > tL & ms == 5, 1);
                tI = Inf;  if ~isempty(kI), tI = t(kI) - tL; end
                tF = Inf;  if ~isempty(kF), tF = t(kF) - tL; end
                w = t > tL & t < tL + min(tI, T);
                tc.m('T-NAV-13', ['IMU %s at %.0f s (greenhouse, LiDAR mode, 0.6 m/s): navValid before %.3f; false after %.2f s ' ...
                    '(max |yaw error| until then %.2f deg), quality %s after; FAULT after %.2f s, modes after the loss %s, ' ...
                    'resetCount %d (wall %.0f s)'], nm, tL, mean(nv(pre)), tI, max(abs(eYaw(w(1:numel(eYaw)))))*180/pi, ...
                    mat2str(unique(q(t > tL + d0 + M.tImuLossInv + dly))'), tF, mat2str(unique(ms(t > tL))'), ...
                    double(n.resetCount(end)), R.wall);
                tc.verifyTrue(all(nv(pre)), [nm ': navValid before the loss']);
                tc.verifyLessThanOrEqual(tI, d0 + M.tImuLossInv + dly + 1e-9, [nm ': navValid false within tImuLossInv of the invalid IMU']);
                tc.verifyFalse(any(nv(t > tL + d0 + M.tImuLossInv + dly)), [nm ': navValid false while the IMU is lost']);
                tc.verifyTrue(all(q(t > tL + d0 + M.tImuLossInv + dly) == 0), [nm ': quality INVALID while the IMU is lost']);
                tc.verifyEqual(tF, d0 + M.tImuLossFault, 'AbsTol', dly + 1e-9, [nm ': FAULT after tImuLossFault']);
                tc.verifyTrue(all(ms(t > tL + d0 + M.tImuLossFault + dly) == 5), [nm ': FAULT while the IMU is lost']);
                tc.verifyEqual(double(n.resetCount(end)), 1, [nm ': one reset pulse']);
            end
        end
    end

    methods (Static, Access = private)
        function [dS, eA] = s7(P, s)
            t0 = 40;  t1 = 70;
            kin = nav_tb_path({{'wait', 10}, {'speed', 1, 2}, {'straight', 88}}, t_nav.base(P, s));
            prof = struct('T', 95, 'kin', kin, 'flt', {{'gnss_outage', [0 0; t0 1; t1 0]; 'gnss_force_float', [0 0; t1 1; t1 + 10 0]}});
            R = nav_tb_run(P, prof, struct('pose0', [0 0 0]));
            M = nav_tb_metrics(R);
            % pre-outage value = mean slip estimate over the 2 s before the outage
            k1 = t_nav.idx(M.t, t1);
            wPre = M.t >= t0 - 2 & M.t < t0;
            dS = abs(R.nav.slip_LR(k1, :) - mean(R.nav.slip_LR(wPre, :), 1));
            eA = M.eAlong(k1);
        end
    end

    % =========================================================================================
    %  code generation and HW acquisition variant
    % =========================================================================================
    methods (Test, TestTags = {'fast'})
        function tCodegenCompanion(tc)
            % Shaddad_NavFusion generates code with cs_Companion (ert C++, ROS 2): BLOCKED while Simulink Coder is missing
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
        end

        function tHwGnssAcquisition(tc)
            % HW choice of IO_In = the sens_acq_gnss content (ros2lib/Subscribe of /fix, /fix_velocity, /navheading)
            % inline: identical blocks in the model and in subsystems/sensors/sens_acq_gnss.slx, and it compiles
            tc.assumeTrue(any(strcmp({ver().Name}, 'ROS Toolbox')), 'ROS Toolbox not installed');
            mdl = 'Shaddad_NavFusion';
            wasLoaded = bdIsLoaded(mdl);
            if ~wasLoaded, load_system(mdl); end
            c0 = onCleanup(@() local_close(mdl, wasLoaded));
            hw = [mdl '/IO_In/HW'];
            b1 = sort(get_param(setdiff(find_system(hw, 'SearchDepth', 1, 'LookUnderMasks', 'all'), {hw}), 'Name'));
            ssrLoaded = bdIsLoaded('sens_acq_gnss');
            if ~ssrLoaded, load_system('sens_acq_gnss'); end
            b2 = sort(get_param(find_system('sens_acq_gnss', 'SearchDepth', 1, 'LookUnderMasks', 'all', 'Type', 'block'), 'Name'));
            tc.verifyEqual(b1, b2, 'IO_In/HW content = sens_acq_gnss.slx');
            n = 'tmp_t12_hw_gnss';
            if bdIsLoaded(n), close_system(n, 0); end
            new_system(n);
            c = onCleanup(@() close_system(n, 0));
            cs = copy(evalin('base', 'cs_SimFixed'));  cs.Name = 'c';
            attachConfigSet(n, cs, true);  setActiveConfigSet(n, 'c');
            add_block('simulink/Ports & Subsystems/Subsystem', [n '/Acq']);
            Simulink.SubSystem.deleteContents([n '/Acq']);
            Simulink.BlockDiagram.copyContentsToSubsystem('sens_acq_gnss', [n '/Acq']);
            if ~ssrLoaded, close_system('sens_acq_gnss', 0); end
            add_block('simulink/Sinks/Terminator', [n '/T']);
            add_line(n, 'Acq/1', 'T/1');
            try
                evalc('set_param(n, ''SimulationCommand'', ''update'')');
                ok = true;  msg = '';
            catch e
                ok = false;  msg = e.message;
            end
            tc.m('HW-GNSS', 'sens_acq_gnss update diagram: %d %s', ok, msg);
            tc.verifyTrue(ok, msg);
            % the whole model with the HW choice active (VC_TARGET = 1 as a simulation-scoped variable): 0.2 s run
            h = 'tmp_t12_nav_hw';
            if bdIsLoaded(h), close_system(h, 0); end
            new_system(h);
            c2 = onCleanup(@() close_system(h, 0));
            cs = copy(evalin('base', 'cs_SimFixed'));  cs.Name = 'c';
            attachConfigSet(h, cs, true);  setActiveConfigSet(h, 'c');
            b = {'imu', 'Bus_ImuMeas', 'Ts.imu'; 'odom', 'Bus_OdomMeas', 'Ts.odom'; 'gnss', 'Bus_GnssRaw', 'Ts.gnss'; ...
                 'sm', 'Bus_ScanMatchMeas', 'Ts.scanmatch'; 'cmd', 'Bus_NavCmd', 'Ts.gnc'; 'bms', 'Bus_BmsStatus', 'Ts.hk'};
            add_block('simulink/Ports & Subsystems/Model', [h '/Nav'], 'ModelName', 'Shaddad_NavFusion');
            for k = 1:size(b, 1)
                add_block('simulink/Sources/Constant', [h '/' b{k, 1}], 'Value', ['P.ic.' b{k, 2}], ...
                    'OutDataTypeStr', ['Bus: ' b{k, 2}], 'SampleTime', b{k, 3});
                add_line(h, [b{k, 1} '/1'], sprintf('Nav/%d', k));
            end
            add_block('simulink/Sinks/Terminator', [h '/T1']);  add_block('simulink/Sinks/Terminator', [h '/T2']);
            add_line(h, 'Nav/1', 'T1/1');  add_line(h, 'Nav/2', 'T2/1');
            in = Simulink.SimulationInput(h);
            in = in.setVariable('VC_TARGET', 1);
            in = in.setModelParameter('StopTime', '0.2');
            try
                so = sim(in);  ok2 = isempty(so.ErrorMessage);  msg2 = so.ErrorMessage;
            catch e
                ok2 = false;  msg2 = e.message;
            end
            tc.m('HW-GNSS', 'Shaddad_NavFusion with VC_TARGET = 1 (HW IO_In): 0.2 s simulation %d %s', ok2, msg2);
            tc.verifyTrue(ok2, msg2);
        end
    end
end

% =============================================================================================
function local_close(mdl, wasLoaded)
if ~wasLoaded && bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
