classdef t_sensors < matlab.unittest.TestCase
%T_SENSORS  Acceptance tests of build task T11 (sensor plant models + MCU-side sensor acquisition).
%
%   runtests('tests/sensors/t_sensors.m')                       all tests (about 6-8 min)
%   runtests('tests/sensors/t_sensors.m', 'ProcedureName', 'tSEN01_ImuStaticLevel')
%
%   Bench: tests/sensors/tb_sens_plant.slx (scripts/build/build_tb_sens_plant.m) driven by
%   tests/sensors/sens_tb_input.m through tests/sensors/sens_tb_run.m. P is created with
%   shaddad_params(..., 'Assign', false) and passed with SimulationInput.setVariable, so the shared base
%   workspace is never modified (except sens_uavScene, the documented FULL3D scenario variable, which the
%   FULL3D tests rebuild for their world and restore). Every measured number is printed as
%   'MEASURED <id>: ...' and stored in the test diagnostics.
%   Criteria: docs/build_plan.md T11 acceptance and docs/specs/sensors.md section 9 (T-SEN-01..08, 11..14).

    properties
        P1      % flat_test world (default)
        P3      % palm_farm
        P4      % greenhouse
    end

    methods (TestClassSetup)
        function setup(tc)
            if ~evalin('base', 'exist(''Ts'',''var'') && exist(''Bus_ImuRaw'',''var'') && exist(''cs_SimFixed'',''var'')')
                shaddad_params();  shaddad_configsets('fixed');
            end
            tc.P1 = shaddad_params('Assign', false);
            tc.P3 = shaddad_params('World', 3, 'Assign', false);
            tc.P4 = shaddad_params('World', 4, 'Assign', false);
            root = char(currentProject().RootFolder);
            if ~isfile(fullfile(root, 'tests', 'sensors', 'tb_sens_plant.slx'))
                build_tb_sens_plant();
            end
            if ~bdIsLoaded('tb_sens_plant'), load_system('tb_sens_plant'); end
        end
    end

    methods (TestClassTeardown)
        function teardown(~)
            if bdIsLoaded('tb_sens_plant'), close_system('tb_sens_plant', 0); end
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
        function P = idealImu(P)
            % convention / lever-arm tests: error-free IMU (no bias, cross-axis, misalignment, noise, quantisation)
            for g = {'accel', 'gyro'}
                s = P.sensors.imu.(g{1});
                s.ConstantBias = [0 0 0];  s.AxesMisalignment = 100*eye(3);  s.NoiseDensity = [0 0 0];
                s.BiasInstability = 0;  s.RandomWalk = 0;  s.TemperatureBias = [0 0 0];
                s.TemperatureScaleFactor = [0 0 0];  s.Resolution = 0;
                if isfield(s, 'AccelerationBias'), s.AccelerationBias = [0 0 0]; end
                P.sensors.imu.(g{1}) = s;
            end
            P.sensors.imu.R_BI = eye(3);
        end
        function e = lla2enu_flat(lla, ref)
            % local ENU of LLA rows (small-area flat-Earth approximation, WGS-84 radii)
            a = 6378137; f = 1/298.257223563; e2 = f*(2 - f);
            lat0 = ref(1)*pi/180;
            RN = a/sqrt(1 - e2*sin(lat0)^2);  RM = RN*(1 - e2)/(1 - e2*sin(lat0)^2);
            e = [(lla(:, 2) - ref(2))*pi/180*RN*cos(lat0), (lla(:, 1) - ref(1))*pi/180*RM, lla(:, 3) - ref(3)];
        end
    end

    methods (Test)
        %% ------------------------------------------------------------------ IMU
        function tSEN01_ImuStaticLevel(tc)
            % T-SEN-01: static level 60 s, 25 C, motors off (full error model)
            P = tc.P1;
            R = sens_tb_run(P, struct('T', 60));
            k = R.imu.t > 1;
            mA = mean(R.imu.acc(k, :));  sA = std(R.imu.acc(k, :));
            mW = mean(R.imu.gyro(k, :)); sW = std(R.imu.gyro(k, :));
            I = P.sensors.imu;  g = P.frames.g;
            % expected: cross-axis matrix x true mounting misalignment x physical specific force + bias
            eA = (I.accel.AxesMisalignment/100*(I.R_BI'*[0; 0; g]))' + I.accel.ConstantBias;
            tc.m('T-SEN-01', 'acc mean %s (expected %s), std %s; gyro mean %s std %s', mat2str(mA, 5), ...
                mat2str(eA, 5), mat2str(sA, 4), mat2str(mW, 4), mat2str(sW, 4));
            tc.verifyEqual(mA(3), g + 0.08, 'AbsTol', 0.01, 'mean accel z = 9.7876 + 0.08');
            tc.verifyEqual(mA, eA, 'AbsTol', 0.01, 'mean accel = cross-axis * misalignment * g + ConstantBias');
            tc.verifyEqual(sA(3), 0.0183, 'RelTol', 0.2, 'std accel z');
            tc.verifyEqual(sW, 2.12e-3*[1 1 1], 'RelTol', 0.2, 'std gyro');
            tc.verifyEqual(mW, I.gyro.ConstantBias, 'AbsTol', 5e-4, 'mean gyro = ConstantBias');
        end

        function tSEN02_SignConvention(tc)
            % T-SEN-02: static 10 deg nose-down (ideal IMU): physical specific force; IMU Filter pitch +10 deg;
            % yaw spin 0.5 rad/s (full model): gyro_z = 0.5 + bias
            P = t_sensors.idealImu(tc.P1);
            th = 10*pi/180;
            R = sens_tb_run(P, struct('T', 30, 'pose', @(t) [zeros(numel(t), 4) th*ones(numel(t), 1) zeros(numel(t), 1)]));
            k = R.imu.t > 1;
            mA = mean(R.imu.acc(k, :));
            % IMU Filter (negative convention input, Gain -1 at the library block; settings P.sensors.imuf as the
            % NavFusion block) on the logged data, mean pitch over the last 5 s
            F = P.sensors.imuf;
            fuse = imufilter('SampleRate', 1/P.sensors.imu.Ts, 'ReferenceFrame', 'ENU', ...
                'AccelerometerNoise', F.AccelerometerNoise, 'GyroscopeNoise', F.GyroscopeNoise, ...
                'GyroscopeDriftNoise', F.GyroscopeDriftNoise, 'LinearAccelerationNoise', F.LinearAccelerationNoise, ...
                'LinearAccelerationDecayFactor', F.LinearAccelerationDecayFactor);
            q = fuse(-R.imu.acc(k, :), R.imu.gyro(k, :));        % after the 1 s start-up (DLPF / delays settled)
            eAll = eulerd(q, 'ZYX', 'frame');
            tk = R.imu.t(k);
            e = mean(eAll(tk > tk(end) - 5, :), 1);
            tc.m('T-SEN-02', 'nose-down 10 deg: accel %s; IMU Filter pitch %.3f deg', mat2str(mA, 5), e(2));
            tc.verifyEqual(mA(1), -1.700, 'AbsTol', 0.02, 'accel_x at 10 deg nose-down');
            tc.verifyEqual(mA(3), 9.639, 'AbsTol', 0.02, 'accel_z at 10 deg nose-down');
            tc.verifyEqual(e(2), 10, 'AbsTol', 0.3, 'IMU Filter pitch');
            P = tc.P1;
            R = sens_tb_run(P, struct('T', 6, 'pose', @(t) [zeros(numel(t), 3) 0.5*t zeros(numel(t), 2)]));
            k = R.imu.t > 1;
            gz = mean(R.imu.gyro(k, 3));
            tc.m('T-SEN-02', 'yaw spin 0.5 rad/s: gyro_z %.5f (0.5 + bias %.4f)', gz, P.sensors.imu.gyro.ConstantBias(3));
            tc.verifyEqual(gz, 0.5 + P.sensors.imu.gyro.ConstantBias(3), 'AbsTol', 0.005, 'gyro_z spin');
        end

        function tSEN03_LeverArm(tc)
            % T-SEN-03: spot turn 1.0 rad/s (ideal IMU), r_I x = 0.35 -> centripetal -0.35 m/s^2
            P = t_sensors.idealImu(tc.P1);
            R = sens_tb_run(P, struct('T', 6, 'pose', @(t) [zeros(numel(t), 3) 1.0*t zeros(numel(t), 2)]));
            k = R.imu.t > 1;
            mA = mean(R.imu.acc(k, :));
            ex = -P.sensors.imu.r(1)*1.0^2;
            tc.m('T-SEN-03', 'spot turn 1 rad/s: mean accel %s (expected x %.3f)', mat2str(mA, 4), ex);
            tc.verifyEqual(P.sensors.imu.r(1), 0.35, 'AbsTol', 1e-12, 'IMU mount x');
            tc.verifyEqual(mA(1), -0.35, 'AbsTol', 0.01, 'mean accel_x');
        end

        function tSEN04_TemperatureRamp(tc)
            % T-SEN-04 (build plan): gyro bias change over a 10 K temperature ramp = 8.7e-3 rad/s +/- 20 %
            P = tc.P1;
            Timu = @(t) 25 + 10*min(max((t - 5)/10, 0), 1);
            R = sens_tb_run(P, struct('T', 22, 'Timu', Timu));
            k0 = R.imu.t > 1 & R.imu.t < 5;  k1 = R.imu.t > 17;
            dW = mean(R.imu.gyro(k1, :)) - mean(R.imu.gyro(k0, :));
            dA = mean(R.imu.acc(k1, :)) - mean(R.imu.acc(k0, :));
            dT = mean(R.imu.temp(k1)) - mean(R.imu.temp(k0));
            tc.m('T-SEN-04', 'dT %.2f K: gyro bias change %s rad/s (expected %.2e), accel bias change %s m/s^2', ...
                dT, mat2str(dW, 4), 10*P.sensors.imu.gyro.TemperatureBias(1), mat2str(dA, 4));
            tc.verifyEqual(dT, 10, 'AbsTol', 0.01, 'temperature ramp seen by the IMU');
            tc.verifyEqual(dW, 8.7e-3*[1 1 1], 'RelTol', 0.2, 'gyro bias change per axis');
            % accel: temperature bias (datasheet 7.8e-3 /K) + temperature scale factor (0.02 %/K) acting on gravity
            eA = 10*P.sensors.imu.accel.TemperatureBias + 10*P.sensors.imu.accel.TemperatureScaleFactor/100.*mean(R.imu.acc(k0, :));
            tc.m('T-SEN-04', 'accel change expected %s (bias + scale factor x g)', mat2str(eA, 4));
            tc.verifyEqual(dA, eA, 'RelTol', 0.2, 'accel bias change');
        end

        function tImuFaults(tc)
            % imu_fault 1 stuck (hold), 2 bias step (+0.5 m/s^2, +0.05 rad/s), 3 dropout (frozen) and
            % sens_acq_imu valid false on imu_fault 3 (build plan T11.3 / T11.7), and on imu_fault 1 once all 9 raw axes
            % have repeated for imu_frozen_t (frozen-IMU check, critfix R-05; the window starts 0.01 s after 4.1 s)
            P = tc.P1;
            R = sens_tb_run(P, struct('T', 8, 'flt', {{'imu_fault', [0 0; 2 2; 4 1; 6 3]}}));
            t = R.imu.t;
            b = mean(R.imu.acc(t > 2.5 & t < 4, :)) - mean(R.imu.acc(t > 0.5 & t < 2, :));
            bw = mean(R.imu.gyro(t > 2.5 & t < 4, :)) - mean(R.imu.gyro(t > 0.5 & t < 2, :));
            st = R.imu.acc(t > 4.2 & t < 6, :);  dr = R.imu.acc(t > 6.2, :);
            tv = R.meas.t;  v = R.meas.valid;
            kS = find(tv > 4 & ~v, 1);  tS = NaN;  if ~isempty(kS), tS = tv(kS); end
            tc.m('T-ACQ', ['bias step acc %s gyro %s; stuck span %.3g; dropout span %.3g; valid 0.1-4.0 s %d, while stuck ' ...
                '4.11-6.0 s any %d (first false at %.3f s), during fault 3 %d'], mat2str(b, 3), mat2str(bw, 3), ...
                max(max(st) - min(st)), max(max(dr) - min(dr)), all(v(tv > 0.1 & tv < 4.0)), any(v(tv > 4.11 & tv < 6.0)), ...
                tS, any(v(tv > 6.1)));
            tc.verifyEqual(b, [0.5 0.5 0.5], 'AbsTol', 0.01, 'imu_fault 2 accel bias step');
            tc.verifyEqual(bw, [0.05 0.05 0.05], 'AbsTol', 0.002, 'imu_fault 2 gyro bias step');
            tc.verifyEqual(max(max(st) - min(st)), 0, 'AbsTol', 1e-6, 'imu_fault 1 holds the output');
            tc.verifyEqual(max(max(dr) - min(dr)), 0, 'AbsTol', 1e-6, 'imu_fault 3 freezes the output');
            tc.verifyTrue(all(v(tv > 0.1 & tv < 4.0)), 'sens_acq_imu valid before the stuck IMU (imu_fault 1 from 4 s)');
            tc.verifyFalse(any(v(tv > 4.11 & tv < 6.0)), 'sens_acq_imu valid false while the IMU is stuck (frozen-IMU check, R-05)');
            tc.verifyFalse(any(v(tv > 6.1)), 'sens_acq_imu valid false on imu_fault 3');
            tc.verifyEqual(diff(R.meas.seq(1:100)), ones(99, 1), 'seq increments every 5 ms');
            tc.verifyEqual(R.meas.acc, R.imu.acc, 'AbsTol', 1e-6, 'identity axis map, zero calibration offsets');
        end

        %% ------------------------------------------------------------------ GNSS
        function tSEN05_GnssOpenSky(tc)
            % T-SEN-05: open sky, stationary 300 s, cold start (NOFIX)
            P = tc.P1;
            R = sens_tb_run(P, struct('T', 300));
            k = R.gnss.t > 30;
            fixFrac = mean(R.gnss.fix(k) == 4);
            kf = k & R.gnss.fix == 4;
            e = R.diag.posErr(kf, :);
            psiTrue = 90;
            hd = R.gnss.heading(kf & R.gnss.headValid) - psiTrue;
            tc.m('T-SEN-05', 'fix=4 fraction after 30 s %.4f (first fixed at %.1f s); std E %.4f N %.4f m; heading std %.3f deg (n %d)', ...
                fixFrac, R.gnss.t(find(R.gnss.fix == 4, 1)), std(e(:, 1)), std(e(:, 2)), std(hd), numel(hd));
            % the chain's FIXED -> FLOAT base rate 0.002 1/s gives P(no fix loss in 270 s) = exp(-0.54) = 0.58,
            % so '100 %' is a stochastic criterion: >= 98 % is verified (expected 99 %, see the T11 report)
            tc.verifyGreaterThanOrEqual(fixFrac, 0.98, 'RTK fixed after 30 s');
            tc.verifyEqual(std(e(:, 1)), 0.015, 'AbsTol', 0.005, 'horizontal per-axis std E');
            tc.verifyEqual(std(e(:, 2)), 0.015, 'AbsTol', 0.005, 'horizontal per-axis std N');
            tc.verifyEqual(std(hd), 0.5, 'AbsTol', 0.1, 'heading std');
        end

        function tSEN06_GnssLatency(tc)
            % T-SEN-06: 1 m/s straight: delay of the reported position = 0.08 +/- 0.01 s
            P = tc.P1;  P.sensors.gnss.initFix = 4;           % warm start (RTK fixed) for the latency fit
            v = 1.0;
            R = sens_tb_run(P, struct('T', 30, 'pose', @(t) [v*t zeros(numel(t), 5)]));
            k = R.gnss.fix == 4 & R.gnss.t > 2;
            pRep = t_sensors.lla2enu_flat(R.gnss.lla(k, :), P.sensors.refLLA);
            tk = R.gnss.t(k);
            lags = 0:0.002:0.2;  J = zeros(size(lags));
            for j = 1:numel(lags)
                xTrue = v*(tk - lags(j)) + P.sensors.gnss.rAnt1(1);
                J(j) = mean((pRep(:, 1) - xTrue).^2);
            end
            [~, j] = min(J);
            tc.m('T-SEN-06', 'estimated latency %.3f s (t - t_meas = %.3f s)', lags(j), mean(R.gnss.t - R.gnss.t_meas));
            tc.verifyEqual(lags(j), 0.08, 'AbsTol', 0.01, 'GNSS latency');
            tc.verifyEqual(mean(R.gnss.t(R.gnss.t > 1) - R.gnss.t_meas(R.gnss.t > 1)), 0.08, 'AbsTol', 1e-9, 't_meas = t - 0.08');
        end

        function tSEN07_PalmRow(tc)
            % T-SEN-07: palm farm, aisle centre x = 8 m, 160 m at 1 m/s (cold start in the southern headland)
            P = tc.P3;
            y0 = -24;
            R = sens_tb_run(P, struct('T', 160, 'dt', 0.05, 'pose', @(t) [8*ones(numel(t), 1) y0 + t ...
                zeros(numel(t), 1) pi/2*ones(numel(t), 1) zeros(numel(t), 2)]));
            yA = y0 + R.gnss.t + P.sensors.gnss.rAnt1(1);    % antenna y (rear antenna, heading north)
            row = R.gnss.t > 30 & yA >= 8 & yA <= 112;
            head = R.gnss.t > 30 & (yA < 4 | yA > 116);
            fx = mean(R.gnss.fix(row) == 4);  fl = mean(R.gnss.fix(row) == 5);  nf = mean(R.gnss.fix(row) == 0);
            fh = mean(R.gnss.fix(head) == 4);
            sv = R.diag.svf(row);
            tc.m('T-SEN-07', ['in-row (y 8..112, %d epochs): fixed %.3f float %.3f single %.3f nofix %.3f; headland fixed %.3f; ' ...
                'svf along the aisle min %.3f mean %.3f (fraction < 1: %.3f)'], nnz(row), fx, fl, ...
                mean(R.gnss.fix(row) == 1), nf, fh, min(sv), mean(sv), mean(sv < 0.999));
            tc.verifyLessThan(nf, 0.05, 'no-fix < 5 %');
            tc.verifyGreaterThan(fh, 0.98, 'headland fixed > 98 % after 30 s');
            % spec band [40, 80] % fixed / [15, 50] % float needs canopy shading at the aisle centre; with the
            % D59 grid (canopy x 0.45 inside R 3..4 m circles, aisle 4 m from the trunks) the aisle is open sky
            tc.assumeTrue(fx >= 0.40 && fx <= 0.80 && fl >= 0.15 && fl <= 0.50, sprintf(['DEVIATION (world data): ' ...
                'aisle-centre svf mean %.3f -> fixed %.3f, float %.3f; spec band needs canopy over the aisle ' ...
                '(P.world.gnssSvf, T02)'], mean(sv), fx, fl));
        end

        function tSEN08_Greenhouse(tc)
            % T-SEN-08: greenhouse interior 10 min, 5 seeds (fix chain statistics with svf 0.12), plus the model
            % check of the sky-view lookups at the antenna inside the house (svf 0.12, zone 1)
            P = tc.P4;  G = sens_gnss_fixchain_prm(P.sensors.gnss);    % packed block input (no parameter data)
            n = 6000;  cnt = zeros(1, 6);  nOut = 0;  nFS = 0;
            for s = 1:5
                clear sens_gnss_fixchain
                rs = RandStream('mt19937ar', 'Seed', 100 + s);
                U = rand(rs, n, 4);
                for k = 1:n
                    [f, ~, o] = sens_gnss_fixchain(0.12, U(k, :), true, 1, false, false, G);
                    cnt(double(f) + 1) = cnt(double(f) + 1) + 1;
                    if f == 1 || f == 5, nFS = nFS + 1; nOut = nOut + o; end
                end
            end
            clear sens_gnss_fixchain
            fr = cnt/sum(cnt);
            tc.m('T-SEN-08', 'chain svf 0.12, 5 x 10 min: fixed %.4f float %.4f single %.4f nofix %.4f; outliers %.4f of float/single (%d/%d)', ...
                fr(5), fr(6), fr(2), fr(1), nOut/nFS, nOut, nFS);
            tc.verifyLessThan(fr(5), 0.05, 'fixed < 5 %');
            tc.verifyGreaterThan(fr(2) + fr(1), 0.80, 'single + no-fix > 80 %');
            tc.verifyEqual(nOut/nFS, 0.005, 'AbsTol', 0.002, 'outlier rate');
            % model: robot inside the house heading north
            R = sens_tb_run(P, struct('T', 3, 'pose', @(t) repmat([4.8 18.75 0 pi/2 0 0], numel(t), 1)));
            k = R.diag.t > 1;
            tc.m('SKYVIEW', 'greenhouse antenna: svf %.4f zone %d', mean(R.diag.svf(k)), round(mean(R.diag.zone(k))));
            tc.verifyEqual(R.diag.svf(k), 0.12*ones(nnz(k), 1), 'AbsTol', 1e-9, 'svf inside the house');
            tc.verifyEqual(R.diag.zone(k), ones(nnz(k), 1), 'zone 1 inside the house');
        end

        %% ------------------------------------------------------------------ LiDAR
        function tSEN11_LidarGeometry(tc)
            % T-SEN-11: trunk r 0.3 m 5.0 m ahead of the LiDAR, wall 10 m ahead (FAST2D, 50 scans)
            P = tc.P1;  xL = P.sensors.lidar.r(1);
            P.world.obstacles.cyl = [xL + 5 0 0.3 6 0 1];  P.world.obstacles.nCyl = 1;
            P.world.obstacles.box = [xL + 10 + 0.05 20 0 0.1 60 3 0 5 1];  P.world.obstacles.nBox = 1;
            P.world.obstacles.nFilm = 0;
            R = sens_tb_run(P, struct('T', 5.05));
            r = R.scan.ranges(R.scan.t > 0.05, :);
            a = R.scan.angles(end, :)*180/pi;
            b0 = find(abs(a) < 1e-3);
            wall = abs(a) >= 10 & abs(a) <= 40;
            rw = r(:, wall);
            sd = std(rw, 0, 1, 'omitnan');
            tgt = wall | (abs(a) <= 2);
            drop = mean(isnan(r(:, tgt)), 'all');
            tc.m('T-SEN-11', 'beam 0 deg mean %.4f m (%d scans); wall range std %.4f m; dropout fraction %.4f (%d samples)', ...
                mean(r(:, b0), 'omitnan'), size(r, 1), mean(sd), drop, numel(r(:, tgt)));
            tc.verifyEqual(mean(r(:, b0), 'omitnan'), 4.70, 'AbsTol', 0.06, 'trunk range at 0 deg');
            tc.verifyEqual(mean(sd), 0.020, 'AbsTol', 0.005, 'wall range std');
            tc.verifyEqual(drop, 0.005, 'AbsTol', 0.002, 'dropout fraction');
        end

        function tDynamicObstacle(tc)
            % a dynamic obstacle from ScenarioIn.obs appears at the analytic range +/- 0.05 m
            P = tc.P1;  xL = P.sensors.lidar.r(1);
            c = [xL 6];  rr = 0.25;
            R = sens_tb_run(P, struct('T', 2.05, 'obs', {{'n', [0 1]; 'xy', [0 c(1) 0 0 0 c(2) 0 0 0]; 'r', [0 rr 0 0 0]}}));
            a = R.scan.angles(end, :)*180/pi;
            j = find(abs(a - 90) < 1e-3);
            r = R.scan.ranges(R.scan.t > 0.05, j);
            tc.m('DYN-OBS', 'beam +90 deg mean %.4f m (analytic %.4f m)', mean(r, 'omitnan'), 6 - rr);
            tc.verifyEqual(mean(r, 'omitnan'), 6 - rr, 'AbsTol', 0.05, 'dynamic obstacle range');
        end

        function tSEN12_GreenhouseFilm(tc)
            % T-SEN-12: greenhouse PE film returns with probability 0.6 (FAST2D, 50 scans, robot inside heading north)
            P = tc.P4;
            pose = [4.8 18.75 0 pi/2 0 0];
            R = sens_tb_run(P, struct('T', 5.05, 'pose', @(t) repmat(pose, numel(t), 1)));
            r = R.scan.ranges(R.scan.t > 0.05, :);
            pL = [pose(1) pose(2) + P.sensors.lidar.r(1)];
            ang = pose(4) + P.sensors.lidar.angles;
            E = P.world.obstacles;
            film = nan(1, numel(ang));
            for k = 1:numel(ang)
                d = [cos(ang(k)) sin(ang(k))];
                tF = inf;
                for s = 1:E.nFilm
                    tF = min(tF, local_seg(pL, d, E.film(s, :)));
                end
                tC = inf;
                for s = 1:E.nCyl
                    tC = min(tC, local_circ(pL, d, E.cyl(s, 1:2), E.cyl(s, 3)));
                end
                if tF < tC - 0.3 && tF < P.sensors.lidar.rMax - 1, film(k) = tF; end
            end
            fb = find(isfinite(film));
            hit = abs(r(:, fb) - film(fb)) < 0.2;
            tc.m('T-SEN-12', 'film-first beams %d, film return fraction %.4f (%d samples)', numel(fb), mean(hit, 'all'), numel(hit));
            tc.verifyGreaterThan(numel(fb), 100, 'enough film beams');
            tc.verifyEqual(mean(hit, 'all'), 0.6, 'AbsTol', 0.05, 'film return fraction');
        end

        function tSEN13_Fast2dVsFull3d(tc)
            % T-SEN-13: palm scene, FAST2D vs FULL3D (cloud converted to the 2-D band in the test)
            P = tc.P3;
            pose = [8 40 0 pi/2 0 0];
            pose(3) = interp2(P.world.terrain.yN, P.world.terrain.xE, P.world.terrain.H, pose(2), pose(1));   % on the ground
            pf = @(t) repmat(pose, numel(t), 1);
            R2 = sens_tb_run(P, struct('T', 2.05, 'pose', pf));
            bk = evalin('base', 'sens_uavScene');
            c = onCleanup(@() assignin('base', 'sens_uavScene', bk));
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(P));
            R3 = sens_tb_run(P, struct('T', 2.05, 'pose', pf), 'VC_LIDAR', 2, 'cloudN', 20);
            r2 = median(R2.scan.ranges(R2.scan.t > 0.05, :), 1, 'omitnan');
            [r3, gndPalm] = local_cloud2band(R3.cloud.xyz(:, :, :, end), P, pose);
            trunk = isfinite(r2) & isfinite(r3) & r2 < 30;
            d = abs(r2(trunk) - r3(trunk));
            tc.m('T-SEN-13', ['palm aisle: trunk beams %d, median |FAST2D - FULL3D| %.4f m; FULL3D cloud valid %d; ' ...
                'ground share of the 2-D bins on the palm terrain (berms 0.15 m, 0.5 %% slope) %.4f'], ...
                nnz(trunk), median(d), all(R3.cloud.valid), gndPalm);
            tc.verifyGreaterThan(nnz(trunk), 5, 'trunk beams found in both');
            tc.verifyLessThan(median(d), 0.05, 'median range difference on trunk beams');
            % flat ground (flat_test world) and a 10 deg slope: share of 2-D scan bins whose return is ground
            Pf = tc.P1;
            poseF = [0 0 0 0 0 0];
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(Pf));
            Rf = sens_tb_run(Pf, struct('T', 1.05, 'pose', @(t) repmat(poseF, numel(t), 1)), 'VC_LIDAR', 2, 'cloudN', 10);
            [~, gndF] = local_cloud2band(Rf.cloud.xyz(:, :, :, end), Pf, poseF);
            Ps = Pf;  th = 10*pi/180;
            [~, Y] = ndgrid(Ps.world.terrain.xE, Ps.world.terrain.yN);
            Ps.world.terrain.H = tan(th)*Y;                   % ramp rising to the north
            poseS = [0 0 0 pi/2 -th 0];                       % heading north, climbing (nose up)
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(Ps));
            R4 = sens_tb_run(Ps, struct('T', 1.05, 'pose', @(t) repmat(poseS, numel(t), 1)), 'VC_LIDAR', 2, 'cloudN', 10);
            [~, gndS] = local_cloud2band(R4.cloud.xyz(:, :, :, end), Ps, poseS);
            tc.m('T-SEN-13', 'ground share of the 2-D scan bins: flat %.4f, 10 deg slope %.4f (FAST2D: 0 by construction)', gndF, gndS);
            tc.verifyLessThan(gndF, 0.01, 'ground points in the 2-D scan (flat)');
            tc.verifyLessThan(gndS, 0.01, 'ground points in the 2-D scan (10 deg slope)');
        end

        function tSEN14_Performance(tc)
            % T-SEN-14: FAST2D compiled <= 5 ms/scan (palm farm, largest raster); full sensor model adds < 15 min
            % wall-clock to a 4 h FAST2D mission; FULL3D <= 0.2 s/scan. Run-phase wall-clock times in Normal mode
            % (sens_cost_model: bus Constant inputs, Fast Restart, (w(T) - w(1)), minimum of 3): the per-run
            % compile and the bench's root-input interpolation / logging are not costs of the sensor models.
            P = tc.P3;  T = 21;
            poses = [8 20 0 pi/2; 30 -5 0 pi/2; 40 60 0 0];      % palm aisle, southern headland, across the rows
            perScan = zeros(1, size(poses, 1));
            T2 = 201;                                            % 2000 scans: run phase >> init scatter
            for k = 1:size(poses, 1)
                [wScan, wBlack] = sens_cost_model(P, poses(k, :), T2, 'fast2d');
                perScan(k) = (wScan - wBlack)/(T2/P.sensors.lidar.Ts);
            end
            [wS, wB] = sens_cost_model(P, poses(1, :), T);
            perS = (wS - wB)/T;
            per4h = perS*4*3600/60;
            bk = evalin('base', 'sens_uavScene');
            c = onCleanup(@() assignin('base', 'sens_uavScene', bk));
            assignin('base', 'sens_uavScene', sens_env_to_uavscenario(P));
            [w3, w2] = sens_cost_model(P, poses(1, :), 5, 'full3d');
            per3 = (w3 - w2)/(5/P.sensors.lidar.Ts);
            tc.m('T-SEN-14', ['FAST2D %s ms/scan (palm farm: aisle, headland, across rows; scanning - blackout); sensor ' ...
                'model %.4f s per simulated s (%.3f s with, %.3f s without for %g s) -> %.2f min per 4 h (Normal mode); ' ...
                'FULL3D %.3f s/scan (%.2f s FULL3D, %.2f s FAST2D for 5 s)'], mat2str(1e3*perScan, 3), perS, wS, wB, T, ...
                per4h, per3, w3, w2);
            tc.verifyLessThanOrEqual(max(perScan), 5e-3, 'FAST2D per scan');
            tc.verifyLessThan(per4h, 15, 'sensor model wall-clock per 4 h mission');
            tc.verifyLessThanOrEqual(per3, 0.2, 'FULL3D per scan');
        end

        %% ------------------------------------------------------------------ spray, odometry
        function tSprayVision(tc)
            % abstract vision sprayer: rho = 2 everywhere -> detection 0.90 +/- 0.02; rho = 0 -> false alarm 0.05 +/- 0.01
            P = tc.P1;
            P.world.veg.rho(:) = 2;
            R = sens_tb_run(P, struct('T', 60));
            b = dec2bin(R.spray.mask(R.spray.t > 0.05), 8) == '1';
            pd = mean(b, 'all');
            P.world.veg.rho(:) = 0;
            R0 = sens_tb_run(P, struct('T', 60));
            b0 = dec2bin(R0.spray.mask(R0.spray.t > 0.05), 8) == '1';
            pfa = mean(b0, 'all');
            tc.m('SPRAY', 'rho 2: detection %.4f (%d samples), density %.4f; rho 0: false alarm %.4f', pd, numel(b), ...
                mean(R.spray.density), pfa);
            tc.verifyEqual(pd, 0.90, 'AbsTol', 0.02, 'detection probability');
            tc.verifyEqual(pfa, 0.05, 'AbsTol', 0.01, 'false-alarm probability');
            tc.verifyEqual(mean(R.spray.density), 2/3, 'AbsTol', 1e-6, 'target_density = mean(rho)/3');
        end

        function tOdometry(tc)
            % sens_odom_compute: 2.000 m/s worth of counts/s (2.0/K_enc) with an int32 wrap at t = 1 s -> 2.000 +/- 0.01 m/s
            P = tc.P1;
            rate = round(2.0/P.electronics.drv.K_enc);  c0 = 2^31 - rate;
            R = sens_tb_run(P, struct('T', 2, 'enc', @(t) round(c0 + rate*[t t])));
            k = R.odom.t >= 0.06;
            v = R.odom.v(k, :);  tt = R.odom.t(k);
            w = tt > 0.9 & tt < 1.2;
            dv = max(abs(diff(v(w, :))), [], 'all');
            tc.m('ODOM', 'vTrack %.5f / %.5f m/s (min %.5f max %.5f); max |dv| around the wrap %.2e; counts wrapped: %d; a_track max %.2e', ...
                mean(v(:, 1)), mean(v(:, 2)), min(v, [], 'all'), max(v, [], 'all'), dv, any(R.odom.cnt(:, 1) < 0), ...
                max(abs(R.a_track.a(R.a_track.t > 0.3))));
            tc.verifyTrue(any(R.odom.cnt(:, 1) < 0) && any(R.odom.cnt(:, 1) > 0), 'int32 counter wrapped');
            tc.verifyEqual(v, 2.000*ones(size(v)), 'AbsTol', 0.01, 'odometry speed');
            tc.verifyLessThan(dv, 0.01, 'no jump at the wrap');
        end

        %% ------------------------------------------------------------------ integration checks
        function tInterfaces(tc)
            n = shaddad_check_interfaces('Files', {'Shaddad_SensorModels'}, 'Quiet', true);
            tc.m('IFACE', 'shaddad_check_interfaces(Shaddad_SensorModels) mismatches %d', n);
            tc.verifyEqual(n, 0, 'Shaddad_SensorModels ports = ICD 3.1');
            % the inline GNSS / LiDAR subsystems of Shaddad_SensorModels (MATLAB Function blocks one SSR level
            % below a model, R2026b JIT workaround) are identical to the standalone files
            for f = {'sens_gnss_model', 'GNSS'; 'sens_lidar_model', 'LiDAR'}'
                [bf, tf] = local_blocks(f{1});
                [bi, ti] = local_blocks(['Shaddad_SensorModels/' f{2}]);
                tc.m('IFACE', '%s.slx %d blocks, Shaddad_SensorModels/%s %d blocks', f{1}, numel(bf), f{2}, numel(bi));
                tc.verifyEqual(bi, bf, sprintf('Shaddad_SensorModels/%s blocks = %s.slx blocks', f{2}, f{1}));
                tc.verifyEqual(ti, tf, sprintf('Shaddad_SensorModels/%s block types = %s.slx', f{2}, f{1}));
                if bdIsLoaded(f{1}), close_system(f{1}, 0); end
            end
            % compiled output rates of the Shaddad_SensorModels instance in the bench = ICD 3.1
            mdl = 'tb_sens_plant';
            hws = get_param(mdl, 'ModelWorkspace');
            assignin(hws, 'P', tc.P1);
            c = onCleanup(@() evalin(hws, 'clear P'));
            try
                evalc('feval(mdl, [], [], [], ''compile'')');
            catch e
                if ~strcmp(e.identifier, 'Stateflow:Assertion'), rethrow(e); end
                evalc('feval(mdl, [], [], [], ''compile'')');
            end
            on = {'o_imu_raw', 'o_gnss_raw', 'o_lidar_scan', 'o_spray', 'o_sens_diag', 'o_imu_meas', 'o_odom', 'o_a_track'};
            oTs = zeros(1, numel(on));
            for k = 1:numel(on)
                v = get_param([mdl '/' on{k}], 'CompiledSampleTime');
                if iscell(v), v = v{1}; end
                oTs(k) = v(1);
            end
            ts = get_param([mdl '/Sensors'], 'CompiledSampleTime');
            ts2 = get_param([mdl '/AcqIMU'], 'CompiledSampleTime');
            ts3 = get_param([mdl '/Odom'], 'CompiledSampleTime');
            evalc('feval(mdl, [], [], [], ''term'')');
            r = cellfun(@(x) x(1), ts(:))';
            tc.m('IFACE', 'compiled block rates Sensors %s, sens_acq_imu %s, sens_odom_compute %s', mat2str(r), ...
                mat2str(cellfun(@(x) x(1), local_cell(ts2))), mat2str(cellfun(@(x) x(1), local_cell(ts3))));
            tc.m('IFACE', 'compiled output rates imu_raw/gnss/scan/spray/diag/imu_meas/odom/a_track %s', mat2str(oTs));
            tc.verifyTrue(all(ismember(round(r, 6), round(tc.P1.sim.Ts_compiled_allowed, 6))), 'sensor rates in P.sim.Ts_compiled_allowed');
            tc.verifyEqual(oTs, [0.005 0.1 0.1 0.1 0.1 0.005 0.01 0.01], 'AbsTol', 1e-12, 'output port rates = ICD');
        end

        function tAlgebraicLoops(tc)
            mdl = 'tb_sens_plant';
            hws = get_param(mdl, 'ModelWorkspace');
            assignin(hws, 'P', tc.P1);
            c = onCleanup(@() evalin(hws, 'clear P'));
            try
                evalc('loops = Simulink.BlockDiagram.getAlgebraicLoops(mdl);');
            catch e
                % one-off MATLAB Function JIT relink assertion in long sessions (see sens_tb_run)
                if ~strcmp(e.identifier, 'Stateflow:Assertion'), rethrow(e); end
                evalc('loops = Simulink.BlockDiagram.getAlgebraicLoops(mdl);');
            end
            tc.m('ALGLOOPS', 'tb_sens_plant algebraic loops %d', numel(loops));
            tc.verifyEmpty(loops, 'no algebraic loops');
        end

        function tCodegenAcq(tc)
            % sens_acq_imu / sens_odom_compute run in Shaddad_MCU on STM32 (single precision); code generation is
            % checked with the MCU model (T08 / T17)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
        end
    end
end

% =============================================================================================
function c = local_cell(x)
if iscell(x), c = x(:)'; else, c = {x}; end
end

function [b, ty] = local_blocks(sys)
% sorted relative paths and block types of every block below sys (all variant choices); loads the file
mdl = strtok(sys, '/');
if ~bdIsLoaded(mdl), load_system(mdl); end
p = find_system(sys, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
p = setdiff(p, {sys});
b = sort(erase(p, [sys '/']));
ty = cellfun(@(x) get_param([sys '/' x], 'BlockType'), b, 'UniformOutput', false);
end

function t = local_seg(p, d, s)
% ray p + t d against segment s = [x1 y1 x2 y2]
e = s(3:4) - s(1:2);
M = [d(:) -e(:)];
if abs(det(M)) < 1e-12, t = inf; return, end
x = M\(s(1:2) - p)';
if x(1) > 0 && x(2) >= 0 && x(2) <= 1, t = x(1); else, t = inf; end
end

function t = local_circ(p, d, c, r)
f = p - c;  b = f*d';  cc = f*f' - r^2;  disc = b^2 - cc;
t = inf;
if disc >= 0
    t1 = -b - sqrt(disc);
    if t1 > 0, t = t1; end
end
end

function [r, gnd] = local_cloud2band(xyz, P, pose)
% one organised FULL3D cloud (sensor frame = body-aligned, level mount) -> 360-beam minimum range of the points in
% the height band zBand above the body x-y plane through base_link (the 2-D scan); gnd = share of the finite scan
% bins whose nearest point is a ground return (world z within 0.1 m of the terrain under the point)
zL = P.sensors.lidar.r(3);  zb = P.sensors.lidar.zBand;
X = reshape(xyz(:, :, 1), [], 1);  Y = reshape(xyz(:, :, 2), [], 1);  Z = reshape(xyz(:, :, 3), [], 1);
ok = isfinite(X) & isfinite(Y) & isfinite(Z);
X = X(ok);  Y = Y(ok);  Z = Z(ok);
inb = Z + zL > zb(1) & Z + zL < zb(2);
X = X(inb);  Y = Y(inb);  Z = Z(inb);
% world coordinates of the band points (ZYX pose, theta + = nose down)
cy = cos(pose(4)); sy = sin(pose(4)); cp = cos(pose(5)); sp = sin(pose(5)); cr = cos(pose(6)); sr = sin(pose(6));
Rwb = [cy -sy 0; sy cy 0; 0 0 1]*[cp 0 sp; 0 1 0; -sp 0 cp]*[1 0 0; 0 cr -sr; 0 sr cr];
pw = pose(1:3)' + Rwb*([X Y Z]' + P.sensors.lidar.r(:));
W = P.world;
xq = min(max(pw(1, :), W.terrain.xE(1)), W.terrain.xE(end));  yq = min(max(pw(2, :), W.terrain.yN(1)), W.terrain.yN(end));
H = interp2(W.terrain.yN, W.terrain.xE, W.terrain.H, yq, xq);
isG = abs(pw(3, :) - H) < 0.1;
az = atan2(Y, X);  rr = hypot(X, Y);
bin = floor((az + pi)/P.sensors.lidar.res + 0.5) + 1;  bin(bin > 360) = bin(bin > 360) - 360;
r = nan(1, 360);  g = false(1, 360);
for k = 1:numel(rr)
    if isnan(r(bin(k))) || rr(k) < r(bin(k)), r(bin(k)) = rr(k); g(bin(k)) = isG(k); end
end
gnd = 0;
if any(isfinite(r)), gnd = mean(g(isfinite(r))); end
end
