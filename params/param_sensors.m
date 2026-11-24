function Sn = param_sensors(P)
%PARAM_SENSORS  Sensor models, acquisition, navigation fusion, perception (owner: sensors).
%   Sn = param_sensors(P)   P: partially assembled (needs P.world, P.frames, P.structure, P.drive)
%   Mount positions are in body FLU w.r.t. base_link (= structure reference point R).
%   Variant control derived by shaddad_params: VC_LIDAR (1 FAST2D | 2 FULL3D).
%   Revision B: ICM-20948 datasheet temperature coefficients (D49), slip states held during GNSS /
%   scan-match denial (D49), chi prior from P.structure.track.chi_nom (D39), scan-matching window
%   10 m / 10 cells/m (D55, measured 29 ms per call interpreted).
%   Build task T11 additions (sensor plant models, sens_acq_imu, sens_odom_compute): imu.vib.bpRms/gainA/gainW/
%   seedA/seedW, imu.R_BI (true misalignment), imu.gcorr_W (mspsensorlib/IMU uses g = 9.81 internally),
%   imu.fault; gnss.dt/svfClamp/acc*/repClamp/numSV/head_sigma/headAccInvalid_deg/seedU/seedHead/seedJit/svfOutlier;
%   lidar.angles/rasterRes/filmBoxClass/tiltedBoxClass/rangeNoiseK/pDropDust/dustRange/dustRangeLoss/
%   seedNR/seedDrop/seedDust/seedFilm/full3d.nEl/nAz; spray.pts_B/seeds/rhoMax; acq (MCU IMU acquisition).
%   Build task T12 additions (Shaddad_NavFusion): ekf.u0 (16-element state-transition input incl. s0L/s0R and the
%   clamps), ekf.Qd_denied, ekf.odomDecim/headDecim, ekf.tauSlipHold, navgate (sens_nav_gate incl. kRpos / nBypass),
%   navmode (sens_nav_mode), nav.vStill/wStill/tauMax/tauMaxSm/imufQ0/tauAcc, acq_gnss (HW GNSS acquisition).
%   Values that world_gnss_grids('defaults') / world_occupancy('defaults') copy (gnss.svf, lidar.zBand,
%   lidar.mapRes) are unchanged (tests/structure/t_world asserts equality).
%   Build task T13 additions (Shaddad_Perception): per.dustR/dustDr/secIdx/secLen/protXF/protXR/minPtsFarRange/
%   compactR/nClMax/rowSeed/smNMin/smCovFloor/priorMapSeed/cloudMinPts/cloudLevel/hwNAz; per.rowXWin [-4 20].
%   S4 nav fix round 2 (2026-11-14): gnss.seedVel (zero-mean GNSS velocity noise of sens_gnss_model), navgate.kRvel
%   (GNSS velocity R inflation without RTK), navgate.smWeakRatio/smCap (scan-match start guard), per.smGeoGap/
%   smGeoDmax (weak axis of the scan-match covariance from the scan geometry).
%   Critical fixes (critfix, 2026-11-15): per.protXF / protXR from the fitted implement (P.structure.impl with the
%   setting P.structure.cfg.implement = VC_IMPL), per.protVBoth and per.protNStale (Shaddad_Perception ProtField: both
%   fields at standstill, stop on a lost or stale scan); navmode.tImuLossInv / tImuLossFault (sens_nav_mode, IMU loss).
g = P.frames.g;

%% ---- general ----
Sn.refLLA = P.world.lla0;                    % single datum (P.world)
Sn.g = g;
Sn.magDecl = P.frames.mag_dec_deg;
Sn.geom.B = P.structure.geom.B;

%% ---- IMU (ICM-20948 on the MCU board in the electronics bay, 200 Hz) ----
imu.Ts = 0.005;
imu.r  = [0.25 0.00 0.40];                   % FLU w.r.t. base_link (MCU PCB, electronics bay) [EST, real robot]
imu.eulMis_deg = [0.3 -0.5 0.4];             % true residual misalignment (firmware assumes identity)
imu.magENU = P.frames.mag_ENU_uT;
imu.seed = P.sim.seed_base + P.sim.seed.imu;
imu.g_correction = P.frames.imu_g_internal - g;   % add [0 0 -corr] to the block linear-accel input if needed
imu.accel = struct('MeasurementRange',39.2266,'Resolution',1.1971e-3,'ConstantBias',[0.05 -0.04 0.08], ...
    'AxesMisalignment',[100 0.5 0.5; 0.5 100 0.5; 0.5 0.5 100],'NoiseDensity',2.2555e-3*[1 1 1], ...
    'BiasInstability',4.9e-4,'RandomWalk',2.0e-4,'TemperatureBias',7.8e-3*[1 1 1], ...   % +/-0.80 mg/C (DS-000189)
    'TemperatureScaleFactor',0.02*[1 1 1]);
imu.gyro = struct('MeasurementRange',8.7266,'Resolution',2.6632e-4,'ConstantBias',[0.0061 -0.0073 0.0049], ...
    'AxesMisalignment',[100 0.5 0.5; 0.5 100 0.5; 0.5 0.5 100],'AccelerationBias',1.78e-4*[1 1 1], ...
    'NoiseDensity',2.618e-4*[1 1 1],'BiasInstability',4.85e-5,'RandomWalk',2.0e-5, ...
    'TemperatureBias',8.7e-4*[1 1 1],'TemperatureScaleFactor',0.02*[1 1 1]);   % +/-0.05 dps/C (DS-000189)
imu.mag = struct('MeasurementRange',4900,'Resolution',0.15,'ConstantBias',[1.5 -2.0 1.0], ...
    'NoiseDensity',0.085*[1 1 1],'BiasInstability',0.1,'TemperatureBias',0.02*[1 1 1]);
imu.T_default_degC = 45;
imu.kMagI = [0.04 0.02 0.05];                % uT/A traction-cable disturbance
fs = 200; [bb, aa] = local_bp2(10, 60, fs);
imu.vib = struct('sigA',[0.6 0.5 1.2],'sigW',[0.03 0.03 0.01],'bpNum',bb,'bpDen',aa, ...
                 'cov',1/fs, 'seeds',401:406);
a = exp(-2*pi*50/200);
imu.dlpf = struct('num',1 - a,'den',[1 -a]);
% --- T11 build additions (sens_imu_model) ---
% BLWN (Cov = Ts) gives unit-variance white noise; the band-pass output is scaled to sigA / sigW rms
imu.vib.bpRms = local_rms_impulse(bb, aa);                 % rms of the band-pass output for unit white noise
imu.vib.gainA = imu.vib.sigA / imu.vib.bpRms;              % m/s^2 per (m/s) per unit noise
imu.vib.gainW = imu.vib.sigW / imu.vib.bpRms;              % rad/s per (m/s) per unit noise
imu.vib.seedA = imu.vib.seeds(1:3); imu.vib.seedW = imu.vib.seeds(4:6);
% true mounting misalignment sensor -> body (v_B = R_BI v_I), ZYX [yaw pitch roll], pitch + = nose down
imu.R_BI = local_eul2dcm(imu.eulMis_deg*pi/180);
% mspsensorlib/IMU uses gravity 9.81 internally (verified: ENU level at rest reads [0 0 -9.81]);
% adding gcorr_W to its linear-acceleration input makes the (sign-flipped) reading equal P.frames.g
imu.gcorr_W = [0 0 -imu.g_correction];
imu.fault = struct('dAcc',[0.5 0.5 0.5],'dGyro',[0.05 0.05 0.05]);   % imu_fault 2: bias step on every axis
Sn.imu = imu;

%% ---- GNSS (dual ZED-F9P moving base, 10 Hz) ----
gn.Ts = 0.1; gn.latency = 0.08; gn.latencySteps = 8; gn.delayTs = 0.01;
gn.rAnt1 = [-0.40 0 1.05]; gn.rAnt2 = [0.40 0 1.05]; gn.psiBaseline = 0;
gn.sigH = [0.015 0.30 1.20]; gn.sigV = [0.030 0.60 2.50]; gn.sigVel = [0.02 0.03 0.05];
gn.decay = [0.90 0.995 0.998];               % FIXED / FLOAT / SINGLE
gn.seeds = P.sim.seed_base + P.sim.seed.gnss*100 + [1 2 3];
gn.markov = struct('l_fx_fl0',0.002,'l_fx_fl1',0.50,'l_fl_fx',0.20,'l_fl_sg',0.30,'l_sg_fl',0.50, ...
    'l_nofix',2.0,'svf_nofix',0.25,'l_nf_sg0',0.05,'l_nf_sg1',0.5,'h_lose0',0.002,'h_lose1',0.30,'h_gain',0.50);
gn.pOutlier = 0.005; gn.outlierMag = [2 5]; gn.head_sigma_deg = 0.5;
gn.corrAgeFixLimit = 10; gn.corrAgeFloatLimit = 30; gn.initFix = 0; gn.repJitter = 0.15;
gn.svf = struct('open',1.0,'greenhouse',0.12,'underPanel',0.45,'betweenPanels',0.75,'canopyAtt',0.45, ...
                'svf_min',0.02);
% --- T11 build additions (sens_gnss_model, sens_gnss_fixchain) ---
gn.dt = gn.Ts;                                % fix-chain step
gn.svfClamp = [gn.svf.svf_min 1];
% reported accuracies [hAcc vAcc sAcc] per fix code (rows FIXED 4, FLOAT 5, SINGLE 1, NOFIX 0)
gn.accFixed = [gn.sigH(1) gn.sigV(1) gn.sigVel(1)];
gn.accFloat = [gn.sigH(2) gn.sigV(2) gn.sigVel(2)];
gn.accSingle = [gn.sigH(3) gn.sigV(3) gn.sigVel(3)];
gn.accNoFix = [99 99 9.9];
gn.repClamp = [0.7 1.5];                      % clamp(1 + repJitter n, 0.7, 1.5)
gn.numSV = [6 22];                            % numSV = round(6 + 22 SVF)
gn.head_sigma = gn.head_sigma_deg*pi/180;
gn.headAccInvalid_deg = 180;
gn.seedU = P.sim.seed_base + P.sim.seed.gnss*100 + (4:9);   % fix chain u(1:4), outlier magnitude / direction
gn.seedHead = P.sim.seed_base + P.sim.seed.gnss*100 + 10;
gn.seedJit = P.sim.seed_base + P.sim.seed.gnss*100 + 11;
gn.svfOutlier = 0.7;                          % multipath outliers only below this sky-view factor (spec 3.2)
% zero-mean velocity noise per axis E N U, sigma sigVel(fix), added in sens_gnss_model (VelNoise): the GPS blocks
% run with VelocityAccuracy 0, because their own velocity noise reads 1.2533 sigVel high along the track (S4 nav
% fix round 2, 2026-11-14)
gn.seedVel = P.sim.seed_base + P.sim.seed.gnss*100 + (12:14);
Sn.gnss = gn;

%% ---- odometry (computed in the MCU from ESC FB3 encoder counts) ----
Sn.odom = struct('Ts',0.01,'window',0.05,'CPR',P.drive.esc.enc_cpr,'Ng',P.drive.gear.N, ...
                 'rs',P.structure.r_sprocket,'sig0',0.01,'sig1',0.03);

%% ---- LiDAR (Ouster OS1-32 class) ----
li.Ts = 0.1; li.r = [0.45 0 0.75]; li.eul_deg = [0 0 0];   % [EST] front of the 1.0 m body, above the 0.57 m deck
li.nBeams = 360; li.res = pi/180; li.rMin = 0.5; li.rMax = 40; li.sigR = 0.02;
li.zBand = [0.15 1.20]; li.pDrop = 0.005; li.pDustMax = 0.02; li.pRetFilm = 0.6;
li.mapRes = 10; li.mapResGreenhouse = 20;
li.maxCircles = 1500; li.maxSegments = 300; li.maxBoxes = 300; li.maxDyn = 4;
li.full3d = struct('MaxRange',45,'RangeAccuracy',0.015,'AzimuthLimits',[-180 180],'AzimuthResolution',1.0, ...
                   'ElevationLimits',[-22.5 22.5],'ElevationResolution',1.45);
li.cloudSize = [32 1024];
li.seeds = P.sim.seed_base + P.sim.seed.lidar2d*100 + [1 2 3 4];
% --- T11 build additions (sens_lidar_model, sens_lidar2d_fast) ---
li.angles = -pi + (0:li.nBeams-1)*li.res;     % beam k at -pi + (k-1) deg (Bus_LidarScanRaw)
li.rasterRes = li.mapRes;                     % FAST2D raster of the active world (cells/m)
if P.world.id == 4, li.rasterRes = li.mapResGreenhouse; end   % thin greenhouse posts (r 0.05 m)
li.filmBoxClass = 4;                          % greenhouse wall boxes = the PE film (film layer, not solid)
li.tiltedBoxClass = 6;                        % solar tables: only the in-band strip at the low (south) edge
li.rangeNoiseK = 0.005;                       % sigma_r = sigR (1 + 0.005 r)
li.pDropDust = 0.03; li.dustRange = [0.3 3.0]; li.dustRangeLoss = 0.5;
sb = mod(P.sim.seed_base, 100000)*1000 + P.sim.seed.lidar2d*100000;   % per-beam seeds < 2^32
li.seedNR   = sb + 10000 + (0:li.nBeams-1);   % range noise, dropout, dust, film draws
li.seedDrop = sb + 20000 + (0:li.nBeams-1);
li.seedDust = sb + 30000 + (0:li.nBeams-1);
li.seedFilm = sb + 40000 + (0:li.nBeams-1);
% FULL3D (uavsimlib/UAV Scenario Lidar): organised output size [nEl nAz] (verified: 31 x 360 x 3)
li.full3d.nEl = floor((li.full3d.ElevationLimits(2) - li.full3d.ElevationLimits(1))/li.full3d.ElevationResolution + 1e-6);
li.full3d.nAz = floor((li.full3d.AzimuthLimits(2) - li.full3d.AzimuthLimits(1))/li.full3d.AzimuthResolution + 1e-6);
Sn.lidar = li;

%% ---- abstract vision-sprayer detection ----
Sn.spray = struct('Ts',0.1,'nSect',8,'boom_w',1.2,'x_boom',-0.6,'p_d',0.90,'p_fa',0.05, ...
                  'rho_thr',0.5,'seed',P.sim.seed_base + P.sim.seed.spray);
% section points in body FLU (row 1 x, row 2 y, row 3 z = 0): [x_boom, (i - 4.5)*boom_w/8], i = 1..8
Sn.spray.pts_B = [Sn.spray.x_boom*ones(1,8); ((1:8) - 4.5)*Sn.spray.boom_w/8; zeros(1,8)];
Sn.spray.seeds = Sn.spray.seed*10 + (0:7);
Sn.spray.rhoMax = 3;                          % target_density = mean(rho)/3

%% ---- MCU-side IMU acquisition (sens_acq_imu, Task_5, single precision) ----
Sn.acq = struct('R_SB',eye(3),'accOff',[0 0 0],'gyroOff',[0 0 0],'magOff',[0 0 0], ...
                'accNormRange',[5 15],'faultMask',64);    % fault_inject b6 = IMU invalid (MCU_In)

%% ---- navigation EKF (11 states) ----
qc = [4e-4 4e-4 2.5e-3 7e-7 2.5e-3 5e-3 1e-8 2e-6 6.4e-4 6.4e-4 1.5e-4];   % qc(7) bg, qc(8) ba raised for enclosure dT/dt (D49)
chi0 = P.structure.track.chi_nom;
ek.Ts = 0.01; ek.measTs = [0.1 0.1 0.1 0.05 0.2];
ek.x0 = [0 0 0 0 0 0 0 0 0.005 0.005 chi0]';
ek.P0 = diag([1e4 1e4 100 9.87 0.25 0.01 7.6e-5 0.04 0.01 0.01 0.09]);
ek.qc = qc; ek.Qd = diag(qc)*ek.Ts;
ek.tau_vy = 1.0; ek.tau_s = 20; ek.s0 = 0.005; ek.tau_chi = 300; ek.chi0 = chi0;
% D49: during GNSS + scan-match denial the slip states are held (random walk): tau_s -> tau_s_denied,
% prior mean -> last converged estimate (sample-and-hold at denial onset), slip process noise -> qc_slip_denied
ek.tau_s_denied = 1e6; ek.qc_slip_denied = 2e-6; ek.t_denied = 0.5;
ek.tauSlipHold = 2.0;                         % T12: low-pass time constant of the slip value held at denial onset [s]
ek.slip_clamp = [-0.5 0.6]; ek.chi_clamp = [1.0 2.5];
ek.kR = 1.5; ek.chi2 = [10.83 13.82 16.27]; ek.Nmax = [30 30 30 25 10]; ek.kForce = [25 25 25 100 10];
ek.od0 = 0.01; ek.od1 = 0.03;
ek.smFloor = diag([0.03^2 0.03^2 (0.5*pi/180)^2]); ek.smInfl = 2.0;
ek.R0_1 = 0.015^2*eye(3); ek.R0_2 = 0.02^2*eye(2); ek.R0_3 = (0.5*pi/180)^2;
ek.R0_4 = 0.02^2*eye(2);  ek.R0_5 = ek.smFloor;
Sn.ekf = ek;
Sn.nav = struct('sigPos',0.15,'sigYaw',2*pi/180,'tAbsMax',10,'drTimeout',2, ...
                'ecompass_sig',8*pi/180,'ecompass_iMax',5,'t_imu_valid',2);

% --- T12 build additions (Shaddad_NavFusion, sens_nav_gate, sens_nav_mode, sens_nav_ekf, sens_acq_gnss) ---
% state-transition input u (16x1) = [psidot_m fx sinTheta cosTheta dt g tau_vy tau_s s0L s0R tau_chi chi0
%                                    sMin sMax chiMin chiMax] (s0 split per track for the held prior, D49)
Sn.ekf.u0 = [0; 0; 0; 1; ek.Ts; g; ek.tau_vy; ek.tau_s; ek.s0; ek.s0; ek.tau_chi; chi0; ...
             ek.slip_clamp(1); ek.slip_clamp(2); ek.chi_clamp(1); ek.chi_clamp(2)];
qd = qc; qd(9:10) = ek.qc_slip_denied;
Sn.ekf.Qd_denied = diag(qd)*ek.Ts;           % slip process noise during GNSS + scan-match denial (D49)
Sn.ekf.odomDecim = round(ek.measTs(4)/ek.Ts);   % odometry update every 5th 0.01 s sample (D13)
Sn.ekf.headDecim = round(ek.measTs(3)/ek.Ts);   % ecompass heading update at 10 Hz
% sens_nav_gate parameters (non-tunable sub-struct of the MATLAB Function)
% kRpos = R inflation of the GNSS position per fix [SINGLE FLOAT FIXED]: FIXED / FLOAT / SINGLE errors are Gauss-Markov
% (decay 0.9 / 0.995 / 0.998 at 10 Hz: 1 / 20 / 50 s), so their epochs carry less information than white noise
% (FIXED 2.5 instead of kR = 1.5 keeps the 3-DOF NEES of T-NAV-02 in [1.5, 4.5]);
% nBypass = position epochs accepted without gating after a fix improvement (RTK re-fix)
% kRvel = R inflation of the GNSS velocity per fix [SINGLE FLOAT FIXED] (update; the gate keeps kR): without RTK,
% under the greenhouse film roof, the Doppler velocity is less certain than the reported sAcc (multipath, few
% satellites). Round 2 of the S4 nav fix (2026-11-14) set kR [3 2 1]; the lead's 100 s S4 test the same day showed that
% in the simulation, where the GNSS velocity is zero-mean since the plant fix, it cost accuracy (along-aisle error
% 0.095 against 0.065 m RMS, NEES 1.66 against 1.08, slip -0.019 against -0.010), so the simulation uses kR [1 1 1]. Keep
% the hook for the real robot [EST: set it from the receiver's Doppler in SINGLE / FLOAT under the film roof against a
% LiDAR or odometry reference]
% smWeakRatio / smCap = start guard on the weak translation axis of a scan match (an aisle): the larger axis is weak
% when its R5 variance is smWeakRatio x the smaller one or more; when the predicted pose is uncertain along it beyond
% both smCap and the match's own weak-axis sigma (about 0.24 m in S4) and the match moved less than its own sigma, the
% match may have followed its start, so it gets no more weight than the prior along that axis. Each firing halves the
% prior variance, so the guard delays such a match by a few scans (about 3 from a 0.6 m prior); it does not catch it
Sn.navgate = struct('kR', ek.kR, 'kRpos', [5 5 2.5], 'kRvel', ek.kR*[1 1 1], 'nBypass', 5, 'chi2', ek.chi2, 'Nmax', ek.Nmax, ...
    'kForce', ek.kForce, 'od0', ek.od0, 'od1', ek.od1, 'smFloor', diag(ek.smFloor)', 'smInfl', ek.smInfl, ...
    'smWeakRatio', 4, 'smCap', 0.15, 'x0', ek.x0, 'P0', ek.P0);
% sens_nav_mode parameters (non-tunable sub-struct of the MATLAB Function)
% IMU loss (critfix R-05, NAV-03): the IMU-invalid time counts in every state; after tImuLossInv [s] navValid is false
% and quality INVALID (20 IMU samples at 200 Hz); after tImuLossFault [s] in ALIGN, RUN_* or DEAD_RECKONING the mode
% goes to FAULT (EKF reset pulse), stays there while the IMU is invalid, and returns to INIT with a second reset pulse
% when it is valid again (re-alignment) [EST]
Sn.navmode = struct('dt', ek.Ts, 'sigPos', Sn.nav.sigPos, 'sigYaw', Sn.nav.sigYaw, 'tAbsMax', Sn.nav.tAbsMax, ...
    'drTimeout', Sn.nav.drTimeout, 'tImuValid', Sn.nav.t_imu_valid, 'tLidarSwitch', 5, 'tGnssReturn', 2, ...
    'priorMap', 1, 'nisK', 3, 'nisT', 5, 'nisTau', 2, 'tDenied', ek.t_denied, 'chi2', ek.chi2, ...
    'tLidarFresh', 1.0, 'm', [3 2 1 2 3], 'sigPosValid', 2.0, 'tImuLossInv', 0.1, 'tImuLossFault', 1.0);
Sn.nav.vStill = 0.02;                        % standstill: max |vTrack| [m/s] (ecompass gate, ALIGN)
Sn.nav.wStill = 0.02;                        % standstill: |gyro z| [rad/s]
Sn.nav.tauMax = 0.5;                         % GNSS epoch age t - t_meas used for latency compensation, saturation [s]
Sn.nav.tauMaxSm = 1.0;                       % scan-match age t - sm.t (channel 5 latency compensation), saturation [s]
% IMU Filter: initial error-state covariance [orientation(3) gyro bias(3) linear accel(3)]; the toolbox default
% orientation / bias variances (~1e-6 / 1e-4) make the filter integrate the 0.35-0.42 dps turn-on gyro bias for
% tens of seconds (T12 offline tuning: 3 deg tilt error at 10 s); 1e-2 rad^2 / 4e-3 (rad/s)^2 converge in < 10 s
% also with the 45 degC biases of the plant (about 0.025 rad/s)
Sn.nav.imufQ0 = diag([1e-2 1e-2 1e-2 4e-3 4e-3 4e-3 0.0096 0.0096 0.0096]);
% longitudinal kinematic acceleration for the tilt filter = d/dt mean track speed, first order tau_acc (as the
% MCU tilt filter, D47); with the centripetal term v*omega_z the IMU Filter sees gravity only
Sn.nav.tauAcc = 0.05;
% HW GNSS acquisition (sens_acq_gnss, ROS 2 ublox driver standard topics /fix, /fix_velocity, /navheading)
Sn.acq_gnss = struct('rtkFixedAcc', 0.05, 'headAccMax_deg', 5, 'Ts', gn.Ts);

%% ---- attitude filter (IMU Filter) ----
Sn.imuf = struct('AccelerometerNoise',0.05,'GyroscopeNoise',1e-5,'GyroscopeDriftNoise',1e-11, ...
                 'LinearAccelerationNoise',0.3,'LinearAccelerationDecayFactor',0.7);

%% ---- perception ----
Sn.per = struct('selfBox',[-0.55 0.55 -0.42 0.42],'mapSize',30,'mapRes',10,'occThr',0.65, ...
    'hit',0.7,'miss',0.4,'sat',[0.12 0.97],'maxRange',30,'segLambda',10*pi/180,'minPts',2,'maxObs',20, ...
    'maxRadius',2.0,'nSectors',8,'protL0',0.5,'protKv',0.8,'protW',1.08, ...
    'rowRRange',[0.05 0.5],'rowXWin',[-4 12],'rowYWin',[1 6],'rowInlier',0.25,'rowIter',50,'rowMinInl',3, ...
    'smTs',0.2,'smTrans',[0.3 0.3],'smRot',0.1745,'smRes',10,'smMinScoreRatio',0.5,'smMaxRange',10, ...   % D55
    'priorMapErr',0.03);
% --- T13 build additions (Shaddad_Perception, sens_scan_filter, sens_acq_lidar, F4 / F8-F11) ---
Sn.per.dustR = 3.0;                          % isolated near-return (dust) removal: r < dustR ...
Sn.per.dustDr = 0.3;                         % ... and both neighbours differ by > dustDr [m]
% 8 x 45 deg sectors, sector 1 centred forward, CCW: beam order for Reshape [45 8] (beam k at -pi + (k-1) deg)
k0 = find(abs(li.angles*180/pi + 22) < 1e-6, 1);        % first beam of sector 1 (-22 deg)
Sn.per.secIdx = mod((0:li.nBeams-1) + k0 - 1, li.nBeams) + 1;
Sn.per.secLen = li.nBeams/Sn.per.nSectors;   % 45 beams per sector
% protective field limits = the front / rear of the body or of the fitted implement, whichever reaches further (critfix
% R-03, NAV-02): the implement of the setting P.structure.cfg.implement (= VC_IMPL; every scenario builds P with its
% VC_IMPL), its box P.structure.impl.<name>.pos(1) +/- dims(1)/2 (x forward in the structure frame too, origin R =
% base_link): none 0.55 / -0.55 m, mower (deck front) 1.20 / -0.55 m, tiller 0.55 / -1.125 m, sprayer 0.55 / -0.55 m
% (the module box lies inside the body)
imp = P.structure.impl;
switch P.structure.cfg.implement
    case 1, xImp = imp.mower.pos(1) + [-1 1]*imp.mower.dims(1)/2;
    case 2, xImp = imp.tiller.pos(1) + [-1 1]*imp.tiller.dims(1)/2;
    case 3, xImp = imp.sprayer.pos_dry(1) + [-1 1]*imp.sprayer.dims_dry(1)/2;
    otherwise, xImp = [0 0];
end
Sn.per.protXF = max(Sn.per.selfBox(2), xImp(2));   % front field starts here ...
Sn.per.protXR = min(Sn.per.selfBox(1), xImp(1));   % ... the rear field here
Sn.per.protVBoth = 0.1;                      % [m/s] front field on for v > -protVBoth, rear field for v < protVBoth: both
                                             % at standstill and in pivots, where the sign of the EKF speed is noise [EST]
Sn.per.protNStale = 2;                       % [scans of 0.1 s] scan lost or stale (critfix R-02, NAV-01): on an invalid
                                             % scan the last valid field result is held for up to protNStale scans, then
                                             % protectiveStop until a valid scan (the last valid scan is then older than two
                                             % scan periods); on the robot a 0.1 s step without a new /ouster/points message
                                             % is invalid, so one missed message does not stop; stop from the start until
                                             % the first valid scan [EST]
% segmentation: beyond minPtsFarRange a 1-beam cluster is accepted (a 0.3 m box subtends < 2 beams beyond 8.6 m;
% T-PER-01 asks for detection within 20 m); compact clusters (radius <= compactR) are centred at
% (min range + radius) along the centroid bearing (visible-face bias of the plain centroid, T-PER-01 < 0.10 m)
Sn.per.minPtsFarRange = 8;
Sn.per.compactR = 0.6;
Sn.per.nClMax = 60;                          % internal cluster table (row detection input)
Sn.per.rowSeed = 12345;                      % RANSAC LCG seed (re-seeded every scan, deterministic)
% row window x [-4 12] holds only 2 trunks per side at the 8 m palm spacing (P.world), < rowMinInl 3: the window is
% extended to 24 m = 3 spacings (measured on the palm aisle: valid 0 % with [-4 12], 98 % with [-4 20])
Sn.per.rowXWin = [-4 20];
Sn.per.smNMin = 60;                          % min points of both scans inside [1/smRes, smMaxRange]
Sn.per.smCovFloor = [0.02^2 0.02^2 (0.3*pi/180)^2];     % diagonal floor of the scan-match covariance
Sn.per.priorMapSeed = P.sim.seed_base + 1301;           % as-built perturbation of the prior map
Sn.per.cloudMinPts = 50;                     % sens_lidar_cloud2scan: valid when > 50 band points
Sn.per.cloudLevel = 0;                       % 0 band in the body x-y plane (terrain-following, = FAST2D band
                                             % semantics); 1 levelled with the estimated roll / pitch
Sn.per.hwNAz = li.cloudSize(2);              % HW cloud columns (Ouster 1024)
% weak axis of the scan-match covariance from the scan geometry (S4 nav fix round 2, 2026-11-14; sens_scanmatch_loc):
% the point information of the matched scan against the reference scan; a reference point is on a line when both
% beam neighbours lie within smGeoGap (else isolated: post, edge, jamb), a current point corresponds within smGeoDmax
Sn.per.smGeoGap = 0.25;                      % [m] 2.5 cells at smRes 10
Sn.per.smGeoDmax = 0.30;                     % [m] 3 cells

%% ---- environment view for sensors (derived from P.world; build task T11 may extend) ----
W = P.world;
Sn.env = struct('nCyl',W.obstacles.nCyl,'cyl',W.obstacles.cyl,'nBox',W.obstacles.nBox,'box',W.obstacles.box, ...
    'nFilm',W.obstacles.nFilm,'film',W.obstacles.film,'nCanopy',W.nCanopy,'canopy',W.canopy, ...
    'nZones',W.gnssZones.n,'zoneX',W.gnssZones.X,'zoneY',W.gnssZones.Y,'zoneNv',W.gnssZones.nv, ...
    'zoneSvf',W.gnssZones.svf,'zoneType',W.gnssZones.type,'xlim',W.xlim,'ylim',W.ylim);

Sn.variant = struct('lidar',1);
end

function r = local_rms_impulse(b, a)
% rms of the output of filter(b, a) driven by unit-variance white noise = sqrt(sum(h.^2))
h = filter(b, a, [1 zeros(1, 4999)]);
r = sqrt(sum(h.^2));
end

function R = local_eul2dcm(e)
% ZYX Euler [yaw pitch roll] -> DCM (point rotation, v_B = R v_I); pitch about +y (FLU: + = nose down)
cy = cos(e(1)); sy = sin(e(1)); cp = cos(e(2)); sp = sin(e(2)); cr = cos(e(3)); sr = sin(e(3));
Rz = [cy -sy 0; sy cy 0; 0 0 1];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rx = [1 0 0; 0 cr -sr; 0 sr cr];
R = Rz*Ry*Rx;
end

function [b, a] = local_bp2(f1, f2, fs)
% 2nd-order digital band-pass (bilinear, pre-warped) without Signal Processing Toolbox calls
w1 = 2*fs*tan(pi*f1/fs);  w2 = 2*fs*tan(pi*f2/fs);
w0 = sqrt(w1*w2);  bw = w2 - w1;  K = 2*fs;
% H(s) = bw s / (s^2 + bw s + w0^2)
a0 = K^2 + bw*K + w0^2;
b = [bw*K 0 -bw*K]/a0;
a = [1 (2*w0^2 - 2*K^2)/a0 (K^2 - bw*K + w0^2)/a0];
end
