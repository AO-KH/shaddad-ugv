function R = sens_tb_run(P, prof, varargin)
%SENS_TB_RUN  Simulate tests/sensors/tb_sens_plant.slx for one stimulus profile and unpack the outputs.
%
%   R = sens_tb_run(P, prof)                 prof: see sens_tb_input (prof.T = StopTime)
%   R = sens_tb_run(P, prof, 'VC_LIDAR', 2, 'cloudN', 20)
%   R fields: imu (t, acc, gyro, mag, temp), gnss (t, t_meas, lla, vel, fix, numSV, hAcc, vAcc, sAcc,
%     headValid, heading, headAcc, corrAge, newData), scan (t, ranges, angles, valid), spray (t, mask,
%     density, valid), diag (t, svf, zone, fix, relpos, posErr, outlier, dust, poseL), meas (t, acc, gyro,
%     mag, temp, t_us, seq, valid), odom (t, v, cnt, valid), a_track (t, a), cloud (xyz [32x1024x3xN], valid),
%     wall (wall-clock time of sim), out (Simulink.SimulationOutput)
%   P is passed with SimulationInput.setVariable (the base workspace is not modified).

ip = inputParser;
ip.addParameter('VC_LIDAR', 1);
ip.addParameter('cloudN', 2);
ip.addParameter('Mode', 'normal');
ip.parse(varargin{:});
o = ip.Results;
mdl = 'tb_sens_plant';
if ~bdIsLoaded(mdl), load_system(mdl); end
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(sens_tb_input(P, prof));
in = in.setVariable('P', P);
in = in.setVariable('VC_LIDAR', o.VC_LIDAR);
in = in.setVariable('tbs', struct('cloudN', o.cloudN), 'Workspace', mdl);
in = in.setModelParameter('StopTime', num2str(prof.T, 17), 'SimulationMode', o.Mode);
t0 = tic;
try
    out = sim(in);
catch e
    % MATLAB Function JIT relink assertion (Stateflow:Assertion) seen once after a rebuilt / edited chart in a
    % long-lived session; the second compile succeeds
    if ~strcmp(e.identifier, 'Stateflow:Assertion'), rethrow(e); end
    t0 = tic;
    out = sim(in);
end
R.wall = toc(t0);
R.out = out;
if ~isempty(out.ErrorMessage), error('sens_tb_run:sim', '%s', out.ErrorMessage); end
y = out.yout;
v = @(k) y{k}.Values;
I = v(1);
R.imu = struct('t', I.Acceleration.Time, 'acc', double(I.Acceleration.Data), 'gyro', double(I.AngularRate.Data), ...
    'mag', double(I.MagneticField.Data), 'temp', double(I.Temperature.Data));
G = v(2);
R.gnss = struct('t', G.t_meas.Time, 't_meas', G.t_meas.Data, 'lla', G.lla.Data, 'vel', G.vel_ENU.Data, ...
    'fix', double(G.fixType.Data), 'numSV', double(G.numSV.Data), 'hAcc', double(G.hAcc.Data), ...
    'vAcc', double(G.vAcc.Data), 'sAcc', double(G.sAcc.Data), 'headValid', G.headingValid.Data, ...
    'heading', G.heading_deg.Data, 'headAcc', double(G.headAcc_deg.Data), 'corrAge', double(G.corrAge.Data), ...
    'newData', G.newData.Data);
S = v(3);
R.scan = struct('t', S.t_scan.Time, 'ranges', double(S.ranges.Data), 'angles', double(S.angles.Data), ...
    'valid', S.valid.Data);
Sp = v(4);
R.spray = struct('t', Sp.t_stamp.Time, 'mask', double(Sp.section_mask.Data), 'density', double(Sp.target_density.Data), ...
    'valid', Sp.valid.Data);
D = v(5);
R.diag = struct('t', D.svf.Time, 'svf', D.svf.Data, 'zone', double(D.zoneId.Data), 'fix', double(D.fixTypeTruth.Data), ...
    'relpos', D.relposFixed.Data, 'posErr', D.gnssPosErr.Data, 'outlier', D.outlier.Data, 'dust', D.dustLevel.Data, ...
    'poseL', D.lidarPoseTruth.Data);
M = v(6);
R.meas = struct('t', M.accel_B.Time, 'acc', double(M.accel_B.Data), 'gyro', double(M.gyro_B.Data), ...
    'mag', double(M.mag_B.Data), 'temp', double(M.temp_degC.Data), 't_us', double(M.t_us.Data), ...
    'seq', double(M.seq.Data), 'valid', M.valid.Data);
Od = v(7);
R.odom = struct('t', Od.vTrack_LR.Time, 'v', double(Od.vTrack_LR.Data), 'cnt', double(Od.encCnt_LR.Data), ...
    'valid', Od.valid.Data);
A = v(8);
R.a_track = struct('t', A.Time, 'a', double(A.Data(:)));
R.cloud = struct('xyz', [], 'valid', [], 't', []);
try
    cx = out.get('tb_cloud_xyz');  cv = out.get('tb_cloud_valid');
    R.cloud.xyz = double(cx.signals.values);  R.cloud.t = cx.time;  R.cloud.valid = cv.signals.values(:);
catch
end
% bus outputs: 1-D vectors arrive as N x n, scalars as N x 1
fn = {'acc', 'gyro', 'mag'};
for k = 1:numel(fn)
    R.imu.(fn{k}) = local_rows(R.imu.(fn{k}), 3);
    R.meas.(fn{k}) = local_rows(R.meas.(fn{k}), 3);
end
R.gnss.lla = local_rows(R.gnss.lla, 3);  R.gnss.vel = local_rows(R.gnss.vel, 3);
R.diag.posErr = local_rows(R.diag.posErr, 3);  R.diag.poseL = local_rows(R.diag.poseL, 3);
R.scan.ranges = local_rows(R.scan.ranges, 360);  R.scan.angles = local_rows(R.scan.angles, 360);
R.odom.v = local_rows(R.odom.v, 2);  R.odom.cnt = local_rows(R.odom.cnt, 2);
end

function x = local_rows(x, n)
% normalise logged data to N x n (time first)
if ndims(x) == 3
    x = reshape(permute(x, [3 1 2]), size(x, 3), []);
elseif size(x, 2) ~= n && size(x, 1) == n
    x = x.';
end
end
