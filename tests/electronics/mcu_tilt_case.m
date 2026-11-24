function r = mcu_tilt_case(theta, aProf, T, P, gyroBias, pivot)
%MCU_TILT_CASE  Run the tilt chain harness (sens_odom_compute -> RT -> fw_imu_tilt) for one motion case.
%
%   r = mcu_tilt_case(theta, aProf, T, P, gyroBias, pivot)
%   theta  vehicle pitch [rad] in the ICD convention (+ nose down); the ground is a plane inclined by theta
%   aProf  function handle a(t) [m/s^2]: track (along-slope) acceleration, v(0) = 0
%   T      stop time [s];  P  parameter struct override ([] = base P);  gyroBias  gyro-z offset [rad/s]
%   pivot  [t0 t1 vp w] (optional): from t0 to t1 the tracks counter-rotate at -vp (left) / +vp (right) m/s
%          on top of the motion and the gyro z reads w more (a pivot turn: mean track speed unchanged)
%   The IMU (Bus_ImuMeas at 0.005 s) sends the physical specific force of that motion in body FLU:
%   f = [a - g*sin(theta); 0; g*cos(theta)]; encoder counts = round(x_L,R/K_enc) at 0.01 s.
%   r: .t5 .roll .pitch .tilt_gate .yaw_rate_meas .gyro_bias (0.005 s), .t10 .a_track .v_track (0.01 s)

if nargin < 4 || isempty(P), P = evalin('base', 'P'); end
P.electronics.imu_cal = struct('roll0', 0, 'pitch0', 0);       % ideal specific force: no level calibration
if nargin < 5 || isempty(gyroBias), gyroBias = 0; end
if nargin < 6 || isempty(pivot), pivot = [0 0 0 0]; end
g = 9.80665;
dt = 1e-4;                                     % fine grid for the kinematics
tf = (0:dt:T)';
a = arrayfun(aProf, tf);
v = cumtrapz(tf, a);
x = cumtrapz(tf, v);
t10 = (0:0.01:T)';
t5 = (0:0.005:T)';
inPiv = @(tt) tt >= pivot(1) & tt < pivot(2);
vp = pivot(3)*inPiv(tf);  xp = cumtrapz(tf, vp);
xi = interp1(tf, x, t10); vi10 = interp1(tf, v, t10);
xpi = interp1(tf, xp, t10); vpi = interp1(tf, vp, t10);
Kenc = P.electronics.drv.K_enc;
enc = int32(round([xi - xpi, xi + xpi]/Kenc));
wm = single([vi10 - vpi, vi10 + vpi] * P.electronics.drv.k_v2w);
tus = uint32(mod(round(t10*1e6), 2^32));
a5 = interp1(tf, a, t5);
acc = single([a5 - g*sin(theta), zeros(size(t5)), g*cos(theta)*ones(size(t5))]);
gyr = single([zeros(numel(t5), 2), gyroBias + pivot(4)*inPiv(t5)]);
imu.accel_B = timeseries(acc, t5);
imu.gyro_B = timeseries(gyr, t5);
imu.mag_B = timeseries(zeros(numel(t5), 3, 'single'), t5);
imu.temp_degC = timeseries(single(35*ones(size(t5))), t5);
imu.t_us = timeseries(uint32(mod(round(t5*1e6), 2^32)), t5);
imu.seq = timeseries(uint16(mod((0:numel(t5)-1)', 65536)), t5);
imu.valid = timeseries(true(size(t5)), t5);
ds = mcu_harness('ds', {'enc', timeseries(enc, t10)}, {'wm', timeseries(wm, t10)}, {'t_us', timeseries(tus, t10)}, ...
    {'fbv', timeseries(true(size(t10)), t10)}, {'imu', imu});
o = mcu_harness('sim', 'tilt', T, ds, P);
y = o.yout;
get1 = @(nm) squeeze(double(y.get(nm).Values.Data));
r.t5 = y.get('pitch').Values.Time;
r.roll = get1('roll'); r.pitch = get1('pitch'); r.tilt_gate = logical(get1('tilt_gate'));
r.yaw_rate_meas = get1('yaw_rate_meas'); r.gyro_bias = get1('gyro_bias');
r.t10 = y.get('a_track').Values.Time; r.a_track = get1('a_track'); r.v_track = get1('v_track');
end
