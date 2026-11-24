function ds = sens_tb_input(P, prof)
%SENS_TB_INPUT  External input Dataset of tests/sensors/tb_sens_plant.slx (build task T11).
%
%   ds = sens_tb_input(P, prof)       ds: Simulink.SimulationData.Dataset, elements in root-port order
%                                     truth, drive_truth, env, elec_env, flt, obs, enc_cnt
%   prof fields (all optional):
%     T      stop time [s] (default 10)
%     dt     truth sample grid [s] (default 0.01; the root port interpolates linearly to 1 ms)
%     pose   @(t) -> N x 6 [xE yN zU psi theta phi] of base_link (ZYX, theta + = nose down), t column
%            (default all zero). Velocities, accelerations, body rates and angular accelerations are
%            computed by central differences of this function (h = 1e-3 s).
%     iDC    [I_L I_R] traction DC currents [A] (default [0 0])
%     Timu   IMU temperature [degC]: scalar or @(t) (default 25)
%     corrOK, corrAge   RTK correction link state (default true, 1 s)
%     flt    N x 2 cell {'field', [t v; ...]} zero-order-hold tables of Bus_FaultInject elements
%     obs    N x 2 cell {'n' | 'xy' | 'r', [t v...]} ZOH tables of Bus_ObstaclesDyn (xy values in (:)' order)
%     enc    @(t) -> N x 2 unwrapped encoder counts (sampled every 0.01 s, wrapped to int32)

if nargin < 2, prof = struct(); end
d = struct('T', 10, 'dt', 0.01, 'pose', @(t) zeros(numel(t), 6), 'iDC', [0 0], 'Timu', 25, ...
           'corrOK', true, 'corrAge', 1, 'flt', {{}}, 'obs', {{}}, 'enc', @(t) zeros(numel(t), 2));
fn = fieldnames(d);
for k = 1:numel(fn)
    if ~isfield(prof, fn{k}), prof.(fn{k}) = d.(fn{k}); end
end
T = prof.T;
g = P.frames.g;

%% ---- truth ----
t = (0:prof.dt:T)';
if t(end) < T, t(end+1) = T; end
N = numel(t);
X = prof.pose(t);
h = 1e-3;
Xp = prof.pose(t + h);  Xm = prof.pose(t - h);
pos = X(:, 1:3);
vel = (Xp(:, 1:3) - Xm(:, 1:3))/(2*h);
acc = (Xp(:, 1:3) - 2*X(:, 1:3) + Xm(:, 1:3))/h^2;
R = local_R(X);
w0 = local_omega(prof.pose, t, h);
alpha = (local_omega(prof.pose, t + h, h) - local_omega(prof.pose, t - h, h))/(2*h);
velB = zeros(N, 3); accB = zeros(N, 3); sfB = zeros(N, 3); q = zeros(N, 4);
for k = 1:N
    Rk = R(:, :, k);
    velB(k, :) = (Rk'*vel(k, :)')';
    accB(k, :) = (Rk'*acc(k, :)')';
    sfB(k, :) = (Rk'*(acc(k, :)' + [0; 0; g]))';
    q(k, :) = local_rotm2quat(Rk);
end
psi = atan2(sin(X(:, 4)), cos(X(:, 4)));
S = P.ic.Bus_TruthState;
V = struct('t', t, 'pos_W', pos, 'vel_W', vel, 'acc_W', acc, 'q_WB', q, 'R_WB', R, ...
    'eul', [psi X(:, 5) X(:, 6)], 'vel_B', velB, 'omega_B', w0, 'alpha_B', alpha, 'acc_B', accB, ...
    'specForce_B', sfB, 'r_cg_B', zeros(N, 3), 'lla', repmat(P.world.lla0, N, 1), 'pose2D', [pos(:, 1:2) psi], ...
    'speed_fwd', velB(:, 1), 'mass', P.structure.mp.m0*ones(N, 1), 'terrainH', pos(:, 3), 'grade_long', zeros(N, 1), ...
    'grade_lat', zeros(N, 1), 'vBelt_LR', [velB(:, 1) velB(:, 1)], 'vGround_LR', [velB(:, 1) velB(:, 1)], ...
    'slip_LR', zeros(N, 2));
TS = struct();
f = fieldnames(S);
for k = 1:numel(f)
    TS.(f{k}) = local_ts(V.(f{k}), t, S.(f{k}), f{k});
end
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_TruthState', TS), 'truth');

%% ---- drive truth (traction currents only) ----
D = P.ic.Bus_DriveTruth;
D.L.i_dc = prof.iDC(1);  D.R.i_dc = prof.iDC(2);
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_DriveTruth', local_const(D, T)), 'drive_truth');

%% ---- environment ----
E = P.ic.Bus_EnvState;  E.T_amb = 318.15;  E.T_sky = 306.15;  E.T_ground = 333.15;  E.soil_id = uint8(2);
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_EnvState', local_const(E, T)), 'env');

%% ---- electronics environment ----
te = (0:0.1:T)';  if te(end) < T, te(end+1) = T; end
if isa(prof.Timu, 'function_handle'), Ti = prof.Timu(te); else, Ti = prof.Timu*ones(size(te)); end
EE = local_const(P.ic.Bus_ElecEnv, T);
EE.T_imu_degC = timeseries(Ti(:), te);  EE.T_imu_degC.Name = 'T_imu_degC';
EE.T_encl_K = timeseries(Ti(:) - 5 + 273.15, te);  EE.T_encl_K.Name = 'T_encl_K';
EE.gnssCorrOK = local_zoh(logical(prof.corrOK), [0 double(prof.corrOK)], T, 'gnssCorrOK');
EE.gnssCorrAge = local_zoh(prof.corrAge, [0 prof.corrAge], T, 'gnssCorrAge');
EE.sensPowerOK = local_zoh(uint8(7), [0 7], T, 'sensPowerOK');
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_ElecEnv', EE), 'elec_env');

%% ---- fault injection, dynamic obstacles (ZOH tables) ----
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_FaultInject', ...
    local_tables(P.ic.Bus_FaultInject, prof.flt, T)), 'flt');
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_ObstaclesDyn', ...
    local_tables(P.ic.Bus_ObstaclesDyn, prof.obs, T)), 'obs');

%% ---- encoder counts (int32 wrap) at 0.01 s ----
tc = (0:0.01:T)';
c = prof.enc(tc);
c = mod(c + 2^31, 2^32) - 2^31;
ds = ds.addElement(timeseries(int32(c), tc, 'Name', 'enc_cnt'), 'enc_cnt');
end

% =============================================================================================
function R = local_R(X)
N = size(X, 1);  R = zeros(3, 3, N);
for k = 1:N
    cy = cos(X(k, 4)); sy = sin(X(k, 4)); cp = cos(X(k, 5)); sp = sin(X(k, 5)); cr = cos(X(k, 6)); sr = sin(X(k, 6));
    R(:, :, k) = [cy -sy 0; sy cy 0; 0 0 1]*[cp 0 sp; 0 1 0; -sp 0 cp]*[1 0 0; 0 cr -sr; 0 sr cr];
end
end

function w = local_omega(pose, t, h)
R0 = local_R(pose(t));  Rp = local_R(pose(t + h));  Rm = local_R(pose(t - h));
N = numel(t);  w = zeros(N, 3);
for k = 1:N
    W = R0(:, :, k)'*(Rp(:, :, k) - Rm(:, :, k))/(2*h);
    W = (W - W')/2;
    w(k, :) = [W(3, 2) W(1, 3) W(2, 1)];
end
end

function q = local_rotm2quat(R)
% scalar-first quaternion of a rotation matrix (same as rotm2quat), robust branch
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1)*2;
    q = [s/4, (R(3,2) - R(2,3))/s, (R(1,3) - R(3,1))/s, (R(2,1) - R(1,2))/s];
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = sqrt(1 + R(1,1) - R(2,2) - R(3,3))*2;
    q = [(R(3,2) - R(2,3))/s, s/4, (R(1,2) + R(2,1))/s, (R(1,3) + R(3,1))/s];
elseif R(2,2) > R(3,3)
    s = sqrt(1 + R(2,2) - R(1,1) - R(3,3))*2;
    q = [(R(1,3) - R(3,1))/s, (R(1,2) + R(2,1))/s, s/4, (R(2,3) + R(3,2))/s];
else
    s = sqrt(1 + R(3,3) - R(1,1) - R(2,2))*2;
    q = [(R(2,1) - R(1,2))/s, (R(1,3) + R(3,1))/s, (R(2,3) + R(3,2))/s, s/4];
end
if q(1) < 0, q = -q; end
end

function ts = local_ts(v, t, leaf, name)
% timeseries of one bus leaf from N-row data (vectors N x n, matrices r x c x N)
cls = class(leaf);
dims = size(leaf);
if ndims(v) == 3 || (numel(dims) == 2 && all(dims > 1))
    data = cast(v, cls);
    ts = timeseries(data, t);
    ts.IsTimeFirst = false;
else
    if islogical(leaf), data = v ~= 0; else, data = cast(v, cls); end
    ts = timeseries(data, t);
end
ts.Name = name;
end

function S = local_const(ic, T)
% constant struct of timeseries (two samples) of a bus ground value
S = struct();
f = fieldnames(ic);
for k = 1:numel(f)
    v = ic.(f{k});
    if isstruct(v)
        S.(f{k}) = local_const(v, T);
    else
        S.(f{k}) = local_zoh(v, [0 double(v(:))'], T, f{k});
    end
end
end

function S = local_tables(ic, items, T)
S = struct();
f = fieldnames(ic);
for k = 1:numel(f)
    v = ic.(f{k});
    j = [];
    if ~isempty(items), j = find(strcmp(items(:, 1), f{k}), 1, 'last'); end
    if isempty(j), tbl = [0 double(v(:))']; else, tbl = items{j, 2}; end
    S.(f{k}) = local_zoh(v, tbl, T, f{k});
end
end

function ts = local_zoh(leaf, tbl, T, name)
% exactly piecewise-constant timeseries (step pairs at every change instant)
dims = size(leaf);  n = numel(leaf);  cls = class(leaf);
t = double(tbl(:, 1));  v = double(tbl(:, 2:end));
if size(v, 2) == 1 && n > 1, v = repmat(v, 1, n); end
if t(1) > 0, t = [0; t]; v = [double(leaf(:))'; v]; end
tt = t(1);  vv = v(1, :);
for k = 2:numel(t)
    tt = [tt; t(k); t(k)]; %#ok<AGROW>
    vv = [vv; v(k - 1, :); v(k, :)]; %#ok<AGROW>
end
if T > tt(end), tt(end+1, 1) = T; vv(end+1, :) = vv(end, :); end
Nt = numel(tt);
if islogical(leaf), vv = vv ~= 0; else, vv = cast(vv, cls); end
if n == 1
    data = vv(:);
elseif numel(dims) == 2 && (dims(2) == 1 || dims(1) == 1)
    data = vv;
else
    data = reshape(vv', [dims Nt]);
end
ts = timeseries(data, tt);
if n > 1 && ~(numel(dims) == 2 && (dims(2) == 1 || dims(1) == 1))
    ts.IsTimeFirst = false;
end
ts.Name = name;
end
