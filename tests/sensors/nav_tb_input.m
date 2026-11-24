function ds = nav_tb_input(P, prof)
%NAV_TB_INPUT  External input Dataset of tests/sensors/tb_nav.slx (build task T12).
%
%   ds = nav_tb_input(P, prof)   elements in root-port order: prof, drive_truth, env, elec_env, flt, obs, navcmd,
%                                bms, smctl
%   prof fields (all optional):
%     T        stop time [s] (default 10)
%     kin      N x 9 breakpoints [t v r vy sL sR chi gx gy] (linear interpolation, held after the last row);
%              default standstill with slip P.sensors.ekf.s0 and chi = P.structure.track.chi_nom on level ground
%     iDC      N x 3 ZOH table [t I_L I_R] traction DC currents [A] (default 0)
%     Timu     IMU temperature [degC]: scalar or N x 2 table [t T] (linear) (default 25 = calibration temperature)
%     corrOK, corrAge   RTK correction link (default true, 1 s)
%     flt      cell {'field', [t v; ...]} ZOH tables of Bus_FaultInject elements (e.g. gnss_outage)
%     navcmd   cell {'navMode' | 'navReset' | 'priorMapId', [t v; ...]} ZOH tables of Bus_NavCmd
%     bms      cell {'I_pack', [t v; ...]} ZOH tables of Bus_BmsStatus elements (valid = true)
%     sm       N x 5 ZOH table [t on sigXY sigPsi pFail] of the scan-match emulation (default off)
d = struct('T', 10, 'kin', [], 'iDC', [0 0 0], 'Timu', 25, 'corrOK', true, 'corrAge', 1, 'flt', {{}}, ...
           'navcmd', {{}}, 'bms', {{}}, 'sm', [0 0 0.03 0.5*pi/180 0]);
fn = fieldnames(d);
for k = 1:numel(fn)
    if ~isfield(prof, fn{k}), prof.(fn{k}) = d.(fn{k}); end
end
T = prof.T;
if isempty(prof.kin)
    prof.kin = [0 0 0 0 P.sensors.ekf.s0 P.sensors.ekf.s0 P.structure.track.chi_nom 0 0];
end
ds = Simulink.SimulationData.Dataset;

%% ---- kinematic profile (linear breakpoints) ----
K = prof.kin;
if K(1, 1) > 0, K = [[0 K(1, 2:end)]; K]; end
if K(end, 1) < T, K(end+1, :) = [T K(end, 2:end)]; end
ds = ds.addElement(timeseries(K(:, 2:9), K(:, 1), 'Name', 'prof'), 'prof');

%% ---- drive truth (traction currents only) ----
D = local_const(P.ic.Bus_DriveTruth, T);
D.L.i_dc = local_zoh(0, prof.iDC(:, [1 2]), T, 'i_dc');
D.R.i_dc = local_zoh(0, prof.iDC(:, [1 3]), T, 'i_dc');
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_DriveTruth', D), 'drive_truth');

%% ---- environment ----
E = P.ic.Bus_EnvState;  E.T_amb = 318.15;  E.T_sky = 306.15;  E.T_ground = 333.15;  E.soil_id = uint8(2);
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_EnvState', local_const(E, T)), 'env');

%% ---- electronics environment ----
EE = local_const(P.ic.Bus_ElecEnv, T);
if isscalar(prof.Timu)
    Ti = [0 prof.Timu; T prof.Timu];
else
    Ti = prof.Timu;
    if Ti(1, 1) > 0, Ti = [[0 Ti(1, 2)]; Ti]; end
    if Ti(end, 1) < T, Ti(end+1, :) = [T Ti(end, 2)]; end
end
EE.T_imu_degC = timeseries(Ti(:, 2), Ti(:, 1), 'Name', 'T_imu_degC');
EE.T_encl_K = timeseries(Ti(:, 2) - 5 + 273.15, Ti(:, 1), 'Name', 'T_encl_K');
EE.gnssCorrOK = local_zoh(logical(prof.corrOK), [0 double(prof.corrOK)], T, 'gnssCorrOK');
EE.gnssCorrAge = local_zoh(prof.corrAge, [0 prof.corrAge], T, 'gnssCorrAge');
EE.sensPowerOK = local_zoh(uint8(7), [0 7], T, 'sensPowerOK');
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_ElecEnv', EE), 'elec_env');

%% ---- fault injection, dynamic obstacles ----
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_FaultInject', ...
    local_tables(P.ic.Bus_FaultInject, prof.flt, T)), 'flt');
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_ObstaclesDyn', ...
    local_tables(P.ic.Bus_ObstaclesDyn, {}, T)), 'obs');

%% ---- navigation command, BMS status ----
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_NavCmd', ...
    local_tables(P.ic.Bus_NavCmd, prof.navcmd, T)), 'navcmd');
B = prof.bms;
if isempty(B) || ~any(strcmp(B(:, 1), 'valid')), B = [B; {'valid', [0 1]}]; end
ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries('Bus_BmsStatus', ...
    local_tables(P.ic.Bus_BmsStatus, B, T)), 'bms');

%% ---- scan-match emulation control (0.2 s, ZOH) ----
ds = ds.addElement(local_zoh(zeros(4, 1), prof.sm, T, 'smctl'), 'smctl');
end

% =============================================================================================
function S = local_const(ic, T)
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
