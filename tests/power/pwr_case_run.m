function R = pwr_case_run(name, varargin)
%PWR_CASE_RUN  Run one tb_pack_bms scenario of pwr_case_def and return compact results (owner: power, T04).
%
%   R = pwr_case_run(name)                    complete run (or the cached result) of case 'name'
%   R = pwr_case_run(name, 'Force', true)     discard the cache and simulate again
%   R = pwr_case_run(name, 'CacheOnly', true) never simulate: R = [] when no valid cache exists
%   R = pwr_case_run(name, 'Chunk', dT)       advance a long case by at most dT seconds of simulated
%                                             time per call (operating-point continuation);
%                                             R.done is false until the case is complete
%   in = pwr_case_run(name, 'InputOnly', true) only the Simulink.SimulationInput of the complete run
%                                             (profiling / speed experiments; nothing is simulated)
%
%   The run is independent of the shared base workspace: P, Ts and VC_* come from
%   shaddad_params('Assign', false) plus the case modification C.pmod, passed with
%   Simulink.SimulationInput.setVariable (global scope), TB into the bench model workspace.
%   Mission mode (C.mission, docs/specs/power.md R9 / T-PWR-19 "Lumped at 5 ms"): the 1 ms plant
%   rates of the bench (Ts.base, Ts.ssc, Ts.esc, Ts.safety, Ts.enc) and the Simscape local solver
%   step P.sim.ssc.Ts become P.power.tb.Ts_mission (5 ms); the BMS rates (0.01 / 0.05 / 0.1 / 1 s)
%   are unchanged. Used for the multi-hour charge / endurance / estimation cases.
%   Only the Bus_* objects are taken from the base workspace (created if missing).
%   Cache: work/T04/<name>.mat (complete result) and work/T04/<name>_run.mat (partial run with the
%   operating point), valid while the key (case definition, P.power except the T05-only lv / impl /
%   safety branches, P.sim.ssc, Ts, VC, the content
%   of the T04 generator sources - build scripts, bms_mode_manager.m, bench defaults, config sets -,
%   the used bus definitions and the MATLAB version) is unchanged (see local_key).
%
%   R fields: name, desc, key, done, T_end (s), wall_init / wall_exec (s, summed over chunks),
%     rtf (= T_end / wall_exec), n_chunks, VC_PACK,
%     f   log_fast [t V_pack V_bus V_dock I_pack I_load I_chg K1 K2 K4 P_pack P_trac P_chg_bus P_pdu_loss]
%     c   log_cmd  [t K1_Cmd K2_Cmd K4_Cmd Charger_Enable I_req Fan_Cmd n_Bal HV_Enable_Req_MCU]
%     tel telemetry: t + one field per Bus_BMS_Telemetry element (N x width)
%     tru truth (Bus_PowerTruth): t + fields; vector fields decimated to 1 s for runs longer than 2 h
%         (time vector tru.t_vec); tru_full: full-rate max / min envelopes of V, T, SOC over the groups
%     ev  internal BMS signals as change points: bms_state, bms_fault_latched, bms_hv_enable_eff,
%         bms_mcu_timeout (fields .t .v)

ip = inputParser;
ip.addParameter('Force', false);
ip.addParameter('CacheOnly', false);
ip.addParameter('Chunk', inf);
ip.addParameter('InputOnly', false);
ip.parse(varargin{:});
o = ip.Results;

C = pwr_case_def(name);
[P, Ts, VC] = shaddad_params('Assign', false);
P = C.pmod(P);
if C.mission
    tm = P.power.tb.Ts_mission;
    for fn = {'base', 'ssc', 'esc', 'safety', 'enc'}
        Ts.(fn{1}) = tm;
    end
    P.sim.ssc.Ts = tm;
end
VC.VC_PACK = C.VC_PACK;
mdl = 'tb_pack_bms';
if o.InputOnly
    load_system(mdl);
    R = local_input(mdl, C, P, Ts, VC, C.T);
    return
end
key = local_key(C, P, Ts, VC);
keyFull = local_key(C, P, Ts, VC, true);      % key v1 (whole P.power): accepted once, then upgraded
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cdir = fullfile(root, 'work', 'T04');
if ~isfolder(cdir), mkdir(cdir); end
cf = fullfile(cdir, [name '.mat']);
sf = fullfile(cdir, [name '_run.mat']);
if o.Force
    if isfile(cf), delete(cf); end
    if isfile(sf), delete(sf); end
end
if isfile(cf)
    S = load(cf, 'R');
    if isfield(S, 'R') && isfield(S.R, 'key') && strcmp(S.R.key, keyFull)
        S.R.key = key;                         % cache made with the v1 key of the current data
        R = S.R; %#ok<NASGU>
        save(cf, 'R', '-v7.3');
    end
    if isfield(S, 'R') && isfield(S.R, 'key') && strcmp(S.R.key, key)
        R = S.R;
        return
    end
end
if o.CacheOnly
    R = [];
    return
end
if ~evalin('base', 'exist(''Bus_BMS_Meas'', ''var'')')
    shaddad_buses();
end
load_system(mdl);
% ---- resume a partial run ----
R = []; op = [];
if isfile(sf)
    S = load(sf, 'R', 'op');
    if isfield(S, 'R') && strcmp(S.R.key, key)
        R = S.R; op = S.op;
    end
end
t0 = 0;
if ~isempty(R), t0 = R.T_end; end
if isinf(o.Chunk)
    tStop = C.T;
else
    tStop = min(C.T, t0 + o.Chunk);
end
chunked = ~(t0 == 0 && tStop >= C.T);
in = local_input(mdl, C, P, Ts, VC, tStop);
if chunked
    in = in.setModelParameter('SaveFinalState', 'on', 'SaveOperatingPoint', 'on', 'FinalStateName', 'xFinal');
end
if ~isempty(op)
    in = in.setInitialState(op);
end
% (no Fast Restart: SaveFinalState is not run-time tunable and the bench uses the shared config
%  reference, so every chunk recompiles; about 25-45 s per chunk)
if strcmp(get_param(mdl, 'FastRestart'), 'on'), set_param(mdl, 'FastRestart', 'off'); end
% The shared session records logged data in the Simulation Data Inspector (needed: with recording
% off the logged outputs come back empty); this run's data are copied into R and saved, then the SDI
% runs this call created for the bench are deleted (keeps the shared session's memory bounded).
ids0 = Simulink.sdi.getAllRunIDs;
out = sim(in);
Rk = local_extract(out, C);
if isempty(R)
    R = Rk;
    R.n_chunks = 1;
else
    R = local_merge(R, Rk);
end
R.key = key;
tEnd = R.T_end;
R.done = tEnd >= C.T - 1e-6 || tEnd < tStop - 1e-3;       % stop time reached or a Stop block fired
if R.done
    R.rtf = R.T_end / R.wall_exec;
    save(cf, 'R', '-v7.3');
    if isfile(sf), delete(sf); end
else
    op = out.xFinal;
    save(sf, 'R', 'op', '-v7.3');
end
clear out op
ids = setdiff(Simulink.sdi.getAllRunIDs, ids0);
for k = 1:numel(ids)
    try
        if strcmp(Simulink.sdi.getRun(ids(k)).Model, mdl), Simulink.sdi.deleteRun(ids(k)); end
    catch
    end
end
end

% -------------------------------------------------------------------------------------------
function in = local_input(mdl, C, P, Ts, VC, tStop)
in = Simulink.SimulationInput(mdl);
in = in.setVariable('TB', C.TB, 'Workspace', mdl);
in = in.setVariable('P', P);
in = in.setVariable('Ts', Ts);
vn = fieldnames(VC);
for k = 1:numel(vn)
    in = in.setVariable(vn{k}, VC.(vn{k}));
end
in = in.setModelParameter('StopTime', sprintf('%.15g', tStop));
end

function key = local_key(C, P, Ts, VC, v1)
% v2 (default): P.power without the T05-only branches lv / impl / safety (not used by the T04 bench,
% models or cases); v1 (v1 = true): the whole P.power (format of the caches written before)
if nargin > 4 && v1
    Pp = P.power;
else
    Pp = rmfield(P.power, intersect(fieldnames(P.power), {'lv', 'impl', 'safety'}));
end
% The models are generated by idempotent build scripts, so the key hashes the CONTENT of the generator
% sources (build scripts, firmware function, bench stimulus defaults, config-set script) and the bus
% definitions the bench uses, not the .slx modification dates: a functionally identical rebuild keeps
% the cache, any change of a generator, parameter, bus or rate invalidates it.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
files = {'scripts/build/build_shaddad_battery.m', 'scripts/build/build_power_pack.m', ...
         'scripts/build/build_shaddad_bms.m', 'scripts/build/build_tb_pack_bms.m', 'scripts/build/shaddad_bu.m', ...
         'subsystems/power/bms_mode_manager.m', 'tests/power/tb_pack_bms_default.m', 'params/shaddad_configsets.m'};
d = cell(size(files));
for k = 1:numel(files)
    f = fullfile(root, files{k});
    if isfile(f), d{k} = fileread(f); else, d{k} = 'missing'; end
end
bn = {'Bus_BMS_Meas', 'Bus_BMS_Cmd', 'Bus_Can1_Bms', 'Bus_BMS_Telemetry', 'Bus_Can1_McuSlow', 'Bus_CanFrame8', ...
      'Bus_BmsReq', 'Bus_PowerStatus', 'Bus_PowerTruth', 'Bus_BmsStatus'};
defs = shaddad_buses('defs');
bd = defs(ismember({defs.name}, bn));
c = rmfield(C, 'pmod');
blob = double(getByteStreamFromArray({c, Pp, P.sim.ssc, Ts, VC, d, bd, version}));
blob = blob(:);
n = numel(blob);
w1 = mod((1:n)', 65521); w2 = mod((1:n)' * 7919, 65537);
key = sprintf('%d_%.0f_%.0f_%.0f', n, sum(blob), mod(sum(blob .* w1), 2^50), mod(sum(blob .* w2), 2^50));
end

function R = local_extract(out, C)
R = struct('name', C.name, 'desc', C.desc, 'VC_PACK', C.VC_PACK, 'T_end', NaN);
ti = out.SimulationMetadata.TimingInfo;
R.wall_init = ti.InitializationElapsedWallTime;
R.wall_exec = ti.ExecutionElapsedWallTime;
long = C.T > 7200;
cls = @(x) x;
if long, cls = @single; end
R.f = cls(out.log_fast);
R.c = cls(out.log_cmd);
R.T_end = double(out.tout(end));
R.rtf = R.T_end / R.wall_exec;
ls = out.logsout;
R.tel = local_bus(ls.get('telem').Values, 1);
dec = 1;
if long, dec = 10; end                                  % vectors at 1 s for long runs
B = ls.get('power_truth').Values;
R.tru = local_bus(B, dec);
R.tru_full = local_env(B);
R.ev = struct();
for nm = {'bms_state', 'bms_fault_latched', 'bms_hv_enable_eff', 'bms_mcu_timeout', 'bms_r0_ratio'}
    try
        e = ls.get(nm{1});
        if isa(e, 'Simulink.SimulationData.Signal'), ts = e.Values; else, ts = e{1}.Values; end
        v = double(squeeze(ts.Data)); t = ts.Time;
        k = [1; find(diff(v(:)) ~= 0) + 1];
        R.ev.(nm{1}) = struct('t', t(k), 'v', v(k));
    catch
        R.ev.(nm{1}) = struct('t', zeros(0, 1), 'v', zeros(0, 1));
    end
end
end

function S = local_bus(B, dec)
% struct of timeseries -> struct of N x width arrays (vector fields decimated by dec)
fn = fieldnames(B);
t = B.(fn{1}).Time;
S.t = t;
for k = 1:numel(fn)
    v = B.(fn{k}).Data;
    if ndims(v) == 3, v = reshape(permute(v, [3 1 2]), size(v, 3), []); end
    if size(v, 1) ~= numel(t), v = v.'; end
    if size(v, 2) > 1 && dec > 1
        S.(fn{k}) = v(1:dec:end, :);
    else
        S.(fn{k}) = v;
    end
end
S.t_vec = t(1:dec:end);
end

function S = local_env(B)
% full-rate envelopes of the truth vectors (max / min over groups); logged vector data come as
% N x 16 (1-D bus element) or 16 x 1 x N (column) arrays: both become N x 16
S.t = B.V_cell_true.Time;
n = numel(S.t);
v = local_rows(B.V_cell_true.Data, n); T = local_rows(B.T_cell_true.Data, n); s = local_rows(B.SOC_true.Data, n);
S.V_max = max(v, [], 2); S.V_min = min(v, [], 2);
S.T_max = max(T, [], 2); S.T_min = min(T, [], 2);
S.SOC_max = max(s, [], 2); S.SOC_min = min(s, [], 2);
end

function x = local_rows(x, n)
% time along the first dimension
if ndims(x) == 3
    x = reshape(permute(x, [3 1 2]), size(x, 3), []);
elseif size(x, 1) ~= n
    x = x.';
end
end

function R = local_merge(R, K)
% append chunk K to R (drop samples at or before the last stored time)
R.wall_init = R.wall_init + K.wall_init;
R.wall_exec = R.wall_exec + K.wall_exec;
R.n_chunks = R.n_chunks + 1;
R.T_end = K.T_end;
R.f = local_cat(R.f, K.f);
R.c = local_cat(R.c, K.c);
R.tel = local_catS(R.tel, K.tel, 't');
R.tru = local_catS(R.tru, K.tru, 't');
R.tru_full = local_catS(R.tru_full, K.tru_full, 't');
fn = fieldnames(R.ev);
for k = 1:numel(fn)
    a = R.ev.(fn{k}); b = K.ev.(fn{k});
    if isempty(b.t), continue, end
    keep = b.t > max([a.t; -inf]);
    t = [a.t; b.t(keep)]; v = [a.v; b.v(keep)];
    j = [1; find(diff(v) ~= 0) + 1];
    R.ev.(fn{k}) = struct('t', t(j), 'v', v(j));
end
end

function A = local_cat(A, B)
keep = B(:, 1) > A(end, 1);
A = [A; B(keep, :)];
end

function S = local_catS(S, K, tf)
% concatenate struct-of-arrays; fields with the length of t / t_vec are cut at the old end time
tOld = S.(tf)(end);
keep = K.(tf) > tOld;
nT = numel(K.(tf));
hasVec = isfield(K, 't_vec');
if hasVec
    keepV = K.t_vec > S.t_vec(end);
    nV = numel(K.t_vec);
end
fn = fieldnames(K);
for k = 1:numel(fn)
    f = fn{k};
    if hasVec && strcmp(f, 't_vec')
        S.t_vec = [S.t_vec; K.t_vec(keepV)];
    elseif size(K.(f), 1) == nT && ~(hasVec && strcmp(f, 't_vec'))
        if hasVec && size(K.(f), 2) > 1 && size(K.(f), 1) == nV && nV ~= nT
            S.(f) = [S.(f); K.(f)(keepV, :)];
        else
            S.(f) = [S.(f); K.(f)(keep, :)];
        end
    elseif hasVec && size(K.(f), 1) == nV
        S.(f) = [S.(f); K.(f)(keepV, :)];
    end
end
end
