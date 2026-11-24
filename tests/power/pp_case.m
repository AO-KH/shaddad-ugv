function R = pp_case(name, varargin)
%PP_CASE  Run a long T05 bench case of pp_case_def with a content-keyed result cache (owner: power, T05).
%
%   R = pp_case(name)                      complete run (or the cached result)
%   R = pp_case(name, 'Force', true)       discard the cache and simulate again
%   R = pp_case(name, 'CacheOnly', true)   never simulate: R = [] when no valid cache exists
%   R = pp_case(name, 'Chunk', dT)         advance by at most dT s of simulated time per call (operating-point
%                                          continuation); R.done is false until the case is complete
%
%   Cache: work/T05/<name>.mat (complete) and work/T05/<name>_run.mat (partial run + operating point), valid
%   while the key (case definition, P after the case modification, Ts, VC, the T04 / T05 generator sources,
%   the bench defaults / runner, the config-set script, the used bus definitions and the MATLAB version) is
%   unchanged. Runs go through pp_run (independent of the shared base workspace; SDI runs cleaned up).
%   R: name, desc, key, done, T_end, wall (s, summed), rtf (= T_end / wall), n_chunks and the logs of pp_run
%   (log_fast, log_slow, log_bms, log_truth; telem / power_truth when the case keeps signal logging).
ip = inputParser;
ip.addParameter('Force', false);
ip.addParameter('CacheOnly', false);
ip.addParameter('Chunk', []);
ip.parse(varargin{:});
o = ip.Results;
C = pp_case_def(name);
if isempty(o.Chunk), o.Chunk = C.chunk; end
key = local_key(C);
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cdir = fullfile(root, 'work', 'T05');
if ~isfolder(cdir), mkdir(cdir); end
cf = fullfile(cdir, [name '.mat']);
sf = fullfile(cdir, [name '_run.mat']);
if o.Force
    if isfile(cf), delete(cf); end
    if isfile(sf), delete(sf); end
end
if isfile(cf)
    S = load(cf, 'R');
    if isfield(S, 'R') && isfield(S.R, 'key') && strcmp(S.R.key, key)
        R = S.R;
        return
    end
end
if o.CacheOnly
    R = [];
    return
end
R = []; op = [];
if isfile(sf)
    S = load(sf, 'R', 'op');
    if isfield(S, 'R') && strcmp(S.R.key, key)
        R = S.R; op = S.op;
    end
end
t0 = 0;
if ~isempty(R), t0 = R.T_end; end
tStop = min(C.T, t0 + o.Chunk);
chunked = ~(t0 == 0 && tStop >= C.T);
args = {'StopTime', tStop, 'VC', C.VC, 'Mode', C.mode, 'SignalLogging', C.siglog};
if ~isempty(C.pmod), args = [args, {'PMod', @(P) local_pmod(P, C.pmod)}]; end
if chunked, args = [args, {'SaveState', true}]; end
if ~isempty(op), args = [args, {'InitialState', op}]; end
Rk = pp_run(C.mdl, C.TB, args{:});
if isempty(R)
    R = Rk;
    R.n_chunks = 1;
else
    R = local_merge(R, Rk);
end
R.name = name; R.desc = C.desc; R.key = key;
R.done = R.T_end >= C.T - 1e-6;
if R.done
    if isfield(R, 'xFinal'), R = rmfield(R, 'xFinal'); end
    R.rtf = R.T_end / R.wall;
    save(cf, 'R', '-v7.3');
    if isfile(sf), delete(sf); end
else
    op = R.xFinal;
    R = rmfield(R, 'xFinal');
    save(sf, 'R', 'op', '-v7.3');
end
end

% -------------------------------------------------------------------------------------------
function P = local_pmod(P, code)
eval(code);
end

function R = local_merge(R, Rk)
% append the logs of chunk Rk (rows after the last time of R)
for v = {'log_fast', 'log_slow', 'log_bms', 'log_truth'}
    if isfield(R, v{1}) && isfield(Rk, v{1})
        a = R.(v{1}); b = Rk.(v{1});
        if ~isempty(a), b = b(b(:, 1) > a(end, 1) + 1e-9, :); end
        R.(v{1}) = [a; b];
    end
end
R.wall = R.wall + Rk.wall;
R.T_end = Rk.T_end;
R.n_chunks = R.n_chunks + 1;
if isfield(Rk, 'xFinal'), R.xFinal = Rk.xFinal; end
end

function key = local_key(C)
[P, Ts, VC] = shaddad_params('Assign', false);
if ~isempty(C.pmod), P = local_pmod(P, C.pmod); end
vn = fieldnames(C.VC);
for k = 1:numel(vn), VC.(vn{k}) = C.VC.(vn{k}); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
files = {'scripts/build/build_power_plant.m', 'scripts/build/build_tb_power_plant.m', 'scripts/build/build_power_pack.m', ...
         'scripts/build/build_shaddad_battery.m', 'scripts/build/build_shaddad_bms.m', 'scripts/build/shaddad_bu.m', ...
         'subsystems/power/bms_mode_manager.m', 'tests/power/tb_power_plant_default.m', 'tests/power/pp_case_def.m', ...
         'tests/power/pp_run.m', 'params/shaddad_configsets.m'};
d = cell(size(files));
for k = 1:numel(files)
    f = fullfile(root, files{k});
    if isfile(f), d{k} = fileread(f); else, d{k} = 'missing'; end
end
defs = shaddad_buses('defs');
bn = {'Bus_BMS_Meas', 'Bus_BMS_Cmd', 'Bus_Can1_Bms', 'Bus_BMS_Telemetry', 'Bus_Can1_McuSlow', 'Bus_CanFrame8', ...
      'Bus_BmsReq', 'Bus_PowerStatus', 'Bus_PowerTruth', 'Bus_BmsStatus', 'Bus_PowerHwCmd', 'Bus_LV_LoadDemand', ...
      'Bus_ImplElecState', 'Bus_Can1_Impl', 'Bus_HmiInputs', 'Bus_FaultInject', 'Bus_EnvState', 'Bus_ImplMechState'};
bd = defs(ismember({defs.name}, bn));
Pk = {P.power, P.sim.ssc, P.drive.tie, P.structure.impl.sprayer, P.ic.Bus_ImplElecState};
blob = double(getByteStreamFromArray({C, Pk, Ts, VC, d, bd, version}));
blob = blob(:);
n = numel(blob);
w1 = mod((1:n)', 65521); w2 = mod((1:n)' * 7919, 65537);
key = sprintf('%d_%.0f_%.0f_%.0f', n, sum(blob), mod(sum(blob .* w1), 2^50), mod(sum(blob .* w2), 2^50));
end
