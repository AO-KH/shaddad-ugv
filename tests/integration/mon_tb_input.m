function ds = mon_tb_input(P, spec, T, model)
%MON_TB_INPUT  External-input Dataset for a bench whose root Inports carry buses (T16 test helper).
%
%   ds = mon_tb_input(P, spec, T)            ports of Sim_Monitors (tb_sim_monitors)
%   ds = mon_tb_input(P, spec, T, model)     ports of build_stubs('table') row 'model'
%     spec   struct with one field per port name; each is a struct of [t value] ZOH tables per leaf
%            path, e.g. spec.truth.pose2D = [0 0 0 0; 1 1 0 0]; spec.power_truth.SOC_pack_true = [0 0.9].
%            Missing leaves hold P.ic.<Bus> (ground value).
%     T      StopTime [s]
%   Steps are written as two samples at the same instant (exact ZOH through the interpolating root
%   Inports), as in utilities/scn_make_input.m.

if nargin < 4, model = 'Sim_Monitors'; end
if nargin < 2 || isempty(spec), spec = struct(); end
Tb = build_stubs('table');
R = Tb(strcmp({Tb.name}, model));
ds = Simulink.SimulationData.Dataset;
for k = 1:numel(R.in)
    bus = strtrim(extractAfter(R.in(k).type, 'Bus:'));
    ic = P.ic.(bus);
    items = cell(0, 2);
    if isfield(spec, R.in(k).name)
        items = local_flatten(spec.(R.in(k).name), '');
    end
    S = local_fill(ic, '', items, max(R.in(k).ts, 1e-3), T);
    ts = Simulink.SimulationData.createStructOfTimeseries(bus, S);
    ds = ds.addElement(ts, R.in(k).name);
end
end

% -------------------------------------------------------------------------------------------
function S = local_fill(ic, prefix, items, Ts, T)
S = struct();
fn = fieldnames(ic);
for k = 1:numel(fn)
    p = fn{k};
    if ~isempty(prefix), p = [prefix '.' fn{k}]; end
    v = ic.(fn{k});
    if isstruct(v)
        S.(fn{k}) = local_fill(v, p, items, Ts, T);
    else
        j = [];
        if ~isempty(items), j = find(strcmp(items(:, 1), p), 1, 'last'); end
        if isempty(j)
            tbl = [0 double(v(:))'];
        else
            tbl = items{j, 2};
        end
        S.(fn{k}) = local_ts(tbl, v, Ts, T, p);
        S.(fn{k}).Name = fn{k};
    end
end
end

function items = local_flatten(s, prefix)
items = cell(0, 2);
fn = fieldnames(s);
for k = 1:numel(fn)
    v = s.(fn{k});
    p = fn{k};
    if ~isempty(prefix), p = [prefix '.' fn{k}]; end
    if isstruct(v)
        items = [items; local_flatten(v, p)]; %#ok<AGROW>
    else
        items(end+1, :) = {p, v}; %#ok<AGROW>
    end
end
end

function ts = local_ts(tbl, leaf, Ts, T, path)
dims = size(leaf);
n = numel(leaf);
cls = class(leaf);
t = double(tbl(:, 1));
v = double(tbl(:, 2:end));
if size(v, 2) == 1 && n > 1, v = repmat(v, 1, n); end
if size(v, 2) ~= n
    error('mon_tb_input:width', '%s needs %d value columns, got %d', path, n, size(v, 2));
end
t = round(t / Ts) * Ts;
[t, ord] = sort(t); v = v(ord, :);
keep = [diff(t) > 0; true];
t = t(keep); v = v(keep, :);
if t(1) > 0
    t = [0; t]; v = [double(leaf(:))'; v];
end
tt = t(1); vv = v(1, :);
for k = 2:numel(t)
    tt = [tt; t(k); t(k)]; %#ok<AGROW>
    vv = [vv; v(k - 1, :); v(k, :)]; %#ok<AGROW>
end
if T > tt(end)
    tt(end+1, 1) = T; vv(end+1, :) = vv(end, :);
end
N = numel(tt);
if islogical(leaf), vv = vv ~= 0; else, vv = cast(vv, cls); end
if n == 1
    data = vv(:);
elseif numel(dims) == 2 && dims(2) == 1
    data = vv;
else
    data = reshape(vv', [dims N]);
end
ts = timeseries(data, tt);
if n > 1 && ~(numel(dims) == 2 && dims(2) == 1)
    ts.IsTimeFirst = false;
end
end
