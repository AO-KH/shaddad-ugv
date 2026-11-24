function ds = scn_make_input(P, tab, T)
%SCN_MAKE_INPUT  Root external input of Shaddad_System (ScenarioIn, Bus_ScenarioIn) from ZOH tables.
%
%   ds = scn_make_input(P, tab, T)
%     P    parameter struct (P.ic.Bus_ScenarioIn gives the element classes / sizes, P.sim.Ts_scen the grid)
%     tab  struct of ZOH tables per field path of Bus_ScenarioIn, e.g.
%             tab.op.tx_power_on = [0 0; 2 1];          % [t value] rows, value held until the next row
%             tab.op.stick_thr   = [0 0; 5 0.6; 30 0];
%             tab.hmi.estop_front_pressed = [0 false; 10 true];
%             tab.obs.xy = [0 reshape(xy0,1,[]); 20 reshape(xy1,1,[])];   % matrix leaf: values in (:)' order
%          or an N x 2 cell {'op.stick_thr', [t v; ...]; ...}. Missing fields = 0 (ground value of the bus).
%     T    StopTime [s] (the last value is held until T)
%     ds   Simulink.SimulationData.Dataset with one element 'ScenarioIn' built by
%          Simulink.SimulationData.createStructOfTimeseries('Bus_ScenarioIn', ...) (build plan T01.8)
%
%   Times are snapped to the 0.01 s grid. The root bus element port interpolates double data and
%   its Interpolate attribute cannot be changed for a bus-object type, so every step is written as
%   two samples at the same time instant (left and right value): the signal is exactly piecewise
%   constant (ZOH) and the tables stay sparse (multi-hour scenarios do not need dense arrays).

if nargin < 2 || isempty(tab), tab = struct(); end
if nargin < 3 || isempty(T), T = 10; end
Ts = P.sim.Ts_scen;
ic = P.ic.Bus_ScenarioIn;

% ---- normalise tab to a list of {path, table} ----
if iscell(tab)
    items = tab;
else
    items = local_flatten(tab, '');
end
if isempty(items)
    items = {'op.tx_power_on', [0 0]};        % defines the time base; value = ground value
end

% ---- full struct of timeseries with the bus hierarchy (createStructOfTimeseries needs every node);
%      leaves without a table hold their ground value over [0 T] (two samples) ----
for k = 1:size(items, 1)
    parts = strsplit(items{k, 1}, '.');
    try
        leaf = getfield(ic, parts{:});
    catch
        error('scn_make_input:path', 'Bus_ScenarioIn has no element %s', items{k, 1});
    end
    if isstruct(leaf)
        error('scn_make_input:leaf', '%s is a bus, give tables for its leaf elements', items{k, 1});
    end
end
S = local_fill(ic, '', items, Ts, T);
tsStruct = Simulink.SimulationData.createStructOfTimeseries('Bus_ScenarioIn', S);
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(tsStruct, 'ScenarioIn');
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
        j = find(strcmp(items(:, 1), p), 1, 'last');
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
if isempty(tbl), tbl = [0 zeros(1, n)]; end
t = double(tbl(:, 1));
v = double(tbl(:, 2:end));
if size(v, 2) == 1 && n > 1, v = repmat(v, 1, n); end
if size(v, 2) ~= n
    error('scn_make_input:width', '%s needs %d value columns, got %d', path, n, size(v, 2));
end
t = round(t / Ts) * Ts;
[t, ord] = sort(t); v = v(ord, :);
% keep the last row of rows that share a time instant
keep = [diff(t) > 0; true];
t = t(keep); v = v(keep, :);
% ZOH as step pairs (left value, right value) at every change instant
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
    data = vv;                                  % N x n, time first (vector element)
else
    data = reshape(vv', [dims N]);              % r x c x N, time last (matrix element)
end
ts = timeseries(data, tt);
if n > 1 && ~(numel(dims) == 2 && dims(2) == 1)
    ts.IsTimeFirst = false;
end
end
