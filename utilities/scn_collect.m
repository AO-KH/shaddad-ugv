function S = scn_collect(src, S0)
%SCN_COLLECT  Logged scenario signals as plain arrays (owner: integration, T16).
%
%   S = scn_collect(logsout)          logsout: Simulink.SimulationData.Dataset (or a SimulationOutput)
%   S = scn_collect(logsout, S0)      append the samples to an earlier collection S0 (chunked runs that
%                                     continue from an operating point; samples at t <= the last time of
%                                     S0 are dropped)
%   S.<logName>.<busElement path>.t   N x 1 time [s]
%   S.<logName>.<busElement path>.v   N x numel(element) data (time first, column-major per sample)
%   Leaves keep their class (logical, integers, single, double).

if isa(src, 'Simulink.SimulationOutput')
    src = src.logsout;
end
S = struct();
if ~isempty(src) && isa(src, 'Simulink.SimulationData.Dataset')
    for k = 1:src.numElements
        e = src.getElement(k);
        nm = matlab.lang.makeValidName(e.Name);
        S.(nm) = local_leaf(e.Values);
    end
end
if nargin > 1 && ~isempty(S0)
    S = local_append(S0, S);
end
end

% -------------------------------------------------------------------------------------------
function r = local_leaf(v)
if isstruct(v)
    r = struct();
    fn = fieldnames(v);
    for k = 1:numel(fn)
        r.(fn{k}) = local_leaf(v.(fn{k}));
    end
elseif isa(v, 'timeseries')
    t = v.Time(:);
    d = v.Data;
    n = numel(t);
    if n == 0
        r = struct('t', t, 'v', d);
        return
    end
    if v.IsTimeFirst
        r = struct('t', t, 'v', reshape(d, n, []));
    else
        nd = ndims(d);
        d = permute(d, [nd 1:nd-1]);
        r = struct('t', t, 'v', reshape(d, n, []));
    end
else
    r = v;
end
end

function A = local_append(A, B)
fn = fieldnames(B);
for k = 1:numel(fn)
    f = fn{k};
    if ~isfield(A, f)
        A.(f) = B.(f);
    elseif isstruct(B.(f)) && isfield(B.(f), 't') && isfield(B.(f), 'v') && ~isstruct(B.(f).t)
        tl = A.(f).t;
        if isempty(tl)
            A.(f) = B.(f);
        else
            keep = B.(f).t > tl(end) + 1e-9;
            A.(f).t = [tl; B.(f).t(keep)];
            A.(f).v = [A.(f).v; B.(f).v(keep, :)];
        end
    elseif isstruct(B.(f))
        A.(f) = local_append(A.(f), B.(f));
    end
end
end
