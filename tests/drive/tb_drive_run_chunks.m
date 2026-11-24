function [out, op, wall] = tb_drive_run_chunks(model, t1, chunk, mwVars, globVars, modelParams, op, out)
%TB_DRIVE_RUN_CHUNKS  Long T06 bench run split into chunks joined by model operating points (T5, T8, T17).
%   [out, op, wall] = tb_drive_run_chunks(model, t1, chunk, mwVars, globVars, modelParams)
%       simulate from 0 to t1 in pieces of 'chunk' seconds (Accelerator unless modelParams says otherwise)
%   [out, op, wall] = tb_drive_run_chunks(..., op, out)
%       continue a previous call from its final operating point op, appending to its outputs out
%   out  : struct with the logged bench outputs (fb, truth, ... as timeseries / structs of timeseries),
%          concatenated over the chunks; op: final operating point; wall: wall-clock of this call (s).
%   The continuation is bit-exact (verified on tb_drive: 2 s + 2 s vs 4 s, max |dw| 4e-15 rad/s), so a 2 h
%   thermal run can be spread over several bounded MATLAB calls.
if nargin < 4 || isempty(mwVars), mwVars = struct(); end
if nargin < 5 || isempty(globVars), globVars = struct(); end
if nargin < 6 || isempty(modelParams), modelParams = {'SimulationMode', 'accelerator'}; end
if nargin < 7, op = []; end
if nargin < 8, out = struct(); end
mp = [modelParams, {'SaveFinalState', 'on', 'SaveOperatingPoint', 'on', 'FinalStateName', 'xFinal'}];
if isempty(op), t = 0; else, t = op.snapshotTime; end
w0 = tic;
while t < t1 - 1e-9
    te = min(t + chunk, t1);
    if isempty(op)
        so = tb_drive_run(model, te, mwVars, globVars, mp);
    else
        so = tb_drive_run(model, te, mwVars, globVars, [mp, {'InitialState', op}]);
    end
    op = so.xFinal;
    names = setdiff(so.who, {'xFinal', 'tout', 'logsout', 'xout', 'yout'});
    for k = 1:numel(names)
        v = so.(names{k});
        if isfield(out, names{k}), out.(names{k}) = local_cat(out.(names{k}), v); else, out.(names{k}) = v; end
    end
    t = te;
end
wall = toc(w0);
end

function a = local_cat(a, b)
% append b to a (timeseries or nested struct of timeseries); the first sample of b repeats the last of a
if isa(a, 'timeseries')
    tb = b.Time; db = b.Data;
    keep = tb > a.Time(end) + 1e-12;
    if ~any(keep), return; end
    da = a.Data; ta = a.Time;
    nd = ndims(db);
    if isvector(db) || (nd == 2 && size(db, 2) == 1)
        a = timeseries([da(:); reshape(db(keep), [], 1)], [ta; tb(keep)], 'Name', a.Name);
    else
        idx = repmat({':'}, 1, nd); idx{nd} = keep;           % time is the last dimension
        a = timeseries(cat(nd, da, db(idx{:})), [ta; tb(keep)], 'Name', a.Name);
    end
elseif isstruct(a)
    fn = fieldnames(a);
    for k = 1:numel(fn), a.(fn{k}) = local_cat(a.(fn{k}), b.(fn{k})); end
end
end
