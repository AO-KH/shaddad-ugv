function res = eval_criteria(out, crit, varargin)
%EVAL_CRITERIA  Evaluate scenario pass/fail criteria on logged data (integration spec 5.2 / 7.8, T16).
%
%   res = eval_criteria(out, crit)
%   res = eval_criteria(out, crit, 'Print', false)
%     out   Simulink.SimulationOutput, logsout Dataset, or a collected struct S (scn_collect)
%     crit  struct array (scenario scripts scn_S<k>_*.m), fields
%             id      short id, e.g. 'S1.rise'
%             desc    text of the criterion (architecture section 12)
%             signal  logged source(s), informative
%             metric  function handle @(S) -> scalar value (S = scn_collect output)
%             op      '<=' | '<' | '>=' | '>' | '==' | 'in' (limit = [lo hi]) | 'true' (value must be true)
%             limit   numeric limit
%             unit    unit text
%             window  optional [t0 t1] (informative, the metric applies it)
%             blocked optional text: when not empty the criterion is reported BLOCKED (not evaluated)
%   res   struct array: id, desc, value, op, limit, unit, status ('PASS' | 'FAIL' | 'BLOCKED' | 'ERROR'),
%         pass (logical; BLOCKED and ERROR count as not passed), msg
%   A table of the results is printed unless 'Print' is false.

ip = inputParser;
ip.addParameter('Print', true);
ip.parse(varargin{:});
if isa(out, 'Simulink.SimulationOutput') || isa(out, 'Simulink.SimulationData.Dataset')
    S = scn_collect(out);
else
    S = out;
end
res = repmat(struct('id', '', 'desc', '', 'value', NaN, 'op', '', 'limit', [], 'unit', '', ...
    'status', '', 'pass', false, 'msg', ''), numel(crit), 1);
for k = 1:numel(crit)
    c = crit(k);
    r = res(k);
    r.id = c.id;  r.desc = local_get(c, 'desc', '');  r.op = c.op;  r.limit = c.limit;
    r.unit = local_get(c, 'unit', '');
    blk = local_get(c, 'blocked', '');
    if ~isempty(blk)
        r.status = 'BLOCKED';  r.msg = blk;
        try
            v = double(c.metric(S));                   % still reported when it can be measured
            if ~isempty(v), r.value = v(1); end
        catch
        end
        res(k) = r;
        continue
    end
    try
        v = double(c.metric(S));
        if isempty(v), v = NaN; end
        r.value = v(1);
        r.pass = local_cmp(r.value, c.op, c.limit);
        if r.pass, r.status = 'PASS'; else, r.status = 'FAIL'; end
    catch tmp_ec_err
        r.status = 'ERROR';  r.msg = tmp_ec_err.message;  r.pass = false;
    end
    res(k) = r;
end
if ip.Results.Print
    local_print(res);
end
end

% -------------------------------------------------------------------------------------------
function tf = local_cmp(v, op, lim)
if isempty(v) || any(isnan(v))
    tf = false;
    return
end
v = v(1);
switch op
    case '<=', tf = v <= lim;
    case '<',  tf = v < lim;
    case '>=', tf = v >= lim;
    case '>',  tf = v > lim;
    case '==', tf = v == lim;
    case 'in', tf = v >= lim(1) && v <= lim(2);
    case 'true', tf = logical(v);
    otherwise, error('eval_criteria:op', 'Unknown op %s', op);
end
end

function v = local_get(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function local_print(res)
fprintf('%-18s %-8s %14s %4s %-16s %s\n', 'criterion', 'status', 'value', 'op', 'limit', 'description');
for k = 1:numel(res)
    r = res(k);
    if isnumeric(r.limit), lim = mat2str(r.limit, 4); else, lim = char(string(r.limit)); end
    fprintf('%-18s %-8s %14.5g %4s %-16s %s %s\n', r.id, r.status, r.value(1), r.op, [lim ' ' r.unit], ...
        r.desc, r.msg);
end
end
