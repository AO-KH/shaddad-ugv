function [same, rep] = shaddad_compare_logs(a, b)
%SHADDAD_COMPARE_LOGS  Bit-identical comparison of the logged signals of two runs (model layout spec 9).
%
%   [same, rep] = shaddad_compare_logs(a, b)
%   a, b   each one of: a run file of shaddad_layout_runs (.mat path, char or string, holding the struct res
%          with field S); a res-style struct (fields S and ok); an S struct in the scn_collect form
%          (S.<log>.<busElement path>.t and .v). S = [] (run_scenario's S of a run without logs) has no
%          leaves. a is the reference: first_t reports its sample times.
%   rep    table, one row per leaf present in either input, sorted by path:
%            .path          string, e.g. "truth.pose2D"
%            .nA, .nB       sample counts (numel of t), 0 when the leaf is missing
%            .identical     t and v both isequaln, with the same class and size (a class change is a difference)
%            .max_abs_diff  max |vA - vB| computed in double when t and v have the same sizes: a position
%                           where both values are NaN counts as equal, one where only one is NaN as Inf;
%                           NaN when the sizes differ or the leaf is missing (and for values without a double)
%            .first_t       time of the first sample where t or v differ; when the common samples agree, the
%                           first time past the shorter run (a missing leaf: the other run's first sample); a
%                           class or width change: the first sample; NaN when identical (and when neither run
%                           has a sample)
%   same   all(rep.identical) (true for two inputs without leaves)
%
%   A leaf is a struct with fields t and v whose t is not a struct (as in scn_collect); every other struct
%   field is walked as a branch. Any other value in S is an error: a value the compare cannot classify would
%   otherwise pass unchecked.

[pA, LA] = local_leaves(local_S(a, 'a'), '', {}, {});
[pB, LB] = local_leaves(local_S(b, 'b'), '', {}, {});
p = reshape(unique([pA; pB]), [], 1);                % sorted
[~, ia] = ismember(p, pA);
[~, ib] = ismember(p, pB);
n = numel(p);
nA = zeros(n, 1);  nB = zeros(n, 1);  ident = false(n, 1);  mad = NaN(n, 1);  ft = NaN(n, 1);
for k = 1:n
    if ia(k) > 0, nA(k) = numel(LA{ia(k)}.t); end
    if ib(k) > 0, nB(k) = numel(LB{ib(k)}.t); end
    if ia(k) > 0 && ib(k) > 0
        [ident(k), mad(k), ft(k)] = local_leaf(LA{ia(k)}, LB{ib(k)});
    elseif ia(k) > 0
        ft(k) = local_tk(LA{ia(k)}.t, [], 1);        % missing in b: from a's first sample
    else
        ft(k) = local_tk([], LB{ib(k)}.t, 1);        % missing in a: from b's first sample
    end
end
rep = table(string(p), nA, nB, ident, mad, ft, ...
    'VariableNames', {'path', 'nA', 'nB', 'identical', 'max_abs_diff', 'first_t'});
same = all(rep.identical);
end

% -------------------------------------------------------------------------------------------
function S = local_S(x, nm)
% the S struct of one input: a run file path, a res-style struct or an S struct ([] = no leaves)
if (ischar(x) && isrow(x)) || (isstring(x) && isscalar(x))
    f = char(x);
    if ~isfile(f), error('shaddad_compare_logs:file', 'Input %s: no file %s', nm, f); end
    if ~any(strcmp(who('-file', f), 'res'))
        error('shaddad_compare_logs:file', 'Input %s: %s holds no variable res', nm, f);
    end
    r = load(f, 'res');
    x = r.res;
    if ~(isstruct(x) && isscalar(x) && isfield(x, 'S'))
        error('shaddad_compare_logs:file', 'Input %s: res in %s has no field S', nm, f);
    end
    S = x.S;
elseif isstruct(x) && isscalar(x) && isfield(x, 'S') && isfield(x, 'ok')
    S = x.S;                                         % res-style struct
elseif (isstruct(x) && isscalar(x)) || (isnumeric(x) && isempty(x))
    S = x;
else
    error('shaddad_compare_logs:input', 'Input %s must be a run file path, a res struct or an S struct', nm);
end
if isempty(S), S = struct(); end                     % a run without logs ([])
if ~(isstruct(S) && isscalar(S))
    error('shaddad_compare_logs:input', 'Input %s: S must be a scalar struct', nm);
end
end

function [p, L] = local_leaves(S, pre, p, L)
% dotted paths and leaves (structs with t and v) of an S struct, depth first
fn = fieldnames(S);
for k = 1:numel(fn)
    x = S.(fn{k});
    if isempty(pre), q = fn{k}; else, q = [pre '.' fn{k}]; end
    if isstruct(x) && isscalar(x) && isfield(x, 't') && isfield(x, 'v') && ~isstruct(x.t)
        p{end+1, 1} = q; %#ok<AGROW>
        L{end+1, 1} = x; %#ok<AGROW>
    elseif isstruct(x) && isscalar(x)
        [p, L] = local_leaves(x, q, p, L);
    else
        error('shaddad_compare_logs:node', '%s is a %s %s, neither a leaf (struct with t and v) nor a struct', ...
            q, mat2str(size(x)), class(x));
    end
end
end

function [ident, mad, ft] = local_leaf(A, B)
% one leaf present in both runs
tA = A.t;  tB = B.t;  vA = A.v;  vB = B.v;
cls = strcmp(class(tA), class(tB)) && strcmp(class(vA), class(vB));
siz = isequal(size(tA), size(tB)) && isequal(size(vA), size(vB));
eqT = isequaln(tA, tB);
eqV = isequaln(vA, vB);
ident = cls && siz && eqT && eqV;
mad = NaN;  ft = NaN;
if siz
    if eqV, mad = 0; else, mad = local_maxdiff(vA, vB); end
end
if ident, return, end
VA = reshape(vA, size(vA, 1), prod(size(vA, 2:ndims(vA))));   % one row per sample (scn_collect: N x numel)
VB = reshape(vB, size(vB, 1), prod(size(vB, 2:ndims(vB))));
if ~cls || size(VA, 2) ~= size(VB, 2)
    k = 1;                                           % a class or width change: every sample differs
else
    m = min([numel(tA), numel(tB), size(VA, 1), size(VB, 1)]);
    k = find(local_rowdiff(tA(:), tB(:), m) | local_rowdiff(VA, VB, m), 1);
    if isempty(k), k = m + 1; end                    % the common samples agree: past the shorter run
end
ft = local_tk(tA, tB, k);
end

function d = local_rowdiff(X, Y, m)
% rows 1..m of X and Y (same class and width) that are not isequaln
X = X(1:m, :);  Y = Y(1:m, :);
try
    e = X == Y;
    if isfloat(X), e = e | (isnan(X) & isnan(Y)); end
    d = ~all(e, 2);
catch %#ok<CTCH> values without an elementwise == (cells, structs)
    d = false(m, 1);
    for r = 1:m, d(r) = ~isequaln(X(r, :), Y(r, :)); end
end
end

function mx = local_maxdiff(vA, vB)
% max |vA - vB| in double over two arrays of one size: NaN in both counts as equal, NaN in one as Inf
try
    x = double(vA(:));  y = double(vB(:));
catch %#ok<CTCH> values without a double form
    mx = NaN;
    return
end
d = abs(x - y);
d(x == y | (isnan(x) & isnan(y))) = 0;               % equal values, equal infinities, NaN in both
d(isnan(d)) = Inf;                                   % NaN in only one of the two
mx = max([0; d]);
end

function t = local_tk(tA, tB, k)
% time of sample k: a's when a has it, else b's, NaN when neither has it
if k <= numel(tA)
    t = double(tA(k));
elseif k <= numel(tB)
    t = double(tB(k));
else
    t = NaN;
end
end
