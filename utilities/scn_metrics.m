function out = scn_metrics(op, varargin)
%SCN_METRICS  Small metric helpers for scenario criteria (T16). sig = struct('t', N x 1, 'v', N x m).
%
%   t   = scn_metrics('first', sig, cond, t0)      first time >= t0 where cond(v_row) is true (NaN if never)
%   t   = scn_metrics('last_true', sig, cond, t0)   last time >= t0 where cond(v) is true (NaN if never)
%   v   = scn_metrics('at', sig, t)                value at time t (zero-order hold, last sample <= t)
%   [t,v] = scn_metrics('win', sig, t0, t1)        samples with t0 <= t <= t1
%   b   = scn_metrics('bit', v, k)                 bit k (0-based) of an integer word, logical
%   w   = scn_metrics('pulse', sig, cond, t0)      width of the first interval where cond holds, starting >= t0
%   y   = scn_metrics('movmean', t, v, T)          trailing moving average over T seconds (uniform sampling)
%   d   = scn_metrics('path', pos, t0, t1)         path length of pos (N x >= 2) between t0 and t1
%   v   = scn_metrics('resample', sig, t)          ZOH resample of sig onto times t

switch lower(op)
    case 'first'
        [sig, cond, t0] = varargin{1:3};
        k = find(sig.t >= t0 - 1e-9 & local_cond(cond, sig.v), 1, 'first');
        if isempty(k), out = NaN; else, out = sig.t(k); end
    case 'last_true'
        [sig, cond, t0] = varargin{1:3};
        k = find(sig.t >= t0 - 1e-9 & local_cond(cond, sig.v), 1, 'last');
        if isempty(k), out = NaN; else, out = sig.t(k); end
    case 'at'
        [sig, t] = varargin{1:2};
        k = find(sig.t <= t + 1e-9, 1, 'last');
        if isempty(k), k = 1; end
        out = sig.v(k, :);
    case 'win'
        [sig, t0, t1] = varargin{1:3};
        m = sig.t >= t0 - 1e-9 & sig.t <= t1 + 1e-9;
        out = sig.t(m);
        if nargout > 1, out = {out, sig.v(m, :)}; end
    case 'bit'
        [v, k] = varargin{1:2};
        out = bitget(uint32(double(v)), k + 1) ~= 0;
    case 'pulse'
        [sig, cond, t0] = varargin{1:3};
        c = local_cond(cond, sig.v) & sig.t >= t0 - 1e-9;
        k1 = find(c, 1, 'first');
        if isempty(k1), out = NaN; return, end
        k2 = find(~c(k1:end), 1, 'first');
        if isempty(k2), out = Inf; return, end
        out = sig.t(k1 + k2 - 1) - sig.t(k1);
    case 'movmean'
        [t, v, T] = varargin{1:3};
        dt = median(diff(t));
        n = max(1, round(T/dt));
        out = movmean(v, [n - 1 0]);
    case 'path'
        [pos, t0, t1] = varargin{1:3};
        m = pos.t >= t0 - 1e-9 & pos.t <= t1 + 1e-9;
        p = pos.v(m, 1:2);
        out = sum(sqrt(sum(diff(p).^2, 2)));
    case 'resample'
        [sig, t] = varargin{1:2};
        n = numel(sig.t);
        if n == 1
            idx = ones(numel(t), 1);
        else
            idx = interp1(sig.t(:) + (0:n-1)'*1e-12, (1:n)', t(:) + 1e-9, 'previous', 'extrap');
            idx(isnan(idx) | t(:) < sig.t(1)) = 1;
            idx(t(:) >= sig.t(end)) = n;
        end
        out = sig.v(idx, :);
    otherwise
        error('scn_metrics:op', 'Unknown op %s', op);
end
end

function c = local_cond(cond, v)
c = false(size(v, 1), 1);
r = cond(v);
c(:) = r(:, 1);
end
