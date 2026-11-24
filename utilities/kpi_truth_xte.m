function [xte, info] = kpi_truth_xte(S, plan, varargin)
%KPI_TRUTH_XTE  Truth cross-track error against the planned path (offline KPI, integration spec 5.2, T16).
%
%   [xte, info] = kpi_truth_xte(S, plan)
%   [xte, info] = kpi_truth_xte(S, plan, 'Types', [2], 'Rate', 10, 'Window', [t0 t1])
%     S      collected logs: truth.pose2D, gnc_status.seg_idx / gnc_state
%     plan   Bus_MissionPlan (path rows per segment: x y theta dir; seg column 1 = SegType)
%     Types  segment types evaluated (default 2 = ROW)
%   xte      N x 1 signed truth XTE [m] (+ = left of the path direction) at the evaluated samples
%   info     .t sample times, .seg segment index, .rms, .max (abs), .rms_per_seg, .n

ip = inputParser;
ip.addParameter('Types', 2);
ip.addParameter('Rate', 10);
ip.addParameter('Window', [-Inf Inf]);
ip.parse(varargin{:});
o = ip.Results;
ps = S.truth.pose2D;
t = (ps.t(1):1/o.Rate:ps.t(end))';
t = t(t >= o.Window(1) & t <= o.Window(2));
pose = scn_metrics('resample', ps, t);
si = double(scn_metrics('resample', S.gnc_status.seg_idx, t));
st = double(scn_metrics('resample', S.gnc_status.gnc_state, t));
nr = size(plan.path, 1)/size(plan.seg, 1);
ok = si >= 1 & si <= double(plan.n_seg) & st == 2;
ty = zeros(size(si));
ty(ok) = double(plan.seg(si(ok), 1));
use = ok & ismember(ty, o.Types);
xte = nan(nnz(use), 1);
k = 0;
iu = find(use);
for j = iu'
    k = k + 1;
    rows = (si(j) - 1)*nr + (1:nr);
    pth = double(plan.path(rows, 1:2));
    xte(k) = local_xte(pose(j, 1:2), pth);
end
info.t = t(use);  info.seg = si(use);  info.n = numel(xte);
info.rms = sqrt(mean(xte.^2));  info.max = max(abs(xte));
segs = unique(info.seg);
info.rms_per_seg = [segs, arrayfun(@(s) sqrt(mean(xte(info.seg == s).^2)), segs)];
end

function d = local_xte(p, pth)
% signed distance of point p to the polyline pth (closest segment)
best = Inf;  d = NaN;
for i = 1:size(pth, 1) - 1
    a = pth(i, :);  b = pth(i + 1, :);
    ab = b - a;  L2 = ab*ab';
    if L2 < 1e-12, continue, end
    s = min(max(((p - a)*ab')/L2, 0), 1);
    q = a + s*ab;
    e = p - q;
    dist = norm(e);
    if dist < best
        best = dist;
        sg = sign(ab(1)*e(2) - ab(2)*e(1));
        if sg == 0, sg = 1; end
        d = sg*dist;
    end
end
end
