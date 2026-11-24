function [cov, info] = kpi_coverage(S, P, plan, areaPoly, varargin)
%KPI_COVERAGE  Swept-area coverage of the implement (offline KPI, integration spec 5.2, T16).
%
%   cov = kpi_coverage(S, P, plan, areaPoly)
%   [cov, info] = kpi_coverage(..., 'Swath', w, 'Grid', g, 'Depth', d, 'Active', 'impl'|'all')
%     S         collected logs (scn_collect): truth.pose2D (logged), gnc_status.seg_idx
%     P         parameter struct (P.gnc.impl.mower_w is the default swath)
%     plan      Bus_MissionPlan (P.gnc.mission); segment column 6 = impl_mode (> 0 = implement working)
%     areaPoly  N x 2 polygon [x y] of the area to be covered (e.g. the floor inset by c_in, D54)
%   The implement swath (width w across the body, depth d along it, centred on R) is rasterised along
%   the logged pose resampled to 0.02 m path spacing (10 Hz poses at 0.5-1 m/s would leave gaps) on a
%   g = 0.05 m grid, only while the active segment has impl_mode > 0 ('impl', default) or always ('all').
%   cov = covered cells inside areaPoly / cells inside areaPoly (0..1).
%   info: .covered, .total, .grid (logical), .xg, .yg, .n_poses

ip = inputParser;
ip.addParameter('Swath', P.gnc.impl.mower_w);
ip.addParameter('Grid', 0.05);
ip.addParameter('Depth', 0.10);
ip.addParameter('Active', 'impl');
ip.parse(varargin{:});
o = ip.Results;
g = o.Grid;
xg = (min(areaPoly(:, 1)) + g/2):g:max(areaPoly(:, 1));
yg = (min(areaPoly(:, 2)) + g/2):g:max(areaPoly(:, 2));
[X, Y] = meshgrid(xg, yg);
inA = inpolygon(X, Y, areaPoly(:, 1), areaPoly(:, 2));
C = false(size(X));

% poses (10 Hz) and the active flag
ps = S.truth.pose2D;
t10 = (ps.t(1):0.1:ps.t(end))';
pose = scn_metrics('resample', ps, t10);
act = true(size(t10));
if strcmpi(o.Active, 'impl') && isstruct(plan) && isfield(S, 'gnc_status')
    si = double(scn_metrics('resample', S.gnc_status.seg_idx, t10));
    st = double(scn_metrics('resample', S.gnc_status.gnc_state, t10));
    act = false(size(t10));
    ok = si >= 1 & si <= double(plan.n_seg) & st == 2;      % AUTO_MISSION
    act(ok) = double(plan.seg(si(ok), 6)) > 0;
end
% resample to 0.02 m path spacing inside active stretches
P2 = zeros(0, 3);
for k = 1:numel(t10) - 1
    if ~(act(k) && act(k + 1)), continue, end
    a = pose(k, :);  b = pose(k + 1, :);
    d = hypot(b(1) - a(1), b(2) - a(2));
    n = max(1, ceil(d/0.02));
    s = (0:n - 1)'/n;
    dpsi = atan2(sin(b(3) - a(3)), cos(b(3) - a(3)));
    P2 = [P2; a(1) + s*(b(1) - a(1)), a(2) + s*(b(2) - a(2)), a(3) + s*dpsi]; %#ok<AGROW>
end
hw = o.Swath/2;  hd = o.Depth/2;  rr = hypot(hw, hd);
for k = 1:size(P2, 1)
    x = P2(k, 1); y = P2(k, 2); c = cos(P2(k, 3)); s = sin(P2(k, 3));
    ix = find(xg >= x - rr & xg <= x + rr);
    iy = find(yg >= y - rr & yg <= y + rr);
    if isempty(ix) || isempty(iy), continue, end
    dx = X(iy, ix) - x;  dy = Y(iy, ix) - y;
    u = c*dx + s*dy;  v = -s*dx + c*dy;
    C(iy, ix) = C(iy, ix) | (abs(u) <= hd & abs(v) <= hw);
end
info.covered = nnz(C & inA);
info.total = nnz(inA);
info.grid = C;  info.xg = xg;  info.yg = yg;  info.n_poses = size(P2, 1);
cov = info.covered/max(info.total, 1);
end
