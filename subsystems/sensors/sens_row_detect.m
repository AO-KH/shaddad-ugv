function row = sens_row_detect(cl, nCl, c)
%SENS_ROW_DETECT  Row geometry (palm trunks / solar piles) from the obstacle clusters (sensors spec 3.7 / F10).
%#codegen
%   row = sens_row_detect(cl, nCl, c)
%     cl, nCl  cluster table [x y radius range] in base_link (sens_obstacle_extract), rows 1..nCl
%     c        sens_per_prm(P, 'row') = [rMin rMax xMin xMax yMin yMax inlier iter minInl seed]
%     row      Bus_RowFeature: valid, lateralOffset (+ = robot left of the row centre), headingErr (robot heading
%              minus row direction, + = robot yawed CCW), distLeft, distRight (perpendicular distances from
%              base_link to the fitted lines, > 0), nLeft, nRight (inliers), fitRms (inlier residual rms)
%
%   Candidates: clusters with radius in [rMin, rMax], x in [xMin, xMax], |y| in [yMin, yMax]; y > 0 left side,
%   y < 0 right side. RANSAC line fit per side with a fixed number of hypotheses (iter): when the side has at
%   most iter point pairs every pair is a hypothesis (deterministic), otherwise iter pairs are drawn with a
%   coder-safe linear congruential generator re-seeded with seed every scan. Score = inliers (perpendicular
%   distance < inlier), ties -> smaller residual. The best hypothesis is refined by an orthogonal least-squares
%   fit of its inliers. A side is valid with >= minInl inliers; row.valid needs both sides. Heading uses the
%   inlier-weighted mean direction of the two lines; lateralOffset = (distRight - distLeft)/2.

row = struct('valid', false, 'lateralOffset', single(0), 'headingErr', single(0), 'distLeft', single(0), ...
             'distRight', single(0), 'nLeft', uint8(0), 'nRight', uint8(0), 'fitRms', single(0));
NC = size(cl, 1);
xs = zeros(NC, 1);  ys = zeros(NC, 1);  side = zeros(NC, 1);  n = 0;
for i = 1:min(nCl, NC)
    x = cl(i, 1);  y = cl(i, 2);  rd = cl(i, 3);
    if rd >= c(1) && rd <= c(2) && x >= c(3) && x <= c(4) && abs(y) >= c(5) && abs(y) <= c(6)
        n = n + 1;  xs(n) = x;  ys(n) = y;  side(n) = sign(y);
    end
end
[okL, thL, dL, nL, ssL] = local_side(xs, ys, side, n, 1, c);
[okR, thR, dR, nR, ssR] = local_side(xs, ys, side, n, -1, c);
row.nLeft = uint8(nL);  row.nRight = uint8(nR);
if okL, row.distLeft = single(abs(dL)); end
if okR, row.distRight = single(abs(dR)); end
if okL && okR
    % mean direction of the two (nearly parallel) lines, weighted by inliers
    th = atan2(nL*sin(thL) + nR*sin(thR), nL*cos(thL) + nR*cos(thR));
    row.valid = true;
    row.headingErr = single(-th);
    row.lateralOffset = single((abs(dR) - abs(dL))/2);
    row.fitRms = single(sqrt((ssL + ssR)/(nL + nR)));
end
end

% -------------------------------------------------------------------------------------------
function [ok, th, d, nIn, ss] = local_side(xs, ys, side, n, sgn, c)
% RANSAC + orthogonal refinement on one side. Line: points p with n'(p) = d, direction angle th (body frame,
% folded to (-pi/2, pi/2]), unit normal nrm = [-sin th, cos th]; d = signed offset of base_link.
ok = false;  th = 0;  d = 0;  nIn = 0;  ss = 0;
NC = numel(xs);
px = zeros(NC, 1);  py = zeros(NC, 1);  m = 0;
for i = 1:n
    if side(i) == sgn, m = m + 1;  px(m) = xs(i);  py(m) = ys(i); end
end
if m < max(2, c(9)), nIn = m; return, end
tol = c(7);  iter = round(c(8));
nPairs = m*(m - 1)/2;
bestN = 0;  bestS = Inf;  bestI = 1;  bestJ = 2;
s = uint32(c(10));
h = 0;  i = 1;  j = 1;
while h < iter
    if nPairs <= iter
        % enumerate the pairs (i < j)
        j = j + 1;
        if j > m, i = i + 1;  j = i + 1; end
        if i >= m, break, end
    else
        [s, i] = local_lcg(s, m);
        [s, j] = local_lcg(s, m - 1);
        if j >= i, j = j + 1; end
    end
    h = h + 1;
    dx = px(j) - px(i);  dy = py(j) - py(i);  L = hypot(dx, dy);
    if L < 1e-6, continue, end
    nx = -dy/L;  ny = dx/L;  dd = nx*px(i) + ny*py(i);
    cnt = 0;  sr = 0;
    for k = 1:m
        e = abs(nx*px(k) + ny*py(k) - dd);
        if e < tol, cnt = cnt + 1;  sr = sr + e^2; end
    end
    if cnt > bestN || (cnt == bestN && sr < bestS)
        bestN = cnt;  bestS = sr;  bestI = i;  bestJ = j;
    end
end
% inliers of the best hypothesis -> orthogonal least squares
dx = px(bestJ) - px(bestI);  dy = py(bestJ) - py(bestI);  L = max(hypot(dx, dy), 1e-9);
nx = -dy/L;  ny = dx/L;  dd = nx*px(bestI) + ny*py(bestI);
inl = false(NC, 1);
for k = 1:m
    inl(k) = abs(nx*px(k) + ny*py(k) - dd) < tol;
end
nIn = sum(inl);
if nIn < c(9), return, end
mx = sum(px(inl))/nIn;  my = sum(py(inl))/nIn;
sxx = sum((px(inl) - mx).^2);  syy = sum((py(inl) - my).^2);  sxy = sum((px(inl) - mx).*(py(inl) - my));
th = 0.5*atan2(2*sxy, sxx - syy);           % principal direction
if th > pi/2, th = th - pi; elseif th <= -pi/2, th = th + pi; end
nx = -sin(th);  ny = cos(th);
d = nx*mx + ny*my;                          % signed offset; base_link side is n'p = 0
ss = sum((nx*px(inl) + ny*py(inl) - d).^2);
ok = true;
end

function [s, v] = local_lcg(s, m)
% Numerical Recipes LCG (uint32 arithmetic in double to stay overflow-free), value in 1..m
s = uint32(mod(1664525*double(s) + 1013904223, 4294967296));
v = floor(double(s)/4294967296*m) + 1;
if v > m, v = m; end
end
