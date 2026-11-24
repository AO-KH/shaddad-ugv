function [obsCount, obsPos, obsRadius, obsRange, cl, nCl] = sens_obstacle_extract(rangesF, angles, xB, yB, c)
%SENS_OBSTACLE_EXTRACT  Obstacle list by adaptive break-point segmentation (sensors spec 3.7 / F9, D59).
%#codegen
%   [obsCount, obsPos, obsRadius, obsRange, cl, nCl] = sens_obstacle_extract(rangesF, angles, xB, yB, c)
%     rangesF  single[360] filtered ranges, LiDAR frame (NaN = no return / removed); angles single[360]
%     xB, yB   double[360] the same points in base_link (NaN where rangesF is NaN), from sens_scan_filter
%     c        sens_per_prm(P, 'obs') = [segLambda sigR minPts minPtsFarRange maxObs maxRadius compactR nClMax
%              rLx rLy psiL beamRes]
%     obsCount uint8 number of valid rows (<= maxObs = 20), sorted by range
%     obsPos   single[20 x 2] obstacle centres in base_link (rows > obsCount are 0)
%     obsRadius, obsRange   single[20] radius and minimum range from base_link
%     cl       double[nClMax x 4] clusters [x y radius range] (base_link, the nClMax nearest, sorted by range;
%              row detection)
%     nCl      number of rows of cl
%
%   Only the segmentation loop is code (D59); the scan filter, sector minima and protective field are blocks.
%   Segmentation (circular scan, beams in angular order): consecutive returns p(k-1), p(k) with angular gap dphi
%   belong to one segment when |p(k) - p(k-1)| <= r(k-1) sin(dphi)/sin(lambda - dphi) + 3 sigR (Borges and
%   Aldon adaptive break point; a gap dphi >= lambda always breaks) and |p(k) - p(first)| <= 2 maxRadius (long
%   walls are cut into pieces of at most 2 maxRadius). A segment is a cluster when it has >= minPts points, or
%   >= 1 point beyond minPtsFarRange (where a 0.3 m object subtends less than two 1-degree beams).
%   Cluster radius = half of (chord first-last + one beam spacing at its range: the object extends on average half a
%   beam beyond each outermost hit); range = min |p| in base_link;
%   centre = for compact clusters (radius <= compactR) the point at (min LiDAR range + radius) along the
%   bearing of the centroid seen from the LiDAR (removes the visible-face bias of the centroid), otherwise the
%   centroid.

lam = c(1);  sigR = c(2);  minPts = c(3);  farR = c(4);  maxObs = c(5);  maxRad = c(6);  compactR = c(7);
nClMax = c(8);  rL = [c(9) c(10)];  dRes = c(12);
nB = numel(rangesF);
NCL = 60;                                   % fixed table size (codegen); nClMax <= 60
obsCount = uint8(0);
obsPos = zeros(20, 2, 'single');  obsRadius = zeros(20, 1, 'single');  obsRange = zeros(20, 1, 'single');
cl = zeros(NCL, 4);  nCl = 0;
nMax = min(NCL, nClMax);

r = double(rangesF(:));
a = double(angles(:));
ok = ~isnan(r);
if ~any(ok)
    return
end
% start just after a break: the first valid beam whose previous valid beam is not connected
kStart = 0;
for k = 1:nB
    if ok(k)
        kp = local_prev_valid(ok, k, nB);
        if ~local_connected(r, xB, yB, kp, k, a, lam, sigR, k, maxRad)
            kStart = k;
            break
        end
    end
end
if kStart == 0                              % fully connected ring: start at the first valid beam
    for k = nB:-1:1
        if ok(k), kStart = k; end
    end
end

segFirst = kStart;  segN = 0;  prevK = 0;
sx = 0;  sy = 0;  rminL = Inf;  rminB = Inf;
for i = 0:nB - 1
    k = mod(kStart - 1 + i, nB) + 1;
    if ~ok(k), continue, end
    if segN > 0 && ~local_connected(r, xB, yB, prevK, k, a, lam, sigR, segFirst, maxRad)
        [cl, nCl] = local_close(cl, nCl, nMax, segFirst, prevK, segN, sx, sy, rminL, rminB, xB, yB, ...
                                minPts, farR, compactR, rL, dRes);
        segN = 0;
    end
    if segN == 0
        segFirst = k;  sx = 0;  sy = 0;  rminL = Inf;  rminB = Inf;
    end
    segN = segN + 1;
    sx = sx + xB(k);  sy = sy + yB(k);
    rminL = min(rminL, r(k));
    rminB = min(rminB, hypot(xB(k), yB(k)));
    prevK = k;
end
if segN > 0
    [cl, nCl] = local_close(cl, nCl, nMax, segFirst, prevK, segN, sx, sy, rminL, rminB, xB, yB, ...
                            minPts, farR, compactR, rL, dRes);
end

% sort by range (insertion sort, nCl <= 60)
for i = 2:nCl
    row = cl(i, :);  j = i - 1;
    while j >= 1 && cl(j, 4) > row(4)
        cl(j + 1, :) = cl(j, :);  j = j - 1;
    end
    cl(j + 1, :) = row;
end
nOut = min(nCl, min(maxObs, 20));
for i = 1:nOut
    obsPos(i, 1) = single(cl(i, 1));  obsPos(i, 2) = single(cl(i, 2));
    obsRadius(i) = single(cl(i, 3));  obsRange(i) = single(cl(i, 4));
end
obsCount = uint8(nOut);
end

% -------------------------------------------------------------------------------------------
function kp = local_prev_valid(ok, k, nB)
kp = 0;
for i = 1:nB - 1
    j = mod(k - 1 - i, nB) + 1;
    if ok(j), kp = j; return, end
end
end

function tf = local_connected(r, xB, yB, kp, k, a, lam, sigR, kFirst, maxRad)
% adaptive break-point test between consecutive valid beams kp -> k, plus the chord limit from kFirst
tf = false;
if kp == 0 || kp == k, return, end
dphi = mod(a(k) - a(kp), 2*pi);                % angular gap between the two returns
if dphi >= lam, return, end
dmax = r(kp)*sin(dphi)/sin(lam - dphi) + 3*sigR;
d = hypot(xB(k) - xB(kp), yB(k) - yB(kp));
if d > dmax, return, end
tf = hypot(xB(k) - xB(kFirst), yB(k) - yB(kFirst)) <= 2*maxRad;
end

function [cl, nCl] = local_close(cl, nCl, nMax, kF, kL, n, sx, sy, rminL, rminB, xB, yB, minPts, farR, ...
                                 compactR, rL, dRes)
% finish one segment: cluster test, radius, centre; a full table keeps the nMax nearest clusters
if n < minPts && ~(n >= 1 && rminB > farR), return, end
cx = sx/n;  cy = sy/n;
rad = 0.5*(hypot(xB(kL) - xB(kF), yB(kL) - yB(kF)) + rminL*dRes);   % + half a beam spacing at each end
if rad <= compactR
    b = atan2(cy - rL(2), cx - rL(1));
    cx = rL(1) + (rminL + rad)*cos(b);
    cy = rL(2) + (rminL + rad)*sin(b);
end
if nCl < nMax
    nCl = nCl + 1;
    cl(nCl, :) = [cx cy rad rminB];
else
    % table full: replace the farthest cluster when this one is nearer
    jMax = 1;
    for j = 2:nCl
        if cl(j, 4) > cl(jMax, 4), jMax = j; end
    end
    if rminB < cl(jMax, 4)
        cl(jMax, :) = [cx cy rad rminB];
    end
end
end
