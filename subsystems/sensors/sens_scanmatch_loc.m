function [sm, rel, nRef, geo] = sens_scanmatch_loc(poseNav, ranges, angles, tScan, grid, c)
%SENS_SCANMATCH_LOC  Scan-to-prior-map localisation of base_link (sensors spec 3.7 / F11, D55).
%#codegen
%   [sm, rel, nRef, geo] = sens_scanmatch_loc(poseNav, ranges, angles, tScan, grid, c)
%     poseNav  [x_E y_N psi] predicted base_link pose (Bus_NavState pos_W(1:2), eul(1))
%     ranges   single[360] filtered scan (NaN = no return), LiDAR frame; angles single[360]
%     tScan    scan time [s]
%     grid     prior occupancy grid (logical, row 1 = max y; sens_per_prior_map(P.world, P.sensors.per))
%     c        sens_per_prm(P, 'sm'):
%              [mapRes x0 y0 smRes smMaxRange smTransX smTransY smRot minScoreRatio nMin rLx rLy psiL
%               floorX floorY floorPsi rMin beamRes geoGap geoDmax]
%     sm       Bus_ScanMatchMeas: valid, t, pose [x_E y_N psi] of base_link, cov (W, 3x3), score, nPts
%     rel      relative pose of the current scan in the predicted LiDAR frame (tests)
%     nRef     points of the virtual reference scan inside [1/smRes, smMaxRange] (tests)
%     geo      weak-axis correction of the covariance (tests): [r sigWeakIn sigWeakOut applied nCorr]
%
%   At the first call the prior grid becomes a binaryOccupancyMap (mapRes cells/m, lower-left corner [x0 y0])
%   and a noise-free rangeSensor with the 360 beams of the scan (range [rMin smMaxRange]) is created.
%   Each call: predicted LiDAR pose = poseNav (+) mount; virtual reference scan of the prior map at that pose;
%   matchScansGrid(cur, ref, 'InitialPose', [0 0 0], 'Resolution', smRes, 'MaxRange', smMaxRange,
%   'TranslationSearchRange', smTrans, 'RotationSearchRange', smRot).
%   Notes on matchScansGrid (R2026b, read from nav.algs.internal.CorrelativeScanMatcher):
%   * The search window is used only with an InitialPose (without it the whole 2*MaxRange square and all
%     angles are searched). The branch-and-bound start level is 5, so the translation window is snapped to
%     2^5/smRes = 3.2 m coarse cells: the effective window is one coarse cell (3.2 m) around the snapped guess.
%   * The returned angle is one angular step too large: candidate i is rotated by (i-1)*dTh but reported as
%     i*dTh, dTh = 2*pi/(ceil(2*pi/acos(1 - (1/smRes)^2/(2 smMaxRange^2))) - 1) (0.0100 rad at 10 / 10 m).
%     The bias is removed here (measured +0.0111 rad mean over 30 synthetic matches, 0.0100 expected).
%   * stats.Covariance (estimateCovariance) samples the score in +-10 cells and +-50 angle steps around the best
%     candidate, keeps the samples above 60 % of the best score and weights them by the score. Along a direction
%     that fewer than about 40 % of the points constrain (a corridor: posts, door jambs, end walls) every sample
%     passes, so the reported variance is the sampling window's (sigma about 0.5-0.6 m at 10 cells/m) whatever the
%     constraint. In the S4 greenhouse aisle the match is good to 0.07 m RMS along the aisle against a reported
%     0.52 m (S4 fix 2, 2026-11-14), and the EKF ignored the along-aisle position.
%   * Along the aisle the result keeps part of a start error ahead of the truth (S4 replay, logged scans: from
%     0.15 / 0.3 m ahead one third remains, from 0.45-0.6 m ahead deeper than 6 m the start comes back), while a
%     start behind the truth is recovered from 0.6 m. sens_nav_gate therefore down-weights a weak-axis match that
%     did not move from an uncertain start (navgate.smCap).
%   valid = |rel(1:2)| < 0.9 * effective window, |rel(3)| < 0.9 smRot, score / nCur >= minScoreRatio and both
%   scans have >= nMin points inside [1/smRes, smMaxRange] (nCur, nRef).
%   Pose: base_link = (predicted LiDAR (+) rel) (+) inverse(mount).
%   Covariance: stats.Covariance (predicted LiDAR frame) with the weak translation axis from the scan geometry,
%   rotated to W and propagated through the lever arm, diagonal floored at [floorX floorY floorPsi].
%   Weak axis: when the larger eigenvalue lam2 of the translation block exceeds a quarter of the sampling-window
%   variance ((21^2 - 1)/12/smRes^2), lam2 is replaced by lam1 * r: the matcher's strong-axis variance times the
%   anisotropy r = max(g2/g1, 1) of the point information F of the matched scan against the reference scan, g = v'
%   inv(F) v along the two eigenvectors (each current point moved by rel with a reference point within geoDmax on
%   the same beam adds J'J: a line point, both reference neighbours within geoGap, J = [n', n x p] with the chord
%   normal n; an isolated point (post, edge, jamb) both axes, [I2, [-py; px]]). The result is capped at the effective
%   search window squared, so a direction without information is ignored by the EKF; with fewer than nMin
%   correspondences the matcher's value is kept. The weak axis is scaled with a congruence transform, which keeps its
%   correlation with the heading and the matrix positive definite.
%   Revision (S4 nav fix round 2, 2026-11-14): this weak axis replaces the sampling-window value. On 349 logged S4
%   aisle scans it is 0.15 m (median) against 0.085-0.093 m RMS along-aisle error, a per-scan NEES of 0.2; the
%   along-aisle errors of consecutive matches are correlated over about 3 matches while driving (more at a
%   standstill), which this margin and smInfl = 2 cover (tests: t_perception/tSM01_AlongAisleHonesty).

persistent mp rs
if isempty(mp)
    mp = binaryOccupancyMap(grid, c(1));
    mp.GridLocationInWorld = [c(2) c(3)];
    % beam layout of Bus_LidarScanRaw (ICD 10: 360 beams, beam k at -pi + (k-1) deg); rangeSensor needs the
    % angles as compile-time constants (non-tunable properties)
    rs = rangeSensor('HorizontalAngle', [-pi, pi - pi/180], 'HorizontalAngleResolution', pi/180, 'RangeNoise', 0);
    rs.Range = [c(17) c(5)];
end
smRes = c(4);  maxR = c(5);  smTrans = [c(6) c(7)];  smRot = c(8);
rL = [c(11); c(12)];  psiL = c(13);

sm = struct('valid', false, 't', tScan, 'pose', zeros(3, 1), 'cov', zeros(3, 3), 'score', 0, 'nPts', uint16(0));
rel = zeros(1, 3);
geo = zeros(1, 5);

% predicted LiDAR pose in W
th = poseNav(3);
Rb = [cos(th) -sin(th); sin(th) cos(th)];
pL = [poseNav(1); poseNav(2)] + Rb*rL;
poseLp = [pL(1) pL(2) th + psiL];

% virtual reference scan of the prior map
[rr, aa] = rs(poseLp, mp);
rCur = double(ranges(:));  aCur = double(angles(:));
nCur = 0;  nRef = 0;
for k = 1:numel(rCur)
    if ~isnan(rCur(k)) && rCur(k) >= 1/smRes && rCur(k) <= maxR, nCur = nCur + 1; end
end
for k = 1:numel(rr)
    if ~isnan(rr(k)) && rr(k) >= 1/smRes && rr(k) <= maxR, nRef = nRef + 1; end
end
sm.nPts = uint16(nCur);
if nCur < c(10) || nRef < c(10)
    return
end
cur = lidarScan(rCur, aCur);
ref = lidarScan(rr(:), aa(:));
[relM, st] = matchScansGrid(cur, ref, 'InitialPose', [0 0 0], 'Resolution', smRes, 'MaxRange', maxR, ...
    'TranslationSearchRange', smTrans, 'RotationSearchRange', smRot);
dTh = 2*pi/(ceil(2*pi/acos(1 - (1/smRes)^2/(2*maxR^2))) - 1);
rel = [relM(1) relM(2) relM(3) - dTh];
winEff = ceil(smTrans*smRes/32)*32/smRes;
inside = abs(rel(1)) < 0.9*winEff(1) && abs(rel(2)) < 0.9*winEff(2) && abs(rel(3)) < 0.9*smRot;
sm.score = st.Score;
sm.valid = inside && st.Score/nCur >= c(9);

% corrected LiDAR pose -> base_link pose
thL = poseLp(3) + rel(3);
RL = [cos(poseLp(3)) -sin(poseLp(3)); sin(poseLp(3)) cos(poseLp(3))];
pLc = [poseLp(1); poseLp(2)] + RL*[rel(1); rel(2)];
thB = thL - psiL;
RB = [cos(thB) -sin(thB); sin(thB) cos(thB)];
pB = pLc - RB*rL;
sm.pose = [pB(1); pB(2); atan2(sin(thB), cos(thB))];

% covariance: weak translation axis from the scan geometry, predicted LiDAR frame -> W, lever arm of the mount
[Cl, geo] = local_weak_axis(st.Covariance, rCur, aCur, rr(:), aa(:), rel, 1/smRes, maxR, c(19), c(20), smRes, ...
    max(winEff), c(10));
J = [RL zeros(2, 1); 0 0 1];
Cw = J*Cl*J';
dRB = [-sin(thB) -cos(thB); cos(thB) -sin(thB)];
G = [eye(2) -dRB*rL; 0 0 1];
C = G*Cw*G';
C = 0.5*(C + C');
fl = [c(14) c(15) c(16)];
for i = 1:3
    if C(i, i) < fl(i), C(i, i) = fl(i); end
end
sm.cov = C;
end

% --------------------------------------------------------------------------------------------
function [C, geo] = local_weak_axis(C0, rC, aC, rR, aR, rel, rMin, rMax, gap, dMax, smRes, win, nMin)
% weak translation axis of the matcher's covariance from the point information of the scan (header)
C = 0.5*(C0 + C0.');
geo = zeros(1, 5);
a = C(1, 1);  b = C(1, 2);  d = C(2, 2);
if ~(isfinite(a) && isfinite(b) && isfinite(d))
    return
end
df = sqrt(((a - d)/2)^2 + b^2);
lam1 = (a + d)/2 - df;  lam2 = (a + d)/2 + df;              % lam1 <= lam2
if abs(b) > 1e-15
    v2 = [lam2 - d; b];
elseif a >= d
    v2 = [1; 0];
else
    v2 = [0; 1];
end
v2 = v2/sqrt(v2(1)^2 + v2(2)^2);  v1 = [-v2(2); v2(1)];
geo(2) = sqrt(max(lam2, 0));  geo(3) = geo(2);
lamWin = ((21^2 - 1)/12)/smRes^2;                           % +-10 cells sampled by estimateCovariance
if lam2 <= 0.25*lamWin || lam1 <= 0
    return
end
[F, nCorr] = local_point_info(rC, aC, rR, aR, rel, rMin, rMax, gap, dMax);
geo(5) = nCorr;
if nCorr < nMin
    return
end
Fi = (F + 1e-9*eye(3))\eye(3);
T2 = Fi(1:2, 1:2);
g1 = v1.'*T2*v1;  g2 = v2.'*T2*v2;
if ~(g1 > 0 && isfinite(g2))
    return
end
r = max(g2/g1, 1);
lam2n = min(lam1*r, win^2);
s = sqrt(lam2n/lam2);
T = eye(3);
T(1:2, 1:2) = v1*v1.' + s*(v2*v2.');
C = T*C*T.';
C = 0.5*(C + C.');
geo = [r sqrt(lam2) sqrt(lam2n) 1 nCorr];
end

% --------------------------------------------------------------------------------------------
function [F, nCorr] = local_point_info(rC, aC, rR, aR, rel, rMin, rMax, gap, dMax)
% unit-weight information of [x y psi] (predicted LiDAR frame) of the current scan moved by rel against the
% reference scan: one correspondence per current point, the reference point on the same beam within dMax
n = numel(rR);
qx = zeros(n, 1);  qy = zeros(n, 1);  okR = false(n, 1);
for k = 1:n
    okR(k) = ~isnan(rR(k)) && rR(k) >= rMin && rR(k) <= rMax;
    if okR(k)
        qx(k) = rR(k)*cos(aR(k));  qy(k) = rR(k)*sin(aR(k));
    end
end
c3 = cos(rel(3));  s3 = sin(rel(3));
res = 2*pi/n;
F = zeros(3);  nCorr = 0;
for i = 1:numel(rC)
    if isnan(rC(i)) || rC(i) < rMin || rC(i) > rMax, continue, end
    px0 = rC(i)*cos(aC(i));  py0 = rC(i)*sin(aC(i));
    px = c3*px0 - s3*py0 + rel(1);  py = s3*px0 + c3*py0 + rel(2);
    k = mod(round((atan2(py, px) - aR(1))/res), n) + 1;
    if ~okR(k), continue, end
    dx = px - qx(k);  dy = py - qy(k);
    if dx*dx + dy*dy > dMax*dMax, continue, end
    km = mod(k - 2, n) + 1;  kp = mod(k, n) + 1;
    isLine = false;
    if okR(km) && okR(kp)
        isLine = hypot(qx(kp) - qx(k), qy(kp) - qy(k)) < gap && hypot(qx(k) - qx(km), qy(k) - qy(km)) < gap;
    end
    if isLine
        tx = qx(kp) - qx(km);  ty = qy(kp) - qy(km);  Lt = hypot(tx, ty);
        nx = -ty/Lt;  ny = tx/Lt;
        Jl = [nx ny (ny*px - nx*py)];
        F = F + Jl.'*Jl;
    else
        J1 = [1 0 -py];  J2 = [0 1 px];
        F = F + J1.'*J1 + J2.'*J2;
    end
    nCorr = nCorr + 1;
end
end
