function S = per_scan_gen(P, t, poseB, varargin)
%PER_SCAN_GEN  FAST2D LiDAR scans along a base_link trajectory (T13 perception test input).
%
%   S = per_scan_gen(P, t, poseB)
%   S = per_scan_gen(P, t, poseB, 'dust', d, 'obs', @(t) [x y r; ...], 'seed', s)
%     t       N x 1 scan times [s]
%     poseB   N x 3 base_link poses [x_E y_N psi] (level)
%     dust    airborne dust level 0..1 (scalar, default 0)
%     obs     @(t) -> M x 3 dynamic obstacles [x y r] (M <= 4, default none)
%     seed    random seed of the range noise / dropout / dust / film draws (default 1)
%     S       struct: t (N x 1), ranges (N x 360 single, NaN = no return), angles (1 x 360 single), valid (N x 1),
%             poseL (N x 3 LiDAR pose)
%
%   Calls the plant FAST2D model sens_lidar2d_fast (T11) with the static obstacles of P.world (rasterised at the
%   first call: the function is cleared first so the raster belongs to this P.world) and the LiDAR mount
%   P.sensors.lidar.r / eul_deg. The draws use a private RandStream (the global stream is not touched).

ip = inputParser;
ip.addParameter('dust', 0);
ip.addParameter('obs', []);
ip.addParameter('seed', 1);
ip.parse(varargin{:});
o = ip.Results;
L = P.sensors.lidar;  O = P.world.obstacles;
c = sens_lidar2d_fast_prm(L, P.world);
clear sens_lidar2d_fast
rs = RandStream('mt19937ar', 'Seed', o.seed);
N = numel(t);
S.t = t(:);
S.ranges = NaN(N, L.nBeams, 'single');
S.angles = single(L.angles);
S.valid = false(N, 1);
S.poseL = zeros(N, 3);
psiM = L.eul_deg(1)*pi/180;
for k = 1:N
    p = poseB(k, :);
    pL = [p(1) + L.r(1)*cos(p(3)) - L.r(2)*sin(p(3)), p(2) + L.r(1)*sin(p(3)) + L.r(2)*cos(p(3)), p(3) + psiM];
    obsN = 0;  obsXY = zeros(4, 2);  obsR = zeros(4, 1);
    if ~isempty(o.obs)
        M = o.obs(t(k));
        obsN = size(M, 1);
        obsXY(1:obsN, :) = M(:, 1:2);  obsR(1:obsN) = M(:, 3);
    end
    nR = randn(rs, 1, L.nBeams);  uD = rand(rs, 1, L.nBeams);  uU = rand(rs, 1, L.nBeams);  uF = rand(rs, 1, L.nBeams);
    [r, v] = sens_lidar2d_fast(pL, nR, uD, uU, uF, o.dust, false, obsN, obsXY, obsR, O.cyl, O.box, O.film, L.angles, c);
    S.ranges(k, :) = r(:)';
    S.valid(k) = v;
    S.poseL(k, :) = pL;
end
clear sens_lidar2d_fast
end
