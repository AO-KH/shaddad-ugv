function [map, pOcc] = sens_occmap_update(poseL, ranges, rangesF, angles, valid, c)
%SENS_OCCMAP_UPDATE  Rolling local occupancy grid in the TEB MapInfo format (sensors spec 3.7 / F8).
%#codegen
%   [map, pOcc] = sens_occmap_update(poseL, ranges, rangesF, angles, valid, c)
%     poseL    [x_E y_N psi] of the LiDAR in W (level mount, psi CCW from East)
%     ranges   single[360] raw scan (NaN = no return within the sensor range)
%     rangesF  single[360] filtered scan (sens_scan_filter: NaN = no return or removed beam)
%     angles   single[360] beam angles in the LiDAR frame
%     valid    scan valid
%     c        sens_per_prm(P, 'occ') = [mapSize mapRes hit miss satLo satHi maxRange occThr]
%     map      Bus_TebMapInfo: Resolution, GridLocationInWorld, GridSize, OccupancyMatrix (p > occThr,
%              row 1 = max y), GridOriginInLocal (field order of the TEB default MapInfo struct)
%     pOcc     single[N x N] occupancy probabilities (tests; the block does not use it)
%
%   Semantics of occupancyMap(mapSize, mapSize, mapRes) with GridOriginInLocal = -mapSize/2 (robot-centred),
%   move(map, poseL(1:2), 'MoveType', 'Absolute', 'FillValue', 0.5) and insertRay(map, poseL, r, a, maxRange,
%   [miss hit]) with ProbabilitySaturation [satLo satHi], implemented in the function on a persistent log-odds
%   grid (N = 300 cells per side):
%     rolling  the lower-left corner is snapped to the world cell grid (floor(x*res) - N/2 cells); cells that
%              enter the window are unknown (log-odds 0, p = 0.5); the stored cells keep their world position
%     rays     Amanatides-Woo traversal from the LiDAR cell; every cell crossed by a ray gets one miss update
%              (log-odds of miss), the endpoint cell of a return gets one hit update (log-odds of hit); log-odds
%              saturated at [satLo satHi]; a ray stops at the window edge or at maxRange
%     beams    kept return (rangesF) -> hit ray; raw no-return beam (NaN in ranges) -> free ray to maxRange;
%              beam removed by the scan filter (self box, isolated dust return) -> not inserted
%   A scan with valid = false is not inserted (the window still moves).
%   Why not the occupancyMap object itself: in R2026b the generated code of insertRay (nav.algs.internal.impl.
%   raycastInternal) marks cells next to the ray origin as hits for rays that leave the map (measured 10-17
%   spurious occupied cells around the LiDAR in the compiled block, none in MATLAB execution) and costs
%   about 0.3 us per traversed cell (19 ms for the 360 free rays of an open-field scan, budget 20 ms per
%   perception step). tests/sensors/t_perception compares this function with occupancyMap / insertRay
%   executed in MATLAB.

persistent Lg ix0 iy0
N = 300;                                    % Bus_TebMapInfo.OccupancyMatrix is [300 300] (ICD 10, fixed size)
res = c(2);
if isempty(Lg)
    assert(round(c(1)*c(2)) == N, 'sens_occmap_update: mapSize*mapRes must equal the bus grid size 300');
    Lg = zeros(N, N);
    ix0 = floor(poseL(1)*res) - N/2;
    iy0 = floor(poseL(2)*res) - N/2;
end
lHit = log(c(3)/(1 - c(3)));  lMiss = log(c(4)/(1 - c(4)));
lLo = log(c(5)/(1 - c(5)));   lHi = log(c(6)/(1 - c(6)));
lThr = log(c(8)/(1 - c(8)));
maxR = c(7);

%% ---- rolling window ----
nx0 = floor(poseL(1)*res) - N/2;
ny0 = floor(poseL(2)*res) - N/2;
dx = nx0 - ix0;  dy = ny0 - iy0;
if dx ~= 0 || dy ~= 0
    Ln = zeros(N, N);
    if abs(dx) < N && abs(dy) < N
        % new(r, col) = old(r - dy, col + dx)
        r1 = max(1, 1 + dy);  r2 = min(N, N + dy);
        c1 = max(1, 1 - dx);  c2 = min(N, N - dx);
        for col = c1:c2
            for r = r1:r2
                Ln(r, col) = Lg(r - dy, col + dx);
            end
        end
    end
    Lg = Ln;
    ix0 = nx0;  iy0 = ny0;
end

%% ---- insert the scan ----
if valid
    % LiDAR position in grid units (column coordinate u = x*res - ix0, row-from-bottom coordinate v)
    u0 = poseL(1)*res - ix0;  v0 = poseL(2)*res - iy0;
    for k = 1:numel(ranges)
        if ~isnan(rangesF(k))
            rr = double(rangesF(k));  isHit = rr <= maxR;  rr = min(rr, maxR);
        elseif isnan(ranges(k))
            rr = maxR;  isHit = false;
        else
            continue
        end
        th = poseL(3) + double(angles(k));
        u1 = u0 + rr*res*cos(th);  v1 = v0 + rr*res*sin(th);
        % Amanatides-Woo traversal in grid units (inline: the persistent grid is updated in place); cell (i, j)
        % with i = column 1..N (x), j = row-from-bottom 1..N (y), matrix index (N + 1 - j, i)
        i = floor(u0) + 1;  j = floor(v0) + 1;
        iE = floor(u1) + 1;  jE = floor(v1) + 1;
        du = u1 - u0;  dv = v1 - v0;
        if du > 0, si = 1; tMx = (floor(u0) + 1 - u0)/du; tDx = 1/du;
        elseif du < 0, si = -1; tMx = (u0 - floor(u0))/(-du); tDx = -1/du;
        else, si = 0; tMx = Inf; tDx = Inf;
        end
        if dv > 0, sj = 1; tMy = (floor(v0) + 1 - v0)/dv; tDy = 1/dv;
        elseif dv < 0, sj = -1; tMy = (v0 - floor(v0))/(-dv); tDy = -1/dv;
        else, sj = 0; tMy = Inf; tDy = Inf;
        end
        nSteps = abs(iE - i) + abs(jE - j);
        for s = 0:nSteps
            if i < 1 || i > N || j < 1 || j > N
                break                           % left the window: the rest of the ray is not stored
            end
            rw = N + 1 - j;
            if s == nSteps || (i == iE && j == jE)
                if isHit
                    Lg(rw, i) = min(Lg(rw, i) + lHit, lHi);      % endpoint of a return: hit
                else
                    Lg(rw, i) = max(Lg(rw, i) + lMiss, lLo);
                end
                break
            end
            Lg(rw, i) = max(Lg(rw, i) + lMiss, lLo);            % crossed cell: miss
            if tMx < tMy
                i = i + si;  tMx = tMx + tDx;
            else
                j = j + sj;  tMy = tMy + tDy;
            end
        end
    end
end

%% ---- outputs ----
map.Resolution = res;
map.GridLocationInWorld = [ix0 iy0]/res;
map.GridSize = [N N];
map.OccupancyMatrix = Lg > lThr;
map.GridOriginInLocal = [-c(1)/2 -c(1)/2];
if nargout > 1
    pOcc = single(1 - 1./(1 + exp(Lg)));
else
    pOcc = zeros(N, N, 'single');
end
end
