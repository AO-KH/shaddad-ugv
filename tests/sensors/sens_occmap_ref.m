function [pRef, glRef, occRef] = sens_occmap_ref(S, P)
%SENS_OCCMAP_REF  Reference rolling occupancy grid with occupancyMap / insertRay in MATLAB execution (T13 test).
%
%   [pRef, glRef, occRef] = sens_occmap_ref(S, P)
%     S        scans (per_scan_gen: ranges, angles, valid, poseL); the filtered ranges come from
%              sens_scan_filter_ref
%     pRef     N x N x K occupancy probabilities after every scan, glRef K x 2 GridLocationInWorld, occRef
%              logical N x N x K (p > occThr)
%   occupancyMap(mapSize, mapSize, mapRes), GridOriginInLocal = -mapSize/2, ProbabilitySaturation, move(...,
%   'MoveType', 'Absolute', 'FillValue', 0.5), insertRay(map, poseL, r, a, maxRange, [miss hit]) with r = filtered
%   range for kept beams and maxRange + 1 (free ray) for raw no-return beams - the specification of
%   subsystems/sensors/sens_occmap_update.m (sensors spec 3.7 / F8).

per = P.sensors.per;
om = occupancyMap(per.mapSize, per.mapSize, per.mapRes);
om.GridOriginInLocal = [-per.mapSize/2 -per.mapSize/2];
om.ProbabilitySaturation = per.sat;
K = size(S.ranges, 1);  N = per.mapSize*per.mapRes;
pRef = zeros(N, N, K, 'single');  glRef = zeros(K, 2);  occRef = false(N, N, K);
for k = 1:K
    pL = S.poseL(k, :);
    move(om, pL(1:2), 'MoveType', 'Absolute', 'FillValue', 0.5);
    if S.valid(k)
        r = S.ranges(k, :)';  a = double(S.angles(:));
        rF = sens_scan_filter_ref(r, S.angles(:), P);
        rr = NaN(numel(r), 1);
        rr(~isnan(rF)) = double(rF(~isnan(rF)));
        rr(isnan(r)) = per.maxRange + 1;
        m = ~isnan(rr);
        insertRay(om, pL, rr(m), a(m), per.maxRange, [per.miss per.hit]);
    end
    p = occupancyMatrix(om);
    pRef(:, :, k) = single(p);
    occRef(:, :, k) = p > per.occThr;
    glRef(k, :) = om.GridLocationInWorld;
end
end
