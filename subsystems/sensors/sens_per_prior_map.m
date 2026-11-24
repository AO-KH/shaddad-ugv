function [grid, occ] = sens_per_prior_map(W, per)
%SENS_PER_PRIOR_MAP  Prior occupancy map of the scan-to-map localisation (sensors spec 3.7 / F11, T13).
%
%   [grid, occ] = sens_per_prior_map(P.world, P.sensors.per)
%     grid   logical occupancy grid, same resolution / extent / orientation as P.world.occ (row 1 = max y,
%            lower-left corner P.world.occ.origin)
%     occ    struct {res, origin, grid}
%
%   The prior map is P.world.occ with the as-built deviation per.priorMapErr (sensors spec 7.8, 0.03 m): every
%   static obstacle of P.world.obstacles (cylinders, boxes, film segments; rows 1..n*) is shifted by an
%   independent N(0, priorMapErr^2) offset per axis (RandStream mt19937ar, seed per.priorMapSeed; the global
%   random stream is not touched) and the grid is rebuilt with world_occupancy (T02) at P.world.occ.res.
%   priorMapErr = 0 returns P.world.occ itself. Evaluated once per compile by the PriorMap Constant block of
%   Shaddad_Perception/ScanMatch ("built at initialisation").

if per.priorMapErr <= 0
    occ = W.occ;
    grid = logical(occ.grid);
    return
end
rs = RandStream('mt19937ar', 'Seed', per.priorMapSeed);
O = W.obstacles;
e = per.priorMapErr;
if O.nCyl > 0
    O.cyl(1:O.nCyl, 1:2) = O.cyl(1:O.nCyl, 1:2) + e*randn(rs, O.nCyl, 2);
end
if O.nBox > 0
    O.box(1:O.nBox, 1:2) = O.box(1:O.nBox, 1:2) + e*randn(rs, O.nBox, 2);
end
if O.nFilm > 0
    d = e*randn(rs, O.nFilm, 2);
    O.film(1:O.nFilm, 1:2) = O.film(1:O.nFilm, 1:2) + d;
    O.film(1:O.nFilm, 3:4) = O.film(1:O.nFilm, 3:4) + d;
end
W.obstacles = O;
occ = world_occupancy(W);
assert(isequal(occ.origin, W.occ.origin) && occ.res == W.occ.res && isequal(size(occ.grid), size(W.occ.grid)), ...
    'sens_per_prior_map:grid', 'prior map grid differs from P.world.occ');
grid = logical(occ.grid);
end
