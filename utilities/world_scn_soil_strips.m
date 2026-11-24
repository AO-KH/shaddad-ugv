function G = world_scn_soil_strips(W, Dsv, rs)
%WORLD_SCN_SOIL_STRIPS  Scenario geometry of world 6 'soil_strips' (build plan T02.5). Called by world_build.
%
%   G = world_scn_soil_strips(W, Dsv, rs)     (Dsv, rs unused: no zones, no random draws)
%
%   60 x 60 m flat grid (x -10..50, y -10..50, 0.25 m). Five 10 m wide strips running along +x:
%   soil k = 1..5 (W.soilTable ids) for y in [10(k-1), 10k) (the last strip includes y = 50); the
%   approach area y < 0 is W.soilDefault. One lane per strip centreline (y = 10k - 5, x 0..40) for
%   straight-line slip and turning tests. No obstacles, veg 1.0, shade 1.0. Start [-5 5 0] (strip 1
%   centreline), dock [-6 -5 0].
%#ok<*INUSD>

res = 0.25;
xE = -10:res:50;  yN = -10:res:50;
[X, Y] = ndgrid(xE, yN);
ids = W.soilTable(:, 1)';
assert(numel(ids) >= 5, 'soil_strips needs 5 soil classes');

soilId = W.soilDefault*ones(size(X));
for k = 1:5
    m = Y >= 10*(k-1) & Y < 10*k;
    if k == 5, m = Y >= 40; end
    soilId(m) = ids(k);
end
yc = 10*(1:5)' - 5;
rows = [zeros(5,1) yc 40*ones(5,1) yc (1:5)'];

G = struct('xE', xE, 'yN', yN, 'H', zeros(size(X)), 'soilId', soilId, 'veg', ones(size(X)), ...
    'shade', ones(size(X)), 'cyl', zeros(0, 6), 'box', zeros(0, 9), 'film', zeros(0, 4), ...
    'canopy', zeros(0, 3), 'zones', struct('X', {}, 'Y', {}, 'svf', {}, 'type', {}), 'rows', rows, ...
    'boundary', [-8 -8; 48 -8; 48 48; -8 48], 'start', [-5 5 0], 'dock', [-6 -5 0], ...
    'info', struct('stripEdgesY', 10*(0:5)));
end
