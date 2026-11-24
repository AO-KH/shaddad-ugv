function G = world_scn_solar_farm(W, Dsv, rs)
%WORLD_SCN_SOLAR_FARM  Scenario geometry of world 5 'solar_farm' (build plan T02.4). Called by world_build.
%
%   G = world_scn_solar_farm(W, Dsv, rs)      (rs unused: no random draws)
%
%   Grid x -10..50, y -10..50 (0.25 m), flat, soil 2, veg 2.0. Five E-W table rows (x 0..40, 4.0 m
%   deep, centres y = 7k, k = 1..5) as boxes class 6 (zBase = lower edge 0.8 m, lz 1.5 m). Piers
%   cyl class 3 r 0.08 every 6 m (x = 2 + 6m, m = 0..6) on y = yc +/- 1.4, height zBase + lz/2.
%   Shade 0.2 under the tables, 1.0 elsewhere. GNSS zones: under each table (svf Dsv.underPanel 0.45,
%   type 2) and between adjacent rows (Dsv.betweenPanels 0.75, type 3). Aisles at y = 7k + 3.5
%   (k = 0..5), x 0..40. Start [-5 3.5 0] (west end of aisle 1), dock [-6 -1.5 0].
%#ok<*INUSL,*INUSD>

res = 0.25;
xE = -10:res:50;  yN = -10:res:50;
[X, Y] = ndgrid(xE, yN);
x0 = 0;  x1 = 40;  depth = 4.0;  zB = 0.8;  lz = 1.5;
yc = 7*(1:5)';  nT = numel(yc);

box = [((x0 + x1)/2)*ones(nT,1) yc zB*ones(nT,1) (x1 - x0)*ones(nT,1) depth*ones(nT,1) ...
       lz*ones(nT,1) zeros(nT,1) 6*ones(nT,1) (1:nT)'];

xp = 2 + 6*(0:6)';  nX = numel(xp);
cyl = zeros(0, 6);
for k = 1:nT
    for s = [-1 1]
        cyl = [cyl; xp, (yc(k) + s*1.4)*ones(nX,1), 0.08*ones(nX,1), (zB + lz/2)*ones(nX,1), ...
               zeros(nX,1), 3*ones(nX,1)]; %#ok<AGROW>
    end
end

H = zeros(size(X));
soilId = 2*ones(size(X));
veg = 2.0*ones(size(X));
shade = ones(size(X));
for k = 1:nT
    shade(X >= x0 & X <= x1 & abs(Y - yc(k)) <= depth/2) = 0.2;
end

zones = struct('X', {}, 'Y', {}, 'svf', {}, 'type', {});
for k = 1:nT                                 % under the tables
    zones(end+1) = struct('X', [x0 x1 x1 x0], 'Y', yc(k) + depth/2*[-1 -1 1 1], ...
                          'svf', Dsv.underPanel, 'type', 2); %#ok<AGROW>
end
for k = 1:nT-1                               % between adjacent rows
    zones(end+1) = struct('X', [x0 x1 x1 x0], 'Y', [yc(k) + depth/2, yc(k) + depth/2, ...
                          yc(k+1) - depth/2, yc(k+1) - depth/2], ...
                          'svf', Dsv.betweenPanels, 'type', 3); %#ok<AGROW>
end

ya = 7*(0:5)' + 3.5;
rows = [x0*ones(6,1) ya x1*ones(6,1) ya (1:6)'];

G = struct('xE', xE, 'yN', yN, 'H', H, 'soilId', soilId, 'veg', veg, 'shade', shade, ...
    'cyl', cyl, 'box', box, 'film', zeros(0, 4), 'canopy', zeros(0, 3), 'zones', zones, 'rows', rows, ...
    'boundary', [-8 -8; 48 -8; 48 48; -8 48], 'start', [-5 3.5 0], 'dock', [-6 -1.5 0], ...
    'info', struct('tableCentresY', yc, 'pierX', xp));
end
