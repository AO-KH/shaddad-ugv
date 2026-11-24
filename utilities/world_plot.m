function fig = world_plot(W, varargin)
%WORLD_PLOT  Plot a P.world: terrain height, soil map, GNSS sky-view grid and occupancy, with
%   obstacles, canopy circles, film segments, GNSS zones, rows, boundary, start and dock overlaid.
%
%   fig = world_plot(W)                          new figure, 2 x 2 tiles
%   fig = world_plot(W, 'Visible', 'off')        invisible figure (tests / batch); caller closes it
%   fig = world_plot(W, 'File', 'work/world.png')   also export the figure (exportgraphics)
%   The function only creates its own figure; it never closes or reuses other figures.

ip = inputParser;
ip.addParameter('Visible', 'on');
ip.addParameter('File', '');
ip.parse(varargin{:});
o = ip.Results;

names = {'flat_test', 'slope_test', 'palm_farm', 'greenhouse', 'solar_farm', 'soil_strips'};
nm = sprintf('world %d', W.id);
if W.id >= 1 && W.id <= numel(names), nm = sprintf('%s (id %d)', names{W.id}, W.id); end

fig = figure('Name', ['P.world: ' nm], 'Visible', o.Visible, 'Color', 'w', 'Position', [100 100 1200 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, ['P.world  ' strrep(nm, '_', '\_')]);
xE = W.terrain.xE;  yN = W.terrain.yN;

ax = nexttile(tl);
imagesc(ax, xE, yN, W.terrain.H');  set(ax, 'YDir', 'normal');  axis(ax, 'equal', 'tight');
cb = colorbar(ax);  cb.Label.String = 'H [m]';
title(ax, 'terrain height');  local_overlay(ax, W, true);

ax = nexttile(tl);
imagesc(ax, xE, yN, W.soilId');  set(ax, 'YDir', 'normal');  axis(ax, 'equal', 'tight');
colormap(ax, lines(max(W.soilTable(:,1))));  clim(ax, [0.5 max(W.soilTable(:,1)) + 0.5]);
cb = colorbar(ax);  cb.Ticks = W.soilTable(:,1);  cb.Label.String = 'soilId';
title(ax, 'soil (1 sand, 2 sandy loam, 3 wet loam, 4 greenhouse, 5 asphalt)');  local_overlay(ax, W, false);

ax = nexttile(tl);
imagesc(ax, xE, yN, W.gnssSvf');  set(ax, 'YDir', 'normal');  axis(ax, 'equal', 'tight');
clim(ax, [0 1]);  cb = colorbar(ax);  cb.Label.String = 'sky-view factor';
title(ax, 'GNSS sky view (gnssSvf) and zones');  local_overlay(ax, W, false);

ax = nexttile(tl);
oc = W.occ;
xo = oc.origin(1) + ((1:size(oc.grid,2)) - 0.5)/oc.res;
yo = oc.origin(2) + ((size(oc.grid,1):-1:1) - 0.5)/oc.res;
imagesc(ax, xo, yo, oc.grid);  set(ax, 'YDir', 'normal');  axis(ax, 'equal', 'tight');
colormap(ax, [1 1 1; 0 0 0]);
title(ax, sprintf('occupancy (%d cells/m, %d occupied)', oc.res, nnz(oc.grid)));
local_overlay(ax, W, true);

if ~isempty(o.File)
    exportgraphics(fig, o.File);
end
end

% ---------------------------------------------------------------------------------------------
function local_overlay(ax, W, full)
hold(ax, 'on');
b = W.boundary([1:end 1], :);
plot(ax, b(:,1), b(:,2), 'm-', 'LineWidth', 1.2);
Z = W.gnssZones;
for k = 1:Z.n
    nv = Z.nv(k);
    plot(ax, Z.X(k, [1:nv 1]), Z.Y(k, [1:nv 1]), 'c--', 'LineWidth', 1);
end
if full
    O = W.obstacles;
    th = linspace(0, 2*pi, 25);
    for k = 1:O.nCyl
        c = O.cyl(k, :);
        plot(ax, c(1) + c(3)*cos(th), c(2) + c(3)*sin(th), 'r-');
    end
    for k = 1:W.nCanopy
        c = W.canopy(k, :);
        plot(ax, c(1) + c(3)*cos(th), c(2) + c(3)*sin(th), 'g:');
    end
    for k = 1:O.nBox
        bx = O.box(k, :);
        u = [-1 1 1 -1 -1]*bx(4)/2;  v = [-1 -1 1 1 -1]*bx(5)/2;
        plot(ax, bx(1) + cos(bx(7))*u - sin(bx(7))*v, bx(2) + sin(bx(7))*u + cos(bx(7))*v, 'b-');
    end
    for k = 1:O.nFilm
        f = O.film(k, :);
        plot(ax, f([1 3]), f([2 4]), 'y-', 'LineWidth', 2);
    end
end
for k = 1:W.nRows
    r = W.rows(k, :);
    plot(ax, r([1 3]), r([2 4]), 'k--');
end
local_pose(ax, W.start, 'k');
local_pose(ax, W.dock.pose, 'r');
hold(ax, 'off');
end

function local_pose(ax, p, c)
L = 2;
plot(ax, p(1), p(2), [c 'o'], 'MarkerFaceColor', c);
quiver(ax, p(1), p(2), L*cos(p(3)), L*sin(p(3)), 0, 'Color', c, 'LineWidth', 1.5, 'MaxHeadSize', 2);
end
