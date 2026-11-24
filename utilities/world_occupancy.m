function occ = world_occupancy(W, zBand, buf)
%WORLD_OCCUPANCY  A-priori occupancy grid of a world (binaryOccupancyMap semantics).
%
%   occ = world_occupancy(W)                 occ.res, occ.origin, occ.grid (struct of P.world.occ)
%   occ = world_occupancy(W, zBand, buf)     height band [zLow zHigh] above ground (default
%                                            world_occupancy('defaults').zBand = LiDAR band
%                                            P.sensors.lidar.zBand) and film buffer buf [m]
%   D   = world_occupancy('defaults')        struct('zBand',[0.15 1.20],'res',10,'filmBuffer',0.05)
%
%   Grid: W.occ.res cells/m (10 if W has no occ yet) over W.xlim x W.ylim, origin = lower-left
%   corner [xlim(1) ylim(1)], row 1 = max y, column 1 = min x (binaryOccupancyMap). A cell is
%   occupied when its centre lies inside a cylinder, inside a (rotated) box, or within buf of a
%   film segment, and the obstacle's vertical extent [zBase, zBase + h] (heights above the local
%   ground) overlaps zBand. Film segments span the full band (greenhouse walls).
%   Only list rows 1..n* are used (the dummy rows of empty lists are ignored).
%   Used by world_build and by scenario scripts that append obstacles (D42: scn_S5 rebuilds
%   P.world.occ = world_occupancy(P.world)).

D = struct('zBand', [0.15 1.20], 'res', 10, 'filmBuffer', 0.05);
if ischar(W) || isstring(W)
    if strcmpi(W, 'defaults'), occ = D; return, end
    error('world_occupancy:arg', 'world_occupancy(W, zBand, buf) | world_occupancy(''defaults'')');
end
if nargin < 2 || isempty(zBand), zBand = D.zBand; end
if nargin < 3 || isempty(buf),   buf = D.filmBuffer; end
res = D.res;
if isfield(W, 'occ') && isfield(W.occ, 'res') && ~isempty(W.occ.res), res = W.occ.res; end

origin = [W.xlim(1) W.ylim(1)];
nx = round((W.xlim(2) - W.xlim(1)) * res);
ny = round((W.ylim(2) - W.ylim(1)) * res);
xc = origin(1) + ((1:nx) - 0.5) / res;             % column centres (x ascending)
yc = origin(2) + ((ny:-1:1)' - 0.5) / res;         % row centres (row 1 = max y)
grid = false(ny, nx);
O = W.obstacles;

%% ---- cylinders [xE yN r h zBase class] ----
for k = 1:O.nCyl
    c = O.cyl(k, :);
    if ~local_band(c(5), c(5) + c(4), zBand), continue, end
    [ir, ic] = local_window(xc, yc, c(1), c(2), c(3));
    if isempty(ir) || isempty(ic), continue, end
    m = (xc(ic) - c(1)).^2 + (yc(ir) - c(2)).^2 <= c(3)^2;
    grid(ir, ic) = grid(ir, ic) | m;
end

%% ---- boxes [xE yN zBase lx ly lz yaw class id] ----
for k = 1:O.nBox
    b = O.box(k, :);
    if ~local_band(b(3), b(3) + b(6), zBand), continue, end
    [ir, ic] = local_window(xc, yc, b(1), b(2), hypot(b(4), b(5)) / 2);
    if isempty(ir) || isempty(ic), continue, end
    dx = xc(ic) - b(1);  dy = yc(ir) - b(2);       % row vector / column vector -> matrix
    cy = cos(b(7));  sy = sin(b(7));
    u =  cy * dx + sy * dy;
    v = -sy * dx + cy * dy;
    m = abs(u) <= b(4)/2 + 1e-12 & abs(v) <= b(5)/2 + 1e-12;
    grid(ir, ic) = grid(ir, ic) | m;
end

%% ---- film segments [x1 y1 x2 y2], buffered ----
for k = 1:O.nFilm
    f = O.film(k, :);
    xm = (f(1) + f(3))/2;  ym = (f(2) + f(4))/2;
    L = hypot(f(3) - f(1), f(4) - f(2));
    [ir, ic] = local_window(xc, yc, xm, ym, L/2 + buf);
    if isempty(ir) || isempty(ic), continue, end
    px = repmat(xc(ic), numel(ir), 1);  py = repmat(yc(ir), 1, numel(ic));
    if L < eps
        d = hypot(px - f(1), py - f(2));
    else
        ux = (f(3) - f(1))/L;  uy = (f(4) - f(2))/L;
        t = min(max((px - f(1))*ux + (py - f(2))*uy, 0), L);
        d = hypot(px - (f(1) + t*ux), py - (f(2) + t*uy));
    end
    grid(ir, ic) = grid(ir, ic) | d <= buf + 1e-12;
end

occ = struct('res', res, 'origin', origin, 'grid', grid);
end

% ---------------------------------------------------------------------------------------------
function tf = local_band(z0, z1, zBand)
tf = z0 < zBand(2) && z1 > zBand(1);
end

function [ir, ic] = local_window(xc, yc, x0, y0, r)
ic = find(xc >= x0 - r & xc <= x0 + r);
ir = find(yc >= y0 - r & yc <= y0 + r);
end
