function [ranges, valid] = sens_lidar2d_fast(poseL, nR, uDrop, uDust, uFilm, dust, blackout, obsN, obsXY, obsR, cyl, box, film, angles, c)
%SENS_LIDAR2D_FAST  FAST2D LiDAR model: 360-beam height-band scan by ray casting (sensors spec 3.4 / F3).
%#codegen
%   [ranges, valid] = sens_lidar2d_fast(poseL, nR, uDrop, uDust, uFilm, dust, blackout, obsN, obsXY, obsR, ...
%                                       cyl, box, film, angles, c)
%     poseL    [x_E y_N psi] of the LiDAR (level mount, psi CCW from East)
%     nR       360 standard-normal draws (range noise)
%     uDrop, uDust, uFilm   360 uniform draws U(0,1) each (dropout, dust clutter, film return)
%     dust     airborne dust level 0..1 (Bus_FaultInject.dust_level)
%     blackout LiDAR blackout (Bus_FaultInject.lidar_blackout): all NaN, valid false
%     obsN, obsXY (4x2), obsR (4)   dynamic obstacles (Bus_ObstaclesDyn), cylinders, analytic ray-circle
%     cyl, box, film   P.world.obstacles.cyl / box / film (Constant block inputs; rows 1..n* are read)
%     angles   P.sensors.lidar.angles (beam angles in the LiDAR frame)
%     c        sens_lidar2d_fast_prm(P.sensors.lidar, P.world): raster extent [xlim ylim], rasterRes, rMin,
%              rMax, sigR, rangeNoiseK, zBand, pDrop, pDropDust, pDustMax, dustRange, dustRangeLoss, pRetFilm,
%              filmBoxClass, tiltedBoxClass, nCyl, nBox, nFilm (constants on an input, no parameter data)
%     ranges   single[360], beam k at angle -pi + (k-1) deg in the LiDAR frame, NaN = no return
%     valid    scan valid
%
%   At the first call the static obstacles whose vertical extent overlaps the height band zBand are
%   rasterised (rasterRes cells/m over the world extent) into a cell-class grid (bit 0 solid, bit 1 film):
%     cylinders [xE yN r h zBase class]   every cell that the circle touches (conservative), with the cylinder
%              index in an id grid: a ray entering such a cell is intersected EXACTLY with that circle
%              (return at the true surface; a ray that only grazes the cell continues)
%     boxes [xE yN zBase lx ly lz yaw class id]  cells whose centre lies inside the rotated rectangle plus the
%              sampled outline (return at the cell entry, +/- half a cell); class filmBoxClass (greenhouse
%              PE-film walls) is represented by the film layer only; class tiltedBoxClass (tilted solar
%              table, low edge south) contributes only the in-band strip
%              v in [-ly/2, -ly/2 + ly*(zBand(2) - zBase)/lz]
%     film [x1 y1 x2 y2]                  sampled segment -> film layer (return probability pRetFilm)
%   plus a Chebyshev distance grid (cells to the nearest non-empty cell, two-pass transform, capped at 255).
%   Each scan, per beam: grid traversal (Amanatides-Woo) with empty-space skipping (inside a cell at
%   Chebyshev distance D >= 3 the ray advances (D - 1) cells at once; every skipped cell is provably empty, so
%   the first hit is the same as a plain traversal) up to rMax; first film cell and first solid return;
%   dynamic obstacles by exact ray-circle intersection; then r = film if (film nearer and uFilm < pRetFilm),
%   range noise sigR (1 + k r) nR, no return outside [rMin, rMax (1 - 0.5 dust)] or when
%   uDrop < pDrop + 0.03 dust, dust clutter r = 0.3 + 2.7 uDust/(pDustMax dust) when uDust < pDustMax dust.
%   The grids are persistent variable-size arrays sized from the world extent at the first call (plant model,
%   simulation only: needs MATLABDynamicMemAlloc 'on', as in cs_SimFixed / cs_SimRef / cs_Shaddad).
%   rangeSensor / binaryOccupancyMap are not used.

persistent cg dg idg
coder.varsize('cg', 'dg', 'idg');
res = c(5);
x0 = c(1);  y0 = c(3);
nx = round((c(2) - c(1))*res);
ny = round((c(4) - c(3))*res);
if isempty(cg)
    cg = zeros(nx, ny, 'uint8');
    idg = zeros(nx, ny, 'uint16');
    [cg, idg] = local_raster(cg, idg, cyl, box, film, c, res, x0, y0, nx, ny);
    dg = local_chebyshev(cg, nx, ny);
end
rMin = c(6);  rMax = c(7);  sigR = c(8);  kR = c(9);
pDrop = c(12);  pDropDust = c(13);  pDustMax = c(14);  dR1 = c(15);  dR2 = c(16);  dLoss = c(17);
pRetFilm = c(18);

nB = numel(angles);
ranges = NaN(nB, 1, 'single');
valid = ~blackout;
if blackout
    return
end
px = poseL(1);  py = poseL(2);  psi = poseL(3);
rMaxEff = rMax*(1 - dLoss*dust);
pD = pDustMax*dust;
cs = 1/res;
skip = 0.999*cs;
ix0 = floor((px - x0)*res) + 1;
iy0 = floor((py - y0)*res) + 1;
inside = ix0 >= 1 && ix0 <= nx && iy0 >= 1 && iy0 <= ny;
nObs = min(double(obsN), size(obsXY, 1));

for k = 1:nB
    a = psi + angles(k);
    dx = cos(a);  dy = sin(a);
    rO = Inf;  rF = Inf;
    if inside
        % grid traversal (Amanatides-Woo) with empty-space skipping, inlined (the persistent grids are never
        % passed by value)
        if dx > 0
            sx = 1;  ox = 0;  ivx = 1/dx;
        elseif dx < 0
            sx = -1; ox = -1; ivx = 1/dx;
        else
            sx = 0;  ox = 0;  ivx = 0;
        end
        if dy > 0
            sy = 1;  oy = 0;  ivy = 1/dy;
        elseif dy < 0
            sy = -1; oy = -1; ivy = 1/dy;
        else
            sy = 0;  oy = 0;  ivy = 0;
        end
        ix = ix0;  iy = iy0;  t = 0;
        while true
            D = dg(ix, iy);
            if D >= 3
                % every cell within Chebyshev distance D - 1 of the current one is empty
                t = t + (double(D) - 1)*skip;
                if t > rMax, break, end
                ix = floor((px + t*dx - x0)*res) + 1;
                iy = floor((py + t*dy - y0)*res) + 1;
                if ix < 1 || ix > nx || iy < 1 || iy > ny, break, end
                continue
            end
            % one traversal step out of the current cell
            tMx = Inf;  tMy = Inf;
            if sx ~= 0, tMx = (x0 + (ix + ox)*cs - px)*ivx; end
            if sy ~= 0, tMy = (y0 + (iy + oy)*cs - py)*ivy; end
            if tMx < tMy
                ix = ix + sx;  t = tMx;
            else
                iy = iy + sy;  t = tMy;
            end
            if ix < 1 || ix > nx || iy < 1 || iy > ny || t > rMax
                break
            end
            g = cg(ix, iy);
            if g ~= 0
                if g >= 2 && isinf(rF)
                    rF = t;                                % first film cell
                end
                if bitand(g, uint8(1)) ~= 0
                    id = idg(ix, iy);
                    if id == 0 || id == intmax('uint16')
                        rO = t;                            % box (or overlapping obstacles): cell entry
                        break
                    end
                    % cylinder: exact intersection; a ray that only grazes the cell continues
                    j = double(id);
                    fx = px - cyl(j, 1);  fy = py - cyl(j, 2);  rc = cyl(j, 3);
                    b = fx*dx + fy*dy;
                    cc = fx*fx + fy*fy - rc*rc;
                    disc = b*b - cc;
                    if disc >= 0
                        tc = -b - sqrt(disc);
                        if tc < 0, tc = 0; end             % LiDAR inside the circle
                        rO = tc;
                        break
                    end
                end
            end
        end
    end
    % dynamic obstacles (cylinders), exact
    for j = 1:nObs
        rj = obsR(j);
        if rj <= 0, continue, end
        fx = px - obsXY(j, 1);  fy = py - obsXY(j, 2);
        b = fx*dx + fy*dy;
        cc = fx*fx + fy*fy - rj*rj;
        disc = b*b - cc;
        if disc >= 0
            sq = sqrt(disc);
            t = -b - sq;
            if t <= 0, t = -b + sq; end
            if t > 0 && t < rO, rO = t; end
        end
    end
    % semi-transparent film in front of the solid return
    r = rO;
    if rF < rO && uFilm(k) < pRetFilm
        r = rF;
    end
    if isinf(r)
        continue                                   % no return (NaN), dust clutter below still possible
    end
    r = r + sigR*(1 + kR*r)*nR(k);
    if r > rMaxEff || r < rMin || uDrop(k) < pDrop + pDropDust*dust
        r = NaN;
    end
    ranges(k) = single(r);
end
% dust clutter (spurious near returns)
if pD > 0
    for k = 1:nB
        if uDust(k) < pD
            ranges(k) = single(dR1 + (dR2 - dR1)*uDust(k)/pD);
        end
    end
end
end

% ------------------------------------------------------------------------------------------------
function [cg, idg] = local_raster(cg, idg, cyl, box, film, c, res, x0, y0, nx, ny)
% cell classes: bit 0 (1) solid, bit 1 (2) film; idg = cylinder index (0 none, intmax = several obstacles)
zb = c(10:11);
filmBoxClass = c(19);  tiltedBoxClass = c(20);
nCyl = min(c(21), size(cyl, 1));  nBox = min(c(22), size(box, 1));  nFilm = min(c(23), size(film, 1));
cs = 1/res;
many = intmax('uint16');
% ---- cylinders: every cell the circle touches ----
for k = 1:nCyl
    cx = cyl(k, 1);  cy = cyl(k, 2);  r = cyl(k, 3);
    z0 = cyl(k, 5);  z1 = z0 + cyl(k, 4);
    if ~(z0 < zb(2) && z1 > zb(1)) || r <= 0, continue, end
    i1 = max(1, floor((cx - r - x0)*res) + 1);  i2 = min(nx, floor((cx + r - x0)*res) + 1);
    j1 = max(1, floor((cy - r - y0)*res) + 1);  j2 = min(ny, floor((cy + r - y0)*res) + 1);
    for i = i1:i2
        xa = x0 + (i - 1)*cs;
        ex = max(max(xa - cx, cx - (xa + cs)), 0);         % distance of the centre to the cell in x
        for j = j1:j2
            ya = y0 + (j - 1)*cs;
            ey = max(max(ya - cy, cy - (ya + cs)), 0);
            if ex*ex + ey*ey <= r*r
                if bitand(cg(i, j), uint8(1)) ~= 0
                    idg(i, j) = many;                      % already solid: no single exact shape
                else
                    idg(i, j) = uint16(k);
                end
                cg(i, j) = bitor(cg(i, j), uint8(1));
            end
        end
    end
end
% ---- boxes ----
for k = 1:nBox
    b = box(k, :);
    cls = b(8);
    if cls == filmBoxClass, continue, end
    z0 = b(3);  z1 = b(3) + b(6);
    if ~(z0 < zb(2) && z1 > zb(1)), continue, end
    lx = b(4);  ly = b(5);  cyw = cos(b(7));  syw = sin(b(7));
    vLo = -ly/2;  vHi = ly/2;
    if cls == tiltedBoxClass && b(6) > 0
        f = min(1, max(0, (zb(2) - z0)/b(6)));
        vHi = -ly/2 + ly*f;                            % in-band strip at the low (south) edge
    end
    uLo = -lx/2;  uHi = lx/2;
    rr = sqrt(lx*lx + ly*ly)/2;
    i1 = max(1, floor((b(1) - rr - x0)*res) + 1);  i2 = min(nx, floor((b(1) + rr - x0)*res) + 1);
    j1 = max(1, floor((b(2) - rr - y0)*res) + 1);  j2 = min(ny, floor((b(2) + rr - y0)*res) + 1);
    for i = i1:i2
        xc = x0 + (i - 0.5)*cs - b(1);
        for j = j1:j2
            yc = y0 + (j - 0.5)*cs - b(2);
            u = cyw*xc + syw*yc;  v = -syw*xc + cyw*yc;
            if u >= uLo && u <= uHi && v >= vLo && v <= vHi
                [cg, idg] = local_solid(cg, idg, i, j, many);
            end
        end
    end
    % outline (closes thin / rotated walls)
    cu = [uLo uHi uHi uLo];  cv = [vLo vLo vHi vHi];
    for e = 1:4
        e2 = mod(e, 4) + 1;
        xa = b(1) + cyw*cu(e) - syw*cv(e);   ya = b(2) + syw*cu(e) + cyw*cv(e);
        xb = b(1) + cyw*cu(e2) - syw*cv(e2); yb = b(2) + syw*cu(e2) + cyw*cv(e2);
        L = sqrt((xb - xa)^2 + (yb - ya)^2);
        n = max(1, ceil(L*res*4));
        for s = 0:n
            w = s/n;
            i = floor((xa + w*(xb - xa) - x0)*res) + 1;  j = floor((ya + w*(yb - ya) - y0)*res) + 1;
            if i >= 1 && i <= nx && j >= 1 && j <= ny
                [cg, idg] = local_solid(cg, idg, i, j, many);
            end
        end
    end
end
% ---- film segments ----
for k = 1:nFilm
    f = film(k, :);
    L = sqrt((f(3) - f(1))^2 + (f(4) - f(2))^2);
    n = max(1, ceil(L*res*4));
    for s = 0:n
        w = s/n;
        i = floor((f(1) + w*(f(3) - f(1)) - x0)*res) + 1;  j = floor((f(2) + w*(f(4) - f(2)) - y0)*res) + 1;
        if i >= 1 && i <= nx && j >= 1 && j <= ny
            cg(i, j) = bitor(cg(i, j), uint8(2));
        end
    end
end
end

function [cg, idg] = local_solid(cg, idg, i, j, many)
% box cell: solid without an exact shape (a cylinder cell that a box also covers loses its exact shape)
if bitand(cg(i, j), uint8(1)) ~= 0 && idg(i, j) ~= 0
    idg(i, j) = many;
end
cg(i, j) = bitor(cg(i, j), uint8(1));
end

function D = local_chebyshev(G, nx, ny)
% Chebyshev (chessboard) distance in cells to the nearest non-empty cell, exact two-pass transform,
% computed in double and stored saturated at 255 (no integer overflow)
D = zeros(nx, ny, 'uint8');
for j = 1:ny
    for i = 1:nx
        if G(i, j) ~= 0
            continue
        end
        d = 255;
        if i > 1, d = min(d, double(D(i - 1, j)) + 1); end
        if j > 1
            d = min(d, double(D(i, j - 1)) + 1);
            if i > 1, d = min(d, double(D(i - 1, j - 1)) + 1); end
            if i < nx, d = min(d, double(D(i + 1, j - 1)) + 1); end
        end
        D(i, j) = uint8(d);
    end
end
for j = ny:-1:1
    for i = nx:-1:1
        d = double(D(i, j));
        if d == 0
            continue
        end
        if i < nx, d = min(d, double(D(i + 1, j)) + 1); end
        if j < ny
            d = min(d, double(D(i, j + 1)) + 1);
            if i < nx, d = min(d, double(D(i + 1, j + 1)) + 1); end
            if i > 1, d = min(d, double(D(i - 1, j + 1)) + 1); end
        end
        D(i, j) = uint8(min(d, 255));
    end
end
end
