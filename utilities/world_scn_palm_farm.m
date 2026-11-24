function G = world_scn_palm_farm(W, Dsv, rs)
%WORLD_SCN_PALM_FARM  Scenario geometry of world 3 'palm_farm' (build plan T02.2). Called by world_build.
%
%   G = world_scn_palm_farm(W, Dsv, rs)
%     W   : partial P.world (soilTable, soilDefault)     Dsv : world_gnss_grids('defaults') (unused here)
%     rs  : RandStream('mt19937ar','Seed',20261026) of world_build (only source of random draws;
%           draw order: trunk radii, canopy radii, sand discs, roughness field)
%     G   : xE yN H soilId veg shade cyl box film canopy zones rows boundary start dock info
%
%   Grid x -10..74, y -10..130 (0.25 m). 7 palm rows N-S at x = 8k+4 (k = 0..6), 13 palms per row at
%   y = 8j+12 (j = 0..12): trunks cyl class 1 (r U(0.30,0.40), h 6 m, zBase 0), canopy circles
%   R U(3.0,4.0). Irrigation basin berm around each palm: ring r 1.5 m, height 0.15 m, width 0.4 m
%   (full width at half maximum of a Gaussian profile), wet loam (soil 3) for r < 1.3 m. Sandy loam
%   (2) elsewhere with dry-sand (1) discs of r 3 m covering 10 % of the grid outside the basins.
%   Roughness 1.5 cm RMS, Gaussian correlation C(d) = s^2 exp(-d^2/lc^2), lc = 0.5 m. Slope 0.5 %
%   northwards (H += 0.005 y). 6 aisles at x = 8k+8 (k = 0..5) from y 8 to 112. Veg 1.0 (2.0 under
%   canopy), shade 0.6 under canopy. Boundary [-8 -8; 72 -8; 72 128; -8 128], start [-5 0 pi/2],
%   dock [-6 -5 pi/2].
%   info: H_slope, H_berm, H_rough (components of H), sandFrac, nSandDiscs.
%#ok<*INUSL>

res = 0.25;
xE = -10:res:74;  yN = -10:res:130;
[X, Y] = ndgrid(xE, yN);

%% ---- palms ----
xr = 8*(0:6) + 4;  yp = 8*(0:12) + 12;
[PY, PX] = ndgrid(yp, xr);                   % row by row (same x), y ascending
PX = PX(:);  PY = PY(:);  nP = numel(PX);
rT = 0.30 + 0.10*rand(rs, nP, 1);            % trunk radius U(0.30, 0.40)
RC = 3.0 + 1.0*rand(rs, nP, 1);              % canopy radius U(3.0, 4.0)
cyl    = [PX PY rT 6*ones(nP,1) zeros(nP,1) ones(nP,1)];
canopy = [PX PY RC];

%% ---- basins (berm + wet loam) and canopy grids ----
rB = 1.5;  hB = 0.15;  wB = 0.4;  rWet = 1.3;
sig = wB / (2*sqrt(2*log(2)));               % FWHM = wB
Hb = zeros(size(X));
wet = false(size(X));  underC = false(size(X));
for k = 1:nP
    rw = max(RC(k), rB + 4*sig);
    ix = find(abs(xE - PX(k)) <= rw);  iy = find(abs(yN - PY(k)) <= rw);
    rho = hypot(X(ix,iy) - PX(k), Y(ix,iy) - PY(k));
    Hb(ix,iy) = max(Hb(ix,iy), hB*exp(-0.5*((rho - rB)/sig).^2));
    wet(ix,iy) = wet(ix,iy) | rho < rWet;
    underC(ix,iy) = underC(ix,iy) | rho <= RC(k);
end

%% ---- soil: sandy loam, 10 % dry-sand discs (r 3 m), wet basins ----
soilId = 2*ones(size(X));
sand = false(size(X));
rS = 3.0;  fracTarget = 0.10;  nDisc = 0;  nFree = nnz(~wet);
while nnz(sand & ~wet)/nFree < fracTarget && nDisc < 1000
    c = [xE(1) + (xE(end) - xE(1))*rand(rs), yN(1) + (yN(end) - yN(1))*rand(rs)];
    ix = find(abs(xE - c(1)) <= rS);  iy = find(abs(yN - c(2)) <= rS);
    sand(ix,iy) = sand(ix,iy) | (X(ix,iy) - c(1)).^2 + (Y(ix,iy) - c(2)).^2 <= rS^2;
    nDisc = nDisc + 1;
end
soilId(sand) = 1;
soilId(wet)  = 3;

%% ---- roughness: Gaussian-filtered white noise, 1.5 cm RMS, lc = 0.5 m ----
sRMS = 0.015;  lc = 0.5;
sk = lc/sqrt(2)/res;                          % kernel std [cells]: C(d) ~ exp(-d^2/lc^2)
hw = ceil(4*sk);
kx = exp(-0.5*((-hw:hw)/sk).^2);  kx = kx/sum(kx);
N = randn(rs, numel(xE) + 2*hw, numel(yN) + 2*hw);
Hr = conv2(kx, kx, N, 'valid');               % columns then rows (separable)
Hr = Hr - mean(Hr(:));
Hr = Hr * (sRMS/std(Hr(:)));

%% ---- terrain ----
Hs = 0.005*Y;                                 % 0.5 % slope northwards
H  = Hs + Hb + Hr;

%% ---- vegetation and shade ----
veg = ones(size(X));    veg(underC) = 2.0;
shade = ones(size(X));  shade(underC) = 0.6;

%% ---- rows, boundary, poses ----
xa = 8*(0:5)' + 8;
rows = [xa 8*ones(6,1) xa 112*ones(6,1) (1:6)'];

G = struct('xE', xE, 'yN', yN, 'H', H, 'soilId', soilId, 'veg', veg, 'shade', shade, ...
    'cyl', cyl, 'box', zeros(0, 9), 'film', zeros(0, 4), 'canopy', canopy, ...
    'zones', struct('X', {}, 'Y', {}, 'svf', {}, 'type', {}), 'rows', rows, ...
    'boundary', [-8 -8; 72 -8; 72 128; -8 128], 'start', [-5 0 pi/2], 'dock', [-6 -5 pi/2], ...
    'info', struct('H_slope', Hs, 'H_berm', Hb, 'H_rough', Hr, ...
                   'sandFrac', nnz(soilId == 1)/numel(soilId), 'nSandDiscs', nDisc));
end
