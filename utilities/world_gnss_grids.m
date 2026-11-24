function [svf, zoneId] = world_gnss_grids(W, Psens)
%WORLD_GNSS_GRIDS  Precomputed GNSS sky-view factor and zone-type grids on the terrain grid (D59).
%
%   [svf, zoneId] = world_gnss_grids(W, Psens)
%     W      : P.world (needs terrain.xE/yN, gnssZones, canopy, nCanopy)
%     Psens  : P.sensors, or any struct with Psens.gnss.svf.canopyAtt / .svf_min; [] or omitted ->
%              the world-side defaults below (world_build runs before P.sensors exists)
%     svf    : numel(xE) x numel(yN) double, Table(i,j) <-> (xE(i), yN(j)); zone svf inside each
%              gnssZones polygon (inpolygon, min over overlapping zones), times canopyAtt under any
%              canopy circle, floored at svf_min; 1 in the open
%     zoneId : same size, uint8-valued double: gnssZones.type of the zone that sets the minimum
%              svf (0 = open sky)
%   D = world_gnss_grids('defaults')   world-side copy of P.sensors.gnss.svf (open, greenhouse,
%              underPanel, betweenPanels, canopyAtt, svf_min). tests/structure/t_world asserts it equals
%              P.sensors.gnss.svf, so the sensor parameters stay the single source of the numbers.
%
%   The grids are read by two 2-D Lookup Table blocks in the GNSS sensor model (svf Linear,
%   zoneId Nearest) at the antenna xy (build plan T02.7 / T11).

if ischar(W) || isstring(W)
    if strcmpi(W, 'defaults')
        svf = local_defaults();
        return
    end
    error('world_gnss_grids:arg', 'world_gnss_grids(W, Psens) | world_gnss_grids(''defaults'')');
end
if nargin < 2 || isempty(Psens)
    sv = local_defaults();
else
    sv = Psens.gnss.svf;
end

xE = W.terrain.xE(:);  yN = W.terrain.yN(:);
[X, Y] = ndgrid(xE, yN);
svf    = ones(size(X));
zoneId = zeros(size(X));

%% ---- zone polygons: min svf over overlapping zones ----
Z = W.gnssZones;
for k = 1:Z.n
    nv = Z.nv(k);
    xv = Z.X(k, 1:nv);  yv = Z.Y(k, 1:nv);
    % bounding-box window for speed
    ix = find(xE >= min(xv) - 1e-9 & xE <= max(xv) + 1e-9);
    iy = find(yN >= min(yv) - 1e-9 & yN <= max(yv) + 1e-9);
    if isempty(ix) || isempty(iy), continue, end
    Xs = X(ix, iy);  Ys = Y(ix, iy);
    in = inpolygon(Xs, Ys, xv, yv);
    S  = svf(ix, iy);  Zi = zoneId(ix, iy);
    lower = in & (Z.svf(k) < S);
    setId = lower | (in & Zi == 0);
    S(lower)  = Z.svf(k);
    Zi(setId) = Z.type(k);
    svf(ix, iy) = S;  zoneId(ix, iy) = Zi;
end

%% ---- canopy attenuation (multiplicative, once per grid point) ----
under = false(size(X));
for i = 1:W.nCanopy
    c = W.canopy(i, :);
    ix = find(abs(xE - c(1)) <= c(3));
    iy = find(abs(yN - c(2)) <= c(3));
    if isempty(ix) || isempty(iy), continue, end
    under(ix, iy) = under(ix, iy) | ((X(ix, iy) - c(1)).^2 + (Y(ix, iy) - c(2)).^2 <= c(3)^2);
end
svf(under) = svf(under) * sv.canopyAtt;
svf = max(svf, sv.svf_min);
end

% ---------------------------------------------------------------------------------------------
function D = local_defaults()
% must equal param_sensors gn.svf (asserted by tests/structure/t_world)
D = struct('open', 1.0, 'greenhouse', 0.12, 'underPanel', 0.45, 'betweenPanels', 0.75, ...
           'canopyAtt', 0.45, 'svf_min', 0.02);
end
