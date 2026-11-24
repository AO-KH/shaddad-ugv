function [ok, msg] = world_check(W, varargin)
%WORLD_CHECK  Check a P.world struct against the field contract of params/param_world.m.
%
%   ok = world_check(W)              true when every check passes (prints nothing)
%   [ok, msg] = world_check(W)       msg: cellstr of the failed checks
%   world_check(W)                   (no output) error 'world_check:failed' listing the failures
%   world_check(W, 'Psens', S)       also check gnssSvf / gnssZoneId against world_gnss_grids(W, S)
%                                    and occ against world_occupancy(W, S.lidar.zBand) (S = P.sensors)
%
%   Checks: exact field names and order (top level and sub-structs, against param_world(1));
%   numeric / logical only, no NaN / Inf in grids; grid vectors (res, evenly spaced, ascending) and
%   every terrain-grid field sized numel(xE) x numel(yN) (H, dHdE, dHdN, soilId, soil.*, veg.rho,
%   shade, gnssSvf, gnssZoneId); gradients equal central differences (1e-9); soilId integer ids of
%   soilTable; soil.* within the table range; gnssSvf in [svf_min, 1]; gnssZoneId integer-valued in
%   {0} U gnssZones.type; lists (cyl 6, box 9, film 4, canopy 3, rows 5 columns) have at least one row
%   and n* <= rows; zone polygons nv <= columns, n <= rows; occ grid size / origin / res; boundary
%   polygon m x 2 (m >= 3); start / dock pose 1 x 3 inside xlim / ylim and inside the boundary; start
%   and dock cells free in occ; dock tolerances positive.

ip = inputParser;
ip.addParameter('Psens', []);
ip.parse(varargin{:});
Ps = ip.Results.Psens;
msg = {};

%% ---- field contract (names and order) ----
T = param_world(1, W.env.g);
msg = [msg, local_fields(W, T, 'W')];
if ~isempty(msg)
    ok = false;
    if nargout == 0, error('world_check:failed', 'world_check failed:\n  %s', strjoin(msg, '\n  ')); end
    return
end
for f = {'env', 'terrain', 'soil', 'veg', 'obstacles', 'gnssZones', 'dock', 'occ'}
    msg = [msg, local_fields(W.(f{1}), T.(f{1}), ['W.' f{1}])]; %#ok<AGROW>
end
msg = [msg, local_fields(W.env.diurnal, T.env.diurnal, 'W.env.diurnal')];
msg = [msg, local_numeric(W, 'W')];

%% ---- grid vectors ----
tr = W.terrain;
nx = numel(tr.xE);  ny = numel(tr.yN);
if nx < 3 || ny < 3, msg{end+1} = 'terrain grid smaller than 3 x 3'; end
for v = {'xE', 'yN'}
    d = diff(tr.(v{1}));
    if any(abs(d - tr.res) > 1e-9), msg{end+1} = sprintf('terrain.%s not evenly spaced at res %g', v{1}, tr.res); end %#ok<AGROW>
end
if any(abs(W.xlim - [tr.xE(1) tr.xE(end)]) > 1e-12) || any(abs(W.ylim - [tr.yN(1) tr.yN(end)]) > 1e-12)
    msg{end+1} = 'xlim / ylim differ from the terrain grid extent';
end

%% ---- grid sizes (incl. gnssSvf / gnssZoneId, D59) ----
sz = [nx ny];
grids = {'terrain.H', tr.H; 'terrain.dHdE', tr.dHdE; 'terrain.dHdN', tr.dHdN; 'soilId', W.soilId; ...
         'veg.rho', W.veg.rho; 'shade', W.shade; 'gnssSvf', W.gnssSvf; 'gnssZoneId', W.gnssZoneId};
sp = fieldnames(W.soil);
for k = 1:numel(sp), grids(end+1, :) = {['soil.' sp{k}], W.soil.(sp{k})}; end %#ok<AGROW>
for k = 1:size(grids, 1)
    g = grids{k, 2};
    if ~isequal(size(g), sz)
        msg{end+1} = sprintf('%s is %s, expected %s', grids{k,1}, mat2str(size(g)), mat2str(sz)); %#ok<AGROW>
    elseif any(~isfinite(double(g(:))))
        msg{end+1} = sprintf('%s has NaN / Inf', grids{k,1}); %#ok<AGROW>
    end
end
if ~isempty(msg), [ok, msg] = local_out(msg, nargout); return, end

%% ---- gradients ----
[gN, gE] = gradient(tr.H, tr.res);           % gradient: 1st output along dim 2 (yN), 2nd along dim 1 (xE)
eG = max([max(abs(gE(:) - tr.dHdE(:))), max(abs(gN(:) - tr.dHdN(:)))]);
if eG > 1e-9, msg{end+1} = sprintf('gradient error %.3g > 1e-9', eG); end

%% ---- soil ----
ids = W.soilTable(:, 1);
if any(W.soilId(:) ~= round(W.soilId(:))) || ~all(ismember(W.soilId(:), ids))
    msg{end+1} = 'soilId has values outside soilTable ids';
end
if ~ismember(W.soilDefault, ids), msg{end+1} = 'soilDefault not in soilTable'; end
for k = 1:numel(sp)
    col = W.soilTable(:, k + 1);
    v = W.soil.(sp{k});
    if min(v(:)) < min(col) - 1e-12 || max(v(:)) > max(col) + 1e-12
        msg{end+1} = sprintf('soil.%s outside the soilTable range', sp{k}); %#ok<AGROW>
    end
end

%% ---- lists ----
O = W.obstacles;
L = {'obstacles.cyl', O.cyl, O.nCyl, 6; 'obstacles.box', O.box, O.nBox, 9; ...
     'obstacles.film', O.film, O.nFilm, 4; 'canopy', W.canopy, W.nCanopy, 3; 'rows', W.rows, W.nRows, 5};
for k = 1:size(L, 1)
    [nm, M, n, nc] = L{k, :};
    if size(M, 1) < 1 || size(M, 2) ~= nc
        msg{end+1} = sprintf('%s is %s (need >= 1 row, %d columns)', nm, mat2str(size(M)), nc); %#ok<AGROW>
    end
    if ~isscalar(n) || n < 0 || n ~= round(n) || n > size(M, 1)
        msg{end+1} = sprintf('%s count %s inconsistent with %d rows', nm, mat2str(n), size(M, 1)); %#ok<AGROW>
    end
end
if O.nCyl > 0 && any(O.cyl(1:O.nCyl, 3) <= 0 | O.cyl(1:O.nCyl, 4) <= 0), msg{end+1} = 'cylinder with r or h <= 0'; end
if O.nBox > 0 && any(any(O.box(1:O.nBox, 4:6) <= 0)), msg{end+1} = 'box with a size <= 0'; end
if W.nCanopy > 0 && any(W.canopy(1:W.nCanopy, 3) <= 0), msg{end+1} = 'canopy with R <= 0'; end

Z = W.gnssZones;
if size(Z.X, 1) < max(Z.n, 1) || ~isequal(size(Z.X), size(Z.Y)) || numel(Z.nv) < max(Z.n, 1) || ...
        numel(Z.svf) < max(Z.n, 1) || numel(Z.type) < max(Z.n, 1)
    msg{end+1} = 'gnssZones arrays smaller than max(n,1)';
elseif Z.n > 0
    if any(Z.nv(1:Z.n) < 3 | Z.nv(1:Z.n) > size(Z.X, 2)), msg{end+1} = 'gnssZones.nv outside [3, columns]'; end
    if any(Z.svf(1:Z.n) <= 0 | Z.svf(1:Z.n) > 1), msg{end+1} = 'gnssZones.svf outside (0, 1]'; end
    if any(Z.type(1:Z.n) < 1 | Z.type(1:Z.n) ~= round(Z.type(1:Z.n))), msg{end+1} = 'gnssZones.type not a positive integer'; end
end

%% ---- GNSS grids ----
svfMin = world_gnss_grids('defaults').svf_min;
if ~isempty(Ps), svfMin = Ps.gnss.svf.svf_min; end
if min(W.gnssSvf(:)) < svfMin - 1e-12 || max(W.gnssSvf(:)) > 1 + 1e-12
    msg{end+1} = sprintf('gnssSvf outside [%g, 1]', svfMin);
end
zt = [0; Z.type(1:Z.n)];
if any(W.gnssZoneId(:) ~= round(W.gnssZoneId(:))) || ~all(ismember(W.gnssZoneId(:), zt))
    msg{end+1} = 'gnssZoneId values not in {0} U gnssZones.type';
end
if ~isempty(Ps)
    [s2, z2] = world_gnss_grids(W, Ps);
    if max(abs(s2(:) - W.gnssSvf(:))) > 1e-12 || ~isequal(z2, W.gnssZoneId)
        msg{end+1} = 'gnssSvf / gnssZoneId differ from world_gnss_grids(W, Psens)';
    end
end

%% ---- occupancy ----
oc = W.occ;
nxo = round(diff(W.xlim)*oc.res);  nyo = round(diff(W.ylim)*oc.res);
if ~islogical(oc.grid) || ~isequal(size(oc.grid), [nyo nxo])
    msg{end+1} = sprintf('occ.grid is %s %s, expected logical %s', class(oc.grid), mat2str(size(oc.grid)), mat2str([nyo nxo]));
end
if any(abs(oc.origin - [W.xlim(1) W.ylim(1)]) > 1e-12), msg{end+1} = 'occ.origin is not the lower-left corner'; end
if ~isempty(Ps) && isfield(Ps, 'lidar') && W.id >= 3
    o2 = world_occupancy(W, Ps.lidar.zBand);
    if ~isequal(o2.grid, oc.grid), msg{end+1} = 'occ differs from world_occupancy(W, P.sensors.lidar.zBand)'; end
end

%% ---- boundary, poses ----
if size(W.boundary, 2) ~= 2 || size(W.boundary, 1) < 3, msg{end+1} = 'boundary must be m x 2, m >= 3'; end
poses = {'start', W.start; 'dock.pose', W.dock.pose};
for k = 1:2
    p = poses{k, 2};
    if ~isequal(size(p), [1 3])
        msg{end+1} = sprintf('%s must be 1 x 3', poses{k,1}); %#ok<AGROW>
        continue
    end
    if p(1) < W.xlim(1) || p(1) > W.xlim(2) || p(2) < W.ylim(1) || p(2) > W.ylim(2)
        msg{end+1} = sprintf('%s outside the grid', poses{k,1}); %#ok<AGROW>
    end
    if size(W.boundary, 2) == 2 && ~inpolygon(p(1), p(2), W.boundary(:,1), W.boundary(:,2))
        msg{end+1} = sprintf('%s outside the boundary', poses{k,1}); %#ok<AGROW>
    end
    if islogical(oc.grid) && isequal(size(oc.grid), [nyo nxo])
        ic = min(max(floor((p(1) - oc.origin(1))*oc.res) + 1, 1), nxo);
        ir = min(max(nyo - floor((p(2) - oc.origin(2))*oc.res), 1), nyo);
        if oc.grid(ir, ic), msg{end+1} = sprintf('%s lies in an occupied cell', poses{k,1}); end %#ok<AGROW>
    end
end
if ~(W.dock.tol_xy > 0 && W.dock.tol_yaw > 0 && W.dock.v_max > 0), msg{end+1} = 'dock tolerances must be > 0'; end

[ok, msg] = local_out(msg, nargout);
end

% =============================================================================================
function [ok, msg] = local_out(msg, nout)
ok = isempty(msg);
if ~ok && nout == 0
    error('world_check:failed', 'world_check failed:\n  %s', strjoin(msg, '\n  '));
end
end

function m = local_fields(S, T, nm)
m = {};
if ~isstruct(S), m{end+1} = sprintf('%s is not a struct', nm); return, end
fs = fieldnames(S);  ft = fieldnames(T);
miss = setdiff(ft, fs, 'stable');  extra = setdiff(fs, ft, 'stable');
if ~isempty(miss),  m{end+1} = sprintf('%s missing fields: %s', nm, strjoin(miss', ', ')); end
if ~isempty(extra), m{end+1} = sprintf('%s extra fields: %s', nm, strjoin(extra', ', ')); end
if isempty(miss) && isempty(extra) && ~isequal(fs, ft)
    m{end+1} = sprintf('%s field order differs from the contract', nm);
end
end

function m = local_numeric(S, nm)
% P is numeric / logical only (D58)
m = {};
fs = fieldnames(S);
for k = 1:numel(fs)
    v = S.(fs{k});
    if isstruct(v)
        for j = 1:numel(v), m = [m, local_numeric(v(j), [nm '.' fs{k}])]; end %#ok<AGROW>
    elseif ~(isnumeric(v) || islogical(v)) || ~isreal(v)
        m{end+1} = sprintf('%s.%s is %s (numeric / logical only)', nm, fs{k}, class(v)); %#ok<AGROW>
    end
end
end
