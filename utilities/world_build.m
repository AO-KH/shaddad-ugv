function [W, info] = world_build(W, id)
%WORLD_BUILD  Build the scenario worlds 3..6 of P.world (owner: structure, build task T02).
%
%   W = world_build(W, id)       W: the partial struct of params/param_world.m (id, lla0, env,
%                                soilTable, soilDefault); id 3 palm_farm, 4 greenhouse, 5 solar_farm,
%                                6 soil_strips. Normally called by param_world(id, g).
%   [W, info] = world_build(...) info: build diagnostics (name, build time, the scenario
%                                builder's components, e.g. info.comp.H_rough of palm_farm)
%
%   Produces exactly the field contract of params/param_world.m (same field names and order as the
%   procedural worlds 1-2, checked with orderfields against param_world(1)) with the same conventions:
%     - terrain grid res 0.25 m, Table(i,j) <-> (xE(i), yN(j)) (ndgrid), central-difference gradients
%       (one-sided at the edges, identical to MATLAB gradient()),
%     - soil property grids from soilTable by soilId, blended over one cell (3x3 box filter, edge
%       replicated); soilId itself stays integer (Nearest lookups),
%     - occ from world_occupancy (10 cells/m, row 1 = max y), gnssSvf / gnssZoneId from
%       world_gnss_grids (world-side svf defaults = P.sensors.gnss.svf, D59),
%     - every list padded to at least one (dummy, all-zero) row; consumers use rows 1..n* only,
%     - random draws only from RandStream('mt19937ar','Seed',20261026) (P-independent, a fresh
%       stream per build, so every world is reproducible on its own),
%     - obstacle heights (cyl zBase/h, box zBase/lz) are above the local ground.
%   Scenario geometry: utilities/world_scn_<name>.m (docs/build_plan.md T02.2-T02.5).

t0 = tic;
if nargin < 2 || isempty(id), id = W.id; end
names = {'', '', 'palm_farm', 'greenhouse', 'solar_farm', 'soil_strips'};
if ~(isscalar(id) && any(id == 3:6))
    error('world_build:id', 'world_build builds ids 3..6 (palm_farm, greenhouse, solar_farm, soil_strips); got %s', mat2str(id));
end
rs  = RandStream('mt19937ar', 'Seed', 20261026);
Dsv = world_gnss_grids('defaults');
G   = feval(['world_scn_' names{id}], W, Dsv, rs);

% template (procedural flat_test): field order, dock tolerances, occupancy resolution
T = param_world(1, W.env.g);

%% ---- assemble in the contract order ----
R = struct();
R.id = id;
R.lla0 = W.lla0;
R.env = W.env;
R.soilTable = W.soilTable;
R.soilDefault = W.soilDefault;
res = G.xE(2) - G.xE(1);
R.terrain = struct('res', res, 'xE', G.xE, 'yN', G.yN, 'H', G.H, 'dHdE', [], 'dHdN', []);
[R.terrain.dHdE, R.terrain.dHdN] = local_grad(G.H, res);
R.soilId = G.soilId;
props = {'muPeak', 'K', 'muLat', 'fRoll', 'kScale'};
[ok, row] = ismember(G.soilId, W.soilTable(:, 1));
if ~all(ok(:))
    error('world_build:soil', '%s: soilId values not in soilTable', names{id});
end
for k = 1:numel(props)
    raw = reshape(W.soilTable(row(:), k + 1), size(G.soilId));
    R.soil.(props{k}) = local_box3(raw);
end
R.veg = struct('rho', G.veg);
R.shade = G.shade;
R.xlim = [G.xE(1) G.xE(end)];
R.ylim = [G.yN(1) G.yN(end)];
R.obstacles = struct('cyl', local_pad(G.cyl, 6), 'nCyl', size(G.cyl, 1), ...
                     'box', local_pad(G.box, 9), 'nBox', size(G.box, 1), ...
                     'film', local_pad(G.film, 4), 'nFilm', size(G.film, 1));
R.canopy  = local_pad(G.canopy, 3);
R.nCanopy = size(G.canopy, 1);
R.gnssZones = local_zones(G.zones, size(T.gnssZones.X, 2));
R.gnssSvf = [];  R.gnssZoneId = [];
R.rows  = local_pad(G.rows, 5);
R.nRows = size(G.rows, 1);
R.boundary = G.boundary;
R.start = G.start;
R.dock = struct('pose', G.dock, 'tol_xy', T.dock.tol_xy, 'tol_yaw', T.dock.tol_yaw, 'v_max', T.dock.v_max);
R.occ = struct('res', T.occ.res, 'origin', [R.xlim(1) R.ylim(1)], 'grid', []);
R.occ = world_occupancy(R);
[R.gnssSvf, R.gnssZoneId] = world_gnss_grids(R, struct('gnss', struct('svf', Dsv)));

% same field order as the procedural worlds (errors if a field is missing or extra)
W = orderfields(R, T);
info = struct('name', names{id}, 't_build', toc(t0), 'comp', G.info);
end

% =============================================================================================
function [gx, gy] = local_grad(H, res)
% central differences inside, one-sided at the edges (identical to param_world / gradient())
[gx, gy] = deal(zeros(size(H)));
gx(2:end-1,:) = (H(3:end,:) - H(1:end-2,:))/(2*res);
gx(1,:) = (H(2,:) - H(1,:))/res;  gx(end,:) = (H(end,:) - H(end-1,:))/res;
gy(:,2:end-1) = (H(:,3:end) - H(:,1:end-2))/(2*res);
gy(:,1) = (H(:,2) - H(:,1))/res;  gy(:,end) = (H(:,end) - H(:,end-1))/res;
end

function B = local_box3(A)
% 3x3 box filter with replicated edges (blend over one cell)
Ap = A([1 1:end end], [1 1:end end]);
B = conv2(Ap, ones(3)/9, 'valid');
end

function M = local_pad(M, nc)
if isempty(M), M = zeros(1, nc); end
if size(M, 2) ~= nc
    error('world_build:list', 'list has %d columns, expected %d', size(M, 2), nc);
end
end

function Z = local_zones(zones, nvMax)
n = numel(zones);
if n == 0
    Z = struct('X', zeros(1, nvMax), 'Y', zeros(1, nvMax), 'nv', 0, 'svf', 1, 'type', 0, 'n', 0);
    return
end
nvMax = max([nvMax, arrayfun(@(z) numel(z.X), zones)]);
Z = struct('X', zeros(n, nvMax), 'Y', zeros(n, nvMax), 'nv', zeros(n, 1), ...
           'svf', zeros(n, 1), 'type', zeros(n, 1), 'n', n);
for k = 1:n
    nv = numel(zones(k).X);
    Z.X(k, 1:nv) = zones(k).X;  Z.Y(k, 1:nv) = zones(k).Y;
    Z.nv(k) = nv;  Z.svf(k) = zones(k).svf;  Z.type(k) = zones(k).type;
end
end
