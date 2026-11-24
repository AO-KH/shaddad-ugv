function W = param_world(worldId, g, R)
%PARAM_WORLD  Shared world / terrain / environment model P.world (owner: structure).
%   W = param_world(worldId, g)
%   W = param_world(7, g, R)
%     worldId : 1 flat_test (default), 2 slope_test, 3 palm_farm, 4 greenhouse, 5 solar_farm,
%               6 soil_strips, 7 row_field. Ids 1-2 and 7 are generated here procedurally; ids 3-6 are
%               built by utilities/world_build.m (build task T02) and error if it is not yet available.
%     g       : gravity [m/s^2] (P.frames.g)
%     R       : RC row test settings (param_rowtest, P.scen.rowtest), world 7 only (default param_rowtest()):
%               row_field is sized to the rows of row_test_rows(R)
%
%   Single source for: DEM + soil + vegetation grids (STRUCTURE contact), obstacles / canopy /
%   greenhouse film / GNSS sky zones (SENSORS), rows / boundary / occupancy grid / dock (GNC,
%   Sim_Monitors), ambient conditions (structure_env -> Bus_EnvState).
%   Grids: Table(i,j) <-> (xE(i), yN(j)); site ENU at lla0. Occupancy grid: binaryOccupancyMap
%   semantics (row 1 = max y), P.world.occ.res cells/m, origin = lower-left corner [xE yN].
%   Every list has a count field (n*) and at least one (dummy) row so MATLAB Function parameters
%   are never empty.
%
%   Field contract (world_build must produce exactly these):
%     id, lla0, env{T_amb,G_solar,T_ground_offset,T_sky_offset,g,use_diurnal,diurnal}, soilTable,
%     soilDefault, terrain{res,xE,yN,H,dHdE,dHdN}, soilId, soil{muPeak,K,muLat,fRoll,kScale},
%     veg.rho, shade, xlim, ylim, obstacles{cyl,nCyl,box,nBox,film,nFilm}, canopy, nCanopy,
%     gnssZones{X,Y,nv,svf,type,n}, rows, nRows, boundary, start, dock{pose,tol_xy,tol_yaw,v_max},
%     occ{res,origin,grid}, gnssSvf (grid, sky-view factor 0..1 incl. canopy attenuation),
%     gnssZoneId (grid, uint8-valued double: 0 open, else gnssZones.type)  [D59: replaces sens_env_skyview]
%   cyl  rows [xE yN r h zBase class]  class 1 palm trunk, 2 post, 3 solar pier, 7 person-size static
%   box  rows [xE yN zBase lx ly lz yaw class id]  class 4 greenhouse wall, 5 raised bed / box,
%        6 solar table (zBase = lower edge)
%        (cyl zBase / h and box zBase / lz are heights above the local ground)
%   film rows [x1 y1 x2 y2]  semi-transparent greenhouse film segments (LiDAR return p = 0.6)
%   gnssZones: polygon k = (X(k,1:nv(k)), Y(k,1:nv(k))), sky-view factor svf(k),
%        type 1 greenhouse, 2 under panel, 3 between panels
if nargin < 1 || isempty(worldId), worldId = 1; end
if nargin < 2 || isempty(g), g = 9.7876; end

W.id   = worldId;
W.lla0 = [24.1556 47.3120 430];              % Al-Kharj palm district (deg, deg, m)

%% ---- environment ----
W.env.T_amb          = 318.15;               % K (45 C design)
W.env.G_solar        = 1000;                 % W/m^2 open sky
W.env.T_ground_offset = 15;                  % K above ambient (sunlit soil)
W.env.T_sky_offset   = -12;                  % K relative to ambient
W.env.g              = g;
W.env.use_diurnal    = false;                % true: sinusoidal day profile below
W.env.diurnal = struct('T_mean',311.15,'T_amp',9,'h_peak',15,'t0_hour',6,'G_pk',1000);

%% ---- soil table [id muPeak K muLat fRoll kScale] (Wong-based, see structure spec 6.6) ----
W.soilTable = [1 0.60 0.025 0.45 0.10  0.5    % dry sand
               2 0.66 0.025 0.55 0.07  0.8    % farm soil, sandy loam (default)
               3 0.50 0.030 0.45 0.12  0.6    % wet / irrigated loam
               4 0.80 0.010 0.60 0.05  1.0    % greenhouse floor
               5 0.90 0.005 0.75 0.035 1.0];  % asphalt / concrete
W.soilDefault = 2;

if worldId <= 2
    W = local_procedural(W, worldId);
elseif worldId == 7
    if nargin < 3 || isempty(R), R = param_rowtest(); end
    W = local_row_field(W, R);
else
    if exist('world_build', 'file') ~= 2
        error('param_world:notBuilt', ['World id %d needs utilities/world_build.m (build task T02). ' ...
              'Use worldId 1 (flat_test) or 2 (slope_test) until then.'], worldId);
    end
    W = world_build(W, worldId);
end
end

% =====================================================================================
function W = local_procedural(W, worldId)
res = 0.25;
xE = -10:res:50;  yN = -10:res:50;           % 60 x 60 m, 241 x 241 grid
[X, Y] = ndgrid(xE, yN);
H = zeros(size(X));
soilId = 2*ones(size(X));
if worldId == 1
    % flat_test: farm soil, asphalt strip y in [20 26], sand strip y in [30 36]
    soilId(Y >= 20 & Y < 26) = 5;
    soilId(Y >= 30 & Y < 36) = 1;
    start = [0 0 0];
else
    % slope_test: 20 deg ramp along +x from x = 10 to 30 m with 3 m fillets, plateau after
    th = 20*pi/180; x0 = 10; x1 = 30; f = 3;
    s = local_ramp(X, x0, x1, f);            % smooth 0..1 arc-length factor
    H = tan(th) * s;
    start = [0 0 0];
end
W.terrain.res = res;  W.terrain.xE = xE;  W.terrain.yN = yN;
W.terrain.H = H;
[W.terrain.dHdE, W.terrain.dHdN] = local_grad(H, res);
W.soilId = soilId;
props = {'muPeak','K','muLat','fRoll','kScale'};
for k = 1:numel(props)
    tab = W.soilTable(:, k+1);
    W.soil.(props{k}) = reshape(tab(soilId(:)), size(soilId));
end
W.veg.rho = ones(size(X));                    % medium vegetation everywhere
W.shade   = ones(size(X));                    % solar transmission factor 0..1 (greenhouse 0.6, under panels 0.2)
W.xlim = [xE(1) xE(end)];  W.ylim = [yN(1) yN(end)];

% obstacles: cylinders [xE yN r h zBase class], boxes [xE yN zBase lx ly lz yaw class id]
W.obstacles.cyl  = [25 10 0.35 6 0 1];  W.obstacles.nCyl = 1;      % one palm trunk
W.obstacles.box  = [40 40 0 0.6 0.6 0.6 0 5 1];  W.obstacles.nBox = 1;
W.obstacles.film = zeros(1,4);  W.obstacles.nFilm = 0;             % greenhouse film segments [x1 y1 x2 y2]
W.canopy  = [25 10 3.0];  W.nCanopy = 1;                           % palm canopy circles [xE yN R]
W.gnssZones.X = zeros(1,8); W.gnssZones.Y = zeros(1,8);
W.gnssZones.nv = 0; W.gnssZones.svf = 1; W.gnssZones.type = 0; W.gnssZones.n = 0;
W.gnssSvf    = ones(size(X));                 % precomputed sky-view grid (2-D Lookup Table in sens_gnss_model)
W.gnssZoneId = zeros(size(X));                % precomputed zone-type grid (2-D Lookup Table, Nearest)
c = W.canopy(1,:); W.gnssSvf(((X - c(1)).^2 + (Y - c(2)).^2) <= c(3)^2) = 0.45;   % canopy attenuation (P.sensors.gnss.svf.canopyAtt)
W.rows = [0 0 40 0 1];  W.nRows = 1;                                % [x1E y1N x2E y2N laneId]
W.boundary = [-8 -8; 48 -8; 48 48; -8 48];
W.start = start;
W.dock = struct('pose', [-5 0 0], 'tol_xy', 0.05, 'tol_yaw', 3*pi/180, 'v_max', 0.05);
% occupancy grid at 10 cells/m from cylinders and boxes (inflation 0)
W.occ.res = 10;  W.occ.origin = [xE(1) yN(1)];
nx = round((xE(end)-xE(1))*W.occ.res);  ny = round((yN(end)-yN(1))*W.occ.res);
grid = false(ny, nx);
xc = W.occ.origin(1) + ((1:nx) - 0.5)/W.occ.res;
yc = W.occ.origin(2) + ((ny:-1:1) - 0.5)/W.occ.res;    % row 1 = max y
[XC, YC] = meshgrid(xc, yc);
for k = 1:W.obstacles.nCyl
    c = W.obstacles.cyl(k,:);
    grid = grid | ((XC-c(1)).^2 + (YC-c(2)).^2 <= c(3)^2);
end
for k = 1:W.obstacles.nBox
    b = W.obstacles.box(k,:);
    grid = grid | (abs(XC-b(1)) <= b(4)/2 & abs(YC-b(2)) <= b(5)/2);
end
W.occ.grid = grid;
end

function W = local_row_field(W, R)
% row_field (RC row test, docs/superpowers/specs/2026-10-27-rc-row-test-design.md section 6): flat, farm soil
% everywhere, no obstacles, the rows of row_test_rows(R). The 241 x 241 grid of worlds 1-2 is kept and centred on
% the field (the rectangle of the row lines); its resolution follows the field: res = max(0.25, span/240) rounded up
% to a multiple of 1/64 m, span = the largest side of the field plus a headland and the margin at each end
% (R.headland, R.margin); the centre is rounded to the same 1/64 m lattice.
rows = row_test_rows(R);
x = rows(:, [1 3]);  y = rows(:, [2 4]);
fx = [min(x(:)) max(x(:))];  fy = [min(y(:)) max(y(:))];
span = max(diff(fx), diff(fy)) + 2*(R.headland + R.margin);
q = 1/64;                                    % the step and the centre lie on a 1/64 m lattice, so the grid is evenly
res = max(0.25, ceil(span/240/q)*q);         % spaced to the last bit, which the plant's 2-D Lookup Tables on P.world
c = round([mean(fx) mean(fy)]/q)*q;          % ('Evenly spaced points') require
xE = c(1) + (-120:120)*res;  yN = c(2) + (-120:120)*res;
H = zeros(numel(xE), numel(yN));             % flat
soilId = 2*ones(size(H));                    % farm soil (sandy loam) everywhere
W.terrain.res = res;  W.terrain.xE = xE;  W.terrain.yN = yN;
W.terrain.H = H;
[W.terrain.dHdE, W.terrain.dHdN] = local_grad(H, res);
W.soilId = soilId;
props = {'muPeak','K','muLat','fRoll','kScale'};
for k = 1:numel(props)
    tab = W.soilTable(:, k+1);
    W.soil.(props{k}) = reshape(tab(soilId(:)), size(soilId));
end
W.veg.rho = ones(size(H));                    % medium vegetation everywhere (as world 1)
W.shade   = ones(size(H));                    % open sky
W.xlim = [xE(1) xE(end)];  W.ylim = [yN(1) yN(end)];

% no obstacles: every list keeps one dummy (all-zero) row, its count is 0
W.obstacles.cyl  = zeros(1,6);  W.obstacles.nCyl = 0;
W.obstacles.box  = zeros(1,9);  W.obstacles.nBox = 0;
W.obstacles.film = zeros(1,4);  W.obstacles.nFilm = 0;
W.canopy  = zeros(1,3);  W.nCanopy = 0;
W.gnssZones.X = zeros(1,8); W.gnssZones.Y = zeros(1,8);
W.gnssZones.nv = 0; W.gnssZones.svf = 1; W.gnssZones.type = 0; W.gnssZones.n = 0;
W.gnssSvf    = ones(size(H));                 % open sky everywhere (no zones, no canopy)
W.gnssZoneId = zeros(size(H));
W.rows = rows;  W.nRows = R.n_rows;                                 % [x1E y1N x2E y2N laneId]
W.boundary = [xE(1)+1 yN(1)+1; xE(end)-1 yN(1)+1; xE(end)-1 yN(end)-1; xE(1)+1 yN(end)-1];   % the grid edge minus 1 m
W.start = [-R.headland/2 0 0];                % half a headland before row 1, heading east (start of row_test_prm)
% dock (not used by the row test) at the outer edge of the headland before row 1: off the driven path, so the
% robot never stands on it (dock contact would connect the charger)
W.dock = struct('pose', [-R.headland 0 0], 'tol_xy', 0.05, 'tol_yaw', 3*pi/180, 'v_max', 0.05);
% occupancy grid at 10 cells/m as in worlds 1-2, empty
W.occ.res = 10;  W.occ.origin = [xE(1) yN(1)];
nx = round((xE(end)-xE(1))*W.occ.res);  ny = round((yN(end)-yN(1))*W.occ.res);
W.occ.grid = false(ny, nx);
end

function s = local_ramp(X, x0, x1, f)
% height factor (in metres along x) of a ramp with parabolic fillets of length f
s = zeros(size(X));
a = X > x0 - f/2 & X <= x0 + f/2;  s(a) = (X(a) - (x0 - f/2)).^2 / (2*f);
b = X > x0 + f/2 & X <= x1 - f/2;  s(b) = (X(b) - x0);
c = X > x1 - f/2 & X <= x1 + f/2;  L = x1 - x0;
s(c) = L - (x1 + f/2 - X(c)).^2/(2*f);
s(X > x1 + f/2) = L;
end

function [gx, gy] = local_grad(H, res)
[gx, gy] = deal(zeros(size(H)));
gx(2:end-1,:) = (H(3:end,:) - H(1:end-2,:))/(2*res);
gx(1,:) = (H(2,:) - H(1,:))/res;  gx(end,:) = (H(end,:) - H(end-1,:))/res;
gy(:,2:end-1) = (H(:,3:end) - H(:,1:end-2))/(2*res);
gy(:,1) = (H(:,2) - H(:,1))/res;  gy(:,end) = (H(:,end) - H(:,end-1))/res;
end
