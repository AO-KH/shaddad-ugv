function G = world_scn_greenhouse(W, Dsv, rs)
%WORLD_SCN_GREENHOUSE  Scenario geometry of world 4 'greenhouse' (build plan T02.3). Called by world_build.
%
%   G = world_scn_greenhouse(W, Dsv, rs)      (rs unused: no random draws)
%
%   House [0 9.6] x [0 37.5] m (inner faces of the walls, 360 m^2) on the grid x -10..20, y -10..48.
%   Floor soil 4 inside, soil 2 outside; flat terrain. PE-film walls: film segments on the inner wall
%   faces plus wall boxes class 4 (0.1 m thick, 3 m high) outside the house footprint, leaving a 3 m
%   door in the south wall (x 3.3..6.3). Steel posts cyl class 2 r 0.05, h 3 m, every 3.0 m along
%   x = 0.15 and x = 9.45 (y = 0.75 + 3k, k = 0..12; post edge 0.20 m, D54). GNSS zone = house polygon,
%   svf Dsv.greenhouse (0.12), type 1. Shade 0.6 and veg 1.5 inside (1.0 outside). Start [4.8 -2 pi/2],
%   dock [4.8 -4 pi/2]; one row along the house axis x = 4.8. No raised beds (D54).
%#ok<*INUSL,*INUSD>

res = 0.25;
xE = -10:res:20;  yN = -10:res:48;
[X, Y] = ndgrid(xE, yN);
Lx = 9.6;  Ly = 37.5;  t = 0.1;  hW = 3.0;  door = [3.3 6.3];
inside = X >= 0 & X <= Lx & Y >= 0 & Y <= Ly;

H = zeros(size(X));
soilId = 2*ones(size(X));  soilId(inside) = 4;
veg   = ones(size(X));     veg(inside) = 1.5;
shade = ones(size(X));     shade(inside) = 0.6;

%% ---- walls: film on the inner faces, boxes outside the footprint (corners covered by N/S walls) ----
film = [0 0 0 Ly                % west
        Lx 0 Lx Ly              % east
        0 Ly Lx Ly              % north
        0 0 door(1) 0           % south, west of the door
        door(2) 0 Lx 0];        % south, east of the door
%        xE           yN       zBase lx                   ly      lz  yaw class id
box  = [-t/2          Ly/2     0     t                    Ly      hW  0   4     1   % west
        Lx + t/2      Ly/2     0     t                    Ly      hW  0   4     2   % east
        Lx/2          Ly + t/2 0     Lx + 2*t             t       hW  0   4     3   % north
        (door(1) - t)/2  -t/2  0     door(1) + t          t       hW  0   4     4   % south-west
        (door(2) + Lx + t)/2 -t/2 0  Lx + t - door(2)     t       hW  0   4     5]; % south-east

%% ---- steel posts ----
yPost = 0.75 + 3.0*(0:12)';
nPo = numel(yPost);
cyl = [0.15*ones(nPo,1) yPost 0.05*ones(nPo,1) hW*ones(nPo,1) zeros(nPo,1) 2*ones(nPo,1)
       9.45*ones(nPo,1) yPost 0.05*ones(nPo,1) hW*ones(nPo,1) zeros(nPo,1) 2*ones(nPo,1)];

%% ---- GNSS zone, rows, boundary, poses ----
zones = struct('X', [0 Lx Lx 0], 'Y', [0 0 Ly Ly], 'svf', Dsv.greenhouse, 'type', 1);
rows = [Lx/2 0 Lx/2 Ly 1];

G = struct('xE', xE, 'yN', yN, 'H', H, 'soilId', soilId, 'veg', veg, 'shade', shade, ...
    'cyl', cyl, 'box', box, 'film', film, 'canopy', zeros(0, 3), 'zones', zones, 'rows', rows, ...
    'boundary', [-3 -8; 12.6 -8; 12.6 40.5; -3 40.5], 'start', [4.8 -2 pi/2], 'dock', [4.8 -4 pi/2], ...
    'info', struct('house', [0 Lx 0 Ly], 'door', door, 'wallThickness', t));
end
