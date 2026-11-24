function [scene, info] = sens_env_to_uavscenario(P, varargin)
%SENS_ENV_TO_UAVSCENARIO  uavScenario of the active world for the FULL3D LiDAR (owner: sensors, build task T11).
%
%   scene = sens_env_to_uavscenario(P)
%   scene = sens_env_to_uavscenario(P, 'Ground', true, 'GroundRes', 2, 'PoseB', [x y z yaw pitch roll], ...
%                                   'Merge', true)
%   [scene, info] = ...      info: object counts per class, scene meshes and faces
%   Merge true (default): all static objects in one triangulated 'custom' mesh (FULL3D cost per scan grows
%   with the number of scene meshes); false: one mesh per object.
%
%   Builds the same geometry the FAST2D model rasterises (P.world.obstacles, D10) as 3-D meshes:
%     cylinders [xE yN r h zBase class] -> 'cylinder' meshes from zBase to zBase + h above the local ground
%     boxes [xE yN zBase lx ly lz yaw class id] -> 'polygon' meshes (rotated footprint, vertical extent);
%        class P.sensors.lidar.tiltedBoxClass (solar table) -> thin tilted 'custom' quad from the low (south)
%        edge zBase to the high edge zBase + lz; class filmBoxClass (greenhouse wall) -> thin wall slab
%        (FULL3D has no semi-transparency: the film is solid in this fidelity)
%     ground (default on) -> 'surface' mesh of P.world.terrain.H decimated to GroundRes m
%   Platform 'Shaddad' (ReferenceFrame 'ENU', no own mesh) with uavSensor 'Lidar'
%   (uavLidarPointCloudGenerator with P.sensors.lidar.full3d, organised output) mounted at P.sensors.lidar.r.
%   The FULL3D choice of sens_lidar_model reads the base-workspace variable sens_uavScene through
%   uavsimlib/UAV Scenario Configuration; scripts/build/build_sensor_models.m creates it with
%   assignin('base', 'sens_uavScene', sens_env_to_uavscenario(P)). Interpreted only, short runs (risk R3).

ip = inputParser;
ip.addParameter('Ground', true);
ip.addParameter('GroundRes', 2);
ip.addParameter('PoseB', []);
ip.addParameter('Merge', true);
ip.parse(varargin{:});
o = ip.Results;

W = P.world;
E = W.obstacles;
L = P.sensors.lidar;
scene = uavScenario('UpdateRate', 1/L.Ts, 'ReferenceLocation', P.sensors.refLLA, 'StopTime', Inf);
info = struct('nCyl', 0, 'nBox', 0, 'nTilted', 0, 'nWall', 0, 'ground', false, 'nMeshes', 0, 'nFaces', 0);
hfun = @(x, y) local_height(W, x, y);
% Merge (default): every static object goes into ONE 'custom' mesh (triangulated here; cylinders as
% nSides-gon prisms). The UAV Scenario Lidar block joins and transforms all scene meshes in MATLAB at every
% scan, so the cost per scan grows with the number of meshes (measured palm farm: 92 meshes 0.27 s/scan,
% one merged mesh see t_sensors/tSEN14). Merge false keeps one addMesh per object (cylinder / polygon / surface).
nSides = 24;                                       % chord error r (1 - cos(pi/24)) = 0.9 % of r
MV = zeros(0, 3);  MF = zeros(0, 3);
    function addTri(V, F, kind, args, color)
        if o.Merge
            MF = [MF; F + size(MV, 1)];
            MV = [MV; V];
        else
            addMesh(scene, kind, args, color);
        end
    end

%% ---- ground ----
if o.Ground
    xE = W.terrain.xE;  yN = W.terrain.yN;
    st = max(1, round(o.GroundRes/W.terrain.res));
    ix = unique([1:st:numel(xE) numel(xE)]);  iy = unique([1:st:numel(yN) numel(yN)]);
    [Xg, Yg] = meshgrid(xE(ix), yN(iy));
    Zg = W.terrain.H(ix, iy)';
    [V, F] = local_surface(Xg, Yg, Zg);
    addTri(V, F, 'surface', {Xg, Yg, Zg}, [0.6 0.5 0.4]);
    info.ground = true;
end

%% ---- cylinders ----
for k = 1:E.nCyl
    c = E.cyl(k, :);
    if c(3) <= 0 || c(4) <= 0, continue, end
    z0 = hfun(c(1), c(2)) + c(5);
    th = (0:nSides - 1)'*2*pi/nSides;
    [V, F] = local_prism([c(1) + c(3)*cos(th), c(2) + c(3)*sin(th)], z0, z0 + c(4));
    addTri(V, F, 'cylinder', {[c(1) c(2) c(3)], [z0 z0 + c(4)]}, [0.45 0.3 0.2]);
    info.nCyl = info.nCyl + 1;
end

%% ---- boxes ----
for k = 1:E.nBox
    b = E.box(k, :);
    if all(b(4:6) == 0), continue, end
    cy = cos(b(7));  sy = sin(b(7));
    zg = hfun(b(1), b(2));
    if b(8) == L.tiltedBoxClass
        % tilted panel: low edge v = -ly/2 at zBase, high edge v = +ly/2 at zBase + lz (thin quad, two faces)
        u = [-1 1 1 -1]*b(4)/2;  v = [-1 -1 1 1]*b(5)/2;
        z = zg + b(3) + [0 0 b(6) b(6)];
        V = [b(1) + cy*u - sy*v; b(2) + sy*u + cy*v; z]';
        F = [1 2 3; 1 3 4; 3 2 1; 4 3 1];
        addTri(V, F, 'custom', {V, F}, [0.2 0.2 0.5]);
        info.nTilted = info.nTilted + 1;
    else
        u = [-1 1 1 -1]*b(4)/2;  v = [-1 -1 1 1]*b(5)/2;
        xy = [b(1) + cy*u - sy*v; b(2) + sy*u + cy*v]';
        [V, F] = local_prism(xy, zg + b(3), zg + b(3) + b(6));
        addTri(V, F, 'polygon', {xy, [zg + b(3), zg + b(3) + b(6)]}, [0.5 0.5 0.5]);
        if b(8) == L.filmBoxClass, info.nWall = info.nWall + 1; else, info.nBox = info.nBox + 1; end
    end
end
if o.Merge && ~isempty(MF)
    addMesh(scene, 'custom', {MV, MF}, [0.5 0.45 0.35]);
end
info.nMeshes = numel(scene.Meshes);
for k = 1:info.nMeshes, info.nFaces = info.nFaces + size(scene.Meshes{k}.Faces, 1); end

%% ---- platform + LiDAR ----
if isempty(o.PoseB)
    p0 = [W.start(1) W.start(2) hfun(W.start(1), W.start(2))];
    q0 = quaternion([W.start(3) 0 0], 'euler', 'ZYX', 'point');
else
    p0 = o.PoseB(1:3);
    q0 = quaternion(o.PoseB(4:6), 'euler', 'ZYX', 'point');
end
plat = uavPlatform('Shaddad', scene, 'ReferenceFrame', 'ENU', 'InitialPosition', p0, ...
    'InitialOrientation', compact(q0));
F3 = L.full3d;
lid = uavLidarPointCloudGenerator('UpdateRate', 1/L.Ts, 'MaxRange', F3.MaxRange, ...
    'RangeAccuracy', F3.RangeAccuracy, 'AzimuthLimits', F3.AzimuthLimits, ...
    'AzimuthResolution', F3.AzimuthResolution, 'ElevationLimits', F3.ElevationLimits, ...
    'ElevationResolution', F3.ElevationResolution, 'HasNoise', true, 'HasOrganizedOutput', true);
uavSensor('Lidar', plat, lid, 'MountingLocation', L.r, 'MountingAngles', L.eul_deg);
end

% ---------------------------------------------------------------------------------------------
function [V, F] = local_prism(xy, z0, z1)
% closed vertical prism over the CCW footprint xy (n x 2) from z0 to z1, outward-facing triangles
n = size(xy, 1);
V = [xy, z0*ones(n, 1); xy, z1*ones(n, 1); mean(xy, 1) z0; mean(xy, 1) z1];
i = (1:n)';  j = [2:n 1]';
cb = 2*n + 1;  ct = 2*n + 2;
F = [i, j, n + j; i, n + j, n + i; ...                 % sides
     ct*ones(n, 1), n + i, n + j; ...                  % top (normal up)
     cb*ones(n, 1), j, i];                             % bottom (normal down)
end

function [V, F] = local_surface(X, Y, Z)
% triangulated height field from meshgrid arrays (rows = y, columns = x), normals up
[ny, nx] = size(X);
V = [X(:) Y(:) Z(:)];
id = reshape(1:ny*nx, ny, nx);
a = id(1:end-1, 1:end-1);  b = id(1:end-1, 2:end);  c = id(2:end, 2:end);  d = id(2:end, 1:end-1);
F = [a(:) b(:) c(:); a(:) c(:) d(:)];
end

function h = local_height(W, x, y)
xE = W.terrain.xE;  yN = W.terrain.yN;
x = min(max(x, xE(1)), xE(end));  y = min(max(y, yN(1)), yN(end));
h = interp2(yN, xE, W.terrain.H, y, x, 'linear');
end
