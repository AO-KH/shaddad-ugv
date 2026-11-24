function init = structure_initial_state(P, varargin)
%STRUCTURE_INITIAL_STATE  Settled initial state of the HiFi chassis on the DEM (owner: structure, build task T03).
%
%   init = structure_initial_state(P)                         at P.world.start, soil from the blended grids
%   init = structure_initial_state(P, 'Pose', [xE yN psi_enu], 'Soil', id, 'Mass', m)
%
%   P : struct with fields world (P.world), structure (needs geom, mp, contact) and frames.g; it may be a
%       partially assembled P (param_structure calls it with struct('world',W,'structure',S,'frames',...)).
%   'Pose' : pose of the reference point R in site ENU (default P.world.start)
%   'Soil' : soil class 1..5 used for every contact point (env_ovr.use_soil), default [] = grids
%   'Mass' : total mass [kg] (default P.structure.mp.m0); the CG follows r_cg(m) of the 6DOF model
%
%   Static equilibrium of the 14 compliant contact points of structure_contact_normal (spring part
%   N_j = kc*kScale_j*pen_j, pen_j = (D_j + h_j)*n_z) for heave (NED down of R), pitch and roll at the fixed
%   horizontal position of R and the given heading (Newton iteration, numerical Jacobian). The tangential
%   part of gravity is carried by contact shear forces distributed in proportion to N_j, so the moment
%   balance includes the shift of the centre of pressure on slopes. The shear states start at 0, so a
%   parked robot on a slope settles elastically by K*s (a few mm, T8) after t = 0.
%   With the mower fitted (P.structure.contact.gw.on = 1, 2026-11-15) the two gauge wheels of the rigid deck
%   join with their normal spring force only (N_g = gw.k*pen_g, no soil scale, no tangential force: casters,
%   structure_gauge_contact).
%
%   init fields (row vectors as the 6DOF block expects them):
%     xme0 [1x3] CG position NED, eul0 [1x3] [phi theta psi] NED->FRD, pose2D0 [3x1] [xE yN psi_enu],
%     v0 (0), Vb0 [1x3] (0), N0 [14x1], pen0 [14x1], sink0 (N-weighted mean penetration), D_R0 (NED down of R),
%     m0, rcg0 [1x3] (FRD w.r.t. R), converged (logical), resid (final residual norm)

ip = inputParser;
ip.addParameter('Pose', []);
ip.addParameter('Soil', []);
ip.addParameter('Mass', []);
ip.parse(varargin{:});
o = ip.Results;

W = P.world;  S = P.structure;  g = P.frames.g;
pose = o.Pose;  if isempty(pose), pose = W.start; end
pose = pose(:)';
m = o.Mass;  if isempty(m), m = S.mp.m0; end
mp = S.mp;
rcg = ((mp.me*mp.rcg_e + (m - mp.me)*mp.r_tank)/m).';     % FRD w.r.t. R (3x1)
RcR = S.contact.Rc_R;                                       % 3 x 14, FRD w.r.t. R
M   = size(RcR, 2);
kc  = S.contact.kc;
Rg  = zeros(3, 0);  kg = 0;                                 % mower gauge wheels (P.structure.contact.gw)
if isfield(S.contact, 'gw') && S.contact.gw.on > 0
    Rg = S.contact.gw.R;  kg = S.contact.gw.k;
end

FH  = griddedInterpolant({W.terrain.xE, W.terrain.yN}, W.terrain.H,    'linear', 'nearest');
FhE = griddedInterpolant({W.terrain.xE, W.terrain.yN}, W.terrain.dHdE, 'linear', 'nearest');
FhN = griddedInterpolant({W.terrain.xE, W.terrain.yN}, W.terrain.dHdN, 'linear', 'nearest');
if isempty(o.Soil)
    FkS = griddedInterpolant({W.terrain.xE, W.terrain.yN}, W.soil.kScale, 'linear', 'nearest');
else
    kSc = W.soilTable(W.soilTable(:,1) == o.Soil, 6);
    FkS = @(e, n) kSc*ones(size(e));
end

psi_ned = atan2(sin(pi/2 - pose(3)), cos(pi/2 - pose(3)));
E0 = pose(1);  N0 = pose(2);
hR = FH(E0, N0);  hE = FhE(E0, N0);  hN = FhN(E0, N0);
gl = atan(hE*cos(pose(3)) + hN*sin(pose(3)));               % grade along heading (+ uphill ahead)
gt = atan(-hE*sin(pose(3)) + hN*cos(pose(3)));              % cross grade (+ left side higher)
kS0 = FkS(E0, N0);
u = [-hR + m*g/(M*kc*kS0); gl; gt];                         % [D_R; theta; phi], theta + = nose up (NED/FRD)

res = @(x) local_res(x, psi_ned, E0, N0, rcg, RcR, m, g, kc, FH, FhE, FhN, FkS, Rg, kg);
r = res(u);
converged = false;
for it = 1:60
    J = zeros(3);
    for k = 1:3
        du = zeros(3,1);  du(k) = 1e-7*max(1, abs(u(k)));
        J(:,k) = (res(u + du) - r)/du(k);
    end
    step = -J\r;
    % damped Newton (keeps the iterate inside the contact region)
    a = 1;
    for ls = 1:20
        rn = res(u + a*step);
        if norm(rn) < norm(r) || a < 1e-3, break, end
        a = a/2;
    end
    u = u + a*step;  r = rn;
    if norm(r) < 1e-6 && norm(a*step) < 1e-12, converged = true; break, end
    if norm(r) < 1e-8, converged = true; break, end
end

[~, Nj, pen] = res(u);
C_nb = local_cbn(u(3), u(2), psi_ned).';
pR = [N0; E0; u(1)];
xme = pR + C_nb*rcg;

init = struct();
init.xme0    = xme(:)';
init.eul0    = [u(3) u(2) psi_ned];
init.pose2D0 = pose(:);
init.v0      = 0;
init.Vb0     = [0 0 0];
init.N0      = Nj(:);
init.pen0    = pen(:);
init.sink0   = sum(Nj(:).*pen(:))/max(sum(Nj), 1);
init.D_R0    = u(1);
init.m0      = m;
init.rcg0    = rcg(:)';
init.converged = converged;
init.resid   = norm(r);
end

% -------------------------------------------------------------------------------------------
function [r, Nj, pen] = local_res(u, psi, E0, N0, rcg, RcR, m, g, kc, FH, FhE, FhN, FkS, Rg, kg)
C_bn = local_cbn(u(3), u(2), psi);  C_nb = C_bn.';
pR = [N0; E0; u(1)];
Pj = pR + C_nb*RcR;                                         % 3 x 14 NED
En = Pj(2,:);  Nn = Pj(1,:);
h  = FH(En, Nn);  hE = FhE(En, Nn);  hN = FhN(En, Nn);  kS = FkS(En, Nn);
nz = 1./sqrt(1 + hE.^2 + hN.^2);
pen = (Pj(3,:) + h).*nz;
Nj = kc*kS.*max(pen, 0);
Gb = m*C_bn*[0; 0; g];                                      % gravity in body (FRD)
nb = C_bn*[-hN.*nz; -hE.*nz; -nz];                          % upward terrain normals in body axes (as MF-B)
Fn = nb.*Nj;                                                % 3 x 14 normal forces
% mower gauge wheels (3 x 0 without the mower): normal spring force only (structure_gauge_contact)
Pg  = pR + C_nb*Rg;
hg  = FH(Pg(2,:), Pg(1,:));  hEg = FhE(Pg(2,:), Pg(1,:));  hNg = FhN(Pg(2,:), Pg(1,:));
nzg = 1./sqrt(1 + hEg.^2 + hNg.^2);
Ng  = kg*max((Pg(3,:) + hg).*nzg, 0);
Fg  = (C_bn*[-hNg.*nzg; -hEg.*nzg; -nzg]).*Ng;              % 3 x G normal forces
Fr = Gb + sum(Fn, 2) + sum(Fg, 2);                          % resultant without shear
sN = sum(Nj);
w  = Nj/max(sN, 1e-9);
Ft = -Fr(1:2)*w;                                            % 2 x 14 shear forces (body x-y), sum = -Fr(1:2)
F  = Fn + [Ft; zeros(1, numel(Nj))];
rj = RcR - rcg;                                             % lever arms w.r.t. CG
Mo = sum(cross(rj, F, 1), 2) + sum(cross(Rg - rcg, Fg, 1), 2);
r  = [Fr(3)/(m*g); Mo(1)/(m*g); Mo(2)/(m*g)];
end

function C = local_cbn(phi, theta, psi)
% NED -> FRD direction cosine matrix, ZYX Euler angles (aerospace convention, = angle2dcm(psi,theta,phi))
cf = cos(phi); sf = sin(phi); ct = cos(theta); st = sin(theta); cp = cos(psi); sp = sin(psi);
Rz = [cp sp 0; -sp cp 0; 0 0 1];
Ry = [ct 0 -st; 0 1 0; st 0 ct];
Rx = [1 0 0; 0 cf sf; 0 -sf cf];
C = Rx*Ry*Rz;
end
