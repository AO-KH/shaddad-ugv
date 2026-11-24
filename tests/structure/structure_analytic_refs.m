function r = structure_analytic_refs(what, P, varargin)
%STRUCTURE_ANALYTIC_REFS  Analytic / semi-analytic references of the STRUCTURE acceptance tests (T03).
%
%   r = structure_analytic_refs(what, P, ...)      P = assembled parameter struct (shaddad_params)
%
%   'static', soil              flat static settle: r.sumN (= W), r.sink (= W/(Nc_tot kc kS), mean penetration),
%                               r.pitch_up (rad, nose up +): alpha = -W x_cg / (kc kS sum(x^2) - W h_cg)
%   'slip', soil, v             steady straight-line slip: f(i L/K) = f_r tanh(v/vEps)/mu_p,
%                               f(x) = 1 - (1 - exp(-x))/x  (lumped Janosi-Hanamoto, spec 2.5a)
%   'accel', T, soil, t1, t2    mean acceleration over [t1 t2] for a torque step T on both sprockets (T5):
%                               a = (2T/r_s - f_r W - 2(T_col + c_visc w)/r_s)/(m + 2(J_eq/r_s^2 + m_belt)),
%                               w at the window mean speed (fixed-point)
%   'climb', theta, soil, v     straight climb on a grade theta (T7): r.Fx (per track), r.slip, r.torque
%                               (= r_s Fx + T_col + c_visc v/r_s), r.N_trk
%   'spot', soil                spot-turn resistance reference mu_lat W L/4 (uniform pressure)
%   'wong_turn', R, v, soil     Wong's general theory of skid-steering on firm ground (Theory of Ground Vehicles,
%                               4th ed., ch. 7): shear displacement integrated along the contact patch of each
%                               track from the slip velocities, Janosi shear stress mu p (1 - exp(-j/K)) opposite
%                               to the local sliding velocity (friction ellipse mu_p / mu_lat as the HiFi model),
%                               rolling resistance, centripetal force and lateral load transfer; solves force and
%                               moment equilibrium for the belt speeds and the lateral velocity of the reference
%                               point. r.chi = (v_bo - v_bi)/(B Omega), r.vb [L R], r.slip [L R], r.Fx [L R],
%                               r.torque [L R] (incl. internal friction), r.P_turn (sprocket power), r.vy, r.ok
%   'slope_frames', theta       static uphill on a grade theta: eul(2) and specForce_B of the rigid body
%                               ([-theta, g sin, 0, g cos]) and with the compliance pitch of the 14 contact springs
%                               (soil from P.world at the start: r.pitch_c, r.eul2_c, r.sf_c)
%
%   Soil properties come from P.world.soilTable (row = soil id: [id mu_p K mu_lat f_r kScale]).
switch lower(what)
    case 'static'
        r = local_static(P, varargin{:});
    case 'slip'
        r = local_slip(P, varargin{:});
    case 'accel'
        r = local_accel(P, varargin{:});
    case 'climb'
        r = local_climb(P, varargin{:});
    case 'spot'
        s = local_soil(P, varargin{1});
        r.M_turn = s.muLat*local_W(P)*P.structure.geom.L/4;
    case 'wong_turn'
        r = local_wong(P, varargin{:});
    case 'slope_frames'
        r = local_slope_frames(P, varargin{:});
    otherwise
        error('structure_analytic_refs:what', 'Unknown reference %s', what);
end
end

% -------------------------------------------------------------------------------------------
function s = local_soil(P, id)
row = P.world.soilTable(P.world.soilTable(:,1) == id, :);
s = struct('muP', row(2), 'K', row(3), 'muLat', row(4), 'fR', row(5), 'kS', row(6));
end

function W = local_W(P)
W = P.structure.mp.m0*P.frames.g;
end

function f = local_f(x)
f = zeros(size(x));
k = x > 1e-9;
f(k) = 1 - (1 - exp(-x(k)))./x(k);
f(~k) = x(~k)/2;
end

function x = local_finv(q)
% inverse of f on [0, 1): bisection
lo = 0; hi = 1e3;
for k = 1:200
    x = 0.5*(lo + hi);
    if local_f(x) < q, lo = x; else, hi = x; end
end
end

function r = local_static(P, soil)
s = local_soil(P, soil);
W = local_W(P);
c = P.structure.contact;
x = c.Rc_R(1, :);
k = c.kc*s.kS;
rcg = P.structure.mp.rcg0;
r.sumN = W;
r.sink = W/(numel(x)*k);
r.pitch_up = -W*rcg(1)/(k*sum(x.^2) - W*(-rcg(3)));
end

function r = local_slip(P, soil, v)
if nargin < 3, v = 1; end
s = local_soil(P, soil);
q = s.fR*tanh(v/P.structure.track.vEpsRR)/s.muP;
r.slip = local_finv(q)*s.K/P.structure.geom.L;
r.Fx_trk = s.fR*tanh(v/P.structure.track.vEpsRR)*local_W(P)/2;
end

function r = local_accel(P, T, soil, t1, t2)
s = local_soil(P, soil);
tr = P.structure.track;  rs = P.structure.r_sprocket;
m = P.structure.mp.m0;  W = local_W(P);
meff = m + 2*(tr.J_eq/rs^2 + tr.m_belt);
a = 1;
for k = 1:50
    w = a*(t1 + t2)/2/rs;
    a = (2*T/rs - s.fR*W - 2*(tr.T_col + tr.c_visc*w)/rs)/meff;
end
r.a = a;  r.meff = meff;  r.a_coulomb_only = (2*T/rs - s.fR*W - 2*tr.T_col/rs)/meff;
end

function r = local_climb(P, th, soil, v)
s = local_soil(P, soil);
tr = P.structure.track;  rs = P.structure.r_sprocket;
W = local_W(P);
r.N_trk = W*cos(th)/2;
r.Fx = (W*sin(th) + s.fR*W*cos(th))/2;
r.slip = local_finv(r.Fx/(s.muP*r.N_trk))*s.K/P.structure.geom.L;
r.torque = rs*r.Fx + tr.T_col + tr.c_visc*v/rs;
end

function r = local_slope_frames(P, th)
g = P.frames.g;
r.eul2 = -th;
r.sf = [g*sin(th) 0 g*cos(th)];
% compliance pitch: overturning moment of the slope-parallel shear at ground level (h_cg) and of the
% CG offset, resisted by the contact springs (soil at the start, P.world grids)
W = local_W(P);
x = P.structure.contact.Rc_R(1, :);
st = P.world.start;
kS = interp2(P.world.terrain.yN, P.world.terrain.xE, P.world.soil.kScale, st(2), st(1), 'linear');
k = P.structure.contact.kc*kS;
rcg = P.structure.mp.rcg0;  h = -rcg(3);
a = (W*sin(th)*h - W*cos(th)*rcg(1))/(k*sum(x.^2) - W*cos(th)*h);
r.pitch_c = a;
r.eul2_c = -(th + a);
r.sf_c = [g*sin(th + a) 0 g*cos(th + a)];
end

% -------------------------------------------------------------------------------------------
function r = local_wong(P, R, v, soil)
% unknowns u = [v_bL; v_bR; V_y] (left turn, Omega = v/R > 0, FLU, reference point R of the chassis)
s = local_soil(P, soil);
Om = v/R;
u = [v - P.structure.track.chi_nom*P.structure.geom.B*Om/2; v + P.structure.track.chi_nom*P.structure.geom.B*Om/2; 0];
f = @(x) local_wong_res(x, P, s, v, Om);
res = f(u);
ok = false;
for it = 1:80
    J = zeros(3);
    for k = 1:3
        du = zeros(3, 1);  du(k) = 1e-6;
        J(:, k) = (f(u + du) - res)/du(k);
    end
    step = -J\res;
    a = 1;
    for ls = 1:30
        rn = f(u + a*step);
        if norm(rn) < norm(res), break, end
        a = a/2;
    end
    u = u + a*step;  res = rn;
    if norm(res) < 1e-6, ok = true; break, end
end
[~, d] = f(u);
tr = P.structure.track;  rs = P.structure.r_sprocket;
r.ok = ok;
r.vb = u(1:2).';
r.vy = u(3);
r.Omega = Om;
r.chi = (u(2) - u(1))/(P.structure.geom.B*Om);
r.vg = d.vg;
r.slip = (u(1:2).' - d.vg)./max(abs(u(1:2).'), 0.05);
r.Fx = d.Fx;
r.torque = rs*d.Fx + sign(u(1:2).').*(tr.T_col + tr.c_visc*abs(u(1:2).')/rs);
r.P_turn = sum(r.torque.*u(1:2).'/rs);
r.resid = norm(res);
end

function [res, d] = local_wong_res(u, P, s, v, Om)
B = P.structure.geom.B;  L = P.structure.geom.L;  m = P.structure.mp.m0;  g = P.frames.g;  W = m*g;
rcg = P.structure.mp.rcg0 .* [1 -1 -1];                 % FLU w.r.t. R: [x y z]
xc = rcg(1);  h = rcg(3);
nx = 400;
x = L/2 - ((1:nx) - 0.5)*L/nx;                          % patch coordinate (front -> rear), body FLU
dx = L/nx;
vb = u(1:2);  Vy = u(3);  Vx = v;                       % velocities of R (FLU)
% CG kinematics and inertial (centripetal) force at the CG: m a = m [-Om V_y,cg; Om V_x,cg]
Vxc = Vx;  Vyc = Vy + Om*xc;
Fin = m*[-Om*Vyc; Om*Vxc];
% lateral load transfer (outer = right track for a left turn): +/- m a_y h / B
dN = Fin(2)*h/B;
Ntr = [W/2 - dN; W/2 + dN];
yv = [B/2; -B/2];                                       % track centrelines (FLU): left +, right -
Fx = zeros(1, 2);  Fy = 0;  Mz = 0;  vg = zeros(1, 2);
for i = 1:2
    y = yv(i);
    p = Ntr(i)/L*(1 + 12*xc*x/L^2);                     % pressure per unit length, centre of pressure under the CG
    vsx = (Vx - Om*y) - vb(i);                          % sliding velocity of the track element w.r.t. ground
    vsy = Vy + Om*x;
    vbi = max(abs(vb(i)), 1e-3);
    jx = vsx*(L/2 - x)/vbi;                             % shear displacement accumulated from the front
    jy = (Vy*(L/2 - x) + Om*(L^2/4 - x.^2)/2)/vbi;
    jn = sqrt(jx.^2 + jy.^2);
    vsn = sqrt(vsx^2 + vsy.^2) + 1e-12;
    cx = -vsx./vsn;  cy = -vsy./vsn;                    % shear stress on the vehicle opposite to the sliding
    mu = s.muP*s.muLat./sqrt((s.muLat*cx).^2 + (s.muP*cy).^2);
    tau = mu.*p.*(1 - exp(-jn/s.K));
    dFx = tau.*cx*dx;  dFy = tau.*cy*dx;
    vgx = Vx - Om*y;                                    % ground speed of the track centreline along x
    Frr = -s.fR*Ntr(i)*tanh(vgx/P.structure.track.vEpsRR);
    Fx(i) = sum(dFx);
    Fy = Fy + sum(dFy);
    Mz = Mz + sum(x.*dFy - y*dFx) - y*Frr;
    vg(i) = vgx;
    Fxr(i) = Frr; %#ok<AGROW>
end
FxT = sum(Fx) + sum(Fxr);
% equilibrium in the rotating body frame: forces = m a (at the CG), moment about R of the inertial force
res = [(FxT - Fin(1))/W; (Fy - Fin(2))/W; (Mz - (xc*Fin(2) - rcg(2)*Fin(1)))/(W*L)];
d.vg = vg;  d.Fx = Fx;
end
