function [F_c, M_c, sDot, Fx_pt, Fy_trk, M_turn, P_diss, chi_eff, s_trk] = structure_planar_forces(N_pt, Vb_pt, s, vB, muP, muLat, fR, K, Rcg, omega, hE, hN, DCM_be, prm)
%STRUCTURE_PLANAR_FORCES  MF-B trackPlanarForces: per-point ISOTROPIC 2-D Dahl/Janosi shear (revision B, D39),
%   rolling resistance and the contact wrench (docs/specs/structure.md 2.5 / 2.6 / 4.2; owner structure, T03).
%#codegen
%
%   Inputs  N_pt [M x 1] normal forces (MF-A), Vb_pt [3 x M] point velocities in body axes (FRD),
%           s [2M x 1] shear states [s_x(1:M); s_y(1:M)] (Integrator, structure_track_shear),
%           vB [2 x 1] belt speeds [L; R] DELAYED by one 1 ms sample (D36 loop breaker),
%           muP, muLat, fR, K [1 x M] soil properties at the points, Rcg [3 x M] lever arms w.r.t. the CG (FRD),
%           omega [3 x 1] body rates (FRD), hE, hN [1 x M] terrain gradients at the points, DCM_be [3 x 3] NED->body,
%           prm = struct with the P.structure.track fields ell, tau_d, vEpsRR, gS, gG, B, chi_rmin (Nc = numel(N_pt)/2)
%   Per point j of track i:
%     v_slip = [v_b,i - v_x,j; -v_y,j]                       (belt minus ground)
%     ds/dt  = (v_slip - |v_slip| s)/K_j - (|v_b,i|/ell) s     (2-vector, |s| <= 1)
%     mu_j   = mu_p mu_lat |s| / sqrt((mu_lat s_x)^2 + (mu_p s_y)^2)   (friction ellipse along s; mu_p for |s| < 1e-6)
%     F_j    = sat_|.|( mu_j N_j [g(|s|) s/|s| + tau_d ds/dt], mu_j N_j ),  g = table prm.gS / prm.gG
%     F_rr,j = -f_r,j N_j tanh(v_x,j / vEpsRR)                (rolling resistance, body only)
%     F_c,j  = [F_x,j + F_rr,j; F_y,j; 0] + N_j n_j  applied at r_j, n_j = DCM_be [-hN; -hE; -1]/sqrt(1+hE^2+hN^2)
%              (normal force along the local terrain normal, expressed in body axes; T03 refinement of spec 2.6,
%              which applied it along -z_b: removes the spurious N sin(pitch) force of the static compliance pitch)
%   Outputs F_c, M_c [3 x 1] contact wrench about the CG (FRD), sDot [2M x 1], Fx_pt [M x 1] shear traction per
%           point (sum per track = sprocket belt load), Fy_trk [2 x 1] lateral shear force per track (FRD),
%           M_turn lateral-shear yaw moment sum(x_j F_y,j) (FRD z), P_diss = sum_i F_x,i v_b,i - sum_j F_c,j . v_j
%           (contact dissipation incl. shear slip, rolling resistance and normal damping), chi_eff =
%           (v_bR - v_bL)/(B r_FLU) (NaN for |r| < chi_rmin), s_trk [2 x 1] N-weighted mean s_x per track
M  = numel(N_pt);              % 2*Nc points (compile-time constant from the input dimension)
Nc = M/2;
F_pt = zeros(3, M);
sDot = zeros(2*M, 1);
Fx_pt = zeros(M, 1);
Fy_pt = zeros(M, 1);
gS = prm.gS;  gG = prm.gG;
for j = 1:M
    if j <= Nc, i = 1; else, i = 2; end
    vb  = vB(i);
    vx  = Vb_pt(1, j);  vy = Vb_pt(2, j);
    vsx = vb - vx;      vsy = -vy;
    vsn = sqrt(vsx*vsx + vsy*vsy);
    sx  = s(j);         sy = s(M + j);
    Kj  = max(K(j), 1e-6);
    rl  = abs(vb)/prm.ell;
    dsx = (vsx - vsn*sx)/Kj - rl*sx;
    dsy = (vsy - vsn*sy)/Kj - rl*sy;
    sDot(j) = dsx;  sDot(M + j) = dsy;
    sn  = sqrt(sx*sx + sy*sy);
    if sn < 1e-6
        mu = muP(j);  gx = 0;  gy = 0;
    else
        mu = muP(j)*muLat(j)*sn/sqrt((muLat(j)*sx)^2 + (muP(j)*sy)^2);
        gv = local_g(min(sn, 1), gS, gG);
        gx = gv*sx/sn;  gy = gv*sy/sn;
    end
    cap = mu*N_pt(j);
    fx  = cap*(gx + prm.tau_d*dsx);
    fy  = cap*(gy + prm.tau_d*dsy);
    fm  = sqrt(fx*fx + fy*fy);
    if fm > cap && fm > 0
        fx = fx*cap/fm;  fy = fy*cap/fm;
    end
    frr = -fR(j)*N_pt(j)*tanh(vx/prm.vEpsRR);
    nz  = 1/sqrt(1 + hE(j)^2 + hN(j)^2);
    nN  = -hN(j)*nz;  nE = -hE(j)*nz;  nD = -nz;            % upward terrain normal (NED)
    Fx_pt(j) = fx;  Fy_pt(j) = fy;
    F_pt(1, j) = fx + frr + N_pt(j)*(DCM_be(1,1)*nN + DCM_be(1,2)*nE + DCM_be(1,3)*nD);
    F_pt(2, j) = fy       + N_pt(j)*(DCM_be(2,1)*nN + DCM_be(2,2)*nE + DCM_be(2,3)*nD);
    F_pt(3, j) =            N_pt(j)*(DCM_be(3,1)*nN + DCM_be(3,2)*nE + DCM_be(3,3)*nD);
end
F_c = zeros(3, 1);  M_c = zeros(3, 1);
P_body = 0;  M_turn = 0;
Fy_trk = zeros(2, 1);  Fx_trk = zeros(2, 1);
for j = 1:M
    if j <= Nc, i = 1; else, i = 2; end
    f = F_pt(:, j);  r = Rcg(:, j);
    F_c = F_c + f;
    M_c = M_c + [r(2)*f(3) - r(3)*f(2); r(3)*f(1) - r(1)*f(3); r(1)*f(2) - r(2)*f(1)];
    P_body = P_body + f(1)*Vb_pt(1, j) + f(2)*Vb_pt(2, j) + f(3)*Vb_pt(3, j);
    M_turn = M_turn + r(1)*Fy_pt(j);
    Fy_trk(i) = Fy_trk(i) + Fy_pt(j);
    Fx_trk(i) = Fx_trk(i) + Fx_pt(j);
end
P_diss = Fx_trk(1)*vB(1) + Fx_trk(2)*vB(2) - P_body;
rF = -omega(3);                                  % yaw rate FLU (+ = left turn)
if abs(rF) < prm.chi_rmin
    chi_eff = NaN;
else
    chi_eff = (vB(2) - vB(1))/(prm.B*rF);
end
s_trk = zeros(2, 1);
for i = 1:2
    i0 = (i - 1)*Nc;
    sN = 0;  a = 0;  b = 0;
    for k = 1:Nc
        sN = sN + N_pt(i0 + k);
        a  = a + N_pt(i0 + k)*s(i0 + k);
        b  = b + s(i0 + k)/Nc;
    end
    if sN < 1
        s_trk(i) = b;
    else
        s_trk(i) = a/sN;
    end
end
end

% -------------------------------------------------------------------------------------------
function y = local_g(x, gS, gG)
% codegen-safe linear interpolation of the lumped Janosi table g(s) (same data as P.structure.track.gS/gG)
n = numel(gS);
if x <= gS(1)
    y = gG(1);
elseif x >= gS(n)
    y = gG(n);
else
    k = 1;
    while k < n - 1 && gS(k + 1) < x
        k = k + 1;
    end
    y = gG(k) + (gG(k + 1) - gG(k))*(x - gS(k))/(gS(k + 1) - gS(k));
end
end
