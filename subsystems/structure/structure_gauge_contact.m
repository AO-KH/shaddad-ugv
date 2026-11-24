function [F_g, M_g, P_g, N_g] = structure_gauge_contact(Pn, Vn, r, h, hE, hN, DCM_be, prm)
%STRUCTURE_GAUGE_CONTACT  MF-C2 gaugeWheelContact: compliant normal contact of the mower gauge wheels
%   (rigid deck on caster gauge wheels that touch the ground, the user 2026-11-15; owner structure, build task T03).
%#codegen
%
%   Inputs  Pn, Vn [3 x G] wheel contact-point positions / velocities in NED (MF-C1 gaugeWheelPoints),
%           r [3 x G] lever arms w.r.t. the CG (FRD), h, hE, hN [1 x G] terrain height (up) and gradients dH/dE,
%           dH/dN at the points (2-D Lookup Tables), DCM_be [3 x 3] NED -> body,
%           prm = [on; k; c] of P.structure.contact.gw (on = 1 with the mower fitted, else 0; k, c per wheel)
%   Outputs F_g, M_g [3] contact wrench about the CG (FRD), P_g = -sum_g F_g . v_g (power the wheels take from the
%           body, added to the contact dissipation P_diss of Bus_StructureDiag, D40), N_g [G] normal loads (>= 0)
%
%   Per wheel (the normal law of the track points, MF-A, without the soil stiffness scale):
%     pen = (D + h) n_z,  pen' = (vD + hE vE + hN vN) n_z,  n_z = 1/sqrt(1 + hE^2 + hN^2)
%     N   = on [pen > 0] max(0, k pen + c pen'),  F = N n  along the upward terrain normal n (as MF-B)
%   No tangential force: the wheels are casters; their rolling drag stays in the mower wrench (structure_impl_mower).
G   = size(Pn, 2);
F_g = zeros(3, 1);  M_g = zeros(3, 1);  P_g = 0;  N_g = zeros(G, 1);
for k = 1:G
    nz   = 1/sqrt(1 + hE(k)^2 + hN(k)^2);
    pen  = (Pn(3, k) + h(k))*nz;
    penD = (Vn(3, k) + hE(k)*Vn(2, k) + hN(k)*Vn(1, k))*nz;
    Nk = 0;
    if pen > 0
        Nk = prm(1)*max(0, prm(2)*pen + prm(3)*penD);
    end
    N_g(k) = Nk;
    nN = -hN(k)*nz;  nE = -hE(k)*nz;  nD = -nz;            % upward terrain normal (NED)
    f  = Nk*[DCM_be(1,1)*nN + DCM_be(1,2)*nE + DCM_be(1,3)*nD; ...   % body axes (FRD), as MF-B
             DCM_be(2,1)*nN + DCM_be(2,2)*nE + DCM_be(2,3)*nD; ...
             DCM_be(3,1)*nN + DCM_be(3,2)*nE + DCM_be(3,3)*nD];
    rk = r(:, k);
    F_g = F_g + f;
    M_g = M_g + [rk(2)*f(3) - rk(3)*f(2); rk(3)*f(1) - rk(1)*f(3); rk(1)*f(2) - rk(2)*f(1)];
    P_g = P_g - Nk*(nN*Vn(1, k) + nE*Vn(2, k) + nD*Vn(3, k));
end
end
