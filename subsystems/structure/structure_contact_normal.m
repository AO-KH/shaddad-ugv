function [N_pt, N_trk, muP_trk, K_trk, vG_trk, pen_pt, sink_trk] = structure_contact_normal(Pn, Vn, Vb_pt, h, hE, hN, kS, muP, K, prm)
%STRUCTURE_CONTACT_NORMAL  MF-A trackContactNormal: compliant normal contact of the 2 x Nc track points
%   (docs/specs/structure.md 2.4 / 4.1; owner structure, build task T03).
%#codegen
%
%   Inputs  Pn, Vn [3 x M] contact-point positions / velocities in NED (M = 2*prm.Nc, 1..Nc left, Nc+1..M right)
%           Vb_pt  [3 x M] contact-point velocities in body axes (FRD)
%           h, hE, hN, kS, muP, K [1 x M] terrain height (up), gradients dH/dE, dH/dN, stiffness scale,
%                  peak traction coefficient and shear modulus at the points (2-D Lookup Tables)
%           prm    struct with the P.structure.contact fields kc, cc (Nc = numel(h)/2 from the inputs)
%   Outputs N_pt [M x 1] normal force (>= 0), N_trk [2 x 1], muP_trk / K_trk [2 x 1] N-weighted means
%           (plain means while N_trk < 1 N), vG_trk [2 x 1] mean body-x ground speed of the track points,
%           pen_pt [M x 1] penetration (> 0 below the surface), sink_trk [2 x 1] N-weighted mean penetration
%
%   pen_j  = (D_j + h_j) n_z,  pen'_j = (vD_j + hE vE_j + hN vN_j) n_z,  n_z = 1/sqrt(1 + hE^2 + hN^2)
%   N_j    = [pen_j > 0] max(0, kc kS_j pen_j + cc sqrt(kS_j) pen'_j)
M  = numel(h);                 % 2*Nc points (compile-time constant from the input dimension)
Nc = M/2;
N_pt = zeros(M, 1);  pen_pt = zeros(M, 1);
N_trk = zeros(2, 1); muP_trk = zeros(2, 1); K_trk = zeros(2, 1); vG_trk = zeros(2, 1); sink_trk = zeros(2, 1);
for j = 1:M
    nz   = 1/sqrt(1 + hE(j)^2 + hN(j)^2);
    pen  = (Pn(3,j) + h(j))*nz;
    penD = (Vn(3,j) + hE(j)*Vn(2,j) + hN(j)*Vn(1,j))*nz;
    pen_pt(j) = pen;
    if pen > 0
        N_pt(j) = max(0, prm.kc*kS(j)*pen + prm.cc*sqrt(kS(j))*penD);
    end
end
for i = 1:2
    i0 = (i - 1)*Nc;
    sN = 0;
    for k = 1:Nc
        sN = sN + N_pt(i0 + k);
    end
    N_trk(i) = sN;
    a = 0; b = 0; c = 0; v = 0;
    for k = 1:Nc
        if sN < 1
            w = 1/Nc;
        else
            w = N_pt(i0 + k)/sN;
        end
        a = a + w*muP(i0 + k);
        b = b + w*K(i0 + k);
        c = c + w*pen_pt(i0 + k);
        v = v + Vb_pt(1, i0 + k)/Nc;
    end
    muP_trk(i) = a;  K_trk(i) = b;  sink_trk(i) = c;  vG_trk(i) = v;
end
end
