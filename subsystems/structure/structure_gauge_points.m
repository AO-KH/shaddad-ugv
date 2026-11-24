function [Pn, Vn, r] = structure_gauge_points(Xe, DCM_be, Vb, omega, rcg, Rg)
%STRUCTURE_GAUGE_POINTS  MF-C1 gaugeWheelPoints: positions and velocities of the mower gauge-wheel contact points
%   (rigid deck on caster gauge wheels that touch the ground, the user 2026-11-15; owner structure, build task T03).
%#codegen
%
%   Inputs  Xe [3] CG position NED, DCM_be [3 x 3] NED -> body, Vb [3] CG velocity (FRD), omega [3] body rates
%           (FRD), rcg [3] CG w.r.t. R (FRD), Rg [3 x G] wheel contact points w.r.t. R (FRD, P.structure.contact.gw.R)
%   Outputs Pn, Vn [3 x G] point positions / velocities in NED, r [3 x G] lever arms w.r.t. the CG (FRD)
%
%   r_g = Rg_g - rcg,  Pn_g = Xe + C_nb r_g,  Vn_g = C_nb (Vb + omega x r_g),  C_nb = DCM_be'
%   (as the track points of structure_contact)
G  = size(Rg, 2);
Pn = zeros(3, G);  Vn = zeros(3, G);  r = zeros(3, G);
C_nb = DCM_be.';
for k = 1:G
    rk = [Rg(1, k) - rcg(1); Rg(2, k) - rcg(2); Rg(3, k) - rcg(3)];
    vb = [Vb(1) + omega(2)*rk(3) - omega(3)*rk(2); ...
          Vb(2) + omega(3)*rk(1) - omega(1)*rk(3); ...
          Vb(3) + omega(1)*rk(2) - omega(2)*rk(1)];
    r(:, k)  = rk;
    Pn(:, k) = [Xe(1); Xe(2); Xe(3)] + C_nb*rk;
    Vn(:, k) = C_nb*vb;
end
end
