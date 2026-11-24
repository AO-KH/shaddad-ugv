function [l, va, dl_dpsi, dva] = sens_ekf_antenna(x, u)
%SENS_EKF_ANTENNA  GNSS antenna lever arm and velocity used by the EKF channels 1 and 2 (build task T12).
%   [l, va, dl_dpsi, dva] = sens_ekf_antenna(x, u)   u = [r_a(3); phi; theta; tau; psidot_m]
%   l       = R_WB(psi, theta, phi) r_a  (ZYX, R = Rz(psi) Ry(theta) Rx(phi), theta + = nose down)
%   va      = [pdotE - w l_y; pdotN + w l_x; pdotU],  w = psidot_m - bg
%   dl_dpsi = d l / d psi
%   dva     = d va / d x (3 x 11)
%#codegen
ra  = u(1:3);
phi = u(4);
th  = u(5);
w   = u(7) - x(7);
psi = x(4);
v   = x(5);
vy  = x(6);
cP = cos(psi); sP = sin(psi);
cT = cos(th);  sT = sin(th);
cF = cos(phi); sF = sin(phi);
Rz = [cP -sP 0; sP cP 0; 0 0 1];
Ry = [cT 0 sT; 0 1 0; -sT 0 cT];
Rx = [1 0 0; 0 cF -sF; 0 sF cF];
l = Rz*(Ry*(Rx*ra));
pdot = [v*cT*cP - vy*sP; v*cT*sP + vy*cP; -v*sT];
va = pdot + [-w*l(2); w*l(1); 0];
dl_dpsi = [-l(2); l(1); 0];
dva = zeros(3, 11);
% d va / d psi
dva(:, 4) = [-(v*cT*sP + vy*cP) - w*l(1); (v*cT*cP - vy*sP) - w*l(2); 0];
% d va / d v, d vy
dva(:, 5) = [cT*cP; cT*sP; -sT];
dva(:, 6) = [-sP; cP; 0];
% d va / d bg (w = psidot_m - bg)
dva(:, 7) = [l(2); -l(1); 0];
end
