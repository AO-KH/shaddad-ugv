function [z, b] = sens_ekf_h_lidarpose(x, u5)
%SENS_EKF_H_LIDARPOSE  EKF channel 5: LiDAR scan-to-map pose of base_link [E N psi] (sensors spec 3.5, T12).
%   [z, b] = sens_ekf_h_lidarpose(x, u5)   u5 (2x1) = [tau; psidot_m], b = [-Inf Inf; -Inf Inf; -pi pi]
%   z = [pE - tau (v cos psi - vy sin psi); pN - tau (v sin psi + vy cos psi); psi - tau w], w = psidot_m - bg:
%   back-extrapolation (level-frame velocity) to the scan time, tau = t - Bus_ScanMatchMeas.t (0.2 s match
%   period plus the slow-to-fast rate transition). (Revision of the spec h(x) = [pE; pN; psi], which it equals
%   for tau = 0.) The EKF wraps the yaw residual to b.
%#codegen
tau = u5(1);
w = u5(2) - x(7);
psi = x(4);
cP = cos(psi);
sP = sin(psi);
z = [x(1) - tau*(x(5)*cP - x(6)*sP); ...
     x(2) - tau*(x(5)*sP + x(6)*cP); ...
     psi - tau*w];
b = [-Inf Inf; -Inf Inf; -pi pi];
end
