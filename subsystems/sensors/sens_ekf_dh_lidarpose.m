function H = sens_ekf_dh_lidarpose(x, u5)
%SENS_EKF_DH_LIDARPOSE  Jacobian of sens_ekf_h_lidarpose (3 x 11), build task T12.
%   H = sens_ekf_dh_lidarpose(x, u5)   u5 = [tau; psidot_m]
%#codegen
tau = u5(1);
psi = x(4);
cP = cos(psi);
sP = sin(psi);
v = x(5);
vy = x(6);
H = zeros(3, numel(x));
H(1, 1) = 1;
H(1, 4) = tau*(v*sP + vy*cP);
H(1, 5) = -tau*cP;
H(1, 6) = tau*sP;
H(2, 2) = 1;
H(2, 4) = -tau*(v*cP - vy*sP);
H(2, 5) = -tau*sP;
H(2, 6) = -tau*cP;
H(3, 4) = 1;
H(3, 7) = tau;
end
