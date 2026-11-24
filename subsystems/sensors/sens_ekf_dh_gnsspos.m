function H = sens_ekf_dh_gnsspos(x, u1)
%SENS_EKF_DH_GNSSPOS  Jacobian of sens_ekf_h_gnsspos (3 x 11), build task T12.
%   H = sens_ekf_dh_gnsspos(x, u1)   u1 = [r_a(3); phi; theta; tau; psidot_m]
%#codegen
[~, ~, dl_dpsi, dva] = sens_ekf_antenna(x, u1);
tau = u1(6);
H = -tau*dva;
H(1:3, 1:3) = eye(3);
H(:, 4) = dl_dpsi - tau*dva(:, 4);
end
