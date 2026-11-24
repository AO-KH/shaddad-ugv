function H = sens_ekf_dh_gnssvel(x, u2)
%SENS_EKF_DH_GNSSVEL  Jacobian of sens_ekf_h_gnssvel (2 x 11), build task T12.
%   H = sens_ekf_dh_gnssvel(x, u2)   u2 = [r_a(3); phi; theta; tau; psidot_m]
%#codegen
[~, va, ~, dva] = sens_ekf_antenna(x, u2);
tau = u2(6);
a = (u2(7) - x(7))*tau;
da = zeros(1, 11);
da(7) = -tau;
H = [dva(1, :) + a*dva(2, :) + va(2)*da; dva(2, :) - a*dva(1, :) - va(1)*da];
end
