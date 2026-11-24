function H = sens_ekf_dh_heading(x, u3)
%SENS_EKF_DH_HEADING  Jacobian of sens_ekf_h_heading (1 x 11), build task T12.
%   H = sens_ekf_dh_heading(x, u3)   u3 = [tau; psidot_m]
%#codegen
H = zeros(1, numel(x));
H(4) = 1;
H(7) = u3(1);
end
