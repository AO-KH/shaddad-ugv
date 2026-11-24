function H = sens_ekf_dh_odom(x, u4)
%SENS_EKF_DH_ODOM  Jacobian of sens_ekf_h_odom (2 x 11), build task T12.
%   H = sens_ekf_dh_odom(x, u4)   u4 = [w_m; B]
%#codegen
w  = u4(1) - x(7);
hb = u4(2)/2;
d  = w*x(11)*hb;
eL = 1/(1 - x(9));
eR = 1/(1 - x(10));
H = zeros(2, numel(x));
H(1, 5)  = eL;
H(1, 7)  = x(11)*hb*eL;
H(1, 9)  = (x(5) - d)*eL^2;
H(1, 11) = -w*hb*eL;
H(2, 5)  = eR;
H(2, 7)  = -x(11)*hb*eR;
H(2, 10) = (x(5) + d)*eR^2;
H(2, 11) = w*hb*eR;
end
