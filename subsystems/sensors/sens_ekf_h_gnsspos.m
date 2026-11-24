function z = sens_ekf_h_gnsspos(x, u1)
%SENS_EKF_H_GNSSPOS  EKF channel 1: GNSS primary-antenna position [E N U] (sensors spec 3.5, T12).
%   z = sens_ekf_h_gnsspos(x, u1)   u1 (7x1) = [r_a(3); phi; theta; tau; psidot_m]
%   z = p + l - tau*v_a,  l = R_WB(psi,theta,phi) r_a,  v_a = [pdotE - w l_y; pdotN + w l_x; pdotU],
%   w = psidot_m - bg (latency back-extrapolation; tau = t - t_meas of the epoch).
%#codegen
[l, va] = sens_ekf_antenna(x, u1);
z = x(1:3) + l - u1(6)*va;
end
