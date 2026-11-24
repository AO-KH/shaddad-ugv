function z = sens_ekf_h_odom(x, u4)
%SENS_EKF_H_ODOM  EKF channel 4: track (sprocket pitch-line) speeds [vL vR] (sensors spec 3.5, T12).
%   z = sens_ekf_h_odom(x, u4)   u4 (2x1) = [w_m; B]
%   z = [(v - w chi B/2)/(1 - sL); (v + w chi B/2)/(1 - sR)],  w = w_m - bg
%   w_m is the body z rate of the gyro (equal to psidot_m on level ground; the tracks turn about the
%   body z axis, so the body rate is used on slopes), B the track gauge.
%#codegen
w = u4(1) - x(7);
d = w*x(11)*u4(2)/2;
z = [(x(5) - d)/(1 - x(9)); (x(5) + d)/(1 - x(10))];
end
