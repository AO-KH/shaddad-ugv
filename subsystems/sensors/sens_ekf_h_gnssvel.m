function z = sens_ekf_h_gnssvel(x, u2)
%SENS_EKF_H_GNSSVEL  EKF channel 2: GNSS antenna velocity [vE vN] (sensors spec 3.5, build task T12).
%   z = sens_ekf_h_gnssvel(x, u2)   u2 (7x1) = [r_a(3); phi; theta; tau; psidot_m]
%   z = va(1:2) rotated back by the latency: va(t - tau) ~ Rz(-w tau) va (constant-rate turn), w = psidot_m - bg.
%   (Revision of the spec u2 = [r_a; phi; theta; psidot_m]: tau added so the 0.08 s epoch latency is
%   compensated in turns as in channel 1; with tau = 0 it is the spec model.)
%#codegen
[~, va] = sens_ekf_antenna(x, u2);
a = (u2(7) - x(7))*u2(6);
z = [va(1) + a*va(2); va(2) - a*va(1)];
end
