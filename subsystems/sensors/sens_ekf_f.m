function xNext = sens_ekf_f(x, u)
%SENS_EKF_F  Navigation EKF state transition (docs/specs/sensors.md 3.5, build task T12).
%   xNext = sens_ekf_f(x, u)   one Euler step of the 11-state skid-steer model
%   x = [pE pN pU psi v vy bg ba sL sR chi]'   (ENU / FLU, psi CCW from East, theta + = nose down)
%   u (16x1) = [psidot_m; fx; sinTheta; cosTheta; dt; g; tau_vy; tau_s; s0L; s0R; tau_chi; chi0;
%               sMin; sMax; chiMin; chiMax]
%     psidot_m  level-frame yaw rate (wy sin(phi) + wz cos(phi))/cos(theta) from the gyro [rad/s]
%     fx        body-x physical specific force [m/s^2]
%     tau_s, s0L, s0R  slip Gauss-Markov time constant and prior mean per track; during GNSS + scan-match
%               denial u carries tau_s_denied and the slip estimates held at denial onset (D49).
%               (Revision of the spec u-vector: the single s0 is split into s0L / s0R for the held prior,
%               and the clamps P.sensors.ekf.slip_clamp / chi_clamp are appended.)
%#codegen
w   = u(1) - x(7);                 % bias-corrected yaw rate
fx  = u(2);
sT  = u(3);
cT  = u(4);
dt  = u(5);
g   = u(6);
psi = x(4);
v   = x(5);
vy  = x(6);
cP  = cos(psi);
sP  = sin(psi);
xNext = x;
xNext(1)  = x(1) + dt*(v*cT*cP - vy*sP);
xNext(2)  = x(2) + dt*(v*cT*sP + vy*cP);
xNext(3)  = x(3) - dt*v*sT;
p4 = psi + dt*w;
xNext(4)  = atan2(sin(p4), cos(p4));
xNext(5)  = v + dt*(fx - x(8) + g*sT + w*vy);
xNext(6)  = vy*(1 - dt/u(7));
xNext(7)  = x(7);
xNext(8)  = x(8);
xNext(9)  = x(9)  + dt*(u(9)  - x(9))/u(8);
xNext(10) = x(10) + dt*(u(10) - x(10))/u(8);
xNext(11) = x(11) + dt*(u(12) - x(11))/u(11);
xNext(9)  = min(max(xNext(9),  u(13)), u(14));
xNext(10) = min(max(xNext(10), u(13)), u(14));
xNext(11) = min(max(xNext(11), u(15)), u(16));
end
