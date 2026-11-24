function F = sens_ekf_dfdx(x, u)
%SENS_EKF_DFDX  Jacobian of sens_ekf_f, F = I + dt*A (docs/specs/sensors.md 3.5, build task T12).
%   F = sens_ekf_dfdx(x, u)   x, u as in sens_ekf_f (u 16x1); the clamps of sens_ekf_f are not
%   differentiated (inactive inside the clamp range)
%#codegen
w   = u(1) - x(7);
sT  = u(3);
cT  = u(4);
dt  = u(5);
psi = x(4);
v   = x(5);
vy  = x(6);
cP  = cos(psi);
sP  = sin(psi);
A = zeros(11, 11);
A(1, 4) = -(v*cT*sP + vy*cP);
A(1, 5) = cT*cP;
A(1, 6) = -sP;
A(2, 4) = v*cT*cP - vy*sP;
A(2, 5) = cT*sP;
A(2, 6) = cP;
A(3, 5) = -sT;
A(4, 7) = -1;
A(5, 6) = w;
A(5, 7) = -vy;
A(5, 8) = -1;
A(6, 6) = -1/u(7);
A(9, 9) = -1/u(8);
A(10, 10) = -1/u(8);
A(11, 11) = -1/u(11);
F = eye(11) + dt*A;
end
