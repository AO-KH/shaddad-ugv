function [z, b] = sens_ekf_h_heading(x, u3)
%SENS_EKF_H_HEADING  EKF channel 3: heading (dual antenna or ecompass), wrapped (sensors spec 3.5, T12).
%   [z, b] = sens_ekf_h_heading(x, u3)   u3 (2x1) = [tau; psidot_m],  b = [-pi pi]
%   z = psi - tau*(psidot_m - bg): latency back-extrapolation of the 0.08 s GNSS epoch (tau = 0 for the
%   ecompass). (Revision of the spec signature h(x): the latency term keeps the dual-antenna heading
%   unbiased in turns; with tau = 0 it is the spec model.) The EKF block wraps the residual to b.
%#codegen
z = x(4) - u3(1)*(u3(2) - x(7));
b = [-pi pi];
end
