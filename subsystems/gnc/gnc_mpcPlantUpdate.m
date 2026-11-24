function mdl = gnc_mpcPlantUpdate(v, prm) %#codegen
%GNC_MPCPLANTUPDATE  LTV prediction model (Bus_MpcModel) of the row-tracking Adaptive MPC.
%
%   mdl = gnc_mpcPlantUpdate(v, prm)
%     v    measured forward speed [m/s] (Bus_NavCond.v)
%     prm  P.gnc.mpc (non-tunable sub-struct): Ts, tau_r, v_min_model, n_delay (= 2)
%     mdl  struct Plant (A 5x5, B 5x5, C 2x5, D 2x5), Nominal (X 5, U 5, Y 2, DX 5)
%
%   Why a MATLAB Function (docs/specs/gnc.md 4.3): the Adaptive MPC 'model' inport needs the plant
%   discretised at the current speed every step; no library block builds it.
%
%   Frenet error model (spec 2.3), x = [e_y; e_psi; r], inputs
%   u = [r_cmd (MV); kappa_ref (MD); d (UD, lateral drift); w_y (UD); w_psi (UD)]:
%       e_y_dot   = v e_psi + d
%       e_psi_dot = r - v kappa_ref
%       r_dot     = (r_cmd - r)/tau_r
%   discretised by ZOH (expm of the augmented matrix) at Ts; the MV is delayed by n_delay = 2 samples
%   (GNC -> ESC and MCU AUTO shaper dead time, revision B D41):
%       x+  = Ad x + Bd_mv z2 + Bd_md kappa + Bd_d d + [w_y; w_psi; 0],   z1+ = r_cmd,   z2+ = z1
%   w_y / w_psi are direct (white) jump channels of e_y / e_psi: with them the default Kalman estimator
%   attributes a path-error step (row entry, new window) to the state instead of to the drift random
%   walk, which otherwise overshoots by 0.2 m from a 0.3 m entry offset (T14 tuning, see gnc_designMPC).
%   States [e_y e_psi r z1 z2] (z1 = r_cmd(k-1), z2 = r_cmd(k-2)), outputs y = [e_y; e_psi].
%   gnc_designMPC builds mpcRowObj from this same function, so the object and the inport agree.

nd = 2;                                      % Bus_MpcPlant is sized for n_delay = 2 (P.gnc.mpc.nx = 5)
assert(prm.n_delay == nd);
vv  = max(abs(v), prm.v_min_model);
tau = prm.tau_r;
Ts  = prm.Ts;
Ac = [0 vv 0; 0 0 1; 0 0 -1/tau];
Bc = [0 0 1; 0 -vv 0; 1/tau 0 0];
M  = expm([Ac Bc; zeros(3, 6)]*Ts);
Ad = M(1:3, 1:3);
Bd = M(1:3, 4:6);

A = zeros(5, 5);
A(1:3, 1:3) = Ad;
A(1:3, 5)   = Bd(:, 1);                      % r_cmd acts through z2 = r_cmd(k-2)
A(5, 4)     = 1;                             % z2+ = z1
B = zeros(5, 5);
B(4, 1)     = 1;                             % z1+ = r_cmd (MV)
B(1:3, 2:3) = Bd(:, 2:3);                    % kappa_ref (MD), drift d (UD) undelayed
B(1, 4)     = 1;                             % w_y  : e_y jump (UD)
B(2, 5)     = 1;                             % w_psi: e_psi jump (UD)
C = [1 0 0 0 0; 0 1 0 0 0];
D = zeros(2, 5);

mdl.Plant   = struct('A', A, 'B', B, 'C', C, 'D', D);
mdl.Nominal = struct('X', zeros(5, 1), 'U', zeros(5, 1), 'Y', zeros(2, 1), 'DX', zeros(5, 1));
end
