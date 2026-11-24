function [rate, up, lo, tau_avail, derating] = esc_torque_limit(tau_lim_mot, tau_lim_reg, w_meas, v_dc, ...
    tau_applied, T_pk, T_cont, exhausted, f_Tm, f_Te, f_uv, f_ov, tlv)
%ESC_TORQUE_LIMIT  Quadrant torque limits of one traction ESC (drive_esc/Torque_Limiter, drive spec 2.9).
%
%   [rate, up, lo, tau_avail, derating] = esc_torque_limit(tau_lim_mot, tau_lim_reg, w_meas, v_dc, ...
%                                         tau_applied, T_pk, T_cont, exhausted, f_Tm, f_Te, f_uv, f_ov, tlv)
%     tau_lim_mot, tau_lim_reg   cmd_in: the motoring and regenerative limits commanded by the MCU (N m)
%     w_meas, v_dc, tau_applied  measured speed (rad/s), DC-link voltage (V), applied torque (N m)
%     T_pk       envelope peak torque: the 2-D Lookup Table EnvPeak (|w| in rpm, v_dc)
%     T_cont     continuous rating: the 1-D Lookup Table EnvCont (|w| in rpm)
%     exhausted  boost exhausted, 1 or 0: the Relay Exhausted on the I2t state x
%     f_Tm, f_Te, f_uv, f_ov     derating factors: the 1-D Lookup Tables of T_mot_meas, T_esc and v_dc
%     tlv        constants (Constant block TlPar): [tau_cont_min, 1/k_boost, 1/t_recover, w_min_dc,
%                I_dc_max_mot, I_dc_max_regen, w_quad, 1 - derate_eps] of P.drive.esc
%     rate       dx/dt of the I2t state, into the Discrete-Time Integrator I2t (x in [0, 1]):
%                (r^2 - 1)/k_boost when r > 1, else (r^2 - 1)/t_recover, r = |tau_applied|/max(T_cont, tau_cont_min)
%     up, lo     upper and lower torque limits (N m): w >= w_quad up = tau_mot, lo = -tau_reg; w <= -w_quad
%                up = tau_reg, lo = -tau_mot; in between +-tau_mot, with
%                tau_mot = min(tau_lim_mot, tau_I2t f_Tm f_Te f_uv, I_dc_max_mot v_dc/max(|w|, w_min_dc)),
%                tau_reg = min(tau_lim_reg, tau_I2t f_Tm f_Te f_ov, I_dc_max_regen v_dc/max(|w|, w_min_dc)),
%                tau_I2t = T_cont when exhausted, else T_pk
%     tau_avail  the ESC's own motoring capability, without the commanded tau_lim_mot (N m)
%     derating   boost exhausted or f_Tm f_Te f_uv < 1 - derate_eps (boolean)
%
%   The MATLAB Function LimitLogic of drive_esc/Torque_Limiter calls it every 1 ms (D70). The Lookup Tables, the
%   I2t integrator and its Relay stay blocks around it, and so do Abs and Gain of the speed in rpm in front of
%   the Lookup Tables (computed here, it would close an algebraic loop through them). The integrator (Forward
%   Euler) has no direct feedthrough, so the loop rate -> I2t -> Exhausted -> exhausted is no algebraic loop. It
%   does what the block network did, operation for operation and in the same order
%   (tests/integration/t_mf_equiv, testDriveEscEquivalent): products left to right as the Product blocks, the
%   Gains as multiplications by the same constants, the Saturations as comparisons that pass NaN as the blocks
%   do, MinMax as local_min (measured, R2026b: it skips NaN as MATLAB's min does, but a tie of +0 and -0 gives the
%   later input, where min gives the first). The comments name the former blocks.
%
%   See also esc_feedback, esc_sm, build_drive.

%#codegen
tau_cont_min = tlv(1);
k_up = tlv(2);
k_dn = tlv(3);
w_min_dc = tlv(4);
I_dcm = tlv(5);
I_dcr = tlv(6);
w_quad = tlv(7);
k_derated = tlv(8);
% I2t charge and recovery rate
tau_abs = abs(tau_applied);                              % AbsTau
cont = T_cont;                                           % ContFloor: Saturation [tau_cont_min, inf]
if cont < tau_cont_min
    cont = tau_cont_min;
end
ratio = tau_abs/cont;                                    % Ratio
sq = ratio*ratio;                                        % Square
m1 = sq + -1;                                            % Minus1 (Bias -1)
if m1 > 0                                                % RateSel (u2 > 0)
    rate = m1*k_up;                                      % RateUp (Gain 1/k_boost)
else
    rate = m1*k_dn;                                      % RateDn (Gain 1/t_recover)
end
% thermal and voltage derating of the envelope
if exhausted > 0.5                                       % TauI2t (u2 > 0.5)
    tau_i2t = T_cont;
else
    tau_i2t = T_pk;
end
kT = f_Tm*f_Te;                                          % kT
tau_th_mot = tau_i2t*kT*f_uv;                            % TauThMot
tau_th_reg = tau_i2t*kT*f_ov;                            % TauThReg
% DC-current limits: tau = I_dc_max v_dc/max(|w|, w_min_dc)
w_fl = abs(w_meas);                                      % AbsW
if w_fl < w_min_dc                                       % WFloor: Saturation [w_min_dc, inf]
    w_fl = w_min_dc;
end
tau_dcm = v_dc*I_dcm/w_fl;                               % Idm, TauDcm
tau_dcr = v_dc*I_dcr/w_fl;                               % Idr, TauDcr
tau_mot = local_min(local_min(tau_lim_mot, tau_th_mot), tau_dcm);   % TauMot
tau_reg = local_min(local_min(tau_lim_reg, tau_th_reg), tau_dcr);   % TauReg
% quadrants
if w_meas*-1 >= w_quad                                   % NegW, UpSel (u2 >= w_quad)
    up = tau_reg;
else
    up = tau_mot;
end
if w_meas >= w_quad                                      % LoSel (u2 >= w_quad)
    lo = tau_reg*-1;                                     % NegReg
else
    lo = tau_mot*-1;                                     % NegMot
end
% tau_avail: the ESC's own capability; min(tau_lim_mot, ...) would deadlock with the MCU, which sets
% tau_lim_mot = min(.., tau_avail)
tau_avail = local_min(tau_th_mot, tau_dcm);              % TauAvail
derating = kT*f_uv < k_derated || exhausted > 0.5;       % kAll, Derated, IsExh, DerOr
end

function y = local_min(a, b)
% the MinMax block (min) on two inputs: b, unless a < b or b is NaN; so NaN is skipped and a tie gives b
if a < b || isnan(b)
    y = a;
else
    y = b;
end
end
