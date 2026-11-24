function v = mcu_cl_track_dyn(v_cmd, en, tlim, pl, prm)
%MCU_CL_TRACK_DYN  Physical track speeds of the bench tb_mcu_closed_loop (plant(8) mass > 0; descent task, 2026-11-15).
%#codegen
%
%   v = mcu_cl_track_dyn(v_cmd, en, tlim, pl, prm)   one step at Ts.base (MATLAB Function Plant/Track_dyn)
%   v_cmd  double[2]  commanded track speeds w_ref/k_v2w (0 while the ESC is disabled), decoded DRV_CMD
%   en     boolean[2] ESC enables (decoded DRV_CMD)
%   tlim   double[4]  [tau_lim_mot_L tau_lim_mot_R tau_lim_reg_L tau_lim_reg_R] (N m at the motor, decoded DRV_CMD)
%   pl     double[9]  the bench plant input [yaw_gain pitch roll ground_ratio a_run brake_fail impl m f_rr]
%   prm    double[4]  [Ts tau g k_v2w]
%   Each track carries m/2. ESC enabled: dv/dt = a_esc + a_load with a_load = g sin(pitch) - f_rr g cos(pitch) sign(v)
%   (pitch + nose down: the slope pushes forward) and a_esc = sat((v_cmd - v)/tau - a_load, a_lo, a_hi), the speed loop
%   of the first-order plant (time constant tau) whose motor force tau*k_v2w is limited by the torque limits: moving
%   forward up to +tau_lim_mot and down to -tau_lim_reg (regen, limited by the BMS CCL), mirrored in reverse. ESC
%   disabled: an ideal brake, dv/dt = -v/tau. Euler at Ts; the output is the state before this step's update (the block
%   has no direct feedthrough). The runaway load a_run acts only in the first-order plant.
%   plant(6) brake_fail b2 (descent task, step 9, 2026-11-16; b0 / b1 are the brake faults of ESC_Fb): the ESCs take
%   over without a hold-torque preload. The load term -a_load of the speed loop is then a state ff that starts at 0 at
%   each enable and follows -a_load with the time constant tau_take [EST 0.15 s: with tau 0.05 s about the 0.09 m/s
%   roll-back of the HiFi S2 hill start, 20 deg climb, full-tank sprayer]; 0 while disabled. b2 clear: as before

persistent x ff
if isempty(x)
    x = [0 0];
    ff = [0 0];
end
v = x;
Ts = prm(1); tau = prm(2); g = prm(3); kv = prm(4);
pitch = pl(2); ms = max(pl(8), 1)/2; frr = pl(9);
take = bitand(uint8(pl(6)), uint8(4)) ~= 0;      % b2: no hold-torque preload at an enable
tau_take = 0.15;                                 % s [EST], see above
for k = 1:2
    vk = x(k);
    if en(k)
        aload = g*sin(pitch) - frr*g*cos(pitch)*sign(vk);
        if take
            ff(k) = ff(k) + Ts/tau_take*(-aload - ff(k));
            areq = (v_cmd(k) - vk)/tau + ff(k);
        else
            areq = (v_cmd(k) - vk)/tau - aload;
        end
        if vk >= 0
            ahi = tlim(k)*kv/ms; alo = -tlim(2 + k)*kv/ms;
        else
            ahi = tlim(2 + k)*kv/ms; alo = -tlim(k)*kv/ms;
        end
        a = min(max(areq, alo), ahi) + aload;
    else
        ff(k) = 0;
        a = -vk/tau;
    end
    x(k) = vk + Ts*a;
end
end
