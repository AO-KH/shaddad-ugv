function D = param_drive(r_spr, option)
%PARAM_DRIVE  Traction motors, ESCs, gearboxes, brakes, drive thermal, CAN protocol (owner: drive).
%   D = param_drive(r_spr)           r_spr = P.structure.r_sprocket (default 0.090 m), motor of param_motor_unit
%   D = param_drive(r_spr, option)   motor option of param_motor_unit ('kw1' | 'movematic750')
%   Variant control derived by shaddad_params: VC_DRIVE_FID (1 SYS | 3 IDEAL in the system;
%   2 FOC is bench-only, tests/drive/tb_drive_foc, D52).
%   Revision B: ESC setpoint ramp above every MCU deceleration (D33), speed PI re-tuned on the
%   two-mass belt/vehicle model (D50), 1 ms command delay = loop breaker (D36), gearbox rating
%   >= motor peak through the gear (D51), network tie for the IDEAL choice (D35).
%   Real robot (2026-10-27, docs/hardware/real_robot_components.md): four traction motors, two per track (front and
%   rear sprocket) on one dual-channel ESC per side [DATA, user]. Each side is modelled as ONE equivalent drive of
%   n_mot = 2 identical units in parallel (same speed and voltage): torque, power, current, inertia, iron losses,
%   brake and thermal capacity x 2; winding resistance and inductance / 2. Unit data: param_motor_unit.
if nargin < 1 || isempty(r_spr), r_spr = 0.090; end
if nargin < 2, option = []; end
u = param_motor_unit(option);                % one motor (with its gearbox, brake, ESC channel)
n = 2;                                       % motors per side: front + rear sprocket
u0 = param_motor_unit('movematic750');       % as built: the 220 kg of the spec sheet include four of these
m_side = (220 + 4*(u.unit_mass - u0.unit_mass) + 50 + 40)/2;   % [EST] kg per track: robot + sprayer + 40 L liquid
k_t = u.mass/u0.mass;                        % thermal size of the unit relative to the 3.5 kg motor

D.Ts_esc = 1e-3; D.Ts_can = 1e-2; D.Ts_can_slow = 5e-2; D.Ts_plant = 1e-3;
D.Ts_foc = 1e-4; D.Ts_foc_plant = 5e-5;
D.fidelity = 1;
D.motor_unit = rmfield(u, {'option', 'name'}); D.n_mot_side = n;   % P stays numeric (D58): name in P_meta.drive

%% ---- gearbox / kinematics (two 2-stage planetary gearboxes per side, one at each sprocket) ----
gr.N = 20; gr.eta = 0.90; gr.k_loss = (1 - gr.eta)/gr.eta; gr.w_th = 0.5;   % [EST]
gr.T_nl_brk = n*0.04; gr.T_nl_C = n*0.03; gr.w_nl_brk = 0.5; gr.c_nl = n*0.6e-4;  % [EST] scaled with the rating
gr.J_out = n*0.004; gr.backlash_arcmin = 15; gr.mass = n*u.gear_mass;
gr.T_rated_out = n*u.gear_T(1);              % T2N of the side (per gearbox >= adhesion share and the motor continuous)
gr.T_acc_out   = n*u.gear_T(2);              % T2B of the side (per gearbox >= motor peak through the gear)
gr.T_estop_out = n*u.gear_T(3);              % T2NOT of the side
gr.r_spr = r_spr;
gr.k_v2w = gr.N/gr.r_spr;                    % (rad/s motor)/(m/s track) = 222.2 -> 1.41 m/s at 3000 rpm
D.gear = gr;

%% ---- motor: equivalent of the side's two units (surface PMSM model) ----
% unit [EST] values in param_motor_unit; consistent with 48 V (lambda*w_e <= 48/sqrt(3) at 3600 rpm)
m.P_rated = n*u.P_rated; m.n_rated = u.n_rated; m.T_cont = n*u.T_cont; m.T_peak = n*u.T_peak;
m.P_peak_int = n*u.P_peak_int;
m.n_max = u.n_max; m.n_mech_max = u.n_mech_max; m.p = u.p; m.lambda_pm = u.lambda_pm; m.Kt = 1.5*m.p*m.lambda_pm;
m.Rs = u.Rs/n; m.Ld = u.L/n; m.Lq = u.L/n; m.L0 = u.L0/n; m.alpha_cu = 3.93e-3;
m.k_h = n*u.k_h; m.k_e = n*u.k_e; m.J = n*u.J; m.lam = n*0.5e-4; m.Tc = 5e-3; m.Res = 3e-3;
m.V_nom = 48;
m.w_t_rpm   = u.w_t_rpm;
m.T_pk_nom  = n*u.T_pk_nom;
m.T_cont_nom = n*u.T_cont_nom;
m.vdc_t     = [36 42 48 54.6];
m.w_eff_rpm = [-4000 -3000 -2000 -1000 -400 0 400 1000 2000 3000 4000];
m.T_eff     = linspace(0, ceil(1.1*m.T_peak), 9);
m.Tmeas1 = 25; m.Tmeas2 = 125;
m.mass = n*u.mass;
m.T_pk_2D = local_envelope2D(m);
D.motor = m;

%% ---- ESC (dual-channel CiA-402-class FOC controller per side, speed mode; ratings [EST] per channel x 2) ----
e.node_id = [1 2];                           % one CAN node per side (both channels get the same command)
e.I_rms_cont = n*u.esc(1); e.I_rms_peak = n*u.esc(2); e.I_dc_max_mot = n*u.esc(3); e.I_dc_max_regen = n*u.esc(4);
e.R_on = 4e-3/n; e.k_sw = 1.6e-4; e.P_q = 5; e.R_q = m.V_nom^2/e.P_q;
e.V_on = 38; e.V_off = 35; e.V_uv_derate = [37 40]; e.V_ov_derate = [57 60];      % 13S NMC: 39.0 .. 54.6 V
e.V_ov_fault = 62; e.V_uv_fault = 35; e.I_oc_fault = 1.3*e.I_dc_max_mot;
e.J_eq = m.J + (m_side*r_spr^2)/gr.N^2;      % m/2 = m_side kg reflected to the motor
% speed PI (D64): re-tuned for the four-motor drive on the two-mass model (tb_drive_2mass_lti, the T21 cases k_x
% 40 / 61 / 540 kN/m at bench2m.m_side). The D50 gains (0.13 / 0.39 at J_eq 10.18e-3) scaled with J_eq gave Ms 1.65
% (one motor per side: Ms 2.25, zeta 0.25). Ki/Kp = 3 as in D50: with Ki 0.060 the integrator still held about
% 0.2 N m after a controlled stop and S6a reached standstill only after 1.63 s (limit 1.5 s). kw1 x 4: PM 62 deg,
% GM 15.0 dB, Ms 1.37, zeta 0.57; movematic750 x 4: PM 63 deg, GM 13.1 dB, Ms 1.46, zeta 0.51 (T21: Ms <= 1.6,
% zeta >= 0.35)
e.Kp_w = 0.030; e.Ki_w = 0.090;
e.w_c = e.Kp_w/e.J_eq;                       % rad/s rigid-model crossover (8.0 rad/s, kw1)
e.cmd_delay = 1;                             % samples (1 ms) Unit Delay on every ESC -> plant S-PS input (Tr, brake): loop breaker
e.a_max = 2.24*gr.k_v2w; e.a_qs = 2.0*gr.k_v2w;   % rad/s^2 motor (2.24 / 2.0 m/s^2): above every MCU shaper decel (MCU is the only shaper)
e.J_ff = e.J_eq;                             % kg m^2, ESC inertia feed-forward of its own ramp in the hold-zero states
                                             % (QUICK_STOP / DISABLING / ENABLING / PARKED, where no MCU feed-forward exists):
                                             % with the low D50 gains the stop would otherwise lag and overshoot (T10)
e.enc_cpr = 4096; e.n_diff = 4; e.tau_wf = 2e-3;
e.t_boost = 30; e.t_recover = 60; e.x_off = 0.8;
e.T_mot_derate = [413.15 438.15]; e.T_mot_fault = 443.15;
e.T_esc_derate = [358.15 373.15]; e.T_esc_fault = 378.15;
e.tau_Tsens = 5; e.w_overspeed = m.n_mech_max*pi/30; e.t_can_timeout = 0.1;
e.t_premag = 0.05; e.t_stop_hold = 0.2; e.w_stop = 2.0; e.t_park = 2.0; e.t_init = 0.1;
e.t_stop_max = 2.0;                          % s [EST] (critfix R-01): DISABLING / QUICK_STOP time-out from the start of the
                                             % stop, then the brake engages whatever the speed; above every normal ramp stop
                                             % (1.41 m/s at a_qs 2.0 m/s^2 = 0.7 s, + t_stop_hold 0.2 s)
e.stall_tau_frac = 0.9; e.stall_w = 5; e.t_stall_warn = 5; e.t_stall_fault = 10;
e.C_link = 2.2e-3; e.ESR_link = 10e-3; e.R_bleed = 1e4;          % DC link inside the ESC (POWER precharge)
e.vc0 = 0;                                   % V, initial DC-link capacitor voltage (system: precharged by POWER; benches set V_src)
e.mass = n*u.esc_mass;                       % dual-channel ESC of the side
% ---- ESC firmware-model details (build plan T06.2; all at Ts_esc) ----
e.x_on = 1 - 1e-6;                           % I2t relay on point (boost exhausted)
e.x_off = 0.01;                              % I2t relay off point: T4 needs peak torque again only after ~60 s of zero
                                             % torque (x decays at 1/t_recover); spec 6.4 value 0.8 gave 12 s (contradiction)
e.k_boost = ((m.T_peak/m.T_cont)^2 - 1)*e.t_boost;   % s, I2t charge-rate denominator (= 90 s)
e.tau_cont_min = n*0.15;                     % N m, floor of the continuous rating in the I2t ratio (above 3600 rpm it is 0)
e.w_min_dc = 50;                             % rad/s, speed floor in the DC-current torque limits tau_dcm / tau_dcr
e.w_quad = 5;                                % rad/s, quadrant threshold of the torque limiter
e.a_track = 1e6;                             % rad/s^2, ramp "limit" while the speed loop is inactive (ramp tracks w_meas)
e.n_can_to = round(e.t_can_timeout/D.Ts_esc);         % 100 samples, Software Watchdog Timer maxCount
e.n_oc = 10; e.n_ov = 5; e.n_uv = 50; e.n_os = 10;    % Protection Relay CountMax (samples at 1 ms)
e.n_ot = 1; e.n_enc = 2; e.n_brake = 100;             % temperature / encoder / brake fault debounce (samples)
e.n_stall = round(e.t_stall_fault/D.Ts_esc);          % 10000 samples
e.tau_trip = 1e-3;                           % tau_scale below this with enable -> OVERCURRENT (drive trip)
e.lim_eps = 1e-6;                            % N m, TORQUE_LIMIT_ACTIVE detection band
e.derate_eps = 1e-3;                         % DERATING flag when the derating product < 1 - derate_eps
e.n_alive = round(D.Ts_can/D.Ts_esc);        % ESC alive counter increments every 10 ms (one FB frame)
e.a_wf = exp(-D.Ts_esc/e.tau_wf);            % speed LPF pole
e.a_Tc = exp(-D.Ts_esc/m.Tc);                % torque-estimate pole (motor torque lag)
e.a_Ts = exp(-D.Ts_esc/e.tau_Tsens);         % winding sensor pole
D.esc = e;

%% ---- brake (spring applied, electrically released; one per motor, the side's two as one equivalent) ----
% [EST] unit 5 N m: the four hold the full 320 kg robot on 20 deg with margin 4
% Elasto-plastic friction (D64, subsystems/drive/+shaddad_drive/elastoplastic_brake.ssc): a bristle state sticks as
% a spring k_b below mu_s r_eff N and slides with the Stribeck curve (w_st) to mu_c r_eff N. The former Rotational
% Friction changed sign at every 1 ms step when engaged and, with two brakes per side, diverged (S8a: rotor 3400 rad/s
% in ESTOP). Bristle [EST] 25 N m/rad and 0.15 N m s/rad per brake (x n); creep w_r*T/g (T8: 2.4e-5 rad/s);
% T_eps, w_eps and N_eps are numerical guards (released brake: no torque)
D.brake = struct('T_static',n*u.brake_T,'T_dyn',n*0.84*u.brake_T,'F_spring',n*250,'r_eff',0.05,'mu_s',0.40,'mu_c',0.333, ...
    'w_st',0.1,'k_b',n*25,'c_b',n*0.15,'w_d',1,'T_eps',1e-4,'w_eps',1e-3,'w_r',1e-4,'N_eps',1, ...
    't_release',0.04,'t_engage',0.06,'R_coil',92/n,'duty_hold',0.48,'t_pullin',0.2,'mass',n*u.brake_mass);
D.brake.duty_min = 1e-4;                     % coil resistance R_coil/max(duty, duty_min)
D.brake.t_engage_wait = 0.08;                % s, torque stays on 80 ms after the brake command drops (spec 2.9)

%% ---- ESC state machine (subsystems/drive/esc_sm.m, MATLAB Function FSM, timers in 1 ms ticks; D1) ----
tk = @(t) round(t/D.Ts_esc);
D.sm = struct('n_init',tk(e.t_init),'n_premag',tk(e.t_premag),'n_release',tk(D.brake.t_release), ...
    'n_engage',tk(D.brake.t_engage_wait),'n_stop_hold',tk(e.t_stop_hold),'n_park',tk(e.t_park), ...
    'n_pullin',tk(D.brake.t_pullin),'n_reset_win',3,'V_on',e.V_on,'V_off',e.V_off,'w_stop',e.w_stop, ...
    'w_park_ref',0.05,'tau_park_ref',0.005,'duty_hold',D.brake.duty_hold, ...
    'hard_mask',bitcmp(uint16(2^2 + 2^6 + 2^10)), ...  % every fault bit except UNDERVOLTAGE, CAN_TIMEOUT, STO_ACTIVE
    'n_stop_max',tk(e.t_stop_max));                    % ticks, DISABLING / QUICK_STOP brake time-out (critfix R-01)
% the same values as a numeric vector on an input of the ESC_SM MATLAB Function (Constant block, layout in esc_sm.m)
s_ = D.sm;
D.sm_vec = [s_.n_init s_.n_premag s_.n_release s_.n_engage s_.n_stop_hold s_.n_park s_.n_pullin s_.n_reset_win ...
    s_.V_on s_.V_off s_.w_stop s_.w_park_ref s_.tau_park_ref s_.duty_hold double(s_.hard_mask) s_.n_stop_max];

%% ---- thermal (the side's two motors and ESC channels as one node set: capacities x n, resistances / n) ----
% [EST] 3.5 kg motor values scaled with the unit size k_t (capacity ~ mass, surface ~ mass^(2/3))
D.thermal = struct('C_w',n*550*k_t,'m_h',n*2.5*k_t,'cp_h',750,'R_wh',0.25/(n*k_t),'A_h',n*0.07*k_t^(2/3),'h_h',10, ...
    'R_hc',0.6/(n*k_t^(2/3)),'m_esc',n*0.5,'cp_esc',900,'R_esc',0.8/n,'dT_solar',10,'T_amb0',318.15);

%% ---- loss maps (motor + ESC), W, [numel(w_eff) x numel(T_eff)] ----
[D.motor.loss_T1, D.motor.loss_T2] = local_lossmap(D.motor, D.esc);

%% ---- CAN protocol (CAN1, classic 500 kbit/s, little-endian) ----
D.can = struct('bitrate',500e3,'id_cmd',hex2dec('200'),'id_fb1',hex2dec('180'),'id_fb2',hex2dec('280'), ...
    'id_fb3',hex2dec('380'),'res_w',2*pi/60,'res_tau',0.01,'res_tau_lim',0.1,'res_i',0.1,'res_v',0.01, ...
    'res_T',0.1,'t_rx_timeout',0.05);
D.can.n_rx_timeout = round(D.can.t_rx_timeout/D.Ts_can);   % 5 frames at 10 ms: drive_fb_unpack rx_ok
D.can.T0 = 273.15;                           % K <-> degC on the wire (FB2 T_motor 0.1 degC, T_esc 1 degC)
D.can.T_esc_ofs = 40;                        % degC, FB2 u8 T_esc offset (-40 .. 215 degC)
D.can.len = 8;                               % classic CAN DLC of every DRV frame
fi = struct('can_drop',false,'enc_fail',false,'brake_stuck',false,'tau_scale',1,'T_offset',0);
D.fault_inj_default = struct('L', fi, 'R', fi);

%% ---- IDEAL variant ----
D.ideal = struct('tau_v',0.05,'a_max',e.a_max,'eta',0.80,'P_aux',8);
D.ideal.a_v = exp(-D.Ts_esc/D.ideal.tau_v);  % first-order speed lag pole at 1 ms
D.ideal.k_loss = (1 - D.ideal.eta)/D.ideal.eta;   % DC load P_dc = P_mech + k_loss*|P_mech| (+ quiescent resistor)
D.ideal.V_min = 20; D.ideal.V_gate = 10;     % V, current-source divisor floor and gate width (no current below ~V_min)
% network tie (D35): Rotational Electromechanical Converter K between Spr and a 1 MOhm thermal resistor to DC-
% keeps Spr, the DC48 network and drive_thermal one Simscape network in choices without a motor (IDEAL, T01 stub)
D.tie = struct('K',1e-3,'R',1e6);
D.fid_system = [1 3];                        % VC_DRIVE_FID values allowed in Shaddad_Plant (2 FOC bench-only)
% two-mass bench load (D50): belt side (motor + gearbox + sprocket + belt) vs vehicle half-mass via shear spring
D.bench2m = struct('J_b',m.J + (gr.J_out + 0.5*0.03 + 8*r_spr^2)/gr.N^2,'k_x',[4e4 6.1e4 5.4e5],'tau_d',0.01, ...
                   'm_side',[m_side - 45, m_side],'Ms_max',1.6,'zeta_min',0.35,'os_max',0.02);   % robot alone / full sprayer [EST]
% one-side bench tb_drive (spec 7/8): battery 46.8 V + 20 mOhm, m_side kg at the sprocket
D.bench = struct('V_src',46.8,'R_src',0.02,'m_side',m_side,'g',9.81,'f_rr',0.05,'slope_deg',0, ...
                 'J_dyno',1e3,'w_brk_rr',0.05);
D.bench.J_load = D.bench.m_side*r_spr^2;     % 1.26 kg m^2
% FOC bench variant (spec 3.7, bench-only, D52): PMSM + average-value inverter + ee_sl_lib current control
D.foc = struct('Kp_i',0.25,'Ki_i',25,'P_max',1.2*m.P_peak_int,'I_max',1.6*e.I_rms_peak,'C_link',2.2e-3,'thermal_mass_inv',10);
D.foc.Kp_i = m.Lq*2*pi*500; D.foc.Ki_i = m.Rs*2*pi*500;     % pole-zero cancellation, 500 Hz current bandwidth
D.foc.Irms_vect = n*[5 10 20 30 40 60]*u.esc(2)/50;      % A rms, average-value inverter loss profile
D.foc.Ploss_vect = 0.5 + 3*e.R_on*D.foc.Irms_vect.^2 + e.k_sw*m.V_nom*D.foc.Irms_vect;   % W, SYS-map law (+0.5 W:
                                                           % the block needs a positive derived fixed loss)
D.foc.f_losses = 100;                                      % Hz electrical, empirical iron-loss reference point
D.foc.losses_oc = [m.k_h*2*pi*D.foc.f_losses, m.k_e*(2*pi*D.foc.f_losses)^2, 0];   % W [hysteresis eddy excess]
D.foc.C_stator = n*400*k_t; D.foc.C_rotor = n*300*k_t;    % J/K per phase / rotor: 3*C_stator + C_rotor = C_w (winding node)
D.foc.thermal_mass_inv = n*10;                             % J/K, inverter thermal port on H (heat split moves P_esc to E)

%% ---- mass / budget handed to structure and power ----
D.mass_per_side = n*u.unit_mass;             % [EST] 2 x (motor + gearbox + brake + half dual ESC + cables)
D.J_reflected_spr = m.J*gr.N^2;              % kg m^2 per side (included dynamically)
end

function T2D = local_envelope2D(m)
wt = m.w_t_rpm; T2D = zeros(numel(wt), numel(m.vdc_t));
for j = 1:numel(m.vdc_t)
    T2D(:,j) = interp1(wt, m.T_pk_nom, min(wt*m.V_nom/m.vdc_t(j), wt(end)), 'linear', 0);
end
T2D(wt > m.n_mech_max, :) = 0;
end

function [L1, L2] = local_lossmap(m, esc)
we = m.p*m.w_eff_rpm(:)*pi/30;  [WE, TT] = ndgrid(we, m.T_eff(:)');
I = TT/m.Kt;  Irms = I/sqrt(2);
Pfe  = m.k_h*abs(WE) + m.k_e*WE.^2;
Pesc = 3*esc.R_on*Irms.^2 + esc.k_sw*m.V_nom*Irms;
Pcu  = @(Tc) 1.5*m.Rs*(1 + m.alpha_cu*(Tc - 25)).*I.^2;
L1 = Pcu(m.Tmeas1) + Pfe + Pesc;
L2 = Pcu(m.Tmeas2) + Pfe + Pesc;
end
