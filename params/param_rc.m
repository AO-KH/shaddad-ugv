function R = param_rc()
%PARAM_RC  RC transmitter / radio / receiver / SBUS line model (owner: electronics, RC sub-domain).
%   R = param_rc() -> P.rc. Default hardware: RadioMaster TX16S + FrSky Archer SR10 (SBUS,
%   raw range 172..992..1811), receiver failsafe mode FLAG_PRESET with the presets below.

R.frame_period   = 0.014;                    % s (normal SBUS); 0.007 = high speed
R.latency_frames = 1;                        % total TX-frame -> SBUS-output latency in frames (D37). One frame is
                                             % produced by the deterministic frame-period -> 1 ms Rate Transition,
R.extra_delay_frames = R.latency_frames - 1; % so the explicit radio Delay block length is latency_frames - 1 = 0
R.p_loss         = 0.01;                     % nominal random frame loss
R.seed           = 4242;
R.rx_failsafe_time = 1.0;                    % s of continuous loss before failsafe
R.rx_failsafe_mode = 1;                      % 1 FLAG_PRESET, 2 NO_PULSES, 3 HOLD

% TX channel map (1-based) - identical to P.electronics.rcmap.ch
R.ch_map = struct('steer',1,'thr',2,'aux_lv',3,'aux_lh',4,'mode',5,'arm',6,'speed',7,'impl',8, ...
                  'reset',9,'lights',10,'lift',11,'estop',12,'mission',13,'rth',14);
R.raw_mid  = 992;  R.raw_half = 819.5;  R.raw_min = 172;  R.raw_max = 1811;
R.raw_sw3  = [172 992 1811];                 % 3-position switch positions 1/2/3
R.raw_sw2  = [172 1811];                     % 2-position switch off/on

% failsafe presets (FLAG_PRESET): sticks centred, mode HOLD, arm OFF, remote E-STOP = RUN (D34: the MCU
% handles failsafe as RC_FAILSAFE class B; REMOTE_ESTOP is evaluated only on valid frames)
fs = 992*ones(1,16);
fs(R.ch_map.mode)  = 992;                    % HOLD
fs(R.ch_map.arm)   = 172;                    % disarmed
fs(R.ch_map.estop) = 172;                    % RUN (no latched ESTOP from a failsafe frame)
fs(R.ch_map.reset) = 172; fs(R.ch_map.lights) = 172; fs(R.ch_map.rth) = 172;
fs(R.ch_map.mission) = 172; fs(R.ch_map.impl) = 172; fs(R.ch_map.lift) = 992;
R.failsafe_raw = fs;

% ---- RC_Link_Sim block parameters (T07, subsystems/rc) ----
R.n_ch = 16;                                 % SBUS proportional channels
% rc_transmitter assembles [sticks(5) sw3(4) sw2(5) spare] with a Vector Concatenate; a Selector with
% tx_perm reorders it into channel order (tx_perm(c) = element of that vector carried by channel c),
% so the channel map stays a parameter (ch_map) and is never hard-wired in the block diagram.
R.tx_sticks = {'steer','thr','aux_lv','aux_lh','speed'};      % Bus_OperatorCmd: stick_steer, stick_thr, stick_aux_lv, stick_aux_lh, knob_speed
R.tx_sw3    = {'mode','impl','lift','mission'};              % sw_mode, sw_impl, sw_lift, sw_mission (1/2/3, 0 = 1)
R.tx_sw2    = {'arm','reset','lights','estop','rth'};        % sw_arm, btn_reset, sw_lights, sw_estop, btn_rth (0/1)
src = [R.tx_sticks, R.tx_sw3, R.tx_sw2];
perm = zeros(1, R.n_ch);
for k = 1:numel(src), perm(R.ch_map.(src{k})) = k; end
spare = find(perm == 0);
perm(spare) = numel(src) + (1:numel(spare));
R.tx_perm = perm;                            % [1 2 3 4 6 10 5 7 11 12 8 13 9 14 15 16] for the default map
R.tx_spare_raw = R.raw_mid*ones(1, numel(spare));   % unassigned channels (15, 16) sit at centre
R.tx_ch_dig = false(1, 2);                   % CH17 / CH18 digital channels (not mapped on the TX)
R = rmfield(R, {'tx_sticks','tx_sw3','tx_sw2'});    % P stays numeric/logical (D58); names documented above

% receiver (D59: blocks, no chart): flags [failsafe_bit sent] per failsafe mode row (1 FLAG_PRESET,
% 2 NO_PULSES, 3 HOLD) once the continuous-loss timer reaches rx_failsafe_time; rx_flags_ok otherwise
R.rx_fs_flags  = logical([1 1; 0 0; 0 1]);
R.rx_flags_ok  = logical([0 1]);
R.rx_timer_sat = 10;                         % s, upper limit of the continuous-loss timer (bounded state)

% UART line timing (sim only)
R.uart = struct('Tb',1.2e-4,'phase0',3.7e-4,'drift_ppm',200,'p_err',0,'p_drop',0);
R.seed_err  = R.seed + 1000*(1:25);          % per-byte Uniform Random Number seeds (byte errors)
R.seed_drop = R.seed + 1000*(26:50);         % per-byte Uniform Random Number seeds (byte drops)

% joystick mapping (vrlib Joystick Input, Xbox layout) for VC_OPSRC == 3
R.joy = struct('axis_steer',4,'axis_thr',5,'thr_sign',-1,'btn_arm',1,'btn_estop',2,'btn_mode',3, ...
               'btn_reset',8,'btn_start',4,'btn_pause',5);
end
