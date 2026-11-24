function E = param_electronics(P)
%PARAM_ELECTRONICS  STM32H743 firmware, CAN networks, electronics plant (owner: electronics).
%   E = param_electronics(P)   P: partially assembled P (needs P.structure, P.drive, P.world)
%   Architecture decisions applied here (docs/architecture.md section 2):
%     * traction ESCs run their own 1 kHz speed loop (SPEED mode); the MCU runs vehicle-level
%       control at 100 Hz: shaping, yaw-rate PI on the IMU gyro, skid mixing, torque limits;
%     * the BMS ECU owns the contactors; the MCU only requests HV over CAN (BMS_REQ);
%     * motor encoders are read by the ESCs and forwarded over CAN (FB3);
%     * all state machines are MATLAB Function FSMs (Stateflow product not installed).
%   Revision B (docs/architecture.md D30-D63): MCU_SAFETY_OK depends on MCU-internal health only and
%   the MCU re-arms the safety relay (D30); stop profiles use a constant-deceleration law (D33); the
%   remote E-stop is evaluated only on valid frames (D34); single owner of rc_required_in_auto (D44);
%   gauge factor chi comes from P.structure.track.chi_nom (D39); AUTO yaw shaper is non-binding (D41);
%   tilt estimate compensates linear acceleration (D47); CAN2 loop breaker moved to MCU_STATUS (D41).

%% ---- task rates ----
E.ts = struct('base',1e-3,'imu',5e-3,'ctrl',1e-2,'comm',5e-2,'hk',0.1);

%% ---- SBUS ----
E.sbus = struct('baud',100000,'bits_per_byte',12,'t_byte',1.2e-4,'frame_len',25,'payload_len',22, ...
    'n_ch',16,'ch_bits',11,'header',15,'footer',0,'sbus2_low_nibble',4,'read_max',32,'gap_ticks',2, ...
    'link_timeout',0.10,'lq_alpha',0.05,'lq_warn',0.70);

%% ---- RC channel mapping (Mode 2 TX; see docs/icd.md section 5) ----
rc.ch = struct('steer',1,'thr',2,'aux_lv',3,'aux_lh',4,'mode',5,'arm',6,'speed',7,'impl',8, ...
               'reset',9,'lights',10,'lift',11,'estop',12,'mission',13,'rth',14);
rc.raw_mid = 992*ones(1,16); rc.raw_half = 819.5; rc.raw_min = 172; rc.raw_max = 1811;
rc.sign = ones(1,16);
rc.deadband = 0.05; rc.expo_thr = 0.20; rc.expo_steer = 0.35; rc.sw_thresh = 0.5;
rc.debounce_n = 4;                           % samples at 10 ms = 40 ms (about 3 SBUS frames)
rc.s_min = 0.25; rc.neutral_tol = 0.08; rc.override_tol = 0.30; rc.override_t = 0.20;
rc.steer_to_omega_sign = -1;                 % stick right (+) => clockwise => omega < 0
rc.estop_debounce_n = 0;                     % CH12: no tapped-delay debounce (supervisor t_persist 0.04 s only)
rc.estop_valid_frames_only = true;           % REMOTE_ESTOP evaluated only on frames with failsafe = 0 and frame_lost = 0
E.rcmap = rc;

%% ---- limits, shaping ----
% real robot (2026-10-27): top track speed n_rated / k_v2w = 1.41 m/s (750 W motors at 3000 rpm, 20:1, r 0.090 [EST]);
% overspeed 1.2 x that, below the ESC overspeed n_mech_max / k_v2w = 1.88 m/s
v_trk = P.drive.motor.n_rated*pi/30/P.drive.gear.k_v2w;
E.lim = struct('v_max_man',1.4,'w_max_man',1.0,'v_max_auto',1.2,'v_rev_auto',0.5,'w_max_auto',1.2, ...
    'a_lat_max',1.0,'v_track_max',v_trk,'v_overspeed',1.2*v_trk,'t_overspeed',0.2);
% stop profiles (CTRL_STOP, ESTOP.DECEL): constant-deceleration law
%   a_dem = -sign(x_ref)*min(a_stop, sqrt(2*j_stop*|x_ref|))   (no proportional tail; D33)
% AUTO yaw axis: K_w_auto / alpha_auto / jw_auto above the GNC CommandShaping limits so the MCU
% shaper is a safety envelope only and adds about 0.05 s lag (D41).
E.shaper = struct('K_v',4,'K_w',5,'a_acc_man',0.8,'a_dec_man',1.2,'a_acc_auto',0.5,'a_dec_auto',1.0, ...
    'a_ctrlstop',1.0,'a_estop',2.0,'j_max',5,'j_stop',10,'alpha_max',1.5,'alpha_stop',3.0,'jw_max',8, ...
    'estop_tmax',1.2,'stop_law',2,'K_w_auto',20,'alpha_auto',3.0,'jw_auto',30);

%% ---- drive kinematics (single source: P.structure / P.drive) ----
B   = P.structure.geom.B;
r_s = P.structure.r_sprocket;
N   = P.drive.gear.N;
E.mix = struct('B',B,'chi',P.structure.track.chi_nom);   % single source: structure (identified in T03)
m_eq = P.structure.mp.m0/2 + P.drive.motor.J*N^2/r_s^2;
E.drv = struct('r_s',r_s,'N',N,'eta_g',P.drive.gear.eta,'eta_drv',0.85,'T_peak',P.drive.motor.T_peak, ...
    'T_cont',P.drive.motor.T_cont,'w_min',20,'J_m',P.drive.motor.J,'m_eq_side',m_eq, ...
    'k_v2w',N/r_s,'enc_cpr',P.drive.esc.enc_cpr,'K_enc',2*pi*r_s/(P.drive.esc.enc_cpr*N), ...
    'Kff_a',m_eq*r_s/(N*P.drive.gear.eta),'odom_window',5,'enc_mismatch_v',0.3,'enc_mismatch_t',0.2, ...
    'w_ref_max',P.drive.motor.n_max*pi/30);

%% ---- yaw-rate loop (outer, on bias-corrected gyro z) ----
% commanded standstill (S4 stop 3, 2026-11-14): v_target = w_target = 0 with |v_ref| < cmd_still_v and |w_ref| <
% cmd_still_w (or the drive disabled) resets the yaw PI (integrator 0, u = 0) and, with the measured standstill
% (max |v_track_L,R| < standstill_v), enables the gyro re-bias average. cmd_still_v 0.005 m/s, cmd_still_w
% 0.005 rad/s: the commanded track speed left then is at most 0.005 + 0.005*chi*B/2 = 0.007 m/s, a third of
% standstill_v, and a turn still decaying under 0.005 rad/s adds under 1e-3 rad/s to the 2 s bias average; in S4
% the references fall under them 1.3 s (v) and 0.1-0.4 s (w) after the targets reach 0 (pauses of 5-90 s)
E.yaw = struct('enable',true,'Kp',0.4,'Ki',2.0,'u_max',0.4,'tau_bias',2.0,'standstill_v',0.02, ...
               'tau_target',0.25,'cmd_still_v',0.005,'cmd_still_w',0.005);

%% ---- traction control, torque/power limiter ----
E.trac = struct('enable',true,'s_on',0.30,'s_off',0.20,'k_min',0.5,'k_dn',2.0,'k_up',1.0,'v_min',0.2);
E.tlim = struct('dis_margin',0.95,'P_aux_est',120);

%% ---- safety supervisor (bit table: docs/icd.md section 6) ----
% bit 22 STOP_FAIL (D71, 2026-11-15; WDG_RESET_SEEN, info, before): class A, latched, in maskDO0 (STO backup of the
% soft stops); a watchdog reset stays visible in MCU_STATUS flags b7 wdg_reset_seen and in the BOOT -> SELFTEST sequence
bitsA  = [0 5 6 7 8 22 25 27 29 31];
bitsB  = [2 3 4 9 10 12 13 15 16 17 18 19 21 23 26];
bitsC  = [14 20 24 28 30];                   % bit 11 BMS_WARN is info since D65 (2026-11-12), no longer class C
bitsL  = [0 1 5 6 7 8 9 10 12 13 15 16 17 21 22 23 25 26 27 28 29];
mk = @(b) uint32(sum(2.^b));
tp = zeros(1,32);
bitsDO0 = [5 6 7 8 22 23 25];                % MCU-internal health only (D30): DRV timeouts/faults, STOP_FAIL (D71),
                                             % overrun, CAN1 bus-off
tp(1+[1 12 13 16 17 19]) = [0.04 0.2 0.2 0.2 0.2 0.2];
tp(1+[14 15 24]) = 0.5;  tp(1+[20 21]) = 5;  tp(1+27) = 0.05;  tp(1+28) = 0.2;  tp(1+30) = 1;
% tilt_warn 25 deg (was 22, 2026-10-27): 20 deg design slope + about 1 deg body pitch on the ramp + residual
% estimate error after the level calibration below (S2 / D47); tilt_stop unchanged
% derate_bms_warn 1 (was 0.5, D65, 2026-11-12): a BMS warning does not derate; the BMS limits reach the drives only
% through the torque limiter (DCL -> motoring, CCL -> regen, spec 2.7.9, D2). The field stays so a derate can come back
E.safety = struct('tilt_warn',25*pi/180,'tilt_stop',28*pi/180,'t_tilt',0.5,'cf_accel_gain',0.02, ...
    'comp_timeout',0.20,'drv_timeout',0.05,'bms_timeout',0.5,'impl_timeout',0.2, ...
    'estop_discrepancy_t',0.05,'standstill_v',0.05,'standstill_t',0.5,'ctrlstop_timeout',5.0, ...
    'selftest_t',1.0,'selftest_timeout',2.0,'auto_start_warn_t',3.0, ...
    'V24_min',21.6,'V12_min',11.0,'V5_min',4.6,'T_encl_warn',343.15,'T_encl_crit',353.15,'T_mcu_max',378.15, ...
    'maskA',mk(bitsA),'maskB',mk(bitsB),'maskC',mk(bitsC),'latch_mask',mk(bitsL),'t_persist',tp, ...
    'maskDO0',mk(bitsDO0),'overrun_n',3,'derate_bms_warn',1,'derate_tilt',0.5,'derate_imu',0.25,'derate_encl',0.5, ...
    'rearm',struct('t_do0_settle',0.02,'t_pulse',0.05,'auto_selftest',true), ...
    'rc_required_in_auto',true, ...
    'tilt_comp',struct('enable',true,'tau_acc',0.05,'gate_a',0.3));
% bit 31 SAFETY_RELAY_OPEN is evaluated only while ACTIVE (MANUAL/HOLD/AUTO_START/AUTO/CTRL_STOP), D30
% IMU level calibration (commissioning on level ground, D47 fix 2026-10-27): static roll / pitch of the tilt
% estimate with the robot standing on level ground (accelerometer and temperature bias, mounting misalignment);
% fw_imu_tilt subtracts it. Measured with scripts/imu_level_calibration.m; ideal-IMU benches set it to 0
E.imu_cal = struct('roll0', 0.03367, 'pitch0', -0.03714);   % rad, imu_level_calibration(S0_20261027_141618)
% IMU freshness (sens_acq_imu, MCU Task_5; 2026-11-15, R-05, review MCU-03 / NAV-03): imu_meas.valid is false once all
% 9 raw axes (accel, gyro, mag) have repeated bit for bit for imu_frozen_t, a frozen I2C read or a stuck sensor (plant
% imu_fault 1). The ICM-20948 noise at 200 Hz (about 19 / 10 / 6 LSB rms on accel / gyro / mag, P.sensors.imu) changes
% some axis every sample, so a healthy sensor never trips it; benches with a noise-free IMU must toggle one LSB
E.safety.imu_frozen_t = 0.1;                 % s, 20 samples at Ts.imu [EST]
% Soft-stop policy (D71, R-01, the user's decision of 2026-11-15: brakes first, STO as the backup, HV stays on after a
% normal stop). CTRL_STOP and ESTOP.DECEL decelerate, then park: park_req with the torque limits held until both ESCs
% report the brake engaged with the torque off. CTRL_STOP time-out min(ctrlstop_timeout, v0/a_ctrlstop +
% ctrlstop_t_extra), v0 = the faster track's |v| at entry. Stop monitor (fault bit 22 STOP_FAIL, class A, DO0): (a) the
% faster track rises stopfail_dv above v0, or from stopfail_t0 after entry stays above max(v0 - stopfail_rate*a*t,
% standstill_v) (a = a_ctrlstop / a_estop); (b) the brakes do not confirm stopfail_brake_t after the park request;
% (c) motion above standstill_v for stopfail_move_t in STANDBY, or in ESTOP.SAFE or FAULT once standstill was seen
% there (those two can be entered at speed)
E.safety.ctrlstop_t_extra = 1.0;             % s, CTRL_STOP time-out margin over v0/a_ctrlstop [EST]
E.safety.stopfail_dv      = 0.1;             % m/s, (a) rise above v0 [EST]
E.safety.stopfail_t0      = 0.5;             % s, (a) half-rate check from this time after entry [EST]
E.safety.stopfail_rate    = 0.5;             % 1, (a) the speed must fall at least at this fraction of a [EST]
E.safety.stopfail_brake_t = 0.5;             % s, (b) brake confirmation time-out after the park request [EST]
E.safety.stopfail_move_t  = 0.2;             % s, (c) motion while stopped [EST]

%% ---- watchdog ----
E.wdg = struct('timeout',0.05,'prescaler',4,'reload',399,'age_lim',[0.02 0.1 0.2],'t_boot',0.3, ...
               'kr_refresh',43690);

%% ---- ADC front end ----
ntc_T = linspace(233.15, 393.15, 33);        % K
R25 = 10e3; Bn = 3950; Rpu = 10e3;
Rn = R25*exp(Bn*(1./ntc_T - 1/298.15));
cnt = 3.3*Rn./(Rn + Rpu)/3.3*65535;
[cs, ix] = sort(cnt);
E.adc = struct('vref',3.3,'nbits',16,'k_vpack',21,'k_vlink',21,'k_v24',11,'k_v12',5.7,'k_v5',2, ...
    'i_sens',4.1667e-3,'i_off',1.65,'ntc_R25',R25,'ntc_B',Bn,'ntc_Rpu',Rpu, ...
    'ntc_lut_counts',cs,'ntc_lut_K',ntc_T(ix),'noise_lsb',4,'seed',1101,'liftpot_lim',[0.02 0.98]);

%% ---- CAN networks ----
h = @hex2dec;
E.can = struct('can1_bitrate',500e3,'can2_nominal',1e6,'can2_data',2e6, ...
    'can1_latency_samples',0,'can2_cmd_delay',0,'can2_status_delay',1, ...
    'id',struct('drv_cmd',h('200'),'drv_fb1',h('180'),'drv_fb2',h('280'),'drv_fb3',h('380'), ...
                'impl_cmd',h('140'),'impl_fb',h('141'),'bms_req',h('3D0'),'bms_limits',h('351'), ...
                'bms_soc',h('355'),'bms_vit',h('356'),'bms_alarm',h('35A'),'bms_cells',h('373'), ...
                'bms_state',h('3C0'),'bms_power',h('3C1'),'comp_cmd',h('100'),'mcu_status',h('200'), ...
                'mcu_imu',h('201'),'mcu_odom',h('202'),'mcu_power',h('203')), ...
    'len',struct('comp_cmd',32,'mcu_status',48,'mcu_imu',48,'mcu_odom',32,'mcu_power',64));

%% ---- DI / DO bit indices (0-based) ----
E.di = struct('estop_ch1_ok',0,'estop_ch2_ok',1,'safety_ok',2,'bumper_front',3,'bumper_rear',4, ...
    'reset_btn',5,'k1_aux',6,'k2_aux',7,'impl_id0',8,'impl_id1',9,'lid_closed',10,'key_on',11, ...
    'safety_hv_ok',12,'dock_present',13);
E.do = struct('mcu_safety_ok',0,'safety_reset_req',1,'impl24_en',2,'beacon_green',3,'beacon_amber',4, ...
    'beacon_red',5,'buzzer',6,'work_lights',7,'companion_pwr_en',8,'lidar_pwr_en',9,'nozzle1',10, ...
    'lift_dir',18);

%% ---- auxiliary outputs / implement ----
E.aux = struct('fan_bp',[308.15 313.15 328.15],'fan_duty',[0 0.2 1.0], ...
    'derate_bp',[338.15 343.15 353.15],'derate_k',[1 0.5 0], ...
    'spray_v_min',0.2,'pump_duty_man',0.6,'lift_Kp',20,'lift_duty_max',0.6, ...
    'blink_f',1,'buzz_period',2.0,'buzz_on',0.1, ...
    'deck_h_default',0.06,'till_d_default',0.08,'mower_w_default',293.2,'tiller_w_default',26.18);

%% ---- electronics loads (W at the rail) ----
E.load = struct('mcu',3.0,'rc_rx',0.5,'gnss',1.6,'imu',0.03,'can_xcvr',0.3, ...
    'jetson',[10 12 22],'lidar3d',20,'lidar2d',0,'modem',3,'cams',8,'display',5, ...
    'fan_each',5,'beacon',3,'lights_each',20, ...
    'inbox_mcu',true,'inbox_jetson',true,'P_inbox_extra',15);
% P_inbox_extra (T10): DC/DC conversion losses + quiescent (24 V: 0.06*~25 + 4 W, 12 V: 0.07*~40 + 3 W, 5 V ~0.5 W)
% and relay / CAN / fuse dissipation inside the enclosure; with MCU 3 W + Jetson 12 W the in-box heat is
% Q_int = 30 W (spec 2.9.4: 25-40 W).

%% ---- enclosure thermal (own Simscape network, local BE 0.1 s) ----
E.therm = struct('mass',12,'cp',1000,'area',1.2,'h_nat',4,'h_fan',12,'alpha_s',0.3,'A_s',0.24, ...
                 'T0',318.15,'Ts',0.1,'dT_imu',5,'dT_mcu',15);
% h_nat (T10): laminar natural convection on the sealed box, h = 1.42*(dT/L)^0.25 = 3.8 W/(m^2 K) at
% dT = 20 K, L = 0.4 m (no radiation credit). 50 degC ambient + 1000 W/m^2: Q = 30 + 72 = 102 W ->
% steady 55.3 degC with both fans at 1.0 (h 16), 71.3 degC with fans disabled (h 4) (build plan T10 T-EL-17).

%% ---- NTRIP / RTK correction link model ----
E.ntrip = struct('reconnect_t',10);

%% ---- variant default ----
E.variant = struct('target',0);

%% ---- T08 firmware platform (Shaddad_MCU: IO variants, SBUS decoder, RC mapping, tilt, codecs) ----
E.sbus.age_sat  = 10;                        % s, rc_age saturation (also its value at boot: no link yet)
E.sbus.lq_ic    = 0;                         % LQ filter state before the first frame (no evidence)
E.rcmap.sw2_thresh  = 0;                     % 2-position switches (172 / 1811 -> x = -1 / +1): ON when x > 0
E.rcmap.debounce_ic = [2 1 2 1];             % debounced 3-position IC [mode(HOLD) impl(OFF) lift(HOLD) mission(none)]
E.rcmap.aux_code    = [2 3 6];               % rc_aux_cmd MissionCmd codes [START(CH13 pos 2) PAUSE(CH13 pos 3) RTH(CH14)]
E.rcmap.override_n_sat = 1000;               % override persistence counter saturation (samples)
% CAN codecs (ICD 4 / 5): companion-command and MCU_STATUS / MCU_POWER resolutions, frame ages
E.can.res = struct('mower_w',0.1,'tiller_w',0.01,'deck_h',1e-3,'till_d',1e-3,'spray_flow',0.01, ...
                   'rc_lq',0.01);            % rad/s, rad/s, m, m, L/min, 1 per LSB
E.can.age_sat_s = 10;                        % s, saturation of every frame-age counter (and its boot value)
E.can.t_ms_mod  = 2^32;                      % MCU_STATUS t_ms wraps (uint32 ms); companion unwraps to t_mcu
E.can.gnc_age_ms_max = 65535;                % uint16 saturation of MCU_STATUS gnc_cmd_age_ms
% STM32H743 ADC3 internal temperature sensor (HW IO_In only): linear two-point calibration
% T = T1 + (cnt - c1)*(T2 - T1)/(c2 - c1); T17 replaces c1/c2 by the factory TS_CAL1/TS_CAL2 words
E.adc.ts_cal_T   = [30 110];                 % degC
E.adc.ts_cal_cnt = [12500 16100];            % ADC3 counts (16 bit, 3.3 V reference), placeholder until T17
% HW driver constants used by fw_io_in_hw / fw_io_out_hw (peripheral pin maps live in the CubeMX .ioc, T17)
E.hw = struct('uart_len',32,'di_pins',14,'do_g_pins',16,'do_e_pins',4,'adc1_n',8, ...
    'can1_rx_iter',16,'can2_rx_iter',8,'rcc_rsr_iwdg1rstf_bit',26,'rcc_rsr_rmvf_bit',23,'fdcan_psr_bo_bit',7);

%% ---- T10 electronics plant, CAN network emulation, companion IO ----
% NTRIP / RTK correction link: age of the corrections while the link is up (one RTCM epoch per second)
E.ntrip.corr_age_ok = 1;                     % s
% CAN1_Net_Sim drop-mask bits (Bus_FaultInject.can1_drop_mask, ICD 10) and MCU-originated slots
E.can.drop_bit = struct('drv_cmd_L',0,'drv_cmd_R',1,'fb1_L',2,'fb1_R',3,'fb3_L',4,'fb3_R',5,'fb2_L',6,'fb2_R',7, ...
    'impl_cmd',8,'bms_req',9,'impl_fb',10,'bms',11);
% companion side: Bus_McuStatus.valid = no MCU_STATUS frame for more than mcu_status_timeout (ICD 5)
E.can.mcu_status_timeout = 0.2;              % s
% companion_io HW (D38): /can2/rx and /can2/tx std_msgs/UInt8MultiArray, 69 B = id u32 LE, len u8, data 64 B;
% the subscriber is sampled at Ts.base and the external SocketCAN-FD bridge spaces its messages by >= hw_min_gap
E.can.hw = struct('msg_len',69,'min_gap',0.002,'qos_depth',16);

%% ---- T09 vehicle control, mode manager, supervisor, implement / aux, watchdog (build_fw_control) ----
E.safety.dcl_warn = 25;                      % A, BMS_WARN when DCL is below (ICD 6 bit 11); 1/6 of the 150 A DCL
E.safety.mask_rc = mk([2 3]);                % RC_LINK_LOST / RC_FAILSAFE: ignored in AUTO when rc_required_in_auto = false (D44)
dv = ones(1,32);                             % k_der candidate per active fault bit (1 = no derate)
dv(1+[11 14 20 24]) = [E.safety.derate_bms_warn E.safety.derate_tilt E.safety.derate_encl E.safety.derate_imu];
E.safety.derate_vec = dv;
E.safety.drv_fault_mask = 65535 - 2^10;      % ESC fault_code bits that make DRV_x_FAULT (b10 STO_ACTIVE excluded)
E.safety.esc_fault_state = 7;                % EscState FAULT
E.safety.monitor_bits = [5 6 9 28];          % frame timeouts evaluated from STANDBY on (no frames yet in BOOT / SELFTEST)
E.shaper.jw_stop = 30;                       % rad/s^3, yaw-axis jerk of the stop profiles (CTRL_STOP, ESTOP.DECEL)
E.lim.v_lat_min = 0.1;                       % m/s, floor of |v_ref| in the lateral limit |w| <= a_lat_max/max(|v_ref|, v_lat_min)
E.lim.v_follow_dv = 0.2;                     % m/s, OVERSPEED (bit 16) also when a track runs this far above its reference
                                             % for t_persist 0.2 s while driving (MANUAL..AUTO; runaway, R-01 / DRV-01,
                                             % 2026-11-15) [EST]. Since the descent task (2026-11-15) the reference of a
                                             % track is held at the lowest speed seen while the speed is above it, so a
                                             % regen-limited deceleration does not trip; a speed that rises again does
% Descent speed cap (Agents task descent; the user's decisions of 2026-11-15: always on, in every mode, by slope and fitted
% implement; no reversing with the tiller). P.structure.descent (single source, read only here; theta_bp = deg of the
% measured downhill body pitch in the direction of travel, 12 breakpoints since structure step 8 (2026-11-16), v_fwd /
% v_rev m/s, rows = implement code + 1, the steeper breakpoint between two) is the
% highest speed at which a brakes-locked stop may start. fw_vehicle_ctrl (subsystem Descent_cap) limits the speed target to
% ref_fwd / ref_rev; fw_safety_supervisor (subsystem Descent_trip) sets OVERSPEED (bit 16, no persistence) when the
% predicted body speed passes trip_fwd / trip_rev while the reference is steady or rising, or runs v_follow_dv above a
% falling reference; CTRL_STOP and ESTOP.DECEL then park at once. Rows of ref_* / trip_*: 1-4 implement code 0..3 (none,
% mower, tiller, sprayer), 5 an unknown implement (IMPL_FB received with another type than the coding plug): the smallest
% entry of all rows, the raised deck included, 6 the mower with its deck reported raised (v_fwd_raised / v_rev_raised; the
% user, 2026-11-15: the GNC keeps the deck down whenever the robot moves, the MCU applies these rows as a backstop). Entry
% cap >= v_top (the top speed, no cap below it): ref = trip = v_top + dv_top, so the speed target stays at
% k_der*v_track_max as before, below the trip; else ref = max(cap - dv, 0), trip = max(cap, safety.standstill_v). The
% table fields are copied from P.structure.descent when P is built, so a change there carries through
dsc = P.structure.descent;
dsc_top = floor(100*v_trk + 1e-9)/100;          % m/s, 1.41: an entry at or above it is the top speed (no cap below it)
dsc_dv = 0.10;                                  % m/s, the speed target sits this far below the cap, the trip [EST: about
                                                % twice the ESC lag of a 1.2 m/s^2 deceleration on the bench, 0.06 m/s]
dsc_dvt = 0.03;                                 % m/s, the trip at a top-speed entry sits this far above v_top: 1.44 m/s,
                                                % the lowest bound behind those entries (sprayer forward at 5 deg,
                                                % work/agentic/descent/structure/report_part2.md) [EST]
% mower deck reported raised: the lift potentiometer adc.lift_pos (0..1 over the deck height range
% P.structure.impl.mower.h_range, the h_meas of fw_implement_aux) at or above lift_raised, the fraction of deck height
% dsc_h_raised [EST: 0.09 m, above the default working height aux.deck_h_default / GNC mower_h 0.06 m, below the full lift
% 0.12 m of the structure analysis; the height where the gauge wheels leave the ground is in no parameter yet]
dsc_h_raised = 0.09;
dsc_hr = P.structure.impl.mower.h_range;
cf = [dsc.v_fwd; min([dsc.v_fwd; dsc.v_fwd_raised], [], 1); dsc.v_fwd_raised];
cr = [dsc.v_rev; min([dsc.v_rev; dsc.v_rev_raised], [], 1); dsc.v_rev_raised];
istop = @(c) c >= dsc_top - 1e-4;
dref = @(c) istop(c)*(dsc_top + dsc_dvt) + ~istop(c).*max(c - dsc_dv, 0);
dtrip = @(c) istop(c)*(dsc_top + dsc_dvt) + ~istop(c).*max(c, E.safety.standstill_v);
% t_hold   s, peak hold of the downhill pitch, so crests and bumps count [lead's brief]
% theta_imu deg, downhill pitch in both directions while the IMU sample is invalid (the 20 deg column) [lead's brief]
% t_pred   s, the trip predicts the speed this far ahead with the positive odometry acceleration a_track: the odometry
%          speed lags half its 0.05 s window, a_track its tau_acc 0.05 s filter, plus one sample and the DRV_CMD cycle to
%          the brake [EST; 0.04 s let the bench speed peak 0.023 m/s above the cap, T-EL-58]
% a_fall   m/s^2, a reference falling faster than this is a commanded deceleration [EST]
% dv_fall  m/s, a reference this far above the cap's speed target is a commanded deceleration too (the cap pulls it
%          down; the K_v 4 shaper tail falls below 0.1 m/s^2 within 0.025 m/s of its target) [EST]
% t_release s, hill start (descent task, step 9, 2026-11-16): the trip drops the t_pred term while a brake is engaged
%          and for t_release after both brakes released, while the reference in the direction of travel is at most
%          yaw.cmd_still_v; the roll-back while the speed loop takes over (S2: 0.09 m/s, a prediction of 0.175 m/s
%          against the 0.15 m/s reverse trip at a measured 20.5 deg) no longer trips, the measured speed still does [EST]
% ref_fwd / ref_rev, trip_fwd / trip_rev   m/s, 6 x numel(theta_bp) (6 x 12): the speed target limits and the trip speeds
E.lim.descent = struct('theta_bp', dsc.theta_bp, 'v_fwd', dsc.v_fwd, 'v_rev', dsc.v_rev, ...
    'v_fwd_raised', dsc.v_fwd_raised, 'v_rev_raised', dsc.v_rev_raised, 'v_top', dsc_top, ...
    'dv', dsc_dv, 'dv_top', dsc_dvt, 't_hold', 1.0, 'theta_imu', 20, 't_pred', 0.08, 't_release', 0.5, ...
    'a_fall', 0.1, 'dv_fall', 0.02, ...
    'h_raised', dsc_h_raised, 'lift_raised', (dsc_h_raised - dsc_hr(1))/(dsc_hr(2) - dsc_hr(1)), ...
    'ref_fwd', dref(cf), 'ref_rev', dref(cr), 'trip_fwd', dtrip(cf), 'trip_rev', dtrip(cr));
E.wdg.stall_1k_bit = 7;                      % SIM fault_inject bit: emulated hang of the 1 kHz kick (no kick while set)
E.wdg.age_lim = [0.02 0.06 0.2];             % s, alive-age limits [Task_10 Task_50 Task_100]; Task_50 0.06 (was 0.1) so a
                                             % stall stops the kick <= 110 ms after its injection (T-EL-34; normal age <= 49 ms)
E.aux.blink_fast = 2;                        % Hz (AUTO_START green, FAULT red, CTRL_STOP amber); blink_f = 1 Hz (STANDBY)
E.aux.moving_v = 0.05;                       % m/s, AUTO "moving" threshold of the 0.1 s / 2 s beep
E.aux.pump_flow_full = P.power.impl.sprayer.k_q*P.power.impl.sprayer.w_nom;   % L/min at pump duty 1
E.aux.lift_h_range = P.structure.impl.mower.h_range;    % m, mower deck height at lift_pos 0 / 1
E.aux.till_d_max = P.structure.impl.tiller.d_max;       % m, tiller depth at lift_pos 0 (0 at lift_pos 1)

%% ---- T17 hardware build setup (utilities/stm32/stm32_ioc_configure, stm32_target_sets, build_targets) ----
% Peripheral targets written into the CubeMX projects utilities/stm32/shaddad_h743.ioc (MCU) and
% shaddad_g474.ioc (BMS); prescalers / bit timings are derived from these values and the clock tree of
% the NUCLEO templates (docs/hw_bringup.md). Pin maps are frozen in stm32_ioc_configure (strings).
tg.mcu = struct('f_cpu',480e6,'flash',2*2^20,'ram',1*2^20, ...
    'tim2_f',1e6, ...                        % Hz, TIM2 32-bit free-running counter -> t_us (D60)
    'pwm_f',[20e3 25e3], ...                 % Hz, TIM1 CH1/CH2 (pump, lift), TIM4 CH1/CH2 (fans) (ICD 6)
    'i2c_f',400e3,'i2c_t_low',1.35e-6,'i2c_t_high',1.0e-6,'i2c_t_presc',25e-9,'i2c_t_scldel',0.4e-6, ...
    'i2c_t_sync',[0.15e-6 0.45e-6], ...      % s, SCL synchronisation + rise/fall (min, max) of the I2C1 timing check
    'can_sp',[0.875 0.8], ...                % sample point CAN1 (classic, CANopen) / CAN2 (CAN-FD, CiA 601-3)
    'fdcan2_ram_offset',1280, ...            % words, FDCAN2 message-RAM offset (FDCAN1 uses words 0..1279)
    'can_rx_fifo',[16 16],'can_tx_fifo',[8 16],'can_std_filters',[16 8], ...
    'ts_cal_addr',[535947296 535947328]);    % 0x1FF1E820 / 0x1FF1E840: TS_CAL1 (30 degC) / TS_CAL2 (110 degC), RM0433
tg.bms = struct('f_cpu',170e6,'flash',512*2^10,'ram',128*2^10, ...
    'can_sp',0.875,'spi_f_max',1e6, ...      % Hz, AFE isoSPI bridge limit (LTC6820 class)
    'adc_n',6,'di_n',8,'do_n',5);
% Task-load design estimate (spec electronics 2.7.1 mapped to the revision-B task set; replaced by the
% Execution Profiler / PIL measurement T-EL-42 once the board build runs). MCU Task_5 includes the
% blocking ICM-20948 read at 400 kHz (about 38 bytes x 22.5 us = 0.6 ms); BMS Task_10 includes the
% blocking AFE SPI transfer (64 B at 664 kbit/s = 0.77 ms).
tg.mcu.task_T    = [1e-3 5e-3 1e-2 5e-2 0.1];          % s, Task_1k, Task_5 (IMU), Task_10, Task_50, Task_100
tg.mcu.task_wcet = [25e-6 650e-6 380e-6 100e-6 50e-6]; % s, estimated WCET at 480 MHz
tg.bms.task_T    = [1e-2 5e-2 0.1 1];                  % s, bms_fast/afe, can_slow, bms_slow, bms_bal
tg.bms.task_wcet = [1.0e-3 60e-6 80e-6 200e-6];        % s, estimated WCET at 170 MHz
tg.cpu_load_max  = 0.5;                                % build plan T17 acceptance: MCU CPU load < 50 % at 480 MHz
tg.flash_max     = 2^20;                               % B, build plan T17 acceptance: MCU flash < 1 MB
E.target = tg;
end
