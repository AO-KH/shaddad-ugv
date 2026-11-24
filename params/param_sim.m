function S = param_sim()
%PARAM_SIM  Integration / simulation-framework parameters (owner: integration).
%   S = param_sim() returns a struct with fields sim, frames, viz, log, scen.
%   shaddad_params assigns them to P.sim, P.frames, P.viz, P.log, P.scen.
%   Pure function: no base-workspace access, no global rng, numeric/logical fields only.

%% ---- rates (binding allowed set, docs/architecture.md section 5) ----
sim.Ts_base   = 1e-3;                        % Simscape local solver, MCU base task, ESC, encoders
sim.Ts_allowed = [1e-3 5e-3 1e-2 5e-2 0.1 0.2 1.0];   % every task / port rate must be one of these
sim.Ts_rc_allowed = [0.007 0.014];           % SBUS frame periods, ONLY inside RC_Link_Sim (RT to 1 ms)
sim.Ts_compiled_allowed = [0 Inf sim.Ts_allowed sim.Ts_rc_allowed];   % T01 / T-INT-05 compiled-rate audit (D32)
sim.Ts_scen   = 0.01;                        % root scenario input (ZOH)
sim.Ts_monitor = 0.1;                        % Sim_Monitors, KPIs, collision check
sim.compute_delay = 1;                       % samples (GNC->MCU link)

%% ---- solver ----
sim.solver_id    = 1;                        % 1 = ode3 fixed step (P_meta.sim.solver_names)
sim.ref.RelTol   = 1e-4;                     % accuracy-reference config (daessc)
sim.ref.AbsTol   = 1e-6;
sim.ref.MaxStep  = 1e-3;
sim.ssc.Ts            = 1e-3;                % Simscape local solver step (Backward Euler, fixed cost)
sim.ssc.MaxNonlinIter = 3;
sim.ssc.MaxModeIter   = 2;
sim.ssc.tau_filt      = 2e-3;                % Simulink-PS input filter where derivatives are needed
sim.ssc.Ts_fault      = 1e-4;                % fuse / short-circuit harness tests only
sim.ssc.Ts_encl       = 0.1;                 % electronics-enclosure thermal network (own Solver Configuration)
sim.lambda_max_explicit = 1000;              % |lambda| limit [1/s] for Simulink continuous states with ode3 @ 1 ms

%% ---- interface stubs (scripts/build/build_stubs.m, T01) - values only used until a domain replaces its stub ----
sim.stub.J_spr      = 1.3;                   % kg*m^2, structure_plant stub: sprocket load inertia (each side, 155 kg x 0.09^2)
sim.stub.D_spr      = 50;                    % N*m*s/rad, structure_plant stub: sprocket load damping
sim.stub.J_impl     = 1e-3;                  % kg*m^2, structure_plant stub: implement shaft inertia
sim.stub.V_bus      = 46.8;                  % V, Power_Plant stub: ideal pack voltage (13S NMC nominal)
sim.stub.R_feed     = 10e-3;                 % Ohm, Power_Plant stub: feeder resistance to each DC48 _p port
sim.stub.D_impl     = 1e-4;                  % N*m*s/rad, Power_Plant stub: implement shaft damping (Impl_None)
sim.stub.R_esc      = 100;                   % Ohm, drive_system stub: ESC idle load across p/n (each side)
sim.stub.J_drv      = 0.02;                  % kg*m^2, drive_system stub: gearbox output inertia (each side)
sim.stub.D_drv      = 1;                     % N*m*s/rad, drive_system stub: gearbox output damping (each side)
sim.stub.T_amb      = 318.15;                % K, scalar T_amb stub value (45 C)

%% ---- run control ----
sim.seed_base = 20261018;
sim.seed = struct('gnss',1,'imu',2,'lidar2d',3,'lidar3d',4,'rc',6,'obstacles',7, ...
                  'terrain',8,'power',9,'adc',10,'spray',11);
sim.rtf_min_mission = 4;                     % S3 with the endurance fidelity set, accelerator (D56; re-baselined in T16)
sim.rtf_min_short   = 5;                     % S1 Phase 1, accelerator, LCC (target 10 with MinGW-w64)
sim.wall_timeout    = 1800;                  % s, sim(...,'Timeout') guard for batches
sim.stop_soc        = 0.05;                  % stop if true SOC < 5 %

%% ---- endurance fidelity set (S3 and long power runs; D56) - applied with SimulationInput.setVariable ----
sim.endurance = struct('VC_PACK',2,'VC_DRIVE_FID',3,'VC_STRUCT_FID',2,'VC_LIDAR',1,'VC_TEB',0,'VC_VIZ',0, ...
                       'lidar_div',2);       % LiDAR + perception every 2nd 0.1 s sample (5 Hz)

%% ---- Sim_Monitors (T16; integration spec 4.5, D22/D40/D42) ----
sim.mon.Kc        = 12;                      % nearest static cylinders checked per 0.1 s step
sim.mon.Kb        = 8;                       % nearest static boxes (incl. film segments as thin boxes)
sim.mon.Nd        = 4;                       % dynamic obstacles (ScenarioIn.obs)
sim.mon.footprint = [1.0 0.78];              % m, collision footprint L x W (P.structure.geom.L_overall / W_overall)
sim.mon.z_robot   = 1.1;                     % m, robot height incl. mast: obstacles entirely above it are ignored
sim.mon.film_thick = 0.02;                   % m, film segment -> thin box (D42)
sim.mon.far       = 1e6;                     % m, padding obstacle position / "no obstacle" separation
sim.mon.E_out_min_Wh = 1;                    % Wh, eps_el denominator floor (D40)
sim.mon.E_spr_min_J  = 1000;                 % J, eps_mech denominator floor (D40)
sim.mon.v_abs_max = 3;                       % m/s, Check Static Range on |v| (warning only)
sim.mon.T_cell_rng = [250 360];              % K, Check Static Range on the cell temperature (warning only)

%% ---- integration overrides applied by every scenario (utilities/scn_integration_overrides.m, T16) ----
% Found by the T16 system runs and reported to the owners (docs: T16 report, issues_for_other_tasks):
% the two ESCs draw about 7 W each from the DC link at standstill (drive_system idle loss, 0.30 A at 47 V),
% so through the 22 Ohm precharge resistor (P.power.pdu.R_pre) the bus settles at 87 % of the pack
% voltage and the BMS precharge (k_pre 0.97 within t_pre_max 1.5 s) fails: HV_Ready never rises. 4.7 Ohm
% leaves a 1.3 V drop (97.6 %); the inrush peak 53.8 V / 4.7 Ohm = 11 A is far below the fuse rating.
sim.ovr.R_pre = 4.7;                         % Ohm, scenario value of P.power.pdu.R_pre

%% ---- frames (binding, docs/architecture.md section 3) ----
frames.g          = 9.7876;                  % m/s^2, gravitywgs84(430, 24.15)
frames.mag_ENU_uT = [1.745 34.116 -26.317];  % WMM2025 @ 2026.8 at the site, uT (ENU)
frames.mag_dec_deg = 2.93;
frames.mag_dip_deg = 37.6;
frames.R_NED_ENU  = [0 1 0; 1 0 0; 0 0 -1];  % v_NED = R_NED_ENU * v_ENU
frames.R_FRD_FLU  = diag([1 -1 -1]);         % v_FRD = R_FRD_FLU * v_FLU
frames.imu_g_internal = 9.81;                % gravity assumed inside mspsensorlib/IMU (verify at build)

%% ---- visualisation / logging ----
viz.enable     = 0;
viz.Ts         = 0.1;
viz.trail_len  = 20000;
viz.scan_decim = 4;
viz.footprint  = [1.0 0.78];
viz.Ts3d       = 0.1;                        % s, live 3-D view (shaddad.Robot3DView, VC_VIZ 3), 10 Hz
viz.camera     = 1;                          % 3-D view camera: 1 chase, 2 side, 3 top, 4 free (mouse)

log.Ts_mission        = 0.1;
log.Ts_test           = 0.01;
log.to_file_threshold = 600;                 % s StopTime above which logging goes to file
log.sdi_archive_runs  = 10;

%% ---- scenario defaults ----
scen.soc0              = 0.95;
scen.stop_on_collision = true;
scen.stop_on_mission_done = true;            % Sim_Monitors stops the run when gnc_status.mission_done (S3 repeats: false)
% Operator_Console dashboard (VC_OPSRC 2) initial values: a Bus_OperatorCmd struct (same field order) with the
% transmitter ON, sticks neutral, disarmed, MANUAL.
scen.dash = struct('stick_thr', 0, 'stick_steer', 0, 'stick_aux_lv', 0, 'stick_aux_lh', 0, 'knob_speed', 1, ...
    'sw_mode', 1, 'sw_arm', 0, 'sw_impl', 1, 'sw_lift', 2, 'sw_lights', 0, 'sw_estop', 0, 'btn_reset', 0, ...
    'sw_mission', 1, 'btn_rth', 0, 'tx_power_on', 1);
% Live RC drive with a real transmitter in USB joystick mode (VC_OPSRC 3, Simulation 3D Joystick block,
% utilities/joy_map.m). EdgeTX/OpenTX "Classic" joystick mode sends CH1-CH8 as axes 1-8 (-1..1) and CH9 and
% up as buttons 1, 2, ... (pressed while the channel is high). Check the mapping with phase1_joystick_check.
%   ax  : axis index for [steer thr aux_lv aux_lh mode arm speed impl] (CH1..CH8; 0 = not used)
%   inv : sign per axis entry (-1 reverses the axis)
%   btn : button index for [reset lights estop] (CH9, CH10, CH12; 0 = not used)
%   sw3 : threshold of the 3-position switch axes (mode, impl): < -sw3 position 1, > +sw3 position 3
scen.joy = struct('id', 1, 'profile', 1, 'ax', 1:8, 'inv', ones(1, 8), 'btn', [1 2 4], 'sw3', 0.33);
% profile 2: PS5 DualSense gamepad (DirectInput on Windows: 6 axes LX LY RX L2 R2 RY, 15 buttons
% 1 Square 2 Cross 3 Circle 4 Triangle 5 L1 6 R1 7 L2 8 R2 9 Create 10 Options 11 L3 12 R3 13 PS 14 Touchpad).
% Its buttons spring back, so joy_map latches them: Cross toggles arm, Circle latches the E-stop (and disarms),
% Triangle (held) resets, R1 / L1 step the speed scale 25-50-75-100 %, Square toggles the lights, Create the
% implement, Options MANUAL / HOLD; L2 / R2 lift the implement up / down. Left stick up/down = throttle,
% right stick left/right = steering. dz = stick dead zone.
scen.joy.pad = struct('ax_thr', 2, 'inv_thr', -1, 'ax_steer', 3, 'inv_steer', 1, 'ax_lift_up', 4, 'ax_lift_dn', 5, ...
    'dz', 0.08, 'b_arm', 2, 'b_estop', 3, 'b_reset', 4, 'b_lights', 1, 'b_impl', 9, 'b_spd_up', 6, 'b_spd_dn', 5, ...
    'b_hold', 10);
scen.T_amb_hot         = 323.15;
scen.world_id          = 1;                  % 1 flat_test (see param_world)

S = struct('sim', sim, 'frames', frames, 'viz', viz, 'log', log, 'scen', scen);
end
