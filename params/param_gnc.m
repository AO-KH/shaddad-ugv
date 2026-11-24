function G = param_gnc(P)
%PARAM_GNC  Guidance, path planning / following, mission manager (owner: gnc).
%   G = param_gnc(P)   P: partially assembled (needs P.structure, P.power, P.electronics, P.sensors)
%   Variant control derived by shaddad_params: VC_ROW_TRACKER (1 Adaptive MPC | 2 Stanley).
%   The MPC object mpcRowObj is created in base by scripts/gnc_designMPC.m (not stored in P).
%   P.gnc.mission (Bus_MissionPlan struct) is written by scripts/gnc_compileMission.m; until then
%   shaddad_params fills it with the zero struct (valid = false). It is read by a Constant INSIDE
%   Shaddad_GNC (no root inport, D57). shaddad_startup creates mpcRowObj when VC_ROW_TRACKER == 1 (D58).
%   Revision B: command envelope derived from the MCU limits (D43), slope-dependent turn limits (D53),
%   greenhouse inset clears the wall posts (D54), MPC model with input dead time (D41), palm working
%   speed 1.0 m/s from the shift budget (D46), vegetation speed limit, TEB variant VC_TEB (D56).
%   RC-loss behaviour in AUTO is owned by the MCU (P.electronics.safety.rc_required_in_auto, D44).
deg = pi/180;
EL = P.electronics;                          % MCU limits are binding for every GNC command (D43)

%% ---- timing ----
G.Ts = 0.05; G.Ts_avoid = 0.1; G.Ts_teb = 0.2; G.Ts_status = 0.2; G.Ts_plan = 1.0;

%% ---- geometry (single source: P.structure) ----
Lb = P.structure.geom.L_overall; Wb = P.structure.geom.W_overall;
G.geom = struct('L',Lb,'W',Wb,'B_track',P.structure.geom.B,'x_front',Lb/2, ...
                'r_circ',hypot(Lb/2, Wb/2));

%% ---- path window ----
% dummy: straight 10 m path served while P.gnc.mission is invalid (Path Smoother Spline rejects
% coincident poses); columns x, y, theta_deg, dir as Bus_MissionPlan.path
G.path = struct('N_raw',100,'N_smooth',400,'S_max',512,'seg_end_tol',0.05, ...
    'dummy',[linspace(0,10,100)' zeros(100,1) zeros(100,1) ones(100,1)]);

%% ---- speed planning / governor ----
G.spd = struct('v_max',min(2.0, EL.lim.v_max_auto),'v_work_palm',1.0,'v_work_greenhouse',0.5,'v_work_solar',0.8, ...
    'v_transit',min(1.5, EL.lim.v_max_auto),'v_turn_max',0.5,'v_dock',0.2,'a_acc',0.5,'a_dec',EL.shaper.a_dec_auto,'jerk',1.0,'a_lat',0.3, ...
    'obs_d',[0 1.5 2 3 4 6],'obs_v',[0 0 0.6 1.2 1.6 2.0], ...
    'xte_x',[0 0.1 0.3 0.5],'xte_v',[2 2 0.5 0.3], ...
    'head_x',[0 10 30 60]*deg,'head_v',[2 2 0.5 0.2], ...
    'pitch_x',[0 10 15 20 25]*deg,'pitch_v',[2 1.2 0.8 0.5 0], ...
    'roll_x',[0 8 12 15 18]*deg,'roll_v',[2 1.0 0.6 0.3 0], ...
    'nav_x',[0 0.05 0.10 0.30],'nav_v',[2 2 0.5 0],'soc_low',0.15,'v_soc_low',0.6,'t_lat',0.35, ...
    'v_prof_margin',0.99, ...                % Velocity Profiler start / end speed <= margin * sqrt(a_lat/max|kappa|)
    'veg_x',[0 1.5 2.5 3.0],'veg_v',[2 2 0.6 0.5]);   % vegetation density (3*spray.density) -> v limit (mower power, D46)

%% ---- path followers ----
G.pp   = struct('kappa_max',2.5,'Ld_v',[0 0.3 0.6 1.2 2.0],'Ld',[0.6 0.8 1.0 1.5 2.0]);
% Stanley: the drivingvehiclecontroller block computes delta = (theta_ref - theta) + atan(k_b e_f / (v + 1 m/s)) with the
% error e_f at CurrPose + Wheelbase (front axle). T14 feeds CurrPose = base pose - Lv [cos sin] (so e_f is the base-point
% error of spec 2.4) and k_b = k (1 + v_nom)/v_nom (exact spec law at v_nom; softer below, stiffer above)
G.stan = struct('k',0.8,'Lv',P.structure.geom.L/2,'delta_max_deg',60,'v_nom',0.8);   % virtual wheelbase = half contact length
G.stan.k_block = G.stan.k*(1 + G.stan.v_nom)/G.stan.v_nom;
G.mpc  = struct('Ts',0.05,'p',40,'m',8,'tau_r',0.25,'W_ov',[1 0.1],'W_dmv',1.0,'scale_ov',[0.1 0.1], ...
                'u_min',-1.0,'u_max',1.0,'du_max',0.1,'ud_gain',0.04,'noise',[0.15 0.05], ...
                'w_ey',0.7,'w_epsi',0.01, ...        % white UD channels (jumps of e_y / e_psi, e.g. row entry); tuned with the D41 delay
                'epsi_max',0.25, ...                 % rad, soft bound on e_psi (approach angle on row entry, ECR 1)
                'v_min_model',0.2,'n_delay',2, ...   % 2 x 0.05 s input dead time (GNC -> ESC ~0.06 s + MCU shaper ~0.05 s, D41)
                'v_design',0.8, ...                  % speed of the nominal model used by gnc_designMPC
                'v_engage',0.2);                     % MPC omega scaled by min(1, |v|/v_engage): no pivoting on the row at standstill
G.mpc.nx = 3 + G.mpc.n_delay;                % prediction-model states [e_y e_psi r u(k-1) u(k-2)] (Bus_MpcPlant dims)
G.mpc.nu = 5;                                % model inputs [r_cmd (MV) kappa_ref (MD) d w_y w_psi (UD)]
% path analyser (MATLAB Function gnc_pathAnalyzer): monotonic local search window, curvature preview
G.pa = struct('N',G.path.N_smooth,'n_back',5,'n_fwd',60,'p',G.mpc.p,'Ts',G.Ts,'v_prev_min',0.2, ...
              'd_valid',2.0, ...
              't_look',0.6);                 % s, v_ref read at s + v t_look (dead time + MCU shaper + drive lag)
G.align = struct('enter',45*deg,'exit',8*deg,'K',1.5,'w_max',0.6,'pitch_inhibit',10*deg,'roll_inhibit',10*deg);
G.cmd   = struct('w_max_v',[0 0.5 1 2],'w_max',[min(1.2,EL.lim.w_max_auto) 1.0 0.6 0.3],'w_rate',2.0, ...
                 'v_track_max',EL.lim.v_track_max,'chi',EL.mix.chi,'B',EL.mix.B, ...   % |v| + |w|*chi*B/2 <= v_track_max
                 'v_min_reverse',-EL.lim.v_rev_auto, ...
                 'w_tilt_x',[0 8 12 16 20]*deg,'w_tilt',[1.2 1.2 0.6 0.3 0.2], ...  % omega_max vs max(|pitch|,|roll|) (D53)
                 'accel_limit',max(G.spd.a_acc, G.spd.a_dec), ...                 % Bus_GncCmd.accel_limit (MCU ramp cap)
                 'beacon_by_mode',[0 0 1 2 3]);       % beacon_cmd per GncModeReq 0 STOP 1 MANUAL_ACK 2 AUTO 3 HOLD 4 FAULT_STOP

%% ---- downhill speed cap (Agents task descent, 2026-11-15; the user: always active, AUTO plans within it) ----
% Single source P.structure.descent (structure, work/agentic/descent/structure/report_part2.md): the highest speed at
% which a brakes-locked stop may start without tipping, rows = implement_type + 1 (0 none, 1 mower, 2 tiller, 3 sprayer
% full tank), columns = theta_bp of downhill pitch in the direction of travel; between breakpoints the steeper one.
% SpeedGovernor: v_pitch = min(LUT(|pitch|), gnc_descentLimit) = the cap at the nav pitch held over t_hold and a
% look-ahead along the planned path (DEM chord pitch, braking curve at a_la). CommandShaping: gnc_descentEnvelope scales
% v (and omega with it) to the cap at the nav pitch, forward and reverse (the tiller: no reverse at any slope).
% tol: a pitch within it above a breakpoint counts as that breakpoint (an exact 20 deg ramp is 20 deg + 1e-15 rad in
% floating point). t_hold: as the MCU cap (report_part2 section 4). a_la: half of a_dec_auto, for the slope's own pull
% and a limited regen. t_resp: 0.1 s dead time (D41), MCU shaper 1/K_v 0.25 s, drive lag 0.35 s (bench tau_v) and their
% tail; the sprayer down the world-2 fillet at 0.8-1.2 m/s, planned 0.20 m/s (0.30 - dv) from 15 deg, enters that class
% at 0.54-0.56 m/s with t_lat (0.35 s), 0.23 with 1.5 s, 0.203 with 2.0 s (work/agentic/descent/gnc/lookahead_sim.py)
Ds = P.structure.descent;                    % every table entry below is read from it, none is copied as a number
G.desc = struct('theta_bp',Ds.theta_bp*deg,'v_fwd',Ds.v_fwd,'v_rev',Ds.v_rev, ...
    'v_fwd_raised',Ds.v_fwd_raised,'v_rev_raised',Ds.v_rev_raised, ...   % mower with the deck raised (nose-heavy)
    'tol',1e-9, ...                          % rad, breakpoint tolerance
    't_hold',1.0, ...                        % s, the nav pitch is held at its peak this long (crests, bumps) [EST]
    'L_c',P.structure.geom.L, ...            % m, track contact length: predicted body pitch = DEM chord over it
    'ds',0.25, ...                           % m, look-ahead sample spacing (the DEM resolution of every world)
    'a_la',0.5, ...                          % m/s^2, deceleration the look-ahead plans with [EST]
    't_resp',2.0);                           % s, from a speed-command change until the robot's speed follows it [EST]
% margin below the cap = the MCU's (D43: the MCU limits bind every GNC command): fw_vehicle_ctrl limits its speed target to
% cap - P.electronics.lim.descent.dv and trips at the cap, so AUTO plans to the MCU's target and reaches each steeper
% class there, with nothing left to brake on the steeper part (0 while the MCU has no such field)
G.desc.dv = 0;
if isfield(EL.lim, 'descent') && isfield(EL.lim.descent, 'dv'), G.desc.dv = EL.lim.descent.dv; end
G.desc.n_hold = max(1, round(G.desc.t_hold/G.Ts));
% look-ahead length: the braking curve from v_max down to a cap of 0 after t_resp, plus one sample (4.25 m)
G.desc.d_look = G.desc.ds*(ceil((G.spd.v_max*G.desc.t_resp + G.spd.v_max^2/(2*G.desc.a_la))/G.desc.ds) + 1);
G.desc.n_look = round(G.desc.d_look/G.desc.ds) + 1;
G.desc.no_rev = double(all(Ds.v_rev <= 0, 2))';   % implements that may not reverse at any slope (1: the tiller)

%% ---- obstacles / local avoidance ----
G.obs = struct('d_stop',1.5,'d_release',2.0,'w_c',Wb/2 + 0.15,'L_c',6.0,'t_clear',2,'t_bypass',10, ...
               't_hold_max',120,'t_mower_off',3);
G.vfh = struct('NumAngularSectors',180,'DistanceLimits',[0.3 8],'HistogramThresholds',[3 10], ...
    'RobotRadius',G.geom.r_circ,'SafetyDistance',0.3,'MinTurningRadius',0.5,'TargetDirectionWeight',5, ...
    'CurrentDirectionWeight',2,'PreviousDirectionWeight',2);
G.teb = struct('Ts',0.2,'map_window',10,'map_decim',2,'LookAheadTime',4,'GoalTolerance',[0.1 0.1 0.087],'MaxVelocity',[0.6 0.6], ...
    'MaxAcceleration',[0.5 1.0],'MinTurningRadius',0.5,'ReferenceDeltaTime',0.3,'NumIteration',3, ...
    'Time',10,'Smoothness',1000,'Obstacle',50,'Dimension',[Lb Wb],'ObstacleSafetyMargin',0.3);

%% ---- Hybrid A* replanning ----
G.hastar = struct('R_min',1.5,'mpl',1.2,'n_prim',5,'cost_rev',5,'cost_switch',10,'aei',3,'interp',0.25, ...
                  'r_infl',G.geom.r_circ + 0.06,'max_nodes',20000,'t_budget',1.5,'rejoin_off',4.0,'retries',3);
G.map = struct('local_res',10,'local_N',300,'static_res',4,'static_N',400);

%% ---- mission manager ----
G.fsm = struct('fix_min',4,'sigma_auto',0.05,'sigma_pause',0.15,'t_nav_lost',0.3,'t_nav_degr',2.0, ...
    't_nav_rec',5.0,'t_mcu_timeout',0.2,'xte_fault',0.5,'t_xte',3.0,'t_stuck',5.0,'v_stuck_cmd',0.3, ...
    'v_stuck_meas',0.05,'pitch_fault',25*deg,'roll_fault',18*deg,'geofence_margin',0.5, ...
    'soc_start_min',0.35,'soc_reserve',0.20,'soc_critical',0.10,'k_tort',1.4,'e_m',0.12,'k_margin',1.3, ...
    'E_nom_Wh',P.power.pack.E_Wh,'auto_resume_nav',true, ...
    'resume_xte',0.3,'resume_epsi',30*deg,'t_impl_ready',10,'dock_tol_xy',0.03,'dock_tol_yaw',2*deg, ...
    'sigma_degr',0.10,'sigma_lost',0.30,'v_odo_tol',0.3, ...   % InputConditioning: T7 cross-track off / lost sigma, odometry
    'q_good',4);                             % NavQuality >= 4 (RTK_FIXED, LIDAR_LOCALIZED) is not degraded
% T7 sigma term of nav_degraded (InputConditioning, S4 fix 2026-11-14, Relays): the position sigma across the lane
% (sqrt(n'Cn), n the normal of the path heading psi_ref) is degraded above sigma_pause and clear again below sigma_degr;
% the sigma along the lane (sqrt(t'Ct)) above sigma_along, clear again below sigma_along_off. In the S4 rows (nav round
% 2, 100 s) the honest sigmas are across 0.050 m median / 0.058 max (NEES 0.55) and along 0.078 median / 0.091 p99 /
% 0.093 max (NEES 1.66): sigma_along is 2.7 x that maximum (the cross-track 0.15 is 2.6 x 0.058) and 0.05 m below
% sigma_lost, so a lost along-track position pauses (P2 can resume) before nav_ok drops (NAV_INVALID, FAULT); the
% 0.05 m band is the cross-track one. At the row ends the headland turn runs across the aisle, where the along-aisle
% sigma is the cross-track one and pauses above 0.15 m
G.fsm.sigma_along = 0.25;                    % m, T7 along-track sigma: degraded above
G.fsm.sigma_along_off = 0.20;                % m, T7 along-track sigma: clear again below
% MissionManager stub (T14; replaced by gnc_missionFSM in T15): per SegType 0 NONE 1 TRANSIT 2 ROW 3 TURN
% 4 DOCK 5 TERMINAL -> TrackerSel, AutoSub and mode speed cap
G.stub = struct('trk_by_type',[0 1 2 1 1 0],'sub_by_type',[0 2 3 4 9 0], ...
    'vcap_by_type',[0 G.spd.v_max G.spd.v_max G.spd.v_turn_max G.spd.v_dock 0]);

%% ---- implements (widths from P.structure.impl) ----
G.impl = struct('mower_w',P.structure.impl.mower.w,'mower_h',0.06,'mower_speed',P.structure.impl.mower.w_nom, ...
    'till_w',P.structure.impl.tiller.w,'till_depth',P.structure.impl.tiller.d_nom, ...
    'till_speed',P.structure.impl.tiller.w_nom,'till_lift_lead',0.5,'till_lower_lag',0.3, ...
    'boom_w',1.2,'n_sect',8,'R_app',300,'v_min',0.3);

%% ---- coverage presets (numeric; geometry from P.world scenarios) ----
% greenhouse inset (D54): c_in = gh_post_edge + gh_c_clear + gh_c_xte + max(0, W/2 - w_tool/2) = 0.20+0.15+0.05+0.05 = 0.45 m
G.cov = struct('gh_swath',P.structure.impl.mower.w,'gh_sidelap',0.10,'gh_R',0.81,'gh_skip',1, ...
    'gh_post_edge',0.20,'gh_c_clear',0.15,'gh_c_xte',0.05, ...
    'palm_spacing',8.0,'palm_R',2.0,'palm_halfwidth',2.5,'solar_pitch',7.0,'solar_R',1.5, ...
    'slope_turn_max',10*deg, ...             % no headland turns / Dubins bulbs where the terrain slope exceeds this
    'wall_clear',0.20, ...                   % m, footprint clearance kept to walls / posts in headland turns (S4 0.15 + XTE 0.05)
    'clear_min',0.15, ...                    % m, minimum footprint clearance to P.world.occ accepted by the compiler
    'clear_keepout',0.30, ...                % m, body-edge clearance to JSON keep-outs (W/2 + 0.30 from the pose)
    'dock_approach',3.0,'terminal_len',2.0, ...   % m, straight dock approach, terminal extension (window B only)
    'raster',0.05,'slope_step',1.0, ...      % m, coverage raster; lane shortening step for slope-limited turns
    'mower_x',P.structure.impl.mower.pos(1),'mower_len',P.structure.impl.mower.dims(1), ...   % deck centre ahead of R, deck length
    'via_tol',0.02, ...                      % m, coincident-waypoint tolerance
    'entry_Rs',10.0,'entry_dstep',0.05,'entry_dmax',0.5);  % lane lead-in S-curve (radius, shift step / max) when a
    % forward entry would swing the footprint closer than clear_min to a wall; Rs >= 10 m keeps the deck corner
    % (1.2 m ahead, 0.525 m to the side) on the safe side of the post line along the whole S

%% ---- metrics ----
G.met = struct('rms_window',200);
% GNC test bench tests/gnc/tb_gnc (spec 8): kinematic plant with first-order v / yaw-rate lags, input
% dead time and the MCU AUTO shaper (P.electronics.shaper), RTK noise; values are bench-only.
% dead time 0.1 s (D41) = 0.05 s Rate Transition 0.05 -> 0.01 (MCU Task_50 -> Task_10) + n_dead x 0.01 s
% + 0.01 s Rate Transition 0.01 -> continuous kinematics
G.tb = struct('tau_v',0.35,'tau_r',0.30,'n_dead',4, ...
    'sig_pos',0.015,'sig_yaw',0.3*deg,'seed_x',14001,'seed_y',14002,'seed_psi',14003, ...
    'drift',0,'x0',[0 0 0],'tilt',[0 0],'soc',0.8);   % drift m/s (+ = to the left of the heading); tilt [pitch roll] rad
G.variant = struct('row_tracker',1,'teb',1);   % VC_ROW_TRACKER, VC_TEB (0 = no TEB: endurance / rapid accelerator)

%% ---- build task T15: mission manager, local avoidance, replanning, implements, status ----
% Mission FSM (gnc_missionFSM, spec 4.1). The FSM gets the single sub-struct P.gnc.fsm (build rule 2), so the
% values it needs from obs / align / spd / hastar / path are copied here (single source = the lines above).
G.fsm.Ts = G.Ts;
G.fsm.auto_start = false;                    % true: IDLE treats (rc AUTO, drive_ready, plan, nav, SOC ok) as START
                                             % (bench option only; the deployed FSM needs MissionCmd START, T1)
G.fsm.t_geofence = 0.2;                      % s, geofence breach persistence (FAULT GEOFENCE within 0.5 s, T16)
G.fsm.t_vfh = 0.2;                           % s, VFH SteerDir NaN persistence before OBSTACLE_HOLD (2 scans)
G.fsm.t_scan_lost = P.sensors.per.protNStale*P.sensors.lidar.Ts;   % s (0.2), the scan is lost after more than
                                             % protNStale consecutive invalid scans (Bus_Perception.scanValid false): the
                                             % protective field's stale rule (Shaddad_Perception ProtField). Lost: T5 / R6
                                             % OBSTACLE_HOLD, HoldReason SENSOR_LOST, implement latch; a missed scan is
                                             % no loss, but no hold releases on an invalid scan (critfix R-02)
G.fsm.t_docked = 2.0;                        % s, DOCKED -> IDLE without a charging flag (R5)
G.fsm.t_drive_lost = 0.5;                    % s, T36: drive_ready false this long -> PAUSE. A parked ESC unparks through
                                             % ENABLING (P.drive t_premag + t_release = 0.09 s, drive spec 2.9), which the
                                             % 50 ms MCU status shows as a 0.1-0.15 s drop: no pause for that. With the RC
                                             % link lost T36 fires at once (MCU failsafe, D44)
G.fsm.no_rev = G.desc.no_rev;               % H2: no TEB bypass (TEB may plan reverse) with such an implement (descent)
G.fsm.desc = G.desc;                         % the raised-deck rows for v_cap while the mower moves with the deck up
% Mower deck (the user, 2026-11-15: the deck is down whenever the robot moves; raised, the robot is nose-heavy). The GNC
% commands it to impl.mower_h in every state and never raises it. No deck report reaches the GNC (Bus_McuStatus has
% none), so the FSM counts the deck down once the MCU could have driven it there: the MCU moves the lift toward
% deck_height_cmd only in its AUTO mode (requested in AUTO_MISSION / RETURN_TO_HOME), with a saturated P law (aux.lift_Kp,
% at most aux.lift_duty_max*h_rate); the operator may move it in MANUAL; its height at power-up is unknown. t_deck: from
% the top stop to within deck_tol of mower_h, plus 3 lift lags and 0.5 s for AUTO_START -> AUTO [EST]: 6.35 s
mw = P.structure.impl.mower;  ax = EL.aux;
G.fsm.deck_tol = 0.01;                       % m, the deck counts as lowered within this of mower_h [EST]
e0 = mw.h_range(2) - G.impl.mower_h;  eSat = ax.lift_duty_max/ax.lift_Kp;
G.fsm.t_deck = max(0, e0 - eSat)/(ax.lift_duty_max*mw.h_rate) + log(min(e0, eSat)/G.fsm.deck_tol)/(ax.lift_Kp*mw.h_rate) ...
    + 3*mw.h_tau + 0.5;                      % s, 0.5 s = AUTO_START -> AUTO [EST]
G.fsm.deck_t0 = 0;                           % s of deck lowering credited at power-up: 0 = height unknown (a bench without
                                             % a deck sets t_deck: down)
G.fsm.d_stop = G.obs.d_stop; G.fsm.d_release = G.obs.d_release;
G.fsm.t_clear_eff = G.obs.t_clear - G.Ts_avoid;   % H1 clear timer: t_clear minus the perception -> 0.05 s
                                             % Rate Transition latency (one 0.1 s scan), so the robot resumes
                                             % t_clear after the obstacle physically left the scan (T6)
G.fsm.t_bypass = G.obs.t_bypass; G.fsm.t_hold_max = G.obs.t_hold_max; G.fsm.t_mower_off = G.obs.t_mower_off;
G.fsm.align_enter = G.align.enter; G.fsm.align_exit = G.align.exit; G.fsm.align_tilt = G.align.pitch_inhibit;
G.fsm.v_transit = G.spd.v_transit; G.fsm.v_row = G.spd.v_max;
G.fsm.v_turn = G.spd.v_max;                  % TURN: the plan's per-segment turn speed (seg v_seg_max, compiled from the
                                             % mission speed_turn_max, default spd.v_turn_max) governs, not a second cap
G.fsm.v_dock = G.spd.v_dock; G.fsm.v_bypass = G.teb.MaxVelocity(1);
G.fsm.rejoin_off = G.hastar.rejoin_off; G.fsm.retries = G.hastar.retries; G.fsm.x_front = G.geom.x_front;
G.fsm.teb_t_bad = 15*G.Ts_teb;               % S4: TEB infeasible for 15 consecutive 0.2 s calls (3 s): after
                                             % enabling, TEB needs up to ~2.4 s (12 calls) to push the band off an
                                             % obstacle on its reference (LastFeasibleIdx 1, measured), so 3 calls abort
                                             % every bypass
G.fsm.teb_lfi_min = 3;                       % S4: LastFeasibleIdx below this is infeasible
G.fsm.N_raw = G.path.N_raw; G.fsm.seg_end_tol = G.path.seg_end_tol;
G.fsm.replan_end_tol = 0.10;                 % m, end of a replanned path reached (R3, S3, S5)
G.fsm.nav_mode_gh = 2;                       % Bus_NavCmd.navMode for GREENHOUSE_COVERAGE (LIDAR_PRIOR_MAP, T12 note)
% Local avoidance (spec 2.6): path-following corridor. The beam points are tested against a tube of
% half-width obs.w_c around n_tube path samples spaced L_c/(n_tube - 1) ahead of the closest point
% (window A then B), d_obs = along-path distance of the nearest sample in reach minus x_front. A body-
% frame box would see the greenhouse walls ahead in every headland turn and curved transit (T15 check).
G.obs.n_tube = 31;
G.obs.x_f = G.geom.x_front;
G.obs.s_tube = (0:G.obs.n_tube - 1)*(G.obs.x_f + G.obs.L_c)/(G.obs.n_tube - 1);   % sample offsets from the robot centre
G.teb.ref_decim = 4;                         % RefPath = window rows 1:4:end (band initialisation spacing, T15)
G.obs.n_beams = 360;                          % Bus_Perception.ranges_B / angles_B length
G.obs.vfh_Ld = G.pp.Ld; G.obs.vfh_Ld_v = G.pp.Ld_v;   % VC_TEB = 0: VFH-steered pursuit omega = 2 v sin(steer)/Ld(v)
% Hybrid A* (gnc_globalReplanner, spec 4.4): LOCAL (rejoin / bypass) plans use a tighter turn radius and
% a lane mask so the detour stays within the lane half-width (T7: deviation <= 2.5 m); GLOBAL plans (RTH, and
% the fallback of a rejoin after a pause whose LOCAL plan failed; never for a bypass or a rejoin resumed from a
% paused bypass, which stay in the lane) use R_min on the static occupancy grid window of start / goal + win_margin.
G.hastar.R_min_local = 1.0; G.hastar.mpl_local = 1.0;
G.hastar.r_infl_min = G.geom.W/2 + 0.10;     % fallback inflation when the start / goal is inside r_infl (walls)
G.hastar.win_margin = 10;                    % m, GLOBAL window margin around start and goal
G.hastar.vdist = 0.1;                        % m, validator ValidationDistance
G.hastar.kind_local = 0; G.hastar.kind_global = 1; G.hastar.N_raw = G.path.N_raw;
G.hastar.slope_turn_max = G.cov.slope_turn_max;   % no turning primitives / Dubins arcs where the DEM slope exceeds this (D53)
G.hastar.slope_head_tol = 10*deg;             % on such terrain only headings within this of the fall line are valid
G.hastar.slope_base = G.geom.L;               % m, the DEM slope is averaged over the footprint length (vehicle attitude)
G.fsm.kind_local = G.hastar.kind_local; G.fsm.kind_global = G.hastar.kind_global;
% Implement manager (spec 2.9)
G.impl.Q_k = 0.006;                          % Q [L/min] = R_app [L/ha] * v [m/s] * w_boom [m] * 0.006
G.impl.sect_all = 2^G.impl.n_sect - 1;       % all boom sections
% Status metrics
G.met.v_eta_min = 0.1;                       % m/s, ETA speed floor
% Mission bench tests/gnc/tb_gnc_mission (T15; harness LiDAR gnc_harnessLidar, scenario tables). Rows of
% the scenario tables are [t values...], zero-order hold:
%   scn_mcu   [t rc_mode_sw rc_link_ok drive_ready estop charging impl_mode freeze]
%             charging 0 off / 1 on / 2 when within dock_tol of plan.dock and stopped; impl_mode 0 auto
%             (implement_ready after t_spin of any implement enable) / 1 false / 2 true; freeze 1 holds the
%             heartbeat (MCU status frozen)
%   scn_cmd   [t cmd seq]      a new MissionCmd at t when seq changes (Bus_MissionCmd.cmd_seq)
%   scn_bms   [t SOC bms_critical]   SOC linearly interpolated
%   scn_nav   [t navValid sigma quality dx dy]   sigma 0 = nominal, dx/dy position offset (geofence push)
%   scn_plant [t v_factor]     1 normal, 0 = tracks blocked (stuck)
%   dyn       [type x y a b yaw t_on t_off] dynamic obstacles (type 0 none, 1 circle r = a, 2 box a x b)
%   scn_scan  [t valid]       scan validity; 0 = a blank scan as Perception publishes a LiDAR loss (scanValid false,
%             ranges NaN, protectiveStop false)
G.tb.t_spin = 2.0; G.tb.lidar_on = 1; G.tb.lidar_rmax = 20; G.tb.prot_static = 0; G.tb.dock_tol = 0.10;
G.tb.spray_mask = 255; G.tb.spray_density = 0;
G.tb.scn_mcu = [0 2 1 1 0 0 0 0; 1e5 2 1 1 0 0 0 0];
G.tb.scn_cmd = [0 0 0; 1e5 0 0];
G.tb.scn_bms = [0 G.tb.soc 0; 1e5 G.tb.soc 0];
G.tb.scn_nav = [0 1 0 4 0 0; 1e5 1 0 4 0 0];
G.tb.scn_plant = [0 1; 1e5 1];
G.tb.scn_scan = [0 1; 1e5 1];
G.tb.dyn = zeros(8, 8);
end
