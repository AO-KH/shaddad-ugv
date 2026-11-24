function d = buses_gnc()
%BUSES_GNC  Guidance / mission / companion-command buses (owner: gnc).
%   World ENU, yaw CCW from East, cross-track error e_y > 0 when the robot is LEFT of the path.
E = @bus_el;
P_MPC = 40;   % MPC prediction horizon = P.gnc.mpc.p (curvature preview length)
NX_MPC = 5;   % MPC prediction-model states = P.gnc.mpc.nx = 3 + n_delay: [e_y e_psi r u(k-1) u(k-2)] (D41)
NU_MPC = 5;   % MPC model inputs = P.gnc.mpc.nu: r_cmd (MV), kappa_ref (MD), drift d, e_y jump, e_psi jump (UD)
N_VLIM = 12;  % speed-governor MinMax inputs (spec 2.5 + vegetation limit, D46)
N_TEB = 50;   % TEB map crop cells = P.gnc.teb.map_window * P.gnc.map.local_res / P.gnc.teb.map_decim (T15)
d = bus_def('Bus_GncCmd', 0.05, 'Shaddad_GNC/CommandShaping (packed by Companion_IO into CAN2 0x100 COMP_CMD)', ...
  'Shaddad_MCU (decoded in Task_50)', 'Companion -> MCU command (MCU stays final mode authority).', {
  E('seq',               'uint8', 1,'',     'Rolling heartbeat; MCU declares companion stale after 0.2 s without change')
  E('t_stamp',           'double',1,'s',    'Companion time (not transmitted)')
  E('cmd_valid',         'boolean',1,'',    'v/omega valid (AUTO/RTH motion only)')
  E('gnc_ready',         'boolean',1,'',    'GNC healthy: nav_ok, plan valid, no GNC fault (allows MCU AUTO_START)')
  E('mode_req',          'uint8', 1,'',     'GncModeReq: 0 STOP 1 MANUAL_ACK 2 AUTO 3 HOLD 4 FAULT_STOP',0,4)
  E('gnc_state',         'uint8', 1,'',     'GncState (info)')
  E('v_cmd',             'single',1,'m/s',  'Forward speed command (envelope = MCU AUTO limits, D43)',-0.5,1.5)
  E('omega_cmd',         'single',1,'rad/s','Yaw-rate command, CCW +',-1.2,1.2)
  E('accel_limit',       'single',1,'m/s^2','MCU ramp cap',0,2)
  E('soft_stop_req',     'boolean',1,'',    'Controlled deceleration to 0')
  E('v_ground_est',      'single',1,'m/s',  'Fused ground speed (traction control)')
  E('v_ground_valid',    'boolean',1,'',    '')
  E('nav_fix',           'uint8', 1,'',     'NavQuality')
  E('mower_enable',      'boolean',1,'',    '')
  E('mower_speed_cmd',   'single',1,'rad/s','Blade speed (293.2 = 2800 rpm)',0,330)
  E('deck_height_cmd',   'single',1,'m',    'Mower deck height',0.03,0.12)
  E('tiller_enable',     'boolean',1,'',    '')
  E('tiller_speed_cmd',  'single',1,'rad/s','Rotor speed (26.18 = 250 rpm)',0,30)
  E('tiller_depth_cmd',  'single',1,'m',    'Tiller depth, + down',0,0.12)
  E('spray_enable',      'boolean',1,'',    '')
  E('spray_section_mask','uint16',1,'',     'Bit i = nozzle section i (8 used)')
  E('spray_flow_cmd',    'single',1,'L/min','Total spray flow',0,10)
  E('beacon_cmd',        'uint8', 1,'',     '0 off 1 auto 2 hold 3 fault',0,3)
  E('buzzer_cmd',        'boolean',1,'',    '')
  });
d(end+1) = bus_def('Bus_MissionCmd', 0.2, 'Shaddad_System root ScenarioIn.mcmd (ground station in HW)', 'Shaddad_GNC/InputConditioning', ...
  'Mission command; a new command is detected on a change of cmd_seq.', {
  E('cmd',           'uint8', 1,'', 'MissionCmd: 0 NONE 1 LOAD 2 START 3 PAUSE 4 RESUME 5 ABORT 6 RTH 7 RESET_FAULT',0,7)
  E('cmd_seq',       'uint8', 1,'', 'Increment to issue cmd')
  E('mission_id',    'uint16',1,'', '')
  E('end_action',    'uint8', 1,'', '0 STOP 1 RTH',0,1)
  E('speed_override','single',1,'1','Speed scale (0 = use 1)',0,1)
  });
d(end+1) = bus_def('Bus_MissionPlan', Inf, 'scripts/gnc_compileMission.m -> P.gnc.mission (Constant MissionSource inside Shaddad_GNC, no root port, D57)', 'Shaddad_GNC', ...
  'Compiled mission (fixed size, 512 segments x 100 poses).', {
  E('valid',         'boolean',1,        '', '')
  E('mission_id',    'uint16', 1,        '', '')
  E('mission_type',  'uint8',  1,        '', 'MissionType: 1 PALM_ROW_PATROL 2 GREENHOUSE_COVERAGE 3 SOLAR_ROW_MOWING 4 GOTO')
  E('implement_type','uint8',  1,        '', '0 NONE 1 MOWER 2 TILLER 3 SPRAYER')
  E('n_seg',         'uint16', 1,        '', 'Used segments',0,512)
  E('dock_seg',      'uint16', 1,        '', 'Index of the dock-approach segment')
  E('path',          'single', [51200 4],'', 'Per segment 100 rows: x m, y m (ENU), theta deg, dir +1/-1')
  E('seg',           'single', [512 10], '', 'type, dir, length m, v_seg_max m/s, v_end m/s, impl_mode, allow_bypass, lane_halfwidth m, lane_id, pivot_allowed')
  E('boundary',      'double', [64 2],   'm','Geofence vertices ENU (grown by 0.5 m)')
  E('n_bnd',         'uint8',  1,        '', 'Used boundary vertices')
  E('home',          'double', 3,        '', '[x m, y m, psi rad]')
  E('dock',          'double', 3,        '', '[x m, y m, psi rad]')
  E('datum_lla',     'double', 3,        '', '[lat deg, lon deg, h m] (must equal P.world.lla0)')
  E('swath',         'single', 1,        'm','')
  E('area_m2',       'single', 1,        'm^2','')
  E('speed_work',    'single', 1,        'm/s','')
  E('end_action',    'uint8',  1,        '', '0 STOP 1 RTH')
  });
d(end+1) = bus_def('Bus_GncStatus', 0.2, 'Shaddad_GNC/StatusMetrics', 'Viz_Logging, Sim_Monitors, logging (HW: ROS 2 topic)', 'GNC telemetry.', {
  E('gnc_state',     'uint8', 1,'', 'GncState: 0 IDLE 1 MANUAL 2 AUTO_MISSION 3 PAUSE 4 OBSTACLE_HOLD 5 RETURN_TO_HOME 6 FAULT')
  E('sub_mode',      'uint8', 1,'', 'AutoSub: 0 NONE 1 ALIGN 2 TRANSIT 3 ROW 4 TURN 5 BYPASS 6 REJOIN 7 RTH_PLANNING 8 RTH_TRANSIT 9 DOCK_APPROACH 10 DOCKED')
  E('mission_type',  'uint8', 1,'', '')
  E('mission_id',    'uint16',1,'', '')
  E('seg_idx',       'uint16',1,'', '')
  E('n_seg',         'uint16',1,'', '')
  E('progress',      'single',1,'1','',0,1)
  E('area_done_m2',  'single',1,'m^2','')
  E('xte',           'single',1,'m','Cross-track error (estimated)')
  E('heading_err',   'single',1,'rad','')
  E('xte_rms',       'single',1,'m','10 s moving RMS on ROW segments')
  E('v_ref',         'single',1,'m/s','')
  E('dist_home',     'single',1,'m','')
  E('soc_rth',       'single',1,'1','Return-to-home SOC threshold')
  E('fault_word',    'uint32',1,'', 'b0 NAV_INVALID b1 NAV_DEGRADED b2 MCU_LINK_LOST b3 XTE_EXCEEDED b4 STUCK b5 TILT b6 BMS_CRITICAL b7 PLANNER_FAIL b8 OBST_TIMEOUT b9 GEOFENCE b10 MISSION_INVALID b11 ESTOP b12 IMPLEMENT_TIMEOUT')
  E('hold_reason',   'uint8', 1,'', 'HoldReason')
  E('pause_reason',  'uint8', 1,'', 'HoldReason')
  E('hold_timer',    'single',1,'s','')
  E('replan_count',  'uint16',1,'', '')
  E('tracker_active','uint8', 1,'', 'TrackerSel: 0 STOP 1 PP 2 ROW 3 TEB 4 ALIGN')
  E('nav_ok',        'boolean',1,'','')
  E('eta_s',         'single',1,'s','')
  E('pose_est',      'double',3,'', '[x m, y m, psi rad] used by GNC')
  E('mission_done',  'boolean',1,'','')
  });
d(end+1) = bus_def('Bus_GncDebug', 0.05, 'Shaddad_GNC', 'logging', 'GNC internals.', {
  E('pose_ref',     'double',3, '',     '[x y psi] of the closest path point')
  E('kappa_ref',    'single',1, '1/m',  '')
  E('v_prof',       'single',1, 'm/s',  'Velocity-profile speed')
  E('v_gov',        'single',1, 'm/s',  'Governor speed')
  E('v_limits',     'single',N_VLIM,'m/s','MinMax inputs: prof seg obs xte head pitch roll nav soc mode override veg')
  E('omega_pp',     'single',1, 'rad/s','')
  E('omega_row',    'single',1, 'rad/s','')
  E('omega_teb',    'single',1, 'rad/s','')
  E('d_obs',        'single',1, 'm',    'Corridor obstacle distance (Inf = clear)')
  E('vfh_steer',    'single',1, 'rad',  'VFH steering direction (NaN = blocked)')
  E('mpc_qp_status','int32', 1, '',     '')
  E('window_ver',   'uint16',1, '',     '')
  });
d(end+1) = bus_def('Bus_PathWindow', 0.05, 'Shaddad_GNC/PathServer/SegmentPrep', 'Shaddad_GNC/PathAnalyzer, PathTracker', ...
  'Smoothed and profiled current (A) and next (B) segment.', {
  E('PA',   'double',[400 3],'', 'Current segment poses [x m, y m, theta deg]')
  E('sA',   'double',400,    'm','Cumulative length')
  E('kA',   'double',400,    '1/m','Curvature')
  E('vA',   'double',400,    'm/s','Profiled speed')
  E('PB',   'double',[400 3],'', 'Next segment poses')
  E('sB',   'double',400,    'm','')
  E('kB',   'double',400,    '1/m','')
  E('ver',  'uint16',1,      '', 'Window version (+1 per SegmentPrep run)')
  E('segA', 'uint16',1,      '', 'Index of segment A')
  E('segB', 'uint16',1,      '', 'Index of segment B')
  E('typeA','uint8', 1,      '', 'SegType of A')
  E('typeB','uint8', 1,      '', 'SegType of B')
  E('valid','boolean',1,     '', '')
  });
d(end+1) = bus_def('Bus_PathError', 0.05, 'Shaddad_GNC/PathAnalyzer (gnc_pathAnalyzer)', 'Shaddad_GNC MissionManager (delayed), InputConditioning (delayed), PathTracker, SpeedGovernor, LocalAvoidance', ...
  'Tracking errors w.r.t. the current window.', {
  E('xte',       'double',1,    'm',  '+ = robot left of path')
  E('epsi',      'double',1,    'rad','wrap(psi - psi_ref)')
  E('psi_ref',   'double',1,    'rad','')
  E('kappa_ref', 'double',1,    '1/m','')
  E('v_ref',     'double',1,    'm/s','')
  E('s',         'double',1,    'm',  'Arc length along segment A')
  E('s_rem',     'double',1,    'm',  'Remaining length of A')
  E('idx',       'uint16',1,    '',   'Closest pose index in A')
  E('kappa_prev','double',P_MPC,'1/m','Curvature preview over the MPC horizon')
  E('xy_ref',    'double',2,    'm',  'Closest point')
  E('valid',     'boolean',1,   '',   'Closest distance < 2 m')
  });
d(end+1) = bus_def('Bus_FsmOut', 0.05, 'Shaddad_GNC/MissionManager (gnc_missionFSM)', 'Shaddad_GNC subsystems', 'Mission-manager outputs.', {
  E('state',       'uint8', 1,'', 'GncState')
  E('sub',         'uint8', 1,'', 'AutoSub')
  E('seg_idx',     'uint16',1,'', '')
  E('nxt_idx',     'uint16',1,'', '')
  E('path_src',    'uint8', 1,'', 'PathSrc: 0 MISSION 1 REPLAN')
  E('trk_sel',     'uint8', 1,'', 'TrackerSel')
  E('mode_req',    'uint8', 1,'', 'GncModeReq')
  E('cmd_valid',   'boolean',1,'','')
  E('gnc_ready',   'boolean',1,'','')
  E('impl_allow',  'boolean',1,'','Implement allowed (ROW / TURN); false for the rest of an OBSTACLE_HOLD once a person was seen or the scan was lost')
  E('replan_req',  'boolean',1,'','Toggles to request a replan')
  E('replan_goal', 'double',3,'', '[x y psi]')
  E('replan_kind', 'uint8', 1,'', '0 LOCAL 1 GLOBAL')
  E('hold_reason', 'uint8', 1,'', 'HoldReason')
  E('pause_reason','uint8', 1,'', 'HoldReason')
  E('mission_done','boolean',1,'','')
  E('teb_enable',  'boolean',1,'','')
  E('v_cap',       'double',1,'m/s','Mode speed cap')
  E('s_obs',       'double',1,'m','Arc length where the obstacle hold started')
  E('t_hold',      'double',1,'s','')
  E('fault_word',  'uint32',1,'', 'As Bus_GncStatus.fault_word')
  E('trans_id',    'uint8', 1,'', 'Transition fired in this step (gnc_missionFSM table: G1 1, G2 2, T1..T10 11..20, S1..S5 21..25, H1..H3 31..33, impl. timeout 35, RC loss 36, P1..P4 41..44, R1..R6 51..56, F1 61; 0 none)')
  });
d(end+1) = bus_def('Bus_ReplanResult', 0.05, 'Shaddad_GNC/GlobalReplanner (gnc_globalReplanner)', 'Shaddad_GNC MissionManager (delayed), PathServer', ...
  'Hybrid A* replanning result (held).', {
  E('ok',        'boolean',1,      '', '')
  E('busy',      'boolean',1,      '', '')
  E('ver',       'uint16', 1,      '', '+1 per successful plan')
  E('n',         'uint16', 1,      '', 'Used poses',0,100)
  E('path',      'double', [100 4],'', '[x m, y m, theta deg, dir]')
  E('fail_count','uint8',  1,      '', '')
  E('t_plan',    'double', 1,      's','')
  });
d(end+1) = bus_def('Bus_MpcPlant', -1, 'gnc_mpcPlantUpdate', 'Adaptive MPC Controller (model port)', 'Discrete LTV plant.', {
  E('A','double',[NX_MPC NX_MPC],'','x = [e_y e_psi r u(k-1) u(k-2)] (2-sample MV delay, D41)')
  E('B','double',[NX_MPC NU_MPC],'','u = [r_cmd (MV) kappa_ref (MD) d w_y w_psi (UD)]')
  E('C','double',[2 NX_MPC],'','y = [e_y e_psi]')
  E('D','double',[2 NU_MPC],'','')
  });
d(end+1) = bus_def('Bus_MpcNominal', -1, 'gnc_mpcPlantUpdate', 'Adaptive MPC Controller (model port)', 'Nominal operating point.', {
  E('X', 'double',[NX_MPC 1],'','column matrix (Adaptive MPC model inport requirement)')
  E('U', 'double',[NU_MPC 1],'','')
  E('Y', 'double',[2 1],'','')
  E('DX','double',[NX_MPC 1],'','')
  });
d(end+1) = bus_def('Bus_MpcModel', 0.05, 'Shaddad_GNC/PathTracker/RowTracker (gnc_mpcPlantUpdate)', 'Adaptive MPC Controller', 'Adaptive MPC model input.', {
  E('Plant',  'Bus: Bus_MpcPlant',  1,'','')
  E('Nominal','Bus: Bus_MpcNominal',1,'','')
  });
d(end+1) = bus_def('Bus_NavCond', 0.05, 'Shaddad_GNC/InputConditioning', 'Shaddad_GNC subsystems', 'Conditioned navigation state.', {
  E('t',           'double', 1,'s',    '')
  E('pose',        'double', 3,'',     '[x m, y m, psi rad]')
  E('v',           'double', 1,'m/s',  '')
  E('yaw_rate',    'double', 1,'rad/s','')
  E('pitch',       'double', 1,'rad',  '+ nose down')
  E('roll',        'double', 1,'rad',  '')
  E('sigma_pos',   'double', 1,'m',    'max(posStd(1:2))')
  E('nav_ok',      'boolean',1,'',     '')
  E('nav_degraded','boolean',1,'',     '')
  E('gnss_fix',    'uint8',  1,'',     'NavQuality of nav_state (0 INVALID 1 DR 2 GNSS_SINGLE 3 GNSS_FLOAT 4 RTK_FIXED 5 LIDAR_LOCALIZED)')
  });
d(end+1) = bus_def('Bus_TebMapCrop', 0.2, 'Shaddad_GNC/LocalAvoidance/Bypass (crop of Bus_Perception.map)', ...
  'Timed Elastic Band (TEB) MapInfo', ['TEB map window: P.gnc.teb.map_window (10 m) crop of the robot-centred ' ...
  'perception grid, 2 x 2 max-pooled to 5 cells/m (D56). Field order = TEB MapInfo.'], {
  E('Resolution',          'double', 1,               '', 'Cells per metre (5)')
  E('GridLocationInWorld', 'double', [1 2],           'm','World location of the crop lower-left corner')
  E('GridSize',            'double', [1 2],           '', '[rows cols]')
  E('OccupancyMatrix',     'boolean',[N_TEB N_TEB],   '', 'true = occupied; row 1 = max y')
  E('GridOriginInLocal',   'double', [1 2],           'm','')
  });
d(end+1) = bus_def('Bus_ObsFlags', 0.05, 'Shaddad_GNC/LocalAvoidance', 'Shaddad_GNC MissionManager, SpeedGovernor', 'Obstacle flags (rate-transitioned to 0.05 s).', {
  E('d_obs',       'double', 1,'m',  'Corridor distance, Inf = clear')
  E('blocked_vfh', 'boolean',1,'',   'VFH SteerDir is NaN')
  E('person_near', 'boolean',1,'',   '= Bus_Perception.protectiveStop')
  E('vfh_steer',   'double', 1,'rad','')
  E('scan_valid',  'boolean',1,'',   '= Bus_Perception.scanValid (false until the first scan); lost scan -> OBSTACLE_HOLD SENSOR_LOST')
  });
end
