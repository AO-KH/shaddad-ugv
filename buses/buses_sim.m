function d = buses_sim()
%BUSES_SIM  Scenario / integration buses (owner: integration).
%   All-zero values of Bus_ScenarioIn are the SAFE IDLE: key ON, no E-stop pressed,
%   transmitter OFF, no faults, no obstacles (fields are named so that 0/false = normal).
E = @bus_el;
d = bus_def('Bus_OperatorCmd', 0.01, 'Shaddad_System/Operator_Console (sys.operator.op)', 'Shaddad_System/Operator_RC/RC_Link_Sim', ...
  'Operator stick/switch intent fed to the simulated RC transmitter (Mode 2 TX16S-class).', {
  E('stick_thr',   'double',1,'1','Throttle stick, + = forward',-1,1)
  E('stick_steer', 'double',1,'1','Steering stick, + = stick RIGHT = turn right (omega < 0)',-1,1)
  E('stick_aux_lv','double',1,'1','Left stick vertical (spare)',-1,1)
  E('stick_aux_lh','double',1,'1','Left stick horizontal (spare)',-1,1)
  E('knob_speed',  'double',1,'1','Speed-scale knob S1 (-1 = min 25 %, +1 = 100 %)',-1,1)
  E('sw_mode',     'double',1,'', 'CH5 SA 3-pos: 1 MANUAL, 2 HOLD, 3 AUTO (0 treated as 1)',0,3)
  E('sw_arm',      'double',1,'', 'CH6 SF arm switch: 0 disarmed, 1 armed',0,1)
  E('sw_impl',     'double',1,'', 'CH8 SB implement 3-pos: 1 OFF, 2 ON, 3 ON + auto-spray',0,3)
  E('sw_lift',     'double',1,'', 'CH11 SC lift 3-pos: 1 UP, 2 HOLD, 3 DOWN',0,3)
  E('sw_lights',   'double',1,'', 'CH10 SD work lights: 0 off, 1 on',0,1)
  E('sw_estop',    'double',1,'', 'CH12 SE remote (software) E-STOP: 0 run, 1 STOP',0,1)
  E('btn_reset',   'double',1,'', 'CH9 SH momentary fault/E-stop reset: 1 pressed',0,1)
  E('sw_mission',  'double',1,'', 'CH13 SG 3-pos momentary: 1 none, 2 START/RESUME, 3 PAUSE',0,3)
  E('btn_rth',     'double',1,'', 'CH14 SI momentary return-to-home: 1 pressed',0,1)
  E('tx_power_on', 'double',1,'', 'Transmitter powered: 0 off (no RF frames), 1 on',0,1)
  });
d(end+1) = bus_def('Bus_HmiInputs', 0.01, 'Shaddad_System root ScenarioIn.hmi', ...
  'Power_Plant/Safety_Chain, Electronics_Plant/HW_Frontend_Sim', ...
  'Hard-wired operator inputs on the robot body. All false = normal operation.', {
  E('estop_front_pressed',   'boolean',1,'','Front mushroom E-stop pressed')
  E('estop_rear_pressed',    'boolean',1,'','Rear mushroom E-stop pressed')
  E('wireless_estop_tripped','boolean',1,'','Certified wireless E-stop (optional) tripped')
  E('bumper_front',          'boolean',1,'','Front bumper switch actuated')
  E('bumper_rear',           'boolean',1,'','Rear bumper switch actuated')
  E('reset_btn',             'boolean',1,'','Safety-relay reset button pressed (rising edge re-arms)')
  E('key_off',               'boolean',1,'','Key switch OFF (false = key ON)')
  E('lid_open',              'boolean',1,'','Electronics enclosure lid open')
  });
d(end+1) = bus_def('Bus_FaultInject', 0.01, 'Shaddad_System root ScenarioIn.flt', ...
  'RC_Link_Sim, Shaddad_SensorModels, Shaddad_Plant (drive/power/electronics), CAN1_Net_Sim, Link_MCU_GNC, MCU_In assembly', ...
  'Scenario fault injection flags (all false/0 = no fault).', {
  E('rc_link_loss',     'boolean',1,'','RF frames lost (receiver sets frame_lost, then failsafe after P.rc.rx_failsafe_time)')
  E('rc_line_silent',   'boolean',1,'','Receiver SBUS output stops (no bytes)')
  E('rc_byte_err_rate', 'double', 1,'1','Probability of a corrupted SBUS byte',0,1)
  E('gnss_outage',      'boolean',1,'','No GNSS fix (fixType 0, newData false)')
  E('gnss_force_float', 'boolean',1,'','RTK fixed forced to float')
  E('gnss_corr_loss',   'boolean',1,'','NTRIP / RTK correction link lost (corrAge grows)')
  E('imu_fault',        'uint8',  1,'','IMU fault: 0 none, 1 stuck, 2 bias step, 3 dropout (invalid)',0,3)
  E('lidar_blackout',   'boolean',1,'','LiDAR returns no points (valid false)')
  E('enc_fault_L',      'boolean',1,'','Left motor encoder lost (ESC ENCODER fault)')
  E('enc_fault_R',      'boolean',1,'','Right motor encoder lost')
  E('motor_fault_L',    'boolean',1,'','Left drive trips (tau_scale = 0, overcurrent fault)')
  E('motor_fault_R',    'boolean',1,'','Right drive trips')
  E('brake_stuck_L',    'boolean',1,'','Left parking brake stuck engaged')
  E('brake_stuck_R',    'boolean',1,'','Right parking brake stuck engaged')
  E('estop_ch2_stuck',  'boolean',1,'','E-stop channel 2 contact welded closed (discrepancy test)')
  E('bms_fault',        'boolean',1,'','Inject +0.5 V on cell-group 1 voltage measurement (BMS OV fault)')
  E('can1_drop_mask',   'uint32', 1,'','CAN1 frame drop: b0 DRV_CMD_L b1 DRV_CMD_R b2 FB1_L b3 FB1_R b4 FB3_L b5 FB3_R b6 FB2_L b7 FB2_R b8 IMPL_CMD b9 BMS_REQ b10 IMPL_FB b11 all BMS frames')
  E('can2_loss',        'boolean',1,'','All CAN-FD frames MCU<->companion lost')
  E('can1_busoff',      'boolean',1,'','MCU FDCAN1 bus-off: MCU_In sets fault_inject b4, CAN1_Net_Sim drops all MCU-originated CAN1 frames')
  E('can2_busoff',      'boolean',1,'','MCU FDCAN2 bus-off: MCU_In sets fault_inject b5, Link_MCU_GNC drops all CAN-FD frames')
  E('task_stall',       'uint8',  1,'','MCU task alive-counter stall: b0 10 ms task, b1 50 ms task, b2 100 ms task, b3 force overrun',0,15)
  E('dust_level',       'double', 1,'1','Airborne dust level for LiDAR clutter',0,1)
  });
d(end+1) = bus_def('Bus_ObstaclesDyn', 0.01, 'Shaddad_System root ScenarioIn.obs', ...
  'Shaddad_SensorModels (LiDAR), Sim_Monitors (collision)', ...
  'Dynamic obstacles (people, vehicles) in site ENU; rows > n are ignored.', {
  E('n',   'uint8', 1,     '',   'Number of active dynamic obstacles',0,4)
  E('xy',  'double',[4 2], 'm',  'Centre position [x_E y_N] per obstacle')
  E('v_xy','double',[4 2], 'm/s','Velocity [vE vN] per obstacle')
  E('r',   'double',4,     'm',  'Radius (cylinder model)',0,5)
  });
d(end+1) = bus_def('Bus_EnvOverride', 0.01, 'Shaddad_System root ScenarioIn.env', ...
  'structure_env (Structure)', 'Scenario overrides of the environment.', {
  E('use_T',              'boolean',1,'','Override ambient temperature')
  E('T_amb',              'double', 1,'K','Ambient temperature when use_T',233.15,343.15)
  E('use_G',              'boolean',1,'','Override solar irradiance')
  E('G_solar',            'double', 1,'W/m^2','Global horizontal irradiance when use_G',0,1400)
  E('use_soil',           'boolean',1,'','Override soil class everywhere (contact model uses this soil id)')
  E('soil_id',            'uint8',  1,'','Soil class 1..5 (P.world.soilTable)',0,5)
  E('dock_contact_force', 'boolean',1,'','Force dock contacts closed (charging tests without docking)')
  });
d(end+1) = bus_def('Bus_ScenarioIn', 0.01, 'Simulink.SimulationInput.setExternalInput (root In Bus Element ScenarioIn)', ...
  'Operator_Console, Shaddad_Plant, Shaddad_SensorModels, CAN nets, GNC_Variant', ...
  'Root scenario input of Shaddad_System (ZOH, 100 Hz).', {
  E('op',  'Bus: Bus_OperatorCmd', 1,'','Operator intent')
  E('hmi', 'Bus: Bus_HmiInputs',   1,'','Body-mounted E-stops, bumpers, key, reset')
  E('flt', 'Bus: Bus_FaultInject', 1,'','Fault injection')
  E('obs', 'Bus: Bus_ObstaclesDyn',1,'','Dynamic obstacles')
  E('env', 'Bus: Bus_EnvOverride', 1,'','Environment overrides')
  E('mcmd','Bus: Bus_MissionCmd',  1,'','Ground-station mission command (GNC)')
  });
d(end+1) = bus_def('Bus_SimKPI', 0.1, 'Sim_Monitors', 'logging, eval_criteria', ...
  'Simulation key performance indicators and stop reason.', {
  E('t_mission',  'double',1,'s','Time since McuStatus.mode first became AUTO')
  E('dist',       'double',1,'m','Integral of |v| of point R (truth)')
  E('E_batt',     'double',1,'J','Net battery terminal energy = 3600*(power_truth.E_out_Wh - E_in_Wh), discharge positive')
  E('E_regen',    'double',1,'J','Charge energy into the pack = 3600*power_truth.E_in_Wh')
  E('eps_el',     'double',1,'1','Electrical energy-audit residual (architecture D40)')
  E('eps_mech',   'double',1,'1','Mechanical energy-audit residual (architecture D40)')
  E('clear_min',  'double',1,'m','Minimum footprint-to-obstacle separation so far')
  E('collision',  'boolean',1,'','Any collision now')
  E('n_coll',     'uint16',1,'','Number of collision onsets')
  E('soc_min',    'double',1,'1','Running minimum true pack SOC',0,1)
  E('T_cell_max', 'double',1,'K','Running maximum true cell temperature')
  E('I_batt_peak','double',1,'A','Running maximum |I_pack| at 1 ms (= power_truth.I_pack_peak_A)')
  E('out_of_map', 'boolean',1,'','Point R outside the DEM extent')
  E('stop_reason','uint8', 1,'','0 running, 1 StopTime, 2 mission done, 3 SoC floor, 4 collision, 5 NaN, 6 assertion, 7 wall-time')
  });
end
