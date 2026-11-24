function d = buses_drive()
%BUSES_DRIVE  Traction drive buses (owner: drive). Engineering values; on CAN1 they are packed
%   by drive_can_pack / drive_can_unpack (frames 0x201/0x202 CMD, 0x181/0x182 FB1,
%   0x281/0x282 FB2, 0x381/0x382 FB3). Positive speed/torque = forward track motion on both sides.
E = @bus_el;
d = bus_def('Bus_DriveSideCmd', 0.01, 'Shaddad_MCU/Task_10/Drive_Cmd (decoded inside drive_system by drive_can_unpack)', ...
  'drive_system/drive_esc', 'Command to one traction ESC (CiA-402 style speed mode).', {
  E('enable',      'boolean',1,'',     'Request OPERATION_ENABLED')
  E('mode',        'uint8',  1,'',     '0 OFF, 1 SPEED (default), 2 TORQUE',0,2)
  E('w_ref',       'single', 1,'rad/s','Motor-shaft speed setpoint (= P.drive.gear.k_v2w * v_track)',-502.7,502.7)
  E('tau_ff',      'single', 1,'N*m',  'Torque feed-forward (SPEED) or setpoint (TORQUE), motor shaft',-16,16)
  E('tau_lim_mot', 'single', 1,'N*m',  'Motoring torque limit (magnitude) from MCU limiter',0,16)
  E('tau_lim_reg', 'single', 1,'N*m',  'Regenerative torque limit (magnitude) from MCU limiter (BMS CCL)',0,16)
  E('park_req',    'boolean',1,'',     'Force brake engaged and torque off')
  E('fault_reset', 'boolean',1,'',     'Rising edge clears latched ESC faults')
  E('alive',       'uint8',  1,'',     'Rolling counter +1 per frame; frozen 100 ms => QUICK_STOP')
  });
d(end+1) = bus_def('Bus_DriveCmd', 0.01, 'Shaddad_MCU', 'drive_system', 'Left/right drive commands.', {
  E('L','Bus: Bus_DriveSideCmd',1,'','Left ESC (node 1)')
  E('R','Bus: Bus_DriveSideCmd',1,'','Right ESC (node 2)')
  });
d(end+1) = bus_def('Bus_DriveSideFb', 0.01, 'drive_system/drive_esc (packed by drive_can_pack, decoded in Shaddad_MCU)', ...
  'Shaddad_MCU (limiter, supervisor, odometry)', 'Feedback from one ESC.', {
  E('state',          'uint8',  1,'',     'EscState: 0 INIT 1 NOT_READY 2 READY 3 ENABLING 4 OPERATION 5 DISABLING 6 QUICK_STOP 7 FAULT 8 STO',0,8)
  E('status',         'uint16', 1,'',     'b0 READY b1 OP_ENABLED b2 FAULT b3 DERATING b4 BRAKE_RELEASED b5 MODE_SPEED b6 MODE_TORQUE b7 TORQUE_LIMIT_ACTIVE b8 STO_ACTIVE b9 QUICKSTOP b10 STALL_WARNING')
  E('fault_code',     'uint16', 1,'',     'b0 OVERCURRENT b1 OVERVOLTAGE b2 UNDERVOLTAGE b3 MOTOR_OVERTEMP b4 ESC_OVERTEMP b5 OVERSPEED b6 CAN_TIMEOUT b7 ENCODER b8 BRAKE b9 STALL b10 STO_ACTIVE')
  E('w_m',            'single', 1,'rad/s','Measured motor speed (1 rpm resolution)')
  E('enc_cnt',        'int32',  1,'',     'Motor encoder count, 4096/rev, wraps; use wrap-safe difference')
  E('tau_est',        'single', 1,'N*m',  'Estimated motor torque')
  E('tau_avail',      'single', 1,'N*m',  'Available motoring torque after all derating',0,16)
  E('i_q',            'single', 1,'A',    'q-axis current (peak)')
  E('i_dc',           'single', 1,'A',    'DC input current, + = discharge')
  E('v_dc',           'single', 1,'V',    'DC link voltage')
  E('T_motor',        'single', 1,'K',    'Winding sensor (5 s lag)')
  E('T_esc',          'single', 1,'K',    'Heat-sink temperature')
  E('brake_released', 'boolean',1,'',     'Parking brake released')
  E('alive',          'uint8',  1,'',     'ESC rolling counter')
  E('rx_ok',          'boolean',1,'',     'FB1 and FB3 fresh (< 50 ms), set by the MCU decoder')
  });
d(end+1) = bus_def('Bus_DriveFb', 0.01, 'Shaddad_MCU (decoder)', 'Shaddad_MCU tasks', 'Left/right ESC feedback.', {
  E('L','Bus: Bus_DriveSideFb',1,'','Left ESC')
  E('R','Bus: Bus_DriveSideFb',1,'','Right ESC')
  });
d(end+1) = bus_def('Bus_DriveSideTruth', 0.001, 'drive_system/drive_side (PS-Simulink)', 'Shaddad_SensorModels (mag disturbance), Sim_Monitors, logging', ...
  'Plant truth of one drive (double).', {
  E('w_m',        'double',1,'rad/s','Motor speed')
  E('theta_m',    'double',1,'rad',  'Motor angle (unwrapped)')
  E('w_spr',      'double',1,'rad/s','Sprocket (gearbox output) speed')
  E('theta_spr',  'double',1,'rad',  'Sprocket angle (unwrapped)')
  E('tau_m',      'double',1,'N*m',  'Motor shaft torque')
  E('tau_spr',    'double',1,'N*m',  'Gearbox output torque')
  E('tau_brake',  'double',1,'N*m',  'Brake friction torque')
  E('f_brake_rel','double',1,'1',    'Brake release fraction 0 engaged .. 1 released',0,1)
  E('v_dc',       'double',1,'V',    'DC voltage at ESC terminals')
  E('i_dc',       'double',1,'A',    'DC current into ESC, + = discharge')
  E('p_dc',       'double',1,'W',    'DC power')
  E('p_spr',      'double',1,'W',    'Sprocket mechanical power')
  E('P_loss',     'double',1,'W',    'Motor + ESC losses')
  E('T_winding',  'double',1,'K',    '')
  E('T_housing',  'double',1,'K',    '')
  E('T_esc',      'double',1,'K',    '')
  E('E_dc',       'double',1,'J',    'Integral of p_dc')
  E('E_spr',      'double',1,'J',    'Integral of p_spr (sprocket mechanical energy, 1 ms)')
  E('E_loss',     'double',1,'J',    'Integral of p_dc - p_spr (motor + ESC + gearbox + brake + coil losses, 1 ms)')
  });
d(end+1) = bus_def('Bus_DriveTruth', 0.001, 'drive_system', 'Shaddad_SensorModels, Sim_Monitors, logging', 'Left/right drive truth.', {
  E('L','Bus: Bus_DriveSideTruth',1,'','Left')
  E('R','Bus: Bus_DriveSideTruth',1,'','Right')
  });
d(end+1) = bus_def('Bus_DriveSideFaultInj', 0.001, 'Shaddad_Plant/Plant_In (mapped from Bus_FaultInject)', 'drive_system/drive_side', ...
  'Drive fault injection for one side.', {
  E('can_drop',   'boolean',1,'', 'Reserved (frame drops are injected in CAN1_Net_Sim); keep false')
  E('enc_fail',   'boolean',1,'', 'Encoder signal lost -> ESC ENCODER fault')
  E('brake_stuck','boolean',1,'', 'Brake forced engaged')
  E('tau_scale',  'double', 1,'1','Multiplier on the torque reference (1 nominal, 0 = drive trip)',0,2)
  E('T_offset',   'double', 1,'K','Offset added to the winding temperature sensor')
  });
d(end+1) = bus_def('Bus_DriveFaultInj', 0.001, 'Shaddad_Plant/Plant_In', 'drive_system', 'Left/right drive fault injection.', {
  E('L','Bus: Bus_DriveSideFaultInj',1,'','Left')
  E('R','Bus: Bus_DriveSideFaultInj',1,'','Right')
  });
end
