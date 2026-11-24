function d = buses_sensors()
%BUSES_SENSORS  Sensor-model, acquisition, navigation-fusion and perception buses (owner: sensors).
%   Accelerometer values on ALL buses are PHYSICAL specific force (level & at rest: z = +g in FLU),
%   the same as the real ICM-20948. MATLAB imuSensor / imufilter / ecompass / complementaryFilter
%   use the negative convention -> Gain -1 directly at those library blocks only.
%   Magnetic field in uT. No LiDAR return = NaN.
E = @bus_el;
d = bus_def('Bus_ImuRaw', 0.005, 'Shaddad_SensorModels/sens_imu_model (SIM) | ICM20948 IMU Sensor block (HW)', ...
  'Shaddad_MCU IO_In -> sens_acq_imu', 'Mirrors the ICM20948 IMU Sensor block outputs (sensor frame).', {
  E('Acceleration', 'single',3,'m/s^2','Specific force, sensor axes (physical sign)',-39.2266,39.2266)
  E('AngularRate',  'single',3,'rad/s','Angular rate, sensor axes',-8.7266,8.7266)
  E('MagneticField','single',3,'uT',   'Magnetic field, sensor axes')
  E('Temperature',  'single',1,'degC', 'Die temperature')
  });
d(end+1) = bus_def('Bus_GnssRaw', 0.1, 'Shaddad_SensorModels/sens_gnss_model (SIM) | ROS 2 ublox topics (HW)', ...
  'Shaddad_NavFusion IO_In', 'Dual-antenna RTK receiver output (primary antenna).', {
  E('t_meas',      'double', 1,'s',  'Time of validity (= t - latency)')
  E('lla',         'double', 3,'',   '[lat deg, lon deg, h m] primary antenna')
  E('vel_ENU',     'double', 3,'m/s','Antenna velocity ENU')
  E('fixType',     'uint8',  1,'',   'NMEA GGA quality: 0 NOFIX 1 SINGLE 4 RTK_FIXED 5 RTK_FLOAT')
  E('numSV',       'uint8',  1,'',   'Satellites used')
  E('hAcc',        'single', 1,'m',  'Reported horizontal accuracy (1 sigma)')
  E('vAcc',        'single', 1,'m',  'Reported vertical accuracy')
  E('sAcc',        'single', 1,'m/s','Reported speed accuracy')
  E('headingValid','boolean',1,'',   'Moving-base RELPOS heading valid')
  E('heading_deg', 'double', 1,'deg','Baseline heading, CW from North',0,360)
  E('headAcc_deg', 'single', 1,'deg','Heading accuracy')
  E('corrAge',     'single', 1,'s',  'Correction age')
  E('newData',     'boolean',1,'',   'New epoch in this sample')
  });
d(end+1) = bus_def('Bus_LidarScanRaw', 0.1, 'Shaddad_SensorModels/sens_lidar_model FAST2D', 'Shaddad_Perception IO_In', ...
  '2-D height-band scan in the LiDAR frame (beam k at angle -pi + (k-1) deg).', {
  E('t_scan','double', 1,  's',  'Scan time')
  E('ranges','single', 360,'m',  'Range per beam, NaN = no return')
  E('angles','single', 360,'rad','Beam angle CCW from LiDAR x')
  E('valid', 'boolean',1,  '',   'Scan valid')
  });
d(end+1) = bus_def('Bus_LidarCloudRaw', 0.1, 'Shaddad_SensorModels/sens_lidar_model FULL3D (UAV Scenario Lidar) | ROS 2 /ouster/points', ...
  'Shaddad_Perception IO_In (sens_lidar_cloud2scan)', 'Organised 3-D point cloud, sensor frame, NaN padded.', {
  E('t_scan','double', 1,         's','Scan time')
  E('xyz',   'single', [32 1024 3],'m','Organised cloud [rings x azimuth x xyz]')
  E('nAz',   'uint16', 1,         '', 'Used azimuth columns (360 in SIM, 1024 on HW)')
  E('valid', 'boolean',1,         '', 'Cloud valid')
  });
d(end+1) = bus_def('Bus_SensTruthDiag', 0.1, 'Shaddad_SensorModels', 'logging, tests', 'Sensor-model truth diagnostics.', {
  E('svf',           'double', 1,'1','GNSS sky-view factor',0,1)
  E('zoneId',        'uint8',  1,'', '0 open, 1 greenhouse, 2 under panel, 3 between panels')
  E('fixTypeTruth',  'uint8',  1,'', 'Fix-chain state')
  E('relposFixed',   'boolean',1,'', '')
  E('gnssPosErr',    'double', 3,'m','Reported minus true antenna position (ENU)')
  E('outlier',       'boolean',1,'', '')
  E('dustLevel',     'double', 1,'1','',0,1)
  E('lidarPoseTruth','double', 3,'', '[x_E y_N psi] of the LiDAR')
  });
d(end+1) = bus_def('Bus_SprayDemand', 0.1, 'Shaddad_SensorModels/sens_spray_vision (abstract camera + detector) | ROS 2 vision node (HW)', ...
  'Shaddad_GNC/ImplementManager', 'Vision-sprayer section demand.', {
  E('t_stamp',        'double', 1,'s','')
  E('section_mask',   'uint16', 1,'', 'Bit i = weed/target detected under boom section i (8 used)')
  E('target_density', 'single', 1,'1','Mean detected density 0..1',0,1)
  E('valid',          'boolean',1,'', '')
  });
d(end+1) = bus_def('Bus_ImuMeas', 0.005, 'Companion_IO (decoded CAN2 0x201 MCU_IMU)', 'Shaddad_NavFusion', ...
  'Calibrated IMU sample forwarded by the MCU (body FLU, physical specific force).', {
  E('accel_B',  'single',3,'m/s^2','Specific force, body FLU')
  E('gyro_B',   'single',3,'rad/s','Angular rate, body FLU')
  E('mag_B',    'single',3,'uT',   'Magnetic field, body FLU')
  E('temp_degC','single',1,'degC', '')
  E('t_us',     'uint32',1,'us',   'MCU timestamp of the sample')
  E('seq',      'uint16',1,'',     'Sample counter')
  E('valid',    'boolean',1,'',    '')
  });
d(end+1) = bus_def('Bus_OdomMeas', 0.01, 'Companion_IO (decoded CAN2 0x202 MCU_ODOM; computed in Shaddad_MCU sens_odom_compute)', ...
  'Shaddad_NavFusion', 'Track odometry from the ESC motor encoders (50 ms window).', {
  E('vTrack_LR','single',2,'m/s',  'Sprocket pitch-line speed [L R] = dCount*K_enc/0.05 s')
  E('encCnt_LR','int32', 2,'',     'Latest encoder counts [L R] (wrapping)')
  E('wMotor_LR','single',2,'rad/s','ESC-reported motor speed [L R]')
  E('t_us',     'uint32',1,'us',   'MCU timestamp')
  E('valid',    'boolean',1,'',    'Both ESC feedbacks fresh')
  });
d(end+1) = bus_def('Bus_NavCmd', 0.05, 'Shaddad_GNC/MissionManager', 'Shaddad_NavFusion/NavModeManager', 'Navigation mode requests.', {
  E('navMode',   'uint8', 1,'','0 AUTO, 1 GNSS, 2 LIDAR_PRIOR_MAP, 3 DR_TEST',0,3)
  E('navReset',  'boolean',1,'','Rising edge resets the EKF')
  E('priorMapId','uint8', 1,'','Prior map index (0 = P.world.occ)')
  });
d(end+1) = bus_def('Bus_NavState', 0.01, 'Shaddad_NavFusion/NavOutput', 'Shaddad_GNC, Shaddad_Perception (delayed), Viz_Logging, logging', ...
  'Fused navigation state of base_link (ENU / FLU).', {
  E('t',              'double', 1,    's',    '')
  E('pos_W',          'double', 3,    'm',    'base_link position ENU')
  E('lla',            'double', 3,    '',     '[lat deg, lon deg, h m]')
  E('eul',            'double', 3,    'rad',  'ZYX [yaw pitch roll]; yaw CCW from East; pitch + = nose down')
  E('q_WB',           'double', 4,    '1',    'Frame-rotation quaternion W->B, scalar first')
  E('vel_B',          'double', 3,    'm/s',  '[v vy 0]')
  E('vel_W',          'double', 3,    'm/s',  '')
  E('yawRate',        'double', 1,    'rad/s','psidot_m - bg')
  E('omega_B',        'double', 3,    'rad/s','Bias-corrected gyro')
  E('accel_B',        'double', 3,    'm/s^2','Kinematic acceleration (gravity removed)')
  E('slip_LR',        'double', 2,    '1',    'Estimated track slip')
  E('chi',            'double', 1,    '1',    'Estimated effective-gauge factor')
  E('gyroBiasZ',      'double', 1,    'rad/s','')
  E('accelBiasX',     'double', 1,    'm/s^2','')
  E('posStd',         'double', 3,    'm',    'sqrt(diag P) of position')
  E('yawStd',         'double', 1,    'rad',  '')
  E('velStd',         'double', 1,    'm/s',  '')
  E('covPose',        'double', [3 3],'',     'Covariance of [pE pN psi]')
  E('quality',        'uint8',  1,    '',     'NavQuality: 0 INVALID 1 DR 2 GNSS_SINGLE 3 GNSS_FLOAT 4 RTK_FIXED 5 LIDAR_LOCALIZED')
  E('navValid',       'boolean',1,    '',     '')
  E('headingValid',   'boolean',1,    '',     '')
  E('aligned',        'boolean',1,    '',     '')
  E('lidarLocActive', 'boolean',1,    '',     '')
  E('gnssFixType',    'uint8',  1,    '',     'Last GNSS fix type')
  E('tSinceAbsUpdate','double', 1,    's',    'Time since last absolute (GNSS/LiDAR) update')
  E('resetCount',     'uint16', 1,    '',     '')
  });
d(end+1) = bus_def('Bus_NavDiag', 0.01, 'Shaddad_NavFusion', 'logging, tests', 'EKF diagnostics.', {
  E('nis',        'double', 5, '', 'Normalised innovation squared per channel')
  E('enable',     'boolean',5, '', 'Measurement accepted per channel')
  E('rejCount',   'double', 5, '', 'Consecutive rejections')
  E('gnssLatency','double', 1, 's','')
  E('ekfTrace',   'double', 1, '', 'trace(P)')
  E('P_diag',     'double', 11,'', 'diag(P)')
  E('x',          'double', 11,'', 'EKF state')
  E('modeState',  'uint8',  1, '', 'Nav mode manager state')
  });
d(end+1) = bus_def('Bus_ScanMatchMeas', 0.2, 'Shaddad_Perception/sens_scanmatch_loc', 'Shaddad_NavFusion (EKF channel 5)', ...
  'LiDAR scan-to-prior-map pose of base_link.', {
  E('valid','boolean',1,    '',  '')
  E('t',    'double', 1,    's', '')
  E('pose', 'double', 3,    '',  '[x_E m, y_N m, psi rad]')
  E('cov',  'double', [3 3],'',  'Covariance in W')
  E('score','double', 1,    '',  'matchScansGrid score')
  E('nPts', 'uint16', 1,    '',  'Valid points')
  });
d(end+1) = bus_def('Bus_TebMapInfo', 0.1, 'Shaddad_Perception/sens_occmap_update', 'Shaddad_GNC/LocalAvoidance (TEB MapInfo)', ...
  'Rolling 30 x 30 m occupancy grid; FIELD ORDER = TEB default MapInfo struct (verified).', {
  E('Resolution',          'double', 1,        '', 'Cells per metre (10)')
  E('GridLocationInWorld', 'double', [1 2],    'm','World location of the grid lower-left corner')
  E('GridSize',            'double', [1 2],    '', '[rows cols] = [300 300]')
  E('OccupancyMatrix',     'boolean',[300 300],'', 'true = occupied (p > 0.65); row 1 = max y')
  E('GridOriginInLocal',   'double', [1 2],    'm','Grid origin in local frame')
  });
d(end+1) = bus_def('Bus_RowFeature', 0.1, 'Shaddad_Perception/sens_row_detect', 'Shaddad_GNC (row following aid), logging', ...
  'Palm / solar-pile row geometry relative to the robot.', {
  E('valid',        'boolean',1,'',   '')
  E('lateralOffset','single', 1,'m',  '+ = robot left of row centre')
  E('headingErr',   'single', 1,'rad','')
  E('distLeft',     'single', 1,'m',  '')
  E('distRight',    'single', 1,'m',  '')
  E('nLeft',        'uint8',  1,'',   'Inliers left')
  E('nRight',       'uint8',  1,'',   'Inliers right')
  E('fitRms',       'single', 1,'m',  '')
  });
d(end+1) = bus_def('Bus_Perception', 0.1, 'Shaddad_Perception', 'Shaddad_GNC (corridor, VFH, TEB, hold logic), Viz_Logging', ...
  'Perception products (base_link frame).', {
  E('t_scan',        'double', 1,     's',  '')
  E('scanValid',     'boolean',1,     '',   'Scan of this step is valid (read by the protective field and, through Bus_ObsFlags.scan_valid, the GNC hold logic)')
  E('ranges_B',      'single', 360,   'm',  'Filtered ranges about base_link, NaN = no return (VFH Ranges)')
  E('angles_B',      'single', 360,   'rad','Beam angles CCW from body x (VFH Angles)')
  E('minSectorRange','single', 8,     'm',  '8 x 45 deg sectors, sector 1 centred forward, CCW')
  E('protectiveStop','boolean',1,     '',   ['Object inside the speed-dependent protective field (front field from protXF, the fitted ' ...
      'implement front, while v > -0.1 m/s; rear field from protXR while v < 0.1 m/s), or more than protNStale invalid scans in a ' ...
      'row (true from the start until the first valid scan); used as person_near'])
  E('protFieldLen',  'single', 1,     'm',  '0.5 + 0.8*|v|')
  E('obsCount',      'uint8',  1,     '',   'Valid obstacle rows',0,20)
  E('obsPos_B',      'single', [20 2],'m',  'Obstacle centroids, body frame')
  E('obsRadius',     'single', 20,    'm',  '')
  E('obsRange',      'single', 20,    'm',  '')
  E('row',           'Bus: Bus_RowFeature',   1,'','')
  E('map',           'Bus: Bus_TebMapInfo',   1,'','')
  E('scanMatch',     'Bus: Bus_ScanMatchMeas',1,'','Latest scan-match result (held)')
  });
end
