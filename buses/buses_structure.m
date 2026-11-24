function d = buses_structure()
%BUSES_STRUCTURE  Chassis / tracks / implements-mechanics / environment buses (owner: structure).
%   Frames: world W = site ENU at P.world.lla0, body B = FLU, origin R = base_footprint
%   (ground contact plane, mid-gauge, mid contact length). The 6DOF integrates internally in
%   NED/FRD; conversion happens inside structure_truth.slx.
E = @bus_el;
d = bus_def('Bus_TruthState', 0.001, 'structure_plant/structure_truth (exported by Shaddad_Plant at 1 ms via ZOH)', ...
  'Shaddad_SensorModels, Sim_Monitors, Viz_Logging, logging (never used for control)', ...
  'True vehicle state at reference point R, ENU/FLU.', {
  E('t',           'double',1,    's',      'Simulation time')
  E('pos_W',       'double',3,    'm',      'Position of R in W (ENU)')
  E('vel_W',       'double',3,    'm/s',    'Velocity of R in W')
  E('acc_W',       'double',3,    'm/s^2',  'Kinematic acceleration of R in W (gravity excluded)')
  E('q_WB',        'double',4,    '1',      'MATLAB frame-rotation quaternion W->B, scalar first (v_B = rotateframe(q,v_W))')
  E('R_WB',        'double',[3 3],'1',      'DCM body->world (v_W = R_WB*v_B)')
  E('eul',         'double',3,    'rad',    'ZYX [psi theta phi]; psi CCW from East, theta + = nose DOWN, phi + = left side up')
  E('vel_B',       'double',3,    'm/s',    'Velocity of R in B (FLU)')
  E('omega_B',     'double',3,    'rad/s',  'Body rates [p q r] FLU, r > 0 = turning left')
  E('alpha_B',     'double',3,    'rad/s^2','Angular acceleration in B')
  E('acc_B',       'double',3,    'm/s^2',  'Kinematic acceleration of R in B (gravity excluded)')
  E('specForce_B', 'double',3,    'm/s^2',  'PHYSICAL specific force at R in B (a - g; reads [0 0 +g] at rest, level)')
  E('r_cg_B',      'double',3,    'm',      'CG position w.r.t. R in B (FLU)')
  E('lla',         'double',3,    '',       'Geodetic position of R [lat deg, lon deg, h m] (WGS-84)')
  E('pose2D',      'double',3,    '',       '[x_E m, y_N m, psi rad] of R (GNC convention)')
  E('speed_fwd',   'double',1,    'm/s',    'vel_B(1)')
  E('mass',        'double',1,    'kg',     'Current total mass (sprayer tank depletes)',100,400)
  E('terrainH',    'double',1,    'm',      'DEM height under R')
  E('grade_long',  'double',1,    'rad',    'Terrain slope along body x, + = uphill ahead')
  E('grade_lat',   'double',1,    'rad',    'Terrain cross slope, + = left side higher')
  E('vBelt_LR',    'double',2,    'm/s',    'Belt (sprocket pitch-line) speed [L R], + = forward')
  E('vGround_LR',  'double',2,    'm/s',    'Track ground speed along body x [L R]')
  E('slip_LR',     'double',2,    '1',      '(v_belt - v_ground)/max(|v_belt|,0.05) [L R]')
  });
d(end+1) = bus_def('Bus_TrackState', 0.001, 'structure_plant', 'logging, Sim_Monitors, tests', ...
  'Per-track terramechanics state; index 1 = left, 2 = right.', {
  E('v_belt',    'double',2,'m/s',  'Belt speed')
  E('v_ground',  'double',2,'m/s',  'Ground speed of the track along body x')
  E('slip',      'double',2,'1',    'Diagnostic slip ratio')
  E('s_shear',   'double',2,'1',    'Normalised longitudinal shear state',-1,1)
  E('Fx',        'double',2,'N',    'Traction force on vehicle, + forward')
  E('Fy',        'double',2,'N',    'Lateral force on vehicle (FLU y)')
  E('Fn',        'double',2,'N',    'Normal load per track',0,[])
  E('sprAngle',  'double',2,'rad',  'Sprocket angle (unwrapped)')
  E('sprSpeed',  'double',2,'rad/s','Sprocket speed, + forward')
  E('sprTorque', 'double',2,'N*m',  'Sprocket input torque from DRIVE')
  E('sinkage',   'double',2,'m',    'N-weighted mean penetration')
  E('muPeak',    'double',2,'1',    'Load-weighted peak traction coefficient')
  E('Mturn',     'double',1,'N*m',  'Lateral-shear turning moment about z')
  E('Pslip',     'double',2,'W',    'Slip power Fx*(v_belt - v_ground)')
  });
d(end+1) = bus_def('Bus_ImplementCmd', 0.01, 'Electronics_Plant/Actuator_Outputs_Sim (decoded MCU DO/PWM)', ...
  'structure_plant/structure_implement', 'Implement actuator commands seen by the mechanics.', {
  E('nozzleOn', 'boolean',8,'',   'Spray nozzle solenoid i open (DO bits 10..17)')
  E('liftDuty', 'double', 1,'1',  'Lift actuator duty, + = lower tool (DOWN), - = raise',-1,1)
  E('speedCmd', 'double', 1,'rad/s','Harness only (VC_IMPL_DRIVE==1 ideal drive): shaft speed command; 0 in system')
  });
d(end+1) = bus_def('Bus_ImplMechState', 0.01, 'structure_plant/structure_implement', ...
  'Power_Plant/Implement_Loads (ECU feedback), Electronics_Plant (lift pot ADC), logging', ...
  'Mechanical state of the active implement.', {
  E('type',        'uint8',  1,'',     '0 none, 1 mower, 2 tiller, 3 sprayer',0,3)
  E('shaftSpeed',  'double', 1,'rad/s','Impl_Shaft speed (blade / tiller rotor-side motor shaft / pump)')
  E('shaftTorque', 'double', 1,'N*m',  'Load torque on Impl_Shaft')
  E('mechPower',   'double', 1,'W',    'shaftSpeed*shaftTorque')
  E('toolHeight',  'double', 1,'m',    'Mower deck height above ground or tiller depth (+ down)')
  E('liftPos01',   'double', 1,'1',    'Lift actuator position normalised 0 (up) .. 1 (down) for the potentiometer',0,1)
  E('liftMoving',  'boolean',1,'',     'Lift actuator moving (|duty| > 0.05 and not at end stop)')
  E('tankLevel',   'double', 1,'L',    'Sprayer tank liquid volume',0,100)
  E('tankMass',    'double', 1,'kg',   'Sprayer tank liquid mass',0,100)
  E('sprayFlow',   'double', 1,'L/min','Total nozzle flow')
  E('linePressure','double', 1,'Pa',   'Boom line pressure',0,6e5)
  E('vegDensity',  'double', 1,'1',    'Vegetation density at the deck (P.world.veg)',0,3)
  E('F_impl_B',    'double', 3,'N',    'Implement force on chassis, body FLU')
  });
d(end+1) = bus_def('Bus_EnvDisturbance', 0.001, 'test harness (Constant zero struct in system)', 'structure_plant', ...
  'External wrench at CG for drawbar / push tests.', {
  E('F_ext_B','double',3,'N',  'External force at CG, body FLU')
  E('M_ext_B','double',3,'N*m','External moment about CG, body FLU')
  });
d(end+1) = bus_def('Bus_EnvState', 0.1, 'structure_plant/structure_env', ...
  'Power_Plant, drive_system (T_amb), Electronics_Plant, Shaddad_SensorModels, logging', ...
  'Ambient conditions at the robot and dock contact state (single producer of T_amb).', {
  E('T_amb',        'double', 1,'K',    'Ambient air temperature',233.15,343.15)
  E('T_sky',        'double', 1,'K',    'Effective sky temperature')
  E('T_ground',     'double', 1,'K',    'Ground surface temperature')
  E('G_solar',      'double', 1,'W/m^2','Global horizontal irradiance at the robot (zone-attenuated)',0,1400)
  E('soil_id',      'uint8',  1,'',     'Soil class under R (1..5)',1,5)
  E('dock_contact', 'boolean',1,'',     'Charger contacts closed (within P.world.dock tolerance and stopped, or override)')
  E('dist_dock',    'double', 1,'m',    'Distance from R to dock pose')
  });
d(end+1) = bus_def('Bus_StructureDiag', 0.01, 'structure_plant', 'logging, tests', ...
  'Contact-model diagnostics.', {
  E('N_pt',     'double',14,'N',  'Normal force per contact point (1..7 left, 8..14 right)')
  E('pen_pt',   'double',14,'m',  'Penetration per contact point')
  E('sX',       'double',14,'1',  'Longitudinal component of the isotropic 2-D shear state per point (D39)',-1,1)
  E('sY',       'double',14,'1',  'Lateral component of the isotropic 2-D shear state per point (D39)',-1,1)
  E('slopeLong','double',1, 'rad','Terrain slope along heading at R')
  E('slopeLat', 'double',1, 'rad','Terrain cross slope at R')
  E('P_diss',   'double',1, 'W',  'Track-terrain dissipation: 2-D shear slip + rolling resistance + internal track friction')
  E('E_mech',   'double',1, 'J',  'Kinetic (body + belts + sprocket/track inertia) + potential energy')
  E('E_diss_J', 'double',1, 'J',  'Integral of P_diss plus the work of the implement wrench on the chassis (D40)')
  E('E_impl_shaft_J','double',1,'J','Integral of the implement shaft power absorbed by the load mechanics (D40)')
  E('chi_eff',  'double',1, '1',  'Instantaneous effective gauge factor (v_bR - v_bL)/(B*r), NaN when |r| < 0.05 rad/s')
  E('outOfMap', 'boolean',1,'',   'R outside DEM extent (terrain clipped)')
  });
end
