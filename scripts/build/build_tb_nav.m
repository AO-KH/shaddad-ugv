function build_tb_nav()
%BUILD_TB_NAV  Build the navigation bench tests/sensors/tb_nav.slx (build task T12, build plan rule 10).
%
%   Chain (build plan T12): truth generator (robotmobilelib/Differential Drive Kinematic Model + track slip, side
%   slip and a planar slope) -> Shaddad_SensorModels (T11) -> sens_acq_imu (T11, MCU Task_5) + odometry emulation
%   with sens_odom_compute (T11, MCU Task_10) -> Model block Shaddad_NavFusion. Scan matching (normally
%   Shaddad_Perception, T13) is emulated at 0.2 s from the truth pose plus noise.
%   Root inputs (external input Dataset from tests/sensors/nav_tb_input.m):
%     1 prof double[8] 0.001 (interpolated): [v r vy sL sR chi gx gy]
%        v body-x ground speed, r body z rate, vy body-y skid velocity, sL/sR track slip (vGround = (1 - s) vTrack),
%        chi effective gauge factor (vGround_R - vGround_L = r chi B), slope plane z = gx x + gy y
%     2 drive_truth Bus_DriveTruth 0.001 (traction DC currents for the magnetometer disturbance)
%     3 env Bus_EnvState 0.1, 4 elec_env Bus_ElecEnv 0.1, 5 flt Bus_FaultInject 0.01, 6 obs Bus_ObstaclesDyn 0.01,
%     7 navcmd Bus_NavCmd 0.05, 8 bms Bus_BmsStatus 0.1, 9 smctl double[4] 0.2 [on sigXY sigPsi pFail]
%   Root outputs (yout, Dataset): 1 nav_state, 2 nav_diag, 3 truth10 double[15] 0.01
%     [t x y z psi theta phi v vy r sL sR chi vG_L vG_R], 4 gnss_raw, 5 sens_diag, 6 odom, 7 scanmatch.
%   Model workspace: tbn.pose0 [x y psi] initial pose, tbn.seedSm (4 seeds), tbn.seedVel (3 seeds of the bench
%   GNSS velocity noise, subsystem GnssVelNoise). Tests override tbn and P with
%   Simulink.SimulationInput.setVariable, so the shared base workspace is not modified.
%   Configuration: embedded copy of cs_SimFixed (ode3, 1 ms, multitasking).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out the bench and its three
%   subsystems with shaddad_layout(name, 'Recursive', true). Every signal line is named (L5, shaddad_bu('nline'); into a
%   bus-typed Bus Creator a line without a name takes its element's name). The root ports stay classic (nav_tb_input
%   and nav_tb_run use them by number); ScanMatchEmu picks truth with In Bus Element blocks (spec 6.2), GnssVelNoise
%   takes one Bus Selector per consumer of its classic port gnss_in, which also feeds the Bus Assignment whole.

name = 'tb_nav';
root = char(currentProject().RootFolder);
f = fullfile(root, 'tests', 'sensors', [name '.slx']);
if ~evalin('base', 'exist(''Bus_TruthState'',''var'') && exist(''Bus_ScanMatchMeas'',''var'')')
    shaddad_params();
end
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
    shaddad_configsets('fixed');
end
if bdIsLoaded(name), close_system(name, 0); end
for m = {'Shaddad_NavFusion', 'Shaddad_SensorModels', 'sens_imu_model', 'sens_spray_vision', 'sens_acq_imu', ...
         'sens_odom_compute'}
    if bdIsLoaded(m{1}) && strcmp(get_param(m{1}, 'Dirty'), 'on')
        try
            close_system(m{1}, 0);
        catch %#ok<CTCH> referenced by a loaded model of another task
        end
    end
end
shaddad_bu('fresh', name, f, 'model');
cs = copy(evalin('base', 'cs_SimFixed'));
cs.Name = 'tb_cfg';
attachConfigSet(name, cs, true);
setActiveConfigSet(name, 'tb_cfg');
try
    detachConfigSet(name, 'Configuration');
catch %#ok<CTCH> default set already gone
end
set_param(name, 'FixedStep', 'Ts.base', 'StopTime', '10', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', 'SaveTime', 'on', 'LoadExternalInput', 'off', 'SignalLogging', 'off', ...
    'ReturnWorkspaceOutputs', 'on', 'AlgebraicLoopMsg', 'error');
mws = get_param(name, 'ModelWorkspace');
assignin(mws, 'tbn', struct('pose0', [0 0 0], 'seedSm', [9101 9102 9103 9104], 'seedVel', [9201 9202 9203]));
n = name;

%% ---- root inputs ----
ins = {'prof', 'double', 'Ts.base', '8'; 'drive_truth', 'Bus: Bus_DriveTruth', 'Ts.base', '-1'; ...
       'env', 'Bus: Bus_EnvState', 'Ts.hk', '-1'; 'elec_env', 'Bus: Bus_ElecEnv', 'Ts.hk', '-1'; ...
       'flt', 'Bus: Bus_FaultInject', 'Ts.ctrl', '-1'; 'obs', 'Bus: Bus_ObstaclesDyn', 'Ts.ctrl', '-1'; ...
       'navcmd', 'Bus: Bus_NavCmd', 'Ts.gnc', '-1'; 'bms', 'Bus: Bus_BmsStatus', 'Ts.hk', '-1'; ...
       'smctl', 'double', 'Ts.scanmatch', '4'};
for k = 1:size(ins, 1)
    A(n, 'simulink/Sources/In1', ins{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ins{k, 2}, ...
      'SampleTime', ins{k, 3}, 'PortDimensions', ins{k, 4}, 'Interpolate', 'on');
end
set_param([n '/prof'], 'Interpolate', 'on');

%% ---- truth generator ----
local_truth(n);
L(n, 'prof/1', 'TruthGen/1', 'prof');

%% ---- devices under test ----
SSR = 'simulink/Ports & Subsystems/Subsystem Reference';
A(n, SSR, 'Sensors', 'ReferencedSubsystem', 'Shaddad_SensorModels');
L(n, 'TruthGen/1', 'Sensors/1', 'truth');
for k = 2:6, L(n, sprintf('%s/1', ins{k, 1}), sprintf('Sensors/%d', k), ins{k, 1}); end
% MCU-side IMU acquisition (Task_5): t_us from a digital clock, fault_inject b6 = (imu_fault == 3)
A(n, SSR, 'AcqIMU', 'ReferencedSubsystem', 'sens_acq_imu');
A(n, 'simulink/Sources/Digital Clock', 'Clock5', 'SampleTime', 'Ts.imu');
A(n, 'simulink/Math Operations/Gain', 'ToUs5', 'Gain', '1e6');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'Us5', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
  'SaturateOnIntegerOverflow', 'off');
L(n, 'Clock5/1', 'ToUs5/1', 't5');  L(n, 'ToUs5/1', 'Us5/1', 't5_us');
RT(n, 'RT_flt5', 'Ts.imu');  L(n, 'flt/1', 'RT_flt5/1');
A(n, 'simulink/Signal Routing/Bus Selector', 'FltImu', 'OutputSignals', 'imu_fault');  L(n, 'RT_flt5/1', 'FltImu/1', 'flt_5ms');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'ImuDrop', 'relop', '==', 'const', '3');
L(n, 'FltImu/1', 'ImuDrop/1');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'DropU32', 'OutDataTypeStr', 'uint32');
L(n, 'ImuDrop/1', 'DropU32/1', 'imu_dropout');
A(n, 'simulink/Math Operations/Gain', 'Bit6', 'Gain', 'P.sensors.acq.faultMask', 'OutDataTypeStr', 'uint32', ...
  'ParamDataTypeStr', 'uint32');
L(n, 'DropU32/1', 'Bit6/1');
L(n, 'Sensors/1', 'AcqIMU/1', 'imu_raw');  L(n, 'Us5/1', 'AcqIMU/2');  L(n, 'Bit6/1', 'AcqIMU/3', 'fault_inject');
% MCU-side odometry (Task_10) on the emulated ESC encoder counts
A(n, SSR, 'Odom', 'ReferencedSubsystem', 'sens_odom_compute');
C(n, 'WMotor', 'single([0 0])', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.ctrl');
A(n, 'simulink/Sources/Digital Clock', 'Clock10', 'SampleTime', 'Ts.ctrl');
A(n, 'simulink/Math Operations/Gain', 'ToUs10', 'Gain', '1e6');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'Us10', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
  'SaturateOnIntegerOverflow', 'off');
C(n, 'FbValid', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.ctrl');
L(n, 'Clock10/1', 'ToUs10/1', 't10');  L(n, 'ToUs10/1', 'Us10/1', 't10_us');
L(n, 'TruthGen/3', 'Odom/1', 'enc_cnt');  L(n, 'WMotor/1', 'Odom/2', 'wMotor');  L(n, 'Us10/1', 'Odom/3');
L(n, 'FbValid/1', 'Odom/4', 'fb_valid');
A(n, 'simulink/Sinks/Terminator', 'T_atrack');  L(n, 'Odom/2', 'T_atrack/1', 'a_track');
% scan-match emulation (stand-in for Shaddad_Perception scan-to-map matching)
local_scanmatch(n);
L(n, 'TruthGen/1', 'ScanMatchEmu/1');  L(n, 'smctl/1', 'ScanMatchEmu/2', 'smctl');
% navigation fusion
A(n, 'simulink/Ports & Subsystems/Model', 'Nav', 'ModelName', 'Shaddad_NavFusion');
local_gnss_velnoise(n);
L(n, 'Sensors/2', 'GnssVelNoise/1', 'gnss_raw_plant');
L(n, 'AcqIMU/1', 'Nav/1', 'imu_meas');  L(n, 'Odom/1', 'Nav/2', 'odom');  L(n, 'GnssVelNoise/1', 'Nav/3', 'gnss_raw');
L(n, 'ScanMatchEmu/1', 'Nav/4', 'scanmatch');  L(n, 'navcmd/1', 'Nav/5');  L(n, 'bms/1', 'Nav/6');
for k = [3 4 5]
    A(n, 'simulink/Sinks/Terminator', sprintf('T_sens%d', k));  L(n, sprintf('Sensors/%d', k), sprintf('T_sens%d/1', k));
end

%% ---- root outputs ----
outs = {'nav_state', 'Nav/1'; 'nav_diag', 'Nav/2'; 'truth10', 'TruthGen/2'; 'gnss_raw', 'GnssVelNoise/1'; ...
        'sens_diag', 'Sensors/6'; 'odom', 'Odom/1'; 'scanmatch', 'ScanMatchEmu/1'};
for k = 1:size(outs, 1)
    A(n, 'simulink/Sinks/Out1', ['o_' outs{k, 1}], 'Port', num2str(k));
    L(n, outs{k, 2}, ['o_' outs{k, 1} '/1'], outs{k, 1});
end
set_param(n, 'Description', ['T12 navigation bench: Differential Drive Kinematic Model truth (+ slip, side slip, slope) -> ' ...
    'Shaddad_SensorModels -> sens_acq_imu + sens_odom_compute -> Shaddad_NavFusion; scan matching emulated from the ' ...
    'truth pose. External inputs from tests/sensors/nav_tb_input.m; built by scripts/build/build_tb_nav.m.']);
shaddad_bu('save', n, f);
close_system(n, 0);
end

% ============================================================================================
function p = A(sys, lib, blk, varargin)
p = shaddad_bu('add', lib, [sys '/' blk], varargin{:});
end
function L(sys, src, dst, nm)
% signal into dst ('Blk/k'), named nm (layout spec 6.1, L5). src 'Blk/k': a line, without nm the default of
% shaddad_bu('nline') (the name the source port's line already has, else the source block's name, '_k' for output k of
% a block with several outputs; a Bus Selector's output keeps '<element>'). src 'port.element': a pick of that element
% of the In Bus Element port right in front of dst (spec 6.2), its line named nm (default the element), shaddad_bu('pick')
if nargin < 4, nm = ''; end
if contains(src, '/')
    shaddad_bu('nline', sys, src, dst, nm);
else
    shaddad_bu('pick', sys, extractBefore(src, '.'), extractAfter(src, '.'), dst, nm);
end
end
function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line src ('Blk/k'; the
% line named nm when given) and feeds its output k to dsts{k} ('Blk/j')
if nargin < 6, nm = ''; end
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end
function BC(sys, blk, bus, srcs)
% nonvirtual Bus Creator of type bus, srcs 'Blk/k' per element in bus order; a line without a name yet takes its
% element's name (L5)
b = evalin('base', bus);
A(sys, 'simulink/Signal Routing/Bus Creator', blk, 'Inputs', num2str(numel(srcs)), 'OutDataTypeStr', ['Bus: ' bus], ...
  'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(srcs)
    nm = b.Elements(k).Name;
    ph = get_param([sys '/' extractBefore(srcs{k}, '/')], 'PortHandles');
    old = get_param(ph.Outport(str2double(extractAfter(srcs{k}, '/'))), 'Line');
    if old ~= -1 && ~isempty(get_param(old, 'Name')), nm = ''; end
    L(sys, srcs{k}, sprintf('%s/%d', blk, k), nm);
end
end
function C(sys, blk, val, varargin)
A(sys, 'simulink/Sources/Constant', blk, 'Value', val, varargin{:});
end
function RT(sys, blk, ts)
A(sys, 'simulink/Signal Attributes/Rate Transition', blk, 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
end
function SEL(sys, blk, idx, w)
A(sys, 'simulink/Signal Routing/Selector', blk, 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'IndexParamArray', {idx}, 'InputPortWidth', num2str(w));
end
function CAT(sys, blk, n)
A(sys, 'simulink/Signal Routing/Vector Concatenate', blk, 'NumInputs', num2str(n), 'Mode', 'Vector');
end
function FCN(sys, blk, expr)
A(sys, 'simulink/User-Defined Functions/Fcn', blk, 'Expr', expr);
end
function s = SUB(parent, blk, ins, outs)
s = A(parent, 'simulink/Ports & Subsystems/Subsystem', blk);
lns = find_system(s, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(s, 'SearchDepth', 1), {s});
for k = 1:numel(b), delete_block(b{k}); end
for k = 1:numel(ins), add_block('simulink/Sources/In1', [s '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [s '/' outs{k}], 'Port', num2str(k)); end
end

% --------------------------------------------------------------------------------------------
function local_truth(n)
% Truth at 1 ms: planar motion on the slope plane z = gx x + gy y (body z = plane normal, omega_B = [0 0 r]).
%   theta = -atan(gx cos(psi) + gy sin(psi)),  phi = atan((-gx sin(psi) + gy cos(psi)) cos(theta))
%   psidot = r cos(phi)/cos(theta); horizontal speed along the heading v cos(theta) + vy sin(theta) sin(phi)
%   (Differential Drive Kinematic Model, 'Vehicle Speed & Heading Angular Velocity'), across it vy cos(phi)
%   (Discrete-Time Integrator); vel_W = R_WB [v; vy; 0], acc_W / alpha_B by Discrete Derivative.
s = SUB(n, 'TruthGen', {'prof'}, {'truth', 'truth10', 'enc_cnt'});
tb = 'Ts.base';
names = {'v', 'r', 'vy', 'sL', 'sR', 'chi', 'gx', 'gy'};
for k = 1:8, SEL(s, names{k}, num2str(k), 8); L(s, 'prof/1', [names{k} '/1'], 'prof'); end
% ---- Differential Drive Kinematic Model ----
A(s, 'robotmobilelib/Differential Drive Kinematic Model', 'DiffDrive', ...
  'VehicleInputs', 'Vehicle Speed & Heading Angular Velocity', 'InitialState', 'tbn.pose0(:)', ...
  'TrackWidth', 'P.sensors.geom.B', 'SimulateUsing', 'Code generation');
SEL(s, 'X', '1', 3);  SEL(s, 'Y', '2', 3);  SEL(s, 'Psi', '3', 3);
A(s, 'simulink/Math Operations/Reshape', 'StateVec', 'OutputDimensionality', '1-D array');
L(s, 'DiffDrive/1', 'StateVec/1', 'state');
for b = {'X', 'Y', 'Psi'}, L(s, 'StateVec/1', [b{1} '/1']); end
A(s, 'simulink/Sinks/Terminator', 'T_stateDot');  L(s, 'DiffDrive/2', 'T_stateDot/1');
% ---- attitude on the slope plane ----
CAT(s, 'AttIn', 3);  L(s, 'gx/1', 'AttIn/1', 'gx');  L(s, 'gy/1', 'AttIn/2', 'gy');  L(s, 'Psi/1', 'AttIn/3', 'psi');
FCN(s, 'Theta', '-atan(u(1)*cos(u(3)) + u(2)*sin(u(3)))');  L(s, 'AttIn/1', 'Theta/1');
CAT(s, 'PhiIn', 4);  L(s, 'gx/1', 'PhiIn/1');  L(s, 'gy/1', 'PhiIn/2');  L(s, 'Psi/1', 'PhiIn/3');  L(s, 'Theta/1', 'PhiIn/4', 'theta');
FCN(s, 'Phi', 'atan((-u(1)*sin(u(3)) + u(2)*cos(u(3)))*cos(u(4)))');  L(s, 'PhiIn/1', 'Phi/1');
% the kinematic-model inputs use the attitude of the previous 1 ms step (no algebraic loop through psi)
A(s, 'simulink/Discrete/Unit Delay', 'Att_z', 'InitialCondition', '[0; 0]', 'SampleTime', tb);
CAT(s, 'Att', 2);  L(s, 'Theta/1', 'Att/1');  L(s, 'Phi/1', 'Att/2', 'phi');  L(s, 'Att/1', 'Att_z/1');
CAT(s, 'KinIn', 4);  L(s, 'v/1', 'KinIn/1', 'v');  L(s, 'vy/1', 'KinIn/2', 'vy');  L(s, 'r/1', 'KinIn/3', 'r');
L(s, 'Att_z/1', 'KinIn/4', 'att_prev');
FCN(s, 'Vh', 'u(1)*cos(u(4)) + u(2)*sin(u(4))*sin(u(5))');  L(s, 'KinIn/1', 'Vh/1');
FCN(s, 'PsiDot', 'u(3)*cos(u(5))/cos(u(4))');  L(s, 'KinIn/1', 'PsiDot/1');
L(s, 'Vh/1', 'DiffDrive/1', 'v_horizontal');  L(s, 'PsiDot/1', 'DiffDrive/2', 'psidot');
% ---- lateral (skid) displacement ----
CAT(s, 'LatIn', 3);  L(s, 'vy/1', 'LatIn/1');  L(s, 'Att_z/1', 'LatIn/2');  L(s, 'Psi/1', 'LatIn/3');
FCN(s, 'LatE', '-u(1)*cos(u(3))*sin(u(4))');  FCN(s, 'LatN', 'u(1)*cos(u(3))*cos(u(4))');
L(s, 'LatIn/1', 'LatE/1');  L(s, 'LatIn/1', 'LatN/1');
CAT(s, 'LatV', 2);  L(s, 'LatE/1', 'LatV/1');  L(s, 'LatN/1', 'LatV/2');
A(s, 'simulink/Discrete/Discrete-Time Integrator', 'LatInt', 'IntegratorMethod', 'Integration: Forward Euler', ...
  'InitialCondition', '[0; 0]', 'SampleTime', tb);
L(s, 'LatV/1', 'LatInt/1', 'v_lateral_EN');
SEL(s, 'LatX', '1', 2);  SEL(s, 'LatY', '2', 2);  L(s, 'LatInt/1', 'LatX/1');  L(s, 'LatInt/1', 'LatY/1');
A(s, 'simulink/Math Operations/Sum', 'PE', 'Inputs', '++');  L(s, 'X/1', 'PE/1');  L(s, 'LatX/1', 'PE/2');
A(s, 'simulink/Math Operations/Sum', 'PN', 'Inputs', '++');  L(s, 'Y/1', 'PN/1');  L(s, 'LatY/1', 'PN/2');
CAT(s, 'ZIn', 4);  L(s, 'gx/1', 'ZIn/1');  L(s, 'gy/1', 'ZIn/2');  L(s, 'PE/1', 'ZIn/3', 'pE');  L(s, 'PN/1', 'ZIn/4', 'pN');
FCN(s, 'PU', 'u(1)*u(3) + u(2)*u(4)');  L(s, 'ZIn/1', 'PU/1');
CAT(s, 'Pos', 3);  L(s, 'PE/1', 'Pos/1');  L(s, 'PN/1', 'Pos/2');  L(s, 'PU/1', 'Pos/3', 'pU');
% ---- orientation ----
TRIG(s, 'PsiS', 'sin');  TRIG(s, 'PsiC', 'cos');  TRIG(s, 'PsiW', 'atan2');
L(s, 'Psi/1', 'PsiS/1');  L(s, 'Psi/1', 'PsiC/1');  L(s, 'PsiS/1', 'PsiW/1');  L(s, 'PsiC/1', 'PsiW/2');
CAT(s, 'Eul', 3);  L(s, 'PsiW/1', 'Eul/1', 'psi_wrapped');  L(s, 'Theta/1', 'Eul/2');  L(s, 'Phi/1', 'Eul/3');
A(s, 'simulink/Math Operations/Reshape', 'EulCol', 'OutputDimensionality', 'Column vector (2-D)');  L(s, 'Eul/1', 'EulCol/1', 'eul');
A(s, 'navlib/Utilities/Coordinate Transformation Conversion', 'Eul2R', 'InputRepresentation', 'Euler Angles', ...
  'InputEulerSequence', 'ZYX', 'OutputRepresentation', 'Rotation Matrix');
A(s, 'navlib/Utilities/Coordinate Transformation Conversion', 'Eul2Q', 'InputRepresentation', 'Euler Angles', ...
  'InputEulerSequence', 'ZYX', 'OutputRepresentation', 'Quaternion');
L(s, 'EulCol/1', 'Eul2R/1');  L(s, 'EulCol/1', 'Eul2Q/1');
A(s, 'simulink/Math Operations/Reshape', 'QVec', 'OutputDimensionality', '1-D array');  L(s, 'Eul2Q/1', 'QVec/1', 'q_WB');
% ---- velocities, rates, accelerations ----
C(s, 'Zero', '0');
CAT(s, 'VelB', 3);  L(s, 'v/1', 'VelB/1');  L(s, 'vy/1', 'VelB/2');  L(s, 'Zero/1', 'VelB/3');
A(s, 'simulink/Math Operations/Reshape', 'VelBCol', 'OutputDimensionality', 'Column vector (2-D)');  L(s, 'VelB/1', 'VelBCol/1', 'vel_B');
A(s, 'simulink/Math Operations/Product', 'VelW', 'Inputs', '2', 'Multiplication', 'Matrix(*)');
L(s, 'Eul2R/1', 'VelW/1', 'R_WB');  L(s, 'VelBCol/1', 'VelW/2');
A(s, 'simulink/Math Operations/Reshape', 'VelWVec', 'OutputDimensionality', '1-D array');  L(s, 'VelW/1', 'VelWVec/1', 'vel_W');
CAT(s, 'OmegaB', 3);  L(s, 'Zero/1', 'OmegaB/1');  L(s, 'Zero/1', 'OmegaB/2');  L(s, 'r/1', 'OmegaB/3');
A(s, 'simulink/Discrete/Discrete Derivative', 'AccW', 'gainval', '1', 'ICPrevScaledInput', '0');
L(s, 'VelWVec/1', 'AccW/1');
A(s, 'simulink/Discrete/Discrete Derivative', 'AlphaB', 'gainval', '1', 'ICPrevScaledInput', '0');
L(s, 'OmegaB/1', 'AlphaB/1', 'omega_B');
A(s, 'simulink/Math Operations/Math Function', 'Rt', 'Operator', 'transpose');  L(s, 'Eul2R/1', 'Rt/1');
A(s, 'simulink/Math Operations/Reshape', 'AccWCol', 'OutputDimensionality', 'Column vector (2-D)');  L(s, 'AccW/1', 'AccWCol/1', 'acc_W');
A(s, 'simulink/Math Operations/Product', 'AccB', 'Inputs', '2', 'Multiplication', 'Matrix(*)');
L(s, 'Rt/1', 'AccB/1', 'R_BW');  L(s, 'AccWCol/1', 'AccB/2');
C(s, 'GravW', '[0; 0; P.frames.g]');
A(s, 'simulink/Math Operations/Sum', 'AccPlusG', 'Inputs', '++');  L(s, 'AccWCol/1', 'AccPlusG/1');  L(s, 'GravW/1', 'AccPlusG/2');
A(s, 'simulink/Math Operations/Product', 'SfB', 'Inputs', '2', 'Multiplication', 'Matrix(*)');
L(s, 'Rt/1', 'SfB/1');  L(s, 'AccPlusG/1', 'SfB/2');
for b = {'AccBVec', 'SfBVec'}
    A(s, 'simulink/Math Operations/Reshape', b{1}, 'OutputDimensionality', '1-D array');
end
L(s, 'AccB/1', 'AccBVec/1', 'acc_B');  L(s, 'SfB/1', 'SfBVec/1', 'specForce_B');
% ---- tracks: vGround = v -/+ r chi B/2, vTrack = vGround/(1 - s) ----
CAT(s, 'TrkIn', 6);
for k = 1:6
    b = {'v', 'r', 'chi', 'sL', 'sR', 'Gauge'};
    if k == 6, C(s, 'Gauge', 'P.sensors.geom.B'); end
    L(s, [b{k} '/1'], sprintf('TrkIn/%d', k));
end
FCN(s, 'VgL', 'u(1) - u(2)*u(3)*u(6)/2');  FCN(s, 'VgR', 'u(1) + u(2)*u(3)*u(6)/2');
FCN(s, 'VtL', '(u(1) - u(2)*u(3)*u(6)/2)/(1 - u(4))');  FCN(s, 'VtR', '(u(1) + u(2)*u(3)*u(6)/2)/(1 - u(5))');
for b = {'VgL', 'VgR', 'VtL', 'VtR'}, L(s, 'TrkIn/1', [b{1} '/1']); end
CAT(s, 'VGround', 2);  L(s, 'VgL/1', 'VGround/1', 'vG_L');  L(s, 'VgR/1', 'VGround/2', 'vG_R');
CAT(s, 'VBelt', 2);  L(s, 'VtL/1', 'VBelt/1', 'vT_L');  L(s, 'VtR/1', 'VBelt/2', 'vT_R');
CAT(s, 'Slip', 2);  L(s, 'sL/1', 'Slip/1', 'sL');  L(s, 'sR/1', 'Slip/2', 'sR');
% ---- emulated ESC encoder counts (DRV_FB3, int32 wrapping) at 0.01 s ----
A(s, 'simulink/Discrete/Discrete-Time Integrator', 'CntInt', 'IntegratorMethod', 'Integration: Forward Euler', ...
  'gainval', '1/P.electronics.drv.K_enc', 'InitialCondition', '[0; 0]', 'SampleTime', tb);
L(s, 'VBelt/1', 'CntInt/1', 'vTrack');
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_cnt', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', 'Ts.ctrl');
L(s, 'CntInt/1', 'RT_cnt/1', 'counts');
A(s, 'simulink/Math Operations/Rounding Function', 'CntRound', 'Operator', 'floor');  L(s, 'RT_cnt/1', 'CntRound/1');
A(s, 'simulink/Math Operations/Bias', 'CntShift', 'Bias', '2^31');  L(s, 'CntRound/1', 'CntShift/1');
C(s, 'TwoPow32', '2^32');
A(s, 'simulink/Math Operations/Math Function', 'CntMod', 'Operator', 'mod');
L(s, 'CntShift/1', 'CntMod/1');  L(s, 'TwoPow32/1', 'CntMod/2');
A(s, 'simulink/Math Operations/Bias', 'CntBack', 'Bias', '-2^31');  L(s, 'CntMod/1', 'CntBack/1');
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'CntI32', 'OutDataTypeStr', 'int32');
L(s, 'CntBack/1', 'CntI32/1');  L(s, 'CntI32/1', 'enc_cnt/1', 'enc_cnt');
% ---- Bus_TruthState ----
A(s, 'simulink/Sources/Digital Clock', 'Clock', 'SampleTime', tb);
C(s, 'Zero3', 'zeros(3, 1)', 'VectorParams1D', 'on');
C(s, 'LLA0', 'P.world.lla0(:)', 'VectorParams1D', 'on');
C(s, 'Mass', 'P.structure.mp.m0');
CAT(s, 'Pose2D', 3);  L(s, 'PE/1', 'Pose2D/1');  L(s, 'PN/1', 'Pose2D/2');  L(s, 'PsiW/1', 'Pose2D/3');
src = {'Clock/1', 'Pos/1', 'VelWVec/1', 'AccW/1', 'QVec/1', 'Eul2R/1', 'Eul/1', 'VelB/1', 'OmegaB/1', 'AlphaB/1', ...
       'AccBVec/1', 'SfBVec/1', 'Zero3/1', 'LLA0/1', 'Pose2D/1', 'v/1', 'Mass/1', 'PU/1', 'Zero/1', 'Zero2/1', ...
       'VBelt/1', 'VGround/1', 'Slip/1'};
% grade_lat takes its own Constant: with every line named (L5), a second branch of Zero's line into BC_Truth would warn
% about duplicated names at the Bus Creator's inputs
C(s, 'Zero2', '0');
BC(s, 'BC_Truth', 'Bus_TruthState', src);
L(s, 'BC_Truth/1', 'truth/1', 'truth');
% ---- truth log at 0.01 s: [t x y z psi theta phi v vy r sL sR chi vG_L vG_R] ----
CAT(s, 'Log', 11);
src = {'Clock/1', 'Pos/1', 'Eul/1', 'v/1', 'vy/1', 'r/1', 'sL/1', 'sR/1', 'chi/1', 'VgL/1', 'VgR/1'};
for k = 1:numel(src), L(s, src{k}, sprintf('Log/%d', k)); end
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_log', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', 'Ts.nav');
L(s, 'Log/1', 'RT_log/1', 'truth_log');  L(s, 'RT_log/1', 'truth10/1', 'truth10');
end

function TRIG(sys, blk, op)
A(sys, 'simulink/Math Operations/Trigonometric Function', blk, 'Operator', op);
end

% --------------------------------------------------------------------------------------------
function local_scanmatch(n)
% Bus_ScanMatchMeas at 0.2 s: pose = truth [x y psi] + N(0, diag(sig^2)), cov = diag(sig^2), valid = on and
% not dropped (uniform < pFail); t = truth time of the scan
% in truth (In Bus Element: t, pose2D), smctl; out scanmatch
s = SUB(n, 'ScanMatchEmu', {'smctl'}, {'scanmatch'});
ts = 'Ts.scanmatch';
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_t', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
A(s, 'simulink/Signal Attributes/Rate Transition', 'RT_pose', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
L(s, 'truth.t', 'RT_t/1', 't');  L(s, 'truth.pose2D', 'RT_pose/1', 'pose2D');
SEL(s, 'On', '1', 4);  SEL(s, 'SigXY', '2', 4);  SEL(s, 'SigPsi', '3', 4);  SEL(s, 'PFail', '4', 4);
for b = {'On', 'SigXY', 'SigPsi', 'PFail'}, L(s, 'smctl/1', [b{1} '/1'], 'smctl'); end
CAT(s, 'Sig', 3);  L(s, 'SigXY/1', 'Sig/1');  L(s, 'SigXY/1', 'Sig/2');  L(s, 'SigPsi/1', 'Sig/3');
A(s, 'simulink/Sources/Random Number', 'Noise', 'Mean', '[0 0 0]', 'Variance', '[1 1 1]', 'Seed', 'tbn.seedSm(1:3)', ...
  'SampleTime', ts);
A(s, 'simulink/Math Operations/Product', 'NoiseS', 'Inputs', '2');  L(s, 'Noise/1', 'NoiseS/1');  L(s, 'Sig/1', 'NoiseS/2', 'sig');
A(s, 'simulink/Math Operations/Sum', 'Pose', 'Inputs', '++');  L(s, 'RT_pose/1', 'Pose/1', 'pose_true');  L(s, 'NoiseS/1', 'Pose/2');
A(s, 'simulink/Math Operations/Product', 'Var', 'Inputs', '2');  L(s, 'Sig/1', 'Var/1');  L(s, 'Sig/1', 'Var/2');
SEL(s, 'Var1', '1', 3);  SEL(s, 'Var2', '2', 3);  SEL(s, 'Var3', '3', 3);
for b = {'Var1', 'Var2', 'Var3'}, L(s, 'Var/1', [b{1} '/1']); end
C(s, 'Z0', '0', 'SampleTime', ts);
CAT(s, 'CovV', 9);
src = {'Var1/1', 'Z0/1', 'Z0/1', 'Z0/1', 'Var2/1', 'Z0/1', 'Z0/1', 'Z0/1', 'Var3/1'};
for k = 1:9, L(s, src{k}, sprintf('CovV/%d', k)); end
A(s, 'simulink/Math Operations/Reshape', 'Cov', 'OutputDimensionality', 'Customize', 'OutputDimensions', '[3 3]');
L(s, 'CovV/1', 'Cov/1');
A(s, 'simulink/Sources/Uniform Random Number', 'UFail', 'Minimum', '0', 'Maximum', '1', 'Seed', 'tbn.seedSm(4)', ...
  'SampleTime', ts);
A(s, 'simulink/Logic and Bit Operations/Relational Operator', 'NotFail', 'Operator', '>=');
L(s, 'UFail/1', 'NotFail/1');  L(s, 'PFail/1', 'NotFail/2');
A(s, 'simulink/Logic and Bit Operations/Compare To Constant', 'OnB', 'relop', '>', 'const', '0.5');  L(s, 'On/1', 'OnB/1');
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Valid', 'Operator', 'AND', 'Inputs', '2');
L(s, 'OnB/1', 'Valid/1');  L(s, 'NotFail/1', 'Valid/2');
C(s, 'Score', '0', 'SampleTime', ts);  C(s, 'NPts', 'uint16(270)', 'OutDataTypeStr', 'uint16', 'SampleTime', ts);
BC(s, 'BC_Sm', 'Bus_ScanMatchMeas', {'Valid/1', 'RT_t/1', 'Pose/1', 'Cov/1', 'Score/1', 'NPts/1'});
L(s, 'BC_Sm/1', 'scanmatch/1', 'scanmatch');
shaddad_bu('portorder', s, {'truth', 'smctl'}, 'Inport');
end

% --------------------------------------------------------------------------------------------
function local_gnss_velnoise(n)
% Zero-mean white GNSS velocity noise added on the bench side (sigma = reported sAcc of the fix:
% P.sensors.gnss.accFixed / accFloat / accSingle(3)). The tests switch the velocity noise of the plant GPS blocks
% off (P.sensors.gnss.sigVel = 0, nav_tb_run): mspsensorlib/GPS (gpsSensor) adds the magnitude of its velocity
% noise to the ground speed, an along-track bias of 1.2533 VelocityAccuracy (+2.5 cm/s RTK fixed), which a real
% receiver does not have (T12 finding, reported to T11).
s = SUB(n, 'GnssVelNoise', {'gnss_in'}, {'gnss'});
ts = 'Ts.gnss';
A(s, 'simulink/Signal Attributes/Data Type Conversion', 'FixD', 'OutDataTypeStr', 'double');
PICK(s, 'gnss_in/1', 'Sel_fixType', {'fixType'}, {'FixD/1'}, 'gnss');
A(s, 'simulink/Lookup Tables/1-D Lookup Table', 'SigVel', 'BreakpointsForDimension1', '0:5', ...
  'Table', ['[P.sensors.gnss.accSingle(3) P.sensors.gnss.accSingle(3) P.sensors.gnss.accSingle(3) ' ...
            'P.sensors.gnss.accSingle(3) P.sensors.gnss.accFixed(3) P.sensors.gnss.accFloat(3)]'], ...
  'InterpMethod', 'Nearest', 'ExtrapMethod', 'Clip');
L(s, 'FixD/1', 'SigVel/1');
A(s, 'simulink/Sources/Random Number', 'Noise', 'Mean', '[0 0 0]', 'Variance', '[1 1 1]', 'Seed', 'tbn.seedVel', ...
  'SampleTime', ts);
A(s, 'simulink/Math Operations/Product', 'NoiseS', 'Inputs', '2');  L(s, 'Noise/1', 'NoiseS/1');  L(s, 'SigVel/1', 'NoiseS/2', 'sigma_vel');
A(s, 'simulink/Math Operations/Sum', 'Vel', 'Inputs', '++');  PICK(s, 'gnss_in/1', 'Sel_vel_ENU', {'vel_ENU'}, {'Vel/1'});
L(s, 'NoiseS/1', 'Vel/2');
A(s, 'simulink/Signal Routing/Bus Assignment', 'Assign', 'AssignedSignals', 'vel_ENU');
L(s, 'gnss_in/1', 'Assign/1');  L(s, 'Vel/1', 'Assign/2', 'vel_noisy');  L(s, 'Assign/1', 'gnss/1', 'gnss');
end
