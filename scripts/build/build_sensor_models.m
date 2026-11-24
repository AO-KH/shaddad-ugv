function build_sensor_models(which)
%BUILD_SENSOR_MODELS  Build the sensor plant models and the MCU-side sensor acquisition files (build task T11).
%
%   build_sensor_models()            build all files below (children first) and create sens_uavScene in base
%   build_sensor_models(names)       build only the listed files (char or cellstr of the names below)
%
%   Files (Subsystem Reference, subsystems/sensors/), programmatic and idempotent (build plan rule 3):
%     sens_imu_model      ICM-20948 IMU at 0.005 s: mspsensorlib/IMU + lever arm + misalignment + vibration +
%                         gravity correction + Gain -1 + DLPF + magnetometer current disturbance + faults
%     sens_gnss_model     dual ZED-F9P at 0.1 s: antenna lever arm, 0.08 s latency (Delay 8 at 0.01 s), sky view
%                         by 2-D Lookup Tables on P.world.gnssSvf / gnssZoneId (D59), MATLAB Function
%                         sens_gnss_fixchain, three mspsensorlib/GPS (FIXED/FLOAT/SINGLE) + Multiport Switch,
%                         zero-mean velocity noise per axis (VelNoise, not the GPS block's own), reported
%                         accuracies, heading noise, outliers
%     sens_lidar_model    Variant Subsystem VC_LIDAR: FAST2D (MATLAB Function sens_lidar2d_fast) | FULL3D
%                         (uavsimlib UAV Scenario Configuration / Motion Write / Lidar on sens_uavScene)
%     sens_spray_vision   abstract vision-sprayer detector at 0.1 s (blocks only, D24)
%     Shaddad_SensorModels  plant-side top (ICD 3.1 Sensors row), children as Subsystem References
%     sens_acq_imu        MCU Task_5 (single): Bus_ImuRaw -> axis map, calibration, t_us, seq, valid -> Bus_ImuMeas
%     sens_odom_compute   MCU Task_10 (single): DRV_FB3 encoder counts -> wrap-safe window difference -> Bus_OdomMeas
%   Needs P, Ts, Bus_* in base (shaddad_params). Every number comes from P.sensors / P.world / Ts.
%   The two MATLAB Function blocks (FixChain, Scan2D) have no parameter data: their constants arrive on inputs
%   from Constant blocks (sens_gnss_fixchain_prm / sens_lidar2d_fast_prm and P.world.obstacles tables).
%   R2026b note: a MATLAB Function block two Subsystem Reference levels below a model (Shaddad_System ->
%   Shaddad_SensorModels -> sens_gnss_model / sens_lidar_model -> MATLAB Function) fails the FIRST compile
%   after its SSR files are loaded with 'Stateflow:Assertion o != nullptr'
%   (Stateflow.JIT.Utils.relinkToExistingJITEngine), reproduced in T11 (parameter-free blocks) and in
%   T03 / T07. Therefore (same pattern as T07 RC_Link_Sim / rc_uart_line_sim): Shaddad_SensorModels holds
%   the GNSS and LiDAR contents INLINE (plain Subsystems built by the same builder functions
%   local_sens_gnss_model / local_sens_lidar_model, so they are identical to sens_gnss_model.slx /
%   sens_lidar_model.slx; t_sensors/tInterfaces checks it), which puts both MATLAB Function blocks one SSR
%   level below any model. IMU and SprayVision (no MATLAB Function) stay Subsystem References.
%   sens_gnss_model.slx / sens_lidar_model.slx remain the standalone files of the two sensors.
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of each
%   file with shaddad_layout(<file>, 'Recursive', true), the inline GNSS and LiDAR and the Fidelity choices
%   included. Every signal line is named (L5, shaddad_bu('nline')). The root ports of every file stay classic
%   Inports and Outports; each consumer of a bus takes its elements from its own Bus Selector right in front of it
%   (spec 6.2, helper PICK): Truth_<element>, Flt_<element>, Drive_<element>, Raw_<element> (a further pick of
%   the same element _2, _3), while Corr / Flt (FixChain) and Truth / Flt / Obs (Fidelity) keep their names.

names = {'sens_imu_model', 'sens_gnss_model', 'sens_lidar_model', 'sens_spray_vision', ...
       'Shaddad_SensorModels', 'sens_acq_imu', 'sens_odom_compute'};
if nargin < 1 || isempty(which), which = names; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_ImuRaw'',''var'')')
    shaddad_params();
end
root = char(currentProject().RootFolder);
dirS = fullfile(root, 'subsystems', 'sensors');
% FULL3D scenario object (UAV Scenario Configuration reads it from the base workspace)
if any(strcmp(which, 'sens_lidar_model')) || any(strcmp(which, 'Shaddad_SensorModels'))
    assignin('base', 'sens_uavScene', sens_env_to_uavscenario(evalin('base', 'P')));
end
% the sensor bench and the sensor files themselves (owned by this task) are closed unsaved so they can be
% replaced (a loaded parent SSR is marked dirty when a child file is rebuilt); models of other tasks are never
% closed here (build plan rule 1)
for m = [{'tb_sens_plant', 'Shaddad_SensorModels'}, names]
    if bdIsLoaded(m{1})
        try
            close_system(m{1}, 0);
        catch %#ok<CTCH> still referenced by a loaded model of another task
        end
    end
end
for k = 1:numel(names)
    if ~any(strcmp(which, names{k})), continue, end
    f = fullfile(dirS, [names{k} '.slx']);
    shaddad_bu('fresh', names{k}, f, 'ssr');
    feval(['local_' names{k}], names{k});
    shaddad_bu('save', names{k}, f);
    close_system(names{k}, 0);
end
end

% ============================================================================================
% helpers
% ============================================================================================
function p = A(sys, lib, blk, varargin)
p = shaddad_bu('add', lib, [sys '/' blk], varargin{:});
end
function L(sys, src, dst, nm)
% a named signal line (layout spec 6.1, L5): shaddad_bu('nline') names the line nm, else keeps the name its source
% port's line already has, else names it after its source block; a Bus Selector's output keeps '<element>'
if nargin < 4, nm = ''; end
shaddad_bu('nline', sys, src, dst, nm);
end
function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (layout spec 6.2): it selects sigs from the bus line src
% ('Blk/k'; the line named nm when given) and feeds its output k to dsts{k} ('Blk/j')
if nargin < 6, nm = ''; end
BS(sys, blk, sigs);
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end
function RT(sys, blk, ts)
A(sys, 'simulink/Signal Attributes/Rate Transition', blk, 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
end
function BS(sys, blk, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
end
function BC(sys, blk, bus, srcs)
% nonvirtual Bus Creator, srcs = 'Blk/port' per element in bus order (lines named by element)
b = evalin('base', bus);
nm = {b.Elements.Name};
A(sys, 'simulink/Signal Routing/Bus Creator', blk, 'Inputs', num2str(numel(nm)), ...
  'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(nm)
    L(sys, srcs{k}, sprintf('%s/%d', blk, k), nm{k});
end
end
function RESH(sys, blk, dimty)
% Reshape: 'col' -> column vector (2-D), 'row' -> row vector (2-D), '1d' -> 1-D array
switch dimty
    case 'col', d = 'Column vector (2-D)';
    case 'row', d = 'Row vector (2-D)';
    otherwise,  d = '1-D array';
end
A(sys, 'simulink/Math Operations/Reshape', blk, 'OutputDimensionality', d);
end
function MATMUL(sys, blk)
A(sys, 'simulink/Math Operations/Product', blk, 'Inputs', '2', 'Multiplication', 'Matrix(*)');
end
function C(sys, blk, val, varargin)
A(sys, 'simulink/Sources/Constant', blk, 'Value', val, varargin{:});
end
function DTC(sys, blk, ty, varargin)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', blk, 'OutDataTypeStr', ty, ...
  'RndMeth', 'Nearest', 'SaturateOnIntegerOverflow', 'off', varargin{:});
end
function SEL(sys, blk, idx, w)
A(sys, 'simulink/Signal Routing/Selector', blk, 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', idx, 'InputPortWidth', num2str(w));
end
function IN(sys, blk, port, ty, ts, dims)
if nargin < 6, dims = '-1'; end
A(sys, 'simulink/Sources/In1', blk, 'Port', num2str(port), 'OutDataTypeStr', ty, 'SampleTime', ts, ...
  'PortDimensions', dims);
end
function OUT(sys, blk, port, ty)
A(sys, 'simulink/Sinks/Out1', blk, 'Port', num2str(port), 'OutDataTypeStr', ty);
end
function TERM(sys, src, blk, nm)
if nargin < 4, nm = ''; end
A(sys, 'simulink/Sinks/Terminator', blk);
L(sys, src, [blk '/1'], nm);
end
function MF(sys, blk, script, desc)
% MATLAB Function block with its body in a %#codegen .m file (one-line call). It has NO parameter data: every
% constant arrives on an input from a Constant block (expressions of P sub-structs, rule 2). In R2026b a
% MATLAB Function parameter (masked, tunable or not) two Subsystem Reference levels below a model
% (Shaddad_System -> Shaddad_SensorModels -> sens_*_model) fails the first compile after a rebuild with
% 'Stateflow:Assertion o != nullptr' in cdr_get_param_props_for_jit_registration (reproduced for both blocks).
mf = A(sys, 'simulink/User-Defined Functions/MATLAB Function', blk);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = script;
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = false;
cfg.Description = desc;
end
function XPROD(sys, blk)
A(sys, 'aerolibutil/3x3 Cross Product', blk);
end

% ============================================================================================
% sens_imu_model
% ============================================================================================
function local_sens_imu_model(n)
ts = 'Ts.imu';
IN(n, 'truth', 1, 'Bus: Bus_TruthState', 'Ts.base');
IN(n, 'drive_truth', 2, 'Bus: Bus_DriveTruth', 'Ts.base');
IN(n, 'elec_env', 3, 'Bus: Bus_ElecEnv', 'Ts.hk');
IN(n, 'flt', 4, 'Bus: Bus_FaultInject', 'Ts.ctrl');
OUT(n, 'imu_raw', 1, 'Bus: Bus_ImuRaw');

%% ---- truth at 200 Hz (the bus truth_5ms; each consumer picks its element: Bus Selectors Truth_<element>) ----
RT(n, 'RT_truth', ts);  L(n, 'truth/1', 'RT_truth/1', 'truth');
tr = 'RT_truth/1';

%% ---- lever arm: a_rel_B = alpha x r + omega x (omega x r) + a_vib ----
C(n, 'r_I', 'P.sensors.imu.r');
XPROD(n, 'AlphaXr');  XPROD(n, 'OmegaXr');  XPROD(n, 'OmegaXOmegaXr');
PICK(n, tr, 'Truth_alpha_B', {'alpha_B'}, {'AlphaXr/1'}, 'truth_5ms');  L(n, 'r_I/1', 'AlphaXr/2', 'r_I');
PICK(n, tr, 'Truth_omega_B', {'omega_B'}, {'OmegaXr/1'});               L(n, 'r_I/1', 'OmegaXr/2');
PICK(n, tr, 'Truth_omega_B_2', {'omega_B'}, {'OmegaXOmegaXr/1'});       L(n, 'OmegaXr/1', 'OmegaXOmegaXr/2', 'w_x_r');

%% ---- vibration (|v_x| scaled band-pass white noise) ----
SEL(n, 'Vx', '1', 3);  PICK(n, tr, 'Truth_vel_B', {'vel_B'}, {'Vx/1'});
A(n, 'simulink/Math Operations/Abs', 'AbsVx');  L(n, 'Vx/1', 'AbsVx/1', 'v_x');
for s = {'A', 'W'}
    q = s{1};
    A(n, 'simulink/Sources/Band-Limited White Noise', ['Noise_' q], 'Cov', 'P.sensors.imu.vib.cov*[1 1 1]', ...
      'Ts', ts, 'seed', ['P.sensors.imu.vib.seed' q]);
    A(n, 'simulink/Discrete/Discrete Transfer Fcn', ['BP_' q], 'Numerator', 'P.sensors.imu.vib.bpNum', ...
      'Denominator', 'P.sensors.imu.vib.bpDen', 'SampleTime', ts);
    A(n, 'simulink/Math Operations/Gain', ['Sig_' q], 'Gain', ['P.sensors.imu.vib.gain' q], ...
      'Multiplication', 'Element-wise(K.*u)');
    A(n, 'simulink/Math Operations/Product', ['Vib_' q], 'Inputs', '**');
    RESH(n, ['BP1d_' q], '1d');
    L(n, ['Noise_' q '/1'], ['BP_' q '/1'], ['white_' q]);
    L(n, ['BP_' q '/1'], ['BP1d_' q '/1'], ['bp_' q]);
    L(n, ['BP1d_' q '/1'], ['Sig_' q '/1']);
    L(n, ['Sig_' q '/1'], ['Vib_' q '/1'], ['vib1_' q]);
    L(n, 'AbsVx/1', ['Vib_' q '/2'], 'abs_vx');
end
A(n, 'simulink/Math Operations/Sum', 'ARel', 'Inputs', '+++');
L(n, 'AlphaXr/1', 'ARel/1', 'alpha_x_r');  L(n, 'OmegaXOmegaXr/1', 'ARel/2', 'centripetal');
L(n, 'Vib_A/1', 'ARel/3', 'a_vib');

%% ---- linear acceleration of the IMU in W (+ gravity correction, block uses 9.81) ----
RESH(n, 'ARel_col', 'col');  MATMUL(n, 'RWB_aRel');  RESH(n, 'ARel_W', '1d');
L(n, 'ARel/1', 'ARel_col/1', 'a_rel_B');
PICK(n, tr, 'Truth_R_WB', {'R_WB'}, {'RWB_aRel/1'});  L(n, 'ARel_col/1', 'RWB_aRel/2');
L(n, 'RWB_aRel/1', 'ARel_W/1');
C(n, 'g_corr', 'P.sensors.imu.gcorr_W');
A(n, 'simulink/Math Operations/Sum', 'AccIMU_W', 'Inputs', '+++');
PICK(n, tr, 'Truth_acc_W', {'acc_W'}, {'AccIMU_W/1'});  L(n, 'ARel_W/1', 'AccIMU_W/2', 'a_rel_W');
L(n, 'g_corr/1', 'AccIMU_W/3', 'g_corr');

%% ---- angular velocity in W ----
A(n, 'simulink/Math Operations/Sum', 'OmegaB_vib', 'Inputs', '++');
PICK(n, tr, 'Truth_omega_B_3', {'omega_B'}, {'OmegaB_vib/1'});  L(n, 'Vib_W/1', 'OmegaB_vib/2', 'w_vib');
RESH(n, 'Omega_col', 'col');  MATMUL(n, 'RWB_omega');  RESH(n, 'Omega_W', '1d');
L(n, 'OmegaB_vib/1', 'Omega_col/1', 'omega_B_vib');
PICK(n, tr, 'Truth_R_WB_2', {'R_WB'}, {'RWB_omega/1'});  L(n, 'Omega_col/1', 'RWB_omega/2');
L(n, 'RWB_omega/1', 'Omega_W/1');

%% ---- orientation q_WI = quat(R_WB R_BI) (true mounting misalignment) ----
C(n, 'R_BI', 'P.sensors.imu.R_BI');
MATMUL(n, 'R_WI');
PICK(n, tr, 'Truth_R_WB_3', {'R_WB'}, {'R_WI/1'});  L(n, 'R_BI/1', 'R_WI/2', 'R_BI');
A(n, 'navlib/Utilities/Coordinate Transformation Conversion', 'Rot2Quat', 'InputRepresentation', 'Rotation Matrix', ...
  'OutputRepresentation', 'Quaternion', 'SimulateUsing', 'Code generation');
L(n, 'R_WI/1', 'Rot2Quat/1', 'R_WI');

%% ---- temperature (degC) at 200 Hz ----
BS(n, 'Env', {'T_imu_degC'});  L(n, 'elec_env/1', 'Env/1', 'elec_env');
RT(n, 'RT_T', ts);  L(n, 'Env/1', 'RT_T/1');

%% ---- IMU sensor block ----
imuNV = {'DispRefFrame', 'ENU', 'TemperaturePort', 'on', 'MagneticFieldPort', 'off', ...
    'MagneticFieldENU', 'P.sensors.imu.magENU', 'SeedDouble', 'P.sensors.imu.seed', 'SimulateUsing', 'Code generation'};
grp = {'Accel', 'accel', {'MeasurementRange', 'Resolution', 'ConstantBias', 'AxesMisalignment', 'NoiseDensity', ...
          'BiasInstability', 'RandomWalk', 'TemperatureBias', 'TemperatureScaleFactor'}; ...
       'Gyro', 'gyro', {'MeasurementRange', 'Resolution', 'ConstantBias', 'AxesMisalignment', 'AccelerationBias', ...
          'NoiseDensity', 'BiasInstability', 'RandomWalk', 'TemperatureBias', 'TemperatureScaleFactor'}; ...
       'Mag', 'mag', {'MeasurementRange', 'Resolution', 'ConstantBias', 'NoiseDensity', 'BiasInstability', ...
          'TemperatureBias'}};
for g = 1:size(grp, 1)
    for k = 1:numel(grp{g, 3})
        expr = ['P.sensors.imu.' grp{g, 2} '.' grp{g, 3}{k}];
        % the block output is the NEGATIVE specific force (Gain -1 follows): the accelerometer bias terms are
        % negated so that P.sensors.imu.accel.ConstantBias / TemperatureBias appear with their sign on the
        % physical bus (verified: level at rest reads g + ConstantBias(3) on Bus_ImuRaw)
        if g == 1 && any(strcmp(grp{g, 3}{k}, {'ConstantBias', 'TemperatureBias'})), expr = ['-' expr]; end
        imuNV(end+1:end+2) = {[grp{g, 1} 'Params' grp{g, 3}{k}], expr};
    end
end
A(n, 'mspsensorlib/IMU', 'IMU', imuNV{:});
RESH(n, 'AccRow', 'row');  RESH(n, 'OmegaRow', 'row');      % System block inputs are N x 3 (one row)
L(n, 'AccIMU_W/1', 'AccRow/1', 'acc_imu_W');  L(n, 'Omega_W/1', 'OmegaRow/1', 'omega_W');
L(n, 'AccRow/1', 'IMU/1');  L(n, 'OmegaRow/1', 'IMU/2');
RESH(n, 'QuatRow', 'row');  L(n, 'Rot2Quat/1', 'QuatRow/1', 'q_WI');  L(n, 'QuatRow/1', 'IMU/3');       L(n, 'RT_T/1', 'IMU/4', 'T_imu_degC');

%% ---- physical sign, DLPF ----
RESH(n, 'Acc1d', '1d');  RESH(n, 'Gyr1d', '1d');  RESH(n, 'Mag1d', '1d');
L(n, 'IMU/1', 'Acc1d/1', 'accel_matlab');  L(n, 'IMU/2', 'Gyr1d/1', 'gyro');  L(n, 'IMU/3', 'Mag1d/1', 'mag');
A(n, 'simulink/Math Operations/Gain', 'PhysSign', 'Gain', '-1');
L(n, 'Acc1d/1', 'PhysSign/1');
A(n, 'simulink/Discrete/Discrete Transfer Fcn', 'DLPF_A', 'Numerator', 'P.sensors.imu.dlpf.num', ...
  'Denominator', 'P.sensors.imu.dlpf.den', 'SampleTime', ts);
A(n, 'simulink/Discrete/Discrete Transfer Fcn', 'DLPF_W', 'Numerator', 'P.sensors.imu.dlpf.num', ...
  'Denominator', 'P.sensors.imu.dlpf.den', 'SampleTime', ts);
L(n, 'PhysSign/1', 'DLPF_A/1', 'accel_phys');  L(n, 'Gyr1d/1', 'DLPF_W/1');
RESH(n, 'DLPF_A1d', '1d');  RESH(n, 'DLPF_W1d', '1d');
L(n, 'DLPF_A/1', 'DLPF_A1d/1', 'accel_lp');  L(n, 'DLPF_W/1', 'DLPF_W1d/1', 'gyro_lp');

%% ---- magnetometer disturbance k_I (|I_L| + |I_R|) ----
RT(n, 'RT_drive', ts);  L(n, 'drive_truth/1', 'RT_drive/1', 'drive_truth');
A(n, 'simulink/Math Operations/Abs', 'AbsIL');  A(n, 'simulink/Math Operations/Abs', 'AbsIR');
PICK(n, 'RT_drive/1', 'Drive_L_i_dc', {'L.i_dc'}, {'AbsIL/1'}, 'drive_truth_5ms');
PICK(n, 'RT_drive/1', 'Drive_R_i_dc', {'R.i_dc'}, {'AbsIR/1'});
A(n, 'simulink/Math Operations/Sum', 'ISum', 'Inputs', '++');
L(n, 'AbsIL/1', 'ISum/1');  L(n, 'AbsIR/1', 'ISum/2');
A(n, 'simulink/Math Operations/Gain', 'MagDist', 'Gain', 'P.sensors.imu.kMagI', 'Multiplication', 'Element-wise(K.*u)');
L(n, 'ISum/1', 'MagDist/1', 'i_traction');
A(n, 'simulink/Math Operations/Sum', 'MagTot', 'Inputs', '++');
L(n, 'Mag1d/1', 'MagTot/1');  L(n, 'MagDist/1', 'MagTot/2', 'm_dist');

%% ---- faults: 1 stuck (hold), 2 bias step, 3 dropout (frozen) ----
RT(n, 'RT_flt', ts);  L(n, 'flt/1', 'RT_flt/1', 'flt');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'IsBias', 'relop', '==', 'const', '2');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'IsStuck', 'relop', '==', 'const', '1');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'IsDrop', 'relop', '==', 'const', '3');
PICK(n, 'RT_flt/1', 'Flt_imu_fault', {'imu_fault'}, {'IsBias/1'}, 'flt_5ms');
PICK(n, 'RT_flt/1', 'Flt_imu_fault_2', {'imu_fault'}, {'IsStuck/1'});
PICK(n, 'RT_flt/1', 'Flt_imu_fault_3', {'imu_fault'}, {'IsDrop/1'});
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Hold', 'Operator', 'OR', 'Inputs', '2');
L(n, 'IsStuck/1', 'Hold/1');  L(n, 'IsDrop/1', 'Hold/2');
DTC(n, 'BiasOn', 'double');  L(n, 'IsBias/1', 'BiasOn/1');
C(n, 'dAcc', 'P.sensors.imu.fault.dAcc');  C(n, 'dGyro', 'P.sensors.imu.fault.dGyro');
A(n, 'simulink/Math Operations/Product', 'StepA', 'Inputs', '**');
A(n, 'simulink/Math Operations/Product', 'StepW', 'Inputs', '**');
L(n, 'dAcc/1', 'StepA/1');  L(n, 'BiasOn/1', 'StepA/2', 'bias_on');
L(n, 'dGyro/1', 'StepW/1');  L(n, 'BiasOn/1', 'StepW/2');
A(n, 'simulink/Math Operations/Sum', 'AccF', 'Inputs', '++');
A(n, 'simulink/Math Operations/Sum', 'GyrF', 'Inputs', '++');
L(n, 'DLPF_A1d/1', 'AccF/1');  L(n, 'StepA/1', 'AccF/2', 'dacc');
L(n, 'DLPF_W1d/1', 'GyrF/1');   L(n, 'StepW/1', 'GyrF/2', 'dgyro');
A(n, 'simulink/Signal Routing/Mux', 'Pack', 'Inputs', '[3 3 3 1]');
L(n, 'AccF/1', 'Pack/1');  L(n, 'GyrF/1', 'Pack/2');  L(n, 'MagTot/1', 'Pack/3', 'mag_tot');  L(n, 'RT_T/1', 'Pack/4');
A(n, 'simulink/Signal Routing/Switch', 'HoldSw', 'Criteria', 'u2 ~= 0');
A(n, 'simulink/Discrete/Unit Delay', 'Last', 'InitialCondition', 'zeros(10,1)', 'SampleTime', ts);
L(n, 'Last/1', 'HoldSw/1', 'last');  L(n, 'Hold/1', 'HoldSw/2', 'hold');  L(n, 'Pack/1', 'HoldSw/3', 'sample');
L(n, 'HoldSw/1', 'Last/1', 'imu_out');
DTC(n, 'ToSingle', 'single');  L(n, 'HoldSw/1', 'ToSingle/1');
A(n, 'simulink/Signal Routing/Demux', 'Unpack', 'Outputs', '[3 3 3 1]');
L(n, 'ToSingle/1', 'Unpack/1', 'imu_single');
BC(n, 'BC_ImuRaw', 'Bus_ImuRaw', {'Unpack/1', 'Unpack/2', 'Unpack/3', 'Unpack/4'});
L(n, 'BC_ImuRaw/1', 'imu_raw/1', 'imu_raw');
set_param(n, 'Description', sprintf([ ...
  'sens_imu_model (T11, sensors spec 3.1). Ports: in 1 truth Bus_TruthState 0.001, 2 drive_truth Bus_DriveTruth 0.001, ' ...
  '3 elec_env Bus_ElecEnv 0.1, 4 flt Bus_FaultInject 0.01; out 1 imu_raw Bus_ImuRaw 0.005 (single, PHYSICAL specific force).\n' ...
  'a_IMU,W = acc_W + R_WB (alpha x r_I + omega x (omega x r_I) + a_vib) + gcorr_W (block gravity 9.81 -> P.frames.g); ' ...
  'omega_W = R_WB (omega_B + w_vib); q_WI = quat(R_WB R_BI) (true misalignment); mspsensorlib/IMU (ENU, temperature port = ' ...
  'elec_env.T_imu_degC); Gain -1; DLPF 50 Hz; mag + k_I(|I_L|+|I_R|); imu_fault 1 stuck / 3 dropout = hold, 2 = +dAcc/+dGyro.']));
end

% ============================================================================================
% sens_gnss_model
% ============================================================================================
function local_sens_gnss_model(n)
tsD = 'Ts.ctrl';  ts = 'Ts.gnss';
IN(n, 'truth', 1, 'Bus: Bus_TruthState', 'Ts.base');
IN(n, 'elec_env', 2, 'Bus: Bus_ElecEnv', 'Ts.hk');
IN(n, 'flt', 3, 'Bus: Bus_FaultInject', 'Ts.ctrl');
OUT(n, 'gnss_raw', 1, 'Bus: Bus_GnssRaw');
OUT(n, 'svf', 2, 'double');  OUT(n, 'zoneId', 3, 'uint8');  OUT(n, 'fixTypeTruth', 4, 'uint8');
OUT(n, 'relposFixed', 5, 'boolean');  OUT(n, 'gnssPosErr', 6, 'double');  OUT(n, 'outlier', 7, 'boolean');

%% ---- antenna truth at 100 Hz (the bus truth_10ms; each consumer picks its element: Bus Selectors Truth_<element>) ----
RT(n, 'RT_truth', tsD);  L(n, 'truth/1', 'RT_truth/1', 'truth');
tr = 'RT_truth/1';
C(n, 'r_a', 'P.sensors.gnss.rAnt1');
C(n, 'r_a_col', 'P.sensors.gnss.rAnt1(:)', 'VectorParams1D', 'off');
MATMUL(n, 'RWB_ra');  RESH(n, 'La_W', '1d');
PICK(n, tr, 'Truth_R_WB', {'R_WB'}, {'RWB_ra/1'}, 'truth_10ms');
L(n, 'r_a_col/1', 'RWB_ra/2', 'r_a');  L(n, 'RWB_ra/1', 'La_W/1');
A(n, 'simulink/Math Operations/Sum', 'PosAnt', 'Inputs', '++');
PICK(n, tr, 'Truth_pos_W', {'pos_W'}, {'PosAnt/1'});  L(n, 'La_W/1', 'PosAnt/2', 'lever_W');
XPROD(n, 'OmegaXra');
PICK(n, tr, 'Truth_omega_B', {'omega_B'}, {'OmegaXra/1'});  L(n, 'r_a/1', 'OmegaXra/2');
RESH(n, 'Wxr_col', 'col');  MATMUL(n, 'RWB_wxr');  RESH(n, 'Wxr_W', '1d');
L(n, 'OmegaXra/1', 'Wxr_col/1', 'w_x_ra');  PICK(n, tr, 'Truth_R_WB_2', {'R_WB'}, {'RWB_wxr/1'});
L(n, 'Wxr_col/1', 'RWB_wxr/2');
L(n, 'RWB_wxr/1', 'Wxr_W/1');
A(n, 'simulink/Math Operations/Sum', 'VelAnt', 'Inputs', '++');
PICK(n, tr, 'Truth_vel_W', {'vel_W'}, {'VelAnt/1'});  L(n, 'Wxr_W/1', 'VelAnt/2', 'w_x_ra_W');
SEL(n, 'Psi', '1', 3);  PICK(n, tr, 'Truth_eul', {'eul'}, {'Psi/1'});

%% ---- latency: Delay 8 x 0.01 s, then 10 Hz epochs ----
A(n, 'simulink/Signal Routing/Mux', 'TruthAnt', 'Inputs', '[1 3 3 1]');
PICK(n, tr, 'Truth_t', {'t'}, {'TruthAnt/1'});  L(n, 'PosAnt/1', 'TruthAnt/2', 'p_ant');
L(n, 'VelAnt/1', 'TruthAnt/3', 'v_ant');  L(n, 'Psi/1', 'TruthAnt/4', 'psi');
A(n, 'simulink/Discrete/Delay', 'Latency', 'DelayLength', 'P.sensors.gnss.latencySteps', ...
  'InitialCondition', '0', 'SampleTime', tsD);
L(n, 'TruthAnt/1', 'Latency/1', 'ant_truth');
RT(n, 'RT_epoch', ts);  L(n, 'Latency/1', 'RT_epoch/1', 'ant_truth_delayed');
A(n, 'simulink/Signal Routing/Demux', 'Epoch', 'Outputs', '[1 3 3 1]');
L(n, 'RT_epoch/1', 'Epoch/1', 'ant_epoch');

%% ---- sky view at the antenna (D59: 2-D Lookup Tables on the precomputed grids) ----
SEL(n, 'AntX', '1', 3);  SEL(n, 'AntY', '2', 3);
L(n, 'Epoch/2', 'AntX/1', 'p_ant_d');  L(n, 'Epoch/2', 'AntY/1');
lut = {'BreakpointsForDimension1', 'P.world.terrain.xE', 'BreakpointsForDimension2', 'P.world.terrain.yN', ...
       'ExtrapMethod', 'Clip', 'IndexSearchMethod', 'Evenly spaced points', 'DiagnosticForOutOfRangeInput', 'None'};
A(n, 'simulink/Lookup Tables/2-D Lookup Table', 'SVF_LUT', 'Table', 'P.world.gnssSvf', ...
  'InterpMethod', 'Linear point-slope', lut{:});
A(n, 'simulink/Lookup Tables/2-D Lookup Table', 'Zone_LUT', 'Table', 'P.world.gnssZoneId', ...
  'InterpMethod', 'Nearest', lut{:});
L(n, 'AntX/1', 'SVF_LUT/1', 'x_ant');  L(n, 'AntY/1', 'SVF_LUT/2', 'y_ant');
L(n, 'AntX/1', 'Zone_LUT/1');  L(n, 'AntY/1', 'Zone_LUT/2');
A(n, 'simulink/Discontinuities/Saturation', 'SVF_Clamp', 'UpperLimit', 'P.sensors.gnss.svfClamp(2)', ...
  'LowerLimit', 'P.sensors.gnss.svfClamp(1)');
L(n, 'SVF_LUT/1', 'SVF_Clamp/1', 'svf_raw');
DTC(n, 'ZoneU8', 'uint8');  L(n, 'Zone_LUT/1', 'ZoneU8/1', 'zone_dbl');

%% ---- corrections, fault flags (Corr and Flt are FixChain's picks; the other consumers pick their own) ----
BS(n, 'Corr', {'gnssCorrOK', 'gnssCorrAge'});  L(n, 'elec_env/1', 'Corr/1', 'elec_env');
RT(n, 'RT_flt', ts);  L(n, 'flt/1', 'RT_flt/1', 'flt');
BS(n, 'Flt', {'gnss_outage', 'gnss_force_float'});  L(n, 'RT_flt/1', 'Flt/1', 'flt_100ms');

%% ---- fix chain (MATLAB Function, D1) ----
A(n, 'simulink/Sources/Uniform Random Number', 'U', 'Minimum', 'zeros(1,6)', 'Maximum', 'ones(1,6)', ...
  'Seed', 'P.sensors.gnss.seedU', 'SampleTime', ts);
SEL(n, 'U_chain', '1:4', 6);  SEL(n, 'U_mag', '5', 6);  SEL(n, 'U_dir', '6', 6);
L(n, 'U/1', 'U_chain/1', 'u');  L(n, 'U/1', 'U_mag/1');  L(n, 'U/1', 'U_dir/1');
MF(n, 'FixChain', sprintf(['function [fixType, relposFixed, outlier] = fcn(svf, u, corrOK, corrAge, outage, forceFloat, c)\n' ...
    '%%#codegen\n' ...
    '[fixType, relposFixed, outlier] = sens_gnss_fixchain(svf, u, corrOK, corrAge, outage, forceFloat, c);\n']), ...
    ['RTK fix-state Markov chain (sensors spec 3.2, F2): subsystems/sensors/sens_gnss_fixchain.m; input c = ' ...
     'sens_gnss_fixchain_prm(P.sensors.gnss)']);
C(n, 'ChainPrm', 'sens_gnss_fixchain_prm(P.sensors.gnss)');
L(n, 'SVF_Clamp/1', 'FixChain/1', 'svf');  L(n, 'U_chain/1', 'FixChain/2', 'u_chain');
L(n, 'Corr/1', 'FixChain/3');  L(n, 'Corr/2', 'FixChain/4');  L(n, 'Flt/1', 'FixChain/5');  L(n, 'Flt/2', 'FixChain/6');
L(n, 'ChainPrm/1', 'FixChain/7', 'chain_prm');

%% ---- outlier jump U(2,5) m in a random direction (one epoch) ----
A(n, 'simulink/Math Operations/Gain', 'MagSpan', 'Gain', 'P.sensors.gnss.outlierMag(2) - P.sensors.gnss.outlierMag(1)');
A(n, 'simulink/Math Operations/Bias', 'MagMin', 'Bias', 'P.sensors.gnss.outlierMag(1)');
A(n, 'simulink/Math Operations/Gain', 'Dir2Pi', 'Gain', '2*pi');
L(n, 'U_mag/1', 'MagSpan/1', 'u_mag');  L(n, 'MagSpan/1', 'MagMin/1');  L(n, 'U_dir/1', 'Dir2Pi/1', 'u_dir');
A(n, 'simulink/Math Operations/Trigonometric Function', 'CosDir', 'Operator', 'cos');
A(n, 'simulink/Math Operations/Trigonometric Function', 'SinDir', 'Operator', 'sin');
L(n, 'Dir2Pi/1', 'CosDir/1', 'dir');  L(n, 'Dir2Pi/1', 'SinDir/1');
DTC(n, 'OutlierD', 'double');  L(n, 'FixChain/3', 'OutlierD/1', 'outlier');
A(n, 'simulink/Math Operations/Product', 'JumpMag', 'Inputs', '**');
L(n, 'MagMin/1', 'JumpMag/1', 'jump');  L(n, 'OutlierD/1', 'JumpMag/2', 'outlier_d');
A(n, 'simulink/Math Operations/Product', 'JumpE', 'Inputs', '**');
A(n, 'simulink/Math Operations/Product', 'JumpN', 'Inputs', '**');
L(n, 'JumpMag/1', 'JumpE/1', 'jump_on');  L(n, 'CosDir/1', 'JumpE/2');
L(n, 'JumpMag/1', 'JumpN/1');  L(n, 'SinDir/1', 'JumpN/2');
C(n, 'Zero', '0', 'SampleTime', ts);
A(n, 'simulink/Signal Routing/Mux', 'Jump', 'Inputs', '3');
L(n, 'JumpE/1', 'Jump/1');  L(n, 'JumpN/1', 'Jump/2');  L(n, 'Zero/1', 'Jump/3');
A(n, 'simulink/Math Operations/Sum', 'PosIn', 'Inputs', '++');
L(n, 'Epoch/2', 'PosIn/1');  L(n, 'Jump/1', 'PosIn/2', 'jump_ENU');

%% ---- three GPS blocks (FIXED / FLOAT / SINGLE noise sets) + Multiport Switch by fix ----
% The GPS blocks model the position only (VelocityAccuracy 0: the velocity passes through unchanged); the velocity
% noise is added after the switch (VelNoise below), see there.
A(n, 'simulink/Signal Routing/Multiport Switch', 'FixSel', 'Inputs', '3', 'DataPortOrder', 'Specify indices', ...
  'DataPortIndices', '{4,5,1}', 'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
L(n, 'FixChain/1', 'FixSel/1', 'fixType');
nm = {'FIXED', 'FLOAT', 'SINGLE'};
RESH(n, 'PosRow', 'row');  RESH(n, 'VelRow', 'row');        % System block inputs are N x 3 (one row)
L(n, 'PosIn/1', 'PosRow/1', 'p_in');  L(n, 'Epoch/3', 'VelRow/1', 'v_ant_d');
for k = 1:3
    g = ['GPS_' nm{k}];
    A(n, 'mspsensorlib/GPS', g, 'DispRefFrame', 'ENU', 'PositionInputFormat', 'Local', ...
      'ReferenceLocation', 'P.sensors.refLLA', ...
      'HorizontalPositionAccuracy', sprintf('P.sensors.gnss.sigH(%d)', k), ...
      'VerticalPositionAccuracy', sprintf('P.sensors.gnss.sigV(%d)', k), ...
      'VelocityAccuracy', '0', ...
      'DecayFactor', sprintf('P.sensors.gnss.decay(%d)', k), ...
      'SeedDouble', sprintf('P.sensors.gnss.seeds(%d)', k), 'SimulateUsing', 'Code generation');
    L(n, 'PosRow/1', [g '/1']);  L(n, 'VelRow/1', [g '/2']);
    RESH(n, ['LLA1d_' nm{k}], '1d');  RESH(n, ['Vel1d_' nm{k}], '1d');
    L(n, [g '/1'], ['LLA1d_' nm{k} '/1'], ['lla_' nm{k}]);  L(n, [g '/2'], ['Vel1d_' nm{k} '/1'], ['vel_' nm{k}]);
    TERM(n, [g '/3'], ['TermGS_' nm{k}], ['groundspeed_' nm{k}]);
    TERM(n, [g '/4'], ['TermCrs_' nm{k}], ['course_' nm{k}]);
    A(n, 'simulink/Signal Routing/Mux', ['Out_' nm{k}], 'Inputs', '[3 3]');
    L(n, ['LLA1d_' nm{k} '/1'], ['Out_' nm{k} '/1']);  L(n, ['Vel1d_' nm{k} '/1'], ['Out_' nm{k} '/2']);
    L(n, ['Out_' nm{k} '/1'], sprintf('FixSel/%d', k + 1), ['gps_' nm{k}]);
end
A(n, 'simulink/Signal Routing/Demux', 'GpsOut', 'Outputs', '[3 3]');
L(n, 'FixSel/1', 'GpsOut/1', 'gps_sel');

%% ---- velocity: v_a(t - latency) + zero-mean white noise per axis, sigma sigVel(fix) ----
% mspsensorlib/GPS (fusion.internal.GPSSensorBase.stepImpl, R2026b) adds the MAGNITUDE sqrt(e1^2 + e2^2) of its 2-D
% velocity noise to the ground speed, and its course noise has sigma VelocityAccuracy/speed in rad added to a
% course in deg: its velocity reads sqrt(pi/2) sigVel = 1.2533 sigVel high along the track (+0.025 m/s FIXED,
% +0.063 m/s SINGLE) with almost no noise across it, a bias a receiver's Doppler velocity does not have (S4 nav
% fix round 2, 2026-11-14; the nav bench had worked around it since T12). The blocks run with VelocityAccuracy 0
% and the noise is added here; NOFIX takes the SINGLE sigma, as FixSel takes the SINGLE solution.
A(n, 'simulink/Sources/Random Number', 'VelNoise', 'Mean', '[0 0 0]', 'Variance', '[1 1 1]', ...
  'Seed', 'P.sensors.gnss.seedVel', 'SampleTime', ts);
A(n, 'simulink/Signal Routing/Multiport Switch', 'VelSigSel', 'Inputs', '3', 'DataPortOrder', 'Specify indices', ...
  'DataPortIndices', '{4,5,1}', 'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
L(n, 'FixChain/1', 'VelSigSel/1');
for k = 1:3
    C(n, ['SigVel_' nm{k}], sprintf('P.sensors.gnss.sigVel(%d)', k), 'SampleTime', ts);
    L(n, ['SigVel_' nm{k} '/1'], sprintf('VelSigSel/%d', k + 1), ['sigVel_' nm{k}]);
end
A(n, 'simulink/Math Operations/Product', 'VelNoiseS', 'Inputs', '**');
L(n, 'VelNoise/1', 'VelNoiseS/1', 'n_vel');  L(n, 'VelSigSel/1', 'VelNoiseS/2', 'sig_vel');
A(n, 'simulink/Math Operations/Sum', 'VelMeas', 'Inputs', '++');
L(n, 'GpsOut/2', 'VelMeas/1', 'vel_gps');  L(n, 'VelNoiseS/1', 'VelMeas/2', 'vel_noise');

%% ---- reported accuracies [hAcc vAcc sAcc] = acc(fix) * clamp(1 + 0.15 n) ----
A(n, 'simulink/Signal Routing/Multiport Switch', 'AccSel', 'Inputs', '4', 'DataPortOrder', 'Specify indices', ...
  'DataPortIndices', '{4,5,1,0}', 'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
L(n, 'FixChain/1', 'AccSel/1');
acc = {'accFixed', 'accFloat', 'accSingle', 'accNoFix'};
for k = 1:4
    C(n, ['Acc_' acc{k}], ['P.sensors.gnss.' acc{k}], 'SampleTime', ts);
    L(n, ['Acc_' acc{k} '/1'], sprintf('AccSel/%d', k + 1));
end
A(n, 'simulink/Sources/Random Number', 'RepNoise', 'Mean', '0', 'Variance', '1', 'Seed', 'P.sensors.gnss.seedJit', ...
  'SampleTime', ts);
A(n, 'simulink/Math Operations/Gain', 'RepGain', 'Gain', 'P.sensors.gnss.repJitter');
A(n, 'simulink/Math Operations/Bias', 'RepBias', 'Bias', '1');
A(n, 'simulink/Discontinuities/Saturation', 'RepClamp', 'UpperLimit', 'P.sensors.gnss.repClamp(2)', ...
  'LowerLimit', 'P.sensors.gnss.repClamp(1)');
L(n, 'RepNoise/1', 'RepGain/1', 'n_rep');  L(n, 'RepGain/1', 'RepBias/1');  L(n, 'RepBias/1', 'RepClamp/1');
A(n, 'simulink/Math Operations/Product', 'AccRep', 'Inputs', '**');
L(n, 'AccSel/1', 'AccRep/1', 'acc_fix');  L(n, 'RepClamp/1', 'AccRep/2', 'k_rep');
DTC(n, 'AccSingle', 'single');  L(n, 'AccRep/1', 'AccSingle/1', 'acc_rep');
A(n, 'simulink/Signal Routing/Demux', 'AccOut', 'Outputs', '3');
L(n, 'AccSingle/1', 'AccOut/1');

%% ---- numSV = round(6 + 22 SVF), 0 in an outage ----
A(n, 'simulink/Math Operations/Gain', 'SV_gain', 'Gain', 'P.sensors.gnss.numSV(2)');
A(n, 'simulink/Math Operations/Bias', 'SV_bias', 'Bias', 'P.sensors.gnss.numSV(1)');
L(n, 'SVF_Clamp/1', 'SV_gain/1');  L(n, 'SV_gain/1', 'SV_bias/1');
C(n, 'SV_zero', '0', 'SampleTime', ts);
A(n, 'simulink/Signal Routing/Switch', 'SV_out', 'Criteria', 'u2 ~= 0');
L(n, 'SV_zero/1', 'SV_out/1');  PICK(n, 'RT_flt/1', 'Flt_gnss_outage', {'gnss_outage'}, {'SV_out/2'});
L(n, 'SV_bias/1', 'SV_out/3', 'numSV_dbl');
DTC(n, 'SV_u8', 'uint8');  L(n, 'SV_out/1', 'SV_u8/1');

%% ---- dual-antenna heading: heading_deg = mod(90 - deg(psi + n) - psiBaseline, 360) ----
A(n, 'simulink/Sources/Random Number', 'HeadNoise', 'Mean', '0', 'Variance', 'P.sensors.gnss.head_sigma^2', ...
  'Seed', 'P.sensors.gnss.seedHead', 'SampleTime', ts);
A(n, 'simulink/Math Operations/Sum', 'PsiMeas', 'Inputs', '++');
L(n, 'Epoch/4', 'PsiMeas/1', 'psi_d');  L(n, 'HeadNoise/1', 'PsiMeas/2', 'n_head');
A(n, 'simulink/Math Operations/Gain', 'PsiDeg', 'Gain', '180/pi');
L(n, 'PsiMeas/1', 'PsiDeg/1', 'psi_meas');
C(n, 'Head90', '90 - P.sensors.gnss.psiBaseline*180/pi', 'SampleTime', ts);
A(n, 'simulink/Math Operations/Sum', 'HeadCW', 'Inputs', '+-');
L(n, 'Head90/1', 'HeadCW/1');  L(n, 'PsiDeg/1', 'HeadCW/2', 'psi_deg');
C(n, 'C360', '360', 'SampleTime', ts);
A(n, 'simulink/Math Operations/Math Function', 'HeadMod', 'Operator', 'mod');
L(n, 'HeadCW/1', 'HeadMod/1', 'heading_unwrapped');  L(n, 'C360/1', 'HeadMod/2');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'NotOutage', 'Operator', 'NOT', 'Inputs', '1');
PICK(n, 'RT_flt/1', 'Flt_gnss_outage_2', {'gnss_outage'}, {'NotOutage/1'});
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'HeadValid', 'Operator', 'AND', 'Inputs', '2');
L(n, 'FixChain/2', 'HeadValid/1', 'relposFixed');  L(n, 'NotOutage/1', 'HeadValid/2', 'newData');
C(n, 'HeadAccOK', 'P.sensors.gnss.head_sigma_deg', 'SampleTime', ts);
C(n, 'HeadAccBad', 'P.sensors.gnss.headAccInvalid_deg', 'SampleTime', ts);
A(n, 'simulink/Signal Routing/Switch', 'HeadAcc', 'Criteria', 'u2 ~= 0');
L(n, 'HeadAccOK/1', 'HeadAcc/1');  L(n, 'HeadValid/1', 'HeadAcc/2', 'headingValid');  L(n, 'HeadAccBad/1', 'HeadAcc/3');
DTC(n, 'HeadAccS', 'single');  L(n, 'HeadAcc/1', 'HeadAccS/1');
DTC(n, 'CorrAgeS', 'single');  PICK(n, 'elec_env/1', 'Corr_gnssCorrAge', {'gnssCorrAge'}, {'CorrAgeS/1'});

%% ---- Bus_GnssRaw ----
BC(n, 'BC_Gnss', 'Bus_GnssRaw', {'Epoch/1', 'GpsOut/1', 'VelMeas/1', 'FixChain/1', 'SV_u8/1', 'AccOut/1', ...
    'AccOut/2', 'AccOut/3', 'HeadValid/1', 'HeadMod/1', 'HeadAccS/1', 'CorrAgeS/1', 'NotOutage/1'});
L(n, 'BC_Gnss/1', 'gnss_raw/1', 'gnss_raw');

%% ---- truth diagnostics: reported (LLA -> local ENU) minus true antenna position ----
SEL(n, 'LatLon', '[1 2]', 3);  SEL(n, 'Alt', '3', 3);
L(n, 'GpsOut/1', 'LatLon/1', 'lla');  L(n, 'GpsOut/1', 'Alt/1');
C(n, 'Href', '-P.sensors.refLLA(3)', 'SampleTime', ts);     % flat-Earth z = -h - href (verified)
A(n, 'aerolibtransform2/LLA to Flat Earth', 'LLA2Flat', 'units', 'Metric (MKS)', 'ptype', 'Earth (WGS84)', ...
  'LL0', 'P.sensors.refLLA(1:2)', 'psi', '0', 'refPosPort', 'off');
L(n, 'LatLon/1', 'LLA2Flat/1', 'latlon');  L(n, 'Alt/1', 'LLA2Flat/2', 'h');  L(n, 'Href/1', 'LLA2Flat/3', 'href');
RESH(n, 'NED1d', '1d');  L(n, 'LLA2Flat/1', 'NED1d/1', 'p_NED');
SEL(n, 'NED2ENU_sel', '[2 1 3]', 3);  L(n, 'NED1d/1', 'NED2ENU_sel/1');
A(n, 'simulink/Math Operations/Gain', 'NED2ENU', 'Gain', '[1 1 -1]', 'Multiplication', 'Element-wise(K.*u)');
L(n, 'NED2ENU_sel/1', 'NED2ENU/1');
A(n, 'simulink/Math Operations/Sum', 'PosErr', 'Inputs', '+-');
L(n, 'NED2ENU/1', 'PosErr/1', 'p_rep_ENU');  L(n, 'Epoch/2', 'PosErr/2');
L(n, 'SVF_Clamp/1', 'svf/1');  L(n, 'ZoneU8/1', 'zoneId/1', 'zoneId');
L(n, 'FixChain/1', 'fixTypeTruth/1');  L(n, 'FixChain/2', 'relposFixed/1');
L(n, 'PosErr/1', 'gnssPosErr/1', 'gnssPosErr');  L(n, 'FixChain/3', 'outlier/1');
set_param(n, 'Description', sprintf([ ...
  'sens_gnss_model (T11, sensors spec 3.2). Ports: in 1 truth Bus_TruthState 0.001, 2 elec_env Bus_ElecEnv 0.1, ' ...
  '3 flt Bus_FaultInject 0.01; out 1 gnss_raw Bus_GnssRaw 0.1, 2 svf double, 3 zoneId uint8, 4 fixTypeTruth uint8, ' ...
  '5 relposFixed boolean, 6 gnssPosErr double[3] (reported - true antenna ENU), 7 outlier boolean (all 0.1).\n' ...
  'p_a = pos_W + R_WB rAnt1, v_a = vel_W + R_WB (omega x rAnt1); Delay 8 x 0.01 s (latency) -> 0.1 s epochs; sky view = ' ...
  '2-D Lookup Tables P.world.gnssSvf (linear) / gnssZoneId (nearest) at the antenna xy (D59); fix chain MATLAB Function ' ...
  'sens_gnss_fixchain (corrOK/corrAge from elec_env, gnss_outage / gnss_force_float from flt); three mspsensorlib/GPS ' ...
  '(FIXED/FLOAT/SINGLE, position only: VelocityAccuracy 0) + Multiport Switch {4,5,1}; velocity v_a + N(0, ' ...
  'sigVel(fix)^2) per axis (the GPS block''s own velocity noise reads 1.2533 sigVel high along the track); ' ...
  'outliers U(2,5) m; heading CW from N + N(0, 0.5 deg).']));
end

% ============================================================================================
% sens_lidar_model
% ============================================================================================
function local_sens_lidar_model(n)
ts = 'Ts.lidar';
IN(n, 'truth', 1, 'Bus: Bus_TruthState', 'Ts.base');
IN(n, 'flt', 2, 'Bus: Bus_FaultInject', 'Ts.ctrl');
IN(n, 'obs', 3, 'Bus: Bus_ObstaclesDyn', 'Ts.ctrl');
OUT(n, 'lidar_scan', 1, 'Bus: Bus_LidarScanRaw');  OUT(n, 'lidar_cloud', 2, 'Bus: Bus_LidarCloudRaw');
OUT(n, 'dustLevel', 3, 'double');  OUT(n, 'lidarPoseTruth', 4, 'double');

% the bus truth_100ms: Truth picks Fidelity's elements, the lever arm and heading pick their own (Truth_<element>)
RT(n, 'RT_truth', ts);  L(n, 'truth/1', 'RT_truth/1', 'truth');
tr = 'RT_truth/1';
C(n, 'r_L', 'P.sensors.lidar.r(:)', 'VectorParams1D', 'off');
MATMUL(n, 'RWB_rL');  RESH(n, 'LL_W', '1d');
PICK(n, tr, 'Truth_R_WB', {'R_WB'}, {'RWB_rL/1'}, 'truth_100ms');
L(n, 'r_L/1', 'RWB_rL/2', 'r_L');  L(n, 'RWB_rL/1', 'LL_W/1');
A(n, 'simulink/Math Operations/Sum', 'PosL', 'Inputs', '++');
PICK(n, tr, 'Truth_pos_W', {'pos_W'}, {'PosL/1'});  L(n, 'LL_W/1', 'PosL/2', 'lever_W');
SEL(n, 'PosLxy', '[1 2]', 3);  L(n, 'PosL/1', 'PosLxy/1', 'p_L');
SEL(n, 'Psi', '1', 3);  PICK(n, tr, 'Truth_eul', {'eul'}, {'Psi/1'});
A(n, 'simulink/Math Operations/Bias', 'PsiL', 'Bias', 'P.sensors.lidar.eul_deg(1)*pi/180');
L(n, 'Psi/1', 'PsiL/1', 'psi');
A(n, 'simulink/Signal Routing/Mux', 'PoseL', 'Inputs', '[2 1]');
L(n, 'PosLxy/1', 'PoseL/1');  L(n, 'PsiL/1', 'PoseL/2', 'psi_L');

RT(n, 'RT_flt', ts);  L(n, 'flt/1', 'RT_flt/1', 'flt');
BS(n, 'Flt', {'dust_level', 'lidar_blackout'});  L(n, 'RT_flt/1', 'Flt/1', 'flt_100ms');
RT(n, 'RT_obs', ts);  L(n, 'obs/1', 'RT_obs/1', 'obs');
BS(n, 'Obs', {'n', 'xy', 'r'});  L(n, 'RT_obs/1', 'Obs/1', 'obs_100ms');

%% ---- Variant Subsystem VC_LIDAR ----
vs = [n '/Fidelity'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
c = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
c = setdiff(c, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
ins = {'t', 'poseL', 'dust', 'blackout', 'obsN', 'obsXY', 'obsR', 'pos_W', 'q_WB', 'vel_W', 'acc_W', 'omega_B'};
outs = {'scan', 'cloud'};
for k = 1:numel(ins), add_block('simulink/Sources/In1', [vs '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [vs '/' outs{k}], 'Port', num2str(k)); end
add_block('simulink/Ports & Subsystems/Subsystem', [vs '/FAST2D'], 'VariantControl', 'VC_LIDAR == 1');
add_block('simulink/Ports & Subsystems/Subsystem', [vs '/FULL3D'], 'VariantControl', 'VC_LIDAR == 2');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
local_lidar_fast2d([vs '/FAST2D'], ins, ts);
local_lidar_full3d([vs '/FULL3D'], ins, ts);
BS(n, 'Truth', {'t', 'pos_W', 'q_WB', 'vel_W', 'acc_W', 'omega_B'});  L(n, tr, 'Truth/1');
src = {'Truth/1', 'PoseL/1', 'Flt/1', 'Flt/2', 'Obs/1', 'Obs/2', 'Obs/3', 'Truth/2', 'Truth/3', 'Truth/4', ...
       'Truth/5', 'Truth/6'};
for k = 1:numel(src), L(n, src{k}, sprintf('Fidelity/%d', k)); end
L(n, 'Fidelity/1', 'lidar_scan/1', 'lidar_scan');  L(n, 'Fidelity/2', 'lidar_cloud/1', 'lidar_cloud');
PICK(n, 'RT_flt/1', 'Flt_dust_level', {'dust_level'}, {'dustLevel/1'});  L(n, 'PoseL/1', 'lidarPoseTruth/1', 'poseL');
set_param(n, 'Description', sprintf([ ...
  'sens_lidar_model (T11, sensors spec 3.4). Ports: in 1 truth Bus_TruthState 0.001, 2 flt Bus_FaultInject 0.01, ' ...
  '3 obs Bus_ObstaclesDyn 0.01; out 1 lidar_scan Bus_LidarScanRaw 0.1, 2 lidar_cloud Bus_LidarCloudRaw 0.1, ' ...
  '3 dustLevel double 0.1, 4 lidarPoseTruth double[3] [x_E y_N psi] 0.1.\n' ...
  'Variant Subsystem VC_LIDAR (update diagram): 1 FAST2D = MATLAB Function sens_lidar2d_fast (raster of P.world.obstacles ' ...
  'at first call, film layer, dynamic obstacles from obs, dust, blackout), cloud = P.ic; 2 FULL3D = uavsimlib UAV ' ...
  'Scenario Configuration (sens_uavScene from utilities/sens_env_to_uavscenario.m) / Motion Write / Lidar ' ...
  '(interpreted only, short runs), scan = P.ic.']));
end

function local_lidar_fast2d(s, ins, ts)
delete_line(find_system(s, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));
delete_block(find_system(s, 'SearchDepth', 1, 'BlockType', 'Inport'));
delete_block(find_system(s, 'SearchDepth', 1, 'BlockType', 'Outport'));
for k = 1:numel(ins), A(s, 'simulink/Sources/In1', ins{k}, 'Port', num2str(k)); end
A(s, 'simulink/Sinks/Out1', 'scan', 'Port', '1');  A(s, 'simulink/Sinks/Out1', 'cloud', 'Port', '2');
nb = 'P.sensors.lidar.nBeams';
A(s, 'simulink/Sources/Random Number', 'NR', 'Mean', ['zeros(1,' nb ')'], 'Variance', ['ones(1,' nb ')'], ...
  'Seed', 'P.sensors.lidar.seedNR', 'SampleTime', ts);
u = {'Drop', 'Dust', 'Film'};
for k = 1:3
    A(s, 'simulink/Sources/Uniform Random Number', ['U' u{k}], 'Minimum', ['zeros(1,' nb ')'], ...
      'Maximum', ['ones(1,' nb ')'], 'Seed', ['P.sensors.lidar.seed' u{k}], 'SampleTime', ts);
end
MF(s, 'Scan2D', sprintf(['function [ranges, valid] = fcn(poseL, nR, uDrop, uDust, uFilm, dust, blackout, obsN, obsXY, obsR, cyl, box, film, angles, c)\n' ...
    '%%#codegen\n' ...
    '[ranges, valid] = sens_lidar2d_fast(poseL, nR, uDrop, uDust, uFilm, dust, blackout, obsN, obsXY, obsR, cyl, box, film, angles, c);\n']), ...
    ['FAST2D height-band ray casting (sensors spec 3.4, F3): subsystems/sensors/sens_lidar2d_fast.m; constants on ' ...
     'inputs 11..15 (P.world.obstacles.cyl/box/film, P.sensors.lidar.angles, sens_lidar2d_fast_prm(P.sensors.lidar, P.world))']);
L(s, 'poseL/1', 'Scan2D/1', 'poseL');  L(s, 'NR/1', 'Scan2D/2', 'nR');  L(s, 'UDrop/1', 'Scan2D/3', 'uDrop');
L(s, 'UDust/1', 'Scan2D/4', 'uDust');  L(s, 'UFilm/1', 'Scan2D/5', 'uFilm');  L(s, 'dust/1', 'Scan2D/6', 'dust');
L(s, 'blackout/1', 'Scan2D/7', 'blackout');  L(s, 'obsN/1', 'Scan2D/8', 'obsN');  L(s, 'obsXY/1', 'Scan2D/9', 'obsXY');
L(s, 'obsR/1', 'Scan2D/10', 'obsR');
cst = {'Cyl', 'P.world.obstacles.cyl', 'cyl'; 'Box', 'P.world.obstacles.box', 'box'; ...
       'Film', 'P.world.obstacles.film', 'film'; 'BeamAng', 'P.sensors.lidar.angles', 'angles'; ...
       'LidarPrm', 'sens_lidar2d_fast_prm(P.sensors.lidar, P.world)', 'c'};
for k = 1:size(cst, 1)
    C(s, cst{k, 1}, cst{k, 2}, 'VectorParams1D', 'off');
    L(s, [cst{k, 1} '/1'], sprintf('Scan2D/%d', 10 + k), cst{k, 3});
end
C(s, 'Angles', 'single(P.sensors.lidar.angles)', 'OutDataTypeStr', 'single', 'SampleTime', ts);
BC(s, 'BC_Scan', 'Bus_LidarScanRaw', {'t/1', 'Scan2D/1', 'Angles/1', 'Scan2D/2'});
L(s, 'BC_Scan/1', 'scan/1', 'scan');
C(s, 'CloudIC', 'P.ic.Bus_LidarCloudRaw', 'OutDataTypeStr', 'Bus: Bus_LidarCloudRaw', 'SampleTime', ts);
L(s, 'CloudIC/1', 'cloud/1', 'cloud_inactive');
for k = 8:numel(ins), TERM(s, [ins{k} '/1'], ['Term_' ins{k}]); end
set_param(s, 'Description', 'FAST2D (VC_LIDAR == 1): 360-beam height-band scan, lidar_cloud = P.ic.Bus_LidarCloudRaw.');
end

function local_lidar_full3d(s, ins, ts)
delete_line(find_system(s, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));
delete_block(find_system(s, 'SearchDepth', 1, 'BlockType', 'Inport'));
delete_block(find_system(s, 'SearchDepth', 1, 'BlockType', 'Outport'));
for k = 1:numel(ins), A(s, 'simulink/Sources/In1', ins{k}, 'Port', num2str(k)); end
A(s, 'simulink/Sinks/Out1', 'scan', 'Port', '1');  A(s, 'simulink/Sinks/Out1', 'cloud', 'Port', '2');
F = 'P.sensors.lidar.full3d.';
A(s, 'uavsimlib/UAV Scenario Configuration', 'ScenarioConfig', 'UAVScenarioObject', 'sens_uavScene', 'SampleTime', ts);
A(s, 'uavsimlib/UAV Scenario Motion Write', 'MotionWrite', 'PlatName', '''Shaddad''', 'ReferenceFrame', 'ENU');
L(s, 'pos_W/1', 'MotionWrite/1', 'pos_W');  L(s, 'q_WB/1', 'MotionWrite/2', 'q_WB');
L(s, 'vel_W/1', 'MotionWrite/3', 'vel_W');  L(s, 'acc_W/1', 'MotionWrite/4', 'acc_W');
L(s, 'omega_B/1', 'MotionWrite/5', 'omega_B');
A(s, 'uavsimlib/UAV Scenario Lidar', 'Lidar3D', 'SensorName', 'Shaddad/Lidar', 'MaxRange', [F 'MaxRange'], ...
  'RangeAccuracy', [F 'RangeAccuracy'], 'AzimuthLimits', [F 'AzimuthLimits'], ...
  'AzimuthResolution', [F 'AzimuthResolution'], 'ElevationLimits', [F 'ElevationLimits'], ...
  'ElevationResolution', [F 'ElevationResolution'], 'HasNoise', 'on', 'HasOrganizedOutput', 'on', 'SampleTime', ts);
% pad the organised [nEl nAz 3] cloud with NaN to Bus_LidarCloudRaw [32 1024 3]
C(s, 'PadRows', ['NaN(P.sensors.lidar.cloudSize(1) - ' F 'nEl, ' F 'nAz, 3)'], 'SampleTime', ts);
C(s, 'PadCols', ['NaN(P.sensors.lidar.cloudSize(1), P.sensors.lidar.cloudSize(2) - ' F 'nAz, 3)'], 'SampleTime', ts);
A(s, 'simulink/Math Operations/Matrix Concatenate', 'CatRows', 'NumInputs', '2', 'ConcatenateDimension', '1');
A(s, 'simulink/Math Operations/Matrix Concatenate', 'CatCols', 'NumInputs', '2', 'ConcatenateDimension', '2');
L(s, 'Lidar3D/1', 'CatRows/1', 'cloud_organised');  L(s, 'PadRows/1', 'CatRows/2');
L(s, 'CatRows/1', 'CatCols/1');  L(s, 'PadCols/1', 'CatCols/2');
DTC(s, 'XYZ', 'single');  L(s, 'CatCols/1', 'XYZ/1', 'xyz_dbl');
C(s, 'NAz', [F 'nAz'], 'OutDataTypeStr', 'uint16', 'SampleTime', ts);
A(s, 'simulink/Logic and Bit Operations/Logical Operator', 'Valid', 'Operator', 'NOT', 'Inputs', '1');
L(s, 'blackout/1', 'Valid/1', 'blackout');
BC(s, 'BC_Cloud', 'Bus_LidarCloudRaw', {'t/1', 'XYZ/1', 'NAz/1', 'Valid/1'});
L(s, 'BC_Cloud/1', 'cloud/1', 'cloud');
C(s, 'ScanIC', 'P.ic.Bus_LidarScanRaw', 'OutDataTypeStr', 'Bus: Bus_LidarScanRaw', 'SampleTime', ts);
L(s, 'ScanIC/1', 'scan/1', 'scan_inactive');
for k = [2 3 5 6 7], TERM(s, [ins{k} '/1'], ['Term_' ins{k}]); end
set_param(s, 'Description', ['FULL3D (VC_LIDAR == 2): uavScenario sens_uavScene (base), platform Shaddad with sensor ' ...
    'Lidar; organised cloud padded with NaN to [32 1024 3]; lidar_scan = P.ic.Bus_LidarScanRaw. Interpreted only.']);
end

% ============================================================================================
% sens_spray_vision
% ============================================================================================
function local_sens_spray_vision(n)
ts = 'Ts.spray';
IN(n, 'truth', 1, 'Bus: Bus_TruthState', 'Ts.base');
OUT(n, 'spray', 1, 'Bus: Bus_SprayDemand');
% the bus truth_100ms: each consumer picks its element (Bus Selectors Truth_<element>)
RT(n, 'RT_truth', ts);  L(n, 'truth/1', 'RT_truth/1', 'truth');
tr = 'RT_truth/1';
C(n, 'Pts_B', 'P.sensors.spray.pts_B');
MATMUL(n, 'RWB_pts');
PICK(n, tr, 'Truth_R_WB', {'R_WB'}, {'RWB_pts/1'}, 'truth_100ms');  L(n, 'Pts_B/1', 'RWB_pts/2', 'pts_B');
RESH(n, 'Pos_col', 'col');  PICK(n, tr, 'Truth_pos_W', {'pos_W'}, {'Pos_col/1'});
C(n, 'Ones8', 'ones(1, P.sensors.spray.nSect)', 'VectorParams1D', 'off');
MATMUL(n, 'PosRep');  L(n, 'Pos_col/1', 'PosRep/1');  L(n, 'Ones8/1', 'PosRep/2');
A(n, 'simulink/Math Operations/Sum', 'Pts_W', 'Inputs', '++');
L(n, 'RWB_pts/1', 'Pts_W/1', 'pts_rel_W');  L(n, 'PosRep/1', 'Pts_W/2', 'pos_rep');
for k = 1:2
    nm = {'X', 'Y'};
    A(n, 'simulink/Signal Routing/Selector', ['Sel' nm{k}], 'NumberOfDimensions', '2', 'IndexMode', 'One-based', ...
      'IndexOptions', 'Index vector (dialog),Select all', 'Indices', sprintf('%d,1', k), 'InputPortWidth', '-1');
    L(n, 'Pts_W/1', ['Sel' nm{k} '/1'], 'pts_W');
    RESH(n, [nm{k} '1d'], '1d');  L(n, ['Sel' nm{k} '/1'], [nm{k} '1d/1']);
end
A(n, 'simulink/Lookup Tables/2-D Lookup Table', 'Veg_LUT', 'Table', 'P.world.veg.rho', ...
  'InterpMethod', 'Linear point-slope', 'BreakpointsForDimension1', 'P.world.terrain.xE', ...
  'BreakpointsForDimension2', 'P.world.terrain.yN', 'ExtrapMethod', 'Clip', ...
  'IndexSearchMethod', 'Evenly spaced points', 'DiagnosticForOutOfRangeInput', 'None');
L(n, 'X1d/1', 'Veg_LUT/1', 'x_sect');  L(n, 'Y1d/1', 'Veg_LUT/2', 'y_sect');
A(n, 'simulink/Sources/Uniform Random Number', 'U', 'Minimum', 'zeros(1, P.sensors.spray.nSect)', ...
  'Maximum', 'ones(1, P.sensors.spray.nSect)', 'Seed', 'P.sensors.spray.seeds', 'SampleTime', ts);
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Veg', 'relop', '>', 'const', 'P.sensors.spray.rho_thr');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Pd', 'relop', '<', 'const', 'P.sensors.spray.p_d');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Pfa', 'relop', '<', 'const', 'P.sensors.spray.p_fa');
L(n, 'Veg_LUT/1', 'Veg/1', 'rho');  L(n, 'U/1', 'Pd/1', 'u');  L(n, 'U/1', 'Pfa/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Hit', 'Operator', 'AND', 'Inputs', '2');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Detect', 'Operator', 'OR', 'Inputs', '2');
L(n, 'Veg/1', 'Hit/1');  L(n, 'Pd/1', 'Hit/2');  L(n, 'Hit/1', 'Detect/1', 'true_det');  L(n, 'Pfa/1', 'Detect/2', 'false_alarm');
RESH(n, 'Det_col', 'col');  L(n, 'Detect/1', 'Det_col/1', 'detected');
A(n, 'commutil2/Bit to Integer Converter', 'Mask', 'nbits', 'P.sensors.spray.nSect', 'bitOrder', 'LSB first', ...
  'signedOutputValues', 'Unsigned');
L(n, 'Det_col/1', 'Mask/1');
DTC(n, 'MaskU16', 'uint16');  L(n, 'Mask/1', 'MaskU16/1', 'mask');
A(n, 'simulink/Math Operations/Sum', 'RhoSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(n, 'Veg_LUT/1', 'RhoSum/1');
A(n, 'simulink/Math Operations/Gain', 'Density', 'Gain', '1/(P.sensors.spray.nSect*P.sensors.spray.rhoMax)');
L(n, 'RhoSum/1', 'Density/1', 'rho_sum');
DTC(n, 'DensityS', 'single');  L(n, 'Density/1', 'DensityS/1', 'density');
C(n, 'Valid', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', ts);
BS(n, 'Truth_t', {'t'});  L(n, tr, 'Truth_t/1');
BC(n, 'BC_Spray', 'Bus_SprayDemand', {'Truth_t/1', 'MaskU16/1', 'DensityS/1', 'Valid/1'});
L(n, 'BC_Spray/1', 'spray/1', 'spray');
set_param(n, 'Description', sprintf([ ...
  'sens_spray_vision (T11, D24; blocks only). Ports: in 1 truth Bus_TruthState 0.001; out 1 spray Bus_SprayDemand 0.1.\n' ...
  '8 boom section points [x_boom, (i - 4.5) boom_w/8] -> world via R_WB -> 2-D Lookup P.world.veg.rho; detected = ' ...
  '(rho > rho_thr AND U < p_d) OR (U < p_fa) (Uniform Random Number, seeds P.sensors.spray.seeds) -> Bit to Integer ' ...
  '(LSB = section 1) -> section_mask; target_density = mean(rho)/3.']));
end

% ============================================================================================
% Shaddad_SensorModels (top, ICD 3.1 Sensors row)
% ============================================================================================
function local_Shaddad_SensorModels(n)
IN(n, 'truth', 1, 'Bus: Bus_TruthState', 'Ts.base');
IN(n, 'drive_truth', 2, 'Bus: Bus_DriveTruth', 'Ts.base');
IN(n, 'env', 3, 'Bus: Bus_EnvState', 'Ts.hk');
IN(n, 'elec_env', 4, 'Bus: Bus_ElecEnv', 'Ts.hk');
IN(n, 'flt', 5, 'Bus: Bus_FaultInject', 'Ts.ctrl');
IN(n, 'obs', 6, 'Bus: Bus_ObstaclesDyn', 'Ts.ctrl');
outs = {'imu_raw', 'ImuRaw'; 'gnss_raw', 'GnssRaw'; 'lidar_scan', 'LidarScanRaw'; 'lidar_cloud', 'LidarCloudRaw'; ...
        'spray', 'SprayDemand'; 'sens_diag', 'SensTruthDiag'};
for k = 1:size(outs, 1), OUT(n, outs{k, 1}, k, ['Bus: Bus_' outs{k, 2}]); end
SSR = 'simulink/Ports & Subsystems/Subsystem Reference';
A(n, SSR, 'IMU', 'ReferencedSubsystem', 'sens_imu_model');
% GNSS / LiDAR contain the MATLAB Function blocks: inline copies built by the same builders (see header)
SUB(n, 'GNSS');   local_sens_gnss_model([n '/GNSS']);
SUB(n, 'LiDAR');  local_sens_lidar_model([n '/LiDAR']);
A(n, SSR, 'SprayVision', 'ReferencedSubsystem', 'sens_spray_vision');
L(n, 'truth/1', 'IMU/1', 'truth');  L(n, 'drive_truth/1', 'IMU/2', 'drive_truth');
L(n, 'elec_env/1', 'IMU/3', 'elec_env');  L(n, 'flt/1', 'IMU/4', 'flt');
L(n, 'truth/1', 'GNSS/1');  L(n, 'elec_env/1', 'GNSS/2');  L(n, 'flt/1', 'GNSS/3');
L(n, 'truth/1', 'LiDAR/1');  L(n, 'flt/1', 'LiDAR/2');  L(n, 'obs/1', 'LiDAR/3', 'obs');
L(n, 'truth/1', 'SprayVision/1');
TERM(n, 'env/1', 'Term_env');
L(n, 'IMU/1', 'imu_raw/1', 'imu_raw');  L(n, 'GNSS/1', 'gnss_raw/1', 'gnss_raw');
L(n, 'LiDAR/1', 'lidar_scan/1', 'lidar_scan');  L(n, 'LiDAR/2', 'lidar_cloud/1', 'lidar_cloud');
L(n, 'SprayVision/1', 'spray/1', 'spray');
BC(n, 'BC_Diag', 'Bus_SensTruthDiag', {'GNSS/2', 'GNSS/3', 'GNSS/4', 'GNSS/5', 'GNSS/6', 'GNSS/7', 'LiDAR/3', 'LiDAR/4'});
L(n, 'BC_Diag/1', 'sens_diag/1', 'sens_diag');
set_param(n, 'Description', sprintf([ ...
  'Shaddad_SensorModels (T11; ICD 3.1 Sensors row). In: 1 truth Bus_TruthState 0.001, 2 drive_truth Bus_DriveTruth 0.001, ' ...
  '3 env Bus_EnvState 0.1 (unused), 4 elec_env Bus_ElecEnv 0.1, 5 flt Bus_FaultInject 0.01, 6 obs Bus_ObstaclesDyn 0.01. ' ...
  'Out: 1 imu_raw Bus_ImuRaw 0.005, 2 gnss_raw Bus_GnssRaw 0.1, 3 lidar_scan Bus_LidarScanRaw 0.1, 4 lidar_cloud ' ...
  'Bus_LidarCloudRaw 0.1, 5 spray Bus_SprayDemand 0.1, 6 sens_diag Bus_SensTruthDiag 0.1.\n' ...
  'Children: IMU = Subsystem Reference sens_imu_model, SprayVision = Subsystem Reference sens_spray_vision; ' ...
  'GNSS / LiDAR (VC_LIDAR) = inline copies of sens_gnss_model / sens_lidar_model built by the same builder ' ...
  '(R2026b: MATLAB Function blocks at most one Subsystem Reference level below a model). ' ...
  'No encoder model (the ESC models the encoders, D3).']));
end

function SUB(sys, blk)
% empty plain (virtual) Subsystem: the library block's default In1 -> Out1 is removed
p = A(sys, 'simulink/Ports & Subsystems/Subsystem', blk);
delete_line(find_system(p, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));
delete_block(find_system(p, 'SearchDepth', 1, 'BlockType', 'Inport'));
delete_block(find_system(p, 'SearchDepth', 1, 'BlockType', 'Outport'));
end

% ============================================================================================
% sens_acq_imu (MCU Task_5, single precision, D62)
% ============================================================================================
function local_sens_acq_imu(n)
ts = 'Ts.imu';
IN(n, 'imu', 1, 'Bus: Bus_ImuRaw', ts);
IN(n, 't_us', 2, 'uint32', ts, '1');
IN(n, 'fault_inject', 3, 'uint32', ts, '1');
OUT(n, 'imu_meas', 1, 'Bus: Bus_ImuMeas');
% each consumer of the imu bus picks its element with its own Bus Selector Raw_<element> (layout spec 6.2)
q = {'Acc', 'accOff', 'Acceleration'; 'Gyr', 'gyroOff', 'AngularRate'; 'Mag', 'magOff', 'MagneticField'};
for k = 1:3
    A(n, 'simulink/Math Operations/Gain', ['Axis' q{k, 1}], 'Gain', 'single(P.sensors.acq.R_SB)', ...
      'Multiplication', 'Matrix(K*u)', 'ParamDataTypeStr', 'Inherit: Same as input', ...
      'OutDataTypeStr', 'Inherit: Same as input');
    C(n, ['Off' q{k, 1}], ['single(P.sensors.acq.' q{k, 2} ')'], 'OutDataTypeStr', 'single');
    A(n, 'simulink/Math Operations/Sum', ['Cal' q{k, 1}], 'Inputs', '+-', 'OutDataTypeStr', 'Inherit: Same as first input');
    PICK(n, 'imu/1', ['Raw_' q{k, 3}], q(k, 3), {['Axis' q{k, 1} '/1']}, 'imu');
    L(n, ['Axis' q{k, 1} '/1'], ['Cal' q{k, 1} '/1'], ['axis_' q{k, 1}]);
    L(n, ['Off' q{k, 1} '/1'], ['Cal' q{k, 1} '/2'], ['off_' q{k, 1}]);
end
A(n, 'simulink/Sources/Counter Free-Running', 'Seq', 'NumBits', '16', 'tsamp', ts);
% valid = NOT fault_inject b6 AND |acc| in [5 15] m/s^2 AND fresh (below)
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'FaultBit', 'logicop', 'AND', 'UseBitMask', 'on', ...
  'BitMask', 'P.sensors.acq.faultMask', 'NumInputPorts', '1');
L(n, 'fault_inject/1', 'FaultBit/1', 'fault_inject');
A(n, 'simulink/Logic and Bit Operations/Compare To Zero', 'NoFault', 'relop', '==');
L(n, 'FaultBit/1', 'NoFault/1', 'imu_invalid_bit');
A(n, 'simulink/Math Operations/Product', 'AccSq', 'Inputs', '**');
L(n, 'CalAcc/1', 'AccSq/1', 'accel_B');  L(n, 'CalAcc/1', 'AccSq/2');
A(n, 'simulink/Math Operations/Sum', 'AccSqSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(n, 'AccSq/1', 'AccSqSum/1');
A(n, 'simulink/Math Operations/Sqrt', 'AccNorm');  L(n, 'AccSqSum/1', 'AccNorm/1');
A(n, 'simulink/Logic and Bit Operations/Interval Test', 'NormOK', 'IntervalClosedRight', 'on', ...
  'uplimit', 'P.sensors.acq.accNormRange(2)', 'IntervalClosedLeft', 'on', 'lowlimit', 'P.sensors.acq.accNormRange(1)');
L(n, 'AccNorm/1', 'NormOK/1', 'acc_norm');
% fresh = some raw axis changed within the last P.electronics.safety.imu_frozen_t: all 9 raw axes (accel, gyro, mag)
% bit-identical to the previous sample for round(imu_frozen_t/Ts.imu) samples in a row is a frozen I2C read or a
% stuck sensor (plant imu_fault 1); the sensor noise changes some axis every sample (2026-11-15, R-05, review
% MCU-03 / NAV-03). Rep_cnt = samples since the last change (uint16, saturating)
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Raw9', 'NumInputs', '3');
PICK(n, 'imu/1', 'Raw_Frozen', {'Acceleration', 'AngularRate', 'MagneticField'}, {'Raw9/1', 'Raw9/2', 'Raw9/3'});
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'RawChange', 'vinit', '0');
L(n, 'Raw9/1', 'RawChange/1', 'raw9');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'AnyChange', 'Operator', 'OR', 'Inputs', '1');
L(n, 'RawChange/1', 'AnyChange/1', 'axis_changed');
A(n, 'simulink/Discrete/Unit Delay', 'Rep_prev', 'InitialCondition', '0', 'SampleTime', ts);
C(n, 'Rep_one', '1', 'OutDataTypeStr', 'uint16');
A(n, 'simulink/Math Operations/Sum', 'Rep_inc', 'Inputs', '++', 'OutDataTypeStr', 'uint16', ...
  'SaturateOnIntegerOverflow', 'on');
C(n, 'Rep_zero', '0', 'OutDataTypeStr', 'uint16');
A(n, 'simulink/Signal Routing/Switch', 'Rep_cnt', 'Criteria', 'u2 ~= 0');
L(n, 'Rep_prev/1', 'Rep_inc/1', 'rep_prev');  L(n, 'Rep_one/1', 'Rep_inc/2', 'one');
L(n, 'Rep_zero/1', 'Rep_cnt/1', 'zero');  L(n, 'AnyChange/1', 'Rep_cnt/2', 'any_change');
L(n, 'Rep_inc/1', 'Rep_cnt/3', 'rep_inc');  L(n, 'Rep_cnt/1', 'Rep_prev/1', 'rep_cnt');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Fresh', 'relop', '<', ...
  'const', 'round(P.electronics.safety.imu_frozen_t/Ts.imu)', 'OutDataTypeStr', 'boolean');
L(n, 'Rep_cnt/1', 'Fresh/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Valid', 'Operator', 'AND', 'Inputs', '3');
L(n, 'NoFault/1', 'Valid/1', 'no_fault');  L(n, 'NormOK/1', 'Valid/2', 'norm_ok');  L(n, 'Fresh/1', 'Valid/3', 'fresh');
BS(n, 'Raw_Temperature', {'Temperature'});  L(n, 'imu/1', 'Raw_Temperature/1');
BC(n, 'BC_ImuMeas', 'Bus_ImuMeas', {'CalAcc/1', 'CalGyr/1', 'CalMag/1', 'Raw_Temperature/1', 't_us/1', 'Seq/1', ...
    'Valid/1'});
L(n, 'BC_ImuMeas/1', 'imu_meas/1', 'imu_meas');
set_param(n, 'Description', sprintf([ ...
  'sens_acq_imu (T11 for Shaddad_MCU Task_5, D62; single precision). Ports: in 1 imu Bus_ImuRaw 0.005 (IO_In: SIM ' ...
  'Shaddad_SensorModels / HW ICM20948 IMU Sensor), 2 t_us uint32 0.005 (TIM2 1 MHz), 3 fault_inject uint32 0.005 ' ...
  '(Bus_McuIn1k.fault_inject, b6 = IMU invalid); out 1 imu_meas Bus_ImuMeas 0.005.\n' ...
  'Axis map Gain P.sensors.acq.R_SB (identity), calibration offsets P.sensors.acq.*Off (0), seq = 16-bit free-running ' ...
  'counter, valid = NOT (fault_inject AND faultMask) AND |accel| in accNormRange [5 15] m/s^2 AND fresh (some of ' ...
  'the 9 raw axes changed within P.electronics.safety.imu_frozen_t: Detect Change -> OR -> uint16 count of the ' ...
  'samples since the last change < round(imu_frozen_t/Ts.imu); a frozen or stuck IMU repeats every axis bit for bit).']));
end

% ============================================================================================
% sens_odom_compute (MCU Task_10, single precision, D3 / D62)
% ============================================================================================
function local_sens_odom_compute(n)
ts = 'Ts.ctrl';
IN(n, 'enc_cnt_LR', 1, 'int32', ts, '2');
IN(n, 'wMotor_LR', 2, 'single', ts, '2');
IN(n, 't_us', 3, 'uint32', ts, '1');
IN(n, 'fb_valid', 4, 'boolean', ts, '1');
OUT(n, 'odom', 1, 'Bus: Bus_OdomMeas');
OUT(n, 'a_track', 2, 'single');
% wrap-safe window difference: modular (two's complement) int32 subtraction without saturation, i.e. the
% difference of the stored integers taken mod 2^32 and read as int32 (valid for |dCount| < 2^31 per window);
% only a real counter wrap overflows (Simulink reports the intended wrap as an overflow diagnostic, at most
% once per counter wrap, about every 2.4 h at 2 m/s)
A(n, 'simulink/Discrete/Delay', 'Window', 'DelayLength', 'P.electronics.drv.odom_window', ...
  'InitialConditionSource', 'Input port', 'SampleTime', ts);
% initial delay states = first count of each side ([2 x odom_window], so the first windows give dCount 0)
RESH(n, 'Cnt_col', 'col');
C(n, 'OnesW', 'int32(ones(1, P.electronics.drv.odom_window))', 'OutDataTypeStr', 'int32', 'VectorParams1D', 'off');
A(n, 'simulink/Math Operations/Product', 'IC_rep', 'Inputs', '2', 'Multiplication', 'Matrix(*)', ...
  'OutDataTypeStr', 'int32', 'SaturateOnIntegerOverflow', 'off');
L(n, 'enc_cnt_LR/1', 'Cnt_col/1', 'enc_cnt');  L(n, 'Cnt_col/1', 'IC_rep/1');  L(n, 'OnesW/1', 'IC_rep/2');
L(n, 'enc_cnt_LR/1', 'Window/1');  L(n, 'IC_rep/1', 'Window/2', 'ic_states');
A(n, 'simulink/Math Operations/Sum', 'dCnt', 'Inputs', '+-', 'OutDataTypeStr', 'int32', ...
  'SaturateOnIntegerOverflow', 'off');
L(n, 'enc_cnt_LR/1', 'dCnt/1');  L(n, 'Window/1', 'dCnt/2', 'cnt_window_ago');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'dCntS', 'OutDataTypeStr', 'single', ...
  'ConvertRealWorld', 'Stored Integer (SI)', 'SaturateOnIntegerOverflow', 'off');
L(n, 'dCnt/1', 'dCntS/1', 'dCount');
A(n, 'simulink/Math Operations/Gain', 'K_enc', 'Gain', ...
  'single(P.electronics.drv.K_enc/(P.electronics.drv.odom_window*Ts.odom))', ...
  'ParamDataTypeStr', 'Inherit: Same as input', 'OutDataTypeStr', 'Inherit: Same as input');
L(n, 'dCntS/1', 'K_enc/1', 'dCount_s');
% a_track = d/dt mean track speed, first-order filter tau_acc (MCU tilt compensation, D47)
A(n, 'simulink/Math Operations/Sum', 'VSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(n, 'K_enc/1', 'VSum/1', 'vTrack_LR');
A(n, 'simulink/Math Operations/Gain', 'VMean', 'Gain', 'single(0.5)', 'ParamDataTypeStr', 'Inherit: Same as input');
L(n, 'VSum/1', 'VMean/1');
A(n, 'simulink/Discrete/Discrete Derivative', 'dVdt', 'gainval', '1', 'ICPrevScaledInput', '0');
L(n, 'VMean/1', 'dVdt/1', 'v_track');
A(n, 'simulink/Discrete/Discrete Transfer Fcn', 'AccFilt', ...
  'Numerator', 'single(1 - exp(-Ts.odom/P.electronics.safety.tilt_comp.tau_acc))', ...
  'Denominator', 'single([1 -exp(-Ts.odom/P.electronics.safety.tilt_comp.tau_acc)])', 'SampleTime', ts);
L(n, 'dVdt/1', 'AccFilt/1', 'dv_dt');
DTC(n, 'AccS', 'single');  L(n, 'AccFilt/1', 'AccS/1');
L(n, 'AccS/1', 'a_track/1', 'a_track');
BC(n, 'BC_Odom', 'Bus_OdomMeas', {'K_enc/1', 'enc_cnt_LR/1', 'wMotor_LR/1', 't_us/1', 'fb_valid/1'});
L(n, 'BC_Odom/1', 'odom/1', 'odom');
set_param(n, 'Description', sprintf([ ...
  'sens_odom_compute (T11 for Shaddad_MCU Task_10, D3/D62; single precision). Ports: in 1 enc_cnt_LR int32[2] 0.01 ' ...
  '(DRV_FB3 enc_cnt L/R, wrapping), 2 wMotor_LR single[2] 0.01 (FB1 speed, rad/s), 3 t_us uint32 0.01, 4 fb_valid ' ...
  'boolean 0.01 (FB1 and FB3 fresh); out 1 odom Bus_OdomMeas 0.01, 2 a_track single 0.01 (m/s^2, d/dt of the mean ' ...
  'track speed, first-order tau_acc, for fw_imu_tilt).\n' ...
  'vTrack = (cnt - cnt[k - odom_window]) (int32, modular, no saturation) -> Data Type Conversion (stored integer) ' ...
  '-> single * K_enc/(odom_window*0.01 s) (Delay odom_window with IC = first input).']));
end
