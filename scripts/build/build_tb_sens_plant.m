function build_tb_sens_plant()
%BUILD_TB_SENS_PLANT  Build the sensor test bench tests/sensors/tb_sens_plant.slx (build task T11, rule 10).
%
%   Root inputs (external input Dataset from tests/sensors/sens_tb_input.m, interpolated to 1 ms):
%     1 truth Bus_TruthState 0.001, 2 drive_truth Bus_DriveTruth 0.001, 3 env Bus_EnvState 0.1,
%     4 elec_env Bus_ElecEnv 0.1, 5 flt Bus_FaultInject 0.01, 6 obs Bus_ObstaclesDyn 0.01,
%     7 enc_cnt int32[2] 0.01 (DRV_FB3 counts for the odometry path)
%   Devices under test (Subsystem Reference):
%     Sensors = Shaddad_SensorModels (plant side), AcqIMU = sens_acq_imu (MCU Task_5, with t_us from a
%     digital clock and fault_inject b6 = (flt.imu_fault == 3) as in MCU_In), Odom = sens_odom_compute
%     (MCU Task_10, wMotor 0, fb_valid true)
%   Root outputs (yout, Dataset): 1 imu_raw, 2 gnss_raw, 3 lidar_scan, 4 spray, 5 sens_diag, 6 imu_meas,
%     7 odom, 8 a_track. lidar_cloud.xyz / .valid go to To Workspace blocks (variables tb_cloud_xyz,
%     tb_cloud_valid) limited to the last tbs.cloudN samples (the cloud is 0.4 MB per sample).
%   tbs (struct) lives in the model workspace (defaults below); tests override it with
%   Simulink.SimulationInput.setVariable('tbs', s, 'Workspace', 'tb_sens_plant') and pass P with setVariable,
%   so the shared base workspace is not modified. Configuration: embedded copy of cs_SimFixed (1 ms).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out the bench with
%   shaddad_layout(name, 'Recursive', true). Every signal line is named (L5, shaddad_bu('nline')); the cloud bus
%   is picked by one Bus Selector in front of each To Workspace block (spec 6.2).

name = 'tb_sens_plant';
root = char(currentProject().RootFolder);
f = fullfile(root, 'tests', 'sensors', [name '.slx']);
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
    shaddad_configsets('fixed');
end
% referenced sensor files left dirty in memory (e.g. by a rebuild of a child) would block save_system
if bdIsLoaded(name), close_system(name, 0); end
for m = {'Shaddad_SensorModels', 'sens_imu_model', 'sens_gnss_model', 'sens_lidar_model', 'sens_spray_vision', ...
         'sens_acq_imu', 'sens_odom_compute'}
    if bdIsLoaded(m{1}) && strcmp(get_param(m{1}, 'Dirty'), 'on')
        try
            close_system(m{1}, 0);
        catch %#ok<CTCH> referenced by a loaded model of another task
        end
    end
end
shaddad_bu('fresh', name, f, 'model');

%% ---- configuration ----
cs = copy(evalin('base', 'cs_SimFixed'));
cs.Name = 'tb_cfg';
attachConfigSet(name, cs, true);
setActiveConfigSet(name, 'tb_cfg');
try
    detachConfigSet(name, 'Configuration');
catch %#ok<CTCH> default set already gone
end
set_param(name, 'FixedStep', 'Ts.base', 'StopTime', '1', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', 'SaveTime', 'on', 'LoadExternalInput', 'off', ...
    'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');

mws = get_param(name, 'ModelWorkspace');
assignin(mws, 'tbs', struct('cloudN', 2));

A = @(lib, blk, varargin) shaddad_bu('add', lib, [name '/' blk], varargin{:});
% a named signal line (layout spec 6.1, L5): the name nm, else the name the source port's line already has, else
% the source block's name; a Bus Selector's output keeps '<element>'
L = @(src, dst, nm) shaddad_bu('nline', name, src, dst, nm);

%% ---- root inputs ----
ins = {'truth', 'Bus: Bus_TruthState', 'Ts.base', '-1'; 'drive_truth', 'Bus: Bus_DriveTruth', 'Ts.base', '-1'; ...
       'env', 'Bus: Bus_EnvState', 'Ts.hk', '-1'; 'elec_env', 'Bus: Bus_ElecEnv', 'Ts.hk', '-1'; ...
       'flt', 'Bus: Bus_FaultInject', 'Ts.ctrl', '-1'; 'obs', 'Bus: Bus_ObstaclesDyn', 'Ts.ctrl', '-1'; ...
       'enc_cnt', 'int32', 'Ts.ctrl', '2'};
for k = 1:size(ins, 1)
    A('simulink/Sources/In1', ins{k, 1}, 'Port', num2str(k), 'OutDataTypeStr', ins{k, 2}, ...
      'SampleTime', ins{k, 3}, 'PortDimensions', ins{k, 4}, 'Interpolate', 'on');
end

%% ---- devices under test ----
SSR = 'simulink/Ports & Subsystems/Subsystem Reference';
A(SSR, 'Sensors', 'ReferencedSubsystem', 'Shaddad_SensorModels');
for k = 1:6, L(sprintf('%s/1', ins{k, 1}), sprintf('Sensors/%d', k), ins{k, 1}); end

% MCU-side IMU acquisition (Task_5): t_us from a digital clock, fault_inject b6 = (imu_fault == 3)
A(SSR, 'AcqIMU', 'ReferencedSubsystem', 'sens_acq_imu');
A('simulink/Sources/Digital Clock', 'Clock5', 'SampleTime', 'Ts.imu');
A('simulink/Math Operations/Gain', 'ToUs5', 'Gain', '1e6');
A('simulink/Signal Attributes/Data Type Conversion', 'Us5', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
  'SaturateOnIntegerOverflow', 'off');
L('Clock5/1', 'ToUs5/1', 't5');  L('ToUs5/1', 'Us5/1', 't5_us');
A('simulink/Signal Attributes/Rate Transition', 'RT_flt5', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', 'Ts.imu');
L('flt/1', 'RT_flt5/1', '');
A('simulink/Signal Routing/Bus Selector', 'FltImu', 'OutputSignals', 'imu_fault');
L('RT_flt5/1', 'FltImu/1', 'flt_5ms');
A('simulink/Logic and Bit Operations/Compare To Constant', 'ImuDrop', 'relop', '==', 'const', '3');
L('FltImu/1', 'ImuDrop/1', '');
A('simulink/Signal Attributes/Data Type Conversion', 'DropU32', 'OutDataTypeStr', 'uint32');
L('ImuDrop/1', 'DropU32/1', 'imu_dropout');
A('simulink/Math Operations/Gain', 'Bit6', 'Gain', '64', 'OutDataTypeStr', 'uint32', 'ParamDataTypeStr', 'uint32');
L('DropU32/1', 'Bit6/1', 'imu_dropout_u32');
L('Sensors/1', 'AcqIMU/1', 'imu_raw');  L('Us5/1', 'AcqIMU/2', 't_us5');  L('Bit6/1', 'AcqIMU/3', 'fault_inject');

% MCU-side odometry (Task_10)
A(SSR, 'Odom', 'ReferencedSubsystem', 'sens_odom_compute');
A('simulink/Sources/Constant', 'WMotor', 'Value', 'single([0 0])', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.ctrl');
A('simulink/Sources/Digital Clock', 'Clock10', 'SampleTime', 'Ts.ctrl');
A('simulink/Math Operations/Gain', 'ToUs10', 'Gain', '1e6');
A('simulink/Signal Attributes/Data Type Conversion', 'Us10', 'OutDataTypeStr', 'uint32', 'RndMeth', 'Floor', ...
  'SaturateOnIntegerOverflow', 'off');
A('simulink/Sources/Constant', 'FbValid', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.ctrl');
L('Clock10/1', 'ToUs10/1', 't10');  L('ToUs10/1', 'Us10/1', 't10_us');
L('enc_cnt/1', 'Odom/1', 'enc_cnt');  L('WMotor/1', 'Odom/2', 'wMotor');  L('Us10/1', 'Odom/3', 't_us10');
L('FbValid/1', 'Odom/4', 'fb_valid');

%% ---- root outputs ----
outs = {'imu_raw', 'Sensors/1'; 'gnss_raw', 'Sensors/2'; 'lidar_scan', 'Sensors/3'; 'spray', 'Sensors/5'; ...
        'sens_diag', 'Sensors/6'; 'imu_meas', 'AcqIMU/1'; 'odom', 'Odom/1'; 'a_track', 'Odom/2'};
for k = 1:size(outs, 1)
    A('simulink/Sinks/Out1', ['o_' outs{k, 1}], 'Port', num2str(k));
    L(outs{k, 2}, ['o_' outs{k, 1} '/1'], outs{k, 1});
end
% lidar cloud: last tbs.cloudN samples only (one Bus Selector in front of each To Workspace block)
A('simulink/Sinks/To Workspace', 'CloudXYZ', 'VariableName', 'tb_cloud_xyz', 'MaxDataPoints', 'tbs.cloudN', ...
  'SaveFormat', 'Structure With Time', 'SampleTime', '-1');
A('simulink/Sinks/To Workspace', 'CloudValid', 'VariableName', 'tb_cloud_valid', 'MaxDataPoints', 'tbs.cloudN', ...
  'SaveFormat', 'Structure With Time', 'SampleTime', '-1');
A('simulink/Signal Routing/Bus Selector', 'Cloud_xyz', 'OutputSignals', 'xyz');
L('Sensors/4', 'Cloud_xyz/1', 'lidar_cloud');  L('Cloud_xyz/1', 'CloudXYZ/1', '');
A('simulink/Signal Routing/Bus Selector', 'Cloud_valid', 'OutputSignals', 'valid');
L('Sensors/4', 'Cloud_valid/1', '');  L('Cloud_valid/1', 'CloudValid/1', '');

set_param(name, 'Description', ['T11 sensor bench: Shaddad_SensorModels + sens_acq_imu + sens_odom_compute driven by ' ...
    'external inputs (tests/sensors/sens_tb_input.m); built by scripts/build/build_tb_sens_plant.m.']);
shaddad_bu('save', name, f);
close_system(name, 0);
end
