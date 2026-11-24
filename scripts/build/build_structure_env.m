function build_structure_env()
%BUILD_STRUCTURE_ENV  Build subsystems/structure/structure_env.slx (Subsystem Reference, owner structure,
%   build task T02.8). Single producer of Bus_EnvState (D10); instantiated inside structure_plant by T03.
%
%   Ports: in 1 pose2D double[3] [xE yN psi_enu] (Ts 0), 2 speed double (Ts 0), 3 env_ovr Bus_EnvOverride
%   (Ts.env = 0.1 s); out 1 env Bus_EnvState (Ts.env).
%   Contents (library blocks only, every number from P.world / Ts):
%     Zero-Order Hold Ts.env on pose2D and speed, Demux -> x, y, psi
%     T_amb    = Switch(use_T, ovr.T_amb, base), base = Switch(P.world.env.use_diurnal, Sine Wave, Constant
%                P.world.env.T_amb); Sine Wave time based, Amplitude diurnal.T_amp, Bias diurnal.T_mean,
%                Frequency 2*pi/86400, Phase 2*pi*(t0_hour - h_peak + 6)/24 (peak at h_peak, t = 0 is
%                t0_hour o'clock), Ts.env
%     G_solar  = Switch(use_G, ovr.G_solar, base G) x 2-D Lookup Table P.world.shade(x, y) (Linear
%                point-slope, Clip, Evenly spaced points); base G = Switch(use_diurnal, max(0, Sine Wave
%                Amplitude diurnal.G_pk, Phase 2*pi*(t0_hour - 6)/24: solar noon at 12:00), Constant
%                P.world.env.G_solar)
%     T_sky    = T_amb + P.world.env.T_sky_offset (Bias)
%     T_ground = T_amb + P.world.env.T_ground_offset * G_solar / 1000
%     soil_id  = uint8(Switch(use_soil, double(ovr.soil_id), 2-D Lookup P.world.soilId (Nearest, Clip)))
%     dist_dock = hypot(x - dock.pose(1), y - dock.pose(2))
%     dock_contact = dock_contact_force OR (dist_dock < tol_xy AND |atan2(sin(dpsi), cos(dpsi))| < tol_yaw
%                    AND |speed| < v_max), dpsi = psi - dock.pose(3)
%     Bus Creator 'Bus: Bus_EnvState' (nonvirtual) -> env
%   Layout (model layout spec 6, 7): no block positions here; shaddad_bu('save') lays the file out with
%   shaddad_layout before it saves. Every signal line is named (L5, shaddad_bu('nline')); env_ovr is picked by a
%   Bus Selector Ovr_<signal> in front of each consumer (spec 6.2, a bus at a classic port).
%   Idempotent: closes the model if loaded, deletes the file and rebuilds it (build plan rule 3).

name = 'structure_env';
root = char(currentProject().RootFolder);
f = fullfile(root, 'subsystems', 'structure', [name '.slx']);
shaddad_bu('fresh', name, f, 'ssr');

A = @(lib, blk, varargin) shaddad_bu('add', lib, [name '/' blk], varargin{:});
L = @(src, dst, nm) shaddad_bu('nline', name, src, dst, nm);    % '' -> the source line's name, else the block's
PK = @(blk, sigs, dsts) local_pick(name, blk, sigs, dsts);
tsEnv = 'Ts.env';
lut2 = {'BreakpointsForDimension1', 'P.world.terrain.xE', 'BreakpointsForDimension2', 'P.world.terrain.yN', ...
        'ExtrapMethod', 'Clip', 'IndexSearchMethod', 'Evenly spaced points', ...
        'DiagnosticForOutOfRangeInput', 'None'};
sw = {'Criteria', 'u2 ~= 0'};

%% ---- ports ----
A('simulink/Sources/In1', 'pose2D', 'Port', '1', 'PortDimensions', '3', 'OutDataTypeStr', 'double', 'SampleTime', '0');
A('simulink/Sources/In1', 'speed', 'Port', '2', 'PortDimensions', '1', 'OutDataTypeStr', 'double', 'SampleTime', '0');
A('simulink/Sources/In1', 'env_ovr', 'Port', '3', 'OutDataTypeStr', 'Bus: Bus_EnvOverride', 'SampleTime', tsEnv);
A('simulink/Sinks/Out1', 'env', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_EnvState', 'SampleTime', tsEnv);

%% ---- input sampling ----
A('simulink/Discrete/Zero-Order Hold', 'ZOH_pose', 'SampleTime', tsEnv);
A('simulink/Discrete/Zero-Order Hold', 'ZOH_speed', 'SampleTime', tsEnv);
A('simulink/Signal Routing/Demux', 'Pose_xyz', 'Outputs', '3');
L('pose2D/1', 'ZOH_pose/1', 'pose2D');
L('speed/1', 'ZOH_speed/1', 'speed');
L('ZOH_pose/1', 'Pose_xyz/1', 'pose2D_s');

%% ---- ambient temperature ----
A('simulink/Sources/Constant', 'T_amb_const', 'Value', 'P.world.env.T_amb');
A('simulink/Sources/Sine Wave', 'T_amb_diurnal', 'SineType', 'Time based', 'TimeSource', 'Use simulation time', ...
  'Amplitude', 'P.world.env.diurnal.T_amp', 'Bias', 'P.world.env.diurnal.T_mean', 'Frequency', '2*pi/86400', ...
  'Phase', '2*pi*(P.world.env.diurnal.t0_hour - P.world.env.diurnal.h_peak + 6)/24', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'Use_diurnal', 'Value', 'P.world.env.use_diurnal');
A('simulink/Signal Routing/Switch', 'Sw_T_base', sw{:});
A('simulink/Signal Routing/Switch', 'Sw_T', sw{:});
L('T_amb_diurnal/1', 'Sw_T_base/1', 'T_amb_diurnal');
L('Use_diurnal/1', 'Sw_T_base/2', 'use_diurnal');
L('T_amb_const/1', 'Sw_T_base/3', 'T_amb_const');
PK('Ovr_T', {'T_amb', 'use_T'}, {'Sw_T/1', 'Sw_T/2'});
L('Sw_T_base/1', 'Sw_T/3', 'T_amb_base');

%% ---- solar irradiance ----
A('simulink/Sources/Constant', 'G_const', 'Value', 'P.world.env.G_solar');
A('simulink/Sources/Sine Wave', 'G_diurnal', 'SineType', 'Time based', 'TimeSource', 'Use simulation time', ...
  'Amplitude', 'P.world.env.diurnal.G_pk', 'Bias', '0', 'Frequency', '2*pi/86400', ...
  'Phase', '2*pi*(P.world.env.diurnal.t0_hour - 6)/24', 'SampleTime', tsEnv);
A('simulink/Discontinuities/Saturation', 'G_daylight', 'UpperLimit', 'inf', 'LowerLimit', '0');
A('simulink/Signal Routing/Switch', 'Sw_G_base', sw{:});
A('simulink/Signal Routing/Switch', 'Sw_G', sw{:});
A('simulink/Lookup Tables/2-D Lookup Table', 'Shade_LUT', 'Table', 'P.world.shade', ...
  'InterpMethod', 'Linear point-slope', lut2{:});
A('simulink/Math Operations/Product', 'G_at_robot', 'Inputs', '2');
L('G_diurnal/1', 'G_daylight/1', 'G_diurnal');
L('G_daylight/1', 'Sw_G_base/1', 'G_day');
L('Use_diurnal/1', 'Sw_G_base/2', '');
L('G_const/1', 'Sw_G_base/3', 'G_const');
PK('Ovr_G', {'G_solar', 'use_G'}, {'Sw_G/1', 'Sw_G/2'});
L('Sw_G_base/1', 'Sw_G/3', 'G_base');
L('Pose_xyz/1', 'Shade_LUT/1', 'x');
L('Pose_xyz/2', 'Shade_LUT/2', 'y');
L('Sw_G/1', 'G_at_robot/1', 'G_open');
L('Shade_LUT/1', 'G_at_robot/2', 'shade');

%% ---- sky and ground temperature ----
A('simulink/Math Operations/Bias', 'T_sky_bias', 'Bias', 'P.world.env.T_sky_offset');
A('simulink/Math Operations/Gain', 'G_to_dTground', 'Gain', 'P.world.env.T_ground_offset/1000');
A('simulink/Math Operations/Sum', 'T_ground_sum', 'Inputs', '++');

%% ---- soil class ----
A('simulink/Lookup Tables/2-D Lookup Table', 'Soil_LUT', 'Table', 'P.world.soilId', ...
  'InterpMethod', 'Nearest', lut2{:});
A('simulink/Signal Attributes/Data Type Conversion', 'Soil_ovr_dbl', 'OutDataTypeStr', 'double');
A('simulink/Signal Routing/Switch', 'Sw_soil', sw{:});
A('simulink/Signal Attributes/Data Type Conversion', 'Soil_u8', 'OutDataTypeStr', 'uint8', ...
  'RndMeth', 'Nearest', 'SaturateOnIntegerOverflow', 'on');
L('Pose_xyz/1', 'Soil_LUT/1', '');
L('Pose_xyz/2', 'Soil_LUT/2', '');
PK('Ovr_soil_id', {'soil_id'}, {'Soil_ovr_dbl/1'});
L('Soil_ovr_dbl/1', 'Sw_soil/1', 'soil_ovr');
PK('Ovr_use_soil', {'use_soil'}, {'Sw_soil/2'});
L('Soil_LUT/1', 'Sw_soil/3', 'soil_map');
L('Sw_soil/1', 'Soil_u8/1', 'soil_dbl');

%% ---- dock distance and contact ----
A('simulink/Sources/Constant', 'Dock_x', 'Value', 'P.world.dock.pose(1)');
A('simulink/Sources/Constant', 'Dock_y', 'Value', 'P.world.dock.pose(2)');
A('simulink/Sources/Constant', 'Dock_psi', 'Value', 'P.world.dock.pose(3)');
A('simulink/Math Operations/Sum', 'dx', 'Inputs', '+-');
A('simulink/Math Operations/Sum', 'dy', 'Inputs', '+-');
A('simulink/Math Operations/Math Function', 'Dist', 'Operator', 'hypot');
A('simulink/Math Operations/Sum', 'dpsi', 'Inputs', '+-');
A('simulink/Math Operations/Trigonometric Function', 'Sin_dpsi', 'Operator', 'sin');
A('simulink/Math Operations/Trigonometric Function', 'Cos_dpsi', 'Operator', 'cos');
A('simulink/Math Operations/Trigonometric Function', 'Wrap_dpsi', 'Operator', 'atan2');
A('simulink/Math Operations/Abs', 'Abs_dpsi');
A('simulink/Math Operations/Abs', 'Abs_speed');
A('simulink/Logic and Bit Operations/Compare To Constant', 'In_xy', 'relop', '<', 'const', 'P.world.dock.tol_xy');
A('simulink/Logic and Bit Operations/Compare To Constant', 'In_yaw', 'relop', '<', 'const', 'P.world.dock.tol_yaw');
A('simulink/Logic and Bit Operations/Compare To Constant', 'Stopped', 'relop', '<', 'const', 'P.world.dock.v_max');
A('simulink/Logic and Bit Operations/Logical Operator', 'Docked', 'Operator', 'AND', 'Inputs', '3');
A('simulink/Logic and Bit Operations/Logical Operator', 'Contact', 'Operator', 'OR', 'Inputs', '2');
L('Pose_xyz/1', 'dx/1', '');
L('Dock_x/1', 'dx/2', 'dock_x');
L('Pose_xyz/2', 'dy/1', '');
L('Dock_y/1', 'dy/2', 'dock_y');
L('dx/1', 'Dist/1', 'dx');
L('dy/1', 'Dist/2', 'dy');
L('Pose_xyz/3', 'dpsi/1', 'psi');
L('Dock_psi/1', 'dpsi/2', 'dock_psi');
L('dpsi/1', 'Sin_dpsi/1', 'dpsi');
L('dpsi/1', 'Cos_dpsi/1', '');
L('Sin_dpsi/1', 'Wrap_dpsi/1', 'sin_dpsi');
L('Cos_dpsi/1', 'Wrap_dpsi/2', 'cos_dpsi');
L('Wrap_dpsi/1', 'Abs_dpsi/1', 'dpsi_wrapped');
L('Abs_dpsi/1', 'In_yaw/1', 'abs_dpsi');
L('ZOH_speed/1', 'Abs_speed/1', 'speed_s');
L('Abs_speed/1', 'Stopped/1', 'abs_speed');
L('In_xy/1', 'Docked/1', 'in_xy');
L('In_yaw/1', 'Docked/2', 'in_yaw');
L('Stopped/1', 'Docked/3', 'stopped');
L('Docked/1', 'Contact/1', 'docked');
PK('Ovr_dock_force', {'dock_contact_force'}, {'Contact/2'});

%% ---- Bus_EnvState ----
A('simulink/Signal Routing/Bus Creator', 'EnvState', 'Inputs', '7', ...
  'OutDataTypeStr', 'Bus: Bus_EnvState', 'NonVirtualBus', 'on');
L('Sw_T/1', 'EnvState/1', 'T_amb');
L('Sw_T/1', 'T_sky_bias/1', '');
L('Sw_T/1', 'T_ground_sum/1', '');
L('T_sky_bias/1', 'EnvState/2', 'T_sky');
L('G_at_robot/1', 'G_to_dTground/1', '');
L('G_to_dTground/1', 'T_ground_sum/2', 'dT_ground');
L('T_ground_sum/1', 'EnvState/3', 'T_ground');
L('G_at_robot/1', 'EnvState/4', 'G_solar');
L('Soil_u8/1', 'EnvState/5', 'soil_id');
L('Contact/1', 'EnvState/6', 'dock_contact');
L('Dist/1', 'EnvState/7', 'dist_dock');
L('Dist/1', 'In_xy/1', '');
L('EnvState/1', 'env/1', 'env');

set_param(name, 'Description', ['structure_env (build task T02): ambient conditions at the robot and ' ...
    'dock contact (Bus_EnvState) from P.world; built by scripts/build/build_structure_env.m']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

function local_pick(name, blk, sigs, dsts)
% a Bus Selector blk of env_ovr right in front of its one consumer (layout spec 6.2): it selects sigs, output k
% into dsts{k} ('Blk/j')
shaddad_bu('add', 'simulink/Signal Routing/Bus Selector', [name '/' blk], 'OutputSignals', strjoin(sigs, ','));
shaddad_bu('nline', name, 'env_ovr/1', [blk '/1'], 'env_ovr');
for k = 1:numel(sigs), shaddad_bu('nline', name, sprintf('%s/%d', blk, k), dsts{k}); end
end
