function build_tb_structure_env()
%BUILD_TB_STRUCTURE_ENV  Build the test bench tests/structure/tb_structure_env.slx (build task T02, rule 10).
%
%   Bench of subsystems/structure/structure_env.slx: Subsystem Reference block 'Env' driven by
%     pose2D  = Constant tbe.pose (Ts 0),  speed = Constant tbe.speed (Ts 0),
%     env_ovr = Bus Creator 'Bus: Bus_EnvOverride' of
%               use_T = Step(time tbe.t_ovr, 0 -> tbe.use_T) (Ts.env) -> boolean,
%               T_amb = tbe.T_ovr, use_G = tbe.use_G, G_solar = tbe.G_ovr, use_soil = tbe.use_soil,
%               soil_id = uint8 tbe.soil_ovr, dock_contact_force = tbe.dock_force (Constants, Ts.env)
%   Outputs: the 7 Bus_EnvState elements as root Outports (yout, Dataset).
%   tbe (struct) lives in the bench's model workspace (defaults below); tests override it with
%   Simulink.SimulationInput.setVariable('tbe', s, 'Workspace', 'tb_structure_env') and pass P / Ts with
%   setVariable, so the shared base workspace is never modified.
%   Configuration: embedded copy 'tb_cfg' of cs_SimFixed (same diagnostics) with FixedStep Ts.env
%   (the structure_env contents run at 0.1 s; there are no continuous states).
%   Layout (model layout spec 6, 7): no block positions here; shaddad_bu('save') lays the bench out with
%   shaddad_layout before it saves. Every signal line is named (L5, shaddad_bu('nline')); EnvSel stands right in
%   front of the seven outports it feeds, in element order (the test reads yout by index).

name = 'tb_structure_env';
root = char(currentProject().RootFolder);
f = fullfile(root, 'tests', 'structure', [name '.slx']);
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
    shaddad_configsets('fixed');
end
shaddad_bu('fresh', name, f, 'model');

%% ---- configuration (embedded copy of cs_SimFixed, 0.1 s base step) ----
cs = copy(evalin('base', 'cs_SimFixed'));
cs.Name = 'tb_cfg';
attachConfigSet(name, cs, true);
setActiveConfigSet(name, 'tb_cfg');
try
    detachConfigSet(name, 'Configuration');
catch %#ok<CTCH> default set already gone
end
set_param(name, 'FixedStep', 'Ts.env', 'StopTime', '2', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', 'SaveTime', 'on', 'SignalLogging', 'on');

%% ---- model workspace defaults ----
tbe = struct('pose', [0 0 0], 'speed', 0, 'use_T', 0, 'T_ovr', 300, 't_ovr', 1.0, ...
             'use_G', false, 'G_ovr', 500, 'use_soil', false, 'soil_ovr', uint8(5), 'dock_force', false);
mws = get_param(name, 'ModelWorkspace');
assignin(mws, 'tbe', tbe);

A = @(lib, blk, varargin) shaddad_bu('add', lib, [name '/' blk], varargin{:});
L = @(src, dst, nm) shaddad_bu('nline', name, src, dst, nm);    % '' -> the source line's name, else the block's
tsEnv = 'Ts.env';

%% ---- device under test ----
A('simulink/Ports & Subsystems/Subsystem Reference', 'Env', 'ReferencedSubsystem', 'structure_env');

%% ---- stimuli ----
A('simulink/Sources/Constant', 'Pose', 'Value', 'tbe.pose', 'SampleTime', '0');
A('simulink/Sources/Constant', 'Speed', 'Value', 'tbe.speed', 'SampleTime', '0');
A('simulink/Sources/Step', 'Step_useT', 'Time', 'tbe.t_ovr', 'Before', '0', 'After', 'tbe.use_T', ...
  'SampleTime', tsEnv);
A('simulink/Signal Attributes/Data Type Conversion', 'useT_bool', 'OutDataTypeStr', 'boolean');
A('simulink/Sources/Constant', 'T_ovr', 'Value', 'tbe.T_ovr', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'useG', 'Value', 'tbe.use_G', 'OutDataTypeStr', 'boolean', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'G_ovr', 'Value', 'tbe.G_ovr', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'useSoil', 'Value', 'tbe.use_soil', 'OutDataTypeStr', 'boolean', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'Soil_ovr', 'Value', 'tbe.soil_ovr', 'OutDataTypeStr', 'uint8', 'SampleTime', tsEnv);
A('simulink/Sources/Constant', 'DockForce', 'Value', 'tbe.dock_force', 'OutDataTypeStr', 'boolean', 'SampleTime', tsEnv);
A('simulink/Signal Routing/Bus Creator', 'EnvOvr', 'Inputs', '7', 'OutDataTypeStr', 'Bus: Bus_EnvOverride', ...
  'NonVirtualBus', 'on');
L('Step_useT/1', 'useT_bool/1', 'use_T_dbl');
L('useT_bool/1', 'EnvOvr/1', 'use_T');
L('T_ovr/1', 'EnvOvr/2', 'T_amb');
L('useG/1', 'EnvOvr/3', 'use_G');
L('G_ovr/1', 'EnvOvr/4', 'G_solar');
L('useSoil/1', 'EnvOvr/5', 'use_soil');
L('Soil_ovr/1', 'EnvOvr/6', 'soil_id');
L('DockForce/1', 'EnvOvr/7', 'dock_contact_force');
L('Pose/1', 'Env/1', 'pose2D');
L('Speed/1', 'Env/2', 'speed');
L('EnvOvr/1', 'Env/3', 'env_ovr');

%% ---- outputs ----
el = {'T_amb', 'T_sky', 'T_ground', 'G_solar', 'soil_id', 'dock_contact', 'dist_dock'};
A('simulink/Signal Routing/Bus Selector', 'EnvSel', 'OutputSignals', strjoin(el, ','));
L('Env/1', 'EnvSel/1', 'env');
for k = 1:numel(el)
    A('simulink/Sinks/Out1', ['o_' el{k}], 'Port', num2str(k));
    L(sprintf('EnvSel/%d', k), ['o_' el{k} '/1'], '');
end

set_param(name, 'Description', 'Test bench of structure_env (build task T02); built by scripts/build/build_tb_structure_env.m');
shaddad_bu('save', name, f);
shaddad_bu('finalize', name, f);
close_system(name, 0);
end
