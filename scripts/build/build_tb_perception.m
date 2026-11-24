function build_tb_perception(which)
%BUILD_TB_PERCEPTION  Build the perception test benches (build task T13).
%
%   build_tb_perception()          build all benches below
%   build_tb_perception(names)     build only the listed benches
%
%   tests/sensors/tb_perception.slx       root inports scan (Bus_LidarScanRaw 0.1), cloud (Bus_LidarCloudRaw 0.1),
%                                         nav_state (Bus_NavState 0.01), Interpolate off -> Model Shaddad_Perception
%                                         -> root outports perception (0.1), scanmatch (0.2). Driven by
%                                         tests/sensors/per_tb_input / per_tb_run.
%   tests/sensors/tb_per_scanfilter.slx   root inports ranges, angles single[360] (0.1) -> Subsystem Reference
%                                         sens_scan_filter -> 6 root outports (bit-exact test).
%   tests/sensors/tb_per_tebvfh.slx       T-PER-05 test model (referenced in Accelerator mode): in 1 perception
%                                         Bus_Perception 0.1, 2 nav_state Bus_NavState 0.01; perception.map drives
%                                         navlib/Control Algorithms/Timed Elastic Band (TEB) (Path Following with
%                                         Obstacle Avoidance, reference path tebRefPath in its model workspace) and
%                                         ranges_B / angles_B drive navlib/Control Algorithms/Vector Field Histogram;
%                                         out 1 TEB velocity commands, 2 VFH steering direction.
%   tests/sensors/tb_per_tebvfh_top.slx   root inports as tb_perception -> Model Shaddad_Perception (Normal) ->
%                                         Model tb_per_tebvfh (Accelerator) -> root outports.
%   All benches use the cs_Shaddad configuration reference (shaddad_configsets('attach')).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out each bench with
%   shaddad_layout(name, 'Recursive', true). Every signal line is named (L5, shaddad_bu('nline')). The root ports stay
%   classic (per_tb_input, per_tb_run and per_sf_run use them by order or name); tb_per_tebvfh takes one Bus Selector per
%   consumer on nav_state and perception (spec 6.2).

names = {'tb_perception', 'tb_per_scanfilter', 'tb_per_tebvfh', 'tb_per_tebvfh_top'};
if nargin < 1 || isempty(which), which = names; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_Perception'',''var'')')
    shaddad_params();
end
root = char(currentProject().RootFolder);
for m = fliplr(names)
    if bdIsLoaded(m{1}), close_system(m{1}, 0); end
end
for k = 1:numel(names)
    if ~any(strcmp(which, names{k})), continue, end
    f = fullfile(root, 'tests', 'sensors', [names{k} '.slx']);
    shaddad_bu('fresh', names{k}, f, 'model');
    shaddad_configsets('attach', names{k});
    feval(['local_' names{k}], names{k});
    shaddad_bu('save', names{k}, f);
    close_system(names{k}, 0);
end
end

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
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line src ('Blk/k'; the
% line named nm when given) and feeds its output k to dsts{k} ('Blk/j')
if nargin < 6, nm = ''; end
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
end
function IN(sys, blk, port, ty, ts, dims)
if nargin < 6, dims = '-1'; end
A(sys, 'simulink/Sources/In1', blk, 'Port', num2str(port), 'OutDataTypeStr', ty, 'SampleTime', ts, ...
  'PortDimensions', dims, 'Interpolate', 'off');
end
function OUT(sys, blk, port)
A(sys, 'simulink/Sinks/Out1', blk, 'Port', num2str(port));
end

function local_inputs(n)
IN(n, 'scan', 1, 'Bus: Bus_LidarScanRaw', 'Ts.lidar');
IN(n, 'cloud', 2, 'Bus: Bus_LidarCloudRaw', 'Ts.lidar');
IN(n, 'nav_state', 3, 'Bus: Bus_NavState', 'Ts.nav');
end

function local_tb_perception(n)
local_inputs(n);
A(n, 'simulink/Ports & Subsystems/Model', 'Perception', 'ModelName', 'Shaddad_Perception', 'SimulationMode', 'Normal');
L(n, 'scan/1', 'Perception/1', 'scan');  L(n, 'cloud/1', 'Perception/2', 'cloud');
L(n, 'nav_state/1', 'Perception/3', 'nav_state');
OUT(n, 'perception', 1);  OUT(n, 'scanmatch', 2);
L(n, 'Perception/1', 'perception/1', 'perception');  L(n, 'Perception/2', 'scanmatch/1', 'scanmatch');
set_param(n, 'Description', ['T13 perception bench: root inputs (Interpolate off, per_tb_input) -> Model ' ...
    'Shaddad_Perception -> root outputs perception (0.1 s) and scanmatch (0.2 s). Run with per_tb_run.']);
end

function local_tb_per_scanfilter(n)
IN(n, 'ranges', 1, 'single', 'Ts.perception', 'P.sensors.lidar.nBeams');
IN(n, 'angles', 2, 'single', 'Ts.perception', 'P.sensors.lidar.nBeams');
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'ScanFilter', 'ReferencedSubsystem', 'sens_scan_filter');
L(n, 'ranges/1', 'ScanFilter/1', 'ranges');  L(n, 'angles/1', 'ScanFilter/2', 'angles');
outs = {'rangesF', 'rangesB', 'anglesB', 'xB', 'yB', 'keep'};
for k = 1:numel(outs)
    OUT(n, outs{k}, k);
    L(n, sprintf('ScanFilter/%d', k), [outs{k} '/1'], outs{k});
end
set_param(n, 'Description', 'T13 bit-exact harness of sens_scan_filter (tests/sensors/sens_scan_filter_ref.m).');
end

function local_tb_per_tebvfh(n)
IN(n, 'perception', 1, 'Bus: Bus_Perception', 'Ts.perception');
IN(n, 'nav_state', 2, 'Bus: Bus_NavState', 'Ts.nav');
for b = {'perception', 'nav_state'}, set_param([n '/' b{1}], 'BusOutputAsStruct', 'on'); end
A(n, 'simulink/Signal Attributes/Rate Transition', 'RT_nav', 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', 'Ts.perception');
L(n, 'nav_state/1', 'RT_nav/1', 'nav_state');
A(n, 'simulink/Signal Routing/Selector', 'XY', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', '[1 2]', 'InputPortWidth', '3');
A(n, 'simulink/Signal Routing/Selector', 'Yaw', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', '1', 'InputPortWidth', '3');
A(n, 'simulink/Signal Routing/Selector', 'V', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', '1', 'InputPortWidth', '3');
A(n, 'simulink/Signal Routing/Selector', 'W', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', '3', 'InputPortWidth', '3');
PICK(n, 'RT_nav/1', 'Nav_pos_W', {'pos_W'}, {'XY/1'}, 'nav_100ms');  PICK(n, 'RT_nav/1', 'Nav_eul', {'eul'}, {'Yaw/1'});
PICK(n, 'RT_nav/1', 'Nav_vel_B', {'vel_B'}, {'V/1'});  PICK(n, 'RT_nav/1', 'Nav_omega_B', {'omega_B'}, {'W/1'});
A(n, 'simulink/Signal Routing/Mux', 'Pose', 'Inputs', '[2 1]');  L(n, 'XY/1', 'Pose/1');  L(n, 'Yaw/1', 'Pose/2');
A(n, 'simulink/Signal Routing/Mux', 'Vel', 'Inputs', '2');  L(n, 'V/1', 'Vel/1');  L(n, 'W/1', 'Vel/2');
% ---- TEB: perception.map (Bus_TebMapInfo) ----
A(n, 'navlib/Control Algorithms/Timed Elastic Band (TEB)', 'TEB', 'SampleTime', 'Ts.perception');
A(n, 'simulink/Sources/Constant', 'RefPath', 'Value', 'tebRefPath', 'SampleTime', 'Ts.perception');
L(n, 'Pose/1', 'TEB/1', 'pose');  L(n, 'Vel/1', 'TEB/2', 'vel');
PICK(n, 'perception/1', 'Per_map', {'map'}, {'TEB/3'}, 'perception');  L(n, 'RefPath/1', 'TEB/4', 'refPath');
OUT(n, 'teb_velcmds', 1);  L(n, 'TEB/1', 'teb_velcmds/1', 'velcmds');
for k = 2:4
    A(n, 'simulink/Sinks/Terminator', sprintf('Term_teb%d', k));
    L(n, sprintf('TEB/%d', k), sprintf('Term_teb%d/1', k));
end
% ---- VFH: ranges_B / angles_B (single -> double) ----
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'RangesD', 'OutDataTypeStr', 'double');
A(n, 'simulink/Signal Attributes/Data Type Conversion', 'AnglesD', 'OutDataTypeStr', 'double');
PICK(n, 'perception/1', 'Per_ranges_B', {'ranges_B'}, {'RangesD/1'});
PICK(n, 'perception/1', 'Per_angles_B', {'angles_B'}, {'AnglesD/1'});
A(n, 'navlib/Control Algorithms/Vector Field Histogram', 'VFH');
A(n, 'simulink/Sources/Constant', 'TargetDir', 'Value', '0', 'SampleTime', 'Ts.perception');
L(n, 'RangesD/1', 'VFH/1');  L(n, 'AnglesD/1', 'VFH/2');  L(n, 'TargetDir/1', 'VFH/3', 'targetDir');
OUT(n, 'vfh_steer', 2);  L(n, 'VFH/1', 'vfh_steer/1', 'steerDir');
mws = get_param(n, 'ModelWorkspace');
assignin(mws, 'tebRefPath', [0 0 0; 5 0 0; 10 0 0; 15 0 0; 20 0 0]);
set_param(n, 'Description', ['T-PER-05 test model (referenced in Accelerator mode by tb_per_tebvfh_top): ' ...
    'Bus_Perception.map -> TEB MapInfo, ranges_B / angles_B -> VFH. tebRefPath (model workspace) = straight path along ' ...
    '+x from the origin.']);
end

function local_tb_per_tebvfh_top(n)
local_inputs(n);
A(n, 'simulink/Ports & Subsystems/Model', 'Perception', 'ModelName', 'Shaddad_Perception', 'SimulationMode', 'Normal');
L(n, 'scan/1', 'Perception/1', 'scan');  L(n, 'cloud/1', 'Perception/2', 'cloud');
L(n, 'nav_state/1', 'Perception/3', 'nav_state');
A(n, 'simulink/Ports & Subsystems/Model', 'TebVfh', 'ModelName', 'tb_per_tebvfh', 'SimulationMode', 'Accelerator');
L(n, 'Perception/1', 'TebVfh/1', 'perception');  L(n, 'nav_state/1', 'TebVfh/2');
A(n, 'simulink/Sinks/Terminator', 'Term_sm');  L(n, 'Perception/2', 'Term_sm/1', 'scanmatch');
OUT(n, 'teb_velcmds', 1);  OUT(n, 'vfh_steer', 2);
L(n, 'TebVfh/1', 'teb_velcmds/1', 'velcmds');  L(n, 'TebVfh/2', 'vfh_steer/1', 'steerDir');
set_param(n, 'Description', 'T-PER-05: Shaddad_Perception (Normal) -> tb_per_tebvfh (Model block, Accelerator).');
end
