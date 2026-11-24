function build_perception(which)
%BUILD_PERCEPTION  Build the companion perception files (build task T13, sensors spec 3.7 / 4.4 / F4, F7-F11).
%
%   build_perception()            build all files below (children first)
%   build_perception(names)       build only the listed files (char or cellstr of the names below)
%
%   Files (programmatic and idempotent, build plan rule 3):
%     subsystems/sensors/sens_scan_filter.slx   Subsystem Reference, blocks only (D59): NaN removal (isNaN +
%                                               Switch), isolated near-return (dust) removal with Selector index
%                                               shifts, self-box mask on x = r cos a, y = r sin a, frame change
%                                               LiDAR -> base_link (hypot / atan2)
%     subsystems/sensors/sens_acq_lidar.slx     Subsystem Reference (SIM acquisition): scan source Variant
%                                               VC_LIDAR {FAST2D pass-through | FULL3D MATLAB Function
%                                               sens_lidar_cloud2scan}
%     models/Shaddad_Perception.slx             Model (cs_Shaddad reference): ICD 3.1 Perception ports; RT 0.01 ->
%                                               0.1 on nav_state; IO Variant VC_TARGET {SIM: sens_acq_lidar | HW:
%                                               ros2lib/Subscribe /ouster/points + ros2lib/Read Point Cloud +
%                                               sens_lidar_cloud2scan} (the ROS 2 blocks sit in the model: their
%                                               message buses live in ros2lib.sldd, which a model only sees through
%                                               ros2lib blocks linked in the model itself, not inside an SSR);
%                                               sector minima / protective field from blocks
%                                               (Reshape + dspstat3/Minimum + Compare); MATLAB Functions
%                                               sens_occmap_update, sens_obstacle_extract (segmentation loop only),
%                                               sens_row_detect; scan matching sens_scanmatch_loc at 0.2 s in an
%                                               Enabled Subsystem (enable = nav_state.lidarLocActive)
%   Needs P, Ts, VC_*, Bus_* in base (shaddad_params). Every number comes from P.sensors / P.world / Ts.
%   MATLAB Function blocks take their constants on inputs (Constant blocks with sens_per_prm(P, ...)), no
%   parameter data (R2026b 'Stateflow:Assertion o != nullptr', see build_sensor_models); the only MATLAB
%   Function below a Subsystem Reference is Cloud2Scan (one SSR level below Shaddad_Perception).
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of each file
%   with shaddad_layout(<file>, 'Recursive', true), the Variant Subsystem levels and their choices included. Every
%   signal line is named (L5, shaddad_bu('nline')). The root ports of the three files stay classic (ICD 3.1, fed by
%   port), and so do the ports of the variant choices, each with one Bus Selector per consumer right in front of it
%   (spec 6.2: Nav_<element> on nav_state, Scan_<element> on the scan, Cloud_<element> on the cloud). Pose picks its
%   elements itself (In Bus Element port nav).

names = {'sens_scan_filter', 'sens_acq_lidar', 'Shaddad_Perception'};
files = {fullfile('subsystems', 'sensors', 'sens_scan_filter.slx'), ...
         fullfile('subsystems', 'sensors', 'sens_acq_lidar.slx'), ...
         fullfile('models', 'Shaddad_Perception.slx')};
kinds = {'ssr', 'ssr', 'model'};
if nargin < 1 || isempty(which), which = names; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_Perception'',''var'')')
    shaddad_params();
end
% a base P assembled before the T13 parameter additions: refresh only P.sensors.per (additive; the shared base
% workspace keeps its world / other domains)
if ~evalin('base', 'isfield(P.sensors.per, ''secIdx'') && isfield(P.sensors.per, ''cloudLevel'')')
    tmp_T13_P = shaddad_params('World', evalin('base', 'P.world.id'), 'Assign', false);
    assignin('base', 'tmp_T13_per', tmp_T13_P.sensors.per);
    evalin('base', 'P.sensors.per = tmp_T13_per; clear tmp_T13_per');
end
root = char(currentProject().RootFolder);
% the perception benches and files (owned by this task) are closed unsaved so they can be replaced; models of
% other tasks are never closed here (build plan rule 1)
for m = [{'tb_per_tebvfh_top', 'tb_per_tebvfh', 'tb_perception', 'tb_per_scanfilter'}, fliplr(names)]
    if bdIsLoaded(m{1})
        try
            close_system(m{1}, 0);
        catch %#ok<CTCH> still referenced by a loaded model of another task
        end
    end
end
for k = 1:numel(names)
    if ~any(strcmp(which, names{k})), continue, end
    f = fullfile(root, files{k});
    shaddad_bu('fresh', names{k}, f, kinds{k});
    if strcmp(kinds{k}, 'model')
        shaddad_configsets('attach', names{k});
    end
    feval(['local_' names{k}], names{k});
    shaddad_bu('save', names{k}, f);
    if strcmp(kinds{k}, 'model')
        shaddad_bu('finalize', names{k}, f);
    end
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
function RT(sys, blk, ts, ic)
p = A(sys, 'simulink/Signal Attributes/Rate Transition', blk, 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
if nargin > 3, set_param(p, 'InitialCondition', ic); end
end
function BS(sys, blk, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
end
function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (spec 6.2): it selects sigs from the bus line src ('Blk/k'; the
% line named nm when given) and feeds its output k to dsts{k} ('Blk/j')
if nargin < 6, nm = ''; end
BS(sys, blk, sigs);
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts), L(sys, sprintf('%s/%d', blk, k), dsts{k}); end
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
function C(sys, blk, val, varargin)
A(sys, 'simulink/Sources/Constant', blk, 'Value', val, varargin{:});
end
function DTC(sys, blk, ty)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', blk, 'OutDataTypeStr', ty, ...
  'RndMeth', 'Nearest', 'SaturateOnIntegerOverflow', 'off');
end
function SEL(sys, blk, idx, w)
A(sys, 'simulink/Signal Routing/Selector', blk, 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'Indices', idx, 'InputPortWidth', w);
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
function CMP(sys, blk, op, val)
% Compare To Constant, boolean output
A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', blk, 'relop', op, 'const', val, ...
  'OutDataTypeStr', 'boolean');
end
function REL(sys, blk, op)
A(sys, 'simulink/Logic and Bit Operations/Relational Operator', blk, 'Operator', op, 'OutDataTypeStr', 'boolean');
end
function LOG(sys, blk, op, n)
A(sys, 'simulink/Logic and Bit Operations/Logical Operator', blk, 'Operator', op, 'Inputs', num2str(n), ...
  'AllPortsSameDT', 'off', 'OutDataTypeStr', 'boolean');
end
function SW(sys, blk)
% Switch: in 1 data if control (in 2) ~= 0, in 3 otherwise
A(sys, 'simulink/Signal Routing/Switch', blk, 'Criteria', 'u2 ~= 0');
end
function TRIG(sys, blk, op)
A(sys, 'simulink/Math Operations/Trigonometric Function', blk, 'Operator', op);
end
function PROD(sys, blk)
A(sys, 'simulink/Math Operations/Product', blk, 'Inputs', '**');
end
function SUM(sys, blk, signs)
A(sys, 'simulink/Math Operations/Sum', blk, 'Inputs', signs);
end
function BIAS(sys, blk, b)
A(sys, 'simulink/Math Operations/Bias', blk, 'Bias', b);
end
function GAIN(sys, blk, g)
A(sys, 'simulink/Math Operations/Gain', blk, 'Gain', g);
end
function ABS(sys, blk)
A(sys, 'simulink/Math Operations/Abs', blk);
end
function mf = MF(sys, blk, script, desc, outTypes)
% MATLAB Function block with its body in a %#codegen .m file (one-line call). No parameter data: every
% constant arrives on an input from a Constant block (rule 2). outTypes = {name, type; ...} sets bus-typed outputs.
mf = A(sys, 'simulink/User-Defined Functions/MATLAB Function', blk);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = script;
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = false;
cfg.Description = desc;
if nargin > 4 && ~isempty(outTypes)
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', mf);
    for k = 1:size(outTypes, 1)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', outTypes{k, 1});
        d.DataType = outTypes{k, 2};
    end
end
end
function SUB(sys, blk)
% empty plain (virtual) Subsystem: the library block's default In1 -> Out1 is removed
p = A(sys, 'simulink/Ports & Subsystems/Subsystem', blk);
CLEAR(p);
end
function CLEAR(p)
lns = find_system(p, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(p, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants), {p});
for k = 1:numel(b), delete_block(b{k}); end
end
function VS(sys, blk, ins, outs, choices)
% Variant Subsystem (expression mode, 'update diagram') with plain choice subsystems {name, condition}
vs = [sys '/' blk];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
CLEAR(vs);
for k = 1:numel(ins), add_block('simulink/Sources/In1', [vs '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [vs '/' outs{k}], 'Port', num2str(k)); end
for k = 1:size(choices, 1)
    c = [vs '/' choices{k, 1}];
    add_block('simulink/Ports & Subsystems/Subsystem', c, 'VariantControl', choices{k, 2});
    CLEAR(c);
    for j = 1:numel(ins), add_block('simulink/Sources/In1', [c '/' ins{j}], 'Port', num2str(j)); end
    for j = 1:numel(outs), add_block('simulink/Sinks/Out1', [c '/' outs{j}], 'Port', num2str(j)); end
end
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

% ============================================================================================
% sens_scan_filter (blocks only, D59)
% ============================================================================================
function local_sens_scan_filter(n)
ts = 'Ts.perception';
nb = 'P.sensors.lidar.nBeams';
IN(n, 'ranges', 1, 'single', ts, nb);
IN(n, 'angles', 2, 'single', ts, nb);
outs = {'rangesF', 'rangesB', 'anglesB', 'xB', 'yB', 'keep'};
tys = {'single', 'single', 'single', 'double', 'double', 'boolean'};
for k = 1:numel(outs), OUT(n, outs{k}, k, tys{k}); end
% ---- NaN removal: r0 = ranges (valid) | Inf (isNaN) ----
REL(n, 'IsNaN', 'isNaN');  L(n, 'ranges/1', 'IsNaN/1', 'ranges');
LOG(n, 'NotNaN', 'NOT', 1);  L(n, 'IsNaN/1', 'NotNaN/1', 'nan');
C(n, 'InfS', 'single(Inf)', 'OutDataTypeStr', 'single');
SW(n, 'R0');  L(n, 'ranges/1', 'R0/1');  L(n, 'NotNaN/1', 'R0/2', 'valid0');  L(n, 'InfS/1', 'R0/3');
% ---- isolated near-return (dust): r0 < dustR and both neighbours differ by > dustDr (circular) ----
SEL(n, 'Prev', ['[' nb ', 1:' nb '-1]'], nb);  L(n, 'R0/1', 'Prev/1', 'r0');
SEL(n, 'Next', ['[2:' nb ', 1]'], nb);         L(n, 'R0/1', 'Next/1');
SUM(n, 'DPrev', '+-');  L(n, 'R0/1', 'DPrev/1');  L(n, 'Prev/1', 'DPrev/2', 'rPrev');
SUM(n, 'DNext', '+-');  L(n, 'R0/1', 'DNext/1');  L(n, 'Next/1', 'DNext/2', 'rNext');
ABS(n, 'AbsPrev');  L(n, 'DPrev/1', 'AbsPrev/1');
ABS(n, 'AbsNext');  L(n, 'DNext/1', 'AbsNext/1');
CMP(n, 'Near', '<', 'single(P.sensors.per.dustR)');      L(n, 'R0/1', 'Near/1');
CMP(n, 'JumpPrev', '>', 'single(P.sensors.per.dustDr)'); L(n, 'AbsPrev/1', 'JumpPrev/1');
CMP(n, 'JumpNext', '>', 'single(P.sensors.per.dustDr)'); L(n, 'AbsNext/1', 'JumpNext/1');
LOG(n, 'Isolated', 'AND', 3);
L(n, 'Near/1', 'Isolated/1');  L(n, 'JumpPrev/1', 'Isolated/2');  L(n, 'JumpNext/1', 'Isolated/3');
% ---- frame change: a = angle + psiL; x = r0 cos a + rLx, y = r0 sin a + rLy (double) ----
DTC(n, 'R0d', 'double');  L(n, 'R0/1', 'R0d/1');
DTC(n, 'Ad', 'double');   L(n, 'angles/1', 'Ad/1', 'angles');
BIAS(n, 'AngMount', 'P.sensors.lidar.eul_deg(1)*pi/180');  L(n, 'Ad/1', 'AngMount/1');
TRIG(n, 'Cos', 'cos');  L(n, 'AngMount/1', 'Cos/1', 'aS');
TRIG(n, 'Sin', 'sin');  L(n, 'AngMount/1', 'Sin/1');
PROD(n, 'Xs');  L(n, 'R0d/1', 'Xs/1');  L(n, 'Cos/1', 'Xs/2');
PROD(n, 'Ys');  L(n, 'R0d/1', 'Ys/1');  L(n, 'Sin/1', 'Ys/2');
BIAS(n, 'XB', 'P.sensors.lidar.r(1)');  L(n, 'Xs/1', 'XB/1');
BIAS(n, 'YB', 'P.sensors.lidar.r(2)');  L(n, 'Ys/1', 'YB/1');
% ---- self-box mask (robot body incl. front implement) ----
sb = 'P.sensors.per.selfBox';
CMP(n, 'BoxX1', '>=', [sb '(1)']);  L(n, 'XB/1', 'BoxX1/1', 'xB_all');
CMP(n, 'BoxX2', '<=', [sb '(2)']);  L(n, 'XB/1', 'BoxX2/1');
CMP(n, 'BoxY1', '>=', [sb '(3)']);  L(n, 'YB/1', 'BoxY1/1', 'yB_all');
CMP(n, 'BoxY2', '<=', [sb '(4)']);  L(n, 'YB/1', 'BoxY2/1');
LOG(n, 'InBox', 'AND', 4);
L(n, 'BoxX1/1', 'InBox/1');  L(n, 'BoxX2/1', 'InBox/2');  L(n, 'BoxY1/1', 'InBox/3');  L(n, 'BoxY2/1', 'InBox/4');
% ---- keep = ~isNaN & ~selfBox & ~dust ----
LOG(n, 'NotBox', 'NOT', 1);   L(n, 'InBox/1', 'NotBox/1', 'inBox');
LOG(n, 'NotDust', 'NOT', 1);  L(n, 'Isolated/1', 'NotDust/1', 'isolated');
LOG(n, 'Keep', 'AND', 3);
L(n, 'NotNaN/1', 'Keep/1');  L(n, 'NotBox/1', 'Keep/2');  L(n, 'NotDust/1', 'Keep/3');
L(n, 'Keep/1', 'keep/1', 'keep');
C(n, 'NaNS', 'single(NaN)', 'OutDataTypeStr', 'single');
C(n, 'NaND', 'NaN', 'OutDataTypeStr', 'double');
SW(n, 'RF');  L(n, 'ranges/1', 'RF/1');  L(n, 'Keep/1', 'RF/2');  L(n, 'NaNS/1', 'RF/3');
L(n, 'RF/1', 'rangesF/1', 'rangesF');
A(n, 'simulink/Math Operations/Math Function', 'Hypot', 'Operator', 'hypot');
L(n, 'XB/1', 'Hypot/1');  L(n, 'YB/1', 'Hypot/2');
DTC(n, 'RBs', 'single');  L(n, 'Hypot/1', 'RBs/1', 'rB');
SW(n, 'RB');  L(n, 'RBs/1', 'RB/1');  L(n, 'Keep/1', 'RB/2');  L(n, 'NaNS/1', 'RB/3');
L(n, 'RB/1', 'rangesB/1', 'rangesB');
TRIG(n, 'Atan2', 'atan2');  L(n, 'YB/1', 'Atan2/1');  L(n, 'XB/1', 'Atan2/2');
DTC(n, 'ABs', 'single');  L(n, 'Atan2/1', 'ABs/1', 'aB');
DTC(n, 'ASs', 'single');  L(n, 'AngMount/1', 'ASs/1');
SW(n, 'AB');  L(n, 'ABs/1', 'AB/1');  L(n, 'Keep/1', 'AB/2');  L(n, 'ASs/1', 'AB/3');
L(n, 'AB/1', 'anglesB/1', 'anglesB');
SW(n, 'XBk');  L(n, 'XB/1', 'XBk/1');  L(n, 'Keep/1', 'XBk/2');  L(n, 'NaND/1', 'XBk/3');
SW(n, 'YBk');  L(n, 'YB/1', 'YBk/1');  L(n, 'Keep/1', 'YBk/2');  L(n, 'NaND/1', 'YBk/3');
L(n, 'XBk/1', 'xB/1', 'xB');  L(n, 'YBk/1', 'yB/1', 'yB');
set_param(n, 'Description', sprintf([ ...
  'sens_scan_filter (T13, sensors spec 3.7 / F7, D59; blocks only). In: 1 ranges single[360] (LiDAR frame, NaN = no ' ...
  'return), 2 angles single[360] (beam k at -pi + (k-1) deg), Ts.perception. Out: 1 rangesF single[360] filtered, ' ...
  'LiDAR frame; 2 rangesB single[360] about base_link (Bus_Perception.ranges_B); 3 anglesB single[360] ' ...
  '(angles_B; sensor angle + mount yaw where removed); 4 xB, 5 yB double[360] kept points in base_link (NaN ' ...
  'otherwise); 6 keep boolean[360].\n' ...
  'NaN removal: isNaN Relational Operator + Switch (NaN -> Inf); dust: r0 < P.sensors.per.dustR and both neighbours ' ...
  '(Selector index shifts, circular) differ by > dustDr; self box P.sensors.per.selfBox on x = r cos a + rLx, ' ...
  'y = r sin a + rLy (Trigonometric Function, Product, Bias, Compare To Constant); frame change with hypot / ' ...
  'atan2. Reference implementation: tests/sensors/sens_scan_filter_ref.m (bit-exact test in t_perception).']));
end

% ============================================================================================
% sens_acq_lidar (companion SIM LiDAR acquisition: scan source Variant VC_LIDAR)
% ============================================================================================
function local_sens_acq_lidar(n)
ts = 'Ts.lidar';
IN(n, 'scan', 1, 'Bus: Bus_LidarScanRaw', ts);
IN(n, 'cloud', 2, 'Bus: Bus_LidarCloudRaw', ts);
IN(n, 'rp', 3, 'double', ts, '2');
OUT(n, 'scan_out', 1, 'Bus: Bus_LidarScanRaw');
ins = {'scan', 'cloud', 'rp'};
VS(n, 'ScanSource', ins, {'scan_out'}, {'FAST2D', 'VC_LIDAR == 1'; 'FULL3D', 'VC_LIDAR == 2'});
for k = 1:3, L(n, [ins{k} '/1'], sprintf('ScanSource/%d', k), ins{k}); end
L(n, 'ScanSource/1', 'scan_out/1', 'scan');
f2 = [n '/ScanSource/FAST2D'];
L(f2, 'scan/1', 'scan_out/1', 'scan_fast2d');
TERM(f2, 'cloud/1', 'Term_cloud');  TERM(f2, 'rp/1', 'Term_rp');
set_param(f2, 'Description', 'FAST2D (VC_LIDAR == 1): the plant 360-beam scan is passed through.');
f3 = [n '/ScanSource/FULL3D'];
for c = {'t_scan', 'xyz', 'nAz', 'valid'}
    BS(f3, ['Cloud_' c{1}], c);  L(f3, 'cloud/1', ['Cloud_' c{1} '/1'], 'cloud');
end
local_cloud2scan(f3, 'Cloud_xyz/1', 'Cloud_nAz/1', 'rp/1', 'Cloud_valid/1', 'Cloud_t_scan/1');
TERM(f3, 'scan/1', 'Term_scan');
set_param(f3, 'Description', ['FULL3D (VC_LIDAR == 2): organised cloud (Bus_LidarCloudRaw) -> MATLAB Function ' ...
    'sens_lidar_cloud2scan (F4) -> Bus_LidarScanRaw.']);
set_param(n, 'Description', sprintf([ ...
  'sens_acq_lidar (T13, companion SIM LiDAR acquisition, sensors spec 4.4 C2; SIM choice of Shaddad_Perception/IO_In). ' ...
  'In: 1 scan Bus_LidarScanRaw 0.1, 2 cloud Bus_LidarCloudRaw 0.1, 3 rp double[2] [roll pitch] of base_link 0.1. ' ...
  'Out: 1 scan_out Bus_LidarScanRaw 0.1 (360 beams, LiDAR frame, NaN = no return).\n' ...
  'ScanSource Variant VC_LIDAR (update diagram): 1 FAST2D pass-through of the plant scan, 2 FULL3D MATLAB Function ' ...
  'sens_lidar_cloud2scan on the organised cloud. The HW choice (ros2lib/Subscribe /ouster/points + ros2lib/Read ' ...
  'Point Cloud) is the sibling choice of Shaddad_Perception/IO_In: ROS 2 message buses live in ros2lib.sldd, which a ' ...
  'model sees only through ros2lib blocks linked in the model itself, not inside a Subsystem Reference (R2026b).']));
end

function local_acq_hw(h)
% HW choice of Shaddad_Perception/IO_In (VC_TARGET == 1): ROS 2 /ouster/points
ts = 'Ts.lidar';
A(h, 'ros2lib/Subscribe', 'Sub_Points', 'topicSource', 'Specify your own', 'topic', '/ouster/points', ...
  'messageType', 'sensor_msgs/PointCloud2', 'sampleTime', ts);
A(h, 'ros2lib/Read Point Cloud', 'ReadCloud', 'MaximumPointCloudSize', 'P.sensors.lidar.cloudSize', ...
  'PreserveStructure', 'on', 'ShowErrorCodeOutput', 'on', 'VariableSizeOutputs', 'off');
L(h, 'Sub_Points/2', 'ReadCloud/1', 'msg');
% scan time = ROS time at reception (ros2lib/Current Time, double s)
A(h, 'ros2lib/Current Time', 'TStamp', 'OutputFormat', 'double', 'SampleTime', ts);
CMP(h, 'ErrOK', '==', '0');  L(h, 'ReadCloud/2', 'ErrOK/1', 'errorCode');
LOG(h, 'CloudValid', 'AND', 2);  L(h, 'Sub_Points/1', 'CloudValid/1', 'isNew');  L(h, 'ErrOK/1', 'CloudValid/2');
C(h, 'NAzHW', 'P.sensors.per.hwNAz');
local_cloud2scan(h, 'ReadCloud/1', 'NAzHW/1', 'rp/1', 'CloudValid/1', 'TStamp/1');
TERM(h, 'scan/1', 'Term_scan');  TERM(h, 'cloud/1', 'Term_cloud');
set_param(h, 'Description', ['HW (VC_TARGET == 1): ros2lib/Subscribe /ouster/points (sensor_msgs/PointCloud2, 0.1 s) -> ' ...
    'ros2lib/Read Point Cloud [32 1024] organised -> MATLAB Function sens_lidar_cloud2scan -> Bus_LidarScanRaw ' ...
    '(t_scan = ROS time at reception, ros2lib/Current Time).']);
end


function local_cloud2scan(s, srcXyz, srcNAz, srcRp, srcValid, srcT)
MF(s, 'Cloud2Scan', sprintf(['function [ranges, valid] = fcn(xyz, nAz, rp, cloudValid, c)\n' ...
    '%%#codegen\n' ...
    '[ranges, valid] = sens_lidar_cloud2scan(xyz, nAz, rp, cloudValid, c);\n']), ...
    ['Cloud -> 360-beam height-band scan (sensors spec 3.4 / F4): subsystems/sensors/sens_lidar_cloud2scan.m; ' ...
     'constants on input 5 (sens_per_prm(P, ''c2s''))']);
C(s, 'C2sPrm', 'sens_per_prm(P, ''c2s'')');
L(s, srcXyz, 'Cloud2Scan/1', 'xyz');  L(s, srcNAz, 'Cloud2Scan/2', 'nAz');  L(s, srcRp, 'Cloud2Scan/3', 'rp');
L(s, srcValid, 'Cloud2Scan/4', 'cloudValid');  L(s, 'C2sPrm/1', 'Cloud2Scan/5', 'c');
C(s, 'Angles', 'single(P.sensors.lidar.angles)', 'OutDataTypeStr', 'single', 'SampleTime', 'Ts.lidar');
BC(s, 'BC_Scan', 'Bus_LidarScanRaw', {srcT, 'Cloud2Scan/1', 'Angles/1', 'Cloud2Scan/2'});
L(s, 'BC_Scan/1', 'scan_out/1', 'scan');
end

% ============================================================================================
% Shaddad_Perception (model)
% ============================================================================================
function local_Shaddad_Perception(n)
ts = 'Ts.perception';
IN(n, 'scan', 1, 'Bus: Bus_LidarScanRaw', 'Ts.lidar');
IN(n, 'cloud', 2, 'Bus: Bus_LidarCloudRaw', 'Ts.lidar');
IN(n, 'nav_state', 3, 'Bus: Bus_NavState', 'Ts.nav');
for b = {'scan', 'cloud', 'nav_state'}, set_param([n '/' b{1}], 'BusOutputAsStruct', 'on'); end
OUT(n, 'perception', 1, 'Bus: Bus_Perception');
OUT(n, 'scanmatch', 2, 'Bus: Bus_ScanMatchMeas');
set_param([n '/perception'], 'SampleTime', ts, 'BusOutputAsStruct', 'on');
set_param([n '/scanmatch'], 'SampleTime', 'Ts.scanmatch', 'BusOutputAsStruct', 'on');

% ---- nav_state 0.01 -> 0.1: the bus line nav_100ms, one Bus Selector per consumer (Pose picks its own elements) ----
RT(n, 'RT_nav', ts);  L(n, 'nav_state/1', 'RT_nav/1', 'nav_state');
SEL(n, 'RP', '[3 2]', '3');  PICK(n, 'RT_nav/1', 'Nav_eul', {'eul'}, {'RP/1'}, 'nav_100ms');
SEL(n, 'VFwd', '1', '3');    PICK(n, 'RT_nav/1', 'Nav_vel_B', {'vel_B'}, {'VFwd/1'});

% ---- LiDAR acquisition: IO Variant VC_TARGET {SIM: Subsystem Reference sens_acq_lidar (scan source Variant
% VC_LIDAR) | HW: ROS 2 /ouster/points}; the ROS 2 blocks sit in the model itself (ros2lib.sldd buses) ----
ains = {'scan', 'cloud', 'rp'};
VS(n, 'IO_In', ains, {'scan_out'}, {'SIM', 'VC_TARGET == 0'; 'HW', 'VC_TARGET == 1'});
sim0 = [n '/IO_In/SIM'];
A(sim0, 'simulink/Ports & Subsystems/Subsystem Reference', 'Acq', 'ReferencedSubsystem', 'sens_acq_lidar');
for k = 1:3, L(sim0, [ains{k} '/1'], sprintf('Acq/%d', k), ains{k}); end
L(sim0, 'Acq/1', 'scan_out/1', 'scan');
set_param(sim0, 'Description', 'SIM (VC_TARGET == 0): Subsystem Reference sens_acq_lidar (scan source Variant VC_LIDAR).');
local_acq_hw([n '/IO_In/HW']);
L(n, 'scan/1', 'IO_In/1', 'scan');  L(n, 'cloud/1', 'IO_In/2', 'cloud');  L(n, 'RP/1', 'IO_In/3', 'rp');

% ---- scan filter (Subsystem Reference, blocks); the scan bus line scan2d, one Bus Selector per consumer ----
A(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'ScanFilter', 'ReferencedSubsystem', 'sens_scan_filter');
PICK(n, 'IO_In/1', 'Scan_ranges', {'ranges'}, {'ScanFilter/1'}, 'scan2d');
PICK(n, 'IO_In/1', 'Scan_angles', {'angles'}, {'ScanFilter/2'});

% ---- poses: LiDAR pose in W (occupancy map) and base_link pose (scan matching) ----
SUB(n, 'Pose');  local_pose([n '/Pose']);
L(n, 'RT_nav/1', 'Pose/1');

% ---- sector minima and protective field (blocks) ----
SUB(n, 'Sectors');  local_sectors([n '/Sectors']);
L(n, 'ScanFilter/2', 'Sectors/1', 'rangesB');
SUB(n, 'ProtField');  local_protfield([n '/ProtField']);
L(n, 'ScanFilter/4', 'ProtField/1', 'xB');  L(n, 'ScanFilter/5', 'ProtField/2', 'yB');  L(n, 'VFwd/1', 'ProtField/3', 'v');
PICK(n, 'IO_In/1', 'Scan_valid_3', {'valid'}, {'ProtField/4'});

% ---- rolling occupancy map (F8) ----
MF(n, 'OccMap', sprintf(['function map = fcn(poseL, ranges, rangesF, angles, valid, c)\n' ...
    '%%#codegen\n' ...
    'map = sens_occmap_update(poseL, ranges, rangesF, angles, valid, c);\n']), ...
    ['Rolling 30 x 30 m occupancy grid in the TEB MapInfo format (sensors spec 3.7 / F8): ' ...
     'subsystems/sensors/sens_occmap_update.m; constants on input 6 (sens_per_prm(P, ''occ''))'], ...
    {'map', 'Bus: Bus_TebMapInfo'});
C(n, 'OccPrm', 'sens_per_prm(P, ''occ'')');
L(n, 'Pose/1', 'OccMap/1', 'poseL');  PICK(n, 'IO_In/1', 'Scan_ranges_2', {'ranges'}, {'OccMap/2'});
L(n, 'ScanFilter/1', 'OccMap/3', 'rangesF');  PICK(n, 'IO_In/1', 'Scan_angles_2', {'angles'}, {'OccMap/4'});
PICK(n, 'IO_In/1', 'Scan_valid', {'valid'}, {'OccMap/5'});  L(n, 'OccPrm/1', 'OccMap/6', 'c');

% ---- obstacles: segmentation loop (F9) ----
MF(n, 'Obstacles', sprintf(['function [obsCount, obsPos, obsRadius, obsRange, cl, nCl] = fcn(rangesF, angles, xB, yB, c)\n' ...
    '%%#codegen\n' ...
    '[obsCount, obsPos, obsRadius, obsRange, cl, nCl] = sens_obstacle_extract(rangesF, angles, xB, yB, c);\n']), ...
    ['Obstacle segmentation loop (sensors spec 3.7 / F9, D59): subsystems/sensors/sens_obstacle_extract.m; ' ...
     'constants on input 5 (sens_per_prm(P, ''obs''))']);
C(n, 'ObsPrm', 'sens_per_prm(P, ''obs'')');
L(n, 'ScanFilter/1', 'Obstacles/1');  PICK(n, 'IO_In/1', 'Scan_angles_3', {'angles'}, {'Obstacles/2'});
L(n, 'ScanFilter/4', 'Obstacles/3');  L(n, 'ScanFilter/5', 'Obstacles/4');  L(n, 'ObsPrm/1', 'Obstacles/5', 'c');

% ---- row detection (F10) ----
MF(n, 'RowDetect', sprintf(['function row = fcn(cl, nCl, c)\n' ...
    '%%#codegen\n' ...
    'row = sens_row_detect(cl, nCl, c);\n']), ...
    ['Row detection, RANSAC line per side (sensors spec 3.7 / F10): subsystems/sensors/sens_row_detect.m; ' ...
     'constants on input 3 (sens_per_prm(P, ''row''))'], {'row', 'Bus: Bus_RowFeature'});
C(n, 'RowPrm', 'sens_per_prm(P, ''row'')');
L(n, 'Obstacles/5', 'RowDetect/1', 'clusters');  L(n, 'Obstacles/6', 'RowDetect/2', 'nCl');
L(n, 'RowPrm/1', 'RowDetect/3', 'c');

% ---- scan-to-map localisation at 0.2 s while lidarLocActive (F11, D55) ----
RT(n, 'RT_smPose', 'Ts.scanmatch');    L(n, 'Pose/2', 'RT_smPose/1', 'poseNav');
RT(n, 'RT_smRanges', 'Ts.scanmatch');  L(n, 'ScanFilter/1', 'RT_smRanges/1');
RT(n, 'RT_smAngles', 'Ts.scanmatch');  PICK(n, 'IO_In/1', 'Scan_angles_4', {'angles'}, {'RT_smAngles/1'});
RT(n, 'RT_smT', 'Ts.scanmatch');       PICK(n, 'IO_In/1', 'Scan_t_scan', {'t_scan'}, {'RT_smT/1'});
RT(n, 'RT_smOn', 'Ts.scanmatch');      PICK(n, 'RT_nav/1', 'Nav_lidarLocActive', {'lidarLocActive'}, {'RT_smOn/1'});
sm = [n '/ScanMatch'];
A(n, 'simulink/Ports & Subsystems/Enabled Subsystem', 'ScanMatch');
CLEAR(sm);
smIn = {'poseNav', 'ranges', 'angles', 'tScan'};
for k = 1:numel(smIn), A(sm, 'simulink/Sources/In1', smIn{k}, 'Port', num2str(k)); end
A(sm, 'simulink/Ports & Subsystems/Enable', 'Enable', 'StatesWhenEnabling', 'held');
A(sm, 'simulink/Sinks/Out1', 'sm', 'Port', '1', 'OutputWhenDisabled', 'reset', ...
  'InitialOutput', 'P.ic.Bus_ScanMatchMeas', 'OutDataTypeStr', 'Bus: Bus_ScanMatchMeas');
MF(sm, 'ScanMatchLoc', sprintf(['function sm = fcn(poseNav, ranges, angles, tScan, grid, c)\n' ...
    '%%#codegen\n' ...
    'sm = sens_scanmatch_loc(poseNav, ranges, angles, tScan, grid, c);\n']), ...
    ['Scan-to-prior-map localisation with matchScansGrid (sensors spec 3.7 / F11, D55): ' ...
     'subsystems/sensors/sens_scanmatch_loc.m; prior map on input 5 (sens_per_prior_map(P.world, P.sensors.per)), ' ...
     'constants on input 6 (sens_per_prm(P, ''sm''))'], {'sm', 'Bus: Bus_ScanMatchMeas'});
C(sm, 'PriorMap', 'sens_per_prior_map(P.world, P.sensors.per)', 'OutDataTypeStr', 'boolean', 'VectorParams1D', 'off');
C(sm, 'SmPrm', 'sens_per_prm(P, ''sm'')');
for k = 1:numel(smIn), L(sm, [smIn{k} '/1'], sprintf('ScanMatchLoc/%d', k), smIn{k}); end
L(sm, 'PriorMap/1', 'ScanMatchLoc/5', 'grid');  L(sm, 'SmPrm/1', 'ScanMatchLoc/6', 'c');
L(sm, 'ScanMatchLoc/1', 'sm/1', 'sm');
set_param(sm, 'Description', ['Enabled at 0.2 s while nav_state.lidarLocActive (D55); output reset to ' ...
    'P.ic.Bus_ScanMatchMeas (valid false) while disabled.']);
L(n, 'RT_smPose/1', 'ScanMatch/1', 'poseNav_200ms');  L(n, 'RT_smRanges/1', 'ScanMatch/2', 'rangesF_200ms');
L(n, 'RT_smAngles/1', 'ScanMatch/3', 'angles_200ms');  L(n, 'RT_smT/1', 'ScanMatch/4', 'tScan_200ms');
L(n, 'RT_smOn/1', 'ScanMatch/Enable', 'smOn');
L(n, 'ScanMatch/1', 'scanmatch/1', 'scanmatch');
RT(n, 'RT_smHold', ts, 'P.ic.Bus_ScanMatchMeas');  L(n, 'ScanMatch/1', 'RT_smHold/1');

% ---- Bus_Perception ----
BS(n, 'Scan_t_scan_2', {'t_scan'});  L(n, 'IO_In/1', 'Scan_t_scan_2/1');
BS(n, 'Scan_valid_2', {'valid'});    L(n, 'IO_In/1', 'Scan_valid_2/1');
BC(n, 'BC_Perception', 'Bus_Perception', {'Scan_t_scan_2/1', 'Scan_valid_2/1', 'ScanFilter/2', 'ScanFilter/3', 'Sectors/1', ...
    'ProtField/1', 'ProtField/2', 'Obstacles/1', 'Obstacles/2', 'Obstacles/3', 'Obstacles/4', 'RowDetect/1', ...
    'OccMap/1', 'RT_smHold/1'});
L(n, 'BC_Perception/1', 'perception/1', 'perception');
TERM(n, 'ScanFilter/6', 'Term_keep', 'keep');
set_param(n, 'Description', sprintf([ ...
  'Shaddad_Perception (T13, sensors spec 3.7 / 4.4, ICD 3.1 Perception row). In: 1 scan Bus_LidarScanRaw 0.1, ' ...
  '2 cloud Bus_LidarCloudRaw 0.1, 3 nav_state Bus_NavState 0.01 (delayed 10 ms in Shaddad_Companion). Out: ' ...
  '1 perception Bus_Perception 0.1, 2 scanmatch Bus_ScanMatchMeas 0.2.\n' ...
  'RT_nav 0.01 -> 0.1; IO_In Variant VC_TARGET (SIM = Subsystem Reference sens_acq_lidar with the scan source Variant ' ...
  'VC_LIDAR; HW = ros2lib/Subscribe /ouster/points + Read Point Cloud + sens_lidar_cloud2scan); ' ...
  'ScanFilter = Subsystem Reference sens_scan_filter (blocks, D59); Sectors (Selector + Reshape + dspstat3/Minimum) ' ...
  'and ProtField (Compare / Logical Operator, length P.sensors.per.protL0 + protKv |v|, width protW, front field from ' ...
  'protXF (body or implement front) for v > -protVBoth, rear field from protXR for v < protVBoth; stop also on a ' ...
  'scan lost or stale for more than protNStale scans and until the first valid scan) from blocks; MATLAB Functions ' ...
  'OccMap (sens_occmap_update, TEB MapInfo field order), Obstacles ' ...
  '(sens_obstacle_extract, segmentation loop only), RowDetect (sens_row_detect); ScanMatch = Enabled Subsystem at ' ...
  '0.2 s (enable nav_state.lidarLocActive) with sens_scanmatch_loc on the prior map sens_per_prior_map(P.world, ' ...
  'P.sensors.per) (P.world.occ geometry + priorMapErr as-built perturbation, built at initialisation); ' ...
  'perception.scanMatch = latest result held (RT 0.2 -> 0.1). NaN = no return.']));
end

function local_pose(s)
% in nav (In Bus Element: pos_W double[3], eul double[3]); out 1 poseL [x y psi] of the LiDAR in W, 2 poseNav [x y psi] of base_link
A(s, 'simulink/Sinks/Out1', 'poseL', 'Port', '1');
A(s, 'simulink/Sinks/Out1', 'poseNav', 'Port', '2');
SEL(s, 'X', '1', '3');  SEL(s, 'Y', '2', '3');  SEL(s, 'Psi', '1', '3');
L(s, 'nav.pos_W', 'X/1');  L(s, 'nav.pos_W', 'Y/1');  L(s, 'nav.eul', 'Psi/1');
TRIG(s, 'Cos', 'cos');  TRIG(s, 'Sin', 'sin');
L(s, 'Psi/1', 'Cos/1', 'psi');  L(s, 'Psi/1', 'Sin/1');
GAIN(s, 'rLxC', 'P.sensors.lidar.r(1)');  GAIN(s, 'rLyS', 'P.sensors.lidar.r(2)');
GAIN(s, 'rLxS', 'P.sensors.lidar.r(1)');  GAIN(s, 'rLyC', 'P.sensors.lidar.r(2)');
L(s, 'Cos/1', 'rLxC/1');  L(s, 'Sin/1', 'rLyS/1');  L(s, 'Sin/1', 'rLxS/1');  L(s, 'Cos/1', 'rLyC/1');
SUM(s, 'XL', '++-');  L(s, 'X/1', 'XL/1');  L(s, 'rLxC/1', 'XL/2');  L(s, 'rLyS/1', 'XL/3');
SUM(s, 'YL', '+++');  L(s, 'Y/1', 'YL/1');  L(s, 'rLxS/1', 'YL/2');  L(s, 'rLyC/1', 'YL/3');
BIAS(s, 'PsiL', 'P.sensors.lidar.eul_deg(1)*pi/180');  L(s, 'Psi/1', 'PsiL/1');
A(s, 'simulink/Signal Routing/Mux', 'MuxL', 'Inputs', '3');
L(s, 'XL/1', 'MuxL/1', 'xL');  L(s, 'YL/1', 'MuxL/2', 'yL');  L(s, 'PsiL/1', 'MuxL/3', 'psiL');
L(s, 'MuxL/1', 'poseL/1', 'poseL');
A(s, 'simulink/Signal Routing/Mux', 'MuxB', 'Inputs', '3');
L(s, 'X/1', 'MuxB/1');  L(s, 'Y/1', 'MuxB/2');  L(s, 'Psi/1', 'MuxB/3');
L(s, 'MuxB/1', 'poseNav/1', 'poseNav');
set_param(s, 'Description', 'poseL = [x + rLx cos psi - rLy sin psi, y + rLx sin psi + rLy cos psi, psi + psiL]; poseNav = [x y psi].');
end

function local_sectors(s)
% in 1 rangesB single[360]; out 1 minSectorRange single[8] (sector 1 centred forward, CCW; NaN = no return)
A(s, 'simulink/Sources/In1', 'rangesB', 'Port', '1');
A(s, 'simulink/Sinks/Out1', 'minSectorRange', 'Port', '1');
REL(s, 'IsNaN', 'isNaN');  L(s, 'rangesB/1', 'IsNaN/1', 'rangesB');
C(s, 'InfS', 'single(Inf)', 'OutDataTypeStr', 'single');
SW(s, 'NaNToInf');  L(s, 'InfS/1', 'NaNToInf/1');  L(s, 'IsNaN/1', 'NaNToInf/2', 'nan');  L(s, 'rangesB/1', 'NaNToInf/3');
SEL(s, 'SectorOrder', 'P.sensors.per.secIdx', 'P.sensors.lidar.nBeams');  L(s, 'NaNToInf/1', 'SectorOrder/1');
A(s, 'simulink/Math Operations/Reshape', 'ToSectors', 'OutputDimensionality', 'Customize', ...
  'OutputDimensions', '[P.sensors.per.secLen P.sensors.per.nSectors]');
L(s, 'SectorOrder/1', 'ToSectors/1');
A(s, 'dspstat3/Minimum', 'SectorMin', 'fcn', 'Value', 'operateOver', 'Each column');
L(s, 'ToSectors/1', 'SectorMin/1', 'sectors45x8');
A(s, 'simulink/Math Operations/Reshape', 'To1D', 'OutputDimensionality', '1-D array');
L(s, 'SectorMin/1', 'To1D/1');
REL(s, 'IsInf', 'isInf');  L(s, 'To1D/1', 'IsInf/1', 'secMin');
C(s, 'NaNS', 'single(NaN)', 'OutDataTypeStr', 'single');
SW(s, 'InfToNaN');  L(s, 'NaNS/1', 'InfToNaN/1');  L(s, 'IsInf/1', 'InfToNaN/2');  L(s, 'To1D/1', 'InfToNaN/3');
L(s, 'InfToNaN/1', 'minSectorRange/1', 'minSectorRange');
set_param(s, 'Description', ['8 x 45 deg sector minima of ranges_B (sector j covers sensor beams -22 + 45 (j-1) ... ' ...
    '+22 + 45 (j-1) deg, CCW): NaN -> Inf, Selector P.sensors.per.secIdx, Reshape [45 8], dspstat3/Minimum per ' ...
    'column, empty sector -> NaN.']);
end

function local_protfield(s)
% in 1 xB, 2 yB double[360] (base_link, NaN = none), 3 v forward speed, 4 valid scan valid; out 1 protectiveStop,
% 2 protFieldLen single
A(s, 'simulink/Sources/In1', 'xB', 'Port', '1');
A(s, 'simulink/Sources/In1', 'yB', 'Port', '2');
A(s, 'simulink/Sources/In1', 'v', 'Port', '3');
A(s, 'simulink/Sources/In1', 'valid', 'Port', '4');
A(s, 'simulink/Sinks/Out1', 'protectiveStop', 'Port', '1');
A(s, 'simulink/Sinks/Out1', 'protFieldLen', 'Port', '2');
ABS(s, 'AbsV');  L(s, 'v/1', 'AbsV/1', 'v');
GAIN(s, 'Kv', 'P.sensors.per.protKv');  L(s, 'AbsV/1', 'Kv/1');
BIAS(s, 'Len', 'P.sensors.per.protL0');  L(s, 'Kv/1', 'Len/1');
DTC(s, 'LenS', 'single');  L(s, 'Len/1', 'LenS/1', 'len');
L(s, 'LenS/1', 'protFieldLen/1', 'protFieldLen');
ABS(s, 'AbsY');  L(s, 'yB/1', 'AbsY/1', 'yB');
CMP(s, 'InWidth', '<=', 'P.sensors.per.protW/2');  L(s, 'AbsY/1', 'InWidth/1');
% front field: xF <= x <= xF + L
CMP(s, 'AfterFront', '>=', 'P.sensors.per.protXF');  L(s, 'xB/1', 'AfterFront/1', 'xB');
BIAS(s, 'FrontEnd', 'P.sensors.per.protXF');  L(s, 'Len/1', 'FrontEnd/1');
REL(s, 'BeforeFrontEnd', '<=');  L(s, 'xB/1', 'BeforeFrontEnd/1');  L(s, 'FrontEnd/1', 'BeforeFrontEnd/2');
LOG(s, 'InFront', 'AND', 3);
L(s, 'AfterFront/1', 'InFront/1');  L(s, 'BeforeFrontEnd/1', 'InFront/2');  L(s, 'InWidth/1', 'InFront/3');
LOG(s, 'AnyFront', 'OR', 1);  L(s, 'InFront/1', 'AnyFront/1');
% rear field (reversing and standstill): xR - L <= x <= xR
CMP(s, 'BeforeRear', '<=', 'P.sensors.per.protXR');  L(s, 'xB/1', 'BeforeRear/1');
GAIN(s, 'NegLen', '-1');  L(s, 'Len/1', 'NegLen/1');
BIAS(s, 'RearEnd', 'P.sensors.per.protXR');  L(s, 'NegLen/1', 'RearEnd/1');
REL(s, 'AfterRearEnd', '>=');  L(s, 'xB/1', 'AfterRearEnd/1');  L(s, 'RearEnd/1', 'AfterRearEnd/2');
LOG(s, 'InRear', 'AND', 3);
L(s, 'BeforeRear/1', 'InRear/1');  L(s, 'AfterRearEnd/1', 'InRear/2');  L(s, 'InWidth/1', 'InRear/3');
LOG(s, 'AnyRear', 'OR', 1);  L(s, 'InRear/1', 'AnyRear/1');
% front field on for v > -protVBoth, rear field for v < protVBoth: both while |v| < protVBoth (standstill, pivots;
% the sign of the EKF speed is noise there)
CMP(s, 'FrontOn', '>', '-P.sensors.per.protVBoth');  L(s, 'v/1', 'FrontOn/1');
CMP(s, 'RearOn', '<', 'P.sensors.per.protVBoth');    L(s, 'v/1', 'RearOn/1');
LOG(s, 'StopFront', 'AND', 2);  L(s, 'AnyFront/1', 'StopFront/1', 'inFront');  L(s, 'FrontOn/1', 'StopFront/2', 'frontOn');
LOG(s, 'StopRear', 'AND', 2);   L(s, 'AnyRear/1', 'StopRear/1', 'inRear');     L(s, 'RearOn/1', 'StopRear/2', 'rearOn');
LOG(s, 'FieldStop', 'OR', 2);   L(s, 'StopFront/1', 'FieldStop/1', 'stopFront');  L(s, 'StopRear/1', 'FieldStop/2', 'stopRear');
% scan lost or stale (critfix R-02, NAV-01): age = invalid scans since the last valid one (0 on a valid scan, saturated
% at protNStale + 1); the field result of the last valid scan is held while the age is <= protNStale, then stop.
% Both delays start stale / stopped, so the field stops from the start until the first valid scan.
A(s, 'simulink/Discrete/Unit Delay', 'AgePrev', 'InitialCondition', 'P.sensors.per.protNStale + 1', ...
  'SampleTime', 'Ts.perception');
BIAS(s, 'AgeInc', '1');  L(s, 'AgePrev/1', 'AgeInc/1', 'agePrev');
A(s, 'simulink/Discontinuities/Saturation', 'AgeSat', 'UpperLimit', 'P.sensors.per.protNStale + 1', 'LowerLimit', '0');
L(s, 'AgeInc/1', 'AgeSat/1');
C(s, 'AgeZero', '0');
A(s, 'simulink/Signal Routing/Switch', 'Age', 'Criteria', 'u2 ~= 0', 'OutDataTypeStr', 'double');
L(s, 'AgeZero/1', 'Age/1');  L(s, 'valid/1', 'Age/2', 'valid');  L(s, 'AgeSat/1', 'Age/3');
L(s, 'Age/1', 'AgePrev/1', 'age');
CMP(s, 'Stale', '>', 'P.sensors.per.protNStale');  L(s, 'Age/1', 'Stale/1');
A(s, 'simulink/Discrete/Unit Delay', 'HeldPrev', 'InitialCondition', 'true', 'SampleTime', 'Ts.perception');
A(s, 'simulink/Signal Routing/Switch', 'Held', 'Criteria', 'u2 ~= 0', 'OutDataTypeStr', 'boolean');
L(s, 'FieldStop/1', 'Held/1', 'fieldStop');  L(s, 'valid/1', 'Held/2');  L(s, 'HeldPrev/1', 'Held/3', 'heldPrev');
L(s, 'Held/1', 'HeldPrev/1', 'held');
LOG(s, 'Stop', 'OR', 2);  L(s, 'Held/1', 'Stop/1');  L(s, 'Stale/1', 'Stop/2', 'stale');
L(s, 'Stop/1', 'protectiveStop/1', 'protectiveStop');
set_param(s, 'Description', ['Protective field (sensors spec 3.7): length protL0 + protKv |v|, width protW; the front ' ...
    'field ahead of x = protXF (front of the body or of the fitted implement) when v > -protVBoth, the rear field ' ...
    'behind x = protXR (rear of the body or implement) when v < protVBoth (both at standstill); a field stops when ' ...
    'any kept point lies inside. Scan lost or stale (critfix R-02): on an invalid scan the last valid field result ' ...
    'is held for up to protNStale scans, then protectiveStop is true until a valid scan arrives; true from the ' ...
    'start until the first valid scan.']);
end
