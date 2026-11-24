function out = build_gnc_mission(which)
%BUILD_GNC_MISSION  Build the GNC mission-layer Subsystem References and the mission bench (task T15, gnc).
%
%   build_gnc_mission()             build every T15 file (the five SSRs and tests/gnc/tb_gnc_mission.slx)
%   build_gnc_mission(names)        only these (cellstr / char): gnc_MissionManager, gnc_LocalAvoidance,
%                                   gnc_GlobalReplanner, gnc_ImplementManager, gnc_StatusMetrics, tb_gnc_mission
%   built = build_gnc_mission(...)  names of the files written
%
%   Files (docs/build_plan.md T15, docs/specs/gnc.md 2.6, 2.8, 2.9, 4.1, 4.4, revision B):
%     subsystems/gnc/gnc_MissionManager.slx   MATLAB Function gnc_missionFSM (GncState / AutoSub / HoldReason
%                                             transition table), Unit Delays on pe / rp / teb_info (loop
%                                             breakers), soc_rth block math (spec 2.8), Bus_NavCmd (navMode
%                                             LIDAR_PRIOR_MAP for greenhouse missions)
%     subsystems/gnc/gnc_LocalAvoidance.slx   path-tube corridor (blocks, 0.1 s), Vector Field Histogram,
%                                             Variant VC_TEB: 1 Timed Elastic Band (TEB) in an Enabled
%                                             Subsystem at 0.2 s on a 10 x 10 m crop of Bus_Perception.map max-
%                                             pooled to 5 cells/m; 0 VFH-steered pursuit (no TEB), D56
%     subsystems/gnc/gnc_GlobalReplanner.slx  Rate Transitions to 1 s, Triggered Subsystem (replan_req edge) with
%                                             MATLAB Function gnc_globalReplanner (Hybrid A*), held result
%     subsystems/gnc/gnc_ImplementManager.slx mower / tiller / sprayer laws (blocks, spec 2.9)
%     subsystems/gnc/gnc_StatusMetrics.slx    10 s Moving RMS of the XTE on ROW, progress, area, dist_home,
%                                             ETA, Bus_GncStatus via Rate Transition 0.2 s, Bus_GncDebug
%     tests/gnc/tb_gnc_mission.slx            mission bench: Shaddad_GNC Model block, kinematic plant with the
%                                             MCU AUTO shaper (as tb_gnc), scenario tables P.gnc.tb.scn_*,
%                                             MCU / BMS / nav surrogates, harness LiDAR gnc_harnessLidar
%   Ports: the five SSRs keep the T14 contract build_shaddad_gnc('ports'). Every number comes from P / Ts.
%   Needs P, Ts, VC_*, Bus_* in base (shaddad_params is run when missing).
%   Diagrams (layout spec 6): every file is saved through shaddad_bu('save'), which lays out every level of it. The root
%   ports stay classic: a bus reaches each consumer through its own Bus Selector right in front of it (helpers BS, PICK),
%   and every signal line is named (L5), through shaddad_bu('nline') (helper L). The variant choices keep identical
%   classic ports.

all = {'gnc_MissionManager', 'gnc_LocalAvoidance', 'gnc_GlobalReplanner', 'gnc_ImplementManager', ...
       'gnc_StatusMetrics', 'tb_gnc_mission'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_FsmOut'',''var'')')
    shaddad_params();
end
T = build_shaddad_gnc('ports');
root = char(currentProject().RootFolder);
dirS = fullfile(root, 'subsystems', 'gnc');
% models of the gnc domain that reference the files (closed unsaved so the files can be replaced)
for m = {'tb_gnc_mission', 'tb_gnc', 'Shaddad_GNC'}
    if bdIsLoaded(m{1}) && strcmp(get_param(m{1}, 'Dirty'), 'off'), close_system(m{1}, 0); end
end
built = {};
for k = 1:numel(which)
    n = which{k};
    if strcmp(n, 'tb_gnc_mission')
        local_tb(n, fullfile(root, 'tests', 'gnc', [n '.slx']));
    else
        feval(['local_' n], n, fullfile(dirS, [n '.slx']), T);
    end
    built{end+1} = n; %#ok<AGROW>
end
if nargout > 0, out = built; end
end

% =============================================================================================
%  Helpers (same conventions as build_shaddad_gnc)
% =============================================================================================
function p = A(sys, lib, name, varargin)
p = [sys '/' name];
add_block(lib, p, varargin{:});
end

function L(sys, src, dst, nm)
% signal into dst ('Blk/k'), named nm (layout spec 6.1, L5); without nm the default of shaddad_bu('nline'): the name the
% source port's line already has, else the source block's name ('_k' for output k of a block with several outputs); a
% Bus Selector's output keeps R2026b's '<element>'
if nargin < 4, nm = ''; end
shaddad_bu('nline', sys, src, dst, nm);
end

function NM(sys, src, nm)
% name the line that leaves src ('Blk/k') (L5), for an output whose consumers other helpers wire
ph = get_param([sys '/' extractBefore(src, '/')], 'PortHandles');
set_param(get_param(ph.Outport(str2double(extractAfter(src, '/'))), 'Line'), 'Name', nm);
end

function LP(sys, src, blk, kind, nm)
% the line from src ('Blk/k') to the Trigger or Enable port (kind) of the subsystem blk, named nm (L5)
ps = get_param([sys '/' extractBefore(src, '/')], 'PortHandles');
pd = get_param([sys '/' blk], 'PortHandles');
h = add_line(sys, ps.Outport(str2double(extractAfter(src, '/'))), pd.(kind)(1), 'autorouting', 'smart');
set_param(h, 'Name', nm);
end

function PICK(sys, src, blk, sigs, dsts, nm)
% one consumer's Bus Selector blk right in front of it (layout spec 6.2): it selects sigs from the bus line src ('Blk/k';
% the line named nm when given) and feeds its output k to dsts{k} ('Blk/j', or a cellstr of such inputs)
if nargin < 5, dsts = {}; end
if nargin < 6, nm = ''; end
BS(sys, blk, sigs);
L(sys, src, [blk '/1'], nm);
for k = 1:numel(dsts)
    for d = reshape(cellstr(dsts{k}), 1, [])
        L(sys, sprintf('%s/%d', blk, k), d{1});
    end
end
end

function local_new_ssr(n, f, R)
shaddad_bu('fresh', n, f, 'ssr');
for k = 1:numel(R.in)
    p = R.in(k);
    A(n, 'simulink/Sources/In1', p.name, 'Port', num2str(k), 'OutDataTypeStr', p.type, ...
        'SampleTime', p.ts, 'PortDimensions', p.dims);
end
for k = 1:numel(R.out)
    p = R.out(k);
    A(n, 'simulink/Sinks/Out1', p.name, 'Port', num2str(k), 'OutDataTypeStr', p.type);
end
end

function local_save(n, f, desc)
set_param(n, 'Description', desc);     % no SHADDAD_GNC_STUB prefix: build_shaddad_gnc leaves the file alone
shaddad_bu('save', n, f);
close_system(n, 0);
end

function BS(sys, name, sigs, src)
A(sys, 'simulink/Signal Routing/Bus Selector', name, 'OutputSignals', strjoin(sigs, ','));
if nargin >= 4, L(sys, src, [name '/1']); end
end

function BC(sys, name, bus, elems, srcs)
A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(elems)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on');
for k = 1:numel(elems)
    src = srcs{k};
    blk = [sys '/' extractBefore(src, '/')];
    if strcmp(get_param(blk, 'BlockType'), 'BusSelector')
        sc = [name '_' elems{k}];
        A(sys, 'simulink/Signal Attributes/Signal Conversion', sc);
        L(sys, src, [sc '/1']);
        src = [sc '/1'];
    end
    L(sys, src, sprintf('%s/%d', name, k), elems{k});
end
end

function BA(sys, name, busSrc, sigs, srcs)
A(sys, 'simulink/Signal Routing/Bus Assignment', name, 'AssignedSignals', strjoin(sigs, ','));
L(sys, busSrc, [name '/1']);
for k = 1:numel(sigs)
    L(sys, srcs{k}, sprintf('%s/%d', name, k + 1));
end
end

function C(sys, name, val, ty, ts)
if nargin < 5, ts = 'Ts.gnc'; end
A(sys, 'simulink/Sources/Constant', name, 'Value', val, 'OutDataTypeStr', ty, 'SampleTime', ts);
end

function DTC(sys, name, ty, src)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', name, 'OutDataTypeStr', ty, 'RndMeth', 'Nearest', ...
    'SaturateOnIntegerOverflow', 'on');
if nargin >= 4, L(sys, src, [name '/1']); end
end

function RT(sys, name, ts, src)
A(sys, 'simulink/Signal Attributes/Rate Transition', name, 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTime', ts);
if nargin >= 4, L(sys, src, [name '/1']); end
end

function LUT(sys, name, bp, tab, src)
A(sys, 'simulink/Lookup Tables/1-D Lookup Table', name, 'BreakpointsForDimension1', bp, 'Table', tab, ...
    'InterpMethod', 'Linear point-slope', 'ExtrapMethod', 'Clip', 'UseLastTableValue', 'on');
if nargin >= 5, L(sys, src, [name '/1']); end
end

function TERM(sys, src, name)
A(sys, 'simulink/Sinks/Terminator', name);
L(sys, src, [name '/1']);
end

function SEL(sys, name, opts, prm, src, idxSrc, varargin)
nd = numel(strsplit(opts, ','));
if nd == 1, varargin = [{'InputPortWidth', '-1'}, varargin]; end
A(sys, 'simulink/Signal Routing/Selector', name, 'NumberOfDimensions', num2str(nd), 'IndexMode', 'One-based', ...
    varargin{:}, 'IndexOptions', opts, 'IndexParamArray', prm(:));
if nargin >= 5 && ~isempty(src), L(sys, src, [name '/1']); end
if nargin >= 6 && ~isempty(idxSrc), L(sys, idxSrc, [name '/2']); end
end

function CMP(sys, name, op, val, src)
A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', name, 'relop', op, 'const', val);
if nargin >= 5, L(sys, src, [name '/1']); end
end

function LOG(sys, name, op, srcs)
A(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', op, 'Inputs', num2str(max(1, numel(srcs))));
for k = 1:numel(srcs), L(sys, srcs{k}, sprintf('%s/%d', name, k)); end
end

function SW(sys, name, u1, u2, u3, crit)
if nargin < 6, crit = 'u2 ~= 0'; end
A(sys, 'simulink/Signal Routing/Switch', name, 'Criteria', crit);
L(sys, u1, [name '/1']); L(sys, u2, [name '/2']); L(sys, u3, [name '/3']);
end

function mf = MFB(sys, name, script, data, params, ts, desc)
% MATLAB Function block: body = one-line call of a %#codegen file; data = {name, DataType}; params =
% {name, expr} non-tunable parameter data (sub-structs, build rule 2) set through a mask
mf = A(sys, 'simulink/User-Defined Functions/MATLAB Function', name);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = script;
if strcmp(ts, '-1')
    cfg.UpdateMethod = 'Inherited';
else
    cfg.UpdateMethod = 'Discrete';
    cfg.SampleTime = ts;
end
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = false;
cfg.Description = desc;
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', mf);
for k = 1:size(data, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', data{k, 1});
    d.DataType = data{k, 2};
end
mk = Simulink.Mask.create(mf);
for k = 1:size(params, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k, 1});
    d.Scope = 'Parameter';
    d.Tunable = false;
    mk.addParameter('Type', 'edit', 'Name', params{k, 1}, 'Prompt', params{k, 1}, 'Value', params{k, 2}, ...
        'Evaluate', 'on', 'Tunable', 'off');
end
end

function local_clear(sys)
lns = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
blks = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all');
blks = setdiff(blks, {sys});
for k = 1:numel(blks), delete_block(blks{k}); end
end

function local_log(sys, blkPort, nm)
% mark the line leaving 'Blk/k' for signal logging (benches read it from logsout)
ph = get_param([sys '/' extractBefore(blkPort, '/')], 'PortHandles');
k = str2double(extractAfter(blkPort, '/'));
set_param(ph.Outport(k), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', nm);
end

% =============================================================================================
%  gnc_MissionManager
% =============================================================================================
function local_gnc_MissionManager(n, f, T)
local_new_ssr(n, f, T.(n));
% loop breakers (spec 7): delayed path error, replan result, TEB info
A(n, 'simulink/Discrete/Unit Delay', 'pe_z', 'InitialCondition', 'P.ic.Bus_PathError', 'SampleTime', '-1');
L(n, 'pe/1', 'pe_z/1');
A(n, 'simulink/Discrete/Unit Delay', 'rp_z', 'InitialCondition', 'P.ic.Bus_ReplanResult', 'SampleTime', '-1');
L(n, 'rp/1', 'rp_z/1');
A(n, 'simulink/Discrete/Unit Delay', 'teb_z', 'InitialCondition', 'zeros(4,1)', 'SampleTime', '-1');
L(n, 'teb_info/1', 'teb_z/1');
% ---- soc_rth = soc_reserve + k_tort e_m k_margin d_home / E_nom (spec 2.8, block math) ----
BS(n, 'NavSel', {'pose'}, 'nav/1');
BS(n, 'PlanSel_home', {'home'}, 'plan/1');
SEL(n, 'xy', 'Index vector (dialog)', {'[1 2]'}, 'NavSel/1');
SEL(n, 'home_xy', 'Index vector (dialog)', {'[1 2]'}, 'PlanSel_home/1');
A(n, 'simulink/Math Operations/Sum', 'dxy', 'Inputs', '+-');
L(n, 'xy/1', 'dxy/1'); L(n, 'home_xy/1', 'dxy/2');
A(n, 'simulink/Math Operations/Product', 'dxy2', 'Inputs', '**');
L(n, 'dxy/1', 'dxy2/1'); L(n, 'dxy/1', 'dxy2/2');
A(n, 'simulink/Math Operations/Sum of Elements', 'd2');
L(n, 'dxy2/1', 'd2/1');
A(n, 'simulink/Math Operations/Sqrt', 'd_home');
L(n, 'd2/1', 'd_home/1');
A(n, 'simulink/Math Operations/Gain', 'E_home', 'Gain', 'P.gnc.fsm.k_tort*P.gnc.fsm.e_m*P.gnc.fsm.k_margin/P.gnc.fsm.E_nom_Wh');
L(n, 'd_home/1', 'E_home/1');
A(n, 'simulink/Math Operations/Bias', 'soc_rth_b', 'Bias', 'P.gnc.fsm.soc_reserve');
L(n, 'E_home/1', 'soc_rth_b/1');
L(n, 'soc_rth_b/1', 'soc_rth/1', 'soc_rth');
% ---- mission FSM ----
MFB(n, 'MissionFSM', sprintf(['function fsm = fcn(nav, mcu, bms, mcmd, obs, pe, plan, rp, teb, socRth, prm)\n' ...
    '%%#codegen\nfsm = gnc_missionFSM(nav, mcu, bms, mcmd, obs, pe, plan, rp, teb, socRth, prm);\n']), ...
    {'nav', 'Bus: Bus_NavCond'; 'mcu', 'Bus: Bus_McuStatus'; 'bms', 'Bus: Bus_BmsStatus'; ...
     'mcmd', 'Bus: Bus_MissionCmd'; 'obs', 'Bus: Bus_ObsFlags'; 'pe', 'Bus: Bus_PathError'; ...
     'plan', 'Bus: Bus_MissionPlan'; 'rp', 'Bus: Bus_ReplanResult'; 'teb', 'double'; 'socRth', 'double'; ...
     'fsm', 'Bus: Bus_FsmOut'}, {'prm', 'P.gnc.fsm'}, 'Ts.gnc', ...
    'Mission manager FSM (gnc spec 4.1, build plan T15); body subsystems/gnc/gnc_missionFSM.m.');
src = {'nav/1', 'mcu/1', 'bms/1', 'mcmd/1', 'obs/1', 'pe_z/1', 'plan/1', 'rp_z/1', 'teb_z/1', 'soc_rth_b/1'};
for k = 1:numel(src), L(n, src{k}, sprintf('MissionFSM/%d', k)); end
L(n, 'MissionFSM/1', 'fsm/1', 'fsm');
local_log(n, 'MissionFSM/1', 'fsm');
% ---- Bus_NavCmd: navMode LIDAR_PRIOR_MAP for greenhouse missions (T12 note), else AUTO ----
BS(n, 'PlanSel_type', {'mission_type'}, 'plan/1');
CMP(n, 'is_gh', '==', 'double(MissionType.GREENHOUSE_COVERAGE)', 'PlanSel_type/1');
C(n, 'mode_gh', 'uint8(P.gnc.fsm.nav_mode_gh)', 'uint8');
C(n, 'mode_auto', 'uint8(0)', 'uint8');
SW(n, 'navMode', 'mode_gh/1', 'is_gh/1', 'mode_auto/1');
C(n, 'NavCmdIC', 'P.ic.Bus_NavCmd', 'Bus: Bus_NavCmd');
BA(n, 'NavCmd', 'NavCmdIC/1', {'navMode'}, {'navMode/1'});
L(n, 'NavCmd/1', 'nav_cmd/1', 'nav_cmd');
local_save(n, f, sprintf(['gnc_MissionManager (T15, gnc spec 4.1 / 2.8, build plan T15, D44). MATLAB Function ' ...
    'gnc_missionFSM (Stateflow is not installed, F1): states GncState, sub-modes AutoSub, HoldReason; ordered ' ...
    'transition table G1 G2 T1-T10 S1-S5 H1-H3 P1-P4 R1-R6 F1 (see the function header); fsm.trans_id logs ' ...
    'the transition of each step; output line logged as ''fsm''. Inputs: nav, mcu (rc_mode_sw 0 MANUAL / ' ...
    '1 HOLD / 2 AUTO, drive_ready, estop_active, charging, rc_link_ok, implement_ready, heartbeat, ' ...
    'v_applied), bms (SOC, bms_critical), conditioned mission command, obs flags, pe / rp / teb_info through ' ...
    'Unit Delays (loop breakers), plan (MissionSource). obs.scan_valid (Bus_Perception.scanValid): a scan lost (invalid ' ...
    'for more than P.sensors.per.protNStale scans, P.gnc.fsm.t_scan_lost) enters or keeps OBSTACLE_HOLD with ' ...
    'HoldReason SENSOR_LOST (T5 / R6); H1 / H2 wait for a valid scan. T1 needs START and mcu.drive_ready; RC loss: drive_ready ' ...
    'drops with rc_link_ok false -> PAUSE, HoldReason RC_LOSS (the MCU owns RC-loss behaviour, D44). ' ...
    'E_nom = P.gnc.fsm.E_nom_Wh. soc_rth = soc_reserve + k_tort e_m k_margin |pose - home| / E_nom ' ...
    '(Sum, Product, Sum of Elements, Sqrt, Gain, Bias). nav_cmd.navMode = P.gnc.fsm.nav_mode_gh ' ...
    '(LIDAR_PRIOR_MAP) for GREENHOUSE_COVERAGE plans, else 0 AUTO. Parameter P.gnc.fsm (non-tunable mask).']));
end

% =============================================================================================
%  gnc_LocalAvoidance
% =============================================================================================
function local_gnc_LocalAvoidance(n, f, T)
local_new_ssr(n, f, T.(n));
tsA = 'Ts.gnc_avoid';
% ---- inputs to the 0.1 s avoidance rate; one Bus Selector per consumer, right in front of it (layout spec 6.2) ----
RT(n, 'RT_nav', tsA, 'nav/1');
RT(n, 'RT_win', tsA, 'win/1');
RT(n, 'RT_pe', tsA, 'pe/1');
RT(n, 'RT_tdir', tsA, 'tdir/1');
PICK(n, 'RT_nav/1', 'NavSel', {'pose'}, {}, 'nav_100ms');
PICK(n, 'RT_pe/1', 'PeSel', {'s'}, {}, 'pe_100ms');
SEL(n, 'px', 'Index vector (dialog)', {'1'}, 'NavSel/1');
SEL(n, 'py', 'Index vector (dialog)', {'2'}, 'NavSel/1');
SEL(n, 'psi', 'Index vector (dialog)', {'3'}, 'NavSel/1');
% ---- beam points: body frame, then world frame (column vectors n_beams x 1) ----
BS(n, 'PerSel_r', {'ranges_B'}, 'perception/1');
BS(n, 'PerSel_a', {'angles_B'}, 'perception/1');
DTC(n, 'r_d', 'double', 'PerSel_r/1');
DTC(n, 'a_d', 'double', 'PerSel_a/1');
A(n, 'simulink/Math Operations/Reshape', 'r_col', 'OutputDimensionality', 'Column vector (2-D)');  L(n, 'r_d/1', 'r_col/1');
A(n, 'simulink/Math Operations/Reshape', 'a_col', 'OutputDimensionality', 'Column vector (2-D)');  L(n, 'a_d/1', 'a_col/1');
A(n, 'simulink/Math Operations/Sum', 'a_w', 'Inputs', '++');  L(n, 'a_col/1', 'a_w/1'); L(n, 'psi/1', 'a_w/2');
A(n, 'simulink/Math Operations/Trigonometric Function', 'cos_a', 'Operator', 'cos');  L(n, 'a_w/1', 'cos_a/1');
A(n, 'simulink/Math Operations/Trigonometric Function', 'sin_a', 'Operator', 'sin');  L(n, 'a_w/1', 'sin_a/1');
A(n, 'simulink/Math Operations/Product', 'rx', 'Inputs', '**');  L(n, 'r_col/1', 'rx/1'); L(n, 'cos_a/1', 'rx/2');
A(n, 'simulink/Math Operations/Product', 'ry', 'Inputs', '**');  L(n, 'r_col/1', 'ry/1'); L(n, 'sin_a/1', 'ry/2');
A(n, 'simulink/Math Operations/Sum', 'Xb', 'Inputs', '++');  L(n, 'rx/1', 'Xb/1'); L(n, 'px/1', 'Xb/2');
A(n, 'simulink/Math Operations/Sum', 'Yb', 'Inputs', '++');  L(n, 'ry/1', 'Yb/1'); L(n, 'py/1', 'Yb/2');
% ---- path samples ahead of the closest point: s_q = s + s_tube on [A; B(2:end)] (Lookup Table Dynamic) ----
PICK(n, 'RT_win/1', 'WinSel_sA', {'sA'}, {}, 'win_100ms');
PICK(n, 'RT_win/1', 'WinSel_sB', {'sB'});
SEL(n, 'sA_end', 'Index vector (dialog)', {'P.gnc.path.N_smooth'}, 'WinSel_sA/1');
SEL(n, 'sB2', 'Index vector (dialog)', {'2:P.gnc.path.N_smooth'}, 'WinSel_sB/1');
A(n, 'simulink/Math Operations/Sum', 'sB_off', 'Inputs', '++');  L(n, 'sB2/1', 'sB_off/1'); L(n, 'sA_end/1', 'sB_off/2');
A(n, 'simulink/Signal Routing/Vector Concatenate', 's_all', 'NumInputs', '2', 'Mode', 'Vector');
PICK(n, 'RT_win/1', 'WinSel_sAll', {'sA'}, {'s_all/1'}); L(n, 'sB_off/1', 's_all/2');
PICK(n, 'RT_win/1', 'WinSel_PA', {'PA'});
PICK(n, 'RT_win/1', 'WinSel_PB', {'PB'});
for c = {'x', 'y'}
    x = c{1};  col = num2str(1 + strcmp(x, 'y'));
    SEL(n, ['PA_' x], 'Select all,Index vector (dialog)', {''; col}, 'WinSel_PA/1');
    SEL(n, ['PB_' x], 'Index vector (dialog),Index vector (dialog)', {'2:P.gnc.path.N_smooth'; col}, 'WinSel_PB/1');
    A(n, 'simulink/Signal Routing/Vector Concatenate', [x '_all'], 'NumInputs', '2', 'Mode', 'Vector');
    L(n, ['PA_' x '/1'], [x '_all/1']); L(n, ['PB_' x '/1'], [x '_all/2']);
end
C(n, 's_tube', 'P.gnc.obs.s_tube', 'double', tsA);
set_param([n '/s_tube'], 'VectorParams1D', 'off');
A(n, 'simulink/Math Operations/Sum', 's_q', 'Inputs', '++');  L(n, 's_tube/1', 's_q/1'); L(n, 'PeSel/1', 's_q/2');
A(n, 'simulink/Lookup Tables/Lookup Table Dynamic', 'path_x');
A(n, 'simulink/Lookup Tables/Lookup Table Dynamic', 'path_y');
L(n, 's_q/1', 'path_x/1'); L(n, 's_all/1', 'path_x/2'); L(n, 'x_all/1', 'path_x/3');
L(n, 's_q/1', 'path_y/1'); L(n, 's_all/1', 'path_y/2'); L(n, 'y_all/1', 'path_y/3');
% ---- distance of every beam point to every path sample (n_beams x n_tube) ----
C(n, 'ones_t', 'ones(1, P.gnc.obs.n_tube)', 'double', tsA);
C(n, 'ones_b', 'ones(P.gnc.obs.n_beams, 1)', 'double', tsA);
set_param([n '/ones_t'], 'VectorParams1D', 'off'); set_param([n '/ones_b'], 'VectorParams1D', 'off');
for c = {'X', 'Y'}
    x = c{1};  src = 'path_x/1';  if x == 'Y', src = 'path_y/1'; end
    A(n, 'simulink/Math Operations/Product', [x 'm'], 'Multiplication', 'Matrix(*)', 'Inputs', '**');
    L(n, [x 'b/1'], [x 'm/1']); L(n, 'ones_t/1', [x 'm/2']);
    A(n, 'simulink/Math Operations/Product', [x 'p'], 'Multiplication', 'Matrix(*)', 'Inputs', '**');
    L(n, 'ones_b/1', [x 'p/1']); L(n, src, [x 'p/2']);
    A(n, 'simulink/Math Operations/Sum', ['d' x], 'Inputs', '+-');  L(n, [x 'm/1'], ['d' x '/1']); L(n, [x 'p/1'], ['d' x '/2']);
end
A(n, 'simulink/Math Operations/Math Function', 'D', 'Operator', 'hypot');  L(n, 'dX/1', 'D/1'); L(n, 'dY/1', 'D/2');
CMP(n, 'in_tube', '<', 'P.gnc.obs.w_c', 'D/1');
C(n, 'ahead', 'repmat(P.gnc.obs.s_tube > P.gnc.obs.x_f, P.gnc.obs.n_beams, 1)', 'boolean', tsA);
LOG(n, 'hit', 'AND', {'in_tube/1', 'ahead/1'});
C(n, 'd_along', 'repmat(P.gnc.obs.s_tube - P.gnc.obs.x_f, P.gnc.obs.n_beams, 1)', 'double', tsA);
C(n, 'd_inf', 'inf', 'double', tsA);
SW(n, 'd_hit', 'd_along/1', 'hit/1', 'd_inf/1');
A(n, 'simulink/Math Operations/MinMax', 'd_obs_s', 'Function', 'min', 'Inputs', '1');  L(n, 'd_hit/1', 'd_obs_s/1');
% ---- Vector Field Histogram (0.1 s; NaN ranges = no return, ignored by VFH: T13 note) ----
v = 'P.gnc.vfh';
A(n, 'navlib/Control Algorithms/Vector Field Histogram', 'VFH', 'NumAngularSectors', [v '.NumAngularSectors'], ...
    'DistanceLimits', [v '.DistanceLimits'], 'HistogramThresholds', [v '.HistogramThresholds'], ...
    'RobotRadius', [v '.RobotRadius'], 'SafetyDistance', [v '.SafetyDistance'], ...
    'MinTurningRadius', [v '.MinTurningRadius'], 'TargetDirectionWeight', [v '.TargetDirectionWeight'], ...
    'CurrentDirectionWeight', [v '.CurrentDirectionWeight'], 'PreviousDirectionWeight', [v '.PreviousDirectionWeight'], ...
    'SimulateUsing', 'Code generation');
L(n, 'r_d/1', 'VFH/1'); L(n, 'a_d/1', 'VFH/2'); L(n, 'RT_tdir/1', 'VFH/3');
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'vfh_nan', 'Operator', 'isNaN');
L(n, 'VFH/1', 'vfh_nan/1', 'steer');
% ---- 0.1 -> 0.05 s and Bus_ObsFlags ----
RT(n, 'RT_dobs', 'Ts.gnc', 'd_obs_s/1');
set_param([n '/RT_dobs'], 'InitialCondition', '1e6');    % clear until the first scan (finite IC required)
RT(n, 'RT_blk', 'Ts.gnc', 'vfh_nan/1');
BS(n, 'PerSel_prot', {'protectiveStop'}, 'perception/1');
RT(n, 'RT_prot', 'Ts.gnc', 'PerSel_prot/1');
RT(n, 'RT_steer', 'Ts.gnc', 'VFH/1');
% scan validity for the FSM (critfix R-02): a lost scan holds the robot (SENSOR_LOST); invalid until the first scan
BS(n, 'PerSel_scan', {'scanValid'}, 'perception/1');
RT(n, 'RT_scan', 'Ts.gnc', 'PerSel_scan/1');
set_param([n '/RT_scan'], 'InitialCondition', 'false');
BC(n, 'ObsFlags', 'Bus_ObsFlags', {'d_obs', 'blocked_vfh', 'person_near', 'vfh_steer', 'scan_valid'}, ...
    {'RT_dobs/1', 'RT_blk/1', 'RT_prot/1', 'RT_steer/1', 'RT_scan/1'});
L(n, 'ObsFlags/1', 'obs/1', 'obs');
local_log(n, 'ObsFlags/1', 'obs');
% ---- bypass: Variant Subsystem VC_TEB (D56) ----
vs = [n '/Bypass'];
A(n, 'simulink/Ports & Subsystems/Variant Subsystem', 'Bypass');
c = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
c = setdiff(c, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
bin = {'nav', 'fsm', 'win', 'map', 'steer'};  bout = {'teb_cmd', 'teb_info'};
for k = 1:numel(bin), A(vs, 'simulink/Sources/In1', bin{k}, 'Port', num2str(k)); end
for k = 1:numel(bout), A(vs, 'simulink/Sinks/Out1', bout{k}, 'Port', num2str(k)); end
A(vs, 'simulink/Ports & Subsystems/Subsystem', 'TEB', 'VariantControl', 'VC_TEB == 1');
A(vs, 'simulink/Ports & Subsystems/Subsystem', 'NO_TEB', 'VariantControl', 'VC_TEB == 0');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
local_teb([vs '/TEB'], bin, bout);
local_noteb([vs '/NO_TEB'], bin, bout);
L(n, 'nav/1', 'Bypass/1'); L(n, 'fsm/1', 'Bypass/2'); L(n, 'win/1', 'Bypass/3');
PICK(n, 'perception/1', 'PerSel_map', {'map'}, {'Bypass/4'}); L(n, 'VFH/1', 'Bypass/5');
L(n, 'Bypass/1', 'teb_cmd/1', 'teb_cmd'); L(n, 'Bypass/2', 'teb_info/1', 'teb_info');
local_save(n, f, sprintf(['gnc_LocalAvoidance (T15, gnc spec 2.6 / 3.1, D23, D56). Corridor (blocks, 0.1 s): beam points ' ...
    '(ranges_B, angles_B; NaN = no return, compares false) in the world frame are tested against a tube of ' ...
    'half-width P.gnc.obs.w_c around P.gnc.obs.n_tube path samples s + P.gnc.obs.s_tube (0 .. x_front + L_c ' ...
    'ahead of the closest point) on the window [A; B] (Lookup Table Dynamic on the cumulative length); d_obs = min ' ...
    'over the hits of (sample offset - x_front), samples beside the body excluded, Inf = clear. The tube follows ' ...
    'the planned path, so walls beyond a lane end or beside a curved transit do not stop the robot (a body-frame ' ...
    'box does, greenhouse turns / transits). VFH: navlib Vector Field Histogram (P.gnc.vfh), TargetDir = Pure ' ...
    'Pursuit target direction (delayed), blocked = isNaN(SteerDir); person_near = Bus_Perception.protectiveStop ' ...
    '(D23), scan_valid = Bus_Perception.scanValid (Rate Transition IC false: invalid until the first scan). Rate ' ...
    'Transitions 0.1 -> 0.05 s into Bus_ObsFlags (logged as ''obs'').\nBypass Variant Subsystem ' ...
    'VC_TEB (update diagram): 1 TEB = Enabled Subsystem at P.gnc.teb.Ts (enable fsm.teb_enable, outputs reset ' ...
    'when disabled) with navlib Timed Elastic Band (TEB) (Path Following with Obstacle Avoidance, P.gnc.teb) on ' ...
    'MapInfo = P.gnc.teb.map_window crop of Bus_Perception.map (Selectors), 2 x 2 max-pooled (map_decim, Logical ' ...
    'OR) to Bus_TebMapCrop, RefPath = window [A; B] rows 1:teb.ref_decim:end (heading deg -> rad), Pose, Vel = ' ...
    'previous TEB command ' ...
    '(Unit Delay; the measured speed lags by the dead time and drive lag); teb_cmd = ' ...
    'VelCmds row 1, teb_info = SolnInfo [ExitFlag LastFeasibleIdx HasReachedGoal TrajectoryCost]; Rate ' ...
    'Transitions 0.05/0.1 -> 0.2 -> 0.05 s. 0 NO_TEB = VFH-steered pursuit v = min(fsm.v_cap, ' ...
    'teb.MaxVelocity(1)) (0 when VFH blocked), omega = 2 v sin(SteerDir) / Ld(v), teb_info = [-1 0 0 0] ' ...
    '(ExitFlag -1 tells the FSM that no TEB exists). In both variants the FSM serves a LOCAL Hybrid A* detour ' ...
    'as the window during BYPASS.']));
end

function local_teb(s, bin, bout)
local_clear(s);
for k = 1:numel(bin), A(s, 'simulink/Sources/In1', bin{k}, 'Port', num2str(k)); end
for k = 1:numel(bout), A(s, 'simulink/Sinks/Out1', bout{k}, 'Port', num2str(k)); end
TERM(s, 'steer/1', 'T_steer');
ts = 'P.gnc.teb.Ts';
RT(s, 'RT_nav', ts, 'nav/1'); RT(s, 'RT_fsm', ts, 'fsm/1'); RT(s, 'RT_win', ts, 'win/1'); RT(s, 'RT_map', ts, 'map/1');
PICK(s, 'RT_nav/1', 'NavSel', {'pose', 'v', 'yaw_rate'}, {}, 'nav_200ms');
PICK(s, 'RT_fsm/1', 'FsmSel', {'teb_enable'}, {}, 'fsm_200ms');
PICK(s, 'RT_win/1', 'WinSel', {'PA', 'PB'}, {}, 'win_200ms');
PICK(s, 'RT_map/1', 'MapSel', {'GridLocationInWorld', 'OccupancyMatrix', 'GridOriginInLocal'}, {}, 'map_200ms');
e = [s '/TEB_Enabled'];
A(s, 'simulink/Ports & Subsystems/Enabled Subsystem', 'TEB_Enabled');
local_clear(e);
ein = {'pose', 'v', 'w', 'PA', 'PB', 'gl', 'occ', 'org'};
for k = 1:numel(ein), A(e, 'simulink/Sources/In1', ein{k}, 'Port', num2str(k)); end
A(e, 'simulink/Ports & Subsystems/Enable', 'Enable', 'StatesWhenEnabling', 'reset', 'PropagateVarSize', 'During execution');
A(e, 'simulink/Sinks/Out1', 'cmd', 'Port', '1', 'InitialOutput', 'zeros(2,1)', 'OutputWhenDisabled', 'reset');
A(e, 'simulink/Sinks/Out1', 'info', 'Port', '2', 'InitialOutput', 'zeros(4,1)', 'OutputWhenDisabled', 'reset');
% MapInfo: map_window crop of the robot-centred grid, 2 x 2 max pooling (map_decim) -> Bus_TebMapCrop
nc = 'P.gnc.teb.map_window*P.gnc.map.local_res';
c0 = ['(P.gnc.map.local_N - ' nc ')/2'];
SEL(e, 'crop', 'Index vector (dialog),Index vector (dialog)', {[c0 ' + (1:' nc ')']; [c0 ' + (1:' nc ')']}, 'occ/1');
q = {'o11', '1:2:', '1:2:'; 'o21', '2:2:', '1:2:'; 'o12', '1:2:', '2:2:'; 'o22', '2:2:', '2:2:'};
for k = 1:4
    SEL(e, q{k, 1}, 'Index vector (dialog),Index vector (dialog)', {[q{k, 2} nc]; [q{k, 3} nc]}, 'crop/1');
end
LOG(e, 'pool', 'OR', {'o11/1', 'o21/1', 'o12/1', 'o22/1'});
C(e, 'res', 'P.gnc.map.local_res/P.gnc.teb.map_decim', 'double', '-1');
C(e, 'gsize', ['[1 1]*' nc '/P.gnc.teb.map_decim'], 'double', '-1');
set_param([e '/gsize'], 'VectorParams1D', 'off');
A(e, 'simulink/Math Operations/Bias', 'gl_c', 'Bias', [c0 '/P.gnc.map.local_res']);  L(e, 'gl/1', 'gl_c/1');
A(e, 'simulink/Math Operations/Bias', 'org_c', 'Bias', [c0 '/P.gnc.map.local_res']); L(e, 'org/1', 'org_c/1');
BC(e, 'MapInfo', 'Bus_TebMapCrop', {'Resolution', 'GridLocationInWorld', 'GridSize', 'OccupancyMatrix', 'GridOriginInLocal'}, ...
    {'res/1', 'gl_c/1', 'gsize/1', 'pool/1', 'org_c/1'});
% RefPath = [PA; PB] decimated by teb.ref_decim (TEB initialises its band from the reference poses with
% ReferenceDeltaTime each: the 400-pose window of a short detour, 0.02 m apart, held TEB at 0.3 m/s), heading in rad
A(e, 'simulink/Signal Routing/Vector Concatenate', 'AB', 'NumInputs', '2', 'ConcatenateDimension', '1', ...
    'Mode', 'Multidimensional array');
SEL(e, 'PA_d', 'Index vector (dialog),Select all', {'1:P.gnc.teb.ref_decim:P.gnc.path.N_smooth'; ''}, 'PA/1');
SEL(e, 'PB2', 'Index vector (dialog),Select all', {'1 + P.gnc.teb.ref_decim:P.gnc.teb.ref_decim:P.gnc.path.N_smooth'; ''}, 'PB/1');
L(e, 'PA_d/1', 'AB/1'); L(e, 'PB2/1', 'AB/2');
SEL(e, 'ab_xy', 'Select all,Index vector (dialog)', {''; '[1 2]'}, 'AB/1');
SEL(e, 'ab_th', 'Select all,Index vector (dialog)', {''; '3'}, 'AB/1');
A(e, 'simulink/Math Operations/Gain', 'th_rad', 'Gain', 'pi/180');  L(e, 'ab_th/1', 'th_rad/1');
A(e, 'simulink/Signal Routing/Vector Concatenate', 'RefPath', 'NumInputs', '2', 'ConcatenateDimension', '2', ...
    'Mode', 'Multidimensional array');
L(e, 'ab_xy/1', 'RefPath/1'); L(e, 'th_rad/1', 'RefPath/2');
% Vel = the previous TEB command (Unit Delay, reset on enable): with the measured [v w] the 0.1 s dead
% time + MCU shaper + drive lag keep TEB's first command near the lagging speed (0.3 m/s measured, T15)
TERM(e, 'v/1', 'T_v'); TERM(e, 'w/1', 'T_w');
A(e, 'simulink/Discrete/Unit Delay', 'Vel', 'InitialCondition', 'zeros(2,1)', 'SampleTime', '-1');
t = 'P.gnc.teb';
A(e, 'navlib/Control Algorithms/Timed Elastic Band (TEB)', 'TEB', 'ControlMode', 'Path Following with Obstacle Avoidance', ...
    'SampleTime', '-1', 'LookAheadTime', [t '.LookAheadTime'], 'GoalTolerance', [t '.GoalTolerance'], ...
    'MaxVelocity', [t '.MaxVelocity'], 'MaxReverseVelocity', 'NaN', 'MaxAcceleration', [t '.MaxAcceleration'], ...
    'MinTurningRadius', [t '.MinTurningRadius'], 'ReferenceDeltaTime', [t '.ReferenceDeltaTime'], ...
    'NumIteration', [t '.NumIteration'], 'Time', [t '.Time'], 'Smoothness', [t '.Smoothness'], ...
    'Obstacle', [t '.Obstacle'], 'Shape', 'Rectangle', 'Dimension', [t '.Dimension'], ...
    'ObstacleSafetyMargin', [t '.ObstacleSafetyMargin']);
L(e, 'pose/1', 'TEB/1'); L(e, 'Vel/1', 'TEB/2'); L(e, 'MapInfo/1', 'TEB/3'); L(e, 'RefPath/1', 'TEB/4');
SEL(e, 'cmd1', 'Index vector (dialog),Index vector (dialog)', {'1'; '[1 2]'}, 'TEB/1');
A(e, 'simulink/Math Operations/Reshape', 'cmd_col', 'OutputDimensionality', '1-D array');
L(e, 'cmd1/1', 'cmd_col/1');
L(e, 'cmd_col/1', 'cmd/1'); L(e, 'cmd_col/1', 'Vel/1');
TERM(e, 'TEB/2', 'T_ts'); TERM(e, 'TEB/3', 'T_path');
BS(e, 'InfoSel', {'ExitFlag', 'LastFeasibleIdx', 'HasReachedGoal', 'TrajectoryCost'}, 'TEB/4');
NM(e, 'TEB/1', 'VelCmds'); NM(e, 'TEB/2', 'TimeStamps'); NM(e, 'TEB/3', 'OptPath'); NM(e, 'TEB/4', 'SolnInfo');
for k = 1:4, DTC(e, sprintf('i%d', k), 'double', sprintf('InfoSel/%d', k)); end
A(e, 'simulink/Signal Routing/Mux', 'InfoVec', 'Inputs', '4');
for k = 1:4, L(e, sprintf('i%d/1', k), sprintf('InfoVec/%d', k)); end
L(e, 'InfoVec/1', 'info/1');
% wire the enabled subsystem
src = {'NavSel/1', 'NavSel/2', 'NavSel/3', 'WinSel/1', 'WinSel/2', 'MapSel/1', 'MapSel/2', 'MapSel/3'};
for k = 1:numel(src), L(s, src{k}, sprintf('TEB_Enabled/%d', k)); end
ph = get_param(e, 'PortHandles');  pf = get_param([s '/FsmSel'], 'PortHandles');
add_line(s, pf.Outport(1), ph.Enable(1), 'autorouting', 'smart');
RT(s, 'RT_cmd', 'Ts.gnc', 'TEB_Enabled/1'); RT(s, 'RT_info', 'Ts.gnc', 'TEB_Enabled/2');
NM(s, 'TEB_Enabled/1', 'cmd_200ms'); NM(s, 'TEB_Enabled/2', 'info_200ms');
L(s, 'RT_cmd/1', 'teb_cmd/1', 'teb_cmd'); L(s, 'RT_info/1', 'teb_info/1', 'teb_info');
end

function local_noteb(s, bin, bout)
local_clear(s);
for k = 1:numel(bin), A(s, 'simulink/Sources/In1', bin{k}, 'Port', num2str(k)); end
for k = 1:numel(bout), A(s, 'simulink/Sinks/Out1', bout{k}, 'Port', num2str(k)); end
TERM(s, 'nav/1', 'T_nav'); TERM(s, 'win/1', 'T_win'); TERM(s, 'map/1', 'T_map');
ts = 'Ts.gnc_avoid';
RT(s, 'RT_fsm', ts, 'fsm/1');
BS(s, 'FsmSel', {'v_cap'}, 'RT_fsm/1');
C(s, 'v_teb', 'P.gnc.teb.MaxVelocity(1)', 'double', ts);
A(s, 'simulink/Math Operations/MinMax', 'v_b0', 'Function', 'min', 'Inputs', '2');
L(s, 'FsmSel/1', 'v_b0/1'); L(s, 'v_teb/1', 'v_b0/2');
A(s, 'simulink/Logic and Bit Operations/Relational Operator', 'blocked', 'Operator', 'isNaN');  L(s, 'steer/1', 'blocked/1');
C(s, 'zero', '0', 'double', ts);
SW(s, 'v_b', 'zero/1', 'blocked/1', 'v_b0/1');
SW(s, 'steer0', 'zero/1', 'blocked/1', 'steer/1');
LUT(s, 'Ld', 'P.gnc.obs.vfh_Ld_v', 'P.gnc.obs.vfh_Ld', 'v_b/1');
A(s, 'simulink/Math Operations/Trigonometric Function', 'sin_s', 'Operator', 'sin');  L(s, 'steer0/1', 'sin_s/1');
A(s, 'simulink/Math Operations/Product', 'w_num', 'Inputs', '**');  L(s, 'v_b/1', 'w_num/1'); L(s, 'sin_s/1', 'w_num/2');
A(s, 'simulink/Math Operations/Product', 'w_div', 'Inputs', '*/');  L(s, 'w_num/1', 'w_div/1'); L(s, 'Ld/1', 'w_div/2');
A(s, 'simulink/Math Operations/Gain', 'w_b', 'Gain', '2');  L(s, 'w_div/1', 'w_b/1');
A(s, 'simulink/Signal Routing/Mux', 'cmd', 'Inputs', '2');  L(s, 'v_b/1', 'cmd/1'); L(s, 'w_b/1', 'cmd/2');
RT(s, 'RT_cmd', 'Ts.gnc', 'cmd/1');
L(s, 'RT_cmd/1', 'teb_cmd/1', 'teb_cmd');
C(s, 'info', '[-1; 0; 0; 0]', 'double', 'Ts.gnc');
L(s, 'info/1', 'teb_info/1', 'teb_info');
end

% =============================================================================================
%  gnc_GlobalReplanner
% =============================================================================================
function local_gnc_GlobalReplanner(n, f, T)
local_new_ssr(n, f, T.(n));
tp = 'Ts.plan';
RT(n, 'RT_fsm', tp, 'fsm/1'); RT(n, 'RT_nav', tp, 'nav/1'); RT(n, 'RT_per', tp, 'perception/1');
PICK(n, 'RT_fsm/1', 'FsmSel_trig', {'replan_req'}, {}, 'fsm_1s');
PICK(n, 'RT_fsm/1', 'FsmSel', {'replan_goal', 'replan_kind', 'seg_idx'});
PICK(n, 'RT_nav/1', 'NavSel', {'pose'}, {}, 'nav_1s');
PICK(n, 'RT_per/1', 'PerSel', {'map'}, {}, 'per_1s');
DTC(n, 'trig', 'double', 'FsmSel_trig/1');
tr = [n '/Replan'];
A(n, 'simulink/Ports & Subsystems/Triggered Subsystem', 'Replan');
local_clear(tr);
rin = {'goal', 'kind', 'seg', 'pose', 'map', 'plan'};
for k = 1:numel(rin), A(tr, 'simulink/Sources/In1', rin{k}, 'Port', num2str(k)); end
A(tr, 'simulink/Ports & Subsystems/Trigger', 'Trigger', 'TriggerType', 'either', 'InitialTriggerSignalState', 'zero');
A(tr, 'simulink/Sinks/Out1', 'rp', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_ReplanResult', 'InitialOutput', 'P.ic.Bus_ReplanResult');
C(tr, 'occ', 'P.world.occ.grid', 'boolean', 'inf');
C(tr, 'occRes', 'P.world.occ.res', 'double', 'inf');
C(tr, 'occOrigin', 'P.world.occ.origin', 'double', 'inf');
% terrain slope grids (no turning on slopes above P.gnc.hastar.slope_turn_max, D53)
C(tr, 'xE', 'P.world.terrain.xE', 'double', 'inf');  C(tr, 'yN', 'P.world.terrain.yN', 'double', 'inf');
C(tr, 'dHdE', 'P.world.terrain.dHdE', 'double', 'inf');  C(tr, 'dHdN', 'P.world.terrain.dHdN', 'double', 'inf');
MFB(tr, 'HybridAStar', sprintf(['function rp = fcn(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, xE, yN, dHdE, dHdN, prm)\n' ...
    '%%#codegen\nrp = gnc_globalReplanner(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, ...\n' ...
    '    struct(''xE'', xE, ''yN'', yN, ''dHdE'', dHdE, ''dHdN'', dHdN), prm);\n']), ...
    {'map', 'Bus: Bus_TebMapInfo'; 'plan', 'Bus: Bus_MissionPlan'; 'rp', 'Bus: Bus_ReplanResult'}, ...
    {'prm', 'P.gnc.hastar'}, '-1', ...
    'Hybrid A* replanning (gnc spec 4.4); body subsystems/gnc/gnc_globalReplanner.m.');
src = {'goal/1', 'kind/1', 'seg/1', 'pose/1', 'map/1', 'plan/1', 'occ/1', 'occRes/1', 'occOrigin/1', 'xE/1', 'yN/1', ...
       'dHdE/1', 'dHdN/1'};
for k = 1:numel(src), L(tr, src{k}, sprintf('HybridAStar/%d', k)); end
L(tr, 'HybridAStar/1', 'rp/1', 'rp');
L(n, 'FsmSel/1', 'Replan/1'); L(n, 'FsmSel/2', 'Replan/2'); L(n, 'FsmSel/3', 'Replan/3');
L(n, 'NavSel/1', 'Replan/4'); L(n, 'PerSel/1', 'Replan/5'); L(n, 'plan/1', 'Replan/6');
LP(n, 'trig/1', 'Replan', 'Trigger', 'trig');
RT(n, 'RT_rp', 'Ts.gnc', 'Replan/1');
NM(n, 'Replan/1', 'rp_1s');
L(n, 'RT_rp/1', 'rp/1', 'rp');
local_log(n, 'RT_rp/1', 'rp');
local_save(n, f, sprintf(['gnc_GlobalReplanner (T15, gnc spec 4.4, F2, F7). fsm, nav and perception go through ' ...
    'Rate Transitions to Ts.plan (1 s polling); the Triggered Subsystem Replan runs on either edge of ' ...
    'fsm.replan_req and holds its Bus_ReplanResult (IC P.ic.Bus_ReplanResult). MATLAB Function ' ...
    'gnc_globalReplanner (no planner block exists): Hybrid A* on the static grid P.world.occ (Constant) merged ' ...
    'with Bus_Perception.map; LOCAL (rejoin / bypass) plans on the perception window with a lane mask and ' ...
    'R_min_local, GLOBAL (RTH) plans on the start / goal window + win_margin with R_min; inflation r_infl ' ...
    '(r_infl_min fallback), forward-only; turning primitives and Dubins arcs on terrain steeper than ' ...
    'slope_turn_max (P.world.terrain constants) cost steep_cost (no turns on slopes, D53); 100 poses [x y deg dir]; ' ...
    'ver +1 per success, fail_count +1 per ' ...
    'failure. In simulation the planner runs interpreted (coder.target Sfun -> extrinsic: its codegen path needs a ' ...
    'C compiler, D31). Rate Transition 1 -> 0.05 s; output logged as ''rp''. Parameter P.gnc.hastar.']));
end

% =============================================================================================
%  gnc_ImplementManager
% =============================================================================================
function local_gnc_ImplementManager(n, f, T)
local_new_ssr(n, f, T.(n));
% one Bus Selector per consumer, right in front of it (layout spec 6.2)
BS(n, 'FsmSel_state', {'state'}, 'fsm/1');
BS(n, 'FsmSel_sub', {'sub'}, 'fsm/1');
BS(n, 'FsmSel_hold', {'t_hold'}, 'fsm/1');
BS(n, 'PlanSel', {'implement_type'}, 'plan/1');
BS(n, 'ObsSel', {'person_near'}, 'obs/1');
TERM(n, 'mcu/1', 'T_mcu');
CMP(n, 'is_mower', '==', 'double(ImplementType.MOWER)', 'PlanSel/1');
CMP(n, 'is_tiller', '==', 'double(ImplementType.TILLER)', 'PlanSel/1');
CMP(n, 'is_sprayer', '==', 'double(ImplementType.SPRAYER)', 'PlanSel/1');
CMP(n, 'in_hold', '==', 'double(GncState.OBSTACLE_HOLD)', 'FsmSel_state/1');
CMP(n, 'is_auto', '==', 'double(GncState.AUTO_MISSION)', 'FsmSel_state/1');
CMP(n, 'on_row', '==', 'double(AutoSub.ROW)', 'FsmSel_sub/1');
CMP(n, 'hold_long', '>=', 'P.gnc.obs.t_mower_off - 1e-9', 'FsmSel_hold/1');
LOG(n, 'hold_off', 'AND', {'in_hold/1', 'hold_long/1'});
LOG(n, 'not_hold_off', 'NOT', {'hold_off/1'});
LOG(n, 'no_person', 'NOT', {'ObsSel/1'});
% mower: ROW / TURN (fsm.impl_allow), off after t_mower_off in OBSTACLE_HOLD (H4), immediately on person_near; the FSM
% keeps impl_allow false for the rest of an OBSTACLE_HOLD once person_near was seen (or the scan was lost) in it
BS(n, 'FsmSel_mower', {'impl_allow'}, 'fsm/1');
LOG(n, 'mower_on', 'AND', {'is_mower/1', 'FsmSel_mower/1', 'not_hold_off/1', 'no_person/1'});
C(n, 'mower_w', 'single(P.gnc.impl.mower_speed)', 'single');
C(n, 'zero_s', 'single(0)', 'single');
SW(n, 'mower_speed', 'mower_w/1', 'mower_on/1', 'zero_s/1');
C(n, 'deck_h', 'single(P.gnc.impl.mower_h)', 'single');
% tiller: rotor on with the implement allowance; lowered on ROW from lower_lag into the row until lift_lead
% before its end (raised in turns / transits)
BS(n, 'FsmSel_till', {'impl_allow'}, 'fsm/1');
LOG(n, 'till_on', 'AND', {'is_tiller/1', 'FsmSel_till/1', 'not_hold_off/1', 'no_person/1'});
C(n, 'till_w', 'single(P.gnc.impl.till_speed)', 'single');
SW(n, 'till_speed', 'till_w/1', 'till_on/1', 'zero_s/1');
BS(n, 'PeSel_s', {'s'}, 'pe/1');
BS(n, 'PeSel_rem', {'s_rem'}, 'pe/1');
CMP(n, 'past_lag', '>=', 'P.gnc.impl.till_lower_lag', 'PeSel_s/1');
CMP(n, 'before_lead', '>=', 'P.gnc.impl.till_lift_lead', 'PeSel_rem/1');
LOG(n, 'till_low', 'AND', {'till_on/1', 'on_row/1', 'past_lag/1', 'before_lead/1'});
C(n, 'till_d', 'single(P.gnc.impl.till_depth)', 'single');
SW(n, 'till_depth', 'till_d/1', 'till_low/1', 'zero_s/1');
% sprayer: AUTO on ROW with v >= v_min, mask = vision mask AND enable, Q = R_app v w_boom 0.006
BS(n, 'NavSel_ok', {'v'}, 'nav/1');
BS(n, 'FsmSel_spray', {'impl_allow'}, 'fsm/1');
BS(n, 'SprSel_valid', {'valid'}, 'spray/1');
CMP(n, 'v_ok', '>=', 'P.gnc.impl.v_min', 'NavSel_ok/1');
LOG(n, 'spray_on', 'AND', {'is_sprayer/1', 'is_auto/1', 'on_row/1', 'FsmSel_spray/1', 'v_ok/1', 'no_person/1', 'SprSel_valid/1'});
C(n, 'mask_all', 'uint16(P.gnc.impl.sect_all)', 'uint16');
C(n, 'mask_0', 'uint16(0)', 'uint16');
SW(n, 'mask_en', 'mask_all/1', 'spray_on/1', 'mask_0/1');
A(n, 'simulink/Logic and Bit Operations/Bitwise Operator', 'mask', 'logicop', 'AND', 'UseBitMask', 'off', 'NumInputPorts', '2');
PICK(n, 'spray/1', 'SprSel_mask', {'section_mask'}, {'mask/1'}); L(n, 'mask_en/1', 'mask/2');
A(n, 'simulink/Math Operations/Gain', 'q_rate', 'Gain', 'P.gnc.impl.R_app*P.gnc.impl.boom_w*P.gnc.impl.Q_k');
PICK(n, 'nav/1', 'NavSel_q', {'v'}, {'q_rate/1'});
C(n, 'zero_d', '0', 'double');
SW(n, 'flow', 'q_rate/1', 'spray_on/1', 'zero_d/1');
DTC(n, 'flow_s', 'single', 'flow/1');
C(n, 'ImplIC', 'P.ic.Bus_GncCmd', 'Bus: Bus_GncCmd');
BA(n, 'Impl', 'ImplIC/1', {'mower_enable', 'mower_speed_cmd', 'deck_height_cmd', 'tiller_enable', 'tiller_speed_cmd', ...
    'tiller_depth_cmd', 'spray_enable', 'spray_section_mask', 'spray_flow_cmd'}, ...
    {'mower_on/1', 'mower_speed/1', 'deck_h/1', 'till_on/1', 'till_speed/1', 'till_depth/1', 'spray_on/1', ...
     'mask/1', 'flow_s/1'});
L(n, 'Impl/1', 'impl/1', 'impl');
local_save(n, f, sprintf(['gnc_ImplementManager (T15, gnc spec 2.9, blocks). impl = Bus_GncCmd with the implement ' ...
    'fields (CommandShaping assigns motion / mode fields on top). Mower (plan.implement_type MOWER): blades on with ' ...
    'fsm.impl_allow (AUTO on ROW / TURN, or OBSTACLE_HOLD entered there until person_near is seen or the scan is ' ...
    'lost in it: then false for the rest of the hold, spec 2.9) and not after P.gnc.obs.t_mower_off in ' ...
    'OBSTACLE_HOLD (H4) and never with person_near; speed P.gnc.impl.mower_speed, deck P.gnc.impl.mower_h. ' ...
    'Tiller: rotor on with impl_allow, lowered to till_depth on ROW once pe.s >= till_lower_lag and while pe.s_rem ' ...
    '>= till_lift_lead (raised in turns / transits). Sprayer: AUTO on ROW with v >= impl.v_min, valid vision ' ...
    'demand and no person: spray_section_mask = spray.section_mask AND (enable * sect_all), Q = R_app v w_boom ' ...
    '0.006 L/min. The FSM keeps v_cap = 0 until mcu.implement_ready (10 s timeout -> PAUSE).']));
end

% =============================================================================================
%  gnc_StatusMetrics
% =============================================================================================
function local_gnc_StatusMetrics(n, f, T)
local_new_ssr(n, f, T.(n));
% the fields of fsm: one Bus Selector per consumer, right in front of it (layout spec 6.2). The fields of pe, nav and plan:
% one Bus Selector each: with picks per consumer the trunks of the four branching port lines share one track (80 line
% overlaps, which only an engine change would remove; task 13 decisions 7)
BS(n, 'PeSel', {'xte', 'epsi', 'v_ref', 's'}, 'pe/1');
BS(n, 'NavSel', {'nav_ok', 'pose'}, 'nav/1');
BS(n, 'PlanSel', {'mission_type', 'mission_id', 'n_seg', 'seg', 'home', 'area_m2', 'speed_work'}, 'plan/1');
BS(n, 'RpSel', {'ver'}, 'rp/1');
% ---- 10 s moving RMS of the cross-track error on ROW segments (held otherwise) ----
BS(n, 'FsmSel_sub', {'sub'}, 'fsm/1');
BS(n, 'FsmSel_state', {'state'}, 'fsm/1');
CMP(n, 'on_row', '==', 'double(AutoSub.ROW)', 'FsmSel_sub/1');
CMP(n, 'is_auto', '==', 'double(GncState.AUTO_MISSION)', 'FsmSel_state/1');
LOG(n, 'rms_en', 'AND', {'on_row/1', 'is_auto/1'});
e = [n '/RowRms'];
A(n, 'simulink/Ports & Subsystems/Enabled Subsystem', 'RowRms');
local_clear(e);
A(e, 'simulink/Sources/In1', 'xte', 'Port', '1');
A(e, 'simulink/Ports & Subsystems/Enable', 'Enable', 'StatesWhenEnabling', 'held');
A(e, 'simulink/Sinks/Out1', 'rms', 'Port', '1', 'InitialOutput', '0', 'OutputWhenDisabled', 'held');
A(e, 'dspstat3/Moving RMS', 'MovingRMS', 'Method', 'Sliding window', 'SpecifyWindowLength', 'on', ...
    'WindowLength', 'P.gnc.met.rms_window', 'OverlapLength', 'P.gnc.met.rms_window - 1', ...
    'SimulateUsing', 'Code generation');
L(e, 'xte/1', 'MovingRMS/1'); L(e, 'MovingRMS/1', 'rms/1', 'rms');
L(n, 'PeSel/1', 'RowRms/1');
LP(n, 'rms_en/1', 'RowRms', 'Enable', 'rms_en');
% ---- progress along the driven segments: (sum of completed lengths + s) / sum(1 .. n_seg - 1) ----
SEL(n, 'seg_len', 'Select all,Index vector (dialog)', {''; '3'}, 'PlanSel/4');
DTC(n, 'seg_len_d', 'double', 'seg_len/1');
A(n, 'dspmathops/Cumulative Sum', 'cum_len', 'dim', 'Columns');
L(n, 'seg_len_d/1', 'cum_len/1');
C(n, 'zero', '0', 'double');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'cum0', 'NumInputs', '2', 'Mode', 'Vector');
L(n, 'zero/1', 'cum0/1'); L(n, 'cum_len/1', 'cum0/2');              % cum0(k) = length of segments 1 .. k-1
BS(n, 'FsmSel_seg', {'seg_idx'}, 'fsm/1');
DTC(n, 'seg_d', 'double', 'FsmSel_seg/1');
SEL(n, 'done_seg', 'Index vector (port)', {''}, 'cum0/1', 'seg_d/1');
DTC(n, 'nseg_d', 'double', 'PlanSel/3');
A(n, 'simulink/Discontinuities/Saturation', 'nseg_c', 'UpperLimit', 'P.gnc.path.S_max', 'LowerLimit', '1');
L(n, 'nseg_d/1', 'nseg_c/1');
SEL(n, 'total', 'Index vector (port)', {''}, 'cum0/1', 'nseg_c/1');  % = length of segments 1 .. n_seg - 1
BS(n, 'FsmSel_src', {'path_src'}, 'fsm/1');
CMP(n, 'src_mission', '==', 'double(PathSrc.MISSION)', 'FsmSel_src/1');
SW(n, 's_mis', 'PeSel/4', 'src_mission/1', 'zero/1');
% s within [0, length of the current segment] (the analyser extends the polyline ends)
A(n, 'simulink/Math Operations/Bias', 'seg_p1', 'Bias', '1');  L(n, 'seg_d/1', 'seg_p1/1');
SEL(n, 'cum_k', 'Index vector (port)', {''}, 'cum0/1', 'seg_p1/1');
A(n, 'simulink/Math Operations/Sum', 'len_k', 'Inputs', '+-');  L(n, 'cum_k/1', 'len_k/1'); L(n, 'done_seg/1', 'len_k/2');
A(n, 'simulink/Discontinuities/Saturation Dynamic', 's_pos');
L(n, 'len_k/1', 's_pos/1'); L(n, 's_mis/1', 's_pos/2'); L(n, 'zero/1', 's_pos/3');
A(n, 'simulink/Math Operations/Sum', 'done', 'Inputs', '++');  L(n, 'done_seg/1', 'done/1'); L(n, 's_pos/1', 'done/2');
A(n, 'simulink/Discontinuities/Saturation', 'total_p', 'UpperLimit', 'inf', 'LowerLimit', '1e-6');
L(n, 'total/1', 'total_p/1');
A(n, 'simulink/Math Operations/Divide', 'ratio');  L(n, 'done/1', 'ratio/1'); L(n, 'total_p/1', 'ratio/2');
C(n, 'one', '1', 'double');
BS(n, 'FsmSel_prog', {'mission_done'}, 'fsm/1');
SW(n, 'prog_done', 'one/1', 'FsmSel_prog/1', 'ratio/1');
A(n, 'simulink/Discontinuities/Saturation', 'progress', 'UpperLimit', '1', 'LowerLimit', '0');
L(n, 'prog_done/1', 'progress/1');
DTC(n, 'area_d', 'double', 'PlanSel/6');
A(n, 'simulink/Math Operations/Product', 'area_done', 'Inputs', '**');  L(n, 'progress/1', 'area_done/1'); L(n, 'area_d/1', 'area_done/2');
% ETA = remaining length / max(speed_work, v_eta_min)
A(n, 'simulink/Math Operations/Sum', 'remain', 'Inputs', '+-');  L(n, 'total/1', 'remain/1'); L(n, 'done/1', 'remain/2');
A(n, 'simulink/Discontinuities/Saturation', 'remain_p', 'UpperLimit', 'inf', 'LowerLimit', '0');  L(n, 'remain/1', 'remain_p/1');
BS(n, 'FsmSel_rem', {'mission_done'}, 'fsm/1');
SW(n, 'remain_d', 'zero/1', 'FsmSel_rem/1', 'remain_p/1');
DTC(n, 'vw_d', 'double', 'PlanSel/7');
A(n, 'simulink/Discontinuities/Saturation', 'vw_p', 'UpperLimit', 'inf', 'LowerLimit', 'P.gnc.met.v_eta_min');  L(n, 'vw_d/1', 'vw_p/1');
A(n, 'simulink/Math Operations/Divide', 'eta');  L(n, 'remain_d/1', 'eta/1'); L(n, 'vw_p/1', 'eta/2');
% distance to home
SEL(n, 'xy', 'Index vector (dialog)', {'[1 2]'}, 'NavSel/2');
SEL(n, 'home_xy', 'Index vector (dialog)', {'[1 2]'}, 'PlanSel/5');
A(n, 'simulink/Math Operations/Sum', 'dxy', 'Inputs', '+-');  L(n, 'xy/1', 'dxy/1'); L(n, 'home_xy/1', 'dxy/2');
A(n, 'simulink/Math Operations/Product', 'dxy2', 'Inputs', '**');  L(n, 'dxy/1', 'dxy2/1'); L(n, 'dxy/1', 'dxy2/2');
A(n, 'simulink/Math Operations/Sum of Elements', 'd2');  L(n, 'dxy2/1', 'd2/1');
A(n, 'simulink/Math Operations/Sqrt', 'dist_home');  L(n, 'd2/1', 'dist_home/1');
% ---- Bus_GncStatus ----
BS(n, 'FsmSel_thold', {'t_hold'}, 'fsm/1');
sg = {'xte_s', 'PeSel/1'; 'epsi_s', 'PeSel/2'; 'vref_s', 'PeSel/3'; 'soc_s', 'soc_rth/1'; 'thold_s', 'FsmSel_thold/1'; ...
      'prog_s', 'progress/1'; 'area_s', 'area_done/1'; 'rms_s', 'RowRms/1'; 'home_s', 'dist_home/1'; 'eta_s', 'eta/1'};
for k = 1:size(sg, 1), DTC(n, sg{k, 1}, 'single', sg{k, 2}); end
C(n, 'StatusIC', 'P.ic.Bus_GncStatus', 'Bus: Bus_GncStatus');
BS(n, 'Status_state', {'state', 'sub'}, 'fsm/1');
BS(n, 'Status_seg', {'seg_idx'}, 'fsm/1');
BS(n, 'Status_fault', {'fault_word', 'hold_reason', 'pause_reason'}, 'fsm/1');
BS(n, 'Status_trk', {'trk_sel'}, 'fsm/1');
BS(n, 'Status_done', {'mission_done'}, 'fsm/1');
BA(n, 'Status', 'StatusIC/1', {'gnc_state', 'sub_mode', 'mission_type', 'mission_id', 'seg_idx', 'n_seg', 'progress', ...
    'area_done_m2', 'xte', 'heading_err', 'xte_rms', 'v_ref', 'dist_home', 'soc_rth', 'fault_word', 'hold_reason', ...
    'pause_reason', 'hold_timer', 'replan_count', 'tracker_active', 'nav_ok', 'eta_s', 'pose_est', 'mission_done'}, ...
    {'Status_state/1', 'Status_state/2', 'PlanSel/1', 'PlanSel/2', 'Status_seg/1', 'PlanSel/3', 'prog_s/1', 'area_s/1', 'xte_s/1', ...
     'epsi_s/1', 'rms_s/1', 'vref_s/1', 'home_s/1', 'soc_s/1', 'Status_fault/1', 'Status_fault/2', 'Status_fault/3', 'thold_s/1', ...
     'RpSel/1', 'Status_trk/1', 'NavSel/1', 'eta_s/1', 'NavSel/2', 'Status_done/1'});
RT(n, 'RT_status', 'Ts.gnc_status', 'Status/1');
L(n, 'RT_status/1', 'gnc_status/1', 'gnc_status');
L(n, 'dbg/1', 'gnc_debug/1', 'gnc_debug');
local_save(n, f, sprintf(['gnc_StatusMetrics (T15, gnc spec 5.2, build plan T15). xte_rms = dspstat3/Moving RMS ' ...
    '(sliding window P.gnc.met.rms_window = 200 samples, 10 s at 0.05 s) of pe.xte inside an Enabled Subsystem ' ...
    'active in AUTO on ROW (held otherwise); progress = (cumulative length of the completed segments + s on the ' ...
    'mission path) / length of segments 1 .. n_seg - 1 (dspmathops Cumulative Sum, 1 when mission_done); ' ...
    'area_done = progress area_m2; dist_home = |pose - home|; eta = remaining length / max(speed_work, ' ...
    'v_eta_min); replan_count = rp.ver; other fields from fsm / pe / nav / plan / soc_rth. Bus_GncStatus through a ' ...
    'Rate Transition 0.05 -> 0.2 s; gnc_debug = dbg (Bus_GncDebug assembled in Shaddad_GNC).']));
end

% =============================================================================================
%  tests/gnc/tb_gnc_mission (bench)
% =============================================================================================
function local_tb(name, f)
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')')
    shaddad_configsets('fixed');
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
set_param(name, 'StopTime', '10', 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'SaveTime', 'on', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
n = name;
ts = 'Ts.ctrl';  tg = 'Ts.gnc';
FW = @(nm, var, tsx, interp) A(n, 'simulink/Sources/From Workspace', nm, 'VariableName', var, 'SampleTime', tsx, ...
    'Interpolate', interp, 'OutputAfterFinalValue', 'Holding final value');
%% ---- device under test ----
A(n, 'simulink/Ports & Subsystems/Model', 'GNC', 'ModelName', 'Shaddad_GNC', 'SimulationMode', 'Normal');
%% ---- scenario tables ----
FW('scn_mcu', 'P.gnc.tb.scn_mcu', tg, 'off');
A(n, 'simulink/Signal Routing/Demux', 'mcu_d', 'Outputs', '7');  L(n, 'scn_mcu/1', 'mcu_d/1');
FW('scn_cmd', 'P.gnc.tb.scn_cmd', 'Ts.mcmd', 'off');
A(n, 'simulink/Signal Routing/Demux', 'cmd_d', 'Outputs', '2');  L(n, 'scn_cmd/1', 'cmd_d/1');
FW('scn_bms', 'P.gnc.tb.scn_bms', 'Ts.hk', 'on');
A(n, 'simulink/Signal Routing/Demux', 'bms_d', 'Outputs', '2');  L(n, 'scn_bms/1', 'bms_d/1');
FW('scn_nav', 'P.gnc.tb.scn_nav', ts, 'off');
A(n, 'simulink/Signal Routing/Demux', 'nav_d', 'Outputs', '5');  L(n, 'scn_nav/1', 'nav_d/1');
FW('scn_plant', 'P.gnc.tb.scn_plant', ts, 'off');
% MCU drive gating: commands reach the plant only with drive_ready and no E-stop
CMP(n, 'drv_rdy', '>', '0.5', 'mcu_d/3');
CMP(n, 'estop', '>', '0.5', 'mcu_d/4');
LOG(n, 'no_estop', 'NOT', {'estop/1'});
LOG(n, 'drive_ok', 'AND', {'drv_rdy/1', 'no_estop/1'});
A(n, 'simulink/Signal Attributes/Rate Transition', 'RT_drive', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', ts);
L(n, 'drive_ok/1', 'RT_drive/1');
%% ---- command path: RT -> dead time -> MCU AUTO shaper -> lags (as tb_gnc) ----
BS(n, 'CmdSel', {'v_cmd', 'omega_cmd'}, 'GNC/1');
for s = {'v', 'w'}
    x = s{1};  k = 1 + strcmp(x, 'w');
    DTC(n, ['cmd_' x], 'double', sprintf('CmdSel/%d', k));
    RT(n, ['RT_' x], ts, ['cmd_' x '/1']);
    A(n, 'simulink/Discrete/Delay', ['dead_' x], 'DelayLength', 'P.gnc.tb.n_dead', 'InitialCondition', '0', 'SampleTime', ts);
    L(n, ['RT_' x '/1'], ['dead_' x '/1']);
    C(n, ['z_' x], '0', 'double', ts);
    SW(n, ['gate_' x], ['dead_' x '/1'], 'RT_drive/1', ['z_' x '/1']);
end
A(n, 'simulink/Math Operations/Sum', 'ev', 'Inputs', '+-');
A(n, 'simulink/Math Operations/Gain', 'Kv', 'Gain', 'P.electronics.shaper.K_v');
A(n, 'simulink/Discontinuities/Saturation', 'a_sat', 'UpperLimit', 'P.electronics.shaper.a_acc_auto', ...
    'LowerLimit', '-P.electronics.shaper.a_dec_auto');
A(n, 'simulink/Discontinuities/Rate Limiter', 'jerk_v', 'RisingSlewLimit', 'P.electronics.shaper.j_max', ...
    'FallingSlewLimit', '-P.electronics.shaper.j_max', 'SampleTimeMode', 'inherited');
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'v_ref', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'SampleTime', ts, 'InitialCondition', '0');
L(n, 'gate_v/1', 'ev/1'); L(n, 'v_ref/1', 'ev/2', 'v_ref'); L(n, 'ev/1', 'Kv/1');
L(n, 'Kv/1', 'a_sat/1'); L(n, 'a_sat/1', 'jerk_v/1'); L(n, 'jerk_v/1', 'v_ref/1');
A(n, 'simulink/Math Operations/Sum', 'ew', 'Inputs', '+-');
A(n, 'simulink/Math Operations/Gain', 'Kw', 'Gain', 'P.electronics.shaper.K_w_auto');
A(n, 'simulink/Discontinuities/Saturation', 'al_sat', 'UpperLimit', 'P.electronics.shaper.alpha_auto', ...
    'LowerLimit', '-P.electronics.shaper.alpha_auto');
A(n, 'simulink/Discontinuities/Rate Limiter', 'jerk_w', 'RisingSlewLimit', 'P.electronics.shaper.jw_auto', ...
    'FallingSlewLimit', '-P.electronics.shaper.jw_auto', 'SampleTimeMode', 'inherited');
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'w_ref', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'SampleTime', ts, 'InitialCondition', '0');
L(n, 'gate_w/1', 'ew/1'); L(n, 'w_ref/1', 'ew/2', 'w_ref'); L(n, 'ew/1', 'Kw/1');
L(n, 'Kw/1', 'al_sat/1'); L(n, 'al_sat/1', 'jerk_w/1'); L(n, 'jerk_w/1', 'w_ref/1');
A(n, 'simulink/Discrete/Transfer Fcn First Order', 'lag_v', 'PoleZ', 'exp(-Ts.ctrl/P.gnc.tb.tau_v)');
A(n, 'simulink/Discrete/Transfer Fcn First Order', 'lag_r', 'PoleZ', 'exp(-Ts.ctrl/P.gnc.tb.tau_r)');
L(n, 'v_ref/1', 'lag_v/1'); L(n, 'w_ref/1', 'lag_r/1');
% stuck: the tracks are blocked (plant speed x v_factor)
A(n, 'simulink/Math Operations/Product', 'v_true', 'Inputs', '**');  L(n, 'lag_v/1', 'v_true/1'); L(n, 'scn_plant/1', 'v_true/2');
%% ---- kinematic plant + lateral drift ----
A(n, 'robotmobilelib/Differential Drive Kinematic Model', 'Kinematics', 'VehicleInputs', ...
    'Vehicle Speed & Heading Angular Velocity', 'TrackWidth', 'P.structure.geom.B', ...
    'WheelRadius', 'P.structure.r_sprocket', 'InitialState', 'P.gnc.tb.x0(:)', 'SimulateUsing', 'Code generation');
A(n, 'simulink/Signal Attributes/Rate Transition', 'RT_vc', 'OutPortSampleTime', '0', 'Integrity', 'on', 'Deterministic', 'on');
A(n, 'simulink/Signal Attributes/Rate Transition', 'RT_rc', 'OutPortSampleTime', '0', 'Integrity', 'on', 'Deterministic', 'on');
L(n, 'v_true/1', 'RT_vc/1', 'v_true'); L(n, 'lag_r/1', 'RT_rc/1', 'r_true');
L(n, 'RT_vc/1', 'Kinematics/1'); L(n, 'RT_rc/1', 'Kinematics/2');
TERM(n, 'Kinematics/2', 'T_stateDot');
A(n, 'simulink/Signal Routing/Demux', 'state', 'Outputs', '3');  L(n, 'Kinematics/1', 'state/1', 'state');
RT(n, 'ZOH_psi', ts, 'state/3');
A(n, 'simulink/Math Operations/Trigonometric Function', 'sin_psi', 'Operator', 'sin');  L(n, 'ZOH_psi/1', 'sin_psi/1');
A(n, 'simulink/Math Operations/Trigonometric Function', 'cos_psi', 'Operator', 'cos');  L(n, 'ZOH_psi/1', 'cos_psi/1');
A(n, 'simulink/Math Operations/Gain', 'drift_x', 'Gain', '-P.gnc.tb.drift');  L(n, 'sin_psi/1', 'drift_x/1');
A(n, 'simulink/Math Operations/Gain', 'drift_y', 'Gain', 'P.gnc.tb.drift');   L(n, 'cos_psi/1', 'drift_y/1');
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'dx', 'IntegratorMethod', 'Integration: Forward Euler', 'SampleTime', ts);
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'dy', 'IntegratorMethod', 'Integration: Forward Euler', 'SampleTime', ts);
L(n, 'drift_x/1', 'dx/1'); L(n, 'drift_y/1', 'dy/1');
RT(n, 'ZOH_x', ts, 'state/1'); RT(n, 'ZOH_y', ts, 'state/2');
A(n, 'simulink/Math Operations/Sum', 'x_true', 'Inputs', '++');  L(n, 'ZOH_x/1', 'x_true/1'); L(n, 'dx/1', 'x_true/2');
A(n, 'simulink/Math Operations/Sum', 'y_true', 'Inputs', '++');  L(n, 'ZOH_y/1', 'y_true/1'); L(n, 'dy/1', 'y_true/2');
A(n, 'simulink/Math Operations/Bias', 'psi_b', 'Bias', 'pi');
A(n, 'simulink/Math Operations/Gain', 'psi_n', 'Gain', '1/(2*pi)');
A(n, 'simulink/Math Operations/Rounding Function', 'psi_f', 'Operator', 'floor');
A(n, 'simulink/Math Operations/Gain', 'psi_2pi', 'Gain', '2*pi');
A(n, 'simulink/Math Operations/Sum', 'psi_w', 'Inputs', '+-');
L(n, 'ZOH_psi/1', 'psi_b/1'); L(n, 'psi_b/1', 'psi_n/1'); L(n, 'psi_n/1', 'psi_f/1');
L(n, 'psi_f/1', 'psi_2pi/1'); L(n, 'ZOH_psi/1', 'psi_w/1'); L(n, 'psi_2pi/1', 'psi_w/2');
%% ---- navigation surrogate (Bus_NavState, 0.01 s): RTK noise, scenario validity / sigma / quality / offset ----
noise = {'n_x', 'P.gnc.tb.sig_pos^2', 'P.gnc.tb.seed_x'; 'n_y', 'P.gnc.tb.sig_pos^2', 'P.gnc.tb.seed_y'; ...
         'n_psi', 'P.gnc.tb.sig_yaw^2', 'P.gnc.tb.seed_psi'};
for k = 1:3
    A(n, 'simulink/Sources/Random Number', noise{k, 1}, 'Mean', '0', 'Variance', noise{k, 2}, 'Seed', noise{k, 3}, 'SampleTime', ts);
end
A(n, 'simulink/Math Operations/Sum', 'x_meas', 'Inputs', '+++');
L(n, 'x_true/1', 'x_meas/1', 'x'); L(n, 'n_x/1', 'x_meas/2'); L(n, 'nav_d/4', 'x_meas/3');
A(n, 'simulink/Math Operations/Sum', 'y_meas', 'Inputs', '+++');
L(n, 'y_true/1', 'y_meas/1', 'y'); L(n, 'n_y/1', 'y_meas/2'); L(n, 'nav_d/5', 'y_meas/3');
A(n, 'simulink/Math Operations/Sum', 'psi_meas', 'Inputs', '++');  L(n, 'psi_w/1', 'psi_meas/1', 'psi_w'); L(n, 'n_psi/1', 'psi_meas/2');
C(n, 'zero', '0', 'double', ts);
A(n, 'simulink/Signal Routing/Mux', 'pos_W', 'Inputs', '3');
L(n, 'x_meas/1', 'pos_W/1'); L(n, 'y_meas/1', 'pos_W/2'); L(n, 'zero/1', 'pos_W/3');
C(n, 'tilt', 'P.gnc.tb.tilt(:)', 'double', ts);
A(n, 'simulink/Signal Routing/Mux', 'eul', 'Inputs', '[1 2]');  L(n, 'psi_meas/1', 'eul/1'); L(n, 'tilt/1', 'eul/2');
A(n, 'simulink/Signal Routing/Mux', 'vel_B', 'Inputs', '3');
L(n, 'v_true/1', 'vel_B/1'); L(n, 'zero/1', 'vel_B/2'); L(n, 'zero/1', 'vel_B/3');
% sigma: scenario value when > 0, else nominal
C(n, 'sig_nom', 'P.gnc.tb.sig_pos', 'double', ts);
SW(n, 'sig', 'nav_d/2', 'nav_d/2', 'sig_nom/1', 'u2 > Threshold');
set_param([n '/sig'], 'Threshold', '0');
A(n, 'simulink/Signal Routing/Mux', 'posStd', 'Inputs', '3');
A(n, 'simulink/Math Operations/Gain', 'sig_z', 'Gain', '2');  L(n, 'sig/1', 'sig_z/1');
L(n, 'sig/1', 'posStd/1'); L(n, 'sig/1', 'posStd/2'); L(n, 'sig_z/1', 'posStd/3');
DTC(n, 'quality', 'uint8', 'nav_d/3');
CMP(n, 'valid', '>', '0.5', 'nav_d/1');
A(n, 'simulink/Sources/Digital Clock', 'clock', 'SampleTime', ts);
C(n, 'NavIC', 'P.ic.Bus_NavState', 'Bus: Bus_NavState', ts);
BA(n, 'NavState', 'NavIC/1', {'t', 'pos_W', 'eul', 'vel_B', 'yawRate', 'posStd', 'quality', 'navValid', ...
    'headingValid', 'aligned', 'gnssFixType'}, {'clock/1', 'pos_W/1', 'eul/1', 'vel_B/1', 'lag_r/1', 'posStd/1', ...
    'quality/1', 'valid/1', 'valid/1', 'valid/1', 'quality/1'});
L(n, 'NavState/1', 'GNC/1', 'nav_state');
%% ---- MCU status surrogate (0.05 s) ----
A(n, 'simulink/Math Operations/Gain', 'w_track', 'Gain', 'P.electronics.mix.chi*P.electronics.mix.B/2');  L(n, 'lag_r/1', 'w_track/1');
A(n, 'simulink/Math Operations/Sum', 'vL', 'Inputs', '+-');  L(n, 'v_true/1', 'vL/1'); L(n, 'w_track/1', 'vL/2');
A(n, 'simulink/Math Operations/Sum', 'vR', 'Inputs', '++');  L(n, 'v_true/1', 'vR/1'); L(n, 'w_track/1', 'vR/2');
A(n, 'simulink/Signal Routing/Mux', 'vLR', 'Inputs', '2');  L(n, 'vL/1', 'vLR/1'); L(n, 'vR/1', 'vLR/2');
RT(n, 'RT_vLR', tg, 'vLR/1');  DTC(n, 'vLR_s', 'single', 'RT_vLR/1');
RT(n, 'RT_vapp', tg, 'v_ref/1');  DTC(n, 'vapp_s', 'single', 'RT_vapp/1');
% heartbeat (frozen while scn freeze = 1)
A(n, 'simulink/Sources/Counter Free-Running', 'hb_cnt', 'NumBits', '8', 'tsamp', tg);
A(n, 'simulink/Discrete/Unit Delay', 'hb_z', 'InitialCondition', 'uint8(0)', 'SampleTime', tg);
CMP(n, 'freeze', '>', '0.5', 'mcu_d/7');
% frozen from t_f on: the frame of t_f is the last fresh one (delayed flag)
A(n, 'simulink/Discrete/Unit Delay', 'freeze_z', 'InitialCondition', 'false', 'SampleTime', tg);  L(n, 'freeze/1', 'freeze_z/1');
SW(n, 'hb', 'hb_z/1', 'freeze_z/1', 'hb_cnt/1');
L(n, 'hb/1', 'hb_z/1');
% implement ready: mode 0 = any implement enable held for t_spin, 1 = false, 2 = true
BS(n, 'ImplSel', {'mower_enable', 'tiller_enable', 'spray_enable'}, 'GNC/1');
LOG(n, 'impl_en', 'OR', {'ImplSel/1', 'ImplSel/2', 'ImplSel/3'});
A(n, 'simulink/Discrete/Unit Delay', 'impl_en_z', 'InitialCondition', 'false', 'SampleTime', tg);
L(n, 'impl_en/1', 'impl_en_z/1');
DTC(n, 'impl_en_d', 'double', 'impl_en_z/1');
LOG(n, 'impl_off', 'NOT', {'impl_en_z/1'});
A(n, 'simulink/Discrete/Discrete-Time Integrator', 'spin_t', 'IntegratorMethod', 'Integration: Forward Euler', ...
    'SampleTime', tg, 'ExternalReset', 'level', 'InitialCondition', '0');
L(n, 'impl_en_d/1', 'spin_t/1'); L(n, 'impl_off/1', 'spin_t/2');
CMP(n, 'spun', '>=', 'P.gnc.tb.t_spin', 'spin_t/1');
DTC(n, 'spun_d', 'double', 'spun/1');
C(n, 'f0', '0', 'double', tg);  C(n, 'f1', '1', 'double', tg);
A(n, 'simulink/Signal Routing/Multiport Switch', 'impl_rdy_m', 'Inputs', '3', 'DataPortOrder', 'Zero-based contiguous', ...
    'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
L(n, 'mcu_d/6', 'impl_rdy_m/1'); L(n, 'spun_d/1', 'impl_rdy_m/2'); L(n, 'f0/1', 'impl_rdy_m/3'); L(n, 'f1/1', 'impl_rdy_m/4');
CMP(n, 'impl_rdy', '>', '0.5', 'impl_rdy_m/1');
% charging: 0 off, 1 on, 2 within dock_tol of plan.dock and |v| < 0.05
RT(n, 'RT_xd', tg, 'x_true/1'); RT(n, 'RT_yd', tg, 'y_true/1'); RT(n, 'RT_vd', tg, 'v_true/1');
A(n, 'simulink/Signal Routing/Mux', 'xy_t', 'Inputs', '2');  L(n, 'RT_xd/1', 'xy_t/1'); L(n, 'RT_yd/1', 'xy_t/2');
C(n, 'dock_xy', 'P.gnc.mission.dock(1:2)', 'double', tg);
A(n, 'simulink/Math Operations/Sum', 'ddock', 'Inputs', '+-');  L(n, 'xy_t/1', 'ddock/1'); L(n, 'dock_xy/1', 'ddock/2');
A(n, 'simulink/Math Operations/Math Function', 'ddock2', 'Operator', 'magnitude^2');  L(n, 'ddock/1', 'ddock2/1');
A(n, 'simulink/Math Operations/Sum of Elements', 'ddock_s');  L(n, 'ddock2/1', 'ddock_s/1');
CMP(n, 'near_dock', '<', 'P.gnc.tb.dock_tol^2', 'ddock_s/1');
A(n, 'simulink/Math Operations/Abs', 'v_abs');  L(n, 'RT_vd/1', 'v_abs/1');
CMP(n, 'stopped', '<', '0.05', 'v_abs/1');
LOG(n, 'at_dock', 'AND', {'near_dock/1', 'stopped/1'});
DTC(n, 'at_dock_d', 'double', 'at_dock/1');
A(n, 'simulink/Signal Routing/Multiport Switch', 'chg_m', 'Inputs', '3', 'DataPortOrder', 'Zero-based contiguous', ...
    'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
L(n, 'mcu_d/5', 'chg_m/1'); L(n, 'f0/1', 'chg_m/2'); L(n, 'f1/1', 'chg_m/3'); L(n, 'at_dock_d/1', 'chg_m/4');
CMP(n, 'charging', '>', '0.5', 'chg_m/1');
DTC(n, 'rc_sw', 'uint8', 'mcu_d/1');
CMP(n, 'rc_link', '>', '0.5', 'mcu_d/2');
C(n, 'mcu_true', 'true', 'boolean', tg);
C(n, 'impl_type', 'P.gnc.mission.implement_type', 'uint8', tg);
C(n, 'McuIC', 'P.ic.Bus_McuStatus', 'Bus: Bus_McuStatus', tg);
BA(n, 'McuStatus', 'McuIC/1', {'rc_mode_sw', 'rc_link_ok', 'estop_active', 'drive_ready', 'power_ready', ...
    'implement_ready', 'charging', 'implement_type', 'heartbeat', 'v_track_meas', 'v_applied', 'valid'}, ...
    {'rc_sw/1', 'rc_link/1', 'estop/1', 'drive_ok/1', 'mcu_true/1', 'impl_rdy/1', 'charging/1', 'impl_type/1', ...
     'hb/1', 'vLR_s/1', 'vapp_s/1', 'mcu_true/1'});
L(n, 'McuStatus/1', 'GNC/3', 'mcu_status');
%% ---- BMS, mission command, spray ----
DTC(n, 'soc_s', 'single', 'bms_d/1');
CMP(n, 'bms_crit', '>', '0.5', 'bms_d/2');
C(n, 'bms_true', 'true', 'boolean', 'Ts.hk');
RT(n, 'RT_chg', 'Ts.hk', 'charging/1');
C(n, 'BmsIC', 'P.ic.Bus_BmsStatus', 'Bus: Bus_BmsStatus', 'Ts.hk');
BA(n, 'BmsStatus', 'BmsIC/1', {'SOC', 'bms_critical', 'charging', 'valid'}, {'soc_s/1', 'bms_crit/1', 'RT_chg/1', 'bms_true/1'});
L(n, 'BmsStatus/1', 'GNC/4', 'bms_status');
DTC(n, 'cmd_u8', 'uint8', 'cmd_d/1');
DTC(n, 'seq_u8', 'uint8', 'cmd_d/2');
C(n, 'CmdIC', 'P.ic.Bus_MissionCmd', 'Bus: Bus_MissionCmd', 'Ts.mcmd');
BA(n, 'MissionCmd', 'CmdIC/1', {'cmd', 'cmd_seq'}, {'cmd_u8/1', 'seq_u8/1'});
L(n, 'MissionCmd/1', 'GNC/5', 'mcmd');
C(n, 'SprayIC', 'P.ic.Bus_SprayDemand', 'Bus: Bus_SprayDemand', 'Ts.spray');
C(n, 'spr_mask', 'uint16(P.gnc.tb.spray_mask)', 'uint16', 'Ts.spray');
C(n, 'spr_dens', 'single(P.gnc.tb.spray_density)', 'single', 'Ts.spray');
C(n, 'spr_valid', 'true', 'boolean', 'Ts.spray');
BA(n, 'Spray', 'SprayIC/1', {'section_mask', 'target_density', 'valid'}, {'spr_mask/1', 'spr_dens/1', 'spr_valid/1'});
L(n, 'Spray/1', 'GNC/6', 'spray');
%% ---- perception surrogate: harness LiDAR (0.1 s) ----
tl = 'Ts.lidar';
RT(n, 'RT_px', tl, 'x_true/1'); RT(n, 'RT_py', tl, 'y_true/1'); RT(n, 'RT_ppsi', tl, 'psi_w/1'); RT(n, 'RT_pv', tl, 'v_true/1');
A(n, 'simulink/Signal Routing/Mux', 'pose_l', 'Inputs', '3');
L(n, 'RT_px/1', 'pose_l/1'); L(n, 'RT_py/1', 'pose_l/2'); L(n, 'RT_ppsi/1', 'pose_l/3');
A(n, 'simulink/Sources/Digital Clock', 'clock_l', 'SampleTime', tl);
C(n, 'occ', 'P.world.occ.grid', 'boolean', 'inf');
C(n, 'dyn', 'P.gnc.tb.dyn', 'double', 'inf');
prm = ['struct(''res'',P.world.occ.res,''origin'',P.world.occ.origin,''rmin'',P.sensors.lidar.rMin,' ...
    '''rmax'',P.gnc.tb.lidar_rmax,''nBeams'',P.gnc.obs.n_beams,''on'',P.gnc.tb.lidar_on,' ...
    '''protXF'',P.sensors.per.protXF,''protXR'',P.sensors.per.protXR,''protL0'',P.sensors.per.protL0,' ...
    '''protKv'',P.sensors.per.protKv,''protW'',P.sensors.per.protW,''protStatic'',P.gnc.tb.prot_static,' ...
    '''mapN'',P.gnc.map.local_N,''mapRes'',P.gnc.map.local_res)'];
MFB(n, 'HarnessLidar', sprintf(['function [ranges, angles, prot, protLen, occL, gl] = fcn(pose, v, t, occ, dyn, prm)\n' ...
    '%%#codegen\n[ranges, angles, prot, protLen, occL, gl] = gnc_harnessLidar(pose, v, t, occ, dyn, prm);\n']), ...
    cell(0, 2), {'prm', prm}, tl, 'Harness LiDAR (gnc spec 4.5, tests only); body subsystems/gnc/gnc_harnessLidar.m.');
L(n, 'pose_l/1', 'HarnessLidar/1'); L(n, 'RT_pv/1', 'HarnessLidar/2'); L(n, 'clock_l/1', 'HarnessLidar/3');
L(n, 'occ/1', 'HarnessLidar/4'); L(n, 'dyn/1', 'HarnessLidar/5');
C(n, 'map_res', 'P.gnc.map.local_res', 'double', tl);
C(n, 'map_size', '[1 1]*P.gnc.map.local_N', 'double', tl);
C(n, 'map_org', '-[1 1]*P.gnc.map.local_N/(2*P.gnc.map.local_res)', 'double', tl);
set_param([n '/map_size'], 'VectorParams1D', 'off'); set_param([n '/map_org'], 'VectorParams1D', 'off');
BC(n, 'Map', 'Bus_TebMapInfo', {'Resolution', 'GridLocationInWorld', 'GridSize', 'OccupancyMatrix', 'GridOriginInLocal'}, ...
    {'map_res/1', 'HarnessLidar/6', 'map_size/1', 'HarnessLidar/5', 'map_org/1'});
% scan validity (scn_scan [t valid]): an invalid scan is blank, as Shaddad_Perception publishes a LiDAR loss today
% (NAV-01): scanValid false, ranges NaN, protectiveStop false
FW('scn_scan', 'P.gnc.tb.scn_scan', tl, 'off');
CMP(n, 'scan_ok', '>', '0.5', 'scn_scan/1');
C(n, 'nan_r', 'NaN(P.gnc.obs.n_beams, 1)', 'single', tl);
SW(n, 'ranges_v', 'HarnessLidar/1', 'scan_ok/1', 'nan_r/1');
LOG(n, 'prot_v', 'AND', {'HarnessLidar/3', 'scan_ok/1'});
C(n, 'PerIC', 'P.ic.Bus_Perception', 'Bus: Bus_Perception', tl);
BA(n, 'Perception', 'PerIC/1', {'t_scan', 'scanValid', 'ranges_B', 'angles_B', 'protectiveStop', 'protFieldLen', 'map'}, ...
    {'clock_l/1', 'scan_ok/1', 'ranges_v/1', 'HarnessLidar/2', 'prot_v/1', 'HarnessLidar/4', 'Map/1'});
L(n, 'Perception/1', 'GNC/2', 'perception');
%% ---- outputs ----
A(n, 'simulink/Signal Routing/Mux', 'truth_mux', 'Inputs', '5');
L(n, 'x_true/1', 'truth_mux/1'); L(n, 'y_true/1', 'truth_mux/2'); L(n, 'psi_w/1', 'truth_mux/3');
L(n, 'v_true/1', 'truth_mux/4'); L(n, 'lag_r/1', 'truth_mux/5');
A(n, 'simulink/Sinks/Out1', 'truth', 'Port', '1');       L(n, 'truth_mux/1', 'truth/1', 'truth');
A(n, 'simulink/Sinks/Out1', 'gnc_cmd', 'Port', '2');     L(n, 'GNC/1', 'gnc_cmd/1', 'gnc_cmd');
A(n, 'simulink/Sinks/Out1', 'gnc_debug', 'Port', '3');   L(n, 'GNC/4', 'gnc_debug/1', 'gnc_debug');
A(n, 'simulink/Sinks/Out1', 'gnc_status', 'Port', '4');  L(n, 'GNC/3', 'gnc_status/1', 'gnc_status');
A(n, 'simulink/Signal Routing/Mux', 'shaped_mux', 'Inputs', '2');  L(n, 'v_ref/1', 'shaped_mux/1'); L(n, 'w_ref/1', 'shaped_mux/2');
A(n, 'simulink/Sinks/Out1', 'shaped', 'Port', '5');      L(n, 'shaped_mux/1', 'shaped/1', 'shaped');
A(n, 'simulink/Sinks/Out1', 'nav_cmd', 'Port', '6');     L(n, 'GNC/2', 'nav_cmd/1', 'nav_cmd');
A(n, 'simulink/Sinks/Out1', 'prot', 'Port', '7');        L(n, 'HarnessLidar/3', 'prot/1', 'prot');
%% ---- line names of the scenario Demuxes and the harness outputs (L5) ----
scn = {'mcu_d', {'scn_rc_sw', 'scn_rc_link', 'scn_drive_ready', 'scn_estop', 'scn_charging', 'scn_impl_ready', 'scn_freeze'}; ...
       'cmd_d', {'scn_cmd', 'scn_seq'}; 'bms_d', {'scn_soc', 'scn_bms_crit'}; ...
       'nav_d', {'scn_valid', 'scn_sigma', 'scn_quality', 'scn_dx', 'scn_dy'}; ...
       'HarnessLidar', {'ranges', 'angles'}};
for k = 1:size(scn, 1)
    for j = 1:numel(scn{k, 2}), NM(n, sprintf('%s/%d', scn{k, 1}, j), scn{k, 2}{j}); end
end
NM(n, 'HarnessLidar/4', 'protLen');
NM(n, 'Kinematics/2', 'state_dot');
set_param(n, 'Description', ['tb_gnc_mission: GNC mission bench (T15, gnc spec 8). Built by scripts/build/build_gnc_mission.m ' ...
    '(''tb_gnc_mission''). Shaddad_GNC (Normal) + kinematic plant with dead time, MCU AUTO shaper and v / yaw lags ' ...
    '(as tb_gnc), drive gating (drive_ready, E-stop), stuck factor; Bus_NavState with RTK noise and scenario ' ...
    'validity / sigma / quality / offset; Bus_McuStatus (rc switch, link, E-stop, drive_ready, implement_ready ' ...
    'model, charging, frozen heartbeat, v_applied); BMS SOC ramp; MissionCmd table; spray demand; harness LiDAR ' ...
    'gnc_harnessLidar (rangeSensor on P.world.occ + dynamic obstacles P.gnc.tb.dyn) -> Bus_Perception; scan validity ' ...
    'P.gnc.tb.scn_scan (an invalid scan is blank: scanValid false, ranges NaN, protectiveStop false). Scenario ' ...
    'data: P.gnc.tb.scn_* tables and P.gnc.mission via Simulink.SimulationInput.setVariable. Logged: fsm, obs, rp ' ...
    '(inside Shaddad_GNC) in logsout; outputs truth, gnc_cmd, gnc_debug, gnc_status, shaped, nav_cmd, prot.']);
shaddad_bu('save', n, f);
close_system(n, 0);
end
