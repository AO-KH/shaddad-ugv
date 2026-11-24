function out = build_shaddad_gnc(which)
%BUILD_SHADDAD_GNC  Build models/Shaddad_GNC.slx and the GNC Subsystem References (build task T14, owner gnc).
%
%   build_shaddad_gnc()              build every T14 file (stubs only while they are still stubs)
%   build_shaddad_gnc(names)         build only these files (cellstr / char), e.g. 'gnc_PathTracker'
%   T = build_shaddad_gnc('ports')   port table of every GNC Subsystem Reference (contract for T15:
%                                    the stubs below have their final ports; T15 must keep them)
%   built = build_shaddad_gnc(...)   names of the files written
%
%   Files (docs/build_plan.md T14, docs/specs/gnc.md 2-7, revision B):
%     subsystems/gnc/gnc_InputConditioning.slx  Rate Transitions to 0.05 s, Bus_NavCond, nav_ok / nav_degraded
%                                               (position sigma across / along the lane with hysteresis, NavQuality,
%                                               odometry consistency; pe one step late), mission command =
%                                               mcmd OR rc_aux_cmd (Detect Change on cmd_seq / rc_aux_seq)
%     subsystems/gnc/gnc_PathServer.slx         Selectors on P.gnc.mission rows (seg_idx, nxt_idx), REPLAN
%                                               switch, Detect Change -> toggle -> Triggered Subsystem
%                                               SegmentPrep (2 x Path Smoother Spline, Velocity Profiler) -> Bus_PathWindow
%     subsystems/gnc/gnc_PathAnalyzer.slx       MATLAB Function gnc_pathAnalyzer -> Bus_PathError
%     subsystems/gnc/gnc_SpeedGovernor.slx      1-D LUTs, MinMax(12), Rate Limiter Dynamic, d_stop_req; MATLAB Function
%                                               gnc_descentLimit (downhill cap, held pitch + DEM look-ahead) in v_pitch
%     subsystems/gnc/gnc_PathTracker.slx        Pure Pursuit (omega = v*curvature), RowTracker Variant
%                                               VC_ROW_TRACKER (1 gnc_mpcPlantUpdate + Adaptive MPC, 2 Stanley),
%                                               Align, TrackerSelect Multiport Switch
%     subsystems/gnc/gnc_CommandShaping.slx     omega_max(v), omega_max(tilt), omega rate limit, track-speed
%                                               envelope |v| + |w| chi B/2 <= v_track_max (D43), heartbeat, Bus_GncCmd;
%                                               MATLAB Function gnc_descentEnvelope (downhill cap at the nav pitch)
%     stubs with final ports (replaced by T15): gnc_MissionManager (fixed AUTO mode with a block-based
%     segment sequencer for the tracking bench, soc_rth block math), gnc_LocalAvoidance (d_obs = Inf),
%     gnc_GlobalReplanner, gnc_ImplementManager (all zero), gnc_StatusMetrics (status / debug pass-through)
%     models/Shaddad_GNC.slx                    root ports ICD 3.1 GNC (6 inputs, no plan port: Constant
%                                               MissionSource = P.gnc.mission, D57), base 0.05 s, Unit Delays on
%                                               the Pure Pursuit target direction and on the applied omega (MPC ext.mv)
%   Needs P, Ts, VC_*, Bus_* in base (shaddad_params is run when missing). Every number comes from P.gnc,
%   P.ic or Ts. A stub file is rebuilt only while its Description starts with SHADDAD_GNC_STUB.
%   Diagrams (layout spec 6): every file is saved through shaddad_bu('save'), which lays out every level of it. The root
%   ports stay classic (the table above, ICD 3.1): a bus reaches each consumer through its own Bus Selector right in front
%   of it (helper PICK), and every signal line is named (L5), through shaddad_bu('nline') (helper L). The variant
%   choices keep identical classic ports.

T = local_ports();
if nargin >= 1 && (ischar(which) || isstring(which)) && strcmpi(which, 'ports')
    out = T;
    return
end
all = {'gnc_InputConditioning', 'gnc_MissionManager', 'gnc_PathServer', 'gnc_PathAnalyzer', ...
       'gnc_LocalAvoidance', 'gnc_GlobalReplanner', 'gnc_SpeedGovernor', 'gnc_PathTracker', ...
       'gnc_ImplementManager', 'gnc_CommandShaping', 'gnc_StatusMetrics', 'Shaddad_GNC'};
stubs = {'gnc_MissionManager', 'gnc_LocalAvoidance', 'gnc_GlobalReplanner', 'gnc_ImplementManager', ...
         'gnc_StatusMetrics'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_GncCmd'',''var'')')
    shaddad_params();
end
root = char(currentProject().RootFolder);
dirS = fullfile(root, 'subsystems', 'gnc');
% models of this task that reference the files (closed unsaved so the files can be replaced)
for m = {'tb_gnc', 'Shaddad_GNC'}
    if bdIsLoaded(m{1}), close_system(m{1}, 0); end
end
built = {};
for k = 1:numel(which)
    n = which{k};
    if strcmp(n, 'Shaddad_GNC')
        f = fullfile(root, 'models', 'Shaddad_GNC.slx');
    else
        f = fullfile(dirS, [n '.slx']);
        if any(strcmp(n, stubs)) && isfile(f)
            info = Simulink.MDLInfo(f);
            if ~startsWith(strtrim(info.Description), 'SHADDAD_GNC_STUB')
                fprintf('build_shaddad_gnc: %s has been replaced by its owner (T15), not rebuilt\n', n);
                continue
            end
        end
    end
    feval(['local_' n], n, f, T);
    built{end+1} = n; %#ok<AGROW>
end
if nargout > 0, out = built; end
end

% =============================================================================================
%  Port table (contract)
% =============================================================================================
function T = local_ports()
b = @(x) ['Bus: Bus_' x];
T = struct();
T.gnc_InputConditioning = local_pt( ...
    {'nav_state', b('NavState'), 'Ts.nav', '-1'; 'mcu_status', b('McuStatus'), 'Ts.gnc', '-1'; ...
     'bms_status', b('BmsStatus'), 'Ts.hk', '-1'; 'mcmd', b('MissionCmd'), 'Ts.mcmd', '-1'; ...
     'spray', b('SprayDemand'), 'Ts.spray', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'}, ...
    {'nav', b('NavCond'); 'mcu', b('McuStatus'); 'bms', b('BmsStatus'); 'mcmd_c', b('MissionCmd'); ...
     'spray_c', b('SprayDemand')});
T.gnc_MissionManager = local_pt( ...
    {'nav', b('NavCond'), 'Ts.gnc', '-1'; 'mcu', b('McuStatus'), 'Ts.gnc', '-1'; 'bms', b('BmsStatus'), 'Ts.gnc', '-1'; ...
     'mcmd', b('MissionCmd'), 'Ts.gnc', '-1'; 'obs', b('ObsFlags'), 'Ts.gnc', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'; ...
     'plan', b('MissionPlan'), '-1', '-1'; 'rp', b('ReplanResult'), 'Ts.gnc', '-1'; 'teb_info', 'double', 'Ts.gnc', '4'}, ...
    {'fsm', b('FsmOut'); 'nav_cmd', b('NavCmd'); 'soc_rth', 'double'});
T.gnc_PathServer = local_pt( ...
    {'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'plan', b('MissionPlan'), '-1', '-1'; 'rp', b('ReplanResult'), 'Ts.gnc', '-1'}, ...
    {'win', b('PathWindow')});
T.gnc_PathAnalyzer = local_pt( ...
    {'nav', b('NavCond'), 'Ts.gnc', '-1'; 'win', b('PathWindow'), 'Ts.gnc', '-1'}, {'pe', b('PathError')});
T.gnc_LocalAvoidance = local_pt( ...
    {'perception', b('Perception'), 'Ts.perception', '-1'; 'nav', b('NavCond'), 'Ts.gnc', '-1'; ...
     'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'win', b('PathWindow'), 'Ts.gnc', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'; ...
     'tdir', 'double', 'Ts.gnc', '1'}, ...
    {'obs', b('ObsFlags'); 'teb_cmd', 'double'; 'teb_info', 'double'});
T.gnc_GlobalReplanner = local_pt( ...
    {'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'nav', b('NavCond'), 'Ts.gnc', '-1'; ...
     'perception', b('Perception'), 'Ts.perception', '-1'; 'plan', b('MissionPlan'), '-1', '-1'}, ...
    {'rp', b('ReplanResult')});
T.gnc_SpeedGovernor = local_pt( ...
    {'pe', b('PathError'), 'Ts.gnc', '-1'; 'nav', b('NavCond'), 'Ts.gnc', '-1'; 'obs', b('ObsFlags'), 'Ts.gnc', '-1'; ...
     'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'bms', b('BmsStatus'), 'Ts.gnc', '-1'; 'mcmd', b('MissionCmd'), 'Ts.gnc', '-1'; ...
     'spray', b('SprayDemand'), 'Ts.gnc', '-1'; 'win', b('PathWindow'), 'Ts.gnc', '-1'; 'plan', b('MissionPlan'), '-1', '-1'}, ...
    {'v_gov', 'double'; 'v_limits', 'single'; 'd_stop_req', 'double'; 'v_prof', 'double'});
T.gnc_PathTracker = local_pt( ...
    {'nav', b('NavCond'), 'Ts.gnc', '-1'; 'win', b('PathWindow'), 'Ts.gnc', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'; ...
     'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'v_gov', 'double', 'Ts.gnc', '1'; 'teb_cmd', 'double', 'Ts.gnc', '2'; ...
     'w_applied', 'double', 'Ts.gnc', '1'}, ...
    {'v_trk', 'double'; 'w_trk', 'double'; 'tdir', 'double'; 'w_pp', 'double'; 'w_row', 'double'; 'qp_status', 'int32'});
T.gnc_ImplementManager = local_pt( ...
    {'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'; 'nav', b('NavCond'), 'Ts.gnc', '-1'; ...
     'mcu', b('McuStatus'), 'Ts.gnc', '-1'; 'spray', b('SprayDemand'), 'Ts.gnc', '-1'; 'plan', b('MissionPlan'), '-1', '-1'; ...
     'obs', b('ObsFlags'), 'Ts.gnc', '-1'}, ...
    {'impl', b('GncCmd')});
T.gnc_CommandShaping = local_pt( ...
    {'v_trk', 'double', 'Ts.gnc', '1'; 'w_trk', 'double', 'Ts.gnc', '1'; 'fsm', b('FsmOut'), 'Ts.gnc', '-1'; ...
     'nav', b('NavCond'), 'Ts.gnc', '-1'; 'impl', b('GncCmd'), 'Ts.gnc', '-1'; 'plan', b('MissionPlan'), '-1', '-1'}, ...
    {'gnc_cmd', b('GncCmd'); 'w_applied', 'double'});
T.gnc_StatusMetrics = local_pt( ...
    {'fsm', b('FsmOut'), 'Ts.gnc', '-1'; 'pe', b('PathError'), 'Ts.gnc', '-1'; 'nav', b('NavCond'), 'Ts.gnc', '-1'; ...
     'plan', b('MissionPlan'), '-1', '-1'; 'rp', b('ReplanResult'), 'Ts.gnc', '-1'; 'soc_rth', 'double', 'Ts.gnc', '1'; ...
     'dbg', b('GncDebug'), 'Ts.gnc', '-1'}, ...
    {'gnc_status', b('GncStatus'); 'gnc_debug', b('GncDebug')});
end

function R = local_pt(in, out)
R.in = cell2struct(in, {'name', 'type', 'ts', 'dims'}, 2);
R.out = cell2struct(out, {'name', 'type'}, 2);
end

% =============================================================================================
%  Helpers
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
% fresh Subsystem Reference file with the ports of the table
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
set_param(n, 'Description', desc);
shaddad_bu('save', n, f);
close_system(n, 0);
end

function BS(sys, name, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', name, 'OutputSignals', strjoin(sigs, ','));
end

function BC(sys, name, bus, elems, srcs)
% Bus Creator of 'Bus: <bus>' fed by srcs{k} (port strings), lines named after the elements
A(sys, 'simulink/Signal Routing/Bus Creator', name, 'Inputs', num2str(numel(elems)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on');
for k = 1:numel(elems)
    src = srcs{k};
    blk = [sys '/' extractBefore(src, '/')];
    if strcmp(get_param(blk, 'BlockType'), 'BusSelector')
        % a Bus Selector output keeps its element name; a Signal Conversion lets the line carry the
        % element name of the created bus (the Bus Creator takes names from its input signals)
        sc = [name '_' elems{k}];
        A(sys, 'simulink/Signal Attributes/Signal Conversion', sc);
        L(sys, src, [sc '/1']);
        src = [sc '/1'];
    end
    L(sys, src, sprintf('%s/%d', name, k), elems{k});
end
end

function BA(sys, name, busSrc, sigs, srcs)
% Bus Assignment: busSrc -> port 1, srcs{k} -> port k+1 assigned to element sigs{k}
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
% Selector: opts e.g. 'Index vector (port),Index vector (dialog)', prm = {'', '4'}; varargin: extra
% parameters set with the block (e.g. OutputSizes for 'Starting index (port)')
nd = numel(strsplit(opts, ','));
if nd == 1, varargin = [{'InputPortWidth', '-1'}, varargin]; end
A(sys, 'simulink/Signal Routing/Selector', name, 'NumberOfDimensions', num2str(nd), 'IndexMode', 'One-based', ...
    varargin{:}, 'IndexOptions', opts, 'IndexParamArray', prm(:));
if nargin >= 5 && ~isempty(src), L(sys, src, [name '/1']); end
if nargin >= 6 && ~isempty(idxSrc), L(sys, idxSrc, [name '/2']); end
end

function mf = MFB(sys, name, script, data, params, ts, desc)
% MATLAB Function block: body = one-line call of a %#codegen file; data = {name, DataType}; params =
% {name, expr} non-tunable parameter data (sub-structs of P.gnc only, build rule 2) set through a mask
mf = A(sys, 'simulink/User-Defined Functions/MATLAB Function', name);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = script;
cfg.UpdateMethod = 'Discrete';
cfg.SampleTime = ts;
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
% delete every line and block inside a (choice) subsystem
lns = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
blks = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all');
blks = setdiff(blks, {sys});
for k = 1:numel(blks), delete_block(blks{k}); end
end

% =============================================================================================
%  gnc_InputConditioning
% =============================================================================================
function local_gnc_InputConditioning(n, f, T)
local_new_ssr(n, f, T.(n));
% ---- navigation 0.01 -> 0.05; one Bus Selector per consumer, right in front of it (layout spec 6.2) ----
RT(n, 'RT_nav', 'Ts.gnc', 'nav_state/1');
PICK(n, 'RT_nav/1', 'NavSel_pos', {'pos_W'}, {}, 'nav_50ms');
SEL(n, 'xy', 'Index vector (dialog)', {'[1 2]'}, 'NavSel_pos/1');
PICK(n, 'RT_nav/1', 'NavSel_eul', {'eul'});
SEL(n, 'yaw', 'Index vector (dialog)', {'1'}, 'NavSel_eul/1');
SEL(n, 'pitch', 'Index vector (dialog)', {'2'}, 'NavSel_eul/1');
SEL(n, 'roll', 'Index vector (dialog)', {'3'}, 'NavSel_eul/1');
PICK(n, 'RT_nav/1', 'NavSel_vel', {'vel_B'});
SEL(n, 'v', 'Index vector (dialog)', {'1'}, 'NavSel_vel/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'pose', 'NumInputs', '2', 'Mode', 'Vector');
L(n, 'xy/1', 'pose/1'); L(n, 'yaw/1', 'pose/2');
PICK(n, 'RT_nav/1', 'NavSel_std', {'posStd'});
SEL(n, 'stdXY', 'Index vector (dialog)', {'[1 2]'}, 'NavSel_std/1');
A(n, 'simulink/Math Operations/MinMax', 'sigma_pos', 'Function', 'max', 'Inputs', '1');
L(n, 'stdXY/1', 'sigma_pos/1');
% nav_ok = navValid AND sigma_pos < fsm.sigma_lost
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'sig_lost_ok', 'relop', '<', 'const', 'P.gnc.fsm.sigma_lost');
L(n, 'sigma_pos/1', 'sig_lost_ok/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'nav_ok', 'Operator', 'AND', 'Inputs', '2');
PICK(n, 'RT_nav/1', 'NavSel_ok', {'navValid'}, {'nav_ok/1'}); L(n, 'sig_lost_ok/1', 'nav_ok/2');
% nav_degraded = navValid AND (quality < q_good OR ct_on OR at_on OR odometry inconsistent)
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'q_low', 'relop', '<', 'const', 'P.gnc.fsm.q_good');
PICK(n, 'RT_nav/1', 'NavSel_q', {'quality'}, {'q_low/1'});
% position sigma in the lane frame (T7, S4 fix 2026-11-14): tangent t = [cos psi_ref; sin psi_ref], normal
% n = [-sin psi_ref; cos psi_ref] (ENU, psi CCW from East) of the path error one step late (pe_z breaks the loop
% nav -> PathAnalyzer -> pe); C = [sE^2 cEN; cEN sN^2], [sE sN] = posStd(1:2), cEN = covPose(1,2) (one EKF P,
% Shaddad_NavFusion/NavOutput; the benches leave covPose at 0); sigma_ct = sqrt(n'Cn), sigma_at = sqrt(t'Ct), both
% sigma_pos without a valid path error. Relays: ct on at >= fsm.sigma_pause, off at <= fsm.sigma_degr; at on at
% >= fsm.sigma_along, off at <= fsm.sigma_along_off (double out, then boolean: a boolean Relay casts its switch values)
A(n, 'simulink/Discrete/Unit Delay', 'pe_z', 'InitialCondition', 'P.ic.Bus_PathError', 'SampleTime', '-1');
L(n, 'pe/1', 'pe_z/1');
PICK(n, 'pe_z/1', 'PeSel_lane', {'psi_ref', 'valid'});
PICK(n, 'RT_nav/1', 'NavSel_cov', {'covPose'});
SEL(n, 'cEN', 'Index vector (dialog),Index vector (dialog)', {'1', '2'}, 'NavSel_cov/1');
A(n, 'simulink/Signal Routing/Mux', 'lane_cov', 'Inputs', '3');
L(n, 'stdXY/1', 'lane_cov/1'); L(n, 'cEN/1', 'lane_cov/2'); L(n, 'PeSel_lane/1', 'lane_cov/3');
A(n, 'simulink/User-Defined Functions/Fcn', 'sig_ct_lane', 'Expr', ...
    'sqrt(abs(sin(u(4))^2*u(1)^2 + cos(u(4))^2*u(2)^2 - 2*sin(u(4))*cos(u(4))*u(3)))');
A(n, 'simulink/User-Defined Functions/Fcn', 'sig_at_lane', 'Expr', ...
    'sqrt(abs(cos(u(4))^2*u(1)^2 + sin(u(4))^2*u(2)^2 + 2*sin(u(4))*cos(u(4))*u(3)))');
L(n, 'lane_cov/1', 'sig_ct_lane/1'); L(n, 'lane_cov/1', 'sig_at_lane/1');
rly = {'ct', 'P.gnc.fsm.sigma_pause', 'P.gnc.fsm.sigma_degr'; 'at', 'P.gnc.fsm.sigma_along', 'P.gnc.fsm.sigma_along_off'};
for k = 1:size(rly, 1)
    a = rly{k, 1};
    A(n, 'simulink/Signal Routing/Switch', ['sig_' a], 'Criteria', 'u2 ~= 0');
    L(n, ['sig_' a '_lane/1'], ['sig_' a '/1']); L(n, 'PeSel_lane/2', ['sig_' a '/2']); L(n, 'sigma_pos/1', ['sig_' a '/3']);
    A(n, 'simulink/Discontinuities/Relay', [a '_hyst'], 'OnSwitchValue', rly{k, 2}, 'OffSwitchValue', rly{k, 3}, ...
        'OnOutputValue', '1', 'OffOutputValue', '0', 'ZeroCross', 'off');
    L(n, ['sig_' a '/1'], [a '_hyst/1']);
    DTC(n, [a '_on'], 'boolean', [a '_hyst/1']);
end
% odometry consistency: mcu.valid AND |v_nav - mean(v_track_meas)| > v_odo_tol
PICK(n, 'mcu_status/1', 'McuSel_vt', {'v_track_meas'});
DTC(n, 'vtr_d', 'double', 'McuSel_vt/1');
A(n, 'simulink/Math Operations/Sum of Elements', 'vtr_sum');
L(n, 'vtr_d/1', 'vtr_sum/1');
A(n, 'simulink/Math Operations/Gain', 'vtr_mean', 'Gain', '0.5');
L(n, 'vtr_sum/1', 'vtr_mean/1');
A(n, 'simulink/Math Operations/Sum', 'dv_odo', 'Inputs', '+-');
L(n, 'v/1', 'dv_odo/1'); L(n, 'vtr_mean/1', 'dv_odo/2');
A(n, 'simulink/Math Operations/Abs', 'dv_abs');
L(n, 'dv_odo/1', 'dv_abs/1');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'odo_bad0', 'relop', '>', 'const', 'P.gnc.fsm.v_odo_tol');
L(n, 'dv_abs/1', 'odo_bad0/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'odo_bad', 'Operator', 'AND', 'Inputs', '2');
L(n, 'odo_bad0/1', 'odo_bad/1'); PICK(n, 'mcu_status/1', 'McuSel_valid', {'valid'}, {'odo_bad/2'});
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'degr_any', 'Operator', 'OR', 'Inputs', '4');
L(n, 'q_low/1', 'degr_any/1'); L(n, 'ct_on/1', 'degr_any/2'); L(n, 'at_on/1', 'degr_any/3'); L(n, 'odo_bad/1', 'degr_any/4');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'nav_degraded', 'Operator', 'AND', 'Inputs', '2');
PICK(n, 'RT_nav/1', 'NavSel_degr', {'navValid'}, {'nav_degraded/1'}); L(n, 'degr_any/1', 'nav_degraded/2');
PICK(n, 'RT_nav/1', 'NavSel_t', {'t'});
PICK(n, 'RT_nav/1', 'NavSel_yaw_rate', {'yawRate'});
PICK(n, 'RT_nav/1', 'NavSel_fix', {'quality'});
BC(n, 'NavCond', 'Bus_NavCond', {'t', 'pose', 'v', 'yaw_rate', 'pitch', 'roll', 'sigma_pos', 'nav_ok', ...
    'nav_degraded', 'gnss_fix'}, {'NavSel_t/1', 'pose/1', 'v/1', 'NavSel_yaw_rate/1', 'pitch/1', 'roll/1', 'sigma_pos/1', ...
    'nav_ok/1', 'nav_degraded/1', 'NavSel_fix/1'});
L(n, 'NavCond/1', 'nav/1', 'nav');
% ---- MCU status (0.05) passes; BMS / spray 0.1 -> 0.05 ----
L(n, 'mcu_status/1', 'mcu/1', 'mcu');
RT(n, 'RT_bms', 'Ts.gnc', 'bms_status/1');  L(n, 'RT_bms/1', 'bms/1', 'bms');
RT(n, 'RT_spray', 'Ts.gnc', 'spray/1');      L(n, 'RT_spray/1', 'spray_c/1', 'spray_c');
% ---- mission command: HMI mcmd (0.2 -> 0.05) OR RC aux (new cmd on cmd_seq / rc_aux_seq change) ----
RT(n, 'RT_mcmd', 'Ts.gnc', 'mcmd/1');
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'new_hmi', 'vinit', '0');
PICK(n, 'RT_mcmd/1', 'CmdSel_seq', {'cmd_seq'}, {'new_hmi/1'}, 'mcmd_50ms');
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'new_rc', 'vinit', '0');
PICK(n, 'mcu_status/1', 'McuSel_seq', {'rc_aux_seq'}, {'new_rc/1'});
C(n, 'cmd_none', '0', 'uint8');
A(n, 'simulink/Signal Routing/Switch', 'cmd_rc', 'Criteria', 'u2 ~= 0');
PICK(n, 'mcu_status/1', 'McuSel_cmd', {'rc_aux_cmd'}, {'cmd_rc/1'}); L(n, 'new_rc/1', 'cmd_rc/2'); L(n, 'cmd_none/1', 'cmd_rc/3');
A(n, 'simulink/Signal Routing/Switch', 'cmd_out', 'Criteria', 'u2 ~= 0');
PICK(n, 'RT_mcmd/1', 'CmdSel_cmd', {'cmd'}, {'cmd_out/1'}); L(n, 'new_hmi/1', 'cmd_out/2'); L(n, 'cmd_rc/1', 'cmd_out/3');
% merged sequence counter (uint8, wraps): +1 on every new command from either source
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'new_any', 'Operator', 'OR', 'Inputs', '2');
L(n, 'new_hmi/1', 'new_any/1'); L(n, 'new_rc/1', 'new_any/2');
DTC(n, 'new_u8', 'uint8', 'new_any/1');
A(n, 'simulink/Math Operations/Sum', 'seq_add', 'Inputs', '++', 'OutDataTypeStr', 'uint8', ...
    'SaturateOnIntegerOverflow', 'off');
A(n, 'simulink/Discrete/Unit Delay', 'seq_z', 'InitialCondition', 'uint8(0)', 'SampleTime', '-1');
L(n, 'seq_z/1', 'seq_add/1'); L(n, 'new_u8/1', 'seq_add/2'); L(n, 'seq_add/1', 'seq_z/1');
BA(n, 'McmdCond', 'RT_mcmd/1', {'cmd', 'cmd_seq'}, {'cmd_out/1', 'seq_add/1'});
L(n, 'McmdCond/1', 'mcmd_c/1', 'mcmd_c');
local_save(n, f, sprintf(['gnc_InputConditioning (T14, gnc spec 7 / build plan T14.2). Rate Transitions: nav_state 0.01, ' ...
    'bms 0.1, spray 0.1, mcmd 0.2 -> 0.05 s (integrity + deterministic); mcu_status is already 0.05 s; pe (PathAnalyzer, ' ...
    '0.05 s) through the Unit Delay pe_z.\n' ...
    'Bus_NavCond: pose = [pos_W(1:2); eul(1)], v = vel_B(1), pitch = eul(2), roll = eul(3), sigma_pos = max(posStd(1:2)), ' ...
    'nav_ok = navValid AND sigma_pos < P.gnc.fsm.sigma_lost, nav_degraded = navValid AND (quality < q_good OR ' ...
    'ct_on OR at_on OR (mcu.valid AND |v - mean(v_track_meas)| > v_odo_tol)), gnss_fix = NavQuality.\n' ...
    'T7 sigma term (S4 fix 2026-11-14): sigma_ct = sqrt(n''Cn), sigma_at = sqrt(t''Ct), t / n the tangent / normal of ' ...
    'pe_z.psi_ref, C = [sE^2 cEN; cEN sN^2] from posStd(1:2) and covPose(1,2); sigma_pos for both without a valid pe_z. ' ...
    'Relays: ct_on on at >= sigma_pause, off at <= sigma_degr; at_on on at >= sigma_along, off at <= sigma_along_off.\n' ...
    'Mission command: Detect Change on mcmd.cmd_seq (HMI, priority) and mcu.rc_aux_seq (RC aux CH13/CH14); ' ...
    'mcmd_c.cmd = the new command in the step it arrives, else NONE; mcmd_c.cmd_seq = merged counter. ' ...
    'McuStatus rc_mode_sw codes: 0 MANUAL, 1 HOLD, 2 AUTO.']));
end

% =============================================================================================
%  gnc_MissionManager (stub, final ports; T15 replaces it with gnc_missionFSM)
% =============================================================================================
function local_gnc_MissionManager(n, f, T)
local_new_ssr(n, f, T.(n));
% loop breakers kept in the final design (spec 7): delayed path error, replan result, TEB info
A(n, 'simulink/Discrete/Unit Delay', 'pe_z', 'InitialCondition', 'P.ic.Bus_PathError', 'SampleTime', '-1');
L(n, 'pe/1', 'pe_z/1');
A(n, 'simulink/Discrete/Unit Delay', 'rp_z', 'InitialCondition', 'P.ic.Bus_ReplanResult', 'SampleTime', '-1');
L(n, 'rp/1', 'rp_z/1');  TERM(n, 'rp_z/1', 'T_rp');
A(n, 'simulink/Discrete/Unit Delay', 'teb_z', 'InitialCondition', 'zeros(4,1)', 'SampleTime', '-1');
L(n, 'teb_info/1', 'teb_z/1');  TERM(n, 'teb_z/1', 'T_teb');
TERM(n, 'mcu/1', 'T_mcu');  TERM(n, 'bms/1', 'T_bms');  TERM(n, 'mcmd/1', 'T_mcmd');  TERM(n, 'obs/1', 'T_obs');
% ---- segment sequencer: seg++ on the rising edge of (pe_z.valid AND pe_z.s_rem < seg_end_tol) ----
BS(n, 'PeSel', {'s_rem', 'valid'});
L(n, 'pe_z/1', 'PeSel/1');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'at_end', 'relop', '<', 'const', 'P.gnc.path.seg_end_tol');
L(n, 'PeSel/1', 'at_end/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'seg_done', 'Operator', 'AND', 'Inputs', '2');
L(n, 'at_end/1', 'seg_done/1'); L(n, 'PeSel/2', 'seg_done/2');
A(n, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'done_edge', 'vinit', '0');
L(n, 'seg_done/1', 'done_edge/1');
DTC(n, 'edge_d', 'double', 'done_edge/1');
A(n, 'simulink/Math Operations/Sum', 'cnt_add', 'Inputs', '++');
A(n, 'simulink/Discrete/Unit Delay', 'cnt_z', 'InitialCondition', '1', 'SampleTime', 'Ts.gnc');
L(n, 'cnt_z/1', 'cnt_add/1'); L(n, 'edge_d/1', 'cnt_add/2'); L(n, 'cnt_add/1', 'cnt_z/1');
% last driven segment = n_seg - 1 (the last row of the plan is the TERMINAL extension)
BS(n, 'PlanSel', {'n_seg', 'seg', 'valid', 'home'});
L(n, 'plan/1', 'PlanSel/1');
DTC(n, 'nseg_d', 'double', 'PlanSel/1');
A(n, 'simulink/Math Operations/Bias', 'last_drv0', 'Bias', '-1');
L(n, 'nseg_d/1', 'last_drv0/1');
C(n, 'one', '1', 'double');
A(n, 'simulink/Math Operations/MinMax', 'last_drv', 'Function', 'max', 'Inputs', '2');
L(n, 'last_drv0/1', 'last_drv/1'); L(n, 'one/1', 'last_drv/2');
A(n, 'simulink/Math Operations/MinMax', 'seg_d', 'Function', 'min', 'Inputs', '2');
L(n, 'cnt_add/1', 'seg_d/1'); L(n, 'last_drv/1', 'seg_d/2');
A(n, 'simulink/Logic and Bit Operations/Relational Operator', 'done', 'Operator', '>');
L(n, 'cnt_add/1', 'done/1'); L(n, 'last_drv/1', 'done/2');
A(n, 'simulink/Math Operations/Bias', 'nxt0', 'Bias', '1');
L(n, 'seg_d/1', 'nxt0/1');
A(n, 'simulink/Math Operations/MinMax', 'nseg_max1', 'Function', 'max', 'Inputs', '2');
L(n, 'nseg_d/1', 'nseg_max1/1'); L(n, 'one/1', 'nseg_max1/2');
A(n, 'simulink/Math Operations/MinMax', 'nxt_d', 'Function', 'min', 'Inputs', '2');
L(n, 'nxt0/1', 'nxt_d/1'); L(n, 'nseg_max1/1', 'nxt_d/2');
DTC(n, 'seg_idx', 'uint16', 'seg_d/1');
DTC(n, 'nxt_idx', 'uint16', 'nxt_d/1');
% segment type of the current segment -> tracker / sub-mode / mode speed cap (P.gnc.stub tables)
SEL(n, 'type_s', 'Index vector (port),Index vector (dialog)', {'1'; '1'}, 'PlanSel/2', 'seg_idx/1');
DTC(n, 'type_d', 'double', 'type_s/1');
A(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'trk_tab', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.gnc.stub.trk_by_type', 'DiagnosticForOutOfRangeInput', 'None');
A(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'sub_tab', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.gnc.stub.sub_by_type', 'DiagnosticForOutOfRangeInput', 'None');
A(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'vcap_tab', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.gnc.stub.vcap_by_type', 'DiagnosticForOutOfRangeInput', 'None');
L(n, 'type_d/1', 'trk_tab/1'); L(n, 'type_d/1', 'sub_tab/1'); L(n, 'type_d/1', 'vcap_tab/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'not_done', 'Operator', 'NOT');
L(n, 'done/1', 'not_done/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'cmd_valid', 'Operator', 'AND', 'Inputs', '2');
L(n, 'not_done/1', 'cmd_valid/1'); L(n, 'PlanSel/3', 'cmd_valid/2');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'stop', 'Operator', 'NOT');
C(n, 'gnc_ready', 'true', 'boolean');
% armed one step after start (the real FSM leaves IDLE on START): the governor rate limiter then
% starts from v = 0 (Rate Limiter Dynamic initialises its state with its first input)
A(n, 'simulink/Discrete/Unit Delay', 'armed', 'InitialCondition', 'false', 'SampleTime', 'Ts.gnc');
L(n, 'gnc_ready/1', 'armed/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'run', 'Operator', 'AND', 'Inputs', '2');
L(n, 'armed/1', 'run/1'); L(n, 'cmd_valid/1', 'run/2');
L(n, 'run/1', 'stop/1');
C(n, 'zero_d', '0', 'double');
C(n, 'st_auto', '2', 'double');            % GncState.AUTO_MISSION / GncModeReq.AUTO
A(n, 'simulink/Signal Routing/Switch', 'trk_sw', 'Criteria', 'u2 ~= 0');
L(n, 'zero_d/1', 'trk_sw/1'); L(n, 'stop/1', 'trk_sw/2'); L(n, 'trk_tab/1', 'trk_sw/3');
A(n, 'simulink/Signal Routing/Switch', 'sub_sw', 'Criteria', 'u2 ~= 0');
L(n, 'zero_d/1', 'sub_sw/1'); L(n, 'stop/1', 'sub_sw/2'); L(n, 'sub_tab/1', 'sub_sw/3');
A(n, 'simulink/Signal Routing/Switch', 'vcap_sw', 'Criteria', 'u2 ~= 0');
L(n, 'zero_d/1', 'vcap_sw/1'); L(n, 'stop/1', 'vcap_sw/2'); L(n, 'vcap_tab/1', 'vcap_sw/3');
A(n, 'simulink/Signal Routing/Switch', 'state_sw', 'Criteria', 'u2 ~= 0');
L(n, 'zero_d/1', 'state_sw/1'); L(n, 'stop/1', 'state_sw/2'); L(n, 'st_auto/1', 'state_sw/3');
DTC(n, 'trk_u8', 'uint8', 'trk_sw/1');
DTC(n, 'sub_u8', 'uint8', 'sub_sw/1');
DTC(n, 'state_u8', 'uint8', 'state_sw/1');
C(n, 'FsmIC', 'P.ic.Bus_FsmOut', 'Bus: Bus_FsmOut');
BA(n, 'Fsm', 'FsmIC/1', {'state', 'sub', 'seg_idx', 'nxt_idx', 'trk_sel', 'mode_req', 'cmd_valid', 'gnc_ready', ...
    'impl_allow', 'mission_done', 'v_cap'}, {'state_u8/1', 'sub_u8/1', 'seg_idx/1', 'nxt_idx/1', 'trk_u8/1', ...
    'state_u8/1', 'run/1', 'gnc_ready/1', 'run/1', 'done/1', 'vcap_sw/1'});
L(n, 'Fsm/1', 'fsm/1', 'fsm');
C(n, 'NavCmd', 'P.ic.Bus_NavCmd', 'Bus: Bus_NavCmd');
L(n, 'NavCmd/1', 'nav_cmd/1', 'nav_cmd');
% ---- soc_rth = soc_reserve + k_tort e_m k_margin d_home / E_nom (spec 2.8, block math) ----
BS(n, 'NavSel', {'pose'});
L(n, 'nav/1', 'NavSel/1');
SEL(n, 'xy', 'Index vector (dialog)', {'[1 2]'}, 'NavSel/1');
SEL(n, 'home_xy', 'Index vector (dialog)', {'[1 2]'}, 'PlanSel/4');
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
local_save(n, f, sprintf(['SHADDAD_GNC_STUB gnc_MissionManager (T14 stub with the final ports; T15 replaces it by the ' ...
    'gnc_missionFSM MATLAB Function, gnc spec 4.1).\nStub behaviour (tracking bench only): fixed AUTO_MISSION, ' ...
    'cmd_valid = plan.valid AND NOT mission_done, mode_req AUTO, gnc_ready true; block-based segment sequencer: ' ...
    'seg_idx += 1 on the rising edge of (pe_z.valid AND pe_z.s_rem < P.gnc.path.seg_end_tol), last driven segment ' ...
    'n_seg - 1 (the last row is the TERMINAL extension), then mission_done -> IDLE / STOP; trk_sel, sub and v_cap ' ...
    'from the segment type via P.gnc.stub tables (ROW -> ROW tracker, TRANSIT/TURN/DOCK -> Pure Pursuit). ' ...
    'Loop breakers of the final design: Unit Delays on pe, rp, teb_info. soc_rth block math of spec 2.8. ' ...
    'nav_cmd = P.ic.Bus_NavCmd (navMode AUTO).']));
end

% =============================================================================================
%  gnc_PathServer
% =============================================================================================
function local_gnc_PathServer(n, f, T)
local_new_ssr(n, f, T.(n));
% the fields of plan: one Bus Selector per consumer, right in front of it (layout spec 6.2). The fields of fsm and rp: one
% Bus Selector each: with picks per consumer the trunks of the three branching port lines share one track (21 line
% overlaps, which only an engine change would remove; task 13 decisions 7)
BS(n, 'FsmSel', {'seg_idx', 'nxt_idx', 'path_src'});
L(n, 'fsm/1', 'FsmSel/1');
BS(n, 'RpSel', {'ok', 'ver', 'path'});
L(n, 'rp/1', 'RpSel/1');
C(n, 'one', '1', 'double');
% first row of segment k in the path table: (k - 1) N_raw + 1
for s = {'A', 'B'}
    x = s{1};  src = 'FsmSel/1';  if x == 'B', src = 'FsmSel/2'; end
    DTC(n, ['k0' x], 'double', src);
    A(n, 'simulink/Math Operations/MinMax', ['k' x], 'Function', 'max', 'Inputs', '2');   % index >= 1
    L(n, ['k0' x '/1'], ['k' x '/1']);  L(n, 'one/1', ['k' x '/2']);
    A(n, 'simulink/Math Operations/Bias', ['km1' x], 'Bias', '-1');  L(n, ['k' x '/1'], ['km1' x '/1']);
    A(n, 'simulink/Math Operations/Gain', ['r0' x], 'Gain', 'P.gnc.path.N_raw');  L(n, ['km1' x '/1'], ['r0' x '/1']);
    A(n, 'simulink/Math Operations/Bias', ['row' x], 'Bias', '1');  L(n, ['r0' x '/1'], ['row' x '/1']);
    PICK(n, 'plan/1', ['PlanSel_rows' x], {'path'});
    SEL(n, ['rows' x], 'Starting index (port),Select all', {''; ''}, ['PlanSel_rows' x '/1'], ['row' x '/1'], ...
        'OutputSizes', 'P.gnc.path.N_raw,1');
    PICK(n, 'plan/1', ['PlanSel_seg' x], {'seg'});
    SEL(n, ['segrow' x], 'Index vector (port),Select all', {''; ''}, ['PlanSel_seg' x '/1'], ['k' x '/1']);
    A(n, 'simulink/Math Operations/Reshape', ['segv' x], 'OutputDimensionality', '1-D array');
    L(n, ['segrow' x '/1'], ['segv' x '/1']);
end
% invalid plan -> straight dummy path (Path Smoother Spline rejects coincident poses)
C(n, 'dummy', 'single(P.gnc.path.dummy)', 'single', 'inf');
for s = {'A', 'B'}
    x = s{1};
    A(n, 'simulink/Signal Routing/Switch', ['valid' x], 'Criteria', 'u2 ~= 0');
    L(n, ['rows' x '/1'], ['valid' x '/1']);
    PICK(n, 'plan/1', ['PlanSel_valid' x], {'valid'}, {['valid' x '/2']});
    L(n, 'dummy/1', ['valid' x '/3']);
end
% REPLAN source only with a successful replan result
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'src_rp', 'relop', '==', 'const', '1');
L(n, 'FsmSel/3', 'src_rp/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'use_rp', 'Operator', 'AND', 'Inputs', '2');
L(n, 'src_rp/1', 'use_rp/1'); L(n, 'RpSel/1', 'use_rp/2');
DTC(n, 'rp_path', 'single', 'RpSel/3');
A(n, 'simulink/Signal Routing/Switch', 'srcA', 'Criteria', 'u2 ~= 0');
L(n, 'rp_path/1', 'srcA/1'); L(n, 'use_rp/1', 'srcA/2'); L(n, 'validA/1', 'srcA/3');
% window trigger: any change of [seg_idx nxt_idx path_src rp.ver plan.valid] toggles the trigger signal
PICK(n, 'plan/1', 'PlanSel_chg', {'valid'});
DTC(n, 'c1', 'double', 'FsmSel/1');  DTC(n, 'c2', 'double', 'FsmSel/2');  DTC(n, 'c3', 'double', 'FsmSel/3');
DTC(n, 'c4', 'double', 'RpSel/2');   DTC(n, 'c5', 'double', 'PlanSel_chg/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'chg_vec', 'NumInputs', '5', 'Mode', 'Vector');
for k = 1:5, L(n, sprintf('c%d/1', k), sprintf('chg_vec/%d', k)); end
A(n, 'simulink/Logic and Bit Operations/Detect Change', 'chg', 'vinit', '0');
L(n, 'chg_vec/1', 'chg/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'chg_any', 'Operator', 'OR', 'Inputs', '1');
L(n, 'chg/1', 'chg_any/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'toggle', 'Operator', 'XOR', 'Inputs', '2');
A(n, 'simulink/Discrete/Unit Delay', 'toggle_z', 'InitialCondition', 'false', 'SampleTime', 'Ts.gnc');
L(n, 'chg_any/1', 'toggle/1'); L(n, 'toggle_z/1', 'toggle/2'); L(n, 'toggle/1', 'toggle_z/1');
DTC(n, 'trig', 'double', 'toggle/1');
% ---- SegmentPrep (Triggered Subsystem, either edge) ----
sp = [n '/SegmentPrep'];
A(n, 'simulink/Ports & Subsystems/Triggered Subsystem', 'SegmentPrep');
local_clear(sp);
spIn = {'rowsA', 'rowsB', 'segA', 'segB', 'idxA', 'idxB', 'valid'};
for k = 1:numel(spIn), A(sp, 'simulink/Sources/In1', spIn{k}, 'Port', num2str(k)); end
A(sp, 'simulink/Ports & Subsystems/Trigger', 'Trigger', 'TriggerType', 'either', 'InitialTriggerSignalState', 'zero');
A(sp, 'simulink/Sinks/Out1', 'win', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_PathWindow', 'InitialOutput', 'P.ic.Bus_PathWindow');
local_segprep(sp);
L(n, 'srcA/1', 'SegmentPrep/1'); L(n, 'validB/1', 'SegmentPrep/2');
L(n, 'segvA/1', 'SegmentPrep/3'); L(n, 'segvB/1', 'SegmentPrep/4');
L(n, 'FsmSel/1', 'SegmentPrep/5'); L(n, 'FsmSel/2', 'SegmentPrep/6');
PICK(n, 'plan/1', 'PlanSel_spv', {'valid'}, {'SegmentPrep/7'});
LP(n, 'trig/1', 'SegmentPrep', 'Trigger', 'trig');
L(n, 'SegmentPrep/1', 'win/1', 'win');
local_save(n, f, sprintf(['gnc_PathServer (T14, gnc spec 2.5 / 7, F4-F5). Selectors take the N_raw = 100 rows of ' ...
    'segments seg_idx (A) and nxt_idx (B) from P.gnc.mission.path (MissionSource) plus their seg-table rows; an invalid ' ...
    'plan serves P.gnc.path.dummy; path_src REPLAN (with rp.ok) serves rp.path as A.\nWindow trigger: Detect Change on ' ...
    '[seg_idx nxt_idx path_src rp.ver plan.valid] toggles a boolean that drives the Triggered Subsystem SegmentPrep ' ...
    '(TriggerType either, initial state zero, so it also runs at t = 0): two drivinglib/Path Smoother Spline ' ...
    '(NumSmoothPoses N_smooth, SampleTime -1, heading in degrees) and one drivinglib/Velocity Profiler on A ' ...
    '(per-segment profile, F4; StartVelocity = seg v_seg_max, the start ramp from standstill is the governor rate ' ...
    'limiter; EndVelocity = seg v_end), version counter, held Bus_PathWindow output.']));
end

function local_segprep(sp)
for s = {'A', 'B'}
    x = s{1};
    DTC(sp, ['d' x], 'double', ['rows' x '/1']);
    SEL(sp, ['ref' x], 'Select all,Index vector (dialog)', {''; '[1 2 3]'}, ['d' x '/1']);
    SEL(sp, ['dir' x], 'Select all,Index vector (dialog)', {''; '4'}, ['d' x '/1']);
    A(sp, 'drivinglib/Path Smoother Spline', ['Smoother' x], 'NumSmoothPoses', 'P.gnc.path.N_smooth', ...
        'MinSeparation', '1e-3', 'SampleTime', '-1', 'ShowOptionalOutports', 'on', 'SimulateUsing', 'Code generation');
    L(sp, ['ref' x '/1'], ['Smoother' x '/1']); L(sp, ['dir' x '/1'], ['Smoother' x '/2']);
    % the smoother outputs are variable-size (NumSmoothPoses rows at run time); a Selector with the fixed
    % index vector 1:N_smooth makes them fixed-size for the nonvirtual Bus_PathWindow
    SEL(sp, ['padP' x], 'Index vector (dialog),Index vector (dialog)', {'1:P.gnc.path.N_smooth'; '[1 2 3]'}, ...
        sprintf('Smoother%s/1', x));
    po = {'D', 's', 'k'};
    for j = 1:3
        SEL(sp, ['pad' po{j} x], 'Index vector (dialog)', {'1:P.gnc.path.N_smooth'}, sprintf('Smoother%s/%d', x, j + 1));
    end
    NM(sp, sprintf('Smoother%s/1', x), ['poses' x]);   NM(sp, sprintf('Smoother%s/2', x), ['dirs' x]);
    NM(sp, sprintf('Smoother%s/3', x), ['cumLen' x]);  NM(sp, sprintf('Smoother%s/4', x), ['curv' x]);
end
TERM(sp, 'padDB/1', 'T_dirB');
A(sp, 'drivinglib/Velocity Profiler', 'VelProfiler', 'MaxSpeed', 'P.gnc.spd.v_max', ...
    'MaxLongitudinalAccel', 'P.gnc.spd.a_acc', 'MaxLongitudinalDecel', 'P.gnc.spd.a_dec', ...
    'MaxLongitudinalJerk', 'P.gnc.spd.jerk', 'MaxLateralAccel', 'P.gnc.spd.a_lat', 'ShowOptionalOutport', 'off', ...
    'SampleTime', '-1', 'SimulateUsing', 'Code generation');
L(sp, 'padDA/1', 'VelProfiler/1'); L(sp, 'padsA/1', 'VelProfiler/2'); L(sp, 'padkA/1', 'VelProfiler/3');
% start / end velocity from the segment table row of A: [type dir length v_seg_max v_end ...]
SEL(sp, 'vstart', 'Index vector (dialog)', {'4'}, 'segA/1');
SEL(sp, 'vend', 'Index vector (dialog)', {'5'}, 'segA/1');
DTC(sp, 'vstart_d', 'double', 'vstart/1');  DTC(sp, 'vend_d', 'double', 'vend/1');
A(sp, 'simulink/Discontinuities/Saturation', 'vstart_s', 'UpperLimit', 'P.gnc.spd.v_max', 'LowerLimit', '0');
A(sp, 'simulink/Discontinuities/Saturation', 'vend_s', 'UpperLimit', 'P.gnc.spd.v_max', 'LowerLimit', '0');
L(sp, 'vstart_d/1', 'vstart_s/1'); L(sp, 'vend_d/1', 'vend_s/1');
% the profiler errors when Start / EndVelocity exceed sqrt(a_lat / max|kappa|) of the smoothed path:
% both are clipped to spd.v_prof_margin times that limit (smoothing can raise the curvature slightly)
A(sp, 'simulink/Math Operations/Abs', 'kabs');  L(sp, 'padkA/1', 'kabs/1');
A(sp, 'simulink/Math Operations/MinMax', 'kmax', 'Function', 'max', 'Inputs', '1');  L(sp, 'kabs/1', 'kmax/1');
A(sp, 'simulink/Discontinuities/Saturation', 'kpos', 'UpperLimit', 'inf', 'LowerLimit', '1e-9');  L(sp, 'kmax/1', 'kpos/1');
C(sp, 'a_lat', 'P.gnc.spd.a_lat', 'double', '-1');
A(sp, 'simulink/Math Operations/Divide', 'vlat2');  L(sp, 'a_lat/1', 'vlat2/1'); L(sp, 'kpos/1', 'vlat2/2');
A(sp, 'simulink/Math Operations/Sqrt', 'vlat');  L(sp, 'vlat2/1', 'vlat/1');
A(sp, 'simulink/Math Operations/Gain', 'vlat_m', 'Gain', 'P.gnc.spd.v_prof_margin');  L(sp, 'vlat/1', 'vlat_m/1');
A(sp, 'simulink/Math Operations/MinMax', 'vs_c', 'Function', 'min', 'Inputs', '2');
A(sp, 'simulink/Math Operations/MinMax', 've_c', 'Function', 'min', 'Inputs', '2');
L(sp, 'vstart_s/1', 'vs_c/1'); L(sp, 'vlat_m/1', 'vs_c/2'); L(sp, 'vend_s/1', 've_c/1'); L(sp, 'vlat_m/1', 've_c/2');
L(sp, 'vs_c/1', 'VelProfiler/4'); L(sp, 've_c/1', 'VelProfiler/5');
SEL(sp, 'padvA', 'Index vector (dialog)', {'1:P.gnc.path.N_smooth'}, 'VelProfiler/1');
% window version (+1 per run; the Unit Delay updates only when the subsystem runs)
A(sp, 'simulink/Math Operations/Sum', 'ver_add', 'Inputs', '++', 'OutDataTypeStr', 'uint16', 'SaturateOnIntegerOverflow', 'off');
A(sp, 'simulink/Discrete/Unit Delay', 'ver_z', 'InitialCondition', 'uint16(0)', 'SampleTime', '-1');
C(sp, 'one', 'uint16(1)', 'uint16', '-1');
L(sp, 'ver_z/1', 'ver_add/1'); L(sp, 'one/1', 'ver_add/2'); L(sp, 'ver_add/1', 'ver_z/1');
SEL(sp, 'typeA_s', 'Index vector (dialog)', {'1'}, 'segA/1');
SEL(sp, 'typeB_s', 'Index vector (dialog)', {'1'}, 'segB/1');
DTC(sp, 'typeA', 'uint8', 'typeA_s/1');  DTC(sp, 'typeB', 'uint8', 'typeB_s/1');
BC(sp, 'Window', 'Bus_PathWindow', {'PA', 'sA', 'kA', 'vA', 'PB', 'sB', 'kB', 'ver', 'segA', 'segB', 'typeA', 'typeB', 'valid'}, ...
    {'padPA/1', 'padsA/1', 'padkA/1', 'padvA/1', 'padPB/1', 'padsB/1', 'padkB/1', ...
     'ver_add/1', 'idxA/1', 'idxB/1', 'typeA/1', 'typeB/1', 'valid/1'});
L(sp, 'Window/1', 'win/1', 'win');
end

% =============================================================================================
%  gnc_PathAnalyzer
% =============================================================================================
function local_gnc_PathAnalyzer(n, f, T)
local_new_ssr(n, f, T.(n));
MFB(n, 'PathAnalyzer', sprintf('function pe = fcn(nav, win, pa)\n%%#codegen\npe = gnc_pathAnalyzer(nav, win, pa);\n'), ...
    {'nav', 'Bus: Bus_NavCond'; 'win', 'Bus: Bus_PathWindow'; 'pe', 'Bus: Bus_PathError'}, {'pa', 'P.gnc.pa'}, 'Ts.gnc', ...
    'Closest point, cross-track / heading error, progress, curvature preview (gnc spec 4.2); body subsystems/gnc/gnc_pathAnalyzer.m.');
L(n, 'nav/1', 'PathAnalyzer/1'); L(n, 'win/1', 'PathAnalyzer/2');
L(n, 'PathAnalyzer/1', 'pe/1', 'pe');
local_save(n, f, sprintf(['gnc_PathAnalyzer (T14, gnc spec 4.2). MATLAB Function gnc_pathAnalyzer (no library block ' ...
    'computes cross-track / heading error, arc-length progress and a curvature preview on a polyline): monotonic ' ...
    'local search i0-5 .. i0+60 reset on window version change, xte + = left, epsi = wrap(psi - psi_ref), s / s_rem, ' ...
    'profiled speed v_ref, curvature preview over the MPC horizon (A then B). Parameter P.gnc.pa (non-tunable).']));
end

% =============================================================================================
%  gnc_LocalAvoidance (stub)
% =============================================================================================
function local_gnc_LocalAvoidance(n, f, T)
local_new_ssr(n, f, T.(n));
for p = {'perception', 'nav', 'fsm', 'win', 'pe', 'tdir'}
    TERM(n, [p{1} '/1'], ['T_' p{1}]);
end
C(n, 'ObsIC', 'P.ic.Bus_ObsFlags', 'Bus: Bus_ObsFlags');
C(n, 'd_inf', 'inf', 'double');
C(n, 'scan_ok', 'true', 'boolean');                % a valid scan (the FSM holds on a lost one)
BA(n, 'Obs', 'ObsIC/1', {'d_obs', 'scan_valid'}, {'d_inf/1', 'scan_ok/1'});
L(n, 'Obs/1', 'obs/1', 'obs');
C(n, 'teb0', 'zeros(2,1)', 'double');  L(n, 'teb0/1', 'teb_cmd/1', 'teb_cmd');
C(n, 'info0', 'zeros(4,1)', 'double'); L(n, 'info0/1', 'teb_info/1', 'teb_info');
local_save(n, f, sprintf(['SHADDAD_GNC_STUB gnc_LocalAvoidance (T14 stub with the final ports; T15 implements corridor, ' ...
    'VFH, TEB behind VC_TEB, gnc spec 2.6). Stub: obs.d_obs = Inf (clear), blocked_vfh / person_near false, ' ...
    'teb_cmd = [v omega] = 0, teb_info = [ExitFlag LastFeasibleIdx HasReachedGoal TrajectoryCost] = 0. Inputs: ' ...
    'perception 0.1 s, nav, fsm, win, pe, tdir (Pure Pursuit target direction, delayed one step) 0.05 s.']));
end

% =============================================================================================
%  gnc_GlobalReplanner (stub)
% =============================================================================================
function local_gnc_GlobalReplanner(n, f, T)
local_new_ssr(n, f, T.(n));
for p = {'fsm', 'nav', 'perception', 'plan'}
    TERM(n, [p{1} '/1'], ['T_' p{1}]);
end
C(n, 'RpIC', 'P.ic.Bus_ReplanResult', 'Bus: Bus_ReplanResult');
L(n, 'RpIC/1', 'rp/1', 'rp');
local_save(n, f, sprintf(['SHADDAD_GNC_STUB gnc_GlobalReplanner (T14 stub with the final ports; T15 implements the ' ...
    'triggered gnc_globalReplanner Hybrid A*, gnc spec 4.4). Stub: rp = P.ic.Bus_ReplanResult (ok false).']));
end

% =============================================================================================
%  gnc_ImplementManager (stub)
% =============================================================================================
function local_gnc_ImplementManager(n, f, T)
local_new_ssr(n, f, T.(n));
for p = {'fsm', 'pe', 'nav', 'mcu', 'spray', 'plan', 'obs'}
    TERM(n, [p{1} '/1'], ['T_' p{1}]);
end
C(n, 'ImplIC', 'P.ic.Bus_GncCmd', 'Bus: Bus_GncCmd');
L(n, 'ImplIC/1', 'impl/1', 'impl');
local_save(n, f, sprintf(['SHADDAD_GNC_STUB gnc_ImplementManager (T14 stub with the final ports; T15 implements the ' ...
    'mower / tiller / sprayer laws, gnc spec 2.9). Output impl = Bus_GncCmd carrying the implement fields ' ...
    '(mower_*, deck_height_cmd, tiller_*, spray_*); CommandShaping assigns the motion / mode fields on top. ' ...
    'Stub: P.ic.Bus_GncCmd (implements off).']));
end

% =============================================================================================
%  gnc_SpeedGovernor
% =============================================================================================
function local_gnc_SpeedGovernor(n, f, T)
local_new_ssr(n, f, T.(n));
% one Bus Selector per consumer, right in front of it (layout spec 6.2)
% 1 v_prof: profiled speed at s (Velocity Profiler of the current segment)
PICK(n, 'pe/1', 'PeSel_vref', {'v_ref'});
% 2 v_seg: segment cruise speed plan.seg(segA, 4)
PICK(n, 'win/1', 'WinSel', {'segA'});
DTC(n, 'segA_d', 'double', 'WinSel/1');
C(n, 'one', '1', 'double');
A(n, 'simulink/Math Operations/MinMax', 'segA1', 'Function', 'max', 'Inputs', '2');   % index >= 1
L(n, 'segA_d/1', 'segA1/1'); L(n, 'one/1', 'segA1/2');
PICK(n, 'plan/1', 'PlanSel', {'seg'});
SEL(n, 'vseg_s', 'Index vector (port),Index vector (dialog)', {'1'; '4'}, 'PlanSel/1', 'segA1/1');
DTC(n, 'v_seg', 'double', 'vseg_s/1');
% 3 v_obs(d_obs), 4 v_xte(|e_y|), 5 v_head(|e_psi|), 6 v_pitch(|theta|), 7 v_roll(|phi|), 8 v_nav(sigma)
PICK(n, 'obs/1', 'ObsSel', {'d_obs'});
LUT(n, 'v_obs', 'P.gnc.spd.obs_d', 'P.gnc.spd.obs_v', 'ObsSel/1');
A(n, 'simulink/Math Operations/Abs', 'xte_abs');   PICK(n, 'pe/1', 'PeSel_xte', {'xte'}, {'xte_abs/1'});
LUT(n, 'v_xte', 'P.gnc.spd.xte_x', 'P.gnc.spd.xte_v', 'xte_abs/1');
A(n, 'simulink/Math Operations/Abs', 'epsi_abs');  PICK(n, 'pe/1', 'PeSel_epsi', {'epsi'}, {'epsi_abs/1'});
LUT(n, 'v_head', 'P.gnc.spd.head_x', 'P.gnc.spd.head_v', 'epsi_abs/1');
A(n, 'simulink/Math Operations/Abs', 'pitch_abs'); PICK(n, 'nav/1', 'NavSel_pitch', {'pitch'}, {'pitch_abs/1'});
LUT(n, 'v_pitch_lut', 'P.gnc.spd.pitch_x', 'P.gnc.spd.pitch_v', 'pitch_abs/1');
% downhill cap (P.structure.descent through P.gnc.desc, Agents task descent 2026-11-15): the cap at the nav pitch held
% over desc.t_hold and the look-ahead along the window (DEM chord pitch, braking curve); v_pitch = min(LUT, cap)
C(n, 'dem_grid', '[P.world.terrain.xE(1) P.world.terrain.yN(1) P.world.terrain.res]', 'double', 'inf');
C(n, 'dem_H', 'P.world.terrain.H', 'double', 'inf');
MFB(n, 'DescentCap', sprintf(['function v = fcn(nav, pe, win, impl, grid, H, desc)\n%%#codegen\n' ...
    'v = gnc_descentLimit(nav, pe, win, impl, grid, H, desc);\n']), ...
    {'nav', 'Bus: Bus_NavCond'; 'pe', 'Bus: Bus_PathError'; 'win', 'Bus: Bus_PathWindow'; 'v', 'double'}, ...
    {'desc', 'P.gnc.desc'}, 'Ts.gnc', ['Downhill speed limit: descent cap at the held nav pitch and along the path ' ...
    'ahead (DEM look-ahead); body subsystems/gnc/gnc_descentLimit.m.']);
L(n, 'nav/1', 'DescentCap/1'); L(n, 'pe/1', 'DescentCap/2'); L(n, 'win/1', 'DescentCap/3');
PICK(n, 'plan/1', 'PlanSel_impl', {'implement_type'}, {'DescentCap/4'});
L(n, 'dem_grid/1', 'DescentCap/5'); L(n, 'dem_H/1', 'DescentCap/6');
A(n, 'simulink/Math Operations/MinMax', 'v_pitch', 'Function', 'min', 'Inputs', '2');
L(n, 'v_pitch_lut/1', 'v_pitch/1'); L(n, 'DescentCap/1', 'v_pitch/2', 'v_desc');
A(n, 'simulink/Math Operations/Abs', 'roll_abs');  PICK(n, 'nav/1', 'NavSel_roll', {'roll'}, {'roll_abs/1'});
LUT(n, 'v_roll', 'P.gnc.spd.roll_x', 'P.gnc.spd.roll_v', 'roll_abs/1');
PICK(n, 'nav/1', 'NavSel_sigma', {'sigma_pos'});
LUT(n, 'v_nav', 'P.gnc.spd.nav_x', 'P.gnc.spd.nav_v', 'NavSel_sigma/1');
% 9 v_soc: valid SOC below soc_low -> v_soc_low
PICK(n, 'bms/1', 'BmsSel_soc', {'SOC'});
DTC(n, 'soc_d', 'double', 'BmsSel_soc/1');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'soc_low', 'relop', '<', 'const', 'P.gnc.spd.soc_low');
L(n, 'soc_d/1', 'soc_low/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'soc_lim', 'Operator', 'AND', 'Inputs', '2');
L(n, 'soc_low/1', 'soc_lim/1'); PICK(n, 'bms/1', 'BmsSel_valid', {'valid'}, {'soc_lim/2'});
C(n, 'v_soc_low', 'P.gnc.spd.v_soc_low', 'double');
C(n, 'v_max', 'P.gnc.spd.v_max', 'double');
A(n, 'simulink/Signal Routing/Switch', 'v_soc', 'Criteria', 'u2 ~= 0');
L(n, 'v_soc_low/1', 'v_soc/1'); L(n, 'soc_lim/1', 'v_soc/2'); L(n, 'v_max/1', 'v_soc/3');
% 10 v_mode = fsm.v_cap; 11 v_override = speed_override * v_max (0 = no override)
PICK(n, 'fsm/1', 'FsmSel', {'v_cap'});
PICK(n, 'mcmd/1', 'CmdSel', {'speed_override'});
DTC(n, 'ovr_d', 'double', 'CmdSel/1');
A(n, 'simulink/Math Operations/Gain', 'ovr_v', 'Gain', 'P.gnc.spd.v_max');  L(n, 'ovr_d/1', 'ovr_v/1');
A(n, 'simulink/Signal Routing/Switch', 'v_override', 'Criteria', 'u2 > Threshold', 'Threshold', '0');
L(n, 'ovr_v/1', 'v_override/1'); L(n, 'ovr_d/1', 'v_override/2'); L(n, 'v_max/1', 'v_override/3');
% 12 v_veg(3 * spray.target_density) (D46), only with a valid detector
PICK(n, 'spray/1', 'SprSel_dens', {'target_density'});
DTC(n, 'dens_d', 'double', 'SprSel_dens/1');
A(n, 'simulink/Math Operations/Gain', 'veg_x', 'Gain', '3');  L(n, 'dens_d/1', 'veg_x/1');
C(n, 'zero', '0', 'double');
A(n, 'simulink/Signal Routing/Switch', 'veg_in', 'Criteria', 'u2 ~= 0');
L(n, 'veg_x/1', 'veg_in/1'); PICK(n, 'spray/1', 'SprSel_valid', {'valid'}, {'veg_in/2'}); L(n, 'zero/1', 'veg_in/3');
LUT(n, 'v_veg', 'P.gnc.spd.veg_x', 'P.gnc.spd.veg_v', 'veg_in/1');
% the 12 limits, in the order of v_limits, travel as one virtual bus to their two consumers (layout spec 6.3); a Bus
% Selector output keeps its element name, so a Signal Conversion gives the two picked limits their bus element names
for s = {'v_ref', 'PeSel_vref/1'; 'v_cap', 'FsmSel/1'}'
    A(n, 'simulink/Signal Attributes/Signal Conversion', ['Lim_' s{1}]);
    L(n, s{2}, ['Lim_' s{1} '/1']);
end
lim = {'v_ref', 'Lim_v_ref/1'; 'v_seg', 'v_seg/1'; 'v_obs', 'v_obs/1'; 'v_xte', 'v_xte/1'; 'v_head', 'v_head/1'; ...
       'v_pitch', 'v_pitch/1'; 'v_roll', 'v_roll/1'; 'v_nav', 'v_nav/1'; 'v_soc', 'v_soc/1'; 'v_cap', 'Lim_v_cap/1'; ...
       'v_override', 'v_override/1'; 'v_veg', 'v_veg/1'};
shaddad_bu('vbus', n, 'Limits', lim);
A(n, 'simulink/Math Operations/MinMax', 'v_min', 'Function', 'min', 'Inputs', '12');
A(n, 'simulink/Signal Routing/Mux', 'lim_vec', 'Inputs', '12');
PICK(n, 'Limits/1', 'LimSel_min', lim(:, 1)', compose('v_min/%d', 1:12), 'limits');
PICK(n, 'Limits/1', 'LimSel_vec', lim(:, 1)', compose('lim_vec/%d', 1:12));
DTC(n, 'lim_s', 'single', 'lim_vec/1');  L(n, 'lim_s/1', 'v_limits/1', 'v_limits');
A(n, 'simulink/Discontinuities/Saturation', 'v_sat', 'UpperLimit', 'P.gnc.spd.v_max', 'LowerLimit', '0');
L(n, 'v_min/1', 'v_sat/1');
% accel / decel shaping (rates in m/s^2)
C(n, 'a_up', 'P.gnc.spd.a_acc', 'double');
C(n, 'a_lo', '-P.gnc.spd.a_dec', 'double');
A(n, 'simulink/Discontinuities/Rate Limiter Dynamic', 'RateLim');
L(n, 'a_up/1', 'RateLim/1'); L(n, 'v_sat/1', 'RateLim/2'); L(n, 'a_lo/1', 'RateLim/3');
L(n, 'RateLim/1', 'v_gov/1', 'v_gov');
PICK(n, 'pe/1', 'PeSel_prof', {'v_ref'}, {'v_prof/1'});
% stopping-distance consistency d_stop_req = |v| t_lat + v^2 / (2 a_dec) (spec 2.5, D33)
A(n, 'simulink/Math Operations/Abs', 'v_abs');  PICK(n, 'nav/1', 'NavSel_v', {'v'}, {'v_abs/1'});
A(n, 'simulink/Math Operations/Gain', 'd_lat', 'Gain', 'P.gnc.spd.t_lat');  L(n, 'v_abs/1', 'd_lat/1');
A(n, 'simulink/Math Operations/Product', 'v2', 'Inputs', '**');  L(n, 'v_abs/1', 'v2/1'); L(n, 'v_abs/1', 'v2/2');
A(n, 'simulink/Math Operations/Gain', 'd_brk', 'Gain', '1/(2*P.gnc.spd.a_dec)');  L(n, 'v2/1', 'd_brk/1');
A(n, 'simulink/Math Operations/Sum', 'd_stop', 'Inputs', '++');
L(n, 'd_lat/1', 'd_stop/1'); L(n, 'd_brk/1', 'd_stop/2');
L(n, 'd_stop/1', 'd_stop_req/1', 'd_stop_req');
local_save(n, f, sprintf(['gnc_SpeedGovernor (T14, gnc spec 2.5, D33, D46). v_gov = RateLimiterDynamic(sat_[0, v_max](min of 12 ' ...
    'limits), +a_acc, -a_dec) with the limits (v_limits order): v_prof (Velocity Profiler at s), v_seg_max ' ...
    '(plan.seg(segA,4)), v_obs(d_obs), v_xte(|e_y|), v_head(|e_psi|), v_pitch = min(LUT(|theta|), DescentCap: the ' ...
    'downhill cap P.gnc.desc (P.structure.descent, by plan.implement_type) at the nav pitch held over desc.t_hold and ' ...
    'along the window ahead: the DEM chord pitch over the contact length every desc.ds to desc.d_look, braking curve ' ...
    'sqrt(cap^2 + 2 a_la (d - v t_resp)), gnc_descentLimit), v_roll(|phi|), v_nav(sigma_pos), ' ...
    'v_soc (valid SOC < soc_low -> v_soc_low), v_mode = fsm.v_cap, v_override = speed_override*v_max (0 = none), ' ...
    'v_veg(3*spray.target_density) (valid detector only). 1-D Lookup Tables linear, clipped, tables from P.gnc.spd. ' ...
    'd_stop_req = |v| t_lat + v^2/(2 a_dec) (logged consistency check of the v_obs table).']));
end

% =============================================================================================
%  gnc_PathTracker
% =============================================================================================
function local_gnc_PathTracker(n, f, T)
local_new_ssr(n, f, T.(n));
% one Bus Selector per consumer, right in front of it (layout spec 6.2)
% ---- Pure Pursuit on the 799 x 2 window [A; B(2:end)] ----
PICK(n, 'win/1', 'WinSel_A', {'PA'});
PICK(n, 'win/1', 'WinSel_B', {'PB'});
SEL(n, 'wpA', 'Select all,Index vector (dialog)', {''; '[1 2]'}, 'WinSel_A/1');
SEL(n, 'wpB', 'Index vector (dialog),Index vector (dialog)', {'2:P.gnc.path.N_smooth'; '[1 2]'}, 'WinSel_B/1');
A(n, 'simulink/Signal Routing/Vector Concatenate', 'Waypoints', 'NumInputs', '2', 'ConcatenateDimension', '1', ...
    'Mode', 'Multidimensional array');
L(n, 'wpA/1', 'Waypoints/1'); L(n, 'wpB/1', 'Waypoints/2');
A(n, 'simulink/Math Operations/Abs', 'v_abs');  PICK(n, 'nav/1', 'NavSel_v', {'v'}, {'v_abs/1'});
LUT(n, 'Ld', 'P.gnc.pp.Ld_v', 'P.gnc.pp.Ld', 'v_abs/1');
A(n, 'robotmobilelib/Pure Pursuit', 'PurePursuit', 'DesiredLinearVelocityPort', 'on', 'LookaheadDistancePort', 'on', ...
    'TargetDirPort', 'on', 'MaxCurvature', 'P.gnc.pp.kappa_max', 'SimulateUsing', 'Code generation');
PICK(n, 'nav/1', 'NavSel_pose', {'pose'}, {'PurePursuit/1'}); L(n, 'Waypoints/1', 'PurePursuit/2');
L(n, 'v_gov/1', 'PurePursuit/3'); L(n, 'Ld/1', 'PurePursuit/4');
A(n, 'simulink/Math Operations/Product', 'w_pp_p', 'Inputs', '**');      % omega = v * curvature (F3)
L(n, 'PurePursuit/1', 'w_pp_p/1', 'v_pp'); L(n, 'PurePursuit/2', 'w_pp_p/2', 'kappa_pp');
% ---- RowTracker variant (VC_ROW_TRACKER) ----
vs = [n '/RowTracker'];
A(n, 'simulink/Ports & Subsystems/Variant Subsystem', 'RowTracker');
c = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
c = setdiff(c, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
rin = {'nav', 'pe', 'w_applied'};  rout = {'w_row', 'qp_status'};
for k = 1:numel(rin), A(vs, 'simulink/Sources/In1', rin{k}, 'Port', num2str(k)); end
for k = 1:numel(rout), A(vs, 'simulink/Sinks/Out1', rout{k}, 'Port', num2str(k)); end
A(vs, 'simulink/Ports & Subsystems/Subsystem', 'ROW_MPC', 'VariantControl', 'VC_ROW_TRACKER == 1');
A(vs, 'simulink/Ports & Subsystems/Subsystem', 'ROW_STANLEY', 'VariantControl', 'VC_ROW_TRACKER == 2');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
local_row_mpc([vs '/ROW_MPC'], rin, rout);
local_row_stanley([vs '/ROW_STANLEY'], rin, rout);
L(n, 'nav/1', 'RowTracker/1'); L(n, 'pe/1', 'RowTracker/2'); L(n, 'w_applied/1', 'RowTracker/3');
% ---- Align (pivot): w = sat(-K e_psi, +/- w_max), inhibited on tilt (D53) ----
A(n, 'simulink/Math Operations/Gain', 'al_gain', 'Gain', '-P.gnc.align.K');  PICK(n, 'pe/1', 'PeSel', {'epsi'}, {'al_gain/1'});
A(n, 'simulink/Discontinuities/Saturation', 'al_sat', 'UpperLimit', 'P.gnc.align.w_max', 'LowerLimit', '-P.gnc.align.w_max');
L(n, 'al_gain/1', 'al_sat/1');
A(n, 'simulink/Math Operations/Abs', 'pitch_abs');  PICK(n, 'nav/1', 'NavSel_pitch', {'pitch'}, {'pitch_abs/1'});
A(n, 'simulink/Math Operations/Abs', 'roll_abs');   PICK(n, 'nav/1', 'NavSel_roll', {'roll'}, {'roll_abs/1'});
A(n, 'simulink/Math Operations/MinMax', 'tilt', 'Function', 'max', 'Inputs', '2');
L(n, 'pitch_abs/1', 'tilt/1'); L(n, 'roll_abs/1', 'tilt/2');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'tilt_ok', 'relop', '<=', 'const', 'P.gnc.align.pitch_inhibit');
L(n, 'tilt/1', 'tilt_ok/1');
C(n, 'zero', '0', 'double');
A(n, 'simulink/Signal Routing/Switch', 'w_align', 'Criteria', 'u2 ~= 0');
L(n, 'al_sat/1', 'w_align/1'); L(n, 'tilt_ok/1', 'w_align/2'); L(n, 'zero/1', 'w_align/3');
% ---- TEB command [v omega] ----
SEL(n, 'teb_v', 'Index vector (dialog)', {'1'}, 'teb_cmd/1');
SEL(n, 'teb_w', 'Index vector (dialog)', {'2'}, 'teb_cmd/1');
% ---- TrackerSelect (TrackerSel: 1 PP, 2 ROW, 3 TEB, 4 ALIGN, 0 STOP = default / last port) ----
for s = {'v', 'w'}
    x = s{1};
    A(n, 'simulink/Signal Routing/Multiport Switch', ['sel_' x], 'Inputs', '5', 'DataPortOrder', 'Specify indices', ...
        'DataPortIndices', '{1,2,3,4,0}', 'DataPortForDefault', 'Last data port', 'DiagnosticForDefault', 'None');
    PICK(n, 'fsm/1', ['FsmSel_' x], {'trk_sel'}, {['sel_' x '/1']});
end
L(n, 'PurePursuit/1', 'sel_v/2'); L(n, 'v_gov/1', 'sel_v/3'); L(n, 'teb_v/1', 'sel_v/4'); L(n, 'zero/1', 'sel_v/5'); L(n, 'zero/1', 'sel_v/6');
L(n, 'w_pp_p/1', 'sel_w/2'); L(n, 'RowTracker/1', 'sel_w/3'); L(n, 'teb_w/1', 'sel_w/4'); L(n, 'w_align/1', 'sel_w/5'); L(n, 'zero/1', 'sel_w/6');
L(n, 'sel_v/1', 'v_trk/1', 'v_trk'); L(n, 'sel_w/1', 'w_trk/1', 'w_trk');
L(n, 'PurePursuit/3', 'tdir/1', 'tdir'); L(n, 'w_pp_p/1', 'w_pp/1', 'w_pp');
L(n, 'RowTracker/1', 'w_row/1', 'w_row'); L(n, 'RowTracker/2', 'qp_status/1', 'qp_status');
local_save(n, f, sprintf(['gnc_PathTracker (T14, gnc spec 2.4 / 7, F3, F5, F8). PurePursuit: robotmobilelib/Pure Pursuit ' ...
    '(Desired Linear Velocity = v_gov, Lookahead Distance = LUT(|v|) P.gnc.pp.Ld_v/Ld, Target Direction out, ' ...
    'MaxCurvature P.gnc.pp.kappa_max) on the 799 x 2 window [PA; PB(2:end)], omega_pp = v * curvature.\n' ...
    'RowTracker Variant Subsystem VC_ROW_TRACKER (update diagram): 1 ROW_MPC = MATLAB Function gnc_mpcPlantUpdate ' ...
    '(LTV model with the 2-sample input delay, D41) + mpclib/Adaptive MPC Controller (mpcobj mpcRowObj, mo = [xte; ' ...
    'epsi], ref 0, md = curvature preview [p x 1], ext.mv = applied omega of the previous step, online MV bounds ' ...
    'umin/umax = -/+ min(omega_max(|v|), omega_max(tilt), u_max) as in CommandShaping, qp.status; omega scaled by ' ...
    'min(1, |v|/v_engage)); 2 ROW_STANLEY = drivingvehiclecontroller/Lateral Controller Stanley (kinematic; CurrPose = ' ...
    'base pose - Lv [cos sin] because the block adds the wheelbase itself, gain k_block = k (1+v_nom)/v_nom compensates ' ...
    'the block''s v + 1 m/s softening, degrees at the boundary) -> omega = v tan(delta)/Lv.\nAlign: omega = ' ...
    'sat(-K e_psi, +/-w_max), 0 when max(|pitch|,|roll|) > ' ...
    'align.pitch_inhibit (D53). TrackerSelect: Multiport Switch on fsm.trk_sel (1 PP, 2 ROW, 3 TEB, 4 ALIGN, default ' ...
    'STOP 0) for v and omega.']));
end

function local_row_mpc(s, rin, rout)
local_clear(s);
for k = 1:numel(rin), A(s, 'simulink/Sources/In1', rin{k}, 'Port', num2str(k)); end
for k = 1:numel(rout), A(s, 'simulink/Sinks/Out1', rout{k}, 'Port', num2str(k)); end
MFB(s, 'PlantUpdate', sprintf('function mdl = fcn(v, mpc)\n%%#codegen\nmdl = gnc_mpcPlantUpdate(v, mpc);\n'), ...
    {'mdl', 'Bus: Bus_MpcModel'}, {'mpc', 'P.gnc.mpc'}, 'Ts.gnc', ...
    'LTV prediction model for the Adaptive MPC model inport (gnc spec 4.3, D41); body subsystems/gnc/gnc_mpcPlantUpdate.m.');
PICK(s, 'nav/1', 'NavSel_mdl', {'v'}, {'PlantUpdate/1'});
A(s, 'simulink/Signal Routing/Mux', 'mo', 'Inputs', '2');
PICK(s, 'pe/1', 'PeSel_mo', {'xte', 'epsi'}, {'mo/1', 'mo/2'});
C(s, 'ref', 'zeros(1,2)', 'double');
A(s, 'mpclib/Adaptive MPC Controller', 'AdaptiveMPC', 'mpcobj', 'mpcRowObj', 'md_inport', 'on', 'mv_inport', 'on', ...
    'umin_inport', 'on', 'umax_inport', 'on', 'return_qpstatus', 'on', 'BlockDataType', 'double', 'SampleTimeInherited', 'off');
% online MV bounds = the CommandShaping omega envelope (omega_max(|v|), omega_max(tilt), mpc.u_max), so the
% MPC plans with the limits that will actually be applied (no saturation-induced overshoot)
A(s, 'simulink/Math Operations/Abs', 'v_abs_u');  PICK(s, 'nav/1', 'NavSel_v', {'v'}, {'v_abs_u/1'});
LUT(s, 'w_max_v', 'P.gnc.cmd.w_max_v', 'P.gnc.cmd.w_max', 'v_abs_u/1');
A(s, 'simulink/Math Operations/Abs', 'pitch_abs');  PICK(s, 'nav/1', 'NavSel_pitch', {'pitch'}, {'pitch_abs/1'});
A(s, 'simulink/Math Operations/Abs', 'roll_abs');   PICK(s, 'nav/1', 'NavSel_roll', {'roll'}, {'roll_abs/1'});
A(s, 'simulink/Math Operations/MinMax', 'tilt', 'Function', 'max', 'Inputs', '2');
L(s, 'pitch_abs/1', 'tilt/1'); L(s, 'roll_abs/1', 'tilt/2');
LUT(s, 'w_max_tilt', 'P.gnc.cmd.w_tilt_x', 'P.gnc.cmd.w_tilt', 'tilt/1');
C(s, 'u_max', 'P.gnc.mpc.u_max', 'double');
A(s, 'simulink/Math Operations/MinMax', 'umax', 'Function', 'min', 'Inputs', '3');
L(s, 'w_max_v/1', 'umax/1'); L(s, 'w_max_tilt/1', 'umax/2'); L(s, 'u_max/1', 'umax/3');
A(s, 'simulink/Math Operations/Gain', 'umin', 'Gain', '-1');  L(s, 'umax/1', 'umin/1');
L(s, 'umin/1', 'AdaptiveMPC/6'); L(s, 'umax/1', 'AdaptiveMPC/7');
L(s, 'PlantUpdate/1', 'AdaptiveMPC/1'); L(s, 'mo/1', 'AdaptiveMPC/2'); L(s, 'ref/1', 'AdaptiveMPC/3');
PICK(s, 'pe/1', 'PeSel_kappa', {'kappa_prev'}, {'AdaptiveMPC/4'}); L(s, 'w_applied/1', 'AdaptiveMPC/5');
% engagement: omega scaled by min(1, |v|/v_engage) (the Frenet model has no lateral authority at
% standstill; without it the MPC pivots the robot on the row). ext.mv carries the applied omega.
A(s, 'simulink/Math Operations/Gain', 'v_rel', 'Gain', '1/P.gnc.mpc.v_engage');  L(s, 'v_abs_u/1', 'v_rel/1');
A(s, 'simulink/Discontinuities/Saturation', 'k_eng', 'UpperLimit', '1', 'LowerLimit', '0');  L(s, 'v_rel/1', 'k_eng/1');
A(s, 'simulink/Math Operations/Product', 'w_eng', 'Inputs', '**');
L(s, 'AdaptiveMPC/1', 'w_eng/1', 'w_mpc'); L(s, 'k_eng/1', 'w_eng/2');
L(s, 'w_eng/1', 'w_row/1', 'w_row');
DTC(s, 'qp_i32', 'int32', 'AdaptiveMPC/2');
NM(s, 'AdaptiveMPC/2', 'qp_mpc');
L(s, 'qp_i32/1', 'qp_status/1', 'qp_status');
end

function local_row_stanley(s, rin, rout)
local_clear(s);
for k = 1:numel(rin), A(s, 'simulink/Sources/In1', rin{k}, 'Port', num2str(k)); end
for k = 1:numel(rout), A(s, 'simulink/Sinks/Out1', rout{k}, 'Port', num2str(k)); end
TERM(s, 'w_applied/1', 'T_wa');
% reference pose [x y deg] = closest path point
A(s, 'simulink/Math Operations/Gain', 'ref_deg', 'Gain', '180/pi');  PICK(s, 'pe/1', 'PeSel_psi', {'psi_ref'}, {'ref_deg/1'});
A(s, 'simulink/Signal Routing/Vector Concatenate', 'RefPose', 'NumInputs', '2', 'Mode', 'Vector');
PICK(s, 'pe/1', 'PeSel_xy', {'xy_ref'}, {'RefPose/1'}); L(s, 'ref_deg/1', 'RefPose/2');
% the block treats CurrPose as the rear axle and adds Wheelbase internally (verified): CurrPose = base pose
% - Lv [cos psi, sin psi] puts its front-axle error point on base_link (spec 2.4 law: delta = e_psi + atan(k e_y / v))
PICK(s, 'nav/1', 'NavSel_pose', {'pose'});
SEL(s, 'xy', 'Index vector (dialog)', {'[1 2]'}, 'NavSel_pose/1');
SEL(s, 'psi', 'Index vector (dialog)', {'3'}, 'NavSel_pose/1');
A(s, 'simulink/Math Operations/Trigonometric Function', 'cpsi', 'Operator', 'cos');  L(s, 'psi/1', 'cpsi/1');
A(s, 'simulink/Math Operations/Trigonometric Function', 'spsi', 'Operator', 'sin');  L(s, 'psi/1', 'spsi/1');
A(s, 'simulink/Signal Routing/Mux', 'u_psi', 'Inputs', '2');  L(s, 'cpsi/1', 'u_psi/1'); L(s, 'spsi/1', 'u_psi/2');
A(s, 'simulink/Math Operations/Gain', 'lv', 'Gain', 'P.gnc.stan.Lv');  L(s, 'u_psi/1', 'lv/1');
A(s, 'simulink/Math Operations/Sum', 'front', 'Inputs', '+-');  L(s, 'xy/1', 'front/1'); L(s, 'lv/1', 'front/2');
A(s, 'simulink/Math Operations/Gain', 'psi_deg', 'Gain', '180/pi');  L(s, 'psi/1', 'psi_deg/1');
A(s, 'simulink/Signal Routing/Vector Concatenate', 'CurrPose', 'NumInputs', '2', 'Mode', 'Vector');
L(s, 'front/1', 'CurrPose/1'); L(s, 'psi_deg/1', 'CurrPose/2');
A(s, 'simulink/Discontinuities/Saturation', 'v_pos', 'UpperLimit', 'inf', 'LowerLimit', 'P.gnc.mpc.v_min_model');
PICK(s, 'nav/1', 'NavSel_v', {'v'}, {'v_pos/1'});
C(s, 'dir', '1', 'double');
A(s, 'drivingvehiclecontroller/Lateral Controller Stanley', 'Stanley', 'ModelType', 'Kinematic bicycle model', ...
    'PositionGainF', 'P.gnc.stan.k_block', 'PositionGainR', 'P.gnc.stan.k_block', 'Wheelbase', 'P.gnc.stan.Lv', ...
    'MaxSteeringAngle', 'P.gnc.stan.delta_max_deg');
L(s, 'RefPose/1', 'Stanley/1'); L(s, 'CurrPose/1', 'Stanley/2'); L(s, 'v_pos/1', 'Stanley/3'); L(s, 'dir/1', 'Stanley/4');
% omega = v tan(delta) / Lv
A(s, 'simulink/Math Operations/Gain', 'delta_rad', 'Gain', 'pi/180');  L(s, 'Stanley/1', 'delta_rad/1');
A(s, 'simulink/Math Operations/Trigonometric Function', 'tan_d', 'Operator', 'tan');  L(s, 'delta_rad/1', 'tan_d/1');
A(s, 'simulink/Math Operations/Product', 'v_tan', 'Inputs', '**');
PICK(s, 'nav/1', 'NavSel_vt', {'v'}, {'v_tan/1'}); L(s, 'tan_d/1', 'v_tan/2');
A(s, 'simulink/Math Operations/Gain', 'inv_lv', 'Gain', '1/P.gnc.stan.Lv');  L(s, 'v_tan/1', 'inv_lv/1');
L(s, 'inv_lv/1', 'w_row/1', 'w_row');
C(s, 'qp0', 'int32(0)', 'int32');
L(s, 'qp0/1', 'qp_status/1', 'qp_status');
end

% =============================================================================================
%  gnc_CommandShaping
% =============================================================================================
function local_gnc_CommandShaping(n, f, T)
local_new_ssr(n, f, T.(n));
% one Bus Selector per bus: with picks per consumer of fsm and nav the trunks of their port lines share one track with each
% other and with v_trk's (22 line overlaps, which only an engine change would remove; task 13 decisions 7)
BS(n, 'FsmSel', {'cmd_valid', 'mode_req', 'state', 'fault_word'});  L(n, 'fsm/1', 'FsmSel/1');
BS(n, 'NavSel', {'t', 'v', 'pitch', 'roll', 'nav_ok', 'gnss_fix'}); L(n, 'nav/1', 'NavSel/1');
% omega limits: omega_max(|v|) and omega_max(tilt) (D53)
A(n, 'simulink/Math Operations/Abs', 'v_abs');  L(n, 'v_trk/1', 'v_abs/1');
LUT(n, 'w_max_v', 'P.gnc.cmd.w_max_v', 'P.gnc.cmd.w_max', 'v_abs/1');
A(n, 'simulink/Math Operations/Abs', 'pitch_abs');  L(n, 'NavSel/3', 'pitch_abs/1');
A(n, 'simulink/Math Operations/Abs', 'roll_abs');   L(n, 'NavSel/4', 'roll_abs/1');
A(n, 'simulink/Math Operations/MinMax', 'tilt', 'Function', 'max', 'Inputs', '2');
L(n, 'pitch_abs/1', 'tilt/1'); L(n, 'roll_abs/1', 'tilt/2');
LUT(n, 'w_max_tilt', 'P.gnc.cmd.w_tilt_x', 'P.gnc.cmd.w_tilt', 'tilt/1');
A(n, 'simulink/Math Operations/MinMax', 'w_lim', 'Function', 'min', 'Inputs', '2');
L(n, 'w_max_v/1', 'w_lim/1'); L(n, 'w_max_tilt/1', 'w_lim/2');
A(n, 'simulink/Math Operations/Gain', 'w_lim_n', 'Gain', '-1');  L(n, 'w_lim/1', 'w_lim_n/1');
% omega rate limit, then the dynamic omega bound (the bound always holds at the output)
A(n, 'simulink/Discontinuities/Rate Limiter', 'w_rate', 'RisingSlewLimit', 'P.gnc.cmd.w_rate', ...
    'FallingSlewLimit', '-P.gnc.cmd.w_rate', 'SampleTimeMode', 'inherited', 'InitialCondition', '0');
L(n, 'w_trk/1', 'w_rate/1');
A(n, 'simulink/Discontinuities/Saturation Dynamic', 'w_sat');
L(n, 'w_lim/1', 'w_sat/1'); L(n, 'w_rate/1', 'w_sat/2'); L(n, 'w_lim_n/1', 'w_sat/3');
A(n, 'simulink/Discontinuities/Saturation', 'v_sat', 'UpperLimit', 'P.gnc.spd.v_max', 'LowerLimit', 'P.gnc.cmd.v_min_reverse');
L(n, 'v_trk/1', 'v_sat/1');
% downhill cap at the nav pitch (P.gnc.desc, Agents task descent 2026-11-15), forward and reverse: v scaled to the cap,
% omega with it (curvature kept); the tiller's reverse cap is 0 at any slope (gnc_descentEnvelope)
MFB(n, 'DescentEnv', sprintf(['function [v, w] = fcn(v_in, w_in, pitch, impl, desc)\n%%#codegen\n' ...
    '[v, w] = gnc_descentEnvelope(v_in, w_in, pitch, impl, desc);\n']), {'v', 'double'; 'w', 'double'}, ...
    {'desc', 'P.gnc.desc'}, 'Ts.gnc', ['Downhill speed cap at the nav pitch, forward and reverse, omega scaled with v; ' ...
    'body subsystems/gnc/gnc_descentEnvelope.m.']);
L(n, 'v_sat/1', 'DescentEnv/1'); L(n, 'w_sat/1', 'DescentEnv/2'); L(n, 'NavSel/3', 'DescentEnv/3');
PICK(n, 'plan/1', 'PlanSel_impl', {'implement_type'}, {'DescentEnv/4'});
% track-speed envelope: k = min(1, v_track_max / (|v| + |w| chi B/2)), v and w scaled by k (curvature kept, D43)
A(n, 'simulink/Math Operations/Abs', 'v1_abs');  L(n, 'DescentEnv/1', 'v1_abs/1', 'v_desc');
A(n, 'simulink/Math Operations/Abs', 'w1_abs');  L(n, 'DescentEnv/2', 'w1_abs/1', 'w_desc');
A(n, 'simulink/Math Operations/Gain', 'w_track', 'Gain', 'P.gnc.cmd.chi*P.gnc.cmd.B/2');  L(n, 'w1_abs/1', 'w_track/1');
A(n, 'simulink/Math Operations/Sum', 'v_track', 'Inputs', '++');  L(n, 'v1_abs/1', 'v_track/1'); L(n, 'w_track/1', 'v_track/2');
A(n, 'simulink/Discontinuities/Saturation', 'v_track_pos', 'UpperLimit', 'inf', 'LowerLimit', '1e-6');
L(n, 'v_track/1', 'v_track_pos/1');
C(n, 'vt_max', 'P.gnc.cmd.v_track_max', 'double');
A(n, 'simulink/Math Operations/Divide', 'k_ratio');  L(n, 'vt_max/1', 'k_ratio/1'); L(n, 'v_track_pos/1', 'k_ratio/2');
C(n, 'one', '1', 'double');
A(n, 'simulink/Math Operations/MinMax', 'k_env', 'Function', 'min', 'Inputs', '2');
L(n, 'k_ratio/1', 'k_env/1'); L(n, 'one/1', 'k_env/2');
A(n, 'simulink/Math Operations/Product', 'v2', 'Inputs', '**');  L(n, 'DescentEnv/1', 'v2/1'); L(n, 'k_env/1', 'v2/2');
A(n, 'simulink/Math Operations/Product', 'w2', 'Inputs', '**');  L(n, 'DescentEnv/2', 'w2/1'); L(n, 'k_env/1', 'w2/2');
% zero command unless fsm.cmd_valid
C(n, 'zero', '0', 'double');
A(n, 'simulink/Signal Routing/Switch', 'v3', 'Criteria', 'u2 ~= 0');
L(n, 'v2/1', 'v3/1'); L(n, 'FsmSel/1', 'v3/2'); L(n, 'zero/1', 'v3/3');
A(n, 'simulink/Signal Routing/Switch', 'w3', 'Criteria', 'u2 ~= 0');
L(n, 'w2/1', 'w3/1'); L(n, 'FsmSel/1', 'w3/2'); L(n, 'zero/1', 'w3/3');
L(n, 'w3/1', 'w_applied/1', 'w_applied');
DTC(n, 'v_cmd', 'single', 'v3/1');  DTC(n, 'omega_cmd', 'single', 'w3/1');
% heartbeat, readiness, mode-derived fields
A(n, 'simulink/Sources/Counter Free-Running', 'seq', 'NumBits', '8', 'tsamp', 'Ts.gnc');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'no_fault', 'relop', '==', 'const', '0');
L(n, 'FsmSel/4', 'no_fault/1');
A(n, 'simulink/Logic and Bit Operations/Logical Operator', 'gnc_ready', 'Operator', 'AND', 'Inputs', '3');
L(n, 'NavSel/5', 'gnc_ready/1'); PICK(n, 'plan/1', 'PlanSel', {'valid'}, {'gnc_ready/2'}); L(n, 'no_fault/1', 'gnc_ready/3');
C(n, 'accel_limit', 'single(P.gnc.cmd.accel_limit)', 'single');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'soft_stop', 'relop', '==', 'const', 'double(GncModeReq.HOLD)');
L(n, 'FsmSel/2', 'soft_stop/1');
A(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'buzzer', 'relop', '==', 'const', 'double(GncModeReq.FAULT_STOP)');
L(n, 'FsmSel/2', 'buzzer/1');
DTC(n, 'mode_d', 'double', 'FsmSel/2');
A(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'beacon_tab', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.gnc.cmd.beacon_by_mode', 'DiagnosticForOutOfRangeInput', 'None');
L(n, 'mode_d/1', 'beacon_tab/1');
DTC(n, 'beacon', 'uint8', 'beacon_tab/1');
DTC(n, 'v_ground', 'single', 'NavSel/2');
BA(n, 'Cmd', 'impl/1', {'seq', 't_stamp', 'cmd_valid', 'gnc_ready', 'mode_req', 'gnc_state', 'v_cmd', 'omega_cmd', ...
    'accel_limit', 'soft_stop_req', 'v_ground_est', 'v_ground_valid', 'nav_fix', 'beacon_cmd', 'buzzer_cmd'}, ...
    {'seq/1', 'NavSel/1', 'FsmSel/1', 'gnc_ready/1', 'FsmSel/2', 'FsmSel/3', 'v_cmd/1', 'omega_cmd/1', ...
     'accel_limit/1', 'soft_stop/1', 'v_ground/1', 'NavSel/5', 'NavSel/6', 'beacon/1', 'buzzer/1'});
L(n, 'Cmd/1', 'gnc_cmd/1', 'gnc_cmd');
local_save(n, f, sprintf(['gnc_CommandShaping (T14, gnc spec 2.2 / 7, D43, D53). omega: rate limit +/- cmd.w_rate, then ' ...
    'Saturation Dynamic +/- min(omega_max(|v|) [cmd.w_max_v/w_max], omega_max(max(|pitch|,|roll|)) [cmd.w_tilt_x/w_tilt]); ' ...
    'v: saturation [cmd.v_min_reverse, spd.v_max], then DescentEnv (gnc_descentEnvelope): v beyond the downhill cap ' ...
    'P.gnc.desc at the nav pitch (forward cap(pitch), reverse cap(-pitch), by plan.implement_type; the tiller may not ' ...
    'reverse) scaled to it, omega with it; track-speed envelope: v and omega scaled by ' ...
    'k = min(1, v_track_max/(|v| + |omega| chi B/2)) (chi, B, v_track_max from P.electronics via P.gnc.cmd); zero ' ...
    'command unless fsm.cmd_valid. Bus_GncCmd = impl (ImplementManager) with assigned seq (Counter Free-Running ' ...
    'uint8), t_stamp = nav.t, cmd_valid, gnc_ready = nav_ok AND plan.valid AND fsm.fault_word == 0, mode_req, ' ...
    'gnc_state, v_cmd, omega_cmd, accel_limit, soft_stop_req (mode_req HOLD), v_ground_est = nav_state.vel_B(1), ' ...
    'v_ground_valid = nav_ok, nav_fix = NavQuality, beacon_cmd (P.gnc.cmd.beacon_by_mode), buzzer (FAULT_STOP). ' ...
    'w_applied = shaped omega (MPC ext.mv through a Unit Delay in Shaddad_GNC).']));
end

% =============================================================================================
%  gnc_StatusMetrics (stub)
% =============================================================================================
function local_gnc_StatusMetrics(n, f, T)
local_new_ssr(n, f, T.(n));
BS(n, 'FsmSel', {'state', 'sub', 'seg_idx', 'fault_word', 'hold_reason', 'pause_reason', 't_hold', 'trk_sel', 'mission_done'});
L(n, 'fsm/1', 'FsmSel/1');
BS(n, 'PeSel', {'xte', 'epsi', 'v_ref'});            L(n, 'pe/1', 'PeSel/1');
BS(n, 'NavSel', {'nav_ok', 'pose'});                 L(n, 'nav/1', 'NavSel/1');
BS(n, 'PlanSel', {'mission_type', 'mission_id', 'n_seg'}); L(n, 'plan/1', 'PlanSel/1');
BS(n, 'RpSel', {'ver'});                             L(n, 'rp/1', 'RpSel/1');
DTC(n, 'xte_s', 'single', 'PeSel/1');  DTC(n, 'epsi_s', 'single', 'PeSel/2');  DTC(n, 'vref_s', 'single', 'PeSel/3');
DTC(n, 'soc_s', 'single', 'soc_rth/1'); DTC(n, 'thold_s', 'single', 'FsmSel/7');
C(n, 'StatusIC', 'P.ic.Bus_GncStatus', 'Bus: Bus_GncStatus');
BA(n, 'Status', 'StatusIC/1', {'gnc_state', 'sub_mode', 'mission_type', 'mission_id', 'seg_idx', 'n_seg', 'xte', ...
    'heading_err', 'v_ref', 'soc_rth', 'fault_word', 'hold_reason', 'pause_reason', 'hold_timer', 'replan_count', ...
    'tracker_active', 'nav_ok', 'pose_est', 'mission_done'}, ...
    {'FsmSel/1', 'FsmSel/2', 'PlanSel/1', 'PlanSel/2', 'FsmSel/3', 'PlanSel/3', 'xte_s/1', 'epsi_s/1', 'vref_s/1', ...
     'soc_s/1', 'FsmSel/4', 'FsmSel/5', 'FsmSel/6', 'thold_s/1', 'RpSel/1', 'FsmSel/8', 'NavSel/1', 'NavSel/2', 'FsmSel/9'});
RT(n, 'RT_status', 'Ts.gnc_status', 'Status/1');
L(n, 'RT_status/1', 'gnc_status/1', 'gnc_status');
L(n, 'dbg/1', 'gnc_debug/1', 'gnc_debug');
local_save(n, f, sprintf(['SHADDAD_GNC_STUB gnc_StatusMetrics (T14 stub with the final ports; T15 adds the 10 s moving RMS ' ...
    '(dspstat3/Moving RMS on ROW), progress, area, dist_home, eta). Stub: Bus_GncStatus fields copied from fsm / pe / nav / ' ...
    'plan / soc_rth / rp.ver (metrics 0), Rate Transition 0.05 -> 0.2 s; gnc_debug = dbg (assembled in Shaddad_GNC).']));
end

% =============================================================================================
%  Shaddad_GNC (Model Reference)
% =============================================================================================
function local_Shaddad_GNC(n, f, ~)
shaddad_bu('fresh', n, f, 'model');
shaddad_configsets('attach', n);
R = build_stubs('table');
R = R(strcmp({R.name}, 'Shaddad_GNC'));
for k = 1:numel(R.in)
    p = R.in(k);
    A(n, 'simulink/Sources/In1', p.name, 'Port', num2str(k), 'OutDataTypeStr', p.type, ...
        'SampleTime', shaddad_bu('tsexpr', p.ts), 'BusOutputAsStruct', 'on');
end
for k = 1:numel(R.out)
    p = R.out(k);
    A(n, 'simulink/Sinks/Out1', p.name, 'Port', num2str(k), 'OutDataTypeStr', p.type, ...
        'SampleTime', shaddad_bu('tsexpr', p.ts), 'BusOutputAsStruct', 'on');
end
ssr = {'InputConditioning', 'MissionManager', 'PathServer', 'PathAnalyzer', 'LocalAvoidance', 'GlobalReplanner', ...
       'SpeedGovernor', 'PathTracker', 'ImplementManager', 'CommandShaping', 'StatusMetrics'};
for k = 1:numel(ssr)
    A(n, 'simulink/Ports & Subsystems/Subsystem Reference', ssr{k}, 'ReferencedSubsystem', ['gnc_' ssr{k}]);
end
C(n, 'MissionSource', 'P.gnc.mission', 'Bus: Bus_MissionPlan', 'inf');
% ---- InputConditioning: nav_state mcu_status bms_status mcmd spray pe (delayed inside, T7 lane frame) ----
L(n, 'nav_state/1', 'InputConditioning/1'); L(n, 'mcu_status/1', 'InputConditioning/2');
L(n, 'bms_status/1', 'InputConditioning/3'); L(n, 'mcmd/1', 'InputConditioning/4'); L(n, 'spray/1', 'InputConditioning/5');
L(n, 'PathAnalyzer/1', 'InputConditioning/6', 'pe');
% ---- MissionManager: nav mcu bms mcmd obs pe plan rp teb_info ----
L(n, 'InputConditioning/1', 'MissionManager/1', 'nav'); L(n, 'InputConditioning/2', 'MissionManager/2', 'mcu');
L(n, 'InputConditioning/3', 'MissionManager/3', 'bms'); L(n, 'InputConditioning/4', 'MissionManager/4', 'mcmd_c');
L(n, 'LocalAvoidance/1', 'MissionManager/5', 'obs'); L(n, 'PathAnalyzer/1', 'MissionManager/6', 'pe');
L(n, 'MissionSource/1', 'MissionManager/7', 'plan'); L(n, 'GlobalReplanner/1', 'MissionManager/8', 'rp');
L(n, 'LocalAvoidance/3', 'MissionManager/9', 'teb_info');
% ---- PathServer: fsm plan rp ----
L(n, 'MissionManager/1', 'PathServer/1', 'fsm'); L(n, 'MissionSource/1', 'PathServer/2');
L(n, 'GlobalReplanner/1', 'PathServer/3');
% ---- PathAnalyzer: nav win ----
L(n, 'InputConditioning/1', 'PathAnalyzer/1'); L(n, 'PathServer/1', 'PathAnalyzer/2', 'win');
% ---- LocalAvoidance: perception nav fsm win pe tdir(z^-1) ----
A(n, 'simulink/Discrete/Unit Delay', 'tdir_z', 'InitialCondition', '0', 'SampleTime', 'Ts.gnc');
L(n, 'perception/1', 'LocalAvoidance/1'); L(n, 'InputConditioning/1', 'LocalAvoidance/2');
L(n, 'MissionManager/1', 'LocalAvoidance/3'); L(n, 'PathServer/1', 'LocalAvoidance/4');
L(n, 'PathAnalyzer/1', 'LocalAvoidance/5'); L(n, 'tdir_z/1', 'LocalAvoidance/6', 'tdir_z');
% ---- GlobalReplanner: fsm nav perception plan ----
L(n, 'MissionManager/1', 'GlobalReplanner/1'); L(n, 'InputConditioning/1', 'GlobalReplanner/2');
L(n, 'perception/1', 'GlobalReplanner/3'); L(n, 'MissionSource/1', 'GlobalReplanner/4');
% ---- SpeedGovernor: pe nav obs fsm bms mcmd spray win plan ----
L(n, 'PathAnalyzer/1', 'SpeedGovernor/1'); L(n, 'InputConditioning/1', 'SpeedGovernor/2');
L(n, 'LocalAvoidance/1', 'SpeedGovernor/3'); L(n, 'MissionManager/1', 'SpeedGovernor/4');
L(n, 'InputConditioning/3', 'SpeedGovernor/5'); L(n, 'InputConditioning/4', 'SpeedGovernor/6');
L(n, 'InputConditioning/5', 'SpeedGovernor/7', 'spray_c'); L(n, 'PathServer/1', 'SpeedGovernor/8');
L(n, 'MissionSource/1', 'SpeedGovernor/9');
% ---- PathTracker: nav win pe fsm v_gov teb_cmd w_applied(z^-1) ----
A(n, 'simulink/Discrete/Unit Delay', 'w_applied_z', 'InitialCondition', '0', 'SampleTime', 'Ts.gnc');
L(n, 'InputConditioning/1', 'PathTracker/1'); L(n, 'PathServer/1', 'PathTracker/2');
L(n, 'PathAnalyzer/1', 'PathTracker/3'); L(n, 'MissionManager/1', 'PathTracker/4');
L(n, 'SpeedGovernor/1', 'PathTracker/5', 'v_gov'); L(n, 'LocalAvoidance/2', 'PathTracker/6', 'teb_cmd');
L(n, 'w_applied_z/1', 'PathTracker/7', 'w_applied_z');
L(n, 'PathTracker/3', 'tdir_z/1', 'tdir');
% ---- ImplementManager: fsm pe nav mcu spray plan obs ----
L(n, 'MissionManager/1', 'ImplementManager/1'); L(n, 'PathAnalyzer/1', 'ImplementManager/2');
L(n, 'InputConditioning/1', 'ImplementManager/3'); L(n, 'InputConditioning/2', 'ImplementManager/4');
L(n, 'InputConditioning/5', 'ImplementManager/5'); L(n, 'MissionSource/1', 'ImplementManager/6');
L(n, 'LocalAvoidance/1', 'ImplementManager/7');
% ---- CommandShaping: v_trk w_trk fsm nav impl plan ----
L(n, 'PathTracker/1', 'CommandShaping/1', 'v_trk'); L(n, 'PathTracker/2', 'CommandShaping/2', 'w_trk');
L(n, 'MissionManager/1', 'CommandShaping/3'); L(n, 'InputConditioning/1', 'CommandShaping/4');
L(n, 'ImplementManager/1', 'CommandShaping/5', 'impl'); L(n, 'MissionSource/1', 'CommandShaping/6');
L(n, 'CommandShaping/2', 'w_applied_z/1', 'w_applied');
% ---- debug bus (Bus_GncDebug); one Bus Selector per consumer, right in front of it (layout spec 6.2) ----
A(n, 'simulink/Signal Routing/Vector Concatenate', 'pose_ref', 'NumInputs', '2', 'Mode', 'Vector');
PICK(n, 'PathAnalyzer/1', 'PeDbg_pose', {'xy_ref', 'psi_ref'}, {'pose_ref/1', 'pose_ref/2'});
PICK(n, 'PathAnalyzer/1', 'PeDbg_kappa', {'kappa_ref'});
PICK(n, 'LocalAvoidance/1', 'ObsDbg', {'d_obs', 'vfh_steer'});
PICK(n, 'PathServer/1', 'WinDbg', {'ver'});
SEL(n, 'teb_w', 'Index vector (dialog)', {'2'}, 'LocalAvoidance/2');
dbgS = {'kappa_s', 'PeDbg_kappa/1'; 'vprof_s', 'SpeedGovernor/4'; 'vgov_s', 'SpeedGovernor/1'; 'wpp_s', 'PathTracker/4'; ...
        'wrow_s', 'PathTracker/5'; 'wteb_s', 'teb_w/1'; 'dobs_s', 'ObsDbg/1'; 'vfh_s', 'ObsDbg/2'};
for k = 1:size(dbgS, 1), DTC(n, dbgS{k, 1}, 'single', dbgS{k, 2}); end
NM(n, 'SpeedGovernor/4', 'v_prof'); NM(n, 'PathTracker/4', 'w_pp'); NM(n, 'PathTracker/5', 'w_row');
BC(n, 'DebugBus', 'Bus_GncDebug', {'pose_ref', 'kappa_ref', 'v_prof', 'v_gov', 'v_limits', 'omega_pp', 'omega_row', ...
    'omega_teb', 'd_obs', 'vfh_steer', 'mpc_qp_status', 'window_ver'}, ...
    {'pose_ref/1', 'kappa_s/1', 'vprof_s/1', 'vgov_s/1', 'SpeedGovernor/2', 'wpp_s/1', 'wrow_s/1', 'wteb_s/1', ...
     'dobs_s/1', 'vfh_s/1', 'PathTracker/6', 'WinDbg/1'});
TERM(n, 'SpeedGovernor/3', 'T_dstop');
NM(n, 'SpeedGovernor/3', 'd_stop_req');
% ---- StatusMetrics: fsm pe nav plan rp soc_rth dbg ----
L(n, 'MissionManager/1', 'StatusMetrics/1'); L(n, 'PathAnalyzer/1', 'StatusMetrics/2');
L(n, 'InputConditioning/1', 'StatusMetrics/3'); L(n, 'MissionSource/1', 'StatusMetrics/4');
L(n, 'GlobalReplanner/1', 'StatusMetrics/5'); L(n, 'MissionManager/3', 'StatusMetrics/6', 'soc_rth');
L(n, 'DebugBus/1', 'StatusMetrics/7', 'dbg');
% ---- root outputs ----
L(n, 'CommandShaping/1', 'gnc_cmd/1', 'gnc_cmd'); L(n, 'MissionManager/2', 'nav_cmd/1', 'nav_cmd');
L(n, 'StatusMetrics/1', 'gnc_status/1', 'gnc_status'); L(n, 'StatusMetrics/2', 'gnc_debug/1', 'gnc_debug');
set_param(n, 'Description', sprintf(['Shaddad_GNC (T14, docs/specs/gnc.md, build plan T14). Root ports ICD 3.1 GNC: in nav_state ' ...
    'Bus_NavState 0.01, perception Bus_Perception 0.1, mcu_status Bus_McuStatus 0.05, bms_status Bus_BmsStatus 0.1, ' ...
    'mcmd Bus_MissionCmd 0.2, spray Bus_SprayDemand 0.1; out gnc_cmd Bus_GncCmd 0.05, nav_cmd Bus_NavCmd 0.05, ' ...
    'gnc_status Bus_GncStatus 0.2, gnc_debug Bus_GncDebug 0.05. No plan port: Constant MissionSource = P.gnc.mission ' ...
    '(Bus_MissionPlan, D57). Base rate 0.05 s; every subsystem is a Subsystem Reference (subsystems/gnc). Loop ' ...
    'breakers: Unit Delays on the Pure Pursuit target direction (-> LocalAvoidance) and on the applied omega ' ...
    '(-> PathTracker MPC ext.mv); MissionManager delays pe / rp / teb_info and InputConditioning pe (lane heading of the ' ...
    'T7 sigma term) internally. Config Shaddad_Ref (cs_Shaddad); ' ...
    'codegen with cs_Companion (BLOCKED while Simulink Coder is missing, D31).']));
shaddad_bu('save', n, f);
close_system(n, 0);
end
