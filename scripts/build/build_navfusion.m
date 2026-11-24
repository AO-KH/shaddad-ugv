function build_navfusion(which)
%BUILD_NAVFUSION  Build models/Shaddad_NavFusion.slx and subsystems/sensors/sens_acq_gnss.slx (build task T12).
%
%   build_navfusion()              build both files (sens_acq_gnss first)
%   build_navfusion(names)         build only 'sens_acq_gnss' and/or 'Shaddad_NavFusion'
%
%   Shaddad_NavFusion (Model Reference, companion navigation, docs/specs/sensors.md 3.5/3.6/4.3/5, D49/D63):
%     root ports ICD 3.1 NavFusion: in 1 imu Bus_ImuMeas 0.005, 2 odom Bus_OdomMeas 0.01, 3 gnss Bus_GnssRaw 0.1,
%     4 scanmatch Bus_ScanMatchMeas 0.2, 5 navcmd Bus_NavCmd 0.05, 6 bms Bus_BmsStatus 0.1;
%     out 1 nav_state Bus_NavState 0.01, 2 nav_diag Bus_NavDiag 0.01. Base rate 0.005 s, EKF 0.01 s.
%     IO_In          Variant Subsystem VC_TARGET for GNSS: 0 SIM pass-through, 1 HW = the sens_acq_gnss content inline
%                    (ros2lib/Subscribe of the ublox driver topics; ROS 2 message buses do not propagate inside a
%                    Subsystem Reference in R2026b, so the choice is a plain subsystem from the same builder)
%     NavPreproc     IMU hold on invalid samples, Discrete FIR [0.5 0.5] at 0.005 s, Rate Transition to 0.01 s,
%                    odometry, |I_pack| < 5 A gate, standstill, navMode / navReset (Detect Rise Positive)
%     AttitudeFilter centripetal compensation (v_hat[k-1] * omega_z), Gain -1 (physical -> toolbox accelerometer
%                    convention), mspfiltlib/IMU Filter (roll/pitch), mspfiltlib/ecompass (initial heading,
%                    minus declination), navlib Coordinate Transformation Conversion, level-frame yaw rate
%     NavMeas        GNSS: aerolibtransform2/LLA to Flat Earth -> ENU [y x -z], velocity, heading
%                    psi = wrap(pi/2 - heading - psiBaseline), accuracies; scan match; all slow measurements
%                    rate-transitioned to 0.01 s with new-data flags (counter at the slow rate -> RT ->
%                    Detect Change); odometry update every 5th sample (Pulse Generator, D13); heading source
%                    Multiport Switch (dual antenna / ecompass); extra inputs u1..u4 of the measurement functions
%     EkfInputs      state-transition input u (16x1) and process noise Q; D49 slip hold: while GNSS and scan
%                    matching are both unavailable for > t_denied, u carries tau_s_denied and the slip estimate
%                    held at denial onset (Switch + Unit Delay sample-and-hold) and Q = Qd_denied
%     NavGate        MATLAB Function sens_nav_gate (chi-square gating, time-varying R, forced acceptance)
%     NavModeManager MATLAB Function sens_nav_mode (INIT / ALIGN / RUN_GNSS / RUN_LIDAR / DEAD_RECKONING / FAULT)
%     NavEKF         Resettable Subsystem (reset = navReset rising edge or FAULT) holding the MATLAB System
%                    sens_nav_ekf: extendedKalmanFilter with predict + five gated corrections at 0.01 s
%                    (EnableMultirate-off structure of D63). R6 fallback: the T12 first-step prototype showed
%                    that the cstblocks_estimation/Extended Kalman Filter block keeps x/P in Data Store Memory
%                    that the Resettable Subsystem does not re-initialise (T-NAV-09 failed with the block).
%     NavOutput      aerolibtransform2/Flat Earth to LLA, Euler -> quaternion, Bus_NavState / Bus_NavDiag
%     Loop breakers: Unit Delays on xhat, P, u, Q (gate prediction), the reset pulse (gate counters), the heading
%     source and the channel mask (mode manager -> measurements).
%   sens_acq_gnss (Subsystem Reference, HW choice of IO_In, 0.1 s): ros2lib/Subscribe /fix
%     (sensor_msgs/NavSatFix), /fix_velocity (geometry_msgs/TwistWithCovarianceStamped), /navheading
%     (sensor_msgs/Imu) -> Bus_GnssRaw.
%   Needs P, Ts, VC_*, Bus_* in base (shaddad_params is run when missing). Every number comes from P / Ts.
%   Layout (model layout spec 6 and 7): no block is placed here; shaddad_bu('save') lays out every level of each file
%   with shaddad_layout(<file>, 'Recursive', true), the inner subsystems and the IO_In choices included. Every signal
%   line is named (L5, shaddad_bu('nline'); into a bus-typed Bus Creator a line without a name takes its element's
%   name). The root ports stay classic (ICD 3.1). The inner subsystems pass their signals on virtual buses (spec 6.2,
%   6.3; no bus objects): the Out Bus Element ports pre (NavPreproc), attitude (AttitudeFilter) and meas (NavMeas), and
%   the virtual Bus Creators Gate (NavGate's outputs) and Mode (NavModeManager's outputs to NavOutput). Each inner
%   subsystem picks what it uses with In Bus Element blocks right in front of each consumer, the two MATLAB Functions
%   with the Bus Selectors Sel_meas_gate, Sel_pre_mode, Sel_meas_mode and Sel_gate_mode. The acquisition content takes
%   one Bus Selector per consumer from each ROS 2 message.

names = {'sens_acq_gnss', 'Shaddad_NavFusion'};
if nargin < 1 || isempty(which), which = names; end
if ischar(which) || isstring(which), which = cellstr(which); end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_NavState'',''var'')')
    shaddad_params();
end
root = char(currentProject().RootFolder);
% the bench and the files of this task are closed unsaved so they can be replaced
for m = {'tb_nav', 'Shaddad_NavFusion', 'sens_acq_gnss'}
    if bdIsLoaded(m{1})
        try
            close_system(m{1}, 0);
        catch %#ok<CTCH> referenced by a loaded model of another task
        end
    end
end
if any(strcmp(which, 'sens_acq_gnss'))
    f = fullfile(root, 'subsystems', 'sensors', 'sens_acq_gnss.slx');
    shaddad_bu('fresh', 'sens_acq_gnss', f, 'ssr');
    local_sens_acq_gnss('sens_acq_gnss');
    shaddad_bu('save', 'sens_acq_gnss', f);
    close_system('sens_acq_gnss', 0);
end
if any(strcmp(which, 'Shaddad_NavFusion'))
    f = fullfile(root, 'models', 'Shaddad_NavFusion.slx');
    shaddad_bu('fresh', 'Shaddad_NavFusion', f, 'model');
    local_navfusion('Shaddad_NavFusion');
    shaddad_bu('save', 'Shaddad_NavFusion', f);
    close_system('Shaddad_NavFusion', 0);
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
function OB(sys, port, el, src, nm)
% the element el of the Out Bus Element port 'port' of sys, fed by src ('Blk/k'), its line named nm (default el)
% (spec 6.3): shaddad_bu('outbus')
if nargin < 5, nm = ''; end
shaddad_bu('outbus', sys, port, el, src, nm);
end
function OBP(sys, port, el, first, src)
% the element el of the Out Bus Element port 'port' of sys, fed straight by the pick src ('port.element'): its block
% comes first, copied from the port's block first (spec 13), then the pick right in front of it
h = add_block([sys '/' first], [sys '/' port '_' el]);
set_param(h, 'Element', el);
L(sys, src, [port '_' el '/1'], el);
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
% nonvirtual Bus Creator of type bus, srcs per element in bus order ('Blk/k' or a pick 'port.element'); a line without
% a name yet takes its element's name (L5)
b = evalin('base', bus);
A(sys, 'simulink/Signal Routing/Bus Creator', blk, 'Inputs', num2str(numel(srcs)), 'OutDataTypeStr', ['Bus: ' bus], ...
  'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:numel(srcs)
    nm = b.Elements(k).Name;
    if contains(srcs{k}, '/')
        ph = get_param([sys '/' extractBefore(srcs{k}, '/')], 'PortHandles');
        old = get_param(ph.Outport(str2double(extractAfter(srcs{k}, '/'))), 'Line');
        if old ~= -1 && ~isempty(get_param(old, 'Name')), nm = ''; end
    end
    L(sys, srcs{k}, sprintf('%s/%d', blk, k), nm);
end
end
function C(sys, blk, val, varargin)
A(sys, 'simulink/Sources/Constant', blk, 'Value', val, 'SampleTime', 'inf', varargin{:});
end
function RT(sys, blk, ts)
A(sys, 'simulink/Signal Attributes/Rate Transition', blk, 'Integrity', 'on', 'Deterministic', 'on', ...
  'OutPortSampleTime', ts);
end
function BS(sys, blk, sigs)
A(sys, 'simulink/Signal Routing/Bus Selector', blk, 'OutputSignals', strjoin(sigs, ','));
end
function SEL(sys, blk, idx, w)
A(sys, 'simulink/Signal Routing/Selector', blk, 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog)', 'IndexParamArray', {idx}, 'InputPortWidth', num2str(w));
end
function DTC(sys, blk, type)
A(sys, 'simulink/Signal Attributes/Data Type Conversion', blk, 'OutDataTypeStr', type, ...
  'SaturateOnIntegerOverflow', 'on');
end
function CAT(sys, blk, n)
A(sys, 'simulink/Signal Routing/Vector Concatenate', blk, 'NumInputs', num2str(n), 'Mode', 'Vector');
end
function G(sys, blk, k)
A(sys, 'simulink/Math Operations/Gain', blk, 'Gain', k, 'Multiplication', 'Element-wise(K.*u)');
end
function FCN(sys, blk, expr)
A(sys, 'simulink/User-Defined Functions/Fcn', blk, 'Expr', expr);
end
function TRIG(sys, blk, op)
A(sys, 'simulink/Math Operations/Trigonometric Function', blk, 'Operator', op);
end
function UD(sys, blk, ic, ts)
A(sys, 'simulink/Discrete/Unit Delay', blk, 'InitialCondition', ic, 'SampleTime', ts);
end
function LOG(sys, blk, op, n)
A(sys, 'simulink/Logic and Bit Operations/Logical Operator', blk, 'Operator', op, 'Inputs', num2str(n));
end
function CMP(sys, blk, op, c)
A(sys, 'simulink/Logic and Bit Operations/Compare To Constant', blk, 'relop', op, 'const', c, ...
  'OutDataTypeStr', 'boolean');
end
function SW(sys, blk)
A(sys, 'simulink/Signal Routing/Switch', blk, 'Criteria', 'u2 ~= 0');
end
function WRAP(sys, blk, src)
% wrap an angle to [-pi, pi]: atan2(sin(a), cos(a))
TRIG(sys, [blk '_s'], 'sin'); TRIG(sys, [blk '_c'], 'cos'); TRIG(sys, blk, 'atan2');
L(sys, src, [blk '_s/1']); L(sys, src, [blk '_c/1']);
L(sys, [blk '_s/1'], [blk '/1']); L(sys, [blk '_c/1'], [blk '/2']);
end
function s = SUB(parent, blk, ins, outs)
% plain (virtual) subsystem with the listed inports / outports
s = A(parent, 'simulink/Ports & Subsystems/Subsystem', blk);
lns = find_system(s, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(s, 'SearchDepth', 1), {s});
for k = 1:numel(b), delete_block(b{k}); end
for k = 1:numel(ins), add_block('simulink/Sources/In1', [s '/' ins{k}], 'Port', num2str(k)); end
for k = 1:numel(outs), add_block('simulink/Sinks/Out1', [s '/' outs{k}], 'Port', num2str(k)); end
end
function mf = MFB(sys, name, script, params, ts, desc)
% MATLAB Function block: body = one-line call of a %#codegen file; params = {name, expr} non-tunable
% sub-structs of P.sensors (build rule 2) set through a mask
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
mk = Simulink.Mask.create(mf);
for k = 1:size(params, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k, 1});
    d.Scope = 'Parameter';
    d.Tunable = false;
    mk.addParameter('Type', 'edit', 'Name', params{k, 1}, 'Prompt', params{k, 1}, 'Value', params{k, 2}, ...
        'Evaluate', 'on', 'Tunable', 'off');
end
end
function clearsub(s)
lns = find_system(s, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants), {s});
for k = 1:numel(b), delete_block(b{k}); end
end

% ============================================================================================
%  Shaddad_NavFusion
% ============================================================================================
function local_navfusion(n)
shaddad_configsets('attach', n);
R = build_stubs('table');
R = R(strcmp({R.name}, 'Shaddad_NavFusion'));
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
ts = 'Ts.nav';

local_io_in(n);
local_preproc(n);
local_attitude(n);
local_meas(n);
local_ekfinputs(n);

% ---- loop-breaking Unit Delays (0.01 s) ----
UD(n, 'xhat_z', 'P.sensors.ekf.x0', ts);
UD(n, 'P_z', 'P.sensors.ekf.P0', ts);
UD(n, 'u_z', 'P.sensors.ekf.u0', ts);
UD(n, 'Q_z', 'P.sensors.ekf.Qd', ts);
UD(n, 'rst_z', 'false', ts);
UD(n, 'headSel_z', '0', ts);
UD(n, 'chMask_z', 'true(5, 1)', ts);
A(n, 'simulink/Math Operations/Reshape', 'P_vec', 'OutputDimensionality', '1-D array');
SEL(n, 'Pd', '[1 13 37]', 121);
SEL(n, 'v_prev', '5', 11);
L(n, 'P_z/1', 'P_vec/1', 'P_prev');  L(n, 'P_vec/1', 'Pd/1');  L(n, 'xhat_z/1', 'v_prev/1', 'x_prev');

% ---- gate ----
MFB(n, 'NavGate', ['function [en, R1, R2, R3, R4, R5, nis, rej] = fcn(xP, PP, uP, QP, rst, z1, z2, z3, z4, z5, ' ...
    'avail, accG, s3, C5, u1, u2, u3, u4, u5, fix, prm)' newline ...
    '[en, R1, R2, R3, R4, R5, nis, rej] = sens_nav_gate(xP, PP, uP, QP, rst, z1, z2, z3, z4, z5, avail, accG, s3, ' ...
    'C5, u1, u2, u3, u4, u5, fix, prm);' newline], {'prm', 'P.sensors.navgate'}, ts, ...
    'Innovation gating and time-varying R of the navigation EKF (subsystems/sensors/sens_nav_gate.m).');
% ---- mode manager ----
A(n, 'simulink/Sources/Digital Clock', 'Clock', 'SampleTime', ts);
MFB(n, 'NavModeManager', ['function [modeState, quality, navValid, aligned, headSel, lidarLocOn, rstOut, tSinceAbs, ' ...
    'denied, resetCount, headingValid, chMask] = fcn(t, navMode, rstExt, imuValid, still, iLow, headOkG, fix, avail, ' ...
    'en, nis, Pd, prm)' newline ...
    '[modeState, quality, navValid, aligned, headSel, lidarLocOn, rstOut, tSinceAbs, denied, resetCount, headingValid, ' ...
    'chMask] = sens_nav_mode(t, navMode, rstExt, imuValid, still, iLow, headOkG, fix, avail, en, nis, Pd, prm);' newline], ...
    {'prm', 'P.sensors.navmode'}, ts, 'Navigation mode manager FSM (subsystems/sensors/sens_nav_mode.m).');

local_ekf(n);
local_output(n);

% ---- the inner virtual buses (spec 6.3): NavGate's outputs (Gate) and NavModeManager's outputs to NavOutput (Mode) ----
shaddad_bu('vbus', n, 'Gate', {'en', 'NavGate/1'; 'R1', 'NavGate/2'; 'R2', 'NavGate/3'; 'R3', 'NavGate/4'; ...
    'R4', 'NavGate/5'; 'R5', 'NavGate/6'; 'nis', 'NavGate/7'; 'rej', 'NavGate/8'});
shaddad_bu('vbus', n, 'Mode', {'modeState', 'NavModeManager/1'; 'quality', 'NavModeManager/2'; ...
    'navValid', 'NavModeManager/3'; 'aligned', 'NavModeManager/4'; 'lidarLocOn', 'NavModeManager/6'; ...
    'tSinceAbs', 'NavModeManager/8'; 'resetCount', 'NavModeManager/10'; 'headingValid', 'NavModeManager/11'});
% ---- wiring: IO_In, preprocessing, attitude ----
L(n, 'gnss/1', 'IO_In/1', 'gnss');
L(n, 'imu/1', 'NavPreproc/1', 'imu');  L(n, 'odom/1', 'NavPreproc/2', 'odom');
L(n, 'bms/1', 'NavPreproc/3', 'bms');  L(n, 'navcmd/1', 'NavPreproc/4', 'navcmd');
L(n, 'NavPreproc/1', 'AttitudeFilter/1', 'pre');  L(n, 'v_prev/1', 'AttitudeFilter/2', 'v_prev');
% ---- NavMeas: gnss scanmatch pre attitude headSel_z chMask_z ----
L(n, 'IO_In/1', 'NavMeas/1', 'gnss_c');  L(n, 'scanmatch/1', 'NavMeas/2', 'scanmatch');
L(n, 'NavPreproc/1', 'NavMeas/3');  L(n, 'AttitudeFilter/1', 'NavMeas/4', 'attitude');
L(n, 'headSel_z/1', 'NavMeas/5', 'headSel_z');  L(n, 'chMask_z/1', 'NavMeas/6', 'chMask_z');
% ---- EkfInputs: pre attitude denied x_prev ----
L(n, 'NavPreproc/1', 'EkfInputs/1');  L(n, 'AttitudeFilter/1', 'EkfInputs/2');
L(n, 'NavModeManager/9', 'EkfInputs/3', 'denied');  L(n, 'xhat_z/1', 'EkfInputs/4');
L(n, 'EkfInputs/1', 'u_z/1', 'u');  L(n, 'EkfInputs/2', 'Q_z/1', 'Q');
% ---- NavGate inputs: xP PP uP QP rst from the delays, then z1 ... fix from meas ----
src = {'xhat_z/1', 'P_z/1', 'u_z/1', 'Q_z/1', 'rst_z/1'};
nm = {'', '', 'u_prev', 'Q_prev', 'rst_prev'};
for k = 1:numel(src), L(n, src{k}, sprintf('NavGate/%d', k), nm{k}); end
el = {'z1', 'z2', 'z3', 'z4', 'z5', 'avail', 'accG', 's3', 'C5', 'u1', 'u2', 'u3', 'u4', 'u5', 'fix'};
PICK(n, 'NavMeas/1', 'Sel_meas_gate', el, arrayfun(@(k) sprintf('NavGate/%d', k + 5), 1:numel(el), ...
    'UniformOutput', false), 'meas');
% ---- NavModeManager inputs: t, navMode ... iLow from pre, headOkG fix avail from meas, en nis from gate, Pd ----
L(n, 'Clock/1', 'NavModeManager/1', 't');
PICK(n, 'NavPreproc/1', 'Sel_pre_mode', {'navMode', 'rstExt', 'imuValid', 'still', 'iLow'}, ...
    {'NavModeManager/2', 'NavModeManager/3', 'NavModeManager/4', 'NavModeManager/5', 'NavModeManager/6'});
PICK(n, 'NavMeas/1', 'Sel_meas_mode', {'headOkG', 'fix', 'avail'}, ...
    {'NavModeManager/7', 'NavModeManager/8', 'NavModeManager/9'});
PICK(n, 'Gate/1', 'Sel_gate_mode', {'en', 'nis'}, {'NavModeManager/10', 'NavModeManager/11'}, 'gate');
L(n, 'Pd/1', 'NavModeManager/12', 'Pd');
L(n, 'NavModeManager/5', 'headSel_z/1', 'headSel');  L(n, 'NavModeManager/12', 'chMask_z/1', 'chMask');
L(n, 'NavModeManager/7', 'rst_z/1', 'rstOut');
% ---- NavEKF: u Q meas gate hold (+ reset) ----
L(n, 'EkfInputs/1', 'NavEKF/1');  L(n, 'EkfInputs/2', 'NavEKF/2');  L(n, 'NavMeas/1', 'NavEKF/3');
L(n, 'Gate/1', 'NavEKF/4');  L(n, 'NavModeManager/9', 'NavEKF/5');
ph = get_param([n '/NavEKF'], 'PortHandles');
pm = get_param([n '/NavModeManager'], 'PortHandles');
add_line(n, pm.Outport(7), ph.Reset(1), 'autorouting', 'smart');
L(n, 'NavEKF/1', 'xhat_z/1', 'xhat');  L(n, 'NavEKF/2', 'P_z/1', 'Pcov');
% ---- NavOutput: pre attitude meas gate mode xhat P ----
L(n, 'NavPreproc/1', 'NavOutput/1');  L(n, 'AttitudeFilter/1', 'NavOutput/2');  L(n, 'NavMeas/1', 'NavOutput/3');
L(n, 'Gate/1', 'NavOutput/4');  L(n, 'Mode/1', 'NavOutput/5', 'mode');
L(n, 'NavEKF/1', 'NavOutput/6');  L(n, 'NavEKF/2', 'NavOutput/7');
L(n, 'NavOutput/1', 'nav_state/1', 'nav_state');  L(n, 'NavOutput/2', 'nav_diag/1', 'nav_diag');

set_param(n, 'Description', sprintf(['Shaddad_NavFusion (T12, docs/specs/sensors.md 3.5/3.6/4.3/5, D49/D63; built by ' ...
    'scripts/build/build_navfusion.m). Root ports ICD 3.1 NavFusion: in 1 imu Bus_ImuMeas 0.005, 2 odom Bus_OdomMeas 0.01, ' ...
    '3 gnss Bus_GnssRaw 0.1, 4 scanmatch Bus_ScanMatchMeas 0.2, 5 navcmd Bus_NavCmd 0.05, 6 bms Bus_BmsStatus 0.1; out 1 ' ...
    'nav_state Bus_NavState 0.01, 2 nav_diag Bus_NavDiag 0.01. Base rate 0.005 s, EKF 0.01 s.\n' ...
    'IO_In Variant VC_TARGET (0 SIM pass-through, 1 HW = sens_acq_gnss content inline, ros2lib/Subscribe); NavPreproc (IMU hold, FIR ' ...
    '[0.5 0.5], RT 0.005 -> 0.01, odometry, |I_pack| < 5 A, standstill, navReset edge); AttitudeFilter (centripetal ' ...
    'compensation, Gain -1, mspfiltlib/IMU Filter, mspfiltlib/ecompass - declination); NavMeas (LLA to Flat Earth -> ' ...
    'ENU, slow-to-fast RT + new-data flags, odometry every 5th sample, heading Multiport Switch); EkfInputs (u 16x1, ' ...
    'time-varying Q, D49 slip hold); NavGate (MATLAB Function sens_nav_gate); NavModeManager (MATLAB Function ' ...
    'sens_nav_mode); NavEKF = Resettable Subsystem (navReset / FAULT) with MATLAB System sens_nav_ekf ' ...
    '(extendedKalmanFilter, predict + 5 gated corrections at 0.01 s; R6 fallback because the EKF block''s Data Store ' ...
    'Memory is not reset by a Resettable Subsystem); NavOutput (Flat Earth to LLA, Bus_NavState, Bus_NavDiag). ' ...
    'Config Shaddad_Ref (cs_Shaddad, multitasking, MultiTaskDSMMsg error); codegen with cs_Companion (BLOCKED while ' ...
    'Simulink Coder is missing, D31).']));
end

% --------------------------------------------------------------------------------------------
function local_io_in(n)
vs = A(n, 'simulink/Ports & Subsystems/Variant Subsystem', 'IO_In');
clearsub(vs);
add_block('simulink/Sources/In1', [vs '/gnss_in'], 'Port', '1');
add_block('simulink/Sinks/Out1', [vs '/gnss'], 'Port', '1');
s = A(vs, 'simulink/Ports & Subsystems/Subsystem', 'SIM', 'VariantControl', 'VC_TARGET == 0');
clearsub(s);
add_block('simulink/Sources/In1', [s '/gnss_in'], 'Port', '1');
add_block('simulink/Sinks/Out1', [s '/gnss'], 'Port', '1');
L(s, 'gnss_in/1', 'gnss/1', 'gnss');
% HW choice: the sens_acq_gnss content INLINE (plain Subsystem built by the same builder function as
% subsystems/sensors/sens_acq_gnss.slx, t_nav/tHwGnssAcquisition checks they are identical). R2026b: inside a
% Subsystem Reference the ros2lib/Subscribe message buses do not propagate (one of the three Bus Selectors fails
% with 'must receive a bus'), while the same content compiles as a plain subsystem (T12 finding).
h = A(vs, 'simulink/Ports & Subsystems/Subsystem', 'HW', 'VariantControl', 'VC_TARGET == 1');
clearsub(h);
local_sens_acq_gnss(h);
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

% --------------------------------------------------------------------------------------------
function local_preproc(n)
% in imu, odom, bms, navcmd (In Bus Element ports, the picks right in front of each consumer); out bus pre
s = SUB(n, 'NavPreproc', {}, {});
ti = 'Ts.imu'; ts = 'Ts.nav';
% ---- IMU: hold the last valid sample, 2-sample average (anti-alias 200 -> 100 Hz), RT to 0.01 s ----
CAT(s, 'Imu9s', 3);  L(s, 'imu.accel_B', 'Imu9s/1');  L(s, 'imu.gyro_B', 'Imu9s/2');
L(s, 'imu.mag_B', 'Imu9s/3');
DTC(s, 'Imu9d', 'double');  L(s, 'Imu9s/1', 'Imu9d/1');
SW(s, 'ImuHold');  UD(s, 'ImuHold_z', '[0; 0; P.sensors.g; 0; 0; 0; 0; 0; 0]', ti);
L(s, 'Imu9d/1', 'ImuHold/1', 'imu9_raw');  L(s, 'imu.valid', 'ImuHold/2');  L(s, 'ImuHold_z/1', 'ImuHold/3');
L(s, 'ImuHold/1', 'ImuHold_z/1', 'imu9_held');
A(s, 'simulink/Discrete/Discrete FIR Filter', 'ImuFIR', 'Coefficients', '[0.5 0.5]', 'SampleTime', ti, ...
  'InputProcessing', 'Elements as channels (sample based)', 'InitialStates', '0');
L(s, 'ImuHold/1', 'ImuFIR/1');
RT(s, 'RT_imu', ts);  L(s, 'ImuFIR/1', 'RT_imu/1', 'imu9_fir');  OB(s, 'pre', 'imu9', 'RT_imu/1');
RT(s, 'RT_valid', ts);  L(s, 'imu.valid', 'RT_valid/1');  OB(s, 'pre', 'imuValid', 'RT_valid/1');
% ---- odometry (0.01 s) ----
DTC(s, 'vTrackD', 'double');  L(s, 'odom.vTrack_LR', 'vTrackD/1');  OB(s, 'pre', 'vTrack', 'vTrackD/1');
OBP(s, 'pre', 'odomValid', 'pre_imu9', 'odom.valid');
% ---- BMS current gate for the ecompass: abs(I_pack) < ecompass_iMax ----
DTC(s, 'IpackD', 'double');  L(s, 'bms.I_pack', 'IpackD/1');
RT(s, 'RT_bms', ts);  L(s, 'IpackD/1', 'RT_bms/1');
A(s, 'simulink/Math Operations/Abs', 'Iabs');  L(s, 'RT_bms/1', 'Iabs/1');
CMP(s, 'ILow', '<', 'P.sensors.nav.ecompass_iMax');  L(s, 'Iabs/1', 'ILow/1');  OB(s, 'pre', 'iLow', 'ILow/1');
% ---- navigation command (0.05 -> 0.01 s), navReset rising edge ----
RT(s, 'RT_mode', ts);  RT(s, 'RT_reset', ts);
L(s, 'navcmd.navMode', 'RT_mode/1');  L(s, 'navcmd.navReset', 'RT_reset/1');
OB(s, 'pre', 'navMode', 'RT_mode/1');
A(s, 'simulink/Logic and Bit Operations/Detect Rise Positive', 'ResetEdge', 'vinit', '0');
L(s, 'RT_reset/1', 'ResetEdge/1', 'navReset');  OB(s, 'pre', 'rstExt', 'ResetEdge/1');
% ---- standstill: max |vTrack| < vStill and |gyro z| < wStill and odometry valid ----
A(s, 'simulink/Math Operations/Abs', 'VAbs');  L(s, 'vTrackD/1', 'VAbs/1');
A(s, 'simulink/Math Operations/MinMax', 'VMax', 'Function', 'max', 'Inputs', '1');  L(s, 'VAbs/1', 'VMax/1');
CMP(s, 'VStill', '<', 'P.sensors.nav.vStill');  L(s, 'VMax/1', 'VStill/1');
SEL(s, 'Wz', '6', 9);  L(s, 'RT_imu/1', 'Wz/1');
A(s, 'simulink/Math Operations/Abs', 'WAbs');  L(s, 'Wz/1', 'WAbs/1');
CMP(s, 'WStill', '<', 'P.sensors.nav.wStill');  L(s, 'WAbs/1', 'WStill/1');
LOG(s, 'Still', 'AND', 3);
L(s, 'VStill/1', 'Still/1');  L(s, 'WStill/1', 'Still/2');  L(s, 'odom.valid', 'Still/3');
OB(s, 'pre', 'still', 'Still/1');
shaddad_bu('portorder', s, {'imu', 'odom', 'bms', 'navcmd'}, 'Inport');
end

% --------------------------------------------------------------------------------------------
function local_attitude(n)
% in pre (In Bus Element: imu9, vTrack), v_prev; out bus attitude
s = SUB(n, 'AttitudeFilter', {'v_prev'}, {});
SEL(s, 'Acc', '1:3', 9);  SEL(s, 'Gyro', '4:6', 9);  SEL(s, 'Mag', '7:9', 9);  SEL(s, 'Wy', '5', 9);  SEL(s, 'Wz', '6', 9);
for b = {'Acc', 'Gyro', 'Mag', 'Wy', 'Wz'}, L(s, 'pre.imu9', [b{1} '/1']); end
% ---- kinematic acceleration a_kin = [a_track; v_hat[k-1]*omega_z; 0] (centripetal per spec 3.6; longitudinal
%      from the odometry track speed derivative, first order tau_acc, as the MCU tilt filter D47) ----
A(s, 'simulink/Math Operations/Product', 'aCent', 'Inputs', '2');
L(s, 'v_prev/1', 'aCent/1', 'v_prev');  L(s, 'Wz/1', 'aCent/2', 'wz');
A(s, 'simulink/Math Operations/Sum', 'VSum', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(s, 'pre.vTrack', 'VSum/1', 'vTrack');
A(s, 'simulink/Math Operations/Gain', 'VMean', 'Gain', '0.5');  L(s, 'VSum/1', 'VMean/1');
A(s, 'simulink/Discrete/Discrete Derivative', 'dVdt', 'gainval', '1', 'ICPrevScaledInput', '0');
L(s, 'VMean/1', 'dVdt/1', 'v_track');
A(s, 'simulink/Discrete/Discrete Transfer Fcn', 'ATrack', 'Numerator', '1 - exp(-Ts.nav/P.sensors.nav.tauAcc)', ...
  'Denominator', '[1 -exp(-Ts.nav/P.sensors.nav.tauAcc)]', 'SampleTime', 'Ts.nav');
L(s, 'dVdt/1', 'ATrack/1', 'dv_dt');
C(s, 'Zero', '0');
CAT(s, 'aKin', 3);  L(s, 'ATrack/1', 'aKin/1', 'a_track');  L(s, 'aCent/1', 'aKin/2', 'a_cent');  L(s, 'Zero/1', 'aKin/3');
A(s, 'simulink/Math Operations/Sum', 'AccComp', 'Inputs', '+-');
L(s, 'Acc/1', 'AccComp/1', 'acc_phys');  L(s, 'aKin/1', 'AccComp/2', 'a_kin');
% ---- Gain -1: physical specific force -> toolbox accelerometer convention (spec 2, R6) ----
A(s, 'simulink/Math Operations/Gain', 'NegAcc', 'Gain', '-1');  L(s, 'AccComp/1', 'NegAcc/1', 'acc_comp');
A(s, 'simulink/Math Operations/Gain', 'NegAccRaw', 'Gain', '-1');  L(s, 'Acc/1', 'NegAccRaw/1');
for b = {'AccRow', 'GyroRow', 'AccRawRow', 'MagRow'}
    A(s, 'simulink/Math Operations/Reshape', b{1}, 'OutputDimensionality', 'Customize', 'OutputDimensions', '[1 3]');
end
L(s, 'NegAcc/1', 'AccRow/1', 'acc_tb');  L(s, 'Gyro/1', 'GyroRow/1', 'gyro');
L(s, 'NegAccRaw/1', 'AccRawRow/1', 'acc_raw_tb');  L(s, 'Mag/1', 'MagRow/1', 'mag');
% ---- IMU Filter (roll / pitch) ----
im = 'P.sensors.imuf';
A(s, 'mspfiltlib/IMU Filter', 'IMUFilter', 'DispRefFrame', 'ENU', 'OrientationFormat', 'quaternion', ...
  'DecimationFactor', '1', 'AccelerometerNoise', [im '.AccelerometerNoise'], 'GyroscopeNoise', [im '.GyroscopeNoise'], ...
  'GyroscopeDriftNoise', [im '.GyroscopeDriftNoise'], 'LinearAccelerationNoise', [im '.LinearAccelerationNoise'], ...
  'LinearAccelerationDecayFactor', [im '.LinearAccelerationDecayFactor'], 'InitialProcessNoise', 'P.sensors.nav.imufQ0', ...
  'SimulateUsing', 'Code generation');
L(s, 'AccRow/1', 'IMUFilter/1');  L(s, 'GyroRow/1', 'IMUFilter/2');
A(s, 'simulink/Sinks/Terminator', 'T_angvel');  L(s, 'IMUFilter/2', 'T_angvel/1');
A(s, 'navlib/Utilities/Coordinate Transformation Conversion', 'Q2Eul', 'InputRepresentation', 'Quaternion', ...
  'OutputRepresentation', 'Euler Angles', 'OutputEulerSequence', 'ZYX');
A(s, 'simulink/Math Operations/Reshape', 'QCol', 'OutputDimensionality', 'Column vector (2-D)');
L(s, 'IMUFilter/1', 'QCol/1', 'q_imuf');  L(s, 'QCol/1', 'Q2Eul/1');
A(s, 'simulink/Math Operations/Reshape', 'EulVec', 'OutputDimensionality', '1-D array');  L(s, 'Q2Eul/1', 'EulVec/1');
SEL(s, 'PhiTheta', '[3 2]', 3);  L(s, 'EulVec/1', 'PhiTheta/1', 'eul_imuf');
OB(s, 'attitude', 'att', 'PhiTheta/1');
% ---- ecompass (initial heading, true north = magnetic - declination) ----
A(s, 'mspfiltlib/ecompass', 'ECompass', 'ReferenceFrame', 'ENU', 'OrientationFormat', 'quaternion', ...
  'SimulateUsing', 'Code generation');
L(s, 'AccRawRow/1', 'ECompass/1');  L(s, 'MagRow/1', 'ECompass/2');
A(s, 'navlib/Utilities/Coordinate Transformation Conversion', 'Q2EulEc', 'InputRepresentation', 'Quaternion', ...
  'OutputRepresentation', 'Euler Angles', 'OutputEulerSequence', 'ZYX');
A(s, 'simulink/Math Operations/Reshape', 'QEcCol', 'OutputDimensionality', 'Column vector (2-D)');
L(s, 'ECompass/1', 'QEcCol/1', 'q_ec');  L(s, 'QEcCol/1', 'Q2EulEc/1');
A(s, 'simulink/Math Operations/Reshape', 'EulEcVec', 'OutputDimensionality', '1-D array');  L(s, 'Q2EulEc/1', 'EulEcVec/1');
SEL(s, 'YawMag', '1', 3);  L(s, 'EulEcVec/1', 'YawMag/1');
A(s, 'simulink/Math Operations/Bias', 'MinusDecl', 'Bias', '-P.sensors.magDecl*pi/180');
L(s, 'YawMag/1', 'MinusDecl/1', 'psi_mag');
WRAP(s, 'PsiEc', 'MinusDecl/1');  OB(s, 'attitude', 'psi_ec', 'PsiEc/1');
% ---- level-frame yaw rate psidot_m = (wy sin(phi) + wz cos(phi)) / cos(theta) ----
CAT(s, 'YrIn', 3);  L(s, 'Wy/1', 'YrIn/1', 'wy');  L(s, 'Wz/1', 'YrIn/2');  L(s, 'PhiTheta/1', 'YrIn/3');
FCN(s, 'PsiDot', '(u(1)*sin(u(3)) + u(2)*cos(u(3)))/cos(u(4))');  L(s, 'YrIn/1', 'PsiDot/1');
OB(s, 'attitude', 'psidot_m', 'PsiDot/1');  OB(s, 'attitude', 'wz', 'Wz/1');
shaddad_bu('portorder', s, {'pre', 'v_prev'}, 'Inport');
end

% --------------------------------------------------------------------------------------------
function local_meas(n)
% in gnss, scanmatch, pre, attitude (In Bus Element ports, the picks right in front of each consumer), headSel_z,
% chMask_z; out bus meas (z1 ... z5, avail, accG, s3, C5, u1 ... u5, fix, headOkG, tau)
s = SUB(n, 'NavMeas', {'headSel_z', 'chMask_z'}, {});
tg = 'Ts.gnss'; tsm = 'Ts.scanmatch'; ts = 'Ts.nav';
% ================= GNSS (0.1 s side) =================
SEL(s, 'LatLon', '[1 2]', 3);  SEL(s, 'Alt', '3', 3);
L(s, 'gnss.lla', 'LatLon/1');  L(s, 'gnss.lla', 'Alt/1');
C(s, 'Href', '-P.sensors.refLLA(3)');   % block convention h = -z - href
A(s, 'aerolibtransform2/LLA to Flat Earth', 'LLA2Flat', 'units', 'Metric (MKS)', 'LL0', 'P.sensors.refLLA(1:2)', ...
  'psi', '0');
L(s, 'LatLon/1', 'LLA2Flat/1');  L(s, 'Alt/1', 'LLA2Flat/2');  L(s, 'Href/1', 'LLA2Flat/3');
A(s, 'simulink/Math Operations/Reshape', 'XeVec', 'OutputDimensionality', '1-D array');  L(s, 'LLA2Flat/1', 'XeVec/1', 'Xe_NED');
SEL(s, 'NED2ENU', '[2 1 3]', 3);  L(s, 'XeVec/1', 'NED2ENU/1');
G(s, 'UpSign', '[1 1 -1]');  L(s, 'NED2ENU/1', 'UpSign/1');
SEL(s, 'VelEN', '[1 2]', 3);  L(s, 'gnss.vel_ENU', 'VelEN/1');
% heading psi = wrap(pi/2 - heading_deg*pi/180 - psiBaseline)
A(s, 'simulink/Math Operations/Gain', 'HdgRad', 'Gain', '-pi/180');  L(s, 'gnss.heading_deg', 'HdgRad/1');
A(s, 'simulink/Math Operations/Bias', 'HdgENU', 'Bias', 'pi/2 - P.sensors.gnss.psiBaseline');  L(s, 'HdgRad/1', 'HdgENU/1');
WRAP(s, 'PsiG', 'HdgENU/1');
% slow-side vector (double): [enu(3) vel(2) psiG hAcc vAcc sAcc headAcc_rad fix headingValid newData t_meas]
for b = {'hAccD', 'vAccD', 'sAccD', 'headAccD', 'fixD', 'hValD', 'newD'}, DTC(s, b{1}, 'double'); end
L(s, 'gnss.hAcc', 'hAccD/1');  L(s, 'gnss.vAcc', 'vAccD/1');  L(s, 'gnss.sAcc', 'sAccD/1');
L(s, 'gnss.headAcc_deg', 'headAccD/1');  L(s, 'gnss.fixType', 'fixD/1');
L(s, 'gnss.headingValid', 'hValD/1');  L(s, 'gnss.newData', 'newD/1');
A(s, 'simulink/Math Operations/Gain', 'HeadAccRad', 'Gain', 'pi/180');  L(s, 'headAccD/1', 'HeadAccRad/1');
CAT(s, 'GnssVec', 11);
src = {'UpSign/1', 'VelEN/1', 'PsiG/1', 'hAccD/1', 'vAccD/1', 'sAccD/1', 'HeadAccRad/1', 'fixD/1', 'hValD/1', ...
       'newD/1', 'gnss.t_meas'};
for k = 1:numel(src), L(s, src{k}, sprintf('GnssVec/%d', k)); end
RT(s, 'RT_gnss', ts);  L(s, 'GnssVec/1', 'RT_gnss/1', 'gnss_vec');
% new-epoch flag: counter at the GNSS rate -> RT -> Detect Change at 0.01 s
A(s, 'simulink/Sources/Counter Free-Running', 'CntG', 'NumBits', '16', 'tsamp', tg);
RT(s, 'RT_cntG', ts);  L(s, 'CntG/1', 'RT_cntG/1');
A(s, 'simulink/Logic and Bit Operations/Detect Change', 'NewG', 'vinit', '0');  L(s, 'RT_cntG/1', 'NewG/1');
% ================= fast side (0.01 s) =================
SEL(s, 'Z1', '1:3', 14);  SEL(s, 'Z2', '4:5', 14);  SEL(s, 'PsiGf', '6', 14);  SEL(s, 'AccG', '7:9', 14);
SEL(s, 'HeadAccF', '10', 14);  SEL(s, 'FixF', '11', 14);  SEL(s, 'HValF', '12', 14);  SEL(s, 'NewF', '13', 14);
SEL(s, 'TmeasF', '14', 14);
for b = {'Z1', 'Z2', 'PsiGf', 'AccG', 'HeadAccF', 'FixF', 'HValF', 'NewF', 'TmeasF'}, L(s, 'RT_gnss/1', [b{1} '/1']); end
OB(s, 'meas', 'z1', 'Z1/1');  OB(s, 'meas', 'z2', 'Z2/1');  OB(s, 'meas', 'accG', 'AccG/1');
DTC(s, 'FixU8', 'uint8');  L(s, 'FixF/1', 'FixU8/1');  OB(s, 'meas', 'fix', 'FixU8/1');
% latency tau = t - t_meas (saturated)
A(s, 'simulink/Sources/Digital Clock', 'Clock', 'SampleTime', ts);
A(s, 'simulink/Math Operations/Sum', 'Age', 'Inputs', '+-');  L(s, 'Clock/1', 'Age/1', 't');  L(s, 'TmeasF/1', 'Age/2', 't_meas');
A(s, 'simulink/Discontinuities/Saturation', 'Tau', 'UpperLimit', 'P.sensors.nav.tauMax', 'LowerLimit', '0');
L(s, 'Age/1', 'Tau/1');  OB(s, 'meas', 'tau', 'Tau/1');
% availability of GNSS position / velocity: new epoch, newData, fix > 0
CMP(s, 'NewDataB', '>', '0.5');  L(s, 'NewF/1', 'NewDataB/1');
CMP(s, 'HasFix', '>', '0');  L(s, 'FixF/1', 'HasFix/1');
LOG(s, 'AvG', 'AND', 3);  L(s, 'NewG/1', 'AvG/1', 'new_epoch');  L(s, 'NewDataB/1', 'AvG/2');  L(s, 'HasFix/1', 'AvG/3');
% dual-antenna heading
CMP(s, 'HValB', '>', '0.5');  L(s, 'HValF/1', 'HValB/1');
LOG(s, 'HeadOk', 'AND', 2);  L(s, 'HValB/1', 'HeadOk/1');  L(s, 'NewDataB/1', 'HeadOk/2');
OB(s, 'meas', 'headOkG', 'HeadOk/1');
LOG(s, 'AvH', 'AND', 2);  L(s, 'NewG/1', 'AvH/1');  L(s, 'HeadOk/1', 'AvH/2');
% ecompass heading at 10 Hz while standing still with low current
A(s, 'simulink/Sources/Pulse Generator', 'TickEc', 'PulseType', 'Sample based', 'Amplitude', '1', ...
  'Period', 'P.sensors.ekf.headDecim', 'PulseWidth', '1', 'PhaseDelay', '0', 'SampleTime', ts);
CMP(s, 'TickEcB', '>', '0.5');  L(s, 'TickEc/1', 'TickEcB/1');
LOG(s, 'AvE', 'AND', 3);  L(s, 'TickEcB/1', 'AvE/1');  L(s, 'pre.still', 'AvE/2');  L(s, 'pre.iLow', 'AvE/3');
% heading source (Multiport Switch: 1 dual antenna, 2 ecompass; 0 = none)
A(s, 'simulink/Discontinuities/Saturation', 'HsCtl', 'UpperLimit', '2', 'LowerLimit', '1');  L(s, 'headSel_z/1', 'HsCtl/1', 'headSel_z');
CMP(s, 'HsOn', '>', '0.5');  L(s, 'headSel_z/1', 'HsOn/1');
G(s, 'S3G', 'P.sensors.ekf.kR');  L(s, 'HeadAccF/1', 'S3G/1', 'headAcc_rad');
C(s, 'S3Ec', 'P.sensors.nav.ecompass_sig');  C(s, 'Tau0', '0');
for b = {'Z3', 'S3', 'Tau3', 'Av3'}
    A(s, 'simulink/Signal Routing/Multiport Switch', b{1}, 'Inputs', '2', 'DataPortOrder', 'One-based contiguous');
    L(s, 'HsCtl/1', [b{1} '/1']);
end
L(s, 'PsiGf/1', 'Z3/2', 'psi_gnss');  L(s, 'attitude.psi_ec', 'Z3/3');  OB(s, 'meas', 'z3', 'Z3/1');
L(s, 'S3G/1', 'S3/2');  L(s, 'S3Ec/1', 'S3/3');  OB(s, 'meas', 's3', 'S3/1');
L(s, 'Tau/1', 'Tau3/2');  L(s, 'Tau0/1', 'Tau3/3');
L(s, 'AvH/1', 'Av3/2', 'avail_head_gnss');  L(s, 'AvE/1', 'Av3/3', 'avail_head_ec');
LOG(s, 'AvHd', 'AND', 2);  L(s, 'Av3/1', 'AvHd/1');  L(s, 'HsOn/1', 'AvHd/2');
CAT(s, 'U3', 2);  L(s, 'Tau3/1', 'U3/1', 'tau3');  L(s, 'attitude.psidot_m', 'U3/2');  OB(s, 'meas', 'u3', 'U3/1');
% ================= odometry: update every odomDecim samples (0.05 s, D13) =================
A(s, 'simulink/Sources/Pulse Generator', 'TickOd', 'PulseType', 'Sample based', 'Amplitude', '1', ...
  'Period', 'P.sensors.ekf.odomDecim', 'PulseWidth', '1', 'PhaseDelay', '2', 'SampleTime', ts);
CMP(s, 'TickOdB', '>', '0.5');  L(s, 'TickOd/1', 'TickOdB/1');
LOG(s, 'AvO', 'AND', 2);  L(s, 'TickOdB/1', 'AvO/1');  L(s, 'pre.odomValid', 'AvO/2');
OBP(s, 'meas', 'z4', 'meas_z1', 'pre.vTrack');
C(s, 'Gauge', 'P.sensors.geom.B');
CAT(s, 'U4', 2);  L(s, 'attitude.wz', 'U4/1');  L(s, 'Gauge/1', 'U4/2');  OB(s, 'meas', 'u4', 'U4/1');
% ================= scan match (0.2 s -> 0.01 s) =================
DTC(s, 'SmValD', 'double');  L(s, 'scanmatch.valid', 'SmValD/1', 'sm_valid');
A(s, 'simulink/Math Operations/Reshape', 'CovVec', 'OutputDimensionality', '1-D array');  L(s, 'scanmatch.cov', 'CovVec/1', 'sm_cov');
CAT(s, 'SmVec', 4);  L(s, 'SmValD/1', 'SmVec/1');  L(s, 'scanmatch.t', 'SmVec/2', 'sm_t');  L(s, 'scanmatch.pose', 'SmVec/3', 'sm_pose');
L(s, 'CovVec/1', 'SmVec/4');
RT(s, 'RT_sm', ts);  L(s, 'SmVec/1', 'RT_sm/1', 'sm_vec');
A(s, 'simulink/Sources/Counter Free-Running', 'CntS', 'NumBits', '16', 'tsamp', tsm);
RT(s, 'RT_cntS', ts);  L(s, 'CntS/1', 'RT_cntS/1');
A(s, 'simulink/Logic and Bit Operations/Detect Change', 'NewS', 'vinit', '0');  L(s, 'RT_cntS/1', 'NewS/1');
SEL(s, 'SmValF', '1', 14);  SEL(s, 'SmTF', '2', 14);  SEL(s, 'Z5', '3:5', 14);  SEL(s, 'C5v', '6:14', 14);
for b = {'SmValF', 'SmTF', 'Z5', 'C5v'}, L(s, 'RT_sm/1', [b{1} '/1']); end
% scan age tau5 = t - sm.t (latency back-extrapolation of channel 5), u5 = [tau5; psidot_m]
A(s, 'simulink/Math Operations/Sum', 'AgeS', 'Inputs', '+-');  L(s, 'Clock/1', 'AgeS/1');  L(s, 'SmTF/1', 'AgeS/2', 'sm_t');
A(s, 'simulink/Discontinuities/Saturation', 'Tau5', 'UpperLimit', 'P.sensors.nav.tauMaxSm', 'LowerLimit', '0');
L(s, 'AgeS/1', 'Tau5/1');
CAT(s, 'U5', 2);  L(s, 'Tau5/1', 'U5/1', 'tau5');  L(s, 'attitude.psidot_m', 'U5/2');  OB(s, 'meas', 'u5', 'U5/1');
A(s, 'simulink/Math Operations/Reshape', 'C5m', 'OutputDimensionality', 'Customize', 'OutputDimensions', '[3 3]');
L(s, 'C5v/1', 'C5m/1');  OB(s, 'meas', 'C5', 'C5m/1');  OB(s, 'meas', 'z5', 'Z5/1');
CMP(s, 'SmValB', '>', '0.5');  L(s, 'SmValF/1', 'SmValB/1');
LOG(s, 'AvS', 'AND', 2);  L(s, 'NewS/1', 'AvS/1', 'new_match');  L(s, 'SmValB/1', 'AvS/2');
% ================= availability vector (masked by the mode manager) =================
CAT(s, 'AvRaw', 5);
L(s, 'AvG/1', 'AvRaw/1', 'avail_gnss');  L(s, 'AvG/1', 'AvRaw/2');  L(s, 'AvHd/1', 'AvRaw/3', 'avail_head');
L(s, 'AvO/1', 'AvRaw/4', 'avail_odom');  L(s, 'AvS/1', 'AvRaw/5', 'avail_sm');
LOG(s, 'AvMask', 'AND', 2);  L(s, 'AvRaw/1', 'AvMask/1');  L(s, 'chMask_z/1', 'AvMask/2', 'chMask_z');
OB(s, 'meas', 'avail', 'AvMask/1');
% ================= extra inputs u1 = u2 = [r_a; phi; theta; tau; psidot_m] =================
C(s, 'Ra', 'P.sensors.gnss.rAnt1(:)', 'VectorParams1D', 'on');
CAT(s, 'U1', 4);  L(s, 'Ra/1', 'U1/1');  L(s, 'attitude.att', 'U1/2');  L(s, 'Tau/1', 'U1/3');  L(s, 'attitude.psidot_m', 'U1/4');
OB(s, 'meas', 'u1', 'U1/1');  OB(s, 'meas', 'u2', 'U1/1', 'u1');
shaddad_bu('portorder', s, {'gnss', 'scanmatch', 'pre', 'attitude', 'headSel_z', 'chMask_z'}, 'Inport');
end

% --------------------------------------------------------------------------------------------
function local_ekfinputs(n)
% in pre, attitude (In Bus Element: imu9; att, psidot_m), denied, x_prev; out u, Q
s = SUB(n, 'EkfInputs', {'denied', 'x_prev'}, {'u', 'Q'});
ek = 'P.sensors.ekf';
SEL(s, 'Fx', '1', 9);  L(s, 'pre.imu9', 'Fx/1');
SEL(s, 'Theta', '2', 2);  L(s, 'attitude.att', 'Theta/1');
TRIG(s, 'SinT', 'sin');  TRIG(s, 'CosT', 'cos');  L(s, 'Theta/1', 'SinT/1', 'theta');  L(s, 'Theta/1', 'CosT/1');
C(s, 'Consts1', sprintf('[%s.Ts; P.sensors.g; %s.tau_vy]', ek, ek));
% ---- D49 slip hold: sample-and-hold of the slip estimates at denial onset ----
SEL(s, 'Slip', '9:10', 11);  L(s, 'x_prev/1', 'Slip/1', 'x_prev');
% held value = slip estimate low-pass filtered with tauSlipHold (y = a y[k-1] + (1 - a) s, a = exp(-Ts/tau)), so the
% hold does not freeze a momentary excursion of the estimate
G(s, 'LpfIn', sprintf('1 - exp(-Ts.nav/%s.tauSlipHold)', ek));  G(s, 'LpfFb', sprintf('exp(-Ts.nav/%s.tauSlipHold)', ek));
A(s, 'simulink/Math Operations/Sum', 'SlipLpf', 'Inputs', '++');
UD(s, 'SlipLpf_z', sprintf('[%s.s0; %s.s0]', ek, ek), 'Ts.nav');
L(s, 'Slip/1', 'LpfIn/1', 'slip_prev');  L(s, 'LpfIn/1', 'SlipLpf/1');  L(s, 'SlipLpf_z/1', 'LpfFb/1');  L(s, 'LpfFb/1', 'SlipLpf/2');
L(s, 'SlipLpf/1', 'SlipLpf_z/1', 'slip_lpf');
SW(s, 'SlipHold');  UD(s, 'SlipHold_z', sprintf('[%s.s0; %s.s0]', ek, ek), 'Ts.nav');
L(s, 'SlipHold_z/1', 'SlipHold/1');  L(s, 'denied/1', 'SlipHold/2', 'denied');  L(s, 'SlipLpf/1', 'SlipHold/3');
L(s, 'SlipHold/1', 'SlipHold_z/1', 'slip_held');
C(s, 'S0', sprintf('[%s.s0; %s.s0]', ek, ek));
SW(s, 'S0Sel');  L(s, 'SlipHold/1', 'S0Sel/1');  L(s, 'denied/1', 'S0Sel/2');  L(s, 'S0/1', 'S0Sel/3');
C(s, 'TauSDen', [ek '.tau_s_denied']);  C(s, 'TauS', [ek '.tau_s']);
SW(s, 'TauSel');  L(s, 'TauSDen/1', 'TauSel/1');  L(s, 'denied/1', 'TauSel/2');  L(s, 'TauS/1', 'TauSel/3');
C(s, 'Consts2', sprintf('[%s.tau_chi; %s.chi0; %s.slip_clamp(:); %s.chi_clamp(:)]', ek, ek, ek, ek));
CAT(s, 'U', 8);
src = {'attitude.psidot_m', 'Fx/1', 'SinT/1', 'CosT/1', 'Consts1/1', 'TauSel/1', 'S0Sel/1', 'Consts2/1'};
nm = {'psidot_m', 'fx', 'sin_theta', 'cos_theta', '', 'tau_s', 's0_LR', ''};
for k = 1:numel(src), L(s, src{k}, sprintf('U/%d', k), nm{k}); end
L(s, 'U/1', 'u/1', 'u');
% ---- process noise (time-varying, D49) ----
C(s, 'QdDen', [ek '.Qd_denied']);  C(s, 'Qd', [ek '.Qd']);
SW(s, 'QSel');  L(s, 'QdDen/1', 'QSel/1');  L(s, 'denied/1', 'QSel/2');  L(s, 'Qd/1', 'QSel/3');
L(s, 'QSel/1', 'Q/1', 'Q');
shaddad_bu('portorder', s, {'pre', 'attitude', 'denied', 'x_prev'}, 'Inport');
end

% --------------------------------------------------------------------------------------------
function local_ekf(n)
% in u, Q, meas, gate (In Bus Element: z1 ... z5, u1 ... u5; en, R1 ... R5), hold; reset navReset; out xhat, P
rs = A(n, 'simulink/Ports & Subsystems/Resettable Subsystem', 'NavEKF');
lns = find_system(rs, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
b = setdiff(find_system(rs, 'SearchDepth', 1), {rs});
for k = 1:numel(b)
    if ~strcmp(get_param(b{k}, 'BlockType'), 'ResetPort'), delete_block(b{k}); end
end
rp = find_system(rs, 'SearchDepth', 1, 'BlockType', 'ResetPort');
set_param(rp{1}, 'ResetTriggerType', 'rising', 'Name', 'navReset');
ins = {'u', 'Q', 'hold'};
for k = 1:numel(ins), add_block('simulink/Sources/In1', [rs '/' ins{k}], 'Port', num2str(k)); end
add_block('simulink/Sinks/Out1', [rs '/xhat'], 'Port', '1');
add_block('simulink/Sinks/Out1', [rs '/P'], 'Port', '2');
A(rs, 'simulink/User-Defined Functions/MATLAB System', 'EKF', 'System', 'sens_nav_ekf', ...
  'InitialState', 'P.sensors.ekf.x0', 'InitialStateCovariance', 'P.sensors.ekf.P0');
A(rs, 'simulink/Signal Routing/Demux', 'EnSplit', 'Outputs', '5');  L(rs, 'gate.en', 'EnSplit/1');
order = {'u/1', 'Q/1', 'meas.z1', 'EnSplit/1', 'gate.R1', 'meas.u1', 'meas.z2', 'EnSplit/2', 'gate.R2', 'meas.u2', ...
         'meas.z3', 'EnSplit/3', 'gate.R3', 'meas.u3', 'meas.z4', 'EnSplit/4', 'gate.R4', 'meas.u4', 'meas.z5', ...
         'EnSplit/5', 'gate.R5', 'meas.u5', 'hold/1'};
for k = 1:numel(order)
    nm = '';
    if startsWith(order{k}, 'EnSplit'), nm = strrep(order{k}, 'EnSplit/', 'en'); end
    L(rs, order{k}, sprintf('EKF/%d', k), nm);
end
A(rs, 'simulink/Math Operations/Reshape', 'XVec', 'OutputDimensionality', '1-D array');
L(rs, 'EKF/1', 'XVec/1', 'xhat_col');  L(rs, 'XVec/1', 'xhat/1', 'xhat');  L(rs, 'EKF/2', 'P/1', 'P');
shaddad_bu('portorder', rs, {'u', 'Q', 'meas', 'gate', 'hold'}, 'Inport');
set_param(rs, 'Description', ['EKF (MATLAB System sens_nav_ekf = extendedKalmanFilter, predict + 5 gated corrections ' ...
    'at 0.01 s) inside a Resettable Subsystem: navReset rising edge / FAULT re-initialise x0 / P0 (T-NAV-09).']);
end

% --------------------------------------------------------------------------------------------
function local_output(n)
% in pre, attitude, meas, gate, mode (In Bus Element ports, the picks right in front of each consumer), xhat, P; out
% nav_state, nav_diag
s = SUB(n, 'NavOutput', {'xhat', 'P'}, {'nav_state', 'nav_diag'});
ts = 'Ts.nav';
A(s, 'simulink/Sources/Digital Clock', 'Clock', 'SampleTime', ts);
SEL(s, 'Pos', '1:3', 11);  SEL(s, 'Psi', '4', 11);  SEL(s, 'V', '5', 11);  SEL(s, 'Vy', '6', 11);
SEL(s, 'Bg', '7', 11);  SEL(s, 'Ba', '8', 11);  SEL(s, 'Slip', '9:10', 11);  SEL(s, 'Chi', '11', 11);
for b = {'Pos', 'Psi', 'V', 'Vy', 'Bg', 'Ba', 'Slip', 'Chi'}, L(s, 'xhat/1', [b{1} '/1'], 'xhat'); end
SEL(s, 'Phi', '1', 2);  SEL(s, 'Theta', '2', 2);  L(s, 'attitude.att', 'Phi/1');  L(s, 'attitude.att', 'Theta/1');
WRAP(s, 'PsiW', 'Psi/1');
% ---- eul = [yaw pitch roll], q_WB (frame rotation W->B, scalar first) ----
CAT(s, 'Eul', 3);  L(s, 'PsiW/1', 'Eul/1', 'yaw');  L(s, 'Theta/1', 'Eul/2', 'pitch');  L(s, 'Phi/1', 'Eul/3', 'roll');
A(s, 'navlib/Utilities/Coordinate Transformation Conversion', 'Eul2Q', 'InputRepresentation', 'Euler Angles', ...
  'InputEulerSequence', 'ZYX', 'OutputRepresentation', 'Quaternion');
A(s, 'simulink/Math Operations/Reshape', 'EulCol', 'OutputDimensionality', 'Column vector (2-D)');
L(s, 'Eul/1', 'EulCol/1', 'eul');  L(s, 'EulCol/1', 'Eul2Q/1');
A(s, 'simulink/Math Operations/Reshape', 'QVec', 'OutputDimensionality', '1-D array');  L(s, 'Eul2Q/1', 'QVec/1', 'q_WB');
% ---- LLA (Flat Earth to LLA, href input port = -P.world.lla0(3): the block computes h = -z - href, D62) ----
SEL(s, 'NE', '[2 1 3]', 3);  L(s, 'Pos/1', 'NE/1', 'pos_W');
G(s, 'NED', '[1 1 -1]');  L(s, 'NE/1', 'NED/1');
C(s, 'Href', '-P.world.lla0(3)');   % block convention h = -z - href (D62 input port)
A(s, 'aerolibtransform2/Flat Earth to LLA', 'Flat2LLA', 'units', 'Metric (MKS)', 'LL0', 'P.sensors.refLLA(1:2)', 'psi', '0');
L(s, 'NED/1', 'Flat2LLA/1', 'pos_NED');  L(s, 'Href/1', 'Flat2LLA/2');
A(s, 'simulink/Math Operations/Reshape', 'LatLonV', 'OutputDimensionality', '1-D array');  L(s, 'Flat2LLA/1', 'LatLonV/1');
CAT(s, 'LLA', 2);  L(s, 'LatLonV/1', 'LLA/1', 'latlon');  L(s, 'Flat2LLA/2', 'LLA/2', 'h');
% ---- velocities ----
C(s, 'Zero', '0');
CAT(s, 'VelB', 3);  L(s, 'V/1', 'VelB/1', 'v');  L(s, 'Vy/1', 'VelB/2', 'vy');  L(s, 'Zero/1', 'VelB/3');
CAT(s, 'VwIn', 4);  L(s, 'V/1', 'VwIn/1');  L(s, 'Vy/1', 'VwIn/2');  L(s, 'PsiW/1', 'VwIn/3');  L(s, 'Theta/1', 'VwIn/4');
FCN(s, 'VE', 'u(1)*cos(u(4))*cos(u(3)) - u(2)*sin(u(3))');
FCN(s, 'VN', 'u(1)*cos(u(4))*sin(u(3)) + u(2)*cos(u(3))');
FCN(s, 'VU', '-u(1)*sin(u(4))');
CAT(s, 'VelW', 3);
for k = 1:3
    b = {'VE', 'VN', 'VU'};
    L(s, 'VwIn/1', [b{k} '/1']);  L(s, [b{k} '/1'], sprintf('VelW/%d', k));
end
% ---- rates, kinematic acceleration ----
A(s, 'simulink/Math Operations/Sum', 'YawRate', 'Inputs', '+-');  L(s, 'attitude.psidot_m', 'YawRate/1');
L(s, 'Bg/1', 'YawRate/2', 'bg');
SEL(s, 'Gyro', '4:6', 9);  SEL(s, 'Fsp', '1:3', 9);  L(s, 'pre.imu9', 'Gyro/1');  L(s, 'pre.imu9', 'Fsp/1');
CAT(s, 'BgVec', 3);  L(s, 'Zero/1', 'BgVec/1');  L(s, 'Zero/1', 'BgVec/2');  L(s, 'Bg/1', 'BgVec/3');
A(s, 'simulink/Math Operations/Sum', 'OmegaB', 'Inputs', '+-');  L(s, 'Gyro/1', 'OmegaB/1', 'gyro');  L(s, 'BgVec/1', 'OmegaB/2');
C(s, 'Grav', 'P.sensors.g');
CAT(s, 'AccIn', 5);  L(s, 'Fsp/1', 'AccIn/1', 'f_B');  L(s, 'Theta/1', 'AccIn/2');  L(s, 'Phi/1', 'AccIn/3');
L(s, 'Ba/1', 'AccIn/4', 'ba');  L(s, 'Grav/1', 'AccIn/5');
FCN(s, 'AX', 'u(1) + u(7)*sin(u(4)) - u(6)');
FCN(s, 'AY', 'u(2) - u(7)*cos(u(4))*sin(u(5))');
FCN(s, 'AZ', 'u(3) - u(7)*cos(u(4))*cos(u(5))');
CAT(s, 'AccB', 3);
for k = 1:3
    b = {'AX', 'AY', 'AZ'};
    L(s, 'AccIn/1', [b{k} '/1']);  L(s, [b{k} '/1'], sprintf('AccB/%d', k));
end
% ---- covariance outputs ----
A(s, 'simulink/Math Operations/Reshape', 'Pvec', 'OutputDimensionality', '1-D array');  L(s, 'P/1', 'Pvec/1', 'P');
SEL(s, 'Pdiag', '1:12:121', 121);  L(s, 'Pvec/1', 'Pdiag/1');
SEL(s, 'PdPos', '1:3', 11);  SEL(s, 'PdYaw', '4', 11);  SEL(s, 'PdVel', '5', 11);
for b = {'PdPos', 'PdYaw', 'PdVel'}, L(s, 'Pdiag/1', [b{1} '/1'], 'P_diag'); end
for b = {'PosStd', 'YawStd', 'VelStd'}
    A(s, 'simulink/Math Operations/Math Function', b{1}, 'Operator', 'sqrt');
end
L(s, 'PdPos/1', 'PosStd/1');  L(s, 'PdYaw/1', 'YawStd/1');  L(s, 'PdVel/1', 'VelStd/1');
A(s, 'simulink/Signal Routing/Selector', 'CovPose', 'NumberOfDimensions', '2', 'IndexMode', 'One-based', ...
  'IndexOptions', 'Index vector (dialog),Index vector (dialog)', 'IndexParamArray', {'[1 2 4]', '[1 2 4]'});
L(s, 'P/1', 'CovPose/1');
A(s, 'simulink/Math Operations/Sum', 'Trace', 'Inputs', '+', 'CollapseMode', 'All dimensions');
L(s, 'Pdiag/1', 'Trace/1');
% ---- Bus_NavState (ICD element order) ----
BC(s, 'BC_NavState', 'Bus_NavState', {'Clock/1', 'Pos/1', 'LLA/1', 'Eul/1', 'QVec/1', 'VelB/1', 'VelW/1', 'YawRate/1', ...
    'OmegaB/1', 'AccB/1', 'Slip/1', 'Chi/1', 'Bg/1', 'Ba/1', 'PosStd/1', 'YawStd/1', 'VelStd/1', 'CovPose/1', ...
    'mode.quality', 'mode.navValid', 'mode.headingValid', 'mode.aligned', 'mode.lidarLocOn', 'meas.fix', ...
    'mode.tSinceAbs', 'mode.resetCount'});
L(s, 'BC_NavState/1', 'nav_state/1', 'nav_state');
% ---- Bus_NavDiag ----
BC(s, 'BC_NavDiag', 'Bus_NavDiag', {'gate.nis', 'gate.en', 'gate.rej', 'meas.tau', 'Trace/1', 'Pdiag/1', 'xhat/1', ...
    'mode.modeState'});
L(s, 'BC_NavDiag/1', 'nav_diag/1', 'nav_diag');
shaddad_bu('portorder', s, {'pre', 'attitude', 'meas', 'gate', 'mode', 'xhat', 'P'}, 'Inport');
end

% ============================================================================================
%  sens_acq_gnss (HW choice of NavFusion/IO_In)
% ============================================================================================
function local_sens_acq_gnss(n)
ts = 'P.sensors.acq_gnss.Ts';
add_block('simulink/Sinks/Out1', [n '/gnss'], 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_GnssRaw');
sub = {'SubFix', '/fix', 'sensor_msgs/NavSatFix'; 'SubVel', '/fix_velocity', 'geometry_msgs/TwistWithCovarianceStamped'; ...
       'SubHead', '/navheading', 'sensor_msgs/Imu'};
for k = 1:size(sub, 1)
    A(n, 'ros2lib/Subscribe', sub{k, 1}, 'topicSource', 'Specify your own', 'topic', sub{k, 2}, ...
      'messageType', sub{k, 3}, 'SubscriberMode', 'At sample time');
    set_param([n '/' sub{k, 1}], 'sampleTime', ts);
end
% ---- /fix: time of validity, LLA, accuracies, fix type ----
DTC(n, 'SecD', 'double');  DTC(n, 'NsD', 'double');
PICK(n, 'SubFix/2', 'FixSel_sec', {'header.stamp.sec'}, {'SecD/1'}, 'fix_msg');
PICK(n, 'SubFix/2', 'FixSel_nanosec', {'header.stamp.nanosec'}, {'NsD/1'});
A(n, 'simulink/Math Operations/Gain', 'NsS', 'Gain', '1e-9');  L(n, 'NsD/1', 'NsS/1');
A(n, 'simulink/Math Operations/Sum', 'Tmeas', 'Inputs', '++');  L(n, 'SecD/1', 'Tmeas/1');  L(n, 'NsS/1', 'Tmeas/2');
CAT(n, 'LLA', 3);  PICK(n, 'SubFix/2', 'FixSel_lla', {'latitude', 'longitude', 'altitude'}, {'LLA/1', 'LLA/2', 'LLA/3'});
SEL(n, 'CovE', '1', 9);  SEL(n, 'CovU', '9', 9);
PICK(n, 'SubFix/2', 'FixSel_covE', {'position_covariance'}, {'CovE/1'});
PICK(n, 'SubFix/2', 'FixSel_covU', {'position_covariance'}, {'CovU/1'});
for b = {'HAccR', 'VAccR'}, A(n, 'simulink/Math Operations/Math Function', b{1}, 'Operator', 'sqrt'); end
L(n, 'CovE/1', 'HAccR/1');  L(n, 'CovU/1', 'VAccR/1');
DTC(n, 'HAccS', 'single');  DTC(n, 'VAccS', 'single');  L(n, 'HAccR/1', 'HAccS/1', 'hAcc');  L(n, 'VAccR/1', 'VAccS/1', 'vAcc');
% NavSatStatus -1 NO_FIX -> 0, 0 FIX -> 1 SINGLE, 1 SBAS -> 1, 2 GBAS (RTK) -> 4 fixed if hAcc < rtkFixedAcc else 5 float
DTC(n, 'StatD', 'double');  PICK(n, 'SubFix/2', 'FixSel_status', {'status.status'}, {'StatD/1'});
A(n, 'simulink/Math Operations/Bias', 'StatIdx', 'Bias', '2');  L(n, 'StatD/1', 'StatIdx/1');
A(n, 'simulink/Discontinuities/Saturation', 'StatSat', 'UpperLimit', '4', 'LowerLimit', '1');  L(n, 'StatIdx/1', 'StatSat/1');
C(n, 'FixNo', '0');  C(n, 'FixSingle', '1');  C(n, 'FixRtk4', '4');  C(n, 'FixRtk5', '5');
CMP(n, 'IsFixed', '<', 'P.sensors.acq_gnss.rtkFixedAcc');  L(n, 'HAccR/1', 'IsFixed/1');
SW(n, 'RtkSel');  L(n, 'FixRtk4/1', 'RtkSel/1');  L(n, 'IsFixed/1', 'RtkSel/2');  L(n, 'FixRtk5/1', 'RtkSel/3');
A(n, 'simulink/Signal Routing/Multiport Switch', 'FixMap', 'Inputs', '4', 'DataPortOrder', 'One-based contiguous');
L(n, 'StatSat/1', 'FixMap/1');  L(n, 'FixNo/1', 'FixMap/2');  L(n, 'FixSingle/1', 'FixMap/3');
L(n, 'FixSingle/1', 'FixMap/4');  L(n, 'RtkSel/1', 'FixMap/5');
DTC(n, 'FixU8', 'uint8');  L(n, 'FixMap/1', 'FixU8/1', 'fixType');
C(n, 'NumSV', 'uint8(0)', 'OutDataTypeStr', 'uint8');
% ---- /fix_velocity: ENU velocity and speed accuracy ----
CAT(n, 'VelENU', 3);
PICK(n, 'SubVel/2', 'VelSel_linear', {'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z'}, ...
    {'VelENU/1', 'VelENU/2', 'VelENU/3'}, 'vel_msg');
SEL(n, 'CovV', '1', 36);  PICK(n, 'SubVel/2', 'VelSel_cov', {'twist.covariance'}, {'CovV/1'});
A(n, 'simulink/Math Operations/Math Function', 'SAccR', 'Operator', 'sqrt');  L(n, 'CovV/1', 'SAccR/1');
DTC(n, 'SAccS', 'single');  L(n, 'SAccR/1', 'SAccS/1', 'sAcc');
A(n, 'simulink/Sinks/Terminator', 'T_velNew');  L(n, 'SubVel/1', 'T_velNew/1', 'vel_new');
% ---- /navheading: baseline yaw (ENU) -> heading CW from North, accuracy, validity ----
CAT(n, 'Quat', 4);
PICK(n, 'SubHead/2', 'HeadSel_q', {'orientation.w', 'orientation.x', 'orientation.y', 'orientation.z'}, ...
    {'Quat/1', 'Quat/2', 'Quat/3', 'Quat/4'}, 'heading_msg');
A(n, 'simulink/Math Operations/Reshape', 'QuatCol', 'OutputDimensionality', 'Column vector (2-D)');
L(n, 'Quat/1', 'QuatCol/1');
A(n, 'navlib/Utilities/Coordinate Transformation Conversion', 'Q2Eul', 'InputRepresentation', 'Quaternion', ...
  'OutputRepresentation', 'Euler Angles', 'OutputEulerSequence', 'ZYX');
L(n, 'QuatCol/1', 'Q2Eul/1');
A(n, 'simulink/Math Operations/Reshape', 'EulV', 'OutputDimensionality', '1-D array');  L(n, 'Q2Eul/1', 'EulV/1');
SEL(n, 'Yaw', '1', 3);  L(n, 'EulV/1', 'Yaw/1');
A(n, 'simulink/Math Operations/Gain', 'YawDeg', 'Gain', '-180/pi');  L(n, 'Yaw/1', 'YawDeg/1', 'yaw_enu');
A(n, 'simulink/Math Operations/Bias', 'HdgCW', 'Bias', '90');  L(n, 'YawDeg/1', 'HdgCW/1');
C(n, 'C360', '360');
A(n, 'simulink/Math Operations/Math Function', 'HdgMod', 'Operator', 'mod');
L(n, 'HdgCW/1', 'HdgMod/1');  L(n, 'C360/1', 'HdgMod/2');
SEL(n, 'CovYaw', '9', 9);  PICK(n, 'SubHead/2', 'HeadSel_cov', {'orientation_covariance'}, {'CovYaw/1'});
A(n, 'simulink/Math Operations/Math Function', 'HdgAccR', 'Operator', 'sqrt');  L(n, 'CovYaw/1', 'HdgAccR/1');
A(n, 'simulink/Math Operations/Gain', 'HdgAccDeg', 'Gain', '180/pi');  L(n, 'HdgAccR/1', 'HdgAccDeg/1');
DTC(n, 'HdgAccS', 'single');  L(n, 'HdgAccDeg/1', 'HdgAccS/1', 'headAcc_deg');
CMP(n, 'HdgAccOK', '<', 'P.sensors.acq_gnss.headAccMax_deg');  L(n, 'HdgAccDeg/1', 'HdgAccOK/1');
LOG(n, 'HdgValid', 'AND', 2);  L(n, 'SubHead/1', 'HdgValid/1', 'heading_new');  L(n, 'HdgAccOK/1', 'HdgValid/2');
C(n, 'CorrAge', 'single(0)', 'OutDataTypeStr', 'single');
% ---- Bus_GnssRaw ----
BC(n, 'BC_Gnss', 'Bus_GnssRaw', {'Tmeas/1', 'LLA/1', 'VelENU/1', 'FixU8/1', 'NumSV/1', 'HAccS/1', 'VAccS/1', 'SAccS/1', ...
    'HdgValid/1', 'HdgMod/1', 'HdgAccS/1', 'CorrAge/1', 'SubFix/1'});
L(n, 'BC_Gnss/1', 'gnss/1', 'gnss');
set_param(n, 'Description', sprintf(['sens_acq_gnss (T12, HW choice of Shaddad_NavFusion/IO_In, VC_TARGET == 1; 0.1 s). ' ...
    'Ports: out 1 gnss Bus_GnssRaw 0.1. ros2lib/Subscribe (At sample time) of the ROS 2 ublox driver standard topics: ' ...
    '/fix sensor_msgs/NavSatFix (t_meas = header.stamp, lla, hAcc / vAcc = sqrt(position_covariance(1 / 9)), fixType ' ...
    'from status: -1 -> 0, 0 / 1 -> 1, 2 -> 4 if hAcc < rtkFixedAcc else 5; newData = IsNew), /fix_velocity ' ...
    'geometry_msgs/TwistWithCovarianceStamped (vel_ENU, sAcc), /navheading sensor_msgs/Imu (baseline yaw ENU -> ' ...
    'heading_deg = mod(90 - yaw, 360), headAcc_deg, headingValid = IsNew and headAcc < headAccMax_deg). numSV and ' ...
    'corrAge are not carried by the standard messages (0). Parameters P.sensors.acq_gnss.']));
end
