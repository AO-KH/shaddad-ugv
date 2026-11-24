function build_structure(sel)
%BUILD_STRUCTURE  Build the STRUCTURE plant Subsystem References of build task T03 (owner: structure).
%
%   build_structure()            rebuild every file (dependency order, leaves first)
%   build_structure(names)       rebuild only the listed files (cellstr of file base names)
%
%   Files (subsystems/structure/, all Subsystem Reference files, docs/specs/structure.md 7, revision B):
%     structure_track_side    sprocket / belt Simscape network of one side (mask 'side'), port Spr
%     structure_track_shear   28 isotropic 2-D shear states (Integrator, limits +/-1), per-track sums,
%                             1 ms Unit Delay of the belt speeds (D36 loop breaker of the damping term)
%     structure_contact       contact-point kinematics, 2-D Lookup Tables on P.world, soil override
%                             (Direct Lookup Table + Switch), MF-A trackContactNormal, MF-B trackPlanarForces,
%                             MF-C1 / MF-C2 the gauge-wheel contact of the rigid mower deck (2026-11-15)
%     structure_body6dof      Simple Variable Mass 6DOF (Euler Angles), 6DOF (Angular) Acceleration,
%                             gravity, wrench sum, r_cg(m), implement wrench power on the chassis
%     structure_truth         NED/FRD -> ENU/FLU truth (Bus_TruthState), pose2D, Flat Earth to LLA (href port)
%     structure_hifi          HiFi chassis (VC_STRUCT_FID == 1): the files above + Bus_TrackState and
%                             Bus_StructureDiag (energy bookkeeping, D40)
%     structure_lofi          LoFi (VC_STRUCT_FID == 2): Differential Drive Kinematic Model + quasi-static
%                             sprocket loads (Simscape), same buses
%     structure_impl_none / _mower / _tiller / _sprayer   implement load mechanics (VC_IMPL choices) with the
%                             inner VC_IMPL_DRIVE variant (1 ideal speed source, harness only; 2 Impl_Shaft)
%     structure_implement     VC_IMPL Variant Subsystem, impl_mech at 0.01 s (internal Zero-Order Hold, D32),
%                             E_impl_shaft_J integral (D40)
%     structure_plant         domain top (ICD 3.2 ports): Fidelity variant, implement, structure_env (T02)
%   Rules: every number from P / Ts; MATLAB Function bodies in structure_contact_normal.m /
%   structure_planar_forces.m / structure_gauge_points.m / structure_gauge_contact.m (copied verbatim into the
%   blocks as local functions, see MFB), the
%   P.structure.contact / .track numbers on Constant inputs (R2026b JIT relink workaround, see MFB);
%   no Solver Configuration; explicit Rate Transitions (0.1 / 0.01 -> 0.001); Unit Delays on every
%   Simscape -> Simulink -> Simscape path (D36). Idempotent (close, delete, new_system, save).
%   Layout (model layout spec 6, 7): no block positions here; shaddad_bu('save') lays out every level of each
%   file with shaddad_layout before it saves. Every signal line is named (L5, see L). Buses (spec 6.3):
%   structure_body6dof sends its 12 outputs as the virtual bus 'body' (Out Bus Element port), which
%   structure_contact and structure_truth pick with In Bus Element ports and structure_hifi with a Bus Selector
%   in front of each consumer; structure_implement sends wrench, dmdt and E_impl as the bus 'impl', picked in
%   front of the Fidelity variant of structure_plant. Every other bus is picked by a Bus Selector in front of each
%   consumer (spec 6.2, PICK).

names = {'structure_track_side', 'structure_track_shear', 'structure_contact', 'structure_body6dof', ...
    'structure_truth', 'structure_hifi', 'structure_lofi', 'structure_impl_none', 'structure_impl_mower', ...
    'structure_impl_tiller', 'structure_impl_sprayer', 'structure_implement', 'structure_plant'};
fns = {@local_track_side, @local_track_shear, @local_contact, @local_body6dof, @local_truth, @local_hifi, ...
    @local_lofi, @local_impl_none, @local_impl_mower, @local_impl_tiller, @local_impl_sprayer, ...
    @local_implement, @local_plant};
if nargin < 1 || isempty(sel) || (ischar(sel) && strcmpi(sel, 'all'))
    idx = 1:numel(names);
else
    idx = find(ismember(names, cellstr(sel)));
end
root = char(currentProject().RootFolder);
d = fullfile(root, 'subsystems', 'structure');
% close this task's diagrams top-down so referenced files can be replaced
local_close([{'tb_structure'}, fliplr(names)]);
for k = idx
    n = names{k};
    f = fullfile(d, [n '.slx']);
    shaddad_bu('fresh', n, f, 'ssr');
    fns{k}(n);
    shaddad_bu('save', n, f);
    close_system(n, 0);
    local_addfile(f);
end
end

% ===========================================================================================
%  library paths and small helpers
% ===========================================================================================
function c = LIB()
c.ROT   = 'Connection: foundation.mechanical.rotational.rotational';
c.CP    = 'nesl_utility/Connection Port';
c.SPS   = 'nesl_utility/Simulink-PS Converter';
c.PSS   = 'nesl_utility/PS-Simulink Converter';
c.J     = 'fl_lib/Mechanical/Rotational Elements/Inertia';
c.FRIC  = 'fl_lib/Mechanical/Rotational Elements/Rotational Friction';
c.DAMP  = 'fl_lib/Mechanical/Rotational Elements/Rotational Damper';
c.RREF  = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
c.FREE  = 'fl_lib/Mechanical/Rotational Elements/Rotational Free End';
c.WA    = 'fl_lib/Mechanical/Mechanisms/Wheel and Axle';
c.MASS  = 'fl_lib/Mechanical/Translational Elements/Mass';
c.TREF  = sprintf('fl_lib/Mechanical/Translational Elements/Mechanical\nTranslational Reference');
c.FSRC  = 'fl_lib/Mechanical/Mechanical Sources/Ideal Force Source';
c.TSRC  = 'fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source';
c.WSRC  = 'fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source';
c.TMS   = 'fl_lib/Mechanical/Mechanical Sensors/Ideal Translational Motion Sensor';
c.RMS   = 'fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor';
c.TQS   = 'fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor';
end

function p = A(lib, path, varargin)
p = shaddad_bu('add', lib, path, varargin{:});
end

function L(sys, src, dst, nm)
% signal into dst ('Blk/k'), named nm (layout spec L5). src 'Blk/k': a line, without nm the default of
% shaddad_bu('nline') (the name the source port's line already has, else the source block's name, '_k' for
% output k of a block with several outputs; a Bus Selector output keeps '<element>'). src 'port.element': a pick
% of that element of the In Bus Element port right in front of dst (spec 6.2), its line named nm (default the
% element), shaddad_bu('pick')
if nargin < 4, nm = ''; end
if ~contains(src, '/')
    shaddad_bu('pick', sys, extractBefore(src, '.'), extractAfter(src, '.'), dst, nm);
else
    shaddad_bu('nline', sys, src, dst, nm);
end
end

function OB(sys, port, el, src)
% the element el of the Out Bus Element port 'port' of sys, fed by src ('Blk/k'), its line named el (spec 6.3):
% shaddad_bu('outbus')
shaddad_bu('outbus', sys, port, el, src);
end

function PICK(sys, name, src, sigs, dsts, bus)
% a Bus Selector name right in front of its one consumer (spec 6.2): it selects the elements sigs (cellstr) of
% the bus src ('Blk/k'), whose line is named bus; output k feeds dsts{k} ('Blk/j', or a cellstr of inputs of
% that consumer)
BSEL(sys, name, strjoin(sigs, ','));
L(sys, src, [name '/1'], bus);
for k = 1:numel(sigs)
    d = cellstr(dsts{k});
    for j = 1:numel(d), L(sys, sprintf('%s/%d', name, k), d{j}); end
end
end

function PORTS(sys, names, kind)
% number the ports of kind 'Inport' | 'Outport' of sys (classic and bus element ports) in the order of names
shaddad_bu('portorder', sys, names, kind);
end

function PL(sys, a, b)
shaddad_bu('pline', sys, a, b);
end

function ph = PH(blk)
ph = get_param(blk, 'PortHandles');
end

function IN(sys, name, port, varargin)
A('simulink/Sources/In1', [sys '/' name], 'Port', num2str(port), varargin{:});
end

function OUT(sys, name, port, varargin)
A('simulink/Sinks/Out1', [sys '/' name], 'Port', num2str(port), varargin{:});
end

function CPORT(sys, name, port, side)
c = LIB();
A(c.CP, [sys '/' name], 'Port', num2str(port), 'Side', side, 'ConnectionType', c.ROT);
end

function h = CPH(sys, name)          % internal node handle of a Connection Port block
ph = PH([sys '/' name]); h = ph.RConn(1);
end

function C(sys, name, val, varargin)
% VectorParams1D off: a [1 x n] / [n x 1] value stays 2-D (needed by the matrix Products)
A('simulink/Sources/Constant', [sys '/' name], 'Value', val, 'VectorParams1D', 'off', varargin{:});
end

function K(sys, name, gain, mult)
if nargin < 4, mult = 'Element-wise(K.*u)'; end
A('simulink/Math Operations/Gain', [sys '/' name], 'Gain', gain, 'Multiplication', mult);
end

function SUM(sys, name, signs)
A('simulink/Math Operations/Sum', [sys '/' name], 'Inputs', signs, 'IconShape', 'rectangular');
end

function PROD(sys, name, ins, mode)
if nargin < 4, mode = 'Element-wise(.*)'; end
A('simulink/Math Operations/Product', [sys '/' name], 'Inputs', ins, 'Multiplication', mode);
end

function MM(sys, name, n)            % matrix product of n inputs
if nargin < 3, n = 2; end
PROD(sys, name, repmat('*', 1, n), 'Matrix(*)');
end

function MUX(sys, name, n)
A('simulink/Signal Routing/Mux', [sys '/' name], 'Inputs', num2str(n));
end

function DEMUX(sys, name, n)
A('simulink/Signal Routing/Demux', [sys '/' name], 'Outputs', n);
end

function SEL(sys, name, idx, width)  % 1-D selector
A('simulink/Signal Routing/Selector', [sys '/' name], 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', idx, 'InputPortWidth', width);
end

function SELROW(sys, name, row)      % row of a matrix (2-D selector)
A('simulink/Signal Routing/Selector', [sys '/' name], 'NumberOfDimensions', '2', ...
    'IndexOptionArray', {'Index vector (dialog)', 'Select all'}, 'IndexParamArray', {row, '1'});
end

function COL(sys, name)              % force a 3x1 (2-D) column
A('simulink/Math Operations/Reshape', [sys '/' name], 'OutputDimensionality', 'Column vector (2-D)');
end

function VEC(sys, name)              % force a 1-D vector
A('simulink/Math Operations/Reshape', [sys '/' name], 'OutputDimensionality', '1-D array');
end

function TRIG(sys, name, op)
A('simulink/Math Operations/Trigonometric Function', [sys '/' name], 'Operator', op);
end

function XPROD(sys, name)
A('aerolibutil/3x3 Cross Product', [sys '/' name]);
end

function DOT(sys, name)
A('simulink/Math Operations/Dot Product', [sys '/' name]);
end

function TRANSP(sys, name)
A('simulink/Math Operations/Math Function', [sys '/' name], 'Operator', 'transpose');
end

function LUT2(sys, name, tab, interp)
if nargin < 4, interp = 'Linear point-slope'; end
A('simulink/Lookup Tables/2-D Lookup Table', [sys '/' name], 'Table', tab, ...
    'BreakpointsForDimension1', 'P.world.terrain.xE', 'BreakpointsForDimension2', 'P.world.terrain.yN', ...
    'InterpMethod', interp, 'ExtrapMethod', 'Clip', 'IndexSearchMethod', 'Evenly spaced points', ...
    'DiagnosticForOutOfRangeInput', 'None');
end

function RT(sys, name, tsOut, ic)
A('simulink/Signal Attributes/Rate Transition', [sys '/' name], 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Specify', 'OutPortSampleTime', tsOut, 'InitialCondition', ic);
end

function UD(sys, name, ic)
A('simulink/Discrete/Unit Delay', [sys '/' name], 'InitialCondition', ic, 'SampleTime', 'Ts.base');
end

function DTI(sys, name, ts)
if nargin < 3, ts = 'Ts.base'; end
A('simulink/Discrete/Discrete-Time Integrator', [sys '/' name], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1', 'InitialCondition', '0', 'SampleTime', ts);
end

function BSEL(sys, name, sigs)
A('simulink/Signal Routing/Bus Selector', [sys '/' name], 'OutputSignals', sigs);
end

function BC(sys, name, bus, n)
A('simulink/Signal Routing/Bus Creator', [sys '/' name], 'Inputs', num2str(n), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on');
end

function SW(sys, name)
A('simulink/Signal Routing/Switch', [sys '/' name], 'Criteria', 'u2 ~= 0');
end

function SIGC(sys, name, src)
% Signal Conversion (signal copy) that carries a bus element name different from its source line
A('simulink/Signal Attributes/Signal Conversion', [sys '/' name]);
if nargin >= 3, L(sys, src, [name '/1']); end
end

function SPSB(sys, name, unit, mode, tau)
c = LIB();
if nargin < 4, mode = 'zero'; end
if strcmp(mode, 'filter')
    A(c.SPS, [sys '/' name], 'Unit', unit, 'FilteringAndDerivatives', 'filter', ...
        'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', tau);
else
    A(c.SPS, [sys '/' name], 'Unit', unit, 'FilteringAndDerivatives', mode);
end
end

function PSSB(sys, name, unit)
c = LIB();
A(c.PSS, [sys '/' name], 'Unit', unit);
end

function mf = MFB(sys, name, script, helper, desc)
% MATLAB Function block whose body is one call of the %#codegen helper <helper>.m
% (subsystems/structure/). R2026b workaround, measured in T03:
%  (1) A MATLAB Function block that calls an EXTERNAL .m function two or more Subsystem Reference levels
%      below a model (Shaddad_Plant / tb_structure -> structure_plant -> structure_hifi ->
%      structure_contact -> MF) fails EVERY first compile after a load with the Stateflow JIT relink
%      assertion 'o != nullptr' (Stateflow.JIT.Utils.relinkToExistingJITEngine); reproduced with a
%      2-line helper in a scratch SSR chain (depth 1 ok, depth 2 / 3 fail on every reload). The helper
%      file stays the single source of the body: the build copies its text verbatim into the block as a
%      local function of the same name (rebuild with build_structure({'structure_contact'}) after
%      editing the helper; t_structure/mf_bodies_current checks that the copies are current).
%  (2) No Parameter-scope data, no mask: the numbers of the sub-structs P.structure.contact /
%      P.structure.track enter on inputs from Constant blocks (never the whole P; build_issues lesson on
%      masked MATLAB Function parameters inside Subsystem References).
mf = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', mf);
cfg = get_param(mf, 'MATLABFunctionConfiguration');
root = char(currentProject().RootFolder);
body = fileread(fullfile(root, 'subsystems', 'structure', [helper '.m']));
cfg.FunctionScript = [script sprintf('end\n\n') MFB_MARK(helper) newline body];
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.Description = desc;
end

function s = MFB_MARK(helper)
s = sprintf('%% ---- verbatim copy of subsystems/structure/%s.m (generated by build_structure, do not edit here) ----', helper);
end

function local_close(list)
for k = 1:numel(list)
    if bdIsLoaded(list{k})
        try
            close_system(list{k}, 0);
        catch tmp_T03_err
            warning('build_structure:close', 'Could not close %s: %s', list{k}, tmp_T03_err.message);
        end
    end
end
end

function local_addfile(f)
try
    pr = currentProject();
    if isempty(pr.findFile(f)), pr.addFile(f); end
catch %#ok<CTCH> project bookkeeping only
end
end

% ===========================================================================================
%  structure_track_side: sprocket / belt network of one side
% ===========================================================================================
function local_track_side(n)
c = LIB();
CPORT(n, 'Spr', 1, 'Left');
IN(n, 'Fx', 1);
OUT(n, 'v_b', 1); OUT(n, 'spr_angle', 2); OUT(n, 'spr_speed', 3); OUT(n, 'spr_torque', 4); OUT(n, 'P_fric', 5);
A(c.TQS, [n '/Sprocket_Torque_Sensor']);
A(c.J, [n '/J_eq'], 'inertia', 'P.structure.track.J_eq', 'num_ports', 'foundation.enum.numPorts2.one', ...
    'w_specify', 'on', 'w_priority', 'High', 'w', 'P.structure.init.v0/P.structure.r_sprocket');
A(c.FRIC, [n '/Internal_Track_Friction'], 'brkwy_trq', 'P.structure.track.T_brk', 'brkwy_vel', 'P.structure.track.w_brk', ...
    'Col_trq', 'P.structure.track.T_col', 'visc_coef', 'P.structure.track.c_visc');
A(c.TQS, [n '/Friction_Torque_Sensor']);
A(c.RREF, [n '/Chassis_Rot_Ref']);
A(c.RMS, [n '/Sprocket_Motion_Sensor'], 'reference', 'foundation.enum.MeasurementReference.difference', ...
    'velocity_port', 'true', 'position_port', 'true', 'acceleration_port', 'false', 'wrap_angle', 'false');
A(c.WA, [n '/Sprocket_Wheel_Axle'], 'R', 'P.structure.r_sprocket', 'or', 'foundation.enum.DriveOrientation.Positive');
A(c.MASS, [n '/Belt_Mass'], 'mass', 'P.structure.track.m_belt', 'num_ports', 'foundation.enum.numPorts2.one');
A(c.TREF, [n '/Chassis_Trans_Ref']);
A(c.FSRC, [n '/Shear_Force_Source']);
A(c.TMS, [n '/Belt_Motion_Sensor'], 'reference', 'foundation.enum.MeasurementReference.difference', ...
    'velocity_port', 'true', 'position_port', 'false', 'acceleration_port', 'false');
K(n, 'Neg_Fx', '-1');
SPSB(n, 'SPS_belt_force', 'N', 'zero');
PSSB(n, 'PSS_v_belt', 'm/s'); PSSB(n, 'PSS_angle', 'rad'); PSSB(n, 'PSS_speed', 'rad/s');
PSSB(n, 'PSS_torque', 'N*m'); PSSB(n, 'PSS_fric_torque', 'N*m');
PROD(n, 'Fric_Power', '**');
% Simulink side
L(n, 'Fx/1', 'Neg_Fx/1', 'Fx');
L(n, 'Neg_Fx/1', 'SPS_belt_force/1', 'minus_Fx');
L(n, 'PSS_v_belt/1', 'v_b/1', 'v_b');
L(n, 'PSS_angle/1', 'spr_angle/1', 'spr_angle');
L(n, 'PSS_speed/1', 'spr_speed/1', 'spr_speed');
L(n, 'PSS_torque/1', 'spr_torque/1', 'spr_torque');
L(n, 'PSS_fric_torque/1', 'Fric_Power/1', 'T_fric');
L(n, 'PSS_speed/1', 'Fric_Power/2');
L(n, 'Fric_Power/1', 'P_fric/1', 'P_fric');
% physical network (port order from the foundation .ssc sources: sensors R | C, outputs; sources R | S, C)
hS = CPH(n, 'Spr');
tq = PH([n '/Sprocket_Torque_Sensor']); J = PH([n '/J_eq']); fr = PH([n '/Internal_Track_Friction']);
ft = PH([n '/Friction_Torque_Sensor']); rr = PH([n '/Chassis_Rot_Ref']); rm = PH([n '/Sprocket_Motion_Sensor']);
wa = PH([n '/Sprocket_Wheel_Axle']); mb = PH([n '/Belt_Mass']); tr = PH([n '/Chassis_Trans_Ref']);
fs = PH([n '/Shear_Force_Source']); tm = PH([n '/Belt_Motion_Sensor']);
sps = PH([n '/SPS_belt_force']);
p1 = PH([n '/PSS_v_belt']); p2 = PH([n '/PSS_angle']); p3 = PH([n '/PSS_speed']); p4 = PH([n '/PSS_torque']);
p5 = PH([n '/PSS_fric_torque']);
PL(n, hS, tq.LConn(1));                 % Spr -> torque sensor R
nodeA = tq.RConn(1);                    % sprocket node (sensor C)
PL(n, nodeA, J.LConn(1));
PL(n, nodeA, fr.LConn(1));
PL(n, fr.RConn(1), ft.LConn(1));
PL(n, ft.RConn(1), rr.LConn(1));
PL(n, nodeA, rm.LConn(1));
PL(n, rm.RConn(1), rr.LConn(1));
PL(n, nodeA, wa.LConn(1));
nodeB = wa.RConn(1);                    % belt node
PL(n, nodeB, mb.LConn(1));
PL(n, nodeB, fs.LConn(1));
PL(n, fs.RConn(2), tr.LConn(1));
PL(n, sps.RConn(1), fs.RConn(1));
PL(n, nodeB, tm.LConn(1));
PL(n, tm.RConn(1), tr.LConn(1));
PL(n, tm.RConn(2), p1.LConn(1));
PL(n, rm.RConn(2), p3.LConn(1));
PL(n, rm.RConn(3), p2.LConn(1));
PL(n, tq.RConn(2), p4.LConn(1));
PL(n, ft.RConn(2), p5.LConn(1));
% mask: side (1 left, 2 right) - instance label of the two identical networks
try
    mk = Simulink.Mask.create(n);
    mk.addParameter('Type', 'popup', 'Name', 'side', 'Prompt', 'Track side', ...
        'TypeOptions', {'1 left', '2 right'}, 'Value', '1 left', 'Evaluate', 'off', 'Tunable', 'off');
    mk.Description = ['Sprocket / idler / road wheels (Inertia J_eq), internal track friction, belt (Wheel and Axle + Mass), ' ...
        'ground shear on the belt (Ideal Force Source S = -Fx). Positive = forward track motion on both sides.'];
catch tmp_T03_err
    warning('build_structure:mask', 'structure_track_side mask not created: %s', tmp_T03_err.message);
end
set_param(n, 'Description', ['structure_track_side (T03): sprocket/belt Simscape network of one track ' ...
    '(docs/specs/structure.md 2.7, 3.1). In Fx = sum of the shear traction of the track points [N, + forward]; ' ...
    'outputs belt speed, sprocket angle/speed/torque (input from DRIVE) and internal friction power.']);
end

% ===========================================================================================
%  structure_track_shear: 28 shear states, per-track sums, delayed belt speed
% ===========================================================================================
function local_track_shear(n)
IN(n, 'sDot', 1); IN(n, 'Fx_pt', 2); IN(n, 'vB', 3);
OUT(n, 's', 1); OUT(n, 'Fx_trk', 2); OUT(n, 'vB_d', 3);
A('simulink/Continuous/Integrator', [n '/Shear_States'], 'InitialCondition', 'zeros(4*P.structure.track.Nc,1)', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '1', 'LowerSaturationLimit', '-1', 'ZeroCross', 'off');
SEL(n, 'Points_L', '1:P.structure.track.Nc', '2*P.structure.track.Nc');
SEL(n, 'Points_R', 'P.structure.track.Nc+1:2*P.structure.track.Nc', '2*P.structure.track.Nc');
A('simulink/Math Operations/Sum of Elements', [n '/Fx_L']);
A('simulink/Math Operations/Sum of Elements', [n '/Fx_R']);
MUX(n, 'Fx_LR', 2);
UD(n, 'vB_Delay', 'P.structure.init.v0*[1;1]');
L(n, 'sDot/1', 'Shear_States/1', 'sDot');
L(n, 'Shear_States/1', 's/1', 's');
L(n, 'Fx_pt/1', 'Points_L/1', 'Fx_pt');
L(n, 'Fx_pt/1', 'Points_R/1');
L(n, 'Points_L/1', 'Fx_L/1', 'Fx_pt_L');
L(n, 'Points_R/1', 'Fx_R/1', 'Fx_pt_R');
L(n, 'Fx_L/1', 'Fx_LR/1', 'Fx_L');
L(n, 'Fx_R/1', 'Fx_LR/2', 'Fx_R');
L(n, 'Fx_LR/1', 'Fx_trk/1', 'Fx_trk');
L(n, 'vB/1', 'vB_Delay/1', 'vB');
L(n, 'vB_Delay/1', 'vB_d/1', 'vB_d');
set_param(n, 'Description', ['structure_track_shear (T03, D39): isotropic 2-D shear states s_j = [s_x; s_y] of the ' ...
    '2 x Nc contact points (Integrator, IC 0, limits +/-1 per component), per-track sum of the point traction ' ...
    '(sprocket belt load F_x,i), belt speed delayed by one 1 ms sample (P.structure.track.vb_delay = 1) for the ' ...
    'shear law (D36 loop breaker).']);
end

% ===========================================================================================
%  structure_contact: kinematics, terrain / soil lookups, MF-A, MF-B
% ===========================================================================================
function local_contact(n)
% inputs: the bus 'body' of structure_body6dof (In Bus Element port 1, each element picked right in front of its
% consumer, layout spec 6.2), then s, vB_d, use_soil, soil_id (PORTS at the end)
IN(n, 's', 1); IN(n, 'vB_d', 2); IN(n, 'use_soil', 3); IN(n, 'soil_id', 4);
OUT(n, 'F_c', 1); OUT(n, 'M_c', 2); OUT(n, 'sDot', 3); OUT(n, 'Fx_pt', 4); OUT(n, 'cdiag', 5);
% ---- columns ----
COL(n, 'Xe_col'); COL(n, 'Vb_col'); COL(n, 'w_col'); COL(n, 'rcg_col');
L(n, 'body.Xe', 'Xe_col/1'); L(n, 'body.Vb', 'Vb_col/1'); L(n, 'body.omega', 'w_col/1');
L(n, 'body.r_cg', 'rcg_col/1');
% ---- lever arms r_j = Rc_R - r_cg 1' ----
TRANSP(n, 'C_nb');
C(n, 'Rc_R', 'P.structure.contact.Rc_R');
C(n, 'Ones', 'ones(1,2*P.structure.track.Nc)');
MM(n, 'Rcg_rep'); SUM(n, 'r_j', '+-');
L(n, 'body.DCM_be', 'C_nb/1');
L(n, 'rcg_col/1', 'Rcg_rep/1'); L(n, 'Ones/1', 'Rcg_rep/2', 'ones');
L(n, 'Rc_R/1', 'r_j/1', 'Rc_R'); L(n, 'Rcg_rep/1', 'r_j/2');
% ---- positions p_j = Xe 1' + C_nb r_j (NED) ----
MM(n, 'Xe_rep'); MM(n, 'Cnb_rj'); SUM(n, 'Pn', '++');
L(n, 'Xe_col/1', 'Xe_rep/1'); L(n, 'Ones/1', 'Xe_rep/2');
L(n, 'C_nb/1', 'Cnb_rj/1', 'C_nb'); L(n, 'r_j/1', 'Cnb_rj/2', 'r_j');
L(n, 'Xe_rep/1', 'Pn/1'); L(n, 'Cnb_rj/1', 'Pn/2');
% ---- point velocities v_j^b = v_b 1' + Omega^ r_j, v_j^n = C_nb v_j^b ----
DEMUX(n, 'pqr', '3');
A('simulink/Math Operations/Unary Minus', [n '/neg_p']);
A('simulink/Math Operations/Unary Minus', [n '/neg_q']);
A('simulink/Math Operations/Unary Minus', [n '/neg_r']);
C(n, 'Zero', '0');
A('aerolibutil/Create 3x3 Matrix', [n '/Omega_hat']);
L(n, 'w_col/1', 'pqr/1');
L(n, 'pqr/1', 'neg_p/1', 'p'); L(n, 'pqr/2', 'neg_q/1', 'q'); L(n, 'pqr/3', 'neg_r/1', 'r');
% [0 -r q; r 0 -p; -q p 0]
L(n, 'Zero/1', 'Omega_hat/1'); L(n, 'neg_r/1', 'Omega_hat/2'); L(n, 'pqr/2', 'Omega_hat/3');
L(n, 'pqr/3', 'Omega_hat/4'); L(n, 'Zero/1', 'Omega_hat/5'); L(n, 'neg_p/1', 'Omega_hat/6');
L(n, 'neg_q/1', 'Omega_hat/7'); L(n, 'pqr/1', 'Omega_hat/8'); L(n, 'Zero/1', 'Omega_hat/9');
MM(n, 'Vb_rep'); MM(n, 'Wxr'); SUM(n, 'Vb_pt', '++'); MM(n, 'Vn');
L(n, 'Vb_col/1', 'Vb_rep/1'); L(n, 'Ones/1', 'Vb_rep/2');
L(n, 'Omega_hat/1', 'Wxr/1', 'Omega_hat'); L(n, 'r_j/1', 'Wxr/2');
L(n, 'Vb_rep/1', 'Vb_pt/1'); L(n, 'Wxr/1', 'Vb_pt/2');
L(n, 'C_nb/1', 'Vn/1'); L(n, 'Vb_pt/1', 'Vn/2', 'Vb_pt');
% ---- terrain and soil lookups at (E_j, N_j) ----
SELROW(n, 'N_j', '1'); SELROW(n, 'E_j', '2');
L(n, 'Pn/1', 'N_j/1', 'Pn'); L(n, 'Pn/1', 'E_j/1');
luts = {'H', 'P.world.terrain.H'; 'dHdE', 'P.world.terrain.dHdE'; 'dHdN', 'P.world.terrain.dHdN'; ...
        'muPeak_map', 'P.world.soil.muPeak'; 'K_map', 'P.world.soil.K'; 'muLat_map', 'P.world.soil.muLat'; ...
        'fRoll_map', 'P.world.soil.fRoll'; 'kScale_map', 'P.world.soil.kScale'};
for k = 1:size(luts, 1)
    LUT2(n, luts{k, 1}, luts{k, 2});
    L(n, 'E_j/1', [luts{k, 1} '/1'], 'E_j'); L(n, 'N_j/1', [luts{k, 1} '/2'], 'N_j');
end
% soil override: all points use P.world.soilTable(soil_id,:) when env_ovr.use_soil
A('simulink/Math Operations/Matrix Concatenate', [n '/Soil_map'], 'NumInputs', '5', ...
    'Mode', 'Multidimensional array', 'ConcatenateDimension', '1');
for k = 4:8, L(n, [luts{k, 1} '/1'], sprintf('Soil_map/%d', k - 3), strrep(luts{k, 1}, '_map', '')); end
A('simulink/Signal Attributes/Data Type Conversion', [n '/soil_dbl'], 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Bias', [n '/soil_idx0'], 'Bias', '-1');
A('simulink/Lookup Tables/Direct Lookup Table (n-D)', [n '/Soil_table'], 'NumberOfTableDimensions', '2', ...
    'InputsSelectThisObjectFromTable', 'Column', 'Table', 'P.world.soilTable''', 'DiagnosticForOutOfRangeInput', 'None');
VEC(n, 'Soil_row'); SEL(n, 'Soil_props', '2:6', '6'); COL(n, 'Soil_col');
MM(n, 'Soil_rep');
SW(n, 'Soil_sw');
L(n, 'soil_id/1', 'soil_dbl/1', 'soil_id'); L(n, 'soil_dbl/1', 'soil_idx0/1');
L(n, 'soil_idx0/1', 'Soil_table/1', 'soil_index0');
L(n, 'Soil_table/1', 'Soil_row/1', 'soil_row'); L(n, 'Soil_row/1', 'Soil_props/1');
L(n, 'Soil_props/1', 'Soil_col/1', 'soil_props5'); L(n, 'Soil_col/1', 'Soil_rep/1');
L(n, 'Ones/1', 'Soil_rep/2');
L(n, 'Soil_rep/1', 'Soil_sw/1', 'soil_ovr'); L(n, 'use_soil/1', 'Soil_sw/2', 'use_soil');
L(n, 'Soil_map/1', 'Soil_sw/3', 'soil_map');
props = {'muP', 'K', 'muLat', 'fR', 'kS'};
for k = 1:5
    SELROW(n, props{k}, num2str(k));
    L(n, 'Soil_sw/1', [props{k} '/1'], 'soil_props');
end
% ---- MF-A trackContactNormal ----
% parameters of the sub-structs on Constant inputs (see MFB): the helper bodies still receive the
% sub-struct fields they use, rebuilt in the one-line block body (Nc from the 2*Nc point dimension)
C(n, 'prmC', '[P.structure.contact.kc; P.structure.contact.cc]');
MFB(n, 'MF_A_trackContactNormal', sprintf(['function [N_pt, N_trk, muP_trk, K_trk, vG_trk, pen_pt, sink_trk] = ' ...
    'trackContactNormal(Pn, Vn, Vb_pt, h, hE, hN, kS, muP, K, prmC)\n%%#codegen\n' ...
    '[N_pt, N_trk, muP_trk, K_trk, vG_trk, pen_pt, sink_trk] = structure_contact_normal(Pn, Vn, Vb_pt, h, hE, hN, kS, muP, K, ' ...
    'struct(''kc'', prmC(1), ''cc'', prmC(2)));\n']), ...
    'structure_contact_normal', 'MF-A: compliant normal contact of the 14 track points (structure_contact_normal.m); prmC = [kc; cc] of P.structure.contact');
mfa = 'MF_A_trackContactNormal';
L(n, 'Pn/1', [mfa '/1']); L(n, 'Vn/1', [mfa '/2'], 'Vn'); L(n, 'Vb_pt/1', [mfa '/3']);
L(n, 'H/1', [mfa '/4'], 'h'); L(n, 'dHdE/1', [mfa '/5'], 'hE'); L(n, 'dHdN/1', [mfa '/6'], 'hN');
L(n, 'kS/1', [mfa '/7'], 'kS'); L(n, 'muP/1', [mfa '/8'], 'muP'); L(n, 'K/1', [mfa '/9'], 'K');
L(n, 'prmC/1', [mfa '/10'], 'prmC');
% ---- MF-B trackPlanarForces ----
C(n, 'prmT', ['[P.structure.track.ell; P.structure.track.tau_d; P.structure.track.vEpsRR; ' ...
    'P.structure.track.B; P.structure.track.chi_rmin]']);
C(n, 'gS', 'P.structure.track.gS');
C(n, 'gG', 'P.structure.track.gG');
MFB(n, 'MF_B_trackPlanarForces', sprintf(['function [F_c, M_c, sDot, Fx_pt, Fy_trk, M_turn, P_diss, chi_eff, s_trk] = ' ...
    'trackPlanarForces(N_pt, Vb_pt, s, vB, muP, muLat, fR, K, Rcg, omega, hE, hN, DCM_be, prmT, gS, gG)\n%%#codegen\n' ...
    '[F_c, M_c, sDot, Fx_pt, Fy_trk, M_turn, P_diss, chi_eff, s_trk] = structure_planar_forces(N_pt, Vb_pt, s, vB, muP, muLat, fR, K, Rcg, omega, hE, hN, DCM_be, ' ...
    'struct(''ell'', prmT(1), ''tau_d'', prmT(2), ''vEpsRR'', prmT(3), ''B'', prmT(4), ''chi_rmin'', prmT(5), ''gS'', gS, ''gG'', gG));\n']), ...
    'structure_planar_forces', ['MF-B: isotropic 2-D shear law, rolling resistance, contact wrench (structure_planar_forces.m); ' ...
    'prmT = [ell; tau_d; vEpsRR; B; chi_rmin], gS / gG of P.structure.track']);
mfb = 'MF_B_trackPlanarForces';
L(n, [mfa '/1'], [mfb '/1'], 'N_pt'); L(n, 'Vb_pt/1', [mfb '/2']); L(n, 's/1', [mfb '/3'], 's');
L(n, 'vB_d/1', [mfb '/4'], 'vB_d'); L(n, 'muP/1', [mfb '/5']); L(n, 'muLat/1', [mfb '/6'], 'muLat');
L(n, 'fR/1', [mfb '/7'], 'fR'); L(n, 'K/1', [mfb '/8']); L(n, 'r_j/1', [mfb '/9']); L(n, 'w_col/1', [mfb '/10']);
L(n, 'dHdE/1', [mfb '/11']); L(n, 'dHdN/1', [mfb '/12']); L(n, 'body.DCM_be', [mfb '/13']);
L(n, 'prmT/1', [mfb '/14'], 'prmT'); L(n, 'gS/1', [mfb '/15'], 'gS'); L(n, 'gG/1', [mfb '/16'], 'gG');
L(n, [mfb '/3'], 'sDot/1', 'sDot');
L(n, [mfb '/4'], 'Fx_pt/1', 'Fx_pt');
% ---- MF-C1 gaugeWheelPoints, terrain at the wheels, MF-C2 gaugeWheelContact (mower gauge wheels, 2026-11-15) ----
% the rigid mower deck rides on two caster gauge wheels (P.structure.contact.gw, on = 1 with the mower fitted): their
% normal force (the MF-A law without the soil scale, no tangential force) joins the contact wrench F_c, M_c, and their
% power the contact dissipation P_diss (D40)
C(n, 'Rg_R', 'P.structure.contact.gw.R');
MFB(n, 'MF_C1_gaugeWheelPoints', sprintf(['function [Pn_g, Vn_g, r_g] = gaugeWheelPoints(Xe, DCM_be, Vb, omega, rcg, Rg)\n' ...
    '%%#codegen\n[Pn_g, Vn_g, r_g] = structure_gauge_points(Xe, DCM_be, Vb, omega, rcg, Rg);\n']), ...
    'structure_gauge_points', ['MF-C1: positions / velocities of the mower gauge-wheel contact points ' ...
    '(structure_gauge_points.m); Rg = P.structure.contact.gw.R']);
mc1 = 'MF_C1_gaugeWheelPoints';
% the body elements are picked right in front of MF-C1 (layout spec 6.2), not branched off the track kinematics
L(n, 'body.Xe', [mc1 '/1']); L(n, 'body.DCM_be', [mc1 '/2']); L(n, 'body.Vb', [mc1 '/3']); L(n, 'body.omega', [mc1 '/4']);
L(n, 'body.r_cg', [mc1 '/5']); L(n, 'Rg_R/1', [mc1 '/6'], 'Rg_R');
SELROW(n, 'N_g', '1'); SELROW(n, 'E_g', '2');
L(n, [mc1 '/1'], 'N_g/1', 'Pn_g'); L(n, [mc1 '/1'], 'E_g/1');
lutg = {'H_g', 'P.world.terrain.H'; 'dHdE_g', 'P.world.terrain.dHdE'; 'dHdN_g', 'P.world.terrain.dHdN'};
for k = 1:size(lutg, 1)
    LUT2(n, lutg{k, 1}, lutg{k, 2});
    L(n, 'E_g/1', [lutg{k, 1} '/1'], 'E_g'); L(n, 'N_g/1', [lutg{k, 1} '/2'], 'N_g');
end
C(n, 'prmG', '[P.structure.contact.gw.on; P.structure.contact.gw.k; P.structure.contact.gw.c]');
MFB(n, 'MF_C2_gaugeWheelContact', sprintf(['function [F_g, M_g, P_g, N_g] = gaugeWheelContact(Pn_g, Vn_g, r_g, h, hE, hN, ' ...
    'DCM_be, prmG)\n%%#codegen\n[F_g, M_g, P_g, N_g] = structure_gauge_contact(Pn_g, Vn_g, r_g, h, hE, hN, DCM_be, prmG);\n']), ...
    'structure_gauge_contact', ['MF-C2: compliant normal contact of the mower gauge wheels (structure_gauge_contact.m); ' ...
    'prmG = [on; k; c] of P.structure.contact.gw']);
mc2 = 'MF_C2_gaugeWheelContact';
L(n, [mc1 '/1'], [mc2 '/1']); L(n, [mc1 '/2'], [mc2 '/2'], 'Vn_g'); L(n, [mc1 '/3'], [mc2 '/3'], 'r_g');
L(n, 'H_g/1', [mc2 '/4'], 'h_g'); L(n, 'dHdE_g/1', [mc2 '/5'], 'hE_g'); L(n, 'dHdN_g/1', [mc2 '/6'], 'hN_g');
L(n, 'body.DCM_be', [mc2 '/7']); L(n, 'prmG/1', [mc2 '/8'], 'prmG');
A('simulink/Sinks/Terminator', [n '/Term_N_gw']);
L(n, [mc2 '/4'], 'Term_N_gw/1', 'N_gw');
% ---- contact wrench = tracks (MF-B) + gauge wheels (MF-C2) ----
SUM(n, 'F_c_sum', '++'); SUM(n, 'M_c_sum', '++');
L(n, [mfb '/1'], 'F_c_sum/1', 'F_c_trk'); L(n, [mc2 '/1'], 'F_c_sum/2', 'F_gw'); L(n, 'F_c_sum/1', 'F_c/1', 'F_c');
L(n, [mfb '/2'], 'M_c_sum/1', 'M_c_trk'); L(n, [mc2 '/2'], 'M_c_sum/2', 'M_gw'); L(n, 'M_c_sum/1', 'M_c/1', 'M_c');
% ---- diagnostics (virtual bus) ----
A('simulink/Signal Routing/Bus Creator', [n '/Contact_Diag'], 'Inputs', '11');
L(n, [mfa '/1'], 'Contact_Diag/1');
L(n, [mfa '/6'], 'Contact_Diag/2', 'pen_pt');
L(n, [mfa '/2'], 'Contact_Diag/3', 'N_trk');
L(n, [mfa '/7'], 'Contact_Diag/4', 'sink_trk');
L(n, [mfa '/3'], 'Contact_Diag/5', 'muP_trk');
L(n, [mfa '/5'], 'Contact_Diag/6', 'vG_trk');
L(n, [mfb '/5'], 'Contact_Diag/7', 'Fy_trk');
L(n, [mfb '/6'], 'Contact_Diag/8', 'M_turn');
SUM(n, 'P_diss_sum', '++');
L(n, [mfb '/7'], 'P_diss_sum/1', 'P_diss_trk'); L(n, [mc2 '/3'], 'P_diss_sum/2', 'P_gw');
L(n, 'P_diss_sum/1', 'Contact_Diag/9', 'P_diss');
L(n, [mfb '/8'], 'Contact_Diag/10', 'chi_eff');
L(n, [mfb '/9'], 'Contact_Diag/11', 's_trk');
A('simulink/Sinks/Terminator', [n '/Term_K_trk']);
L(n, [mfa '/4'], 'Term_K_trk/1', 'K_trk');
L(n, 'Contact_Diag/1', 'cdiag/1', 'cdiag');
PORTS(n, {'body', 's', 'vB_d', 'use_soil', 'soil_id'}, 'Inport');
set_param(n, 'Description', ['structure_contact (T03): contact-point kinematics r_j, p_j (NED), v_j (FRD, NED) with ' ...
    'matrix Product blocks, 2-D Lookup Tables on P.world (Evenly spaced points, Clip), soil override by ' ...
    'Direct Lookup Table on P.world.soilTable + Switch (env_ovr.use_soil), MF-A trackContactNormal and ' ...
    'MF-B trackPlanarForces (docs/specs/structure.md 2.3-2.6, 3.3, 4), MF-C1 gaugeWheelPoints and MF-C2 ' ...
    'gaugeWheelContact (the two caster gauge wheels of the rigid mower deck, P.structure.contact.gw, on with the ' ...
    'mower: normal force into F_c / M_c, power into P_diss). Inputs: the bus body of ' ...
    'structure_body6dof (In Bus Element port; Xe, DCM_be, Vb, omega, r_cg picked), s, vB_d, use_soil, soil_id.']);
root = char(currentProject().RootFolder);
for hf = {'structure_gauge_points', 'structure_gauge_contact'}
    local_addfile(fullfile(root, 'subsystems', 'structure', [hf{1} '.m']));
end
end

% ===========================================================================================
%  structure_body6dof: 6DOF rigid body
% ===========================================================================================
function local_body6dof(n)
% outputs: the virtual bus 'body' (Out Bus Element port 1, see the end)
IN(n, 'F_c', 1); IN(n, 'M_c', 2); IN(n, 'wrench', 3); IN(n, 'dmdt', 4); IN(n, 'F_ext_B', 5); IN(n, 'M_ext_B', 6);
eom = [n '/EOM_6DOF'];
A('aerolib6dof2/Simple Variable Mass 6DOF (Euler Angles)', eom, 'units', 'Metric (MKS)', 'mtype', 'Simple Variable', ...
    'xme_0', 'P.structure.init.xme0', 'Vm_0', 'P.structure.init.Vb0', 'eul_0', 'P.structure.init.eul0', 'pm_0', '[0 0 0]', ...
    'mass_0', 'P.structure.mp.m0', 'mass_e', 'P.structure.mp.me', 'mass_f', 'P.structure.mp.mf', ...
    'inertia_e', 'P.structure.mp.Je', 'inertia_f', 'P.structure.mp.Jf', 'vre_flag', 'off', 'mass_flag', 'on', ...
    'zeroCrossing', 'off');
A('aerolib6dof2/6DOF Acceleration', [n '/Acceleration'], 'units', 'Metric (MKS)', 'mtype', 'Simple Variable', 'vre_flag', 'off');
A('aerolib6dof2/6DOF Angular Acceleration', [n '/Angular_Acceleration'], 'units', 'Metric (MKS)', 'mtype', 'Simple Variable');
% ---- implement wrench [F (3) FRD; M (3) FRD; r (3) FRD w.r.t. R] ----
DEMUX(n, 'Wrench', '[3 3 3]');
L(n, 'wrench/1', 'Wrench/1', 'wrench');
% ---- r_cg(m) = (me rcg_e + (m - me) r_tank)/m ----
A('simulink/Math Operations/Bias', [n '/m_minus_me'], 'Bias', '-P.structure.mp.me');
K(n, 'Tank_moment', 'P.structure.mp.r_tank(:)');
C(n, 'Empty_moment', 'P.structure.mp.me*P.structure.mp.rcg_e(:)');
SUM(n, 'First_moment', '++');
PROD(n, 'rcg_of_m', '*/');
L(n, [eom(numel(n)+2:end) '/8'], 'm_minus_me/1', 'mass');
L(n, 'm_minus_me/1', 'Tank_moment/1');
L(n, 'Empty_moment/1', 'First_moment/1'); L(n, 'Tank_moment/1', 'First_moment/2');
L(n, 'First_moment/1', 'rcg_of_m/1'); L(n, 'EOM_6DOF/8', 'rcg_of_m/2');
% ---- gravity m DCM_be [0;0;g] ----
C(n, 'g_ned', '[0;0;P.frames.g]');
MM(n, 'g_body'); PROD(n, 'F_grav', '**');
L(n, 'EOM_6DOF/4', 'g_body/1', 'DCM_be'); L(n, 'g_ned/1', 'g_body/2');
L(n, 'EOM_6DOF/8', 'F_grav/1'); L(n, 'g_body/1', 'F_grav/2');
% ---- external wrench (FLU -> FRD) ----
K(n, 'F_ext_FRD', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
K(n, 'M_ext_FRD', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
L(n, 'F_ext_B/1', 'F_ext_FRD/1', 'F_ext_B'); L(n, 'M_ext_B/1', 'M_ext_FRD/1', 'M_ext_B');
% ---- implement moment (r_impl - r_cg) x F_impl + M_impl ----
SUM(n, 'r_impl_cg', '+-'); XPROD(n, 'rxF_impl'); SUM(n, 'M_impl_tot', '++');
L(n, 'Wrench/3', 'r_impl_cg/1', 'r_impl'); L(n, 'rcg_of_m/1', 'r_impl_cg/2', 'r_cg');
L(n, 'r_impl_cg/1', 'rxF_impl/1'); L(n, 'Wrench/1', 'rxF_impl/2', 'F_impl');
L(n, 'rxF_impl/1', 'M_impl_tot/1'); L(n, 'Wrench/2', 'M_impl_tot/2', 'M_impl');
% ---- wrench sums ----
SUM(n, 'F_total', '++++'); SUM(n, 'M_total', '+++');
L(n, 'F_c/1', 'F_total/1', 'F_c'); L(n, 'F_grav/1', 'F_total/2', 'F_grav');
L(n, 'Wrench/1', 'F_total/3'); L(n, 'F_ext_FRD/1', 'F_total/4', 'F_ext');
L(n, 'M_c/1', 'M_total/1', 'M_c'); L(n, 'M_impl_tot/1', 'M_total/2'); L(n, 'M_ext_FRD/1', 'M_total/3', 'M_ext');
L(n, 'F_total/1', 'EOM_6DOF/1', 'F_b'); L(n, 'M_total/1', 'EOM_6DOF/2', 'M_b'); L(n, 'dmdt/1', 'EOM_6DOF/3', 'dmdt');
% ---- accelerations (outputs only, never fed back into the force path) ----
% 6DOF Acceleration: Vb, omega, fuel, mass, Forces, mdot -> Abb, Abe
L(n, 'EOM_6DOF/5', 'Acceleration/1'); L(n, 'EOM_6DOF/6', 'Acceleration/2'); L(n, 'EOM_6DOF/7', 'Acceleration/3', 'fuel');
L(n, 'EOM_6DOF/8', 'Acceleration/4'); L(n, 'F_total/1', 'Acceleration/5'); L(n, 'dmdt/1', 'Acceleration/6');
A('simulink/Sinks/Terminator', [n '/Term_Abb']);
L(n, 'Acceleration/1', 'Term_Abb/1', 'Abb');
% 6DOF Angular Acceleration: omega, I, dIdtConst, Moment, mdot -> omegaDot
L(n, 'EOM_6DOF/6', 'Angular_Acceleration/1'); L(n, 'EOM_6DOF/9', 'Angular_Acceleration/2', 'J');
L(n, 'EOM_6DOF/10', 'Angular_Acceleration/3', 'dJdm'); L(n, 'M_total/1', 'Angular_Acceleration/4');
L(n, 'dmdt/1', 'Angular_Acceleration/5');
% ---- implement wrench power on the chassis: F_impl . (Vb + w x (r_impl - r_cg)) + M_impl . w ----
XPROD(n, 'w_x_r'); SUM(n, 'v_app', '++'); DOT(n, 'P_force'); DOT(n, 'P_moment'); SUM(n, 'P_wrench', '++');
L(n, 'EOM_6DOF/6', 'w_x_r/1'); L(n, 'r_impl_cg/1', 'w_x_r/2');
L(n, 'EOM_6DOF/5', 'v_app/1'); L(n, 'w_x_r/1', 'v_app/2');
L(n, 'Wrench/1', 'P_force/1'); L(n, 'v_app/1', 'P_force/2', 'v_app');
L(n, 'Wrench/2', 'P_moment/1'); L(n, 'EOM_6DOF/6', 'P_moment/2');
L(n, 'P_force/1', 'P_wrench/1'); L(n, 'P_moment/1', 'P_wrench/2');
% ---- outputs: the virtual bus 'body' (layout spec 6.3: structure_contact takes 5 of its elements, structure_truth 8) ----
OB(n, 'body', 'Xe', 'EOM_6DOF/2'); OB(n, 'body', 'Ve', 'EOM_6DOF/1'); OB(n, 'body', 'eul', 'EOM_6DOF/3');
OB(n, 'body', 'DCM_be', 'EOM_6DOF/4'); OB(n, 'body', 'Vb', 'EOM_6DOF/5'); OB(n, 'body', 'omega', 'EOM_6DOF/6');
OB(n, 'body', 'mass', 'EOM_6DOF/8'); OB(n, 'body', 'J', 'EOM_6DOF/9'); OB(n, 'body', 'Abe', 'Acceleration/2');
OB(n, 'body', 'omegaDot', 'Angular_Acceleration/1'); OB(n, 'body', 'r_cg', 'rcg_of_m/1');
OB(n, 'body', 'P_impl', 'P_wrench/1');
set_param(n, 'Description', ['structure_body6dof (T03): Simple Variable Mass 6DOF (Euler Angles) in flat-Earth NED / ' ...
    'body FRD at the CG (mass_flag on: outputs Ve Xe eul DCM_be Vb omega fuel mass I dI/dm), 6DOF Acceleration ' ...
    '(Abe) and 6DOF Angular Acceleration, gravity m DCM_be [0;0;g], wrench sum (contact + gravity + implement + ' ...
    'external), r_cg(m) of the depleting sprayer tank, implement wrench power on the chassis (D40). Output: the ' ...
    'virtual bus body (Xe Ve eul DCM_be Vb omega mass J Abe omegaDot r_cg P_impl, an Out Bus Element port).']);
end

% ===========================================================================================
%  structure_truth: Bus_TruthState (ENU/FLU at R)
% ===========================================================================================
function local_truth(n)
% inputs: the bus 'body' of structure_body6dof (In Bus Element port 1, each element picked right in front of its
% consumer, layout spec 6.2), then vB, vG (PORTS at the end)
IN(n, 'vB', 1); IN(n, 'vG', 2);
OUT(n, 'truth', 1, 'OutDataTypeStr', 'Bus: Bus_TruthState');
OUT(n, 'pose2D', 2); OUT(n, 'speed_fwd', 3); OUT(n, 'slip', 4); OUT(n, 'slope', 5); OUT(n, 'outOfMap', 6);
COL(n, 'Xe_col'); COL(n, 'Vb_col'); COL(n, 'w_col'); COL(n, 'wd_col'); COL(n, 'Abe_col'); COL(n, 'rcg_col');
L(n, 'body.Xe', 'Xe_col/1'); L(n, 'body.Vb', 'Vb_col/1'); L(n, 'body.omega', 'w_col/1');
L(n, 'body.omegaDot', 'wd_col/1'); L(n, 'body.Abe', 'Abe_col/1'); L(n, 'body.r_cg', 'rcg_col/1');
% ---- point R: r_R = -r_cg (FRD w.r.t. CG) ----
K(n, 'r_R', '-1');
TRANSP(n, 'C_nb');
MM(n, 'Cnb_rR'); SUM(n, 'p_R_ned', '++');
K(n, 'pos_W', 'P.frames.R_NED_ENU''', 'Matrix(K*u)');
L(n, 'rcg_col/1', 'r_R/1');
L(n, 'body.DCM_be', 'C_nb/1');
L(n, 'C_nb/1', 'Cnb_rR/1', 'C_nb'); L(n, 'r_R/1', 'Cnb_rR/2', 'r_R');
L(n, 'Xe_col/1', 'p_R_ned/1'); L(n, 'Cnb_rR/1', 'p_R_ned/2');
L(n, 'p_R_ned/1', 'pos_W/1', 'p_R_ned');
% ---- R_WB = R_NED_ENU' C_nb R_FRD_FLU ----
C(n, 'R_ENU_NED', 'P.frames.R_NED_ENU''');
C(n, 'R_FRD_FLU', 'P.frames.R_FRD_FLU');
MM(n, 'R_WB', 3);
L(n, 'R_ENU_NED/1', 'R_WB/1'); L(n, 'C_nb/1', 'R_WB/2'); L(n, 'R_FRD_FLU/1', 'R_WB/3');
TRANSP(n, 'R_BW');
L(n, 'R_WB/1', 'R_BW/1', 'R_WB');
% ---- velocity of R ----
XPROD(n, 'w_x_rR'); SUM(n, 'v_R_frd', '++');
K(n, 'vel_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
MM(n, 'vel_W');
L(n, 'w_col/1', 'w_x_rR/1'); L(n, 'r_R/1', 'w_x_rR/2');
L(n, 'Vb_col/1', 'v_R_frd/1'); L(n, 'w_x_rR/1', 'v_R_frd/2');
L(n, 'v_R_frd/1', 'vel_B/1', 'v_R_frd');
L(n, 'R_WB/1', 'vel_W/1'); L(n, 'vel_B/1', 'vel_W/2', 'vel_B');
% ---- acceleration of R: a = Abe + wdot x r + w x (w x r) ----
XPROD(n, 'wd_x_rR'); XPROD(n, 'w_x_w_x_rR'); SUM(n, 'a_R_frd', '+++');
K(n, 'acc_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
MM(n, 'acc_W');
L(n, 'wd_col/1', 'wd_x_rR/1'); L(n, 'r_R/1', 'wd_x_rR/2');
L(n, 'w_col/1', 'w_x_w_x_rR/1'); L(n, 'w_x_rR/1', 'w_x_w_x_rR/2');
L(n, 'Abe_col/1', 'a_R_frd/1'); L(n, 'wd_x_rR/1', 'a_R_frd/2'); L(n, 'w_x_w_x_rR/1', 'a_R_frd/3');
L(n, 'a_R_frd/1', 'acc_B/1', 'a_R_frd');
L(n, 'R_WB/1', 'acc_W/1'); L(n, 'acc_B/1', 'acc_W/2', 'acc_B');
% ---- specific force (physical, + up at rest) ----
C(n, 'g_enu', '[0;0;P.frames.g]');
MM(n, 'g_B'); SUM(n, 'specForce_B', '++');
L(n, 'R_BW/1', 'g_B/1', 'R_BW'); L(n, 'g_enu/1', 'g_B/2');
L(n, 'acc_B/1', 'specForce_B/1'); L(n, 'g_B/1', 'specForce_B/2', 'g_B');
% ---- quaternion W->B (frame rotation) from R_WB' ----
A('aerolibtransform2/Direction Cosine Matrix  to Quaternions', [n '/q_WB']);
L(n, 'R_BW/1', 'q_WB/1');
% ---- Euler angles ZYX [psi theta phi] from R_WB ----
VEC(n, 'R_vec');                                  % column-major: R11 R21 R31 R12 R22 R32 R13 R23 R33
L(n, 'R_WB/1', 'R_vec/1');
DEMUX(n, 'R_el', '9');
L(n, 'R_vec/1', 'R_el/1', 'R_WB_vec');
unused = {4, 'R12'; 5, 'R22'; 7, 'R13'; 8, 'R23'};
for k = 1:size(unused, 1)
    A('simulink/Sinks/Terminator', sprintf('%s/Term_R%d', n, unused{k, 1}));
    L(n, sprintf('R_el/%d', unused{k, 1}), sprintf('Term_R%d/1', unused{k, 1}), unused{k, 2});
end
TRIG(n, 'psi', 'atan2'); TRIG(n, 'asin_R31', 'asin'); TRIG(n, 'phi', 'atan2');
A('simulink/Discontinuities/Saturation', [n '/R31_clamp'], 'UpperLimit', '1', 'LowerLimit', '-1');
A('simulink/Math Operations/Unary Minus', [n '/theta']);
L(n, 'R_el/2', 'psi/1', 'R21'); L(n, 'R_el/1', 'psi/2', 'R11');
L(n, 'R_el/3', 'R31_clamp/1', 'R31'); L(n, 'R31_clamp/1', 'asin_R31/1'); L(n, 'asin_R31/1', 'theta/1');
L(n, 'R_el/6', 'phi/1', 'R32'); L(n, 'R_el/9', 'phi/2', 'R33');
MUX(n, 'eul', 3);
L(n, 'psi/1', 'eul/1', 'psi'); L(n, 'theta/1', 'eul/2', 'theta'); L(n, 'phi/1', 'eul/3', 'phi');
% ---- body rates and CG offset in FLU ----
K(n, 'omega_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
K(n, 'alpha_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
K(n, 'r_cg_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
L(n, 'w_col/1', 'omega_B/1'); L(n, 'wd_col/1', 'alpha_B/1'); L(n, 'rcg_col/1', 'r_cg_B/1');
% ---- geodetic position of R (Flat Earth to LLA, href input port, D62) ----
C(n, 'href', '-P.world.lla0(3)');   % Flat Earth to LLA: h = -p_z - href (href along the flat-Earth down axis)
A('aerolibtransform2/Flat Earth to LLA', [n '/Flat_Earth_to_LLA'], 'units', 'Metric (MKS)', 'ptype', 'Earth (WGS84)', ...
    'LL0', 'P.world.lla0(1:2)', 'psi', '0', 'refPosPort', 'off');
MUX(n, 'lla', 2);
L(n, 'p_R_ned/1', 'Flat_Earth_to_LLA/1'); L(n, 'href/1', 'Flat_Earth_to_LLA/2', 'href');
L(n, 'Flat_Earth_to_LLA/1', 'lla/1', 'mu_l'); L(n, 'Flat_Earth_to_LLA/2', 'lla/2', 'h');
% ---- planar pose, speed ----
DEMUX(n, 'pos_xyz', '3'); DEMUX(n, 'vel_B_xyz', '3');
L(n, 'pos_W/1', 'pos_xyz/1', 'pos_W'); L(n, 'vel_B/1', 'vel_B_xyz/1');
A('simulink/Sinks/Terminator', [n '/Term_z']); A('simulink/Sinks/Terminator', [n '/Term_vy']);
A('simulink/Sinks/Terminator', [n '/Term_vz']);
L(n, 'pos_xyz/3', 'Term_z/1', 'z_U'); L(n, 'vel_B_xyz/2', 'Term_vy/1', 'v_y'); L(n, 'vel_B_xyz/3', 'Term_vz/1', 'v_z');
MUX(n, 'pose2D_mux', 3);
L(n, 'pos_xyz/1', 'pose2D_mux/1', 'x_E'); L(n, 'pos_xyz/2', 'pose2D_mux/2', 'y_N'); L(n, 'psi/1', 'pose2D_mux/3');
% ---- terrain under R, grades ----
LUT2(n, 'H_R', 'P.world.terrain.H'); LUT2(n, 'hE_R', 'P.world.terrain.dHdE'); LUT2(n, 'hN_R', 'P.world.terrain.dHdN');
for k = {'H_R', 'hE_R', 'hN_R'}
    L(n, 'pos_xyz/1', [k{1} '/1']); L(n, 'pos_xyz/2', [k{1} '/2']);
end
TRIG(n, 'cos_psi', 'cos'); TRIG(n, 'sin_psi', 'sin');
L(n, 'psi/1', 'cos_psi/1'); L(n, 'psi/1', 'sin_psi/1');
PROD(n, 'hE_c', '**'); PROD(n, 'hN_s', '**'); PROD(n, 'hE_s', '**'); PROD(n, 'hN_c', '**');
L(n, 'hE_R/1', 'hE_c/1', 'hE'); L(n, 'cos_psi/1', 'hE_c/2', 'cos_psi');
L(n, 'hN_R/1', 'hN_s/1', 'hN'); L(n, 'sin_psi/1', 'hN_s/2', 'sin_psi');
L(n, 'hE_R/1', 'hE_s/1'); L(n, 'sin_psi/1', 'hE_s/2');
L(n, 'hN_R/1', 'hN_c/1'); L(n, 'cos_psi/1', 'hN_c/2');
SUM(n, 'tan_long', '++'); SUM(n, 'tan_lat', '-+');
L(n, 'hE_c/1', 'tan_long/1'); L(n, 'hN_s/1', 'tan_long/2');
L(n, 'hE_s/1', 'tan_lat/1'); L(n, 'hN_c/1', 'tan_lat/2');
TRIG(n, 'grade_long', 'atan'); TRIG(n, 'grade_lat', 'atan');
L(n, 'tan_long/1', 'grade_long/1'); L(n, 'tan_lat/1', 'grade_lat/1');
MUX(n, 'slope_mux', 2);
L(n, 'grade_long/1', 'slope_mux/1', 'grade_long'); L(n, 'grade_lat/1', 'slope_mux/2', 'grade_lat');
% ---- track slip ----
SUM(n, 'v_slip', '+-');
A('simulink/Math Operations/Abs', [n '/abs_vB'], 'ZeroCross', 'off');
C(n, 'v_eps', 'P.structure.track.v_eps_slip');
A('simulink/Math Operations/MinMax', [n '/vB_floor'], 'Function', 'max', 'Inputs', '2', 'ZeroCross', 'off');
PROD(n, 'slip_LR', '*/');
L(n, 'vB/1', 'v_slip/1', 'vBelt_LR'); L(n, 'vG/1', 'v_slip/2', 'vGround_LR');
L(n, 'vB/1', 'abs_vB/1'); L(n, 'abs_vB/1', 'vB_floor/1'); L(n, 'v_eps/1', 'vB_floor/2');
L(n, 'v_slip/1', 'slip_LR/1'); L(n, 'vB_floor/1', 'slip_LR/2');
% ---- out of the DEM ----
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/x_lo'], 'relop', '<', 'const', 'P.world.terrain.xE(1)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/x_hi'], 'relop', '>', 'const', 'P.world.terrain.xE(end)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/y_lo'], 'relop', '<', 'const', 'P.world.terrain.yN(1)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/y_hi'], 'relop', '>', 'const', 'P.world.terrain.yN(end)');
A('simulink/Logic and Bit Operations/Logical Operator', [n '/Out_of_map'], 'Operator', 'OR', 'Inputs', '4');
L(n, 'pos_xyz/1', 'x_lo/1'); L(n, 'pos_xyz/1', 'x_hi/1'); L(n, 'pos_xyz/2', 'y_lo/1'); L(n, 'pos_xyz/2', 'y_hi/1');
L(n, 'x_lo/1', 'Out_of_map/1'); L(n, 'x_hi/1', 'Out_of_map/2'); L(n, 'y_lo/1', 'Out_of_map/3'); L(n, 'y_hi/1', 'Out_of_map/4');
% ---- Bus_TruthState ----
A('simulink/Sources/Clock', [n '/Clock']);
BC(n, 'TruthState', 'Bus_TruthState', 23);
src = {'Clock/1', 't'; 'pos_W/1', 'pos_W'; 'vel_W/1', 'vel_W'; 'acc_W/1', 'acc_W'; 'q_WB/1', 'q_WB'; ...
    'R_WB/1', ''; 'eul/1', 'eul'; 'vel_B/1', ''; 'omega_B/1', 'omega_B'; 'alpha_B/1', 'alpha_B'; ...
    'acc_B/1', ''; 'specForce_B/1', 'specForce_B'; 'r_cg_B/1', 'r_cg_B'; 'lla/1', 'lla'; ...
    'pose2D_mux/1', 'pose2D'; 'vel_B_xyz/1', 'speed_fwd'; 'body.mass', 'mass'; 'H_R/1', 'terrainH'; ...
    'grade_long/1', ''; 'grade_lat/1', ''; 'vB/1', ''; 'vG/1', ''; 'slip_LR/1', 'slip_LR'};
for k = 1:size(src, 1)
    L(n, src{k, 1}, sprintf('TruthState/%d', k), src{k, 2});
end
L(n, 'TruthState/1', 'truth/1', 'truth');
L(n, 'pose2D_mux/1', 'pose2D/1'); L(n, 'vel_B_xyz/1', 'speed_fwd/1');
L(n, 'slip_LR/1', 'slip/1'); L(n, 'slope_mux/1', 'slope/1', 'slope'); L(n, 'Out_of_map/1', 'outOfMap/1', 'outOfMap');
PORTS(n, {'body', 'vB', 'vG'}, 'Inport');
set_param(n, 'Description', ['structure_truth (T03): NED/FRD -> ENU/FLU exactly per ICD 1: R_WB = R_NED_ENU'' DCM_be'' ' ...
    'R_FRD_FLU, q_WB = DCM-to-quaternion of R_WB'', eul = [atan2(R21,R11); -asin(R31); atan2(R32,R33)], position, ' ...
    'velocity and acceleration of R (r_R = -r_cg), specForce_B = acc_B + R_WB''[0;0;g], lla of R by Flat Earth to ' ...
    'LLA (href port = P.world.lla0(3), D62), pose2D, grades, track slip, out-of-map flag. Inputs: the bus body of ' ...
    'structure_body6dof (In Bus Element port; Xe, DCM_be, Vb, omega, omegaDot, Abe, mass, r_cg picked), vB, vG.']);
end

% ===========================================================================================
%  structure_hifi
% ===========================================================================================
function local_hifi(n)
CPORT(n, 'Spr_L', 1, 'Left'); CPORT(n, 'Spr_R', 2, 'Left');
IN(n, 'wrench', 1); IN(n, 'dmdt', 2); IN(n, 'E_impl', 3);
IN(n, 'env_dist', 4, 'OutDataTypeStr', 'Bus: Bus_EnvDisturbance');
IN(n, 'env_ovr', 5, 'OutDataTypeStr', 'Bus: Bus_EnvOverride');
OUT(n, 'truth', 1, 'OutDataTypeStr', 'Bus: Bus_TruthState');
OUT(n, 'tracks', 2, 'OutDataTypeStr', 'Bus: Bus_TrackState');
OUT(n, 'diag', 3, 'OutDataTypeStr', 'Bus: Bus_StructureDiag');
OUT(n, 'pose2D', 4); OUT(n, 'speed_fwd', 5);
SSR = 'simulink/Ports & Subsystems/Subsystem Reference';
A(SSR, [n '/TrackL'], 'ReferencedSubsystem', 'structure_track_side');
A(SSR, [n '/TrackR'], 'ReferencedSubsystem', 'structure_track_side');
try, set_param([n '/TrackR'], 'side', '2 right'); catch, end %#ok<CTCH> mask optional
A(SSR, [n '/Shear'], 'ReferencedSubsystem', 'structure_track_shear');
A(SSR, [n '/Contact'], 'ReferencedSubsystem', 'structure_contact');
A(SSR, [n '/Body'], 'ReferencedSubsystem', 'structure_body6dof');
A(SSR, [n '/Truth'], 'ReferencedSubsystem', 'structure_truth');
% physical
PL(n, CPH(n, 'Spr_L'), shaddad_bu('pport', [n '/TrackL'], 'Spr'));
PL(n, CPH(n, 'Spr_R'), shaddad_bu('pport', [n '/TrackR'], 'Spr'));
% inputs
BSEL(n, 'Dist', 'F_ext_B,M_ext_B'); BSEL(n, 'Ovr', 'use_soil,soil_id');
L(n, 'env_dist/1', 'Dist/1', 'env_dist'); L(n, 'env_ovr/1', 'Ovr/1', 'env_ovr');
% body: in F_c M_c wrench dmdt F_ext M_ext; out the bus body (Xe Ve eul DCM_be Vb omega mass J Abe omegaDot r_cg
% P_impl; no block here picks Ve or eul)
L(n, 'Contact/1', 'Body/1', 'F_c'); L(n, 'Contact/2', 'Body/2', 'M_c');
L(n, 'wrench/1', 'Body/3', 'wrench'); L(n, 'dmdt/1', 'Body/4', 'dmdt');
L(n, 'Dist/1', 'Body/5'); L(n, 'Dist/2', 'Body/6');
% contact: in body (Xe DCM_be Vb omega r_cg picked inside) s vB_d use_soil soil_id
L(n, 'Body/1', 'Contact/1', 'body');
L(n, 'Shear/1', 'Contact/2', 's'); L(n, 'Shear/3', 'Contact/3', 'vB_d');
L(n, 'Ovr/1', 'Contact/4'); L(n, 'Ovr/2', 'Contact/5');
% shear: in sDot Fx_pt vB; out s Fx_trk vB_d
L(n, 'Contact/3', 'Shear/1', 'sDot'); L(n, 'Contact/4', 'Shear/2', 'Fx_pt');
DEMUX(n, 'Fx_LR', '2');
L(n, 'Shear/2', 'Fx_LR/1', 'Fx_trk');
L(n, 'Fx_LR/1', 'TrackL/1', 'Fx_L'); L(n, 'Fx_LR/2', 'TrackR/1', 'Fx_R');
MUX(n, 'vB', 2); MUX(n, 'sprAngle', 2); MUX(n, 'sprSpeed', 2); MUX(n, 'sprTorque', 2); SUM(n, 'P_fric', '++');
L(n, 'TrackL/1', 'vB/1', 'v_b_L'); L(n, 'TrackR/1', 'vB/2', 'v_b_R');
L(n, 'TrackL/2', 'sprAngle/1', 'sprAngle_L'); L(n, 'TrackR/2', 'sprAngle/2', 'sprAngle_R');
L(n, 'TrackL/3', 'sprSpeed/1', 'sprSpeed_L'); L(n, 'TrackR/3', 'sprSpeed/2', 'sprSpeed_R');
L(n, 'TrackL/4', 'sprTorque/1', 'sprTorque_L'); L(n, 'TrackR/4', 'sprTorque/2', 'sprTorque_R');
L(n, 'TrackL/5', 'P_fric/1', 'P_fric_L'); L(n, 'TrackR/5', 'P_fric/2', 'P_fric_R');
L(n, 'vB/1', 'Shear/3', 'vB');
% contact diagnostics: the bus cdiag (Contact/5), picked by a Bus Selector CD_<element> in front of each consumer
% truth: in body (Xe DCM_be Vb omega omegaDot Abe mass r_cg picked inside) vB vG
L(n, 'Body/1', 'Truth/1'); L(n, 'vB/1', 'Truth/2');
PICK(n, 'CD_vG_trk', 'Contact/5', {'vG_trk'}, {'Truth/3'}, 'cdiag');
L(n, 'Truth/1', 'truth/1', 'truth'); L(n, 'Truth/2', 'pose2D/1', 'pose2D'); L(n, 'Truth/3', 'speed_fwd/1', 'speed_fwd');
% ---- Bus_TrackState ----
K(n, 'Fy_FLU', '-1'); K(n, 'Mturn_FLU', '-1');
SUM(n, 'v_slip', '+-'); PROD(n, 'Pslip', '**');
PICK(n, 'CD_Fy_trk', 'Contact/5', {'Fy_trk'}, {'Fy_FLU/1'}, 'cdiag');
PICK(n, 'CD_M_turn', 'Contact/5', {'M_turn'}, {'Mturn_FLU/1'}, 'cdiag');
L(n, 'vB/1', 'v_slip/1'); PICK(n, 'CD_vG_trk_2', 'Contact/5', {'vG_trk'}, {'v_slip/2'}, 'cdiag');
L(n, 'Shear/2', 'Pslip/1'); L(n, 'v_slip/1', 'Pslip/2', 'v_slip');
BC(n, 'TrackState', 'Bus_TrackState', 14);
SIGC(n, 'v_ground'); SIGC(n, 's_shear'); SIGC(n, 'Fn'); SIGC(n, 'sinkage'); SIGC(n, 'muPeak');
cds = {'CD_vG_trk_3', 'vG_trk', 'v_ground'; 'CD_s_trk', 's_trk', 's_shear'; 'CD_N_trk', 'N_trk', 'Fn'; ...
       'CD_sink_trk', 'sink_trk', 'sinkage'; 'CD_muP_trk', 'muP_trk', 'muPeak'};
for k = 1:size(cds, 1), PICK(n, cds{k, 1}, 'Contact/5', cds(k, 2), {[cds{k, 3} '/1']}, 'cdiag'); end
tsrc = {'vB/1', 'v_belt'; 'v_ground/1', 'v_ground'; 'Truth/4', 'slip'; 's_shear/1', 's_shear'; 'Shear/2', 'Fx'; ...
        'Fy_FLU/1', 'Fy'; 'Fn/1', 'Fn'; 'sprAngle/1', 'sprAngle'; 'sprSpeed/1', 'sprSpeed'; 'sprTorque/1', 'sprTorque'; ...
        'sinkage/1', 'sinkage'; 'muPeak/1', 'muPeak'; 'Mturn_FLU/1', 'Mturn'; 'Pslip/1', 'Pslip'};
for k = 1:size(tsrc, 1)
    L(n, tsrc{k, 1}, sprintf('TrackState/%d', k), tsrc{k, 2});
end
L(n, 'TrackState/1', 'tracks/1', 'tracks');
% ---- energy bookkeeping (D40) ----
DOT(n, 'v2'); PROD(n, 'mv2', '**'); MM(n, 'Jw'); DOT(n, 'wJw'); DOT(n, 'vb2');
K(n, 'half_mv2', '0.5'); K(n, 'half_wJw', '0.5'); K(n, 'E_belt', 'P.structure.energy.k_belt');
SEL(n, 'D_cg', '3', '3'); K(n, 'gh', '-P.frames.g'); PROD(n, 'E_pot', '**');
SUM(n, 'E_mech', '++++');
COL(n, 'w_col');
PICK(n, 'Sel_Vb', 'Body/1', {'Vb'}, {{'v2/1', 'v2/2'}}, 'body');
PICK(n, 'Sel_mass', 'Body/1', {'mass'}, {'mv2/1'}, 'body'); L(n, 'v2/1', 'mv2/2', 'v2'); L(n, 'mv2/1', 'half_mv2/1');
PICK(n, 'Sel_omega', 'Body/1', {'omega'}, {'w_col/1'}, 'body'); PICK(n, 'Sel_J', 'Body/1', {'J'}, {'Jw/1'}, 'body');
L(n, 'w_col/1', 'Jw/2');
VEC(n, 'Jw_vec'); L(n, 'Jw/1', 'Jw_vec/1', 'Jw');
PICK(n, 'Sel_omega_2', 'Body/1', {'omega'}, {'wJw/1'}, 'body'); L(n, 'Jw_vec/1', 'wJw/2'); L(n, 'wJw/1', 'half_wJw/1');
L(n, 'vB/1', 'vb2/1'); L(n, 'vB/1', 'vb2/2'); L(n, 'vb2/1', 'E_belt/1', 'vb2');
PICK(n, 'Sel_Xe', 'Body/1', {'Xe'}, {'D_cg/1'}, 'body'); L(n, 'D_cg/1', 'gh/1', 'D_cg');
PICK(n, 'Sel_mass_2', 'Body/1', {'mass'}, {'E_pot/1'}, 'body'); L(n, 'gh/1', 'E_pot/2');
L(n, 'half_mv2/1', 'E_mech/1', 'E_trans'); L(n, 'half_wJw/1', 'E_mech/2', 'E_rot');
L(n, 'E_belt/1', 'E_mech/3', 'E_belt'); L(n, 'E_pot/1', 'E_mech/4', 'E_pot');
SUM(n, 'P_diss_tot', '++'); SUM(n, 'P_diss_E', '+-'); DTI(n, 'E_diss_J');
PICK(n, 'CD_P_diss', 'Contact/5', {'P_diss'}, {'P_diss_tot/1'}, 'cdiag'); L(n, 'P_fric/1', 'P_diss_tot/2', 'P_fric');
L(n, 'P_diss_tot/1', 'P_diss_E/1', 'P_diss'); PICK(n, 'Sel_P_impl', 'Body/1', {'P_impl'}, {'P_diss_E/2'}, 'body');
L(n, 'P_diss_E/1', 'E_diss_J/1');
% ---- Bus_StructureDiag ----
SEL(n, 'sX', '1:2*P.structure.track.Nc', '4*P.structure.track.Nc');
SEL(n, 'sY', '2*P.structure.track.Nc+1:4*P.structure.track.Nc', '4*P.structure.track.Nc');
L(n, 'Shear/1', 'sX/1'); L(n, 'Shear/1', 'sY/1');
DEMUX(n, 'Slope', '2');
L(n, 'Truth/5', 'Slope/1', 'slope');
BC(n, 'StructureDiag', 'Bus_StructureDiag', 12);
PICK(n, 'CD_diag', 'Contact/5', {'N_pt', 'pen_pt', 'chi_eff'}, ...
    {'StructureDiag/1', 'StructureDiag/2', 'StructureDiag/11'}, 'cdiag');
dsrc = {3, 'sX/1', 'sX'; 4, 'sY/1', 'sY'; 5, 'Slope/1', 'slopeLong'; 6, 'Slope/2', 'slopeLat'; 7, 'P_diss_tot/1', ''; ...
        8, 'E_mech/1', 'E_mech'; 9, 'E_diss_J/1', 'E_diss_J'; 10, 'E_impl/1', 'E_impl_shaft_J'; 12, 'Truth/6', 'outOfMap'};
for k = 1:size(dsrc, 1)
    L(n, dsrc{k, 2}, sprintf('StructureDiag/%d', dsrc{k, 1}), dsrc{k, 3});
end
L(n, 'StructureDiag/1', 'diag/1', 'diag');
set_param(n, 'Description', ['structure_hifi (T03, VC_STRUCT_FID == 1): TrackL/TrackR (structure_track_side), Shear ' ...
    '(28 2-D shear states), Contact (kinematics, lookups, MF-A, MF-B), Body (6DOF), Truth; assembles Bus_TrackState ' ...
    'and Bus_StructureDiag (E_mech = translational + rotational + belt/track inertia + potential; E_diss_J = ' ...
    'integral of P_diss minus the implement wrench power on the chassis, D40). Body sends the bus body (picked ' ...
    'inside Contact and Truth, and by a Bus Selector Sel_<element> in front of each energy block here); the bus ' ...
    'cdiag of Contact is picked by a Bus Selector CD_<element> in front of each consumer.']);
end

% ===========================================================================================
%  structure_lofi
% ===========================================================================================
function local_lofi(n)
c = LIB();
CPORT(n, 'Spr_L', 1, 'Left'); CPORT(n, 'Spr_R', 2, 'Left');
IN(n, 'wrench', 1); IN(n, 'dmdt', 2); IN(n, 'E_impl', 3);
IN(n, 'env_dist', 4, 'OutDataTypeStr', 'Bus: Bus_EnvDisturbance');
IN(n, 'env_ovr', 5, 'OutDataTypeStr', 'Bus: Bus_EnvOverride');
OUT(n, 'truth', 1, 'OutDataTypeStr', 'Bus: Bus_TruthState');
OUT(n, 'tracks', 2, 'OutDataTypeStr', 'Bus: Bus_TrackState');
OUT(n, 'diag', 3, 'OutDataTypeStr', 'Bus: Bus_StructureDiag');
OUT(n, 'pose2D', 4); OUT(n, 'speed_fwd', 5);
% ---- per-side Simscape sprocket load networks ----
for s = {'L', 'R'}
    sd = s{1};
    A(c.TQS, [n '/Torque_Sensor_' sd]);
    A(c.J, [n '/J_side_' sd], 'inertia', 'P.structure.lofi.J_side', 'num_ports', 'foundation.enum.numPorts2.one', ...
        'w_specify', 'on', 'w_priority', 'High', 'w', 'P.structure.init.v0/P.structure.r_sprocket');
    A(c.FRIC, [n '/Friction_' sd], 'brkwy_trq', 'P.structure.track.T_brk', 'brkwy_vel', 'P.structure.track.w_brk', ...
        'Col_trq', 'P.structure.track.T_col', 'visc_coef', 'P.structure.track.c_visc');
    A(c.TQS, [n '/Fric_Sensor_' sd]);
    A(c.TSRC, [n '/Load_Torque_' sd]);
    A(c.RMS, [n '/Motion_Sensor_' sd], 'reference', 'foundation.enum.MeasurementReference.difference', ...
        'velocity_port', 'true', 'position_port', 'true', 'acceleration_port', 'false', 'wrap_angle', 'false');
    A(c.RREF, [n '/Ref_' sd]);
    SPSB(n, ['SPS_load_' sd], 'N*m', 'zero');
    PSSB(n, ['PSS_w_' sd], 'rad/s'); PSSB(n, ['PSS_th_' sd], 'rad');
    PSSB(n, ['PSS_T_' sd], 'N*m'); PSSB(n, ['PSS_Tf_' sd], 'N*m');
    tq = PH([n '/Torque_Sensor_' sd]); J = PH([n '/J_side_' sd]); fr = PH([n '/Friction_' sd]);
    ft = PH([n '/Fric_Sensor_' sd]); ts = PH([n '/Load_Torque_' sd]); rm = PH([n '/Motion_Sensor_' sd]);
    rr = PH([n '/Ref_' sd]); sp = PH([n '/SPS_load_' sd]);
    PL(n, CPH(n, ['Spr_' sd]), tq.LConn(1));
    nd = tq.RConn(1);
    PL(n, nd, J.LConn(1)); PL(n, nd, fr.LConn(1)); PL(n, fr.RConn(1), ft.LConn(1)); PL(n, ft.RConn(1), rr.LConn(1));
    PL(n, nd, ts.LConn(1)); PL(n, ts.RConn(2), rr.LConn(1)); PL(n, sp.RConn(1), ts.RConn(1));
    PL(n, nd, rm.LConn(1)); PL(n, rm.RConn(1), rr.LConn(1));
    % motion sensor outputs: W, A
    pw = PH([n '/PSS_w_' sd]); pt = PH([n '/PSS_th_' sd]);
    PL(n, rm.RConn(2), pw.LConn(1)); PL(n, rm.RConn(3), pt.LConn(1));
    pT = PH([n '/PSS_T_' sd]); pTf = PH([n '/PSS_Tf_' sd]);
    PL(n, tq.RConn(2), pT.LConn(1)); PL(n, ft.RConn(2), pTf.LConn(1));
    % continuous kinematic copy (2 ms lag) and the 1 ms loop-breaker copy (D36) of the sprocket speed
    A('simulink/Continuous/Transfer Fcn', [n '/w_filt_' sd], 'Numerator', '1', 'Denominator', '[P.sim.ssc.tau_filt 1]');
    UD(n, ['w_delay_' sd], 'P.structure.init.v0/P.structure.r_sprocket');
    L(n, ['PSS_w_' sd '/1'], ['w_filt_' sd '/1'], ['w_' sd]);
    L(n, ['PSS_w_' sd '/1'], ['w_delay_' sd '/1']);
    % sprocket acceleration = derivative of the filtered speed (w - w_f)/tau (continuous)
    SUM(n, ['a_err_' sd], '+-'); K(n, ['alpha_' sd], '1/P.sim.ssc.tau_filt');
    L(n, ['PSS_w_' sd '/1'], ['a_err_' sd '/1']); L(n, ['w_filt_' sd '/1'], ['a_err_' sd '/2']);
    L(n, ['a_err_' sd '/1'], ['alpha_' sd '/1']);
    PROD(n, ['Pf_' sd], '**');
    L(n, ['PSS_Tf_' sd '/1'], ['Pf_' sd '/1'], ['T_fric_' sd]); L(n, ['PSS_w_' sd '/1'], ['Pf_' sd '/2']);
end
% ---- inputs (a Bus Selector in front of each consumer, layout spec 6.2; M_ext_B is not used here) ----
SEL(n, 'Fext_x', '1', '3'); PICK(n, 'Dist', 'env_dist/1', {'F_ext_B'}, {'Fext_x/1'}, 'env_dist');
DEMUX(n, 'Wrench', '[3 3 3]'); L(n, 'wrench/1', 'Wrench/1', 'wrench');
A('simulink/Sinks/Terminator', [n '/Term_Mimpl']); A('simulink/Sinks/Terminator', [n '/Term_rimpl']);
L(n, 'Wrench/2', 'Term_Mimpl/1', 'M_impl'); L(n, 'Wrench/3', 'Term_rimpl/1', 'r_impl');
SEL(n, 'Fimpl_x', '1', '3'); L(n, 'Wrench/1', 'Fimpl_x/1', 'F_impl');
% ---- mass ----
A('simulink/Continuous/Integrator', [n '/Mass'], 'InitialCondition', 'P.structure.mp.m0');
L(n, 'dmdt/1', 'Mass/1', 'dmdt');
K(n, 'Weight', 'P.frames.g'); L(n, 'Mass/1', 'Weight/1', 'mass');
% ---- kinematics (DDKM on the filtered sprocket speeds) ----
A('robotmobilelib/Differential Drive Kinematic Model', [n '/DDKM'], 'VehicleInputs', 'Wheel Speeds', ...
    'WheelRadius', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)', 'TrackWidth', 'P.structure.lofi.chi*P.structure.geom.B', ...
    'WheelSpeedRange', '[-inf inf]', 'InitialState', 'P.structure.init.pose2D0(:)');
try
    set_param([n '/DDKM'], 'SimulateUsing', 'Code generation');
catch tmp_T03_err
    warning('build_structure:ddkm', 'DDKM SimulateUsing: %s', tmp_T03_err.message);
end
L(n, 'w_filt_L/1', 'DDKM/1', 'w_L'); L(n, 'w_filt_R/1', 'DDKM/2', 'w_R');
A('simulink/Sinks/Terminator', [n '/Term_stateDot']); L(n, 'DDKM/2', 'Term_stateDot/1', 'stateDot');
DEMUX(n, 'State', '3'); L(n, 'DDKM/1', 'State/1', 'state');
TRIG(n, 'sin_th', 'sin'); TRIG(n, 'cos_th', 'cos'); TRIG(n, 'psi', 'atan2');
L(n, 'State/3', 'sin_th/1', 'theta'); L(n, 'State/3', 'cos_th/1');
L(n, 'sin_th/1', 'psi/1'); L(n, 'cos_th/1', 'psi/2');
% v, r from the filtered speeds; r for the loads from the delayed speeds
K(n, 'kv', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)/2');
K(n, 'kr', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)/(P.structure.lofi.chi*P.structure.geom.B)');
K(n, 'kr_d', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)/(P.structure.lofi.chi*P.structure.geom.B)');
SUM(n, 'w_sum', '++'); SUM(n, 'w_diff', '-+'); SUM(n, 'wd_diff', '-+');
L(n, 'w_filt_L/1', 'w_sum/1'); L(n, 'w_filt_R/1', 'w_sum/2');
L(n, 'w_filt_L/1', 'w_diff/1'); L(n, 'w_filt_R/1', 'w_diff/2');
L(n, 'w_delay_L/1', 'wd_diff/1', 'w_d_L'); L(n, 'w_delay_R/1', 'wd_diff/2', 'w_d_R');
L(n, 'w_sum/1', 'kv/1'); L(n, 'w_diff/1', 'kr/1'); L(n, 'wd_diff/1', 'kr_d/1');
% accelerations from the sprocket accelerations
SUM(n, 'a_sum', '++'); SUM(n, 'a_diff', '-+');
K(n, 'kvd', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)/2');
K(n, 'krd', 'P.structure.r_sprocket*(1-P.structure.lofi.i_nom)/(P.structure.lofi.chi*P.structure.geom.B)');
L(n, 'alpha_L/1', 'a_sum/1', 'alpha_L'); L(n, 'alpha_R/1', 'a_sum/2', 'alpha_R');
L(n, 'alpha_L/1', 'a_diff/1'); L(n, 'alpha_R/1', 'a_diff/2');
L(n, 'a_sum/1', 'kvd/1'); L(n, 'a_diff/1', 'krd/1');
% ---- terrain at R, soil (override) ----
LUT2(n, 'H', 'P.world.terrain.H'); LUT2(n, 'hE', 'P.world.terrain.dHdE'); LUT2(n, 'hN', 'P.world.terrain.dHdN');
LUT2(n, 'muPeak_map', 'P.world.soil.muPeak'); LUT2(n, 'muLat_map', 'P.world.soil.muLat');
LUT2(n, 'fRoll_map', 'P.world.soil.fRoll'); LUT2(n, 'kScale_map', 'P.world.soil.kScale');
lu = {'H', 'hE', 'hN', 'muPeak_map', 'muLat_map', 'fRoll_map', 'kScale_map'};
for k = 1:numel(lu)
    L(n, 'State/1', [lu{k} '/1']); L(n, 'State/2', [lu{k} '/2']);
end
MUX(n, 'Soil_map', 4);
L(n, 'muPeak_map/1', 'Soil_map/1', 'muPeak'); L(n, 'muLat_map/1', 'Soil_map/2', 'muLat');
L(n, 'fRoll_map/1', 'Soil_map/3', 'fRoll'); L(n, 'kScale_map/1', 'Soil_map/4', 'kScale');
A('simulink/Signal Attributes/Data Type Conversion', [n '/soil_dbl'], 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Bias', [n '/soil_idx0'], 'Bias', '-1');
A('simulink/Lookup Tables/Direct Lookup Table (n-D)', [n '/Soil_table'], 'NumberOfTableDimensions', '2', ...
    'InputsSelectThisObjectFromTable', 'Column', 'Table', 'P.world.soilTable''', 'DiagnosticForOutOfRangeInput', 'None');
VEC(n, 'Soil_row'); SEL(n, 'Soil_props', '[2 4 5 6]', '6');
SW(n, 'Soil_sw');
PICK(n, 'Ovr_soil_id', 'env_ovr/1', {'soil_id'}, {'soil_dbl/1'}, 'env_ovr');
L(n, 'soil_dbl/1', 'soil_idx0/1'); L(n, 'soil_idx0/1', 'Soil_table/1');
L(n, 'Soil_table/1', 'Soil_row/1'); L(n, 'Soil_row/1', 'Soil_props/1'); L(n, 'Soil_props/1', 'Soil_sw/1', 'soil_ovr');
PICK(n, 'Ovr_use_soil', 'env_ovr/1', {'use_soil'}, {'Soil_sw/2'}, 'env_ovr'); L(n, 'Soil_map/1', 'Soil_sw/3', 'soil_map');
DEMUX(n, 'Soil', '4'); L(n, 'Soil_sw/1', 'Soil/1', 'soil');
% grades along the heading
TRIG(n, 'cos_psi', 'cos'); TRIG(n, 'sin_psi', 'sin');
L(n, 'psi/1', 'cos_psi/1', 'psi'); L(n, 'psi/1', 'sin_psi/1');
PROD(n, 'hE_c', '**'); PROD(n, 'hN_s', '**'); PROD(n, 'hE_s', '**'); PROD(n, 'hN_c', '**');
L(n, 'hE/1', 'hE_c/1', 'hE'); L(n, 'cos_psi/1', 'hE_c/2'); L(n, 'hN/1', 'hN_s/1', 'hN'); L(n, 'sin_psi/1', 'hN_s/2');
L(n, 'hE/1', 'hE_s/1'); L(n, 'sin_psi/1', 'hE_s/2'); L(n, 'hN/1', 'hN_c/1'); L(n, 'cos_psi/1', 'hN_c/2');
SUM(n, 'tan_long', '++'); SUM(n, 'tan_lat', '-+');
L(n, 'hE_c/1', 'tan_long/1'); L(n, 'hN_s/1', 'tan_long/2'); L(n, 'hE_s/1', 'tan_lat/1'); L(n, 'hN_c/1', 'tan_lat/2');
TRIG(n, 'grade_long', 'atan'); TRIG(n, 'grade_lat', 'atan');
L(n, 'tan_long/1', 'grade_long/1'); L(n, 'tan_lat/1', 'grade_lat/1');
TRIG(n, 'cos_gl', 'cos'); TRIG(n, 'sin_gl', 'sin'); TRIG(n, 'sin_gt', 'sin');
L(n, 'grade_long/1', 'cos_gl/1', 'grade_long'); L(n, 'grade_long/1', 'sin_gl/1'); L(n, 'grade_lat/1', 'sin_gt/1', 'grade_lat');
% ---- track normal loads N_i = W/2 cos(gl) -/+ W h_cg sin(gt)/B ----
PROD(n, 'Wcos', '**'); PROD(n, 'Wsin_t', '**');
K(n, 'half', '0.5'); K(n, 'k_lat', 'P.structure.lofi.h_cg/P.structure.geom.B');
SUM(n, 'N_L', '+-'); SUM(n, 'N_R', '++');
L(n, 'Weight/1', 'Wcos/1', 'W'); L(n, 'cos_gl/1', 'Wcos/2');
L(n, 'Weight/1', 'Wsin_t/1'); L(n, 'sin_gt/1', 'Wsin_t/2');
L(n, 'Wcos/1', 'half/1'); L(n, 'Wsin_t/1', 'k_lat/1');
L(n, 'half/1', 'N_L/1'); L(n, 'k_lat/1', 'N_L/2'); L(n, 'half/1', 'N_R/1'); L(n, 'k_lat/1', 'N_R/2');
MUX(n, 'N_LR', 2); L(n, 'N_L/1', 'N_LR/1', 'N_L'); L(n, 'N_R/1', 'N_LR/2', 'N_R');
% ---- quasi-static sprocket loads (spec 2.10), all from the 1 ms delayed speeds ----
MUX(n, 'w_d', 2); L(n, 'w_delay_L/1', 'w_d/1'); L(n, 'w_delay_R/1', 'w_d/2');
K(n, 'v_b_d', 'P.structure.r_sprocket');
K(n, 'inv_vEps', '1/P.structure.track.vEpsRR'); TRIG(n, 'tanh_vb', 'tanh');
L(n, 'w_d/1', 'v_b_d/1', 'w_d'); L(n, 'v_b_d/1', 'inv_vEps/1', 'v_b_d'); L(n, 'inv_vEps/1', 'tanh_vb/1');
PROD(n, 'fN', '**'); PROD(n, 'F_roll', '**');
L(n, 'Soil/3', 'fN/1', 'fRoll'); L(n, 'N_LR/1', 'fN/2', 'N_LR');
L(n, 'fN/1', 'F_roll/1'); L(n, 'tanh_vb/1', 'F_roll/2');
PROD(n, 'Wsin_l', '**'); K(n, 'F_grade', '0.5');
L(n, 'Weight/1', 'Wsin_l/1'); L(n, 'sin_gl/1', 'Wsin_l/2'); L(n, 'Wsin_l/1', 'F_grade/1');
% turning resistance mu_lat W L/(4B) tanh(r/w_eps) (+ right / - left)
K(n, 'inv_wEps', '1/P.structure.lofi.w_eps'); TRIG(n, 'tanh_r', 'tanh');
L(n, 'kr_d/1', 'inv_wEps/1', 'r_d'); L(n, 'inv_wEps/1', 'tanh_r/1');
PROD(n, 'muW', '**'); K(n, 'k_turn', 'P.structure.geom.L/(4*P.structure.geom.B)'); PROD(n, 'F_turn', '**');
L(n, 'Soil/2', 'muW/1', 'muLat'); L(n, 'Weight/1', 'muW/2'); L(n, 'muW/1', 'k_turn/1');
L(n, 'k_turn/1', 'F_turn/1'); L(n, 'tanh_r/1', 'F_turn/2');
K(n, 'turn_sign', '[-1;1]');
L(n, 'F_turn/1', 'turn_sign/1', 'F_turn');
SUM(n, 'F_push', '++'); K(n, 'half_push', '0.5');
L(n, 'Fimpl_x/1', 'F_push/1'); L(n, 'Fext_x/1', 'F_push/2'); L(n, 'F_push/1', 'half_push/1', 'F_push');
SUM(n, 'F_load', '+++-');
L(n, 'F_roll/1', 'F_load/1', 'F_roll'); L(n, 'F_grade/1', 'F_load/2', 'F_grade'); L(n, 'turn_sign/1', 'F_load/3', 'F_turn_LR');
L(n, 'half_push/1', 'F_load/4', 'F_push_side');
K(n, 'T_load', '-P.structure.r_sprocket');
L(n, 'F_load/1', 'T_load/1', 'F_load');
DEMUX(n, 'T_load_LR', '2'); L(n, 'T_load/1', 'T_load_LR/1', 'minus_T_load');
L(n, 'T_load_LR/1', 'SPS_load_L/1', 'minus_T_load_L'); L(n, 'T_load_LR/2', 'SPS_load_R/1', 'minus_T_load_R');
% ---- truth (quasi-static attitude from the DEM) ----
A('simulink/Math Operations/Unary Minus', [n '/theta']);
L(n, 'grade_long/1', 'theta/1');
MUX(n, 'eul', 3); L(n, 'psi/1', 'eul/1'); L(n, 'theta/1', 'eul/2', 'theta'); L(n, 'grade_lat/1', 'eul/3');
A('aerolibtransform2/Rotation Angles to Direction Cosine Matrix', [n '/DCM_WB'], 'rotationOrder', 'ZYX');
L(n, 'eul/1', 'DCM_WB/1', 'eul');
TRANSP(n, 'R_WB'); L(n, 'DCM_WB/1', 'R_WB/1', 'R_BW');
A('aerolibtransform2/Direction Cosine Matrix  to Quaternions', [n '/q_WB']); L(n, 'DCM_WB/1', 'q_WB/1');
MUX(n, 'pos_W', 3); L(n, 'State/1', 'pos_W/1', 'x_E'); L(n, 'State/2', 'pos_W/2', 'y_N'); L(n, 'H/1', 'pos_W/3', 'h');
K(n, 'Zero', '0'); L(n, 'kv/1', 'Zero/1');          % continuous zero (no constant in the bus Muxes)
MUX(n, 'vel_B', 3); L(n, 'kv/1', 'vel_B/1', 'v'); L(n, 'Zero/1', 'vel_B/2'); L(n, 'Zero/1', 'vel_B/3');
MM(n, 'vel_W'); L(n, 'R_WB/1', 'vel_W/1', 'R_WB'); L(n, 'vel_B/1', 'vel_W/2', 'vel_B');
PROD(n, 'v_r', '**'); L(n, 'kv/1', 'v_r/1'); L(n, 'kr/1', 'v_r/2', 'r');
MUX(n, 'acc_B', 3); L(n, 'kvd/1', 'acc_B/1', 'v_dot'); L(n, 'v_r/1', 'acc_B/2', 'v_r'); L(n, 'Zero/1', 'acc_B/3');
MM(n, 'acc_W'); L(n, 'R_WB/1', 'acc_W/1'); L(n, 'acc_B/1', 'acc_W/2', 'acc_B');
C(n, 'g_enu', '[0;0;P.frames.g]'); MM(n, 'g_B'); SUM(n, 'specForce_B', '++');
L(n, 'DCM_WB/1', 'g_B/1'); L(n, 'g_enu/1', 'g_B/2'); L(n, 'acc_B/1', 'specForce_B/1'); L(n, 'g_B/1', 'specForce_B/2', 'g_B');
MUX(n, 'omega_B', 3); L(n, 'Zero/1', 'omega_B/1'); L(n, 'Zero/1', 'omega_B/2'); L(n, 'kr/1', 'omega_B/3');
MUX(n, 'alpha_B', 3); L(n, 'Zero/1', 'alpha_B/1'); L(n, 'Zero/1', 'alpha_B/2'); L(n, 'krd/1', 'alpha_B/3', 'r_dot');
C(n, 'r_cg_B', 'P.structure.mp.rcg0_FLU(:)');
C(n, 'href', '-P.world.lla0(3)');   % Flat Earth to LLA: h = -p_z - href (href along the flat-Earth down axis)
A('simulink/Math Operations/Unary Minus', [n '/D_R']); L(n, 'H/1', 'D_R/1');
MUX(n, 'p_ned', 3); L(n, 'State/2', 'p_ned/1'); L(n, 'State/1', 'p_ned/2'); L(n, 'D_R/1', 'p_ned/3', 'D_R');
A('aerolibtransform2/Flat Earth to LLA', [n '/Flat_Earth_to_LLA'], 'units', 'Metric (MKS)', 'ptype', 'Earth (WGS84)', ...
    'LL0', 'P.world.lla0(1:2)', 'psi', '0', 'refPosPort', 'off');
L(n, 'p_ned/1', 'Flat_Earth_to_LLA/1', 'p_R_ned'); L(n, 'href/1', 'Flat_Earth_to_LLA/2', 'href');
MUX(n, 'lla', 2); L(n, 'Flat_Earth_to_LLA/1', 'lla/1', 'mu_l'); L(n, 'Flat_Earth_to_LLA/2', 'lla/2', 'h_geo');
MUX(n, 'pose2D_mux', 3); L(n, 'State/1', 'pose2D_mux/1'); L(n, 'State/2', 'pose2D_mux/2'); L(n, 'psi/1', 'pose2D_mux/3');
% belt / ground speeds and slip
MUX(n, 'w_raw', 2); L(n, 'PSS_w_L/1', 'w_raw/1'); L(n, 'PSS_w_R/1', 'w_raw/2');
K(n, 'vBelt', 'P.structure.r_sprocket'); L(n, 'w_raw/1', 'vBelt/1', 'w_spr');
K(n, 'half_B', '[-1;1]*P.structure.geom.B/2'); L(n, 'kr/1', 'half_B/1');
SUM(n, 'vGround', '++'); L(n, 'kv/1', 'vGround/1'); L(n, 'half_B/1', 'vGround/2', 'rB2');
SUM(n, 'v_slip', '+-'); L(n, 'vBelt/1', 'v_slip/1', 'vBelt'); L(n, 'vGround/1', 'v_slip/2', 'vGround');
A('simulink/Math Operations/Abs', [n '/abs_vB'], 'ZeroCross', 'off');
C(n, 'v_eps', 'P.structure.track.v_eps_slip');
A('simulink/Math Operations/MinMax', [n '/vB_floor'], 'Function', 'max', 'Inputs', '2', 'ZeroCross', 'off');
PROD(n, 'slip_LR', '*/');
L(n, 'vBelt/1', 'abs_vB/1'); L(n, 'abs_vB/1', 'vB_floor/1'); L(n, 'v_eps/1', 'vB_floor/2');
L(n, 'v_slip/1', 'slip_LR/1', 'v_slip'); L(n, 'vB_floor/1', 'slip_LR/2');
% out of map
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/x_lo'], 'relop', '<', 'const', 'P.world.terrain.xE(1)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/x_hi'], 'relop', '>', 'const', 'P.world.terrain.xE(end)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/y_lo'], 'relop', '<', 'const', 'P.world.terrain.yN(1)');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/y_hi'], 'relop', '>', 'const', 'P.world.terrain.yN(end)');
A('simulink/Logic and Bit Operations/Logical Operator', [n '/Out_of_map'], 'Operator', 'OR', 'Inputs', '4');
L(n, 'State/1', 'x_lo/1'); L(n, 'State/1', 'x_hi/1'); L(n, 'State/2', 'y_lo/1'); L(n, 'State/2', 'y_hi/1');
L(n, 'x_lo/1', 'Out_of_map/1'); L(n, 'x_hi/1', 'Out_of_map/2'); L(n, 'y_lo/1', 'Out_of_map/3'); L(n, 'y_hi/1', 'Out_of_map/4');
A('simulink/Sources/Clock', [n '/Clock']);
BC(n, 'TruthState', 'Bus_TruthState', 23);
src = {'Clock/1', 't'; 'pos_W/1', 'pos_W'; 'vel_W/1', 'vel_W'; 'acc_W/1', 'acc_W'; 'q_WB/1', 'q_WB'; ...
    'R_WB/1', ''; 'eul/1', ''; 'vel_B/1', ''; 'omega_B/1', 'omega_B'; 'alpha_B/1', 'alpha_B'; ...
    'acc_B/1', ''; 'specForce_B/1', 'specForce_B'; 'r_cg_B/1', 'r_cg_B'; 'lla/1', 'lla'; ...
    'pose2D_mux/1', 'pose2D'; 'kv/1', 'speed_fwd'; 'Mass/1', 'mass'; 'H/1', 'terrainH'; ...
    'grade_long/1', 'grade_long'; 'grade_lat/1', 'grade_lat'; 'vBelt/1', 'vBelt_LR'; 'vGround/1', 'vGround_LR'; ...
    'slip_LR/1', 'slip_LR'};
for k = 1:size(src, 1)
    L(n, src{k, 1}, sprintf('TruthState/%d', k), src{k, 2});
end
L(n, 'TruthState/1', 'truth/1', 'truth');
L(n, 'pose2D_mux/1', 'pose2D/1', 'pose2D'); L(n, 'kv/1', 'speed_fwd/1');
% ---- Bus_TrackState ----
MUX(n, 'sprAngle', 2); L(n, 'PSS_th_L/1', 'sprAngle/1', 'th_L'); L(n, 'PSS_th_R/1', 'sprAngle/2', 'th_R');
MUX(n, 'sprTorque', 2); L(n, 'PSS_T_L/1', 'sprTorque/1', 'T_spr_L'); L(n, 'PSS_T_R/1', 'sprTorque/2', 'T_spr_R');
C(n, 'Zeros2a', 'zeros(2,1)'); C(n, 'Zeros2b', 'zeros(2,1)');
PROD(n, 'kN', '**'); C(n, 'kNc', 'P.structure.track.Nc*P.structure.contact.kc');
PROD(n, 'sink', '*/');
L(n, 'Soil/4', 'kN/1', 'kScale'); L(n, 'kNc/1', 'kN/2');
L(n, 'N_LR/1', 'sink/1'); L(n, 'kN/1', 'sink/2', 'k_track');
C(n, 'Ones2', 'ones(2,1)'); PROD(n, 'muPeak2', '**');
L(n, 'Soil/1', 'muPeak2/1', 'muPeak'); L(n, 'Ones2/1', 'muPeak2/2');
K(n, 'Mres', '-P.structure.geom.L/4'); PROD(n, 'Mturn', '**');
L(n, 'muW/1', 'Mres/1', 'muLat_W'); L(n, 'Mres/1', 'Mturn/1'); L(n, 'tanh_r/1', 'Mturn/2');
PROD(n, 'Pslip', '**'); L(n, 'F_load/1', 'Pslip/1'); L(n, 'v_slip/1', 'Pslip/2');
BC(n, 'TrackState', 'Bus_TrackState', 14);
SIGC(n, 'v_belt', 'vBelt/1'); SIGC(n, 'v_ground', 'vGround/1'); SIGC(n, 'slip', 'slip_LR/1');
tsrc = {'v_belt/1', 'v_ground/1', 'slip/1', 'Zeros2a/1', 'F_load/1', 'Zeros2b/1', 'N_LR/1', 'sprAngle/1', 'w_raw/1', ...
        'sprTorque/1', 'sink/1', 'muPeak2/1', 'Mturn/1', 'Pslip/1'};
tnm = {'v_belt', 'v_ground', 'slip', 's_shear', 'Fx', 'Fy', 'Fn', 'sprAngle', 'sprSpeed', 'sprTorque', 'sinkage', ...
       'muPeak', 'Mturn', 'Pslip'};
for k = 1:numel(tsrc), L(n, tsrc{k}, sprintf('TrackState/%d', k), tnm{k}); end
L(n, 'TrackState/1', 'tracks/1', 'tracks');
% ---- Bus_StructureDiag (point loads spread evenly, energy of the quasi-static model) ----
C(n, 'Spread', 'kron(eye(2),ones(P.structure.track.Nc,1))');
MM(n, 'N_pt'); L(n, 'Spread/1', 'N_pt/1'); L(n, 'N_LR/1', 'N_pt/2');
MM(n, 'pen_pt'); L(n, 'Spread/1', 'pen_pt/1'); L(n, 'sink/1', 'pen_pt/2', 'sinkage');
C(n, 'Zeros14a', 'zeros(2*P.structure.track.Nc,1)'); C(n, 'Zeros14b', 'zeros(2*P.structure.track.Nc,1)');
SIGC(n, 'slopeLong', 'grade_long/1'); SIGC(n, 'slopeLat', 'grade_lat/1');
SUM(n, 'F_diss', '++'); L(n, 'F_roll/1', 'F_diss/1'); L(n, 'turn_sign/1', 'F_diss/2');
DOT(n, 'P_res'); L(n, 'F_diss/1', 'P_res/1', 'F_diss'); L(n, 'v_b_d/1', 'P_res/2');
SUM(n, 'P_diss', '+++'); L(n, 'P_res/1', 'P_diss/1', 'P_res'); L(n, 'Pf_L/1', 'P_diss/2', 'P_fric_L'); L(n, 'Pf_R/1', 'P_diss/3', 'P_fric_R');
DOT(n, 'w2J'); L(n, 'w_raw/1', 'w2J/1'); L(n, 'w_raw/1', 'w2J/2');
K(n, 'E_kin', '0.5*P.structure.lofi.J_side'); L(n, 'w2J/1', 'E_kin/1', 'w2');
PROD(n, 'E_pot', '**'); L(n, 'Weight/1', 'E_pot/1'); L(n, 'H/1', 'E_pot/2');
SUM(n, 'E_mech', '++'); L(n, 'E_kin/1', 'E_mech/1', 'E_kin'); L(n, 'E_pot/1', 'E_mech/2', 'E_pot');
PROD(n, 'P_impl', '**'); L(n, 'Fimpl_x/1', 'P_impl/1'); L(n, 'kv/1', 'P_impl/2');
SUM(n, 'P_diss_E', '+-'); L(n, 'P_diss/1', 'P_diss_E/1', 'P_diss'); L(n, 'P_impl/1', 'P_diss_E/2', 'P_impl_chassis');
DTI(n, 'E_diss_J'); L(n, 'P_diss_E/1', 'E_diss_J/1');
% chi_eff = (v_bR - v_bL)/(B r), NaN for |r| < chi_rmin
SUM(n, 'dvB', '-+'); DEMUX(n, 'vBelt_LR', '2');
L(n, 'vBelt/1', 'vBelt_LR/1'); L(n, 'vBelt_LR/1', 'dvB/1', 'vBelt_L'); L(n, 'vBelt_LR/2', 'dvB/2', 'vBelt_R');
K(n, 'Br', 'P.structure.geom.B'); L(n, 'kr/1', 'Br/1');
PROD(n, 'chi_raw', '*/'); L(n, 'dvB/1', 'chi_raw/1', 'dvB'); L(n, 'Br/1', 'chi_raw/2', 'B_r');
A('simulink/Math Operations/Abs', [n '/abs_r'], 'ZeroCross', 'off'); L(n, 'kr/1', 'abs_r/1');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/r_small'], 'relop', '<', 'const', 'P.structure.track.chi_rmin');
L(n, 'abs_r/1', 'r_small/1'); C(n, 'NaN', 'NaN'); SW(n, 'chi_eff');
L(n, 'NaN/1', 'chi_eff/1'); L(n, 'r_small/1', 'chi_eff/2', 'r_small'); L(n, 'chi_raw/1', 'chi_eff/3', 'chi_raw');
BC(n, 'StructureDiag', 'Bus_StructureDiag', 12);
dsrc = {'N_pt/1', 'N_pt'; 'pen_pt/1', 'pen_pt'; 'Zeros14a/1', 'sX'; 'Zeros14b/1', 'sY'; 'slopeLong/1', 'slopeLong'; ...
        'slopeLat/1', 'slopeLat'; 'P_diss/1', ''; 'E_mech/1', 'E_mech'; 'E_diss_J/1', 'E_diss_J'; ...
        'E_impl/1', 'E_impl_shaft_J'; 'chi_eff/1', 'chi_eff'; 'Out_of_map/1', 'outOfMap'};
for k = 1:size(dsrc, 1)
    L(n, dsrc{k, 1}, sprintf('StructureDiag/%d', k), dsrc{k, 2});
end
L(n, 'StructureDiag/1', 'diag/1', 'diag');
set_param(n, 'Description', ['structure_lofi (T03, VC_STRUCT_FID == 2, spec 2.10): Differential Drive Kinematic Model ' ...
    '(Wheel Speeds, r_s(1 - i_nom), chi*B, Code generation) on 2 ms-filtered sprocket speeds; per side Simscape ' ...
    'Inertia (J_eq + m_belt r_s^2 + m/2 r_s^2), internal friction and the quasi-static load T = r_s(f_r N tanh(v_b/vEps) ' ...
    '+ m/2 g sin(grade) -/+ mu_lat W L/(4B) tanh(r/w_eps) - (F_impl,x + F_ext,x)/2) from 1 ms delayed speeds (D36); ' ...
    'quasi-static attitude from the DEM; the same Bus_TruthState / Bus_TrackState / Bus_StructureDiag (ENU/FLU).']);
end

% ===========================================================================================
%  implements
% ===========================================================================================
function local_impl_ports(n)
CPORT(n, 'Impl_Shaft', 1, 'Left');
IN(n, 'impl_cmd', 1, 'OutDataTypeStr', 'Bus: Bus_ImplementCmd', 'SampleTime', 'Ts.ctrl');
IN(n, 'v_fwd', 2); IN(n, 'pose2D', 3);
OUT(n, 'mech', 1, 'OutDataTypeStr', 'Bus: Bus_ImplMechState');
OUT(n, 'wrench', 2); OUT(n, 'dmdt', 3); OUT(n, 'P_shaft', 4);
end

function local_impl_shaft(n, J, D)
% Impl_Shaft -> torque sensor -> load node: inertia (+ bearing damper), Ideal Torque Source (S = -T_load),
% motion sensor; shaft speed delayed 1 ms for every load-torque law (D36); inner VC_IMPL_DRIVE variant
c = LIB();
A(c.TQS, [n '/Shaft_Torque_Sensor']);
A(c.J, [n '/J_load'], 'inertia', J, 'num_ports', 'foundation.enum.numPorts2.one');
A(c.TSRC, [n '/Load_Torque']);
A(c.RMS, [n '/Shaft_Motion_Sensor'], 'reference', 'foundation.enum.MeasurementReference.difference', ...
    'velocity_port', 'true', 'position_port', 'true', 'acceleration_port', 'false', 'wrap_angle', 'false');
A(c.RREF, [n '/Rot_Ref']);
SPSB(n, 'SPS_load', 'N*m', 'zero');
PSSB(n, 'PSS_w', 'rad/s'); PSSB(n, 'PSS_theta', 'rad'); PSSB(n, 'PSS_T', 'N*m');
hS = CPH(n, 'Impl_Shaft');
tq = PH([n '/Shaft_Torque_Sensor']); jj = PH([n '/J_load']); ts = PH([n '/Load_Torque']);
rm = PH([n '/Shaft_Motion_Sensor']); rr = PH([n '/Rot_Ref']); sp = PH([n '/SPS_load']);
pw = PH([n '/PSS_w']); pt = PH([n '/PSS_theta']); pT = PH([n '/PSS_T']);
PL(n, hS, tq.LConn(1));
nd = tq.RConn(1);
PL(n, nd, jj.LConn(1)); PL(n, nd, ts.LConn(1)); PL(n, ts.RConn(2), rr.LConn(1)); PL(n, sp.RConn(1), ts.RConn(1));
PL(n, nd, rm.LConn(1)); PL(n, rm.RConn(1), rr.LConn(1));
PL(n, rm.RConn(2), pw.LConn(1)); PL(n, rm.RConn(3), pt.LConn(1)); PL(n, tq.RConn(2), pT.LConn(1));
if ~isempty(D)
    A(c.DAMP, [n '/Bearing_Drag'], 'D', D);
    dp = PH([n '/Bearing_Drag']);
    PL(n, nd, dp.LConn(1)); PL(n, dp.RConn(1), rr.LConn(1));
end
UD(n, 'w_delay', '0');
L(n, 'PSS_w/1', 'w_delay/1', 'shaftSpeed');
PROD(n, 'mechPower', '**');
L(n, 'PSS_w/1', 'mechPower/1'); L(n, 'PSS_T/1', 'mechPower/2', 'shaftTorque');
L(n, 'mechPower/1', 'P_shaft/1', 'mechPower');
K(n, 'Neg_T_load', '-1');
L(n, 'Neg_T_load/1', 'SPS_load/1', 'minus_T_load');
% command: each element of impl_cmd that a file uses is picked by a Bus Selector Cmd_<element> in front of its
% consumer (layout spec 6.2); speedCmd here, liftDuty in local_lift, nozzleOn in the sprayer
RT(n, 'RT_speedCmd', 'Ts.base', '0');
PICK(n, 'Cmd_speedCmd', 'impl_cmd/1', {'speedCmd'}, {'RT_speedCmd/1'}, 'impl_cmd');
local_drive_variant(n);
L(n, 'RT_speedCmd/1', 'Drive/1', 'speedCmd_1k');
PL(n, hS, shaddad_bu('pport', [n '/Drive'], 'Shaft'));
end

function local_drive_variant(n)
% Variant Subsystem VC_IMPL_DRIVE: 1 ideal speed source from speedCmd (Rate Limiter +/-a_max, S-PS filter tau_f;
% structure harness only), 2 shaft driven by the POWER implement motor through Impl_Shaft (system default)
c = LIB();
vs = [n '/Drive'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
ch = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
ch = setdiff(ch, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(ch), delete_block(ch{k}); end
IN(vs, 'speedCmd', 1);
CPORT(vs, 'Shaft', 1, 'Left');
% choice 1: ideal speed source
s1 = [vs '/Ideal_Speed'];
add_block('built-in/SubSystem', s1, 'VariantControl', 'VC_IMPL_DRIVE == 1');
IN(s1, 'speedCmd', 1); CPORT(s1, 'Shaft', 1, 'Left');
A('simulink/Discontinuities/Rate Limiter', [s1 '/Speed_Ramp'], 'RisingSlewLimit', 'P.structure.impl.drive.a_max', ...
    'FallingSlewLimit', '-P.structure.impl.drive.a_max', 'InitialCondition', '0');
SPSB(s1, 'SPS_w', 'rad/s', 'filter', 'P.structure.impl.drive.tau_f');
A(c.WSRC, [s1 '/Ideal_Speed_Source']);
A(c.RREF, [s1 '/Rot_Ref']);
L(s1, 'speedCmd/1', 'Speed_Ramp/1', 'speedCmd'); L(s1, 'Speed_Ramp/1', 'SPS_w/1', 'w_ramp');
ws = PH([s1 '/Ideal_Speed_Source']); rr = PH([s1 '/Rot_Ref']); sp = PH([s1 '/SPS_w']);
PL(s1, CPH(s1, 'Shaft'), ws.LConn(1)); PL(s1, ws.RConn(2), rr.LConn(1)); PL(s1, sp.RConn(1), ws.RConn(1));
% choice 2: motor through Impl_Shaft (free end inside)
s2 = [vs '/Motor_Port'];
add_block('built-in/SubSystem', s2, 'VariantControl', 'VC_IMPL_DRIVE == 2');
IN(s2, 'speedCmd', 1); CPORT(s2, 'Shaft', 1, 'Left');
A('simulink/Sinks/Terminator', [s2 '/Term']);
L(s2, 'speedCmd/1', 'Term/1');
A(c.FREE, [s2 '/Free_End']);
fe = PH([s2 '/Free_End']);
PL(s2, CPH(s2, 'Shaft'), fe.LConn(1));
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

function local_veg(n, xVeg)
% vegetation density at the tool point (x_E + x cos psi, y_N + x sin psi)
DEMUX(n, 'Pose', '3');
L(n, 'pose2D/1', 'Pose/1', 'pose2D');
TRIG(n, 'cos_psi', 'cos'); TRIG(n, 'sin_psi', 'sin');
L(n, 'Pose/3', 'cos_psi/1', 'psi'); L(n, 'Pose/3', 'sin_psi/1');
K(n, 'dx', xVeg); K(n, 'dy', xVeg);
L(n, 'cos_psi/1', 'dx/1'); L(n, 'sin_psi/1', 'dy/1');
SUM(n, 'x_tool', '++'); SUM(n, 'y_tool', '++');
L(n, 'Pose/1', 'x_tool/1', 'x_E'); L(n, 'dx/1', 'x_tool/2'); L(n, 'Pose/2', 'y_tool/1', 'y_N'); L(n, 'dy/1', 'y_tool/2');
LUT2(n, 'Veg', 'P.world.veg.rho');
L(n, 'x_tool/1', 'Veg/1', 'x_tool'); L(n, 'y_tool/1', 'Veg/2', 'y_tool');
end

function local_lift(n, rateExpr, lo, hi, ic, tau)
% tool actuator: Discrete-Time Integrator (Ts.ctrl) of liftDuty*rate with limits, first-order lag (continuous)
K(n, 'Lift_rate', rateExpr);
PICK(n, 'Cmd_liftDuty', 'impl_cmd/1', {'liftDuty'}, {'Lift_rate/1'}, 'impl_cmd');
A('simulink/Discrete/Discrete-Time Integrator', [n '/Lift_Actuator'], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1', 'InitialCondition', ic, 'SampleTime', 'Ts.ctrl', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', hi, 'LowerSaturationLimit', lo, 'ShowSaturationPort', 'on');
L(n, 'Lift_rate/1', 'Lift_Actuator/1');
A('simulink/Math Operations/Abs', [n '/abs_duty'], 'ZeroCross', 'off');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/duty_on'], 'relop', '>', 'const', 'P.structure.impl.lift.duty_dead');
A('simulink/Logic and Bit Operations/Compare To Zero', [n '/not_at_stop'], 'relop', '==');
A('simulink/Logic and Bit Operations/Logical Operator', [n '/moving'], 'Operator', 'AND', 'Inputs', '2');
PICK(n, 'Cmd_liftDuty_2', 'impl_cmd/1', {'liftDuty'}, {'abs_duty/1'}, 'impl_cmd'); L(n, 'abs_duty/1', 'duty_on/1');
L(n, 'Lift_Actuator/2', 'not_at_stop/1', 'end_stop');
L(n, 'duty_on/1', 'moving/1'); L(n, 'not_at_stop/1', 'moving/2');
RT(n, 'RT_moving', 'Ts.base', 'false');
L(n, 'moving/1', 'RT_moving/1', 'liftMoving_10ms');
RT(n, 'RT_actuator', 'Ts.base', ic);
L(n, 'Lift_Actuator/1', 'RT_actuator/1', 'actuator_pos');
% first-order lag with the actuator initial position: y' = (u - y)/tau
SUM(n, 'lag_err', '+-'); K(n, 'lag_gain', ['1/' tau]);
A('simulink/Continuous/Integrator', [n '/Tool_Lag'], 'InitialCondition', ic);
L(n, 'RT_actuator/1', 'lag_err/1'); L(n, 'Tool_Lag/1', 'lag_err/2', 'toolHeight');
L(n, 'lag_err/1', 'lag_gain/1'); L(n, 'lag_gain/1', 'Tool_Lag/1');
end

function local_mech_bus(n, src, nm)
BC(n, 'ImplMechState', 'Bus_ImplMechState', 13);
for k = 1:numel(src)
    L(n, src{k}, sprintf('ImplMechState/%d', k), nm{k});
end
L(n, 'ImplMechState/1', 'mech/1', 'mech');
end

% ---- none ----
function local_impl_none(n)
c = LIB();
local_impl_ports(n);
A(c.J, [n '/J_shaft'], 'inertia', 'P.structure.impl.none.J', 'num_ports', 'foundation.enum.numPorts2.one');
jj = PH([n '/J_shaft']);
PL(n, CPH(n, 'Impl_Shaft'), jj.LConn(1));
for k = {'impl_cmd', 'v_fwd', 'pose2D'}
    A('simulink/Sinks/Terminator', [n '/Term_' k{1}]); L(n, [k{1} '/1'], ['Term_' k{1} '/1'], k{1});
end
C(n, 'Mech_zero', 'P.ic.Bus_ImplMechState', 'OutDataTypeStr', 'Bus: Bus_ImplMechState');
L(n, 'Mech_zero/1', 'mech/1', 'mech');
C(n, 'Wrench_zero', 'zeros(9,1)'); L(n, 'Wrench_zero/1', 'wrench/1', 'wrench');
C(n, 'dmdt_zero', '0'); L(n, 'dmdt_zero/1', 'dmdt/1', 'dmdt');
C(n, 'P_zero', '0'); L(n, 'P_zero/1', 'P_shaft/1', 'P_shaft');
set_param(n, 'Description', ['structure_impl_none (T03, VC_IMPL == 0): Inertia P.structure.impl.none.J on Impl_Shaft, ' ...
    'zero outputs (the POWER-side Impl_None carries the network tie, ICD 2).']);
end

% ---- mower ----
function local_impl_mower(n)
local_impl_ports(n);
local_impl_shaft(n, 'P.structure.impl.mower.J', 'P.structure.impl.mower.D');
local_veg(n, 'P.structure.impl.mower.pos(1)');
local_lift(n, '-P.structure.impl.mower.h_rate', 'P.structure.impl.mower.h_range(1)', 'P.structure.impl.mower.h_range(2)', ...
    'P.structure.impl.mower.h0', 'P.structure.impl.mower.h_tau');
A('simulink/Sinks/Terminator', [n '/Term_theta']); L(n, 'PSS_theta/1', 'Term_theta/1', 'theta_shaft');   % unused here
% load torque T = k_aero w|w| + T_cut, T_cut = e_A w max(v,0) rho/max(|w|, 0.3 w_nom) sgn(w)
A('simulink/Math Operations/Abs', [n '/abs_w'], 'ZeroCross', 'off');
PROD(n, 'w_absw', '**'); K(n, 'T_aero', 'P.structure.impl.mower.k_aero');
L(n, 'w_delay/1', 'abs_w/1', 'w_d'); L(n, 'w_delay/1', 'w_absw/1'); L(n, 'abs_w/1', 'w_absw/2', 'abs_w');
L(n, 'w_absw/1', 'T_aero/1');
A('simulink/Discontinuities/Saturation', [n '/v_pos'], 'UpperLimit', 'inf', 'LowerLimit', '0');
L(n, 'v_fwd/1', 'v_pos/1', 'v_fwd');
C(n, 'w_min', 'P.structure.impl.mower.w_min_frac*P.structure.impl.mower.w_nom');
A('simulink/Math Operations/MinMax', [n '/w_floor'], 'Function', 'max', 'Inputs', '2', 'ZeroCross', 'off');
L(n, 'abs_w/1', 'w_floor/1'); L(n, 'w_min/1', 'w_floor/2');
A('simulink/Math Operations/Sign', [n '/sgn_w'], 'ZeroCross', 'off');
L(n, 'w_delay/1', 'sgn_w/1');
PROD(n, 'T_cut', '***/');
K(n, 'eA_w', 'P.structure.impl.mower.e_A*P.structure.impl.mower.w');
L(n, 'v_pos/1', 'eA_w/1', 'v_pos');
L(n, 'eA_w/1', 'T_cut/1'); L(n, 'Veg/1', 'T_cut/2', 'rho_veg'); L(n, 'sgn_w/1', 'T_cut/3', 'sgn_w');
L(n, 'w_floor/1', 'T_cut/4', 'w_floor');
SUM(n, 'T_load', '++');
L(n, 'T_aero/1', 'T_load/1', 'T_aero'); L(n, 'T_cut/1', 'T_load/2', 'T_cut');
L(n, 'T_load/1', 'Neg_T_load/1', 'T_load');
% chassis wrench: F = [-(f_gw lambda m g + c_veg rho w) tanh(v/v_eps); 0; 0] at pos, M_z = k_yaw T_cut. The deck is
% rigid on two caster gauge wheels that touch the ground (the user, 2026-11-15): their normal force is a HiFi contact
% (MF-C2 of structure_contact, P.structure.contact.gw); their rolling drag stays here, at the static wheel load lambda m g
C(n, 'F_gw', 'P.structure.impl.mower.f_gw*P.structure.impl.mower.lambda_gw*P.structure.impl.mower.mass*P.frames.g');
K(n, 'F_veg', 'P.structure.impl.mower.c_veg*P.structure.impl.mower.w');
L(n, 'Veg/1', 'F_veg/1');
SUM(n, 'F_drag', '++'); L(n, 'F_gw/1', 'F_drag/1', 'F_gw'); L(n, 'F_veg/1', 'F_drag/2', 'F_veg');
K(n, 'inv_veps', '1/P.structure.impl.mower.v_eps'); TRIG(n, 'tanh_v', 'tanh');
L(n, 'v_fwd/1', 'inv_veps/1'); L(n, 'inv_veps/1', 'tanh_v/1');
PROD(n, 'Fx_drag', '**'); K(n, 'Neg_Fx', '-1');
L(n, 'F_drag/1', 'Fx_drag/1', 'F_drag'); L(n, 'tanh_v/1', 'Fx_drag/2'); L(n, 'Fx_drag/1', 'Neg_Fx/1');
C(n, 'Zero', '0');
K(n, 'Mz_yaw', 'P.structure.impl.mower.k_yaw'); L(n, 'T_cut/1', 'Mz_yaw/1');
C(n, 'r_deck', 'P.structure.impl.mower.pos(:)');
MUX(n, 'Wrench_mux', 7);
L(n, 'Neg_Fx/1', 'Wrench_mux/1', 'F_x'); L(n, 'Zero/1', 'Wrench_mux/2'); L(n, 'Zero/1', 'Wrench_mux/3');
L(n, 'Zero/1', 'Wrench_mux/4'); L(n, 'Zero/1', 'Wrench_mux/5'); L(n, 'Mz_yaw/1', 'Wrench_mux/6', 'M_z');
L(n, 'r_deck/1', 'Wrench_mux/7', 'r_deck');
L(n, 'Wrench_mux/1', 'wrench/1', 'wrench');
C(n, 'dmdt_zero', '0'); L(n, 'dmdt_zero/1', 'dmdt/1', 'dmdt');
% bus
local_mech_common(n, 'P.structure.impl.mower.h_range(2)', 'P.structure.impl.mower.h_range(2)-P.structure.impl.mower.h_range(1)', 1);
set_param(n, 'Description', ['structure_impl_mower (T03, VC_IMPL == 1, spec 2.8): blade load T = k_aero w|w| + e_A w ' ...
    'max(v,0) rho_veg / max(|w|, 0.3 w_nom) sgn(w) on the 1 ms delayed shaft speed (D36), inertia J + bearing drag D, ' ...
    'deck drag on the chassis (vegetation + the gauge wheels'' rolling drag at their static load; the rigid deck''s ' ...
    'gauge-wheel normal force is the HiFi contact MF-C2 of structure_contact), deck height actuator (Discrete-Time ' ...
    'Integrator 0.01 s + lag).']);
end

function local_mech_common(n, posTop, posSpan, typeId)
% liftPos01 = (posTop - actuator)/posSpan (mower) or actuator/posSpan (tiller: posTop = '0', negative span)
SUM(n, 'pos_rel', '+-'); C(n, 'pos_top', posTop);
L(n, 'pos_top/1', 'pos_rel/1'); L(n, 'RT_actuator/1', 'pos_rel/2');
K(n, 'liftPos01', ['1/(' posSpan ')']); L(n, 'pos_rel/1', 'liftPos01/1');
C(n, 'Type', sprintf('uint8(%d)', typeId), 'OutDataTypeStr', 'uint8');
C(n, 'Zero_tankLevel', '0'); C(n, 'Zero_tankMass', '0'); C(n, 'Zero_sprayFlow', '0'); C(n, 'Zero_linePressure', '0');
K(n, 'F_impl_B', 'P.frames.R_FRD_FLU', 'Matrix(K*u)');
SEL(n, 'F_frd', '1:3', '9'); L(n, 'Wrench_mux/1', 'F_frd/1'); L(n, 'F_frd/1', 'F_impl_B/1', 'F_impl_FRD');
src = {'Type/1', 'PSS_w/1', 'PSS_T/1', 'mechPower/1', 'Tool_Lag/1', 'liftPos01/1', 'RT_moving/1', ...
       'Zero_tankLevel/1', 'Zero_tankMass/1', 'Zero_sprayFlow/1', 'Zero_linePressure/1', 'Veg/1', 'F_impl_B/1'};
nm = {'type', '', '', '', '', 'liftPos01', 'liftMoving', 'tankLevel', 'tankMass', 'sprayFlow', 'linePressure', ...
      'vegDensity', 'F_impl_B'};
local_mech_bus(n, src, nm);
end

% ---- tiller ----
function local_impl_tiller(n)
local_impl_ports(n);
local_impl_shaft(n, 'P.structure.impl.tiller.J', '');
local_veg(n, 'P.structure.impl.tiller.x_rot');
local_lift(n, 'P.structure.impl.tiller.d_rate', '0', 'P.structure.impl.tiller.d_max', ...
    'P.structure.impl.tiller.d0', 'P.structure.impl.tiller.d_tau');
% P_soil = e_V w d max(v,0); T_soil = P_soil/max(|w|, 0.3 w_nom) (1 + a_r sin(n_t theta)) sgn(w) + T_idle [d > 0] sgn(w)
A('simulink/Discontinuities/Saturation', [n '/v_pos'], 'UpperLimit', 'inf', 'LowerLimit', '0');
A('simulink/Discontinuities/Saturation', [n '/d_pos'], 'UpperLimit', 'inf', 'LowerLimit', '0');
L(n, 'v_fwd/1', 'v_pos/1', 'v_fwd'); L(n, 'Tool_Lag/1', 'd_pos/1');
PROD(n, 'P_soil', '**'); K(n, 'eV_w', 'P.structure.impl.tiller.e_V*P.structure.impl.tiller.w');
L(n, 'v_pos/1', 'P_soil/1', 'v_pos'); L(n, 'd_pos/1', 'P_soil/2', 'd_pos'); L(n, 'P_soil/1', 'eV_w/1');
A('simulink/Math Operations/Abs', [n '/abs_w'], 'ZeroCross', 'off'); L(n, 'w_delay/1', 'abs_w/1', 'w_d');
C(n, 'w_min', 'P.structure.impl.tiller.w_min_frac*P.structure.impl.tiller.w_nom');
A('simulink/Math Operations/MinMax', [n '/w_floor'], 'Function', 'max', 'Inputs', '2', 'ZeroCross', 'off');
L(n, 'abs_w/1', 'w_floor/1'); L(n, 'w_min/1', 'w_floor/2');
PROD(n, 'T_soil0', '*/'); L(n, 'eV_w/1', 'T_soil0/1', 'P_soil'); L(n, 'w_floor/1', 'T_soil0/2', 'w_floor');
UD(n, 'theta_delay', '0'); L(n, 'PSS_theta/1', 'theta_delay/1', 'theta_rotor');
K(n, 'n_t', 'P.structure.impl.tiller.n_t'); TRIG(n, 'sin_nt', 'sin'); K(n, 'a_r', 'P.structure.impl.tiller.a_r');
A('simulink/Math Operations/Bias', [n '/ripple'], 'Bias', '1');
L(n, 'theta_delay/1', 'n_t/1'); L(n, 'n_t/1', 'sin_nt/1'); L(n, 'sin_nt/1', 'a_r/1'); L(n, 'a_r/1', 'ripple/1');
PROD(n, 'T_soil', '**'); L(n, 'T_soil0/1', 'T_soil/1', 'T_soil_mean'); L(n, 'ripple/1', 'T_soil/2', 'ripple');
A('simulink/Logic and Bit Operations/Compare To Constant', [n '/in_ground'], 'relop', '>', 'const', 'P.structure.impl.tiller.d_on');
L(n, 'Tool_Lag/1', 'in_ground/1');
A('simulink/Signal Attributes/Data Type Conversion', [n '/in_ground_dbl'], 'OutDataTypeStr', 'double');
L(n, 'in_ground/1', 'in_ground_dbl/1', 'in_ground');
K(n, 'T_idle', 'P.structure.impl.tiller.T_idle'); L(n, 'in_ground_dbl/1', 'T_idle/1');
SUM(n, 'T_abs', '++'); L(n, 'T_soil/1', 'T_abs/1', 'T_soil'); L(n, 'T_idle/1', 'T_abs/2', 'T_idle');
A('simulink/Math Operations/Sign', [n '/sgn_w'], 'ZeroCross', 'off'); L(n, 'w_delay/1', 'sgn_w/1');
PROD(n, 'T_load', '**'); L(n, 'T_abs/1', 'T_load/1'); L(n, 'sgn_w/1', 'T_load/2', 'sgn_w');
L(n, 'T_load/1', 'Neg_T_load/1', 'T_load');
% chassis wrench at r_rotor = [x_rot 0 -(R_rot - d)]
K(n, 'F_push', 'P.structure.impl.tiller.kPush/P.structure.impl.tiller.R_rot');
K(n, 'F_lift', '-P.structure.impl.tiller.kLift/P.structure.impl.tiller.R_rot');
L(n, 'T_soil/1', 'F_push/1'); L(n, 'T_soil/1', 'F_lift/1');
C(n, 'W_skid', 'P.structure.impl.tiller.lambda_skid*P.structure.impl.tiller.mass*P.frames.g');
PROD(n, 'W_skid_on', '**'); L(n, 'W_skid/1', 'W_skid_on/1'); L(n, 'in_ground_dbl/1', 'W_skid_on/2');
K(n, 'inv_veps', '1/P.structure.impl.tiller.v_eps'); TRIG(n, 'tanh_v', 'tanh');
L(n, 'v_fwd/1', 'inv_veps/1'); L(n, 'inv_veps/1', 'tanh_v/1');
PROD(n, 'F_skid', '**'); K(n, 'mu_skid', 'P.structure.impl.tiller.mu_skid');
L(n, 'W_skid_on/1', 'F_skid/1', 'W_skid'); L(n, 'tanh_v/1', 'F_skid/2'); L(n, 'F_skid/1', 'mu_skid/1');
SUM(n, 'F_x', '+-'); L(n, 'F_push/1', 'F_x/1', 'F_push'); L(n, 'mu_skid/1', 'F_x/2', 'F_skid');
SUM(n, 'F_z', '+-'); L(n, 'F_lift/1', 'F_z/1', 'F_lift'); L(n, 'W_skid_on/1', 'F_z/2');
C(n, 'Zero', '0'); C(n, 'x_rot', 'P.structure.impl.tiller.x_rot');
A('simulink/Math Operations/Bias', [n '/z_rot'], 'Bias', '-P.structure.impl.tiller.R_rot');
L(n, 'Tool_Lag/1', 'z_rot/1');
MUX(n, 'Wrench_mux', 9);
wsrc = {'F_x/1', 'Zero/1', 'F_z/1', 'Zero/1', 'Zero/1', 'Zero/1', 'x_rot/1', 'Zero/1', 'z_rot/1'};
wnm = {'F_x', '', 'F_z', '', '', '', 'x_rot', '', 'z_rot'};
for k = 1:9, L(n, wsrc{k}, sprintf('Wrench_mux/%d', k), wnm{k}); end
L(n, 'Wrench_mux/1', 'wrench/1', 'wrench');
C(n, 'dmdt_zero', '0'); L(n, 'dmdt_zero/1', 'dmdt/1', 'dmdt');
local_mech_common(n, '0', '-P.structure.impl.tiller.d_max', 2);
set_param(n, 'Description', ['structure_impl_tiller (T03, VC_IMPL == 2, spec 2.8): rotor-side soil torque ' ...
    'e_V w d max(v,0)/max(|w|, 0.3 w_nom) (1 + a_r sin(n_t theta)) + T_idle [d > 0] on the 1 ms delayed rotor speed / ' ...
    'angle (D36), rotor inertia J, down-cut push / uplift and depth-skid support on the chassis at the rotor, depth ' ...
    'actuator (Discrete-Time Integrator 0.01 s, [0 d_max] + lag).']);
end

% ---- sprayer ----
function local_impl_sprayer(n)
local_impl_ports(n);
local_impl_shaft(n, 'P.structure.impl.sprayer.J', '');
local_veg(n, 'P.structure.impl.sprayer.x_boom');
A('simulink/Sinks/Terminator', [n '/Term_v']); L(n, 'v_fwd/1', 'Term_v/1', 'v_fwd');
A('simulink/Sinks/Terminator', [n '/Term_theta']); L(n, 'PSS_theta/1', 'Term_theta/1', 'theta_shaft');   % unused here
RT(n, 'RT_nozzles', 'Ts.base', 'false(P.structure.impl.sprayer.nNoz,1)');
PICK(n, 'Cmd_nozzleOn', 'impl_cmd/1', {'nozzleOn'}, {'RT_nozzles/1'}, 'impl_cmd');
A('simulink/Signal Attributes/Data Type Conversion', [n '/noz_dbl'], 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Sum of Elements', [n '/n_on']);
L(n, 'RT_nozzles/1', 'noz_dbl/1', 'nozzleOn_1k'); L(n, 'noz_dbl/1', 'n_on/1');
% pump flow Q_pump = D_p max(w,0) eta_v (1 ms delayed speed)
A('simulink/Discontinuities/Saturation', [n '/w_pos'], 'UpperLimit', 'inf', 'LowerLimit', '0');
K(n, 'Q_pump', 'P.structure.impl.sprayer.D_p*P.structure.impl.sprayer.eta_v');
L(n, 'w_delay/1', 'w_pos/1', 'w_d'); L(n, 'w_pos/1', 'Q_pump/1');
% nozzle demand at the set pressure; line pressure p = p_set min(1, (Q_pump/Q_open)^2) [V > 0]
K(n, 'Q_open', 'P.structure.impl.sprayer.Q_ref*sqrt(P.structure.impl.sprayer.p_set/P.structure.impl.sprayer.p_ref)');
L(n, 'n_on/1', 'Q_open/1', 'n_on');
C(n, 'Q_eps', 'P.structure.impl.sprayer.Q_eps');
A('simulink/Math Operations/MinMax', [n '/Q_open_floor'], 'Function', 'max', 'Inputs', '2', 'ZeroCross', 'off');
L(n, 'Q_open/1', 'Q_open_floor/1', 'Q_open'); L(n, 'Q_eps/1', 'Q_open_floor/2');
PROD(n, 'q_ratio', '*/'); L(n, 'Q_pump/1', 'q_ratio/1', 'Q_pump'); L(n, 'Q_open_floor/1', 'q_ratio/2');
A('simulink/Math Operations/Math Function', [n '/q_ratio2'], 'Operator', 'square');
L(n, 'q_ratio/1', 'q_ratio2/1', 'q_ratio');
C(n, 'One', '1');
A('simulink/Math Operations/MinMax', [n '/p_frac'], 'Function', 'min', 'Inputs', '2', 'ZeroCross', 'off');
L(n, 'q_ratio2/1', 'p_frac/1'); L(n, 'One/1', 'p_frac/2');
% tank volume: Integrator [0 V_max], IC fill0 V_max
A('simulink/Continuous/Integrator', [n '/Tank_Volume'], 'InitialCondition', ...
    'P.structure.impl.sprayer.fill0*P.structure.impl.sprayer.V_max', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.structure.impl.sprayer.V_max', 'LowerSaturationLimit', '0', 'ZeroCross', 'off');
A('simulink/Logic and Bit Operations/Compare To Zero', [n '/tank_has_liquid'], 'relop', '>');
A('simulink/Signal Attributes/Data Type Conversion', [n '/liquid_dbl'], 'OutDataTypeStr', 'double');
L(n, 'Tank_Volume/1', 'tank_has_liquid/1', 'V_tank'); L(n, 'tank_has_liquid/1', 'liquid_dbl/1');
PROD(n, 'p_frac_on', '**'); K(n, 'p_line', 'P.structure.impl.sprayer.p_set');
L(n, 'p_frac/1', 'p_frac_on/1', 'p_frac'); L(n, 'liquid_dbl/1', 'p_frac_on/2', 'liquid');
L(n, 'p_frac_on/1', 'p_line/1');
% nozzle flow Q = n_on Q_ref sqrt(p/p_ref)
K(n, 'p_rel', '1/P.structure.impl.sprayer.p_ref'); A('simulink/Math Operations/Sqrt', [n '/sqrt_p']);
L(n, 'p_line/1', 'p_rel/1', 'p'); L(n, 'p_rel/1', 'sqrt_p/1');
K(n, 'Q_ref', 'P.structure.impl.sprayer.Q_ref');
PROD(n, 'Q_noz', '**');
L(n, 'n_on/1', 'Q_ref/1'); L(n, 'Q_ref/1', 'Q_noz/1'); L(n, 'sqrt_p/1', 'Q_noz/2');
A('simulink/Math Operations/Unary Minus', [n '/minus_Q']);
L(n, 'Q_noz/1', 'minus_Q/1', 'Q_noz'); L(n, 'minus_Q/1', 'Tank_Volume/1');
K(n, 'dmdt_gain', '-P.structure.impl.sprayer.rho_liq'); L(n, 'Q_noz/1', 'dmdt_gain/1');
L(n, 'dmdt_gain/1', 'dmdt/1', 'dmdt');
% pump torque T = D_p p/eta_m sgn(w)
K(n, 'T_pump', 'P.structure.impl.sprayer.D_p/P.structure.impl.sprayer.eta_m');
L(n, 'p_line/1', 'T_pump/1');
A('simulink/Math Operations/Sign', [n '/sgn_w'], 'ZeroCross', 'off'); L(n, 'w_delay/1', 'sgn_w/1');
PROD(n, 'T_load', '**'); L(n, 'T_pump/1', 'T_load/1', 'T_pump'); L(n, 'sgn_w/1', 'T_load/2', 'sgn_w');
L(n, 'T_load/1', 'Neg_T_load/1', 'T_load');
C(n, 'Wrench_zero', 'zeros(9,1)'); L(n, 'Wrench_zero/1', 'wrench/1', 'wrench');
% bus
C(n, 'Type', 'uint8(3)', 'OutDataTypeStr', 'uint8');
C(n, 'Zero_toolHeight', '0'); C(n, 'Zero_liftPos01', '0');
C(n, 'False', 'false', 'OutDataTypeStr', 'boolean');
K(n, 'tankLevel', '1000'); K(n, 'tankMass', 'P.structure.impl.sprayer.rho_liq'); K(n, 'sprayFlow', '60000');
L(n, 'Tank_Volume/1', 'tankLevel/1'); L(n, 'Tank_Volume/1', 'tankMass/1'); L(n, 'Q_noz/1', 'sprayFlow/1');
C(n, 'F_zero', 'zeros(3,1)');
src = {'Type/1', 'PSS_w/1', 'PSS_T/1', 'mechPower/1', 'Zero_toolHeight/1', 'Zero_liftPos01/1', 'False/1', 'tankLevel/1', 'tankMass/1', ...
       'sprayFlow/1', 'p_line/1', 'Veg/1', 'F_zero/1'};
nm = {'type', '', '', '', 'toolHeight', 'liftPos01', 'liftMoving', 'tankLevel', 'tankMass', 'sprayFlow', ...
      'linePressure', 'vegDensity', 'F_impl_B'};
local_mech_bus(n, src, nm);
set_param(n, 'Description', ['structure_impl_sprayer (T03, VC_IMPL == 3, spec 2.8, D9): pump on Impl_Shaft, ' ...
    'Q_pump = D_p w eta_v (1 ms delayed speed, D36), nozzle demand Q_open = n_on Q_ref sqrt(p_set/p_ref), ' ...
    'p = p_set min(1, (Q_pump/Q_open)^2) [V > 0], Q_noz = n_on Q_ref sqrt(p/p_ref) (= min(Q_open, Q_pump), evaluated ' ...
    'without an algebraic loop), T_pump = D_p p/eta_m, tank Integrator [0 V_max] (IC fill0 V_max), dm/dt = -rho Q_noz.']);
end

% ---- implement container ----
function local_implement(n)
% outputs: impl_mech, and the virtual bus 'impl' (wrench, dmdt, E_impl: the chassis takes all three, layout spec
% 6.3; Out Bus Element port 2, see the end)
CPORT(n, 'Impl_Shaft', 1, 'Left');
IN(n, 'impl_cmd', 1, 'OutDataTypeStr', 'Bus: Bus_ImplementCmd', 'SampleTime', 'Ts.ctrl');
IN(n, 'v_fwd', 2); IN(n, 'pose2D', 3);
OUT(n, 'impl_mech', 1, 'OutDataTypeStr', 'Bus: Bus_ImplMechState', 'SampleTime', 'Ts.ctrl');
vs = [n '/Impl'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
ch = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
ch = setdiff(ch, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(ch), delete_block(ch{k}); end
IN(vs, 'impl_cmd', 1, 'OutDataTypeStr', 'Bus: Bus_ImplementCmd'); IN(vs, 'v_fwd', 2); IN(vs, 'pose2D', 3);
OUT(vs, 'mech', 1, 'OutDataTypeStr', 'Bus: Bus_ImplMechState'); OUT(vs, 'wrench', 2); OUT(vs, 'dmdt', 3); OUT(vs, 'P_shaft', 4);
CPORT(vs, 'Impl_Shaft', 1, 'Left');
chs = {'None', 'structure_impl_none', 0; 'Mower', 'structure_impl_mower', 1; ...
       'Tiller', 'structure_impl_tiller', 2; 'Sprayer', 'structure_impl_sprayer', 3};
for k = 1:size(chs, 1)
    A('simulink/Ports & Subsystems/Subsystem Reference', [vs '/' chs{k, 1}], 'ReferencedSubsystem', chs{k, 2}, ...
        'VariantControl', sprintf('VC_IMPL == %d', chs{k, 3}));
end
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
L(n, 'impl_cmd/1', 'Impl/1', 'impl_cmd'); L(n, 'v_fwd/1', 'Impl/2', 'v_fwd'); L(n, 'pose2D/1', 'Impl/3', 'pose2D');
PL(n, CPH(n, 'Impl_Shaft'), shaddad_bu('pport', vs, 'Impl_Shaft'));
A('simulink/Discrete/Zero-Order Hold', [n '/ZOH_impl_mech'], 'SampleTime', 'Ts.ctrl');
L(n, 'Impl/1', 'ZOH_impl_mech/1', 'mech'); L(n, 'ZOH_impl_mech/1', 'impl_mech/1', 'impl_mech');
OB(n, 'impl', 'wrench', 'Impl/2'); OB(n, 'impl', 'dmdt', 'Impl/3');
DTI(n, 'E_impl_shaft');
L(n, 'Impl/4', 'E_impl_shaft/1', 'P_shaft'); OB(n, 'impl', 'E_impl', 'E_impl_shaft/1');
PORTS(n, {'impl_mech', 'impl'}, 'Outport');
set_param(n, 'Description', ['structure_implement (T03): Variant Subsystem VC_IMPL (0 none, 1 mower, 2 tiller, ' ...
    '3 sprayer; update diagram) of the implement load mechanics on Impl_Shaft; impl_mech exported at 0.01 s through ' ...
    'an internal Zero-Order Hold (D32); E_impl_shaft_J = 1 ms integral of the shaft power (D40). wrench = ' ...
    '[F (FRD) ; M (FRD) ; application point r (FRD w.r.t. R)] on the chassis, dmdt = sprayer tank mass rate; ' ...
    'wrench, dmdt and E_impl leave as the virtual bus impl (an Out Bus Element port).']);
end

% ===========================================================================================
%  structure_plant (domain top, ICD 3.2)
% ===========================================================================================
function local_plant(n)
CPORT(n, 'Spr_L', 1, 'Left'); CPORT(n, 'Spr_R', 2, 'Left'); CPORT(n, 'Impl_Shaft', 3, 'Left');
IN(n, 'impl_cmd', 1, 'OutDataTypeStr', 'Bus: Bus_ImplementCmd', 'SampleTime', 'Ts.ctrl');
IN(n, 'env_dist', 2, 'OutDataTypeStr', 'Bus: Bus_EnvDisturbance', 'SampleTime', 'Ts.base');
IN(n, 'env_ovr', 3, 'OutDataTypeStr', 'Bus: Bus_EnvOverride', 'SampleTime', 'Ts.env');
OUT(n, 'truth', 1, 'OutDataTypeStr', 'Bus: Bus_TruthState');
OUT(n, 'tracks', 2, 'OutDataTypeStr', 'Bus: Bus_TrackState');
OUT(n, 'impl_mech', 3, 'OutDataTypeStr', 'Bus: Bus_ImplMechState', 'SampleTime', 'Ts.ctrl');
OUT(n, 'env', 4, 'OutDataTypeStr', 'Bus: Bus_EnvState', 'SampleTime', 'Ts.env');
OUT(n, 'diag', 5, 'OutDataTypeStr', 'Bus: Bus_StructureDiag');
% fidelity variant
vs = [n '/Fidelity'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
ch = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
ch = setdiff(ch, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(ch), delete_block(ch{k}); end
IN(vs, 'wrench', 1); IN(vs, 'dmdt', 2); IN(vs, 'E_impl', 3);
IN(vs, 'env_dist', 4, 'OutDataTypeStr', 'Bus: Bus_EnvDisturbance');
IN(vs, 'env_ovr', 5, 'OutDataTypeStr', 'Bus: Bus_EnvOverride');
OUT(vs, 'truth', 1, 'OutDataTypeStr', 'Bus: Bus_TruthState');
OUT(vs, 'tracks', 2, 'OutDataTypeStr', 'Bus: Bus_TrackState');
OUT(vs, 'diag', 3, 'OutDataTypeStr', 'Bus: Bus_StructureDiag');
OUT(vs, 'pose2D', 4); OUT(vs, 'speed_fwd', 5);
CPORT(vs, 'Spr_L', 1, 'Left'); CPORT(vs, 'Spr_R', 2, 'Left');
A('simulink/Ports & Subsystems/Subsystem Reference', [vs '/HiFi'], 'ReferencedSubsystem', 'structure_hifi', ...
    'VariantControl', 'VC_STRUCT_FID == 1');
A('simulink/Ports & Subsystems/Subsystem Reference', [vs '/LoFi'], 'ReferencedSubsystem', 'structure_lofi', ...
    'VariantControl', 'VC_STRUCT_FID == 2');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Implement'], 'ReferencedSubsystem', 'structure_implement');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Env'], 'ReferencedSubsystem', 'structure_env');
RT(n, 'RT_env_ovr', 'Ts.base', 'P.ic.Bus_EnvOverride');
% lines
L(n, 'env_ovr/1', 'RT_env_ovr/1', 'env_ovr');
PICK(n, 'Impl_sel', 'Implement/2', {'wrench', 'dmdt', 'E_impl'}, {'Fidelity/1', 'Fidelity/2', 'Fidelity/3'}, 'impl');
L(n, 'env_dist/1', 'Fidelity/4', 'env_dist'); L(n, 'RT_env_ovr/1', 'Fidelity/5', 'env_ovr_1k');
L(n, 'impl_cmd/1', 'Implement/1', 'impl_cmd'); L(n, 'Fidelity/5', 'Implement/2', 'speed_fwd');
L(n, 'Fidelity/4', 'Implement/3', 'pose2D');
L(n, 'Fidelity/4', 'Env/1'); L(n, 'Fidelity/5', 'Env/2'); L(n, 'env_ovr/1', 'Env/3');
L(n, 'Fidelity/1', 'truth/1', 'truth'); L(n, 'Fidelity/2', 'tracks/1', 'tracks'); L(n, 'Fidelity/3', 'diag/1', 'diag');
L(n, 'Implement/1', 'impl_mech/1', 'impl_mech'); L(n, 'Env/1', 'env/1', 'env');
PL(n, CPH(n, 'Spr_L'), shaddad_bu('pport', vs, 'Spr_L'));
PL(n, CPH(n, 'Spr_R'), shaddad_bu('pport', vs, 'Spr_R'));
PL(n, CPH(n, 'Impl_Shaft'), shaddad_bu('pport', [n '/Implement'], 'Impl_Shaft'));
set_param(n, 'Description', ['structure_plant (T03, domain top, ICD 3.2): ports Spr_L, Spr_R, Impl_Shaft (classic ' ...
    'rotational) | impl_cmd (0.01), env_dist (0.001), env_ovr (0.1) | truth, tracks, impl_mech (0.01, ZOH inside), ' ...
    'env (0.1, structure_env of T02 fed by truth pose2D and speed_fwd), diag. Fidelity Variant Subsystem VC_STRUCT_FID ' ...
    '(1 structure_hifi, 2 structure_lofi), structure_implement (VC_IMPL, VC_IMPL_DRIVE; its bus impl of wrench, ' ...
    'dmdt, E_impl picked by Impl_sel in front of Fidelity). No Solver Configuration, no enclosure thermal model, ' ...
    'no obstacle monitor (Sim_Monitors). Built by scripts/build/build_structure.m.']);
end
