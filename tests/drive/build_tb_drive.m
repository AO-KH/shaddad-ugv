function build_tb_drive(which)
%BUILD_TB_DRIVE  Generate the traction-drive test benches of build task T06 (owner: drive).
%
%   build_tb_drive()             all benches
%   build_tb_drive(which)        'tb_drive' | 'tb_drive_sys' | 'tb_drive_2mass' (cellstr allowed)
%
%   tb_drive        one side (spec 7/8): DRV_CMD frames from drive_cmd_pack -> drive_cmd_unpack -> RT 0.01 -> 0.001
%                   -> drive_side (side 1); battery = Controlled Voltage Source (tb_vsrc) + R_src + contactor
%                   Switch (tb_contactor); Electrical Reference + Solver Configuration (local Backward Euler 1 ms,
%                   fixed cost); load Variant tb_load: 1 vehicle (Inertia tb_J, rolling Rotational Friction
%                   tb_Trr, grade Ideal Torque Source tb_grade), 2 dyno (Ideal Angular Velocity Source tb_w_dyno).
%                   Feedback re-encoded by drive_fb_pack and decoded by drive_fb_unpack (fb_rx, 10 ms).
%   tb_drive_sys    drive_system with both sides (T14, T17): DRV_CMD frames for L / R, one DC source per side,
%                   inertia + rolling friction + grade torque per sprocket.
%   tb_drive_2mass  two-mass load (D50, T21/T22): drive_side commanded at 1 ms, belt-side inertia
%                   (P.drive.bench2m.J_b reflected to the sprocket minus the modelled motor/gear inertia),
%                   Wheel and Axle (r_spr), shear spring k_x / damper c = tau_d k_x, vehicle Mass m_side.
%
%   Every bench input lives in the bench model workspace (tb_* variables, defaults set here); tests override
%   them with Simulink.SimulationInput/setVariable(..., 'Workspace', <bench>). Outputs are To Workspace blocks
%   (timeseries, decimation tb_dec) returned in the SimulationOutput. Config set: a local copy of cs_SimFixed.
%   Layout (model layout spec 6 and 7): no block position is set here; every bench is laid out at every level by
%   shaddad_layout before it is saved (local_save), and every signal line is named (L5).

if nargin < 1 || isempty(which), which = {'tb_drive', 'tb_drive_sys', 'tb_drive_2mass', 'tb_drive_foc'}; end
if ischar(which), which = {which}; end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Bus_DriveSideCmd'',''var'')'), shaddad_params(); end
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')'), shaddad_configsets('fixed'); end
d = fileparts(mfilename('fullpath'));
for k = 1:numel(which)
    feval(['local_' which{k}], fullfile(d, [which{k} '.slx']));
end
end

% =============================================================================================
function p = A(lib, path, varargin)
% add_block (no position: the layout engine places every block when the bench is saved, local_save)
p = shaddad_bu('add', lib, path, varargin{:});
end
function L(sys, s, d, nm)
% signal line s -> d ('Blk/k'), named nm (L5); without nm the default of shaddad_bu('nline')
if nargin < 4, nm = ''; end
shaddad_bu('nline', sys, s, d, nm);
end
function PL(sys, a, b), add_line(sys, a, b); end   % no autorouting for physical lines
function ph = H(b), ph = get_param(b, 'PortHandles'); end
function h = CP(b, p), h = shaddad_bu('pport', b, p); end

function local_new(n, f)
if bdIsLoaded(n), close_system(n, 0); end
if isfile(f), delete(f); end
new_system(n);
cs = copy(evalin('base', 'cs_SimFixed'));
cs.Name = 'tb_config';
attachConfigSet(n, cs, true);
setActiveConfigSet(n, cs.Name);
set_param(n, 'StopTime', '1', 'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on', 'ReturnWorkspaceOutputsName', 'out');
end

function local_save(n, f, desc)
% the layout engine at every level of the bench (model layout spec 7; it keeps every physical node, and an engine
% error stops the build), then save
set_param(n, 'Description', desc);
shaddad_layout(n, 'Recursive', true);
save_system(n, f);
close_system(n, 0);
end

function mws(n, varargin)
mw = get_param(n, 'ModelWorkspace');
for k = 1:2:numel(varargin), assignin(mw, varargin{k}, varargin{k + 1}); end
end

function b = FW(sys, name, var, type, ts)
b = A('simulink/Sources/From Workspace', [sys '/' name], 'VariableName', var, 'SampleTime', ts, ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
if ~isempty(type), set_param(b, 'OutDataTypeStr', type); end
end

function b = TW(sys, name, var)
b = A('simulink/Sinks/To Workspace', [sys '/' name], 'VariableName', var, 'SaveFormat', 'Timeseries', ...
    'Decimation', 'tb_dec', 'MaxDataPoints', 'inf');
end

function b = SPS(sys, name, unit)
% Rate Transition (inherited output rate = network rate) named 'name' + Simulink-PS converter
A('simulink/Signal Attributes/Rate Transition', [sys '/' name], 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTimeOpt', 'Inherit');
b = A('nesl_utility/Simulink-PS Converter', [sys '/' name '_PS'], 'Unit', unit, 'FilteringAndDerivatives', 'zero');
L(sys, [name '/1'], [name '_PS/1']);
end

function b = MREF(sys, name)
b = A(sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference'), [sys '/' name]);
end

function sc = local_solver(n, hGnd)
sc = A(sprintf('nesl_utility/Solver\nConfiguration'), [n '/Solver']);
set_param(sc, 'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
    'LocalSolverSampleTime', 'P.sim.ssc.Ts', 'DoFixedCost', 'on', 'MaxNonlinIter', 'P.sim.ssc.MaxNonlinIter', ...
    'MaxModeIter', 'P.sim.ssc.MaxModeIter');
PL(n, H(sc).RConn(1), hGnd);
end

function ts = ts_const(v, type)
if strcmp(type, 'boolean'), ts = timeseries(logical(v), 0); else, ts = timeseries(cast(v, type), 0); end
end

function S = bus_ts(busName, vals)
S = tb_drive_bus_ts(busName, 0, vals);
end

% =============================================================================================
% vehicle / dyno load variant on one sprocket port
% =============================================================================================
function local_load(n, name, suffix)
vs = [n '/' name];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
c = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
c = setdiff(c, {vs});
lns = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lns), delete_line(lns); end
for k = 1:numel(c), delete_block(c{k}); end
A('nesl_utility/Connection Port', [vs '/Spr'], 'Port', '1', 'Side', 'Left', ...
    'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
% choice 1: vehicle-equivalent load
v = [vs '/Vehicle'];
add_block('simulink/Ports & Subsystems/Subsystem', v);
delete_line(find_system(v, 'FindAll', 'on', 'Type', 'line')); delete_block([v '/In1']); delete_block([v '/Out1']);
A('nesl_utility/Connection Port', [v '/Spr'], 'Port', '1', 'Side', 'Left', ...
    'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
j = A('fl_lib/Mechanical/Rotational Elements/Inertia', [v '/J_load'], 'inertia', ['tb_J' suffix], ...
    'num_ports', 'foundation.enum.numPorts2.one');
fr = A('fl_lib/Mechanical/Rotational Elements/Rotational Friction', [v '/Rolling'], 'brkwy_trq', ['1.0001*tb_Trr' suffix], ...
    'Col_trq', ['tb_Trr' suffix], 'brkwy_vel', 'P.drive.bench.w_brk_rr', 'visc_coef', '0');
tq = A('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', [v '/Grade']);
g = MREF(v, 'Ground');
FW(v, 'GradeTorque', ['tb_grade' suffix], '', 'P.drive.Ts_esc');
s = SPS(v, 'GradeSPS', 'N*m');
L(v, 'GradeTorque/1', 'GradeSPS/1');
hS = H([v '/Spr']); hS = hS.RConn(1);
PL(v, hS, H(j).LConn(1)); PL(v, hS, H(fr).LConn(1)); PL(v, H(fr).RConn(1), H(g).LConn(1));
PL(v, hS, H(tq).LConn(1)); PL(v, H(tq).RConn(2), H(g).LConn(1)); PL(v, H(s).RConn(1), H(tq).RConn(1));
set_param(v, 'VariantControl', 'tb_load == 1');
% choice 2: dyno (ideal speed source; 0 = locked sprocket)
y = [vs '/Dyno'];
add_block('simulink/Ports & Subsystems/Subsystem', y);
delete_line(find_system(y, 'FindAll', 'on', 'Type', 'line')); delete_block([y '/In1']); delete_block([y '/Out1']);
A('nesl_utility/Connection Port', [y '/Spr'], 'Port', '1', 'Side', 'Left', ...
    'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
ws = A(sprintf('fl_lib/Mechanical/Mechanical Sources/Ideal Angular\nVelocity Source'), [y '/Speed']);
g = MREF(y, 'Ground');
FW(y, 'DynoSpeed', ['tb_w_dyno' suffix], '', 'P.drive.Ts_esc');
s = SPS(y, 'DynoSPS', 'rad/s');
L(y, 'DynoSpeed/1', 'DynoSPS/1');
hS = H([y '/Spr']); hS = hS.RConn(1);
PL(y, hS, H(ws).LConn(1)); PL(y, H(ws).RConn(2), H(g).LConn(1)); PL(y, H(s).RConn(1), H(ws).RConn(1));
set_param(y, 'VariantControl', 'tb_load == 2');
set_param(vs, 'VariantControlMode', 'expression', 'VariantActivationTime', 'update diagram');
end

function [hp, hn] = local_battery(n, sfx, withContactor)
% Controlled Voltage Source (tb_vsrc) + series resistance (+ contactor switch); returns DC+ / DC- handles
vs = A('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', [n '/Battery' sfx]);
r = A('fl_lib/Electrical/Electrical Elements/Resistor', [n '/R_src' sfx], 'R', 'tb_Rsrc');
FW(n, ['Vsrc' sfx], 'tb_vsrc', '', 'P.drive.Ts_esc');
s = SPS(n, ['VsrcSPS' sfx], 'V');
L(n, ['Vsrc' sfx '/1'], ['VsrcSPS' sfx '/1']);
PL(n, H(s).RConn(1), H(vs).RConn(1));
PL(n, H(vs).LConn(1), H(r).LConn(1));
hp = H(r).RConn(1); hn = H(vs).RConn(2);
if withContactor
    sw = A('fl_lib/Electrical/Electrical Elements/Switch', [n '/Contactor' sfx], 'R_closed', '1e-4', 'G_open', '1e-8', ...
        'Threshold', '0.5');
    FW(n, ['ContactorCmd' sfx], 'tb_contactor', '', 'P.drive.Ts_esc');
    c = SPS(n, ['ContactorSPS' sfx], '1');
    L(n, ['ContactorCmd' sfx '/1'], ['ContactorSPS' sfx '/1']);
    PL(n, H(c).RConn(1), H(sw).RConn(1));
    PL(n, hp, H(sw).LConn(1)); hp = H(sw).RConn(2);
end
end

function local_defaults(n, extra)
% bench inputs (model workspace): constant defaults, overridden per test
P = evalin('base', 'P');
cmd = bus_ts('Bus_DriveSideCmd', struct('mode', 1, 'tau_lim_mot', 16, 'tau_lim_reg', 16));
fi = bus_ts('Bus_DriveFaultInj', struct('L', struct('tau_scale', 1), 'R', struct('tau_scale', 1)));
mws(n, 'tb_cmd', cmd, 'tb_sto', ts_const(true, 'boolean'), 'tb_fi', fi, 'tb_T_amb', P.drive.thermal.T_amb0, ...
    'tb_vsrc', timeseries(P.drive.bench.V_src, 0), 'tb_Rsrc', P.drive.bench.R_src, 'tb_contactor', timeseries(1, 0), ...
    'tb_load', 1, 'tb_dec', 1, 'tb_drop', ts_const(false, 'boolean'));
for k = 1:2:numel(extra), mws(n, extra{k}, extra{k + 1}); end
end

% =============================================================================================
function local_tb_drive(f)
n = 'tb_drive'; local_new(n, f);
P = evalin('base', 'P');
ld = tb_drive_load(n, P);                    % bench load of this P (tb_drive_run re-sets it per run)
local_defaults(n, {'tb_J', ld.tb_J, 'tb_Trr', ld.tb_Trr, 'tb_grade', timeseries(0, 0), 'tb_w_dyno', timeseries(0, 0)});
% command path: frames as on CAN1
FW(n, 'Cmd', 'tb_cmd', 'Bus: Bus_DriveSideCmd', 'P.drive.Ts_can');
A('simulink/Sources/Constant', [n '/Node'], 'Value', 'P.drive.esc.node_id(1)', 'OutDataTypeStr', 'uint8');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/CmdPack'], 'ReferencedSubsystem', 'drive_cmd_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/CmdUnpack'], 'ReferencedSubsystem', 'drive_cmd_unpack');
RT = 'simulink/Signal Attributes/Rate Transition';
A(RT, [n '/RT_cmd'], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'P.drive.Ts_esc');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Drive'], 'ReferencedSubsystem', 'drive_side');
set_param([n '/Drive'], 'side', '1');
FW(n, 'Sto', 'tb_sto', 'boolean', 'P.drive.Ts_esc');
A('simulink/Sources/Constant', [n '/T_amb'], 'Value', 'tb_T_amb', 'SampleTime', 'P.drive.Ts_esc');
FW(n, 'Faults', 'tb_fi', 'Bus: Bus_DriveFaultInj', 'P.drive.Ts_esc');
L(n, 'Cmd/1', 'CmdPack/1', 'cmd_tx'); L(n, 'Node/1', 'CmdPack/2');
% frame drops (T18): valid = valid AND NOT tb_drop
FW(n, 'Drop', 'tb_drop', 'boolean', 'P.drive.Ts_can');
% each consumer picks its frame fields with a Bus Selector right in front of it (model layout spec 6.2)
A('simulink/Signal Routing/Bus Selector', [n '/FrameSel'], 'OutputSignals', 'id,len,data');
A('simulink/Signal Routing/Bus Selector', [n '/ValidSel'], 'OutputSignals', 'valid');
A('simulink/Logic and Bit Operations/Logical Operator', [n '/NotDrop'], 'Operator', 'NOT', 'Inputs', '1');
A('simulink/Logic and Bit Operations/Logical Operator', [n '/Kept'], 'Operator', 'AND', 'Inputs', '2');
A('simulink/Signal Routing/Bus Creator', [n '/FrameOut'], 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', 'NonVirtualBus', 'on');
L(n, 'CmdPack/1', 'FrameSel/1', 'drv_cmd'); L(n, 'CmdPack/1', 'ValidSel/1');
L(n, 'Drop/1', 'NotDrop/1'); L(n, 'ValidSel/1', 'Kept/1'); L(n, 'NotDrop/1', 'Kept/2');
L(n, 'FrameSel/1', 'FrameOut/1', 'id'); L(n, 'FrameSel/2', 'FrameOut/2', 'len'); L(n, 'FrameSel/3', 'FrameOut/3', 'data');
L(n, 'Kept/1', 'FrameOut/4', 'valid');
L(n, 'FrameOut/1', 'CmdUnpack/1', 'drv_cmd_rx'); L(n, 'Node/1', 'CmdUnpack/2');
L(n, 'CmdUnpack/1', 'RT_cmd/1', 'cmd_rx'); L(n, 'RT_cmd/1', 'Drive/1');
L(n, 'Sto/1', 'Drive/2', 'sto_ok'); L(n, 'T_amb/1', 'Drive/3'); L(n, 'Faults/1', 'Drive/4', 'fault_inj');
% feedback: 1 ms bus, frames, decoded at 10 ms
A(RT, [n '/RT_fb'], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'P.drive.Ts_can');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/FbPack'], 'ReferencedSubsystem', 'drive_fb_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/FbUnpack'], 'ReferencedSubsystem', 'drive_fb_unpack');
L(n, 'Drive/1', 'RT_fb/1', 'fb'); L(n, 'RT_fb/1', 'FbPack/1'); L(n, 'Node/1', 'FbPack/2');
L(n, 'FbPack/1', 'FbUnpack/1', 'fb1'); L(n, 'FbPack/2', 'FbUnpack/2', 'fb2'); L(n, 'FbPack/3', 'FbUnpack/3', 'fb3');
L(n, 'Node/1', 'FbUnpack/4');
TW(n, 'Log_fb', 'fb'); TW(n, 'Log_truth', 'truth'); TW(n, 'Log_fb_rx', 'fb_rx'); TW(n, 'Log_cmd', 'cmd_rx');
L(n, 'Drive/1', 'Log_fb/1'); L(n, 'Drive/2', 'Log_truth/1', 'truth'); L(n, 'FbUnpack/1', 'Log_fb_rx/1', 'fb_rx');
L(n, 'RT_cmd/1', 'Log_cmd/1');
% electrical
[hp, hn] = local_battery(n, '', true);
gnd = A('fl_lib/Electrical/Electrical Elements/Electrical Reference', [n '/GND']);
PL(n, hn, H(gnd).LConn(1));
PL(n, hp, CP([n '/Drive'], 'DC_p')); PL(n, CP([n '/Drive'], 'DC_n'), H(gnd).LConn(1));
local_solver(n, H(gnd).LConn(1));
% mechanical load
local_load(n, 'Load', '');
PL(n, CP([n '/Drive'], 'Spr'), CP([n '/Load'], 'Spr'));
local_save(n, f, ['T06 one-side traction drive bench (drive spec 7/8, build plan T06.8). Inputs in the model workspace: ' ...
    'tb_cmd (Bus_DriveSideCmd timeseries, 10 ms), tb_sto, tb_fi (Bus_DriveFaultInj), tb_T_amb, tb_vsrc, tb_Rsrc, ' ...
    'tb_contactor, tb_load (1 vehicle: tb_J, tb_Trr, tb_grade; 2 dyno: tb_w_dyno), tb_dec. Outputs: fb, truth, fb_rx, cmd_rx.']);
end

% =============================================================================================
function local_tb_drive_foc(f)
% FOC bench (spec 3.7, T15, D52): tb_drive with the drive side rebuilt around drive_plant_foc; plant solver
% P.drive.Ts_foc_plant (5e-5 s), FOC current control P.drive.Ts_foc (1e-4 s), ESC 1 ms. Bench-only rates.
n = 'tb_drive_foc'; local_new(n, f);
P = evalin('base', 'P');
set_param(n, 'FixedStep', 'P.drive.Ts_foc_plant');
ld = tb_drive_load(n, P);                    % bench load of this P (tb_drive_run re-sets it per run)
local_defaults(n, {'tb_J', ld.tb_J, 'tb_Trr', ld.tb_Trr, 'tb_grade', timeseries(0, 0), 'tb_w_dyno', timeseries(0, 0)});
FW(n, 'Cmd', 'tb_cmd', 'Bus: Bus_DriveSideCmd', 'P.drive.Ts_can');
A('simulink/Sources/Constant', [n '/Node'], 'Value', 'P.drive.esc.node_id(1)', 'OutDataTypeStr', 'uint8');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/CmdPack'], 'ReferencedSubsystem', 'drive_cmd_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/CmdUnpack'], 'ReferencedSubsystem', 'drive_cmd_unpack');
RT = 'simulink/Signal Attributes/Rate Transition';
A(RT, [n '/RT_cmd'], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'P.drive.Ts_esc');
% drive side with the FOC plant (same contents as drive_side, built by build_drive)
dr = [n '/Drive'];
add_block('simulink/Ports & Subsystems/Subsystem', dr);
delete_line(find_system(dr, 'FindAll', 'on', 'Type', 'line')); delete_block([dr '/In1']); delete_block([dr '/Out1']);
build_drive('side_foc_body', dr);
mo = Simulink.Mask.create(dr);
mo.addParameter('Name', 'side', 'Prompt', 'Side', 'Value', '1', 'Type', 'edit');
FW(n, 'Sto', 'tb_sto', 'boolean', 'P.drive.Ts_esc');
A('simulink/Sources/Constant', [n '/T_amb'], 'Value', 'tb_T_amb', 'SampleTime', 'P.drive.Ts_esc');
FW(n, 'Faults', 'tb_fi', 'Bus: Bus_DriveFaultInj', 'P.drive.Ts_esc');
L(n, 'Cmd/1', 'CmdPack/1', 'cmd_tx'); L(n, 'Node/1', 'CmdPack/2');
L(n, 'CmdPack/1', 'CmdUnpack/1', 'drv_cmd'); L(n, 'Node/1', 'CmdUnpack/2');
L(n, 'CmdUnpack/1', 'RT_cmd/1', 'cmd_rx'); L(n, 'RT_cmd/1', 'Drive/1');
L(n, 'Sto/1', 'Drive/2', 'sto_ok'); L(n, 'T_amb/1', 'Drive/3'); L(n, 'Faults/1', 'Drive/4', 'fault_inj');
TW(n, 'Log_fb', 'fb'); TW(n, 'Log_truth', 'truth'); TW(n, 'Log_cmd', 'cmd_rx');
L(n, 'Drive/1', 'Log_fb/1', 'fb'); L(n, 'Drive/2', 'Log_truth/1', 'truth'); L(n, 'RT_cmd/1', 'Log_cmd/1');
[hp, hn] = local_battery(n, '', true);
gnd = A('fl_lib/Electrical/Electrical Elements/Electrical Reference', [n '/GND']);
PL(n, hn, H(gnd).LConn(1));
PL(n, hp, CP(dr, 'DC_p')); PL(n, CP(dr, 'DC_n'), H(gnd).LConn(1));
sc = local_solver(n, H(gnd).LConn(1));
set_param(sc, 'LocalSolverSampleTime', 'P.drive.Ts_foc_plant');
local_load(n, 'Load', '');
PL(n, CP(dr, 'Spr'), CP([n '/Load'], 'Spr'));
local_save(n, f, ['T06 FOC bench (BENCH-ONLY, D52, T15): tb_drive with drive_plant_foc (PMSM + average-value inverter + ' ...
    'current control at 1e-4 s), plant solver 5e-5 s. Same inputs / outputs as tb_drive.']);
end

% =============================================================================================
function local_tb_drive_sys(f)
n = 'tb_drive_sys'; local_new(n, f);
P = evalin('base', 'P');
ld = tb_drive_load(n, P);                    % bench load of this P (tb_drive_run re-sets it per run)
local_defaults(n, {'tb_cmdL', evalin('base', 'P.ic.Bus_DriveSideCmd'), 'tb_J', ld.tb_J, 'tb_Trr', ld.tb_Trr, ...
    'tb_gradeL', timeseries(0, 0), 'tb_gradeR', timeseries(0, 0)});
mw = get_param(n, 'ModelWorkspace');
c = evalin(mw, 'tb_cmd'); assignin(mw, 'tb_cmdL', c); assignin(mw, 'tb_cmdR', c);
RT = 'simulink/Signal Attributes/Rate Transition';
FW(n, 'CmdL', 'tb_cmdL', 'Bus: Bus_DriveSideCmd', 'P.drive.Ts_can');
FW(n, 'CmdR', 'tb_cmdR', 'Bus: Bus_DriveSideCmd', 'P.drive.Ts_can');
A('simulink/Sources/Constant', [n '/NodeL'], 'Value', 'P.drive.esc.node_id(1)', 'OutDataTypeStr', 'uint8');
A('simulink/Sources/Constant', [n '/NodeR'], 'Value', 'P.drive.esc.node_id(2)', 'OutDataTypeStr', 'uint8');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/PackL'], 'ReferencedSubsystem', 'drive_cmd_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/PackR'], 'ReferencedSubsystem', 'drive_cmd_pack');
A('simulink/Signal Routing/Bus Creator', [n '/CanTx'], 'Inputs', '2', 'OutDataTypeStr', 'Bus: Bus_Can1_McuFast', 'NonVirtualBus', 'on');
L(n, 'CmdL/1', 'PackL/1'); L(n, 'NodeL/1', 'PackL/2'); L(n, 'CmdR/1', 'PackR/1'); L(n, 'NodeR/1', 'PackR/2');
L(n, 'PackL/1', 'CanTx/1', 'drv_cmd_L'); L(n, 'PackR/1', 'CanTx/2', 'drv_cmd_R');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Drive'], 'ReferencedSubsystem', 'drive_system');
FW(n, 'Sto', 'tb_sto', 'boolean', 'P.drive.Ts_esc');
A('simulink/Sources/Constant', [n '/T_amb'], 'Value', 'tb_T_amb', 'SampleTime', 'Ts.hk');
FW(n, 'Faults', 'tb_fi', 'Bus: Bus_DriveFaultInj', 'P.drive.Ts_esc');
L(n, 'CanTx/1', 'Drive/1', 'can_mcu_fast'); L(n, 'Sto/1', 'Drive/2', 'sto_ok'); L(n, 'T_amb/1', 'Drive/3', 'T_amb');
L(n, 'Faults/1', 'Drive/4', 'fault_inj');
% feedback decoding as in the MCU (FB2 re-timed 0.05 -> 0.01); each decoder picks its frames with Bus Selectors
% right in front of it (model layout spec 6.2)
for s = {'L', 'R'}
    sd = s{1};
    A(RT, [n '/RT_fb2_' sd], 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'P.drive.Ts_can');
    A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Unpack' sd], 'ReferencedSubsystem', 'drive_fb_unpack');
    A('simulink/Signal Routing/Bus Selector', [n '/FastSel_' sd], 'OutputSignals', ['fb1_' sd ',fb3_' sd]);
    A('simulink/Signal Routing/Bus Selector', [n '/SlowSel_' sd], 'OutputSignals', ['fb2_' sd]);
    L(n, 'Drive/1', ['FastSel_' sd '/1'], 'can_esc_fast'); L(n, 'Drive/2', ['SlowSel_' sd '/1'], 'can_esc_slow');
    L(n, ['SlowSel_' sd '/1'], ['RT_fb2_' sd '/1']);
    L(n, ['FastSel_' sd '/1'], ['Unpack' sd '/1']); L(n, ['RT_fb2_' sd '/1'], ['Unpack' sd '/2'], ['fb2_' sd '_10ms']);
    L(n, ['FastSel_' sd '/2'], ['Unpack' sd '/3']); L(n, ['Node' sd '/1'], ['Unpack' sd '/4']);
    TW(n, ['Log_fb' sd], ['fb' sd]);
    L(n, ['Unpack' sd '/1'], ['Log_fb' sd '/1'], ['fb_rx_' sd]);
end
TW(n, 'Log_truth', 'truth');
L(n, 'Drive/3', 'Log_truth/1', 'drive_truth');
% electrical: one source per side, common reference
gnd = A('fl_lib/Electrical/Electrical Elements/Electrical Reference', [n '/GND']);
for s = {'L', 'R'}
    sd = s{1};
    [hp, hn] = local_battery(n, sd, false);
    PL(n, hn, H(gnd).LConn(1));
    PL(n, hp, CP([n '/Drive'], ['DC48_Trac' sd '_p'])); PL(n, CP([n '/Drive'], ['DC48_Trac' sd '_n']), H(gnd).LConn(1));
end
local_solver(n, H(gnd).LConn(1));
% sprocket loads (vehicle-equivalent per side)
for s = {'L', 'R'}
    sd = s{1};
    v = [n '/Load' sd];
    add_block('simulink/Ports & Subsystems/Subsystem', v);
    delete_line(find_system(v, 'FindAll', 'on', 'Type', 'line')); delete_block([v '/In1']); delete_block([v '/Out1']);
    A('nesl_utility/Connection Port', [v '/Spr'], 'Port', '1', 'Side', 'Left', ...
        'ConnectionType', 'Connection: foundation.mechanical.rotational.rotational');
    j = A('fl_lib/Mechanical/Rotational Elements/Inertia', [v '/J_load'], 'inertia', 'tb_J', 'num_ports', 'foundation.enum.numPorts2.one');
    fr = A('fl_lib/Mechanical/Rotational Elements/Rotational Friction', [v '/Rolling'], 'brkwy_trq', '1.0001*tb_Trr', ...
        'Col_trq', 'tb_Trr', 'brkwy_vel', 'P.drive.bench.w_brk_rr', 'visc_coef', '0');
    tq = A('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', [v '/Grade']);
    g = MREF(v, 'Ground');
    FW(v, 'GradeTorque', ['tb_grade' sd], '', 'P.drive.Ts_esc');
    sp = SPS(v, 'GradeSPS', 'N*m');
    L(v, 'GradeTorque/1', 'GradeSPS/1');
    hS = H([v '/Spr']); hS = hS.RConn(1);
    PL(v, hS, H(j).LConn(1)); PL(v, hS, H(fr).LConn(1)); PL(v, H(fr).RConn(1), H(g).LConn(1));
    PL(v, hS, H(tq).LConn(1)); PL(v, H(tq).RConn(2), H(g).LConn(1)); PL(v, H(sp).RConn(1), H(tq).RConn(1));
    PL(n, CP([n '/Drive'], ['Spr_' sd]), CP(v, 'Spr'));
end
local_save(n, f, ['T06 two-side bench of drive_system (T14 sign / side independence, T17 1 h duty speed). Inputs: tb_cmdL, ' ...
    'tb_cmdR, tb_sto, tb_fi, tb_T_amb, tb_vsrc, tb_Rsrc, tb_J, tb_Trr, tb_gradeL/R, tb_dec. Outputs: fbL, fbR (decoded at ' ...
    '10 ms), truth (Bus_DriveTruth).']);
end

% =============================================================================================
function local_tb_drive_2mass(f)
n = 'tb_drive_2mass'; local_new(n, f);
P = evalin('base', 'P');
b2 = P.drive.bench2m;
ld = tb_drive_load(n, P);                    % belt-side inertia at the sprocket not in drive_side
local_defaults(n, {'tb_Jb_spr', ld.tb_Jb_spr, 'tb_kx', b2.k_x(1), 'tb_cx', b2.tau_d*b2.k_x(1), 'tb_m', b2.m_side(1)});
FW(n, 'Cmd', 'tb_cmd', 'Bus: Bus_DriveSideCmd', 'P.drive.Ts_esc');
A('simulink/Ports & Subsystems/Subsystem Reference', [n '/Drive'], 'ReferencedSubsystem', 'drive_side');
set_param([n '/Drive'], 'side', '1');
FW(n, 'Sto', 'tb_sto', 'boolean', 'P.drive.Ts_esc');
A('simulink/Sources/Constant', [n '/T_amb'], 'Value', 'tb_T_amb', 'SampleTime', 'P.drive.Ts_esc');
FW(n, 'Faults', 'tb_fi', 'Bus: Bus_DriveFaultInj', 'P.drive.Ts_esc');
L(n, 'Cmd/1', 'Drive/1', 'cmd'); L(n, 'Sto/1', 'Drive/2', 'sto_ok'); L(n, 'T_amb/1', 'Drive/3'); L(n, 'Faults/1', 'Drive/4', 'fault_inj');
TW(n, 'Log_fb', 'fb'); TW(n, 'Log_truth', 'truth');
L(n, 'Drive/1', 'Log_fb/1', 'fb'); L(n, 'Drive/2', 'Log_truth/1', 'truth');
[hp, hn] = local_battery(n, '', false);
gnd = A('fl_lib/Electrical/Electrical Elements/Electrical Reference', [n '/GND']);
PL(n, hn, H(gnd).LConn(1));
PL(n, hp, CP([n '/Drive'], 'DC_p')); PL(n, CP([n '/Drive'], 'DC_n'), H(gnd).LConn(1));
local_solver(n, H(gnd).LConn(1));
% two-mass load: belt side tb_Jb_spr on the sprocket, Wheel and Axle r_spr, shear spring / damper, vehicle mass
j = A('fl_lib/Mechanical/Rotational Elements/Inertia', [n '/J_belt'], 'inertia', 'tb_Jb_spr', 'num_ports', 'foundation.enum.numPorts2.one');
wa = A('fl_lib/Mechanical/Mechanisms/Wheel and Axle', [n '/Sprocket'], 'R', 'P.drive.gear.r_spr');
sp = A('fl_lib/Mechanical/Translational Elements/Translational Spring', [n '/ShearSpring'], 'spr_rate', 'tb_kx');
dm = A('fl_lib/Mechanical/Translational Elements/Translational Damper', [n '/ShearDamper'], 'D', 'tb_cx');
ms = A('fl_lib/Mechanical/Translational Elements/Mass', [n '/Vehicle'], 'mass', 'tb_m');
vs = A(sprintf('fl_lib/Mechanical/Mechanical Sensors/Ideal Translational\nMotion Sensor'), [n '/VehicleSpeed']);
tr = A(sprintf('fl_lib/Mechanical/Translational Elements/Mechanical\nTranslational Reference'), [n '/Ground']);
hS = CP([n '/Drive'], 'Spr');
PL(n, hS, H(j).LConn(1)); PL(n, hS, H(wa).LConn(1));
hP = H(wa).RConn(1); hM = H(ms).LConn(1);
PL(n, hP, H(sp).LConn(1)); PL(n, H(sp).RConn(1), hM);
PL(n, hP, H(dm).LConn(1)); PL(n, H(dm).RConn(1), hM);
PL(n, hM, H(vs).LConn(1)); PL(n, H(vs).RConn(1), H(tr).LConn(1));
c = A('nesl_utility/PS-Simulink Converter', [n '/v_veh_PS'], 'Unit', 'm/s');
PL(n, H(vs).RConn(2), H(c).LConn(1));
% the sensor's position output is not used: a PS Terminator (no equation) ends it, so that no port is left open
pt = A('fl_lib/Physical Signals/Sinks/PS Terminator', [n '/Term_x']); PL(n, H(vs).RConn(3), H(pt).LConn(1));
A('simulink/Discrete/Zero-Order Hold', [n '/ZOH_v'], 'SampleTime', 'P.drive.Ts_esc');
TW(n, 'Log_v', 'v_veh');
L(n, 'v_veh_PS/1', 'ZOH_v/1', 'v_veh_cont'); L(n, 'ZOH_v/1', 'Log_v/1', 'v_veh');
local_save(n, f, ['T06 two-mass bench (D50, T21/T22): drive_side commanded at 1 ms (tb_cmd), belt-side inertia tb_Jb_spr at the ' ...
    'sprocket, Wheel and Axle r_spr, shear spring tb_kx / damper tb_cx, vehicle mass tb_m. Outputs: fb, truth, v_veh.']);
end
