function built = build_tb_power_plant(which)
%BUILD_TB_POWER_PLANT  T05 test benches (owner: power, T05).
%
%   build_tb_power_plant()          build all benches
%   build_tb_power_plant(names)     'tb_power_plant' | 'tb_shift_cycle' | 'tb_lv_rails' | 'tb_impl_codec'
%
%   tests/power/tb_power_plant.slx (build plan T05.7): Power_Plant + per-side DRIVE stub (ee_lib Capacitor
%     TB.C_drv with ESR TB.R_esr parallel to a Controlled Current Source profile, gated by the 1 ms-delayed port
%     voltage, D36; switchable short TB.R_short across DC48_TracL) + implement load stub on Impl_Shaft (Inertia +
%     Ideal Torque Source, speed through a 1 ms Unit Delay) + Model block Shaddad_BMS + MCU stimuli (hw_cmd incl.
%     the DO0 / DO1 re-arm pattern, BMS_REQ via bmsreq_can_pack, IMPL_CMD via implcmd_can_pack) + operator / fault /
%     LV-demand / environment stimuli + Solver Configuration (local Backward Euler P.sim.ssc.Ts, fixed cost; no extra
%     Electrical Reference: the only one is in Power_Plant/PDU). Stimulus = model-workspace TB
%     (tests/power/tb_power_plant_default.m).
%   tests/power/tb_shift_cycle.slx (T05.8, T-PWR-20): the same bench with the 24 h shift-cycle defaults
%     (tb_power_plant_default('shift'), DRIVE stubs as DC power loads, 1 s logs); run with VC_PACK = 2,
%     VC_CHG = 1, VC_IMPL = 0 and the 5 ms mission rate (tests/power/pp_case_run).
%   tests/power/tb_lv_rails.slx: LV_Distribution alone, AUX48 from a Controlled Voltage Source (TB.vaux_*),
%     Electrical Reference + Solver Configuration, rail-load / coil stimuli (T-PWR-05, rail OK flags, UVLO); the
%     fan command and the coil word go in as the buses thermal and safety that LV_Distribution picks from.
%   tests/power/tb_impl_codec.slx: implcmd / implfb pack -> unpack round trip from workspace samples.
%   Logs: log_fast (Ts.base, TB.dec_fast) and log_slow (Ts.ctrl, TB.dec_slow) To Workspace arrays (columns in
%   pp_log_cols), signal logging telem (BMS), power_truth (Power_Plant), ecu_w_target / ecu_enable (impl_ecu).
%   Every signal line is named; the layout engine places each bench (shaddad_layout) before it is saved.

all = {'tb_power_plant', 'tb_shift_cycle', 'tb_lv_rails', 'tb_impl_codec'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
built = {};
for k = 1:numel(which)
    nm = which{k};
    f = fullfile(root, 'tests', 'power', [nm '.slx']);
    switch nm
        case 'tb_power_plant', local_bench(nm, f, tb_power_plant_default());
        case 'tb_shift_cycle', local_bench(nm, f, tb_power_plant_default('shift'));
        case 'tb_lv_rails',    local_lv(nm, f);
        case 'tb_impl_codec',  local_codec(nm, f);
        otherwise, error('build_tb_power_plant:name', 'Unknown bench %s', nm);
    end
    built{end+1} = nm; %#ok<AGROW>
end
end

% -------------------------------------------------------------------------------------------
function p = A(sys, lib, n, varargin)
% add_block (no position: the layout engine places every block when the bench is saved, local_save)
p = shaddad_bu('add', lib, [sys '/' n], varargin{:});
end
function local_save(mdl, f)
% the layout engine at every level of the bench (an engine error stops the build), the physical net check, save
shaddad_layout(mdl, 'Recursive', true);
bad = {};
b = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
for k = 1:numel(b)
    pc = get_param(b{k}, 'PortConnectivity');
    me = get_param(b{k}, 'Handle');
    for j = 1:numel(pc)
        if ~(startsWith(pc(j).Type, 'LConn') || startsWith(pc(j).Type, 'RConn')), continue, end
        h = [pc(j).DstBlock(:); pc(j).SrcBlock(:)]; h = h(h > 0);
        if isempty(h) || all(h == me), bad{end+1} = sprintf('  %s %s', b{k}, pc(j).Type); end %#ok<AGROW>
    end
end
if ~isempty(bad)
    error('build_tb_power_plant:net', '%s: physical connection problem:\n%s', mdl, strjoin(bad, newline));
end
save_system(mdl, f);
close_system(mdl, 0);
end
function L(sys, s, d, nm)
% signal line s -> d ('Blk/k'), named nm (model layout spec 6.1, L5); without nm the default name of
% shaddad_bu('nline')
if nargin < 4, nm = ''; end
shaddad_bu('nline', sys, s, d, nm);
end
function PL(sys, a, b)
add_line(sys, a, b, 'autorouting', 'smart');
end
function ph = PH(blk)
ph = get_param(blk, 'PortHandles');
end
function RT(sys, n, ts)
% Rate Transition, 30 px tall like the conversions and comparisons it feeds, so their chains run straight
A(sys, 'simulink/Signal Attributes/Rate Transition', n, 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', ts, ...
    'Position', [0 0 40 30]);
end
function o = TAB(sys, n, clk, field)
% Flat / clip 1-D lookup of TB.<field>_t / _v driven by clock clk ('Blk/1', its line t_<clock>); returns 'n/1'. The
% table stands 30 px tall like the comparison or Bus Creator input it feeds, so that its chain runs straight (the
% height is graphical)
A(sys, 'simulink/Lookup Tables/1-D Lookup Table', n, 'BreakpointsForDimension1', ['TB.' field '_t'], ...
    'Table', ['TB.' field '_v'], 'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip', 'Position', [0 0 40 30]);
L(sys, clk, [n '/1'], ['t_' extractAfter(extractBefore(clk, '/'), 'Clock_')]);
o = [n '/1'];
end
function o = TABB(sys, n, clk, field)
% boolean version (value ~= 0)
TAB(sys, [n '_tab'], clk, field);
A(sys, 'simulink/Logic and Bit Operations/Compare To Zero', n, 'relop', '~=', 'OutDataTypeStr', 'boolean', ...
    'Position', [0 0 40 30]);
L(sys, [n '_tab/1'], [n '/1'], [n '_tab']);
o = [n '/1'];
end
function BC(sys, n, bus, el)
A(sys, 'simulink/Signal Routing/Bus Creator', n, 'Inputs', num2str(size(el, 1)), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:size(el, 1)
    L(sys, el{k, 2}, [n '/' num2str(k)], el{k, 1});
end
end
function c = CLK(sys, orig, group)
% a copy of the Digital Clock orig (same parameters and sample time) for one group of stimulus tables, named
% <orig>_<group>; returns 'n/1' (fix round 1, ruling F-R3: in a bench a clock may be split per consumer group;
% the original keeps its name and feeds the log's time column)
n = [orig '_' group];
add_block([sys '/' orig], [sys '/' n]);
c = [n '/1'];
end
function BCV(sys, n, el)
% a virtual Bus Creator n next to its producers, rows {elementName, 'Blk/port'} (shaddad_bu('vbus'))
shaddad_bu('vbus', sys, n, el);
end
function o = PSS(sys, n, hPS, unit)
b = A(sys, sprintf('nesl_utility/PS-Simulink\nConverter'), n, 'Unit', unit);
pb = PH(b); PL(sys, hPS, pb.LConn(1));
o = [n '/1'];
end
function h = SPS(sys, n, src, unit, nm)
if nargin < 5, nm = ''; end
b = A(sys, sprintf('nesl_utility/Simulink-PS\nConverter'), n, 'Unit', unit, 'FilteringAndDerivatives', 'zero');
L(sys, src, [n '/1'], nm);
pb = PH(b); h = pb.RConn(1);
end
function local_new(mdl, f, TB)
shaddad_bu('fresh', mdl, f, 'model');
shaddad_configsets('attach', mdl);
hws = get_param(mdl, 'ModelWorkspace');
assignin(hws, 'TB', TB);
end
function local_solver(mdl, h)
b = A(mdl, sprintf('nesl_utility/Solver\nConfiguration'), 'Solver', 'UseLocalSolver', 'on', ...
    'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', 'LocalSolverSampleTime', 'P.sim.ssc.Ts', 'DoFixedCost', 'on', ...
    'MaxNonlinIter', 'TB.MaxNonlinIter', 'MaxModeIter', 'TB.MaxModeIter');
pb = PH(b); PL(mdl, pb.RConn(1), h);
end
function local_log(mdl, mux, pre, srcs, names, sink, var, varargin)
% log mux: each source through a double conversion <pre>k into input k of mux, then the To Workspace sink;
% names{k} the signal of column k (its conversion's output line <name>_dbl; a clock's own line t_<clock>)
A(mdl, 'simulink/Signal Routing/Mux', mux, 'Inputs', num2str(numel(srcs)));
for k = 1:numel(srcs)
    dt = sprintf('%s%d', pre, k);
    A(mdl, 'simulink/Signal Attributes/Data Type Conversion', dt, 'OutDataTypeStr', 'double');
    nm = '';
    if startsWith(srcs{k}, 'Clock_'), nm = ['t_' extractAfter(extractBefore(srcs{k}, '/'), 'Clock_')]; end
    L(mdl, srcs{k}, [dt '/1'], nm);
    L(mdl, [dt '/1'], sprintf('%s/%d', mux, k), [names{k} '_dbl']);
end
A(mdl, 'simulink/Sinks/To Workspace', sink, 'VariableName', var, 'SaveFormat', 'Array', varargin{:}, ...
    'SampleTime', '-1', 'MaxDataPoints', 'inf');
L(mdl, [mux '/1'], [sink '/1'], var);
end

% ===========================================================================================
%  tb_power_plant / tb_shift_cycle
% ===========================================================================================
function local_bench(mdl, f, TB)
local_new(mdl, f, TB);
S = @(varargin) A(mdl, varargin{:});
Ln = @(varargin) L(mdl, varargin{:});
S('simulink/Sources/Digital Clock', 'Clock_base', 'SampleTime', 'Ts.base');
S('simulink/Sources/Digital Clock', 'Clock_10ms', 'SampleTime', 'Ts.ctrl');
S('simulink/Sources/Digital Clock', 'Clock_50ms', 'SampleTime', 'Ts.can_slow');
S('simulink/Sources/Digital Clock', 'Clock_100ms', 'SampleTime', 'Ts.env');
c100 = 'Clock_100ms/1';
% one clock per group of stimulus tables (ruling F-R3), each a copy of its original: the original clocks Clock_base
% and Clock_10ms feed the time columns of log_fast and log_slow, Clock_50ms the BMS_REQ stimuli
cbH = CLK(mdl, 'Clock_base', 'hmi'); cbW = CLK(mdl, 'Clock_base', 'hw'); cbF = CLK(mdl, 'Clock_base', 'flt');
cbS = CLK(mdl, 'Clock_base', 'stub'); c10L = CLK(mdl, 'Clock_10ms', 'lv'); c50 = 'Clock_50ms/1';
c50I = CLK(mdl, 'Clock_50ms', 'impl'); c50A = CLK(mdl, 'Clock_50ms', 'alive');
S('simulink/Ports & Subsystems/Subsystem Reference', 'Power_Plant', 'ReferencedSubsystem', 'Power_Plant');
pp = [mdl '/Power_Plant'];

%% ---- hmi (Ts.base) ----
hm = {'estop_front_pressed', 'estopF'; 'estop_rear_pressed', 'estopR'; 'wireless_estop_tripped', 'wless'; ...
      'bumper_front', 'bumpF'; 'bumper_rear', 'bumpR'; 'reset_btn', 'reset'; 'key_off', 'keyoff'};
el = cell(size(hm, 1) + 1, 2);
for k = 1:size(hm, 1)
    el(k, :) = {hm{k, 1}, TABB(mdl, ['hmi_' hm{k, 2}], cbH, hm{k, 2})};
end
S('simulink/Sources/Constant', 'hmi_lid', 'Value', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
el(end, :) = {'lid_open', 'hmi_lid/1'};
BC(mdl, 'Hmi_BC', 'Bus_HmiInputs', el);
Ln('Hmi_BC/1', 'Power_Plant/1', 'hmi');

%% ---- hw_cmd (MCU DO / PWM, Ts.base) ----
TABB(mdl, 'hw_do0', cbW, 'do0'); TABB(mdl, 'hw_do1', cbW, 'do1'); TABB(mdl, 'hw_k3', cbW, 'k3');
TAB(mdl, 'hw_duty', cbW, 'duty'); TAB(mdl, 'hw_noz', cbW, 'noz'); TAB(mdl, 'hw_lift', cbW, 'lift');
S('simulink/Sources/Constant', 'Noz_idx', 'Value', '(1:8)''', 'SampleTime', 'Ts.base');
S('simulink/Logic and Bit Operations/Relational Operator', 'Noz_on', 'Operator', '<=', 'OutDataTypeStr', 'boolean');
Ln('Noz_idx/1', 'Noz_on/1', 'nozzle_index'); Ln('hw_noz/1', 'Noz_on/2', 'nozzles_on_count');
BC(mdl, 'Hw_BC', 'Bus_PowerHwCmd', {'MCU_Safety_OK', 'hw_do0/1'; 'Safety_Reset_Req', 'hw_do1/1'; ...
    'ImpPwr24_Enable', 'hw_k3/1'; 'Spray_Pump_Duty', 'hw_duty/1'; 'Spray_Nozzle_Cmd', 'Noz_on/1'; 'Lift_Duty', 'hw_lift/1'});
Ln('Hw_BC/1', 'Power_Plant/2', 'hw_cmd');

%% ---- lv_demand (0.01 s) ----
BC(mdl, 'Lv_BC', 'Bus_LV_LoadDemand', {'P_24V_W', TAB(mdl, 'lv_P24', c10L, 'P24'); 'P_12V_W', TAB(mdl, 'lv_P12', c10L, 'P12'); ...
    'P_5V_W', TAB(mdl, 'lv_P5', c10L, 'P5'); 'P_3V3_W', TAB(mdl, 'lv_P3', c10L, 'P3')});
Ln('Lv_BC/1', 'Power_Plant/3', 'lv_demand');

%% ---- env (0.1 s) ----
S('simulink/Sources/Constant', 'T_amb', 'Value', 'TB.T_amb', 'SampleTime', 'Ts.env');
S('simulink/Sources/Constant', 'G_solar', 'Value', 'TB.G_solar', 'SampleTime', 'Ts.env');
S('simulink/Sources/Constant', 'Soil', 'Value', 'uint8(1)', 'OutDataTypeStr', 'uint8', 'SampleTime', 'Ts.env');
S('simulink/Sources/Constant', 'Dist', 'Value', '0', 'SampleTime', 'Ts.env');
TABB(mdl, 'env_dock', c100, 'dock');
BC(mdl, 'Env_BC', 'Bus_EnvState', {'T_amb', 'T_amb/1'; 'T_sky', 'T_amb/1'; 'T_ground', 'T_amb/1'; 'G_solar', 'G_solar/1'; ...
    'soil_id', 'Soil/1'; 'dock_contact', 'env_dock/1'; 'dist_dock', 'Dist/1'});
Ln('Env_BC/1', 'Power_Plant/5', 'env');

%% ---- flt (Ts.base) ----
S('simulink/Sources/Constant', 'Flt_ic', 'Value', 'P.ic.Bus_FaultInject', 'OutDataTypeStr', 'Bus: Bus_FaultInject', ...
    'SampleTime', 'Ts.base');
TABB(mdl, 'flt_ch2', cbF, 'ch2'); TABB(mdl, 'flt_bms', cbF, 'bmsf');
S('simulink/Signal Routing/Bus Assignment', 'Flt_BA', 'AssignedSignals', 'estop_ch2_stuck,bms_fault');
Ln('Flt_ic/1', 'Flt_BA/1', 'flt_ic'); Ln('flt_ch2/1', 'Flt_BA/2', 'estop_ch2_stuck'); Ln('flt_bms/1', 'Flt_BA/3', 'bms_fault');
Ln('Flt_BA/1', 'Power_Plant/8', 'flt');

%% ---- MCU CAN1 slow frames: IMPL_CMD (0x140) + BMS_REQ (0x3D0), 0.05 s ----
S('simulink/Sources/Counter Free-Running', 'Alive_cnt', 'NumBits', '8', 'tsamp', 'Ts.can_slow');
S('simulink/Signal Attributes/Data Type Conversion', 'Alive_u8', 'OutDataTypeStr', 'uint8');
Ln('Alive_cnt/1', 'Alive_u8/1', 'alive_count');
% BMS_REQ alive frozen after TB.alive_freeze_t
S('simulink/Logic and Bit Operations/Compare To Constant', 'Bms_alive_run', 'relop', '<', 'const', 'TB.alive_freeze_t', ...
    'OutDataTypeStr', 'boolean');
S('simulink/Signal Routing/Switch', 'Bms_alive_sw', 'Criteria', 'u2 ~= 0');
S('simulink/Discrete/Unit Delay', 'Bms_alive_hold', 'InitialCondition', 'uint8(0)', 'SampleTime', 'Ts.can_slow');
Ln(c50A, 'Bms_alive_run/1', 't_50ms_alive');
Ln('Alive_u8/1', 'Bms_alive_sw/1', 'alive_u8'); Ln('Bms_alive_run/1', 'Bms_alive_sw/2', 'bms_alive_run');
Ln('Bms_alive_hold/1', 'Bms_alive_sw/3', 'bms_alive_held');
Ln('Bms_alive_sw/1', 'Bms_alive_hold/1', 'AliveCounter');
BC(mdl, 'BmsReq_BC', 'Bus_BmsReq', {'AliveCounter', 'Bms_alive_sw/1'; 'HV_Enable_Req', TABB(mdl, 'mcu_hv', c50, 'hv'); ...
    'Charge_Enable_Req', TABB(mdl, 'mcu_chg', c50, 'chg'); 'Fault_Reset_Req', TABB(mdl, 'mcu_rst', c50, 'rst'); ...
    'Shutdown_Req', TABB(mdl, 'mcu_sd', c50, 'sd')});
S('simulink/Ports & Subsystems/Subsystem Reference', 'BMS_REQ_Pack', 'ReferencedSubsystem', 'bmsreq_can_pack');
Ln('BmsReq_BC/1', 'BMS_REQ_Pack/1', 'bms_req');
% IMPL_CMD: frames stop (valid false, alive frozen) from TB.icmd_stop_t
S('simulink/Logic and Bit Operations/Compare To Constant', 'Icmd_run', 'relop', '<', 'const', 'TB.icmd_stop_t', ...
    'OutDataTypeStr', 'boolean');
Ln(c50I, 'Icmd_run/1', 't_50ms_impl');
S('simulink/Signal Routing/Switch', 'Icmd_alive_sw', 'Criteria', 'u2 ~= 0');
S('simulink/Discrete/Unit Delay', 'Icmd_alive_hold', 'InitialCondition', 'uint8(0)', 'SampleTime', 'Ts.can_slow');
Ln('Alive_u8/1', 'Icmd_alive_sw/1'); Ln('Icmd_run/1', 'Icmd_alive_sw/2', 'impl_cmd_run');
Ln('Icmd_alive_hold/1', 'Icmd_alive_sw/3', 'impl_alive_held');
Ln('Icmd_alive_sw/1', 'Icmd_alive_hold/1', 'impl_alive');
TABB(mdl, 'mcu_ien', c50I, 'ien'); TAB(mdl, 'mcu_ispd', c50I, 'ispd');
S('simulink/Signal Attributes/Data Type Conversion', 'Ispd_s', 'OutDataTypeStr', 'single');
Ln('mcu_ispd/1', 'Ispd_s/1', 'impl_speed_rpm');
S('simulink/Sources/Constant', 'Icmd_rst', 'Value', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.can_slow');
S('simulink/Ports & Subsystems/Subsystem Reference', 'IMPL_CMD_Pack', 'ReferencedSubsystem', 'implcmd_can_pack');
Ln('mcu_ien/1', 'IMPL_CMD_Pack/1', 'impl_enable'); Ln('Icmd_rst/1', 'IMPL_CMD_Pack/2', 'impl_fault_reset');
Ln('Icmd_alive_sw/1', 'IMPL_CMD_Pack/3');
Ln('Ispd_s/1', 'IMPL_CMD_Pack/4', 'impl_speed_rpm_s');
% after TB.icmd_stop_t no frame is delivered (empty slot P.ic.Bus_CanFrame8: valid false)
S('simulink/Sources/Constant', 'Icmd_none', 'Value', 'P.ic.Bus_CanFrame8', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', ...
    'SampleTime', 'Ts.can_slow');
S('simulink/Signal Routing/Switch', 'Icmd_valid', 'Criteria', 'u2 ~= 0');
Ln('IMPL_CMD_Pack/1', 'Icmd_valid/1', 'impl_cmd_frame'); Ln('Icmd_run/1', 'Icmd_valid/2');
Ln('Icmd_none/1', 'Icmd_valid/3', 'impl_cmd_none');
BC(mdl, 'CanSlow_BC', 'Bus_Can1_McuSlow', {'impl_cmd', 'Icmd_valid/1'; 'bms_req', 'BMS_REQ_Pack/1'});
Ln('CanSlow_BC/1', 'Power_Plant/7', 'can_rx');

%% ---- BMS model ----
S('simulink/Ports & Subsystems/Model', 'BMS', 'ModelName', 'Shaddad_BMS', 'SimulationMode', 'Normal');
Ln('Power_Plant/1', 'BMS/1', 'bms_meas');
Ln('CanSlow_BC/1', 'BMS/2');
Ln('BMS/1', 'Power_Plant/4', 'bms_cmd');
S('simulink/Sinks/Terminator', 'can_tx_bms_term'); Ln('BMS/2', 'can_tx_bms_term/1', 'can_tx_bms');

%% ---- implement load stub on Impl_Shaft + impl_mech ----
MREF = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
pj = PH(S('fl_lib/Mechanical/Rotational Elements/Inertia', 'J_impl', 'inertia', 'TB.J_impl'));
pr = PH(S(MREF, 'Ref_impl'));
pts = PH(S('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', 'T_load_src'));
pws = PH(S(sprintf('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational\nMotion Sensor'), 'W_impl_sens'));
hs = shaddad_bu('pport', pp, 'Impl_Shaft');
PL(mdl, hs, pj.LConn(1));
% torque source: LConn(1) = R (shaft), RConn(1) = S (PS input), RConn(2) = C (reference); sensor L1 = R, R1 = C
PL(mdl, pts.LConn(1), hs); PL(mdl, pts.RConn(2), pr.LConn(1));
PL(mdl, pws.LConn(1), hs); PL(mdl, pws.RConn(1), pr.LConn(1));
w = PSS(mdl, 'W_impl', pws.RConn(2), 'rad/s');
pa = PH(S('fl_lib/Physical Signals/Sinks/PS Terminator', 'Angle_term'));
PL(mdl, pws.RConn(3), pa.LConn(1));
S('simulink/Discrete/Unit Delay', 'W_impl_d', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
Ln(w, 'W_impl_d/1', 'w_impl');
S('simulink/Math Operations/Abs', 'W_abs'); Ln('W_impl_d/1', 'W_abs/1', 'w_impl_d');
S('simulink/Math Operations/Product', 'W_sq', 'Inputs', '2'); Ln('W_impl_d/1', 'W_sq/1'); Ln('W_abs/1', 'W_sq/2', 'w_impl_abs');
S('simulink/Math Operations/Gain', 'K_aero', 'Gain', 'TB.k_aero'); Ln('W_sq/1', 'K_aero/1', 'w_impl_sq');
S('simulink/Math Operations/Gain', 'K_lin', 'Gain', 'TB.k_lin'); Ln('W_impl_d/1', 'K_lin/1');
S('simulink/Math Operations/Gain', 'W_10', 'Gain', '0.1'); Ln('W_impl_d/1', 'W_10/1');
S('simulink/Discontinuities/Saturation', 'W_sat', 'UpperLimit', '1', 'LowerLimit', '-1'); Ln('W_10/1', 'W_sat/1', 'w_impl_10');
S('simulink/Math Operations/Gain', 'T_c', 'Gain', 'TB.T_c'); Ln('W_sat/1', 'T_c/1', 'w_impl_sign');
S('simulink/Math Operations/Gain', 'K_lf', 'Gain', 'TB.k_lf'); Ln('W_impl_d/1', 'K_lf/1');
S('simulink/Math Operations/Sum', 'T_lf_sum', 'Inputs', '++'); Ln('T_c/1', 'T_lf_sum/1', 'T_coulomb');
Ln('K_lf/1', 'T_lf_sum/2', 'T_lf_lin');
TAB(mdl, 'LF', cbS, 'lf');
S('simulink/Math Operations/Product', 'T_lf', 'Inputs', '2'); Ln('T_lf_sum/1', 'T_lf/1', 'T_lf_base');
Ln('LF/1', 'T_lf/2', 'load_factor');
S('simulink/Math Operations/Sum', 'T_load', 'Inputs', '---'); Ln('K_aero/1', 'T_load/1', 'T_aero');
Ln('K_lin/1', 'T_load/2', 'T_lin'); Ln('T_lf/1', 'T_load/3', 'T_lf');
h = SPS(mdl, 'T_load_PS', 'T_load/1', 'N*m', 'T_load'); PL(mdl, h, pts.RConn(1));
% impl_mech: type = VC_IMPL, shaftSpeed, shaftTorque, linePressure (sprayer stub: p_set (w / w_nom)^2)
RT(mdl, 'W_impl_RT', 'Ts.ctrl'); Ln('W_impl_d/1', 'W_impl_RT/1');
RT(mdl, 'T_load_RT', 'Ts.ctrl'); Ln('T_load/1', 'T_load_RT/1');
S('simulink/Math Operations/Product', 'W_sq_p', 'Inputs', '2');
Ln('W_impl_RT/1', 'W_sq_p/1', 'w_impl_10ms'); Ln('W_impl_RT/1', 'W_sq_p/2');
S('simulink/Math Operations/Gain', 'P_line', 'Gain', 'TB.p_line*P.structure.impl.sprayer.p_set/P.power.impl.sprayer.w_nom^2');
Ln('W_sq_p/1', 'P_line/1', 'w_impl_sq_10ms');
S('simulink/Sources/Constant', 'Mech_ic', 'Value', 'P.ic.Bus_ImplMechState', 'OutDataTypeStr', 'Bus: Bus_ImplMechState', ...
    'SampleTime', 'Ts.ctrl');
S('simulink/Sources/Constant', 'Impl_type', 'Value', 'uint8(VC_IMPL)', 'OutDataTypeStr', 'uint8', 'SampleTime', 'Ts.ctrl');
S('simulink/Signal Routing/Bus Assignment', 'Mech_BA', 'AssignedSignals', 'type,shaftSpeed,shaftTorque,linePressure');
Ln('Mech_ic/1', 'Mech_BA/1', 'impl_mech_ic'); Ln('Impl_type/1', 'Mech_BA/2', 'impl_type');
Ln('W_impl_RT/1', 'Mech_BA/3'); Ln('T_load_RT/1', 'Mech_BA/4', 'T_load_10ms');
Ln('P_line/1', 'Mech_BA/5', 'line_pressure');
Ln('Mech_BA/1', 'Power_Plant/6', 'impl_mech');

%% ---- DRIVE stubs on DC48_TracL / DC48_TracR ----
CCS = 'fl_lib/Electrical/Electrical Sources/Controlled Current Source';
for sd = {'L', 'R'}
    s = sd{1};
    hpP = shaddad_bu('pport', pp, ['DC48_Trac' s '_p']); hpN = shaddad_bu('pport', pp, ['DC48_Trac' s '_n']);
    pc = PH(S('ee_lib/Passive/Capacitor', ['C_drv_' s], 'c', 'TB.C_drv', 'r', 'TB.R_esr', 'vc_specify', 'on', ...
        'vc_priority', 'High', 'vc', '0'));
    PL(mdl, pc.LConn(1), hpP); PL(mdl, pc.RConn(1), hpN);
    pl = PH(S(CCS, ['I_drv_' s]));
    PL(mdl, pl.RConn(2), hpP); PL(mdl, pl.LConn(1), hpN);                    % tail on DC+, head on DC-: + = load
    pv = PH(S('ee_lib/Sensors & Transducers/Voltage Sensor', ['V_drv_' s '_sens']));
    PL(mdl, pv.LConn(1), hpP); PL(mdl, pv.RConn(2), hpN);
    v = PSS(mdl, ['V_drv_' s], pv.RConn(1), 'V');
    S('simulink/Discrete/Unit Delay', ['V_drv_' s '_d'], 'InitialCondition', '0', 'SampleTime', 'Ts.base');
    Ln(v, ['V_drv_' s '_d/1'], ['V_drv_' s]);
    TAB(mdl, ['Load_' s], cbS, ['load' s]);
    S('simulink/Math Operations/Bias', ['Gate_' s '_off'], 'Bias', '-TB.V_gate(1)');
    S('simulink/Math Operations/Gain', ['Gate_' s '_k'], 'Gain', '1/(TB.V_gate(2) - TB.V_gate(1))');
    S('simulink/Discontinuities/Saturation', ['Gate_' s], 'UpperLimit', '1', 'LowerLimit', '0');
    Ln(['V_drv_' s '_d/1'], ['Gate_' s '_off/1'], ['V_drv_' s '_d']);
    Ln(['Gate_' s '_off/1'], ['Gate_' s '_k/1'], ['gate_dV_' s]); Ln(['Gate_' s '_k/1'], ['Gate_' s '/1'], ['gate_raw_' s]);
    S('simulink/Discontinuities/Saturation', ['V_' s '_min'], 'UpperLimit', 'inf', 'LowerLimit', '30');
    Ln(['V_drv_' s '_d/1'], ['V_' s '_min/1']);
    S('simulink/Math Operations/Product', ['I_from_P_' s], 'Inputs', '*/');
    Ln(['Load_' s '/1'], ['I_from_P_' s '/1'], ['load_' s]); Ln(['V_' s '_min/1'], ['I_from_P_' s '/2'], ['V_drv_' s '_min']);
    S('simulink/Sources/Constant', ['Is_power_' s], 'Value', 'TB.load_is_power', 'SampleTime', 'Ts.base');
    S('simulink/Signal Routing/Switch', ['Load_mode_' s], 'Criteria', 'u2 ~= 0');
    Ln(['I_from_P_' s '/1'], ['Load_mode_' s '/1'], ['I_from_P_' s]);
    Ln(['Is_power_' s '/1'], ['Load_mode_' s '/2'], ['load_is_power_' s]);
    Ln(['Load_' s '/1'], ['Load_mode_' s '/3']);
    S('simulink/Math Operations/Product', ['I_load_' s], 'Inputs', '2');
    Ln(['Load_mode_' s '/1'], ['I_load_' s '/1'], ['I_load_req_' s]); Ln(['Gate_' s '/1'], ['I_load_' s '/2'], ['gate_' s]);
    h = SPS(mdl, ['I_drv_' s '_PS'], ['I_load_' s '/1'], 'A', ['I_load_' s]); PL(mdl, h, pl.RConn(1));
end
% switchable short (fault block) across DC48_TracL
ps = PH(S('ee_lib/Switches & Breakers/SPST Switch', 'S_short', 'R_closed', '1e-6', 'G_open', '1e-8', 'Threshold', '0.5'));
prs = PH(S('ee_lib/Passive/Resistor', 'R_short', 'R', 'TB.R_short'));
PL(mdl, ps.LConn(2), shaddad_bu('pport', pp, 'DC48_TracL_p')); PL(mdl, ps.RConn(1), prs.LConn(1));
PL(mdl, prs.RConn(1), shaddad_bu('pport', pp, 'DC48_TracL_n'));
TAB(mdl, 'Short', cbS, 'short');
h = SPS(mdl, 'Short_PS', 'Short/1', 'V', 'short_cmd'); PL(mdl, h, ps.LConn(1));
local_solver(mdl, shaddad_bu('pport', pp, 'DC48_TracL_n'));

%% ---- outputs and logs ----
S('simulink/Sinks/Terminator', 'impl_can_term'); Ln('Power_Plant/4', 'impl_can_term/1', 'can_tx_impl');
S('simulink/Sinks/Terminator', 'safety_term'); Ln('Power_Plant/6', 'safety_term/1', 'safety_ok');
st = {'V_pack_V', 'I_pack_A', 'V_bus_V', 'V_24V_V', 'I_24V_A', 'V_24IMP_V', 'V_12V_V', 'I_12V_A', 'V_5V_V', 'V_3V3_V', ...
      'Rail_24V_OK', 'Rail_12V_OK', 'Rail_5V_OK', 'Rail_3V3_OK', 'K1_Closed', 'K2_Closed', 'K3_Closed', 'K4_Closed', ...
      'Safety_OK', 'Safety_HV_OK', 'EStop_Any', 'EStop_Ch1_OK', 'EStop_Ch2_OK', 'Safety_Latched_Off', 'Dock_Present', ...
      'Key_On', 'Fuse_Blown'};
S('simulink/Signal Routing/Bus Selector', 'Status_sel', 'OutputSignals', strjoin(st, ','));
Ln('Power_Plant/2', 'Status_sel/1', 'power_status');
% the six stub signals of log_fast travel as one virtual bus from the stubs to the log (spec 6.3)
sb = {'I_load_L', 'I_load_L/1'; 'I_load_R', 'I_load_R/1'; 'w_impl_d', 'W_impl_d/1'; 'T_load', 'T_load/1'; ...
      'V_drv_L_d', 'V_drv_L_d/1'; 'V_drv_R_d', 'V_drv_R_d/1'};
BCV(mdl, 'Stub_log_BC', sb);
S('simulink/Signal Routing/Bus Selector', 'Stub_log_sel', 'OutputSignals', strjoin(sb(:, 1), ','));
Ln('Stub_log_BC/1', 'Stub_log_sel/1', 'stub_log');
lf = [{'Clock_base/1'}, arrayfun(@(k) sprintf('Status_sel/%d', k), 1:numel(st), 'UniformOutput', false), ...
      arrayfun(@(k) sprintf('Stub_log_sel/%d', k), 1:size(sb, 1), 'UniformOutput', false)];
local_log(mdl, 'log_fast_mux', 'lf_dbl', lf, [{'t'}, st, {'I_load_L', 'I_load_R', 'w_impl_d', 'T_load', 'V_drv_L_d', 'V_drv_R_d'}], ...
    'log_fast', 'log_fast', 'Decimation', 'TB.dec_fast');
ie = {'Impl_Running', 'Impl_Fault', 'Impl_Ready', 'Impl_Speed_radps', 'Impl_Torque_Nm', 'Impl_Power_W', 'Impl_Current_A', ...
      'Fault_Code'};
S('simulink/Signal Routing/Bus Selector', 'Impl_sel', 'OutputSignals', strjoin(ie, ','));
Ln('Power_Plant/3', 'Impl_sel/1', 'impl_elec');
cs = {'K1_Cmd', 'K2_Cmd', 'K4_Cmd', 'Charger_Enable', 'Charger_I_Req_A', 'Fan_Cmd'};
S('simulink/Signal Routing/Bus Selector', 'Cmd_sel', 'OutputSignals', strjoin(cs, ','));
Ln('BMS/1', 'Cmd_sel/1');
% the three MCU stimuli of log_slow travel as one virtual bus, each picked right in front of its rate transition
BCV(mdl, 'Mcu_log_BC', {'HV_Enable_Req', 'mcu_hv/1'; 'impl_enable', 'mcu_ien/1'; 'impl_speed_rpm', 'mcu_ispd/1'});
mp = {'Hv', 'HV_Enable_Req'; 'Ien', 'impl_enable'; 'Ispd', 'impl_speed_rpm'};
for k = 1:size(mp, 1)
    RT(mdl, [mp{k, 1} '_RT'], 'Ts.ctrl');
    S('simulink/Signal Routing/Bus Selector', [mp{k, 1} '_log_sel'], 'OutputSignals', mp{k, 2});
    Ln('Mcu_log_BC/1', [mp{k, 1} '_log_sel/1'], 'mcu_log');
    Ln([mp{k, 1} '_log_sel/1'], [mp{k, 1} '_RT/1']);
end
p = get_param([mdl '/Hv_RT'], 'Position');           % Hv_RT 90 px tall (graphical; fix round 2 trial, t6_fix2_hprobe.log)
set_param([mdl '/Hv_RT'], 'Position', [p(1) p(2) p(3) p(2) + 90]);
ls = [{'Clock_10ms/1'}, arrayfun(@(k) sprintf('Impl_sel/%d', k), 1:numel(ie), 'UniformOutput', false), ...
      arrayfun(@(k) sprintf('Cmd_sel/%d', k), 1:numel(cs), 'UniformOutput', false), {'Hv_RT/1', 'Ien_RT/1', 'Ispd_RT/1'}];
local_log(mdl, 'log_slow_mux', 'ls_dbl', ls, [{'t'}, ie, cs, {'HV_Enable_Req_10ms', 'impl_enable_10ms', 'impl_speed_rpm_10ms'}], ...
    'log_slow', 'log_slow', 'Decimation', 'TB.dec_slow');
% BMS telemetry and plant truth summaries at 0.1 s (decimation TB.dec_bms / TB.dec_truth) for long runs
tb = {'Contactor_State', 'HV_Ready', 'Fault_Level', 'Fault_Word', 'Warning_Word', 'SOC_pack', 'Charging_Active', ...
      'Charge_Complete', 'I_chg_lim_A', 'I_dis_lim_A', 'T_cell_max_K', 'Fan_Cmd', 'K1_Closed', 'Runtime_Remaining_s'};
S('simulink/Signal Routing/Bus Selector', 'Tel_sel', 'OutputSignals', strjoin(tb, ','));
Ln('BMS/3', 'Tel_sel/1', 'telem');
S('simulink/Sources/Digital Clock', 'Clock_100ms_log', 'SampleTime', 'Ts.hk');
cbm = CLK(mdl, 'Clock_100ms_log', 'bms');            % ruling F-R3: log_bms's time column has a clock of its own
lb = [{cbm}, arrayfun(@(k) sprintf('Tel_sel/%d', k), 1:numel(tb), 'UniformOutput', false)];
local_log(mdl, 'log_bms_mux', 'lb_dbl', lb, [{'t'}, tb], 'log_bms', 'log_bms', 'Decimation', 'TB.dec_bms');
tt = {'SOC_pack_true', 'T_cell_max_K', 'T_cell_min_K', 'V_term_V', 'I_pack_A', 'E_out_Wh', 'E_in_Wh', 'E_chg_bus_Wh', ...
      'E_trac_dc_Wh', 'E_impl_dc_Wh', 'E_lv_Wh', 'E_dcdc_loss_Wh', 'E_pdu_loss_Wh', 'E_loss_batt_Wh', 'I_pack_peak_A', ...
      'P_grid_W'};
S('simulink/Signal Routing/Bus Selector', 'Truth_sel', 'OutputSignals', strjoin(tt, ','));
Ln('Power_Plant/5', 'Truth_sel/1', 'power_truth');
% the cell vectors: a pick right in front of each MinMax (spec 6.2)
S('simulink/Signal Routing/Bus Selector', 'SOC_true_sel', 'OutputSignals', 'SOC_true');
S('simulink/Signal Routing/Bus Selector', 'V_cell_true_sel', 'OutputSignals', 'V_cell_true');
Ln('Power_Plant/5', 'SOC_true_sel/1'); Ln('Power_Plant/5', 'V_cell_true_sel/1');
S('simulink/Math Operations/MinMax', 'SOC_true_min', 'Function', 'min', 'Inputs', '1');
S('simulink/Math Operations/MinMax', 'V_cell_true_max', 'Function', 'max', 'Inputs', '1');
Ln('SOC_true_sel/1', 'SOC_true_min/1'); Ln('V_cell_true_sel/1', 'V_cell_true_max/1');
lt = [{'Clock_100ms_log/1'}, arrayfun(@(k) sprintf('Truth_sel/%d', k), 1:numel(tt), 'UniformOutput', false), ...
      {'SOC_true_min/1', 'V_cell_true_max/1'}];
local_log(mdl, 'log_truth_mux', 'lt_dbl', lt, [{'t'}, tt, {'SOC_true_min', 'V_cell_true_max'}], 'log_truth', ...
    'log_truth', 'Decimation', 'TB.dec_truth');
for lg = {'BMS', 3, 'telem'; 'Power_Plant', 5, 'power_truth'}'
    ph = get_param([mdl '/' lg{1}], 'PortHandles');
    set_param(ph.Outport(lg{2}), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', lg{3});
end
set_param(mdl, 'Description', sprintf(['%s (T05 bench, build plan T05.7 / T05.8): see scripts/build/build_tb_power_plant.m. ' ...
    'Stimulus = model-workspace TB (tests/power/tb_power_plant_default.m); log columns: tests/power/pp_log_cols.m.'], mdl));
local_save(mdl, f);
end

% ===========================================================================================
%  tb_lv_rails (LV_Distribution alone)
% ===========================================================================================
function local_lv(mdl, f)
TB = struct('vaux_t', [0 1e7], 'vaux_v', [52 52], 'P24_t', [0 1e7], 'P24_v', [53 53], 'P12_t', [0 1e7], 'P12_v', [99 99], ...
    'P5_t', [0 1e7], 'P5_v', [2.5 2.5], 'P3_t', [0 1e7], 'P3_v', [0.6 0.6], 'fan_t', [0 1e7], 'fan_v', [0 0], ...
    'k3_t', [0 1e7], 'k3_v', [0 0], 'imp_R', 1e3, 'MaxNonlinIter', 3, 'MaxModeIter', 2);
local_new(mdl, f, TB);
S = @(varargin) A(mdl, varargin{:});
Ln = @(varargin) L(mdl, varargin{:});
S('simulink/Sources/Digital Clock', 'Clock_base', 'SampleTime', 'Ts.base');
S('simulink/Sources/Digital Clock', 'Clock_10ms', 'SampleTime', 'Ts.ctrl');
S('simulink/Ports & Subsystems/Subsystem Reference', 'LV', 'ReferencedSubsystem', 'LV_Distribution');
lv = [mdl '/LV'];
BC(mdl, 'Lv_BC', 'Bus_LV_LoadDemand', {'P_24V_W', TAB(mdl, 'lv_P24', 'Clock_10ms/1', 'P24'); ...
    'P_12V_W', TAB(mdl, 'lv_P12', 'Clock_10ms/1', 'P12'); 'P_5V_W', TAB(mdl, 'lv_P5', 'Clock_10ms/1', 'P5'); ...
    'P_3V3_W', TAB(mdl, 'lv_P3', 'Clock_10ms/1', 'P3')});
Ln('Lv_BC/1', 'LV/1', 'lv_demand');
% LV_Distribution picks thermal.Fan_eff and safety.coil: the bench sends the fan table and the coil word as those buses
cs = CLK(mdl, 'Clock_base', 'stim');                % ruling F-R3: the stimulus tables' own clock, the original
TAB(mdl, 'Fan', cs, 'fan');                          % feeds the log's time column
BCV(mdl, 'Thermal_BC', {'Fan_eff', 'Fan/1'});
Ln('Thermal_BC/1', 'LV/2', 'thermal');
TABB(mdl, 'K3_req', cs, 'k3');
S('simulink/Sources/Constant', 'Coil_off', 'Value', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
add_block([mdl '/Coil_off'], [mdl '/Coil_off_K4']);  % ruling F-R3: the K4 slot's own copy, below the K3 request
S('simulink/Signal Routing/Mux', 'Coil_mux', 'Inputs', '4');
Ln('Coil_off/1', 'Coil_mux/1', 'coil_off'); Ln('Coil_off/1', 'Coil_mux/2'); Ln('K3_req/1', 'Coil_mux/3', 'K3_coil');
Ln('Coil_off_K4/1', 'Coil_mux/4', 'coil_off_K4');
BCV(mdl, 'Safety_BC', {'coil', 'Coil_mux/1'});
Ln('Safety_BC/1', 'LV/3', 'safety');
pv = PH(S('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', 'V_aux'));
pg = PH(S('ee_lib/Connectors & References/Electrical Reference', 'GND'));
pis = PH(S('ee_lib/Sensors & Transducers/Current Sensor', 'I_aux_sens'));
PL(mdl, pv.LConn(1), pis.LConn(1)); PL(mdl, pis.RConn(2), shaddad_bu('pport', lv, 'AUX48_p'));
PL(mdl, pv.RConn(2), pg.LConn(1));
PL(mdl, shaddad_bu('pport', lv, 'AUX48_n'), pg.LConn(1));
TAB(mdl, 'Vaux', cs, 'vaux');
set_param([mdl '/Vaux'], 'InterpMethod', 'Linear point-slope');          % AUX48 ramps (UVLO test)
h = SPS(mdl, 'Vaux_PS', 'Vaux/1', 'V', 'V_aux_set'); PL(mdl, h, pv.RConn(1));
pr = PH(S('ee_lib/Passive/Resistor', 'R_imp', 'R', 'TB.imp_R'));
PL(mdl, pr.LConn(1), shaddad_bu('pport', lv, 'V24IMP_p')); PL(mdl, pr.RConn(1), shaddad_bu('pport', lv, 'V24IMP_n'));
local_solver(mdl, pg.LConn(1));
pvs = PH(S('ee_lib/Sensors & Transducers/Voltage Sensor', 'V_aux_sens'));
PL(mdl, pvs.LConn(1), shaddad_bu('pport', lv, 'AUX48_p')); PL(mdl, pvs.RConn(2), pg.LConn(1));
va = PSS(mdl, 'V_aux_out', pvs.RConn(1), 'V'); ia = PSS(mdl, 'I_aux_out', pis.RConn(1), 'A');
% the bus lv, picked right in front of each consumer (L3): rails and rail_ok in front of their Demuxes, the six
% other elements in front of the log's conversions (in log column order)
lvs = {'rails', 'rail_ok', 'K3_closed', 'fuse24_ok', 'P_lv', 'P_dcdc_loss', 'P_k3_loss', 'P_conv'};
S('simulink/Signal Routing/Bus Selector', 'Rails_sel', 'OutputSignals', 'rails');
S('simulink/Signal Routing/Bus Selector', 'Ok_sel', 'OutputSignals', 'rail_ok');
S('simulink/Signal Routing/Bus Selector', 'Lv_sel', 'OutputSignals', strjoin(lvs(3:8), ','));
Ln('LV/1', 'Rails_sel/1', 'lv'); Ln('LV/1', 'Ok_sel/1'); Ln('LV/1', 'Lv_sel/1');
S('simulink/Signal Routing/Demux', 'Rails_dmx', 'Outputs', '7'); Ln('Rails_sel/1', 'Rails_dmx/1');
S('simulink/Signal Routing/Demux', 'Ok_dmx', 'Outputs', '4'); Ln('Ok_sel/1', 'Ok_dmx/1');
rn = {'V_24V', 'I_24V', 'V_24IMP', 'V_12V', 'I_12V', 'V_5V', 'V_3V3'};
on = {'Rail_24V_OK', 'Rail_12V_OK', 'Rail_5V_OK', 'Rail_3V3_OK'};
src = [{'Clock_base/1', va, ia}, arrayfun(@(k) sprintf('Rails_dmx/%d', k), 1:7, 'UniformOutput', false), ...
       arrayfun(@(k) sprintf('Ok_dmx/%d', k), 1:4, 'UniformOutput', false), ...
       arrayfun(@(k) sprintf('Lv_sel/%d', k), 1:6, 'UniformOutput', false)];
nms = [{'t', 'V_aux', 'I_aux'}, rn, on, lvs(3:8)];
S('simulink/Signal Routing/Mux', 'log_mux', 'Inputs', num2str(numel(src)));
for k = 1:numel(src)
    S('simulink/Signal Attributes/Data Type Conversion', sprintf('dbl%d', k), 'OutDataTypeStr', 'double');
    if k == 1, Ln(src{k}, 'dbl1/1'); else, Ln(src{k}, sprintf('dbl%d/1', k), nms{k}); end    % the clock: t_base
    Ln(sprintf('dbl%d/1', k), sprintf('log_mux/%d', k), [nms{k} '_dbl']);
end
S('simulink/Sinks/To Workspace', 'log_lv', 'VariableName', 'log_lv', 'SaveFormat', 'Array', 'SampleTime', '-1', ...
    'MaxDataPoints', 'inf');
Ln('log_mux/1', 'log_lv/1', 'log_lv');
set_param(mdl, 'Description', ['tb_lv_rails (T05): LV_Distribution with AUX48 from a Controlled Voltage Source (TB.vaux), ' ...
    'R_imp on V24IMP, Electrical Reference, Solver Configuration; the fan table and the coil word go in as the buses ' ...
    'thermal (Fan_eff) and safety (coil). log_lv = [t V_aux I_aux rails(7) rail_ok(4) K3_closed fuse24_ok P_lv ' ...
    'P_dcdc_loss P_k3_loss P_in24 P_out24 P_in12 P_out12 P_in5 P_out5] (columns 1..26).']);
local_save(mdl, f);
end

% ===========================================================================================
%  tb_impl_codec (IMPL_CMD / IMPL_FB round trip)
% ===========================================================================================
function local_codec(mdl, f)
TB = struct('cmd', zeros(1, 5), 'fb', zeros(1, 9));       % rows [t inputs...] (From Workspace, 0.05 s)
local_new(mdl, f, TB);
S = @(varargin) A(mdl, varargin{:});
Ln = @(varargin) L(mdl, varargin{:});
S('simulink/Sources/From Workspace', 'Cmd_in', 'VariableName', 'TB.cmd', 'SampleTime', 'Ts.can_slow', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
S('simulink/Sources/From Workspace', 'Fb_in', 'VariableName', 'TB.fb', 'SampleTime', 'Ts.can_slow', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
S('simulink/Signal Routing/Demux', 'Cmd_dmx', 'Outputs', '4'); Ln('Cmd_in/1', 'Cmd_dmx/1', 'cmd_in');
S('simulink/Signal Routing/Demux', 'Fb_dmx', 'Outputs', '8'); Ln('Fb_in/1', 'Fb_dmx/1', 'fb_in');
S('simulink/Ports & Subsystems/Subsystem Reference', 'Cmd_pack', 'ReferencedSubsystem', 'implcmd_can_pack');
S('simulink/Ports & Subsystems/Subsystem Reference', 'Cmd_unpack', 'ReferencedSubsystem', 'implcmd_can_unpack');
S('simulink/Ports & Subsystems/Subsystem Reference', 'Fb_pack', 'ReferencedSubsystem', 'implfb_can_pack');
S('simulink/Ports & Subsystems/Subsystem Reference', 'Fb_unpack', 'ReferencedSubsystem', 'implfb_can_unpack');
ct = {'boolean', 'boolean', 'uint8', 'single'};
cn = {'enable', 'fault_reset', 'alive', 'speed_rpm', 'valid'};
for k = 1:4
    S('simulink/Signal Attributes/Data Type Conversion', sprintf('c_in%d', k), 'OutDataTypeStr', ct{k}, 'RndMeth', 'Nearest');
    Ln(sprintf('Cmd_dmx/%d', k), sprintf('c_in%d/1', k), [cn{k} '_in']);
    Ln(sprintf('c_in%d/1', k), sprintf('Cmd_pack/%d', k), cn{k});
end
ft = {'boolean', 'boolean', 'boolean', 'uint8', 'single', 'single', 'single', 'uint8'};
fn = {'running', 'fault', 'ready', 'impl_type', 'speed_rpm', 'current_A', 'temp_degC', 'alive', 'valid'};
for k = 1:8
    S('simulink/Signal Attributes/Data Type Conversion', sprintf('f_in%d', k), 'OutDataTypeStr', ft{k}, 'RndMeth', 'Nearest');
    Ln(sprintf('Fb_dmx/%d', k), sprintf('f_in%d/1', k), [fn{k} '_in']);
    Ln(sprintf('f_in%d/1', k), sprintf('Fb_pack/%d', k), fn{k});
end
Ln('Cmd_pack/1', 'Cmd_unpack/1', 'impl_cmd'); Ln('Fb_pack/1', 'Fb_unpack/1', 'impl_fb');
S('simulink/Signal Routing/Mux', 'Cmd_mux', 'Inputs', '5');
for k = 1:5
    S('simulink/Signal Attributes/Data Type Conversion', sprintf('c_out%d', k), 'OutDataTypeStr', 'double');
    Ln(sprintf('Cmd_unpack/%d', k), sprintf('c_out%d/1', k), [cn{k} '_out']);
    Ln(sprintf('c_out%d/1', k), sprintf('Cmd_mux/%d', k), [cn{k} '_out_dbl']);
end
S('simulink/Signal Routing/Mux', 'Fb_mux', 'Inputs', '9');
for k = 1:9
    S('simulink/Signal Attributes/Data Type Conversion', sprintf('f_out%d', k), 'OutDataTypeStr', 'double');
    Ln(sprintf('Fb_unpack/%d', k), sprintf('f_out%d/1', k), [fn{k} '_out']);
    Ln(sprintf('f_out%d/1', k), sprintf('Fb_mux/%d', k), [fn{k} '_out_dbl']);
end
S('simulink/Sinks/To Workspace', 'cmd_out', 'VariableName', 'cmd_out', 'SaveFormat', 'Array', 'SampleTime', '-1', ...
    'MaxDataPoints', 'inf');
S('simulink/Sinks/To Workspace', 'fb_out', 'VariableName', 'fb_out', 'SaveFormat', 'Array', 'SampleTime', '-1', ...
    'MaxDataPoints', 'inf');
Ln('Cmd_mux/1', 'cmd_out/1', 'cmd_out'); Ln('Fb_mux/1', 'fb_out/1', 'fb_out');
set_param(mdl, 'Description', ['tb_impl_codec (T05): TB.cmd rows [t enable fault_reset alive speed_rpm] -> implcmd_can_pack -> ' ...
    'implcmd_can_unpack -> cmd_out [enable fault_reset alive speed_rpm valid]; TB.fb rows [t running fault ready impl_type ' ...
    'speed_rpm current_A temp_degC alive] -> implfb_can_pack -> implfb_can_unpack -> fb_out (+ valid).']);
local_save(mdl, f);
end
