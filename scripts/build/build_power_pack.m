function built = build_power_pack(which)
%BUILD_POWER_PACK  Pack, pack thermal and power sensor Subsystem References (owner: power, T04).
%
%   build_power_pack()            build all four files
%   build_power_pack(names)       build only the listed files ('Pack_Grouped', 'Pack_Lumped',
%                                 'Pack_Thermal', 'Power_Sensors')
%   built = build_power_pack(...) names of the files written
%
%   Files (docs/specs/power.md 2.2, 2.3, 2.10, 3.1, 3.2, 3.8; build plan T04.2-T04.4):
%     subsystems/power/Pack_Grouped.slx   16S3P Grouped Battery Builder module (VC_PACK == 1)
%     subsystems/power/Pack_Lumped.slx      16S3P Lumped module (VC_PACK == 2), identical ports
%     subsystems/power/Pack_Thermal.slx     enclosure AIR / WALL network, solar, fans, dock air (D45)
%     subsystems/power/Power_Sensors.slx    BMS sensor models, status bus, truth bus, 1 ms energy
%                                           integrals (D40)
%   Pack ports: Connection Ports pack_p, pack_n (electrical), AmbH (thermal); input Bal_Cmd
%   boolean[16] (0.01 s); outputs T_cell_true[16] (K), SOC_true[16], V_cell_true[16] (V) (double, Probe at
%   the local-solver rate, in the Probe's variable order). Every numeric block value is a P.power.* expression.
%   Pack_Thermal and Power_Sensors take buses (model layout spec 6.2): each consumer picks its element from an
%   In Bus Element port right in front of it, and Pack_Thermal sends its outputs as the bus thermal (Out Bus
%   Element). Every file is laid out by the layout engine when shaddad_bu saves it (shaddad_layout).
%   Prerequisite: build_shaddad_battery (generated libraries on the path). Needs P / Ts in base only
%   for the optional update in the test benches (the build itself does not compile).

all = {'Pack_Grouped', 'Pack_Lumped', 'Pack_Thermal', 'Power_Sensors'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which) || isstring(which), which = cellstr(which); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dirp = fullfile(root, 'subsystems', 'power');
built = {};
for k = 1:numel(which)
    nm = which{k};
    f = fullfile(dirp, [nm '.slx']);
    switch nm
        case 'Pack_Grouped', local_pack(nm, f, false);
        case 'Pack_Lumped',    local_pack(nm, f, true);
        case 'Pack_Thermal',   local_thermal(nm, f);
        case 'Power_Sensors',  local_sensors(nm, f);
        otherwise, error('build_power_pack:name', 'Unknown file %s', nm);
    end
    built{end+1} = nm; %#ok<AGROW>
end
end

% ===========================================================================================
%  Pack_Grouped / Pack_Lumped
% ===========================================================================================
function local_pack(name, f, lumped)
EL = 'Connection: foundation.electrical.electrical';
TH = 'Connection: foundation.thermal.thermal';
SPS = sprintf('nesl_utility/Simulink-PS\nConverter');
shaddad_bu('fresh', name, f, 'ssr');
ip = shaddad_bu('add', 'simulink/Sources/In1', [name '/Bal_Cmd'], 'Port', '1', ...
    'OutDataTypeStr', 'boolean', 'PortDimensions', 'P.power.pack.Ns', 'SampleTime', 'Ts.bms_fast');
cpP = shaddad_bu('add', 'nesl_utility/Connection Port', [name '/pack_p'], 'Port', '1', 'Side', 'Right', 'ConnectionType', EL);
cpN = shaddad_bu('add', 'nesl_utility/Connection Port', [name '/pack_n'], 'Port', '2', 'Side', 'Right', 'ConnectionType', EL);
cpA = shaddad_bu('add', 'nesl_utility/Connection Port', [name '/AmbH'],   'Port', '3', 'Side', 'Left',  'ConnectionType', TH);
% outputs in the Probe's variable order (Simscape sorts them by name), so the truth lines run straight
o1 = shaddad_bu('add', 'simulink/Sinks/Out1', [name '/T_cell_true'], 'Port', '1', 'OutDataTypeStr', 'double');
o2 = shaddad_bu('add', 'simulink/Sinks/Out1', [name '/SOC_true'],    'Port', '2', 'OutDataTypeStr', 'double');
o3 = shaddad_bu('add', 'simulink/Sinks/Out1', [name '/V_cell_true'], 'Port', '3', 'OutDataTypeStr', 'double');
%#ok<*NASGU>
if ~lumped
    load_system('Shaddad_Battery_lib');
    mb = shaddad_bu('add', 'Shaddad_Battery_lib/Shaddad_Pack', [name '/Module']);
    nz = 'zeros(P.power.pack.Ns,1)';
    set_param(mb, ...
        'ExtrapolationMethodCell', 'simscape.enum.extrapolation.linear', ...
        'BatteryCapacityCell', 'P.power.cell.Q_Ah*P.power.pack.age.cap_factor', ...
        'BatteryCapacityCellPercentDeviation', 'P.power.pack.var.cap_dev_pct(:)', ...
        'SOCBreakpointsCell', 'P.power.cell.SOC_bp', 'SOCBreakpointsCellPercentDeviation', nz, ...
        'TemperatureBreakpointsCell', 'P.power.cell.T_bp', 'TemperatureBreakpointsCellPercentDeviation', nz, ...
        'OpenCircuitVoltageThermalCell', 'P.power.cell.OCV_tab', 'OpenCircuitVoltageThermalCellPercentDeviation', nz, ...
        'VoltageRangeCell', 'P.power.cell.VoltageRange', 'VoltageRangeCellPercentDeviation', nz, ...
        'ResistanceSOCBreakpointsCell', 'P.power.cell.SOC_bp', 'ResistanceSOCBreakpointsCellPercentDeviation', nz, ...
        'ResistanceTemperatureBreakpointsCell', 'P.power.cell.T_bp', 'ResistanceTemperatureBreakpointsCellPercentDeviation', nz, ...
        'R0ThermalCell', 'P.power.cell.R0_tab*P.power.pack.age.R0_factor', ...
        'R0ThermalCellPercentDeviation', 'P.power.pack.var.R0_dev_pct(:)', ...
        'R1ThermalCell', 'P.power.cell.R1_tab', 'R1ThermalCellPercentDeviation', nz, ...
        'Tau1ThermalCell', 'P.power.cell.tau1_tab', 'Tau1ThermalCellPercentDeviation', nz, ...
        'BatteryThermalMassCell', 'P.power.cell.Cth', 'BatteryThermalMassCellPercentDeviation', nz, ...
        'AmbientResistance', 'P.power.pack.R_amb_cell', ...
        'NonCellElectricalResistance', 'P.power.pack.R_noncell_total', ...
        'CellBalancingClosedResistance', 'P.power.pack.bal_Rsw_cell', ...
        'CellBalancingOpenConductance', 'P.power.pack.bal_G_open', ...
        'CellBalancingThreshold', 'P.power.pack.bal_threshold', ...
        'CellBalancingResistance', 'P.power.pack.bal_R_cell', ...
        'socCell_specify', 'on', 'socCell_priority', 'High', 'socCell', 'P.power.pack.var.SOC0_vec(:)', ...
        'batteryTemperature_specify', 'on', 'batteryTemperature_priority', 'High', ...
        'batteryTemperature', 'P.power.thermal.T_init_K*ones(P.power.pack.Ns,1)');
else
    load_system('Shaddad_Battery_Lumped_lib');
    mb = shaddad_bu('add', 'Shaddad_Battery_Lumped_lib/Shaddad_Pack_Lumped', [name '/Module']);
    set_param(mb, ...
        'ExtrapolationMethodCell', 'simscape.enum.extrapolation.linear', ...
        'BatteryCapacityCell', 'P.power.cell.Q_Ah*P.power.pack.age.cap_factor', ...
        'SOCBreakpointsCell', 'P.power.cell.SOC_bp', 'TemperatureBreakpointsCell', 'P.power.cell.T_bp', ...
        'OpenCircuitVoltageThermalCell', 'P.power.cell.OCV_tab', 'VoltageRangeCell', 'P.power.cell.VoltageRange', ...
        'ResistanceSOCBreakpointsCell', 'P.power.cell.SOC_bp', 'ResistanceTemperatureBreakpointsCell', 'P.power.cell.T_bp', ...
        'R0ThermalCell', 'P.power.cell.R0_tab*P.power.pack.age.R0_factor', 'R1ThermalCell', 'P.power.cell.R1_tab', ...
        'Tau1ThermalCell', 'P.power.cell.tau1_tab', 'BatteryThermalMassCell', 'P.power.cell.Cth', ...
        'AmbientResistance', 'P.power.pack.R_amb_cell', 'NonCellElectricalResistance', 'P.power.pack.R_noncell_total', ...
        'socCell_specify', 'on', 'socCell_priority', 'High', 'socCell', 'mean(P.power.pack.var.SOC0_vec)', ...
        'batteryTemperature_specify', 'on', 'batteryTemperature_priority', 'High', ...
        'batteryTemperature', 'P.power.thermal.T_init_K');
end
pm = get_param(mb, 'PortHandles');
% ---- electrical / thermal terminals (port order found by domain-validated connection) ----
local_plink(name, local_cp(cpP), pm.LConn);
local_plink(name, local_cp(cpN), pm.RConn);
local_plink(name, local_cp(cpA), pm.LConn);
% ---- balancing command: 0.01 s -> 1 ms (local solver rate) -> PS input enableCellBalancing ----
if ~lumped
    shaddad_bu('add', 'simulink/Signal Attributes/Rate Transition', [name '/RT_bal'], ...
        'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.base');
    shaddad_bu('add', 'simulink/Signal Attributes/Data Type Conversion', [name '/Bal_double'], 'OutDataTypeStr', 'double');
    sb = shaddad_bu('add', SPS, [name '/Bal_PS'], 'Unit', '1', 'FilteringAndDerivatives', 'zero');
    shaddad_bu('line', name, 'Bal_Cmd/1', 'RT_bal/1', 'Bal_Cmd');
    shaddad_bu('line', name, 'RT_bal/1', 'Bal_double/1', 'Bal_Cmd_1ms');
    shaddad_bu('line', name, 'Bal_double/1', 'Bal_PS/1', 'Bal_Cmd_dbl');
    psb = get_param(sb, 'PortHandles');
    local_plink(name, psb.RConn(1), pm.LConn);
else
    shaddad_bu('add', 'simulink/Sinks/Terminator', [name '/Bal_unused']);
    shaddad_bu('line', name, 'Bal_Cmd/1', 'Bal_unused/1', 'Bal_Cmd');
end
% ---- truth via Probe (variables sorted lexicographically) ----
pr = shaddad_bu('add', 'nesl_utility/Probe', [name '/Probe']);
simscape.probe.setBoundBlock(pr, mb);
simscape.probe.setVariables(pr, ["batteryTemperature", "socParallelAssembly", "vParallelAssembly"]);
% probe outputs: 1 batteryTemperature, 2 socParallelAssembly, 3 vParallelAssembly
if lumped
    shaddad_bu('add', 'simulink/Math Operations/Gain', [name '/T_expand'], 'Gain', 'ones(P.power.pack.Ns,1)', ...
        'Multiplication', 'Element-wise(K.*u)');
    shaddad_bu('line', name, 'Probe/1', 'T_expand/1', 'batteryTemperature');
    shaddad_bu('line', name, 'T_expand/1', 'T_cell_true/1', 'T_cell_true');
else
    shaddad_bu('line', name, 'Probe/1', 'T_cell_true/1', 'T_cell_true');
end
shaddad_bu('line', name, 'Probe/2', 'SOC_true/1', 'SOC_true');
shaddad_bu('line', name, 'Probe/3', 'V_cell_true/1', 'V_cell_true');
if lumped
    d = ['Pack_Lumped (VC_PACK == 2, T04): 16S3P LFP Battery Builder module, ModelResolution Lumped ' ...
         '(1 cell model), no variation, no balancing (Bal_Cmd terminated). Truth vectors from the module ' ...
         'variables vParallelAssembly / socParallelAssembly (16 equal groups) and batteryTemperature x ones(16,1).'];
else
    d = ['Pack_Grouped (VC_PACK == 1, T04): 16S3P LFP Battery Builder module, ModelResolution Grouped ' ...
         '(16 parallel-assembly models), passive balancing (Bal_Cmd -> enableCellBalancing, R_bal x Np at the ' ...
         'cell node = 0.17 A per channel), cell-based ambient path (AmbH), PercentDeviation capacity / R0.'];
end
set_param(name, 'Description', [d ' Ports: pack_p/pack_n electrical, AmbH thermal (single node); in Bal_Cmd ' ...
    'boolean[16] @0.01 s; out T_cell_true[16] K, SOC_true[16], V_cell_true[16] V (Probe, double, in the ' ...
    'Probe''s variable order).']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ===========================================================================================
%  Pack_Thermal (spec 3.2 T1-T10, D45 dock air)
% ===========================================================================================
function local_thermal(name, f)
TH = 'Connection: foundation.thermal.thermal';
SPS = sprintf('nesl_utility/Simulink-PS\nConverter');
PSS = sprintf('nesl_utility/PS-Simulink\nConverter');
L_M = 'fl_lib/Thermal/Thermal Elements/Thermal Mass';
L_C = 'fl_lib/Thermal/Thermal Elements/Convective Heat Transfer';
L_R = 'fl_lib/Thermal/Thermal Elements/Radiative Heat Transfer';
L_TS = 'fl_lib/Thermal/Thermal Sources/Controlled Temperature Source';
L_QS = 'fl_lib/Thermal/Thermal Sources/Controlled Heat Flow Rate Source';
L_SN = 'fl_lib/Thermal/Thermal Sensors/Temperature Sensor';
L_REF = 'fl_lib/Thermal/Thermal Elements/Thermal Reference';
shaddad_bu('fresh', name, f, 'ssr');
% ---- ports: in 1 bms_cmd (Bus_BMS_Cmd: Fan_Cmd, 0.01 s), 2 env (Bus_EnvState: T_amb, G_solar,
%      dock_contact, 0.1 s), each element picked in front of its rate transition (the order of Power_Plant's
%      root ports bms_cmd 4, env 5); out thermal (T_air, T_wall, Fan_eff); Connection Port AmbH ----
cpA = shaddad_bu('add', 'nesl_utility/Connection Port', [name '/AmbH'], 'Port', '1', 'Side', 'Left', 'ConnectionType', TH);
PK = @(port, el, dst, varargin) shaddad_bu('pick', name, port, el, dst, varargin{:});
L = @(s, d, nm) shaddad_bu('line', name, s, d, nm);
% ---- discrete logic (rates -> 1 ms local-solver rate through explicit RTs) ----
RT = @(n, ts) shaddad_bu('add', 'simulink/Signal Attributes/Rate Transition', [name '/' n], ...
    'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', ts);
RT('RT_Tamb', 'Ts.base'); RT('RT_G', 'Ts.base'); RT('RT_fan', 'Ts.base'); RT('RT_dock', 'Ts.base');
PK('env', 'T_amb', 'RT_Tamb/1');
PK('env', 'G_solar', 'RT_G/1');
PK('bms_cmd', 'Fan_Cmd', 'RT_fan/1');
PK('env', 'dock_contact', 'RT_dock/1');
shaddad_bu('add', 'simulink/Signal Attributes/Data Type Conversion', [name '/Fan_double'], 'OutDataTypeStr', 'double');
L('RT_fan/1', 'Fan_double/1', 'Fan_Cmd_1ms');
shaddad_bu('add', 'simulink/Sources/Constant', [name '/DockAir_enable'], 'Value', 'P.power.thermal.dock_air.enable', ...
    'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
shaddad_bu('add', 'simulink/Sources/Constant', [name '/DockAir_fan_force'], 'Value', 'P.power.thermal.dock_air.fan_force', ...
    'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
% the ANDs stand 30 px per input, room for the name labels of their input lines (L6)
shaddad_bu('add', 'simulink/Logic and Bit Operations/Logical Operator', [name '/Dock_air_on'], 'Operator', 'AND', ...
    'Inputs', '2', 'Position', [0 0 30 60]);
shaddad_bu('add', 'simulink/Logic and Bit Operations/Logical Operator', [name '/Fan_forced'], 'Operator', 'AND', ...
    'Inputs', '2', 'Position', [0 0 30 60]);
L('RT_dock/1', 'Dock_air_on/1', 'dock_1ms');
L('DockAir_enable/1', 'Dock_air_on/2', 'dock_air_enable');
L('Dock_air_on/1', 'Fan_forced/1', 'dock_air_on');
L('DockAir_fan_force/1', 'Fan_forced/2', 'dock_air_fan_force');
% fan-inlet temperature: Switch(dock air on, dock_air.T_air, T_amb)
shaddad_bu('add', 'simulink/Sources/Constant', [name '/T_dock_air'], 'Value', 'P.power.thermal.dock_air.T_air', 'SampleTime', 'Ts.base');
shaddad_bu('add', 'simulink/Signal Routing/Switch', [name '/T_inlet_sw'], 'Criteria', 'u2 ~= 0');
L('T_dock_air/1', 'T_inlet_sw/1', 'T_dock_air');
L('Dock_air_on/1', 'T_inlet_sw/2', 'dock_air_on');
L('RT_Tamb/1', 'T_inlet_sw/3', 'T_amb_1ms');
% effective fan command: forced to 1 while docked with dock air (D45)
shaddad_bu('add', 'simulink/Sources/Constant', [name '/Fan_one'], 'Value', '1', 'SampleTime', 'Ts.base');
shaddad_bu('add', 'simulink/Signal Routing/Switch', [name '/Fan_sw'], 'Criteria', 'u2 ~= 0');
local_height([name '/Fan_sw'], 90);    % 30 px per input (graphical; fix round 1 trial: 2 crossings fewer)
shaddad_bu('add', 'simulink/Discontinuities/Saturation', [name '/Fan_sat'], 'UpperLimit', '1', 'LowerLimit', '0');
L('Fan_one/1', 'Fan_sw/1', 'Fan_one');
L('Fan_forced/1', 'Fan_sw/2', 'fan_forced');
L('Fan_double/1', 'Fan_sat/1', 'Fan_Cmd_dbl');
L('Fan_sat/1', 'Fan_sw/3', 'Fan_Cmd_sat');
% h_fan * Fan_eff * k_filter
shaddad_bu('add', 'simulink/Math Operations/Gain', [name '/h_fan'], 'Gain', 'P.power.thermal.h_fan*P.power.thermal.k_filter');
L('Fan_sw/1', 'h_fan/1', 'Fan_eff');
% solar heat into the wall: alpha * A_sol * G
shaddad_bu('add', 'simulink/Math Operations/Gain', [name '/Q_solar'], 'Gain', 'P.power.thermal.alpha_sol*P.power.thermal.A_sol');
L('RT_G/1', 'Q_solar/1', 'G_solar_1ms');
% ---- Simulink -> PS ----
% (no input filtering: a filtered Simulink-PS input is rejected by Simscape as an 'unsupported internal
%  rate transition' when the local solver runs at the 5 ms mission rate; the temperature sources drive
%  convective / radiative elements only, so they need no input derivative)
s_amb = shaddad_bu('add', SPS, [name '/T_amb_PS'], 'Unit', 'K', 'FilteringAndDerivatives', 'zero');
s_in  = shaddad_bu('add', SPS, [name '/T_inlet_PS'], 'Unit', 'K', 'FilteringAndDerivatives', 'zero');
s_h   = shaddad_bu('add', SPS, [name '/h_fan_PS'], 'Unit', 'W/(m^2*K)', 'FilteringAndDerivatives', 'zero');
s_q   = shaddad_bu('add', SPS, [name '/Q_solar_PS'], 'Unit', 'W', 'FilteringAndDerivatives', 'zero');
L('RT_Tamb/1', 'T_amb_PS/1', 'T_amb_1ms');
L('T_inlet_sw/1', 'T_inlet_PS/1', 'T_fan_inlet');
L('h_fan/1', 'h_fan_PS/1', 'h_fan');
L('Q_solar/1', 'Q_solar_PS/1', 'Q_solar');
% ---- network ----
air  = shaddad_bu('add', L_M, [name '/AIR'],  'mass', 'P.power.thermal.m_int',  'sp_heat', 'P.power.thermal.cp_int', ...
    'T_specify', 'on', 'T_priority', 'High', 'T', 'P.power.thermal.T_init_K');
wall = shaddad_bu('add', L_M, [name '/WALL'], 'mass', 'P.power.thermal.m_wall', 'sp_heat', 'P.power.thermal.cp_wall', ...
    'T_specify', 'on', 'T_priority', 'High', 'T', 'P.power.thermal.T_init_K');
caw  = shaddad_bu('add', L_C, [name '/Conv_AIR_WALL'], 'area', 'P.power.thermal.A_int', 'heat_tr_coeff', 'P.power.thermal.h_in');
cfan = shaddad_bu('add', L_C, [name '/Fan_ventilation'], 'thermal_type', 'foundation.enum.constant_variable.variable', ...
    'area', 'P.power.thermal.A_int', 'min_heat_tr_coeff', '1e-3');
cext = shaddad_bu('add', L_C, [name '/Conv_WALL_AMB'], 'area', 'P.power.thermal.A_ext', 'heat_tr_coeff', 'P.power.thermal.h_ext');
rad  = shaddad_bu('add', L_R, [name '/Rad_WALL_AMB'], 'area', 'P.power.thermal.A_ext', 'rad_tr_coeff', 'P.power.thermal.rad_tr_coeff');
tamb = shaddad_bu('add', L_TS, [name '/Ambient']);
tin  = shaddad_bu('add', L_TS, [name '/Fan_inlet_air']);
qsol = shaddad_bu('add', L_QS, [name '/Solar_load']);
sair = shaddad_bu('add', L_SN, [name '/T_sens_AIR'],  'temperature_measure', 'foundation.enum.MeasurementReference.absolute');
swal = shaddad_bu('add', L_SN, [name '/T_sens_WALL'], 'temperature_measure', 'foundation.enum.MeasurementReference.absolute');
ref1 = shaddad_bu('add', L_REF, [name '/Ref_amb']);
ref2 = shaddad_bu('add', L_REF, [name '/Ref_inlet']);
ref3 = shaddad_bu('add', L_REF, [name '/Ref_solar']);
ps = @(b) get_param(b, 'PortHandles');
P_air = ps(air); P_wall = ps(wall); P_caw = ps(caw); P_cfan = ps(cfan); P_cext = ps(cext); P_rad = ps(rad);
P_tamb = ps(tamb); P_tin = ps(tin); P_q = ps(qsol); P_sa = ps(sair); P_sw = ps(swal);
P_r1 = ps(ref1); P_r2 = ps(ref2); P_r3 = ps(ref3);
nA = P_air.LConn(1); nW = P_wall.LConn(1);
pl = @(a, b) shaddad_bu('pline', name, a, b);
pl(local_cp(cpA), nA);                         % cells -> AIR (module AmbH)
pl(P_caw.LConn(1), nA);  pl(P_caw.RConn(1), nW);
pl(P_cfan.LConn(1), nA); pl(P_cfan.RConn(1), P_tin.LConn(1));     % convection var: LConn(1) A, LConn(2) K, RConn(1) B
pl(P_cext.LConn(1), nW); pl(P_cext.RConn(1), P_tamb.LConn(1));
pl(P_rad.LConn(1), nW);  pl(P_rad.RConn(1), P_tamb.LConn(1));
pl(P_tamb.RConn(2), P_r1.LConn(1));            % controlled T source: LConn(1) B (= S), RConn(1) S, RConn(2) A
pl(P_tin.RConn(2), P_r2.LConn(1));
pl(P_q.LConn(1), nW); pl(P_q.RConn(2), P_r3.LConn(1));             % heat flow A (reference) -> B (WALL)
pl(P_sa.LConn(1), nA); pl(P_sw.LConn(1), nW);
pl(ps(s_amb).RConn(1), P_tamb.RConn(1));
pl(ps(s_in).RConn(1), P_tin.RConn(1));
pl(ps(s_h).RConn(1), P_cfan.LConn(2));
pl(ps(s_q).RConn(1), P_q.RConn(1));
o_air = shaddad_bu('add', PSS, [name '/T_air_out'], 'Unit', 'K');
o_wal = shaddad_bu('add', PSS, [name '/T_wall_out'], 'Unit', 'K');
pl(P_sa.RConn(1), ps(o_air).LConn(1));
pl(P_sw.RConn(1), ps(o_wal).LConn(1));
% ---- output bus thermal ----
shaddad_bu('outbus', name, 'thermal', 'T_air', 'T_air_out/1');
shaddad_bu('outbus', name, 'thermal', 'T_wall', 'T_wall_out/1');
shaddad_bu('outbus', name, 'thermal', 'Fan_eff', 'Fan_sw/1');
shaddad_bu('portorder', name, {'bms_cmd', 'env'}, 'Inport');
set_param(name, 'Description', ['Pack_Thermal (T04, spec 2.3 / 3.2 T1-T10, D45). Thermal port AmbH (cells -> AIR node). ' ...
    'Inputs (In Bus Element ports, each element picked in front of its rate transition): 1 bms_cmd (Bus_BMS_Cmd, ' ...
    '0.01 s: Fan_Cmd 0..1), 2 env (Bus_EnvState, 0.1 s: T_amb K, G_solar W/m^2, dock_contact boolean). Output bus thermal ' ...
    '(Out Bus Element): T_air K, T_wall K, Fan_eff 0..1 (effective fan command incl. dock forcing, for the 24 V fan ' ...
    'load). Fan-inlet air = dock_air.T_air while docked and dock_air.enable, else T_amb; fans forced to 1 while ' ...
    'docked when dock_air.fan_force.']);
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ===========================================================================================
%  Power_Sensors (spec 2.10 / 3.8 M1-M7, D40)
% ===========================================================================================
function local_sensors(name, f)
shaddad_bu('fresh', name, f, 'ssr');
% ---- inputs: one bus per source block of Power_Plant (In Bus Element ports, model layout spec 6.2-6.3),
%      every consumer picking its element right in front of it (documented in the Description) ----
ports = {'pack', 'thermal', 'pdu', 'lv', 'safety', 'impl', 'charger', 'env', 'flt'};   % the producers' order
shaddad_bu('add', 'simulink/Sinks/Out1', [name '/bms_meas'], 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_BMS_Meas');
shaddad_bu('add', 'simulink/Sinks/Out1', [name '/power_status'], 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_PowerStatus');
shaddad_bu('add', 'simulink/Sinks/Out1', [name '/power_truth'], 'Port', '3', 'OutDataTypeStr', 'Bus: Bus_PowerTruth');

A = @(lib, n, varargin) shaddad_bu('add', lib, [name '/' n], varargin{:});
L = @(s, d, varargin) shaddad_bu('nline', name, s, d, varargin{:});
PK = @(port, el, dst, varargin) shaddad_bu('pick', name, port, el, dst, varargin{:});
ZOH = @(n, ts) A('simulink/Discrete/Zero-Order Hold', n, 'SampleTime', ts);
DTC = @(n, t) A('simulink/Signal Attributes/Data Type Conversion', n, 'OutDataTypeStr', t, 'RndMeth', 'Nearest', ...
    'SaturateOnIntegerOverflow', 'on');
DMX = @(n, k) A('simulink/Signal Routing/Demux', n, 'Outputs', num2str(k));

% ================= BMS measurement models (0.01 s) =================
% cell voltages: + flt.bms_fault x 0.5 V on group 1, ZOH, + offset + noise, quantize, single
A('simulink/Signal Attributes/Data Type Conversion', 'bms_fault_dbl', 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Gain', 'bms_fault_V', 'Gain', '[0.5; zeros(P.power.pack.Ns-1,1)]', 'Multiplication', 'Element-wise(K.*u)');
A('simulink/Math Operations/Sum', 'Vcell_fault', 'Inputs', '++');
PK('flt', 'bms_fault', 'bms_fault_dbl/1');
L('bms_fault_dbl/1', 'bms_fault_V/1', 'bms_fault_dbl');
PK('pack', 'V_cell_true', 'Vcell_fault/1');
L('bms_fault_V/1', 'Vcell_fault/2', 'dV_bms_fault');
ZOH('Vcell_ZOH', 'Ts.afe');
L('Vcell_fault/1', 'Vcell_ZOH/1', 'V_cell_fault');
local_noise_chain(name, 'Vcell', 'Vcell_ZOH/1', 'P.power.sensors.Vcell.offset(:)', 'P.power.sensors.Vcell.sigma', ...
    '(1:P.power.pack.Ns)''', 'P.power.sensors.Vcell.q', 'P.power.pack.Ns');
% cell NTCs: ZOH, 5 s lag (at the 10 ms AFE rate: tau >> Ts), + offset + noise, quantize
ZOH('Tcell_ZOH', 'Ts.afe');
PK('pack', 'T_cell_true', 'Tcell_ZOH/1');
A('ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)', 'Tcell_NTC_lag', 'K', '1', ...
    'T', 'P.power.sensors.Tcell.tau', 'Ts', 'Ts.afe');
L('Tcell_ZOH/1', 'Tcell_NTC_lag/1');
local_noise_chain(name, 'Tcell', 'Tcell_NTC_lag/1', 'P.power.sensors.Tcell.offset(:)', 'P.power.sensors.Tcell.sigma', ...
    '(P.power.pack.Ns+1:2*P.power.pack.Ns)''', 'P.power.sensors.Tcell.q', 'P.power.pack.Ns');
% pack current (shunt): gain error + offset, 1 ms amplifier filter, ZOH, + noise, quantize
A('simulink/Math Operations/Gain', 'Ipack_gain', 'Gain', '1 + P.power.sensors.Ipack.gain_err');
A('simulink/Math Operations/Bias', 'Ipack_bias', 'Bias', 'P.power.sensors.Ipack.offset');
A('ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)', 'Ipack_LPF', 'K', '1', ...
    'T', 'P.power.sensors.Ipack.tau', 'Ts', 'Ts.base');
PK('pdu', 'I_pack', 'Ipack_gain/1');
L('Ipack_gain/1', 'Ipack_bias/1');
L('Ipack_bias/1', 'Ipack_LPF/1');
ZOH('Ipack_ZOH', 'Ts.afe');
L('Ipack_LPF/1', 'Ipack_ZOH/1');
local_noise_chain(name, 'Ipack', 'Ipack_ZOH/1', '0', 'P.power.sensors.Ipack.sigma', '33', 'P.power.sensors.Ipack.q', '1');
% pack / bus / dock voltage dividers: gain error, ZOH, + noise, quantize
vn = {'V_pack', 'Vpack', '34'; 'V_bus', 'Vbus', '35'; 'V_dock', 'Vdock', '36'};
for k = 1:size(vn, 1)
    A('simulink/Math Operations/Gain', [vn{k, 2} '_gain'], 'Gain', '1 + P.power.sensors.Vpack.gain_err');
    PK('pdu', vn{k, 1}, [vn{k, 2} '_gain/1']);
    ZOH([vn{k, 2} '_ZOH'], 'Ts.afe');
    L([vn{k, 2} '_gain/1'], [vn{k, 2} '_ZOH/1']);
    local_noise_chain(name, vn{k, 2}, [vn{k, 2} '_ZOH/1'], '0', 'P.power.sensors.Vpack.sigma', vn{k, 3}, ...
        'P.power.sensors.Vpack.q', '1');
end
% enclosure-air and ambient NTCs
ZOH('Tair_ZOH', 'Ts.afe');
PK('thermal', 'T_air', 'Tair_ZOH/1');
A('ee_sl_lib/General Control/Low-Pass Filter (Discrete or Continuous)', 'Tair_NTC_lag', 'K', '1', ...
    'T', 'P.power.sensors.Tcell.tau', 'Ts', 'Ts.afe');
L('Tair_ZOH/1', 'Tair_NTC_lag/1');
local_noise_chain(name, 'Tair', 'Tair_NTC_lag/1', '0', 'P.power.sensors.Tcell.sigma', '37', 'P.power.sensors.Tcell.q', '1');
A('simulink/Signal Attributes/Rate Transition', 'Tamb_RT', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.afe');
PK('env', 'T_amb', 'Tamb_RT/1');
local_noise_chain(name, 'Tamb', 'Tamb_RT/1', '0', 'P.power.sensors.Tcell.sigma', '38', 'P.power.sensors.Tcell.q', '1');
% fuse bit word (M7): sum((~fuse_ok) .* 2.^(0:6)) -> uint8, fuse_ok = [PDU fuses MAIN TL TR IMP AUX CHG; LV 24IMP]
A('simulink/Signal Routing/Mux', 'Fuse_ok', 'Inputs', '2');
PK('pdu', 'fuse_ok', 'Fuse_ok/1');
PK('lv', 'fuse24_ok', 'Fuse_ok/2');
A('simulink/Logic and Bit Operations/Logical Operator', 'fuse_blown_bits', 'Operator', 'NOT');
A('simulink/Signal Attributes/Data Type Conversion', 'fuse_bits_dbl', 'OutDataTypeStr', 'double');
A('simulink/Math Operations/Gain', 'fuse_weights', 'Gain', '2.^(0:6)', 'Multiplication', 'Matrix(K*u)');
DTC('Fuse_Blown_u8', 'uint8');
L('Fuse_ok/1', 'fuse_blown_bits/1', 'fuse_ok');
L('fuse_blown_bits/1', 'fuse_bits_dbl/1');
L('fuse_bits_dbl/1', 'fuse_weights/1');
L('fuse_weights/1', 'Fuse_Blown_u8/1', 'fuse_word');
% discrete status into the BMS (0.01 s)
dsig = {'K1_fb', 'pdu', 'K1_closed'; 'K2_fb', 'pdu', 'K2_closed'; 'K4_fb', 'pdu', 'K4_closed'; ...
        'Dock_Present', 'safety', 'dock_present'; 'Safety_OK', 'safety', 'Safety_OK'; ...
        'Safety_HV_OK', 'safety', 'Safety_HV_OK'; 'Key_On', 'safety', 'key_on'; 'FuseMain_OK', 'pdu', 'F_MAIN_ok'};
for k = 1:size(dsig, 1)
    ZOH(['meas_' dsig{k, 1}], 'Ts.afe');
    PK(dsig{k, 2}, dsig{k, 3}, ['meas_' dsig{k, 1} '/1']);
end
ZOH('meas_Fuse_Blown', 'Ts.afe');
L('Fuse_Blown_u8/1', 'meas_Fuse_Blown/1', 'Fuse_Blown');
% Bus_BMS_Meas
mel = {'V_cell_V', 'Vcell_q'; 'T_cell_K', 'Tcell_q'; 'I_pack_A', 'Ipack_q'; 'V_pack_V', 'Vpack_q'; ...
       'V_bus_V', 'Vbus_q'; 'V_dock_V', 'Vdock_q'; 'T_amb_K', 'Tamb_q'; 'T_air_K', 'Tair_q'; ...
       'K1_fb', 'meas_K1_fb'; 'K2_fb', 'meas_K2_fb'; 'K4_fb', 'meas_K4_fb'; 'Dock_Present', 'meas_Dock_Present'; ...
       'Safety_OK', 'meas_Safety_OK'; 'Safety_HV_OK', 'meas_Safety_HV_OK'; 'Key_On', 'meas_Key_On'; ...
       'FuseMain_OK', 'meas_FuseMain_OK'; 'Fuse_Blown', 'meas_Fuse_Blown'};
local_buscreator(name, 'BMS_Meas_BC', 'Bus_BMS_Meas', mel);
L('BMS_Meas_BC/1', 'bms_meas/1', 'bms_meas');

% ================= Bus_PowerStatus (true physical values, 1 ms) =================
DMX('rails_dmx', 7); PK('lv', 'rails', 'rails_dmx/1');
DMX('rail_dmx', 4); PK('lv', 'rail_ok', 'rail_dmx/1');
% each row: element, then either {port, element} of a pick or the source 'Blk/k'
sel = {'V_pack_V', {'pdu', 'V_pack'}; 'I_pack_A', {'pdu', 'I_pack'}; 'V_bus_V', {'pdu', 'V_bus'}; ...
       'V_24V_V', 'rails_dmx/1'; 'I_24V_A', 'rails_dmx/2'; 'V_24IMP_V', 'rails_dmx/3'; 'V_12V_V', 'rails_dmx/4'; ...
       'I_12V_A', 'rails_dmx/5'; 'V_5V_V', 'rails_dmx/6'; 'V_3V3_V', 'rails_dmx/7'; ...
       'Rail_24V_OK', 'rail_dmx/1'; 'Rail_12V_OK', 'rail_dmx/2'; 'Rail_5V_OK', 'rail_dmx/3'; 'Rail_3V3_OK', 'rail_dmx/4'; ...
       'K1_Closed', {'pdu', 'K1_closed'}; 'K2_Closed', {'pdu', 'K2_closed'}; 'K3_Closed', {'lv', 'K3_closed'}; ...
       'K4_Closed', {'pdu', 'K4_closed'}; 'Safety_OK', {'safety', 'Safety_OK'}; 'Safety_HV_OK', {'safety', 'Safety_HV_OK'}; ...
       'EStop_Any', {'safety', 'EStop_Any'}; 'EStop_Ch1_OK', {'safety', 'EStop_Ch1_OK'}; ...
       'EStop_Ch2_OK', {'safety', 'EStop_Ch2_OK'}; 'Safety_Latched_Off', {'safety', 'Safety_Latched_Off'}; ...
       'Dock_Present', {'safety', 'dock_present'}; 'Key_On', {'safety', 'key_on'}; 'Fuse_Blown', 'Fuse_Blown_u8/1'};
rn = {'V_24V', 'I_24V', 'V_24IMP', 'V_12V', 'I_12V', 'V_5V', 'V_3V3'};
on = {'Rail_24V_OK', 'Rail_12V_OK', 'Rail_5V_OK', 'Rail_3V3_OK'};
st = cell(size(sel, 1), 2);
for k = 1:size(sel, 1)
    ZOH(['st_' sel{k, 1}], 'Ts.base');
    s = sel{k, 2};
    if iscell(s)
        PK(s{1}, s{2}, ['st_' sel{k, 1} '/1']);
    elseif startsWith(s, 'rails_dmx/')
        L(s, ['st_' sel{k, 1} '/1'], rn{str2double(extractAfter(s, '/'))});
    elseif startsWith(s, 'rail_dmx/')
        L(s, ['st_' sel{k, 1} '/1'], on{str2double(extractAfter(s, '/'))});
    else
        L(s, ['st_' sel{k, 1} '/1']);
    end
    st(k, :) = {sel{k, 1}, ['st_' sel{k, 1}]};
end
local_buscreator(name, 'PowerStatus_BC', 'Bus_PowerStatus', st);
L('PowerStatus_BC/1', 'power_status/1', 'power_status');

% ================= Bus_PowerTruth (1 ms computations, ZOH 0.1 s) =================
% pack power and energy integrals (Discrete-Time Integrator, Forward Euler, gain 1/3600, 1 ms)
A('simulink/Math Operations/Product', 'P_pack', 'Inputs', '2');
local_height([name '/P_pack'], 80);    % graphical (fix round 1 trial: 2 crossings fewer)
PK('pdu', 'V_pack', 'P_pack/1'); PK('pdu', 'I_pack', 'P_pack/2');
A('simulink/Discontinuities/Saturation', 'P_out_pos', 'UpperLimit', 'inf', 'LowerLimit', '0');
A('simulink/Math Operations/Gain', 'P_neg', 'Gain', '-1');
A('simulink/Discontinuities/Saturation', 'P_in_pos', 'UpperLimit', 'inf', 'LowerLimit', '0');
L('P_pack/1', 'P_out_pos/1', 'P_pack');
L('P_pack/1', 'P_neg/1', 'P_pack');
L('P_neg/1', 'P_in_pos/1');
% battery internal loss: (sum OCV(SOC_i, T_i) - V_pack) * I_pack
A('simulink/Lookup Tables/2-D Lookup Table', 'OCV_lookup', 'BreakpointsForDimension1', 'P.power.cell.SOC_bp', ...
    'BreakpointsForDimension2', 'P.power.cell.T_bp', 'Table', 'P.power.cell.OCV_tab', 'ExtrapMethod', 'Clip');
PK('pack', 'SOC_true', 'OCV_lookup/1');
PK('pack', 'T_cell_true', 'OCV_lookup/2');
A('simulink/Math Operations/Sum of Elements', 'OCV_sum');
L('OCV_lookup/1', 'OCV_sum/1', 'OCV_cell');
A('simulink/Math Operations/Sum', 'dV_internal', 'Inputs', '+-');
L('OCV_sum/1', 'dV_internal/1', 'OCV_pack'); PK('pdu', 'V_pack', 'dV_internal/2');
A('simulink/Math Operations/Product', 'P_loss_batt', 'Inputs', '2');
L('dV_internal/1', 'P_loss_batt/1'); PK('pdu', 'I_pack', 'P_loss_batt/2');
A('simulink/Math Operations/Sum', 'P_trac', 'Inputs', '++');
PK('pdu', 'P_trac_L', 'P_trac/1'); PK('pdu', 'P_trac_R', 'P_trac/2');
% the two power sums of the PDU and LV_Distribution outputs (moved here from Power_Plant, their only consumer):
% P_lv = rail loads + BMS supply, P_pdu_loss = PDU loss + K3 / F_24IMP drop
A('simulink/Math Operations/Sum', 'P_lv_total', 'Inputs', '++');
PK('lv', 'P_lv', 'P_lv_total/1'); PK('pdu', 'P_bms', 'P_lv_total/2');
A('simulink/Math Operations/Sum', 'P_pdu_total', 'Inputs', '++');
PK('pdu', 'P_pdu_loss', 'P_pdu_total/1'); PK('lv', 'P_k3_loss', 'P_pdu_total/2');
ints = {'E_out', 'P_out_pos/1'; 'E_in', 'P_in_pos/1'; 'E_loss_batt', 'P_loss_batt/1'; 'E_trac_dc', 'P_trac/1'; ...
        'E_impl_dc', {'impl', 'P_impl'}; 'E_lv', 'P_lv_total/1'; 'E_dcdc_loss', {'lv', 'P_dcdc_loss'}; ...
        'E_pdu_loss', 'P_pdu_total/1'; 'E_chg_bus', {'pdu', 'P_chg_bus'}};
inm = {'P_out_pos', 'P_out'; 'P_in_pos', 'P_in'; 'P_loss_batt', 'P_loss_batt'; 'P_trac', 'P_trac'; ...
       'P_lv_total', 'P_lv'; 'P_pdu_total', 'P_pdu_loss'};
for k = 1:size(ints, 1)
    A('simulink/Discrete/Discrete-Time Integrator', ['Int_' ints{k, 1}], 'IntegratorMethod', 'Integration: Forward Euler', ...
        'gainval', '1/3600', 'InitialCondition', '0', 'SampleTime', 'Ts.base');
    s = ints{k, 2};
    if iscell(s)
        PK(s{1}, s{2}, ['Int_' ints{k, 1} '/1']);
    else
        L(s, ['Int_' ints{k, 1} '/1'], inm{strcmp(inm(:, 1), extractBefore(s, '/')), 2});
    end
end
% running maximum of |I_pack| at 1 ms
A('simulink/Math Operations/Abs', 'I_abs');
A('simulink/Math Operations/MinMax Running Resettable', 'I_peak', 'Function', 'max');
A('simulink/Sources/Constant', 'I_peak_reset', 'Value', 'false', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.base');
PK('pdu', 'I_pack', 'I_abs/1');
L('I_abs/1', 'I_peak/1'); L('I_peak_reset/1', 'I_peak/2');
% pack SOC as the BMS defines it: SOC_min / (1 - (SOC_max - SOC_min)), saturated [0, 1]
A('simulink/Math Operations/MinMax', 'SOC_min', 'Function', 'min', 'Inputs', '1');
A('simulink/Math Operations/MinMax', 'SOC_max', 'Function', 'max', 'Inputs', '1');
A('simulink/Math Operations/Sum', 'SOC_den', 'Inputs', '+-+', 'IconShape', 'rectangular', 'Position', [0 0 30 90]);
A('simulink/Sources/Constant', 'one', 'Value', '1', 'SampleTime', 'Ts.base');
A('simulink/Math Operations/Product', 'SOC_pack_div', 'Inputs', '*/');
A('simulink/Discontinuities/Saturation', 'SOC_pack_sat', 'UpperLimit', '1', 'LowerLimit', '0');
PK('pack', 'SOC_true', 'SOC_min/1'); PK('pack', 'SOC_true', 'SOC_max/1');
L('one/1', 'SOC_den/1'); L('SOC_max/1', 'SOC_den/2'); L('SOC_min/1', 'SOC_den/3');
L('SOC_min/1', 'SOC_pack_div/1'); L('SOC_den/1', 'SOC_pack_div/2');
L('SOC_pack_div/1', 'SOC_pack_sat/1');
A('simulink/Math Operations/MinMax', 'T_max', 'Function', 'max', 'Inputs', '1');
local_height([name '/T_max'], 20);     % graphical (fix round 2 trial, t6_fix2_hprobe.log: 44 -> 43)
A('simulink/Math Operations/MinMax', 'T_min', 'Function', 'min', 'Inputs', '1');
PK('pack', 'T_cell_true', 'T_max/1'); PK('pack', 'T_cell_true', 'T_min/1');
tr = {'SOC_true', {'pack', 'SOC_true'}; 'V_cell_true', {'pack', 'V_cell_true'}; 'T_cell_true', {'pack', 'T_cell_true'}; ...
      'SOC_pack_true', 'SOC_pack_sat/1'; 'V_term_V', {'pdu', 'V_pack'}; 'I_pack_A', {'pdu', 'I_pack'}; 'P_pack_W', 'P_pack/1'; ...
      'E_out_Wh', 'Int_E_out/1'; 'E_in_Wh', 'Int_E_in/1'; 'E_loss_batt_Wh', 'Int_E_loss_batt/1'; ...
      'P_grid_W', {'charger', 'P_grid'}; 'P_dcdc_loss_W', {'lv', 'P_dcdc_loss'}; 'P_pdu_loss_W', 'P_pdu_total/1'; ...
      'E_trac_dc_Wh', 'Int_E_trac_dc/1'; 'E_impl_dc_Wh', 'Int_E_impl_dc/1'; 'E_lv_Wh', 'Int_E_lv/1'; ...
      'E_dcdc_loss_Wh', 'Int_E_dcdc_loss/1'; 'E_pdu_loss_Wh', 'Int_E_pdu_loss/1'; 'E_chg_bus_Wh', 'Int_E_chg_bus/1'; ...
      'I_pack_peak_A', 'I_peak/1'; 'T_cell_max_K', 'T_max/1'; 'T_cell_min_K', 'T_min/1'; ...
      'contactor_closed', {'pdu', 'K1_closed'}; 'fuse_blown', 'Fuse_Blown_u8/1'};
tt = cell(size(tr, 1), 2);
for k = 1:size(tr, 1)
    ZOH(['tr_' tr{k, 1}], 'Ts.hk');
    s = tr{k, 2};
    if iscell(s)
        PK(s{1}, s{2}, ['tr_' tr{k, 1} '/1']);
    else
        L(s, ['tr_' tr{k, 1} '/1'], local_truth_name(s));
    end
    tt(k, :) = {tr{k, 1}, ['tr_' tr{k, 1}]};
end
local_buscreator(name, 'PowerTruth_BC', 'Bus_PowerTruth', tt);
L('PowerTruth_BC/1', 'power_truth/1', 'power_truth');
shaddad_bu('portorder', name, ports, 'Inport');
set_param(name, 'Description', sprintf(['Power_Sensors (T04, spec 2.10 / 3.8, D40). Inputs: one bus per source ' ...
    'block of Power_Plant (In Bus Element ports, model layout spec 6.2-6.3; each consumer picks its element; ' ...
    'double unless noted, 1 ms = Ts.base unless noted):\n' ...
    ' 1 pack: SOC_true[16] -, V_cell_true[16] V, T_cell_true[16] K (Pack_* Probe)\n' ...
    ' 2 thermal: T_air K (Pack_Thermal)\n' ...
    ' 3 pdu: V_pack V (pack terminals pack_p-pack_n), V_bus V (after K1), V_dock V, I_pack A (+ discharge, shunt), ' ...
    'K1_closed, K2_closed, K4_closed boolean (aux contacts), fuse_ok boolean[6] = [MAIN TL TR IMP AUX CHG], F_MAIN_ok boolean, P_trac_L W, ' ...
    'P_trac_R W (DC power into DC48_TracL/R), P_chg_bus W (charger power into DOCK_P), P_pdu_loss W (fuses, shunt, ' ...
    'contactors, cables, precharge R, bleed), P_bms W (BMS supply)\n' ...
    ' 4 lv: rails[7] = [V_24V I_24V V_24IMP V_12V I_12V V_5V V_3V3] (V, A), rail_ok boolean[4] = [24V 12V 5V 3V3], ' ...
    'K3_closed boolean, fuse24_ok boolean, P_lv W (rail loads incl. LDO, coils, fans), P_dcdc_loss W, P_k3_loss W\n' ...
    ' 5 safety: Safety_OK, Safety_HV_OK, EStop_Any, EStop_Ch1_OK, EStop_Ch2_OK, Safety_Latched_Off, key_on, ' ...
    'dock_present (boolean, Safety_Chain)\n' ...
    ' 6 impl: P_impl W (IMP48 + V24IMP loads), 7 charger: P_grid W\n' ...
    ' 8 env: T_amb K (Bus_EnvState, 0.1 s), 9 flt: bms_fault boolean (Bus_FaultInject: +0.5 V on V_cell(1)).\n' ...
    'Inside: K_closed = [K1 K2 K3 K4], fuse_ok[7] = [pdu.fuse_ok; lv.fuse24_ok], P_lv = lv.P_lv + pdu.P_bms, ' ...
    'P_pdu_loss = pdu.P_pdu_loss + lv.P_k3_loss.\n' ...
    'Outputs: 1 Bus_BMS_Meas (0.01 s, noise/offset/quantisation/ZOH), 2 Bus_PowerStatus (0.001 s, true values), ' ...
    '3 Bus_PowerTruth (0.1 s; energy integrals and I_pack_peak_A at 1 ms, Forward Euler 1/3600).']));
shaddad_bu('save', name, f);
close_system(name, 0);
end

function nm = local_truth_name(src)
% the name of a truth line from src ('Blk/1'): the signal the block computes ('' keeps the name its line has)
b = extractBefore(src, '/');
switch b
    case 'SOC_pack_sat', nm = 'SOC_pack';
    case 'I_peak', nm = 'I_pack_peak';
    case 'T_max', nm = 'T_cell_max';
    case 'T_min', nm = 'T_cell_min';
    otherwise
        nm = '';
        if startsWith(b, 'Int_'), nm = extractAfter(b, 'Int_'); end
end
end

function local_noise_chain(name, tag, src, offset, sigma, seedIdx, q, w)
% y = Quantize(x + offset + sigma*N(0,1)) -> single (all at Ts.afe); the lines named after their signals
A = @(lib, n, varargin) shaddad_bu('add', lib, [name '/' n], varargin{:});
A('simulink/Sources/Random Number', [tag '_noise'], 'Mean', ['zeros(' w ',1)'], 'Variance', [sigma '^2'], ...
    'Seed', ['P.power.sensors.seed + ' seedIdx], 'SampleTime', 'Ts.afe');
A('simulink/Sources/Constant', [tag '_offset'], 'Value', offset, 'SampleTime', 'Ts.afe');
% a rectangular Sum, its three inputs 30 px apart down its left side in operand order (x, offset, noise), so that
% the three sources stand level with them (a round Sum takes them around its circle, and the sources of the next
% chain cross); IconShape is graphical, the operands stay
A('simulink/Math Operations/Sum', [tag '_sum'], 'Inputs', '+++', 'IconShape', 'rectangular', 'Position', [0 0 30 90]);
A('simulink/Discontinuities/Quantizer', [tag '_quant'], 'QuantizationInterval', q);
A('simulink/Signal Attributes/Data Type Conversion', [tag '_q'], 'OutDataTypeStr', 'single');
% the quantizer 90 px and the conversion 130 px tall (graphical, fix round 1): with them the rows space the eight
% chains apart enough that their lines between the sum, quantizer and conversion columns no longer cross in steps
% (Power_Sensors 68 -> 46 crossings in the trial, work/layout/logs/t6_fix1_psgrid.log)
local_height([name '/' tag '_quant'], 90);
local_height([name '/' tag '_q'], 130);
shaddad_bu('nline', name, src, [tag '_sum/1']);
shaddad_bu('nline', name, [tag '_offset/1'], [tag '_sum/2']);
shaddad_bu('nline', name, [tag '_noise/1'], [tag '_sum/3']);
shaddad_bu('nline', name, [tag '_sum/1'], [tag '_quant/1']);
shaddad_bu('nline', name, [tag '_quant/1'], [tag '_q/1']);
end

function local_height(b, h)
% set block b's height to h px, its width kept (the engine keeps a block's own height; graphical only)
p = get_param(b, 'Position');
set_param(b, 'Position', [p(1) p(2) p(3) p(2) + h]);
end

function local_buscreator(name, bc, bus, el)
% nonvirtual Bus Creator with named input signals (element names = signal names)
n = size(el, 1);
shaddad_bu('add', 'simulink/Signal Routing/Bus Creator', [name '/' bc], 'Inputs', num2str(n), ...
    'OutDataTypeStr', ['Bus: ' bus], 'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
for k = 1:n
    h = add_line(name, [el{k, 2} '/1'], [bc '/' num2str(k)], 'autorouting', 'smart');
    set_param(h, 'Name', el{k, 1});
end
end

% -------------------------------------------------------------------------------------------
function h = local_cp(cpBlk)
ph = get_param(cpBlk, 'PortHandles'); h = ph.RConn(1);
end

function k = local_plink(sys, hSrc, cand)
% connect hSrc to the first candidate port the physical-domain rules accept
for k = 1:numel(cand)
    try
        add_line(sys, hSrc, cand(k), 'autorouting', 'smart');
        return
    catch %#ok<CTCH> wrong domain: try the next port
    end
end
error('build_power_pack:plink', '%s: no compatible physical port found', sys);
end
