%% Battery parameters

%% ParallelAssemblyType1
ParallelAssemblyType1.BatteryCapacityCell = 27; % Battery capacity, A*hr
ParallelAssemblyType1.SOCBreakpointsCell = [0, .1, .25, .5, .75, .9, 1]; % State of charge breakpoints, SOC
ParallelAssemblyType1.TemperatureBreakpointsCell = [278, 293, 313]; % Temperature breakpoints, T, K
ParallelAssemblyType1.OpenCircuitVoltageThermalCell = [3.49, 3.5, 3.51; 3.55, 3.57, 3.56; 3.62, 3.63, 3.64; 3.71, 3.71, 3.72; 3.91, 3.93, 3.94; 4.07, 4.08, 4.08; 4.19, 4.19, 4.19]; % Open-circuit voltage, OCV(SOC,T), V
ParallelAssemblyType1.VoltageRangeCell = [0, inf]; % Terminal voltage operating range, [Min Max], V
ParallelAssemblyType1.ResistanceSOCBreakpointsCell = [0, .1, .25, .5, .75, .9, 1]; % State of charge breakpoints for resistance, SOC
ParallelAssemblyType1.ResistanceTemperatureBreakpointsCell = [278, 293, 313]; % Temperature breakpoints for resistance, T, K
ParallelAssemblyType1.R0ThermalCell = [.0117, .0085, .009; .011, .0085, .009; .0114, .0087, .0092; .0107, .0082, .0088; .0107, .0083, .0091; .0113, .0085, .0089; .0116, .0085, .0089]; % Instantaneous resistance, R0(SOC,T), Ohm
ParallelAssemblyType1.R1ThermalCell = [.0109, .0029, .0013; .0069, .0024, .0012; .0047, .0026, .0013; .0034, .0016, .001; .0033, .0023, .0014; .0033, .0018, .0011; .0028, .0017, .0011]; % First polarization resistance, R1(SOC,T), Ohm
ParallelAssemblyType1.Tau1ThermalCell = [20, 36, 39; 31, 45, 39; 109, 105, 61; 36, 29, 26; 59, 77, 67; 40, 33, 29; 25, 39, 33]; % First time constant, Tau1(SOC,T), s
ParallelAssemblyType1.BatteryThermalMassCell = 100; % Battery thermal mass, J/K
ParallelAssemblyType1.AmbientResistance = 25; % Cell level ambient thermal path resistance, K/W
ParallelAssemblyType1.CellBalancingClosedResistance = 0.01; % Cell balancing switch closed resistance, Ohm
ParallelAssemblyType1.CellBalancingOpenConductance = 1e-8; % Cell balancing switch open conductance, 1/Ohm
ParallelAssemblyType1.CellBalancingThreshold = 0.5; % Cell balancing switch operation threshold
ParallelAssemblyType1.CellBalancingResistance = 4; % Cell balancing shunt resistance, Ohm

%% Shaddad_Pack
Shaddad_Pack.BatteryCapacityCell = 27; % Battery capacity, A*hr
Shaddad_Pack.BatteryCapacityCellPercentDeviation = zeros([13, 1]); % Percent deviation for BatteryCapacityCell
Shaddad_Pack.SOCBreakpointsCell = [0, .1, .25, .5, .75, .9, 1]; % State of charge breakpoints, SOC
Shaddad_Pack.SOCBreakpointsCellPercentDeviation = zeros([13, 1]); % Percent deviation for SOCBreakpointsCell
Shaddad_Pack.TemperatureBreakpointsCell = [278, 293, 313]; % Temperature breakpoints, T, K
Shaddad_Pack.TemperatureBreakpointsCellPercentDeviation = zeros([13, 1]); % Percent deviation for TemperatureBreakpointsCell
Shaddad_Pack.OpenCircuitVoltageThermalCell = [3.49, 3.5, 3.51; 3.55, 3.57, 3.56; 3.62, 3.63, 3.64; 3.71, 3.71, 3.72; 3.91, 3.93, 3.94; 4.07, 4.08, 4.08; 4.19, 4.19, 4.19]; % Open-circuit voltage, OCV(SOC,T), V
Shaddad_Pack.OpenCircuitVoltageThermalCellPercentDeviation = zeros([13, 1]); % Percent deviation for OpenCircuitVoltageThermalCell
Shaddad_Pack.VoltageRangeCell = [0, inf]; % Terminal voltage operating range, [Min Max], V
Shaddad_Pack.VoltageRangeCellPercentDeviation = zeros([13, 1]); % Percent deviation for VoltageRangeCell
Shaddad_Pack.ResistanceSOCBreakpointsCell = [0, .1, .25, .5, .75, .9, 1]; % State of charge breakpoints for resistance, SOC
Shaddad_Pack.ResistanceSOCBreakpointsCellPercentDeviation = zeros([13, 1]); % Percent deviation for ResistanceSOCBreakpointsCell
Shaddad_Pack.ResistanceTemperatureBreakpointsCell = [278, 293, 313]; % Temperature breakpoints for resistance, T, K
Shaddad_Pack.ResistanceTemperatureBreakpointsCellPercentDeviation = zeros([13, 1]); % Percent deviation for ResistanceTemperatureBreakpointsCell
Shaddad_Pack.R0ThermalCell = [.0117, .0085, .009; .011, .0085, .009; .0114, .0087, .0092; .0107, .0082, .0088; .0107, .0083, .0091; .0113, .0085, .0089; .0116, .0085, .0089]; % Instantaneous resistance, R0(SOC,T), Ohm
Shaddad_Pack.R0ThermalCellPercentDeviation = zeros([13, 1]); % Percent deviation for R0ThermalCell
Shaddad_Pack.R1ThermalCell = [.0109, .0029, .0013; .0069, .0024, .0012; .0047, .0026, .0013; .0034, .0016, .001; .0033, .0023, .0014; .0033, .0018, .0011; .0028, .0017, .0011]; % First polarization resistance, R1(SOC,T), Ohm
Shaddad_Pack.R1ThermalCellPercentDeviation = zeros([13, 1]); % Percent deviation for R1ThermalCell
Shaddad_Pack.Tau1ThermalCell = [20, 36, 39; 31, 45, 39; 109, 105, 61; 36, 29, 26; 59, 77, 67; 40, 33, 29; 25, 39, 33]; % First time constant, Tau1(SOC,T), s
Shaddad_Pack.Tau1ThermalCellPercentDeviation = zeros([13, 1]); % Percent deviation for Tau1ThermalCell
Shaddad_Pack.BatteryThermalMassCell = 100; % Battery thermal mass, J/K
Shaddad_Pack.BatteryThermalMassCellPercentDeviation = zeros([13, 1]); % Percent deviation for BatteryThermalMassCell
Shaddad_Pack.AmbientResistance = 25; % Cell level ambient thermal path resistance, K/W
Shaddad_Pack.NonCellElectricalResistance = 1 / 1000; % Non-cell electrical resistance, Ohm
Shaddad_Pack.CellBalancingClosedResistance = 0.01; % Cell balancing switch closed resistance, Ohm
Shaddad_Pack.CellBalancingOpenConductance = 1e-8; % Cell balancing switch open conductance, 1/Ohm
Shaddad_Pack.CellBalancingThreshold = 0.5; % Cell balancing switch operation threshold
Shaddad_Pack.CellBalancingResistance = 4; % Cell balancing shunt resistance, Ohm
