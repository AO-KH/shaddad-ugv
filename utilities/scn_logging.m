function out = scn_logging(op, varargin)
%SCN_LOGGING  Signal-logging manifest of Shaddad_System (integration spec 7.6, owner: integration).
%
%   L  = scn_logging('table')            N x 5 cell {name, block, port, dec_test, Ts_signal}: logged
%                                        output ports (DataLoggingName = name) with the test decimation set
%                                        on the ports by build_shaddad_system; block is the path below
%                                        Shaddad_System, inside the components of the top level (layout
%                                        spec 5.3)
%   mi = scn_logging('override', P, T)   Simulink.SimulationData.ModelLoggingInfo for a run of StopTime T:
%                                        [] (port settings) when T <= P.log.to_file_threshold, otherwise
%                                        every signal decimated to P.log.Ts_mission (missions, 10 Hz)
%   in = scn_logging('apply', in, P, T)  in.setModelParameter('DataLoggingOverride', mi) when mi is not
%                                        empty, plus LoggingToFile for missions
%
%   Signals inside Sim_Monitors (mon_sep_cyl / mon_sep_box / mon_sep_dyn, 0.1 s) are logged by the SSR.

L = {'truth',        'Plant/Shaddad_Plant',              1, 10, 1e-3
     'drive_truth',  'Plant/Shaddad_Plant',              3, 10, 1e-3
     'power_truth',  'Plant/Shaddad_Plant',              4,  1, 0.1
     'env',          'Plant/Shaddad_Plant',             12,  1, 0.1
     'struct_diag',  'Plant/Shaddad_Plant',             16,  1, 1e-2
     'power_status', 'Plant/Shaddad_Plant',             17,  1, 1e-3
     'elec_hw',      'Plant/ElecHw_Delay',               1,  1, 1e-3
     'mcu_out1k',    'MCU/Shaddad_MCU',                  1,  1, 1e-3
     'mcu_out10',    'MCU/Shaddad_MCU',                  3,  1, 1e-2
     'telem',        'MCU/Shaddad_MCU',                  6,  1, 1e-2
     'rc_frame',     'Operator_RC/RC_Link_Sim',          2,  1, 1e-3
     'bms_telem',    'BMS/Shaddad_BMS',                  3,  1, 0.1
     'kpi',          'Sim_Monitors/Sim_Monitors',        1,  1, 0.1
     'nav_state',    'GNC/GNC_Variant',                  2,  1, 1e-2
     'gnc_status',   'GNC/GNC_Variant',                  4,  1, 0.2
     'gnc_debug',    'GNC/GNC_Variant',                  5,  1, 5e-2
     'nav_diag',     'GNC/GNC_Variant',                  6,  1, 1e-2
     'scen_in',      'Spec_ScenarioIn',                  1,  1, 1e-2
     'imu_raw',      'Sensors/Shaddad_SensorModels',     1,  1, 5e-3
     'gnss_raw',     'Sensors/Shaddad_SensorModels',     2,  1, 0.1};
switch lower(op)
    case 'table'
        out = L;
    case 'override'
        [P, T] = varargin{1:2};
        out = local_override(L, P, T);
    case 'apply'
        [in, P, T] = varargin{1:3};
        mi = local_override(L, P, T);
        if ~isempty(mi)
            in = in.setModelParameter('DataLoggingOverride', mi);
        end
        out = in;
    otherwise
        error('scn_logging:op', 'op must be table | override | apply');
end
end

% -------------------------------------------------------------------------------------------
function mi = local_override(L, P, T)
mi = [];
if T <= P.log.to_file_threshold
    return
end
mdl = 'Shaddad_System';
mi = Simulink.SimulationData.ModelLoggingInfo(mdl);
mi.LoggingMode = 'OverrideSignals';
for k = 1:size(L, 1)
    s = Simulink.SimulationData.SignalLoggingInfo([mdl '/' L{k, 2}], L{k, 3});
    s.LoggingInfo.DataLogging = true;
    s.LoggingInfo.NameMode = true;
    s.LoggingInfo.LoggingName = L{k, 1};
    s.LoggingInfo.DecimateData = true;
    s.LoggingInfo.Decimation = max(1, round(P.log.Ts_mission / L{k, 5}));
    mi.Signals(end+1) = s;
end
for nm = {'Min_cyl', 'Min_box', 'Min_dyn'}
    s = Simulink.SimulationData.SignalLoggingInfo([mdl '/Sim_Monitors/Sim_Monitors/' nm{1}], 1);
    s.LoggingInfo.DataLogging = true;
    mi.Signals(end+1) = s;
end
end
