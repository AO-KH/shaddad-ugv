function out = build_stubs(force)
%BUILD_STUBS  Port table of docs/icd.md section 3 and interface stubs of every model / SSR file.
%
%   T = build_stubs('table')   struct array, one row per file (the wiring contract as data):
%                              .name .file (project-relative) .kind ('ssr'|'model') .stub (true = created
%                              here) .owner (build task) .in/.out (struct: name, type, ts) .conn (struct:
%                              name, domain, type (ConnectionType), side)
%   build_stubs()              create every stub file that does not exist yet
%   build_stubs(true)          also recreate files whose Description is 'SHADDAD_STUB'
%                              (a file a domain builder has replaced is never touched)
%   created = build_stubs(...) names of the files written
%
%   Stub contents (build plan T01.3): inputs = Inport (bus type, explicit SampleTime; models also
%   BusOutputAsStruct) -> Terminator; outputs = Constant P.ic.<bus> (port Ts, 0.001 for continuous ports)
%   -> Outport; physical ports = Connection Ports with explicit ConnectionType loaded by the minimal
%   networks of ICD 2 (incl. the network tie of D35). Description = 'SHADDAD_STUB'.
%   Port rates are written as Ts.<field> expressions, values from P.sim.stub / P.drive.tie.

if nargin < 1, force = false; end
T = local_table();
if ischar(force) || isstring(force)
    if strcmpi(force, 'table'), out = T; return, end
    error('build_stubs:arg', 'build_stubs(''table'') | build_stubs(force)');
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
created = {};
for k = 1:numel(T)
    if ~T(k).stub, continue, end
    f = fullfile(root, T(k).file);
    if isfile(f)
        if ~force, continue, end
        try
            info = Simulink.MDLInfo(f);
            isStub = strcmp(strtrim(info.Description), 'SHADDAD_STUB');
        catch
            isStub = false;
        end
        if ~isStub, continue, end
        local_close_users();
    end
    local_make(T(k), f);
    created{end+1} = T(k).name; %#ok<AGROW>
end
if nargout > 0, out = created; end
end

% ===========================================================================================
function local_close_users()
% integration-owned models that reference the stubs (closed so a stub file can be replaced)
for m = {'Shaddad_System', 'Shaddad_Companion', 'Shaddad_Plant'}
    if bdIsLoaded(m{1}), close_system(m{1}, 0); end
end
end

function local_make(R, f)
name = R.name;
isModel = strcmp(R.kind, 'model');
if ~isfolder(fileparts(f)), mkdir(fileparts(f)); end
shaddad_bu('fresh', name, f, R.kind);
if isModel
    shaddad_configsets('attach', name);
end
% ---- inputs ----
for k = 1:numel(R.in)
    p = R.in(k);
    ip = shaddad_bu('add', 'simulink/Sources/In1', [name '/' p.name], 'Port', num2str(k), ...
        'OutDataTypeStr', p.type, 'SampleTime', shaddad_bu('tsexpr', p.ts));
    if isModel && startsWith(p.type, 'Bus:')
        set_param(ip, 'BusOutputAsStruct', 'on');
    end
    shaddad_bu('add', 'simulink/Sinks/Terminator', [name '/Term_' p.name]);
    shaddad_bu('line', name, [p.name '/1'], ['Term_' p.name '/1'], p.name);
end
% ---- outputs ----
for k = 1:numel(R.out)
    p = R.out(k);
    c = ['IC_' p.name];
    tsx = shaddad_bu('tsexpr', p.ts);
    if startsWith(p.type, 'Bus:')
        val = ['P.ic.' strtrim(extractAfter(p.type, 'Bus:'))];
    elseif strcmp(p.type, 'boolean')
        val = 'true';
    else
        val = 'P.sim.stub.T_amb';
    end
    shaddad_bu('add', 'simulink/Sources/Constant', [name '/' c], 'Value', val, ...
        'OutDataTypeStr', p.type, 'SampleTime', tsx);
    op = shaddad_bu('add', 'simulink/Sinks/Out1', [name '/' p.name], 'Port', num2str(k), ...
        'OutDataTypeStr', p.type);
    if isModel
        set_param(op, 'SampleTime', tsx);
        if startsWith(p.type, 'Bus:'), set_param(op, 'BusOutputAsStruct', 'on'); end
    end
    shaddad_bu('line', name, [c '/1'], [p.name '/1'], p.name);
end
% ---- physical ports and minimal networks ----
for k = 1:numel(R.conn)
    c = R.conn(k);
    shaddad_bu('add', 'nesl_utility/Connection Port', [name '/' c.name], ...
        'Port', num2str(k), 'Side', c.side, 'ConnectionType', c.type);
end
if ~isempty(R.conn)
    feval(['local_net_' R.name], name);
end
set_param(name, 'Description', 'SHADDAD_STUB');
shaddad_bu('save', name, f);
close_system(name, 0);
end

% ---- minimal physical networks (build plan T01.3, ICD 2, D35) ------------------------------
function h = cp(name, port)          % Connection Port block handle (its single RConn)
ph = get_param([name '/' port], 'PortHandles'); h = ph.RConn(1);
end
function ph = php(blk)
ph = get_param(blk, 'PortHandles');
end

function local_net_structure_plant(n)
I = 'fl_lib/Mechanical/Rotational Elements/Inertia';
D = 'fl_lib/Mechanical/Rotational Elements/Rotational Damper';
G = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
for s = {'L', 'R'}
    sd = s{1};
    bi = shaddad_bu('add', I, [n '/J_spr_' sd], 'inertia', 'P.sim.stub.J_spr');
    bd = shaddad_bu('add', D, [n '/D_spr_' sd], 'D', 'P.sim.stub.D_spr');
    bg = shaddad_bu('add', G, [n '/Ref_spr_' sd]);
    pi_ = php(bi); pd = php(bd); pg = php(bg);
    shaddad_bu('pline', n, cp(n, ['Spr_' sd]), pi_.LConn(1));
    shaddad_bu('pline', n, cp(n, ['Spr_' sd]), pd.LConn(1));
    shaddad_bu('pline', n, pd.RConn(1), pg.LConn(1));
end
bi = shaddad_bu('add', I, [n '/J_impl'], 'inertia', 'P.sim.stub.J_impl');
pi_ = php(bi);
shaddad_bu('pline', n, cp(n, 'Impl_Shaft'), pi_.LConn(1));
end

function local_net_Power_Plant(n)
V = 'fl_lib/Electrical/Electrical Sources/DC Voltage Source';
Rf = 'fl_lib/Electrical/Electrical Elements/Resistor';
E = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
D = 'fl_lib/Mechanical/Rotational Elements/Rotational Damper';
G = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
bv = shaddad_bu('add', V, [n '/V_pack'], 'v0', 'P.sim.stub.V_bus');
be = shaddad_bu('add', E, [n '/GND']);
pv = php(bv); pe = php(be);
shaddad_bu('pline', n, pv.RConn(1), pe.LConn(1));             % source - to GND
for s = {'L', 'R'}
    sd = s{1};
    br = shaddad_bu('add', Rf, [n '/R_feed_' sd], 'R', 'P.sim.stub.R_feed');
    pr = php(br);
    shaddad_bu('pline', n, pv.LConn(1), pr.LConn(1));         % source + -> feeder
    shaddad_bu('pline', n, pr.RConn(1), cp(n, ['DC48_Trac' sd '_p']));
    shaddad_bu('pline', n, cp(n, ['DC48_Trac' sd '_n']), pe.LConn(1));
end
% Impl_None: damper + network tie (D35)
bd = shaddad_bu('add', D, [n '/D_impl'], 'D', 'P.sim.stub.D_impl');
bg = shaddad_bu('add', G, [n '/Ref_impl']);
pd = php(bd); pg = php(bg);
shaddad_bu('pline', n, cp(n, 'Impl_Shaft'), pd.LConn(1));
shaddad_bu('pline', n, pd.RConn(1), pg.LConn(1));
local_tie(n, 'Tie_impl', cp(n, 'Impl_Shaft'), pg.LConn(1), pe.LConn(1));
end

function local_net_drive_system(n)
Rf = 'fl_lib/Electrical/Electrical Elements/Resistor';
I = 'fl_lib/Mechanical/Rotational Elements/Inertia';
D = 'fl_lib/Mechanical/Rotational Elements/Rotational Damper';
G = sprintf('fl_lib/Mechanical/Rotational Elements/Mechanical\nRotational Reference');
for s = {'L', 'R'}
    sd = s{1};
    hp = cp(n, ['DC48_Trac' sd '_p']); hn = cp(n, ['DC48_Trac' sd '_n']);
    br = shaddad_bu('add', Rf, [n '/R_esc_' sd], 'R', 'P.sim.stub.R_esc');
    pr = php(br);
    shaddad_bu('pline', n, hp, pr.LConn(1));
    shaddad_bu('pline', n, pr.RConn(1), hn);
    bi = shaddad_bu('add', I, [n '/J_drv_' sd], 'inertia', 'P.sim.stub.J_drv');
    bd = shaddad_bu('add', D, [n '/D_drv_' sd], 'D', 'P.sim.stub.D_drv');
    bg = shaddad_bu('add', G, [n '/Ref_drv_' sd]);
    pi_ = php(bi); pd = php(bd); pg = php(bg);
    hs = cp(n, ['Spr_' sd]);
    shaddad_bu('pline', n, hs, pi_.LConn(1));
    shaddad_bu('pline', n, hs, pd.LConn(1));
    shaddad_bu('pline', n, pd.RConn(1), pg.LConn(1));
    local_tie(n, ['Tie_' sd], hs, pg.LConn(1), hn);
end
end

function local_tie(n, nm, hShaft, hRef, hNeg)
% network tie (ICD 2, D35): converter K = P.drive.tie.K, R on the shaft, C on the reference,
% - on DC-/GND, + through ee_lib/Passive/Resistor P.drive.tie.R to DC-/GND
C = sprintf('fl_lib/Electrical/Electrical Elements/Rotational\nElectromechanical\nConverter');
bc = shaddad_bu('add', C, [n '/' nm], 'K', 'P.drive.tie.K');
br = shaddad_bu('add', 'ee_lib/Passive/Resistor', [n '/' nm '_R'], 'R', 'P.drive.tie.R');
pc = php(bc); pr = php(br);
% converter ports: LConn = [p(+) R], RConn = [n(-) C] (rotational_converter.ssc node order)
shaddad_bu('pline', n, pc.LConn(2), hShaft);
shaddad_bu('pline', n, pc.RConn(2), hRef);
shaddad_bu('pline', n, pc.RConn(1), hNeg);
shaddad_bu('pline', n, pc.LConn(1), pr.LConn(1));
shaddad_bu('pline', n, pr.RConn(1), hNeg);
end

% ===========================================================================================
function T = local_table()
E = 'Connection: foundation.electrical.electrical';
M = 'Connection: foundation.mechanical.rotational.rotational';
b = @(x) ['Bus: Bus_' x];
F64 = b('CanFrame64');
T = struct('name', {}, 'file', {}, 'kind', {}, 'stub', {}, 'owner', {}, 'in', {}, 'out', {}, 'conn', {});

% ---------------- plant-side Subsystem References (ICD 3.2) ----------------
T(end+1) = row('structure_plant', 'subsystems/structure/structure_plant.slx', 'ssr', true, 'T03', ...
    {'impl_cmd', b('ImplementCmd'), 0.01; 'env_dist', b('EnvDisturbance'), 0.001; 'env_ovr', b('EnvOverride'), 0.1}, ...
    {'truth', b('TruthState'), 0; 'tracks', b('TrackState'), 0; 'impl_mech', b('ImplMechState'), 0.01; ...
     'env', b('EnvState'), 0.1; 'diag', b('StructureDiag'), 0}, ...
    {'Spr_L', M, 'Left'; 'Spr_R', M, 'Left'; 'Impl_Shaft', M, 'Left'});
T(end+1) = row('Power_Plant', 'subsystems/power/Power_Plant.slx', 'ssr', true, 'T05', ...
    {'hmi', b('HmiInputs'), 0.001; 'hw_cmd', b('PowerHwCmd'), 0.001; 'lv_demand', b('LV_LoadDemand'), 0.01; ...
     'bms_cmd', b('BMS_Cmd'), 0.01; 'env', b('EnvState'), 0.1; 'impl_mech', b('ImplMechState'), 0.01; ...
     'can_rx', b('Can1_McuSlow'), 0.05; 'flt', b('FaultInject'), 0.001}, ...
    {'bms_meas', b('BMS_Meas'), 0.01; 'power_status', b('PowerStatus'), 0.001; 'impl_elec', b('ImplElecState'), 0.01; ...
     'can_tx', b('Can1_Impl'), 0.05; 'power_truth', b('PowerTruth'), 0.1; 'safety_ok', 'boolean', 0.001}, ...
    {'DC48_TracL_p', E, 'Right'; 'DC48_TracL_n', E, 'Right'; 'DC48_TracR_p', E, 'Right'; ...
     'DC48_TracR_n', E, 'Right'; 'Impl_Shaft', M, 'Right'});
T(end+1) = row('drive_system', 'subsystems/drive/drive_system.slx', 'ssr', true, 'T06', ...
    {'can_rx', b('Can1_McuFast'), 0.01; 'sto_ok', 'boolean', 0.001; 'T_amb', 'double', 0.1; ...
     'fault_inj', b('DriveFaultInj'), 0.001}, ...
    {'can_tx_fast', b('Can1_EscFast'), 0.01; 'can_tx_slow', b('Can1_EscSlow'), 0.05; 'truth', b('DriveTruth'), 0.001}, ...
    {'DC48_TracL_p', E, 'Left'; 'DC48_TracL_n', E, 'Left'; 'DC48_TracR_p', E, 'Left'; 'DC48_TracR_n', E, 'Left'; ...
     'Spr_L', M, 'Right'; 'Spr_R', M, 'Right'});
T(end+1) = row('Electronics_Plant', 'subsystems/electronics/Electronics_Plant.slx', 'ssr', true, 'T10', ...
    {'mcu_out1k', b('McuOut1k'), 0.001; 'mcu_out10', b('McuOut10'), 0.01; 'power_status', b('PowerStatus'), 0.001; ...
     'hmi', b('HmiInputs'), 0.001; 'env', b('EnvState'), 0.1; 'impl_mech', b('ImplMechState'), 0.01; ...
     'flt', b('FaultInject'), 0.001}, ...
    {'elec_hw', b('ElecHwIn'), 0.001; 'elec_adc', b('ElecAdc'), 0.05; 'elec_env', b('ElecEnv'), 0.1; ...
     'hw_cmd', b('PowerHwCmd'), 0.001; 'impl_cmd', b('ImplementCmd'), 0.01; 'lv_demand', b('LV_LoadDemand'), 0.01; ...
     'indicators', b('IndicatorOut'), 0.1}, {});

% ---------------- top-level Subsystem References (ICD 3.1) ----------------
T(end+1) = row('RC_Link_Sim', 'subsystems/rc/RC_Link_Sim.slx', 'ssr', true, 'T07', ...
    {'op', b('OperatorCmd'), 0.01; 'flt', b('FaultInject'), 0.01}, ...
    {'sbus', b('SbusStream'), 0.001; 'rc_frame', b('RcFrame'), 0.001}, {});
T(end+1) = row('CAN1_Net_Sim', 'subsystems/electronics/CAN1_Net_Sim.slx', 'ssr', true, 'T10', ...
    {'mcu_fast', b('Can1_McuFast'), 0.01; 'mcu_slow', b('Can1_McuSlow'), 0.05; 'esc_fast', b('Can1_EscFast'), 0.01; ...
     'esc_slow', b('Can1_EscSlow'), 0.05; 'impl', b('Can1_Impl'), 0.05; 'bms', b('Can1_Bms'), 0.1; ...
     'flt', b('FaultInject'), 0.01; 'elec_hw', b('ElecHwIn'), 0.001}, ...
    {'mcu_fast_rx', b('Can1_McuFast'), 0.01; 'mcu_slow_rx', b('Can1_McuSlow'), 0.05; 'esc_fast_rx', b('Can1_EscFast'), 0.01; ...
     'esc_slow_rx', b('Can1_EscSlow'), 0.05; 'impl_rx', b('Can1_Impl'), 0.05; 'bms_rx', b('Can1_Bms'), 0.1}, {});
T(end+1) = row('Shaddad_SensorModels', 'subsystems/sensors/Shaddad_SensorModels.slx', 'ssr', true, 'T11', ...
    {'truth', b('TruthState'), 0.001; 'drive_truth', b('DriveTruth'), 0.001; 'env', b('EnvState'), 0.1; ...
     'elec_env', b('ElecEnv'), 0.1; 'flt', b('FaultInject'), 0.01; 'obs', b('ObstaclesDyn'), 0.01}, ...
    {'imu_raw', b('ImuRaw'), 0.005; 'gnss_raw', b('GnssRaw'), 0.1; 'lidar_scan', b('LidarScanRaw'), 0.1; ...
     'lidar_cloud', b('LidarCloudRaw'), 0.1; 'spray', b('SprayDemand'), 0.1; 'sens_diag', b('SensTruthDiag'), 0.1}, {});
T(end+1) = row('Link_MCU_GNC', 'subsystems/electronics/Link_MCU_GNC.slx', 'ssr', true, 'T10', ...
    {'mcu_imu', F64, 0.005; 'mcu_odom', F64, 0.01; 'mcu_status', F64, 0.05; 'mcu_power', F64, 0.1; ...
     'comp_cmd', F64, 0.05; 'flt', b('FaultInject'), 0.01; 'elec_hw', b('ElecHwIn'), 0.001}, ...
    {'mcu_imu_rx', F64, 0.005; 'mcu_odom_rx', F64, 0.01; 'mcu_status_rx', F64, 0.05; 'mcu_power_rx', F64, 0.1; ...
     'comp_cmd_rx', F64, 0.05}, {});
T(end+1) = row('Viz_Logging', 'subsystems/integration/Viz_Logging.slx', 'ssr', true, 'T16', ...
    {'truth', b('TruthState'), 0.001; 'nav_state', b('NavState'), 0.01; 'gnc_status', b('GncStatus'), 0.2; ...
     'perception', b('Perception'), 0.1; 'telem', b('FwTelemetry'), 0.01; 'indicators', b('IndicatorOut'), 0.1}, {}, {});
T(end+1) = row('Sim_Monitors', 'subsystems/integration/Sim_Monitors.slx', 'ssr', true, 'T16', ...
    {'truth', b('TruthState'), 0.001; 'power_truth', b('PowerTruth'), 0.1; 'drive_truth', b('DriveTruth'), 0.001; ...
     'obs', b('ObstaclesDyn'), 0.01; 'telem', b('FwTelemetry'), 0.01; 'gnc_status', b('GncStatus'), 0.2; ...
     'struct_diag', b('StructureDiag'), 0.01}, {'kpi', b('SimKPI'), 0.1}, {});
% companion IO: output 3 is named mcu_status_dec because Simulink block names must be unique
% (input 3 mcu_status = CAN-FD frame, output 3 = decoded Bus_McuStatus; ICD 3.1 uses both names)
T(end+1) = row('companion_io', 'subsystems/electronics/companion_io.slx', 'ssr', true, 'T10', ...
    {'mcu_imu', F64, 0.005; 'mcu_odom', F64, 0.01; 'mcu_status', F64, 0.05; 'mcu_power', F64, 0.1; ...
     'gnc_cmd', b('GncCmd'), 0.05}, ...
    {'imu_meas', b('ImuMeas'), 0.005; 'odom_meas', b('OdomMeas'), 0.01; 'mcu_status_dec', b('McuStatus'), 0.05; ...
     'bms_status', b('BmsStatus'), 0.1; 'comp_cmd', F64, 0.05}, {});

% ---------------- controller models (ICD 3.1 / 3.3) ----------------
T(end+1) = row('Shaddad_MCU', 'models/Shaddad_MCU.slx', 'model', true, 'T08', ...
    {'in1k', b('McuIn1k'), 0.001; 'in5', b('McuIn5'), 0.005; 'in10', b('McuIn10'), 0.01; ...
     'in50', b('McuIn50'), 0.05; 'in100', b('McuIn100'), 0.1}, ...
    {'out1k', b('McuOut1k'), 0.001; 'out5', b('McuOut5'), 0.005; 'out10', b('McuOut10'), 0.01; ...
     'out50', b('McuOut50'), 0.05; 'out100', b('McuOut100'), 0.1; 'telem', b('FwTelemetry'), 0.01}, {});
T(end+1) = row('Shaddad_BMS', 'models/Shaddad_BMS.slx', 'model', true, 'T04', ...
    {'meas', b('BMS_Meas'), 0.01; 'can_rx', b('Can1_McuSlow'), 0.05}, ...
    {'cmd', b('BMS_Cmd'), 0.01; 'can_tx', b('Can1_Bms'), 0.1; 'telem', b('BMS_Telemetry'), 0.1}, {});
T(end+1) = row('Shaddad_NavFusion', 'models/Shaddad_NavFusion.slx', 'model', true, 'T12', ...
    {'imu', b('ImuMeas'), 0.005; 'odom', b('OdomMeas'), 0.01; 'gnss', b('GnssRaw'), 0.1; ...
     'scanmatch', b('ScanMatchMeas'), 0.2; 'navcmd', b('NavCmd'), 0.05; 'bms', b('BmsStatus'), 0.1}, ...
    {'nav_state', b('NavState'), 0.01; 'nav_diag', b('NavDiag'), 0.01}, {});
T(end+1) = row('Shaddad_Perception', 'models/Shaddad_Perception.slx', 'model', true, 'T13', ...
    {'scan', b('LidarScanRaw'), 0.1; 'cloud', b('LidarCloudRaw'), 0.1; 'nav_state', b('NavState'), 0.01}, ...
    {'perception', b('Perception'), 0.1; 'scanmatch', b('ScanMatchMeas'), 0.2}, {});
T(end+1) = row('Shaddad_GNC', 'models/Shaddad_GNC.slx', 'model', true, 'T14', ...
    {'nav_state', b('NavState'), 0.01; 'perception', b('Perception'), 0.1; 'mcu_status', b('McuStatus'), 0.05; ...
     'bms_status', b('BmsStatus'), 0.1; 'mcmd', b('MissionCmd'), 0.2; 'spray', b('SprayDemand'), 0.1}, ...
    {'gnc_cmd', b('GncCmd'), 0.05; 'nav_cmd', b('NavCmd'), 0.05; 'gnc_status', b('GncStatus'), 0.2; ...
     'gnc_debug', b('GncDebug'), 0.05}, {});

% ---------------- integration models built by their own scripts (not stubs) ----------------
T(end+1) = row('Shaddad_Plant', 'models/Shaddad_Plant.slx', 'model', false, 'T01', ...
    {'scen', b('ScenarioIn'), 0.01; 'mcu_out1k', b('McuOut1k'), 0.001; 'mcu_out10', b('McuOut10'), 0.01; ...
     'can_mcu_fast', b('Can1_McuFast'), 0.01; 'can_mcu_slow', b('Can1_McuSlow'), 0.05; 'bms_cmd', b('BMS_Cmd'), 0.01}, ...
    {'truth', b('TruthState'), 0.001; 'tracks', b('TrackState'), 0.001; 'drive_truth', b('DriveTruth'), 0.001; ...
     'power_truth', b('PowerTruth'), 0.1; 'bms_meas', b('BMS_Meas'), 0.01; 'can_esc_fast', b('Can1_EscFast'), 0.01; ...
     'can_esc_slow', b('Can1_EscSlow'), 0.05; 'can_impl', b('Can1_Impl'), 0.05; 'elec_hw', b('ElecHwIn'), 0.001; ...
     'elec_adc', b('ElecAdc'), 0.05; 'elec_env', b('ElecEnv'), 0.1; 'env', b('EnvState'), 0.1; ...
     'indicators', b('IndicatorOut'), 0.1; 'impl_mech', b('ImplMechState'), 0.01; 'impl_elec', b('ImplElecState'), 0.01; ...
     'struct_diag', b('StructureDiag'), 0.01; 'power_status', b('PowerStatus'), 0.001}, {});
T(end+1) = row('Shaddad_Companion', 'models/Shaddad_Companion.slx', 'model', false, 'T01', ...
    {'mcu_imu', F64, 0.005; 'mcu_odom', F64, 0.01; 'mcu_status', F64, 0.05; 'mcu_power', F64, 0.1; ...
     'gnss_raw', b('GnssRaw'), 0.1; 'lidar_scan', b('LidarScanRaw'), 0.1; 'lidar_cloud', b('LidarCloudRaw'), 0.1; ...
     'spray', b('SprayDemand'), 0.1; 'mcmd', b('MissionCmd'), 0.2}, ...
    {'comp_cmd', F64, 0.05; 'nav_state', b('NavState'), 0.01; 'perception', b('Perception'), 0.1; ...
     'gnc_status', b('GncStatus'), 0.2; 'gnc_debug', b('GncDebug'), 0.05; 'nav_diag', b('NavDiag'), 0.01}, {});
T(end+1) = row('Shaddad_System', 'models/Shaddad_System.slx', 'model', false, 'T01', ...
    {'ScenarioIn', b('ScenarioIn'), 0.01}, {}, {});
end

function r = row(name, file, kind, stub, owner, in, out, conn)
r.name = name; r.file = file; r.kind = kind; r.stub = stub; r.owner = owner;
r.in = ports(in); r.out = ports(out);
r.conn = struct('name', {}, 'domain', {}, 'type', {}, 'side', {});
for k = 1:size(conn, 1)
    if contains(conn{k, 2}, 'electrical'), d = 'electrical';
    elseif contains(conn{k, 2}, 'thermal'), d = 'thermal';
    else, d = 'rotational';
    end
    r.conn(k) = struct('name', conn{k, 1}, 'domain', d, 'type', conn{k, 2}, 'side', conn{k, 3});
end
end

function p = ports(c)
p = struct('name', {}, 'type', {}, 'ts', {});
for k = 1:size(c, 1)
    p(k) = struct('name', c{k, 1}, 'type', c{k, 2}, 'ts', c{k, 3});
end
end
