function [P, Ts, VC, P_meta] = shaddad_params(varargin)
%SHADDAD_PARAMS  Assemble the Shaddad UGV parameter struct P, sample-time struct Ts, the
%   variant-control scalars VC_* and the descriptive-string struct P_meta, and assign them in the
%   base workspace.
%
%   shaddad_params()                              defaults (flat_test world, no implement, SIM, phase 1)
%   shaddad_params('World',4,'Implement',1,'Phase',2,'Target',0)
%   [P,Ts,VC,P_meta] = shaddad_params(...,'Assign',false)   build without touching the base workspace
%   shaddad_params(...,'Motor','movematic750')   traction motor option of param_motor_unit (default 'kw1')
%
%   Revision B (docs/architecture.md D58): P is purely numeric/logical (tunable-safe, parameter
%   behaviour 'Tunable' in simulation); descriptive strings live in the separate base variable
%   P_meta, which is never referenced by a block or a MATLAB Function parameter.
%
%   Assembly order (dependencies): buses (for P.ic) -> sim/frames -> RC row test settings
%   (P.scen.rowtest) -> world (world 7 row_field is sized from them) -> power -> structure
%   (battery mass) -> drive (sprocket radius) -> electronics (B, r_s, N, mass) -> rc -> sensors ->
%   gnc (E_nom, implement widths) -> P.ic (bus initial-condition structs) -> checks -> RC row test
%   driver parameters (P.scen.rowdrv: needs P.sim, P.electronics, P.rc).
%   Base-workspace writes: P, Ts, P_meta, VC_* scalars (and Bus_* via shaddad_buses).
%   Variant controls are plain double scalars because variant expressions cannot index structs.

ip = inputParser;
ip.addParameter('World', 1);
ip.addParameter('Implement', 0);
ip.addParameter('Phase', 1);
ip.addParameter('Target', 0);
ip.addParameter('Assign', true);
ip.addParameter('Motor', []);                % traction motor option of param_motor_unit ([] = its default)
ip.parse(varargin{:});
o = ip.Results;

%% ---- buses (Bus_* objects in base; definitions used for P.ic) ----
if o.Assign
    shaddad_buses();
end
busDefs = shaddad_buses('defs');

%% ---- domain parameters ----
S = param_sim();
P = struct();
P.sim = S.sim; P.frames = S.frames; P.viz = S.viz; P.log = S.log; P.scen = S.scen;
P.scen.world_id = o.World;
P.scen.rowtest = param_rowtest();           % RC row test settings (numeric, D58); world 7 row_field is sized from them
if o.World == 7
    P.world = param_world(o.World, P.frames.g, P.scen.rowtest);
else
    P.world = param_world(o.World, P.frames.g);
end
P.power = param_power();
P.structure = param_structure(struct('implement', o.Implement, ...
    'batteryMass_kg', P.power.pack.mass_kg, 'batteryPos_m', P.power.pack.pos_FRD, 'motorOption', o.Motor), P.world, P.frames.g);
P.drive = param_drive(P.structure.r_sprocket, o.Motor);
P.electronics = param_electronics(P);       % needs P.structure.track.chi_nom (D39)
P.rc = param_rc();
P.sensors = param_sensors(P);
P.gnc = param_gnc(P);                       % needs P.electronics (command envelope, D43)

%% ---- bus initial conditions (zero structs, identical to Simulink.Bus.createMATLABStruct) ----
P.ic = bus_ic(busDefs);
P.gnc.mission = P.ic.Bus_MissionPlan;              % replaced by scripts/gnc_compileMission.m (Constant inside Shaddad_GNC)

%% ---- sample-time struct (every value must be in P.sim.Ts_allowed, except rc_frame) ----
Ts = struct('base',1e-3,'esc',1e-3,'ssc',P.sim.ssc.Ts,'safety',1e-3,'enc',1e-3, ...
    'imu',5e-3,'nav_imu',5e-3, ...
    'ctrl',1e-2,'can_drive',1e-2,'nav',1e-2,'odom',1e-2,'bms_fast',1e-2,'afe',1e-2,'impl_ctrl',1e-2,'scen',1e-2, ...
    'comm',5e-2,'can_fb2',5e-2,'can_slow',5e-2,'gnc',5e-2, ...
    'hk',0.1,'bms_slow',0.1,'gnss',0.1,'lidar',0.1,'perception',0.1,'gnc_avoid',0.1,'env',0.1, ...
    'monitor',0.1,'log',0.1,'encl_ssc',0.1,'spray',0.1, ...
    'scanmatch',0.2,'teb',0.2,'gnc_status',0.2,'mcmd',0.2, ...
    'plan',1.0,'bms_bal',1.0, ...
    'rc_frame',P.rc.frame_period);
fn = setdiff(fieldnames(Ts), {'rc_frame'});
for k = 1:numel(fn)
    if ~any(abs(Ts.(fn{k}) - P.sim.Ts_allowed) < 1e-12)
        error('shaddad_params:rate', 'Ts.%s = %g is not in the allowed rate set', fn{k}, Ts.(fn{k}));
    end
end
if ~any(abs(Ts.rc_frame - P.sim.Ts_rc_allowed) < 1e-12)
    error('shaddad_params:rate', 'P.rc.frame_period must be 0.007 or 0.014 s');
end

%% ---- variant controls ----
VC = struct('VC_TARGET', o.Target, 'VC_PHASE', o.Phase, 'VC_STRUCT_FID', 1, ...
    'VC_IMPL', o.Implement, 'VC_IMPL_DRIVE', 2, 'VC_PACK', P.power.variant.pack, ...
    'VC_CHG', P.power.variant.chg, 'VC_DRIVE_FID', P.drive.fidelity, 'VC_OPSRC', 1, ...
    'VC_LIDAR', P.sensors.variant.lidar, 'VC_ROW_TRACKER', P.gnc.variant.row_tracker, 'VC_VIZ', P.viz.enable, ...
    'VC_TEB', P.gnc.variant.teb);

%% ---- descriptive strings: separate base variable P_meta (never referenced by blocks), D58 ----
P_meta = local_meta();
mu_ = param_motor_unit(o.Motor);
P_meta.drive.motor = mu_.name; P_meta.drive.motor_option = mu_.option;   % 4 units, 2 per side

%% ---- consistency checks between domains ----
assert(abs(P.drive.gear.r_spr - P.structure.r_sprocket) < 1e-12, 'sprocket radius mismatch');
assert(abs(P.electronics.drv.N - P.drive.gear.N) < 1e-12, 'gear ratio mismatch');
assert(abs(P.gnc.fsm.E_nom_Wh - P.power.pack.E_Wh) < 1e-9, 'pack energy mismatch');
assert(isequal(P.sensors.refLLA, P.world.lla0), 'datum mismatch');
% revision B cross-domain checks
assert(any(VC.VC_DRIVE_FID == P.drive.fid_system), 'VC_DRIVE_FID = %g: FOC (2) is bench-only (tests/drive/tb_drive_foc)', VC.VC_DRIVE_FID);
assert(abs(P.electronics.mix.chi - P.structure.track.chi_nom) < 1e-12 && ...
       abs(P.sensors.ekf.chi0 - P.structure.track.chi_nom) < 1e-12 && ...
       abs(P.structure.lofi.chi - P.structure.track.chi_nom) < 1e-12, 'gauge factor chi not single-sourced');
assert(P.gnc.spd.v_max <= P.electronics.lim.v_max_auto + 1e-12 && ...
       P.gnc.cmd.v_track_max <= P.electronics.lim.v_track_max + 1e-12 && ...
       max(P.gnc.cmd.w_max) <= P.electronics.lim.w_max_auto + 1e-12, 'GNC envelope exceeds the MCU limits');
assert(P.drive.esc.a_max/P.drive.gear.k_v2w >= max([P.electronics.shaper.a_estop, P.electronics.shaper.a_dec_man, ...
       P.electronics.shaper.a_ctrlstop, P.electronics.shaper.a_dec_auto]) - 1e-9, 'ESC ramp would cap an MCU deceleration');
assert(P.drive.motor.T_peak*P.drive.gear.N*P.drive.gear.eta <= P.drive.gear.T_acc_out, 'motor peak exceeds gearbox T2B');
assert(P.rc.failsafe_raw(P.rc.ch_map.estop) <= P.rc.raw_mid, 'receiver failsafe preset must not command remote E-STOP');
assert(P.electronics.safety.t_persist(1+17) > 0, 'HV_LOST needs a persistence time');

%% ---- RC row test driver parameters (RowDriver, VC_OPSRC 4), for every world: the model compiles with any operator source ----
P.scen.rowdrv = row_test_prm(P, P.scen.rowtest);

%% ---- assign ----
if o.Assign
    assignin('base', 'P', P);
    assignin('base', 'Ts', Ts);
    assignin('base', 'P_meta', P_meta);
    vn = fieldnames(VC);
    for k = 1:numel(vn), assignin('base', vn{k}, VC.(vn{k})); end
end
end

function M = local_meta()
M.project = 'Shaddad_UGV';
M.release = 'R2026b';
M.world_names = {'flat_test','slope_test','palm_farm','greenhouse','solar_farm','soil_strips','row_field'};
M.implement_names = {'none','mower','tiller','sprayer'};
M.structure.componentNames = {'chassisFrame','bodyPanels','trackAssyL','trackAssyR','driveLF','driveLR','driveRF','driveRR', ...
    'batteryPack','electronicsBay','sensorMast','coolingMisc','frontCoupler','rearHitch'};
M.sim.solver_names = {'ode3'};
M.sim.ssc_solver = 'NE_BACKWARD_EULER_ADVANCER';
M.hw.mcu_board = 'STM32H7xx Based (Single-core)';
M.hw.mcu = 'STM32H743ZIT6 (NUCLEO-H743ZI2 dev)';
M.hw.bms_board = 'STM32G4xx Based';
M.hw.companion = 'NVIDIA Jetson Orin NX 16 GB, JetPack 6, ROS 2 Humble';
M.hw.cubemx = 'utilities/stm32/shaddad_h743.ioc';
M.files.top = 'Shaddad_System'; M.files.plant = 'Shaddad_Plant';
M.files.mcu = 'Shaddad_MCU'; M.files.bms = 'Shaddad_BMS'; M.files.gnc = 'Shaddad_GNC';
M.files.nav = 'Shaddad_NavFusion'; M.files.per = 'Shaddad_Perception';
M.mpc_object = 'mpcRowObj';
M.hw.cubemx_bms = 'utilities/stm32/shaddad_g474.ioc';
M.hw.cubemx_template = 'toolbox/stm32b/stm32target/stm32h7xx_based/boards/nucleo_h743zi2/nucleo_h743zi2.ioc';
M.codegen.required_products = {'Simulink Coder','Embedded Coder','STM32 Microcontroller Blockset'};
M.codegen.external_tools = {'STM32CubeMX','GNU Tools for STM32','MinGW-w64 C/C++ (MATLAB add-on)'};
M.companion.can_fd_topics = {'/can2/rx','/can2/tx'};
M.companion.can_fd_msg = 'std_msgs/UInt8MultiArray [id u32 LE | len u8 | data 64 B] (69 B)';
end
