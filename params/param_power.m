function Pw = param_power()
%PARAM_POWER  Battery pack, BMS, PDU, safety chain, LV rails, implement drives, charger (owner: power).
%   Pw = param_power() -> P.power. All SI (K, A discharge +, V, W, s). No global rng (RandStream seeds).
%   Variant controls derived by shaddad_params: VC_PACK (1 Grouped | 2 Lumped),
%   VC_CHG (1 BMS-controlled source | 2 batt_lib Charger), VC_IMPL (shared with structure).
%   Revision B: hot-charge derating re-based on the cell datasheet charge window plus
%   dock-conditioned pack air while docked (D45); duty-based shift budget (D46); variable-capacity
%   SOC/SOE estimators fed by the capacity estimator (D48); safety-relay reset stretch (D30).
%   Real robot (2026-10-27, docs/hardware/real_robot_components.md): XMachines 13S15P Molicel P42A NMC
%   pack, 46.8 V / 63 Ah / ~3 kWh, 120-150 A BMS, 54.6 V 20 A charger [DATA]; cell model tables,
%   thermal and fuse values [EST].

%% ---- sample times / variants ----
Pw.Ts = struct('plant',1e-3,'local',1e-3,'local_mission',1e-3,'afe',0.01, ...
               'bms_fast',0.01,'bms_slow',0.1,'bal',1.0,'telem',0.1,'impl_ctrl',0.01,'can_rx',0.05);
Pw.variant = struct('pack',1,'chg',1);

%% ---- cell (Molicel P42A, 21700 Li-ion NMC, 4.2 Ah [DATA]; tables [EST] from typical P42A data) ----
c.Q_Ah = 4.2; c.V_nom = 3.6; c.V_cv = 4.20; c.V_max_abs = 4.25; c.V_min_abs = 2.50;
c.mass = 0.070; c.cp = 1000; c.Cth = c.mass*c.cp;
c.D = 0.021; c.H = 0.070;                    % cylindrical 21700: diameter, height
c.SOC_bp = [0 .05 .10 .20 .35 .50 .65 .80 .90 .95 .98 1];
c.T_bp   = [273.15 298.15 318.15 333.15];
% rest OCV of an NMC 21700 cell with the steep end-of-discharge knee; OCV(0) = 2.60 V so that the 1-RC
% cell at 1C (R0 + R1 = 30 mOhm at SOC 0, 0.13 V drop) reaches the 2.5 V cell cut-off just before SOC 0
% (the Battery Equivalent Circuit asserts below SOC 0; T-PWR-00) and the 2.8 V BMS cut-off at about 2 %
c.OCV25  = [2.60 3.30 3.45 3.55 3.62 3.70 3.82 3.96 4.06 4.11 4.15 4.20];
c.kT     = [0.3 0.3 0.2 0.1 0.05 0 0 0 0 0 0 0]*1e-3;
c.R0_25  = [22 18 16 15 14 14 14 14 14 15 15 16]*1e-3;
c.fR0    = [2.5 1.0 0.8 0.75];
c.R1_25  = [8 6 5 4.5 4 4 4 4 4 4.5 5 5]*1e-3;
c.fR1    = [2.5 1.0 0.70 0.60];
c.tau1_25 = 30;  c.ftau = [1.3 1.0 0.9 0.85];
c.VoltageRange = [2.0 4.4];
c.I_dis_cont = 45; c.I_dis_peak10s = 60; c.I_chg_max = 4.2;   % P42A: 45 A continuous, 4.2 A standard charge
% 2-D tables (SOC x T), cell level
dT = c.T_bp - 298.15;
c.OCV_tab  = c.OCV25(:) + c.kT(:)*dT;
c.R0_tab   = c.R0_25(:) * c.fR0;
c.R1_tab   = c.R1_25(:) * c.fR1;
c.tau1_tab = c.tau1_25 * ones(numel(c.SOC_bp),1) * c.ftau;
Pw.cell = c;

%% ---- pack 13S15P [DATA]: 195 cells, 46.8 V / 54.6 V, 63 Ah, ~2.95 kWh ----
pk.Ns = 13; pk.Np = 15; pk.Q_Ah = c.Q_Ah*pk.Np; pk.V_nom = c.V_nom*pk.Ns; pk.E_Wh = pk.V_nom*pk.Q_Ah;
pk.V_oper_min = 3.0*pk.Ns; pk.V_oper_max = c.V_cv*pk.Ns; pk.V_abs_max = c.V_max_abs*pk.Ns;   % 39.0 / 54.6 / 55.25 V
pk.mass_kg = 18.5;                           % [EST] cells 13.65 + 3 mm aluminium box 3.3 + BMS / busbars 1.5 kg
pk.MassFactor = pk.mass_kg/(c.mass*pk.Ns*pk.Np);
pk.dims = [0.393 0.3585 0.087];  pk.pos_FRD = [0.05 0 -0.30];     % box XMT-P1-E01 [DATA], lying flat; position [EST]
pk.R_noncell = 0.10e-3; pk.R_amb_cell = 20; pk.R_bal = 60; pk.R_bal_switch = 0.1;   % [EST] 70 mA bleed at 4.2 V
pk.SOC0 = 0.95; pk.SOC0_spread = 0.01;
pk.var = struct('cap_sigma',0.01,'R0_sigma',0.03,'seed',42);
rs = RandStream('mt19937ar', 'Seed', pk.var.seed);
pk.var.cap_dev_pct = 100*pk.var.cap_sigma*randn(rs, 1, pk.Ns);
pk.var.R0_dev_pct  = 100*pk.var.R0_sigma *randn(rs, 1, pk.Ns);
pk.var.SOC0_vec    = pk.SOC0 + pk.SOC0_spread*(2*rand(rs, 1, pk.Ns) - 1);
% T04: truth ageing factors (EOL tests T-PWR-13/13b: cap 0.8, R0 1.5) and builder scaling.
% Battery Builder models one cell per parallel assembly behind a x Np lumping adapter: the balancing
% branch sits at that cell node, so the cell-level balancing resistance is R_bal*Np for the
% group-level (channel) bleed; the ambient resistance is per cell (scaled by the builder).
pk.age = struct('cap_factor',1,'R0_factor',1);
pk.R_noncell_total = pk.R_noncell*(pk.Ns-1);  % Ns-1 inter-group links (module NonCellResistance)
pk.bal_R_cell      = pk.R_bal*pk.Np;          % CellBalancingResistance of the generated module
pk.bal_Rsw_cell    = pk.R_bal_switch*pk.Np;   % CellBalancingClosedResistance
pk.bal_G_open      = 1e-8;                    % CellBalancingOpenConductance (1/Ohm)
pk.bal_threshold   = 0.5;                     % controlled-switch threshold on Bal_Cmd (0/1)
Pw.pack = pk;
% group-level tables for the BMS (Np-parallel lumped)
Pw.group = struct('AH',pk.Q_Ah,'SOC_vec',c.SOC_bp,'T_vec',c.T_bp,'V0_mat',c.OCV_tab, ...
                  'R0_mat',c.R0_tab/pk.Np,'R1_mat',c.R1_tab/pk.Np,'tau1_mat',c.tau1_tab);

%% ---- PDU ----
F = @(Ir,I,t,R) struct('I_rated',Ir,'I_vec',I,'t_vec',t,'R_closed',R,'G_open',1e-8);
tA = [3600 300 30 3 0.3 0.02]; tB = [3600 200 20 2 0.2 0.01]; tC = [3600 60 5 0.5 0.05 0.005];
% fuse ratings [EST] for the 150 A BMS / SB120 main connector and the dual-channel ESC of each side
% (2 x 55 A DC motoring with the 1.0 kW units, 2026-10-27; was 60 A for one 40 A ESC)
kI = [1.1 1.5 2 3 5 10];                     % time-current curve points as multiples of the rating
pd.F_MAIN  = F(150,150*kI, tA, 0.3e-3);
pd.F_TL    = F(125, 125*kI, tB, 0.5e-3);
pd.F_TR    = pd.F_TL;
pd.F_IMP   = F(40, 40*kI,  tB, 1.2e-3);
pd.F_AUX   = F(30, [33 45 60 90 150 300],       tC, 2.5e-3);
pd.F_CHG   = F(30, 30*kI,  tB, 1.5e-3);
pd.F_24IMP = F(20, [22 30 40 60 100 200],       tC, 4e-3);
K = @(R,ton,toff) struct('R_closed',R,'G_open',1e-8,'t_on',ton,'t_off',toff);
pd.K1 = K(0.2e-3,0.015,0.008); pd.K2 = K(50e-3,0.010,0.005); pd.K4 = K(0.3e-3,0.015,0.008);
pd.K3 = K(5e-3,0.008,0.004);
pd.R_pre = 22; pd.R_shunt = 100e-6; pd.C_pdu = 100e-6; pd.ESR_pdu = 5e-3; pd.g_bleed = 2e-4;
pd.R_cab_pack = 0.34e-3; pd.R_cab_trac = 0.98e-3; pd.R_key = 10e-3; pd.P_bms_supply = 2;
pd.C_bus_total_assumed = 5.5e-3;             % 2 x 2.2 mF ESC + 1.0 mF implement drive + 0.1 mF PDU
pd.mass_kg = 8;
Pw.pdu = pd;

%% ---- thermal (pack enclosure) ----
% enclosure: 393 x 358.5 x 87 mm, 3 mm aluminium (3.3 kg) [DATA]; the real box is sealed (no fan is
% mentioned): the fan model is kept as an option [EST]
Pw.thermal = struct('T_init_K',308.15,'T_amb_design',318.15,'T_amb_max',323.15, ...
    'm_int',2,'cp_int',900,'m_wall',3.3,'cp_wall',900,'A_int',0.41,'h_in',4,'A_ext',0.41,'h_ext',10, ...
    'eps',0.85,'rad_tr_coeff',0.85*5.670374e-8,'alpha_sol',0.6,'A_sol',0.14,'h_fan',20,'k_filter',1.0, ...
    'P_fan',24,'TempOn',313.15,'TempOff',308.15,'T_force',323.15, ...
    'dock_air',struct('enable',true,'T_air',303.15,'fan_force',true));   % D45: conditioned air from the dock while docked

%% ---- LV distribution ----
dc = @(v,P,eff,Pf,tau,v1) struct('v_ref',v,'P_rated',P,'eff_rated',eff,'P_fixed',Pf,'tau',tau, ...
                                 'v1_min',v1,'R1',1e4,'R2',1e4);
lv.dcdc24 = dc(24,600,94,4,0.005,36);
lv.dcdc12 = dc(12,300,93,3,0.005,36);
lv.dcdc5  = dc(5,25,90,0.3,0.002,8);
lv.ldo33  = struct('V_out',3.3,'V_dropout',0.25,'Iq',5e-3);
lv.load   = struct('min_supply_frac',0.7,'deltaV',0.5,'pow_min',1e-3,'G_reverse',1e-6,'tau_filt',2e-3);
lv.coil_hold_W = struct('K1',2.0,'K2',1.5,'K3',0.5,'K4',2.0);    % on the 12 V rail (POWER owns coil loads)
lv.coil_hold_vec = [lv.coil_hold_W.K1 lv.coil_hold_W.K2 lv.coil_hold_W.K3 lv.coil_hold_W.K4];   % W, order [K1 K2 K3 K4]
lv.rail_ok_tol = 0.10;
lv.V_nom = [24 12 5 3.3];                    % T05: rail nominal voltages [24V 12V 5V 3V3] for the +/-10 % OK flags
Pw.lv = lv;

%% ---- implement electrical drives (load mechanics are in STRUCTURE via Impl_Shaft) ----
rpm2rad = pi/30;
% T05: the Motor & Drive (System Level) block requires DC-voltage breakpoints > 0, so the spec 6.6 grid
% [0 30 40 60] V starts at 1 V; the envelope is still zero torque for V_dc <= 30 V (spec 2.7, T-PWR-18).
% Pbase (fixed drive losses, spec 5 / 6 W) and Piron (iron losses, spec 20 / 25 W) = 0: a constant or speed-dependent
% power draw at zero torque has no solution on a dead DC bus (K1 open, coasting shaft, V_dc -> 0, I = P/V; verified:
% initial-condition failure with Pbase, 'DC voltage supply must be > 0' assertion with Piron on a decaying bus), so
% drive standby / iron losses are not modelled; eff is raised from the spec 88 % so that the copper loss at the
% measurement point (w_eff, T_eff) equals the spec value without Piron / Pbase (mower 167.8 W, tiller 226.0 W).
% T05: the 5-point spec table interpolates torque linearly between 2000 and 3000 rpm, which allows up to 2.15 kW
% (I_imp 50 A in T-PWR-14) against the stated "1.8 kW power limit above 1700 rpm"; the envelope below follows the
% 1.8 kW constant-power curve (T = 1800 W / w) at the intermediate speeds, same end points as the spec table.
mw.w_t_rpm = [0 1700 2000 2500 3000 3600 4000];  mw.vdc_t = [1 30 40 60];
mw.T_t_2D  = [0 0 10 10; 0 0 10 10; 0 0 8.59 8.59; 0 0 6.88 6.88; 0 0 5.7 5.7; 0 0 4.8 4.8; 0 0 0 0];
mw.eff = 89.4; mw.w_eff_rpm = 3000; mw.T_eff = 4.5; mw.Piron = 0; mw.Pbase = 0; mw.Tc = 0.02;
mw.J_motor = 4e-4; mw.lam = 1e-4;
mw.ratio = 1;                                % direct drive to the blade shaft
mw.w_nom = 293.2;                            % rad/s blade (2800 rpm) - from P.structure.impl.mower.w_nom
mw.Kp = 0.6; mw.Ki = 1.5; mw.T_brake = 10; mw.ramp_up = 150; mw.ramp_dn = -400;
mw.T_max = 10;                               % N m, PI upper limit (= envelope maximum)
mw.C_link = 1.0e-3;  mw.C_link_esr = 10e-3; % F / Ohm, drive DC-link capacitor on IMP48
Pw.impl.mower = mw;
tl.w_t_rpm = [0 2000 3000 3500 4000];  tl.vdc_t = [1 30 40 60];
tl.T_t_2D  = [0 0 12 12; 0 0 12 12; 0 0 9.5 9.5; 0 0 8.2 8.2; 0 0 0 0];
tl.eff = 89.3; tl.w_eff_rpm = 3000; tl.T_eff = 6; tl.Piron = 0; tl.Pbase = 0; tl.Tc = 0.02;
tl.ratio = 12; tl.J_motor = 6e-4; tl.lam = 1e-4;
tl.w_nom_rotor = 26.18;                      % rad/s (250 rpm)
tl.w_nom = tl.w_nom_rotor;                   % rad/s at Impl_Shaft (rotor side of the 12:1 gearbox)
tl.Kp = 0.1; tl.Ki = 0.5; tl.T_brake = 12; tl.ramp_up = 15; tl.ramp_dn = -40;
tl.T_max = 12;                               % N m (motor), PI upper limit (= envelope maximum)
tl.C_link = 1.0e-3;  tl.C_link_esr = 10e-3;
Pw.impl.tiller = tl;
sp.Ra = 0.20; sp.La = 0.25e-3; sp.Kv = 6.5e-3; sp.J = 7e-5; sp.lam = 1e-5;   % 24 V PM DC pump motor
sp.buck_eff = 97;
sp.w_nom = 314.16;                           % rad/s pump (3000 rpm)
sp.k_p = 4.9e-6; sp.k_q = 0.0255;            % N m s^2 pump torque k_p w^2 / L/min per rad/s (bench stub, spec 6.6)
sp.duty_slew = 1.5;                          % 1/s, pump driver soft start of the PWM duty (inrush limit, T-PWR-16)
sp.run_duty = 0.05;                          % duty above which the pump counts as commanded on
sp.p_ready_frac = 0.8;                       % IMPL_FB ready: line pressure > 0.8 * P.structure.impl.sprayer.p_set
Pw.impl.sprayer = sp;
Pw.impl.nozzle_P_W = 8;  Pw.impl.nNoz = 8;
Pw.impl.lift_P_W = 150;
Pw.impl.load_min_V = 16.8;                   % V, min supply voltage of the nozzle / lift Dynamic Loads (spec I15)
Pw.impl.D_none = 1e-4;                       % N m s/rad, Impl_None shaft damper (ICD 2)
% T05: DC-link over-voltage clamp of the 48 V implement drives (brake chopper / TVS as Zener diode): conducts only
% above BV (> V_abs_max 58.4 V, so never with K1 closed); limits an isolated link pumped by regenerative braking
Pw.impl.dc_clamp = struct('BV', 60, 'Rz', 0.05);    % V, Ohm
% implement ECU (COTS drive controller, impl_ecu.slx; Fault_Code bits b0 OC b1 OV b2 UV b3 OT b4 CAN_TIMEOUT
% b5 STALL b6 SAFETY_OFF). The motor temperature is not modelled (Motor & Drive without thermal port):
% Impl_Temp_K reports T_motor_K.
Pw.impl.ecu = struct('t_can_timeout',0.2,'ready_frac',0.9,'node_id',0,'run_frac',0.1, ...
    'V_ov',58.4,'V_uv',30,'I_oc',120,'t_stall',2,'stall_frac',0.1,'stall_T_frac',0.95, ...
    'T_motor_K',313.15,'T_ot_K',413.15,'age_max',10);
Pw.impl.rpm2rad = rpm2rad;

%% ---- dock charger and safety chain ----
% charger 54.6 V / 20 A, PFC, IP54 [DATA]; OVP 4.3 V/cell, efficiency [EST]
Pw.charger = struct('I_max',20,'P_max',1200,'V_ovp',4.30*pk.Ns,'eta',0.92,'slew',20,'tau',0.5, ...
    'R_dock',15e-3,'diode_Vf',0.7,'diode_Ron',2e-3,'diode_Goff',1e-8,'R_bleed',1e4, ...
    'cccv_I',20,'cccv_Vth',c.V_cv*pk.Ns,'cccv_V',c.V_cv*pk.Ns,'cccv_Ith',2);
Pw.safety = struct('Q0',1,'t_relay_on',0.020,'t_relay_off',0.015,'t_cat1',1.5, ...
                   't_reset_hold',0.05);     % D30: Reset_edge stretched 50 ms (On-Off Delay Td_on 0, Td_off 0.05)

%% ---- BMS ----
% 13S NMC BMS [DATA]: 120-150 A continuous, 200-300 A peak, CAN 500 kbit/s; cell thresholds [EST] (P42A 2.5-4.2 V)
b.V_ov = 4.25; b.V_ov_warn = 4.22; b.V_uv = 2.80; b.V_uv_warn = 3.00; b.V_cv = c.V_cv; b.V_bal_enable = 4.00;
b.T_ot = 333.15; b.T_ut = 253.15; b.T_warn = 323.15;
b.qual = struct('OV',1,'UV',2,'OC',1,'OT',5,'sensor',2);
b.OC_ErrorLimit = 20;  b.FaultySensorThreshold = 1.0;
b.ekf = struct('Q',[1e-7 0; 0 1e-6],'R',1e-4,'P0',[1e-2 0; 0 1e-4],'SOC_init_bias',0);
b.dis.T_vec  = [253.15 263.15 273.15 283.15 318.15 323.15 328.15 333.15];
b.dis.I_cool = [40 75 110 150 150 150 150 150];
b.dis.I_hot  = [150 150 150 150 150 125 75 0];
b.chg.T_vec  = [273.15 278.15 283.15 288.15 313.15 318.15 323.15 328.15];
b.chg.I_cool = [0 10 20 30 30 30 30 30];
b.chg.I_hot  = [30 30 30 30 30 15 0 0];      % D45: NMC charge window 0..45 C (P42A)
b.R0_limit = 2.0e-3;                         % group resistance limit (BOL group R0 ~ 1 mOhm)
b.I_dis_cont = 150; b.I_dis_peak = 250; b.I_chg_max = 30;   % [DATA] 150 / 200-300 A; charge 30 A [EST]
% SOC taper on discharge (spec 2.9.6: x0.5 below 10 %, x0.2 below 5 %), 1-D Lookup 'Flat' (input below)
b.soc_taper = struct('bp',[0 0.05 0.10],'k',[0.2 0.5 1]);
% CC-CV PI (T04 design rule Kp*R ~ 0.9, Kp/Ki = 5 s against the 0.5 s charger lag), re-scaled to the
% 13S15P group resistance R0 + R1 ~ 1.2 mOhm [EST: re-check the CV overshoot on tests/power]
b.cccv = struct('Kp',750,'Ki',150,'Kaw',1,'Kt',1);
b.charge_done = struct('V',c.V_cv - 0.005,'I',2,'t',60);   % C/30 cut-off
b.bal = struct('Threshold',0.010,'Hyst',0.003,'Td_on',5,'Td_off',5);
% Simscape Battery BMS library sign convention (verified T04): current + = CHARGE inside batt_sl_lib
% blocks (estimators, limits, monitoring, power estimator); Shaddad buses use + = discharge, so the
% BMS feeds -I_pack to the library blocks. imin/imax and Pmin/Pmax are therefore given as
% [-discharge, +charge] limits.
b.pwr_est = struct('vmin',b.V_uv,'vmax',c.V_cv,'SOCmin',0.05,'SOCmax',1,'imin',-b.I_dis_peak,'imax',b.I_chg_max, ...
                   'Pmin',-b.I_dis_peak*pk.V_nom,'Pmax',b.I_chg_max*c.V_cv*pk.Ns,'TimeHorizon',10,'ChargingEfficiency',0.99);
b.pwr_est.R_10s = Pw.group.R0_mat + Pw.group.R1_mat.*(1 - exp(-b.pwr_est.TimeHorizon./Pw.group.tau1_mat));
b.cap_est = struct('SocChangeThreshold',0.05,'gamma',0.99,'Ts',1);
b.soh = struct('AH_new',pk.Q_Ah,'AH_eol',0.8*pk.Q_Ah);   % library SOH = (AH - AH_eol)/(AH_new - AH_eol): 1 new, 0 EOL
% D48: SOC/SOE estimators with variable capacity, fed by the capacity estimator (rate-limited, bounded)
b.est = struct('variable_capacity',true,'AH_rate_lim',1e-3,'AH_bounds',[2/3 1.033]*pk.Q_Ah, ...
               'E_nom_group_Wh',pk.Q_Ah*c.V_nom,'eol_test_cap',0.8,'eol_test_R0',1.5,'soc_err_max',0.03);
% initial estimator states = BMS non-volatile memory (SIM: truth SOC0 + SOC_init_bias, learned AH)
b.est.SOC_init = min(max(pk.var.SOC0_vec + b.ekf.SOC_init_bias, 0), 1);   % 1 x Ns (EKF InitialSOC)
b.est.AH_init  = pk.Q_Ah;                    % learned group capacity in NVM (Ah)
b.est.V_nom_pack = c.V_nom*pk.Ns;            % 46.8 V: EnergyCapacity = CellAH x V_nom x Ns (D48)
% R0 ageing adaptation in front of the EKF (Estimation/R0a_*): ratio = measured string resistance at
% current steps / BOL group-table sum, filtered with alpha per detected step; ratio_init = NVM value
b.r0a = struct('dI_min',10,'alpha',0.3,'ratio_init',1,'ratio_bounds',[0.7 2.5]);   % A, -, -, -
b.chart = struct('t_pre_min',0.25,'t_pre_max',1.5,'k_pre',0.97,'t_overlap',0.05,'t_open_wait',0.2, ...
                 'I_open_max',10,'n_pre_retry',3,'t_retry',5,'t_weld',5,'k_weld',0.95,'t_init',0.1, ...
                 't_chg_en',0.1,'dt',0.01);  % dt = chart tick (after() counters at Ts.bms_fast)
b.mcu_cmd_timeout = 0.5;
b.P_avg_window = 600;                        % samples at Ts.bms_bal = 1 s (600 s = one shift duty period; a 60 s
                                             % average swings with the mowing pulses / cut phases, T-PWR-11 / 13b)
b.runtime_P_min = 200;                       % W
b.soc_reserve_runtime = 0.10;
% library Fault Qualification: DisqualTime = 0 never clears (no reset input), so the protection blocks
% use a short disqualification and the latch (reset on Fault_Reset_Req rising edge) is in the fault word
b.disqual = 0.1;                             % s
b.qual_aux = 0.1;                            % s, fuse / charger OVP qualification (E5)
b.contact_enable = false;                    % E6 cell-contact monitoring present but disabled (spec 3.9)
b.V_dock_ovp = Pw.charger.V_ovp;             % 58.4 V charger over-voltage
b.I_rest = 2;                                % A, |I| below = REST (balancing allowed near top of charge)
b.V_cvl = b.V_cv*pk.Ns;                      % CVL 54.6 V (charger voltage request)
b.V_dvl = b.V_uv*pk.Ns;                      % DVL 36.4 V
b.fan = struct('dT_amb',1,'T_force',Pw.thermal.T_force);   % fan only if T_cell_max > T_amb - 1 K; forced above 50 C
b.warn = struct('SOC_low',0.15,'SOC_crit',0.08,'dV_imb',0.030,'V_imb_top',4.10, ...
                'I_dis_derate',b.I_dis_cont,'I_chg_derate',b.I_chg_max);
% HW variant placeholders (VC_TARGET == 1; values confirmed by T17 with the AFE / board design)
ntc_K = 233.15:5:373.15;                                                % NTC 10 k / B 3950, 10 k pull-up, 12 bit
ntc_R = 1e4*exp(3950*(1./ntc_K - 1/298.15));
b.hw = struct('afe_rx_len',4*pk.Ns,'afe_lsb_V',1e-4, ...   % 2 bytes x (Ns voltages + Ns temperatures)
    'ntc_counts',fliplr(4095*ntc_R./(ntc_R + 1e4)),'ntc_K',fliplr(ntc_K), ...
    'adc_gain',[500/65535; 21*3.3/65535; 21*3.3/65535; 21*3.3/65535; 1; 1], ...   % I, V_pack, V_bus, V_dock, T_amb, T_air
    'adc_offset',[-250; 0; 0; 0; 0; 0]);
% fault word groups (Bus_BMS_Telemetry.Fault_Word bit numbers)
b.fault = struct( ...
    'mask_latched', uint32(sum(2.^[0 1 2 3 4 5 8 9 10 11 12 13 14 15])), ...   % sticky until reset
    'mask_crit_in', uint32(sum(2.^[0 1 2 3 4 5 8 14 15])), ...                 % FaultCritical -> ModeManager
    'mask_level3',  uint32(sum(2.^[0 1 2 3 4 5 6 7 8 14 15])), ...             % Fault_Level 3 (HV off)
    'mask_level2',  uint32(sum(2.^[9 10 11 12 13 17 18 19 20])));              % Fault_Level 2 (derate)
Pw.bms = b;

%% ---- shift energy budget (D46; checked in T-PWR-20 and S3) ----
% [EST] sprayer robot at 1.0 m/s: working (traction 320 W, pump 75 W, nozzles 64 W, LV 60 W) 520 W, transit 380 W;
% a 3 h shift (2 h 06 working) takes 1.43 kWh. The end-of-shift SOC follows from the pack at EOL (soh_eol).
Pw.budget = struct('t_shift',3*3600,'duty_cut',0.70,'v_work',1.0,'P_cut_W',520,'P_noncut_W',380, ...
    'soc_window',[0.95 NaN],'soh_eol',0.8,'eta_dis',0.97,'shifts_per_day',3,'t_transit',0.5*3600);
bu = Pw.budget;
E_shift = bu.t_shift/3600*(bu.duty_cut*bu.P_cut_W + (1 - bu.duty_cut)*bu.P_noncut_W);
Pw.budget.soc_window(2) = bu.soc_window(1) - E_shift/(pk.E_Wh*bu.eta_dis*bu.soh_eol);   % 0.32

%% ---- T04 bench (tests/power/tb_pack_bms) local solver ----
% Backward Euler 1 ms with fixed cost; one Newton / one mode iteration per step (results identical to
% P.sim.ssc 3/2 on the pack bench within 1e-5 V, about 4x cheaper Simscape step; T04 measurement).
% Ts_mission: plant rate + local solver step of the bench in mission mode (spec R9, T-PWR-19 "Lumped at
% 5 ms"), used by tests/power/pwr_case_run for the multi-hour cases (C.mission)
Pw.tb = struct('MaxNonlinIter',1,'MaxModeIter',1,'Ts_mission',5e-3);

%% ---- BMS sensor models ----
rs2 = RandStream('mt19937ar', 'Seed', 1001);
Pw.sensors = struct('seed',1000, ...
    'Vcell', struct('sigma',1.0e-3,'offset',2e-3*(2*rand(rs2,1,pk.Ns)-1),'q',0.1e-3), ...
    'Ipack', struct('sigma',0.05,'offset',0.10,'gain_err',0.003,'q',0.01,'tau',1e-3), ...
    'Vpack', struct('sigma',20e-3,'gain_err',0.002,'q',1e-3), ...
    'Tcell', struct('sigma',0.3,'offset',0.5*(2*rand(rs2,1,pk.Ns)-1),'q',0.1,'tau',5));
end
