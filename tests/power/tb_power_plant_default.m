function TB = tb_power_plant_default(kind)
%TB_POWER_PLANT_DEFAULT  Default stimulus struct of the T05 benches (owner: power, T05).
%
%   TB = tb_power_plant_default()          tests/power/tb_power_plant.slx (model workspace TB)
%   TB = tb_power_plant_default('shift')   tests/power/tb_shift_cycle.slx: 24 h shift-cycle schedule (T-PWR-20)
%
%   Tables are [t value] breakpoints (strictly increasing t, s), held between breakpoints (Flat, clip).
%   Tests pass a modified copy with Simulink.SimulationInput.setVariable('TB', TB, 'Workspace', <bench>).
%
%   MCU hardware outputs (Bus_PowerHwCmd, Ts.base):
%     do0_*  MCU_Safety_OK (DO0)   do1_*  Safety_Reset_Req (DO1)   k3_*  ImpPwr24_Enable (DO2)
%     duty_* Spray_Pump_Duty       noz_*  number of open nozzles (0..8)   lift_* Lift_Duty
%     Default = ICD 6 power-up: DO0 rises at 10 ms, automatic 50 ms DO1 pulse 20 ms later.
%   Operator (Bus_HmiInputs, Ts.base): estopF_* estopR_* wless_* bumpF_* bumpR_* reset_* keyoff_*
%   Fault injection (Bus_FaultInject, Ts.base): ch2_* estop_ch2_stuck, bmsf_* bms_fault
%   MCU CAN (0.05 s): BMS_REQ hv_* chg_* rst_* sd_*, alive_freeze_t; IMPL_CMD ien_* (enable),
%     ispd_* (rpm at blade / rotor), icmd_stop_t (no IMPL_CMD frame from this time: valid false, alive frozen)
%   LV demand (Bus_LV_LoadDemand, 0.01 s): P24_* P12_* P5_* P3_* (W); spec 8 ELECTRONICS stub 53 / 99 / 2.5 / 0.6 W
%   Environment (Bus_EnvState, 0.1 s): T_amb (K), G_solar (W/m^2), dock_* dock contact
%   DRIVE stubs (per side, DC48_Trac*): C_drv (F) with ESR R_esr, Controlled Current Source loadL_* / loadR_*
%     (A, + = consumption; load_is_power = 1: W, I = P / max(V(k-1), 30 V)), gated by the 1 ms-delayed port
%     voltage V_gate = [off on] V (D36); short_* switch of R_short (Ohm) across DC48_TracL (T-PWR-03b / 06)
%   Implement load stub on Impl_Shaft: inertia J_impl, load torque k_aero w|w| + k_lin w + LF(t) (T_c sat(w/10)
%     + k_lf w) with lf_* table; w through a 1 ms Unit Delay (D36); p_line = 1: impl_mech.linePressure =
%     p_set (w / w_nom)^2 (sprayer stub)
%   Solver: MaxNonlinIter / MaxModeIter of the bench Solver Configuration (local BE, P.sim.ssc.Ts)
%   Logs: dec_fast (1 ms To Workspace decimation), dec_slow (10 ms), dec_bms / dec_truth (0.1 s BMS telemetry /
%   plant-truth summaries log_bms / log_truth); columns in tests/power/pp_log_cols.m
if nargin < 1, kind = 'bench'; end
H = 1e7;
TB.do0_t = [0 0.010 H];     TB.do0_v = [0 1 1];
TB.do1_t = [0 0.030 0.080 H]; TB.do1_v = [0 1 0 0];
TB.k3_t = [0 H];   TB.k3_v = [0 0];
TB.duty_t = [0 H]; TB.duty_v = [0 0];
TB.noz_t = [0 H];  TB.noz_v = [0 0];
TB.lift_t = [0 H]; TB.lift_v = [0 0];
for n = {'estopF', 'estopR', 'wless', 'bumpF', 'bumpR', 'reset', 'keyoff', 'ch2', 'bmsf'}
    TB.([n{1} '_t']) = [0 H]; TB.([n{1} '_v']) = [0 0];
end
TB.hv_t = [0 1.0 H]; TB.hv_v = [0 1 1];
TB.chg_t = [0 H];    TB.chg_v = [0 0];
TB.rst_t = [0 H];    TB.rst_v = [0 0];
TB.sd_t = [0 H];     TB.sd_v = [0 0];
TB.alive_freeze_t = 1e9;
TB.ien_t = [0 H];  TB.ien_v = [0 0];
TB.ispd_t = [0 H]; TB.ispd_v = [0 0];
TB.icmd_stop_t = 1e9;
TB.P24_t = [0 H]; TB.P24_v = [53 53];
TB.P12_t = [0 H]; TB.P12_v = [99 99];
TB.P5_t = [0 H];  TB.P5_v = [2.5 2.5];
TB.P3_t = [0 H];  TB.P3_v = [0.6 0.6];
TB.T_amb = 298.15; TB.G_solar = 0;
TB.dock_t = [0 H]; TB.dock_v = [0 0];
TB.C_drv = 2.2e-3; TB.R_esr = 10e-3;
TB.loadL_t = [0 H]; TB.loadL_v = [0 0];
TB.loadR_t = [0 H]; TB.loadR_v = [0 0];
TB.load_is_power = 0;
TB.V_gate = [30 35];
TB.short_t = [0 H]; TB.short_v = [0 0]; TB.R_short = 5e-3;
TB.J_impl = 0.05; TB.k_aero = 1.19e-5; TB.k_lin = 0; TB.T_c = 0; TB.k_lf = 0;
TB.lf_t = [0 H]; TB.lf_v = [1 1];
TB.p_line = 0;
TB.MaxNonlinIter = 3; TB.MaxModeIter = 2;
TB.dec_fast = 1; TB.dec_slow = 1; TB.dec_bms = 1; TB.dec_truth = 1;
if strcmpi(kind, 'shift')
    TB = local_shift(TB);
end
end

function TB = local_shift(TB)
% 24 h shift cycle (T-PWR-20, D45/D46): three 8 h slots, each = transit out, 4 h shift (P.power.budget duty:
% 7 min cutting at 2352 W / 3 min non-cutting at 979 W total pack power), transit back (979 W), docked
% charging for the rest of the slot (MCU: HV off, Charge_Enable_Req on while docked). The DRIVE stubs draw
% the budget pack power minus the LV share (P_lv_budget = 194 W, D46) as DC power, split L / R.
H = 1e7;
Pw = param_power(); bu = Pw.budget;          % real robot budget (P.power.budget); LV share about 60 W [EST]
Pc = bu.P_cut_W; Pn = bu.P_noncut_W; P_lv = 60;
t_slot = 8 * 3600; t_tr = 0.35 * 3600; t_shift = bu.t_shift; per = 600; t_cut = bu.duty_cut * per;
tP = 0; vP = 0; tD = 0; vD = 1; tH = 0; vH = 0; tC = 0; vC = 1;
for s = 0:2
    t0 = s * t_slot;
    % undock at t0 + 60 s (charging request dropped), HV request at t0 + 70 s, transit after HV is up
    tD = [tD, t0 + 60]; vD = [vD, 0]; %#ok<AGROW>
    tC = [tC, t0 + 55]; vC = [vC, 0]; %#ok<AGROW>
    tH = [tH, t0 + 70]; vH = [vH, 1]; %#ok<AGROW>
    ts = t0 + 80;
    tP = [tP, ts]; vP = [vP, Pn]; %#ok<AGROW>                       % transit out
    tw = ts + t_tr;
    for k = 0:(t_shift / per - 1)
        tP = [tP, tw + k * per, tw + k * per + t_cut]; vP = [vP, Pc, Pn]; %#ok<AGROW>
    end
    tb = tw + t_shift;                                              % transit back (non-cutting power)
    tP = [tP, tb]; vP = [vP, Pn]; %#ok<AGROW>
    te = tb + t_tr;
    tP = [tP, te]; vP = [vP, 0]; %#ok<AGROW>
    tH = [tH, te + 5]; vH = [vH, 0]; %#ok<AGROW>                   % HV off at the dock
    tD = [tD, te + 10]; vD = [vD, 1]; %#ok<AGROW>                  % dock contact
    tC = [tC, te + 20]; vC = [vC, 1]; %#ok<AGROW>                  % charge request
end
tP = [tP, H]; vP = [vP, 0];
Pdrv = max(vP - P_lv, 0) / 2;
Pdrv(vP == 0) = 0;
TB.loadL_t = tP; TB.loadL_v = Pdrv;
TB.loadR_t = tP; TB.loadR_v = Pdrv;
TB.load_is_power = 1;
TB.dock_t = [tD H]; TB.dock_v = [vD vD(end)];
TB.hv_t = [tH H]; TB.hv_v = [vH vH(end)];
TB.chg_t = [tC H]; TB.chg_v = [vC vC(end)];
TB.T_amb = 318.15; TB.G_solar = 600;
TB.MaxNonlinIter = 1; TB.MaxModeIter = 1;
TB.dec_fast = 200; TB.dec_slow = 100; TB.dec_bms = 10; TB.dec_truth = 10;   % 1 s logs (5 ms mission base)
TB.shift = struct('t_slot', t_slot, 't_transit', t_tr, 't_shift', t_shift, 'P_cut', Pc, 'P_noncut', Pn, 'P_lv', P_lv, ...
    'n', 3, 't_start', (0:2) * t_slot + 80 + t_tr);
end
