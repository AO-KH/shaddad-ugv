function C = pwr_case_def(name)
%PWR_CASE_DEF  Scenario definitions of the T04 pack + BMS bench tests (owner: power, T04).
%
%   C = pwr_case_def(name)   C.name, C.T (stop time, s), C.VC_PACK, C.TB (bench stimulus, see
%                            tb_pack_bms_default), C.pmod (function handle P -> P applied before the
%                            run), C.long (true = long simulation, cached in work/T04),
%                            C.mission (true = mission mode: 5 ms plant / Simscape rate, see
%                            pwr_case_run; every long case except rtf_grouped), C.desc
%   names = pwr_case_def()   list of all case names
%
%   Cases (docs/specs/power.md section 8; build plan T04 acceptance):
%     precharge     HV_Enable_Req at 1 s: K2 / K1 / K2-open timing, HV_Ready (T-PWR-03 bench part)
%     mcu_timeout   BMS_REQ alive frozen at 5 s while HV_ON: HV opening sequence <= 0.6 s, Fault_Word b20
%     overcurrent   350 A discharge at 4 s, reset attempts at 5.1 s (current high) and 7 s (T-PWR-07)
%     ov_inject     forced 100 A charge + bms_fault (+0.5 V on group 1) at 3 s (T-PWR-08 OV path)
%     ov_forced     forced 100 A charge ignoring the BMS from SOC 0.975 (T-PWR-08 physical over-charge)
%     hppc          300 A 10 s pulse at SOC 0.5, 25 C (T-PWR-02)
%     energy        load / regen pulse train + dock charge, 1 ms energy integrals vs offline trapezoid (D40)
%     capacity      100 A CC discharge from 100 % to V_cell_min = 2.80 V, 25 C (T-PWR-01)
%     cccv25        CC-CV dock charge from SOC 0.10 at 25 C (T-PWR-09)
%     hot45         45 C, 600 W/m^2, fans auto, dock air off, 90 A charger, 0.10 -> 0.95 (T-PWR-10)
%     hot45_nofan   as hot45 with fans forced off, pack arriving at 56 C, 2 h: derating, no OT fault (T-PWR-10)
%     hot45_sealed  sealed pack (no fan path, no dock air), 45 C, 1000 W/m^2, 90 A, 0.10 -> 0.95 (review R-04)
%     hot50_dock    50 C, 600 W/m^2, dock air on (T-PWR-10b)
%     hot50_nodock  50 C, 600 W/m^2, dock air off, fans auto (T-PWR-10c)
%     soc_est       4 h mowing profile (1.95 kW mean, +/-1.5 kW pulses), SOC_init_est = truth + 0.10 (T-PWR-11)
%     balance       one group +3 % SOC, three dock CC-CV cycles with top rest (T-PWR-12)
%     eol_mow       EOL pack (cap x0.8, R0 x1.5), 45 C, 1000 W/m^2, mow shift duty from 0.95 to SOC 0.10
%                   (T-PWR-13 mow, T-PWR-13b, T-PWR-17)
%     eol_till      EOL pack, till shift duty 4 h (T-PWR-13 till)
%     rtf_grouped   1 h mowing bench, Grouped (speed)
%     rtf_lumped    1 h mowing bench, Lumped (speed)

all = {'precharge', 'mcu_timeout', 'overcurrent', 'ov_inject', 'ov_forced', 'hppc', 'energy', 'capacity', ...
       'cccv25', 'hot45', 'hot45_nofan', 'hot45_sealed', 'hot50_dock', 'hot50_nodock', 'soc_est', 'balance', 'eol_mow', ...
       'eol_till', 'rtf_grouped', 'rtf_lumped'};
if nargin < 1, C = all; return, end
TB = tb_pack_bms_default();
C = struct('name', name, 'T', 10, 'VC_PACK', 1, 'TB', TB, 'pmod', @(P) P, 'long', false, 'mission', false, 'desc', '');
on = @(t) deal([0 t 1e7], [0 1 1]);       % [breakpoints, values]: off until t, then on
Pw = param_power();                          % real 13S15P pack: case currents from the BMS limits
switch name
    case 'precharge'
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        C.T = 4; C.desc = 'HV_Enable_Req at 1 s (MCU frame), precharge into 5.5 mF';
    case 'mcu_timeout'
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        C.TB.load_t = [0 3 1e7]; C.TB.load_I = [0 20 20];
        C.TB.alive_freeze_t = 5;
        C.T = 7; C.desc = 'BMS_REQ alive frozen at 5 s while HV_ON with 20 A load';
    case 'overcurrent'
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        Ioc = Pw.bms.I_dis_cont + 50;
        C.TB.load_t = [0 4 1e7]; C.TB.load_I = [0 Ioc Ioc];
        C.TB.rst_t = [0 5.1 5.3 7.0 7.2 1e7]; C.TB.rst_v = [0 1 0 1 0 0];
        C.T = 8.5; C.desc = sprintf('%.0f A from 4 s (> DCL %.0f + %.0f A); reset requests at 5.1 s and 7.0 s', ...
            Ioc, Pw.bms.I_dis_cont, Pw.bms.OC_ErrorLimit);
    case 'ov_inject'
        C.pmod = @(P) local_soc(P, 0.90);
        [C.TB.dock_t, C.TB.dock_v] = on(0.5);
        [C.TB.chg_t, C.TB.chg_v] = on(1);
        C.TB.chg_force = 1; C.TB.chg_force_I = Pw.bms.I_chg_max;
        C.TB.flt_t = [0 6 1e7]; C.TB.flt_v = [0 1 1];
        C.T = 9; C.desc = sprintf('charger forced to %.0f A, bms_fault (+0.5 V on V_cell(1)) at 6 s', Pw.bms.I_chg_max);
    case 'ov_forced'
        C.pmod = @(P) local_soc(P, 0.975);
        [C.TB.dock_t, C.TB.dock_v] = on(0.5);
        [C.TB.chg_t, C.TB.chg_v] = on(1);
        C.TB.chg_force = 1; C.TB.chg_force_I = Pw.bms.I_chg_max;
        C.TB.dec_fast = 10;
        C.T = 400; C.desc = sprintf('charger forced to %.0f A ignoring the BMS request from SOC 0.975', Pw.bms.I_chg_max);
    case 'hppc'
        C.pmod = @(P) local_temp(local_soc(P, 0.50, 0), 298.15);
        C.TB.T_amb = 298.15;
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        C.TB.load_t = [0 5 15 1e7]; C.TB.load_I = [0 Pw.bms.I_dis_cont 0 0];
        C.T = 20; C.desc = sprintf('HPPC %.0f A (DCL) 10 s pulse at SOC 0.5, 25 C', Pw.bms.I_dis_cont);
    case 'energy'
        % load / regen pulses with HV on (1..62 s), then a dock charge (dock 63 s, Charge_Enable 64 s) so
        % that E_chg_bus / E_in see the charger path as well
        C.TB.hv_t = [0 1 62 1e7]; C.TB.hv_v = [0 1 0 0];
        C.TB.load_t = [0 3 10 15 25 30 40 45 60 1e7]; C.TB.load_I = [0 50 150 -40 120 0 -20 80 0 0];
        C.TB.dock_t = [0 63 1e7]; C.TB.dock_v = [0 1 1];
        C.TB.chg_t = [0 64 1e7]; C.TB.chg_v = [0 1 1];
        C.T = 95; C.desc = 'load / regen pulses, then a dock charge, for the energy-integral check';
    case 'capacity'
        % pack characterisation: the bench Stop block ends the run at the true V_cell_min 2.80 V; the
        % BMS discharge-limit enforcement (SOC taper x0.5 / x0.2 below 10 / 5 %, overcurrent = DCL + 20 A)
        % would open K1 under the forced 100 A load before the cut-off, so it is relaxed for this case
        C.pmod = @(P) local_char(local_temp(local_soc(P, 1.0, 0), 298.15));
        C.TB.T_amb = 298.15;
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        C.TB.load_t = [0 3 1e7]; C.TB.load_I = [0 100 100];
        C.TB.stop_Vmin = 2.80; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.T = 4*3600; C.long = true; C.desc = 'T-PWR-01 100 A CC from 100 % to V_cell_min 2.80 V, 25 C';
    case 'cccv25'
        % series-consistent start state: the seeded group SOCs (defined at SOC0 = 0.95) minus the SAME
        % discharged charge (0.95 - 0.10) x 300 Ah for every group (series string), i.e. the pack as it
        % comes back from a shift; the top-of-charge spread is then the seeded imbalance
        C.pmod = @(P) local_temp(local_soc_series(P, 0.10), 298.15);
        C.TB.T_amb = 298.15;
        [C.TB.dock_t, C.TB.dock_v] = on(0.5);
        [C.TB.chg_t, C.TB.chg_v] = on(1);
        C.TB.stop_on_done = 1; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.T = 5*3600; C.long = true; C.desc = 'T-PWR-09 CC-CV charge from SOC 0.10 at 25 C';
    case {'hot45', 'hot45_nofan', 'hot50_dock', 'hot50_nodock'}
        Ta = 318.15; dockAir = false; fan = 0; Ichg = 90;
        if startsWith(name, 'hot50'), Ta = 323.15; end
        if strcmp(name, 'hot50_dock'), dockAir = true; end
        if strcmp(name, 'hot45_nofan'), fan = 1; end
        C.VC_PACK = 2;
        T0 = Ta;
        % fans off: the pack arrives at 56 C (inside the 50..60 C derating ramp of the charge table), so
        % the CCL derating is exercised (from a 45 C start the cell mass, tau ~ 3 h, stays below 55 C
        % during a whole charge)
        if strcmp(name, 'hot45_nofan'), T0 = 329.15; end
        C.pmod = @(P) local_hot(P, Ta, dockAir, Ichg, T0);
        C.TB.T_amb = Ta; C.TB.G_solar = 600; C.TB.fan_mode = fan;
        [C.TB.dock_t, C.TB.dock_v] = on(0.5);
        [C.TB.chg_t, C.TB.chg_v] = on(1);
        C.TB.stop_SOCmax = 0.95; C.TB.stop_on_done = 1; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        if strcmp(name, 'hot45_nofan'), C.TB.stop_SOCmax = 2; end
        C.T = 5*3600; C.long = true;
        if strcmp(name, 'hot45_nofan'), C.T = 2*3600; end
        C.desc = sprintf('hot charge %.0f C, 600 W/m^2, dock air %d, fan mode %d, charger %d A, pack start %.0f C, Lumped pack', ...
            Ta - 273.15, dockAir, fan, Ichg, T0 - 273.15);
    case 'hot45_sealed'
        % review R-04 / BMS-01 (2026-11-15): the real pack is sealed (battery spec sheet p. 2), so no fan air
        % path, no fan power and no dock air into the box; full sun. Otherwise as hot45: 45 C, 90 A charger,
        % SOC 0.10 -> 0.95, the pack starting at ambient
        Ta = 318.15;
        C.VC_PACK = 2;
        C.pmod = @(P) local_sealed(local_hot(P, Ta, false, 90, Ta));
        C.TB.T_amb = Ta; C.TB.G_solar = 1000; C.TB.fan_mode = 1;
        [C.TB.dock_t, C.TB.dock_v] = on(0.5);
        [C.TB.chg_t, C.TB.chg_v] = on(1);
        C.TB.stop_SOCmax = 0.95; C.TB.stop_on_done = 1; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.T = 5*3600; C.long = true;
        C.desc = 'sealed pack (no fan path, no dock air), 45 C, 1000 W/m^2, charger 90 A, from SOC 0.10 at 45 C (R-04)';
    case 'soc_est'
        % true pack at 0.80 (series-consistent shift) so that the +0.10 initial estimator error is not
        % clipped at 1.0 (from 0.95 it would be +0.05 at most)
        C.pmod = @(P) local_bias(local_temp(local_soc_series(P, 0.80), 298.15), 0.10);
        C.TB.T_amb = 298.15;
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        [C.TB.load_t, C.TB.load_I] = local_mow_pulses(3, 4*3600 + 4*3600, 1950, 1500, 7);
        C.TB.load_is_power = 1; C.TB.stop_SOCmin = 0.10; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.T = 9*3600; C.long = true; C.desc = 'T-PWR-11 mowing profile 1.95 kW +/-1.5 kW pulses until SOC_min 0.10';
    case 'balance'
        C.pmod = @(P) local_temp(local_imbalance(P), 298.15);
        C.TB.T_amb = 298.15;
        [C.TB, C.T] = local_balance_cycles(C.TB, 3);
        C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.long = true; C.desc = 'T-PWR-12 one group +3 % SOC, 3 x (dock CC-CV + top rest, 10 % discharge)';
    case {'eol_mow', 'eol_till'}
        C.pmod = @(P) local_eol(P);
        C.TB.T_amb = 318.15; C.TB.G_solar = 1000;
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        Pw = param_power();
        if strcmp(name, 'eol_mow'), Pcut = Pw.budget.P_cut_W; else, Pcut = 2195; end
        [C.TB.load_t, C.TB.load_I] = local_duty(3, 12*3600, Pcut, Pw.budget.P_noncut_W, Pw.budget.duty_cut, 600);
        C.TB.load_is_power = 1; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        if strcmp(name, 'eol_mow')
            C.VC_PACK = 1; C.TB.stop_SOCmin = 0.10; C.T = 8*3600;
        else
            C.VC_PACK = 2; C.T = 4*3600 + 3;
        end
        C.long = true; C.desc = sprintf('EOL pack (cap x0.8, R0 x1.5), 45 C, 1000 W/m^2, %s shift duty from SOC 0.95', name(5:end));
    case {'rtf_grouped', 'rtf_lumped'}
        if strcmp(name, 'rtf_lumped'), C.VC_PACK = 2; end
        [C.TB.hv_t, C.TB.hv_v] = on(1);
        [C.TB.load_t, C.TB.load_I] = local_mow_pulses(3, 3600, 1950, 1500, 11);
        C.TB.load_is_power = 1; C.TB.dec_fast = 100; C.TB.dec_cmd = 10;
        C.T = 3600; C.long = true; C.desc = '1 h mowing bench (speed)';
    otherwise
        error('pwr_case_def:name', 'Unknown case %s', name);
end
% mission mode (5 ms plant / local-solver rate, BMS rates unchanged; spec R9, T-PWR-19) for every
% multi-hour case; the Grouped speed case stays at 1 ms (T-PWR-19 "Grouped at 1 ms")
C.mission = C.long && ~strcmp(name, 'rtf_grouped');
end

% -------------------------------------------------------------------------------------------
function P = local_soc(P, soc, spread)
% initial truth SOC (all groups soc + the seeded spread, unless spread = 0) and BMS NVM SOC = truth
v = P.power.pack.var.SOC0_vec;
if nargin > 2 && spread == 0
    v = soc*ones(size(v));
else
    v = soc + (v - P.power.pack.SOC0);
end
P.power.pack.var.SOC0_vec = min(v, 1);
P.power.bms.est.SOC_init = min(max(P.power.pack.var.SOC0_vec + P.power.bms.ekf.SOC_init_bias, 0), 1);
end

function P = local_soc_series(P, soc)
% group SOCs after discharging (SOC0 - soc) x Q_nom from the seeded state (same Ah from every group)
pk = P.power.pack;
Qg = pk.Q_Ah*pk.age.cap_factor*(1 + pk.var.cap_dev_pct(:)'/100);
v = pk.var.SOC0_vec(:)' - (pk.SOC0 - soc)*pk.Q_Ah*pk.age.cap_factor./Qg;
P.power.pack.var.SOC0_vec = reshape(min(max(v, 0), 1), size(pk.var.SOC0_vec));
P.power.bms.est.SOC_init = min(max(P.power.pack.var.SOC0_vec + P.power.bms.ekf.SOC_init_bias, 0), 1);
end

function P = local_temp(P, T)
P.power.thermal.T_init_K = T;
end

function P = local_char(P)
% capacity characterisation: no BMS discharge-limit enforcement against the forced CC load
P.power.bms.soc_taper.k = ones(size(P.power.bms.soc_taper.k));
P.power.bms.OC_ErrorLimit = 1e3;
end

function P = local_bias(P, b)
P.power.bms.ekf.SOC_init_bias = b;
P.power.bms.est.SOC_init = min(max(P.power.pack.var.SOC0_vec + b, 0), 1);
end

function P = local_hot(P, Ta, dockAir, Ichg, T0)
P = local_temp(local_soc(P, 0.10), T0);
P.power.thermal.dock_air.enable = dockAir;
P.power.charger.I_max = Ichg;
end

function P = local_sealed(P)
% sealed enclosure (review R-04): no fan air path (h_fan ~ 0) and no fan power; dock air stays outside
P.power.thermal.h_fan = 1e-6;
P.power.thermal.P_fan = 0;
P.power.thermal.dock_air.enable = false;
end

function P = local_imbalance(P)
v = 0.93*ones(1, P.power.pack.Ns);
v(5) = 0.96;                                         % one group +3 % SOC
P.power.pack.var.SOC0_vec = v;
P.power.bms.est.SOC_init = v;
end

function P = local_eol(P)
P = local_temp(P, 318.15);
P.power.pack.age.cap_factor = P.power.bms.est.eol_test_cap;
P.power.pack.age.R0_factor = P.power.bms.est.eol_test_R0;
P.power.bms.est.AH_init = P.power.pack.Q_Ah*P.power.bms.est.eol_test_cap;   % learned capacity in NVM
end

function [t, v] = local_mow_pulses(t0, T, Pmean, dP, seed)
% mowing power profile: mean Pmean, +/-dP pulses of 5..30 s separated by 5..30 s, zero-mean pulses
rs = RandStream('mt19937ar', 'Seed', seed);
t = [0 t0]; v = [0 Pmean];
tc = t0;
while tc < t0 + T
    d = 5 + 25*rand(rs); g = 5 + 25*rand(rs);
    s = sign(rand(rs) - 0.5);
    t = [t, tc + g, tc + g + d]; %#ok<AGROW>
    v = [v, Pmean + s*dP, Pmean]; %#ok<AGROW>
    tc = tc + g + d;
end
t(end+1) = 1e7; v(end+1) = v(end);
end

function [t, v] = local_duty(t0, T, Pcut, Pnon, duty, period)
% shift duty: duty*period at Pcut, the rest at Pnon (repeated)
t = 0; v = 0;
tc = t0;
while tc < t0 + T
    t = [t, tc, tc + duty*period]; %#ok<AGROW>
    v = [v, Pcut, Pnon]; %#ok<AGROW>
    tc = tc + period;
end
t(end+1) = 1e7; v(end+1) = v(end);
end

function [TB, T] = local_balance_cycles(TB, n)
% each cycle: dock + charge enable for 2.2 h (CC-CV, then CHARGE_DONE top rest), undock, HV on,
% 100 A for 1080 s (30 Ah = 10 %), HV off
tc = 1; Tdock = 2.2*3600; Tdis = 1080;
dk_t = 0; dk_v = 0; ce_t = 0; ce_v = 0; hv_t = 0; hv_v = 0; ld_t = 0; ld_v = 0;
for k = 1:n
    dk_t = [dk_t, tc - 0.5, tc + Tdock]; dk_v = [dk_v, 1, 0]; %#ok<AGROW>
    ce_t = [ce_t, tc, tc + Tdock]; ce_v = [ce_v, 1, 0]; %#ok<AGROW>
    t1 = tc + Tdock + 5;
    hv_t = [hv_t, t1, t1 + Tdis + 10]; hv_v = [hv_v, 1, 0]; %#ok<AGROW>
    ld_t = [ld_t, t1 + 3, t1 + 3 + Tdis]; ld_v = [ld_v, 100, 0]; %#ok<AGROW>
    tc = t1 + Tdis + 30;
end
TB.dock_t = [dk_t 1e7]; TB.dock_v = [dk_v 0];
TB.chg_t = [ce_t 1e7]; TB.chg_v = [ce_v 0];
TB.hv_t = [hv_t 1e7]; TB.hv_v = [hv_v 0];
TB.load_t = [ld_t 1e7]; TB.load_I = [ld_v 0];
T = tc + 600;
end
