function C = pp_case_def(name)
%PP_CASE_DEF  Long T05 bench cases run through the content-keyed cache pp_case (owner: power, T05).
%
%   C = pp_case_def(name)   name = 'audit_1h' | 'shift_45' | 'shift_50' | 'overload_06b'
%   Fields: name, desc, mdl, TB, VC (variant controls), T (s), mode ('normal' | 'mission'), pmod (MATLAB
%   statements applied to P, '' = none), siglog (signal logging on/off), chunk (s of simulated time per
%   pp_case call when chunked).
C = struct('name', name, 'desc', '', 'mdl', 'tb_power_plant', 'TB', tb_power_plant_default(), ...
    'VC', struct('VC_PACK', 2, 'VC_IMPL', 0, 'VC_CHG', 1), 'T', 10, 'mode', 'normal', 'pmod', '', ...
    'siglog', false, 'chunk', 1200);
switch name
    case 'audit_1h'
        % 1 h energy audit (eps_el, D40): mower running, traction cycles with regen, lift pulses, LV loads,
        % docked charging in the last 10 min
        C.desc = 'energy audit 1 h: traction + regen, mower 2800 rpm, lift, LV, dock charge 3000..3600 s';
        C.VC.VC_IMPL = 1;
        C.T = 3600;
        TB = C.TB;
        TB.hv_t = [0 1 3005 1e7]; TB.hv_v = [0 1 0 0];
        TB.k3_t = [0 0.5 1e7]; TB.k3_v = [0 1 1];
        TB.ien_t = [0 5 3000 1e7]; TB.ien_v = [0 1 0 0];
        TB.ispd_t = [0 5 1e7]; TB.ispd_v = [0 2800 2800];
        tc = 60:60:2940;
        tL = [0, reshape([tc; tc + 30; tc + 35], 1, []), 3000, 1e7];
        vL = [0, reshape(repmat([15; -5; 0], 1, numel(tc)), 1, []), 0, 0];   % traction pulses per side (750 W motors)
        TB.loadL_t = tL; TB.loadL_v = vL; TB.loadR_t = tL; TB.loadR_v = vL;
        tl = 100:120:2900;
        TB.lift_t = [0, reshape([tl; tl + 2], 1, []), 1e7]; TB.lift_v = [0, reshape(repmat([1; 0], 1, numel(tl)), 1, []), 0];
        TB.dock_t = [0 3010 1e7]; TB.dock_v = [0 1 1];
        TB.chg_t = [0 3020 1e7]; TB.chg_v = [0 1 1];
        TB.dec_fast = 1000; TB.dec_slow = 100; TB.dec_bms = 10; TB.dec_truth = 10;
        C.TB = TB;
    case {'shift_45', 'shift_50'}
        % T-PWR-20 (D45 / D46): 24 h, three 8 h slots (transit, 4 h shift duty of P.power.budget, transit, dock charge)
        C.mdl = 'tb_shift_cycle';
        C.TB = tb_power_plant_default('shift');
        C.T = 24 * 3600;
        C.mode = 'mission';
        C.chunk = 3600;
        if strcmp(name, 'shift_45')
            C.desc = 'T-PWR-20: 24 h shift cycle at 45 C ambient, 600 W/m^2, dock air OFF';
            C.TB.T_amb = 318.15;
            C.pmod = 'P.power.thermal.dock_air.enable = false;';
        else
            C.desc = 'T-PWR-20: 24 h shift cycle at 50 C ambient, 600 W/m^2, dock air ON';
            C.TB.T_amb = 323.15;
            C.pmod = 'P.power.thermal.dock_air.enable = true;';
        end
    case 'overload_06b'
        % T-PWR-06b: overload of 1.5 x the F_TL rating on DC48_TracL after HV is up (fuse curve: 1.5 x -> 200 s;
        % 225 A for the former 150 A fuse, 187.5 A for the 125 A fuse of the four-motor drive), fault mode
        % (Ts.local = Ts.base = P.sim.ssc.Ts_fault = 1e-4 s)
        Pw = param_power(); I_ov = 1.5*Pw.pdu.F_TL.I_rated;
        C.desc = sprintf('T-PWR-06b: %.1f A (1.5 x F_TL) on DC48_TracL from 2 s, Ts.local 1e-4 s', I_ov);
        C.T = 320;
        C.mode = 'fault';
        TB = C.TB;
        TB.hv_t = [0 0.2 1e7]; TB.hv_v = [0 1 1];
        TB.loadL_t = [0 2 1e7]; TB.loadL_v = [0 I_ov I_ov];
        TB.dec_fast = 1000; TB.dec_slow = 10; TB.dec_bms = 1; TB.dec_truth = 1;   % log_fast at 0.1 s
        C.TB = TB;
        C.chunk = 80;
    otherwise
        error('pp_case_def:name', 'Unknown case %s', name);
end
end
