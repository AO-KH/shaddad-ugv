function [in, crit, info] = scn_S3_endurance(P, varargin)
%SCN_S3_ENDURANCE  S3 Battery endurance (Phase 2), architecture section 12 / D45 / D46 / D56.
%
%   [in, crit] = scn_S3_endurance(P, 'Variant', v, 'Duration', T)
%     v = 'bol' (default, 45 C) | 'eol' (capacity x 0.8, R0 x 1.5, 45 C) | 'hot' (BOL, 50 C)
%     T = P.power.budget.t_shift (default, 3 h)
%   4 h palm patrol (scripts/missions/palm_demo.json, 6 aisles, RTH to the dock) repeated: a new START is
%   issued every mission period (compiler estimate + 60 s; the FSM goes IDLE after DOCKED). Mower on along
%   the rows (about 70 % duty, D46) at v_work_palm 1.0 m/s, ambient override 45 C (hot case 50 C),
%   SOC0 = 0.95, endurance fidelity set P.sim.endurance (Lumped pack, IDEAL drive, LoFi structure,
%   LiDAR / perception 5 Hz, VC_TEB 0, VC_VIZ 0), P.scen.stop_on_mission_done = false.
%   Criteria: completes the duration, no BMS trip; SOC_end >= soc_rth (P.power.budget.soc_window(2), D46); T_cell_max <= 328.15 K;
%   eps_el <= 1 % and eps_mech <= 1 % (D40); RTF >= 4 (accelerator; BLOCKED without a C compiler).

ip = inputParser;
ip.addParameter('Variant', 'bol');
ip.addParameter('Duration', []);              % default: P.power.budget.t_shift (3 h for the real 3 kWh pack)
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
var = lower(char(ip.Results.Variant));
T = ip.Results.Duration;
if isempty(T), T = getfield(param_power(), 'budget', 't_shift'); end
if ~isempty(ip.Results.StopTime), T = ip.Results.StopTime; end
t_start = 15;
[P, plan, info, mpc, tab] = scn_auto_setup(P, 'palm_demo', 'World', 3, 'Implement', 1, 't_start', t_start, ...
    'sw_impl', 2);                           % implement switch ON: the MCU runs the rotor only at sw_impl >= 2
assert(plan.valid, 'scn_S3: mission plan invalid');
E = P.sim.endurance;
P.scen.stop_on_mission_done = false;
T_amb = 318.15;                              % 45 C (P.power.pack.SOC0 = 0.95 = P.scen.soc0 by default)
switch var
    case 'bol'
    case 'eol'
        P.power.pack.age.cap_factor = P.power.bms.est.eol_test_cap;
        P.power.pack.age.R0_factor  = P.power.bms.est.eol_test_R0;
    case 'hot'
        T_amb = P.scen.T_amb_hot;
    otherwise
        error('scn_S3_endurance:variant', 'Variant must be bol, eol or hot');
end
tab.env.use_T = [0 1];
tab.env.T_amb = [0 T_amb];
% repeated mission: START every period (cmd_seq increments)
t_est = 600;
if isfield(info, 't_est_s') && info.t_est_s > 0, t_est = info.t_est_s; end
per = ceil(t_est + 60);
ts = (t_start:per:T - 60)';
tab.mcmd.cmd = [0 0; ts 2*ones(size(ts))];
tab.mcmd.cmd_seq = [0 0; ts (1:numel(ts))'];
VC = struct('VC_PHASE', 2, 'VC_IMPL', 1, 'VC_VIZ', E.VC_VIZ, 'VC_OPSRC', 1, 'VC_PACK', E.VC_PACK, ...
    'VC_DRIVE_FID', E.VC_DRIVE_FID, 'VC_STRUCT_FID', E.VC_STRUCT_FID, 'VC_LIDAR', E.VC_LIDAR, 'VC_TEB', E.VC_TEB);
in = scn_base_input(P, ['S3' var], 'StopTime', T, 'Input', tab, 'VC', VC);
if ~isempty(mpc), in = in.setVariable('mpcRowObj', mpc); end
in = scn_logging('apply', in, P, T);
info.period = per;  info.n_missions = numel(ts);

soc_rth = P.power.budget.soc_window(2);      % end-of-shift SOC of the energy budget (D46)
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S3.complete', sprintf('simulated time / duration (%.0f s)', T), 'kpi', ...
    @(S) S.kpi.t_mission.t(end)/T, '>=', 0.999, '1');
crit(end+1) = local_c('S3.no_bms_trip', 'BMS fault level < 3 and HV_Ready kept', 'bms_telem', ...
    @(S) max(double(S.bms_telem.Fault_Level.v)) < 3 && all(S.bms_telem.HV_Ready.v(S.bms_telem.HV_Ready.t > 2) ~= 0), ...
    'true', 1, '');
crit(end+1) = local_c('S3.soc_end', sprintf('SOC at the end (soc_rth = %.3f, D46)', soc_rth), 'power_truth.SOC_pack_true', ...
    @(S) S.power_truth.SOC_pack_true.v(end), '>=', soc_rth, '1');
crit(end+1) = local_c('S3.T_cell', 'maximum cell temperature', 'kpi.T_cell_max', ...
    @(S) S.kpi.T_cell_max.v(end), '<=', 328.15, 'K');
crit(end+1) = local_c('S3.eps_el', 'electrical energy-audit residual', 'kpi.eps_el', @(S) S.kpi.eps_el.v(end), '<=', 0.01, '1');
crit(end+1) = local_c('S3.eps_mech', 'mechanical energy-audit residual', 'kpi.eps_mech', @(S) S.kpi.eps_mech.v(end), '<=', 0.01, '1');
c = local_c('S3.rtf', 'RTF >= 4 in accelerator with the endurance set', 'run wall time', @(S) NaN, '>=', 4, '');
c.blocked = 'BLOCKED: no C compiler (accelerator unavailable); Normal-mode RTF in res.rtf';
crit(end+1) = c;
end

function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end
