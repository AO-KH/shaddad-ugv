function [in, crit, info] = scn_S4_greenhouse(P, varargin)
%SCN_S4_GREENHOUSE  S4 Greenhouse coverage (Phase 2), architecture section 12 / D42 / D54.
%
%   [in, crit] = scn_S4_greenhouse(P)
%   greenhouse world (9.6 x 37.5 m house), mower (1.0 m swath, 10 % sidelap) at 0.5 m/s, mission
%   scripts/missions/greenhouse_std.json (boustrophedon lanes, LiDAR localisation inside, RTH to the dock).
%   Criteria: coverage (kpi_coverage) >= 95 % of the floor inset by c_in (D54: 0.40 m for the 0.78 m wide robot); mission time
%   <= 22 min; truth XTE RMS <= 0.05 m on rows, max <= 0.15 m (kpi_truth_xte); clearance >= 0.15 m to posts
%   and walls (Sim_Monitors cylinder + box monitor, D42); 0 collisions; scan-match valid > 80 % of the
%   attempts (EKF channel 5 accepted per 0.2 s while LiDAR localisation is active).

ip = inputParser;
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
t_start = 15;
[P, plan, info, mpc, tab] = scn_auto_setup(P, 'greenhouse_std', 'World', 4, 'Implement', 1, 't_start', t_start, ...
    'sw_impl', 2);                           % implement switch ON: the MCU runs the rotor only at sw_impl >= 2
assert(plan.valid, 'scn_S4: mission plan invalid');
cv = P.gnc.cov;                              % D54 inset: post edge + clearance + XTE + max(0, W/2 - w_tool/2)
c_in = cv.gh_post_edge + cv.gh_c_clear + cv.gh_c_xte + max(0, P.gnc.geom.W/2 - cv.gh_swath/2);
W = [0 9.6];  L = [0 37.5];                  % inner wall faces of the house (P.world film lines)
area = [W(1)+c_in L(1)+c_in; W(2)-c_in L(1)+c_in; W(2)-c_in L(2)-c_in; W(1)+c_in L(2)-c_in];
t_est = 0;
if isfield(info, 't_est_s'), t_est = info.t_est_s; end
T = ip.Results.StopTime;
if isempty(T), T = ceil(t_start + max(22*60, 1.1*t_est) + 30); end
VC = struct('VC_PHASE', 2, 'VC_IMPL', 1, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, 'S4', 'StopTime', T, 'Input', tab, 'VC', VC);
if ~isempty(mpc), in = in.setVariable('mpcRowObj', mpc); end
in = scn_logging('apply', in, P, T);
info.area = area;

crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S4.coverage', sprintf('coverage of the floor inset by c_in = %.2f m', c_in), 'truth.pose2D, gnc_status.seg_idx', ...
    @(S) kpi_coverage(S, P, plan, area), '>=', 0.95, '1');
crit(end+1) = local_c('S4.time', 'mission time START -> mission_done', 'gnc_status.mission_done', ...
    @(S) (scn_metrics('first', S.gnc_status.mission_done, @(v) v ~= 0, t_start) - t_start)/60, '<=', 22, 'min');
crit(end+1) = local_c('S4.xte_rms', 'truth XTE RMS on ROW segments', 'truth.pose2D', ...
    @(S) local_xte(S, plan, 'rms'), '<=', 0.05, 'm');
crit(end+1) = local_c('S4.xte_max', 'truth XTE max on ROW segments', 'truth.pose2D', ...
    @(S) local_xte(S, plan, 'max'), '<=', 0.15, 'm');
crit(end+1) = local_c('S4.clearance', 'minimum clearance to posts and walls', 'kpi.clear_min', ...
    @(S) S.kpi.clear_min.v(end), '>=', 0.15, 'm');
crit(end+1) = local_c('S4.no_coll', 'collisions', 'kpi.n_coll', @(S) double(S.kpi.n_coll.v(end)), '==', 0, '');
crit(end+1) = local_c('S4.scanmatch', 'scan-match accepted / attempts while LiDAR localisation is active', ...
    'nav_diag.enable(5), nav_state.lidarLocActive', @(S) local_sm(S), '>', 0.80, '1');
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function v = local_xte(S, plan, what)
[~, inf] = kpi_truth_xte(S, plan, 'Types', 2);
v = inf.(what);
end

function r = local_sm(S)
a = S.nav_state.lidarLocActive;
act = double(a.v(:)) ~= 0;
dur = sum(act)*median(diff(a.t));             % active time [s]
en = S.nav_diag.enable;
acc = double(en.v(:, 5)) ~= 0;
acc = acc & double(scn_metrics('resample', a, en.t)) ~= 0;
r = sum(acc)/max(dur/0.2, 1);                % one scan-match attempt per 0.2 s (D55, 5 Hz)
end
