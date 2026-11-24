function [in, crit] = scn_S7_gnss_dropout(P, varargin)
%SCN_S7_GNSS_DROPOUT  S7 GNSS dropout (Phase 2), architecture section 12 / D49.
%
%   [in, crit] = scn_S7_gnss_dropout(P, 'Variant', v)   v = 'farm' (default) | 'wet' | 'temp'
%   Autonomous straight aisle in palm_farm (aisle x = 16 m, y 14 -> 102 m = 88 m) at 1.0 m/s, RTK fixed
%   (open sky on the aisle centre line). ScenarioIn.flt.gnss_outage 20-50 s (30 s), gnss_force_float
%   50-60 s, then RTK fixed again. Variants (D49): farm soil (default), wet loam (env override soil 3),
%   10 K enclosure temperature ramp (ambient override ramp +10 K over the run, the enclosure follows).
%   Criteria: position error at the end of the outage <= 0.5 m; heading error <= 2 deg; truth XTE <= 0.3 m
%   during the outage; position error < 0.05 m within 5 s after RTK fixed returns; NEES (pose [pE pN psi]
%   with NavState.covPose) inside the chi-square 95 % bound for >= 90 % of the samples.

ip = inputParser;
ip.addParameter('Variant', 'farm');
ip.addParameter('StopTime', 125);
ip.parse(varargin{:});
var = lower(char(ip.Results.Variant));
T = ip.Results.StopTime;
xr = 16;  y0 = 14;  y1 = 102;
ms = struct('mission_id', 701, 'mission_type', 'PALM_ROW_PATROL', 'world_id', 3, ...
    'rows_enu', [xr y0 xr y1], 'home_enu', [xr y0 - 4 pi/2], 'swath_m', 1, 'turn_radius_m', 2, ...
    'skip_rows', 0, 'speed_work', 1.0, 'speed_transit', 1.0, 'speed_turn_max', 0.8, ...
    'implement', struct('type', 'NONE'), 'allow_bypass', false, 'lane_halfwidth_m', 2.5, 'end_action', 'STOP');
[P, plan, ~, mpc, tab] = scn_auto_setup(P, ms, 'World', 3, 'Implement', 0);
t_o0 = 20; t_o1 = 50; t_f1 = 60;
tab.flt.gnss_outage = [0 0; t_o0 1; t_o1 0];
tab.flt.gnss_force_float = [0 0; t_o1 1; t_f1 0];
switch var
    case 'farm'
    case 'wet'
        tab.env.use_soil = [0 1];
        tab.env.soil_id = [0 3];
    case 'temp'
        tt = (0:1:T)';
        tab.env.use_T = [0 1];
        tab.env.T_amb = [tt, P.world.env.T_amb + 10*tt/T];
    otherwise
        error('scn_S7_gnss_dropout:variant', 'Variant must be farm, wet or temp');
end
VC = struct('VC_PHASE', 2, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, ['S7' var], 'StopTime', T, 'Input', tab, 'VC', VC);
if ~isempty(mpc), in = in.setVariable('mpcRowObj', mpc); end
in = scn_logging('apply', in, P, T);

chi95 = 7.8147;                              % chi2inv(0.95, 3)
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
tag = ['S7' var];
crit(end+1) = local_c([tag '.pos_end'], 'position error at the end of the outage', 'nav_state.pos_W, truth.pos_W', ...
    @(S) local_pos_err(S, t_o1), '<=', 0.5, 'm');
crit(end+1) = local_c([tag '.yaw_end'], 'heading error at the end of the outage', 'nav_state.eul(1), truth.eul(1)', ...
    @(S) local_yaw_err(S, t_o1)*180/pi, '<=', 2, 'deg');
crit(end+1) = local_c([tag '.xte_out'], 'max truth XTE during the outage', 'truth.pos_W', ...
    @(S) max(abs(local_win(S.truth.pos_W, t_o0, t_o1, 1) - xr)), '<=', 0.3, 'm');
crit(end+1) = local_c([tag '.pos_rec'], 'position error 5 s after RTK fixed returns', 'nav_state, truth', ...
    @(S) local_pos_err(S, t_f1 + 5), '<', 0.05, 'm');
crit(end+1) = local_c([tag '.nees'], 'fraction of samples with NEES <= chi2(0.95, 3)', 'nav_state.covPose', ...
    @(S) local_nees(S, chi95), '>=', 0.90, '1');
crit(end+1) = local_c([tag '.moving'], 'robot on the aisle during the outage (distance driven 20-50 s)', 'truth.pos_W', ...
    @(S) scn_metrics('path', S.truth.pos_W, t_o0, t_o1), '>=', 20, 'm');
crit(end+1) = local_c([tag '.no_coll'], 'collisions', 'kpi.n_coll', @(S) double(S.kpi.n_coll.v(end)), '==', 0, '');
assert(plan.valid, 'scn_S7: mission plan invalid');
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function v = local_win(sig, t0, t1, col)
m = sig.t >= t0 & sig.t <= t1;
v = double(sig.v(m, col));
end

function e = local_pos_err(S, t)
pn = scn_metrics('at', S.nav_state.pos_W, t);
pt = scn_metrics('at', S.truth.pos_W, t);
e = norm(pn(1:2) - pt(1:2));
end

function e = local_yaw_err(S, t)
yn = scn_metrics('at', S.nav_state.eul, t);
yt = scn_metrics('at', S.truth.eul, t);
e = abs(atan2(sin(yn(1) - yt(1)), cos(yn(1) - yt(1))));
end

function f = local_nees(S, lim)
% NEES of the planar pose [pE pN psi] with NavState.covPose, over the samples with navValid after 15 s
n = S.nav_state;
t = n.pos_W.t;
ok = n.navValid.v(:) ~= 0 & t > 15;
pt = scn_metrics('resample', S.truth.pos_W, t);
yt = scn_metrics('resample', S.truth.eul, t);
e = [double(n.pos_W.v(:, 1:2)) - pt(:, 1:2), atan2(sin(double(n.eul.v(:, 1)) - yt(:, 1)), ...
                                                   cos(double(n.eul.v(:, 1)) - yt(:, 1)))];
C = double(n.covPose.v);
nees = nan(numel(t), 1);
for k = find(ok)'
    Ck = reshape(C(k, :), 3, 3);
    if all(isfinite(Ck(:))) && rcond(Ck) > 1e-15
        nees(k) = e(k, :)/Ck*e(k, :)';
    end
end
f = mean(nees(ok) <= lim);
end
