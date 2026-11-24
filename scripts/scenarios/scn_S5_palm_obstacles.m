function [in, crit, info] = scn_S5_palm_obstacles(P, varargin)
%SCN_S5_PALM_OBSTACLES  S5 Palm-row patrol with obstacles (Phase 2), architecture section 12 / D42.
%
%   [in, crit] = scn_S5_palm_obstacles(P)
%   palm_farm: the first 5 aisles of P.world.rows driven in alternate directions with 4 headland turns at
%   1.0 m/s (turn radius 2 m, bypass allowed, lane half-width 2.5 m as scripts/missions/palm_demo.json),
%   end action STOP. A 0.6 x 0.6 x 0.6 m box is appended to P.world.obstacles.box on the aisle-2 centre
%   line 40 m from the aisle start AFTER the mission is compiled (an unknown obstacle; T02 note: the dummy
%   row of an empty list is replaced) and the occupancy grid is rebuilt with world_occupancy (D42). A
%   pedestrian (r = 0.3 m, 0.8 m/s, ScenarioIn.obs) walks from the tree line into the aisle-4 centre line,
%   waits 8 s and walks out, timed from the plan's nominal timing to meet the robot.
%   Criteria: 0 collisions; clearance >= 0.5 m to the box and >= 1.0 m to the person (Sim_Monitors per-class
%   minima); deceleration starts <= 0.5 s after the box is inside 10 m LiDAR range in the robot's corridor;
%   |XTE| < 0.1 m within 3 m after passing the box; route time <= 1.25 x nominal.
%
%   [in, crit] = scn_S5_palm_obstacles(P, 'Variant', 'b')     run_scenario id 'S5b' (critfix R-02 system check,
%   2026-11-15): the lost LiDAR scan in AUTO, on aisle 1 (104 m: the robot stays on that ROW for the whole run).
%   ScenarioIn.flt.lidar_blackout 35-38 s on the ROW, and 74-82 s during an OBSTACLE_HOLD in front of a person (r 0.3 m,
%   static on the aisle-1 centre line 34 m from the aisle start, present 45-80 s, so it leaves during the blank). The
%   bypass is off in this variant (allow_bypass false), so the person hold lasts until the person has left and the
%   scan is back (no H2 detour, whatever the exact arrival time). StopTime 95 s. Criteria: on a ROW at the first
%   blackout; SENSOR_LOST hold <= 0.4 s after each blackout starts; still in the hold, SENSOR_LOST shown, and stopped
%   while the scan is blank (no release on a blank scan); the hold ends (H1) 1.8-2.6 s after the scan returns;
%   fsm.impl_allow false from the loss to the end of each hold (the implement latch; S5 carries no implement);
%   an OBSTACLE / PERSON hold active when the second blackout starts; 0 collisions.

ip = inputParser;
ip.addParameter('StopTime', []);
ip.addParameter('Variant', '');
ip.parse(varargin{:});
var = lower(char(ip.Results.Variant));
if ~ismember(var, {'', 'b'}), error('scn_S5_palm_obstacles:variant', 'Variant must be '''' (nominal) or ''b'''); end
Pw = shaddad_params('World', 3, 'Implement', 0, 'Phase', 2, 'Assign', false);
rw = Pw.world.rows(1:Pw.world.nRows, :);
[~, o] = sort(rw(:, 1));  rw = rw(o, :);
rw = rw(1:5, 1:4);
y0 = min(rw(1, [2 4]));
ms = struct('mission_id', 501, 'mission_type', 'PALM_ROW_PATROL', 'world_id', 3, 'rows_enu', rw, ...
    'home_enu', [rw(1, 1), y0 - 4, pi/2], 'swath_m', 1, 'turn_radius_m', 2, 'skip_rows', 0, ...
    'speed_work', 1.0, 'speed_transit', 1.0, 'speed_turn_max', 0.8, 'transit_radius_m', 1, ...
    'implement', struct('type', 'NONE'), 'allow_bypass', true, 'lane_halfwidth_m', 2.5, 'end_action', 'STOP');
if strcmp(var, 'b'), ms.allow_bypass = false; end   % variant b: the person hold lasts until the person has left
t_start = 15;
[P, plan, info, mpc, tab] = scn_auto_setup(P, ms, 'World', 3, 'Implement', 0, 't_start', t_start);
assert(plan.valid, 'scn_S5: mission plan invalid');

%% ---- static box on aisle 2 (appended after compiling) ----
lane = local_lane_rows(plan, rw);
a2 = lane(2);                                 % aisle 2 as driven: start point, unit direction
pb = a2.p0 + 40*a2.u;
box = [pb(1) pb(2) 0 0.6 0.6 0.6 0 5 99];
O = P.world.obstacles;
if O.nBox == 0, O.box = box; else, O.box = [O.box(1:O.nBox, :); box]; end
O.nBox = O.nBox + 1;
P.world.obstacles = O;
P.world.occ = world_occupancy(P.world);

%% ---- pedestrian crossing aisle 4 ----
a4 = lane(4);
s_cross = 44;                                 % along aisle 4 from its start
pc = a4.p0 + s_cross*a4.u;
t_nom = local_nominal_time(plan, info);       % [segment start times], nominal
t_meet = t_start + 0.2 + local_time_at(plan, a4.seg, s_cross, t_nom) + 8;   % + bypass / hold allowance
nrm = [-a4.u(2) a4.u(1)];
vp = 0.8;  d_out = 4;  t_wait = 8;
t_in0 = t_meet - d_out/vp - t_wait/2;          % enters at the tree line
t_mid0 = t_in0 + d_out/vp;  t_mid1 = t_mid0 + t_wait;  t_out = t_mid1 + d_out/vp;
tt = (t_in0:0.1:t_out)';
pos = zeros(numel(tt), 2);  vel = zeros(numel(tt), 2);
for k = 1:numel(tt)
    if tt(k) < t_mid0
        s = -d_out + vp*(tt(k) - t_in0);  v = vp;
    elseif tt(k) < t_mid1
        s = 0; v = 0;
    else
        s = vp*(tt(k) - t_mid1);  v = vp;
    end
    pos(k, :) = pc + s*nrm;  vel(k, :) = v*nrm;
end
far = P.sim.mon.far;
tab.obs.n = [0 0; t_in0 1; t_out 0];
tab.obs.r = [0 0.3 0.3 0.3 0.3];
tab.obs.xy = [0, far*ones(1, 8); tt, pos(:, 1), far*ones(numel(tt), 3), pos(:, 2), far*ones(numel(tt), 3); ...
              t_out + 0.1, far*ones(1, 8)];
tab.obs.v_xy = [0, zeros(1, 8); tt, vel(:, 1), zeros(numel(tt), 3), vel(:, 2), zeros(numel(tt), 3); ...
                t_out + 0.1, zeros(1, 8)];

T_route_nom = t_nom(end);
T = ip.Results.StopTime;
if isempty(T), T = ceil(t_start + 1.25*T_route_nom + 30); end
VC = struct('VC_PHASE', 2, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
if strcmp(var, 'b')                          % critfix variant b (header): own person, blackouts and criteria
    [tab, T, crit, info.lidar] = local_variant_b(tab, lane(1), far, ip.Results.StopTime);
    in = scn_base_input(P, 'S5b', 'StopTime', T, 'Input', tab, 'VC', VC);
    if ~isempty(mpc), in = in.setVariable('mpcRowObj', mpc); end
    in = scn_logging('apply', in, P, T);
    info.box = box;  info.T_route_nom = T_route_nom;
    return
end
in = scn_base_input(P, 'S5', 'StopTime', T, 'Input', tab, 'VC', VC);
if ~isempty(mpc), in = in.setVariable('mpcRowObj', mpc); end
in = scn_logging('apply', in, P, T);
info.box = box;  info.ped = struct('t_in', t_in0, 't_out', t_out, 'pc', pc);  info.T_route_nom = T_route_nom;

crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S5.no_coll', 'collisions', 'kpi.n_coll', @(S) double(S.kpi.n_coll.v(end)), '==', 0, '');
crit(end+1) = local_c('S5.clear_box', 'minimum clearance to the box', 'mon_sep_box', ...
    @(S) min(double(S.mon_sep_box.v)), '>=', 0.5, 'm');
crit(end+1) = local_c('S5.clear_person', 'minimum clearance to the pedestrian', 'mon_sep_dyn', ...
    @(S) min(double(S.mon_sep_dyn.v)), '>=', 1.0, 'm');
crit(end+1) = local_c('S5.decel', 'deceleration start after the box enters the 10 m corridor', 'truth, telem.v_target', ...
    @(S) local_decel_delay(S, pb, P.gnc.obs.w_c + 0.3), '<=', 0.5, 's');
crit(end+1) = local_c('S5.rejoin', 'distance after passing the box until |XTE| < 0.1 m', 'truth.pos_W', ...
    @(S) local_rejoin(S, a2, pb), '<=', 3, 'm');
crit(end+1) = local_c('S5.route_time', sprintf('route time / nominal (%.0f s)', T_route_nom), 'gnc_status.mission_done', ...
    @(S) (scn_metrics('first', S.gnc_status.mission_done, @(v) v ~= 0, t_start) - t_start)/T_route_nom, ...
    '<=', 1.25, '1');
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function lane = local_lane_rows(plan, rw)
% ROW segments of the plan in driving order: start point, unit direction, segment index
n = double(plan.n_seg);  nr = size(plan.path, 1)/size(plan.seg, 1);
lane = struct('p0', {}, 'u', {}, 'seg', {});
for s = 1:n
    if double(plan.seg(s, 1)) ~= 2, continue, end
    rows = (s - 1)*nr + (1:nr);
    p = double(plan.path(rows, 1:2));
    u = p(end, :) - p(1, :);  u = u/norm(u);
    lane(end+1) = struct('p0', p(1, :), 'u', u, 'seg', s); %#ok<AGROW>
end
assert(numel(lane) >= size(rw, 1), 'scn_S5: expected %d ROW segments', size(rw, 1));
end

function tn = local_nominal_time(plan, info)
% nominal start time of every segment (constant speed v_seg_max per segment, plus the compiler's estimate)
n = double(plan.n_seg);
L = double(plan.seg(1:n, 3));  v = max(double(plan.seg(1:n, 4)), 0.1);
tn = [0; cumsum(L./v)];
if isstruct(info) && isfield(info, 't_est_s') && info.t_est_s > tn(end)
    tn = tn*info.t_est_s/tn(end);            % scale to the compiler's acceleration-aware estimate
end
end

function t = local_time_at(plan, seg, s, tn)
v = max(double(plan.seg(seg, 4)), 0.1);
t = tn(seg) + s/v;
end

function d = local_decel_delay(S, pb, dyc)
% first time the box centre is ahead (0 < dx <= 10 m) and within the corridor |dy| <= dyc (obs.w_c + 0.3 m) of the
% truth pose; deceleration start = first time after that the MCU target speed drops 0.05 m/s
ps = S.truth.pose2D;
c = cos(ps.v(:, 3));  s = sin(ps.v(:, 3));
dx = c.*(pb(1) - ps.v(:, 1)) + s.*(pb(2) - ps.v(:, 2));
dy = -s.*(pb(1) - ps.v(:, 1)) + c.*(pb(2) - ps.v(:, 2));
k = find(dx > 0 & dx <= 10 & abs(dy) <= dyc, 1, 'first');
if isempty(k), d = NaN; return, end
t_in = ps.t(k);
v0 = double(scn_metrics('at', S.telem.v_target, t_in));
t_dec = scn_metrics('first', S.telem.v_target, @(v) double(v) < v0 - 0.05, t_in);
d = t_dec - t_in;
end

function d = local_rejoin(S, a, pb)
% after the robot is 1.0 m past the box along aisle 2, path length until |XTE| < 0.1 m
ps = S.truth.pose2D;
rel = ps.v(:, 1:2) - a.p0;
along = rel*a.u(:);
xte = rel*[-a.u(2); a.u(1)];
sb = (pb - a.p0)*a.u(:);
k0 = find(along > sb + 1.0 & abs(xte) < 3, 1, 'first');
if isempty(k0), d = NaN; return, end
k1 = find(abs(xte(k0:end)) < 0.1, 1, 'first');
if isempty(k1), d = Inf; return, end
d = along(k0 + k1 - 1) - along(k0);
end

% -------------------------------------------------------------------------------------------
% critfix variant b (header): the lost LiDAR scan in AUTO on aisle 1
function [tab, T, crit, lid] = local_variant_b(tab, a1, far, Tuser)
lid = struct('tb', [35 38; 74 82], 's_p', 34, 't_p', [45 80]);   % blackouts [on off]; person along aisle 1, [in out]
pc = a1.p0 + lid.s_p*a1.u;                   % on the aisle-1 centre line
lid.pc = pc;
tb = lid.tb;  tp = lid.t_p;
tab.flt.lidar_blackout = [0 0; tb(1, 1) 1; tb(1, 2) 0; tb(2, 1) 1; tb(2, 2) 0];
tab.obs.n = [0 0; tp(1) 1; tp(2) 0];         % replaces the aisle-4 pedestrian of the nominal S5
tab.obs.r = [0 0.3 0.3 0.3 0.3];
tab.obs.xy = [0, far*ones(1, 8); tp(1), pc(1), far*ones(1, 3), pc(2), far*ones(1, 3); tp(2), far*ones(1, 8)];
tab.obs.v_xy = [0, zeros(1, 8)];
T = Tuser;  if isempty(T), T = 95; end
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S5b.row_pre', 'precondition: AUTO_MISSION on a ROW when the first blackout starts', ...
    'fsm.state, fsm.sub', @(S) local_on_row(S, tb(1, 1)), 'true', 1, '');
tags = {'row', 'hold'};
for j = 1:2
    if j == 2
        crit(end+1) = local_c('S5b.hold_pre', 'precondition: an OBSTACLE / PERSON / VFH_BLOCKED hold when the second blackout starts', ...
            'fsm.state, fsm.hold_reason', @(S) local_hold_at(S, tb(2, 1)), 'true', 1, ''); %#ok<AGROW>
    end
    b = tb(j, :);
    crit(end+1) = local_c(sprintf('S5b.%s_lost', tags{j}), 'SENSOR_LOST hold after the blackout starts', ...
        'fsm.state, fsm.hold_reason', @(S) local_tlost(S, b(1)) - b(1), '<=', 0.4, 's'); %#ok<AGROW>
    crit(end+1) = local_c(sprintf('S5b.%s_held', tags{j}), ['in the hold, SENSOR_LOST shown, until the scan returns; ' ...
        '|v| <= 0.02 m/s from 2 s after the loss'], 'fsm, truth.speed_fwd', @(S) local_held(S, b), 'true', 1, ''); %#ok<AGROW>
    crit(end+1) = local_c(sprintf('S5b.%s_release', tags{j}), 'end of the hold after the scan returns', ...
        'fsm.state', @(S) local_trel(S, b) - b(2), 'in', [1.8 2.6], 's'); %#ok<AGROW>
    crit(end+1) = local_c(sprintf('S5b.%s_h1', tags{j}), 'the hold ends by H1 (trans_id 31) back to AUTO_MISSION', ...
        'fsm.trans_id, fsm.state', @(S) local_h1(S, b), 'true', 1, ''); %#ok<AGROW>
    crit(end+1) = local_c(sprintf('S5b.%s_impl', tags{j}), 'fsm.impl_allow false from the loss to the end of the hold', ...
        'fsm.impl_allow', @(S) local_impl_off(S, b), 'true', 1, ''); %#ok<AGROW>
end
crit(end+1) = local_c('S5b.no_coll', 'collisions', 'kpi.n_coll', @(S) double(S.kpi.n_coll.v(end)), '==', 0, '');
end

function tf = local_on_row(S, t)
f = S.fsm;
tf = double(scn_metrics('at', f.state, t)) == 2 && double(scn_metrics('at', f.sub, t)) == 3;
end

function tf = local_hold_at(S, t)
f = S.fsm;
tf = double(scn_metrics('at', f.state, t)) == 4 && ismember(double(scn_metrics('at', f.hold_reason, t)), [1 2 3]);
end

function t = local_tlost(S, t0)
% first fsm sample at or after t0 in OBSTACLE_HOLD with hold_reason SENSOR_LOST (10)
f = S.fsm;
k = find(f.state.t >= t0 - 1e-9 & double(f.state.v) == 4 & double(f.hold_reason.v) == 10, 1, 'first');
if isempty(k), t = NaN; else, t = f.state.t(k); end
end

function tf = local_held(S, b)
% from the loss to the scan return: OBSTACLE_HOLD with SENSOR_LOST shown; stopped from the loss + 2 s
f = S.fsm;  tl = local_tlost(S, b(1));
tf = false;
if isnan(tl), return, end
w = f.state.t >= tl - 1e-9 & f.state.t <= b(2) + 1e-9;
tf = all(double(f.state.v(w)) == 4) && all(double(f.hold_reason.v(w)) == 10);
v = S.truth.speed_fwd;
wv = v.t >= tl + 2 & v.t <= b(2);
tf = tf && (~any(wv) || max(abs(double(v.v(wv, 1)))) <= 0.02);
end

function t = local_trel(S, b)
% first fsm sample after the scan return outside OBSTACLE_HOLD (the hold has ended)
f = S.fsm;
k = find(f.state.t > b(2) + 1e-9 & double(f.state.v) ~= 4, 1, 'first');
if isempty(k), t = NaN; else, t = f.state.t(k); end
end

function tf = local_h1(S, b)
f = S.fsm;
k = find(f.state.t > b(2) + 1e-9 & double(f.state.v) ~= 4, 1, 'first');
tf = ~isempty(k) && double(f.trans_id.v(k)) == 31 && double(f.state.v(k)) == 2;
end

function tf = local_impl_off(S, b)
f = S.fsm;  tl = local_tlost(S, b(1));  tr = local_trel(S, b);
tf = false;
if isnan(tl) || isnan(tr), return, end
w = f.state.t >= tl - 1e-9 & f.state.t < tr - 1e-9;
tf = any(w) && ~any(double(f.impl_allow.v(w)) ~= 0);
end
