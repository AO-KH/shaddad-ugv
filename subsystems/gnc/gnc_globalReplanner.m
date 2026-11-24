function rp = gnc_globalReplanner(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm) %#codegen
%GNC_GLOBALREPLANNER  Hybrid A* replanning (MATLAB Function in a Triggered Subsystem, 1 s polling).
%
%   rp = gnc_globalReplanner(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm)
%     goal       [x y psi] (fsm.replan_goal)          kind  0 LOCAL (rejoin / bypass), 1 GLOBAL (RTH)
%     seg        current mission segment (lane mask of LOCAL plans)
%     pose       start [x y psi] (Bus_NavCond.pose)
%     map        Bus_TebMapInfo   robot-centred perception grid (merged with the static grid)
%     plan       Bus_MissionPlan  lane geometry (path rows of seg, lane_halfwidth = seg column 8)
%     occ        P.world.occ.grid static occupancy (row 1 = max y), occRes cells/m, occOrigin lower-left [x y]
%     terr       struct xE, yN, dHdE, dHdN of P.world.terrain (Table(i, j) <-> (xE(i), yN(j))): on cells steeper
%                than prm.slope_turn_max only headings along the fall line are valid (gnc_slopeValidator: no turns,
%                no Dubins bulbs, no driving across such slopes, D53)
%     prm        P.gnc.hastar (non-tunable)
%     rp         Bus_ReplanResult ok, busy, ver (+1 per success), n, path [100 x 4] (x, y, theta deg, dir),
%                fail_count (+1 per failure), t_plan (wall time of the call, s)
%
%   Why a MATLAB Function (docs/specs/gnc.md 4.4, F2): no global planner block exists in navlib,
%   robotmobilelib or uavlib. plannerHybridAStar, binaryOccupancyMap, validatorOccupancyMap and
%   stateSpaceSE2 support code generation (the ert / companion build path below).
%
%   Simulation: the code-generation path of plannerHybridAStar needs a C compiler (BLAS build info; none is
%   installed, D31), so in simulation (coder.target 'Sfun') the same file is called extrinsically and the
%   planner runs interpreted. The planning code is identical in both paths.
%
%   Planning: occupancy = static grid window (GLOBAL: bounding box of start / goal + win_margin; LOCAL: the
%   perception window) OR the perception grid; LOCAL plans add a lane mask (cells farther than
%   lane_halfwidth + r_infl from the lane polyline are occupied, so the detour keeps the robot centre in the
%   lane, T7); inflate(r_infl) (r_infl_min when start or goal would be inside); forward-only Hybrid A*
%   (R_min / mpl GLOBAL, R_min_local / mpl_local LOCAL; ReverseCost, DirectionSwitchingCost,
%   AnalyticExpansionInterval, InterpolationDistance, MaxNumNodes from prm); the path is resampled to 100
%   poses of equal arc length.

persistent ver nFail
if isempty(ver)
    ver = uint16(0);
    nFail = uint8(0);
end
if coder.target('Sfun')
    r = local_sim(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm);
else
    r = local_plan(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm);
end
if r.ok
    if ver == intmax('uint16'), ver = uint16(1); else, ver = ver + 1; end
else
    if nFail == intmax('uint8'), nFail = uint8(1); else, nFail = nFail + 1; end
end
rp = struct('ok', r.ok, 'busy', false, 'ver', ver, 'n', uint16(100*double(r.ok)), 'path', r.path, ...
    'fail_count', nFail, 't_plan', r.t_plan);
end

% ---------------------------------------------------------------------------------------------
function r = local_sim(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm)
coder.extrinsic('gnc_globalReplanner');
rr = struct('ok', false, 'busy', false, 'ver', uint16(0), 'n', uint16(0), 'path', zeros(100, 4), ...
    'fail_count', uint8(0), 't_plan', 0);
rr = gnc_globalReplanner(goal, kind, seg, pose, map, plan, occ, occRes, occOrigin, terr, prm);
r = struct('ok', rr.ok, 'path', rr.path, 't_plan', rr.t_plan);
end

% ---------------------------------------------------------------------------------------------
function r = local_plan(goal, kind, seg, pose, map, mplan, occ, occRes, occOrigin, terr, prm)
% (the mission plan is called mplan here: the name plan is the plannerHybridAStar method)
t0 = tic;
N = 100;
r = struct('ok', false, 'path', zeros(N, 4), 't_plan', 0);
res = double(occRes);
st = double(pose(:)');
g = double(goal(:)');
isLocal = double(kind) == prm.kind_local;
% ---- window [lo, hi] on the static grid (cell edges) ----
if isLocal && map.Resolution > 0
    lo = double(map.GridLocationInWorld(:)');
    hi = lo + [double(map.GridSize(2)) double(map.GridSize(1))]/double(map.Resolution);
    lo = min(lo, min(st(1:2), g(1:2)) - 2);
    hi = max(hi, max(st(1:2), g(1:2)) + 2);
else
    lo = min(st(1:2), g(1:2)) - prm.win_margin;
    hi = max(st(1:2), g(1:2)) + prm.win_margin;
end
org = double(occOrigin(:)');
lo = org + floor((lo - org)*res)/res;
hi = org + ceil((hi - org)*res)/res;
nc = round((hi(1) - lo(1))*res);
nr = round((hi(2) - lo(2))*res);
W = false(nr, nc);
% static grid: window cell (i, j) = world cell (i + dr, j + dc)
NRw = size(occ, 1);  NCw = size(occ, 2);
dc = round((lo(1) - org(1))*res);
dr = NRw - round((hi(2) - org(2))*res);
i1 = max(1, 1 - dr);  i2 = min(nr, NRw - dr);
j1 = max(1, 1 - dc);  j2 = min(nc, NCw - dc);
if i2 >= i1 && j2 >= j1
    W(i1:i2, j1:j2) = occ((i1:i2) + dr, (j1:j2) + dc);
end
% perception grid (same resolution, 0.1 m aligned)
if map.Resolution > 0 && abs(double(map.Resolution) - res) < 1e-9
    gl = double(map.GridLocationInWorld(:)');
    mR = double(map.GridSize(1));  mC = double(map.GridSize(2));
    mdc = round((gl(1) - lo(1))*res);                 % window col = map col + mdc
    mdr = round((hi(2) - (gl(2) + mR/res))*res);      % window row = map row + mdr
    a1 = max(1, 1 - mdr);  a2 = min(mR, nr - mdr);
    b1 = max(1, 1 - mdc);  b2 = min(mC, nc - mdc);
    if a2 >= a1 && b2 >= b1
        W((a1:a2) + mdr, (b1:b2) + mdc) = W((a1:a2) + mdr, (b1:b2) + mdc) | map.OccupancyMatrix(a1:a2, b1:b2);
    end
end
% lane mask (LOCAL): keep the robot centre within lane_halfwidth of the current lane
k = min(max(1, double(seg)), size(mplan.seg, 1));
half = double(mplan.seg(k, 8));
if isLocal && mplan.valid && half > 0
    rows = (k - 1)*prm.N_raw + (1:prm.N_raw);
    px = double(mplan.path(rows, 1));  py = double(mplan.path(rows, 2));
    xc = lo(1) + ((1:nc) - 0.5)/res;
    yc = hi(2) - ((1:nr)' - 0.5)/res;
    X = repmat(xc, nr, 1);  Y = repmat(yc, 1, nc);
    D = inf(nr, nc);
    for q = 1:prm.N_raw - 1
        ux = px(q + 1) - px(q);  uy = py(q + 1) - py(q);
        L2 = ux*ux + uy*uy;
        if L2 < 1e-12, continue; end
        t = min(max(((X - px(q))*ux + (Y - py(q))*uy)/L2, 0), 1);
        D = min(D, hypot(X - px(q) - t*ux, Y - py(q) - t*uy));
    end
    W = W | (D > half + prm.r_infl);
end
% ---- inflate, start / goal validity ----
bmap = binaryOccupancyMap(W, res);
bmap.GridLocationInWorld = lo;
inflate(bmap, prm.r_infl);
if local_blocked(bmap, st, g)
    bmap = binaryOccupancyMap(W, res);
    bmap.GridLocationInWorld = lo;
    inflate(bmap, prm.r_infl_min);
    if local_blocked(bmap, st, g)
        r.t_plan = toc(t0);
        return
    end
end
ss = stateSpaceSE2;
ss.StateBounds = [bmap.XWorldLimits; bmap.YWorldLimits; [-pi pi]];
if isLocal
    R = prm.R_min_local;  mpl = prm.mpl_local;
else
    R = prm.R_min;  mpl = prm.mpl;
end
% terrain steeper than slope_turn_max: states there must head along the fall line (no turns, no driving across
% the slope; D53), checked by gnc_slopeValidator for the motion primitives and the analytic expansions
sv = gnc_slopeValidator(ss, bmap, terr, prm, st);
sv.ValidationDistance = prm.vdist;
if ~all(isStateValid(sv, [st; g]))
    r.t_plan = toc(t0);                       % goal heading across a steep slope: no plan
    return
end
pl = plannerHybridAStar(sv, 'MinTurningRadius', R, 'MotionPrimitiveLength', mpl, ...
    'NumMotionPrimitives', prm.n_prim, 'ReverseCost', prm.cost_rev, 'DirectionSwitchingCost', prm.cost_switch, ...
    'AnalyticExpansionInterval', prm.aei, 'InterpolationDistance', prm.interp, 'MaxNumNodes', prm.max_nodes, ...
    'MotionDirection', 'forward');
ref = plan(pl, st, g);
S = ref.States;
if size(S, 1) < 2
    r.t_plan = toc(t0);
    return
end
% ---- resample to N poses of equal arc length ----
ds = hypot(diff(S(:, 1)), diff(S(:, 2)));
keep = [true; ds > 1e-6];
S = S(keep, :);
s = [0; cumsum(hypot(diff(S(:, 1)), diff(S(:, 2))))];
if numel(s) < 2 || s(end) < 0.05
    r.t_plan = toc(t0);
    return
end
sq = linspace(0, s(end), N)';
th = unwrap(S(:, 3));
r.path = [interp1(s, S(:, 1), sq), interp1(s, S(:, 2), sq), ...
    rad2deg(local_wrap(interp1(s, th, sq))), ones(N, 1)];
r.ok = true;
r.t_plan = toc(t0);
end

function b = local_blocked(bmap, st, g)
xl = bmap.XWorldLimits;  yl = bmap.YWorldLimits;
inS = st(1) > xl(1) && st(1) < xl(2) && st(2) > yl(1) && st(2) < yl(2);
inG = g(1) > xl(1) && g(1) < xl(2) && g(2) > yl(1) && g(2) < yl(2);
b = ~inS || ~inG || checkOccupancy(bmap, st(1:2)) > 0 || checkOccupancy(bmap, g(1:2)) > 0;
end

function a = local_wrap(a)
a = a - 2*pi*floor((a + pi)/(2*pi));
end
