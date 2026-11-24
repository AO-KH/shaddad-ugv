function [plan, info] = gnc_compileMission(P, src, varargin)
%GNC_COMPILEMISSION  Offline mission compiler: JSON field map -> Bus_MissionPlan struct (owner: gnc).
%
%   plan = gnc_compileMission(P, jsonFile)        jsonFile: name in scripts/missions (with or without
%                                                 .json) or a full path
%   plan = gnc_compileMission(P, S)               S: decoded mission struct (same fields as the JSON)
%   [plan, info] = gnc_compileMission(...)        info: lanes, lengths, analytic check, clearances,
%                                                 time and coverage estimates, compile time, warnings
%   ... = gnc_compileMission(..., 'World', W)     use world struct W instead of P.world / world_id
%   ... = gnc_compileMission(..., 'Validate', false)   skip the clearance / coverage raster checks
%
%   The caller assigns the result: P.gnc.mission = plan (read by the Constant MissionSource inside
%   Shaddad_GNC, D57). No base-workspace access. Datum = P.world.lla0 (D10); geometry comes from the
%   JSON (LLA, converted with lla2enu 'flat') and from P.world (walls, posts, palms, piers, terrain).
%
%   Pipeline (docs/specs/gnc.md 2.7, revision B):
%   1  work area / rows / keep-outs / poses LLA -> ENU at P.world.lla0.
%   2  coverage missions (GREENHOUSE_COVERAGE): the work area (inner wall faces) is inset by
%      c_in = gh_post_edge + gh_c_clear + gh_c_xte + max(0, W/2 - w_tool/2) (= 0.45 m, D54);
%      uavCoverageSpace(UnitWidth = swath, Sidelap) + setCoveragePattern(SweepAngle) +
%      uavCoveragePlanner(Solver "MinTraversal") give the lane count and direction; the lanes are then
%      spread evenly so that the outer tool edges lie on the inset line (the planner places the last
%      lane at UnitWidth*(1-sidelap)/2 from one edge only, which would put the body into the wall
%      posts). Lane ends are trimmed so the swept footprint (body + implement) of a headland U-turn
%      keeps cov.wall_clear to the end walls. Explicit rows (rows_lla, palm / solar aisles) are used
%      as given (no trimming: turns are made beyond the row ends).
%   3  ordering: lane spacing s < 2 R -> skip-row order 1,3,5,..., 2,4,6,... (every switch-back spans
%      >= 2 lanes; unlike 1,3,5,...,6,4,2 it needs no adjacent RLR bulb, which in a 9.6 m house would
%      swing the body into the side-wall posts); s >= 2 R -> sequential.
%   4  headland turns and transits: dubinsConnection(MinTurningRadius = R); no turn where the DEM
%      slope exceeds cov.slope_turn_max (D53): the two lane ends are shortened by cov.slope_step and
%      the turn is re-planned (up to 10 times, then error). Transits: home -> via_in ... -> first lane,
%      last lane -> via_out ... -> dock-approach start, straight dock approach (approach_m), terminal
%      extension (cov.terminal_len, window B only, never driven).
%   5  every segment is resampled to exactly N_raw = 100 poses (equal arc length); segment table
%      [type dir length v_seg_max v_end impl_mode allow_bypass lane_halfwidth lane_id pivot_allowed];
%      v_seg_max = min(segment speed, sqrt(a_lat / kappa_max)), v_end = min(v_seg(i), v_seg(i+1)).
%   6  validation: <= 512 segments, all turn radii >= R, footprint clearance to P.world obstacles
%      (cylinders, boxes, film) >= cov.clear_min, keep-outs >= W/2 + cov.clear_keepout, every pose
%      inside the geofence (site boundary grown by fsm.geofence_margin), analytic length check
%      (rows + closed-form U-turns pi R + d - 2R), time estimate (speed profile with a_acc / a_dec),
%      coverage raster (cov.raster) of the implement footprint over the inset area.
%
%   JSON fields (scripts/missions/*.json): mission_id, mission_type (PALM_ROW_PATROL |
%   GREENHOUSE_COVERAGE | SOLAR_ROW_MOWING | GOTO), world_id, datum {lat, lon, alt}, work_area_lla
%   [[lat lon]...], rows_lla [{p0 [lat lon], p1 [lat lon]}], keep_out [{type circle, lat, lon, r} |
%   {type polygon, lla [[lat lon]...]}], geofence_lla (optional, default P.world.boundary),
%   sweep_angle_deg, swath_m, sidelap, inset {post_edge_m, c_clear_m, c_xte_m}, turn_radius_m,
%   skip_rows (auto | 0 | 1), transit_radius_m (default turn_radius_m), speed_work, speed_transit, speed_turn_max, implement {type, speed_rpm,
%   height_m}, allow_bypass, lane_halfwidth_m, home {lat, lon, yaw_enu_deg}, dock {lat, lon,
%   yaw_enu_deg, approach_m} (optional), via_in_lla / via_out_lla [{lat, lon, yaw_enu_deg}],
%   end_action (STOP | RTH). yaw_enu_deg = ENU yaw, CCW from East (ICD 1), not a compass heading.
%   Test missions may give rows_enu [x1 y1 x2 y2] and home_enu / dock_enu [x y psi] directly.

t0 = tic;
ip = inputParser;
ip.addParameter('World', []);
ip.addParameter('Validate', true);
ip.parse(varargin{:});
opt = ip.Results;

S = local_load(src);
G = P.gnc;
warn = {};

%% ---- world and datum ----
if ~isempty(opt.World)
    W = opt.World;
elseif isfield(S, 'world_id') && S.world_id ~= P.world.id
    W = param_world(S.world_id, P.frames.g);
else
    W = P.world;
end
lla0 = W.lla0;
if isfield(S, 'datum')
    d = [S.datum.lat S.datum.lon S.datum.alt];
    if max(abs(d(1:2) - lla0(1:2))) > 1e-9 || abs(d(3) - lla0(3)) > 1e-6
        warn{end+1} = 'JSON datum differs from P.world.lla0; P.world.lla0 used (D10)';
    end
end
toENU = @(ll) local_lla2enu(ll, lla0);

%% ---- mission type, implement, speeds ----
mtype = local_enum('MissionType', local_get(S, 'mission_type', 'GOTO'));
impl = local_get(S, 'implement', struct('type', 'NONE'));
itype = local_enum('ImplementType', local_get(impl, 'type', 'NONE'));
R = local_get(S, 'turn_radius_m', G.cov.gh_R);
vmax = G.spd.v_max;
vWork = local_clip(local_get(S, 'speed_work', G.spd.v_work_greenhouse), vmax, 'speed_work', warn);
vTran = local_clip(local_get(S, 'speed_transit', G.spd.v_transit), vmax, 'speed_transit', warn);
vTurn = min(local_get(S, 'speed_turn_max', G.spd.v_turn_max), vmax);
swath = local_get(S, 'swath_m', G.impl.mower_w);
sidelap = local_get(S, 'sidelap', G.cov.gh_sidelap);
allowBypass = logical(local_get(S, 'allow_bypass', false));
laneHalf = local_get(S, 'lane_halfwidth_m', 0.5);
endAction = double(strcmpi(local_get(S, 'end_action', 'STOP'), 'RTH'));
foot = local_footprint(P, itype);

%% ---- poses ----
home = local_pose(S, 'home', toENU, W.start);
hasDock = isfield(S, 'dock') || isfield(S, 'dock_enu');
dock = [];  approach = G.cov.dock_approach;
if hasDock
    dock = local_pose(S, 'dock', toENU, W.dock.pose);
    if isfield(S, 'dock') && isfield(S.dock, 'approach_m'), approach = S.dock.approach_m; end
end
viaIn  = local_vias(S, 'via_in_lla', toENU);
viaOut = local_vias(S, 'via_out_lla', toENU);
Rtr = local_get(S, 'transit_radius_m', R);   % transit / dock-leg turning radius (low speed)

%% ---- keep-outs ----
keep = struct('type', {}, 'c', {}, 'r', {}, 'poly', {});
if isfield(S, 'keep_out') && ~isempty(S.keep_out)
    ko = S.keep_out; if ~iscell(ko), ko = num2cell(ko); end
    for k = 1:numel(ko)
        if strcmpi(ko{k}.type, 'circle')
            e = toENU([ko{k}.lat ko{k}.lon]);
            keep(end+1) = struct('type', 'circle', 'c', e, 'r', ko{k}.r, 'poly', []); %#ok<AGROW>
        else
            keep(end+1) = struct('type', 'polygon', 'c', [], 'r', 0, 'poly', toENU(ko{k}.lla)); %#ok<AGROW>
        end
    end
end

%% ---- lanes ----
lanes = struct('p0', {}, 'p1', {}, 'n', {});
cin = NaN; insetPoly = []; areaPoly = []; covArea = 0;
cpInfo = struct('n_planner', NaN, 't_plan', 0);
if isfield(S, 'work_area_lla') && ~isempty(S.work_area_lla)
    areaPoly = toENU(S.work_area_lla);
    ins = local_get(S, 'inset', struct());
    cin = local_get(ins, 'post_edge_m', G.cov.gh_post_edge) + local_get(ins, 'c_clear_m', G.cov.gh_c_clear) + ...
          local_get(ins, 'c_xte_m', G.cov.gh_c_xte) + max(0, G.geom.W/2 - swath/2);
    ps = polybuffer(polyshape(areaPoly), -cin, 'JointType', 'miter');
    insetPoly = ps.Vertices;
    covArea = area(ps);
    sweep = local_get(S, 'sweep_angle_deg', 90);
    [lanes, cpInfo] = local_coverage_lanes(areaPoly, insetPoly, swath, sidelap, sweep, R, foot, G, home, dock);
elseif isfield(S, 'rows_lla') || isfield(S, 'rows_enu')
    if isfield(S, 'rows_enu')
        rw = S.rows_enu;
    else
        rr = S.rows_lla; if ~iscell(rr), rr = num2cell(rr); end
        rw = zeros(numel(rr), 4);
        for k = 1:numel(rr)
            rw(k, :) = [toENU(rr{k}.p0(:)') toENU(rr{k}.p1(:)')];
        end
    end
    u = rw(1, 3:4) - rw(1, 1:2);  u = u/norm(u);  nvec = [-u(2) u(1)];
    for k = 1:size(rw, 1)
        lanes(k) = struct('p0', rw(k, 1:2), 'p1', rw(k, 3:4), 'n', dot(rw(k, 1:2), nvec)); %#ok<AGROW>
    end
    [~, o] = sort([lanes.n]);  lanes = lanes(o);
    covArea = sum(arrayfun(@(l) norm(l.p1 - l.p0), lanes))*swath;
end
nL = numel(lanes);
if nL >= 2 && norm(local_mid(lanes(nL)) - home(1:2)) < norm(local_mid(lanes(1)) - home(1:2))
    lanes = lanes(nL:-1:1);                  % lane 1 = the outer lane on the home side
end

%% ---- ordering and directions ----
if nL >= 2
    sL = median(abs(diff([lanes.n])));
else
    sL = Inf;
end
skip = local_get(S, 'skip_rows', 'auto');
if ischar(skip) || isstring(skip), useSkip = sL < 2*R; else, useSkip = skip > 0; end
if useSkip && nL > 2
    order = [1:2:nL, 2:2:nL];
    % the far outer lane nL must not be entered from the neighbouring skip lane (2 lanes): that U-turn
    % has its centre R inside the outer lane and swings the implement corner hypot(x_f, R + w/2) past
    % the wall line. Swap it with its predecessor (..., nL-4, nL, nL-2, ...) when no adjacent switch results.
    j = find(order == nL);
    if j > 2 && abs(order(j-1) - nL) < 3 && abs(order(j-2) - nL) >= 3
        order([j-1 j]) = order([j j-1]);
    end
else
    order = 1:nL;
end
if nL > 0
    % common orientation first (explicit rows may be listed in either direction)
    u1 = lanes(1).p1 - lanes(1).p0;
    for k = 2:nL
        if dot(lanes(k).p1 - lanes(k).p0, u1) < 0, lanes(k) = local_flip(lanes(k)); end
    end
    % lane 'order(1)' starts at the end nearer to home; directions alternate
    l1 = lanes(order(1));
    if norm(l1.p1 - home(1:2)) < norm(l1.p0 - home(1:2))
        for k = 1:nL, lanes(k) = local_flip(lanes(k)); end
    end
    for k = 2:2:nL
        lanes(order(k)) = local_flip(lanes(order(k)));
    end
end

%% ---- segments (fine geometry) ----
dsF = 0.01;                                  % fine geometry step [m]
segs = struct('type', {}, 'xy', {}, 'th', {}, 'lane', {}, 'v', {}, 'impl', {}, 'bypass', {}, ...
    'half', {}, 'pivot', {}, 'd_lat', {});
ST = struct('TRANSIT', 1, 'ROW', 2, 'TURN', 3, 'DOCK', 4, 'TERMINAL', 5);
implTurn = double(itype == 1);               % mower blades run on ROW and TURN (spec 2.9)
% lane entries: with world validation on, a forward entry whose swept footprint (body + implement)
% comes closer than cov.clear_min to a wall / post ends at the lane start shifted inward and joins
% the lane line with an S-curve lead-in (two arcs cov.entry_Rs) that becomes the start of the ROW
entry = cell(1, nL);
leadIn = zeros(0, 2);                        % [lane shift_m] of every lead-in
fixEntry = opt.Validate;
noKeep = struct('type', {}, 'c', {}, 'r', {}, 'poly', {});
clrFun = @(g) local_geom_clear(g, foot, W, noKeep, G);
if nL > 0
    first = local_lanepose(lanes(order(1)), 'start');
    wps = [home; viaIn];
    segs = local_add_transit(segs, wps, Rtr, dsF, vTran, ST);
    if norm(first(1:2) - wps(end, 1:2)) >= G.cov.via_tol
        legFun = @(pt) local_dubins(wps(end, :), pt, Rtr, dsF);
        [gL, entry{order(1)}, dIn] = local_entry(legFun, lanes(order(1)), lanes, clrFun, fixEntry, false, W, G, dsF);
        segs(end+1) = local_seg(ST.TRANSIT, gL, 0, vTran, 0, false, 0.5, 1, 0);
        if dIn > 0, leadIn(end+1, :) = [order(1) dIn]; end %#ok<AGROW>
    end
    k = 1;
    while k <= nL
        Lk = lanes(order(k));
        segs(end+1) = local_seg(ST.ROW, local_rowgeom(Lk, entry{order(k)}, dsF), order(k), vWork, 1, allowBypass, laneHalf, 1, 0); %#ok<AGROW>
        if k < nL
            Ln = lanes(order(k+1));
            [tseg, Lk2, Ln2, ok] = local_turn(Lk, Ln, R, dsF, W, G.cov);
            if ~ok
                error('gnc_compileMission:slope', 'No headland turn below slope_turn_max between lanes %d and %d', order(k), order(k+1));
            end
            if ~isequal(Lk2, Lk)                % lane ends shortened for a slope-limited turn
                lanes(order(k)) = Lk2; lanes(order(k+1)) = Ln2;
                segs(end) = local_seg(ST.ROW, local_rowgeom(Lk2, entry{order(k)}, dsF), order(k), vWork, 1, allowBypass, laneHalf, 1, 0);
                warn{end+1} = sprintf('lanes %d/%d shortened for the slope limit (D53)', order(k), order(k+1)); %#ok<AGROW>
            end
            if fixEntry && clrFun(tseg) < G.cov.clear_min
                legFun = @(pt) local_dubins(local_lanepose(Lk2, 'end'), pt, R, dsF);
                [tseg, entry{order(k+1)}, dIn] = local_entry(legFun, Ln2, lanes, clrFun, true, true, W, G, dsF);
                if dIn > 0, leadIn(end+1, :) = [order(k+1) dIn]; end %#ok<AGROW>
            end
            segs(end+1) = local_seg(ST.TURN, tseg, 0, vTurn, implTurn, false, laneHalf, 0, abs(Ln.n - Lk.n)); %#ok<AGROW>
        end
        k = k + 1;
    end
    last = local_lanepose(lanes(order(nL)), 'end');
else
    last = home;
end
if isfield(S, 'segments')                      % explicit test segments (GOTO / bench missions)
    sg = S.segments; if ~iscell(sg), sg = num2cell(sg); end
    for k = 1:numel(sg)
        q = sg{k};
        g = local_poly_geom(q.xy, dsF);
        v = local_get(q, 'v', vTran);
        typ = ST.(upper(q.type));
        segs(end+1) = local_seg(typ, g, local_get(q, 'lane', 0), v, local_get(q, 'impl', 0), ...
            logical(local_get(q, 'bypass', 0)), laneHalf, double(typ ~= ST.TURN), 0); %#ok<AGROW>
    end
    last = [segs(end).xy(end, :) segs(end).th(end)];
end
if hasDock
    ds0 = dock - approach*[cos(dock(3)) sin(dock(3)) 0];
    wps = [last; viaOut; ds0];
    segs = local_add_transit(segs, wps, Rtr, dsF, vTran, ST);
    segs(end+1) = local_seg(ST.DOCK, local_line(ds0(1:2), dock(1:2), dsF), 0, G.spd.v_dock, 0, false, laneHalf, 1, 0);
    endPose = dock;
else
    endPose = [segs(end).xy(end, :) segs(end).th(end)];
end
te = endPose(1:2) + G.cov.terminal_len*[cos(endPose(3)) sin(endPose(3))];
segs(end+1) = local_seg(ST.TERMINAL, local_line(endPose(1:2), te, dsF), 0, 0, 0, false, laneHalf, 0, 0);
nS = numel(segs);
if nS > G.path.S_max
    error('gnc_compileMission:size', '%d segments exceed S_max = %d', nS, G.path.S_max);
end

%% ---- speeds: v_seg_max (curvature limited), v_end ----
for k = 1:nS
    km = max(abs(local_curv(segs(k).xy, segs(k).th)));
    if km > 1e-6, segs(k).v = min(segs(k).v, sqrt(G.spd.a_lat/km)); end
end
vEnd = zeros(nS, 1);
for k = 1:nS - 1
    vEnd(k) = min(segs(k).v, segs(k+1).v);
end
iDock = find([segs.type] == ST.DOCK, 1);
if ~isempty(iDock), vEnd(iDock) = 0; end
lastDrive = nS - 1;
vEnd(lastDrive) = 0;                         % the robot stops at the end of the last driven segment

%% ---- resample and pack ----
N = G.path.N_raw;
plan = P.ic.Bus_MissionPlan;
pathM = zeros(size(plan.path), 'single');
segM = zeros(size(plan.seg), 'single');
segLen = zeros(nS, 1);
for k = 1:nS
    [xyR, thR, Lk] = local_resample(segs(k).xy, segs(k).th, N);
    segLen(k) = Lk;
    rows = (k-1)*N + (1:N);
    pathM(rows, :) = single([xyR, rad2deg(local_wrap(thR)), ones(N, 1)]);
    segM(k, :) = single([segs(k).type, 1, Lk, segs(k).v, vEnd(k), segs(k).impl, segs(k).bypass, ...
        segs(k).half, segs(k).lane, segs(k).pivot]);
end
geo = local_geofence(S, W, toENU, G.fsm.geofence_margin);
if size(geo, 1) > size(plan.boundary, 1)
    error('gnc_compileMission:geofence', 'geofence has %d vertices (max %d)', size(geo, 1), size(plan.boundary, 1));
end
bnd = zeros(size(plan.boundary));  bnd(1:size(geo, 1), :) = geo;

plan.valid = true;
plan.mission_id = uint16(local_get(S, 'mission_id', 0));
plan.mission_type = uint8(mtype);
plan.implement_type = uint8(itype);
plan.n_seg = uint16(nS);
plan.dock_seg = uint16(local_ifempty(iDock, 0));
plan.path = pathM;
plan.seg = segM;
plan.boundary = bnd;
plan.n_bnd = uint8(size(geo, 1));
plan.home = reshape(home(:), size(plan.home));
if hasDock, plan.dock = reshape(dock(:), size(plan.dock)); end
plan.datum_lla = reshape(lla0(:), size(plan.datum_lla));
plan.swath = single(swath);
plan.area_m2 = single(covArea);
plan.speed_work = single(vWork);
plan.end_action = uint8(endAction);

%% ---- checks and estimates ----
info = struct();
info.n_lanes = nL;
info.lane_n = [lanes.n];
info.lane_spacing = sL;
info.order = order;
info.lead_in = leadIn;
info.c_in = cin;
info.inset_poly = insetPoly;
info.work_poly = areaPoly;
info.n_seg = nS;
info.seg_type = [segs.type];
info.seg_len = segLen;
info.seg_v = [segs.v];
info.v_end = vEnd;
isRow = [segs.type] == ST.ROW;  isTurn = [segs.type] == ST.TURN;
info.len_rows = sum(segLen(isRow));
info.len_turns = sum(segLen(isTurn));
info.len_total = sum(segLen(1:lastDrive));
dLat = [segs(isTurn).d_lat];
anTurn = sum(pi*R + max(0, dLat - 2*R));
anRows = sum(arrayfun(@(l) norm(l.p1 - l.p0), lanes));
info.len_analytic = anRows + anTurn;
info.len_ratio = (info.len_rows + info.len_turns)/max(info.len_analytic, eps);
kAll = [];
for k = find(isTurn)
    kAll = [kAll; local_curv(segs(k).xy, segs(k).th)]; %#ok<AGROW>
end
info.R_min_turns = 1/max([abs(kAll); eps]);
kTr = [];
for k = find([segs.type] == ST.TRANSIT)
    kTr = [kTr; local_curv(segs(k).xy, segs(k).th)]; %#ok<AGROW>
end
info.R_min_transit = 1/max([abs(kTr); eps]);
info.R_req = R;
info.t_est_s = local_time(segs(1:lastDrive), G.spd.a_acc, G.spd.a_dec);
info.cov_planner = cpInfo;
info.warnings = warn;
if opt.Validate
    info.clear = local_clearance(segs(1:lastDrive), foot, W, keep, G);
    info.geofence_ok = local_in_geofence(segs, geo);
    if ~isempty(insetPoly) && any(itype == [1 2])
        info.coverage = local_coverage(segs, insetPoly, P, itype, G.cov.raster, ST);
    else
        info.coverage = NaN;
    end
    if ~isempty(areaPoly)
        info.body_edge_to_wall = local_edge_to_wall(lanes, areaPoly, G.geom.W);
    else
        info.body_edge_to_wall = NaN;
    end
    if info.clear.min < G.cov.clear_min
        warn{end+1} = sprintf('footprint clearance %.3f m < clear_min %.2f m at segment %d', ...
            info.clear.min, G.cov.clear_min, info.clear.iseg);
    end
    if ~info.geofence_ok, warn{end+1} = 'path leaves the geofence'; end
    info.warnings = warn;
end
info.t_compile = toc(t0);
for k = 1:numel(warn), warning('gnc_compileMission:check', '%s', warn{k}); end
end

% =============================================================================================
%  input helpers
% =============================================================================================
function S = local_load(src)
if isstruct(src), S = src; return, end
f = char(src);
if ~isfile(f)
    root = fileparts(fileparts(mfilename('fullpath')));
    f2 = fullfile(root, 'scripts', 'missions', f);
    if ~endsWith(f2, '.json'), f2 = [f2 '.json']; end
    if ~isfile(f2), error('gnc_compileMission:file', 'Mission file %s not found', char(src)); end
    f = f2;
end
S = jsondecode(fileread(f));
end

function v = local_get(S, name, def)
if isstruct(S) && isfield(S, name) && ~isempty(S.(name)), v = S.(name); else, v = def; end
end

function v = local_ifempty(v, def)
if isempty(v), v = def; end
end

function v = local_clip(v, vmax, name, warn) %#ok<INUSD>
if v > vmax
    warning('gnc_compileMission:speed', '%s %.2f m/s clipped to P.gnc.spd.v_max %.2f m/s (D43)', name, v, vmax);
    v = vmax;
end
end

function c = local_enum(cls, s)
if isnumeric(s), c = double(s); return, end
c = double(eval([cls '.' upper(char(s))]));
end

function e = local_lla2enu(ll, lla0)
ll = double(ll);
if size(ll, 2) == 2, ll = [ll lla0(3)*ones(size(ll, 1), 1)]; end
e = lla2enu(ll, lla0, 'flat');
e = e(:, 1:2);
end

function p = local_pose(S, name, toENU, def)
if isfield(S, [name '_enu'])
    p = S.([name '_enu'])(:)';
elseif isfield(S, name)
    q = S.(name);
    p = [toENU([q.lat q.lon]) deg2rad(q.yaw_enu_deg)];
else
    p = def(:)';
end
end

function V = local_vias(S, name, toENU)
V = zeros(0, 3);
en = strrep(name, '_lla', '_enu');
if isfield(S, en) && ~isempty(S.(en)), V = S.(en); return, end
if ~isfield(S, name) || isempty(S.(name)), return, end
q = S.(name); if ~iscell(q), q = num2cell(q); end
for k = 1:numel(q)
    V(end+1, :) = [toENU([q{k}.lat q{k}.lon]) deg2rad(q{k}.yaw_enu_deg)]; %#ok<AGROW>
end
end

function geo = local_geofence(S, W, toENU, margin)
if isfield(S, 'geofence_lla') && ~isempty(S.geofence_lla)
    b = toENU(S.geofence_lla);
else
    b = W.boundary;
end
ps = polybuffer(polyshape(b), margin, 'JointType', 'miter', 'MiterLimit', 3);
geo = ps.Vertices;
end

% =============================================================================================
%  geometry
% =============================================================================================
function F = local_footprint(P, itype)
% footprint rectangles [x0 x1 y0 y1] in FLU (body + implement) for clearance checks
gm = P.structure.geom;
F = [-gm.L_overall/2 gm.L_overall/2 -gm.W_overall/2 gm.W_overall/2];
im = P.structure.impl;
switch itype
    case 1, F = [F; local_rect(im.mower.pos, im.mower.dims)];
    case 2, F = [F; local_rect(im.tiller.pos, im.tiller.dims)];
    case 3, F = [F; local_rect(im.sprayer.pos_dry, im.sprayer.dims_dry)];
end
end

function r = local_rect(pos, dims)
r = [pos(1) - dims(1)/2, pos(1) + dims(1)/2, -dims(2)/2, dims(2)/2];   % pos FRD x = FLU x
end

function [lanes, info] = local_coverage_lanes(areaPoly, insetPoly, swath, sidelap, sweep, R, foot, G, home, dock)
% lane count / direction from uavCoveragePlanner, positions spread over the inset polygon
t0 = tic;
cs = uavCoverageSpace(Polygons = {insetPoly}, UseLocalCoordinates = true, UnitWidth = swath, ...
    Sidelap = sidelap, ReferenceHeight = 0);
setCoveragePattern(cs, 1, SweepAngle = sweep);
cp = uavCoveragePlanner(cs, Solver = "MinTraversal");
goal = home; if ~isempty(dock), goal = dock; end
[wp, soln] = plan(cp, [home(1:2) 0], [goal(1:2) 0]);
if ~soln.IsPathFound, error('gnc_compileMission:coverage', 'uavCoveragePlanner found no path'); end
lw = wp(2:end-1, 1:2);                         % lane end points (pairs)
nPl = floor(size(lw, 1)/2);
u = lw(2, :) - lw(1, :);  u = u/norm(u);       % lane direction
nvec = [-u(2) u(1)];
% inset extents across (n) and along (u) the lanes
nIn = insetPoly*nvec';  uIn = insetPoly*u';
nA  = areaPoly*nvec';   uA  = areaPoly*u';
n1 = min(nIn) + swath/2;  n2 = max(nIn) - swath/2;
nL = max(nPl, 1 + ceil((n2 - n1)/(swath*(1 - sidelap)) - 1e-9));
nPos = linspace(n1, n2, nL);
% headland trim: footprint corners swept around a U-turn centre at lateral offset R
xc = [foot(:, 2); foot(:, 2)];  yc = [foot(:, 3); foot(:, 4)];
aTurn = G.cov.wall_clear + max(hypot(max(xc, 0), R + abs(yc)));
u1 = max(min(uIn), min(uA) + aTurn);
u2 = min(max(uIn), max(uA) - aTurn);
lanes = struct('p0', {}, 'p1', {}, 'n', {});
for k = 1:nL
    lanes(k) = struct('p0', nPos(k)*nvec + u1*u, 'p1', nPos(k)*nvec + u2*u, 'n', nPos(k)); %#ok<AGROW>
end
info = struct('n_planner', nPl, 't_plan', toc(t0), 'a_turn', aTurn, 'spacing', (n2 - n1)/max(nL - 1, 1));
end

function m = local_mid(L)
m = (L.p0 + L.p1)/2;
end

function L = local_flip(L)
L = struct('p0', L.p1, 'p1', L.p0, 'n', L.n);
end

function p = local_lanepose(L, which)
d = L.p1 - L.p0;  th = atan2(d(2), d(1));
if strcmp(which, 'start'), p = [L.p0 th]; else, p = [L.p1 th]; end
end

function g = local_line(a, b, ds)
L = norm(b - a);
n = max(2, ceil(L/ds) + 1);
t = linspace(0, 1, n)';
g.xy = a + t.*(b - a);
g.th = atan2(b(2) - a(2), b(1) - a(1))*ones(n, 1);
end

function g = local_poly_geom(xy, ds)
d = sqrt(sum(diff(xy).^2, 2));  s = [0; cumsum(d)];
n = max(2, ceil(s(end)/ds) + 1);
si = linspace(0, s(end), n)';
g.xy = interp1(s, xy, si);
g.th = unwrap(atan2(gradient(g.xy(:, 2)), gradient(g.xy(:, 1))));
end

function g = local_dubins(p0, p1, R, ds)
dc = dubinsConnection('MinTurningRadius', R);
[pc, ~] = connect(dc, p0, p1);
L = pc{1}.Length;
n = max(2, ceil(L/ds) + 1);
q = interpolate(pc{1}, linspace(0, L, n));
g.xy = q(:, 1:2);
g.th = unwrap(q(:, 3));
g.types = pc{1}.MotionTypes;
end

function s = local_seg(type, g, lane, v, impl, bypass, half, pivot, dlat)
s = struct('type', type, 'xy', g.xy, 'th', g.th, 'lane', lane, 'v', v, 'impl', impl, ...
    'bypass', double(bypass), 'half', half, 'pivot', pivot, 'd_lat', dlat);
end

function segs = local_add_transit(segs, wps, R, ds, v, ST)
% one TRANSIT segment per leg between consecutive (non-coincident) way poses
for k = 1:size(wps, 1) - 1
    if norm(wps(k+1, 1:2) - wps(k, 1:2)) < 0.02, continue, end
    g = local_dubins(wps(k, :), wps(k+1, :), R, ds);
    segs(end+1) = local_seg(ST.TRANSIT, g, 0, v, 0, false, 0.5, 1, 0); %#ok<AGROW>
end
end

function [g, Lk, Ln, ok] = local_turn(Lk, Ln, R, ds, W, cov)
% headland turn lane end -> next lane start; shortened while the slope exceeds cov.slope_turn_max
ok = false;
for it = 1:11
    g = local_dubins(local_lanepose(Lk, 'end'), local_lanepose(Ln, 'start'), R, ds);
    if local_max_slope(g.xy, W) <= cov.slope_turn_max
        ok = true; return
    end
    uk = (Lk.p1 - Lk.p0)/norm(Lk.p1 - Lk.p0);  un = (Ln.p1 - Ln.p0)/norm(Ln.p1 - Ln.p0);
    Lk.p1 = Lk.p1 - cov.slope_step*uk;
    Ln.p0 = Ln.p0 + cov.slope_step*un;
end
end

function [g, gS, dlt] = local_entry(legFun, L, lanes, clrFun, fix, checkSlope, W, G, ds)
% entry leg into lane L (legFun(pose) = geometry ending at pose). When the straight entry keeps less
% than cov.clear_min, the leg ends at the lane start shifted inward (toward the other lanes) by dlt and
% the S-curve gS (two arcs cov.entry_Rs) joins the lane line; the smallest shift that clears is used.
p0 = local_lanepose(L, 'start');
g = legFun(p0); gS = []; dlt = 0;
if ~fix || clrFun(g) >= G.cov.clear_min, return, end
u = [cos(p0(3)) sin(p0(3))];
mids = reshape([lanes.p0] + [lanes.p1], 2, [])'/2;
v = mean(mids, 1) - (L.p0 + L.p1)/2;  v = v - dot(v, u)*u;
if norm(v) < 1e-6, return, end            % single lane: no inward direction
nIn = v/norm(v);
for d = G.cov.entry_dstep:G.cov.entry_dstep:G.cov.entry_dmax
    ps = [p0(1:2) + d*nIn, p0(3)];
    g2 = local_scurve(ps, -d*nIn, G.cov.entry_Rs, ds);
    if norm(g2.xy(end, :) - p0(1:2)) > norm(L.p1 - L.p0) - 1, break, end
    g1 = legFun(ps);
    if checkSlope && local_max_slope(g1.xy, W) > G.cov.slope_turn_max, continue, end
    if min(clrFun(g1), clrFun(g2)) >= G.cov.clear_min
        g = g1; gS = g2; dlt = d; return
    end
end
end

function g = local_scurve(ps, dvec, Rs, ds)
% S-curve from pose ps shifting laterally by dvec (perpendicular to the heading): two arcs of radius Rs
th0 = ps(3);  dl = norm(dvec);
side = sign(dot(dvec, [-sin(th0) cos(th0)]));   % +1: first arc to the left
a = acos(1 - dl/(2*Rs));
n = max(2, ceil(Rs*a/ds) + 1);
s = linspace(0, Rs*a, n)';
k = side/Rs;
t1 = th0 + k*s;
x1 = ps(1) + (sin(t1) - sin(th0))/k;  y1 = ps(2) - (cos(t1) - cos(th0))/k;
th1 = t1(end);  t2 = th1 - k*s;
x2 = x1(end) - (sin(t2) - sin(th1))/k;  y2 = y1(end) + (cos(t2) - cos(th1))/k;
g.xy = [x1 y1; x2(2:end) y2(2:end)];
g.th = [t1; t2(2:end)];
end

function g = local_rowgeom(L, gS, ds)
% ROW geometry: optional S-curve lead-in, then the straight lane to L.p1
if isempty(gS), g = local_line(L.p0, L.p1, ds); return, end
gl = local_line(gS.xy(end, :), L.p1, ds);
g.xy = [gS.xy; gl.xy(2:end, :)];
g.th = [gS.th; gl.th(2:end)];
end

function c = local_geom_clear(g, foot, W, keep, G)
% minimum footprint clearance of one geometry to the P.world obstacles
C = local_clearance(local_seg(1, g, 0, 0, 0, false, 0, 0, 0), foot, W, keep, G);
c = C.min;
end

function sl = local_max_slope(xy, W)
T = W.terrain;
gx = griddedInterpolant({T.xE, T.yN}, T.dHdE, 'linear', 'nearest');
gy = griddedInterpolant({T.xE, T.yN}, T.dHdN, 'linear', 'nearest');
sl = max(atan(hypot(gx(xy(:, 1), xy(:, 2)), gy(xy(:, 1), xy(:, 2)))));
end

function k = local_curv(xy, th)
d = sqrt(sum(diff(xy).^2, 2));
dth = diff(unwrap(th));
k = dth./max(d, 1e-9);
k = k(d > 1e-6);
if isempty(k), k = 0; end
end

function [xyR, thR, L] = local_resample(xy, th, N)
d = sqrt(sum(diff(xy).^2, 2));
keep = [true; d > 1e-9];
xy = xy(keep, :);  th = unwrap(th(keep));
s = [0; cumsum(sqrt(sum(diff(xy).^2, 2)))];
L = s(end);
si = linspace(0, L, N)';
xyR = interp1(s, xy, si);
thR = interp1(s, th, si);
end

function a = local_wrap(a)
a = mod(a + pi, 2*pi) - pi;
end

% =============================================================================================
%  estimates and validation
% =============================================================================================
function T = local_time(segs, aAcc, aDec)
% forward-backward speed profile on 0.1 m steps with per-segment cruise and v_end constraints
ds = 0.1;  vlim = [];
for k = 1:numel(segs)
    L = sum(sqrt(sum(diff(segs(k).xy).^2, 2)));
    n = max(1, round(L/ds));
    vlim = [vlim; segs(k).v*ones(n, 1)]; %#ok<AGROW>
end
v = vlim;  v(1) = 0;  v(end) = 0;
for i = 2:numel(v), v(i) = min(v(i), sqrt(v(i-1)^2 + 2*aAcc*ds)); end
for i = numel(v)-1:-1:1, v(i) = min(v(i), sqrt(v(i+1)^2 + 2*aDec*ds)); end
vm = (v(1:end-1) + v(2:end))/2;
T = sum(ds./max(vm, 1e-3));
end

function C = local_clearance(segs, foot, W, keep, G)
% minimum distance between the footprint (body + implement) and P.world obstacles / keep-outs
O = W.obstacles;
cyl = O.cyl(1:O.nCyl, :);
box = O.box(1:O.nBox, :);
film = O.film(1:O.nFilm, :);
pts = local_perimeter(foot, 0.05);           % FLU perimeter points
C = struct('min', Inf, 'iseg', 0, 'per_type', Inf(1, 5), 'keep_min', Inf, 'min_cyl', Inf, 'min_wall', Inf, ...
    'per_seg', Inf(1, numel(segs)));
% vectorised over the samples of a segment (5 cm steps): perimeter points PX / PY [nSamp x nPts]
fx = pts(:, 1)';  fy = pts(:, 2)';
for k = 1:numel(segs)
    xy = segs(k).xy;  th = segs(k).th;
    s = [0; cumsum(sqrt(sum(diff(xy).^2, 2)))];
    idx = unique([1; find(diff(floor(s/0.05)) > 0) + 1; numel(s)]);
    X = xy(idx, 1);  Y = xy(idx, 2);  c = cos(th(idx));  sn = sin(th(idx));
    PX = X + c.*fx - sn.*fy;
    PY = Y + sn.*fx + c.*fy;
    dcyl = Inf; dwall = Inf;
    for q = 1:size(cyl, 1)
        near = hypot(cyl(q, 1) - X, cyl(q, 2) - Y) < 4;
        if ~any(near), continue, end
        dq = min(hypot(PX(near, :) - cyl(q, 1), PY(near, :) - cyl(q, 2)), [], 2) - cyl(q, 3);
        % axis inside the footprint (body-frame test against the footprint rectangles) -> -1
        bx = c(near).*(cyl(q, 1) - X(near)) + sn(near).*(cyl(q, 2) - Y(near));
        by = -sn(near).*(cyl(q, 1) - X(near)) + c(near).*(cyl(q, 2) - Y(near));
        inF = any(bx >= foot(:, 1)' & bx <= foot(:, 2)' & by >= foot(:, 3)' & by <= foot(:, 4)', 2);
        dq(inF) = -1;
        dcyl = min(dcyl, min(dq));
    end
    for q = 1:size(film, 1)
        dwall = min(dwall, min(local_pt_seg([PX(:) PY(:)], film(q, 1:2), film(q, 3:4))));
    end
    for q = 1:size(box, 1)
        near = hypot(box(q, 1) - X, box(q, 2) - Y) <= 4 + hypot(box(q, 4), box(q, 5))/2;
        if ~any(near), continue, end
        Pn = [reshape(PX(near, :), [], 1), reshape(PY(near, :), [], 1)];
        dwall = min(dwall, local_pt_box(Pn, box(q, :)));
    end
    for q = 1:numel(keep)
        if strcmp(keep(q).type, 'circle')
            dk = min(hypot(X - keep(q).c(1), Y - keep(q).c(2))) - keep(q).r - G.geom.W/2;
        else
            dk = Inf;
            for j = 1:numel(X)
                [~, dj] = local_poly_dist([X(j) Y(j)], keep(q).poly);
                dk = min(dk, dj);
            end
            dk = dk - G.geom.W/2;
        end
        C.keep_min = min(C.keep_min, dk);
    end
    dmin = min([dcyl, dwall]);
    C.min_cyl = min(C.min_cyl, dcyl);  C.min_wall = min(C.min_wall, dwall);
    C.per_type(segs(k).type) = min(C.per_type(segs(k).type), dmin);
    C.per_seg(k) = dmin;
    if dmin < C.min, C.min = dmin; C.iseg = k; end
end
end

function pts = local_perimeter(F, h)
pts = zeros(0, 2);
for r = 1:size(F, 1)
    x = F(r, 1:2); y = F(r, 3:4);
    nx = max(2, ceil(diff(x)/h) + 1); ny = max(2, ceil(diff(y)/h) + 1);
    xs = linspace(x(1), x(2), nx)'; ys = linspace(y(1), y(2), ny)';
    pts = [pts; [xs y(1)*ones(nx, 1)]; [xs y(2)*ones(nx, 1)]; [x(1)*ones(ny, 1) ys]; [x(2)*ones(ny, 1) ys]]; %#ok<AGROW>
end
end

function d = local_pt_seg(Pt, a, b)
ab = b - a;  L2 = max(dot(ab, ab), 1e-12);
t = min(max(((Pt(:, 1) - a(1))*ab(1) + (Pt(:, 2) - a(2))*ab(2))/L2, 0), 1);
d = hypot(Pt(:, 1) - (a(1) + t*ab(1)), Pt(:, 2) - (a(2) + t*ab(2)));
end

function d = local_pt_box(Pt, b)
% distance of points to a yawed rectangle [xE yN zBase lx ly lz yaw ...] (negative inside)
c = cos(b(7)); s = sin(b(7));
dx = Pt(:, 1) - b(1); dy = Pt(:, 2) - b(2);
lx = c*dx + s*dy;  ly = -s*dx + c*dy;
qx = abs(lx) - b(4)/2;  qy = abs(ly) - b(5)/2;
out = hypot(max(qx, 0), max(qy, 0));
in = min(max(qx, qy), 0);
d = min(out + in);
end

function [in, d] = local_poly_dist(p, poly)
in = inpolygon(p(1), p(2), poly(:, 1), poly(:, 2));
d = Inf;
for k = 1:size(poly, 1)
    a = poly(k, :); b = poly(mod(k, size(poly, 1)) + 1, :);
    d = min(d, local_pt_seg(p, a, b));
end
if in, d = -d; end
end

function ok = local_in_geofence(segs, geo)
ok = true;
for k = 1:numel(segs)
    if ~all(inpolygon(segs(k).xy(:, 1), segs(k).xy(:, 2), geo(:, 1), geo(:, 2))), ok = false; return, end
end
end

function d = local_edge_to_wall(lanes, areaPoly, Wb)
% minimum lateral distance between an outer lane's body edge and the work-area (wall) line
d = Inf;
for k = [1 numel(lanes)]
    m = (lanes(k).p0 + lanes(k).p1)/2;
    [~, dw] = local_poly_dist(m, areaPoly);
    d = min(d, abs(dw) - Wb/2);
end
end

function c = local_coverage(segs, insetPoly, P, itype, h, ST)
% fraction of the inset polygon swept by the working implement (ROW and, for the mower, TURN)
im = P.structure.impl;
if itype == 1
    w = im.mower.w; xr = im.mower.pos(1) + [-1 1]*im.mower.dims(1)/2;
else
    w = im.tiller.w; xr = im.tiller.pos(1) + [-1 1]*im.tiller.dims(1)/2;
end
xg = (min(insetPoly(:, 1)):h:max(insetPoly(:, 1))) + h/2;
yg = (min(insetPoly(:, 2)):h:max(insetPoly(:, 2))) + h/2;
[X, Y] = meshgrid(xg, yg);
inP = inpolygon(X, Y, insetPoly(:, 1), insetPoly(:, 2));
cov = false(size(X));
rr = hypot(max(abs(xr)), w/2) + h;           % footprint reach from R
for k = 1:numel(segs)
    if ~(segs(k).type == ST.ROW || (segs(k).type == ST.TURN && segs(k).impl > 0)), continue, end
    xy = segs(k).xy; th = segs(k).th;
    s = [0; cumsum(sqrt(sum(diff(xy).^2, 2)))];
    idx = unique([1; find(diff(floor(s/h)) > 0) + 1; numel(s)]);
    for j = idx'
        ix = find(xg >= xy(j, 1) - rr & xg <= xy(j, 1) + rr);
        iy = find(yg >= xy(j, 2) - rr & yg <= xy(j, 2) + rr);
        if isempty(ix) || isempty(iy), continue, end
        c0 = cos(th(j)); s0 = sin(th(j));
        dx = X(iy, ix) - xy(j, 1); dy = Y(iy, ix) - xy(j, 2);
        lx = c0*dx + s0*dy; ly = -s0*dx + c0*dy;
        cov(iy, ix) = cov(iy, ix) | (lx >= xr(1) & lx <= xr(2) & abs(ly) <= w/2);
    end
end
c = nnz(cov & inP)/max(nnz(inP), 1);
end
