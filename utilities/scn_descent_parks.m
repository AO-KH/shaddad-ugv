function [E, B] = scn_descent_parks(S, P, t_trip, row, dirn)
%SCN_DESCENT_PARKS  Brakes-locked stops of a run against the structure park criterion (card G2-DSC).
%
%   E = scn_descent_parks(S, P, t_trip, row, dirn)
%   [E, B] = scn_descent_parks(S, P, t_trip, row, dirn)   also bound (a) at every sample of the run (t_trip may be [])
%     S       scn_collect logs (truth, telem, drive_truth)
%     P       the run's parameter struct
%     t_trip  start of each stop (s): an OVERSPEED trip (fault bit 16 rising edge) or the plant-side E-stop edge
%     row     implement row of P.structure.descent (implement code + 1: 1 none, 2 mower, 3 tiller, 4 sprayer)
%     dirn    +1 the stops of a forward descent, -1 of a reverse one
%   The criterion of the structure engineer (descent step 8, work/agentic/descent/structure/report_step8.md
%   section 6): a stop is safe when both hold.
%   (a) speed: v_park <= V_lin(p_trip) + dv(theta_R)
%       v_park   the largest truth |speed| from t_trip until both brakes are engaged (f_brake_rel <= 0.01, L and R)
%       p_trip   the MCU's held downhill pitch at t_trip, deg: dirn x telem.pitch peak-held over descent.t_hold
%                (as Descent_cap and Descent_trip; the hold starts at descent.theta_imu). The hold takes the last
%                round(t_hold/dt) samples, dt the step of the telem grid: on the 10 ms grid the MCU's own
%                round(t_hold/Ts.ctrl), and a decimated log keeps the t_hold window (card G2-DSC step 12c, review
%                RID-DSC-04)
%       V_lin    the row's v_fwd (dirn +1) or v_rev entry interpolated linearly in theta_bp at p_trip (the first
%                entry below the first breakpoint, the last one, 0, at and above the last)
%       dv       0.05 + g (sin theta_R - 0.035 cos theta_R) 0.05 s, the allowance the table rule subtracted;
%                theta_R the terrain slope under R at t_trip, downhill in the direction of travel
%                (-dirn x truth.grade_long)
%   (b) pitch: the peak body pitch relative to the terrain under R, the leading end down
%       (dirn x (truth.eul(2) + truth.grade_long)), from t_trip until 2 s after both brakes engage,
%       <= 1/2 (atan(d/h) - theta_Rmax): d the CG behind the leading contact station (the front station x of
%       P.structure.contact.Rc_R minus x_cg forward, x_cg minus the rear station in reverse), h the CG height above
%       the track bottom (-P.structure.mp.rcg0(3): R lies on the track bottom plane); theta_Rmax the steepest
%       downhill terrain under R in the same window (the robot still slides after the trip; the structure engineer's
%       own S2c check took the slope where the pitch peaks, 12.0 and 15.7 deg)
%   E: struct array, one element per stop, with t_trip, t_brakes (NaN: not both engaged to the end), v_park,
%      p_trip (deg), theta_R (deg), V_lin, dv, v_bound, ok_v, theta_Rmax (deg), rel_peak (deg), rel_limit (deg),
%      ok_pitch, d, h
%   B (card G2-DSC step 12c, review RID-DSC-03): bound (a) at every sample of truth.speed_fwd, with V_lin and dv
%      as above and the telem values of the last telem sample at or before it. Fields (column vectors): t; v the
%      truth |speed|; p the held pitch (deg); theta_R (deg); V_lin; dv; v_bound = V_lin + dv; on, true while the
%      drive is on and the MCU drives it (telem.mode MANUAL, HOLD, AUTO_START, AUTO or CTRL_STOP) with OVERSPEED
%      (b16) clear. The stops are not in it: from a trip on, (a) and (b) of E judge the robot with the pitch held at
%      the trip (the pitch-over of the braking raises the held pitch, so V_lin would fall under a stop that (a)
%      accepts). dv is made for downhill or level ground: on an uphill theta_R it falls below the 0.05 m/s sensing
%      allowance.

E = struct('t_trip', {}, 't_brakes', {}, 'v_park', {}, 'p_trip', {}, 'theta_R', {}, 'V_lin', {}, 'dv', {}, ...
    'v_bound', {}, 'ok_v', {}, 'theta_Rmax', {}, 'rel_peak', {}, 'rel_limit', {}, 'ok_pitch', {}, 'd', {}, 'h', {});
D = P.structure.descent;
if dirn > 0, V = D.v_fwd(row, :); else, V = D.v_rev(row, :); end
bp = D.theta_bp;
g = P.frames.g;
vlin = @(p) interp1(bp, V, min(max(p, bp(1)), bp(end)));
dvf = @(thR) 0.05 + g*(sin(thR) - 0.035*cos(thR))*0.05;          % thR rad, downhill +
% the MCU's held downhill pitch in the direction of travel (deg) on the telem grid
L = P.electronics.lim.descent;
tp = S.telem.pitch.t;
x = dirn*double(S.telem.pitch.v(:, 1))*180/pi;
n = max(1, round(L.t_hold/median(diff(tp))));
th = movmax([L.theta_imu*ones(n - 1, 1); x], [n - 1 0]);
th = th(n:end);
v = S.truth.speed_fwd;
e = S.truth.eul;
gr = scn_metrics('resample', S.truth.grade_long, e.t);
if nargout > 1
    gv = scn_metrics('resample', S.truth.grade_long, v.t);
    md = scn_metrics('resample', S.telem.mode, v.t);
    fw = scn_metrics('resample', S.telem.fault_word, v.t);
    B.t = v.t;
    B.v = abs(double(v.v(:, 1)));
    B.p = scn_metrics('resample', struct('t', tp, 'v', th), v.t);
    thv = -dirn*double(gv(:, 1));                                % rad
    B.theta_R = thv*180/pi;
    B.V_lin = vlin(B.p);
    B.dv = dvf(thv);
    B.v_bound = B.V_lin + B.dv;
    B.on = ismember(double(md(:, 1)), 3:7) & ~scn_metrics('bit', fw(:, 1), 16);
end
t_trip = t_trip(isfinite(t_trip));
if isempty(t_trip), return, end
xj = P.structure.contact.Rc_R(1, :);
rcg = P.structure.mp.rcg0;
if dirn > 0, d = max(xj) - rcg(1); else, d = rcg(1) - min(xj); end
h = -rcg(3);
rel = dirn*(double(e.v(:, 2)) + double(gr(:, 1)))*180/pi;
for j = 1:numel(t_trip)
    t0 = t_trip(j);
    tb = local_t_brakes(S, t0);
    t1 = tb;
    if isnan(t1), t1 = v.t(end); end
    w = v.t >= t0 - 1e-9 & v.t <= t1 + 1e-9;
    if ~any(w), w = v.t >= t0 - 1e-9; end
    k = find(tp <= t0 + 1e-9, 1, 'last');
    if isempty(k), k = 1; end
    p = th(k);
    thR = -dirn*double(scn_metrics('at', S.truth.grade_long, t0));        % rad
    Vl = vlin(p);
    dv = dvf(thR);
    we = e.t >= t0 - 1e-9 & e.t <= t1 + 2 + 1e-9;
    E(j).t_trip = t0;
    E(j).t_brakes = tb;
    E(j).v_park = max(abs(double(v.v(w, 1))));
    E(j).p_trip = p;
    E(j).theta_R = thR*180/pi;
    E(j).V_lin = Vl;
    E(j).dv = dv;
    E(j).v_bound = Vl + dv;
    E(j).ok_v = E(j).v_park <= E(j).v_bound;
    E(j).theta_Rmax = max(-dirn*double(gr(we, 1)))*180/pi;
    E(j).rel_peak = max(rel(we));
    E(j).rel_limit = 0.5*(atan(d/h)*180/pi - E(j).theta_Rmax);
    E(j).ok_pitch = E(j).rel_peak <= E(j).rel_limit;
    E(j).d = d;
    E(j).h = h;
end
end

% -------------------------------------------------------------------------------------------
function t = local_t_brakes(S, t0)
% first time >= t0 with both brakes engaged (f_brake_rel <= 0.01)
fL = S.drive_truth.L.f_brake_rel;  fR = S.drive_truth.R.f_brake_rel;
bR = scn_metrics('resample', fR, fL.t);
k = find(fL.t >= t0 - 1e-9 & double(fL.v(:, 1)) <= 0.01 & double(bR(:, 1)) <= 0.01, 1, 'first');
if isempty(k), t = NaN; else, t = fL.t(k); end
end
