function pe = gnc_pathAnalyzer(nav, win, prm) %#codegen
%GNC_PATHANALYZER  Closest point, cross-track / heading error, progress and curvature preview.
%
%   pe = gnc_pathAnalyzer(nav, win, prm)          (MATLAB Function block in gnc_PathAnalyzer, 0.05 s)
%     nav  Bus_NavCond     pose = [x y psi] (ENU, psi CCW from East, rad), v (m/s)
%     win  Bus_PathWindow  PA [N x 3] (x, y, theta deg), sA, kA, vA [N x 1] of the current segment A,
%                          PB, sB, kB of the next segment B, ver (window version)
%     prm  P.gnc.pa        N, n_back, n_fwd, p, Ts, v_prev_min, d_valid, t_look (non-tunable sub-struct)
%     pe   Bus_PathError   xte (+ = robot LEFT of the path), epsi = wrap(psi - psi_ref), psi_ref,
%                          kappa_ref, v_ref (profiled speed at s + v t_look), s, s_rem, idx, kappa_prev [p x 1],
%                          xy_ref, valid (closest distance < d_valid)
%
%   Why a MATLAB Function (docs/specs/gnc.md 4.2): no library block computes the cross-track error,
%   heading error, arc-length progress or a curvature preview on a polyline (Pure Pursuit keeps its
%   projection internal; the Stanley block needs the reference pose from outside).
%
%   Search: monotonic local window i0-n_back .. i0+n_fwd around the previous closest segment, reset to
%   the start of A whenever win.ver changes (prevents snapping to the neighbouring lane 0.81 m away).
%   The first and last polyline segments are extended (projection parameter not clamped) so s, s_rem
%   and xte stay continuous slightly before the start and past the end of A. The curvature preview
%   samples s + max(v, v_prev_min) k Ts, k = 1..p, on A followed by B (beyond B: last value of B).

persistent i0 lastVer
if isempty(i0)
    i0 = 1;
    lastVer = uint16(0);
end
N = prm.N;
if win.ver ~= lastVer
    i0 = 1;
    lastVer = win.ver;
end

px = nav.pose(1);  py = nav.pose(2);
best = inf;  ib = 1;  tb = 0;  qx = win.PA(1, 1);  qy = win.PA(1, 2);  abx = 1;  aby = 0;
lo = max(1, i0 - prm.n_back);
hi = min(N - 1, i0 + prm.n_fwd);
for i = lo:hi
    ax = win.PA(i, 1);      ay = win.PA(i, 2);
    ux = win.PA(i + 1, 1) - ax;  uy = win.PA(i + 1, 2) - ay;
    L2 = ux*ux + uy*uy;
    if L2 < 1e-12
        continue                              % coincident poses (zero window before the first prep)
    end
    t = ((px - ax)*ux + (py - ay)*uy)/L2;
    if i > 1,     t = max(t, 0); end          % extend the first segment backwards
    if i < N - 1, t = min(t, 1); end          % extend the last segment forwards
    cx = ax + t*ux;  cy = ay + t*uy;
    d2 = (px - cx)^2 + (py - cy)^2;
    if d2 < best
        best = d2;  ib = i;  tb = t;  qx = cx;  qy = cy;  abx = ux;  aby = uy;
    end
end

pe = struct('xte', 0, 'epsi', 0, 'psi_ref', 0, 'kappa_ref', 0, 'v_ref', 0, 's', 0, 's_rem', 0, ...
    'idx', uint16(1), 'kappa_prev', zeros(prm.p, 1), 'xy_ref', [px; py], 'valid', false);
if ~isfinite(best)
    return                                    % no usable window
end
i0 = ib;

tc = min(max(tb, 0), 1);
th1 = win.PA(ib, 3)*pi/180;
th2 = win.PA(ib + 1, 3)*pi/180;
psiRef = local_wrap(th1 + tc*local_wrap(th2 - th1));
nab = sqrt(abx*abx + aby*aby);
xte = (abx*(py - qy) - aby*(px - qx))/nab;     % + = left of the path (cross(ab, p - q))
s = win.sA(ib) + tb*(win.sA(ib + 1) - win.sA(ib));
sEnd = win.sA(N);

pe.xte = xte;
pe.epsi = local_wrap(nav.pose(3) - psiRef);
pe.psi_ref = psiRef;
pe.kappa_ref = win.kA(ib) + tc*(win.kA(ib + 1) - win.kA(ib));
% profiled speed read ahead by v t_look so the commanded deceleration anticipates the command lag
sv = min(s + max(nav.v, 0)*prm.t_look, sEnd);
jv = ib;
while jv < N - 1 && win.sA(jv + 1) < sv
    jv = jv + 1;
end
pe.v_ref = local_lin(win.sA(jv), win.sA(jv + 1), win.vA(jv), win.vA(jv + 1), sv);
pe.s = s;
pe.s_rem = sEnd - s;
pe.idx = uint16(ib);
pe.xy_ref = [qx; qy];
pe.valid = best < prm.d_valid^2;

% ---- curvature preview over the MPC horizon (A then B) ----
vp = max(nav.v, prm.v_prev_min);
ja = max(ib, 1);  jb = 1;
for k = 1:prm.p
    sk = s + vp*k*prm.Ts;
    if sk <= sEnd
        while ja < N - 1 && win.sA(ja + 1) < sk
            ja = ja + 1;
        end
        pe.kappa_prev(k) = local_lin(win.sA(ja), win.sA(ja + 1), win.kA(ja), win.kA(ja + 1), sk);
    else
        sb = sk - sEnd;
        while jb < N - 1 && win.sB(jb + 1) < sb
            jb = jb + 1;
        end
        if sb >= win.sB(N)
            pe.kappa_prev(k) = win.kB(N);
        else
            pe.kappa_prev(k) = local_lin(win.sB(jb), win.sB(jb + 1), win.kB(jb), win.kB(jb + 1), sb);
        end
    end
end
end

% ============================================================================================
function y = local_lin(x0, x1, y0, y1, x)
dx = x1 - x0;
if dx <= 1e-12
    y = y1;
else
    w = min(max((x - x0)/dx, 0), 1);
    y = y0 + w*(y1 - y0);
end
end

function a = local_wrap(a)
a = a - 2*pi*floor((a + pi)/(2*pi));
end
