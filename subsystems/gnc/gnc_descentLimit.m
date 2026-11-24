function v = gnc_descentLimit(nav, pe, win, impl, grid, H, prm) %#codegen
%GNC_DESCENTLIMIT  Downhill speed limit of the speed governor: the descent cap at the held nav pitch and along the path.
%
%   v = gnc_descentLimit(nav, pe, win, impl, grid, H, prm)    (MATLAB Function DescentCap in gnc_SpeedGovernor, 0.05 s)
%     nav   Bus_NavCond     pitch (rad, + nose down), v (m/s)
%     pe    Bus_PathError   s (arc length of the closest point on segment A), idx, valid
%     win   Bus_PathWindow  PA / sA (current segment A), PB / sB (next segment B, starting where A ends), valid
%     impl  Bus_MissionPlan.implement_type (0 NONE 1 MOWER 2 TILLER 3 SPRAYER)
%     grid  [xE(1) yN(1) res] of the DEM P.world.terrain (evenly spaced, Table(i, j) <-> (xE(i), yN(j)))
%     H     P.world.terrain.H, m (elevation, up)
%     prm   P.gnc.desc (non-tunable): theta_bp, v_fwd, v_rev, tol (gnc_descentCap), n_hold, L_c, ds, n_look, a_la,
%           t_resp
%     v     m/s, forward speed limit = the lowest of
%           - the cap at the nav pitch held at its peak over the last n_hold steps (t_hold; crests and bumps count, as
%             in the MCU cap);
%           - along the planned path ahead (window A, then B), every ds from the closest point up to (n_look - 1) ds:
%             sqrt(c_k^2 + 2 a_la max(0, d_k - v t_resp)), with c_k the cap at the body pitch predicted for the robot
%             centre at distance d_k: the DEM chord over the track contact length L_c along the path heading (rear
%             end minus front end, + = nose down). The command reaches c_k t_resp before the centre reaches d_k
%             (t_resp: the time the robot's speed needs to follow the command), so the robot is down to c_k there,
%             having slowed at a_la, and never needs to brake on the steeper part itself.
%     Without a valid window and path error only the held nav pitch counts.
%
%   Why a MATLAB Function (gnc spec 2.5): no block samples a DEM along a polyline. Forward travel only: every tracker
%   that v_gov feeds drives forward (Pure Pursuit, row tracker, dock approach). The reverse cap, and the cap on the TEB
%   command, which does not use v_gov, are CommandShaping's (gnc_descentEnvelope).

persistent buf k0
if isempty(buf)
    buf = zeros(prm.n_hold, 1);
    k0 = 1;
end
buf(k0) = nav.pitch;
k0 = k0 + 1;
if k0 > prm.n_hold
    k0 = 1;
end
v = gnc_descentCap(max(buf), impl, false, prm);
if ~(pe.valid && win.valid)
    return
end
N = size(win.PA, 1);
dLat = max(nav.v, 0)*prm.t_resp;
sEnd = win.sA(N);
ja = max(1, min(double(pe.idx), N - 1));
jb = 1;
h = prm.L_c/2;
x = 0;  y = 0;  th = 0;                         % defined on every path (code generation)
for q = 0:prm.n_look - 1
    d = q*prm.ds;
    sq = pe.s + d;
    if sq <= sEnd
        [x, y, th, ja] = local_at(win.PA, win.sA, sq, ja);
    else
        sb = sq - sEnd;
        if sb > win.sB(N)
            break                                 % beyond the next segment: nothing planned
        end
        [x, y, th, jb] = local_at(win.PB, win.sB, sb, jb);
    end
    c = cos(th);  sn = sin(th);
    hr = local_dem(grid, H, x - h*c, y - h*sn);
    hf = local_dem(grid, H, x + h*c, y + h*sn);
    ck = gnc_descentCap(atan((hr - hf)/prm.L_c), impl, false, prm);
    v = min(v, sqrt(ck*ck + 2*prm.a_la*max(0, d - dLat)));
end
end

% ============================================================================================
function [x, y, th, j] = local_at(Pw, sw, sq, j)
% pose at arc length sq on the window polyline Pw [x y theta_deg] with cumulative length sw (forward search from j)
N = numel(sw);
while j < N - 1 && sw(j + 1) < sq
    j = j + 1;
end
dsj = sw(j + 1) - sw(j);
w = 0;
if dsj > 1e-9
    w = min(max((sq - sw(j))/dsj, 0), 1);
end
x = Pw(j, 1) + w*(Pw(j + 1, 1) - Pw(j, 1));
y = Pw(j, 2) + w*(Pw(j + 1, 2) - Pw(j, 2));
t1 = Pw(j, 3)*pi/180;
t2 = Pw(j + 1, 3)*pi/180;
d12 = t2 - t1;
d12 = d12 - 2*pi*floor((d12 + pi)/(2*pi));
th = t1 + w*d12;
end

function z = local_dem(grid, H, x, y)
% bilinear DEM height at (x, y), clamped to the grid edge
nx = size(H, 1);
ny = size(H, 2);
if nx < 2 || ny < 2
    z = H(1);
    return
end
u = min(max((x - grid(1))/grid(3) + 1, 1), nx);
w = min(max((y - grid(2))/grid(3) + 1, 1), ny);
i = min(floor(u), nx - 1);
j = min(floor(w), ny - 1);
fu = u - i;
fw = w - j;
z = (1 - fu)*(1 - fw)*H(i, j) + fu*(1 - fw)*H(i + 1, j) + (1 - fu)*fw*H(i, j + 1) + fu*fw*H(i + 1, j + 1);
end
