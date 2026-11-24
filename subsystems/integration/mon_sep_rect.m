function sep = mon_sep_rect(row, p, R, hA)
%MON_SEP_RECT  Separation of the robot footprint from one oriented box obstacle (Sim_Monitors FE_Box).
%
%   sep = mon_sep_rect(row, p, R, hA)
%     row  obstacle row [x y yaw hx hy] (1 x 5; world frame, m and rad; hx, hy the half extents)
%     p    footprint centre (2 x 1; world frame, m)
%     R    footprint rotation, body to world (2 x 2)
%     hA   footprint half extents (2 x 1, m): P.sim.mon.footprint(:)/2
%     sep  planar separation (m), negative when they overlap. Apart: the distance between the two rectangles,
%          the smallest signed distance of a corner of either one to the other. Overlapping (every margin of
%          the separating-axis test >= 0): minus the smallest margin, the penetration.
%
%   The MATLAB Function Sep of the For Each Subsystem FE_Box calls it once per obstacle row (D70). It does
%   what the block network did, operation for operation and in the same order, so the result is bit-identical
%   (tests/integration/t_mf_equiv, testMonitorsEquivalent). The comments name the blocks of that network.
%
%   See also mon_sep_circle, build_sim_monitors.

%#codegen
Rt = R.';                                   % Rt: world to body
d = Rt * ([row(1); row(2)] - p);            % box centre in the body frame: Sel_c, cT, Dw, d
hB = [row(4); row(5)];                      % Sel_h, hT
c = cos(row(3));                            % Sel_yaw, Cos
s = sin(row(3));                            % Sin
Rrel = Rt * [c, -s; s, c];                  % box to body: NSin, Rb_vec, Rb, Rrel
RrelT = Rrel.';                             % RrelT
one4 = ones(1, 4);                          % Ones4
S = [1 1 -1 -1; 1 -1 -1 1];                 % S, the corner signs
hA4 = hA*one4;                              % hA4
hB4 = hB*one4;                              % hB4
d4 = d*one4;                                % D4
VB = Rrel*(hB4.*S) + d4;                    % box corners in the body frame: VBloc, VBrot, VB
W = RrelT*(hA4.*S - d4);                    % footprint corners in the box frame: VAloc, VAd, W
dist = min(local_dist(VB, hA4), local_dist(W, hB4));    % BinA_*, AinB_*, Dist_min
absR = abs(Rrel);                           % AbsR
margA = hA + absR*hB - abs(d);              % on the footprint's axes: Proj_A, AbsD, Marg_A
margB = absR.'*hA + hB - abs(RrelT*d);      % on the box's axes: AbsRT, Proj_B, dB, AbsdB, Marg_B
pen = min([margA; margB]);                  % Marg, Pen
if pen >= 0                                 % Overlap
    sep = -pen;                             % NegPen
else
    sep = dist;                             % Sep (Switch)
end
end

function dist = local_dist(v, h)
% smallest signed distance of the points v (2 x 4) to the centred axis-aligned rectangle of half extents h
% (2 x 4): outside |max(q, 0)|, inside min(max(q1, q2), 0), q = |v| - h (the block group <tag>_*)
q = abs(v) - h;                             % _abs, _q
dist = min(sqrt(sum(max(q, 0).^2, 1)) + min(max(q(1, :), q(2, :)), 0));    % _qpos ... _signed, _dist
end
