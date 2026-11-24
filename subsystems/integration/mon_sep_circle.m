function sep = mon_sep_circle(row, p, R, hA)
%MON_SEP_CIRCLE  Separation of the robot footprint from one circular obstacle (Sim_Monitors FE_Cyl, FE_Dyn).
%
%   sep = mon_sep_circle(row, p, R, hA)
%     row  obstacle row [x y r] (1 x 3; world frame, m; r the radius)
%     p    footprint centre (2 x 1; world frame, m)
%     R    footprint rotation, body to world (2 x 2)
%     hA   footprint half extents (2 x 1, m): P.sim.mon.footprint(:)/2
%     sep  planar separation (m), negative when they overlap: the signed distance of the circle centre to the
%          footprint rectangle (outside, the distance to it; inside, minus the distance to its nearest edge),
%          minus r
%
%   The MATLAB Function Sep of the For Each Subsystems FE_Cyl and FE_Dyn calls it once per obstacle row
%   (D70). It does what their block network did, operation for operation and in the same order, so the
%   result is bit-identical (tests/integration/t_mf_equiv, testMonitorsEquivalent). The comments name the
%   blocks of that network.
%
%   See also mon_sep_rect, build_sim_monitors.

%#codegen
d = R.' * ([row(1); row(2)] - p);           % centre in the body frame: Sel_c, cT, Rt, Dw, d
q = abs(d) - hA;                            % Pt_abs, Pt_q
outside = sqrt(sum(max(q, 0).^2, 1));       % Pt_qpos, Pt_sq, Pt_sum, Pt_out
inside = min(max(q(1, :), q(2, :)), 0);     % Pt_q1, Pt_q2, Pt_qmax, Pt_in
sep = min(outside + inside) - row(3);       % Pt_signed, Pt_dist, Sel_r, Sep
end
