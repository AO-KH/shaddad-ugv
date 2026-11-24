function v = gnc_descentCap(theta, impl, rev, prm, raised) %#codegen
%GNC_DESCENTCAP  Downhill speed cap for one pitch and one fitted implement (P.structure.descent through P.gnc.desc).
%
%   v = gnc_descentCap(theta, impl, rev, prm)
%   v = gnc_descentCap(theta, impl, rev, prm, raised)   raised true with the mower: the deck-raised rows
%                                                       prm.v_fwd_raised / v_rev_raised (gauge wheels off the ground)
%     theta  downhill pitch in the direction of travel, rad (+ = downhill; level or uphill gives the 0 deg value).
%            Forward travel: theta = Bus_NavCond.pitch (+ nose down); reverse travel: theta = -pitch
%     impl   fitted implement, Bus_MissionPlan.implement_type (0 NONE 1 MOWER 2 TILLER 3 SPRAYER); any other value
%            gets the lowest value of all rows (unknown load)
%     rev    false: forward travel (prm.v_fwd), true: reverse travel (prm.v_rev)
%     prm    P.gnc.desc: theta_bp (rad), v_fwd / v_rev (rows implement_type + 1, columns theta_bp), tol (rad), dv (m/s)
%     v      m/s, max(0, c - dv): c is the highest speed at which a brakes-locked stop may start without tipping the
%            robot over, the value of the first breakpoint at or above theta, so between breakpoints the steeper one (a
%            pitch within tol above a breakpoint counts as that breakpoint: an exact 20 deg is 20 deg + 1e-15 rad in
%            floating point), above the last breakpoint its value (0, TILT_WARN); dv is the MCU's margin of its speed
%            target below the cap (P.electronics.lim.descent.dv), so the GNC plans to what the MCU applies
%
%   Used by gnc_descentLimit (SpeedGovernor) and gnc_descentEnvelope (CommandShaping), so both apply one rule.
%   Table and rule: params/param_structure.m S.descent, work/agentic/descent/structure/report_part2.md.

if nargin < 5
    raised = false;
end
bp = prm.theta_bp;
n = numel(bp);
j = n;
for k = 1:n
    if theta <= bp(k) + prm.tol
        j = k;
        break
    end
end
if rev
    T = prm.v_rev;
else
    T = prm.v_fwd;
end
r = double(impl) + 1;
if raised && r == 2                             % mower (implement_type 1) with the deck raised
    if rev
        c = prm.v_rev_raised(j);
    else
        c = prm.v_fwd_raised(j);
    end
elseif r >= 1 && r <= size(T, 1) && r == round(r)
    c = T(r, j);
else
    c = min(T(:, j));
end
v = max(0, c - prm.dv);
end
