function [v, w] = gnc_descentEnvelope(v, w, pitch, impl, prm) %#codegen
%GNC_DESCENTENVELOPE  Command envelope of the downhill speed cap (MATLAB Function DescentEnv in gnc_CommandShaping).
%
%   [v, w] = gnc_descentEnvelope(v, w, pitch, impl, prm)
%     v, w   speed command (m/s, after the [v_min_reverse, v_max] saturation) and yaw-rate command (rad/s, after its
%            rate limit and bound)
%     pitch  Bus_NavCond.pitch, rad (+ nose down): the downhill pitch of forward travel; -pitch is that of reverse travel
%     impl   Bus_MissionPlan.implement_type (0 NONE 1 MOWER 2 TILLER 3 SPRAYER)
%     prm    P.gnc.desc (gnc_descentCap)
%   A forward speed above gnc_descentCap(pitch, impl, forward), or a reverse speed beyond gnc_descentCap(-pitch, impl,
%   reverse), is scaled down to the cap, and the yaw rate with it (curvature kept, as in the track-speed envelope,
%   D43); a pivot (v = 0) is not limited. With a cap of 0 the command becomes 0: the tiller's reverse cap is 0 at any
%   slope, so no reverse command leaves the GNC with the tiller fitted (the user, 2026-11-15).
%   The cap at the current pitch only: the look-ahead is the governor's (gnc_descentLimit). This envelope binds for the
%   TEB command (BYPASS), which does not use v_gov, for reverse commands, and while the rate-limited v_gov catches up
%   with a pitch class change.

cf = gnc_descentCap(pitch, impl, false, prm);
cr = gnc_descentCap(-pitch, impl, true, prm);
k = 1;
if v > cf
    k = cf/v;
elseif v < -cr
    k = -cr/v;
end
v = k*v;
w = k*w;
end
