function [x, v] = scn_rc_stick(P, v_target, axis, knob)
%SCN_RC_STICK  Operator stick value that makes the MCU RC mapping output a given target (T16 helper).
%
%   [x, v] = scn_rc_stick(P, v_target)               throttle stick for v_rc = v_target [m/s]
%   [x, w] = scn_rc_stick(P, w_target, 'steer')      steering stick for omega_rc = w_target [rad/s]
%   ... = scn_rc_stick(P, target, axis, knob)        speed-scale knob position (default +1 = 100 %)
%
%   Inverts the ICD 7 chain of fw_rc_mapping: TX raw = raw_mid + round(raw_half*x) (RC_Link_Sim),
%   receiver x = (raw - raw_mid)/raw_half, Dead Zone +/-deadband, Gain 1/(1 - deadband),
%   Polynomial e*u^3 + (1-e)*u (expo_thr / expo_steer), speed scale s = s_min + (knob+1)/2*(1 - s_min),
%   v_rc = thr*v_max_man*s, omega_rc = steer_to_omega_sign*steer*w_max_man*s.
%   The returned stick is on the TX raw grid; the second output is the target the MCU then decodes.

if nargin < 3 || isempty(axis), axis = 'thr'; end
if nargin < 4 || isempty(knob), knob = 1; end
rm = P.electronics.rcmap;
s = rm.s_min + (knob + 1)/2*(1 - rm.s_min);
switch axis
    case 'thr'
        e = rm.expo_thr;   full = P.electronics.lim.v_max_man*s;   sg = 1;
    case 'steer'
        e = rm.expo_steer; full = P.electronics.lim.w_max_man*s;   sg = rm.steer_to_omega_sign;
    otherwise
        error('scn_rc_stick:axis', 'axis must be thr or steer');
end
y = sg*v_target/full;
if abs(y) > 1, error('scn_rc_stick:range', 'target %g is outside the stick range', v_target); end
% invert e*u^3 + (1-e)*u = y (monotonic on [-1 1])
u = fzero(@(u) e*u.^3 + (1 - e)*u - y, [-1 1]);
xs = sign(u)*(abs(u)*(1 - rm.deadband) + rm.deadband*(abs(u) > 0));
raw = P.rc.raw_mid + round(P.rc.raw_half*xs);
x = (raw - P.rc.raw_mid)/P.rc.raw_half;
% forward model of the decoded target
xr = (raw - rm.raw_mid(1))/rm.raw_half;
ud = sign(xr)*max(abs(xr) - rm.deadband, 0)/(1 - rm.deadband);
v = sg*(e*ud^3 + (1 - e)*ud)*full;
end
