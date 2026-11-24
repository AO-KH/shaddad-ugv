function [in, crit] = scn_S1_rc_drive(P, varargin)
%SCN_S1_RC_DRIVE  S1 RC drive (Phase 1), architecture section 12 / D33 / D37.
%
%   [in, crit] = scn_S1_rc_drive(P)
%   [in, crit] = scn_S1_rc_drive(P, 'StopTime', T, 'Mode', m)
%   120 s, flat farm soil, no implement. Transmitter on at 0.5 s (the link must be up before the arm
%   edge), t = 2 s arm edge (sticks neutral); t = 5 s throttle for v_target 1.2 m/s; t = 30 s steer +0.5
%   (right, omega < 0); t = 45 s steer 0; t = 52 s throttle released (stop-distance measurement from
%   1.2 m/s with the sticks centred, D33 - inserted, the listed schedule has no release from 1.2 m/s);
%   t = 60 s throttle for -0.6 m/s; t = 80 s throttle 0.
%   Criteria: 90 % speed rise <= 2.0 s; overshoot <= 5 %; steady error <= 0.03 m/s; yaw rate within
%   +/-10 % of w_ref after 2 s and negative for steer > 0; TX frame -> DRV_CMD frame <= 30 ms; ScenarioIn
%   stick -> ESC speed reference <= 65 ms (D37); stop distance after the release from 1.2 m/s <= 0.95 m;
%   energy audit eps_el, eps_mech <= 1 % (T-INT-08); RTF >= 5 in accelerator (BLOCKED without a C
%   compiler, the measured Normal-mode RTF is reported by run_scenario).

ip = inputParser;
ip.addParameter('StopTime', 120);
ip.addParameter('Mode', 'normal');
ip.parse(varargin{:});
T = ip.Results.StopTime;
if P.world.id ~= 1
    P = shaddad_params('World', 1, 'Implement', 0, 'Phase', 1, 'Assign', false);
end
[x12, v12] = scn_rc_stick(P, 1.2);
[xr, vr] = scn_rc_stick(P, -0.6);
t_thr = 5; t_steer = 30; t_steer0 = 45; t_rel = 52; t_rev = 60; t_stop = 80;
tab = scn_rc_tab(0.5, 2);
tab.op.stick_thr = [0 0; t_thr x12; t_rel 0; t_rev xr; t_stop 0];
tab.op.stick_steer = [0 0; t_steer 0.5; t_steer0 0];
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, 'S1', 'StopTime', T, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T);

M = @scn_metrics;
Tf = P.rc.frame_period;
chThr = local_ch(P, 'thr', 2);
t_esc_rt = Ts_ctrl(P);                       % drive_system CAN input RT 0.01 -> 0.001 (deterministic, one 10 ms period)
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S1.rise', sprintf('90 %% speed rise time (target %.3f m/s)', v12), 'truth.speed_fwd', ...
    @(S) M('first', S.truth.speed_fwd, @(v) v >= 0.9*v12, t_thr) - t_thr, '<=', 2.0, 's');
crit(end+1) = local_c('S1.overshoot', 'speed overshoot', 'truth.speed_fwd', ...
    @(S) local_overshoot(S, t_thr, t_steer, v12), '<=', 0.05, '1');
crit(end+1) = local_c('S1.ss_err', 'steady speed error, mean |v - v_target| over 20-30 s', 'truth.speed_fwd', ...
    @(S) local_mean_abs(S.truth.speed_fwd, 20, 30, v12), '<=', 0.03, 'm/s');
crit(end+1) = local_c('S1.yaw_track', 'max |r - w_ref|/|w_ref| over 32-45 s (0.1 s mean)', 'truth.omega_B, telem.w_ref', ...
    @(S) local_yaw_err(S, t_steer + 2, t_steer0), '<=', 0.10, '1');
crit(end+1) = local_c('S1.yaw_sign', 'mean yaw rate for steer +0.5 (must be < 0)', 'truth.omega_B(3)', ...
    @(S) mean(local_win(S.truth.omega_B, t_steer + 2, t_steer0, 3)), '<', 0, 'rad/s');
crit(end+1) = local_c('S1.lat_tx_drv', 'TX frame emission -> DRV_CMD frame (MCU Task_10 output)', ...
    'rc_frame.ch_raw, telem.w_ref_motor', @(S) local_latency_tx(S, t_thr, chThr, Tf), '<=', 0.030, 's');
crit(end+1) = local_c('S1.lat_stick_esc', 'ScenarioIn stick -> ESC speed reference (DRV_CMD + 10 ms ESC RT)', ...
    'telem.w_ref_motor', @(S) local_first_cmd(S, t_thr) + t_esc_rt - t_thr, '<=', 0.065, 's');
crit(end+1) = local_c('S1.stop_dist', 'stop distance after throttle release from 1.2 m/s', 'truth.pos_W', ...
    @(S) M('path', S.truth.pos_W, t_rel, t_rev - 0.1), '<=', 0.95, 'm');
crit(end+1) = local_c('S1.rev_speed', sprintf('reverse speed at 75 s (target %.3f m/s)', vr), 'truth.speed_fwd', ...
    @(S) abs(M('at', S.truth.speed_fwd, 75) - vr), '<=', 0.05, 'm/s');
crit(end+1) = local_c('S1.eps_el', 'electrical energy-audit residual (T-INT-08)', 'kpi.eps_el', ...
    @(S) S.kpi.eps_el.v(end), '<=', 0.01, '1');
crit(end+1) = local_c('S1.eps_mech', 'mechanical energy-audit residual (T-INT-08)', 'kpi.eps_mech', ...
    @(S) S.kpi.eps_mech.v(end), '<=', 0.01, '1');
crit(end+1) = local_c('S1.no_coll', 'collisions', 'kpi.n_coll', @(S) double(S.kpi.n_coll.v(end)), '==', 0, '');
c = local_c('S1.rtf', 'RTF >= 5 in accelerator (top + Model blocks)', 'run wall time', ...
    @(S) NaN, '>=', 5, '');
c.blocked = 'BLOCKED: no C compiler (accelerator / rapid accelerator unavailable); Normal-mode RTF in res.rtf';
crit(end+1) = c;
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function t = Ts_ctrl(P)
t = P.sim.Ts_allowed(3);                     % 0.01 s
end

function ch = local_ch(P, name, def)
ch = def;
if isfield(P.rc, 'ch_map') && isfield(P.rc.ch_map, name)
    ch = P.rc.ch_map.(name);
end
end

function v = local_win(sig, t0, t1, col)
m = sig.t >= t0 & sig.t <= t1;
v = double(sig.v(m, col));
end

function e = local_mean_abs(sig, t0, t1, ref)
e = mean(abs(local_win(sig, t0, t1, 1) - ref));
end

function o = local_overshoot(S, t0, t1, ref)
o = max(local_win(S.truth.speed_fwd, t0, t1, 1))/ref - 1;
end

function e = local_yaw_err(S, t0, t1)
r = S.truth.omega_B;
m = r.t >= t0 & r.t <= t1;
tr = r.t(m);
rm = scn_metrics('movmean', tr, double(r.v(m, 3)), 0.1);
wr = double(scn_metrics('resample', S.telem.w_ref, tr));
e = max(abs(rm - wr)./max(abs(wr), 1e-3));
end

function t = local_first_cmd(S, t0)
% first Task_10 step whose shaped speed reference (packed into DRV_CMD as w_ref = k_v2w*v_track in the
% same step) responds to the new throttle; the yaw-rate loop part of w_ref_motor is excluded
t = scn_metrics('first', S.telem.v_ref, @(v) double(v) > 1e-4, t0);
end

function L = local_latency_tx(S, t0, ch, Tf)
% TX sample time of the first frame carrying the new throttle = release time of that frame (rc_frame,
% aligned with the release, T07) - one frame period; DRV_CMD time = first Task_10 output with a
% non-zero ESC speed setpoint (telem.w_ref_motor, packed into DRV_CMD in the same Task_10 step).
f = S.rc_frame.ch_raw;
mid = double(f.v(find(f.t <= t0, 1, 'last'), ch));
t_rel = scn_metrics('first', f, @(v) abs(double(v(:, ch)) - mid) > 0.5, t0);
L = local_first_cmd(S, t_rel - Tf) - (t_rel - Tf);
end
