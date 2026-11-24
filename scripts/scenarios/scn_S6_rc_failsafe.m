function [in, crit] = scn_S6_rc_failsafe(P, varargin)
%SCN_S6_RC_FAILSAFE  S6 RC failsafe (Phase 1), architecture section 12 / D21 / D33 / D34 / D37.
%
%   [in, crit] = scn_S6_rc_failsafe(P, 'Variant', v)     v = 'a' (default) | 'b' | 'c' | 'd'
%   MANUAL drive at 1.0 m/s (transmitter on 0.5 s, arm 2 s, throttle 4 s).
%   (a) t = 20 s ScenarioIn.flt.rc_link_loss (RF frames lost: frame_lost, then failsafe frames),
%       link back at 25 s with the throttle still held; throttle neutral at 28 s, arm off 29 s, arm on
%       30 s, throttle again at 31 s (re-arm resumes motion).
%   (b) same with flt.rc_line_silent (no SBUS bytes).
%   (c) flt.rc_byte_err_rate = 0.01 from 10 to 25 s while driving at 1.0 m/s (about 78 % of the SBUS
%       frames intact; the longest gap between valid frames measured there is 70 ms, below the 0.10 s RC
%       link-loss timeout of ICD 7, so the drive continues).
%   (d) flt.rc_byte_err_rate = 0.05 from 10 to 25 s (only about 21 % of the frames intact): gaps between
%       valid frames exceed the 0.10 s timeout, so RC_LINK_LOST and the controlled stop are the correct
%       reaction (the speed / MANUAL rows of (c) do not apply).
%   Criteria (a), (b): CTRL_STOP <= 0.13 s after the loss onset (the fault reaches RC_Link_Sim through the
%   10 ms input RT, so onset = ScenarioIn edge + 0.01 s); deceleration (100 ms moving average)
%   <= 1.05 m/s^2; standstill <= 1.5 s after detection; ESTOP never entered (D34); no motion after the
%   link returns until the arm switch is toggled off -> on. (c): no command glitch > 2 % of full scale
%   (MCU v_target / w_target and the normalised stick channels), corrupted frames rejected
%   (telem.sbus_bad increases), MCU stays in MANUAL. (d): no stick-channel glitch > 2 % of full scale,
%   corrupted frames rejected, CTRL_STOP <= 0.2 s after RC_LINK_LOST (telem.rc_link_ok false), ESTOP never
%   entered (D34).

ip = inputParser;
ip.addParameter('Variant', 'a');
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
var = lower(char(ip.Results.Variant));
if P.world.id ~= 1
    P = shaddad_params('World', 1, 'Implement', 0, 'Phase', 1, 'Assign', false);
end
[x10, v10] = scn_rc_stick(P, 1.0);
tab = scn_rc_tab(0.5, 2);
M = @scn_metrics;
CTRL_STOP = 7; ESTOP = 8; MANUAL = 3;
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
switch var
    case {'a', 'b'}
        T = 34;
        t_loss = 20; t_back = 25; t_neutral = 28; t_off = 29; t_on = 30; t_go = 31;
        tab.op.stick_thr = [0 0; 4 x10; t_neutral 0; t_go x10];
        tab.op.sw_arm = [0 0; 2 1; t_off 0; t_on 1];
        if var == 'a'
            tab.flt.rc_link_loss = [0 0; t_loss 1; t_back 0];
        else
            tab.flt.rc_line_silent = [0 0; t_loss 1; t_back 0];
        end
        t_on_set = t_loss + 0.01;
        tag = ['S6' var];
        crit(end+1) = local_c([tag '.v0'], sprintf('speed 0.1 s before the loss (target %.2f m/s)', v10), ...
            'truth.speed_fwd', @(S) M('at', S.truth.speed_fwd, t_loss - 0.1), '>=', 0.9*v10, 'm/s');
        crit(end+1) = local_c([tag '.ctrl_stop'], 'CTRL_STOP after the loss onset (D37 <= 0.13 s)', 'telem.mode', ...
            @(S) M('first', S.telem.mode, @(v) v == CTRL_STOP, t_loss) - t_on_set, '<=', 0.13, 's');
        crit(end+1) = local_c([tag '.decel'], 'max deceleration, 100 ms moving average', 'truth.speed_fwd', ...
            @(S) local_decel(S, t_loss, t_back), '<=', 1.05, 'm/s^2');
        crit(end+1) = local_c([tag '.standstill'], 'standstill after detection', 'truth.speed_fwd', ...
            @(S) local_standstill(S, t_loss, CTRL_STOP), '<=', 1.5, 's');
        crit(end+1) = local_c([tag '.no_estop'], 'ESTOP never entered (D34)', 'telem.mode', ...
            @(S) ~any(double(S.telem.mode.v) == ESTOP), 'true', 1, '');
        crit(end+1) = local_c([tag '.no_resume'], 'max |v| after the link returns until arm off -> on', 'truth.speed_fwd', ...
            @(S) max(abs(local_win(S.truth.speed_fwd, t_back, t_on + 0.05))), '<=', 0.02, 'm/s');
        crit(end+1) = local_c([tag '.resume'], sprintf('speed 2.5 s after re-arm + throttle (target %.2f m/s)', v10), ...
            'truth.speed_fwd', @(S) M('at', S.truth.speed_fwd, t_go + 2.5), '>=', 0.5*v10, 'm/s');
    case 'c'
        T = 27;
        t_e0 = 10; t_e1 = 25;
        tab.op.stick_thr = [0 0; 4 x10];
        tab.flt.rc_byte_err_rate = [0 0; t_e0 0.01; t_e1 0];
        crit(end+1) = local_c('S6c.v0', sprintf('speed during the byte errors at 20 s (target %.2f m/s)', v10), ...
            'truth.speed_fwd', @(S) M('at', S.truth.speed_fwd, 20), '>=', 0.9*v10, 'm/s');
        crit(end+1) = local_c('S6c.v_glitch', 'max |v_target - nominal| / v_max_man during the byte errors', ...
            'telem.v_target', @(S) local_glitch(S.telem.v_target, t_e0, t_e1, 1)/P.electronics.lim.v_max_man, ...
            '<=', 0.02, '1');
        crit(end+1) = local_c('S6c.w_glitch', 'max |w_target - nominal| / w_max_man during the byte errors', ...
            'telem.w_target', @(S) local_glitch(S.telem.w_target, t_e0, t_e1, 1)/P.electronics.lim.w_max_man, ...
            '<=', 0.02, '1');
        crit(end+1) = local_c('S6c.ch_glitch', 'max |rc_ch_norm(1:2) - nominal| (stick channels, FS 1)', ...
            'telem.rc_ch_norm', @(S) max([local_glitch(S.telem.rc_ch_norm, t_e0, t_e1, 1), ...
                                           local_glitch(S.telem.rc_ch_norm, t_e0, t_e1, 2)]), '<=', 0.02, '1');
        crit(end+1) = local_c('S6c.rejected', 'corrupted frames rejected (sbus_bad increase)', 'telem.sbus_bad', ...
            @(S) double(M('at', S.telem.sbus_bad, t_e1)) - double(M('at', S.telem.sbus_bad, t_e0)), '>', 0, 'frames');
        crit(end+1) = local_c('S6c.manual', 'MCU stays MANUAL during the byte errors', 'telem.mode', ...
            @(S) all(double(local_win(S.telem.mode, t_e0, t_e1)) == MANUAL), 'true', 1, '');
    case 'd'
        T = 27;
        t_e0 = 10; t_e1 = 25;
        tab.op.stick_thr = [0 0; 4 x10];
        tab.flt.rc_byte_err_rate = [0 0; t_e0 0.05; t_e1 0];
        crit(end+1) = local_c('S6d.ch_glitch', 'max |rc_ch_norm(1:2) - nominal| (stick channels, FS 1)', ...
            'telem.rc_ch_norm', @(S) max([local_glitch(S.telem.rc_ch_norm, t_e0, t_e1, 1), ...
                                           local_glitch(S.telem.rc_ch_norm, t_e0, t_e1, 2)]), '<=', 0.02, '1');
        crit(end+1) = local_c('S6d.rejected', 'corrupted frames rejected (sbus_bad increase)', 'telem.sbus_bad', ...
            @(S) double(M('at', S.telem.sbus_bad, t_e1)) - double(M('at', S.telem.sbus_bad, t_e0)), '>', 0, 'frames');
        crit(end+1) = local_c('S6d.ctrl_stop', 'CTRL_STOP after RC_LINK_LOST (telem.rc_link_ok false)', ...
            'telem.rc_link_ok, telem.mode', @(S) local_stop_after_loss(S, t_e0, CTRL_STOP), '<=', 0.2, 's');
        crit(end+1) = local_c('S6d.no_estop', 'ESTOP never entered (D34)', 'telem.mode', ...
            @(S) ~any(double(S.telem.mode.v) == ESTOP), 'true', 1, '');
    otherwise
        error('scn_S6_rc_failsafe:variant', 'Variant must be a, b, c or d');
end
if ~isempty(ip.Results.StopTime), T = ip.Results.StopTime; end
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, ['S6' var], 'StopTime', T, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T);
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function v = local_win(sig, t0, t1)
m = sig.t >= t0 & sig.t <= t1;
v = double(sig.v(m, 1));
end

function a = local_decel(S, t0, t1)
v = S.truth.speed_fwd;
m = v.t >= t0 - 0.5 & v.t <= t1;
t = v.t(m);  x = double(v.v(m, 1));
acc = [0; diff(x)./diff(t)];
a = max(-scn_metrics('movmean', t, acc, 0.1));
end

function d = local_standstill(S, t0, CTRL_STOP)
tc = scn_metrics('first', S.telem.mode, @(v) v == CTRL_STOP, t0);
ts = scn_metrics('first', S.truth.speed_fwd, @(v) abs(v) < 0.01, tc);
d = ts - tc;
end

function d = local_stop_after_loss(S, t0, CTRL_STOP)
% RC_LINK_LOST = first sample >= t0 with telem.rc_link_ok false; CTRL_STOP entry at or after it
tl = scn_metrics('first', S.telem.rc_link_ok, @(v) v == 0, t0);
if isnan(tl), d = NaN; return, end
tc = scn_metrics('first', S.telem.mode, @(v) v == CTRL_STOP, tl);
d = tc - tl;
end

function g = local_glitch(sig, t0, t1, col)
m = sig.t >= t0 & sig.t <= t1;
x = double(sig.v(m, col));
g = max(abs(x - median(x)));
end
