function [in, crit] = scn_S8_estop(P, varargin)
%SCN_S8_ESTOP  S8 E-stop (Phase 1), architecture section 12 / D29 / D33 / D62 / ICD 6.
%
%   [in, crit] = scn_S8_estop(P, 'Variant', v)     v = 'a' (default, front hardware E-stop) | 'b' (remote CH12)
%   MANUAL drive at 1.2 m/s (transmitter on 0.5 s, arm 2 s, throttle 4 s), E-stop at t = 10 s.
%   (a) ScenarioIn.hmi.estop_front_pressed 10 -> 15 s (throttle still held until 16 s), reset button
%       17.0-17.3 s, arm off 19 s / on 20 s with the sticks neutral, throttle again at 21 s.
%       Criteria: STO (power_status.Safety_OK false) <= 20 ms after the plant-side hmi edge (Plant_In
%       output = ScenarioIn edge + 10 ms deterministic RT, D62); MCU fault bit HW_ESTOP <= 11 ms after the
%       DI change seen by the MCU (elec_hw di_word b0); stop distance <= 0.6 m; no motion (|v| <= 0.02 m/s)
%       from 2 s after ESTOP entry (chassis rocking after the hard stop has decayed) until reset + re-arm
%       (20.05 s, before the throttle at 21 s).
%   (b) op.sw_estop (CH12) 10 -> 15 s. Criteria: mean deceleration over the constant-deceleration phase
%       2.0 +/- 0.1 m/s^2 (100 ms moving-average peak <= 2.4); stop distance from the CH12 switch <= 0.70 m
%       (D33 replay 0.62 m); ESTOP latched after the switch returns to RUN.

ip = inputParser;
ip.addParameter('Variant', 'a');
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
var = lower(char(ip.Results.Variant));
if P.world.id ~= 1
    P = shaddad_params('World', 1, 'Implement', 0, 'Phase', 1, 'Assign', false);
end
[x12, v12] = scn_rc_stick(P, 1.2);
tab = scn_rc_tab(0.5, 2);
M = @scn_metrics;
ESTOP = 8;
t_e = 10;
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
% precondition: the robot really drives at 1.2 m/s when the E-stop comes (stop criteria are void otherwise)
crit(end+1) = local_c(['S8' var '.v0'], sprintf('speed 0.1 s before the E-stop (target %.2f m/s)', v12), ...
    'truth.speed_fwd', @(S) M('at', S.truth.speed_fwd, t_e - 0.1), '>=', 0.9*v12, 'm/s');
switch var
    case 'a'
        T = 24;
        tab.op.stick_thr = [0 0; 4 x12; 16 0; 21 x12];
        tab.op.sw_arm = [0 0; 2 1; 19 0; 20 1];
        tab.hmi.estop_front_pressed = [0 0; t_e 1; 15 0];
        tab.hmi.reset_btn = [0 0; 17 1; 17.3 0];
        t_plant = t_e + 0.01;                % plant-side hmi edge (Plant_In RT 0.01 -> 0.001, D62)
        crit(end+1) = local_c('S8a.sto', 'STO (Safety_OK false) after the plant-side hmi edge', ...
            'power_status.Safety_OK', @(S) M('first', S.power_status.Safety_OK, @(v) v == 0, t_e) - t_plant, ...
            '<=', 0.020, 's');
        crit(end+1) = local_c('S8a.hw_estop', 'fault bit HW_ESTOP after the DI change at the MCU (di_word b0)', ...
            'elec_hw.di_word, telem.fault_word', @(S) local_hw_estop(S, t_e), '<=', 0.011, 's');
        crit(end+1) = local_c('S8a.stop_dist', 'stop distance from the plant-side hmi edge', 'truth.pos_W', ...
            @(S) M('path', S.truth.pos_W, t_plant, 15), '<=', 0.6, 'm');
        crit(end+1) = local_c('S8a.no_restart', 'max |v| from ESTOP entry + 2 s until reset + re-arm (20 s)', ...
            'truth.speed_fwd, telem.mode', @(S) local_no_restart(S, t_e, ESTOP, 2, 20.05), '<=', 0.02, 'm/s');
        crit(end+1) = local_c('S8a.resume', sprintf('speed 2.5 s after re-arm + throttle (target %.2f m/s)', v12), ...
            'truth.speed_fwd', @(S) M('at', S.truth.speed_fwd, 23.5), '>=', 0.5*v12, 'm/s');
    case 'b'
        T = 18;
        tab.op.stick_thr = [0 0; 4 x12; 16 0];
        tab.op.sw_estop = [0 0; t_e 1; 15 0];
        crit(end+1) = local_c('S8b.decel_mean', 'mean deceleration over the constant-deceleration phase (90 % -> 10 %)', ...
            'truth.speed_fwd', @(S) local_decel_mean(S, t_e), 'in', [1.9 2.1], 'm/s^2');
        crit(end+1) = local_c('S8b.decel_peak', 'deceleration, 100 ms moving-average peak', 'truth.speed_fwd', ...
            @(S) local_decel_peak(S, t_e), '<=', 2.4, 'm/s^2');
        crit(end+1) = local_c('S8b.stop_dist', 'stop distance from the CH12 switch (ScenarioIn edge)', 'truth.pos_W', ...
            @(S) M('path', S.truth.pos_W, t_e, 15), '<=', 0.70, 'm');
        crit(end+1) = local_c('S8b.latched', 'ESTOP latched after CH12 returns to RUN (mode at 17.9 s)', 'telem.mode', ...
            @(S) double(M('at', S.telem.mode, 17.9)), '==', ESTOP, '');
        crit(end+1) = local_c('S8b.estop_entered', 'ESTOP entered after the CH12 switch', 'telem.mode', ...
            @(S) M('first', S.telem.mode, @(v) v == ESTOP, t_e) - t_e, '<=', 0.2, 's');
    otherwise
        error('scn_S8_estop:variant', 'Variant must be a or b');
end
if ~isempty(ip.Results.StopTime), T = ip.Results.StopTime; end
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, ['S8' var], 'StopTime', T, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T);
end

% -------------------------------------------------------------------------------------------
function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function L = local_hw_estop(S, t0)
t_di = scn_metrics('first', S.elec_hw.di_word, @(v) ~scn_metrics('bit', v, 0), t0);
t_f = scn_metrics('first', S.telem.fault_word, @(v) scn_metrics('bit', v, 0), t0);
L = t_f - t_di;
end

function m = local_no_restart(S, t0, ESTOP, dt, t1)
% window [ESTOP entry + dt, t1]: the first |v| < 0.01 after the hard brake stop is a zero crossing of the
% chassis rocking (pitch / track compliance, decays in about 1.5 s), not the standstill, so the window
% starts dt after the MCU entered ESTOP and ends at reset + re-arm (before the throttle returns)
te = scn_metrics('first', S.telem.mode, @(u) u == ESTOP, t0);
v = S.truth.speed_fwd;
w = v.t >= te + dt & v.t <= t1;
if isnan(te) || ~any(w), m = NaN; return, end
m = max(abs(double(v.v(w, 1))));
end

function [t, a] = local_acc(S, t0)
v = S.truth.speed_fwd;
m = v.t >= t0 - 0.5 & v.t <= t0 + 3;
t = v.t(m);  x = double(v.v(m, 1));
a = -[0; diff(x)./diff(t)];
end

function a = local_decel_mean(S, t0)
v = S.truth.speed_fwd;
v0 = scn_metrics('at', v, t0);
if v0 < 0.1, a = NaN; return, end            % not moving: no deceleration phase
t1 = scn_metrics('first', v, @(u) u <= 0.9*v0, t0);
t2 = scn_metrics('first', v, @(u) u <= 0.1*v0, t1);
a = 0.8*v0/(t2 - t1);
end

function p = local_decel_peak(S, t0)
[t, a] = local_acc(S, t0);
p = max(scn_metrics('movmean', t, a, 0.1));
end
