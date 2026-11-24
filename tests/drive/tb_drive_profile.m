function [cmd, pr] = tb_drive_profile(o)
%TB_DRIVE_PROFILE  MCU-like DRV_CMD sequence (Bus_DriveSideCmd timeseries) for the T06 drive benches.
%   [cmd, pr] = tb_drive_profile(o), fields of o (all optional):
%     t_end (s, 5), Ts (s, P.drive.Ts_can)
%     v_tgt      [t v] table of track-speed targets (m/s), held between rows (default [0 0])
%     shaped     true: MCU shaper (K_v, a_acc, a_dec, j_max of P.electronics.shaper, see build plan T09.3);
%                false: unshaped steps of the target
%     a_acc, a_dec, j_max, K_v   shaper overrides
%     stop       [t_start a_stop j_stop]: from t_start the constant-deceleration stop law of D33
%                a = -sign(v)*min(a_stop, sqrt(2*j_stop*|v|)) (jerk-limited by j_stop) to zero
%     Kff        torque feed-forward per track acceleration (N m per m/s^2, P.electronics.drv.Kff_a); 0 = off
%     enable     [t value] table (default [0 1]); mode (1); tau_lim_mot, tau_lim_reg [t value] tables (P.drive.motor.T_peak)
%     tau_ff_tab [t value] table added to the feed-forward (TORQUE-mode setpoints)
%     park_req, fault_reset  [t value] tables (0)
%     alive_freeze  time from which the alive counter is frozen (Inf)
%     w_ref_tab  [t w] motor-speed table (rad/s) used instead of v_tgt when given (no shaping)
%   pr: .t .v_ref .a_ref .w_ref .tau_ff (double, at Ts)
P = evalin('base', 'P');
if nargin < 1, o = struct(); end
d = struct('t_end', 5, 'Ts', P.drive.Ts_can, 'v_tgt', [0 0], 'shaped', true, ...
    'a_acc', P.electronics.shaper.a_acc_man, 'a_dec', P.electronics.shaper.a_dec_man, ...
    'j_max', P.electronics.shaper.j_max, 'K_v', P.electronics.shaper.K_v, 'stop', [], ...
    'Kff', P.electronics.drv.Kff_a, 'enable', [0 1], 'mode', 1, 'tau_lim_mot', [0 P.drive.motor.T_peak], 'tau_lim_reg', [0 P.drive.motor.T_peak], ...
    'tau_ff_tab', [0 0], 'park_req', [0 0], 'fault_reset', [0 0], 'alive_freeze', inf, 'w_ref_tab', []);
fn = fieldnames(o);
for k = 1:numel(fn), d.(fn{k}) = o.(fn{k}); end
o = d;
t = (0:o.Ts:o.t_end)';
n = numel(t);
tab = @(T) local_hold(T, t);
k_v2w = P.drive.gear.k_v2w;
v_ref = zeros(n, 1); a_ref = zeros(n, 1);
if ~isempty(o.w_ref_tab)
    w_ref = tab(o.w_ref_tab);
    v_ref = w_ref/k_v2w;
    a_ref = [0; diff(v_ref)]/o.Ts;
    if ~o.shaped, a_ref(:) = 0; end
else
    vt = tab(o.v_tgt);
    v = 0; a = 0;
    for i = 1:n
        if ~isempty(o.stop) && t(i) >= o.stop(1)
            ad = -sign(v)*min(o.stop(2), sqrt(2*o.stop(3)*abs(v)));
            a = a + max(-o.stop(3)*o.Ts, min(o.stop(3)*o.Ts, ad - a));
            vn = v + a*o.Ts;
            if sign(vn) ~= sign(v) || v == 0, vn = 0; a = 0; end
            v = vn;
        elseif o.shaped
            ad = max(-o.a_dec, min(o.a_acc, o.K_v*(vt(i) - v)));
            a = a + max(-o.j_max*o.Ts, min(o.j_max*o.Ts, ad - a));
            v = v + a*o.Ts;
        else
            v = vt(i); a = 0;
        end
        v_ref(i) = v; a_ref(i) = a;
    end
    w_ref = k_v2w*v_ref;
end
tau_ff = o.Kff*a_ref + tab(o.tau_ff_tab);
alive = mod(0:n - 1, 256)';
if isfinite(o.alive_freeze)
    i0 = find(t >= o.alive_freeze, 1);
    if ~isempty(i0), alive(i0:end) = alive(max(i0 - 1, 1)); end
end
vals = struct('enable', tab(o.enable), 'mode', o.mode, 'w_ref', w_ref, 'tau_ff', tau_ff, ...
    'tau_lim_mot', tab(o.tau_lim_mot), 'tau_lim_reg', tab(o.tau_lim_reg), 'park_req', tab(o.park_req), ...
    'fault_reset', tab(o.fault_reset), 'alive', alive);
cmd = tb_drive_bus_ts('Bus_DriveSideCmd', t, vals);
pr = struct('t', t, 'v_ref', v_ref, 'a_ref', a_ref, 'w_ref', w_ref, 'tau_ff', tau_ff);
end

function y = local_hold(T, t)
% zero-order-hold table lookup: T = [t_i v_i] rows
if isempty(T), y = zeros(size(t)); return, end
if size(T, 1) == 1, y = repmat(T(1, 2), size(t)); return, end
y = zeros(size(t));
for i = 1:numel(t)
    j = find(T(:, 1) <= t(i) + 1e-12, 1, 'last');
    if isempty(j), j = 1; end
    y(i) = T(j, 2);
end
end
