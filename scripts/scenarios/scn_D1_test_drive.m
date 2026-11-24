function [in, crit] = scn_D1_test_drive(P, varargin)
%SCN_D1_TEST_DRIVE  User Phase-1 test drive: SBUS remote-control driving from a simple maneuver plan.
%
%   [in, crit] = scn_D1_test_drive(P, 'Plan', plan, ...)   (normally called through phase1_test_drive)
%
%   'Plan'      N x 3 [t  v  w]: from time t [s] hold speed v [m/s] (+ forward) and yaw rate w [rad/s]
%               (+ = left / counter-clockwise). Converted to transmitter stick positions with scn_rc_stick,
%               so the MCU RC mapping decodes exactly v and w. Limits: |v| <= P.electronics.lim.v_max_man,
%               |w| <= P.electronics.lim.w_max_man.
%   'EStopAt'   time [s] the remote E-stop switch (CH12) is thrown; it returns to RUN 1 s before the reset
%   'ResetAt'   time [s] the reset button (CH9) is pressed; the arm switch is cycled 2-3 s later
%   'RCLoss'    [t_loss t_back]: RF link lost / restored; the arm switch is cycled 4-5 s after t_back
%   'World'     world id (default 1 = flat test field); 'Implement' 0 none (default), 1 mower, 2 tiller, 3 sprayer
%   'StopTime'  default: last event + 8 s
%
%   Sticks are forced to neutral from an E-stop / RC loss until 1 s after the re-arm, as the MCU only
%   re-arms with neutral sticks (ICD 7). Transmitter on at 0.5 s, armed at 2 s (scn_rc_tab).

ip = inputParser;
ip.addParameter('Plan', [0 0 0; 4 0.8 0; 14 0.8 0.3; 24 1.2 0; 34 0.5 -0.4; 44 0 0.5; 50 -0.4 0; 56 0 0]);
ip.addParameter('EStopAt', []);
ip.addParameter('ResetAt', []);
ip.addParameter('RCLoss', []);
ip.addParameter('World', 1);
ip.addParameter('Implement', 0);
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
o = ip.Results;
plan = o.Plan;
if size(plan, 2) ~= 3, error('scn_D1:plan', 'Plan must be N x 3 [t v w]'); end
plan = sortrows(plan, 1);
if plan(1, 1) > 0, plan = [0 0 0; plan]; end

P = shaddad_params('World', o.World, 'Implement', o.Implement, 'Phase', 1, 'Assign', false);
L = P.electronics.lim;
if any(abs(plan(:, 2)) > L.v_max_man) || any(abs(plan(:, 3)) > L.w_max_man)
    warning('scn_D1:limit', 'Plan clipped to |v| <= %.2f m/s and |w| <= %.2f rad/s', L.v_max_man, L.w_max_man);
    plan(:, 2) = max(min(plan(:, 2), L.v_max_man), -L.v_max_man);
    plan(:, 3) = max(min(plan(:, 3), L.w_max_man), -L.w_max_man);
end

%% ---- sticks from the plan ----
thr = zeros(size(plan, 1), 1);  steer = thr;
for k = 1:size(plan, 1)
    if plan(k, 2) ~= 0, thr(k) = scn_rc_stick(P, plan(k, 2)); end
    if plan(k, 3) ~= 0, steer(k) = scn_rc_stick(P, plan(k, 3), 'steer'); end
end
tab = scn_rc_tab(0.5, 2);
tab.op.stick_thr = [plan(:, 1) thr];
tab.op.stick_steer = [plan(:, 1) steer];
arm = [0 0; 2 1];
t_last = plan(end, 1);

%% ---- remote E-stop + reset ----
if ~isempty(o.EStopAt)
    t_e = o.EStopAt;
    t_r = o.ResetAt;
    if isempty(t_r)
        tab.op.sw_estop = [0 0; t_e 1];
        hold_to = inf;
    else
        if t_r < t_e + 1.5, error('scn_D1:reset', 'ResetAt must be >= EStopAt + 1.5 s'); end
        tab.op.sw_estop = [0 0; t_e 1; t_r - 1 0];
        tab.op.btn_reset = [0 0; t_r 1; t_r + 0.3 0];
        arm = [arm; t_r + 2 0; t_r + 3 1];
        hold_to = t_r + 4;
    end
    tab.op.stick_thr = local_neutral(tab.op.stick_thr, t_e, hold_to);
    tab.op.stick_steer = local_neutral(tab.op.stick_steer, t_e, hold_to);
    t_last = max([t_last, t_e, t_r + 4]);
end

%% ---- RF link loss ----
if ~isempty(o.RCLoss)
    t1 = o.RCLoss(1);  t2 = o.RCLoss(2);
    tab.flt.rc_link_loss = [0 0; t1 1; t2 0];
    arm = [arm; t2 + 4 0; t2 + 5 1];
    tab.op.stick_thr = local_neutral(tab.op.stick_thr, t1, t2 + 6);
    tab.op.stick_steer = local_neutral(tab.op.stick_steer, t1, t2 + 6);
    t_last = max(t_last, t2 + 6);
end
tab.op.sw_arm = sortrows(arm, 1);

T = o.StopTime;
if isempty(T), T = t_last + 8; end
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', o.Implement, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, 'D1', 'StopTime', T, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T);
crit = [];                                   % free test drive: no pass/fail criteria
end

% -------------------------------------------------------------------------------------------
function z = local_neutral(z, ta, tb)
% Hold a zero-order-hold [t value] table at 0 on [ta, tb), then resume the planned value.
v_after = z(find(z(:, 1) <= tb, 1, 'last'), 2);
keep = z(:, 1) < ta | z(:, 1) > tb;
z = [z(keep, :); ta 0];
if isfinite(tb), z = [z; tb v_after]; end
z = sortrows(z, 1);
end
