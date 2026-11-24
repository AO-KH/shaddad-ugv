function B = row_driver_bench(prm, T, opt)
%ROW_DRIVER_BENCH  Kinematic bench of the simulated row-test driver: a MATLAB loop, no Simulink (RC row test).
%   B = row_driver_bench(prm, T)          prm = row_test_prm(P, R); T = simulated time, s
%   B = row_driver_bench(prm, T, opt)     opt fields, each optional:
%     slip      the robot turns at slip x the decoded yaw rate (track slip in the turns), default 0.9
%     fault_at  time from which the MCU is in FAULT (9), s; default Inf (no fault)
%     tank0     sprayer tank volume at the start, L; default Inf (a tank that never empties: the bench as it was
%               before it had a tank)
%     drain     liquid sprayed while the implement switch is ON, L/s; default 4.8/60 (8 nozzles at the set pressure)
%   Spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, section 11. Every step at prm.Ts (t = 0 .. T):
%   row_driver_step(true pose, true speed, MCU mode, tank volume) -> MCU model -> vehicle and tank.
%     MCU model: STANDBY (2) until the arm switch rises while the transmitter is on, the mode switch is MANUAL (1)
%       and both sticks are neutral (|stick| <= 0.08, P.electronics.rcmap.neutral_tol); then MANUAL (3); arm
%       switch off -> STANDBY. Only MANUAL drives: speed v_c = interp1(thr_x, thr_v, thr) and yaw rate
%       w_c = slip*interp1(steer_x, steer_w, steer) (the MCU RC mapping as tabulated in prm, sticks clamped to the
%       tables); in any other mode v_c = w_c = 0.
%     Vehicle: unicycle starting at prm.start, first-order lags 0.3 s (speed) and 0.15 s (yaw rate).
%     Tank: drain x Ts less after every step with the implement switch ON (op.sw_impl 2), down to 0; no refill.
%   B: n x 1 per step (n x 3 for pose): t; pose [x y psi], v and tank, the true values the driver gets at that step;
%      phase (s.phase after the step); thr, steer, arm, impl, tx (op.stick_thr, op.stick_steer, op.sw_arm,
%      op.sw_impl, op.tx_power_on); ended_t: time of the first step with ended = 1, s (NaN if none).

if nargin < 3, opt = struct(); end
slip = 0.9;
if isfield(opt, 'slip'), slip = opt.slip; end
fault_at = Inf;
if isfield(opt, 'fault_at'), fault_at = opt.fault_at; end
tank = Inf;
if isfield(opt, 'tank0'), tank = opt.tank0; end
drain = 4.8/60;
if isfield(opt, 'drain'), drain = opt.drain; end
TAU_V = 0.3;  TAU_W = 0.15;   % s, speed and yaw-rate lags
NEUTRAL = 0.08;               % stick, P.electronics.rcmap.neutral_tol

Ts = prm.Ts;
n = round(T/Ts) + 1;
B = struct('t', (0:n - 1)'*Ts, 'pose', zeros(n, 3), 'v', zeros(n, 1), 'tank', zeros(n, 1), 'phase', zeros(n, 1), ...
    'thr', zeros(n, 1), 'steer', zeros(n, 1), 'arm', zeros(n, 1), 'impl', zeros(n, 1), 'tx', zeros(n, 1), ...
    'ended_t', NaN);
x_thr = [min(prm.thr_x), max(prm.thr_x)];
x_steer = [min(prm.steer_x), max(prm.steer_x)];
a_v = 1 - exp(-Ts/TAU_V);
a_w = 1 - exp(-Ts/TAU_W);

s = row_driver_init(prm);
pose = prm.start(:)';
v = 0;
w = 0;
mode = 2;
arm_last = 0;
for i = 1:n
    t = B.t(i);
    [op, s, ended] = row_driver_step(s, pose, v, mode, tank, prm);
    B.pose(i, :) = pose;
    B.v(i) = v;
    B.tank(i) = tank;
    B.phase(i) = s.phase;
    B.thr(i) = op.stick_thr;
    B.steer(i) = op.stick_steer;
    B.arm(i) = op.sw_arm;
    B.impl(i) = op.sw_impl;
    B.tx(i) = op.tx_power_on;
    if ended > 0.5 && isnan(B.ended_t)
        B.ended_t = t;
    end
    % MCU mode
    if t >= fault_at
        mode = 9;
    elseif mode == 2 && op.sw_arm > 0.5 && arm_last < 0.5 && op.tx_power_on > 0.5 && op.sw_mode == 1 && ...
            abs(op.stick_thr) <= NEUTRAL && abs(op.stick_steer) <= NEUTRAL
        mode = 3;
    elseif mode == 3 && op.sw_arm < 0.5
        mode = 2;
    end
    arm_last = op.sw_arm;
    % vehicle
    v_c = 0;
    w_c = 0;
    if mode == 3
        v_c = interp1(prm.thr_x, prm.thr_v, min(max(op.stick_thr, x_thr(1)), x_thr(2)));
        w_c = slip*interp1(prm.steer_x, prm.steer_w, min(max(op.stick_steer, x_steer(1)), x_steer(2)));
    end
    v = v + a_v*(v_c - v);
    w = w + a_w*(w_c - w);
    pose = pose + Ts*[v*cos(pose(3)), v*sin(pose(3)), w];
    % tank
    if op.sw_impl > 1.5
        tank = max(tank - drain*Ts, 0);
    end
end
end
