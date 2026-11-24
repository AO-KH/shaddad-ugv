function [op, s, ended] = row_driver_step(s, pose, v, mcu_mode, tank_L, prm)
%ROW_DRIVER_STEP  One step (prm.Ts) of the simulated RC row-test driver: what it sees -> sticks and switches.
%   [op, s, ended] = row_driver_step(s, pose, v, mcu_mode, tank_L, prm)
%     s         driver state: row_driver_init(prm) before the first step, then the s returned by the last step
%     pose      true robot pose [x y psi]: m, m, rad (x east, y north, psi from east, positive to the left)
%     v         true forward speed, m/s
%     mcu_mode  MCU mode, ShaddadMode code as a double (2 STANDBY, 3 MANUAL, 7 CTRL_STOP, 8 ESTOP, 9 FAULT, ...)
%     tank_L    true volume in the sprayer tank, L (impl_mech.tankLevel; 0 with any other implement)
%     prm       driver parameters, row_test_prm(P, R) (P.scen.rowdrv)
%     op        operator command Bus_OperatorCmd: the fields and order of P.scen.dash, all double
%     ended     1 once the run is over (phase ENDED), else 0
%   Spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, section 7. The driver acts only through the
%   transmitter. Field of row_test_rows: row i along +x from x = 0 to L_row at y = (i-1)*spacing, n_rows >= 2.
%
%   Phases (s.phase):
%     0 START    transmitter off, on from t_tx; sticks neutral, mode switch MANUAL, arm switch off
%     1 ARMING   arm switch on at t_arm; drive once the MCU is in MANUAL (3), ABORT after t_arm_wait without it
%     4 RETURN   on the headland to row 1 and into it, at v_turn: from the end of the last row on along its line
%                to the headland lane x_e + dir*headland/2, along the lane to row 1, into row 1 against dir. The
%                first drive after MANUAL is the last part of it: the start pose is on row 1's line, half a
%                headland before the row
%     2 ROW      along the row at v_row, v_turn within d_slow of its end; implement switch ON (2) with the
%                sprayer while the tank holds more than tank_empty_L, else OFF (1) in every phase: once the tank is
%                empty the pump stays off, as an operator who sees the spray stop switches it off (no latch: the
%                level never rises, there is no refill). Serpentine: row 1 east, row 2 west, ...; after the last
%                row RETURN, then row 1 again (the next pass, against the direction of the last row)
%     3 TURN     180 deg U-turn, radius spacing/2, beyond the row end into the next row, at v_turn
%     5 STOP     t_drive after MANUAL was first seen: sticks neutral until the robot has been below 0.05 m/s for
%                t_end_hold
%     6 DISARM   arm switch off for t_disarm
%     7 ENDED    ended = 1
%     8 ABORT    MCU out of MANUAL for t_abort while driving (ROW, TURN, RETURN), or no MANUAL t_arm_wait after
%                arming: sticks neutral, implement OFF, arm switch left as it is (no reset, no re-arm); ENDED 3 s
%                later
%   The driver sees the pose and speed n_delay samples late (reaction delay, ring buffer in s); it reads the MCU
%   mode and the tank volume as they come.
%   Steering: pure pursuit on the path of the phase (row line, U-turn arc, headland legs). The aim point lies
%   L = L_look*v_target/v_row (L_look at the row speed) along the path ahead of the robot's projection and runs
%   on into the next part of the path; curvature 2*sin(alpha)/d (alpha: direction of the aim point from the
%   heading, d: its distance), yaw rate target = curvature x seen speed. The speed and yaw rate targets become
%   sticks through the prm tables (inverse of the MCU RC mapping, so the MCU decodes the intended values; targets
%   clamped to the tables first: interp1 is NaN outside them); while driving the steering stick gets the
%   unsteadiness wobble*(sin(2*pi*f1*t) + sin(2*pi*f2*t + 1))/2. Both sticks move at most dx_max per step.
%   Code generation: no persistent variables (the MATLAB Function block keeps s), s keeps its fields and sizes.
%#codegen

% phase codes and fixed values
START = 0; ARMING = 1; ROW = 2; TURN = 3; RETURN = 4; STOP = 5; DISARM = 6; ENDED = 7; ABORT = 8;
MANUAL = 3;          % ShaddadMode.MANUAL
V_STOPPED = 0.05;    % m/s, below this the robot counts as stopped (STOP)
T_ABORT_END = 3;     % s, ABORT -> ENDED
EPS_T = 1e-6;        % s, timer comparisons on the sample grid

t = s.k*prm.Ts;
[s, seen] = local_seen(s, [pose(1), pose(2), pose(3), v]);
p = seen(1:2);
manual = mcu_mode == MANUAL;

%% phase transitions
switch s.phase
    case START
        if t >= prm.t_arm - EPS_T
            s.arm = 1;
            s = local_enter(s, ARMING, t);
        end
    case ARMING
        if manual
            s.t_man = t;
            s = local_enter(s, RETURN, t);     % route state from row_driver_init: into row 1, heading east
        elseif t - s.t_ph >= prm.t_arm_wait - EPS_T
            s = local_enter(s, ABORT, t);
        end
    case {ROW, TURN, RETURN}
        if manual
            s.t_off = -1;
        elseif s.t_off < 0
            s.t_off = t;
        end
        if s.t_off >= 0 && t - s.t_off >= prm.t_abort - EPS_T
            s = local_enter(s, ABORT, t);
        elseif t - s.t_man >= prm.t_drive - EPS_T
            s = local_enter(s, STOP, t);
        else
            s = local_advance(s, p, prm);
        end
    case STOP
        if seen(4) >= V_STOPPED
            s.t_slow = -1;
        elseif s.t_slow < 0
            s.t_slow = t;
        end
        if s.t_slow >= 0 && t - s.t_slow >= prm.t_end_hold - EPS_T
            s.arm = 0;
            s = local_enter(s, DISARM, t);
        end
    case DISARM
        if t - s.t_ph >= prm.t_disarm - EPS_T
            s = local_enter(s, ENDED, t);
        end
    case ABORT
        if t - s.t_ph >= T_ABORT_END - EPS_T
            s = local_enter(s, ENDED, t);
        end
    otherwise   % ENDED
end

%% sticks: targets while driving, neutral otherwise; rate limited
thr = 0;
steer = 0;
if s.phase == ROW || s.phase == TURN || s.phase == RETURN
    v_t = prm.v_turn;
    if s.phase == ROW
        el = local_element(s.phase, s.row, s.dir, s.leg, prm);
        if el(6) - local_along(el, p) > prm.d_slow
            v_t = prm.v_row;
        end
    end
    q = local_aim(s, p, prm.L_look*v_t/prm.v_row, prm);
    dx = q(1) - p(1);
    dy = q(2) - p(2);
    d = hypot(dx, dy);
    al = atan2(dy, dx) - seen(3);
    al = atan2(sin(al), cos(al));            % -pi .. pi
    curv = 0;
    if d > 1e-6
        if abs(al) < pi/2
            curv = 2*sin(al)/d;
        else
            curv = 2*sign(al)/d;             % aim point behind: full turn towards it
        end
    end
    w_t = curv*seen(4);
    thr = interp1(prm.thr_v, prm.thr_x, min(max(v_t, prm.thr_v(1)), prm.thr_v(end)));
    steer = interp1(prm.steer_w, prm.steer_x, min(max(w_t, prm.steer_w(1)), prm.steer_w(end))) + ...
        prm.wobble*(sin(2*pi*prm.f_wobble(1)*t) + sin(2*pi*prm.f_wobble(2)*t + 1))/2;
    steer = min(max(steer, -1), 1);
end
s.thr = s.thr + min(max(thr - s.thr, -prm.dx_max), prm.dx_max);
s.steer = s.steer + min(max(steer - s.steer, -prm.dx_max), prm.dx_max);

%% operator command (Bus_OperatorCmd, field order of P.scen.dash)
impl_on = s.phase == ROW && prm.sprayer > 0.5 && tank_L > prm.tank_empty_L;   % pump: along the rows, until empty
op = struct('stick_thr', s.thr, 'stick_steer', s.steer, 'stick_aux_lv', 0, 'stick_aux_lh', 0, 'knob_speed', 1, ...
    'sw_mode', 1, 'sw_arm', s.arm, 'sw_impl', 1 + double(impl_on), 'sw_lift', 2, ...
    'sw_lights', 0, 'sw_estop', 0, 'btn_reset', 0, 'sw_mission', 1, 'btn_rth', 0, ...
    'tx_power_on', double(t >= prm.t_tx - EPS_T));
ended = double(s.phase == ENDED);
s.k = s.k + 1;
end

% ------------------------------------------------------------------------------------------------
function [s, seen] = local_seen(s, cur)
% reaction delay: write what is there now, read what was there n_delay steps ago (n_delay + 1 rows)
n = size(s.buf, 1);
if s.k == 0
    s.buf = repmat(cur, n, 1);               % before the first step: as it is now
end
s.buf(s.ib, :) = cur;
s.ib = mod(s.ib, n) + 1;
seen = s.buf(s.ib, :);
end

function s = local_enter(s, phase, t)
s.phase = phase;
s.t_ph = t;
end

function s = local_advance(s, p, prm)
% go on to the next part of the path once the robot has passed the end of the current one
for it = 1:4
    el = local_element(s.phase, s.row, s.dir, s.leg, prm);
    [phase, row, dir, leg] = local_next(s.phase, s.row, s.dir, s.leg, prm);
    nx = local_element(phase, row, dir, leg, prm);
    if el(1) == 2
        passed = local_along(el, p) > el(6);                   % arc: beyond its end
    else
        e = el(2:3) + el(6)*el(4:5);                           % line: beyond the bisector of the corner at its end
        b = el(4:5) + nx(4:5);                                 % (straight on or into an arc: beyond the end normal)
        passed = (p(1) - e(1))*b(1) + (p(2) - e(2))*b(2) > 0;
    end
    if ~passed
        break
    end
    s.phase = phase;
    s.row = row;
    s.dir = dir;
    s.leg = leg;
end
end

function q = local_aim(s, p, L, prm)
% aim point: L along the path ahead of the robot's projection on the current part, on into the next parts
phase = s.phase; row = s.row; dir = s.dir; leg = s.leg;
el = local_element(phase, row, dir, leg, prm);
a = local_along(el, p) + L;
for it = 1:4
    if a <= el(6)
        break
    end
    a = a - el(6);
    [phase, row, dir, leg] = local_next(phase, row, dir, leg, prm);
    el = local_element(phase, row, dir, leg, prm);
end
q = local_point(el, a);
end

function [phase, row, dir, leg] = local_next(phase, row, dir, leg, prm)
% the part of the path after (phase, row, dir, leg); phase codes 2 ROW, 3 TURN, 4 RETURN
if phase == 2                               % ROW: U-turn into the next row, after the last row the return
    if row < prm.n_rows
        phase = 3;
    else
        phase = 4;
        leg = 1;
    end
elseif phase == 3                            % TURN: the next row, the other way
    phase = 2;
    row = row + 1;
    dir = -dir;
elseif leg < 3                               % RETURN: the next leg
    leg = leg + 1;
else                                         % RETURN done: row 1 against the last row (next pass)
    phase = 2;
    row = 1;
    dir = -dir;
end
end

function el = local_element(phase, row, dir, leg, prm)
% part of the path (phase codes 2 ROW, 3 TURN, 4 RETURN) as [kind ax ay ux uy len r sg]
%   kind 1: line from (ax, ay) along the unit vector (ux, uy), length len
%   kind 2: half circle of radius r around (ax, ay), turning sg (+1 left, -1 right); (ux, uy) = unit vector from
%           the centre to the middle of the arc = direction at its start; len = pi*r
xe = prm.L_row*(dir > 0);                    % end of a row driven in direction dir
if phase == 3                                % TURN out of row 'row' into row + 1 (north)
    r = prm.spacing/2;
    el = [2, xe, (row - 1)*prm.spacing + r, dir, 0, pi*r, r, dir];
elseif phase == 4                            % RETURN after the last row (driven in direction dir)
    xl = xe + dir*prm.headland/2;            % headland lane
    yn = (prm.n_rows - 1)*prm.spacing;
    if leg == 1
        el = [1, xe, yn, dir, 0, prm.headland/2, 0, 0];
    elseif leg == 2
        el = [1, xl, yn, 0, -1, yn, 0, 0];
    else
        el = [1, xl, 0, -dir, 0, prm.headland/2, 0, 0];
    end
else                                         % ROW: the row line from its start to its end
    el = [1, prm.L_row - xe, (row - 1)*prm.spacing, dir, 0, prm.L_row, 0, 0];
end
end

function a = local_along(el, p)
% position of the projection of p on the part, measured along it from its start (negative before it)
rx = p(1) - el(2);
ry = p(2) - el(3);
if el(1) == 2
    % arc: r*(phi + pi/2), phi = angle of p around the centre from the middle of the arc in the turning sense
    % (-pi/2 at the start, +pi/2 at the end)
    a = el(7)*(atan2(el(8)*(el(4)*ry - el(5)*rx), el(4)*rx + el(5)*ry) + pi/2);
else
    a = rx*el(4) + ry*el(5);
end
end

function q = local_point(el, a)
% point at a along the part (lines and arcs go on beyond their ends)
if el(1) == 2
    ph = a/el(7) - pi/2;                     % centre + r*(cos(ph)*u + sin(ph)*sg*[-uy ux])
    q = [el(2) + el(7)*(cos(ph)*el(4) - sin(ph)*el(8)*el(5)), el(3) + el(7)*(cos(ph)*el(5) + sin(ph)*el(8)*el(4))];
else
    q = [el(2) + a*el(4), el(3) + a*el(5)];
end
end
