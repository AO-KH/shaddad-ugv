function prm = row_test_prm(P, R)
%ROW_TEST_PRM  Parameters of the simulated row-test driver (parameter of the RowDriver MATLAB Function block).
%   prm = row_test_prm(P, R)     P = shaddad_params('Assign', false) (sample time, RC limits and mapping)
%                                R = param_rowtest() or P.scen.rowtest
%   Spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, section 7. prm is a struct of doubles with the same
%   fields and sizes for every input (D58: P is numeric; a MATLAB Function block takes a struct of fixed-size doubles):
%     n_rows sprayer L_row spacing headland v_row v_turn d_slow L_look t_react t_stick wobble f_wobble
%     t_tx t_arm t_arm_wait t_abort t_end_hold t_disarm tank_empty_L   the driving fields of R, copied
%     Ts         driver sample time = P.sim.Ts_scen, s
%     n_delay    reaction delay = round(t_react/Ts), samples
%     dx_max     largest stick step per sample = Ts/t_stick (a full deflection, 1 stick unit, takes t_stick)
%     t_drive    driving time from the moment the robot is armed = 60*minutes, s
%     start      start pose [x y psi] = [-headland/2 0 0] (half a headland before row 1, heading east), m, rad
%     thr_v, thr_x       21 points: speed 0 .. v_max_man (m/s) and the throttle stick that makes the MCU decode it
%     steer_w, steer_x   41 points: yaw rate -w_max_man .. +w_max_man (rad/s) and the steering stick for it
%   The sticks are those of scn_rc_stick (inverse of the MCU RC mapping: dead zone, expo, TX raw grid). Both tables
%   are sorted by the target (speed, yaw rate) and strictly monotonic. interp1(thr_v, thr_x, v) and
%   interp1(steer_w, steer_x, w) give the stick for a target; interp1(thr_x, thr_v, x) and
%   interp1(steer_x, steer_w, x) the target a stick gives. The steering stick falls as the yaw rate rises
%   (P.electronics.rcmap.steer_to_omega_sign = -1); interp1 takes a strictly decreasing x, also in generated code.
%   The end points lie one TX raw count beyond the stick travel (1.0006): the TX clips them to full stick.
%   The tables are linear between the points, so the dead zone (+-5 % of the stick) is only approximated below the
%   first non-zero point (v_max_man/20, w_max_man/20): a smaller target gives a stick that decodes to less.
%   interp1 returns NaN outside the tables: clamp the target or the stick first.

% driving fields of R, copied as doubles (sprayer may be logical)
f = {'n_rows', 'sprayer', 'L_row', 'spacing', 'headland', 'v_row', 'v_turn', 'd_slow', 'L_look', ...
    't_react', 't_stick', 'wobble', 'f_wobble', 't_tx', 't_arm', 't_arm_wait', 't_abort', 't_end_hold', 't_disarm', ...
    'tank_empty_L'};
prm = struct();
for k = 1:numel(f)
    prm.(f{k}) = double(R.(f{k}));
end

% sample time, reaction delay, stick rate limit, driving time, start pose
prm.Ts = double(P.sim.Ts_scen);
prm.n_delay = round(prm.t_react/prm.Ts);
prm.dx_max = prm.Ts/prm.t_stick;
prm.t_drive = 60*double(R.minutes);
prm.start = [-prm.headland/2, 0, 0];

% stick tables: target -> the stick whose decoded target is closest (scn_rc_stick)
lim = P.electronics.lim;
prm.thr_v = linspace(0, lim.v_max_man, 21);
prm.thr_x = arrayfun(@(v) scn_rc_stick(P, v), prm.thr_v);
prm.steer_w = linspace(-lim.w_max_man, lim.w_max_man, 41);
prm.steer_x = arrayfun(@(w) scn_rc_stick(P, w, 'steer'), prm.steer_w);
end
