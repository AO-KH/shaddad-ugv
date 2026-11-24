function s = row_driver_init(prm)
%ROW_DRIVER_INIT  Initial state of the simulated row-test driver (row_driver_step, RC row test).
%   s = row_driver_init(prm)     prm = row_test_prm(P, R) (P.scen.rowdrv)
%   Spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, section 7. s is a struct of doubles with the same
%   fields and sizes at every step (the RowDriver MATLAB Function block keeps it in a persistent variable;
%   row_driver_step has none). buf has n_delay + 1 rows: a fixed size when prm is a constant (non-tunable).
%     k        steps done; the next step is at t = k*Ts
%     phase    0 START, 1 ARMING, 2 ROW, 3 TURN, 4 RETURN, 5 STOP, 6 DISARM, 7 ENDED, 8 ABORT
%     t_ph     time the phase was entered, s (used by ARMING, DISARM and ABORT)
%     t_man    time MANUAL was first seen: the driving time t_drive starts here, s (-1 before)
%     t_off    time the MCU was first seen out of MANUAL while driving, s (-1 while MANUAL)
%     t_slow   time the speed was first seen below 0.05 m/s in STOP, s (-1 while faster)
%     row      row driven (ROW), turned out of (TURN) or driven last (RETURN), 1 .. n_rows
%     dir      direction of that row: +1 east (its end at x = L_row), -1 west (its end at x = 0)
%     leg      part of the RETURN: 1 on along the last row line to the headland lane, 2 along the lane to row 1,
%              3 from the lane into row 1
%     arm      arm switch position, 0/1
%     thr, steer   sticks sent at the last step (state of the rate limiter)
%     buf      ring buffer of what the driver saw, (n_delay + 1) x 4 [x y psi v]
%     ib       row of buf written next
%   The first drive after MANUAL is the end of a return to row 1 (leg 3) as if the last row had been driven west:
%   the start pose prm.start lies on that line, half a headland before row 1, heading east.
%#codegen

s = struct('k', 0, 'phase', 0, 't_ph', 0, 't_man', -1, 't_off', -1, 't_slow', -1, ...
    'row', prm.n_rows, 'dir', -1, 'leg', 3, 'arm', 0, 'thr', 0, 'steer', 0, ...
    'buf', zeros(prm.n_delay + 1, 4), 'ib', 1);
end
