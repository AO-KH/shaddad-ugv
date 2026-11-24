function [state, torque_en, brake_rel, duty, hold0, qs, use_speed, fault_word, brake_released] = ...
    esc_sm(enable, mode, w_ref, tau_ff, park_req, reset_edge, sto_ok, v_dc, can_to, fault_cond, w_meas, smv)
%ESC_SM  DS402-like state machine of the traction ESC (drive spec 2.9, build plan T06.2, D1).
%   Called in one line from the MATLAB Function block drive_esc/ESC_SM at Ts_esc = 1 ms.
%   Stateflow is not installed (D1): this is an ordered transition table with persistent state
%   that maps 1:1 to an sflib/Chart. Timers are 1 ms tick counts.
%   smv : parameter vector P.drive.sm_vec on an input port (Constant block), not mask-resolved
%         parameter data: avoids the R2026b JIT relink assertion 'o != nullptr' with MATLAB Function
%         blocks nested several Subsystem Reference levels deep. Layout (param_drive.m, P.drive.sm):
%         1 n_init 2 n_premag 3 n_release 4 n_engage 5 n_stop_hold 6 n_park 7 n_pullin 8 n_reset_win
%         9 V_on 10 V_off 11 w_stop 12 w_park_ref 13 tau_park_ref 14 duty_hold 15 hard_mask 16 n_stop_max
%
%   States (enum EscState, output as uint8):
%     0 INIT        torque off, brake engaged      -> NOT_READY after n_init
%     1 NOT_READY   torque off, brake engaged      -> READY when v_dc > V_on (and sto_ok)
%     2 READY       torque off, brake engaged      -> ENABLING on enable, mode SPEED/TORQUE, no fault,
%                                                     no CAN timeout, no park_req
%     3 ENABLING    torque on (hold w = 0), brake released at n_premag -> OPERATION after
%                   n_premag + n_release
%     4 OPERATION   torque on, brake released; substate PARKED (w_ref = 0 and |w| < w_stop for
%                   n_park: brake engaged, torque off after n_engage; a non-zero command -> ENABLING);
%                   !enable or park_req -> DISABLING; CAN timeout -> QUICK_STOP
%     5 DISABLING   ramp to 0; brake engaged at once on park_req, whatever the speed (ICD: force brake
%                   engaged and torque off); without park_req "stop first, then engage": brake engaged after
%                   |w| < w_stop for n_stop_hold, or n_stop_max after the stop began whatever the speed;
%                   torque off n_engage after the brake -> READY; enable again -> OPERATION (brake still
%                   released) / ENABLING
%     6 QUICK_STOP  ramp at a_qs, brake engaged at standstill, at once on park_req, or n_stop_max after the
%                   stop began whatever the speed; n_engage later -> FAULT (CAN_TIMEOUT latched)
%   Stop time-out (critfix R-01): a stop begins on entry to DISABLING or QUICK_STOP from another state;
%   DISABLING -> QUICK_STOP continues the same stop, so its time-out keeps counting.
%     7 FAULT       torque off immediately, brake engaged -> READY on a fault_reset edge without active fault
%     8 STO         torque off immediately (hardware) -> NOT_READY when sto_ok
%   Global priority: STO > hard fault > undervoltage (v_dc < V_off -> NOT_READY) > CAN timeout > operator.
%
%   fault_cond : uint16 latched protection word from the Protection Relay blocks (bits 0-5, 7-9)
%   fault_word : fault_cond | CAN_TIMEOUT (bit 6, latched here) | STO_ACTIVE (bit 10, info)
%   duty       : brake-coil duty (1 for n_pullin after release, then duty_hold; 0 = spring applied)
%#codegen

S = struct('n_init', smv(1), 'n_premag', smv(2), 'n_release', smv(3), 'n_engage', smv(4), ...
    'n_stop_hold', smv(5), 'n_park', smv(6), 'n_pullin', smv(7), 'n_reset_win', smv(8), 'V_on', smv(9), ...
    'V_off', smv(10), 'w_stop', smv(11), 'w_park_ref', smv(12), 'tau_park_ref', smv(13), 'duty_hold', smv(14), ...
    'hard_mask', smv(15), 'n_stop_max', smv(16));

persistent st tmr brk brk_tmr parked park_cnt stop_cnt eng eng_tmr latched_can rst_win stp_tmr
if isempty(st)
    st = EscState.INIT; tmr = 0; brk = false; brk_tmr = 0; parked = false; park_cnt = 0;
    stop_cnt = 0; eng = false; eng_tmr = 0; latched_can = false; rst_win = 0; stp_tmr = 0;
end

hard    = bitand(uint16(fault_cond), uint16(S.hard_mask)) ~= uint16(0);
mode_ok = (mode == 1) || (mode == 2);
run_req = enable && mode_ok && ~park_req;
w_abs   = abs(w_meas);
cmd_nz  = (mode == 1 && abs(w_ref) >= S.w_park_ref) || (mode == 2 && abs(tau_ff) >= S.tau_park_ref);
done_engage = eng && (eng_tmr >= S.n_engage);

%% ---- transitions (ordered, highest priority first) ----
nxt = st;
if ~sto_ok
    nxt = EscState.STO;
else
    switch st
        case EscState.INIT
            if tmr >= S.n_init, nxt = EscState.NOT_READY; end
        case EscState.NOT_READY
            if v_dc > S.V_on, nxt = EscState.READY; end
        case EscState.STO
            nxt = EscState.NOT_READY;
        case EscState.FAULT
            if reset_edge
                rst_win = S.n_reset_win;
                latched_can = latched_can && can_to;
            end
            if rst_win > 0
                rst_win = rst_win - 1;
                if ~hard && ~latched_can, nxt = EscState.READY; end
            end
        otherwise   % powered states READY, ENABLING, OPERATION, DISABLING, QUICK_STOP
            if hard
                nxt = EscState.FAULT;
            elseif v_dc < S.V_off
                nxt = EscState.NOT_READY;
            else
                switch st
                    case EscState.READY
                        if run_req && ~can_to, nxt = EscState.ENABLING; end
                    case EscState.ENABLING
                        if can_to
                            nxt = EscState.QUICK_STOP; latched_can = true;
                        elseif ~run_req
                            nxt = EscState.DISABLING;
                        elseif tmr >= S.n_premag + S.n_release
                            nxt = EscState.OPERATION;
                        end
                    case EscState.OPERATION
                        if can_to
                            nxt = EscState.QUICK_STOP; latched_can = true;
                        elseif ~run_req
                            nxt = EscState.DISABLING;
                        elseif parked && cmd_nz
                            nxt = EscState.ENABLING;
                        end
                    case EscState.DISABLING
                        if can_to
                            nxt = EscState.QUICK_STOP; latched_can = true;
                        elseif run_req && ~eng
                            nxt = EscState.OPERATION;
                        elseif run_req
                            nxt = EscState.ENABLING;
                        elseif done_engage
                            nxt = EscState.READY;
                        end
                    case EscState.QUICK_STOP
                        if done_engage, nxt = EscState.FAULT; end
                    otherwise
                end
            end
    end
end

%% ---- entry actions / state timer ----
if nxt ~= st
    if st == EscState.DISABLING && nxt == EscState.QUICK_STOP
        stp_tmr = min(stp_tmr + 1, 1e9);           % the same stop goes on: its time-out keeps counting
    else
        stp_tmr = 0;                               % a stop (DISABLING / QUICK_STOP) begins here
    end
    tmr = 0; stop_cnt = 0; park_cnt = 0; parked = false; rst_win = 0;
    if nxt == EscState.DISABLING || nxt == EscState.QUICK_STOP
        if brk
            eng = false; eng_tmr = 0;              % brake released: stop first, then engage
        else
            eng = true; eng_tmr = S.n_engage;      % brake already engaged: torque may go off at once
        end
    else
        eng = false; eng_tmr = 0;
    end
    st = nxt;
else
    tmr = min(tmr + 1, 1e9); stp_tmr = min(stp_tmr + 1, 1e9);
end

%% ---- in-state counters ----
if st == EscState.OPERATION
    if ~parked
        if mode == 1 && abs(w_ref) < S.w_park_ref && w_abs < S.w_stop
            park_cnt = park_cnt + 1;
        else
            park_cnt = 0;
        end
        if park_cnt >= S.n_park
            parked = true; eng = true; eng_tmr = 0;
        end
    else
        eng_tmr = min(eng_tmr + 1, 1e9);
    end
elseif st == EscState.DISABLING || st == EscState.QUICK_STOP
    if ~eng
        if w_abs < S.w_stop, stop_cnt = stop_cnt + 1; else, stop_cnt = 0; end
        % brake command drops (torque off n_engage later): at once on park_req whatever the speed (ICD), at
        % standstill, or n_stop_max after the stop began whatever the speed (critfix R-01: a stop that cannot
        % reach standstill, e.g. regen-limited on a descent, still ends on the spring brake)
        if park_req || stop_cnt >= S.n_stop_hold || stp_tmr >= S.n_stop_max
            eng = true; eng_tmr = 0;
        end
    else
        eng_tmr = min(eng_tmr + 1, 1e9);
    end
end

%% ---- outputs ----
done_engage = eng && (eng_tmr >= S.n_engage);
if st == EscState.ENABLING
    torque_en = true;  brk_new = (tmr >= S.n_premag);  hold0 = true;
elseif st == EscState.OPERATION
    torque_en = ~(parked && done_engage);  brk_new = ~parked;  hold0 = parked;
elseif st == EscState.DISABLING || st == EscState.QUICK_STOP
    torque_en = ~done_engage;  brk_new = ~eng;  hold0 = true;
else
    torque_en = false; brk_new = false; hold0 = false;
end
qs = (st == EscState.QUICK_STOP);
use_speed = (mode == 1) || hold0;

% brake coil: pull-in at full duty, then PWM hold; spring applied when not energized (fail-safe)
if brk_new
    if brk
        brk_tmr = min(brk_tmr + 1, 1e9);
    else
        brk_tmr = 0;
    end
    if brk_tmr < S.n_pullin, duty = 1; else, duty = S.duty_hold; end
else
    brk_tmr = 0; duty = 0;
end
brk = brk_new;
brake_rel = brk;
brake_released = brk && (brk_tmr >= S.n_release);

fw = uint16(fault_cond);
if latched_can, fw = bitor(fw, uint16(64)); end
if ~sto_ok,     fw = bitor(fw, uint16(1024)); end
fault_word = fw;
state = uint8(st);
end
