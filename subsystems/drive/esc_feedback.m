function fb = esc_feedback(mode, state, brake_released, fault_word, derating, lim_active, ~, stall_warn, ...
    w_meas, enc_cnt, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc, tick, k_alive)
%ESC_FEEDBACK  Status word, alive counter and feedback bus of one traction ESC (drive_esc/Feedback, spec 2.9).
%
%   fb = esc_feedback(mode, state, brake_released, fault_word, derating, lim_active, sto_ok, stall_warn, ...
%                     w_meas, enc_cnt, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc, tick, k_alive)
%     mode            cmd_in.mode: 0 OFF, 1 SPEED, 2 TORQUE (double)
%     state           sm.state: EscState 0 INIT .. 8 STO (uint8)
%     brake_released  sm.brake_released (boolean)
%     fault_word      sm.fault_word (uint16; bit 10 STO_ACTIVE is set by ESC_SM while sto_ok is false)
%     derating        Torque_Limiter: boost exhausted or a derating factor active (boolean)
%     lim_active      Speed_Ctrl: the commanded torque is clipped by the limiter (boolean)
%     sto_ok          (the 7th input, ~) not used: STO_ACTIVE comes from the state and from fault-word bit 10
%                     (the former block network terminated it)
%     stall_warn      Protections: stall warning (boolean)
%     w_meas, tau_est, tau_avail, i_q, i_dc, v_dc, T_mot_meas, T_esc   measured values (double), sent as single
%     enc_cnt         encoder count (int32)
%     tick            the alive tick 0 .. 256*n_alive - 1 (the Counter Limited block Tick, a state, stays a block)
%     k_alive         1/P.drive.esc.n_alive (Constant block FbPar)
%     fb              Bus_DriveSideFb (FB frames, ICD 4)
%
%   Status word (uint16): b0 READY (state 2-6), b1 OP_ENABLED (4), b2 FAULT (7), b3 DERATING, b4 BRAKE_RELEASED,
%   b5 MODE_SPEED, b6 MODE_TORQUE, b7 TORQUE_LIMIT_ACTIVE, b8 STO_ACTIVE (state 8 or fault-word bit 10),
%   b9 QUICKSTOP (6), b10 STALL_WARNING. Alive counter: floor(tick/n_alive), +1 per 10 ms (one FB frame).
%
%   The MATLAB Function FbPack of drive_esc/Feedback calls it every 1 ms (D70). It does what the block network
%   did, operation for operation (tests/integration/t_mf_equiv, testDriveEscEquivalent): each cast is that of its
%   Data Type Conversion, Nearest and saturating. MATLAB's casts round to nearest with ties away from zero and
%   saturate, which is the same for the unsigned targets uint16 and uint8 (a negative tie saturates to 0 either
%   way); a float to single cast rounds as the block does. The comments name the former blocks.
%
%   See also esc_torque_limit, esc_sm, build_drive.

%#codegen
st = double(state);                                      % StateD
ready = st >= 2 && st <= 6;                              % Ready1, Ready2, IsReady
op = st == 4;                                            % IsOp
fault = st == 7;                                         % IsFault
sto = st == 8;                                           % IsSto
quick = st == 6;                                         % IsQs
speed = mode == 1;                                       % IsSpeed
torque = mode == 2;                                      % IsTorque
sto_act = sto || bitand(fault_word, uint16(1024)) ~= 0;  % StoBit ([10 10]), StoBitOn, StoAct
b = [uint16(ready), uint16(op), uint16(fault), uint16(derating), uint16(brake_released), uint16(speed), ...
    uint16(torque), uint16(lim_active), uint16(sto_act), uint16(quick), uint16(stall_warn)];   % sb0 .. sb10
status = uint16(0);                                      % StatusMux, StatusShift (0:10 left), Status (OR)
for k = 1:11
    status = bitor(status, bitshift(b(k), k - 1));
end
alive = uint8(floor(double(tick)*k_alive));              % TickD, PerFrame, Floor, alive
fb = struct('state', state, 'status', status, 'fault_code', fault_word, 'w_m', single(w_meas), ...
    'enc_cnt', enc_cnt, 'tau_est', single(tau_est), 'tau_avail', single(tau_avail), 'i_q', single(i_q), ...
    'i_dc', single(i_dc), 'v_dc', single(v_dc), 'T_motor', single(T_mot_meas), 'T_esc', single(T_esc), ...
    'brake_released', brake_released, 'alive', alive, 'rx_ok', true);    % the *_s conversions, RxOk, Fb
end
