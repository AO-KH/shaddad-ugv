function [bytes, len, uart_err, pending] = sbus_uart_line_sim(frame, frame_new, t, ...
                                   r_err, r_drop, Tb, phase0, drift_ppm, Tf, p_err, p_drop)
%SBUS_UART_LINE_SIM  SBUS UART byte-timing model of RC_Link_Sim/UART_Line_Sim (simulation only, 1 kHz).
%#codegen
%   docs/specs/electronics.md 4.2 (owner T07). Called in one line from the MATLAB Function block
%   rc_uart_line_sim/UART_Active/UART_Line (the only MATLAB Function of RC_Link_Sim).
%
%   Why there is no block: a timed, variable-length byte queue in which each byte completes at
%   t0 + i*Tb (sub-millisecond phase, drift, drops, errors) chunked per 1 ms tick; Buffer / Unbuffer
%   cannot produce variable-length chunks aligned to byte completion times.
%
%   frame      uint8[25]  SBUS frame (held at 1 kHz by the frame-period -> 1 ms Rate Transition)
%   frame_new  boolean    true on the 1 ms tick at which a new frame is released
%   t          double     s, Digital Clock 1e-3
%   r_err, r_drop double[25]  seeded Uniform Random Number vectors (1 ms)
%   Tb         s per byte on the line (12 bit at 100 kbit/s = 120 us)
%   phase0     s, initial sub-millisecond frame-start phase
%   drift_ppm  receiver clock drift (the phase advances Tf*drift each frame, modulo 1 ms)
%   Tf         s, SBUS frame period
%   p_err      byte error probability (max of P.rc.uart.p_err and flt.rc_byte_err_rate)
%   p_drop     byte drop probability
%   bytes uint8[32] (valid 1..len, rest 0), len uint8, uart_err boolean = this chunk contains a byte
%   with an injected bit error (the receiver UART sees a parity error: a single flipped bit always
%   violates even parity).
%   pending boolean = bytes are still queued after this tick. Extension of spec 4.2: the block runs in
%   an Enabled Subsystem enabled by frame_new OR pending(k-1), so the function executes only while a
%   frame is on the line (identical outputs: with an empty queue and no new frame it returns len = 0,
%   which is also the disabled-output reset value) - this removes the per-tick call cost on the
%   ~9 idle ticks of every 14 ms frame.
%
%   Queue: ring buffer of 64 bytes (head hd, count nq) with completion times tq and error marks eq.

QN = 64;
persistent q tq eq hd nq ph
if isempty(q)
    q = zeros(QN, 1, 'uint8'); tq = zeros(QN, 1); eq = false(QN, 1); hd = 0; nq = 0; ph = phase0;
end
if frame_new
    ph = mod(ph + Tf*drift_ppm*1e-6, 1e-3);          % drifting sub-ms phase
    t0 = t + ph;
    for i = 1:25
        if r_drop(i) < p_drop, continue; end          % lost byte (overrun / noise)
        b = frame(i);
        e = false;
        if r_err(i) < p_err                           % flip one bit, chosen by the same draw
            b = bitxor(b, uint8(2^floor(8*r_err(i)/p_err)));
            e = true;
        end
        if nq < QN
            j = mod(hd + nq, QN) + 1;
            nq = nq + 1; q(j) = b; tq(j) = t0 + i*Tb; eq(j) = e;
        end
    end
end
bytes = zeros(32, 1, 'uint8'); uart_err = false;
n = 0;
while n < nq && n < 32 && tq(hd + 1) <= t + 1e-9    % bytes completed by this tick (FIFO)
    n = n + 1; bytes(n) = q(hd + 1); uart_err = uart_err || eq(hd + 1);
    hd = mod(hd + 1, QN);
end
nq = nq - n;
len = uint8(n);
pending = nq > 0;
end
