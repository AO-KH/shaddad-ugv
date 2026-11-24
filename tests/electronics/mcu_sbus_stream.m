function [bytes, len, err] = mcu_sbus_stream(frames, startTick, N, chunk)
%MCU_SBUS_STREAM  Synthetic 1 ms UART chunks (Bus_SbusStream layout) from SBUS frames.
%
%   [bytes, len, err] = mcu_sbus_stream(frames, startTick, N)
%   frames     M x 25 uint8 (e.g. sbus_ref_encode output)
%   startTick  M x 1 tick index (1-based, 1 ms ticks) of the first chunk of each frame
%   N          number of ticks
%   chunk      bytes per tick (default [9 8 8], the 120 us byte time for an aligned start)
%   bytes N x 32 uint8, len N x 1 uint8, err N x 1 logical (false)

if nargin < 4, chunk = [9 8 8]; end
bytes = zeros(N, 32, 'uint8');
len = zeros(N, 1, 'uint8');
err = false(N, 1);
for m = 1:size(frames, 1)
    p = 0;
    for c = 1:numel(chunk)
        t = startTick(m) + c - 1;
        if t > N, break, end
        nb = min(chunk(c), 25 - p);
        bytes(t, double(len(t)) + (1:nb)) = frames(m, p + (1:nb));
        len(t) = len(t) + nb;
        p = p + nb;
        if p >= 25, break, end
    end
end
end
