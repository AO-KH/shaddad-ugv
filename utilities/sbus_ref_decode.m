function [ch, fl, ok] = sbus_ref_decode(bytes)
%SBUS_REF_DECODE  Plain-MATLAB reference SBUS frame decoder (test oracle; owner T07, also used by T08).
%
%   [ch, fl, ok] = sbus_ref_decode(bytes)
%   bytes : 1x25 (or Nx25) frame bytes [0x0F, payload(22), flags, footer]
%   ch    : N x 16 uint16 raw channel values (0..2047), ch_c = sum_j bit(11c + j) 2^j with
%           bit(8i + k) = bit k of payload byte i (LSB first, docs/specs/electronics.md 2.6.1)
%   fl    : struct of N x 1 columns: ch17, ch18, frame_lost, failsafe (logical), byte (uint8 flags byte),
%           footer (uint8)
%   ok    : N x 1 logical frame validity as the firmware frame sync checks it: header 0x0F, footer 0x00
%           or SBUS2 ((footer & 0x0F) == 0x04), flags reserved bits 4-7 zero

if isvector(bytes) && numel(bytes) == 25, bytes = reshape(bytes, 1, 25); end
assert(size(bytes, 2) == 25, 'sbus_ref_decode:dims', 'bytes must be N x 25');
b = double(bytes);
N = size(b, 1);

% payload -> 176 bits (LSB first per byte) -> 16 x 11-bit channels (LSB first)
bits = zeros(N, 176);
for i = 0:21
    for k = 0:7
        bits(:, 8*i + k + 1) = bitget(b(:, i + 2), k + 1);
    end
end
w = 2.^(0:10);
ch = zeros(N, 16);
for c = 0:15
    ch(:, c + 1) = bits(:, 11*c + (1:11)) * w.';
end
ch = uint16(ch);

f = b(:, 24);
fl = struct('ch17', bitget(f, 1) == 1, 'ch18', bitget(f, 2) == 1, ...
    'frame_lost', bitget(f, 3) == 1, 'failsafe', bitget(f, 4) == 1, ...
    'byte', uint8(f), 'footer', uint8(b(:, 25)));
ftr = b(:, 25);
ok = b(:, 1) == 15 & (ftr == 0 | bitand(ftr, 15) == 4) & bitand(f, 240) == 0;
end
