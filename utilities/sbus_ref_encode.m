function bytes = sbus_ref_encode(ch, flags, footer)
%SBUS_REF_ENCODE  Plain-MATLAB reference SBUS frame encoder (test oracle; owner T07, also used by T08).
%
%   bytes = sbus_ref_encode(ch)                   ch: 1x16 (or Nx16) raw channel values 0..2047
%   bytes = sbus_ref_encode(ch, flags)            flags: Nx4 logical [ch17 ch18 frame_lost failsafe]
%                                                 or Nx1 column of flags bytes (0..255, bits 4-7 kept)
%   bytes = sbus_ref_encode(ch, flags, footer)    footer byte (scalar or Nx1), default 0x00
%                                                 (SBUS2 telemetry footers 0x04/0x14/0x24/0x34)
%   bytes : N x 25 uint8 = [0x0F, payload(22), flags, footer]
%
%   Bit packing (docs/specs/electronics.md 2.6.1): the 16 channels x 11 bit are packed LSB first, i.e.
%   bit(11c + j) = bit j of channel c, and payload byte i holds bits 8i .. 8i+7 (LSB first). This is the
%   reference for rc_sbus_encoder (Integer to Bit 11 LSB first -> Bit to Integer 8 LSB first).
%   Values above 2047 are rejected (error), they cannot be represented in 11 bits.

if nargin < 2 || isempty(flags), flags = false(size(ch, 1), 4); end
if nargin < 3 || isempty(footer), footer = 0; end
if isvector(ch) && numel(ch) == 16, ch = reshape(ch, 1, 16); end
assert(size(ch, 2) == 16, 'sbus_ref_encode:dims', 'ch must be N x 16');
ch = double(ch);
assert(all(ch(:) >= 0 & ch(:) <= 2047 & ch(:) == round(ch(:))), 'sbus_ref_encode:range', ...
    'channel values must be integers 0..2047');
N = size(ch, 1);

% flags byte
if size(flags, 2) == 4                       % [ch17 ch18 frame_lost failsafe] per row
    fl = double(flags ~= 0);
    if size(fl, 1) == 1 && N > 1, fl = repmat(fl, N, 1); end
    fbyte = fl(:, 1) + 2*fl(:, 2) + 4*fl(:, 3) + 8*fl(:, 4);
else
    fbyte = double(flags(:));
    if isscalar(fbyte) && N > 1, fbyte = repmat(fbyte, N, 1); end
end
assert(numel(fbyte) == N && all(fbyte >= 0 & fbyte <= 255), 'sbus_ref_encode:flags', 'bad flags');
footer = double(footer(:));
if isscalar(footer), footer = repmat(footer, N, 1); end

% 176 payload bits, LSB first per channel
bits = zeros(N, 176);
for c = 0:15
    for j = 0:10
        bits(:, 11*c + j + 1) = bitget(ch(:, c + 1), j + 1);
    end
end
payload = zeros(N, 22);
w = 2.^(0:7);
for i = 0:21
    payload(:, i + 1) = bits(:, 8*i + (1:8)) * w.';
end
bytes = uint8([repmat(15, N, 1), payload, fbyte, footer]);
end
