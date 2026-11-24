function [payload, flags, frame_new, n_ok, n_bad, n_skip] = ...
         sbus_frame_sync(bytes, len, uart_err, HDR, GAP_TICKS, RAW_MIN, RAW_MAX)
%SBUS_FRAME_SYNC  SBUS frame synchroniser of the MCU firmware (docs/specs/electronics.md 4.1, revision C).
%   Called in one line from the MATLAB Function block fw_sbus_decoder/Frame_Sync (Task_1k, 1 ms).
%   Why no library block: SBUS is a header-only frame (0x0F can also appear inside the payload) with
%   several valid footers; robust framing needs byte-by-byte iteration over a 0..32-byte chunk, a
%   persistent parser state and gap-based resynchronisation (stmutilitieslib/Protocol Decoder was
%   rejected, spec 3.4). The bit unpacking of the accepted frame is left to commutil2 blocks.
%   HDR (uint8(P.electronics.sbus.header)), GAP_TICKS (uint8(P.electronics.sbus.gap_ticks)), RAW_MIN /
%   RAW_MAX (uint16(P.electronics.rcmap.raw_min / raw_max)) arrive on block inputs from Constant blocks,
%   not as parameter data (R2026b JIT relink assertion, T07).
%   Integer saturation is off in the block (the counters wrap).
%
%   Frame validation (SBUS has no checksum; a frame is accepted only if ALL hold):
%     1. header HDR at index 0 (hunting otherwise), exactly 25 bytes before an inter-frame gap of
%        GAP_TICKS empty ticks, and no further byte in the chunk that completes the frame (length);
%     2. no UART error (parity / framing / noise / overrun, uart_err of the chunk) on ANY chunk that
%        delivered a byte of the frame. Revision C: revision B reset the parser *before* consuming the
%        flagged chunk, so a bit error in the chunk that starts a frame (bytes 1..8/9: header, CH1-CH5)
%        was accepted (S6c: 1.25 FS glitch = flipped bit 10 of CH1/CH2). The error now taints the frame;
%     3. footer 0x00 or SBUS2 ((footer & 0x0F) == 0x04), flags reserved bits 4-7 zero;
%     4. plausibility: every 11-bit channel within [RAW_MIN, RAW_MAX] (FrSky 172..1811, ICD 7).
%   All checks complete on the tick of the last byte: no added latency (D37).
%#codegen
% bytes uint8[32], len uint8, uart_err boolean
% HDR uint8 = 0x0F, GAP_TICKS uint8 = 2, RAW_MIN / RAW_MAX uint16 = 172 / 1811
% payload uint8[22], flags uint8, frame_new boolean, n_* uint32
persistent buf idx idle okc badc skipc pay_l flg_l ferr
if isempty(buf)
    buf = zeros(25,1,'uint8'); idx = uint8(0); idle = uint8(0);
    okc = uint32(0); badc = uint32(0); skipc = uint32(0);
    pay_l = zeros(22,1,'uint8'); flg_l = uint8(0); ferr = false;
end
frame_new = false;
if len == 0
    if idle < 255, idle = idle + 1; end
    if idle >= GAP_TICKS && idx > 0      % inter-frame gap => resync
        if idx < 25, badc = badc + 1; end
        idx = uint8(0); ferr = false;
    end
else
    idle = uint8(0);
    if uart_err && idx > 0, ferr = true; end   % frame in progress receives a corrupted chunk
    used = false;                              % a byte of this chunk went into a frame buffer
    n = min(double(len), 32);
    for k = 1:n
        b = bytes(k);
        if idx == 0
            if b ~= HDR
                skipc = skipc + 1;             % hunting for header
                continue
            end
            ferr = uart_err;                   % frame starts in this chunk: its error flag applies
        end
        used = true;
        idx = idx + 1; buf(idx) = b;
        if idx == 25
            ftr = buf(25);
            okFtr = (ftr == 0) || (bitand(ftr, uint8(15)) == 4);   % SBUS / SBUS2
            okFlg = bitand(buf(24), uint8(240)) == 0;              % reserved bits
            okLen = k == n;                    % no byte follows the footer in the same 1 ms chunk
            okRng = true;                      % 16 x 11-bit channels, LSB first (spec 2.6.1)
            for c = 0:15
                i = floor(11*c/8); s = 11*c - 8*i;          % payload byte i (0-based), bit shift s
                w = uint32(buf(2+i)) + bitshift(uint32(buf(3+i)), 8) + bitshift(uint32(buf(4+i)), 16);
                v = bitand(bitshift(w, -s), uint32(2047));
                if v < uint32(RAW_MIN) || v > uint32(RAW_MAX), okRng = false; end
            end
            if okFtr && okFlg && okLen && okRng && ~ferr
                pay_l = buf(2:23); flg_l = buf(24);
                okc = okc + 1; frame_new = true;
            else
                badc = badc + 1;
            end
            idx = uint8(0); ferr = false;
        end
    end
    if uart_err && ~used, badc = badc + 1; end   % corrupted chunk outside any frame (header hit)
end
payload = pay_l; flags = flg_l; n_ok = okc; n_bad = badc; n_skip = skipc;
end
