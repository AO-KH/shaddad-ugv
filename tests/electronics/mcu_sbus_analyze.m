function a = mcu_sbus_analyze(r)
%MCU_SBUS_ANALYZE  Wire-level check of the SBUS decoder from a tb_mcu_rc run (tests/electronics/mcu_tb_run).
%
%   a = mcu_sbus_analyze(r)
%   The byte stream at the MCU input (r.len / r.bytes / 1 ms) is split into bursts (frames on the wire,
%   separated by >= 2 empty ticks). A burst is intact when it carries exactly 25 bytes that pass
%   sbus_ref_decode. Every frame_new event of the decoder (r.dec, logged in Shaddad_MCU) is matched
%   with the burst that ends at that tick and compared with (a) the reference decode of the burst
%   bytes and (b) the transmitted receiver frame (rc_frame port, r.ch_raw_tx / lost_tx / fs_tx).
%   Output a:
%     .nBurst .nIntact .nDamaged    bursts on the wire (damaged = fewer / more than 25 bytes or bad frame)
%     .nFrameNew                    decoder frame_new events
%     .nWrong                       frame_new events whose channels / flags differ from the wire or TX frame
%     .nMissed                      intact bursts without a frame_new at their last tick
%     .nSpurious                    frame_new events not at the end of an intact burst
%     .resync_max                   max number of intact bursts lost after a damaged burst (0 = next frame OK)
%     .lat_lastbyte_ms              frame_new tick - last-byte tick of the burst (vector, ms)
%     .n0F .n00                     intact frames whose payload (bytes 2..23) contains 0x0F / 0x00
%     .pay                          N x 25 bytes of the intact bursts

len = r.len(:);
n = numel(len);
busy = len > 0;
% burst segmentation: a new burst starts after >= 2 empty ticks (decoder GAP_TICKS)
starts = []; ends = [];
k = 1;
while k <= n
    if busy(k)
        s = k; e = k; gap = 0; j = k + 1;
        while j <= n
            if busy(j), e = j; gap = 0; else, gap = gap + 1; if gap >= 2, break, end, end
            j = j + 1;
        end
        starts(end+1) = s; ends(end+1) = e; %#ok<AGROW>
        k = j;
    else
        k = k + 1;
    end
end
nb = numel(starts);
intact = false(nb, 1); bytesB = zeros(nb, 25, 'uint8');
for b = 1:nb
    seq = [];
    for t = starts(b):ends(b)
        seq = [seq, r.bytes(t, 1:len(t))]; %#ok<AGROW>
    end
    if numel(seq) == 25
        [~, ~, ok] = sbus_ref_decode(uint8(seq));
        intact(b) = ok;
        bytesB(b, :) = uint8(seq);
    end
end
a.nBurst = nb;
a.nIntact = nnz(intact);
a.nDamaged = nb - a.nIntact;
fn = find(r.dec.frame_new(:));
a.nFrameNew = numel(fn);
a.nWrong = 0; a.nSpurious = 0;
lat = nan(numel(fn), 1);
hit = false(nb, 1);
for i = 1:numel(fn)
    t = fn(i);
    b = find(starts <= t & ends >= t, 1);
    if isempty(b) || ~intact(b)
        a.nSpurious = a.nSpurious + 1;
        continue
    end
    hit(b) = true;
    lat(i) = t - ends(b);                       % ms (1 ms ticks)
    [chW, flW] = sbus_ref_decode(bytesB(b, :));
    dec = double(r.dec.ch_raw(t, :));
    okW = isequal(dec, double(chW)) && r.dec.frame_lost(t) == flW.frame_lost && r.dec.failsafe(t) == flW.failsafe ...
        && r.dec.ch17(t) == flW.ch17 && r.dec.ch18(t) == flW.ch18;
    okT = isequal(dec, double(r.ch_raw_tx(t, :))) && r.dec.frame_lost(t) == r.lost_tx(t) && r.dec.failsafe(t) == r.fs_tx(t);
    if ~(okW && okT), a.nWrong = a.nWrong + 1; end
end
a.nMissed = nnz(intact & ~hit);
a.lat_lastbyte_ms = lat(~isnan(lat));
% resync: intact bursts not decoded right after a damaged one
a.resync_max = 0;
for b = find(~intact)'
    m = 0;
    for c = b + 1:nb
        if ~intact(c), break, end
        if hit(c), break, end
        m = m + 1;
    end
    a.resync_max = max(a.resync_max, m);
end
pay = bytesB(intact, 2:23);
a.n0F = nnz(any(pay == 15, 2));
a.n00 = nnz(any(pay == 0, 2));
a.pay = bytesB(intact, :);
end
