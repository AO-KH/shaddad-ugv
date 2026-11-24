function [txt, M] = row_test_summary(S, run, cfg, rows)
%ROW_TEST_SUMMARY  Summary of an RC row test run, worked out from its saved log (spec section 9, RC row test).
%   [txt, M] = row_test_summary(S, run, cfg, rows)
%     S     run log as run_scenario saves it (variable S of work/results/<id>_<time>.mat): every signal S.<group>.<name> is
%           a struct with t (N x 1, s) and v (N x width); the groups run at their own rates, a missing signal gives NaN
%           (S may be [] or empty after a failed run: the text then gives the error and n/a)
%     run   struct: wall_s (wall-clock time, s), rtf (real-time factor), t_end (simulated time, s), file (path of the log),
%           msg (error text of the run, '' when it ended without error)
%     cfg   settings of the run: param_rowtest() with the five inputs set (n_rows, minutes, T_amb_C, sprayer, fast)
%     rows  centre lines of the field, M x 5 [x1 y1 x2 y2 laneId] (row_test_rows(cfg))
%   txt   the summary as one char row, lines separated by newline (the worker writes it to summary.txt)
%   M     the figures behind it, all double except the two texts:
%     rows_done, passes   rows completed, repeats counted (row_test_count_rows, cfg.row_tol and cfg.row_frac, on the true
%                         path truth.pose2D); times the whole field was covered = the least count of any one row
%     t_done              completion time of each counted row, s, sorted, rows_done x 1 (row_test_count_rows: the last
%                         sample of its run along the row, up to about a second after the row end; a robot that stops
%                         inside the band, as at the end of a run, makes it the end of the log)
%     E_row_Wh            energy used between one completion and the next, Wh, rows_done x 1; the first row counts from
%                         the moment the robot was armed (MANUAL), so each figure includes the turn into its row
%     t_drive, dist, v_mean   driving time, s (MANUAL until the MCU leaves it, at most the minutes asked); distance
%                         (kpi.dist), m; dist / t_drive, m/s
%     E_Wh                energy from the pack, net of regeneration (kpi.E_batt, J, here Wh)
%     Wh_per_row          mean of E_row_Wh, a row without a figure left out: the energy per row of the field page bars
%     Wh_per_row_run      E_Wh / rows_done, the whole run: also the start-up before arming and all after the last
%                         completed row (the unfinished pass, a return to row 1, the stop)
%     Wh_per_km           E_Wh / (dist / 1000)
%     soc_start, soc_end  state of charge at the first and the last sample, %
%     I_peak              largest |pack current|, A
%     T_cell_max_C, T_cell_bms_max_C, T_winding_max_C, T_esc_max_C, T_encl_max_C   highest temperature, degC, of the
%                         cells (true: power_truth.T_cell_max_K, the hottest cell group), the cells as the BMS reads
%                         them (bms_telem.T_cell_max_K, with its sensor offset and noise), the motor windings and the
%                         motor controllers (left and right, drive_truth) and the enclosure (the MCU's sensor
%                         telem.T_encl_degC: the true enclosure air temperature is not logged)
%     dy_max              largest distance from a row centre line over the samples that lie along a row (between its two
%                         ends; the headlands and the U-turns are outside), m
%     mcu_fault_bits, mcu_warn_bits, bms_fault_bits   every bit that was set at any sample (bitwise OR) of telem.fault_word,
%                         telem.warn_word and bms_telem.Fault_Word (bit numbers: docs/icd.md); 0 = none
%     end_reason          text, the first that applies: 'simulation error: <msg>' (run.msg on one line, without hyperlinks)
%                         | 'battery at the SoC floor' (kpi.stop_reason 3) | 'MCU never reached MANUAL (<MODE> at <t> s)'
%                         | 'MCU <MODE> at <t> s' (the MCU left MANUAL for good before the time asked, so not the driver's
%                         doing) | 'run ended early at <t> s (stop_reason <k>)' (the log ends in MANUAL before the time
%                         asked) | 'time up'
%     fidelity            'full' | 'fast' (cfg.fast)
%   The maxima, the current peak and the bits count from the moment the MCU left BOOT and SELFTEST (mode STANDBY or later):
%   the samples before hold start-up values (enclosure sensor -273 and 120 degC, RC link and contactor bits, precharge).

%% ---- the MCU's mode: start-up over, armed, left MANUAL ----
mode = local_sig(S, 'telem.mode');
t_up = -Inf;                                          % the whole log when the mode is missing or never leaves BOOT / SELFTEST
k = find(mode.v >= double(ShaddadMode.STANDBY), 1);
if ~isempty(k), t_up = mode.t(k); end
im = find(mode.v == double(ShaddadMode.MANUAL));      % MANUAL samples; the first and the last frame the driving
armed = ~isempty(im);
t_man = NaN; t_leave = NaN; k_leave = [];
if armed
    t_man = mode.t(im(1));
    if im(end) < numel(mode.t)
        k_leave = im(end) + 1; t_leave = mode.t(k_leave);
    else
        t_leave = mode.t(end);                         % still in MANUAL at the end of the log
    end
end

%% ---- rows from the true path ----
pose = local_sig(S, 'truth.pose2D');
xy = zeros(0, 2);
if ~isempty(pose.t), xy = pose.v(:, 1:2); end
n_row = zeros(size(rows, 1), 1);                      % traversals of each row
t_row = cell(size(rows, 1), 1);
for r = 1:size(rows, 1)
    [n_row(r), t_row{r}] = row_test_count_rows(pose.t, xy, rows(r, :), cfg.row_tol, cfg.row_frac);
end
t_done = sort(vertcat(zeros(0, 1), t_row{:}));
Eb = local_sig(S, 'kpi.E_batt');                      % J, discharge positive
E_row_Wh = zeros(numel(t_done), 1);
if ~isempty(t_done), E_row_Wh = diff(local_at(Eb, [t_man; t_done]))/3600; end

%% ---- figures ----
M.rows_done = sum(n_row);
M.passes = min(n_row);
M.t_done = t_done;
M.E_row_Wh = E_row_Wh;
M.t_drive = 0;
if armed, M.t_drive = min(t_leave - t_man, 60*cfg.minutes); end    % the stop and disarm after the time are not driving
M.dist = local_last(local_sig(S, 'kpi.dist'));
M.v_mean = NaN;
if M.t_drive > 0, M.v_mean = M.dist/M.t_drive; end
M.E_Wh = local_last(Eb)/3600;
M.Wh_per_row = mean(E_row_Wh, 'omitnan');            % NaN when no row has a figure
M.Wh_per_row_run = NaN;
if M.rows_done > 0, M.Wh_per_row_run = M.E_Wh/M.rows_done; end
M.Wh_per_km = NaN;
if M.dist > 0, M.Wh_per_km = M.E_Wh/(M.dist/1000); end
soc = local_sig(S, 'power_truth.SOC_pack_true');
M.soc_start = 100*local_first(soc);
M.soc_end = 100*local_last(soc);
cur = local_sig(S, 'power_truth.I_pack_A');
cur.v = abs(cur.v);                                    % discharge and regeneration both count
M.I_peak = local_max(cur, t_up);
T0 = 273.15;                                           % 0 degC in K
M.T_cell_max_C = local_max(local_sig(S, 'power_truth.T_cell_max_K'), t_up) - T0;
M.T_cell_bms_max_C = local_max(local_sig(S, 'bms_telem.T_cell_max_K'), t_up) - T0;
M.T_winding_max_C = max([local_max(local_sig(S, 'drive_truth.L.T_winding'), t_up), ...
                         local_max(local_sig(S, 'drive_truth.R.T_winding'), t_up)]) - T0;
M.T_esc_max_C = max([local_max(local_sig(S, 'drive_truth.L.T_esc'), t_up), ...
                     local_max(local_sig(S, 'drive_truth.R.T_esc'), t_up)]) - T0;
M.T_encl_max_C = local_max(local_sig(S, 'telem.T_encl_degC'), t_up);
M.dy_max = local_dy_max(xy, rows);
M.mcu_fault_bits = local_bits(local_sig(S, 'telem.fault_word'), t_up);
M.mcu_warn_bits = local_bits(local_sig(S, 'telem.warn_word'), t_up);
M.bms_fault_bits = local_bits(local_sig(S, 'bms_telem.Fault_Word'), t_up);

%% ---- how the run ended ----
msg = regexprep(strtrim(regexprep(char(run.msg), '<[^>]*>', '')), '\s+', ' ');   % one line, without the hyperlinks of MATLAB errors
stop = local_max(local_sig(S, 'kpi.stop_reason'), -Inf);
soc_floor = 3;                                         % Bus_SimKPI stop_reason: 3 SoC floor
if ~isempty(msg)
    M.end_reason = ['simulation error: ' msg];
elseif stop == soc_floor
    M.end_reason = 'battery at the SoC floor';
elseif ~armed
    M.end_reason = 'MCU never reached MANUAL';
    if ~isempty(mode.t)
        M.end_reason = sprintf('%s (%s at %.1f s)', M.end_reason, local_mode(mode.v(end)), mode.t(end));
    end
elseif t_leave < t_man + 60*cfg.minutes                % no driver action ends the driving before the time asked
    if isempty(k_leave)
        M.end_reason = sprintf('run ended early at %.1f s (stop_reason %g)', t_leave, stop);   % MANUAL to the end of the log
    else
        M.end_reason = sprintf('MCU %s at %.1f s', local_mode(mode.v(k_leave)), t_leave);
    end
else
    M.end_reason = 'time up';
end
M.fidelity = 'full';
if cfg.fast
    M.fidelity = 'fast';
end

%% ---- text ----
if cfg.fast
    detail = 'fast (lumped pack, ideal drive, kinematic chassis)';
else
    detail = 'full detail (grouped pack with BMS, drive with losses and temperatures, high-fidelity tracks)';
end
onoff = {'off', 'on'};
soc_txt = 'n/a';
if ~isnan(M.soc_start) && ~isnan(M.soc_end)
    soc_txt = sprintf('%.1f %% at the start, %.1f %% at the end', M.soc_start, M.soc_end);
end
run_txt = local_num(M.Wh_per_row_run, '%.2f Wh (total / rows, incl. start-up, returns, last pass)');
out = {
    'RC row test summary'
    ''
    local_ln('Run log', char(run.file))
    local_ln('Fidelity', detail)
    local_ln('Ended', M.end_reason)
    ''
    'Inputs'
    local_ln('  Rows', sprintf('%d', cfg.n_rows))
    local_ln('  Driving time asked', sprintf('%g min', cfg.minutes))
    local_ln('  Ambient temperature', sprintf('%g degC', cfg.T_amb_C))
    local_ln('  Sprayer', onoff{double(logical(cfg.sprayer)) + 1})
    ''
    'Rows and driving'
    local_ln('  Rows completed', sprintf('%d (repeats counted)', M.rows_done))
    local_ln('  Field covered', local_times(M.passes))
    local_ln('  Driving time', local_dur(M.t_drive))
    local_ln('  Distance', local_num(M.dist, '%.1f m'))
    local_ln('  Mean speed', local_num(M.v_mean, '%.2f m/s'))
    ''
    'Battery'
    local_ln('  Energy from the pack', local_num(M.E_Wh, '%.1f Wh'))
    local_ln('  Energy per row', local_num(M.Wh_per_row, '%.2f Wh'))
    local_ln('  Energy per row, whole run', run_txt)
    local_ln('  Energy per km', local_num(M.Wh_per_km, '%.1f Wh/km'))
    local_ln('  State of charge', soc_txt)
    local_ln('  Peak pack current', local_num(M.I_peak, '%.1f A'))
    ''
    'Highest temperatures'
    local_ln('  Cell', local_num(M.T_cell_max_C, '%.1f degC'))
    local_ln('  Cell (BMS reading)', local_num(M.T_cell_bms_max_C, '%.1f degC'))
    local_ln('  Motor winding', local_num(M.T_winding_max_C, '%.1f degC'))
    local_ln('  Motor controller', local_num(M.T_esc_max_C, '%.1f degC'))
    local_ln('  Enclosure (MCU sensor)', local_num(M.T_encl_max_C, '%.1f degC'))
    ''
    'Row keeping'
    local_ln('  Largest distance from a row line', local_num(M.dy_max, '%.2f m'))
    ''
    'Faults and warnings set during the run'
    local_ln('  MCU fault bits', local_bits_txt(M.mcu_fault_bits))
    local_ln('  MCU warning bits', local_bits_txt(M.mcu_warn_bits))
    local_ln('  BMS fault bits', local_bits_txt(M.bms_fault_bits))
    ''
    'Run'
    local_ln('  Simulated time', local_dur(run.t_end))
    local_ln('  Wall-clock time', local_dur(run.wall_s))
    local_ln('  Real-time factor', local_num(run.rtf, '%.2g'))
    };
txt = strjoin(out', newline);
end

% -------------------------------------------------------------------------------------------
function s = local_sig(S, path)
% logged signal S.<path> as t (N x 1) and v (N x width, double); empty when the log has no such signal
s = struct('t', zeros(0, 1), 'v', zeros(0, 1));
for name = strsplit(path, '.')
    if ~isstruct(S) || ~isfield(S, name{1}), return, end
    S = S.(name{1});
end
if isstruct(S) && isfield(S, 't') && isfield(S, 'v')
    s = struct('t', double(S.t(:)), 'v', double(S.v));
end
end

function x = local_first(s)
% first value of a signal, NaN when it is empty
x = NaN;
if ~isempty(s.v), x = s.v(1); end
end

function x = local_last(s)
% last value of a signal, NaN when it is empty
x = NaN;
if ~isempty(s.v), x = s.v(end); end
end

function x = local_max(s, t0)
% largest value from time t0 on, NaN when there is none
x = NaN;
v = s.v(s.t >= t0, :);
if ~isempty(v), x = max(v, [], 'all', 'omitnan'); end
end

function y = local_at(s, tq)
% value of a signal at the times tq (linear), NaN outside its samples or when it has fewer than two
y = NaN(size(tq));
[t, i] = unique(s.t);
if numel(t) < 2, return, end
y = interp1(t, s.v(i), tq);
end

function b = local_bits(s, t0)
% every bit set in a word signal from time t0 on (bitwise OR of its values), NaN when there is no sample
b = NaN;
w = s.v(s.t >= t0);
if isempty(w), return, end
b = 0;
for u = unique(w(isfinite(w)))'
    b = bitor(b, u);
end
end

function d = local_dy_max(xy, rows)
% largest distance from a row centre line over the samples that lie along a row (projection between its two ends), m
d = NaN;
dmin = inf(size(xy, 1), 1);
for r = 1:size(rows, 1)
    a = rows(r, 1:2);
    u = rows(r, 3:4) - a;
    L = hypot(u(1), u(2));
    u = u/L;                                           % unit vector along the row
    px = xy(:, 1) - a(1);
    py = xy(:, 2) - a(2);
    along = px*u(1) + py*u(2);
    on = along >= 0 & along <= L;
    dmin(on) = min(dmin(on), abs(px(on)*u(2) - py(on)*u(1)));
end
dmin = dmin(isfinite(dmin));
if ~isempty(dmin), d = max(dmin); end
end

function s = local_mode(k)
% name of an MCU mode code (ShaddadMode), 'mode <k>' for a code that is no mode
try
    s = char(ShaddadMode(uint8(k)));
catch
    s = sprintf('mode %d', k);
end
end

function s = local_ln(label, value)
s = sprintf('%-36s %s', label, value);
end

function s = local_num(x, fmt)
% number as text, 'n/a' when there is none (NaN or Inf)
if ~isfinite(x), s = 'n/a'; else, s = sprintf(fmt, x); end
end

function s = local_dur(t)
% duration as text: 45.0 s | 7 min 10 s | 1 h 02 min | n/a
if ~isfinite(t)
    s = 'n/a';
elseif t < 60
    s = sprintf('%.1f s', t);
elseif t < 3600
    n = round(t);
    s = sprintf('%d min %02d s', floor(n/60), mod(n, 60));
else
    m = round(t/60);
    s = sprintf('%d h %02d min', floor(m/60), mod(m, 60));
end
end

function s = local_times(n)
if n == 1, s = '1 time'; else, s = sprintf('%d times', n); end
end

function s = local_bits_txt(b)
% a fault word as text: none | 0x00000401 (bits 0, 10) | n/a
if isnan(b)
    s = 'n/a';
elseif b == 0
    s = 'none';
else
    k = find(bitget(uint32(b), 1:32)) - 1;
    plural = '';
    if numel(k) > 1, plural = 's'; end
    s = sprintf('0x%08X (bit%s %s)', b, plural, strjoin(arrayfun(@(x) sprintf('%d', x), k, 'UniformOutput', false), ', '));
end
end
