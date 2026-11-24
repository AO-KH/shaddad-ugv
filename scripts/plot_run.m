function figs = plot_run(src, varargin)
%PLOT_RUN  Plot everything in a saved run log: commands vs response, tracks and motors, battery and energy,
%   power electronics and temperatures, RC link and MCU safety words, navigation and vehicle dynamics.
%
%   plot_run()                                   newest log in work/results
%   plot_run(res)  or  plot_run(res.file)        a run from phase1_live_drive, phase1_test_drive or run_scenario
%   plot_run('L1')                               newest log of one scenario ID (L1 live drive, D1 test drive, S1 ...)
%   plot_run('L1_20261027_011510')               one log by name (work/results)
%   plot_run(..., 'Pages', {'drive', 'battery'}) some pages only (default: all six)
%       drive    operator sticks, speed and yaw-rate command vs response, path on the field, MCU mode
%       tracks   track speed reference / encoder / belt / ground, motor speed, torque and limit, slip, motor power
%       battery  SOC, pack voltage / current / power, energy breakdown, cells, temperatures, BMS limits
%       power    LV rails, HV bus, contactors and safety chain, motor / ESC / enclosure temperatures, losses
%       rc       all RC channels, link quality, SBUS counters, MCU fault and warning bits, command source
%       nav      position and heading estimate vs truth, roll / pitch, accelerations, GNSS, nav flags, contact
%   plot_run(..., 'TimeRange', [t0 t1])          zoom the time axes (the time axes of one page are linked)
%   plot_run(..., 'Save', true)                  also write one PNG per page to work/reports/plots_<log name>/
%   plot_run(..., 'Save', true, 'Folder', d)     the same into the folder d (made when it does not exist)
%   plot_run(..., 'Save', true, 'Fig', true)     also one .fig per page; it opens visible, also when drawn hidden
%       files <k>_<page>.png and <k>_<page>.fig, k = place in Pages; 'Folder' and 'Fig' take effect only with
%       'Save', true: without it nothing is written
%   plot_run(..., 'Visible', 'off')              hidden figures (batch use); the caller closes them
%   plot_run(..., 'Size', [1000 640])            figure size in pixels (default: up to 1500 x 950, fits the screen)
%   plot_run(..., 'Note', 'full detail')         a text added to the title of every page, after the log name and its
%       figures (default '': none); phase1_row_run gives the fidelity of the run, as the log does not hold it
%
%   Bands on the time plots: grey = not armed (BOOT, SELFTEST, STANDBY), yellow = HOLD, red = ESTOP or FAULT.
%   A tile whose signals the run did not log, or that stay at zero in the chosen variant, shows a note instead.

if nargin < 1, src = ''; end
if (ischar(src) || isstring(src)) && ...
        any(strcmpi(src, {'Pages', 'TimeRange', 'Save', 'Folder', 'Fig', 'Visible', 'Size', 'Note'}))
    varargin = [{src} varargin];  src = '';
end
ip = inputParser;
ip.addParameter('Pages', {'drive', 'tracks', 'battery', 'power', 'rc', 'nav'});
ip.addParameter('TimeRange', []);
ip.addParameter('Save', false);
ip.addParameter('Folder', '');
ip.addParameter('Fig', false);
ip.addParameter('Visible', 'on');
ip.addParameter('Size', []);
ip.addParameter('Note', '');
ip.parse(varargin{:});
o = ip.Results;

[S, info] = local_load(src);
note = char(o.Note);                                % char: an empty string "" is no note either
if ~isempty(note), info.title = sprintf('%s  |  %s', info.title, note); end
pages = cellstr(lower(string(o.Pages)));
figs = gobjects(0);
for k = 1:numel(pages)
    switch pages{k}
        case 'drive',   figs(end+1) = local_drive(S, info, o);    %#ok<AGROW>
        case 'tracks',  figs(end+1) = local_tracks(S, info, o);   %#ok<AGROW>
        case 'battery', figs(end+1) = local_battery(S, info, o);  %#ok<AGROW>
        case 'power',   figs(end+1) = local_power(S, info, o);    %#ok<AGROW>
        case 'rc',      figs(end+1) = local_rc(S, info, o);       %#ok<AGROW>
        case 'nav',     figs(end+1) = local_nav(S, info, o);      %#ok<AGROW>
        otherwise
            error('plot_run:page', 'Unknown page ''%s'' (drive, tracks, battery, power, rc, nav).', pages{k});
    end
end
if o.Save                                           % files only here: without Save, Folder and Fig do nothing
    d = char(o.Folder);
    if isempty(d), d = fullfile(info.root, 'work', 'reports', ['plots_' info.name]); end
    if ~isfolder(d), mkdir(d); end
    for k = 1:numel(figs)                           % getframe: exportgraphics takes ~30 s per page with legends
        fr = getframe(figs(k));
        imwrite(fr.cdata, fullfile(d, sprintf('%d_%s.png', k, pages{k})));
        if o.Fig, local_savefig(figs(k), fullfile(d, sprintf('%d_%s.fig', k, pages{k}))); end
    end
    fprintf('Plots saved in %s\n', d);
end
if nargout == 0, clear figs; end
end

% ================================================================================================ pages
function f = local_drive(S, info, o)
[f, tl] = local_fig('Commands and response', info, o, 3, 2);
ax = local_ax(tl, 1, 'Operator commands (RC channels as decoded by the MCU)', 'stick [-1 .. 1]');
local_line(ax, S, 'telem.rc_ch_norm', 2, 'throttle CH2 (+ = forward)');
local_line(ax, S, 'telem.rc_ch_norm', 1, 'steering CH1 (+ = right)');
local_line(ax, S, 'telem.rc_ch_norm', 7, 'speed scale CH7 (0.25 .. 1)', @(x) 0.25 + (x + 1)/2*0.75);
axs = ax;
ax = local_ax(tl, 3, 'Forward speed: command vs response', 'speed [m/s]');
local_line(ax, S, 'telem.v_target', 1, 'MCU target (stick x scale)', [], 'LineWidth', 1.2);
local_line(ax, S, 'telem.v_ref', 1, 'MCU reference (ramped)');
local_line(ax, S, 'truth.speed_fwd', 1, 'actual (truth)', [], 'LineWidth', 1.2);
if info.nav, local_line(ax, S, 'nav_state.vel_B', 1, 'estimated (navigation)', [], '--'); end
axs(end+1) = ax;
ax = local_ax(tl, 5, 'Yaw rate: command vs response (+ = left)', 'yaw rate [rad/s]');
local_line(ax, S, 'telem.w_target', 1, 'MCU target', [], 'LineWidth', 1.2);
local_line(ax, S, 'telem.w_ref', 1, 'MCU reference (ramped)');
local_line(ax, S, 'truth.omega_B', 3, 'actual (truth)', [], 'LineWidth', 1.2);
local_line(ax, S, 'telem.yaw_rate_meas', 1, 'gyro (MCU)', [], '--');
axs(end+1) = ax;
ax = local_ax(tl, 6, 'MCU mode', '');
local_mode(ax, S);
axs(end+1) = ax;
local_path(nexttile(tl, 2, [2 1]), S, info);
local_finish(axs, S, info, o);
end

function f = local_tracks(S, info, o)
[f, tl] = local_fig('Tracks and motors', info, o, 3, 2);
[cL, cR] = local_lr();
side = {'Left', 'Right'};
for k = 1:2
    ax = local_ax(tl, k, sprintf('%s track speed', side{k}), 'speed [m/s]');
    local_line(ax, S, 'telem.vtrack_ref', k, 'MCU reference', [], 'LineWidth', 1.2);
    local_line(ax, S, 'telem.vtrack_meas', k, 'measured (encoder)');
    local_line(ax, S, 'truth.vBelt_LR', k, 'belt (truth)', [], '--');
    local_line(ax, S, 'truth.vGround_LR', k, 'ground (truth)', [], ':', 'LineWidth', 1.2);
    axs(k) = ax; %#ok<AGROW>
end
rpm = @(x) x*60/(2*pi);
ax = local_ax(tl, 3, 'Motor speed', 'speed [rpm]');
local_line(ax, S, 'telem.w_ref_motor', 1, 'L setpoint', rpm, '--', 'Color', cL);
local_line(ax, S, 'drive_truth.L.w_m', 1, 'L actual', rpm, 'Color', cL);
local_line(ax, S, 'telem.w_ref_motor', 2, 'R setpoint', rpm, '--', 'Color', cR);
local_line(ax, S, 'drive_truth.R.w_m', 1, 'R actual', rpm, 'Color', cR);
axs(end+1) = ax;
ax = local_ax(tl, 4, 'Motor torque per side (2 motors), feed-forward and MCU motoring limit', 'torque [N m]');
local_line(ax, S, 'drive_truth.L.tau_m', 1, 'L motor', [], 'Color', cL);
local_line(ax, S, 'drive_truth.R.tau_m', 1, 'R motor', [], 'Color', cR);
local_line(ax, S, 'telem.tau_ff', 1, 'L feed-forward', [], ':', 'Color', cL);
local_line(ax, S, 'telem.tau_ff', 2, 'R feed-forward', [], ':', 'Color', cR);
local_line(ax, S, 'telem.tau_lim_mot', 1, 'L limit', [], '--', 'Color', cL);
local_line(ax, S, 'telem.tau_lim_mot', 2, 'R limit', [], '--', 'Color', cR);
axs(end+1) = ax;
ax = local_ax(tl, 5, 'Track slip (truth) and MCU traction-control factor', '[-]');
local_line(ax, S, 'truth.slip_LR', 1, 'L slip', [], 'Color', cL);
local_line(ax, S, 'truth.slip_LR', 2, 'R slip', [], 'Color', cR);
local_line(ax, S, 'telem.k_traction', 1, 'L traction factor', [], '--', 'Color', cL);
local_line(ax, S, 'telem.k_traction', 2, 'R traction factor', [], '--', 'Color', cR);
axs(end+1) = ax;
ax = local_ax(tl, 6, 'DC power and losses per side (2 motors + dual ESC + 2 gearboxes)', 'power [W]');
local_line(ax, S, 'drive_truth.L.p_dc', 1, 'L DC power', [], 'Color', cL);
local_line(ax, S, 'drive_truth.R.p_dc', 1, 'R DC power', [], 'Color', cR);
local_line(ax, S, 'drive_truth.L.P_loss', 1, 'L losses', [], ':', 'Color', cL);
local_line(ax, S, 'drive_truth.R.P_loss', 1, 'R losses', [], ':', 'Color', cR);
axs(end+1) = ax;
local_finish(axs, S, info, o);
end

function f = local_battery(S, info, o)
[f, tl] = local_fig('Battery and energy', info, o, 4, 2);
degC = @(x) x - 273.15;
ax = local_ax(tl, 1, 'State of charge', 'SOC [%]');
local_line(ax, S, 'power_truth.SOC_pack_true', 1, 'true', @local_pct, 'LineWidth', 1.4);
local_line(ax, S, 'bms_telem.SOC_pack', 1, 'BMS estimate', @local_pct, '--');
local_line(ax, S, 'bms_telem.SOC_cc', 1, 'BMS coulomb count', @local_pct, ':');
local_line(ax, S, 'telem.SOC', 1, 'MCU (from BMS over CAN)', @local_pct, '-.');
axs = ax;
ax = local_ax(tl, 2, 'Pack voltage and current', 'voltage [V]');
local_line(ax, S, 'power_truth.V_term_V', 1, 'terminal voltage (truth)');
local_line(ax, S, 'bms_telem.V_pack_V', 1, 'voltage (BMS)', [], '--');
yyaxis(ax, 'right');  ylabel(ax, 'current [A] (+ = discharge)');
local_line(ax, S, 'power_truth.I_pack_A', 1, 'current (truth)');
yyaxis(ax, 'left');
axs(end+1) = ax;
ax = local_ax(tl, 3, 'Power', 'power [W]');
local_line(ax, S, 'power_truth.P_pack_W', 1, 'battery pack', [], 'LineWidth', 1.2);
[t, pl] = local_sig(S, 'drive_truth.L.p_dc');
pr = local_at(S, 'drive_truth.R.p_dc', t);
if ~isempty(pl) && ~isempty(pr), plot(ax, t, pl(:, 1) + pr(:, 1), 'DisplayName', 'traction (both motors, DC)'); end
[t, v24] = local_sig(S, 'power_status.V_24V_V');
i24 = local_at(S, 'power_status.I_24V_A', t);  v12 = local_at(S, 'power_status.V_12V_V', t);
i12 = local_at(S, 'power_status.I_12V_A', t);
if ~isempty(v24) && ~isempty(i24) && ~isempty(v12) && ~isempty(i12)
    plot(ax, t, v24.*i24 + v12.*i12, 'DisplayName', 'LV electronics (24 V + 12 V rails)');
end
axs(end+1) = ax;
[~, e] = local_sig(S, 'power_truth.E_out_Wh');
ttl = 'Energy (cumulative)';
if ~isempty(e) && info.dist > 1
    ttl = sprintf('Energy (cumulative): %.1f Wh from the pack, %.0f Wh/km', e(end) - e(1), 1000*(e(end) - e(1))/info.dist);
end
ax = local_ax(tl, 4, ttl, 'energy [Wh]');
local_line(ax, S, 'power_truth.E_out_Wh', 1, 'out of the pack', [], 'LineWidth', 1.4);
local_line(ax, S, 'power_truth.E_trac_dc_Wh', 1, 'traction');
local_line(ax, S, 'power_truth.E_impl_dc_Wh', 1, 'implement');
local_line(ax, S, 'power_truth.E_lv_Wh', 1, 'LV electronics');
local_line(ax, S, 'power_truth.E_loss_batt_Wh', 1, 'battery losses', [], '--');
local_line(ax, S, 'power_truth.E_dcdc_loss_Wh', 1, 'DC/DC losses', [], '--');
local_line(ax, S, 'power_truth.E_pdu_loss_Wh', 1, 'PDU losses', [], '--');
local_line(ax, S, 'power_truth.E_in_Wh', 1, 'charged in', [], ':');
axs(end+1) = ax;
[~, vc] = local_sig(S, 'power_truth.V_cell_true');  ng = size(vc, 2);   % cell groups of the pack (13 on the real robot)
ax = local_ax(tl, 5, sprintf('Cell voltages (%d groups, thin) and BMS min / max', ng), 'voltage [V]');
local_line(ax, S, 'power_truth.V_cell_true', [], '', [], 'Color', [0.75 0.75 0.75]);
local_line(ax, S, 'bms_telem.V_cell_max_V', 1, 'BMS max', [], 'LineWidth', 1.2);
local_line(ax, S, 'bms_telem.V_cell_min_V', 1, 'BMS min', [], 'LineWidth', 1.2);
axs(end+1) = ax;
ax = local_ax(tl, 6, sprintf('Cell temperatures (%d groups, thin), pack air and ambient', ng), 'temperature [degC]');
local_line(ax, S, 'power_truth.T_cell_true', [], '', degC, 'Color', [0.75 0.75 0.75]);
local_line(ax, S, 'bms_telem.T_cell_max_K', 1, 'BMS cell max', degC, 'LineWidth', 1.2);
local_line(ax, S, 'bms_telem.T_pack_air_K', 1, 'pack air', degC);
local_line(ax, S, 'env.T_amb', 1, 'ambient', degC, '--');
axs(end+1) = ax;
ax = local_ax(tl, 7, 'BMS current limits and pack current', 'current [A]');
local_line(ax, S, 'bms_telem.I_dis_lim_A', 1, 'discharge limit (DCL)', [], 'LineWidth', 1.2);
local_line(ax, S, 'bms_telem.I_chg_lim_A', 1, 'charge limit (CCL)', [], 'LineWidth', 1.2);
local_line(ax, S, 'bms_telem.I_pack_A', 1, 'pack current (BMS)');
axs(end+1) = ax;
ax = local_ax(tl, 8, 'BMS remaining energy and runtime', 'energy [Wh]');
local_line(ax, S, 'bms_telem.E_remaining_Wh', 1, 'remaining energy');
yyaxis(ax, 'right');  ylabel(ax, 'runtime [h] (shown up to 24 h)');
local_line(ax, S, 'bms_telem.Runtime_Remaining_s', 1, 'remaining runtime at the present load', ...
    @(x) min(x/3600, 24));
yyaxis(ax, 'left');
axs(end+1) = ax;
local_finish(axs, S, info, o);
end

function f = local_power(S, info, o)
[f, tl] = local_fig('Power electronics, safety chain and temperatures', info, o, 3, 2);
[cL, cR] = local_lr();
degC = @(x) x - 273.15;
ax = local_ax(tl, 1, 'Low-voltage rails', 'voltage [V]');
local_line(ax, S, 'power_status.V_24V_V', 1, '24 V');
local_line(ax, S, 'power_status.V_24IMP_V', 1, '24 V implement', [], '--');
local_line(ax, S, 'power_status.V_12V_V', 1, '12 V');
local_line(ax, S, 'power_status.V_5V_V', 1, '5 V');
local_line(ax, S, 'power_status.V_3V3_V', 1, '3.3 V');
axs = ax;
ax = local_ax(tl, 3, 'HV bus and low-voltage rail currents', 'voltage [V]');
local_line(ax, S, 'power_status.V_bus_V', 1, 'HV bus');
local_line(ax, S, 'power_status.V_pack_V', 1, 'pack', [], '--');
yyaxis(ax, 'right');  ylabel(ax, 'current [A]');
local_line(ax, S, 'power_status.I_24V_A', 1, '24 V rail current');
local_line(ax, S, 'power_status.I_12V_A', 1, '12 V rail current', [], '--');
yyaxis(ax, 'left');
axs(end+1) = ax;
ax = local_ax(tl, 5, 'Conversion losses', 'power [W]');
local_line(ax, S, 'power_truth.P_dcdc_loss_W', 1, 'DC/DC');
local_line(ax, S, 'power_truth.P_pdu_loss_W', 1, 'PDU');
local_line(ax, S, 'drive_truth.L.P_loss', 1, 'L drive', [], ':', 'Color', cL);
local_line(ax, S, 'drive_truth.R.P_loss', 1, 'R drive', [], ':', 'Color', cR);
axs(end+1) = ax;
ax = local_ax(tl, 2, 'Contactors, safety chain and power flags (high = true)', '', [2 1]);
local_digital(ax, S, {'power_status.Key_On', 'key on', 0; 'power_status.K1_Closed', 'K1 main', 0; ...
    'power_status.K2_Closed', 'K2 precharge', 0; 'power_status.K3_Closed', 'K3 implement 24 V', 0; ...
    'power_status.K4_Closed', 'K4 charge', 0; 'bms_telem.HV_Ready', 'BMS HV ready', 0; ...
    'telem.power_ready', 'MCU power ready', 0; 'power_status.Rail_24V_OK', '24 V OK', 0; ...
    'power_status.Rail_12V_OK', '12 V OK', 0; 'power_status.Rail_5V_OK', '5 V OK', 0; ...
    'power_status.Rail_3V3_OK', '3.3 V OK', 0; 'power_status.Safety_OK', 'safety relay OK', 0; ...
    'power_status.Safety_HV_OK', 'safety HV OK', 0; 'power_status.EStop_Ch1_OK', 'E-stop ch 1 OK', 0; ...
    'power_status.EStop_Ch2_OK', 'E-stop ch 2 OK', 0; 'power_status.EStop_Any', 'E-stop pressed', 1; ...
    'power_status.Safety_Latched_Off', 'safety latched off', 1; 'power_status.Fuse_Blown', 'fuse blown', 1; ...
    'power_status.Dock_Present', 'on the dock', 2; 'bms_telem.Charging_Active', 'charging', 2});
axs(end+1) = ax;
ax = local_ax(tl, 6, 'Motor, ESC, enclosure and ambient temperatures', 'temperature [degC]');
local_line(ax, S, 'drive_truth.L.T_winding', 1, 'L winding', degC, 'Color', cL);
local_line(ax, S, 'drive_truth.R.T_winding', 1, 'R winding', degC, 'Color', cR);
local_line(ax, S, 'drive_truth.L.T_housing', 1, 'L housing', degC, '--', 'Color', cL);
local_line(ax, S, 'drive_truth.R.T_housing', 1, 'R housing', degC, '--', 'Color', cR);
local_line(ax, S, 'drive_truth.L.T_esc', 1, 'L ESC', degC, ':', 'Color', cL);
local_line(ax, S, 'drive_truth.R.T_esc', 1, 'R ESC', degC, ':', 'Color', cR);
local_line(ax, S, 'telem.T_encl_degC', 1, 'enclosure (MCU)', [], 'LineWidth', 1.2);
local_line(ax, S, 'env.T_amb', 1, 'ambient', degC, '-.');
axs(end+1) = ax;
local_finish(axs, S, info, o);
end

function f = local_rc(S, info, o)
[f, tl] = local_fig('RC link and MCU health', info, o, 3, 2);
ax = local_ax(tl, 1, 'RC channels as decoded by the MCU (-1 .. 1 around each line)', '', [2 1]);
names = {'CH1 steering', 'CH2 throttle', 'CH3 aux', 'CH4 aux', 'CH5 mode', 'CH6 arm', 'CH7 speed', ...
    'CH8 implement', 'CH9 reset', 'CH10 lights', 'CH11 lift', 'CH12 E-stop', 'CH13 mission', 'CH14 home'};
[t, ch] = local_sig(S, 'telem.rc_ch_norm');
if ~isempty(ch)
    n = min(14, size(ch, 2));
    for k = 1:n
        plot(ax, t, (n + 1 - k) + 0.4*ch(:, k), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    yticks(ax, 1:n);  yticklabels(ax, flip(names(1:n)));  ylim(ax, [0.4 n + 0.6]);
end
axs = ax;
ax = local_ax(tl, 2, 'Radio link quality and frame age', 'link quality [0 .. 1]');
local_line(ax, S, 'telem.rc_lq', 1, 'link quality');
[t, lost] = local_sig(S, 'telem.rc_frame_lost');
if ~isempty(lost) && any(lost(:, 1))
    plot(ax, t(lost(:, 1) > 0), 0.02 + 0*t(lost(:, 1) > 0), 'rv', 'MarkerFaceColor', 'r', 'MarkerSize', 4, ...
        'DisplayName', 'frame lost');
end
[t, fs] = local_sig(S, 'telem.rc_failsafe');
if ~isempty(fs) && any(fs(:, 1))
    plot(ax, t(fs(:, 1) > 0), 0.1 + 0*t(fs(:, 1) > 0), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 4, ...
        'DisplayName', 'failsafe');
end
yyaxis(ax, 'right');  ylabel(ax, 'frame age [ms]');
local_line(ax, S, 'telem.rc_age', 1, 'age of last good frame', @(x) 1000*min(x, 1));
yyaxis(ax, 'left');
axs(end+1) = ax;
ax = local_ax(tl, 4, 'SBUS frame counters and MCU task overruns', 'frames OK');
local_line(ax, S, 'telem.sbus_ok', 1, 'frames OK');
yyaxis(ax, 'right');  ylabel(ax, 'count');
local_line(ax, S, 'telem.sbus_bad', 1, 'frames rejected');
local_line(ax, S, 'telem.sbus_skip', 1, 'frames skipped', [], '--');
local_line(ax, S, 'telem.overruns', 1, 'task overruns', [], ':');
yyaxis(ax, 'left');
axs(end+1) = ax;
ax = local_ax(tl, 5, 'MCU fault and warning bits (only bits that were set)', '');
local_bits(ax, S);
axs(end+1) = ax;
ax = local_ax(tl, 6, 'Command source and ESC states', '');
local_line(ax, S, 'telem.cmd_src', 1, 'command source', [], 'LineWidth', 1.4);
local_line(ax, S, 'telem.drive_state', 1, 'left ESC state', [], '--');
local_line(ax, S, 'telem.drive_state', 2, 'right ESC state', [], ':', 'LineWidth', 1.2);
yticks(ax, 0:8);  ylim(ax, [-0.5 8.5]);
yticklabels(ax, {'0 NONE / INIT', '1 RC / NOT_READY', '2 AUTO / READY', '3 ENABLING', '4 OPERATION', ...
    '5 DISABLING', '6 QUICK_STOP', '7 FAULT', '8 STO'});
axs(end+1) = ax;
local_finish(axs, S, info, o);
end

function f = local_nav(S, info, o)
[f, tl] = local_fig('Navigation and vehicle dynamics', info, o, 4, 2);
[cL, cR] = local_lr();
deg = @(x) rad2deg(x);
ax = local_ax(tl, 1, 'Position estimate error (horizontal) and estimated 1-sigma', 'error [m]');
[t, pt] = local_sig(S, 'truth.pos_W');
pe = local_at(S, 'nav_state.pos_W', t);
if ~info.nav
    local_note(ax, 'navigation fusion not running (Phase 1)');
elseif ~isempty(pt) && ~isempty(pe)
    plot(ax, t, vecnorm(pe(:, 1:2) - pt(:, 1:2), 2, 2), 'LineWidth', 1.2, 'DisplayName', '|estimate - truth|');
    ps = local_at(S, 'nav_state.posStd', t);
    if ~isempty(ps), plot(ax, t, vecnorm(ps(:, 1:2), 2, 2), '--', 'DisplayName', 'estimated 1-sigma'); end
end
axs = ax;
ax = local_ax(tl, 2, 'Heading (yaw, CCW from East, unwrapped)', 'heading [deg]');
local_line(ax, S, 'truth.eul', 1, 'truth', @(x) rad2deg(unwrap(x)), 'LineWidth', 1.2);
if info.nav, local_line(ax, S, 'nav_state.eul', 1, 'estimate', @(x) rad2deg(unwrap(x)), '--'); end
axs(end+1) = ax;
ax = local_ax(tl, 3, 'Roll and pitch (pitch + = nose down) and terrain grade', 'angle [deg]');
local_line(ax, S, 'truth.eul', 3, 'roll (truth)', deg);
local_line(ax, S, 'truth.eul', 2, 'pitch (truth)', deg);
local_line(ax, S, 'telem.roll', 1, 'roll (MCU)', deg, '--');
local_line(ax, S, 'telem.pitch', 1, 'pitch (MCU)', deg, '--');
local_line(ax, S, 'truth.grade_long', 1, 'terrain grade ahead', deg, ':');
axs(end+1) = ax;
ax = local_ax(tl, 4, 'Body accelerations', 'acceleration [m/s^2]');
local_line(ax, S, 'telem.a_ref', 1, 'MCU reference (forward)', [], 'LineWidth', 1.2);
local_line(ax, S, 'truth.acc_B', 1, 'forward (truth)');
local_line(ax, S, 'truth.acc_B', 2, 'lateral (truth)');
local_line(ax, S, 'imu_raw.Acceleration', 1:2, {'IMU x', 'IMU y'}, [], ':');
axs(end+1) = ax;
ax = local_ax(tl, 5, 'GNSS fix and satellites', 'fix type / satellites');
local_line(ax, S, 'gnss_raw.fixType', 1, 'fix (0 none, 1 single, 4 RTK fixed, 5 RTK float)', [], 'LineWidth', 1.2);
local_line(ax, S, 'gnss_raw.numSV', 1, 'satellites');
yyaxis(ax, 'right');  ylabel(ax, 'accuracy [m]');
local_line(ax, S, 'gnss_raw.hAcc', 1, 'horizontal accuracy');
local_line(ax, S, 'nav_state.tSinceAbsUpdate', 1, 'time since GNSS / LiDAR update [s]', [], '--');
yyaxis(ax, 'left');
axs(end+1) = ax;
ax = local_ax(tl, 6, 'Navigation flags (high = true)', '');
local_digital(ax, S, {'nav_state.aligned', 'aligned', 0; 'nav_state.navValid', 'nav valid', 0; ...
    'nav_state.headingValid', 'heading valid', 0; 'nav_state.lidarLocActive', 'LiDAR localisation', 2; ...
    'gnss_raw.headingValid', 'GNSS heading valid', 0; 'gnc_status.nav_ok', 'GNC nav OK', 0});
axs(end+1) = ax;
ax = local_ax(tl, 7, 'Track slip: estimate vs truth', '[-]');
local_line(ax, S, 'truth.slip_LR', 1, 'L truth', [], 'Color', cL);
local_line(ax, S, 'truth.slip_LR', 2, 'R truth', [], 'Color', cR);
if info.nav
    local_line(ax, S, 'nav_state.slip_LR', 1, 'L estimate', [], '--', 'Color', cL);
    local_line(ax, S, 'nav_state.slip_LR', 2, 'R estimate', [], '--', 'Color', cR);
end
axs(end+1) = ax;
ax = local_ax(tl, 8, 'Ground contact: normal force per track', 'force [kN]');
[t, N] = local_sig(S, 'struct_diag.N_pt');
if size(N, 2) >= 14
    plot(ax, t, sum(N(:, 1:7), 2)/1000, 'Color', cL, 'DisplayName', 'left track');
    plot(ax, t, sum(N(:, 8:14), 2)/1000, 'Color', cR, 'DisplayName', 'right track');
end
axs(end+1) = ax;
local_finish(axs, S, info, o);
end

% ================================================================================================ helpers
function [S, info] = local_load(src)
try
    root = char(currentProject().RootFolder);
catch
    root = fileparts(fileparts(mfilename('fullpath')));
end
d = fullfile(root, 'work', 'results');
if isstruct(src)
    r = src;  file = '';
    if isfield(r, 'file'), file = char(r.file); end
else
    src = char(src);
    if isfile(src)
        file = src;
    elseif isfile(fullfile(d, src))
        file = fullfile(d, src);
    elseif isfile(fullfile(d, [src '.mat']))
        file = fullfile(d, [src '.mat']);
    else                                            % '' = newest log, otherwise newest log of that scenario ID
        pat = '*.mat';
        if ~isempty(src), pat = [src '_*.mat']; end
        L = dir(fullfile(d, pat));
        if isempty(L), error('plot_run:noLog', 'No run log %s in %s.', pat, d); end
        [~, i] = max([L.datenum]);
        file = fullfile(L(i).folder, L(i).name);
    end
    fprintf('Plotting %s\n', file);
    r = load(file);
end
if ~isfield(r, 'S') || ~isstruct(r.S)
    error('plot_run:noData', 'The run has no logged signals (it stopped before logging started).');
end
S = r.S;
info.root = root;
info.name = 'run';
if ~isempty(file), [~, info.name] = fileparts(file); end
info.world_id = NaN;  info.occ = [];                % the run's world and its occupancy map (run_scenario: viz)
if isfield(r, 'viz') && isstruct(r.viz)
    if isfield(r.viz, 'world_id'), info.world_id = double(r.viz.world_id); end
    if isfield(r.viz, 'occ'), info.occ = r.viz.occ; end
end
[t, p] = local_sig(S, 'truth.pos_W');
info.dist = 0;
if ~isempty(p), info.dist = sum(vecnorm(diff(p(:, 1:2)), 2, 2)); end
[~, pe] = local_sig(S, 'nav_state.pos_W');
[~, nv] = local_sig(S, 'nav_state.navValid');
info.nav = any(pe(:) ~= 0) || any(nv(:) ~= 0);    % navigation fusion runs (Phase 2 variants)
[tm, ~] = local_sig(S, 'telem.mode');
if isempty(tm), tm = t; end
info.trange = [0 1];
if numel(tm) > 1, info.trange = [tm(1) tm(end)]; end
[~, e] = local_sig(S, 'power_truth.E_out_Wh');
info.title = sprintf('%s  |  %.0f s, %.1f m', info.name, info.trange(2), info.dist);
if ~isempty(e), info.title = sprintf('%s, %.1f Wh', info.title, e(end) - e(1)); end
end

function [t, v] = local_sig(S, p)
% logged signal 'group.field' (or 'group.sub.field') -> time column and double values; [] when not logged
t = [];  v = [];
x = S;
for k = strsplit(p, '.')
    if ~isstruct(x) || ~isfield(x, k{1}), return, end
    x = x.(k{1});
end
if ~isstruct(x) || ~isfield(x, 'v') || ~isfield(x, 't') || isempty(x.v), return, end
t = double(x.t(:));
if mod(numel(x.v), numel(t)) ~= 0, t = [];  return, end
v = reshape(double(x.v), numel(t), []);
end

function v = local_at(S, p, t)
% signal p at the times t (signals of different groups may be logged at different rates)
[ts, v] = local_sig(S, p);
if isempty(v) || isempty(t), v = [];  return, end
if numel(ts) ~= numel(t) || any(ts ~= t)
    [ts, iu] = unique(ts);
    v = interp1(ts, v(iu, :), t, 'previous', NaN);
end
end

function h = local_line(ax, S, p, cols, name, fx, varargin)
% plot columns cols of signal p (fx: unit conversion, [] = none); nothing when the signal was not logged
h = gobjects(0);
[t, v] = local_sig(S, p);
if isempty(v), return, end
if isempty(cols), cols = 1:size(v, 2); end
cols = cols(cols <= size(v, 2));
if isempty(cols), return, end
y = v(:, cols);
if nargin >= 6 && ~isempty(fx), y = fx(y); end
h = plot(ax, t, y, varargin{:});
if isempty(name)
    set(h, 'HandleVisibility', 'off');
else
    name = cellstr(name);
    for k = 1:numel(h), h(k).DisplayName = name{min(k, numel(name))}; end
end
end

function local_digital(ax, S, specs)
% boolean signals, one row each from the top: thick bar where true (green = normal, red = problem,
% blue = information; specs column 3: 0 / 1 / 2), thin grey line where false
cols = [0.2 0.65 0.3; 0.85 0.15 0.15; 0.2 0.45 0.8];
rows = {};
for k = 1:size(specs, 1)
    [t, v] = local_sig(S, specs{k, 1});
    if isempty(v), continue, end
    y = -numel(rows);
    on = v(:, 1) ~= 0;
    yon = nan(size(t));  yon(on) = y;
    yoff = nan(size(t));  yoff(~on) = y;
    c = cols(1, :);
    if size(specs, 2) >= 3, c = cols(specs{k, 3} + 1, :); end
    plot(ax, t, yoff, '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(ax, t, yon, 's-', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 2, 'LineWidth', 5, ...
        'HandleVisibility', 'off');
    rows{end+1} = specs{k, 2}; %#ok<AGROW>
end
if isempty(rows), return, end
n = numel(rows);
yticks(ax, -(n - 1):0);  yticklabels(ax, flip(rows));  ylim(ax, [-n + 0.4, 0.6]);
end

function local_mode(ax, S)
modes = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', 'CTRL_STOP', 'ESTOP', 'FAULT'};
[t, m] = local_sig(S, 'telem.mode');
if isempty(m), return, end
stairs(ax, t, m(:, 1), 'LineWidth', 1.5, 'HandleVisibility', 'off');
yticks(ax, 0:9);  yticklabels(ax, modes);  ylim(ax, [-0.5 9.5]);
end

function local_bits(ax, S)
% MCU safety supervisor words (docs/icd.md, fault word): warning = condition present, fault = persisted
% bit 22 is STOP_FAIL since 2026-11-15 (review R-01); in runs before that it was WDG_RESET_SEEN (info)
names = {'HW_ESTOP', 'REMOTE_ESTOP', 'RC_LINK_LOST', 'RC_FAILSAFE', 'COMP_TIMEOUT', 'DRV_L_TIMEOUT', ...
    'DRV_R_TIMEOUT', 'DRV_L_FAULT', 'DRV_R_FAULT', 'BMS_TIMEOUT', 'BMS_ALARM', 'BMS_WARN', 'ENC_L_MISMATCH', ...
    'ENC_R_MISMATCH', 'TILT_WARN', 'TILT_STOP', 'OVERSPEED', 'HV_LOST', 'HV_OPENING', 'RAIL_UV', ...
    'ENCL_TEMP_WARN', 'ENCL_TEMP_CRIT', 'STOP_FAIL', 'TASK_OVERRUN', 'IMU_FAULT', 'CAN1_BUSOFF', ...
    'CAN2_BUSOFF', 'SAFETY_CHAIN_DISCREPANCY', 'IMPL_FAULT', 'BUMPER', 'LID_OPEN', 'SAFETY_RELAY_OPEN'};
[t, fw] = local_sig(S, 'telem.fault_word');
ww = local_at(S, 'telem.warn_word', t);
if isempty(fw), return, end
if isempty(ww), ww = zeros(size(fw)); end
fw = uint32(fw(:, 1));  ww = uint32(ww(:, 1));
used = find(arrayfun(@(b) any(bitget(fw, b + 1)) || any(bitget(ww, b + 1)), 0:31)) - 1;
if isempty(used)
    local_note(ax, 'no fault or warning bit was set');
    return
end
cw = [0.95 0.6 0.1];  cf = [0.85 0.1 0.1];
for k = 1:numel(used)
    b = used(k);
    yw = nan(size(t));  yw(bitget(ww, b + 1) > 0) = k;
    yf = nan(size(t));  yf(bitget(fw, b + 1) > 0) = k;
    hw = plot(ax, t, yw, 's-', 'Color', cw, 'MarkerFaceColor', cw, 'MarkerSize', 3, 'LineWidth', 4, ...
        'DisplayName', 'warning (condition present)');
    hf = plot(ax, t, yf, 's-', 'Color', cf, 'MarkerFaceColor', cf, 'MarkerSize', 4, 'LineWidth', 6, ...
        'DisplayName', 'fault (persisted or latched)');
    if k > 1, set([hw hf], 'HandleVisibility', 'off'); end
end
yticks(ax, 1:numel(used));  ylim(ax, [0.4 numel(used) + 0.6]);
yticklabels(ax, arrayfun(@(b) sprintf('%d %s', b, names{b + 1}), used, 'UniformOutput', false));
end

function local_path(ax, S, info)
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');  axis(ax, 'equal');
title(ax, 'Path on the field (colour = forward speed)', 'FontWeight', 'normal');
xlabel(ax, 'East [m]');  ylabel(ax, 'North [m]');
[~, p] = local_sig(S, 'truth.pos_W');
if isempty(p), local_note(ax, 'position not logged');  return, end
local_field(ax, info);
plot(ax, p(:, 1), p(:, 2), '-', 'Color', [0.8 0.8 0.8], 'HandleVisibility', 'off');
[~, v] = local_sig(S, 'truth.speed_fwd');
if size(v, 1) == size(p, 1)
    scatter(ax, p(:, 1), p(:, 2), 9, v(:, 1), 'filled', 'DisplayName', 'path (truth)');
    cb = colorbar(ax);  cb.Label.String = 'speed [m/s]';
end
[~, pe] = local_sig(S, 'nav_state.pos_W');
if info.nav, plot(ax, pe(:, 1), pe(:, 2), 'k:', 'DisplayName', 'navigation estimate'); end
[~, oom] = local_sig(S, 'kpi.out_of_map');
if size(oom, 1) == size(p, 1) && any(oom(:, 1))
    plot(ax, p(oom(:, 1) > 0, 1), p(oom(:, 1) > 0, 2), 'r.', 'MarkerSize', 6, 'DisplayName', 'outside the field map');
end
plot(ax, p(1, 1), p(1, 2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'start');
plot(ax, p(end, 1), p(end, 2), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'end');
lo = min(p(:, 1:2), [], 1);  hi = max(p(:, 1:2), [], 1);
m = max(3, 0.1*max(hi - lo));
xlim(ax, [lo(1) - m, hi(1) + m]);  ylim(ax, [lo(2) - m, hi(2) + m]);
legend(ax, 'Location', 'best', 'FontSize', 8);
end

function local_field(ax, info)
% field edge and obstacles of the run's world (P.world when it matches, else rebuilt by param_world); the row field
% (world 7, no obstacles) is sized to the rows of its run, so its edge comes from the run's own map (viz.occ)
if ~isfinite(info.world_id), return, end
W = [];
if info.world_id == 7 && all(isfield(info.occ, {'res', 'origin', 'grid'}))
    occ = info.occ;                                 % origin = lower-left corner, grid = ny x nx cells at res cells/m
    W = struct('xlim', occ.origin(1) + [0 size(occ.grid, 2)]/occ.res, ...
        'ylim', occ.origin(2) + [0 size(occ.grid, 1)]/occ.res, 'obstacles', struct('nCyl', 0, 'nBox', 0));
end
if isempty(W)
    try
        W = evalin('base', 'P.world');
        if W.id ~= info.world_id, W = []; end
    catch
    end
end
if isempty(W)
    try
        W = param_world(info.world_id, 9.81);
    catch
        return
    end
end
c = [0.35 0.6 0.25];
plot(ax, W.xlim([1 2 2 1 1]), W.ylim([1 1 2 2 1]), '--', 'Color', c, 'LineWidth', 1.2, 'DisplayName', 'field map edge');
for k = 1:W.obstacles.nCyl
    o = W.obstacles.cyl(k, :);
    rectangle(ax, 'Position', [o(1) - o(3), o(2) - o(3), 2*o(3), 2*o(3)], 'Curvature', [1 1], 'FaceColor', c);
end
for k = 1:W.obstacles.nBox
    o = W.obstacles.box(k, :);
    q = [-1 -1; 1 -1; 1 1; -1 1; -1 -1] .* (o([4 5])/2);
    R = [cos(o(7)) -sin(o(7)); sin(o(7)) cos(o(7))];
    q = q*R' + o([1 2]);
    patch(ax, q(:, 1), q(:, 2), c, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end

function local_finish(axs, S, info, o)
% mode bands, notes for empty tiles, legends, linked time axes
[tm, m] = local_sig(S, 'telem.mode');
bands = {[0 1 2], [0.5 0.5 0.5]; 4, [1 0.8 0.1]; [8 9], [0.9 0.15 0.15]};
for k = 1:numel(axs)
    ax = axs(k);
    xlabel(ax, 'time [s]');
    sides = {'left'};
    if numel(ax.YAxis) > 1, sides = {'right', 'left'}; end
    kids = gobjects(0);
    for s = 1:numel(sides)
        if numel(sides) > 1, yyaxis(ax, sides{s}); end
        c = local_kids(ax);
        kids = [kids; c]; %#ok<AGROW>
    end
    if ~strcmp(ax.Tag, 'noted')
        if isempty(kids)
            local_note(ax, 'not logged in this run');
        elseif all(arrayfun(@(h) all(h.YData(isfinite(h.YData)) == 0), kids))
            local_note(ax, 'all zero in this run (not active in this model variant)');
        end
    end
    if ~isempty(m)
        for b = 1:size(bands, 1)
            in = ismember(m(:, 1), bands{b, 1});
            dd = diff([false; in; false]);
            i1 = find(dd == 1);  i2 = min(find(dd == -1), numel(tm));
            if ~isempty(i1)
                xregion(ax, tm(i1), tm(i2), 'FaceColor', bands{b, 2}, 'FaceAlpha', 0.1, 'HandleVisibility', 'off');
            end
        end
    end
    for s = 1:numel(sides)                          % y range from the data after the boot transient
        if numel(sides) > 1, yyaxis(ax, sides{s}); end
        if strcmp(ax.YLimMode, 'auto'), local_yfit(ax, info.trange(1) + 1.5); end
    end
    if any(arrayfun(@(h) ~isempty(h.DisplayName) && strcmp(h.HandleVisibility, 'on'), kids))
        legend(ax, 'Location', 'best', 'FontSize', 7);
    end
end
setappdata(ancestor(axs(1), 'figure'), 'plot_run_xlink', linkprop(axs, 'XLim'));   % linkaxes is ~20x slower
tr = info.trange;
if ~isempty(o.TimeRange), tr = o.TimeRange; end
if tr(2) > tr(1), xlim(axs(1), tr); end
end

function local_yfit(ax, tcut)
% fit the active y axis to the data at t >= tcut (rate-transition initial values at t = 0 are often 0,
% which would otherwise squeeze 53 V, 95 % or 310 K signals against the axis limit)
c = local_kids(ax);
y = [];
for k = 1:numel(c)
    x = c(k).XData;  yk = c(k).YData;
    if numel(x) ~= numel(yk), continue, end
    y = [y, yk(x >= tcut & isfinite(yk))]; %#ok<AGROW>
end
if isempty(y), return, end
lo = min(y);  hi = max(y);
if hi - lo < 1e-9*max(1, abs(hi)), lo = lo - max(1, 0.05*abs(lo));  hi = hi + max(1, 0.05*abs(hi)); end
pad = 0.06*(hi - lo);
ylim(ax, [lo - pad, hi + pad]);
end

function c = local_kids(ax)
% data objects of the active y side (allchild also returns the hidden lines, but for both yyaxis sides)
if numel(ax.YAxis) > 1
    c = ax.Children;                                % yyaxis tiles draw only named (visible) lines
else
    c = allchild(ax);
end
c = c(isprop(c, 'YData') & isprop(c, 'XData'));
end

function ax = local_ax(tl, n, ttl, ylab, span)
if nargin < 5, span = [1 1]; end
ax = nexttile(tl, n, span);
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
title(ax, ttl, 'FontWeight', 'normal', 'Interpreter', 'none');
ylabel(ax, ylab);
set(ax, 'TickLabelInterpreter', 'none');
end

function [f, tl] = local_fig(name, info, o, rows, cols)
ss = get(groot, 'ScreenSize');
w = min(1500, ss(3) - 80);  h = min(950, ss(4) - 120);
if ~isempty(o.Size), w = o.Size(1);  h = o.Size(2); end
f = figure('Name', sprintf('%s - %s', name, info.name), 'NumberTitle', 'off', 'Color', 'w', ...
    'Visible', o.Visible, 'Position', [40, max(40, ss(4) - h - 80), w, h]);
tl = tiledlayout(f, rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s  -  %s', name, info.title), 'FontSize', 12, 'Interpreter', 'none');
end

function local_savefig(f, file)
% the page as a .fig file that opens visible also when it was drawn hidden (savefig keeps Visible off; the CreateFcn
% saved with it turns the figure on when the file is opened); the page keeps its own CreateFcn
fcn = f.CreateFcn;
f.CreateFcn = 'set(gcbo, ''Visible'', ''on'')';
c = onCleanup(@() set(f, 'CreateFcn', fcn));
savefig(f, file);
end

function local_note(ax, msg)
text(ax, 0.5, 0.5, msg, 'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.45 0.45 0.45]);
ax.Tag = 'noted';
end

function x = local_pct(x)
if max(abs(x(:))) <= 1.0001, x = 100*x; end
end

function [cL, cR] = local_lr()
cL = [0 0.447 0.741];  cR = [0.85 0.325 0.098];
end
