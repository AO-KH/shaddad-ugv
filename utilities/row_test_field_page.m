function f = row_test_field_page(S, M, rows, varargin)
%ROW_TEST_FIELD_PAGE  Field page of an RC row test run (spec section 9, RC row test): the path over the rows, rows
%   completed, distance from the row line and energy per row, one page in the look of the plot_run pages.
%   f = row_test_field_page(S, M, rows)
%   f = row_test_field_page(S, M, rows, 'Visible', 'off', 'Name', 'R1_20261028_101500', 'Cfg', cfg)
%     S     run log as run_scenario saves it (see row_test_summary); only the true path truth.pose2D is read, N x 3 [x y psi]
%           (m, m, rad); S may be [] or lack the signal: the tiles that need it then show a note
%     M     the figures behind the summary, 2nd output of row_test_summary (fields read: rows_done, passes, t_done, E_row_Wh,
%           dy_max, fidelity)
%     rows  centre lines of the field, n x 5 [x1 y1 x2 y2 laneId] (row_test_rows(cfg)), at least one
%     'Visible'  'on' (default) | 'off': a hidden figure for batch use; the caller closes it
%     'Name'     name of the run log, for the title (char, default '': left out)
%     'Cfg'      inputs of the run, the cfg of row_test_summary (fields read: minutes, T_amb_C, sprayer), for the title
%                (default []: the title gives only what rows and M hold)
%   f   figure of 2 x 2 tiles; each axes has the Tag shown here
%       (a) 'path'       the rows (green, numbered) and the true path, coloured by time; start and end marked
%       (b) 'rows_done'  rows completed against time: a step at every completion M.t_done, repeats counted
%       (c) 'offset'     distance from the row line against time, m, + = left of the row direction (north for the
%                        east-pointing rows of row_test_rows), the sign of kpi_truth_xte: the signed distance to the nearest
%                        row line over the samples that lie along a row (between its two ends: the samples of M.dy_max, so
%                        the largest |value| is M.dy_max); NaN for the others (headlands, U-turns), so each pass is a trace
%       (d) 'energy'     energy per row, Wh, M.E_row_Wh: a bar per completed row in the order of completion; a row without a
%                        figure (NaN: the kpi log ended before it) is marked n/a
%   The time axes of (b) and (c) are linked. The title gives the log name, the fidelity and the inputs.

ip = inputParser;
ip.addParameter('Visible', 'on');
ip.addParameter('Name', '');
ip.addParameter('Cfg', []);
ip.parse(varargin{:});
o = ip.Results;

[t, xy] = local_pose(S);
d = local_offset(xy, rows);
tr = [0 1];                                            % time span of the log, s
if numel(t) > 1, tr = [t(1) t(end)]; end
t_done = M.t_done(:);
E = M.E_row_Wh(:);

%% ---- figure and title, as plot_run's local_fig ----
figname = 'Field page';
head = figname;
log_name = char(o.Name);
if ~isempty(log_name)
    head = sprintf('%s  -  %s', figname, log_name);
    figname = sprintf('%s - %s', figname, log_name);
end
onoff = {'off', 'on'};
items = {sprintf('%d rows', size(rows, 1))};           % the inputs the title can give
if isfield(o.Cfg, 'minutes'), items{end + 1} = sprintf('%g min', o.Cfg.minutes); end
if isfield(o.Cfg, 'T_amb_C'), items{end + 1} = sprintf('%g degC', o.Cfg.T_amb_C); end
if isfield(o.Cfg, 'sprayer'), items{end + 1} = ['sprayer ' onoff{double(logical(o.Cfg.sprayer)) + 1}]; end
ss = get(groot, 'ScreenSize');                         % size and place of the plot_run pages
w = min(1500, ss(3) - 80);  h = min(950, ss(4) - 120);
f = figure('Name', figname, 'NumberTitle', 'off', 'Color', 'w', 'Visible', o.Visible, ...
    'Position', [40, max(40, ss(4) - h - 80), w, h]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s  |  %s fidelity  |  %s', head, M.fidelity, strjoin(items, ', ')), 'FontSize', 12, 'Interpreter', 'none');

%% ---- (a) the rows and the true path, coloured by time ----
ax_a = local_ax(tl, 1, 'path', 'Path on the rows (colour = time)');
axis(ax_a, 'equal');
xlabel(ax_a, 'East [m]');  ylabel(ax_a, 'North [m]');
pts = [rows(:, 1:2); rows(:, 3:4); xy];
lo = min(pts, [], 1);  hi = max(pts, [], 1);           % NaN samples are left out
m = max(3, 0.1*max(hi - lo));                          % margin around the field and the path, as plot_run's path
hr = plot(ax_a, rows(:, [1 3])', rows(:, [2 4])', '-', 'Color', [0.6 0.78 0.5], 'LineWidth', 5, 'Tag', 'row');
for r = 1:ceil(size(rows, 1)/10):size(rows, 1)         % row numbers in the left margin, at most ten
    text(ax_a, lo(1) - m/2, rows(r, 2), sprintf('%g', rows(r, 5)), 'HorizontalAlignment', 'center', 'FontSize', 8, ...
        'Color', [0.25 0.5 0.15]);
end
if isempty(t)
    local_note(ax_a, 'path not logged in this run');
else
    plot(ax_a, xy(:, 1), xy(:, 2), '-', 'Color', [0.8 0.8 0.8]);
    hp = scatter(ax_a, xy(:, 1), xy(:, 2), 9, t, 'filled');
    cb = colorbar(ax_a);  cb.Label.String = 'time [s]';
    h0 = plot(ax_a, xy(1, 1), xy(1, 2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    h1 = plot(ax_a, xy(end, 1), xy(end, 2), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    legend(ax_a, [hr(1) hp h0 h1], {'rows (centre lines)', 'path (truth)', 'start', 'end'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 8);   % below the axes: it hides no row
end
xlim(ax_a, [lo(1) - m, hi(1) + m]);  ylim(ax_a, [lo(2) - m, hi(2) + m]);

%% ---- (b) rows completed against time ----
times = 'times';
if M.passes == 1, times = 'time'; end
ax_b = local_ax(tl, 2, 'rows_done', ...
    sprintf('Rows completed: %d (repeats counted), field covered %d %s', M.rows_done, M.passes, times));
xlabel(ax_b, 'time [s]');  ylabel(ax_b, 'rows completed');
n = numel(t_done);
if n == 0
    local_note(ax_b, 'no row was completed');
else
    stairs(ax_b, [tr(1); t_done; tr(2)], [0; (1:n)'; n], 'LineWidth', 1.4);
    ylim(ax_b, [0 1.1*n]);                             % the last step clear of the box
    if n <= 10, yticks(ax_b, 0:n); end                 % whole rows
end

%% ---- (c) distance from the row line ----
ttl = 'Distance from the row line along the rows (+ = north)';
if isfinite(M.dy_max), ttl = sprintf('%s: largest %.2f m', ttl, M.dy_max); end
ax_c = local_ax(tl, 3, 'offset', ttl);
xlabel(ax_c, 'time [s]');  ylabel(ax_c, 'distance [m]');
if any(isfinite(d))
    plot(ax_c, t, d, 'LineWidth', 0.75);
    yline(ax_c, 0, 'k:');
    dm = max(abs(d));                                  % NaN (off the rows) is left out
    if dm > 0, ylim(ax_c, 1.1*[-dm dm]); end           % symmetric about the row line
else
    local_note(ax_c, 'no sample along a row');
end

%% ---- (d) energy per row ----
ax_d = local_ax(tl, 4, 'energy', 'Energy per row (since the previous row, the first since arming)');
xlabel(ax_d, 'row completed (in order)');  ylabel(ax_d, 'energy [Wh]');
if isempty(E)
    local_note(ax_d, 'no row was completed');
elseif all(isnan(E))
    local_note(ax_d, 'energy not logged in this run');
else
    bar(ax_d, (1:numel(E))', E);
    for k = find(isnan(E))'                            % the last row, when the kpi log ended before it
        text(ax_d, k, 0, 'n/a', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8, ...
            'Color', [0.45 0.45 0.45]);
    end
    xlim(ax_d, [0.5, numel(E) + 0.5]);
end

setappdata(f, 'row_test_xlink', linkprop([ax_b ax_c], 'XLim'));   % as plot_run (linkaxes is slower)
if tr(2) > tr(1), xlim(ax_b, tr); end
if nargout == 0, clear f; end
end

% -------------------------------------------------------------------------------------------
function [t, xy] = local_pose(S)
% true path of the log: sample times (N x 1, s) and positions (N x 2, m); empty when the log has none
t = zeros(0, 1);  xy = zeros(0, 2);
if isstruct(S) && isfield(S, 'truth') && isstruct(S.truth) && isfield(S.truth, 'pose2D')
    p = S.truth.pose2D;
    if isstruct(p) && isfield(p, 't') && isfield(p, 'v') && ~isempty(p.t)
        t = double(p.t(:));
        xy = double(p.v(:, 1:2));
    end
end
end

function d = local_offset(xy, rows)
% signed distance of each sample from the nearest row centre line, m, + = left of the row direction; NaN for a sample that
% lies along no row. Along a row = its projection falls between the two ends of the row, the rule of local_dy_max in
% row_test_summary (the same distances, bit for bit), so max(abs(d)) is M.dy_max. N x R arrays: one column per row.
a = rows(:, 1:2)';                                     % 2 x R, start of each row
u = rows(:, 3:4)' - a;
L = hypot(u(1, :), u(2, :));                           % 1 x R, row lengths
u = u./L;                                              % unit vectors along the rows
px = xy(:, 1) - a(1, :);                               % N x R, sample relative to each row start
py = xy(:, 2) - a(2, :);
D = u(1, :).*py - u(2, :).*px;                         % signed distance to each row line
along = u(1, :).*px + u(2, :).*py;                     % position along each row
D(along < 0 | along > L) = NaN;                        % not along that row
[~, k] = min(abs(D), [], 2);                           % nearest row line (NaN is left out; a sample along none keeps NaN)
d = D(sub2ind(size(D), (1:size(D, 1))', k));
end

function ax = local_ax(tl, n, tag, ttl)
% tile n of the layout, in the look of plot_run's local_ax
ax = nexttile(tl, n);
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
title(ax, ttl, 'FontWeight', 'normal', 'Interpreter', 'none');
set(ax, 'TickLabelInterpreter', 'none', 'Tag', tag);
end

function local_note(ax, msg)
% note in the middle of a tile that has nothing to show
text(ax, 0.5, 0.5, msg, 'Units', 'normalized', 'HorizontalAlignment', 'center', 'Color', [0.45 0.45 0.45]);
end
