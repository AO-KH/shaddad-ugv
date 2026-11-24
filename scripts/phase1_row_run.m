function phase1_row_run(folder)
%PHASE1_ROW_RUN  Run one RC row test and save its summary and every plot page in its folder (the worker of
%   phase1_row_test).
%
%   phase1_row_run(folder)     folder = work/reports/rowtest_<yyyymmdd_HHMMSS> with inputs.mat, from phase1_row_test
%
%   phase1_row_test starts it in a separate MATLAB (matlab -batch, after openProject); with 'Background', false it runs
%   in the calling MATLAB, which it keeps busy for the whole run. Steps: the inputs (inputs.mat: cfg = param_rowtest()
%   with the five inputs) are checked again (row_test_inputs), so a bad folder ends before any simulation; scenario R1
%   runs in Normal mode with no time limit (it ends at its stop time or earlier, docs/row_test.md; 'KeepOut' false: the
%   run keeps its log res.S in memory, not the whole SimulationOutput); then the summary and the field page; then the
%   log leaves memory (res.S = []) before plot_run loads it again from its file for the six pages, each page title
%   with the fidelity of the run ('Note': full detail | fast). It writes into the folder:
%     progress.log                    what it prints (diary), each step with its time of day; the full report of an error
%     summary.txt                     row_test_summary; an error of any step adds the line "error: <message>" (written
%                                     alone when there is no summary yet)
%     summary.mat                     M (row_test_summary), rows (row_test_rows), cfg, log_file (the run log in
%                                     work/results: res.file; the log is not copied)
%     1_drive ... 6_nav .png / .fig   the plot_run pages
%     7_field.png / 7_field.fig       the field page (row_test_field_page; the .fig opens visible)
%     done.txt                        on every exit, last (the finish time)
%   and deletes work/rowtest.lock when it names this folder (a run started later with 'Force' keeps its own lock).
%   It opens nothing; on every exit the diary and the warning states are put back as they were before the call.

folder = char(folder);
lockfile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'work', 'rowtest.lock');
dy = {get(0, 'Diary'), get(0, 'DiaryFile')};
ws = warning;
cleanup = onCleanup(@() local_finish(folder, lockfile, dy, ws));
warning('off', 'all');                                 % as phase1_test_drive: the model's warnings stay out of the log
diary(fullfile(folder, 'progress.log'));
local_say('RC row test in %s', folder);
try
    s = load(fullfile(folder, 'inputs.mat'), 'cfg');
    [ok, msg, cfg, est_min] = row_test_inputs(s.cfg, param_rowtest());
    if ~ok
        error('phase1_row_run:inputs', 'inputs.mat: %s', msg);
    end
    onoff = {'off', 'on'};
    detail = {'full detail', 'fast'};
    local_say('%d rows, %g min, %g degC, sprayer %s, %s; estimated %.0f min', cfg.n_rows, cfg.minutes, cfg.T_amb_C, ...
        onoff{cfg.sprayer + 1}, detail{cfg.fast + 1}, est_min);
    rows = row_test_rows(cfg);
    args = {'Rows', cfg.n_rows, 'Minutes', cfg.minutes, 'Ambient', cfg.T_amb_C, 'Sprayer', cfg.sprayer == 1, ...
        'Fast', cfg.fast == 1};
    local_say('Scenario R1: compile and run (Normal mode)');
    res = run_scenario('R1', 'Args', args, 'Mode', 'normal', 'KeepOut', false);
    run = struct('wall_s', res.wall_s, 'rtf', res.rtf, 't_end', res.t_end, 'file', res.file, 'msg', res.msg);
    [txt, M] = row_test_summary(res.S, run, cfg, rows);
    writelines(txt, fullfile(folder, 'summary.txt'));
    log_file = res.file;
    save(fullfile(folder, 'summary.mat'), 'M', 'rows', 'cfg', 'log_file');
    local_say('Summary written: %s, %d rows completed', M.end_reason, M.rows_done);
    local_field_page(res, M, rows, cfg, folder);
    res.S = [];                                        % out of memory before plot_run loads it again (240 min: ~1 GB)
    figs = plot_run(res.file, 'Save', true, 'Folder', folder, 'Fig', true, 'Visible', 'off', ...
        'Note', detail{cfg.fast + 1});
    close(figs);
    local_say('Plot pages saved');
catch e
    local_error(folder, e);
end
end

% -------------------------------------------------------------------------------------------
function local_field_page(res, M, rows, cfg, folder)
% the field page as 7_field.png (taken as plot_run takes its pages) and 7_field.fig (drawn hidden: opens visible)
[~, name] = fileparts(res.file);
f = row_test_field_page(res.S, M, rows, 'Visible', 'off', 'Name', name, 'Cfg', cfg);
c = onCleanup(@() close(f));
fr = getframe(f);
imwrite(fr.cdata, fullfile(folder, '7_field.png'));
f.CreateFcn = 'set(gcbo, ''Visible'', ''on'')';
savefig(f, fullfile(folder, '7_field.fig'));
end

function local_error(folder, e)
% an error of a step: its full report to the progress log, the line "error: <message>" to summary.txt
fprintf('%s\n', getReport(e, 'extended', 'hyperlinks', 'off'));
msg = regexprep(strtrim(regexprep(e.message, '</?a(\s[^>]*)?>', '')), '\s+', ' ');   % one line, no hyperlink markup
f = fullfile(folder, 'summary.txt');
if isfile(f)
    writelines({''; ['error: ' msg]}, f, 'WriteMode', 'append');
else
    writelines({'RC row test summary'; ''; ['error: ' msg]}, f);
end
local_say('Ended with an error: %s', msg);
end

function local_finish(folder, lockfile, dy, ws)
% every exit: this run's lock removed, the progress log closed, diary and warnings as before the call, done.txt last
try
    if isfile(lockfile)                                % none: a dry-run folder, or removed by hand
        tok = regexp(fileread(lockfile), '^folder[ \t]+([^\r\n]*?)[ \t]*\r?$', 'tokens', 'once', 'lineanchors');
        if ~isempty(tok) && strcmpi(tok{1}, folder)
            delete(lockfile);
        end
    end
catch e
    local_say('The lock %s was not removed: %s', lockfile, e.message);
end
local_say('Finished');
diary('off');
set(0, 'DiaryFile', dy{2});
if strcmp(dy{1}, 'on'), diary('on'); end
warning(ws);
writelines(['finished ' char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'))], fullfile(folder, 'done.txt'));
end

function local_say(fmt, varargin)
% one progress line with the time of day
fprintf(['%s  ' fmt '\n'], char(datetime('now', 'Format', 'HH:mm:ss')), varargin{:});
end
