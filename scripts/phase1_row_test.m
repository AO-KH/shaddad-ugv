function job = phase1_row_test(varargin)
%PHASE1_ROW_TEST  Start an RC row test (Phase 1): a simulated driver drives the Shaddad UGV by remote control up and down
%   crop rows in a separate MATLAB; every plot page and a summary are saved in one folder per run.
%
%   phase1_row_test()                                   a window asks the inputs, last used values pre-filled; Run starts
%   job = phase1_row_test('Rows', 8, 'Minutes', 20, 'Ambient', 40, 'Sprayer', true)   no window; full detail
%   job = phase1_row_test(..., 'Fast', true)          the fast preset instead of full detail
%   job = phase1_row_test(..., 'Window', tf)          the window even with the inputs given, or never (default: when Rows,
%                                                     Minutes, Ambient or Sprayer is missing)
%   job = phase1_row_test(..., 'Background', false)   run in this MATLAB instead (busy for the whole run)
%   job = phase1_row_test(..., 'DryRun', true)        prepare the folder and the command, start nothing
%   job = phase1_row_test(..., 'Force', true)         start even while another row test runs
%
%   Inputs: Rows 2 to 30 (whole number), Minutes 1 to 240 of driving after arming, Ambient -10 to 55 degC, Sprayer
%   (implement 3, the pump on along the rows until the tank is empty), Fast (fast preset: lumped pack, ideal drive,
%   kinematic chassis; off = full detail). Limits and defaults: params/param_rowtest.m. A missing input takes the last
%   used value (work/rowtest_last.mat, written whenever the inputs are accepted), else the default; but without the
%   window a missing Fast is off (a scripted run is full detail unless it asks for Fast).
%
%   The run goes on in a separate MATLAB, as the Phase 1 suite runs its cases: this MATLAB stays free and the memory of
%   the long run is given back when it ends. The start line runs folder\run.cmd in a minimised window "Shaddad row test";
%   run.cmd starts matlab -batch, which opens the project and calls phase1_row_run(folder): scenario R1 in Normal mode
%   without the 3-D view, writing into folder = work/reports/rowtest_<yyyymmdd_HHMMSS>/ progress.log during the run and
%   at the end summary.txt, summary.mat, the six plot_run pages and the field page (PNG and .fig), then done.txt. It
%   opens nothing. When that MATLAB ends without done.txt (licence refused, project start-up error, crash, killed),
%   run.cmd adds "error: the background MATLAB ended (exit <code>) before the row test finished" to summary.txt,
%   removes work\rowtest.lock when it names this folder and writes done.txt.
%   Estimated wall-clock time (row_test_inputs): t_compile + the minutes at the real-time factor of the case (rtf: full
%   detail, full detail with the sprayer, fast, fast with the sprayer), both in params/param_rowtest.m; the measured
%   figures behind them are in docs/row_test.md (How long it takes).
%
%   One row test at a time: work/rowtest.lock (three lines "folder <path>", "start <yyyy-MM-dd HH:mm:ss>",
%   "est_min <estimate>") refuses a new start (error phase1_row_test:busy) while the folder of that run has no done.txt
%   and it started less than 3 x its estimate ago; 'Force' starts all the same. phase1_row_run removes the lock at the
%   end. A dry run checks the lock as a start does, but takes none.
%
%   job: folder, cfg (param_rowtest() with the five inputs, numeric), est_min (estimated wall-clock time, min), cmd (the
%   start line: start "Shaddad row test" /min "<folder>\run.cmd"; phase1_row_run(...) when 'Background' is false, and
%   then no run.cmd). [] when the window is cancelled.

names = {'Rows', 'n_rows'; 'Minutes', 'minutes'; 'Ambient', 'T_amb_C'; 'Sprayer', 'sprayer'; 'Fast', 'fast'};
ip = inputParser;
for k = 1:size(names, 1)
    ip.addParameter(names{k, 1}, []);
end
ip.addParameter('Window', []);
ip.addParameter('Background', true);
ip.addParameter('DryRun', false);
ip.addParameter('Force', false);
ip.parse(varargin{:});
o = ip.Results;
root = char(currentProject().RootFolder);
work = fullfile(root, 'work');
R = param_rowtest();

%% ---- one row test at a time ----
if ~o.Force
    local_check_lock(fullfile(work, 'rowtest.lock'));
end

%% ---- inputs: the arguments, else the last used values, else the defaults; the window when one is missing ----
given = ~ismember(names(:, 1), ip.UsingDefaults);
fast = strcmp(names(:, 1), 'Fast');                    % optional: without the window a missing Fast is off
show = o.Window;
if isempty(show), show = ~all(given(~fast)); end
cfg = R;
last = local_last(fullfile(work, 'rowtest_last.mat'));
for k = 1:size(names, 1)
    if given(k)
        cfg.(names{k, 2}) = o.(names{k, 1});
    elseif isfield(last, names{k, 2}) && (show || ~fast(k))
        cfg.(names{k, 2}) = last.(names{k, 2});
    end
end
if show
    cfg = local_window(cfg, R);
end
if isempty(cfg)
    fprintf('RC row test cancelled.\n');
    job = [];
else
    job = local_start(cfg, R, o, root, names);
end
if nargout == 0, clear job; end
end

% -------------------------------------------------------------------------------------------
function job = local_start(cfg, R, o, root, names)
% check the inputs, prepare the run folder and the command, then start the run (or not: dry run)
[ok, msg, cfg, est_min] = row_test_inputs(cfg, R);
if ~ok
    error('phase1_row_test:inputs', '%s', msg);
end
work = fullfile(root, 'work');
folder = local_new_folder(fullfile(work, 'reports'));
save(fullfile(folder, 'inputs.mat'), 'cfg');
for k = 1:size(names, 1)
    last.(names{k, 2}) = cfg.(names{k, 2});
end
save(fullfile(work, 'rowtest_last.mat'), 'last');
if o.Background
    runcmd = fullfile(folder, 'run.cmd');
    local_runcmd(runcmd, root, folder);
    cmd = sprintf('start "Shaddad row test" /min "%s"', runcmd);
else
    cmd = sprintf('phase1_row_run(''%s'')', strrep(folder, '''', ''''''));   % a quote in the path doubled for MATLAB
end
job = struct('folder', folder, 'cfg', cfg, 'est_min', est_min, 'cmd', cmd);
onoff = {'off', 'on'};
detail = {'full detail', 'fast'};
what = sprintf('%d rows, %g min, %g degC, sprayer %s, %s', cfg.n_rows, cfg.minutes, cfg.T_amb_C, ...
    onoff{cfg.sprayer + 1}, detail{cfg.fast + 1});
if o.DryRun
    fprintf('RC row test, dry run (nothing started): %s; estimated %.0f min.\n  Folder : %s\n  Command: %s\n', ...
        what, est_min, folder, cmd);
    return
end

%% ---- start ----
lockfile = fullfile(work, 'rowtest.lock');
t0 = datetime('now');
writelines({['folder  ' folder]; ['start   ' char(t0, 'yyyy-MM-dd HH:mm:ss')]; sprintf('est_min %.6g', est_min)}, lockfile);
end_txt = char(t0 + minutes(est_min), 'yyyy-MM-dd HH:mm');
if o.Background
    [st, out] = system(cmd);
    if st ~= 0
        delete(lockfile);
        error('phase1_row_test:start', 'The background MATLAB did not start (exit %d): %s', st, strtrim(out));
    end
    fprintf(['RC row test started %s in a separate MATLAB (minimised window "Shaddad row test"): %s.\n' ...
        '  Expected end: about %s (estimated %.0f min, compile included).\n' ...
        '  Results in %s\n  (progress.log during the run; summary.txt, the plot pages and done.txt at the end)\n'], ...
        char(t0, 'yyyy-MM-dd HH:mm:ss'), what, end_txt, est_min, folder);
else
    fprintf('RC row test started %s in this MATLAB: %s. Expected end: about %s. Results in %s\n', ...
        char(t0, 'yyyy-MM-dd HH:mm:ss'), what, end_txt, folder);
    phase1_row_run(folder);
end
end

function local_runcmd(file, root, folder)
% folder\run.cmd: the background MATLAB of the run, then a guard for a MATLAB that ended without done.txt (licence
% refused, project start-up error, crash, killed): the error line with its exit code in summary.txt (written when
% missing), work\rowtest.lock removed when its first line names this folder, done.txt. Every path is quoted (the project
% path has a space); a % in a path is doubled for cmd, a ' also for the MATLAB text.
b = @(s) strrep(s, '%', '%%');
m = @(s) b(strrep(s, '''', ''''''));
in = @(name) ['"' b(fullfile(folder, name)) '"'];
lock = ['"' b(fullfile(root, 'work', 'rowtest.lock')) '"'];
writelines({
    '@echo off'
    'rem RC row test, written by phase1_row_test: the run in a separate MATLAB, then the guard for a MATLAB that ended early'
    sprintf('"%s" -batch "cd(''%s''); openProject(''Shaddad_UGV.prj''); phase1_row_run(''%s'')"', ...
        b(fullfile(matlabroot, 'bin', 'matlab.exe')), m(root), m(folder))
    'set "code=%errorlevel%"'
    ['if exist ' in('done.txt') ' exit %code%']
    ['if not exist ' in('summary.txt') ' >' in('summary.txt') ' echo RC row test summary']
    ['>>' in('summary.txt') ' echo.']
    ['>>' in('summary.txt') ' echo error: the background MATLAB ended (exit %code%) before the row test finished']
    'set "lock="'
    ['if exist ' lock ' set /p lock=<' lock]
    ['if /i "%lock%"=="folder  ' b(folder) '" del ' lock]
    ['>' in('done.txt') ' echo finished %date% %time%']
    'exit %code%'}, file);
end

function local_check_lock(lockfile)
% error phase1_row_test:busy while the run named in the lock is still going: its folder has no done.txt and it started
% less than 3 x its estimate ago (a lock that cannot be read, or whose folder is gone, is left over: no run)
L = local_read_lock(lockfile);
if isempty(L) || ~isfolder(L.folder) || isfile(fullfile(L.folder, 'done.txt'))
    return
end
if minutes(datetime('now') - L.start) < 3*L.est_min
    error('phase1_row_test:busy', ['A row test is still running in %s (started %s, estimated %.0f min). Wait for ' ...
        'done.txt in that folder, or start anyway with ''Force'', true.'], ...
        L.folder, char(L.start, 'yyyy-MM-dd HH:mm:ss'), L.est_min);
end
end

function L = local_read_lock(lockfile)
% the lock as a struct folder, start (datetime), est_min; [] when there is none or it cannot be read
L = [];
if ~isfile(lockfile), return, end
try
    tok = regexp(fileread(lockfile), '^(folder|start|est_min)[ \t]+([^\r\n]*?)[ \t]*\r?$', 'tokens', 'lineanchors');
    for k = 1:numel(tok)
        v.(tok{k}{1}) = tok{k}{2};
    end
    L = struct('folder', v.folder, 'start', datetime(v.start, 'InputFormat', 'yyyy-MM-dd HH:mm:ss'), ...
        'est_min', str2double(v.est_min));
    if isnan(L.est_min) || isnat(L.start), L = []; end
catch
    L = [];
end
end

function last = local_last(file)
% the inputs of the last start (work/rowtest_last.mat: struct last), struct() when there are none
last = struct();
if ~isfile(file), return, end
try
    s = load(file, 'last');
    last = s.last;
catch
end
end

function cfg = local_window(cfg, R)
% the start window (row_test_window) until Run or Cancel: the inputs checked by row_test_inputs, [] when cancelled
f = row_test_window(cfg, R);
c = onCleanup(@() delete(f(isvalid(f))));
uiwait(f);
cfg = [];
if isvalid(f) && strcmp(f.UserData.choice, 'run')
    cfg = f.UserData.cfg;
end
end

function folder = local_new_folder(reports)
% a new run folder reports/rowtest_<yyyymmdd_HHMMSS>; a second one prepared within the same second waits for the next
stamp = @() ['rowtest_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
folder = fullfile(reports, stamp());
while isfolder(folder)
    pause(0.1);
    folder = fullfile(reports, stamp());
end
mkdir(folder);
end
