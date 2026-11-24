function files = shaddad_layout_runs(tag, cases)
%SHADDAD_LAYOUT_RUNS  Compare-set runs of the model layout cleanup, one run file per case (spec 9).
%
%   files = shaddad_layout_runs(tag)            the quick set {'S0', 'S1', 'S8a', 'R1'}
%   files = shaddad_layout_runs(tag, cases)     cases: a cellstr of scenario ids, or 'quick' | 'full' (the full
%                                               set adds 'S2' and 'S6c')
%   tag     name of the run set (letters, digits, _ and -), e.g. 'today', 'det1': the files are
%           work/layout/runs/<tag>/<id>.mat (folders created when missing, an earlier file of the case replaced)
%   files   cellstr, the paths of the files written, in the order of the cases
%
%   Each case runs in this MATLAB session as run_scenario(id, 'Save', false); R1 with 'Args', {'Rows', 4,
%   'Minutes', 2, 'Ambient', 45, 'Sprayer', true} (a 2-minute row test on 4 rows at 45 degC with the sprayer).
%   The run file holds the struct res, without the SimulationOutput:
%     .id      scenario id
%     .S       logged signals in the scn_collect form (S.<log>.<path>.t / .v); struct() when the run gave none
%     .wall_s  wall-clock time of the run_scenario call [s] (input, compile, simulation and log collection)
%     .rtf     simulated seconds (run_scenario's t_end) / wall_s
%     .ok      run_scenario's ok (the simulation ended without an error); false when run_scenario threw
%     .msg     run_scenario's message, or the error text (with its causes) when it threw
%   A failing case is recorded, not fatal: its file is written with ok false and msg, and the next case runs.
%   The base workspace changes only as far as run_scenario changes it. Compare two run files with
%   shaddad_compare_logs (the bit-identical rule needs each case to be deterministic).

if nargin < 2 || isempty(cases), cases = 'quick'; end
tag = char(tag);
if isempty(regexp(tag, '^[\w-]+$', 'once'))
    error('shaddad_layout_runs:tag', 'The tag names a folder (letters, digits, _ and -), not ''%s''', tag);
end
cases = local_cases(cases);
d = fullfile(char(currentProject().RootFolder), 'work', 'layout', 'runs', tag);
if ~isfolder(d), mkdir(d); end
files = cell(size(cases));
for k = 1:numel(cases)
    id = cases{k};
    args = {};
    if strcmp(id, 'R1')
        args = {'Args', {'Rows', 4, 'Minutes', 2, 'Ambient', 45, 'Sprayer', true}};
    end
    res = struct('id', id, 'S', struct(), 'wall_s', NaN, 'rtf', NaN, 'ok', false, 'msg', '');
    t0 = tic;
    try
        r = run_scenario(id, 'Save', false, args{:});
        res.wall_s = toc(t0);
        if ~isempty(r.S), res.S = r.S; end
        res.rtf = r.t_end/res.wall_s;
        res.ok = logical(r.ok);
        res.msg = r.msg;
    catch tmp_lr_err
        res.wall_s = toc(t0);
        res.msg = local_err_text(tmp_lr_err);
        warning('shaddad_layout_runs:failed', 'Case %s failed: %s', id, res.msg);
    end
    r = []; %#ok<NASGU> the SimulationOutput is released before the next case
    files{k} = fullfile(d, [id '.mat']);
    save(files{k}, 'res', '-v7.3');
    fprintf('shaddad_layout_runs %s/%s: ok=%d, wall %.1f s, RTF %.3g -> %s\n', tag, id, res.ok, res.wall_s, ...
        res.rtf, files{k});
end
end

% -------------------------------------------------------------------------------------------
function c = local_cases(c)
% the case list: a cellstr or string array of ids, or the name of a set
quick = {'S0', 'S1', 'S8a', 'R1'};
if (ischar(c) && isrow(c)) || (isstring(c) && isscalar(c))
    switch lower(char(c))
        case 'quick'
            c = quick;
        case 'full'
            c = [quick, {'S2', 'S6c'}];
        otherwise
            error('shaddad_layout_runs:cases', 'cases must be a cellstr of scenario ids, ''quick'' or ''full''');
    end
elseif isstring(c) || iscellstr(c)
    c = cellstr(c);
else
    error('shaddad_layout_runs:cases', 'cases must be a cellstr of scenario ids, ''quick'' or ''full''');
end
if ~all(cellfun(@(s) ~isempty(regexp(s, '^\w+$', 'once')), c))
    error('shaddad_layout_runs:cases', 'A case id is a scenario id such as ''S8a'' (letters, digits, _)');
end
end

function s = local_err_text(e)
% error message with its causes (as run_scenario)
s = e.message;
for k = 1:numel(e.cause)
    s = sprintf('%s | %s', s, local_err_text(e.cause{k}));
end
end
