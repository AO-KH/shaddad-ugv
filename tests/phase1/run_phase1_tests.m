function R = run_phase1_tests(level, varargin)
%RUN_PHASE1_TESTS  Phase 1 (SBUS remote-control) test suite: run the cases, print a summary, write a report.
%
%   R = run_phase1_tests('Quick')      about 20 min: power-up, RC drive, corrupted SBUS, E-stop, 2 unit classes
%   R = run_phase1_tests('Full')       about 1.5-2 h: every Phase 1 scenario, 5 test drives, 9 unit classes
%   R = run_phase1_tests(level, 'Cases', {'S1','D2'})   only these case ids (from the level)
%   R = run_phase1_tests(level, 'Isolate', false)       run in this MATLAB instead of a fresh process per case
%   R = run_phase1_tests(level, 'Open', true)           open the HTML report at the end
%
%   Every case runs in a fresh MATLAB process (`matlab -batch`), because each full-system compile grows the
%   MATLAB memory by about 0.5-1 GB and slows the PC down above about 9 GB. Case list: phase1_cases.m;
%   accepted deviations: phase1_known_issues.m (reported KNOWN, not FAIL).
%
%   Result per case and per check: PASS | KNOWN (accepted known issue) | FAIL | BLOCKED (cannot be checked on
%   this PC) | SKIPPED (test not applicable / gated) | ERROR (the case did not run).
%   Overall verdict: PASS when no case has a FAIL or ERROR.
%   Output: work/reports/phase1_<level>_<yyyymmdd_HHMMSS>/ with report.html, results.mat and one MAT file
%   per case. R: struct with fields level, verdict, cases (per-case results), folder, report.

ip = inputParser;
ip.addParameter('Cases', {});
ip.addParameter('Isolate', true);
ip.addParameter('Open', false);
ip.parse(varargin{:});
o = ip.Results;
if nargin < 1, level = 'Quick'; end

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
C = phase1_cases(level);
if ~isempty(o.Cases)
    C = C(ismember({C.id}, o.Cases));
end
K = phase1_known_issues();
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
folder = fullfile(root, 'work', 'reports', sprintf('phase1_%s_%s', lower(char(level)), stamp));
if ~isfolder(folder), mkdir(folder); end
mexe = fullfile(matlabroot, 'bin', 'matlab.exe');
if ~isfile(mexe), mexe = fullfile(matlabroot, 'bin', 'matlab'); end

fprintf('\n=== Shaddad Phase 1 test suite (%s): %d cases, report folder %s ===\n', level, numel(C), folder);
t0 = tic;
cases = repmat(struct('id', '', 'type', '', 'title', '', 'status', '', 'checks', [], 'n', [0 0 0 0 0], ...
    'metrics', '', 'wall_s', NaN, 'file', '', 'msg', ''), numel(C), 1);
for k = 1:numel(C)
    c = C(k);
    fprintf('[%2d/%2d] %-15s %-60s ', k, numel(C), c.id, c.title);
    f = fullfile(folder, [c.id '.mat']);
    tc = tic;
    if o.Isolate
        cmd = sprintf('"%s" -batch "addpath(''%s''); phase1_run_case(''%s'', ''%s'')"', mexe, here, c.id, f);
        [st, out] = system(cmd);
        if isfile(f)
            s = load(f, 'r');  r = s.r;
        else
            r = local_error(c, sprintf('process exit %d: %s', st, local_tail(out)));
        end
    else
        try
            r = phase1_run_case(c.id, f);
        catch e
            r = local_error(c, e.message);
        end
    end
    cases(k) = local_classify(r, K, toc(tc));
    fprintf('%-7s %s (%.0f s)\n', cases(k).status, local_counts(cases(k).n), cases(k).wall_s);
end

st = {cases.status};
if any(ismember(st, {'FAIL', 'ERROR'})), verdict = 'FAIL'; else, verdict = 'PASS'; end
R = struct('level', char(level), 'verdict', verdict, 'cases', cases, 'folder', folder, 'report', '', ...
    'date', stamp, 'wall_s', toc(t0));
save(fullfile(folder, 'results.mat'), 'R');
R.report = phase1_report(R, K);
save(fullfile(folder, 'results.mat'), 'R');

fprintf('\n=== Verdict: %s  (%d cases: %d PASS, %d KNOWN, %d FAIL, %d ERROR) in %.0f min ===\n', verdict, numel(cases), ...
    nnz(strcmp(st, 'PASS')), nnz(strcmp(st, 'KNOWN')), nnz(strcmp(st, 'FAIL')), nnz(strcmp(st, 'ERROR')), R.wall_s/60);
for k = find(ismember(st, {'FAIL', 'ERROR'}))
    ch = cases(k).checks;
    bad = ch(strcmp({ch.status}, 'FAIL'));
    fprintf('  %s %s: %s\n', cases(k).status, cases(k).id, strjoin(arrayfun(@(b) sprintf('%s = %s', b.id, local_num(b.value)), ...
        bad, 'UniformOutput', false), ', '));
    if ~isempty(cases(k).msg), fprintf('      %s\n', cases(k).msg(1:min(end, 300))); end
end
fprintf('Report: %s\n', R.report);
if o.Open, web(R.report, '-browser'); end
end

% ------------------------------------------------------------------------------------------------
function c = local_classify(r, K, wall)
% apply known issues and roll the check results up to a case status
ch = r.checks;
for j = 1:numel(ch)
    if strcmp(ch(j).status, 'FAIL')
        hit = K(strcmp({K.case}, r.id) & cellfun(@(s) contains(ch(j).id, s), {K.check}));
        if ~isempty(hit)
            ch(j).status = 'KNOWN';
            ch(j).desc = strtrim([ch(j).desc ' [known: ' hit(1).note ']']);
        end
    end
end
s = {ch.status};
n = [nnz(strcmp(s, 'PASS')), nnz(strcmp(s, 'KNOWN')), nnz(strcmp(s, 'FAIL')), nnz(strcmp(s, 'BLOCKED')), nnz(strcmp(s, 'SKIPPED'))];
if isempty(ch) || (~r.ok && n(3) == 0 && strcmp(r.type, 'unit'))
    status = 'ERROR';
elseif n(3) > 0
    status = 'FAIL';
elseif n(2) > 0
    status = 'KNOWN';
else
    status = 'PASS';
end
if any(strcmp({ch.id}, 'run') & strcmp(s, 'FAIL')) && isscalar(ch), status = 'ERROR'; end
c = struct('id', r.id, 'type', r.type, 'title', r.title, 'status', status, 'checks', ch, 'n', n, ...
    'metrics', r.metrics, 'wall_s', wall, 'file', r.file, 'msg', r.msg);
end

function r = local_error(c, msg)
r = struct('id', c.id, 'type', c.type, 'title', c.title, 'ok', false, ...
    'checks', struct('id', 'run', 'desc', 'case ran', 'status', 'FAIL', 'value', NaN, 'limit', '', 'unit', ''), ...
    'metrics', '', 'wall_s', NaN, 'rtf', NaN, 'file', '', 'msg', msg);
end

function s = local_counts(n)
s = sprintf('%d pass', n(1));
if n(2), s = [s sprintf(', %d known', n(2))]; end
if n(3), s = [s sprintf(', %d FAIL', n(3))]; end
if n(4), s = [s sprintf(', %d blocked', n(4))]; end
if n(5), s = [s sprintf(', %d skipped', n(5))]; end
end

function s = local_num(v)
if isnumeric(v) && isscalar(v), s = sprintf('%.4g', v); else, s = '-'; end
end

function s = local_tail(out)
L = strtrim(splitlines(string(out)));
L = L(strlength(L) > 0);
s = char(strjoin(L(max(1, end - 4):end), ' | '));
end
