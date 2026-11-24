function r = phase1_run_case(caseId, outFile)
%PHASE1_RUN_CASE  Run one Phase 1 test case and save its result (normally in a fresh MATLAB process).
%
%   r = phase1_run_case('S1', 'C:\...\S1.mat')
%   Called by run_phase1_tests through `matlab -batch`, so every case starts with a clean MATLAB (the model
%   compile grows MATLAB memory by about 0.5-1 GB per run, see docs/STATUS.md). Opens the project when it is
%   not open. r: id, type, title, ok, checks (id, desc, status, value, limit, unit), metrics, wall_s, rtf,
%   file, msg. Check status: PASS | FAIL | BLOCKED | SKIPPED (status KNOWN is applied later by the runner).

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
t0 = tic;
c = phase1_cases('All');
c = c(strcmp({c.id}, caseId));
if isempty(c), error('phase1_run_case:id', 'Unknown case %s', caseId); end
r = struct('id', c.id, 'type', c.type, 'title', c.title, 'ok', false, ...
    'checks', struct('id', {}, 'desc', {}, 'status', {}, 'value', {}, 'limit', {}, 'unit', {}), ...
    'metrics', '', 'wall_s', NaN, 'rtf', NaN, 'file', '', 'msg', '');
try
    try
        p = currentProject();
        if ~strcmp(p.Name, 'Shaddad_UGV'), error('other project'); end
    catch
        evalc('openProject(fullfile(root, ''Shaddad_UGV.prj''))');
    end
    ws = warning('off', 'all');
    cw = onCleanup(@() warning(ws));
    switch c.type
        case 'scenario'
            r = local_scenario(c, r);
        case 'drive'
            r = local_drive(c, r);
        case 'unit'
            r = local_unit(c, r, root);
    end
catch e
    r.ok = false;
    r.msg = local_err(e);
    r.checks(end+1) = local_chk('run', 'case ran without error', 'FAIL', NaN, '', '');
end
r.wall_s = toc(t0);
if nargin > 1 && ~isempty(outFile)
    save(outFile, 'r');
end
end

% ------------------------------------------------------------------------------------------------
function r = local_scenario(c, r)
evalc('res = run_scenario(c.scn, ''Mode'', ''normal'');');
r.ok = res.ok;  r.rtf = res.rtf;  r.file = res.file;  r.msg = res.msg;
if ~res.ok
    r.checks(end+1) = local_chk('run', 'simulation finished without error', 'FAIL', NaN, '', '');
end
for k = 1:numel(res.eval)
    e = res.eval(k);
    st = e.status;  if strcmp(st, 'ERROR'), st = 'FAIL'; end
    r.checks(end+1) = local_chk(e.id, e.desc, st, e.value, local_limit(e.op, e.limit), e.unit);
end
if isempty(res.eval) && res.ok
    r.checks(end+1) = local_chk('criteria', 'scenario criteria evaluated', 'FAIL', NaN, '', '');
end
r.metrics = sprintf('%.0f s simulated, RTF %.2f', res.t_end, res.rtf);
end

function r = local_drive(c, r)
evalc('res = phase1_test_drive(c.args{:}, ''Replay'', false);');
close all force
r.ok = res.ok;  r.rtf = res.rtf;  r.file = res.file;  r.msg = res.msg;
[chk, met] = phase1_check_drive(res, c.args);
r.checks = chk;
r.metrics = met;
end

function r = local_unit(c, r, root)
import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.DiagnosticsRecordingPlugin
suite = TestSuite.fromFile(fullfile(root, 'tests', c.file));
keep = true(size(suite));
if ~isempty(c.tags)
    keep = arrayfun(@(s) any(ismember(s.Tags, c.tags)), suite);
end
for k = 1:numel(c.exclude)
    keep = keep & ~contains({suite.ProcedureName}, c.exclude{k});
end
suite = suite(keep);
runner = TestRunner.withNoPlugins;
runner.addPlugin(DiagnosticsRecordingPlugin);
res = runner.run(suite);
r.ok = true;
for k = 1:numel(res)
    t = res(k);
    if t.Passed, st = 'PASS';
    elseif t.Failed, st = 'FAIL';
    else, st = 'SKIPPED';
    end
    msg = '';
    if ~t.Passed
        recs = t.Details.DiagnosticRecord;
        for j = 1:numel(recs)
            L = strtrim(splitlines(string(recs(j).Report)));
            L = L(strlength(L) > 0 & ~startsWith(L, ["=", "-", "Error in", "Framework Diagnostic", "Test Diagnostic"]));
            if ~isempty(L), msg = char(strjoin(L(1:min(end, 3)), ' | ')); break; end
        end
    end
    r.checks(end+1) = local_chk(t.Name(find(t.Name == '/', 1, 'last') + 1:end), msg, st, t.Duration, '', 's');
end
n = numel(res);
r.metrics = sprintf('%d tests: %d passed, %d failed, %d skipped/blocked', n, nnz([res.Passed]), nnz([res.Failed]), ...
    nnz([res.Incomplete] & ~[res.Failed]));
end

% ------------------------------------------------------------------------------------------------
function s = local_chk(id, desc, status, value, limit, unit)
s = struct('id', char(id), 'desc', char(desc), 'status', char(status), 'value', double(value), ...
    'limit', char(limit), 'unit', char(unit));
end

function s = local_limit(op, lim)
if isempty(lim), s = op; return, end
if numel(lim) == 2, s = sprintf('%s [%g, %g]', op, lim(1), lim(2));
else, s = sprintf('%s %g', op, lim);
end
end

function m = local_err(e)
m = regexprep(e.message, '<[^>]*>', '');
for k = 1:min(numel(e.cause), 3)
    m = [m ' | ' regexprep(e.cause{k}.message, '<[^>]*>', '')]; %#ok<AGROW>
end
end
