function r = shaddad_run_tests(target, varargin)
%SHADDAD_RUN_TESTS  Run a Shaddad test class, or the tests of it picked by name or tag, one line per test.
%
%   r = shaddad_run_tests('t_infra')                           the class, without the tests that cannot finish here
%   r = shaddad_run_tests('t_infra', 'Name', 'testStartup*')   only the tests whose name matches (* is a wildcard)
%   r = shaddad_run_tests('t_layout', 'Tag', 'Top')            only the tests with that tag
%   r = shaddad_run_tests('t_infra', 'All', true)              the whole class, the left-out tests too
%   r = shaddad_run_tests('*', 'Tag', 'Fast')                  every class in tests/**: here the Fast tier of
%                                                              run_regression('Fast'), one line per test
%
%   Left out unless 'All' is true, as in the layout plan's full test run: the tests that rebuild model files
%   (t_world and t_infra tag Build, t_framework/testBuildIdempotent) and the code builds that cannot finish on this
%   PC (no STM32 tools; the Companion build fails at HybridAStar): t_infra tag Codegen, testCodegenBlocked of
%   t_mcu_platform and t_mcu_control, t_perception/tCodegen and the four build tests of t_targets.
%
%   Prints 'PASSED|FAILED|INCOMPLETE <name> (<s> s)', up to three '    | <diagnostic>' lines for each test that did
%   not pass, and '== <label>: total N, passed N, failed N, incomplete N, N s': the format that
%   tools/agentic/test_baseline.py reads. r is the matlab.unittest.TestResult array.
%
%   It works in a batch MATLAB and in a shared session: it needs the Shaddad project open, leaves the base
%   workspace alone and puts the current folder back.
%
%   See also run_regression.

p = inputParser;
p.FunctionName = 'shaddad_run_tests';
text = @(s) ischar(s) || (isstring(s) && isscalar(s));
p.addRequired('target', text);
p.addParameter('Name', '', text);
p.addParameter('Tag', '', text);
p.addParameter('All', false, @(b) islogical(b) && isscalar(b));
p.parse(target, varargin{:});
o = p.Results;
cls = char(o.target);

root = fileparts(fileparts(mfilename('fullpath')));
prj = matlab.project.rootProject;
if isempty(prj) || ~strcmpi(char(prj.RootFolder), root)
    error('shaddad_run_tests:project', 'Open the Shaddad project first: openProject(''%s'').', ...
        fullfile(root, 'Shaddad_UGV.prj'));
end
every = strcmp(cls, '*');
f = which(cls);
if ~every && isempty(f)
    error('shaddad_run_tests:class', 'No test class %s on the path.', cls);
end
old = cd(root);
back = onCleanup(@() cd(old));

fprintf('T06_LONG = "%s", T08_LONG = "%s", T09_LONG = "%s", T17_LONG = "%s"\n', getenv('T06_LONG'), ...
    getenv('T08_LONG'), getenv('T09_LONG'), getenv('T17_LONG'));
args = {};
label = cls;
if strlength(o.Name) > 0
    if every, args = [args, {'Name', char(o.Name)}]; else, args = [args, {'Name', [cls '/' char(o.Name)]}]; end
    label = [label ' ' char(o.Name)];
end
if strlength(o.Tag) > 0
    args = [args, {'Tag', char(o.Tag)}];
    label = [label ' tag ' char(o.Tag)];
end
if every
    s = matlab.unittest.TestSuite.fromFolder(fullfile(root, 'tests'), 'IncludingSubfolders', true, args{:});
else
    s = matlab.unittest.TestSuite.fromFile(f, args{:});
end
if ~o.All
    if every
        classes = {'t_mcu_platform', 't_mcu_control', 't_targets', 't_infra', 't_world', 't_framework', 't_perception'};
    else
        classes = {cls};
    end
    for c = classes
        [sel, why] = local_left_out(c{1});
        if ~isempty(sel)
            s = s.selectIf(sel);
            if ~every, label = [label ' ' why]; end
        end
    end
    if every, label = [label ' without the left-out tests']; end
end

fprintf('\n===== %s: %d tests\n', label, numel(s));
runner = matlab.unittest.TestRunner.withTextOutput('LoggingLevel', matlab.unittest.Verbosity.Concise);
r = runner.run(s);
for k = 1:numel(r)
    st = 'PASSED';
    if r(k).Failed, st = 'FAILED'; elseif r(k).Incomplete, st = 'INCOMPLETE'; end
    fprintf('%-10s %s (%.1f s)\n', st, r(k).Name, r(k).Duration);
    if r(k).Failed || r(k).Incomplete
        d = r(k).Details;
        if isfield(d, 'DiagnosticRecord') && ~isempty(d.DiagnosticRecord)
            for j = 1:min(3, numel(d.DiagnosticRecord))
                rep = regexprep(d.DiagnosticRecord(j).Report, '\s+', ' ');
                fprintf('    | %s\n', rep(1:min(900, end)));
            end
        end
    end
end
fprintf('== %s: total %d, passed %d, failed %d, incomplete %d, %.0f s\n', label, numel(r), nnz([r.Passed]), ...
    nnz([r.Failed]), nnz([r.Incomplete]), sum([r.Duration]));
clear back
end

function [sel, why] = local_left_out(cls)
% The selector that drops the tests a default run leaves out, and the words the label gets for it.
import matlab.unittest.selectors.HasTag
import matlab.unittest.selectors.HasName
sel = [];
why = '';
switch cls
    case {'t_mcu_platform', 't_mcu_control'}
        sel = ~HasName([cls '/testCodegenBlocked']);
        why = 'without testCodegenBlocked';
    case 't_targets'
        sel = ~HasName('t_targets/testBuildBlockedWithoutProducts') & ~HasName('t_targets/testBuildCompanion') & ...
            ~HasName('t_targets/testBuildMCU') & ~HasName('t_targets/testBuildBMS');
        why = 'without the four build tests';
    case 't_infra'
        sel = ~HasTag('Build') & ~HasTag('Codegen');
        why = 'without tags Build and Codegen';
    case 't_world'
        sel = ~HasTag('Build');
        why = 'without tag Build';
    case 't_framework'
        sel = ~HasName('t_framework/testBuildIdempotent');
        why = 'without testBuildIdempotent';
    case 't_perception'
        sel = ~HasName('t_perception/tCodegen');
        why = 'without tCodegen';
end
end
