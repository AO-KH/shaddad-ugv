function results = run_regression(tagSet)
%RUN_REGRESSION  Shaddad regression runner (integration spec 7.11, build plan T16).
%
%   results = run_regression('Fast')     tests tagged 'Fast' in tests/** (target <= 10 min)
%   results = run_regression('All')      every test class in tests/** (hours: system scenarios, benches)
%   results = run_regression(tag)        any other tag, e.g. 'Integration', 'System'
%
%   matlab.unittest.TestSuite.fromFolder('tests', 'IncludingSubfolders', true) with a HasTag selector,
%   TestReportPlugin (HTML, work/reports/<tag>_<time>) and XMLPlugin (JUnit, work/reports/<tag>_<time>.xml).
%   Coverage is not collected (Simulink Coverage is not installed). Prints a summary table and the wall time.

if nargin < 1 || isempty(tagSet), tagSet = 'Fast'; end
import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.selectors.HasTag
import matlab.unittest.plugins.TestReportPlugin
import matlab.unittest.plugins.XMLPlugin
root = char(currentProject().RootFolder);
if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_startup('quiet'); end
tdir = fullfile(root, 'tests');
if strcmpi(tagSet, 'All')
    suite = TestSuite.fromFolder(tdir, 'IncludingSubfolders', true);
else
    suite = TestSuite.fromFolder(tdir, HasTag(tagSet), 'IncludingSubfolders', true);
end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
rdir = fullfile(root, 'work', 'reports');
if ~isfolder(rdir), mkdir(rdir); end
runner = TestRunner.withTextOutput;
try
    runner.addPlugin(TestReportPlugin.producingHTML(fullfile(rdir, [tagSet '_' stamp]), 'IncludingPassingDiagnostics', false));
catch tmp_rr_err
    warning('run_regression:html', 'HTML report plugin unavailable: %s', tmp_rr_err.message);
end
runner.addPlugin(XMLPlugin.producingJUnitFormat(fullfile(rdir, [tagSet '_' stamp '.xml'])));
t0 = tic;
results = runner.run(suite);
wall = toc(t0);
fprintf('\nrun_regression(''%s''): %d tests, %d passed, %d failed, %d incomplete, %.1f min\n', tagSet, ...
    numel(results), nnz([results.Passed]), nnz([results.Failed]), nnz([results.Incomplete]), wall/60);
end
