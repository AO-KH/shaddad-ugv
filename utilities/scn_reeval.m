function r = scn_reeval(id, varargin)
%SCN_REEVAL  Re-evaluate the latest saved result of a scenario with the current criteria (T16 helper).
%
%   r = scn_reeval('S8a')            rebuilds [in, crit] with the scenario script (no simulation),
%                                    evaluates the saved logs r.S and writes r.eval / r.pass back to the file
%   r = scn_reeval(id, 'Save', false)
%   Used when criteria code changes after a long run (the logs do not change).

ip = inputParser;
ip.addParameter('Save', true);
ip.parse(varargin{:});
[r, file] = scn_latest(id);
if isempty(r), error('scn_reeval:none', 'No saved result for %s', id); end
tok = regexp(id, '^(S\d+)([a-z]*)$', 'tokens', 'once');
scn = tok{1};  var = tok{2};
root = char(currentProject().RootFolder);
f = dir(fullfile(root, 'scripts', 'scenarios', ['scn_' scn '_*.m']));
[~, fn] = fileparts(f(1).name);
args = {};
if ~isempty(var), args = {'Variant', var}; end
P = evalin('base', 'P');
[~, crit] = feval(fn, P, args{:});
fprintf('--- %s criteria (re-evaluated from %s) ---\n', id, file);
r.eval = eval_criteria(r.S, crit);
st = {r.eval.status};
r.pass = logical(r.ok) && all(strcmp(st, 'PASS') | strcmp(st, 'BLOCKED'));
r.crit = rmfield(crit, 'metric');
if ip.Results.Save
    save(file, '-struct', 'r', '-v7.3');
end
end
