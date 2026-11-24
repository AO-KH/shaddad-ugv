function R = run_all_scenarios(ids, varargin)
%RUN_ALL_SCENARIOS  Run scenarios S0-S8 (all variants) grouped by world + variant set (D58, T16).
%
%   R = run_all_scenarios()                          every scenario and variant except S3 (4 h runs)
%   R = run_all_scenarios({'S0','S1','S6a'})         selected ids (trailing letter = variant)
%   R = run_all_scenarios(ids, 'Parallel', p, 'Save', tf, 'Workers', n)
%     'Parallel'  'auto' (default) | true | false. true: one parsim call per group with
%                 'ShowProgress','on', 'SetupFcn', @() shaddad_startup('quiet'),
%                 'TransferBaseWorkspaceVariables','off', 'UseFastRestart','on' (Fast Restart only within a
%                 group: world dimensions and variant changes force rebuilds). 'auto' uses parsim only
%                 when a parallel pool exists or can be opened with at least 8 GB of free physical memory
%                 per worker; otherwise the groups run serially through run_scenario.
%   R: struct array (id, group, ok, pass, rtf, wall_s, eval, file)
%
%   Inputs come from the scenario scripts ([in, crit] = scn_S<k>_*(P, ...)); every SimulationInput carries
%   P, Ts, VC_* and mpcRowObj, so workers never read the client's base workspace.

ip = inputParser;
ip.addParameter('Parallel', 'auto');
ip.addParameter('Save', true);
ip.addParameter('Workers', 2);
ip.parse(varargin{:});
o = ip.Results;
if nargin < 1 || isempty(ids)
    ids = {'S0', 'S1', 'S2', 'S6a', 'S6b', 'S6c', 'S6d', 'S8a', 'S8b', 'S7farm', 'S7wet', 'S7temp', 'S5', 'S4'};
end
if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_startup('quiet'); end
P = evalin('base', 'P');
root = char(currentProject().RootFolder);

%% ---- build every input, group by world + variant set ----
ins = Simulink.SimulationInput.empty;
crits = cell(numel(ids), 1);
keys = cell(numel(ids), 1);
for k = 1:numel(ids)
    [scn, var] = local_split(ids{k});
    f = dir(fullfile(root, 'scripts', 'scenarios', ['scn_' scn '_*.m']));
    [~, fn] = fileparts(f(1).name);
    args = {};
    if ~isempty(var), args = {'Variant', var}; end
    [in, crits{k}] = feval(fn, P, args{:});
    ins(k) = in;
    keys{k} = local_key(in);
end
[ukeys, ~, gidx] = unique(keys, 'stable');

usePar = local_use_parallel(o.Parallel);
R = repmat(struct('id', '', 'group', 0, 'ok', false, 'pass', false, 'rtf', NaN, 'wall_s', NaN, ...
    'eval', [], 'file', ''), numel(ids), 1);
for g = 1:numel(ukeys)
    sel = find(gidx == g);
    fprintf('=== group %d/%d: %s (%d runs) ===\n', g, numel(ukeys), ukeys{g}, numel(sel));
    if usePar
        t0 = tic;
        outs = parsim(ins(sel), 'ShowProgress', 'on', 'SetupFcn', @() shaddad_startup('quiet'), ...
            'TransferBaseWorkspaceVariables', 'off', 'UseFastRestart', 'on'); %#ok<PARSIMTRANSF> plan D58: explicit 'off'
        wall = toc(t0);
        for j = 1:numel(sel)
            k = sel(j);
            out = outs(j);
            R(k).id = ids{k};  R(k).group = g;  R(k).ok = isempty(out.ErrorMessage);
            R(k).wall_s = wall/numel(sel);
            S = scn_collect(out);
            if ~isempty(crits{k}) && ~isempty(fieldnames(S))
                fprintf('--- %s ---\n', ids{k});
                R(k).eval = eval_criteria(S, crits{k});
                st = {R(k).eval.status};
                R(k).pass = R(k).ok && all(strcmp(st, 'PASS') | strcmp(st, 'BLOCKED'));
            end
            try
                R(k).rtf = S.kpi.t_mission.t(end)/out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
            catch
            end
            if o.Save
                R(k).file = local_save(root, ids{k}, S, R(k), crits{k});
            end
        end
    else
        for j = 1:numel(sel)
            k = sel(j);
            [scn, var] = local_split(ids{k});
            r = run_scenario(scn, 'Variant', var, 'Save', o.Save, 'KeepOut', false);
            R(k) = struct('id', ids{k}, 'group', g, 'ok', r.ok, 'pass', r.pass, 'rtf', r.rtf, ...
                'wall_s', r.wall_s, 'eval', r.eval, 'file', r.file);
        end
    end
end
fprintf('\n%-8s %-6s %-5s %-5s %8s %9s\n', 'id', 'group', 'ok', 'pass', 'RTF', 'wall [s]');
for k = 1:numel(R)
    fprintf('%-8s %-6d %-5d %-5d %8.3g %9.1f\n', R(k).id, R(k).group, R(k).ok, R(k).pass, R(k).rtf, R(k).wall_s);
end
if o.Save
    d = fullfile(root, 'work', 'results');
    if ~isfolder(d), mkdir(d); end
    save(fullfile(d, sprintf('all_%s.mat', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')))), 'R');
end
end

% -------------------------------------------------------------------------------------------
function [scn, var] = local_split(id)
tok = regexp(id, '^(S\d+)(.*)$', 'tokens', 'once');
scn = tok{1};  var = tok{2};
end

function key = local_key(in)
v = in.Variables;
names = {v.Name};
Pv = v(strcmp(names, 'P')).Value;
vc = sort(names(startsWith(names, 'VC_')));
parts = {sprintf('world%d', Pv.world.id)};
for k = 1:numel(vc)
    parts{end+1} = sprintf('%s=%g', vc{k}, v(strcmp(names, vc{k})).Value); %#ok<AGROW>
end
key = strjoin(parts, ',');
end

function tf = local_use_parallel(opt)
if islogical(opt) || isnumeric(opt), tf = logical(opt); return, end
tf = false;
if ~license('test', 'Distrib_Computing_Toolbox') || isempty(ver('parallel')), return, end
if ~isempty(gcp('nocreate')), tf = true; return, end
try
    [~, m] = memory;
    tf = m.PhysicalMemory.Available > 8e9;
catch
    tf = false;
end
if ~tf
    fprintf('run_all_scenarios: not enough free memory for parsim workers, running serially.\n');
end
end

function f = local_save(root, id, S, r, crit)
d = fullfile(root, 'work', 'results');
if ~isfolder(d), mkdir(d); end
f = fullfile(d, sprintf('%s_%s.mat', id, char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
s = struct('id', id, 'S', S, 'eval', r.eval, 'ok', r.ok, 'pass', r.pass, 'rtf', r.rtf, 'wall_s', r.wall_s);
if ~isempty(crit), s.crit = rmfield(crit, 'metric'); end
save(f, '-struct', 's', '-v7.3');
end
