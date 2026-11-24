function res = run_scenario(id, varargin)
%RUN_SCENARIO  Run one Shaddad scenario and evaluate its criteria (build plan T01.8 skeleton, completed by T16).
%
%   res = run_scenario(id)
%   res = run_scenario(id, 'Mode', m, 'Solver', s, 'Variant', v, 'Args', {..}, 'StopTime', T, 'Viz', k, ...
%                      'Save', tf, 'Profile', tf, 'ProfileFile', f, 'P', P, 'Timeout', s)
%
%   id        scenario id 'S0' .. 'S8'; a trailing letter selects a variant ('S6b' = S6 variant b).
%             scripts/scenarios/scn_<S#>_*.m is called as [in, crit] = scn_<S#>_<name>(P, args{:}) with
%             'Variant' and 'StopTime' added when given. Without a scenario file, scn_base_input(P, id, ...)
%             builds a default input (all-zero ScenarioIn: key on, TX off, no faults) with 'StopTime',
%             'Input', 'VC'.
%   'Mode'    'normal' (default) | 'accelerator' | 'rapid' (SimulationMode of Shaddad_System; D56 missions
%             use accelerator for the top model and every Model block - needs a C compiler, none is installed,
%             so the request falls back to normal with a warning)
%   'Solver'  'fixed' (default, cs_SimFixed: ode3 1 ms, local Simscape solvers) | 'ref' (accuracy reference,
%             D58: UseLocalSolver 'off' on the Solver Configuration blocks of Shaddad_Plant and
%             Electronics_Plant (block parameters), cs_Shaddad = cs_SimRef (SimscapeLogType 'none'), both
%             restored afterwards without saving any file)
%   'Viz'     VC_VIZ for this run (default: scenario value, batch runs use 0)
%   'Save'    true (default): work/results/<id>_<yyyymmdd_HHMMSS>.mat with id, crit (no handles: ids and
%             limits), eval, S (collected logs), kpi summary, wall time, RTF, mode, solver
%   'Profile' true: Simulink Profiler on; 'ProfileFile' (default work/profile_<id>.html) receives the report
%   'P'       parameter struct (default: base P)
%   'Timeout' wall-time guard [s] (default P.sim.wall_timeout*10); stop_reason 7 when it fires
%
%   res: .id .scn .variant .mode .solver .in .out .crit .eval .S .wall_s .exec_s .stop_time .t_end .rtf
%        .profile_file .file .ok .pass .msg

ip = inputParser;
ip.addParameter('Mode', 'normal');
ip.addParameter('Solver', 'fixed');
ip.addParameter('Variant', '');
ip.addParameter('Args', {});
ip.addParameter('StopTime', []);
ip.addParameter('Input', struct());
ip.addParameter('VC', struct());
ip.addParameter('Viz', []);
ip.addParameter('Save', true);
ip.addParameter('Profile', false);
ip.addParameter('ProfileFile', '');
ip.addParameter('P', []);
ip.addParameter('Timeout', []);
ip.addParameter('KeepOut', true);
ip.parse(varargin{:});
o = ip.Results;
id = char(id);
root = char(currentProject().RootFolder);
if isempty(o.P)
    if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_startup('quiet'); end
    P = evalin('base', 'P');
else
    P = o.P;
end
if isempty(o.Timeout), o.Timeout = 10*P.sim.wall_timeout; end

%% ---- scenario id / variant ----
tok = regexp(id, '^(S\d+)([a-z]*)$', 'tokens', 'once');
if isempty(tok), scn = id; var = char(o.Variant); else, scn = tok{1}; var = tok{2}; end
if ~isempty(o.Variant), var = char(o.Variant); end
if ~isempty(tok), id = [scn var]; end
args = o.Args;
if ~isempty(var), args = [args, {'Variant', var}]; end
if ~isempty(o.StopTime), args = [args, {'StopTime', o.StopTime}]; end

%% ---- scenario input ----
crit = [];
f = dir(fullfile(root, 'scripts', 'scenarios', ['scn_' scn '_*.m']));
if ~isempty(f)
    [~, fn] = fileparts(f(1).name);
    [in, crit] = feval(fn, P, args{:});
else
    T = o.StopTime; if isempty(T), T = 10; end
    in = scn_base_input(P, id, 'StopTime', T, 'Input', o.Input, 'VC', o.VC);
end
if ~isempty(o.Viz), in = in.setVariable('VC_VIZ', o.Viz); end
modeMap = struct('normal', 'normal', 'accelerator', 'accelerator', 'rapid', 'rapid-accelerator');
m = lower(char(o.Mode));
if ~isfield(modeMap, m), error('run_scenario:mode', 'Mode must be normal | accelerator | rapid'); end
if ~strcmp(m, 'normal') && isempty(mex.getCompilerConfigurations('C', 'Selected'))
    warning('run_scenario:noCompiler', ['Mode %s needs a C compiler (none installed, D31/D56): ' ...
        'running in normal mode.'], m);
    m = 'normal';
end
in = in.setModelParameter('SimulationMode', modeMap.(m));
in = in.setModelParameter('CaptureErrors', 'on');     % errors (e.g. the NaN assertion) return the logs
if o.Profile
    in = in.setModelParameter('Profile', 'on');
end

res = struct('id', id, 'scn', scn, 'variant', var, 'mode', m, 'solver', lower(char(o.Solver)), 'in', [], ...
    'out', [], 'crit', [], 'eval', [], 'S', [], 'wall_s', NaN, 'exec_s', NaN, ...
    'stop_time', str2double(in.getModelParameter('StopTime')), 't_end', NaN, 'rtf', NaN, ...
    'profile_file', '', 'file', '', 'ok', false, 'pass', false, 'msg', '');
res.in = in;            % assigned separately: struct() with an object argument calls its struct method
res.crit = crit;
% map / plan / footprint for the offline bird's-eye replay (utilities/shaddad_replay_birdseye.m)
try
    v = in.Variables;
    Pin = v(strcmp({v.Name}, 'P')).Value;
    res.viz = struct('occ', Pin.world.occ, 'path_xy', shaddad.BirdsEyeView.planPath(Pin.gnc.mission), ...
        'footprint', Pin.viz.footprint, 'world_id', Pin.world.id);
catch
    res.viz = [];
end

%% ---- solver selection (D58) ----
restore = struct('blk', {{}}, 'val', {{}}, 'dirty', {{}}, 'bd', {{}});
switch res.solver
    case 'fixed'
        % cs_Shaddad is the fixed set after shaddad_startup
    case 'ref'
        for bd = {'Shaddad_Plant', 'Electronics_Plant'}
            if ~bdIsLoaded(bd{1}), load_system(bd{1}); end
            blks = find_system(bd{1}, 'MatchFilter', @Simulink.match.allVariants, 'LookUnderMasks', 'all', ...
                'FollowLinks', 'on', 'RegExp', 'on', 'ReferenceBlock', '^nesl_utility/Solver.Configuration$');
            for k = 1:numel(blks)
                if strcmp(bd{1}, 'Shaddad_Plant') && startsWith(blks{k}, 'Shaddad_Plant/Electronics/')
                    continue                       % same block as in Electronics_Plant (SSR instance)
                end
                restore.blk{end+1} = blks{k};
                restore.val{end+1} = get_param(blks{k}, 'UseLocalSolver');
                restore.bd{end+1} = bd{1};
                restore.dirty{end+1} = get_param(bd{1}, 'Dirty');
                set_param(blks{k}, 'UseLocalSolver', 'off');
            end
        end
        shaddad_configsets('ref');               % cs_Shaddad = copy of cs_SimRef (variable-step daessc)
        % Simscape logging is not supported inside a referenced model (Shaddad_Plant is a Model block,
        % D14), so the top-level reference run logs signals only.
        csS = evalin('base', 'cs_Shaddad');
        set_param(csS, 'SimscapeLogType', 'none');
        assignin('base', 'cs_Shaddad', csS);
    otherwise
        error('run_scenario:solver', 'Solver must be fixed | ref');
end

%% ---- run (first update retried once on the R2026b JIT relink assertion, T03-T07 lesson) ----
for attempt = 1:2
    try
        t0 = tic;
        out = sim(in);
        res.wall_s = toc(t0);
        res.out = out;
        res.msg = local_diag_text(out);
        res.ok = isempty(out.ErrorMessage);
        if ~res.ok && attempt == 1 && contains(res.msg, 'o != nullptr')
            continue
        end
        break
    catch tmp_rs_err
        res.msg = local_err_text(tmp_rs_err);
        if attempt == 1 && contains(res.msg, 'o != nullptr'), continue, end
        break
    end
end
local_restore(restore, res.solver);

%% ---- results ----
if ~isempty(res.out)
    try
        ti = res.out.SimulationMetadata.TimingInfo;
        res.exec_s = ti.ExecutionElapsedWallTime;
    catch
    end
    try
        res.S = scn_collect(res.out);
        if isfield(res.S, 'kpi')
            res.t_end = res.S.kpi.t_mission.t(end);
        end
    catch tmp_rs_err
        res.msg = [res.msg ' | collect: ' tmp_rs_err.message];
    end
    if isnan(res.t_end), res.t_end = res.stop_time; end
    res.rtf = res.t_end / res.exec_s;
end
if ~res.ok
    warning('run_scenario:failed', 'Scenario %s ended with an error: %s', id, res.msg);
end

%% ---- profiler report ----
if o.Profile && ~isempty(res.out)
    pf = o.ProfileFile;
    if isempty(pf), pf = fullfile(root, 'work', ['profile_' id '.html']); end
    try
        pd = res.out.SimulationMetadata.TimingInfo.ProfilerData;
        if ~isfolder(fileparts(pf)), mkdir(fileparts(pf)); end
        local_profile_report(pd, pf);
        res.profile_file = pf;
    catch tmp_rs_err
        warning('run_scenario:profile', 'Profiler report not written: %s', tmp_rs_err.message);
    end
end

%% ---- criteria ----
if ~isempty(crit) && ~isempty(res.S)
    fprintf('--- %s criteria ---\n', id);
    res.eval = eval_criteria(res.S, crit);
    st = {res.eval.status};
    res.pass = res.ok && all(strcmp(st, 'PASS') | strcmp(st, 'BLOCKED'));
end

%% ---- save ----
if o.Save
    d = fullfile(root, 'work', 'results');
    if ~isfolder(d), mkdir(d); end
    res.file = fullfile(d, sprintf('%s_%s.mat', id, char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
    s = rmfield(res, {'out', 'in', 'crit'});
    if ~isempty(crit)
        s.crit = rmfield(crit, 'metric');            % function handles are not saved
    end
    save(res.file, '-struct', 's', '-v7.3');
end
if ~o.KeepOut
    res.out = [];
end
fprintf('run_scenario %s: %s, solver %s, %.4g s simulated, exec %.4g s, wall %.4g s (RTF %.3g), ok=%d, pass=%d\n', ...
    id, m, res.solver, res.t_end, res.exec_s, res.wall_s, res.rtf, res.ok, res.pass);
end

% -------------------------------------------------------------------------------------------
function s = local_diag_text(out)
% full error text of a SimulationOutput (message plus every cause), '' when the run succeeded
s = char(out.ErrorMessage);
if isempty(s), return, end
try
    d = out.SimulationMetadata.ExecutionInfo.ErrorDiagnostic.Diagnostic;
    s = local_mdiag(d, s);
catch
end
end

function s = local_mdiag(d, s)
for k = 1:numel(d.cause)
    s = sprintf('%s | %s', s, d.cause{k}.message);
    s = local_mdiag(d.cause{k}, s);
end
end

function s = local_err_text(e)
s = e.message;
for k = 1:numel(e.cause)
    s = sprintf('%s | %s', s, local_err_text(e.cause{k}));
end
end

function local_restore(r, solver)
for k = 1:numel(r.blk)
    try
        set_param(r.blk{k}, 'UseLocalSolver', r.val{k});
        if strcmp(r.dirty{k}, 'off'), set_param(r.bd{k}, 'Dirty', 'off'); end   % restored, nothing saved
    catch tmp_rs_err
        warning('run_scenario:restore', 'Could not restore %s: %s', r.blk{k}, tmp_rs_err.message);
    end
end
if strcmp(solver, 'ref')
    shaddad_configsets('fixed');
end
end

function local_profile_report(pd, pf)
% Simulink.profiler.Data -> HTML table of the 60 most expensive nodes (self and total time)
rows = {};
q = {pd.rootUINode};
while ~isempty(q)
    n = q{1}; q(1) = [];
    ch = n.children;
    for j = 1:numel(ch)
        c = ch(j);
        p = char(c.path);
        if numel(p) > 1 && p(1) == p(min(2, end))
            p = strtrim(p(2:2:end));              % interleaved top/referenced path text
        end
        rows(end+1, :) = {p, c.totalTime, c.selfTime, c.numberOfCalls}; %#ok<AGROW>
        q{end+1} = c; %#ok<AGROW>
    end
end
[~, ix] = sort(cell2mat(rows(:, 3)), 'descend');
fid = fopen(pf, 'w');
fprintf(fid, '<html><head><meta charset="utf-8"><title>Shaddad profile</title></head><body>\n');
fprintf(fid, '<h3>Simulink Profiler: %s (root total %.3f s)</h3>\n', pf, pd.rootUINode.totalTime);
fprintf(fid, '<table border="1" cellpadding="3"><tr><th>self [s]</th><th>total [s]</th><th>calls</th><th>path</th></tr>\n');
for k = ix(1:min(60, end))'
    fprintf(fid, '<tr><td>%.3f</td><td>%.3f</td><td>%d</td><td>%s</td></tr>\n', rows{k, 3}, rows{k, 2}, ...
        rows{k, 4}, strrep(strrep(rows{k, 1}, '<', '&lt;'), '>', '&gt;'));
end
fprintf(fid, '</table></body></html>\n');
fclose(fid);
end
