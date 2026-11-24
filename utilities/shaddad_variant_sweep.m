function R = shaddad_variant_sweep(controls)
%SHADDAD_VARIANT_SWEEP  Update Shaddad_System for every value of every variant control, one control at a time
%   (model layout cleanup, Review Focus 2: every choice that compiled before still compiles).
%
%   R = shaddad_variant_sweep()            every control found in the variant conditions
%   R = shaddad_variant_sweep(controls)    controls: a cellstr (or string array) of VC_* names, e.g.
%                                          {'VC_IMPL', 'VC_PACK'}
%   R   table, one row per control and value (the controls in the given order, sorted by default; the values
%       ascending):
%         .control  the VC_* name
%         .value    the value tried
%         .ok       true when Shaddad_System updated without an error
%         .msg      the error message with its causes ('' when ok)
%
%   Values: the k of every condition 'VC_X == k' under Shaddad_System: the VariantControl of the choices of
%   Variant Subsystems (Subsystem, Subsystem Reference and Model blocks), the VariantControls of Variant Source
%   and Variant Sink blocks and the variants of variant Model blocks, read in Shaddad_System and in every file
%   it references (Model and Subsystem Reference files, recursively), in every variant choice, under masks and
%   inside library links. A control without such a condition is the error shaddad_variant_sweep:control, and
%   so is one without a variable in the base workspace.
%   Defaults: the values of the VC_* variables in the base workspace when the sweep starts. The all-defaults
%   configuration is updated once, first, and its result is the row of each control's default value. Every
%   other value is tried with the other controls at their defaults: VC_X = k in the base workspace,
%   set_param('Shaddad_System', 'SimulationCommand', 'update') (its output captured with evalc), then VC_X back
%   to its default. An update that fails with the R2026b Stateflow assertion 'o != nullptr' (seen on the first
%   Shaddad_Plant update of a session) runs once more, as in t_framework; its progress line says so.
%   A choice that sits inside an inactive choice of another control is not compiled by its update, for example
%   the Shaddad_Companion internals while VC_PHASE is 1: make the gating value the default (VC_PHASE = 2 in the
%   base workspace) before the sweep to try those choices. VC_LIDAR = 2 (FULL3D) also needs the base variable
%   sens_uavScene = sens_env_to_uavscenario(P), which build_sensor_models and the sensor tests create.
%   Every VC_* variable of the base workspace is restored afterwards, also after an error (onCleanup), and the
%   block diagrams the sweep loaded are closed again without saving (shaddad_close_since, which warns about
%   any that stays loaded); nothing is saved. Prints the controls and
%   values found (with the files whose hierarchy holds their conditions), one line per update and the total
%   time.

top = 'Shaddad_System';
t0 = tic;
vc = evalin('base', 'who(''VC_*'')');
base0 = struct();
for k = 1:numel(vc), base0.(vc{k}) = evalin('base', vc{k}); end
before = find_system('SearchDepth', 0, 'type', 'block_diagram');   % block diagrams loaded before the sweep
cl = onCleanup(@() local_restore(base0, before));    % also after an error
if ~bdIsLoaded(top), load_system(top); end
[V, W] = local_values(top);
found = sort(keys(V));
if nargin >= 1 && isstring(controls), controls = cellstr(controls); end
if nargin < 1 || isempty(controls)
    controls = found;
elseif iscellstr(controls)
    controls = reshape(controls, 1, []);
    bad = setdiff(controls, found);
    if ~isempty(bad)
        error('shaddad_variant_sweep:control', 'No variant condition ''%s == k'' under %s', bad{1}, top);
    end
else
    error('shaddad_variant_sweep:control', 'controls must be a cellstr of VC_* names');
end
noVar = setdiff(controls, vc);
if ~isempty(noVar)
    error('shaddad_variant_sweep:control', '%s is not a variable of the base workspace', noVar{1});
end
fprintf('shaddad_variant_sweep: %d controls found under %s (%.0f s)\n', numel(found), top, toc(t0));
for k = 1:numel(found)
    n = found{k};
    if isfield(base0, n), d = sprintf('%g', base0.(n)); else, d = 'none'; end
    fprintf('  %-15s values %-10s default %-4s in %s\n', n, num2str(V(n)), d, strjoin(W(n), ', '));
end
[ok0, msg0] = local_update(top, 'all defaults');
rows = cell(0, 4);
for c = controls
    n = c{1};
    def = base0.(n);
    for v = V(n)
        if isequal(v, def)
            rows(end+1, :) = {n, v, ok0, msg0}; %#ok<AGROW> the all-defaults configuration
        else
            assignin('base', n, v);
            [ok, msg] = local_update(top, sprintf('%s = %g', n, v));
            assignin('base', n, def);
            rows(end+1, :) = {n, v, ok, msg}; %#ok<AGROW>
        end
    end
end
R = table(rows(:, 1), reshape(double(cell2mat(rows(:, 2))), [], 1), reshape(logical(cell2mat(rows(:, 3))), [], 1), ...
    rows(:, 4), 'VariableNames', {'control', 'value', 'ok', 'msg'});
fprintf('shaddad_variant_sweep: %d choices of %d controls, %d fail, %.1f min\n', height(R), numel(controls), ...
    nnz(~R.ok), toc(t0)/60);
end

% -------------------------------------------------------------------------------------------
function [V, W] = local_values(top)
% control -> sorted values k of every 'VC_X == k' in the variant conditions of top and of the files it
% references (W: control -> the files whose search found it; a file's hierarchy includes its Subsystem
% References and library links, not its Model blocks)
V = containers.Map('KeyType', 'char', 'ValueType', 'any');
W = containers.Map('KeyType', 'char', 'ValueType', 'any');
opts = {'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants};
todo = {top};
seen = {};
while ~isempty(todo)
    f = todo{1};
    todo(1) = [];
    if any(strcmp(seen, f)), continue, end
    seen{end+1} = f; %#ok<AGROW>
    if ~bdIsLoaded(f), load_system(f); end
    cond = {};
    % choices of Variant Subsystems (and any other block with a VC_ condition)
    b = find_system(f, opts{:}, 'RegExp', 'on', 'VariantControl', 'VC_');
    for k = 1:numel(b), cond{end+1} = get_param(b{k}, 'VariantControl'); end %#ok<AGROW>
    % Variant Source / Sink blocks
    b = [find_system(f, opts{:}, 'BlockType', 'VariantSource'); find_system(f, opts{:}, 'BlockType', 'VariantSink')];
    for k = 1:numel(b), cond = [cond, reshape(get_param(b{k}, 'VariantControls'), 1, [])]; end %#ok<AGROW>
    % Model blocks: the files they reference, and the variants of a variant Model block
    b = find_system(f, opts{:}, 'BlockType', 'ModelReference');
    for k = 1:numel(b)
        todo{end+1} = get_param(b{k}, 'ModelName'); %#ok<AGROW>
        try
            if strcmp(get_param(b{k}, 'Variant'), 'on')
                mv = get_param(b{k}, 'Variants');
                cond = [cond, {mv.Name}]; %#ok<AGROW>
                todo = [todo, {mv.ModelName}]; %#ok<AGROW>
            end
        catch %#ok<CTCH> a Model block without model variants
        end
    end
    % Subsystem Reference blocks: the files they reference
    b = find_system(f, opts{:}, 'RegExp', 'on', 'BlockType', '^SubSystem$', 'ReferencedSubsystem', '.');
    for k = 1:numel(b), todo{end+1} = get_param(b{k}, 'ReferencedSubsystem'); end %#ok<AGROW>
    for k = 1:numel(cond)
        if ~ischar(cond{k}), continue, end
        t = regexp(cond{k}, '\<(VC_\w+)\s*==\s*(-?\d+(?:\.\d+)?)', 'tokens');
        for j = 1:numel(t)
            n = t{j}{1};
            v = str2double(t{j}{2});
            if isKey(V, n)
                V(n) = unique([V(n), v]);
                W(n) = unique([W(n), {f}], 'stable');
            else
                V(n) = v;
                W(n) = {f};
            end
        end
    end
end
end

function [ok, msg] = local_update(top, what)
% one update of the top model; once more after the R2026b Stateflow assertion 'o != nullptr'
t0 = tic;
ok = false;
msg = '';
again = '';
for attempt = 1:2
    try
        evalc(sprintf('set_param(''%s'', ''SimulationCommand'', ''update'')', top));
        ok = true;
        msg = '';
        break
    catch tmp_vs_err
        msg = local_err_text(tmp_vs_err);
        if attempt == 1 && contains(msg, 'o != nullptr')
            again = ', run again after o != nullptr';
        else
            break
        end
    end
end
if ok
    fprintf('shaddad_variant_sweep: %-20s ok, %.0f s%s\n', what, toc(t0), again);
else
    fprintf('shaddad_variant_sweep: %-20s FAILED, %.0f s%s: %s\n', what, toc(t0), again, msg(1:min(300, end)));
end
end

function s = local_err_text(e)
% error message with its causes, on one line, without hyperlinks, 2000 characters at most
s = local_msg(e);
s = regexprep(s, '<a\s[^>]*>|</a>', '');
s = strtrim(regexprep(s, '\s+', ' '));
if numel(s) > 2000, s = [s(1:1997) '...']; end
end

function s = local_msg(e)
s = e.message;
for k = 1:numel(e.cause)
    s = sprintf('%s | %s', s, local_msg(e.cause{k}));
end
end

function local_restore(base0, before)
% the VC_* values of the base workspace as they were, and every block diagram the sweep loaded closed
% without saving (shaddad_close_since warns about any that stays loaded)
fn = fieldnames(base0);
for k = 1:numel(fn), assignin('base', fn{k}, base0.(fn{k})); end
shaddad_close_since(before);
end
