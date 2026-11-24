function [n, rep] = shaddad_check_interfaces(varargin)
%SHADDAD_CHECK_INTERFACES  Compare the root ports of every model / Subsystem Reference file with the
%   wiring contract build_stubs('table') (docs/icd.md section 3; build plan T01.7).
%
%   n = shaddad_check_interfaces()                 check every file of the table that exists
%   n = shaddad_check_interfaces('Files', names)   only these rows (names = file base names)
%   n = shaddad_check_interfaces(..., 'Compiled', true)
%                                                  additionally compile Shaddad_System (current VC_*)
%                                                  and compare compiled sample times / bus types at
%                                                  every instance of an SSR or Model block of the
%                                                  table (catches inherited SSR output rates)
%   n = shaddad_check_interfaces(..., 'Quiet', true)   print nothing
%   [n, rep] = ...      rep(k): .name .file .ok .mismatch (cellstr)
%
%   Static checks per file: Inport / Outport count, order (Port), names (block Name or the PortName of
%   bus element ports), data types (OutDataTypeStr), sample times (Inports always; Outports of
%   Model files always, of SSR files when set - an inherited SSR output (-1) is only checked by
%   'Compiled'), Model files: BusOutputAsStruct 'on' on bus ports (nonvirtual buses at Model
%   boundaries, rule 5); Connection Ports: count, order, names, ConnectionType, Side.
%   Sample-time expressions (e.g. 'Ts.base') are evaluated in the base workspace (P, Ts needed).
%   Models are loaded if necessary and closed again (only those this function loaded).
%   Returns the number of mismatches (0 = every interface matches).

ip = inputParser;
ip.addParameter('Files', {});
ip.addParameter('Compiled', false);
ip.addParameter('Quiet', false);
ip.parse(varargin{:});
o = ip.Results;
if ischar(o.Files) || isstring(o.Files), o.Files = cellstr(o.Files); end

if ~evalin('base', 'exist(''Ts'',''var'') && exist(''P'',''var'')')
    shaddad_params();
end
root = fileparts(fileparts(mfilename('fullpath')));
T = build_stubs('table');
if ~isempty(o.Files)
    T = T(ismember({T.name}, o.Files));
end

rep = struct('name', {}, 'file', {}, 'ok', {}, 'mismatch', {});
for k = 1:numel(T)
    R = T(k);
    f = fullfile(root, R.file);
    r = struct('name', R.name, 'file', R.file, 'ok', true, 'mismatch', {{}});
    if ~isfile(f)
        r.mismatch{end+1} = 'file missing';
    else
        wasLoaded = bdIsLoaded(R.name);
        try
            if ~wasLoaded, load_system(f); end
            r.mismatch = local_static(R);
        catch tmp_ci_err
            r.mismatch{end+1} = ['load/inspect error: ' tmp_ci_err.message];
        end
        if ~wasLoaded && bdIsLoaded(R.name) && ~local_in_use(R.name)
            close_system(R.name, 0);
        end
    end
    r.ok = isempty(r.mismatch);
    rep(end+1) = r; %#ok<AGROW>
end

if o.Compiled
    cm = local_compiled(T);
    for k = 1:numel(cm)
        j = find(strcmp({rep.name}, cm(k).name), 1);
        if isempty(j), continue, end
        rep(j).mismatch = [rep(j).mismatch, cm(k).mismatch];
        rep(j).ok = isempty(rep(j).mismatch);
    end
end

n = sum(cellfun(@numel, {rep.mismatch}));
if ~o.Quiet
    fprintf('shaddad_check_interfaces: %d files, %d mismatches\n', numel(rep), n);
    for k = 1:numel(rep)
        if rep(k).ok
            fprintf('  OK    %s\n', rep(k).name);
        else
            fprintf('  FAIL  %s\n', rep(k).name);
            fprintf('          - %s\n', rep(k).mismatch{:});
        end
    end
end
end

% ===========================================================================================
function mm = local_static(R)
mm = {};
mdl = R.name;
isModel = strcmp(R.kind, 'model');
ins = local_ports(mdl, 'Inport');
outs = local_ports(mdl, 'Outport');
mm = [mm, local_cmp(mdl, 'in', R.in, ins, isModel, true)];
mm = [mm, local_cmp(mdl, 'out', R.out, outs, isModel, false)];
% ---- Connection Ports ----
cps = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
num = zeros(numel(cps), 1);
for k = 1:numel(cps), num(k) = str2double(get_param(cps{k}, 'Port')); end
[~, ord] = sort(num); cps = cps(ord);
if numel(cps) ~= numel(R.conn)
    mm{end+1} = sprintf('%d Connection Ports, expected %d', numel(cps), numel(R.conn));
end
for k = 1:min(numel(cps), numel(R.conn))
    c = R.conn(k);
    nm = get_param(cps{k}, 'Name');
    if ~strcmp(nm, c.name)
        mm{end+1} = sprintf('Connection Port %d named %s, expected %s', k, nm, c.name); %#ok<AGROW>
    end
    ct = get_param(cps{k}, 'ConnectionType');
    if ~strcmp(strtrim(ct), c.type)
        mm{end+1} = sprintf('Connection Port %s type ''%s'', expected ''%s''', c.name, ct, c.type); %#ok<AGROW>
    end
    sd = get_param(cps{k}, 'Side');
    if ~strcmpi(sd, c.side)
        mm{end+1} = sprintf('Connection Port %s on side %s, expected %s', c.name, sd, c.side); %#ok<AGROW>
    end
end
end

function P = local_ports(mdl, type)
% one entry per root port number: name, type, ts (string), block (for bus element ports the block
% that carries the top-level type), BusOutputAsStruct, isBE
blks = find_system(mdl, 'SearchDepth', 1, 'BlockType', type);
P = struct('num', {}, 'name', {}, 'type', {}, 'ts', {}, 'block', {}, 'nvbus', {}, 'isBE', {});
if isempty(blks), return, end
num = zeros(numel(blks), 1);
for k = 1:numel(blks), num(k) = str2double(get_param(blks{k}, 'Port')); end
for p = unique(num)'
    bb = blks(num == p);
    isBE = strcmp(get_param(bb{1}, 'IsBusElementPort'), 'on');
    if isBE
        el = cellfun(@(b) get_param(b, 'Element'), bb, 'UniformOutput', false);
        j = find(cellfun(@isempty, el), 1);
        if isempty(j), j = 1; end
        b = bb{j};
        nm = get_param(b, 'PortName');
        if ~isempty(get_param(b, 'Element')), ty = 'unknown (no top-level bus element block)';
        else, ty = get_param(b, 'OutDataTypeStr');
        end
    else
        b = bb{1};
        nm = get_param(b, 'Name');
        ty = get_param(b, 'OutDataTypeStr');
    end
    ts = local_ts_of(b, type, isBE);
    nv = '';
    if strcmp(type, 'Inport') || isfield(get_param(b, 'DialogParameters'), 'BusOutputAsStruct')
        try, nv = get_param(b, 'BusOutputAsStruct'); catch, nv = ''; end %#ok<CTCH>
    end
    P(end+1) = struct('num', p, 'name', nm, 'type', ty, 'ts', ts, 'block', b, 'nvbus', nv, 'isBE', isBE); %#ok<AGROW>
end
end

function ts = local_ts_of(b, type, isBE)
ts = '-1';
if isfield(get_param(b, 'DialogParameters'), 'SampleTime')
    ts = get_param(b, 'SampleTime');
end
if isBE && strcmp(strtrim(ts), '-1') && strcmp(type, 'Inport')
    % a bus-object typed bus element port cannot carry a sample time; it is pinned by the
    % Signal Specification directly downstream (Shaddad_System root ScenarioIn)
    lh = get_param(b, 'LineHandles');
    if ~isempty(lh.Outport) && lh.Outport(1) > 0
        dst = get_param(lh.Outport(1), 'DstBlockHandle');
        for d = dst(:)'
            if strcmp(get_param(d, 'BlockType'), 'SignalSpecification')
                ts = get_param(d, 'SampleTime');
                break
            end
        end
    end
end
end

function mm = local_cmp(mdl, dir, exp, act, isModel, isIn)
mm = {};
if numel(act) ~= numel(exp)
    mm{end+1} = sprintf('%d %sputs, expected %d', numel(act), dir, numel(exp));
end
nums = [act.num];
if ~isempty(act) && ~isequal(nums(:)', 1:numel(act))
    mm{end+1} = sprintf('%sport numbers are not 1..%d', dir, numel(act));
end
for k = 1:min(numel(act), numel(exp))
    e = exp(k); a = act(k);
    if ~strcmp(a.name, e.name)
        mm{end+1} = sprintf('%s %d named %s, expected %s', dir, k, a.name, e.name); %#ok<AGROW>
    end
    if ~strcmp(local_norm(a.type), local_norm(e.type))
        mm{end+1} = sprintf('%s %s type ''%s'', expected ''%s''', dir, e.name, a.type, e.type); %#ok<AGROW>
    end
    % sample time
    tsStr = strtrim(a.ts);
    inherited = any(strcmp(tsStr, {'-1', '[-1 0]', '[-1, 0]'}));
    if inherited
        if isIn || isModel
            mm{end+1} = sprintf('%s %s has an inherited sample time, expected %g', dir, e.name, e.ts); %#ok<AGROW>
        end
    else
        try
            v = evalin('base', tsStr);
            v = v(1);
        catch tmp_ci_err
            v = NaN;
            mm{end+1} = sprintf('%s %s sample time ''%s'' not evaluable: %s', dir, e.name, tsStr, tmp_ci_err.message); %#ok<AGROW>
        end
        okTs = abs(v - e.ts) < 1e-12 || (e.ts == 0 && abs(v - 1e-3) < 1e-12 && isIn);
        if ~isnan(v) && ~okTs
            mm{end+1} = sprintf('%s %s sample time %g, expected %g', dir, e.name, v, e.ts); %#ok<AGROW>
        end
    end
    if isModel && startsWith(e.type, 'Bus:') && ~a.isBE && ~strcmp(a.nvbus, 'on')
        mm{end+1} = sprintf('%s %s: BusOutputAsStruct must be ''on'' (nonvirtual bus at the Model boundary)', dir, e.name); %#ok<AGROW>
    end
end
if ~isempty(mm), mm = cellfun(@(x) [mdl ': ' x], mm, 'UniformOutput', false); end
mm = mm(:)';
end

function s = local_norm(s)
s = regexprep(strtrim(char(s)), '\s*:\s*', ': ');
end

function tf = local_in_use(name)
% a file is still needed when an integration parent that references it is loaded
tf = false;
for m = {'Shaddad_System', 'Shaddad_Plant', 'Shaddad_Companion'}
    if ~strcmp(m{1}, name) && bdIsLoaded(m{1})
        tf = true; return
    end
end
end

% ===========================================================================================
function cm = local_compiled(T)
% compile the top model with the current variant controls and compare every instance port
cm = struct('name', {}, 'mismatch', {});
top = 'Shaddad_System';
tops = {top, 'Shaddad_Plant', 'Shaddad_Companion'};
loaded = cellfun(@bdIsLoaded, tops);
for k = 1:numel(tops), if ~loaded(k), load_system(tops{k}); end, end
compiled = false;
try
    evalc('feval(top, [], [], [], ''compile'')');     % compile diagnostics are not echoed
    compiled = true;
catch tmp_ci_err
    cm(end+1) = struct('name', top, 'mismatch', {{['compile failed: ' tmp_ci_err.message]}});
end
if compiled
    try
        act = find_mdlrefs(top, 'MatchFilter', @Simulink.match.activeVariants);   % compiled hierarchy only
        for m = intersect(tops, [{top}; act(:)], 'stable')
            if ~bdIsLoaded(m{1}), continue, end
            blks = [find_system(m{1}, 'MatchFilter', @Simulink.match.activeVariants, 'LookUnderMasks', 'all', ...
                        'FollowLinks', 'on', 'BlockType', 'ModelReference'); ...
                    find_system(m{1}, 'MatchFilter', @Simulink.match.activeVariants, 'LookUnderMasks', 'all', ...
                        'FollowLinks', 'on', 'BlockType', 'SubSystem')];
            for b = blks(:)'
                ref = local_refname(b{1});
                if isempty(ref), continue, end
                j = find(strcmp({T.name}, ref), 1);
                if isempty(j), continue, end
                mm = local_inst(b{1}, T(j));
                if ~isempty(mm)
                    cm(end+1) = struct('name', ref, 'mismatch', {mm}); %#ok<AGROW>
                end
            end
        end
    catch tmp_ci_err
        cm(end+1) = struct('name', top, 'mismatch', {{['compiled inspection error: ' tmp_ci_err.message]}});
    end
    feval(top, [], [], [], 'term');
end
for k = numel(tops):-1:1, if ~loaded(k) && bdIsLoaded(tops{k}), close_system(tops{k}, 0); end, end
end

function ref = local_refname(b)
ref = '';
bt = get_param(b, 'BlockType');
if strcmp(bt, 'ModelReference')
    try
        if strcmp(get_param(b, 'Commented'), 'off'), ref = get_param(b, 'ModelName'); end
    catch %#ok<CTCH>
    end
else
    try, ref = get_param(b, 'ReferencedSubsystem'); catch, ref = ''; end %#ok<CTCH>
end
end

function mm = local_inst(b, R)
mm = {};
ph = get_param(b, 'PortHandles');
try
    if strcmp(get_param(b, 'CompiledIsActive'), 'off'), return, end
catch %#ok<CTCH>
end
for k = 1:min(numel(ph.Outport), numel(R.out))
    e = R.out(k);
    st = get_param(ph.Outport(k), 'CompiledSampleTime');
    if iscell(st), st = st{1}; end
    v = st(1);
    ok = abs(v - e.ts) < 1e-12 || (e.ts == 0 && (abs(v - 1e-3) < 1e-12 || v == 0));
    if ~ok
        mm{end+1} = sprintf('%s out %s compiled sample time %g, expected %g', getfullname(b), e.name, v, e.ts); %#ok<AGROW>
    end
    if startsWith(e.type, 'Bus:')
        bt = get_param(ph.Outport(k), 'CompiledPortDataType');
        want = strtrim(extractAfter(e.type, 'Bus:'));
        if ~strcmp(bt, want)
            mm{end+1} = sprintf('%s out %s compiled type %s, expected %s', getfullname(b), e.name, bt, want); %#ok<AGROW>
        end
    end
end
end
