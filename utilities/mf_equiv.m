function r = mf_equiv(oldSlx, newSlx, sub, U, varargin)
%MF_EQUIV  Run two versions of a subsystem side by side on the same inputs and compare their outputs bit for bit.
%
%   r = mf_equiv(oldSlx, newSlx, sub, U)
%   r = mf_equiv(oldSlx, newSlx, sub, U, 'InportTypes', types)
%
%     oldSlx  path of the old file (the task snapshot)
%     newSlx  path of the new file (the rebuilt one)
%     sub     path of the subsystem inside both files, without the model name ('FE_Box', 'Feedback'), or ''
%             for a whole Subsystem Reference file
%     U       Simulink.SimulationData.Dataset with one element per inport of the subsystem, in port order (a bus
%             inport as a structure of timeseries), on the subsystem's own sample time: one uniform time vector,
%             starting at a multiple of the step, the same for every element
%     types   cellstr, one data type per inport for the harness root Inports ('' keeps the default): needed only
%             for a bus inport whose bus type mf_equiv cannot read from the subsystem's own Inport block
%
%     r.same       true when every output has the same class, the same size and the same value at every sample
%                  (as isequaln: NaN equals NaN, and -0 equals +0)
%     r.nSamples   the number of time samples compared (of each output)
%     r.firstDiff  '' when same; else the first difference: output (and bus element), time, element, old and new
%                  value, or what else differs (ports, data type, size)
%     r.nZeroSign  how many output values are zero in both versions with opposite signs (information only)
%
%   How: the old file is copied to a temporary folder under a unique name, so both versions can be loaded at
%   once. A harness model, built in memory, holds the two side by side (copies of the subsystem block for a sub,
%   Subsystem Reference blocks for ''), feeds both from the same root Inports (with the dimensions and data type
%   of each element of U; fixed step = the step of U, no interpolation) and logs both sets of outputs on root
%   Outports. Then everything mf_equiv loaded is closed
%   without saving and the temporary folder is deleted. The block parameters resolve in the base workspace
%   (P, Ts, the Bus_* objects of shaddad_startup).
%   A sub that is itself a Subsystem Reference, a Model block or a library link is an error, because both copies
%   would show the current referenced file: pass that file with sub ''.
%
%   Plan docs/superpowers/plans/2026-11-01-matlab-function-conversion.md, Task 1; decision D70.
%
%   See also t_mf_equiv.

p = inputParser;
p.FunctionName = 'mf_equiv';
p.addRequired('oldSlx', @local_text);
p.addRequired('newSlx', @local_text);
p.addRequired('sub', @local_text);
p.addRequired('U', @(u) isa(u, 'Simulink.SimulationData.Dataset'));
p.addParameter('InportTypes', {}, @iscellstr);
p.parse(oldSlx, newSlx, sub, U, varargin{:});
types = p.Results.InportTypes;
oldAbs = local_file(char(oldSlx));
newAbs = local_file(char(newSlx));
sub = char(sub);
nIn = U.numElements;
if nIn < 1
    error('mf_equiv:input', 'U has no elements.');
end
[t, dt] = local_time(U);
dtStr = sprintf('%.17g', dt);
r = struct('same', false, 'nSamples', 0, 'firstDiff', '', 'nZeroSign', 0);

% ---- unique names, the old copy, both versions loaded ----
[~, id] = fileparts(tempname);
id = regexprep(id, '[^A-Za-z0-9]', '');
id = id(max(1, end - 11):end);
[~, newName] = fileparts(newAbs);
[~, ~, ext] = fileparts(oldAbs);
S = struct('h', ['mfeq_h_' id], 'old', ['mfeq_old_' id], 'new', newName, 'closeNew', false, ...
    'dir', fullfile(tempdir, ['mfeq_' id]), 'onPath', false);
mkdir(S.dir);
try
    copyfile(oldAbs, fullfile(S.dir, [S.old ext]), 'f');
    if isempty(sub)
        addpath(S.dir);              % the Subsystem Reference block of the old copy needs it on the path
        S.onPath = true;
    end
    S.closeNew = local_load_new(newAbs, newName);
    load_system(fullfile(S.dir, [S.old ext]));
catch tmp_err
    local_cleanup(S);
    rethrow(tmp_err);
end
done = onCleanup(@() local_cleanup(S));

% ---- harness: root Inports -> Old and New -> root Outports old_j, new_j ----
new_system(S.h);
set_param(S.h, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepAuto', 'FixedStep', dtStr, ...
    'StartTime', sprintf('%.17g', t(1)), 'StopTime', sprintf('%.17g', t(end)), 'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'SaveTime', 'off', 'SaveState', 'off', ...
    'SaveFinalState', 'off', 'LimitDataPoints', 'off', 'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');
if isempty(sub)
    ssr = 'simulink/Ports & Subsystems/Subsystem Reference';
    bOld = add_block(ssr, [S.h '/Old'], 'ReferencedSubsystem', S.old);
    bNew = add_block(ssr, [S.h '/New'], 'ReferencedSubsystem', S.new);
    srcOld = S.old;
else
    srcOld = [S.old '/' sub];
    local_check_block(srcOld, oldAbs);
    local_check_block([S.new '/' sub], newAbs);
    bOld = add_block(srcOld, [S.h '/Old']);
    bNew = add_block([S.new '/' sub], [S.h '/New']);
end
phO = get_param(bOld, 'PortHandles');
phN = get_param(bNew, 'PortHandles');
other = setdiff(fieldnames(phO), {'Inport', 'Outport'});
if any(cellfun(@(f) ~isempty(phO.(f)) || ~isempty(phN.(f)), other))
    error('mf_equiv:ports', '%s: only subsystems with data inports and outports are supported.', local_label(sub));
end
nOut = numel(phO.Outport);
if numel(phN.Inport) ~= numel(phO.Inport) || numel(phN.Outport) ~= nOut
    r.firstDiff = sprintf('ports: old %d in, %d out; new %d in, %d out', numel(phO.Inport), nOut, ...
        numel(phN.Inport), numel(phN.Outport));
    return
end
if numel(phO.Inport) ~= nIn
    error('mf_equiv:input', '%s has %d inports, U has %d elements.', local_label(sub), numel(phO.Inport), nIn);
end
if nOut < 1
    error('mf_equiv:ports', '%s has no outport.', local_label(sub));
end
[~, inTypes] = local_ports(srcOld, 'Inport', nIn);
[outNames, outTypes] = local_ports(srcOld, 'Outport', nOut);
for k = 1:nIn
    blk = sprintf('%s/u%d', S.h, k);
    add_block('simulink/Sources/In1', blk, 'Port', num2str(k), 'SampleTime', dtStr, 'Interpolate', 'off');
    v = local_values(U.getElement(k));
    ty = '';
    if numel(types) >= k, ty = types{k}; end
    if isstruct(v)
        if isempty(ty), ty = inTypes{k}; end
        if ~startsWith(ty, 'Bus:')
            error('mf_equiv:bus', ['Inport %d of %s carries a bus but its type is ''%s'': pass the bus type with ' ...
                '''InportTypes''.'], k, local_label(sub), ty);
        end
    else
        % a root Inport does not take its dimensions from the loaded data: set them, and the type, from U
        [dims, cls, cplx] = local_sample(v);
        set_param(blk, 'PortDimensions', mat2str(dims));
        if cplx, set_param(blk, 'SignalType', 'complex'); end
        if isempty(ty), ty = cls; end
    end
    if ~isempty(ty), set_param(blk, 'OutDataTypeStr', ty); end
    ph = get_param(blk, 'PortHandles');
    add_line(S.h, ph.Outport(1), phO.Inport(k));
    add_line(S.h, ph.Outport(1), phN.Inport(k));
end
for j = 1:nOut
    pair = {sprintf('%s/old_%d', S.h, j), phO.Outport(j); sprintf('%s/new_%d', S.h, j), phN.Outport(j)};
    for q = 1:2
        add_block('simulink/Sinks/Out1', pair{q, 1}, 'Port', num2str(2*(j - 1) + q));
        if startsWith(outTypes{j}, 'Bus:'), set_param(pair{q, 1}, 'OutDataTypeStr', outTypes{j}); end
        ph = get_param(pair{q, 1}, 'PortHandles');
        add_line(S.h, pair{q, 2}, ph.Inport(1));
    end
end

% ---- simulate and compare ----
in = Simulink.SimulationInput(S.h);
in = in.setExternalInput(U);
o = local_sim(in);
r.same = true;
for j = 1:nOut
    a = o.yout.getElement(2*j - 1);
    b = o.yout.getElement(2*j);
    [s, msg, n, nz] = local_cmp(a.Values, b.Values, sprintf('output %d (%s)', j, outNames{j}));
    if j == 1, r.nSamples = n; end
    r.nZeroSign = r.nZeroSign + nz;
    if ~s && r.same
        r.same = false;
        r.firstDiff = msg;
    end
end
clear done
end

% ===========================================================================================
function tf = local_text(s)
tf = ischar(s) || (isstring(s) && isscalar(s));
end

function a = local_file(f)
% absolute path of an existing file: as given, relative to the current folder, or found on the path
if isfile(f)
    if isempty(regexp(f, '^([A-Za-z]:[\\/]|[\\/]{2})', 'once')), f = fullfile(pwd, f); end
    a = strrep(f, '/', filesep);
else
    a = which(f);
    if isempty(a)
        error('mf_equiv:file', 'No file %s.', f);
    end
end
end

function [t, dt] = local_time(U)
% the common uniform time vector of the elements of U
t = [];
for k = 1:U.numElements
    ts = local_first_ts(local_values(U.getElement(k)));
    if k == 1
        t = ts.Time(:);
    elseif ~isequal(ts.Time(:), t)
        error('mf_equiv:time', 'Element %d of U has another time vector than element 1.', k);
    end
end
if numel(t) < 2
    error('mf_equiv:time', 'U needs at least 2 samples.');
end
dt = t(2) - t(1);
if dt <= 0 || any(abs(diff(t) - dt) > 1e-9*dt)
    error('mf_equiv:time', 'The time vector of U is not uniform.');
end
if abs(t(1)/dt - round(t(1)/dt)) > 1e-9
    error('mf_equiv:time', 'U starts at %g s, not at a multiple of its step %g s.', t(1), dt);
end
end

function [dims, cls, cplx] = local_sample(ts)
% dimensions, Simulink data type ('' when it cannot be named) and complexity of one sample of a timeseries
sz = size(ts.Data);
if ts.IsTimeFirst, dims = sz(2:end); else, dims = sz(1:end - 1); end
if isempty(dims), dims = 1; end
c = class(ts.Data);
if islogical(ts.Data)
    cls = 'boolean';
elseif any(strcmp(c, {'double', 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'}))
    cls = c;
elseif isenum(ts.Data)
    cls = ['Enum: ' c];
else
    cls = '';
end
cplx = isnumeric(ts.Data) && ~isreal(ts.Data);
end

function v = local_values(e)
if isa(e, 'Simulink.SimulationData.Signal'), v = e.Values; else, v = e; end
end

function ts = local_first_ts(v)
if isa(v, 'timeseries')
    ts = v;
elseif isstruct(v)
    f = fieldnames(v);
    ts = local_first_ts(v(1).(f{1}));
else
    error('mf_equiv:input', 'U elements must be timeseries or structures of timeseries, not %s.', class(v));
end
end

function closeIt = local_load_new(f, name)
% load the new file unless it is loaded already (then it stays loaded); it must be that very file and clean
closeIt = false;
if bdIsLoaded(name)
    g = get_param(name, 'FileName');
    if ~strcmpi(strrep(g, '/', filesep), f)
        error('mf_equiv:loaded', '%s is loaded from %s, not from %s.', name, g, f);
    end
    if bdIsDirty(name)
        error('mf_equiv:dirty', '%s has unsaved changes: save or close it first.', name);
    end
else
    load_system(f);
    closeIt = true;
end
end

function local_check_block(b, file)
if getSimulinkBlockHandle(b) == -1
    error('mf_equiv:sub', 'No block %s in %s.', b(strfind(b, '/') + 1:end), file);
end
isRef = strcmp(get_param(b, 'BlockType'), 'ModelReference') || ...
    (strcmp(get_param(b, 'BlockType'), 'SubSystem') && ~isempty(get_param(b, 'ReferencedSubsystem'))) || ...
    any(strcmp(get_param(b, 'LinkStatus'), {'resolved', 'implicit'}));
if isRef
    error('mf_equiv:sub', ['%s in %s is a Subsystem Reference, a Model block or a library link: compare ' ...
        'its referenced file with sub ''''.'], b(strfind(b, '/') + 1:end), file);
end
end

function [names, types] = local_ports(sys, kind, n)
% names and data types of the port blocks of sys (a subsystem or a block diagram) in port order
names = repmat({''}, 1, n);
types = repmat({''}, 1, n);
blks = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', kind);
for k = 1:numel(blks)
    if strcmp(blks{k}, sys), continue, end
    q = str2double(get_param(blks{k}, 'Port'));
    if q < 1 || q > n || ~isempty(names{q}), continue, end
    names{q} = get_param(blks{k}, 'Name');
    types{q} = get_param(blks{k}, 'OutDataTypeStr');
    if isfield(get_param(blks{k}, 'ObjectParameters'), 'IsBusElementPort') && ...
            strcmp(get_param(blks{k}, 'IsBusElementPort'), 'on')
        names{q} = get_param(blks{k}, 'PortName');
        if ~isempty(get_param(blks{k}, 'Element')), types{q} = ''; end
    end
end
end

function o = local_sim(in)
% sim, once more after the transient R2026b Stateflow JIT relink assertion ('o != nullptr')
o = [];
for k = 1:2
    try
        o = sim(in);
        break
    catch tmp_err
        if k == 2 || ~contains(getReport(tmp_err, 'extended', 'hyperlinks', 'off'), 'o != nullptr')
            rethrow(tmp_err);
        end
    end
end
end

function [same, msg, n, nz] = local_cmp(a, b, label)
% compare two logged values (timeseries, or structures of timeseries for a bus) sample by sample
same = true;
msg = '';
n = 0;
nz = 0;
if isstruct(a) || isstruct(b)
    if ~(isstruct(a) && isstruct(b)) || ~isequal(size(a), size(b)) || ~isequal(fieldnames(a), fieldnames(b))
        same = false;
        msg = sprintf('%s: the bus structure differs', label);
        return
    end
    f = fieldnames(a);
    for e = 1:numel(a)
        for k = 1:numel(f)
            if numel(a) > 1, lab = sprintf('%s(%d).%s', label, e, f{k}); else, lab = [label '.' f{k}]; end
            [s, m, nk, zk] = local_cmp(a(e).(f{k}), b(e).(f{k}), lab);
            if n == 0, n = nk; end
            nz = nz + zk;
            if ~s && same
                same = false;
                msg = m;
            end
        end
    end
    return
end
if ~isa(a, 'timeseries') || ~isa(b, 'timeseries')
    error('mf_equiv:log', '%s: unexpected logged values (%s, %s).', label, class(a), class(b));
end
n = numel(a.Time);
if ~isequal(a.Time, b.Time)
    same = false;
    msg = sprintf('%s: the time vectors differ (old %d samples, new %d)', label, numel(a.Time), numel(b.Time));
    return
end
da = a.Data;
db = b.Data;
if ~strcmp(class(da), class(db))
    same = false;
    msg = sprintf('%s: data type old %s, new %s', label, class(da), class(db));
    return
end
if ~isequal(size(da), size(db)) || a.IsTimeFirst ~= b.IsTimeFirst
    same = false;
    msg = sprintf('%s: size old %s, new %s', label, mat2str(size(da)), mat2str(size(db)));
    return
end
if isfloat(da)
    eq = (da == db) | (isnan(da) & isnan(db));
    nz = nnz(da == 0 & db == 0 & (1./da) ~= (1./db));
else
    eq = (da == db);
end
if all(eq(:)), return, end
same = false;
sz = size(da);
if a.IsTimeFirst
    E = reshape(eq, n, []);
    i = find(~all(E, 2), 1);
    e = find(~E(i, :), 1);
    A = reshape(da, n, []);
    B = reshape(db, n, []);
    va = A(i, e);
    vb = B(i, e);
    sz = sz(2:end);
else
    E = reshape(eq, [], n);
    i = find(~all(E, 1), 1);
    e = find(~E(:, i), 1);
    A = reshape(da, [], n);
    B = reshape(db, [], n);
    va = A(e, i);
    vb = B(e, i);
    sz = sz(1:end - 1);
end
if numel(sz) <= 1 || min(sz) == 1
    where = sprintf('element %d', e);
else
    c = cell(1, numel(sz));
    [c{:}] = ind2sub(sz, e);
    where = sprintf('element (%s)', strjoin(cellfun(@num2str, c, 'UniformOutput', false), ','));
end
msg = sprintf('%s, t = %.10g, %s: old %s, new %s', label, a.Time(i), where, local_num(va), local_num(vb));
end

function s = local_num(v)
if isfloat(v), s = sprintf('%.17g', v); else, s = sprintf('%d', v); end
end

function s = local_label(sub)
if isempty(sub), s = 'the referenced subsystem'; else, s = sub; end
end

function local_cleanup(S)
% close what mf_equiv loaded (harness first), drop the temporary folder and the harness's build artefacts
names = {S.h, S.old};
if S.closeNew, names{end + 1} = S.new; end
for k = 1:numel(names)
    try
        if bdIsLoaded(names{k}), close_system(names{k}, 0); end
    catch tmp_err
        warning('mf_equiv:close', 'Could not close %s: %s', names{k}, tmp_err.message);
    end
end
try
    if S.onPath, rmpath(S.dir); end
    if isfolder(S.dir), rmdir(S.dir, 's'); end
    cf = Simulink.fileGenControl('get', 'CacheFolder');
    if isfile(fullfile(cf, [S.h '.slxc'])), delete(fullfile(cf, [S.h '.slxc'])); end
    for d = {'_sfprj', 'sim', '_jitprj'}
        g = fullfile(cf, 'slprj', d{1}, S.h);
        if isfolder(g), rmdir(g, 's'); end
    end
catch tmp_err
    warning('mf_equiv:cleanup', 'Temporary files left: %s', tmp_err.message);
end
end
