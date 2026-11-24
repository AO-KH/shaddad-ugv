function rep = shaddad_configsets(mode, mdl)
%SHADDAD_CONFIGSETS  Create the Shaddad configuration sets in the base workspace / attach them.
%
%   shaddad_configsets()            same as shaddad_configsets('fixed')
%   shaddad_configsets('fixed')     create all cs_* sets; cs_Shaddad = the fixed-step settings
%   shaddad_configsets('ref')       create all cs_* sets; cs_Shaddad = the reference (daessc) settings
%   shaddad_configsets('codegen')   complete the four code-generation sets (HardwareBoard included):
%                                   load them from the cache, or build them and write the cache (1-5 min);
%                                   replaces only cs_GenARM / cs_STM32H7 / cs_STM32G4 / cs_Companion.
%                                   Call it before any code generation / board build (first codegen use).
%   shaddad_configsets('codegen', true)   rebuild them even when the cache is valid
%   shaddad_configsets('attach', model)
%                                   add Simulink.ConfigSetRef 'Shaddad_Ref' (SourceName 'cs_Shaddad')
%                                   to a loaded model and activate it (creates cs_Shaddad if missing)
%   D = shaddad_configsets('defs')  the settings of every set as data (name/value cells)
%   rep = shaddad_configsets(...)   report struct: .sets (names written), .blocked (codegen fields that
%                                   could not be set), .deferred (HardwareBoard fields not set yet),
%                                   .coder (Simulink Coder installed), .shaddad (mode), .cache (state of
%                                   the codegen sets, see below), .cache_valid (cache matches local_defs),
%                                   .t (seconds)
%
%   Base-workspace writes: cs_SimFixed, cs_SimRef, cs_GenARM, cs_STM32H7, cs_STM32G4, cs_Companion,
%   cs_Shaddad (docs/architecture.md section 7, D31, D58; build plan T01.2).
%   Simulation sets use DefaultParameterBehavior 'Tunable'; code-generation sets 'Inlined'.
%   Code-generation sets are created even when Simulink Coder is missing (D31): every
%   target-specific set_param is wrapped in try/catch and recorded as BLOCKED.
%   T17 owns the STM32 / companion entries of local_defs (CubeMX project, FDCAN rates, ROS 2 build).
%
%   Start-up cost (budget 15 s, build plan T01 acceptance; measured in fresh MATLAB processes):
%   - every set is built from scratch (new Simulink.ConfigSet + set_param); copy() of a set costs
%     ~8 s in a fresh process (one-time target-hardware initialisation), so it is not used; a
%     parameter is set only when it differs from the default (ProdHWDeviceType alone costs ~2 s).
%   - HardwareBoard is the expensive field: STM32H7 ~160 s, STM32G4 ~80 s, ROS 2 ~4 s in a fresh
%     process, and even deserialising complete sets from a cache costs ~4 s (board registry).
%     'fixed' / 'ref' therefore create the codegen sets with every field except HardwareBoard
%     (rep.deferred, note in the Description; rep.cache 'deferred'), unless shaddad_configsets('codegen')
%     already ran in this MATLAB session (then the complete sets come from the cache: rep.cache 'hit').
%   - 'codegen' keeps the complete sets in work/shaddad_configsets_cache.mat. The key is the settings
%     data of local_defs (not the file text) plus CACHE_FMT and the MATLAB version: only a change of a
%     setting, of CACHE_FMT or of the release forces the slow rebuild. The environment variable
%     SHADDAD_CS_CACHE overrides the cache file (t_infra uses it so tests never touch the project cache).

CACHE_FMT = 2;      % bump when the way the sets are built changes (invalidates every cache)
persistent boardsReady   % true once 'codegen' produced complete sets in this MATLAB session
if isempty(boardsReady), boardsReady = false; end

if nargin < 1 || isempty(mode), mode = 'fixed'; end
mode = lower(char(mode));

switch mode
    case 'attach'
        if nargin < 2, error('shaddad_configsets:attach', 'shaddad_configsets(''attach'', model)'); end
        rep = local_attach(char(mdl));
        return
    case 'defs'
        rep = local_defs();
        return
    case {'fixed', 'ref', 'codegen'}
        % fall through
    otherwise
        error('shaddad_configsets:mode', 'Unknown mode ''%s'' (fixed | ref | codegen | attach | defs)', mode);
end

t0 = tic;
rep = struct('sets', {{}}, 'blocked', {{}}, 'deferred', {{}}, 'coder', false, 'shaddad', mode, ...
             'cache', '', 'cache_valid', false, 't', 0);
rep.coder = any(strcmp({ver().Name}, 'Simulink Coder'));
D = local_defs();

if strcmp(mode, 'codegen')
    %% ---- complete code-generation sets only (HardwareBoard included; cache written) ----
    if nargin > 1 && isequal(mdl, true), how = 'force'; else, how = 'full'; end
    [cg, rep] = local_codegen_sets(D, CACHE_FMT, rep, how);
    for k = 1:numel(cg)
        assignin('base', cg{k}.Name, cg{k});
        rep.sets{end+1} = cg{k}.Name;
    end
    boardsReady = isempty(rep.deferred);
    if ~rep.coder
        rep.blocked{end+1} = 'codegen BLOCKED: Simulink Coder not installed (D31)';
    end
    rep.t = toc(t0);
    return
end

%% ---- simulation sets: cs_SimFixed (missions, tests), cs_SimRef (accuracy reference) ----
csF = local_new('cs_SimFixed', D.fixed, D.fixed_desc);
% local Simscape solvers of the plant are switched off by run_scenario('ref') (block parameters)
csR = local_new('cs_SimRef', [D.fixed; D.ref], D.ref_desc);

%% ---- code-generation sets (build only, Inlined parameters; BLOCKED while Simulink Coder is missing) ----
if boardsReady, how = 'cached'; else, how = 'light'; end
[cg, rep] = local_codegen_sets(D, CACHE_FMT, rep, how);

%% ---- cs_Shaddad: the set every ConfigSetRef 'Shaddad_Ref' points to ----
if strcmp(mode, 'ref')
    csS = local_new('cs_Shaddad', [D.fixed; D.ref], D.ref_desc);
else
    csS = local_new('cs_Shaddad', D.fixed, D.fixed_desc);
end

sets = [{csF, csR}, cg, {csS}];
for k = 1:numel(sets)
    assignin('base', sets{k}.Name, sets{k});
    rep.sets{end+1} = sets{k}.Name;
end
if ~rep.coder
    rep.blocked{end+1} = 'codegen BLOCKED: Simulink Coder not installed (D31)';
end
local_refresh_refs();
rep.t = toc(t0);
end

% -------------------------------------------------------------------------------------------
function D = local_defs()
% Every setting as data (name/value pairs applied in order). The cache key of the codegen sets.
D.fixed = { ...
    'SolverType', 'Fixed-step'; 'Solver', 'ode3'; 'FixedStep', '1e-3'; 'StartTime', '0.0'; ...
    'EnableMultiTasking', 'on'; 'AutoInsertRateTranBlk', 'off'; ...
    'MultiTaskRateTransMsg', 'error'; 'SingleTaskRateTransMsg', 'error'; ...
    'AlgebraicLoopMsg', 'error'; 'ArtificialAlgebraicLoopMsg', 'error'; ...
    'StrictBusMsg', 'ErrorOnBusTreatedAsVector'; 'UnconnectedInputMsg', 'warning'; ...
    'InheritedTsInSrcMsg', 'warning'; 'ProdHWDeviceType', 'Intel->x86-64 (Windows64)'; ...
    'SignalLogging', 'on'; 'SignalLoggingName', 'logsout'; 'SaveFormat', 'Dataset'; ...
    'ReturnWorkspaceOutputs', 'on'; 'SimCompilerOptimization', 'on'; ...
    'UpdateModelReferenceTargets', 'IfOutOfDate'; 'DefaultParameterBehavior', 'Tunable'; ...
    'SimscapeLogType', 'none'};
D.fixed_desc = 'Shaddad fixed-step 1 ms ode3, multitasking, explicit rate transitions (arch 7, D58).';
D.ref = { ...
    'SolverType', 'Variable-step'; 'Solver', 'daessc'; 'MaxStep', '1e-3'; ...
    'RelTol', '1e-4'; 'AbsTol', '1e-6'; 'SimscapeLogType', 'local'};
D.ref_desc = 'Shaddad accuracy reference: daessc, MaxStep 1 ms, RelTol 1e-4, AbsTol 1e-6 (arch 7).';
% code-generation sets = fixed settings + DefaultParameterBehavior 'Inlined' + target fields (try/catch)
D.cg_common = {'DefaultParameterBehavior', 'Inlined'};
D.cg = struct('name', {}, 'target', {}, 'desc', {});
D.cg(end+1) = struct('name', 'cs_GenARM', 'target', {{'SystemTargetFile', 'ert.tlc'; ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M'; 'GenCodeOnly', 'on'}}, ...
    'desc', 'Early code-generation check: ert.tlc, generic ARM Cortex-M (M7), GenCodeOnly (D31).');
D.cg(end+1) = struct('name', 'cs_STM32H7', 'target', {{'SystemTargetFile', 'ert.tlc'; ...
    'HardwareBoard', 'STM32H7xx Based (Single-core)'}}, ...
    'desc', 'Shaddad_MCU build: STM32H743 (CubeMX project and FDCAN rates set by T17).');
D.cg(end+1) = struct('name', 'cs_STM32G4', 'target', {{'SystemTargetFile', 'ert.tlc'; ...
    'HardwareBoard', 'STM32G4xx Based'}}, ...
    'desc', 'Shaddad_BMS build: STM32G474 (CubeMX project set by T17).');
D.cg(end+1) = struct('name', 'cs_Companion', 'target', {{'SystemTargetFile', 'ert.tlc'; ...
    'TargetLang', 'C++'; 'HardwareBoard', 'Robot Operating System 2 (ROS 2)'; 'MATLABDynamicMemAlloc', 'on'}}, ...
    'desc', 'Shaddad_Companion build: ert C++, ROS 2 board (Jetson, D38).');
end

% -------------------------------------------------------------------------------------------
function cs = local_new(name, nv, desc)
% Fresh configuration set with the Simscape component; parameters already at their value are skipped.
cs = Simulink.ConfigSet;
cs.Name = name;
local_ssc(cs);
local_set(cs, nv);
cs.Description = desc;
end

function local_set(cs, nv)
for k = 1:size(nv, 1)
    if ~isequal(get_param(cs, nv{k, 1}), nv{k, 2})
        set_param(cs, nv{k, 1}, nv{k, 2});
    end
end
end

function local_ssc(cs)
% Simscape options live in a separate configuration component; a fresh Simulink.ConfigSet
% does not carry it, so attach it explicitly (needed for SimscapeLogType).
try
    get_param(cs, 'SimscapeLogType');
catch
    attachComponent(cs, SSC.SimscapeCC);
end
end

% -------------------------------------------------------------------------------------------
function local_refresh_refs()
% Loaded models keep a cached copy of the referenced set: after cs_Shaddad is replaced (e.g. fixed ->
% ref -> fixed in run_scenario) the first update would fail with "Cannot apply change to solver type
% since model compilation is underway" unless every Shaddad_Ref is refreshed.
bds = find_system('type', 'block_diagram');
for k = 1:numel(bds)
    try
        c = getActiveConfigSet(bds{k});
        if isa(c, 'Simulink.ConfigSetRef') && strcmp(c.SourceName, 'cs_Shaddad') && ...
                strcmp(get_param(bds{k}, 'SimulationStatus'), 'stopped')
            refresh(c);
        end
    catch %#ok<CTCH> library / harness without a configuration set
    end
end
end

% -------------------------------------------------------------------------------------------
function [cg, rep] = local_codegen_sets(D, fmt, rep, how)
% how = 'light'  : every field except HardwareBoard (rep.deferred); the cache is only checked (key)
%       'cached' : complete sets from a valid cache, otherwise 'light'
%       'full'   : complete sets from a valid cache, otherwise build them and write the cache
%       'force'  : build the complete sets and write the cache
cacheFile = getenv('SHADDAD_CS_CACHE');
if isempty(cacheFile)
    root = fileparts(fileparts(mfilename('fullpath')));
    cacheFile = fullfile(root, 'work', 'shaddad_configsets_cache.mat');
end
key = struct('fmt', fmt, 'fixed', {D.fixed}, 'common', {D.cg_common}, 'cg', D.cg, 'ver', version);
names = {D.cg.name};
if isfile(cacheFile) && ~strcmp(how, 'force')
    try
        K = load(cacheFile, 'key');       % the key alone: no config set is deserialised
        rep.cache_valid = isequal(K.key, key);
    catch %#ok<CTCH> corrupt / old cache
    end
end
if rep.cache_valid && any(strcmp(how, {'cached', 'full'}))
    try
        C = load(cacheFile, 'cg', 'blocked');
        if isequal(cellfun(@(c) c.Name, C.cg, 'UniformOutput', false), names)
            cg = C.cg;
            rep.blocked = [rep.blocked, C.blocked];
            rep.cache = 'hit';
            return
        end
    catch %#ok<CTCH> unreadable cache: rebuild
    end
    rep.cache_valid = false;
end
full = any(strcmp(how, {'full', 'force'}));
blocked = {};
deferred = {};
cg = cell(1, numel(D.cg));
for j = 1:numel(D.cg)
    cs = local_new(D.cg(j).name, [D.fixed; D.cg_common], D.cg(j).desc);
    for k = 1:size(D.cg(j).target, 1)
        nv = D.cg(j).target(k, :);
        if ~full && strcmp(nv{1}, 'HardwareBoard')
            deferred{end+1} = sprintf('%s.%s = %s', cs.Name, nv{1}, nv{2}); %#ok<AGROW>
            cs.Description = [cs.Description ' [HardwareBoard deferred: shaddad_configsets(''codegen'')]'];
            continue
        end
        try
            set_param(cs, nv{:});
        catch tmp_cfg_err
            blocked{end+1} = sprintf('%s.%s = %s: %s', cs.Name, nv{1}, nv{2}, tmp_cfg_err.message); %#ok<AGROW>
        end
    end
    cg{j} = cs;
end
rep.blocked = [rep.blocked, blocked];
rep.deferred = deferred;
if ~full
    rep.cache = 'deferred';
    return
end
rep.cache = 'rebuilt';
try
    if ~isfolder(fileparts(cacheFile)), mkdir(fileparts(cacheFile)); end
    save(cacheFile, 'key', 'cg', 'blocked');
    rep.cache_valid = true;
catch %#ok<CTCH> read-only work folder: no cache, sets are still valid
    rep.cache = 'rebuilt, not saved';
end
end

% -------------------------------------------------------------------------------------------
function rep = local_attach(mdl)
rep = struct('sets', {{}}, 'blocked', {{}}, 'deferred', {{}}, ...
             'coder', any(strcmp({ver().Name}, 'Simulink Coder')), 'shaddad', 'attach', 'cache', '', ...
             'cache_valid', false, 't', 0);
if ~bdIsLoaded(mdl), load_system(mdl); end
if ~evalin('base', 'exist(''cs_Shaddad'', ''var'')')
    shaddad_configsets('fixed');
end
existing = getConfigSets(mdl);
if ~any(strcmp(existing, 'Shaddad_Ref'))
    ref = Simulink.ConfigSetRef;
    ref.Name = 'Shaddad_Ref';
    ref.SourceName = 'cs_Shaddad';
    attachConfigSet(mdl, ref, true);
else
    ref = getConfigSet(mdl, 'Shaddad_Ref');
    if isa(ref, 'Simulink.ConfigSetRef')
        ref.SourceName = 'cs_Shaddad';
    end
end
setActiveConfigSet(mdl, 'Shaddad_Ref');
% remove every other (unused) configuration set so the model only carries the reference
others = setdiff(getConfigSets(mdl), {'Shaddad_Ref'});
for k = 1:numel(others)
    detachConfigSet(mdl, others{k});
end
rep.sets = {'Shaddad_Ref'};
end
