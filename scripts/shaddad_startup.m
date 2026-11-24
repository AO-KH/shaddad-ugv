function rep = shaddad_startup(mode)
%SHADDAD_STARTUP  Project startup of the Shaddad UGV model (registered with addStartupFile).
%
%   shaddad_startup()          'normal': data, config sets, MPC object, product / tool report
%   shaddad_startup('quiet')   same without printing (parsim SetupFcn on workers, D58)
%   rep = shaddad_startup(...) returns the product / tool report as a table plus timing
%                              (.t_startup total, .t_project = total without the Simulink product
%                              start, .t_steps per step, .cache / .cache_valid / .deferred state of the
%                              codegen config sets, .sdi SDI archive settings 'applied' | 'unchanged')
%
%   Creates in the base workspace: P, Ts, P_meta, VC_*, Bus_* (shaddad_params), cs_* (shaddad_configsets),
%   mpcRowObj (gnc_designMPC, only when the file exists and VC_ROW_TRACKER == 1).
%   Never runs cd, clear all, close all or bdclose all (build plan rule 1).
%
%   Time budget 15 s (build plan T01 acceptance). It applies to every call in a running session and to
%   the project-open start-up in a fresh MATLAB process, counted from the moment Simulink itself is
%   initialised: the product start-up of Simulink (start_simulink, 8-18 s on the reference PC, paid
%   by any Simulink session on first use and outside the project's control) is measured and reported
%   separately (t_infra testStartup, testStartupFreshProcess). The STM32 / ROS 2 HardwareBoard fields
%   of the codegen sets are set on first codegen use (shaddad_configsets('codegen')), because setting
%   them costs 1-5 min and even loading them from the cache ~4 s (report row 'Codegen config sets').

if nargin < 1 || isempty(mode), mode = 'normal'; end
quiet = strcmpi(mode, 'quiet');
t0 = tic; t1 = tic;

%% ---- Simulink product start (first use in a MATLAB process only; timed separately) ----
start_simulink;
tStep = struct('simulink', toc(t1), 'params', 0, 'configsets', 0, 'mpc', 0, 'report', 0, 'sdi', 0);
t1 = tic;

%% ---- data and configuration sets ----
shaddad_params();
tStep.params = toc(t1); t1 = tic;
cfg = shaddad_configsets('fixed');
tStep.configsets = toc(t1); t1 = tic;
Pb = evalin('base', 'P');

%% ---- MPC object of the row tracker (T14 provides gnc_designMPC) ----
mpcMsg = 'gnc_designMPC not on the path (T14) - skipped';
if exist('gnc_designMPC', 'file') && evalin('base', 'exist(''VC_ROW_TRACKER'',''var'')') ...
        && evalin('base', 'VC_ROW_TRACKER') == 1
    try
        gnc_designMPC(Pb);             % creates mpcRowObj in base
        mpcMsg = 'mpcRowObj created';
    catch tmp_su_err
        mpcMsg = ['gnc_designMPC failed: ' tmp_su_err.message];
        warning('shaddad_startup:mpc', '%s', mpcMsg);
    end
end
tStep.mpc = toc(t1); t1 = tic;

%% ---- product and tool report (D31) ----
prods = {ver().Name};
item = {}; status = {}; note = {};
need = {'Simulink Coder', 'required for every ERT / STM32 / SIL / PIL build (D31)'; ...
        'Embedded Coder', 'ERT code generation'; ...
        'STM32 Microcontroller Blockset', 'STM32 driver blocks, board config sets'; ...
        'Simscape', 'plant networks'; 'Simscape Electrical', 'drive / power'; ...
        'Simscape Battery', 'pack / BMS'; 'ROS Toolbox', 'companion HW I/O'};
for k = 1:size(need, 1)
    item{end+1} = need{k, 1}; %#ok<AGROW>
    status{end+1} = local_ok(any(strcmp(prods, need{k, 1}))); %#ok<AGROW>
    note{end+1} = need{k, 2}; %#ok<AGROW>
end
cc = local_compilers('C'); cxx = local_compilers('C++');
item{end+1} = 'C compiler (mex)';   status{end+1} = local_ok(~isempty(cc));  note{end+1} = local_list(cc, 'none installed (LCC only / MinGW-w64 needed for RTF targets, R2)');
item{end+1} = 'C++ compiler (mex)'; status{end+1} = local_ok(~isempty(cxx)); note{end+1} = local_list(cxx, 'none installed (C++ SIL impossible)');
[cubeOk, cubeNote] = local_cubemx();
item{end+1} = 'STM32CubeMX';        status{end+1} = local_ok(cubeOk); note{end+1} = cubeNote;
[gnuOk, gnuNote] = local_gnu();
item{end+1} = 'GNU Tools for STM32'; status{end+1} = local_ok(gnuOk); note{end+1} = gnuNote;
coderOk = any(strcmp(prods, 'Simulink Coder'));
item{end+1} = 'Code generation';    status{end+1} = local_ok(coderOk);
if coderOk, note{end+1} = 'available'; else, note{end+1} = 'codegen BLOCKED (Simulink Coder missing, D31)'; end
item{end+1} = 'Codegen config sets';
if isempty(cfg.deferred)
    status{end+1} = 'OK'; note{end+1} = sprintf('complete (cache %s)', cfg.cache);
elseif cfg.cache_valid
    status{end+1} = 'DEFERRED'; note{end+1} = 'HardwareBoard set on first codegen use: shaddad_configsets(''codegen'') (cache valid, ~5 s)';
else
    status{end+1} = 'DEFERRED'; note{end+1} = 'HardwareBoard set on first codegen use: shaddad_configsets(''codegen'') (no valid cache: 1-5 min)';
end
item{end+1} = 'MPC object';
if startsWith(mpcMsg, 'mpcRowObj'), status{end+1} = 'OK';
elseif contains(mpcMsg, 'failed'),  status{end+1} = 'FAILED';
else,                               status{end+1} = 'SKIPPED';
end
note{end+1} = mpcMsg;
tab = table(string(item(:)), string(status(:)), string(note(:)), 'VariableNames', {'Item', 'Status', 'Note'});

tStep.report = toc(t1); t1 = tic;

%% ---- Simulation Data Inspector archive (bounded memory in batches) ----
% The SDI archive mode and run limit are persistent preferences, but reading or writing them loads
% the SDI engine (3-4 s in a fresh MATLAB). The values applied last are remembered in the MATLAB
% preference Shaddad_UGV.sdi_archive, so they are re-applied only when they change.
sdiWant = [1, double(Pb.log.sdi_archive_runs)];
sdiHave = [];
try
    if ispref('Shaddad_UGV', 'sdi_archive'), sdiHave = getpref('Shaddad_UGV', 'sdi_archive'); end
catch %#ok<CTCH>
end
if isequal(sdiHave, sdiWant)
    sdiMsg = 'unchanged (persisted)';
else
    try
        Simulink.sdi.setAutoArchiveMode(true);
        Simulink.sdi.setArchiveRunLimit(Pb.log.sdi_archive_runs);
        setpref('Shaddad_UGV', 'sdi_archive', sdiWant);
        sdiMsg = 'applied';
    catch tmp_su_err
        sdiMsg = ['not applied: ' tmp_su_err.message];
        warning('shaddad_startup:sdi', 'SDI archive settings not applied: %s', tmp_su_err.message);
    end
end
tStep.sdi = toc(t1);

tTot = toc(t0);
rep = struct('table', tab, 'codegen_blocked', ~coderOk, 'configsets', {cfg.sets}, ...
             'blocked', {cfg.blocked}, 'deferred', {cfg.deferred}, 'cache', cfg.cache, ...
             'cache_valid', cfg.cache_valid, 'sdi', sdiMsg, 't_steps', tStep, ...
             't_startup', tTot, 't_project', tTot - tStep.simulink);
if ~quiet
    fprintf('\n=== Shaddad UGV startup (%s) ===\n', version);
    fprintf('Data: P, Ts, P_meta, %d VC_*, %d Bus_*; config sets: %s\n', ...
        numel(evalin('base', 'who(''VC_*'')')), numel(evalin('base', 'who(''Bus_*'')')), strjoin(cfg.sets, ', '));
    disp(tab);
    if ~coderOk
        fprintf('codegen BLOCKED: Simulink Coder is not installed - every ERT/STM32/SIL/PIL item is BLOCKED, not failed (D31).\n');
    end
    if ~isempty(cfg.deferred)
        fprintf(['Code-generation config sets: HardwareBoard deferred (%s); shaddad_configsets(''codegen'') ' ...
                 'completes them before code generation / board builds.\n'], strjoin(cfg.deferred, ', '));
    end
    fprintf(['Startup finished in %.1f s: Simulink product start %.1f s, project %.1f s (params %.1f, ' ...
             'config sets %.1f, report %.1f, SDI %.1f %s).\n\n'], rep.t_startup, tStep.simulink, rep.t_project, ...
             tStep.params, tStep.configsets, tStep.report, tStep.sdi, sdiMsg);
end
end

% -------------------------------------------------------------------------------------------
function s = local_ok(tf)
if tf, s = 'OK'; else, s = 'MISSING'; end
end

function c = local_compilers(lang)
c = {};
try
    cfg = mex.getCompilerConfigurations(lang, 'Installed');
    if ~isempty(cfg), c = {cfg.Name}; end
catch %#ok<CTCH>
end
end

function s = local_list(c, emptyMsg)
if isempty(c), s = emptyMsg; else, s = strjoin(c, ', '); end
end

function [ok, note] = local_cubemx()
ok = false; note = 'not found (install before T17 board builds)';
try
    d = stm32cube.hwsetup.stm32Tools.getInstalledSTM32CubeMX;
    if ~isempty(d)
        ok = true; note = char(d.Location);
    end
catch %#ok<CTCH>
end
if ~ok
    % MATLAB preferences written by the STM32 hardware setup (any group named *stm32*, field *cubemx*)
    try
        pr = getpref;
        if isstruct(pr)
            g = fieldnames(pr);
            g = g(contains(lower(g), 'stm32'));
            for k = 1:numel(g)
                f = fieldnames(pr.(g{k}));
                f = f(contains(lower(f), 'cubemx'));
                for j = 1:numel(f)
                    v = pr.(g{k}).(f{j});
                    if (ischar(v) || isstring(v)) && isfolder(v)
                        ok = true; note = char(v);
                    end
                end
            end
        end
    catch %#ok<CTCH>
    end
end
if ~ok
    cand = {fullfile(getenv('ProgramFiles'), 'STMicroelectronics', 'STM32Cube', 'STM32CubeMX')};
    for k = 1:numel(cand)
        if isfolder(cand{k}), ok = true; note = cand{k}; end
    end
end
end

function [ok, note] = local_gnu()
ok = false; note = 'not found (install before T17 board builds)';
try
    d = stm32cube.utils.getGnuArmToolsDir;
    if ~isempty(d) && isfolder(d), ok = true; note = char(d); end
catch %#ok<CTCH>
end
end
