function out = stm32_target_sets(mode, cs)
%STM32_TARGET_SETS  Board fields of the code-generation configuration sets (T17, build plan T17.2).
%
%   D   = stm32_target_sets('defs')        T17 fields per set as data: struct array .name .board .ioc
%                                          .fields {name value} (applied) .expect {name value} (checked)
%   rep = stm32_target_sets('apply')       complete cs_STM32H7 / cs_STM32G4 / cs_Companion in the base workspace;
%                                          runs shaddad_configsets('codegen') first when a set or its HardwareBoard
%                                          is missing (cache hit 1-5 s, rebuild 2-5 min)
%   [cs, blocked] = stm32_target_sets('apply', cs)   apply the fields of cs.Name to one Simulink.ConfigSet
%   rep = stm32_target_sets('check')       compare the base sets with D.expect: rep.mismatch (empty = all set)
%
%   Fields (D31, D38, docs/hw_bringup.md):
%   * cs_STM32H7 (Shaddad_MCU): STM32CubeMX project utilities/stm32/shaddad_h743.ioc, selected with the
%     blockset's own routine stm32cube.parameters.setCubeMXProjectDetails (the configuration-dialog path:
%     it also sets DeviceId, Family, CPU clock 480 MHz from the project, RTOS, stack size and the PIL /
%     external-mode serial module USART3), FDCAN1 NominalBaudrate 500e3 (classic), FDCAN2 NominalBaudrate
%     1e6 / DataBaudrate 2e6 (BRS), toolchain 'GNU Tools for STM32'.
%   * cs_STM32G4 (Shaddad_BMS): utilities/stm32/shaddad_g474.ioc (170 MHz), FDCAN1 500e3, 'GNU Tools for STM32'.
%   * cs_Companion (Shaddad_Companion and its referenced models): ROS 2 board, ert C++, ARM 64-bit (LP64)
%     device (Jetson Orin NX), 'Colcon Tools', remote colcon build on the Jetson (ROS 2 install folder and
%     workspace, device address / user), build action 'Build and load', package metadata.
%   Values come from P.electronics.can (bit rates), P.electronics.target (clocks) and P_meta.hw (paths,
%   boards). This is a T17 file, not an edit of params/shaddad_configsets.m, because the content-keyed
%   result caches of T04 / T05 (tests/power/pwr_case_run.m, pp_case.m) hash the full text of
%   shaddad_configsets.m (an edit there forces about 4 h of re-simulation) while the board fields do not
%   change any simulation result. docs/hw_bringup.md gives the one-line hook for shaddad_configsets.

if nargin < 1 || isempty(mode), mode = 'apply'; end
switch lower(char(mode))
    case 'defs'
        out = local_defs();
    case 'apply'
        if nargin > 1
            out = local_apply_one(cs, local_defs());
        else
            out = local_apply_base();
        end
    case 'check'
        out = local_check_base();
    otherwise
        error('stm32_target_sets:mode', 'Unknown mode ''%s'' (defs | apply | check)', mode);
end
end

% -------------------------------------------------------------------------------------------
function D = local_defs()
[P, ~, ~, M] = shaddad_params('Assign', false);
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
can = P.electronics.can; tg = P.electronics.target;
num = @(x) sprintf('%d', round(x));
ctd = 'CoderTargetData.';
D = struct('name', {}, 'board', {}, 'ioc', {}, 'fields', {}, 'expect', {});
% ---- MCU: STM32H743 ----
f = {'Toolchain', 'GNU Tools for STM32'; ...
     [ctd 'FDCAN1.NominalBaudrate'], num(can.can1_bitrate); [ctd 'FDCAN1.DataBaudrate'], num(can.can1_bitrate); ...
     [ctd 'FDCAN2.NominalBaudrate'], num(can.can2_nominal); [ctd 'FDCAN2.DataBaudrate'], num(can.can2_data)};
ioc = fullfile(root, M.hw.cubemx);
e = [f; {[ctd 'STM32CubeMX.ProjectFile'], ioc; [ctd 'STM32CubeMX.Family'], 'STM32H7'; ...
         [ctd 'STM32CubeMX.DeviceId'], 'STM32H743ZITx'; [ctd 'Clocking.cpuClockRateMHz'], tg.mcu.f_cpu / 1e6}];
D(end+1) = struct('name', 'cs_STM32H7', 'board', M.hw.mcu_board, 'ioc', ioc, 'fields', {f}, 'expect', {e});
% ---- BMS: STM32G474 ----
f = {'Toolchain', 'GNU Tools for STM32'; ...
     [ctd 'FDCAN1.NominalBaudrate'], num(can.can1_bitrate); [ctd 'FDCAN1.DataBaudrate'], num(can.can1_bitrate)};
ioc = fullfile(root, M.hw.cubemx_bms);
e = [f; {[ctd 'STM32CubeMX.ProjectFile'], ioc; [ctd 'STM32CubeMX.Family'], 'STM32G4'; ...
         [ctd 'Clocking.cpuClockRateMHz'], tg.bms.f_cpu / 1e6}];
D(end+1) = struct('name', 'cs_STM32G4', 'board', M.hw.bms_board, 'ioc', ioc, 'fields', {f}, 'expect', {e});
% ---- Companion: Jetson Orin NX, ROS 2 node built remotely with colcon ----
f = {'ProdHWDeviceType', 'ARM Compatible->ARM 64-bit (LP64)'; 'Toolchain', 'Colcon Tools'; ...
     [ctd 'ROS.RemoteBuild'], true; ...
     [ctd 'ROS2Install.Folder'], '/opt/ros/humble'; [ctd 'ROS2Install.Workspace'], '/home/shaddad/shaddad_ws'; ...
     [ctd 'BoardParameters.DeviceAddress'], '192.168.55.1'; [ctd 'BoardParameters.Username'], 'shaddad'; ...
     [ctd 'Runtime.BuildAction'], 'Build and load'; ...
     [ctd 'Packaging.MaintainerName'], 'Shaddad UGV'; [ctd 'Packaging.MaintainerEmail'], 'shaddad-ugv@example.com'; ...
     [ctd 'Packaging.License'], 'Proprietary'; [ctd 'Packaging.Version'], '1.0.0'};
e = [f; {'TargetLang', 'C++'; 'SystemTargetFile', 'ert.tlc'}];
D(end+1) = struct('name', 'cs_Companion', 'board', 'Robot Operating System 2 (ROS 2)', 'ioc', '', ...
                  'fields', {f}, 'expect', {e});
end

% -------------------------------------------------------------------------------------------
function rep = local_apply_base()
t0 = tic;
D = local_defs();
rep = struct('sets', {{D.name}}, 'applied', 0, 'blocked', {{}}, 'codegen_called', false, 't', 0);
need = false;
for j = 1:numel(D)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', D(j).name))
        need = true;
    else
        c = evalin('base', D(j).name);
        if ~strcmp(get_param(c, 'HardwareBoard'), D(j).board), need = true; end
    end
end
if need
    shaddad_configsets('codegen');          % complete sets (HardwareBoard included) from the cache
    rep.codegen_called = true;
end
for j = 1:numel(D)
    c = evalin('base', D(j).name);          % Simulink.ConfigSet is a handle: edited in place
    [~, b] = local_apply_one(c, D);
    rep.applied = rep.applied + size(D(j).fields, 1) + ~isempty(D(j).ioc) - numel(b);
    rep.blocked = [rep.blocked, b];
end
rep.t = toc(t0);
end

function [cs, blocked] = local_apply_one(cs, D)
j = find(strcmp({D.name}, cs.Name), 1);
if isempty(j)
    error('stm32_target_sets:name', 'No T17 fields for the configuration set ''%s''', cs.Name);
end
if ~strcmp(get_param(cs, 'HardwareBoard'), D(j).board)
    error('stm32_target_sets:board', '%s: HardwareBoard is ''%s'' (expected ''%s''); run shaddad_configsets(''codegen'') first', ...
          cs.Name, get_param(cs, 'HardwareBoard'), D(j).board);
end
blocked = {};
if ~isempty(D(j).ioc)
    try
        % the blockset's configuration-dialog routine: ProjectFile, DeviceId, Family, CPU clock, RTOS,
        % stack size and serial module from the .ioc (validates the project against the board)
        stm32cube.parameters.setCubeMXProjectDetails(cs, [], '', D(j).ioc);
    catch tmp_t17_err
        blocked{end+1} = sprintf('%s.STM32CubeMX project: %s', cs.Name, tmp_t17_err.message);
        try
            codertarget.data.setParameterValue(cs, 'STM32CubeMX.ProjectFile', D(j).ioc);
        catch %#ok<CTCH> reported above
        end
    end
end
F = D(j).fields;
for k = 1:size(F, 1)
    try
        if startsWith(F{k, 1}, 'CoderTargetData.')
            codertarget.data.setParameterValue(cs, extractAfter(F{k, 1}, 'CoderTargetData.'), F{k, 2});
        elseif ~isequal(get_param(cs, F{k, 1}), F{k, 2})
            set_param(cs, F{k, 1}, F{k, 2});
        end
    catch tmp_t17_err
        blocked{end+1} = sprintf('%s.%s: %s', cs.Name, F{k, 1}, tmp_t17_err.message); %#ok<AGROW>
    end
end
end

% -------------------------------------------------------------------------------------------
function rep = local_check_base()
D = local_defs();
rep = struct('mismatch', {{}}, 'values', struct());
for j = 1:numel(D)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', D(j).name))
        rep.mismatch{end+1} = sprintf('%s missing', D(j).name);
        continue
    end
    cs = evalin('base', D(j).name);
    F = [{'HardwareBoard', D(j).board}; D(j).expect];
    for k = 1:size(F, 1)
        try
            if startsWith(F{k, 1}, 'CoderTargetData.')
                v = codertarget.data.getParameterValue(cs, extractAfter(F{k, 1}, 'CoderTargetData.'));
            else
                v = get_param(cs, F{k, 1});
            end
        catch tmp_t17_err
            v = ['<' tmp_t17_err.message '>'];
        end
        rep.values.(D(j).name).(matlab.lang.makeValidName(F{k, 1})) = v;
        if ~local_same(v, F{k, 2})
            rep.mismatch{end+1} = sprintf('%s.%s = %s (expected %s)', D(j).name, F{k, 1}, local_str(v), local_str(F{k, 2}));
        end
    end
end
end

function tf = local_same(v, e)
if isstring(v), v = char(v); end
if islogical(e)
    tf = (islogical(v) || isnumeric(v)) && isequal(logical(v), e);
elseif isnumeric(e)
    if ischar(v), v = str2double(v); end
    tf = isnumeric(v) && isscalar(v) && abs(double(v) - e) < 1e-9;
else
    tf = ischar(v) && strcmp(v, e);
end
end

function s = local_str(v)
if ischar(v) || isstring(v), s = char(v); else, s = mat2str(v); end
end
