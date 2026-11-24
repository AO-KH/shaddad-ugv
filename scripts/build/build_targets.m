function rep = build_targets(varargin)
%BUILD_TARGETS  Board builds of the three deployable models (T17, build plan T17.3, D31, D38).
%
%   rep = build_targets()                      Shaddad_MCU (cs_STM32H7, VC_TARGET 1), Shaddad_BMS (cs_STM32G4) and
%                                              Shaddad_Companion (cs_Companion, ROS 2 node); every target reports its
%                                              status (BUILT | BLOCKED | FAILED), code size, task-timing estimate
%                                              and PIL readiness. Missing products give BLOCKED, never an error.
%   rep = build_targets(names)                 names = 'MCU' | 'BMS' | 'Companion' or a cell array of them
%   S   = build_targets('products')            availability of the products / tools the builds need (D31)
%   E   = build_targets('estimate')            task-timing / CPU-load design estimates (P.electronics.target)
%   R   = build_targets('registers')           the stmutilitieslib Register Read//Write placeholders of
%                                              fw_io_in_hw / fw_io_out_hw (T08, D60): intended register parsed
%                                              from each block Description, current mask values, configured flag
%   R   = build_targets('registers', 'apply')  select the intended registers in memory (the SSR files are not
%                                              saved); BLOCKED while STM32CubeCLT (device SVD data) is missing
%   R   = build_targets('peripherals' [, 'apply' [, {'MCU','BMS'}]])
%                                              driver blocks of the HW variants (fw_io_in_hw / fw_io_out_hw of
%                                              T08, Shaddad_BMS IO_In / IO_Out HW of T04) against the CubeMX pin
%                                              maps (stm32_ioc_configure('pinmap')): module, port, pins, AFE SPI
%                                              mode / speed / chip select, conversions; 'apply' sets the
%                                              mismatching parameters in memory (models not saved)
%   F   = build_targets('preflight')           fresh MATLAB process: update diagram of the three models with
%                                              VC_TARGET 1, the board sets and the board setup (peripherals,
%                                              registers); status OK | BLOCKED | FAILED per model (5-20 min)
%   S   = build_targets('preflight', 'script', dir)   only write dir/t17_preflight.m (run it with
%                                              matlab -sd dir -batch t17_preflight, outside the shared session)
%   F   = build_targets('preflight', 'result', file)  classify the result file of such a run
%
%   A board build changes VC_TARGET in the base workspace for its duration (restored afterwards) and
%   attaches a temporary ConfigSetRef 'Shaddad_Build' (-> cs_<board>) to the top model and to every
%   referenced model; models are never saved. Run it in a dedicated MATLAB session.
%   Without Simulink Coder (D31) every build reports BLOCKED; the analytical items (estimate, registers
%   plan, products) and the preflight still run. docs/hw_bringup.md describes the procedure.

if nargin >= 1 && (ischar(varargin{1}) || isstring(varargin{1}))
    switch lower(char(varargin{1}))
        case 'products'
            rep = local_products();
            return
        case 'estimate'
            rep = local_estimate();
            return
        case 'registers'
            rep = local_registers(nargin > 1 && strcmpi(varargin{2}, 'apply'));
            return
        case 'peripherals'
            which = {'MCU', 'BMS'};
            if nargin > 2, which = cellstr(varargin{3}); end
            rep = local_peripherals(nargin > 1 && strcmpi(varargin{2}, 'apply'), which);
            return
        case 'preflight'
            if nargin >= 3 && strcmpi(varargin{2}, 'script')
                rep = local_preflight_script(char(varargin{3}));      % write only (run it outside MATLAB)
            elseif nargin >= 3 && strcmpi(varargin{2}, 'result')
                rep = local_preflight_result(char(varargin{3}), NaN); % classify a result file
            else
                rep = local_preflight();
            end
            return
    end
end
T = local_targets();
if nargin >= 1
    want = cellstr(varargin{1});
    T = T(ismember(lower({T.name}), lower(want)));
    if isempty(T), error('build_targets:name', 'Unknown target (MCU | BMS | Companion)'); end
end
S = local_products();
E = local_estimate();
rep = struct('name', {}, 'model', {}, 'cs', {}, 'status', {}, 'reasons', {}, 'code', {}, 'estimate', {}, ...
             'pil', {}, 't', {});
for k = 1:numel(T)
    rep(end+1) = local_build(T(k), S, E); %#ok<AGROW>
end
end

% ===========================================================================================
function T = local_targets()
T = struct('name', {'MCU', 'BMS', 'Companion'}, 'model', {'Shaddad_MCU', 'Shaddad_BMS', 'Shaddad_Companion'}, ...
           'cs', {'cs_STM32H7', 'cs_STM32G4', 'cs_Companion'}, 'kind', {'stm32', 'stm32', 'ros2'}, ...
           'est', {'mcu', 'bms', ''});
end

% ===========================================================================================
function S = local_products()
v = ver; names = {v.Name};
S.simulink_coder = any(strcmp(names, 'Simulink Coder'));
S.embedded_coder = any(strcmp(names, 'Embedded Coder'));
S.stm32_blockset = any(strcmp(names, 'STM32 Microcontroller Blockset'));
S.ros_toolbox    = any(strcmp(names, 'ROS Toolbox'));
S.cubemx     = local_tool('getInstalledSTM32CubeMX');
S.cubeclt    = local_tool('getInstalledSTM32CubeCLT');
S.gnu        = local_tool('getInstalledGNUToolsForSTM32');
if isempty(S.gnu)
    try
        d = stm32cube.utils.getGnuArmToolsDir;
        if ~isempty(d) && isfolder(d), S.gnu = char(d); end
    catch %#ok<CTCH> blockset helper not available
    end
end
S.programmer = local_tool('getInstalledSTM32CubeProgrammer');
S.cxx = {};
try
    c = mex.getCompilerConfigurations('C++', 'Installed');
    if ~isempty(c), S.cxx = {c.Name}; end
catch %#ok<CTCH>
end
need = {'Simulink Coder', S.simulink_coder; 'Embedded Coder', S.embedded_coder; ...
        'STM32 Microcontroller Blockset', S.stm32_blockset; 'STM32CubeMX', ~isempty(S.cubemx); ...
        'STM32CubeCLT / GNU Tools for STM32', ~isempty(S.cubeclt) || ~isempty(S.gnu)};
S.missing_stm32 = need(~[need{:, 2}], 1)';
need = {'Simulink Coder', S.simulink_coder; 'Embedded Coder', S.embedded_coder; 'ROS Toolbox', S.ros_toolbox};
S.missing_ros2 = need(~[need{:, 2}], 1)';
S.missing_pil = S.missing_stm32;
if isempty(S.programmer), S.missing_pil{end+1} = 'STM32CubeProgrammer'; end
end

function p = local_tool(fn)
p = '';
try
    p = char(stm32cube.hwsetup.stm32Tools.(fn)());
catch %#ok<CTCH> blockset helper missing: tool reported as not installed
end
end

% ===========================================================================================
function E = local_estimate()
% task CPU-load design estimate: sum(WCET / period); the measurement is T-EL-42 (PIL + Execution Profiler)
P = shaddad_params('Assign', false);
tg = P.electronics.target;
for b = {'mcu', 'bms'}
    t = tg.(b{1});
    load = t.task_wcet ./ t.task_T;
    E.(b{1}) = struct('f_cpu', t.f_cpu, 'task_T', t.task_T, 'task_wcet', t.task_wcet, 'task_load', load, ...
                      'cpu_load', sum(load), 'cpu_load_max', tg.cpu_load_max, 'flash', t.flash, ...
                      'flash_max', tg.flash_max, 'source', 'design estimate (P.electronics.target), not measured');
end
end

% ===========================================================================================
function r = local_build(t, S, E)
t0 = tic;
r = struct('name', t.name, 'model', t.model, 'cs', t.cs, 'status', 'BLOCKED', 'reasons', {{}}, ...
           'code', struct('flash', NaN, 'ram', NaN, 'file', '', 'note', ''), 'estimate', [], 'pil', [], 't', 0);
if ~isempty(t.est), r.estimate = E.(t.est); end
switch t.kind
    case 'stm32'
        miss = S.missing_stm32;
        r.pil = struct('ready', isempty(S.missing_pil), 'missing', {S.missing_pil}, ...
                       'note', 'PIL over USART3 (ST-LINK VCP, Connection.SerialModule from the .ioc); board connection checked at run time');
    otherwise
        miss = S.missing_ros2;
        r.pil = struct('ready', false, 'missing', {{'SIL/PIL of the C++ ROS 2 node needs a C++ compiler'}}, ...
                       'note', 'remote colcon build on the Jetson (ssh); deployment verified on the device');
        if ~isempty(S.cxx), r.pil.missing = {}; end
end
if ~isempty(miss)
    r.reasons = strcat({'BLOCKED: '}, miss, {' not installed'});
    r.t = toc(t0);
    return
end
% ---- products available: board build ----
vcOld = evalin('base', 'VC_TARGET');
restoreVC = onCleanup(@() assignin('base', 'VC_TARGET', vcOld));
assignin('base', 'VC_TARGET', 1);
shaddad_configsets('codegen');
sr = stm32_target_sets('apply');
if ~isempty(sr.blocked)
    r.reasons = [r.reasons, sr.blocked];
end
if strcmp(t.kind, 'stm32')
    % board setup of the HW-variant driver blocks (in memory; the model files are never saved)
    pr = local_peripherals(true, {t.name});
    if ~all([pr.ok]), r.reasons{end+1} = 'driver blocks not configured (see build_targets(''peripherals''))'; end
    if strcmp(t.name, 'MCU')
        rr = local_registers(true);
        if ~all([rr.configured]), r.reasons{end+1} = 'register placeholders not configured (see build_targets(''registers''))'; end
    end
end
[mdls, restore] = local_attach_build_set(t.model, t.cs); %#ok<ASGLU> onCleanup restores the models
try
    slbuild(t.model);
    r.status = 'BUILT';
catch tmp_t17_err
    r.status = 'FAILED';
    r.reasons{end+1} = getReport(tmp_t17_err, 'basic');
end
if strcmp(r.status, 'BUILT')
    r.code = local_code_size(t, S);
end
r.t = toc(t0);
end

function [mdls, c] = local_attach_build_set(top, csName)
% temporary ConfigSetRef 'Shaddad_Build' -> csName on the top model and every referenced model
load_system(top);
mdls = find_mdlrefs(top, 'MatchFilter', @Simulink.match.activeVariants);
st = struct('m', mdls, 'loaded', [], 'dirty', '', 'active', '');
for k = 1:numel(mdls)
    st(k).loaded = bdIsLoaded(mdls{k});
    load_system(mdls{k});
    st(k).dirty = get_param(mdls{k}, 'Dirty');
    st(k).active = get_param(getActiveConfigSet(mdls{k}), 'Name');
    ref = Simulink.ConfigSetRef;
    ref.Name = 'Shaddad_Build';
    ref.SourceName = csName;
    if any(strcmp(getConfigSets(mdls{k}), 'Shaddad_Build')), detachConfigSet(mdls{k}, 'Shaddad_Build'); end
    attachConfigSet(mdls{k}, ref, true);
    setActiveConfigSet(mdls{k}, 'Shaddad_Build');
end
c = onCleanup(@() local_restore(st));
end

function local_restore(st)
for k = 1:numel(st)
    try
        setActiveConfigSet(st(k).m, st(k).active);
        detachConfigSet(st(k).m, 'Shaddad_Build');
        if ~st(k).loaded
            close_system(st(k).m, 0);
        elseif strcmp(st(k).dirty, 'off')
            set_param(st(k).m, 'Dirty', 'off');
        end
    catch %#ok<CTCH> best effort: the model is never saved
    end
end
end

function c = local_code_size(t, S)
% flash = text + data, RAM = data + bss of the ELF (arm-none-eabi-size, Berkeley format)
c = struct('flash', NaN, 'ram', NaN, 'file', '', 'note', '');
cgf = Simulink.fileGenControl('get', 'CodeGenFolder');
switch t.kind
    case 'stm32'
        f = dir(fullfile(cgf, '**', [t.model '.elf']));
        if isempty(f), c.note = 'ELF not found'; return; end
        [~, ix] = max([f.datenum]);
        c.file = fullfile(f(ix).folder, f(ix).name);
        tools = {S.gnu, fullfile(S.cubeclt, 'GNU-tools-for-STM32', 'bin')};
        exe = '';
        for k = 1:numel(tools)
            cand = fullfile(tools{k}, 'arm-none-eabi-size.exe');
            if ~isempty(tools{k}) && isfile(cand), exe = cand; break; end
        end
        if isempty(exe), c.note = 'arm-none-eabi-size not found'; return; end
        [st, out] = system(sprintf('"%s" -B "%s"', exe, c.file));
        tok = regexp(out, '\n\s*(\d+)\s+(\d+)\s+(\d+)', 'tokens', 'once');
        if st ~= 0 || isempty(tok), c.note = out; return; end
        n = str2double(tok);
        c.flash = n(1) + n(2);
        c.ram = n(2) + n(3);
    otherwise
        % ROS 2 node: compiled remotely by colcon; the size of the generated sources is reported
        d = dir(fullfile(cgf, '**', '*.cpp'));
        c.flash = NaN; c.ram = NaN;
        c.note = sprintf('remote colcon build; generated C++ sources: %d files, %.0f kB', numel(d), sum([d.bytes]) / 1e3);
end
end

% ===========================================================================================
function R = local_registers(apply)
files = {'fw_io_in_hw', 'fw_io_out_hw'};
tsOf = struct('TIM2_CNT', 'Ts.base', 'RCC_RSR', 'Ts.base', 'FDCAN1_PSR', 'Ts.ctrl', 'FDCAN2_PSR', 'Ts.ctrl');
R = struct('block', {}, 'tgtHW', {}, 'devID', {}, 'regGrpName', {}, 'regName', {}, 'opCode', {}, ...
           'SampleTime', {}, 'current', {}, 'configured', {}, 'status', {});
S = [];
if apply, S = local_products(); end
for f = files
    wasLoaded = bdIsLoaded(f{1});
    load_system(f{1});
    blks = find_system(f{1}, 'LookUnderMasks', 'none', 'FollowLinks', 'on', ...
        'MatchFilter', @Simulink.match.allVariants, 'RegExp', 'on', 'Description', '^INTENDED');
    for k = 1:numel(blks)
        d = get_param(blks{k}, 'Description');
        tok = regexp(d, ['tgtHW ''([^'']+)'', devID ''([^'']+)'', regGrpName ''([^'']+)'', ' ...
                         'regName ''([^'']+)'', opCode ''([^'']+)'''], 'tokens', 'once');
        if isempty(tok), continue; end
        nm = get_param(blks{k}, 'Name');
        ts = '-1';
        if isfield(tsOf, nm), ts = tsOf.(nm); end
        want = {'tgtHW', tok{1}; 'devID', tok{2}; 'regGrpName', tok{3}; 'regName', tok{4}; 'opCode', tok{5}; ...
                'SampleTime', ts};
        cur = struct();
        for j = 1:size(want, 1), cur.(want{j, 1}) = get_param(blks{k}, want{j, 1}); end
        ok = all(cellfun(@(n, v) strcmp(cur.(n), v), want(:, 1), want(:, 2)));
        r = struct('block', blks{k}, 'tgtHW', tok{1}, 'devID', tok{2}, 'regGrpName', tok{3}, 'regName', tok{4}, ...
                   'opCode', tok{5}, 'SampleTime', ts, 'current', cur, 'configured', ok, 'status', '');
        if ok
            r.status = 'CONFIGURED';
        elseif ~apply
            r.status = 'PLACEHOLDER (library defaults)';
        elseif isempty(S.cubeclt)
            r.status = 'BLOCKED: STM32CubeCLT not installed (device SVD data for the register list)';
        else
            r = local_register_apply(r, want);
        end
        R(end+1) = r; %#ok<AGROW>
    end
    if ~wasLoaded && ~apply
        close_system(f{1}, 0);
    end
end
end

function R = local_peripherals(apply, which)
% driver-block parameters of the HW variants derived from the CubeMX pin maps; kind 'num' compares the
% evaluated value (pin vectors, counts, bit rates), 'str' the text
P = shaddad_params('Assign', false);
S = {};
if any(strcmpi(which, 'MCU'))
    m = stm32_ioc_configure('pinmap', 'h743');
    S = [S; ...
        {'fw_io_in_hw',  'DI_GPIOF', 'PortName', m.di_port, 'str'; 'fw_io_in_hw', 'DI_GPIOF', 'PinNumber', m.di_pins, 'num'; ...
         'fw_io_out_hw', 'DO_GPIOG', 'PortName', m.do_ports{1}, 'str'; 'fw_io_out_hw', 'DO_GPIOG', 'PinNumber', m.do_pins{1}, 'num'; ...
         'fw_io_out_hw', 'DO_GPIOE', 'PortName', m.do_ports{2}, 'str'; 'fw_io_out_hw', 'DO_GPIOE', 'PinNumber', m.do_pins{2}, 'num'; ...
         'fw_io_in_hw',  'ADC1_Conv', 'ADCModule', 'ADC1', 'str'; 'fw_io_in_hw', 'ADC1_Conv', 'NumberOfConversions', m.adc1_n, 'num'; ...
         'fw_io_in_hw',  'ADC3_TempSensor', 'ADCModule', 'ADC3', 'str'; ...
         'fw_io_in_hw',  'ADC3_TempSensor', 'NumberOfConversions', m.adc3_n, 'num'; ...
         'fw_io_in_hw',  'SBUS_UART', 'UARTModule', m.uart, 'str'; ...
         'fw_io_in_hw',  'ICM20948', 'I2CModule', m.i2c, 'str'; 'fw_io_in_hw', 'ICM20948', 'BitRate', m.i2c_f, 'num'; ...
         'fw_io_out_hw', 'PWM_TIM1', 'TimerModule', m.pwm{1}, 'str'; 'fw_io_out_hw', 'PWM_TIM4', 'TimerModule', m.pwm{2}, 'str'; ...
         'fw_io_in_hw',  'CAN1_RX/FDCAN_Read', 'FDCANModule', m.fdcan{1}, 'str'; ...
         'fw_io_in_hw',  'CAN2_RX/FDCAN_Read', 'FDCANModule', m.fdcan{2}, 'str'}];
end
if any(strcmpi(which, 'BMS'))
    m = stm32_ioc_configure('pinmap', 'g474');
    hw = 'IO_In/HW/';
    S = [S; ...
        {'Shaddad_BMS', [hw 'AFE_SPI'], 'SPIModuleSelect', m.spi, 'str'; 'Shaddad_BMS', [hw 'AFE_SPI'], 'Mode', 'Master', 'str'; ...
         'Shaddad_BMS', [hw 'AFE_SPI'], 'ClockPolarity', m.spi_cpol, 'str'; 'Shaddad_BMS', [hw 'AFE_SPI'], 'ClockPhase', m.spi_cpha, 'str'; ...
         'Shaddad_BMS', [hw 'AFE_SPI'], 'BusSpeed', floor(m.spi_f), 'num'; ...
         'Shaddad_BMS', [hw 'AFE_SPI'], 'UseCustomSSPin', 'Explicit GPIO calls', 'str'; ...
         'Shaddad_BMS', [hw 'AFE_SPI'], 'Port_Select', m.spi_cs_port, 'str'; 'Shaddad_BMS', [hw 'AFE_SPI'], 'Pin', m.spi_cs_pin, 'num'; ...
         'Shaddad_BMS', [hw 'AFE_SPI'], 'OutputDataLength', P.power.bms.hw.afe_rx_len, 'num'; ...
         'Shaddad_BMS', [hw 'ADC'], 'ADCModule', 'ADC1', 'str'; 'Shaddad_BMS', [hw 'ADC'], 'NumberOfConversions', m.adc1_n, 'num'; ...
         'Shaddad_BMS', [hw 'ADC'], 'SampleTime', 'Ts.bms_fast', 'str'; ...
         'Shaddad_BMS', [hw 'DI'], 'PortName', m.di_port, 'str'; 'Shaddad_BMS', [hw 'DI'], 'PinNumber', m.di_pins, 'num'; ...
         'Shaddad_BMS', [hw 'DI'], 'SampleTime', 'Ts.bms_fast', 'str'; ...
         'Shaddad_BMS', 'IO_Out/HW/DO_contactors', 'PortName', m.do_port, 'str'; ...
         'Shaddad_BMS', 'IO_Out/HW/DO_contactors', 'PinNumber', m.do_pins, 'num'; ...
         'Shaddad_BMS', [hw 'FDCAN1_Read'], 'FDCANModule', m.fdcan{1}, 'str'}];
end
R = struct('block', {}, 'param', {}, 'want', {}, 'current', {}, 'ok', {}, 'status', {});
sysList = unique(S(:, 1), 'stable');
loaded = cellfun(@bdIsLoaded, sysList);
for k = 1:numel(sysList), load_system(sysList{k}); end
for k = 1:size(S, 1)
    b = [S{k, 1} '/' S{k, 2}];
    want = S{k, 4};
    if strcmp(S{k, 5}, 'num')
        if strcmp(S{k, 3}, 'BusSpeed'), ws = sprintf('uint32(%d)', want);
        elseif isscalar(want), ws = sprintf('%d', want);
        else, ws = mat2str(want);
        end
    else
        ws = want;
    end
    r = struct('block', b, 'param', S{k, 3}, 'want', ws, 'current', '', 'ok', false, 'status', '');
    try
        r.current = get_param(b, S{k, 3});
        r.ok = local_param_equal(r.current, want, S{k, 5});
    catch tmp_t17_err
        r.status = ['MISSING: ' tmp_t17_err.message];
        R(end+1) = r; %#ok<AGROW>
        continue
    end
    if r.ok
        r.status = 'OK';
    elseif ~apply
        r.status = 'MISMATCH';
    else
        try
            set_param(b, S{k, 3}, ws);
            r.ok = true;
            r.status = 'SET (in memory, not saved)';
        catch tmp_t17_err
            r.status = ['FAILED: ' strtok(tmp_t17_err.message, newline)];
        end
    end
    R(end+1) = r; %#ok<AGROW>
end
if ~apply
    for k = 1:numel(sysList)
        if ~loaded(k), close_system(sysList{k}, 0); end
    end
end
end

function tf = local_param_equal(cur, want, kind)
if strcmp(kind, 'str')
    tf = strcmp(strtrim(cur), want);
else
    v = str2num(cur); %#ok<ST2NM> pin vectors such as '[0:13]' or 'uint32(664062)'
    tf = isequal(double(v(:)'), double(want(:)'));
end
end

function r = local_register_apply(r, want)
% select the register in memory; a Write gets its value signal (from the '<name>_value' Terminator)
b = r.block; sys = get_param(b, 'Parent'); nm = get_param(b, 'Name');
try
    src = [];
    if strcmp(r.opCode, 'Write')
        tv = [sys '/' nm '_value'];
        if getSimulinkBlockHandle(tv) > 0
            ph = get_param(tv, 'PortHandles');
            ln = get_param(ph.Inport(1), 'Line');
            src = get_param(ln, 'SrcPortHandle');
            delete_line(ln); delete_block(tv);
        end
        trd = [sys '/' nm '_rd'];
        if getSimulinkBlockHandle(trd) > 0
            ph = get_param(trd, 'PortHandles');
            delete_line(get_param(ph.Inport(1), 'Line')); delete_block(trd);
        end
    end
    for j = 1:size(want, 1)
        set_param(b, want{j, 1}, want{j, 2});
    end
    if ~isempty(src)
        ph = get_param(b, 'PortHandles');
        add_line(sys, src, ph.Inport(1), 'autorouting', 'on');
    end
    r.configured = true;
    r.status = 'CONFIGURED (in memory, not saved)';
catch tmp_t17_err
    r.status = ['FAILED: ' tmp_t17_err.message];
end
end

% ===========================================================================================
function F = local_preflight()
% update diagram of the three models with VC_TARGET 1 and their board sets, in a fresh MATLAB process
% (the shared session's VC_TARGET and models are never touched)
d = tempname; mkdir(d);
cleanup = onCleanup(@() rmdir(d, 's'));
S = local_preflight_script(d);
if ispc, exe = fullfile(matlabroot, 'bin', 'matlab.exe'); else, exe = fullfile(matlabroot, 'bin', 'matlab'); end
t = tic;
[st, out] = system(sprintf('"%s" -sd "%s" -batch %s', exe, d, S.name));
wall = toc(t);
if ~isfile(S.result)                     % the process died: record why, the classification reports FAILED
    R = struct('error', sprintf('no result from the fresh process (exit %d): %s', st, out));
    save(S.result, 'R');
end
F = local_preflight_result(S.result, wall);
end

function S = local_preflight_script(d)
% write the preflight script t17_preflight.m into folder d; it saves struct R to d/t17_preflight_res.mat
T = local_targets();
% project start-up data (P, Ts, VC_*, Bus_*, cs_*, mpcRowObj of the row tracker), then VC_TARGET 1 and the board sets
body = {'R.startup = shaddad_startup(''quiet'');', 'assignin(''base'', ''VC_TARGET'', 1);', ...
        'shaddad_configsets(''codegen'');', 'R.sets = stm32_target_sets(''apply'');'};
for k = 1:numel(T)
    setup = {};
    if strcmp(T(k).kind, 'stm32')      % board setup of the HW-variant driver blocks (in memory)
        setup = {sprintf('    R.%s_peripherals = build_targets(''peripherals'', ''apply'', ''%s'');', T(k).name, T(k).name)};
        if strcmp(T(k).name, 'MCU')
            setup{end+1} = '    R.MCU_registers = build_targets(''registers'', ''apply'');'; %#ok<AGROW>
        end
    end
    body = [body, { ...
        sprintf('t1 = tic; m = ''%s''; c = ''%s''; mr = {};', T(k).model, T(k).cs), ...
        'try', ...
        '    load_system(m); mr = find_mdlrefs(m, ''MatchFilter'', @Simulink.match.activeVariants);', ...
        '    for j = 1:numel(mr)', ...
        '        load_system(mr{j}); ref = Simulink.ConfigSetRef; ref.Name = ''Shaddad_Build''; ref.SourceName = c;', ...
        '        attachConfigSet(mr{j}, ref, true); setActiveConfigSet(mr{j}, ''Shaddad_Build'');', ...
        '    end'}, setup, { ...
        '    set_param(m, ''SimulationCommand'', ''update'');', ...
        sprintf('    R.%s = struct(''status'', ''OK'', ''msg'', '''', ''models'', {mr}, ''t'', toc(t1));', T(k).name), ...
        'catch tmp_err', ...
        sprintf('    R.%s = struct(''status'', ''ERROR'', ''msg'', getReport(tmp_err, ''basic''), ''models'', {mr}, ''t'', toc(t1));', T(k).name), ...
        'end', ...
        'for j = numel(mr):-1:1, try, close_system(mr{j}, 0); catch, end, end   % private process: discard'}]; %#ok<AGROW>
end
S.name = 't17_preflight';
S.file = fullfile(d, [S.name '.m']);
S.result = fullfile(d, [S.name '_res.mat']);
pp = arrayfun(@(p) char(p.File), currentProject().ProjectPath, 'UniformOutput', false);
q = @(s) strrep(s, '''', '''''');
L = {'R = struct(); t0 = tic;', 'try'};
for k = 1:numel(pp), L{end+1} = sprintf('    addpath(''%s'');', q(pp{k})); end %#ok<AGROW>
L = [L, strcat({'    '}, body(:)'), {'catch tmp_err', '    R.error = getReport(tmp_err, ''basic'');', 'end', ...
     'R.t_total = toc(t0);', sprintf('save(''%s'', ''R'');', q(S.result))}];
fid = fopen(S.file, 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
end

function F = local_preflight_result(resFile, wall)
T = local_targets();
R = struct('error', sprintf('result file missing: %s', resFile));
if isfile(resFile)
    X = load(resFile, 'R');
    R = X.R;
end
F = struct('wall', wall, 't_total', NaN, 'error', '', 'sets', [], 'setup', struct());
fn = fieldnames(R);
for k = 1:numel(fn)
    if endsWith(fn{k}, {'_peripherals', '_registers'}), F.setup.(fn{k}) = R.(fn{k}); end
end
if isfield(R, 't_total'), F.t_total = R.t_total; end
if isfield(R, 'error'), F.error = R.error; end
if isfield(R, 'sets'), F.sets = R.sets; end
for k = 1:numel(T)
    if isfield(R, T(k).name)
        x = R.(T(k).name);
        x.status = local_classify(x.status, x.msg);
    else
        x = struct('status', 'FAILED', 'msg', ['no result: ' F.error], 'models', {{}}, 't', NaN);
    end
    F.(T(k).name) = x;
end
end

function s = local_classify(s, msg)
% a compile error caused by a missing build product is BLOCKED (D31), anything else FAILED
if strcmp(s, 'OK'), return; end
tools = {'STM32CubeCLT', 'STM32CubeMX', 'GNU Tools for STM32', 'Simulink Coder', 'STM32CubeProgrammer'};
if any(contains(msg, tools, 'IgnoreCase', true))
    s = 'BLOCKED';
else
    s = 'FAILED';
end
end
