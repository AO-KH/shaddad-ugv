function out = stm32_ioc_configure(mode, proj)
%STM32_IOC_CONFIGURE  Generate the Shaddad STM32CubeMX projects from the STM32 blockset NUCLEO templates (T17).
%
%   rep = stm32_ioc_configure()              write utilities/stm32/shaddad_h743.ioc (MCU) and shaddad_g474.ioc (BMS)
%   rep = stm32_ioc_configure('h743')        MCU project only ('g474': BMS project only)
%   E   = stm32_ioc_configure('edits', w)    the edits for w = 'h743' | 'g474' as data:
%                                            .set {key value} rows, .del key prefixes, .ips IP list, .tim timing
%   T   = stm32_ioc_configure('timing', w)   derived timing values (FDCAN bit timing, TIM, I2C, SPI, IWDG)
%   map = stm32_ioc_configure('pinmap', w)   peripheral / port / pin choices for the driver blocks of the HW
%                                            variants (used by build_targets('peripherals'))
%   kv  = stm32_ioc_configure('read', file)  parse an .ioc file -> containers.Map key -> value (raw text)
%
%   Build plan T17.1 (D31, D60, ICD 6): the H743 project is the blockset template
%   P_meta.hw.cubemx_template (NUCLEO-H743ZI2) with USART2 100 kBd 8E2 RX inverted on PD6 (SBUS), FDCAN1
%   500 kbit/s classic (PD0/PD1), FDCAN2 1 Mbit/s nominal / 2 Mbit/s data with BRS (PB12/PB13), I2C1
%   400 kHz (PB8/PB9, ICM-20948), ADC1 8 regular conversions + ADC3 temperature sensor, TIM1 20 kHz PWM
%   CH1/CH2 (PE9/PE11), TIM4 25 kHz CH1/CH2 (PD12/PD13), TIM2 32-bit free-running at 1 MHz (t_us),
%   GPIOF[0:13] inputs, GPIOG[0:15] + GPIOE[0:3] outputs, IWDG1 PR /4 RLR 399 (50 ms). The G474 project
%   (NUCLEO-G474RE template, BMS) gets SPI2 for the AFE, FDCAN1 500 kbit/s, ADC1 6 conversions, 8 GPIO
%   inputs, 5 GPIO outputs and the AFE chip select.
%   Rates come from P.electronics (sbus.baud, can.*, wdg.*, target.*) and P.power.bms.hw; prescalers and
%   bit timings are derived from the clock tree of the template (RCC.*Freq_Value keys) and checked to be
%   exact. Pin maps are frozen here (strings are not allowed in P); docs/hw_bringup.md lists them.
%
%   The .ioc file is the Java-properties text of STM32CubeMX: the first line '#MicroXplorer ...', one
%   key=value per line, keys in ASCII order, '\' escapes kept verbatim. The function copies the template,
%   deletes the template entries of re-assigned pins, applies the edits, rebuilds the Mcu.IP* / Mcu.Pin*
%   lists and writes the file only when its text changes (idempotent). Open the file in STM32CubeMX
%   (6.16, the template version) and regenerate the code when the tool is installed.

if nargin < 1 || isempty(mode), mode = 'all'; end
mode = lower(char(mode));
switch mode
    case 'all'
        out = [local_write('h743'), local_write('g474')];
    case {'h743', 'g474'}
        out = local_write(mode);
    case 'edits'
        [P, M] = local_params();
        out = local_edits(local_which(proj), P, local_clocks(local_template(local_which(proj), M)));
    case 'timing'
        [P, M] = local_params();
        E = local_edits(local_which(proj), P, local_clocks(local_template(local_which(proj), M)));
        out = E.tim;
    case 'pinmap'
        [P, M] = local_params();
        E = local_edits(local_which(proj), P, local_clocks(local_template(local_which(proj), M)));
        out = E.map;
    case 'read'
        [k, v] = local_read(proj);
        out = containers.Map(k, v);
    otherwise
        error('stm32_ioc_configure:mode', 'Unknown mode ''%s'' (all | h743 | g474 | edits | timing | read)', mode);
end
end

% ===========================================================================================
function w = local_which(w)
w = lower(char(w));
if ~any(strcmp(w, {'h743', 'g474'}))
    error('stm32_ioc_configure:which', 'Project must be ''h743'' or ''g474''');
end
end

function [P, M] = local_params()
[P, ~, ~, M] = shaddad_params('Assign', false);
end

function root = local_root()
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

function [tmpl, outFile, name] = local_template(w, M)
switch w
    case 'h743'
        tmpl = fullfile(matlabroot, M.hw.cubemx_template);
        outFile = fullfile(local_root(), M.hw.cubemx);
    case 'g474'
        tmpl = fullfile(matlabroot, 'toolbox', 'stm32b', 'stm32target', 'stm32g4xx_based', 'boards', ...
                        'nucleo_g474re', 'nucleo_g474re.ioc');
        outFile = fullfile(local_root(), M.hw.cubemx_bms);
end
[~, name] = fileparts(outFile);
end

function C = local_clocks(tmpl)
% clock tree of the template (Hz): the RCC.*Freq_Value / *_VALUE keys
[k, v] = local_read(tmpl);
C = struct();
for j = 1:numel(k)
    t = regexp(k{j}, '^RCC\.(\w+?)(Freq_Value|Freq_VALUE)$', 'tokens', 'once');
    if ~isempty(t), C.(t{1}) = str2double(v{j}); end
end
end

% ===========================================================================================
function rep = local_write(w)
[P, M] = local_params();
[tmpl, outFile, name] = local_template(w, M);
if ~isfile(tmpl)
    error('stm32_ioc_configure:template', 'STM32 blockset template not found: %s', tmpl);
end
[keys, vals, header] = local_read(tmpl);
E = local_edits(w, P, local_clocks(tmpl));
% delete the template entries of re-assigned pins and the lists rebuilt below
del = startsWith(keys, [E.del, {'Mcu.IP', 'Mcu.Pin'}]);
keys(del) = []; vals(del) = [];
nDel = nnz(del);
% project name
E.set = [E.set; {'ProjectManager.ProjectFileName', [name '.ioc']; 'ProjectManager.ProjectName', name}];
for j = 1:size(E.set, 1)
    i = find(strcmp(keys, E.set{j, 1}), 1);
    if isempty(i)
        keys{end+1} = E.set{j, 1}; vals{end+1} = E.set{j, 2}; %#ok<AGROW>
    else
        vals{i} = E.set{j, 2};
    end
end
% IP list
for j = 1:numel(E.ips)
    keys{end+1} = sprintf('Mcu.IP%d', j-1); vals{end+1} = E.ips{j}; %#ok<AGROW>
end
keys{end+1} = 'Mcu.IPNb'; vals{end+1} = sprintf('%d', numel(E.ips));
% pin list: every key <pin>.Signal (physical pins in port order, then virtual pins)
sig = keys(endsWith(keys, '.Signal') & ~startsWith(keys, {'SH.', 'Dma.'}));
pins = strrep(extractBefore(sig, strlength(sig) - strlength('.Signal') + 1), '\ ', ' ');
pins = local_pin_sort(pins);
for j = 1:numel(pins)
    keys{end+1} = sprintf('Mcu.Pin%d', j-1); vals{end+1} = pins{j}; %#ok<AGROW>
end
keys{end+1} = 'Mcu.PinsNb'; vals{end+1} = sprintf('%d', numel(pins));
% duplicate keys are an error (would be ambiguous for CubeMX)
[u, ~, ic] = unique(keys);
cnt = accumarray(ic(:), 1);
if any(cnt > 1)
    error('stm32_ioc_configure:dup', 'Duplicate keys: %s', strjoin(u(cnt > 1), ', '));
end
[keys, ix] = sort(keys);           % ASCII order, as written by STM32CubeMX
vals = vals(ix);
lines = [header; strcat(keys(:), '=', vals(:))];
txt = [strjoin(lines, newline) newline];
old = '';
if isfile(outFile), old = fileread(outFile); end
changed = ~strcmp(old, txt);
if changed
    if ~isfolder(fileparts(outFile)), mkdir(fileparts(outFile)); end
    fid = fopen(outFile, 'w');
    if fid < 0, error('stm32_ioc_configure:write', 'Cannot write %s', outFile); end
    fwrite(fid, txt, 'char');
    fclose(fid);
end
rep = struct('project', w, 'file', outFile, 'template', tmpl, 'changed', changed, 'n_keys', numel(keys), ...
             'n_set', size(E.set, 1), 'n_deleted', nDel, 'n_pins', numel(pins), 'n_ips', numel(E.ips), ...
             'timing', E.tim);
end

function p = local_pin_sort(p)
% physical pins by port letter and number (e.g. PA3 < PA11 < PB0), virtual pins (VP_*) last
key = zeros(numel(p), 2);
for j = 1:numel(p)
    t = regexp(p{j}, '^P([A-K])(\d+)', 'tokens', 'once');
    if isempty(t)
        key(j, :) = [100, j];
    else
        key(j, :) = [double(t{1}) - 64, str2double(t{2})];
    end
end
[~, ix] = sortrows([key, (1:numel(p))']);
p = p(ix);
end

% ===========================================================================================
function [keys, vals, header] = local_read(file)
% Java-properties text -> keys / values (raw, escapes kept); comment lines are the header
txt = fileread(file);
L = splitlines(txt);
L = L(~cellfun(@isempty, strtrim(L)));
isC = startsWith(strtrim(L), {'#', '!'});
header = L(isC);
L = L(~isC);
keys = cell(1, numel(L)); vals = cell(1, numel(L));
for j = 1:numel(L)
    t = regexp(L{j}, '^(.*?)(?<!\\)=(.*)$', 'tokens', 'once');
    if isempty(t)
        keys{j} = L{j}; vals{j} = '';
    else
        keys{j} = t{1}; vals{j} = t{2};
    end
end
end

% ===========================================================================================
function E = local_edits(w, P, C)
switch w
    case 'h743', E = local_edits_h743(P, C);
    case 'g474', E = local_edits_g474(P, C);
end
end

function E = local_edits_h743(P, C)
el = P.electronics; tg = el.target.mcu;
S = {};
NVIC_ON = 'true\:0\:0\:false\:false\:true\:true\:true\:true';
% ---- template pins re-assigned: LD1 (PB0) -> ADC, ETH_TXD1 (PB13) -> FDCAN2_TX, LD2 (PE1) -> DO17,
%      USB OVCR (PG7) and ETH (PG11, PG13) -> DO7/DO11/DO13 ----
del = {'PB0.', 'PB13.', 'PE1.', 'PG7.', 'PG11.', 'PG13.', 'SH.GPXTI7.'};

% ---- USART2: SBUS 100 kBd, 8 data + even parity (9-bit word) + 2 stop, RX inverted (RXINV), RX only ----
baud = el.sbus.baud;
assert(abs(C.USART234578 / baud - round(C.USART234578 / baud)) < 1e-9, 'USART2 BRR not exact');
u = {'BaudRate', sprintf('%d', baud); 'VirtualMode-Asynchronous', 'VM_ASYNC'; 'WordLength', 'WORDLENGTH_9B'; ...
     'Parity', 'PARITY_EVEN'; 'StopBits', 'STOPBITS_2'; 'Mode', 'MODE_RX'; 'OverSampling', 'UART_OVERSAMPLING_16'; ...
     'OneBitSampling', 'UART_ONE_BIT_SAMPLE_DISABLE'; 'ClockPrescaler', 'PRESCALER_DIV1'; 'FIFOMode', 'FIFOMODE_DISABLE'; ...
     'TXFIFOThreshold', 'TXFIFO_THRESHOLD_1EIGHTHFULL'; 'RXFIFOThreshold', 'RXFIFO_THRESHOLD_1EIGHTHFULL'; ...
     'AutoBaudRateEnableParam', 'UART_ADVFEATURE_AUTOBAUDRATE_DISABLE'; 'TxPinLevelInvertParam', 'ADVFEATURE_TXINV_DISABLE'; ...
     'RxPinLevelInvertParam', 'ADVFEATURE_RXINV_ENABLE'; 'DataInvertParam', 'ADVFEATURE_DATAINV_DISABLE'; ...
     'SwapParam', 'ADVFEATURE_SWAP_DISABLE'; 'OverrunDisableParam', 'ADVFEATURE_OVERRUN_ENABLE'; ...
     'DMADisableonRxErrorParam', 'ADVFEATURE_DMA_ENABLEONRXERROR'; 'MSBFirstParam', 'ADVFEATURE_MSBFIRST_DISABLE'};
S = [S; local_ip('USART2', u)];
S = [S; local_pin('PD6', 'USART2_RX', 'Asynchronous', 'SBUS_RX'); local_pin('PD5', 'USART2_TX', 'Asynchronous', 'SBUS_TX_NC')];
S = [S; {'NVIC.USART2_IRQn', NVIC_ON}];   % blockset UART Read: buffered RXNE-interrupt receive

% ---- FDCAN1 (CAN1, classic 500 kbit/s) and FDCAN2 (CAN2, CAN-FD 1 M / 2 M with BRS) ----
fk = C.FDCAN;
t1n = local_bittime(fk, el.can.can1_bitrate, tg.can_sp(1), 'nominal');
t1d = local_bittime(fk, el.can.can1_bitrate, tg.can_sp(1), 'data');
t2n = local_bittime(fk, el.can.can2_nominal, tg.can_sp(2), 'nominal');
t2d = local_bittime(fk, el.can.can2_data, tg.can_sp(2), 'data');
S = [S; local_fdcan('FDCAN1', 'FDCAN_FRAME_CLASSIC', t1n, t1d, true, 0, tg.can_std_filters(1), ...
                    tg.can_rx_fifo(1), tg.can_tx_fifo(1), 'FDCAN_DATA_BYTES_8')];
S = [S; local_fdcan('FDCAN2', 'FDCAN_FRAME_FD_BRS', t2n, t2d, true, tg.fdcan2_ram_offset, tg.can_std_filters(2), ...
                    tg.can_rx_fifo(2), tg.can_tx_fifo(2), 'FDCAN_DATA_BYTES_64')];
S = [S; local_pin('PD0', 'FDCAN1_RX', 'FDCAN_Activate', 'CAN1_RX'); local_pin('PD1', 'FDCAN1_TX', 'FDCAN_Activate', 'CAN1_TX'); ...
        local_pin('PB12', 'FDCAN2_RX', 'FDCAN_Activate', 'CAN2_RX'); local_pin('PB13', 'FDCAN2_TX', 'FDCAN_Activate', 'CAN2_TX')];

% ---- I2C1 400 kHz (ICM-20948 at 0x69) ----
ti = local_i2c(C.I2C123, tg);
S = [S; local_ip('I2C1', {'I2C_Speed_Mode', 'I2C_Fast'; 'Timing', sprintf('0x%08X', ti.timing)})];
S = [S; local_pin('PB8', 'I2C1_SCL', 'I2C', 'IMU_SCL'); local_pin('PB9', 'I2C1_SDA', 'I2C', 'IMU_SDA')];

% ---- ADC1: 8 regular conversions (ICD 6 order), 16 bit, software trigger, scan ----
adc = {'PA3', 15, 'ADC_VPACK'; 'PC0', 10, 'ADC_VBUS'; 'PB1', 5, 'ADC_IPACK'; 'PA6', 3, 'ADC_V24'; ...
       'PA4', 18, 'ADC_V12'; 'PA5', 19, 'ADC_V5'; 'PA0', 16, 'ADC_NTC_ENCL'; 'PB0', 9, 'ADC_LIFT_POT'};
assert(size(adc, 1) == el.hw.adc1_n, 'ADC1 pin map must have P.electronics.hw.adc1_n channels');
exclusive = [16 17];                       % ADC1-only inputs (PA0 INP16, PA1 INP17); the others are shared ADCx
S = [S; local_adc('ADC1', [adc{:, 2}], 'ADC_SAMPLETIME_64CYCLES_5', 'ADC_RESOLUTION_16B', true)];
for j = 1:size(adc, 1)
    n = adc{j, 2};
    if any(n == exclusive)
        S = [S; local_pin(adc{j, 1}, sprintf('ADC1_INP%d', n), sprintf('IN%d-Single-Ended', n), adc{j, 3})]; %#ok<AGROW>
    else
        S = [S; local_pin(adc{j, 1}, sprintf('ADCx_INP%d', n), '', adc{j, 3}); ...
                {sprintf('SH.ADCx_INP%d.0', n), sprintf('ADC1_INP%d,IN%d-Single-Ended', n, n); ...
                 sprintf('SH.ADCx_INP%d.ConfNb', n), '1'}]; %#ok<AGROW>
    end
end
% ---- ADC3: internal temperature sensor (T_mcu) ----
S = [S; local_adc('ADC3', -1, 'ADC_SAMPLETIME_810CYCLES_5', 'ADC_RESOLUTION_16B', false)];
S = [S; {'VP_ADC3_TempSens_Input.Mode', 'IN-TempSens'; 'VP_ADC3_TempSens_Input.Signal', 'ADC3_TempSens_Input'}];

% ---- TIM1 20 kHz PWM CH1/CH2 (pump, lift), TIM4 25 kHz CH1/CH2 (fans), TIM2 1 MHz free-running ----
[p1, a1] = local_tim(C.Tim1Output, tg.pwm_f(1), 16);
[p4, a4] = local_tim(C.Tim2Output, tg.pwm_f(2), 16);         % TIM4 is an APB1 timer (same clock as TIM2)
p2 = C.Tim2Output / tg.tim2_f - 1;                            % counter clock = tim2_f (1 MHz tick)
assert(abs(p2 - round(p2)) < 1e-9 && p2 <= 65535, 'TIM2 prescaler for %g Hz not exact', tg.tim2_f);
p2 = round(p2);
a2 = 2^32 - 1;                                                % free-running: full 32-bit period
S = [S; local_pwm('TIM1', p1, a1, {'PE9', 'PWM_PUMP'; 'PE11', 'PWM_LIFT'})];
S = [S; local_pwm('TIM4', p4, a4, {'PD12', 'PWM_FAN1'; 'PD13', 'PWM_FAN2'})];
S = [S; local_ip('TIM2', {'Prescaler', sprintf('%d', p2); 'Period', sprintf('%d', a2)}); ...
        {'VP_TIM2_VS_ClockSourceINT.Mode', 'Internal'; 'VP_TIM2_VS_ClockSourceINT.Signal', 'TIM2_VS_ClockSourceINT'}];

% ---- GPIO: DI word GPIOF[0:13] (ICD 6), DO word GPIOG[0:15] + GPIOE[0:3] ----
diName = local_bitnames(el.di, el.hw.di_pins, 'DI');
for b = 0:el.hw.di_pins-1
    S = [S; local_pin(sprintf('PF%d', b), 'GPIO_Input', '', sprintf('DI%d_%s', b, diName{b+1}))]; %#ok<AGROW>
end
nDo = el.hw.do_g_pins + el.hw.do_e_pins;
doName = local_bitnames(el.do, nDo, 'DO');
for b = 0:nDo-1
    if b < el.hw.do_g_pins, pn = sprintf('PG%d', b); else, pn = sprintf('PE%d', b - el.hw.do_g_pins); end
    S = [S; local_pin(pn, 'GPIO_Output', '', sprintf('DO%d_%s', b, doName{b+1}))]; %#ok<AGROW>
end

% ---- IWDG1: LSI 32 kHz, PR /4, RLR 399 -> 50 ms ----
pr = el.wdg.prescaler; rl = el.wdg.reload;
assert(abs(pr * (rl + 1) / 32e3 - el.wdg.timeout) < 1e-12, 'IWDG1 timeout mismatch');
S = [S; local_ip('IWDG1', {'Prescaler', sprintf('IWDG_PRESCALER_%d', pr); 'Reload', sprintf('%d', rl)}); ...
        {'VP_IWDG1_VS_IWDG.Mode', 'IWDG_Activate'; 'VP_IWDG1_VS_IWDG.Signal', 'IWDG1_VS_IWDG'}];

% ---- generated init functions (blockset drivers: LL for GPIO/DMA/RCC/USART/ADC/I2C/TIM/IWDG, HAL for FDCAN) ----
fl = {'MX_GPIO_Init', 'GPIO', 'LL'; 'MX_DMA_Init', 'DMA', 'LL'; 'SystemClock_Config', 'RCC', 'LL'; ...
      'MX_USART3_UART_Init', 'USART3', 'LL'; 'MX_USB_OTG_FS_PCD_Init', 'USB_OTG_FS', 'HAL'; ...
      'MX_USART2_UART_Init', 'USART2', 'LL'; 'MX_FDCAN1_Init', 'FDCAN1', 'HAL'; 'MX_FDCAN2_Init', 'FDCAN2', 'HAL'; ...
      'MX_I2C1_Init', 'I2C1', 'LL'; 'MX_ADC1_Init', 'ADC1', 'LL'; 'MX_ADC3_Init', 'ADC3', 'LL'; ...
      'MX_TIM1_Init', 'TIM1', 'LL'; 'MX_TIM2_Init', 'TIM2', 'LL'; 'MX_TIM4_Init', 'TIM4', 'LL'; ...
      'MX_IWDG1_Init', 'IWDG1', 'LL'};
S = [S; {'ProjectManager.functionlistsort', [local_funclist(fl) ',0-MX_CORTEX_M7_Init-CORTEX_M7-false-HAL-true']}];

E.set = S;
E.del = del;
E.ips = {'ADC1', 'ADC3', 'CORTEX_M7', 'DMA', 'FDCAN1', 'FDCAN2', 'I2C1', 'IWDG1', 'NVIC', 'RCC', 'SYS', ...
         'TIM1', 'TIM2', 'TIM4', 'USART2', 'USART3', 'USB_OTG_FS', 'NUCLEO-H743ZI2'};
E.tim = struct('fdcan_clk', fk, 'fdcan1_nominal', t1n, 'fdcan1_data', t1d, 'fdcan2_nominal', t2n, ...
    'fdcan2_data', t2d, 'i2c1', ti, 'usart2_baud', baud, ...
    'tim1', struct('clk', C.Tim1Output, 'psc', p1, 'arr', a1, 'f', C.Tim1Output / ((p1 + 1) * (a1 + 1))), ...
    'tim4', struct('clk', C.Tim2Output, 'psc', p4, 'arr', a4, 'f', C.Tim2Output / ((p4 + 1) * (a4 + 1))), ...
    'tim2', struct('clk', C.Tim2Output, 'psc', p2, 'arr', a2, 'f', C.Tim2Output / (p2 + 1)), ...
    'iwdg1', struct('pr', pr, 'rlr', rl, 'timeout', pr * (rl + 1) / 32e3), ...
    'adc1_pins', {adc(:, 1)'}, 'adc1_ch', [adc{:, 2}], 'cpu', C.CpuClock);
E.map = struct('uart', 'USART2', 'i2c', 'I2C1', 'i2c_f', tg.i2c_f, 'fdcan', {{'FDCAN1', 'FDCAN2'}}, ...
    'adc1_n', size(adc, 1), 'adc3_n', 1, 'pwm', {{'TIM1', 'TIM4'}}, ...
    'di_port', 'GPIOF', 'di_pins', 0:el.hw.di_pins-1, ...
    'do_ports', {{'GPIOG', 'GPIOE'}}, 'do_pins', {{0:el.hw.do_g_pins-1, 0:el.hw.do_e_pins-1}});
end

function E = local_edits_g474(P, C)
el = P.electronics; tg = el.target.bms;
S = {};
% ---- FDCAN1 classic 500 kbit/s (CAN1: BMS <-> MCU) ----
fk = C.FDCAN;
tn = local_bittime(fk, el.can.can1_bitrate, tg.can_sp, 'nominal');
td = local_bittime(fk, el.can.can1_bitrate, tg.can_sp, 'data');
S = [S; local_fdcan('FDCAN1', 'FDCAN_FRAME_CLASSIC', tn, td, false, 0, 8, 0, 0, '')];
S = [S; local_pin('PA11', 'FDCAN1_RX', 'FDCAN_Activate', 'CAN1_RX'); local_pin('PA12', 'FDCAN1_TX', 'FDCAN_Activate', 'CAN1_TX')];

% ---- SPI2 master to the AFE isoSPI bridge (mode 3, MSB first, 8 bit), software chip select PB12 ----
pres = 2.^(1:8);
ix = find(C.APB1 ./ pres <= tg.spi_f_max, 1);
assert(~isempty(ix), 'no SPI2 prescaler meets P.electronics.target.bms.spi_f_max');
fs = C.APB1 / pres(ix);
sp = {'VirtualType', 'VM_MASTER'; 'Mode', 'SPI_MODE_MASTER'; 'Direction', 'SPI_DIRECTION_2LINES'; ...
      'DataSize', 'SPI_DATASIZE_8BIT'; 'CLKPolarity', 'SPI_POLARITY_HIGH'; 'CLKPhase', 'SPI_PHASE_2EDGE'; ...
      'FirstBit', 'SPI_FIRSTBIT_MSB'; 'NSSPMode', 'SPI_NSS_PULSE_DISABLE'; ...
      'BaudRatePrescaler', sprintf('SPI_BAUDRATEPRESCALER_%d', pres(ix)); ...
      'CalculateBaudRate', sprintf('%.3f KBits/s', fs / 1e3)};
S = [S; local_ip('SPI2', sp)];
S = [S; local_pin('PB13', 'SPI2_SCK', 'Full_Duplex_Master', 'AFE_SCK'); ...
        local_pin('PB14', 'SPI2_MISO', 'Full_Duplex_Master', 'AFE_MISO'); ...
        local_pin('PB15', 'SPI2_MOSI', 'Full_Duplex_Master', 'AFE_MOSI')];
S = [S; {'PB12.GPIOParameters', 'PinState,GPIO_Label'; 'PB12.GPIO_Label', 'AFE_CS'; 'PB12.Locked', 'true'; ...
         'PB12.PinState', 'GPIO_PIN_SET'; 'PB12.Signal', 'GPIO_Output'}];

% ---- ADC1: 6 regular conversions in the Bus_BMS_Meas order of Shaddad_BMS IO_In HW ----
adc = {'PA0', 1, 'ADC_IPACK'; 'PA1', 2, 'ADC_VPACK'; 'PC0', 6, 'ADC_VBUS'; 'PC1', 7, 'ADC_VDOCK'; ...
       'PC2', 8, 'ADC_TAMB'; 'PC3', 9, 'ADC_TAIR'};
assert(size(adc, 1) == tg.adc_n && tg.adc_n == numel(P.power.bms.hw.adc_gain), 'BMS ADC channel count');
S = [S; local_adc('ADC1', [adc{:, 2}], 'ADC_SAMPLETIME_92CYCLES_5', 'ADC_RESOLUTION_12B', true)];
for j = 1:size(adc, 1)
    S = [S; local_pin(adc{j, 1}, sprintf('ADC1_IN%d', adc{j, 2}), sprintf('IN%d-Single-Ended', adc{j, 2}), adc{j, 3})]; %#ok<AGROW>
end

% ---- GPIO: aux contacts / status inputs GPIOC[4:11], contactor / charger / balancing outputs GPIOB ----
di = {'K1_FB', 'K2_FB', 'K4_FB', 'DOCK_PRESENT', 'SAFETY_OK', 'SAFETY_HV_OK', 'KEY_ON', 'FUSE_MAIN_OK'};
assert(numel(di) == tg.di_n);
for b = 1:numel(di)
    S = [S; local_pin(sprintf('PC%d', b + 3), 'GPIO_Input', '', sprintf('DI%d_%s', b - 1, di{b}))]; %#ok<AGROW>
end
do = {'PB4', 'K1_CMD'; 'PB5', 'K2_CMD'; 'PB6', 'K4_CMD'; 'PB7', 'CHARGER_EN'; 'PB10', 'BAL_EN'};
assert(size(do, 1) == tg.do_n);
for b = 1:size(do, 1)
    S = [S; local_pin(do{b, 1}, 'GPIO_Output', '', sprintf('DO%d_%s', b - 1, do{b, 2}))]; %#ok<AGROW>
end

fl = {'MX_GPIO_Init', 'GPIO', 'LL'; 'MX_DMA_Init', 'DMA', 'LL'; 'SystemClock_Config', 'RCC', 'LL'; ...
      'MX_LPUART1_UART_Init', 'LPUART1', 'LL'; 'MX_ADC1_Init', 'ADC1', 'LL'; 'MX_FDCAN1_Init', 'FDCAN1', 'HAL'; ...
      'MX_SPI2_Init', 'SPI2', 'LL'};
S = [S; {'ProjectManager.functionlistsort', local_funclist(fl)}];

E.set = S;
E.del = {};
E.ips = {'ADC1', 'DMA', 'FDCAN1', 'LPUART1', 'NVIC', 'RCC', 'SPI2', 'SYS', 'NUCLEO-G474RE'};
E.tim = struct('fdcan_clk', fk, 'fdcan1_nominal', tn, 'fdcan1_data', td, ...
    'spi2', struct('clk', C.APB1, 'prescaler', pres(ix), 'f', fs), ...
    'adc1_pins', {adc(:, 1)'}, 'adc1_ch', [adc{:, 2}], 'cpu', C.SYSCLK);
E.map = struct('spi', 'SPI2', 'spi_f', fs, 'spi_cpol', 'High', 'spi_cpha', '2 Edge', 'spi_cs_port', 'GPIOB', ...
    'spi_cs_pin', 12, 'fdcan', {{'FDCAN1'}}, 'adc1_n', size(adc, 1), 'di_port', 'GPIOC', 'di_pins', (1:numel(di)) + 3, ...
    'do_port', 'GPIOB', 'do_pins', cellfun(@(p) str2double(p(3:end)), do(:, 1))');
end

% ===========================================================================================
function S = local_ip(ip, nv)
% IP parameter rows + the IPParameters list (names in the given order)
S = [strcat({[ip '.']}, nv(:, 1)), nv(:, 2)];
S = [S; {[ip '.IPParameters'], strjoin(nv(:, 1)', ',')}];
end

function S = local_pin(pin, signal, mode, label)
S = {[pin '.GPIOParameters'], 'GPIO_Label'; [pin '.GPIO_Label'], label; [pin '.Locked'], 'true'};
if ~isempty(mode), S = [S; {[pin '.Mode'], mode}]; end
S = [S; {[pin '.Signal'], signal}];
end

function S = local_fdcan(ip, frame, tn, td, ramCfg, ramOff, nFilt, nRx, nTx, elmt)
nv = {'FrameFormat', frame; 'Mode', 'FDCAN_MODE_NORMAL'; 'AutoRetransmission', 'ENABLE'; ...
      'TransmitPause', 'DISABLE'; 'ProtocolException', 'DISABLE'; ...
      'NominalPrescaler', sprintf('%d', tn.presc); 'NominalSyncJumpWidth', sprintf('%d', tn.sjw); ...
      'NominalTimeSeg1', sprintf('%d', tn.seg1); 'NominalTimeSeg2', sprintf('%d', tn.seg2); ...
      'DataPrescaler', sprintf('%d', td.presc); 'DataSyncJumpWidth', sprintf('%d', td.sjw); ...
      'DataTimeSeg1', sprintf('%d', td.seg1); 'DataTimeSeg2', sprintf('%d', td.seg2); ...
      'StdFiltersNbr', sprintf('%d', nFilt); 'ExtFiltersNbr', '0'; 'TxFifoQueueMode', 'FDCAN_TX_FIFO_OPERATION'};
if ramCfg       % STM32H7: shared 10 KB message RAM, per-instance offset / element sizes
    nv = [nv; {'MessageRAMOffset', sprintf('%d', ramOff); 'RxFifo0ElmtsNbr', sprintf('%d', nRx); ...
               'RxFifo0ElmtSize', elmt; 'TxFifoQueueElmtsNbr', sprintf('%d', nTx); 'TxElmtSize', elmt}];
end
nv = [nv; {'CalculateBaudRateNominal', sprintf('%d', round(tn.rate)); ...
           'CalculateTimeBitNominal', local_num(1e9 / tn.rate); 'CalculateTimeQuantumNominal', local_num(1e9 * tn.presc / tn.fclk); ...
           'CalculateBaudRateData', sprintf('%d', round(td.rate)); ...
           'CalculateTimeBitData', local_num(1e9 / td.rate); 'CalculateTimeQuantumData', local_num(1e9 * td.presc / td.fclk)}];
S = local_ip(ip, nv);
end

function S = local_adc(ip, ch, tsmp, res, scan)
% regular conversions in rank order; ch = -1: internal temperature sensor
nv = {};
for k = 0:numel(ch)-1
    if ch(k+1) < 0, cs = 'ADC_CHANNEL_TEMPSENSOR'; else, cs = sprintf('ADC_CHANNEL_%d', ch(k+1)); end
    t = sprintf('-%d\\#ChannelRegularConversion', k);
    nv = [nv; {['Rank' t], sprintf('%d', k+1); ['Channel' t], cs; ['SamplingTime' t], tsmp; ...
               ['OffsetNumber' t], 'ADC_OFFSET_NONE'}]; %#ok<AGROW>
end
nv = [nv; {'NbrOfConversionFlag', '1'; 'Resolution', res; 'ContinuousConvMode', 'DISABLE'; ...
           'NbrOfConversion', sprintf('%d', numel(ch))}];
if scan, nv = [nv; {'ScanConvMode', 'ADC_SCAN_ENABLE'}]; end
if strcmp(ip, 'ADC1'), nv = [nv; {'master', '1'}]; end
S = local_ip(ip, nv);
end

function S = local_pwm(tim, psc, arr, pins)
nv = {};
for c = 1:size(pins, 1)
    nv = [nv; {sprintf('Channel-PWM\\ Generation%d\\ CH%d', c, c), sprintf('TIM_CHANNEL_%d', c)}]; %#ok<AGROW>
end
nv = [nv; {'Prescaler', sprintf('%d', psc); 'Period', sprintf('%d', arr)}];
S = [strcat({[tim '.']}, nv(:, 1)), nv(:, 2)];
S = [S; {[tim '.IPParameters'], strjoin(strrep(nv(:, 1)', '\ ', ' '), ',')}];
for c = 1:size(pins, 1)
    sname = sprintf('S_%s_CH%d', tim, c);
    S = [S; local_pin(pins{c, 1}, sname, '', pins{c, 2}); ...
            {sprintf('SH.%s.0', sname), sprintf('%s_CH%d,PWM Generation%d CH%d', tim, c, c, c); ...
             sprintf('SH.%s.ConfNb', sname), '1'}]; %#ok<AGROW>
end
S = [S; {sprintf('VP_%s_VS_ClockSourceINT.Mode', tim), 'Internal'; ...
         sprintf('VP_%s_VS_ClockSourceINT.Signal', tim), sprintf('%s_VS_ClockSourceINT', tim)}];
end

function [psc, arr] = local_tim(fclk, f, bits)
% smallest prescaler with an exact period that fits the counter width
N = fclk / f;
assert(abs(N - round(N)) < 1e-9, 'timer frequency %g Hz is not an integer divisor of %g Hz', f, fclk);
N = round(N);
for psc = 0:65535
    if mod(N, psc + 1) == 0 && N / (psc + 1) <= 2^bits
        arr = N / (psc + 1) - 1;
        return
    end
end
error('stm32_ioc_configure:tim', 'no prescaler for %g Hz', f);
end

function t = local_bittime(fk, rate, sp, phase)
% FDCAN bit timing: the smallest prescaler with an integer number of time quanta per bit that fits
% the STM32H7 / G4 FDCAN register limits (RM0433 / RM0440), sample point as close as possible to sp
switch phase
    case 'nominal', lim = [512 256 128 128];   % prescaler, TSEG1, TSEG2, SJW
    case 'data',    lim = [32 32 16 16];
end
for p = 1:lim(1)
    N = fk / (p * rate);
    if abs(N - round(N)) > 1e-9, continue; end
    N = round(N);
    seg1 = round(sp * N) - 1;
    seg2 = N - 1 - seg1;
    if N >= 8 && seg1 >= 1 && seg1 <= lim(2) && seg2 >= 1 && seg2 <= lim(3)
        t = struct('presc', p, 'seg1', seg1, 'seg2', seg2, 'sjw', min(seg2, lim(4)), 'tq', N, ...
                   'rate', fk / (p * N), 'sp', (1 + seg1) / N, 'fclk', fk);
        return
    end
end
error('stm32_ioc_configure:fdcan', 'no %s bit timing for %g bit/s from a %g Hz kernel clock', phase, rate, fk);
end

function ti = local_i2c(fk, tg)
% I2C timing register (RM0433 I2C_TIMINGR): PRESC, SCLDEL, SDADEL, SCLH, SCLL for Fast mode
presc = round(tg.i2c_t_presc * fk) - 1;
tp = (presc + 1) / fk;
scll = ceil(tg.i2c_t_low / tp - 1e-9) - 1;
sclh = ceil(tg.i2c_t_high / tp - 1e-9) - 1;
scldel = ceil(tg.i2c_t_scldel / tp - 1e-9) - 1;
sdadel = 0;
assert(presc <= 15 && scll <= 255 && sclh <= 255 && scldel <= 15, 'I2C timing out of register range');
timing = presc * 2^28 + scldel * 2^20 + sdadel * 2^16 + sclh * 2^8 + scll;
tl = (scll + 1) * tp; th = (sclh + 1) * tp;
ti = struct('clk', fk, 'presc', presc, 'scll', scll, 'sclh', sclh, 'scldel', scldel, 'sdadel', sdadel, ...
            'timing', timing, 't_low', tl, 't_high', th, 'f_scl', 1 ./ (tl + th + tg.i2c_t_sync));
end

function n = local_bitnames(s, nb, pre)
% names of the bits 0..nb-1 from a struct of 0-based bit indices (P.electronics.di / .do); NOZZLE_k and
% SPARE for the bits without a field (ICD 6: DO b10..b17 NOZZLE_1..8, b19 spare)
n = repmat({''}, 1, nb);
f = fieldnames(s);
for j = 1:numel(f)
    b = s.(f{j});
    if b < nb, n{b+1} = upper(f{j}); end
end
for b = 0:nb-1
    if isempty(n{b+1})
        if strcmp(pre, 'DO') && b >= 10 && b <= 17
            n{b+1} = sprintf('NOZZLE%d', b - 9);
        else
            n{b+1} = 'SPARE';
        end
    end
end
end

function s = local_funclist(fl)
c = cell(1, size(fl, 1));
for j = 1:size(fl, 1)
    c{j} = sprintf('%d-%s-%s-false-%s-false', j, fl{j, 1}, fl{j, 2}, fl{j, 3});
end
s = strjoin(c, ',');
end

function s = local_num(x)
% CubeMX writes doubles with Java Double.toString (integral values with '.0')
if abs(x - round(x)) < 1e-9
    s = sprintf('%d.0', round(x));
else
    s = sprintf('%.12g', x);
end
end
