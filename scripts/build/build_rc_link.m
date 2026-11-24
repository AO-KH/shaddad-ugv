function build_rc_link(what)
%BUILD_RC_LINK  Programmatic build of the RC link simulation (build plan T07; owner electronics / rc).
%
%   build_rc_link()          build every T07 model file:
%                              subsystems/rc/rc_transmitter.slx    sticks / switches -> raw[16]      Ts frame
%                              subsystems/rc/rc_radio_channel.slx  random loss, link loss, TX off     Ts frame
%                              subsystems/rc/rc_receiver.slx       hold, loss timer, failsafe (D59)   Ts frame
%                              subsystems/rc/rc_sbus_encoder.slx   SBUS bytes, frame -> 1 ms RT       frame / 1 ms
%                              subsystems/rc/rc_uart_line_sim.slx  byte timing (MATLAB Function)      1 ms
%                              subsystems/rc/RC_Link_Sim.slx       top (ICD 3.1 Operator_RC)          0.01 in / 0.001 out
%                              tests/rc/tb_rc_link.slx             bench: root op / flt -> RC_Link_Sim
%                              tests/rc/tb_rc_sbus_encoder.slx     bench: random Bus_RcFrame -> encoder
%                              tests/rc/tb_rc_link_speed.slx       bench: constant inputs (real-time factor)
%                              tests/rc/tb_rc_uart_line.slx        bench: rc_uart_line_sim.slx unit
%   build_rc_link('ssr')     the six subsystem files only
%   build_rc_link('bench')   the four benches only
%
%   Idempotent (build plan rule 3): every file is closed, deleted and recreated with new_system /
%   add_block / add_line / set_param and saved through shaddad_bu('save'), which lays out every level of
%   the file (shaddad_layout); nothing is hand-edited. Every numeric block
%   value comes from P.rc, P.electronics.sbus, P.ic or Ts (structural constants such as bit widths,
%   0/1 bounds and index offsets excepted). Needs P, Ts and the Bus_* objects in the base workspace
%   (shaddad_params); only P.rc is refreshed from params/param_rc.m when its T07 fields are missing.
%
%   Rate chain (D32): op / flt 0.01 -> RT -> 0.001 -> RT -> P.rc.frame_period; frame-rate units;
%   frame -> 0.001 deterministic RT inside rc_sbus_encoder (= the one-frame radio latency, D37) and
%   for the rc_frame logging copy. Every Rate Transition: Integrity on, Deterministic on.
%
%   RC_Link_Sim hierarchy: RC_Transmitter, RC_Radio_Channel, RC_Receiver, SBUS_Encoder are Subsystem
%   References to their files; UART_Line_Sim is the rc_uart_line_sim contents built inline by the same
%   builder function (local_uart_body), because a MATLAB Function block two Subsystem Reference levels
%   deep (model -> RC_Link_Sim -> rc_uart_line_sim -> MATLAB Function) fails the first compile of each
%   model in R2026b with the Stateflow JIT assertion 'o != nullptr' (relinkToExistingJITEngine).
%   The MATLAB Function has no parameter data (constants arrive on an input) for the same reason.
%
%   Buses (model layout spec 6.2, 6.3), without bus objects: RC_Transmitter -> RC_Radio_Channel (tx: ch_tx, dig_tx,
%   tx_on) and RC_Radio_Channel -> RC_Receiver (rx: ch_rx, dig_rx, rx_ok) are virtual buses between Out and In Bus
%   Element ports. The op bus of rc_transmitter and the flt bus of rc_uart_line_sim (and of its inline copy) are
%   In Bus Element ports, picked right in front of each consumer. RC_Link_Sim (ICD 3.1) and rc_sbus_encoder keep
%   classic bus ports, with one Bus Selector right in front of each consumer. Every signal line is named (L5).

if nargin < 1, what = 'all'; end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
local_env();
rcdir = fullfile(root, 'subsystems', 'rc');
tdir = fullfile(root, 'tests', 'rc');
if any(strcmpi(what, {'all', 'ssr'}))
    local_close_users();
    local_transmitter(fullfile(rcdir, 'rc_transmitter.slx'));
    local_radio(fullfile(rcdir, 'rc_radio_channel.slx'));
    local_receiver(fullfile(rcdir, 'rc_receiver.slx'));
    local_encoder(fullfile(rcdir, 'rc_sbus_encoder.slx'));
    local_uart(fullfile(rcdir, 'rc_uart_line_sim.slx'));
    local_top(fullfile(rcdir, 'RC_Link_Sim.slx'));
end
if any(strcmpi(what, {'all', 'bench'}))
    local_tb_link(fullfile(tdir, 'tb_rc_link.slx'));
    local_tb_encoder(fullfile(tdir, 'tb_rc_sbus_encoder.slx'));
    local_tb_speed(fullfile(tdir, 'tb_rc_link_speed.slx'));
    local_tb_uart(fullfile(tdir, 'tb_rc_uart_line.slx'));
end
% close the subsystem files this build loaded (benches are closed by their builders)
for n = {'RC_Link_Sim', 'rc_transmitter', 'rc_radio_channel', 'rc_receiver', 'rc_sbus_encoder', 'rc_uart_line_sim'}
    if bdIsLoaded(n{1}) && strcmp(get_param(n{1}, 'Dirty'), 'off') && ~local_in_use(n{1})
        close_system(n{1}, 0);
    end
end
end

% ===========================================================================================
function local_env()
if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_RcFrame'',''var'')')
    shaddad_params();
end
if ~evalin('base', 'isfield(P.rc, ''tx_perm'') && isfield(P.rc, ''seed_drop'') && isfield(P.rc, ''rx_fs_flags'')')
    evalin('base', 'P.rc = param_rc();');      % only the RC sub-struct (owner T07) is refreshed
end
end

function local_close_users()
% the T07 benches and RC_Link_Sim reference the subsystem files; close them (owner T07, parents
% first) so the files can be rebuilt. A file still referenced by a non-T07 model stays loaded and
% local_fresh reports it.
for n = {'tb_rc_link', 'tb_rc_sbus_encoder', 'tb_rc_link_speed', 'tb_rc_uart_line'}
    if bdIsLoaded(n{1}), close_system(n{1}, 0); end
end
for n = {'RC_Link_Sim', 'rc_transmitter', 'rc_radio_channel', 'rc_receiver', 'rc_sbus_encoder', 'rc_uart_line_sim'}
    if bdIsLoaded(n{1}) && ~local_in_use(n{1}), close_system(n{1}, 0); end
end
end

function tf = local_in_use(name)
% true when another loaded block diagram references the subsystem file 'name'
tf = false;
bds = find_system('SearchDepth', 0, 'type', 'block_diagram');
for k = 1:numel(bds)
    if strcmp(bds{k}, name) || ~any(strcmp(get_param(bds{k}, 'BlockDiagramType'), {'model', 'subsystem'}))
        continue                                   % libraries cannot hold a Subsystem Reference instance
    end
    try
        r = find_system(bds{k}, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants, ...
            'BlockType', 'SubSystem', 'ReferencedSubsystem', name);
    catch %#ok<CTCH> libraries / models that cannot be searched
        r = {};
    end
    if ~isempty(r), tf = true; return, end
end
end

function local_fresh(name, file, kind)
if bdIsLoaded(name)
    if local_in_use(name)
        error('build_rc_link:inuse', ['%s is referenced by another loaded model; close that model ' ...
            '(e.g. Shaddad_System) before rebuilding the RC subsystem files'], name);
    end
    close_system(name, 0);
end
if isfile(file), delete(file); end
if strcmpi(kind, 'ssr'), new_system(name, 'Subsystem'); else, new_system(name); end
end

function p = B(sys, lib, name, varargin)
p = shaddad_bu('add', lib, [sys '/' name], varargin{:});
end

function L(sys, src, dst, name)
% signal into dst ('Blk/k'), named name (layout spec 6.1, L5). src 'Blk/k': a line, without name the default of
% shaddad_bu('nline') (the name the source port's line already has, else the source block's name, '_k' for output k
% of a block with several outputs; a Bus Selector's output keeps '<element>'). src 'port.element': a pick of that
% element of the In Bus Element port right in front of dst (spec 6.2), its line named name (default the element)
if nargin < 4, name = ''; end
if contains(src, '/')
    shaddad_bu('nline', sys, src, dst, name);
else
    shaddad_bu('pick', sys, extractBefore(src, '.'), extractAfter(src, '.'), dst, name);
end
end

function OB(sys, port, el, src)
% the element el of the Out Bus Element port 'port' of sys, fed by src ('Blk/k'), its line named el (spec 6.3)
shaddad_bu('outbus', sys, port, el, src);
end

function IN(sys, name, num, type, dims, ts)
B(sys, 'simulink/Sources/In1', name, 'Port', num2str(num), 'OutDataTypeStr', type, ...
    'PortDimensions', dims, 'SampleTime', ts);
end

function OUT(sys, name, num, type, dims, ts)
B(sys, 'simulink/Sinks/Out1', name, 'Port', num2str(num), 'OutDataTypeStr', type, ...
    'PortDimensions', dims, 'SampleTime', ts);
end

function NOT(sys, name)
B(sys, 'simulink/Logic and Bit Operations/Logical Operator', name, 'Operator', 'NOT', 'Inputs', '1');
end

function local_save(name, file, desc)
set_param(name, 'Description', desc);
shaddad_bu('save', name, file);
end

function local_finalize(name, file)
% update diagram once after the build (MATLAB Function JIT artefacts, SSR caches) and save again when
% the update marked the bench dirty. A first update right after rebuilding the MATLAB Function can hit
% a transient Stateflow JIT relink assertion in a long-running session; it is retried once.
for k = 1:2
    try
        evalc('set_param(name, ''SimulationCommand'', ''update'')');
        break
    catch err
        if k == 2
            warning('build_rc_link:update', '%s: update after build failed (model saved): %s', name, err.message);
            return
        end
    end
end
if strcmp(get_param(name, 'Dirty'), 'on'), save_system(name, file); end
end

% ===========================================================================================
function local_transmitter(f)
% Stick / switch intent -> 16 raw SBUS channels at the frame period (build plan T07.3). In Bus Element port op
% (Bus_OperatorCmd, its rate from the parent's Rate Transition), picked right in front of each consumer; the outputs
% leave as the virtual bus tx (Out Bus Element, layout spec 6.2, 6.3).
n = 'rc_transmitter';
local_fresh(n, f, 'ssr');
FR = 'Ts.rc_frame';
sticks = {'stick_steer', 'stick_thr', 'stick_aux_lv', 'stick_aux_lh', 'knob_speed'};   % P.rc order
sw3 = {'sw_mode', 'sw_impl', 'sw_lift', 'sw_mission'};                                 % 3-position
sw2 = {'sw_arm', 'btn_reset', 'sw_lights', 'sw_estop', 'btn_rth'};                     % 2-position
% sticks: raw = sat(raw_mid + round(raw_half*x), raw_min, raw_max) (rounding about the centre keeps
% the TX endpoints symmetric: +1 -> 1811.5 -> 1812 -> 1811, -1 -> 172, 0 -> 992)
B(n, 'simulink/Signal Routing/Mux', 'Mux_sticks', 'Inputs', '5');
for k = 1:5, L(n, ['op.' sticks{k}], sprintf('Mux_sticks/%d', k)); end
B(n, 'simulink/Math Operations/Gain', 'Stick_Gain', 'Gain', 'P.rc.raw_half');
B(n, 'simulink/Math Operations/Rounding Function', 'Stick_Round', 'Operator', 'round');
B(n, 'simulink/Math Operations/Bias', 'Stick_Bias', 'Bias', 'P.rc.raw_mid');
B(n, 'simulink/Discontinuities/Saturation', 'Stick_Sat', 'UpperLimit', 'P.rc.raw_max', 'LowerLimit', 'P.rc.raw_min');
L(n, 'Mux_sticks/1', 'Stick_Gain/1', 'stick_x');
L(n, 'Stick_Gain/1', 'Stick_Round/1');
L(n, 'Stick_Round/1', 'Stick_Bias/1');
L(n, 'Stick_Bias/1', 'Stick_Sat/1');
% 3-position switches: position 1/2/3 (0 treated as 1) -> index 0..2 -> P.rc.raw_sw3
B(n, 'simulink/Signal Routing/Mux', 'Mux_sw3', 'Inputs', '4');
for k = 1:4, L(n, ['op.' sw3{k}], sprintf('Mux_sw3/%d', k)); end
B(n, 'simulink/Math Operations/Rounding Function', 'Sw3_Round', 'Operator', 'round');
B(n, 'simulink/Math Operations/Bias', 'Sw3_Index', 'Bias', '-1');
B(n, 'simulink/Discontinuities/Saturation', 'Sw3_Sat', 'UpperLimit', 'numel(P.rc.raw_sw3)-1', 'LowerLimit', '0');
B(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'Sw3_Table', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.rc.raw_sw3', 'DiagnosticForOutOfRangeInput', 'Error');
L(n, 'Mux_sw3/1', 'Sw3_Round/1', 'sw3_pos');
L(n, 'Sw3_Round/1', 'Sw3_Index/1');
L(n, 'Sw3_Index/1', 'Sw3_Sat/1');
L(n, 'Sw3_Sat/1', 'Sw3_Table/1', 'sw3_idx');
% 2-position switches / momentary buttons: 0/1 -> index 0/1 -> P.rc.raw_sw2
B(n, 'simulink/Signal Routing/Mux', 'Mux_sw2', 'Inputs', '5');
for k = 1:5, L(n, ['op.' sw2{k}], sprintf('Mux_sw2/%d', k)); end
B(n, 'simulink/Math Operations/Rounding Function', 'Sw2_Round', 'Operator', 'round');
B(n, 'simulink/Discontinuities/Saturation', 'Sw2_Sat', 'UpperLimit', 'numel(P.rc.raw_sw2)-1', 'LowerLimit', '0');
B(n, 'simulink/Lookup Tables/Direct Lookup Table (n-D)', 'Sw2_Table', 'NumberOfTableDimensions', '1', ...
    'Table', 'P.rc.raw_sw2', 'DiagnosticForOutOfRangeInput', 'Error');
L(n, 'Mux_sw2/1', 'Sw2_Round/1', 'sw2_pos');
L(n, 'Sw2_Round/1', 'Sw2_Sat/1');
L(n, 'Sw2_Sat/1', 'Sw2_Table/1', 'sw2_idx');
% spare channels, concatenation and channel map
B(n, 'simulink/Sources/Constant', 'Spare_raw', 'Value', 'P.rc.tx_spare_raw', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Vector Concatenate', 'Cat_raw', 'NumInputs', '4', 'Mode', 'Vector');
L(n, 'Stick_Sat/1', 'Cat_raw/1', 'raw_sticks');
L(n, 'Sw3_Table/1', 'Cat_raw/2', 'raw_sw3');
L(n, 'Sw2_Table/1', 'Cat_raw/3', 'raw_sw2');
L(n, 'Spare_raw/1', 'Cat_raw/4', 'raw_spare');
B(n, 'simulink/Signal Routing/Selector', 'Ch_Map', 'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', 'P.rc.tx_perm', 'InputPortWidth', 'P.rc.n_ch');
L(n, 'Cat_raw/1', 'Ch_Map/1', 'raw_cat');
B(n, 'simulink/Signal Attributes/Data Type Conversion', 'Ch_uint16', 'OutDataTypeStr', 'uint16', ...
    'RndMeth', 'Round', 'SaturateOnIntegerOverflow', 'on');
L(n, 'Ch_Map/1', 'Ch_uint16/1');
OB(n, 'tx', 'ch_tx', 'Ch_uint16/1');
% CH17 / CH18 (not mapped on the transmitter)
B(n, 'simulink/Sources/Constant', 'Ch_dig', 'Value', 'P.rc.tx_ch_dig', 'OutDataTypeStr', 'boolean', 'SampleTime', FR);
OB(n, 'tx', 'dig_tx', 'Ch_dig/1');
% transmitter powered
B(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Tx_On', 'relop', '~=', 'const', '0', ...
    'OutDataTypeStr', 'boolean');
L(n, 'op.tx_power_on', 'Tx_On/1');
OB(n, 'tx', 'tx_on', 'Tx_On/1');
local_save(n, f, ['T07 rc_transmitter (Ts P.rc.frame_period). in 1 op Bus_OperatorCmd (picks); out 1 tx, a virtual bus: ' ...
    'ch_tx uint16[16] (channel map P.rc.ch_map via P.rc.tx_perm), dig_tx boolean[2] (CH17/CH18, false), tx_on boolean.']);
end

% ===========================================================================================
function local_radio(f)
% Frame loss (seeded uniform draw < p_loss, scenario link loss, TX off) and extra latency (T07.4). The virtual bus tx
% comes in on an In Bus Element port, picked right in front of each consumer; rx leaves on an Out Bus Element port.
n = 'rc_radio_channel';
local_fresh(n, f, 'ssr');
FR = 'Ts.rc_frame';
IN(n, 'link_loss', 1, 'boolean', '1', FR);
B(n, 'simulink/Sources/Uniform Random Number', 'Loss_Rand', 'Minimum', '0', 'Maximum', '1', ...
    'Seed', 'P.rc.seed', 'SampleTime', FR);
B(n, 'simulink/Sources/Constant', 'P_loss', 'Value', 'P.rc.p_loss', 'SampleTime', FR);
B(n, 'simulink/Logic and Bit Operations/Relational Operator', 'Loss_Draw', 'Operator', '<');
L(n, 'Loss_Rand/1', 'Loss_Draw/1', 'u_loss');
L(n, 'P_loss/1', 'Loss_Draw/2');
NOT(n, 'Tx_Off');
L(n, 'tx.tx_on', 'Tx_Off/1');
B(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Lost_OR', 'Operator', 'OR', 'Inputs', '3');
L(n, 'Loss_Draw/1', 'Lost_OR/1', 'lost_random');
L(n, 'link_loss/1', 'Lost_OR/2', 'link_loss');
L(n, 'Tx_Off/1', 'Lost_OR/3', 'tx_off');
NOT(n, 'Rx_Good');
L(n, 'Lost_OR/1', 'Rx_Good/1', 'lost');
% extra radio latency (P.rc.extra_delay_frames = 0: the one-frame latency is the frame -> 1 ms RT, D37)
B(n, 'simulink/Discrete/Delay', 'Delay_ch', 'DelayLength', 'P.rc.extra_delay_frames', ...
    'InitialCondition', 'P.rc.failsafe_raw', 'SampleTime', '-1');
B(n, 'simulink/Discrete/Delay', 'Delay_dig', 'DelayLength', 'P.rc.extra_delay_frames', ...
    'InitialCondition', '0', 'SampleTime', '-1');
B(n, 'simulink/Discrete/Delay', 'Delay_ok', 'DelayLength', 'P.rc.extra_delay_frames', ...
    'InitialCondition', '0', 'SampleTime', '-1');
L(n, 'tx.ch_tx', 'Delay_ch/1');
L(n, 'tx.dig_tx', 'Delay_dig/1');
L(n, 'Rx_Good/1', 'Delay_ok/1', 'good');
OB(n, 'rx', 'ch_rx', 'Delay_ch/1');
OB(n, 'rx', 'dig_rx', 'Delay_dig/1');
OB(n, 'rx', 'rx_ok', 'Delay_ok/1');
shaddad_bu('portorder', n, {'tx', 'link_loss'}, 'Inport');
local_save(n, f, ['T07 rc_radio_channel (Ts P.rc.frame_period). in 1 tx, a virtual bus: ch_tx uint16[16], dig_tx boolean[2], ' ...
    'tx_on; 2 link_loss (flt.rc_link_loss); out 1 rx, a virtual bus: ch_rx, dig_rx, rx_ok = NOT(U(seed P.rc.seed) < P.rc.p_loss ' ...
    'OR link_loss OR NOT tx_on), delayed by P.rc.extra_delay_frames.']);
end

% ===========================================================================================
function local_receiver(f)
% Receiver failsafe from library blocks (D59): hold, continuous-loss timer, failsafe mode selection. The virtual bus rx
% comes in on an In Bus Element port, picked right in front of each consumer (layout spec 6.2).
n = 'rc_receiver';
local_fresh(n, f, 'ssr');
FR = 'Ts.rc_frame';
IN(n, 'line_silent', 1, 'boolean', '1', FR);
% hold last good channels
B(n, 'simulink/Signal Routing/Switch', 'Hold_ch', 'Criteria', 'u2 ~= 0');
B(n, 'simulink/Discrete/Unit Delay', 'Last_ch', 'InitialCondition', 'P.rc.failsafe_raw', 'SampleTime', FR);
L(n, 'rx.ch_rx', 'Hold_ch/1');
L(n, 'rx.rx_ok', 'Hold_ch/2');
L(n, 'Last_ch/1', 'Hold_ch/3', 'ch_last');
L(n, 'Hold_ch/1', 'Last_ch/1', 'ch_held');
B(n, 'simulink/Signal Routing/Switch', 'Hold_dig', 'Criteria', 'u2 ~= 0');
B(n, 'simulink/Discrete/Unit Delay', 'Last_dig', 'InitialCondition', '0', 'SampleTime', FR);
L(n, 'rx.dig_rx', 'Hold_dig/1');
L(n, 'rx.rx_ok', 'Hold_dig/2');
L(n, 'Last_dig/1', 'Hold_dig/3', 'dig_last');
L(n, 'Hold_dig/1', 'Last_dig/1', 'dig_held');
% continuous-loss timer = time since the last good frame (Backward Euler: T after the first lost frame),
% level reset on every good frame
B(n, 'simulink/Sources/Constant', 'Rate_One', 'Value', '1', 'SampleTime', FR);
B(n, 'simulink/Discrete/Discrete-Time Integrator', 'Loss_Timer', 'IntegratorMethod', 'Integration: Backward Euler', ...
    'gainval', '1', 'ExternalReset', 'level', 'InitialCondition', '0', 'SampleTime', FR, ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'P.rc.rx_timer_sat', 'LowerSaturationLimit', '0');
L(n, 'Rate_One/1', 'Loss_Timer/1');
L(n, 'rx.rx_ok', 'Loss_Timer/2');
B(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'FS_Active', 'relop', '>=', ...
    'const', 'P.rc.rx_failsafe_time', 'OutDataTypeStr', 'boolean');
L(n, 'Loss_Timer/1', 'FS_Active/1', 't_loss');
% failsafe output selection by P.rc.rx_failsafe_mode: 1 FLAG_PRESET, 2 NO_PULSES, 3 HOLD
B(n, 'simulink/Sources/Constant', 'FS_Mode', 'Value', 'P.rc.rx_failsafe_mode', 'SampleTime', FR);
B(n, 'simulink/Sources/Constant', 'FS_Preset', 'Value', 'P.rc.failsafe_raw', 'OutDataTypeStr', 'uint16', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Multiport Switch', 'FS_Ch_Sel', 'Inputs', '3', 'DataPortOrder', 'One-based contiguous', ...
    'DiagnosticForDefault', 'Error');
L(n, 'FS_Mode/1', 'FS_Ch_Sel/1', 'fs_mode');
L(n, 'FS_Preset/1', 'FS_Ch_Sel/2', 'ch_preset');
L(n, 'Hold_ch/1', 'FS_Ch_Sel/3');
L(n, 'Hold_ch/1', 'FS_Ch_Sel/4');
for k = 1:3
    B(n, 'simulink/Sources/Constant', sprintf('FS_Flags_%d', k), 'Value', sprintf('P.rc.rx_fs_flags(%d,:)', k), ...
        'OutDataTypeStr', 'boolean', 'SampleTime', FR);
end
B(n, 'simulink/Signal Routing/Multiport Switch', 'FS_Flag_Sel', 'Inputs', '3', 'DataPortOrder', 'One-based contiguous', ...
    'DiagnosticForDefault', 'Error');
L(n, 'FS_Mode/1', 'FS_Flag_Sel/1');
for k = 1:3, L(n, sprintf('FS_Flags_%d/1', k), sprintf('FS_Flag_Sel/%d', k + 1)); end
B(n, 'simulink/Signal Routing/Switch', 'Out_ch', 'Criteria', 'u2 ~= 0');
L(n, 'FS_Ch_Sel/1', 'Out_ch/1', 'ch_fs');
L(n, 'FS_Active/1', 'Out_ch/2', 'fs_active');
L(n, 'Hold_ch/1', 'Out_ch/3');
B(n, 'simulink/Sources/Constant', 'Flags_OK', 'Value', 'P.rc.rx_flags_ok', 'OutDataTypeStr', 'boolean', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Switch', 'Out_flags', 'Criteria', 'u2 ~= 0');
L(n, 'FS_Flag_Sel/1', 'Out_flags/1', 'flags_fs');
L(n, 'FS_Active/1', 'Out_flags/2');
L(n, 'Flags_OK/1', 'Out_flags/3', 'flags_ok');
B(n, 'simulink/Signal Routing/Demux', 'Flags_Demux', 'Outputs', '2');
L(n, 'Out_flags/1', 'Flags_Demux/1', 'flags_out');
% line silent -> nothing sent; frame_lost = no good frame this period
NOT(n, 'Not_Silent');
L(n, 'line_silent/1', 'Not_Silent/1', 'line_silent');
B(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Sent_AND', 'Operator', 'AND', 'Inputs', '2');
L(n, 'Flags_Demux/2', 'Sent_AND/1', 'sent_fs');
L(n, 'Not_Silent/1', 'Sent_AND/2');
NOT(n, 'Lost');
L(n, 'rx.rx_ok', 'Lost/1');
B(n, 'simulink/Signal Routing/Demux', 'Dig_Demux', 'Outputs', '2');
L(n, 'Hold_dig/1', 'Dig_Demux/1');
B(n, 'simulink/Signal Routing/Bus Creator', 'BC_frame', 'Inputs', '6', 'OutDataTypeStr', 'Bus: Bus_RcFrame', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Out_ch/1', 'BC_frame/1', 'ch_raw');
L(n, 'Dig_Demux/1', 'BC_frame/2', 'ch17');
L(n, 'Dig_Demux/2', 'BC_frame/3', 'ch18');
L(n, 'Lost/1', 'BC_frame/4', 'frame_lost');
L(n, 'Flags_Demux/1', 'BC_frame/5', 'failsafe');
L(n, 'Sent_AND/1', 'BC_frame/6', 'sent');
OUT(n, 'frame', 1, 'Bus: Bus_RcFrame', '-1', FR);
L(n, 'BC_frame/1', 'frame/1', 'frame');
shaddad_bu('portorder', n, {'rx', 'line_silent'}, 'Inport');
local_save(n, f, ['T07 rc_receiver (Ts P.rc.frame_period, blocks D59). in 1 rx, a virtual bus: ch_rx uint16[16], dig_rx boolean[2], ' ...
    'rx_ok; 2 line_silent (flt.rc_line_silent); out 1 frame Bus_RcFrame. Holds the last good channels; ' ...
    'frame_lost = NOT rx_ok; after P.rc.rx_failsafe_time of continuous loss: mode 1 FLAG_PRESET (failsafe = 1, ' ...
    'P.rc.failsafe_raw), 2 NO_PULSES (sent = 0), 3 HOLD (lost bits only); line_silent -> sent = 0.']);
end

% ===========================================================================================
function local_encoder(f)
% SBUS frame bytes from library blocks (T07.6) + frame -> 1 ms deterministic RT and frame_new. The classic bus port
% frame feeds one Bus Selector right in front of each consumer (layout spec 6.2).
n = 'rc_sbus_encoder';
local_fresh(n, f, 'ssr');
FR = 'Ts.rc_frame';
IN(n, 'frame', 1, 'Bus: Bus_RcFrame', '-1', FR);
B(n, 'simulink/Signal Routing/Bus Selector', 'BS_ch', 'OutputSignals', 'ch_raw');
L(n, 'frame/1', 'BS_ch/1', 'frame');
B(n, 'commutil2/Integer to Bit Converter', 'Ch_Bits', 'nbits', 'P.electronics.sbus.ch_bits', ...
    'signedInputValues', 'Unsigned', 'bitOrder', 'LSB first', 'outDtype', 'boolean');
B(n, 'commutil2/Bit to Integer Converter', 'Payload_Bytes', 'nbits', '8', 'bitOrder', 'LSB first', ...
    'signedOutputValues', 'Unsigned', 'outDtype', 'uint8');
L(n, 'BS_ch/1', 'Ch_Bits/1');
L(n, 'Ch_Bits/1', 'Payload_Bytes/1', 'payload_bits');
% flags byte [ch17 ch18 frame_lost failsafe 0 0 0 0], LSB first
B(n, 'simulink/Sources/Constant', 'Flag_Reserved', 'Value', 'false(1,4)', 'OutDataTypeStr', 'boolean', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Vector Concatenate', 'Flag_Bits', 'NumInputs', '5', 'Mode', 'Vector');
B(n, 'simulink/Signal Routing/Bus Selector', 'BS_flags', 'OutputSignals', 'ch17,ch18,frame_lost,failsafe');
L(n, 'frame/1', 'BS_flags/1');
for k = 1:4, L(n, sprintf('BS_flags/%d', k), sprintf('Flag_Bits/%d', k)); end
L(n, 'Flag_Reserved/1', 'Flag_Bits/5');
B(n, 'commutil2/Bit to Integer Converter', 'Flag_Byte', 'nbits', '8', 'bitOrder', 'LSB first', ...
    'signedOutputValues', 'Unsigned', 'outDtype', 'uint8');
L(n, 'Flag_Bits/1', 'Flag_Byte/1', 'flag_bits');
% [0x0F; payload(22); flags; 0x00]
B(n, 'simulink/Sources/Constant', 'Header', 'Value', 'P.electronics.sbus.header', 'OutDataTypeStr', 'uint8', 'SampleTime', FR);
B(n, 'simulink/Sources/Constant', 'Footer', 'Value', 'P.electronics.sbus.footer', 'OutDataTypeStr', 'uint8', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Vector Concatenate', 'Frame_Cat', 'NumInputs', '4', 'Mode', 'Vector');
L(n, 'Header/1', 'Frame_Cat/1', 'header');
L(n, 'Payload_Bytes/1', 'Frame_Cat/2', 'payload');
L(n, 'Flag_Byte/1', 'Frame_Cat/3', 'flags');
L(n, 'Footer/1', 'Frame_Cat/4', 'footer');
% frame period -> 1 ms (deterministic: one-frame radio latency, D37)
B(n, 'simulink/Signal Attributes/Rate Transition', 'RT_bytes', 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTime', 'Ts.base', 'InitialCondition', '0');
L(n, 'Frame_Cat/1', 'RT_bytes/1', 'frame_bytes_fr');
OUT(n, 'frame_bytes', 1, 'uint8', 'P.electronics.sbus.frame_len', 'Ts.base');
L(n, 'RT_bytes/1', 'frame_bytes/1', 'frame_bytes');
% frame counter (counts sent frames, wraps) -> RT -> Detect Change = frame_new at 1 ms
B(n, 'simulink/Signal Attributes/Data Type Conversion', 'Sent_u32', 'OutDataTypeStr', 'uint32');
B(n, 'simulink/Signal Routing/Bus Selector', 'BS_sent', 'OutputSignals', 'sent');
L(n, 'frame/1', 'BS_sent/1');
L(n, 'BS_sent/1', 'Sent_u32/1');
B(n, 'simulink/Math Operations/Sum', 'Cnt_Sum', 'Inputs', '++', 'OutDataTypeStr', 'uint32', ...
    'SaturateOnIntegerOverflow', 'off');
B(n, 'simulink/Discrete/Unit Delay', 'Cnt_Last', 'InitialCondition', '0', 'SampleTime', FR);
L(n, 'Sent_u32/1', 'Cnt_Sum/1', 'sent_u32');
L(n, 'Cnt_Last/1', 'Cnt_Sum/2', 'cnt_last');
L(n, 'Cnt_Sum/1', 'Cnt_Last/1', 'frame_cnt');
B(n, 'simulink/Signal Attributes/Rate Transition', 'RT_cnt', 'Integrity', 'on', 'Deterministic', 'on', ...
    'OutPortSampleTime', 'Ts.base', 'InitialCondition', '0');
L(n, 'Cnt_Sum/1', 'RT_cnt/1');
B(n, 'simulink/Logic and Bit Operations/Detect Change', 'Frame_New', 'vinit', '0');
L(n, 'RT_cnt/1', 'Frame_New/1', 'frame_cnt_1k');
OUT(n, 'frame_new', 2, 'boolean', '1', 'Ts.base');
L(n, 'Frame_New/1', 'frame_new/1', 'frame_new');
local_save(n, f, ['T07 rc_sbus_encoder. in 1 frame Bus_RcFrame (Ts P.rc.frame_period); out 1 frame_bytes uint8[25] ' ...
    '(0.001, [0x0F payload(22) flags 0x00], one frame late = radio latency D37), 2 frame_new boolean (0.001). ' ...
    'Integer to Bit (11, LSB first) -> Bit to Integer (8, LSB first); flags [ch17 ch18 frame_lost failsafe 0 0 0 0].']);
end

% ===========================================================================================
function local_uart(f)
% UART byte timing at 1 ms: MATLAB Function sbus_uart_line_sim (spec 4.2) + line-silent gate (T07.7).
n = 'rc_uart_line_sim';
local_fresh(n, f, 'ssr');
local_uart_body(n);
local_save(n, f, ['T07 rc_uart_line_sim (0.001). in 1 frame_bytes uint8[25], 2 frame_new, 3 flt Bus_FaultInject (picks); ' ...
    'out 1 sbus Bus_SbusStream. MATLAB Function sbus_uart_line_sim (P.rc.uart, error probability ' ...
    'max(P.rc.uart.p_err, flt.rc_byte_err_rate)); flt.rc_line_silent forces len = 0 at the same 1 ms tick. ' ...
    'RC_Link_Sim/UART_Line_Sim is built from the same builder function (local_uart_body) as a plain subsystem.']);
end

function local_uart_body(n)
% contents of rc_uart_line_sim (also built inline as RC_Link_Sim/UART_Line_Sim, see local_top). Port 3 flt is an In Bus
% Element port, picked right in front of each consumer (layout spec 6.2); ports 1 and 2 are classic.
BT = 'Ts.base';
IN(n, 'frame_bytes', 1, 'uint8', 'P.electronics.sbus.frame_len', BT);
IN(n, 'frame_new', 2, 'boolean', '1', BT);
B(n, 'simulink/Sources/Constant', 'P_err_nom', 'Value', 'P.rc.uart.p_err', 'SampleTime', BT);
B(n, 'simulink/Math Operations/MinMax', 'P_err', 'Function', 'max', 'Inputs', '2');
L(n, 'P_err_nom/1', 'P_err/1');
L(n, 'flt.rc_byte_err_rate', 'P_err/2');
B(n, 'simulink/Sources/Digital Clock', 'Clock', 'SampleTime', BT);
B(n, 'simulink/Sources/Uniform Random Number', 'Rand_err', 'Minimum', 'zeros(1,P.electronics.sbus.frame_len)', ...
    'Maximum', 'ones(1,P.electronics.sbus.frame_len)', 'Seed', 'P.rc.seed_err', 'SampleTime', BT);
B(n, 'simulink/Sources/Uniform Random Number', 'Rand_drop', 'Minimum', 'zeros(1,P.electronics.sbus.frame_len)', ...
    'Maximum', 'ones(1,P.electronics.sbus.frame_len)', 'Seed', 'P.rc.seed_drop', 'SampleTime', BT);
% UART_Active: Enabled Subsystem around the MATLAB Function, enabled by frame_new OR pending(k-1)
% (a frame is on the line); disabled outputs reset to 0 = the function's own idle output, so the
% byte stream is identical while the function call cost is paid only on ~5 of every 14 ticks
ss = B(n, 'built-in/SubSystem', 'UART_Active');
B(ss, 'simulink/Ports & Subsystems/Enable', 'Enable', 'StatesWhenEnabling', 'held');
inNames = {'frame', 'frame_new', 't', 'r_err', 'r_drop', 'p_err', 'line_cfg'};
for k = 1:numel(inNames)
    B(ss, 'simulink/Sources/In1', inNames{k}, 'Port', num2str(k));
end
outNames = {'bytes', 'len', 'uart_err', 'pending'};
for k = 1:numel(outNames)
    B(ss, 'simulink/Sinks/Out1', outNames{k}, 'Port', num2str(k), 'OutputWhenDisabled', 'reset', 'InitialOutput', '0');
end
% MATLAB Function (the only one in RC_Link_Sim): body in subsystems/rc/sbus_uart_line_sim.m, called in
% one line. The line constants (P.rc.uart.Tb / phase0 / drift_ppm, P.rc.frame_period, P.rc.uart.p_drop)
% arrive as one constant input vector: MATLAB Function parameter data (mask or workspace resolved)
% hit a Stateflow JIT relink assertion ('o != nullptr', cdr_get_param_props_for_jit_registration) on
% the first compile after the bench is reloaded in R2026b, so the block has no parameter data at all.
mf = B(ss, 'simulink/User-Defined Functions/MATLAB Function', 'UART_Line');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = sprintf(['function [bytes, len, uart_err, pending] = fcn(frame, frame_new, t, r_err, r_drop, p_err, c)\n' ...
    '%%#codegen\n' ...
    '%% c = [Tb phase0 drift_ppm Tf p_drop] (Constant Line_Cfg)\n' ...
    '[bytes, len, uart_err, pending] = sbus_uart_line_sim(frame, frame_new, t, r_err, r_drop, c(1), c(2), c(3), c(4), p_err, c(5));\n']);
cfg.UpdateMethod = 'Inherited';
cfg.VectorOutputs1D = true;
cfg.SaturateOnIntegerOverflow = false;
cfg.Description = 'SBUS UART byte timing (docs/specs/electronics.md 4.2), simulation only';
for k = 1:numel(inNames), L(ss, [inNames{k} '/1'], sprintf('UART_Line/%d', k), inNames{k}); end
for k = 1:numel(outNames), L(ss, sprintf('UART_Line/%d', k), [outNames{k} '/1'], outNames{k}); end
set_param(ss, 'Description', ['Runs sbus_uart_line_sim while a frame is on the line (frame_new OR pending ' ...
    'of the previous tick); disabled outputs reset to 0 (no bytes).']);
L(n, 'frame_bytes/1', 'UART_Active/1', 'frame_bytes');
L(n, 'frame_new/1', 'UART_Active/2', 'frame_new');
L(n, 'Clock/1', 'UART_Active/3', 't');
L(n, 'Rand_err/1', 'UART_Active/4', 'r_err');
L(n, 'Rand_drop/1', 'UART_Active/5', 'r_drop');
L(n, 'P_err/1', 'UART_Active/6', 'p_err');
B(n, 'simulink/Sources/Constant', 'Line_Cfg', 'Value', ...
    '[P.rc.uart.Tb, P.rc.uart.phase0, P.rc.uart.drift_ppm, P.rc.frame_period, P.rc.uart.p_drop]', 'SampleTime', BT);
L(n, 'Line_Cfg/1', 'UART_Active/7', 'line_cfg');
B(n, 'simulink/Discrete/Unit Delay', 'Pending_Last', 'InitialCondition', '0', 'SampleTime', BT);
L(n, 'UART_Active/4', 'Pending_Last/1', 'pending');
B(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Line_Busy', 'Operator', 'OR', 'Inputs', '2');
L(n, 'frame_new/1', 'Line_Busy/1');
L(n, 'Pending_Last/1', 'Line_Busy/2', 'pending_last');
phS = get_param(ss, 'PortHandles');
phB = get_param([n '/Line_Busy'], 'PortHandles');
hl = add_line(n, phB.Outport(1), phS.Enable(1), 'autorouting', 'smart');
set_param(hl, 'Name', 'line_busy');
% receiver output silent (flt.rc_line_silent at 1 ms): no bytes, no error flag
B(n, 'simulink/Sources/Constant', 'No_bytes', 'Value', 'zeros(1,P.electronics.sbus.read_max)', ...
    'OutDataTypeStr', 'uint8', 'SampleTime', BT);
B(n, 'simulink/Sources/Constant', 'No_len', 'Value', '0', 'OutDataTypeStr', 'uint8', 'SampleTime', BT);
B(n, 'simulink/Signal Routing/Switch', 'Gate_bytes', 'Criteria', 'u2 ~= 0');
B(n, 'simulink/Signal Routing/Switch', 'Gate_len', 'Criteria', 'u2 ~= 0');
L(n, 'No_bytes/1', 'Gate_bytes/1');
L(n, 'flt.rc_line_silent', 'Gate_bytes/2');
L(n, 'UART_Active/1', 'Gate_bytes/3', 'bytes_line');
L(n, 'No_len/1', 'Gate_len/1');
L(n, 'flt.rc_line_silent', 'Gate_len/2');
L(n, 'UART_Active/2', 'Gate_len/3', 'len_line');
NOT(n, 'Not_Silent');
L(n, 'flt.rc_line_silent', 'Not_Silent/1');
B(n, 'simulink/Logic and Bit Operations/Logical Operator', 'Gate_err', 'Operator', 'AND', 'Inputs', '2');
L(n, 'UART_Active/3', 'Gate_err/1', 'uart_err_line');
L(n, 'Not_Silent/1', 'Gate_err/2');
B(n, 'simulink/Signal Routing/Bus Creator', 'BC_sbus', 'Inputs', '3', 'OutDataTypeStr', 'Bus: Bus_SbusStream', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Gate_bytes/1', 'BC_sbus/1', 'bytes');
L(n, 'Gate_len/1', 'BC_sbus/2', 'len');
L(n, 'Gate_err/1', 'BC_sbus/3', 'uart_err');
OUT(n, 'sbus', 1, 'Bus: Bus_SbusStream', '-1', BT);
L(n, 'BC_sbus/1', 'sbus/1', 'sbus');
shaddad_bu('portorder', n, {'frame_bytes', 'frame_new', 'flt'}, 'Inport');
end

% ===========================================================================================
function local_top(f)
% RC_Link_Sim (ICD 3.1 Operator_RC): ports kept from the T01 stub.
n = 'RC_Link_Sim';
local_fresh(n, f, 'ssr');
B(n, 'simulink/Sources/In1', 'op', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_OperatorCmd', 'SampleTime', 'Ts.ctrl');
B(n, 'simulink/Sources/In1', 'flt', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_FaultInject', 'SampleTime', 'Ts.ctrl');
% rate chain D32: 0.01 -> 0.001 -> frame period (integer ratios 10 and 14 / 7)
RT = 'simulink/Signal Attributes/Rate Transition';
B(n, RT, 'RT_op_1k', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.base', ...
    'InitialCondition', 'P.ic.Bus_OperatorCmd');
B(n, RT, 'RT_op_frame', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.rc_frame', ...
    'InitialCondition', 'P.ic.Bus_OperatorCmd');
B(n, RT, 'RT_flt_1k', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.base', ...
    'InitialCondition', 'P.ic.Bus_FaultInject');
B(n, RT, 'RT_flt_frame', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.rc_frame', ...
    'InitialCondition', 'P.ic.Bus_FaultInject');
L(n, 'op/1', 'RT_op_1k/1', 'op');
L(n, 'RT_op_1k/1', 'RT_op_frame/1', 'op_1k');
L(n, 'flt/1', 'RT_flt_1k/1', 'flt');
L(n, 'RT_flt_1k/1', 'RT_flt_frame/1', 'flt_1k');
% the classic flt path feeds one Bus Selector right in front of each consumer (layout spec 6.2)
B(n, 'simulink/Signal Routing/Bus Selector', 'BS_flt_radio', 'OutputSignals', 'rc_link_loss');
B(n, 'simulink/Signal Routing/Bus Selector', 'BS_flt_receiver', 'OutputSignals', 'rc_line_silent');
L(n, 'RT_flt_frame/1', 'BS_flt_radio/1', 'flt_frame');
L(n, 'RT_flt_frame/1', 'BS_flt_receiver/1');
SSR = 'simulink/Ports & Subsystems/Subsystem Reference';
B(n, SSR, 'RC_Transmitter', 'ReferencedSubsystem', 'rc_transmitter');
B(n, SSR, 'RC_Radio_Channel', 'ReferencedSubsystem', 'rc_radio_channel');
B(n, SSR, 'RC_Receiver', 'ReferencedSubsystem', 'rc_receiver');
B(n, SSR, 'SBUS_Encoder', 'ReferencedSubsystem', 'rc_sbus_encoder');
% UART_Line_Sim: the rc_uart_line_sim contents built inline (same builder function) as a plain
% subsystem, NOT as a Subsystem Reference: in R2026b a MATLAB Function block inside a Subsystem
% Reference that is itself referenced from a Subsystem Reference (model -> RC_Link_Sim ->
% rc_uart_line_sim -> MATLAB Function) fails the first compile of every model that uses it with the
% Stateflow JIT assertion 'o != nullptr' (Stateflow.JIT.Utils.relinkToExistingJITEngine); one level
% of Subsystem Reference (model -> RC_Link_Sim -> MATLAB Function) compiles cleanly.
us = B(n, 'built-in/SubSystem', 'UART_Line_Sim');
local_uart_body(us);
set_param(us, 'Description', ['Contents of subsystems/rc/rc_uart_line_sim.slx, generated by build_rc_link>local_uart_body ' ...
    '(inline instead of a nested Subsystem Reference: R2026b Stateflow JIT assertion with a MATLAB Function two ' ...
    'Subsystem Reference levels deep).']);
L(n, 'RT_op_frame/1', 'RC_Transmitter/1', 'op_frame');
L(n, 'RC_Transmitter/1', 'RC_Radio_Channel/1', 'tx');
L(n, 'BS_flt_radio/1', 'RC_Radio_Channel/2');
L(n, 'RC_Radio_Channel/1', 'RC_Receiver/1', 'rx');
L(n, 'BS_flt_receiver/1', 'RC_Receiver/2');
L(n, 'RC_Receiver/1', 'SBUS_Encoder/1', 'rc_frame_fr');
L(n, 'SBUS_Encoder/1', 'UART_Line_Sim/1', 'frame_bytes');
L(n, 'SBUS_Encoder/2', 'UART_Line_Sim/2', 'frame_new');
L(n, 'RT_flt_1k/1', 'UART_Line_Sim/3');
% logging copy of the receiver frame at 0.001 (deterministic slow -> fast: aligned with the SBUS bytes)
B(n, RT, 'RT_rc_frame', 'Integrity', 'on', 'Deterministic', 'on', 'OutPortSampleTime', 'Ts.base', ...
    'InitialCondition', 'P.ic.Bus_RcFrame');
L(n, 'RC_Receiver/1', 'RT_rc_frame/1');
B(n, 'simulink/Sinks/Out1', 'sbus', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_SbusStream', 'SampleTime', 'Ts.base');
B(n, 'simulink/Sinks/Out1', 'rc_frame', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_RcFrame', 'SampleTime', 'Ts.base');
L(n, 'UART_Line_Sim/1', 'sbus/1', 'sbus');
L(n, 'RT_rc_frame/1', 'rc_frame/1', 'rc_frame');
local_save(n, f, ['T07 RC_Link_Sim (ICD 3.1 Operator_RC). in 1 op Bus_OperatorCmd 0.01, 2 flt Bus_FaultInject 0.01; ' ...
    'out 1 sbus Bus_SbusStream 0.001, 2 rc_frame Bus_RcFrame 0.001 (receiver frame through the deterministic ' ...
    'frame -> 1 ms RT: same timing as the SBUS bytes). Rates 0.01 -> 0.001 -> P.rc.frame_period -> 0.001 (D32); ' ...
    'no 0.014 / 0.007 signal leaves this file.']);
end

% ===========================================================================================
function local_tb_link(f)
% Bench: root op / flt (0.01, external input) -> RC_Link_Sim -> sbus / rc_frame root outports.
n = 'tb_rc_link';
local_fresh(n, f, 'model');
shaddad_configsets('attach', n);
B(n, 'simulink/Sources/In1', 'op', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_OperatorCmd', 'SampleTime', 'Ts.ctrl', ...
    'BusOutputAsStruct', 'on');
B(n, 'simulink/Sources/In1', 'flt', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_FaultInject', 'SampleTime', 'Ts.ctrl', ...
    'BusOutputAsStruct', 'on');
B(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Link', 'ReferencedSubsystem', 'RC_Link_Sim');
B(n, 'simulink/Sinks/Out1', 'sbus', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_SbusStream');
B(n, 'simulink/Sinks/Out1', 'rc_frame', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_RcFrame');
L(n, 'op/1', 'RC_Link/1', 'op');
L(n, 'flt/1', 'RC_Link/2', 'flt');
L(n, 'RC_Link/1', 'sbus/1', 'sbus');
L(n, 'RC_Link/2', 'rc_frame/1', 'rc_frame');
set_param(n, 'Description', 'T07 bench (built by build_rc_link): external op / flt -> RC_Link_Sim; tests/rc/t_rc_link.m');
shaddad_bu('save', n, f);
local_finalize(n, f);
close_system(n, 0);
end

function local_tb_speed(f)
% Speed bench: constant op (TX on) / flt -> RC_Link_Sim -> root Outports (root Outports keep the
% block-reduction optimisation from removing the RC chain; the test runs with output saving off).
n = 'tb_rc_link_speed';
local_fresh(n, f, 'model');
shaddad_configsets('attach', n);
B(n, 'simulink/Sources/Constant', 'op', 'Value', 'setfield(P.ic.Bus_OperatorCmd, ''tx_power_on'', 1)', ...
    'OutDataTypeStr', 'Bus: Bus_OperatorCmd', 'SampleTime', 'Ts.ctrl');
B(n, 'simulink/Sources/Constant', 'flt', 'Value', 'P.ic.Bus_FaultInject', 'OutDataTypeStr', 'Bus: Bus_FaultInject', ...
    'SampleTime', 'Ts.ctrl');
B(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'RC_Link', 'ReferencedSubsystem', 'RC_Link_Sim');
B(n, 'simulink/Sinks/Out1', 'sbus', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_SbusStream');
B(n, 'simulink/Sinks/Out1', 'rc_frame', 'Port', '2', 'OutDataTypeStr', 'Bus: Bus_RcFrame');
L(n, 'op/1', 'RC_Link/1', 'op');
L(n, 'flt/1', 'RC_Link/2', 'flt');
L(n, 'RC_Link/1', 'sbus/1', 'sbus');
L(n, 'RC_Link/2', 'rc_frame/1', 'rc_frame');
set_param(n, 'Description', 'T07 speed bench (built by build_rc_link): constant inputs -> RC_Link_Sim; t_rc_link/real_time_factor');
shaddad_bu('save', n, f);
local_finalize(n, f);
close_system(n, 0);
end

function local_tb_uart(f)
% Unit bench of the rc_uart_line_sim file: frame bytes 1..25, frame_new every frame period, no faults.
n = 'tb_rc_uart_line';
local_fresh(n, f, 'model');
shaddad_configsets('attach', n);
B(n, 'simulink/Sources/Constant', 'Frame', 'Value', 'uint8(1:25)', 'OutDataTypeStr', 'uint8', 'SampleTime', 'Ts.base');
B(n, 'simulink/Sources/Pulse Generator', 'Release', 'PulseType', 'Sample based', 'Amplitude', '1', ...
    'Period', 'round(P.rc.frame_period/Ts.base)', 'PulseWidth', '1', 'PhaseDelay', '0', 'SampleTime', 'Ts.base');
B(n, 'simulink/Signal Attributes/Data Type Conversion', 'Release_bool', 'OutDataTypeStr', 'boolean');
B(n, 'simulink/Sources/Constant', 'flt', 'Value', 'P.ic.Bus_FaultInject', 'OutDataTypeStr', 'Bus: Bus_FaultInject', ...
    'SampleTime', 'Ts.base');
B(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'UART', 'ReferencedSubsystem', 'rc_uart_line_sim');
B(n, 'simulink/Sinks/Out1', 'sbus', 'Port', '1', 'OutDataTypeStr', 'Bus: Bus_SbusStream');
L(n, 'Frame/1', 'UART/1', 'frame_bytes');
L(n, 'Release/1', 'Release_bool/1');
L(n, 'Release_bool/1', 'UART/2', 'frame_new');
L(n, 'flt/1', 'UART/3', 'flt');
L(n, 'UART/1', 'sbus/1', 'sbus');
set_param(n, 'Description', 'T07 unit bench (built by build_rc_link): rc_uart_line_sim.slx byte timing; t_rc_link/uart_line_unit');
shaddad_bu('save', n, f);
local_finalize(n, f);
close_system(n, 0);
end

function local_tb_encoder(f)
% Bench: random full-range Bus_RcFrame (0..2047 channels, random flags) -> rc_sbus_encoder.
n = 'tb_rc_sbus_encoder';
local_fresh(n, f, 'model');
shaddad_configsets('attach', n);
FR = 'Ts.rc_frame';
B(n, 'simulink/Sources/Uniform Random Number', 'Rand_ch', 'Minimum', 'zeros(1,16)', 'Maximum', '2048*ones(1,16)', ...
    'Seed', '7*(1:16)+11', 'SampleTime', FR);
B(n, 'simulink/Math Operations/Rounding Function', 'Floor_ch', 'Operator', 'floor');
B(n, 'simulink/Discontinuities/Saturation', 'Sat_ch', 'UpperLimit', '2047', 'LowerLimit', '0');
B(n, 'simulink/Signal Attributes/Data Type Conversion', 'Ch_u16', 'OutDataTypeStr', 'uint16');
L(n, 'Rand_ch/1', 'Floor_ch/1');
L(n, 'Floor_ch/1', 'Sat_ch/1');
L(n, 'Sat_ch/1', 'Ch_u16/1');
B(n, 'simulink/Sources/Uniform Random Number', 'Rand_fl', 'Minimum', 'zeros(1,4)', 'Maximum', 'ones(1,4)', ...
    'Seed', '101:104', 'SampleTime', FR);
B(n, 'simulink/Logic and Bit Operations/Compare To Constant', 'Fl_bool', 'relop', '>=', 'const', '0.5', ...
    'OutDataTypeStr', 'boolean');
B(n, 'simulink/Signal Routing/Demux', 'Fl_demux', 'Outputs', '4');
L(n, 'Rand_fl/1', 'Fl_bool/1');
L(n, 'Fl_bool/1', 'Fl_demux/1');
B(n, 'simulink/Sources/Constant', 'Sent', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', FR);
B(n, 'simulink/Signal Routing/Bus Creator', 'BC', 'Inputs', '6', 'OutDataTypeStr', 'Bus: Bus_RcFrame', ...
    'NonVirtualBus', 'on', 'InheritFromInputs', 'off');
L(n, 'Ch_u16/1', 'BC/1', 'ch_raw');
L(n, 'Fl_demux/1', 'BC/2', 'ch17');
L(n, 'Fl_demux/2', 'BC/3', 'ch18');
L(n, 'Fl_demux/3', 'BC/4', 'frame_lost');
L(n, 'Fl_demux/4', 'BC/5', 'failsafe');
L(n, 'Sent/1', 'BC/6', 'sent');
B(n, 'simulink/Ports & Subsystems/Subsystem Reference', 'Encoder', 'ReferencedSubsystem', 'rc_sbus_encoder');
L(n, 'BC/1', 'Encoder/1', 'frame');
B(n, 'simulink/Sinks/Out1', 'frame_bytes', 'Port', '1', 'OutDataTypeStr', 'uint8');
B(n, 'simulink/Sinks/Out1', 'frame_new', 'Port', '2', 'OutDataTypeStr', 'boolean');
B(n, 'simulink/Sinks/Out1', 'frame', 'Port', '3', 'OutDataTypeStr', 'Bus: Bus_RcFrame');
L(n, 'Encoder/1', 'frame_bytes/1', 'frame_bytes');
L(n, 'Encoder/2', 'frame_new/1', 'frame_new');
L(n, 'BC/1', 'frame/1');
set_param(n, 'Description', 'T07 bench (built by build_rc_link): random Bus_RcFrame -> rc_sbus_encoder; tests/rc/t_rc_link.m');
shaddad_bu('save', n, f);
local_finalize(n, f);
close_system(n, 0);
end
