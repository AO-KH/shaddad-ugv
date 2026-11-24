function r = mcu_cl_run(T, tab, P)
%MCU_CL_RUN  Simulate the T09 bench tests/electronics/tb_mcu_closed_loop.slx and return plain arrays.
%
%   r = mcu_cl_run(T, tab)        inputs as mcu_cl_input(T, tab); base-workspace P
%   r = mcu_cl_run(T, tab, P)     P override for this run (SimulationInput.setVariable)
%   r = mcu_cl_run(T, {tab1, tab2, ...}, P)   several runs in one fast-restart batch (T scalar or one
%                                 stop time per run; r is then a cell array; P is the same for all)
%   Output r:
%     .t1 (0.001 s): .state (N1 x 8 [v_L v_R v_mean yaw_rate x y psi relay]), .di_word, .do_word, .wdg_kick,
%           .do0 / .do1 (DO bits 0 / 1), .rc_frame (struct of arrays: ch_raw, frame_lost, failsafe, sent),
%           .dec (logged rc_dec of Shaddad_MCU/Task_1k: frame_new, failsafe, rc_link_lost, ...)
%     .t10 (0.01 s): .telem (Bus_FwTelemetry arrays), .cmd (decoded DRV_CMD: .L / .R arrays), .drvL / .drvR
%           (N10 x 8 DRV_CMD payload bytes), .rc (logged fw_rc_mapping bus)
%     .t50 / .status (N50 x 64 MCU_STATUS payload), .wall (s, execution without compile)

mdl = 'tb_mcu_closed_loop';
if ~bdIsLoaded(mdl), load_system(mdl); end
multi = iscell(tab);
if ~multi, tab = {tab}; end
if isscalar(T), T = repmat(T, 1, numel(tab)); end
in = Simulink.SimulationInput.empty;
for k = 1:numel(tab)
    x = Simulink.SimulationInput(mdl);
    x = x.setExternalInput(mcu_cl_input(T(k), tab{k}));
    x = x.setModelParameter('StopTime', sprintf('%.6f', T(k)), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
        'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
    if nargin >= 3 && ~isempty(P), x = x.setVariable('P', P); end
    in(k) = x; %#ok<AGROW>
end
if numel(in) > 1
    out = sim(in, 'UseFastRestart', 'on', 'ShowProgress', 'off', 'ShowSimulationManager', 'off');
else
    out = sim(in);
end
res = cell(1, numel(out));
for k = 1:numel(out)
    if ~isempty(out(k).ErrorMessage), error('mcu_cl_run:sim', '%s', out(k).ErrorMessage); end
    res{k} = local_unpack(out(k));
end
if multi, r = res; else, r = res{1}; end
end

function r = local_unpack(out)
y = out.yout;
st = y{7}.Values;                                   % state (port 7)
r.t1 = st.Time;
n1 = numel(r.t1);
r.state = rows(st.Data, n1);
r.di_word = double(rows(y{8}.Values.Data, n1));
o1 = y{1}.Values;
r.do_word = double(rows(o1.do_word.Data, n1));
r.wdg_kick = logical(rows(o1.wdg_kick.Data, n1));
r.do0 = bitand(r.do_word, 1) ~= 0; r.do1 = bitand(r.do_word, 2) ~= 0;
r.rc_frame = flat(y{6}.Values, n1);
o10 = y{2}.Values;
r.t10 = o10.pwm_duty.Time;
n10 = numel(r.t10);
r.drvL = rows(o10.can1_drv.drv_cmd_L.data.Data, n10);
r.drvR = rows(o10.can1_drv.drv_cmd_R.data.Data, n10);
r.telem = flat(y{4}.Values, n10);
r.cmd = flat(y{5}.Values, n10);
o50 = y{3}.Values;
r.t50 = o50.can2_status.valid.Time;
r.status = rows(o50.can2_status.data.Data, numel(r.t50));
ls = out.logsout;
r.dec = flat(ls.get('rc_dec').Values, n1);
r.rc = flat(ls.get('rc').Values, n10);
r.meta = out.SimulationMetadata;
r.wall = r.meta.TimingInfo.ExecutionElapsedWallTime;
end

function s = flat(v, n)
s = struct();
fn = fieldnames(v);
for k = 1:numel(fn)
    x = v.(fn{k});
    if isstruct(x)
        s.(fn{k}) = flat(x, n);
    else
        m = n;
        if isempty(m), m = numel(x.Time); end
        d = rows(x.Data, m);
        if islogical(d) || isinteger(d), s.(fn{k}) = d; else, s.(fn{k}) = double(d); end
    end
end
end

function A = rows(D, n)
D = squeeze(D);
if isvector(D)
    A = D(:);
elseif size(D, 1) == n
    A = D;
else
    A = reshape(D, [], n).';
end
end
