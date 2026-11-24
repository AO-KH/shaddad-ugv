function r = mcu_tb_run(T, opTab, fltTab, P, varargin)
%MCU_TB_RUN  Simulate the T08 bench tests/electronics/tb_mcu_rc.slx and return plain arrays.
%
%   r = mcu_tb_run(T, opTab, fltTab)        base-workspace P (always with P.electronics.imu_cal = 0: ideal IMU)
%   r = mcu_tb_run(T, opTab, fltTab, P)     P override for this run (SimulationInput.setVariable)
%   r = mcu_tb_run(..., 'SimulationMode', m)
%   Inputs as tests/rc/rc_tb_input (op / flt tables on the 0.01 s grid, tx_power_on = 1 by default).
%   Output r:
%     .t1 (N1 x 1, 0.001 s): .len .bytes (N1 x 32) .ch_raw_tx (N1 x 16, rc_frame port) .lost_tx .fs_tx .sent_tx
%     .dec  struct of N1-sample arrays logged in Shaddad_MCU/Task_1k (rc_dec): ch_raw (N1 x 16), ch17, ch18,
%           frame_lost, failsafe, frame_new, frame_valid, last_valid, rc_age, rc_link_lost, rc_ok, lq,
%           lq_warn, n_ok, n_bad, n_skip
%     .t10 (N10 x 1, 0.01 s): .rc struct of arrays (fw_rc_mapping output), .drvL / .drvR (N10 x 8 uint8 DRV_CMD
%           data), .drvL_valid, .telem (struct of arrays of Bus_FwTelemetry), .busoff (N10 x 2)
%     .do_word / .wdg_kick (N1 x 1, out1k), .t50 / .status (N50 x 64 MCU_STATUS data) / .bmsreq (N50 x 8),
%     .tilt (logged Task_5 tilt bus), .wall (s, execution without compile)

ip = inputParser;
ip.addParameter('SimulationMode', 'normal');
ip.parse(varargin{:});
mdl = 'tb_mcu_rc';
if ~bdIsLoaded(mdl), load_system(mdl); end
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(rc_tb_input(T, opTab, fltTab));
in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', 'SimulationMode', ip.Results.SimulationMode);
if nargin < 4 || isempty(P), P = evalin('base', 'P'); end
P.electronics.imu_cal = struct('roll0', 0, 'pitch0', 0);       % tb_mcu_rc has a level, ideal IMU: no level calibration
in = in.setVariable('P', P);
out = sim(in);
if ~isempty(out.ErrorMessage), error('mcu_tb_run:sim', '%s', out.ErrorMessage); end
y = out.yout;
sb = y.get('sbus').Values;
rf = y.get('rc_frame').Values;
r.t1 = sb.len.Time;
n1 = numel(r.t1);
r.len = double(sb.len.Data(:));
r.bytes = rows(sb.bytes.Data, n1);
r.ch_raw_tx = rows(rf.ch_raw.Data, n1);
r.lost_tx = logical(rf.frame_lost.Data(:));
r.fs_tx = logical(rf.failsafe.Data(:));
r.sent_tx = logical(rf.sent.Data(:));
o1 = y.get('out1k').Values;
r.do_word = double(o1.do_word.Data(:));
r.wdg_kick = logical(o1.wdg_kick.Data(:));
o10 = y.get('out10').Values;
r.t10 = o10.pwm_duty.Time;
n10 = numel(r.t10);
r.drvL = rows(o10.can1_drv.drv_cmd_L.data.Data, n10);
r.drvR = rows(o10.can1_drv.drv_cmd_R.data.Data, n10);
r.drvL_valid = logical(o10.can1_drv.drv_cmd_L.valid.Data(:));
r.telem = flat(y.get('telem').Values, n10);
o50 = y.get('out50').Values;
r.t50 = o50.can2_status.valid.Time;
r.status = rows(o50.can2_status.data.Data, numel(r.t50));
r.bmsreq = rows(o50.can1_slow.bms_req.data.Data, numel(r.t50));
ls = out.logsout;
r.dec = flat(ls.get('rc_dec').Values, n1);
r.rc = flat(ls.get('rc').Values, n10);
r.busoff = rows(ls.get('busoff').Values.Data, n10);
r.tilt = flat(ls.get('tilt').Values, []);
r.meta = out.SimulationMetadata;
r.wall = r.meta.TimingInfo.ExecutionElapsedWallTime;
end

function s = flat(v, n)
% struct of timeseries -> struct of arrays (rows = samples)
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
% logged data -> n x width (time is the last dimension for vectors, the first for scalars)
D = squeeze(D);
if isvector(D)
    A = D(:);
elseif size(D, 1) == n
    A = D;
else
    A = reshape(D, [], n).';
end
end
