function R = tb_drive_codec_check(P, N)
%TB_DRIVE_CODEC_CHECK  T16: bit-exact round trip of the DRV CAN codecs + data-type audit (T06).
%   R = tb_drive_codec_check(P, N)   N random commands / feedbacks at 10 ms (seeded)
%   Builds an in-memory harness (never saved): DRV_CMD drive_cmd_pack -> frame drop (10 %) -> drive_cmd_unpack;
%   DRV_FB drive_fb_pack -> drive_fb_unpack. Expected values replicate the single / integer arithmetic of the
%   codec blocks (Gain / Bias in single, Data Type Conversion round-to-nearest (ties to +inf) with saturation).
%   R: .cmd_exact .fb_exact .cmd_mismatch .fb_mismatch .hold_ok .ids_ok .wrap_ok .rxok_ok .doubles (cellstr)
%      .layout_ok .layout_mismatch [CMD FB1 FB2 FB3]: frame bytes equal the ICD 4 table (offsets, types, scaling,
%      little-endian, ctrl bits b0 enable b1 park_req b2 fault_reset b3-4 mode b8-15 alive)
if nargin < 2, N = 10000; end
m = 'tmp_T06_codec';
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', ...
    'StopTime', num2str((N - 1)*0.01, 12), 'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'off', ...
    'UnconnectedInputMsg', 'none');
A = @(lib, p, varargin) add_block(lib, [m '/' p], varargin{:});
Ln = @(s, d) add_line(m, s, d, 'autorouting', 'smart');
FW = @(p, v, ty) A('simulink/Sources/From Workspace', p, 'VariableName', v, 'SampleTime', '0.01', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'OutDataTypeStr', ty);
TW = @(p, v) A('simulink/Sinks/To Workspace', p, 'VariableName', v, 'SaveFormat', 'Timeseries');
FW('CmdIn', 'cmd_in', 'Bus: Bus_DriveSideCmd'); FW('Drop', 'drop', 'boolean'); FW('FbIn', 'fb_in', 'Bus: Bus_DriveSideFb');
A('simulink/Sources/Constant', 'Node1', 'Value', 'uint8(1)', 'OutDataTypeStr', 'uint8', 'SampleTime', '0.01');
A('simulink/Sources/Constant', 'Node2', 'Value', 'uint8(2)', 'OutDataTypeStr', 'uint8', 'SampleTime', '0.01');
A('simulink/Ports & Subsystems/Subsystem Reference', 'CmdPack', 'ReferencedSubsystem', 'drive_cmd_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', 'CmdUnpack', 'ReferencedSubsystem', 'drive_cmd_unpack');
A('simulink/Ports & Subsystems/Subsystem Reference', 'FbPack', 'ReferencedSubsystem', 'drive_fb_pack');
A('simulink/Ports & Subsystems/Subsystem Reference', 'FbUnpack', 'ReferencedSubsystem', 'drive_fb_unpack');
A('simulink/Signal Routing/Bus Selector', 'Sel', 'OutputSignals', 'id,len,data,valid');
A('simulink/Logic and Bit Operations/Logical Operator', 'NotDrop', 'Operator', 'NOT', 'Inputs', '1');
A('simulink/Logic and Bit Operations/Logical Operator', 'Kept', 'Operator', 'AND', 'Inputs', '2');
A('simulink/Signal Routing/Bus Creator', 'Frame', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', 'NonVirtualBus', 'on');
Ln('CmdIn/1', 'CmdPack/1'); Ln('Node1/1', 'CmdPack/2'); Ln('CmdPack/1', 'Sel/1');
Ln('Drop/1', 'NotDrop/1'); Ln('Sel/4', 'Kept/1'); Ln('NotDrop/1', 'Kept/2');
Ln('Sel/1', 'Frame/1'); Ln('Sel/2', 'Frame/2'); Ln('Sel/3', 'Frame/3');
h = Ln('Kept/1', 'Frame/4'); set_param(h, 'Name', 'valid');
Ln('Frame/1', 'CmdUnpack/1'); Ln('Node1/1', 'CmdUnpack/2');
TW('LogCmd', 'cmd_out'); TW('LogCmdFrame', 'cmd_frame'); Ln('CmdUnpack/1', 'LogCmd/1'); Ln('CmdPack/1', 'LogCmdFrame/1');
Ln('FbIn/1', 'FbPack/1'); Ln('Node2/1', 'FbPack/2');
% FB1 passes a drop gate (10 consecutive frames lost: rx_ok timeout and payload hold); FB2 / FB3 direct
FW('DropF', 'dropf', 'boolean');
A('simulink/Signal Routing/Bus Selector', 'SelF', 'OutputSignals', 'id,len,data,valid');
A('simulink/Logic and Bit Operations/Logical Operator', 'NotDropF', 'Operator', 'NOT', 'Inputs', '1');
A('simulink/Logic and Bit Operations/Logical Operator', 'KeptF', 'Operator', 'AND', 'Inputs', '2');
A('simulink/Signal Routing/Bus Creator', 'FrameF', 'Inputs', '4', 'OutDataTypeStr', 'Bus: Bus_CanFrame8', 'NonVirtualBus', 'on');
Ln('FbPack/1', 'SelF/1'); Ln('DropF/1', 'NotDropF/1'); Ln('SelF/4', 'KeptF/1'); Ln('NotDropF/1', 'KeptF/2');
Ln('SelF/1', 'FrameF/1'); Ln('SelF/2', 'FrameF/2'); Ln('SelF/3', 'FrameF/3');
h = Ln('KeptF/1', 'FrameF/4'); set_param(h, 'Name', 'valid');
Ln('FrameF/1', 'FbUnpack/1');
for k = 2:3, Ln(sprintf('FbPack/%d', k), sprintf('FbUnpack/%d', k)); end
Ln('Node2/1', 'FbUnpack/4');
TW('LogFb', 'fb_out'); Ln('FbUnpack/1', 'LogFb/1');
for k = 1:3, TW(sprintf('LogF%d', k), sprintf('fb%d_frame', k)); Ln(sprintf('FbPack/%d', k), sprintf('LogF%d/1', k)); end

% ---------------- random inputs ----------------
rs = RandStream('mt19937ar', 'Seed', 16);
t = (0:N - 1)'*0.01;
U = @(a, b) a + (b - a)*rand(rs, N, 1);
c.enable = rand(rs, N, 1) > 0.5; c.park_req = rand(rs, N, 1) > 0.5; c.fault_reset = rand(rs, N, 1) > 0.5;
c.mode = randi(rs, [0 2], N, 1); c.alive = mod((0:N - 1)', 256);
c.w_ref = U(-502.7, 502.7); c.tau_ff = U(-16, 16); c.tau_lim_mot = U(0, 16); c.tau_lim_reg = U(0, 16);
c.w_ref(1:4) = [-0.0523599; 0.0523599; -502.7; 0]; c.tau_ff(1:3) = [-0.005; 0.005; -16];   % ties / extremes / negatives
drop = rand(rs, N, 1) < 0.1; drop(1) = false;
f.state = randi(rs, [0 8], N, 1); f.status = randi(rs, [0 65535], N, 1); f.fault_code = randi(rs, [0 65535], N, 1);
f.w_m = U(-502.7, 502.7); f.enc_cnt = randi(rs, [intmin('int32') intmax('int32')], N, 1);
f.tau_est = U(-16, 16); f.tau_avail = U(0, 16); f.i_q = U(-250, 250); f.i_dc = U(-130, 130); f.v_dc = U(0, 65);
f.T_motor = U(233.15, 473.15); f.T_esc = U(233.15, 400); f.alive = mod((0:N - 1)', 256);
f.brake_released = false(N, 1); f.rx_ok = false(N, 1);
cmd_in = tb_drive_bus_ts('Bus_DriveSideCmd', t, c);
fb_in = tb_drive_bus_ts('Bus_DriveSideFb', t, f);
si = Simulink.SimulationInput(m);
si = si.setVariable('cmd_in', cmd_in, 'Workspace', m);
si = si.setVariable('fb_in', fb_in, 'Workspace', m);
si = si.setVariable('drop', timeseries(drop, t), 'Workspace', m);
dropf = false(N, 1);
if N >= 40, kf = min(501, N - 20); dropf(kf:kf + 9) = true; end   % FB1 frames kf .. kf+9 lost
si = si.setVariable('dropf', timeseries(dropf, t), 'Workspace', m);
ws = warning('off', 'Simulink:Parameters:ParamPrecisionLossWarn');
so = sim(si);
warning(ws);

% ---------------- expected (codec arithmetic) ----------------
nr = @(x, lo, hi) min(max(floor(double(x) + 0.5), lo), hi);        % Nearest (ties to +inf), saturate
s = @single;
cn = P.drive.can;
e.w_ref = s(nr(s(c.w_ref)*s(1/cn.res_w), -32768, 32767))*s(cn.res_w);
e.tau_ff = s(nr(s(c.tau_ff)*s(1/cn.res_tau), -32768, 32767))*s(cn.res_tau);
e.tau_lim_mot = s(nr(s(c.tau_lim_mot)*s(1/cn.res_tau_lim), 0, 255))*s(cn.res_tau_lim);
e.tau_lim_reg = s(nr(s(c.tau_lim_reg)*s(1/cn.res_tau_lim), 0, 255))*s(cn.res_tau_lim);
e.enable = c.enable; e.park_req = c.park_req; e.fault_reset = c.fault_reset;
e.mode = uint8(bitand(c.mode, 3)); e.alive = uint8(c.alive);
% hold last accepted frame
keep = find(~drop);
idx = keep(cumsum(~drop));                      % index of the last kept sample
fn = fieldnames(e);
co = so.cmd_out;
R.cmd_mismatch = 0; R.hold_ok = true;
for k = 1:numel(fn)
    exp_ = e.(fn{k})(idx);
    got = co.(fn{k}).Data(:);
    if ~strcmp(class(got), class(exp_)), exp_ = cast(exp_, class(got)); end
    bad = got ~= exp_;
    R.cmd_mismatch = R.cmd_mismatch + sum(bad);
    if any(bad(drop)), R.hold_ok = false; end
end
R.cmd_exact = R.cmd_mismatch == 0;
% FB
g.w_m = s(nr(s(f.w_m)*s(1/cn.res_w), -32768, 32767))*s(cn.res_w);
g.tau_est = s(nr(s(f.tau_est)*s(1/cn.res_tau), -32768, 32767))*s(cn.res_tau);
g.i_dc = s(nr(s(f.i_dc)*s(1/cn.res_i), -32768, 32767))*s(cn.res_i);
g.v_dc = s(nr(s(f.v_dc)*s(1/cn.res_v), 0, 65535))*s(cn.res_v);
g.i_q = s(nr(s(f.i_q)*s(1/cn.res_i), -32768, 32767))*s(cn.res_i);
g.T_motor = s(nr((s(f.T_motor) + s(-cn.T0))*s(1/cn.res_T), -32768, 32767))*s(cn.res_T) + s(cn.T0);
g.T_esc = s(nr(s(f.T_esc) + s(-cn.T0 + cn.T_esc_ofs), 0, 255)) + s(cn.T0 - cn.T_esc_ofs);
g.tau_avail = s(nr(s(f.tau_avail)*s(1/cn.res_tau_lim), 0, 255))*s(cn.res_tau_lim);
g.state = uint8(f.state); g.status = uint16(f.status); g.fault_code = uint16(f.fault_code);
g.enc_cnt = int32(f.enc_cnt); g.alive = uint8(f.alive);
g.brake_released = bitget(uint16(f.status), 5) == 1;
% FB1 fields hold the last accepted frame while FB1 is lost
keepF = find(~dropf); idxF = keepF(cumsum(~dropf));
for fb1f = {'status', 'w_m', 'tau_est', 'i_dc', 'brake_released'}
    g.(fb1f{1}) = g.(fb1f{1})(idxF);
end
fo = so.fb_out; fn = fieldnames(g);
R.fb_mismatch = 0;
for k = 1:numel(fn)
    got = fo.(fn{k}).Data(:); exp_ = g.(fn{k});
    if ~strcmp(class(got), class(exp_)), exp_ = cast(exp_, class(got)); end
    R.fb_mismatch = R.fb_mismatch + sum(got ~= exp_);
end
R.fb_exact = R.fb_mismatch == 0;
% rx_ok: FB1 and FB3 ages (samples since the last accepted frame) < n_rx_timeout (50 ms at 10 ms)
age = zeros(N, 1); a = cn.n_rx_timeout;
for k = 1:N
    if dropf(k), a = min(a + 1, 255); else, a = 0; end
    age(k) = a;
end
rx_exp = age < cn.n_rx_timeout;
R.rx_off_samples = find(~fo.rx_ok.Data(:))';
R.rxok_ok = isequal(logical(fo.rx_ok.Data(:)), rx_exp) && (N < 40 || sum(~rx_exp) == 6);  % frames kf+4 .. kf+9
% frame ids / len / valid
ids = [double(so.cmd_frame.id.Data(1)), double(so.fb1_frame.id.Data(1)), double(so.fb2_frame.id.Data(1)), double(so.fb3_frame.id.Data(1))];
R.ids_ok = isequal(ids, [cn.id_cmd + 1, cn.id_fb1 + 2, cn.id_fb2 + 2, cn.id_fb3 + 2]) && all(so.cmd_frame.len.Data == 8) ...
    && all(so.cmd_frame.valid.Data);
% ---------------- ICD 4 byte layout of the frames (little-endian, offsets / scaling as in the ICD table) ----------------
q = @(x, r, lo, hi) nr(s(x)*s(1/r), lo, hi);
ctrl = uint16(c.enable) + 2*uint16(c.park_req) + 4*uint16(c.fault_reset) + 8*uint16(bitand(c.mode, 3)) + 256*uint16(c.alive);
B = @(v, ty) reshape(typecast(reshape(cast(v, ty), 1, []), 'uint8'), [], numel(v))';   % N x bytes
expCmd = [B(ctrl, 'uint16'), B(q(c.w_ref, cn.res_w, -32768, 32767), 'int16'), B(q(c.tau_ff, cn.res_tau, -32768, 32767), 'int16'), ...
          B(q(c.tau_lim_mot, cn.res_tau_lim, 0, 255), 'uint8'), B(q(c.tau_lim_reg, cn.res_tau_lim, 0, 255), 'uint8')];
expFb1 = [B(f.status, 'uint16'), B(q(f.w_m, cn.res_w, -32768, 32767), 'int16'), B(q(f.tau_est, cn.res_tau, -32768, 32767), 'int16'), ...
          B(q(f.i_dc, cn.res_i, -32768, 32767), 'int16')];
expFb2 = [B(q(f.v_dc, cn.res_v, 0, 65535), 'uint16'), B(q(f.i_q, cn.res_i, -32768, 32767), 'int16'), ...
          B(nr((s(f.T_motor) + s(-cn.T0))*s(1/cn.res_T), -32768, 32767), 'int16'), ...
          B(nr(s(f.T_esc) + s(-cn.T0 + cn.T_esc_ofs), 0, 255), 'uint8'), B(q(f.tau_avail, cn.res_tau_lim, 0, 255), 'uint8')];
expFb3 = [B(f.enc_cnt, 'int32'), B(f.fault_code, 'uint16'), B(f.alive, 'uint8'), B(f.state, 'uint8')];
bytes = @(ts) local_bytes(ts.data.Data, N);
R.layout_mismatch = [sum(any(bytes(so.cmd_frame) ~= expCmd, 2)), sum(any(bytes(so.fb1_frame) ~= expFb1, 2)), ...
                     sum(any(bytes(so.fb2_frame) ~= expFb2, 2)), sum(any(bytes(so.fb3_frame) ~= expFb3, 2))];
R.layout_ok = all(R.layout_mismatch == 0);
% alive wrap 255 -> 0 decoded in sequence
al = double(co.alive.Data(:)); kept = ~drop;
R.wrap_ok = any(al(1:end - 1) == 255 & al(2:end) == 0) && isequal(al(kept), mod(find(kept) - 1, 256));
% ---------------- data-type audit of the codec instances (single / int only) ----------------
R.doubles = {}; R.n_audited = 0;
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'cmd_in', cmd_in); assignin(mw, 'fb_in', fb_in); assignin(mw, 'drop', timeseries(drop, t));
assignin(mw, 'dropf', timeseries(dropf, t));
try
    evalc('feval(m, [], [], [], ''compile'')');
    for b = {'CmdPack', 'CmdUnpack', 'FbPack', 'FbUnpack'}
        bl = find_system([m '/' b{1}], 'LookUnderMasks', 'all', 'FollowLinks', 'on');
        for j = 2:numel(bl)
            try
                dt = get_param(bl{j}, 'CompiledPortDataTypes');
            catch
                continue
            end
            if isempty(dt) || ~isfield(dt, 'Outport'), continue, end
            R.n_audited = R.n_audited + 1;
            if any(strcmp(dt.Outport, 'double'))
                R.doubles{end + 1} = strrep(bl{j}, [m '/'], ''); %#ok<AGROW>
            end
        end
    end
    evalc('feval(m, [], [], [], ''term'')');
catch err
    R.doubles{end + 1} = ['compile error: ' err.message];
end
close_system(m, 0);
end

function b = local_bytes(d, N)
% logged uint8[8] frame data -> N x 8
d = squeeze(d);
if size(d, 1) == 8 && size(d, 2) == N, b = d'; else, b = reshape(d, N, 8); end
end
