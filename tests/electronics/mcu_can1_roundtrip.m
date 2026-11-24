function res = mcu_can1_roundtrip(N, seed)
%MCU_CAN1_ROUNDTRIP  Random round trips through fw_can1_codec and the CAN1 codecs of T04 / T05 / T06 (T-EL-15).
%
%   res = mcu_can1_roundtrip(N, seed)     N samples at 0.01 s (T = N*0.01)
%   TX: Bus_DriveCmd -> fw_can1_codec (drive_cmd_pack) -> drive_cmd_unpack; implement command ->
%       implcmd_can_pack -> implcmd_can_unpack; BMS request flags -> bmsreq_can_pack -> bmsreq_can_unpack.
%   RX: Bus_DriveSideFb -> drive_fb_pack -> fw_can1_codec (drive_fb_unpack); implfb_can_pack -> codec;
%       Bus_BMS_Telemetry -> bms_can_pack -> codec (bms_can_unpack, valid by frame age).
%   res.<check> = struct('maxErr', e, 'tol', tol, 'ok', e <= tol) (tol = 1/2 LSB + single rounding;
%   0 for integers / booleans); res.bms_timeout: valid drops 0.6 s after the last frame (> bms_timeout 0.5 s
%   in 0.1 s frame-age steps), res.alive_*: +1 per frame.

if nargin < 1, N = 300; end
if nargin < 2, seed = 16; end
rng(seed);
P = evalin('base', 'P');
T = (N - 1)*0.01;
t10 = (0:N-1)'*0.01; t50 = (0:0.05:T)'; t100 = (0:0.1:T)';
n50 = numel(t50); n100 = numel(t100);
U = @(n, lo, hi) single(lo + rand(n, 1)*(hi - lo));
B = @(n) rand(n, 1) > 0.5;
% ---- drive commands ----
for sd = {'L', 'R'}
    c.(sd{1}) = struct('enable', timeseries(B(N), t10), 'mode', timeseries(uint8(randi([0 2], N, 1)), t10), ...
        'w_ref', timeseries(U(N, -502.7, 502.7), t10), 'tau_ff', timeseries(U(N, -16, 16), t10), ...
        'tau_lim_mot', timeseries(U(N, 0, 16), t10), 'tau_lim_reg', timeseries(U(N, 0, 16), t10), ...
        'park_req', timeseries(B(N), t10), 'fault_reset', timeseries(B(N), t10), ...
        'alive', timeseries(uint8(randi([0 255], N, 1)), t10));
end
% ---- ESC feedback: FB1 / FB3 fields random per sample, FB2 fields constant ----
for sd = {'L', 'R'}
    f.(sd{1}) = struct('state', timeseries(uint8(randi([0 8], N, 1)), t10), ...
        'status', timeseries(uint16(randi([0 2047], N, 1)), t10), 'fault_code', timeseries(uint16(randi([0 2047], N, 1)), t10), ...
        'w_m', timeseries(U(N, -600, 600), t10), 'enc_cnt', timeseries(int32(randi([-2^31 2^31-1], N, 1)), t10), ...
        'tau_est', timeseries(U(N, -16, 16), t10), 'tau_avail', timeseries(repmat(U(1, 0, 16), N, 1), t10), ...
        'i_q', timeseries(repmat(U(1, -100, 100), N, 1), t10), 'i_dc', timeseries(U(N, -150, 150), t10), ...
        'v_dc', timeseries(repmat(U(1, 40, 58), N, 1), t10), 'T_motor', timeseries(repmat(U(1, 280, 400), N, 1), t10), ...
        'T_esc', timeseries(repmat(single(273.15 + randi([0 100])), N, 1), t10), 'brake_released', timeseries(B(N), t10), ...
        'alive', timeseries(uint8(randi([0 255], N, 1)), t10), 'rx_ok', timeseries(true(N, 1), t10));
end
% ---- implement / BMS request (0.05) ----
ien = B(n50); irst = B(n50); irpm = U(n50, -3000, 3000); req = rand(n50, 4) > 0.5;
ifb = {B(n50), B(n50), B(n50), uint8(randi([0 3], n50, 1)), U(n50, -3000, 3000), U(n50, -300, 300), ...
    U(n50, -40, 200), uint8(randi([0 255], n50, 1))};
% ---- BMS telemetry (0.1), frames dropped after T/2 ----
BT = evalin('base', 'Bus_BMS_Telemetry');
tel = struct();
for k = 1:numel(BT.Elements)
    e = BT.Elements(k);
    nd = prod(e.Dimensions);
    switch e.DataType
        case 'boolean', v = rand(n100, nd) > 0.5;
        case 'single'
            lo = 0; hi = 1;
            if ~isempty(e.Min), lo = e.Min; end
            if ~isempty(e.Max), hi = e.Max; end
            if isempty(e.Max) && isempty(e.Min), lo = 1; hi = 100; end
            v = single(lo + rand(n100, nd)*(hi - lo));
        otherwise
            hi = double(intmax(e.DataType)); if ~isempty(e.Max), hi = e.Max; end
            v = cast(randi([0 hi], n100, nd), e.DataType);
    end
    tel.(e.Name) = timeseries(v, t100);
end
tel.V_pack_V = timeseries(U(n100, 44, 58), t100); tel.I_pack_A = timeseries(U(n100, -100, 300), t100);
tel.T_cell_min_K = timeseries(U(n100, 270, 320), t100); tel.T_cell_max_K = timeseries(U(n100, 290, 330), t100);
tel.V_cell_min_V = timeseries(U(n100, 2.8, 3.3), t100); tel.V_cell_max_V = timeseries(U(n100, 3.3, 3.6), t100);
bmsOn = t100 < T/2;
ds = mcu_harness('ds', {'drive_cmd', c}, {'fb_L', f.L}, {'fb_R', f.R}, {'impl_en', timeseries(ien, t50)}, ...
    {'impl_rst', timeseries(irst, t50)}, {'impl_rpm', timeseries(irpm, t50)}, {'req', timeseries(req, t50)}, ...
    {'ifb_running', timeseries(ifb{1}, t50)}, {'ifb_fault', timeseries(ifb{2}, t50)}, {'ifb_ready', timeseries(ifb{3}, t50)}, ...
    {'ifb_impl_type', timeseries(ifb{4}, t50)}, {'ifb_speed_rpm', timeseries(ifb{5}, t50)}, ...
    {'ifb_current_A', timeseries(ifb{6}, t50)}, {'ifb_temp_degC', timeseries(ifb{7}, t50)}, ...
    {'ifb_alive', timeseries(ifb{8}, t50)}, {'bms_telem', tel}, {'bms_on', timeseries(bmsOn, t100)});
o = mcu_harness('sim', 'can1', T, ds);
y = o.yout;
g = @(nm) y.get(nm).Values;
res = struct();
d = P.drive.can;
% ---- drive commands (10 ms, same step) ----
for sd = {'L', 'R'}
    Y = g(['cmd_' sd{1}]); X = c.(sd{1});
    chk = {'enable', 0; 'mode', 0; 'park_req', 0; 'fault_reset', 0; 'alive', 0; 'w_ref', d.res_w/2; ...
        'tau_ff', d.res_tau/2; 'tau_lim_mot', d.res_tau_lim/2; 'tau_lim_reg', d.res_tau_lim/2};
    for k = 1:size(chk, 1)
        res.(['cmd_' sd{1} '_' chk{k, 1}]) = cmp(X.(chk{k, 1}).Data, Y.(chk{k, 1}).Data, chk{k, 2});
    end
end
% ---- ESC feedback (FB1 / FB3 same step; FB2 constant) ----
FB = g('drive_fb');
for sd = {'L', 'R'}
    Y = FB.(sd{1}); X = f.(sd{1});
    chk = {'state', 0; 'status', 0; 'fault_code', 0; 'enc_cnt', 0; 'alive', 0; 'w_m', pi/30/2; ...
        'tau_est', d.res_tau/2; 'i_dc', d.res_i/2; 'v_dc', d.res_v/2; 'i_q', d.res_i/2; 'T_motor', d.res_T/2; ...
        'T_esc', 0.5; 'tau_avail', d.res_tau_lim/2};
    for k = 1:size(chk, 1)
        xs = X.(chk{k, 1}).Data; ys = Y.(chk{k, 1}).Data;
        if any(strcmp(chk{k, 1}, {'v_dc', 'i_q', 'T_motor', 'T_esc', 'tau_avail'}))
            xs = xs(11:end); ys = squeeze(ys); ys = ys(11:end);     % FB2: after the first 0.1 s
        end
        res.(['fb_' sd{1} '_' chk{k, 1}]) = cmp(xs, ys, chk{k, 2});
    end
    rx = squeeze(Y.rx_ok.Data); res.(['fb_' sd{1} '_rx_ok']) = struct('maxErr', double(~all(rx(2:end))), 'tol', 0, 'ok', all(rx(2:end)));
end
% ---- implement command (50 ms) ----
res.impl_cmd_enable = cmp(ien, g('icu_en').Data, 0);
res.impl_cmd_fault_reset = cmp(irst, g('icu_rst').Data, 0);
res.impl_cmd_speed = cmp(irpm, g('icu_rpm').Data, 0.5);
al = double(squeeze(g('icu_alive').Data));
res.alive_impl = struct('maxErr', max(abs(mod(diff(al), 256) - 1)), 'tol', 0, 'ok', all(mod(diff(al), 256) == 1));
% ---- BMS request (50 ms) ----
RQ = g('req_out');
nmr = {'HV_Enable_Req', 'Charge_Enable_Req', 'Fault_Reset_Req', 'Shutdown_Req'};
for k = 1:4, res.(['bms_req_' nmr{k}]) = cmp(req(:, k), RQ.(nmr{k}).Data, 0); end
al = double(squeeze(RQ.AliveCounter.Data));
res.alive_bms_req = struct('maxErr', max(abs(mod(diff(al), 256) - 1)), 'tol', 0, 'ok', all(mod(diff(al), 256) == 1));
% ---- implement feedback (50 ms, logged codec output) ----
IM = o.logsout.get('impl').Values;
nmi = {'running', 0; 'fault', 0; 'ready', 0; 'impl_type', 0; 'speed_rpm', 0.5; 'current_A', 0.05; 'temp_degC', 0.5; 'alive', 0};
for k = 1:size(nmi, 1), res.(['impl_fb_' nmi{k, 1}]) = cmp(ifb{k}, IM.(nmi{k, 1}).Data, nmi{k, 2}); end
% ---- BMS (100 ms): selected fields while frames are on, timeout after they stop ----
BS = g('bms'); on = find(bmsOn);
pick = @(ts) local_rows(ts.Data);
bf = {'V_pack', 'V_pack_V', 0.005; 'I_pack', 'I_pack_A', 0.05; 'DCL', 'I_dis_lim_A', 0.05; 'CCL', 'I_chg_lim_A', 0.05; ...
    'alarm_word', 'Fault_Word', 0; 'warn_word', 'Warning_Word', 0; 'fault_level', 'Fault_Level', 0; ...
    'contactor_state', 'Contactor_State', 0; 'hv_ready', 'HV_Ready', 0};
for k = 1:size(bf, 1)
    xs = pick(tel.(bf{k, 2})); ys = pick(BS.(bf{k, 1}));
    res.(['bms_' bf{k, 1}]) = cmp(xs(on), ys(on), bf{k, 3});
end
vb = logical(pick(BS.valid)); tb = BS.valid.Time;
tLast = t100(find(bmsOn, 1, 'last'));
tOff = tb(find(~vb & tb > tLast, 1));
res.bms_valid_on = struct('maxErr', double(~all(vb(on))), 'tol', 0, 'ok', all(vb(on)));
res.bms_timeout = struct('maxErr', tOff - tLast, 'tol', 0.6 + 1e-9, 'ok', ~isempty(tOff) && tOff - tLast > P.electronics.safety.bms_timeout && tOff - tLast <= 0.6 + 1e-9);
end

function s = cmp(x, y, tol)
x = double(local_rows(x)); y = double(local_rows(y));
n = min(numel(x), numel(y));
if n < 5 || abs(numel(x) - numel(y)) > 1     % both series must exist with the same sample count (+/- 1)
    s = struct('maxErr', NaN, 'tol', tol, 'ok', false);
    return
end
e = max(abs(x(1:n) - y(1:n)));
if tol > 0, tol = tol + double(eps(single(max(abs(x(1:n)))))) * 2; end
s = struct('maxErr', e, 'tol', tol, 'ok', e <= tol);
end

function A = local_rows(D)
A = squeeze(D); A = A(:);
end
