function res = mcu_can2_roundtrip(msg, N, seed)
%MCU_CAN2_ROUNDTRIP  Random round trip through can2_<msg>_pack -> can2_<msg>_unpack (T-EL-15).
%
%   res = mcu_can2_roundtrip(msg, N, seed)   msg: comp_cmd | mcu_status | mcu_imu | mcu_odom | mcu_power
%   N random samples (0.01 s apart) of the engineering bus. Per element:
%     res.(element) = struct('maxErr', ..., 'tol', ..., 'ok', ...)
%   tol = 0 for singles, integers and booleans (bit-exact), resolution/2 for scaled integers; elements
%   that the layout does not carry are checked against their documented substitutes (P_pack = V*I,
%   CVL = DVL = P_chg_lim = t_stamp = 0, McuStatus.t_mcu = unwrapped t_ms / 1000, valid = input valid).
%   res.frame: .id .len (constant), .pad_zero (bytes after len are 0), .valid

if nargin < 2, N = 400; end
if nargin < 3, seed = 15; end
rng(seed);
P = evalin('base', 'P');
map = struct('comp_cmd', 'Bus_GncCmd', 'mcu_status', 'Bus_McuStatus', 'mcu_imu', 'Bus_ImuMeas', ...
    'mcu_odom', 'Bus_OdomMeas', 'mcu_power', 'Bus_BmsStatus');
B = evalin('base', map.(msg));
res_ = P.electronics.can.res;
scaled = struct('mower_speed_cmd', res_.mower_w, 'tiller_speed_cmd', res_.tiller_w, 'deck_height_cmd', res_.deck_h, ...
    'tiller_depth_cmd', res_.till_d, 'spray_flow_cmd', res_.spray_flow, 'rc_lq', res_.rc_lq);
t = (0:N-1)'*0.01;
X = struct();
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    d = prod(e.Dimensions);
    switch e.DataType
        case 'boolean', v = rand(N, d) > 0.5;
        case 'single', v = single((rand(N, d) - 0.5) * 2 * 100 .* 10.^(rand(N, d) * 4 - 2));
        case 'double', v = (rand(N, d) - 0.5) * 1e3;
        otherwise
            lo = double(intmin(e.DataType)); hi = double(intmax(e.DataType));
            v = cast(randi([lo hi], N, d), e.DataType);
    end
    if isfield(scaled, e.Name)                % physical range of the scaled integer field
        % physical range of the element (bus Min / Max, e.g. mower 0..330 rad/s, deck 0.03..0.12 m)
        lo = 0; hi = 1;
        if ~isempty(e.Min), lo = e.Min; end
        if ~isempty(e.Max), hi = e.Max; end
        v = single(lo + rand(N, d) * (hi - lo));
    end
    X.(e.Name) = timeseries(v, t);
end
inputs = {{'x', X}};
if strcmp(msg, 'mcu_status')
    tms = uint32(mod(2^32 - 3000 + 50*(0:N-1)', 2^32));         % crosses the uint32 wrap
    inputs{end+1} = {'t_ms', timeseries(tms, t)};
end
ds = mcu_harness('ds', inputs{:});
o = mcu_harness('sim', ['can2_' msg], t(end), ds);
Y = o.yout.get('y').Values;
F = o.yout.get('frame').Values;
res = struct();
for k = 1:numel(B.Elements)
    nm = B.Elements(k).Name;
    xin = double(rows(X.(nm).Data, N));
    yout = double(rows(Y.(nm).Data, N));
    tol = 0;
    if isfield(scaled, nm)                % 1/2 LSB (+ single rounding of the engineering value)
        tol = scaled.(nm)/2 + double(eps(single(max(abs(double(rows(X.(nm).Data, N))))))) * 2;
    end
    ref = xin;
    switch [msg ':' nm]
        case 'comp_cmd:t_stamp', ref = zeros(size(xin));
        case 'mcu_status:t_mcu', ref = (2^32 - 3000 + 50*(0:N-1)')/1000; tol = 1e-9;
        case 'mcu_status:valid', ref = ones(size(xin));
        case {'mcu_power:CVL', 'mcu_power:DVL', 'mcu_power:P_chg_lim'}, ref = zeros(size(xin));
        case 'mcu_power:P_pack'
            ref = double(single(rows(X.V_pack.Data, N)) .* single(rows(X.I_pack.Data, N)));
    end
    if numel(yout) ~= numel(ref) || isempty(ref)
        res.(nm) = struct('maxErr', NaN, 'tol', tol, 'ok', false);
        continue
    end
    err = max(abs(yout(:) - ref(:)));
    res.(nm) = struct('maxErr', err, 'tol', tol, 'ok', err <= tol);
end
dat = rows(F.data.Data, N);
len = P.electronics.can.len.(msg);
res.frame = struct('id', unique(double(F.id.Data(:)))', 'len', unique(double(F.len.Data(:)))', ...
    'pad_zero', all(all(dat(:, len+1:end) == 0)), 'valid', all(F.valid.Data(:)), ...
    'id_ok', all(double(F.id.Data(:)) == P.electronics.can.id.(msg)), 'len_ok', all(double(F.len.Data(:)) == len));
end

function A = rows(D, n)
D = squeeze(D);
if isvector(D), A = D(:); elseif size(D, 1) == n, A = D; else, A = reshape(D, [], n).'; end
end
