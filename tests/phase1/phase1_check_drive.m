function [checks, metrics] = phase1_check_drive(res, args)
%PHASE1_CHECK_DRIVE  Pass/fail checks for a phase1_test_drive run (test-drive cases D1-D5).
%
%   [checks, metrics] = phase1_check_drive(res, args)
%     res   result of phase1_test_drive (res.S = logged signals)
%     args  the phase1_test_drive options of the case ('Plan', 'EStopAt', 'ResetAt', 'RCLoss', ...)
%   Common checks: run ok, MANUAL by 3 s, no FAULT, no collision, no rejected SBUS frame, and speed and
%   yaw-rate tracking per plan segment (mean error over the segment after a 3 s settle, outside event
%   windows) within 0.05 m/s and 0.05 rad/s. E-stop: ESTOP within 0.2 s, standstill within 2 s, MANUAL again
%   within 1 s of the automatic re-arm. RC loss: CTRL_STOP within 0.2 s, never ESTOP, standstill within
%   2.5 s, MANUAL again within 1 s of the re-arm. Mode codes: ShaddadMode (3 MANUAL, 7 CTRL_STOP, 8 ESTOP, 9 FAULT).

o = local_args(args);
MANUAL = 3; CTRL_STOP = 7; ESTOP = 8; FAULT = 9;
checks = struct('id', {}, 'desc', {}, 'status', {}, 'value', {}, 'limit', {}, 'unit', {});
checks(end+1) = chk('run', 'simulation finished without error', res.ok, double(res.ok), '== 1', '');
if isempty(res.S)
    metrics = res.msg;
    return
end
S = res.S;
mt = S.telem.mode.t(:);  mv = double(S.telem.mode.v(:));
vt = S.truth.speed_fwd.t(:);  vv = double(S.truth.speed_fwd.v(:));
wt = S.truth.omega_B.t(:);  wv = double(S.truth.omega_B.v(:, 3));
tEnd = res.t_end;

% ---- common checks
tMan = first(mt, mv == MANUAL, 0);
checks(end+1) = chk('manual', 'MCU reaches MANUAL (arm at 2 s)', tMan <= 3.0, tMan, '<= 3.0', 's');
nFault = nnz(diff([0; mv == FAULT]) == 1);
checks(end+1) = chk('no_fault', 'MCU never enters FAULT', nFault == 0, nFault, '== 0', '');
nColl = double(S.kpi.n_coll.v(end));
checks(end+1) = chk('no_collision', 'no collision', nColl == 0, nColl, '== 0', '');
nBad = double(S.rc_dec.n_bad.v(end));
checks(end+1) = chk('sbus_rejected', 'no rejected SBUS frame (clean radio link)', nBad == 0, nBad, '== 0', 'frames');

% ---- event windows excluded from tracking (event start -> re-arm + 3 s settle)
ex = zeros(0, 2);
if ~isempty(o.EStopAt)
    if isempty(o.ResetAt), ex(end+1, :) = [o.EStopAt inf]; else, ex(end+1, :) = [o.EStopAt o.ResetAt + 3 + 3]; end
end
if ~isempty(o.RCLoss)
    ex(end+1, :) = [o.RCLoss(1) o.RCLoss(2) + 5 + 3];
end

% ---- tracking per plan segment
plan = sortrows(o.Plan, 1);
ev = 0; ew = 0; nSeg = 0;
for i = 1:size(plan, 1)
    a = plan(i, 1) + 3;
    if i < size(plan, 1), b = plan(i + 1, 1); else, b = tEnd; end
    for j = 1:size(ex, 1)                 % keep the longest part outside the event window
        if a < ex(j, 2) && b > ex(j, 1)
            if ex(j, 1) - a >= b - ex(j, 2), b = min(b, ex(j, 1)); else, a = max(a, ex(j, 2)); end
        end
    end
    if b - a < 2, continue, end
    nSeg = nSeg + 1;
    ev = max(ev, abs(mean(vv(vt >= a & vt < b)) - plan(i, 2)));
    ew = max(ew, abs(mean(wv(wt >= a & wt < b)) - plan(i, 3)));
end
checks(end+1) = chk('track_speed', sprintf('speed follows the plan (%d segments, mean error)', nSeg), ev <= 0.05, ev, '<= 0.05', 'm/s');
checks(end+1) = chk('track_yaw', sprintf('yaw rate follows the plan (%d segments, mean error)', nSeg), ew <= 0.05, ew, '<= 0.05', 'rad/s');

% ---- remote E-stop
if ~isempty(o.EStopAt)
    tE = o.EStopAt;
    dE = first(mt, mv == ESTOP, tE) - tE;
    checks(end+1) = chk('estop_entered', 'ESTOP after the CH12 switch', dE <= 0.2, dE, '<= 0.2', 's');
    dS = first(vt, abs(vv) < 0.05, tE + 0.05) - tE;
    checks(end+1) = chk('estop_standstill', 'standstill after the E-stop', dS <= 2.0, dS, '<= 2.0', 's');
    if ~isempty(o.ResetAt)
        tArm = o.ResetAt + 3;
        dR = first(mt, mv == MANUAL, tArm - 0.5) - tArm;
        checks(end+1) = chk('estop_rearm', 'MANUAL again after reset and re-arm', dR <= 1.0, dR, '<= 1.0', 's');
    end
end

% ---- RF link loss
if ~isempty(o.RCLoss)
    t1 = o.RCLoss(1);  t2 = o.RCLoss(2);
    dC = first(mt, mv == CTRL_STOP, t1) - t1;
    checks(end+1) = chk('rcloss_ctrl_stop', 'controlled stop after the link loss', dC <= 0.2, dC, '<= 0.2', 's');
    nE = nnz(mv == ESTOP);
    checks(end+1) = chk('rcloss_no_estop', 'RC loss never latches ESTOP', nE == 0, nE, '== 0', 'samples');
    dS = first(vt, abs(vv) < 0.05, t1 + 0.05) - t1;
    checks(end+1) = chk('rcloss_standstill', 'standstill after the link loss', dS <= 2.5, dS, '<= 2.5', 's');
    tArm = t2 + 5;
    dR = first(mt, mv == MANUAL, tArm - 0.5) - tArm;
    checks(end+1) = chk('rcloss_rearm', 'MANUAL again after the link returns and re-arm', dR <= 1.0, dR, '<= 1.0', 's');
end

pos = S.truth.pos_W.v;
soc = double(S.power_truth.SOC_pack_true.v);
metrics = sprintf('%.1f m, max %.2f m/s, SOC %.2f->%.2f %%, %d SBUS frames OK, RTF %.2f', ...
    sum(vecnorm(diff(pos(:, 1:2)), 2, 2)), max(abs(vv)), 100*soc(1), 100*soc(end), ...
    double(S.rc_dec.n_ok.v(end)), res.rtf);
end

% ------------------------------------------------------------------------------------------------
function o = local_args(args)
ip = inputParser;
ip.KeepUnmatched = true;
ip.addParameter('Plan', [0 0 0; 4 0.8 0; 14 0.8 0.3; 24 1.2 0; 34 0.5 -0.4; 44 0 0.5; 50 -0.4 0; 56 0 0]);
ip.addParameter('EStopAt', []);
ip.addParameter('ResetAt', []);
ip.addParameter('RCLoss', []);
ip.parse(args{:});
o = ip.Results;
end

function t = first(tt, cond, t0)
i = find(tt >= t0 & cond, 1);
if isempty(i), t = inf; else, t = tt(i); end
end

function s = chk(id, desc, ok, value, limit, unit)
if ok, st = 'PASS'; else, st = 'FAIL'; end
s = struct('id', id, 'desc', desc, 'status', st, 'value', double(value), 'limit', limit, 'unit', unit);
end
