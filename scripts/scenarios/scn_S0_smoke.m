function [in, crit] = scn_S0_smoke(P, varargin)
%SCN_S0_SMOKE  S0 Smoke (Phase 1): 10 s, all inputs zero (key on, TX off), flat_test (architecture 12).
%
%   [in, crit] = scn_S0_smoke(P)
%   [in, crit] = scn_S0_smoke(P, 'StopTime', T)
%   Pass criteria: position drift < 1e-3 m; |dSOC| < 1e-4; no NaN / Inf; automatic re-arm (ICD 6, D30):
%   DO0 high <= 0.05 s, DO1 pulse 50 +/- 2 ms, Safety_OK and Safety_HV_OK true <= 0.2 s; HV_Ready true
%   <= 1.0 s; MCU mode STANDBY <= 1.5 s and at 10 s; no class-A fault bit at 10 s.

ip = inputParser;
ip.addParameter('StopTime', 10);
ip.parse(varargin{:});
T = ip.Results.StopTime;
if P.world.id ~= 1
    P = shaddad_params('World', 1, 'Implement', 0, 'Phase', 1, 'Assign', false);
end
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', 0, 'VC_VIZ', 0, 'VC_OPSRC', 1);
in = scn_base_input(P, 'S0', 'StopTime', T, 'Input', struct(), 'VC', VC);
in = scn_logging('apply', in, P, T);

M = @scn_metrics;
STANDBY = 2;
maskA = double(P.electronics.safety.maskA);
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c('S0.drift', 'position drift |pos(t) - pos(0)|', 'truth.pos_W', ...
    @(S) max(sqrt(sum((S.truth.pos_W.v(:, 1:2) - S.truth.pos_W.v(1, 1:2)).^2, 2))), '<', 1e-3, 'm');
crit(end+1) = local_c('S0.dsoc', '|SOC(T) - SOC(0)|', 'power_truth.SOC_pack_true', ...
    @(S) abs(S.power_truth.SOC_pack_true.v(end) - S.power_truth.SOC_pack_true.v(1)), '<', 1e-4, '1');
crit(end+1) = local_c('S0.finite', 'no NaN / Inf (truth, power truth, kpi.stop_reason ~= 5)', 'truth, kpi', ...
    @(S) all(isfinite(S.truth.pos_W.v(:))) && all(isfinite(S.truth.eul.v(:))) && ...
         all(isfinite(S.power_truth.SOC_pack_true.v(:))) && all(S.kpi.stop_reason.v ~= 5), 'true', 1, '');
crit(end+1) = local_c('S0.do0', 'DO0 MCU_SAFETY_OK high', 'mcu_out1k.do_word b0', ...
    @(S) M('first', S.mcu_out1k.do_word, @(v) M('bit', v, 0), 0), '<=', 0.05, 's');
crit(end+1) = local_c('S0.do1', 'DO1 SAFETY_RESET_REQ pulse width', 'mcu_out1k.do_word b1', ...
    @(S) M('pulse', S.mcu_out1k.do_word, @(v) M('bit', v, 1), 0), 'in', [0.048 0.052], 's');
crit(end+1) = local_c('S0.safety_ok', 'Safety_OK true', 'power_status.Safety_OK', ...
    @(S) M('first', S.power_status.Safety_OK, @(v) v ~= 0, 0), '<=', 0.2, 's');
crit(end+1) = local_c('S0.safety_hv_ok', 'Safety_HV_OK true', 'power_status.Safety_HV_OK', ...
    @(S) M('first', S.power_status.Safety_HV_OK, @(v) v ~= 0, 0), '<=', 0.2, 's');
crit(end+1) = local_c('S0.hv_ready', 'BMS HV_Ready true', 'bms_telem.HV_Ready', ...
    @(S) M('first', S.bms_telem.HV_Ready, @(v) v ~= 0, 0), '<=', 1.0, 's');
crit(end+1) = local_c('S0.standby', 'MCU mode STANDBY reached', 'telem.mode', ...
    @(S) M('first', S.telem.mode, @(v) v == STANDBY, 0), '<=', 1.5, 's');
crit(end+1) = local_c('S0.standby_end', 'MCU mode at 10 s (2 = STANDBY)', 'telem.mode', ...
    @(S) double(S.telem.mode.v(end)), '==', STANDBY, '');
crit(end+1) = local_c('S0.classA', 'class-A fault bits at 10 s (fault_word AND maskA)', 'telem.fault_word', ...
    @(S) double(bitand(uint32(S.telem.fault_word.v(end)), uint32(maskA))), '==', 0, '');
end

function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end
