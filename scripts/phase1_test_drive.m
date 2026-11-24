function res = phase1_test_drive(varargin)
%PHASE1_TEST_DRIVE  Drive the Shaddad UGV with the simulated RC transmitter (Phase 1) and plot the result.
%
%   res = phase1_test_drive()                      default demo drive (~62 s simulated, ~2-3 min wall)
%   res = phase1_test_drive('Plan', plan)          your own maneuver
%   res = phase1_test_drive(..., 'EStopAt', 20, 'ResetAt', 25)   remote E-stop (CH12) + reset (CH9)
%   res = phase1_test_drive(..., 'RCLoss', [20 23])               RF link lost 20-23 s (failsafe test)
%   res = phase1_test_drive(..., 'World', 1, 'Implement', 0, 'StopTime', T, 'Replay', true)
%
%   plan = N x 3 [t v w]: from time t [s] hold speed v [m/s] and yaw rate w [rad/s], + = left turn.
%   Manual limits: |v| <= 2.0 m/s, |w| <= 1.0 rad/s. Example:
%       plan = [0 0 0;  4 1.0 0;  15 1.0 0.4;  25 0 0];
%       res = phase1_test_drive('Plan', plan);
%
%   The run goes through the full chain: transmitter -> SBUS bytes -> STM32 SBUS decoder and RC mapping
%   -> mode manager / vehicle control -> CAN -> motor controllers -> PMSM + gearbox -> tracks on soil,
%   powered by the 48 V pack and BMS. Results: a figure, a printed summary and (Replay) the bird's-eye
%   replay. Logs are saved in work/results/D1_<time>.mat. Runs in Normal mode (Accelerator MEX files are
%   blocked by Windows Application Control on this PC).

ip = inputParser;
ip.KeepUnmatched = true;
ip.addParameter('Replay', true);
ip.parse(varargin{:});
args = namedargs2cell(ip.Unmatched);

ws = warning('off', 'all');
c = onCleanup(@() warning(ws));
res = run_scenario('D1', 'Args', args, 'Mode', 'normal');
if isempty(res.S)
    fprintf(2, 'Test drive failed: %s\n', res.msg);
    return
end
S = res.S;

%% ---- summary ----
modes = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', 'CTRL_STOP', 'ESTOP', 'FAULT'};
m = S.telem.mode;  mv = double(m.v(:));
i = [1; find(diff(mv) ~= 0) + 1];
fprintf('\n=== Phase 1 test drive: %.0f s simulated in %.0f s (%.2fx real time) ===\n', res.t_end, res.wall_s, res.rtf);
fprintf('MCU modes : %s\n', strjoin(arrayfun(@(k) sprintf('%s@%.2fs', modes{mv(k) + 1}, m.t(k)), i', ...
    'UniformOutput', false), ' -> '));
pos = S.truth.pos_W.v;
fprintf('Distance  : %.1f m driven, max speed %.2f m/s\n', sum(vecnorm(diff(pos(:, 1:2)), 2, 2)), ...
    max(abs(double(S.truth.speed_fwd.v))));
soc = double(S.power_truth.SOC_pack_true.v);
fprintf('Battery   : SOC %.2f %% -> %.2f %%, %.1f Wh used, peak current %.0f A\n', 100*soc(1), 100*soc(end), ...
    double(S.power_truth.E_out_Wh.v(end)), max(double(S.power_truth.I_pack_A.v)));
ll = S.rc_dec.rc_link_lost;                  % the link comes up at ~0.6 s (transmitter on at 0.5 s)
fprintf('SBUS      : %d frames OK, %d rejected, link lost after arming: %s\n', double(S.rc_dec.n_ok.v(end)), ...
    double(S.rc_dec.n_bad.v(end)), mat2str(any(double(ll.v(ll.t >= 2)) ~= 0)));
fprintf('Safety    : collisions %d, max motor torque limit %.1f N m\n', double(S.kpi.n_coll.v(end)), ...
    max(double(S.telem.tau_lim_mot.v(:))));
fprintf('Log file  : %s  (all signals: plot_run(res))\n\n', res.file);

%% ---- figure ----
f = figure('Name', 'Shaddad Phase 1 test drive', 'Color', 'w', 'Position', [80 80 1250 760]);
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Shaddad UGV - Phase 1 (SBUS remote control) test drive');

nexttile; hold on; grid on;
stairs(S.telem.v_target.t, double(S.telem.v_target.v), 'LineWidth', 1.2);
plot(S.truth.speed_fwd.t, double(S.truth.speed_fwd.v), 'LineWidth', 1.2);
ylabel('speed [m/s]'); legend('RC command (MCU)', 'actual (truth)', 'Location', 'best'); title('Forward speed');

nexttile; hold on; grid on;
stairs(S.telem.w_target.t, double(S.telem.w_target.v), 'LineWidth', 1.2);
plot(S.truth.omega_B.t, double(S.truth.omega_B.v(:, 3)), 'LineWidth', 1.2);
ylabel('yaw rate [rad/s]'); legend('RC command (MCU)', 'actual (truth)', 'Location', 'best'); title('Yaw rate (+ = left)');

nexttile([2 1]); hold on; grid on; axis equal;
plot(pos(:, 1), pos(:, 2), 'LineWidth', 1.5);
plot(pos(1, 1), pos(1, 2), 'go', 'MarkerFaceColor', 'g'); plot(pos(end, 1), pos(end, 2), 'rs', 'MarkerFaceColor', 'r');
xlabel('East [m]'); ylabel('North [m]'); legend('path', 'start', 'end', 'Location', 'best'); title('Path on the field');

nexttile; hold on; grid on;
vb = S.truth.vBelt_LR;
plot(vb.t, double(vb.v(:, 1)), vb.t, double(vb.v(:, 2)), 'LineWidth', 1.1);
ylabel('track speed [m/s]'); legend('left', 'right', 'Location', 'best'); title('Track belt speeds');

nexttile; grid on;
yyaxis left;  plot(S.power_truth.I_pack_A.t, double(S.power_truth.I_pack_A.v), 'LineWidth', 1.1); ylabel('pack current [A]');
yyaxis right; stairs(m.t, mv, 'LineWidth', 1.2); ylabel('MCU mode');
yticks(0:9); yticklabels(modes); ylim([-0.5 9.5]); set(gca, 'TickLabelInterpreter', 'none');
xlabel('time [s]'); title('Battery current and MCU mode');

%% ---- bird's-eye replay ----
if ip.Results.Replay
    try
        shaddad_replay_birdseye(res.file);
    catch tmp_err
        fprintf(2, 'Replay not shown: %s\n', tmp_err.message);
    end
end
end
