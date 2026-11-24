function files = real_mbd_figs(which, d1file)
%REAL_MBD_FIGS  Engineering figures of the model-based design report of the real robot (figures M20..M28).
%
%   files = real_mbd_figs()                 every figure
%   files = real_mbd_figs({'M21', 'M23'})   some figures
%   files = real_mbd_figs(which, d1file)    operating points of a saved run (default: newest work/results/D1_*.mat)
%
%   Figures (docs/report/figs, PNG at 200 dpi) are computed from the parameters (shaddad_params), the saved
%   test drive and the gauge-factor identification of 2026-10-27:
%     M20 mass breakdown and configurations      M21 motor torque-speed envelope (Movematic 750 W)
%     M22 motor + ESC efficiency map with the D1 operating points
%     M23 Molicel P42A cell model and the 13S15P pack      M25 gauge factor chi per soil (old vs real geometry)
%     M24 ESC speed loop of the four-motor drive (D64): sensitivity and vehicle-speed step, scaled vs re-tuned gains
%     M26 dimensioned top and side view           M27 architecture of the real robot as modelled
%     M28 model-based design workflow of the project
%     M29 S8a in ESTOP with both brakes engaged: former friction element vs elasto-plastic brake (D64, newest S8a log)
%     M42 / M43 test drive D1 on the real-robot model: motion and energy (compact pages for the report)
%     M44 / M45 Phase 1 deviations: S2 tilt warning and ramp speed, S6a controlled-stop deceleration (newest logs)

all = {'M20', 'M21', 'M22', 'M23', 'M24', 'M25', 'M26', 'M27', 'M28', 'M29', 'M42', 'M43', 'M44', 'M45'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which), which = {which}; end
root = char(currentProject().RootFolder);
out = fullfile(root, 'docs', 'report', 'figs');
P = shaddad_params('Assign', false);
if nargin < 2 || isempty(d1file)
    L = dir(fullfile(root, 'work', 'results', 'D1_*.mat'));
    [~, i] = max([L.datenum]);
    d1file = fullfile(L(i).folder, L(i).name);
end
files = {};
for k = 1:numel(which)
    switch which{k}
        case 'M20', f = local_m20(P);
        case 'M21', f = local_m21(P);
        case 'M22', f = local_m22(P, d1file);
        case 'M23', f = local_m23(P);
        case 'M24', f = local_m24(P);
        case 'M25', f = local_m25(P);
        case 'M26', f = local_m26(P);
        case 'M27', f = local_m27(P);
        case 'M28', f = local_m28();
        case 'M29', f = local_m29(root);
        case 'M42', f = local_m42(P, d1file);
        case 'M43', f = local_m43(P, d1file);
        case 'M44', f = local_m44(P, local_newest(root, 'S2_*.mat'));
        case 'M45', f = local_m45(P, local_newest(root, 'S6a_*.mat'));
        otherwise, error('real_mbd_figs:which', 'unknown figure %s', which{k});
    end
    name = fullfile(out, [which{k} '_' f.UserData '.png']);
    exportgraphics(f, name, 'Resolution', 200, 'BackgroundColor', 'white', 'Padding', 12);
    close(f);
    files{end+1} = name; %#ok<AGROW>
    fprintf('%s\n', name);
end
end

% =========================================================================================== figures
function f = local_m20(P)
f = local_fig(1100, 480, 'mass_breakdown');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
C = P.structure.mass.components;
names = {'Chassis frame', 'Body panels, deck rail', 'Track assembly left', 'Track assembly right', ...
    'Drive unit front-left', 'Drive unit rear-left', 'Drive unit front-right', 'Drive unit rear-right', ...
    'Battery pack 13S15P', 'Electronics box', 'Sensor mast (Phase 2)', 'Cooling, misc.', 'Front coupler', 'Rear hitch'};
m = C(:, 1);
[ms, i] = sort(m, 'ascend');
ax = nexttile(tl);
b = barh(ax, ms, 'FaceColor', 'flat', 'EdgeColor', 'none');
b.CData = repmat([0.66 0.6 0.8], numel(ms), 1);
b.CData(strcmp(names(i), 'Battery pack 13S15P'), :) = [0.17 0.18 0.49];
yticks(ax, 1:numel(ms)); yticklabels(ax, names(i)); grid(ax, 'on'); box(ax, 'on');
xlabel(ax, 'mass [kg]'); xlim(ax, [0 1.2*max(ms)]);
for j = 1:numel(ms), text(ax, ms(j) + 1, j, sprintf('%.1f', ms(j)), 'FontSize', 9); end
title(ax, sprintf('(a) Mass table: %.1f kg (as built 220 kg)', sum(m)), 'FontWeight', 'normal');
text(ax, 0.98, 0.04, {'dark: battery, from the cell count and box [DATA]', 'light: split [EST]; drive units 1.0 kW'}, ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', [0.3 0.3 0.35]);
ax = nexttile(tl);
cfg = {'Robot', 0, 1; 'Sprayer, empty tank', 3, 0; 'Sprayer, full 40 L', 3, 1; 'Mower deck', 1, 1; 'Tiller', 2, 1};
mc = zeros(1, size(cfg, 1));
for k = 1:size(cfg, 1)
    S = param_structure(struct('implement', cfg{k, 2}, 'tankFill0', cfg{k, 3}, 'batteryMass_kg', P.power.pack.mass_kg, ...
        'batteryPos_m', P.power.pack.pos_FRD), P.world, P.frames.g);
    mc(k) = S.mp.m0;
end
b = bar(ax, mc, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'none');
b.CData = [0.17 0.18 0.49; 0.43 0.31 0.64; 0.43 0.31 0.64; 0.75 0.75 0.78; 0.75 0.75 0.78];
xticks(ax, 1:numel(mc)); xticklabels(ax, cfg(:, 1)); grid(ax, 'on'); box(ax, 'on');
ylabel(ax, 'total mass m_0 [kg]'); ylim(ax, [0 380]);
for k = 1:numel(mc), text(ax, k, mc(k) + 8, sprintf('%.0f kg', mc(k)), 'HorizontalAlignment', 'center', 'FontSize', 10); end
yline(ax, 270, '--', 'spec sheet: 220 + 50 kg', 'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom', ...
    'Color', [0.65 0.29 0.61], 'FontSize', 9);
title(ax, '(b) Configurations', 'FontWeight', 'normal');
text(ax, 0.5, 362, 'grey: mower and tiller are model implements, not on the real robot', 'HorizontalAlignment', 'left', ...
    'FontSize', 8, 'Color', [0.3 0.3 0.35]);
end

function f = local_m21(P)
% motor sizing per unit: as-built Movematic 750 W and the proposed 1.0 kW unit (48 V envelopes, continuous at
% 25 and 45 degC) with the duty points of motor_sizing (farm soil and wet loam)
f = local_fig(1000, 600, 'motor_sizing');
g = P.drive.gear; S = motor_sizing(shaddad_params('Implement', 3, 'Assign', false));
ax = axes(f, 'Position', [0.08 0.11 0.83 0.72]); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
U = {param_motor_unit('movematic750'), [0.55 0.55 0.75]; param_motor_unit('kw1'), [0.17 0.18 0.49]};
lbl = {'750 W as built', '1.0 kW proposed'};
for k = 1:2
    u = U{k, 1}; c = U{k, 2};
    plot(ax, u.w_t_rpm, u.T_pk_nom, '-', 'LineWidth', 1.8, 'Color', c, 'DisplayName', [lbl{k} ': peak (48 V)']);
    plot(ax, u.w_t_rpm, u.T_cont_nom, '--', 'LineWidth', 1.6, 'Color', c, 'DisplayName', [lbl{k} ': continuous']);
    plot(ax, u.w_t_rpm, S.k45*u.T_cont_nom, ':', 'LineWidth', 1.4, 'Color', c, 'DisplayName', [lbl{k} ': continuous at 45 degC']);
end
plot(ax, 3000, 2.39, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.95 0.55 0.1], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Movematic nameplate 750 W, 3000 rpm [DATA]');
p = S.points;
plot(ax, p.n_rpm, p.T_farm, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.65 0.29 0.61], 'MarkerEdgeColor', 'k', 'DisplayName', 'duty points, farm soil');
plot(ax, p.n_rpm, p.T_wet, 's', 'MarkerSize', 7, 'MarkerFaceColor', [0.95 0.75 0.3], 'MarkerEdgeColor', 'k', 'DisplayName', 'duty points, wet loam');
for k = 1:height(p)
    text(ax, p.n_rpm(k) + 45, max(p.T_farm(k), p.T_wet(k)) + 0.32, p.name{k}, 'FontSize', 8, 'Color', [0.3 0.3 0.35]);
end
yline(ax, S.rms_wet, '-.', sprintf('cycle RMS, wet loam %.2f N m', S.rms_wet), 'Color', [0.65 0.29 0.61], ...
    'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8.5, 'HandleVisibility', 'off');
xlim(ax, [0 4200]); ylim(ax, [0 10.5]);
xlabel(ax, 'motor speed [rpm]'); ylabel(ax, 'torque per motor [N m]');
legend(ax, 'Location', 'northeast', 'FontSize', 8.5);
ax2 = axes(f, 'Position', ax.Position, 'XAxisLocation', 'top', 'YAxisLocation', 'right', 'Color', 'none', ...
    'XLim', ax.XLim*pi/30/g.k_v2w, 'YLim', ax.YLim*g.N*g.eta, 'Box', 'off');
xlabel(ax2, 'track speed [m/s]'); ylabel(ax2, 'sprocket torque per motor [N m]');
ax2.XGrid = 'off'; ax2.YGrid = 'off';
end

function f = local_m22(P, d1file)
f = local_fig(1000, 560, 'motor_efficiency');
m = P.drive.motor; n = P.drive.n_mot_side; u = P.drive.motor_unit;   % side equivalent of n units; plot one unit
Fi = griddedInterpolant({m.w_eff_rpm(:), m.T_eff(:)}, m.loss_T1, 'linear');
[W, T] = ndgrid(linspace(200, 4000, 160), linspace(0.1, 1.06*u.T_peak, 120));
Pm = T.*W*pi/30;
eta = 100*Pm./(Pm + Fi(W, n*T)/n);           % one unit at T = the side equivalent at n*T, losses / n
Tmax = interp1(m.w_t_rpm, m.T_pk_2D(:, m.vdc_t == 48)/n, W, 'linear', 0);
eta(T > Tmax) = NaN;
ax = axes(f, 'Position', [0.08 0.11 0.76 0.78]); hold(ax, 'on'); box(ax, 'on');
[~, hc] = contourf(ax, W, T, eta, [0 50 60 70 75 80 85 88 90 92 94], 'LineColor', [0.3 0.3 0.3], 'HandleVisibility', 'off');
clabel([], hc, 'FontSize', 8, 'Color', 'w');
colormap(ax, parula); clim(ax, [50 94]); cb = colorbar(ax);
cb.Label.String = 'motor + ESC efficiency [%] (model loss map at 25 degC; darkest band < 50 %)';
plot(ax, m.w_t_rpm, m.T_pk_2D(:, m.vdc_t == 48)/n, 'k-', 'LineWidth', 1.8, 'DisplayName', 'peak envelope 48 V');
plot(ax, m.w_t_rpm, m.T_cont_nom/n, 'k--', 'LineWidth', 1.4, 'DisplayName', 'continuous');
try
    r = load(d1file); S = r.S;                % side torques of the equivalent drive -> per motor
    w1 = abs(double(S.drive_truth.L.w_m.v))*30/pi; t1 = abs(double(S.drive_truth.L.tau_m.v))/n;
    w2 = abs(double(S.drive_truth.R.w_m.v))*30/pi; t2 = abs(double(S.drive_truth.R.tau_m.v))/n;
    ww = [w1; w2]; tt = [t1; t2]; k = find(ww > 50 & tt > 0.05); k = k(1:20:end);   % every 20th point of the run
    [~, nm] = fileparts(d1file);
    plot(ax, ww(k), tt(k), '.', 'Color', [0.85 0.1 0.1], 'MarkerSize', 6, ...
        'DisplayName', sprintf('operating points of %s (all motors)', strrep(nm, '_', '\_')));
catch
end
xlim(ax, [0 4000]); ylim(ax, [0 1.06*u.T_peak]);
xlabel(ax, 'motor speed [rpm]'); ylabel(ax, 'motor torque [N m]');
legend(ax, 'Location', 'northeast', 'FontSize', 9);
title(ax, '1.0 kW traction motor (one of four): modelled efficiency (loss map [EST]) and the test-drive operating points', 'FontWeight', 'normal');
end

function f = local_m23(P)
f = local_fig(1150, 480, 'cell_pack_model');
c = P.power.cell; pk = P.power.pack; b = P.power.bms;
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
soc = c.SOC_bp*100;
plot(ax, soc, c.OCV_tab(:, 1), '-', 'Color', [0.55 0.55 0.85], 'LineWidth', 1.4, 'DisplayName', 'OCV 0 degC');
plot(ax, soc, c.OCV_tab(:, 2), '-', 'Color', [0.17 0.18 0.49], 'LineWidth', 2, 'DisplayName', 'OCV 25 degC');
plot(ax, soc, c.OCV_tab(:, 3), '-', 'Color', [0.65 0.29 0.61], 'LineWidth', 1.4, 'DisplayName', 'OCV 45 degC');
plot(ax, soc, c.OCV25(:) - c.Q_Ah*(c.R0_25(:) + c.R1_25(:)), '--', 'Color', [0.17 0.18 0.49], 'LineWidth', 1.4, ...
    'DisplayName', 'terminal, 1C (4.2 A) discharge, 25 degC');
yline(ax, c.V_cv, ':', sprintf('CV %.2f V', c.V_cv), 'FontSize', 8, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(ax, b.V_uv, ':', sprintf('BMS UV %.2f V', b.V_uv), 'FontSize', 8, 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
yline(ax, c.V_min_abs, ':', sprintf('cell min %.2f V', c.V_min_abs), 'FontSize', 8, 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
xlabel(ax, 'state of charge [%]'); ylabel(ax, 'cell voltage [V]'); ylim(ax, [2.4 4.35]);
lg = legend(ax, 'FontSize', 8); drawnow;
lg.Position(1:2) = [ax.Position(1) + 0.97*ax.Position(3) - lg.Position(3), ax.Position(2) + 0.30*ax.Position(4)];
yyaxis(ax, 'right'); ylim(ax, [2.4 4.35]*pk.Ns); ylabel(ax, sprintf('pack voltage (%dS) [V]', pk.Ns));
ax.YAxis(2).Color = [0.3 0.3 0.35];
title(ax, sprintf('(a) Molicel P42A open-circuit voltage; pack %.1f V nominal, %.1f V full', pk.V_nom, pk.V_oper_max), 'FontWeight', 'normal');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, soc, 1e3*c.R0_25, '-o', 'Color', [0.17 0.18 0.49], 'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', 'R0 (instantaneous) 25 degC');
plot(ax, soc, 1e3*c.R1_25, '-s', 'Color', [0.65 0.29 0.61], 'LineWidth', 1.6, 'MarkerSize', 4, ...
    'DisplayName', sprintf('R1 (polarisation, tau_1 = %g s)', c.tau1_25));
plot(ax, soc, 1e3*c.R0_25*c.fR0(1), '--', 'Color', [0.55 0.55 0.85], 'LineWidth', 1.2, 'DisplayName', 'R0 at 0 degC');
xlabel(ax, 'state of charge [%]'); ylabel(ax, 'cell resistance [m\Omega]'); ylim(ax, [0 60]);
legend(ax, 'Location', 'north', 'FontSize', 8);
yyaxis(ax, 'right'); ylim(ax, [0 60]*pk.Ns/pk.Np); ylabel(ax, sprintf('pack resistance (%dS%dP) [m\\Omega]', pk.Ns, pk.Np));
ax.YAxis(2).Color = [0.3 0.3 0.35];
title(ax, sprintf('(b) 1-RC cell resistances [EST]; pack R0 about %.1f m\\Omega at 50 %% SOC', ...
    1e3*pk.Ns*c.R0_25(c.SOC_bp == 0.5)/pk.Np), 'FontWeight', 'normal');
end

function f = local_m24(P)
% ESC speed loop of the four-motor drive (D64) on the two-mass model (tb_drive_2mass_lti, softest shear spring,
% full sprayer): D50 gains scaled with J_eq vs the re-tuned gains of P
f = local_fig(1100, 440, 'speed_loop');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
b2 = P.drive.bench2m; kx = b2.k_x(1); ms = b2.m_side(end); e = P.drive.esc;
Po = P; Po.drive.esc.Kp_w = 0.13*e.J_eq/10.18e-3; Po.drive.esc.Ki_w = 0.39*e.J_eq/10.18e-3;
G = {Po, [0.85 0.33 0.10], sprintf('D50 gains scaled with J_{eq} (Kp %.3f, Ki %.3f)', Po.drive.esc.Kp_w, Po.drive.esc.Ki_w); ...
     P, [0.17 0.18 0.49], sprintf('re-tuned, D64 (Kp %.3f, Ki %.3f)', e.Kp_w, e.Ki_w)};
w = logspace(0, log10(pi/P.drive.Ts_esc), 600);
ax1 = local_tile(tl, sprintf('(a) sensitivity |S|, shear spring %.0f kN/m, %.0f kg per side', kx/1e3, ms), '|S(j\omega)|');
ax2 = local_tile(tl, '(b) 5 rad/s motor-speed step at 100 rad/s: bench and linear model', 'motor speed change [rad/s]');
% the T21 validation step on the two-mass bench (tb_drive_2mass) and the linear loop model w_ref -> w_meas
tt = (0:0.001:4)'; wr = 100*min(tt/1.5, 1); wr(tt >= 3) = 105;
cs = tb_drive_bus_ts('Bus_DriveSideCmd', tt, struct('enable', 1, 'mode', 1, 'w_ref', wr, 'tau_lim_mot', P.drive.motor.T_peak, ...
    'tau_lim_reg', P.drive.motor.T_peak, 'alive', mod(floor(tt/0.01), 256)));
ws = warning('off', 'all'); cw = onCleanup(@() warning(ws));
lbl = {'D50 scaled:', 'D64:'};
for k = 1:2
    R = tb_drive_2mass_lti(G{k, 1}, kx, ms);
    plot(ax1, w, squeeze(abs(freqresp(R.S, w))), '-', 'Color', G{k, 2}, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s: Ms %.2f', G{k, 3}, R.Ms));
    so = tb_drive_run('tb_drive_2mass', 3.6, struct('tb_cmd', cs, 'tb_kx', kx, 'tb_cx', b2.tau_d*kx, 'tb_m', ms), ...
        struct('P', G{k, 1}));
    tw = so.fb.w_m.Time; yw = double(so.fb.w_m.Data); y0 = mean(yw(tw >= 2.9 & tw < 3)); m = tw >= 2.95;
    plot(ax2, tw(m) - 3, yw(m) - y0, '-', 'Color', G{k, 2}, 'LineWidth', 1.6, ...
        'DisplayName', sprintf('%s bench (model: PM %.0f deg, GM %.1f dB, zeta %.2f)', lbl{k}, R.PM, R.GM_dB, R.zeta_min));
    tk = (0:P.drive.Ts_esc:0.6)';
    plot(ax2, tk, 5*step(R.Tr, tk), '--', 'Color', G{k, 2}, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
plot(ax2, NaN, NaN, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'DisplayName', 'linear model (dashed)');
yline(ax1, b2.Ms_max, '--', sprintf('T21 bound %.1f', b2.Ms_max), 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off', ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8.5);
yline(ax1, 1.43, ':', 'D50 design target 1.43', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off', ...
    'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8.5);
set(ax1, 'XScale', 'log'); xlim(ax1, w([1 end])); ylim(ax1, [0 1.9]); xlabel(ax1, 'frequency [rad/s]');
xlim(ax2, [-0.05 0.6]); ylim(ax2, [-0.5 7.5]);
legend(ax1, 'Location', 'southeast', 'FontSize', 8.5); legend(ax2, 'Location', 'southeast', 'FontSize', 8.5);
end

function f = local_m29(root)
% S8a (front hardware E-stop at 10 s, ESTOP with STO and both brakes engaged until the reset at 17 s): motor speeds
% with the former Rotational Friction brake (Full suite of 2026-10-27 18:04) and the elasto-plastic brake (D64,
% newest S8a log)
f = local_fig(1100, 440, 'brake_estop');
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
G = {fullfile(root, 'work', 'results', 'S8a_20261027_182145.mat'), '(a) former friction element (stick-slip at the 1 ms step)'; ...
     local_newest(root, 'S8a_*.mat'), '(b) elasto-plastic brake (D64)'};
col = {[0.17 0.18 0.49], [0.85 0.33 0.10]}; yl = [-500 3600];
for k = 1:2
    S = load(G{k, 1}).S;
    [t, wl] = local_sg(S, 'drive_truth.L.w_m'); [~, wr] = local_sg(S, 'drive_truth.R.w_m');
    [tm, md] = local_sg(S, 'telem.mode');
    te = tm(find(md == 8, 1)); tx = tm(find(tm > te & md ~= 8, 1));
    ax = local_tile(tl, G{k, 2}, 'motor speed [rad/s]');
    local_band(ax, t, t >= te & t < tx, yl);
    plot(ax, t, wl, '-', 'Color', col{1}, 'LineWidth', 1.0, 'DisplayName', 'left drive');
    plot(ax, t, wr, '-', 'Color', col{2}, 'LineWidth', 1.0, 'DisplayName', 'right drive');
    m = t > te + 0.5 & t < tx;
    text(ax, 0.6, 3300, sprintf('ESTOP %.1f-%.1f s (shaded): max |w_m| %.1f rad/s', te, tx, max(abs([wl(m); wr(m)]))), ...
        'FontSize', 9.5, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.25]);
    text(ax, 0.6, 2950, sprintf('MCU mode at the end of the run: %s', char(ShaddadMode(md(end)))), 'FontSize', 9.5, ...
        'Color', [0.2 0.2 0.25]);
    xlim(ax, [0 t(end)]); ylim(ax, yl);
    legend(ax, 'Location', 'east', 'FontSize', 8.5);
end
end

function f = local_m25(P)
f = local_fig(1000, 480, 'chi_soils');
soils = {'sand', 'sandy loam (farm)', 'wet loam', 'greenhouse', 'asphalt'};
chiOld = [1.3458 1.3361 1.4219 1.2800 1.2788];          % former geometry (B 0.85 m, L 1.00 m), 2026-10-20
chiNew = P.structure.track.chi_soil;                      % real geometry (B 0.60 m, L 0.75 m), identified 2026-10-27
chiSpot = [1.5056 1.4355 1.6220 1.3655 1.3490];          % spot turn, same run (structure_identify_chi)
ax = axes(f, 'Position', [0.08 0.18 0.88 0.72]); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
b = bar(ax, [chiOld(:) chiNew(:) chiSpot(:)], 0.8, 'EdgeColor', 'none');
b(1).FaceColor = [0.75 0.75 0.78]; b(2).FaceColor = [0.17 0.18 0.49]; b(3).FaceColor = [0.65 0.29 0.61];
xticks(ax, 1:5); xticklabels(ax, soils); ylabel(ax, 'gauge factor \chi = B_{eff} / B'); ylim(ax, [1 1.75]);
legend(ax, {'former model geometry, R = 2 m turn', 'real robot geometry, R = 2 m turn (used by the MCU, EKF, GNC)', ...
    'real robot geometry, spot turn'}, 'Location', 'northwest', 'FontSize', 8.5);
for k = 1:5, text(ax, k, chiNew(k) + 0.03, sprintf('%.3f', chiNew(k)), 'HorizontalAlignment', 'center', 'FontSize', 8.5, 'Color', [0.17 0.18 0.49]); end
title(ax, 'Skid-steer gauge factor identified with the HiFi track model (structure\_identify\_chi, 0.5 m/s)', 'FontWeight', 'normal');
end

function f = local_m26(P)
f = local_fig(1150, 560, 'dimensions');
gm = P.structure.geom; rs = P.structure.r_sprocket; mp = P.structure.mp; pk = P.power.pack;
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nv = [0.17 0.18 0.49]; mg = [0.65 0.29 0.61]; gy = [0.35 0.35 0.38];
Ltr = 0.94; r = 0.13; a = Ltr/2 - r;                      % track loop of the 3-D view (side outline)
% ---- side view (x forward to the right, z up)
ax = nexttile(tl); hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');
plot(ax, [-0.75 0.75], [0 0], 'Color', [0.6 0.5 0.35], 'LineWidth', 2);
th = linspace(-pi/2, pi/2, 40);
xs = [a + r*cos(th), -a + r*cos(th + pi)]; zs = [r + r*sin(th), r + r*sin(th + pi)];
patch(ax, [-0.45 0.45 0.45 -0.45], [0.12 0.12 0.30 0.30], [0.25 0.25 0.27], 'EdgeColor', 'none');
patch(ax, [-0.48 0.42 0.48 0.48 0.44 -0.48], [0.30 0.30 0.34 0.44 0.47 0.47], [0.86 0.13 0.1], 'EdgeColor', 'k');
patch(ax, [xs xs(1)], [zs zs(1)], [0.15 0.15 0.16], 'EdgeColor', 'k');
for xc = [-a a], local_circle(ax, xc, r, rs, [0.65 0.66 0.7]); end
for xc = linspace(-a + 0.08, a - 0.08, gm.nRoadWheels), local_circle(ax, xc, gm.r_roadwheel + 0.005, gm.r_roadwheel, [0.9 0.9 0.92]); end
patch(ax, [-0.5 0.5 0.5 -0.5], [0.47 0.47 0.49 0.49], [0.08 0.08 0.09], 'EdgeColor', 'none');
for xx = [-0.47 0.47], plot(ax, [xx xx], [0.49 0.57], 'k', 'LineWidth', 2.5); end
plot(ax, [-0.49 0.49], [0.565 0.565], 'k', 'LineWidth', 2.5);
bx = pk.pos_FRD(1) + [-1 1]*pk.dims(1)/2; bz = -pk.pos_FRD(3) + [-1 1]*pk.dims(3)/2;
patch(ax, bx([1 2 2 1]), bz([1 1 2 2]), [1 1 1], 'FaceAlpha', 0.15, 'EdgeColor', [1 0.85 0.2], 'LineStyle', '--', 'LineWidth', 1.4);
text(ax, mean(bx), bz(2) + 0.02, 'battery 13S15P', 'HorizontalAlignment', 'center', 'FontSize', 7.5, 'Color', [0.95 0.8 0.1]);
cgx = mp.rcg0(1); cgz = -mp.rcg0(3);
local_cg(ax, cgx, cgz); text(ax, cgx + 0.04, cgz - 0.035, sprintf('CG, %.0f kg (h %.2f m)', mp.m0, cgz), 'FontSize', 8, 'Color', 'w');
local_dim(ax, [-0.5 0.5], [0.66 0.66], sprintf('%.2f m body / deck length [EST]', gm.L_overall), nv, 'h');
local_dim(ax, [-gm.L/2 gm.L/2], [-0.06 -0.06], sprintf('%.2f m contact length [EST]', gm.L), mg, 'hb');
local_dim(ax, [0.62 0.62], [0 gm.H_body], sprintf('%.2f m [DATA]', gm.H_body), nv, 'v');
local_dim(ax, [-0.56 -0.56], [0 gm.groundClearance], sprintf('clearance\n%.2f m [EST]', gm.groundClearance), gy, 'vl');
plot(ax, [-0.6 -a - 0.04], [0.36 r + 0.06], '-', 'Color', gy, 'LineWidth', 0.8);
text(ax, -0.61, 0.37, sprintf('sprocket / idler\nr_s %.3f m [EST]', rs), 'HorizontalAlignment', 'right', 'FontSize', 8, 'Color', gy);
title(ax, '(a) Side view (x forward to the right)', 'FontWeight', 'normal');
xlim(ax, [-1.0 0.8]); ylim(ax, [-0.2 0.75]);
% ---- top view (x forward to the right, y left up)
ax = nexttile(tl); hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');
patch(ax, [-0.5 0.5 0.5 -0.5], [-0.39 -0.39 0.39 0.39], [0.93 0.93 0.95], 'EdgeColor', [0.4 0.4 0.42]);
for s = [-1 1]
    yc = s*gm.B/2;
    patch(ax, [-Ltr/2 Ltr/2 Ltr/2 -Ltr/2], yc + [-1 -1 1 1]*gm.b/2, [0.15 0.15 0.16], 'FaceAlpha', 0.85, 'EdgeColor', 'k');
    plot(ax, [-gm.L/2 gm.L/2], [yc yc], ':', 'Color', mg, 'LineWidth', 1.6);
end
patch(ax, bx([1 2 2 1]), pk.dims(2)/2*[-1 -1 1 1], [1 0.85 0.2], 'FaceAlpha', 0.25, 'EdgeColor', [0.85 0.65 0.05], 'LineStyle', '--');
text(ax, mean(bx), pk.dims(2)/2 - 0.07, sprintf('battery\n%.0f x %.0f mm', 1e3*pk.dims(1), 1e3*pk.dims(2)), 'HorizontalAlignment', 'center', 'FontSize', 7.5);
text(ax, cgx, -0.065, sprintf('CG (x %+.3f m)', cgx), 'HorizontalAlignment', 'center', 'FontSize', 7.5);
local_cg(ax, cgx, 0);
local_dim(ax, [-0.66 -0.66], [-gm.B/2 gm.B/2], sprintf('gauge B %.2f m [EST]', gm.B), mg, 'v');
local_dim(ax, [0.66 0.66], [-0.39 0.39], sprintf('%.2f m [DATA]', gm.W_overall), nv, 'v');
local_dim(ax, [-Ltr/2 Ltr/2], [-0.5 -0.5], sprintf('track %.2f m, width b %.2f m [EST]', Ltr, gm.b), gy, 'h');
title(ax, '(b) Top view (dotted: contact length of the 7 contact points per track)', 'FontWeight', 'normal');
xlim(ax, [-0.95 0.95]); ylim(ax, [-0.62 0.5]);
end

function f = local_m27(P)
f = local_fig(1500, 1000, 'architecture_real');
ax = axes(f, 'Position', [0.01 0.01 0.98 0.96]); hold(ax, 'on'); axis(ax, [0 150 0 100]); axis(ax, 'off');
nv = [0.17 0.18 0.49]; mg = [0.65 0.29 0.61]; rd = [0.8 0.15 0.15]; gy = [0.62 0.62 0.66];
text(ax, 75, 98, 'Real robot as modelled in Shaddad\_System (Phase 1): component, real part and model', ...
    'HorizontalAlignment', 'center', 'FontSize', 15, 'FontWeight', 'bold', 'Color', nv);
local_legend_line(ax, [12 18], 93, nv, '-', 2, 'signal / CAN data');
local_legend_line(ax, [42 48], 93, mg, '-', 5, '48 V power');
local_legend_line(ax, [70 76], 93, rd, '--', 2, 'hard-wired safety (STO, coils)');
local_legend_line(ax, [108 114], 93, gy, '--', 2, 'Phase 2 / model addition');
% boxes [x y w h]
local_box(ax, [2 72 30 17], 'Operator', {'PS5 controller or RC transmitter', 'USB joystick (live drive) or SBUS', 'CH1 steer, CH2 throttle, CH6 arm,', 'CH12 remote E-stop'}, nv);
local_box(ax, [2 50 30 15], 'RC receiver (model)', {'SBUS: 25 B every 14 ms', 'frame_lost / failsafe flags', 'real: not in the photos'}, nv);
local_box(ax, [38 72 22 17], 'IMU', {'ICM-20948 class, 200 Hz', 'tilt, yaw rate (MCU Task_5)', 'model addition [EST]'}, nv);
local_box(ax, [64 76 26 13], 'GNSS + LiDAR', {'RTK GNSS, 2-D / 3-D LiDAR', 'Phase 2 model additions'}, gy, true);
local_box(ax, [94 76 54 13], 'Companion computer (Phase 2)', {'NavFusion EKF, perception, GNC (Shaddad_Companion)', 'Phase 1: GNC stub, AUTO cannot arm'}, gy, true);
local_box(ax, [38 44 38 25], 'Main controller (STM32)', {'model: Shaddad_MCU firmware model', 'real: XT_PS V1.0 board + STM32 Nucleo-32,', 'STM32 USB-C board, USB-UART, ADS1115 ADC', 'Task_1k SBUS decode, Task_5 IMU tilt,', 'Task_10 modes, safety, vehicle control', 'CAN1 500 kbit/s, CAN2 to the companion'}, nv);
y = [58 44];
lbl = {'L', 'R'};
for s = 1:2
    local_box(ax, [82 y(s) 15 11], ['Dual ESC ' lbl{s}], {'2 channels, FOC', '2 x 27 / 65 A rms [EST]'}, nv);
    local_box(ax, [100 y(s) 15 11], ['2 motors ' lbl{s}], {'front + rear, 48 V', '1.0 kW, 3000 rpm each'}, nv);
    local_box(ax, [118 y(s) 13 11], ['2 gearboxes ' lbl{s}], {'20:1 [EST]', 'brakes 2 x 5 N m'}, nv);
    local_box(ax, [134 y(s) 14 11], ['Track ' lbl{s}], {'both sprockets', 'driven, 0.090 m'}, nv);
    local_arrow(ax, [97 y(s) + 5.5], [100 y(s) + 5.5], nv, 1.5);
    local_arrow(ax, [115 y(s) + 5.5], [118 y(s) + 5.5], nv, 1.5);
    local_arrow(ax, [131 y(s) + 5.5], [134 y(s) + 5.5], nv, 1.5);
end
local_box(ax, [2 4 30 30], 'Safety chain', {'front / rear E-stops, bumpers,', 'wireless E-stop, safety relay', '(SR latch, re-armed by MCU DO1)', 'STO to both ESCs at once,', 'contactor coils off after 1.5 s', 'model: Power_Plant/Safety_Chain'}, rd);
local_box(ax, [38 22 24 15], 'BMS', {'13S Li-ion / NMC BMS', '150 A cont. / 250 A peak', 'CAN 500 kbit/s (GX-12)', 'model: Shaddad_BMS'}, nv);
local_box(ax, [38 3 24 16], 'Battery pack', {'13S15P Molicel P42A NMC', '46.8 V, 63 Ah, 2.95 kWh', '18.5 kg, 393 x 359 x 87 mm', 'model: Simscape Battery, 13 groups'}, nv);
local_box(ax, [66 3 30 34], 'PDU + LV rails', {'SB120 main connector', 'fuses, shunt, K1 / K2 precharge,', 'K3 implement, K4 charge (SB50)', 'LV DC/DC 24 / 12 / 5 / 3.3 V', '(real: LM2596 buck for 5 V)', 'charger 54.6 V 20 A', 'model: Power_Plant'}, nv);
local_box(ax, [100 3 48 34], 'Chassis + terrain + implement', {'XMachines MAX: 226 kg robot + 50 kg sprayer module', '1.0 x 0.78 x 0.57 m, 2 tracks on a 0.60 m gauge', 'model: structure_plant, 6-DOF body,', '2 x 7 track contact points, 2-D Dahl / Janosi shear,', 'terrain DEM and soil classes', 'sprayer: brass piston pump, 40 L tank, 8 nozzles'}, nv);
% links
local_arrow(ax, [17 72], [17 65], nv, 1.5); text(ax, 18, 68.5, '2.4 GHz RF', 'FontSize', 8, 'Color', nv);
local_arrow(ax, [32 56], [38 56], nv, 1.5); text(ax, 32.3, 58, 'SBUS', 'FontSize', 8, 'Color', nv);
local_arrow(ax, [49 72], [49 69], nv, 1.5);
local_arrow(ax, [76 63.5], [82 63.5], nv, 1.8); local_arrow(ax, [76 49.5], [82 49.5], nv, 1.8);
text(ax, 76.4, 56.5, 'CAN1', 'FontSize', 8, 'Color', nv);
local_arrow(ax, [94 76], [76 66], gy, 1.2, '--');
local_arrow(ax, [50 44], [50 37], nv, 1.5); text(ax, 51, 39.2, 'BMS\_REQ / BMS frames', 'FontSize', 8, 'Color', nv);
local_arrow(ax, [50 22], [50 19], nv, 1.2);
local_arrow(ax, [62 11], [66 11], mg, 4);
local_arrow(ax, [96 20], [100 20], mg, 4);
local_arrow(ax, [91 37], [91 44], mg, 4); text(ax, 92.3, 40.5, 'DC48 L / R', 'FontSize', 8, 'Color', mg);
plot(ax, [32 35 35 85], [30 30 41.6 41.6], '--', 'Color', rd, 'LineWidth', 1.8);
local_arrow(ax, [85 41.6], [85 44], rd, 1.8); text(ax, 64, 42.7, 'STO to both ESCs', 'FontSize', 8, 'Color', rd);
plot(ax, [141 141], [44 37], '-', 'Color', nv, 'LineWidth', 1.5);
text(ax, 139.5, 40.5, 'sprocket torque / belt speed', 'FontSize', 8, 'Color', nv, 'HorizontalAlignment', 'right');
text(ax, 2, 0.5, 'Values: [DATA] from the spec sheet, motor nameplate and battery spec sheet; [EST] estimated (docs/hardware/real_robot_components.md).', ...
    'FontSize', 8.5, 'Color', [0.35 0.35 0.4], 'Interpreter', 'none');
end

function f = local_m28()
f = local_fig(1500, 600, 'mbd_workflow');
ax = axes(f, 'Position', [0.005 0.01 0.99 0.95]); hold(ax, 'on'); axis(ax, [0 150 0 72]); axis(ax, 'off');
nv = [0.17 0.18 0.49]; mg = [0.65 0.29 0.61]; gy = [0.6 0.6 0.64];
text(ax, 75, 70, 'Model-based design workflow of the Shaddad\_UGV project', 'HorizontalAlignment', 'center', 'FontSize', 15, 'FontWeight', 'bold', 'Color', nv);
ph = {'Real-robot data', 'Parameters (single source)', 'Generated models', 'Simulation and verification', 'Results and deliverables'};
x0 = [2 32 62 92 122]; w = 26;
for k = 1:5
    patch(ax, x0(k) + [0 w w 0], [58 58 64 64], [0.93 0.91 0.97], 'EdgeColor', 'none');
    text(ax, x0(k) + w/2, 61, ph{k}, 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', mg);
end
local_box(ax, [2 26 26 29], 'Sources', {'spec sheet: 1730 x 780 x 570 mm,', '220 kg + 50 kg module', 'motor nameplate: 48 V, 750 W', 'battery spec: 13S15P P42A,', '46.8 V / 63 Ah, BMS, charger', 'battery-box drawing XMT-P1-E01', 'photos of boards and pump'}, nv);
local_box(ax, [32 26 26 29], 'params/*.m -> P', {'param_structure, param_drive,', 'param_power, param_electronics,', 'param_rc, param_sensors, param_gnc', 'every value tagged [DATA] / [EST]', 'buses/*.m: Bus_* interfaces', 'shaddad_params: P, Ts, VC_*'}, nv);
local_box(ax, [62 26 26 29], 'scripts/build/*.m -> .slx', {'every model is generated:', 'Shaddad_System (top)', 'Shaddad_Plant (Simscape:', 'structure, drive, power, electronics)', 'Shaddad_MCU, Shaddad_BMS', 'variants: VC_PHASE, VC_DRIVE_FID, ...'}, nv);
local_box(ax, [92 26 26 29], 'Tests and scenarios', {'unit benches per domain', '(t_structure, t_drive, t_pack_bms, ...)', 'scenarios S0-S8, test drives D1-D5', 'Phase 1 suite (Quick / Full)', 'identification: structure_identify_chi', 'live drive with a PS5 / RC transmitter'}, nv);
local_box(ax, [122 26 26 29], 'Outputs', {'plot_run: six plot pages', 'Robot3DView: live 3-D view', 'HTML test reports', 'NASL reports and manuals', 'code generation for the', 'STM32 targets (planned)'}, nv);
for k = 1:4, local_arrow(ax, [x0(k) + w 40.5], [x0(k + 1) 40.5], nv, 2.2); end
plot(ax, [135 135 15 15], [26 17 17 26], '--', 'Color', mg, 'LineWidth', 1.8);
local_arrow(ax, [15 18], [15 26], mg, 1.8, '--');
text(ax, 75, 14, 'calibration loop: measurements on the real robot (speed, current, slip, temperatures) -> identification -> replace [EST] values in params', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', mg);
text(ax, 75, 7, 'Rule: the .slx files are never edited by hand - change a parameter or a build script, regenerate, re-run the tests.', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0.35 0.35 0.4], 'FontAngle', 'italic');
end

function f = local_m42(P, d1file)
% test drive D1: speed and yaw rate against the command, path, motor torques
S = load(d1file).S; [~, nm] = fileparts(d1file);
f = local_fig(900, 620, 'd1_motion');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Test drive %s on the real-robot model: motion', strrep(nm, '_', '\_')), 'FontSize', 11, 'Color', [0.17 0.18 0.49]);
ax = local_tile(tl, '(a) Forward speed', 'speed [m/s]');
local_plot(ax, S, 'telem.v_target', 1, 'command (MCU target)', [0.65 0.29 0.61], '-', 1.2);
local_plot(ax, S, 'truth.speed_fwd', 1, 'actual', [0.17 0.18 0.49], '-', 1.4);
legend(ax, 'Location', 'southwest');
ax = local_tile(tl, '(b) Yaw rate (+ = left)', 'yaw rate [rad/s]');
local_plot(ax, S, 'telem.w_target', 1, 'command', [0.65 0.29 0.61], '-', 1.2);
local_plot(ax, S, 'truth.omega_B', 3, 'actual', [0.17 0.18 0.49], '-', 1.4);
local_plot(ax, S, 'telem.yaw_rate_meas', 1, 'gyro (MCU)', [0.95 0.55 0.1], '--', 1.0);
legend(ax, 'Location', 'southwest');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
[~, p] = local_sg(S, 'truth.pos_W'); [~, v] = local_sg(S, 'truth.speed_fwd');
scatter(ax, p(:, 1), p(:, 2), 6, v(:, 1), 'filled'); colormap(ax, parula); cb = colorbar(ax); cb.Label.String = 'speed [m/s]';
plot(ax, p(1, 1), p(1, 2), 'o', 'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'k');
plot(ax, p(end, 1), p(end, 2), 's', 'MarkerFaceColor', [0.85 0.1 0.1], 'MarkerEdgeColor', 'k');
title(ax, '(c) Path on the field (o start, square end)', 'FontWeight', 'normal'); xlabel(ax, 'East [m]'); ylabel(ax, 'North [m]');
n = P.drive.n_mot_side; Tc = P.drive.motor_unit.T_cont;      % side equivalent of n units -> per motor
ax = local_tile(tl, '(d) Motor torque (each motor)', 'torque per motor [N m]');
local_plot(ax, S, 'drive_truth.L.tau_m', 1, 'left-side motors', [0.17 0.18 0.49], '-', 1.2, 1/n);
local_plot(ax, S, 'drive_truth.R.tau_m', 1, 'right-side motors', [0.65 0.29 0.61], '-', 1.2, 1/n);
yline(ax, Tc*[-1 1], ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
text(ax, 1, Tc + 0.3, sprintf('continuous rating %.2f N m', Tc), 'FontSize', 8, 'Color', [0.35 0.35 0.4]);
ylim(ax, [-3.8 4]); legend(ax, 'Location', 'southwest');
end

function f = local_m43(P, d1file)
% test drive D1: pack voltage and current, power, state of charge, group voltages
S = load(d1file).S; [~, nm] = fileparts(d1file);
f = local_fig(900, 620, 'd1_energy');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Test drive %s on the real-robot model: battery and energy', strrep(nm, '_', '\_')), 'FontSize', 11, 'Color', [0.17 0.18 0.49]);
ax = local_tile(tl, '(a) Pack voltage and current', 'voltage [V]');
local_plot(ax, S, 'power_truth.V_term_V', 1, 'voltage', [0.17 0.18 0.49], '-', 1.2);
yyaxis(ax, 'right'); local_plot(ax, S, 'power_truth.I_pack_A', 1, 'current', [0.95 0.55 0.1], '-', 1.0);
ylabel(ax, 'current [A] (+ = discharge)'); ax.YAxis(2).Color = [0.85 0.45 0.05]; yyaxis(ax, 'left');
legend(ax, 'Location', 'south');
ax = local_tile(tl, '(b) Power', 'power [W]');
local_plot(ax, S, 'power_truth.P_pack_W', 1, 'battery pack', [0.17 0.18 0.49], '-', 1.4);
[t, pl] = local_sg(S, 'drive_truth.L.p_dc'); [~, pr] = local_sg(S, 'drive_truth.R.p_dc');
plot(ax, t, pl + pr, '-', 'Color', [0.65 0.29 0.61], 'LineWidth', 1.0, 'DisplayName', 'both traction drives (DC)');
legend(ax, 'Location', 'northeast');
ax = local_tile(tl, '(c) State of charge', 'SOC [%]');
local_plot(ax, S, 'power_truth.SOC_pack_true', 1, 'true', [0.17 0.18 0.49], '-', 1.4, 100);
local_plot(ax, S, 'bms_telem.SOC_pack', 1, 'BMS estimate', [0.65 0.29 0.61], '--', 1.2, 100);
legend(ax, 'Location', 'northeast');
ax = local_tile(tl, sprintf('(d) Cell-group voltages (%d groups)', P.power.pack.Ns), 'voltage [V]');
[t, vc] = local_sg(S, 'power_truth.V_cell_true');
plot(ax, t, vc, '-', 'Color', [0.7 0.7 0.75], 'HandleVisibility', 'off');
plot(ax, t, max(vc, [], 2), '-', 'Color', [0.65 0.29 0.61], 'LineWidth', 1.2, 'DisplayName', 'highest group');
plot(ax, t, min(vc, [], 2), '-', 'Color', [0.17 0.18 0.49], 'LineWidth', 1.2, 'DisplayName', 'lowest group');
legend(ax, 'Location', 'southwest');
end

function f = local_m44(P, file)
% S2 on the real-robot model: true and estimated pitch against TILT_WARN, speeds, slip and torque limit
S = load(file).S; [~, nm] = fileparts(file);
f = local_fig(900, 760, 's2_tilt_speed');
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
m3 = shaddad_params('Implement', 3, 'Assign', false).structure.mp.m0;
title(tl, sprintf('S2 slope run %s: 20 deg ramp, full sprayer (%.0f kg)', strrep(nm, '_', '\_'), m3), 'FontSize', 11, 'Color', [0.17 0.18 0.49]);
[tf, fw] = local_sg(S, 'telem.fault_word'); warn = double(bitget(uint32(fw), 15));
ax = local_tile(tl, '(a) Pitch: true body pitch, MCU estimate and the TILT\_WARN threshold', 'pitch [deg]');
local_band(ax, tf, warn, [-3 27]);
local_plot(ax, S, 'truth.eul', 2, 'true pitch', [0.17 0.18 0.49], '-', 1.4, -180/pi);
local_plot(ax, S, 'telem.pitch', 1, 'MCU estimate', [0.95 0.55 0.1], '-', 1.2, -180/pi);
yline(ax, P.electronics.safety.tilt_warn*180/pi, '--', sprintf('TILT\\_WARN %.0f deg', P.electronics.safety.tilt_warn*180/pi), ...
    'Color', [0.8 0.15 0.15], 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
ylim(ax, [-3 27]); legend(ax, 'Location', 'southeast');
ax = local_tile(tl, '(b) Speed: MCU target, belt and ground (shaded: TILT\_WARN set)', 'speed [m/s]');
local_band(ax, tf, warn, [0 1]);
local_plot(ax, S, 'telem.v_target', 1, 'MCU target', [0.65 0.29 0.61], '-', 1.2);
local_plot(ax, S, 'truth.vBelt_LR', 1, 'belt (left)', [0.4 0.4 0.75], '--', 1.0);
local_plot(ax, S, 'truth.speed_fwd', 1, 'ground', [0.17 0.18 0.49], '-', 1.4);
yline(ax, 0.75, ':', 'criterion 0.75 m/s', 'Color', [0.3 0.3 0.3], 'LabelHorizontalAlignment', 'center', 'HandleVisibility', 'off');
ylim(ax, [0 0.95]); legend(ax, 'Location', 'southeast');
ax = local_tile(tl, '(c) Track slip and the MCU motor-torque limit', 'slip [%]');
local_band(ax, tf, warn, [0 40]);
local_plot(ax, S, 'truth.slip_LR', 1, 'slip left', [0.17 0.18 0.49], '-', 1.0, 100);
local_plot(ax, S, 'truth.slip_LR', 2, 'slip right', [0.65 0.29 0.61], '-', 1.0, 100);
ylim(ax, [-5 40]);
yyaxis(ax, 'right'); local_plot(ax, S, 'telem.tau_lim_mot', 1, 'torque limit', [0.95 0.55 0.1], '-', 1.2);
ylabel(ax, 'torque limit [N m]'); ylim(ax, [0 8]); ax.YAxis(2).Color = [0.85 0.45 0.05]; yyaxis(ax, 'left');
legend(ax, 'Location', 'northeast');
end

function f = local_m45(P, file)
% S6a on the real-robot model: controlled stop after the radio loss (reference, belt and ground speed,
% 100 ms deceleration, motor torque and feed-forward)
S = load(file).S; [~, nm] = fileparts(file);
[tm, mo] = local_sg(S, 'telem.mode'); tc = tm(find(mo == 7, 1));
f = local_fig(900, 620, 's6_stop');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('S6a run %s: controlled stop after the radio loss (t = 0 at CTRL\\_STOP)', strrep(nm, '_', '\_')), 'FontSize', 11, 'Color', [0.17 0.18 0.49]);
ax = local_tile(tl, '(a) Speed', 'speed [m/s]');
local_plot(ax, S, 'telem.v_ref', 1, 'MCU reference', [0.65 0.29 0.61], '-', 1.2, 1, tc);
local_plot(ax, S, 'truth.vBelt_LR', 1, 'belt (left)', [0.4 0.4 0.75], '--', 1.0, 1, tc);
local_plot(ax, S, 'truth.speed_fwd', 1, 'ground', [0.17 0.18 0.49], '-', 1.4, 1, tc);
xlim(ax, [-0.3 1.5]); legend(ax, 'Location', 'northeast');
ax = local_tile(tl, '(b) Deceleration, 100 ms moving average', 'deceleration [m/s^2]');
[t, v] = local_sg(S, 'truth.speed_fwd'); m = t >= tc - 0.5 & t <= tc + 2;
t = t(m) - tc; v = v(m, 1); a = -movmean([0; diff(v)./diff(t)], round(0.1/median(diff(t))));
plot(ax, t, a, '-', 'Color', [0.17 0.18 0.49], 'LineWidth', 1.4, 'DisplayName', 'ground');
yline(ax, P.electronics.shaper.a_ctrlstop, '--', 'shaper 1.0', 'Color', [0.65 0.29 0.61], 'HandleVisibility', 'off');
yline(ax, 1.05, ':', 'criterion 1.05', 'Color', [0.8 0.15 0.15], 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
xlim(ax, [-0.3 1.5]); ylim(ax, [-0.2 1.4]);
n = P.drive.n_mot_side;                       % side values of the equivalent drive -> per motor
ax = local_tile(tl, '(c) Torque per motor (left) and feed-forward', 'torque per motor [N m]');
local_plot(ax, S, 'drive_truth.L.tau_m', 1, 'motor torque', [0.17 0.18 0.49], '-', 1.4, 1/n, tc);
local_plot(ax, S, 'telem.tau_ff', 1, 'feed-forward', [0.95 0.55 0.1], '--', 1.2, 1/n, tc);
xlim(ax, [-0.3 1.5]); legend(ax, 'Location', 'southeast');
ax = local_tile(tl, '(d) Motor speed: reference and actual (left)', 'speed [rad/s]');
local_plot(ax, S, 'telem.w_ref_motor', 1, 'reference', [0.65 0.29 0.61], '-', 1.2, 1, tc);
local_plot(ax, S, 'drive_truth.L.w_m', 1, 'actual', [0.17 0.18 0.49], '-', 1.4, 1, tc);
xlim(ax, [-0.3 1.5]); legend(ax, 'Location', 'northeast');
end

function local_band(ax, t, on, yl)
% shade the intervals where on ~= 0
d = diff([0; on(:) ~= 0; 0]); i0 = find(d == 1); i1 = find(d == -1) - 1;
for k = 1:numel(i0)
    patch(ax, t([i0(k) i1(k) i1(k) i0(k)]), yl([1 1 2 2]), [1 0.85 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end

function file = local_newest(root, pat)
L = dir(fullfile(root, 'work', 'results', pat));
[~, i] = max([L.datenum]);
file = fullfile(L(i).folder, L(i).name);
end

function ax = local_tile(tl, ttl, ylab)
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
title(ax, ttl, 'FontWeight', 'normal'); xlabel(ax, 'time [s]'); ylabel(ax, ylab);
end

function local_plot(ax, S, p, col, name, c, ls, lw, k, t0)
% plot column col of a logged signal, scaled by k, against t - t0
if nargin < 9, k = 1; end
if nargin < 10, t0 = 0; end
[t, v] = local_sg(S, p);
if isempty(t), return, end
plot(ax, t - t0, k*v(:, col), ls, 'Color', c, 'LineWidth', lw, 'DisplayName', name);
xlim(ax, [0 t(end)] - t0);
end

function [t, v] = local_sg(S, p)
% logged signal 'group.field' -> time column and values (empty when not logged)
t = []; v = []; x = S;
for k = strsplit(p, '.')
    if ~isstruct(x) || ~isfield(x, k{1}), return, end
    x = x.(k{1});
end
t = double(x.t(:)); v = reshape(double(x.v), numel(t), []);
end

% =========================================================================================== helpers
function f = local_fig(w, h, tag)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 w h], 'UserData', tag);
end

function local_circle(ax, xc, zc, r, col)
th = linspace(0, 2*pi, 60);
patch(ax, xc + r*cos(th), zc + r*sin(th), col, 'EdgeColor', [0.2 0.2 0.22]);
end

function local_cg(ax, x, z)
th = linspace(0, 2*pi, 40); r = 0.025;
patch(ax, x + r*cos(th), z + r*sin(th), 'w', 'EdgeColor', 'k', 'LineWidth', 1.2);
patch(ax, x + [0 r*cos(linspace(0, pi/2, 10)) 0], z + [0 r*sin(linspace(0, pi/2, 10)) 0], 'k', 'EdgeColor', 'none');
patch(ax, x + [0 r*cos(linspace(pi, 3*pi/2, 10)) 0], z + [0 r*sin(linspace(pi, 3*pi/2, 10)) 0], 'k', 'EdgeColor', 'none');
end

function local_dim(ax, xs, ys, label, col, dir)
plot(ax, xs, ys, '-', 'Color', col, 'LineWidth', 1.2);
if any(strcmp(dir, {'h', 'hb'}))
    plot(ax, xs(1)*[1 1], ys(1) + [-0.02 0.02], '-', 'Color', col); plot(ax, xs(2)*[1 1], ys(1) + [-0.02 0.02], '-', 'Color', col);
    if strcmp(dir, 'h')
        text(ax, mean(xs), ys(1) + 0.025, label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8.5, 'Color', col);
    else
        text(ax, mean(xs), ys(1) - 0.025, label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontSize', 8.5, 'Color', col);
    end
elseif strcmp(dir, 'vl')
    plot(ax, xs(1) + [-0.02 0.02], ys(1)*[1 1], '-', 'Color', col); plot(ax, xs(1) + [-0.02 0.02], ys(2)*[1 1], '-', 'Color', col);
    text(ax, xs(1) - 0.03, mean(ys), label, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', 'FontSize', 8.5, 'Color', col);
else
    plot(ax, xs(1) + [-0.02 0.02], ys(1)*[1 1], '-', 'Color', col); plot(ax, xs(1) + [-0.02 0.02], ys(2)*[1 1], '-', 'Color', col);
    text(ax, xs(1) + sign(xs(1))*0.03, mean(ys), label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'Rotation', 90, 'FontSize', 8.5, 'Color', col);
end
end

function local_box(ax, r, ttl, lines, col, dashed)
if nargin < 6, dashed = false; end
ls = '-'; fc = [0.95 0.95 0.97];
if dashed, ls = '--'; fc = [0.97 0.97 0.98]; end
rectangle(ax, 'Position', r, 'Curvature', 0.08, 'FaceColor', fc, 'EdgeColor', col, 'LineWidth', 1.6, 'LineStyle', ls);
text(ax, r(1) + r(3)/2, r(2) + r(4) - 2.2, ttl, 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
    'Color', col, 'Interpreter', 'none');
n = numel(lines);
for k = 1:n
    text(ax, r(1) + r(3)/2, r(2) + r(4) - 5 - (k - 1)*min(2.6, (r(4) - 6)/max(n, 1)), lines{k}, 'HorizontalAlignment', 'center', ...
        'FontSize', 8.6, 'Color', [0.2 0.2 0.25], 'Interpreter', 'none');
end
end

function local_arrow(ax, p0, p1, col, lw, ls)
if nargin < 6, ls = '-'; end
plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], ls, 'Color', col, 'LineWidth', lw);
d = p1 - p0; L = norm(d);
if L < 1e-9, d = [0 1]; L = 1; end
u = d/L; n = [-u(2) u(1)]; h = 1.4 + 0.25*lw;
pts = [p1; p1 - h*u + 0.5*h*n; p1 - h*u - 0.5*h*n];
patch(ax, pts(:, 1), pts(:, 2), col, 'EdgeColor', 'none');
end

function local_legend_line(ax, xs, y, col, ls, lw, label)
plot(ax, xs, [y y], ls, 'Color', col, 'LineWidth', lw);
text(ax, xs(2) + 1, y, label, 'FontSize', 9.5, 'Color', [0.2 0.2 0.25], 'VerticalAlignment', 'middle');
end
