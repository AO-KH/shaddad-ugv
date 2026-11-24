% diag_f33.m - project folder structure (Figure 'Project folders'); fix round 2: 39 build_*.m + helper shaddad_bu.m
addpath(fullfile(fileparts(mfilename('fullpath')), 'lib'));
c = nasl_pal();
W = 180; H = 134;
[f, ax] = dcanvas(W, H, 16.5);
dtext(ax, W/2, 131, 'MATLAB project Shaddad_UGV - folder structure (C:\Users\aomar\Desktop\Robot project)', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', c.navy);
dbox(ax, 50, 112, 80, 13, 'Shaddad_UGV.prj', {'project root: startup shaddad_startup (P, buses, config sets),', 'shutdown shaddad_shutdown; all folders on the project path'}, 'Fill', [0.93 0.92 0.97], 'SubSize', 5.8);
cards = {
 'models/  (8 model files)', {'Shaddad_System (top model), Shaddad_Plant (all Simscape),', 'Shaddad_MCU, Shaddad_BMS (ECU firmware),', 'Shaddad_Companion, _GNC, _NavFusion, _Perception (Phase 2)'}, c.navy
 'subsystems/  (117 Subsystem Reference .slx)', {'drive 13, electronics 37 (MCU fw_*, CAN, plant), power 24,', 'rc 6, structure 14, sensors 10, gnc 11, integration 2;', 'MATLAB Function sources (mcu_mode_manager.m, esc_sm.m, ...)'}, c.navy
 'scripts/', {'phase1_test_drive.m (D1 demo), run_scenario.m, run_all_scenarios.m', 'scenarios/: scn_S0 .. scn_S8, scn_D1 (13 files);  missions/: 3 JSON', 'build/: 39 build_*.m model generators (+ helper shaddad_bu.m)'}, c.purple
 'params/  (11 files)', {'shaddad_params.m assembles param_<domain>.m into P', '(drive, electronics, gnc, power, rc, sensors, sim, structure,', 'world); shaddad_configsets.m (cs_SimFixed, cs_SimRef, ...)'}, c.purple
 'buses/  (26 files)', {'shaddad_buses.m + buses_<domain>.m: 88 Bus objects', 'enumerations: ShaddadMode, EscState, BmsContactorState,', 'GncState, HoldReason, ... (uint8)'}, c.purple
 'utilities/  (32 files + package)', {'eval_criteria, scn_* helpers (inputs, logging, metrics),', 'sbus_ref_encode / decode, shaddad_replay_birdseye,', 'world_* builders, +shaddad/BirdsEyeView'}, c.magenta
 'tests/', {'per-domain test benches and test classes:', 'drive, electronics, gnc, integration, power, rc,', 'sensors, structure; system/tScenarios.m'}, c.magenta
 'docs/', {'architecture.md, icd.md (interfaces), build_plan.md,', 'STATUS.md, hw_bringup.md, specs/ (7 domain specs),', 'report/ (this report and its figures)'}, c.magenta
 'work/  (generated at run time)', {'results/: scenario logs <id>_<date>_<time>.mat', 'slprj/, *.slxc caches, profiler reports', '(Simulink cache and code-generation folder)'}, c.dgrey
 'resources/, +bus_conv_fcns/', {'project metadata (resources/project)', 'bus conversion functions used by', 'the ROS 2 companion I/O'}, c.dgrey
 };
cw = 84; ch = 18.5; gx = 6; x1 = 3; x2 = x1 + cw + gx;
ys = 89 - (0:4)*20.8;
for i = 1:size(cards, 1)
    col = 1 + (i > 5); row = i - 5*(col - 1);
    x = x1 + (col - 1)*(cw + gx); y = ys(row);
    dbox(ax, x, y, cw, ch, cards{i, 1}, cards{i, 2}, 'Align', 'left', 'TitleSize', 7.4, 'SubSize', 5.8, 'Edge', cards{i, 3}, 'Text', cards{i, 3}, 'Radius', 1.5);
end
% tree connectors
plot(ax, [90 90], [112 108.5], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
plot(ax, [x1 + cw/2, x2 + cw/2], [108.5 108.5], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
plot(ax, [x1 + cw/2, x1 + cw/2], [108.5 ys(1) + ch], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
plot(ax, [x2 + cw/2, x2 + cw/2], [108.5 ys(1) + ch], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
for i = 1:4
    plot(ax, [x1 + cw/2, x1 + cw/2], [ys(i) ys(i + 1) + ch], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
    plot(ax, [x2 + cw/2, x2 + cw/2], [ys(i) ys(i + 1) + ch], '-', 'Color', c.mgrey, 'LineWidth', 1.0);
end
dtext(ax, W/2, 2.2, 'Colour key: navy = models, purple = how to run and parameterise, magenta = support, test and documentation, grey = generated / metadata', 'FontSize', 5.8, 'Color', c.dgrey);
exportgraphics(f, 'C:\Users\aomar\Desktop\Robot project\docs\report\figs\F33_project_structure.png', 'Resolution', 200, 'BackgroundColor', 'white');
close(f); disp('F33 done');
