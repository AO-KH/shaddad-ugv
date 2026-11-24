function files = real_mbd_screens(which, d1file)
%REAL_MBD_SCREENS  Model views of the real robot for the model-based design report (figures M30..M41).
%
%   files = real_mbd_screens()                 every figure
%   files = real_mbd_screens({'M30', 'M33'})   some figures
%   files = real_mbd_screens(which, d1file)    plot pages of a saved run (default: newest work/results/D1_*.mat)
%
%     M30 13S15P battery module as built with Simscape Battery (batteryChart, 195 cells)
%     M31 Pack_Grouped model (13 cell groups)      M32 Power_Plant model
%     M33 3-D view, side camera                    M34 3-D view with the sprayer module (free camera)
%     M35 3-D view, chase camera
%     M36..M41 plot_run pages (drive, tracks, battery, power, rc, nav) of the test drive D1
%   Models are loaded and closed without saving.

all = {'M30', 'M31', 'M32', 'M33', 'M34', 'M35', 'M36'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which), which = {which}; end
root = char(currentProject().RootFolder);
out = fullfile(root, 'docs', 'report', 'figs');
if nargin < 2 || isempty(d1file)
    L = dir(fullfile(root, 'work', 'results', 'D1_*.mat'));
    [~, i] = max([L.datenum]);
    d1file = fullfile(L(i).folder, L(i).name);
end
files = {};
for k = 1:numel(which)
    switch which{k}
        case 'M30', files = [files, local_m30(out)]; %#ok<AGROW>
        case 'M31', files = [files, local_print('Pack_Grouped', fullfile(out, 'M31_pack_grouped.png'))]; %#ok<AGROW>
        case 'M32', files = [files, local_print('Power_Plant', fullfile(out, 'M32_power_plant.png'))]; %#ok<AGROW>
        case 'M33', files = [files, local_view(fullfile(out, 'M33_robot3d_side.png'), 0, 'side', [0 0 0], 0)]; %#ok<AGROW>
        case 'M34', files = [files, local_view(fullfile(out, 'M34_robot3d_sprayer.png'), 3, 'free', [0 0 0], 0)]; %#ok<AGROW>
        case 'M35', files = [files, local_view(fullfile(out, 'M35_robot3d_chase.png'), 3, 'chase', [0 0 0], 20)]; %#ok<AGROW>
        case 'M36', files = [files, local_plots(out, d1file)]; %#ok<AGROW>
        otherwise, error('real_mbd_screens:which', 'unknown figure %s', which{k});
    end
end
fprintf('%s\n', files{:});
end

function file = local_m30(out)
% the module object of build_shaddad_battery (same cell, 15 in parallel, 13 in series) drawn with batteryChart
pw = param_power(); c0 = pw.cell; pk = pw.pack;
g = batteryCylindricalGeometry();
g.Radius = simscape.Value(c0.D/2, 'm'); g.Height = simscape.Value(c0.H, 'm');
c = batteryCell(g);
c.Mass = simscape.Value(c0.mass, 'kg');
c.Capacity = simscape.Value(c0.Q_Ah, 'A*hr');
pa = batteryParallelAssembly(c, pk.Np, Topology = "Square");          % a row of 15 cells along y
m = batteryModule(pa, pk.Ns, StackingAxis = "X");                     % 13 rows side by side, as in the 393 x 358 mm box
m.ModelResolution = "Grouped"; m.SeriesGrouping = ones(1, pk.Ns); m.ParallelGrouping = ones(1, pk.Ns);
f = figure('Visible', 'off', 'Color', 'w', 'Position', [20 20 1400 900]);
bc = batteryChart(f, m);
bc.SimulationStrategyVisible = "on";
ax = findall(f, 'Type', 'axes');
for a = ax(:)', view(a, -32, 38); end
title(bc, sprintf('Shaddad pack model: %dS%dP Molicel P42A (%d cells, %d series models, %.1f kg cells)', ...
    pk.Ns, pk.Np, pk.Ns*pk.Np, m.NumModels, value(m.CumulativeMass, 'kg')));
file = fullfile(out, 'M30_battery_module_chart.png');
exportgraphics(f, file, 'Resolution', 200, 'BackgroundColor', 'white', 'Padding', 10);
close(f);
end

function file = local_print(mdl, file)
% print a model at 200 dpi without changing its file
wasLoaded = bdIsLoaded(mdl);
load_system(mdl);
if strcmp(mdl, 'Pack_Grouped') && getSimulinkBlockHandle([mdl '/Probe']) > 0
    set_param([mdl '/Probe'], 'Position', [-110 40 90 150]);      % port labels clear of the icon (in memory)
end
print(['-s' mdl], '-dpng', '-r200', file);
if ~wasLoaded, close_system(mdl, 0); end
end

function file = local_view(file, impl, cam, pos, psi)
% one frame of the live 3-D view (real-robot geometry), MCU mode MANUAL, belts at 0.8 m/s
v = shaddad.Robot3DView('Implement', impl, 'Camera', cam, 'Visible', false, 'SampleTime', 0.05);
R = [cosd(psi) -sind(psi) 0; sind(psi) cosd(psi) 0; 0 0 1];
rc = zeros(16, 1); rc(2) = 0.55; rc(5) = -1; rc(6) = 1;
for k = 1:6, v(pos(:), R, [0.8; 0.8], 3, 0.82, 0.8, rc); end
f = v.figureHandle();
f.Position = [20 20 1400 820];
if strcmp(cam, 'free')
    ax = findobj(f, 'Type', 'axes', '-and', 'Projection', 'perspective');
    tg = [pos(1) pos(2) 0.45]; az = -38; el = 24;
    campos(ax(1), tg + 4.2*[sind(az)*cosd(el), -cosd(az)*cosd(el), sind(el)]); camtarget(ax(1), tg); camva(ax(1), 30);
end
drawnow;
exportgraphics(f, file, 'Resolution', 150, 'Padding', 0);
release(v); close(f);
end

function files = local_plots(out, d1file)
% the six plot_run pages of the test drive at report size
figs = plot_run(d1file, 'Visible', 'off', 'Size', [1400 900]);
pages = {'drive', 'tracks', 'battery', 'power', 'rc', 'nav'};
files = {};
for k = 1:min(numel(figs), numel(pages))
    file = fullfile(out, sprintf('M%d_d1_%s.png', 35 + k, pages{k}));
    fr = getframe(figs(k)); imwrite(fr.cdata, file);
    close(figs(k));
    files{end+1} = file; %#ok<AGROW>
end
end
