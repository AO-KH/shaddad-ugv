function files = real_electronics_3d(which, labels)
%REAL_ELECTRONICS_3D  3-D renders of the real robot's electronics, modelled on the photos (figures M50..M55).
%
%   files = real_electronics_3d()             every render
%   files = real_electronics_3d({'M51'})      some renders
%   files = real_electronics_3d(which, false) without callouts, saved as <name>_plain.png (collage tiles)
%
%   The photos in docs/hardware/source/ show the boards glued into the electronics box. These renders rebuild
%   each board from generic parts (standard package sizes) placed as in the photos, with the silkscreen drawn
%   as a texture, so the report can show clean views of the hardware. Layouts and part values are approximate.
%     M50 XT_PS V1.0 control board with the STM32 Nucleo-32
%     M51 STM32 USB-C core board and FT232RL USB-UART adapter on the control board
%     M52 ADS1115 / ADS1015 ADC module on the control board
%     M53 LM2596 DC-DC buck module (top side: typical layout; bottom side: as photographed)
%     M54 XMachines SPR V1.3 sprayer board (detail)
%     M55 electronics box (overview)

all = {'M50', 'M51', 'M52', 'M53', 'M54', 'M55'};
if nargin < 1 || isempty(which), which = all; end
if ischar(which), which = {which}; end
if nargin < 2, labels = true; end
root = char(currentProject().RootFolder);
out = fullfile(root, 'docs', 'report', 'figs');
files = {};
for k = 1:numel(which)
    G('reset');
    switch which{k}
        case 'M50', [opt, tag] = local_m50();
        case 'M51', [opt, tag] = local_m51();
        case 'M52', [opt, tag] = local_m52();
        case 'M53', [opt, tag] = local_m53();
        case 'M54', [opt, tag] = local_m54();
        case 'M55', [opt, tag] = local_m55();
        otherwise, error('real_electronics_3d:which', 'unknown render %s', which{k});
    end
    if ~labels, G('nocallouts'); tag = [tag '_plain']; end
    f = local_render(opt);
    name = fullfile(out, [which{k} '_' tag '.png']);
    exportgraphics(f, name, 'Resolution', 200, 'BackgroundColor', 'white', 'Padding', 10);
    close(f);
    img = imread(name); m = min(img, [], 3) < 236;            % crop to the content (ground tint counts as white)
    r = find(any(m, 2)); q = find(any(m, 1)); p = 30;
    imwrite(img(max(1, r(1) - p):min(end, r(end) + p), max(1, q(1) - p):min(end, q(end) + p), :), name);
    files{end+1} = name; %#ok<AGROW>
    fprintf('%s\n', name);
end
end

% =========================================================================================== scenes
function [opt, tag] = local_m50()
% XT_PS V1.0 control board (68 x 56 mm) with the STM32 Nucleo-32, as in electronics_box_xt_ps_nucleo.jpeg
tag = 'xt_ps_board_3d';
local_xtps_geometry(true);
callout([57.8 31 13.6], [74 50 22], 'STM32 Nucleo-32 board (main MCU)', 'left');
callout([6 51.5 10.5], [-8 66 30], '+48 V input', 'right');
callout([26 52 3.3], [22 70 26], 'on-board buck regulator', 'center');
callout([0.8 30.5 6.0], [-24 40 26], 'CAN connectors', 'right');
callout([17 25.6 1.7], [-22 6 22], 'U3: CAN transceiver (SOIC-8)', 'right');
callout([18.6 14 2.2], [6 -12 18], 'optocouplers U5, U7', 'center');
callout([46 30 0.1], [44 -12 22], 'Nucleo pin labels (A0-A7, D13, 5V, VIN)', 'center');
opt = struct('size', [1250 840], 'az', -28, 'el', 48, 'target', [30 24 4], 'dist', 330, 'va', 19.5, 'ground', -1.7, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_xtps_geometry(wires)
% parts of the XT_PS board and the Nucleo-32 (board origin at its bottom-left corner)
c = local_colors();
local_board([34 28], 68, 56, 0, 0, 1.6, c.green_edge, local_tex(68, 56, 28, c.green, @local_xtps_tex));
% 48 V input (J1) with its cable, on-board buck regulator (U1, L1, D2, C4)
part_housing([6 51.5], 0, 0, [7.6 5.8 7.0], c.jst);
part_housing([6 51.8], 7.0, 0, [6.6 4.6 3.2], [0.9 0.88 0.8]);
if wires
    local_wire([4.7 51.8 10; 4.7 53 16; 1 60 22; -10 70 24], 0.9, [0.78 0.08 0.06]);
    local_wire([7.3 51.8 10; 7.3 53.5 16; 4 61 22; -6 72 25], 0.9, [0.08 0.08 0.09]);
end
part_soic([13.5 52], 0, 0, 8, 1.27, 4.9, 3.9, 1.5);
part_smd([19.3 52], 0, 90, [3.5 2.8 1.9], [0.93 0.72 0.12]);
part_sma([24.0 52], 0, 90);
[V, F] = p_box([0 0 0], [6.0 6.0 3.2]); put('plastic', V, F, [0.12 0.12 0.13], [30.2 51.6], 0, 0);
local_mark([30.2 51.6], 3.205, 0, [3.6 2.4], '220', [0.8 0.8 0.8]);
part_housing([40.2 52.2], 0, 0, [10.6 9.0 9.5], [0.10 0.10 0.11], [7.0 5.5]);
part_elec([25.5 42.2], 0, 0, 5, 7, [], []);
part_disc([31.0 42.6], 0, 0, 4.2);
part_res([35.6 42.0], 0, 90, 10.16, 'nkkrn');
part_diode([14.5 43.6], 0, 180, 10.16);
% logic: U9, U3 (CAN side), optocouplers U5 / U7, resistors, C2
part_soic([18.0 34.0], 0, 0, 8, 1.27, 4.9, 3.9, 1.5);
part_soic([17.0 25.6], 0, 0, 8, 1.27, 4.9, 3.9, 1.5);
part_soic([18.6 17.6], 0, 0, 4, 2.54, 4.4, 3.6, 2.0);
part_soic([18.6 10.6], 0, 0, 4, 2.54, 4.4, 3.6, 2.0);
part_res([10.2 16.4], 0, 90, 10.16, 'nkkon');
part_res([12.9 16.4], 0, 90, 10.16, 'yvkrn', c.res_beige);
part_res([27.6 11.6], 0, 90, 10.16, 'nkkrn');
part_res([29.9 11.6], 0, 90, 10.16, 'rrkon');
part_res([32.2 11.6], 0, 90, 10.16, 'nkknn');
part_disc([36.6 10.0], 0, 0, 4.2);
% CAN connectors on the left edge with twisted pairs
for v = [30.5 15.5]
    part_housing([3.4 v], 0, 90, [12.4 5.8 6.0], c.jst);
    part_housing([3.2 v], 6.0, 90, [11.2 4.4 3.4], [0.9 0.88 0.8]);
end
if wires
    local_twisted([3.4 32.0 9.4; 2 32.5 14; -6 34 17; -20 36 15], [0.95 0.75 0.1; 0.15 0.45 0.85], 0.55, 6);
    local_twisted([3.4 29.0 9.4; 1.5 28.5 13; -6 27 15; -20 25 14], [0.2 0.6 0.25; 0.95 0.95 0.95], 0.55, 6);
    local_twisted([3.4 17.0 9.4; 2 17.5 13; -6 18.5 15; -20 20 13], [0.9 0.4 0.1; 0.95 0.95 0.95], 0.55, 6);
    local_twisted([3.4 14.0 9.4; 1.5 13.5 12; -6 12 13; -20 10 12], [0.8 0.1 0.1; 0.15 0.45 0.85], 0.55, 6);
end
% STM32 Nucleo-32 on female headers (CN4 on the labelled side)
for u = [57.8 - 7.62, 57.8 + 7.62]
    part_fheader([u 28], 0, 90, 15, 1, 8.5);
    part_header([u 28], 8.5, 90, 15, 1, 0, c.gold, 2.5);
end
N = local_board([57.8 28], 18.54, 50.04, 0, 12.6, 1.6, [0.82 0.82 0.84], local_tex(18.54, 50.04, 40, c.nucleo, @local_nucleo_tex));
zt = 12.6;
for u = [1.65 16.89], part_pintips(N.xy(u, 25.02), zt, 90, 15, 1, 1.0, c.gold); end
part_socket(N.xy(9.27, 48.3), zt, 0, 7.5, 2.6, 5.6, 0.6, c.alu);
part_qfp(N.xy(9.27, 37.2), zt, 0, 12, 0.5, 7, 1.4);
part_qfp(N.xy(9.27, 20.5), zt, 0, 8, 0.8, 7, 1.4);
part_xtal(N.xy(13.9, 30.8), zt, 0, [3.2 2.5 0.8]);
part_soic(N.xy(4.6, 29.5), zt, 90, 6, 0.95, 2.9, 1.6, 1.1);
part_led(N.xy(5.2, 45.4), zt, 0, [1 0.6 0.1]);
part_led(N.xy(13.4, 45.4), zt, 0, [1 0.12 0.08]);
part_led(N.xy(15.2, 3.6), zt, 90, [1 0.12 0.08]);
part_button(N.xy(4.4, 4.4), zt, 0, [3.6 3.0], 1.1, 0.85, [0.12 0.12 0.12], [0.85 0.85 0.86]);
for k = 1:numel(local_nucleo_smd())
    q = local_nucleo_smd(); q = q{k};
    part_smd(N.xy(q(1), q(2)), zt, q(3), [1.0 0.5 0.45], q(4)*[0.12 0.12 0.12] + (1 - q(4))*[0.72 0.62 0.48]);
end
end

function [opt, tag] = local_m51()
% STM32 USB-C core board and FT232RL USB-UART adapter on the control board (electronics_ftdi_stm32_board.jpeg)
tag = 'stm32_ftdi_3d';
c = local_colors();
local_board([32 31], 64, 62, 0, 0, 1.6, c.green_edge, local_tex(64, 62, 24, c.green, @local_m51_tex));
part_dpak([6.5 7.0], 0, -90);
part_elec([16.5 6.2], 0, 0, 5, 7, [], []);
for x = [17.8 20.5 23.2], part_disc([x 12.0], 0, 0, 3.6); end
part_header([3.0 24.0], 0, 90, 3, 1, 6.0, c.tin, 2.5);
% FT232RL adapter (18.3 x 31 mm) on its 6-pin header
part_header([16 18.7], 0, 0, 6, 1, 0, c.gold, 2.5);
Fb = local_board([16 32], 18.3, 31, 0, 4.1, 1.6, [0.62 0.12 0.12], local_tex(18.3, 31, 40, c.red_pcb, @local_ftdi_tex));
zt = 4.1;
part_pintips(Fb.xy(9.15, 2.2), zt, 0, 6, 1, 0.8, c.gold);
part_socket(Fb.xy(9.15, 27.6), zt, 0, 7.7, 4.0, 9.2, 0.5, c.alu);
part_soic(Fb.xy(9.15, 15.2), zt, 90, 28, 0.65, 10.2, 5.3, 1.75, {'FTDI', 'FT232RL'});
part_header(Fb.xy(9.15, 6.6), zt, 0, 3, 1, 2.2, c.gold, 2.5);
part_housing(Fb.xy(10.42, 6.6), zt + 2.5, 0, [5.0 2.5 6.0], [0.05 0.05 0.06]);
for v = [24.6 22.8], part_smd(Fb.xy(4.0, v), zt, 0, [1.6 0.8 0.5], [0.95 0.92 0.86]); end
for v = [24.8 22.9 21.0], part_smd(Fb.xy(14.2, v), zt, 0, [1.6 0.8 0.5], [0.12 0.12 0.12]); end
part_smd(Fb.xy(4.0, 20.2), zt, 0, [1.6 0.8 0.5], [0.72 0.6 0.45]);
part_soic(Fb.xy(9.15, 22.6), zt, 0, 6, 0.95, 2.9, 1.6, 1.1);
% STM32 USB-C core board (22.86 x 53.3 mm) soldered on its two 20-pin headers
for u = [44.5 - 10.16, 44.5 + 10.16], part_header([u 31.5], 0, 90, 20, 1, 0, c.gold, 2.5); end
Bp = local_board([44.5 31.5], 22.86, 53.3, 0, 4.1, 1.6, [0.08 0.18 0.42], local_tex(22.86, 53.3, 32, c.blue_pcb, @local_bluepill_tex));
for u = [1.27 21.59], part_pintips(Bp.xy(u, 26.65), zt, 90, 20, 1, 0.7, c.tin); end
part_socket(Bp.xy(11.43, 50.8), zt, 0, 8.94, 3.26, 7.35, 1.5, c.alu);
part_button(Bp.xy(6.6, 43.6), zt, 0, [4.0 3.0], 1.3, 0.9, [0.08 0.08 0.08], [0.16 0.16 0.17]);
part_button(Bp.xy(16.3, 43.6), zt, 0, [4.0 3.0], 1.3, 0.9, [0.08 0.08 0.08], [0.16 0.16 0.17]);
part_qfp(Bp.xy(11.43, 29.5), zt, 0, 12, 0.5, 7, 1.4, 'STM32');
part_xtal(Bp.xy(15.6, 20.2), zt, 0, [5.0 3.2 1.1]);
part_xtal(Bp.xy(7.2, 20.2), zt, 90, [3.2 1.5 0.9]);
part_button(Bp.xy(11.43, 11.0), zt, 0, [5.2 5.2], 1.5, 1.45, [0.08 0.08 0.08], [0.18 0.18 0.19]);
part_led(Bp.xy(4.6, 11.0), zt, 90, [1 0.12 0.08]);
part_led(Bp.xy(18.3, 11.0), zt, 90, [0.25 0.45 1]);
part_soic(Bp.xy(17.2, 37.6), zt, 90, 6, 0.95, 2.9, 1.6, 1.1);
smd = [5.2 37 0; 5.2 35 0; 5.2 33 1; 17.6 33 0; 17.6 31 1; 5.2 25 0; 17.6 25 1; 11.4 37.2 1; 11.4 22.4 0; ...
    3.8 16 1; 7.6 16 0; 15.4 16 0; 19 16 1; 8.3 6.5 0; 14.5 6.5 1];
for k = 1:size(smd, 1)
    part_smd(Bp.xy(smd(k, 1), smd(k, 2)), zt, 90*(smd(k, 1) == 11.4), [1.0 0.5 0.45], ...
        smd(k, 3)*[0.12 0.12 0.12] + (1 - smd(k, 3))*[0.72 0.62 0.48]);
end
% labels
callout([44.5 31 5.7], [74 52 24], 'STM32 USB-C core board (LQFP48 MCU)', 'left');
callout([44.5 57.5 5.8], [62 74 16], 'USB-C', 'left');
callout([44.5 15.9 6.4], [76 18 14], 'BOOT0 / NRST / KEY buttons', 'left');
callout([16 31.7 6.0], [-14 54 24], 'FT232RL USB-UART adapter', 'right');
callout([17.3 22.6 12.7], [-14 34 22], '3.3 V / 5 V jumper', 'right');
callout([3.0 24.0 8.6], [-16 16 14], 'CUR: current-sensor header', 'right');
callout([6.5 7.0 2.4], [-10 -6 10], 'U4 (D-PAK)', 'right');
opt = struct('size', [1250 840], 'az', -22, 'el', 52, 'target', [30 29 3], 'dist', 330, 'va', 20, 'ground', -1.7, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_m51_tex(ax, s)
% control-board region around the two modules (64 x 62 mm)
c = local_colors(); tr = c.green_tr; w = c.silk;
for k = 0:5, t_trace(ax, s, [9.85 + 2.54*k 18.7; 9.85 + 2.54*k 15.5 - 0.6*k; 30 14.5 - 0.8*k; 34.34 14.5 - 0.8*k], 0.35, tr); end
t_trace(ax, s, [3 24; 3 30; 8 34; 8 50; 30 56; 34.34 55.6], 0.4, tr);
t_trace(ax, s, [8.5 7; 13 7; 14 6.2], 1.0, tr); t_trace(ax, s, [18.6 6.2; 26 6.2; 30 9.5; 34.34 9.5], 1.0, tr);
t_trace(ax, s, [17.8 10.7; 17.8 9.2; 23.2 9.2; 23.2 10.7], 0.5, tr);
for k = 0:4, t_trace(ax, s, [54.66 50.6 - 2.54*k; 60 47 - 2.54*k; 60 30 - 2.54*k; 54.66 27.8 - 2.54*k], 0.3, tr); end
t_box(ax, s, [6.5 7.0], 8.4, 8.6, 0, w); t_text(ax, s, [6.5 13.4], 'U4', 1.2, 0, w);
t_text(ax, s, [16.5 2.0], 'C1', 1.1, 0, w); t_box(ax, s, [16.5 6.2], 5.6, 5.6, 0, w);
t_text(ax, s, [20.5 14.6], 'C5   C6   C7', 0.95, 0, w);
t_box(ax, s, [3.0 24.0], 3.2, 8.2, 0, w); t_text(ax, s, [3.0 19.0], 'CUR', 1.1, 0, w);
for xy = [3 58.5; 61 3; 61 58.5]', t_ring(ax, xy', 3.4, 2.4); end
end

function local_ftdi_tex(ax, s)
% red FT232RL breakout (18.3 x 31 mm): side and bottom holes with their labels
w = [0.97 0.96 0.95];
lv = {'PWREN', 'SLEEP', 'CTS', '3.3V', '5V', 'RXL', 'TXL', 'GND'};
rv = {'DCD', 'DSR', 'GND', 'RI', 'RXD', 'VCC', 'RTS', 'DTR', 'TXD'};
for k = 1:8, v = 24.5 - (k - 1)*2.54; t_ring(ax, [1.6 v], 1.7, 0.9); t_text(ax, s, [2.8 v], lv{k}, 0.75, 0, w, 'left', 'normal'); end
for k = 1:9, v = 27.0 - (k - 1)*2.54; t_ring(ax, [16.7 v], 1.7, 0.9); t_text(ax, s, [15.5 v], rv{k}, 0.75, 0, w, 'right', 'normal'); end
bv = {'DTR', 'RX', 'TX', 'VCC', 'CTS', 'GND'};
for k = 1:6, t_text(ax, s, [9.15 + (k - 3.5)*2.54, 4.2], bv{k}, 0.62, 0, w, 'center', 'normal'); end
t_text(ax, s, [6.0 6.6], '5V', 0.7, 0, w); t_text(ax, s, [13.9 6.6], '3.3V', 0.7, 0, w);
t_box(ax, s, [9.15 15.2], 7.0, 11.6, 0, w, 0.12);
end

function local_bluepill_tex(ax, s)
% blue STM32 core board (22.86 x 53.3 mm): pin labels, buttons, LEDs, SWD pads
w = [0.97 0.97 0.98];
lp = {'B12', 'B13', 'B14', 'B15', 'A8', 'A9', 'A10', 'A11', 'A12', 'A15', 'B3', 'B4', 'B5', 'B6', 'B7', 'B8', 'B9', '5V', 'G', '3V3'};
rp = {'G', 'G', '3V3', 'R', 'B11', 'B10', 'B1', 'B0', 'A7', 'A6', 'A5', 'A4', 'A3', 'A2', 'A1', 'A0', 'C15', 'C14', 'C13', 'VB'};
for k = 1:20
    v = 26.65 + (10.5 - k)*2.54;
    t_text(ax, s, [2.6 v], lp{k}, 0.7, 0, w, 'left', 'normal'); t_text(ax, s, [20.3 v], rp{k}, 0.7, 0, w, 'right', 'normal');
end
t_text(ax, s, [6.6 41.2], 'BOOT0', 0.7, 0, w); t_text(ax, s, [16.3 41.2], 'NRST', 0.7, 0, w);
t_text(ax, s, [11.43 7.4], 'KEY', 0.8, 0, w); t_text(ax, s, [4.6 8.4], 'PWR', 0.7, 0, w); t_text(ax, s, [18.3 8.4], 'PB2', 0.7, 0, w);
sw = {'3V3', 'IO', 'CLK', 'G'};
for k = 1:4
    x = 11.43 + (k - 2.5)*2.54; t_ring(ax, [x 2.0], 1.6, 0.9); t_text(ax, s, [x 3.6], sw{k}, 0.55, 0, w, 'center', 'normal');
end
t_box(ax, s, [11.43 29.5], 9.0, 9.0, 0, w, 0.12);
end

function [opt, tag] = local_m52()
% ADS1115 / ADS1015 ADC module on the control board (electronics_ads1115_adc.jpg)
tag = 'ads1115_3d';
c = local_colors();
local_board([25 20], 50, 40, 0, 0, 1.6, c.green_edge, local_tex(50, 40, 30, c.green, @local_m52_tex));
a = 12;                                                           % module tilt as in the photo
Mo = struct('L', 12, 'W', 27.5); Mo.c = [26.5 20.5];
R = [cosd(a) -sind(a); sind(a) cosd(a)]; Mo.xy = @(u, v) (Mo.c(:) + R*[u - 6; v - 13.75]).';
part_header(Mo.xy(10.4, 13.75), 0, a + 90, 10, 1, 0, c.gold, 2.5);
local_board(Mo.c, 12, 27.5, a, 4.1, 1.6, [0.3 0.08 0.32], local_tex(12, 27.5, 48, c.purple, @local_ads_tex));
zt = 4.1;
part_pintips(Mo.xy(10.4, 13.75), zt, a + 90, 10, 1, 0.5, c.tin);
part_soic(Mo.xy(5.4, 18.4), zt, a + 90, 10, 0.5, 3.0, 3.0, 1.0);
for k = 0:3
    p = Mo.xy(4.6, 13.4 - 1.6*k);
    part_smd(p, zt, a, [1.6 0.8 0.45], [0.12 0.12 0.12]);
    local_mark(p, zt + 0.455, a, [0.9 0.75], '103', [0.9 0.9 0.9]);
end
part_smd(Mo.xy(4.6, 7.0), zt, a, [1.6 0.8 0.45], [0.72 0.62 0.48]);
% control board: R1-R4, headers J5-J7 and J4, diode
for k = 1:4, part_res([2.6 + 3.0*k, 7.0], 0, 78, 10.16, 'nkkon'); end
for k = 1:3, part_header([6.0 21.0 + 6.5*(k - 1)], 0, 0, 2, 1, 6.0, c.tin, 2.5); end
part_header([44.0 36.0], 0, 0, 2, 1, 6.0, c.tin, 2.5);
part_diode([45.0 24.0], 0, -90, 10.16);
callout([Mo.xy(2.4, 9.0) 4.15], [10 54 26], 'ADS1115 / ADS1015 module (16-bit, I2C, 4 inputs)', 'center');
callout([Mo.xy(5.4, 18.4) 5.3], [34 50 14], 'ADC chip (MSOP-10)', 'left');
callout([Mo.xy(10.4, 13.75) 4.8], [52 40 20], 'header: VDD GND SCL SDA ADDR ALRT A0-A3', 'left');
callout([8.6 7.0 2.6], [-10 -6 12], 'R1-R4', 'right');
opt = struct('size', [1400 820], 'az', -20, 'el', 52, 'target', [25 19 3], 'dist', 260, 'va', 17, 'ground', -1.7, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_m52_tex(ax, s)
% control-board region around the ADC module (50 x 40 mm)
c = local_colors(); tr = c.green_tr; w = c.silk;
R = [cosd(12) -sind(12); sind(12) cosd(12)];
P = ([-7.6 -15.4; 7.6 -15.4; 7.6 15.4; -7.6 15.4; -7.6 -15.4])*R.' + [26.5 20.5];
plot(ax, P(:, 1), P(:, 2), '-', 'Color', w, 'LineWidth', 0.2*s.ppm*s.pt);
for k = 1:4
    x = 2.6 + 3.0*k; t_text(ax, s, [x + 1.3, 13.4], sprintf('R%d', k), 1.2, 0, w);
    t_trace(ax, s, [x + 1.06 12.0; x + 1.06 16 + k; 18 18 + 1.2*k], 0.35, tr);
end
for k = 1:3, t_text(ax, s, [1.6 21.0 + 6.5*(k - 1)], sprintf('J%d', k + 4), 1.2, 0, w); t_box(ax, s, [6 21 + 6.5*(k - 1)], 5.6, 3.0, 0, w); end
t_text(ax, s, [38.5 36.5], 'J4', 1.2, 0, w); t_text(ax, s, [44.0 32.4], '12V', 1.1, 0, w);
t_box(ax, s, [45.0 24.0], 3.4, 7.0, 0, w); t_text(ax, s, [48.3 24.0], 'D3', 1.1, 90, w);
for k = 0:9
    p = [10.4 13.75 + (k - 4.5)*2.54] - [6 13.75]; p = (R*p(:)).' + [26.5 20.5];
    t_trace(ax, s, [p; p + [6 0.8]; 47 2 + 2.6*k], 0.3, tr);
end
t_trace(ax, s, [12 23; 16 27; 20 27], 0.3, tr); t_trace(ax, s, [12 29.5; 16 33; 22 33], 0.3, tr);
end

function local_ads_tex(ax, s)
% purple ADS1115 module (12 x 27.5 mm): holes, label, header labels
w = [0.97 0.95 0.97];
for v = [3.0 24.5], t_disc(ax, [4.6 v], 5.2, w); t_disc(ax, [4.6 v], 4.6, [0.4 0.12 0.42]); t_ring(ax, [4.6 v], 3.6, 3.0); end
t_text(ax, s, [1.6 13.75], 'ADS1115/ADS1015', 1.25, -90, w);
lb = {'VDD', 'GND', 'SCL', 'SDA', 'ADDR', 'ALRT', 'A0', 'A1', 'A2', 'A3'};
for k = 1:10, t_text(ax, s, [9.3 13.75 + (5.5 - k)*2.54], lb{k}, 0.5, 0, w, 'right', 'normal'); end
t_rect(ax, [2.6 21.4], 0.9, 0.9, 0, w); t_rect(ax, [6.6 4.2], 0.9, 0.8, 0, w);
end

function [opt, tag] = local_m53()
% LM2596 DC-DC module (43.2 x 21 mm): (a) top side, typical layout of this module; (b) bottom side as photographed
tag = 'lm2596_3d';
c = local_colors();
A = local_board([-25 0], 43.2, 21, 0, 0, 1.6, [0.02 0.27 0.38], local_tex(43.2, 21, 40, c.lm_blue, @local_lm_top_tex));
zt = 0;
part_trim(A.xy(13.6, 3.9), zt, 0);
part_vchip(A.xy(5.0, 11.2), zt, 0, 6.3, 7.7);
part_vchip(A.xy(39.0, 13.6), zt, 0, 6.3, 7.7);
part_drum(A.xy(17.0, 13.0), zt, 0, 11.6, 7.0, '330');
part_to263(A.xy(28.8, 11.5), zt, -90, {'LM2596S', 'ADJ'});
part_sma(A.xy(30.6, 3.4), zt, 0);
part_smd(A.xy(23.5, 3.0), zt, 0, [2.0 1.25 0.6], [0.72 0.62 0.48]);
part_led(A.xy(23.5, 18.6), zt, 0, [1 0.15 0.1]);
B = local_board([25 0], 43.2, 21, 0, 0, 1.6, [0.02 0.27 0.38], local_tex(43.2, 21, 40, c.lm_blue, @local_lm_bottom_tex));
for x = [11 13.6 16.2], local_joint([0 0], B.xy(x, 17.1), 0, 0); end
callout([A.xy(28.8, 12.0) 4.5], [-34 34 22], 'LM2596S buck regulator (TO-263)', 'center');
callout([A.xy(17.0, 15.5) 7.0], [-58 22 20], ['power inductor 33 ' char(181) 'H'], 'right');
callout([A.xy(10.6, 3.9) 10.6], [-60 -10 18], 'trimmer: sets the output voltage', 'right');
callout([A.xy(39.0, 13.6) 7.7], [-6 30 26], 'input capacitor', 'center');
callout([A.xy(30.6, 3.4) 2.1], [-14 -22 6], 'Schottky diode SS34', 'center');
opt = struct('size', [1350 760], 'az', -12, 'el', 52, 'target', [0 2 2], 'dist', 300, 'va', 21, 'ground', -1.7, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_lm_top_tex(ax, s)
% top side: corner pads with their labels, mounting holes
w = [0.95 0.96 0.97];
t_ring(ax, [4.8 16.7], 4.4, 3.2); t_ring(ax, [36.4 4.2], 4.4, 3.2);
pads = [1.4 1.6; 1.4 19.4; 41.8 1.6; 41.8 19.4]; lbl = {'OUT+', 'OUT-', 'IN+', 'IN-'};
for k = 1:4, t_ring(ax, pads(k, :), 2.2, 1.0); end
t_text(ax, s, [3.2 1.6], 'OUT+', 0.9, 0, w, 'left'); t_text(ax, s, [8.4 19.4], 'OUT-', 0.9, 0, w, 'left');
t_text(ax, s, [40.0 1.6], 'IN+', 0.9, 0, w, 'right'); t_text(ax, s, [40.0 19.4], 'IN-', 0.9, 0, w, 'right');
t_box(ax, s, [13.6 3.9], 10.0, 5.2, 0, w); t_box(ax, s, [28.8 11.5], 14.0, 11.0, 0, w);
end

function local_lm_bottom_tex(ax, s)
% bottom side as in electronics_lm2596_buck.jpg: arrow, LM2596 DC-DC, pad labels, holes, vias
w = [0.95 0.96 0.97];
t_ring(ax, [4.8 4.3], 3.6, 3.2); t_ring(ax, [36.4 16.8], 3.6, 3.2);
for p = [1.1 19.4; 1.2 3.9; 41.2 17.4; 41.2 1.3]', t_ring(ax, p', 1.4, 0.8); end
for p = [11 17.1; 13.6 17.1; 16.2 17.1]', t_ring(ax, p', 1.9, 0.8); end
for p = [2.6 13.2; 2.6 10.5; 38.2 10.9; 38.2 8.6; 1.4 8.8; 2.6 8.2; 3.6 7.6; 4.4 8.6]', t_ring(ax, p', 0.8, 0.35); end
t_text(ax, s, [0.6 17.0], 'OUT+', 1.0, 0, w, 'left'); t_text(ax, s, [0.4 6.2], 'OUT-', 1.0, 0, w, 'left');
t_text(ax, s, [42.6 15.1], 'IN+', 1.0, 0, w, 'right'); t_text(ax, s, [42.6 3.9], 'IN-', 1.0, 0, w, 'right');
patch(ax, [15.4 19.6 19.6 25.0 25.0 19.6 19.6], [12.2 15.4 13.0 13.0 11.4 11.4 9.0], w, 'EdgeColor', 'none');
t_text(ax, s, [21.0 7.0], 'LM2596 DC-DC', 1.9, 0, w);
t_trace(ax, s, [16.2 17.1; 20 15.5; 27 15.5], 0.6, [0.06 0.42 0.56]);
end

function [opt, tag] = local_m54()
% detail of the XMachines SPR V1.3 sprayer board (electronics_xmachines_spr_v1_3_board.jpg)
tag = 'spr_board_3d';
c = local_colors();
local_board([20 25], 40, 50, 0, 0, 1.6, c.green_edge, local_tex(40, 50, 30, c.green, @local_spr_tex));
part_res([28.6 36.6], 0, 11, 17.0, 'neekn');
part_res([24.1 20.9], 0, 113, 18.0, 'nkkon');
part_diode([22.6 5.8], 0, 20, 19.0);
part_soic([34.6 25.2], 0, 20, 4, 2.54, 6.6, 6.0, 3.4);
for k = 1:2
    p = [3.6 13.0; 6.0 7.0]; p = p(k, :);
    [V, F] = p_ell([0 0 1.6], [3.6 1.75 1.5], 20); put('smooth', V, F, [0.80 0.50 0.20], p, 0, 14);
    P = g_spline([3.4 0 1.6; 4.6 0 1.4; 5.4 0 0.6; 5.6 0 -0.2], 0.3);
    [V, F] = p_tube(P, 0.25, 10); put('msmooth', V, F, c.tin, p, 0, 14);
    local_joint([5.6 0], p, 0, 14);
end
callout([15 21 0.05], [4 60 18], 'XMachines SPR V1.3 sprayer board', 'center');
callout([34.6 25.2 3.6], [54 38 16], 'SMD IC (4-pin package)', 'left');
callout([28.6 36.6 2.6], [44 58 14], 'R9, R21: 1 % metal-film resistors', 'left');
callout([22.6 5.8 2.8], [50 -6 12], 'D5: axial diode', 'left');
callout([4.0 12.0 3.2], [-14 0 12], 'C7, C8', 'right');
opt = struct('size', [1150 820], 'az', -14, 'el', 56, 'target', [20 24 2], 'dist', 260, 'va', 18.5, 'ground', -1.7, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_spr_tex(ax, s)
% SPR V1.3 region of the photo (40 x 50 mm)
c = local_colors(); tr = c.green_tr; w = c.silk;
for xy = {[0 44; 9 40.5; 19 31.5; 22 29.5], [0 30; 9 28.5; 18 25.8; 26 23], [0 19; 10 18.8; 15 16; 24.5 12], ...
        [7 0; 11 6; 13.5 9.6], [20.5 50; 22 42; 20.1 35], [40 31; 37.5 29], [40 14; 33 9; 31 8.8], [2 50; 4 46; 0.8 42]}
    t_trace(ax, s, xy{1}, 0.45, tr);
end
t_text(ax, s, [14.2 23.0], 'XMACHINES', 2.4, -76, w); t_text(ax, s, [11.4 15.6], 'SPR V1.3', 2.0, -76, w);
t_text(ax, s, [3.2 44.6], 'R14', 1.4, -76, w); t_text(ax, s, [4.4 35.8], 'R16 R17', 1.4, -76, w);
t_text(ax, s, [36.6 41.4], 'R9', 1.5, 0, w); t_text(ax, s, [27.0 26.6], 'R21', 1.4, -76, w);
t_text(ax, s, [3.6 17.0], 'C7', 1.4, 0, w); t_text(ax, s, [3.4 3.4], 'C8', 1.4, 180, w);
t_text(ax, s, [33.4 4.4], 'D5', 1.4, 180, w);
t_box(ax, s, [28.6 36.6], 9.0, 3.6, 11, w); t_box(ax, s, [24.1 20.9], 8.6, 3.4, 113, w);
t_box(ax, s, [22.6 5.8], 8.0, 3.8, 20, w); t_box(ax, s, [34.6 25.2], 9.4, 8.0, 20, w);
t_box(ax, s, [29.6 47.3], 17.6, 8.6, 0, w);
for p = [0.6 33; 1.0 38.6; 37.6 38.4; 19.6 34.8; 21.9 30.0; 26.3 11.8; 13.7 2.6; 31.5 9.0]', t_ring(ax, p', 2.4, 0.9); end
end

function [opt, tag] = local_m55()
% electronics box of the robot (inner 104 x 156 x 50 mm) as in electronics_box_xt_ps_nucleo.jpeg
tag = 'electronics_box_3d';
c = local_colors(); bk = [0.11 0.11 0.12]; t = 1.5; H = 50; Wb = 104; Lb = 156;
[V, F] = p_box([Wb/2 Lb/2 -t], [Wb + 2*t, Lb + 2*t, t]); put('plastic', V, F, bk, [0 0], 0, 0);
for w = [Wb/2 -t/2 Wb + 2*t t; Wb/2 Lb + t/2 Wb + 2*t t; -t/2 Lb/2 t Lb; Wb + t/2 Lb/2 t Lb]'
    [V, F] = p_box([w(1) w(2) 0], [w(3) w(4) H]); put('plastic', V, F, bk, [0 0], 0, 0);
end
G('shadow', [-t -t; Wb + t -t; Wb + t Lb + t; -t Lb + t], H);
% XT_PS board with the Nucleo-32 on 6 mm standoffs
G('offset', [25.5 62.3 7.6]); local_xtps_geometry(false); G('offset', [0 0 0]);
for p = [36.8 132.2; 60.2 133.6]'
    [V, F] = p_cyl([p' 0], [p' 22], 2.9, 6); put('metal', V, F, c.brass, [0 0], 0, 0);
end
% perfboard with the optocouplers and the 8-channel level shifter
Pb = local_board([47.3 35.45], 62, 42.5, 0, 7.6, 1.6, [0.62 0.52 0.34], local_tex(62, 42.5, 16, [0.80 0.72 0.52], @local_perf_tex));
for v = [27.7 12.0], part_soic(Pb.xy(21.1, v), 7.6, 0, 4, 2.54, 4.6, 6.5, 3.5); end
rs = [11.5 31 30; 30.5 24.5 -30; 11.5 17 20; 30.5 9.5 -20];
for k = 1:4, part_res(Pb.xy(rs(k, 1), rs(k, 2)), 7.6, rs(k, 3), 10.16, 'nkkrn'); end
part_header(Pb.xy(49.5 - 7.62, 18.4), 7.6, 90, 10, 1, 0, c.gold, 2.5);
part_header(Pb.xy(49.5 + 7.62, 18.4), 7.6, 90, 10, 1, 0, c.gold, 2.5);
Ls = local_board(Pb.xy(49.5, 18.4), 20, 28, 0, 11.7, 1.6, [0.6 0.1 0.1], local_tex(20, 28, 30, c.red_pcb, @local_shifter_tex));
for k = 1:8
    part_soic(Ls.xy(10, 24.3 - 2.9*(k - 1)), 11.7, 0, 4, 0.95, 2.9, 1.3, 1.0);
    for s = [-1 1], part_smd(Ls.xy(10 + 5.2*s, 24.3 - 2.9*(k - 1)), 11.7, 0, [1.6 0.8 0.45], [0.12 0.12 0.12]); end
end
for u = [1.27 18.73], part_pintips(Ls.xy(u, 14), 11.7, 90, 10, 1, 0.6, c.tin); end
% cable conduits and wires
cv = [0.08 0.08 0.09];
local_conduit([26 -40 18; 26 -4 18; 27 4 16; 28 8 14], 6.5, cv);
local_conduit([49 -40 18; 49 -4 18; 50 8 15; 50 12 14], 6.0, cv);
local_conduit([140 120 22; 108 120 22; 96 118 20; 90 116 18], 6.5, cv);
local_conduit([8 196 24; 8 160 24; 10 150 22; 14 146 20], 7.0, cv);
local_wire([14 146 20; 22 130 18; 28 118 16; 30 114 12], 0.9, [0.8 0.1 0.08]);
local_wire([16 145 19; 25 131 17; 32.5 117 15; 32.8 114 12], 0.9, [0.1 0.1 0.1]);
local_twisted([28 8 14; 20 30 18; 14 60 20; 24 80 18; 28.9 92 17], [0.95 0.75 0.1; 0.15 0.45 0.85], 0.55, 6);
local_twisted([50 12 14; 48 30 18; 52 48 18; 60 54 16], [0.2 0.6 0.25; 0.95 0.95 0.95], 0.55, 6);
local_wire([90 116 18; 94 100 16; 96 80 14; 93 70 12], 0.8, [0.95 0.8 0.1]);
local_wire([90 117 18; 95 102 17; 97.5 82 15; 95 66 13], 0.8, [0.15 0.45 0.85]);
local_wire([91 115 17; 97 98 16; 99 76 15; 92 52 15; 77 42 13.4], 0.8, [0.2 0.6 0.25]);
callout([60 92 8], [70 186 60], 'XT_PS V1.0 control board + STM32 Nucleo-32', 'center');
callout([37.4 41.9 11.3], [-6 34 46], 'perfboard: optocouplers and resistors', 'right');
callout([65.8 30.0 13.0], [110 44 46], '8-channel logic-level shifter', 'left');
callout([26 -20 24], [-6 -16 42], 'cable conduits to the robot', 'right');
opt = struct('size', [1150 960], 'az', -18, 'el', 60, 'target', [52 74 10], 'dist', 640, 'va', 22, 'ground', -1.6, ...
    'light', [-0.5 -0.9 1.5; 1 0.35 0.7; 0.2 1 0.45], 'lightc', [0.78 0.38 0.22]);
end

function local_conduit(P, r, col)
% corrugated cable conduit along the path P
Q = g_spline(P, 0.8);
s = [0; cumsum(vecnorm(diff(Q), 2, 2))];
[V, F] = p_tube(Q, r, 20);
n = 20; ring = repelem((1:size(Q, 1) - 1)', n);
rr = 0.75 + 0.5*(0.5 + 0.5*cos(2*pi*s/3.2));              % ribs every 3.2 mm (shading only)
col2 = min(1, col.*rr(ring) + 0.04*(rr(ring) - 0.75));
put('smooth', V, F, [col2; repmat(col, size(F, 1) - numel(ring), 1)], [0 0], 0, 0);
end

function local_perf_tex(ax, s)
% perfboard: 2.54 mm hole grid with tin pads
for x = 1.27:2.54:62
    for y = 1.27:2.54:42.5, t_ring(ax, [x y], 1.6, 0.8); end
end
end

function local_shifter_tex(ax, s)
% red level-shifter module (20 x 28 mm)
w = [0.97 0.96 0.95];
for k = 1:10
    v = 14 + (5.5 - k)*2.54;
    t_text(ax, s, [3.0 v], sprintf('A%d', min(k, 8)), 0.6, 0, w, 'left', 'normal');
    t_text(ax, s, [17.0 v], sprintf('B%d', min(k, 8)), 0.6, 0, w, 'right', 'normal');
end
t_text(ax, s, [10 1.4], 'LV          HV', 0.7, 0, w);
end

function q = local_nucleo_smd()
% small passives on the Nucleo-32: {u v ang dark(1 = resistor)}
q = {[3.6 41 0 1], [3.6 39 0 0], [3.6 34 0 1], [3.6 33 0 0], [14.9 41 0 0], [14.9 39 0 1], [14.9 35 0 0], ...
    [7.0 31.6 90 0], [11.5 31.6 90 1], [3.6 25 0 0], [3.6 23 0 1], [3.6 16 0 0], [14.9 25 0 1], [14.9 23 0 0], ...
    [14.9 16 0 1], [14.9 14 0 0], [7 14.5 90 0], [11.5 14.5 90 1], [6 9 0 1], [9.5 9 0 0], [13 9 0 1], [9.3 6.2 0 0], ...
    [6.5 47.2 0 0], [12 47.2 0 1]};
end

function local_xtps_tex(ax, s)
% silkscreen, copper and pads of the XT_PS V1.0 board (68 x 56 mm)
c = local_colors(); tr = c.green_tr; w = c.silk;
% traces: power along the top, CAN pairs, logic to the Nucleo headers
t_trace(ax, s, [6 49; 6 46; 22 46; 25.5 44.7], 1.2, tr); t_trace(ax, s, [10.5 52; 9 49; 9 47.5], 0.8, tr);
t_trace(ax, s, [16 52; 17.5 49.5; 27 49.5; 28 48.5], 0.8, tr); t_trace(ax, s, [33.2 51.6; 36 47; 44 47; 48 44], 0.8, tr);
for k = 0:3
    t_trace(ax, s, [3.4 26.75 + 2.5*k; 8 26.75 + 2.5*k; 10 24.8 + 0.6*k; 14.5 24.8 + 0.6*k], 0.35, tr);
    t_trace(ax, s, [3.4 11.75 + 2.5*k; 7 11.75 + 2.5*k; 8 12.5 + k; 8 22; 13.5 23.2], 0.35, tr);
end
for k = 0:3
    y = 23.6 + 0.65*k; t_trace(ax, s, [20.5 y; 24 y; 25.5 29 + 2*k; 44 29 + 2*k; 46 30.8 + 0*k; 50.2 30.8 - 2.54*k], 0.3, tr);
end
for k = 0:5, t_trace(ax, s, [21 36.5 - 0.6*k; 40 36.5 - 0.6*k; 46 40.7 - 2.54*k; 50.2 40.7 - 2.54*k], 0.3, tr); end
t_trace(ax, s, [21 18.9; 26 18.9; 26 17.8; 50.2 17.9], 0.3, tr); t_trace(ax, s, [21 11.9; 24 11.9; 24 15.3; 50.2 15.3], 0.3, tr);
t_trace(ax, s, [10.2 21.5; 10.2 23; 14 23], 0.3, tr); t_trace(ax, s, [12.9 6.3; 12.9 4.5; 32.2 4.5; 32.2 6.5], 0.4, tr);
t_trace(ax, s, [27.6 16.7; 27.6 18.2; 44 18.2; 50.2 20.4], 0.3, tr); t_trace(ax, s, [36.6 8.7; 40 8.7; 50.2 10.2], 0.4, tr);
% header pads and the Nucleo pin labels (CN4 order of the Nucleo-32)
pins = {'D13', '3V3', 'REF', 'A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', '5V', 'RST', 'GND', 'VIN'};
for k = 1:15
    v = 45.80 - (k - 1)*2.54;
    t_ring(ax, [50.18 v], 1.7, 1.0); t_ring(ax, [65.42 v], 1.7, 1.0);
    t_text(ax, s, [47.9 v], pins{k}, 1.05, 0, w, 'right');
end
% silkscreen: board name upside down at the bottom edge, connectors, references
t_text(ax, s, [38.6 1.7], 'XMACHINES', 1.5, 180, w); t_text(ax, s, [39.6 3.8], 'XT_PS V1.0', 1.25, 180, w);
t_text(ax, s, [8.3 39.0], '+48V_IN', 1.25, 90, w); t_box(ax, s, [6 51.5], 8.4, 6.6, 0, w);
t_text(ax, s, [7.4 37.6], 'CAN', 1.2, 90, w); t_text(ax, s, [7.4 22.6], 'CAN', 1.2, 90, w);
t_box(ax, s, [3.4 30.5], 6.6, 13.2, 0, w); t_box(ax, s, [3.4 15.5], 6.6, 13.2, 0, w);
t_box(ax, s, [13.5 52], 6.6, 4.4, 0, w); t_text(ax, s, [13.5 48.9], 'U1', 1.0, 0, w);
t_text(ax, s, [30.2 47.8], 'L1', 1.0, 0, w); t_text(ax, s, [19.3 48.9], 'C4', 1.0, 0, w); t_text(ax, s, [24 48.9], 'D2', 1.0, 0, w);
t_box(ax, s, [25.5 42.2], 5.6, 5.6, 0, w); t_text(ax, s, [22 38.4], 'C1', 1.1, 0, w);
t_text(ax, s, [31.0 39.6], 'C3', 1.0, 0, w); t_text(ax, s, [37.6 37.3], 'R1', 1.0, 0, w);
t_box(ax, s, [14.5 43.6], 7, 3.2, 0, w); t_text(ax, s, [14.5 41.2], 'D1', 1.0, 0, w);
t_box(ax, s, [18.0 34.0], 6.6, 5.6, 0, w); t_text(ax, s, [18.0 30.3], 'U9', 1.1, 0, w);
t_box(ax, s, [33.0 33.5], 9.0, 3.6, 0, w); t_rect(ax, [30.6 33.5], 1.6, 2.2, 0, c.tin); t_rect(ax, [35.4 33.5], 1.6, 2.2, 0, c.tin);
t_text(ax, s, [33.0 36.4], 'X1', 1.1, 0, w); t_rect(ax, [33 33.5], 1.0, 0.6, 0, c.tin);
t_box(ax, s, [17.0 25.6], 6.6, 5.6, 0, w); t_text(ax, s, [12.2 25.6], 'U3', 1.1, 0, w);
for k = 1:8
    x = 32.5 + (k - 4.5)*1.27;
    t_rect(ax, [x 26.4], 0.6, 1.8, 0, c.tin); t_rect(ax, [x 19.6], 0.6, 1.8, 0, c.tin);
end
t_box(ax, s, [32.5 23], 11.6, 9.2, 0, w); t_text(ax, s, [40.0 23], 'U6', 1.1, 0, w); t_disc(ax, [27.5 18.9], 0.5, w);
t_box(ax, s, [18.6 17.6], 6.0, 5.4, 0, w); t_text(ax, s, [23.6 18.6], 'U5', 1.1, 0, w);
t_box(ax, s, [18.6 10.6], 6.0, 5.4, 0, w); t_text(ax, s, [23.6 9.4], 'U7', 1.1, 0, w);
t_text(ax, s, [10.2 9.6], 'R5', 1.0, 90, w); t_text(ax, s, [12.9 9.6], 'R6', 1.0, 90, w);
t_text(ax, s, [27.6 18.4], 'R3', 1.0, 0, w); t_text(ax, s, [29.9 18.4], 'R2', 1.0, 0, w); t_text(ax, s, [32.2 18.4], 'R4', 1.0, 0, w);
t_text(ax, s, [36.6 6.6], 'C2', 1.1, 0, w); t_box(ax, s, [36.6 10.0], 5.4, 2.6, 0, w);
t_box(ax, s, [40.2 52.2], 11.2, 9.6, 0, w); t_text(ax, s, [46.6 52.6], 'J2', 1.0, 0, w);
for xy = [3 3; 3 53; 46 3]', t_ring(ax, xy', 3.4, 2.4); end
end

function local_nucleo_tex(ax, s)
% white solder mask of the Nucleo-32 (18.54 x 50.04 mm): header pads, labels, ST logo
k = [0.12 0.12 0.14]; tr = [0.86 0.86 0.88];
for v = 25.02 + ((1:15) - 8)*2.54
    t_trace(ax, s, [1.65 v; 3.2 v], 0.3, tr); t_trace(ax, s, [16.89 v; 15.3 v], 0.3, tr);
end
lbl = {'D13', '3V3', 'AREF', 'A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', '5V', 'RST', 'GND', 'VIN'};
lbr = {'D12', 'D11', 'D10', 'D9', 'D8', 'D7', 'D6', 'D5', 'D4', 'D3', 'D2', 'GND', 'RST', 'RX', 'TX'};
for j = 1:15
    v = 42.80 - (j - 1)*2.54;
    t_text(ax, s, [3.0 v], lbl{j}, 0.62, 0, k, 'left', 'normal'); t_text(ax, s, [15.5 v], lbr{j}, 0.62, 0, k, 'right', 'normal');
end
t_box(ax, s, [9.27 37.2], 8.6, 8.6, 0, k, 0.12); t_box(ax, s, [9.27 20.5], 8.6, 8.6, 0, k, 0.12);
t_text(ax, s, [5.2 43.6], 'LD1', 0.6, 0, k, 'center', 'normal'); t_text(ax, s, [13.4 43.6], 'LD3', 0.6, 0, k, 'center', 'normal');
t_text(ax, s, [12.6 3.6], 'LD2', 0.6, 0, k, 'center', 'normal'); t_text(ax, s, [4.4 6.9], 'B1', 0.6, 0, k, 'center', 'normal');
t_text(ax, s, [9.27 12.0], 'ST', 2.4, 0, [0.02 0.22 0.55], 'center', 'bold');
t_text(ax, s, [9.27 2.2], 'CN4                CN3', 0.6, 0, k, 'center', 'normal');
end

% =========================================================================================== scene store
function out = G(cmd, varargin)
% meshes per material, textured planes, callouts and shadow casters of the scene being built
persistent M T C S O
out = [];
switch cmd
    case 'reset'
        M = struct('plastic', {{}}, 'metal', {{}}, 'smooth', {{}}, 'msmooth', {{}}, 'glow', {{}});
        T = {}; C = {}; S = {}; O = [0 0 0];
    case 'nocallouts', C = {};
    case 'offset', O = varargin{1};          % everything added next is moved by O (reuse of a board in a scene)
    case 'mesh', M.(varargin{1}){end+1} = {varargin{2} + O, varargin{3}, varargin{4}};
    case 'tex'
        t = varargin{1}; t.X = t.X + O(1); t.Y = t.Y + O(2); t.Z = t.Z + O(3); T{end+1} = t;
    case 'callout', C{end+1} = [{varargin{1} + O, varargin{2} + O}, varargin(3:end)];
    case 'shadow', S{end+1} = {varargin{1} + O(1:2), varargin{2} + O(3)};   % polygon (n x 2), height
    case 'get', out = struct('M', M, 'T', {T}, 'C', {C}, 'S', {S});
end
end

function put(mat, V, F, col, xy, z, ang)
% add a mesh built in part coordinates (origin on the board surface) at xy, height z, rotation ang [deg]
if nargin < 7, ang = 0; end
R = [cosd(ang) -sind(ang) 0; sind(ang) cosd(ang) 0; 0 0 1];
V = V*R.' + [xy(1) xy(2) z];
if size(col, 1) == 1, col = repmat(col, size(F, 1), 1); end
G('mesh', mat, V, F, col);
end

function c = local_colors()
c.green = [0.06 0.40 0.18]; c.green_tr = [0.11 0.50 0.24]; c.green_edge = [0.45 0.55 0.32];
c.silk = [0.96 0.96 0.94]; c.tin = [0.80 0.80 0.80]; c.gold = [0.86 0.70 0.32]; c.alu = [0.80 0.81 0.83];
c.ic_black = [0.09 0.09 0.10]; c.hdr_black = [0.07 0.07 0.08]; c.res_blue = [0.42 0.74 0.88];
c.res_beige = [0.86 0.78 0.60]; c.cer_orange = [0.86 0.50 0.16]; c.elec_black = [0.10 0.10 0.12];
c.elec_stripe = [0.62 0.62 0.68]; c.jst = [0.95 0.93 0.86]; c.ferrite = [0.13 0.13 0.14];
c.copper = [0.80 0.48 0.22]; c.brass = [0.83 0.67 0.30]; c.trim_blue = [0.16 0.36 0.78];
c.nucleo = [0.94 0.94 0.95]; c.blue_pcb = [0.10 0.27 0.62]; c.red_pcb = [0.78 0.09 0.11];
c.purple = [0.40 0.12 0.42]; c.lm_blue = [0.02 0.36 0.50];
c.band = struct('k', [0.05 0.05 0.05], 'n', [0.45 0.24 0.10], 'r', [0.80 0.10 0.10], 'o', [0.95 0.50 0.10], ...
    'y', [0.95 0.85 0.10], 'g', [0.10 0.60 0.20], 'b', [0.10 0.30 0.80], 'v', [0.50 0.20 0.60], ...
    'e', [0.50 0.50 0.50], 'w', [0.95 0.95 0.95], 'd', [0.80 0.65 0.30]);
c.navy = [0.17 0.18 0.49];
end

% =========================================================================================== primitives
function [V, F] = p_box(c, s)
% box: centre c(1:2), bottom c(3), size s
x = [-1 1 1 -1 -1 1 1 -1]'*s(1)/2 + c(1);
y = [-1 -1 1 1 -1 -1 1 1]'*s(2)/2 + c(2);
z = [0 0 0 0 1 1 1 1]'*s(3) + c(3);
V = [x y z];
F = [1 4 3 2; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function P = g_rrect(w, h, r, n)
% rounded rectangle centred at 0, corner radius r, n points per corner, counter-clockwise
r = min(r, min(w, h)/2 - 1e-6);
cx = [w/2 - r, -w/2 + r, -w/2 + r, w/2 - r]; cy = [h/2 - r, h/2 - r, -h/2 + r, -h/2 + r];
P = zeros(0, 2);
for k = 1:4
    th = linspace((k - 1)*pi/2, k*pi/2, n)';
    P = [P; cx(k) + r*cos(th), cy(k) + r*sin(th)]; %#ok<AGROW>
end
end

function [V, F] = p_extrude(P, z0, h)
% prism of the convex polygon P (n x 2) from z0 to z0 + h; the caps have their own vertices
n = size(P, 1); o = ones(n, 1); c = mean(P, 1);
V = [P z0*o; P (z0 + h)*o; P z0*o; P (z0 + h)*o; c z0; c z0 + h];
i = (1:n)'; j = [2:n 1]';
F = [i j j + n i + n; j + 2*n, i + 2*n, (4*n + 1)*o, (4*n + 1)*o; i + 3*n, j + 3*n, (4*n + 2)*o, (4*n + 2)*o];
end

function [V, F] = p_cyl(p0, p1, r, n, caps)
% cylinder or cone from p0 to p1, radius r (scalar or [r0 r1]), n sides; side faces come first
if nargin < 4 || isempty(n), n = 24; end
if nargin < 5, caps = true; end
if isscalar(r), r = [r r]; end
d = (p1 - p0)/norm(p1 - p0);
a = [1 0 0]; if abs(dot(a, d)) > 0.9, a = [0 1 0]; end
u = cross(d, a); u = u/norm(u); w = cross(d, u);
th = (0:n - 1)'*2*pi/n; e = cos(th)*u + sin(th)*w;
V = [p0 + r(1)*e; p1 + r(2)*e];
i = (1:n)'; j = [2:n 1]';
F = [i j j + n i + n];
if caps
    o = ones(n, 1);
    V = [V; p0 + r(1)*e; p1 + r(2)*e; p0; p1];
    F = [F; j + 2*n, i + 2*n, (4*n + 1)*o, (4*n + 1)*o; i + 3*n, j + 3*n, (4*n + 2)*o, (4*n + 2)*o];
end
end

function [V, F] = p_ell(c, r, n)
% ellipsoid, centre c, semi-axes r
[x, y, z] = sphere(n);
[F, V] = surf2patch(r(1)*x + c(1), r(2)*y + c(2), r(3)*z + c(3));
end

function [V, F] = p_tube(P, r, n)
% tube of radius r along the polyline P (m x 3) with parallel-transported frames and closed ends
if nargin < 3, n = 12; end
m = size(P, 1);
T = zeros(m, 3);
T(1, :) = P(2, :) - P(1, :); T(m, :) = P(m, :) - P(m - 1, :);
if m > 2, T(2:m - 1, :) = P(3:m, :) - P(1:m - 2, :); end
T = T./vecnorm(T, 2, 2);
a = [0 0 1]; if abs(dot(a, T(1, :))) > 0.9, a = [1 0 0]; end
u = cross(T(1, :), a); u = u/norm(u);
th = (0:n - 1)'*2*pi/n;
V = zeros(m*n, 3);
for k = 1:m
    u = u - dot(u, T(k, :))*T(k, :); u = u/norm(u);
    w = cross(T(k, :), u);
    V((k - 1)*n + (1:n), :) = P(k, :) + r*(cos(th)*u + sin(th)*w);
end
i = (1:n)'; j = [2:n 1]';
F = zeros((m - 1)*n, 4);
for k = 1:m - 1, F((k - 1)*n + (1:n), :) = [i j j + n i + n] + (k - 1)*n; end
o = ones(n, 1); nv = size(V, 1);
V = [V; V(1:n, :); V(nv - n + 1:nv, :); P(1, :); P(m, :)];
F = [F; j + nv, i + nv, (nv + 2*n + 1)*o, (nv + 2*n + 1)*o; i + nv + n, j + nv + n, (nv + 2*n + 2)*o, (nv + 2*n + 2)*o];
end

function Q = g_spline(P, ds)
% smooth path through the points P (m x 3), resampled about every ds mm
s = [0; cumsum(vecnorm(diff(P), 2, 2))];
Q = interp1(s, P, linspace(0, s(end), max(8, ceil(s(end)/ds)))', 'makima');
end

% =========================================================================================== textures
function img = local_tex(L, W, ppm, base, drawfn)
% texture of an L x W mm face, ppm pixels per mm, drawn in mm coordinates by drawfn(ax, s)
f = figure('Visible', 'off', 'Units', 'pixels', 'Position', [10 10 round(L*ppm) round(W*ppm)], 'Color', base, ...
    'InvertHardcopy', 'off');
ax = axes(f, 'Units', 'normalized', 'Position', [0 0 1 1], 'XLim', [0 L], 'YLim', [0 W], 'Visible', 'off');
hold(ax, 'on');
s = struct('ppm', ppm, 'L', L, 'W', W, 'pt', 72/get(groot, 'ScreenPixelsPerInch'));
drawfn(ax, s);
ax.XLim = [0 L]; ax.YLim = [0 W];
img = print(f, '-RGBImage', '-r0');
close(f);
end

function t_trace(ax, s, xy, w, col)
plot(ax, xy(:, 1), xy(:, 2), '-', 'Color', col, 'LineWidth', w*s.ppm*s.pt);
end

function t_rect(ax, xy, w, h, ang, col)
% filled rectangle (pad, mark) centred at xy, rotated by ang
P = [-w -h; w -h; w h; -w h]/2*[cosd(ang) sind(ang); -sind(ang) cosd(ang)] + xy;
patch(ax, P(:, 1), P(:, 2), col, 'EdgeColor', 'none');
end

function t_disc(ax, xy, d, col)
th = linspace(0, 2*pi, 40);
patch(ax, xy(1) + d/2*cos(th), xy(2) + d/2*sin(th), col, 'EdgeColor', 'none');
end

function t_ring(ax, xy, dout, din)
% plated through-hole: tin ring and dark drill
t_disc(ax, xy, dout, [0.80 0.80 0.80]); t_disc(ax, xy, din, [0.12 0.12 0.12]);
end

function t_box(ax, s, xy, w, h, ang, col, lw)
% silkscreen outline
if nargin < 8, lw = 0.18; end
P = [-w -h; w -h; w h; -w h; -w -h]/2*[cosd(ang) sind(ang); -sind(ang) cosd(ang)] + xy;
plot(ax, P(:, 1), P(:, 2), '-', 'Color', col, 'LineWidth', lw*s.ppm*s.pt);
end

function t_text(ax, s, xy, str, h, ang, col, ha, fw)
% text with cap height h [mm]
if nargin < 8, ha = 'center'; end
if nargin < 9, fw = 'bold'; end
text(ax, xy(1), xy(2), str, 'FontUnits', 'normalized', 'FontSize', 1.45*h/s.W, 'Rotation', ang, 'Color', col, ...
    'HorizontalAlignment', ha, 'VerticalAlignment', 'middle', 'FontName', 'Arial', 'FontWeight', fw, 'Interpreter', 'none');
end

function B = local_board(c, L, W, ang, z, t, edgecol, img)
% board body (L x W x t, top at height z, centre c, rotation ang) with the texture img on its top face
B.c = c; B.L = L; B.W = W; B.ang = ang; B.z = z;
R = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
B.xy = @(u, v) (c(:) + R*[u - L/2; v - W/2]).';
[V, F] = p_box([0 0 -t], [L W t - 0.03]);
put('plastic', V, F, edgecol, c, z, ang);
k = [B.xy(0, W); B.xy(L, W); B.xy(0, 0); B.xy(L, 0)];          % TL TR BL BR
G('tex', struct('X', [k(1, 1) k(2, 1); k(3, 1) k(4, 1)], 'Y', [k(1, 2) k(2, 2); k(3, 2) k(4, 2)], ...
    'Z', z*ones(2), 'img', img, 'mat', 'pcb'));
G('shadow', k([1 2 4 3], :), z);
end

function local_mark(xy, z, ang, sz, str, col)
% marking printed on a package top (sz = [l w] mm, text lines str)
if ischar(str), str = {str}; end
h = min(0.8, 0.55*sz(2)/numel(str));
img = local_tex(sz(1), sz(2), 60, [0.09 0.09 0.10], @(ax, s) t_text(ax, s, [sz(1)/2 sz(2)/2], str, h, 0, col, 'center', 'normal'));
R = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
k = ([-1 1; 1 1; -1 -1; 1 -1].*sz/2)*R.' + xy;
G('tex', struct('X', [k(1, 1) k(2, 1); k(3, 1) k(4, 1)], 'Y', [k(1, 2) k(2, 2); k(3, 2) k(4, 2)], ...
    'Z', z*ones(2), 'img', img, 'mat', 'chip'));
end

function callout(p, q, str, ha)
% label str at q with a leader to the part at p (drawn on top of the scene)
if nargin < 4, ha = 'left'; end
G('callout', p, q, str, ha);
end

% =========================================================================================== rendering
function f = local_render(opt)
% opt: size [w h] px, az, el, target [x y z], dist, va, ground z, light directions
S = G('get');
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [20 20 opt.size]);
ax = axes(f, 'Position', [0 0 1 1], 'Clipping', 'off'); hold(ax, 'on'); axis(ax, 'off');
mats = struct( ...
    'plastic', {{'flat', 0.42, 0.72, 0.25, 12, 1.0}}, 'metal', {{'flat', 0.38, 0.55, 0.85, 22, 0.35}}, ...
    'smooth', {{'gouraud', 0.42, 0.72, 0.35, 14, 1.0}}, 'msmooth', {{'gouraud', 0.36, 0.55, 0.95, 26, 0.35}}, ...
    'glow', {{'flat', 1.0, 0.25, 0.1, 5, 1.0}});
lim = [inf -inf; inf -inf; inf -inf];
fn = fieldnames(S.M);
for i = 1:numel(fn)
    parts = S.M.(fn{i});
    if isempty(parts), continue; end
    V = cell(numel(parts), 1); F = V; Cc = V; n0 = 0;
    for j = 1:numel(parts)
        V{j} = parts{j}{1}; F{j} = parts{j}{2} + n0; Cc{j} = parts{j}{3}; n0 = n0 + size(V{j}, 1);
    end
    V = vertcat(V{:}); m = mats.(fn{i});
    patch(ax, 'Vertices', V, 'Faces', vertcat(F{:}), 'FaceVertexCData', vertcat(Cc{:}), 'FaceColor', 'flat', ...
        'EdgeColor', 'none', 'FaceLighting', m{1}, 'AmbientStrength', m{2}, 'DiffuseStrength', m{3}, ...
        'SpecularStrength', m{4}, 'SpecularExponent', m{5}, 'SpecularColorReflectance', m{6}, 'BackFaceLighting', 'reverselit');
    lim = [min(lim(:, 1), min(V)'), max(lim(:, 2), max(V)')];
end
for i = 1:numel(S.T)
    t = S.T{i};
    if strcmp(t.mat, 'chip'), a = [0.5 0.6 0.25]; else, a = [0.5 0.62 0.3]; end
    surface(ax, t.X, t.Y, t.Z, 'CData', t.img, 'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', a(1), 'DiffuseStrength', a(2), 'SpecularStrength', a(3), ...
        'SpecularExponent', 18, 'BackFaceLighting', 'reverselit');
end
ld = opt.light(1, :)/norm(opt.light(1, :));
local_ground(ax, lim, opt.ground, S.S, ld, 30);
for k = 1:size(opt.light, 1)
    light(ax, 'Style', 'infinite', 'Position', opt.light(k, :), 'Color', opt.lightc(k)*[1 1 1]);
end
axis(ax, 'equal');
pad = 60;
ax.XLim = lim(1, :) + [-pad pad]; ax.YLim = lim(2, :) + [-pad pad]; ax.ZLim = [opt.ground - 1, lim(3, 2) + pad];
camproj(ax, 'perspective');
d = [sind(opt.az)*cosd(opt.el), -cosd(opt.az)*cosd(opt.el), sind(opt.el)];
campos(ax, opt.target + opt.dist*d); camtarget(ax, opt.target); camup(ax, [0 0 1]); camva(ax, opt.va);
% callouts in an overlay axes with the same camera, so leaders and labels are never hidden
if ~isempty(S.C)
    a2 = axes(f, 'Position', ax.Position, 'Color', 'none', 'Clipping', 'off'); hold(a2, 'on'); axis(a2, 'off');
    a2.DataAspectRatio = ax.DataAspectRatio; a2.XLim = ax.XLim; a2.YLim = ax.YLim; a2.ZLim = ax.ZLim;
    a2.Projection = 'perspective'; a2.CameraPosition = ax.CameraPosition; a2.CameraTarget = ax.CameraTarget;
    a2.CameraUpVector = ax.CameraUpVector; a2.CameraViewAngle = ax.CameraViewAngle;
    c = local_colors();
    for k = 1:numel(S.C)
        [p, q, str, ha] = S.C{k}{:};
        if ~isequal(p, q)
            plot3(a2, [p(1) q(1)], [p(2) q(2)], [p(3) q(3)], '-', 'Color', c.navy, 'LineWidth', 1.0);
            plot3(a2, p(1), p(2), p(3), 'o', 'MarkerSize', 4.5, 'MarkerFaceColor', c.navy, 'MarkerEdgeColor', 'w');
        end
        text(a2, q(1), q(2), q(3), str, 'HorizontalAlignment', ha, 'VerticalAlignment', 'middle', 'FontSize', 13, ...
            'Color', c.navy, 'BackgroundColor', 'w', 'EdgeColor', [0.72 0.72 0.8], 'Margin', 2.5, 'Interpreter', 'none');
    end
end
end

function local_ground(ax, lim, zg, shadows, ld, pad)
% light-grey ground fading to white, with soft shadows of the boards along the key light ld
ppm = 2;
x = lim(1, 1) - pad:1/ppm:lim(1, 2) + pad; y = lim(2, 1) - pad:1/ppm:lim(2, 2) + pad;
[X, Y] = meshgrid(x, y);
sh = zeros(size(X)); sig = zeros(1, numel(shadows));
for k = 1:numel(shadows)
    [P, h] = shadows{k}{:};
    off = -(h - zg)*ld(1:2)/ld(3);
    m = inpolygon(X, Y, P(:, 1) + off(1), P(:, 2) + off(2));
    sh = max(sh, m*min(1, 0.55 + 0.01*(h - zg)));
    sig(k) = 1.5 + 0.12*(h - zg);
end
if ~isempty(shadows)
    s = max(sig)*ppm; g = exp(-(-ceil(3*s):ceil(3*s)).^2/(2*s^2)); g = g/sum(g);
    sh = conv2(g, g, sh, 'same');
end
cx = mean(lim(1, :)); cy = mean(lim(2, :)); rx = diff(lim(1, :))/2 + pad; ry = diff(lim(2, :))/2 + pad;
r = sqrt(((X - cx)/rx).^2 + ((Y - cy)/ry).^2);
base = 0.95 + 0.05*min(1, max(0, (r - 0.45)/0.5)).^1.2;
v = base.*(1 - 0.32*sh);
img = repmat(v, 1, 1, 3); img(:, :, 3) = min(1, img(:, :, 3)*1.01);
surface(ax, [x(1) x(end); x(1) x(end)], [y(1) y(1); y(end) y(end)], zg*ones(2), 'CData', flipud(img), ...
    'FaceColor', 'texturemap', 'EdgeColor', 'none', 'FaceLighting', 'none');
end

% =========================================================================================== parts (mm)
function local_joint(p, xy, z, ang)
% solder fillet around a lead at p (part coordinates)
c = local_colors();
[V, F] = p_cyl([p 0], [p 0.5], [0.8 0.28], 16); put('msmooth', V, F, c.tin, xy, z, ang);
end

function local_axleads(xy, z, ang, L, pitch, zc)
% bent leads of an axial part (body length L at height zc) into holes at +-pitch/2
c = local_colors();
for s = [-1 1]
    P = g_spline([s*0.46*L 0 zc; s*(pitch/2 - 0.5) 0 zc; s*pitch/2 0 zc - 0.6; s*pitch/2 0 -0.2], 0.3);
    [V, F] = p_tube(P, 0.25, 10); put('msmooth', V, F, c.tin, xy, z, ang);
    local_joint([s*pitch/2 0], xy, z, ang);
end
end

function part_res(xy, z, ang, pitch, bands, body)
% axial metal-film resistor (6.3 mm body) lying along local x; bands: string of colour codes (k n r o y g b v e w d)
c = local_colors();
if nargin < 6 || isempty(body), body = c.res_blue; end
L = 6.3; r1 = 1.0; r2 = 1.18; zc = r2 + 0.15;
[V, F] = p_cyl([-0.28*L 0 zc], [0.28*L 0 zc], r1, 24, false); put('smooth', V, F, body, xy, z, ang);
for s = [-1 1]
    [V, F] = p_cyl([s*0.28*L 0 zc], [s*0.42*L 0 zc], [r1 r2], 24, false); put('smooth', V, F, body, xy, z, ang);
    [V, F] = p_ell([s*0.42*L 0 zc], [0.08*L r2 r2], 16); put('smooth', V, F, body, xy, z, ang);
end
xb = [-0.38 -0.2 -0.08 0.04 0.38]*L;
if numel(bands) == 4, xb = xb([1 2 3 5]); end
for k = 1:numel(bands)
    rb = r1 + 0.03 + (r2 - r1)*min(1, max(0, (abs(xb(k)) - 0.28*L)/(0.14*L)));
    [V, F] = p_cyl([xb(k) - 0.22 0 zc], [xb(k) + 0.22 0 zc], rb, 24, false);
    put('smooth', V, F, c.band.(bands(k)), xy, z, ang);
end
local_axleads(xy, z, ang, L, pitch, zc);
end

function part_diode(xy, z, ang, pitch)
% axial rectifier (DO-41), cathode band at local +x
c = local_colors();
L = 5.2; r = 1.3; zc = r + 0.15;
[V, F] = p_cyl([-L/2 0 zc], [L/2 0 zc], r, 24); put('smooth', V, F, c.ic_black, xy, z, ang);
[V, F] = p_cyl([L/2 - 1.2 0 zc], [L/2 - 0.5 0 zc], r + 0.03, 24, false); put('smooth', V, F, [0.78 0.78 0.8], xy, z, ang);
local_axleads(xy, z, ang, L, pitch, zc);
end

function part_disc(xy, z, ang, d, col, h0)
% ceramic disc capacitor standing on its leads (disc in the local x-z plane, lead pitch 2.54)
c = local_colors();
if nargin < 5 || isempty(col), col = c.cer_orange; end
if nargin < 6, h0 = 2.0; end
[V, F] = p_ell([0 0 h0 + d/2], [d/2 0.95 d/2], 20); put('smooth', V, F, col, xy, z, ang);
for s = [-1 1]
    [V, F] = p_tube([s*1.27 0 h0 + 0.8; s*1.27 0 -0.2], 0.25, 10); put('msmooth', V, F, c.tin, xy, z, ang);
    local_joint([s*1.27 0], xy, z, ang);
end
end

function part_elec(xy, z, ang, d, h, body, stripe)
% radial electrolytic capacitor, polarity stripe toward local -y
c = local_colors();
if nargin < 6 || isempty(body), body = c.elec_black; end
if nargin < 7 || isempty(stripe), stripe = c.elec_stripe; end
n = 40; r = d/2;
[V, F] = p_cyl([0 0 0.5], [0 0 h - 0.35], r, n, false);
mid = (V(F(:, 1), :) + V(F(:, 2), :))/2; phi = atan2d(mid(:, 2), mid(:, 1));
col = repmat(body, n, 1); col(abs(phi + 90) < 26, :) = repmat(stripe, nnz(abs(phi + 90) < 26), 1);
put('smooth', V, F, col, xy, z, ang);
[V, F] = p_cyl([0 0 h - 0.35], [0 0 h], [r r - 0.3], n, false); put('smooth', V, F, body, xy, z, ang);
[V, F] = p_cyl([0 0 h - 0.05], [0 0 h + 0.02], r - 0.3, n); put('msmooth', V, F, c.alu, xy, z, ang);
for a = [0 90]
    [V, F] = p_box([0 0 h + 0.02], [2*(r - 0.7) 0.2 0.02]); put('plastic', V, F, [0.35 0.35 0.37], xy, z, ang + a);
end
[V, F] = p_cyl([0 0 0], [0 0 0.5], r - 0.35, n); put('smooth', V, F, [0.25 0.25 0.27], xy, z, ang);
end

function part_vchip(xy, z, ang, d, h)
% SMD aluminium electrolytic (V-chip): black base, bare can, dark polarity mark on the top
c = local_colors();
[V, F] = p_box([0 0 0], [d + 0.3 d + 0.3 1.0]); put('plastic', V, F, c.ic_black, xy, z, ang);
[V, F] = p_cyl([0 0 1.0], [0 0 h - 0.3], d/2, 40, false); put('msmooth', V, F, c.alu, xy, z, ang);
[V, F] = p_cyl([0 0 h - 0.3], [0 0 h], [d/2 d/2 - 0.3], 40); put('msmooth', V, F, c.alu, xy, z, ang);
th = linspace(-60, 60, 16)'; P = [0.45*d/2*cosd(th) - 0.05*d, 0.45*d/2*sind(th)];
[V, F] = p_extrude([P; P(end, 1) - 0.3*d, P(end, 2); P(1, 1) - 0.3*d, P(1, 2)], h, 0.02);
put('plastic', V, F, [0.12 0.12 0.14], xy, z, ang + 180);
for s = [-1 1], [V, F] = p_box([0 s*(d/2 + 0.4) 0], [1.0 0.9 0.2]); put('metal', V, F, c.tin, xy, z, ang); end
end

function part_smd(xy, z, ang, sz, body)
% chip resistor or capacitor (sz = [l w h]) with metal end caps
c = local_colors();
e = 0.22*sz(1);
[V, F] = p_box([0 0 0], [sz(1) - 2*e + 0.02, sz(2), sz(3)]); put('plastic', V, F, body, xy, z, ang);
for s = [-1 1]
    [V, F] = p_box([s*(sz(1)/2 - e/2) 0 0], [e sz(2) + 0.02 sz(3) + 0.02]); put('metal', V, F, c.tin, xy, z, ang);
end
end

function part_led(xy, z, ang, glow)
% 0603 LED, lit
c = local_colors();
[V, F] = p_box([0 0 0], [1.6 0.8 0.3]); put('plastic', V, F, [0.92 0.92 0.88], xy, z, ang);
[V, F] = p_box([0 0 0.3], [1.1 0.75 0.3]); put('glow', V, F, glow, xy, z, ang);
for s = [-1 1], [V, F] = p_box([s*0.65 0 0], [0.3 0.82 0.32]); put('metal', V, F, c.tin, xy, z, ang); end
end

function local_gullwing(xk, yb, zs, lw, xy, z, ang, sgn)
% one gull-wing lead leaving the body side yb (sign sgn) at shoulder height zs
c = local_colors();
[V, F] = p_box([xk sgn*(yb + 0.2) zs], [lw 0.4 0.18]); put('metal', V, F, c.tin, xy, z, ang);
[V, F] = p_box([xk sgn*(yb + 0.4) 0], [lw 0.18 zs + 0.18]); put('metal', V, F, c.tin, xy, z, ang);
[V, F] = p_box([xk sgn*(yb + 0.7) 0], [lw 0.62 0.18]); put('metal', V, F, c.tin, xy, z, ang);
end

function part_soic(xy, z, ang, n, pitch, bl, bw, h, mark)
% gull-wing package (SOIC, SSOP, SOP-4): n leads in two rows along local x, body bl x bw x h
c = local_colors();
[V, F] = p_box([0 0 0.1], [bl bw h]); put('plastic', V, F, c.ic_black, xy, z, ang);
np = n/2; xk = ((1:np) - (np + 1)/2)*pitch; lw = min(0.45, 0.55*pitch);
for s = [-1 1]
    for k = 1:np, local_gullwing(xk(k), bw/2, 0.1 + 0.4*h, lw, xy, z, ang, s); end
end
[V, F] = p_cyl([-bl/2 + 0.7 -bw/2 + 0.7 0.1 + h], [-bl/2 + 0.7 -bw/2 + 0.7 0.12 + h], 0.3, 16);
put('smooth', V, F, [0.25 0.25 0.27], xy, z, ang);
if nargin > 8 && ~isempty(mark)
    local_mark(xy, z + 0.1 + h + 0.005, ang, [bl - 0.6 bw - 0.6], mark, [0.72 0.72 0.74]);
end
end

function part_qfp(xy, z, ang, np, pitch, b, h, mark)
% quad flat package: np leads per side, body b x b x h
c = local_colors();
[V, F] = p_box([0 0 0.1], [b b h]); put('plastic', V, F, c.ic_black, xy, z, ang);
xk = ((1:np) - (np + 1)/2)*pitch; lw = 0.55*pitch;
for side = 0:3
    for k = 1:np, local_gullwing(xk(k), b/2, 0.1 + 0.4*h, lw, xy, z, ang + 90*side, 1); end
end
[V, F] = p_cyl([-b/2 + 0.9 -b/2 + 0.9 0.1 + h], [-b/2 + 0.9 -b/2 + 0.9 0.12 + h], 0.35, 16);
put('smooth', V, F, [0.25 0.25 0.27], xy, z, ang);
if nargin > 7 && ~isempty(mark)
    local_mark(xy, z + 0.1 + h + 0.005, ang, [b - 1.2 b - 1.2], mark, [0.72 0.72 0.74]);
end
end

function part_header(xy, z, ang, n, rows, pinTop, pinCol, hbody)
% male pin header along local x: n x rows pins, plastic hbody high, pins pinTop above the plastic
c = local_colors();
if nargin < 8, hbody = 2.5; end
p = 2.54; xk = ((1:n) - (n + 1)/2)*p; yk = ((1:rows) - (rows + 1)/2)*p;
[V, F] = p_box([0 0 0], [n*p - 0.1, rows*p - 0.1, hbody]); put('plastic', V, F, c.hdr_black, xy, z, ang);
for i = 1:n
    for j = 1:rows
        [V, F] = p_box([xk(i) yk(j) hbody], [0.64 0.64 pinTop]); put('metal', V, F, pinCol, xy, z, ang);
    end
end
end

function part_fheader(xy, z, ang, n, rows, h)
% female header along local x, h high, dark contact openings on the top
c = local_colors();
p = 2.54; xk = ((1:n) - (n + 1)/2)*p; yk = ((1:rows) - (rows + 1)/2)*p;
[V, F] = p_box([0 0 0], [n*p - 0.1, rows*p - 0.1, h]); put('plastic', V, F, c.hdr_black, xy, z, ang);
for i = 1:n
    for j = 1:rows
        [V, F] = p_box([xk(i) yk(j) h], [1.0 1.0 0.02]); put('plastic', V, F, [0.01 0.01 0.01], xy, z, ang);
    end
end
end

function part_pintips(xy, z, ang, n, rows, len, col)
% pins coming through a board top with solder fillets (n x rows, pitch 2.54, along local x)
c = local_colors();
p = 2.54; xk = ((1:n) - (n + 1)/2)*p; yk = ((1:rows) - (rows + 1)/2)*p;
for i = 1:n
    for j = 1:rows
        [V, F] = p_cyl([xk(i) yk(j) 0], [xk(i) yk(j) 0.6], [0.95 0.42], 16); put('msmooth', V, F, c.tin, xy, z, ang);
        if len > 0
            [V, F] = p_box([xk(i) yk(j) 0], [0.64 0.64 len]); put('metal', V, F, col, xy, z, ang);
        end
    end
end
end

function part_button(xy, z, ang, s, h, capr, capcol, basecol)
% SMD tact switch: base s(1) x s(2) x h, round cap
c = local_colors();
[V, F] = p_box([0 0 0], [s(1) s(2) h]); put('plastic', V, F, basecol, xy, z, ang);
[V, F] = p_cyl([0 0 h], [0 0 h + 0.7], capr, 28); put('smooth', V, F, capcol, xy, z, ang);
for sx = [-1 1]
    for sy = [-1 1]
        [V, F] = p_box([sx*(s(1)/2 + 0.3) sy*s(2)/3 0], [0.6 0.5 0.2]); put('metal', V, F, c.tin, xy, z, ang);
    end
end
end

function part_socket(xy, z, ang, w, h, len, r, col)
% receptacle with a rounded section (w x h, radius r) and its opening toward local +y (USB-C, micro, mini USB)
P = g_rrect(w, h, r, 8);
[V, F] = p_extrude(P, -len/2, len); V = [V(:, 1), V(:, 3), V(:, 2) + h/2];
put('msmooth', V, F, col, xy, z, ang);
P = g_rrect(w - 1.1, h - 1.0, max(0.1, r - 0.4), 8);
[V, F] = p_extrude(P, len/2, 0.03); V = [V(:, 1), V(:, 3), V(:, 2) + h/2];
put('plastic', V, F, [0.04 0.04 0.05], xy, z, ang);
[V, F] = p_box([0 len/2 + 0.04 h/2 - 0.35], [w - 3.2, 0.02, 0.7]); put('plastic', V, F, [0.2 0.2 0.22], xy, z, ang);
end

function part_xtal(xy, z, ang, s)
% SMD crystal: ceramic base, metal lid (s = [l w h])
c = local_colors();
[V, F] = p_box([0 0 0], [s(1) s(2) 0.3]); put('plastic', V, F, [0.85 0.80 0.68], xy, z, ang);
[V, F] = p_extrude(g_rrect(s(1) - 0.4, s(2) - 0.4, 0.35, 5), 0.3, s(3) - 0.3); put('msmooth', V, F, c.alu, xy, z, ang);
end

function part_to263(xy, z, ang, mark)
% TO-263-5 (D2PAK): body, heat tab toward local -y, five leads toward +y
c = local_colors();
[V, F] = p_box([0 0.4 0], [10.1 8.6 4.4]); put('plastic', V, F, c.ic_black, xy, z, ang);
[V, F] = p_box([0 -4.4 0], [10.6 1.8 1.3]); put('metal', V, F, c.tin, xy, z, ang);
for k = -2:2, local_gullwing(1.7*k, 4.7, 1.2, 0.8, xy, z, ang, 1); [V, F] = p_box([1.7*k 6.0 0], [0.8 1.2 0.18]); put('metal', V, F, c.tin, xy, z, ang); end
if nargin > 3, local_mark(xy + [-sind(ang) cosd(ang)]*0.6, z + 4.405, ang, [8.6 6.0], mark, [0.72 0.72 0.74]); end
end

function part_dpak(xy, z, ang)
% TO-252 (DPAK): body, heat tab toward local -y, two leads toward +y
c = local_colors();
[V, F] = p_box([0 0.4 0], [6.6 6.1 2.3]); put('plastic', V, F, c.ic_black, xy, z, ang);
[V, F] = p_box([0 -3.3 0], [5.4 1.6 0.6]); put('metal', V, F, c.tin, xy, z, ang);
for k = [-1 1], local_gullwing(2.28*k, 3.45, 0.7, 0.8, xy, z, ang, 1); [V, F] = p_box([2.28*k 5.0 0], [0.8 1.4 0.18]); put('metal', V, F, c.tin, xy, z, ang); end
end

function part_sma(xy, z, ang)
% SMA diode, cathode band at local -x
c = local_colors();
[V, F] = p_box([0 0 0.1], [4.3 2.6 2.0]); put('plastic', V, F, c.ic_black, xy, z, ang);
[V, F] = p_box([-1.5 0 2.1], [0.5 2.6 0.01]); put('plastic', V, F, [0.62 0.62 0.64], xy, z, ang);
for s = [-1 1], [V, F] = p_box([s*2.35 0 0], [0.8 1.5 0.25]); put('metal', V, F, c.tin, xy, z, ang); end
end

function part_drum(xy, z, ang, d, h, mark)
% unshielded drum-core power inductor: ferrite flanges and copper winding, marking on the top
c = local_colors();
r = d/2;
[V, F] = p_cyl([0 0 0], [0 0 1.0], r, 48); put('smooth', V, F, c.ferrite, xy, z, ang);
nz = 12; zz = linspace(1.0, h - 1.0, nz + 1);
for k = 1:nz
    [V, F] = p_cyl([0 0 zz(k)], [0 0 zz(k + 1)], r - 0.75 + 0.08*mod(k, 2), 48, false);
    put('msmooth', V, F, c.copper*(0.92 + 0.08*mod(k, 2)), xy, z, ang);
end
[V, F] = p_cyl([0 0 h - 1.0], [0 0 h], r, 48); put('smooth', V, F, c.ferrite, xy, z, ang);
if nargin > 5, local_mark(xy, z + h + 0.005, ang, [0.9*r 0.6*r], mark, [0.85 0.85 0.85]); end
end

function part_trim(xy, z, ang)
% 3296W multi-turn trimmer: blue body, brass screw at local -x
c = local_colors();
[V, F] = p_box([0 0 0], [9.5 4.8 10]); put('plastic', V, F, c.trim_blue, xy, z, ang);
[V, F] = p_cyl([-3.0 0 10], [-3.0 0 10.6], 1.15, 28); put('msmooth', V, F, c.brass, xy, z, ang);
[V, F] = p_box([-3.0 0 10.6], [1.9 0.32 0.04]); put('plastic', V, F, [0.25 0.2 0.08], xy, z, ang);
end

function part_housing(xy, z, ang, s, col, opening)
% plastic housing (connector, jack): box s, optional dark opening on the local +y face (opening = [w h])
[V, F] = p_box([0 0 0], s); put('plastic', V, F, col, xy, z, ang);
if nargin > 5
    [V, F] = p_box([0 s(2)/2 (s(3) - opening(2))/2], [opening(1) 0.04 opening(2)]);
    put('plastic', V, F, [0.02 0.02 0.02], xy, z, ang);
end
end

function local_wire(P, r, col, ds)
% insulated wire along the smooth path through the world points P
if nargin < 4, ds = 0.8; end
[V, F] = p_tube(g_spline(P, ds), r, 14); put('smooth', V, F, col, [0 0], 0, 0);
end

function local_twisted(P, cols, r, pitch)
% twisted wire bundle along the path P: strands of radius r, one twist every pitch mm
Q = g_spline(P, 0.4); m = size(Q, 1); ns = size(cols, 1);
s = [0; cumsum(vecnorm(diff(Q), 2, 2))];
T = gradient(Q.').'; T = T./vecnorm(T, 2, 2);
a = [0 0 1]; if abs(dot(a, T(1, :))) > 0.9, a = [1 0 0]; end
u = cross(T(1, :), a); u = u/norm(u);
U = zeros(m, 3); W = U;
for k = 1:m
    u = u - dot(u, T(k, :))*T(k, :); u = u/norm(u); U(k, :) = u; W(k, :) = cross(T(k, :), u);
end
d = r*(ns > 1)*1.05/max(sin(pi/ns), 0.5);
for j = 1:ns
    ph = 2*pi*s/pitch + 2*pi*(j - 1)/ns;
    S = Q + d*(cos(ph).*U + sin(ph).*W);
    [V, F] = p_tube(S, r, 10); put('smooth', V, F, cols(j, :), [0 0], 0, 0);
end
end
