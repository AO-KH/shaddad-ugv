function phase1_joystick_check(controller, id)
%PHASE1_JOYSTICK_CHECK  Show the USB controller axes, buttons and the mapped drive commands live.
%
%   phase1_joystick_check()                  auto-detect (PS5 DualSense or RC transmitter), joystick ID 1
%   phase1_joystick_check('ps5')             PS5 layout (P.scen.joy.pad)
%   phase1_joystick_check('transmitter')     RC transmitter in USB joystick mode (P.scen.joy.ax / inv / btn)
%   Close the window to stop. Move each stick and press each button: throttle forward must give a positive
%   stick_thr, steering right a positive stick_steer. To change the mapping edit scen.joy in
%   params/param_sim.m and run shaddad_params.

if nargin < 1 || isempty(controller), controller = 'auto'; end
if nargin < 2, id = 1; end
if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_params(); end
m = evalin('base', 'P.scen.joy');
try
    j = sim3d.io.Joystick(ID = id);
catch e
    error('phase1_joystick_check:noDevice', ['No controller %d: %s\nConnect the PS5 controller (USB or ' ...
        'Bluetooth) or the RC transmitter in "USB Joystick (HID)" mode.'], id, e.message);
end
c = caps(j);
ctl = lower(char(controller));
if strcmp(ctl, 'auto')
    if c.Axes == 6 && c.Buttons >= 13 && c.Buttons <= 18, ctl = 'ps5'; else, ctl = 'transmitter'; end
end
if strcmp(ctl, 'ps5'), m.profile = 2; else, m.profile = 1; end
clear joy_map                                   % reset the latched gamepad states
psNames = {'Square', 'Cross', 'Circle', 'Triangle', 'L1', 'R1', 'L2', 'R2', 'Create', 'Options', 'L3', 'R3', ...
    'PS', 'Touchpad', 'Mute'};
axNames = {'LX', 'LY', 'RX', 'L2', 'R2', 'RY'};
f = figure('Name', sprintf('Controller check: %s layout (%d axes, %d buttons)', upper(ctl), c.Axes, c.Buttons), ...
    'NumberTitle', 'off', 'Color', 'w', 'Position', [100 100 980 540]);
ax1 = subplot(1, 2, 1, 'Parent', f);
na = max(1, c.Axes);
hb = barh(ax1, zeros(1, na), 'FaceColor', [0.17 0.19 0.5]);
xlim(ax1, [-1.05 1.05]); grid(ax1, 'on');
if strcmp(ctl, 'ps5') && na == 6, lab = axNames; else, lab = compose('axis %d', 1:na); end
yticks(ax1, 1:na); yticklabels(ax1, lab); title(ax1, 'Axes');
ax2 = subplot(1, 2, 2, 'Parent', f); axis(ax2, 'off');
ht = text(ax2, 0, 1, '', 'FontName', 'Consolas', 'FontSize', 11, 'VerticalAlignment', 'top', 'Interpreter', 'none');
modes = {'MANUAL', 'HOLD', 'AUTO'};  lifts = {'UP', 'HOLD', 'DOWN'};  impls = {'OFF', 'ON', 'ON+auto'};
while isvalid(f)
    [a, b] = read(j);
    a = double(a(:)');  b = double(b(:)');
    hb.YData = a(1:numel(hb.YData));
    op = joy_map(a, b, m);
    pressed = find(b > 0.5);
    if strcmp(ctl, 'ps5'), pn = strjoin(psNames(pressed(pressed <= numel(psNames))), ' '); else, pn = mat2str(pressed); end
    ht.String = {sprintf('buttons pressed: %s', pn), '', ...
        sprintf('stick_thr    %+5.2f   (+ = forward)', op.stick_thr), ...
        sprintf('stick_steer  %+5.2f   (+ = right)', op.stick_steer), ...
        sprintf('speed scale  %3.0f %%', 100*(0.25 + (op.knob_speed + 1)/2*0.75)), ...
        sprintf('mode         %s', modes{op.sw_mode}), ...
        sprintf('armed        %d', op.sw_arm), ...
        sprintf('E-stop       %d   (latched)', op.sw_estop), ...
        sprintf('reset        %d', op.btn_reset), ...
        sprintf('lights       %d', op.sw_lights), ...
        sprintf('implement    %s', impls{op.sw_impl}), ...
        sprintf('lift         %s', lifts{op.sw_lift}), '', ...
        sprintf('Layout: %s (P.scen.joy, params/param_sim.m)', upper(ctl))};
    drawnow limitrate
    pause(0.05);
end
delete(j);
end
