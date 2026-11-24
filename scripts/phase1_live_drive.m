function res = phase1_live_drive(varargin)
%PHASE1_LIVE_DRIVE  Drive the Shaddad UGV live with a PS5 controller or an RC transmitter and watch it in 3-D.
%
%   res = phase1_live_drive()                               USB controller (auto-detected), 3-D view, real time
%   res = phase1_live_drive('Controller', 'ps5')            force the PS5 (DualSense) layout
%   res = phase1_live_drive('Controller', 'transmitter')    force the RC transmitter layout (USB joystick mode)
%   res = phase1_live_drive('Source', 'demo')               no hardware: the demo plan drives, 3-D view
%   res = phase1_live_drive(..., 'Implement', 1)            mower mounted (2 tiller, 3 sprayer)
%   res = phase1_live_drive(..., 'Camera', 2)               1 chase, 2 side, 3 top, 4 free (also buttons / keys 1-4)
%   res = phase1_live_drive(..., 'Fidelity', 'full')        high-fidelity plant (slower than real time)
%   res = phase1_live_drive(..., 'Duration', 600, 'Pacing', true, 'PacingRate', 1, 'View', '3d')
%
%   PS5 controller (USB cable or Bluetooth): left stick up/down = throttle, right stick left/right = steering,
%   Cross = arm / disarm (sticks centred), Circle = E-stop (latched), Triangle = reset (then Cross to arm),
%   R1 / L1 = speed 25-50-75-100 %, Options = MANUAL / HOLD, Square = lights, Create = implement on/off,
%   L2 / R2 = implement lift up / down. Mapping: P.scen.joy.pad (params/param_sim.m).
%   RC transmitter: select "USB Joystick (HID)" on the radio; CH6 arm, CH5 low = MANUAL, CH2 throttle, CH1
%   steering, CH7 speed, CH12 E-stop, CH9 reset. Mapping: P.scen.joy.ax / inv / btn.
%   Check either with phase1_joystick_check. Stop the run with the red Stop button or Esc in the 3-D window.
%
%   The whole chain runs: controller -> simulated RC transmitter and SBUS frames -> STM32 decoder and control
%   -> CAN -> motor controllers -> motors and gearboxes -> tracks on the soil, powered by the battery and BMS.
%   The first run compiles the model (about 2-3 minutes on the reference PC) before the 3-D window opens.
%   'Fidelity' 'fast' uses the endurance variants (lumped pack, ideal drive, kinematic chassis). Measured on
%   the reference PC (Normal mode, 2026-10-26): about 1.07x real time without the 3-D view, about 0.85x with
%   it (10 Hz), about 0.6x with an implement. Pacing caps the run at real time when the PC is fast enough.

ip = inputParser;  ip.KeepUnmatched = true;
ip.addParameter('Source', 'joystick');
ip.addParameter('Controller', 'auto');
ip.parse(varargin{:});
args = varargin;
switch lower(char(ip.Results.Source))
    case 'joystick'                    % fail fast (before the model compile) when no controller is connected
        try
            j = sim3d.io.Joystick(ID = 1);  c0 = caps(j);  delete(j);
        catch
            error('phase1_live_drive:noController', ['No USB controller found. Connect the PS5 controller (or the ' ...
                'RC transmitter in "USB Joystick (HID)" mode) and check it with phase1_joystick_check. Without ' ...
                'hardware use phase1_live_drive(''Source'', ''demo'').']);
        end
        ctl = lower(char(ip.Results.Controller));
        if strcmp(ctl, 'auto')
            if c0.Axes == 6 && c0.Buttons >= 13 && c0.Buttons <= 18, ctl = 'ps5'; else, ctl = 'transmitter'; end
        end
        fprintf('Controller found: %d axes, %d buttons -> %s layout.\n', c0.Axes, c0.Buttons, upper(ctl));
        if strcmp(ctl, 'ps5')
            fprintf(['  Left stick = throttle, right stick = steering. Cross = arm, Circle = E-stop, Triangle = reset,\n' ...
                '  R1/L1 = speed, Options = HOLD, Square = lights, Create = implement, L2/R2 = lift.\n']);
        else
            fprintf('  CH6 = arm, CH2 = throttle, CH1 = steering, CH7 = speed, CH12 = E-stop, CH9 = reset.\n');
        end
        args = [args {'Controller', ctl}];
    case 'dashboard'
        error('phase1_live_drive:dashboard', ['The dashboard sliders do not work yet: they are bound to Constant ' ...
            'blocks whose values are expressions (P.scen.dash.*), which dashboard bindings cannot drive. Use a ' ...
            'USB controller (''joystick'') or ''demo''.']);
end
fprintf(['Compiling and starting the model - the 3-D window opens when the run starts (about 2-3 minutes the ' ...
    'first time, faster afterwards). Then arm and drive; stop with the red Stop button or Esc.\n']);
ws = warning('off', 'all');
c = onCleanup(@() warning(ws));
res = run_scenario('L1', 'Args', args, 'Mode', 'normal');
if ~res.ok
    fprintf(2, 'Live drive ended with an error: %s\n', res.msg);
end
if isempty(res.S) || ~isfield(res.S, 'telem') || numel(res.S.telem.mode.t) < 2
    fprintf('\n=== Live drive stopped before any data was logged (%.0f s wall). ===\n', res.wall_s);
    return
end
S = res.S;
tEnd = S.telem.mode.t(end);
modes = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', 'CTRL_STOP', 'ESTOP', 'FAULT'};
m = S.telem.mode;  mv = double(m.v(:));
i = [1; find(diff(mv) ~= 0) + 1];
fprintf('\n=== Live drive: %.0f s simulated, %.2fx real time ===\n', tEnd, tEnd/max(res.exec_s, eps));
fprintf('MCU modes : %s\n', strjoin(arrayfun(@(k) sprintf('%s@%.1fs', modes{mv(k) + 1}, m.t(k)), i(1:min(end, 20))', ...
    'UniformOutput', false), ' -> '));
pos = S.truth.pos_W.v;
fprintf('Distance  : %.1f m, max speed %.2f m/s\n', sum(vecnorm(diff(pos(:, 1:2)), 2, 2)), max(abs(double(S.truth.speed_fwd.v))));
fprintf('Log file  : %s\n', res.file);
fprintf('Look at it: plot_run(res) for all signals, shaddad.Robot3DView.replay(res.file) for the 3-D replay\n\n');
end
