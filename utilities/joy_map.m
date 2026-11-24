function op = joy_map(ax, bt, m)
%JOY_MAP  USB controller -> operator command Bus_OperatorCmd (live RC drive, VC_OPSRC 3).
%
%   op = joy_map(ax, bt, m)
%     ax  joystick axes (-1..1), bt buttons (0/1), from the Simulation 3D Joystick block
%     m   mapping P.scen.joy, field profile:
%         1 RC transmitter in USB joystick mode: ax (axis index of [steer thr aux_lv aux_lh mode arm speed impl],
%           0 = unused), inv (sign per entry), btn (button index of [reset lights estop]), sw3 (3-position
%           threshold). Channels and switches are read as levels, exactly as the radio sets them.
%         2 gamepad (PS5 DualSense), m.pad: its buttons spring back, so arm, E-stop, lights, implement, HOLD and
%           the speed scale are latched here (persistent state, edge detection); see params/param_sim.m.
%   The values then pass the simulated transmitter (RC_Link_Sim): SBUS frames, radio and receiver, so the MCU
%   decodes them exactly as it decodes a real receiver. Unused functions take the neutral / safe value:
%   sticks centred, speed 100 %, mode MANUAL, disarmed, implement off, lift hold, E-stop RUN.
%#codegen

ax = double(ax(:));  bt = double(bt(:));
op = struct('stick_thr', 0, 'stick_steer', 0, 'stick_aux_lv', 0, 'stick_aux_lh', 0, 'knob_speed', 1, ...
    'sw_mode', 1, 'sw_arm', 0, 'sw_impl', 1, 'sw_lift', 2, 'sw_lights', 0, 'sw_estop', 0, 'btn_reset', 0, ...
    'sw_mission', 1, 'btn_rth', 0, 'tx_power_on', 1);
if m.profile == 2
    op = local_pad(ax, bt, m.pad, op);
    return
end
op.stick_steer  = local_ax(ax, m, 1, 0);
op.stick_thr    = local_ax(ax, m, 2, 0);
op.stick_aux_lv = local_ax(ax, m, 3, 0);
op.stick_aux_lh = local_ax(ax, m, 4, 0);
op.sw_mode      = local_sw3(local_ax(ax, m, 5, -1), m.sw3);
op.sw_arm       = double(local_ax(ax, m, 6, -1) > 0);
op.knob_speed   = local_ax(ax, m, 7, 1);
op.sw_impl      = local_sw3(local_ax(ax, m, 8, -1), m.sw3);
op.btn_reset    = local_bt(bt, m.btn(1));
op.sw_lights    = local_bt(bt, m.btn(2));
op.sw_estop     = local_bt(bt, m.btn(3));
end

% ------------------------------------------------------------------------------------------------
function op = local_pad(ax, bt, g, op)
% gamepad: momentary buttons -> latched switch states (re-initialised at every simulation start)
persistent prev armed estop lights impl hold spd
if isempty(prev)
    prev = zeros(32, 1);  armed = 0;  estop = 0;  lights = 0;  impl = 1;  hold = 0;  spd = 4;
end
b = zeros(32, 1);
n = min(numel(bt), 32);
b(1:n) = double(bt(1:n) > 0.5);
rise = b > 0.5 & prev < 0.5;
if local_hit(rise, g.b_arm), armed = 1 - armed; end
if local_hit(rise, g.b_estop), estop = 1; armed = 0; end           % E-stop latches and disarms
reset = local_hit(b > 0.5, g.b_reset);
if reset, estop = 0; end                                            % reset clears the latch; then press arm again
if local_hit(rise, g.b_lights), lights = 1 - lights; end
if local_hit(rise, g.b_impl), impl = 3 - impl; end                 % 1 OFF <-> 2 ON
if local_hit(rise, g.b_hold), hold = 1 - hold; end
if local_hit(rise, g.b_spd_up), spd = min(4, spd + 1); end
if local_hit(rise, g.b_spd_dn), spd = max(1, spd - 1); end
prev = b;

op.stick_thr   = local_dz(local_axis(ax, g.ax_thr)*g.inv_thr, g.dz);
op.stick_steer = local_dz(local_axis(ax, g.ax_steer)*g.inv_steer, g.dz);
op.knob_speed  = -1 + 2*(spd - 1)/3;                                 % 25 / 50 / 75 / 100 % speed scale
op.sw_mode     = 1 + hold;                                           % MANUAL / HOLD
op.sw_arm      = armed;
op.sw_estop    = estop;
op.btn_reset   = double(reset);
op.sw_lights   = lights;
op.sw_impl     = impl;
up = local_axis(ax, g.ax_lift_up) > 0;                               % triggers rest at -1
dn = local_axis(ax, g.ax_lift_dn) > 0;
if up && ~dn
    op.sw_lift = 1;
elseif dn && ~up
    op.sw_lift = 3;
else
    op.sw_lift = 2;
end
end

function tf = local_hit(v, i)
tf = i >= 1 && i <= numel(v) && v(i);
end

function v = local_axis(ax, i)
if i >= 1 && i <= numel(ax), v = max(-1, min(1, ax(i))); else, v = 0; end
end

function y = local_dz(x, dz)
if abs(x) < dz, y = 0; else, y = sign(x)*(abs(x) - dz)/(1 - dz); end
end

function v = local_ax(ax, m, k, dflt)
i = m.ax(k);
if i >= 1 && i <= numel(ax)
    v = max(-1, min(1, m.inv(k)*ax(i)));
else
    v = dflt;
end
end

function p = local_sw3(x, th)
if x < -th
    p = 1;
elseif x > th
    p = 3;
else
    p = 2;
end
end

function v = local_bt(bt, i)
if i >= 1 && i <= numel(bt)
    v = double(bt(i) > 0.5);
else
    v = 0;
end
end
