function u = param_motor_unit(option)
%PARAM_MOTOR_UNIT  One traction motor with its gearbox, brake and controller channel (owner: drive).
%
%   u = param_motor_unit()                 the motor the model uses: 'kw1'
%   u = param_motor_unit('kw1')            proposed upgrade, 1.0 kW 48 V BLDC, 3000 rpm [EST] (2026-10-27)
%   u = param_motor_unit('movematic750')   as built: Movematic 080-240 BLDC, 48 V, 750 W, 3000 rpm [DATA]
%
%   The robot has four traction motors, two per track: one on the front and one on the rear sprocket, and
%   one dual-channel ESC per side (user, 2026-10-27). param_drive builds the equivalent drive of one side
%   from two units (both motors of a side get the same command); param_structure takes the unit masses.
%   Sizing of the upgrade (docs/hardware/real_robot_components.md, section 3): per-motor RMS torque of a
%   hilly-farm spraying cycle 1.45 N m on farm soil and 1.81 N m on wet loam, against 2.15 N m continuous
%   of the 750 W motor at 45 degC ambient; the 1.0 kW unit gives 2.86 N m (margin 1.58 on wet loam).
%   Common to both: 4 pole pairs, the same back-EMF (no-load 3600 rpm at 48 V), gearbox 20:1 [EST].

if nargin < 1 || isempty(option), option = 'kw1'; end
u.option = option;
u.n_rated = 3000; u.n_max = 3600; u.n_mech_max = 4000; u.p = 4; u.lambda_pm = 0.0184;
u.w_t_rpm = [0 500 1000 1500 2000 2500 3000 3300 3600 3800 4000 4200];
switch option
    case 'movematic750'
        u.name = 'Movematic 080-240 BLDC, 48 V, 750 W, 3000 rpm';
        u.P_rated = 750; u.T_cont = 2.39; u.T_peak = 7.16; u.P_peak_int = 1500;   % rated [DATA], peak 3 x [EST]
        u.Rs = 64e-3; u.L = 250e-6; u.L0 = 120e-6; u.J = 2.0e-4; u.k_h = 0.0096; u.k_e = 5.0e-6;
        u.T_pk_nom  = [7.16 7.16 7.16 7.16 7.16 5.73 4.77 2.6 0 0 0 0];    % 1.5 kW peak power above 2000 rpm
        u.T_cont_nom = [2.39 2.39 2.39 2.39 2.39 2.39 2.39 1.5 0 0 0 0];
        u.mass = 3.5;                                % 80 mm flange
        u.esc = [20 50 40 15];                       % A: rms cont, rms peak, DC motoring, DC regen per channel
        u.gear_T = [120 140 240];                    % N m gearbox output: T2N, T2B (>= 7.16*20*0.9 = 129), T2NOT
        u.gear_mass = 3.0;
    case 'kw1'
        u.name = '1.0 kW 48 V BLDC, 3000 rpm (proposed upgrade)';
        u.P_rated = 1000; u.T_cont = 3.18; u.T_peak = 9.55; u.P_peak_int = 2000;  % 3 x rated peak
        u.Rs = 48e-3; u.L = 190e-6; u.L0 = 90e-6; u.J = 2.8e-4; u.k_h = 0.0125; u.k_e = 6.5e-6;
        u.T_pk_nom  = [9.55 9.55 9.55 9.55 9.55 7.64 6.37 3.5 0 0 0 0];    % 2.0 kW peak power above 2000 rpm
        u.T_cont_nom = [3.18 3.18 3.18 3.18 3.18 3.18 3.18 2.0 0 0 0 0];
        u.mass = 4.5;                                % 86-90 mm frame
        u.esc = [27 65 55 20];
        u.gear_T = [120 180 240];                    % T2B >= 9.55*20*0.9 = 172 N m
        u.gear_mass = 3.5;
    otherwise
        error('param_motor_unit:option', 'unknown motor option %s (kw1 | movematic750)', option);
end
u.Kt = 1.5*u.p*u.lambda_pm;                      % N m per A (peak phase current)
u.brake_T = 5; u.brake_mass = 0.5;               % spring brake per motor: four hold 320 kg on 20 deg with margin 4
u.esc_mass = 0.8; u.cable_mass = 0.4;            % half of the dual-channel ESC of a side, cables per motor
u.unit_mass = u.mass + u.gear_mass + u.brake_mass + u.esc_mass + u.cable_mass;
end
