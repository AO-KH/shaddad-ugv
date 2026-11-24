function S = motor_sizing(P, varargin)
%MOTOR_SIZING  Traction-motor sizing of the four-motor robot (two per track) for the spraying application.
%
%   S = motor_sizing()                 parameters of shaddad_params (sprayer, full tank)
%   S = motor_sizing(P)                a parameter struct from shaddad_params('Implement', 3, ...)
%   S = motor_sizing(P, 'Print', true) also print the table
%
%   Torque and speed per motor at the duty points of a hilly-farm spraying cycle, the RMS of the cycle on farm
%   soil and on wet loam, and the continuous / peak ratings of both motor options (param_motor_unit) at 45 degC
%   ambient. Quasi-static track forces: grade + motion resistance f*m*g*cos(theta) + m*a, and for skid-steer
%   turns the turning-resistance moment M = mu_t*m*g*cos(theta)*L/4 taken by the two tracks at the gauge B.
%   f = 0.107 is the farm-soil motion resistance measured in S6 (232 N for 220 kg, 0.58 N m per motor at 1 m/s);
%   wet loam f = 0.2, mu_t 0.6. Force per motor = track force / 2; T = F*r_s/(N*eta).
%   S: .points (table: name, v [m/s], T_farm, T_wet [N m per motor], n [rpm], weight), .rms_farm, .rms_wet,
%      .units (struct array: option, T_cont, T_cont45, T_peak, margin_farm, margin_wet)

ip = inputParser;
ip.addParameter('Print', false);
ip.parse(varargin{:});
if nargin < 1 || isempty(P), P = shaddad_params('Implement', 3, 'Assign', false); end
m = P.structure.mp.m0; g = P.frames.g;
gm = P.structure.geom; r = P.drive.gear.r_spr; N = P.drive.gear.N; eta = P.drive.gear.eta;
th = 20*pi/180; a = 0.8;
soils = struct('name', {'farm soil', 'wet loam'}, 'f', {0.107, 0.20}, 'mu_t', {0.5, 0.6});
% name, v [m/s], grade [rad], acceleration, turn (0 / 1), weight in the cycle
pts = {'flat spraying', 1.0, 0, 0, 0, 0.50; '20 deg climb', 1.0, th, 0, 0, 0.25; '20 deg climb + 0.8 m/s^2', 0.5, th, a, 0, 0.05; ...
       'pivot turn, flat', 0.3, 0, 0, 1, 0.10; 'turn on the 20 deg slope', 0.5, th, 0, 1, 0.10};
T = zeros(size(pts, 1), 2);
for s = 1:2
    f = soils(s).f; mu = soils(s).mu_t;
    for k = 1:size(pts, 1)
        [~, v, gr, ak, turn] = pts{k, 1:5};
        F_side = m*g*(sin(gr) + f*cos(gr))/2 + m*ak/2;                    % per track, straight
        if turn, F_side = F_side + mu*m*g*cos(gr)*gm.L/4/gm.B; end        % outer track of a turn
        T(k, s) = (F_side/2)*r/(N*eta);                                   % per motor
    end
end
w = [pts{:, 6}]';
S.points = table(pts(:, 1), [pts{:, 2}]', T(:, 1), T(:, 2), [pts{:, 2}]'*N/r*30/pi, w, ...
    'VariableNames', {'name', 'v', 'T_farm', 'T_wet', 'n_rpm', 'weight'});
S.rms_farm = sqrt(sum(w.*T(:, 1).^2)/sum(w));
S.rms_wet = sqrt(sum(w.*T(:, 2).^2)/sum(w));
k45 = 0.9;                                                                % continuous rating at 45 degC ambient
opts = {'movematic750', 'kw1'};
for k = 1:2
    u = param_motor_unit(opts{k});
    S.units(k) = struct('option', opts{k}, 'name', u.name, 'T_cont', u.T_cont, 'T_cont45', k45*u.T_cont, ...
        'T_peak', u.T_peak, 'margin_farm', k45*u.T_cont/S.rms_farm, 'margin_wet', k45*u.T_cont/S.rms_wet); %#ok<AGROW>
end
S.mass = m; S.k45 = k45; S.soils = soils;
if ip.Results.Print
    fprintf('Motor sizing, %.0f kg, four motors (per motor):\n', m); disp(S.points);
    fprintf('RMS: farm soil %.2f N m, wet loam %.2f N m\n', S.rms_farm, S.rms_wet);
    for k = 1:2
        fprintf('%-46s T_cont %.2f (45 degC %.2f), peak %.2f N m; margin farm %.2f, wet %.2f\n', S.units(k).name, ...
            S.units(k).T_cont, S.units(k).T_cont45, S.units(k).T_peak, S.units(k).margin_farm, S.units(k).margin_wet);
    end
end
end
