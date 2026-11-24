function tbs = tb_structure_defaults(P)
%TB_STRUCTURE_DEFAULTS  Default scenario struct tbs of the bench tests/structure/tb_structure.slx (T03).
%   tbs = tb_structure_defaults()     neutral defaults (stored in the bench model workspace)
%   tbs = tb_structure_defaults(P)    same, with the PI correction gain kw from P (chi_nom, B, r_s)
%   Fields: see scripts/build/build_tb_structure.m. Sprocket commands are in rad/s (speed mode) or N m
%   (torque mode); a belt speed v [m/s] is the sprocket speed v/P.structure.r_sprocket.
kw = 1.39*0.60/(2*0.090);                   % chi_nom B / (2 r_s) of the real robot (neutral default)
if nargin >= 1 && ~isempty(P)
    kw = P.structure.track.chi_nom*P.structure.geom.B/(2*P.structure.r_sprocket);
end
tbs = struct();
tbs.t     = [0 1e6];
tbs.uL    = [0 0];
tbs.uR    = [0 0];
tbs.rref  = [0 0];
tbs.pi_on = 0;
tbs.pi_Kp = 0.4;
tbs.pi_Ki = 2.0;
tbs.pi_lim = 0.4;
tbs.kw    = kw;
tbs.t_impl = [0 1e6];
tbs.w_impl = [0 0];
tbs.nozzleOn = false(8, 1);
tbs.liftDuty = 0;
tbs.envdist = struct('F_ext_B', zeros(3, 1), 'M_ext_B', zeros(3, 1));
tbs.envovr  = struct('use_T', false, 'T_amb', 318.15, 'use_G', false, 'G_solar', 0, ...
                     'use_soil', false, 'soil_id', uint8(2), 'dock_contact_force', false);
tbs.decim = 10;
end
