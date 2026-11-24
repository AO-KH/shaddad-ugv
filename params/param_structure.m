function S = param_structure(cfg, W, g)
%PARAM_STRUCTURE  Chassis, mass properties, tracks, contact, implements-mechanics (owner: structure).
%   S = param_structure(cfg, W, g)
%     cfg fields (all optional): implement 0 none|1 mower|2 tiller|3 sprayer (= VC_IMPL),
%         tankFill0 (0..1, default 1), batteryMass_kg (default 18.5 = P.power.pack.mass_kg),
%         batteryPos_m (FRD w.r.t. R, default [0.05 0 -0.30]), fidelity (1 HiFi | 2 LoFi),
%         implDrive (1 internal ideal speed source - harness only | 2 POWER motor via Impl_Shaft)
%     W : P.world (needed for the initial state)       g : gravity (P.frames.g)
%
%   INTERNAL frames of this domain: body FRD, positions w.r.t. reference point R (ground level,
%   mid-gauge, mid contact length), plant integrated in flat-Earth NED (Aerospace 6DOF block).
%   The truth bus (Bus_TruthState) is converted to ENU/FLU at the domain boundary.
%   Revision B (D39): per-contact-point ISOTROPIC 2-D Dahl/Janosi shear (28 states, resultant slip
%   velocity) replaces the track-level longitudinal state + per-point lateral state; the effective
%   gauge factor chi_nom is identified from the HiFi model in T03 and is the single source for
%   P.electronics.mix.chi, P.sensors.ekf.chi0, P.structure.lofi.chi and the GNC limits.
if nargin < 1 || isempty(cfg), cfg = struct(); end
if nargin < 2, W = []; end
if nargin < 3 || isempty(g), g = 9.7876; end
def = struct('implement',0,'tankFill0',1,'batteryMass_kg',18.5,'batteryPos_m',[0.05 0 -0.30], ...
             'fidelity',1,'implDrive',2,'motorOption',[]);
fn = fieldnames(def);
for k = 1:numel(fn), if ~isfield(cfg, fn{k}), cfg.(fn{k}) = def.(fn{k}); end, end
S.cfg = cfg;

%% ---- geometry (real robot: XMachines MAX, docs/hardware/real_robot_components.md) ----
% [DATA] W 0.78 m, H 0.57 m, 220 kg + 50 kg module; [EST] track body 1.0 m (the 1730 mm of the spec sheet
% includes the module, which the spec render draws towed on a caster; the sprayer is mounted on the deck, the user
% 2026-11-15), gauge, track width, contact length, sprocket and wheel radii from the drawings.
S.geom = struct('L_overall',1.00,'W_overall',0.78,'H_body',0.57,'H_mast',1.05, ...
    'B',0.60,'b',0.13,'L',0.75,'r_idler',0.090,'r_roadwheel',0.077,'nRoadWheels',3, ...
    'groundClearance',0.12);
S.r_sprocket = 0.090;                        % [EST] pitch radius from the side view, agreed with DRIVE

%% ---- mass breakdown [m x y z lx ly lz] (FRD w.r.t. R); names in P_meta.structure ----
% the as-built robot (four Movematic 750 W units) sums to the 220 kg of the spec sheet [DATA]; the split is [EST]
% (docs/hardware/real_robot_components.md). Four drive units, one at each sprocket (front / rear, left / right):
% motor + gearbox + brake + half the dual ESC of the side + cables (param_motor_unit). The chassis frame takes the
% remainder with the as-built units, so another motor option changes the total mass by 4 x its unit-mass difference.
du = param_motor_unit(cfg.motorOption).unit_mass;
du0 = param_motor_unit('movematic750').unit_mass;
m_known = 20 + 2*22 + 18.5 + 6 + 3 + 4 + 4 + 6;      % panels, tracks, battery [DATA cells and box], bay, mast, misc, hitches
xs = 0.34;                                           % sprocket axles at +-(track length/2 - sprocket-wheel radius)
S.mass.components = [ ...
    220 - m_known - 4*du0  0  0  -0.25  0.95 0.50 0.20    % chassisFrame (remainder to the as-built 220 kg)
     20   0      0     -0.48  0.95 0.70 0.18    % bodyPanels, deck rail
     22   0     -0.30  -0.17  0.94 0.13 0.34    % trackAssyL (rubber track 8, sprocket 2, idler 2, 3 wheels 3, frame 7)
     22   0      0.30  -0.17  0.94 0.13 0.34    % trackAssyR
     du   xs    -0.17  -0.22  0.24 0.12 0.12    % driveLF (front-left sprocket)
     du  -xs    -0.17  -0.22  0.24 0.12 0.12    % driveLR (rear-left)
     du   xs     0.17  -0.22  0.24 0.12 0.12    % driveRF
     du  -xs     0.17  -0.22  0.24 0.12 0.12    % driveRR
    cfg.batteryMass_kg cfg.batteryPos_m 0.393 0.3585 0.087   % batteryPack 13S15P NMC (POWER), box XMT-P1-E01
      6   0.25   0     -0.40  0.30 0.20 0.10    % electronicsBay (MCU boards, DC/DC, harness)
      3   0.30   0     -0.75  0.10 0.20 0.35    % sensorMast (Phase 2 GNSS / LiDAR)
      4  -0.15   0     -0.40  0.30 0.30 0.10    % coolingMisc
      4   0.50   0     -0.25  0.08 0.40 0.15    % frontCoupler
      6  -0.50   0     -0.25  0.08 0.40 0.15];  % rearHitch

%% ---- implements (mechanics) ----
% D (bearing drag) = 1e-4 N m s/rad (0.03 N m at 293 rad/s, 8.6 W): the T12 target P_shaft = 1100 W
% (aero idle k_aero*w^3 = 300 W + cutting 800 W) leaves no room for the 86 W of the earlier 1e-3 (T03).
im.none    = struct('J',1e-3);
im.mower   = struct('mass',65,'pos',[0.90 0 -0.15],'dims',[0.60 1.05 0.25], ...
    'w',1.00,'nBlades',2,'R_blade',0.26,'J',0.05,'D',1e-4,'w_nom',293.2,'k_aero',1.19e-5, ...
    'e_A',800,'c_veg',40,'lambda_gw',0.4,'f_gw',0.10,'h_range',[0.03 0.12],'h_rate',0.02,'h_tau',0.2,'k_yaw',0, ...
    'h0',0.12,'v_eps',0.05,'w_min_frac',0.3);
im.tiller  = struct('mass',85,'pos',[-0.90 0 -0.30],'dims',[0.45 0.75 0.40], ...
    'w',0.60,'d_max',0.12,'d_nom',0.08,'R_rot',0.18,'J',0.35,'w_nom',26.18,'T_idle',4,'e_V',80e3, ...
    'kPush',0.3,'kLift',0.2,'lambda_skid',0.3,'mu_skid',0.4,'a_r',0.3,'n_t',6,'d_rate',0.03,'d_tau',0.3, ...
    'x_rot',-0.95,'d0',0,'d_on',1e-3,'v_eps',0.05,'w_min_frac',0.3);
% sprayer module: 50 kg dry [DATA spec sheet]; 40 L tank, layout and pump values [EST]
im.sprayer = struct('mass_dry',50,'pos_dry',[-0.05 0 -0.62],'dims_dry',[0.80 0.65 0.25], ...
    'pos_liq',[0 0 -0.68],'dims_liq',[0.50 0.40 0.20], ...
    'V_max',0.040,'rho_liq',1000,'nNoz',8,'Q_ref',1.0e-5,'p_ref',3e5,'p_set',3e5, ...
    'D_p',5.3e-7,'w_pump',314.16,'eta_m',0.85,'eta_v',0.90,'J',2e-3,'fill0',cfg.tankFill0, ...
    'Q_eps',1e-9,'x_boom',-0.20);
im.eta_impl = 0.85;
im.drive = struct('a_max',100,'tau_f',0.05);   % VC_IMPL_DRIVE == 1 harness drive: speed ramp rad/s^2, S-PS filter s
im.lift  = struct('duty_dead',0.05);           % |liftDuty| above this (and not at an end stop) = liftMoving
S.impl = im;

%% ---- mass properties of the active configuration ----
C = S.mass.components;
switch cfg.implement
    case 1, C = [C; im.mower.mass  im.mower.pos  im.mower.dims];
    case 2, C = [C; im.tiller.mass im.tiller.pos im.tiller.dims];
    case 3, C = [C; im.sprayer.mass_dry im.sprayer.pos_dry im.sprayer.dims_dry];
end
[me, rcg_e, Je] = local_massprops(C);
if cfg.implement == 3
    mliq = im.sprayer.V_max*im.sprayer.rho_liq;
    [mf, rcg_f, Jf] = local_massprops([C; mliq im.sprayer.pos_liq im.sprayer.dims_liq]);
    m0 = me + cfg.tankFill0*mliq;
    r_tank = im.sprayer.pos_liq;
else
    mf = me + 1e-3; rcg_f = rcg_e; Jf = Je; m0 = me; r_tank = rcg_e;
end
S.mp = struct('m0',m0,'me',me,'mf',mf,'Je',Je,'Jf',Jf,'rcg_e',rcg_e,'rcg_f',rcg_f,'r_tank',r_tank);
S.mp.rcg0 = (me*rcg_e + (m0-me)*r_tank)/m0;  % FRD w.r.t. R at t = 0
S.mp.rcg0_FLU = S.mp.rcg0 .* [1 -1 -1];      % for Bus_TruthState.r_cg_B
S.N_nom = m0*g/2;                            % static load per track

%% ---- track / shear (structure spec 6.4) ----
s  = [0 0.01 0.02 0.05 0.1 0.15 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 0.95 0.98 0.995 1];
x  = 2*s./max(1 - s, eps);
gg = 1 - (1 - exp(-x))./max(x, eps);  gg(1) = 0;  gg(end) = 1;
tr.r_s     = S.r_sprocket;
tr.J_eq    = 0.03;                           % [EST] sprocket + idler + 3 road wheels (excl. DRIVE inertia)
tr.m_belt  = 8;                              % [EST] 130 mm rubber track
tr.f_c     = 0.015;
tr.T_col   = tr.f_c*S.N_nom*tr.r_s;
tr.T_brk   = 1.3*tr.T_col;
tr.w_brk   = 0.05;
tr.c_visc  = 0.005*S.N_nom*tr.r_s^2;
tr.Nc      = 7;
tr.ell     = S.geom.L/2;                     % relaxation length = L/2 (structure spec 6.4; 0.375 m, was 0.50 for L 1.00)
tr.tau_d   = 0.01;
tr.etaMin  = 0.10;                           % (rev A only; the isotropic law needs no friction ellipse)
tr.shear_model = 2;                          % 1 rev A track-level (retired), 2 per-point isotropic 2-D (D39)
tr.vb_delay = 1;                             % samples (1 ms) Unit Delay on belt speed in the damping term (loop breaker, D36)
% chi_nom / chi_soil lines are rewritten by utilities/structure_identify_chi.m (keep the one-line format)
tr.chi_nom  = 1.3936;                        % IDENTIFIED by structure_identify_chi (HiFi, farm soil, R = 2 m turn at 0.5 m/s), D39
                                             % effective gauge factor B_eff/B (single source, D39); range check 1.2..1.8
tr.chi_soil = [1.4038 1.3936 1.4882 1.3281 1.3237];   % IDENTIFIED per soil class 1..5 (R = 2 m, 0.5 m/s)
tr.vEpsRR  = 0.05;
tr.gS      = [-fliplr(s(2:end)) s];
tr.gG      = [-fliplr(gg(2:end)) gg];
tr.B       = S.geom.B;                       % track gauge for chi_eff in MF-B
tr.chi_rmin = 0.05;                          % rad/s: chi_eff = NaN below this yaw rate
tr.v_eps_slip = 0.05;                        % m/s: slip = (v_b - v_g)/max(|v_b|, v_eps_slip)
S.track = tr;

%% ---- contact (2 x 7 points, FRD w.r.t. R) ----
L = S.geom.L;  B = S.geom.B;  Nc = tr.Nc;
xj = -L/2 + ((1:Nc) - 0.5)*L/Nc;
S.contact.Rc_R = [xj xj; -B/2*ones(1,Nc) B/2*ones(1,Nc); zeros(1,2*Nc)];   % 1..7 left, 8..14 right
S.contact.kc = 3.2e4;   S.contact.cc = 1.0e3;   S.contact.Nc = Nc;
S.contact.ell = tr.ell; S.contact.tau_d = tr.tau_d; S.contact.etaMin = tr.etaMin;
S.contact.vEpsRR = tr.vEpsRR; S.contact.gS = tr.gS; S.contact.gG = tr.gG;

%% ---- mower gauge wheels: rigid deck on two caster gauge wheels that touch the ground (the user, 2026-11-15) ----
% HiFi only (MF-C1 / MF-C2 of structure_contact: structure_gauge_points.m, structure_gauge_contact.m); on = 1 with the
% mower fitted. [EST] contact points at the front corners of the deck (0.60 x 1.05 m at x 0.90 m): x 1.15 m, y +-0.45 m
% (FRD w.r.t. R); per wheel the normal law of the track points (MF-A) without the soil scale, k = kc (N/m), c = cc
% (N s/m); no tangential force (casters: their rolling drag f_gw lambda_gw m g stays in the mower wrench, T12). The
% wheel height z (m, FRD w.r.t. R, + below the track bottom plane) makes the two wheels carry lambda_gw of the deck
% weight at rest on level ground with kScale 1 (the share of the former constant support): linear static equilibrium
% of the 2 Nc track springs and the two wheels in heave and pitch (no point lifted).
gw = struct('on', double(cfg.implement == 1), 'x', 1.15, 'y', 0.45, 'z', 0, 'k', S.contact.kc, 'c', S.contact.cc);
if gw.on
    F_gw0 = im.mower.lambda_gw*im.mower.mass*g;                              % N, both wheels
    zR    = (m0*g - F_gw0)/(2*Nc*S.contact.kc);                              % heave of R (m, down)
    th    = (m0*g*S.mp.rcg0(1) - F_gw0*gw.x)/(2*S.contact.kc*sum(xj.^2));   % nose-down pitch (rad)
    gw.z  = F_gw0/(2*gw.k) - zR - th*gw.x;
end
gw.R = [gw.x gw.x; -gw.y gw.y; gw.z gw.z];                                   % left, right
S.contact.gw = gw;

%% ---- LoFi, solver limits ----
% i_nom = steady straight-line slip on the default farm soil (soil 2) at constant speed: the 2-D shear law
% gives f(i L / K) = f_r / mu_p exactly (T4), so LoFi and HiFi travel the same distance on a straight row (T18).
i_nom = local_slip_farm(W, S.geom.L);
S.lofi = struct('chi',tr.chi_nom,'i_nom',i_nom,'w_eps',0.05,'h_cg',-S.mp.rcg0(3), ...
    'J_side',tr.J_eq + tr.m_belt*tr.r_s^2 + 0.5*m0*tr.r_s^2, ...
    'Izz',S.mp.Je(3,3));                     % D39: one chi for MCU mixer, EKF prior and LoFi
S.sim  = struct('h_max',2e-3);

%% ---- energy bookkeeping (Bus_StructureDiag.E_mech, D40) ----
S.energy = struct('k_belt', 0.5*(tr.J_eq/tr.r_s^2 + tr.m_belt));   % 1/2 (J_eq/r_s^2 + m_belt) v_b^2 per side

%% ---- descent speed cap (R-01 follow-up, Agents task descent, 2026-11-15; work/agentic/descent/structure/report_part2.md) ----
% Highest speed at which a brakes-locked stop may START without tipping the robot over its front contact (forward) or
% its rear contact (reverse). Rows = implement setting + 1 (0 none, 1 mower, 2 tiller, 3 sprayer with a full tank).
% Pitch axis (step 8, 2026-11-15): theta_bp = the MEASURED downhill body pitch in the direction of travel (deg), as the
% consumers index it: the MCU fw_imu_tilt pitch x sign(v), the GNC nav pitch or its look-ahead (the terrain-map chord
% over the contact length), each peak-held 1 s; between breakpoints the steeper one; 25 deg = TILT_WARN (0).
% Every entry is safe at the steepest terrain its bin can hold: the slope theta at which theta + settle(theta) - 0.75
% deg reads the breakpoint (settle = the body pitch relative to the terrain, + the leading end down; 0.75 deg = the
% fw_imu_tilt error bound, D83 2026-11-16: HiFi S2c read up to 0.7 deg low; 0.5 deg before step 13), and at least
% the breakpoint itself (the look-ahead's terrain chord), in each pre-stop state
% (coasting without motor torque; braked or brakes held), geometry and soil. The settle (planar model) is +0.07..0.27
% deg coasting and +1.15..1.46 deg braked at 20 deg for the sprayer (HiFi S2d +1.6), 2..6 deg leading end up for the
% raised tiller forward and the raised mower in reverse (work/agentic/descent/structure/descent_pitch_bins*.py).
% At that slope: min(model-geometry v_tip / 1.3, sharp-edge v_tip) - dv(theta), rounded down to 0.05 m/s,
% dv = 0.05 + g (sin(theta) - 0.035 cos(theta)) 0.05 s (speed gained before the brakes engage); a bound at or above
% 1.44 m/s (the MCU trip of a top-speed entry) gives the top speed 1.41 (no cap); planar HiFi contact and shear laws,
% the lower of farm soil and concrete [EST: contact stiffness and damping, track ends, tiller data]. The rows that found
% no tip up to 3 m/s keep their step-2 values (mower, tiller forward, raised mower in reverse: the 2 m/s test ceiling).
% 20.5-24 deg (step 8): the 20 deg design ramp reads 20.5-21.6 deg. The full-tank sprayer is bound by the coasting
% state (sharp edge, farm soil); braked only it would allow 0.35 m/s at 21 deg and 0.25 at 22 deg forward.
% The user, 2026-11-15: always active (MCU in every mode, GNC governor); the mower deck is rigid on gauge wheels that
% touch the ground (no tip found up to 2 m/s); no reversing with the tiller fitted until real tiller data exist; the
% sprayer is mounted on the deck (full-tank row: the robot does not know its tank level).
% The mower deck lifts (user, 2026-11-15): raised (0.09 m, gauge wheels off the ground) the robot is nose-heavy and a
% locked stop tips it from about 0.23 m/s on the flat, hence the *_raised rows; the user chose: the deck is down whenever
% the robot moves (GNC), and the MCU applies the *_raised rows as a backstop if the deck is reported up while moving.
S.descent = struct('theta_bp',[0 5 10 15 20 20.5 21 21.5 22 23 24 25], ...
    'v_fwd',[1.41 1.41 1.41 1.41 1.35 1.30 1.25 1.20 1.15 1.05 0.95 0; ...  % none
             1.41 1.41 1.40 1.35 1.30 1.30 1.30 1.30 1.30 1.30 1.30 0; ...  % mower (rigid deck on its gauge wheels)
             1.41 1.41 1.40 1.35 1.30 1.30 1.30 1.30 1.30 1.30 1.30 0; ...  % tiller, forward
             1.41 1.40 1.05 0.70 0.25 0.20 0.15 0.05 0    0    0    0], ... % sprayer, full tank
    'v_rev',[1.41 1.41 1.41 1.41 1.41 1.41 1.41 1.41 1.41 1.35 1.25 0; ...  % none
             1.41 1.41 1.40 1.35 1.30 1.30 1.30 1.30 1.30 1.30 1.30 0; ...  % mower
             0    0    0    0    0    0    0    0    0    0    0    0; ...  % tiller: no reversing (tips backward)
             1.41 1.35 1.00 0.65 0.20 0.15 0.10 0    0    0    0    0], ... % sprayer, full tank
    'v_fwd_raised',[0.15 0 0 0 0 0 0 0 0 0 0 0], ...                         % mower with the deck raised: nose-heavy
    'v_rev_raised',[1.41 1.41 1.40 1.35 1.30 1.30 1.30 1.30 1.30 1.30 1.30 0]); % mower with the deck raised

%% ---- initial state: settled CG height, terrain roll / pitch at W.start (utilities/structure_initial_state.m) ----
S.init.v0  = 0;
S.init.Vb0 = [0 0 0];
if ~isempty(W)
    st = W.start;
    if exist('structure_initial_state', 'file') == 2
        S.init = structure_initial_state(struct('world', W, 'structure', S, 'frames', struct('g', g)));
    else
        h0 = interp2(W.terrain.yN, W.terrain.xE, W.terrain.H, st(2), st(1), 'linear', 0);
        sink0 = m0*g/(2*Nc*S.contact.kc);
        hcg = -S.mp.rcg0(3);
        S.init.xme0  = [st(2) st(1) -(h0 + hcg - sink0)];        % CG position NED
        S.init.eul0  = [0 0 wrapToPi_local(pi/2 - st(3))];      % [phi theta psi] NED/FRD
        S.init.pose2D0 = st(:);
    end
else
    S.init.xme0 = [0 0 -0.35]; S.init.eul0 = [0 0 pi/2]; S.init.pose2D0 = [0;0;0];
end
end

function i = local_slip_farm(W, L)
% steady slip i on soil 2 (farm): 1 - (1 - exp(-x))/x = f_r/mu_p with x = i L / K (bisection)
if nargin < 1 || isempty(W) || ~isfield(W, 'soilTable')
    row = [2 0.66 0.025 0.55 0.07 0.8];
else
    row = W.soilTable(W.soilTable(:,1) == 2, :);
end
q = row(5)/row(2);  lo = 1e-9;  hi = 50;
for k = 1:200
    x = 0.5*(lo + hi);
    if 1 - (1 - exp(-x))/x < q, lo = x; else, hi = x; end
end
i = 0.5*(lo + hi)*row(3)/L;
end

function [m, rcg, J] = local_massprops(C)
m = sum(C(:,1));
rcg = sum(C(:,1).*C(:,2:4), 1)/m;
J = zeros(3);
for k = 1:size(C,1)
    mk = C(k,1); d = C(k,5:7); p = C(k,2:4) - rcg;
    Jb = mk/12*diag([d(2)^2+d(3)^2, d(1)^2+d(3)^2, d(1)^2+d(2)^2]);
    J = J + Jb + mk*(dot(p,p)*eye(3) - p'*p);
end
end

function a = wrapToPi_local(a)
a = atan2(sin(a), cos(a));
end
