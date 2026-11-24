function [in, crit] = scn_S2_slope(~, varargin)
%SCN_S2_SLOPE  S2 Slope (Phase 1), architecture section 12 / D40 / D47 / D51; the S2 family per CR-001.
%
%   [in, crit] = scn_S2_slope(P)
%   [in, crit] = scn_S2_slope(P, 'StopTime', T)
%   [in, crit] = scn_S2_slope(P, 'Variant', v)     v = '' (default: the nominal S2 below) | 'b' | 'spray' | 'c' | 'd'
%   [in, crit] = scn_S2_slope(P, ..., 'Slope', th) the ramp at th deg instead of the variant's (below)
%   [in, crit] = scn_S2_slope(P, ..., 'Implement', k) implement k (VC_IMPL: 0 none, 1 mower, 2 tiller, 3 sprayer with a
%                                                  full tank) instead of the variant's (below); the default slope follows
%                                                  it (18.5 deg with the sprayer, else 20 deg) and the criteria take its
%                                                  row of the descent table
%   [in, crit] = scn_S2_slope(P, ..., 'Speed', v)  RC throttle for v_target v m/s (default 0.8; at most
%                                                  P.electronics.lim.v_max_man 1.4, full stick)
%   options of (c): 'Rearms' n (14), 'RearmStart' t0 (14 s), 'RearmPeriod' Tc (5 s); of (d): 'EstopTime' t_e (s, Inf:
%   no E-stop, the pilot run), 'StopDist' d (m, the stop distance limit, default 0.10)
%   Card G2-DSC step 12b (2026-11-16): the no-implement descent on the 20 deg design slope runs as (c) and (d) with
%   'Implement' 0; (d) with 'Speed' 1.2, 'EstopTime' 18.8 (from a pilot run without the E-stop: R passes x = 20 m at
%   18.77 s) and 'StopDist' 0.6 m (S8a: the front E-stop at 1.2 m/s stops within 0.6 m; the default 0.10 m is for the
%   sprayer's capped 0.15 m/s). At 1.2 m/s the robot descends at the capped speed (none row: held pitch 21.0-21.8 deg
%   on the straight, the target 1.10-1.20 m/s). Not at full stick ('Speed' 1.4): the top-speed OVERSPEED (1.44 m/s)
%   then parks the robot at the fillet entry, a nuisance trip the owner moved to card G2-MCU.
%   CR-001 (docs/changes/CR-001_sprayer_descent_envelope.md; the owner's decision of 2026-11-15, option 1): the
%   full-tank sprayer descends slopes up to about 18.5 deg only; the 20 deg design slope stays for no implement, the
%   mower and the tiller forward. Since card G2-DSC step 12 (2026-11-16) the family runs:
%     '' and (b)           no implement (VC_IMPL 0) on the 20 deg slope_test ramp
%     (spray), (c), (d)    the sprayer with a full tank (VC_IMPL 3) on an 18.5 deg ramp
%   (until then every variant carried the full-tank sprayer on the 20 deg ramp). A slope other than the world's ramp
%   scales slope_test: param_world builds H = tan(theta_w)*s(x), s the slope-free arc-length factor of the ramp and its
%   3 m fillets, and dHdE / dHdN as central differences of H, so H, dHdE and dHdN times tan(th)/tan(theta_w)
%   (theta_w = atan of the largest dHdE, 20 deg) are the grids of a th deg ramp, fillets and plateau height included;
%   P.structure.init is recomputed on the scaled ramp.
%   Nominal S2: slope_test world (ramp along +x from x = 10 to 30 m, 3 m fillets), RC throttle for v_target 0.8 m/s
%   from x = 0; the throttle is released at t_rel so the robot stops near mid-ramp (x ~ 20 m), holds 30 s, then the
%   throttle is applied again until the plateau (the hill start: the brakes release on the ramp). Criteria: mean ramp
%   speed >= 0.75 m/s; track slip <= 25 %; peak |I_pack| <= BMS continuous DCL (P.power.bms.I_dis_cont, 1 ms peak,
%   D40); roll-back during the hold <= 0.05 m; winding temperature rise <= 20 K; no BMS / ESC trip; no TILT_WARN /
%   TILT_STOP (D47); no OVERSPEED (b16): the hill start does not trip the descent cap (descent step 9, the hill-start
%   window); gearbox output torque <= T2N (P.drive.gear.T_rated_out, 10 s mean) and <= T2B (T_acc_out) peak (D51);
%   T-INT-04: mean pitch on the ramp = -th +/- 0.02 rad (nose up negative).
%   (spray) the nominal S2 with the full-tank sprayer on the 18.5 deg ramp: the sprayer's case on a slope it may use
%       (CR-001), with the heaviest climb and the sprayer's reverse cap at the hill start; the nominal criteria under
%       its own id.
%   Variants (run_scenario ids 'S2b' / 'S2spray' / 'S2c' / 'S2d', saved under their own ids so the nominal S2 result
%   stays the latest 'S2'). Their overrides come after the set-up (shaddad_params and scn_integration_overrides),
%   because P is rebuilt here and run_scenario's 'P' option does not reach it.
%   (b) class-B stop on the ramp (critfix R-01 system checks, 2026-11-15): the nominal run up to t_rel, then
%       ScenarioIn.flt.rc_link_loss from t_rel + 1 s to the end, while the robot stands on the 20 deg ramp before the
%       ESC's own park (P.drive.esc.t_park, 2 s at standstill); StopTime t_rel + 15 s. Criteria: the robot stands at
%       the loss (|v| <= 0.1 m/s: the MANUAL stop profile's tail is still ending 1 s after the release); CTRL_STOP
%       <= 0.13 s after the loss onset (D37; onset = edge + 0.01 s); parking (DRV_CMD L and R with enable 0 and
%       park_req 1) <= 1.1 s after CTRL_STOP; STANDBY <= 1.5 s after CTRL_STOP with both ESCs READY and both brakes
%       engaged; torque limits held (> 0) from the park request to STANDBY and 0 in STANDBY (D71); roll-back after the
%       stop <= 0.05 m; no STOP_FAIL (b22); HV_Ready and Safety_OK kept; no ESTOP / FAULT mode, no ESC QUICK_STOP /
%       FAULT / STO.
%   (c) descent with regen blocked, stop-go (descent task, 2026-11-15): the robot starts on the plateau at x = 36 m
%       facing -x and drives down the ramp at the 0.8 m/s throttle from t_thr, with the BMS charge current limit 0
%       (P.power.bms.chg.I_cool / I_hot = 0: the MCU's tau_lim_reg falls to the P_aux_est share). The MCU caps its
%       speed target at the descent cap - 0.10 m/s (P.electronics.lim.descent from P.structure.descent, indexed by the
%       measured downhill pitch) and parks at once when the speed passes the cap (OVERSPEED, bit 16, class B, latched:
%       the user keeps the latch). The operator then resets and re-arms: cycle k = 1..Rearms at t_k = RearmStart +
%       (k - 1)*RearmPeriod: throttle neutral t_k, arm off t_k + 0.3, CH9 reset (op.btn_reset) t_k + 1.0 to 1.3, arm on
%       t_k + 1.6 (STANDBY -> MANUAL, sticks neutral), throttle again t_k + 2.0. A cycle that finds the robot still
%       driving stops it first (throttle release, arm off: CTRL_STOP). StopTime 7 s after the last cycle. Criteria: CCL
%       0 and the robot in the top fillet (preconditions); at least one OVERSPEED park; the MCU speed target
%       (telem.v_target) never above descent.ref_fwd at the MCU's own pitch (telem.pitch peak-held t_hold, the run's
%       row, of this or the previous 10 ms sample); every OVERSPEED park safe by the structure park criterion (descent
%       step 8, work/agentic/descent/structure/report_step8.md section 6; utilities/scn_descent_parks.m): (a) the
%       largest |speed| from the trip until both brakes are engaged <= the row's v_fwd interpolated linearly at the
%       held pitch at the trip + dv(theta_R), and (b) the peak body pitch relative to the terrain from the trip until
%       2 s after the brakes engage <= 1/2 (atan(d/h) - theta_Rmax), theta_Rmax the steepest downhill terrain under R
%       in that window; the speed within bound (a) at every sample while the drive is on (|speed| <= v_fwd at the held
%       pitch there + dv(theta_R) there, while the MCU is in MANUAL, HOLD, AUTO_START, AUTO or CTRL_STOP with
%       OVERSPEED clear: a trip that does not fire fails here; card G2-DSC step 12c, review RID-DSC-03); no STOP_FAIL
%       (b22); no ESC FAULT (state 7); TILT_STOP (b15) never set; held on the brakes (displacement from the standstill
%       to the end of each STANDBY spell after the first park <= 0.02 m).
%   (d) hardware E-stop on the descent (descent task, 2026-11-15): as (c) with the normal CCL (regen available, so the
%       robot descends at the capped speed) and the throttle held, no re-arm cycles; the front mushroom E-stop
%       (ScenarioIn.hmi.estop_front_pressed) pressed at EstopTime to the end, the throttle released 0.5 s later;
%       StopTime EstopTime + 6 s. The default EstopTime 40 s is on the 18.5 deg straight with the sprayer at the
%       0.8 m/s throttle: the robot passes the bottom of the top fillet (x 28.5 m) at about 17 s and descends at the
%       capped 0.10-0.15 m/s, so it is at x ~ 25-26 m; another implement or speed takes it from a pilot run.
%       Criteria: the robot moves (|v| >= 0.05 m/s) on the straight part at the E-stop (preconditions); the stop safe
%       by the structure park criterion ((a) and (b) as in (c), from the plant-side hmi edge); STO (Safety_OK false)
%       <= 20 ms after that edge (as S8a); both brakes engaged <= 0.15 s after it; stop distance <= StopDist;
%       TILT_STOP never; held (creep <= 0.02 m from the standstill to the end).

ip = inputParser;
ip.addParameter('StopTime', 84);
ip.addParameter('Variant', '');
ip.addParameter('Slope', []);                % deg, [] = the implement's (CR-001: 20 without the sprayer, 18.5 with it)
ip.addParameter('Implement', []);            % VC_IMPL 0..3, [] = the variant's (CR-001: 3 for spray / c / d, else 0)
ip.addParameter('Speed', 0.8);               % m/s, the RC throttle's v_target
ip.addParameter('Rearms', 14);               % (c) operator reset + re-arm cycles
ip.addParameter('RearmStart', 14);           % (c) s, first cycle
ip.addParameter('RearmPeriod', 5);           % (c) s
ip.addParameter('EstopTime', []);            % (d) s, [] = the default below, Inf = no E-stop (pilot run)
ip.addParameter('StopDist', 0.10);           % (d) m, the stop distance limit
ip.parse(varargin{:});
T = ip.Results.StopTime;
var = lower(char(ip.Results.Variant));
defT = any(strcmp(ip.UsingDefaults, 'StopTime'));
if ~any(strcmp(var, {'', 'b', 'spray', 'c', 'd'}))
    error('scn_S2_slope:variant', 'Variant must be '''' (nominal), ''b'', ''spray'', ''c'' or ''d''');
end
impl = ip.Results.Implement;
if isempty(impl), impl = 3*any(strcmp(var, {'spray', 'c', 'd'})); end   % CR-001: 3 the full-tank sprayer, else 0
if ~any(impl == 0:3), error('scn_S2_slope:implement', 'Implement must be 0, 1, 2 or 3'); end
th = ip.Results.Slope;
if isempty(th), th = 20 - 1.5*(impl == 3); end    % deg: the 20 deg design slope, or the sprayer's 18.5 deg
P = shaddad_params('World', 2, 'Implement', impl, 'Phase', 1, 'Assign', false);   % slope_test
P = local_slope(P, th);
[x08, v08] = scn_rc_stick(P, ip.Results.Speed);   % the throttle (0.8 m/s unless 'Speed')
t_thr = 4;
x_mid = 20;                                  % stop target (mid-ramp)
t_rel = t_thr + 0.5 + x_mid/v08;             % release so that the robot stops near x_mid
t_res = t_rel + 2 + 30;                      % 30 s hold after the stop
tab = scn_rc_tab(0.5, 2);
tab.op.stick_thr = [0 0; t_thr x08; t_rel 0; t_res x08];
P = scn_integration_overrides(P);
VC = struct('VC_PHASE', 1, 'VC_IMPL', impl, 'VC_VIZ', 0, 'VC_OPSRC', 1);
id = ['S2' var];
if any(strcmp(var, {'b', 'c', 'd'}))         % variants b / c / d (header): own overrides, inputs and criteria
    [P, tab, T, crit] = local_variant(var, P, tab, T, defT, t_thr, t_rel, x08, ip.Results, impl + 1);
    in = scn_base_input(P, id, 'StopTime', T, 'Input', tab, 'VC', VC);
    in = scn_logging('apply', in, P, T);
    return
end
in = scn_base_input(P, id, 'StopTime', T, 'Input', tab, 'VC', VC);
in = scn_logging('apply', in, P, T);

M = @scn_metrics;
ramp = [11.5 28.5];                          % straight part (between the fillets)
mov = [t_thr + 3, t_rel; t_res + 4, T];      % steady moving phases
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
crit(end+1) = local_c([id '.moving'], 'precondition: x reached past mid-ramp before the stop', 'truth.pos_W', ...
    @(S) M('at', S.truth.pos_W, t_rel + 2)*[1; 0; 0], '>=', 15, 'm');
crit(end+1) = local_c([id '.v_mean'], 'mean speed on the ramp (steady moving phases)', 'truth.speed_fwd', ...
    @(S) mean(local_ramp(S, S.truth.speed_fwd, ramp, mov, 1)), '>=', 0.75, 'm/s');
crit(end+1) = local_c([id '.slip'], 'max |track slip| on the ramp (steady moving phases)', 'truth.slip_LR', ...
    @(S) max(max(abs(local_ramp(S, S.truth.slip_LR, ramp, mov, 1:2)))), '<=', 0.25, '1');
crit(end+1) = local_c([id '.I_peak'], 'peak |I_pack| (1 ms running maximum)', 'kpi.I_batt_peak', ...
    @(S) S.kpi.I_batt_peak.v(end), '<=', P.power.bms.I_dis_cont, 'A');
crit(end+1) = local_c([id '.rollback'], 'roll-back during the 30 s hold', 'truth.pos_W', ...
    @(S) local_rollback(S, t_rel, t_res), '<=', 0.05, 'm');
crit(end+1) = local_c([id '.dT_wind'], 'winding temperature rise', 'drive_truth.L/R.T_winding', ...
    @(S) max([max(S.drive_truth.L.T_winding.v) - S.drive_truth.L.T_winding.v(1), ...
              max(S.drive_truth.R.T_winding.v) - S.drive_truth.R.T_winding.v(1)]), '<=', 20, 'K');
crit(end+1) = local_c([id '.no_trip'], 'BMS fault level < 3, HV_Ready kept, no ESC FAULT / QUICK_STOP, no MCU FAULT', ...
    'bms_telem, telem', @(S) local_no_trip(S), 'true', 1, '');
crit(end+1) = local_c([id '.no_tilt'], 'TILT_WARN (b14) / TILT_STOP (b15) never set', 'telem.fault_word', ...
    @(S) ~any(M('bit', S.telem.fault_word.v, 14) | M('bit', S.telem.fault_word.v, 15)), 'true', 1, '');
crit(end+1) = local_c([id '.no_overspeed'], ['OVERSPEED (b16) never set: the hill start (brake release on the ' ...
    'ramp) does not trip the descent cap'], 'telem.fault_word', @(S) ~any(M('bit', S.telem.fault_word.v, 16)), ...
    'true', 1, '');
crit(end+1) = local_c([id '.tau_mean'], 'gearbox output torque, max 10 s moving mean of |tau_spr|', ...
    'drive_truth.tau_spr', @(S) local_tau10(S), '<=', P.drive.gear.T_rated_out, 'N*m');
crit(end+1) = local_c([id '.tau_peak'], 'gearbox output torque peak |tau_spr|', 'drive_truth.tau_spr', ...
    @(S) max([max(abs(S.drive_truth.L.tau_spr.v)), max(abs(S.drive_truth.R.tau_spr.v))]), '<=', ...
    P.drive.gear.T_acc_out, 'N*m');
crit(end+1) = local_c([id '.pitch'], sprintf(['T-INT-04: mean pitch on the ramp + %g deg = %.4f rad (nose up ' ...
    'negative)'], th, th*pi/180), 'truth.eul(2)', @(S) mean(local_ramp(S, S.truth.eul, ramp, mov, 2)) + th*pi/180, ...
    'in', [-0.02 0.02], 'rad');
end

% -------------------------------------------------------------------------------------------
function P = local_slope(P, th)
% the slope_test ramp at th deg (header): H, dHdE and dHdN times tan(th)/tan(theta_w), P.structure.init again
tr = P.world.terrain;
th_w = atan(max(tr.dHdE(:)));                % the world's ramp (slope_test: 20 deg along +x)
if abs(th*pi/180 - th_w) < 1e-9, return, end
k = tan(th*pi/180)/tan(th_w);
P.world.terrain.H = k*tr.H;
P.world.terrain.dHdE = k*tr.dHdE;
P.world.terrain.dHdN = k*tr.dHdN;
P.structure.init = structure_initial_state(P);
end

function c = local_c(id, desc, sig, f, op, lim, unit)
c = struct('id', id, 'desc', desc, 'signal', sig, 'metric', f, 'op', op, 'limit', lim, 'unit', unit, 'blocked', '');
end

function v = local_ramp(S, sig, ramp, mov, cols)
x = scn_metrics('resample', S.truth.pos_W, sig.t);
m = x(:, 1) >= ramp(1) & x(:, 1) <= ramp(2);
w = false(size(m));
for k = 1:size(mov, 1)
    w = w | (sig.t >= mov(k, 1) & sig.t <= mov(k, 2));
end
v = double(sig.v(m & w, cols));
end

function r = local_rollback(S, t_rel, t_res)
v = S.truth.speed_fwd;
t0 = scn_metrics('first', v, @(u) abs(u) < 0.01, t_rel + 0.2);
if isnan(t0) || t0 >= t_res, r = NaN; return, end
p = S.truth.pos_W;
m = p.t >= t0 & p.t <= t_res;
x = p.v(m, 1);
r = max(0, x(1) - min(x));
end

function tf = local_no_trip(S)
b = S.bms_telem;
tf = max(double(b.Fault_Level.v)) < 3 && all(b.HV_Ready.v(b.HV_Ready.t >= 1.5) ~= 0);
ds = double(S.telem.drive_state.v);
tf = tf && ~any(ds(:) == 7 | ds(:) == 6) && ~any(double(S.telem.mode.v) == 9);
end

function m = local_tau10(S)
m = 0;
for side = {'L', 'R'}
    s = S.drive_truth.(side{1}).tau_spr;
    m = max(m, max(scn_metrics('movmean', s.t, abs(double(s.v)), 10)));
end
end

% -------------------------------------------------------------------------------------------
% variants b (critfix), c and d (descent task; CR-001 since step 12), header
function [P, tab, T, crit] = local_variant(var, P, tab, T, defT, t_thr, t_rel, x08, o, row)
M = @scn_metrics;
crit = struct('id', {}, 'desc', {}, 'signal', {}, 'metric', {}, 'op', {}, 'limit', {}, 'unit', {}, 'blocked', {});
switch var
    case 'b'
        t_loss = t_rel + 1;                      % the robot stands on the ramp, before the ESC's own 2 s park
        tab.flt.rc_link_loss = [0 0; t_loss 1];  % lost to the end
        if defT, T = ceil(t_rel + 15); end
        t_on = t_loss + 0.01;                    % onset at RC_Link_Sim (10 ms input RT, as S6a)
        crit(end+1) = local_c('S2b.moving', 'precondition: x reached past mid-ramp before the stop', 'truth.pos_W', ...
            @(S) M('at', S.truth.pos_W, t_rel + 1)*[1; 0; 0], '>=', 15, 'm');
        crit(end+1) = local_c('S2b.still', 'precondition: |speed| at the loss (the robot stands, or comes to rest)', ...
            'truth.speed_fwd', @(S) abs(M('at', S.truth.speed_fwd, t_loss)), '<=', 0.1, 'm/s');
        crit(end+1) = local_c('S2b.ctrl_stop', 'CTRL_STOP after the loss onset (D37 <= 0.13 s)', 'telem.mode', ...
            @(S) local_tc(S, t_loss) - t_on, '<=', 0.13, 's');
        crit(end+1) = local_c('S2b.park', 'parking (DRV_CMD L and R: enable 0, park_req 1) after CTRL_STOP', ...
            'mcu_out10.can1_drv, telem.mode', @(S) local_tpark(S, t_loss) - local_tc(S, t_loss), '<=', 1.1, 's');
        crit(end+1) = local_c('S2b.standby', 'STANDBY after CTRL_STOP (parking ends when both brakes confirm)', ...
            'telem.mode', @(S) local_tsb(S, t_loss) - local_tc(S, t_loss), '<=', 1.5, 's');
        crit(end+1) = local_c('S2b.brakes', 'at STANDBY: both ESCs READY, both brakes engaged (f_brake_rel <= 0.01)', ...
            'telem.drive_state, drive_truth.L/R.f_brake_rel', @(S) local_brakes_at(S, local_tsb(S, t_loss)), ...
            'true', 1, '');
        crit(end+1) = local_c('S2b.limits', 'torque limits > 0 from the park request to STANDBY, 0 in STANDBY (D71)', ...
            'telem.tau_lim_mot, telem.tau_lim_reg', @(S) local_limits(S, t_loss), 'true', 1, '');
        crit(end+1) = local_c('S2b.rollback', 'roll-back from the stop to the end', 'truth.pos_W', ...
            @(S) local_rollback(S, t_rel, Inf), '<=', 0.05, 'm');
        crit(end+1) = local_c('S2b.no_stop_fail', 'STOP_FAIL (b22) never set', 'telem.fault_word', ...
            @(S) ~any(M('bit', S.telem.fault_word.v, 22)), 'true', 1, '');
        crit(end+1) = local_c('S2b.hv_on', 'HV_Ready kept (t >= 1.5 s), Safety_OK kept (t >= 0.5 s): no STO', ...
            'bms_telem.HV_Ready, power_status.Safety_OK', @(S) local_hv_on(S), 'true', 1, '');
        crit(end+1) = local_c('S2b.no_trip', 'no ESTOP / FAULT mode, no ESC QUICK_STOP / FAULT / STO (t >= 1 s)', ...
            'telem.mode, telem.drive_state', @(S) local_no_trip_b(S), 'true', 1, '');
    case {'c', 'd'}
        x_s = 36;                                % plateau (x > 31.5 m), 4.5 m before the top fillet
        P.world.start = [x_s 0 pi];              % facing -x: forward is downhill past the fillet
        P.structure.init = structure_initial_state(P);
        if var == 'c'
            P.power.bms.chg.I_cool(:) = 0;       % BMS charge current limit (CCL) 0 at every cell temperature: the
            P.power.bms.chg.I_hot(:) = 0;        % regen is blocked, as with a hot (>= 50 degC) or full pack
            [tab, t_end] = local_rearm(tab, x08, t_thr, o.RearmStart, o.RearmPeriod, o.Rearms);
            if defT, T = ceil(t_end); end
            crit(end+1) = local_c('S2c.ccl0', 'precondition: BMS charge current limit (max over t >= 1 s)', ...
                'bms_telem.I_chg_lim_A', @(S) local_max_after(S.bms_telem.I_chg_lim_A, 1), '<=', 0.5, 'A');
            crit(end+1) = local_c('S2c.descent', 'precondition: the robot entered the top fillet (min x of R)', ...
                'truth.pos_W', @(S) min(S.truth.pos_W.v(:, 1)), '<=', 31.0, 'm');
            crit(end+1) = local_c('S2c.parks', 'OVERSPEED (b16) parks: rising edges of bit 16 after t_thr', ...
                'telem.fault_word', @(S) numel(local_trips(S, t_thr)), '>=', 1, '');
            crit(end+1) = local_c('S2c.target_cap', ['max(telem.v_target - descent.ref_fwd at the MCU pitch ' ...
                'peak-held t_hold, the run''s row, this or the previous sample)'], 'telem.v_target, telem.pitch', ...
                @(S) local_target_excess(S, P, row), '<=', 0.005, 'm/s');
            crit(end+1) = local_c('S2c.run_v', ['speed over the run: largest |speed| - (V_lin(held pitch) + ' ...
                'dv(theta_R)) at every sample with the drive on (MANUAL / HOLD / AUTO_START / AUTO / CTRL_STOP, ' ...
                'OVERSPEED clear; scn_descent_parks)'], ...
                'truth.speed_fwd, truth.grade_long, telem.pitch, telem.mode, telem.fault_word', ...
                @(S) local_run_bound(S, P, row), '<=', 0, 'm/s');
            crit(end+1) = local_c('S2c.park_v', ['structure park criterion (a), largest over the OVERSPEED parks of ' ...
                'v_park - (V_lin(p_trip) + dv(theta_R)) (scn_descent_parks)'], ...
                'truth.speed_fwd, telem.pitch, truth.grade_long, drive_truth.L/R.f_brake_rel', ...
                @(S) local_park_crit(S, P, local_trips(S, t_thr), row, 'v'), '<=', 0, 'm/s');
            crit(end+1) = local_c('S2c.park_pitch', ['structure park criterion (b), largest over the parks of the peak ' ...
                'pitch relative to the terrain (trip to 2 s after the brakes) - 1/2 (atan(d/h) - theta_Rmax)'], ...
                'truth.eul, truth.grade_long', @(S) local_park_crit(S, P, local_trips(S, t_thr), row, 'pitch'), ...
                '<=', 0, 'deg');
            crit(end+1) = local_c('S2c.no_stop_fail', 'STOP_FAIL (b22) never set', 'telem.fault_word', ...
                @(S) ~any(M('bit', S.telem.fault_word.v, 22)), 'true', 1, '');
            crit(end+1) = local_c('S2c.no_esc_fault', 'no ESC FAULT (state 7, e.g. the OVERSPEED trip 0x0020)', ...
                'telem.drive_state', @(S) ~any(double(S.telem.drive_state.v(:)) == 7), 'true', 1, '');
            crit(end+1) = local_c('S2c.no_tilt_stop', 'TILT_STOP (b15) never set', 'telem.fault_word', ...
                @(S) ~any(M('bit', S.telem.fault_word.v, 15)), 'true', 1, '');
            crit(end+1) = local_c('S2c.held', ['largest displacement from the standstill to the end of one STANDBY ' ...
                'spell after the first park (brakes hold)'], 'truth.pos_W, telem.mode', ...
                @(S) local_standby_creep(S, t_thr), '<=', 0.02, 'm');
        else
            t_e = o.EstopTime;
            if isempty(t_e)                      % on the 18.5 deg straight at the capped 0.10-0.15 m/s (header)
                t_e = 40;
            end
            tab.op.stick_thr = [0 0; t_thr x08]; % throttle held
            if isfinite(t_e)
                tab.op.stick_thr = [tab.op.stick_thr; t_e + 0.5 0];
                tab.hmi.estop_front_pressed = [0 0; t_e 1];
                if defT, T = ceil(t_e + 6); end
            elseif defT
                T = 60;
            end
            t_pl = t_e + 0.01;                   % plant-side hmi edge (Plant_In RT 0.01 -> 0.001, D62)
            crit(end+1) = local_c('S2d.moving', 'precondition: |speed| 0.1 s before the E-stop', 'truth.speed_fwd', ...
                @(S) abs(M('at', S.truth.speed_fwd, t_e - 0.1)), '>=', 0.05, 'm/s');
            crit(end+1) = local_c('S2d.on_ramp', 'precondition: x of R at the E-stop on the straight part', ...
                'truth.pos_W', @(S) M('at', S.truth.pos_W, t_e)*[1; 0; 0], 'in', [11.5 28.5], 'm');
            crit(end+1) = local_c('S2d.park_v', ['structure park criterion (a) of the E-stop stop: v_park - ' ...
                '(V_lin(p) + dv(theta_R)) from the plant-side edge (scn_descent_parks)'], ...
                'truth.speed_fwd, telem.pitch, truth.grade_long, drive_truth.L/R.f_brake_rel', ...
                @(S) local_park_crit(S, P, t_pl, row, 'v'), '<=', 0, 'm/s');
            crit(end+1) = local_c('S2d.park_pitch', ['structure park criterion (b) of the E-stop stop: peak pitch ' ...
                'relative to the terrain (edge to 2 s after the brakes) - 1/2 (atan(d/h) - theta_Rmax)'], ...
                'truth.eul, truth.grade_long', @(S) local_park_crit(S, P, t_pl, row, 'pitch'), '<=', 0, 'deg');
            crit(end+1) = local_c('S2d.sto', 'STO (Safety_OK false) after the plant-side hmi edge', ...
                'power_status.Safety_OK', @(S) M('first', S.power_status.Safety_OK, @(v) v == 0, t_e) - t_pl, ...
                '<=', 0.020, 's');
            crit(end+1) = local_c('S2d.brakes', 'both brakes engaged (f_brake_rel <= 0.01) after the plant-side edge', ...
                'drive_truth.L/R.f_brake_rel', @(S) local_t_brakes(S, t_e) - t_pl, '<=', 0.15, 's');
            crit(end+1) = local_c('S2d.stop_dist', 'stop distance from the plant-side edge to standstill (0.5 s)', ...
                'truth.pos_W', @(S) local_dist_still(S, t_pl), '<=', o.StopDist, 'm');
            crit(end+1) = local_c('S2d.no_tilt_stop', 'TILT_STOP (b15) never set', 'telem.fault_word', ...
                @(S) ~any(M('bit', S.telem.fault_word.v, 15)), 'true', 1, '');
            crit(end+1) = local_c('S2d.held', 'creep from the standstill after the E-stop to the end', 'truth.pos_W', ...
                @(S) local_hold_after(S, t_pl), '<=', 0.02, 'm');
        end
end
end

function [tab, t_end] = local_rearm(tab, x08, t_thr, t0, Tc, n)
% (c) the throttle from t_thr, then n operator reset + re-arm cycles (header); t_end = 7 s after the last cycle
thr = [0 0; t_thr x08];
arm = tab.op.sw_arm;                         % [0 0; t_arm 1] of scn_rc_tab
rst = [0 0];
for k = 1:n
    tk = t0 + (k - 1)*Tc;
    thr = [thr; tk 0; tk + 2.0 x08]; %#ok<AGROW>
    arm = [arm; tk + 0.3 0; tk + 1.6 1]; %#ok<AGROW>
    rst = [rst; tk + 1.0 1; tk + 1.3 0]; %#ok<AGROW>
end
tab.op.stick_thr = thr;
tab.op.sw_arm = arm;
tab.op.btn_reset = rst;
t_end = t0 + (n - 1)*Tc + 7;
end

function [th, t] = local_held_pitch(S, P)
% the MCU's downhill pitch forward (deg, + nose down), peak-held over descent.t_hold as Descent_cap does it (the
% Tapped Delay starts at theta_imu): the last round(t_hold/dt) samples, dt the step of the telem grid (the MCU's
% round(t_hold/Ts.ctrl) on the 10 ms grid; a decimated log keeps the t_hold window; step 12c, review RID-DSC-04)
D = P.electronics.lim.descent;
s = S.telem.pitch;  t = s.t;
x = double(s.v(:, 1))*180/pi;
n = max(1, round(D.t_hold/median(diff(t))));
th = movmax([D.theta_imu*ones(n - 1, 1); x], [n - 1 0]);
th = th(n:end);
end

function ref = local_ref_tab(P, th, row)
% the speed-target limit descent.ref_fwd of the row at downhill pitch th (deg): the first breakpoint at or above th
% (the steeper one), clipped to the table
D = P.electronics.lim.descent;
ref = zeros(size(th));
for k = 1:numel(th)
    j = find(D.theta_bp >= th(k), 1, 'first');
    if isempty(j), j = numel(D.theta_bp); end
    ref(k) = D.ref_fwd(row, j);
end
end

function e = local_target_excess(S, P, row)
% v_target against ref_fwd at the held pitch of this or the previous Task_10 sample (the logged v_target of a sample
% where the pitch crosses a breakpoint may still carry the previous sample's limit)
[th, t] = local_held_pitch(S, P);
ref = local_ref_tab(P, th, row);
ref = max(ref, [ref(1); ref(1:end-1)]);
vt = double(scn_metrics('resample', S.telem.v_target, t));
e = max(vt(:, 1) - ref);
end

function t = local_trips(S, t0)
% OVERSPEED trips: the rising edges of fault bit 16 at or after t0
fw = S.telem.fault_word;
b = scn_metrics('bit', fw.v, 16);
k = find(b(2:end) & ~b(1:end-1)) + 1;
t = fw.t(k(fw.t(k) >= t0 - 1e-9));
end

function e = local_run_bound(S, P, row)
% (c) bound (a) of the park criterion at every sample with the drive on (scn_descent_parks, B): the largest excess
[~, B] = scn_descent_parks(S, P, [], row, 1);
e = max(B.v(B.on) - B.v_bound(B.on));
if isempty(e), e = NaN; end
end

function e = local_park_crit(S, P, t, row, kind)
% the structure park criterion (scn_descent_parks) of the stops starting at t (forward descent): the largest excess
E = scn_descent_parks(S, P, t, row, 1);
if isempty(E), e = NaN; return, end
if strcmp(kind, 'v')
    e = max([E.v_park] - [E.v_bound]);
else
    e = max([E.rel_peak] - [E.rel_limit]);
end
end

function t = local_t_brakes(S, t0)
% first time >= t0 with both brakes engaged (f_brake_rel <= 0.01)
fL = S.drive_truth.L.f_brake_rel;  fR = S.drive_truth.R.f_brake_rel;
bR = scn_metrics('resample', fR, fL.t);
k = find(fL.t >= t0 - 1e-9 & double(fL.v(:, 1)) <= 0.01 & double(bR(:, 1)) <= 0.01, 1, 'first');
if isempty(k), t = NaN; else, t = fL.t(k); end
end

function d = local_dist_still(S, t0)
ts = local_still(S, t0, 0.5);
if isnan(ts), d = Inf; return, end
d = scn_metrics('path', S.truth.pos_W, t0, ts);
end

function d = local_hold_after(S, t0)
ts = local_still(S, t0, 0.5);
if isnan(ts), d = Inf; return, end
p = S.truth.pos_W;
q = p.v(p.t >= ts - 1e-9, 1:2);
d = max(sqrt(sum((q - q(1, :)).^2, 2)));
end

function d = local_standby_creep(S, t0)
% largest displacement of R within one STANDBY spell that starts after t0, from the standstill in that spell (|v| <
% 0.01 m/s for 0.5 s; STANDBY can come while the locked tracks still skid) to the end of the spell
md = S.telem.mode;  sb = double(md.v(:, 1)) == 2 & md.t >= t0;
k1 = find(sb & ~[false; sb(1:end-1)]);  k2 = find(sb & ~[sb(2:end); false]);
d = 0;
p = S.truth.pos_W;
for j = 1:numel(k1)
    ts = local_still(S, md.t(k1(j)), 0.5);
    if isnan(ts) || ts >= md.t(k2(j)), continue, end
    q = p.v(p.t >= ts - 1e-9 & p.t <= md.t(k2(j)) + 1e-9, 1:2);
    d = max(d, max(sqrt(sum((q - q(1, :)).^2, 2))));
end
end

function t = local_tc(S, t0)
% first CTRL_STOP at or after t0
t = scn_metrics('first', S.telem.mode, @(v) v == 7, t0);
end

function t = local_tsb(S, t0)
% first STANDBY after that CTRL_STOP
tc = local_tc(S, t0);
if isnan(tc), t = NaN; return, end
t = scn_metrics('first', S.telem.mode, @(v) v == 2, tc);
end

function [pk, en] = local_drvcmd(S)
% DRV_CMD L / R (ICD 4: byte 0 b0 enable, b1 park_req): park_req on both sides, enable on either side
c = S.mcu_out10.can1_drv;
dL = double(c.drv_cmd_L.data.v);  dR = double(c.drv_cmd_R.data.v);
pk = bitget(dL(:, 1), 2) & bitget(dR(:, 1), 2);
en = bitget(dL(:, 1), 1) | bitget(dR(:, 1), 1);
end

function t = local_tpark(S, t0)
% first DRV_CMD sample at or after CTRL_STOP with park_req 1 and enable 0 on both sides
tc = local_tc(S, t0);
t = NaN;
if isnan(tc), return, end
[pk, en] = local_drvcmd(S);
td = S.mcu_out10.can1_drv.drv_cmd_L.data.t;
k = find(td >= tc - 1e-9 & pk & ~en, 1, 'first');
if ~isempty(k), t = td(k); end
end

function tf = local_brakes_at(S, ts)
tf = false;
if isnan(ts), return, end
ds = double(scn_metrics('at', S.telem.drive_state, ts));
fL = scn_metrics('at', S.drive_truth.L.f_brake_rel, ts);
fR = scn_metrics('at', S.drive_truth.R.f_brake_rel, ts);
tf = all(ds == 2) && fL <= 0.01 && fR <= 0.01;
end

function tf = local_limits(S, t0)
% D71: tau_lim_mot and tau_lim_reg held (> 0, both sides) from the park request until STANDBY, 0 in STANDBY
tp = local_tpark(S, t0);  ts = local_tsb(S, t0);
tf = false;
if isnan(tp) || isnan(ts), return, end
lm = S.telem.tau_lim_mot;  lr = S.telem.tau_lim_reg;
w = lm.t >= tp - 1e-9 & lm.t < ts - 1e-9;
held = any(w) && all(min([double(lm.v(w, :)) double(lr.v(w, :))], [], 2) > 0);
zero = all(double([scn_metrics('at', lm, ts + 0.05) scn_metrics('at', lr, ts + 0.05)]) == 0);
tf = held && zero;
end

function tf = local_hv_on(S)
b = S.bms_telem.HV_Ready;  s = S.power_status.Safety_OK;
tf = all(b.v(b.t >= 1.5) ~= 0) && all(s.v(s.t >= 0.5) ~= 0);
end

function tf = local_no_trip_b(S)
md = double(S.telem.mode.v);
d = S.telem.drive_state;  ds = double(d.v(d.t >= 1, :));
tf = ~any(md == 8 | md == 9) && ~any(ds(:) == 6 | ds(:) == 7 | ds(:) == 8);
end

function m = local_max_after(sig, t0)
m = max(double(sig.v(sig.t >= t0, :)), [], 'all');
end

function t = local_still(S, t0, dur)
% first time >= t0 from which |speed_fwd| < 0.01 m/s holds for dur seconds (not a zero crossing of the rocking)
v = S.truth.speed_fwd;
ok = abs(double(v.v(:, 1))) < 0.01 & v.t >= t0 - 1e-9;
n = max(1, round(dur/median(diff(v.t))));
run = conv(double(ok), ones(n, 1));
k = find(run(n:end) >= n, 1, 'first');
if isempty(k), t = NaN; else, t = v.t(k); end
end
