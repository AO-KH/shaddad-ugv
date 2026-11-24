function r = gnc_mission_run(P, plan, varargin)
%GNC_MISSION_RUN  Run the GNC mission bench tests/gnc/tb_gnc_mission.slx (T15 test helper).
%
%   r = gnc_mission_run(P, plan, 'Name', value, ...)
%     P      parameter struct (shaddad_params('World', w, 'Assign', false)); never written to the base workspace
%     plan   Bus_MissionPlan struct (gnc_compileMission)
%   Options (scenario tables, see params/param_gnc.m 'Mission bench'):
%     'StopTime' s, 'x0' [x y psi] (default plan.home), 'mcu' scn_mcu, 'cmd' [t cmd] (seq added), 'bms' scn_bms,
%     'soc' constant SOC (when 'bms' is not given), 'nav' scn_nav, 'plant' scn_plant, 'dyn' dynamic obstacles,
%     'Tracker' VC_ROW_TRACKER (1), 'Teb' VC_TEB (1), 'AutoStart' P.gnc.fsm.auto_start, 'Lidar' 0/1,
%     'Drift' m/s, 'Tilt' [pitch roll], 'Spray' [mask density], 'scan' scn_scan [t valid] (0 = blank scan),
%     'tb' struct of extra P.gnc.tb fields, 'DeckDown' true (default: the bench has no mower deck, so the FSM gets the
%     deck as down at the start, P.gnc.fsm.deck_t0 = t_deck) | false (the product default: height unknown, the deck
%     counts as down after t_deck of AUTO)
%   r: t (0.01 s) truth [x y psi v r]; tc (0.05 s) cmd fields, fsm fields (logged inside Shaddad_GNC), obs,
%      debug fields; ts (0.2 s) status fields; tr (1 s grid -> 0.05 s) rp; wall time; P (as simulated)
%   Ts, Bus_* and cs_SimFixed must exist in base (shaddad_params / startup); the scenario reaches the bench
%   through Simulink.SimulationInput.setVariable only.

ip = inputParser;
ip.addParameter('StopTime', 30);
ip.addParameter('x0', []);
ip.addParameter('mcu', []);
ip.addParameter('cmd', []);
ip.addParameter('bms', []);
ip.addParameter('soc', 0.8);
ip.addParameter('nav', []);
ip.addParameter('plant', []);
ip.addParameter('dyn', []);
ip.addParameter('Tracker', 1);
ip.addParameter('Teb', 1);
ip.addParameter('AutoStart', false);
ip.addParameter('DeckDown', true);
ip.addParameter('Lidar', 1);
ip.addParameter('Drift', 0);
ip.addParameter('Tilt', [0 0]);
ip.addParameter('Spray', []);
ip.addParameter('scan', []);
ip.addParameter('tb', struct());
ip.parse(varargin{:});
o = ip.Results;

P.gnc.mission = plan;
if isempty(o.x0), o.x0 = plan.home(:)'; end
tb = P.gnc.tb;
tb.x0 = o.x0;  tb.drift = o.Drift;  tb.tilt = o.Tilt;  tb.lidar_on = o.Lidar;
if ~isempty(o.mcu), tb.scn_mcu = o.mcu; end
if ~isempty(o.cmd)
    c = o.cmd;
    tb.scn_cmd = [0 0 0; c(:, 1) c(:, 2) (1:size(c, 1))'];
end
if ~isempty(o.bms)
    tb.scn_bms = o.bms;
else
    tb.scn_bms = [0 o.soc 0; 1e5 o.soc 0];
end
if ~isempty(o.nav), tb.scn_nav = o.nav; end
if ~isempty(o.plant), tb.scn_plant = o.plant; end
if ~isempty(o.scan), tb.scn_scan = o.scan; end
dyn = zeros(size(tb.dyn));
if ~isempty(o.dyn), dyn(1:size(o.dyn, 1), :) = o.dyn; end
tb.dyn = dyn;
if ~isempty(o.Spray), tb.spray_mask = o.Spray(1); tb.spray_density = o.Spray(2); end
fn = fieldnames(o.tb);
for k = 1:numel(fn), tb.(fn{k}) = o.tb.(fn{k}); end
P.gnc.tb = tb;
P.gnc.fsm.auto_start = logical(o.AutoStart);
if o.DeckDown, P.gnc.fsm.deck_t0 = P.gnc.fsm.t_deck; end
obj = gnc_designMPC(P, 'Assign', false);

m = 'tb_gnc_mission';
if ~bdIsLoaded(m), load_system(m); end
in = Simulink.SimulationInput(m);
in = in.setVariable('P', P);
in = in.setVariable('VC_ROW_TRACKER', o.Tracker);
in = in.setVariable('VC_TEB', o.Teb);
in = in.setVariable('mpcRowObj', obj);
in = in.setModelParameter('StopTime', num2str(o.StopTime));
t0 = tic;
out = sim(in);
r.wall = toc(t0);
y = out.yout;
tr = y.get('truth').Values;
r.t = tr.Time;  r.truth = local_rows(tr.Data, numel(r.t));
cmd = y.get('gnc_cmd').Values;
r.tc = cmd.v_cmd.Time;
for f = {'v_cmd', 'omega_cmd', 'cmd_valid', 'seq', 'gnc_ready', 'mode_req', 'gnc_state', 'mower_enable', ...
        'mower_speed_cmd', 'tiller_enable', 'tiller_depth_cmd', 'spray_enable', 'spray_section_mask', 'spray_flow_cmd', ...
        'soft_stop_req', 'beacon_cmd', 'deck_height_cmd'}
    r.(f{1}) = double(squeeze(cmd.(f{1}).Data));
end
r.cmd_finite = all(isfinite(double(squeeze(cmd.v_cmd.Data)))) && all(isfinite(double(squeeze(cmd.omega_cmd.Data)))) && ...
    all(isfinite(double(squeeze(cmd.spray_flow_cmd.Data)))) && all(isfinite(double(squeeze(cmd.mower_speed_cmd.Data))));
dbg = y.get('gnc_debug').Values;
r.v_gov = double(squeeze(dbg.v_gov.Data));
r.d_obs = double(squeeze(dbg.d_obs.Data));
r.vfh = double(squeeze(dbg.vfh_steer.Data));
r.v_limits = local_rows(double(dbg.v_limits.Data), numel(r.tc));
r.ver = double(squeeze(dbg.window_ver.Data));
pr = local_rows(dbg.pose_ref.Data, numel(r.tc));
r.pose_ref = pr;
st = y.get('gnc_status').Values;
r.ts = st.seg_idx.Time;
for f = {'gnc_state', 'sub_mode', 'seg_idx', 'progress', 'xte', 'xte_rms', 'dist_home', 'soc_rth', 'fault_word', ...
        'hold_reason', 'pause_reason', 'replan_count', 'tracker_active', 'eta_s', 'mission_done', 'area_done_m2'}
    r.(['s_' f{1}]) = double(squeeze(st.(f{1}).Data));
end
sh = y.get('shaped').Values;
r.shaped = local_rows(sh.Data, numel(sh.Time));
nc = y.get('nav_cmd').Values;
r.nav_mode = double(squeeze(nc.navMode.Data));
pv = y.get('prot').Values;
r.tp = pv.Time;  r.prot = double(squeeze(pv.Data));
% signals logged inside Shaddad_GNC
ls = out.logsout;
fsm = ls.getElement('fsm').Values;
r.tf = fsm.state.Time;
for f = {'state', 'sub', 'seg_idx', 'nxt_idx', 'path_src', 'trk_sel', 'mode_req', 'cmd_valid', 'gnc_ready', ...
        'impl_allow', 'replan_req', 'hold_reason', 'pause_reason', 'mission_done', 'teb_enable', 'v_cap', 's_obs', ...
        't_hold', 'fault_word', 'trans_id'}
    r.(['f_' f{1}]) = double(squeeze(fsm.(f{1}).Data));
end
ob = ls.getElement('obs').Values;
r.to = ob.d_obs.Time;
r.o_d_obs = double(squeeze(ob.d_obs.Data));
r.o_blocked = double(squeeze(ob.blocked_vfh.Data));
r.o_person = double(squeeze(ob.person_near.Data));
rp = ls.getElement('rp').Values;
r.tr = rp.ok.Time;
r.rp_ok = double(squeeze(rp.ok.Data));
r.rp_ver = double(squeeze(rp.ver.Data));
r.rp_fail = double(squeeze(rp.fail_count.Data));
r.rp_tplan = double(squeeze(rp.t_plan.Data));
r.rp_path = rp.path.Data;
r.logsout = ls;
r.P = P;
r.plan = plan;
end

function M = local_rows(D, n)
M = squeeze(D);
if size(M, 1) ~= n, M = M'; end
end
