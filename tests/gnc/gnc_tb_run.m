function r = gnc_tb_run(P, S, varargin)
%GNC_TB_RUN  Run the GNC tracking bench tests/gnc/tb_gnc.slx on a test mission (T14 test helper).
%
%   r = gnc_tb_run(P, S, 'x0', [x y psi], 'drift', d, 'tilt', [pitch roll], 'Tracker', 1|2,
%                  'StopTime', T, 'Plan', plan)
%     P   parameter struct (shaddad_params('Assign', false)); never written to the base workspace
%     S   mission struct for gnc_compileMission (rows_enu / segments / home_enu ...), ignored with 'Plan'
%     r   struct: t (0.01 s grid) truth [x y psi v r]; tc (0.05 s) v_cmd, w_cmd, cmd_valid, seq,
%         v_gov, v_prof, w_pp, w_row, qp, ver, pose_ref; ts (0.2 s) seg_idx, trk, xte_est; shaped
%         [v_ref w_ref]; plan, info, wall time
%   The scenario reaches the bench only through Simulink.SimulationInput.setVariable (P, VC_ROW_TRACKER,
%   mpcRowObj); Ts, Bus_* and cs_SimFixed must exist in the base workspace (shaddad_params / startup).

ip = inputParser;
ip.addParameter('x0', []);
ip.addParameter('drift', 0);
ip.addParameter('tilt', [0 0]);
ip.addParameter('Tracker', 1);
ip.addParameter('StopTime', 30);
ip.addParameter('Plan', []);
ip.addParameter('soc', P.gnc.tb.soc);
ip.parse(varargin{:});
o = ip.Results;

if isempty(o.Plan)
    [plan, info] = gnc_compileMission(P, S, 'Validate', false);
else
    plan = o.Plan; info = struct();
end
P.gnc.mission = plan;
if isempty(o.x0), o.x0 = plan.home(:)'; end
P.gnc.tb.x0 = o.x0;
P.gnc.tb.drift = o.drift;
P.gnc.tb.tilt = o.tilt;
P.gnc.tb.soc = o.soc;
obj = gnc_designMPC(P, 'Assign', false);

m = 'tb_gnc';
if ~bdIsLoaded(m), load_system(m); end
in = Simulink.SimulationInput(m);
in = in.setVariable('P', P);
in = in.setVariable('VC_ROW_TRACKER', o.Tracker);
in = in.setVariable('mpcRowObj', obj);
in = in.setModelParameter('StopTime', num2str(o.StopTime));
t0 = tic;
out = sim(in);
r.wall = toc(t0);
y = out.yout;
tr = y.get('truth').Values;
r.t = tr.Time;  r.truth = squeeze(tr.Data);
if size(r.truth, 1) ~= numel(r.t), r.truth = r.truth'; end
cmd = y.get('gnc_cmd').Values;
r.tc = cmd.v_cmd.Time;
r.v_cmd = double(squeeze(cmd.v_cmd.Data));
r.w_cmd = double(squeeze(cmd.omega_cmd.Data));
r.cmd_valid = squeeze(cmd.cmd_valid.Data);
r.seq = squeeze(cmd.seq.Data);
r.gnc_ready = squeeze(cmd.gnc_ready.Data);
dbg = y.get('gnc_debug').Values;
r.v_gov = double(squeeze(dbg.v_gov.Data));
r.v_prof = double(squeeze(dbg.v_prof.Data));
r.w_pp = double(squeeze(dbg.omega_pp.Data));
r.w_row = double(squeeze(dbg.omega_row.Data));
r.qp = double(squeeze(dbg.mpc_qp_status.Data));
r.ver = double(squeeze(dbg.window_ver.Data));
pr = squeeze(dbg.pose_ref.Data);
if size(pr, 1) ~= numel(r.tc), pr = pr'; end
r.pose_ref = pr;
r.v_limits = double(squeeze(dbg.v_limits.Data));
st = y.get('gnc_status').Values;
r.ts = st.seg_idx.Time;
r.seg_idx = double(squeeze(st.seg_idx.Data));
r.trk = double(squeeze(st.tracker_active.Data));
r.xte_est = double(squeeze(st.xte.Data));
sh = y.get('shaped').Values;
r.shaped = squeeze(sh.Data);
if size(r.shaped, 1) ~= numel(sh.Time), r.shaped = r.shaped'; end
r.plan = plan;
r.info = info;
r.P = P;
end
