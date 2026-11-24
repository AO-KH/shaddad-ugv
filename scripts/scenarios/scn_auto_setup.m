function [P, plan, info, mpc, tab] = scn_auto_setup(~, mission, varargin)
%SCN_AUTO_SETUP  Common set-up of the autonomous (Phase 2) scenarios S3, S4, S5, S7 (T16).
%
%   [P, plan, info, mpc, tab] = scn_auto_setup(P, mission, 'World', w, 'Implement', i, 't_arm', ta, 't_start', ts)
%     mission  JSON name in scripts/missions or a mission struct (gnc_compileMission)
%   * P = shaddad_params('World', w, 'Implement', i, 'Phase', 2, 'Assign', false) (base untouched)
%   * plan = gnc_compileMission(P, mission); P.gnc.mission = plan (Constant MissionSource, D57)
%   * truth start pose P.world.start = plan.home (the robot starts at the mission home pose)
%   * mpc = gnc_designMPC(P, 'Assign', false) (passed with setVariable('mpcRowObj', ...), T14 note)
%   * tab: ScenarioIn tables - transmitter on at 0.5 s, mode switch AUTO (3), speed knob 100 %, arm edge
%     at t_arm (MCU STANDBY -> AUTO_START -> AUTO once comp_ok and gnc_ready, D20), mission command START
%     (MissionCmd 2, cmd_seq 1, mission_id = plan.mission_id) at t_start (reaches the FSM 0.2 s later).
%     Implement switch at sw_impl (default 1 OFF): the MCU runs the rotor only at >= 2 (electronics 2.7.12).

ip = inputParser;
ip.addParameter('World', 3);
ip.addParameter('Implement', 0);
ip.addParameter('t_arm', 10);
ip.addParameter('t_start', 15);            % after MCU AUTO_START -> AUTO (auto_start_warn_t 3 s): drive_ready
ip.addParameter('sw_impl', 1);             % implement switch position: 1 OFF, 2 ON, 3 ON + auto-spray
ip.addParameter('Validate', true);
ip.addParameter('WorldEdit', []);          % function handle W = f(W) applied before compiling (S5 box)
ip.parse(varargin{:});
o = ip.Results;
P = shaddad_params('World', o.World, 'Implement', o.Implement, 'Phase', 2, 'Assign', false);
P = scn_integration_overrides(P);
if ~isempty(o.WorldEdit)
    P.world = o.WorldEdit(P.world);
end
[plan, info] = gnc_compileMission(P, mission, 'World', P.world, 'Validate', o.Validate);
P.gnc.mission = plan;
P.world.start = double(plan.home(:)');
P.structure.init = structure_initial_state(P);     % settled initial chassis state at the new start pose
mpc = [];
if exist('gnc_designMPC', 'file')
    mpc = gnc_designMPC(P, 'Assign', false);
end
tab = struct();
tab.op.tx_power_on = [0 0; 0.5 1];
tab.op.knob_speed  = [0 1];
tab.op.sw_mode     = [0 3];
tab.op.sw_impl     = [0 o.sw_impl];
tab.op.sw_lift     = [0 2];
tab.op.sw_mission  = [0 1];
tab.op.sw_arm      = [0 0; o.t_arm 1];
tab.mcmd.cmd        = [0 0; o.t_start 2];
tab.mcmd.cmd_seq    = [0 0; o.t_start 1];
tab.mcmd.mission_id = [0 double(plan.mission_id)];
tab.mcmd.end_action = [0 double(plan.end_action)];
end
