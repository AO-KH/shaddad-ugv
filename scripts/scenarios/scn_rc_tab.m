function tab = scn_rc_tab(t_tx, t_arm)
%SCN_RC_TAB  Common ScenarioIn operator tables of the RC-driven scenarios (S1, S2, S6, S8; T16).
%
%   tab = scn_rc_tab(t_tx, t_arm)
%     transmitter on at t_tx (Bus_OperatorCmd.tx_power_on, T07 note: must be 1 for a live link),
%     arm switch edge 0 -> 1 at t_arm with the sticks neutral, speed knob +1 (100 %), mode switch
%     MANUAL (1), implement OFF (1), lift HOLD (2), mission switch none (1).
%   Scenario scripts add the stick / fault / hmi tables (scn_make_input ZOH tables, [t value] rows).

tab = struct();
tab.op.tx_power_on = [0 0; t_tx 1];
tab.op.knob_speed  = [0 1];
tab.op.sw_mode     = [0 1];
tab.op.sw_impl     = [0 1];
tab.op.sw_lift     = [0 2];
tab.op.sw_mission  = [0 1];
tab.op.sw_arm      = [0 0; t_arm 1];
tab.op.stick_thr   = [0 0];
tab.op.stick_steer = [0 0];
end
