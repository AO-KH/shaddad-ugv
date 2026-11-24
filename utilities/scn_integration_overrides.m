function P = scn_integration_overrides(P)
%SCN_INTEGRATION_OVERRIDES  Parameter overrides the system scenarios need (T16; values in P.sim.ovr).
%
%   P = scn_integration_overrides(P)     called by every scripts/scenarios/scn_S<k>_*.m before the
%                                        SimulationInput is built (the base workspace is not touched)
%   Each override is a cross-domain finding of the T16 system runs, reported to the owning task:
%   * P.power.pdu.R_pre = P.sim.ovr.R_pre (4.7 Ohm instead of 22 Ohm): the ESC idle load on the DC link
%     (drive_system, about 2 x 7 W) keeps the precharged bus at 87 % of the pack voltage with 22 Ohm, so
%     the BMS precharge check (P.power.bms.chart.k_pre = 0.97) times out and HV_Ready never rises.

if isfield(P.sim, 'ovr') && isfield(P.sim.ovr, 'R_pre')
    P.power.pdu.R_pre = P.sim.ovr.R_pre;
end
end
