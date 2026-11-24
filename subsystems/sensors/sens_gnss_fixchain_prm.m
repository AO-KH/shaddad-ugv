function c = sens_gnss_fixchain_prm(G)
%SENS_GNSS_FIXCHAIN_PRM  Pack the fix-chain parameters of P.sensors.gnss into the numeric vector that the
%   MATLAB Function block FixChain (sens_gnss_model) receives on its input 7.
%
%   c = sens_gnss_fixchain_prm(P.sensors.gnss)          c: 1 x 18 double
%
%   The block takes its constants on an input (Constant block 'ChainPrm' = this expression), not as masked
%   parameter data: in R2026b a MATLAB Function parameter two Subsystem Reference levels below a model
%   (Shaddad_System -> Shaddad_SensorModels -> sens_gnss_model) fails the first compile after a rebuild with
%   'Stateflow:Assertion o != nullptr' (cdr_get_param_props_for_jit_registration).
%   Index  field                      Index  field
%     1    dt                           10   markov.l_sg_fl
%     2    initFix                      11   markov.l_nofix
%     3    corrAgeFixLimit              12   markov.svf_nofix
%     4    corrAgeFloatLimit            13   markov.l_nf_sg0
%     5    pOutlier                     14   markov.l_nf_sg1
%     6    markov.l_fx_fl0              15   markov.h_lose0
%     7    markov.l_fx_fl1              16   markov.h_lose1
%     8    markov.l_fl_fx               17   markov.h_gain
%     9    markov.l_fl_sg               18   svfOutlier
%   sens_gnss_fixchain unpacks the same order.

m = G.markov;
c = [G.dt, G.initFix, G.corrAgeFixLimit, G.corrAgeFloatLimit, G.pOutlier, ...
     m.l_fx_fl0, m.l_fx_fl1, m.l_fl_fx, m.l_fl_sg, m.l_sg_fl, m.l_nofix, m.svf_nofix, m.l_nf_sg0, m.l_nf_sg1, ...
     m.h_lose0, m.h_lose1, m.h_gain, G.svfOutlier];
c = double(c);
end
