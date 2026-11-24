function [fixType, relposFixed, outlier] = sens_gnss_fixchain(svf, u, corrOK, corrAge, outage, forceFloat, c)
%SENS_GNSS_FIXCHAIN  RTK fix-state Markov chain of the dual ZED-F9P receiver (sensors spec 3.2 / F2).
%#codegen
%   [fixType, relposFixed, outlier] = sens_gnss_fixchain(svf, u, corrOK, corrAge, outage, forceFloat, c)
%     svf        sky-view factor at the antenna [0.02, 1] (2-D Lookup Table on P.world.gnssSvf, D59)
%     u          4 uniform draws U(0,1): u(1) main transition, u(2) loss of fix (NOFIX),
%                u(3) moving-base heading (RELPOS) state, u(4) outlier
%     corrOK     RTK correction link up (Bus_ElecEnv.gnssCorrOK)
%     corrAge    age of the RTK corrections [s] (Bus_ElecEnv.gnssCorrAge)
%     outage     forced outage (Bus_FaultInject.gnss_outage): state NOFIX, re-acquired by the chain afterwards
%     forceFloat RTK fixed forced to float (Bus_FaultInject.gnss_force_float)
%     c          sens_gnss_fixchain_prm(P.sensors.gnss): 1 x 18 packed constants (block input, not parameter
%                data, see sens_gnss_fixchain_prm): dt, initFix, corrAge limits, pOutlier, markov.*, svfOutlier
%     fixType    NMEA GGA quality code: 0 NOFIX, 1 SINGLE, 4 RTK_FIXED, 5 RTK_FLOAT (uint8)
%     relposFixed moving-base RELPOS heading valid (requires fixType >= 1)
%     outlier    multipath outlier in this epoch (FLOAT / SINGLE with svf < svfOutlier, probability pOutlier)
%
%   Per step dt the transition probability of a rate lambda is p = 1 - exp(-lambda dt):
%     FIXED  -> FLOAT : 0.002 + 0.50 (1-svf)^2, forced when corrAge > corrAgeFixLimit or forceFloat
%     FLOAT  -> FIXED : 0.20 svf^3, only when corrOK, corrAge < corrAgeFixLimit and not forceFloat
%     FLOAT  -> SINGLE: 0.30 (1-svf)^3, forced when corrAge > corrAgeFloatLimit
%     SINGLE -> FLOAT : 0.50 svf^2, only when corrOK
%     any    -> NOFIX : 2.0 max(0, 0.25 - svf)
%     NOFIX  -> SINGLE: 0.5 svf + 0.05
%     RELPOS heading  : lose 0.002 + 0.30 (1-svf)^2, gain 0.50 svf^2
%   State machine with persistent state (Stateflow not installed, D1); ordered transition table per state.

dt = c(1);  corrAgeFixLimit = c(3);  corrAgeFloatLimit = c(4);  pOutlier = c(5);
l_fx_fl0 = c(6);  l_fx_fl1 = c(7);  l_fl_fx = c(8);  l_fl_sg = c(9);  l_sg_fl = c(10);
l_nofix = c(11);  svf_nofix = c(12);  l_nf_sg0 = c(13);  l_nf_sg1 = c(14);
h_lose0 = c(15);  h_lose1 = c(16);  h_gain = c(17);  svfOutlier = c(18);

persistent s h
if isempty(s)
    s = uint8(c(2));
    h = false;
end
sv = min(max(svf, 0), 1);
lamNF = l_nofix*max(0, svf_nofix - sv);

if outage
    s = uint8(0);
    h = false;
else
    switch s
        case uint8(4)                                   % RTK_FIXED
            if corrAge > corrAgeFixLimit || forceFloat || u(1) < local_p(l_fx_fl0 + l_fx_fl1*(1 - sv)^2, dt)
                s = uint8(5);
            elseif u(2) < local_p(lamNF, dt)
                s = uint8(0);
            end
        case uint8(5)                                   % RTK_FLOAT
            if corrOK && corrAge < corrAgeFixLimit && ~forceFloat && u(1) < local_p(l_fl_fx*sv^3, dt)
                s = uint8(4);
            elseif corrAge > corrAgeFloatLimit || u(1) > 1 - local_p(l_fl_sg*(1 - sv)^3, dt)
                s = uint8(1);
            elseif u(2) < local_p(lamNF, dt)
                s = uint8(0);
            end
        case uint8(1)                                   % SINGLE
            if corrOK && corrAge < corrAgeFloatLimit && u(1) < local_p(l_sg_fl*sv^2, dt)
                s = uint8(5);
            elseif u(2) < local_p(lamNF, dt)
                s = uint8(0);
            end
        otherwise                                       % NOFIX
            if u(1) < local_p(l_nf_sg1*sv + l_nf_sg0, dt)
                s = uint8(1);
            end
    end
    % moving-base RELPOS heading state
    if h
        h = ~(u(3) < local_p(h_lose0 + h_lose1*(1 - sv)^2, dt));
    else
        h = u(3) < local_p(h_gain*sv^2, dt);
    end
end
fixType = s;
relposFixed = h && s >= uint8(1);
outlier = (s == uint8(5) || s == uint8(1)) && sv < svfOutlier && u(4) < pOutlier && ~outage;
end

function p = local_p(lam, dt)
p = 1 - exp(-lam*dt);
end
