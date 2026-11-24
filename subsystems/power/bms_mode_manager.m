function [K1_Cmd, K2_Cmd, K4_Cmd, Charger_Enable, HV_Ready, HV_Opening_Warn, Charging_Active, ...
          Charge_Complete, Contactor_State, Fault_PrechargeFail, Fault_Weld, PrechargeRetries] = ...
          bms_mode_manager(HV_Enable_Req, Charge_Enable_Req, Fault_Reset_Req, Shutdown_Req, Safety_OK, ...
          Dock_Present, FaultCritical, V_pack, V_bus, I_pack, ChargeComplete, prm) %#codegen
%BMS_MODE_MANAGER  BMS contactor / precharge / charge mode manager (owner: power, T04).
%
%   Body of the MATLAB Function block Shaddad_BMS/BMS_Core/ModeManager (Ts = P.power.bms.chart.dt
%   = 0.01 s). Implements the transition table of docs/specs/power.md section 4.1 in table order
%   (Stateflow is not installed, D1): enum BmsContactorState, after(t) = tick counter in the state
%   (ticks = round(t/dt)), parameters prm = P.power.bms.chart (non-tunable sub-struct).
%
%   Inputs (scalar): HV_Enable_Req (already false while the MCU command is timed out),
%   Charge_Enable_Req, Fault_Reset_Req, Shutdown_Req, Safety_OK (= Safety_HV_OK: the coil supply of
%   K1/K2 through the delayed stop-category-1 contacts), Dock_Present, FaultCritical (latched
%   protection faults), ChargeComplete (boolean); V_pack, V_bus, I_pack (single, I + = discharge).
%   Outputs: K1/K2/K4 coil requests, Charger_Enable, HV_Ready, HV_Opening_Warn, Charging_Active,
%   Charge_Complete, Contactor_State (uint8 = BmsContactorState code), Fault_PrechargeFail, Fault_Weld
%   (latched until a FAULT reset), PrechargeRetries (uint8).
%
%   Table (priority = order):
%     INIT        all open; after(t_init) -> STANDBY
%     STANDBY     [FaultCritical] -> FAULT
%                 [HV_Enable_Req && Safety_OK && !Dock_Present && armed] -> PRECHARGE
%                 [Dock_Present && Charge_Enable_Req] -> CHARGING
%     PRECHARGE   K2; [!HV_Enable_Req || !Safety_OK] -> STANDBY
%                 [after(t_pre_min) && V_bus >= k_pre*V_pack] -> CLOSE_MAIN
%                 [after(t_pre_max)] {PrechargeRetries++} -> PRE_FAIL
%     PRE_FAIL    K2 = 0; [PrechargeRetries >= n_pre_retry] {Fault_PrechargeFail = 1} -> FAULT
%                 [after(t_retry)] -> STANDBY
%     CLOSE_MAIN  K1, K2; [after(t_overlap)] -> HV_ON
%     HV_ON       K1, HV_Ready; entry PrechargeRetries = 0
%                 [!Safety_OK] -> STANDBY ; [FaultCritical || !HV_Enable_Req || Shutdown_Req] -> OPENING
%     OPENING     K1, HV_Opening_Warn; [abs(I_pack) < I_open_max || after(t_open_wait)] -> WELD_CHECK
%     WELD_CHECK  K1 = 0; [after(t_weld)]: V_bus > k_weld*V_pack ? {Fault_Weld = 1} -> FAULT : -> STANDBY
%     CHARGING    K4, Charger_Enable = after(t_chg_en), Charging_Active
%                 [FaultCritical] -> FAULT ; [!Dock_Present || !Charge_Enable_Req] -> STANDBY
%                 [ChargeComplete] -> CHARGE_DONE
%     CHARGE_DONE K4 = 0, Charge_Complete; [!Dock_Present] -> STANDBY
%     FAULT       all open, latched; [rising(Fault_Reset_Req) && !FaultCritical] -> STANDBY
%   'armed' (the en latch inside STANDBY, spec 4.1 note): after a Safety_OK drop (PRECHARGE / HV_ON
%   -> STANDBY) and after a completed opening (WELD_CHECK -> STANDBY), re-entering HV needs an
%   HV_Enable_Req falling edge followed by a rising edge; PRE_FAIL retries keep it armed.

persistent st tick nRetry fPre fWeld armed rstPrev
if isempty(st)
    st = BmsContactorState.INIT;
    tick = uint32(0);        % ticks since state entry (entry tick = 0)
    nRetry = uint8(0);
    fPre = false;
    fWeld = false;
    armed = true;
    rstPrev = true;          % a reset request already high at start-up is not an edge
elseif tick < intmax('uint32')
    tick = tick + uint32(1); % after(n ticks) is true on the n-th tick after entry (Stateflow semantics)
end

nTicks = @(t) uint32(round(t / prm.dt));
rstEdge = Fault_Reset_Req && ~rstPrev;
rstPrev = Fault_Reset_Req;
if ~HV_Enable_Req
    armed = true;            % falling edge (or low level) re-arms HV entry
end

% ---- transitions (one per tick, table order) ----
nxt = st;
switch st
    case BmsContactorState.INIT
        if tick >= nTicks(prm.t_init)
            nxt = BmsContactorState.STANDBY;
        end
    case BmsContactorState.STANDBY
        if FaultCritical
            nxt = BmsContactorState.FAULT;
        elseif HV_Enable_Req && Safety_OK && ~Dock_Present && armed
            nxt = BmsContactorState.PRECHARGE;
        elseif Dock_Present && Charge_Enable_Req
            nxt = BmsContactorState.CHARGING;
        end
    case BmsContactorState.PRECHARGE
        if ~HV_Enable_Req || ~Safety_OK
            nxt = BmsContactorState.STANDBY;
            armed = ~HV_Enable_Req;
        elseif tick >= nTicks(prm.t_pre_min) && V_bus >= single(prm.k_pre) * V_pack
            nxt = BmsContactorState.CLOSE_MAIN;
        elseif tick >= nTicks(prm.t_pre_max)
            if nRetry < intmax('uint8'), nRetry = nRetry + uint8(1); end
            nxt = BmsContactorState.PRE_FAIL;
        end
    case BmsContactorState.PRE_FAIL
        if double(nRetry) >= prm.n_pre_retry
            fPre = true;
            nxt = BmsContactorState.FAULT;
        elseif tick >= nTicks(prm.t_retry)
            nxt = BmsContactorState.STANDBY;
        end
    case BmsContactorState.CLOSE_MAIN
        if tick >= nTicks(prm.t_overlap)
            nxt = BmsContactorState.HV_ON;
        end
    case BmsContactorState.HV_ON
        if ~Safety_OK
            nxt = BmsContactorState.STANDBY;
            armed = ~HV_Enable_Req;
        elseif FaultCritical || ~HV_Enable_Req || Shutdown_Req
            nxt = BmsContactorState.OPENING;
        end
    case BmsContactorState.OPENING
        if abs(I_pack) < single(prm.I_open_max) || tick >= nTicks(prm.t_open_wait)
            nxt = BmsContactorState.WELD_CHECK;
        end
    case BmsContactorState.WELD_CHECK
        if tick >= nTicks(prm.t_weld)
            if V_bus > single(prm.k_weld) * V_pack
                fWeld = true;
                nxt = BmsContactorState.FAULT;
            else
                nxt = BmsContactorState.STANDBY;
                armed = ~HV_Enable_Req;
            end
        end
    case BmsContactorState.CHARGING
        if FaultCritical
            nxt = BmsContactorState.FAULT;
        elseif ~Dock_Present || ~Charge_Enable_Req
            nxt = BmsContactorState.STANDBY;
        elseif ChargeComplete
            nxt = BmsContactorState.CHARGE_DONE;
        end
    case BmsContactorState.CHARGE_DONE
        if ~Dock_Present
            nxt = BmsContactorState.STANDBY;
        end
    case BmsContactorState.FAULT
        if rstEdge && ~FaultCritical
            nxt = BmsContactorState.STANDBY;
            fPre = false;
            fWeld = false;
            nRetry = uint8(0);
        end
    otherwise
        nxt = BmsContactorState.FAULT;
end
if nxt ~= st
    st = nxt;
    tick = uint32(0);
    if st == BmsContactorState.HV_ON
        nRetry = uint8(0);   % entry action
    end
end

% ---- state outputs (Moore) ----
K1_Cmd = st == BmsContactorState.CLOSE_MAIN || st == BmsContactorState.HV_ON || st == BmsContactorState.OPENING;
K2_Cmd = st == BmsContactorState.PRECHARGE || st == BmsContactorState.CLOSE_MAIN;
K4_Cmd = st == BmsContactorState.CHARGING;
Charger_Enable = st == BmsContactorState.CHARGING && tick >= nTicks(prm.t_chg_en);
HV_Ready = st == BmsContactorState.HV_ON;
HV_Opening_Warn = st == BmsContactorState.OPENING;
Charging_Active = st == BmsContactorState.CHARGING;
Charge_Complete = st == BmsContactorState.CHARGE_DONE;
Contactor_State = uint8(st);
Fault_PrechargeFail = fPre;
Fault_Weld = fWeld;
PrechargeRetries = nRetry;
end
