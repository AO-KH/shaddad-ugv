# --- front-matter:toml ---
model = "Shaddad_BMS.slx"
component = "Shaddad_BMS/BMS_Core/ModeManager/BMS_ModeManager"
[inputs]
HV = "HV_Enable_Req"
CHG = "Charge_Enable_Req"
RST = "Fault_Reset_Req"
SD = "Shutdown_Req"
SOK = "Safety_OK"
DOCK = "Dock_Present"
FC = "FaultCritical"
Vpack = "V_pack"
Vbus = "V_bus"
Ipack = "I_pack"
CC = "ChargeComplete"
[outputs]
K1 = "K1_Cmd"
K2 = "K2_Cmd"
State = "Contactor_State"
# --- end front-matter ---

Feature: T-PWR-03 precharge in the BMS mode manager
  Requirements T-PWR-03.h, T-PWR-03.b and T-PWR-03.c of docs/requirements/power.yaml.
  The component is the MATLAB Function block BMS_ModeManager, the body of subsystems/power/bms_mode_manager.m.
  It runs every 0.01 s (Ts.bms_fast) with prm = P.power.bms.chart, so t_init 0.1 s, t_pre_min 0.25 s,
  t_pre_max 1.5 s, k_pre 0.97 and t_overlap 0.05 s.
  Contactor_State codes are 0 INIT, 1 STANDBY, 2 PRECHARGE, 3 CLOSE_MAIN, 4 HV_ON and 7 PRE_FAIL.
  Its HV_Enable_Req input is the request as the BMS sees it, after the MCU command timeout gate.
  The outputs change only at the 0.01 s samples and hold in between. Each assessment holds at every
  instant of its window, edges included, so a window edge sits at the requirement limit itself.
  The dialect has no tags, so every scenario title starts with its requirement ID.
  The re-arm rule of T-PWR-03.h needs Safety_OK to fall and rise again, which const, step and ramp
  inputs cannot express; t_pack_bms/mode_manager_table covers it.
  Run notes are in docs/requirements/power_coverage.md.

Scenario: T-PWR-03.h-1 a valid request enters PRECHARGE with K2 only
  Requirement T-PWR-03.h. Request at 1.0 s with Safety_OK true, no dock and the bus discharged.
  PRECHARGE, with K2 and without K1, is due within 0.01 s.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(0)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * h1_standby_before: State == 1 when t == [0.11s .. 0.995s]
    * h1_no_k2_before: K2 == 0 when t <= 0.995s
    * h1_precharge: State == 2 when t >= 1.01s
    * h1_k2_on: K2 == 1 when t >= 1.01s
    * h1_no_k1: K1 == 0

Scenario: T-PWR-03.h-2 no PRECHARGE while Dock_Present
  Requirement T-PWR-03.h. Request from 1.0 s while docked; the dock contact opens at 1.5 s,
  after which PRECHARGE is due within 0.01 s.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = step(1 -> 0 @ 1.5s)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(0)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * h2_standby_docked: State == 1 when t == [0.11s .. 1.495s]
    * h2_no_k2_docked: K2 == 0 when t <= 1.495s
    * h2_precharge_undocked: State == 2 when t >= 1.51s
    * h2_k2_undocked: K2 == 1 when t >= 1.51s
    * h2_no_k1: K1 == 0

Scenario: T-PWR-03.h-3 no PRECHARGE without Safety_OK
  Requirement T-PWR-03.h. Request from 1.0 s while Safety_OK is false; Safety_OK comes up at 1.5 s,
  after which PRECHARGE is due within 0.01 s.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = step(0 -> 1 @ 1.5s)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(0)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * h3_standby_unsafe: State == 1 when t == [0.11s .. 1.495s]
    * h3_no_k2_unsafe: K2 == 0 when t <= 1.495s
    * h3_precharge_safe: State == 2 when t >= 1.51s
    * h3_k2_safe: K2 == 1 when t >= 1.51s
    * h3_no_k1: K1 == 0

Scenario: T-PWR-03.h-4 an HV_Enable_Req drop aborts PRECHARGE with K2 open
  Requirement T-PWR-03.h. Request held from 0 s, so PRECHARGE starts at 0.11 s; the request drops at 1.5 s,
  before PRE_FAIL would come at 1.61 s. STANDBY with K2 open is due within 0.01 s.
  Given inputs
    * HV = step(1 -> 0 @ 1.5s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(0)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * h4_precharge: State == 2 when t == [0.12s .. 1.495s]
    * h4_k2_on: K2 == 1 when t == [0.12s .. 1.495s]
    * h4_standby: State == 1 when t >= 1.51s
    * h4_k2_open: K2 == 0 when t >= 1.51s
    * h4_no_k1: K1 == 0

Scenario: T-PWR-03.h-5 a Safety_OK drop aborts PRECHARGE with K2 open
  Requirement T-PWR-03.h. Request held from 0 s, so PRECHARGE starts at 0.11 s; Safety_OK drops at 1.5 s
  while the request stays true. STANDBY with K2 open is due within 0.01 s.
  Given inputs
    * HV = const(1)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = step(1 -> 0 @ 1.5s)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(0)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * h5_precharge: State == 2 when t == [0.12s .. 1.495s]
    * h5_k2_on: K2 == 1 when t == [0.12s .. 1.495s]
    * h5_standby: State == 1 when t >= 1.51s
    * h5_k2_open: K2 == 0 when t >= 1.51s
    * h5_no_k1: K1 == 0

Scenario: T-PWR-03.b-1 K1 waits for t_pre_min when the bus is already charged
  Requirement T-PWR-03.b. V_bus is 52.2 V, 0.978 V_pack, from the start; the request comes at 1.0 s,
  so K1 must stay off before 1.25 s and come at the 1.25 s sample.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(52.2)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * b1_no_k1_early: K1 == 0 when t <= 1.245s
    * b1_precharge_held: State == 2 when t == [1.01s .. 1.245s]
    * b1_k1_after: K1 == 1 when t >= 1.25s

Scenario: T-PWR-03.b-2 no K1 while V_bus is below 0.97 V_pack
  Requirement T-PWR-03.b. V_bus is 51.5 V, 0.964 V_pack; the request comes at 1.0 s and the run ends
  before PRE_FAIL at 2.5 s.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(51.5)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2400ms in Normal mode
  Then outputs
    * b2_no_k1: K1 == 0
    * b2_precharge_held: State == 2 when t >= 1.01s

Scenario: T-PWR-03.c-1 K2 opens t_overlap after the K1 command
  Requirement T-PWR-03.c, on the commands. Same inputs as T-PWR-03.b-1, so K1 rises at the 1.25 s sample.
  K2 must fall 0.050 +/- 0.010 s later, at the 1.29, 1.30 or 1.31 s sample.
  Given inputs
    * HV = step(0 -> 1 @ 1s)
    * CHG = const(0)
    * RST = const(0)
    * SD = const(0)
    * SOK = const(1)
    * DOCK = const(0)
    * FC = const(0)
    * Vpack = const(53.4)
    * Vbus = const(52.2)
    * Ipack = const(0)
    * CC = const(0)
  When simulate for 2s in Normal mode
  Then outputs
    * c1_k1_off_before: K1 == 0 when t <= 1.245s
    * c1_k1_on: K1 == 1 when t >= 1.25s
    * c1_k2_overlap: K2 == 1 when t == [1.25s .. 1.285s]
    * c1_k2_open: K2 == 0 when t >= 1.31s
    * c1_hv_on: State == 4 when t >= 1.31s
