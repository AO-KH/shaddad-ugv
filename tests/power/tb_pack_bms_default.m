function TB = tb_pack_bms_default()
%TB_PACK_BMS_DEFAULT  Default stimulus struct of the bench tests/power/tb_pack_bms.slx (owner: power, T04).
%
%   TB = tb_pack_bms_default()   stored in the bench model workspace by build_tb_pack_bms; tests pass
%   a modified copy with Simulink.SimulationInput.setVariable('TB', TB, 'Workspace', 'tb_pack_bms').
%   Tables are [t value] breakpoints (strictly increasing t, s), held between breakpoints (Flat).
%
%   load_t / load_I   bus load current profile (A, + = discharge), gated to 0 below V_bus 30..35 V
%   hv_* chg_* rst_* sd_*   MCU BMS_REQ flags HV_Enable_Req / Charge_Enable_Req / Fault_Reset_Req /
%                           Shutdown_Req (sampled at 0.05 s, sent through bmsreq_can_pack)
%   alive_freeze_t    time after which the BMS_REQ alive counter is frozen (MCU command timeout test)
%   dock_*            dock contact (Dock_Present, Pack_Thermal dock air)
%   shv_* / sok_*     Safety_HV_OK / Safety_OK (safety-relay contacts; coil supply of K1 / K2)
%   T_amb (K), G_solar (W/m^2)   ambient conditions
%   chg_force, chg_force_I       dock charger ignores the BMS request (T-PWR-08)
%   flt_*             Bus_FaultInject.bms_fault (+0.5 V on V_cell(1))
%   fan_mode          0 BMS Fan_Cmd, 1 fans forced off, 2 forced on
%   dec_fast / dec_cmd   decimation of the 1 ms / 10 ms logs (To Workspace)
%   load_is_power     1: load table in W (I = P / V_bus(k-1)); stop_Vmin / stop_SOCmin / stop_on_done
%                     end the run on the weakest true cell voltage / lowest true group SOC / BMS
%                     Charge_Complete (Stop Simulation block)
TB.load_t = [0 1e7];  TB.load_I = [0 0];
TB.hv_t   = [0 1e7];  TB.hv_v   = [0 0];
TB.chg_t  = [0 1e7];  TB.chg_v  = [0 0];
TB.rst_t  = [0 1e7];  TB.rst_v  = [0 0];
TB.sd_t   = [0 1e7];  TB.sd_v   = [0 0];
TB.alive_freeze_t = 1e9;
TB.dock_t = [0 1e7];  TB.dock_v = [0 0];
TB.shv_t  = [0 1e7];  TB.shv_v  = [1 1];
TB.sok_t  = [0 1e7];  TB.sok_v  = [1 1];
TB.T_amb = 298.15;    TB.G_solar = 0;
TB.chg_force = 0;     TB.chg_force_I = 100;
TB.flt_t  = [0 1e7];  TB.flt_v  = [0 0];
TB.fan_mode = 0;
TB.dec_fast = 1;      TB.dec_cmd = 1;
TB.V_gate = [30 35];
TB.R_esr = 10e-3;     % bus capacitor ESR (drive DC-link stub), Ohm
TB.load_is_power = 0; % 1: load table is a DC power demand (W), I = P / V_bus(k-1)
TB.stop_Vmin = 0;     % stop when the weakest true cell voltage < stop_Vmin (V); 0 = never
TB.stop_SOCmin = -1;  % stop when the lowest true group SOC < stop_SOCmin; -1 = never
TB.stop_SOCmax = 2;   % stop when the lowest true group SOC > stop_SOCmax; 2 = never
TB.stop_on_done = 0;  % 1: stop when the BMS reports Charge_Complete
end
