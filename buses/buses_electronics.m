function d = buses_electronics()
%BUSES_ELECTRONICS  RC/SBUS, CAN frame, STM32 firmware root I/O, electronics-plant buses
%   (owner: electronics; RC part owned by electronics as well).
%   Nonvirtual buses crossing a Model-block boundary are SINGLE-RATE (one bus per task rate).
%   CAN payloads are little-endian Byte Pack / Byte Unpack images (see docs/icd.md section 4).
E = @bus_el;
% ---------------- RC / SBUS ----------------
d = bus_def('Bus_SbusStream', 0.001, 'RC_Link_Sim/UART_Line_Sim (SIM) | STM32 UART/USART Read USART2 (HW)', ...
  'Shaddad_MCU/Task_1k/SBUS_Decoder', 'Bytes received on the SBUS UART during the last 1 ms tick (mirrors UART Read).', {
  E('bytes',   'uint8',  32,'','Received bytes, valid 1..len, rest 0')
  E('len',     'uint8',  1, '','Number of valid bytes this tick',0,32)
  E('uart_err','boolean',1, '','Parity / framing error in this chunk')
  });
d(end+1) = bus_def('Bus_RcFrame', 0.014, 'RC_Link_Sim/RC_Receiver', 'RC_Link_Sim/SBUS_Encoder, logging', ...
  'Receiver output frame (period P.rc.frame_period).', {
  E('ch_raw',     'uint16', 16,'','Raw SBUS channel values (172..1811, mid 992)',0,2047)
  E('ch17',       'boolean',1, '','Digital channel 17')
  E('ch18',       'boolean',1, '','Digital channel 18')
  E('frame_lost', 'boolean',1, '','Flags bit2: RF frame lost')
  E('failsafe',   'boolean',1, '','Flags bit3: receiver failsafe active')
  E('sent',       'boolean',1, '','A frame is emitted in this period (false in NO_PULSES failsafe / line silent)')
  });
% ---------------- CAN frames ----------------
d(end+1) = bus_def('Bus_CanFrame8', -1, 'CAN codec subsystems (Byte Pack)', 'CAN codec subsystems (Byte Unpack)', ...
  'One classic CAN 2.0B frame slot (11-bit ID). valid = frame transmitted in this sample.', {
  E('id',   'uint32',1,'','11-bit identifier',0,2047)
  E('len',  'uint8', 1,'','DLC',0,8)
  E('data', 'uint8', 8,'','Payload bytes (little-endian fields)')
  E('valid','boolean',1,'','Frame present in this sample')
  });
d(end+1) = bus_def('Bus_CanFrame64', -1, 'CAN-FD codec subsystems (Byte Pack)', 'CAN-FD codec subsystems (Byte Unpack)', ...
  'One CAN-FD frame slot (11-bit ID, BRS, up to 64 bytes).', {
  E('id',   'uint32',1, '','11-bit identifier',0,2047)
  E('len',  'uint8', 1, '','Payload length (8,12,16,20,24,32,48,64)',0,64)
  E('data', 'uint8', 64,'','Payload bytes, zero padded')
  E('valid','boolean',1,'','Frame present in this sample')
  });
d(end+1) = bus_def('Bus_Can1_McuFast', 0.01, 'Shaddad_MCU/Task_10 (drive_can_pack)', 'CAN1_Net_Sim -> drive_system (drive_can_unpack)', ...
  'CAN1 frames MCU -> traction ESCs, 100 Hz.', {
  E('drv_cmd_L','Bus: Bus_CanFrame8',1,'','0x201 DRV_CMD_L')
  E('drv_cmd_R','Bus: Bus_CanFrame8',1,'','0x202 DRV_CMD_R')
  });
d(end+1) = bus_def('Bus_Can1_McuSlow', 0.05, 'Shaddad_MCU/Task_50', 'CAN1_Net_Sim -> Power_Plant/Implement ECU (impl_cmd), Shaddad_BMS (bms_req)', ...
  'CAN1 frames MCU -> implement ECU and BMS, 20 Hz.', {
  E('impl_cmd','Bus: Bus_CanFrame8',1,'','0x140 IMPL_CMD')
  E('bms_req', 'Bus: Bus_CanFrame8',1,'','0x3D0 BMS_REQ')
  });
d(end+1) = bus_def('Bus_Can1_EscFast', 0.01, 'drive_system (drive_can_pack)', 'CAN1_Net_Sim -> Shaddad_MCU', ...
  'CAN1 frames ESCs -> MCU, 100 Hz.', {
  E('fb1_L','Bus: Bus_CanFrame8',1,'','0x181 DRV_FB1_L')
  E('fb1_R','Bus: Bus_CanFrame8',1,'','0x182 DRV_FB1_R')
  E('fb3_L','Bus: Bus_CanFrame8',1,'','0x381 DRV_FB3_L')
  E('fb3_R','Bus: Bus_CanFrame8',1,'','0x382 DRV_FB3_R')
  });
d(end+1) = bus_def('Bus_Can1_EscSlow', 0.05, 'drive_system (drive_can_pack)', 'CAN1_Net_Sim -> Shaddad_MCU', ...
  'CAN1 frames ESCs -> MCU, 20 Hz.', {
  E('fb2_L','Bus: Bus_CanFrame8',1,'','0x281 DRV_FB2_L')
  E('fb2_R','Bus: Bus_CanFrame8',1,'','0x282 DRV_FB2_R')
  });
d(end+1) = bus_def('Bus_Can1_Impl', 0.05, 'Power_Plant/Implement_Loads (implement ECU codec)', 'CAN1_Net_Sim -> Shaddad_MCU', ...
  'CAN1 frame implement ECU -> MCU, 20 Hz.', {
  E('impl_fb','Bus: Bus_CanFrame8',1,'','0x141 IMPL_FB')
  });
d(end+1) = bus_def('Bus_Can1_Bms', 0.1, 'Shaddad_BMS (IO_Out SIM, BMS CAN codec)', 'CAN1_Net_Sim -> Shaddad_MCU', ...
  'CAN1 frames BMS -> MCU, 10 Hz (Victron/Pylontech-style + Shaddad extensions).', {
  E('limits','Bus: Bus_CanFrame8',1,'','0x351 BMS_LIMITS')
  E('soc',   'Bus: Bus_CanFrame8',1,'','0x355 BMS_SOC')
  E('vit',   'Bus: Bus_CanFrame8',1,'','0x356 BMS_VIT')
  E('alarm', 'Bus: Bus_CanFrame8',1,'','0x35A BMS_ALARM')
  E('cells', 'Bus: Bus_CanFrame8',1,'','0x373 BMS_CELLS')
  E('state', 'Bus: Bus_CanFrame8',1,'','0x3C0 BMS_STATE')
  E('power', 'Bus: Bus_CanFrame8',1,'','0x3C1 BMS_POWER')
  });
% ---------------- Shaddad_MCU root I/O (one bus per rate) ----------------
d(end+1) = bus_def('Bus_McuIn1k', 0.001, 'Shaddad_System/MCU/MCU_In (RT + Signal Specification)', 'Shaddad_MCU IO_In', ...
  'MCU inputs at the 1 kHz base task.', {
  E('sbus',        'Bus: Bus_SbusStream',1,'','SBUS UART chunk')
  E('di_word',     'uint16',1,'','Digital inputs (docs/icd.md DI map)')
  E('t_us',        'uint32',1,'us','Free-running microsecond timestamp (wraps 71.6 min)')
  E('reset_flag',  'boolean',1,'','Watchdog reset seen (por pulse) - forces BOOT')
  E('fault_inject','uint32',1,'','SIM only: b0 stall 10 ms task, b1 stall 50 ms, b2 stall 100 ms, b3 force overrun, b4 CAN1 bus-off, b5 CAN2 bus-off, b6 IMU invalid')
  });
d(end+1) = bus_def('Bus_McuIn5', 0.005, 'Shaddad_System/MCU/MCU_In', 'Shaddad_MCU IO_In', 'MCU inputs at 200 Hz.', {
  E('imu','Bus: Bus_ImuRaw',1,'','ICM-20948 outputs (SIM: Shaddad_SensorModels)')
  });
d(end+1) = bus_def('Bus_McuIn10', 0.01, 'Shaddad_System/MCU/MCU_In', 'Shaddad_MCU IO_In', 'MCU inputs at 100 Hz.', {
  E('can1_esc','Bus: Bus_Can1_EscFast',1,'','ESC FB1/FB3 frames')
  });
d(end+1) = bus_def('Bus_McuIn50', 0.05, 'Shaddad_System/MCU/MCU_In', 'Shaddad_MCU IO_In', 'MCU inputs at 20 Hz.', {
  E('can1_escslow', 'Bus: Bus_Can1_EscSlow',1,'','ESC FB2 frames')
  E('can1_impl',    'Bus: Bus_Can1_Impl',   1,'','Implement ECU feedback frame')
  E('can2_comp',    'Bus: Bus_CanFrame64',  1,'','0x100 COMP_CMD from the companion')
  E('adc_raw',      'uint16',8,'','ADC1 counts [Vpack Vbus Ipack V24 V12 V5 NTC_encl LiftPot]')
  E('mcu_temp_degC','single',1,'degC','MCU die temperature (ADC3)')
  });
d(end+1) = bus_def('Bus_McuIn100', 0.1, 'Shaddad_System/MCU/MCU_In', 'Shaddad_MCU IO_In', 'MCU inputs at 10 Hz.', {
  E('can1_bms','Bus: Bus_Can1_Bms',1,'','BMS frames')
  });
d(end+1) = bus_def('Bus_McuOut1k', 0.001, 'Shaddad_MCU IO_Out', 'Electronics_Plant (Actuator_Outputs_Sim, Watchdog_Reset_Sim)', ...
  'MCU outputs at 1 kHz.', {
  E('do_word', 'uint32', 1,'','Digital outputs (docs/icd.md DO map)')
  E('wdg_kick','boolean',1,'','IWDG1 refresh this tick')
  });
d(end+1) = bus_def('Bus_McuOut5', 0.005, 'Shaddad_MCU IO_Out', 'Link_MCU_GNC', 'MCU outputs at 200 Hz.', {
  E('can2_imu','Bus: Bus_CanFrame64',1,'','0x201 MCU_IMU')
  });
d(end+1) = bus_def('Bus_McuOut10', 0.01, 'Shaddad_MCU IO_Out', 'CAN1_Net_Sim, Link_MCU_GNC, Electronics_Plant (PWM)', 'MCU outputs at 100 Hz.', {
  E('can1_drv', 'Bus: Bus_Can1_McuFast',1,'','Drive command frames')
  E('can2_odom','Bus: Bus_CanFrame64',  1,'','0x202 MCU_ODOM')
  E('pwm_duty', 'single',4,'1','PWM duty 0..1 [pump TIM1CH1, lift TIM1CH2, fan1 TIM4CH1, fan2 TIM4CH2]',0,1)
  });
d(end+1) = bus_def('Bus_McuOut50', 0.05, 'Shaddad_MCU IO_Out', 'CAN1_Net_Sim, Link_MCU_GNC', 'MCU outputs at 20 Hz.', {
  E('can1_slow',  'Bus: Bus_Can1_McuSlow',1,'','IMPL_CMD, BMS_REQ frames')
  E('can2_status','Bus: Bus_CanFrame64',  1,'','0x200 MCU_STATUS')
  });
d(end+1) = bus_def('Bus_McuOut100', 0.1, 'Shaddad_MCU IO_Out', 'Link_MCU_GNC', 'MCU outputs at 10 Hz.', {
  E('can2_power','Bus: Bus_CanFrame64',1,'','0x203 MCU_POWER (BMS forward)')
  });
d(end+1) = bus_def('Bus_FwTelemetry', 0.01, 'Shaddad_MCU/Task_10/Telemetry', 'logging, Sim_Monitors, Viz_Logging', ...
  'Firmware internals for logging (single).', {
  E('mode',          'uint8', 1, '',     'ShaddadMode')
  E('cmd_src',       'uint8', 1, '',     'ShaddadCmdSrc: 0 NONE 1 RC 2 AUTO')
  E('fault_word',    'uint32',1, '',     'Supervisor fault word (docs/icd.md)')
  E('warn_word',     'uint32',1, '',     'Supervisor warning word')
  E('rc_ch_norm',    'single',16,'1',    'Normalised RC channels',-1,1)
  E('rc_link_ok',    'boolean',1,'',     '')
  E('rc_failsafe',   'boolean',1,'',     '')
  E('rc_frame_lost', 'boolean',1,'',     '')
  E('rc_lq',         'single',1, '1',    'Link quality',0,1)
  E('rc_age',        'single',1, 's',    'Time since last good frame')
  E('v_target',      'single',1, 'm/s',  'Arbitrated target speed')
  E('w_target',      'single',1, 'rad/s','Arbitrated target yaw rate')
  E('v_ref',         'single',1, 'm/s',  'Shaped speed reference')
  E('w_ref',         'single',1, 'rad/s','Shaped yaw-rate reference')
  E('a_ref',         'single',1, 'm/s^2','')
  E('alpha_ref',     'single',1, 'rad/s^2','')
  E('w_mix',         'single',1, 'rad/s','Yaw-rate after yaw-rate PI correction')
  E('yaw_rate_meas', 'single',1, 'rad/s','Bias-corrected gyro z')
  E('vtrack_ref',    'single',2, 'm/s',  'Track speed references [L R]')
  E('vtrack_meas',   'single',2, 'm/s',  'Encoder track speeds [L R]')
  E('w_ref_motor',   'single',2, 'rad/s','ESC speed setpoints')
  E('tau_ff',        'single',2, 'N*m',  '')
  E('tau_lim_mot',   'single',2, 'N*m',  '')
  E('tau_lim_reg',   'single',2, 'N*m',  '')
  E('k_traction',    'single',2, '1',    'Traction-control factor',0,1)
  E('roll',          'single',1, 'rad',  'Complementary-filter roll')
  E('pitch',         'single',1, 'rad',  'Complementary-filter pitch (+ nose down)')
  E('V_pack',        'single',1, 'V',    '')
  E('I_pack',        'single',1, 'A',    '')
  E('SOC',           'single',1, '1',    '')
  E('T_encl_degC',   'single',1, 'degC', '')
  E('comp_age',      'single',1, 's',    'Age of last COMP_CMD')
  E('sbus_ok',       'uint32',1, '',     'Good frames')
  E('sbus_bad',      'uint32',1, '',     'Rejected frames')
  E('sbus_skip',     'uint32',1, '',     'Bytes skipped while hunting header')
  E('overruns',      'uint32',1, '',     'Task overruns')
  E('power_ready',   'boolean',1,'',     'BMS HV_Ready seen')
  E('drive_state',   'uint8', 2, '',     'EscState [L R]')
  });
d(end+1) = bus_def('Bus_McuStatus', 0.05, 'Companion_IO (decoded CAN2 0x200 MCU_STATUS)', 'Shaddad_GNC', ...
  'MCU status as seen by the companion computer.', {
  E('t_mcu',            'double',1,'s',  'MCU time (unwrapped t_ms)')
  E('mode',             'uint8', 1,'',   'ShaddadMode: 0 BOOT 1 SELFTEST 2 STANDBY 3 MANUAL 4 HOLD 5 AUTO_START 6 AUTO 7 CTRL_STOP 8 ESTOP 9 FAULT')
  E('cmd_src',          'uint8', 1,'',   '0 NONE 1 RC 2 AUTO')
  E('rc_mode_sw',       'uint8', 1,'',   'RC mode switch: 0 MANUAL, 1 HOLD, 2 AUTO',0,2)
  E('rc_aux_cmd',       'uint8', 1,'',   'MissionCmd from RC aux channels (0 NONE 2 START 3 PAUSE 6 RTH)')
  E('rc_aux_seq',       'uint8', 1,'',   '+1 on every new rc_aux_cmd edge')
  E('rc_link_ok',       'boolean',1,'',  '')
  E('rc_failsafe',      'boolean',1,'',  '')
  E('estop_active',     'boolean',1,'',  'HW or remote E-stop active')
  E('drive_ready',      'boolean',1,'',  'MCU in AUTO with both ESCs OPERATION/READY and HV ready')
  E('power_ready',      'boolean',1,'',  'BMS HV_Ready')
  E('implement_ready',  'boolean',1,'',  'Implement at speed / pressure')
  E('charging',         'boolean',1,'',  'BMS charging active')
  E('wdg_reset_seen',   'boolean',1,'',  '')
  E('implement_type',   'uint8', 1,'',   '0 none 1 mower 2 tiller 3 sprayer')
  E('heartbeat',        'uint8', 1,'',   'Rolling counter +1 per frame')
  E('fault_word',       'uint32',1,'',   'MCU supervisor fault word')
  E('warn_word',        'uint32',1,'',   'MCU supervisor warning word')
  E('drive_fault_word', 'uint32',1,'',   'ESC fault codes: L in bits 0..15, R in bits 16..31')
  E('v_track_meas',     'single',2,'m/s','Encoder track speeds [L R]')
  E('v_applied',        'single',1,'m/s','Shaped v_ref sent to the mixer')
  E('w_applied',        'single',1,'rad/s','Applied yaw-rate reference')
  E('gnc_cmd_age_ms',   'uint16',1,'ms', 'Age of last valid COMP_CMD at the MCU')
  E('rc_lq',            'single',1,'1',  'RC link quality',0,1)
  E('T_encl',           'single',1,'K',  'Electronics enclosure temperature')
  E('valid',            'boolean',1,'',  'Frame fresh at the companion (< 0.2 s)')
  });
% ---------------- Electronics plant ----------------
d(end+1) = bus_def('Bus_ElecHwIn', 0.001, 'Electronics_Plant/HW_Frontend_Sim + Watchdog_Reset_Sim', ...
  'Shaddad_System/MCU/MCU_In (Bus_McuIn1k), CAN1_Net_Sim, Link_MCU_GNC (through Shaddad_System/Plant/ElecHw_Delay, sys.plant.elec_hw_d)', 'Digital front end and reset emulation.', {
  E('di_word',   'uint16', 1,'','Packed DI bits (docs/icd.md DI map)')
  E('reset_flag','boolean',1,'','Pulse (one tick) after an emulated watchdog reset')
  E('mcu_reset', 'boolean',1,'','MCU held in reset (all MCU TX frames dropped, DO forced 0)')
  });
d(end+1) = bus_def('Bus_ElecAdc', 0.05, 'Electronics_Plant/HW_Frontend_Sim', 'Shaddad_System/MCU/MCU_In (Bus_McuIn50)', ...
  'ADC front-end counts (16-bit, Vref 3.3 V).', {
  E('adc_raw',      'uint16',8,'','[Vpack Vbus Ipack V24 V12 V5 NTC_encl LiftPot]')
  E('mcu_temp_degC','single',1,'degC','MCU die temperature = T_encl + 15 K')
  });
d(end+1) = bus_def('Bus_ElecEnv', 0.1, 'Electronics_Plant (Electronics_Thermal, NTRIP link model, rail OK)', 'Shaddad_SensorModels', ...
  'Electronics-side conditions needed by the sensor models.', {
  E('T_imu_degC',  'double', 1,'degC','IMU (MCU PCB) temperature = T_encl + 5 K')
  E('T_encl_K',    'double', 1,'K',   'Enclosure air temperature')
  E('gnssCorrOK',  'boolean',1,'',    'RTK correction link up')
  E('gnssCorrAge', 'double', 1,'s',   'Age of RTK corrections',0,[])
  E('sensPowerOK', 'uint8',  1,'',    'b0 IMU, b1 GNSS, b2 LiDAR powered (rails OK and DO enables)')
  });
d(end+1) = bus_def('Bus_IndicatorOut', 0.1, 'Electronics_Plant/Actuator_Outputs_Sim', 'Viz_Logging, logging', 'Indicator and fan states.', {
  E('beacon_green','boolean',1,'','')
  E('beacon_amber','boolean',1,'','')
  E('beacon_red',  'boolean',1,'','')
  E('buzzer',      'boolean',1,'','')
  E('work_lights', 'boolean',1,'','')
  E('fan_duty',    'double', 2,'1','Enclosure fans',0,1)
  });
end
