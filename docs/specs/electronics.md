# Shaddad UGV: Electronics, STM32 MCU Firmware and RC (SBUS) Link: Design Specification

> **Revision B note (2026-10-19).** `docs/architecture.md` section 2b (D30-D63) and `docs/build_plan.md`
> override this spec where they differ. Sections changed for this domain: task structure and frame set per architecture D2/D13/D19 (already binding in revision A); DO0 MCU_SAFETY_OK = MCU-internal health only, DO1 re-arm pulse, bit 31 evaluated only while ACTIVE (D30); stop profiles use the constant-deceleration law (D33); receiver failsafe preset CH12 = RUN, remote E-stop only from valid frames, no channel debounce (D34); RC latency budget (D37); receiver failsafe built from blocks (D59); tilt filter with linear-acceleration compensation (D47); AUTO yaw shaper non-binding and CAN2 loop breaker on MCU_STATUS (D41); elec_hw 1 ms Unit Delay (D36); companion HW via ROS 2 Subscribe/Publish CAN-FD bridge (D38); TIM2 / RCC_RSR / FDCAN_PSR register reads (D60); single owner of rc_required_in_auto (D44); codegen BLOCKED until Simulink Coder is installed (D31; resolved by D121, 2026-11-23: Simulink Coder and Embedded Coder have been installed since 2026-11-07, and the STM32 board builds wait on the STM32 tools, card B-00).
>
> **Revision C note (2026-11-18, card G1-01 part 2).** The sections outside the firmware (all but 2.7, 3.1, 3.2, 4.1, 4.3, 7.2 and 8.3) are brought in line with the parameters of 2026-11-17 (the step-3 sweep's dump of `P`) and with decisions D1 to D110 of `docs/architecture.md` (D104 to D110 change no value in these sections). Where this spec and the ICD (`docs/icd.md`) differ on a frame, a bit, a pin or a port, the ICD binds. Under D96 the STM32H743 design described here is the twin's development target: the robot's delivered controller, ESCs, BMS and charger are the baseline, provisionally (§2.2). Revision A tables that the T01 to T10 build replaced are replaced by the current values with a pointer to the ICD section that holds the details.

| Item | Value |
|---|---|
| Domain key | `electronics` (also owns `P.rc` / `subsystems/rc/`) |
| Spec version | 1.0 (design phase), revision C (re-baseline). Revision A's blocks were checked against the live R2026b session; the builders in `scripts/build/` are the binding block lists |
| Date | 2026-10-18; revision C 2026-11-18 |
| Toolchain | MATLAB R2026b (26.2), Simulink, Embedded Coder (licensed; code generation blocked until Simulink Coder is installed, D31; resolved by D121, 2026-11-23: Simulink Coder has been installed since 2026-11-07, and MinGW64 is the C compiler; the STM32 board builds wait on card B-00), STM32 Microcontroller Blockset, Communications Toolbox, Sensor Fusion/Navigation Toolbox, Simscape/Simscape Electrical, Simulink Test. Stateflow is not installed: every state machine is a MATLAB Function. `vrlib/Joystick Input` was removed in R2026b: the joystick source uses Simulation 3D Joystick with `joy_map` (ICD 8) |
| Parameter files | `params/param_electronics.m` → `P.electronics`; `params/param_rc.m` → `P.rc` |
| Bus files | `buses/*.m`: the electronics and RC buses and the enumerations of ICD 9 (§5.3) |

---

## 0. Key decisions (one-page summary)

| # | Decision | Rationale |
|---|---|---|
| D1 | **MCU = STM32H743ZIT6** on NUCLEO-H743ZI2 for development. Production uses STM32H743VIT6 on a custom carrier board. Simulink Hardware Board = **`STM32H7xx Based (Single-core)`** (CubeMX-project workflow). | This board is on the blockset's registered list. 480 MHz Cortex-M7 with DP-FPU, 2x FDCAN, 8 UARTs, 32-bit TIM2/TIM5 encoder timers, 3x 16-bit ADC, IWDG. The `stm32h7xxblockslib` library has FDCAN Read/Write but no bxCAN, which is fine. Note: the "STM32 Nucleo H743ZI2" entry in the board list is the legacy MBED target. Do not use it. Revision C (architecture D96): the H743 is the twin's development target. The robot's delivered controller is the supplier's XT_PS V1.0 board carrying an STM32 Nucleo-32 (§2.2), the baseline provisionally until the supplier's data (card G1-04). The encoders are read by the ESCs (architecture D3); TIM2 is a 1 MHz time base (ICD 6). |
| D2 | **Companion = NVIDIA Jetson Orin NX 16 GB** (JetPack 6, ROS 2 Humble) runs GNC, LiDAR, RTK and vision. | Has a CAN-FD controller on board (MTTCAN, SocketCAN) and a GPU for the vision sprayer. Raspberry Pi 5 would also need an MCP2518FD HAT and has no GPU. Revision C (D96): a development target as well. The as-built record lists no companion computer; the sensor mast of phase 2 is a model addition. |
| D3 | **Two CAN buses.** FDCAN1 is classic CAN 2.0B at 500 kbit/s for the two traction ESCs (one dual-channel ESC per side), the implement ECU and the BMS. FDCAN2 is CAN-FD at 1 Mbit/s nominal / 2 Mbit/s data, point-to-point MCU↔Jetson. | CAN1 load is about 21 % (770 frames/s, ICD 4). CAN2 load is about 9 % (estimate from the ICD 5 frames, §2.8.2). 500 kbit/s is also the delivered BMS's bit rate (as-built record section 2). Isolated transceivers (ISO1042) and deterministic latency. A UART link was rejected because it gives weaker EMC robustness on a 48 V traction vehicle. |
| D4 | Revision A: the MCU closed the track-speed loops and sent torque commands. **Superseded by architecture D2 and D3:** each ESC runs its own speed loop in SPEED mode (DRV_CMD w_ref with tau_ff and the torque limits, ICD 4) and reads its motor encoders, which reach the MCU in DRV_FB3. The MCU runs the vehicle-level control at 100 Hz (`ts.ctrl` 10 ms): shaping, the yaw-rate PI on the IMU gyro, the skid mixing and the torque limits. | `params/param_electronics.m` header; drive spec 2.9. |
| D5 | **SBUS on USART2 RX** (100000 baud, 8E2, RX-pin inversion enabled in CubeMX). Read every 1 ms with `UART/USART Read` (non-blocking, up to 32 bytes, length output). Decoding is split in two. **Frame sync and validation** is a MATLAB Function, because no block exists for it. **Bit unpacking** uses **Communications Toolbox Integer-to-Bit / Bit-to-Integer Converter blocks** (LSB-first). | This keeps the MATLAB Function to the one job no block can do. The same converter blocks build the SBUS encoder in the RC simulation. |
| D6 | **Mode manager = a MATLAB Function FSM** (`fw_mode_manager`, `subsystems/electronics/mcu_mode_manager.m`, Task_10). States (ShaddadMode, ICD 9): BOOT, SELFTEST, STANDBY, MANUAL, HOLD, AUTO_START, AUTO, CTRL_STOP, ESTOP, FAULT. The precharge and contactor sequencing is the BMS's (architecture D4): the MCU only requests HV over CAN (BMS_REQ). | Revision A chose Stateflow; the Stateflow product is not installed (T09), so every state machine is a MATLAB Function. |
| D7 | **The hardware E-stop does not depend on the MCU.** A dual-channel E-stop loop feeds a safety relay. Its immediate contacts remove STO from both ESCs, so the spring brakes engage (stop category 0, architecture D86); its delayed contacts open the main contactors after `P.power.safety.t_cat1` 1.5 s (power spec 2.5). The MCU monitors the chain, and its DO0 MCU_SAFETY_OK contact sits in the loop (ICD 6). The RC remote-stop switch (CH12) is a functional (software) stop, because SBUS is not safety-rated. On the robot the E-stop buttons, the safety relay and the STO wiring are the delivered ones; their parts and category come with card G1-04 (D80, D96). | ISO 18497-1:2024 and ISO 13849-1:2023 good practice (safety plan Table 1, D103). In SIM, the safety chain is modelled on the plant side so tests can prove it works even when the firmware is frozen. |
| D8 | **Firmware uses single precision.** Integer types are used at the hardware boundary. Cumulative encoder counts stay in `int32`/`uint32` and are never integrated in float on the MCU. | Single is native on the FPU and matches the float32 CAN-FD payloads. It also avoids losing float precision over a 4 h shift. |
| D9 | **Task rates** (`P.electronics.ts`): 1 kHz base (`ts.base` 1 ms: SBUS bytes, DI, DO, watchdog), 200 Hz IMU (`ts.imu` 5 ms), 100 Hz control (`ts.ctrl` 10 ms), 20 Hz communications (`ts.comm` 50 ms), 10 Hz housekeeping (`ts.hk` 0.1 s). Rate-monotonic preemptive multitasking with explicit Rate Transition blocks (data integrity + determinism). | See §2.7.1 for the timing budget: about 20 % CPU from the WCET estimates `target.mcu.task_wcet`, against a budget of 50 % (`target.cpu_load_max`). |
| D10 | **SIM/HW I/O isolation.** Each I/O group is a Variant Subsystem with choices `SIM` (root-port plant connection) and `HW` (STM32 driver blocks), switched by `VC_TARGET` (ICD 8). Variant activation time is `update diagram`. **All CAN payload encoding/decoding (Byte Pack/Unpack with scaling, ICD 4 and 5; no DBC) sits in the common firmware logic.** Only FDCAN Read/Write sit in `HW`. The top-level components `CAN1_Net` (`CAN1_Net_Sim`) and `CAN2_Link` (`Link_MCU_GNC`) carry the frames between the MCU, the ESCs, the BMS, the implement and the companion. | The CAN encoding is therefore exercised in every simulation. |

---

## 1. Purpose and scope

### 1.1 In scope
1. **Electronics architecture.** MCU, companion computer, CAN networks, I/O list and pin map, safety chain (E-stop relay, STO, contactor coil, brake-release gating), electronics power rails and consumption, and the electronics-enclosure thermal model for the 45-50 °C Saudi climate.
2. **RC link (Phase 1).** Operator input (scripted, joystick or dashboard), RC transmitter, 2.4 GHz radio channel with loss, RC receiver failsafe behaviour, SBUS frame encoder, UART line timing, SBUS decoder in the MCU, and link-loss supervision.
3. **MCU firmware** (code-generated with Embedded Coder for STM32H7):
   - channel mapping, deadband and expo;
   - mode manager and arbitration between RC and companion commands;
   - reference shaping (acceleration and jerk limiting);
   - skid-steer mixing and the yaw-rate loop;
   - the track-speed references to the ESCs, which close the speed loops (architecture D2, D3);
   - torque, power and traction limits;
   - safety supervisor;
   - the HV requests to the BMS, which owns the precharge and the contactors (architecture D4);
   - implement, lights, beacon, buzzer and fan control;
   - watchdog;
   - CAN1/CAN2 codecs and telemetry.
4. **Plant-side electronics emulation.** The CAN1 network and the CAN2 link (`CAN1_Net_Sim`, `Link_MCU_GNC`), the companion's CAN-FD side (`companion_io`, `companion_io_ros2`), the ADC/DI front end, the actuators, the watchdog-reset emulation, the electronics' LV demand per rail (handed to the power domain's LV distribution, which owns the DC/DC converters) and the enclosure thermal network (Simscape thermal).

### 1.2 Out of scope (owned by neighbours)
| Neighbour domain | What they own | What crosses the interface |
|---|---|---|
| **drive** | Traction motors, the ESCs (speed loop in SPEED mode, encoders, CAN command time-out, brake control; D2, D3, D71), gearboxes, sprockets, spring-applied brakes | The CAN1 drive frames DRV_CMD, DRV_FB1, FB2 and FB3 (ICD 4) through `CAN1_Net` |
| **power** | The pack (13-cell NMC as built), the BMS (the twin's G4 BMS ECU owns K1, K2 and K4, D4; the delivered BMS is the baseline, D96), the PDU, the LV distribution with its DC/DC converters, the safety chain, the implement ECU and power | The CAN1 BMS frames and BMS_REQ (ICD 4); `Bus_PowerStatus` (to me), `Bus_PowerHwCmd` and `Bus_LV_LoadDemand` (from me) (ICD 3.2) |
| **sensors** | IMU, GNSS and LiDAR sensor models (noise, quantization) | The IMU sample into the MCU (`Bus_McuIn5`, ICD 3.1) |
| **gnc** | Companion software: EKF fusion, path planning, path following, missions, vision-sprayer decisions | The CAN2 frames COMP_CMD, MCU_STATUS, MCU_IMU, MCU_ODOM and MCU_POWER (ICD 5) through `CAN2_Link` |
| **structure** | Chassis and track dynamics, implement mechanics (mower, tiller, sprayer, lift) | `Bus_ImplementCmd` (from me), `Bus_ImplMechState` (to me); track gauge B for the mixer |
| scenario / top level | Ambient conditions, operator scenario, E-stop button presses | `Bus_EnvState`, `Bus_HmiInputs`, `Bus_FaultInject`, the operator input of `Operator_Console` (ICD 3.1) |

---

## 2. Physical and algorithmic model description

### 2.1 Electronics architecture

The diagram shows the twin's development target (D96). The robot as delivered is in §2.2.

```
   48 V pack, 13-cell NMC, main fuse, the BMS-owned contactors K1, K2 (precharge) and K4 (D4),
   the LV distribution with its DC/DC converters and LDO (power domain, power spec; P.power.lv)
        |  LV rails 24 / 12 / 5 / 3.3 V, sized from the electronics' LV demand (Bus_LV_LoadDemand)
        |                                        48 V DC link -> ESC L, ESC R, implement ECU
        v
 +-------------------------------------------------------------------------------------+
 |  MCU board: STM32H743 (NUCLEO-H743ZI2), the development target                        |
 |   USART2_RX PD6 (inverted) <- SBUS <- RC receiver (FrSky Archer SR10 class)            |
 |   USART3 <-> ST-LINK VCP (debug / External Mode)                                      |
 |   I2C1 (400 kHz) <-> ICM-20948 9-axis IMU                                             |
 |   TIM2 (32-bit, 1 MHz) -> t_us time base (the encoders are read by the ESCs, D3)       |
 |   ADC1 x8 <- Vpack, Vbus, Ipack (Hall), V24, V12, V5, T_encl (NTC), lift pot           |
 |   GPIOF (14 DI) <- E-stop ch1/ch2, safety OK, bumpers, reset, K1/K2 feedback,          |
 |                    IMPL_ID0/1, lid, key, safety HV OK, dock present                   |
 |   GPIOG/GPIOE (20 DO) -> MCU_SAFETY_OK (in the E-stop loop), safety reset pulse,       |
 |                    IMPL24_EN, beacon RGB, buzzer, work lights, companion and LiDAR     |
 |                    power, nozzles 1-8, lift direction                                 |
 |   TIM1 CH1/CH2 PWM 20 kHz -> sprayer pump, lift actuator; TIM4 CH1/CH2 25 kHz -> fans  |
 |   FDCAN1 (500 kbit/s, ISO1042) <-> ESC L (0x201, 0x181, 0x281, 0x381), ESC R (0x202,   |
 |                    0x182, 0x282, 0x382), implement ECU (0x140, 0x141),                |
 |                    BMS (0x351 to 0x3C1, BMS_REQ 0x3D0)                                |
 |   FDCAN2 (CAN-FD 1 / 2 Mbit/s) <-> Jetson Orin NX (SocketCAN can0, ROS 2 bridge, D38)  |
 |   IWDG1 (50 ms)                                                                       |
 +-------------------------------------------------------------------------------------+
  E-stop front + rear, bumpers and MCU_SAFETY_OK (one NC loop) -> SAFETY RELAY ->
  STO L/R at once; main contactors open after t_cat1 1.5 s (power spec 2.5)
```

### 2.2 Hardware: the twin's design and the robot as delivered (D96; `P.electronics.target`, `P.electronics.hw`)

Architecture D96 (the project owner, 2026-11-18, provisional): the robot's delivered BMS, charger, ESCs and controller board are the baseline that the requirements aim at (need N-17). The twin's STM32H743 design stays the firmware's development target and its STM32G4 BMS a variant. Each unknown takes a provisional value with its margin; ICD sections 4 and 6 are provisional. The final choice comes with the supplier's data (card G1-04), before cards G2-DRV, G2-PWR and G2-MCU start; the final board after the safety concept's architecture (E-05). The robot column comes from the as-built record `docs/hardware/real_robot_components.md` (photos and the supplier's sheets, tagged [DATA] or [EST] there).

| Function | The twin's design (development target) | The robot as delivered (baseline, provisional) | Status |
|---|---|---|---|
| Controller | STM32H743ZIT6 on a NUCLEO-H743ZI2: 480 MHz Cortex-M7 (`target.mcu.f_cpu`), 2 MB flash and 1 MB RAM (`target.mcu.flash`, `ram`), 2 × FDCAN, IWDG; design budget 50 % CPU and 1 MB flash (`target.cpu_load_max`, `flash_max`) | The supplier's XT_PS V1.0 board (+48 V input, CAN, optocoupled I/O) carrying an STM32 Nucleo-32, and a second STM32 board ("Black Pill" style, USB-C) [DATA] | Chips unknown (open item 5, card G1-04). A Nucleo-32-class part may lack the pins and a second FD-capable CAN (review MCU-09, R-59) |
| Companion computer | NVIDIA Jetson Orin NX 16 GB, 10 / 12 / 22 W at 12 V (`load.jetson`) | None recorded (the phase-2 sensor mast is a model addition) | Phase 2 |
| RC transmitter and receiver | RadioMaster TX16S and FrSky Archer SR10, SBUS raw range 172..992..1811, frame 14 ms (`P.rc.frame_period`; 7 ms high speed) | Not recorded | Card G1-04 (REQ-SYS-008 note) |
| CAN1 transceivers | TI ISO1042 (isolated, CAN-FD capable) | On the XT_PS board, type not recorded | Card G1-04 |
| BMS on CAN1 | The twin's STM32G4 BMS ECU (`target.bms`: 170 MHz, 512 KB flash, 128 KB RAM); frames 0x351 to 0x3C1 and BMS_REQ 0x3D0 (ICD 4) | Bought 13-cell Li-ion NMC BMS, 120-150 A continuous, 200-300 A peak, CAN 500 kbit/s on a GX-12 connector (pin 1 CANH, pin 2 CANL) [DATA] | Frame layout, periods and thresholds unknown (R-15, card G1-04) |
| ESCs on CAN1 | A CiA-402-class drive per side in SPEED mode, frames DRV_CMD and DRV_FB1 to FB3 (ICD 4) | One dual-channel 48 V BLDC/FOC controller per side, CAN; 20 A rms continuous, 50 A rms peak per channel [EST] | Model, protocol, enable sequence, STO input and encoder type unknown (R-14, card G1-04) |
| Encoders | 4096 counts per motor revolution in DRV_FB3 (`drv.enc_cpr`, the former motor's 1024 PPR × 4) | Unknown; Hall sensors only would give 24 counts per revolution | Card G1-04 |
| IMU | TDK ICM-20948 on I2C1 at 400 kHz (`target.mcu.i2c_f`); the blockset has an `ICM20948 IMU Sensor` block (risk R9) | Not recorded | Phase 2 sensors |
| E-stop and safety relay | Dual-channel E-stop loop, a PILZ PNOZ s4-class relay with a manual monitored reset; response ≤ 20 ms, modelled 15 ms (`P.power.safety.t_relay_off`), within REQ-SYS-063's 50 ms to STO (provisional) | E-stop buttons and a safety relay with STO wiring (the owner, 2026-11-16) | Parts, category, wiring and the relay's ambient rating unknown (D80, card G1-04) |
| Current sensing | Hall sensor ±300 A at 1.65 V + 4.1667 mV/A (`adc.i_off`, `adc.i_sens`) into the 16-bit ADC1 | ADS1115/ADS1015 16-bit I2C ADC with a current-sensor header "CUR" [DATA] | Sensor type unknown |
| LV supply | The power domain's DC/DC converters (24 V 600 W, 12 V, 5 V and the 3.3 V LDO; `P.power.lv`) | An LM2596 buck module (at most 40 V input, so fed from a lower rail) [DATA]; the converter from 48 V is unknown | Open item 6, card G1-04 |
| Electronics enclosure | Aluminium IP65 box, 12 kg, 1.2 m² of surface (`therm.mass`, `therm.area`; §2.9.4). Revision A gave it as 0.6 × 0.4 × 0.25 m, whose plain surface is only 0.98 m²; no source records the other 0.22 m² (fins or a mounting plate would give it), so the area is to settle on card G2-VAL with the robot's box | Electronics box with boards, enclosure and harness, 6 kg [EST]; rating unknown | Card G1-04 |
| Sprayer electronics | The implement ECU model on CAN1 (IMPL_CMD, IMPL_FB) | The supplier's SPR V1.3 sprayer board [DATA] | Later phase (sprayer) |

The electronics plant parameters carry no [DATA] or [EST] tags yet (review ELC-08); tagging them is a parameter change for card G2-VAL.

### 2.3 I/O list (the development target's map; ICD 6 binds)
The pins are those of the NUCLEO-H743ZI2 proposal, to be frozen in the CubeMX project `utilities/stm32/shaddad_h743.ioc`. The robot's board (§2.2) has its own map, unknown until card G1-04 (D96).

| Group | Peripheral / pin (NUCLEO-H743ZI2) | Signal | Rate | Firmware block (HW variant) |
|---|---|---|---|---|
| SBUS | USART2_RX PD6 (AF7), RXINV=1, 100000 Bd, 9-bit word incl. even parity, 2 stop | SBUS byte stream, up to 32 bytes per tick (`sbus.read_max`, `hw.uart_len`) | 1 ms (`ts.base`) | `UART/USART Read` |
| Debug | USART3 PD8/PD9 (ST-LINK VCP) 921600 Bd | External mode / log | n/a | (External Mode) |
| Time base | TIM2, 32-bit free-running at 1 MHz (`target.mcu.tim2_f`) | `t_us` | 1 ms | `Register Read/Write` (TIM2 CNT, D60) |
| IMU | I2C1 PB8/PB9, addr 0x69, 400 kHz (`target.mcu.i2c_f`) | acc, gyr, mag, temp | 5 ms (`ts.imu`) | `ICM20948 IMU Sensor` |
| CAN1 | FDCAN1 PD0/PD1, 500 kbit/s (`can.can1_bitrate`) | ESCs, implement ECU, BMS (§2.8.1) | frames every 10, 50 or 100 ms (ICD 4) | `FDCAN Read/Write` |
| CAN2 | FDCAN2 PB12/PB13, 1 / 2 Mbit/s (`can.can2_nominal`, `can2_data`) | companion link (§2.8.2) | frames every 5 to 100 ms (ICD 5) | `FDCAN Read/Write` |
| ADC | ADC1, 8 regular conversions at 16 bit (PA3, PA4, PA6, PB0, PB1, PC0, PC2, PC3; `hw.adc1_n`, `adc.nbits`) | `adc.ch` order (§6) | 50 ms (`ts.comm`) | `Analog to Digital Converter` |
| MCU temp | ADC3 internal temperature sensor, factory calibration `adc.ts_cal_T` / `ts_cal_cnt` | T_mcu | housekeeping | `Analog to Digital Converter` |
| DI | GPIOF[0:13], 14 inputs (`hw.di_pins`) | DI word bits 0 to 13 (§5.4, ICD 6) | 1 ms | `Digital Port Read` |
| DO | GPIOG[0:15] + GPIOE[0:3], 20 outputs (`hw.do_g_pins`, `hw.do_e_pins`) | DO word bits 0 to 19 (§5.4, ICD 6) | 1 ms | `Digital Port Write` x2 |
| PWM | TIM1 CH1 PE9, CH2 PE11 at 20 kHz (`target.mcu.pwm_f`) | pump, lift | 10 ms | `PWM Output` |
| PWM | TIM4 CH1 PD12, CH2 PD13 at 25 kHz | fan 1, fan 2 | 10 ms | `PWM Output` |
| Watchdog | IWDG1 (PR /4, RLR 399 → 50 ms; `wdg.prescaler`, `wdg.reload`, `wdg.timeout`) | refresh KR 0xAAAA (`wdg.kr_refresh`) | 1 ms | `Register Read/Write` |
| Reset cause, bus-off | RCC_RSR IWDG1RSTF bit 26 and RMVF bit 23; FDCAN_PSR BO bit 7 (`hw.rcc_rsr_iwdg1rstf_bit`, `hw.rcc_rsr_rmvf_bit`, `hw.fdcan_psr_bo_bit`) | `reset_flag`; bus-off fault bits 25 and 26 | at boot; 10 ms | `Register Read/Write` (D60) |

### 2.4 Electronics power consumption (auxiliary loads)
The electronics plant (`elp_lv_demand`) computes the demand per LV rail (`Bus_LV_LoadDemand`: 24 V, 12 V, 5 V, 3.3 V; the rail of each load per D61: 24 V the beacon, the work lights and the 3-D LiDAR; 12 V the MCU board, the Jetson, the modem, the cameras, the display and the fans; 5 V the RC receiver and the GNSS; 3.3 V the IMU and the CAN transceivers) from the DO and PWM states and `P.electronics.load`; the power domain's LV distribution converts it with its DC/DC converters and LDO (`P.power.lv`) and draws it from the 48 V bus. Values are in W at the rail. "In box" marks the heat inside the enclosure (§2.9.4).

| Load | Rail | `P.electronics.load` | Drawn in the twin | On the robot as built (phase 1) | In box |
|---|---|---|---|---|---|
| MCU board | 12 | `mcu` 3.0 | always | the supplier's XT_PS board, power not recorded | yes (`inbox_mcu`) |
| IMU and CAN transceivers | 3.3 | `imu` 0.03, `can_xcvr` 0.3 | always | not recorded | |
| RC receiver | 5 | `rc_rx` 0.5 | always | not recorded | |
| RTK GNSS | 5 | `gnss` 1.6 | always | not fitted (phase 2) | |
| Jetson Orin NX | 12 | `jetson` 10 / 12 / 22 (standby, work, AUTO with vision) | 12 W while DO8 COMPANION_PWR_EN is set, which is every mode but BOOT | not fitted | yes (`inbox_jetson`) |
| 3-D LiDAR | 24 | `lidar3d` 20 | while DO9 LIDAR_PWR_EN is set (every mode but BOOT) | not fitted | |
| 2-D LiDAR | - | `lidar2d` 0 | none (not in the LV demand) | not fitted | |
| NTRIP / UHF modem | 12 | `modem` 3 | always | not fitted | |
| Sprayer cameras | 12 | `cams` 8 | always | not fitted | |
| Front display | 12 | `display` 5 | always | not recorded | |
| Fans (two) | 12 | `fan_each` 5 × duty | by the fan schedule (`aux.fan_bp`, `aux.fan_duty`) | not recorded | |
| Beacon and buzzer | 24 | `beacon` 3 | by the beacon and buzzer outputs | not recorded | |
| Work lights (two) | 24 | `lights_each` 20 | by DO7 WORK_LIGHTS | not recorded | |
| In-box losses (DC/DC conversion, relays, CAN, fuses) | | `P_inbox_extra` 15 | always, as heat | | yes |

Moved out of this table since revision A: the brake-release holds are the drive's (four brakes held from 48 V at duty 0.48, about 12 W each; D64), and the contactor coil holds are the power domain's (`P.power.lv.coil_hold_W`). T-EL-16 (`testLvDemand4hDayAuto`) checks the per-rail split against `P.electronics.load` within 1 % and the 48 V-side draw (the converters, the coil holds and the brake holds) at 105 W plus the extra brake holds of the four motors, about 128 ± 10 W (`tests/electronics/t_elec_plant.m`).

Energy: at about 128 W, a 4 h day draws about 0.51 kWh from the pack. The power domain includes the LV demand in its energy budget (D101: each mowing mission on one charge, with 10 % margin above the 0.20 reserve). Finding (review ELC-01, R-19): the twin draws about 50 W at the rails for phase-2 loads (Jetson, 3-D LiDAR, GNSS, modem, cameras, display) that the robot driven by RC today does not have, so the row test's energy figures carry them.

### 2.5 Safety architecture
- **Category 0 (hardware, MCU-independent; architecture D86).** If any E-stop or bumper is pressed, the dual NC loop opens and the safety relay drops (modelled 15 ms, `P.power.safety.t_relay_off`). Its immediate contacts remove STO from both ESCs (torque off), so the spring brakes engage, and open K3 (the 24 V implement rail); its delayed contacts open the main contactors after `P.power.safety.t_cat1` 1.5 s (power spec 2.5). REQ-SYS-063 requires STO within 50 ms of the press (bounded by REQ-SYS-139's 0.6 m; provisional until card G1-04's data on the delivered relay and ESCs); the twin's relay drops in 15 ms, and S8a checks STO within 20 ms.
- **Remote E-stop (RC switch SE on CH12, 2-position; ICD 7).** A functional stop, evaluated only on valid frames (D34), with the 40 ms persistence of fault bit 1 (`safety.t_persist`). The mode switch CH5 (SA) selects MANUAL, HOLD or AUTO (positions 1, 2, 3); it carries no stop. The MCU commands a controlled deceleration of 2.0 m/s² for at most 1.2 s, then parks: removes DRIVE_EN and engages the brakes, with the torque limits held until both brakes confirm (§2.7.3). The state latches to ESTOP.
- **Category 2 (controlled stop).** Triggers are RC link loss or failsafe, ARM switch off, companion timeout or stop request, and class-B faults, except TASK_OVERRUN. Bit 23 is class B but in maskDO0 (`safety.maskDO0`: bits 5-8, 22, 23, 25; D30): when it sets, DO0 drops, the safety relay removes STO at once and the spring brakes stop the robot (category 0), and bit 31 then gives FAULT (§2.7.1, ICD 6). For the other class-B faults the MCU decelerates at 1.0 m/s², holds zero speed for 0.5 s (or until its time-out, the smaller of 5 s and v0/a_ctrlstop + 1 s), then parks: the drive disabled with park_req, brakes engaged, torque limits held until both brakes confirm; then STANDBY.
- **Watchdog.** If IWDG1 is not refreshed for 50 ms, the MCU resets. All DO lines then go low, which is fail-safe: DO0 MCU_SAFETY_OK opens its contact in the E-stop loop, so the safety relay drops as for an E-stop (ICD 6). The ESCs also go to QUICK_STOP after 100 ms without a new command, then brake (ICD 4; drive spec 2.9). This requirement is placed on the drive domain.
- **Recommendation (not modelled as safety-rated).** For unattended AUTO operation per ISO 18497-1 to -4:2024, add a certified wireless E-stop such as FORT Robotics SRC (PL d, rated by ISO 13849-1:2023) in series with the E-stop loop (the standards with their editions: safety plan Table 1, D103). The model has a `estop_wireless` input for it.

### 2.6 RC link and SBUS

#### 2.6.1 SBUS physical/frame definition
- Line: 100000 baud, 1 start + 8 data + even parity + 2 stop = **12 bit/byte**. T_byte = 12/100000 = **120 µs**. Inverted TTL, which the STM32 handles with RXINV (no external inverter).
- Frame (25 bytes): `b0 = 0x0F` | `b1..b22` = 16 channels × 11 bit, packed **LSB-first** | `b23` = flags | `b24` = footer `0x00`. SBUS2 telemetry footers `0x04, 0x14, 0x24, 0x34` are also accepted: `(b24 & 0x0F) == 0x04`.
- Flags byte b23: bit0 = CH17 (digital), bit1 = CH18, **bit2 = frame lost**, **bit3 = failsafe**, bits 4-7 = 0 (checked).
- Frame duration = 25 × 120 µs = **3.0 ms**. Period T_f = 14 ms (normal) or 7 ms (high-speed). The inter-frame idle gap is 11 ms or 4 ms.
- Bit packing. Let `bit(8i+k) = (b_{i+1} >> k) & 1` for i = 0..21, k = 0..7. Then

  `ch_c = Σ_{j=0}^{10} bit(11c + j) · 2^j`, for c = 0..15.

  This is exactly *Integer-to-Bit (8 bit, LSB first) → Bit-to-Integer (11 bit, LSB first)*. The encoder is the reverse: Integer-to-Bit (11, LSB first) then Bit-to-Integer (8, LSB first).
- Channel scaling (FrSky default; per-channel vectors allow Futaba 368..1680):
  - decode: x = sat((raw − raw_mid)/819.5, −1, 1);
  - encode: raw = sat(round(raw_mid + 819.5·x), 172, 1811).

#### 2.6.2 Operator → transmitter → radio → receiver (plant side, `RC_Link_Sim`)
1. **Operator input** (the top-level component `Operator_Console`, Variant Source `OpSource` on `VC_OPSRC`, ICD 8), which feeds `Operator_RC` with `Bus_OperatorCmd` every 10 ms:
   - (1) the scenario input `ScenarioIn.op`;
   - (2) `Dashboard`: dashboard blocks bound to the constants `P.scen.dash.*`;
   - (3) `Joystick`: a transmitter in USB joystick mode through Simulation 3D Joystick and `joy_map` (`vrlib/Joystick Input` was removed in R2026b; run with simulation pacing 1×);
   - (4) `RowDriver`: the simulated driver of the RC row test (`row_driver_step`, scenario R1, `docs/row_test.md`).
2. **Transmitter** at T_f. It maps sticks and switches to 16 raw channels per `P.rc.ch_map`. A 3-position switch maps to {172, 992, 1811} and a 2-position switch to {172, 1811}.
3. **Radio channel.** Each frame is lost when `U < p_loss` (Uniform Random Number, seeded) or when the scenario signal `rf_link_ok = 0` or `tx_power_on = 0`. Latency: revision B (D37) - one frame (`P.rc.latency_frames` = 1) is produced by the deterministic frame-period -> 1 ms Rate Transition after the encoder; the explicit Delay has length `P.rc.extra_delay_frames` = 0.
4. **Receiver** (built from blocks, D59; `rc_receiver`):
   - It holds the last good channels.
   - On a lost frame it sets frame_lost = 1.
   - After `rx_failsafe_time` = 1.0 s of continuous loss, behaviour depends on `rx_failsafe_mode`:
     - `1` FLAG_PRESET: frames continue with failsafe = 1 and preset channels;
     - `2` NO_PULSES: frames stop entirely;
     - `3` HOLD: frames continue with lost bits only.
   - The presets of FLAG_PRESET (`P.rc.failsafe_raw`, D34): sticks 992 (centre), the mode switch 992 (HOLD), arm 172 (off), the remote stop CH12 172 (RUN), reset, lights, mission, return home and implement 172, lift 992.
5. **SBUS encoder** (blocks only, §3.3).
6. **UART line timing** (MATLAB Function `sbus_uart_line_sim`, sim-only). It turns each 25-byte frame into a byte stream with byte completion times t_start + i·T_byte (i = 1..25). The frame start has a sub-millisecond phase offset and a clock drift (ppm). Optional byte errors and drops come from seeded Uniform Random Number vectors. On each 1 ms tick it outputs the bytes completed since the previous tick, as `uint8[32]` plus a length. This gives 8, 8 and 9 bytes per tick, exactly as the HW UART Read returns them.

#### 2.6.3 MCU decoding (firmware, 1 kHz)
- **Frame sync** (MATLAB Function `sbus_frame_sync`, §4.1):
  - If `gap_ticks` = 2 consecutive ticks have zero bytes, the parser resets (frames themselves always deliver ≥ 7 bytes/ms).
  - It hunts for `0x0F` at index 0 and collects 25 bytes.
  - It validates the footer and the flag reserved bits, and rejects a frame on a UART error.
  - Revision C (S6c): a UART error (parity / framing, `uart_err` of the 1 ms chunk) taints every frame that has a byte in that chunk, including the frame that starts in it (revision B reset the parser before consuming the flagged chunk, so errors in the header chunk, CH1-CH5, were accepted). A frame is also rejected when a byte follows the footer in the same chunk (length) or when any channel is outside `rcmap.raw_min..raw_max` (172..1811, ICD 7). All checks finish on the tick of the last byte: no added latency (D37).
  - Output: payload `uint8[22]`, flags `uint8`, `frame_new`, counters.
- **Unpack.** Integer to Bit Converter (nbits 8, LSB first) → Bit to Integer Converter (nbits 11, LSB first, uint16) → `ch_raw uint16[16]`. The flags byte goes through Integer to Bit (nbits 8, LSB first) → Demux → ch17, ch18, frame_lost, failsafe. Values are latched by an Enabled Subsystem on `frame_new`.
- **Link supervision** (blocks):
  - `rc_age` resets on a good frame and adds T_base otherwise; it saturates at 10 s.
  - `rc_link_lost = rc_age > link_timeout (0.10 s ≈ 7 frames at 14 ms)`.
  - `rc_ok = ¬rc_link_lost ∧ ¬failsafe`.
  - Link quality LQ is a first-order filter of (1 − frame_lost), updated on each frame with α = 0.05 (≈ 20-frame memory). A warning is raised at LQ < 0.70.

### 2.7 MCU firmware

#### 2.7.1 Tasks and timing (STM32H743 @ 480 MHz, the development target (D96); WCET estimates to be confirmed with the Execution Profiler in PIL)
The revision-B task set is `P.electronics.target.mcu.task_T` = [1 5 10 50 100] ms, with the WCET estimates `task_wcet` (`params/param_electronics.m`; `build_shaddad_mcu.m`; checked by `t_targets` testDesignEstimate).

| Task | Period | Priority | Content | Est. WCET | CPU |
|---|---|---|---|---|---|
| `Task_1k` (base rate) | 1 ms | highest | `fw_sbus_decoder` (UART chunk, SBUS frame sync and unpack, link supervision); `fw_watchdog_mgr` (task ages, IWDG kick, DO0 MCU_SAFETY_OK, DO1 SAFETY_RESET_REQ); DO word; watchdog-reset counter; ms tick. The IO variants read the DI word, the µs timer and the reset flag, and write the DO word, at this rate. | 25 µs | 2.5 % |
| `Task_5` | 5 ms | 2 | `sens_acq_imu` (the IMU read over I2C and its validity checks; sensors spec); `fw_imu_tilt` (tilt estimate, §2.7.10) | 650 µs | 13 % |
| `Task_10` | 10 ms | 3 | `fw_rc_mapping`; `sens_odom_compute` (track speeds from the ESCs' FB3 encoder counts; sensors spec); `fw_safety_supervisor`; `fw_mode_manager`; `fw_vehicle_ctrl` (arbitration, descent cap, shaper, yaw-rate loop, mixer, torque limits, ESC setpoints); `fw_telemetry`; the drive frames on CAN1 (DRV_CMD out, FB1 and FB3 in) | 380 µs | 3.8 % |
| `Task_50` | 50 ms | 4 | `fw_adc_scaling`; `fw_implement_aux` (rotor, nozzles, pump, lift, beacon, buzzer, lights, fans, power enables); MCU_STATUS assembly | 100 µs | 0.2 % |
| `Task_100` | 100 ms | lowest | alive counter (the watchdog's slowest task) | 50 µs | 0.05 % |
| | | | **Total** (limit 50 %: `target.cpu_load_max`, build plan T17) | | **≈ 20 %** |

The CAN1 and CAN2 codec layers (`fw_can1_codec`, `fw_can2_codec`) are multirate: each frame is packed or unpacked at the rate of the task that uses it.

Configuration:
- Fixed-step, FixedStep `1e-3`, through the ConfigSetRef `Shaddad_Ref` (§7.2).
- `EnableMultiTasking = on`, Embedded Coder `ert.tlc`.
- STM32 hardware settings: "Enable overrun detection" on. 3 consecutive overruns (`safety.overrun_n`) set `TASK_OVERRUN` (bit 23, class B, latched, in `maskDO0`). DO0 then drops, so the safety relay removes STO at once and the spring brakes stop the robot (category 0); bit 31 SAFETY_RELAY_OPEN then gives FAULT. The class-B controlled deceleration does not run with torque. On the robot the scheduler's overrun count is not read yet (R-08, card G2-MCU).
- All rate crossings use `Rate Transition` with `Integrity=on` and `Deterministic=on`.

#### 2.7.2 RC channel mapping (Task_10)
Default channel map (`P.electronics.rcmap.ch`; normative in ICD section 7):

| Ch | Function | Source on TX (Mode 2) |
|---|---|---|
| 1 | steer | right stick X |
| 2 | throttle (speed fwd/rev) | right stick Y, spring-centred |
| 3 | aux | left stick Y (not used) |
| 4 | aux | left stick X (not used) |
| 5 | **mode** (3-pos) | SA: MANUAL / HOLD / AUTO |
| 6 | **arm** (2-pos) | SF |
| 7 | speed scale | S1 pot |
| 8 | implement (3-pos) | SB: OFF / ON / ON+auto-spray |
| 9 | fault/E-stop reset | SH (momentary) |
| 10 | work lights | SD |
| 11 | lift (3-pos) | SC: UP / HOLD / DOWN |
| 12 | **remote E-stop** (2-pos, latching) | SE: RUN / STOP |
| 13 | mission (3-pos, momentary) | SG: edge to pos 2 = START/RESUME, pos 3 = PAUSE |
| 14 | return home (momentary) | SI: rising edge = RTH |
| 15-16 | spare | |

Processing chain (blocks):
1. Normalise: `x = sat((raw − mid)/819.5)`, with an optional sign per channel.
2. Deadband with rescaling: `x_db = sign(x)·max(|x|−d,0)/(1−d)`, d = 0.05. Implemented as `Dead Zone` [−d, d] followed by `Gain` 1/(1−d).
3. Expo: `y = e·x³ + (1−e)·x`, implemented with `Polynomial` Coefs `[e 0 1-e 0]`. e_thr = 0.20, e_steer = 0.35.
4. Switch decode: `pos = 1 + (x > −0.5) + (x > 0.5)` from 2× `Compare To Constant` + Sum (`rcmap.sw_thresh` 0.5). Debounce: accept a new position only when the last 4 samples at 10 ms agree (`rcmap.debounce_n` 4): a new position must last 30 ms (3 delays and the current sample) to pass, and a held change is accepted 30 to 40 ms after it, about 3 frames. Implemented with `Tapped Delay` (NumDelays 3, includeCurrent on), `MinMax` min and max, `Relational Operator` ==, `Switch` and `Unit Delay` (hold). The mode is forwarded as `rc_mode_sw` = pos − 1 (0 MANUAL, 1 HOLD, 2 AUTO).
5. Speed scale: `s = s_min + (x+1)/2·(1 − s_min)`, s_min = 0.25.
6. Arming preconditions: `sticks_neutral = |thr| < 0.08 ∧ |steer| < 0.08` (`rcmap.neutral_tol`). `arm_req` = rising edge of arm (`Detect Rise Positive`). The mode manager takes the arm edge, the mode switch and the override only while `rc_ok` (held otherwise). The arm edge and the reset (CH9) are still taken from every decoded frame, so a receiver failsafe preset can give an arm edge when the link returns (R-35). REQ-SYS-011 asks for valid frames only: planned on card G2-MCU.
7. Operator override in AUTO: `override = (|thr| > 0.3 ∨ |steer| > 0.3)` persisting ≥ 0.2 s (`rcmap.override_tol`, `override_t`).
8. Remote E-stop (CH12): threshold decode with no tapped-delay debounce, evaluated only on frames with failsafe = 0 and frame_lost = 0 and held otherwise (D34); the supervisor's 40 ms persistence sets bit 1. It is a software stop, not safety-rated.
9. Mission commands: an edge of CH13 to pos 2 gives START/RESUME and to pos 3 gives PAUSE; a rise of CH14 gives RTH. They go out as `rc_aux_cmd` (`rcmap.aux_code` [2 3 6]) with `rc_aux_seq` (+1 per edge) in MCU_STATUS, and the companion edge-detects them (gnc spec 5.1).

#### 2.7.3 Mode manager (MATLAB Function `mcu_mode_manager.m` in `fw_mode_manager`, Task_10)
The mode manager is a MATLAB Function FSM, because Stateflow is not installed. It is called once per Task_10 step (10 ms), and every timer counts 10 ms ticks (`prm_n`, computed from the times in `P.electronics.safety`). The states are the `ShaddadMode` codes (ICD section 9).

Inputs: `rc_ok, sw_mode (1 MANUAL, 2 HOLD, 3 AUTO), arm, arm_req, sticks_neutral, override` (fw_rc_mapping); `reset_req` (DI RESET_BTN or RC CH9, level) and `reset_evt` (the watchdog-reset event); `fault_word` and `fault_cls` [A B C]; `comp_ok, gnc_ready, mode_req` (the companion's COMP_CMD); `hv_ready` (BMS HV_Ready); `key_on, safety_ok, lid_closed, dock_present` (DI word); `standstill, v_max, brakes_ok, selftest_ok`.

Outputs: `mode (ShaddadMode), cmd_src (ShaddadCmdSrc), drive_enable, rearm_req` (one-sample pulse that starts DO1, D30), `bms_req` [HV_Enable_Req Charge_Enable_Req Fault_Reset_Req Shutdown_Req] (BMS_REQ, D4) and `stop_ctl` [limits_hold stop_fail] (D71).

- `standstill` = max(|v_track_L|, |v_track_R|) < `safety.standstill_v` (0.05 m/s), so a pivot is not a standstill (R-34). It is used by the CTRL_STOP hold, the end of ESTOP.DECEL and the ESTOP and FAULT reset.
- `v_max` = the faster track's |v| (m/s): the stop monitor's speed, and v0 at CTRL_STOP entry.
- `brakes_ok` = both ESCs report the brake engaged with the torque off: FB1 brake_released false, the ESC state READY, NOT_READY, FAULT or STO, rx_ok, and drv_age fresh (D71).
- `selftest_ok` = rails in range, ESC and BMS frames fresh, IMU valid.
- `stop_fail` is the STOP_FAIL monitor (fault bit 22, §2.7.10).
- The RC inputs (the mode switch, arm and override) are used only while `rc_ok`, and held otherwise (§2.7.2).
- The HV request to the BMS is `hv_req = key_on ∧ ¬class A ∧ mode ∉ {BOOT, FAULT}`; the charge request is `key_on ∧ dock_present ∧ STANDBY`. The BMS owns the precharge and the contactors (D4; power spec).

| From | Condition (priority order) | To | Entry actions |
|---|---|---|---|
| any | `reset_evt` (a watchdog reset) | BOOT | all outputs off |
| BOOT | one tick | SELFTEST | `rearm_req` pulse (D30; removed by D91, see below) |
| any except ESTOP | HW E-stop (bit 0) | ESTOP.SAFE | drive off |
| SELFTEST, STANDBY, ACTIVE, CTRL_STOP | remote E-stop (bit 1, CH12) | ESTOP: DECEL if the drive was enabled; the ESTOP park from the CTRL_STOP parking (its timer kept) or when the drive was enabled with bit 16 set; else SAFE | DECEL: the ESTOP stop profile (2.0 m/s², §2.7.5) |
| any except ESTOP and FAULT | class A (except bit 0) | FAULT | drive off (stop category 0) |
| SELFTEST | `selftest_ok` after `selftest_t` 1.0 s | STANDBY | |
| SELFTEST | `selftest_timeout` 2.0 s | FAULT | |
| STANDBY | `rc_ok ∧ arm_req ∧ sticks_neutral ∧ hv_ready ∧` DI SAFETY_OK `∧` sw 1 `∧` no class A or B | ACTIVE.MANUAL | |
| STANDBY | `rc_ok ∧ arm_req ∧ hv_ready ∧` DI SAFETY_OK `∧` sw 3 `∧ comp_ok ∧ gnc_ready ∧` lid closed `∧` no class A or B (D20) | ACTIVE.AUTO_START | |
| ACTIVE (MANUAL, HOLD, AUTO_START, AUTO) | class B, or `¬arm`, or `¬rc_ok` unless in AUTO with `rc_required_in_auto` false (D44) | CTRL_STOP | |
| AUTO_START, AUTO | `¬comp_ok`, or the companion's `mode_req` FAULT_STOP | CTRL_STOP | |
| MANUAL | sw 2 | HOLD | |
| MANUAL | a change of the switch to 3: AUTO_START when `comp_ok ∧ gnc_ready ∧` lid closed, else HOLD | AUTO_START or HOLD | |
| HOLD | sw 1 | MANUAL | |
| HOLD | sw 3, and `comp_ok ∧ gnc_ready ∧` lid closed | AUTO_START | |
| AUTO_START | `auto_start_warn_t` 3 s with sw 3 | AUTO | the start warning before it (§2.7.12) |
| AUTO_START, AUTO | sw 1, or `override` | MANUAL | |
| AUTO_START, AUTO | sw 2 | HOLD | |
| CTRL_STOP | standstill held `standstill_t` 0.5 s, or the time-out min(`ctrlstop_timeout` 5 s, v0/`a_ctrlstop` + `ctrlstop_t_extra` 1 s) | parking (CTRL_STOP) | drive off, `limits_hold` (`stop_ctl`) |
| CTRL_STOP entered with bit 16 set, or bit 16 set during its deceleration | none | parking (CTRL_STOP), at once | no regen deceleration (D74) |
| ESTOP.DECEL | standstill, `estop_tmax` 1.2 s, or bit 16 set | parking (ESTOP) | drive off, `limits_hold` |
| parking (CTRL_STOP) | `brakes_ok` | STANDBY | limits drop to 0 |
| parking (ESTOP) | `brakes_ok` | ESTOP.SAFE | limits drop to 0 |
| ESTOP.SAFE | a `reset_req` edge with bits 0 and 1 cleared and standstill | STANDBY | |
| FAULT | a `reset_req` edge with no class A or B and standstill | STANDBY | |

A new `arm_req` (arm switch off→on) is needed after every stop. This prevents unintended motion when the RC link comes back. After a stick override with the switch left at AUTO, the robot stays in MANUAL: AUTO_START needs the switch moved off AUTO and back.

**Decided, not built yet (card G2-MCU):**
- **D91, with D88.** After every switch-on, watchdog reset or brown-out, the robot waits in FAULT with its brakes held and the safety relay not re-armed, until the operator's deliberate reset and a new arm command.
  - The automatic `rearm_req` on SELFTEST entry goes (D30; `safety.rearm.auto_selftest` true today), and SELFTEST ends in FAULT instead of STANDBY.
  - The cause of the restart is shown and kept until the operator's reset.
  - Today a watchdog reset is shown only as the information flag `wdg_reset_seen` (MCU_STATUS flags b7). A brown-out is not detected, because only RCC_RSR IWDG1RSTF is read (§3.2). On the robot the restart clears the latches; in the twin they survive (R-36).
- **D85, with D90.** IMU_FAULT (bit 24) is class B in AUTO_START and AUTO, and latched until an operator reset with a valid IMU. In RC driving the robot limps at the lower of the quarter-speed derate and the descent cap of the 20 deg column (§2.7.4, §2.7.10).
- **R-35.** The arm edge and the reset are taken from valid frames only (§2.7.2).

#### 2.7.4 Command arbitration (blocks, `fw_vehicle_ctrl`, Task_10)
- RC path: `v_rc = thr·v_max_man·s`, `w_rc = steer·w_max_man·s` (`lim.v_max_man` 1.4 m/s, `w_max_man` 1.0 rad/s).
- AUTO path: `v_auto = sat(v_cmd, −v_rev_auto, v_max_auto)`, `w_auto = sat(w_cmd, ±w_max_auto)` (0.5 m/s, 1.2 m/s, 1.2 rad/s), taken only while the companion's command is valid and it asks for no soft stop (`soft_stop_req`); otherwise the AUTO target is 0.
- `Multiport Switch` on cmd_src: NONE → 0, RC → (v_rc, w_rc), AUTO → (v_auto, w_auto).
- Global derate `k_der ∈ [0,1]` multiplies the speed limit `v_track_max` (1.41 m/s), the turn-rate limit and the motoring torque limit. It is the minimum of the enclosure thermal derate (`aux.derate_bp` 65, 70, 80 °C → `derate_k` 1, 0.5, 0) and the class-C derates (`safety.derate_vec`: tilt and enclosure warnings 0.5, IMU fault 0.25). BMS warnings do not derate (D65): the BMS limits act through DCL / CCL in the torque limiter.
- Lateral-acceleration limit: `|w| ≤ min(k_der·w_max, a_lat_max / max(|v_ref|, v_lat_min))`, with `a_lat_max` 1.0 m/s² and `v_lat_min` 0.1 m/s, implemented with `Saturation Dynamic`.
- **Descent cap (D74, card G2-DSC).** After the mode's own limits, the speed target is limited to `ref_fwd` (forward) or `ref_rev` (reverse) at the measured downhill pitch and the fitted implement, in every mode, RC and the hardware E-stop included (`Descent_cap`, `fw_vehicle_ctrl`). `ref` = max(cap − `dv` 0.10 m/s, 0), where the cap is `P.structure.descent` (§6, `lim.descent`). The pitch is that of `fw_imu_tilt` (nose down forward, minus the pitch in reverse; the largest value of the last `t_hold` 1.0 s; 20 deg in both directions while the IMU sample is invalid, `lim.descent.theta_imu`). With IMU_FAULT set, RC driving therefore limps at the lower of the quarter-speed derate (0.25 × 1.41 = 0.35 m/s) and the target of the 20 deg column: about 0.35 m/s with no implement or the mower, and 0.15 m/s forward and 0.10 m/s in reverse with the full-tank sprayer. That is D90's limp speed; D85's latch and the stop in autonomous work are planned (§2.7.3). The implement is the DI IMPL_ID coding plug; an unknown implement (IMPL_FB of another type) takes the smallest entry of all rows. With the mower's deck reported raised (the lift potentiometer at or above `lift_raised`), the raised-deck rows apply. An entry at the top speed (1.41 m/s) does not limit the target. The tiller has no reverse: its reverse target is 0, the mode stays MANUAL and no fault or warning bit is raised for it (on the bench only bit 28 comes up, because the bench sends no IMPL_FB).

#### 2.7.5 Reference shaper (blocks, per axis v and w, Task_10, Ts = 10 ms)
- `a_dem = sat( K·(x_t − x_ref), −a_dn, +a_up )` (`Gain`, `Sum`, `Saturation Dynamic`) in MANUAL / HOLD / AUTO tracking, with `K_v` 4 1/s on the speed axis.
- **Revision B (D33): CTRL_STOP and ESTOP.DECEL use `a_dem = −sign(x_ref)·min(a_stop, sqrt(2·j_stop·|x_ref|))`** (Abs, Gain, Sqrt, MinMax, Sign, Product): constant deceleration without the proportional tail, jerk-limited approach to zero. The ESC ramp (2.24 m/s²) never limits it.
- `a_ref = RateLimiterDynamic(a_dem, ±j)`.
- `x_ref = DiscreteTimeIntegrator(a_ref)` (Forward Euler, external rising reset to the measured value).

The limits a_up/a_dn are direction-aware: "accelerate" when `e·x_ref > 0`, otherwise "decelerate". They are selected by the mode (`P.electronics.shaper`):

| Profile | a_acc (m/s²) | a_dec (m/s²) | jerk (m/s³) |
|---|---|---|---|
| MANUAL | 0.8 | 1.2 | 5 |
| AUTO | 0.5 | 1.0 | 5 |
| CTRL_STOP | n/a | 1.0 | 10 |
| ESTOP.DECEL | n/a | 2.0 | 10 |

Yaw axis: K_w = 5 1/s, α_max = 1.5 rad/s², jerk 8 rad/s³. The stop profile uses α = 3.0 rad/s². Revision B (D41): in AUTO the yaw axis uses `K_w_auto` 20 1/s, `alpha_auto` 3.0 rad/s², `jw_auto` 30 rad/s³ (above the GNC CommandShaping limits; safety envelope, about 0.05 s lag).

Design rule: `j ≥ K·a_max` avoids overshoot from the jerk-limiter lag (MANUAL: 4·1.2 = 4.8 ≤ 5). Feedforward: `a_ref` per axis.

#### 2.7.6 Skid-steer mixing (blocks)
Track speed references:
- `v_L = v_ref − w_ref·χB/2`, `v_R = v_ref + w_ref·χB/2`, with B = 0.60 m (`P.structure.geom.B`) and the effective-gauge (slip) factor χ = 1.39 (`P.structure.track.chi_nom`, D39), so χB/2 = 0.418 m.
- The inverse (used for telemetry) is `v = (v_R+v_L)/2`, `w = (v_R−v_L)/(χB)`.

Desaturation preserving curvature: `k = max(1, max(|v_L|,|v_R|)/v_track_max)`, then `v_{L,R} /= k`. The acceleration feedforward is mixed the same way: `a_{L,R} = a_v ∓ α·χB/2`.

**Yaw-rate loop (D2; commanded-standstill reset 2026-11-14, S4 stop 3).** The firmware mixes `w_mix = w_ref + u` in place of `w_ref`. `u` is a `Discrete PID Controller` (PI, Forward Euler, `yaw.Kp` 0.4, `yaw.Ki` 2.0 1/s, ±`yaw.u_max` 0.4 rad/s, clamping) on `w_ref − r`. `r` is gyro z re-biased in `Yaw_meas`: the bias is a first-order average (pole `exp(−Ts/yaw.tau_bias)`, τ = 2 s) in an enabled subsystem.
- **Commanded standstill** (subsystem `Cmd_still`): `v_target = w_target = 0` exactly (a zero command passes the arbitration unchanged), `|v_ref| < yaw.cmd_still_v` (0.005 m/s) and `|w_ref| < yaw.cmd_still_w` (0.005 rad/s); or the drive disabled, since the shapers then follow the measured speed and yaw rate.
- **PI reset:** the integrator is reset to 0 and `u = 0` while the drive is disabled, on an invalid IMU sample (`imu_valid` false; not only on IMU_FAULT), when `yaw.enable` is false, and at commanded standstill. It is a reset, not a bleed.
- **Bias average:** it runs only while the robot is both measured still (`max|v_track_L,R| < yaw.standstill_v`, 0.02 m/s) and commanded still. It takes the commanded standstill of the previous step (Unit Delay): `w_ref` follows `r` through the shaper reset, so the current value would close an algebraic loop.
- **Why:** the measured gate alone admits a slow pivot. In S4 every GNC pause left the PI holding a residual u (−0.023 to +0.033 rad/s at the pause ends), and the tracks pivoted under the gate (±0.0105 m/s in the long pause). The bias average absorbed the true 0.03 rad/s spin, `r` read 0 and the PI never unwound: 145° in one pause, 4.2 turns between 464 s and the end of the run at 1365 s. Test: T-EL-43 (§8.2).
- **`fw_imu_tilt`** (Task_5) still gates its own bias average on the measured standstill only. It has no commanded-standstill input, and its bias reaches only the telemetry `yaw_rate_meas`: `Yaw_meas` rebuilds the raw gyro from it, and CAN2 carries the raw gyro.

#### 2.7.7 Track speed measurement (`sens_odom_compute`, Task_10)
- The ESCs read the motor encoders and send their counts in DRV_FB3 over CAN1 (D3, D62). `sens_odom_compute` is a unit of the sensors domain that runs in Task_10 (sensors spec). It takes the count difference over `drv.odom_window` = 5 samples, as int32 with wrap-around and no saturation: `v_track = Δn·K_enc/(5·0.01 s)`.
- `K_enc = 2π·r_s/(CPR·N) = 2π·0.09/(4096·20) = 6.90e-6 m/count` (`P.electronics.drv.K_enc`). One count per 10 ms is 0.69 mm/s, and 2 m/s gives about 2900 counts in 10 ms.
- The unit also gives `a_track`, the filtered derivative of the mean track speed, for `fw_imu_tilt`.
- Plausibility: `|v_enc − ω_drv·r_s/N| > 0.3 m/s` for 0.2 s gives `ENC_x_MISMATCH` (class B; `drv.enc_mismatch_v`, `enc_mismatch_t`), where ω_drv is the ESC's FB1 speed.
  - Both values come from the motor encoder that the ESC reads. So the check catches a frame or computation error, not an encoder fault.
  - Where the robot's encoders are read is open (hazard log SF-18, card G1-04).

#### 2.7.8 Speed loop (in the ESCs, D2)
The MCU closes no speed loop. Each ESC runs its own 1 kHz speed loop in SPEED mode (D2, D50; drive spec 2.9). For each side, `fw_vehicle_ctrl` sends in DRV_CMD (`Bus_DriveCmd`, every 10 ms):
- the speed reference `w_ref = k_v2w·v_track_ref`, saturated at `drv.w_ref_max` (n_max, 377 rad/s), with `k_v2w = N/r_s` = 222 rad/m;
- the torque feedforward `tau_ff = Kff_a·a_track_ref`, saturated at ±T_peak:
  - `Kff_a = m_eq·r_s/(N·η_g)` = 0.703 N·m/(m/s²), with m_eq = m0/2 + J_m·N²/r_s² = 140.7 kg;
  - these values are for the 1 kW motor with no implement; `P.electronics.drv` follows the motor and the implement;
- the torque limits of §2.7.9, the enable (`drive_enable`), `park_req = ¬drive_enable` and `fault_reset = reset_req`.

#### 2.7.9 Torque limiter (Task_10, blocks)
Let ω_m be the motor speed (`v_track·N/r_s`) and ω* = max(|ω_m|, ω_min) with ω_min = 20 rad/s (`drv.w_min`).
- `P_dis = 0.95·V_pack·DCL − P_aux_est`; `P_chg = 0.95·V_pack·CCL + P_aux_est` (`tlim.dis_margin` 0.95, `tlim.P_aux_est` 120 W).
- `T_pow = (P_dis/2)·η_drv/ω*`; `T_regen = (P_chg/2)/(η_drv·ω*)` (`drv.eta_drv` 0.85).
- Motoring direction: `T_mot = min(T_peak, tau_avail, T_pow)·k_tr·k_der`, where `tau_avail` is the torque that the ESC reports available in FB2, with its own thermal and voltage derates (R-39: a lost FB2 frame zeroes it). Braking direction: `T_brk = min(T_peak, T_regen)`.
- `T_peak` = `P.drive.motor.T_peak`: 19.1 N·m for the 1 kW default motor, 14.32 N·m for the as-built 750 W motor (D95).
- If ω_m ≥ 0: `T_hi = T_mot`, `T_lo = −T_brk`. Otherwise the two are swapped. The limits are 0 unless `drive_enable` OR `limits_hold` is true; the soft-stop parking holds them until both brakes confirm (§2.7.3).

Implemented with `MinMax`, `Divide`, `Switch` and `Product`.

**Flashing condition (D95).** No firmware built from the twin goes on the robot until its torque limits match the motors fitted: `T_peak` 14.32 N·m per side for the 750 W motors, not the 1 kW unit's 19.1 (cards G2-MCU and B-00).

**Traction limit** (only when `v_ground_valid` from the companion):
- `s_i = (v_track_i − v_body_i)/max(|v_track_i|, 0.2)`, where `v_body_{L,R} = v_ground ∓ gyr_z·χB/2`.
- A `Relay` with on 0.30 / off 0.20 sets the target k_tr to 0.5 or 1.
- A `Rate Limiter` moves k_tr down at 2 /s and up at 1 /s.

#### 2.7.10 Safety supervisor (`fw_safety_supervisor`, Task_10, blocks)
The supervisor uses a vectorized persistence/latch pattern:
- `cond[32]` → per-element persistence timer in integer 10 ms ticks (`Sum` + `Unit Delay` + `Switch`, reset when the condition is false) → `≥ t_persist[32]` (`P.electronics.safety.t_persist`).
- A set/reset latch for the latched entries (`safety.latch_mask`): `Logical Operator` + `Unit Delay`, reset by `reset_req ∧ ¬cond`.
- `fault_word` is packed with `Bit to Integer Converter` (nbits 32, LSB first, uint32).
- `fault_A = OR(active & maskA)` and `fault_B = OR(active & maskB)` use a single-input `Logical Operator` (vector reduction).
- `k_der` = the minimum of `safety.derate_vec` over the active bits and the enclosure thermal derate (§2.7.4).

The table matches the normative one in ICD section 6 bit by bit (`bitsA`, `bitsB`, `bitsC`, `bitsL` and `tp` in `params/param_electronics.m`).

| Bit | Name | Detection | t_persist | Class | Latched |
|---|---|---|---|---|---|
| 0 | HW_ESTOP | DI ESTOP_CH1_OK or CH2_OK low (no debounce on assertion) | 0 | A (the hardware chain acts) | yes |
| 1 | REMOTE_ESTOP | RC CH12 = STOP, evaluated only on frames with failsafe = 0 and frame_lost = 0 (held otherwise), no channel debounce (D34) | 0.04 s | ESTOP state | yes |
| 2 | RC_LINK_LOST | rc_age > 0.10 s | 0 | B | no |
| 3 | RC_FAILSAFE | SBUS failsafe flag | 0 | B | no |
| 4 | COMP_TIMEOUT | COMP_CMD sequence number unchanged for 0.20 s (`safety.comp_timeout`), in AUTO_START and AUTO only | 0 | B | no |
| 5/6 | DRV_L/R_TIMEOUT | FB1 or FB3 age > 50 ms (`safety.drv_timeout`) | 0 | A | yes |
| 7/8 | DRV_L/R_FAULT | ESC state FAULT or fault_code ≠ 0 (bit 10 STO excluded) | 0 | A | yes |
| 9 | BMS_TIMEOUT | any BMS frame age > 0.5 s (`safety.bms_timeout`) | 0 | B | yes |
| 10 | BMS_ALARM | BMS fault_level == 3 | 0 | B | yes |
| 11 | BMS_WARN | warning word ≠ 0 or DCL < 25 A (`dcl_warn`) | 0 | info (no derate; BMS limits act through DCL / CCL, §2.7.9; D65) | no |
| 12/13 | ENC_L/R_MISMATCH | abs(v_enc − ω_m·r_s/N) > 0.3 m/s (§2.7.7) | 0.2 s | B | yes |
| 14 | TILT_WARN | max(abs(roll), abs(pitch)) > 25° (level-calibrated estimate), only while bit 24 is clear (previous sample) | 0.5 s, held (not restarted) on an invalid IMU sample | C (derate 0.5) | no |
| 15 | TILT_STOP | > 28°, only while bit 24 is clear (previous sample) | 0.5 s, held (not restarted) on an invalid IMU sample | B | yes |
| 16 | OVERSPEED | (a) abs(v_track) > 1.70 m/s (`lim.v_overspeed`); (b) **descent trip** (D74): drive on (modes MANUAL to ESTOP) and odometry valid, the predicted body speed abs(v) + max(a_track·sign(v), 0)·`t_pred` (0.08 s today; 0.09 s decided for REQ-SYS-031, pending on card G2-MCU; provisional under D96: 0.12 s, with `t_pred` to match, if the delivered ESC needs up to 0.10 s, on cards G1-04, G2-MCU and G2-DRV) above the descent cap of the measured pitch (1.44 m/s at a top-speed entry), and the speed reference steady or rising, or the speed 0.2 m/s above a falling reference (falling: fell by more than `a_fall` 0.1 m/s² × Ts in one sample, or more than `dv_fall` 0.02 m/s above the cap's speed target); (c) **speed-follow term**, in the driving modes only (MANUAL, HOLD, AUTO_START, AUTO; previous sample's mode): on either track abs(v_track) above the held reference rf = max(abs(vtrack_ref), min(rf_prev, abs(v_track))) by more than 0.2 m/s (`lim.v_follow_dv`); odometry valid. **Hill start** (D74): while a brake is engaged, and for `lim.descent.t_release` 0.5 s [EST] after both brakes release (`drive_fb` brake_released), with the reference in the direction of travel at most `yaw.cmd_still_v`, the `t_pred` term of (b) is dropped; (a) and the trip on the measured speed stay | (a) and (c) 0.2 s; (b) 0 (same sample) | B | yes |
| 17 | HV_LOST | BMS HV_Ready false while ACTIVE | 0.2 s | B | yes |
| 18 | HV_OPENING | BMS HV_Opening_Warn | 0 | B | no |
| 19 | RAIL_UV | V24 < 21.6, V12 < 11.0 or V5 < 4.6 V | 0.2 s | B | no |
| 20 | ENCL_TEMP_WARN | T_encl > 70 °C | 5 s | C (derate 0.5) | no |
| 21 | ENCL_TEMP_CRIT | T_encl > 80 °C | 5 s | B | yes |
| 22 | STOP_FAIL | stop monitor (`stop_fail`): (a) in CTRL_STOP, ESTOP.DECEL or the ESTOP park, abs(v) > v0 + 0.1 m/s, or from 0.5 s after entry abs(v) > max(v0 − 0.5·a·t, 0.05 m/s), with a = 1.0 or 2.0 m/s²; (b) parking: no brake confirmation 0.5 s after park_req; (c) motion above 0.05 m/s for 0.2 s, in STANDBY from entry, in ESTOP.SAFE and FAULT only after standstill was seen there | 0 | A (in maskDO0) | yes |
| 23 | TASK_OVERRUN | 3 consecutive overruns (on the robot the count is not read yet, R-08) | 0 | B, and in maskDO0: DO0 drops and the relay removes STO at once; bit 31 then gives FAULT (§2.7.1) | yes |
| 24 | IMU_FAULT | IMU invalid (`sens_acq_imu`: b6 (SIM), or abs(acc) outside [5, 15] m/s², or all 9 raw axes unchanged for `safety.imu_frozen_t` 0.1 s) | 0.5 s | C (derate 0.25, yaw loop off; the yaw PI resets on each invalid sample). Decided, planned on card G2-MCU: B in AUTO_START and AUTO (D85), C with D90's limp in RC driving | no; decided: latched until an operator reset with a valid IMU (D85) |
| 25 | CAN1_BUSOFF | FDCAN1_PSR.BO (HW) / `fault_inject` b4 (SIM) | 0 | A | yes |
| 26 | CAN2_BUSOFF | FDCAN2_PSR.BO (HW) / `fault_inject` b5 (SIM) | 0 | B | yes |
| 27 | SAFETY_CHAIN_DISCREPANCY | ESTOP_CH1_OK ≠ CH2_OK | 0.05 s | A | yes |
| 28 | IMPL_FAULT | a fitted implement (DI IMPL_ID ≠ 0) whose IMPL_FB reports a fault or is older than 0.2 s (`safety.impl_timeout`); a lost IMPL_FB is therefore detected after about 0.4 s, its 0.2 s age plus the 0.2 s persistence (REQ-SYS-101) | 0.2 s | C (implement off) | yes |
| 29 | BUMPER | DI bumper, front or rear | 0 | A | yes |
| 30 | LID_OPEN | DI LID_CLOSED low | 1 s | C (AUTO inhibit) | no |
| 31 | SAFETY_RELAY_OPEN | DI SAFETY_OK low (STO active) while ACTIVE (MANUAL, HOLD, AUTO_START, AUTO, CTRL_STOP); in BOOT, SELFTEST, STANDBY, ESTOP and FAULT an open relay only inhibits arming | 0 | A | no |

Class A bits 0, 27, 29 and 31 are consequences of the hardware chain: they stop the robot but are left out of `maskDO0` (bits 5-8, 22, 23, 25), otherwise the loop could never close again (D30, D71).

Tilt is estimated from `Complementary Filter` (accelerometer gain 0.02, no magnetometer, rotation-matrix output, ENU):
- `roll = atan2(R32, R33)`, `pitch = −asin(R31)`, using `Selector` + `Trigonometric Function`.
- With no implement, the robot tips over statically at about 48° of roll: atan((B/2)/h_cg) = atan(0.30/0.267), from `P.structure.geom.B` and `P.structure.lofi.h_cg`. The 28° stop limit leaves about 20°. The structure spec gives the tip angles with each implement.

#### 2.7.11 Watchdog manager (`fw_watchdog_mgr`, Task_1k)
- Task_10, Task_50 and Task_100 each increment an alive counter (`Counter Free-Running`, uint8).
- These pass through Rate Transitions to 1 kHz, then `Detect Change` → age (ms).
- `wdg_kick = (age10 < 20 ms) ∧ (age50 < 60 ms) ∧ (age100 < 200 ms) ∧ ¬inject_stall` (`wdg.age_lim` [0.02 0.06 0.2] s).
  - A hang of a faster task starves these tasks, so it stops the kick too.
  - A stall of Task_50 stops the kick within 0.11 s (T-EL-34).
- HW: an Enabled Subsystem with `Register Read/Write` writes `0xAAAA` to IWDG1->KR. IWDG1 is started in CubeMX with PR = /4 and RLR = 399, giving 50 ms (`wdg.timeout`).
- DO0 MCU_SAFETY_OK = watchdog healthy AND NOT (fault_word AND `maskDO0`) AND mode ≠ BOOT (D30).
- DO1 SAFETY_RESET_REQ is a 50 ms pulse (`safety.rearm.t_pulse`) on `rearm_req`, sent once DO0 has been high for 20 ms (`t_do0_settle`). D91 removes the automatic pulse at SELFTEST entry (planned, card G2-MCU).
- SIM: `fault_inject` b0, b1 and b2 freeze the alive counters of Task_10, Task_50 and Task_100, and b7 stops the 1 kHz kick (`wdg.stall_1k_bit`).

#### 2.7.12 Auxiliary outputs (`fw_implement_aux`, Task_50)
- **Beacon** (`aux.blink_f` 1 Hz, `aux.blink_fast` 2 Hz):

  | Mode | Beacon |
  |---|---|
  | AUTO | green steady |
  | AUTO_START | green blinking 2 Hz |
  | MANUAL, HOLD | amber steady |
  | STANDBY | amber blinking 1 Hz |
  | CTRL_STOP | amber blinking 2 Hz |
  | ESTOP | red steady |
  | FAULT | red blinking 2 Hz |
  | SELFTEST | all three (lamp test) |

- **Buzzer:** continuous for the 3 s of AUTO_START (the start warning, `safety.auto_start_warn_t`). While moving in AUTO (|v| > `aux.moving_v` 0.05 m/s), a 0.1 s beep every 2 s (`aux.buzz_on`, `aux.buzz_period`). In AUTO the companion can also sound it (`buzzer_cmd`).
- **Fans:** `duty = LUT(T_encl: [35 40 55] °C → [0 0.2 1.0])` (`1-D Lookup Table`).
- **Implement:**
  - The rotor (mower or tiller) runs only when three things hold: a rotor implement is fitted, the implement switch is ≥ ON and there is no implement fault. It also needs the mode MANUAL, or AUTO with the companion's mower or tiller command. It is off in every other mode, AUTO_START, HOLD and the stops included.
  - **Decided, not built yet (D92; card G2-MCU for RC driving, card G2-GNC for missions):** after a protective stop of the blades, the blades restart only on the operator's deliberate command. A protective stop is an E-stop, a hold or pause, a tilt stop, a fault or a raised deck. Today in RC driving, the rotor starts again with the arm command if the implement switch was left on. No blade stop on a raised deck exists.
  - Spray nozzles are on only if |v| > 0.2 m/s. The mask comes from the companion (AUTO, position 3) or is all-on (MANUAL, position 2).
  - Pump duty: 0.6 in MANUAL, or the companion's value in AUTO.
  - Lift: ±0.6 duty from the 3-position switch, cut off at pot limits [0.02, 0.98].

### 2.8 CAN networks

The payloads (byte offsets, types, resolutions) are those of ICD sections 4 and 5, which bind. Encoding everywhere is `Byte Pack` / `Byte Unpack` with scaling and rounding (no DBC); the same codec subsystems serve the producer, the consumer and the HW variant. Under D96 both sections are provisional: the ESC and BMS frames are the twin's, and the delivered ESCs' and BMS's protocols are unknown (R-14, R-15, card G1-04).

#### 2.8.1 CAN1 (classic CAN 2.0B, 500 kbit/s, 11-bit IDs, little-endian; ICD 4)
| ID | Name | Producer → consumer | Period | Parameter |
|---|---|---|---|---|
| 0x201 / 0x202 | DRV_CMD_L / _R (enable, park_req, fault_reset, mode; w_ref; tau_ff; tau_lim_mot, tau_lim_reg) | MCU → ESC node 1 / 2 | 10 ms | `can.id.drv_cmd` 0x200 + node |
| 0x181 / 0x182 | DRV_FB1_L / _R | ESC → MCU | 10 ms | `can.id.drv_fb1` 0x180 + node |
| 0x281 / 0x282 | DRV_FB2_L / _R | ESC → MCU | 50 ms | `can.id.drv_fb2` 0x280 + node |
| 0x381 / 0x382 | DRV_FB3_L / _R (encoder counts, fault code, state) | ESC → MCU | 10 ms | `can.id.drv_fb3` 0x380 + node |
| 0x140 | IMPL_CMD | MCU → implement ECU | 50 ms | `can.id.impl_cmd` |
| 0x141 | IMPL_FB (status, implement type) | implement ECU → MCU | 50 ms | `can.id.impl_fb` |
| 0x3D0 | BMS_REQ | MCU → BMS | 50 ms | `can.id.bms_req` |
| 0x351, 0x355, 0x356, 0x35A, 0x373 | BMS_LIMITS, BMS_SOC, BMS_VIT, BMS_ALARM, BMS_CELLS | BMS → MCU | 100 ms | `can.id.bms_limits`, `bms_soc`, `bms_vit`, `bms_alarm`, `bms_cells` |
| 0x3C0, 0x3C1 | BMS_STATE, BMS_POWER | BMS → MCU | 100 ms | `can.id.bms_state`, `bms_power` |

The frames 0x351 to 0x373 follow the widely used Pylontech/Victron-style BMS CAN protocol; 0x3C0, 0x3C1 and 0x3D0 are the twin's own (D4, D5).

Time-outs (ICD 4): the ESC goes to QUICK_STOP after 100 ms without a new DRV_CMD alive value; the MCU sets `rx_ok` false and DRV_x_TIMEOUT after 50 ms without FB1 or FB3 (`safety.drv_timeout`), BMS_TIMEOUT after 0.5 s (`safety.bms_timeout`) and IMPL_FAULT about 0.4 s after the last IMPL_FB (the 0.2 s age `safety.impl_timeout` plus the 0.2 s persistence of bit 28, `safety.t_persist`; REQ-SYS-101); the BMS treats HV_Enable_Req as false after 0.5 s of a frozen BMS_REQ alive value; the implement ECU stops its rotor after 0.2 s without IMPL_CMD.

Bus load: 770 frames/s (DRV_CMD, FB1 and FB3 at 100/s per side, FB2 at 20/s per side, the implement and BMS_REQ frames at 20/s, the seven BMS frames at 10/s) × about 135 bit with worst-case stuffing = 104 kbit/s, **about 21 %** at 500 kbit/s (target < 40 %, T-EL-44).

#### 2.8.2 CAN2 (CAN-FD, 1 Mbit/s arbitration / 2 Mbit/s data with BRS; little-endian; ICD 5)
| ID | Name | Producer → consumer | Period | Length (B) | Parameter |
|---|---|---|---|---|---|
| 0x100 | COMP_CMD | companion → MCU | 50 ms | 32 | `can.id.comp_cmd`, `can.len.comp_cmd` |
| 0x200 | MCU_STATUS | MCU → companion | 50 ms | 48 | `can.id.mcu_status`, `can.len.mcu_status` |
| 0x201 | MCU_IMU | MCU → companion | 5 ms | 48 | `can.id.mcu_imu`, `can.len.mcu_imu` |
| 0x202 | MCU_ODOM | MCU → companion | 10 ms | 32 | `can.id.mcu_odom`, `can.len.mcu_odom` |
| 0x203 | MCU_POWER | MCU → companion | 100 ms | 64 | `can.id.mcu_power`, `can.len.mcu_power` |

Time-outs (ICD 5): the MCU raises COMP_TIMEOUT when the COMP_CMD `seq` has not changed for 0.2 s (`safety.comp_timeout`); the companion marks `Bus_McuStatus.valid` false after 0.2 s without MCU_STATUS (`can.mcu_status_timeout`).

Bus load: about 9 % (estimate from the ICD 5 frames with worst-case stuffing, about 90 ms of bus time per second; MCU_IMU at 200 frames/s is about 6 %). The companion side is a SocketCAN-FD bridge node (`can0`, 1 M / 2 M BRS) that republishes every frame on `/can2/rx` and sends `/can2/tx` (69-byte messages: id, length, 64 data bytes; `can.hw.msg_len`), D38. In SIM, `Link_MCU_GNC` carries the frames (§2.9.6).

### 2.9 Plant-side electronics models

The electronics plant `Electronics_Plant` (inside `Shaddad_Plant`, no physical ports; ICD 3.2) holds the front end, the actuators, the LV demand, the enclosure thermal network, the electronics environment and the watchdog-reset emulation (§7.4). The CAN1 network and the CAN2 link are top-level components (§2.9.6).

#### 2.9.1 Safety chain (the power domain's; revision A's `Safety_Chain_HW` is not built)
The hardware safety chain is `Power_Plant/Safety_Chain` (power spec 2.5): the SR latch of the safety relay on the NC loop of the E-stops, the bumpers and DO0 MCU_SAFETY_OK; set by the DO1 SAFETY_RESET_REQ pulse with the loop closed; its immediate contacts to STO and K3, its delayed contacts to K1 and K2 after `P.power.safety.t_cat1` 1.5 s. The electronics front end (§2.9.5) returns its feedback in the DI word: ESTOP_CH1_OK and CH2_OK (either can be stuck by fault injection), SAFETY_OK and SAFETY_HV_OK (ICD 6).

#### 2.9.2 Watchdog/reset emulation (`elp_watchdog_reset`)
- If no `wdg_kick` arrives for > 50 ms (`wdg.timeout`), `mcu_reset` is asserted for t_boot = 0.3 s (`wdg.t_boot`).
- While in reset, the CAN1 and CAN2 link models drop the MCU's frames (`elec_hw.mcu_reset`, ICD 5) and `elp_actuators` forces DO = 0 and PWM = 0.
- The reset flag pulses (`Bus_McuIn1k`, ICD 6) and the firmware state returns to BOOT.
- **Decided, not built yet (D88; pending on card G2-MCU):** after a reset the twin models the reboot of the companion computer and the 3-D LiDAR, with DO8 and DO9 switched by the MCU. Today the reset only zeroes DO and PWM for t_boot 0.3 s; nothing models a reboot time (the 3-D LiDAR's sensor power only follows DO9 and the 24 V rail). The twin has no brown-out emulation either (a LV dip with the BOR flag, REQ-SYS-079), which card G2-MCU adds with the reset-cause reading of D88.

#### 2.9.3 Electronics LV demand (`elp_lv_demand`)
- The demand per rail (`Bus_LV_LoadDemand`: 24 V, 12 V, 5 V, 3.3 V, every 10 ms) is computed from the DO and PWM states and `P.electronics.load` (§2.4).
- The power domain's LV distribution (`LV_Distribution`, power spec 3.5) converts it with its DC/DC converters and LDO (`P.power.lv`) and draws it from the 48 V bus in its Simscape network. Revision A's `Dynamic Load` and `DC-DC Converter` blocks in the electronics plant (Variant `ELEC_LOAD_MODEL`) are not built.

#### 2.9.4 Enclosure thermal (Simscape thermal, own network, local solver)

`C·dT/dt = Q_int + α_s·A_s·G_solar − h(fan)·A·(T − T_amb)`

| Quantity | Value |
|---|---|
| C | 12 kg × 1000 J/(kg·K) = 12 kJ/K |
| A | 1.2 m² |
| h | 4 W/(m²K) natural (`therm.h_nat`: laminar natural convection on the sealed box, 3.8 W/(m²K) at ΔT 20 K, no radiation credit); h(fan) = 4 + 12·fan_duty (`therm.h_fan`) |
| α_s·A_s | 0.3 × 0.24 m² (`therm.alpha_s`, `therm.A_s`) |
| Q_int | the "in box" loads (§2.4): the MCU 3 W, the Jetson 12 W and the in-box losses 15 W (`load.P_inbox_extra`), about 30 W (25-40 W over the modes) |

Blocks: `Thermal Mass`, `Convective Heat Transfer` (variable coefficient through PS input), `Controlled Heat Flow Rate Source`, `Controlled Temperature Source` (T_amb), `Temperature Sensor`, `Thermal Reference`, `Solver Configuration` (local Backward Euler, 0.1 s, `therm.Ts`). The MCU and IMU temperatures are taken as T_encl + 15 K and + 5 K (`therm.dT_mcu`, `therm.dT_imu`).

Expected steady state at 50 °C ambient and 1000 W/m² sun (Q = 30 + 72 = 102 W): 55.3 °C with both fans at 1.0 (h 16), 71.3 °C with the fans disabled (h 4; `params/param_electronics.m`). τ = C/(h·A): about 10 min with the fans at 1.0, about 42 min without. T-EL-17 checks the first below 62 °C. This is the development target's box (D96); the robot's box (6 kg [EST], §2.2) is not modelled. **The area is not settled (§2.2):** if the box is the plain 0.6 × 0.4 × 0.25 m of revision A (0.98 m², not 1.2 m²), the steady states become about 56.5 °C with the fans and 76 °C without, only 4 K below the 80 °C stop (`safety.T_encl_crit`). A change of `therm.area` is a parameter change for card G2-VAL, which then recomputes these figures and T-EL-17's margin.

#### 2.9.5 ADC / DI front end (`elp_hw_frontend`)
- **ADC:** `counts = sat(round((V/k)/V_ref·65535 + n), 0, 65535)`, with n ~ N(0, 4 LSB) from a seeded `Random Number` (`adc.noise_lsb`, `adc.seed`). Divider ratios: k_vpack = k_vlink = 21 (Vpack, Vbus), k_v24 = 11, k_v12 = 5.7, k_v5 = 2 (`P.electronics.adc`).
- **Current:** `V = 1.65 + 4.1667e-3·I`.
- **NTC:** `R = R25·exp(B·(1/T − 1/298.15))`, `V = 3.3·R/(R + R_pu)`. The firmware inverts this with a 33-point `1-D Lookup Table` (counts → K, `adc.ntc_lut_counts`, `adc.ntc_lut_K`) precomputed in `param_electronics`.
- **Lift potentiometer:** 0 to 3.3 V for 0 to 1, limits [0.02 0.98] (`adc.liftpot_lim`).
- **Encoders:** none. Since architecture D3 the ESCs read the encoders and send the counts in DRV_FB3.
- **DI word:** bit-packs `Bus_HmiInputs`, the safety-chain feedback, the K1/K2 auxiliary contacts, the implement coding plug, the lid, the key and the dock (14 bits, §5.4, ICD 6). It reaches the MCU through a 1 ms Unit Delay (D36).

#### 2.9.6 CAN1 network and CAN2 link (`CAN1_Net_Sim`, `Link_MCU_GNC`, `companion_io`)
- **CAN1 (`CAN1_Net`, SSR `CAN1_Net_Sim`).** Delivers the CAN1 frames between the MCU, the ESCs, the implement ECU and the BMS on the same buses and sample times (ICD 3.1), with no added latency (`can.can1_latency_samples` 0). Frames are dropped by the drop mask of `Bus_FaultInject` (the bits `P.electronics.can.drop_bit`, ICD 10); bus-off is injected and reset (T-EL plant test `testCan1DropMaskBusoffReset`). The codecs sit at each end, in the firmware and in the plant's ESC, BMS and implement models.
- **CAN2 (`CAN2_Link`, SSR `Link_MCU_GNC`).** Delays MCU_STATUS by one 50 ms sample (`can.can2_status_delay` 1, the MCU-GNC loop breaker, D41) and passes COMP_CMD with no delay (`can.can2_cmd_delay` 0); it drops all frames on `flt.can2_loss`, `flt.can2_busoff` or `elec_hw.mcu_reset` (ICD 5; `testLinkDelayAndDrops`).
- **The companion's side (`companion_io`, `companion_io_ros2`).** In SIM the frames pass as `Bus_CanFrame64`. In HW the ROS 2 Subscribe and Publish blocks (D38) take `/can2/rx` every 1 ms and keep the newest message, relying on the bridge to space messages by at least 2 ms (`can.hw.min_gap`, QoS depth 16 `can.hw.qos_depth`). Review ELC-07: under jitter that path can drop frames that the simulated link never drops.

---

## 3. Exact block list (verified in the live R2026b session)

The paths below are exact `add_block` source paths. In names, `//` stands for a literal "/". Paths that contain a newline are given as `sprintf(...)`. Parameter names are the real mask/dialog parameter names read with `get_param(...,'DialogParameters')` or `'MaskNames'`.

### 3.1 Firmware model `Shaddad_MCU` (common logic, code-generated)
| Block | Library path | Key parameters to set |
|---|---|---|
| Mode manager | `simulink/User-Defined Functions/MATLAB Function` | calls `mcu_mode_manager.m` (§2.7.3), sample time `Ts.ctrl`; constants arrive on block inputs, with no parameter data; Stateflow is not installed |
| SBUS frame sync | `simulink/User-Defined Functions/MATLAB Function` | see §4.1 |
| Byte → bits | `commutil2/Integer to Bit Converter` | `nbits=8`, `bitOrder='LSB first'`, `signedInputValues=off`, `outDtype='boolean'` |
| Bits → 11-bit channels | `commutil2/Bit to Integer Converter` | `nbits=11`, `bitOrder='LSB first'`, `signedOutputValues=off`, `outDtype='uint16'` |
| Flags byte → bits | `commutil2/Integer to Bit Converter` | `nbits=8`, `bitOrder='LSB first'`, `outDtype='boolean'` |
| Fault word pack | `commutil2/Bit to Integer Converter` | `nbits=32`, `bitOrder='LSB first'`, `outDtype='uint32'` |
| Yaw-rate PI | `simulink/Discrete/Discrete PID Controller` | `Controller='PI'`, `SampleTime` `Ts.ctrl`, `P='P.electronics.yaw.Kp'`, `I='P.electronics.yaw.Ki'`, `IntegratorMethod='Forward Euler'`, `LimitOutput='on'`, `UpperSaturationLimit='P.electronics.yaw.u_max'`, `LowerSaturationLimit='-P.electronics.yaw.u_max'`, `AntiWindupMode='clamping'`, `ExternalReset='level'` (§2.7.6). The MCU has no speed PI: the ESCs close the speed loop (D2, §2.7.8). |
| Accel/jerk limiter | `simulink/Discontinuities/Rate Limiter Dynamic` | ports `up`, `u`, `lo` |
| Rate-limited trim / k_tr | `simulink/Discontinuities/Rate Limiter` | `RisingSlewLimit`, `FallingSlewLimit`, `SampleTimeMode='inherited'`, `InitialCondition='1'` |
| Dynamic limits | `simulink/Discontinuities/Saturation Dynamic` | ports `up`, `u`, `lo` |
| Static limits | `simulink/Discontinuities/Saturation` | `UpperLimit`, `LowerLimit` |
| Deadband | `simulink/Discontinuities/Dead Zone` | `LowerValue='-P.electronics.rcmap.deadband'`, `UpperValue='P.electronics.rcmap.deadband'` |
| Expo | `simulink/Math Operations/Polynomial` | `Coefs='[e 0 1-e 0]'` |
| Shaper integrator | `simulink/Discrete/Discrete-Time Integrator` | `IntegratorMethod='Integration: Forward Euler'`, `gainval='1.0'`, `ExternalReset='rising'`, `InitialConditionSource='external'`, `SampleTime='P.electronics.ts.ctrl'`, `LimitOutput='on'` |
| Encoder count window | `simulink/Discrete/Delay` (length `drv.odom_window`) + `Data Type Conversion` (stored integer) | in `sens_odom_compute` (sensors domain, Task_10; §2.7.7): int32 difference with wrap-around, no saturation |
| Bias averages (yaw gyro, tilt) | `simulink/Discrete/Transfer Fcn First Order` in an Enabled Subsystem | `PoleZ='exp(-Ts.ctrl/P.electronics.yaw.tau_bias)'` (yaw, §2.7.6); `exp(-Ts.imu/P.electronics.yaw.tau_bias)` (`fw_imu_tilt`) |
| Traction slip relay | `simulink/Discontinuities/Relay` | `OnSwitchValue='P.electronics.trac.s_on'`, `OffSwitchValue` `trac.s_off` (§2.7.9) |
| Switch debounce | `simulink/Discrete/Tapped Delay` | `NumDelays='P.electronics.rcmap.debounce_n-1'`, `includeCurrent='on'` |
| Rate crossings | `simulink/Signal Attributes/Rate Transition` | `Integrity='on'`, `Deterministic='on'`, `OutPortSampleTime` explicit |
| Tilt filter | `mspfiltlib/Complementary Filter` | `OrientationFormat='Rotation matrix'`, `AccelerometerGain='0.02'`, `HasMagnetometer='off'`, `DispRefFrame='ENU'`, `SimulateUsing='Code generation'` |
| CAN1 and CAN2 encode | `mcblib/Signal Management/Byte Pack` | `byteAlign='1'`, with the scaling in front; no DBC (architecture D6). Each frame family's owner builds its codec units (ICD 4 and 5): the drive's for DRV_CMD; `build_can2_codecs.m` for the CAN2 frames MCU_IMU, MCU_ODOM, MCU_STATUS and MCU_POWER |
| CAN1 and CAN2 decode | `mcblib/Signal Management/Byte Unpack` | `byteAlign='1'`, with the scaling behind; no DBC (D6). The drive's units for DRV_FB1 to FB3; `build_can2_codecs.m` for COMP_CMD |
| Thermal derate, fan, NTC | `simulink/Lookup Tables/1-D Lookup Table` | breakpoints/table from `P.electronics` |
| Mode decode | `simulink/Logic and Bit Operations/Compare To Constant` | |
| Bit gating | `simulink/Logic and Bit Operations/Bitwise Operator`, `.../Extract Bits` | `logicop`, `BitMask` / `bitsToExtract`, `bitIdxRange` |
| Edge detect | `simulink/Logic and Bit Operations/Detect Rise Positive`, `.../Detect Change` | |
| Alive counters | `simulink/Sources/Counter Limited` (`uplimit='15'`), `simulink/Sources/Counter Free-Running` | |
| Arbitration | `simulink/Signal Routing/Multiport Switch`, `simulink/Signal Routing/Switch` | `DataPortOrder='Zero-based contiguous'` |
| Generic math | `simulink/Math Operations/{Gain, Sum, Product, Divide, Abs, Sign, MinMax, Trigonometric Function, Dot Product}` | |
| Generic routing | `simulink/Signal Routing/{Selector, Bus Creator, Bus Selector, Vector Concatenate}`, `simulink/Discrete/{Unit Delay, Delay, Memory}` | |
| Message latch | `simulink/Ports & Subsystems/Enabled Subsystem` | outputs "held" |
| Variant I/O | `simulink/Ports & Subsystems/Variant Subsystem` | choices `SIM` (`VC_TARGET==0`) / `HW` (`VC_TARGET==1`); `VariantActivationTime='update diagram'` |
| Unit-testable pieces | `simulink/Ports & Subsystems/Subsystem Reference` | `ReferencedSubsystem='fw_xxx'` |
| Root ports | `simulink/Sources/In Bus Element`, `simulink/Sinks/Out Bus Element` (or Inport/Outport with bus type) | explicit sample times |

### 3.2 Firmware HW variant (STM32 driver blocks, only in `fw_io_in_hw` / `fw_io_out_hw`)
| Block | Library path | Key parameters |
|---|---|---|
| SBUS UART | `stm32h7xxblockslib/UART//USART Read` | `UARTModule='USART2'`, `DataType='uint8'`, `DataLength='32'`, `BlockingMode='off'`, `OutputLength='on'`, `OutputStatus='on'`, `SampleTime='1e-3'` |
| DI | `stm32h7xxblockslib/Digital Port Read` | `PortName='GPIOF'`, `PinNumber='[0:13]'` (`P.electronics.hw.di_pins` 14), `IOAsArray='on'`, `SampleTime='Ts.base'`; packed into the uint16 `di_word` |
| DO | `stm32h7xxblockslib/Digital Port Write` (×2) | `PortName='GPIOG'`, `PinNumber='[0:15]'`; `PortName='GPIOE'`, `PinNumber='[0:3]'`; `IOAsArray='on'` |
| ADC | `stm32h7xxblockslib/Analog to Digital Converter` | `ADCModule='ADC1'`, `ConversionGroup='Regular'`, `TriggerMode='Trigger and read'`, `NumberOfConversions='8'`, `SampleTime='Ts.comm'` (50 ms); second instance `ADC3`, 1 conv (TempSensor), `Ts.comm` |
| PWM pump and lift | `stm32h7xxblockslib/PWM Output` | `TimerModule='TIM1'` (CH1 pump, CH2 lift), `DutycycleUnits='Percentage'`, `EnableChannel1='on'`, `EnableChannel2='on'`, `SampleTime='Ts.ctrl'` (20 kHz set in CubeMX) |
| PWM fans | `stm32h7xxblockslib/PWM Output` | `TimerModule='TIM4'`, channels 1-2, `SampleTime='0.1'` (25 kHz in CubeMX) |
| CAN1 RX | `stm32h7xxblockslib/FDCAN Read` inside `simulink/Ports & Subsystems/For Iterator Subsystem` (at most `hw.can1_rx_iter` 16 iterations, exit on status "no data") | `FDCANModule='FDCAN1'`, `BlockMode='Receive'`, `OperationReadMode='Data'`, `CANFrameType='CAN'`, `ReadSource='FIFO 0'`, `OutputTypeReadMode='Unpacked'`, `OutputStatus='on'`, `SampleTime='-1'` (the iterator's task rate). ID-dispatch to slots with `Compare To Constant` + Enabled Subsystems |
| CAN1 TX | `stm32h7xxblockslib/FDCAN Write` (one per frame, in an Enabled Subsystem on `valid`) | `FDCANModule='FDCAN1'`, `BlockMode='Transmit'`, `CANFrameType='CAN'`, `DataFormatWriteMode='Raw data'`; frames DRV_CMD_L / _R, IMPL_CMD and BMS_REQ, 8 bytes |
| CAN2 RX | `stm32h7xxblockslib/FDCAN Read` in For Iterator (at most `hw.can2_rx_iter` 8 iterations) | `FDCANModule='FDCAN2'`, `CANFrameType='CAN-FD'`, `ReadSource='FIFO 0'`, `OutputTypeReadMode='Unpacked'`, `OutputStatus='on'` |
| CAN2 TX | `stm32h7xxblockslib/FDCAN Write` (one per frame, in an Enabled Subsystem on `valid`) | `FDCANModule='FDCAN2'`, `CANFrameType='CAN-FD'`, `DataFormatWriteMode='Raw data'`, `IdentifierTypeVia='Dialog'`, `IdentifierType='Standard (11-bit identifier)'`, `IdentifierVia='Dialog'`, `DataLengthVia='Dialog'`, `IsBitRateSwitchingEnabled='on'`; frames MCU_IMU (48 bytes), MCU_ODOM (32), MCU_STATUS (48) and MCU_POWER (64) |
| IMU | `stm32IMUsensorslib/ICM20948 IMU Sensor` | `I2CModule='I2C1'`, `I2CAddress='0x69'`, `BitRate='400000'` (400 kHz, ICD 6), `AccelerometerRange='+/-4g'`, `GyroscopeRange='+/-500 dps'`, `DataType='single'`, `SampleTime='Ts.imu'` (5 ms) |
| Watchdog refresh | `stmutilitieslib/Register Read//Write` in Enabled Subsystem | `tgtHW='STM32H7xx Based'`, `devID='STM32H743'`, `regGrpName='IWDG1'`, `regName='KR'`, `opCode='Write'`, `SampleTime='1e-3'`; input `uint32(0xAAAA)` |
| Profiling (PIL only, T-EL-42) | `stmutilitieslib/Execution Profiler` | `measurementModeNumber='Data Watchpoint Trace (DWT) Timer'` |
| µs timestamp | `stmutilitieslib/Register Read//Write` | TIM2 CNT, `opCode='Read'`, `Ts.base` (TIM2 free-running at 1 MHz, D60) |
| Reset flag | `stmutilitieslib/Register Read//Write` (×2) | RCC RSR `Read` at boot: IWDG1RSTF (`hw.rcc_rsr_iwdg1rstf_bit`) → `reset_flag`; RMVF `Write` on the second tick. The brown-out and power-on flags that D88 and D91 need are planned (card G2-MCU) |
| Encoders | none | the ESCs read the motor encoders, and their counts come in DRV_FB3 over CAN1 (§2.7.7) |

### 3.3 RC link plant (`subsystems/rc/`)
Revision C: the builder `scripts/build/build_rc_link.m` is the binding block list. The operator sources moved to the top-level `Operator_Console` (a Variant Source `OpSource` on `VC_OPSRC`: 1 the scenario input, 2 `Dashboard`, 3 `Joystick` through Simulation 3D Joystick and `joy_map`, 4 `RowDriver`; ICD 3.1 and 8), so revision A's Signal Editor, `vrlib/Joystick Input` and operator Variant Subsystem rows are not built.

| Block | Library path | Key parameters |
|---|---|---|
| Ch → 11 bit bits | `commutil2/Integer to Bit Converter` | `nbits=11`, `bitOrder='LSB first'`, `outDtype='boolean'` |
| Bits → bytes | `commutil2/Bit to Integer Converter` | `nbits=8`, `bitOrder='LSB first'`, `outDtype='uint8'` |
| Flags byte | `commutil2/Bit to Integer Converter` | `nbits=8`, `bitOrder='LSB first'`, input `[ch17 ch18 lost fs 0 0 0 0]` |
| Frame assembly | `simulink/Signal Routing/Vector Concatenate` | `[uint8(0x0F); payload(22); flags; uint8(0x00)]` |
| Packet loss / byte errors | `simulink/Sources/Uniform Random Number` | `Minimum='0'`, `Maximum='1'`, `Seed='P.rc.seed'` (+offset per instance), `SampleTime='P.rc.frame_period'` (frame) / `'1e-3'` (bytes, width 25) |
| Radio latency | `simulink/Discrete/Delay` | `DelayLength='P.rc.extra_delay_frames'` (= 0; revision B, the frame RT provides the one-frame latency) |
| Receiver failsafe | blocks (D59): a continuous-loss timer and a flag selection per failsafe mode | sample time `P.rc.frame_period`; `P.rc.rx_fs_flags`, `rx_flags_ok`, `rx_timer_sat` (10 s) |
| UART line timing | `simulink/User-Defined Functions/MATLAB Function` | §4.2 |
| Frame → 1 kHz | `simulink/Signal Attributes/Rate Transition` | `Deterministic='on'` |

### 3.4 Electronics plant (`subsystems/electronics/`)
Revision C: the builder `scripts/build/build_electronics_plant.m` is the binding block list. Revision A's electrical rows (the `Dynamic Load` aux load, the `DC-DC Converter` blocks, the rail loads and sensors, the aux fuse and the `AUX48_P/N` ports) moved to the power domain's LV distribution (§2.9.3), and the safety-relay logic to the power domain's safety chain (§2.9.1); they are not built here.

| Block | Library path | Key parameters |
|---|---|---|
| Thermal mass | `fl_lib/Thermal/Thermal Elements/Thermal Mass` | `mass=12`, `sp_heat=1000`, initial T = `P.electronics.therm.T0` |
| Convection | `sprintf('fl_lib/Thermal/Thermal Elements/Convective Heat\nTransfer')` | `thermal_type=variable` (PS input h), `area=1.2`, `min_heat_tr_coeff=1e-3` |
| Heat input | `sprintf('fl_lib/Thermal/Thermal Sources/Controlled Heat Flow\nRate Source')` | |
| Ambient | `sprintf('fl_lib/Thermal/Thermal Sources/Controlled\nTemperature Source')` | |
| Temp sensing | `fl_lib/Thermal/Thermal Sensors/Temperature Sensor` | |
| Thermal ref | `fl_lib/Thermal/Thermal Elements/Thermal Reference` | |
| Solver (thermal net) | `nesl_utility/Solver Configuration` | `UseLocalSolver='on'`, `LocalSolverChoice='NE_BACKWARD_EULER_ADVANCER'`, `LocalSolverSampleTime='0.1'` |
| S↔PS | `nesl_utility/Simulink-PS Converter`, `nesl_utility/PS-Simulink Converter` | `InputFilterTimeConstant=0.1` (thermal); `Unit` set (W, K, W/(m^2*K)) |
| CAN2 codecs | `mcblib/Signal Management/Byte Pack`, `Byte Unpack` in the Subsystem References `can2_comp_cmd_*`, `can2_mcu_status_*`, `can2_mcu_imu_*`, `can2_mcu_odom_*`, `can2_mcu_power_*` (pack and unpack) | the ICD 5 layouts, `byteAlign = 1` |
| ADC noise | `simulink/Sources/Random Number` | `Seed` from params |
| Tests | `sltestlib/Test Sequence`, `sltestlib/Test Assessment` | in harnesses |

**Blocks I evaluated and rejected:**
- `stmutilitieslib/Protocol Decoder`. It needs a unique header, cannot use the inter-frame gap and would false-lock on 0x0F bytes inside the payload.
- `commcrc2` CRC blocks. They are not needed, because SBUS has no CRC and CAN has its own.
- `stmutilitieslib/CAN FD Pack`. Not needed, because CAN2 uses raw payloads.
- `ee_lib/Passive/Constant Power Load`. Its power is a fixed parameter, so it cannot follow a load profile. `Dynamic Load` takes a PS input instead (now in the power domain's LV distribution).

---

## 4. MATLAB Function blocks (only where no block exists)

### 4.1 `sbus_frame_sync` (firmware, code-generated, Task_1k)
**Why there is no block:** SBUS is a header-only frame with a footer that has several valid values. The header value 0x0F can appear inside the payload, so robust framing needs (a) byte-by-byte iteration over a variable-length chunk (0..32 bytes per tick), (b) a persistent parser state and (c) gap-based resynchronization. No Simulink, STM32 or Comms block does this; Protocol Decoder was rejected (see §3). A For Iterator + Stateflow version would be about 3× larger and slower. The bit unpacking is deliberately left to blocks.

Revision C (S6c byte errors): the listing is the code of `subsystems/electronics/sbus_frame_sync.m` without its header
comment. Changes from revision B: a UART error taints the frame instead of resetting the parser before the flagged
chunk is consumed; trailing-byte length check; channel plausibility `RAW_MIN..RAW_MAX` (two extra Constant inputs).

```matlab
function [payload, flags, frame_new, n_ok, n_bad, n_skip] = ...
         sbus_frame_sync(bytes, len, uart_err, HDR, GAP_TICKS, RAW_MIN, RAW_MAX)
%#codegen
% bytes uint8[32], len uint8, uart_err boolean
% HDR uint8 = 0x0F, GAP_TICKS uint8 = 2, RAW_MIN / RAW_MAX uint16 = 172 / 1811
% payload uint8[22], flags uint8, frame_new boolean, n_* uint32
persistent buf idx idle okc badc skipc pay_l flg_l ferr
if isempty(buf)
    buf = zeros(25,1,'uint8'); idx = uint8(0); idle = uint8(0);
    okc = uint32(0); badc = uint32(0); skipc = uint32(0);
    pay_l = zeros(22,1,'uint8'); flg_l = uint8(0); ferr = false;
end
frame_new = false;
if len == 0
    if idle < 255, idle = idle + 1; end
    if idle >= GAP_TICKS && idx > 0      % inter-frame gap => resync
        if idx < 25, badc = badc + 1; end
        idx = uint8(0); ferr = false;
    end
else
    idle = uint8(0);
    if uart_err && idx > 0, ferr = true; end   % frame in progress receives a corrupted chunk
    used = false;                              % a byte of this chunk went into a frame buffer
    n = min(double(len), 32);
    for k = 1:n
        b = bytes(k);
        if idx == 0
            if b ~= HDR
                skipc = skipc + 1;             % hunting for header
                continue
            end
            ferr = uart_err;                   % frame starts in this chunk: its error flag applies
        end
        used = true;
        idx = idx + 1; buf(idx) = b;
        if idx == 25
            ftr = buf(25);
            okFtr = (ftr == 0) || (bitand(ftr, uint8(15)) == 4);   % SBUS / SBUS2
            okFlg = bitand(buf(24), uint8(240)) == 0;              % reserved bits
            okLen = k == n;                    % no byte follows the footer in the same 1 ms chunk
            okRng = true;                      % 16 x 11-bit channels, LSB first (spec 2.6.1)
            for c = 0:15
                i = floor(11*c/8); s = 11*c - 8*i;          % payload byte i (0-based), bit shift s
                w = uint32(buf(2+i)) + bitshift(uint32(buf(3+i)), 8) + bitshift(uint32(buf(4+i)), 16);
                v = bitand(bitshift(w, -s), uint32(2047));
                if v < uint32(RAW_MIN) || v > uint32(RAW_MAX), okRng = false; end
            end
            if okFtr && okFlg && okLen && okRng && ~ferr
                pay_l = buf(2:23); flg_l = buf(24);
                okc = okc + 1; frame_new = true;
            else
                badc = badc + 1;
            end
            idx = uint8(0); ferr = false;
        end
    end
    if uart_err && ~used, badc = badc + 1; end   % corrupted chunk outside any frame (header hit)
end
payload = pay_l; flags = flg_l; n_ok = okc; n_bad = badc; n_skip = skipc;
end
```
Block settings: `Discrete` sample time `P.electronics.ts.base`. HDR, GAP_TICKS, RAW_MIN (`P.electronics.rcmap.raw_min`) and RAW_MAX (`raw_max`) arrive on block inputs from Constant blocks (no parameter data, R2026b JIT relink assertion). Saturation on integer overflow is off (counters wrap).

### 4.2 `sbus_uart_line_sim` (plant RC_Link_Sim, simulation only, 1 kHz)
**Why there is no block:** it models a timed, variable-length byte queue in which each byte completes at t0 + i·120 µs, with sub-millisecond phase, drift, drops and errors, and chunks the bytes per 1 ms tick. Buffer/Unbuffer (DSP) cannot produce variable-length chunks aligned to byte completion times.

Revision C: the listing below is revision A's sketch; the code is `subsystems/rc/sbus_uart_line_sim.m`. Each injected byte error flips exactly one bit and is flagged as a parity error (`uart_err`), so every corrupted byte is detectable and S6c's no-glitch criterion cannot fail (review ELC-06, R-47). Card G2-COM adds errors with an even number of flipped bits (no `uart_err`) and byte insertions (REQ-SYS-012).

```matlab
function [bytes, len, uart_err] = sbus_uart_line_sim(frame, frame_new, t, ...
                                   r_err, r_drop, Tb, phase0, drift_ppm, Tf, p_err, p_drop)
%#codegen  (simulation target only)
% frame uint8[25]; frame_new boolean (1 kHz, true on tick a frame is released)
% t double (s, Digital Clock 1e-3); r_err,r_drop double[25] from seeded Uniform Random Number
persistent q tq nq ph
if isempty(q), q = zeros(64,1,'uint8'); tq = zeros(64,1); nq = 0; ph = phase0; end
if frame_new
    ph = mod(ph + Tf*drift_ppm*1e-6, 1e-3);          % drifting sub-ms phase
    t0 = t + ph;
    for i = 1:25
        if r_drop(i) < p_drop, continue; end          % lost byte (overrun/noise)
        b = frame(i);
        if r_err(i) < p_err, b = bitxor(b, uint8(2^floor(8*r_err(i)/p_err))); end
        if nq < 64, nq = nq + 1; q(nq) = b; tq(nq) = t0 + i*Tb; end
    end
end
bytes = zeros(32,1,'uint8'); len = uint8(0); uart_err = false;
n = 0;
while n < nq && tq(n+1) <= t + 1e-9 && n < 32
    n = n + 1; bytes(n) = q(n);
end
if n > 0
    q(1:nq-n) = q(n+1:nq); tq(1:nq-n) = tq(n+1:nq); nq = nq - n; len = uint8(n);
end
% parity error flag: set when an injected bit error falls in this chunk (bookkeeping array omitted for brevity)
end
```

### 4.3 (Fallback only) `iwdg_refresh_hw`
This is used only if `Register Read/Write` cannot address IWDG1 on the H7 device list (see risk R6). It is a MATLAB Function in the HW variant: `coder.ceval('HAL_IWDG_Refresh', coder.opaque('IWDG_HandleTypeDef *','&hiwdg1'))`. It is not used in SIM. It is not built today: `Register Read/Write` addresses IWDG1 KR (§3.2; `t_targets` testRegisterSetupPlan, and testH743TimersGpioWatchdog for IWDG1's prescaler and reload).

The only other MATLAB Function of the firmware is the mode manager, `mcu_mode_manager.m` (§2.7.3), because Stateflow is not installed. Mixing, limits, PI, shaping, bit (un)packing, CAN codecs, debouncing, fault logic and the thermal/power models are all built from library blocks.

---

## 5. Interfaces

### 5.1 Inter-domain signals (the root ports of the electronics' components; ICD 3 binds)
| Component (Subsystem Reference) | Inputs (name : bus : sample time) | Outputs |
|---|---|---|
| `Operator_Console` (Variant Source `OpSource`, `VC_OPSRC`) | `ScenarioIn.op`; for choice 4 the pose, the forward speed, the MCU mode and the tank level, each through a Rate Transition to 0.01 s | `op : Bus_OperatorCmd : 0.01` |
| `Operator_RC` (`RC_Link_Sim`) | `op : Bus_OperatorCmd : 0.01`; `flt : Bus_FaultInject : 0.01` | `sbus : Bus_SbusStream : 0.001`; `rc_frame : Bus_RcFrame : 0.001` (logging copy) |
| `CAN1_Net` (`CAN1_Net_Sim`) | `mcu_fast : Bus_Can1_McuFast : 0.01`; `mcu_slow : Bus_Can1_McuSlow : 0.05`; `esc_fast : Bus_Can1_EscFast : 0.01`; `esc_slow : Bus_Can1_EscSlow : 0.05`; `impl : Bus_Can1_Impl : 0.05`; `bms : Bus_Can1_Bms : 0.1`; `flt : Bus_FaultInject : 0.01`; `elec_hw : Bus_ElecHwIn : 0.001` | the same buses delivered (`*_rx`), same sample times |
| `CAN2_Link` (`Link_MCU_GNC`) | `mcu_imu` 0.005, `mcu_odom` 0.01, `mcu_status` 0.05, `mcu_power` 0.1, `comp_cmd` 0.05 (each `Bus_CanFrame64`); `flt : Bus_FaultInject : 0.01`; `elec_hw : Bus_ElecHwIn : 0.001` | the same frames delivered; `mcu_status_rx` through a one-sample Unit Delay (D41); `comp_cmd_rx` with no delay |
| `Electronics` in `Shaddad_Plant` (`Electronics_Plant`) | `mcu_out1k : Bus_McuOut1k : 0.001`; `mcu_out10 : Bus_McuOut10 : 0.01`; `power_status : Bus_PowerStatus : 0.001`; `hmi : Bus_HmiInputs : 0.001`; `env : Bus_EnvState : 0.1`; `impl_mech : Bus_ImplMechState : 0.01`; `flt : Bus_FaultInject : 0.001` | `elec_hw : Bus_ElecHwIn : 0.001` (through a 1 ms Unit Delay, D36); `elec_adc : Bus_ElecAdc : 0.05`; `elec_env : Bus_ElecEnv : 0.1`; `hw_cmd : Bus_PowerHwCmd : 0.001` (1 ms Unit Delay); `impl_cmd : Bus_ImplementCmd : 0.01`; `lv_demand : Bus_LV_LoadDemand : 0.01`; `indicators : Bus_IndicatorOut : 0.1` |

### 5.2 Simscape physical ports
None. The electronics plant has no physical ports (ICD 3.2): its LV demand goes to the power domain as the signal `Bus_LV_LoadDemand` (§2.9.3), and its enclosure thermal network has no external port; ambient enters as `Bus_EnvState.T_amb` (ICD 1 and 2). Revision A's `AUX48_P/N` ports are not built.

### 5.3 Bus definitions
The buses are defined in `buses/*.m` and listed with every root port in ICD 3; the firmware root I/O is one single-rate nonvirtual bus per rate, `Bus_McuIn1k`, `In5`, `In10`, `In50`, `In100` and `Bus_McuOut1k`, `Out10` and onward (ICD 3.1). The enumerations are those of ICD 9 (storage uint8):
- `ShaddadMode` {0 BOOT, 1 SELFTEST, 2 STANDBY, 3 MANUAL, 4 HOLD, 5 AUTO_START, 6 AUTO, 7 CTRL_STOP, 8 ESTOP, 9 FAULT};
- `ShaddadCmdSrc` {0 NONE, 1 RC, 2 AUTO}.

Revision A's bus tables (`Bus_HwIn1k`, `Bus_HwIn200`, `Bus_HwIn50`, `Bus_HwOut*`, `Bus_Can1Rx`, `Bus_MotorDriverCmd`, `Bus_BmsStatus`, `Bus_PowerCmd`, `Bus_AutoCmd`, `Bus_McuSensFwd`, `Bus_EnvAmbient`, `Bus_ElecPowerOut` and others) are removed: those buses were replaced in the T01 to T10 build. Two facts kept from them: the µs time stamp `t_us` is a uint32 that wraps every 71.6 min (risk R10), and `Bus_SbusStream` carries up to 32 bytes per 1 ms tick with a length and the UART error flag.

### 5.4 DI / DO bit maps (ICD 6 binds; `P.electronics.di`, `P.electronics.do`)
| di bit | Signal | | do bit | Signal |
|---|---|---|---|---|
| 0 | ESTOP_CH1_OK (1 = loop closed) | | 0 | MCU_SAFETY_OK (its contact in the E-stop loop; MCU-internal health only, D30) |
| 1 | ESTOP_CH2_OK | | 1 | SAFETY_RESET_REQ (50 ms pulse, `safety.rearm.t_pulse`, after DO0 has been high for 20 ms, `t_do0_settle`) |
| 2 | SAFETY_OK (the relay's immediate contacts) | | 2 | IMPL24_EN (the K3 request) |
| 3 | BUMPER_FRONT (1 = pressed) | | 3 / 4 / 5 | BEACON_GREEN / AMBER / RED |
| 4 | BUMPER_REAR | | 6 | BUZZER |
| 5 | RESET_BTN | | 7 | WORK_LIGHTS |
| 6 | K1_AUX_FB | | 8 | COMPANION_PWR_EN |
| 7 | K2_AUX_FB | | 9 | LIDAR_PWR_EN |
| 8 / 9 | IMPL_ID0 / IMPL_ID1 (the implement coding plug) | | 10..17 | NOZZLE_1..8 |
| 10 | LID_CLOSED | | 18 | LIFT_DIR (1 = down) |
| 11 | KEY_ON | | 19 | spare |
| 12 | SAFETY_HV_OK | | | |
| 13 | DOCK_PRESENT | | | |

All DO = 0 is the safe state (MCU reset, watchdog). The traction enable, the brakes and the contactors are not MCU outputs: the ESCs take enable and park_req over CAN1 (ICD 4), and the BMS owns the contactors (D4).

### 5.5 SIM vs HW variant per I/O (`VC_TARGET` 0 SIM / 1 HW, ICD 8)
| I/O | `SIM` choice (`fw_io_in_sim` / `fw_io_out_sim`) | `HW` choice (`fw_io_in_hw` / `fw_io_out_hw`) |
|---|---|---|
| SBUS | `sbus` ← `Operator_RC` (`RC_Link_Sim`, UART line model) | UART/USART Read (USART2) |
| DI | `elec_hw.di_word` ← `elp_hw_frontend` | Digital Port Read GPIOF[0:13] |
| Time | the simulation clock | Register Read TIM2 CNT (1 MHz) |
| Reset flag | `elec_hw` ← `elp_watchdog_reset` | Register Read RCC_RSR IWDG1RSTF at boot, then the RMVF write |
| IMU | `imu_raw` ← sensors | ICM20948 IMU Sensor (I2C1) |
| CAN1 RX / TX | the CAN1 buses to and from `CAN1_Net` | FDCAN Read / Write FDCAN1 (raw data) |
| ADC | `elec_adc` ← `elp_hw_frontend` | ADC1 (8 conversions) |
| MCU temp | T_encl + 15 K (`therm.dT_mcu`) | ADC3 TempSensor with the factory calibration (`adc.ts_cal_T`, `ts_cal_cnt`) |
| CAN2 RX / TX | the CAN2 frames to and from `CAN2_Link` | FDCAN Read / Write FDCAN2 (CAN-FD, BRS) |
| DO | `mcu_out1k.do_word` → `elp_actuators` | Digital Port Write GPIOG[0:15], GPIOE[0:3] |
| Watchdog | `wdg_kick` → `elp_watchdog_reset` | Register Read/Write IWDG1->KR |
| PWM | `mcu_out10.pwm_duty` → `elp_actuators` | PWM Output TIM1 (ch1, ch2), TIM4 (ch1, ch2) |
| Bus-off | injected (`Bus_FaultInject`) | Register Read FDCAN_PSR BO (Task_10) |

---

## 6. Parameters (`params/param_electronics.m` → `P.electronics`, `params/param_rc.m` → `P.rc`)

Signatures:
- `E = param_electronics(P)`. It takes the partially assembled `P` (it needs `P.structure`, `P.drive` and `P.world`) and derives from it the mixer's gauge and gauge factor (`P.structure.track.chi_nom`, D39), the drive's sprocket radius, ratio, torques and inertia (`P.drive`, with the motor unit of D95) and the descent tables (`P.structure.descent`, D74).
- `R = param_rc()`.

`shaddad_params.m` creates the variant controls in base as plain double scalars (ICD 8): `VC_TARGET` (from `variant.target`), `VC_OPSRC` and the others. Variant expressions cannot reference struct fields. `P` stays purely numeric (D58), so names and paths (the board name `STM32H7xx Based (Single-core)`, the CubeMX project `utilities/stm32/shaddad_h743.ioc`) are not in `P`.

Values are the defaults (no implement, the 1 kW motor unit of D95); where the 750 W unit or an implement changes a value, both are given. Temperatures are stored in K; the table gives °C where the spec reads better so.

| Field | Value | Unit | Source / justification |
|---|---|---|---|
| **target.mcu.f_cpu / flash / ram** | 480e6 / 2 MB / 1 MB | Hz, B, B | the development target, STM32H743 (D96, §2.2) |
| target.mcu.tim2_f / pwm_f / i2c_f | 1e6 / [20e3 25e3] / 400e3 | Hz | TIM2 time base; TIM1 and TIM4 PWM; I2C1 for the IMU (ICD 6) |
| target.mcu.task_T / task_wcet | [1 5 10 50 100] ms / [25 650 380 100 50] µs | s | §2.7.1 |
| target.cpu_load_max / flash_max | 0.5 / 1 MB | -, B | the design budget |
| target.bms.f_cpu / flash / ram | 170e6 / 512 KB / 128 KB | Hz, B, B | the twin's STM32G4 BMS ECU (a variant, D96) |
| **hw.uart_len / di_pins / do_g_pins / do_e_pins / adc1_n** | 32 / 14 / 16 / 4 / 8 | -, pins, pins, pins, - | the I/O of §2.3 and ICD 6 |
| hw.can1_rx_iter / can2_rx_iter | 16 / 8 | frames per tick | the FIFO drain per task |
| hw.rcc_rsr_iwdg1rstf_bit / rcc_rsr_rmvf_bit / fdcan_psr_bo_bit | 26 / 23 / 7 | bit | the register reads of D60 |
| **ts.base / imu / ctrl / comm / hk** | 1e-3 / 5e-3 / 0.01 / 0.05 / 0.1 | s | §2.7.1: Task_1k, Task_5, Task_10, Task_50, Task_100 |
| **sbus.baud** | 100000 | bit/s | SBUS standard |
| sbus.bits_per_byte | 12 | - | 1+8+1(E)+2 |
| sbus.t_byte | 1.2e-4 | s | 12/100000 |
| sbus.frame_len / payload_len / n_ch / ch_bits | 25 / 22 / 16 / 11 | - | SBUS frame |
| sbus.header / footer / sbus2_low_nibble | uint8(0x0F) / uint8(0x00) / uint8(0x04) | - | SBUS / SBUS2 |
| sbus.read_max | 32 | bytes/tick | ≥ 9 bytes/ms worst case ×3 margin |
| sbus.gap_ticks | 2 | ticks | gap ≥ 4 ms; frames give ≥ 7 bytes/ms |
| sbus.link_timeout | 0.10 | s | ≈ 7 frames at 14 ms |
| sbus.lq_alpha / lq_warn | 0.05 / 0.70 | - | ≈ 20-frame memory |
| sbus.age_sat / lq_ic | 10 / 0 | s, - | `rc_age` saturation; initial link quality |
| **rcmap.ch** (struct) | steer 1, thr 2, aux_lv 3, aux_lh 4, mode 5, arm 6, speed 7, impl 8, reset 9, lights 10, lift 11, estop 12, mission 13, rth 14 | - | §2.7.2, ICD 7 |
| rcmap.raw_mid / raw_half / raw_min / raw_max | 992 / 819.5 / 172 / 1811 (`raw_mid` a 1×16 vector) | counts | FrSky SBUS ±100 % |
| rcmap.sign | ones(1,16) | - | per-channel reversal |
| rcmap.steer_to_omega_sign | −1 | - | stick right = turn right (omega_rc = −x·w_max_man·s, ICD 7) |
| rcmap.deadband | 0.05 | - | 5 % stick |
| rcmap.expo_thr / expo_steer | 0.20 / 0.35 | - | smooth fine control at low speed |
| rcmap.sw_thresh / sw2_thresh | 0.5 / 0 | - | 3-position decode; 2-position decode (x > 0) |
| rcmap.debounce_n / debounce_ic | 4 / [2 1 2 1] | samples at 10 ms, positions | the 3-position switches (mode, implement, lift, mission): a new position passes once it has lasted 30 ms (4 samples), and a held change is accepted 30 to 40 ms after it, about 3 frames; initial positions HOLD, OFF, HOLD, position 1 |
| rcmap.estop_debounce_n / estop_valid_frames_only | 0 / 1 | samples, - | CH12 remote stop: no channel debounce, valid frames only (D34); fault bit 1 then persists 40 ms |
| rcmap.aux_code | [2 3 6] | - | rc_aux_cmd codes START/RESUME, PAUSE, RTH (MissionCmd, ICD 9) |
| rcmap.s_min | 0.25 | - | speed knob minimum (0.35 m/s top speed with `lim.v_max_man` 1.4 m/s) |
| rcmap.neutral_tol / override_tol / override_t / override_n_sat | 0.08 / 0.30 / 0.20 / 1000 | -, -, s, samples | arming and takeover |
| **lim.v_max_man / w_max_man** | 1.4 / 1.0 | m/s, rad/s | the top speed, 1.41 m/s at 3000 rpm (`lim.v_track_max`); pivot track speed w·χB/2 = 0.42 m/s |
| lim.v_max_auto / v_rev_auto / w_max_auto | 1.2 / 0.5 / 1.2 | m/s, m/s, rad/s | working speed up to 1.2 m/s (`P.gnc.spd.v_max`) |
| lim.a_lat_max / v_lat_min | 1.0 / 0.1 | m/s², m/s | rollover margin, implement stability; the low-speed guard of the lateral limit |
| lim.v_track_max / v_overspeed / t_overspeed | 1.414 / 1.696 / 0.2 | m/s, m/s, s | v_track_max = ω_max·r_s/N at 3000 rpm; v_overspeed = 1.2 × v_track_max (bit 16) |
| lim.v_follow_dv | 0.2 | m/s | OVERSPEED speed-follow term, driving modes only [EST] |
| **lim.descent** (D74; copied from `P.structure.descent` when P is built) | theta_bp [0 5 10 15 20 20.5 21 21.5 22 23 24 25]; v_fwd, v_rev 4 × 12; v_fwd_raised, v_rev_raised 1 × 12 | deg, m/s | the cap tables (§2.11 of the structure spec). Rows 1 to 4 = implement 0 to 3; row 5 = unknown implement (the smallest entry of all rows); row 6 = mower with the deck raised |
| lim.descent.ref_fwd / ref_rev, trip_fwd / trip_rev | 6 × 12, derived | m/s | ref = max(cap − dv, 0) (the speed target); trip = max(cap, 0.05 m/s) (the trip); a top-speed entry (no cap) has ref = trip = v_top + dv_top = 1.44 m/s |
| lim.descent.dv / dv_top | 0.10 / 0.03 | m/s | [EST] the target sits dv below the cap; a top-speed trip sits dv_top above v_top |
| lim.descent.v_top | 1.41 | m/s | the top speed as the table writes it |
| lim.descent.t_hold / theta_imu | 1.0 / 20 | s, deg | peak hold of the downhill pitch; the pitch used while the IMU sample is invalid (both directions) |
| lim.descent.t_pred / a_fall / dv_fall | 0.08 / 0.1 / 0.02 | s, m/s², m/s | [EST] the prediction ahead of the trip; the falling-reference test of §2.7.10. `t_pred` holds 0.08 s today; 0.09 s is decided, so that the prediction covers the trip's reaction (REQ-SYS-031), pending on card G2-MCU; provisional under D96: 0.12 s, with `t_pred` to match, if the delivered ESC needs up to 0.10 s (cards G1-04, G2-MCU and G2-DRV) |
| lim.descent.t_release | 0.5 | s | [EST] the hill-start window after both brakes release (T-EL-61) |
| lim.descent.h_raised / lift_raised | 0.09 / 0.667 | m, – | [EST] the mower deck reported raised when the lift potentiometer (0 to 1 over `P.structure.impl.mower.h_range`) is at or above lift_raised = (h_raised − h_range(1)) / (h_range(2) − h_range(1)) |
| **shaper.K_v / K_w** | 4 / 5 | 1/s | §2.7.5 |
| shaper.a_acc_man / a_dec_man | 0.8 / 1.2 | m/s² | |
| shaper.a_acc_auto / a_dec_auto | 0.5 / 1.0 | m/s² | gentler with implement |
| shaper.a_ctrlstop / a_estop | 1.0 / 2.0 | m/s² | controlled stop / remote E-stop; μg ≈ 5.9 m/s² available on soil |
| shaper.j_max / j_stop | 5 / 10 | m/s³ | j ≥ K·a rule |
| shaper.alpha_max / alpha_stop / jw_max / jw_stop | 1.5 / 3.0 / 8 / 30 | rad/s², rad/s², rad/s³, rad/s³ | |
| shaper.K_w_auto / alpha_auto / jw_auto | 20 / 3.0 / 30 | 1/s, rad/s², rad/s³ | the AUTO yaw shaper, non-binding (D41) |
| shaper.estop_tmax / stop_law | 1.2 / 2 | s, - | the remote E-stop's time limit; the constant-deceleration stop law (D33) |
| **mix.B** | 0.6 (← `P.structure`, the track gauge) | m | the as-built gauge |
| mix.chi | 1.3936 (← `P.structure.track.chi_nom`, D39) | - | the effective-gauge factor of the track-soil model |
| **drv.r_s / N / eta_g** | 0.09 / 20 / 0.9 (← `P.drive`) | m, -, - | 1.41 m/s ↔ 3000 rpm motor |
| drv.eta_drv | 0.85 | - | motor + inverter |
| drv.T_peak / T_cont | 19.1 / 6.36 per side; 14.32 / 4.78 with the 750 W unit (← `P.drive`) | N·m | two motors per side: twice one motor's peak and continuous torque (D95) |
| drv.w_min | 20 | rad/s | power-limit singularity guard |
| drv.J_m / m_eq_side | 5.6e-4 / 140.7; 4.0e-4 / 129.8 with the 750 W unit; m_eq 173.2 with the mower | kg·m², kg | m_eq = m/2 + J_m·N²/r_s² |
| drv.k_v2w / w_ref_max | 222.2 / 377.0 | rad/m, rad/s | N/r_s; the DRV_CMD w_ref limit (3600 rpm) |
| drv.enc_cpr / K_enc / odom_window | 4096 / 6.903e-6 / 5 | counts/rev, m/count, samples | the DRV_FB3 counts (the former motor's 1024 PPR × 4); K_enc = 2π·r_s/(CPR·N) |
| drv.Kff_a | 0.703; 0.649 with the 750 W unit; 0.866 with the mower | N·m/(m/s²) | m_eq·r_s/(N·η_g), the tau_ff feedforward |
| drv.enc_mismatch_v / t | 0.3 / 0.2 | m/s, s | ENC_x_MISMATCH |
| Speed loop | removed | - | revision A's `spd.*` (the MCU's speed PI) and `drv.speed_limit_rpm` are gone: the ESCs close the speed loops (D2, D3; drive spec, `P.drive.esc`) |
| **yaw.enable / Kp / Ki / u_max** | 1 / 0.4 / 2.0 / 0.4 | -, -, 1/s, rad/s | the yaw-rate PI (§2.7.6) |
| yaw.tau_bias / standstill_v / tau_target / cmd_still_v / cmd_still_w | 2 / 0.02 / 0.25 / 0.005 / 0.005 | s, m/s, s, m/s, rad/s | the bias average and the commanded standstill (§2.7.6) |
| **trac.enable / s_on / s_off / k_min / k_dn / k_up / v_min** | true / 0.30 / 0.20 / 0.5 / 2.0 / 1.0 / 0.2 | -, -, -, -, 1/s, 1/s, m/s | tracks on soil: 5-15 % slip normal |
| **tlim.dis_margin / P_aux_est** | 0.95 / 120 | -, W | §2.4 |
| Temperature derate | removed | - | revision A's `tlim.Tctrl_*` and `tlim.Tmot_*` are gone: the ESC derates on its own temperatures (`P.drive.esc`) |
| **safety.tilt_warn / tilt_stop / t_tilt** | 25·π/180 / 28·π/180 / 0.5 | rad, rad, s | 20° slope spec + about 1° body pitch + residual estimate error (2026-10-27, was 22°); static roll tip-over about 48° with no implement (atan(0.30/0.267): `P.structure.geom.B` 0.6, `P.structure.lofi.h_cg` 0.267) and 38.7° with the full-tank sprayer (the structure spec's static tip-over table) |
| **imu_cal.roll0 / pitch0** | 0.03367 / −0.03714 | rad | level calibration of the installed IMU (commissioning on level ground, `scripts/imu_level_calibration.m`), subtracted in `fw_imu_tilt` |
| safety.cf_accel_gain / tilt_comp.enable / tilt_comp.tau_acc / tilt_comp.gate_a | 0.02 / 1 / 0.05 / 0.3 | -, -, s, - | the complementary filter with linear-acceleration compensation (D47) |
| safety.comp_timeout / drv_timeout / bms_timeout / impl_timeout | 0.20 / 0.05 / 0.5 / 0.2 | s | 4 / 5 / 5 / 4 periods of COMP_CMD, DRV_FB1 and FB3, the BMS frames and IMPL_FB (ICD 4, 5) |
| safety.estop_discrepancy_t | 0.05 | s | dual-channel check |
| safety.standstill_v / standstill_t | 0.05 / 0.5 | m/s, s | |
| safety.ctrlstop_timeout | 5.0 | s | |
| safety.ctrlstop_t_extra | 1.0 | s | CTRL_STOP time-out margin over v0/a_ctrlstop [EST] |
| safety.stopfail_dv / stopfail_t0 / stopfail_rate / stopfail_brake_t / stopfail_move_t | 0.1 / 0.5 / 0.5 / 0.5 / 0.2 | m/s, s, -, s, s | STOP_FAIL checks (a) to (c), bit 22 [EST] |
| safety.imu_frozen_t | 0.1 | s | all 9 raw IMU axes unchanged for this long: IMU invalid (20 samples) [EST] |
| safety.selftest_t / selftest_timeout | 1.0 / 2.0 | s | |
| safety.auto_start_warn_t | 3.0 | s | start warning |
| Brake timing | removed | - | revision A's `safety.brake_release_delay` / `brake_engage_delay` are gone: the ESC applies and releases the brakes (park_req, D71) |
| safety.V24_min / V12_min / V5_min | 21.6 / 11.0 / 4.6 | V | −10 % / −8 % |
| safety.T_encl_warn / T_encl_crit / T_mcu_max | 70 / 80 / 105 (343.15 / 353.15 / 378.15 K in `P`) | °C | the development target's Jetson and H7 limits; the robot's board ratings are unknown (§2.2) |
| safety.maskA / maskB / maskC / latch_mask / maskDO0 / mask_rc | uint32 bit masks per §2.7.10 table (`mask_rc` = bits 2 and 3) | - | |
| safety.t_persist / derate_vec | 1×32 vectors per §2.7.10 | s, - | |
| safety.overrun_n | 3 | - | |
| safety.dcl_warn | 25 | A | BMS_WARN at DCL below this (bit 11, no derate, D65) |
| safety.derate_bms_warn / derate_tilt / derate_imu / derate_encl | 1 / 0.5 / 0.25 / 0.5 | - | the class-C derates (§2.7.4) |
| safety.rearm.t_do0_settle / t_pulse / auto_selftest | 0.02 / 0.05 / 1 | s, s, - | the DO1 re-arm pulse (D30, ICD 6); auto_selftest 1 today. D91 removes the automatic re-arm at SELFTEST entry (decided, pending on card G2-MCU) |
| safety.rc_required_in_auto | 1 | - | single owner (D44) |
| safety.drv_fault_mask / esc_fault_state / monitor_bits | 0xFBFF / 7 / [5 6 9 28] | - | the ESC fault bits, the ESC FAULT state code, the monitored time-out bits |
| **wdg.timeout / prescaler / reload** | 0.05 / 4 / 399 | s, -, - | LSI 32 kHz/4 = 8 kHz → 400 ticks |
| wdg.age_lim | [0.02 0.06 0.2] | s | the alive-age limits of Task_10, Task_50 and Task_100; Task_50's 0.06 s makes a stall stop the kick within 110 ms (T-EL-34) |
| wdg.t_boot / stall_1k_bit | 0.3 / 7 | s, bit | SIM reset duration; the fault-inject bit of a hung 1 kHz kick |
| wdg.kr_refresh | uint32(0xAAAA) | - | IWDG key |
| Power sequencer | removed | - | revision A's `pwr.*` (the MCU's precharge and contactor sequencer) are gone: the BMS owns them (D4; power spec, `P.power.bms`) |
| **adc.vref / nbits** | 3.3 / 16 | V, - | H7 ADC |
| adc.ch | [Vpack Vbus Ipack V24 V12 V5 NTC LiftPot] | - | order of adc_raw (ICD 6) |
| adc.k_vpack / k_vlink / k_v24 / k_v12 / k_v5 | 21 / 21 / 11 / 5.7 / 2 | - | 200k/10k, 100k/10k, 47k/10k, 10k/10k |
| adc.i_sens / i_off | 4.1667e-3 / 1.65 | V/A, V | ±300 A Hall → 0.4..2.9 V |
| adc.ntc_R25 / ntc_B / ntc_Rpu | 10e3 / 3950 / 10e3 | Ω, K, Ω | |
| adc.ntc_lut_counts / ntc_lut_K | 33-point table computed in file | counts, K | |
| adc.noise_lsb | 4 | LSB rms | ≈ 13 ENOB |
| adc.seed | 1101 | - | |
| adc.liftpot_lim | [0.02 0.98] | - | the lift potentiometer's end limits |
| adc.ts_cal_T / ts_cal_cnt | [30 110] / [12500 16100] | °C, counts | the internal temperature sensor's factory calibration points |
| **can.can1_bitrate** | 500e3 | bit/s | also the delivered BMS's bit rate (§2.2) |
| can.can2_nominal / can2_data | 1e6 / 2e6 | bit/s | |
| can.id.* / can.len.* | per §2.8 tables | -, B | |
| can.res.* | mower_w 0.1, tiller_w 0.01, deck_h 0.001, till_d 0.001, spray_flow 0.01, rc_lq 0.01 | rad/s, rad/s, m, m, -, - | payload resolutions (ICD 5) |
| can.can1_latency_samples / can2_cmd_delay / can2_status_delay | 0 / 0 / 1 | samples | §2.9.6 (the CAN2 loop breaker on MCU_STATUS, D41) |
| can.mcu_status_timeout / age_sat_s / gnc_age_ms_max / t_ms_mod | 0.2 / 10 / 65535 / 2^32 | s, s, ms, ms | the companion's MCU_STATUS time-out; age saturations; the t_ms wrap |
| can.drop_bit.* | drv_cmd_L 0, drv_cmd_R 1, fb1_L 2, fb1_R 3, fb3_L 4, fb3_R 5, fb2_L 6, fb2_R 7, impl_cmd 8, bms_req 9, impl_fb 10, bms 11 | bit | the CAN1 drop mask of `Bus_FaultInject` (ICD 10) |
| can.hw.msg_len / min_gap / qos_depth | 69 / 0.002 / 16 | B, s, - | the ROS 2 CAN-FD bridge messages (D38, §2.9.6) |
| **aux.fan_bp / fan_duty** | [35 40 55] / [0 0.2 1.0] | °C, - | |
| aux.derate_bp / derate_k | [65 70 80] / [1 0.5 0.0] | °C, - | thermal speed derate |
| aux.spray_v_min / pump_duty_man / lift_duty_max / lift_Kp | 0.2 / 0.6 / 0.6 / 20 | m/s, -, -, - | the lift's end limits are `adc.liftpot_lim` |
| aux.blink_f / blink_fast / buzz_period / buzz_on | 1 / 2 / 2.0 / 0.1 | Hz, Hz, s, s | |
| aux.moving_v / lift_h_range / till_d_max | 0.05 / [0.03 0.12] / 0.12 | m/s, m, m | |
| aux.deck_h_default / till_d_default / mower_w_default / tiller_w_default | 0.06 / 0.08 / 293.2 / 26.18 | m, m, rad/s, rad/s | the implement defaults |
| **load.*** | mcu 3, rc_rx 0.5, gnss 1.6, imu 0.03, can_xcvr 0.3, jetson [10 12 22], lidar3d 20, lidar2d 0, modem 3, cams 8, display 5, fan_each 5 (× duty), beacon 3, lights_each 20 | W | §2.4 |
| load.inbox_mcu / inbox_jetson / P_inbox_extra | true / true / 15 | -, -, W | the heat in the box (§2.9.4) |
| LV conversion | removed | - | revision A's `load.eta24`, `eta12`, `Vmin48`, `Vmin24` and `Vmin12` are gone: the power domain converts the LV demand (`P.power.lv`, §2.9.3) |
| **therm.mass / cp / area / h_nat / h_fan** | 12 / 1000 / 1.2 / 4 / 12 | kg, J/(kg·K), m², W/(m²K), W/(m²K) per unit duty | aluminium IP65 box (the development target's); `area` 1.2 m² is 1.22 times the 0.98 m² of revision A's 0.6×0.4×0.25 m, and no source records the difference (§2.9.4; card G2-VAL) |
| therm.alpha_s / A_s | 0.3 / 0.24 | -, m² | white/alu top surface |
| therm.T0 | 318.15 | K | 45 °C start |
| therm.Ts / dT_imu / dT_mcu | 0.1 / 5 / 15 | s, K, K | local solver; the IMU and MCU above T_encl |
| **variant.target** | 0 (SIM) | - | → `VC_TARGET` (ICD 8) |
| ntrip.reconnect_t / corr_age_ok | 10 / 1 | s, s | the NTRIP correction link model |
| **P.rc.frame_period** | 0.014 (0.007 option) | s | SBUS normal / HS |
| P.rc.latency_frames / extra_delay_frames | 1 / 0 | frames | 14 ms TX→RX total, realised by the frame RT (revision B) |
| P.rc.p_loss | 0.01 | - | 1 % nominal packet loss |
| P.rc.seed | 4242 | - | |
| P.rc.rx_failsafe_time | 1.0 | s | typical receiver default |
| P.rc.rx_failsafe_mode | 1 (1 FLAG_PRESET, 2 NO_PULSES, 3 HOLD) | - | |
| P.rc.failsafe_raw | 1×16: 992 all sticks, mode 992 (HOLD), arm 172 (off), **E-STOP CH12 172 (RUN)** (revision B, D34); reset, lights, mission, RTH and implement 172, lift 992 | counts | configure receiver presets this way |
| P.rc.rx_fs_flags / rx_flags_ok / rx_timer_sat | [1 1; 0 0; 0 1] / [0 1] / 10 | -, -, s | the receiver's [failsafe bit, frame sent] per failsafe mode; the flags while the link is good; the loss timer's bound (D59) |
| P.rc.uart.Tb / phase0 / drift_ppm / p_err / p_drop | 1.2e-4 / 3.7e-4 / 200 / 0 / 0 | s, s, ppm, -, - | |
| P.rc.seed_err / seed_drop | seed + 1000·(1:25) / seed + 1000·(26:50) | - | the per-byte seeds of the byte errors and drops |
| Operator source | `VC_OPSRC` (in base, ICD 8) | - | revision A's `P.rc.op_src`, `scenario_file` and `default_scenario` are gone: the operator sources are in `Operator_Console` (§2.6.2) |
| P.rc.joy.axis_steer / axis_thr / thr_sign / btn_arm / btn_estop / btn_mode / btn_reset / btn_start / btn_pause | 4 / 5 / −1 / 1 / 2 / 3 / 8 / 4 / 5 | - | Xbox-layout gamepad |
| P.rc.ch_map / n_ch | same as `rcmap.ch` / 16 | - | TX side mapping |
| P.rc.raw_mid / raw_half / raw_min / raw_max / raw_sw3 / raw_sw2 | 992 / 819.5 / 172 / 1811 / [172 992 1811] / [172 1811] | counts | the transmitter's stick and switch positions |
| P.rc.tx_perm / tx_spare_raw / tx_ch_dig | 1×16 permutation / [992 992] / [0 0] | -, counts, - | the channel order (`ch_map`) of the transmitter's vector; the spare channels at centre; CH17 and CH18 not mapped |

---

## 7. Internal structure, files, model types and sample times

### 7.1 Top-level placement (in `models/Shaddad_System.slx`)
The top model has 11 components joined by direct links (ICD 3.1); the links belong to the integration engineer. The electronics' components are:
```
[ Operator_Console ]  Variant Source OpSource (VC_OPSRC)            -> op : Bus_OperatorCmd
[ Operator_RC ]       SSR subsystems/rc/RC_Link_Sim.slx              -> sbus : Bus_SbusStream  -> MCU
[ MCU ]               Model models/Shaddad_MCU.slx (§7.2)
[ CAN1_Net ]          SSR subsystems/electronics/CAN1_Net_Sim.slx    MCU <-> ESCs, implement ECU (Plant), BMS
[ CAN2_Link ]         SSR subsystems/electronics/Link_MCU_GNC.slx    MCU <-> GNC (companion)
[ Plant ]             Model models/Shaddad_Plant.slx, which holds the electronics plant
                      (SSR subsystems/electronics/Electronics_Plant.slx, §7.4; ICD 3.2)
```
The MCU Model block runs in `SimulationMode='Accelerator'` for missions and `'Normal'` for debugging. `ScheduleRates` stays at the default (periodic).

### 7.2 Firmware model `models/Shaddad_MCU.slx` (Model Reference, code-generated)
Built by `scripts/build/build_shaddad_mcu.m` (named parts only, never `'all'`). Root ports (ICD 3.1): `in1k`, `in5`, `in10`, `in50` and `in100` (`Bus_McuIn*`); `out1k` to `out100` (`Bus_McuOut*`); `telem` (`Bus_FwTelemetry`).
```
Shaddad_MCU
├─ IO_In   (Variant Subsystem, VC_TARGET: 0 SIM = fw_io_in_sim.slx | 1 HW = fw_io_in_hw.slx)
├─ RT_to_Task1k, RT_to_Task5, RT_to_Task10, RT_to_Task50   (Rate Transition groups; RT_fi_100, RT_t_ms,
│                                                           RT_bms_req and RT_pwm at the root)
├─ Task_1k  (1 ms)
│   ├─ fw_sbus_decoder.slx      (MATLAB Function sbus_frame_sync + unpack + link supervision)
│   ├─ fw_watchdog_mgr.slx      (task ages, IWDG kick, DO0, DO1)
│   └─ DO word, watchdog-reset counter, ms tick
├─ Task_5   (5 ms)
│   ├─ sens_acq_imu             (sensors domain: the IMU read and its checks)
│   └─ fw_imu_tilt.slx          (Complementary Filter, roll/pitch, bias average)
├─ Task_10  (10 ms)
│   ├─ fw_rc_mapping.slx
│   ├─ sens_odom_compute        (sensors domain: track speeds from the FB3 counts)
│   ├─ fw_safety_supervisor.slx (fault word, latches, k_der, Descent_trip)
│   ├─ fw_mode_manager.slx      (MATLAB Function mcu_mode_manager.m)
│   ├─ fw_vehicle_ctrl.slx      (arbitration, Descent_cap, shaper, yaw-rate PI, mixer, torque limits, ESC setpoints)
│   ├─ fw_telemetry.slx         (Bus_FwTelemetry)
│   └─ platform bus fw10, status bus st10, alive counter
├─ Task_50  (50 ms)
│   ├─ fw_adc_scaling.slx
│   ├─ fw_implement_aux.slx     (implement, beacon, buzzer, lights, fans, power enables)
│   └─ MCU_STATUS assembly, alive counter
├─ Task_100 (100 ms)
│   └─ alive counter
├─ CAN1_Codec / CAN2_Codec      (multirate: fw_can1_codec.slx, fw_can2_codec.slx with the can2_* pack and unpack units)
└─ IO_Out  (Variant Subsystem, VC_TARGET: 0 SIM = fw_io_out_sim.slx | 1 HW = fw_io_out_hw.slx)
```
The `fw_*.slx` and `can2_*.slx` units are Subsystem Reference files in `subsystems/electronics/`, each unit-testable with its own harness. `sens_acq_imu` and `sens_odom_compute` belong to the sensors domain (`build_sensor_models.m`).

Configuration (`params/shaddad_configsets.m`):
- In simulation the model carries `Simulink.ConfigSetRef 'Shaddad_Ref'` (SourceName `cs_Shaddad`): fixed step 1e-3, EnableMultiTasking on, DefaultParameterBehavior Tunable.
- Code generation uses the set `cs_STM32H7` (HardwareBoard `STM32H7xx Based (Single-core)`, ert.tlc, DefaultParameterBehavior Inlined), completed by `shaddad_configsets('codegen')`. The H743 is the development target (D96).

### 7.3 RC plant `subsystems/rc/RC_Link_Sim.slx` (Subsystem Reference)
```
RC_Link_Sim        (input op : Bus_OperatorCmd at 0.01 s from Operator_Console; RT 0.01 -> 0.001 -> frame period, D32)
├─ RC_Transmitter   rc_transmitter.slx     (stick/switch -> raw[16])            Ts = P.rc.frame_period
├─ RC_Radio_Channel rc_radio_channel.slx   (loss, outage, latency)              Ts = frame_period
├─ RC_Receiver      rc_receiver.slx        (failsafe from blocks, D59)          Ts = frame_period
├─ SBUS_Encoder     rc_sbus_encoder.slx    (Comms blocks + concat)              Ts = frame_period
├─ Rate Transition (frame -> 1 ms) + Detect Change(frame counter) => frame_new
└─ UART_Line_Sim    rc_uart_line_sim.slx   (MATLAB Fn sbus_uart_line_sim)       1 ms  -> Bus_SbusStream
```

### 7.4 Electronics plant `subsystems/electronics/Electronics_Plant.slx` (Subsystem Reference)
```
Electronics_Plant      (in Shaddad_Plant; no physical ports, ICD 3.2)
├─ HW_Frontend_Sim        elp_hw_frontend.slx     (ADC quantization, DI word)                       1 ms / 50 ms
├─ Watchdog_Reset_Sim     elp_watchdog_reset.slx  1 ms
├─ Actuator_Outputs_Sim   elp_actuators.slx       (DO/PWM -> Bus_PowerHwCmd, Bus_ImplementCmd, Bus_IndicatorOut)
├─ LV demand              elp_lv_demand.slx       (Bus_LV_LoadDemand to the power domain, §2.9.3)    10 ms
├─ Electronics env        elp_elec_env.slx        (Bus_ElecEnv)                                     0.1 s
└─ Electronics_Thermal    elp_thermal.slx         (Simscape thermal network, local BE 0.1 s)
```
The RC link is the top-level `Operator_RC`, and the CAN1 network and the CAN2 link are the top-level `CAN1_Net` and `CAN2_Link` (§7.1); revision A's `elp_safety_chain`, `elp_can_net` and `elp_power` are not built (§2.9).

### 7.5 Other files (created in the build phase)
- `params/param_electronics.m`, `params/param_rc.m`, `params/param_rowtest.m`. There is no DBC file: the CAN codecs are Byte Pack/Unpack (ICD 4).
- `buses/*.m` (the electronics buses and the enumerations of ICD 9).
- `subsystems/electronics/sbus_frame_sync.m` and `subsystems/rc/sbus_uart_line_sim.m`. These are called from the MATLAB Function blocks so they can also be unit-tested in MATLAB.
- The builders `scripts/build/build_electronics_plant.m`, `build_rc_link.m`, `build_can2_codecs.m`, `build_tb_elec_plant.m` and `build_tb_mcu_rc.m`.
- The tests `tests/electronics/t_elec_plant.m` (with `elec_harness.m`) and `tests/rc/t_rc_link.m`, `t_row_driver.m`, `t_row_test_system.m` with their benches; the firmware tests are the MCU firmware engineer's (`tests/electronics/t_mcu_*.m`).
- The row test and the live drive: `scripts/phase1_row_test.m`, `phase1_row_run.m`, `phase1_live_drive.m` (`docs/row_test.md`, `docs/live_rc_drive.md`).
- `utilities/stm32/shaddad_h743.ioc` (the development target's CubeMX project).
- Revision A's `params/shaddad_can1.dbc`, `scripts/select_fw_target.m`, `scripts/build_fw_stm32.m`, `tests/electronics/rc_scenarios.mat`, `tElectronics.mldatx` and `t_sbus_codec.m` were not made.

### 7.6 Simulation speed estimate
- The firmware at 1 kHz in Accelerator mode costs about 2-5 µs per base step. A 4 h mission is 14.4 M steps, so about 30-70 s of CPU. In Normal mode it would be about 10 min.
- The RC_Link_Sim MATLAB Function and the 176-bit Comms conversions are negligible.
- The CAN Byte Pack/Unpack codecs at 5 to 100 ms: under 1 % of runtime in Accelerator.
- The Simscape thermal network has 1 state at 0.1 s, which is negligible. The LV demand is a signal computation; the LV conversion is the power domain's.
- My domain should therefore add **< 15 % to the power/drive plant run time**.
- For multi-hour missions use: the MCU in Accelerator, `VC_OPSRC = 1`, and minimal signal logging.

---

## 8. Verification (Simulink Test harnesses, `tests/electronics/`)

### 8.1 Unit harnesses (open-loop, per Subsystem Reference)
| ID | Unit | Scenario | Pass/fail criteria |
|---|---|---|---|
| T-EL-01 | SBUS encoder ↔ decoder | 2000 random frames (uniform 172-1811, random ch17/18/flags), 14 ms and 7 ms periods | 100 % of channels and flags decoded bit-exact; latency from frame start to `frame_new` ≤ 4.0 ms; n_bad = 0 |
| T-EL-02 | Frame sync robustness | Payloads forced to contain 0x0F and 0x00; random phase0 ∈ [0,1) ms, drift ±500 ppm; drop 1 byte every 50 frames | No wrong frame ever accepted (decoded ≠ sent count = 0); resync within ≤ 1 frame after each drop; n_ok ≥ 98 % of frames sent |
| T-EL-03 | Byte errors (`t_mcu_platform/testTEL03_ByteErrors`, S6c) | 40 frames, 8 damaged: a bit flip with uart_err in chunk 1 (the header chunk), 2 and 3; a corrupted header; a flagged chunk with no visible flip; two out-of-range channels (high and low, clean UART); a trailing byte after the footer | Every damaged frame rejected (n_bad = 8, one count per frame); the intact frames all accepted, each on the tick of its last byte; the accepted channels equal the transmitted ones. Errors that parity cannot catch are not injected (R-47; card G2-COM, REQ-SYS-012) |
| T-EL-04 | SBUS2 footers | Footers 0x04/0x14/0x24/0x34 | All accepted |
| T-EL-05 | Link timeout (`t_mcu_platform/testTEL05_LinkLost`) | The transmitter switched off at t = 1.0 s, no random loss | `rc_link_lost` rises 100 ± 2 ms after the last valid frame and `rc_ok` stays false; no link loss before. (`rc_age` saturating at 10 s, `sbus.age_sat`, and the recovery on the first good frame are not checked by this test) |
| T-EL-06 | Link quality | p_loss = 0.2 (frame_lost flags) | LQ = 0.80 ± 0.03 after 2 s; no state change |
| T-EL-07 | Deadband/expo | Ramp x from −1 to +1 over 10 s | Output 0 for \|x\| < 0.05; f(±1) = ±1 ± 1e-6; monotonic; max error vs formula < 1e-6 |
| T-EL-08 | Switch debounce | 1-frame glitch to pos 3; then a sustained step | Glitch rejected; step accepted after 40 ± 14 ms |
| T-EL-09 | Mixer (`testTEL09_Mixer`) | (v,w) pairs including a desaturating one | v_L,R = v ∓ w·χB/2 with χB/2 = 0.418 m (B 0.6 m, χ 1.3936); desaturation preserves the ratio; \|v_track\| ≤ `lim.v_track_max` 1.414 m/s always; criteria in the test |
| T-EL-10 | Reference shaper | Target step 0 → 1 m/s, MANUAL profile | a ≤ 0.8 + 1e-3 m/s²; \|jerk\| ≤ 5 + 1e-2 m/s³; overshoot < 1 %; 98 % of target within 1.9 s |
| T-EL-11 | Torque limiter (`testTEL11_TorqueLimiter`) | V, DCL and ω_m steps | T_hi = min(T_peak, (0.95·V·DCL − P_aux_est)/2·η_drv/ω*) with T_peak 19.1 N·m per side (D95), η_drv 0.85, P_aux_est 120 W; criteria in the test. The temperature derate is the ESC's |
| T-EL-12 | Encoder speed | Superseded: the counts come in DRV_FB3 from the ESCs (D3) and the track speeds from `sens_odom_compute` (sensors spec) | - |
| T-EL-13 | Safety supervisor (`testTEL13_FaultBits`) | Each fault condition of the ICD 6 fault word injected singly | Correct bit set after t_persist ± 1 sample; class A/B/C flags per mask; latch/unlatch per table |
| T-EL-14 | Mode manager (the MATLAB Function FSM; `testTEL14_Guards`, `testTEL14_NeverActiveWithoutArm`, `testTEL14_Transitions`) | Every transition of §2.7.3 | Every state and transition reached; no transition to ACTIVE without `arm_req` |
| T-EL-15 | CAN codec round trip (`testTEL15_Can1RoundTrip`, `testTEL15_Can2RoundTrip`) | Firmware CAN1/CAN2 pack ↔ the plant's unpack (and vice versa), random values in range | Error ≤ ½ LSB of the ICD 4 and 5 resolutions (e.g. torque ≤ 0.005 N·m); float32 payloads bit-exact |
| T-EL-16 | Electronics LV demand (`t_elec_plant` `testLvDemand4hDayAuto`) | 4 h day-AUTO profile | The per-rail split equals `P.electronics.load` within 1 %; the 48 V-side draw is 105 W plus the extra brake holds of the four motors, about 128 ± 10 W (§2.4) |
| T-EL-17 | Enclosure thermal (`testEnclosureThermal`) | T_amb = 50 °C, G = 1000 W/m², fans auto | T_ss < 62 °C (55.3 °C expected with `therm.area` 1.2 m², about 56.5 °C if the box is 0.98 m², §2.9.4); fan duty = 1.0 when T_encl ≥ 55 °C; derate flag at 70 °C when the fan is disabled (fault injection) |

### 8.2 Closed-loop harnesses (the MCU bench `tb_mcu_closed_loop` with a simplified track plant, or the drive/structure plant)
| ID | Scenario | Pass/fail criteria |
|---|---|---|
| T-EL-20 to 23 | Revision A's speed-PI tests (step, ramps, slope load, anti-windup) | Superseded: the ESCs close the speed loops (D2, D3); their tests are the drive domain's (drive spec 8) |
| T-EL-24 | RC drive end-to-end (`testTEL24_ThrottleAndLatency`): throttle 50 % at speed scale 1 | Expected target: deadband (0.5 − 0.05)/0.95 = 0.4737 → expo 0.2·0.4737³ + 0.8·0.4737 = 0.4002 → v_target = 0.4002 × `lim.v_max_man` 1.4 = 0.560 m/s. The latency budget is D37's (ICD 7): TX frame → DRV_CMD frame at the MCU output ≤ 28 ms (14 ms frame RT + 3 ms bytes + 1 ms poll + ≤ 10 ms Task_10 phase); criteria in the test |
| T-EL-25 | Pivot turn (`testTEL25_PivotAndMixer`): steer 100 % at v = 0 | v_L = −v_R = w_max_man·χB/2 = 0.418 m/s (track refs); criteria in the test |
| T-EL-26 | RF loss at t = 5.0 s while driving in MANUAL at throttle 0.6, the receiver in FLAG_PRESET with its failsafe after 0.02 s; link back at 6.0 s; arm off at 7.5 s, on at 8.0 s (`testTEL26_Failsafe`) | CTRL_STOP ≤ 20 ms after the first failsafe frame; decel ≤ 1.05 m/s² (100 ms mean); ESTOP never entered; no motion before the new arm edge; MANUAL again after it |
| T-EL-27 | RF loss at t = 5.0 s with the receiver in NO_PULSES after 0.02 s (`testTEL27_NoPulses`) | `RC_LINK_LOST` 100 ± 2 ms after the last valid frame; no frames after the loss; fault bit 2 ≤ 10 ms after it; the robot ends in CTRL_STOP (class-B stop) |
| T-EL-28 | Remote E-stop on CH12 while driving (`testTEL28_RemoteEstop`) | Decel 2.0 ± 0.1 m/s²; the drive disabled at standstill or ≤ 1.2 s, then the park; state ESTOP latched; reset needs CH12 at RUN + reset_req |
| T-EL-29 | Hardware E-stop with the firmware frozen | Now the power domain's safety chain: S8a and `t_power_plant` estop_T04 (STO within 20 ms of the press in the twin, independent of the MCU; REQ-SYS-063 requires at most 50 ms, provisional); the MCU's HW_ESTOP report by `testTEL13_FaultBits` |
| T-EL-30 | AUTO handover: mode switch at AUTO with comp_ok and gnc_ready (D20), the companion commanding 0.8 m/s (`testTEL30_31_AutoStartOverride`) | AUTO_START for 3.0 ± 0.01 s with the buzzer on, then AUTO. (No tracking criterion: the test checks none) |
| T-EL-31 | Stick override in AUTO: throttle 0.5 for 0.3 s | MANUAL within 0.2 + 0.01 s of threshold crossing |
| T-EL-32 | Companion timeout in AUTO | CTRL_STOP at 200 ± 20 ms after the last COMP_CMD |
| T-EL-33 | Pitch ramp 18 → 32° at 1°/s at rest; then 0.8 m/s² starts on a 20° ramp (D47) (`testTEL33_Tilt`) | TILT_WARN at 25 ± 1° and TILT_STOP at 28 ± 1°, the pitch measured 0.5 s (`safety.t_tilt`) before each bit; no TILT_WARN during the ramp starts, and the mode stays MANUAL. (No stop criterion: the test checks none) |
| T-EL-34 | Watchdog: Task_50 stall injected at 3.0 s (`testTEL34_WatchdogStall`) | The kick stops ≤ 110 ms after the stall (`wdg.age_lim`), with no kick while stalled, and DO0 drops. In the plant (`t_elec_plant/testWatchdogResetAndDoGating`): `mcu_reset` 50 ms after the last kick (`wdg.timeout`) for t_boot 0.3 s, all DO and PWM 0 during the reset, so the safety relay drops. The ESCs go to QUICK_STOP 100 ms after the last DRV_CMD (drive `testT10_canTimeout`). **Decided end state (D88, D91; pending on card G2-MCU):** after the reset the robot waits in FAULT, brakes held and the safety relay not re-armed, until the operator's deliberate reset and a new arm command. Today it runs BOOT → SELFTEST → STANDBY; no test checks that sequence |
| T-EL-35 | Drive L feedback loss (`testTEL35_FeedbackLoss`) | DRV_L_TIMEOUT 50 ms after the last FB1/FB3 (`safety.drv_timeout`); FAULT; the drive disabled within one 10 ms task; criteria in the test |
| T-EL-36 | BMS alarm at standstill and while moving | Bit 10 BMS_ALARM in `testTEL13_FaultBits` and the class-B stop and park in `testTEL48_ClassBStopParks`; the contactors are the BMS's (D4; power spec); the DCL limit acts through the torque limiter (D65) |
| T-EL-37 | Precharge | Superseded: the BMS owns the precharge and the contactors (D4; power spec, T-PWR-03) |
| T-EL-38 | Traction: v_ground = 0.5·v_track (50 % slip) | k_tr ramps to 0.5 within 0.3 s; recovers at 1 /s when slip < 0.2 |
| T-EL-43 | AUTO pause after a turn (S4 stop 3, `t_mcu_control/testTEL43_YawPauseNoCreep`): plant yaw gain 0.7, 0.5 m/s with 0.3 rad/s, then v_cmd = omega_cmd = 0 for 66 s (cmd_valid, mode AUTO), then 0.3 m/s straight | Over the last 60 s of the pause: max \|vtrack_ref\| < 1e-4 m/s and heading change < 0.5°. Telemetry yaw rate minus true yaw rate (last 10 s) < 0.002 rad/s. True yaw rate in the straight drive after the pause (the loop's bias error) < 0.002 rad/s |
| T-EL-45 | Pivot at 1.0 rad/s, remote E-stop at 4.0 s (`testTEL45_PivotEstopDecel`) | Pass/fail criteria in the test header comment (`tests/electronics/t_mcu_control.m`) |
| T-EL-46 | 46a: tilt 30°, `imu_valid` false one sample every 0.4 s (`testTEL46_TiltHoldOnInvalidImu`); 46b: yaw PI wound up to u_max, invalid sample | Pass/fail criteria in the test header comment |
| T-EL-47 | Bench IMU stuck at 3.00 s, at rest (`testTEL47_StuckImuFault`) | Pass/fail criteria in the test header comment |
| T-EL-48 | RC link lost while driving: parking with the torque limits held, no bit 22 | Pass/fail criteria in the test header comment |
| T-EL-49 | Speed held at 1.0 m/s through the stop: OVERSPEED stays clear in CTRL_STOP; STOP_FAIL | Pass/fail criteria in the test header comment |
| T-EL-50 | Left brake never engages after park_req | Pass/fail criteria in the test header comment |
| T-EL-51 | Runaway in STANDBY | Pass/fail criteria in the test header comment |
| T-EL-52 | CTRL_STOP that cannot reach standstill: time-out from 1.4 and 1.2 m/s | Pass/fail criteria in the test header comment |
| T-EL-53 | Runaway from 0.8 m/s: speed-follow OVERSPEED, immediate park, STOP_FAIL | Pass/fail criteria in the test header comment |
| T-EL-54 | Speed target per implement and direction at downhill pitch 0, 3, 7, 12, 17, 22 and 27 deg, with the 1 s peak hold and its release (`testTEL54_DescentCapPerImplement`); tiller reverse 0 at every pitch | Pass/fail criteria in the test header comment (`tests/electronics/t_mcu_control.m`) |
| T-EL-55 | Descent trip (`testTEL55_DescentCapTrip`): bit 16 set in the same sample the predicted speed passes the cap; no trip on a crest with regen working, or in STANDBY | Pass/fail criteria in the test header comment |
| T-EL-56 | CCL 0 MANUAL stop from 1.4 m/s on flat ground and on 4 deg (`testTEL56_CclZeroManualStop`): no speed-follow OVERSPEED; the mode stays MANUAL | Pass/fail criteria in the test header comment |
| T-EL-57 | CCL 0 AUTO stop from 1.2 m/s on 4 deg and on flat ground (`testTEL57_CclZeroAutoStop`): the companion commands 0 at 9 s; no OVERSPEED | Pass/fail criteria in the test header comment |
| T-EL-58 | Sprayer with a full tank (316 kg), CCL 0, on a 14.5 deg descent (`testTEL58_CclZeroSprayerDescent`): the speed target stays under the cap; each OVERSPEED park passes the structure park criterion; no STOP_FAIL | Pass/fail criteria in the test header comment |
| T-EL-59 | Tiller, stick back 2 to 4 s (`testTEL59_TillerNoReverse`): the speed target and v_ref stay at 0; the mode stays MANUAL | Pass/fail criteria in the test header comment |
| T-EL-60 | Mower with the deck down (0 deg) and raised (0 and 7 deg) (`testTEL60_MowerDeckRaised`): the speed target takes the raised rows | Pass/fail criteria in the test header comment |
| T-EL-61 | Hill start with the sprayer (316 kg) on a climb (`testTEL61_HillStartRollback`): (a) normal CCL and (c) no regen; the roll-back in the `t_release` window does not trip; a real runaway in the window still trips on the measured speed, parks at once and stops with no STOP_FAIL | Pass/fail criteria in the test header comment |

**Bench (card G2-DSC, 2026-11-16).** `tb_mcu_closed_loop` takes a 9-value plant input: `impl` (the DI IMPL_ID bits), `m` (above 0 the plant is physical, with slope load and rolling resistance, so CCL matters on the bench) and `f_rr`. CCL is set with `bms.I_chg_lim_A`. The MCU bench runs use 226 kg (no implement) with rolling resistance 0.05; T-EL-58 and T-EL-61 use the sprayer's 316 kg.

### 8.3 Code generation / target tests
| ID | Test | Criteria |
|---|---|---|
| T-EL-40 | `slbuild` with `cs_STM32H7` (blocked while the STM32 tools are missing: `t_mcu_platform` testCodegenBlocked) | 0 errors; no MATLAB Function warnings about dynamic memory; Model Advisor (Embedded Coder checks) without errors; the generated interface unchanged (7 entry points, 23 struct types) unless a task changes it |
| T-EL-41 | SIL vs Normal (back-to-back, all §8.2 scenarios) | max \|Δ\| < 1e-5 (single) on the DRV_CMD setpoints and limits; identical mode sequences |
| T-EL-42 | PIL on NUCLEO-H743ZI2 + Execution Profiler | every task within its period; total CPU < 50 % (`target.cpu_load_max`, build plan T17; the design estimate is 20 %, §2.7.1); 0 overruns in a 10 min run |
| T-EL-43 | HW bench: real RC receiver on USART2 | 10 min, 0 bad frames with a clean link; `frame_new` period 14.0 ± 0.1 ms; failsafe flag observed when TX is switched off |
| T-EL-44 | Bus load (analytical script from the DBC/layout tables) | CAN1 < 40 %, CAN2 < 30 % |

What runs today without the STM32 tools:
- `t_targets`:
  - testDesignEstimate: the task set [1 5 10 50 100] ms, and the CPU load estimate under 50 %;
  - testPeripheralsPlan, testRegisterSetupPlan, testIocGenerated, testIocStructure, testH743Fdcan, testH743I2cAdc, testH743Sbus, testH743TimersGpioWatchdog, testFdcanRatesConsistent;
  - the three preflight tests are incomplete without the tools.
- `t_mcu_platform` testHwVariantCompile (needs `T08_LONG`).

Before any firmware built from the twin is flashed to the robot, D95's flashing condition must hold (§2.7.9), and card B-01's bench tests must pass on the board.

The `fault_inject` bits in `HwIn1k` (SIM) are: b0 stall the Task_10 counter, b1 stall Task_50, b2 stall Task_100, b3 force the overrun flag, b4 force CAN1 bus-off, b5 force CAN2 bus-off, b6 force IMU invalid, b7 stop the 1 kHz kick.

---

## 9. Open questions, risks and convention flags

### 9.1 Open questions (for the orchestrator / neighbour domains)
Revision C: the state of each question on 2026-11-18.
- **Q1 (drive). Closed by architecture D2 and D3.** The ESCs run in SPEED mode and go to QUICK_STOP 100 ms after the last command (ICD 4). The values are r_s 0.09 m, N 20, T_peak 19.1 N·m per side and J_m 5.6e-4 kg·m² with the 1 kW unit (14.32 N·m and 4.0e-4 kg·m² with the 750 W unit, D95; §6). The delivered ESC's protocol and time-out come with card G1-04 (D96).
- **Q2 (sensors/gnc). Closed.** The IMU is wired to the MCU (I2C1) and forwarded in MCU_IMU 0x201 every 5 ms (ICD 5). The ICM-20948 stays the development target's part; the robot's IMU is not recorded.
- **Q3 (power/drive). Closed.** The brake-release holds are the drive's (D64) and the contactor coils the power domain's (`P.power.lv.coil_hold_W`); the electronics' LV demand goes to the power domain's LV distribution (§2.4, §2.9.3).
- **Q4 (gnc). Answered by the design:** COMP_CMD every 50 ms with the 0.2 s time-out (ICD 5); REQ-SYS-068 sets the time-out in G1-01 part 2 (the MCU firmware engineer).
- **Q5 (structure). Closed by D39:** χ = `P.structure.track.chi_nom` (1.3936).
- **Q6 (user). Partly closed.** The receiver presets are D34's: the mode switch at HOLD (992) and the remote stop at RUN (172), never E-STOP. The robot's own RC transmitter and receiver are not recorded (card G1-04).
- **Q7 (user). Open.** No certified wireless E-stop is fitted (hazard log E-04); the safety concept decides it (card G2-FSC). The model input exists.

### 9.2 Risks
- **R1. Model-reference hardware-implementation mismatch.** The STM32 firmware is 32-bit ARM Cortex-M. The GNC model, if configured for the Jetson, is ARM64 LP64. Simulink checks hardware-implementation consistency across a model-reference hierarchy, so mixed device types under one top model can error at update. **Mitigation:** in simulation, every referenced model uses the shared `cs_sim` device (ARM Cortex-M or generic 64-bit). Each model then switches to its codegen config only for its own build (`ConfigSetRef`).
- **R2. Unverified driver details on target.**
  - UART Read: does non-blocking mode with `OutputLength=on` and `DataLength=32` return the number of bytes actually available? I believe it does. Fallback: DataLength = 1 inside a For Iterator, up to 32 iterations, exit on status.
  - FDCAN Read raw-data output: the frame length and identifier as the codecs expect them.

  Both are to be checked in the first HW bring-up (T-EL-43 of §8.3). Revision A's Encoder-block question is gone: the encoders are the ESCs' (D3). Under D96 the bring-up waits for the board decision (card G1-04).
- **R3. (Closed.)** Revision A's DBC path risk is gone: the CAN codecs are Byte Pack/Unpack with no DBC (ICD 4).
- **R4. Multirate root I/O.** Nonvirtual buses must be single-rate at model-reference root ports, which is why the I/O is split into one bus per rate (`Bus_McuIn1k` to `In100`, `Bus_McuOut1k` and onward; ICD 3.1). Do not merge them into one bus.
- **R5. Variant activation.** `update diagram` is required. With `code compile` the STM32 blocks would be compiled in the SIM configuration and fail when HardwareBoard is None.
- **R6. Register Read/Write device lists.** The block exists and supports `tgtHW='STM32H7xx Based'`, but the IWDG1 register group for H743 could not be checked without creating a model, which this design phase does not allow. Fallback: §4.3.
- **R7. SIM latency.** CAN1 adds no latency in SIM (`can.can1_latency_samples` 0); the rate transitions between the 10 ms task and the ESC frames set the loop delay. The real bus adds well under 1 ms per frame at 21 % load, so the HW loop is expected to have at least the SIM's phase margin. CAN2 carries the one-sample MCU_STATUS delay (D41).
- **R8. FSM temporal logic.** The state machines are MATLAB Functions (Stateflow is not installed); every timer is a tick count of its task (§2.7.3).
- **R9. ICM-20948 in a 48 V tracked vehicle.** It is sensitive to vibration and magnetic interference near the traction motors and cables. The magnetometer is not used for the MCU tilt (HasMagnetometer off). Heading is left to the GNC (dual-antenna RTK recommended).
- **R10. uint32 µs timestamp wraps every 71.6 min.** Consumers (the companion side and the ROS bridge) must unwrap it. This is tested in multi-hour runs.
- **R11. Simulation pacing.** For the joystick and dashboard operator sources (`VC_OPSRC` 3 and 2) the top model needs Simulation Pacing at 1×. Dashboard blocks inside a Subsystem Reference may need rebinding after the reference is expanded. If binding fails, use them at the top-model level.
- **R12. Algebraic loops.** None are expected: the MCU-GNC loop is broken by the one-sample MCU_STATUS delay in `Link_MCU_GNC` (D41), the DI word reaches the MCU through the 1 ms `elec_hw` Unit Delay (D36), and the MCU's task rate transitions sample the ESC frames. `testLoopBreakersWithFeedthroughMcu` and `testClosedLoopBenchNoAlgebraicLoops` (`t_elec_plant`) check it.
- **R13. Embedded Coder license feature.** `license('test','Embedded_Coder')` returned 0, but the correct feature name `RTW_Embedded_Coder` returned 1, so this is OK. `Instrument_Control_Toolbox` also returned 0 under that feature name, while `Instr_Control_Toolbox` returned 1. Instrument Control is not needed.

### 9.3 Convention flags
- *"Controllers as Model Reference with bus-typed root I/O"*: followed. The root buses are split by rate (R4).
- *"Hardware I/O in Variant Subsystems SIM vs HW"*: followed. Note that the CAN **codecs are deliberately outside the variant** (common code) so the encoding is exercised in SIM. The SIM variant therefore passes CAN_MESSAGE_BUS / byte-array slots, not engineering values.
- *"SI units everywhere"*: plant buses use SI (K for temperature). The firmware uses **°C** internally and on CAN for temperatures, matching drive and BMS protocol conventions. The magnetometer is in **µT**, matching the imuSensor and ICM20948 blocks. Field names make the unit explicit (`_degC`, `_uT`).
- *"Parameters in base-workspace struct P"*: followed. Variant control variables must additionally be **scalar base variables** (`VC_TARGET`, `VC_OPSRC` and the others of ICD 8), because variant expressions cannot index structs.
