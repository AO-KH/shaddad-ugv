# Shaddad UGV - Hardware build setup and bring-up (T17)

Owner: electronics (build plan T17; decisions D31, D38, D60; ICD sections 4-7).
Status on the build PC (2026-10-22): every board build is **BLOCKED** because Simulink Coder,
STM32CubeMX, STM32CubeCLT / GNU Tools for STM32 and STM32CubeProgrammer are not installed (D31). Since 2026-11-07 Simulink Coder is installed (D121, 2026-11-23), so the board builds stay **BLOCKED** for the STM32 tools alone (card B-00).
The file-level items are done and tested (`tests/electronics/t_targets.m`): the two CubeMX projects,
the board fields of the code-generation configuration sets, the build script, the design estimates
and the update-diagram preflight of the three deployable models. The preflight found defects in the
HW variants of `Shaddad_BMS` (T04) and `Shaddad_Companion` / `companion_io` (T10); section 9 lists them
with the fixes.

## 1. Files

| File | Purpose |
|---|---|
| `utilities/stm32/stm32_ioc_configure.m` | Generates both CubeMX projects from the STM32 blockset NUCLEO templates (key=value text edits, idempotent). `stm32_ioc_configure('timing','h743')` returns the derived bit timings and prescalers. |
| `utilities/stm32/shaddad_h743.ioc` | MCU project (STM32H743ZIT6, NUCLEO-H743ZI2 template `P_meta.hw.cubemx_template`). |
| `utilities/stm32/shaddad_g474.ioc` | BMS project (STM32G474RET6, NUCLEO-G474RE template). |
| `utilities/stm32/stm32_target_sets.m` | Board fields of `cs_STM32H7`, `cs_STM32G4`, `cs_Companion` (`'apply'`, `'check'`, `'defs'`). |
| `scripts/build/build_targets.m` | Board builds (`slbuild`) with code size, task-timing estimate and PIL readiness; product check; register setup of the T08 placeholders; fresh-process preflight. |
| `params/param_electronics.m` (`P.electronics.target`) | Peripheral targets (TIM2 1 MHz, PWM 20/25 kHz, I2C 400 kHz timing, CAN sample points, message RAM), TS_CAL addresses, task WCET estimates, acceptance limits. |

Regenerate everything with:

```matlab
stm32_ioc_configure();          % both .ioc files (rewritten only when the text changes)
shaddad_configsets('codegen');  % complete codegen sets (HardwareBoard) from the cache
stm32_target_sets('apply');     % CubeMX projects, FDCAN rates, toolchains, ROS 2 build settings
rep = build_targets();          % BLOCKED until the products below are installed
```

## 2. Products needed for the board builds

| Product | Needed for | State |
|---|---|---|
| Simulink Coder (+ Embedded Coder, installed) | every ERT / STM32 / ROS 2 build, SIL, PIL | missing (licence seat exists); installed since 2026-11-07 (D121, 2026-11-23) |
| STM32 Microcontroller Blockset | STM32 driver blocks, board sets | installed |
| STM32CubeMX 6.16 (the template version) | opening / regenerating the `.ioc` projects | missing |
| STM32CubeCLT (GNU Tools for STM32, SVD files) | compiling for STM32; device data of `stmutilitieslib/Register Read//Write` | missing |
| STM32CubeProgrammer | flashing, PIL, reading TS_CAL words | missing |
| ROS Toolbox | `ros2lib` blocks of the companion | installed |
| On the Jetson: ROS 2, colcon, ssh access | remote colcon build of the companion node | to be prepared |

`build_targets('products')` reports the state; the STM32 tool paths come from the blockset hardware
setup (`stm32cube.hwsetup.stm32Tools`). Install with the STM32 blockset Hardware Setup (Add-Ons).

## 3. MCU project `shaddad_h743.ioc` (STM32H743ZIT6, 480 MHz)

Clock tree of the template kept: HSE 8 MHz (ST-LINK MCO), SYSCLK 480 MHz, APB1/APB2 timer clocks 240 MHz,
FDCAN kernel 64 MHz (PLL1Q), USART2 / I2C1 kernel 120 MHz, HAL time base TIM5.

| Function | Peripheral, pins | Settings (computed by `stm32_ioc_configure`) |
|---|---|---|
| SBUS | USART2 RX PD6 (TX PD5 assigned, unused) | 100000 Bd, 9-bit word incl. even parity, 2 stop, RX only, RX pin level inverted (RXINV), oversampling 16, BRR 1200 exact; NVIC USART2 on (blockset buffered RXNE receive) |
| CAN1 (drives, BMS, implement) | FDCAN1 RX PD0 / TX PD1 | classic, prescaler 1, 128 tq = 1 + 111 + 16, SJW 16 -> 500 kbit/s, sample point 87.5 %; message RAM offset 0, 16 standard filters, RX FIFO0 16 x 8 B, TX FIFO 8 x 8 B |
| CAN2 (Jetson) | FDCAN2 RX PB12 / TX PB13 | CAN-FD with BRS; nominal prescaler 1, 64 tq = 1 + 50 + 13, SJW 13 -> 1 Mbit/s, SP 79.7 %; data prescaler 1, 32 tq = 1 + 25 + 6, SJW 6 -> 2 Mbit/s, SP 81.3 %; message RAM offset 1280 words, 8 filters, RX FIFO0 16 x 64 B, TX FIFO 16 x 64 B |
| IMU ICM-20948 (0x69) | I2C1 SCL PB8 / SDA PB9 (external pull-ups) | Fast mode, TIMINGR 0x20F02735: PRESC 2 (25 ns), SCLL 53 (t_LOW 1.35 us), SCLH 39 (t_HIGH 1.00 us), SCLDEL 15 (0.40 us), SDADEL 0 -> f_SCL 357-400 kHz for 0.15-0.45 us of synchronisation and rise time |
| ADC (Bus_McuIn50.adc_raw order) | ADC1, 16 bit, 64.5-cycle sampling, software trigger, scan | rank 1 PA3 INP15 Vpack, 2 PC0 INP10 Vbus, 3 PB1 INP5 Ipack, 4 PA6 INP3 V24, 5 PA4 INP18 V12, 6 PA5 INP19 V5, 7 PA0 INP16 NTC, 8 PB0 INP9 lift pot |
| T_mcu | ADC3 internal temperature sensor, 810.5-cycle sampling | calibration words TS_CAL1 (30 degC) at 0x1FF1E820, TS_CAL2 (110 degC) at 0x1FF1E840 (`P.electronics.target.mcu.ts_cal_addr`) |
| PWM pump / lift | TIM1 CH1 PE9 / CH2 PE11 | PSC 0, ARR 11999 -> 20 kHz |
| PWM fans | TIM4 CH1 PD12 / CH2 PD13 | PSC 0, ARR 9599 -> 25 kHz |
| t_us | TIM2 (32 bit, internal clock) | PSC 239, ARR 0xFFFFFFFF -> 1 MHz free-running, wraps after 71.6 min (D60) |
| DI word | GPIOF[0:13] inputs | PF0 ESTOP_CH1_OK ... PF13 DOCK_PRESENT (labels `DI<bit>_<name>`, ICD 6) |
| DO word | GPIOG[0:15] + GPIOE[0:3] outputs, reset state low | PG0 MCU_SAFETY_OK ... PG15 NOZZLE6, PE0 NOZZLE7, PE1 NOZZLE8, PE2 LIFT_DIR, PE3 spare |
| Watchdog | IWDG1 (LSI 32 kHz) | prescaler /4, reload 399 -> 50 ms |
| Debug / PIL | USART3 PD8/PD9 (ST-LINK VCP), SWD | kept from the template; `Connection.SerialModule = USART3` |

Template pins re-assigned on the NUCLEO board: PB0 (LD1 green LED: open its solder bridge or the LED
loads the lift-pot input), PB13 (RMII TXD1 of the Ethernet PHY: move the PHY jumper / solder bridge or
use FDCAN2 on PB5/PB6 on the dev board), PE1 (LD2), PG7 (USB OVCR), PG11/PG13 (RMII). The remaining
template ETH, USB and LED pins are untouched. Pin-map deviations from the electronics spec 2.3 (which
predates revision B): DI uses GPIOF[0:13] and DO GPIOG[0:15] + GPIOE[0:3] (ICD 6); the encoder timers
are gone (encoders are read by the ESCs, CAN FB3) and TIM2 is the 1 MHz time base; ADC1 uses PA5 / PA0
instead of PC2 / PC3 (the H743 `_C` analog pads are avoided).

Open the file in STM32CubeMX 6.16, check the pinout (no red pins expected), and regenerate the code
when the tool is installed. CubeMX may reorder keys and recompute the `Calculate*` fields; commit the
saved file only if `stm32_ioc_configure` is updated to produce the same settings.

## 4. BMS project `shaddad_g474.ioc` (STM32G474RET6, 170 MHz)

| Function | Peripheral, pins | Settings |
|---|---|---|
| CAN1 | FDCAN1 RX PA11 / TX PA12 | classic, FDCAN kernel 170 MHz, prescaler 2, 170 tq = 1 + 148 + 21, SJW 21 -> 500 kbit/s, SP 87.6 % |
| AFE (isoSPI bridge) | SPI2 SCK PB13 / MISO PB14 / MOSI PB15, CS PB12 (GPIO, idle high) | master, mode 3 (CPOL high, CPHA 2nd edge), 8 bit MSB first, APB1 170 MHz / 256 = 664 kbit/s (<= 1 Mbit/s isoSPI limit) |
| ADC (Bus_BMS_Meas order of IO_In HW) | ADC1, 12 bit, 92.5-cycle sampling | PA0 IN1 I_pack, PA1 IN2 V_pack, PC0 IN6 V_bus, PC1 IN7 V_dock, PC2 IN8 T_amb, PC3 IN9 T_air |
| DI | GPIOC[4:11] | K1_FB, K2_FB, K4_FB, DOCK_PRESENT, SAFETY_OK, SAFETY_HV_OK, KEY_ON, FUSE_MAIN_OK |
| DO | GPIOB 4, 5, 6, 7, 10 | K1_CMD, K2_CMD, K4_CMD, CHARGER_EN, BAL_EN |
| Debug | LPUART1 PA2/PA3 (ST-LINK VCP), SWD | kept from the template |

## 5. Code-generation configuration sets (`stm32_target_sets`)

| Set | Model(s) | T17 fields |
|---|---|---|
| `cs_STM32H7` | `Shaddad_MCU` | HardwareBoard `STM32H7xx Based (Single-core)`; CubeMX project `shaddad_h743.ioc` selected with the blockset routine `stm32cube.parameters.setCubeMXProjectDetails` (DeviceId STM32H743ZITx, Family STM32H7, CPU 480 MHz, Baremetal, USART3 serial module); `FDCAN1.NominalBaudrate 500000`, `FDCAN2.NominalBaudrate 1000000`, `FDCAN2.DataBaudrate 2000000`; toolchain `GNU Tools for STM32` |
| `cs_STM32G4` | `Shaddad_BMS` | HardwareBoard `STM32G4xx Based`; `shaddad_g474.ioc` (STM32G4, 170 MHz); `FDCAN1.NominalBaudrate 500000`; `GNU Tools for STM32` |
| `cs_Companion` | `Shaddad_Companion`, `Shaddad_NavFusion`, `Shaddad_Perception`, `Shaddad_GNC`, `companion_io_ros2` | HardwareBoard `Robot Operating System 2 (ROS 2)`, ert C++, device `ARM Compatible->ARM 64-bit (LP64)`, toolchain `Colcon Tools`, remote build on (colcon on the Jetson), ROS 2 folder `/opt/ros/humble`, workspace `/home/shaddad/shaddad_ws`, device 192.168.55.1 (Jetson USB device-mode address; change for Ethernet/Wi-Fi), user `shaddad`, build action `Build and load` |

The board fields need the HardwareBoard, which `shaddad_configsets('codegen')` sets (cache). They live in a
T17 file instead of `params/shaddad_configsets.m` because the T04 / T05 result caches hash the full text of
`shaddad_configsets.m` (an edit forces about 4 h of re-simulation). To merge them into the sets at creation,
the integration owner can add, in `local_codegen_sets` after `cg{j} = cs;` (full mode only):
`if full, cg{j} = stm32_target_sets('apply', cs); end`, and add `stm32_target_sets('defs')` to the cache key.

ROS 2 distribution: P_meta says JetPack 6 / ROS 2 Humble; the R2026b ROS Toolbox default install folder is
`/opt/ros/jazzy`. The generated node uses standard messages only (`std_msgs`, `sensor_msgs`, `geometry_msgs`),
but confirm that the ROS Toolbox release supports the distribution on the Jetson before the first remote
build (otherwise run Jazzy in a container or move to an Ubuntu 24.04 based JetPack).

## 6. Board builds (`build_targets`)

`rep = build_targets()` runs, per target, only when its products are present:

1. `VC_TARGET = 1` in base (restored afterwards), `shaddad_configsets('codegen')`, `stm32_target_sets('apply')`.
2. Board setup of the HW-variant driver blocks, in memory (the model files are never saved):
   `build_targets('peripherals','apply',<target>)` compares every driver block with the CubeMX pin map
   (`stm32_ioc_configure('pinmap', ...)`) and sets the mismatching parameters. The 17 MCU driver parameters of
   T08 (`fw_io_in_hw` / `fw_io_out_hw`: DI GPIOF[0:13], DO GPIOG[0:15] / GPIOE[0:3], ADC1 8 conversions,
   ADC3 1, USART2, I2C1 400 kHz, TIM1, TIM4, FDCAN1/2) already match; the 18 BMS parameters of T04
   (`Shaddad_BMS` IO_In / IO_Out HW placeholders) are set here: AFE_SPI SPI2 master mode 3, 664062 bit/s,
   chip select PB12 by explicit GPIO calls, 64-byte transfer; ADC1 6 conversions at Ts.bms_fast; DI GPIOC[4:11]
   at Ts.bms_fast; DO GPIOB [4 5 6 7 10].
   MCU only: `build_targets('registers','apply')` selects the registers of the six
   `stmutilitieslib/Register Read//Write` placeholders of T08 (in memory, the SSR files are not saved):
   `fw_io_in_hw/TIM2_CNT` TIM2.CNT read (Ts.base), `RCC_RSR` RCC.RSR read (Ts.base), `RMVF_write/RCC_RSR_RMVF`
   RCC.RSR write (RMVF, inherited), `FDCAN1_PSR` / `FDCAN2_PSR` PSR read (Ts.ctrl), `fw_io_out_hw/IWDG1_refresh/IWDG1_KR`
   IWDG1.KR write 0xAAAA (inherited). Writes get their value signal re-wired from the placeholder Terminator.
   This needs STM32CubeCLT (device SVD list); without it the mask callbacks fail (verified) -> BLOCKED.
3. A temporary `Simulink.ConfigSetRef 'Shaddad_Build'` -> board set on the top model and every referenced model,
   `slbuild(top)`, then the models are restored (never saved).
4. Report: status BUILT / BLOCKED / FAILED with reasons; code size (MCU / BMS: `arm-none-eabi-size -B` of the
   ELF, flash = text + data, RAM = data + bss; companion: generated C++ size, compiled remotely); task-timing
   estimate; PIL readiness (products, STM32CubeProgrammer, serial module USART3; the board connection is checked
   at run time).

Acceptance (build plan T17): the three `slbuild` calls finish without errors, MCU CPU load estimate < 50 % at
480 MHz, flash < 1 MB (`P.electronics.target.cpu_load_max`, `flash_max`).

Task-timing design estimate (`build_targets('estimate')`, spec 2.7.1 mapped to the revision-B task set; to be
replaced by the Execution Profiler measurement of T-EL-42):

| MCU task | Period | WCET est. | Load |
|---|---|---|---|
| Task_1k (SBUS, DI/DO, watchdog, t_us) | 1 ms | 25 us | 2.5 % |
| Task_5 (ICM-20948 I2C read, 400 kHz blocking about 0.6 ms, tilt) | 5 ms | 650 us | 13.0 % |
| Task_10 (RC mapping, mode manager, control, CAN1) | 10 ms | 380 us | 3.8 % |
| Task_50 (CAN2, BMS, implement, ADC) | 50 ms | 100 us | 0.2 % |
| Task_100 (aux, thermal, MCU_POWER) | 100 ms | 50 us | 0.05 % |
| **Total** | | | **19.6 %** (limit 50 %) |

BMS (170 MHz): 10 ms task with the blocking 64-byte AFE SPI transfer (0.77 ms) 1.0 ms -> 10 %, total 10.2 %.

`build_targets('preflight')` runs, in a fresh MATLAB process (the shared session is not touched), an update
diagram of each model with `VC_TARGET = 1` and its board set attached to the whole model hierarchy. It is the
compile check that is possible without Simulink Coder: a failure that names a missing build product is
BLOCKED, any other error FAILED.

## 7. Companion: SocketCAN-FD bridge node contract (D38, ICD 5)

The bridge is an external ROS 2 node on the Jetson (not generated from the models). `companion_io_ros2`
(T10) is its counterpart inside `Shaddad_Companion`.

* **Interface:** `can0` as CAN-FD, 1 Mbit/s nominal / 2 Mbit/s data, BRS, sample points 0.8 / 0.8 to match
  FDCAN2: `ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 2000000 dsample-point 0.8 fd on restart-ms 100`.
  Bus-off recovery by the kernel (`restart-ms`); the MCU reports its own bus-off (FDCAN2 PSR.BO -> fault bit 26).
* **Message type:** `std_msgs/UInt8MultiArray`, `layout.dim` empty, `data` exactly **69 bytes**:
  bytes 0-3 CAN identifier (uint32, little-endian, 11-bit standard IDs), byte 4 payload length (uint8, a valid
  CAN-FD length 0-8, 12, 16, 20, 24, 32, 48, 64), bytes 5-68 payload (64 bytes, zero padded)
  (`P.electronics.can.hw.msg_len`).
* **`/can2/rx` (bridge -> companion):** one message per received frame with ID 0x200 MCU_STATUS (48 B, 50 ms),
  0x201 MCU_IMU (48 B, 5 ms), 0x202 MCU_ODOM (32 B, 10 ms), 0x203 MCU_POWER (64 B, 100 ms); other IDs are
  dropped. QoS reliable, keep last 16 (`P.electronics.can.hw.qos_depth`). The model samples the subscriber
  every 1 ms and keeps the newest message, so the bridge must space consecutive messages by at least
  **2 ms** (`P.electronics.can.hw.min_gap`): about 330 frames/s average fit (max 500 messages/s), bursts are
  queued by the bridge (worst case 4 queued frames = +8 ms latency).
* **`/can2/tx` (companion -> bridge):** COMP_CMD 0x100, 32 B, every 50 ms (published only when the frame is
  valid); the bridge writes each message as one CAN-FD frame with BRS and the DLC of `len` to `can0`.
* **Timeouts:** the MCU stops AUTO when the COMP_CMD `seq` does not change for 0.2 s (COMP_TIMEOUT); the
  companion marks `Bus_McuStatus.valid = false` after 0.2 s without MCU_STATUS.

Other companion inputs (HW variants, standard messages): ublox ROS 2 driver `/fix` (`sensor_msgs/NavSatFix`),
`/fix_velocity` (`geometry_msgs/TwistWithCovarianceStamped`), `/navheading` (`sensor_msgs/Imu`) for
`Shaddad_NavFusion` (T12); Ouster driver `/ouster/points` (`sensor_msgs/PointCloud2`, 32 x 1024) for
`Shaddad_Perception` (T13; scan time = ROS reception time).

## 8. Bring-up order

Follow the power-up / re-arm sequence of ICD 6 (D30) at every step; keep the tracks off the ground
until step 9.

1. **Tools:** install the products of section 2; `build_targets('products')` must list nothing missing.
   Open both `.ioc` files in STM32CubeMX (no pin conflicts), regenerate. `build_targets('preflight')` OK.
2. **MCU alone (NUCLEO-H743ZI2, bench supply):** `build_targets('registers','apply')`, `build_targets('MCU')`
   (flash < 1 MB), flash with ST-LINK. Check the 1 MHz TIM2 tick, IWDG1 reset after a stalled task (the
   MCU_STATUS flags b7 wdg_reset_seen on the next boot; fault bit 22 is STOP_FAIL since D71), DO all low after reset. Read TS_CAL1/TS_CAL2 (2 bytes each at
   0x1FF1E820 / 0x1FF1E840, STM32CubeProgrammer) and put them into `P.electronics.adc.ts_cal_cnt`.
3. **DI / DO / ADC:** loop-back harness for GPIOF[0:13] and GPIOG/GPIOE; ADC channels against a calibrated
   source (divider and Hall scaling of ICD 6).
4. **SBUS:** receiver on USART2 (inverted line): T-EL-43 (10 min without bad frames, frame period
   14.0 +/- 0.1 ms, failsafe flag with the TX off).
5. **IMU:** ICM-20948 on I2C1 at 400 kHz; tilt estimate at rest.
6. **PIL:** T-EL-42 with the Execution Profiler: Task WCETs within the estimate table, total CPU < 30 %,
   0 overruns in 10 min.
7. **BMS board:** `build_targets('BMS')`, AFE on a cell simulator over SPI2 (isoSPI), CAN1 link to the MCU
   (BMS_REQ 0x3D0, BMS frames 0x351..0x3C1), contactor outputs on LEDs first, then precharge into a resistive
   load (T-EL-37).
8. **CAN1 drives:** one ESC at reduced current limit, DRV_CMD / FB frames, timeouts (T-EL-35), both ESCs.
9. **CAN2 + companion:** bridge node of section 7 on the Jetson, `build_targets('Companion')` (remote colcon
   build), MCU_STATUS / COMP_CMD exchange, COMP_TIMEOUT test (T-EL-32); then GNSS and LiDAR drivers.
10. **Vehicle:** tracks off the ground (RC MANUAL, E-stop chain T-EL-28/29), then ground tests in MANUAL,
    then AUTO handover (T-EL-30) and the scenario set S0-S8.

## 9. Open items found during T17 (reported to the owners)

Preflight results (fresh MATLAB process, update diagram with `VC_TARGET = 1`, 2026-10-22):

| Model | Board set | Result |
|---|---|---|
| `Shaddad_MCU` | `cs_STM32H7` | BLOCKED: "No valid STM32CubeCLT installation detected" (mask initialisation of the Register Read//Write blocks) |
| `Shaddad_BMS` | `cs_STM32G4` + board setup | FAILED (T04): without the board setup the FDCAN Read Data output is [64x1] while `Bus_CanFrame8.data` has 8 elements (`IO_In/HW/rx_data` -> `Frame_BC`); with the board setup (DI GPIOC[4:11]) the DI decode fails first (`DI_u16` [1x8] into the 16-bit Integer to Bit `DI_bits`: "Multichannel operation is not supported") |
| `Shaddad_Companion` (+ `companion_io_ros2`, `Shaddad_GNC`, `Shaddad_NavFusion`, `Shaddad_Perception`) | `cs_Companion` | FAILED (T10): algebraic loop GNC -> `Companion_IO/Comp_Pack` -> `FrameIO/HW/CAN2_ROS2` -> mcu_status -> GNC input 3, also with the simulation set; with a Unit Delay (Ts.gnc, IC `P.ic.Bus_CanFrame64`) on mcu_status after `CAN2_ROS2` (inserted in memory) the update passes with `cs_Shaddad` and with `cs_Companion` (ROS 2 board, ert C++, ARM64), so the ROS 2 message buses of the HW variants resolve in a fresh session (T13 question) |

* **Companion HW loop (T10).** In SIM the MCU_STATUS frame is delayed by one 50 ms sample in `Link_MCU_GNC`
  (ICD 5); the HW choice of `companion_io/FrameIO` has no delay, and the `companion_io_ros2` Model block is
  treated as direct feedthrough, which closes an algebraic loop through `Shaddad_GNC`. Fix: a Unit Delay
  (SampleTime Ts.gnc, InitialCondition `P.ic.Bus_CanFrame64`) between `CAN2_ROS2/3` and `mcu_status_f`
  (verified in memory, see above).
* **BMS IO_In HW frame and DI word (T04).** Add a Selector 1:8 after the FDCAN Read Data output; the DI decode
  (`DI_u16` -> Integer to Bit 16) expects a port word, while `Digital Port Read` with `IOAsArray on` and the
  GPIOC[4:11] pin map returns an 8-element vector.
* **FDCAN Read output order (T08, T04).** The blockset class `stm32cube.blocks.FDCANBlock`
  (`getOutputNamesImpl`) orders the Unpacked outputs **Data, Length, Id**, then the optional outputs, then
  Status. `fw_io_in_hw/CAN1_RX|CAN2_RX` wire output 2 to `Rx_id` and 3 to `Rx_len` (swapped), and
  `Shaddad_BMS` IO_In HW wires `FDCAN1_Read/2` to `rx_id` and `/3` to `rx_len` (swapped).
* **FDCAN Write input order (T04).** In Raw-data mode with length and identifier from ports the inputs are
  **Data, Length, Id**; `Shaddad_BMS` IO_Out HW connects (id, data, len) to inputs 1-3.
* **Register placeholders (T08):** library defaults (STM32F4xx / ADC_Common.CSR, sample time 1 s) until the
  `registers` step of section 6 runs with STM32CubeCLT.
* **TS_CAL (T08):** `P.electronics.adc.ts_cal_cnt` stays a placeholder until the per-device words are read
  (step 2).
