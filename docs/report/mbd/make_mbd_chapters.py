"""Write the chapter JSON files of the model-based design report of the real Shaddad robot.

build/build_mbd.js (made by patch_builder_mbd.py) reads mb*.json from this folder. Values come from
docs/hardware/real_robot_components.md and the parameter files (state of 2026-10-27).
Run: python -I make_mbd_chapters.py [verification.json]
"""
import json
import os
import sys

D = os.path.dirname(os.path.abspath(__file__))
VER = json.load(open(sys.argv[1], encoding='utf-8')) if len(sys.argv) > 1 else None

chapters = []

# ======================================================================================= 1 purpose and sources
chapters.append({"id": "mb01_intro", "title": "Purpose, Scope and Sources", "blocks": [
    {"type": "p", "text": "This report describes the Shaddad robot as it really is: the **XMachines MAX** tracked platform used by NASL, with its sprayer module, battery, traction drive and electronics. For each component it sets the real part and its specification next to the way the component is represented in the `Shaddad_UGV` MATLAB/Simulink/Simscape digital twin. It also explains the model-based design (MBD) method: one parameter source drives generated models, and tests check every change."},
    {"type": "p", "text": "The model was re-parameterised to the real robot on 27 October 2026 from the data received that day: the dimension and weight sheet of the platform, the traction-motor nameplate, the battery specification sheet and battery-box drawing, and photos of the sprayer pump and the electronics. Before that date the model used design estimates for a generic tracked carrier."},
    {"type": "h2", "text": "Source data"},
    {"type": "table", "id": "tab_sources", "caption": "Source data received on 2026-10-27 (copies in `docs/hardware/source/`)", "header": ["Source", "Content", "Used for"], "widths": [30, 38, 32], "rows": [
        ["Platform spec sheet (photo)", "XMachines MAX: 1730 x 780 x 570 mm (robot with towed module), robot 220 kg, module 50 kg", "Body size, total mass, sprayer-module mass"],
        ["Motor nameplate (photo)", "Movematic BLDC, P/N 080-240-0x, 48 V DC, 750 W, 3000 rpm, S/N CFH0136", "Motor ratings, gear ratio for the top speed"],
        ["Drive layout (user)", "four motors, two per track on the front and rear sprockets; one dual-channel ESC per side", "Drive architecture and mass layout"],
        ["Battery spec sheet (PDF)", "13S15P Molicel P42A, 46.8 V, 63 Ah, about 3 kWh; BMS 120-150 A cont. / 200-300 A peak, CAN; charger 54.6 V 20 A; SB120 / SB50 / GX-12 connectors", "Cell model, pack, BMS limits, charger"],
        ["Battery box drawing XMT-P1-E01-000 (PDF)", "393 x 358.5 x 87 mm, 3 mm aluminium sheet", "Pack mass split, position, thermal model"],
        ["Electronics photos", "XT_PS V1.0 board with STM32 Nucleo-32, STM32 USB-C board, FT232RL USB-UART, ADS1115 ADC, LM2596 buck module, SPR V1.3 sprayer board", "Controller architecture, I/O, supply rails (3-D renders in {sec:mb06_electronics})"],
        ["Sprayer photo", "Brass piston pump with pressure regulator, 12 x 8 mm high-pressure hose", "Sprayer hydraulics"],
    ]},
    {"type": "h2", "text": "Value tags"},
    {"type": "p", "text": "Every number in the data sheet `docs/hardware/real_robot_components.md` and in this report carries one of two tags:"},
    {"type": "bullets", "items": [
        "**[DATA]**: read from a source file (the file is named);",
        "**[EST]**: estimated from the drawings, the photos or typical parts of this class. An [EST] value stays in the model until a measurement or a data sheet replaces it ({sec:mb10_open}).",
    ]},
    {"type": "h2", "text": "The robot at a glance"},
    {"type": "figure", "file": "M16_real_components_collage.jpg", "caption": "Real components of the robot: (a) traction motor nameplate, (b) battery in its XMachines box, (c) sprayer pump under the chassis; (d)-(f) electronics as 3-D renders built from the photos, because the boards are glued into their box (section 6)."},
    {"type": "table", "id": "tab_glance", "caption": "Key specifications: real robot and model", "header": ["Item", "Real robot", "Model (params/)", "Tag"], "widths": [24, 30, 32, 14], "rows": [
        ["Platform", "XMachines MAX, 2 rubber tracks, 3 road wheels per side", "6-DOF body on 2 x 7 track contact points", "[DATA]"],
        ["Size (robot)", "1.00 x 0.78 x 0.57 m", "gauge 0.60 m, track 0.13 x 0.75 m", "[DATA] / [EST]"],
        ["Mass", "220 kg robot + 50 kg module", "226 kg with the 1.0 kW drives; 276 / 316 kg with empty / full tank", "[DATA] / [EST]"],
        ["Drive", "4 x Movematic BLDC 48 V, 750 W (two per track), dual ESC per side", "4 x 1.0 kW BLDC (proposed upgrade), gearbox 20:1, one equivalent drive per side", "[DATA] / [EST]"],
        ["Top speed", "about 5 km/h", "1.41 m/s at 3000 rpm", "[EST]"],
        ["Battery", "13S15P Molicel P42A, 46.8 V, 63 Ah", "Simscape Battery, 13 cell groups", "[DATA]"],
        ["BMS / charger", "150 / 250 A, CAN; charger 54.6 V 20 A", "Shaddad_BMS, CC-CV charge", "[DATA]"],
        ["Controller", "XT_PS V1.0 board, STM32 Nucleo-32", "Shaddad_MCU firmware model", "[DATA]"],
        ["Implement", "sprayer module, piston pump", "Impl_Sprayer, 40 L tank, 8 nozzles", "[DATA] / [EST]"],
    ]},
]})

# ======================================================================================= 2 model-based design
chapters.append({"id": "mb02_mbd", "title": "Model-Based Design Approach", "blocks": [
    {"type": "p", "text": "The digital twin follows four model-based design rules. Together they make a change to the real robot (a new motor, a heavier pack, a different gear) a parameter edit, not a model edit:"},
    {"type": "numbered", "items": [
        "**One parameter source.** Every number lives in `params/param_<domain>.m`; `shaddad_params` collects them into the structure `P`, the sample times `Ts` and the variant controls `VC_*`. Models read `P`; they hold no numbers of their own.",
        "**Generated models.** Every `.slx` file is generated by a script in `scripts/build/` and is never edited by hand. A structural change is made in the build script and the model is regenerated.",
        "**Variants for fidelity.** Variant controls select the level of detail per domain (for example the grouped or lumped battery, the drive fidelity, the implement), so the same model serves fast system runs and detailed component studies.",
        "**Tests on every change.** Unit benches per domain, system scenarios and the Phase 1 suite compare the model with requirements; expected values are derived from `P`, so they follow the real robot automatically.",
    ]},
    {"type": "figure", "file": "M28_mbd_workflow.png", "caption": "Model-based design workflow of the project: real-robot data feed the parameters, the parameters drive the generated models, tests verify them, and measurements on the robot close the calibration loop."},
    {"type": "h2", "text": "Model hierarchy"},
    {"type": "p", "text": "`Shaddad_System` is the top model. It references the plant (`Shaddad_Plant`: structure, drive, power and electronics hardware), the firmware of the main controller (`Shaddad_MCU`) and the BMS firmware (`Shaddad_BMS`). Inside them, subsystem references hold the component models of each domain ({fig:F01_system_top_overview.png})."},
    {"type": "figure", "file": "F01_system_top_overview.png", "caption": "Top model Shaddad_System (overview of the full diagram); the numbers mark the blocks of {tab:tab_top}."},
    {"type": "table", "id": "tab_top", "caption": "Blocks of Shaddad_System and what they represent", "header": ["#", "Block", "Represents"], "widths": [6, 24, 70], "rows": [
        ["1", "`ScenarioIn`", "scenario input: operator command, body HMI (E-stops, key, reset), faults, environment"],
        ["2", "`Operator_Console`", "source of the operator command: scripted plan or live joystick (PS5 controller, RC transmitter)"],
        ["3", "`Operator_RC`", "RC transmitter, radio and receiver: SBUS byte stream every 14 ms"],
        ["4", "`MCU_In`", "input rate transitions of the main controller"],
        ["5", "`MCU` (`Shaddad_MCU`)", "firmware of the main controller (XT_PS board with the Nucleo-32)"],
        ["6", "`BMS` (`Shaddad_BMS`)", "firmware of the BMS in the battery box"],
        ["7", "`Plant` (`Shaddad_Plant`)", "chassis, tracks and terrain, drives, battery, PDU, LV rails, electronics hardware, sprayer"],
        ["8", "`CAN1_Net`", "CAN1 bus between MCU, ESCs and BMS, with frame drop and latency"],
        ["9", "`Sensors`", "IMU, GNSS, LiDAR and spray-detector models"],
        ["10", "`CAN2_Link`", "CAN-FD link to the companion computer (Phase 2)"],
        ["11", "`MissionCmd_RT`", "rate transition of the mission command"],
        ["12", "`GNC_Variant`", "guidance stub (Phase 1) or the companion model (Phase 2)"],
        ["13", "`Sim_Monitors`", "KPIs, collision and clearance check, energy audit"],
        ["14", "`Viz_Logging`", "bird's-eye view and logging"],
        ["15", "`*_Delay`", "unit delays that break the algebraic loops between the models"],
    ]},
    {"type": "table", "id": "tab_models", "caption": "Models and the real components they represent", "header": ["Model", "Represents", "Built by"], "widths": [26, 46, 28], "rows": [
        ["`Shaddad_System`", "the whole robot with its operator and radio link", "`build_shaddad_system`"],
        ["`Shaddad_Plant`", "chassis, tracks and terrain; traction drives; battery, PDU, LV rails, charger; sensors and electronics hardware; implement", "`build_shaddad_plant`"],
        ["`Shaddad_MCU`", "firmware of the main controller (XT_PS board with the STM32 Nucleo-32)", "`build_shaddad_mcu('model')`"],
        ["`Shaddad_BMS`", "firmware of the 13S battery-management system", "`build_shaddad_bms('model')`"],
        ["`Pack_Grouped` / `Pack_Lumped`", "13S15P battery module (13 cell groups or one lumped cell)", "`build_shaddad_battery`, `build_power_pack`"],
        ["`Impl_Sprayer`", "sprayer module: tank, pump, boom", "`build_power_plant`"],
    ]},
    {"type": "h2", "text": "Variants and fidelity"},
    {"type": "table", "id": "tab_variants", "caption": "Variant controls used for the real robot", "header": ["Control", "Choices", "Use"], "widths": [18, 40, 42], "rows": [
        ["`VC_PHASE`", "1 RC (Phase 1), 2 autonomy", "Phase 1: remote control, GNC stub"],
        ["`VC_PACK`", "Grouped (13 cell groups), Lumped", "Grouped for battery studies, Lumped for long runs"],
        ["`VC_DRIVE_FID`", "detailed ESC and motor, average drive", "average drive for system scenarios"],
        ["`VC_IMPL`", "0 none, 1 mower, 2 tiller, 3 sprayer", "3 for the real robot's sprayer module"],
        ["`VC_OPSRC`", "scripted plan, live joystick", "live drive with a PS5 controller or RC transmitter"],
    ]},
    {"type": "h2", "text": "From the real data to the model"},
    {"type": "p", "text": "The re-parameterisation of 27 October 2026 touched only parameter files, three build scripts (for the new number of cell groups) and the expected values of the tests; the four-motor drive and the motor upgrade were again parameter changes, the tilt fix rebuilt one firmware unit, and the brake model of decision D64 added one Simscape component and rebuilt the brake subsystem of the drive. {tab:tab_changes} lists the changes; the previous robot is kept in `work/backup/pre_real_robot_20261027.zip`."},
    {"type": "table", "id": "tab_changes", "caption": "Changes made for the real robot (2026-10-27)", "header": ["Area", "Change", "Files"], "widths": [16, 56, 28], "rows": [
        ["Structure", "1.00 x 0.78 x 0.57 m body, 220 kg mass table, gauge 0.60 m, track 0.13 m wide, 0.75 m on the ground, sprocket 0.090 m, 3 road wheels, sprayer module 50 kg + 40 L tank", "`param_structure.m`"],
        ["Gauge factor", "re-identified on the new geometry: farm soil 1.394 (1.32-1.49 over the soils)", "`param_structure.m`"],
        ["Drive", "Movematic 750 W motor, gearbox 20:1, ESC 20 / 50 A rms, brake 5 N m, speed-loop gains scaled with the new inertia", "`param_drive.m`"],
        ["Drive, second change", "four motors, two per side on the front and rear sprockets, one dual ESC per side, modelled as one equivalent drive of two units per side; proposed 1.0 kW unit; traction fuses 125 A; four drive units in the mass table", "`param_motor_unit.m`, `param_drive.m`, `param_structure.m`, `param_power.m`"],
        ["Tilt (S2)", "IMU level calibration subtracted in `fw_imu_tilt`; TILT_WARN 22 to 25 deg", "`param_electronics.m`, `build_fw_platform.m`"],
        ["Drive tuning and brake (D64)", "speed PI re-tuned for two motors per side (Kp 0.030, Ki 0.090); elasto-plastic brake model (Simscape component); track relaxation length = L/2; drive and battery test benches take their load and pack from `P`", "`param_drive.m`, `build_drive.m`, `+shaddad_drive/elastoplastic_brake.ssc`, `param_structure.m`, `tests/drive`, `tb_pack_bms`"],
        ["Battery", "13S15P Molicel P42A (21700 cells), NMC tables, BMS 150 / 250 A, charger 54.6 V 20 A; 13 cell groups in the buses, BMS and pack", "`param_power.m`, `buses_power.m`, 3 build scripts"],
        ["MCU limits", "top speed 1.41 m/s (manual 1.4, AUTO 1.2), overspeed 1.70 m/s, DCL warning 25 A", "`param_electronics.m`"],
        ["GNC, sensors", "footprint, obstacle corridor and Stanley wheelbase from the geometry; IMU, LiDAR and self-box on the new body", "`param_gnc.m`, `param_sensors.m`"],
        ["3-D view", "real shape: tracks with 3 road wheels, red body, deck rail, 40 L tank", "`Robot3DView.m`"],
        ["Tests", "expected values derived from `P` instead of fixed numbers", "`tests/`, `scn_S2..S5`"],
    ]},
    {"type": "h2", "text": "System architecture as modelled"},
    {"type": "p", "text": "{fig:M27_architecture_real.png} shows the robot's architecture as it is modelled, with the real part named in each block. Signals and CAN data are navy, the 48 V power path is purple, the hard-wired safety chain is red, and the Phase 2 additions (GNSS, LiDAR, companion computer) are grey."},
    {"type": "figure", "file": "M27_architecture_real.png", "caption": "Architecture of the real robot as modelled in Shaddad_System (Phase 1): component, real part and model."},
]})

# ======================================================================================= 3 vehicle
chapters.append({"id": "mb03_vehicle", "title": "Vehicle: Chassis, Tracks and Mass", "blocks": [
    {"type": "p", "text": "The XMachines MAX is a rubber-tracked carrier with a sprocket and an idler at the ends of each track and three road wheels between them. The red body carries the battery and the electronics; a deck rail on top takes the implement. The spec sheet ({fig:M01_spec_sheet.png}) gives the overall size and the masses; the track geometry was scaled from its side and front views."},
    {"type": "figure", "file": "M01_spec_sheet.png", "caption": "XMachines MAX dimension and weight sheet (source photo): 1730 x 780 x 570 mm with the towed module, 220 kg robot + 50 kg module."},
    {"type": "table", "id": "tab_vehicle", "caption": "Vehicle: real robot and model", "header": ["Item", "Real robot", "Model value", "Tag"], "widths": [27, 30, 29, 14], "rows": [
        ["Overall size", "1730 x 780 x 570 mm with the towed module", "body 1.00 x 0.78 x 0.57 m", "[DATA]"],
        ["Track gauge B (centre to centre)", "from the front view", "0.60 m", "[EST]"],
        ["Track width b / contact length L", "from the side view", "0.13 m / 0.75 m", "[EST]"],
        ["Sprocket pitch radius", "from the side view", "0.090 m", "[EST]"],
        ["Road wheels", "3 per side", "3 per side, radius 0.077 m", "[DATA] / [EST]"],
        ["Ground clearance", "-", "0.12 m", "[EST]"],
        ["Mass, robot", "220 kg", "220.0 kg as built; 226.0 kg with the 1.0 kW drives (14-part mass table)", "[DATA] / [EST]"],
        ["Mass, sprayer module", "50 kg dry", "50 kg + 40 kg liquid (40 L tank)", "[DATA] / [EST]"],
        ["Centre of gravity", "-", "x = +0.008 m, height 0.27 m (robot alone)", "computed"],
        ["Top speed", "about 5 km/h", "1.41 m/s at 3000 rpm motor speed", "[EST]"],
    ]},
    {"type": "figure", "file": "M26_dimensions.png", "caption": "Model geometry of the real robot: side and top views with the dimensions used by the structure model and the centre of gravity."},
    {"type": "h2", "text": "Mass breakdown"},
    {"type": "p", "text": "Only the total of 220 kg and the 50 kg module are known. The model splits the total into fourteen rigid parts with their own positions and sizes, so the centre of gravity and the inertia follow from the layout ({fig:M20_mass_breakdown.png}). The battery (18.5 kg) is computed from its 195 cells, its 3 mm aluminium box and the BMS and busbars; the four drive units sit at the sprockets, and the chassis frame takes the remainder of the as-built 220 kg (81.7 kg). The proposed 1.0 kW units add 1.5 kg each, so the modelled robot weighs 226 kg. The configurations add the implement and its liquid."},
    {"type": "figure", "file": "M20_mass_breakdown.png", "caption": "Mass model: (a) the mass table with the four 1.0 kW drive units (226 kg), (b) total mass per configuration; the mower and the tiller are model implements that the real robot does not carry."},
    {"type": "h2", "text": "Structure and track model"},
    {"type": "p", "text": "The structure model (`structure_plant`) is a 6-degree-of-freedom rigid body on two tracks. Each track touches the ground at 7 contact points along its 0.75 m contact length. At every point the model computes the normal load from the terrain height map (DEM) and the sinkage, and the tractive force from the slip with the Janosi-Hanamoto shear law and a Dahl bristle state for the transition through zero speed. The shear displacement builds up over a relaxation length of half the contact length (0.375 m). Until this version it had kept the 0.50 m of the former 1.00 m track, so the tracks slipped like a longer track: 25 % less slip than the analytic reference of unit test T4. Soil classes (sand, sandy loam farm soil, wet loam, greenhouse floor, asphalt) set the cohesion, friction angle and shear modulus."},
    {"type": "figure", "file": "F12_structure_plant.png", "caption": "Structure plant model: rigid body, track contact points and terrain."},
    {"type": "h2", "text": "Skid-steer gauge factor"},
    {"type": "p", "text": "A tracked robot turns by driving its tracks at different speeds. Because the tracks slip sideways while turning, the effective gauge is larger than the geometric 0.60 m. The ratio is the gauge factor chi = B_eff / B; the MCU, the navigation filter and the guidance use it to convert track speeds into a yaw rate. `structure_identify_chi` identified it with the high-fidelity track model on the real geometry: **1.394 on farm soil** (1.32-1.49 over the five soils). It is larger than on the former, wider model geometry, as {fig:M25_chi_soils.png} shows. It was identified before the relaxation-length fix; the high-fidelity model now gives 1.417 for the same 2 m turn (1.7 % higher), so a new identification is listed as an open item."},
    {"type": "figure", "file": "M25_chi_soils.png", "caption": "Skid-steer gauge factor per soil: former model geometry, real geometry (2 m turn radius, the value used by the firmware) and real geometry in a spot turn."},
    {"type": "h2", "text": "3-D view of the model"},
    {"type": "p", "text": "The live 3-D view (`shaddad.Robot3DView`) draws the model with the real robot's shape: tracks with three road wheels, the red body, the deck rail and, with the sprayer module, the 40 L tank and the boom. It moves with the simulated pose and belt speeds during a run ({fig:M33_robot3d_side.png})."},
    {"type": "figure", "file": "M33_robot3d_side.png", "caption": "Live 3-D view of the real-robot model (side camera, MANUAL mode, both belts at 0.8 m/s)."},
]})

# ======================================================================================= 4 drive
chapters.append({"id": "mb04_drive", "title": "Traction Drive", "blocks": [
    {"type": "p", "text": "The robot has **four traction motors**, two per track: one drives the front sprocket and one the rear sprocket, each through its own gearbox. One dual-channel motor controller (ESC) per side drives the two motors of that side with the same command. The nameplate ({fig:M02_motor_nameplate.jpg}) gives the rated point of the installed motors: 48 V DC, 750 W, 3000 rpm. The gear ratio is not marked; 20:1 gives the robot its known top speed of about 5 km/h with the 0.090 m sprockets."},
    {"type": "figure", "file": "M02_motor_nameplate.jpg", "caption": "Nameplate of the installed traction motor: Movematic, S/N CFH0136, P/N 080-240-0x, 48 Vdc, 750 W, 3000 rpm."},
    {"type": "h2", "text": "Motor sizing for the application"},
    {"type": "p", "text": "`scripts/motor_sizing.m` computes the torque each motor needs at the duty points of a spraying shift on a hilly farm: a 316 kg robot with a full tank, the farm-soil motion resistance measured in the model (0.107 of the weight), skid-steer turns, and 20 deg slopes ({tab:tab_sizing}). Weighted over the cycle, the RMS torque per motor is 1.45 N m on farm soil and 1.81 N m on wet loam."},
    {"type": "table", "id": "tab_sizing", "caption": "Torque per motor at the duty points (four motors, 316 kg)", "header": ["Duty point", "Speed", "Farm soil", "Wet loam", "Share of the cycle"], "widths": [36, 14, 16, 16, 18], "rows": [
        ["flat spraying", "1.0 m/s", "0.41 N m", "0.77 N m", "50 %"],
        ["20 deg climb", "1.0 m/s", "1.71 N m", "2.05 N m", "25 %"],
        ["20 deg climb + 0.8 m/s² acceleration", "0.5 m/s", "2.03 N m", "2.36 N m", "5 %"],
        ["pivot turn on flat ground", "0.3 m/s", "1.62 N m", "2.22 N m", "10 %"],
        ["turn on the 20 deg slope", "0.5 m/s", "2.85 N m", "3.41 N m", "10 %"],
        ["RMS of the cycle", "", "1.45 N m", "1.81 N m", ""],
    ]},
    {"type": "p", "text": "In the 45 degC summer heat the 750 W motor gives about 2.15 N m continuously. That leaves an RMS margin of 1.19 on wet loam, and every turn on a slope runs above the motor's continuous rating. The model therefore uses a **1.0 kW, 48 V, 3000 rpm** unit (3.18 N m continuous, 9.55 N m peak), which raises the margin to 1.58 and covers the slope turns on farm soil within its continuous rating ({fig:M21_motor_sizing.png}). It keeps the 48 V bus, the 20:1 gear and the 5 km/h top speed, and its currents stay within the 150 / 250 A BMS; a 1.5 kW unit would need a larger BMS. The installed 750 W motor stays available as a model option (`shaddad_params('Motor','movematic750')`)."},
    {"type": "figure", "file": "M21_motor_sizing.png", "caption": "Motor sizing per unit: peak and continuous torque (25 degC and 45 degC) of the installed 750 W motor and the proposed 1.0 kW unit, with the duty points on farm soil and wet loam and the RMS torque of the wet-loam cycle."},
    {"type": "table", "id": "tab_drive", "caption": "Traction drive per motor: installed and proposed (the model uses the 1.0 kW unit)", "header": ["Item", "Installed (Movematic)", "Proposed 1.0 kW unit", "Tag"], "widths": [24, 30, 32, 14], "rows": [
        ["Rated point", "48 V, 750 W, 3000 rpm, 2.39 N m", "48 V, 1.0 kW, 3000 rpm, 3.18 N m", "[DATA] / [EST]"],
        ["Peak torque", "7.16 N m (3 x rated)", "9.55 N m (3 x rated)", "[EST]"],
        ["No-load / max speed", "3600 rpm at 48 V / 4000 rpm", "same", "[EST]"],
        ["Flux, resistance, inductance", "0.0184 Wb, 64 mOhm, 0.25 mH", "0.0184 Wb, 48 mOhm, 0.19 mH", "[EST]"],
        ["Rotor inertia, motor mass", "2.0e-4 kg m², 3.5 kg", "2.8e-4 kg m², 4.5 kg", "[EST]"],
        ["Gearbox (one per motor)", "20:1, efficiency 0.90, T2B 140 N m", "20:1, efficiency 0.90, T2B 180 N m", "[EST]"],
        ["ESC channel (two per dual ESC)", "20 A rms cont., 50 A rms peak, 40 A DC", "27 A rms cont., 65 A rms peak, 55 A DC", "[EST]"],
        ["Holding brake", "5 N m per motor", "5 N m per motor", "[EST]"],
        ["Thermal limits", "winding derate 140 degC, fault 165 degC", "same", "[EST]"],
    ]},
    {"type": "h2", "text": "Drive model"},
    {"type": "p", "text": "The two motors of a side drive the same track, so they turn at the same speed and get the same command from the dual ESC. The model represents each side by **one equivalent drive of two units in parallel**: torque, power, current limits, rotor inertia, iron losses, brake torque and thermal capacity are doubled, and the winding resistance and inductance are halved, at the same speed and voltage. Per side, the drive model chains the ESC (CAN command, speed PI loop, current and power limits, voltage and temperature derating, fault logic), the motor (electrical and thermal model), the spring-applied brake and the gearbox with its losses and backlash, ending at the sprocket that drives the track model ({fig:F07_drive_side_overview.png}). The ESC ramp follows from the gear ratio, so the acceleration limits stay the same in track-speed terms."},
    {"type": "figure", "file": "F07_drive_side_overview.png", "caption": "Drive model of one side: ESC, motor, brake and gearbox (subsystem references); with four motors each block is the equivalent of the side's two units."},
    {"type": "h2", "text": "Speed loop and holding brake with two motors per side"},
    {"type": "p", "text": "Two motors per side change the drive's dynamics, so the ESC speed loop was checked again on the two-mass model of decision D50: the motor side (rotors, gearboxes, sprocket and belt) and the vehicle half-mass are coupled by the shear stiffness of the track on the ground (40-540 kN/m). Scaling the original D50 gains with the reflected inertia gave a peak sensitivity Ms of 1.65 on soft soil, above the 1.6 bound of test T21 (with one motor per side it had been 2.25). The loop was therefore re-tuned (decision D64) to **Kp 0.030 N m/(rad/s) and Ki 0.090 N m/rad**, the D50 ratio Ki/Kp = 3 ({tab:tab_speedloop}): the resonance peak drops below the D50 design target of 1.43 on every soil stiffness ({fig:M24_speed_loop.png}). A first re-tune with Ki 0.060 met every margin, but after a controlled stop its integrator still held about 0.2 N m per motor, so the robot crept at 0.02 m/s and reached standstill only 1.63 s after a radio loss (S6a, limit 1.5 s). The MCU's shaped reference and torque feed-forward carry normal driving, so the lower gains change the tracking little: the T1 ramp to 1.0 m/s overshoots 0.01 %, and the 2 m/s² stop profile is followed within 0.04 m/s."},
    {"type": "table", "id": "tab_speedloop", "keep": True, "caption": "ESC speed loop of the four-motor drive: worst case over the shear stiffness (40 / 61 / 540 kN/m) and the side mass (113 / 158 kg)", "header": ["Quantity", "D50 gains scaled with the inertia", "Re-tuned (D64)", "Target / bound"], "widths": [28, 25, 20, 27], "rows": [
        ["Kp / Ki", "0.048 / 0.144", "0.030 / 0.090", "-"],
        ["Rigid crossover", "12.8 rad/s", "8.0 rad/s", "-"],
        ["Phase margin", "57 deg", "62 deg", ">= 63 deg (D50)"],
        ["Gain margin", "10.9 dB", "15.0 dB", ">= 14.6 dB (D50)"],
        ["Peak sensitivity Ms", "1.65", "1.37", "<= 1.43 (D50), <= 1.6 (T21)"],
        ["Closed-loop damping", "0.44", "0.57", ">= 0.43 (D50), >= 0.35 (T21)"],
    ]},
    {"type": "figure", "file": "M24_speed_loop.png", "caption": "ESC speed loop of the four-motor drive on the two-mass model (40 kN/m shear stiffness, 158 kg per side): (a) sensitivity |S| with the D50 gains scaled by the inertia and with the re-tuned gains (D64); (b) a 5 rad/s motor-speed step on the two-mass bench (solid) and in the linear model (dashed)."},
    {"type": "p", "text": "The second effect is numerical. Each side now has two 5 N m spring brakes. The former brake model, a friction element whose torque changes direction within a breakaway velocity of 0.02 rad/s, does not settle into sticking at the plant's fixed 1 ms step: an engaged brake changes the sign of its torque at every step. With one brake per side this stayed a bounded ripple of about ±90 N m at the sprocket. With two it grew into spikes of up to 3500 N m on the T10 bench, and in the full system it diverged: in S8a the right motor spun up to 3400 rad/s while the robot stood in ESTOP with both brakes engaged, which latched an overspeed fault, so the robot could not resume after the reset ({fig:M29_brake_estop.png}a). Decision D64 therefore replaces the friction element by an **elasto-plastic brake**, a Simscape component written for this model (`shaddad_drive.elastoplastic_brake`). A bristle state holds the shaft like a stiff torsional spring (25 N m/rad per brake) while the torque stays below the static capacity, and lets it slide with the Stribeck curve above it; the torque scales with the engagement, so a released brake transmits nothing. The brake now holds without ripple and without creep (test T8: 0.0004 m/h over 600 s on 20 deg), and the stop from top speed loads the gearbox with at most 164 N m (T9)."},
    {"type": "figure", "file": "M29_brake_estop.png", "caption": "S8a in the full system (front hardware E-stop at 10 s, ESTOP with STO and both brakes engaged until the reset at 17 s): motor speeds of both drives (a) with the former friction element, whose stick-slip spins the right motor up to 3400 rad/s and ends the run in FAULT, and (b) with the elasto-plastic brake (D64)."},
    {"type": "h2", "text": "Efficiency and operating points"},
    {"type": "p", "text": "In test drive D1 (run `D1_20261027_212927`, chapter 8) the motors run between about 460 and 2550 rpm at 0.16-1.05 N m each (5-95 % of the run), a third of the 3.18 N m continuous rating or less ({fig:M22_motor_efficiency.png}). At these light loads the motor and its ESC channel turn 82 % of the electrical energy into shaft work (loss map, energy-weighted mean); the 85-90 % region starts above about 1 N m. The drive is sized for slopes and wet soil, not for flat spraying."},
    {"type": "figure", "file": "M22_motor_efficiency.png", "caption": "Efficiency of one 1.0 kW motor with its ESC channel (model loss map, [EST]) and the operating points of test drive D1_20261027_212927, per motor."},
]})

# ======================================================================================= 5 battery and power
chapters.append({"id": "mb05_battery", "title": "Battery, BMS and Power System", "blocks": [
    {"type": "p", "text": "The battery is a 13S15P pack of Molicel P42A 21700 cells: 15 cells in parallel make a 63 Ah group, and 13 groups in series give 46.8 V nominal and 54.6 V full, about 2.95 kWh. It sits in the XMachines aluminium box of drawing XMT-P1-E01-000 ({fig:M10_battery_box_drawing.png}) with the BMS, the SB120 main connector, the SB50 charge connector and a GX-12 CAN connector."},
    {"type": "figure", "file": "M10_battery_box_drawing.png", "caption": "Battery box drawing XMT-P1-E01-000 (source PDF): 393 x 358.5 x 87 mm, 3 mm aluminium sheet."},
    {"type": "table", "id": "tab_battery", "caption": "Battery and charging: real robot and model", "header": ["Item", "Real robot", "Model value", "Tag"], "widths": [22, 33, 31, 14], "rows": [
        ["Cells", "Molicel P42A 21700, 195 cells, 13S15P", "batteryCell (cylindrical 21 x 70 mm), 1 RC pair, lumped thermal mass", "[DATA]"],
        ["Capacity, energy", "63 Ah, 46.8 V, about 3.0 kWh", "63.0 Ah, 2948 Wh", "[DATA]"],
        ["Voltage range", "54.6 V full; cell 2.5-4.2 V", "39.0-54.6 V in operation; BMS cell limits 2.80 / 4.25 V", "[DATA] / [EST]"],
        ["Cell resistance", "-", "14-16 mOhm DC at 25 degC; pack R0 about 12.1 mOhm at 50 % SOC", "[EST]"],
        ["Pack mass", "-", "18.5 kg (cells 13.7 kg, box 3.3 kg, BMS and busbars 1.5 kg)", "[EST]"],
        ["BMS current", "120-150 A cont., 200-300 A peak", "150 A cont., 250 A peak discharge; 30 A charge", "[DATA]"],
        ["BMS interface", "CAN 500 kbit/s on GX-12 (pin 1 CANH, 2 CANL)", "BMS frames on CAN1 to the MCU", "[DATA]"],
        ["Charger", "54.6 V, 20 A, 100-240 VAC, PFC, IP54", "CC-CV 20 A to 54.6 V, 1200 W, over-voltage trip 55.9 V", "[DATA]"],
        ["Connectors", "SB120 main, SB50 charge, GX-12 CAN", "PDU main and charge paths", "[DATA]"],
    ]},
    {"type": "h2", "text": "Cell and pack model"},
    {"type": "p", "text": "The cell model uses open-circuit voltage and resistance tables over the state of charge at three temperatures, with one RC pair for the polarisation (time constant 30 s). The tables follow the NMC chemistry of the P42A; the resistance values are typical for this cell and marked [EST] ({fig:M23_cell_pack_model.png})."},
    {"type": "figure", "file": "M23_cell_pack_model.png", "caption": "Cell model of the Molicel P42A: (a) open-circuit voltage at 0, 25 and 45 degC and terminal voltage at 1C; (b) resistances of the 1-RC model and the resulting pack values."},
    {"type": "p", "text": "`build_shaddad_battery` builds the module with Simscape Battery (Battery Builder) from the same cell: 15 cells per parallel assembly and 13 assemblies in series. In the Grouped variant each parallel assembly is one cell model, so the pack has 13 cell models with their own state of charge, temperature and parameter spread; the Lumped variant uses one model for long runs. {fig:M30_battery_module_chart.png} shows the module with its cells laid out as in the real box, 13 rows of 15."},
    {"type": "figure", "file": "M30_battery_module_chart.png", "caption": "The 13S15P module object drawn with batteryChart: 195 cells in 13 rows of 15, each orange box one of the 13 grouped cell models."},
    {"type": "figure", "file": "M31_pack_grouped.png", "caption": "Pack_Grouped model: the generated module block with its balancing input, ambient thermal port and probes for the cell voltages, states of charge and temperatures."},
    {"type": "h2", "text": "BMS model"},
    {"type": "p", "text": "`Shaddad_BMS` models the firmware of the 13S BMS: it measures the 13 group voltages, the temperatures and the pack current, estimates the state of charge with an extended Kalman filter, computes the discharge and charge current limits (DCL / CCL), balances the groups passively and controls the contactors and the charge. It sends its state to the MCU over CAN."},
    {"type": "table", "id": "tab_bms", "caption": "BMS thresholds and limits in the model", "header": ["Quantity", "Warning", "Limit / fault", "Tag"], "widths": [34, 22, 30, 14], "rows": [
        ["Cell over-voltage", "4.22 V", "4.25 V", "[EST]"],
        ["Cell under-voltage", "3.00 V", "2.80 V", "[EST]"],
        ["Charge voltage (CV)", "-", "4.20 V per cell, 54.6 V pack", "[DATA]"],
        ["Discharge current", "-", "150 A continuous, 250 A peak", "[DATA]"],
        ["Charge current", "-", "30 A (charger delivers 20 A)", "[EST]"],
        ["Cell temperature", "-", "over 60 degC, under -20 degC", "[EST]"],
    ]},
    {"type": "figure", "file": "F14_bms_top.png", "caption": "Shaddad_BMS model: measurement, state estimation, limits, balancing, contactor and charge control."},
    {"type": "h2", "text": "Power distribution and energy budget"},
    {"type": "p", "text": "The power plant connects the pack through the PDU (fuses, current shunt, main contactors K1 / K2 with precharge, implement contactor K3 and charge contactor K4) to the two dual-channel ESCs (one 125 A traction fuse per side), the implement and the low-voltage DC/DC converters (24, 12, 5 and 3.3 V). The photos show an LM2596 buck module ({sec:mb06_electronics}); its 40 V input limit means it is fed from a lower rail, not from the 48 V bus. The safety chain (E-stops, bumpers, wireless E-stop, safety relay) removes the torque of both ESCs through STO and opens the contactors."},
    {"type": "p", "text": "With the sprayer working at 1.0 m/s the model draws about 520 W (traction 320 W, pump 75 W, nozzles 64 W, electronics 60 W) and about 380 W in transit. A 3-hour shift with 70 % spraying then ends at 32 % state of charge for a pack at its end of life (80 % capacity), above the BMS reserve."},
]})

# ======================================================================================= 6 electronics
chapters.append({"id": "mb06_electronics", "title": "Electronics and Firmware", "blocks": [
    {"type": "p", "text": "The electronics sit in a black steel box on the robot: the XMachines **XT_PS V1.0** control board carrying an **STM32 Nucleo-32**, a perfboard with optocouplers and resistors, an 8-channel logic-level shifter, and the cable conduits to the drives, the battery and the sprayer. The boards are glued and wired in place, so this chapter shows them as 3-D renders built in MATLAB from the photos (`docs/report/build/figtools/real_electronics_3d.m`). Part packages have their standard sizes; positions, silkscreen labels and colours follow the photos, and part functions are inferred from the silkscreen and the wiring."},
    {"type": "figure", "file": "M55_electronics_box_3d.png", "caption": "Electronics box of the robot (3-D render after the photo): XT_PS control board with the Nucleo-32, perfboard with optocouplers, level shifter, cable conduits."},
    {"type": "table", "id": "tab_elec", "caption": "Electronics: real boards and their place in the model", "header": ["Real board", "Function on the robot", "In the model"], "widths": [27, 37, 36], "rows": [
        ["XT_PS V1.0 control board (XMachines)", "48 V input with an on-board buck regulator, CAN connectors and transceiver, optocoupled inputs; carrier of the Nucleo-32", "Electronics plant: LV supply, CAN networks, digital inputs (`Shaddad_Plant`)"],
        ["STM32 Nucleo-32", "main controller: RC input, modes, safety, drive commands, CAN", "`Shaddad_MCU` firmware model"],
        ["STM32 USB-C core board", "second controller (function to be confirmed)", "not modelled separately"],
        ["FT232RL USB-UART adapter", "USB serial link to a PC (programming, logging)", "not in the simulated signal path"],
        ["ADS1115 / ADS1015 ADC (I2C)", "16-bit analog inputs, current sensor on the CUR header", "ADC channels with quantisation and scaling in the electronics plant and `fw_adc_scaling`"],
        ["LM2596 buck module", "5 V supply from a lower rail (max. 40 V input)", "LV DC/DC converters (`LV_Distribution`)"],
        ["SPR V1.3 sprayer board (XMachines)", "pump and nozzle drivers with optocoupled inputs", "`Impl_Sprayer` pump driver and nozzle loads, `impl_ecu`"],
        ["Perfboard: optocouplers, level shifter", "isolated digital I/O, 3.3 / 5 V level shifting", "digital-input channels with debounce in the electronics plant"],
    ]},
    {"type": "h2", "text": "Control board and main controller"},
    {"type": "p", "text": "The XT_PS board takes the 48 V battery voltage at its `+48V_IN` connector and makes its own low voltage with an on-board buck regulator. Two CAN connectors on its left edge lead to a transceiver (U3); two optocouplers (U5, U7) isolate digital inputs. The Nucleo-32 plugs into female headers whose pin labels (A0-A7, D13, 3V3, 5V, VIN) are printed on the board ({fig:M50_xt_ps_board_3d.png})."},
    {"type": "figure", "file": "M50_xt_ps_board_3d.png", "caption": "XT_PS V1.0 control board with the STM32 Nucleo-32 (3-D render after electronics_box_xt_ps_nucleo.jpeg)."},
    {"type": "p", "text": "The firmware of the main controller is modelled in `Shaddad_MCU` as rate-monotonic tasks, as it would run on the STM32: `Task_1k` (1 ms) decodes the SBUS frames and runs the watchdog, `Task_5` (5 ms) reads the IMU and estimates tilt and yaw rate, `Task_10` (10 ms) maps the RC channels and runs the safety supervisor, the mode manager and the vehicle control, and `Task_50` (50 ms) scales the ADC inputs and runs the implement functions. Every signal between tasks passes a rate transition, as in the generated code. For the real robot the speed limits were changed to 1.4 m/s in MANUAL, 1.2 m/s in AUTO and 1.70 m/s overspeed. The mode manager in `Task_10` decides when the robot may drive ({fig:F20_mcu_modes.png}): it arms only from STANDBY with neutral sticks and all safety conditions met, and every stop needs a new arm edge."},
    {"type": "figure", "file": "F20_mcu_modes.png", "caption": "Mode manager of the firmware model (fw_mode_manager in Task_10): Phase 1 transitions."},
    {"type": "note", "kind": "info", "text": "The firmware model was built for an STM32H743 target. The chip of the real Nucleo-32 and of the USB-C board is not yet known (open item 5 in {sec:mb10_open}); the simulation does not depend on it, and code generation will be retargeted once the chip is confirmed."},
    {"type": "h2", "text": "Second controller and USB-UART adapter"},
    {"type": "p", "text": "Next to the control board sit a blue STM32 core board with a USB-C connector, BOOT0, NRST and KEY buttons and an LQFP48 microcontroller, and a red FT232RL USB-UART adapter with its 3.3 / 5 V jumper ({fig:M51_stm32_ftdi_3d.png}). The CUR header on the board below takes the current sensor."},
    {"type": "figure", "file": "M51_stm32_ftdi_3d.png", "caption": "STM32 USB-C core board and FT232RL USB-UART adapter on the control board (3-D render after electronics_ftdi_stm32_board.jpeg and electronics_ftdi_closeup.jpg)."},
    {"type": "h2", "text": "Analog inputs and supply"},
    {"type": "p", "text": "An ADS1115 / ADS1015 module adds four 16-bit analog inputs over I2C ({fig:M52_ads1115_3d.png}). The LM2596 buck module ({fig:M53_lm2596_3d.png}) makes a 5 V rail; its bottom side is rendered as photographed, the top side with the usual component layout of this module."},
    {"type": "figure", "file": "M52_ads1115_3d.png", "caption": "ADS1115 / ADS1015 ADC module on the control board (3-D render after electronics_ads1115_adc.jpg)."},
    {"type": "figure", "file": "M53_lm2596_3d.png", "caption": "LM2596 DC-DC buck module: top side with the usual layout of this module (left) and bottom side as in electronics_lm2596_buck.jpg (right)."},
    {"type": "h2", "text": "Sprayer board"},
    {"type": "p", "text": "The XMachines SPR V1.3 board drives the sprayer. The photo shows part of it: 1 % metal-film resistors, a 4-pin SMD package, an axial diode and two capacitors next to the board name ({fig:M54_spr_board_3d.png})."},
    {"type": "figure", "file": "M54_spr_board_3d.png", "caption": "Detail of the XMachines SPR V1.3 sprayer board (3-D render after electronics_xmachines_spr_v1_3_board.jpg)."},
]})

# ======================================================================================= 7 sprayer
chapters.append({"id": "mb07_sprayer", "title": "Sprayer Module", "blocks": [
    {"type": "p", "text": "The sprayer module is the implement of the real robot. The spec sheet gives its dry mass (50 kg); the photo ({fig:M03_sprayer_pump.jpg}) shows a brass piston pump with a pressure regulator under the chassis and a 12 x 8 mm high-pressure hose. Tank size, pump flow and nozzles are estimated."},
    {"type": "figure", "file": "M03_sprayer_pump.jpg", "caption": "Sprayer pump under the chassis (source photo): brass piston pump with pressure regulator and high-pressure hose."},
    {"type": "table", "id": "tab_sprayer", "caption": "Sprayer module: real robot and model", "header": ["Item", "Real robot", "Model value", "Tag"], "widths": [22, 32, 32, 14], "rows": [
        ["Module mass", "50 kg dry", "50 kg, on the deck behind the centre", "[DATA]"],
        ["Tank", "-", "40 L (40 kg of liquid when full); liquid mass and CG follow the fill level", "[EST]"],
        ["Pump", "brass piston pump with pressure regulator", "positive-displacement pump, about 9 L/min, bypass to 3 bar", "[DATA] / [EST]"],
        ["Pump motor", "not visible in the photos", "24 V PM DC motor, PWM driver with soft start", "[EST]"],
        ["Boom and nozzles", "-", "1.2 m boom, 8 nozzles, 8 W valve each", "[EST]"],
        ["Hose", "12 mm OD x 8 mm ID, high pressure", "pressure losses lumped in the pump model", "[DATA]"],
    ]},
    {"type": "p", "text": "In the model the sprayer is the `Impl_Sprayer` variant (`VC_IMPL = 3`). It draws its power through the implement contactor K3 and the 24 V rail, and the liquid in the tank is a variable point mass, so the robot gets lighter as it sprays. The spec render shows the module towed on a caster wheel; the model mounts it on the deck, which keeps the same mass with a simpler structure model."},
    {"type": "figure", "file": "M34_robot3d_sprayer.png", "caption": "3-D view of the model with the sprayer module: 40 L tank on the deck and the 1.2 m boom with 8 nozzles at the rear."},
]})

# ======================================================================================= 8 simulation of the real robot
chapters.append({"id": "mb08_results", "title": "Simulation of the Real Robot", "blocks": [
    {"type": "p", "text": "Test drive D1 (`phase1_test_drive`, run `D1_20261027_212927` of the final Full suite) drives the four-motor model by remote control on farm soil: straight at 0.8 m/s, a left arc, straight at 1.2 m/s, a right arc at 0.5 m/s, a pivot turn, a reverse at 0.4 m/s, then stop. It covers 34.7 m in 64 s and takes 7.0 Wh from the pack (201 Wh/km). The two figures below show the main signals of the run; `plot_run(res)` plots all of them on six pages (user manual, section 6)."},
    {"type": "figure", "file": "M42_d1_motion.png", "caption": "Test drive D1 on the four-motor model, motion: (a) forward speed and (b) yaw rate against the command, (c) path coloured by speed, (d) torque of each motor against the 3.18 N m continuous rating."},
    {"type": "p", "text": "The robot follows each speed step along the ramp of the MCU reference without overshoot. The yaw rate follows the turn commands: it overshoots by about 13 % at the start of the arcs and by about 20 % at the start of the pivot turn, and settles on the command within about 3 s ({fig:M42_d1_motion.png}). The gyro reading of the MCU lies on the true yaw rate. Each motor needs about 0.3 N m when driving straight and at most about 1.1 N m on the outer track of a turn, a third of the 3.18 N m continuous rating of the 1.0 kW unit."},
    {"type": "figure", "file": "M43_d1_energy.png", "caption": "Test drive D1 on the four-motor model, battery and energy: (a) pack voltage and current, (b) pack and traction power, (c) state of charge, (d) the 13 cell-group voltages."},
    {"type": "p", "text": "The pack stays at 53.2-53.4 V and delivers at most 14.9 A, about 800 W, while the robot accelerates to 1.2 m/s; driving straight at 1.2 m/s takes about 570 W ({fig:M43_d1_energy.png}). The 13 group voltages stay within 21 mV of each other, and the BMS estimate of the state of charge stays within 0.11 % of the true value. In the power-system log the main contactor K1 closes 0.4 s after start, after the precharge through K2, all safety flags stay true, and the motor windings warm from 45.0 to 46.1 degC."},
]})

# ======================================================================================= 9 verification
ver_blocks = [
    {"type": "p", "text": "The model is verified at three levels: unit benches per domain (`tests/`), system scenarios (`scripts/scenarios/scn_S*`) and the Phase 1 suite `run_phase1_tests`, which runs every case in a fresh MATLAB process and writes an HTML report. For the real robot the expected values of the tests were rewritten to follow from `P` (masses, gear ratio, current limits, number of cell groups), so the same tests now check the real-robot model."},
    {"type": "h2", "text": "Phase 1 suite, Full (four-motor model)"},
    {"type": "p", "text": "The Full suite has 23 cases: the six Quick cases (power-up, RC driving, corrupted SBUS frames, front E-stop and two unit classes), the slope and link-loss scenarios, five test drives and nine unit classes. It ran twice on the four-motor model on 27 October 2026. The first run (report `phase1_full_20261027_180406`) used the first form of decision D64 (Ki 0.060 and a wider breakaway velocity of the brake). It failed S8a and test drive D2, where the engaged brakes diverged in ESTOP, and the standstill check of S6a / S6b; these findings gave the final D64 (section 4.3). It also showed that the battery bench still referenced the former 16-group pack ({tab:tab_unitfix}). The second run on the final model is the result below."},
]
if VER and VER.get('full'):
    fv = VER['full']
    ver_blocks.append({"type": "p", "text": fv['summary']})
    ver_blocks.append({"type": "table", "id": "tab_full", "caption": fv['caption'], "header": ["Case", "What it tests", "Result", "Checks"], "widths": [16, 52, 11, 21], "rows": fv['rows']})
    for n in fv.get('notes', []):
        ver_blocks.append({"type": "p", "text": n})
else:
    ver_blocks.append({"type": "note", "kind": "info", "text": "The Full suite results are missing from this build (run make_mbd_chapters.py with verification.json)."})
ver_blocks += [
    {"type": "h2", "text": "Deviations and their causes"},
    {"type": "p", "text": "Five checks fail on the final model ({tab:tab_dev}). Three are vehicle-level deviations accepted as known issues after the analysis of version 1.0 (user decision of 27 October 2026). The other two are unit tests whose causes lie outside the drive change; they are open items 11 and 12. U_mcu_platform also reports four known tests that still expect the stub units the MCU control units replaced (docs/STATUS.md 4.1)."},
    {"type": "table", "id": "tab_dev", "caption": "Failing checks of the Full suite on the four-motor model and their causes", "header": ["Check", "Model", "Limit", "Status", "Cause"], "widths": [18, 12, 15, 10, 45], "rows": [
        ["S2.v_mean", "0.725 m/s", ">= 0.75 m/s", "KNOWN", "about 9.5 % track slip at the 0.44 traction demand of the 316 kg robot on 20 deg (estimated track size)"],
        ["S6a.decel", "1.100 m/s²", "<= 1.05 m/s²", "KNOWN", "0.2 s catch-up of the speed loop at the start of the 1.0 m/s² controlled stop"],
        ["S6b.decel", "1.099 m/s²", "<= 1.05 m/s²", "KNOWN", "same as S6a (silent SBUS line instead of radio loss)"],
        ["U_mcu_control TEL27", "STANDBY", "CTRL_STOP", "FAIL", "the test reads the mode at the last sample, where the class-B stop has just ended; not related to the drive (open item 12)"],
        ["U_structure T10", "1.417", "1.266 ± 10 %", "FAIL", "gauge factor of the high-fidelity turn on the short real track against the analytic Wong model (open item 11)"],
    ]},
    {"type": "h3", "text": "S2: tilt and ramp speed"},
    {"type": "p", "text": "With the IMU level calibration and TILT_WARN at 25 deg, the MCU estimate stays within about 1 deg of the true body pitch on the 20 deg ramp and TILT_WARN is never set ({fig:M44_s2_tilt_speed.png}a): the robot keeps its full speed limit and motor torque on its design slope. The remaining deviation is the ramp speed. With the belts at 0.80 m/s the tracks slip about 9.5 % at the tractive effort the 316 kg robot needs on 20 deg (0.44 of its weight), so the ground speed is 0.725 m/s against the 0.75 m/s criterion ({fig:M44_s2_tilt_speed.png}b, c). The slip is larger than in version 1.0 (about 7 %) because the track's relaxation length now matches the 0.75 m contact length (chapter 3). It depends on the estimated track size and the soil, and should be reviewed after the slope test (calibration test 2). During the 30 s hold on the ramp the engaged brakes keep the robot within 3 mm."},
    {"type": "figure", "file": "M44_s2_tilt_speed.png", "caption": "S2 on the four-motor model: (a) true pitch, MCU estimate and the 25 deg TILT_WARN threshold, (b) MCU target, belt and ground speed against the 0.75 m/s criterion, (c) track slip and the MCU motor-torque limit."},
    {"type": "h3", "text": "S6a / S6b: deceleration of the controlled stop"},
    {"type": "p", "text": "After the radio loss the MCU ramps the speed reference down at 1.0 m/s². On farm soil the tracks' motion resistance alone decelerates the robot at about 1.05 m/s², so the stop needs almost no braking ({fig:M45_s6_stop.png}c). The MCU feed-forward subtracts the inertia torque from the driving torque the speed loop is holding, and the ESC speed loop follows the reference slightly late; when it catches up, the ground deceleration peaks at 1.10 m/s² for about 0.2 s before it settles at the shaped 1.0 m/s² ({fig:M45_s6_stop.png}b). The robot is at standstill 1.12 s after the loss is detected (limit 1.5 s), never enters ESTOP, and its peak deceleration stays far below the 2 m/s² of the E-stop."},
    {"type": "figure", "file": "M45_s6_stop.png", "caption": "S6a on the four-motor model: (a) MCU speed reference, belt and ground speed, (b) ground deceleration (100 ms moving average) against the 1.0 m/s² shaper and the 1.05 m/s² criterion, (c) torque per motor and MCU feed-forward, (d) motor speed reference and actual; t = 0 at CTRL_STOP."},
    {"type": "h3", "text": "Handling of the deviations"},
    {"type": "table", "id": "tab_prop", "caption": "Handling of the deviations", "header": ["Check", "Handling", "Note"], "widths": [20, 24, 56], "rows": [
        ["S2.no_tilt", "**Fixed**", "IMU level calibration on level ground and TILT_WARN 25 deg (proposal of version 1.0, applied): passes."],
        ["S2.v_mean", "**Accepted** (known issue)", "Track slip at this tractive effort; review after the track is measured and the slope test is done (open item 1, calibration test 2), or set the criterion on the belt speed."],
        ["S6a.decel, S6b.decel", "**Accepted** (known issues)", "A 0.2 s transient, 5 % over the criterion. A softer stop jerk would remove it, but the stop jerk is shared with the E-stop profile."],
        ["U_mcu_control TEL27", "open item 12", "Test change only: check the first mode after the link loss."],
        ["U_structure T10", "open item 11", "Re-identify the gauge factor and compare with the turn test."],
    ]},
    {"type": "p", "text": "S8b (remote E-stop), whose mean deceleration was a known issue on the former robot, passes on the real-robot model, and so do S8a and test drive D2 with the elasto-plastic brake."},
    {"type": "h2", "text": "Unit tests on the four-motor model"},
    {"type": "p", "text": "Before the Full suite the affected unit classes were run on the four-motor model (153 tests). Most failures were not in the new drive itself: they were stale values from the change to the real robot that the new drive made visible. {tab:tab_unitfix} lists each finding, its cause and what was done."},
    {"type": "table", "id": "tab_unitfix", "caption": "Unit-test findings on the four-motor model", "header": ["Test", "Finding", "Cause", "Action"], "widths": [15, 25, 33, 27], "rows": [
        ["`t_drive` T1, T2, T22", "ramp overshoot 9.1 %, step overshoot 23 %, stop-profile error 0.23 m/s", "the drive benches had kept the load of the former robot (300 kg per side on a 0.128 m sprocket) in their model workspace since they were built", "the bench load follows the P of every run (`tb_drive_load`): 0.3 %, 7.9 %, 0.038 m/s"],
        ["`t_drive` T21", "peak sensitivity 1.65 (bound 1.6)", "the speed-PI gains were only scaled with the inertia at the real-robot change (one motor per side: Ms 2.25)", "speed PI re-tuned on the two-mass model (D64): Ms 1.37"],
        ["`t_drive` T9, T10", "STO peak sprocket torque 859 N m (limit 480 N m); quick-stop deceleration fit 0.61 m/s²", "stick-slip of the two engaged brakes at the 1 ms step, shown once the bench had the right load", "elasto-plastic brake (D64): 164 N m, 2.03 m/s²"],
        ["`t_pack_bms` (Full suite)", "7 tests stop: subsystem reference `Pack_Grouped16` missing", "the battery bench still referenced the former 16-group pack model; the 13-group pack of the real robot is `Pack_Grouped`", "bench rebuilt (`build_tb_pack_bms`): 15 pass, 1 spec deviation"],
        ["`t_elec_plant` T-EL-16", "48 V-side LV draw 123 W (105 ± 10 W)", "four brakes instead of two: the two extra brake holds draw 23 W", "criterion 105 W + the extra holds = 128 ± 10 W"],
        ["`t_power_plant` T-PWR-06b", "the traction fuse does not blow within 150-300 s", "the overload is now 1.5 x the 125 A fuse rating (187.5 A); the BMS sees more than its 150 A limit and opens the main contactor after 1.3 s, before the fuse", "open item 14 (fuse and BMS coordination)"],
        ["`t_structure` T4, T7", "track slip 25 % below the analytic value", "the shear relaxation length had stayed at 0.50 m (half of the former 1.00 m track)", "relaxation length = L/2 = 0.375 m: pass"],
        ["`t_structure` T10", "gauge factor 1.417 against 1.266 from Wong's turn model (12 %, tolerance 10 %)", "high-fidelity track model against the analytic turn on the short real track", "open item 11"],
        ["`t_mcu_control` TEL27", "STANDBY instead of CTRL_STOP at the end of the run", "the class-B stop ends (standstill + 0.5 s) at the last sample of the 6 s run, which the test reads; the same with the former drive values", "open item 12"],
        ["code generation", "Simulink Coder stops on `Ts.log`", "`log` is a reserved C name in the sample-time struct `Ts`; code generation is outside the Phase 1 suite", "open item 13"],
    ]},
    {"type": "h2", "text": "Tests updated for the real robot"},
    {"type": "table", "id": "tab_tests", "caption": "Test changes for the real robot", "header": ["Test", "Change"], "widths": [26, 74], "rows": [
        ["`t_structure`", "configuration masses 220 / 270 / 310 / 285 / 305 kg and the CG from the mass table; analytic sinkage, slip, acceleration and climbing references from `P`"],
        ["`t_drive`", "torque, speed and current expectations from the motor of `P` and the 20:1 gear; bench load from `P` for every run; T6 (motor efficiency points) pending the motor data sheet"],
        ["`t_mcu_control`, `t_mcu_platform`", "speed limits, overspeed stimulus and telemetry checks from the new limits; new IMU test `testGyroBiasPivot`"],
        ["`t_pack_bms`, `t_power_plant`, `t_elec_plant`", "13 cell groups, 46.8 V nominal, BMS and charger limits from `P`; LV draw with the brake count; overload case from the fuse rating"],
        ["`t_gnc_tracking`, `t_gnc_mission`, `t_sensors`", "footprint, corridor and sensor positions from the new geometry"],
        ["Scenarios S2-S5", "gear and BMS limits, shift duration and end-of-shift SOC, obstacle corridor from `P`"],
    ]},
]
chapters.append({"id": "mb09_verification", "title": "Verification", "blocks": ver_blocks})

# ======================================================================================= 10 open items
chapters.append({"id": "mb10_open", "title": "Open Items and Calibration Plan", "blocks": [
    {"type": "p", "text": "The [EST] values are good enough for system studies, but each one is a source of error until it is measured. {tab:tab_open} lists what is still needed, what it affects in the model and how to obtain it."},
    {"type": "table", "id": "tab_open", "caption": "Open items", "header": ["#", "Open item", "Affects in the model", "How to close"], "widths": [5, 31, 32, 32], "rows": [
        ["1", "Track: gauge, width, contact length, sprocket teeth and pitch", "turning (gauge factor), sinkage, speed scaling", "measure on the robot; re-run `structure_identify_chi`"],
        ["2", "Ratio and type of the four gearboxes", "top speed, torque at the track, inertia", "gearbox plate or turns count (motor turns per sprocket turn)"],
        ["3", "Dual ESC model and motor data sheets (installed Movematic; the 1.0 kW motor chosen for the upgrade)", "drive response, current limits, efficiency (test T6)", "data sheets from Movematic, the ESC vendor and the motor supplier"],
        ["4", "Sprayer: tank volume, pump flow and pressure, pump motor, nozzles, boom width", "implement power, liquid mass, shift energy", "pump label, tank marking, flow test"],
        ["5", "Chips of the Nucleo-32 and of the USB-C board", "code-generation target of `Shaddad_MCU`", "read the chip markings"],
        ["6", "LV supply: converter from 48 V to 12 / 24 V", "LV losses, rail limits", "trace the wiring in the electronics box"],
        ["7", "Cell resistance and BMS thresholds as configured", "voltage sag, BMS limits and warnings", "BMS configuration dump; pulse test on the pack"],
        ["8", "Mass split and centre of gravity", "handling on slopes, track loads", "weigh each track side; tilt test"],
        ["9", "Tilt alarm of the companion (NavLite, Phase 2): 22 deg pitch, the same thin margin over the 20 deg design slope", "Phase 2 navigation on slopes", "raise it with the MCU's TILT_WARN, after the level calibration"],
        ["10", "Holding brake data: static and dynamic torque and the bristle stiffness of the elasto-plastic brake model (D64) are estimates", "holding on slopes, stop distance and gearbox peak torque (T8, T9)", "brake data sheet; slope hold with the motors off (calibration test 3)"],
        ["11", "Turning on the short real track: gauge factor 1.417 (high-fidelity model) against 1.266 (Wong), unit test T10", "turn torques, gauge factor used by the MCU and the guidance", "re-run `structure_identify_chi` after the relaxation-length fix; compare with the turn test (calibration test 2)"],
        ["12", "MCU unit test TEL27 reads the mode at the last sample of the run", "verification only", "check the first mode after the link loss instead of the last sample"],
        ["13", "Code generation of `Shaddad_MCU`: the field `Ts.log` is a reserved C name", "embedded code (Simulink Coder is now installed)", "rename the field in `shaddad_params` and its users"],
        ["14", "Traction fuses and BMS: with 125 A per side (two 55 A ESC channels), any overload that blows the fuse within minutes exceeds the 150 A BMS limit, so the BMS switches the whole pack off first; the fuse only clears short circuits (T-PWR-06b)", "fault isolation of one track side; T-PWR-06b", "decide: accept the BMS as the overload protection and rewrite T-PWR-06b, or fuse each ESC channel (4 x about 63 A: 1.5 x = 95 A, below the BMS limit)"],
    ]},
    {"type": "h2", "text": "Calibration plan"},
    {"type": "p", "text": "Six short tests on the real robot close most of the gap. Each result goes into `params/`, the affected models are regenerated and the tests are run again, as in {fig:M28_mbd_workflow.png}:"},
    {"type": "numbered", "items": [
        "**Speed steps and coast-down** on farm soil, logging the ESC currents and speeds: motor constants, gear losses, rolling resistance.",
        "**Turns at fixed track-speed ratios**: gauge factor per soil.",
        "**Slope hold** with the motors off: brake torque and the CG.",
        "**Pack current pulses** at three states of charge: cell resistance and polarisation.",
        "**A sprayer shift** with the energy logged by the BMS: energy per metre and the end-of-shift state of charge.",
        "**IMU level calibration** with the robot on level ground at commissioning: the mean MCU roll and pitch go into `P.electronics.imu_cal` (`scripts/imu_level_calibration.m`).",
    ]},
]})

# ======================================================================================= 11 appendix
chapters.append({"id": "mb11_appendix", "title": "Appendix: Data and Sources", "blocks": [
    {"type": "h2", "text": "Mass table"},
    {"type": "table", "id": "tab_mass", "caption": "Mass table of the robot (226 kg with the 1.0 kW drive units; 220 kg as built)", "header": ["Component", "Mass [kg]", "Note", "Tag"], "widths": [36, 12, 38, 14], "rows": [
        ["Chassis frame", "81.7", "remainder to the as-built 220 kg", "[EST]"],
        ["Body panels, deck rail", "20", "", "[EST]"],
        ["Track assembly, left / right", "22 / 22", "rubber track, sprocket, idler, 3 road wheels, frame", "[EST]"],
        ["Drive units (4, one at each sprocket)", "4 x 9.7", "1.0 kW motor 4.5, gearbox 3.5, brake 0.5, half dual ESC 0.8, cables 0.4 (as built 4 x 8.2)", "[EST]"],
        ["Battery pack 13S15P", "18.5", "cells 13.7, 3 mm aluminium box 3.3, BMS and busbars 1.5", "[DATA] count / [EST]"],
        ["Electronics box", "6", "boards, enclosure, harness", "[EST]"],
        ["Sensor mast (Phase 2)", "3", "GNSS / LiDAR, model addition", "[EST]"],
        ["Cooling, misc.", "4", "", "[EST]"],
        ["Front coupler / rear hitch", "4 / 6", "", "[EST]"],
    ]},
    {"type": "h2", "text": "Battery specification sheet"},
    {"type": "figure", "file": "M15_battery_spec_p1.png", "caption": "Battery specification sheet, page 1 (source PDF)."},
    {"type": "figure", "file": "M15_battery_spec_p2.png", "caption": "Battery specification sheet, page 2 (source PDF)."},
    {"type": "h2", "text": "Figures and how they are made"},
    {"type": "table", "id": "tab_figsrc", "caption": "Figure sources (all in `docs/report/figs/`)", "header": ["Figures", "Made by"], "widths": [34, 66], "rows": [
        ["Photos M01-M03, drawing M10, spec pages M15", "`figtools/prep_real_photos.py` from `docs/hardware/source/`"],
        ["Collage M16", "`figtools/make_real_collage.py` (photos and label-free renders)"],
        ["Engineering figures M20-M29", "`figtools/real_mbd_figs.m` from `P`, `scripts/motor_sizing.m`, the drive benches (M24) and the S8a logs (M29)"],
        ["Model views M30-M34", "`figtools/real_mbd_screens.m` (Simscape Battery chart, model print, `Robot3DView`)"],
        ["Test-drive and scenario figures M42-M45", "`figtools/real_mbd_figs.m` from the D1, S2 and S6a run logs of the Full suite"],
        ["Electronics renders M50-M55", "`figtools/real_electronics_3d.m` (MATLAB 3-D, built from the electronics photos)"],
        ["Model screenshots F01, F07, F12, F14 and diagram F20", "earlier Phase 1 report (structure unchanged by the re-parameterisation)"],
    ]},
    {"type": "p", "text": "Data sheet with every value and its source: `docs/hardware/real_robot_components.md`. Source files: `docs/hardware/source/`."},
    {"type": "h2", "text": "Sign-off"},
    {"type": "table", "id": "tab_signoff", "caption": "Sign-off", "signoff": True, "header": ["Role", "Name", "Signature", "Date"], "widths": [22, 30, 28, 20], "rows": [
        ["Prepared by", "[TBD - author]", "", ""],
        ["Reviewed by", "[TBD - reviewer]", "", ""],
        ["Approved by", "[TBD - approver]", "", ""],
    ]},
]})

# ======================================================================================= write
for k, ch in enumerate(chapters):
    with open(os.path.join(D, ch['id'] + '.json'), 'w', encoding='utf-8') as f:
        json.dump(ch, f, ensure_ascii=False, indent=1)
print('chapters written:', ', '.join(c['id'] for c in chapters))
