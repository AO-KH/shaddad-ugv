<!--
doc_no: NASL-SHD-REQ-008
doc_type: REQUIREMENTS SPECIFICATION
title: Shaddad UGV: system requirements specification
subtitle: The functional and technical requirements
summary: What the Shaddad UGV shall do as a system, and the values it shall meet, in each mode and situation of this phase: 210 requirements, 121 functional under 15 system functions and 89 technical of five kinds, 114 of them safety requirements, traced to 17 stakeholder needs and 12 safety goals, each with its verification and its state on the robot as built.
version: 2.0
date: 2026-11-18
status: Approved
history: 1.0 | 2026-11-17 | Part 1, the functional requirements (card G1-01); approved by the project owner on 2026-11-17 ("Approve part 1 (Recommended)")
history: 2.0 | 2026-11-18 | Part 2, the technical requirements and the values (card G1-01); approved by the project owner on 2026-11-18 ("Approve part 2 (Recommended)")
prepared_by: the lead systems engineer, with the Shaddad engineering team
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies
classification: Internal
footer: System requirements specification
file: NASL_REQ-008_Shaddad_System_Requirements_2026-11-18_v2.0
outdir: work/reports/requirements
request: The project owner, 2026-11-16: "for now we can start G1-01 Re-baseline the system requirements on the as-built robot but we can work on the functional requirements of the system then we will work on the technical requirements"; part 2 started on 2026-11-17 ("Start part 2")
scope: Card G1-01, parts 1 and 2: the functional and technical requirements of the system on the robot as built
related: the charter NASL-SHD-CHR-001; the hazard log NASL-SHD-HAZ-001; the safety plan NASL-SHD-SAF-001; the decision log (docs/architecture.md); card G1-01
chapter_breaks: yes
cover_image: none
-->
<!-- Generated from docs/requirements/system.yaml by tools/reports/build_requirements_spec.py; do not edit -->
# Shaddad UGV: system requirements specification

## Summary

**This specification states what the Shaddad UGV shall do as a system, and the values it shall meet.** It holds the functional requirements of part 1 of card G1-01, approved at version 1.0, and adds the technical requirements and the values of part 2. It is generated from the requirements register `docs/requirements/system.yaml` and issued for the project owner's approval.

Table 1. The specification in counts.
| Item | Count | Section |
| --- | --- | --- |
| Stakeholder needs | 17, every one traced to a requirement | 2, 8.1 |
| System functions | 15 | 3 |
| Safety goals, the top-level safety requirements | 12 | 4 |
| Functional requirements | 121: REQ-SYS-001 to -117 and REQ-SYS-207 to -210 | 5 |
| Technical requirements | 89, of five kinds: REQ-SYS-118 to -206 | 6 |
| Safety requirements | 114: 83 functional and 31 technical | 5, 6, 8.2 |
| Requirements with a value still to set (TBD) | 8, none a safety requirement | 10.2 |
| Standards, each dated | 33 | 7 |

- **By kind** (section 6): 33 performance, 11 physical, 11 environment, 10 interface and 24 constraint.
- **On the robot as built** (section 9): 36 implemented, 100 partly met, 23 not implemented, 27 waiting on hardware data and 24 unknown.
- **Verification:** 164 by test, 33 by inspection, 12 by analysis and 1 by demonstration; 176 in simulation, 29 in the field and 5 on the bench.
- **Open items** (section 10): the RIDs for the review G1-05, by the card that closes them; the 8 values still to set; and the review decisions still open on card G1-03.

## 1. Scope

### 1.1 The system

The Shaddad UGV as built (2026-10-27 onward), as the twin represents it: the XMachines tracked platform with its pack, motors, ESCs, spring brakes, controller and electronics; the MCU and BMS firmware and the companion software; the sensors; the implements of this phase; the operator's RC transmitter and console; and its stand-alone charger, with the twin's dock (D97: the robot has no dock in this phase). Outside the system: the people and the site, the RTK correction source, and the twin's own tools.

### 1.2 This phase

This phase: mowing in greenhouses and palm groves; transit, docking and charging; RC driving, autonomous missions, the E-stop chain and every safety function (charter NASL-SHD-CHR-001 v1.3, Table 2). The functions of tillage and spraying missions come in a later phase (cards G2-TIL, G2-SPR). The functions that serve every implement (knowing which one is fitted, keeping its limits) are in.

### 1.3 Part 1 and part 2

- **Part 1, version 1.0,** approved by the project owner on 2026-11-17: the functional requirements. Each says what the robot shall do, in which mode or situation, and with what result, seen at the system's boundary: the robot's motion and implement, the operator's RC and console, the dock, and the people nearby. A requirement does not say how; the part of the twin that provides it today is in its "Where" row.
- **Part 2, this version:** the technical requirements, 89 of them, each with a value that a test, an analysis, an inspection or a demonstration can judge; the values that part 1 left to part 2; and the 33 standards, each dated (D103). Part 2 also added 7 functional requirements (REQ-SYS-115 to -117 and REQ-SYS-207 to -210), and of part 1's 51 values marked `[value: part 2]` it set all but REQ-SYS-104's.
- **In the register,** all 210 requirements are agreed: the functional requirements of version 1.0 since 2026-11-17, and the whole baseline since the project owner's approval of 2026-11-18 (D110).
- **Approval:** the project owner approved the functional requirements at version 1.0 on 2026-11-17 ("Approve part 1 (Recommended)"), and the whole baseline at this version on 2026-11-18 ("Approve part 2 (Recommended)"; D110).

### 1.4 How to read a requirement

- **One "shall" in each requirement,** in one of five patterns (EARS):
  - "The robot shall ...";
  - "When &lt;event&gt;, the robot shall ...";
  - "While &lt;state&gt;, the robot shall ...";
  - "If &lt;unwanted event&gt;, then the robot shall ...";
  - "Where &lt;feature&gt;, the robot shall ...".
- **"The robot"** means the whole system of section 1.1, as in the safety goals.
- **The quality rules:** each requirement is necessary, singular, unambiguous, verifiable, feasible on the robot as built, and traceable up and down (NASL-EMS-HBK-001 section 7.2, from ISO/IEC/IEEE 29148).
- **The rows of a requirement** are those of Table 2.

Table 2. The rows of a requirement table.
| Row | What it holds |
| --- | --- |
| **ID and text** | The first row: the requirement's ID and its text, in bold. |
| **Kind** | Technical requirements only: performance, physical, environment, interface or constraint. |
| **Function** | Technical requirements only: the function it serves, or none for a system-wide requirement. |
| **Rationale** | Why the robot needs it, and for a value where the value comes from and, for safety, its margin. |
| **Source** | What it traces up to: stakeholder needs (N-), safety goals (SG-), decision rows (D), change requests (CR-) and charter sections. |
| **Parent** | The safety goal that it realises, or none. |
| **Allocation** | The parts of the system that provide it: the codes of Appendix B. |
| **Safety** | Yes for a safety requirement, one that realises a safety goal. |
| **TBD** | A value still to set, with its owner and when; or none. |
| **Verification** | The method, the level (subsystem, system, PIL/HIL or field) and the campaign (simulation, bench or field), then the reference: the scenario, the test, or the card that plans it. |
| **As built** | The state on the robot as built (Table 12). |
| **Where** | The part of the twin or its firmware that provides it today. |
| **Note** | The evidence, and the card or STATUS item that closes a gap. |

### 1.5 How to read a technical requirement

- **A system requirement with a value:** "The robot shall ..." with a number and a unit that a test, an analysis, an inspection or a demonstration can judge. Like a functional requirement, it is about the whole robot as built, seen at its boundary.
- **Its kind:**
  - performance: a speed, a time, a distance, an accuracy, an endurance, a slope;
  - physical: the mass, the size, the pack, the motors and the implements as built;
  - environment: the ambient temperature, the sun, the dust, the water, the ground and the sites;
  - interface: the operator's RC and console, the dock, the implements, and the links between the robot's parts that the operator or the dock sees;
  - constraint: a standard, a regulation, a decision of the project owner, the as-built hardware.
- **Its function:** the function it serves, or none for a system-wide requirement.
- **Where its value comes from** is in its rationale: a decision row or a change request; the charter or a need; a standard, with its edition; the parameters, with the leaf `P.x.y` and its file; or a test or scenario result, with its date.
- **A safety value** states its margin and why that margin is safe. No safety requirement carries a TBD.
- **A decided value that the twin does not hold yet** is marked "decided, pending", with the card that changes the twin; the note keeps today's value, so that every run until the change can still be read. Each such value is a RID for the review G1-05 (section 10.1).
- **A provisional value** rests on the delivered electronics (D96) and becomes final with card G1-04's data. A value set by risk assessment says that it is not yet confirmed against the standards (D103).

### 1.6 How this document is made

The register `docs/requirements/system.yaml` is the controlled source of the requirements and the standards. The script `tools/reports/build_requirements_spec.py` writes the source of this document, `docs/requirements/NASL_REQ-008_system_requirements.md`, from the register, and builds the Word file and the PDF with `tools/reports/build_document.py`. To change a requirement, change the register and rebuild; never edit this document. The RID list of section 10.1 merges the integration engineer's records of part 2, and the open decisions come from card G1-03 and the review report NASL-SHD-ANR-001. The decision titles of Appendix A come from the decision index `docs/registers/decisions.json`.

## 2. Stakeholder needs

The 17 needs itemise Table 1 of the project management plan NASL-SHD-PMP-001 and the charter NASL-SHD-CHR-001 v1.3. "Traced in" says what meets the need: the functional requirements of part 1, or the technical requirements and the plan of part 2. Of the needs, 15 are traced in part 1, and N-16 and N-17 in part 2 only. Section 8.1 traces each need to its requirements.

Table 3. The stakeholder needs.
| ID | Stakeholder | Need | Source | Traced in |
| --- | --- | --- | --- | --- |
| N-01 | farm, greenhouse and palm-grove customers | Mow the rows of a greenhouse aisle or a palm grove with no operator at the controls, the deck lowered, turning at the row ends. | charter section 6 (normal use); plan Table 1 (customers); the operations table of NASL-SHD-ANR-003 chapter 1 | part 1 |
| N-02 | farm, greenhouse and palm-grove customers | Finish a mowing mission on one charge, and go back to the dock to charge when the energy runs low. | charter goal 4 and section 6 (abnormal use, low energy); plan Table 1 (customers) | part 1 |
| N-03 | farm, greenhouse and palm-grove customers | Move between the work areas and the dock on planned paths, and dock and charge without help. | charter Table 2 (transit, docking and charging) and section 6 (normal use) | part 1 |
| N-04 | operators in the field | Drive the robot by remote control: it moves only as commanded, stops when the sticks are let go, and keeps the speed limit of RC driving. | plan Table 1 (operators); SG-11; NASL-SHD-TP-001, TP-002 | part 1 |
| N-05 | operators in the field; the supervisor | Start, hold, resume and stop a mission, and see the robot's mode, its progress, its holds and its faults with their causes. | charter section 5; plan Table 1 (operators, clear holds, stops and recovery) | part 1 |
| N-06 | operators in the field | Stop the robot at once, from the robot or from a distance, independently of its software, and start it again only by a deliberate reset and re-arm. | SG-06; charter section 6 (abnormal use, the E-stop with reset and re-arm); scenario S8 | part 1 |
| N-07 | people nearby (workers in the greenhouse or grove) | Not be struck, run over or cut: the robot stops for a person in its way and holds until the way is seen clear. | charter section 5 (people nearby); SG-01, SG-02, SG-09; D79 (the greenhouse rule) | part 1 |
| N-08 | operators in the field; the project owner | Work on slopes within the envelope without tipping over, stop and hold on the brakes anywhere in it, and never run away downhill. | charter goal 3; SG-04, SG-05; CR-001, CR-003 | part 1 |
| N-09 | operators in the field; people nearby; the project owner | The robot stays in its permitted area and near its planned path, and stops when it cannot tell where it is. | SG-03 | part 1 |
| N-10 | operators in the field; people nearby | React safely to lost links and faults: a lost RC link, lost GNSS, a lost companion or controller, a sensor fault. | charter section 6 (abnormal use); SG-07; scenarios S6, S7 | part 1 |
| N-11 | operators in the field; maintainers | The robot never moves, releases its brakes or starts its implement unexpectedly: not at power-up, after a reset, when a link returns or during maintenance. | SG-08 | part 1 |
| N-12 | maintainers | Charge the pack, fit and change implements, and inspect the brakes, tracks and sensors safely, with the energy isolated, and get faults that name their cause. | plan Table 1 (maintainers); charter section 5; SG-08; NASL-SHD-MAN-001; NASL-SHD-TP-003 | part 1 |
| N-13 | the project owner; regulators | The pack stays safe in operation, charging and storage, in heat up to 45 to 50 deg C, and live parts stay out of reach. | SG-10; plan Table 1 (regulators) | part 1 |
| N-14 | the supervisor (the project owner on the project) | Plan the missions of a site, and check their results afterwards. | charter section 5 (the supervisor plans the missions and checks the results); plan Table 1 | part 1 |
| N-15 | regulators and standards | Meet the safety standards for highly automated agricultural machines, safety-related control parts and mowers. | plan Table 1; the safety plan NASL-SHD-SAF-001 v1.4, Table 1, and the standards below (D103); SG-12 | part 1 (through the safety goals) and part 2 (the standards with their editions) |
| N-16 | the project owner | A twin of the robot as built, with the owner's decided upgrade (D95), verified against its requirements, whose generated firmware runs on the robot's controller. | charter goals 1, 2 and 5 (v1.4, CR-004); plan Table 1 (the project owner) | part 2 (and the plan; a project need, not a function of the robot) |
| N-17 | the supplier (XMachines) | The robot's platform, ESC, BMS and controller are used as delivered. | plan Table 1 (the supplier); docs/hardware/real_robot_components.md | part 2 (a constraint) |

## 3. The system functions

Each functional requirement belongs to one of the 15 functions, and each technical requirement names the function it serves; 42 technical requirements serve none, as they are system-wide. Table 4 gives the scope of each function and its requirements; section 5 follows the same order.

Table 4. The system functions.
| ID | Function | Scope | Functional | Technical |
| --- | --- | --- | --- | --- |
| F-01 | Manage the modes and the arm state | Power-up, standby, armed, RC, autonomous, hold, fault and E-stop, and the charging and maintenance states; the transitions and what each mode allows. | 8 | 1 |
| F-02 | Drive by remote control | The RC commands to motion, the RC speed limit, the stop when the sticks are let go, the RC switches. | 6 | 4 |
| F-03 | Run missions | Take a mission from the companion and run its rows, turns, transit and docking; start, hold, resume, abort and finish; report the progress. | 10 | 1 |
| F-04 | Know where the robot is | Localisation from GNSS, IMU, odometry and scan matching; the validity of the estimate; the permitted zone; the stop when the position cannot be shown. | 9 | 4 |
| F-05 | Move along the path | Follow the path, turn at the row ends, govern the speed (by mode, implement, slope and the descent cap), control the tracks. | 8 | 6 |
| F-06 | Sense the surroundings | Obstacles, and people: in the zone that the robot or its implement is about to occupy, the near zone of the body included, and round the blades. | 5 | 2 |
| F-07 | Hold and avoid | Stop and hold for a person, go round a static obstacle, resume when the zone has been seen free. | 6 | 3 |
| F-08 | Work the implement | Know which implement is fitted and its limits; raise and lower the deck; start and stop the blades with their interlocks; the implement in a hold. | 15 | 2 |
| F-09 | Manage the energy | Monitor and protect the pack; switch the high voltage on and off (precharge); feed the low-voltage rails; derate on warnings; return to the dock on low energy. | 9 | 6 |
| F-10 | Dock and charge | Approach the dock, dock, charge, undock. | 8 | 3 |
| F-11 | Stop in an emergency and reset | The emergency stops on the robot and from the operator, the bumper and the E-stop chain; staying stopped until a deliberate reset and a new arm command; no unexpected start after a fault reset, a returning link or a restart of the controller. | 7 | 2 |
| F-12 | Communicate and record | The CAN networks, the RC link and the companion link; the operator's console and telemetry; the event and fault records; the mission logs. | 4 | 4 |
| F-13 | Plan the missions | Compile a site's mission from its boundary, rows and dock: the lanes, the turns at the lane ends, the transits and the dock approach, within the permitted zone and the slope limits of the fitted implement. | 7 | 1 |
| F-14 | Respond to lost links | The stop and the park on the brakes when the operator's RC link, the companion's commands, the controller, a drive's reports or the pack's reports are lost. | 5 | 2 |
| F-15 | Stop and hold, and stay stable on slopes | The stops on faults and their park on the brakes; the stop monitor; the overspeed and runaway monitors; the tilt stop; the IMU's faults and the RC limp; no tip-over in the envelope; the roll-back at a hill start; the stuck stop and the electronics' temperature stop. | 14 | 6 |
|  | System-wide |  |  | 42 |
| **Total** |  |  | **121** | **89** |

## 4. The safety goals

The 12 safety goals of the hazard log NASL-SHD-HAZ-001 v1.2 are the system's top-level safety requirements (D80). They stand here word for word, with their safe states; the hazard log is their source and owner. Section 8.2 traces them to the requirements that realise them.

Table 5. The safety goals.
| ID | Goal | Safe state | PL | Hazards |
| --- | --- | --- | --- | --- |
| SG-01 | The robot shall detect a person in the zone that it or its implement is about to occupy, in every autonomous mode and in the dock approach, and stop before contact; it shall keep its speed within what that detection zone can stop. | standstill on the brakes, implement off, holding while the person is in the zone | d | HAZ-001, HAZ-002, HAZ-003, HAZ-004, HAZ-007, HAZ-010, HAZ-013, HAZ-045 |
| SG-02 | After a person stop the robot shall move off, or restart its implement, only when the zone it will occupy, the near zone of the body included, has been seen free. | hold on the brakes, implement off | d | HAZ-005 |
| SG-03 | The robot shall stay inside its permitted operating zone and within a bounded distance of its planned path, and stop when it cannot show where it is. | standstill on the brakes, mission paused | d | HAZ-008, HAZ-009 |
| SG-04 | No brakes-locked stop, E-stop or loss of power shall tip the robot over within its envelope (the 20 deg design slope; the full-tank sprayer to about 18.5 deg, CR-001), and the robot shall not pass its tilt limits. | standstill on the brakes below the tip speed; at the tilt limit, a stop | d | HAZ-017, HAZ-018, HAZ-019, HAZ-023, HAZ-038, HAZ-039, HAZ-040 |
| SG-05 | Every stop shall end with the robot held on its brakes on any slope of the envelope, and the robot shall not gain speed downhill beyond the speed of its mode. | standstill on the brakes, torque off | d | HAZ-024, HAZ-025, HAZ-027, HAZ-040 |
| SG-06 | An emergency stop, from the robot or from the operator's remote stop, shall stop the motion and the implement independently of the software, and keep them stopped until a deliberate reset and a new arm command. | STO, spring brakes engaged, implement power off, latched | d | HAZ-021, HAZ-022 |
| SG-07 | On the loss of the operator's link, of the companion's commands or of the controller, the robot shall stop and park. | standstill on the brakes, torque off | d | HAZ-012, HAZ-016, HAZ-043 |
| SG-08 | The robot shall not move, release its brakes or start its implement at power-up, after a reset, when a link returns or during maintenance, without a deliberate command and a warning; maintenance shall be done with the energy isolated. | standstill on the brakes, torque off, implement off | d | HAZ-011, HAZ-026, HAZ-032, HAZ-034, HAZ-037 |
| SG-09 | The blades shall not run where a person can reach them: they shall stop on an E-stop, a person near, a hold, a tip, a raised deck and outside the working modes, within a run-down time that the mower standard sets, and shall not restart without a deliberate command. | blades stopped, implement power off | d | HAZ-006, HAZ-020, HAZ-033 |
| SG-10 | The pack shall not overheat, overcharge or short in operation, charging or storage at 45 to 50 deg C, and live parts shall be protected against contact and short circuit. | contactors open, charging stopped | d | HAZ-028, HAZ-029, HAZ-030 |
| SG-11 | In RC driving the robot shall move only as the operator commands, stop when the operator lets go of the sticks, and keep the speed limit of RC driving. | standstill on the brakes | d | HAZ-014, HAZ-015 |
| SG-12 | Hot surfaces, noise, the charger's mains, manual handling and recovery, thrown objects and foreseeable misuse shall be reduced by the design, guards and the information for use, to the standards that G1-01 names. | not applicable | n/a | HAZ-031, HAZ-035, HAZ-036, HAZ-041, HAZ-042, HAZ-044 |

In the hazard log, 9 goals are not met (SG-01, SG-02, SG-03, SG-04, SG-05, SG-07, SG-08, SG-09, SG-11), SG-06 is partly met, SG-10 is unknown and SG-12 is open.

## 5. The functional requirements

One section per function, in the order of section 3. Each requirement is one table: the first row holds its ID and its text, and the rows below hold its attributes (Table 2). Within a function the requirements keep the order of the register. The technical requirements that serve a function are in section 6, by kind; each names its function.

### 5.1 F-01 Manage the modes and the arm state

**Scope:** Power-up, standby, armed, RC, autonomous, hold, fault and E-stop, and the charging and maintenance states; the transitions and what each mode allows. (8 requirements, 6 of them safety requirements.)

| REQ-SYS-001 | When the robot is switched on, the robot shall complete its self-test and then wait with its drives disabled, its brakes engaged and its implement off until the operator's deliberate reset and a new arm command (D91). |
| --- | --- |
| **Rationale** | SG-08 and N-11: no motion, no brake release and no implement start at power-up without a deliberate command. D91 applies the reset policy of D88 to every power-up, since the controller cannot tell a deep brown-out from a switch-on; it replaces the automatic re-arm at the self-test of D30. |
| **Source** | N-11, SG-08, D88, D91 |
| **Parent** | SG-08 |
| **Allocation** | MCU, PWR, DRV, BMS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S0 (today: standby within 1.5 s and at 10 s, no class-A fault; its criteria change with D91); t_mcu_control testTEL40_PowerUp, testTEL14_Transitions; planned: card G1-02 for the wait for the operator's reset (D91) |
| **As built** | **Not implemented** |
| **Where** | MCU mode manager, BOOT to SELFTEST to STANDBY (subsystems/electronics/mcu_mode_manager.m; electronics spec 2.7.3); the power-up sequence of ICD section 6 (D30) |
| **Note** | The self-test and the stopped state work in the twin, and S0 passes in the baseline. The wait for the operator's reset does not: the self-test re-arms the safety relay with no manual action, and the robot comes up in STANDBY (D30; hazard log F-01, STATUS 32). Card G2-MCU makes it D91 with D88 (size M). The self-test needs the twin's BMS frames, which the robot's bought BMS does not send (R-15; the delivered BMS is the baseline, provisionally, D96; card G1-04). Not on the robot yet: the firmware generated from the twin is not flashed (card B-01), and the robot's controller board is still to be confirmed (R-59, card G1-04). |

| REQ-SYS-002 | If its self-test does not pass, then the robot shall enter its fault mode and stay stopped until a deliberate reset. |
| --- | --- |
| **Rationale** | A robot whose rails, drives, BMS or IMU are not healthy at power-up must not become armable. |
| **Source** | N-10, N-11, SG-08 |
| **Parent** | SG-08 |
| **Allocation** | MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL14_Guards (the self-test time-out into FAULT, then a reset to STANDBY) |
| **As built** | **Implemented** |
| **Where** | MCU mode manager, the SELFTEST state and its 2 s time-out (electronics spec 2.7.3) |
| **Note** | The test passes. The self-test needs fresh drive and BMS frames; the frames of the robot's bought BMS are unknown, so the firmware could stop the real robot in its self-test (R-15, card G1-04). After the reset the robot goes to STANDBY without a new self-test; every self-test check but the IMU is also a class-A or class-B fault that blocks the arm, and the IMU follows D85 (REQ-SYS-024, -077, -110). |

| REQ-SYS-003 | When the operator arms the robot for RC driving, the robot shall enter RC driving only with a valid RC link, the sticks at neutral, its power ready and no fault that needs a stop. |
| --- | --- |
| **Rationale** | The arm interlocks keep the robot from moving off at the arm command itself (SG-08; hazard log SF-16). |
| **Source** | N-04, N-11, SG-08 |
| **Parent** | SG-08 |
| **Allocation** | MCU, COM, BMS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL14_Guards, testTEL14_NeverActiveWithoutArm; S1 (the arm edge with the sticks at neutral) |
| **As built** | **Partly** |
| **Where** | MCU mode manager, the STANDBY to MANUAL guard (electronics spec 2.7.3) |
| **Note** | The guards work in the twin. An arm edge can come from the receiver's failsafe preset instead of the operator (R-35), and the interlocks run in one MCU (PL b at most, hazard log SF-16; card G2-FSC). One corrupted frame that passes the checks can also give an arm edge, as the 2-position switches have no debounce (REQ-SYS-115; the electronics and comms engineer's finding F1, card G2-MCU). |

| REQ-SYS-004 | When the operator arms the robot for an autonomous mission, the robot shall warn the people around it by sound and light for the start-warning time of 3 s before it moves. |
| --- | --- |
| **Rationale** | SG-08 asks for a warning before a start; people near the robot need the time to step away. The start-warning time is 3 s (P.electronics.safety.auto_start_warn_t, params/param_electronics.m; the MCU firmware engineer, step 4). Margin: a person beside the robot needs about 1 s to notice and react and about 1.5 s to step 1.5 m clear at 1 m/s, which clears the deck's 1.2 m reach ahead of the body centre (D72, D81): 2.5 s, 0.5 s inside the 3 s. While the robot moves in AUTO the warning goes on as a beep every 2 s. It is safe because a person beside the robot is clear before it moves. Set by risk assessment and not yet confirmed against ISO 18497 (D103). |
| **Source** | N-07, N-11, SG-08, D103 |
| **Parent** | SG-08 |
| **Allocation** | MCU, ELC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL30_31_AutoStartOverride |
| **As built** | **Hardware** |
| **Where** | MCU mode manager, AUTO_START (3 s, buzzer and green beacon); the auxiliary outputs (electronics spec 2.7.12) |
| **Note** | The twin warns for 3 s, then beeps every 2 s in AUTO (hazard log SF-15). The beacon and the buzzer are not in the as-built record (card G1-04). |

| REQ-SYS-005 | When the operator moves the mode switch to manual, or moves a stick beyond 30 % of its travel, during autonomous operation, the robot shall pass to RC driving within the takeover time of 0.3 s, counted from the switch or from the stick passing 30 %. |
| --- | --- |
| **Rationale** | The operator is the final authority over the robot's motion: the RC commands win over the companion's (architecture section 1). The takeover time is 0.3 s: a stick override acts once a stick passes 30 % of its travel (rcmap.override_tol 0.30) and stays there for rcmap.override_t 0.20 s, plus D37's stick-to-ESC latency of at most 62 ms (0.26 s), and the mode switch its debounce, which accepts a held change within 40 ms (4 agreeing samples at 10 ms, spanning 30 ms), plus 62 ms (0.10 s) (P.electronics.rcmap, params/param_electronics.m; the MCU firmware engineer, step 4). |
| **Source** | N-04, N-05, D37 |
| **Parent** | None |
| **Allocation** | MCU, GNC, COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL30_31_AutoStartOverride; t_gnc_mission testT11ModeLogic |
| **As built** | **Implemented** |
| **Where** | MCU mode manager, AUTO_START and AUTO to MANUAL on the mode switch or a stick override (electronics spec 2.7.3); the GNC mission manager, transition G2 (gnc spec 4.1) |
| **Note** | The tests pass in the baseline. The GNC follows the mode switch only: after a stick override it does not pause; when the sticks are let go with the switch still at AUTO, the MCU stays in MANUAL, since it enters AUTO_START from MANUAL only on a change of the switch to AUTO (mcu_mode_manager.m; corrected by the MCU firmware engineer, step 4, section 8). |

| REQ-SYS-006 | When the operator disarms the robot while it moves, the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | Disarming is the operator's normal way to stop, and it must end on the brakes on any slope (SG-05, D71). |
| **Source** | N-04, N-05, SG-05, D71 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL14_Transitions (the disarm: arm off in MANUAL, then CTRL_STOP), testTEL48_ClassBStopParks, testTEL52_CtrlStopTimeoutV0 (the same CTRL_STOP and park path); t_drive testT19b_parkOnDescent (the drive's park) |
| **As built** | **Implemented** |
| **Where** | MCU mode manager, ACTIVE to CTRL_STOP on a disarm, then the parking phase (D71; electronics spec 2.7.3) |
| **Note** | The tests pass. The park confirmation only echoes the brake command (STATUS 27), so a stuck brake is not seen; the ESC manual comes on card G1-04. |

| REQ-SYS-007 | While the lid of its electronics enclosure is open, the robot shall not start an autonomous mission. |
| --- | --- |
| **Rationale** | A complementary guard interlock, not credited as a safety function: the enclosure is opened for maintenance, and the robot must not drive off on its own while it is open. The hazard is an unexpected start (SG-08), which the hazard log meets during maintenance by isolating the energy (REQ-SYS-080); the safety concept (G2-FSC) may credit the interlock under SG-08. |
| **Source** | N-11, N-12, SG-08 |
| **Parent** | None |
| **Allocation** | MCU, ELC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bit 30), testTEL14_Guards (an open lid blocks AUTO) |
| **As built** | **Hardware** |
| **Where** | MCU safety supervisor, fault bit 30 LID_OPEN, class C, AUTO inhibited (ICD section 6) |
| **Note** | Implemented in the twin. A lid switch is not in the as-built record (card G1-04). |

| REQ-SYS-080 | While its pack is isolated for maintenance, the robot shall be unable to move, release its brakes or start its implement. |
| --- | --- |
| **Rationale** | SG-08: maintenance shall be done with the energy isolated. |
| **Source** | N-12, SG-08 |
| **Parent** | SG-08 |
| **Allocation** | PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, on the bench: planned: card G1-02 (the isolator and the procedure; the manuals of card V-06) |
| **As built** | **Hardware** |
| **Where** | the key switch S_KEY (AUX48 and Loop_OK; power spec 2.4, 2.5); the spring brakes engage without power (drive spec 2.7) |
| **Note** | No lockable isolator or isolation procedure is recorded (the hazard log's SG-08 status); the brakes hold without power by design. In the twin the key switch removes AUX48 (no LV, so no ESC logic) and opens Loop_OK, but leaves the pack terminals and the BMS live up to K1. The robot's SB120 main connector can be unplugged, but it is not a lockable isolator (the as-built record). |

### 5.2 F-02 Drive by remote control

**Scope:** The RC commands to motion, the RC speed limit, the stop when the sticks are let go, the RC switches. (6 requirements, all of them safety requirements.)

| REQ-SYS-008 | While in RC driving, the robot shall command its tracks only as the operator's sticks command, within 65 ms of the sticks at the ESC speed reference (D37): in their direction, and with no more speed or turn rate than they command beyond an overshoot of 5 % of the commanded speed, a steady error of 0.03 m/s and 10 % of the commanded turn rate. |
| --- | --- |
| **Rationale** | SG-11: in RC driving the robot moves only as the operator commands. The safety content is the bound; how closely the robot tracks the sticks is the performance requirement REQ-SYS-120. The tracking tolerance is S1's (architecture section 12: overshoot at most 5 %, steady error at most 0.03 m/s, yaw rate within 10 % after 2 s) and the latency D37's 65 ms (a budget of 38 to 62 ms) (the MCU firmware engineer, step 4). Margin: the bound applies to the references that the MCU commands to the tracks, as in REQ-SYS-012, and D37's 65 ms is measured to the ESC speed reference; the closest trip is D74's descent trip at 1.44 m/s at every pitch, level ground included (the top-speed entry of 1.41 m/s plus dv_top 0.03 m/s), 0.04 m/s above the 1.40 m/s RC limit, so a reference more than about 3 % over the limit parks the robot; the 1.70 m/s overspeed trip and the speed-follow trip at 0.2 m/s above the command (REQ-SYS-109) follow; 65 ms is 9 cm of travel at 1.40 m/s, which the stop distance of REQ-SYS-009 includes. It is safe because a motion beyond the tolerance meets the trips before it can grow. |
| **Source** | N-04, SG-11, D37 |
| **Parent** | SG-11 |
| **Allocation** | MCU, COM, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S1 (speed rise, overshoot, steady error, yaw rate); t_mcu_control testTEL24_ThrottleAndLatency, testTEL25_PivotAndMixer; t_drive testT01_ramp, testT22_stopProfile (the drive's part) |
| **As built** | **Partly** |
| **Where** | RC link (subsystems/rc); MCU SBUS decode, RC mapping, shaper and mixer (electronics spec 2.6, 2.7.2 to 2.7.6) |
| **Note** | S1 passes. The RC chain is not safety-rated (PL b at most, card G2-FSC); the robot's RC transmitter and receiver are not in the as-built record (card G1-04); the results are for the 1 kW default (D95); the 750 W option is selectable, and every result records its motor. |

| REQ-SYS-009 | When the operator lets go of the sticks, the robot shall come to a stop within 0.95 m from 1.2 m/s (architecture section 12, S1), and within 1.25 m from its RC speed limit of 1.40 m/s. |
| --- | --- |
| **Rationale** | SG-11: the robot stops when the operator lets go. S1's limit is 0.95 m from 1.2 m/s (architecture section 12); D33's replay gave 0.88 m, 60 ms of latency included. RC driving runs up to the RC speed limit (REQ-SYS-010), above 1.2 m/s, so the stop from that speed needs its own distance. The stop distance from the RC speed limit is 1.25 m: a replay of the MANUAL shaper law (electronics spec 2.7.5: K_v 4, a_dec_man 1.2 m/s^2, j_max 5 m/s^3; the ESC's first order of 40 ms; 60 ms of latency) reproduces D33's 0.88 m from 1.2 m/s and gives 1.14 m from 1.40 m/s (the MCU firmware engineer, step 4). Margin: 0.11 m (9 %) over the replay, as S1's own margin at 1.2 m/s (0.95 m against 0.88 m), on level ground; on a descent the cap and its stop-go parks govern (D74). It is safe because the replay of the documented law reproduces the stop that S1 measures. |
| **Source** | N-04, SG-11, D33 |
| **Parent** | SG-11 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S1 (the stop distance after the throttle release from 1.2 m/s); planned: card G1-02 for the stop from the RC speed limit (S1 or a bench run at 1.40 m/s, to confirm the replay's 1.14 m) |
| **As built** | **Implemented** |
| **Where** | MCU reference shaper, the MANUAL deceleration (electronics spec 2.7.5; D33) |
| **Note** | S1 passes. On a descent with a pack that cannot take charge, the stop rests on the descent cap and its stop-go parks (D74). |

| REQ-SYS-010 | While in RC driving, the robot shall keep its speed within the RC speed limit of 1.40 m/s. |
| --- | --- |
| **Rationale** | SG-11 asks for the speed limit of RC driving. The RC speed limit is 1.40 m/s (P.electronics.lim.v_max_man, params/param_electronics.m:41; the MCU firmware engineer, step 4). Margin: the closest trip is D74's descent trip at 1.44 m/s at every pitch, level ground included (the top-speed entry of 1.41 m/s plus dv_top 0.03 m/s), 0.04 m/s above the limit, so a speed more than about 3 % over the limit latches OVERSPEED and parks the robot; the overspeed trip at 1.70 m/s is 0.30 m/s above it, the speed-follow trip 0.2 m/s above the command, and the stop from it takes 1.25 m (REQ-SYS-009). It is safe because the stop from the limit stays within the RC stop distance. |
| **Source** | N-04, SG-11 |
| **Parent** | SG-11 |
| **Allocation** | MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S1; t_mcu_control testTEL24_ThrottleAndLatency |
| **As built** | **Implemented** |
| **Where** | MCU speed limits and reference shaper (electronics spec 2.7.5) |
| **Note** | At full stick on the 20 deg ramp the top-speed trip fires at the slope entry: a safe but nuisance park (STATUS 29, card G2-MCU). The top-speed trip is at 1.44 m/s at every pitch, level ground included (D74). |

| REQ-SYS-011 | The robot shall take the operator's mode, arm, reset, remote-stop and mission commands only from valid RC frames from the operator's transmitter, never from its receiver's failsafe output. |
| --- | --- |
| **Rationale** | A receiver failsafe or a lost frame must not arm, reset, stop or start the robot as if the operator had (D34, SG-08). D84 makes it matter for the mission commands too: a move-off after a person stop must be the operator's. |
| **Source** | N-04, N-11, SG-08, D34 |
| **Parent** | SG-08 |
| **Allocation** | MCU, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL26_Failsafe; t_mcu_platform testRemoteEstopValidFramesOnly (the remote stop); t_rc_link failsafe_modes |
| **As built** | **Partly** |
| **Where** | MCU RC mapping and the REMOTE_ESTOP evaluation (D34; electronics spec 2.7.2) |
| **Note** | The remote stop is taken only from valid frames (D34). Arm, reset and the mission commands (START and PAUSE on CH13, RTH on CH14) are not: with another failsafe preset on the mode switch, the robot can re-enter manual by itself after a link loss, and a preset can send a start, a pause or a return to the dock (R-35, card G2-MCU). One corrupted frame that passes the checks can also give a reset or return-home edge, as the 2-position switches have no debounce (REQ-SYS-115; finding F1, card G2-MCU). |

| REQ-SYS-012 | If an RC frame arrives corrupted, then the robot shall keep the resulting error of the speed and the turn rate that it commands to its tracks within the glitch limit of 0.02 m/s and 0.02 rad/s. |
| --- | --- |
| **Rationale** | SG-11: a corrupted frame must not command a motion that the operator did not give. SBUS has no checksum, so the robot can discard only the frames that fail their parity, framing, header, footer or range checks; a sustained loss is the lost link of REQ-SYS-067. The glitch limit binds the speed and turn-rate references after the shaper, which the drives follow, not the target: one corrupted frame that passes the decoder's checks (even-bit errors pass parity; SBUS has no CRC) can put any in-range value on a channel for one 14 ms frame (P.rc.frame_period, params/param_rc.m:6), which the 10 ms control task sees in at most two samples; the shaper's jerk limits (shaper.j_max 5 m/s^3, jw_max 8 rad/s^3, params/param_electronics.m:48) bound the change to 0.002 m/s and 0.0032 rad/s (the electronics and comms engineer, step 8). Margin: 10 times in speed and 6 times in turn rate for one frame, and 4.4 and 2.8 times for two corrupted frames in a row (about 4e-8 per frame pair at a bit error rate of 1e-3). It is safe because 0.02 m/s over the 0.1 s that the shaper needs to recover moves the robot about 2 mm, and 0.02 rad/s turns it about 0.002 rad, far below D100's hold and clearance distances. The discrete commands on a corrupted frame are REQ-SYS-115 (the lead systems engineer's ruling 5). |
| **Source** | N-04, SG-11 |
| **Parent** | SG-11 |
| **Allocation** | MCU, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S6c (today: no command glitch above 2 % of full scale on the targets; card G2-COM judges it on the references telem.v_ref and telem.w_ref and adds errors that parity cannot catch, R-47); t_mcu_platform testTEL03_ByteErrors; t_rc_link byte_errors |
| **As built** | **Partly** |
| **Where** | MCU SBUS frame sync and decoder: a frame with a UART error, a bad header or footer, or a channel out of range is discarded (sbus_frame_sync, fw_sbus_decoder; electronics spec 2.6.3, 4.1) |
| **Note** | testTEL03_ByteErrors passes. An even number of bit errors in a byte passes parity and is not detected. S6c injects only errors that parity catches, so it cannot fail (R-47, card G2-COM). |

| REQ-SYS-115 | If an RC frame arrives corrupted, then the robot shall not change its mode, arm, reset, remote-stop, implement, lift or mission commands on that frame. |
| --- | --- |
| **Rationale** | SG-08 and N-04, split from REQ-SYS-012 so that each stays singular (the lead systems engineer's ruling 5 of step 13, on the electronics and comms engineer's text of step 8). Its content is that no corrupted frame gives an arm, a reset, a mission start or an implement start: an unexpected start-up (SG-08, with REQ-SYS-011), not the stick motion of SG-11, which stays in the source; ruling 5's same parent is amended for this requirement (the functional safety engineer, step 17, section 4; the lead systems engineer's point 18). One corrupted frame that passes the decoder's checks can put any in-range value on a channel for one frame (14 ms). The 3-position switches need 4 agreeing samples at 10 ms, a debounce of 30 ms (3 delays and the current sample; P.electronics.rcmap.debounce_n, params/param_electronics.m:30), the remote stop the 5 samples of fault bit 1's persistence (40 ms, P.electronics.safety.t_persist, :99) and the AUTO override 0.2 s (rcmap.override_t, :31). Margin: against one 14 ms frame, about 16 ms on the 3-position switches and 26 ms on the remote stop. Not shown when rejected frames follow: the decoder keeps the last accepted channels while rc_ok holds (up to 0.10 s without a valid frame), so a corrupted frame that passes the checks, followed by two rejected frames, keeps its values for about 42 ms, which passes the debounce (the register's re-check, step 14b). It is safe once card G2-MCU debounces on new accepted frames and debounces the 2-position switches: no single corrupted frame can then arm the robot, reset a stop, start a mission or turn on its implement. |
| **Source** | N-04, SG-08, SG-11 |
| **Parent** | SG-08 |
| **Allocation** | MCU, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-MCU (single corrupted frames injected on CH5, CH6, CH8, CH9, CH11, CH13 and CH14, alone and followed by rejected frames: no mode change, arm, reset, implement, lift, mission or return-home edge); t_rc_link byte_errors |
| **As built** | **Partly** |
| **Where** | MCU RC mapping: the 3-position switches debounced over 4 samples (rcmap.debounce_n), the remote stop's 40 ms persistence, the 2-position switches decoded as x > sw2_thresh with no debounce (scripts/build/build_fw_platform.m:209-213) |
| **Note** | The mode, implement, lift and mission switches are debounced over 30 ms (4 samples at 10 ms); the 2-position switches (arm CH6, reset CH9, lights CH10, return home CH14) are not, so today one corrupted frame that passes the checks can give an arm, reset or return-home edge (the electronics and comms engineer's finding F1, card G2-MCU). A corrupted frame that passes the checks and is followed by rejected frames is held by the decoder for up to 0.10 s, longer than the debounce (card G2-MCU). |

### 5.3 F-03 Run missions

**Scope:** Take a mission from the companion and run its rows, turns, transit and docking; start, hold, resume, abort and finish; report the progress. (10 requirements, one of them a safety requirement.)

| REQ-SYS-013 | When the operator commands a mission start, the robot shall start the mission only with a valid position within the accuracy bound of REQ-SYS-022, a valid plan, its drives ready and its charge above the start threshold of SOC 0.35. |
| --- | --- |
| **Rationale** | A mission that starts without a position, a plan, its drives or the energy to finish would stop part way (N-02, N-05). The start threshold is SOC 0.35 (P.gnc.fsm.soc_start_min, params/param_gnc.m:121; the GNC engineer, step 5): the return threshold at 500 m is 0.237 (T12, 2026-11-16), so 0.35 leaves charge for work above the return. A start check on the mission's own energy (the plan's estimate plus the reserve plus D101's 10 %) is a design item for card G2-GNC. |
| **Source** | N-02, N-05 |
| **Parent** | None |
| **Allocation** | GNC, MCU, NAV, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testReadyAndStartGating |
| **As built** | **Partly** |
| **Where** | GNC mission manager, transition T1 (subsystems/gnc/gnc_missionFSM.m; gnc spec 4.1) |
| **Note** | The test passes. The start (T1, T2) checks only the 2 m health flag (nav_ok), not the accuracy gate, so the robot can start above the gate and pause again 2 s later; NOT nav_degraded in T1, T2, P1 and P3 is card G2-GNC (size S). The charge behind the gate is the BMS's estimate; T-PWR-11 (t_pack_bms soc_estimation_T11) fails in the baseline on the former pack's expectations (R-29). |

| REQ-SYS-014 | While it runs a mission, the robot shall work through the plan's segments in order: its rows, its turns at the row ends and its transits. |
| --- | --- |
| **Rationale** | N-01: mow the rows of an aisle or a grove with no operator at the controls, turning at the row ends. |
| **Source** | N-01, N-03 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4; S5b; t_gnc_mission testT8Greenhouse, testT9PalmT20 |
| **As built** | **Partly** |
| **Where** | GNC mission manager and path server, segment transitions S1 to S5 (gnc spec 4.1) |
| **Note** | S5b passes 13 of 13 (STATUS 5.3). With the mower, S4 holds at the lane-end walls and does not reach a row (STATUS 18); the run of 2026-11-14, before the field of D72, completed with coverage 0.971 but failed S4.xte_max (STATUS 16e); S5 cannot pass by design of its criteria (R-63). |

| REQ-SYS-015 | When the operator commands a pause, the robot shall stop on its path and hold, also when its charge falls below the return threshold (D93), until the operator commands a resume, a return to the dock or an abort. |
| --- | --- |
| **Rationale** | N-05: the operator holds a mission. D93: a robot paused by its operator does not set off by itself, which would be a start without a deliberate command (SG-08); on a low charge it tells the operator (REQ-SYS-105) and drives home only on the operator's return command. |
| **Source** | N-05, SG-08, D93 |
| **Parent** | None |
| **Allocation** | GNC, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT11ModeLogic, testT11ClosedLoop |
| **As built** | **Partly** |
| **Where** | GNC mission manager, T6 into PAUSE, and P1 to P4 out of it (gnc spec 4.1) |
| **Note** | The tests pass. P3 also leaves an operator's pause by itself when the charge falls below soc_rth during a mission (gnc_missionFSM.m:286-290): the robot then drives home without an operator command. D93 removes that exit (card G2-GNC, with D84 and D92, size M). |

| REQ-SYS-016 | When the operator commands a resume, the robot shall continue the mission from the point where it stopped. |
| --- | --- |
| **Rationale** | N-05: resume a mission without planning it again from its start. |
| **Source** | N-05 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testResumeAlignHeading, testRejoinGlobalFallback |
| **As built** | **Partly** |
| **Where** | GNC mission manager, P1: continue, pivot in place (ALIGN) or rejoin (D67; gnc spec 4.1) |
| **Note** | A replan answer that arrives during a hold is lost, and the robot then waits for ever (R-11); the rejoin and bypass goals do not match the planner's lane (R-49). The resume pivot is REQ-SYS-091 to -093 (D87). The operator's resume (P1, P3) checks only the 2 m health flag, not the accuracy gate (card G2-GNC, with REQ-SYS-013). |

| REQ-SYS-017 | When the operator commands an abort, the robot shall end the mission, stop and wait for a new command. |
| --- | --- |
| **Rationale** | N-05: the operator stops a mission for good. |
| **Source** | N-05 |
| **Parent** | None |
| **Allocation** | GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT11ClosedLoop |
| **As built** | **Implemented** |
| **Where** | GNC mission manager, T9 and P4 into IDLE, and the abort from OBSTACLE_HOLD and RETURN_TO_HOME into IDLE (gnc spec 4.1) |
| **Note** | The test passes. |

| REQ-SYS-018 | When the robot completes the last segment of its mission, the robot shall return to its dock or stop and wait, as the plan's end action sets. |
| --- | --- |
| **Rationale** | N-02 and N-03: the robot goes back to its dock at the end of its work. |
| **Source** | N-02, N-03 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT8Greenhouse, testT9PalmT20 (mission_done) |
| **As built** | **Implemented** |
| **Where** | GNC mission manager, T10 (gnc spec 4.1) |
| **Note** | The tests pass. The return then depends on docking (REQ-SYS-058). |

| REQ-SYS-019 | If the robot cannot plan a path to rejoin its mission or to return to its dock, then the robot shall pause the mission and report the reason to the operator. |
| --- | --- |
| **Rationale** | N-05: the operator must see a hold and its cause; a robot that waits with no reason cannot be recovered. |
| **Source** | N-05, N-10 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testRejoinGlobalFallback, testT18HybridAStar |
| **As built** | **Partly** |
| **Where** | GNC mission manager, R2 (PLANNER_FAIL), and the global replanner (gnc spec 4.1, 4.4) |
| **Note** | A replan answer that arrives during a hold is lost, and the robot then waits for ever with no reason shown (R-11); the hold reasons cannot name a lost plan (R-40). |

| REQ-SYS-091 | While it runs a mission, the robot shall turn in place only on the segments where its plan permits a pivot (D87). |
| --- | --- |
| **Rationale** | D87: the robot pivots in place only where the footprint of the robot with its fitted implement fits, a planning measure for HAZ-004 (SG-01); the plan's permission is REQ-SYS-093. |
| **Source** | N-05, SG-01, D87 |
| **Parent** | None |
| **Allocation** | GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testResumeAlignHeading (the resume pivot) |
| **As built** | **Implemented** |
| **Where** | GNC mission manager, P1: the ALIGN pivot only on a segment that allows pivoting, with the tilt within align_tilt and not on the dock approach (D67; gnc spec 4.1) |
| **Note** | The test passes. The flag gates the start, segment-entry and resume pivots; the dock trim at the end of the dock approach runs on the dock segment's flag, set today with no footprint check (REQ-SYS-093, R-10). The flag itself is not yet set from the footprint (R-06). Card G2-GNC brings the dock trim under the flag or gives it its own check (D87). |

| REQ-SYS-092 | If its resume pivot is blocked, then the robot shall stop and hold. |
| --- | --- |
| **Rationale** | D87: the robot holds when a pivot is blocked; today nothing detects a blocked pivot (STATUS 16c). A pivot that pushes on can pin a person against a post (HAZ-004, SG-01). |
| **Source** | N-05, N-07, SG-01, D87 |
| **Parent** | SG-01 |
| **Allocation** | GNC, NAV, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 |
| **As built** | **Not implemented** |
| **Where** | none: ALIGN has no time-out, and the stuck check needs at least 0.3 m/s (STATUS 16c) |
| **Note** | The blocked-pivot hold is card G2-GNC (D87, size M). One way to detect it is a heading-progress time-out on ALIGN, on the navigation's heading and yaw rate; the other is a sweep check against the occupancy grid before the pivot, which needs the perception only if card G2-GNC picks it (STATUS 16c). |

| REQ-SYS-094 | When the operator commands a return to the dock, the robot shall end its mission and return to its dock. |
| --- | --- |
| **Rationale** | N-03 and N-05: the operator sends the robot home before its work is done. REQ-SYS-085 lists the command; the docking itself is REQ-SYS-058. |
| **Source** | N-03, N-05 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT36DriveReadyDebounce (the operator's return from a mission, T8); planned: card G1-02 for the return from standstill and from a pause |
| **As built** | **Implemented** |
| **Where** | GNC mission manager: T2 from IDLE, T8 in a mission and P3 in a pause, into RETURN_TO_HOME with a global plan to the start of the dock approach (gnc spec 4.1) |
| **Note** | The command is not taken in an obstacle hold; the operator pauses first. The return then depends on docking (REQ-SYS-058, R-10). |

### 5.4 F-04 Know where the robot is

**Scope:** Localisation from GNSS, IMU, odometry and scan matching; the validity of the estimate; the permitted zone; the stop when the position cannot be shown. (9 requirements, 7 of them safety requirements.)

| REQ-SYS-020 | While it runs a mission, the robot shall keep the error of its position estimate within the accuracy of its kind of path: across the path at most 0.045 m RMS and 0.10 m in a greenhouse or 0.15 m in the open; along it at most 0.10 m RMS and 0.25 m; at most 0.25 m at the end of a GNSS outage of the outage time, and within the open-field values again 5 s after the RTK fix returns; and in the dock approach at most 0.03 m RMS and 0.08 m across the path. |
| --- | --- |
| **Rationale** | N-09 and SG-03: the robot must know where it is to stay in its zone and near its path. Part 2 sets the accuracy as one error budget with REQ-SYS-021, -022, -026 and -027: the sensors and navigation engineer's values (step 6, Table 1) with the guidance shares of the GNC engineer's combined budget (step 5b, Table 1), by the lead systems engineer's ruling 8 (ruling 9, rows on a slope, withdrawn by the lead systems engineer's ruling 4 of step 19). The across RMS is 0.045 m, so that with the 0.02 m guidance RMS the true cross-track RMS is sqrt(0.02^2 + 0.045^2) = 0.049 m, within REQ-SYS-027's 0.05 m; rows on a slope meet the row value: T-NAV-11's horizontal RMS is 0.039 m (2026-11-08; its 0.049 m is the 3-D RMS, the vertical error included), so the true cross-track RMS there is sqrt(0.02^2 + 0.039^2) = 0.044 m. Margin: the measured errors sit below the values: T-NAV-07 0.022 m RMS and 0.065 m at most (2026-11-14), T-NAV-04 0.023 m and 0.104 m (2026-11-15), S4 0.030 to 0.037 m RMS, T-NAV-05 0.108 m after 30 s (2026-11-14). It is safe because each value plus the guidance share stays inside the bound it serves: in a greenhouse 0.10 + 0.05 = 0.15 m, the maximum of REQ-SYS-027 (D100), within D106's planned 0.275 m from the deck to the posts; in the open 0.15 + 0.05 = 0.20 m, inside the path bounds of REQ-SYS-026. |
| **Source** | N-01, N-09, SG-03, D89, D100, D106 |
| **Parent** | SG-03 |
| **Allocation** | SNS, NAV, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (the criteria of D100); S7 (farm, wet, temp); t_nav tNAV02_LawnMower, tNAV04_PalmRow, tNAV07_GreenhouseLidar, tNAV11_Slope (the 20 deg slope, judged on its horizontal RMS); t_perception tScanMatchGreenhouse, tSM01_AlongAisleHonesty |
| **As built** | **Partly** |
| **Where** | Shaddad_NavFusion, the EKF and the navigation mode manager; Shaddad_Perception, the scan matcher (sensors spec 3.5 to 3.7); the sensors and their acquisition (sensors spec 3.1 to 3.4; sens_acq_gnss, sens_acq_imu, sens_acq_lidar) |
| **Note** | S7 fails in the baseline (farm, wet and temp); S4's localisation is optimistic (R-23); NavFusion has no level calibration, so the mounting tilt biases the 1.05 m antenna lever arm by a few cm (R-21); on the hardware, both latency compensations are silently off (R-22). S4's criteria are D100's; S7's position criterion (0.5 m at the end of the outage) is looser than the 0.25 m here, to align on card G2-INT. Not met today: the navigation maximum in the S4 rows (0.131 m after the entry and 0.157 m at it, against 0.10 m; card G2-NAV), and S7's re-convergence (0.24 m 5 s after RTK fixed returns, S7wet, 2026-11-08). No GNSS receiver, LiDAR or companion computer is in the as-built record (card G1-04). The 0.045 m across RMS is the GNC engineer's budget, which the sensors and navigation engineer confirms in step 17 (L-8). tNAV11_Slope judges the 3-D RMS today (rmsP3, tests/sensors/t_nav.m:632-638); the across value is judged on its horizontal RMS (card G2-NAV). |

| REQ-SYS-021 | If GNSS is lost during a mission in the open, then the robot shall continue its mission on its other sensors for the outage time of 30 s. |
| --- | --- |
| **Rationale** | N-10: a lost GNSS must not end every mission in a palm grove; the operating concept lists it as an abnormal case (charter section 6). The outage time is 30 s, S7's outage, then 10 s of float (the charter's palm-grove case, section 6); T-NAV-05 (2026-11-14) gives 0.108 m of error and 0.33 deg of yaw drift after 30 s with a 90 deg turn (the sensors and navigation engineer, step 6; kept in the GNC engineer's budget of step 5b). It holds within the palm-grove gate of 0.15 m and the path bound of 0.50 m (REQ-SYS-022, -026) with an honest covariance. |
| **Source** | N-09, N-10 |
| **Parent** | None |
| **Allocation** | NAV, SNS, PER, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S7 (farm, wet, temp); t_nav tNAV05_GnssOutageTurn, tS7OutageWetSoil |
| **As built** | **Partly** |
| **Where** | NavFusion dead reckoning; the GNC navigation-degraded rule (T7, D69) |
| **Note** | S7 fails in the baseline in all three variants; a GNSS outage gives a NAV pause about 4 s after RTK is lost (STATUS 4), and the covariance is optimistic (STATUS 5). The pause comes from the GNC's gate: it pauses on the quality term (q_good, D69) 2 s after the RTK fix is lost in the open, and faults at a sigma of 0.30 m, so the outage time needs the gate changed as well as an honest covariance (card G2-GNC). As drafted in part 1, the outage time and the bounds of REQ-SYS-022 cannot both pass S7; part 2 sets them as one budget. Decided, pending: it holds only with three changes: the quality term out of the gate's pause (card G2-GNC), the dead-reckoning limit of REQ-SYS-023 at 35 s (card G2-NAV; 10 s today, P.sensors.nav.tAbsMax), and an honest covariance (REQ-SYS-124, card G2-NAV). Until then S7 fails, as recorded. |

| REQ-SYS-022 | When its position uncertainty stays at or above the bound of its kind of path for 2.0 s, the robot shall pause the mission: across the lane 0.10 m on greenhouse rows and turns, 0.15 m on palm-grove rows and turns and on transits, and 0.04 m on the dock approach; along it 0.25 m, and 0.04 m on the dock approach. |
| --- | --- |
| **Rationale** | SG-03: the robot keeps within a bounded distance of its path only while it knows its position well enough. D89 makes this the second layer, the accuracy that the path needs; the first layer, the health flag, is REQ-SYS-023. The bounds are the GNC engineer's combined budget (step 5b, Table 1 and section 4; L-8), on the filter's own 1-sigma: the gate clears at 0.07 m across in a greenhouse, 0.10 m in the open and 0.03 m on the dock approach, and at 0.20 m along (0.03 m on the dock approach). Margin: at the gate the guidance share plus 3 sigma equals the path bound of REQ-SYS-026 for each kind of path, so the true deviation in normal running stays within the bound at 99.7 %, and the normal sigma sits 1.4 times (greenhouse turns) to 5.8 times (palm rows) below the gate, so normal running does not pause; the 3-sigma margin holds only with an honest covariance (REQ-SYS-124; S7: 0.41 in bound against 0.90). It is safe because the pause comes before a navigation error that the path bound cannot see carries the robot past it. Rows on a slope take the row bounds (the lead systems engineer's ruling 4 of step 19). |
| **Source** | N-09, SG-03, D69, D89 |
| **Parent** | SG-03 |
| **Allocation** | NAV, GNC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: t_gnc_mission testT7NavSigmaRule, testT7NavSigmaPause; S4 (no NAV_DEGRADED pause on the rows: D69's margins of 0.094 m across and 0.157 m along are to today's 0.15 m and 0.25 m gate; to the decided 0.10 m greenhouse gate the margin across is about 0.04 m, 1.6 times S4's sigma of at most 0.061 m); S7 (the pause after the RTK loss) |
| **As built** | **Partly** |
| **Where** | GNC input conditioning, nav_degraded with the cross- and along-track sigma relays (D69), and T7 into PAUSE (gnc spec 4.1) |
| **Note** | The tests pass. The rule uses the filter's own covariance, which is optimistic (STATUS 5; REQ-SYS-095). Above 0.30 m the mission faults instead of pausing (REQ-SYS-023). The gate also pauses on the quality term and an odometry mismatch (D69), and P2 resumes by itself after fsm.t_nav_rec 5 s of good navigation (fsm.auto_resume_nav); whether the robot may resume by itself is for the safety concept (G2-FSC). In S7 the gate pauses well before the outage time of REQ-SYS-021 today, through its quality term; the budget takes the quality term out of the pause on card G2-GNC. Card G2-GNC. Decided, pending: the twin changes on card G2-GNC, with the kind of path from the plan's mission type; today 0.15 m across and 0.25 m along on every path, clear at 0.10 and 0.20 m (P.gnc.fsm.sigma_pause, sigma_degr, sigma_along, sigma_along_off), and 2 s (fsm.t_nav_degr). |

| REQ-SYS-023 | If its position becomes invalid during autonomous operation, through a failed position estimate, an IMU loss, 35 s or more without an absolute position fix, or a position uncertainty of 2 m or more (D89), then the robot shall start a controlled stop within 0.5 s and hold on its brakes. |
| --- | --- |
| **Rationale** | SG-03: the robot stops when it cannot show where it is. D89 makes this the first of two layers: the validity flag is a health flag (the filter runs, and the uncertainty is under 2 m); the accuracy that the path needs is the second layer, the guidance's gate of REQ-SYS-022. The dead-reckoning limit bounds how long the robot trusts an uncertainty that nothing checks (STATUS 5). The dead-reckoning limit is 35 s and the reaction time 0.5 s (the sensors and navigation engineer, step 6; L-3), provisional until the field data on card G2-NAV and checked by the functional safety engineer in step 17: 35 s is the outage time of REQ-SYS-021 plus 5 s for the receiver to re-acquire. Margin: at 35 s the twin's error is about 0.15 m (T-NAV-05's 0.108 m at 30 s, its lateral part growing with the square of the time), 3.4 times inside the 0.5 m path bound, so a robot three times worse than the twin stays inside it; the 0.5 s leaves 0.15 s over today's 0.35 s (fsm.t_nav_lost 0.3 s plus one 0.05 s GNC step), during which the filter keeps dead reckoning and the robot travels at most 0.6 m along its path at 1.2 m/s. It is safe because the stop starts while the position error is still inside the path bound. |
| **Source** | N-09, SG-03, D73, D89 |
| **Parent** | SG-03 |
| **Allocation** | NAV, SNS, GNC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT14NavInvalid; t_nav tModeUnit, tModeUnitImuLoss, tNAV13_ImuLoss; planned: card G1-02, a tModeUnit case: navValid false at a position sigma of 2 m and after the dead-reckoning limit without an absolute update (35 s; 10 s until card G2-NAV), and true below them |
| **As built** | **Implemented** |
| **Where** | The navigation mode manager's flag navValid (subsystems/sensors/sens_nav_mode.m): true while the filter is aligned and running, the IMU is valid, an absolute update came within 10 s, and the larger of the East and North position sigmas is under 2 m (P.sensors.navmode.sigPosValid, params/param_sensors.m). The GNC mission manager, G1: navigation lost for 0.3 s in motion goes to FAULT with v = 0 (gnc spec 4.1); the GNC's nav_ok also needs the position sigma under fsm.sigma_lost 0.30 m (InputConditioning). The MCU's CTRL_STOP and park on the GNC's stop request, or the ESCs' park at zero command (D66, D71) |
| **Note** | The tests pass, and the twin's 2 m flag is the one D89 decides; the sensors spec's 0.15 m is the alignment threshold, written as the validity threshold, and part 2 corrects it. Design deviation: the GNC's 0.30 m FAULT is a third layer that D89 does not have, so the mission faults at 0.30 m instead of pausing between D69's bounds and the 2 m flag (card G2-GNC, size S; the value in part 2). No test checks the 2 m bound or the dead-reckoning limit. Nothing checks for a false RTK fix (R-43; REQ-SYS-095), and the position path is companion software (PL b at most, hazard log E-08, card G2-FSC); the re-alignment without RTK is REQ-SYS-096 (card G2-NAV, R-44). Decided, pending: the twin changes on card G2-NAV (P.sensors.nav.tAbsMax 10 s today, params/param_sensors.m:170); the GNC's 0.30 m FAULT layer (fsm.sigma_lost) must not end an outage earlier (card G2-GNC). |

| REQ-SYS-024 | If its IMU fails during an autonomous function, then the robot shall make a controlled stop, park on its brakes and end the mission. |
| --- | --- |
| **Rationale** | The heading and the attitude come from the IMU; without it the robot cannot show where it is (SG-03) or keep its tilt limits (SG-04). D85 makes the IMU fault class B in every autonomous function. |
| **Source** | N-10, SG-03, SG-04, D73, D85 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS, NAV, GNC, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_nav tNAV13_ImuLoss, tModeUnitImuLoss; t_sensors tImuFaults; t_mcu_control testTEL47_StuckImuFault; planned: card G1-02 for the class-B reaction of D85 |
| **As built** | **Partly** |
| **Where** | sens_acq_imu, the frozen-IMU check; the navigation mode manager (invalid after 0.1 s, FAULT after 1 s; D73); the GNC FAULT on lost navigation; the MCU fault bit 24 IMU_FAULT |
| **Note** | In a mission the navigation goes invalid and the GNC faults with a standstill command, so the robot stops and the mission ends; but the MCU still treats IMU_FAULT as class C (derate 0.25), not as the class-B controlled stop and park of D85 (card G2-MCU, size S). A frozen IMU is caught; a wrong but plausible IMU is not (R-05, hazard log F-03). |

| REQ-SYS-025 | The robot shall stop and hold on its brakes before it or its implement passes the boundary of its permitted operating zone. |
| --- | --- |
| **Rationale** | SG-03: the robot stays inside its permitted operating zone, so it stops before the boundary, not after it. |
| **Source** | N-09, SG-03 |
| **Parent** | SG-03 |
| **Allocation** | GNC, NAV, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT16Geofence |
| **As built** | **Partly** |
| **Where** | GNC mission manager, G1: outside the site boundary grown by fsm.geofence_margin 0.5 m for fsm.t_geofence 0.2 s, into FAULT, in a mission, a hold and a return (gnc spec 4.1); the MCU's CTRL_STOP and park on the GNC's stop request (D71) |
| **Note** | The test passes. The check acts only once the robot is 0.5 m outside the site boundary, and only in a mission, a hold and a return; in RC driving the operator may drive outside the zone (gnc_missionFSM.m:125, 137). It runs on the estimated position, so a localisation error passes unseen (hazard log E-08; REQ-SYS-095); companion software, PL b at most (card G2-FSC). The stopping margin inside the boundary is REQ-SYS-122's: 0.30 m in a greenhouse and 0.50 m in the open (decided, pending on card G2-GNC). |

| REQ-SYS-026 | If the robot moves away from its planned path by more than the path bound of its kind of path for 1.0 s, then the robot shall stop and hold on its brakes: 0.35 m on greenhouse rows, 0.40 m in greenhouse turns, 0.50 m on palm-grove rows, 0.55 m in palm-grove turns, 0.75 m on transits and 0.25 m on the dock approach. |
| --- | --- |
| **Rationale** | SG-03: the robot stays within a bounded distance of its planned path. The path bounds are the GNC engineer's combined budget (step 5b, Table 1 and section 4; L-8): each is the guidance share plus 3 sigma at the gate of REQ-SYS-022. Margin against false stops: the bound less today's largest tracking error is 0.12 m (greenhouse rows), 0.175 m (greenhouse turns), 0.42 m (palm rows), 0.26 m (house transits) and 0.06 m (the dock approach). It is safe because normal running stays within the bound at 3 sigma and a divergence stops after 1.0 s: on palm rows at 1.0 m/s with a 10 deg divergence the true deviation at the stop is at most 0.50 + 0.45 + 0.17 + 0.15 = 1.27 m, inside the 2.5 m half-width of the lane. On greenhouse rows it is up to 0.79 m, beyond the deck's 0.475 m to the wall line (D106); there the inset (REQ-SYS-087), the obstacle and person stops and D79's rule, which keeps people out of the aisle being mowed, protect. Rows on a slope take the row bounds (the lead systems engineer's ruling 4 of step 19). |
| **Source** | N-09, SG-03, D89 |
| **Parent** | SG-03 |
| **Allocation** | GNC, NAV, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (no test of the cross-track fault was found) |
| **As built** | **Partly** |
| **Where** | GNC mission manager, G1: abs(e_y) above 0.5 m for 3 s into FAULT (gnc spec 4.1) |
| **Note** | No test of this fault was found. The check runs on the estimated position, so it cannot see a localisation error (hazard log E-08); the detection of a wrong but confident position is REQ-SYS-095. Decided, pending: the twin changes on card G2-GNC, with the kind of path from the plan's mission type; today 0.5 m for 3 s on every path (gnc spec 4.1, G1). On greenhouse rows a divergence of 10 deg carries the deck from the 0.35 m bound to the 0.475 m wall line in about 0.7 s, before the 1.0 s stop; there the protective field, which holds on the posts, is the guard (the functional safety engineer, step 17). |

| REQ-SYS-095 | If its position is wrong by more than 0.15 m in a greenhouse aisle or on the dock approach, or 0.35 m in the open, then the robot shall start a controlled stop within 0.5 s in a greenhouse aisle or on the dock approach, or 0.7 s in the open, and hold on its brakes. |
| --- | --- |
| **Rationale** | SG-03: the robot stops when it cannot show where it is. REQ-SYS-022 and -023 rest on the robot's own position uncertainty, which a false RTK fix, a multipath jump or a scan-match error does not raise (R-43; HAZ-008, HAZ-009), and the geofence and the path bound judge the estimated position (REQ-SYS-025, -026; hazard log E-08). How the error is detected (an honest uncertainty, a second, independent position source or an integrity monitor) is the safety concept's. The integrity bounds and the detection times are the sensors and navigation engineer's (step 19, section 4), on the stricter drift model, the trackers' lateral acceleration P.gnc.spd.a_lat 0.3 m/s^2 from rest, which drifts 0.04 m in 0.5 s and 0.07 m in 0.7 s (the lead systems engineer's point 11): 0.15 m and 0.5 s in a greenhouse aisle and on the dock approach, 0.35 m and 0.7 s in the open (palm-grove rows and turns, transits). Each bound sits above REQ-SYS-020's normal maximum, so that the monitor does not trip on normal errors: 0.05 m above the 0.10 m in a greenhouse, 0.10 m above the 0.25 m after an outage in the open, and 0.07 m above the 0.08 m on the dock approach. Margin: the bound, the 0.05 m guidance share and the drift give 0.24 m in a greenhouse, 0.01 m inside the 0.25 m from the planned body to the posts with D106's 0.50 m inset (decided, pending on card G2-GNC; under D54 today it is 0.04 m short, and the protective field stops the robot for a post until then); 0.47 m in the open, 0.03 m inside the 0.50 m palm-row path bound (REQ-SYS-026); and 0.24 m on the dock approach, 0.01 m inside its 0.25 m path bound. It is safe because a wrong position within the bound cannot carry the robot to the posts or out of its path bound before the stop starts, and at the zone boundary REQ-SYS-122 keeps its margin over the same sum. The functional safety engineer confirms the drift model at the consistency check, and card G1-02's fault injection later. The honesty of the uncertainty, the rest of the part-1 TBD, is REQ-SYS-124. |
| **Source** | N-09, N-10, SG-03, D69, D89, D106 |
| **Parent** | SG-03 |
| **Allocation** | NAV, SNS, PER, GNC, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (fault injection: a false RTK fix, a position jump, a scan-match error, and a map that differs from the world, R-23); for the honesty of the uncertainty: S7 (NEES in bound), t_nav tNAV02_LawnMower, tNAV07_GreenhouseLidar, t_perception tSM01_AlongAisleHonesty |
| **As built** | **Not implemented** |
| **Where** | none: the geofence and the path bound are checked on the estimated position (REQ-SYS-025, -026); the uncertainty is Shaddad_NavFusion's EKF covariance (posStd, covPose) with its GNSS and scan-match noise models (sensors spec 3.5) and the scan-match covariance from the scan geometry (sensors spec 3.7; D68) |
| **Note** | Nothing checks for a false RTK fix (R-43). The covariance is optimistic: NEES in bound 78.7 % against 90 % in S7 (STATUS 5), and S4's localisation is optimistic (R-23); along an aisle the matcher's capture is one-sided (STATUS 16d). No GNSS receiver is in the as-built record (card G1-04). Hazard log E-08 (class 1) is answered by the safety concept (card G2-FSC). Decided, pending: the monitor and its bounds per kind of path on cards G2-FSC and G2-NAV (the lead systems engineer's point 11; the dock is the twin's, D97). |

| REQ-SYS-096 | If its navigation restarts on a site that has a map but no RTK coverage, then the robot shall regain a valid position at standstill within the alignment time of 30 s. |
| --- | --- |
| **Rationale** | N-01 and N-09: one consistency fault or one IMU loss in a greenhouse must not leave the robot without a position for good, to be driven out by RC to RTK coverage (R-44, D73). D85 ends the mission on an IMU fault; this lets the operator start it again after the reset. D89 keeps the re-alignment without RTK on card G2-NAV. The alignment time is 30 s, a design target for card G2-NAV: twice T-NAV-01's 15 s limit for an RTK alignment (2.01 s achieved, 2026-11-15), for an alignment on the scan matcher (the sensors and navigation engineer, step 6). Not a safety value. |
| **Source** | N-01, N-09, D73, D85, D89 |
| **Parent** | None |
| **Allocation** | NAV, PER, SNS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (an IMU loss and a filter fault in the greenhouse with no RTK, then navValid again at standstill); today t_nav tNAV13_ImuLoss shows the loss and the FAULT, not the re-alignment |
| **As built** | **Not implemented** |
| **Where** | Shaddad_NavFusion, the navigation mode manager (INIT and ALIGN; sensors spec F6): it aligns on GNSS only, with the scan matcher off in ALIGN |
| **Note** | After a FAULT the filter restarts from P0 and aligns only once its sigmas fall below 0.15 m and 2 deg. Without RTK, and with the scan matcher off in ALIGN, it does not re-align (R-44, D73). Card G2-NAV. |

### 5.5 F-05 Move along the path

**Scope:** Follow the path, turn at the row ends, govern the speed (by mode, implement, slope and the descent cap), control the tracks. (8 requirements, 5 of them safety requirements.)

| REQ-SYS-027 | While it runs a mission, the robot shall follow its planned rows within 0.05 m RMS of the true path, and within 0.15 m at most in a greenhouse (D100) or 0.20 m in a palm grove, row entries included, and within 0.30 m during a GNSS outage of the outage time, back within these values 5 s after the RTK fix returns. |
| --- | --- |
| **Rationale** | N-01: the mower covers its rows; N-09: the robot stays near its path. The tolerance is D100's for the greenhouse (S4: RMS 0.05 m and maximum 0.15 m over the whole row, its entry included, against the true path) and the GNC engineer's for the palm grove (step 5b, section 4: the open navigation maximum of 0.15 m plus the 0.05 m guidance share, with the trunks about 3.5 m from the path); it is the guidance share (0.02 m RMS, 0.05 m at most) and the navigation share of REQ-SYS-020 together, by RSS for the RMS and linearly for the maximum. Rows on a slope meet the row values: T-NAV-11's horizontal RMS of 0.039 m with the 0.02 m guidance share gives 0.044 m (the lead systems engineer's ruling 4 of step 19, withdrawing ruling 9). During a GNSS outage of REQ-SYS-021's outage time, REQ-SYS-020's 0.25 m at the outage's end and the 0.05 m guidance share give 0.30 m, as S7 judges it, and the row values hold again 5 s after the RTK fix returns (the lead systems engineer's point 25). Not a safety value. |
| **Source** | N-01, N-09, D89, D100 |
| **Parent** | None |
| **Allocation** | GNC, MCU, DRV, NAV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (the criteria of D100: the whole row, its entry included, against the true path); S5b (the palm rows); t_gnc_tracking testT1StraightRow, testT2RowEntry, testT3Drift*, testT4HeadlandTurn, testT5SpeedStep once their bench starts the mission (R-30, card T-03) |
| **As built** | **Partly** |
| **Where** | GNC trackers (Pure Pursuit, MPC, Stanley; gnc spec 2.4) and the MCU yaw loop (electronics spec 2.7.6) |
| **Note** | S4.xte_max fails (0.252 m against 0.15 m after each row entry, STATUS 16e); the tracking tests T2 to T5 fail and their bench never starts its mission (R-30); the MPC row tracker oscillates in yaw (STATUS 7). S4's criteria are D100's. The palm rows meet the tolerance (S5b, 2026-11-16: 0.037 m RMS and 0.081 m at most, true). |

| REQ-SYS-028 | When it reaches the end of a row, the robot shall turn into the next row along its planned turn. |
| --- | --- |
| **Rationale** | N-01: the robot turns at the row ends. |
| **Source** | N-01 |
| **Parent** | None |
| **Allocation** | GNC, CMP, MCU, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4; t_gnc_tracking testT4HeadlandTurn |
| **As built** | **Partly** |
| **Where** | GNC TURN segments, Dubins headlands from the mission compiler (gnc spec 2.7) |
| **Note** | testT4HeadlandTurn fails in the baseline; with the mower, S4 holds at the lane-end walls, because the protective field sees them (STATUS 18, R-06). |

| REQ-SYS-029 | While it runs autonomously, the robot shall keep its speed within the autonomous speed limit of 1.2 m/s. |
| --- | --- |
| **Rationale** | SG-01 asks the robot to keep its speed within what its detection zone can stop; the limit of the mode is the top of that envelope (D43). The autonomous speed limit is 1.2 m/s (P.electronics.lim.v_max_auto, params/param_electronics.m:41; P.gnc.spd.v_max = min(2.0, v_max_auto), params/param_gnc.m:31; the GNC engineer, step 5, checked by the MCU firmware engineer). The stop model (the lead systems engineer's ruling 5a of step 19), which every margin on the robot's stop from a speed uses (REQ-SYS-035, -046, -058, -068, -098, -131 and -132 cite it): from a person entering the protective field, REQ-SYS-132's detection time of 0.25 s, one GNC step of 0.05 s and about 0.06 s from the GNC to the ESC (D41), 0.36 s at the speed held; then the stop that the robot makes, by a replay of the MCU's shaper law with the ESC's 40 ms lag (electronics spec 2.7.5): the AUTO stop when the MCU's target steps to zero (P.electronics.shaper.a_dec_auto 1.0 m/s^2, j_max 5 m/s^3, K_v 4 1/s) takes 0.90 m from 1.2 m/s, 0.65 m from 1.0 m/s, 0.22 m from 0.5 m/s and 0.07 m from 0.2 m/s, at rest within 1.9 s, and a controlled stop (D33's law, a_ctrlstop 1.0 m/s^2, j_stop 10 m/s^3) 0.81 m from 1.2 m/s and 1.09 m from 1.40 m/s. The GNC's own check (d_stop_req, gnc spec 2.5) uses t_lat 0.35 s and an ideal stop, 1.14 m from 1.2 m/s. Margin: from 1.2 m/s the stop ends 0.43 + 0.90 = 1.33 m beyond the implement's front, against the protective field's 0.5 + 0.8 x 1.2 = 1.46 m (D72). It is safe because the stop ends 0.13 m inside the zone that the robot can see. |
| **Source** | N-07, SG-01, D43 |
| **Parent** | SG-01 |
| **Allocation** | GNC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5b (the palm patrol at 1.0 m/s); planned: card G1-02 for the limit itself; t_gnc_tracking testEnvelopeVmax once its bench starts the mission (R-30, card T-03) |
| **As built** | **Implemented** |
| **Where** | MCU speed limit in AUTO (lim.v_max_auto) and the GNC command envelope (D43; gnc spec 2.5) |
| **Note** | The bypass speed skips the speed governor (R-41). |

| REQ-SYS-030 | The robot shall keep its speed on a slope, in every mode, within the safe speed for a brakes-locked stop at its measured pitch and fitted implement (the park criterion, CR-003). |
| --- | --- |
| **Rationale** | SG-04 and SG-05: a brakes-locked stop from that speed does not tip the robot (D74, D82). |
| **Source** | N-08, SG-04, D74, D82, D83, CR-003 |
| **Parent** | SG-04 |
| **Allocation** | MCU, GNC, SNS, NAV, STR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2c; S2d; t_mcu_control testTEL54_DescentCapPerImplement, testTEL55_DescentCapTrip; t_gnc_mission testDescentCapRule, testDescentGovernorCap, testDescentLookAhead; t_nav tNAV11_Slope; the analysis behind the table: the structure engineer's descent steps 8, 11 and 13 (work/team/descent/structure/report_step8.md, report_step11.md, report_step13.md) |
| **As built** | **Implemented** |
| **Where** | MCU Descent_cap and Descent_trip (fw_vehicle_ctrl, fw_safety_supervisor; electronics spec 2.7.3; fault bit 16 in ICD section 6); GNC gnc_descentCap and gnc_descentLimit on the navigation pitch (Bus_NavCond.pitch); the table P.structure.descent (structure spec 2.11) |
| **Note** | Card G2-DSC: every brakes-locked stop of S2c and S2d meets the park criterion (STATUS 5.3; the closest, S2c park 2, by 0.056 m/s and 2.1 deg). Those runs (2026-11-16, 01:18 to 01:20) predate D83's table with its 0.75 deg IMU bound; the no-implement reruns on that table meet it too (G2-DSC step 12d; work/team/descent/integration/runs/S2c_none20_step12d.mat and S2d_none20_step12d.mat): S2c 11 of 11 criteria, the closest park by 0.220 m/s in speed, and S2d 9 of 9 with no OVERSPEED before its E-stop. The runs with an implement were not rerun. NavFusion has no level calibration (R-21), so its pitch can differ from the MCU's by the mounting tilt; the MCU's cap binds every command. The table rests on estimated contact data until the slope brake test (T-04, STATUS 35), and on one IMU (R-05, PL b at most). |

| REQ-SYS-031 | If its speed passes the descent cap of its pitch and fitted implement, then the robot shall park on its brakes within the trip reaction time of 0.09 s and stay parked until the operator resets it. |
| --- | --- |
| **Rationale** | SG-05: the robot shall not gain speed downhill; the owner keeps the latch (D74). The trip reaction time is 0.09 s (the functional safety engineer, step 17, Table 1, in place of the 0.08 s of step 13): the trip in its 10 ms sample (bit 16 (b) has no persistence), the park request in the next DRV_CMD frame, then the brake: T19b applies it 70 ms after the park request (60 ms after DISABLING, one 10 ms frame later; P.drive.brake.t_engage 0.06 s), 80 ms with the trip's sample, and 81 ms in the drive engineer's worst case (10 + 10 + 1 + 60 ms; step 9, Table 2). Margin: 9 ms over that worst case; the trip fires on the speed predicted ahead (lim.descent.t_pred), which card G2-MCU raises from 0.08 s to 0.09 s so that the prediction covers the reaction; the 10 ms costs about 0.034 m/s on the 20 deg descent, inside the cap's 0.10 m/s margin (descent.dv), and the table's own margins (v_tip/1.3, with the speed gained before the brakes engage) add to it. It is safe because the brakes hold before the speed reaches the tip speed. Provisional (D96, card G1-04): if the delivered ESC needs its stated share of up to 0.10 s, the value becomes 0.12 s, with t_pred to match and the descent table's margins checked again (cards G2-MCU, G2-DRV). |
| **Source** | N-08, SG-05, D74, D96 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2c; t_mcu_control testTEL55_DescentCapTrip, testTEL58_CclZeroSprayerDescent, testTEL61_HillStartRollback; t_drive testT19b_parkOnDescent (the drive's park) |
| **As built** | **Implemented** |
| **Where** | MCU Descent_trip: OVERSPEED, bit 16, latched; a CTRL_STOP with bit 16 set parks at once (D71, D74) |
| **Note** | S2c passes with the sprayer and with no implement. The reverse trip after an uphill hill start is low (STATUS 30, card G2-MCU). Decided, pending: lim.descent.t_pred 0.09 s on card G2-MCU (0.08 s today). |

| REQ-SYS-032 | While it is on a slope steeper than 10 deg (D53), the robot shall not turn in place. |
| --- | --- |
| **Rationale** | SG-04: a pivot on a slope can slide the robot downhill or tip it (D53). |
| **Source** | N-08, SG-04, D53 |
| **Parent** | SG-04 |
| **Allocation** | GNC, MCU, NAV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testResumeAlignHeading (case d: no pivot at 14 deg); t_structure T7b_turn_on_slopes; t_gnc_tracking testTiltOmegaLimit once its bench starts the mission (R-30) |
| **As built** | **Partly** |
| **Where** | GNC: ALIGN pivots inhibited above 10 deg of pitch or roll, and the turn rate limited by the tilt (D53; gnc spec 2.6) |
| **Note** | The GNC applies the limit in autonomous operation: the ALIGN inhibit covers the start, each segment entry and the resume, but the dock trim at the end of the dock approach pivots without it (R-10). In RC driving the MCU has no turn limit by tilt: the turn rate is limited only by w_max_man, the class-C derate and a_lat_max (electronics spec 2.7.5, 2.7.6); a pivot limit by tilt in RC driving is card G2-MCU (size S). T7b cannot fail, and D53's premise is not reproduced for 316 kg (R-54). |

| REQ-SYS-097 | The robot shall be able to climb and descend the slopes of its envelope on farm soil at a mean ground speed of no less than 0.30 m/s on the 20 deg design slope with no implement or the mower (deck down), and, with the full-tank sprayer on 18.5 deg (CR-001), no less than 0.40 m/s climbing and 0.05 m/s descending. |
| --- | --- |
| **Rationale** | N-08: work on slopes within the envelope (charter section 3, CR-001). S2 checks the climb and the descent of the 20 deg ramp; on a descent the speed stays within the descent cap (REQ-SYS-030). The values come from the design's own limits, as no stakeholder figure exists (the structure engineer, step 10, section 3; L-4, which takes option a for the sprayer's descent): the GNC slope governor allows 0.5 m/s at 20 deg (P.gnc.spd.pitch_v, params/param_gnc.m), and a 20 deg climb on farm soil slips about 9 % (T7: 9.31 %), so the ground speed is about 0.455 m/s; the drive does not limit it (3.43 N m per side, within 6.36 N m continuous; the drive engineer, step 9, Table 2). The descent cap gives no implement and the mower at least 1.10 m/s on the 20 deg ramp, and the full-tank sprayer 0.15 m/s on 18.5 deg in the twin (S2d), which falls to 0.05 m/s when the IMU reads 0.75 deg high within D83's bound: option a holds at that bound. The value is 0.30 m/s with no implement or the mower (the lead systems engineer's point 17, on the structure engineer's evidence), so that it holds where the governor reads the ramp high: at D83's 0.75 deg IMU bound its pitch table gives 0.425 m/s, about 0.385 m/s on farm soil, and descending, the braked settle of +0.78 deg takes it to 0.42 m/s, 0.35 m/s with the IMU bound, 0.05 m/s above the value. Moving the governor's 0.5 m/s breakpoint above the ramp's measured pitch, to reach 0.40 m/s, stays an option for card G2-GNC. Not a safety value. |
| **Source** | N-08, CR-001, D83 |
| **Parent** | None |
| **Allocation** | DRV, STR, MCU, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2 in RC (the climb and the descent of the 20 deg ramp; its decided criterion of 0.30 m/s, with today's v_mean of at least 0.75 m/s, on card G2-INT; the S2 family's criteria are D100's); planned: the autonomous climb and descent under the speed governor, on card G2-INT; S2c, S2d; t_drive testT03_climb |
| **As built** | **Partly** |
| **Where** | the drive and the tracks on the slope, under the speed governor and the descent cap (REQ-SYS-030; drive spec 2.7; structure spec 2.11) |
| **Note** | testS2 fails in the baseline on v_mean: 0.725 m/s at a 0.8 m/s command, which the 9 % slip of a 20 deg climb on farm soil rules out against its 0.75 m/s; since CR-001 the nominal S2 passes its pitch check and S2spray fails it (research D13; the drive engineer, step 9). The results are for the 1 kW default (D95); the 750 W option is selectable, and every result records its motor. The contact data are estimates until the slope brake test (T-04, STATUS 35). Moving the governor's breakpoint is an option for card G2-GNC (P.gnc.spd.pitch_x [0 10 15 20 25] deg, pitch_v [2 1.2 0.8 0.5 0] m/s today). |

| REQ-SYS-116 | The robot shall apply its speed limits and its descent cap to the higher of the two speed measurements of each track (the drive's speed feedback and the encoder-count speed). |
| --- | --- |
| **Rationale** | SG-01, with SG-04 for the descent cap: a derived safety requirement (the lead systems engineer's ruling 2 of step 13). REQ-SYS-081's mismatch limit of 0.3 m/s for 0.2 s leaves a disagreement below 0.3 m/s unseen, and that is larger than the descent cap's 0.10 m/s margin (lim.descent.dv; the MCU firmware engineer, step 4, Table 1). Margin: with the higher measurement, a measurement that reads low by up to the mismatch limit cannot let the true speed pass a limit or the cap, which keeps its 0.10 m/s. It is safe because the limits then bind on the measurement that such an error cannot lower. Both measurements come from the motor encoder that the ESC reads, so the rule covers a frame or computation error, not an encoder fault (hazard log SF-18, card G1-04). |
| **Source** | N-07, N-08, SG-01, SG-04, D74 |
| **Parent** | SG-01 |
| **Allocation** | MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-MCU (a t_mcu_control case: one speed measurement held 0.25 m/s low on a descent; the speed limits, the cap and the trip act on the other) |
| **As built** | **Not implemented** |
| **Where** | none: the MCU's speed limits and descent cap (fw_vehicle_ctrl, fw_safety_supervisor) do not use the higher measurement today |
| **Note** | Decided by the lead systems engineer's ruling 2 (step 13), not built: card G2-MCU. Whether the robot's encoders reach the MCU over CAN or directly is open (hazard log SF-18, card G1-04). |

### 5.6 F-06 Sense the surroundings

**Scope:** Obstacles, and people: in the zone that the robot or its implement is about to occupy, the near zone of the body included, and round the blades. (5 requirements, 4 of them safety requirements.)

| REQ-SYS-033 | While in an autonomous mode or the dock approach, the robot shall detect a person, of the size and posture of its test person (an adult standing, walking at up to 1.6 m/s, kneeling, or lying on the ground wherever the zone lies beyond 1.1 m from its LiDAR, with a body 0.3 m wide; and a child of about six, with a body 0.2 m wide and 1.1 m tall), in the zone that it or its implement is about to occupy. |
| --- | --- |
| **Rationale** | SG-01, its first part; ISO 18497-2:2024 asks for person detection on autonomous agricultural machines (N-15, D103). The greenhouse rule (D79) sets the level at PL d. The test person is the sensors and navigation engineer's (step 6, Table 1), set by risk assessment and not yet confirmed against ISO 18497-2:2024 and ISO 18497-4:2024 (D103): a body 0.3 m wide (a cylinder of 0.3 m diameter) from the ground up to 1.0 m kneeling or bending, or 1.8 m standing, from the hazard log's causes (a worker kneeling, bending or with the back turned) and D79. The walking speed is the functional safety engineer's 1.6 m/s (ISO 13855's walking speed, step 11, section 3), stricter than the 1.0 m/s proposed, by L-7. Margin: 0.3 m is the narrow side of a small adult at the scan band, narrower than the twin's 0.5 m person, and at the field's far end (2.3 m from the LiDAR with the mower at 1.2 m/s in a greenhouse) it still returns about 7 of the 1 deg beams, and at the far corner of D108's field on open ground (about 4.8 m) about 3, against the one kept return that raises the person stop, or two adjacent returns within 3 m, where an isolated return is removed as dust (sensors spec 3.7). The functional safety engineer adds two postures (step 17, Table 1 and section 3, point 5): a person lying on the ground wherever the zone lies beyond 1.1 m from the LiDAR (the lowest beam is 0.54 m above the ground at the LiDAR's 0.5 m minimum range, so a body 0.3 m high is seen beyond about 1.1 m), and a child of about six (HAZ-044), a body 0.2 m wide and 1.1 m tall, which still returns about 5 of the 1 deg beams at the greenhouse field's far corner and 2 at D108's, against the same thresholds; the child is not yet confirmed against ISO 18497-2 (D103). It is safe for the postures named. The project owner accepts the residual risks, with measures (D109): a person lying within 1.1 m of the LiDAR at a start or a resume (D84's near zone, counted as occupied until the operator's resume, REQ-SYS-004's warning and D79), and a person hidden in grass taller than the body (the operator checks the area before a mission, V-06); both go into the hazard log on card G2-FSC. If the child proves infeasible on the delivered LiDAR (card G1-04), it comes back to the owner as a residual risk, with a site rule that no child enters the work area. |
| **Source** | N-07, N-15, SG-01, D79, D84, D103, D109 |
| **Parent** | SG-01 |
| **Allocation** | PER, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5 (a person crossing aisle 4; its pedestrian aligned with the test person on card G2-INT); t_perception tPER02_ProtectiveField, tPER02b_ImplementField; planned: card G1-02 (the 0.3 m test person at the field's corners, lying beyond 1.1 m, and the 0.2 m child) |
| **As built** | **Partly** |
| **Where** | Shaddad_Perception, protectiveStop from the protective field (sensors spec 3.7; D72), read by the GNC as person_near |
| **Note** | The zone in front of the body (0.05 to 0.45 m) is not seen; D84 counts it as occupied until the operator's resume (REQ-SYS-039). The sides are not covered (R-02, STATUS 21, hazard log F-04); the detection is a LiDAR and a companion that are not safety-rated (PL b at most, hazard log E-02); no LiDAR is in the as-built record (card G1-04); docking has never met a dock body (R-10). The perception does not classify people: any return in the protective field counts as a person, so in S4 the static walls give PERSON holds (STATUS 5.2, 5.3; R-06), and under D84 each such hold waits for the operator. The twin's FAST2D LiDAR would see a person lying down at any range, which is optimistic. D109: the two residual cases of the rationale are accepted with their measures and go into the hazard log on card G2-FSC. |

| REQ-SYS-034 | Where an implement is fitted, the robot shall include the zone of the implement actually fitted in the zone where it detects people. |
| --- | --- |
| **Rationale** | SG-01 names the zone that the robot or its implement is about to occupy; the mower deck reaches 1.20 m ahead of the body centre (D72, D81). |
| **Source** | N-07, SG-01, D72, D81 |
| **Parent** | SG-01 |
| **Allocation** | PER, GNC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_perception tPER02b_ImplementField |
| **As built** | **Partly** |
| **Where** | Shaddad_Perception, protXF and protXR from P.structure.impl (D72) |
| **Note** | The field follows the build setting (VC_IMPL), not the implement actually fitted, and nothing compares the two (STATUS 25). The zone 0.55 to 1.20 m ahead, above the deck, is not covered (STATUS 21). The MCU's only part is to report the fitted implement (MCU_STATUS implement_type), which the perception does not read (STATUS 25). |

| REQ-SYS-035 | If its person detection cannot see its zone for longer than the detection time of 0.3 s, because its scans are lost or have stopped updating, then the robot shall stop and hold as for a person. |
| --- | --- |
| **Rationale** | SG-01: a lost detection must not read as a clear zone (D72, R-02). As for a person, the hold ends only on the operator's resume (D84), because the robot cannot tell whether a person is in the zone it cannot see. The detection time is 0.3 s, three scan periods: P.sensors.per.protNStale 2 at 10 Hz plus the scan in progress (the sensors and navigation engineer, step 6); today protectiveStop comes from the third invalid scan and the GNC's SENSOR_LOST at about 0.3 s (D72). Margin, on the stop model of REQ-SYS-029: at 1.2 m/s with the mower, for a person who stood at the field's far end when the scans were lost: 0.3 s blind, one 0.05 s GNC step and about 0.06 s to the ESC (0.49 m), then the 0.90 m AUTO stop, 1.39 m in all, 0.07 m inside the 1.46 m field. It is safe because the robot stops inside the zone that it last saw clear. |
| **Source** | N-07, N-10, SG-01, D72, D84 |
| **Parent** | SG-01 |
| **Allocation** | PER, SNS, GNC, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_perception tPER07_ScanLost; t_gnc_mission testScanLostOnRow, testScanLostInHold, testScanLostFsm |
| **As built** | **Partly** |
| **Where** | Perception, protectiveStop after protNStale invalid scans; the GNC SENSOR_LOST hold (D72); the MCU's CTRL_STOP and park on the GNC's stop request, or the ESCs' park at zero command (D66, D71) |
| **Note** | The tests pass. After a lost scan the GNC resumes by itself on valid, clear scans (H1), not on the operator's command (D84, card G2-GNC, size M with D92 and D93). On the robot, a frozen but re-published LiDAR cloud stays valid (STATUS 28); the simulated LiDAR can only black out. |

| REQ-SYS-036 | While it runs autonomously, the robot shall detect a static obstacle of 0.30 m or larger in width and height in its path, at 3.0 m or more ahead of its front. |
| --- | --- |
| **Rationale** | N-01 and N-03: rows and transits have posts, boxes and trees that the robot must not hit. The smallest obstacle is 0.30 m in width and height, from T10's piles (radius 0.15 m) and tPER01's box (0.3 x 0.3 x 0.5 m); the robot holds at 1.5 m (P.gnc.obs.d_stop) and needs the obstacle seen at 3.0 m for full speed (P.gnc.spd.obs_d, obs_v; the GNC engineer, step 5). With the deck's 0.7 m reach ahead of the body's front the hold leaves 0.8 m to the obstacle, more than D100's 0.5 m. The 0.30 m height has no test yet (tPER01's box is 0.5 m tall): a planned case on card G1-02, with the LiDAR's mounting height (card G1-04). Not a safety value. |
| **Source** | N-01, N-03, D100 |
| **Parent** | None |
| **Allocation** | PER, SNS, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5 and S5b (the criteria of D100: the slow-down from the obstacle distance, the hold as in T6, at least 0.5 m to the box); t_perception tPER01_Obstacle; t_gnc_mission testT6StaticBox; planned: card G1-02 (an obstacle 0.30 m wide and 0.30 m tall) |
| **As built** | **Partly** |
| **Where** | Perception obstacles and occupancy; the GNC obstacle corridor (gnc spec 2.6) |
| **Note** | S5b passes 13 of 13 (STATUS 5.3). S5 cannot pass by design of its criteria (R-63). The obstacle stop is off in a bypass, its rejoin and the dock approach (gnc_missionFSM.m:158-159); in the dock approach only a person or a lost scan stops the robot (R-10, card G2-GNC). |

| REQ-SYS-098 | While its blades run, in any mode, the robot shall detect a person who comes within the blades' protective zone of REQ-SYS-046 on any side and start the blade stop within 0.35 s of the person's entry. |
| --- | --- |
| **Rationale** | SG-09: the blades stop for a person near. REQ-SYS-046 stops them when a person is detected, but REQ-SYS-033 asks for detection only in the zone the robot is about to occupy, in the autonomous modes: a person at the side of the deck, or near it in RC driving, is not seen (HAZ-006, HAZ-020; hazard log E-06). The protective zone is REQ-SYS-046's: 5.0 m from the outer edge of the deck housing on every side and, while the robot moves, 6.5 m ahead and on the side toward which its deck sweeps, in every mode, with RC driving held at 1.2 m/s while the blades run (REQ-SYS-210), set by risk assessment (D103) and not yet confirmed against the standards (the functional safety engineer, step 11, Table 1, and step 19). The text binds the 0.35 s from a person entering the zone to the start of the blade stop, on which the zone rests, as REQ-SYS-132 binds the person field's 0.25 s (the lead systems engineer's point 8); today's chain through the GNC and the next IMPL_CMD frame takes up to 0.40 s, which does not meet it (decided, pending on card G2-FSC). The margin and why it is safe are REQ-SYS-046's. |
| **Source** | N-07, N-15, SG-09, D103 |
| **Parent** | SG-09 |
| **Allocation** | PER, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 (a person beside the deck while mowing; a person near the deck in RC driving; the blade stop started within 0.35 s of the entry) |
| **As built** | **Not implemented** |
| **Where** | none: the protective fields look to the front and the rear, and the person stop runs only in the autonomous modes (sensors spec 3.7; D72) |
| **Note** | Hazard log HAZ-006 and HAZ-020; E-06 (class 1), which the safety concept answers (card G2-FSC). No LiDAR is in the as-built record (card G1-04). The chain to the blade stop takes up to 0.40 s today, against the 0.35 s (card G2-FSC). |

### 5.7 F-07 Hold and avoid

**Scope:** Stop and hold for a person, go round a static obstacle, resume when the zone has been seen free. (6 requirements, 4 of them safety requirements.)

| REQ-SYS-037 | When it detects a person in the zone it is about to occupy, the robot shall stop before contact and hold on its brakes. |
| --- | --- |
| **Rationale** | SG-01: stop before contact; the safe state is standstill on the brakes with the implement off. ISO 18497 (N-15). |
| **Source** | N-07, N-15, SG-01 |
| **Parent** | SG-01 |
| **Allocation** | GNC, MCU, DRV, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5 (at least 1.0 m to a pedestrian who meets the robot, with D84's resume: the criteria of D100); t_gnc_mission testPersonHoldMowerOff; t_perception tPER02_ProtectiveField (a person crossing at 1 m/s) |
| **As built** | **Partly** |
| **Where** | GNC OBSTACLE_HOLD (T5, PERSON) with a standstill command; the ESCs park after 2 s at zero command (D66, D67) |
| **Note** | The person stop is software (PL b at most, hazard log E-02); the near zone and the sides are not covered (R-02), and the hold ends by itself, not on the operator's resume of D84 (REQ-SYS-039). S5 fails by its criteria (R-63); S5b passes. |

| REQ-SYS-038 | The robot shall keep its speed within what its person detection zone can stop before contact. |
| --- | --- |
| **Rationale** | SG-01, its last part: the speed and the detection zone go together. |
| **Source** | N-07, SG-01 |
| **Parent** | SG-01 |
| **Allocation** | GNC, MCU, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: planned: card G1-02 (the stop distance against the field length 0.5 + 0.8 \|v\| m and the hold at 1.5 m, gnc spec 2.5) |
| **As built** | **Partly** |
| **Where** | The protective field length 0.5 + 0.8 \|v\| m (D72); the GNC obstacle speed table and d_stop_req (gnc spec 2.5) |
| **Note** | No analysis shows the stop inside the field for every speed and implement; the zone in front of the body is not covered (R-02). |

| REQ-SYS-039 | When it has seen a person in its zone, or could not see its zone, during a hold, the robot shall start to move only on the operator's deliberate resume command (D84). |
| --- | --- |
| **Rationale** | SG-02 for the motion, by D84: the near zone of the body (about 0.05 to 0.45 m in front, which the LiDAR cannot see) counts as occupied until the operator resumes. The implement's restart is REQ-SYS-049. |
| **Source** | N-05, N-07, SG-02, D84, D99 |
| **Parent** | SG-02 |
| **Allocation** | GNC, CMP, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 (the S5 variant with a person in the near zone, D84) |
| **As built** | **Not implemented** |
| **Where** | GNC H1: the hold ends by itself after 2 s clear on valid scans (gnc spec 4.1) |
| **Note** | The robot resumes without the operator, on a field that does not cover the zone in front of the body (R-02). Card G2-GNC changes the hold logic to D84 (with D92 and D93, size M). D99 also bars an automatic bypass after a hold in which a person was seen or the scans were lost (REQ-SYS-040). |

| REQ-SYS-040 | When a static obstacle blocks its lane for longer than 10 s, the robot shall go round it where its plan allows a bypass, no person was seen nor scan lost in the hold, and the bypass checks cover its implement's footprint (D99). |
| --- | --- |
| **Rationale** | N-01: an obstacle should not end the mission. The detour keeps to the robot's own lane, the safety bound of REQ-SYS-099 (SG-01, HAZ-007). The hold time is 10 s (P.gnc.obs.t_bypass, params/param_gnc.m:103), and D99 adds its gates (the GNC engineer, step 5). Not a safety value. |
| **Source** | N-01, N-07, D99 |
| **Parent** | None |
| **Allocation** | GNC, PER |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT7BypassNoTeb, testT7BypassTeb; t_perception tPER04_LocalMap, tSectorsAndObstacles (tPER05_TebVfh fails here only because Windows Code Integrity blocks its MEX, in the baseline) |
| **As built** | **Partly** |
| **Where** | GNC H2, the bypass with VFH or TEB (VC_TEB) inside the lane mask (gnc spec 4.1) |
| **Note** | The default bypass method (TEB) fails T7, while VFH passes (R-24); D99 makes VFH the default and TEB a development option kept out of system runs until it passes T7, while the parameters still select TEB (P.gnc.variant.teb = 1) until card G2-GNC; the bypass speed skips the governor (R-41); the goals do not match the lane (R-49). No TEB bypass with an implement that may not reverse (D74, card G2-DSC); the VFH bypass, forward only, stays. A bypass starts only from a ROW of a mission and never with a person in the field (H2). The perception cannot tell a standing person from a static obstacle, so a person who stands still in the lane beyond the protective field is an obstacle to the bypass (HAZ-007, for the safety concept). Under D99 no automatic bypass starts with the mower, or after a hold in which a person was seen or the scans were lost, until card G2-GNC adds the gates; a blocked lane then holds, pauses and alerts the operator (REQ-SYS-041). |

| REQ-SYS-041 | If an obstacle hold lasts longer than the hold limit of 120 s, then the robot shall pause the mission and alert the operator. |
| --- | --- |
| **Rationale** | N-05: the operator must learn of a hold that the robot cannot clear by itself. The hold limit is 120 s (P.gnc.obs.t_hold_max, params/param_gnc.m:104; T11's hold_timeout check; the GNC engineer, step 5). With D99 the mower never bypasses, so a blocked lane in a greenhouse waits 120 s before the alert; a shorter limit is the owner's choice. |
| **Source** | N-05, D99 |
| **Parent** | None |
| **Allocation** | GNC, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (no test checks the hold time-out H3) |
| **As built** | **Implemented** |
| **Where** | GNC H3 into PAUSE with OBST_TIMEOUT, or SENSOR_LOST while the scan is lost (gnc spec 4.1) |
| **Note** | No dedicated test. The alert reaches only the GNC status, as no operator console is defined for the robot (REQ-SYS-082). |

| REQ-SYS-099 | While it goes round an obstacle, the robot shall keep itself and its implement within its own lane. |
| --- | --- |
| **Rationale** | SG-01 (HAZ-007): a detour must not enter a neighbouring lane, where people may work (D79). REQ-SYS-040 keeps the bypass itself, as a mission requirement. |
| **Source** | N-07, SG-01, D79, D99 |
| **Parent** | SG-01 |
| **Allocation** | GNC, PER, NAV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT7BypassNoTeb, testT7BypassTeb (the lane mask); planned: card G1-02 with the implement's footprint (D87) |
| **As built** | **Partly** |
| **Where** | GNC H2, the bypass inside the lane mask (gnc spec 4.1) |
| **Note** | VFH is the default bypass by D99 and TEB a development option until it passes T7 (R-24), while the parameters still select TEB (P.gnc.variant.teb = 1) until card G2-GNC; the bypass goals do not match the lane (R-49); the bypass checks use the body only, and the footprint with the implement is card G2-GNC (D87, R-06). Under D99 no automatic bypass starts with an implement whose footprint the checks do not cover, the mower included, until card G2-GNC. |

### 5.8 F-08 Work the implement

**Scope:** Know which implement is fitted and its limits; raise and lower the deck; start and stop the blades with their interlocks; the implement in a hold. (15 requirements, 14 of them safety requirements.)

| REQ-SYS-042 | The robot shall apply the limits of the implement that is actually fitted. |
| --- | --- |
| **Rationale** | HAZ-039: the descent limits of the wrong implement can tip the robot (SG-04, D74). |
| **Source** | N-08, N-12, SG-04, D74 |
| **Parent** | SG-04 |
| **Allocation** | MCU, ELC, COM, GNC, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL54_DescentCapPerImplement |
| **As built** | **Partly** |
| **Where** | MCU: the DI IMPL_ID coding plug and IMPL_FB, an unknown implement taking the smallest entry of all rows (D74); GNC: the plan's implement_type (gnc spec 2.5) |
| **Note** | The MCU reads the fitted implement; the GNC takes the plan's and the perception the build setting, and nothing compares them (STATUS 25). The MCU's cap binds every command, so the gap is in the GNC's limits and the protective field. A plug that reads 0 while an implement is fitted is not detected unless the implement reports on IMPL_FB; the cap then takes the no-implement row (HAZ-039). The MCU reports the fitted implement in MCU_STATUS (implement_type), which the GNC and the perception could take instead of the plan's and the build setting (STATUS 25). |

| REQ-SYS-043 | While the mower is fitted and the robot runs autonomously, the robot shall move only with the deck down. |
| --- | --- |
| **Rationale** | D81: the deck stays down whenever the robot moves; a raised deck makes the robot nose-heavy (HAZ-018, SG-04). |
| **Source** | N-01, N-08, SG-04, D81 |
| **Parent** | SG-04 |
| **Allocation** | GNC, MCU, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testMowerDeckDownWhenMoving; S4_100 (IMPLEMENT_NOT_READY from 15.20 to 21.55 s) |
| **As built** | **Partly** |
| **Where** | GNC deck command and the deck wait (fsm.t_deck 6.35 s, IMPLEMENT_NOT_READY; gnc spec 2.9) |
| **Note** | The deck counts as down after a fixed wait, and its height is an estimate (STATUS 37); the mower is not in the as-built record (card G1-04). |

| REQ-SYS-044 | If the mower deck is reported raised while the robot moves, then the robot shall keep its speed within the raised-deck limits of D74. |
| --- | --- |
| **Rationale** | D81: a raised deck makes the robot nose-heavy, so the raised rows apply as a backstop (SG-04). |
| **Source** | N-08, SG-04, D74, D81 |
| **Parent** | SG-04 |
| **Allocation** | MCU, ELC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL60_MowerDeckRaised |
| **As built** | **Partly** |
| **Where** | MCU Descent_cap, the raised rows v_fwd_raised and v_rev_raised from the lift potentiometer (lim.descent.lift_raised; D74) |
| **Note** | The test passes. The raised threshold is an estimate (0.09 m [EST]), and a stuck potentiometer that reads low hides a raised deck (STATUS 37). In the twin this backstop does not work yet: the structure plant and the MCU read the lift position in opposite senses, so the raised rows apply with the deck low and not with it raised (structure spec 5.4). The plant follows the sensor and the MCU (a height fraction, 0 at the bottom and 1 at the top), so only the structure twin changes, on card G2-STR, with the ICD row regenerated then and the system check on G2-INT; the real potentiometer's direction is set in the MCU's calibration once card G1-04 gives it (the lead systems engineer's point 22). |

| REQ-SYS-045 | The robot shall run its blades only in RC driving with the implement switched on, or on the rows and turns of a mowing mission. |
| --- | --- |
| **Rationale** | SG-09: the blades stop outside the working modes; the mower standard that card G2-FSC chooses (N-15, D103). |
| **Source** | N-01, N-15, SG-09, D103 |
| **Parent** | SG-09 |
| **Allocation** | GNC, MCU, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testImplementAuxUnit; t_power_plant mower_T14, ecu_can_timeout, impl_codec_round_trip; planned: card G1-02 for the GNC's blade states |
| **As built** | **Hardware** |
| **Where** | GNC implement manager, blades on ROW and TURN only (gnc spec 2.9); MCU implement output: the rotor in MANUAL or AUTO only, with the implement switch on and no implement fault, in AUTO on the GNC's command, and off during the AUTO_START warning (electronics spec 2.7.12); the implement ECU runs the blades only on an enabled IMPL_CMD with Safety_OK, and sets its reference to 0 when the commands have stopped for 0.2 s (impl_ecu; power spec 2.7) |
| **Note** | Implemented in the twin. The mower (blades, guard, motor and brake) is not in the as-built record (card G1-04). |

| REQ-SYS-046 | When it detects a person within the blades' protective zone, 5.0 m from the outer edge of its deck housing on every side, and 6.5 m ahead of it and on the side toward which its deck sweeps while it moves, the robot shall stop its blades within the run-down time of REQ-SYS-048. |
| --- | --- |
| **Rationale** | SG-09: the blades stop for a person near. The stop starts at once, and the blades come to rest within the run-down time of REQ-SYS-048; the detection round the deck is REQ-SYS-098, which binds the 0.35 s to the start of the blade stop, and the motion stop that the zone ahead assumes is REQ-SYS-209. The protective zone is set by risk assessment (D103) by the approach-speed method of ISO 13855:2024: S = K (t_d + t_s) + C on the sides, and ahead plus the robot's travel until it stands, on the stop model of REQ-SYS-029 (the lead systems engineer's ruling 5a of step 19 and point 9): t_d = 0.35 s from a person entering the zone to the start of the blade stop (REQ-SYS-098; the sensors and navigation engineer's bound), K = 1.6 m/s (walking), t_s = 2.0 s (REQ-SYS-048), C = 1.2 m (the reach allowance for a detection plane at the ground), and the robot's travel of 0.36 s at 1.2 m/s (0.43 m) before the 0.90 m AUTO stop. Needed: 4.96 m on the sides and the rear, and 6.29 m ahead at 1.2 m/s. The deck's outer corner moves at up to about 1.1 m/s in the plan's turns, inside the 1.2 m/s case, so the side toward which the deck sweeps takes the 6.5 m (the functional safety engineer, step 19, section 3, Table 1). In RC driving the speed is held at 1.2 m/s while the blades run (REQ-SYS-210); at 1.40 m/s the zone ahead would need about 6.65 m. K and C are the 2010 edition's values; not yet confirmed against ISO 13855:2024, ISO 13857:2019 or the mower standard. Margin: 0.04 m on the sides and 0.21 m ahead; every 0.1 s of detection beyond 0.35 s costs 0.16 m on the sides and about 0.28 m ahead. Today's chain through the GNC and IMPL_CMD (0.25 s, a 0.05 s GNC step, 0.05 s to the MCU and up to 0.05 s for the next IMPL_CMD frame, 0.40 s) does not meet the 0.35 s (5.04 m would be needed; decided, pending on card G2-FSC). It is safe because a person who walks into the zone at 1.6 m/s reaches the deck only after the blades rest, and ahead only after the robot stands. |
| **Source** | N-07, SG-09, D72, D103 |
| **Parent** | SG-09 |
| **Allocation** | GNC, MCU, PER, PWR, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testPersonHoldMowerOff (the person in the protective field); planned: card G1-02 with G2-FSC (a person entering the 5.0 m zone at the side, and the 6.5 m zone ahead and on the sweep side while mowing, in autonomous work and in RC driving: the blades at rest within 2.0 s by REQ-SYS-048) |
| **As built** | **Partly** |
| **Where** | GNC implement manager, the immediate stop on person_near (gnc spec 2.9; D72); the implement ECU's braked stop (power spec 2.7) |
| **Note** | The test passes. As built, the person is the one in the protective field (person_near is the perception's protectiveStop, a lost scan included), which does not watch the sides of the deck (REQ-SYS-098); the person path of the blade stop is software (PL b at most; hazard log SG-09, E-06). Decided, pending: the zone's detection is card G2-FSC's (REQ-SYS-098), with the motion stop of REQ-SYS-209 and the RC speed of REQ-SYS-210; today only a person in the protective field stops the blades, and the robot only in the autonomous modes. The functional safety engineer confirmed the zone on the one stop model (step 19). Built on today's perception, which counts any return as a person, the zone would stop the blades nearly all the time in a greenhouse: it needs a detection that tells people from the site's fixed objects, or a blade brake (RID-G1-01-SAF-02, card G2-FSC). |

| REQ-SYS-047 | While it holds, pauses, is in a fault, is in transit, has stopped for its tilt or has its deck raised, the robot shall keep its blades stopped. |
| --- | --- |
| **Rationale** | SG-09: the blades stop on a hold, a tip, a raised deck and outside the working modes. |
| **Source** | N-07, SG-09 |
| **Parent** | SG-09 |
| **Allocation** | GNC, MCU, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testPersonHoldMowerOff (H4); planned: card G1-02 for the tilt stop and the raised deck |
| **As built** | **Partly** |
| **Where** | GNC implement manager: blades off in TRANSIT, PAUSE, FAULT and MANUAL, and after 3 s in a hold (H4; gnc spec 2.9); the MCU's rotor gate: MANUAL or AUTO only (electronics spec 2.7.12) |
| **Note** | In a hold without a person the blades run on for 3 s (H4). A tilt stop stops the blades through the MCU's CTRL_STOP, and in a mission the GNC's tilt fault (G1) turns the implement off; no test shows either yet (planned: card G1-02). In RC driving the blades follow the implement switch (REQ-SYS-045). Nothing stops the blades on a raised deck (STATUS 37): a rotor interlock on the lift potentiometer is S work for card G2-MCU, or for the GNC on card G2-GNC. |

| REQ-SYS-048 | When the robot stops its blades, the robot shall bring them to rest within 2.0 s. |
| --- | --- |
| **Rationale** | SG-09: the blades stop within a run-down time. 2.0 s to rest is set by risk assessment as the stricter of the candidates (D103): 2 s for the blade stop of robotic mowers on tilt and lift (EN 50636-2-107, the European form of IEC 60335-2-107, from a secondary summary), against 3 s to below 10 % (ISO 5395, which does not cover this mower, and the power spec's target); not yet confirmed against the mower standard that card G2-FSC chooses (the functional safety engineer, step 11, Table 1). To rest, not to 10 %: a blade at 10 % of its 76 m/s tip speed still cuts (P.structure.impl.mower.w_nom 293.2 rad/s, R_blade 0.26 m). Margin: the protective zone of REQ-SYS-046 is sized with the full 2.0 s, and every faster stop adds margin (0.5 s less saves 0.8 m of zone). It is safe because a person walking in from the zone's edge reaches the deck only after the blades rest; a person already beside the deck is protected by the guard (ISO 13857) and the information for use, not by the run-down. |
| **Source** | N-15, SG-09, D86, D103 |
| **Parent** | SG-09 |
| **Allocation** | PWR, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_power_plant mower_T14 (after an E-stop); planned: card G1-02 for the run-down after a stop command; the unbraked run-down by analysis (the structure engineer, step 10: about 112 s to 10 % without the implement drive's braking) |
| **As built** | **Partly** |
| **Where** | Power_Plant implement loads, the mower drive and its speed PI, braked to a stop by the implement ECU (power spec 2.7, which specifies below 10 % of speed within 3 s) |
| **Note** | The twin brakes the blades with its implement drive; mower_T14 passes (below 10 % within 3.0 s after an E-stop, braked while K1 stays closed for 1.5 s). Not checked: the run-down after a stop command, and the run-down without the 48 V bus. The mower is not in the as-built record (card G1-04); the value is set by risk assessment (D103), and the mower standard is chosen on card G2-FSC; D86 leaves the blades' run-down to the safety concept (G2-FSC, E-06). Not met today: the twin brakes the blades to below 10 % within 3.0 s, then lets them coast once K1 opens at +1.5 s, so it meets neither the 2.0 s nor the rest; without the implement drive's braking the blades take about 112 s to reach 10 % (the structure engineer's finding 8.1). A blade brake is the likely answer (card G2-FSC, E-06, with G2-PWR), and the power spec's 3 s to below 10 % is to align (card G2-PWR). |

| REQ-SYS-049 | When it has seen a person in its zone, or could not see its zone, during a hold, the robot shall keep its implement off until the operator's deliberate resume command. |
| --- | --- |
| **Rationale** | SG-02 and SG-09: after a person stop the implement restarts only on the operator's resume (D84); until then it stays off for the rest of the hold, as the owner chose (D72, review decision 3). |
| **Source** | N-07, SG-02, SG-09, D72, D84 |
| **Parent** | SG-02 |
| **Allocation** | GNC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testPersonHoldMowerOff, testPersonHoldTillerOff (the latch); planned: card G1-02 for the operator's resume (D84) |
| **As built** | **Partly** |
| **Where** | GNC implement latch, impl_allow held false for the rest of the hold (D72; gnc spec 2.9) |
| **Note** | The latch tests pass, but the hold ends by itself (H1), and the implement then restarts without the operator's command (D84, card G2-GNC, size M with D92 and D93). |

| REQ-SYS-050 | When its blades have stopped for an emergency stop, a person or a zone it could not see, any other hold or pause, a tilt stop, a fault or a raised deck, the robot shall restart them only on the operator's deliberate command (D92). |
| --- | --- |
| **Rationale** | SG-09: the blades shall not restart without a deliberate command. D92: during a hold a person can reach the deck unseen, since the near zone and the deck's sides are not watched, so after any protective stop the restart is the operator's. The planned starts within a mission, on the rows and turns after a transit, stay automatic: the operator's mission start covers them (REQ-SYS-045). |
| **Source** | N-11, SG-09, D84, D92 |
| **Parent** | SG-09 |
| **Allocation** | GNC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (a restart after an E-stop, a person, an obstacle hold, a pause, a tilt stop, a fault and a raised deck, in a mission and in RC driving) |
| **As built** | **Partly** |
| **Where** | RC driving: the implement switch, the rotor running in MANUAL whenever the switch is on (electronics spec 2.7.12); a mission: the GNC restarts the blades on ROW or TURN after H1 (gnc spec 2.9) |
| **Note** | In a mission the blades restart without an operator command when a hold clears (H1) or when P2 resumes after a navigation pause, and after a hold with a person only once the latch ends (D72), not on the operator's resume (D84). Card G2-GNC makes it D92 (with D84 and D93, size M), and chooses between waiting for the operator and moving on with the blades off. In RC driving the blades restart with the arm command if the implement switch was left on; requiring an off-to-on edge of the switch after every stop would make it a deliberate command (S work, card G2-MCU). The implement ECU restarts the blades by itself when its command frames return with the enable still set (REQ-SYS-101). |

| REQ-SYS-051 | Where the tiller is fitted, the robot shall not reverse. |
| --- | --- |
| **Rationale** | The owner's decision "No reversing with tiller" (D74); the raised tiller can tip the robot backward (charter section 3). |
| **Source** | N-08, SG-04, D74 |
| **Parent** | SG-04 |
| **Allocation** | MCU, GNC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL59_TillerNoReverse; t_gnc_mission testTillerNoReverse |
| **As built** | **Implemented** |
| **Where** | MCU Descent_cap, the tiller's reverse cap 0 at every pitch; GNC, no TEB bypass with an implement that may not reverse (D74) |
| **Note** | The tests pass. The tiller is not used until real tiller data exist (charter section 3, STATUS 36). |

| REQ-SYS-100 | If it cannot identify the fitted implement, then the robot shall apply the most restrictive limits of all implements. |
| --- | --- |
| **Rationale** | HAZ-039: the descent limits of the wrong implement can tip the robot (SG-04, D74), so an implement the robot cannot identify counts as the most restrictive one. |
| **Source** | N-08, N-12, SG-04, D74 |
| **Parent** | SG-04 |
| **Allocation** | MCU, ELC, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL54_DescentCapPerImplement; planned: card G1-02 for an unknown implement |
| **As built** | **Implemented** |
| **Where** | MCU: the DI IMPL_ID coding plug and IMPL_FB; an unknown implement takes the smallest entry of all rows of the descent table (D74) |
| **Note** | The MCU's cap binds every command. A plug that reads 0 (no implement) while an implement is fitted reads as identified, not as unknown (REQ-SYS-042). |

| REQ-SYS-101 | If its implement reports a fault, its commands stop arriving at the implement for 0.2 s, or its reports stop arriving for 0.4 s, then the robot shall stop its implement and keep it stopped until the operator's deliberate restart command (D92). |
| --- | --- |
| **Rationale** | SG-09: the blades stop outside the working modes and do not restart without a deliberate command; a faulty implement drive can run its blades or rotor out of control. SG-08: the implement does not start, without a deliberate command, when a link returns (D92). The implement ECU stops the blades when IMPL_CMD has stopped for P.power.impl.ecu.t_can_timeout 0.2 s (the BMS and power engineer's spec 2.7), and the MCU when IMPL_FB is older than P.electronics.safety.impl_timeout 0.2 s for bit 28's own persistence of 0.2 s (P.electronics.safety.t_persist), about 0.4 s, then sends the enable off in the next IMPL_CMD frame (the MCU firmware engineer, step 4; the register's re-check, step 14b). Margin: 0.2 s is 4 frame periods of 50 ms (20 Hz); the stop starts within about 0.2 s of a lost IMPL_CMD and 0.45 s of a lost IMPL_FB, small against the blades' run-down of REQ-SYS-048. It is safe because the blades start to stop within 0.45 s of a lost link. The MCU's latch (bit 28) covers a lost IMPL_FB; a lost IMPL_CMD alone leaves the enable set, so the ECU would restart the blades when the frames return, and card G2-PWR adds an ECU latch, or the MCU drops the enable. |
| **Source** | N-07, N-10, N-11, N-12, SG-08, SG-09, D92 |
| **Parent** | SG-09 |
| **Allocation** | MCU, PWR, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testImplementAuxUnit (the IMPL_FAULT case: the rotor stops), testTEL13_FaultBits (bit 28); t_power_plant ecu_can_timeout (the reference 0 at 0.2 s after the last command frame); planned: card G1-02 for the return of the link |
| **As built** | **Partly** |
| **Where** | MCU fault bit 28 IMPL_FAULT: a fitted implement whose IMPL_FB reports a fault or is older than 0.2 s, set after bit 28's 0.2 s persistence, class C (implement off), latched (ICD section 6; electronics spec 2.7.12); the implement ECU: the speed reference 0 when the IMPL_CMD frames have stopped for t_can_timeout, then a braked stop (impl_ecu; power spec 2.7) |
| **Note** | The tests pass in the twin. The implement ECU has no latch: when the command frames come back with the enable still set, it restarts the blades without a new command; whether the MCU withdraws the enable first is to be checked (card G2-PWR, with the MCU firmware engineer). The mower, its drive and its frames are not in the as-built record (card G1-04). |

| REQ-SYS-102 | If its implement does not report ready within the ready time of 10 s at the start of a work segment, then the robot shall pause the mission. |
| --- | --- |
| **Rationale** | N-01: the robot works a row only with its implement working, so it waits at the start of the segment (for the mower, the deck wait of REQ-SYS-043). N-05: an implement that never gets ready must reach the operator as a pause with its reason (REQ-SYS-082), not as a robot that waits for ever. The ready time is 10 s (P.gnc.fsm.t_impl_ready, params/param_gnc.m:123; the bench's spin-up of 2 s, tb.t_spin; the GNC engineer, step 5), to recheck when the mower's spin-up is in the as-built record (G1-04). |
| **Source** | N-01, N-05 |
| **Parent** | None |
| **Allocation** | GNC, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (testMowerDeckDownWhenMoving checks the wait, not the time-out) |
| **As built** | **Implemented** |
| **Where** | GNC mission manager: on a ROW or TURN with an implement, no motion until mcu.implement_ready, then PAUSE with IMPLEMENT_NOT_READY after fsm.t_impl_ready (gnc spec 2.9) |
| **Note** | The MCU's implement_ready stands for the implement's own report; the mower is not in the as-built record (card G1-04). |

| REQ-SYS-209 | When it detects a person within the blades' protective zone while it moves with its blades running, in any mode, the robot shall make a controlled stop and hold until the operator's deliberate resume (D84). |
| --- | --- |
| **Rationale** | SG-09, with SG-01: the zone ahead of REQ-SYS-046 is sized with the robot's own travel until it stands, on the stop model of REQ-SYS-029, so the robot makes that stop in every mode while its blades run, as its own requirement so that each stays singular (the functional safety engineer, step 19, section 3; the lead systems engineer's point 7). Margin: REQ-SYS-046's: from 1.2 m/s the robot travels 0.43 m before the 0.90 m AUTO stop, and the zone ahead needs 6.29 m against its 6.5 m (0.21 m), with RC driving held at 1.2 m/s while the blades run (REQ-SYS-210). It is safe because a person who walks into the zone ahead at 1.6 m/s reaches the deck only after the robot stands; the hold ends only on the operator's deliberate resume (D84), whose form in RC driving is card G2-FSC's design, for example the arm switch cycled with the sticks at neutral. |
| **Source** | N-07, SG-01, SG-09, D84, D103 |
| **Parent** | SG-09 |
| **Allocation** | PER, SNS, GNC, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 with G2-FSC (a person entering the 6.5 m zone ahead while mowing, in autonomous work and in RC driving: the controlled stop, the robot standing before the person reaches the deck, and the hold until the deliberate resume) |
| **As built** | **Not implemented** |
| **Where** | none: the blade zone is not detected (REQ-SYS-098), and the person stop runs only in the autonomous modes, on the protective field (sensors spec 3.7; D72) |
| **Note** | Decided, pending: the motion stop on a person in the blade zone, in every mode, on card G2-FSC, with cards G2-MCU and G2-GNC; the form of the resume in RC driving is G2-FSC's design. |

| REQ-SYS-210 | While its blades run in RC driving, the robot shall keep its speed within 1.2 m/s. |
| --- | --- |
| **Rationale** | SG-09: one zone of 6.5 m ahead (REQ-SYS-046) serves both modes only while RC driving is held at 1.2 m/s with the blades running; at the 1.40 m/s RC limit the zone ahead would need about 6.65 m (the functional safety engineer, step 19, section 3, Table 1; the lead systems engineer's point 6). 1.2 m/s lies above every work speed of the plan (0.5 to 1.0 m/s, REQ-SYS-125), so it costs only the RC limit's last 0.2 m/s during mowing; REQ-SYS-010's 1.40 m/s and REQ-SYS-119's transit speed stand, with the blades stopped. Margin: at 1.2 m/s the zone ahead needs 6.29 m against REQ-SYS-046's 6.5 m (0.21 m). It is safe because the robot then stands inside the zone before a person who walks in reaches the deck. The project owner approved this cap with part 2 on 2026-11-18 (NASL-SHD-REQ-008 v2.0), so the 7.0 m zone ahead in RC driving is not needed. |
| **Source** | N-04, N-07, SG-09, D103 |
| **Parent** | SG-09 |
| **Allocation** | MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-MCU (a t_mcu_control case: full stick in MANUAL with the mower running, the speed held within 1.2 m/s; with the blades stopped, 1.40 m/s) |
| **As built** | **Not implemented** |
| **Where** | none: the MCU's RC speed limit of 1.40 m/s (lim.v_max_man) applies whether the blades run or not (electronics spec 2.7.5) |
| **Note** | Decided, pending: the twin changes on card G2-MCU (P.electronics.lim.v_max_man 1.40 m/s today, with no limit for a running implement). |

### 5.9 F-09 Manage the energy

**Scope:** Monitor and protect the pack; switch the high voltage on and off (precharge); feed the low-voltage rails; derate on warnings; return to the dock on low energy. (9 requirements, 5 of them safety requirements.)

| REQ-SYS-052 | If a cell passes its voltage or temperature limit, or the pack current passes its limit, then the robot shall disconnect its pack. |
| --- | --- |
| **Rationale** | SG-10: the pack shall not overheat, overcharge or short. |
| **Source** | N-13, SG-10 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_pack_bms overcurrent_T07, ov_T08, ov_forced_charge; t_power_plant short_T06 (the branch fuse clears a short); planned: card G1-02 for the undervoltage and the over- and under-temperature cases |
| **As built** | **Hardware** |
| **Where** | The twin's Shaddad_BMS protection and its BMS_ModeManager (power spec 2.9, 4.1) |
| **Note** | The tests pass on the twin's BMS. The robot has a bought 13S BMS whose protections, settings and frames are unknown (R-15, hazard log SF-17); the delivered BMS is the baseline, provisionally (D96; card G1-04). The undervoltage and the over- and under-temperature cases are implemented (power spec 2.9, item 2) but not tested. The bought BMS's cell and current limits are named here once card G1-04 gives them (R-15; the functional safety engineer, step 17). |

| REQ-SYS-053 | While it drives, the robot shall keep its pack current within the pack's discharge and charge limits. |
| --- | --- |
| **Rationale** | D65: the BMS limits act through the torque limiter, DCL on motoring and CCL on regeneration (SG-10). |
| **Source** | N-13, SG-10, D65 |
| **Parent** | SG-10 |
| **Allocation** | MCU, BMS, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL11_TorqueLimiter; t_drive testT19_noRegen (the ESC keeps its regeneration within the limit) |
| **As built** | **Partly** |
| **Where** | MCU torque limiter (electronics spec 2.7.9; D65); the BMS's DCL and CCL from the cell temperatures and the SOC (power spec 2.9, item 6) |
| **Note** | The test passes with the twin's BMS frames; the robot's BMS frames and limits are unknown (R-15). With CCL 0 the descent becomes stop-go on the brakes (D74). The limiter allows a fixed 120 W for the other loads (P_aux_est), not the implement's power, so with the mower running (about 1 kW) the pack current can pass DCL by the implement's current (card G2-PWR, with the MCU firmware engineer). |

| REQ-SYS-054 | When the pack warns, at least 3 s ahead, that it will be disconnected for a cause that may wait (a low charge, a cell near its voltage or temperature limit, or a request to switch the high voltage off) while the robot moves, the robot shall stop and park on its brakes before the disconnection. |
| --- | --- |
| **Rationale** | SG-04: a loss of power on a descent must not tip the robot, so the robot stops first. It covers the disconnections the robot can foresee: a shutdown, a low-energy cut-off, a cell nearing its limit. A protective disconnection (REQ-SYS-052) does not wait for the stop. The warning time is at least 3 s for a disconnection that may wait: a low charge (SOC critical), a cell under-voltage after its warning, an over- or under-temperature, a request to switch the high voltage off; an overcurrent, a short circuit, a cell over-voltage, a measurement or cell-contact fault, a fuse, a charger OVP and lost MCU commands do not wait (the BMS and power engineer, step 7). 3 s covers the MCU's longest controlled stop: its CTRL_STOP time-out at the top speed, v_track_max 1.41 m/s / a_ctrlstop 1.0 m/s^2 + ctrlstop_t_extra 1 s = 2.4 s (params/param_electronics.m:42, 47, 122; D71). Margin: about 0.4 s (13 %) over the whole chain from the warning to the brakes: the warning reaches the MCU in BMS_STATE, sent every 100 ms, plus one 10 ms task sample; then the 2.4 s stop; then the park in the next DRV_CMD frame, the brake's 0.06 s and the ESCs' READY 80 ms after the brake command, about 2.6 s in all; 3 s more at the UV warning (3.00 V) stays far from the 2.50 V absolute minimum, and a cell at the OT warning (50 deg C) rises well under 0.1 K in 3 s. It is safe because the robot is parked before the pack opens; if the cut comes anyway, the stop is brakes-locked, which the descent cap keeps within the park criterion (REQ-SYS-075, D74, CR-003). Provisional under D96: the bought BMS's warning is unknown (G1-04). The functional safety engineer's floor of about 1.5 s (step 11) is within it. |
| **Source** | N-08, N-13, SG-04, D96 |
| **Parent** | SG-04 |
| **Allocation** | BMS, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 |
| **As built** | **Partly** |
| **Where** | BMS OPENING with HV_Opening_Warn, on which the MCU stops (power spec 4.1; architecture section 10); the MCU's fault bit 18 HV_OPENING, class B (ICD section 6) |
| **Note** | The twin's BMS warns before it opens, but opens after at most 0.2 s (t_open_wait), before a stop from speed can end; the bought BMS may cut without a warning (R-15). Decided, pending: the twin changes on card G2-PWR (OPENING waits at most 0.2 s today, t_open_wait), with the MCU's stop on the warning on card G2-MCU (bit 18 HV_OPENING, class B). |

| REQ-SYS-055 | When the robot switches on its high voltage, the robot shall bring the bus up with a precharge current of at most 10 A and at most 2 A^2s through its main contactor at closing, and report HV ready within 1.0 s of the request (D98). |
| --- | --- |
| **Rationale** | SG-10: no short or overheating in the power path; R-09 names the inrush and the precharge energy. D98 sets limits that fit any hardware: at most 10 A of precharge current (D98: 10 A for the twin's K2, with the component's data under card G1-04), at most 2 A^2s through the main contactor when it closes, and HV ready within 1.0 s (the BMS and power engineer, step 7). Margin: the twin's D98 design (7.5 Ohm) peaks at 7.3 A at the pack's 54.6 V, 27 % below the 10 A; 2 A^2s is about 5 times the 0.4 A^2s of a close at 97 % (C dV^2/2R with 5.5 mF and about 18 mOhm), and it holds a close at about 93.4 % of the pack voltage or above (a step of up to 6.7 %, power spec 2.4), which the settled-bus test of D98 keeps; HV is ready in 0.60 s today (T-PWR-03) and 0.5 s (S0). It is safe because the current and the I^2t limits keep K2 and K1 within their ratings, so neither welds, and a welded contactor could not disconnect the pack (SG-10). Provisional until the ESC data arrive (G1-04, D96). |
| **Source** | N-12, N-13, SG-10, D96, D98 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_power_plant precharge_T03, precharge_fail_T03b; t_pack_bms precharge_bench (T-PWR-03.g, K2 latency within 30 ms); tests/requirements/power/T-PWR-03.feature; S0 (HV ready within 1.0 s) |
| **As built** | **Partly** |
| **Where** | BMS PRECHARGE and CLOSE_MAIN (power spec 4.1); the PDU precharge resistor |
| **Note** | With the ESC's standby load, precharge cannot complete at 22 Ohm, and every scenario hides it with a 4.7 Ohm override (R-09, STATUS 10); with 4.7 Ohm a shorted bus puts 850 J into the resistor. precharge_T03 runs on the 5.5 mF bench without the ESC's standby load, which is why it passes. D98 sets the limits; the twin moves to about 7 to 8 Ohm with a settled-bus test and an early short check, and the 4.7 Ohm override goes (decided; the twin changes on card G2-PWR). |

| REQ-SYS-056 | When its charge falls to its return threshold during a mission that the operator has not paused, the robot shall end the mission and return to its dock (D93). |
| --- | --- |
| **Rationale** | N-02: go back to the dock to charge when the energy runs low. D93: in the operator's pause the robot stays paused and tells the operator instead (REQ-SYS-015, -105). |
| **Source** | N-02, N-03, D93 |
| **Parent** | None |
| **Allocation** | GNC, BMS, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: t_gnc_mission testT12SocRth; t_pack_bms soc_estimation_T11, eol_endurance_T13_T13b_T17 (the BMS's SOC and SOH); planned: card G1-02 for a system scenario (hazard log F-06) |
| **As built** | **Partly** |
| **Where** | GNC T8 and P3 on an SOC below soc_rth (gnc spec 2.8, 4.1) |
| **Note** | testT12SocRth fails in the baseline; no system scenario covers the low-energy return (STATUS 33; cards G2-INT, T-03). The return is also taken from a pause (P3), the operator's included, which D93 removes (card G2-GNC); T8 no longer triggers once the robot is on its final transit home. The SOC and SOH are the BMS's estimates; soc_estimation_T11 and eol_endurance_T13_T13b_T17 fail in the baseline on the former pack's expectations (R-29). |

| REQ-SYS-057 | When it returns to its dock on a low charge, the robot shall arrive at its dock with at least SOC 0.20. |
| --- | --- |
| **Rationale** | HAZ-040: a low-energy stop on the way back can strand the robot on a slope. The return reserve is an other measure of that hazard, not a safety function: the safety rests on REQ-SYS-054 and -075 (SG-04). The reserve is SOC 0.20 (D101: at least 10 % above the 0.20 reserve; P.gnc.fsm.soc_reserve, params/param_gnc.m:121; the GNC engineer, step 5). |
| **Source** | N-02, SG-04, D101 |
| **Parent** | None |
| **Allocation** | GNC, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: planned: card G1-02 (the energy budget against S3); t_pack_bms soc_estimation_T11, eol_endurance_T13_T13b_T17 (the BMS's part) |
| **As built** | **Partly** |
| **Where** | GNC soc_rth = soc_reserve + E_home / E_nom, the return threshold (gnc spec 2.8) |
| **Note** | The GNC's energy per metre and the row-test energy disagree, and the end SOC at end of life is about 0.19, under the 0.20 reserve (R-19); S3 has never run (R-20). The SOC and SOH are the BMS's estimates; soc_estimation_T11 and eol_endurance_T13_T13b_T17 fail in the baseline (R-29). T12 does not verify the reserve on arrival, as its SOC follows a scripted ramp (DOCKED at 0.184 after a return at 0.202), so S3 must (D101); the energy per metre behind soc_rth (0.12 Wh/m) has no source (R-19, card G2-PWR). D97: the dock is the twin's in this phase, and the robot is charged by hand from its stand-alone charger under the approved charging procedure (D94). |

| REQ-SYS-103 | If its bus does not charge up when it switches on its high voltage, then the robot shall keep its pack disconnected from the bus, with at most 200 J into its precharge resistor over all its attempts (D98), and report a precharge fault. |
| --- | --- |
| **Rationale** | SG-10: closing the main contactor onto a shorted or failed bus would short the pack. REQ-SYS-055 covers the normal precharge; this one covers the failed one. D98 sets at most 200 J into the precharge resistor over the whole sequence of attempts; the time-out and the number of attempts are design values bounded by the energy: two attempts at 7.5 Ohm with the 0.2 s early short check (the BMS and power engineer, step 7). Margin: the twin's D98 design takes about 80 J per 0.2 s check, so two attempts (159 J) stay 20 % below the 200 J, and three would need a check of about 0.17 s. It is safe because K1 stays open, so the pack never closes onto a short (SG-10), and the energy limit keeps the resistor from overheating. Provisional (D96). |
| **Source** | N-12, N-13, SG-10, D96, D98 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_power_plant precharge_fail_T03b (a shorted bus: K1 never closes, three attempts, then FAULT with the precharge-fail bit), judged on the energy over the whole sequence (D98) on card G2-PWR (today 200 J per attempt); t_pack_bms mode_manager_table |
| **As built** | **Partly** |
| **Where** | BMS PRECHARGE to PRE_FAIL after t_pre_max, three attempts, then FAULT with Fault_PrechargeFail (power spec 4.1; subsystems/power/bms_mode_manager.m) |
| **Note** | The tests pass on the twin's BMS. The robot's BMS, contactors and precharge circuit are not in the as-built record (R-15, card G1-04); the delivered BMS is the baseline, provisionally (D96). Decided, pending: the twin's sequence changes to D98's design on card G2-PWR (today 22 Ohm, 1.5 s and three attempts, about 580 J over the sequence; T-PWR-03b checks 200 J per attempt only). |

| REQ-SYS-104 | When it runs a greenhouse or palm-grove mowing mission of phase 2, the robot shall complete the mission within the energy of one charge, from SOC 0.95, ending with an SOC of at least 0.275 at the pack's end of life (SOH 0.80): the 0.20 reserve plus 10 % of the usable window (D101). |
| --- | --- |
| **Rationale** | N-02 and charter goal 4: finish a mowing mission on one charge. D101 sets the energy requirement per implement and phase in place of a fixed shift (it supersedes D45 and D46): each greenhouse and palm-grove mowing mission of phase 2 ends at SOC 0.275 or above at end of life, from 0.95, which is the 0.20 reserve plus 10 % of the 0.75 usable window, the 1592 Wh stored in that window at end of life, about 1544 Wh at the pack terminals with the discharge efficiency of 0.97 (P.power.pack.E_Wh 2948.4 Wh, P.power.budget.soh_eol 0.8, eta_dis 0.97, params/param_power.m:45, 259-263; the BMS and power engineer, step 7; the pack's energy is REQ-SYS-136). D101 notes 3.7 to 4.3 kWh for 3 h of palm mowing, so a mission at end of life lasts about 1.1 to 1.25 h, which the planner sizes its missions to (card G2-GNC). The return on a low charge is REQ-SYS-056 and -057. Not a safety value. |
| **Source** | N-02, D101 |
| **Parent** | None |
| **Allocation** | GNC, BMS, PWR, DRV |
| **Safety** | No |
| **TBD** | the energy per hour of each design mission, from a phase-2 mowing energy run after D94 and D95 (D101); owner the BMS and power engineer with the GNC engineer; by G1-05 (a RID) |
| **Verification** | **Analysis**, system level, in simulation: planned: card G1-02 (the energy budget of the design missions, with the row-test energy, R-19); S3 once it runs (R-20) |
| **As built** | **Unknown** |
| **Where** | the pack's capacity and the BMS's SOC (power spec 2.9); the GNC's energy per metre (gnc spec 2.8) |
| **Note** | No analysis or scenario shows a full mission on one charge: the GNC's energy per metre and the row-test energy disagree, and the end SOC at end of life is about 0.19 (R-19); S3 has never run (R-20). D101 sets the margin per mission in place of the shift; the energy per hour of each design mission comes from a phase-2 mowing energy run (the TBD), and P.power.budget is still the sprayer's 3 h shift, which card G2-PWR replaces. |

| REQ-SYS-105 | If its charge falls below the return threshold while the operator has paused its mission, then the robot shall tell the operator that it needs to return to its dock (D93). |
| --- | --- |
| **Rationale** | D93: in the operator's pause the robot does not drive home by itself on a low charge; it stays paused (REQ-SYS-015) and tells the operator, who commands the return. |
| **Source** | N-02, N-05, D93 |
| **Parent** | None |
| **Allocation** | GNC, CMP, COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (a low charge in an operator's pause: the alert, and no motion until the return command) |
| **As built** | **Not implemented** |
| **Where** | none: P3 leaves the pause and drives home by itself (gnc_missionFSM.m:286-290) |
| **Note** | D93 removes P3's exit from the operator's pause and adds the alert (card G2-GNC, with D84 and D92, size M); the alert reaches the operator only once a console is defined for the robot (REQ-SYS-082, card G2-COM). |

### 5.10 F-10 Dock and charge

**Scope:** Approach the dock, dock, charge, undock. (8 requirements, 4 of them safety requirements.)

| REQ-SYS-058 | When it returns to its dock, the robot shall approach the dock contacts along its dock segment at no more than 0.2 m/s. |
| --- | --- |
| **Rationale** | N-03: dock without help; HAZ-013: a slow approach limits a crush between the robot and the dock (SG-01). The docking speed is 0.2 m/s (P.gnc.spd.v_dock, params/param_gnc.m:32; the GNC engineer, step 5). Margin, on the stop model of REQ-SYS-029: at 0.2 m/s the stop takes 0.2 x 0.36 + 0.07 = 0.14 m, and the protective field reaches 0.5 + 0.8 x 0.2 = 0.66 m, so 0.52 m is spare; the energy is 0.5 x 291 kg x 0.2^2 = 5.8 J with the mower, which limits a crush against the dock (HAZ-013). It is safe because the robot stops well inside the zone that it sees. The docked check is REQ-SYS-117, and the contact speed of 0.05 m/s REQ-SYS-177: 0.2 m/s is the approach speed. |
| **Source** | N-03, SG-01, D97 |
| **Parent** | SG-01 |
| **Allocation** | GNC, CMP, PWR, NAV, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 (no scenario docks against a dock body, R-10) |
| **As built** | **Partly** |
| **Where** | GNC R3 and R4: the dock approach at 0.2 m/s, DOCKED at the end of the segment (gnc spec 4.1) |
| **Note** | Docking is unsupervised and has never met a dock body; on the robot it will stop short of the dock or push against it (R-10), as the dock body enters the protective field. The stuck check needs a 0.3 m/s command, above the 0.2 m/s docking speed, so it never fires at the dock; the obstacle stop is off in the approach (REQ-SYS-036); the approach ends with an in-place trim that has no time-out (R-10). The dock is not in the as-built record (card G1-04). D97: the dock is the twin's in this phase, F-10 is verified in the twin, and the robot is charged by hand from its stand-alone charger under the approved charging procedure (D94). The dock mask for the protective field is card G2-GNC, with the sensors and navigation engineer. |

| REQ-SYS-059 | While it stands on its dock, the robot shall charge its pack to its full level, the charger's 54.6 V with the current below 2 A (an SOC of 0.97 or more), within the charge window of REQ-SYS-060. |
| --- | --- |
| **Rationale** | N-02 and N-03: charge without help, and start the next day full. The full level is the CC-CV end at 54.6 V (4.20 V per cell) with the current below 2 A for 60 s, an SOC of 0.97 or more (P.power.bms.charge_done, params/param_power.m:200; the CV P.power.charger.cccv_V, 174-176; T-PWR-09's end criterion) (the BMS and power engineer, step 7). The top-up is REQ-SYS-208, split so that each stays singular (re-check 14b). Under D97 the dock is the twin's; on the robot the charge is a step of the approved charging procedure (D94, D97). Not a safety value. |
| **Source** | N-02, N-03, D94, D97, D101 |
| **Parent** | None |
| **Allocation** | BMS, PWR, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_pack_bms cccv_T09, hot_charge_T10b_dock_air (passes: 0.10 to 0.95 within 3.6 h at 50 deg C with dock air), mode_manager_table |
| **As built** | **Partly** |
| **Where** | BMS CHARGING and CHARGE_DONE with the dock contactor K4; the Dock_Charger (power spec 2.8, 4.1); the charge starts only on the MCU's request (Charge_Enable_Req: key on, dock present and STANDBY; mcu_mode_manager.m) |
| **Note** | After charging the BMS never tops up on the dock (R-46; REQ-SYS-208); cccv_T09 fails on stale expectations of the former pack (R-29); the robot's charger is a stand-alone 54.6 V, 20 A unit, not a dock (the as-built record, section 2). D97: the dock is the twin's in this phase; the robot is charged by hand. |

| REQ-SYS-060 | While it charges, the robot shall keep the charge current within the limit of its cell temperature: none with the hottest cell above 45 deg C or the coldest below 10 deg C, at most 20 A in between, and a charge started only with the hottest cell at 40 deg C or less (D94). |
| --- | --- |
| **Rationale** | SG-10: the pack shall not overheat or overcharge when it charges at 45 to 50 deg C. D94 sets the window: a charge starts at 40 deg C or less and stops above 45 deg C, 10 K below the supplier's 55 deg C label; the cold end of 10 deg C is the twin's [EST] table, which allows 20 A only from 10 deg C (P.power.bms.chg.I_cool, params/param_power.m:191), and the fixed 20 A charger can only be cut (D94, D96) (the BMS and power engineer, step 7). Margin: the 10 K covers the gap between the hottest measured cell and the hottest real cell in the sealed box, the NTC's error (+/-0.5 K offset and 0.3 K noise in the twin) and the charge's own heat (about 6 W at 20 A, a slow rise); 20 A is 0.32C, a third of the cells' standard charge (63 A per pack), and the cold end keeps it out of the [EST] plating range. It is safe because the cells are charged only inside their temperature window; on the robot the window is the approved charging procedure, which earns no PL credit, so the BMS cut-off stays the safety function (SF-17). Provisional (D96). |
| **Source** | N-13, SG-10, D94, D96 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_pack_bms hot_charge_T10, hot_charge_T10b_dock_air, hot_charge_T10c_no_dock_air |
| **As built** | **Partly** |
| **Where** | BMS hot-charge table and charge current management (D45; power spec 2.9) |
| **Note** | T-PWR-10 and 10c fail in the baseline; the twin cools the pack with a fan path and dock air that the sealed real pack does not have (R-04); the bought BMS's charge limits are unknown (R-15). Today's limit is the NMC table: 30 A up to 40 deg C, 15 A at 45 deg C and 0 from 50 deg C; D45's table is the former LFP pack's. The robot's charger gives a fixed 20 A, and the BMS cannot lower it, so above about 43 deg C (hottest cell) the charge can only be cut, not reduced (the as-built record, R-15). D94 sets the window; the twin's table and mode manager change on card G2-PWR (decided, pending), and today it charges up to 50 deg C, with no start condition. D97: the robot is charged by hand from its stand-alone charger. |

| REQ-SYS-061 | The robot shall keep its dock contacts dead unless it stands on its dock. |
| --- | --- |
| **Rationale** | SG-10: live parts protected against contact; HAZ-030, live dock contacts and arcs. |
| **Source** | N-13, SG-10 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_pack_bms mode_manager_table (K4 commanded only in CHARGING, opened in CHARGE_DONE); planned: card G1-02 for the system check and the K4 weld (R-45) |
| **As built** | **Partly** |
| **Where** | K4 coil = BMS K4 command AND dock contact (power spec 2.5) |
| **Note** | K4 closes only with dock contact, but a K4 weld is not detected, so the pins stay live after a weld (R-45); K4 opens under charger current (R-46). D97: the dock is the twin's in this phase; the robot is charged by hand from its stand-alone charger under the approved charging procedure (D94). |

| REQ-SYS-062 | When it starts a mission from its dock, the robot shall end its charge and leave the dock under its own power. |
| --- | --- |
| **Rationale** | N-03: dock, charge and go back to work without help. |
| **Source** | N-03 |
| **Parent** | None |
| **Allocation** | BMS, GNC, PWR, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 |
| **As built** | **Not implemented** |
| **Where** | none: the specs have no undock function (the GNC's return ends at DOCKED, gnc spec 4.1) |
| **Note** | The BMS precharges only without dock contact (STANDBY to PRECHARGE needs Dock_Present false, power spec 4.1), so the robot cannot switch on its high voltage while on the dock; a charged robot stays in CHARGE_DONE until the dock contact opens, and stopping a charge opens K4 under the charger's current (R-46). No undock segment exists, and nothing records that the robot is on its dock. The robot itself has no dock in this phase (D97): it is charged by hand from its stand-alone charger under the approved charging procedure (D94), and F-10 is verified in the twin. Card G2-GNC (one item with docking, R-10, size M) and card G2-PWR (with R-45 and R-46, size M); the start sequence needs the MCU (card G2-MCU) and a new field on CAN2 (card G2-INT). |

| REQ-SYS-106 | While it charges, the robot shall keep its drives disabled. |
| --- | --- |
| **Rationale** | SG-10 and HAZ-030: a robot that drives off with the charge current flowing arcs its dock contacts and can drag the charger. The undocking of REQ-SYS-062 ends the charge before the robot moves. |
| **Source** | N-12, N-13, SG-10 |
| **Parent** | SG-10 |
| **Allocation** | BMS, MCU, PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_pack_bms mode_manager_table (CHARGING entered from STANDBY only, with K1 open); planned: card G1-02 (an arm and a drive command during a charge) |
| **As built** | **Hardware** |
| **Where** | BMS: CHARGING only from STANDBY, so the main contactor is open while K4 is closed, and no precharge with the dock contact made (power spec 4.1; subsystems/power/bms_mode_manager.m). MCU: the charge request only in STANDBY, no arm without HV ready, and a charging flag in MCU_STATUS (subsystems/electronics/mcu_mode_manager.m) |
| **Note** | It holds in the twin, because the high voltage is off while the robot charges. The undocking change (REQ-SYS-062, card G2-PWR) must keep it; once the high voltage can be on at the dock, an arm guard on the charging flag is S work for card G2-MCU. On the robot the charger is plugged in by hand, and nothing recorded stops a drive with the plug in (the as-built record; card G1-04). D97: the dock is the twin's in this phase; the robot is charged by hand from its stand-alone charger. |

| REQ-SYS-117 | If the dock does not report contact, with the robot within 0.03 m and 2 deg of its dock pose, within 2.0 s of the end of its dock segment, then the robot shall stop, hold and report a failed docking. |
| --- | --- |
| **Rationale** | N-03: a robot that ends its dock approach short of the contacts, or pushes against the dock body, must not wait or push for ever (R-10). The GNC engineer splits the docked check from the docking speed of REQ-SYS-058 (step 5, Table 1): the contact within 2.0 s of the end of the dock segment, within 0.03 m and 2 deg (P.gnc.fsm.t_docked, dock_tol_xy, dock_tol_yaw; T12's tolerances). Under D97 the dock is the twin's, so the check is verified in the twin. Not a safety value: a person between the robot and the dock is in the protective field, which stops the robot in the dock approach (REQ-SYS-033, -058), so the check bounds a push against the dock body, an equipment risk (R-10); REQ-SYS-058 keeps the safety (for the functional safety engineer to confirm). |
| **Source** | N-03, D97 |
| **Parent** | None |
| **Allocation** | GNC, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (a dock approach that ends short of the contacts, in the twin) |
| **As built** | **Not implemented** |
| **Where** | GNC R5: IDLE 2 s after the end of the dock segment, with no check of the contact (gnc spec 4.1) |
| **Note** | Today R5 goes to IDLE after 2 s with no check (card G2-GNC). The dock contact reaches the MCU on its DOCK_PRESENT input; the BMS and power engineer recommends confirming the dock by the charge that starts (Charging_Active), which would add BMS (step 7, section 5), the GNC engineer's choice on card G2-GNC. |

| REQ-SYS-208 | While it stands on its dock with its pack charged, the robot shall charge its pack again whenever its SOC falls below 0.95, within the charge window of REQ-SYS-060. |
| --- | --- |
| **Rationale** | N-02 and N-03: start the next day full. Split from REQ-SYS-059 so that each stays singular (the register's re-check, step 14b). The top-up band starts below 0.95, the mission start of D101 (P.power.pack.SOC0; the BMS and power engineer, step 7). Under D97 the dock is the twin's; on the robot the top-up is a step of the approved charging procedure (D94, D97). Not a safety value. |
| **Source** | N-02, N-03, D94, D97, D101 |
| **Parent** | None |
| **Allocation** | BMS, PWR, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-PWR (a t_pack_bms case: the robot left on the dock after CHARGE_DONE until its SOC falls below 0.95, then charged again within the charge window) |
| **As built** | **Not implemented** |
| **Where** | none: after CHARGE_DONE the BMS never tops up on the dock (R-46; power spec 4.1) |
| **Note** | Decided, pending: the twin changes on card G2-PWR (R-46). D97: the dock is the twin's in this phase; the robot is charged by hand. |

### 5.11 F-11 Stop in an emergency and reset

**Scope:** The emergency stops on the robot and from the operator, the bumper and the E-stop chain; staying stopped until a deliberate reset and a new arm command; no unexpected start after a fault reset, a returning link or a restart of the controller. (7 requirements, all of them safety requirements.)

| REQ-SYS-063 | When an E-stop button on the robot is pressed, the robot shall remove its traction torque within the STO time of 50 ms and engage its spring brakes (stop category 0, D86), and stop its implement, independently of its software. |
| --- | --- |
| **Rationale** | SG-06: an emergency stop stops the motion and the implement independently of the software (ISO 13850, N-15). D86: stop category 0 for traction, as built; the descent cap keeps every such stop within the park criterion (CR-003). The STO time is at most 50 ms, provisional under D96 until the delivered relay's and ESCs' times come with card G1-04 (the functional safety engineer, step 17, Table 1, in place of the 20 ms of step 13): the BMS and power engineer's 50 ms leaves 30 ms for the delivered relay's release and the ESCs' STO reaction (step 7), while 20 ms left only 3 ms over the twin's own chain and bought 3.6 cm. Margin: the twin removes the torque within about 17 ms of the edge (P.power.safety.t_relay_off 0.015 s, the 1 ms logic and the ESC's 1 ms; params/param_power.m:177), 33 ms inside the limit; 50 ms is 0.06 m of travel at 1.2 m/s, and against 20 ms it adds 0.036 m to S8a's 0.20 m stop, inside the 0.6 m of REQ-SYS-139, the criterion that matters: the delivered chain may take up to about 0.3 s before 0.6 m is reached. It is safe because the stop stays within the bound of REQ-SYS-139. |
| **Source** | N-06, N-15, SG-06, D8, D80, D86, D96 |
| **Parent** | SG-06 |
| **Allocation** | PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8a (STO within 50 ms of the edge, the twin checked at 20 ms; the stop within 0.6 m from 1.2 m/s); t_power_plant estop_T04; t_drive testT09_sto; the implement stop judged against REQ-SYS-048 (at rest within 2.0 s), with t_power_plant mower_T14 as today's check (the blades below 10 % within 3.0 s after the E-stop), pending on cards G2-FSC and G2-PWR |
| **As built** | **Hardware** |
| **Where** | Power_Plant Safety_Chain (the SR latch, its immediate contacts to STO and K3; power spec 2.5; architecture section 10), which the electronics only monitor through their inputs (electronics spec 2.9.1); the implement ECU's reference 0 on Safety_OK; K3 on the 24 V implement rail; the delayed contacts open K1 and K2 after t_cat1 1.5 s (power spec 2.5, 2.7) |
| **Note** | S8a passes, and the twin stops traction as category 0, as D86 requires. The owner confirms E-stop buttons and a safety relay with STO wiring (D80), but their parts, category and wiring are not recorded (card G1-04); the power spec's category-1 claim is corrected in its revision C (D86; power spec 2.5 and 9, O4). The blades are braked under power: the 48 V implement feed stays on until the delayed contacts open K1 at +1.5 s, and K3 cuts only the 24 V implement rail at once; cutting the 48 V at once would leave the blades to coast. estop_T04 checks the chain's timing, not the stop; D86 adds the stopping check in part 2. The value is provisional (D96): the delivered relay's and ESCs' times come with card G1-04. |

| REQ-SYS-064 | When an emergency stop has acted, the robot shall stay stopped until the E-stop is released, a deliberate reset is made from a place where the operator can see the zone round the robot, and a new arm command is given. |
| --- | --- |
| **Rationale** | SG-06: keep the motion and the implement stopped until a deliberate reset and a new arm command (ISO 13850, N-15). Every emergency stop ends this way: the buttons and the bumper on the robot and the operator's remote stop (REQ-SYS-063, -065, -066). The manual reset of ISO 13849-1 is made where the operator can see that nobody is in the danger zone. |
| **Source** | N-06, N-11, N-15, SG-06, D30, D88, D91 |
| **Parent** | SG-06 |
| **Allocation** | PWR, MCU, ELC, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8a (no restart before the release, the reset and the re-arm); t_power_plant rearm_powerup, rearm_button_before_do0, rearm_blocked_by_estop; t_power_plant estop_T04 (the release does not re-close K1; a reset and a new HV request restore HV); t_pack_bms mode_manager_table |
| **As built** | **Partly** |
| **Where** | Safety_Chain SR latch with the reset pulse; the MCU's ESTOP reset guard (D30; electronics spec 2.7.3) |
| **Note** | The relay is re-armed by the MCU's reset pulse, on RC CH9 over the radio and once on every SELFTEST entry, so the reset is not a manual one (hazard log F-01, STATUS 32). D88 and D91 remove the re-arm at every power-up (card G2-MCU, size M); whether a reset over the radio, from wherever the transmitter is, meets the place of the reset is for the safety concept (G2-FSC). The BMS adds a second latch: after a Safety_OK drop, the high voltage returns only on a new HV request edge. After a release and a reset the MCU may go to FAULT on the ESC undervoltage (STATUS 9, to re-check). The reset that re-arms the relay is the DI RESET_BTN or RC CH9, with the DO1 pulse on every reset_req edge (ICD section 6): the button and the pulse are the electronics', CH9 the link's (the electronics and comms engineer's finding F2, for card G2-FSC with G2-MCU). |

| REQ-SYS-065 | When a bumper of the robot is pressed, the robot shall stop as for an E-stop. |
| --- | --- |
| **Rationale** | HAZ-021: the robot does not stop on its E-stop button or bumper (SG-06). |
| **Source** | N-06, N-07, SG-06 |
| **Parent** | SG-06 |
| **Allocation** | PWR, ELC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bit 29); t_elec_plant testDiWord16Cases |
| **As built** | **Hardware** |
| **Where** | The bumpers in the E-stop loop (Loop_OK, power spec 2.5); the MCU fault bit 29 BUMPER, class A |
| **Note** | Implemented in the twin. Bumpers are not confirmed on the robot (D80; hazard log E-01). |

| REQ-SYS-066 | When the operator gives the remote emergency stop, the robot shall stop its motion and its implement independently of its software. |
| --- | --- |
| **Rationale** | SG-06 and N-06: an emergency stop from a distance acts independently of the software, as the stop on the robot does. The stay-stopped part is REQ-SYS-064, for every emergency stop. |
| **Source** | N-06, SG-06, D33, D34 |
| **Parent** | SG-06 |
| **Allocation** | MCU, COM, DRV, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8b (2.0 m/s^2 within 0.1, the stop within 0.70 m, ESTOP latched: today's functional stop); t_mcu_control testTEL28_RemoteEstop; t_mcu_platform testRemoteEstopValidFramesOnly; planned: card G1-02 for the stop independent of the software |
| **As built** | **Partly** |
| **Where** | MCU REMOTE_ESTOP from the RC mode switch on valid frames, ESTOP.DECEL at 2.0 m/s^2, then the park (D33, D34, D71) |
| **Note** | S8b passes for today's remote stop. It is a functional stop through SBUS and the MCU, not independent of the software (PL b at most), and no certified wireless E-stop is fitted (hazard log E-04): the architecture is the safety concept's (card G2-FSC), and the device's data are card G1-04's. With CCL 0 it can trip STOP_FAIL (a) (STATUS 39). |

| REQ-SYS-078 | When a lost link returns, the robot shall stay stopped until the operator arms it again. |
| --- | --- |
| **Rationale** | SG-08: no motion when a link returns, without a deliberate command. |
| **Source** | N-11, SG-08 |
| **Parent** | SG-08 |
| **Allocation** | MCU, CMP, GNC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S6a, S6b (no motion after the link returns until arm off, then on); t_mcu_control testTEL14_NeverActiveWithoutArm; t_gnc_mission testT15McuFrozen (the companion faults 0.20 s after the controller's status freezes, and the mission does not go on when the controller returns) |
| **As built** | **Partly** |
| **Where** | MCU mode manager: resuming after any stop needs a new arm edge (electronics spec 2.7.3) |
| **Note** | The new arm edge works in the twin, but it can come from the receiver's failsafe preset (R-35); one MCU, PL b at most (hazard log SF-16). |

| REQ-SYS-079 | If its controller restarts through a watchdog reset or a brown-out, then the robot shall keep its brakes held and its E-stop chain not re-armed, and show the cause of the restart, until the operator's deliberate reset and a new arm command (D88, D91). |
| --- | --- |
| **Rationale** | SG-08: no start after a reset without a deliberate command; D88, the owner's reset policy, which D91 applies to every power-up, the switch-on included (REQ-SYS-001); hazard log F-01, the manual reset of the safety function (ISO 13849-1, IEC 60204-1). |
| **Source** | N-11, N-12, SG-06, SG-08, D88, D91 |
| **Parent** | SG-08 |
| **Allocation** | MCU, PWR, ELC, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: t_mcu_control testTEL14_Transitions (today's reboot), testTEL34_WatchdogStall (the stall detection); planned: card G1-02 (T-EL-34 and S0 with the reset of D88, and a brown-out case) |
| **As built** | **Not implemented** |
| **Where** | MCU: after a reset, BOOT to SELFTEST to STANDBY, with the reset pulse DO1 sent once on every SELFTEST entry (D30); the reset cause read at boot from IWDG1RSTF only, as the sticky flag wdg_reset_seen in MCU_STATUS; the electronics plant's reset emulation (electronics spec 2.9.2) |
| **Note** | Today a restart runs BOOT to SELFTEST to STANDBY, and SELFTEST re-arms the relay with no manual action (hazard log F-01, STATUS 32). A watchdog reset is shown only as the information flag wdg_reset_seen in MCU_STATUS; a brown-out is not detected (only IWDG1RSTF is read, ICD section 6). On the robot the latches are cleared; in the twin they survive the emulated reset (R-36). Motion still needs an arm edge (REQ-SYS-003). Card G2-MCU makes it D88 with D91 (size M): the reset cause read from the IWDG, BOR and POR flags, no re-arm in the self-test, the cause kept until the operator's reset, and the twin's reboot of the companion computer and the 3-D LiDAR; the robot's safety relay must have a manual reset (card G1-04). The twin has no brown-out emulation (an LV dip with the BOR flag) in elp_watchdog_reset, so the brown-out case cannot be run; card G2-MCU adds it with the reset-cause reading (D88; the electronics and comms engineer's finding F5). |

| REQ-SYS-107 | When the operator resets a fault that stopped the robot, the robot shall stay stopped until the operator arms it again. |
| --- | --- |
| **Rationale** | SG-08: no motion after a reset without a deliberate command. REQ-SYS-064, -078 and -079 cover the emergency stop, a returning link and a controller restart; this covers a fault stop (REQ-SYS-071), a failed self-test (REQ-SYS-002) and the wait of every power-up (REQ-SYS-001, D91). |
| **Source** | N-11, SG-08, D91 |
| **Parent** | SG-08 |
| **Allocation** | MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL14_NeverActiveWithoutArm (a new arm edge after any stop); planned: card G1-02 (a class-A and a class-B fault, the reset, no motion until a new arm edge) |
| **As built** | **Implemented** |
| **Where** | the MCU mode manager: a fault reset needs a reset request at standstill, and moving needs a new arm edge (electronics spec 2.7.3; hazard log SF-16) |
| **Note** | In the twin; one MCU (PL b at most); an arm edge can come from the receiver's failsafe preset (R-35). |

### 5.12 F-12 Communicate and record

**Scope:** The CAN networks, the RC link and the companion link; the operator's console and telemetry; the event and fault records; the mission logs. (4 requirements, none a safety requirement.)

| REQ-SYS-082 | The robot shall report to the operator its mode, its mission progress, the reason of each hold or pause, its active faults with their causes and its charge. |
| --- | --- |
| **Rationale** | N-05: see the mode, the progress, the holds and the faults with their causes; N-12: faults that name their cause. |
| **Source** | N-05, N-12 |
| **Parent** | None |
| **Allocation** | MCU, GNC, CMP, COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_platform testTelemetryStatusFrames; t_gnc_mission testTillerAndStatus |
| **As built** | **Partly** |
| **Where** | MCU telemetry and MCU_STATUS on CAN2; Bus_GncStatus every 0.2 s: state, progress, the hold and pause reasons, the fault word (gnc spec 5.2) |
| **Note** | The hold reasons cannot name a lost drive, sensor or plan, and several health flags are never read (R-40). testTelemetryStatusFrames fails in the baseline on its stale T08-stub checks (STATUS 4.1); MCU_STATUS carries the mode, the fault and warning words and the reset flag, and MCU_POWER the charge. No operator console is defined for the robot (card G2-COM). |

| REQ-SYS-083 | The robot shall record each fault and each mode change with its time, for the maintainer to read after the event. |
| --- | --- |
| **Rationale** | N-12: faults that name their cause; the abnormal cases of the operating concept need a record to be explained. |
| **Source** | N-12, N-14 |
| **Parent** | None |
| **Allocation** | MCU, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in simulation: planned: card G1-02 |
| **As built** | **Not implemented** |
| **Where** | none on the robot; the twin logs its signals in simulation only (Viz_Logging; integration spec 7.6) |
| **Note** | No event or fault record is specified for the MCU or the companion. |

| REQ-SYS-084 | When a mission ends, the robot shall make a record of the mission available to the supervisor: the area covered, the path driven, its holds and pauses, and the energy used. |
| --- | --- |
| **Rationale** | N-14: the supervisor checks the results of the missions afterwards. |
| **Source** | N-14 |
| **Parent** | None |
| **Allocation** | CMP, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Demonstration**, system level, in simulation: planned: card G1-02 |
| **As built** | **Partly** |
| **Where** | Bus_GncStatus area_done_m2, xte_rms and progress (gnc spec 5.2); the scenario metrics in simulation (the S4 coverage) |
| **Note** | The values exist in the twin's status, but nothing on the robot keeps a mission record for the supervisor. |

| REQ-SYS-085 | The robot shall take its mission commands (start, pause, resume, return to the dock, abort and the mission's fault reset) from the operator's console or from the mission buttons of the RC transmitter. |
| --- | --- |
| **Rationale** | N-05: the operator starts, holds, resumes and stops a mission. The mission's fault reset clears a mission fault; it is not the deliberate safety reset of REQ-SYS-064 and -079. |
| **Source** | N-05 |
| **Parent** | None |
| **Allocation** | GNC, CMP, COM, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT11ModeLogic, testT11ClosedLoop |
| **As built** | **Partly** |
| **Where** | Bus_MissionCmd from the HMI or the RC auxiliary channels, edge-detected (gnc spec 5.1); the MCU's RC mapping: START and PAUSE on CH13 edges, RTH on a CH14 rise (rc_aux_cmd and rc_aux_seq in MCU_STATUS, frame 0x200; electronics spec 2.7.2) |
| **Note** | In the twin the commands come from the scenario; no console is defined for the robot (card G2-COM). From the RC come start, pause and resume (CH13's position 2 is START/RESUME, ICD section 7) and the return to the dock; abort and fault reset come from the console only. |

### 5.13 F-13 Plan the missions

**Scope:** Compile a site's mission from its boundary, rows and dock: the lanes, the turns at the lane ends, the transits and the dock approach, within the permitted zone and the slope limits of the fitted implement. (7 requirements, 2 of them safety requirements.)

| REQ-SYS-086 | When the supervisor plans a mowing mission, the robot shall compile it from the site's boundary, rows and dock into lanes, turns at the lane ends, transits and a dock approach. |
| --- | --- |
| **Rationale** | N-14: the supervisor plans the missions of a site; N-01: rows with turns at their ends. |
| **Source** | N-01, N-03, N-14 |
| **Parent** | None |
| **Allocation** | CMP, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_tracking testCompilerGreenhouse, testCompilerPalmSolar (the lanes, the turn radii, the geofence, the footprint clearance and coverage of at least 98 %); t_gnc_mission testT8Greenhouse, testT9PalmT20 |
| **As built** | **Implemented** |
| **Where** | scripts/gnc_compileMission.m, offline: the coverage planner, Dubins headlands, the transits and a 3 m dock approach (gnc spec 2.7; D57) |
| **Note** | The compiler runs offline in MATLAB; on the robot the plan is to reach the companion by a ROS 2 subscription still to be built (D57, card B-02). |

| REQ-SYS-087 | When it plans a greenhouse mission, the robot shall place its lanes so that its body and its implement keep at least 0.275 m from the post lines (D106). |
| --- | --- |
| **Rationale** | The clearance keeps the body and the implement off the posts and walls, with room for the cross-track error, while the robot covers the floor (D54); a planning measure for HAZ-004 and HAZ-009 (SG-01). D106 widens the inset to 0.50 m, so that the deck keeps about 0.125 m of clearance at D100's tracking maximum of 0.15 m: the body and the implement are planned at least 0.275 m from the post lines, 0.475 m from the wall line (the GNC engineer, step 5b, section 4). The unmowed strip along each wall is 0.10 m wider than with the 0.40 m inset, and D106 replaces D54's 0.45 m. Not a safety value. |
| **Source** | N-01, SG-01, D54, D106 |
| **Parent** | None |
| **Allocation** | CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (coverage of at least 95 % of the inset floor; the clearance judged during the run with the deck (D87, D100): at least 0.125 m to the posts, D106's margin at D100's 0.15 m tracking maximum, in place of today's 0.15 m, on card G2-INT); t_gnc_tracking testCompilerGreenhouse (the planned body and deck at least 0.275 m from the post lines, in place of today's cov.clear_min 0.15 m, on card G2-GNC) |
| **As built** | **Partly** |
| **Where** | gnc_compileMission: the inset c_in = 0.20 + 0.15 + 0.05 + max(0, W/2 - w_tool/2) (gnc spec 2.7; D54), and the validation of the footprint with the fitted implement against cov.clear_min 0.15 m |
| **Note** | testCompilerGreenhouse passes. With today's parameters (W 0.78 m, the mower's swath 1.00 m) the inset is 0.40 m, not D54's 0.45 m (scripts/gnc_compileMission.m:132; params/param_gnc.m:148, param_structure.m:30 and :65). S4's own clearance of 0.259 m excludes the mower deck, which reaches beyond the body (R-06). Decided, pending: D106's inset of 0.50 m (gh_c_xte 0.15 m) puts the deck 0.275 m from the post envelope, about 0.125 m at the 0.15 m tracking maximum; the twin's compiler changes on card G2-GNC (today 0.40 m, with the deck 0.175 m from the posts). |

| REQ-SYS-088 | When it plans its lane-end turns, the robot shall keep its body and its implement clear of the walls and posts by 0.5 m + 0.8 m per m/s of its turn speed + 0.10 m for guidance + 0.10 m for navigation (1.1 m at 0.5 m/s). |
| --- | --- |
| **Rationale** | R-06: the deck is invisible to the collision checks; STATUS 18: the protective field sees the end walls in the lane-end turns, so the mower holds there. D87's footprint per implement serves the planner too; the resume pivots are REQ-SYS-091 to -093. The turn clearance must also keep the walls and posts out of the zone where the robot detects people, or each turn near a wall becomes a person hold that waits for the operator (D84; STATUS 18, R-06). The turn clearance is the protective field's reach (0.5 m + 0.8 s x v, D72) plus the turn's tracking share of 0.10 m (the GNC engineer, step 5 and step 5b; L-6) and the navigation error of a greenhouse turn, up to 0.10 m more (step 5b, open point 3; the lead systems engineer's ruling 3 of step 19, the stricter reading): 1.1 m at the 0.5 m/s turn speed, 0.9 m at 0.25 m/s. |
| **Source** | N-01, N-07, SG-01, D72, D81, D87 |
| **Parent** | None |
| **Allocation** | CMP, GNC, STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (with the mower); t_gnc_tracking testCompilerGreenhouse; planned: card G1-02 |
| **As built** | **Partly** |
| **Where** | gnc_compileMission validation: the footprint of the body with the fitted implement kept cov.clear_min (0.15 m) from the posts and walls on every segment, turns included |
| **Note** | testCompilerGreenhouse passes. The online replanner, bypass, corridor and pivot use the body only (R-06). With the mower, S4 holds at the end walls because the protective field reaches them in the turns, not the deck (STATUS 18); the turn clearance of part 2 must cover the field's reach. Card G2-GNC (D87, size M, and M for the rest of R-06). By the lead systems engineer's ruling 3 of step 19 the formula adds the navigation share, and choosing the turn speed or the field's shape in the turns so that the S4 house fits is card G2-GNC's design (gnc spec 9, question 12: a slower turn, 0.9 m at 0.25 m/s, or a field shaped for the turns). Decided, pending: the twin's compiler changes on card G2-GNC (today cov.clear_min 0.15 m). |

| REQ-SYS-089 | The robot shall plan its missions only over ground within the slope envelope of its fitted implement: 20 deg, and 18.5 deg with the sprayer (CR-001), on farm soil, dry sand, the greenhouse floor and concrete; 18 deg on wet loam (REQ-SYS-129); and 15 deg across. |
| --- | --- |
| **Rationale** | SG-04 and the envelope of CR-001: the full-tank sprayer descends slopes up to 18.5 deg, and the robot cannot tell a full tank from an empty one (D75, D81); the others work to the 20 deg design slope. The cross-slope of 15 deg is the envelope that REQ-SYS-130 and -149 rest on, so it binds the planning too (HAZ-038; the functional safety engineer, step 17, Table 1). On wet loam a climb slips past 30 % above about 18 deg (REQ-SYS-129; the structure engineer, step 10, section 6), so the planning keeps to 18 deg there, as the project owner approved on 2026-11-18 (step 22). |
| **Source** | N-08, SG-04, CR-001, D75 |
| **Parent** | SG-04 |
| **Allocation** | CMP, GNC, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (the slope check of card G2-GNC) |
| **As built** | **Not implemented** |
| **Where** | none: the mission planner has no slope limit per implement (STATUS 34) |
| **Note** | The planner has no slope limit per implement (STATUS 34). Until card G2-GNC adds it, the descent cap stops the sprayer at the top of steeper ground (its forward cap is 0 from 22 deg, its reverse cap from 21.5 deg). The text names the soils and the 18 deg on wet loam of REQ-SYS-129, as the project owner approved on 2026-11-18 (step 22; the lead systems engineer's point 3). The tiller descends forward only (REQ-SYS-075), which the planning of its missions takes on card G2-TIL. |

| REQ-SYS-090 | When it plans a mission, the robot shall place no lane-end turn on ground steeper than 10 deg (D53). |
| --- | --- |
| **Rationale** | D53: a turn on a slope can slide the robot downhill (SG-04). |
| **Source** | N-08, SG-04, D53 |
| **Parent** | SG-04 |
| **Allocation** | CMP, GNC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_tracking testCompilerSlopeTurn |
| **As built** | **Implemented** |
| **Where** | gnc_compileMission, P.gnc.cov.slope_turn_max 10 deg (D53; gnc spec 2.7), and the Hybrid A* replanner, with no turning primitives on ground steeper than slope_turn_max |
| **Note** | testCompilerSlopeTurn passes in the baseline: on world 2's 20 deg ramp the compiler shortens the rows until the turn is at or below 10 deg. T7b, the structure's turn check, cannot fail (R-54). |

| REQ-SYS-093 | When it plans a mission, the robot shall permit a pivot on a segment only where the segment is wide enough for the footprint of the robot with its fitted implement (D87). |
| --- | --- |
| **Rationale** | D87: on a resume the robot pivots in place only on lanes that the mission compiler flags as wide enough for the footprint with the fitted implement; it replaces D67's pivot without a clearance check (R-06, STATUS 16c). |
| **Source** | N-01, N-05, SG-01, D87 |
| **Parent** | None |
| **Allocation** | CMP, GNC, STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: planned: card G1-02 (S4 re-scored with the deck, D87) |
| **As built** | **Partly** |
| **Where** | The plan's pivot flag per segment (column 10, pivot_allowed), set by gnc_compileMission (gnc spec 4.1, P1) |
| **Note** | The flag exists, but the compiler sets it on every row, transit and dock segment, with no footprint check: a resume pivot on a wall lane drives the deck into the post line (a margin of about -0.61 m, R-06). |

| REQ-SYS-207 | When it plans a mission, the robot shall keep its footprint off every step higher than its ground clearance (REQ-SYS-156, 0.12 m), such as the 0.15 m irrigation berms of the palm basins and the 0.25 m greenhouse beds (REQ-SYS-168), on every segment, turn and transit. |
| --- | --- |
| **Rationale** | N-01: a step higher than the ground clearance grounds the body. REQ-SYS-156 and -168 record the clause, and this requirement makes it binding (the lead systems engineer's ruling 5 of step 19). Today the compiler keeps the footprint cov.clear_min (0.15 m) from the world's boxes and cylinders, and the raised beds are boxes of class 5; it checks the DEM slope for the turns (D53), but no step height, and the berms are not obstacles in the site models. Card G2-GNC turns the steps higher than the clearance into keep-outs, from the site model or the mission file (the GNC engineer, step 19, open point 1; the lead systems engineer's point 27). Not a safety value until the functional safety engineer confirms it. |
| **Source** | N-01, N-08, D53, D54 |
| **Parent** | None |
| **Allocation** | CMP, GNC, STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-GNC (a compiler test with a 0.15 m berm across a planned lane: the plan keeps off it or is refused; testCompilerGreenhouse with the beds) |
| **As built** | **Not implemented** |
| **Where** | none: the mission compiler checks the DEM slope for the turns (D53), but no step height, and the berms are not obstacles in the site models (gnc spec 5.7) |
| **Note** | Decided, pending: the steps higher than the clearance become keep-outs on card G2-GNC, from the site model or the mission file. |

### 5.14 F-14 Respond to lost links

**Scope:** The stop and the park on the brakes when the operator's RC link, the companion's commands, the controller, a drive's reports or the pack's reports are lost. (5 requirements, all of them safety requirements.)

| REQ-SYS-067 | If the operator's RC link is lost or its receiver goes to failsafe, then the robot shall start a controlled stop within 0.13 s (D37) and park on its brakes. |
| --- | --- |
| **Rationale** | SG-07: on the loss of the operator's link the robot stops and parks. D44: the link is required in autonomous operation too. D34: a failsafe never latches an emergency stop. |
| **Source** | N-10, SG-07, D34, D37, D44 |
| **Parent** | SG-07 |
| **Allocation** | MCU, COM, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S6a, S6b (their criteria are D100's); S2b (on the 20 deg ramp); t_mcu_control testTEL26_Failsafe, testTEL41_RcLossInAuto; t_mcu_platform testTEL05_LinkLost (the loss detected 0.100 s after the last valid frame, the first part of D37's 0.13 s) |
| **As built** | **Partly** |
| **Where** | MCU fault bits 2 and 3 (class B) into CTRL_STOP and the park (D44, D71) |
| **Note** | S2b passes (roll-back 1.7 mm). S6a and S6b fail in the baseline on both the deceleration (1.0994 against 1.05 m/s^2) and the standstill (1.61 against 1.5 s); their deceleration limit equals the coasting deceleration (R-63). The radio model of the row tests cannot produce a link loss (R-12); one MCU, PL b at most. |

| REQ-SYS-068 | If the companion's commands stop arriving, or arrive stale or corrupted, for longer than the command time-out of 0.20 s during autonomous operation, then the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | SG-07: on the loss of the companion's commands the robot stops and parks. The command time-out is 0.20 s (P.electronics.safety.comp_timeout: the COMP_CMD sequence number unchanged in AUTO_START and AUTO, bit 4, ICD section 6; the MCU firmware engineer, step 4), 4 COMP_CMD periods of 50 ms (20 Hz), robust to jitter. Margin: from 1.2 m/s the robot moves 0.24 m during it, then about 0.81 m in the controlled stop at 1.0 m/s^2 (jerk 10 m/s^3; the replay of the stop model of REQ-SYS-029, with the ESC's lag), about 1.05 m in all, inside the protective field's 1.46 m at 1.2 m/s (D72), which goes blind with the companion. It is safe because the robot stops inside the zone that it last saw. |
| **Source** | N-10, SG-07 |
| **Parent** | SG-07 |
| **Allocation** | MCU, CMP, COM, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL32_CompanionTimeout, testTEL13_FaultBits (case 4) |
| **As built** | **Partly** |
| **Where** | MCU fault bit 4 COMP_TIMEOUT: the COMP_CMD sequence number unchanged for 0.20 s in AUTO_START and AUTO, class B (ICD section 6) |
| **Note** | The tests pass. A companion that repeats a frozen frame is caught; one whose sender keeps counting while its guidance hangs is not (hazard log SF-06), and no check of the commands' content is shown. COMP_CMD carries no end-to-end checksum, only the sequence counter; an E2E CRC over the payload is proposed for card G2-MCU with G2-COM (the electronics and comms engineer's finding F4). |

| REQ-SYS-069 | If its controller stalls for longer than the watchdog time of 0.08 s (0.35 s for its slowest task), then the robot shall stop its motion and park on its brakes. |
| --- | --- |
| **Rationale** | SG-07: on the loss of the controller the robot stops and parks. The watchdog time is the chain that T-EL-34 shows: a stalled task's age passes its limit after the limit plus one task period (the alive counter's rate transition to 1 kHz), when DO0 drops and the kick stops, and the IWDG resets the controller 0.05 s later (wdg.age_lim [0.02 0.06 0.2] s for Task_10, Task_50 and Task_100, wdg.timeout 0.05 s; electronics spec 2.7.11). For the control task DO0 drops 0.03 s and the reset follows 0.08 s after a stall; for Task_50, 0.11 s and 0.16 s (T-EL-34: the kick stops 110 ms after a Task_50 stall, the reset 50 ms later); for the slowest task (Task_100, period 0.1 s), 0.30 s and 0.35 s (the register's re-check, step 14b, in place of step 4's 0.07 s and 0.25 s, which left out the task period). Provisional (D96). Margin: DO0 low opens the safety relay, so STO and the spring brakes act (category 0, D86) within 0.03 s + 15 ms of a control-task stall, before the reset; if the outputs freeze instead, each ESC quick-stops 0.1 s after its last command (P.drive.esc.t_can_timeout, 10 frames of 10 ms; QUICK_STOP at 2.0 m/s^2; the drive engineer, step 9, Table 2). No test covers a Task_100 stall: card G2-MCU adds one to T-EL-34, or lowers Task_100's age limit. It is safe because the robot stops either way, from the watchdog or from the ESCs' own time-out. |
| **Source** | N-10, SG-07, D96 |
| **Parent** | SG-07 |
| **Allocation** | MCU, ELC, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL34_WatchdogStall; t_elec_plant testWatchdogResetAndDoGating; t_drive testT10_canTimeout; planned: card G2-MCU (a Task_100 stall case in T-EL-34) |
| **As built** | **Partly** |
| **Where** | MCU independent watchdog (50 ms), all outputs low during a reset; the ESC command time-out into QUICK_STOP, then the brake (electronics spec 2.7.11, 2.9.2; drive spec 2.9) |
| **Note** | The tests pass. After the reset the robot is to wait for the operator's reset (REQ-SYS-079, D88, D91). The task-overrun fault cannot fire on the real controller (R-08); the robot's reset behaviour differs from the simulation (R-36); the controller board is still to be confirmed (R-59, card G1-04). |

| REQ-SYS-070 | If a drive stops reporting to the controller for longer than the feedback time-out of 50 ms, then the robot shall stop its motion and park on its brakes. |
| --- | --- |
| **Rationale** | SG-07: the robot stops and parks when it loses control of a drive. The feedback time-out is 50 ms (P.electronics.safety.drv_timeout 0.05, params/param_electronics.m:95: the FB1 or FB3 age; ICD section 6; T-EL-35 verifies 0.050 +/- 0.010 s); the part-1 TBD's 30 ms was the old spec's value (the MCU firmware engineer, step 4; the drive engineer, step 9, Table 2). Provisional (D96). Margin: 50 ms is 5 FB1 periods of 10 ms (100 Hz) and 0.07 m of travel at 1.40 m/s, and T18 shows no false time-out at a 10 % drop rate. It is safe because the class-A reaction drops DO0, so STO and the spring brakes act. |
| **Source** | N-10, SG-07, D96 |
| **Parent** | SG-07 |
| **Allocation** | MCU, DRV, COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL35_FeedbackLoss; t_drive testT18_frameDrops |
| **As built** | **Partly** |
| **Where** | MCU fault bits 5 and 6 DRV_L/R_TIMEOUT, class A (ICD section 6) |
| **Note** | The tests pass. A lost FB2 frame zeroes the available torque while the link reads OK, and no fault says why (R-39); the ESC, its CAN protocol and the meaning of its commands are assumed (R-14). The class-A reaction removes the torque; that it ends parked on the brakes is not shown. |

| REQ-SYS-108 | If the pack's reports stop arriving for longer than the report time-out of 0.5 s, then the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | SG-04 and SG-10: the stop before a disconnection (REQ-SYS-054) and the pack current limits (REQ-SYS-053) rest on the pack's reports; without them the robot can neither keep the pack's limits nor see a disconnection coming, and the pack can open under a moving robot (HAZ-040). It stops as it does for its other lost links (SG-07). The report time-out is 0.5 s (P.electronics.safety.bms_timeout 0.5, bit 9; the MCU firmware engineer, step 4), 5 frame periods at 10 Hz, provisional under D96: the bought BMS's frames and rates are unknown (R-15). Margin: from 1.40 m/s the robot moves 0.7 m before the controlled stop starts, with the last DCL and CCL held, and the BMS protects its cells itself (REQ-SYS-052) and opens when the MCU's commands stop (t_pack_bms mcu_cmd_timeout). It is safe because the pack keeps its own protections while the robot stops. |
| **Source** | N-08, N-10, N-13, SG-04, SG-10, D65, D96 |
| **Parent** | SG-04 |
| **Allocation** | MCU, BMS, COM, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bit 9 BMS_TIMEOUT), testTEL48_ClassBStopParks (the class-B stop and park) |
| **As built** | **Hardware** |
| **Where** | MCU fault bit 9 BMS_TIMEOUT: any BMS frame older than 0.5 s, class B, latched (ICD section 6) |
| **Note** | The tests pass with the twin's BMS frames. The robot's bought BMS, its frames and their rates are unknown (R-15, card G1-04). The BMS in turn opens the main contactor when the MCU's commands stop (t_pack_bms mcu_cmd_timeout). |

### 5.15 F-15 Stop and hold, and stay stable on slopes

**Scope:** The stops on faults and their park on the brakes; the stop monitor; the overspeed and runaway monitors; the tilt stop; the IMU's faults and the RC limp; no tip-over in the envelope; the roll-back at a hill start; the stuck stop and the electronics' temperature stop. (14 requirements, 13 of them safety requirements.)

| REQ-SYS-071 | When a fault that needs a stop occurs while the robot moves, the robot shall decelerate to a standstill and park on its brakes. |
| --- | --- |
| **Rationale** | SG-05: every stop ends held on the brakes (D71, the project owner's decision "brakes first, STO as backup"). |
| **Source** | N-10, SG-05, D33, D71 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL48_ClassBStopParks, testTEL52_CtrlStopTimeoutV0; t_drive testT19b_parkOnDescent; S2b |
| **As built** | **Implemented** |
| **Where** | MCU class-B faults into CTRL_STOP at 1.0 m/s^2, then the parking phase (D33, D71; electronics spec 2.7.3) |
| **Note** | The tests and S2b pass. The class-B faults include a low-voltage rail out of range (bit 19) and a track speed mismatch (REQ-SYS-081). |

| REQ-SYS-072 | The robot shall end every stop held on its brakes, on any slope of its envelope. |
| --- | --- |
| **Rationale** | SG-05, its first part; charter goal 3 (safe stops everywhere in the envelope). |
| **Source** | N-08, SG-05, D71, D82 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2 (the roll-back in the hold); S2b; S2c and S2d (held within 0.02 m; the S2 family's criteria are D100's); t_drive testT08_brakeHold; t_structure T8_parking |
| **As built** | **Partly** |
| **Where** | The D71 parking phase; the spring brakes and the ESC's park (drive spec 2.7, 2.9) |
| **Note** | The S2 family holds within 2 mm. The brakes are estimates (5 N m per motor [EST]); nothing monitors them, because the confirmation echoes the command (STATUS 27, hazard log F-02); testT08_brakeHold is incomplete in the baseline, a Long test not run since D64 (to run with T06_LONG=1 on card G2-DRV); the slope brake test is planned (T-04). |

| REQ-SYS-073 | If a stop does not slow the robot, its brakes do not confirm, or it moves while it should stand still, then the robot shall remove its traction torque within the monitor time: 0.55 s for a stop that does not slow or brakes that do not confirm, 0.05 s for a speed that rises 0.1 m/s above its value at the stop's start, and 0.25 s for motion at standstill. |
| --- | --- |
| **Rationale** | D71: the stop monitor (STOP_FAIL) is the STO backup of a soft stop (SG-05). The monitor times are the MCU firmware engineer's (step 4), provisional under D96 (L-1): safety.stopfail_t0 0.5, stopfail_brake_t 0.5, stopfail_dv 0.1 and stopfail_move_t 0.2 [EST], then bit 22 one sample later and STO 15 ms after DO0 drops (D71, ICD section 6). Margin: STO with the spring brakes is the backup of a failed soft stop, and in 0.55 s at 1.40 m/s the robot moves at most 0.77 m further before the relay opens; the brakes then hold it, which STO alone would not on a slope (STATUS 27). It is safe for SG-05 because the backup acts within a stop distance of the failed stop. It is not a backup of the person stop: a soft stop that does not slow adds up to 0.66 m at 1.2 m/s before STO, against the protective field's 0.13 m margin (REQ-SYS-029), so card G2-FSC does not count it as the person stop's backup, or shortens the time for a stop that does not slow to about 0.3 s (the functional safety engineer, step 17). The times are estimates, which the functional safety engineer checks in step 17 (L-1). |
| **Source** | N-08, SG-05, D71, D96 |
| **Parent** | SG-05 |
| **Allocation** | MCU, PWR, DRV, ELC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL49_StopNotDecelerating, testTEL50_BrakeNeverConfirms, testTEL51_MotionInStandby |
| **As built** | **Partly** |
| **Where** | MCU stop monitor, STOP_FAIL bit 22, class A in maskDO0: DO0 low, and the safety chain opens STO and K3 after 15 ms (D71; ICD section 6) |
| **Note** | The tests pass. The brake confirmation echoes the command, so a stuck brake reaches standby, and the STO alone cannot hold the robot on a slope (STATUS 27); the monitor runs on the same MCU (PL b at most). |

| REQ-SYS-074 | The robot shall not gain speed downhill beyond the speed of its mode. |
| --- | --- |
| **Rationale** | SG-05, its second part (HAZ-025, a runaway when the pack cannot take charge). |
| **Source** | N-08, SG-05, D71, D74 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2c (CCL 0); t_mcu_control testTEL53_RunawayParksAtOnce, testTEL56_CclZeroManualStop, testTEL57_CclZeroAutoStop; t_drive testT19_noRegen, testT07_regen (the ESC's overspeed trip, the last stage) |
| **As built** | **Implemented** |
| **Where** | MCU OVERSPEED bit 16: the 1.70 m/s limit, the speed-follow term and the descent trip (D71, D74) |
| **Note** | S2c passes on CCL 0 with stop-go parks; of its 15 parks with no implement, 14 come from the speed-follow term (STATUS 5.3; inferred from their trip times and speeds, not traced). |

| REQ-SYS-075 | The robot shall not tip over in a brakes-locked stop, an E-stop or a loss of power anywhere in its envelope: the 20 deg design slope, and 18.5 deg for the full-tank sprayer (CR-001); with the tiller, the design slope descended forward only (raised and facing uphill, it tips backward at 11.8 deg with the sharp-edge track end). |
| --- | --- |
| **Rationale** | SG-04, its first part, with the envelope of CR-001, and the tiller's descent forward only, which the project owner approved on 2026-11-18 (step 22): raised and facing uphill, the tiller tips backward at 16.4 deg static and 11.8 deg with the sharp-edge track end, and the climbing torque lowers it further, not yet quantified (structure spec 6.3; the structure engineer, steps 10 and 19). The tiller's climbs are set on card G2-TIL. |
| **Source** | N-08, SG-04, CR-001, CR-003, D75, D82 |
| **Parent** | SG-04 |
| **Allocation** | MCU, DRV, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2c, S2d (park criterion (b), the pitch bound; the S2 family's criteria are D100's); t_structure T3_static_pitch_implements; the analysis behind the table: the structure engineer's descent steps 2, 8 and 11 (work/team/descent/structure) |
| **As built** | **Partly** |
| **Where** | The park criterion (structure spec 2.11; utilities/scn_descent_parks.m) on the stops of the S2 family |
| **Note** | Every stop of S2c and S2d keeps the pitch bound (STATUS 5.3). The tip speeds rest on an estimated contact stiffness, damping and friction, within about +/-50 %, until the slope brake test (T-04, STATUS 35); the raised tiller tips at rest (STATUS 36), and facing uphill it tips backward at 16.4 deg static and 11.8 deg with the sharp-edge track end (the climbing torque lowers it further, not yet quantified), so it may descend the design slope forward but not climb it (structure spec 6.3; card G2-TIL). |

| REQ-SYS-076 | If its tilt passes its stop limit of 28 deg for 0.5 s, then the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | SG-04: the robot shall not pass its tilt limits. The tilt stop limit is 28 deg for 0.5 s (P.electronics.safety.tilt_stop, t_tilt, params/param_electronics.m:94; the MCU firmware engineer, step 4). Margin: 8 deg above the 20 deg design slope, with the warning at 25 deg (REQ-SYS-147, D47). The stop acts on the larger of roll and pitch. In roll it lies 20 deg below the static tip-over with no implement, about 48 deg (atan(0.30/0.267), P.structure.geom.B, P.structure.lofi.h_cg), and 10.7 deg below the full-tank sprayer's lateral tip at 38.7 deg (structure spec 6.3). In pitch it holds for every configuration whose pitch tip angle lies above 28 deg; three lie below it, each near an end contact (structure spec 6.3, on the model geometry and, in brackets, the sharp-edge track end): the raised tiller facing uphill tips backward at 16.4 deg (11.8 deg; the climbing torque lowers it further, not yet quantified), which the tiller's climb limit on card G2-TIL keeps out (a later phase, D76); the raised mower deck facing downhill at 23.7 deg (19.2 deg), which D81's rule (deck down whenever the robot moves) and D74's raised-deck rows keep out, the MCU's backstop waiting for the twin's lift-sense fix (REQ-SYS-044); and the deck-down mower with its gauge wheels lifted at 25.4 deg (20.7 deg), which rests only on the gauge wheels staying down, with no rule yet that keeps them down (card G2-STR). It is safe because a stop at 28 deg leaves the robot short of its tip-over angle in every configuration that may move under those rules; for the deck-down mower with a lifted gauge wheel it is not shown (card G2-STR). |
| **Source** | N-08, SG-04, D47, D74, D76, D81 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL33_Tilt, testTiltGateAndDerate; S2 (no tilt fault in a start or a stop on the ramp) |
| **As built** | **Partly** |
| **Where** | MCU fw_imu_tilt and the fault bits 14 TILT_WARN (derate 0.5) and 15 TILT_STOP (class B) (D47, D73) |
| **Note** | One IMU and one MCU (PL b at most); a wrong but plausible IMU removes the tilt stop without a fault (R-05, hazard log F-03). No IMU is in the as-built record (card G1-04). The warning before the stop is REQ-SYS-147 (25 deg for 0.5 s, half speed, bit 14, class C). |

| REQ-SYS-077 | While its IMU fault is set, the robot shall allow RC driving only at or below the limp speed of D90: the lower of a quarter of its top speed and the descent cap's speed target (the cap less 0.10 m/s) at the 20 deg design slope for the fitted implement. |
| --- | --- |
| **Rationale** | SG-04: without a tilt measurement the robot cannot keep its tilt limits, so RC driving limps at a speed that is safe anywhere in the envelope. D85 lets the operator drive by RC at a limp speed, and D90 sets it: about 0.35 m/s with no implement or the mower, 0.15 m/s forward and 0.10 m/s in reverse with the full-tank sprayer. The latch is REQ-SYS-110, and the autonomous stop REQ-SYS-024. A quarter of its top speed is the firmware's derate of the track speed limit, 1.41 m/s to 0.354 m/s (D90's quarter-speed derate; the MCU firmware engineer, step 4, section 5), and the speed target is the cap less 0.10 m/s (lim.descent.dv; structure spec 2.11), which gives D90's values. |
| **Source** | N-04, N-10, SG-04, D73, D85, D90 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL46_TiltHoldOnInvalidImu, testTEL47_StuckImuFault (their expected values change with the latch of D85); planned: card G1-02 for the RC limp at the speeds of D90 with each implement |
| **As built** | **Implemented** |
| **Where** | MCU fault bit 24 IMU_FAULT, class C: derate 0.25, the yaw loop off, the tilt bits held (D73; ICD section 6); the descent cap on its 20 deg column while the IMU is invalid (lim.descent.theta_imu; D74) |
| **Note** | RC driving limps today at about 0.35 m/s, under the cap of the 20 deg column: the speed that D90 sets. The latch until an operator reset with a valid IMU is REQ-SYS-110 (card G2-MCU, D85, size S). A wrong but plausible IMU is not caught (R-05; REQ-SYS-111). The firmware derates the track speed limit (1.41 m/s) to 0.354 m/s, a quarter of its top speed, as the text says (the lead systems engineer's ruling 2 of step 19). |

| REQ-SYS-081 | If the two speed measurements of a track (the drive's speed feedback and the encoder-count speed) disagree by more than the mismatch limit of 0.3 m/s for 0.2 s, then the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | The speed limits, the descent cap and the stop monitor rest on the track speeds (hazard log SF-18, SG-01). The mismatch limit is 0.3 m/s for 0.2 s (P.electronics.drv.enc_mismatch_v, enc_mismatch_t, params/param_electronics.m:60; bits 12 and 13; the MCU firmware engineer, step 4), provisional under D96 (L-2); the ESC's FB1 speed has 1 rpm per bit (0.0005 m/s at the track) behind a 2 ms filter, so the drive's own error is far below it (the drive engineer, step 9, Table 2). Margin: not shown against the descent cap: a disagreement below 0.3 m/s goes unseen and is larger than the cap's 0.10 m/s margin, so REQ-SYS-116 applies the speed limits and the cap to the higher of the measurements (L-2). It is safe with REQ-SYS-116, as a measurement that reads low within the limit then cannot raise the true speed past the cap. Both speeds come from the motor encoder that the ESC reads, so the check catches a frame or computation error, not an encoder fault (SF-18, G1-04). |
| **Source** | N-10, SG-01, D96 |
| **Parent** | SG-01 |
| **Allocation** | MCU, DRV, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bits 12 and 13) |
| **As built** | **Hardware** |
| **Where** | MCU fault bits 12 and 13 ENC_L/R_MISMATCH, 0.2 s, class B (electronics spec 2.7.7; ICD section 6) |
| **Note** | The test passes. Whether the robot's encoders are read by the MCU or come over CAN is open (hazard log SF-18, card G1-04); the drive's fault detection is idealised (R-42). |

| REQ-SYS-109 | If its speed passes the speed it is commanded by more than 0.2 m/s, or passes 1.70 m/s, for 0.2 s, then the robot shall stop and park on its brakes. |
| --- | --- |
| **Rationale** | SG-01: the robot keeps its speed within what its detection zone can stop. REQ-SYS-029 and -038 set the limit; this is the monitor for a limit that fails or a speed that is measured wrong (HAZ-010; hazard log SF-12). REQ-SYS-074 keeps the downhill case under SG-05. The overspeed margins are 0.2 m/s above the commanded speed for 0.2 s (lim.v_follow_dv, t_persist) and 1.70 m/s (lim.v_overspeed = 1.2 x v_track_max; bit 16 (a) and (c)) (the MCU firmware engineer, step 4), provisional under D96. Margin: 0.2 m/s is above the drives' normal tracking error (S1: at most 5 %, 0.07 m/s at 1.40 m/s), and it catches a runaway (HAZ-025) within 0.2 s; 1.70 m/s is below the ESC's mechanical overspeed (1.88 m/s), and the 0.18 m/s between them holds a runaway that gains speed at up to about 0.67 m/s^2 over the 0.2 s persistence and the 70 ms park; a faster runaway meets the ESC's own overspeed stage, the last stage. The firmware has one absolute limit, not one per mode, and since the command never exceeds the mode's limit, the follow term bounds every mode at its limit plus 0.2 m/s. It is safe because a speed that runs away from its command is stopped within 0.2 s. |
| **Source** | N-07, N-10, SG-01, SG-05, D71, D96 |
| **Parent** | SG-01 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL53_RunawayParksAtOnce, testTEL56_CclZeroManualStop, testTEL57_CclZeroAutoStop (the OVERSPEED tests of REQ-SYS-074); planned: card G1-02 (an overspeed on level ground); t_drive testT19_noRegen, testT07_regen (the ESC's overspeed trip, the last stage) |
| **As built** | **Implemented** |
| **Where** | MCU fault bit 16 OVERSPEED, class B: the 1.70 m/s limit and the speed-follow term (D71; ICD section 6) |
| **Note** | In the twin; one MCU on the encoders (PL b at most, hazard log SF-12). Where the robot's encoders are read is open (card G1-04). |

| REQ-SYS-110 | When its IMU fault has been set, the robot shall keep it set until the operator resets it with a valid IMU (D85). |
| --- | --- |
| **Rationale** | D85: the IMU fault stays latched until an operator reset with a valid IMU, so that the robot neither resumes autonomous work nor leaves the RC limp of REQ-SYS-077 on an IMU that recovers by itself (SG-04). |
| **Source** | N-10, SG-04, D85 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (the latch, and the reset with a valid and with an invalid IMU); t_mcu_control testTEL46_TiltHoldOnInvalidImu and testTEL47_StuckImuFault change their expected values with the latch |
| **As built** | **Not implemented** |
| **Where** | MCU fault bit 24 IMU_FAULT, not latched today (D73; ICD section 6) |
| **Note** | Today the IMU fault clears by itself once the IMU is valid. The latch is card G2-MCU (D85, size S): bit 24 latched in fw_safety_supervisor, cleared only on an operator reset with a valid IMU; T-EL-46 and T-EL-47 change their expected values, which the lead systems engineer approves. |

| REQ-SYS-111 | If its measured pitch or roll is wrong by more than the IMU error bound of its descent table (0.75 deg, D83), then the robot shall treat its IMU as failed within the detection time of 1.0 s (D104). |
| --- | --- |
| **Rationale** | SG-04: the descent table and the tilt stop rest on the measured attitude, and the table allows 0.75 deg of error (D83). A frozen IMU raises IMU_FAULT (D73, D85); a wrong but plausible one (a bias, a misalignment, a slow drift) removes the tilt stop, the descent cap and the heading together with no fault (R-05; hazard log F-03). An IMU treated as failed takes REQ-SYS-024, -077 and -110. The means (a second, independent attitude measurement) is the safety concept's (hazard log E-03). The detection time is 1.0 s (D104): the descent cap's hold window (lim.descent.t_hold), so that a wrong pitch cannot set the cap for more than one window (the MCU firmware engineer, step 4). Margin: before G2 the safety concept must show that its detection method meets 1.0 s and that 1.0 s keeps the tilt limits (D104). It is safe as far as that showing holds; no detection exists today (R-05, E-03). |
| **Source** | N-08, N-10, SG-04, D83, D85, D104 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS, NAV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (fault injection: an IMU bias and a slow drift beyond 0.75 deg) |
| **As built** | **Not implemented** |
| **Where** | none: sens_acq_imu catches a frozen or out-of-range IMU only (D73; ICD section 6) |
| **Note** | A wrong but plausible IMU is not seen (R-05, hazard log F-03). Hazard log E-03 (class 1) is answered by the safety concept (card G2-FSC); card G2-NAV confirms the 0.75 deg bound with real IMU data (D83). D104 sets 1.0 s as a RID to G1-05, not a TBD: the safety concept shows the detection method and that 1.0 s keeps the tilt limits (card G2-FSC, E-03, with cards G2-MCU and G2-NAV). |

| REQ-SYS-112 | When it moves off on a slope of its envelope, the robot shall roll back by no more than 0.05 m, from the release of its brakes until it moves forward. |
| --- | --- |
| **Rationale** | SG-05 (HAZ-027, PL a): since D66 every restart on a slope unparks with no holding torque, about 0.1 m of roll-back at every hill start on 20 deg after a stop of 2 s or more (R-37). The roll-back limit is 0.05 m, on every slope of the envelope (20 deg; 18.5 deg with the full sprayer) and with either motor (the drive engineer, step 9, Table 1): S2 and S2b already allow 0.05 m of roll-back in a hold, and D100 extends S2's roll-back check over the hill start. Margin: half the 0.1 m that HAZ-027 rates S1 (a bruise); it bounds what gravity can put into the motion to 316 kg x g x sin 20 deg x 0.05 m = 53 J, half of HAZ-027's case, and it is a twentieth of the 1.0 m kept to a person (S5, D100). It is safe because a restart then moves the robot no further than a hold already may, and the limit needs no new detection; HAZ-027's PL a is within reach of one MCU. |
| **Source** | N-08, SG-05, D100 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2 (the restart after the hold, with S2's window extended to the restart + 3 s and the hill-start roll-back checked: the criteria of D100, card G2-INT); t_mcu_control testTEL61_HillStartRollback (it expects the roll-back today; its expected values change by decision 14's route, T-03); planned: a t_drive bench case of the hill start (card G2-DRV) |
| **As built** | **Partly** |
| **Where** | the D71 park and the ESC's unpark (drive spec 2.7, 2.9) |
| **Note** | About 0.1 m of roll-back at every hill start on 20 deg (R-37, DRV-05: 0.07 to 0.12 m; S2spray rolls back at 0.061 m/s after the release): not met. The fix is pending on card G2-MCU with G2-DRV: keep the speed integrator through PARKED, or preload the stored holding torque before the brakes release (the drive engineer, step 9). The MCU's hill-start window is lim.descent.t_release 0.5 s (STATUS 30). |

| REQ-SYS-113 | If the robot moves at less than 0.05 m/s for 5 s while it is commanded at 0.15 m/s or more in autonomous operation, then the robot shall stop, hold on its brakes and end the mission. |
| --- | --- |
| **Rationale** | N-10: a robot that pushes against something it cannot see (in its blind near zone, a post beside its deck, the dock body of R-10), or that digs its tracks in, must not go on pushing; N-05: the stop names its cause. The stuck time is 5 s, with a command of at least 0.15 m/s and a measured speed under 0.05 m/s (fsm.t_stuck, v_stuck_meas, params/param_gnc.m:119-120; the GNC engineer, step 5): the lower command threshold also covers the 0.2 m/s dock approach (R-10). Not a safety value. |
| **Source** | N-10, N-05 |
| **Parent** | None |
| **Allocation** | GNC, NAV, MCU, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT17Stuck |
| **As built** | **Implemented** |
| **Where** | GNC mission manager, G1: stuck (fsm.t_stuck, v_stuck_cmd, v_stuck_meas) into FAULT with fault bit 4, in a mission and a return to the dock (subsystems/gnc/gnc_missionFSM.m; gnc spec 4.1) |
| **Note** | testT17Stuck passes: FAULT 5.0 s after the plant is held. The check needs a command of at least 0.3 m/s, so it misses a blocked pivot (REQ-SYS-092) and a push at the 0.2 m/s docking speed (R-10, card G2-GNC). Decided, pending: the twin changes on card G2-GNC (today the command threshold is 0.3 m/s, fsm.v_stuck_cmd). |

| REQ-SYS-114 | If its electronics pass their temperature stop limit of 80 deg C in their enclosure for 5 s, then the robot shall make a controlled stop and park on its brakes. |
| --- | --- |
| **Rationale** | SG-06 and N-13: the robot works in heat up to 45 to 50 deg C (N-13), and REQ-SYS-205 relies on this stop to keep the E-stop chain's parts within their rating, so it is a safety requirement under SG-06 (the functional safety engineer, step 17, section 3, point 8). A controller that overheats can fail while the robot drives, so the robot stops first; before the stop limit it derates at a warning limit. The stop limit is 80 deg C of enclosure air for 5 s (P.electronics.safety.T_encl_crit 353.15 K, params/param_electronics.m:98; t_persist 5 s, bit 21), with the warning at 70 deg C (REQ-SYS-148), 5 K below the STM32H743's 85 deg C maximum ambient (the MCU firmware engineer, step 4), provisional under D96; the electronics and comms engineer, the second owner, keeps the final value open until card G1-04's data on the robot's board and box (step 8, section 5.1). Margin: 5 K below the H743's 85 deg C; at 50 deg C ambient and 1000 W/m^2 the twin's box air is 24.7 K below the stop with the fans and 8.7 K with the fans failed, and on the stricter plain-box reading it is 56.5 deg C with the fans and 76 deg C without, 4 K below the stop (electronics spec 2.9.4; the box's area is settled on card G2-VAL with the robot's box from G1-04). It is safe for the controller, which stops before its limit. For the safety relay it is not shown: the stop lies 25 K above a typical 55 deg C safety relay, the conflict of REQ-SYS-205 and -165, an open safety value for the owner, decided on card G1-04 once the delivered relay's rating and place are known; the lead systems engineer recommends parts rated for the 80 deg C stop or the relay and the STO interface placed outside the box, as both keep the mission in the heat (the lead systems engineer's point 1). The final value comes with card G1-04, as a RID, not a TBD. |
| **Source** | N-10, N-13, SG-06 |
| **Parent** | SG-06 |
| **Allocation** | MCU, ELC, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bits 20 and 21), testTEL48_ClassBStopParks (the class-B stop and park) |
| **As built** | **Hardware** |
| **Where** | MCU fault bits 20 ENCL_TEMP_WARN (class C, derate 0.5) and 21 ENCL_TEMP_CRIT (class B, latched) (ICD section 6); the fans (electronics spec 2.7.12); the enclosure thermal model (electronics spec 2.9.4) |
| **Note** | Implemented in the twin. The thermal model and the limits are those of the STM32H743 board and its enclosure, which may not be the robot's (R-59, card G1-04). |

## 6. The technical requirements

One section per kind, in the register's order. Each requirement is the same table as a functional one, with two more rows: its kind and the function it serves (section 1.5).

### 6.1 Performance

**Kind:** a speed, a time, a distance, an accuracy, an endurance, a slope (33 requirements, 13 of them safety requirements.)

| REQ-SYS-118 | When it is switched on, the robot shall finish its self-test and show its state within 2 s. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-01 Manage the modes and the arm state |
| **Rationale** | N-05 and N-11: after a switch-on the operator sees at once whether the robot came up, before the reset of D91. The value is the self-test time-out (P.electronics.safety.selftest_timeout 2.0 s, params/param_electronics.m; STANDBY at selftest_t 1.0 s today, FAULT under D91) and S0's 1.5 s (architecture section 12, which changes with D91) (the MCU firmware engineer, step 4, T-MCU-5). Not a safety value: the wait for the operator's reset is REQ-SYS-001. |
| **Source** | N-05, N-11, D91 |
| **Parent** | None |
| **Allocation** | MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S0 (architecture section 12; its criteria change with D91 on card G2-INT) |
| **As built** | **Partly** |
| **Where** | the MCU mode manager, BOOT to SELFTEST (subsystems/electronics/mcu_mode_manager.m; electronics spec 2.7.3) |
| **Note** | S0 passes in the baseline: standby within 1.5 s. Under D91 the robot shows FAULT and waits for the operator's reset after the self-test (card G2-MCU). Not on the robot yet: the firmware is not flashed (card B-01). |

| REQ-SYS-119 | The robot shall drive at its RC speed limit of 1.40 m/s on level firm ground with its heaviest implement load, from a full pack down to the 0.20 reserve. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-02 Drive by remote control |
| **Rationale** | N-03 and N-04: the transits and RC driving need the top speed of the platform as built: 5 km/h at the motors' rated 3000 rpm with the 20:1 gearboxes and the 0.090 m sprockets, 1.41 m/s (as-built record section 1; P.electronics.lim.v_track_max = 1.414 m/s, params/param_electronics.m:40), at the RC speed limit of REQ-SYS-010 (P.electronics.lim.v_max_man 1.4, :41) and down to the reserve of D101 (the drive engineer, step 9, TD-1). On level ground the motors need about 40 V at that speed, mostly back-EMF (drive spec 2.3); at the 0.20 reserve the pack's open-circuit voltage is 46.2 V (13 x 3.55 V, P.power.cell.OCV25), which leaves about 6 V for the sag. |
| **Source** | N-03, N-04, D101 |
| **Parent** | None |
| **Allocation** | DRV, MCU, PWR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S1 (1.2 m/s; architecture section 12); planned: card G2-DRV (the T1 profile to 1.40 m/s at the reserve's voltage, with the heaviest implement load) |
| **As built** | **Implemented** |
| **Where** | the drive (P.drive.motor.n_rated, P.drive.gear.N and r_spr) and the MCU limits (P.electronics.lim) |
| **Note** | The twin reaches 1.41 m/s at 3000 rpm, and S1 passes at 1.2 m/s; a run at the reserve's voltage with the heaviest load is not on file (card G2-DRV). On the robot the gear ratio and the sprocket radius are estimates (as-built record open items 1 and 2, card G1-04). |

| REQ-SYS-120 | While in RC driving on level ground, the robot shall reach 90 % of a commanded speed of 1.2 m/s within 2.0 s, and hold the commanded speed within 0.03 m/s and the commanded turn rate within 10 % in steady driving. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-02 Drive by remote control |
| **Rationale** | N-04: the operator drives as the sticks command; this is the performance part next to REQ-SYS-008's bound (the functional safety engineer's FSE-22 of part 1). The values are S1's criteria (architecture section 12: the rise, the steady error of 0.03 m/s, the yaw rate within 10 % after 2 s) (the MCU firmware engineer, step 4, T-MCU-3). |
| **Source** | N-04 |
| **Parent** | None |
| **Allocation** | MCU, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S1 (architecture section 12); t_mcu_control testTEL24_ThrottleAndLatency, testTEL25_PivotAndMixer |
| **As built** | **Implemented** |
| **Where** | the MCU reference shaper, mixer and yaw loop (electronics spec 2.7.5, 2.7.6) and the ESCs' speed loops (drive spec 2.9) |
| **Note** | S1 passes in the twin, with the 1 kW default (D95). Not on the robot yet: the firmware is not flashed (card B-01). |

| REQ-SYS-121 | The robot shall mow the S4 greenhouse (9.6 m by 37.5 m), at least 95 % of its floor inside the coverage inset, within 22 min (D107). |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-03 Run missions |
| **Rationale** | N-01: the customers need the work done in time (plan Table 1). D107 sets the work rate for the greenhouse, the S4 house within 22 min and verified by S4, while the palm grove's work rate comes with its design mission (the GNC engineer's TR-G7, step 5, Table 6, and step 5b). The coverage of at least 95 % of the floor inside the inset is S4's (architecture section 12, D100), with the 0.50 m inset of D106. The robot took 13.2 min in T8 (2026-11-16) with the 0.40 m inset; the 0.50 m inset moves the outer lanes 0.10 m inward, and the estimate is recomputed when card G2-GNC changes the inset. |
| **Source** | N-01, D100, D106, D107 |
| **Parent** | None |
| **Allocation** | GNC, CMP, NAV, PER |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (architecture section 12) |
| **As built** | **Partly** |
| **Where** | the coverage planner and the greenhouse mission (P.gnc.cov; scripts/missions/greenhouse_std.json) |
| **Note** | The last full S4 run (2026-11-14) covered 97.1 % in 13.7 min with the 0.40 m inset, passing both values; T8 took 13.2 min (2026-11-16). Since D72 the run with the mower holds at the lane-end walls and reaches no row (STATUS 18), until the footprint work of R-06 on card G2-GNC. |

| REQ-SYS-122 | The robot shall come to a stop with every part of it and of its implement at least 0.30 m inside the boundary of its permitted operating zone in a greenhouse, and 0.50 m in the open. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-04 Know where the robot is |
| **Rationale** | SG-03 and N-09: the robot stays inside its permitted zone (the functional safety engineer's FSE-13 of part 1; REQ-SYS-025). The margin covers the largest position error that the budget allows, or that the integrity check of REQ-SYS-095 can leave undetected with the drift in its detection time, plus a spare: in a greenhouse REQ-SYS-020's 0.25 m along the path and at the end of an outage, as a boundary can lie across the path at a row end, plus 0.05 m, 0.30 m (the register's re-check, step 14b; the lead systems engineer's point 10 keeps it over the functional safety engineer's 0.20 m, the stricter reading); in the open REQ-SYS-095's undetected error with the guidance share and its drift, 0.35 + 0.05 + 0.07 = 0.47 m, inside 0.50 m (the functional safety engineer, step 17, Table 1; the lead systems engineer's points 10 and 11). With REQ-SYS-095's bounds the drift is covered: 0.24 m in a greenhouse and 0.47 m in the open. Margin: 0.05 m in a greenhouse and 0.03 m in the open. It is safe because the stop then ends inside the zone at the largest error that the budget and the integrity check allow, with the robot's own stop distance inside the check (a predictive check of the footprint over the stop distance, card G2-GNC, which takes the kind of site from the plan's mission type). |
| **Source** | N-09, SG-03 |
| **Parent** | SG-03 |
| **Allocation** | GNC, NAV, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G1-02 (T16 with the robot moving at its speed toward the boundary, the stop point checked) |
| **As built** | **Not implemented** |
| **Where** | the GNC geofence (gnc spec 4.1): the reference point checked 0.5 m outside the boundary for 0.2 s |
| **Note** | Today the check trips 0.5 m outside the boundary, on the reference point, so the stop can end up to 0.5 m + 1.33 m (the stop from 1.2 m/s on the stop model of REQ-SYS-029) + 1.31 m (the deck's reach) beyond it. The fix, a predictive check of the footprint over the stop distance, is card G2-GNC. |

| REQ-SYS-123 | When RTK GNSS is lost for 30 s while it drives a palm-grove row at 1.0 m/s, the robot shall hold its heading error within 2 deg at the end of the outage. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-04 Know where the robot is |
| **Rationale** | N-09 and N-10: palm canopy and buildings cut the RTK corrections. The heading bound is S7's (architecture section 12, D49); T-NAV-05 (2026-11-14) gives 0.33 deg of yaw drift after 30 s with a 90 deg turn. The position error at the end of the outage is REQ-SYS-020's 0.25 m, which part 2's error budget sets, stricter than S7's 0.5 m (the sensors and navigation engineer, step 6); the safety bounds on the path are REQ-SYS-022 and -026, and the stop on a lost position is REQ-SYS-023. |
| **Source** | N-09, N-10, D49 |
| **Parent** | None |
| **Allocation** | NAV, SNS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S7 farm, wet and temp (architecture section 12) |
| **As built** | **Partly** |
| **Where** | the navigation EKF and its dead reckoning (Shaddad_NavFusion; sensors spec 7.6) |
| **Note** | S7 fails in the baseline run of 2026-11-08 (moving 3.7 m against 20 m, NEES in bound 0.41) until the three changes of REQ-SYS-021 (cards G2-GNC and G2-NAV); STATUS item 4 records the pause about 4 s into the outage. S7's position criterion is to align with REQ-SYS-020 on card G2-INT. |

| REQ-SYS-124 | While it runs a mission, the robot shall keep its position uncertainty consistent with its actual position error: in every mission at least 90 % of the samples with a 3-DOF pose NEES within the 95 % chi-square bound (7.81), and a mean NEES within [1.5, 4.5]. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-04 Know where the robot is |
| **Rationale** | SG-03 and N-09: both layers of D89 and the integrity requirement REQ-SYS-095 judge the robot's own uncertainty (REQ-SYS-022, -023), and they stop the robot in time only while that uncertainty is honest (the sensors and navigation engineer, step 6, TR-SN1; SNS-N1 of part 1). Margin: a consistent filter keeps 95 % of the samples within the bound, and 90 % leaves 5 points for correlated errors; the mean NEES of a consistent 3-DOF filter is 3, inside [1.5, 4.5]. It is safe because the gates of REQ-SYS-022 and -023 then act on an uncertainty that the true error does not exceed for long. T-NAV-02 gives a mean NEES of 3.69 (2026-11-08); S7's criterion is the NEES in bound at least 0.9 of the time. |
| **Source** | N-09, SG-03, D69, D89 |
| **Parent** | SG-03 |
| **Allocation** | NAV, SNS, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4; S7 (the NEES in bound); t_nav tNAV02_LawnMower, tNAV07_GreenhouseLidar; t_perception tSM01_AlongAisleHonesty |
| **As built** | **Partly** |
| **Where** | Shaddad_NavFusion's EKF covariance (posStd, covPose); Shaddad_Perception's scan-match covariance (D68) |
| **Note** | T-NAV-02 passes (mean NEES 3.69); S4 along 1.66 and across 0.55 (2026-11-14). S7 fails: NEES in bound 0.41 (wet), 0.41 (temp), 0.44 (farm), 2026-11-08. No false RTK fix is modelled (R-43). Card G2-NAV. |

| REQ-SYS-125 | While it works a row with no condition that its speed governor limits (dense vegetation, a slope, a tracking error, a low charge or an obstacle), the robot shall hold its speed within 0.05 m/s of its plan's work speed: 0.5 m/s in a greenhouse and 1.0 m/s in a palm grove. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-01 and N-02: the rows are mown at the plan's work speed, which the work rate (REQ-SYS-121) and the mission energy (REQ-SYS-104) rest on (the GNC engineer, step 5, TR-G2): 0.5 m/s in a greenhouse (the first design's 22 min benchmark) and 1.0 m/s in a palm grove (D46's speed, while D101 supersedes its shift) (P.gnc.spd.v_work_*, params/param_gnc.m:31), within T5's +/-0.05 m/s. The solar park of TR-G2 is not a site of this phase, and the palm grove's route time comes with its design mission (D107), so neither is in the text. The governor's designed slow-downs (D46's vegetation limit and the pitch, cross-track, heading, low-charge and obstacle limits of P.gnc.spd) lie outside the text; T5 judges the speed against the commanded speed (the register's re-check, step 14b). |
| **Source** | N-01, N-02, D107 |
| **Parent** | None |
| **Allocation** | GNC, MCU, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 and S5 (the rows); t_gnc_tracking testT5SpeedStep once its bench starts the mission (R-30, card T-03) |
| **As built** | **Partly** |
| **Where** | the speed governor (P.gnc.spd) and the MCU's speed loops (electronics spec 2.7.5) |
| **Note** | S5 last ran on 2026-10-23, on the former robot (1.06 times nominal), and has not run on the current robot (card G2-INT, with the bypass of D99). T5 fails with the other tracking tests, whose bench never starts its mission (R-30, card T-03). |

| REQ-SYS-126 | While it turns at a row end, the robot shall keep its speed at or below its plan's turn speed, 0.5 m/s by default and 0.8 m/s in the palm missions, and its lateral acceleration at or below 0.3 m/s^2. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-01 and N-07: the turns at the row ends are slow, so that the turn clearance of REQ-SYS-088 and the protective field fit them. The values are the speed governor's (P.gnc.spd.v_turn_max, a_lat, params/param_gnc.m) and the palm missions' speed_turn_max (the GNC engineer, step 5, TR-G3); the mission bench's T9 turns at 0.764 m/s or less (2026-11-16). |
| **Source** | N-01, N-07 |
| **Parent** | None |
| **Allocation** | GNC, CMP, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: the mission bench's T9 (t_gnc_mission); t_gnc_tracking testT4HeadlandTurn once its bench starts the mission (R-30, card T-03) |
| **As built** | **Partly** |
| **Where** | the speed governor and the plan's turn speed (gnc spec 2.5, 2.7) |
| **Note** | T9 passes its criterion (the commanded turn speed and the true speed over the middle half of each turn at 0.764 m/s or less, 2026-11-16), but the true speed peaks at 0.821 m/s at a turn's entry, above the palm missions' 0.8 m/s (gnc spec 8, T9, reported, not judged): the speed profile ahead of each turn, and a whole-turn true-speed criterion in T9, are pending on card G2-GNC (the lead systems engineer's point 26). |

| REQ-SYS-127 | The robot shall follow lane-end turns of 0.81 m radius or more within 0.10 m of the planned turn. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-01: the greenhouse plan skips a row on the 0.90 m spacing, with turns of 0.81 m radius (P.gnc.cov.gh_R); the palm turns are 2.0 m, and the replans 1.5 m (global) and 1.0 m (local). The 0.10 m is the turns' guidance share in the error budget (the GNC engineer, step 5, TR-G5 and section 4; step 5b, Table 1), on which REQ-SYS-026's turn bounds and REQ-SYS-088's clearance rest. |
| **Source** | N-01 |
| **Parent** | None |
| **Allocation** | GNC, MCU, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 (the turns); t_gnc_tracking testT4HeadlandTurn once its bench starts the mission (R-30, card T-03) |
| **As built** | **Partly** |
| **Where** | the trackers in TURN (gnc spec 2.4) |
| **Note** | Not met today: S4's turns reach 0.225 m off the estimate (2026-11-14); the fix is card G2-GNC. testT4HeadlandTurn judges 0.15 m today (gnc spec 8, T4); its criterion of 0.10 m follows on card G2-GNC. |

| REQ-SYS-128 | The robot shall develop at its sprockets a tractive force of at least 1.4 kN continuously at 1.0 m/s, and at least 1.7 kN for 30 s. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-08: the 20 deg design slope (REQ-SYS-089, -097; CR-001). The duty is 316 kg on 20 deg on farm soil (f 0.107): 1372 N, and 1669 N with 0.8 m/s^2 and the reflected inertia (the drive engineer, step 9, TD-2; drive spec 2.1). Capability today: the 1 kW units give 2.54 kN continuous through the gears (a margin of 1.8), the 750 W motors 1.91 kN (1.4) and about 1.72 kN at 45 deg C (1.2); both are limited by adhesion near 2.1 kN (mu 0.7 [EST]). |
| **Source** | N-08, CR-001, D95 |
| **Parent** | None |
| **Allocation** | DRV, STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_drive testT03_climb and T4; S2; the sizing of drive spec 2.1 |
| **As built** | **Partly** |
| **Where** | the drive (P.drive.motor, P.drive.gear; drive spec 2.1, 2.2) |
| **Note** | The 1 kW default meets it by design (D95); the as-built 750 W motors meet it with less margin, 1.2 at 45 deg C. The motors' data are estimates until their datasheets (card G1-04). |

| REQ-SYS-129 | The robot shall climb, descend, stop and park on longitudinal slopes of up to 20 deg with no implement or the mower (deck down) and of up to 18.5 deg with the full-tank sprayer (CR-001), on farm soil, dry sand, the greenhouse floor and concrete, and of up to 18 deg on wet loam. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-08: the slope envelope by soil (CR-001, D75). The traction limits are the structure engineer's (step 10, section 6; structure spec 2.11, descent step 2): on wet loam a 20 deg climb slips past 30 % above about 18 deg. The tiller, a later phase (card G2-TIL), descends forward only: raised and facing uphill it tips backward at 16.4 deg static and 11.8 deg with the sharp-edge track end (the climbing torque lowers it further, not yet quantified) (structure spec 6.3). |
| **Source** | N-08, CR-001, D75 |
| **Parent** | None |
| **Allocation** | STR, DRV, MCU, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: the S2 family (S2, S2b, S2c, S2d; the criteria of D100); t_structure T7 and T8_parking; the slope brake test on the robot (card T-04) |
| **As built** | **Partly** |
| **Where** | the track-soil contact and the speed governor on slopes (structure spec 2.11; gnc spec 2.5) |
| **Note** | The twin climbs the 20 deg ramp on farm soil (S2, with the speed shortfall of REQ-SYS-097); the other soils are not run on the ramp, and the contact data are estimates until the slope brake test (T-04). |

| REQ-SYS-130 | The robot shall traverse and park on cross-slopes of up to 15 deg on farm soil with any implement, without sliding more than 0.1 m in a turn of 2 m radius, and creeping no more than 0.02 m in 600 s when parked. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-05 Move along the path |
| **Rationale** | N-08: the cross-slope part of the envelope (the structure engineer, step 10, section 6). The GNC's roll governor limits the speed to 0.3 m/s at 15 deg and to 0 at 18 deg; T7b slides 0.000 m at 15 deg (a margin of 0.25) and T8 drifts 0.08 mm on the side slope. The parked drift takes REQ-SYS-145's bound. T7b cannot fail (R-54), so the 0.1 m slide is verified by a test that can fail: R-54's fix, or T11 automated (card T-04). |
| **Source** | N-08 |
| **Parent** | None |
| **Allocation** | STR, GNC, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_structure T7b_turn_on_slopes (it cannot fail, R-54: its fix or T11 automated on card T-04), T8_parking |
| **As built** | **Partly** |
| **Where** | the roll governor (gnc spec 2.5) and the track-soil contact (structure spec 2.11) |
| **Note** | T7b and T8 pass in the twin; T7b cannot fail (R-54), and the contact data are estimates until the slope brake test (T-04). Decided, pending: a slide test that can fail (R-54, T11; card T-04). |

| REQ-SYS-131 | The robot shall detect its test person (REQ-SYS-033) anywhere in its protective field at every speed up to its autonomous limit of 1.2 m/s: from the front of its body or fitted implement to 0.5 m + 0.8 s x \|v\| beyond it, and on open ground (palm groves and transits) ahead of it to at least 3.4 m at 1.0 m/s and 4.0 m at 1.2 m/s (D108), over a width of 1.08 m, ahead of it and, below 0.1 m/s, behind it; the near zone of D84 aside. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-06 Sense the surroundings |
| **Rationale** | SG-01 and N-07 (the sensors and navigation engineer, step 6, TR-SN2). The field's far corner lies 2.3 m from the LiDAR with the mower at 1.2 m/s in a greenhouse, and about 4.8 m on open ground with D108's 4.0 m (1.20 + 4.0 - 0.45 m), so the detection needs a range of at least 4.8 m on the test person (card G1-04's LiDAR), a horizontal field of view of +/-80 deg ahead and 360 deg at standstill, and the body seen between 0.5 m and 1.0 m above the ground (P.sensors.per.protL0, protKv, protW, protXF, protXR; P.electronics.lim.v_max_auto 1.2 m/s). Margin: the 0.3 m body returns about 7 of the 1 deg beams at 2.3 m and about 3 at 4.8 m, and the 0.2 m child 5 and 2, against the one kept return that raises the person stop, or two adjacent returns within 3 m, where an isolated return is removed as dust (sensors spec 3.7). It is safe because the person stop then starts for a person anywhere in the field that the stop analysis of REQ-SYS-038 sizes. D108 (the project owner, on the functional safety engineer's step 17, section 3, point 5): for a person walking head-on, the field ahead on open ground grows to about 3.0 to 3.4 m at 1.0 m/s and 3.6 to 4.0 m at 1.2 m/s, of which the text takes the upper ends, the stricter reading; in a greenhouse the residual risk is accepted under D79. D108's ranges rest on an ideal stop of 1.2 s from 1.2 m/s (0.42 + 0.72 + 1.6 x (0.35 + 1.2) = 3.62 m); the AUTO stop of the stop model of REQ-SYS-029 takes 1.9 s to rest, which card G2-FSC counts when it designs the field. The width covers the mower's 1.05 m with 0.015 m a side, which is thin (the functional safety engineer, step 17). |
| **Source** | N-07, SG-01, D72, D84, D108 |
| **Parent** | SG-01 |
| **Allocation** | PER, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_perception tPER02_ProtectiveField, tPER02b_ImplementField; planned: card G1-02 (the 0.3 m test person at the field's corners, ahead and behind); planned: S5's head-on case (D108, card G2-INT) |
| **As built** | **Partly** |
| **Where** | Shaddad_Perception's protective field (sensors spec 3.7, 7.8); the LiDAR model (3.4, 7.5) |
| **Note** | In the twin the FAST2D LiDAR sees its band at every range from 0.5 m to 40 m, all round. No LiDAR is in the as-built record, so its mount, its view past the body and its range on a 0.3 m body wait for card G1-04. Decided, pending: the field ahead on open ground of D108 on cards G2-FSC and G2-GNC; today the field is 0.5 m + 0.8 s x \|v\| on every site (P.sensors.per.protL0 0.5, protKv 0.8), which covers a standing or crossing person, not one walking toward the robot: about 3.6 m would be needed for a walker at 1.6 m/s against the robot at 1.2 m/s (0.35 s chain, 1.0 m/s^2; REQ-SYS-038's analysis on card G1-02, with G2-FSC). Card G2-FSC sizes the field on the one stop model with a stated contact-speed criterion; if that needs more than D108's range, it goes back to the owner as an amendment of D108 (the lead systems engineer's point 14). |

| REQ-SYS-132 | When its test person enters its protective field, the robot shall raise its person stop within 0.25 s, the LiDAR's own scan time included. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-06 Sense the surroundings |
| **Rationale** | SG-01: the perception's share of the stop of REQ-SYS-038 (the sensors and navigation engineer, step 6, TR-SN3): one scan period (0.1 s, P.sensors.lidar.Ts), the LiDAR's own scan time on the robot (up to 0.1 s) and the processing (at most 40 ms, tCompiledTiming's limit) make 0.24 s. Margin: 0.01 s over that sum; T-PER-02 stops within one scan (at most 0.15 s). It is safe because the stop model of REQ-SYS-029 counts this 0.25 s, then one GNC step and the link to the ESC, 0.36 s from a person entering the field to the start of the stop. |
| **Source** | N-07, SG-01, D72 |
| **Parent** | SG-01 |
| **Allocation** | PER, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_perception tPER02_ProtectiveField, tCompiledTiming |
| **As built** | **Partly** |
| **Where** | Shaddad_Perception at 10 Hz |
| **Note** | tPER02 passes. tCompiledTiming fails on a loaded PC (44.3 ms against 40 ms; 26.2 ms on 2026-11-14). The robot's LiDAR and companion are not in the as-built record (card G1-04). |

| REQ-SYS-133 | When a pedestrian walking at up to 0.8 m/s crosses its path while it works at 1.0 m/s, the robot shall stop and keep at least 1.0 m between itself or its implement and the pedestrian. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-07 Hold and avoid |
| **Rationale** | SG-01 (PL d) and N-07: the robot stops before a person and holds. D100 sets S5's person criterion: at least 1.0 m to a pedestrian who meets the robot, with D84's operator resume. Margin: the guidance holds when an obstacle comes within 1.5 m of the body's front bumper (P.gnc.obs.d_stop, x_f 0.50 m, params/param_gnc.m:103), and the perception's protective field stops the robot on a kept point in its path (D72): with no implement the robot stops 1.5 m from a person standing in its path, 0.5 m beyond the bound, but with the mower deck 0.70 m ahead of the bumper only about 0.8 m from the deck, below the bound, as the obstacle corridor leaves out the implement's reach (R-06): not met with the mower today, and measuring the hold from the fitted implement's front is card G2-GNC (R-06). For a pedestrian crossing from the side the margin is not yet measured, and one walking head-on (D108) needs its own analysis (cards G2-INT, G1-02). It is safe, with the hold measured from the implement's front, because the bound keeps the person outside the robot's stopping distance, and SG-01's own measures (REQ-SYS-033, -037) do not rest on it. |
| **Source** | N-07, SG-01, D100, D84, D108 |
| **Parent** | SG-01 |
| **Allocation** | PER, SNS, GNC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5 (architecture section 12; the pedestrian retimed to cross the robot's path and the scripted resume on card G2-INT); planned: S5 with the mower and S5's head-on case (D108), card G2-INT |
| **As built** | **Partly** |
| **Where** | the perception's protective field (P.sensors.per; D72) and the GNC's person hold (P.gnc.obs) |
| **Note** | S5 last ran on 2026-10-23 on the former robot, and its pedestrian came no closer than 32.7 m, so the criterion passed with no person near. D84's resume is on card G2-GNC. S5's pedestrian (0.8 m/s, D100) is the scenario's meeting case, and REQ-SYS-033's 1.6 m/s the speed at which a person crossing the field must be detected; they do not conflict (the functional safety engineer, step 17). |

| REQ-SYS-134 | While it goes round a static obstacle in its lane, the robot shall keep at least 0.5 m from the obstacle and be back on its path within 5 m after it. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-07 Hold and avoid |
| **Rationale** | N-01 and N-07: a palm-grove lane can be blocked by a box or a branch. D100 sets S5's criteria: at least 0.5 m to the box until G2-FSC rules on detours round a person, T7's lane bound (2.5 m), and the rejoin (abs(XTE) < 0.1 m) within 5 m. D99 allows no automatic bypass after a person hold or lost scans, nor with the mower, until G2-GNC; a blocked lane then holds, pauses and alerts. The person case is REQ-SYS-133, and the bypass speed REQ-SYS-135. |
| **Source** | N-01, N-07, D100, D99 |
| **Parent** | None |
| **Allocation** | GNC, PER |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S5 (architecture section 12); the GNC test T7 |
| **As built** | **Partly** |
| **Where** | the local avoidance (VFH, TEB) and the bypass gates of the GNC (gnc spec) |
| **Note** | S5 on 2026-10-23 (the former robot): 0.40 m to the box and back on the path after 6.2 m, both failing; the current robot has not run S5. VFH is the default bypass by D99, while the parameters still select TEB (P.gnc.variant.teb = 1) until card G2-GNC. |

| REQ-SYS-135 | While it goes round an obstacle, the robot shall keep its speed at or below 0.6 m/s. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-07 Hold and avoid |
| **Rationale** | N-07: a detour is slow, so that a person beside the lane is met at a low speed. The value is the bypass speed (P.gnc.teb.MaxVelocity(1) = P.gnc.fsm.v_bypass, params/param_gnc.m) (the GNC engineer, step 5, TR-G4); D99 gates the bypass itself. |
| **Source** | N-07, D99 |
| **Parent** | None |
| **Allocation** | GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission testT7BypassNoTeb, testT7BypassTeb |
| **As built** | **Partly** |
| **Where** | the GNC's bypass (H2; VFH or TEB, gnc spec 2.6) |
| **Note** | The bypass speed skips the speed governor (R-41, card G2-GNC); under D99 no automatic bypass runs with the mower until card G2-GNC covers its footprint. |

| REQ-SYS-136 | The robot shall deliver from its pack at least 1544 Wh at its terminals between SOC 0.95 and SOC 0.275 at the pack's end of life (SOH 0.80). |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-09 Manage the energy |
| **Rationale** | N-02 and D101: the energy that the mission of REQ-SYS-104 uses: 2948.4 Wh x 0.80 x 0.675 = 1592 Wh stored in the window, 1544 Wh at the terminals with the discharge efficiency of 0.97 (P.power.pack.E_Wh 2948.4 Wh, P.power.budget.soh_eol 0.8, eta_dis 0.97, params/param_power.m:45, 259-263) (the BMS and power engineer, step 7, T7-5; the register's re-check, step 14b, found the former 1590 Wh below the 1592 Wh it served, and the 1592 Wh stored rather than delivered). |
| **Source** | N-02, D101 |
| **Parent** | None |
| **Allocation** | PWR, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the energy analysis of power spec 2.1; T-PWR-13 restated for the end SOC of 0.275 at end of life (card G2-PWR) |
| **As built** | **Partly** |
| **Where** | the pack and the BMS's SOC and SOH estimates (power spec 2.1, 2.9) |
| **Note** | The end-of-life test still runs D46's 4 h duty (T-PWR-13), which D101 supersedes (card G2-PWR); the real cells' capacity at end of life is the supplier's data (card G1-04). |

| REQ-SYS-137 | While its key switch is on, the robot shall keep its 24 V, 12 V, 5 V and 3.3 V rails within 10 % of nominal over the pack's whole operating window of 39.0 to 54.6 V, up to 600 W, 300 W and 25 W on the 24 V, 12 V and 5 V rails. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-09 Manage the energy |
| **Rationale** | N-12 and N-13: the controller, the sensors and the implement need their rails across the pack's window (the BMS and power engineer, step 7, T7-7): P.power.lv (rail_ok_tol 0.10, V_nom, v1_min 36 V, P_rated; params/param_power.m:104-114), the twin's design, provisional under D96 (the robot's own LV supplies are card G1-04's data). The 36 V UVLO sits below the pack's 39.0 V minimum (power spec 2.6). |
| **Source** | N-12, N-13, D96 |
| **Parent** | None |
| **Allocation** | PWR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_power_plant lv_steps_T05, lv_uvlo_rail_ok |
| **As built** | **Implemented** |
| **Where** | the LV distribution (P.power.lv; power spec 2.6) |
| **Note** | Implemented in the twin, and the tests pass; on the robot the LV supplies are the supplier's (card G1-04). |

| REQ-SYS-138 | When it charges from its 0.20 reserve, the robot shall reach the full level of REQ-SYS-059 within 3.5 h, inside the charge window of REQ-SYS-060. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-10 Dock and charge |
| **Rationale** | N-02 and N-03: a pack charged overnight from the reserve (D94, D97). 50.4 Ah at the charger's 20 A is 2.5 h of CC plus about 0.4 h of CV, about 3 h, with about 20 % margin (P.power.charger.I_max, P.power.pack.Q_Ah) (the BMS and power engineer, step 7, T7-6); provisional under D96, since the charger's own CV end is unknown. |
| **Source** | N-02, N-03, D94, D96, D97 |
| **Parent** | None |
| **Allocation** | PWR, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: T-PWR-09 restated for the 13S15P pack (t_pack_bms cccv_T09, card G2-PWR); on the robot, a timed charge |
| **As built** | **Partly** |
| **Where** | the charger and the BMS's charge control (P.power.charger, P.power.bms; power spec 2.8, 2.9) |
| **Note** | The twin's charge test still checks the former pack's 100 A (R-29, card G2-PWR); the robot is charged by hand from its stand-alone charger (D97). |

| REQ-SYS-139 | When the operator presses an E-stop button on the robot while it drives at 1.2 m/s on flat farm soil, the robot shall come to rest within 0.6 m. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-11 Stop in an emergency and reset |
| **Rationale** | SG-06 and N-06: the E-stop removes the traction torque (STO) and engages the spring brakes (stop category 0, D86). The bound is S8a's (architecture section 12). Margin: S8a stops in 0.20 m (2026-11-15), a third of the bound, which leaves room for the robot's own STO and brake times, still unknown (REQ-SYS-063, card G1-04). It is safe because the stop starts within REQ-SYS-063's 50 ms of the button and needs no software. |
| **Source** | N-06, SG-06, D86 |
| **Parent** | SG-06 |
| **Allocation** | PWR, DRV, ELC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8a (architecture section 12) |
| **As built** | **Hardware** |
| **Where** | the E-stop chain, the safety relay and the STO wiring (power spec 2.5; ICD section 6) |
| **Note** | S8a passes 6 of 6 in the twin (STO after 0.015 s, stop distance 0.2045 m, 2026-11-15). The robot's E-stop buttons, safety relay and STO wiring are still to be confirmed (as-built record open item 7, card G1-04). |

| REQ-SYS-140 | When the operator throws the remote E-stop switch while the robot drives at 1.2 m/s, the robot shall decelerate at 2.0 +/- 0.1 m/s^2 and come to rest within 0.70 m of the switch. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-11 Stop in an emergency and reset |
| **Rationale** | SG-06 and N-06: the remote E-stop stops the robot from a distance. The values are S8b's (architecture section 12, D33): the mean deceleration over the constant phase 2.0 +/- 0.1 m/s^2 with its 100 ms peak at most 2.4 m/s^2, and 0.70 m from the switch (D33's replay: 0.62 m with the 92 ms worst-case latency). Margin: S8b stops in 0.589 m (2026-11-15), 0.11 m within the bound. It is safe because the bound includes the worst-case radio latency, and 2.0 m/s^2 is well within the traction of farm soil (peak traction 0.66, 6.5 m/s^2). |
| **Source** | N-06, SG-06, D33 |
| **Parent** | SG-06 |
| **Allocation** | COM, MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8b (architecture section 12) |
| **As built** | **Implemented** |
| **Where** | the RC chain and CH12, the MCU's ESTOP decel (electronics spec 2.7.4) |
| **Note** | S8b passes in the twin (mean 2.069 m/s^2, peak 2.189 m/s^2, 0.589 m; 2026-11-15). Not on the robot yet: the firmware generated from the twin is not flashed (card B-01). The values change if a certified wireless E-stop is fitted (E-04; the functional safety engineer, step 17). |

| REQ-SYS-141 | The robot shall keep its RC link, with no gap between valid frames longer than its link time-out, anywhere within 100 m of the operator in line of sight. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-12 Communicate and record |
| **Rationale** | N-04 and N-10: the operator drives and supervises from the field's edge (the electronics and comms engineer, step 8, TR-1). 100 m is a design target from the R1 row field (6 rows of 50 m at 3 m spacing with 6 m headlands, params/param_rowtest.m:11; its farthest point about 70 m from the field's edge), a margin of 1.4; the link time-out is 0.10 s (P.electronics.sbus.link_timeout, params/param_electronics.m:21). Not a safety value: the reaction to a lost link is REQ-SYS-067. |
| **Source** | N-04, N-10 |
| **Parent** | None |
| **Allocation** | COM |
| **Safety** | No |
| **TBD** | the RC link's range inside a greenhouse, by a range test with the robot's own RC equipment; owner the electronics and comms engineer, card G2-COM; by G1-05 (a RID) |
| **Verification** | **Test**, field level, in the field: a range test with the robot's RC equipment (cards G1-04 and G2-COM) |
| **As built** | **Unknown** |
| **Where** | the robot's RC transmitter and receiver (not in the as-built record) |
| **Note** | The robot's RC equipment is not in the as-built record (card G1-04); the twin's row-test radio model cannot produce a link loss (R-12). |

| REQ-SYS-142 | The robot shall deliver each operator input to its controller within 18 ms of the transmitter's frame. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-12 Communicate and record |
| **Rationale** | N-04: the RC latency is part of REQ-SYS-008's 65 ms and of REQ-SYS-009's stop (the electronics and comms engineer, step 8, TR-2): one 14 ms frame (P.rc.frame_period), 3 ms of SBUS bytes (25 x 120 us, sbus.t_byte) and the 1 ms poll (ts.base) make 18 ms, within D37's 28 ms link-to-drive budget with the 10 ms control task (ICD section 7). The robot's radio adds its own delay, unknown (card G1-04). |
| **Source** | N-04, D37 |
| **Parent** | None |
| **Allocation** | COM, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: planned: card G2-COM (the decoder's output time against the transmitter's frame in the twin, RC_Link_Sim to the MCU's SBUS decoder); t_mcu_control testTEL24_ThrottleAndLatency and S1 (the stick-to-ESC latency and the frame to DRV_CMD within 30 ms, which hold the 18 ms inside them) |
| **As built** | **Implemented** |
| **Where** | the RC link from the transmitter to SBUS (RC_Link_Sim) and the MCU's SBUS decoder (electronics spec 2.6, 2.7.2) |
| **Note** | In the twin; the robot's radio delay is unknown (card G1-04). |

| REQ-SYS-143 | If the operator's RC link is lost while the robot drives at 1.0 m/s in RC driving on flat farm soil, then the robot shall come to rest within 0.8 m of the loss, with a mean deceleration of 1.0 +/- 0.1 m/s^2, and park on both brakes. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-14 Respond to lost links |
| **Rationale** | SG-07 and N-10: on a lost link the robot stops under control (category 2, D33) and parks (D71). D100 sets S6's criteria: the controlled stop judged as S8b (the mean over the constant phase 1.0 +/- 0.1 m/s^2, the 100 ms peak at most 1.2 m/s^2), the standstill in the MCU's terms, parked on both brakes, and stopped within 0.8 m of the loss; REQ-SYS-067 holds the 0.13 s to the controlled stop. Margin: D33's replay stops from 1.0 m/s in 0.71 m, 128 ms of detection included, 0.09 m (11 %) within the bound. It is safe because this is the designed category-2 stop of SG-07 and the person protection (SG-01) does not rest on it; the lower deceleration bound keeps the stop from being softer than 0.9 m/s^2. |
| **Source** | N-10, SG-07, D100, D33 |
| **Parent** | SG-07 |
| **Allocation** | MCU, COM, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S6a and S6b (architecture section 12; the criteria of D100 in the script on card G2-INT) |
| **As built** | **Partly** |
| **Where** | the MCU's RC failsafe and controlled stop (fw_mode_manager, the shaper; electronics spec 2.7) |
| **Note** | S6a and S6b (2026-11-15): CTRL_STOP after 0.12 s, and the 100 ms peak of 1.099 m/s^2 passes D100's 1.2. The mean and the stop distance are not scored yet, and the scripts still judge 1.05 m/s^2 and a 1.5 s standstill (card G2-INT). |

| REQ-SYS-144 | While it stands on its spring brakes with no power on a slope of up to 20 deg with its heaviest implement load, the robot shall hold with a brake torque of at least twice the torque that the slope demands. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | SG-05 (every stop ends held on the brakes) and N-08; D86's category-0 stop rests on these brakes. The case is the full-tank sprayer (316 kg) on 20 deg, the steepest slope of the envelope with the heaviest load (the drive engineer, step 9, TD-3): the demand is m g sin(20 deg) = 1060 N, or 95 N m at the 0.090 m sprockets and 1.19 N m at each of the four motors through the 20:1 gearboxes; the four spring brakes give 5 N m each (P.drive.motor_unit.brake_T, params/param_motor_unit.m:45, [EST]): a margin of 4.2. Margin: the bound of 2 covers the spread of the friction coefficient (mu_s 0.40 [EST]), wear and fade between inspections, and the brakes' estimated rating, so half the rated torque still holds. It is safe because the robot then holds the heaviest case with half its brake torque. Set by risk assessment and not yet confirmed against the standards (D103). |
| **Source** | N-08, SG-05, CR-001, D86, D103 |
| **Parent** | SG-05 |
| **Allocation** | DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the holding torque against the demand at 316 kg on 20 deg (drive spec 2.7); t_drive testT08_brakeHold; S2 and S2b (the hold); the slope brake test on the robot (card T-04); the brake datasheet (card G1-04) |
| **As built** | **Hardware** |
| **Where** | the spring brakes, 5 N m per motor (P.drive.motor_unit.brake_T; drive spec 2.7) |
| **Note** | The twin's brakes are estimates (a margin of 4.2), and S2 and S2b hold within 0.05 m. The robot's brake data are still to come (as-built record open item 3, card G1-04). |

| REQ-SYS-145 | While it stands on its spring brakes with no power on a slope of up to 20 deg, the robot shall creep by no more than 0.02 m in 600 s, counted from 5 s after both brakes engage. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | SG-05 and N-08: the second part of the drive engineer's TD-3 (step 9, Table 5), split so that each stays singular. 0.02 m is the creep limit of S2c and S2d (architecture section 12), and 600 s the drive's T8 hold (drive spec 8). Counting from 5 s after both brakes engage leaves out the elastic settle (21.5 mm on 20 deg and 37.3 mm on the 15 deg side slope in the structure's T8_parking, which holds 70 s and measures the drift from 5 to 65 s: 0.000 and 0.079 mm). Margin: the drive's 600 s T8 has been incomplete since D64, whose run gave 0.0004 m/h, and the structure's 60 s window shows under 0.1 mm, 200 times inside the bound. It is safe because a held robot that creeps no more than the park criterion allows stays where its stop left it. |
| **Source** | N-08, SG-05, D64, D100 |
| **Parent** | SG-05 |
| **Allocation** | DRV, STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_drive testT08_brakeHold (a Long test: T06_LONG=1, card G2-DRV); t_structure T8_parking; S2c, S2d (held within 0.02 m) |
| **As built** | **Hardware** |
| **Where** | the spring brakes and the ESCs' park (drive spec 2.7, 2.9) |
| **Note** | T8 holds with under 0.1 mm of drift in the twin; testT08_brakeHold has not run since D64 (a Long test, card G2-DRV). The robot's brakes are estimates until their data (card G1-04). |

| REQ-SYS-146 | When it parks after a stop, the robot shall have both spring brakes engaged within 0.5 s of the park request, with its torque limits held until both brakes confirm. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | SG-05 and N-08, N-11: every stop ends held on the brakes, with no roll while it parks. D71 parks with the torque limits held until both brakes confirm, and the stop monitor trips STOP_FAIL (bit 22, the STO backup) when the brakes do not confirm within 0.5 s of the park request (P.electronics.safety.stopfail_brake_t, params/param_electronics.m:126, [EST]). Margin: in S6a the brakes held 0.04 to 0.07 s after the park request (2026-11-15), 0.43 s within the bound. It is safe because the drive holds zero speed with its limits until the brakes confirm, so the robot does not roll in the interval (D71). |
| **Source** | N-08, N-11, SG-05, D71 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S2b (parked within 1.1 s of CTRL_STOP, both brakes engaged); S6a and S6b (the park); the STOP_FAIL tests of t_mcu_control |
| **As built** | **Hardware** |
| **Where** | the MCU's parking phase and stop monitor (fw_safety_supervisor; electronics spec 2.7.10) and the ESCs' park (drive spec 2.9) |
| **Note** | The twin parks as D71 says: S2b passes, and in S6a the brakes held at 21.72 to 21.75 s after the park request at 21.68 s (2026-11-15). On the robot the ESCs' park and brake confirmation are unknown (R-14, card G1-04). The brake confirmation echoes the command today (F-02), so 'until both brakes confirm' rests on no real confirmation yet (the functional safety engineer, step 17; STATUS 27). |

| REQ-SYS-147 | If its tilt passes its warning limit of 25 deg for 0.5 s, then the robot shall halve its speed limit. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | SG-04 and N-08: the robot slows before its tilt stop at 28 deg (REQ-SYS-076), so that the operator can turn away from a steep face. The values are the MCU's TILT_WARN (bit 14, class C): 25 deg for 0.5 s and half speed (P.electronics.safety.tilt_warn, t_tilt, derate_tilt, params/param_electronics.m:94-100), 25 deg being the 20 deg design slope plus about 1 deg of body pitch and the residual estimate error (D47) (the MCU firmware engineer, step 4, T-MCU-1). Margin: 3 deg below the 28 deg stop and 4 deg above the design envelope. It is safe because the derate halves the speed, and with it the energy of a stop, before the tilt reaches the stop limit. |
| **Source** | N-08, SG-04, D47 |
| **Parent** | SG-04 |
| **Allocation** | MCU, SNS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL33_Tilt, testTiltGateAndDerate, testTEL13_FaultBits (bit 14); T-EL-33 (TILT_WARN at 25 +/- 1 deg) |
| **As built** | **Implemented** |
| **Where** | the MCU safety supervisor, bit 14 TILT_WARN class C (ICD section 6) |
| **Note** | Implemented in the twin, with the IMU level calibration of 2026-10-27. Not on the robot yet (card B-01). One IMU and one MCU (PL b at most, card G2-FSC). |

| REQ-SYS-148 | If its electronics pass their temperature warning limit of 70 deg C for 5 s, then the robot shall halve its speed limit. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | N-10 and N-13: the robot works in heat up to 45 to 50 deg C, and before the temperature stop of REQ-SYS-114 (80 deg C today) it derates so that the operator can bring it to the shade. The values are the MCU's ENCL_TEMP_WARN (bit 20, class C): 70 deg C (P.electronics.safety.T_encl_warn = 343.15 K, params/param_electronics.m:98) and half speed (P.electronics.safety.derate_encl = 0.5); the MCU firmware engineer's MCU-N3 (step 3 of part 1). The 5 s persistence is P.electronics.safety.t_persist (the MCU firmware engineer, step 4, T-MCU-2), provisional under D96. |
| **Source** | N-10, N-13, D96 |
| **Parent** | None |
| **Allocation** | MCU, ELC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control testTEL13_FaultBits (bit 20) |
| **As built** | **Hardware** |
| **Where** | the MCU fault bit 20 ENCL_TEMP_WARN (class C, derate 0.5; ICD section 6) and the enclosure thermal model (electronics spec 2.9.4) |
| **Note** | Implemented in the twin, on the enclosure model of the STM32H743 board, which may not be the robot's (R-59, card G1-04). |

| REQ-SYS-149 | The robot shall have, in every configuration that may move, a static tip-over angle at least 20 deg above the slope of its envelope, forward, rearward and laterally. |
| --- | --- |
| **Kind** | Performance |
| **Function** | F-15 Stop and hold, and stay stable on slopes |
| **Rationale** | SG-04 and N-08 (the structure engineer, step 10, section 6, and step 19). The tip angles are judged on the sharp-edge track end, the floor that structure spec 2.11 takes (the stricter reading; the lead systems engineer's point 2). The tightest cases that may move are the full-tank sprayer facing uphill (38.2 deg at 18.5 deg: a margin of 19.7 deg, climbing as REQ-SYS-129 allows; 40.4 deg on the model geometry), the sprayer forward (38.7 deg: 20.2 deg), the sprayer laterally to the contact line (38.7 deg at 15 deg: 23.7 deg) and no implement forward (47.4 deg at 20 deg: 27.4 deg) (structure spec 6.3). Two exclusions: the raised tiller, outside this phase (D76, card G2-TIL; facing uphill it tips backward at 16.4 deg static and 11.8 deg with the sharp-edge track end (the climbing torque lowers it further, not yet quantified)), and the raised mower deck (D81). The mower's deck-down value of 75.7 deg needs both gauge wheels on the ground; with a wheel lifted it is 25.4 deg (20.7 deg with the sharp edge), which is not shown to stay out of the envelope: evidence that both wheels stay down over the basins and berms, or a measure, is pending on card G2-STR (the lead systems engineer's point 23), and the descent table's mower rows rest on the same assumption. Margin: not met by 0.3 deg for the full-tank sprayer facing uphill (19.7 deg against the 20 deg bound), an open mark for the owner: it is re-checked with the weighed masses on card G1-04, and if it still falls short, the owner chooses on card G2-SPR, on the functional safety engineer's advice, between an 18.0 deg cap in CR-001 and accepting 19.7 deg (the lead systems engineer's point 2). Every other configuration that may move keeps at least 20.2 deg. For those it is safe because the 20 deg covers the pitch that a brakes-locked stop adds, whose own criterion, the peak pitch within half the rotation to the static tip (D82 (b)), is REQ-SYS-075's. |
| **Source** | N-08, SG-04, D76, D81, D82, CR-001 |
| **Parent** | SG-04 |
| **Allocation** | STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the static tip-over table of structure spec 6.3; t_structure T3_static_pitch_implements |
| **As built** | **Partly** |
| **Where** | the mass properties and the geometry (P.structure.mp, P.structure.geom; structure spec 6.2, 6.3) |
| **Note** | The values rest on the estimated masses and centres of gravity of the implements and of the 1 kW units (D95) until the robot is weighed (card G1-04). Not met on the sharp-edge floor by 0.3 deg for the full-tank sprayer facing uphill, a RID for G1-05, re-checked on card G1-04 (the lead systems engineer's point 2); the deck-down mower with a lifted gauge wheel is not shown (card G2-STR), and the functional safety engineer adds the cause to the tip-over hazard on card G2-FSC. |

| REQ-SYS-150 | The robot shall run its MCU firmware with every task within its period and at most 50 % of the controller's CPU time and 50 % of its flash (1 MB on the STM32H743 development target). |
| --- | --- |
| **Kind** | Performance |
| **Function** | None: system-wide |
| **Rationale** | N-16 and charter goal 5: the firmware generated from the twin runs on the robot's controller. The build plan's acceptance of T17 sets the CPU load below 50 % at 480 MHz and the flash below 1 MB (P.electronics.target.cpu_load_max, flash_max, params/param_electronics.m:324-325), which leaves room for the tasks' worst cases and for later functions. Provisional under D96 until card G1-04 names the controller board: the supplier's board is the baseline, and the H743 the development target (as-built record section 4, open item 5). |
| **Source** | N-16, D96 |
| **Parent** | None |
| **Allocation** | MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, PIL/HIL level, on the bench: T17 (the build and the profile on the controller) |
| **As built** | **Not implemented** |
| **Where** | the MCU firmware model (Shaddad_MCU) and its target (P.electronics.target) |
| **Note** | No code has been built: the STM32 tools are not installed, the robot's controller board is still to be confirmed (R-59, card G1-04), and the firmware is not flashed (card B-01). |

### 6.2 Physical

**Kind:** the mass, the size, the pack, the motors and the implements as built (11 requirements, one of them a safety requirement.)

| REQ-SYS-151 | Where the mower is fitted, the robot shall cut a 1.00 m swath with its blades at 2800 rpm and its deck height set between 0.03 m and 0.12 m. |
| --- | --- |
| **Kind** | Physical |
| **Function** | F-08 Work the implement |
| **Rationale** | N-01: the mowing of this phase (D76). The values are D9's (1.00 m, 2800 rpm) and the deck's range (P.structure.impl.mower.w, w_nom = 293.2 rad/s, h_range, params/param_structure.m:64-66); the greenhouse plan's swath is the same width (P.gnc.cov.gh_swath). |
| **Source** | N-01, D9, D76 |
| **Parent** | None |
| **Allocation** | STR, PWR, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the mower's data (card G1-04); S4 (the swath in the coverage) |
| **As built** | **Hardware** |
| **Where** | the mower's mechanics (P.structure.impl.mower) and drive (P.power.impl.mower) |
| **Note** | The mower's deck, motor, power and mass are estimates; its data are on the supplier request (card G1-04). |

| REQ-SYS-152 | The robot shall carry the delivered 13S15P lithium-ion NMC pack of 195 Molicel P42A 21700 cells, sealed in its aluminium case (D94): 46.8 V nominal, 39.0 to 54.6 V in operation, 63 Ah and 2.95 kWh (46.8 V x 63 Ah). |
| --- | --- |
| **Kind** | Physical |
| **Function** | F-09 Manage the energy |
| **Rationale** | N-02 and N-17: the pack as delivered (as-built record section 2: Molicel P42A cells, 13 in series by 15 in parallel, the 393 x 358.5 x 87 mm case [DATA]), sealed by D94 (P.power.pack.Ns, Np, Q_Ah, V_nom, E_Wh, params/param_power.m:19, 44-46) (the BMS and power engineer, step 7, T7-1). Its energy bounds the missions of REQ-SYS-104 and REQ-SYS-136. |
| **Source** | N-02, N-17, D94 |
| **Parent** | None |
| **Allocation** | PWR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's data sheet and the label (card G1-04); the twin's pack (power spec 6.2 and 6.3) |
| **As built** | **Hardware** |
| **Where** | the pack (P.power.pack; power spec 2 and 6) |
| **Note** | The cell data, the mass and the box are [DATA]; the cell tables are [EST] (power spec 2.2). |

| REQ-SYS-153 | The robot shall manage its pack with the delivered 13S BMS, rated for 120 to 150 A continuous and 200 to 300 A peak (150 A and 250 A in the twin), reporting on CAN at 500 kbit/s. |
| --- | --- |
| **Kind** | Physical |
| **Function** | F-09 Manage the energy |
| **Rationale** | N-13 and N-17: the BMS as delivered (as-built record section 2: 120 to 150 A continuous and 200 to 300 A peak [DATA]; the twin's P.power.bms.I_dis_cont 150 A and I_dis_peak 250 A), the baseline under D96, provisionally (the BMS and power engineer, step 7, T7-2). |
| **Source** | N-13, N-17, D96 |
| **Parent** | None |
| **Allocation** | BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's data (card G1-04) |
| **As built** | **Hardware** |
| **Where** | the bought 13S BMS (as-built record section 2); the twin's G4 BMS is a variant (D96) |
| **Note** | Its thresholds and frames are unknown (R-15, card G1-04). |

| REQ-SYS-154 | The robot shall fit within 1.00 m in length, 0.78 m in width and 0.57 m in height without its sensor mast and with no implement, within 1.70 m by 1.05 m with the mower, and within 1.63 m in length with the tiller. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-01 and N-17: the platform as delivered (XMachines MAX; as-built record section 1: the width of 780 mm and the height of 570 mm [DATA], the body length of 1.00 m [EST]). The greenhouse plan places its lanes from this width (c_in = 0.40 m today, 0.50 m by D106, pending on card G2-GNC; REQ-SYS-087), and the collision footprint is the same box (P.structure.geom.L_overall, W_overall, H_body, params/param_structure.m:30). With the implements (the structure engineer, step 10, section 6; the implement boxes, S.impl): 1.70 m long and 1.05 m wide with the mower, 1.63 m long with the tiller; the sensor mast is 1.05 m [EST]. |
| **Source** | N-01, N-17 |
| **Parent** | None |
| **Allocation** | STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the robot measured (card G1-04); the twin's geometry (P.structure.geom) |
| **As built** | **Hardware** |
| **Where** | the body (P.structure.geom; structure spec 6.1) |
| **Note** | The width and the height are the spec sheet's; the length is estimated from the side view (as-built record section 1). |

| REQ-SYS-155 | The robot shall have a mass and a centre of gravity of at most 226 kg and 0.267 m above the track bottom with no implement, 291 kg and 0.241 m with the mower, 311 kg and 0.276 m with the tiller, and 316 kg and 0.375 m with the sprayer and a full 40 L tank. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | SG-04 and N-08: the descent table (D74, D83), the tip-over checks (REQ-SYS-075) and the sprayer's 18.5 deg limit (CR-001) were derived for these masses (P.structure.mp.m0 per configuration, params/param_structure.m): 220 kg as delivered [DATA] plus 6 kg for the 1 kW motor units of D95, the mower 65 kg [EST], and the sprayer 50 kg dry with its 40 L tank [EST]. Margin: the margins are in the analyses that use the masses (the full-tank sprayer keeps about 0.06 m/s within the park criterion at 0.75 deg, D83). It is safe because a heavier robot needs the descent table derived again, so the masses are its bound. The heights of the centre of gravity and the tiller's mass are the structure engineer's (step 10, section 6; P.structure.mp, params/param_structure.m:35-99). |
| **Source** | N-08, N-17, SG-04, D95, CR-001 |
| **Parent** | SG-04 |
| **Allocation** | STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the robot weighed with each implement (card G1-04); the twin's mass table (structure spec 6.2) |
| **As built** | **Hardware** |
| **Where** | the mass table (P.structure.mass.components, P.structure.mp; structure spec 6.2) |
| **Note** | The robot's 220 kg is the spec sheet's, and the implements' masses are estimates (card G1-04). With the as-built 750 W motors the robot is 220 kg, below the bound, but its centre of gravity with no implement is 0.268 m, 1.3 mm above the 0.267 m bound (P.structure.mp.rcg0, motor_movematic750; structure spec 6.3), about 0.1 deg off the static tip angles, inside every margin. The bound stays, as D95 makes the 1 kW robot the baseline; the descent table and the t_drive runs for the 750 W robot are pending on cards G2-STR and G2-DRV, to be done before the robot moves on its 750 W motors with firmware built from the twin (D95's flashing condition) (the lead systems engineer's point 24). |

| REQ-SYS-156 | The robot shall have a ground clearance of at least 0.12 m. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-01: palm groves have berms, and greenhouse floors are uneven (charter section 6). The value is the platform's (as-built record section 1, [EST]; P.structure.geom.groundClearance, params/param_structure.m:31). The 0.15 m irrigation berms of the palm basins and the 0.25 m greenhouse beds (D54) are higher than the clearance, so the missions keep the robot off them (the structure engineer, step 10, section 6; REQ-SYS-168), which REQ-SYS-207 makes binding. |
| **Source** | N-01, N-17, D54 |
| **Parent** | None |
| **Allocation** | STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the robot measured (card G1-04) |
| **As built** | **Hardware** |
| **Where** | the body (P.structure.geom.groundClearance) |
| **Note** | An estimate from the drawings (as-built record section 1). |

| REQ-SYS-157 | The robot shall exert a mean ground pressure of at most 16 kPa with any implement. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-01 and N-08: the soils' strength in the twin assumes about 16 kPa (Wong; structure spec 6.6). By m g / (2 b L) from P (structure spec 6.1): 11.3 kPa with no implement, 13.3 with the mower on its gauge wheels, 15.6 with the tiller and 15.9 with the full-tank sprayer (the structure engineer, step 10, section 6). |
| **Source** | N-01, N-08 |
| **Parent** | None |
| **Allocation** | STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: m g / (2 b L) from the masses and the track geometry (structure spec 6.1) |
| **As built** | **Hardware** |
| **Where** | the tracks and the mass table (P.structure.geom, P.structure.mp) |
| **Note** | From the estimated masses; the full-tank sprayer is 0.1 kPa inside the bound. |

| REQ-SYS-158 | The robot shall drive each track through two 48 V BLDC traction motors, one at the front and one at the rear sprocket, each through a 20:1 planetary gearbox with its own spring-applied holding brake, and through one dual-channel ESC per track on CAN1. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-17: the drive as delivered (as-built record section 3: the count, the layout and the dual ESC per side are [DATA], 2026-10-27; the ratio is [EST]), the baseline under D96 (the drive engineer, step 9, TD-5). |
| **Source** | N-17, D96 |
| **Parent** | None |
| **Allocation** | DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's data (card G1-04) |
| **As built** | **Hardware** |
| **Where** | the traction drive (drive spec 0, 1 and 2.9) |
| **Note** | As delivered; the gear ratio is an estimate until the supplier's data (card G1-04). |

| REQ-SYS-159 | The robot shall have traction motors each rated at least 1.0 kW and 3.18 N m continuously at 3000 rpm and 48 V, and 9.55 N m for 30 s. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-08 and D95: the baseline is the 1 kW unit, the planned upgrade of the robot's four 750 W motors (P.drive.motor.T_cont, T_peak, params/param_drive.m:41; param_motor_unit.m:31-40 [EST]; CR-004), two per side: 6.36 N m continuous and 19.1 N m peak per side (the drive engineer, step 9, TD-6). At 45 deg C the 750 W motor gives about 2.15 N m continuous, below the turn on the 20 deg slope (2.85 N m per motor), while the 1 kW unit keeps a margin of 1.58 on the wet-loam RMS (as-built record section 3). |
| **Source** | N-08, N-17, D95, CR-004 |
| **Parent** | None |
| **Allocation** | DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the sizing of the as-built record section 3, then the motor's data sheet (card G1-04) |
| **As built** | **Partly** |
| **Where** | the traction drive (P.drive.motor, param_motor_unit; drive spec 2.2) |
| **Note** | In the twin by default (D95); on the robot a recorded deviation until the upgrade (CR-004): its four 750 W Movematic motors give 2.39 N m continuous and 7.16 N m peak each (4.78 and 14.32 N m per side). The 1 kW unit is still to be chosen, so its values are estimates (D95). |

| REQ-SYS-160 | The robot shall have ESC channels that each supply at least 27 A rms continuously and 65 A rms for 30 s. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-08 and D95: the current that the 1 kW unit needs per channel, 55 A DC (the drive engineer, step 9, TD-6 and finding 2; param_motor_unit.m:31-40 [EST]). |
| **Source** | N-08, D95 |
| **Parent** | None |
| **Allocation** | DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the ESC datasheet (card G1-04); the drive bench on card G2-DRV |
| **As built** | **Unknown** |
| **Where** | the delivered dual-channel ESCs (as-built record section 3) |
| **Note** | The as-built ESC is estimated at only 20 A rms continuous and 50 A rms peak per channel (the drive engineer's finding 2), so D95's upgrade may need new ESCs; the ratings are on the supplier request (card G1-04). |

| REQ-SYS-161 | The robot shall have gearboxes that each carry at their output at least 120 N m continuously, 180 N m in acceleration and 240 N m in an emergency stop. |
| --- | --- |
| **Kind** | Physical |
| **Function** | None: system-wide |
| **Rationale** | N-08 and D95: D51's rule, the gearbox's T2B at least the motor's peak through the gear (172 N m with the 1 kW unit) (the drive engineer, step 9, TD-6). T23 measures 343 N m per side, carried by the two gearboxes of a side. |
| **Source** | N-08, D51, D95 |
| **Parent** | None |
| **Allocation** | DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the gearbox datasheet (card G1-04); t_drive T23 (the torques) |
| **As built** | **Hardware** |
| **Where** | the planetary gearboxes (P.drive.gear; drive spec 2.8) |
| **Note** | The gearboxes' ratings are estimates (T2N 240 and T2B 360 N m per side; the 750 W option 240 and 280 N m) until the supplier's data (card G1-04). |

### 6.3 Environment

**Kind:** the ambient temperature, the sun, the dust, the water, the ground and the sites (11 requirements, one of them a safety requirement.)

| REQ-SYS-162 | While it works at 45 deg C ambient in full sun, the robot shall keep its hottest cell at or below 55 deg C. |
| --- | --- |
| **Kind** | Environment |
| **Function** | F-09 Manage the energy |
| **Rationale** | N-13: the pack's label limits it to 55 deg C, the supplier's limit (D94), and S3 checks T_cell_max at most 328.15 K at 45 deg C (architecture section 12). The BMS cut-off above it stays the safety function (SF-17, D94), so this limit is a working one. |
| **Source** | N-13, D94 |
| **Parent** | None |
| **Allocation** | PWR, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S3 (architecture section 12) |
| **As built** | **Unknown** |
| **Where** | the pack thermal model (power spec 2.3) |
| **Note** | No S3 run on the sealed default pack of D94 is on file; the sealed run of 2026-11-15 charged in full sun at 45 deg C. The power spec's hand check of the sealed pack in full sun settles the box about 12 K above ambient, near 57 deg C at 45 deg C, above the 55 deg C: a known risk. The interim measure is the operating rule that the project owner adopted on 2026-11-18 (step 22), beside D94's charging procedure: in full sun, one mission from a cool start, then park in the shade, until the S3 run on the sealed pack (G2-INT). A sun shield is raised only if the run confirms the shortfall (the lead systems engineer's point 5; cards G2-PWR, G2-INT). |

| REQ-SYS-163 | While it is stored with its key switch off, the robot shall keep every cell at or above 3.0 V for at least 90 days from a storage charge of 40 to 60 %, in the shade (D94, D105). |
| --- | --- |
| **Kind** | Environment |
| **Function** | F-09 Manage the energy |
| **Rationale** | SG-10 and N-13: the pack stays safe in storage, as a cell drawn deep below its cut-off can short on recharge (the BMS and power engineer, step 7, T7-8). D105 sets the values: every cell at or above 3.0 V for at least 90 days from a storage charge of 40 to 60 %, in the shade (D94). Margin: 3.0 V is 0.5 V above the cells' 2.50 V absolute minimum. It is safe with D105's fallback until the bought BMS's sleep draw is shown to meet the value (card G1-04): storage beyond 30 days with the main connector (SB120) unplugged and a monthly check. The twin's 2 W BMS supply, drawn upstream of K1, would empty a pack stored at half charge in about 31 days; a draw of 0.5 W or less would give about 110 days to 3.0 V from 0.5. |
| **Source** | N-13, SG-10, D94, D105 |
| **Parent** | SG-10 |
| **Allocation** | BMS, PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the storage draw against the energy from SOC 0.4 to 3.0 V per cell; the inspection of the storage procedure (D105, with the charging procedure of D94; card G2-PWR) |
| **As built** | **Unknown** |
| **Where** | the bought BMS's sleep draw and the storage procedure |
| **Note** | The bought BMS's draw is not recorded (card G1-04), and the twin's 2 W draw would not meet the value (about 31 days from half charge). Until the draw is shown, D105's fallback applies: the main connector (SB120) unplugged for storage beyond 30 days, with a monthly check (the storage procedure, card G2-PWR). |

| REQ-SYS-164 | The robot shall work in ambient temperatures from its lower ambient limit up to 45 deg C, and up to 50 deg C in its hot case, in full sun of 1000 W/m^2. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-13: the sites are in the Al-Kharj summer, 29 to 47 deg C over the day ((E3) of the integration spec). The design ambient is 45 deg C (P.world.env.T_amb = 318.15 K, params/param_world.m:41) with a hot case of 50 deg C (P.scen.T_amb_hot = 323.15 K) and full sun (P.world.env.G_solar = 1000 W/m^2), as S3 runs them (architecture section 12). The structure engineer's ambient (step 10, section 6: up to the 45 deg C design ambient with 1000 W/m^2, and the twin's daily range of 302 to 320 K) lies within this one; the hot case of 50 deg C, which the electronics and the drive proposals use (REQ-SYS-165, REQ-SYS-166), is kept as the stricter. |
| **Source** | N-13 |
| **Parent** | None |
| **Allocation** | PWR, BMS, ELC, DRV |
| **Safety** | No |
| **TBD** | [TBD] the lower ambient limit (the row test runs down to -10 deg C); owner the drive engineer, with the BMS and power engineer for the pack; by G1-05 (a RID) |
| **Verification** | **Test**, system level, in simulation: S3 at 45 deg C and its hot case at 50 deg C (architecture section 12); the thermal benches of the power and drive specs |
| **As built** | **Partly** |
| **Where** | the thermal models of the pack, the drive and the enclosure (power spec 2.3, drive spec 2.6, electronics spec 2.9.4) |
| **Note** | The twin models the heat. S3 has not run on the current robot with the sealed pack of D94. The lower ambient limit is the drive engineer's TBD (step 9, open point 3). The power spec's hand check of the sealed pack in full sun gives about 62 deg C at the 50 deg C hot case, above the BMS's over-temperature cut of 60 deg C (P.power.bms.T_ot): a known risk. The interim measure is the operating rule that the project owner adopted on 2026-11-18 (step 22), as for REQ-SYS-162: in full sun, one mission from a cool start, then park in the shade, until the S3 run on the sealed pack (G2-INT). The rule is the lead systems engineer's point 5 (cards G2-PWR, G2-INT). |

| REQ-SYS-165 | The robot shall keep its electronics working with no thermal stop at an ambient temperature of up to 50 deg C with 1000 W/m^2 of sun on the enclosure, through a whole mowing mission. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-13: the hot case of REQ-SYS-164 (the electronics and comms engineer, step 8, TR-5): P.power.thermal.T_amb_max 323.15 K; T-EL-17. The twin gives 55.3 deg C of box air with the fans, against the 80 deg C stop of REQ-SYS-114 (P.electronics.safety.T_encl_crit), 24.7 K below it, and 71.3 deg C with the fans failed; on the stricter plain-box reading 56.5 deg C and 76 deg C, 4 K below the stop (electronics spec 2.9.4; the box's area is settled on card G2-VAL with the robot's box from G1-04). |
| **Source** | N-13 |
| **Parent** | None |
| **Allocation** | ELC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: T-EL-17 (the enclosure at 50 deg C and 1000 W/m^2); the robot's box tested on cards G1-04 and G2-VAL |
| **As built** | **Partly** |
| **Where** | the development target's enclosure model (electronics spec 2.9.4) |
| **Note** | Shown in the twin for the development target's box (D96); the robot's box is to test (cards G1-04, G2-VAL). With a typical 55 deg C safety relay in the box, REQ-SYS-165, -114 and -205 cannot all hold (about -0.3 K on the twin's box air, -1.5 K on the plain-box reading): an open safety value for the owner, decided on card G1-04 once the delivered relay's rating and place are known; the lead systems engineer recommends parts rated for the 80 deg C stop or the relay and the STO interface placed outside the box, as both keep the mission in the heat (the lead systems engineer's point 1). |

| REQ-SYS-166 | The robot shall deliver the continuous tractive force of REQ-SYS-128 in ambient air of up to 50 deg C in full sun with its motor windings at or below 140 deg C and its ESC heat sinks at or below 85 deg C. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-13: the drive in the hot case (the drive engineer, step 9, TD-4): P.power.thermal.T_amb_max 323.15 K, P.drive.thermal.dT_solar 10 K, and the derating starts P.drive.esc.T_mot_derate and T_esc_derate (params/param_drive.m:79-80), on class-H windings. Estimated at the rated point at 50 deg C plus 10 K of sun: the winding at 120 deg C and the ESC at about 64 deg C with the 1 kW unit, and the winding at 114 deg C with the 750 W motor. |
| **Source** | N-13, D95 |
| **Parent** | None |
| **Allocation** | DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_drive T5 re-baselined at the motors' rating (card G2-DRV, R-57); the hot row test (up to 55 deg C) |
| **As built** | **Partly** |
| **Where** | the drive's thermal network (drive spec 2.6) |
| **Note** | T5 has not run at the right rating (R-57, card G2-DRV). The as-built record's 2.15 N m at 45 deg C for the 750 W motor disagrees with the thermal model, which keeps a 109 deg C winding at 2.39 N m (the drive engineer's open point 4), until the Movematic datasheet arrives (card G1-04). |

| REQ-SYS-167 | The robot shall drive, turn and mow on farm soil, dry sand, wet loam, a compacted greenhouse floor and dry concrete or asphalt, with a peak traction of 0.50 to 0.90 and a rolling resistance of 0.035 to 0.12 [EST]. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-01 and N-08: the soils of the sites (the soil table of the structure spec 6.6, P.world.soilTable, params/param_world.m): dry sand, farm soil (sandy loam, the default), wet or irrigated loam in the palm basins, the greenhouse floor, and the concrete or asphalt paths. The traction and the rolling resistance are the soil table's (P.world.soilTable, params/param_world.m:48-54; Wong), estimates that card T-04 calibrates (the structure engineer, step 10, section 6). |
| **Source** | N-01, N-08 |
| **Parent** | None |
| **Allocation** | STR, DRV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: the structure benches per soil; S7 on wet loam (architecture section 12); the field turning test (card G2-VAL); T4 per soil; card T-04 on the robot |
| **As built** | **Partly** |
| **Where** | the track-soil contact (P.structure.contact, P.world.soilTable; structure spec 6.4 to 6.6) |
| **Note** | The twin models the five soils; the turning model is checked against Wong's theory on farm soil only, with a known gap (T10, D100), until the field turning test. |

| REQ-SYS-168 | The robot shall work on the terrain of the palm groves and greenhouses of this phase: field grades up to 0.5 %, a roughness of 1.5 cm RMS, the palm basins and posts of the site models and the 20 deg design ramp [EST until a site survey]. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-01 and N-08: the terrain of the sites (the structure engineer, step 10, section 6): world_scn_palm_farm, world_scn_greenhouse and param_world.m (structure spec 6.9), estimates until a site survey. The 0.15 m irrigation berms of the palm basins and the 0.25 m greenhouse beds (D54) are higher than the 0.12 m ground clearance (REQ-SYS-156), so the missions keep the robot off them (REQ-SYS-207). |
| **Source** | N-01, N-08, D54 |
| **Parent** | None |
| **Allocation** | STR, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: the scenarios on the site models (S2, S4, S5) |
| **As built** | **Implemented** |
| **Where** | the worlds of P.world (structure spec 6.9) |
| **Note** | The site models are estimates until a site survey (F-13). |

| REQ-SYS-169 | The robot shall work in greenhouse houses of 9.6 m by 37.5 m with wall posts, and in palm groves planted on an 8 m by 8 m grid [EST until a site survey]. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-01: the sites of this phase (charter section 6, D76). The twin's greenhouse is the 9.6 x 37.5 m house of S4 with its posts (world 4), and its palm grove the 8 x 8 m grid of S5 (world 3; P.gnc.cov.palm_spacing, params/param_gnc.m). |
| **Source** | N-01, D76 |
| **Parent** | None |
| **Allocation** | GNC, CMP, PER |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S4 and S5 (architecture section 12) |
| **As built** | **Implemented** |
| **Where** | the worlds of param_world (3 palm_farm, 4 greenhouse) |
| **Note** | The worlds model these sites; a real site's layout is planned from its own survey (F-13). |

| REQ-SYS-170 | The robot shall work greenhouse rows whose ends leave a headland of 3.0 m or more, with the mower fitted. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-01: the lane-end turns need room (the GNC engineer, step 5, TR-G6): the turn radius of 0.81 m, the deck's 1.20 m reach ahead of the reference point and the turn clearance of REQ-SYS-088, 1.1 m at 0.5 m/s, make 3.11 m, so a 3.0 m headland needs the turn at about 0.37 m/s or less (a 1.0 m clearance), which the lead systems engineer's ruling 3 leaves to card G2-GNC's design (pending; the lead systems engineer's point 29). The S4 house is to be checked against it, as the end-wall holds of STATUS 18 suggest that it is shorter (the GNC engineer's finding 9.8). |
| **Source** | N-01 |
| **Parent** | None |
| **Allocation** | CMP, GNC, STR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in simulation: the site's plan against the headland; S4 |
| **As built** | **Unknown** |
| **Where** | the greenhouse world (P.world, world 4) and the site's survey |
| **Note** | The S4 house's headland is not checked yet (cards G2-GNC and G2-INT). |

| REQ-SYS-171 | The robot shall keep out the dust and the water of its sites from its electronics enclosure to IP65 or better (IEC 60529:1989+AMD1:1999+AMD2:2013). |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-13: greenhouse irrigation and spraying, and the dust of dry sand and of the palm groves, reach the electronics (the electronics and comms engineer, step 8, TR-6). IP65 is the development target's design (electronics spec 2.2), and IEC 60529 is listed at its edition in the register's standards (STD-31, D103). The pack's and the motors' degree is REQ-SYS-172. |
| **Source** | N-13, D103 |
| **Parent** | None |
| **Allocation** | ELC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's data (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the electronics box |
| **Note** | The delivered box's rating is unknown (card G1-04); the twin does not model ingress. |

| REQ-SYS-172 | The robot shall keep out the dust and the water of its sites from its pack and its traction motors to the ingress protection degree [TBD]. |
| --- | --- |
| **Kind** | Environment |
| **Function** | None: system-wide |
| **Rationale** | N-13: as REQ-SYS-171, for the pack case and the motors; the charger is IP54 [DATA]. No owner proposed a degree in part 2, and the delivered parts' ratings are card G1-04's data. |
| **Source** | N-13 |
| **Parent** | None |
| **Allocation** | PWR, DRV |
| **Safety** | No |
| **TBD** | the ingress protection degree of the pack case and the traction motors (IEC 60529); owner the BMS and power engineer with the drive engineer; by G1-05 (a RID) |
| **Verification** | **Inspection**, system level, in the field: the supplier's data (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the pack case and the motors |
| **Note** | Not on file for the robot; the twin does not model ingress. |

### 6.4 Interface

**Kind:** the operator's RC and console, the dock, the implements, and the links between the robot's parts that the operator or the dock sees (10 requirements, 2 of them safety requirements.)

| REQ-SYS-173 | The robot shall take the operator's commands from an SBUS receiver at 100000 baud, with 16 channels in frames of 14 ms, mapped as ICD section 7 gives. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-02 Drive by remote control |
| **Rationale** | N-04: the operator drives by RC (TX16S, Mode 2). The MCU reads SBUS on USART2 at 100000 baud (the electronics spec's key decisions; P.electronics.sbus.baud, params/param_electronics.m:20): 16 channels in 25-byte frames, at the normal 14 ms frame period (P.rc.frame_period, params/param_rc.m:6; 7 ms is the receiver's high-speed option), with the channel map of ICD section 7 (CH12 the remote E-stop). |
| **Source** | N-04 |
| **Parent** | None |
| **Allocation** | COM, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: the RC chain tests and the RC row test R1; S1 (the latencies) |
| **As built** | **Implemented** |
| **Where** | the RC link from the transmitter to SBUS (RC_Link_Sim) and the MCU's SBUS decoder (electronics spec 2.7.2) |
| **Note** | The twin's RC chain runs from the transmitter to SBUS; on the robot the firmware is not flashed (card B-01). |

| REQ-SYS-174 | The robot shall have its RC receiver set to failsafe presets of the sticks at centre, the mode switch at HOLD, the arm switch off and the remote stop at RUN, with the remote stop on its own 2-position switch (CH12). |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-02 Drive by remote control |
| **Rationale** | SG-08 and N-11: a receiver in failsafe must not arm or start the robot, nor latch an emergency stop (D34) (the electronics and comms engineer, step 8, TR-3): the presets of P.rc.failsafe_raw (params/param_rc.m:24-30) and ICD section 7. Margin: with these presets a failsafe output reads as a held robot even where the MCU would take it, while the MCU takes the mode, arm, reset and mission commands only from valid frames (REQ-SYS-011). It is safe because the presets are the robot's stopped state. |
| **Source** | N-04, N-11, SG-08, D34 |
| **Parent** | SG-08 |
| **Allocation** | COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in simulation: the receiver's settings on the robot (cards G1-04, G2-COM); t_rc_link failsafe_modes |
| **As built** | **Partly** |
| **Where** | the twin's receiver presets (P.rc.failsafe_raw) |
| **Note** | Set in the twin; the robot's receiver and its presets are not in the as-built record (card G1-04). Another preset on the mode switch lets the robot re-enter manual by itself after a link loss (R-35, REQ-SYS-011). |

| REQ-SYS-175 | The robot shall take its RTK corrections from the site's correction source and use for an RTK fixed position only corrections no older than 10 s. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-04 Know where the robot is |
| **Rationale** | N-09: the RTK correction source is outside the system (the register's statement of the system). The GNSS receivers hold RTK fixed with corrections up to 10 s old and float up to 30 s (P.sensors.gnss.corrAgeFixLimit, corrAgeFloatLimit, params/param_sensors.m:82); the companion's NTRIP client reconnects after 10 s (P.electronics.ntrip.reconnect_t). |
| **Source** | N-09 |
| **Parent** | None |
| **Allocation** | SNS, CMP, COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: the GNSS model's correction-age tests (sensors spec section 9) |
| **As built** | **Hardware** |
| **Where** | the GNSS receivers (P.sensors.gnss) and the companion's NTRIP client (P.electronics.ntrip) |
| **Note** | The GNSS receivers and the modem are on the phase-2 mast, which is not fitted yet (as-built record section 1). |

| REQ-SYS-176 | The robot shall exchange IMPL_CMD (0x140) and IMPL_FB (0x141, with the implement's type) every 50 ms with its implement on CAN1 (classic CAN 2.0B, 500 kbit/s, 11-bit identifiers), and read the implement's type from the coding plug on its IMPL_ID inputs. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-08 Work the implement |
| **Rationale** | N-01 and N-12: the maintainer fits and changes implements. D74 reads the implement from the IMPL_ID coding plug (DI bits 8 and 9, P.electronics.di.impl_id0, impl_id1, params/param_electronics.m:156), and D9 commands the rotor over CAN (IMPL_CMD and IMPL_FB, ICD section 4), with the pump, the nozzles and the lift from the MCU's outputs. An unknown implement takes the most restrictive limits (REQ-SYS-100). The frames and the rates are ICD sections 4 and 6 (P.electronics.can.can1_bitrate, can.id.impl_cmd, can.id.impl_fb, params/param_electronics.m:145-148; ts.comm 0.05 s, :17; di.impl_id0 and impl_id1, :156) (the electronics and comms engineer, step 8, TR-8); the time-outs on both sides are REQ-SYS-101's. |
| **Source** | N-01, N-12, D9, D74 |
| **Parent** | None |
| **Allocation** | MCU, COM, PWR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_mcu_control (the IMPL_ID decoding and the implement frames); the implement benches of the power spec |
| **As built** | **Partly** |
| **Where** | the MCU's implement IO and the implement ECU (electronics spec 2.7.12; power spec 2.7) |
| **Note** | In the twin. On the robot the implements' frames and the coding plug are not confirmed (the sprayer board SPR V1.3 is the supplier's; card G1-04). |

| REQ-SYS-177 | Where a dock is used, the robot shall stop on it within 0.05 m of its charging position and 3 deg of its heading, at no more than 0.05 m/s at contact, after an approach at up to 0.2 m/s (REQ-SYS-058). |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-10 Dock and charge |
| **Rationale** | N-03: the twin's dock (D97: the robot has no dock in this phase, and F-10 is verified in the twin). The tolerances are the dock's (P.world.dock.tol_xy = 0.05 m, tol_yaw = 3 deg, v_max = 0.05 m/s, params/param_world.m:178); the GNC docks tighter, within 0.03 m and 2 deg (P.gnc.fsm.dock_tol_xy, dock_tol_yaw, params/param_gnc.m:123). |
| **Source** | N-03, D97 |
| **Parent** | None |
| **Allocation** | GNC, NAV, PWR |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: the GNC mission tests of the dock approach (t_gnc_mission, T12: the pose; the contact speed of at most 0.05 m/s planned as a T12 criterion on card G2-GNC) |
| **As built** | **Partly** |
| **Where** | the GNC's dock approach (gnc spec 2.8) and the twin's dock (P.world.dock) |
| **Note** | In the twin only (D97); a dock for the robot comes later, by change request. T12 checks the pose only; the contact speed is not shown (card G2-GNC). |

| REQ-SYS-178 | The robot shall be charged from its delivered stand-alone CC-CV charger, 54.6 V and 20 A from 100 to 240 VAC, connected by hand under the approved charging procedure. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-10 Dock and charge |
| **Rationale** | N-03, N-12 and N-17: the robot is charged by hand from its delivered charger under the approved charging procedure (D97, D94). The charger gives 54.6 V and 20 A (about 0.33 C) through the SB50 connector [DATA] (as-built record section 2; P.power.charger.cccv_V, I_max, params/param_power.m:174-176). The charger is not throttled by the BMS (D94) (the BMS and power engineer, step 7, T7-3). |
| **Source** | N-03, N-12, N-17, D94, D97 |
| **Parent** | None |
| **Allocation** | PWR, BMS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the charger's data sheet and the charging procedure (card G1-04) |
| **As built** | **Hardware** |
| **Where** | the charger (P.power.charger; power spec 2.6) |
| **Note** | The twin's charger is the dock's interlocked one; the robot's charger and its connector are the supplier's. |

| REQ-SYS-179 | The robot shall show the operator on its console its mode, its mission state and progress, the reason of each hold or pause, its active faults, its charge and the alert of D93, refreshed at least every 0.5 s. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-12 Communicate and record |
| **Rationale** | N-05: the operator answers the robot's holds and faults (charter section 5). The content is that of REQ-SYS-082, -085 and -105, and the refresh a design target of 0.5 s, 2.5 times the GNC status period of 0.2 s (Bus_GncStatus; P.gnc.Ts_status, params/param_gnc.m:17), with MCU_STATUS every 50 ms (ICD section 5) (the electronics and comms engineer, step 8, TR-4). The draft of step 12 asked for 0.2 s, the status period itself; the owner's 0.5 s is taken. |
| **Source** | N-05, D93 |
| **Parent** | None |
| **Allocation** | COM, GNC, MCU |
| **Safety** | No |
| **TBD** | the console's hardware and its link on the robot; owner the electronics and comms engineer, card G2-COM; by G1-05 (a RID) |
| **Verification** | **Test**, system level, in simulation: the twin's runs with the operator console and the dashboard; the rates of ICD section 3 |
| **As built** | **Partly** |
| **Where** | MCU_STATUS and gnc_status (ICD sections 3 and 4); the twin's Operator_Console |
| **Note** | The twin's console and dashboard show the mode, the faults and the progress; the robot's own operator console is not yet defined (card G2-COM). |

| REQ-SYS-180 | The robot shall record its pose, its mode, its holds, its faults and its pack's state at 10 Hz or faster during a mission, for the supervisor's review after it. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-12 Communicate and record |
| **Rationale** | N-14: the supervisor checks the results afterwards (charter section 5). The twin logs every mission signal at 10 Hz (P.log.Ts_mission = 0.1 s, params/param_sim.m:93). |
| **Source** | N-14 |
| **Parent** | None |
| **Allocation** | CMP, MCU |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: T-INT-06 (logging; integration spec 9.1) |
| **As built** | **Partly** |
| **Where** | the logging manifest (scn_logging, P.log; integration spec 7.6) |
| **Note** | The twin's runs log at 10 Hz; the robot's own mission log (the companion's ROS 2 recording) is on card G2-COM. |

| REQ-SYS-181 | The robot shall take a site's mission as a file of its boundary, its rows, its dock and its implement, in the JSON format of the mission files (gnc spec 5.7). |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-13 Plan the missions |
| **Rationale** | N-14: the supervisor plans the missions of a site (charter section 5). The companion compiles the mission from the file (gnc_compileMission; scripts/missions/greenhouse_std.json and palm_demo.json). |
| **Source** | N-14 |
| **Parent** | None |
| **Allocation** | CMP, GNC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, subsystem level, in simulation: t_gnc_mission (the mission compile); S3 to S5 (the design missions) |
| **As built** | **Implemented** |
| **Where** | the mission compiler (scripts/gnc_compileMission.m) and the mission files (scripts/missions) |
| **Note** | In the twin; the companion's mission upload on the robot is on card G2-COM. |

| REQ-SYS-182 | The robot shall have its RC receiver signal a lost link within 1.0 s, by its failsafe flag or by stopping its frames, and never hold its last channels without the frame-lost flag. |
| --- | --- |
| **Kind** | Interface |
| **Function** | F-14 Respond to lost links |
| **Rationale** | SG-07 and N-10: the MCU detects a lost link itself 0.10 s after the last valid frame (REQ-SYS-067), as long as lost frames carry the flag or stop (the electronics and comms engineer, step 8, TR-3): 1.0 s is P.rc.rx_failsafe_time (params/param_rc.m:12), D34, ICD section 7. Margin: the MCU's own detection does not wait for the receiver's 1.0 s. It is safe because a receiver that held its last channels silently would defeat that detection, which the text forbids. |
| **Source** | N-10, SG-07, D34 |
| **Parent** | SG-07 |
| **Allocation** | COM |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in simulation: the receiver's settings and data on the robot (cards G1-04, G2-COM); t_rc_link failsafe_modes; t_mcu_platform testTEL05_LinkLost |
| **As built** | **Partly** |
| **Where** | the twin's receiver model (subsystems/rc; P.rc) |
| **Note** | In the twin; the robot's receiver is not in the as-built record (card G1-04). |

### 6.5 Constraint

**Kind:** a standard, a regulation, a decision of the project owner, the as-built hardware (24 requirements, 14 of them safety requirements.)

| REQ-SYS-183 | The robot shall conform to the SASO Technical Regulation for Machinery Safety, Part 2: Mobile Machinery and Heavy Duty Equipment (TR 01-08-21-180, published 21/05/2021), with the standards its Annex 1-A adopts at the latest editions on SASO's site. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103: the jurisdiction is the Kingdom of Saudi Arabia, and this regulation, applied through SABER, is the binding layer for machinery (the safety plan NASL-SHD-SAF-001 v1.4, Table 1, STD-01; the functional safety engineer, step 11, Table 3). With C-02 to C-07 it replaces the general constraint on the Kingdom's regulations drafted in step 12. |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | STR, ELC, COM, PWR |
| **Safety** | No |
| **TBD** | the tariff code, and so the regulation's coverage (HS 8433 or 8479), with card G2-FSC's classification; owner the project owner, through a SASO conformity body; by G1-05 (a RID) |
| **Verification** | **Inspection**, system level, in the field: the technical file against the regulation (card G2-FSC; the product certification) |
| **As built** | **Unknown** |
| **Where** | the technical file (not yet made) |
| **Note** | The machine's classification and its tariff code are open (card G2-FSC); product certification is a risk (D103). |

| REQ-SYS-184 | Where the SASO Technical Regulation for Electric Batteries (02-04-18-166, 2018) covers a pack built into a machine, the robot shall carry a pack that conforms to it. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-13, N-15 and D103 (STD-02; the functional safety engineer, step 11, Table 3). |
| **Source** | N-13, N-15, D103 |
| **Parent** | None |
| **Allocation** | PWR, BMS |
| **Safety** | No |
| **TBD** | the regulation's coverage of a pack built into a machine; owner the project owner; by G1-05 (a RID) |
| **Verification** | **Inspection**, system level, in the field: the pack's SABER certificate (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the delivered pack |
| **Note** | The pack's certificates are not on file (card G1-04). |

| REQ-SYS-185 | The robot shall be charged from a charger that conforms to IEC 60335-2-29:2016 with A1:2019, as the GSO low-voltage regulation BD-142004-01 lists it. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-12, N-13, N-15 and D103: the charger's mains are a hazard of SG-12 (HAZ-031), and the GSO regulation makes the standard binding (STD-28; the functional safety engineer, step 11, Table 3). Margin: not a value; the charger's certificate shows the conformity. It is safe because the standard covers the charger's own mains hazards, which SG-12 reduces by design. |
| **Source** | N-13, N-15, SG-12, D103 |
| **Parent** | SG-12 |
| **Allocation** | PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the charger's certificate (card G1-04; HAZ-031) |
| **As built** | **Unknown** |
| **Where** | the delivered charger |
| **Note** | The charger's certificate is not on file (card G1-04). |

| REQ-SYS-186 | The robot shall use, for itself and its operator's equipment, only radios that hold the Saudi equipment approval of their type under the CST technical specifications: GEN001, with RI117 for licence-exempt data equipment and RI120 for LTE user equipment. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103 (STD-04; the functional safety engineer, step 11, Table 3). The electronics and comms engineer adds the type approval of the RC equipment in its 2.4 GHz band (step 8, section 6). |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | COM |
| **Safety** | No |
| **TBD** | the approval route (CST, or SASO 3114:2026); owner the electronics and comms engineer; by G1-05 (a RID) |
| **Verification** | **Inspection**, system level, in the field: the approvals (cards G1-04, G2-COM) |
| **As built** | **Unknown** |
| **Where** | the robot's radios (not in the as-built record) |
| **Note** | The robot's radios are not in the as-built record (card G1-04). |

| REQ-SYS-187 | Where a UHF correction link is fitted, the robot shall use it only under a CST frequency licence. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103 (STD-05; the functional safety engineer, step 11, Table 3). |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | COM, SNS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the licence (card G2-COM) |
| **As built** | **Unknown** |
| **Where** | no UHF link is fitted |
| **Note** | The RTK corrections come over the companion's NTRIP client (REQ-SYS-175). |

| REQ-SYS-188 | Where the pack, or the robot with its pack, is shipped, the robot shall carry a pack that has passed the tests of subsection 38.3 of the UN Manual of Tests and Criteria, Rev.8 (2023) with Amendment 1 (2025). |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-10 and N-13: the pack stays safe in transport (STD-07; the functional safety engineer, step 11, Table 3). Margin: not a value; the tests' own limits. It is safe because the tests of UN 38.3 cover the cells' abuse cases in transport. |
| **Source** | N-13, SG-10, D103 |
| **Parent** | SG-10 |
| **Allocation** | PWR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's test summary (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the delivered pack |
| **Note** | The pack's test summary is not on file (card G1-04). |

| REQ-SYS-189 | The robot shall carry its labels, warnings and operating instructions in Arabic, or in Arabic and English (SASO TR 01-08-21-180, Article 5; the Labor Law, Article 122). |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-12 and N-15: the information for use reduces the residual hazards, and the Kingdom's rules ask for it in Arabic (STD-01, STD-06; the functional safety engineer, step 11, Table 3). Margin: not a value. It is safe because the operator reads the warnings in the language of the site. |
| **Source** | N-15, SG-12, D103 |
| **Parent** | SG-12 |
| **Allocation** | STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the labels and the manual (V-06) |
| **As built** | **Unknown** |
| **Where** | the labels and the operating instructions |
| **Note** | The robot's labels and an Arabic manual are not on file. |

| REQ-SYS-190 | The robot shall implement each safety function to the performance level of its safety goal, rated and shown with ISO 13849-1:2023 and validated with ISO 13849-2:2012. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103: ISO 13849-1:2023 stays the rating method. The performance levels are those of the safety goals SG-01 to SG-11 (hazard log NASL-SHD-HAZ-001 v1.2). The texts of the standards are not bought now, so REQ-SYS-033, -046, -048 and -098 are set by risk assessment and marked as not yet confirmed against the standards (D103). The requirement constrains the method; the levels themselves sit in the safety goals. The validation is ISO 13849-2:2012 (STD-11, STD-12; the functional safety engineer, step 11, C-08); parent none, as the requirement spans every goal, and each goal's PL binds through the goal; it applies to all the safety goals, so a trace from any goal includes it (kept unmarked as a constraint across the goals by the lead systems engineer's point 19, as the functional safety engineer, step 17, has it; the review's RID-G1T-08 asked for the safety mark). |
| **Source** | N-15, D80, D103 |
| **Parent** | None |
| **Allocation** | MCU, PWR, DRV, PER, SNS, GNC, BMS, ELC, COM, NAV |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the safety case, with the FMEA and the fault trees (cards G2-FSC, G3-FME); the validation (V-05) |
| **As built** | **Partly** |
| **Where** | the safety plan and the hazard log (docs/safety) |
| **Note** | In part 1, 8 of the 11 goals with a PL were only partly covered (the functional safety engineer, step 7); the safety architecture is card G2-FSC's (E-05). |

| REQ-SYS-191 | The robot shall have its autonomous functions designed and verified to ISO 18497-1, -2, -3 and -4:2024. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103 (STD-10; the functional safety engineer, step 11, Table 3): parent none, as it spans SG-01, -02, -03, -07 and -08. Not yet confirmed against the texts (D103). |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | GNC, PER, NAV, SNS, MCU, CMP |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the safety concept's trace to the clauses (card G2-FSC) |
| **As built** | **Unknown** |
| **Where** | the safety concept (card G2-FSC) |
| **Note** | The trace to the clauses is card G2-FSC's; the texts are not held (D103). |

| REQ-SYS-192 | The robot shall meet ISO 4254-1:2013 with Amd 1:2021, adopted in the Kingdom as SASO-ISO-4254-1. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-12 and N-15: the general safety requirements of agricultural machinery, binding through SASO TR 01-08-21-180 (Annex 1-A, row 68; STD-22; the functional safety engineer, step 11, Table 3); SASO's edition is to confirm. Margin: not a value. It is safe as far as the standard's measures are met, which card G2-FSC shows. |
| **Source** | N-15, SG-12, D103 |
| **Parent** | SG-12 |
| **Allocation** | STR, PWR, ELC |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the assessment on cards G2-FSC and V-05 |
| **As built** | **Unknown** |
| **Where** | the robot as a whole |
| **Note** | Not yet assessed; SASO's edition is to confirm. |

| REQ-SYS-193 | The robot shall have an emergency stop that meets ISO 13850:2015 and the emergency-stop clauses of IEC 60204-1:2016 with AMD1:2021. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-06 and N-06 (STD-14, STD-15; the functional safety engineer, step 11, Table 3); the reset clauses of F-01 need the texts (D103). Margin: not a value. It is safe as far as the E-stop chain meets the standards' measures, which card G2-FSC shows. |
| **Source** | N-06, N-15, SG-06 |
| **Parent** | SG-06 |
| **Allocation** | PWR, ELC, MCU |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S8a and S8b; the inspection of the E-stop chain (cards G2-FSC, V-02) |
| **As built** | **Partly** |
| **Where** | the E-stop chain (power spec 2.5) |
| **Note** | The owner confirms E-stop buttons and a safety relay with STO wiring (D80), but their parts are not recorded (card G1-04). |

| REQ-SYS-194 | The robot shall have drives whose safe torque off meets IEC 61800-5-2:2016. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-06 and N-06 (STD-16; the functional safety engineer, step 11, Table 3). Margin: not a value. It is safe because the E-stop's category-0 stop rests on the STO (D86). |
| **Source** | N-06, N-15, SG-06 |
| **Parent** | SG-06 |
| **Allocation** | DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the ESC's STO rating (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the delivered ESCs |
| **Note** | The delivered ESC's STO rating is unknown (R-14, card G1-04). |

| REQ-SYS-195 | The robot shall meet ISO 14118:2017 for the prevention of unexpected start-up. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-08 and N-11 (STD-17; the functional safety engineer, step 11, Table 3). Margin: not a value. It is safe as far as the reset policy of D88 and D91 meets the standard's measures, which card G2-FSC shows. |
| **Source** | N-11, N-15, SG-08 |
| **Parent** | SG-08 |
| **Allocation** | MCU, PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in simulation: S0 and S8a (no restart before the reset and the re-arm); the inspection on card G2-FSC |
| **As built** | **Partly** |
| **Where** | the reset policy of D88 and D91 (REQ-SYS-001, -079) |
| **Note** | D88 and D91 are not built yet (card G2-MCU). |

| REQ-SYS-196 | Where a safety-rated person sensor is fitted, the robot shall use one that meets IEC 61496-1:2020 with IEC 61496-3:2025 for use indoors, and IEC TS 62998-1:2019 for use outdoors. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-01 and N-07 (STD-20, STD-21; the functional safety engineer, step 11, Table 3). Margin: not a value. It is safe because a sensor rated to these standards can carry the person detection at its PL, which the companion's LiDAR path cannot (E-02). |
| **Source** | N-07, N-15, SG-01 |
| **Parent** | SG-01 |
| **Allocation** | SNS, PER |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the sensor's certificate (E-02, card G2-FSC) |
| **As built** | **Not implemented** |
| **Where** | none: no safety-rated person sensor is fitted |
| **Note** | The LiDAR and the companion are not safety-rated (PL b at most, hazard log E-02). |

| REQ-SYS-197 | The robot shall have guards, the mower deck's housing first, that keep the safety distances of ISO 13857:2019. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-09 and N-07 (STD-19; the functional safety engineer, step 11, Table 3); not yet confirmed against the text (D103). Margin: not a value. It is safe because a person beside the deck is kept from the blades by the guard, not by the run-down (REQ-SYS-048). |
| **Source** | N-07, N-15, SG-09 |
| **Parent** | SG-09 |
| **Allocation** | STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Test**, system level, in the field: the probe test on the guards (card G2-FSC) |
| **As built** | **Unknown** |
| **Where** | the mower deck's housing |
| **Note** | The mower is not in the as-built record (card G1-04). |

| REQ-SYS-198 | The robot shall carry a pack that meets IEC 62619:2022, which SASO TR 01-08-21-180 lists (Annex 1-A, row 135). |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-10 and N-13 (STD-27; the functional safety engineer, step 11, Table 3); IEC 62133 instead if card G2-FSC chooses IEC 60335-2-107. Margin: not a value. It is safe because the standard covers the pack's own protections against overheating, overcharge and short circuit. |
| **Source** | N-13, N-15, SG-10, D103 |
| **Parent** | SG-10 |
| **Allocation** | PWR, BMS |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the pack's certificate (card G1-04) |
| **As built** | **Unknown** |
| **Where** | the delivered pack |
| **Note** | The pack's certificate is not on file (card G1-04). |

| REQ-SYS-199 | The robot shall come with information for use that follows ISO 3600:2022 for its operator's manual and ISO 3767-1 and -2:2016 with Amd 1:2020 for the symbols on its controls. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-12 and N-12 (STD-25, STD-26; the functional safety engineer, step 11, Table 3); SASO's editions are to confirm. Margin: not a value. It is safe because the manual and the symbols carry the information for use that SG-12 relies on. |
| **Source** | N-12, N-15, SG-12, D103 |
| **Parent** | SG-12 |
| **Allocation** | STR |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the manual and the symbols (V-06) |
| **As built** | **Unknown** |
| **Where** | the operator's manual and the controls' symbols |
| **Note** | Not yet assessed; SASO's editions are to confirm. |

| REQ-SYS-200 | The robot shall meet ISO 14982:1998, adopted as SASO ISO 14982, for electromagnetic compatibility. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103 (STD-29; the functional safety engineer, step 11, Table 3; SASO's edition to confirm); the standard is being revised (the safety plan's watch list). |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | ELC, DRV, PWR, COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, on the bench: the EMC bench (V-02) |
| **As built** | **Unknown** |
| **Where** | the robot as a whole |
| **Note** | Not tested. |

| REQ-SYS-201 | The robot shall keep the required behaviour of its safety functions at the immunity levels of IEC 61000-6-7:2014. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-15 and D103 (STD-30; the functional safety engineer, step 11, Table 3): parent none, as it spans every PL d function, until the revised ISO 14982 covers the EMC of safety functions; it applies to all the safety goals with a PL d function, so a trace from those goals includes it (kept unmarked as a constraint across the goals by the lead systems engineer's point 19; the review's RID-G1T-08 asked for the safety mark). |
| **Source** | N-15, D103 |
| **Parent** | None |
| **Allocation** | MCU, ELC, PWR, DRV, SNS |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Test**, system level, on the bench: the immunity bench (cards G3-FME, V-02) |
| **As built** | **Unknown** |
| **Where** | the robot as a whole |
| **Note** | Not tested. |

| REQ-SYS-202 | The robot shall use its XMachines MAX platform, its pack and BMS, its stand-alone charger, its two dual-channel ESCs and its supplier's controller board as delivered, each unknown of them taking a provisional value with its margin until the supplier's data arrive. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-17 and D96: the delivered hardware is the baseline that the requirements aim at, provisionally; the twin's STM32G4 BMS is a variant and the STM32H743 the firmware's development target. ICD sections 4 and 6 are marked provisional, and the final choice comes with the supplier's data (card G1-04) before cards G2-DRV, G2-PWR and G2-MCU start. |
| **Source** | N-17, D96 |
| **Parent** | None |
| **Allocation** | STR, PWR, BMS, DRV, ELC |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the supplier's data and the model-against-robot audit (charter goal 1; card G1-04) |
| **As built** | **Partly** |
| **Where** | the as-built record (docs/hardware/real_robot_components.md) |
| **Note** | The platform's data are partly estimates (as-built record section 7, open items 1 to 7); the BMS's and the ESCs' protocols are unknown (R-14, R-15), and the controller board is to be confirmed (R-59). |

| REQ-SYS-203 | Where firmware built from the twin goes on the robot, the robot shall run it only with torque limits that match its fitted motors: 14.32 N m per side for the 750 W motors and 19.1 N m per side for the 1 kW units. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-16, N-17, D95 and CR-004: the twin's default is the 1 kW unit, a planned upgrade (D95), and no firmware built from the twin goes on the robot until its torque limits match the motors fitted (P.drive.motor.T_peak per motor option, params/param_drive.m:41) (the drive engineer's TD-7 and the MCU firmware engineer's T-MCU-4, steps 9 and 4). Safety under SG-05, the stricter of the two owners' views (the lead systems engineer's ruling 7; the MCU firmware engineer left the rating to step 17): the 1 kW limit on the 750 W robot would exceed its motors' peak and drive 344 N m per side through gearboxes rated 280 N m in acceleration (the 750 W option's T2B; the drive engineer's finding 7), and a broken gearbox loses the hold of the motors' spring brakes. Margin: none above the fitted motors' peak; the limit is the motors' own. It is safe because the firmware then cannot command more torque than the fitted drive carries. |
| **Source** | N-16, N-17, SG-05, D95, CR-004 |
| **Parent** | SG-05 |
| **Allocation** | MCU, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, on the bench: each firmware build record against the fitted motors (cards B-00, B-01, G2-MCU) |
| **As built** | **Not implemented** |
| **Where** | the MCU's torque limits (P.electronics.drv.T_peak, from P.drive.motor) |
| **Note** | No firmware has been built for the robot (cards B-00 and B-01). |

| REQ-SYS-204 | The robot shall keep its traction bus below 60 V DC in every state, charging included. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-10 and N-13: live parts stay in the 48 V class (the BMS and power engineer, step 7, T7-4): the pack's maximum is 55.25 V and the charger's OVP 55.9 V (P.power.pack.V_abs_max, P.power.charger.V_ovp). Set by risk assessment and not yet confirmed against the standards (D103). Margin: 4.1 V between the charger's OVP and 60 V while charging. Not met for a regen-pumped bus cut off from the pack: the 60 V DC-link clamp on each 48 V implement drive conducts only above 55.25 V and holds such a bus at 60 V plus Rz x I (P.power.impl.dc_clamp.BV 60 V, Rz 0.05 Ohm), the ESCs cut regen only from 57 to 60 V and trip at 62 V (P.drive.esc.V_ov_derate, V_ov_fault), no brake chopper is fitted, and after an E-stop K1 opens 1.5 s after the stop while the blades may run down on regenerative braking for up to 3.0 s (power spec 2.5, 2.7 and section 9, O7; drive spec 2.9). It is safe while charging and with the pack connected. With K1 open the twin does not meet it, about 0.7 V over 60 V: decided, pending on card G2-PWR, with the drive engineer: a clamp that holds below 60 V at its rated current and above the charger's 55.9 V, or no regeneration once K1 can open (the lead systems engineer's point 12). |
| **Source** | N-13, SG-10, D103 |
| **Parent** | SG-10 |
| **Allocation** | PWR, BMS, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the pack's and the charger's maximum voltages; the inspection of the charger's OVP (card G1-04) |
| **As built** | **Partly** |
| **Where** | the pack and the charger's OVP (P.power.pack, P.power.charger; power spec 2.8) |
| **Note** | Implemented in the twin; the charger's OVP on the robot is [EST] (card G1-04). With K1 open the twin's regen-pumped bus reaches about 0.7 V over 60 V: decided, pending on card G2-PWR (the lead systems engineer's point 12). |

| REQ-SYS-205 | The robot shall keep every safety component of its electronics (the safety relay, the controller's safety inputs and outputs, the STO inputs) within its rated ambient temperature in the hot case of REQ-SYS-164, with its enclosure temperature stop (REQ-SYS-114) below the lowest such rating. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | SG-06 and N-13: the parts of the E-stop chain must work in the heat of the sites (the electronics and comms engineer, step 8, TR-7 and finding F3, proposed as safety for the functional safety engineer to judge; parent SG-06 by the integration engineer, as every part named is in the E-stop chain). The twin's box air is 55.3 deg C at 50 deg C ambient with the fans (electronics spec 2.9.4), against a typical 55 deg C rating of compact safety relays (to be confirmed for the delivered relay, card G1-04). Margin: not met with the assumed relay: about -0.3 K against a 55 deg C relay on the twin's box air, and about -1.5 K on the stricter plain-box reading of 56.5 deg C (electronics spec 2.9.4; the box's area is settled on card G2-VAL), and the stop of REQ-SYS-114 at 80 deg C lies 25 K above that rating, while REQ-SYS-165 asks for no thermal stop at 50 deg C ambient, where the box air is 55.3 deg C; so REQ-SYS-205, -114 and -165 cannot all hold with such a relay (the register's re-check, step 14b; the functional safety engineer, step 17, section 3, point 8). The ways out are parts rated for at least the 80 deg C stop, the relay and the STO interface placed outside the box in shaded air, cooling, or the stop below the lowest rating with REQ-SYS-165 relaxed: an open safety value for the owner, decided on card G1-04 once the delivered relay's rating and place are known; the lead systems engineer recommends parts rated for the 80 deg C stop or the relay and the STO interface placed outside the box, as both keep the mission in the heat (the lead systems engineer's point 1). It is safe only once the delivered relay's rating and place are known and one of these holds. |
| **Source** | N-06, N-13, SG-06 |
| **Parent** | SG-06 |
| **Allocation** | ELC, PWR, DRV |
| **Safety** | Yes |
| **TBD** | None |
| **Verification** | **Inspection**, system level, in the field: the delivered safety relay's and STO inputs' ratings and places against the enclosure's temperature (cards G1-04, G2-FSC) |
| **As built** | **Unknown** |
| **Where** | the electronics enclosure and the safety relay (not in the as-built record) |
| **Note** | The delivered relay's rating and place are unknown (card G1-04). |

| REQ-SYS-206 | The robot shall run its CAN1 at 500 kbit/s, the delivered BMS's bit rate, with a load below 40 % with its two ESCs, its BMS and the implement of this phase. |
| --- | --- |
| **Kind** | Constraint |
| **Function** | None: system-wide |
| **Rationale** | N-17: the delivered BMS talks at 500 kbit/s [DATA] (as-built record section 2) (the electronics and comms engineer, step 8, TR-9). The load is 21 % (ICD section 4: 770 frames/s) against the 40 % design rule (T-EL-44). |
| **Source** | N-17, D96 |
| **Parent** | None |
| **Allocation** | COM |
| **Safety** | No |
| **TBD** | None |
| **Verification** | **Analysis**, system level, in simulation: the bus load of ICD section 4 (T-EL-44) |
| **As built** | **Implemented** |
| **Where** | CAN1 (CAN1_Net_Sim; ICD section 4) |
| **Note** | In the twin; the ESC and BMS frames are provisional (D96, card G1-04). |

## 7. The standards

The 33 standards and regulations that apply, from the register, each dated. D103 sets the jurisdiction, the Kingdom of Saudi Arabia, with the binding SASO and CST layer in its own column: 14 are binding, 17 are the state of the art, and 2 are still to choose. ISO 13849-1:2023 stays the rating method. "Cited by" names the requirements whose texts cite the standard by its ID.

Table 6. The standards, with their editions and whether they bind.
| ID | Standard | Title | Edition | Binding | Cited by |
| --- | --- | --- | --- | --- | --- |
| STD-01 | SASO TR 01-08-21-180 | Technical Regulation for Machinery Safety, Part 2: Mobile Machinery and Heavy Duty Equipment | 2021 (published 21/05/2021) | binding: a regulation, through SABER | REQ-SYS-183, REQ-SYS-189 |
| STD-02 | SASO TR 02-04-18-166 | Technical Regulation for Electric Batteries | 2018 | binding: a regulation (a built-in pack to confirm) | REQ-SYS-184 |
| STD-03 | GSO BD-142004-01 | Low-voltage electrical equipment: the list of standards | current list (to confirm) | binding: a regulation | - |
| STD-04 | CST GEN001, RI117, RI120 | Technical specifications for radio equipment | RI117 and RI120 in force since 20/07/2021; GEN001 to confirm | binding: equipment approval | REQ-SYS-186 |
| STD-05 | CST National Frequency Plan | Frequency licences | current | binding: for a licensed band | REQ-SYS-187 |
| STD-06 | Royal Decree M/51 | Labor Law, Articles 122 and 123 | 23/08/1426 H, as amended | binding: on the employer | REQ-SYS-189 |
| STD-07 | UN Manual of Tests and Criteria, 38.3 | Lithium metal and lithium ion batteries: tests for transport | Rev.8 (2023) with Amendment 1 (2025) | binding: for transport | REQ-SYS-188 |
| STD-08 | Regulation (EU) 2023/1230 | Machinery regulation | 2023, applies from 20 January 2027 | not binding: the state of the art (Annex III 3.2.4, 3.3) | - |
| STD-09 | ISO 12100 | Safety of machinery: general principles for design, risk assessment and risk reduction | 2010 | state of the art | - |
| STD-10 | ISO 18497-1, -2, -3, -4 | Safety of partially automated, semi-autonomous and autonomous agricultural machinery | 2024 (all four) | state of the art | REQ-SYS-191 |
| STD-11 | ISO 13849-1 | Safety-related parts of control systems: general principles for design | 2023 | state of the art (the rating method) | REQ-SYS-190 |
| STD-12 | ISO 13849-2 | Safety-related parts of control systems: validation | 2012 | state of the art | REQ-SYS-190 |
| STD-13 | ISO 25119-1, -2, -3, -4 | Safety-related parts of control systems of agricultural machinery | -1 2018+Amd1:2020; -2 2019; -3 2018+Amd1:2020; -4 2018+Amd1:2020 | state of the art (not used for the rating) | - |
| STD-14 | ISO 13850 | Emergency stop function: principles for design | 2015 | state of the art | REQ-SYS-193 |
| STD-15 | IEC 60204-1 | Electrical equipment of machines: general requirements | 2016+AMD1:2021 (edition 6.1) | state of the art | REQ-SYS-193 |
| STD-16 | IEC 61800-5-2 | Adjustable speed electrical power drive systems: functional safety requirements | 2016 (edition 2) | state of the art | REQ-SYS-194 |
| STD-17 | ISO 14118 | Prevention of unexpected start-up | 2017 | state of the art | REQ-SYS-195 |
| STD-18 | ISO 13855 | Positioning of safeguards with respect to the approach speeds of parts of the human body | 2024 (edition 3) | state of the art | - |
| STD-19 | ISO 13857 | Safety distances to prevent hazard zones being reached by upper and lower limbs | 2019 | state of the art | REQ-SYS-197 |
| STD-20 | IEC 61496-1, -3 | Electro-sensitive protective equipment; part 3, AOPDDR | -1 2020; -3 2025 (edition 4) | state of the art | REQ-SYS-196 |
| STD-21 | IEC TS 62998-1 | Safety-related sensors used for the protection of persons | 2019 | state of the art | REQ-SYS-196 |
| STD-22 | ISO 4254-1 | Agricultural machinery: safety, general requirements | 2013+Amd1:2021 | binding through SASO TR 01-08-21-180 (Annex 1-A, row 68; SASO's edition to confirm) | REQ-SYS-192 |
| STD-23 | the mower standard | to choose on G2-FSC with the machine's classification (D103) | to choose | to confirm | - |
| STD-24 | ISO 5718-1 | Harvesting equipment: blades for rotary mowers | 2023 (SASO's edition to confirm) | binding through SASO TR 01-08-21-180 (row 118), where it applies | - |
| STD-25 | ISO 3767-1, -2 | Symbols for operator controls and other displays | 2016+Amd1:2020 each | binding through SASO TR 01-08-21-180 (rows 102, 103; SASO's edition to confirm) | REQ-SYS-199 |
| STD-26 | ISO 3600 | Operator's manuals: content and format | 2022 (edition 5; SASO's edition to confirm) | binding through SASO TR 01-08-21-180 (row 101) | REQ-SYS-199 |
| STD-27 | IEC 62619 | Secondary lithium cells and batteries for industrial applications: safety | 2022 (edition 2; SASO lists 2017) | binding through SASO TR 01-08-21-180 (row 135) | REQ-SYS-198 |
| STD-28 | IEC 60335-2-29 | Household and similar appliances: particular requirements for battery chargers | 2016+A1:2019 | binding through GSO BD-142004-01 | REQ-SYS-185 |
| STD-29 | ISO 14982 | Agricultural and forestry machinery: electromagnetic compatibility | 1998 | binding through SASO TR 01-08-21-180 (row 89; SASO's edition to confirm) | REQ-SYS-200 |
| STD-30 | IEC 61000-6-7 | Immunity of equipment in safety-related systems, industrial locations | 2014 | state of the art | REQ-SYS-201 |
| STD-31 | IEC 60529 | Degrees of protection provided by enclosures (IP code) | edition 2.2 (1989+Amd1:1999+Amd2:2013) | state of the art | REQ-SYS-171 |
| STD-32 | ISO 20653 | Road vehicles: degrees of protection (IP code) | 2023 | state of the art | - |
| STD-33 | the security standard | to choose on G2-CYB (IEC 62443, or ISO 24882 once published) | to choose | to confirm | - |

## 8. Traceability

### 8.1 The needs and their requirements

Each need with the requirements, functional and technical, that name it in their source. Every need is a source of at least one requirement.

Table 7. The needs and the requirements that trace to them.
| Need | Requirements | Count |
| --- | --- | --- |
| N-01 | REQ-SYS-014, REQ-SYS-020, REQ-SYS-027, REQ-SYS-028, REQ-SYS-036, REQ-SYS-040, REQ-SYS-043, REQ-SYS-045, REQ-SYS-086, REQ-SYS-087, REQ-SYS-088, REQ-SYS-093, REQ-SYS-096, REQ-SYS-102, REQ-SYS-121, REQ-SYS-125, REQ-SYS-126, REQ-SYS-127, REQ-SYS-134, REQ-SYS-151, REQ-SYS-154, REQ-SYS-156, REQ-SYS-157, REQ-SYS-167, REQ-SYS-168, REQ-SYS-169, REQ-SYS-170, REQ-SYS-176, REQ-SYS-207 | 29 |
| N-02 | REQ-SYS-013, REQ-SYS-018, REQ-SYS-056, REQ-SYS-057, REQ-SYS-059, REQ-SYS-104, REQ-SYS-105, REQ-SYS-125, REQ-SYS-136, REQ-SYS-138, REQ-SYS-152, REQ-SYS-208 | 12 |
| N-03 | REQ-SYS-014, REQ-SYS-018, REQ-SYS-036, REQ-SYS-056, REQ-SYS-058, REQ-SYS-059, REQ-SYS-062, REQ-SYS-086, REQ-SYS-094, REQ-SYS-117, REQ-SYS-119, REQ-SYS-138, REQ-SYS-177, REQ-SYS-178, REQ-SYS-208 | 15 |
| N-04 | REQ-SYS-003, REQ-SYS-005, REQ-SYS-006, REQ-SYS-008, REQ-SYS-009, REQ-SYS-010, REQ-SYS-011, REQ-SYS-012, REQ-SYS-077, REQ-SYS-115, REQ-SYS-119, REQ-SYS-120, REQ-SYS-141, REQ-SYS-142, REQ-SYS-173, REQ-SYS-174, REQ-SYS-210 | 17 |
| N-05 | REQ-SYS-005, REQ-SYS-006, REQ-SYS-013, REQ-SYS-015, REQ-SYS-016, REQ-SYS-017, REQ-SYS-019, REQ-SYS-039, REQ-SYS-041, REQ-SYS-082, REQ-SYS-085, REQ-SYS-091, REQ-SYS-092, REQ-SYS-093, REQ-SYS-094, REQ-SYS-102, REQ-SYS-105, REQ-SYS-113, REQ-SYS-118, REQ-SYS-179 | 20 |
| N-06 | REQ-SYS-063, REQ-SYS-064, REQ-SYS-065, REQ-SYS-066, REQ-SYS-139, REQ-SYS-140, REQ-SYS-193, REQ-SYS-194, REQ-SYS-205 | 9 |
| N-07 | REQ-SYS-004, REQ-SYS-029, REQ-SYS-033, REQ-SYS-034, REQ-SYS-035, REQ-SYS-037, REQ-SYS-038, REQ-SYS-039, REQ-SYS-040, REQ-SYS-046, REQ-SYS-047, REQ-SYS-049, REQ-SYS-065, REQ-SYS-088, REQ-SYS-092, REQ-SYS-098, REQ-SYS-099, REQ-SYS-101, REQ-SYS-109, REQ-SYS-116, REQ-SYS-126, REQ-SYS-131, REQ-SYS-132, REQ-SYS-133, REQ-SYS-134, REQ-SYS-135, REQ-SYS-196, REQ-SYS-197, REQ-SYS-209, REQ-SYS-210 | 30 |
| N-08 | REQ-SYS-030, REQ-SYS-031, REQ-SYS-032, REQ-SYS-042, REQ-SYS-043, REQ-SYS-044, REQ-SYS-051, REQ-SYS-054, REQ-SYS-072, REQ-SYS-073, REQ-SYS-074, REQ-SYS-075, REQ-SYS-076, REQ-SYS-089, REQ-SYS-090, REQ-SYS-097, REQ-SYS-100, REQ-SYS-108, REQ-SYS-111, REQ-SYS-112, REQ-SYS-116, REQ-SYS-128, REQ-SYS-129, REQ-SYS-130, REQ-SYS-144, REQ-SYS-145, REQ-SYS-146, REQ-SYS-147, REQ-SYS-149, REQ-SYS-155, REQ-SYS-157, REQ-SYS-159, REQ-SYS-160, REQ-SYS-161, REQ-SYS-167, REQ-SYS-168, REQ-SYS-207 | 37 |
| N-09 | REQ-SYS-020, REQ-SYS-021, REQ-SYS-022, REQ-SYS-023, REQ-SYS-025, REQ-SYS-026, REQ-SYS-027, REQ-SYS-095, REQ-SYS-096, REQ-SYS-122, REQ-SYS-123, REQ-SYS-124, REQ-SYS-175 | 13 |
| N-10 | REQ-SYS-002, REQ-SYS-019, REQ-SYS-021, REQ-SYS-024, REQ-SYS-035, REQ-SYS-067, REQ-SYS-068, REQ-SYS-069, REQ-SYS-070, REQ-SYS-071, REQ-SYS-077, REQ-SYS-081, REQ-SYS-095, REQ-SYS-101, REQ-SYS-108, REQ-SYS-109, REQ-SYS-110, REQ-SYS-111, REQ-SYS-113, REQ-SYS-114, REQ-SYS-123, REQ-SYS-141, REQ-SYS-143, REQ-SYS-148, REQ-SYS-182 | 25 |
| N-11 | REQ-SYS-001, REQ-SYS-002, REQ-SYS-003, REQ-SYS-004, REQ-SYS-007, REQ-SYS-011, REQ-SYS-050, REQ-SYS-064, REQ-SYS-078, REQ-SYS-079, REQ-SYS-101, REQ-SYS-107, REQ-SYS-118, REQ-SYS-146, REQ-SYS-174, REQ-SYS-195 | 16 |
| N-12 | REQ-SYS-007, REQ-SYS-042, REQ-SYS-055, REQ-SYS-079, REQ-SYS-080, REQ-SYS-082, REQ-SYS-083, REQ-SYS-100, REQ-SYS-101, REQ-SYS-103, REQ-SYS-106, REQ-SYS-137, REQ-SYS-176, REQ-SYS-178, REQ-SYS-199 | 15 |
| N-13 | REQ-SYS-052, REQ-SYS-053, REQ-SYS-054, REQ-SYS-055, REQ-SYS-060, REQ-SYS-061, REQ-SYS-103, REQ-SYS-106, REQ-SYS-108, REQ-SYS-114, REQ-SYS-137, REQ-SYS-148, REQ-SYS-153, REQ-SYS-162, REQ-SYS-163, REQ-SYS-164, REQ-SYS-165, REQ-SYS-166, REQ-SYS-171, REQ-SYS-172, REQ-SYS-184, REQ-SYS-185, REQ-SYS-188, REQ-SYS-198, REQ-SYS-204, REQ-SYS-205 | 26 |
| N-14 | REQ-SYS-083, REQ-SYS-084, REQ-SYS-086, REQ-SYS-180, REQ-SYS-181 | 5 |
| N-15 | REQ-SYS-033, REQ-SYS-037, REQ-SYS-045, REQ-SYS-048, REQ-SYS-063, REQ-SYS-064, REQ-SYS-098, REQ-SYS-183, REQ-SYS-184, REQ-SYS-185, REQ-SYS-186, REQ-SYS-187, REQ-SYS-189, REQ-SYS-190, REQ-SYS-191, REQ-SYS-192, REQ-SYS-193, REQ-SYS-194, REQ-SYS-195, REQ-SYS-196, REQ-SYS-197, REQ-SYS-198, REQ-SYS-199, REQ-SYS-200, REQ-SYS-201 | 25 |
| N-16 | REQ-SYS-150, REQ-SYS-203 | 2 |
| N-17 | REQ-SYS-152, REQ-SYS-153, REQ-SYS-154, REQ-SYS-155, REQ-SYS-156, REQ-SYS-158, REQ-SYS-159, REQ-SYS-178, REQ-SYS-202, REQ-SYS-203, REQ-SYS-206 | 11 |

### 8.2 The safety goals and their requirements

Each safety goal with the safety requirements, functional and technical, that realise it: the requirements whose parent it is. Every goal has at least one child requirement.

Table 8. The safety goals and the requirements that realise them.
| Goal | Requirements | Count |
| --- | --- | --- |
| SG-01 | REQ-SYS-029, REQ-SYS-033, REQ-SYS-034, REQ-SYS-035, REQ-SYS-037, REQ-SYS-038, REQ-SYS-058, REQ-SYS-081, REQ-SYS-092, REQ-SYS-099, REQ-SYS-109, REQ-SYS-116, REQ-SYS-131, REQ-SYS-132, REQ-SYS-133, REQ-SYS-196 | 16 |
| SG-02 | REQ-SYS-039, REQ-SYS-049 | 2 |
| SG-03 | REQ-SYS-020, REQ-SYS-022, REQ-SYS-023, REQ-SYS-025, REQ-SYS-026, REQ-SYS-095, REQ-SYS-122, REQ-SYS-124 | 8 |
| SG-04 | REQ-SYS-024, REQ-SYS-030, REQ-SYS-032, REQ-SYS-042, REQ-SYS-043, REQ-SYS-044, REQ-SYS-051, REQ-SYS-054, REQ-SYS-075, REQ-SYS-076, REQ-SYS-077, REQ-SYS-089, REQ-SYS-090, REQ-SYS-100, REQ-SYS-108, REQ-SYS-110, REQ-SYS-111, REQ-SYS-147, REQ-SYS-149, REQ-SYS-155 | 20 |
| SG-05 | REQ-SYS-006, REQ-SYS-031, REQ-SYS-071, REQ-SYS-072, REQ-SYS-073, REQ-SYS-074, REQ-SYS-112, REQ-SYS-144, REQ-SYS-145, REQ-SYS-146, REQ-SYS-203 | 11 |
| SG-06 | REQ-SYS-063, REQ-SYS-064, REQ-SYS-065, REQ-SYS-066, REQ-SYS-114, REQ-SYS-139, REQ-SYS-140, REQ-SYS-193, REQ-SYS-194, REQ-SYS-205 | 10 |
| SG-07 | REQ-SYS-067, REQ-SYS-068, REQ-SYS-069, REQ-SYS-070, REQ-SYS-143, REQ-SYS-182 | 6 |
| SG-08 | REQ-SYS-001, REQ-SYS-002, REQ-SYS-003, REQ-SYS-004, REQ-SYS-011, REQ-SYS-078, REQ-SYS-079, REQ-SYS-080, REQ-SYS-107, REQ-SYS-115, REQ-SYS-174, REQ-SYS-195 | 12 |
| SG-09 | REQ-SYS-045, REQ-SYS-046, REQ-SYS-047, REQ-SYS-048, REQ-SYS-050, REQ-SYS-098, REQ-SYS-101, REQ-SYS-197, REQ-SYS-209, REQ-SYS-210 | 10 |
| SG-10 | REQ-SYS-052, REQ-SYS-053, REQ-SYS-055, REQ-SYS-060, REQ-SYS-061, REQ-SYS-103, REQ-SYS-106, REQ-SYS-163, REQ-SYS-188, REQ-SYS-198, REQ-SYS-204 | 11 |
| SG-11 | REQ-SYS-008, REQ-SYS-009, REQ-SYS-010, REQ-SYS-012 | 4 |
| SG-12 | REQ-SYS-185, REQ-SYS-189, REQ-SYS-192, REQ-SYS-199 | 4 |
| **Total** |  | **114** |

### 8.3 The allocation

Table 9. The requirements allocated to each part of the system.
| Code | Part of the system | Functional | Technical | Safety requirements |
| --- | --- | --- | --- | --- |
| MCU | the MCU firmware, Shaddad_MCU | 79 | 26 | 72 |
| BMS | the BMS firmware, Shaddad_BMS | 18 | 12 | 14 |
| PWR | the power hardware | 24 | 28 | 30 |
| DRV | the drive | 41 | 31 | 48 |
| STR | the structure | 10 | 19 | 13 |
| ELC | the electronics hardware | 11 | 13 | 14 |
| COM | communications | 19 | 16 | 19 |
| SNS | the sensors | 19 | 12 | 22 |
| NAV | the navigation fusion, Shaddad_NavFusion | 18 | 7 | 15 |
| PER | the perception, Shaddad_Perception | 18 | 10 | 19 |
| GNC | guidance and the mission, Shaddad_GNC | 61 | 19 | 31 |
| CMP | the companion, Shaddad_Companion | 24 | 8 | 6 |

A requirement can be allocated to several parts, so the counts add up to 342 functional and 201 technical, not 121 and 89.

### 8.4 The functions and the kinds

Table 10. The functional requirements of each function, with their safety, TBD and as-built counts.
| Function | Requirements | Safety | With a TBD | Implemented | Partly | Not implemented | Hardware | Unknown |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| F-01 | 8 | 6 | 0 | 3 | 1 | 1 | 3 | 0 |
| F-02 | 6 | 6 | 0 | 2 | 4 | 0 | 0 | 0 |
| F-03 | 10 | 1 | 0 | 4 | 5 | 1 | 0 | 0 |
| F-04 | 9 | 7 | 0 | 1 | 6 | 2 | 0 | 0 |
| F-05 | 8 | 5 | 0 | 3 | 4 | 1 | 0 | 0 |
| F-06 | 5 | 4 | 0 | 0 | 4 | 1 | 0 | 0 |
| F-07 | 6 | 4 | 0 | 1 | 4 | 1 | 0 | 0 |
| F-08 | 15 | 14 | 0 | 3 | 9 | 2 | 1 | 0 |
| F-09 | 9 | 5 | 1 | 0 | 6 | 1 | 1 | 1 |
| F-10 | 8 | 4 | 0 | 0 | 4 | 3 | 1 | 0 |
| F-11 | 7 | 7 | 0 | 1 | 3 | 1 | 2 | 0 |
| F-12 | 4 | 0 | 0 | 0 | 3 | 1 | 0 | 0 |
| F-13 | 7 | 2 | 0 | 2 | 3 | 2 | 0 | 0 |
| F-14 | 5 | 5 | 0 | 0 | 4 | 0 | 1 | 0 |
| F-15 | 14 | 13 | 0 | 5 | 5 | 2 | 2 | 0 |
| **Total** | **121** | **83** | **1** | **25** | **65** | **19** | **11** | **1** |

Table 11. The technical requirements of each kind, with their safety, TBD and as-built counts.
| Kind | Requirements | Safety | With a TBD | Implemented | Partly | Not implemented | Hardware | Unknown |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Performance | 33 | 13 | 1 | 6 | 19 | 2 | 5 | 1 |
| Physical | 11 | 1 | 0 | 0 | 1 | 0 | 9 | 1 |
| Environment | 11 | 1 | 2 | 2 | 4 | 0 | 0 | 5 |
| Interface | 10 | 2 | 1 | 2 | 6 | 0 | 2 | 0 |
| Constraint | 24 | 14 | 3 | 1 | 5 | 2 | 0 | 16 |
| **Total** | **89** | **31** | **7** | **11** | **35** | **4** | **16** | **23** |

## 9. The robot as built

Each requirement records its state on the robot as built, from 2026-10-27 onward. The sources are the as-built record `docs/hardware/real_robot_components.md`, the parameters, the decisions, `docs/STATUS.md`, the review's ranked list and the hazard log. The states rest on the checks of parts 1 and 2, the test baseline of 2026-11-15 and the dated runs that the notes cite. Part 2 changed documents only: no model, parameter or test.

Table 12. The requirements by their state on the robot as built.
| State | Meaning | Requirements | Safety requirements |
| --- | --- | --- | --- |
| **Implemented** | In the twin and its firmware, with a passing test where one exists | 36 | 18 |
| **Partly** | Met in part; the note names what is missing and the card or STATUS item that closes it | 100 | 56 |
| **Not implemented** | Not provided today; the note names the card that provides it | 23 | 15 |
| **Hardware** | Waits on the as-built data of the robot (card G1-04) | 27 | 15 |
| **Unknown** | Not shown either way yet; the note names the missing evidence | 24 | 10 |
| **Total** |  | **210** | **114** |

### 9.1 Not implemented

Table 13 lists them: 23 requirements, 15 of them safety requirements, each with the cards that close it. Each requirement's note, in its own table, says what is missing.

Table 13. The requirements not implemented on the robot as built.
| Requirement | Function or kind | Safety | Cards |
| --- | --- | --- | --- |
| REQ-SYS-001 | F-01 | Yes | G2-MCU, B-01, G1-04 |
| REQ-SYS-039 | F-07 | Yes | G2-GNC |
| REQ-SYS-062 | F-10 | No | G2-GNC, G2-PWR, G2-MCU, G2-INT, G1-04 |
| REQ-SYS-079 | F-11 | Yes | G2-MCU, G2-FSC, G1-04 |
| REQ-SYS-083 | F-12 | No | G2-MCU, G2-GNC |
| REQ-SYS-089 | F-13 | Yes | G2-GNC |
| REQ-SYS-092 | F-03 | Yes | G2-GNC |
| REQ-SYS-095 | F-04 | Yes | G2-FSC, G2-NAV, G1-02 |
| REQ-SYS-096 | F-04 | No | G2-NAV, G1-02 |
| REQ-SYS-098 | F-06 | Yes | G2-FSC, G1-02, G1-04 |
| REQ-SYS-105 | F-09 | No | G2-GNC, G2-COM |
| REQ-SYS-110 | F-15 | Yes | G2-MCU |
| REQ-SYS-111 | F-15 | Yes | G2-FSC, G2-NAV, G1-02 |
| REQ-SYS-116 | F-05 | Yes | G2-MCU |
| REQ-SYS-117 | F-10 | No | G2-GNC, G1-02 |
| REQ-SYS-122 | performance | Yes | G2-GNC, G1-02 |
| REQ-SYS-150 | performance | No | G1-04, B-01, G2-MCU |
| REQ-SYS-196 | constraint | Yes | G2-FSC |
| REQ-SYS-203 | constraint | Yes | B-00, B-01, G2-MCU |
| REQ-SYS-207 | F-13 | No | G2-GNC |
| REQ-SYS-208 | F-10 | No | G2-PWR, G1-04 |
| REQ-SYS-209 | F-08 | Yes | G2-FSC, G2-MCU, G2-GNC, G1-02 |
| REQ-SYS-210 | F-08 | Yes | G2-MCU, G1-02 |

### 9.2 Waiting on hardware data

Table 14 lists them: 27 requirements, 15 of them safety requirements, each with the cards that close it. Each requirement's note, in its own table, says what is missing.

Table 14. The requirements waiting on hardware data on the robot as built.
| Requirement | Function or kind | Safety | Cards |
| --- | --- | --- | --- |
| REQ-SYS-004 | F-01 | Yes | G1-04 |
| REQ-SYS-007 | F-01 | No | G1-04 |
| REQ-SYS-045 | F-08 | Yes | G1-04, G1-02 |
| REQ-SYS-052 | F-09 | Yes | G1-04 |
| REQ-SYS-063 | F-11 | Yes | G1-04, G2-FSC, G2-PWR |
| REQ-SYS-065 | F-11 | Yes | G1-04 |
| REQ-SYS-080 | F-01 | Yes | G1-04, V-06 |
| REQ-SYS-081 | F-15 | Yes | G1-04, G2-DRV |
| REQ-SYS-106 | F-10 | Yes | G2-PWR, G2-MCU, G1-04 |
| REQ-SYS-108 | F-14 | Yes | G1-04, G1-02 |
| REQ-SYS-114 | F-15 | Yes | G1-04, G2-MCU, G2-FSC |
| REQ-SYS-139 | performance | Yes | G1-04 |
| REQ-SYS-144 | performance | Yes | G1-04, G2-DRV, T-04 |
| REQ-SYS-145 | performance | Yes | G2-DRV, G1-04, T-04 |
| REQ-SYS-146 | performance | Yes | G1-04, G2-MCU |
| REQ-SYS-148 | performance | No | G1-04 |
| REQ-SYS-151 | physical | No | G1-04 |
| REQ-SYS-152 | physical | No | G1-04 |
| REQ-SYS-153 | physical | No | G1-04 |
| REQ-SYS-154 | physical | No | G1-04 |
| REQ-SYS-155 | physical | Yes | G1-04, G2-STR, G2-DRV |
| REQ-SYS-156 | physical | No | G1-04 |
| REQ-SYS-157 | physical | No | G1-04 |
| REQ-SYS-158 | physical | No | G1-04 |
| REQ-SYS-161 | physical | No | G1-04 |
| REQ-SYS-175 | interface | No | G1-04, G2-NAV |
| REQ-SYS-178 | interface | No | G1-04, G2-PWR |

### 9.3 Unknown

Table 15 lists them: 24 requirements, 10 of them safety requirements, each with the cards that close it. Each requirement's note, in its own table, says what is missing.

Table 15. The requirements unknown on the robot as built.
| Requirement | Function or kind | Safety | Cards |
| --- | --- | --- | --- |
| REQ-SYS-104 | F-09 | No | G1-02, G2-PWR |
| REQ-SYS-141 | performance | No | G1-04, G2-COM |
| REQ-SYS-160 | physical | No | G1-04, G2-DRV |
| REQ-SYS-162 | environment | No | G2-PWR, G2-INT |
| REQ-SYS-163 | environment | Yes | G1-04, G2-PWR |
| REQ-SYS-170 | environment | No | G2-GNC, G2-INT |
| REQ-SYS-171 | environment | No | G1-04 |
| REQ-SYS-172 | environment | No | G1-04 |
| REQ-SYS-183 | constraint | No | G2-FSC |
| REQ-SYS-184 | constraint | No | G1-04 |
| REQ-SYS-185 | constraint | Yes | G1-04 |
| REQ-SYS-186 | constraint | No | G1-04, G2-COM |
| REQ-SYS-187 | constraint | No | G2-COM |
| REQ-SYS-188 | constraint | Yes | G1-04 |
| REQ-SYS-189 | constraint | Yes | V-06 |
| REQ-SYS-191 | constraint | No | G2-FSC |
| REQ-SYS-192 | constraint | Yes | G2-FSC, V-05 |
| REQ-SYS-194 | constraint | Yes | G1-04 |
| REQ-SYS-197 | constraint | Yes | G2-FSC, G1-04 |
| REQ-SYS-198 | constraint | Yes | G1-04, G2-FSC |
| REQ-SYS-199 | constraint | Yes | V-06 |
| REQ-SYS-200 | constraint | No | V-02 |
| REQ-SYS-201 | constraint | No | G3-FME, V-02 |
| REQ-SYS-205 | constraint | Yes | G1-04, G2-FSC |

## 10. Open items

### 10.1 The RIDs for G1-05

A RID records what the review G1-05 must still see closed. Most are decided values that the twin does not hold yet, and values provisional until the delivered electronics' data. Table 16 gathers them by the card that closes them: 72 requirements, some under more than one card. Each requirement's note says what is pending, and keeps today's value.

Table 16. The RIDs for G1-05, by the card that closes them.
| Card | What waits on it | Requirements |
| --- | --- | --- |
| G2-GNC | GNC: preliminary design; in the scenarios, S4's inset, S5's VFH and S5b's hold | REQ-SYS-021, REQ-SYS-022, REQ-SYS-026, REQ-SYS-027, REQ-SYS-040, REQ-SYS-087, REQ-SYS-088, REQ-SYS-097, REQ-SYS-099, REQ-SYS-113, REQ-SYS-117, REQ-SYS-122, REQ-SYS-126, REQ-SYS-127, REQ-SYS-131, REQ-SYS-133, REQ-SYS-135, REQ-SYS-177, REQ-SYS-207, REQ-SYS-209 |
| G2-NAV | Sensors and navigation: preliminary design | REQ-SYS-020, REQ-SYS-021, REQ-SYS-023, REQ-SYS-095, REQ-SYS-096, REQ-SYS-124 |
| G2-PWR | Power and BMS: preliminary design | REQ-SYS-048, REQ-SYS-054, REQ-SYS-055, REQ-SYS-060, REQ-SYS-101, REQ-SYS-103, REQ-SYS-136, REQ-SYS-138, REQ-SYS-162, REQ-SYS-163, REQ-SYS-164, REQ-SYS-204, REQ-SYS-208 |
| G2-MCU | MCU and electronics: preliminary design | REQ-SYS-031, REQ-SYS-054, REQ-SYS-069, REQ-SYS-077, REQ-SYS-112, REQ-SYS-115, REQ-SYS-116, REQ-SYS-209, REQ-SYS-210 |
| G2-DRV | Drive: preliminary design | REQ-SYS-112, REQ-SYS-145, REQ-SYS-155, REQ-SYS-166 |
| G2-FSC | Functional safety concept | REQ-SYS-046, REQ-SYS-048, REQ-SYS-095, REQ-SYS-098, REQ-SYS-111, REQ-SYS-114, REQ-SYS-131, REQ-SYS-205, REQ-SYS-209 |
| G2-INT | Integration: preliminary design; the criteria of the scenario scripts: S2's speed and window, S4's clearance, S5's pedestrian, head-on case and mower run, S5b's scripted resume, S6, and S7's 0.25 m and 5 s values | REQ-SYS-020, REQ-SYS-033, REQ-SYS-087, REQ-SYS-097, REQ-SYS-112, REQ-SYS-123, REQ-SYS-133, REQ-SYS-143 |
| G2-COM | Comms and RC: preliminary design | REQ-SYS-012, REQ-SYS-142 |
| G2-STR | Structure: preliminary design | REQ-SYS-044, REQ-SYS-149, REQ-SYS-155 |
| T-04 | Field and bench test procedures | REQ-SYS-130 |
| G1-04 | Supplier data for the real electronics; the values provisional under D96, final with the delivered electronics' data | REQ-SYS-031, REQ-SYS-044, REQ-SYS-054, REQ-SYS-055, REQ-SYS-060, REQ-SYS-063, REQ-SYS-069, REQ-SYS-070, REQ-SYS-073, REQ-SYS-081, REQ-SYS-103, REQ-SYS-108, REQ-SYS-109, REQ-SYS-114, REQ-SYS-149, REQ-SYS-150, REQ-SYS-153, REQ-SYS-158, REQ-SYS-160, REQ-SYS-165, REQ-SYS-205 |

- **Open safety values for the project owner:** REQ-SYS-114, REQ-SYS-165 and REQ-SYS-205. Each one's rationale or note gives the conflict, the options and the card that decides it.
- **Set by risk assessment and not yet confirmed against the standards** (D103): REQ-SYS-033, REQ-SYS-046, REQ-SYS-048 and REQ-SYS-098.

Source: `work/team/g1-01/p2/integration/report_step13.md` section 4.1, `report_step19.md` section 7 and `report_step19c.md`, the integration engineer's records of part 2.

### 10.2 The values still to set

The requirements carry 8 TBDs, none of them on a safety requirement. Each names the value, its owner and the event by which it is set, and each goes to G1-05 as a RID.

Table 17. The values still to set, with their owners.
| Requirement | Kind | The value to set | Owner | By |
| --- | --- | --- | --- | --- |
| REQ-SYS-104 | functional | the energy per hour of each design mission, from a phase-2 mowing energy run after D94 and D95 (D101) | the BMS and power engineer with the GNC engineer | G1-05 (a RID) |
| REQ-SYS-141 | performance | the RC link's range inside a greenhouse, by a range test with the robot's own RC equipment | the electronics and comms engineer, card G2-COM | G1-05 (a RID) |
| REQ-SYS-164 | environment | [TBD] the lower ambient limit (the row test runs down to -10 deg C) | the drive engineer, with the BMS and power engineer for the pack | G1-05 (a RID) |
| REQ-SYS-172 | environment | the ingress protection degree of the pack case and the traction motors (IEC 60529) | the BMS and power engineer with the drive engineer | G1-05 (a RID) |
| REQ-SYS-179 | interface | the console's hardware and its link on the robot | the electronics and comms engineer, card G2-COM | G1-05 (a RID) |
| REQ-SYS-183 | constraint | the tariff code, and so the regulation's coverage (HS 8433 or 8479), with card G2-FSC's classification | the project owner, through a SASO conformity body | G1-05 (a RID) |
| REQ-SYS-184 | constraint | the regulation's coverage of a pack built into a machine | the project owner | G1-05 (a RID) |
| REQ-SYS-186 | constraint | the approval route (CST, or SASO 3114:2026) | the electronics and comms engineer | G1-05 (a RID) |

### 10.3 The review decisions still open

Card G1-03 still has 3 of the review's 20 decisions open. None of them is needed to state a requirement of this version: where a requirement touches one, it is written with today's behaviour and values. Their options are in the review report NASL-SHD-ANR-001, section 7.

Table 18. The review decisions still open on card G1-03.
| Decision | The question | Review items |
| --- | --- | --- |
| 14 | Test expected values | R-28, R-29, R-64 |
| 17 | Simplified vehicle model | R-20 |
| 19 | Layout allowance | R-73 |

## Appendix A. The decisions cited

The 60 decision rows that the requirements cite, with their titles from the decision index `docs/registers/decisions.json`. The rows themselves are in the decisions table of `docs/architecture.md`.

Table 19. The decision rows that the requirements cite.
| Decision | Title | Cited by |
| --- | --- | --- |
| D8 | Safety chain | REQ-SYS-063 |
| D9 | Implements | REQ-SYS-151, REQ-SYS-176 |
| D13 | Rates | REQ-SYS-097 |
| D30 | Safety-chain re-arm (blocker) | REQ-SYS-001, REQ-SYS-064, REQ-SYS-079 |
| D33 | Deceleration chain | REQ-SYS-009, REQ-SYS-029, REQ-SYS-066, REQ-SYS-071, REQ-SYS-140, REQ-SYS-143 |
| D34 | RC failsafe semantics | REQ-SYS-011, REQ-SYS-066, REQ-SYS-067, REQ-SYS-174, REQ-SYS-182 |
| D37 | RC latency budget | REQ-SYS-005, REQ-SYS-008, REQ-SYS-067, REQ-SYS-142 |
| D41 | GNC <-> MCU timing | REQ-SYS-029 |
| D43 | GNC command envelope | REQ-SYS-029 |
| D44 | RC loss in AUTO | REQ-SYS-067 |
| D45 | Charging at 45-50 C | REQ-SYS-060, REQ-SYS-104 |
| D46 | Shift duty and power budget | REQ-SYS-104, REQ-SYS-125, REQ-SYS-136 |
| D47 | MCU tilt estimate | REQ-SYS-076, REQ-SYS-147 |
| D49 | Navigation error model | REQ-SYS-123 |
| D51 | Gearbox rating | REQ-SYS-161 |
| D53 | Turning on slopes | REQ-SYS-032, REQ-SYS-090, REQ-SYS-207 |
| D54 | Greenhouse inset | REQ-SYS-087, REQ-SYS-095, REQ-SYS-156, REQ-SYS-168, REQ-SYS-207 |
| D57 | GNC mission plan | REQ-SYS-086 |
| D64 | Drive tuning, brake model and benches for the four-motor robot | REQ-SYS-072, REQ-SYS-145 |
| D65 | BMS warnings inform, they do not derate | REQ-SYS-053, REQ-SYS-108 |
| D66 | The MCU yaw loop rests at commanded standstill | REQ-SYS-023, REQ-SYS-035, REQ-SYS-037, REQ-SYS-112 |
| D67 | The GNC rides out the ESC unpark and turns round before it rejoins | REQ-SYS-016, REQ-SYS-037, REQ-SYS-091, REQ-SYS-093 |
| D68 | Zero-mean GNSS velocity and an honest along-aisle scan match | REQ-SYS-095, REQ-SYS-124 |
| D69 | T7 judges the position sigma across the lane, with hysteresis | REQ-SYS-021, REQ-SYS-022, REQ-SYS-023, REQ-SYS-095, REQ-SYS-124 |
| D71 | Soft stops end on the brakes, with STOP_FAIL as the STO backup | REQ-SYS-006, REQ-SYS-023, REQ-SYS-025, REQ-SYS-031, REQ-SYS-035, REQ-SYS-054, REQ-SYS-066, REQ-SYS-067, REQ-SYS-071, REQ-SYS-072, REQ-SYS-073, REQ-SYS-074, REQ-SYS-109, REQ-SYS-112, REQ-SYS-143, REQ-SYS-146 |
| D72 | Person protection: lost scan stops the robot, field from the implement, implement off for the rest of the hold | REQ-SYS-004, REQ-SYS-014, REQ-SYS-029, REQ-SYS-033, REQ-SYS-034, REQ-SYS-035, REQ-SYS-038, REQ-SYS-046, REQ-SYS-049, REQ-SYS-050, REQ-SYS-068, REQ-SYS-088, REQ-SYS-098, REQ-SYS-121, REQ-SYS-131, REQ-SYS-132, REQ-SYS-133, REQ-SYS-209 |
| D73 | IMU loss handling | REQ-SYS-023, REQ-SYS-024, REQ-SYS-076, REQ-SYS-077, REQ-SYS-096, REQ-SYS-110, REQ-SYS-111 |
| D74 | Always-on descent speed cap; no reversing with the tiller | REQ-SYS-008, REQ-SYS-009, REQ-SYS-010, REQ-SYS-030, REQ-SYS-031, REQ-SYS-040, REQ-SYS-042, REQ-SYS-044, REQ-SYS-051, REQ-SYS-053, REQ-SYS-054, REQ-SYS-074, REQ-SYS-076, REQ-SYS-077, REQ-SYS-100, REQ-SYS-116, REQ-SYS-155, REQ-SYS-176 |
| D75 | CR-001: the full-tank sprayer's operating limit on descents | REQ-SYS-075, REQ-SYS-089, REQ-SYS-129 |
| D76 | The scope of this phase: mowing, with the charter as its baseline | REQ-SYS-076, REQ-SYS-149, REQ-SYS-151, REQ-SYS-169 |
| D79 | The greenhouse rule: people out of the robot's zone while it mows | REQ-SYS-026, REQ-SYS-033, REQ-SYS-099, REQ-SYS-131 |
| D80 | The safety plan approved and the safety goals accepted | REQ-SYS-063, REQ-SYS-065, REQ-SYS-190, REQ-SYS-193 |
| D81 | Implement mounting and deck rules | REQ-SYS-004, REQ-SYS-034, REQ-SYS-043, REQ-SYS-044, REQ-SYS-076, REQ-SYS-088, REQ-SYS-089, REQ-SYS-149 |
| D82 | CR-003: a safe stop is measured by the park criterion | REQ-SYS-030, REQ-SYS-072, REQ-SYS-075, REQ-SYS-149 |
| D83 | The descent table's IMU error bound: 0.75 deg | REQ-SYS-030, REQ-SYS-097, REQ-SYS-111, REQ-SYS-155 |
| D84 | A person stop resumes only on the operator's command | REQ-SYS-011, REQ-SYS-015, REQ-SYS-033, REQ-SYS-035, REQ-SYS-037, REQ-SYS-039, REQ-SYS-049, REQ-SYS-050, REQ-SYS-088, REQ-SYS-105, REQ-SYS-131, REQ-SYS-133, REQ-SYS-209 |
| D85 | An IMU fault stops autonomous work; RC driving limps under the steepest-slope cap | REQ-SYS-002, REQ-SYS-024, REQ-SYS-077, REQ-SYS-096, REQ-SYS-110, REQ-SYS-111 |
| D86 | The E-stop stops traction as category 0 | REQ-SYS-048, REQ-SYS-063, REQ-SYS-069, REQ-SYS-139, REQ-SYS-144, REQ-SYS-194 |
| D87 | The resume pivot only where the footprint fits | REQ-SYS-016, REQ-SYS-087, REQ-SYS-088, REQ-SYS-091, REQ-SYS-092, REQ-SYS-093, REQ-SYS-099 |
| D88 | After a reset the robot waits in FAULT for the operator | REQ-SYS-001, REQ-SYS-064, REQ-SYS-069, REQ-SYS-079, REQ-SYS-195 |
| D89 | Navigation validity is a health flag; the mission gate judges accuracy | REQ-SYS-020, REQ-SYS-022, REQ-SYS-023, REQ-SYS-026, REQ-SYS-027, REQ-SYS-095, REQ-SYS-096, REQ-SYS-124 |
| D90 | The RC limp speed of D85 | REQ-SYS-077 |
| D91 | Every switch-on waits for the operator's reset | REQ-SYS-001, REQ-SYS-064, REQ-SYS-069, REQ-SYS-079, REQ-SYS-107, REQ-SYS-118, REQ-SYS-195 |
| D92 | After a protective stop the blades restart only on the operator's command | REQ-SYS-015, REQ-SYS-035, REQ-SYS-039, REQ-SYS-049, REQ-SYS-050, REQ-SYS-101, REQ-SYS-105 |
| D93 | A low charge in an operator's pause: alert and wait | REQ-SYS-015, REQ-SYS-035, REQ-SYS-039, REQ-SYS-049, REQ-SYS-050, REQ-SYS-056, REQ-SYS-105, REQ-SYS-179 |
| D94 | Hot-weather charging: the sealed pack charged only when cool | REQ-SYS-057, REQ-SYS-058, REQ-SYS-059, REQ-SYS-060, REQ-SYS-061, REQ-SYS-062, REQ-SYS-104, REQ-SYS-138, REQ-SYS-152, REQ-SYS-162, REQ-SYS-163, REQ-SYS-164, REQ-SYS-178, REQ-SYS-208 |
| D95 | The 1 kW motor stays the default, as a planned upgrade | REQ-SYS-008, REQ-SYS-097, REQ-SYS-104, REQ-SYS-120, REQ-SYS-128, REQ-SYS-149, REQ-SYS-155, REQ-SYS-159, REQ-SYS-160, REQ-SYS-161, REQ-SYS-166, REQ-SYS-203 |
| D96 | The delivered electronics are the baseline, provisionally | REQ-SYS-001, REQ-SYS-031, REQ-SYS-052, REQ-SYS-054, REQ-SYS-055, REQ-SYS-060, REQ-SYS-063, REQ-SYS-069, REQ-SYS-070, REQ-SYS-073, REQ-SYS-081, REQ-SYS-103, REQ-SYS-108, REQ-SYS-109, REQ-SYS-114, REQ-SYS-137, REQ-SYS-138, REQ-SYS-148, REQ-SYS-150, REQ-SYS-153, REQ-SYS-158, REQ-SYS-165, REQ-SYS-202, REQ-SYS-206 |
| D97 | The dock stays the twin's in this phase | REQ-SYS-057, REQ-SYS-058, REQ-SYS-059, REQ-SYS-060, REQ-SYS-061, REQ-SYS-062, REQ-SYS-095, REQ-SYS-106, REQ-SYS-117, REQ-SYS-138, REQ-SYS-177, REQ-SYS-178, REQ-SYS-208 |
| D98 | Precharge limits that fit any hardware, with an early short check | REQ-SYS-055, REQ-SYS-103 |
| D99 | VFH is the default bypass, gated by the hold and the footprint | REQ-SYS-039, REQ-SYS-040, REQ-SYS-041, REQ-SYS-099, REQ-SYS-125, REQ-SYS-134, REQ-SYS-135 |
| D100 | The scenario criteria of S4, S5, S6, S2 and T10 | REQ-SYS-012, REQ-SYS-020, REQ-SYS-027, REQ-SYS-036, REQ-SYS-037, REQ-SYS-067, REQ-SYS-072, REQ-SYS-075, REQ-SYS-087, REQ-SYS-097, REQ-SYS-112, REQ-SYS-121, REQ-SYS-129, REQ-SYS-133, REQ-SYS-134, REQ-SYS-143, REQ-SYS-145, REQ-SYS-167 |
| D101 | No fixed shift: a mowing mission on one charge | REQ-SYS-013, REQ-SYS-057, REQ-SYS-059, REQ-SYS-104, REQ-SYS-119, REQ-SYS-125, REQ-SYS-136, REQ-SYS-208 |
| D103 | The Kingdom of Saudi Arabia, with every standard dated | REQ-SYS-004, REQ-SYS-033, REQ-SYS-045, REQ-SYS-046, REQ-SYS-048, REQ-SYS-098, REQ-SYS-144, REQ-SYS-171, REQ-SYS-183, REQ-SYS-184, REQ-SYS-185, REQ-SYS-186, REQ-SYS-187, REQ-SYS-188, REQ-SYS-189, REQ-SYS-190, REQ-SYS-191, REQ-SYS-192, REQ-SYS-193, REQ-SYS-197, REQ-SYS-198, REQ-SYS-199, REQ-SYS-200, REQ-SYS-201, REQ-SYS-204, REQ-SYS-209, REQ-SYS-210 |
| D104 | The detection time of a wrong but plausible IMU | REQ-SYS-111 |
| D105 | The pack in storage | REQ-SYS-163 |
| D106 | The greenhouse inset widened to 0.50 m | REQ-SYS-020, REQ-SYS-026, REQ-SYS-087, REQ-SYS-095, REQ-SYS-121, REQ-SYS-154 |
| D107 | A work rate for the greenhouse | REQ-SYS-121, REQ-SYS-125 |
| D108 | A person walking head-on: a longer field ahead on open ground | REQ-SYS-033, REQ-SYS-131, REQ-SYS-133 |
| D109 | The test person's residual risks | REQ-SYS-033 |

## Appendix B. The allocation codes

The codes name the components of Shaddad_System and the domains of the plant (`docs/architecture.md` sections 4 and 5).

Table 20. The allocation codes.
| Code | What it covers |
| --- | --- |
| MCU | the MCU firmware, Shaddad_MCU: modes, safety supervisor, vehicle control, IO, CAN codecs, telemetry |
| BMS | the BMS firmware, Shaddad_BMS |
| PWR | the power hardware: pack, PDU and contactors, precharge, LV rails, the safety chain (E-stop, safety relay, STO wiring), implement power, the dock charger |
| DRV | the drive: motors, ESCs, gearboxes, spring brakes |
| STR | the structure: body, tracks, the implements' mechanics |
| ELC | the electronics hardware: the front end, the actuators, the watchdog reset |
| COM | communications: CAN1, CAN2, the RC link from the transmitter to SBUS, the operator console |
| SNS | the sensors: IMU, RTK GNSS, 2-D and 3-D LiDAR, odometry |
| NAV | the navigation fusion, Shaddad_NavFusion |
| PER | the perception, Shaddad_Perception |
| GNC | guidance and the mission, Shaddad_GNC: mission manager, path, trackers, speed governor, avoidance, implement manager |
| CMP | the companion, Shaddad_Companion: mission planning and the companion's side of its links |
