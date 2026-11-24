<!--
doc_no: NASL-SHD-HAZ-001
doc_type: HAZARD LOG
title: Shaddad UGV: hazard log
subtitle: The hazards of the mowing phase, their ratings, the safety goals and the safety functions that meet them
summary: The hazard analysis and risk assessment of the Shaddad UGV for the mowing phase. Every operating mode and abnormal case of the charter is walked through in the greenhouse aisles, the palm groves, the slope test ground and at the dock. 45 hazards are rated with the risk graph of ISO 13849-1:2023; they give 12 safety goals, which the project owner accepted as the top-level safety requirements (D80): 11 at PL d and 1 with no PL, with people kept out of the robot's zone in a greenhouse (D79). No goal can be shown met on the robot as built, and most need hardware it does not have: the gaps are escalated to the project owner and worked on cards G2-FSC and G1-04.
version: 1.5
date: 2026-11-18
status: Issued for review
history: 1.0 | 2026-11-16 | First issue, for the independent review (card G1-07)
history: 1.1 | 2026-11-16 | The independent review RID-G1-07-01 to -08: P and F applied as written, F restated as the exposure to the hazard zone, HAZ-039 split (HAZ-045), SG-03 escalated (E-08), F-01 and SF-01 completed, the Safety Analysis Manager file, the walk-through and the goal traces derived from the hazards and checked, the classes of met today and status today defined, wording; the greenhouse decision E-09
history: 1.2 | 2026-11-16 | Approved by the project owner (D80); the greenhouse rule (D79): six goals to PL d
history: 1.3 | 2026-11-18 | E-07 decided (D103); the references dated
history: 1.4 | 2026-11-18 | RID-G1T-04: SF-06's stale ESC feedback at 50 ms (REQ-SYS-070)
history: 1.5 | 2026-11-18 | Point 31 and check 19b: HAZ-006, HAZ-018 (met today: no), SF-01 and SF-12 aligned
prepared_by: the functional safety engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV Hazard Log
file: NASL_HAZ-001_Shaddad_Hazard_Log_v1.5
outdir: work/reports/safety
chapter_breaks: no
cover_image: none
request: Card G1-07 (CR-002, approved by the owner on 2026-11-16: "Approve option 1"); the independent review of v1.0 (RID-G1-07-01 to -08); the owner's decisions of 2026-11-16 on v1.1 (D79, D80); E-07 by D103
scope: The mowing phase (D76): RC driving, autonomous missions, docking and charging, the E-stop with reset and re-arm, and maintenance, in the places of the charter. Tillage and spraying are later-phase entries, rated provisionally. Hazards to people; damage to the robot only where it leads to harm.
related: NASL-SHD-SAF-001 (the safety plan); NASL-SHD-CHR-001 (the charter); NASL-SHD-CR-001 (the sprayer envelope); NASL-SHD-CR-002 (the plan after G0); NASL-SHD-PMP-001 (the project plan); NASL-SHD-ANR-001 (the review report); docs/safety/hazard_log.json (the record); docs/safety/hazard_log.mldatx (its Safety Analysis Manager copy, OP-01)
-->
# Shaddad UGV: hazard log

## Summary

**45 hazards in the mowing phase, 8 more for later phases.** Every mode and abnormal case of the charter was walked through in each place it names (section 3). Each hazard is rated with the risk graph of ISO 13849-1:2023 and the project rules of section 2, and traced to a safety goal and to the safety functions of the twin.

- **Required levels:** PL d for 36 hazards, PL c for 1, PL b for 1, PL a for 1; 6 are reduced without a control function.
- **Safety goals:** SG-01 person protection in motion (PL d); SG-02 safe resume after a person stop (PL d); SG-03 the operating zone (PL d); SG-04 stability (PL d); SG-05 stop and hold, no runaway (PL d); SG-06 emergency stop (PL d); SG-07 stop on loss of control (PL d); SG-08 no unexpected start-up (PL d); SG-09 blade safety (PL d); SG-10 battery, charging and live parts (PL d); SG-11 manual control integrity (PL d); SG-12 hazards reduced without a control function (no PL).
- **The greenhouse rule sets the levels.** The owner keeps people out of the robot's zone while it mows a greenhouse aisle (D79; decision E-09: "Keep people out (Recommended)"), so no goal is above PL d. People at work beside the robot would put six goals at PL e and would need safety-rated person detection.
- **Approved.** The project owner decided on 2026-11-16, card G1-07: "Approve and accept (Recommended)" (D80): the safety plan NASL-SHD-SAF-001 v1.1 approved, and the 12 safety goals of this log accepted as the top-level safety requirements, at the levels the greenhouse rule (D79) sets.
- **Met today:** no goal can be shown met on the robot as built. Of the 45 hazards, the twin partly meets 11 and does not meet 25; for 3 the evidence is unknown; 6 rest on design measures and procedures still to be shown (the classes are defined in section 2).
- **Why:** the as-built record has no E-stop chain or STO yet, estimated brakes and no perception sensors; the owner confirms E-stop buttons and a safety relay with STO wiring on the robot (D80), whose data come from the supplier on card G1-04. In the twin every safety function except the hardware E-stop chain (with the STO and the brakes it acts on) is a single-channel software function in one microcontroller or in the companion, which reaches PL b at most.
- **Escalated** to the project owner (section 7): 7 class-1 items, each a safety goal the as-built hardware cannot meet, now worked on cards G2-FSC and G1-04 (D80); and 2 plain decisions: the greenhouse operating concept (E-09), decided (D79), and the editions and the jurisdiction (E-07), decided (D103).

## 1. Purpose and scope

This log records the hazards of the Shaddad UGV, the risk of each, the safety goal it gives and the safety function that meets that goal today, or the gap. It is the record of card G1-07 under the safety plan NASL-SHD-SAF-001, and it feeds the requirements baseline (G1-01), the SRR (G1-05) and the functional safety concept (G2-FSC).

- **The system:** The Shaddad UGV as designed in its digital twin (the MCU firmware, the companion software and the plant models), on the as-built robot of docs/hardware/real_robot_components.md (2026-10-27). The robot's controller today is the supplier's board (XT_PS with an STM32 Nucleo-32); the firmware generated from the twin is not on it yet.
- **The operating concept:** The charter NASL-SHD-CHR-001 v1.2, section 6: greenhouses, palm groves and the slope test ground, with the greenhouse rule (D79): people are kept out of the robot's zone while it mows a greenhouse aisle, and work in the other aisles; RC driving, autonomous missions, the hardware E-stop chain; mowing with the deck lowered, turns at the row ends, return to the dock to charge; the abnormal cases of section 6. The envelope of CR-001: the 20 deg design slope with no implement, the mower or the tiller forward; the full-tank sprayer to about 18.5 deg. The tiller is not used until real tiller data exist.
- **The people exposed:** the operator, who drives by RC, starts and stops missions, answers holds and faults in the field (charter 5); the supervisor, who plans missions and decides when the robot works; on the project, the project owner (charter 5); the maintainer, who charges the pack, fits and changes implements, inspects the brakes, tracks and sensors (charter 5); people nearby, who work in the greenhouse or the grove, or are otherwise in or near the operating zone (charter 5).
- **The safe state:** Standstill, held on the spring brakes, with the traction torque off (STO) and the implement off.
- **The record:** `docs/safety/hazard_log.json` holds the log; this document is generated from it by `docs/safety/hazard_log_render.py`, whose `--check` verifies the levels, the traces, the classes and the walk-through. Hazards are never renumbered. The Safety Analysis Manager copy docs/safety/hazard_log.mldatx is not made: Simulink Fault Analyzer is installed but not licensed to the project's MATLAB (its own licence check, in a batch run on 2026-11-16). Open point OP-01: the project owner decides on the licence by 2026-11-23; the functional safety engineer then makes the copy from this record in one batch run, before G1-05. Until then this record and its PDF are the hazard log.

## 2. The method

**Identification.** The hazards come from a walk-through of every operating mode and abnormal case of the charter (section 6) in each place it names, with the hazard list of ISO 12100:2010 (Annex B) and the hazards of automated agricultural machines of ISO 18497-1:2024: the moving machine and its implement, instability on slopes, unexpected start-up, failures of the control system and of perception, energy (the pack, the charger), and foreseeable misuse. The sources were the charter, the as-built record, the specs, the decisions D30 to D80, the open problems of the status record and the review report NASL-SHD-ANR-001 (2026-11-15).

**Rating.** Each hazardous event is rated with the three parameters of ISO 13849-1:2023 Annex A, without the safety function meant to reduce it. The safety plan (section 4) explains the choice of this method over the ISO 25119 risk graph.

**The exposure F.** F is the exposure of people to the zone that the hazardous event can reach (the robot's path and swept zone, its implement, the zone it can tip or roll into, the air of a fire): who is there, how often and for how long, while the robot is in that mode and place, whatever the cause of the event. How often the cause occurs (a failure, an emergency, a lost link) never sets F (rule 2).

Table 1. The risk parameters and how this project applies them.

| Parameter | Class | Applied when |
|---|---|---|
| Severity | S1 | Slight, normally reversible injury (bruises, small burns, a short slow movement of at most 0.1 m at 0.1 m/s). |
| Severity | S2 | Serious, normally irreversible injury, or death: a person crushed, run over, pinned against a post, a wall or the dock, struck by a tipping robot, or reached by the blades. |
| Frequency and exposure | F1 | Seldom to less often, or a short exposure: people are in the zone that the hazardous event can reach at most once per 15 min and for at most 1/20 of the working time. Each F1 gives its exposure reason: a short mode (RC driving to place the robot, a mission start, a reset, docking, a power-up), a place with few people in the robot's zone (a greenhouse under the greenhouse rule, D79; the slope test ground; a palm-grove row in a large field; berms), or a zone entered only for a task (maintenance). |
| Frequency and exposure | F2 | Frequent to continuous, or a long exposure: more often or longer than F1. In this log: the air of a greenhouse, which the people at work in its other aisles share with the working robot (a fire, HAZ-029), and the noise of the shift (HAZ-042). People at work beside the robot in a greenhouse would be F2 too; the greenhouse rule (D79) keeps them out of its zone. |
| Possibility of avoiding | P1 | Possible under specific conditions: the hazard develops slowly (a motion of at most 0.2 m/s, or a fire or heat that builds over minutes) and the person has room to move away. The hazard's own safety functions, warnings included, earn no credit (rule 1). |
| Possibility of avoiding | P2 | Scarcely possible: the person cannot see the hazard coming (back turned, kneeling, vegetation, a corner or a door), cannot move away (an aisle, a wall, a post, the dock), or the event is sudden (a tip-over, an unexpected start, an arc, a thrown object, a stop that fails in an emergency). |

Table 2. The risk graph of ISO 13849-1 Annex A: the required performance level.

| Severity | Frequency | P1 | P2 |
|---|---|---|---|
| S1 | F1 | a | b |
| S1 | F2 | b | c |
| S2 | F1 | c | d |
| S2 | F2 | d | e |

**Rules of this issue:**

1. Each hazardous event is rated without the safety functions meant to reduce it, warnings and stop commands included: they earn no credit in F or P.
2. F is set by the exposure of people to the hazard zone, never by how rare the cause or the event is; no level is lowered for a low probability of the hazardous event, although ISO 13849-1 allows one level with a justification.
3. A hazard that arises in several places is rated in the worst place; its reason names the other places. The goal takes the highest level of its hazards.
4. A hazard reduced by inherently safe design, a guard or the information for use, with no control function, keeps its S, F and P for the record and has PLr n/a.
5. The ceiling of each safety function's architecture is a preliminary judgement for this issue; the functional safety concept (G2-FSC) and the FMEA (G3-FME) settle it. A single-channel function in one microcontroller is Category B (PL b at most), because a microcontroller is not a well-tried component; PL c and above need a tested or a redundant architecture with rated parts.

**The operating concept behind the exposure classes:** the owner's greenhouse rule (1) and the assumptions of this log (2 to 5).

1. The greenhouse rule, the owner's decision (D79; the charter v1.2, section 6): people are kept out of the robot's zone while it mows a greenhouse aisle, and work in the other aisles. Here the robot's zone is the zone its hazardous events can reach: the aisle it mows and any lane its detour can enter, the cross aisles and doors it uses, and the beds and posts beside them. A person is in that zone only seldom and briefly, to cross it or by mistake: F1. People in the other aisles still share the greenhouse's air (HAZ-029) and hear the mower (HAZ-042). People at work beside the robot would set F2, PL e for six goals, and the need for safety-rated person detection (E-09).
2. The palm groves: the robot works one row of a large field; a worker is seldom in the row it mows or on the berm it crosses. The owner has still to confirm this (section 7, open points).
3. RC driving places the robot and moves it between places (the charter, section 6): a few short drives a shift, under 1/20 of the working time. If RC driving becomes a way of working, its hazards are F2 where people work.
4. The slope test ground is closed to people other than the test team.
5. The charging place is attended only to connect and disconnect the pack. The owner has still to confirm this (section 7, open points).

**What "met today" and "status today" mean.** Each hazard gets one class for what meets it today, judged on the twin and the as-built record; each goal's status follows from its hazards, and `--check` verifies it.

Table 3. The classes of "met today" (hazards) and "status today" (goals).

| For | Class | Means |
|---|---|---|
| A hazard | met | a safety function covers the hazardous event at the required level, and its parts are in the as-built record |
| A hazard | partly | a safety function in the twin covers the event by its design and a test or a scenario shows it acting, but its architecture cannot reach the PLr or its parts are not in the as-built record |
| A hazard | no | no safety function covers the event by its design (a zone it does not watch, a cause it cannot see), or no test or scenario shows it acting yet |
| A hazard | unknown | the risk rests on a bought part whose data are missing |
| A hazard | by other measures | no safety function is claimed (PLr n/a); the measures are design, guards, procedures and the information for use, still to be shown |
| A goal | met | every hazard of the goal is met |
| A goal | partly met | every hazard of the goal is partly met or better |
| A goal | unknown | no hazard is unmet, and at least one rests on missing data |
| A goal | not met | at least one hazard of the goal is not met |
| A goal | open | a goal with no PL: its measures are still to be shown |

## 3. The walk-through

Every row is a mode or a case of the charter (section 6: the normal use, the abnormal cases, the modes) or of the maintainer's work (section 5); every column is a place the charter names. A cell lists the hazards whose cases, modes and places put them there; a cell that reads "none" means the case does not arise in that place. The cells are derived from the hazards, and `--check` verifies them.

Table 4. The walk-through: the cases of the charter in each place.

| Case | Greenhouse aisles | Palm groves | The slope test ground | The dock and the charging place |
|---|---|---|---|---|
| Placing the robot by RC (normal use) | HAZ-014, HAZ-020, HAZ-029, HAZ-041, HAZ-042 | HAZ-014, HAZ-017, HAZ-020, HAZ-025, HAZ-029, HAZ-038, HAZ-041, HAZ-042 | HAZ-014, HAZ-017, HAZ-025, HAZ-038 | HAZ-014 |
| Starting a mission (normal use) | HAZ-011 | HAZ-011 | none | HAZ-011 |
| Mowing the rows with the deck lowered (normal use) | HAZ-001, HAZ-006, HAZ-009, HAZ-029, HAZ-041, HAZ-042 | HAZ-002, HAZ-006, HAZ-008, HAZ-025, HAZ-029, HAZ-038, HAZ-041, HAZ-042 | HAZ-025, HAZ-038 | none |
| Turns at the row ends and pivots (normal use) | HAZ-004, HAZ-006, HAZ-009 | HAZ-004, HAZ-006, HAZ-038 | HAZ-038 | none |
| Transit and the return to the dock (normal use) | HAZ-003, HAZ-009, HAZ-029 | HAZ-003, HAZ-008, HAZ-017, HAZ-025, HAZ-029, HAZ-038 | HAZ-017, HAZ-025, HAZ-038 | HAZ-003 |
| Bypass and rejoin (normal use) | HAZ-007 | HAZ-007 | none | none |
| Docking and charging (normal use) | none | none | none | HAZ-013, HAZ-028, HAZ-030 |
| Power-up and self-test (normal use) | HAZ-037 | HAZ-037 | HAZ-037 | HAZ-037 |
| A person in the path (hold, then resume) | HAZ-001, HAZ-003, HAZ-004, HAZ-005, HAZ-006, HAZ-007 | HAZ-002, HAZ-003, HAZ-004, HAZ-005, HAZ-006, HAZ-007 | none | HAZ-003, HAZ-013 |
| Lost RC | HAZ-016, HAZ-022, HAZ-043 | HAZ-016, HAZ-022, HAZ-043 | HAZ-016, HAZ-022, HAZ-043 | HAZ-016, HAZ-022, HAZ-043 |
| Lost GNSS | HAZ-009, HAZ-043 | HAZ-008, HAZ-043 | HAZ-043 | HAZ-043 |
| A sensor fault (LiDAR, IMU, encoders, lift sensor, implement code, companion) | HAZ-001, HAZ-009, HAZ-010, HAZ-012, HAZ-015, HAZ-018, HAZ-045 | HAZ-002, HAZ-008, HAZ-010, HAZ-012, HAZ-015, HAZ-018, HAZ-038, HAZ-039, HAZ-045 | HAZ-010, HAZ-012, HAZ-015, HAZ-018, HAZ-038, HAZ-039, HAZ-045 | HAZ-010, HAZ-012, HAZ-015, HAZ-018, HAZ-045 |
| A stop on a slope (park on the brakes, within the descent cap) | none | HAZ-017, HAZ-023, HAZ-024, HAZ-025, HAZ-027, HAZ-038 | HAZ-017, HAZ-023, HAZ-024, HAZ-025, HAZ-027, HAZ-038 | none |
| An E-stop with reset and re-arm | HAZ-021, HAZ-022, HAZ-026, HAZ-033 | HAZ-021, HAZ-022, HAZ-023, HAZ-026, HAZ-033 | HAZ-021, HAZ-022, HAZ-023, HAZ-026, HAZ-033 | HAZ-021, HAZ-022, HAZ-026, HAZ-033 |
| Low energy (return to the dock) | HAZ-003 | HAZ-003, HAZ-023, HAZ-035, HAZ-040 | HAZ-023, HAZ-035, HAZ-040 | HAZ-003, HAZ-013, HAZ-028 |
| Maintenance: charging | none | none | none | HAZ-028, HAZ-030, HAZ-031 |
| Maintenance: changing implements | HAZ-033, HAZ-034 | HAZ-033, HAZ-034 | HAZ-033, HAZ-034 | HAZ-033, HAZ-034 |
| Maintenance: inspecting the brakes, tracks and sensors | HAZ-032, HAZ-033, HAZ-036 | HAZ-032, HAZ-033, HAZ-036 | HAZ-032, HAZ-033, HAZ-036 | HAZ-032, HAZ-033, HAZ-036 |
| Transport and recovery | HAZ-019 | HAZ-019, HAZ-023, HAZ-035 | HAZ-019, HAZ-023, HAZ-035 | HAZ-019 |
| Foreseeable misuse | HAZ-018, HAZ-019, HAZ-043, HAZ-044, HAZ-045 | HAZ-018, HAZ-019, HAZ-039, HAZ-043, HAZ-044, HAZ-045 | HAZ-018, HAZ-019, HAZ-039, HAZ-043, HAZ-044, HAZ-045 | HAZ-018, HAZ-019, HAZ-043, HAZ-044, HAZ-045 |

## 4. The hazards of the mowing phase

Table 5. The hazards at a glance.

| ID | Hazard | S F P | PLr | Goal | Met today |
|---|---|---|---|---|---|
| HAZ-001 | Person in the path in a greenhouse aisle | S2 F1 P2 | d | SG-01 | no |
| HAZ-002 | Person in the path in a palm-grove row | S2 F1 P2 | d | SG-01 | no |
| HAZ-003 | Person in the path in transit, on the return and at greenhouse doors | S2 F1 P2 | d | SG-01 | no |
| HAZ-004 | Person beside the robot in a turn or a pivot | S2 F1 P2 | d | SG-01 | no |
| HAZ-005 | The robot resumes into a person after a hold | S2 F1 P2 | d | SG-02 | no |
| HAZ-006 | A person reaches the running deck from the side | S2 F1 P2 | d | SG-09 | no |
| HAZ-007 | A detour into a person or an occupied lane | S2 F1 P2 | d | SG-01 | no |
| HAZ-008 | The robot leaves its operating zone in a palm grove | S2 F1 P2 | d | SG-03 | no |
| HAZ-009 | The robot leaves its aisle in a greenhouse | S2 F1 P2 | d | SG-03 | no |
| HAZ-010 | Speed above what the stop distance allows | S2 F1 P2 | d | SG-01 | partly |
| HAZ-011 | A person close to the robot when a mission starts | S2 F1 P2 | d | SG-08 | no |
| HAZ-012 | Loss of the companion or of its commands in AUTO | S2 F1 P2 | d | SG-07 | no |
| HAZ-013 | Crushing between the robot and the dock | S2 F1 P2 | d | SG-01 | no |
| HAZ-014 | A person struck while the operator drives by RC | S2 F1 P2 | d | SG-11 | partly |
| HAZ-015 | Unintended motion in RC driving | S2 F1 P2 | d | SG-11 | no |
| HAZ-016 | The RC link is lost while the robot moves | S2 F1 P2 | d | SG-07 | partly |
| HAZ-017 | A descent too fast, or a hard stop on the slope | S2 F1 P2 | d | SG-04 | partly |
| HAZ-018 | Driving or stopping with the mower deck raised | S2 F1 P2 | d | SG-04 | no |
| HAZ-019 | Loading on a trailer, or crossing a ramp or a berm edge | S2 F1 P2 | d | SG-04 | no |
| HAZ-020 | Blades running while a person approaches in RC driving | S2 F1 P2 | d | SG-09 | no |
| HAZ-021 | The robot does not stop on its E-stop button or bumper | S2 F1 P2 | d | SG-06 | partly |
| HAZ-022 | The operator's remote stop does not act | S2 F1 P2 | d | SG-06 | partly |
| HAZ-023 | An E-stop or a loss of power on a descent tips the robot | S2 F1 P2 | d | SG-04 | no |
| HAZ-024 | A controlled stop on a slope does not hold the robot | S2 F1 P2 | d | SG-05 | partly |
| HAZ-025 | Runaway on a long descent when the battery cannot take charge | S2 F1 P2 | d | SG-05 | no |
| HAZ-026 | Unexpected motion after a reset or a restart | S2 F1 P2 | d | SG-08 | partly |
| HAZ-027 | Roll-back at a hill start | S1 F1 P1 | a | SG-05 | no |
| HAZ-028 | Battery fire while charging in the heat | S2 F1 P1 | c | SG-10 | unknown |
| HAZ-029 | Battery fire in operation | S2 F2 P1 | d | SG-10 | unknown |
| HAZ-030 | Live dock contacts and arcs at the charging connections | S1 F1 P2 | b | SG-10 | partly |
| HAZ-031 | Mains shock from the charger | S2 F1 P2 | n/a | SG-12 | by other measures |
| HAZ-032 | The tracks start during inspection or cleaning | S2 F1 P2 | d | SG-08 | no |
| HAZ-033 | Blades start, or still turn, while the maintainer works at the deck | S2 F1 P2 | d | SG-09 | no |
| HAZ-034 | Crushing when fitting, removing or lifting an implement | S2 F1 P2 | d | SG-08 | no |
| HAZ-035 | Recovering a stranded robot on a slope | S2 F1 P2 | n/a | SG-12 | by other measures |
| HAZ-036 | Hot surfaces | S1 F1 P1 | n/a | SG-12 | by other measures |
| HAZ-037 | Unexpected motion or brake release at power-up | S2 F1 P2 | d | SG-08 | partly |
| HAZ-038 | Tip-over or rollover on a side slope, a berm or a turn on a slope | S2 F1 P2 | d | SG-04 | partly |
| HAZ-039 | The robot applies the descent limits of the wrong implement | S2 F1 P2 | d | SG-04 | no |
| HAZ-040 | A low-energy stop on the way back | S2 F1 P2 | d | SG-05, SG-04 | no |
| HAZ-041 | Thrown objects from the mower deck | S2 F1 P2 | n/a | SG-12 | by other measures |
| HAZ-042 | Mower noise | S2 F2 P1 | n/a | SG-12 | by other measures |
| HAZ-043 | Misuse of the links (provisional, card G2-CYB) | S2 F1 P2 | d | SG-07 | unknown |
| HAZ-044 | Foreseeable misuse: riding on the robot, a bridged E-stop, AUTO with nobody at the transmitter | S2 F1 P2 | n/a | SG-12 | by other measures |
| HAZ-045 | The protective field starts at the wrong implement | S2 F1 P2 | d | SG-01 | no |

### 4.1 Autonomous missions

#### HAZ-001 Person in the path in a greenhouse aisle

- **Mode and place:** autonomous: rows; greenhouse aisles. **Cases:** normal use, a person in the path, a sensor fault.
- **Cause:** A worker is in the aisle the robot mows, kneeling, bending or with the back turned, or steps into it, and the person stop misses the worker or does not stop the robot: the worker is in the zone in front of the body or below the scan plane; the field is set from the build's implement, not the fitted one; the LiDAR gives valid but wrong data (a stale cloud re-published, film reflections, dust); or the companion, CAN2 or the MCU chain fails.
- **Hazardous event:** The robot or its mower deck strikes the worker, runs over a foot or a leg, or pins the worker against a post or the end wall.
- **Exposed:** people nearby, the operator. **Harm:** crushing and fractures; cuts or amputation at the deck.
- **Rating:** S2 F1 P2, **PLr d**. S2: crushing under a 220 to 316 kg tracked robot and blade injuries are irreversible. F1: the greenhouse rule (D79) keeps people out of the aisle the robot mows; a worker is in it only seldom and briefly, to cross it or by mistake (assumption 1). P2: the aisle and its end walls leave no room to step aside, and a kneeling worker facing away cannot see the robot.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes; SF-15 start warning and indication. **Other measures:** beacon and buzzer (warning); the deck housing as a guard.
- **Met today:** no. **Gap:** The person stop runs on a LiDAR and a companion that are not safety-rated, through the GNC and one MCU (PL b at most); the zone in front of the body is not covered (the owner's open decision 2); the LiDAR is not in the as-built record.
- **References:** D72; R-02; R-03; STATUS 18; STATUS 21; STATUS 25; STATUS 28; S5.

#### HAZ-002 Person in the path in a palm-grove row

- **Mode and place:** autonomous: rows; palm groves. **Cases:** normal use, a person in the path, a sensor fault.
- **Cause:** A worker lies, kneels or works in the row, or crosses it; tall grass or fronds hide a low person from the scan plane; the detection and the stop chain can fail as in HAZ-001.
- **Hazardous event:** The robot strikes the person, runs over a limb or cuts it at the deck.
- **Exposed:** people nearby, the operator. **Harm:** crushing; cuts or amputation.
- **Rating:** S2 F1 P2, **PLr d**. S2 as HAZ-001. F1: a palm-grove row is one of many in a large field, and a worker is seldom in the row the robot mows (assumption 2, which the owner has still to confirm). P2: a lying or kneeling person in tall grass cannot see the robot or hear it over the mower.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes; SF-15 start warning and indication. **Other measures:** beacon and buzzer; the deck housing.
- **Met today:** no. **Gap:** As HAZ-001: PL b at most with this architecture; no safety-rated detection; the LiDAR is not in the as-built record.
- **References:** D72; R-02; S5.

#### HAZ-003 Person in the path in transit, on the return and at greenhouse doors

- **Mode and place:** autonomous: transit and return; greenhouse aisles, palm groves, the dock and the charging place. **Cases:** normal use, a person in the path, low energy.
- **Cause:** Transit runs at up to 1.2 m/s with the implement off, on farm tracks, through greenhouse doors and towards the dock, where people walk; the field length grows with the measured speed; static walls at doors cause holds, which tempt people to defeat the field (STATUS 3).
- **Hazardous event:** The robot strikes a person at transit speed or pins them in a doorway.
- **Exposed:** people nearby, the maintainer. **Harm:** impact, crushing in a doorway.
- **Rating:** S2 F1 P2, **PLr d**. S2: pinning in a doorway. F1: in the greenhouse the greenhouse rule (D79) keeps people out of the cross aisles and doors the robot uses (assumption 1); in the groves and at the dock few people are on its way. P2: at a door or a corner the person cannot see the robot coming.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes; SF-15 start warning and indication. **Other measures:** beacon and buzzer.
- **Met today:** no. **Gap:** As HAZ-001.
- **References:** D72; STATUS 3; R-02.

#### HAZ-004 Person beside the robot in a turn or a pivot

- **Mode and place:** autonomous: turns and pivots; greenhouse aisles, palm groves. **Cases:** normal use, a person in the path.
- **Cause:** In a headland turn, an alignment pivot or a resume pivot the body and the mower deck sweep sideways; the fields look to the front and the rear only, and only in the direction of travel above 0.1 m/s; the deck is invisible to the collision checks (R-06: a resume pivot on a wall lane drives the deck into the post line, margin about -0.61 m) and the pivot has no clearance check (STATUS 16c).
- **Hazardous event:** The deck or the body strikes a person beside the robot, or pins them against a post or a wall.
- **Exposed:** people nearby, the operator. **Harm:** crushing against a post or wall; blade contact at the deck edge.
- **Rating:** S2 F1 P2, **PLr d**. S2: pinning against a post or a wall. F1: in the greenhouse the greenhouse rule (D79) keeps people out of the lane ends and the cross aisles where the robot turns (assumption 1); in the groves few people are near a turning robot. P2: a pivot moves the deck sideways at close range.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes. **Other measures:** none.
- **Met today:** no. **Gap:** No function covers the swept zone at the sides; PL b at most.
- **References:** R-06; STATUS 16c; D53; D67.

#### HAZ-005 The robot resumes into a person after a hold

- **Mode and place:** autonomous: hold and resume; greenhouse aisles, palm groves. **Cases:** a person in the path.
- **Cause:** H1 resumes after 2 s clear on valid scans; a person still in the zone in front of the body (not covered) or crouched below the scan plane counts as clear; a second person enters during the clear time; the mower restarts at H1, where the D72 latch ends; the operator walks up to answer the hold.
- **Hazardous event:** The robot moves off and its blades restart into the person.
- **Exposed:** people nearby, the operator. **Harm:** crushing; cuts or amputation.
- **Rating:** S2 F1 P2, **PLr d**. S2 as HAZ-001. F1: the greenhouse rule (D79) keeps people out of the robot's zone, so the robot holds for a person there only seldom (assumption 1); in the groves few people come close. P2: the person believes the robot is stopped for them.
- **Safety goal:** SG-02. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-09 blade stop and the implement latch. **Other measures:** none.
- **Met today:** no. **Gap:** No function checks the near zone before the resume (the owner's open decision 2, R-02); PL b at most.
- **References:** D72; R-02; R-03; STATUS 21.

#### HAZ-006 A person reaches the running deck from the side

- **Mode and place:** autonomous: rows, autonomous: turns and pivots; greenhouse aisles, palm groves. **Cases:** normal use, a person in the path.
- **Cause:** A person approaches the mowing robot from the side, to stop it or to clear debris; the fields look front and rear only, so the blades keep running until the person enters a field.
- **Hazardous event:** A hand or a foot reaches the blades through the side or the discharge opening of the deck.
- **Exposed:** people nearby, the operator. **Harm:** cuts or amputation.
- **Rating:** S2 F1 P2, **PLr d**. S2: amputation. F1: the greenhouse rule (D79) keeps people out of the aisle the robot mows and the beds beside it, so a person is within reach of the deck only seldom (assumption 1); in the groves few people are beside the robot. P2: a blade at 76 m/s tip speed gives no time to withdraw.
- **Safety goal:** SG-09. **Safety function today:** SF-09 blade stop and the implement latch. **Other measures:** the deck housing as a fixed guard (the primary measure), to the mower standard G1-01 names.
- **Met today:** no. **Gap:** No function stops the blades for a person beside the deck (the fields look to the front and the rear); PL b at most; the mower implement (blades, guard, motor, brake) is not in the as-built record.
- **References:** D72; gnc 2.9; power 2.5.

#### HAZ-007 A detour into a person or an occupied lane

- **Mode and place:** autonomous: bypass and rejoin; greenhouse aisles, palm groves. **Cases:** normal use, a person in the path.
- **Cause:** After 10 s in a hold the robot may detour round the obstacle (H2) where the segment allows it, also round a person standing still, or into the next lane where a person works; the bypass speed skips the governor (R-41); the rejoin goal can lie behind the robot (R-49).
- **Hazardous event:** The robot strikes a person on its detour.
- **Exposed:** people nearby. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: impact and pinning. F1: where a mission allows detours in a greenhouse, the greenhouse rule (D79) keeps people out of the lanes the detour can enter (assumption 1); in the groves few people are in the neighbouring rows. P2: the person who made the robot stop does not expect it to move round them.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes. **Other measures:** detours are off in the greenhouse mission of S4 (a mission setting, not a safety function).
- **Met today:** no. **Gap:** As HAZ-002; no rule keeps a detour away from a person who stands still.
- **References:** R-24; R-41; R-49; gnc 4.1 H2.

#### HAZ-008 The robot leaves its operating zone in a palm grove

- **Mode and place:** autonomous: rows, autonomous: transit and return; palm groves. **Cases:** normal use, lost GNSS, a sensor fault.
- **Cause:** A wrong position (a false RTK fix, R-43; multipath under the canopy; the antenna lever-arm bias, R-21), a wrong mission plan or boundary, or a geofence that fails; the robot drives onto a road, a canal bank or a neighbour's plot, or off a berm edge.
- **Hazardous event:** The robot strikes people or vehicles outside its zone, or falls from a berm.
- **Exposed:** people nearby, people outside the zone. **Harm:** impact; a fall or tip-over of the robot.
- **Rating:** S2 F1 P2, **PLr d**. S2: impact; a robot falling from a berm. F1: people beyond the field's edge (a road, a canal, a neighbour's plot) are seldom close to it. P2: the robot arrives at up to 1.2 m/s, and a person outside the field does not expect it.
- **Safety goal:** SG-03. **Safety function today:** SF-14 geofence, path and navigation integrity. **Other measures:** the mission check in the twin before the field.
- **Met today:** no. **Gap:** Companion software on GNSS (PL b at most); no check against a false RTK fix (R-43); no GNSS receiver is in the as-built record.
- **References:** R-21; R-43; S7; gnc 4.1 G1.

#### HAZ-009 The robot leaves its aisle in a greenhouse

- **Mode and place:** autonomous: rows, autonomous: turns and pivots, autonomous: transit and return; greenhouse aisles. **Cases:** normal use, lost GNSS, a sensor fault.
- **Cause:** A scan-match localisation error (R-23, R-51), an IMU loss of 1 s or more that ends navigation in the greenhouse (STATUS 26), cross-track errors at the row entry (R-52).
- **Hazardous event:** The robot leaves the aisle into the beds, a post, the end wall, the film or a door, and can pin a worker there.
- **Exposed:** people nearby. **Harm:** crushing against a post or wall.
- **Rating:** S2 F1 P2, **PLr d**. S2: pinning against a post or a wall. F1: the greenhouse rule (D79) keeps people out of the aisle the robot mows and the beds and posts beside it (assumption 1); a person is there only seldom and briefly. P2: a person between the robot and a post or the end wall cannot move away.
- **Safety goal:** SG-03. **Safety function today:** SF-14 geofence, path and navigation integrity. **Other measures:** none.
- **Met today:** no. **Gap:** The cross-track check runs on the estimated position, so a localisation error is not seen; navigation cannot re-align without RTK (R-44); PL b at most.
- **References:** R-23; R-44; R-51; R-52; STATUS 26.

#### HAZ-010 Speed above what the stop distance allows

- **Mode and place:** autonomous: rows, autonomous: transit and return, RC driving; any place. **Cases:** a sensor fault.
- **Cause:** The speed limit of the mode fails, or the measured speed is wrong (an encoder or odometry error, track slip, a downhill gain), so the field length (0.5 + 0.8 |v| m) and the stop distance no longer match.
- **Hazardous event:** The robot reaches a person before it stops.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2 as HAZ-001. F1: the greenhouse rule (D79) keeps people out of the robot's path in the greenhouse (assumption 1), and elsewhere few people are in it. P2: the person relies on the robot stopping in time.
- **Safety goal:** SG-01. **Safety function today:** SF-13 speed limits by mode and the turn limits on slopes; SF-12 overspeed and speed-follow trip; SF-18 encoder plausibility. **Other measures:** none.
- **Met today:** partly. **Gap:** The speed limits and the overspeed trip act in S1 and the D71 tests, but one MCU on the encoders (PL b at most).
- **References:** D33; D43; D71.

#### HAZ-011 A person close to the robot when a mission starts

- **Mode and place:** autonomous: start; greenhouse aisles, palm groves, the dock and the charging place. **Cases:** normal use.
- **Cause:** The supervisor starts a mission while a worker or the maintainer is beside or in front of the robot.
- **Hazardous event:** After the 3 s warning the robot moves off into the person.
- **Exposed:** people nearby, the maintainer, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: pinning. F1: a start lasts a few seconds, a few times a shift. P2: in an aisle or at the dock the person cannot move away; the start warning earns no credit (rule 1).
- **Safety goal:** SG-08. **Safety function today:** SF-15 start warning and indication; SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-16 start and restart interlocks. **Other measures:** the supervisor's look at the robot before the start (information for use).
- **Met today:** no. **Gap:** The warning is information only; the field does not cover the near zone or the sides; PL b at most.
- **References:** electronics 2.7.3; D72.

#### HAZ-012 Loss of the companion or of its commands in AUTO

- **Mode and place:** autonomous: rows, autonomous: transit and return, autonomous: turns and pivots; any place. **Cases:** a sensor fault.
- **Cause:** The companion stops, or hangs while its CAN frames stay fresh (alive counters not checked, R-39); the CAN2 link fails; ROS frames are dropped (R-48); a stale command persists.
- **Hazardous event:** The robot runs on with its last command, past a person or out of its zone.
- **Exposed:** people nearby. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: impact and crushing by a robot out of control. F1: the greenhouse rule (D79) keeps people out of the working robot's zone in the greenhouse (assumption 1), and elsewhere few people are in its path. P2: an uncontrolled robot gives no warning.
- **Safety goal:** SG-07. **Safety function today:** SF-06 lost-link stops; SF-04 controlled stop with the stop monitor. **Other measures:** none.
- **Met today:** no. **Gap:** The companion time-out (T-EL-32) catches a silent companion, but a hung companion that keeps sending fresh frames is not seen (R-39); PL b at most.
- **References:** R-39; R-48; electronics 2.7.10 bit 4.

#### HAZ-013 Crushing between the robot and the dock

- **Mode and place:** docking and charging; the dock and the charging place. **Cases:** normal use, low energy, a person in the path.
- **Cause:** The robot approaches its dock at 0.2 m/s while the maintainer stands between it and the dock; docking is unsupervised and has never met a dock body (R-10); masking the dock zone for the approach would blind the field there.
- **Hazardous event:** The maintainer's legs are crushed between the robot and the dock.
- **Exposed:** the maintainer, people nearby. **Harm:** crushing of the legs.
- **Rating:** S2 F1 P2, **PLr d**. S2: crushing against a fixed body. F1: people are at the dock only when a robot docks, a few times a day. P2: a person caught between the robot and the dock cannot move away.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume; SF-13 speed limits by mode and the turn limits on slopes. **Other measures:** none.
- **Met today:** no. **Gap:** PL b at most; docking has never met a dock body (R-10); the dock is not in the as-built record.
- **References:** R-10; gnc 4.1 R3 and R4.

### 4.2 RC driving

#### HAZ-014 A person struck while the operator drives by RC

- **Mode and place:** RC driving; greenhouse aisles, palm groves, the slope test ground, the dock and the charging place. **Cases:** normal use.
- **Cause:** The operator misjudges a distance, cannot see a person behind or beside the robot, or steers the wrong way when the robot faces them; the person stop does not act in RC driving.
- **Hazardous event:** The robot strikes or pins the person, or runs over a foot.
- **Exposed:** the operator, people nearby. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: pinning. F1: RC driving places the robot and moves it between places, a few short drives a shift, under 1/20 of the working time. P2: in an aisle the person cannot move away, and RC speeds reach 1.4 m/s; the stick-release stop earns no credit (rule 1).
- **Safety goal:** SG-11. **Safety function today:** SF-19 manual control: the stick-release stop; SF-05 remote stop from the RC transmitter; SF-06 lost-link stops. **Other measures:** the operator's training and line of sight (information for use).
- **Met today:** partly. **Gap:** The stick-release stop (S1) and the remote stop (S8b) act in the twin, but the RC chain is not safety-rated (PL b at most).
- **References:** D33; D34; S1; S8b.

#### HAZ-015 Unintended motion in RC driving

- **Mode and place:** RC driving; any place. **Cases:** a sensor fault.
- **Cause:** Corrupted SBUS bytes accepted as valid frames (STATUS 8, S6c, R-47); a wrong channel map (R-70); a receiver failsafe preset that makes an arm edge (R-35); a stuck stick or trim; a switch from AUTO to MANUAL with the sticks off centre.
- **Hazardous event:** The robot moves without the operator's command, towards a person.
- **Exposed:** the operator, people nearby. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: impact and crushing. F1: RC driving is short, as HAZ-014. P2: the motion is unexpected.
- **Safety goal:** SG-11. **Safety function today:** SF-16 start and restart interlocks; SF-19 manual control: the stick-release stop; SF-05 remote stop from the RC transmitter. **Other measures:** none.
- **Met today:** no. **Gap:** Corrupted bytes can pass as valid frames (STATUS 8); PL b at most.
- **References:** STATUS 8; R-35; R-47; R-70.

#### HAZ-016 The RC link is lost while the robot moves

- **Mode and place:** RC driving; any place. **Cases:** lost RC.
- **Cause:** Out of range, interference, a flat transmitter battery, a receiver failsafe. In AUTO a lost link stops the robot (D44); a failure of that stop leaves the operator without the remote stop, which is HAZ-022.
- **Hazardous event:** The robot runs on without control.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: impact and crushing by a robot out of control. F1: RC driving is short, as HAZ-014. P2: an uncontrolled robot gives no warning.
- **Safety goal:** SG-07. **Safety function today:** SF-06 lost-link stops; SF-04 controlled stop with the stop monitor. **Other measures:** the operator stays in range (in AUTO a lost link stops the robot, D44).
- **Met today:** partly. **Gap:** The stop acts in S6, but one MCU (PL b at most); the radio model of the row tests cannot produce a link loss (R-12).
- **References:** D34; D37; D44; S6; R-12.

#### HAZ-017 A descent too fast, or a hard stop on the slope

- **Mode and place:** RC driving, autonomous: transit and return; the slope test ground, palm groves. **Cases:** normal use, a stop on a slope.
- **Cause:** The robot descends the 20 deg ramp or a berm and stops or E-stops; a brakes-locked stop above the tip speed for the slope and the implement tips it over its front (S2c).
- **Hazardous event:** The robot pitches over its nose onto a person in front of or below it.
- **Exposed:** the operator, people nearby. **Harm:** crushing under the robot.
- **Rating:** S2 F1 P2, **PLr d**. S2: a 220 to 316 kg robot falling on a person. F1: few people are near a robot on a slope: the slope test ground is closed to others, and berms are a small part of a grove. P2: a tip-over is sudden.
- **Safety goal:** SG-04. **Safety function today:** SF-11 descent speed cap and trip; SF-10 tilt stop and the IMU-loss handling. **Other measures:** the CR-001 envelope in the information for use.
- **Met today:** partly. **Gap:** The cap acts on the benches (T-EL-54 to 60), but G2-DSC is open (D74 pending); one IMU and one MCU (PL b at most); the tip speeds rest on estimated contact data until T-04.
- **References:** G2-DSC; CR-001; S2c; D71; STATUS 19.

#### HAZ-018 Driving or stopping with the mower deck raised

- **Mode and place:** RC driving, autonomous: transit and return; any place. **Cases:** a sensor fault, foreseeable misuse.
- **Cause:** The deck is raised for transport or by the lift switch; raised, the robot is nose-heavy and a locked stop tips it from about 0.23 m/s on the flat; the deck-down rule is in the GNC, with the MCU's raised rows as a backstop (0.15 m/s on the flat, 0 from 5 deg); a lift-sensor fault reports the deck down.
- **Hazardous event:** The robot tips forward in a stop.
- **Exposed:** the operator, people nearby. **Harm:** crushing; the deck strikes a person.
- **Rating:** S2 F1 P2, **PLr d**. S2: the robot tips over onto a person. F1: the robot moves with the deck raised only in transport, a short part of the use. P2: a tip-over is sudden.
- **Safety goal:** SG-04. **Safety function today:** SF-11 descent speed cap and trip. **Other measures:** the deck down whenever the robot moves (the owner's decision of 2026-11-15).
- **Met today:** no. **Gap:** The raised-deck rows act in t_mcu_control, in the MCU's own sense of the lift position, but the twin reads that position inverted: the plant gives 0 with the deck at the top, and the MCU counts the deck raised at 0.667 and above, so the rows apply with the deck low and not with it raised (the structure spec; pending, cards G2-STR or G2-MCU, the check on G2-INT). In RC driving the GNC's deck-down rule does not act, so nothing in the twin covers a raised deck there. Once fixed, the rows rest on one lift sensor and one MCU (PL b at most).
- **References:** params/param_structure.m descent rows; params/param_electronics.m lim.descent.

#### HAZ-019 Loading on a trailer, or crossing a ramp or a berm edge

- **Mode and place:** transport and recovery; any place. **Cases:** normal use, foreseeable misuse.
- **Cause:** Driving onto a trailer or a loading ramp, or over a berm edge steeper than the envelope; a track leaves the side of the ramp.
- **Hazardous event:** The robot falls off the ramp or tips over onto the operator.
- **Exposed:** the operator, the maintainer. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: a 220 kg robot falling from a ramp. F1: loading takes minutes, a few times a week. P2: the operator is close to the robot on the ramp.
- **Safety goal:** SG-04. **Safety function today:** SF-11 descent speed cap and trip; SF-10 tilt stop and the IMU-loss handling. **Other measures:** a loading procedure with the ramp angle and width (information for use).
- **Met today:** no. **Gap:** The cap and the tilt stop act on the pitch only; nothing sees a track leaving the side of a ramp; no loading procedure.
- **References:** D47.

#### HAZ-020 Blades running while a person approaches in RC driving

- **Mode and place:** RC driving; greenhouse aisles, palm groves. **Cases:** normal use.
- **Cause:** In RC driving the rotor runs with the implement switch on and the person stop does not act; a helper approaches to guide the robot or clear the deck.
- **Hazardous event:** Blade contact through the deck openings.
- **Exposed:** people nearby, the operator. **Harm:** cuts or amputation.
- **Rating:** S2 F1 P2, **PLr d**. S2: amputation. F1: RC driving with the implement on is short. P2: a blade leaves no time to withdraw.
- **Safety goal:** SG-09. **Safety function today:** SF-09 blade stop and the implement latch; SF-01 hardware E-stop chain. **Other measures:** the deck housing as a guard; the operator's training.
- **Met today:** no. **Gap:** No function stops the blades for a person in RC driving; the mower implement is not in the as-built record.
- **References:** electronics 2.7.12.

### 4.3 Stops, E-stop, reset and re-arm

#### HAZ-021 The robot does not stop on its E-stop button or bumper

- **Mode and place:** any mode; any place. **Cases:** an E-stop with reset and re-arm.
- **Cause:** A part of the chain is missing or fails: the bumpers and the brake-release cut are not confirmed on the robot (the owner confirms the E-stop buttons and a safety relay with STO wiring, D80); a welded relay contact, an STO input that does not act, a brake that does not engage.
- **Hazardous event:** The robot keeps moving into the person who pressed the E-stop.
- **Exposed:** the operator, people nearby, the maintainer. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: the person who pressed the E-stop is struck. F1: the greenhouse rule (D79) keeps people out of the working robot's zone in the greenhouse (assumption 1), and elsewhere few people are close to it; the E-stop is needed by a person in that zone. P2: in an emergency the person is already in danger.
- **Safety goal:** SG-06. **Safety function today:** SF-01 hardware E-stop chain; SF-02 safe torque off (STO) of the ESCs; SF-03 spring brakes and the park at the end of every stop. **Other measures:** none.
- **Met today:** partly. **Gap:** Designed in the twin as two channels with a discrepancy check, and shown in S8a. On the robot the owner confirms E-stop buttons and a safety relay with STO wiring (D80), but their parts, category and wiring are not recorded yet (card G1-04); the ESC's STO and the brakes are unconfirmed (R-14); the relay re-arms without a manual reset (F-01).
- **References:** electronics D7; D30; S8; power 2.5; R-33.

#### HAZ-022 The operator's remote stop does not act

- **Mode and place:** RC driving, autonomous: rows, autonomous: transit and return; any place. **Cases:** an E-stop with reset and re-arm, lost RC.
- **Cause:** The remote E-stop (CH12) travels through the SBUS radio and the MCU, which are not safety-rated; it acts on valid frames only and is lost out of range; no certified wireless E-stop is fitted.
- **Hazardous event:** The robot does not stop when the operator presses the remote stop, and keeps moving into a person.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2 as HAZ-021. F1 as HAZ-021. P2: the stop is used when the person is already in danger.
- **Safety goal:** SG-06. **Safety function today:** SF-05 remote stop from the RC transmitter; SF-06 lost-link stops. **Other measures:** none.
- **Met today:** partly. **Gap:** The remote stop acts in S8b, but it is a functional stop through SBUS (PL b at most) and is lost out of range; a remote emergency stop needs a safety-rated radio stop (the spec's recommendation: a certified wireless E-stop in series with the loop): new hardware.
- **References:** electronics 2.5; D34; D44; architecture 13 R1.

#### HAZ-023 An E-stop or a loss of power on a descent tips the robot

- **Mode and place:** any mode; the slope test ground, palm groves. **Cases:** an E-stop with reset and re-arm, a stop on a slope, low energy.
- **Cause:** An E-stop or a loss of power applies STO and the spring brakes at once (traction stops as category 0, R-33) at a speed above the safe start speed of a brakes-locked stop for the slope and the implement.
- **Hazardous event:** The robot pitches over its front.
- **Exposed:** the operator, people nearby. **Harm:** crushing under the robot.
- **Rating:** S2 F1 P2, **PLr d**. S2: the robot tips over onto a person. F1: few people are near a robot on a descent, as HAZ-017. P2: a tip-over is sudden.
- **Safety goal:** SG-04. **Safety function today:** SF-11 descent speed cap and trip. **Other measures:** the CR-001 envelope.
- **Met today:** no. **Gap:** No run shows yet that an E-stop on the descent does not tip (an exit criterion of G2-DSC); PL b at most.
- **References:** G2-DSC; CR-001; R-33.

#### HAZ-024 A controlled stop on a slope does not hold the robot

- **Mode and place:** stops: controlled stop and park; the slope test ground, palm groves. **Cases:** a stop on a slope.
- **Cause:** The soft stop ends without the brakes (fixed by D71: brakes first, STOP_FAIL as the STO backup), or a brake is worn, stuck released or missing; the brake confirmation only echoes the brake command (STATUS 27).
- **Hazardous event:** The robot rolls away downhill.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: a robot rolling into a person. F1: few people are below a robot on a slope, as HAZ-017. P2: a runaway gathers speed.
- **Safety goal:** SG-05. **Safety function today:** SF-03 spring brakes and the park at the end of every stop; SF-04 controlled stop with the stop monitor; SF-02 safe torque off (STO) of the ESCs. **Other measures:** none.
- **Met today:** partly. **Gap:** D71 parks every soft stop on the brakes (S2 hold), but nothing monitors the brakes (STATUS 27), the detection is one MCU (PL b at most), and the brakes are estimates in the as-built record.
- **References:** D71; R-01; STATUS 27; T-04.

#### HAZ-025 Runaway on a long descent when the battery cannot take charge

- **Mode and place:** RC driving, autonomous: transit and return, autonomous: rows; the slope test ground, palm groves. **Cases:** normal use, a stop on a slope.
- **Cause:** With a full or a hot pack (CCL 0; the sealed pack at 45 deg C stays above 50 deg C, R-04) the drive cannot brake by regeneration and the speed rises downhill; the speed-follow trip then parks at speed (STATUS 19), and with CCL 0 it also trips on ordinary stops (STATUS 24).
- **Hazardous event:** The robot gains speed downhill, then locks its brakes at speed: it skids into a person or tips over.
- **Exposed:** people nearby, the operator. **Harm:** impact; crushing under the robot.
- **Rating:** S2 F1 P2, **PLr d**. S2: an impact at speed or a tip-over. F1: few people are below a robot on a descent, as HAZ-017. P2: a runaway gathers speed.
- **Safety goal:** SG-05. **Safety function today:** SF-12 overspeed and speed-follow trip; SF-11 descent speed cap and trip; SF-03 spring brakes and the park at the end of every stop; SF-04 controlled stop with the stop monitor. **Other measures:** none.
- **Met today:** no. **Gap:** With CCL 0 the speed-follow trip parks at speed on steep descents (STATUS 19) and trips on ordinary stops (STATUS 24): open in G2-DSC; a stop without skidding is proposed (G2-BRK); PL b at most.
- **References:** STATUS 16a; STATUS 19; STATUS 24; R-04; G2-BRK.

#### HAZ-026 Unexpected motion after a reset or a restart

- **Mode and place:** stops: E-stop, reset and re-arm, power-up and self-test; any place. **Cases:** an E-stop with reset and re-arm.
- **Cause:** The safety relay is re-armed by the MCU's reset pulse (DO1): on the reset button, on RC CH9 over the SBUS radio, which is not safety-rated, from wherever the transmitter is, and once automatically on each SELFTEST entry (D30). After a controller restart (power-up or watchdog) the relay re-arms with no manual action, provided every loop contact is closed. A controller reset clears the latches (R-36); an arm edge can come from the receiver (R-35); a stale command or an AUTO resume can follow the reset.
- **Hazardous event:** The robot or its blades start while a person who believes it stopped is close.
- **Exposed:** the operator, the maintainer, people nearby. **Harm:** impact, crushing; cuts.
- **Rating:** S2 F1 P2, **PLr d**. S2: crushing, and cuts if the blades start. F1: a reset is a deliberate act at a stopped robot, a few times a shift. P2: the person believes the robot is stopped.
- **Safety goal:** SG-08. **Safety function today:** SF-16 start and restart interlocks; SF-07 watchdog and the reset behaviour. **Other measures:** none.
- **Met today:** partly. **Gap:** Motion still needs a new arm edge (S6, S8a), but that interlock is one MCU (PL b at most); after a restart, the stop of a maintained E-stop rests on that interlock, not on the hardware latch (F-01).
- **References:** D30; R-35; R-36; S6; S8.

#### HAZ-027 Roll-back at a hill start

- **Mode and place:** RC driving, autonomous: rows; the slope test ground, palm groves. **Cases:** a stop on a slope.
- **Cause:** The brakes release before the speed loop holds the robot (R-37: about 0.1 m at 20 deg; the hill-start fix of CR-001 is in progress).
- **Hazardous event:** The robot rolls back about 0.1 m towards a person behind it.
- **Exposed:** people nearby, the operator. **Harm:** a bruise.
- **Rating:** S1 F1 P1, **PLr a**. S1: a short slow movement (about 0.1 m at 0.09 m/s). F1: few people stand behind a robot on a slope. P1: slow, with room to step back.
- **Safety goal:** SG-05. **Safety function today:** SF-03 spring brakes and the park at the end of every stop; SF-16 start and restart interlocks. **Other measures:** none.
- **Met today:** no. **Gap:** No function prevents the roll-back yet; the hill-start fix of CR-001 is in progress on G2-DSC; PL a is within the reach of one MCU.
- **References:** R-37; CR-001; G2-DSC.

### 4.4 Docking and charging

#### HAZ-028 Battery fire while charging in the heat

- **Mode and place:** docking and charging, maintenance: charging; the dock and the charging place. **Cases:** normal use, low energy, maintenance.
- **Cause:** Charging the sealed NMC pack at 45 to 50 deg C ambient (R-04); a charger or BMS fault (overcharge, charging outside the cell's temperature window); a cell defect; the dock air of the model does not exist for the real pack.
- **Hazardous event:** Thermal runaway, fire and toxic smoke at the charging place.
- **Exposed:** the maintainer, people nearby. **Harm:** burns, smoke inhalation.
- **Rating:** S2 F1 P1, **PLr c**. S2: burns and smoke. F1: people stay at the charging place only to connect and disconnect the pack (assumption 5, which the owner has still to confirm). P1: a fire builds over minutes, and people can leave.
- **Safety goal:** SG-10. **Safety function today:** SF-17 battery protection. **Other measures:** the charger's own constant-voltage limit (54.6 V); charging in a ventilated place away from flammables (information for use).
- **Met today:** unknown. **Gap:** The bought BMS and charger: their protections, settings and frames are unknown (R-15).
- **References:** R-04; R-15; D45.

#### HAZ-029 Battery fire in operation

- **Mode and place:** autonomous: rows, autonomous: transit and return, RC driving; greenhouse aisles, palm groves. **Cases:** normal use.
- **Cause:** A short circuit or an overcurrent (the fuse and BMS coordination is open: at 187.5 A the BMS opens before the 125 A fuse, T-PWR-06b fails); overheating at 45 to 50 deg C; damage after a tip-over or an impact.
- **Hazardous event:** Fire on the robot in a greenhouse with plastic film, or in dry grass.
- **Exposed:** people nearby, the operator. **Harm:** burns, smoke inhalation.
- **Rating:** S2 F2 P1, **PLr d**. S2: burns and smoke. F2: people at work in the other aisles of the greenhouse share its air with the working robot; the greenhouse rule (D79) does not change that (F1 in the groves). P1: a fire builds over minutes, and people can leave.
- **Safety goal:** SG-10. **Safety function today:** SF-17 battery protection. **Other measures:** the 3 mm aluminium pack box; fuses.
- **Met today:** unknown. **Gap:** Fuse and BMS coordination open; the BMS's settings unknown (R-15).
- **References:** real_robot_components 6 (T-PWR-06b); R-04; R-15.

#### HAZ-030 Live dock contacts and arcs at the charging connections

- **Mode and place:** maintenance: charging, docking and charging; the dock and the charging place. **Cases:** normal use, maintenance.
- **Cause:** The dock contacts are live while the robot is not docked (a welded K4 is not detected, R-45); an arc when the charging connections are made or broken. 54.6 V is below the shock limit in dry conditions, but a short of a 3 kWh pack gives a high current.
- **Hazardous event:** An arc flash or a burn; a small shock in wet conditions.
- **Exposed:** the maintainer. **Harm:** burns.
- **Rating:** S1 F1 P2, **PLr b**. S1: local burns. F1: work at the pack and the contacts is short: connecting and disconnecting. P2: an arc is sudden.
- **Safety goal:** SG-10. **Safety function today:** SF-17 battery protection. **Other measures:** insulated tools, terminal covers, touch-protected contacts; a short of the pack by a tool is reduced by these, not by a function.
- **Met today:** partly. **Gap:** K4 closes only with dock contact (power spec 2.5; the charging tests), but no weld check (R-45).
- **References:** R-45; power 2.5 (K4).

#### HAZ-031 Mains shock from the charger

- **Mode and place:** maintenance: charging; the dock and the charging place. **Cases:** maintenance.
- **Cause:** The charger (100 to 240 VAC, IP54, fan cooled) is used outdoors in the wet, or its cable is damaged.
- **Hazardous event:** An electric shock from the mains.
- **Exposed:** the maintainer. **Harm:** electric shock.
- **Rating:** S2 F1 P2, **PLr n/a**. S2 F1 P2 for the record (a shock is sudden); no control function reduces it.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** the charger's own conformity; an RCD-protected socket; use in a dry place (information for use).
- **Met today:** by other measures. **Gap:** The charger's conformity data are not in the records.
- **References:** real_robot_components 2.

### 4.5 Maintenance

#### HAZ-032 The tracks start during inspection or cleaning

- **Mode and place:** maintenance: inspection and cleaning; any place. **Cases:** maintenance.
- **Cause:** The robot is powered while the maintainer inspects the brakes, the tracks or the sensors, and a command (RC, the companion, a reset) starts the tracks.
- **Hazardous event:** Drawing-in at a sprocket, crushing under a track.
- **Exposed:** the maintainer. **Harm:** crushing, drawing-in of a hand.
- **Rating:** S2 F1 P2, **PLr d**. S2: a hand drawn into a sprocket. F1: an inspection is short, about once a day. P2: the start is unexpected, and the hand is already at the track.
- **Safety goal:** SG-08. **Safety function today:** SF-16 start and restart interlocks; SF-01 hardware E-stop chain. **Other measures:** isolation of the pack (the SB120 connector) per a maintenance procedure; the E-stop pressed when power is needed.
- **Met today:** no. **Gap:** The start interlocks act in the twin (PL b at most), but no isolation procedure or lockable isolator is recorded (the manuals come with V-06).
- **References:** charter 5; real_robot_components 2 (SB120).

#### HAZ-033 Blades start, or still turn, while the maintainer works at the deck

- **Mode and place:** maintenance: implement change, maintenance: inspection and cleaning, stops: controlled stop and park; any place. **Cases:** maintenance, an E-stop with reset and re-arm.
- **Cause:** The rotor gets power while the maintainer changes a blade or clears the deck; after a stop the blades coast down with no brake, and the run-down time is unknown and unchecked (R-33).
- **Hazardous event:** Cuts or amputation.
- **Exposed:** the maintainer, the operator. **Harm:** cuts, amputation.
- **Rating:** S2 F1 P2, **PLr d**. S2: amputation. F1: work at the deck is short, a few times a week. P2: a coasting blade is hard to see.
- **Safety goal:** SG-09. **Safety function today:** SF-09 blade stop and the implement latch; SF-01 hardware E-stop chain. **Other measures:** isolation before work at the deck.
- **Met today:** no. **Gap:** Run-down time unknown; no blade brake specified; the mower is not in the as-built record.
- **References:** R-33; power 2.5.

#### HAZ-034 Crushing when fitting, removing or lifting an implement

- **Mode and place:** maintenance: implement change; any place. **Cases:** maintenance.
- **Cause:** The deck lift moves under power (the 3-position switch); the 50 kg module or the deck falls or swings; the robot moves during the change.
- **Hazardous event:** A hand or a foot is crushed.
- **Exposed:** the maintainer. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: a hand or a foot crushed under a 50 kg module. F1: an implement change is short, a few times a week. P2: a falling module or a robot that moves is sudden.
- **Safety goal:** SG-08. **Safety function today:** SF-16 start and restart interlocks. **Other measures:** isolation; lifting aids; a procedure.
- **Met today:** no. **Gap:** No procedure recorded; the coupling and the lift of the as-built implements are not in the record.
- **References:** electronics 2.7.12 (lift); real_robot_components 5.

#### HAZ-035 Recovering a stranded robot on a slope

- **Mode and place:** transport and recovery; the slope test ground, palm groves. **Cases:** low energy, maintenance.
- **Cause:** An empty or tripped battery leaves the robot on its spring brakes; to move it, the brakes are released by hand (if the real brakes allow it) or the robot is towed, and on a slope it runs away.
- **Hazardous event:** The robot rolls onto the maintainer.
- **Exposed:** the maintainer. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr n/a**. S2 F1 P2 for the record; no control function reduces it.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** a recovery procedure: chock the tracks, tow from uphill, release the brakes on flat ground only; a manual release, if any, of the hold-to-release kind.
- **Met today:** by other measures. **Gap:** The brakes' design is not in the as-built record; no procedure.
- **References:** SF-03.

#### HAZ-036 Hot surfaces

- **Mode and place:** maintenance: inspection and cleaning; any place. **Cases:** maintenance.
- **Cause:** Motors, ESCs and the pack box in the sun at 45 to 50 deg C.
- **Hazardous event:** A burn on touch.
- **Exposed:** the maintainer, the operator. **Harm:** a minor burn.
- **Rating:** S1 F1 P1, **PLr n/a**. S1 F1 P1 for the record: a hand withdraws from a hot surface.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** guards, labels, gloves.
- **Met today:** by other measures.

### 4.6 Power-up, sensors and other

#### HAZ-037 Unexpected motion or brake release at power-up

- **Mode and place:** power-up and self-test; any place. **Cases:** normal use.
- **Cause:** A stale command, an ESC that enables on its own, brakes released at power-up, the automatic re-arm on SELFTEST entry (D30), or an MCU that boots into a wrong state; the robot's controller today is the supplier's board, whose start-up behaviour is not documented.
- **Hazardous event:** The robot moves at power-up into the person next to it.
- **Exposed:** the maintainer, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: crushing at close range. F1: a power-up lasts seconds, a few times a day. P2: the motion is unexpected.
- **Safety goal:** SG-08. **Safety function today:** SF-16 start and restart interlocks; SF-07 watchdog and the reset behaviour. **Other measures:** none.
- **Met today:** partly. **Gap:** S0 shows BOOT, SELFTEST and STANDBY with no motion and the brakes applied until ACTIVE, but in one MCU (PL b at most); the robot's controller today is the supplier's board, whose start-up is not documented.
- **References:** S0; D30; R-36.

#### HAZ-038 Tip-over or rollover on a side slope, a berm or a turn on a slope

- **Mode and place:** RC driving, autonomous: rows, autonomous: turns and pivots, autonomous: transit and return; the slope test ground, palm groves. **Cases:** normal use, a stop on a slope, a sensor fault.
- **Cause:** A side slope or a berm beyond the envelope; a turn on a slope (the D53 limits; T7b cannot fail, R-54); a wrong but plausible IMU (a bias, a misalignment, a slow drift) that removes the tilt stop and the slope limits together without a fault (R-05; since D73 a frozen IMU raises IMU_FAULT, whose class is the owner's open decision 5).
- **Hazardous event:** The robot rolls over onto a person beside or below it.
- **Exposed:** the operator, people nearby. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: the robot rolls over onto a person. F1: few people are beside or below a robot on a side slope or a berm, as HAZ-017. P2: a rollover is sudden.
- **Safety goal:** SG-04. **Safety function today:** SF-10 tilt stop and the IMU-loss handling; SF-11 descent speed cap and trip; SF-13 speed limits by mode and the turn limits on slopes. **Other measures:** the slope limits in the information for use.
- **Met today:** partly. **Gap:** The tilt stop acts in the twin (S2 shows no false trip; T-NAV-13), but one IMU and one MCU (PL b at most); a wrong but plausible IMU is not seen (R-05).
- **References:** D47; D53; D73; R-05; R-54.

#### HAZ-039 The robot applies the descent limits of the wrong implement

- **Mode and place:** any mode; the slope test ground, palm groves. **Cases:** a sensor fault, foreseeable misuse.
- **Cause:** The implement code (the coding plug) reports another valid implement, or the lift sensor fails, so the descent cap takes a row that is too high.
- **Hazardous event:** A brakes-locked stop above the tip speed: the robot tips over.
- **Exposed:** people nearby, the operator. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: the robot tips over onto a person. F1: few people are near a robot on a slope, as HAZ-017. P2: a tip-over is sudden.
- **Safety goal:** SG-04. **Safety function today:** SF-11 descent speed cap and trip. **Other measures:** none.
- **Met today:** no. **Gap:** An unknown implement takes the smallest row, but a wrong valid code is not seen; PL b at most.
- **References:** params/param_electronics.m lim.descent rows 5 and 6; G2-DSC.

#### HAZ-040 A low-energy stop on the way back

- **Mode and place:** autonomous: transit and return, RC driving; the slope test ground, palm groves. **Cases:** low energy.
- **Cause:** The SOC estimate of the as-built BMS is wrong or unknown, the return reserve is too small, or the BMS opens the contactor at low voltage while the robot moves (HV_Opening_Warn makes the MCU stop first only if the BMS warns). On flat ground such a stop is a stop on the brakes, and the robot is left where it stopped.
- **Hazardous event:** An uncontrolled stop on the brakes at speed (a tip-over on a descent), or a robot stranded on a slope or in a lane.
- **Exposed:** people nearby, the operator. **Harm:** crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2: a tip-over on a descent. F1: few people are near a robot on a descent, as HAZ-017. P2: the stop is sudden.
- **Safety goal:** SG-05, SG-04. **Safety function today:** SF-17 battery protection; SF-04 controlled stop with the stop monitor; SF-11 descent speed cap and trip. **Other measures:** the return reserve (soc_rth).
- **Met today:** no. **Gap:** The as-built BMS's frames and warnings are unknown (R-15); the return on low energy has no scenario yet (charter 6; cards G2-INT, T-03).
- **References:** R-15; R-19; charter 6; T-03.

#### HAZ-041 Thrown objects from the mower deck

- **Mode and place:** autonomous: rows, RC driving; greenhouse aisles, palm groves. **Cases:** normal use.
- **Cause:** Stones, sticks, palm debris or irrigation fittings thrown by the blades.
- **Hazardous event:** Eye and body injuries of people nearby.
- **Exposed:** people nearby, the operator. **Harm:** eye injury, impact.
- **Rating:** S2 F1 P2, **PLr n/a**. S2 F1 P2 for the record (a thrown object is sudden); reduced by the deck's guarding.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** the deck housing and its discharge guard (to the mower standard); a keep-out distance in the information for use.
- **Met today:** by other measures. **Gap:** The mower deck is not in the as-built record.

#### HAZ-042 Mower noise

- **Mode and place:** autonomous: rows, RC driving; greenhouse aisles, palm groves. **Cases:** normal use.
- **Cause:** Long shifts near the mowing robot.
- **Hazardous event:** Hearing damage.
- **Exposed:** the operator, people nearby. **Harm:** hearing loss.
- **Rating:** S2 F2 P1, **PLr n/a**. S2: hearing loss is irreversible. F2: the operator and people at work hear the mower through the shift. P1: the harm builds over hours, and hearing protection or distance limits it. For the record; no control function reduces it.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** noise emission data; hearing protection.
- **Met today:** by other measures. **Gap:** No noise data.

#### HAZ-043 Misuse of the links (provisional, card G2-CYB)

- **Mode and place:** any mode; any place. **Cases:** lost RC, lost GNSS, foreseeable misuse.
- **Cause:** The RC link jammed or spoofed by another transmitter; false RTK corrections over LTE; frames injected on CAN.
- **Hazardous event:** Unauthorised or wrong commands, a wrong position, a lost link.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. Provisional: S2; F1 as HAZ-012, under the greenhouse rule (D79); P2. The threat analysis G2-CYB rates it.
- **Safety goal:** SG-07. **Safety function today:** SF-06 lost-link stops; SF-14 geofence, path and navigation integrity. **Other measures:** none.
- **Met today:** unknown. **Gap:** G2-CYB is proposed for a later phase; a jammed link stops the robot (SF-06), a spoofed one is not detected. A spoofed position also bears on SG-03, which G2-CYB rates.
- **References:** G2-CYB; CR-002 A.12.

#### HAZ-044 Foreseeable misuse: riding on the robot, a bridged E-stop, AUTO with nobody at the transmitter

- **Mode and place:** any mode; any place. **Cases:** foreseeable misuse.
- **Cause:** A person, a child among them, climbs or rides on the robot; an E-stop or a bumper is bridged; a guard is removed; a mission runs with nobody near the transmitter.
- **Hazardous event:** A fall or crushing; a robot that cannot be stopped.
- **Exposed:** people nearby, the operator. **Harm:** falls, crushing.
- **Rating:** S2 F1 P2, **PLr n/a**. S2 F1 P2 for the record (a fall is sudden); reduced by the design and the information for use.
- **Safety goal:** SG-12. **Safety function today:** none. **Other measures:** no footholds; labels; training; the RC link required in AUTO (D44), so the transmitter stays in range.
- **Met today:** by other measures. **Gap:** The manuals come with V-06.
- **References:** D44.

#### HAZ-045 The protective field starts at the wrong implement

- **Mode and place:** autonomous: rows, autonomous: turns and pivots, autonomous: transit and return; any place. **Cases:** a sensor fault, foreseeable misuse.
- **Cause:** The protective field comes from the build setting, not the fitted implement (STATUS 25): a companion built for the mower and run with no implement or the tiller starts its front field at 1.20 m instead of 0.55 m; nothing compares the build setting with the implement the MCU reports.
- **Hazardous event:** A person between the body and the start of the field is missed and struck.
- **Exposed:** people nearby, the operator. **Harm:** impact, crushing.
- **Rating:** S2 F1 P2, **PLr d**. S2 as HAZ-001. F1: the greenhouse rule (D79) keeps people out of the robot's path in the greenhouse (assumption 1), and elsewhere few people are in it. P2: the person relies on being seen.
- **Safety goal:** SG-01. **Safety function today:** SF-08 person stop: the protective field, the lost-scan stop, hold and resume. **Other measures:** none.
- **Met today:** no. **Gap:** The field does not follow the fitted implement (STATUS 25); PL b at most.
- **References:** STATUS 25; D72.

## 5. The safety goals

The safety goals are the top-level safety requirements of this phase. Card G1-01 takes them into the requirements baseline; the functional safety concept (G2-FSC) gives each one its safety functions, safe state, reaction time, architecture and verification. Each goal takes the highest level of its hazards and lists the safety functions of its hazards.

**Accepted.** The project owner accepted these goals on 2026-11-16, card G1-07: "Approve and accept (Recommended)" (D80), as the top-level safety requirements at the levels the greenhouse rule (D79) sets.

Table 6. The safety goals.

| ID | Goal | PLr | Hazards | Status today |
|---|---|---|---|---|
| SG-01 | The robot shall detect a person in the zone that it or its implement is about to occupy, in every autonomous mode and in the dock approach, and stop before contact; it shall keep its speed within what that detection zone can stop. | d | HAZ-001, HAZ-002, HAZ-003, HAZ-004, HAZ-007, HAZ-010, HAZ-013, HAZ-045 | not met |
| SG-02 | After a person stop the robot shall move off, or restart its implement, only when the zone it will occupy, the near zone of the body included, has been seen free. | d | HAZ-005 | not met |
| SG-03 | The robot shall stay inside its permitted operating zone and within a bounded distance of its planned path, and stop when it cannot show where it is. | d | HAZ-008, HAZ-009 | not met |
| SG-04 | No brakes-locked stop, E-stop or loss of power shall tip the robot over within its envelope (the 20 deg design slope; the full-tank sprayer to about 18.5 deg, CR-001), and the robot shall not pass its tilt limits. | d | HAZ-017, HAZ-018, HAZ-019, HAZ-023, HAZ-038, HAZ-039, HAZ-040 | not met |
| SG-05 | Every stop shall end with the robot held on its brakes on any slope of the envelope, and the robot shall not gain speed downhill beyond the speed of its mode. | d | HAZ-024, HAZ-025, HAZ-027, HAZ-040 | not met |
| SG-06 | An emergency stop, from the robot or from the operator's remote stop, shall stop the motion and the implement independently of the software, and keep them stopped until a deliberate reset and a new arm command. | d | HAZ-021, HAZ-022 | partly met |
| SG-07 | On the loss of the operator's link, of the companion's commands or of the controller, the robot shall stop and park. | d | HAZ-012, HAZ-016, HAZ-043 | not met |
| SG-08 | The robot shall not move, release its brakes or start its implement at power-up, after a reset, when a link returns or during maintenance, without a deliberate command and a warning; maintenance shall be done with the energy isolated. | d | HAZ-011, HAZ-026, HAZ-032, HAZ-034, HAZ-037 | not met |
| SG-09 | The blades shall not run where a person can reach them: they shall stop on an E-stop, a person near, a hold, a tip, a raised deck and outside the working modes, within a run-down time that the mower standard sets, and shall not restart without a deliberate command. | d | HAZ-006, HAZ-020, HAZ-033 | not met |
| SG-10 | The pack shall not overheat, overcharge or short in operation, charging or storage at 45 to 50 deg C, and live parts shall be protected against contact and short circuit. | d | HAZ-028, HAZ-029, HAZ-030 | unknown |
| SG-11 | In RC driving the robot shall move only as the operator commands, stop when the operator lets go of the sticks, and keep the speed limit of RC driving. | d | HAZ-014, HAZ-015 | not met |
| SG-12 | Hot surfaces, noise, the charger's mains, manual handling and recovery, thrown objects and foreseeable misuse shall be reduced by the design, guards and the information for use, to the standards that G1-01 names. | n/a | HAZ-031, HAZ-035, HAZ-036, HAZ-041, HAZ-042, HAZ-044 | open |

#### SG-01 Person protection in motion

- **Requirement:** The robot shall detect a person in the zone that it or its implement is about to occupy, in every autonomous mode and in the dock approach, and stop before contact; it shall keep its speed within what that detection zone can stop.
- **Safe state:** standstill on the brakes, implement off, holding while the person is in the zone. **PLr:** d. PL d in every place: in the greenhouse with people kept out of the robot's zone (D79), in the palm groves, in transit and at the dock. People at work beside the robot in a greenhouse would need PL e (HAZ-001, -003, -004, -007, -010 and -045 at F2).
- **Hazards:** HAZ-001, HAZ-002, HAZ-003, HAZ-004, HAZ-007, HAZ-010, HAZ-013, HAZ-045. **Safety functions today:** SF-08, SF-12, SF-13, SF-15, SF-18.
- **Status:** not met. The person stop is not safety-rated (PL b at most); the zone in front of the body and the sides are not covered; the field does not follow the fitted implement; the LiDAR is not in the as-built record. Escalated: E-01, E-02 (class 1).

#### SG-02 Safe resume after a person stop

- **Requirement:** After a person stop the robot shall move off, or restart its implement, only when the zone it will occupy, the near zone of the body included, has been seen free.
- **Safe state:** hold on the brakes, implement off. **PLr:** d. PL d, with people kept out of the robot's zone in the greenhouse (D79). People at work beside the robot would need PL e (HAZ-005 at F2).
- **Hazards:** HAZ-005. **Safety functions today:** SF-08, SF-09.
- **Status:** not met. The resume (H1) is judged on a field that does not cover the zone in front of the body (the owner's open decision 2); PL b at most. Escalated: E-01, E-02 (class 1).

#### SG-03 The operating zone

- **Requirement:** The robot shall stay inside its permitted operating zone and within a bounded distance of its planned path, and stop when it cannot show where it is.
- **Safe state:** standstill on the brakes, mission paused. **PLr:** d. PL d in the greenhouse under the greenhouse rule (D79, HAZ-009) and in the palm groves (HAZ-008). People at work beside the robot would need PL e.
- **Hazards:** HAZ-008, HAZ-009. **Safety functions today:** SF-14.
- **Status:** not met. Companion software on GNSS and LiDAR localisation (PL b at most); the cross-track check runs on the estimated position; nothing checks for a false RTK fix (R-43); no GNSS receiver is in the as-built record. Escalated: E-01, E-08 (class 1).

#### SG-04 Stability

- **Requirement:** No brakes-locked stop, E-stop or loss of power shall tip the robot over within its envelope (the 20 deg design slope; the full-tank sprayer to about 18.5 deg, CR-001), and the robot shall not pass its tilt limits.
- **Safe state:** standstill on the brakes below the tip speed; at the tilt limit, a stop. **PLr:** d.
- **Hazards:** HAZ-017, HAZ-018, HAZ-019, HAZ-023, HAZ-038, HAZ-039, HAZ-040. **Safety functions today:** SF-04, SF-10, SF-11, SF-13, SF-17.
- **Status:** not met. Card G2-DSC is in progress; the cap, the trip and the tilt stop rest on one IMU, the encoders and one MCU (PL b at most); a wrong but plausible IMU removes them together (R-05); the tip speeds rest on estimated contact data until T-04. Escalated: E-01, E-03 (class 1).

#### SG-05 Stop and hold, no runaway

- **Requirement:** Every stop shall end with the robot held on its brakes on any slope of the envelope, and the robot shall not gain speed downhill beyond the speed of its mode.
- **Safe state:** standstill on the brakes, torque off. **PLr:** d.
- **Hazards:** HAZ-024, HAZ-025, HAZ-027, HAZ-040. **Safety functions today:** SF-02, SF-03, SF-04, SF-11, SF-12, SF-16, SF-17.
- **Status:** not met. D71 ends every soft stop on the brakes with STOP_FAIL as the STO backup, but nothing monitors the brakes (STATUS 27), the detection is one MCU (PL b at most), the steep descent with CCL 0 and the hill start are open (STATUS 19, 24; R-37), and the brakes are estimates in the as-built record. Escalated: E-01, E-03 (class 1).

#### SG-06 Emergency stop

- **Requirement:** An emergency stop, from the robot or from the operator's remote stop, shall stop the motion and the implement independently of the software, and keep them stopped until a deliberate reset and a new arm command.
- **Safe state:** STO, spring brakes engaged, implement power off, latched. **PLr:** d. PL d, with people kept out of the robot's zone in the greenhouse (D79). People at work beside the robot would need PL e (HAZ-021, HAZ-022 at F2).
- **Hazards:** HAZ-021, HAZ-022. **Safety functions today:** SF-01, SF-02, SF-03, SF-05, SF-06.
- **Status:** partly met. The on-board chain is designed as two channels and shown in S8a; the owner confirms E-stop buttons and a safety relay with STO wiring on the robot (D80), but their parts, category and wiring are not recorded yet (card G1-04); the relay re-arms without a manual reset after a controller restart or on RC CH9 (F-01); the remote stop is a functional stop through SBUS (PL b at most); no certified wireless E-stop. Escalated: E-01, E-04 (class 1).

#### SG-07 Stop on loss of control

- **Requirement:** On the loss of the operator's link, of the companion's commands or of the controller, the robot shall stop and park.
- **Safe state:** standstill on the brakes, torque off. **PLr:** d. PL d, with people kept out of the robot's zone in the greenhouse (D79). People at work beside the robot would need PL e (HAZ-012 at F2).
- **Hazards:** HAZ-012, HAZ-016, HAZ-043. **Safety functions today:** SF-04, SF-06, SF-14.
- **Status:** not met. The stops on a lost link act in simulation (S6, T-EL-32), but in one MCU and the ESC software (PL b at most); a hung companion that keeps sending is not detected (R-39); the radio model of the row tests cannot produce a link loss (R-12). Escalated: E-01, E-05 (class 1).

#### SG-08 No unexpected start-up

- **Requirement:** The robot shall not move, release its brakes or start its implement at power-up, after a reset, when a link returns or during maintenance, without a deliberate command and a warning; maintenance shall be done with the energy isolated.
- **Safe state:** standstill on the brakes, torque off, implement off. **PLr:** d. During maintenance the goal is met by isolating the energy (the pack disconnected), not by a control function.
- **Hazards:** HAZ-011, HAZ-026, HAZ-032, HAZ-034, HAZ-037. **Safety functions today:** SF-01, SF-07, SF-08, SF-15, SF-16.
- **Status:** not met. A new arm edge is needed after any stop, but the interlocks are one MCU (PL b at most); the relay re-arms automatically on SELFTEST entry and on RC CH9 (D30, F-01), so a controller restart re-arms it without the manual reset; an arm edge can come from the receiver (R-35); no isolation procedure or lockable isolator is recorded. Escalated: E-01, E-05 (class 1).

#### SG-09 Blade safety

- **Requirement:** The blades shall not run where a person can reach them: they shall stop on an E-stop, a person near, a hold, a tip, a raised deck and outside the working modes, within a run-down time that the mower standard sets, and shall not restart without a deliberate command.
- **Safe state:** blades stopped, implement power off. **PLr:** d. PL d, with people kept out of the robot's zone in the greenhouse (D79). The deck housing is the primary measure (a fixed guard); this level is for the blade stop. People at work beside the robot would need PL e (HAZ-006 at F2).
- **Hazards:** HAZ-006, HAZ-020, HAZ-033. **Safety functions today:** SF-01, SF-09.
- **Status:** not met. The person path of the blade stop is software (PL b at most) and does not watch the sides of the deck; the run-down time is unknown and no blade brake is specified (R-33); the mower implement is not in the as-built record. Escalated: E-01, E-06 (class 1).

#### SG-10 Battery, charging and live parts

- **Requirement:** The pack shall not overheat, overcharge or short in operation, charging or storage at 45 to 50 deg C, and live parts shall be protected against contact and short circuit.
- **Safe state:** contactors open, charging stopped. **PLr:** d. PL d through HAZ-029: a fire reaches the people at work in the other aisles of the greenhouse; the greenhouse rule (D79) does not lower it.
- **Hazards:** HAZ-028, HAZ-029, HAZ-030. **Safety functions today:** SF-17.
- **Status:** unknown. The bought BMS and charger protect the pack, but their settings, rating and frames are unknown (R-15); the fuse and BMS coordination is open (T-PWR-06b); the sealed pack runs hot (R-04); no K4 weld check (R-45). Escalated: E-01 (class 1).

#### SG-11 Manual control integrity

- **Requirement:** In RC driving the robot shall move only as the operator commands, stop when the operator lets go of the sticks, and keep the speed limit of RC driving.
- **Safe state:** standstill on the brakes. **PLr:** d.
- **Hazards:** HAZ-014, HAZ-015. **Safety functions today:** SF-05, SF-06, SF-16, SF-19.
- **Status:** not met. The RC chain (radio, SBUS, one MCU) is not safety-rated (PL b at most); corrupted bytes can pass as valid frames (STATUS 8, R-47). Escalated: E-01, E-05 (class 1).

#### SG-12 Hazards reduced without a control function

- **Requirement:** Hot surfaces, noise, the charger's mains, manual handling and recovery, thrown objects and foreseeable misuse shall be reduced by the design, guards and the information for use, to the standards that G1-01 names.
- **Safe state:** not applicable. **PLr:** n/a. No safety function is claimed; the measures are design, guards, labels, procedures and training.
- **Hazards:** HAZ-031, HAZ-035, HAZ-036, HAZ-041, HAZ-042, HAZ-044. **Safety functions today:** none (non-functional measures).
- **Status:** open. The measures are to be shown by the design records and the manuals (V-06); the mower deck, the charger's conformity and the maintenance procedures are not in the records yet.

## 6. The safety functions today

The safety functions of the twin, as the specs and the decisions describe them, with what the as-built record shows of them and a preliminary judgement of the level their architecture can reach. The concept (G2-FSC) and the FMEA (G3-FME) settle the levels; this log does not design them.

Table 7. The safety functions, where they run and what their architecture can reach.

| ID | Function | Where | In the as-built record | Architecture (preliminary) |
|---|---|---|---|---|
| SF-01 | Hardware E-stop chain | hardware (POWER's Safety_Chain, D8) | on the robot, by the owner's statement (D80): E-stop buttons and a safety relay with STO wiring; their positions, parts, category and wiring are not in the as-built record yet (card G1-04). Bumpers and a key switch are not confirmed | two channels with a discrepancy check (bit 27), independent of the software for the stop: Category 3 is possible, so PL d with rated parts. The re-arm is not a manual reset: a controller restart or RC CH9 re-arms the relay (F-01) |
| SF-02 | Safe torque off (STO) of the ESCs | ESC (drive esc_sm state 8) and the safety relay | STO wiring confirmed by the owner (D80); the dual-channel ESC per side is known, but its STO input and its rating are not documented (R-14; the ESC manual, card G1-04) | depends on the ESC's STO and its rating (unknown) |
| SF-03 | Spring brakes and the park at the end of every stop | drive (brake, ESC) and MCU (mode manager) | an estimate: 5 N m per motor [EST], the brake mass 0.5 kg [EST]; no brake data in the as-built record; the slope brake test T-04 is planned | spring-applied brakes are well-tried mechanical parts if specified so; there is no brake monitoring: the confirmation (FB1 brake_released) echoes the brake command (STATUS 27) |
| SF-04 | Controlled stop with the stop monitor | MCU (mode manager, safety supervisor) | not yet: the firmware generated from the twin is not on the robot | one microcontroller, one channel: Category B, PL b at most. STOP_FAIL gives a second shut-off path (STO through the relay), but its monitor runs on the same MCU |
| SF-05 | Remote stop from the RC transmitter | RC transmitter and receiver (SBUS), MCU | the robot's RC transmitter and receiver are not in the as-built record | a functional stop: SBUS is not safety-rated (electronics spec D7); Category B, PL b at most. The spec recommends a certified wireless E-stop (PL d) in series with the loop of SF-01 |
| SF-06 | Lost-link stops | MCU, ESC | not yet (generated firmware not on the robot) | MCU and ESC software, one channel: Category B, PL b at most. A hung companion that keeps sending fresh frames is not detected (alive counters unchecked, R-39) |
| SF-07 | Watchdog and the reset behaviour | MCU (IWDG), electronics plant (watchdog-reset emulation) | not yet | an independent hardware timer with an independent shut-off path: a diagnostic for a stalled MCU; it does not detect a wrong output. The latches are cleared by a reset (R-36) |
| SF-08 | Person stop: the protective field, the lost-scan stop, hold and resume | companion (LiDAR, perception, GNC), CAN2, MCU | not in the as-built record: the LiDAR and the sensor mast are a model addition (not in the photos), and no companion computer is listed | a LiDAR and a companion computer that are not safety-rated, through CAN2 and one MCU: Category B, PL b at most. Gaps in the twin: the zone in front of the body is not covered (R-02, the owner's open decision 2; STATUS 21); the field follows the build setting, not the fitted implement (STATUS 25); static walls cause holds at lane ends (STATUS 18); on the robot a re-published stale cloud stays valid (STATUS 28) |
| SF-09 | Blade stop and the implement latch | GNC (implement manager), MCU (implement output), safety relay (K3) | the mower implement is not in the as-built record (blades, guard, motor, brake) | the person path is companion and MCU software: Category B, PL b at most; the E-stop path (K3) follows SF-01. The blade run-down time is not specified or checked, and no blade brake is specified (R-33) |
| SF-10 | Tilt stop and the IMU-loss handling | MCU (fw_imu_tilt, supervisor), sensors (sens_acq_imu), GNC | no IMU is listed in the as-built record (the model uses an ICM-20948) | one IMU and one MCU: Category B, PL b at most. Since D73 a frozen IMU raises IMU_FAULT; a wrong but plausible IMU (a bias, a misalignment, a slow drift) removes the tilt stop and the slope limits together without a fault (R-05) |
| SF-11 | Descent speed cap and trip | MCU (Descent_cap, Descent_trip), GNC (governor, look-ahead, deck-down rule) | not yet; the tip speeds rest on estimated contact data until the slope brake test T-04 | one IMU, the encoders and one MCU: Category B, PL b at most; the raised rows rest on one lift sensor |
| SF-12 | Overspeed and speed-follow trip | MCU, ESC | not yet | one MCU on the encoders: Category B, PL b at most |
| SF-13 | Speed limits by mode and the turn limits on slopes | MCU (shaper, limits), GNC (command shaping, mission compiler) | not yet | MCU and companion software: Category B, PL b at most |
| SF-14 | Geofence, path and navigation integrity | companion (GNC, navigation) | not in the as-built record: no GNSS receiver or companion computer is listed | companion software on GNSS and LiDAR localisation: Category B, PL b at most; no false-fix model or check (R-43) |
| SF-15 | Start warning and indication | MCU (auxiliary outputs), beacon and buzzer | the beacon and the buzzer are not in the as-built record | a warning (information for use), not a protective function |
| SF-16 | Start and restart interlocks | MCU (mode manager), safety relay | not yet | MCU software: Category B, PL b at most. An arm edge can come from the receiver (R-35); the relay re-arms on RC CH9 and automatically on SELFTEST entry (D30, F-01), and a reset clears the latches (R-36) |
| SF-17 | Battery protection | BMS, power distribution, charger | a bought 13S BMS (120 to 150 A, CAN) and a 54.6 V 20 A charger (IP54) are in the as-built record; their protections, settings and CAN frames are unknown (R-15); the fuse and BMS coordination is open (T-PWR-06b fails) | the bought BMS's own design and rating (unknown); no K2 or K4 weld detection (R-45) |
| SF-18 | Encoder plausibility | MCU | not yet; whether the encoders are read by the MCU or come over CAN is open | a plausibility check in one MCU: a diagnostic for the speed measurement, no independent shut-off |
| SF-19 | Manual control: the stick-release stop | RC transmitter, SBUS, MCU | not yet: the generated firmware is not on the robot, and the robot's RC equipment is not in the as-built record | a functional control path, not safety-rated: Category B, PL b at most |

#### SF-01 Hardware E-stop chain

- **Does:** E-stop buttons (front, rear), bumpers and an optional certified wireless E-stop in a dual-channel loop to a safety relay. The immediate contacts open the STO of both ESCs, the brake-release power (the spring brakes engage) and the implement 24 V relay K3 in at most 20 ms; the delayed contacts open the main contactor coils after 1.5 s. The stop is independent of the firmware: the MCU can only open the loop (DO0) and pulse a re-arm (DO1). The pulse comes on the reset button, on RC CH9 over the SBUS radio, and once on each SELFTEST entry (D30); the relay closes only if every contact of the loop is closed (finding F-01).
- **Evidence in the twin:** S8a (STO 20 ms after the edge, stop within 0.6 m from 1.2 m/s, no restart before release, reset and re-arm); fault bits 0, 27, 29; T-EL-28/29.

#### SF-02 Safe torque off (STO) of the ESCs

- **Does:** Removes the motor torque at once on the safety relay (SF-01) or when the MCU's stop monitor sets STOP_FAIL (bit 22, DO0 low); the ESC enters its STO state with the brake engaged.
- **Evidence in the twin:** drive spec 2.9; S8a; T19b to T19d.

#### SF-03 Spring brakes and the park at the end of every stop

- **Does:** Spring-applied, electrically released brakes on the motor shafts engage on any loss of power. Every soft stop ends on the brakes: CTRL_STOP and ESTOP.DECEL decelerate, then park with the torque limits held until both ESCs confirm the brake (D71). The ESC applies the brake at once on a park request, and 2.0 s after the start of a DISABLING or QUICK_STOP stop at any speed.
- **Evidence in the twin:** D64 (elasto-plastic brake), D71, S2 hold (roll-back within 0.05 m), T8, T9.

#### SF-04 Controlled stop with the stop monitor

- **Does:** On a class-B fault, an RC link loss or failsafe, a disarm, a companion time-out or stop request, the MCU decelerates at 1.0 m/s^2 (constant-deceleration law), holds zero speed for 0.5 s or to its time-out, then parks on the brakes (SF-03). The stop monitor (STOP_FAIL, bit 22, class A) opens DO0 when a stop does not slow down, the brakes do not confirm within 0.5 s, or the robot moves in STANDBY, ESTOP.SAFE or FAULT: STO and K3 open after 15 ms.
- **Evidence in the twin:** D33 (0.71 m from 1.0 m/s with 128 ms detection), D71, S6a/S6b, T-EL-54 to 60.

#### SF-05 Remote stop from the RC transmitter

- **Does:** RC mode-switch position 3 (CH12) on valid frames, 40 ms persistence: the MCU stops at 2.0 m/s^2 for at most 1.2 s, then parks, and latches ESTOP; reset needs CH12 off, a reset request and standstill. A receiver failsafe never latches ESTOP (D34).
- **Evidence in the twin:** S8b (2.0 m/s^2 within 0.1), D33 (0.62 m from 1.2 m/s with 92 ms worst latency), D34.

#### SF-06 Lost-link stops

- **Does:** RC link lost (rc_age above 0.10 s) or receiver failsafe: CTRL_STOP within 0.13 s, in RC driving and in AUTO (rc_required_in_auto true, D44). Companion command age above 0.20 s in AUTO: CTRL_STOP. ESC: 10 missed command frames (100 ms): QUICK_STOP at 2.0 m/s^2, then the brake. MCU: an ESC feedback (FB1 or FB3) older than 50 ms is a class-A fault (REQ-SYS-070).
- **Evidence in the twin:** S6a, S6b (CTRL_STOP within 0.13 s, standstill within 1.5 s, no motion after the link returns until arm off then on); T-EL-32, T-EL-35.

#### SF-07 Watchdog and the reset behaviour

- **Does:** The IWDG (50 ms) resets a stalled MCU; during the reset all digital outputs are low: DO0 opens the safety loop, K1 opens, the brakes engage, the ESCs time out into QUICK_STOP. After the reset the MCU boots through SELFTEST to STANDBY; the relay is re-armed automatically once on SELFTEST entry (D30).
- **Evidence in the twin:** electronics 2.7.11, 2.9.2; S0 (automatic re-arm at power-up).

#### SF-08 Person stop: the protective field, the lost-scan stop, hold and resume

- **Does:** The perception sets protectiveStop when a kept LiDAR point lies in the front or the rear field, or after more than 2 consecutive invalid scans; the front field starts at the front of the fitted implement (mower 1.20 m, tiller, sprayer and none 0.55 m), runs 0.5 + 0.8 |v| m, 1.08 m wide; both fields are on below 0.1 m/s, otherwise the field in the direction of travel. The GNC enters OBSTACLE_HOLD (PERSON or SENSOR_LOST) and commands standstill; H1 resumes after 2 s clear on valid scans. The field is true from the start until the first valid scan (D72).
- **Evidence in the twin:** D72; T-PER-02, tPER02b, T-PER-07; S5 (clearance at least 1.0 m to a crossing person).

#### SF-09 Blade stop and the implement latch

- **Does:** The rotor runs only in MANUAL or AUTO with the implement switch on and no implement fault; the GNC stops the blades at once on person_near, after more than 3 s in OBSTACLE_HOLD, in TRANSIT, PAUSE, FAULT and MANUAL; once a person or a lost scan is seen in a hold, the implement stays off for the rest of that hold (D72). The E-stop opens K3, the implement 24 V relay.
- **Evidence in the twin:** D72 (latch), gnc 2.9, electronics 2.7.12, power 2.5.

#### SF-10 Tilt stop and the IMU-loss handling

- **Does:** TILT_WARN above 25 deg for 0.5 s derates (0.5); TILT_STOP above 28 deg for 0.5 s is a class-B stop; both use the level-calibrated estimate and are gated on a valid IMU (D47, D73). The GNC faults on pitch above 25 deg or roll above 18 deg in AUTO. A frozen IMU is invalid after 0.1 s; IMU_FAULT is class C (derate 0.25, yaw loop off): its class is the owner's open decision 5.
- **Evidence in the twin:** D47, D73; S2 (no tilt fault in start or stop on the ramp); T-NAV-13.

#### SF-11 Descent speed cap and trip

- **Does:** In every mode the MCU limits the speed target to the highest speed at which a brakes-locked stop may start without tipping, by the measured downhill pitch (peak-held 1 s) and the fitted implement (P.structure.descent, 12 breakpoints to 25 deg); the trip sets OVERSPEED and parks at once. An unknown implement takes the smallest entries; the mower with its deck reported raised takes the raised rows (0.15 m/s on the flat, 0 from 5 deg). The GNC governor and look-ahead slow the robot before a descent; the GNC keeps the deck down while moving. CR-001 limits the full-tank sprayer to about 18.5 deg.
- **Evidence in the twin:** card G2-DSC (in progress; D74 and NASL-SHD-ER-005 pending); S2c, S2d; T-EL-54 to 60.

#### SF-12 Overspeed and speed-follow trip

- **Does:** OVERSPEED (bit 16, class B) when a track exceeds 1.70 m/s for 0.2 s, or runs more than 0.2 m/s above its reference for 0.2 s in the driving modes (REQ-SYS-109); CTRL_STOP and ESTOP.DECEL then park at once (D71). The ESC has its own overspeed trip at 4000 rpm (about 1.885 m/s on the track).
- **Evidence in the twin:** D71; STATUS 19 and 24 (open: with CCL 0 the speed-follow term parks at speed on steep descents and trips on ordinary stops).

#### SF-13 Speed limits by mode and the turn limits on slopes

- **Does:** Top speed 1.41 m/s; RC driving 1.4 m/s, AUTO 1.2 m/s (the GNC limits derived from the MCU, D43), the dock approach 0.2 m/s; the yaw rate limited by the tilt (1.2 rad/s below 8 deg to 0.2 rad/s at 20 deg), no pivots above 10 deg, no headland turns where the slope exceeds 10 deg (D53).
- **Evidence in the twin:** D43, D53; S1; T7b (which cannot fail, R-54).

#### SF-14 Geofence, path and navigation integrity

- **Does:** The GNC faults (FAULT, standstill) on a geofence breach above 0.5 m, navigation lost for 0.3 s, a cross-track error above 0.5 m for 3 s, a robot stuck for 5 s, or the tilt limits; navigation degraded for 2 s pauses the mission.
- **Evidence in the twin:** gnc 4.1 (G1, T7); S7 (GNSS dropout).

#### SF-15 Start warning and indication

- **Does:** AUTO_START sounds the buzzer for 3 s with the beacon blinking green before the robot moves; in AUTO the buzzer beeps every 2 s; the beacon shows the mode (amber MANUAL, red ESTOP and FAULT).
- **Evidence in the twin:** electronics 2.7.3, 2.7.12.

#### SF-16 Start and restart interlocks

- **Does:** Arming from STANDBY needs the RC link, an arm edge, the sticks neutral and the power ready; resuming after any stop needs a new arm edge; ESTOP and FAULT reset need the E-stop released, a reset request and standstill; the relay re-arms only if every hardware contact is closed (D30).
- **Evidence in the twin:** electronics 2.7.3; S6 (no motion after the link returns until arm off then on); S8a.

#### SF-17 Battery protection

- **Does:** The BMS opens the main contactor on cell over- and under-voltage, over-temperature, over-current, a precharge failure or a weld; HV_Opening_Warn makes the MCU stop first; the charge current follows the cell temperature (no charge from 50 deg C); traction fuses (125 A [EST]); the dock contactor K4 closes only with dock contact.
- **Evidence in the twin:** power 2.5, 3.4; T-PWR tests; D45, D65.

#### SF-18 Encoder plausibility

- **Does:** ENC_L/R_MISMATCH (bits 12 and 13, class B, 0.2 s): the MCU-read motor encoders against the drive-reported speed.
- **Evidence in the twin:** electronics 2.7.7, 2.7.10.

#### SF-19 Manual control: the stick-release stop

- **Does:** In RC driving the sticks spring back to centre, which commands zero; the MCU shaper decelerates and the robot stops in about 0.88 m from 1.2 m/s, including 60 ms latency (D33); the speed limit of RC driving is 1.4 m/s.
- **Evidence in the twin:** D33; S1.

## 7. Escalations, findings and open points

**Escalated to the project owner, through the lead systems engineer.** Each class-1 item is a safety goal the as-built hardware cannot meet; closing it needs new hardware or supplier data, which is the owner's decision. E-07 and E-09 are plain decisions, not class-1 changes. The functional safety engineer does not design the fixes.

**The owner's decisions of 2026-11-16 (D79, D80).** E-09 is decided and the question of E-01 is answered. The class-1 items are worked on card G2-FSC, whose exit asks for each one the architecture that meets its goal at the level D79 and D80 set, or a change request for new hardware; the data of the safety devices and the supplier's data come on card G1-04. Nothing is bought now (D80).

**The owner's decision of 2026-11-18 (D103).** E-07 is decided: the Kingdom of Saudi Arabia, with every standard dated in the safety plan's Table 1. No hazard is rated again.

Table 8. The escalations.

| ID | Class | Goals | State | Worked on | What and why |
|---|---|---|---|---|---|
| E-01 | 1 | SG-01, SG-02, SG-03, SG-04, SG-05, SG-06, SG-07, SG-08, SG-09, SG-10, SG-11 | open, its question answered (D80) | G1-04, G2-FSC | No safety goal can be shown met on the robot as built. The question of v1.1, which safety devices the robot actually has, is answered by the owner (2026-11-16, D80): "E-stop button(s), Safety relay / STO wiring". Their positions, parts, category and wiring are not in the as-built record yet; the other devices the question named (bumpers, a key switch, spring brakes on the motors) are not named in the answer and stay unconfirmed. The holding brakes are estimates; no LiDAR, GNSS, IMU or companion computer is in the record; the mower implement and the dock are not recorded; the robot's controller is the supplier's board. Needed: the data of the confirmed devices and the supplier's data (the ESC manual with its STO and brake feedback, the brakes, the controller, the BMS: review decision 7, R-14, R-15), on card G1-04; then the concept (G2-FSC) answers each goal at its level. Nothing is bought now (D80). |
| E-02 | 1 | SG-01, SG-02 | open | G2-FSC | Person protection needs PL d: in the greenhouse with people kept out of the robot's zone (D79), in the palm groves, in transit and at the dock; people at work beside the robot would need PL e. The designed person stop (a LiDAR and a companion that are not safety-rated, through one MCU) reaches PL b at most, and does not cover the zone in front of the body, the sides, or a field set for another implement. A safety-rated detection wired into the stop chain is new hardware. The owner's open decision 2 (the near zone) belongs to these goals. |
| E-03 | 1 | SG-04, SG-05 | open | G2-FSC, G1-04 | Stability and the hold on slopes need PL d. The descent cap, its trip and the tilt stop rest on one IMU, the encoders and one MCU (PL b at most), and nothing monitors the brakes. Since D73 a frozen IMU raises IMU_FAULT, but a wrong but plausible IMU is not seen. PL d needs a second, independent measurement and shut-off path, brake monitoring, or a stop that cannot tip (card G2-BRK, which may need a braking resistor): hardware. The brake feedback comes with the ESC manual (card G1-04, STATUS 27). The owner's open decision 5 (the IMU fault class) belongs to these goals. |
| E-04 | 1 | SG-06 | open | G1-04, G2-FSC | The emergency stop needs PL d (D79). The owner confirms E-stop buttons and a safety relay with STO wiring on the robot (D80); their parts, category and wiring come from the supplier on card G1-04. In the twin the chain is two channels, and its relay re-arms without a manual reset (F-01); the operator's remote stop is a functional stop through the SBUS radio (PL b at most). A certified wireless E-stop in series with the loop, as the electronics spec recommends, is new hardware; whether the chain reaches PL d depends on its parts and its diagnostics (G2-FSC, G3-FME). |
| E-05 | 1 | SG-07, SG-08, SG-11 | open | G2-FSC | The stops on a lost link, the start interlocks and the manual control need PL d and run in one MCU (PL b at most). The concept (G2-FSC) must give them a tested or redundant architecture; that may need a safety controller or a second channel the robot does not have. |
| E-06 | 1 | SG-09 | open | G2-FSC | Blade safety needs PL d (D79), and a run-down time that the mower standard sets. The mower implement is not in the as-built record, its run-down time is unknown, no blade brake is specified, and the blade stop for a person is software that does not watch the sides of the deck: implement data, and maybe a blade brake, are hardware questions. |
| E-07 | decision (not class 1) | none | decided (D103) | none | The editions of the standards and the jurisdiction, decided by the project owner on 2026-11-18: "Kingdom, dated editions (Recommended)"; on the standards' texts: "Buy none now" (D103). The jurisdiction is the Kingdom of Saudi Arabia, and every standard is cited with its edition, the binding SASO and CST layer in its own column (NASL-SHD-SAF-001 v1.4, section 3 and Table 1). ISO 13849-1:2023 stays the rating method, so no hazard is rated again. The machine's classification and its mower standard are decided on card G2-FSC with E-06, because ISO 5395 does not cover a robotic or battery mower. The texts are not bought: the safety values the standards would set (REQ-SYS-033, -046, -048 and -098) are set by risk assessment and marked as not yet confirmed against the standards. A plain decision, not a class-1 change. |
| E-08 | 1 | SG-03 | open | G2-FSC | Localisation integrity: the operating zone and the path bound need PL d: in the greenhouse under the greenhouse rule (D79, HAZ-009) and in the groves (HAZ-008). They rest on GNSS and LiDAR localisation in the companion (PL b at most); nothing checks for a false RTK fix (R-43), and the cross-track check runs on the estimated position, so it cannot see a localisation error; no GNSS receiver is in the as-built record. PL d needs a tested or redundant position path (an independent second source, or an integrity monitor): hardware. |
| E-09 | decision (not class 1) | SG-01, SG-02, SG-03, SG-06, SG-07, SG-09 | decided (D79) | none | The greenhouse operating concept, decided by the project owner on 2026-11-16: "Keep people out (Recommended)" (D79). People are kept out of the robot's zone while it mows a greenhouse aisle, and work in the other aisles (the charter v1.2, section 6; the zone as section 2, assumption 1 defines it). The six goals SG-01, SG-02, SG-03, SG-06, SG-07 and SG-09 are at PL d instead of PL e; SG-10 stays at PL d, because a fire reaches the whole house. People at work beside the robot would need person protection at PL e, with safety-rated detection (D79). A plain decision, not a class-1 change. |

**Findings for the concept (G2-FSC) and the subsystem engineers** (severity: Critical, Major and Minor, as the safety plan uses for the review of every safety record.)

- **F-01 (Major):** The safety relay is re-armed by the MCU's reset pulse (DO1): on the reset button, on RC CH9 over the SBUS radio, which is not safety-rated, from wherever the transmitter is, and once automatically on each SELFTEST entry (D30). After a controller restart (power-up or watchdog) the relay re-arms with no manual action. The bound: the relay closes only with every loop contact closed, and motion still needs an arm edge (SF-16, PL b), so after a restart the stop of a maintained E-stop rests on the MCU's interlock instead of the hardware latch. This bypasses the manual reset the design selects (electronics spec, power spec), and the manual reset of the safety function that ISO 13849-1 and IEC 60204-1 ask for, beside ISO 13850, whose reset is the release at the E-stop device, which must not restart the machine; G1-01 confirms the clauses. SF-01 is corrected to match. Goals SG-06 and SG-08 (HAZ-021, HAZ-026, HAZ-037); for the concept G2-FSC and the MCU engineer.
- **F-02 (Major):** The brake confirmation echoes the brake command (STATUS 27): a stuck or worn brake reaches STANDBY. Goals SG-05 and SG-06 (HAZ-021, HAZ-024).
- **F-03 (Major):** Since D73 a frozen IMU raises IMU_FAULT, but a wrong but plausible IMU (a bias, a misalignment, a slow drift) removes the tilt stop, the descent cap and the heading together without a fault (R-05); IMU_FAULT stays class C pending the owner's decision 5. Goal SG-04.
- **F-04 (Major):** The protective field does not cover the zone in front of the body (0.05 to 0.45 m) or the sides, and follows the build setting, not the fitted implement (STATUS 21, 25; the owner's decision 2). Goals SG-01 and SG-02 (HAZ-001 to HAZ-005, HAZ-045).
- **F-05 (Minor):** Traction stops as category 0 on an E-stop while the spec claims category 1 (R-33, O4, the owner's decision 9); nothing checks that the blades run down. Goals SG-04, SG-06, SG-09.
- **F-06 (Minor):** The return on low energy and the sensor faults other than lost GNSS have no system scenario (the charter, section 6). Goals SG-04 and SG-05 (HAZ-040).

**Open points of this issue:**

- The Safety Analysis Manager copy docs/safety/hazard_log.mldatx is not made: Simulink Fault Analyzer is installed but not licensed to the project's MATLAB (its own licence check, in a batch run on 2026-11-16). Open point OP-01: the project owner decides on the licence by 2026-11-23; the functional safety engineer then makes the copy from this record in one batch run, before G1-05. Until then this record and its PDF are the hazard log.
- **The operator's and the maintainer's view** (the risk on card G1-07): the owner, today the only operator and maintainer of the robot, is asked to check the walk-through (Table 4) for a missing case, and to confirm assumptions 2 and 5 of section 2 (the palm-grove rows, the charging place), by G1-05.
- Each hazard is rated again when the concept, the supplier data or a change request changes its cause or its measures.

## 8. Later-phase entries: tillage and spraying

Rated provisionally, from the estimated tiller data and CR-001. Cards G2-TIL and G2-SPR rate them again before those phases start.

Table 9. The later-phase entries.

| ID | Hazard | S F P | PLr | Goal | Measure today |
|---|---|---|---|---|---|
| HAZ-101 | The raised tiller tips the robot backward | S2 F1 P2 | d | SG-04 | the tiller is not used until real tiller data exist (charter 3) |
| HAZ-102 | Contact with the tiller rotor | S2 F1 P2 | d | SG-09 | the rotor guard; the implement latch (D72) |
| HAZ-103 | Reversing with the tiller | S2 F1 P2 | d | SG-04, SG-01 | no reversing with the tiller (the cap's reverse row is 0) |
| HAZ-104 | Stones and soil thrown by the tiller | S2 F1 P1 | n/a | SG-12 | the tiller guard |
| HAZ-105 | A brakes-locked stop with the tiller on a descent | S2 F1 P2 | d | SG-04 | the tiller is not used until real data exist |
| HAZ-106 | The full-tank sprayer on a descent beyond about 18.5 deg | S2 F1 P2 | d | SG-04 | the CR-001 operating limit (about 18.5 deg), in the requirements and the operating concept |
| HAZ-107 | Spray reaches people | S2 F2 P1 | d | SG-01 | nozzles on only above 0.2 m/s; spray off in a hold; re-entry rules (information for use) |
| HAZ-108 | Chemical exposure in filling, cleaning, a hose burst or a spill | S2 F1 P1 | n/a | SG-12 | PPE; filling and cleaning procedures |

## References

- NASL-SHD-SAF-001, the safety plan; NASL-SHD-CHR-001 v1.2, the project charter; NASL-SHD-CR-001 v1.2 and NASL-SHD-CR-002 v1.1, the change requests; NASL-SHD-PMP-001, the project plan.
- ISO 12100:2010, risk assessment and risk reduction; ISO 13849-1:2023 and ISO 13849-2:2012, safety-related parts of control systems; ISO 18497-1 to -4:2024, safety of partially automated, semi-autonomous and autonomous agricultural machinery; ISO 25119-1:2018 with Amd 1:2020, -2:2019, -3:2018 with Amd 1:2020 and -4:2018 with Amd 1:2020, safety-related parts of control systems of agricultural machinery; ISO 13850:2015, the emergency stop function; IEC 60204-1:2016 with AMD1:2021, electrical equipment of machines. The jurisdiction is the Kingdom of Saudi Arabia (D103). The full dated list, with the binding Saudi layer (the SASO machinery regulation TR 01-08-21-180 and the standards it adopts, the CST radio specifications and the Labor Law), is in NASL-SHD-SAF-001 v1.4, Table 1; the mower standard is chosen on card G2-FSC.
- docs/project/charter.md (NASL-SHD-CHR-001 v1.2)
- docs/hardware/real_robot_components.md (the as-built record)
- docs/specs/*.md (the safety functions: electronics 2.5, 2.7.3, 2.7.10 to 2.7.12, 2.9.1; power 2.5, 3.4; drive 2.7, 2.9; sensors 3.7; gnc 2.6, 2.9, 4.1)
- docs/architecture.md (sections 10 and 12; decisions D30 to D80, above all D71, D72, D73, D79 and D80; D103 for the standards)
- docs/changes/CR-001_sprayer_descent_envelope.md; docs/changes/CR-002_plan_after_g0.md
- docs/STATUS.md section 6 (open problems 3, 8, 9, 16a, 16c, 18 to 28)
- the review report NASL-SHD-ANR-001 of 2026-11-15 (items R-01 to R-87, and the owner's open decisions 1 to 20)
- params/param_structure.m and params/param_electronics.m (the descent cap table, card G2-DSC)
