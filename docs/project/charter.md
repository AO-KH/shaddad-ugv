<!--
doc_no: NASL-SHD-CHR-001
doc_type: PROJECT CHARTER
title: Shaddad UGV: project charter
subtitle: The need, the goals, the constraints and the scope of the Shaddad project, as the robot is built
summary: Why the Shaddad project exists, what it must achieve and how success is measured, the constraints of the as-built robot, what is in and out of scope in this phase, who uses the robot and how. The charter is what gate G0 locks; it supersedes the scope section of the architecture document, which card G1-01 re-baselined on 2026-11-18 (D110).
version: 1.5
date: 2026-11-18
status: Approved
history: 1.0 | 2026-11-16 | First issue, for the G0 delta review (RID-G0-01, RID-G0-03, RID-G0-04)
history: 1.1 | 2026-11-16 | Corrections from the delta re-review (RID-G0-12): the energy margin, the scenario coverage, the tiller record
history: 1.2 | 2026-11-16 | The owner's decisions on the hazard log (card G1-07): the greenhouse rule (D79); the safety goals as the top-level safety requirements (D80)
history: 1.3 | 2026-11-16 | CR-003 (the owner's decision of 2026-11-16, "Restate by the safe-stop rule (Recommended)"): goal 3 measured by the park criterion (D82)
history: 1.4 | 2026-11-18 | CR-004 (the change board's decision of 2026-11-18, "Option 1: both (Recommended)"): the planned 1 kW motors in the constraints and goal 1 (D95); goal 4 and S3 per the mission energy of D101
history: 1.5 | 2026-11-18 | Editorial: card G1-01 has re-baselined the requirements (D110, the owner's approval of 2026-11-18); the note on the architecture's scope section updated
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV Project Charter
file: NASL_CHR-001_Shaddad_Project_Charter_v1.5
outdir: work/reports/project
chapter_breaks: no
cover_image: shaddad
request: Gate G0 delta review (the owner's decision of 2026-11-16) and the owner's choice of the main uses ("Mowing now, the rest later")
related: NASL-SHD-PMP-001 (the project plan); NASL-SHD-CR-001 (the sprayer envelope); NASL-SHD-ANR-003 (the Phase 1 report, with the operations table); docs/hardware/real_robot_components.md (the as-built record)
-->
# Shaddad UGV: project charter

## Summary

**Shaddad is a tracked unmanned ground vehicle for farm work, built by XMachines for NASL's Robotat unit.** The project gives it a digital twin: one simulation of the whole robot, verified against the real one, from which its firmware and companion software are generated. Its missions are planned and checked in the twin before they run in the field.

**In this phase** the robot mows in greenhouses and palm groves. Tillage and spraying missions come in a later phase. The robot works within the descent envelope of CR-001.

## 1. The need

Field robots on farms work near people, on slopes and in confined greenhouse aisles. A robot that stops wrongly can tip over; one that misses a person can injure them. One that runs out of energy mid-row strands its work. NASL needs to know, before the robot goes to the field, that its control software behaves safely and does its work in every situation it will meet.

The need comes from:
- the owner's directions of October and November 2026;
- the operations the robot is offered for (the operations table of the Phase 1 report, NASL-SHD-ANR-003, chapter 1).

## 2. The goals and how they are measured

Table 1. The goals of the project.

| # | Goal | Measure | Checked at |
|---|---|---|---|
| 1 | A digital twin of the robot as built | every parameter traced to the as-built record, the owner's decided upgrade (D95) or a recorded estimate; the model-against-robot audit with no unexplained difference | G1 (re-baseline), G7 (PCA) |
| 2 | The twin verified against its requirements | every requirement of the verification matrix closed: passed, or a deviation the owner accepts | G5 |
| 3 | Safe stops everywhere in the envelope | in every system scenario, every brakes-locked stop meets the park criterion of the structure spec (it starts within the safe speed of its held pitch and implement and keeps the pitch bound; CR-003); every person stop holds; no tip-over | G2 to G5 |
| 4 | Mowing missions that finish | each greenhouse and palm-grove mowing mission (phase 2) runs to its end on one charge, with at least 10 % of the usable energy above the 0.20 reserve, at the pack's end of life, from SOC 0.95 [EST until the phase-2 mowing energy run; D101; SHD-SC3] | G3, G5 |
| 5 | Firmware from the twin, on the robot | the code generated from the build-to baseline runs on the robot's controller, and its bench tests pass | Build, G5 |
| 6 | A reusable way of working | the project runs on the NASL Engineering Handbook, and its records let the next project start from it | G7 |

**Budget and schedule.** The owner sets the effort of each task group when it starts; no overall budget or end date is set at this issue. Each card records its size before it starts and its effort after it ends (NASL-SHD-PMP-001).

## 3. The constraints

- **The robot as built** (`docs/hardware/real_robot_components.md`):
  - the XMachines tracked platform;
  - a 13S15P NMC pack of about 2.95 kWh in a sealed case;
  - four 750 W BLDC motors with their ESCs, as delivered; the owner plans to replace them with four 1 kW units (D95), so the twin's baseline is the 1 kW unit, and firmware from the twin goes on the robot only with torque limits that match the motors fitted;
  - a controller board still to be confirmed, along with the ESC and BMS protocols (cards G1-04, G2-MCU).
- **The descent envelope (CR-001, the change board's decision of 2026-11-15):**
  - with no implement, the mower or the tiller (forward), the robot descends the 20° design slope within the speed cap;
  - the sprayer with a full tank descends slopes up to about 18.5° only.
- **The tiller** is not used until real tiller data exist. As estimated (85 kg, 0.90 m behind the centre), the raised tiller falls over backward at rest on flat soil in the full vehicle model (the structure engineer's step report of 2026-11-15 on card G2-DSC). Part of this comes from the model, whose rearmost track contact sits 54 mm inside the real track end.
- **The tools:**
  - MATLAB and Simulink R2026b;
  - one heavy simulation at a time on the project computer;
  - the firmware is generated from the models.
- **Safety:**
  - only the owner acts on the real robot, under an approved procedure and safety plan;
  - the 12 safety goals of the hazard log NASL-SHD-HAZ-001 are the top-level safety requirements (the owner's acceptance of 2026-11-16, D80), under the safety plan NASL-SHD-SAF-001.

## 4. The scope

Table 2. In and out of scope in this phase.

| In scope | Out of scope in this phase |
|---|---|
| The twin of every subsystem: structure and world, power and BMS, drive, MCU firmware and electronics, comms and RC, sensors and navigation, GNC and the companion | Tillage missions (card G2-TIL, a later phase) |
| The MCU firmware and the companion software generated from the twin | Spraying missions (card G2-SPR, a later phase) |
| Mowing missions in greenhouses and palm groves; transit, docking and charging | Solar farms, until the owner names a site |
| RC driving, autonomous missions, the E-stop chain and every safety function | Production at volume (gate G6) |
| The verification of the twin: simulation, bench and field campaigns | Changes to the robot's hardware (the owner's and the supplier's) |
| The project's documents and records | |

## 5. Who uses the robot

- **The operator** drives the robot by RC, starts and stops missions, and answers its holds and faults in the field.
- **The supervisor** plans the missions, checks the results and decides when the robot works. On the project, this is the project owner.
- **The maintainer** charges the pack, fits and changes implements, and inspects the brakes, tracks and sensors.
- **People nearby:** workers in the greenhouse or grove. The robot must stop for them and hold until they are clear.

The stakeholders and their needs, each with its source, are in the project plan, NASL-SHD-PMP-001 Table 1.

## 6. The operating concept in this phase

- **Places:**
  - greenhouses: narrow aisles, end walls and people at work;
  - palm groves: rows, berms and obstacles;
  - a slope test ground: the 20° design ramp.
- **The greenhouse rule** (the owner's decision of 2026-11-16, D79): people are kept out of the robot's zone while it mows a greenhouse aisle; they work in the other aisles. This sets the person-protection goals at PL d (hazard log NASL-SHD-HAZ-001, E-09).
- **Modes:**
  - RC (manual) driving;
  - autonomous missions planned in the companion: rows, turns, transit, docking;
  - the hardware E-stop chain, independent of the software.
- **Normal use:** the operator places the robot, the supervisor starts a mowing mission, and the robot mows its rows with the deck lowered. It turns at the row ends and returns to its dock to charge.
- **Abnormal use:**
  - a person in the path (hold, then resume);
  - lost RC or GNSS;
  - a sensor fault;
  - a stop on a slope (park on the brakes, within the descent cap);
  - an E-stop with reset and re-arm;
  - low energy (return to dock).

  The system scenarios exercise most of these cases:
  - the stop on a slope (S2);
  - a person in the path (S5);
  - lost RC (S6);
  - lost GNSS (S7);
  - the E-stop with reset and re-arm (S8).

  Two cases have no scenario yet: a sensor fault other than lost GNSS, and the return to the dock on low energy (S3 checks only the charge at the end of the shift; under D101 it is re-scoped to mow until the return threshold and return with the reserve, on card G2-INT). They are on cards G2-INT and T-03. The list of scenarios and their pass criteria is in `docs/architecture.md` section 12, which card G1-01 brings up to date.

The architecture document describes the operating concept in detail. Its scope section (section 1) predates the as-built robot. This charter supersedes it for the scope. Card G1-01 has re-baselined the requirements on the as-built robot (D110, 2026-11-18): the system requirements specification NASL-SHD-REQ-008 v2.0 states them, and section 1 of the architecture points to this charter and to it.

## 7. Approval

The project owner approves this charter at the G0 delta review. The approval is recorded word for word in the G0 gate review report and the decision log.
