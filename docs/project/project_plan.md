<!--
doc_no: NASL-SHD-PMP-001
doc_type: PROJECT MANAGEMENT PLAN
title: Shaddad UGV: project management plan
subtitle: How the Shaddad project applies the Engineering Handbook NASL-EMS-HBK-001
summary: The Shaddad project as an instance of the NASL engineering management system: its goal and stakeholders, the life cycle as tailored, the work breakdown structure and its task cards, the team and the approval authority, the project's own success criteria, the registers and the document map, and where the project stands at this issue.
version: 1.5
date: 2026-11-18
status: Approved
history: 1.0 | 2026-11-15 | First issue, with the engineering management system (card G0-01)
history: 1.1 | 2026-11-16 | The G0 delta review: the charter referenced, the stakeholders with first-hand sources (maintainers and regulators added), the scope of this phase, SHD-SC1 from G1 onward, the team by its roles
history: 1.2 | 2026-11-16 | Corrections from the delta re-review (RID-G0-09, RID-G0-13): the reviewer and the approver by role, the signature rule, the closure of G2-DSC in the future tense
history: 1.3 | 2026-11-16 | CR-002, approved by the owner on 2026-11-16 ("Approve option 1"): the functional safety engineer in the team; the safety programme, version control, model credibility and the schedule in the WBS; the new gate criteria; the status after G0
history: 1.4 | 2026-11-16 | CR-003 (the owner's decision of 2026-11-16, "Restate by the safe-stop rule (Recommended)"): SHD-SC2 stated by the park criterion (D82)
history: 1.5 | 2026-11-18 | CR-004 (the change board's decision of 2026-11-18, "Option 1: both (Recommended)"): SHD-SC3 measured by the mission energy of D101; the regulators' row with the jurisdiction and the dated standards of D103 (card G1-01 part 2)
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV Project Plan
file: NASL_PMP-001_Shaddad_Project_Plan_v1.5
outdir: work/reports/project
chapter_breaks: no
cover_image: shaddad
request: The project owner, 2026-11-15: run the project on the life cycle and task cards, as a baseline for future projects ("Framework first", "Closed cards + doc map")
related: NASL-EMS-HBK-001 (the handbook this plan tailors); NASL-SHD-CHR-001 (the charter); NASL-SHD-PLN-001 (the task cards); NASL-SHD-CR-002 (the plan after G0); docs/registers/ (risks, decisions, changes, documents)
-->
# Shaddad UGV: project management plan

## Summary

**Shaddad is a tracked unmanned ground vehicle for agriculture** (the XMachines platform of NASL's Robotat unit), with its digital twin: a model-based simulation of the whole robot, from which its firmware is generated. This plan applies the Engineering Handbook NASL-EMS-HBK-001 to Shaddad.

**Where it stands (2026-11-16):**
- **The model is mature:** the build tasks T01 to T17 are done, and decisions D1 to D73 are recorded.
- **The life cycle is in place.** Gate G0 passed with actions on 2026-11-16 (NASL-SHD-GRR-001), and the charter NASL-SHD-CHR-001 is the baseline of the scope. CR-002 then added a safety programme, version control, model credibility and a schedule to the plan. G1 is open.
- **The cards** and their status are in the task cards NASL-SHD-PLN-001 and on the dashboard's Project tab, the single source for them.

**The next gate after G0 is G1:**
- the system requirements, re-baselined on the as-built robot;
- the verification matrix for every domain;
- the system design description;
- the hazard analysis and the safety plan;
- version control, with the G1 baseline as a tag;
- the schedule baseline.

## 1. The project

- **Purpose:** a model-based twin of the Shaddad tracked UGV. It is used to design, verify and generate the robot's firmware and companion software, and to plan its field work. The need, the measurable goals, the constraints and the scope are in the charter, NASL-SHD-CHR-001.
- **Scope of this phase** (the owner's decision of 2026-11-16):
  - mowing missions in greenhouses and palm groves;
  - tillage and spraying missions come in a later phase (cards G2-TIL and G2-SPR);
  - solar farms stay out until a site is named.
- **The product and its subsystems:**
  - structure and world;
  - power and BMS;
  - drive;
  - MCU firmware and electronics;
  - comms and RC;
  - sensors and navigation;
  - GNC and the companion computer.

  The integration of the whole is WBS element 4 (`project.json`).
- **Constraints:**
  - MATLAB/Simulink R2026b;
  - firmware for the robot's controller board;
  - one heavy simulation at a time on the project computer;
  - the owner acts on the real robot.

Table 1. Stakeholders and their needs.

| Stakeholder | Need | Source |
|---|---|---|
| The project owner (NASL, Robotat) | a trustworthy twin of the real robot, its firmware, and a reusable way of working | the owner's directions of 2026-10-23 (Phase 1 acceptance), 2026-10-27 (the as-built robot) and 2026-11-15 (the engineering management system) |
| Farm, greenhouse and palm-grove customers | mowing now; tillage and spraying later; work done in time, on one charge | the operations table of the Phase 1 report (NASL-SHD-ANR-003, chapter 1); the owner's scope decision of 2026-11-16 |
| Operators in the field | safe driving on slopes and near people; clear holds, stops and recovery | the Phase 1 user manual (NASL-SHD-MAN-001); the RC row test and live drive procedures (NASL-SHD-TP-001, TP-002) |
| Maintainers | charging, implement changes, brake, track and sensor inspection; faults that name their cause | the user manual (NASL-SHD-MAN-001); the hardware bring-up procedure (NASL-SHD-TP-003) |
| Regulators and standards | in the Kingdom of Saudi Arabia (D103): SASO, through SABER, for machinery (TR 01-08-21-180, 2021, with the standards its Annex 1-A adopts), batteries (TR 02-04-18-166) and low-voltage equipment; CST for the radios and their frequencies; the Ministry of Human Resources and Social Development for safety at work (the Labor Law, Royal Decree M/51). The state of the art, dated: ISO 12100:2010, ISO 18497-1 to -4:2024, ISO 13849-1:2023 (the rating method) and the rest of the safety plan's Table 1. The mower standard is chosen on G2-FSC, because ISO 5395 does not cover the robot | the safety plan NASL-SHD-SAF-001 v1.3, section 3 and Table 1 (D103) |
| The supplier (XMachines) | the robot's platform, ESC, BMS and controller used as delivered | the as-built record (docs/hardware/real_robot_components.md) |
| NASL engineers of later projects | a framework and records they can understand and reuse | the owner's direction of 2026-11-15 |

## 2. The life cycle as tailored

Table 2. The gates of Shaddad and their tailoring (`tools/dashboard/project_cards.json`, gates).

| Gate | Held as | Tailoring and reason |
|---|---|---|
| G0 MCR | once, retrospectively (card G0-02) | the project started in October 2026, before the framework; the evidence is scored as for any gate |
| G1 SRR + SFR | once, for the system (G1-05) | SRR and SFR merged: one small team, one review |
| G2 PDR | per subsystem (G2 cards) | the subsystems mature at different speeds |
| G3 CDR | per subsystem (G3 cards) | as G2 |
| Build | per item: MCU firmware, companion, robot instrumentation | three different targets |
| G4 TRR | per campaign: simulation, bench and HIL, field, fault injection (T-05) | each campaign needs its own readiness |
| G5 FCA / SVR | once (V-04) | — |
| G6 PRR | not held | no production is planned |
| G7 PCA | once (P-03) | — |

**Success criteria:** the standard criteria of NASL-EMS-HBK-001 section 4.4, scored on every gate in the cards file, plus the project's own criteria of section 5 below. CR-002 adds seven to the gates: the hazard log, version control and the schedule at G1 (G1-SC9 to SC11); the safety concept and the model credibility at G2 (G2-SC8, SC9); the FMEA at G3 (G3-SC7); the safety case at G5 (G5-SC5). The handbook v1.2 makes them standard (card G1-09).

**Approval:** the owner decides every gate, after the lead's readiness check and the reviewer's independent assessment. The outcomes are go, go with actions, delta review or no-go.

## 3. The work breakdown structure

Table 3. The WBS of Shaddad and its leads.

| Code | Element | Lead |
|---|---|---|
| 1 | Project management: the plan, the decisions, the gate reviews | the lead systems engineer |
| 2 | Systems engineering: requirements, architecture, verification matrix, design description, model credibility; the safety programme: hazard analysis, safety concept, FMEA, safety case | the lead systems engineer, with the configuration and documentation engineer; the functional safety engineer for the safety programme |
| 3.1 | Structure and world | the structure engineer |
| 3.2 | Power and BMS | the power and BMS engineer |
| 3.3 | Drive | the drive engineer |
| 3.4 | MCU firmware and electronics | the MCU firmware engineer |
| 3.5 | Comms and RC | the electronics and comms engineer |
| 3.6 | Sensors and navigation | the sensors and navigation engineer |
| 3.7 | GNC and the companion | the GNC engineer |
| 4 | Integration, verification and validation: the top level, scenarios, test campaigns | the integration engineer, with the test engineer |
| 5 | Build, release and deployment: firmware, the companion build, the robot's instrumentation | the MCU firmware engineer, the GNC engineer |
| 6 | Tools and infrastructure: version control, the dashboard, builders, runners, regression checks, layout | the lead systems engineer; the configuration and documentation engineer for version control |

The task cards are the WBS dictionary: NASL-SHD-PLN-001, built from `tools/dashboard/project_cards.json` by `tools/reports/build_task_cards.py`. Every card has one WBS code and one gate.

## 4. The team and the approval authority

- **The project owner (the user)** is the single approval authority. The owner:
  - decides the requirements, the safety policy, the hardware, the suppliers and the budget;
  - chairs the change board;
  - approves every gate;
  - is the only one who acts on the real robot.
- **The lead systems engineer:**
  - plans the cards and assigns the engineers;
  - keeps the registers, the dashboard and the records;
  - chairs the gate reviews;
  - writes the task reports.
- **The subsystem engineers,** one per subsystem: structure; power and BMS; drive; MCU firmware; electronics and comms; sensors and navigation; GNC.
- **The specialists:** the integration engineer, the test engineer, the independent reviewer, the functional safety engineer, the configuration and documentation engineer, the data analyst and the technical writer. They work to the team's shared work rules and step-report format.
- **The functional safety engineer** (CR-002) leads the safety life cycle: the safety plan, the hazard log, the functional safety concept, the FMEA and fault trees, and the safety case. It does not design the safety functions it assesses; the subsystem engineers do, which keeps the assessment independent.
- **The project's standing rules** (on top of the handbook):
  - MATLAB runs in batch only, one heavy run at a time;
  - never build every MCU unit at once;
  - P stays numeric;
  - the owner's folder `docs/report/` is never edited;
  - one pass per task;
  - documents name the reviewer and the approver by role; personal names stay [TBD] until the owner names people.

## 5. The project's own success criteria

These add to the standard criteria of the gates they name.

Table 4. Shaddad's own criteria.

| ID | Gate | Criterion | Threshold | Class |
|---|---|---|---|---|
| SHD-SC1 | G1 onward (G0 locks no model) | the tests against the current baseline | 0 new failures; every known failure dispositioned | M |
| SHD-SC2 | G2, G3 | every brakes-locked stop meets the park criterion of the structure spec, section 2.11: it starts within the safe speed of its held pitch and implement and keeps the pitch bound (CR-003, D82) | in every system scenario | M |
| SHD-SC3 | G2 (power) | the energy of each mowing mission at the pack's end of life (D101) | ≥ 10 % of the usable energy above the 0.20 reserve [EST until the phase-2 mowing energy run; D101] | M |
| SHD-SC4 | G3 | model layout | 0 line or block overlaps; crossings within the allowance | D |
| SHD-SC5 | G4 (field) | every field test has the owner's approval and a safety plan | 100 % | M |

## 6. The registers and the documents

Table 5. Where the records are kept.

| Record | Where | Rule |
|---|---|---|
| Task cards and gates | `tools/dashboard/project_cards.json` | NASL-EMS-HBK-001 chapter 5 |
| Requirements and verification | `docs/requirements/<domain>.yaml` (power so far) | section 7.2; the other domains by card G1-02 |
| Risks and issues | `docs/registers/risks.json` | section 8.3 |
| Decisions | the decisions table of `docs/architecture.md`; index `docs/registers/decisions.json` | section 8.2 |
| Changes and nonconformances | `docs/registers/changes.json` | sections 8.1, 8.4 |
| Documents | `docs/registers/documents.json` (the document map) | section 8.5 |
| Safety records | `docs/safety/`: the safety plan, the hazard log, the safety concept, the FMEA and fault trees, the safety case | CR-002; cards G1-07, G2-FSC, G3-FME, V-05 |
| Estimated parameters | `docs/registers/parameters.json` | CR-002; card G2-VAL |
| Versions and baselines | git, with a tag at each gate | CR-002; card G1-08 |
| Journal and work log | the dashboard's journal and work log | the records |

**The document map.** The existing documents keep their content and get a type and a number (the owner's choice of 2026-11-15):
- **CON-001:** the architecture and operating concept (with the decision log).
- **REQ-001 to 007:** the seven domain specifications.
- **ICD-001:** the interface document.
- **PLN-002:** the build plan.
- **TP-001 to 003:** the test procedures.
- **ER-001 to 004:** the engineering reports.
- **ANR-001:** the review report.
- **MAN-001:** the user manual.
- **Planned by CR-002:** SAF-001 the safety plan, HAZ-001 the hazard log, FSC-001 the functional safety concept, FMA-001 the FMEA and fault trees, SCS-001 the safety case, MVP-001 the model validation plan, PRM-001 the estimated-parameter register.

The full list is in `docs/registers/documents.json`. The domain specifications are regenerated from the requirements register at G1 (cards G1-01 and G1-02).

## 7. The work done before the framework

The work done before 2026-11-15 is recorded as closed cards with its records:
- **G0-00:** the project start;
- **H-01:** the simulation model build, T01 to T17;
- **H-02:** the RC row test;
- **H-03:** the model layout clean-up;
- **H-04:** the interconnected top level;
- **H-05:** the engineering team set-up and the dashboard;
- **H-06:** the BMS-warning derate, D65;
- **H-07:** the precharge requirements;
- **H-08:** the S4 analyses;
- **G3-S4, G3-MF, G1-00 and G2-SAF:** the S4 fixes, the MATLAB Function conversion, the review and the critical fixes, with engineering reports ER-001 to ER-004.

The descent task (G2-DSC) will be the first card closed under the framework.

## 8. Status at this issue

- **Gates:**
  - **G0:** passed with actions on 2026-11-16 (NASL-SHD-GRR-001). Open on card G1-01, before G1-05: the risk register review (RID-G0-08) and architecture section 12 (RID-G0-12).
  - **G1:** open. Its criteria now include the hazard log, version control and the schedule (CR-002).
  - **G2 to G7:** open, not yet assessed.
- **Changes:** CR-001 approved on 2026-11-15, in implementation on card G2-DSC; CR-002 approved on 2026-11-16 ("Approve option 1"), implemented in the plan v1.3 and the task cards v1.4; CR-003 approved on 2026-11-16 (option 1), implemented in this issue and the charter v1.3.
- **Cards under way:**
  - G2-DSC, the descent cap: the system runs;
  - G1-07, the hazard analysis and the safety plan.
- **Next:**
  - G1-08 (version control), once the descent runs end;
  - T-01 to T-03 (the regression checks, a fresh test baseline, the stale tests), now in G1;
  - G1-10 (the schedule);
  - G1-03 and G1-04 (the open decisions and the supplier data, with the owner);
  - G1-09 (the handbook v1.2).
- **The critical path and the full card set:** NASL-SHD-PLN-001; the schedule fields come with G1-10.
- **The top risks:** `docs/registers/risks.json`.
