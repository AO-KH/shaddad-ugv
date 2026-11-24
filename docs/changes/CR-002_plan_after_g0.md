<!--
doc_no: NASL-SHD-CR-002
doc_type: CHANGE REQUEST
title: Shaddad UGV: CR-002 the plan after G0
subtitle: Class 1 change to the plan baseline: a safety programme, version control, model credibility and a schedule
summary: After gate G0 the lead compared the 56 task cards with the plan a large engineering company runs for an autonomous machine that works near people and on slopes. Four things are missing: a safety programme, version control, a record of how far the model can be trusted, and a schedule. This change adds 10 cards and proposes 3 more for a later phase, adds a functional safety engineer to the team and 7 gate success criteria, and changes 20 existing cards. The owner decides as the change board.
version: 1.1
date: 2026-11-16
status: Approved
history: 1.0 | 2026-11-16 | Raised by the lead after G0, at the owner's request ("Yes, draft all")
history: 1.1 | 2026-11-16 | Decision of the change board: option 1, all of it
prepared_by: the lead systems engineer
reviewer: [TBD]
approver: the project owner (the change board)
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV CR-002
file: NASL_CR-002_Shaddad_Plan_After_G0_v1.1
outdir: work/reports/changes
request: The owner, 2026-11-16: "what do you think of redefining or add more to the tasks?", then "Yes, draft all"
related: NASL-SHD-GRR-001 (the G0 gate review and its lessons); NASL-SHD-PLN-001 v1.3 (the task cards); NASL-SHD-PMP-001 v1.2 (the plan); NASL-EMS-HBK-001 v1.1 (sections 4, 5 and 8.1); CR-001; docs/changes/CR-002_cards.json (the card set, ready to merge)
-->
# CR-002: the plan after G0

**Class 1:** it changes the plan baseline that gate G0 locked, the task cards NASL-SHD-PLN-001 v1.3 and the plan NASL-SHD-PMP-001 v1.2: new work, a new role, the effort and the gate dates (NASL-EMS-HBK-001 Table 19). **Status:** approved on 2026-11-16 (option 1); implemented in the task cards v1.4 and the plan v1.3.

## 1. The change

- **What changes:** the task cards and the plan:
  - 10 new cards, and 3 more proposed for a later phase (Appendix A);
  - 20 existing cards: their leads, gates, predecessors and exit criteria (Appendix B);
  - 7 new gate success criteria, at G1, G2, G3 and G5 (Appendix C);
  - a new role in the team: the functional safety engineer (Appendix D).
- **Why:** after G0 the owner asked whether the tasks should be redefined or extended. The lead compared the cards with the plan a large engineering company runs for an autonomous machine that works near people and on slopes. Four things are missing:
  - **A safety programme.** The specs cite ISO 18497, and the project has fixed real safety defects (card G2-SAF, CR-001), but no hazard analysis, FMEA or safety case exists. Without them the robot cannot be argued safe for field use.
  - **Version control.** Every change is tracked by copying files to a snapshot folder by hand. No gate baseline can be restored as a whole, and the configuration audit of G7 has nothing to audit against.
  - **Model credibility.** The model is the main verification tool, yet 65 parameter values are marked [EST], and the audit of the robot against the model comes only at G7 (P-01), after the simulation verification (V-01) has used the model as evidence.
  - **A schedule.** The cards have predecessors but no dates: no one can say when a gate is due, or which cards set the date.

  The change also takes in the open points of the descent card G2-DSC and the lessons of G0 (NASL-SHD-GRR-001 section 6).
- **Raised by:** the lead systems engineer, on 2026-11-16, at the owner's request: "Yes, draft all".

Table 1. The changes at a glance.

| Area | New cards | Changed cards |
|---|---|---|
| Safety programme | G1-07, G2-FSC, G3-FME, T-06, V-05 | G1-01, G1-05, G3-MCU, G3-PWR, G3-DRV, G3-NAV, G3-GNC, T-05, V-04 |
| Version control | G1-08 | G1-05 |
| Model credibility | G2-VAL | T-04, V-01, P-01 |
| Schedule | G1-10 | G1-05 |
| The team and the handbook | G1-09 | G1-01, G1-02 (their leads) |
| Test infrastructure earlier | none | T-01, T-02, T-03 (from G4 to G1) |
| The build | B-00 | B-01 |
| The descent card's open points | G2-BRK (proposed) | G2-GNC, G2-NAV, G2-STR |
| A later phase | G2-CYB, V-06 (proposed) | none |

## 2. Impact analysis

Table 2. The impact.

| Area | Impact | By |
|---|---|---|
| The plan | The task cards go to v1.4, and the plan to v1.3: the team table with the new role, the tailoring with the new gate criteria, and the schedule section. | the lead systems engineer |
| The team | A functional safety engineer leads the five safety cards, independent of the engineers who design the safety functions. | the lead systems engineer |
| Requirements | None now. G1-01 takes in the safety goals of the hazard analysis as top-level safety requirements; the safety concept (G2-FSC) adds the safety requirements in G2. | the lead systems engineer |
| Interfaces | None. | none |
| Design and models | None now. A gap the safety analyses or the validation plan find becomes its own card or change request. | the subsystem engineers |
| Tests | One new campaign: fault injection on the safety functions (T-06), with Simulink Fault Analyzer. The regression checks, a fresh test baseline and the fix of the stale tests (T-01 to T-03) move from G4 to G1, because every change from now on relies on them. | the integration engineer; the test engineer |
| Safety | Positive: the safety programme. No safety function changes now. | the functional safety engineer |
| Tools | Git and Git LFS (G1-08); the embedded toolchain (B-00). Both are installs for the owner to approve. | the project owner |
| Cost | About 38 engineering passes over G1 to G5 for the 10 new cards, about 11 of them in G1 (an S card is about one pass, an M card two or three, an L card four to six). The 3 proposed cards add about 10 more if the owner starts them. | the lead systems engineer |
| Schedule | G1 now needs the hazard analysis, version control and the schedule before its review, so it closes later by about the length of one L card. The safety cards add one chain near the end (G3-FME, T-06, V-05, before V-04). G1-10 computes the dates. | the lead systems engineer |

## 3. Options

1. **All of it (recommended).**
   - Everything in Table 1.
   - *Consequence:* the plan matches what a large engineering company runs; about 38 more engineering passes over G1 to G5; G1 closes later by about one L card.
2. **The safety programme and version control only.**
   - G1-07, G2-FSC, G3-FME, T-06, V-05 and G1-08, the new role, and their changes to existing cards.
   - *Consequence:* about 28 more passes. The model's credibility and the schedule stay informal: the simulation evidence of V-01 can be challenged, and the gate dates stay unknown.
3. **No change.**
   - *Consequence:* the plan stays as approved at G0. With no hazard analysis or safety case, the robot cannot be argued safe for field use, and changes stay tracked by hand.

## 4. Decision of the change board

**Approved: option 1, all of it.** The project owner decided on 2026-11-16, as the change board, in the owner's words: "Approve option 1".
- **The cards:** the 10 new cards of Appendix A go into the plan, and the 3 proposed cards wait for the owner to start them.
- **The changes:** the 20 changed cards of Appendix B, and the 7 gate criteria of Appendix C.
- **The team:** the functional safety engineer of Appendix D joins it.
- **Recorded:** decision row D78, the change register, the journal.

## 5. Implementation and closure (on approval)

- **The cards and the plan:** the cards of Appendix A and the changes of Appendix B go into the cards file (NASL-SHD-PLN-001 v1.4), from the card set `docs/changes/CR-002_cards.json`. The gate criteria of Appendix C and the role of Appendix D go into the plan (NASL-SHD-PMP-001 v1.3).
- **The team:** the functional safety engineer joins the team.
- **The first cards to start:** G1-08 (version control) and T-01 (the regression checks) at once; G1-07 (the hazard analysis) as soon as the safety engineer is in place; G1-10 (the schedule) once the cards are in.
- **Records:** a decision row; the change register; the document register, with the planned documents named in Appendix A.
- **Verification:** the cards PDF v1.4 shows every card of Appendix A with its successors derived, and the plan v1.3 is approved by the owner.
- **Closure:** when the cards v1.4 and the plan v1.3 are issued.

## Appendix A. The new cards

Table 3. The new cards (10 to do, 3 proposed for a later phase).

| Card | Title | Gate | Lead | Size | Predecessors |
|---|---|---|---|---|---|
| G1-07 | Hazard analysis, risk assessment and the safety plan | G1 | the functional safety engineer | L | G0-02 |
| G1-08 | Version control and configuration baselines | G1 | the configuration and documentation engineer | M | G0-02 |
| G1-09 | Handbook v1.2: the lessons of G0 and this change | G1 | the lead systems engineer | S | G0-02 |
| G1-10 | Schedule baseline: target dates and the critical path | G1 | the lead systems engineer | M | G0-02 |
| G2-FSC | Functional safety concept | G2 | the functional safety engineer | L | G1-05 |
| G2-VAL | Model credibility: the estimated-parameter register and the validation plan | G2 | the lead systems engineer | L | G1-05 |
| G3-FME | FMEA and fault-tree analysis of the safety chain | G3 | the functional safety engineer | L | G2-FSC |
| T-06 | Fault-injection campaign on the safety functions | G4 | the integration engineer | L | G3-FME, T-02 |
| V-05 | Safety case | G5 | the functional safety engineer | L | T-06, V-01, V-02, V-03 |
| B-00 | The embedded toolchain for the MCU firmware | Build | the MCU firmware engineer | M | G1-04 |
| G2-BRK | A controlled stop without skidding (later phase) | G2 | the MCU firmware engineer | L | G2-DSC |
| G2-CYB | Cybersecurity threat analysis (later phase) | G2 | the electronics and comms engineer | M | G2-COM |
| V-06 | The operator's and the maintainer's manuals for the mowing phase (later) | G5 | the technical writer | M | V-05 |

### A.1 G1-07: Hazard analysis, risk assessment and the safety plan

The robot works near people, on slopes and in greenhouse aisles, and no hazard analysis exists yet: the specs cite ISO 18497, but there is no hazard log. The functional safety engineer:
- writes the safety plan: the safety life cycle tailored to the project, the independence of the safety assessment, and the safety deliverables at each gate;
- identifies the hazards of every operating mode and abnormal case of the charter (ISO 12100, with the hazard list of ISO 18497), in the Safety Analysis Manager;
- rates each hazard, and sets its safety goal and the performance level it needs (the risk graph of ISO 25119 or ISO 13849-1);
- names the safety function that meets each goal.

The hazard log feeds the requirements baseline (G1-01) and the SRR (G1-05).

**The definition:**

- **Gate and WBS:** G1, WBS 2. **Size:** L. **Status:** to do.
- **Lead:** the functional safety engineer. **Support:** the lead systems engineer; the integration engineer; every subsystem engineer (its hazards); the project owner (the operating context).
- **Purpose:** Know every hazard of the robot in this phase, and how safe each safety function must be.
- **Entry** (G0 passed):
  - The charter NASL-SHD-CHR-001 approved: the operating concept and the CR-001 envelope;
  - The functional safety engineer in the team (CR-002);
- **Exit** (Feeds G1-01 and G1-05):
  - Every operating mode and abnormal case of the charter walked through for hazards;
  - Every hazard rated, with its safety goal, its required performance level and the safety function that meets it;
  - The safety plan: the safety life cycle, the independence of the assessment, the deliverables per gate;
  - The independent reviewer's check: no Critical or Major finding open;
  - The project owner approves the safety plan and accepts the safety goals;
- **Predecessors:** G0-02.
- **Inputs:** The charter (the operating concept, CR-001); The as-built record docs/hardware/real_robot_components.md; The safety functions in the specs: the E-stop chain, STO, the spring brakes, the person stop, the tilt stop, the descent cap; ISO 12100, ISO 18497, ISO 25119 and ISO 13849-1, at the editions G1-01 confirms.
- **Outputs:** The hazard log NASL-SHD-HAZ-001 (a Safety Analysis Manager file and its PDF); The safety plan NASL-SHD-SAF-001; The safety goals, as top-level safety requirements for G1-01.
- **Verification:** Inspection (the independent reviewer) and the owner's approval.
- **Risks:**
  - A hazard is missed because the operating concept is incomplete (likelihood M, impact H): A walk-through of every mode and abnormal case of the charter; the operator's and the maintainer's view at the review;
  - A required performance level is higher than the as-built hardware can give, for example a single-channel stop path (likelihood M, impact H): Escalate at once: a hardware change is the owner's decision;
- **Escalation:**
  - A safety goal the as-built hardware cannot meet: to the project owner (class 1);
  - A standard's edition or the jurisdiction unknown: to the project owner;

### A.2 G1-08: Version control and configuration baselines

The project has no version control: every change is tracked by copying files to a snapshot folder by hand. The configuration and documentation engineer:
- puts the project folder under git, with Git LFS for the models, data and documents (.slx, .mat, .docx, .pdf) and a .gitignore for the generated, cache and results folders;
- sets up the MATLAB project's source control and the Simulink model compare;
- tags the G0 baseline, the state the owner approved on 2026-11-16, and adds a tag at each gate;
- changes the work rules: a commit per step in place of the manual snapshots, and a separate working copy for each engineer, so two engineers never edit the same files.

A remote copy, for example a private repository, is the owner's choice.

**The definition:**

- **Gate and WBS:** G1, WBS 6. **Size:** M. **Status:** to do.
- **Lead:** the configuration and documentation engineer. **Support:** the lead systems engineer; the integration engineer (the MATLAB project and model compare).
- **Purpose:** Every change recorded, and every gate baseline restorable as a whole.
- **Entry** (G0 passed):
  - Git and Git LFS available on the project computer;
  - No engineer editing project files while the repository is created;
- **Exit** (Feeds G1-05: the G1 baseline is a tag):
  - The whole project under git: the binaries in LFS, the generated folders ignored;
  - The tag G0 on the approved baseline;
  - A restore test: a clean checkout of the tag opens the project and builds the parameters;
  - The work rules and the handbook's configuration section updated (G1-09);
  - The owner decides on a remote copy;
- **Predecessors:** G0-02.
- **Inputs:** The project folder; The snapshot folders, whose history stays as files.
- **Outputs:** The git repository with LFS; The tag G0; The updated work rules.
- **Verification:** Demonstration (the restore test).
- **Risks:**
  - The repository grows too big from results and logs (likelihood M, impact M): Ignore the results and logs; LFS for the binaries that are kept;
  - Git and the MATLAB project disagree on the files (likelihood L, impact M): The project's source control integration; the restore test;
- **Escalation:**
  - A remote host for the repository: to the project owner;
  - A tool install: to the project owner;

### A.3 G1-09: Handbook v1.2: the lessons of G0 and this change

The G0 gate review (NASL-SHD-GRR-001) recorded lessons for the handbook, and CR-002 adds a role and deliverables that belong in the company's framework, not only in this project. The lead systems engineer issues NASL-EMS-HBK-001 v1.2 with:
- the G0 lessons: the charter before the first readiness check, and the counts taken from the registers;
- the functional safety engineer in the roles and the RACI, and the safety deliverables at each gate: the hazard log at G1, the safety concept at G2, the FMEA at G3, fault injection at G4, the safety case at G5;
- the model credibility deliverables at G2: the estimated-parameter register and the validation plan;
- version control in place of the manual snapshots;
- the schedule fields of a card, and the critical path.

**The definition:**

- **Gate and WBS:** G1, WBS 1. **Size:** S. **Status:** to do.
- **Lead:** the lead systems engineer. **Support:** the independent reviewer.
- **Purpose:** The company's way of working learns from its first gate.
- **Entry** (CR-002 approved):
  - The G0 lessons (NASL-SHD-GRR-001 section 6);
- **Exit** (Feeds G1-05):
  - NASL-EMS-HBK-001 v1.2 issued, with the templates it names;
  - The independent reviewer's check;
  - The project owner approves it;
- **Predecessors:** G0-02.
- **Inputs:** NASL-SHD-GRR-001 section 6; This change request.
- **Outputs:** NASL-EMS-HBK-001 v1.2; New templates: the safety plan, the hazard log, the safety case outline, the parameter register.
- **Verification:** Inspection (the independent reviewer) and the owner's approval.
- **Risks:**
  - The handbook drifts from the project's practice (likelihood M, impact L): Change the project and the handbook in the same issue;
- **Escalation:**
  - A change to the gate structure or the approval rule: to the project owner;

### A.4 G1-10: Schedule baseline: target dates and the critical path

The cards have predecessors but no dates. The lead systems engineer:
- adds planned and actual start and finish dates to the card fields, and a target date to each gate;
- writes a tool that computes the critical path from the predecessors and the card sizes, with durations calibrated on the cards done so far;
- shows the planned dates against the actual ones on the dashboard's Project tab;
- re-projects the dates at every gate review.

The owner's decision cards are milestones on the path, with their own target dates.

**The definition:**

- **Gate and WBS:** G1, WBS 1. **Size:** M. **Status:** to do.
- **Lead:** the lead systems engineer. **Support:** the project owner (the target dates).
- **Purpose:** Know when each gate is due, and which cards set the date.
- **Entry** (CR-002 approved):
  - The card set of CR-002 in the cards file (v1.4);
- **Exit** (Feeds G1-05):
  - Every open card with a planned start and finish; every gate with a target date;
  - The critical path computed and shown;
  - The project owner accepts the gate target dates;
- **Predecessors:** G0-02.
- **Inputs:** The cards and their predecessors; The dates of the cards done so far (the journal).
- **Outputs:** The schedule fields in the cards; framework/tools/critical_path.py; The schedule section of the project plan v1.3.
- **Verification:** Inspection and the owner's acceptance.
- **Risks:**
  - Durations from the size alone are rough (likelihood H, impact L): Calibrate on the cards done; re-project at every gate;
- **Escalation:**
  - A gate target date slips by more than a week: to the project owner;

### A.5 G2-FSC: Functional safety concept

The functional safety engineer turns the safety goals of G1-07 into the functional safety concept. For each safety function (the E-stop chain, STO, the spring brakes, the person stop, the tilt stop, the descent cap, the lost-link stop, the watchdog) it states:
- the safe state and the time to reach it;
- the performance level, and the architecture that gives it: the channels, the diagnostics, the diagnostic coverage;
- its allocation to the hardware, the MCU firmware and the companion;
- how it is verified.

The safety requirements go into the requirements register, traced to the goals. The CDRs of the safety-relevant subsystems design to them.

**The definition:**

- **Gate and WBS:** G2, WBS 2. **Size:** L. **Status:** to do.
- **Lead:** the functional safety engineer. **Support:** the MCU firmware engineer; the power and BMS engineer; the drive engineer; the sensors and navigation engineer; the GNC engineer; the integration engineer.
- **Purpose:** Every safety goal met by named safety functions, allocated to the parts that carry them.
- **Entry** (G1 passed):
  - G1-05 passed: the safety goals in the requirements baseline;
  - The hazard log (G1-07);
- **Exit** (Feeds G3-FME and the CDRs of the safety-relevant subsystems):
  - Every safety goal met by one or more safety functions;
  - Every safety function with its safe state, reaction time, performance level, architecture, allocation and verification method;
  - The safety requirements in the register, traced to the goals and allocated;
  - The independent reviewer's check: no Critical or Major finding open;
- **Predecessors:** G1-05.
- **Inputs:** The hazard log and the safety goals (G1-07); The as-built record; The subsystem specs.
- **Outputs:** The functional safety concept NASL-SHD-FSC-001; The safety requirements in the register.
- **Verification:** Inspection (the independent reviewer) and Analysis.
- **Risks:**
  - A safety function needs a second channel the robot does not have (likelihood M, impact H): Escalate as a hardware change (class 1);
- **Escalation:**
  - A safety function that needs new hardware: to the project owner;

### A.6 G2-VAL: Model credibility: the estimated-parameter register and the validation plan

The model is the main verification tool, but 65 parameter values are marked [EST] in params/, and the audit of the robot against the model comes only at G7 (P-01). The lead systems engineer, with each subsystem engineer for its domain:
- lists every estimated value in a register: its owner, its basis, its uncertainty, and the test or data that will replace it;
- ranks them by the sensitivity of the outputs that safety and the goals rest on: the stop distance and the tip margin on a slope, the person-stop distance, the energy per shift, the tracking error;
- states, for each use of the model in the verification matrix, the credibility it needs and the validation evidence that gives it, in the manner of NASA-STD-7009A;
- specifies the identification tests on the bench and in the field (the slope brake test, the pitch settle, the track-soil parameters, the motor efficiency), which go into the test procedures (T-04).

**The definition:**

- **Gate and WBS:** G2, WBS 2. **Size:** L. **Status:** to do.
- **Lead:** the lead systems engineer. **Support:** every subsystem engineer (its parameters); the data analyst (sensitivity); the integration engineer.
- **Purpose:** Know how far each simulation result can be trusted before it is used as evidence.
- **Entry** (G1 passed):
  - G1-05 passed: the verification matrix (G1-02);
- **Exit** (Feeds T-04, V-01 and P-01):
  - Every estimated value in the register, with its owner and the test or data that closes it, or an accepted reason to keep it;
  - The 20 most sensitive values ranked, with the outputs they move;
  - A credibility level stated for each simulation claim of the verification matrix;
  - The identification tests specified for the test procedures;
  - The independent reviewer's check;
- **Predecessors:** G1-05.
- **Inputs:** The [EST] marks in params/; The verification matrix (G1-02); The supplier data (G1-04).
- **Outputs:** The estimated-parameter register NASL-SHD-PRM-001 (docs/registers/parameters.json); The model validation plan NASL-SHD-MVP-001.
- **Verification:** Analysis (sensitivity) and Inspection.
- **Risks:**
  - A key estimate cannot be measured on the robot (likelihood M, impact M): Supplier data (G1-04) or a conservative bound, recorded;
- **Escalation:**
  - A test that needs equipment or a site the project lacks: to the project owner;

### A.7 G3-FME: FMEA and fault-tree analysis of the safety chain

The functional safety engineer analyses the designs of the safety functions in the Safety Analysis Manager:
- an FMEA of every element of each safety function (the sensors, the MCU logic, the CAN links, the STO of the ESCs, the spring brakes, the E-stop relay, the power rails): each failure mode with its effect, its detection and the diagnostic in the design;
- a fault tree for each top hazard (a person struck, a tip-over on a slope, a runaway downhill), with the single points of failure shown;
- the diagnostic coverage achieved, against the performance level of the concept.

Every gap becomes a card or a change request. The failure modes the design detects are the cases of the fault-injection campaign (T-06).

**The definition:**

- **Gate and WBS:** G3, WBS 2. **Size:** L. **Status:** to do.
- **Lead:** the functional safety engineer. **Support:** the MCU firmware engineer; the power and BMS engineer; the drive engineer; the sensors and navigation engineer; the integration engineer.
- **Purpose:** Every way the safety chain can fail is known, and each one is detected or tolerated.
- **Entry** (G2 passed for the safety-relevant subsystems):
  - The functional safety concept (G2-FSC);
  - The CDR designs of the safety functions in review;
- **Exit** (Feeds the CDRs (G3-SC2, every required analysis closed), T-06 and V-05):
  - An FMEA row for every element of every safety function;
  - A fault tree for each top hazard, with no unmitigated single point of failure;
  - The diagnostic coverage meets the performance level of each function, or a card or CR for the gap;
  - The independent reviewer's check;
- **Predecessors:** G2-FSC.
- **Inputs:** The functional safety concept (G2-FSC); The CDR designs.
- **Outputs:** The FMEA and fault trees NASL-SHD-FMA-001 (Safety Analysis Manager files and their PDF); The list of failure modes for fault injection.
- **Verification:** Analysis and Inspection (the independent reviewer).
- **Risks:**
  - A single point of failure in the as-built stop path (likelihood M, impact H): Escalate: a design or hardware change through a CR;
- **Escalation:**
  - A single point of failure that needs a hardware change: to the project owner;

### A.8 T-06: Fault-injection campaign on the safety functions

From the FMEA (G3-FME), the integration engineer injects each failure mode that the design claims to detect into the system model with Simulink Fault Analyzer: a stuck, noisy or frozen sensor, a lost scan, a lost CAN link, an ESC fault, a brake that does not engage, a stale IMU, a lost RC link. For each fault it checks that the safe state is reached within the reaction time of the concept, and records the margin. A failure that does not reach its safe state is a nonconformance.

**The definition:**

- **Gate and WBS:** G4, WBS 4. **Size:** L. **Status:** to do.
- **Lead:** the integration engineer. **Support:** the functional safety engineer; the test engineer; every subsystem engineer of the safety chain.
- **Purpose:** Show in the model that each detected failure reaches its safe state in time.
- **Entry** (The TRR of the fault-injection campaign (T-05)):
  - The FMEA's failure-mode list (G3-FME);
  - A fresh test baseline (T-02);
- **Exit** (Feeds V-05, the safety case):
  - Every failure mode the design detects injected at least once;
  - The safe state reached within its reaction time, with the margin recorded;
  - Every failure recorded as a nonconformance, with its disposition;
  - The test report (PDF);
- **Predecessors:** G3-FME, T-02.
- **Inputs:** The failure-mode list (G3-FME); The reaction times of the concept (G2-FSC).
- **Outputs:** The fault-injection test report; The faults kept as reusable test cases.
- **Verification:** Test (simulation with fault injection).
- **Risks:**
  - A fault the model cannot represent (likelihood M, impact M): Cover it on the bench (V-02) or by analysis;
- **Escalation:**
  - A safety function that misses its safe state: to the project owner, through a nonconformance report;

### A.9 V-05: Safety case

The functional safety engineer writes the safety case: a structured argument, in goal structuring notation, that each safety goal is met. Its evidence: the hazard log, the safety concept, the FMEA and fault trees, the fault-injection results, the verification campaigns in simulation, on the bench and in the field, and the field-test records. It lists the residual risks and the operating limits, the CR-001 envelope among them. The independent reviewer assesses it, and the owner accepts the residual risks.

**The definition:**

- **Gate and WBS:** G5, WBS 2. **Size:** L. **Status:** to do.
- **Lead:** the functional safety engineer. **Support:** the lead systems engineer; the independent reviewer (the assessment); the project owner (accepts the residual risks).
- **Purpose:** A reasoned argument, with its evidence, that the robot is acceptably safe for mowing within its envelope.
- **Entry** (The verification campaigns done):
  - V-01, V-02 and V-03 done;
  - The fault-injection report (T-06);
- **Exit** (Feeds V-04, the FCA/SVR):
  - Every safety goal argued with its evidence;
  - The residual risks listed, and accepted by the project owner;
  - The operating limits listed for the operator's manual; the owner decides when the manual changes;
  - The independent assessment: no Critical or Major finding open;
- **Predecessors:** T-06, V-01, V-02, V-03.
- **Inputs:** Every safety record of G1 to G5.
- **Outputs:** The safety case NASL-SHD-SCS-001; The list of operating limits.
- **Verification:** Inspection (the independent assessment) and the owner's acceptance.
- **Risks:**
  - Thin field evidence, because only the owner runs the robot (likelihood H, impact M): Plan the field tests early (T-04, V-03); bound the claims by the evidence;
- **Escalation:**
  - A residual risk the owner does not accept: to the project owner: a design change or a narrower envelope;

### A.10 B-00: The embedded toolchain for the MCU firmware

STM32CubeMX, STM32CubeCLT and the GNU Arm toolchain are not installed, so the generated MCU code cannot be compiled for the board (VC_TARGET 1), and the code-generation test is left out. The owner installs the tools (downloads and installs are the owner's to approve). The MCU firmware engineer then checks that the generated code compiles, runs the code-generation test, and records the tool versions for the software version description (P-03). The board itself is confirmed under G1-04.

**The definition:**

- **Gate and WBS:** Build, WBS 5. **Size:** M. **Status:** to do.
- **Lead:** the MCU firmware engineer. **Support:** the project owner (downloads and installs).
- **Purpose:** The firmware generated from the model compiles for the robot's board.
- **Entry** (The owner approves the downloads and installs):
  - The controller board confirmed (G1-04);
- **Exit** (Feeds B-01):
  - The tools installed, with their versions recorded;
  - The generated code compiles for the board with no error;
  - The code-generation test runs;
- **Predecessors:** G1-04.
- **Inputs:** The controller board (G1-04); The generated code.
- **Outputs:** The installed toolchain and its versions; A compile log.
- **Verification:** Demonstration (the compile).
- **Risks:**
  - The board differs from the one the model targets (likelihood M, impact H): G1-04 confirms the board first;
- **Escalation:**
  - A download, an install or a license: to the project owner;

### A.11 G2-BRK: A controlled stop without skidding (later phase)

CR-001 limits the full-tank sprayer to about 18.5 deg, because a brakes-locked stop skids the tracks and can tip the robot over its front. A controlled stop that does not skid, with the drive braking the robot to standstill before the brakes close, would raise the safe speed and could restore the 20 deg slope (the structure engineer's proposal on card G2-DSC). Without regeneration (a full battery, CCL 0), the drive needs another way to absorb the energy, such as a braking resistor: a hardware question for the owner and the supplier.

**The definition:**

- **Gate and WBS:** G2, WBS 3.4. **Size:** L. **Status:** proposed (later phase).
- **Lead:** the MCU firmware engineer. **Support:** the drive engineer; the structure engineer; the power and BMS engineer.
- **Purpose:** Win back the 20 deg design slope for the full-tank sprayer.
- **Entry** (The owner starts the card):
  - G2-DSC closed;
  - The owner's view on a braking resistor or another energy path;
- **Exit** (A change request that restores the envelope (class 1)):
  - A stop without skidding shown in the full vehicle model on 20 deg with the full-tank sprayer;
  - The energy path without regeneration settled;
  - A CR to restore the 20 deg slope for the sprayer;
- **Predecessors:** G2-DSC.
- **Inputs:** The structure reports of G2-DSC; CR-001.
- **Outputs:** The design and its analysis; A change request.
- **Verification:** Analysis and Test (system runs).
- **Risks:**
  - The drive cannot absorb the energy at CCL 0 (likelihood H, impact H): A braking resistor (hardware: the owner's);
- **Escalation:**
  - A hardware change: to the project owner;

### A.12 G2-CYB: Cybersecurity threat analysis (later phase)

The robot is driven by RC, takes corrections over LTE (NTRIP), and links the MCU and the companion over CAN. No one has yet asked how these links could be jammed, spoofed or misused. The electronics and comms engineer runs a threat analysis of the RC link, the LTE and NTRIP link, the CAN buses and the companion's services, rates each threat and proposes the protections. A threat that could defeat a safety function goes into the hazard log. The reference standard (IEC 62443, or one for agricultural machinery) is the owner's to confirm.

**The definition:**

- **Gate and WBS:** G2, WBS 2. **Size:** M. **Status:** proposed (later phase).
- **Lead:** the electronics and comms engineer. **Support:** the functional safety engineer; the GNC engineer.
- **Purpose:** Know how the robot's links could be misused, and what protects them.
- **Entry** (The owner starts the card):
  - The link design of G2-COM;
- **Exit** (Feeds G3-COM):
  - Every external link and service with its threats rated;
  - A protection or an accepted risk for each threat;
  - The threats that touch safety in the hazard log;
- **Predecessors:** G2-COM.
- **Inputs:** The link design (G2-COM); The hazard log (G1-07).
- **Outputs:** The threat analysis report.
- **Verification:** Analysis and Inspection.
- **Risks:**
  - A protection needs new hardware or a supplier change (likelihood M, impact M): Escalate to the owner;
- **Escalation:**
  - A protection that changes the hardware or a supplier: to the project owner;

### A.13 V-06: The operator's and the maintainer's manuals for the mowing phase (later)

The manual NASL-SHD-MAN-001 predates the as-built robot and the CR-001 envelope. When the owner decides to change it, the technical writer updates the operator's manual for mowing (the modes, the holds, the faults and the operating limits of the safety case) and writes the maintainer's procedures (charging, implements, brakes, tracks and sensors).

**The definition:**

- **Gate and WBS:** G5, WBS 2. **Size:** M. **Status:** proposed (later phase).
- **Lead:** the technical writer. **Support:** the lead systems engineer; the functional safety engineer.
- **Purpose:** The people who run and service the robot have the limits and procedures of this phase.
- **Entry** (The owner decides to update the manual):
  - The operating limits of the safety case (V-05);
- **Exit** (Feeds V-04):
  - The operator's manual with every operating limit;
  - The maintainer's procedures;
  - Reviewed, and approved by the owner;
- **Predecessors:** V-05.
- **Inputs:** The safety case (V-05); NASL-SHD-MAN-001.
- **Outputs:** NASL-SHD-MAN-001 updated; The maintenance procedures.
- **Verification:** Inspection.
- **Escalation:**
  - Always: the manual changes only when the owner decides: to the project owner;

## Appendix B. The changes to existing cards

Table 4. The 20 changed cards.

| Card | Change |
|---|---|
| G1-01 | Lead: the lead systems engineer (was the configuration and documentation engineer, who now supports). Predecessor added: G1-07. Exit criterion added: the safety goals in the baseline. |
| G1-02 | Lead: the lead systems engineer (was the test engineer, who now supports). |
| G1-05 | Predecessors added: G1-07, G1-08 and G1-10. The SRR needs the hazard log, a tagged baseline and the schedule. |
| T-01 | Gate G4 to G1, priority now: the regression checks are needed by every change from now on. The id stays. |
| T-02 | Gate G4 to G1: a fresh test baseline before the G2 design work. It is run again before each campaign (G4-SC6). The id stays. |
| T-03 | Gate G4 to G1: the stale and empty tests fixed before the design work relies on them. The id stays. |
| T-04 | Predecessor added: G2-VAL. Exit criteria added: the identification tests of the validation plan, and the field-test safety plan (G4-SC4). |
| T-05 | Scope: the fault-injection campaign (T-06) gets its own TRR. |
| G2-GNC | Exit criterion added (the open point of CR-001): a slope limit per fitted implement in mission planning. |
| G2-NAV | Exit criterion added (from G2-DSC): the IMU pitch error bound confirmed for the descent table. |
| G2-STR | Exit criterion added (from G2-DSC): a 2.5 deg breakpoint in the descent table weighed. |
| G3-MCU | Predecessor added: G2-FSC. Exit criterion added: the safety requirements met, and the FMEA findings closed or on a card. |
| G3-PWR | Predecessor added: G2-FSC. Exit criterion added: the safety requirements met, and the FMEA findings closed or on a card. |
| G3-DRV | Predecessor added: G2-FSC. Exit criterion added: the safety requirements met, and the FMEA findings closed or on a card. |
| G3-NAV | Predecessor added: G2-FSC. Exit criterion added: the safety requirements met, and the FMEA findings closed or on a card. |
| G3-GNC | Predecessor added: G2-FSC. Exit criterion added: the safety requirements met, and the FMEA findings closed or on a card. |
| V-01 | Predecessor added: G2-VAL. Exit criterion added: each simulation claim cites its credibility level. |
| V-04 | Predecessor added: V-05. The FCA/SVR needs the safety case. |
| B-01 | Predecessor added: B-00. The firmware cannot be built for the board without the toolchain. |
| P-01 | Predecessor added: G2-VAL. The final audit confirms the validation evidence the plan set out. |

## Appendix C. The new gate success criteria

Table 5. The new criteria: M mandatory, D desirable (NASL-EMS-HBK-001 section 4.4).

| ID | Criterion | Class | Evidence | Threshold | From card |
|---|---|---|---|---|---|
| G1-SC9 | The hazard log and the safety goals exist, with the safety plan approved | M | the hazard log; the safety plan | every mode and abnormal case of the charter covered | G1-07 |
| G1-SC10 | The baseline is under version control and tagged | M | the repository; the tag | a restore test passes | G1-08 |
| G1-SC11 | Every gate has a target date, and the critical path is known | D | the schedule in the cards | the owner accepts the dates | G1-10 |
| G2-SC8 | The functional safety concept allocates every safety goal | M | the functional safety concept | 100 % of the goals | G2-FSC |
| G2-SC9 | Every estimated parameter has an owner and a closing test, and each simulation claim a credibility level | M | the parameter register; the validation plan | 100 % | G2-VAL |
| G3-SC7 | The FMEA and the fault trees of the safety chain are done | M | the FMEA and fault trees | every gap closed or on a card | G3-FME |
| G5-SC5 | The safety case is assessed, and the owner accepts the residual risks | M | the safety case; the assessment | no Critical or Major finding open | V-05 |

The handbook v1.2 (G1-09) adds the safety and credibility criteria to the generic gates, so the next project starts with them.

## Appendix D. The new role

Table 6. The functional safety engineer.

| Role | Does | Never | Owns |
|---|---|---|---|
| **Functional safety engineer** | Leads the safety life cycle: the safety plan, the hazard log, the functional safety concept, the FMEA and fault trees, and the safety case. Sets the required performance levels; checks that each safety requirement is met and verified; works with the subsystem engineers who design the safety functions; escalates a safety gap to the owner. | Designs or changes the safety functions it assesses: the subsystem engineers do, which keeps the assessment independent. Edits the models or the code. Closes a safety gap that needs a hardware change without the owner. | docs/safety/: the safety plan, the hazard log, the safety concept, the FMEA and fault trees, the safety case; the Safety Analysis Manager files |
