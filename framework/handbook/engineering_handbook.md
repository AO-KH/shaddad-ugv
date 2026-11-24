<!--
doc_no: NASL-EMS-HBK-001
doc_type: ENGINEERING HANDBOOK
title: Engineering Handbook
subtitle: The engineering management system: how we plan, build, verify and document a project
summary: How every NASL engineering project is run, from the first request to the final baseline: the life cycle and its gates, the success criteria and the approval of each gate, the task cards that hold all work, change control, requirements and verification, risks, documents, roles, the team and the tools. The handbook is the same for every project; each project tailors it in its own project plan.
version: 1.1
date: 2026-11-16
status: Approved
history: 1.0 | 2026-11-15 | First issue, written on the project owner's direction of 2026-11-15
history: 1.1 | 2026-11-16 | The team named by its roles; chapter 9 the team and its work rules; the task card description field; sizes by scope
prepared_by: the lead systems engineer
reviewer: [TBD]
approver: [TBD]
entity: NASL Technologies
classification: Internal
footer: Engineering Handbook
file: NASL_EMS-HBK-001_Engineering_Handbook_v1.1
outdir: framework/build
chapter_breaks: yes
cover_image: none
request: The project owner, 2026-11-15: run the project on the life cycle and task cards, "like a real big engineering company", as the baseline for future projects; every gate with success criteria and a way to approve the move to the next phase
related: ISO/IEC/IEEE 15288:2023; IEEE 15288.2-2014; NASA SP-2016-6105 Rev 2; the templates in framework/templates; the first project made with it: Shaddad UGV (NASL-SHD)
-->
# Engineering Handbook

## Summary

**This handbook is the way NASL runs an engineering project.** It applies to every project, large or small, with or without simulation models. One idea holds it together: **all work is planned, done and recorded as task cards, under a life cycle whose gates are passed only on measured success criteria and an explicit approval.**

**The five things to remember:**
1. **The life cycle** has phases separated by gates G0 to G7 (Figure 2). Each gate has written success criteria. A gate is passed when a readiness check shows the criteria met, an independent assessment raises its findings, and the project owner decides: go, go with actions, a delta review, or no-go (section 4.5).
2. **Every piece of work is a task card** (chapter 5): its purpose, its lead, its entry and exit gates with their criteria, its predecessors and successors, its inputs and outputs, its verification, its size, its risks and its escalation. No card, no work.
3. **Baselines change only through change control** (section 8.1). A change request states what changes and why, the owners analyse its impact, and the project owner decides.
4. **Every requirement is verifiable and verified** (sections 7.2 and 7.6). The verification matrix links each requirement to its test, analysis, inspection or demonstration, and to its result.
5. **Everything is recorded where it belongs:** cards, registers (requirements, risks, decisions, changes), controlled documents with numbers and versions, and the dashboard that shows them live. Each task group ends with its PDF engineering report.

**How to use it:** read chapters 2 to 6 once, to know how the work flows. Use chapters 7 and 8 as the reference when you do that kind of work, the appendices for numbers and templates, and chapter 12 to start a new project. Chapter 11 follows one real task from the request to the closed card, to show how the parts fit together.

## 1. Purpose, scope and use

### 1.1 Why this handbook exists

The project owner set the direction on 2026-11-15: "everything should be done a professional way like a real big engineering company", built "as a baseline for future projects", and "what is more important is to understand how things work".

Large engineering companies do not depend on memory or on one person's habits. They run every project through a written management system. It sets:
- the phases and the reviews that end them;
- how work is defined and assigned;
- how a decision is made and kept;
- how a change is controlled;
- how a requirement is shown to be met.

This handbook is that system for NASL. It explains **what** to do, **who** does it, **when**, and **why**, so that anyone can understand how a project works and repeat it.

### 1.2 Scope

- **Applies to:** NASL research and development projects: robots, vehicles, electronics, software, models and their verification. It is written for a small team: the project owner, a lead systems engineer, an engineer per subsystem and a few specialists.
- **Covers:** the life cycle and its gates; planning with a work breakdown structure and task cards; the engineering processes from needs to release; and the management processes (configuration and change control, decisions, risks, documents, quality, measurement). It also covers the team, the tools, and how to start a new project.
- **Does not cover:** finance and contracts (only the effort of the work is tracked here), and production at volume (gate G6 is a placeholder until a project needs it).

### 1.3 The two layers

The system has two layers (Figure 1).
- **The framework** is the same for every project: this handbook, the templates (`framework/templates/`) and the tools (the dashboard, the document and report builders, the batch runner, the team kit).
- **The project instance** is made with the framework, once per project. It holds:
  - the project plan, which tailors the life cycle;
  - the work breakdown structure and the task cards;
  - the registers;
  - the baselines and the controlled documents;
  - the dashboard.

Lessons from each project flow back into the framework as a new version of this handbook.

![Figure 1: The two layers: the framework, the same for every project, and the project instance made with it.](framework/handbook/fig/f1_layers.svg)

### 1.4 The standards it follows

The handbook tailors international practice for a small team; it does not invent its own:
- **ISO/IEC/IEEE 15288:2023,** system life cycle processes. Appendix C maps each process of the standard to the section of this handbook that covers it.
- **IEEE 15288.2-2014 and MIL-STD-1521B:** the technical reviews and audits (SRR, SFR, PDR, CDR, TRR, FCA, SVR, PRR, PCA) and their entry and exit criteria.
- **The NASA Systems Engineering Handbook** (SP-2016-6105 Rev 2): the life cycle, gate products, tailoring and the technical management practices.
- **ISO/IEC/IEEE 29148:2018** (requirements), **ISO 10007:2017** (configuration management) and **ISO 31000:2018** (risk): the principles of the sections that carry those names.

### 1.5 How to read it

- **"Shall"** marks a rule every project follows; a project may tailor it only in its project plan, with a reason. **"Should"** marks a recommendation.
- **"The owner"** is the project owner: the approval authority of the project. **"The lead"** is the lead systems engineer, who plans, assigns and records the work. Chapter 3 defines all roles.
- Terms and abbreviations are in Appendix D.

## 2. Principles

The ten principles behind every rule of this handbook. When a situation is not covered, apply the principles.

Table 1. The principles of the engineering management system.

| # | Principle | Why |
|---|---|---|
| 1 | **Every piece of work is a task card.** No work starts without a card that says why, when it may start and when it is done. | Work that is not written down cannot be planned, checked or learned from. |
| 2 | **One source of truth per kind of record.** Cards, registers, decisions and document sources are edited in one place; everything else is generated from them. | Two copies of a fact drift apart; then nobody knows which is right. |
| 3 | **Gates are passed on evidence.** Each gate has success criteria; the owner approves the move to the next phase after a readiness check and an independent assessment. | Moving on before the base is sound multiplies the cost of every later fix. |
| 4 | **Baselines change only through change control.** | An uncontrolled change breaks what others built on. |
| 5 | **Every requirement is verifiable, and verified.** | A requirement that cannot be checked is a wish; an unchecked one is a risk. |
| 6 | **Decide explicitly.** Each decision is recorded with its options and reasons; the owner decides what is theirs. | Unrecorded decisions get reopened, or silently reversed. |
| 7 | **The owner of a part changes it; an independent reviewer checks it before it is kept.** | A second pair of eyes finds what the author cannot see. |
| 8 | **Change a generated artefact at its generator.** | A hand edit to a generated file is lost at the next build. |
| 9 | **One pass.** Plan well, do the work once, fix real breakage once; do not polish in loops. | Repeated rounds cost more than they find. |
| 10 | **Document as you go, and count the cost.** Each card ends with its record; effort is estimated before and measured after. | The record is how the next person understands; the cost is how the owner steers. |

## 3. Organisation and roles

### 3.1 Roles

Table 2. The roles of a project.

| Role | Who | Responsibilities |
|---|---|---|
| **Project owner** | the person who owns the project (for Shaddad: the user) | Sets the goals and the budget. Decides the requirements, the safety policy, the hardware and the suppliers. Chairs the change board and approves each gate. Is the only one who acts on the real hardware, changes security settings, or sends or publishes. |
| **Lead systems engineer** | one, for the project | Plans the work as cards, writes the briefs, dispatches the engineers, keeps the registers and the dashboard, and chairs the gate reviews. Puts each decision to the owner with options and a recommendation, and writes the task and gate reports. |
| **Subsystem engineer** | one per subsystem | Leads the cards of the subsystem and edits only its own files. Works in one pass with snapshots and writes a step report. |
| **Integration engineer** | a specialist | Owns the top level, the interfaces and the system scenarios, and checks every change that crosses subsystems. |
| **Test engineer** | a specialist | Runs the tests and sorts the results against the test baseline. Keeps the known failures. |
| **Independent reviewer** | a specialist who did not do the work | Checks every change, and every gate package, against the request and the evidence. Returns ranked findings and a verdict: keep, fix, or ask the owner. |
| **Documentation owner** | a specialist | Keeps the controlled documents, the interface document, the decision log and the status in line with every change. |
| **Data analyst, technical writer** | specialists | Analyse run and field logs; write formal reports. |

One person may hold several roles, except that **the independent reviewer of a change shall not be its author.**

### 3.2 Who does what

The responsibility matrix (R does it, A is accountable and approves, C is consulted, I is informed).

Table 3. Responsibility matrix.

| Activity | Owner | Lead | Subsystem lead | Integration | Test | Reviewer | Docs |
|---|---|---|---|---|---|---|---|
| Set goals, budget, requirements | A | R | C | C | I | I | I |
| Create or change a card | A | R | C | C | I | I | I |
| Do the work of a card | I | A | R | C | C | I | I |
| Check a cross-subsystem change | I | A | C | R | C | I | I |
| Run and sort the tests | I | A | C | C | R | I | I |
| Review a change before it is kept | I | A | C | C | C | R | I |
| Update the docs | I | A | C | C | I | I | R |
| Raise a change request | C | R | R | R | R | R | R |
| Approve a change request | A | R | C | C | C | C | I |
| Readiness check of a gate | I | R | C | C | C | C | C |
| Independent assessment of a gate | I | A | C | C | C | R | I |
| Gate decision | A | R | C | C | C | C | I |
| Any action on the real hardware | A, R | C | C | C | C | I | I |

### 3.3 Escalation

When a problem cannot be solved where it is, it goes up the ladder. Each card names its own triggers; all follow this ladder. **A safety finding goes to the lead at once and to the owner the same day, whatever the card.**

Table 4. The escalation ladder.

| Level | To | When | How |
|---|---|---|---|
| 1 | the card's subsystem lead | a check fails for a reason inside the lead's own files | fix it in the same pass |
| 2 | the lead | the work needs another owner's file, an interface change (a bus, an enumeration, an interface row) or a test's expected value; a check fails twice; an engineer is blocked; the cost passes the estimate | at once, in the step report, as a question or a request for a ruling |
| 3 | the owner | a decision that changes a requirement, the safety policy, the hardware or a supplier; cost beyond the approved budget; any action on the real hardware; a security setting; anything to be sent or published | a question with the options, a recommendation and the consequences; the answer becomes a decision record |

## 4. The life cycle and its gates

### 4.1 Phases and gates

A project moves through eight phases. A gate ends each phase: a formal review that decides whether the project may enter the next phase (Figure 2).

![Figure 2: The phases and gates of the life cycle, with the baseline each gate locks.](framework/handbook/fig/f2_lifecycle.svg)

Table 5. The gates.

| Gate | Review | The question it answers | It locks |
|---|---|---|---|
| G0 | Mission concept review (MCR) | Is the need understood, and is the concept worth developing? | the project charter |
| G1 | System requirements and functional review (SRR + SFR) | Do we know what to build? Can every requirement be verified? Is the functional architecture sound and allocated? | the **functional baseline** |
| G2 | Preliminary design review (PDR), per subsystem | Does the chosen design meet the requirements with an acceptable risk? | the **allocated baseline** (design-to) |
| G3 | Critical design review (CDR), per subsystem | Is the detailed design complete enough to build, code and integrate? | the **build-to baseline** |
| G4 | Test readiness review (TRR), per campaign | Are we ready to start formal verification of this campaign? | the go-ahead for the campaign |
| G5 | Functional configuration audit and system verification review (FCA / SVR) | Does the tested system meet every requirement? | the **verified baseline** |
| G6 | Production readiness review (PRR), only if production starts | Can we build it repeatably? | the production baseline |
| G7 | Physical configuration audit (PCA) | Does what we built match the documentation? | the **final product baseline** |

The parallel software track (the software requirements review, then the software PDR and CDR) runs inside the subsystem gates G2 and G3 of each subsystem that has software.

### 4.2 Baselines

A baseline is an agreed state of a set of items. It changes only through change control (section 8.1).

Table 6. The baselines.

| Baseline | Locked at | Contents |
|---|---|---|
| Functional | G1 | the system requirements, the operating concept, the functional architecture, the verification methods |
| Allocated | G2 (per subsystem) | the subsystem requirements, the interfaces, the budgets (mass, power, energy, timing), the preliminary design |
| Build-to | G3 (per subsystem) | the detailed design: the models and builders, the parameters, the code design, the qualification test plan |
| Verified | G5 | the verified configuration, with its test reports, deviations and waivers |
| Final product | G7 | the as-built configuration matching the documents, with the software version description |

Each baseline shall be identified: the versions of its documents, a snapshot or tag of its models and code, and the test baseline of that date.

### 4.3 Tailoring

Each project tailors the life cycle in its project plan, and records every tailoring with its reason. Typical choices for a small team:
- G0 can be a short charter review.
- SRR and SFR are merged into G1.
- PDR and CDR are held per subsystem, not for the whole system.
- G4 is held once per test campaign: simulation, bench or hardware-in-the-loop, field.
- G6 is omitted until production starts.
- Two gates of one subsystem can be held together when its risk is low; the review package then holds the products of both.

Tailoring shall not drop the verification matrix or the independent assessment of a gate.

### 4.4 Gate success criteria

**Every gate has written success criteria: the measurable conditions that show the project is ready for the next phase.** Each criterion has an ID, the evidence that shows it, a threshold, and a class:
- **M (mandatory):** it must be met, or waived by the owner with a recorded reason.
- **D (desirable):** it should be met; a gap becomes an action.

The standard criteria are below, and machine-readable in `framework/templates/gate_success_criteria.json`. A project adds its own criteria in its project plan, for example a budget margin.

Table 7. G0: mission concept review.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G0-SC1 | The need, the goals and the constraints are written | the project charter | approved by the owner | M |
| G0-SC2 | The stakeholders and their needs are listed | the stakeholder list | each need has a source | M |
| G0-SC3 | An operating concept describes how the product is used | the ConOps | scenarios for the main uses | M |
| G0-SC4 | The life cycle is tailored, the WBS and the first cards exist | the project plan; the cards | every phase-1 card has entry and exit criteria | M |
| G0-SC5 | The top risks are known | the risk register | each High risk has an owner | D |

Table 8. G1: system requirements and functional review.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G1-SC1 | Every stakeholder need traces to system requirements | the trace in the requirements register | 100 % | M |
| G1-SC2 | Every requirement meets the quality rules (section 7.2) | the reviewer's check | 0 open quality defects | M |
| G1-SC3 | Every requirement has a verification method and a planned verification | the verification matrix | 100 % (or recorded as unverifiable, with a reason) | M |
| G1-SC4 | Every requirement is allocated to subsystems | the allocation in the register | 100 % | M |
| G1-SC5 | The functional architecture and the interface document exist and match the model | the architecture; the interface document; an automatic check | 0 contradictions | M |
| G1-SC6 | No TBD on a safety requirement; every other TBD has an owner and a date | the register | 0 safety TBDs | M |
| G1-SC7 | Every High risk has a mitigation card | the risk register; the cards | 100 % | M |
| G1-SC8 | All G1 cards done; no Critical or Major finding open | the cards; the findings list | 100 %; 0 | M |

Table 9. G2: preliminary design review, per subsystem.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G2-SC1 | The design meets every allocated requirement on paper | the compliance matrix | 100 % compliant, or each gap dispositioned | M |
| G2-SC2 | The budgets close with the margin set in the project plan | the budgets: mass, power, energy, thermal, timing | margin ≥ the plan's value | M |
| G2-SC3 | The interfaces to other subsystems are agreed | the interface rows, approved by both sides | 100 % | M |
| G2-SC4 | The key trade studies are done and recorded | the decision records | each with options and reasons | M |
| G2-SC5 | Model or prototype results show feasibility for the riskiest requirements | the analysis or test reports | each risky requirement shown | D |
| G2-SC6 | Every High risk of the subsystem has a mitigation card | the risk register | 100 % | M |
| G2-SC7 | All the subsystem's G2 cards done; no Critical finding open | the cards; the findings | 100 %; 0 | M |

Table 10. G3: critical design review, per subsystem.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G3-SC1 | The detailed design is complete | the builders, models and parameters; the design description | no open TBD that affects an interface | M |
| G3-SC2 | Every required analysis is closed | the analysis reports | 100 % | M |
| G3-SC3 | The qualification test plan names the test of every requirement | the test plan; the matrix | 100 % | M |
| G3-SC4 | Every High and Mid review item is closed or waived | the findings list | 0 open without a disposition | M |
| G3-SC5 | The subsystem's tests pass against a fresh test baseline | the test run sorted against the baseline | 0 new failures; each known failure dispositioned | M |
| G3-SC6 | The build approach and the configuration (versions) are identified | the build-to baseline record | complete | M |

Table 11. G4: test readiness review, per campaign.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G4-SC1 | An approved procedure exists for every test of the campaign | the test procedures | 100 %, approved by the owner | M |
| G4-SC2 | The test article is identified and matches the baseline | the configuration record | versions match | M |
| G4-SC3 | The equipment and environment are ready | a check list | all items ready, calibrated where needed | M |
| G4-SC4 | A safety plan exists for hardware and field tests | the safety plan | approved by the owner | M |
| G4-SC5 | Every test has pass/fail criteria and data recording | the procedures | 100 % | M |
| G4-SC6 | The regression checks are green; known failures dispositioned | the test baseline | 0 new failures | M |
| G4-SC7 | The open findings of the CDR do not block the campaign | the findings list | 0 blocking | M |

Table 12. G5: functional configuration audit and system verification review.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G5-SC1 | Every requirement is verified | the closed verification matrix | 100 % passed, or a deviation or waiver approved | M |
| G5-SC2 | A test report is released for every campaign | the test reports | 100 % | M |
| G5-SC3 | Every nonconformance is dispositioned | the nonconformance register | 0 open | M |
| G5-SC4 | The owner accepts the operational scenarios (validation) | the validation record | accepted | M |

Table 13. G6 (if production) and G7.

| ID | Criterion | Evidence | Threshold | Class |
|---|---|---|---|---|
| G6-SC1 | Manufacturing documents, suppliers and processes ready | the production plan | complete | M |
| G7-SC1 | The as-built item matches the documents | the audit checklist | 0 unexplained differences | M |
| G7-SC2 | The software version description is complete | the SVD | every model, firmware and package with its version | M |
| G7-SC3 | Every controlled document is released at its final version | the document index | 100 % | M |
| G7-SC4 | The lessons learned are recorded | the lessons learned report | done | D |

### 4.5 The readiness check and the approval

**A gate is passed in five steps** (Figure 3). The record of each step is kept with the gate.

![Figure 3: How a gate is approved: the readiness check, the review package, the independent assessment, the gate review, and the owner's decision.](framework/handbook/fig/f7_gate.svg)

1. **Readiness check (the lead).** When all cards under the gate are done, or at the date the plan sets, the lead scores every success criterion:
   - **met:** the evidence shows it;
   - **partly met:** some evidence, with a gap;
   - **not met;**
   - **waived:** the owner accepted a gap, with a reason;
   - **not applicable:** the project plan tailored it out.
   - **The readiness** is the share of mandatory criteria that are met or waived. The lead recommends a review when it is 100 %, or a review with known gaps, which becomes a delta review on those criteria.
2. **Review package (the lead and the subsystem leads).** The evidence of every criterion, linked from the readiness checklist (`framework/templates/gate_readiness_checklist.md`).
3. **Independent assessment (the reviewer).** The reviewer checks the package against the criteria, without the authors' reasoning. Each finding is raised as a review item discrepancy (RID) with its criterion, a severity and a proposed action:
   - **Critical:** safety, or a mandatory criterion not met;
   - **Major:** a result or a baseline affected;
   - **Minor:** a document or a presentation.
4. **Gate review (the board).** The owner (approver), the lead (chair), the reviewer and the leads concerned walk through the criteria and the RIDs, and agree an action, an owner and a date for each RID.
5. **Decision (the owner), then the record.** Table 14 gives the four outcomes and their rules. The lead records the decision as a decision record, writes the gate review report (PDF), sets the gate status, and locks the baseline.

Table 14. The gate decision.

| Outcome | Rule | What happens next |
|---|---|---|
| **Go** | every mandatory criterion met or waived; no Critical or Major RID open | the next phase starts; the baseline is locked |
| **Go with actions** | every mandatory criterion met or waived; Major or Minor RIDs open, each on a card with a date; none touches safety | the next phase starts; the open RIDs shall close before the next gate |
| **Delta review** | one or more mandatory criteria not met, and the gaps can be closed within the phase | fix the gaps on cards; then a short review of those criteria only |
| **No-go** | a basic problem: the approach does not work or the base is unsound | the project stays in the phase and re-plans; the owner may also stop it |

**Waivers.** Only the owner waives a mandatory criterion. Each waiver is recorded with its reason, its risk, and the date it expires.

**The approval itself.** The lead puts the decision to the owner as one question, with:
- the readiness score and the open RIDs;
- the recommended outcome first, with its consequence.

The owner's answer is recorded word for word in the decision record and the gate report. Until then the gate shows **in review** on the dashboard. A gate shall not be shown as passed without that recorded decision.

## 5. Planning the work: the WBS and the task cards

### 5.1 The work breakdown structure

The work breakdown structure (WBS) divides the whole project into elements, down to the work packages that are done, checked and closed. It is **product-oriented**: it follows the parts of the product, plus the work that spans them.

Table 15. The standard WBS.

| Code | Element | Holds |
|---|---|---|
| 1 | Project management | the project plan, planning and status, gate reviews, the registers |
| 2 | Systems engineering | the needs and the operating concept, the requirements, the architecture and interfaces, the verification matrix, the design descriptions |
| 3.n | Subsystem n (one per subsystem) | its requirements, design, models or hardware, software, tests and documents |
| 4 | Integration, verification and validation | integration, the test campaigns, verification and validation |
| 5 | Build, release and deployment | code generation and flashing, builds, releases, deployment to the real product |
| 6 | Tools and infrastructure | the dashboard, the builders, the runners, the test infrastructure |

**The WBS dictionary is the set of task cards.** Each card is one work package under one WBS element, and it serves one gate.

### 5.2 The task card

A task card defines one work package so completely that anyone can start it, check it and close it without asking. The cards of a project are kept in one file, the project's cards file (for Shaddad, `tools/dashboard/project_cards.json`; the template is `framework/templates/task_card.json`). They are shown on the dashboard's Project tab and printed as the project's task-card PDF.

Table 16. The fields of a task card.

| Field | Content | Rule |
|---|---|---|
| ID and title | gate prefix and number; a short title | the prefix gives the gate: G1, G2 and G3 cards; B for build, T for test readiness, V for verification, P for the configuration audit |
| WBS | the WBS element | exactly one |
| Gate | the gate the card serves | exactly one |
| Lead and support | the one accountable; who else works on it | one lead |
| Status and priority | see section 5.4; now, next, later, done | kept current |
| Purpose | why the card exists | one sentence |
| Description | what the task is, the work it involves, how it is done and what it delivers; for a done card, what was done and its outcome | three to six sentences, written for someone new to the project |
| Entry gate | the gate or cards before it, then the entry criteria | each criterion checkable: true or false |
| Exit gate | the gate or card it feeds, then the exit criteria | each criterion measurable; together they are the acceptance |
| Predecessors | the cards that shall be done first | successors are derived from them |
| Inputs and outputs | what the card needs; what it delivers | outputs named: files, documents, records |
| Verification | how the exit criteria are checked | Test, Analysis, Inspection or Demonstration |
| Size and effort | S, M, L or XL before; the actual effort after | section 5.6 |
| Risks | each with likelihood, impact and mitigation | section 8.3 |
| Escalation | when, and to whom | section 3.3 |
| Decisions | the owner's open decisions the card needs | by number |
| Links, report, task | review items, documents, the report of a done card, the work log | kept current |

### 5.3 How to write a good card

- **Purpose:** state the reason in the product's terms ("so that a stop on a slope cannot tip the robot"), not the activity ("work on the brakes").
- **Entry criteria** are facts a person can check: "G1-05 passed", "the ESC manual filed". Avoid words like "ready" or "agreed".
- **Exit criteria** are measurable: "0 new test failures", "S2c no longer tips", "the user approves the procedure". The verification line says how each one is checked.
- **Outputs** are named: the file, the document number, the decision record.
- **Risks** are specific to the card, each with a mitigation that is itself work: often a step, sometimes another card.
- **Size it honestly.** If a card is larger than L, split it.

### 5.4 Card states: ready and done

![Figure 4: The states of a task card.](framework/handbook/fig/f3_states.svg)

The states (Figure 4):
- **Proposed:** written by the lead, not yet agreed.
- **Planned:** the owner has agreed it.
- **Ready:** its entry criteria are met.
- **In work:** the lead has dispatched it.
- **In review:** the work is done and its record is in.
- **Done:** the exit criteria are met and the record is complete.

A card that waits on the owner shows **needs you**; one that waits on another card shows **blocked**. A card that is no longer needed is **cancelled**, with the reason recorded.

**The definition of ready.** The entry criteria are met, the predecessors are done, the lead and the support are named, and the brief is written.

**The definition of done:**
1. the exit criteria are met and checked by the stated verification;
2. the outputs exist where the card says;
3. the changes are under configuration control: snapshots, a decision record for each decision, and the test baseline updated;
4. the documents are updated;
5. a task group has its PDF engineering report;
6. the card's status, effort, report and links are updated.

### 5.5 From a request to a card

Every request of the owner is classified before any work starts (Figure 5):
- **A question** is answered; no card.
- **New work** becomes one or more cards. The lead proposes each with its WBS element, gate, lead, size, and entry and exit criteria; the owner approves.
- **A change to a baseline** becomes a change request (section 8.1).
- **A defect or a nonconformance** becomes an issue or a nonconformance report (section 8.4), then a card to fix it.

A small task that takes a few minutes may be done at once, and then recorded as a closed card.

![Figure 5: How a request becomes a card.](framework/handbook/fig/f4_intake.svg)

### 5.6 Size, priority, dependencies and the critical path

- **Size,** by the effort it needs:
  - **S:** one engineer, one pass;
  - **M:** two or three engineers, or two passes;
  - **L:** several engineers and system runs;
  - **XL:** a review of every subsystem, or a change across several safety chains.

  The effort is recorded in engineering passes and engineer-days. The estimate is recorded before the start and the actual after. When the actual passes the estimate, the owner is told at once.
- **Priority:**
  - **now:** in work;
  - **next:** the owner's next choices;
  - **later;**
  - **done.**
- **Dependencies:** a card lists only its predecessors. The successors and the card map are derived from them.
- **The critical path:** the longest chain of open cards, weighted by size, to the end of the project. A delay on it delays the project, so the lead watches it at every planning step.

## 6. Doing the work

### 6.1 The card cycle

Every card runs through the same cycle (Figure 6). Not every card needs every step: a document-only card has no integration check. A step is skipped only with a reason, recorded in the card.

![Figure 6: The work cycle of a card.](framework/handbook/fig/f5_cycle.svg)

### 6.2 Briefs and dispatch

- **The shared brief** of a task group holds what every engineer needs:
  - the request in the owner's words, and the decisions already taken;
  - the rules;
  - who else is working, and which files are theirs.
- **The engineer's brief** holds the change itself:
  - the files and the exact values;
  - the checks to run;
  - the report expected (template: `framework/templates/work_brief.md`).
- **Dispatch:** engineers whose files do not overlap work in parallel. Heavy tools, such as a simulation in batch, run one at a time through the locked runner.

### 6.3 The engineer's pass

1. Snapshot every file before the first edit (or commit on a branch, when the project uses version control).
2. Change the generator, not the generated file: the builder, the parameter file, the source.
3. Rebuild only what changed.
4. Run the checks: structure, interfaces, layout, then the tests of the subsystem, sorted against the test baseline.
5. Write the step report (template: `framework/templates/step_report.md`): status, files changed, checks run, behaviour before and after in numbers, the facts for the docs, the open points.

### 6.4 Checks, tests and the independent review

- **The integration engineer** checks any change that crosses subsystems: the top level, the interfaces, the system scenarios.
- **The test engineer** runs the changed test classes and the fast tier. A known failure that now fails in a new way is reported as new.
- **The independent reviewer** checks the change against the request and the evidence: the diff against the snapshots, the logs and the docs. It does not see the authors' reasoning. It returns its findings ranked, and a verdict:
  - **keep;**
  - **fix:** real breakage, fixed once by the same engineer;
  - **ask:** a question only the owner can answer.

### 6.5 Closing a card and its record

The lead closes the card by the definition of done (section 5.4):
- the documentation owner updates the documents;
- the lead writes the task report and, for a task group, the PDF engineering report (`framework/templates/engineering_report.md`; built with `tools/reports/build_engineering_report.py`);
- the card, the registers, the dashboard and the notes are updated.

### 6.6 Rules of work

- **The owner edits.** Only the subsystem lead edits its files. A change to another owner's file, an interface or a test expectation goes to the lead.
- **One pass.** Fix real breakage once. Do not polish in loops; anything cosmetic is recorded and left.
- **No engineer starts another engineer,** and no engineer runs its own reviewer: the lead dispatches the review.
- **Count the cost** of every run, and tell the owner as soon as the estimate is passed.
- **Never:** change a security setting, act on the real hardware, push, send or publish, or put personal data into an outside request. These are the owner's.

## 7. Engineering processes

### 7.1 Stakeholder needs and the operating concept

- **List the stakeholders** (the owner, the users, the operators, the maintainers, the regulators) and their needs. Each need has a source: a person, a document or a date.
- **Write the operating concept (ConOps):** who uses the product, where, how, and in which scenarios, including the abnormal ones (faults, stops, recovery). The scenarios become the validation tests (section 7.7) and, in a simulation project, the system scenarios.

### 7.2 Requirements

All requirements live in one register per project (template: `framework/templates/requirements_register.yaml`). The specifications are generated from the register, or kept in step with it.

Table 17. The attributes of a requirement.

| Attribute | Content |
|---|---|
| ID | REQ-<domain>-NNN; never reused |
| Text | one "shall" statement |
| Rationale | why it exists |
| Source | the stakeholder need, standard or decision it comes from |
| Parent and allocation | the higher requirement it derives from; the subsystem(s) that meet it |
| Verification method | Test, Analysis, Inspection or Demonstration |
| Verification reference | the test, scenario, analysis or procedure |
| Status | draft, agreed (in the baseline), verified, deviated, deleted |
| Result | pass, fail or open, with the evidence and its date |

**The quality rules** (from ISO/IEC/IEEE 29148). Each requirement is:
- **necessary;**
- **singular:** one "shall";
- **unambiguous:** no "fast", "adequate" or "user-friendly";
- **verifiable:** a number and a method;
- **feasible;**
- **traceable** up and down.

Values not yet known are written **TBD**, with an owner and a date. A safety requirement shall not carry a TBD at G1.

**Traceability:** need → system requirement → subsystem requirement → design element and card → verification → result. The verification matrix is the register's view of the last three columns.

### 7.3 Architecture, interfaces and design

- **The functional architecture:** the functions and their flows. **The physical architecture:** the subsystems and their connections. The requirements are allocated to the subsystems.
- **The interface document (ICD)** is the single source of every signal, bus, message and unit between subsystems. A change to it is a change request.
- **The design description (SDD)** explains how the system and each subsystem work: their purpose, structure, behaviour, modes and faults, interfaces and limits. Its first purpose is understanding. It is written for a new engineer joining the project.
- **Design decisions** go to the decision log with their options and reasons (section 8.2).
- **In a model-based project** the models are design artefacts. They are generated by builder scripts, and the parameters are kept in parameter files.

### 7.4 Implementation

- Models, code and hardware are made to the build-to baseline, by the subsystem lead.
- Generated artefacts are changed only at their generator; every change starts with a snapshot.
- The project plan states the modelling and coding rules: names, layout, numeric parameters, code-generation settings.

### 7.5 Integration

- The integration engineer owns the top level and the interfaces.
- Every change that crosses a subsystem border gets the integration checks: interface consistency, the top-level structure, and the system scenarios it touches.

### 7.6 Verification

Verification shows that the product meets its requirements. Every requirement has one primary method:
- **Test:** measured under controlled conditions;
- **Analysis:** calculated or simulated;
- **Inspection:** examined;
- **Demonstration:** operated and observed.

Table 18. The levels of verification.

| Level | What | Typical evidence |
|---|---|---|
| Unit and bench | one block, function or component | bench tests and their logs |
| Subsystem | one subsystem against its requirements | the subsystem's test classes |
| System | the integrated system in scenarios | system scenario runs and their criteria |
| Processor or hardware in the loop | the real code or hardware with a simulated world | PIL and HIL runs |
| Field | the real product in its environment | test procedures, logs and their analysis |

- **Test procedures and reports.** A formal test follows an approved procedure (`framework/templates/test_procedure.md`), and its result is a test report (`framework/templates/test_report.md`).
- **The test baseline** records every known failure, each with its reason, its signature and its disposition. A run is judged against it: a new failure, or a known one failing in a new way, is reported at once.

### 7.7 Validation

Validation shows that the product meets the owner's real needs in its real use, beyond the written requirements. It is done with the operating-concept scenarios, at the end on the real product. The owner's acceptance is recorded at G5.

### 7.8 Release and transition

A release is a configuration with a version, made from a baseline:
- the generated code and the firmware image;
- the build outputs;
- the software version description, which lists every model, file and package with its version.

Loading a release onto the real product is the owner's action, done to a procedure.

## 8. Managing the engineering

### 8.1 Configuration management and change control

**Configuration items** are the things whose state matters:
- the requirements and the interface document;
- the models, builders, parameters and code;
- the controlled documents;
- the test baseline.

Each has an ID and a version, and each baseline records the versions that make it up.

**Change control** (Figure 7). A change to a baseline goes through these steps:
1. A **change request** (CR-NNN; template `framework/templates/change_request.md`) states what changes, why, and who asks.
2. The owners of the affected parts **analyse its impact:** on requirements, interfaces, tests, cost, schedule and safety.
3. **The change board decides:** approve, reject or defer. The board is the owner, advised by the lead and the affected leads.
4. An approved change becomes **implementation cards.**
5. The change is **verified,** and the baseline is updated with a new version.
6. The **CR is closed.**

![Figure 7: Change control.](framework/handbook/fig/f6_change.svg)

Table 19. The classes of change.

| Class | What | Who decides |
|---|---|---|
| Class 1 | changes a baseline: a requirement, an interface, a safety function, a budget beyond the approved one, or the schedule of a gate | the change board (the owner) |
| Class 2 | stays inside one card's scope and changes no baseline | the lead, recorded in the card |
| Editorial | wording or layout; no technical content | the documentation owner |

**Status accounting:** the dashboard and the registers show, at any time, every configuration item with its version, every open change request, and every baseline. **Audits:** G5 (FCA) checks the function against the requirements; G7 (PCA) checks the as-built item against the documents.

### 8.2 Decisions

- **Every decision that matters is recorded** (template `framework/templates/decision_record.md`):
  - a number (D-NNN), the date, and who decided;
  - the context, the options considered, the decision, the reasons and the consequences.
- **Who decides:** the owner decides requirements, safety, hardware, suppliers, budget, and every gate and change board. The lead decides how work is done inside an approved card. A subsystem lead decides inside its pass and reports it.
- **Putting a decision to the owner:** one question at a time, with the options, the recommended one first, and the consequence of each in plain words. The answer is recorded word for word.

### 8.3 Risks and issues

- **A risk** is an event that may happen and would hurt the project. **An issue** has already happened.
- Both live in the project's register (template `framework/templates/risk_register.json`).

Table 20. Risk rating: likelihood × impact.

| Likelihood ↓ / Impact → | Low (cost or a short delay) | Medium (a result or a gate slips) | High (safety, the product or a baseline) |
|---|---|---|---|
| High (likely, or already seen) | Medium | High | High |
| Medium (possible) | Low | Medium | High |
| Low (unlikely) | Low | Low | Medium |

- **Each risk has:**
  - an ID (RSK-NNN) and an owner;
  - the cause and the effect, and the rating;
  - a response: avoid, mitigate, transfer or accept;
  - the mitigation, as a card or a step, and a date to review it.
- **Every High risk needs a mitigation card.** The register is reviewed at each gate and whenever a card starts.
- A risk that happens becomes an issue, handled as a card, or as a nonconformance when a requirement fails.

### 8.4 Nonconformances

A **nonconformance** is a product that fails a requirement or a verification. It is recorded as NCR-NNN (template `framework/templates/nonconformance_report.md`). Then:
1. **The cause is found.**
2. **The owner decides the disposition:**
   - **fix it** (a card);
   - **use as is** (a waiver, with its risk);
   - **change the requirement** (a change request).
3. **It is closed** when the disposition is done and verified.

A defect found inside a card's own work, before the card is closed, is fixed within the card and recorded in its report.

### 8.5 Documents and records

**Controlled documents** carry:
- a number: NASL-<project code>-<type>-<NNN>, or NASL-EMS-<type>-<NNN> for company-wide documents;
- a version and a date;
- a status: draft, issued for approval, released, or superseded;
- a reviewer and an approver; the names stay **[TBD]** until the owner names them.

The document types are in Appendix A.

**Docs as code.** Every controlled document has one source:
- a markdown file with a header that holds its number, title, version and history (template `framework/templates/controlled_document.md`);
- or a register for the generated ones.

The NASL PDF is generated with `tools/reports/build_document.py`. **A generated document is never edited by hand;** change the source and rebuild with a new version.

**The document index** lists every controlled document with its number, version, status and source (`python tools/reports/build_document.py --index`).

**Records** prove what was done. They are kept, not revised:
- the journal and the work logs;
- the step and task reports;
- the test logs and results;
- the decision records and the gate review reports.

### 8.6 Quality assurance

- **Every change** is checked by an independent reviewer before it is kept (section 6.4).
- **Every gate** has an independent assessment (section 4.5).
- **Every document** follows the writing rules:
  - the result first, then the numbers with units;
  - the owner's words quoted;
  - what is not verified is said, with the command or test that would verify it;
  - estimates are marked [EST], and measured values [DATA];
  - absolute dates;
  - each figure and table numbered and captioned.
- **Lessons learned** are collected at each gate and at the end of each task group (template `framework/templates/lessons_learned.md`). They flow into the next version of this handbook.

### 8.7 Measurement and status

The dashboard shows the project's measures live:
- cards done per gate, and each gate's readiness;
- the critical path;
- tests passing against the baseline, and new failures;
- open risks by rating, and open change requests and nonconformances;
- the effort of each card against its estimate.

The lead reports the status to the owner at the end of each working day: what changed, what is not done or not verified, the cost, and the decisions still open.

## 9. The team and its work rules

A project staffs the roles of Table 2: the project owner, the lead systems engineer, one engineer per subsystem and the specialists. One person may hold several roles, except that the reviewer of a change is never its author.

- **Role descriptions** state each role's scope, the parts it owns and what it must know. They are kept with the team set-up.
- **The rules of chapter 6** apply to everyone:
  - the owner of a part edits it;
  - one pass;
  - snapshots first;
  - heavy runs one at a time;
  - no engineer assigns another engineer or arranges their own review.
- **The step report** ends every pass, in the template `framework/templates/step_report.md`. The lead keeps it word for word as a record.
- **Briefs.** The lead writes a brief for every pass (`framework/templates/work_brief.md`), with a shared brief for the whole task group.
- **Effort.** Every pass is logged with its time. The size of a task group is given to the owner before the start, and the actual effort is reported at the end.
- **Limits.** Only the owner acts on the real hardware, changes security settings, or sends and publishes.

## 10. The tools

The tools turn the sources into the outputs and show the status (Figure 8). **Sources are edited; outputs are generated.**

![Figure 8: The tool chain: sources, generators, outputs and the dashboard.](framework/handbook/fig/f8_tools.svg)

Table 21. The tools of the framework.

| Tool | What it does | Where |
|---|---|---|
| Dashboard | a local web page that reads the project's files and shows the status. Tabs: Overview, Project (gates and cards), Team (who works on what), Subsystems, Journal, Reports, Tests, Models and others. It writes nothing | `tools/dashboard/` |
| Journal and work log | one line per event: work started and done, results, rulings, decisions, issues | `tools/dashboard/log.py` and the work log |
| Task-card builder | the project's task-card PDF from its cards file | `tools/reports/build_task_cards.py` |
| Document builder | any controlled document from its markdown source with a header | `tools/reports/build_document.py` |
| Report builders | the task report (ATR) and the PDF engineering report of a task group | `tools/dashboard/task_report.py`, `tools/reports/build_engineering_report.py` |
| Locked batch runner | one heavy run at a time, with a time cap and a log | the team kit (`run_batch.sh`) |
| Test baseline tool | builds the known-failure baseline and compares a run with it | the team kit (`test_baseline.py`) |
| Team kit | the team playbook, the work rules, the dashboard style, the reports guide | the team kit folder |

## 11. How it works end to end: a worked example

This chapter follows one real task of the Shaddad project through the system: the **downhill speed cap**. Its card is G2-DSC.

1. **The trigger.** The critical-fixes task (card G2-SAF) made every soft stop park on the spring brakes. Its system runs then showed a hazard: in scenario S2c the robot tipped over its front contact in a brakes-locked stop at 1.175 m/s, on a 15° descent, with the sprayer's tank full. This was an **issue** found inside a card. It was recorded in the card's report, and a follow-up card was opened.
2. **The card.**
   - **Purpose:** keep every brakes-locked stop below the tip-over speed.
   - **Lead:** the lead; **support:** the structure, MCU, GNC and integration engineers.
   - **Entry:** G2-SAF done.
   - **Exit:** S2c no longer tips; an E-stop on the descent does not tip; 0 new test failures; a decision record and the engineering report.
   - **Size:** L, estimated at the start and reported against as the work went on.
3. **Decisions, one at a time.** The owner chose the order, "check physics, then cap the speed", then "real loads first, then always on". The owner then answered each question the analysis raised: where the sprayer is mounted, how the mower deck is held, no reversing with the tiller, the deck lowered whenever the robot moves, and keeping the latch after each stop. Each answer went to the journal as a decision.
4. **The work, owner by owner.**
   - **Structure** showed with a planar model that the pitch-over is physical, and computed the safe speed per slope and implement.
   - **The lead** put the table into the parameters, once, so that the MCU and the GNC read it from there.
   - **The MCU engineer** capped the speed in every mode and made the robot park before the cap is passed.
   - **The GNC engineer** made the planner look ahead along the path.
   - Each engineer worked in one pass, with snapshots and a step report, and with 0 new test failures.
5. **Verification found a defect.** The integration engineer's system runs showed a real defect: on the 20° design ramp the robot measures 20.5° to 21°, above the table's last speed column at 20°, so the cap fell to zero and the robot could not descend. This stays inside the card's scope, a class 2 change (section 8.1): the structure engineer re-tabulates the table by the pitch the robot measures, and the system runs are repeated (in work at this issue of the handbook).
6. **Cost control.** When the effort passed the estimate, the lead told the owner, who approved a further budget twice, with the remaining steps listed each time.
7. **Closing.**
   - **The independent reviewer** checks the change against the request: the changed files are listed with their snapshots in a review manifest.
   - **The documentation owner** writes the decision record, D74, and updates the specifications and the status.
   - **The lead** writes engineering report NASL-SHD-ER-005 and sets the card to done, with its report and its actual effort.
   - **The successors,** the PDR cards of structure, MCU, GNC and integration and the slope brake test procedure, now have one input more.

The example shows the system at work:
- a card with clear entry and exit;
- decisions put to the owner one at a time and recorded;
- owners editing their own parts;
- verification that found a real defect before release;
- cost reported against the estimate;
- a complete record at the end.

## 12. Starting a new project with the framework

1. **Copy the kit** into the new project folder: `framework/` (this handbook and the templates), `tools/reports/`, `tools/dashboard/` and the team kit.
2. **Configure the project:** fill `project.json` from `framework/templates/project_config.json` (the project's name, code, entity, document prefix and subsystems).
3. **Write the charter** and the stakeholder list, and **hold G0**.
4. **Write the project plan** (`framework/templates/project_plan.md`):
   - the tailoring of the life cycle;
   - the WBS;
   - the team (who fills each role) and the approval authority;
   - the project's own success criteria, for example the budget margins;
   - the registers.
5. **Create the first cards** under G1, with every field filled. **Set up the registers:** requirements, risks, decisions, changes and nonconformances.
6. **Set up the team** from the team playbook (the roles, the work rules, the briefs), and **start the dashboard**.
7. **Run the work as cards,** through the gates, as this handbook describes.

## Appendix A. Document types and numbers

Controlled documents are numbered NASL-<code>-<type>-<NNN>. <code> is EMS for company-wide documents, or the project code (SHD for Shaddad). The files are named NASL_<type>-<NNN>_<Project>_<Title>_<date>_v<version>.

Table 22. Document types.

| Type | Document | When |
|---|---|---|
| HBK | handbook (this document) | framework |
| PMP | project management plan: tailoring, WBS, team, success criteria, registers | G0 |
| CHR | project charter | G0 |
| CON | concept of operations | G0, G1 |
| REQ | requirements specification (generated from the register) | G1, G2 |
| ICD | interface control document | G1 to G3 |
| SDD | system or subsystem design description: how it works | G1 to G3 |
| ANR | analysis report | G2, G3 |
| PLN | plans: the task cards (WBS dictionary), the verification and validation plan, the campaign plans | G0 to G4 |
| TP | test procedure | G4 |
| TR | test report | G5 |
| GRR | gate review report, with the decision | each gate |
| ER | engineering report of a task group | each task group |
| CR | change request | as needed |
| NCR | nonconformance report | as needed |
| DEC | decision log (generated from the decision records) | ongoing |
| RSK | risk register (generated) | ongoing |
| SVD | software version description | G5, G7 |
| LLR | lessons learned report | each gate, end |
| MAN | manual (user or operator) | G5, G7 |

Register IDs:
- requirements REQ-<domain>-NNN;
- risks RSK-NNN;
- decisions D-NNN;
- changes CR-NNN;
- nonconformances NCR-NNN;
- review items RID-<gate>-NN;
- cards <gate prefix>-<NN or code>.

## Appendix B. Templates

Table 23. The templates in framework/templates.

| Template | Use |
|---|---|
| `project_config.json` | the project's name, code, entity, document prefix and subsystems |
| `project_plan.md` | the project management plan (PMP) |
| `controlled_document.md` | the header and skeleton of any controlled document |
| `task_card.json` | one task card with every field |
| `gate_success_criteria.json` | the standard success criteria of every gate (section 4.4) |
| `gate_readiness_checklist.md` | the readiness check of a gate |
| `gate_review_report.md` | the gate review report, with the RIDs and the decision |
| `change_request.md` | a change request |
| `decision_record.md` | a decision record |
| `nonconformance_report.md` | a nonconformance report |
| `risk_register.json` | the risk register with one example entry |
| `requirements_register.yaml` | the requirements register and verification matrix with one example entry |
| `test_procedure.md` / `test_report.md` | a formal test procedure and its report |
| `engineering_report.md` | the eleven sections of a task-group engineering report |
| `work_brief.md` / `step_report.md` | an engineer's brief and step report |
| `lessons_learned.md` | lessons learned at a gate or at the end of a task group |

## Appendix C. Mapping to ISO/IEC/IEEE 15288:2023

Table 24. The processes of the standard and where this handbook covers them.

| 15288 process | Covered in |
|---|---|
| Life cycle model management | chapter 4; section 4.3 (tailoring) |
| Project planning | chapter 5 (WBS, cards, sizes, critical path) |
| Project assessment and control | sections 4.5 (gates), 8.7 (measurement and status) |
| Decision management | section 8.2 |
| Risk management | section 8.3 |
| Configuration management | section 8.1 |
| Information management | section 8.5 |
| Measurement | section 8.7 |
| Quality assurance | sections 6.4, 8.6 |
| Knowledge management | section 8.6 (lessons learned); chapter 11 |
| Human resource and infrastructure management | chapters 3, 9, 10 |
| Business or mission analysis | section 7.1; gate G0 |
| Stakeholder needs and requirements definition | section 7.1 |
| System requirements definition | section 7.2; gate G1 |
| System architecture definition, design definition | section 7.3; gates G2, G3 |
| System analysis | sections 7.3, 7.6 (analysis as a method) |
| Implementation | section 7.4 |
| Integration | section 7.5 |
| Verification | section 7.6; gates G4, G5 |
| Transition | section 7.8 |
| Validation | section 7.7; gate G5 |
| Operation, maintenance, disposal | outside this version; the project plan covers them when needed |
| Acquisition and supply | outside this version (contracts) |

## Appendix D. Glossary and abbreviations

Table 25. Terms and abbreviations.

| Term | Meaning |
|---|---|
| Baseline | an agreed state of a set of items, changed only through change control |
| Card (task card) | one work package: the unit of planned, done and recorded work |
| CCB | change control board: the owner, advised by the lead |
| CDR / PDR | critical / preliminary design review |
| ConOps | concept of operations |
| FCA / PCA | functional / physical configuration audit |
| Gate | the formal review that ends a phase, passed on success criteria and the owner's decision |
| ICD | interface control document |
| MCR | mission concept review |
| NCR | nonconformance report |
| RID | review item discrepancy: a finding of a gate's independent assessment |
| SDD | system or subsystem design description |
| SRR / SFR | system requirements / system functional review |
| SVD | software version description |
| SVR | system verification review |
| TBD | to be determined: a value not yet known, with an owner and a date |
| TRR | test readiness review |
| V&V | verification and validation |
| WBS | work breakdown structure |

## References

- ISO/IEC/IEEE 15288:2023, Systems and software engineering: system life cycle processes.
- IEEE 15288.2-2014, Standard for technical reviews and audits on defense programs.
- MIL-STD-1521B, Technical reviews and audits for systems, equipments and computer software.
- NASA SP-2016-6105 Rev 2, NASA Systems Engineering Handbook.
- ISO/IEC/IEEE 29148:2018, Requirements engineering.
- ISO 10007:2017, Quality management: guidelines for configuration management.
- ISO 31000:2018, Risk management: guidelines.
- The team playbook and its guides (the reports guide, the dashboard style), in the team kit.
