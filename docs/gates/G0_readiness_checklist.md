<!--
doc_no: NASL-SHD-GRR-001-RC
doc_type: GATE READINESS CHECKLIST
title: Shaddad UGV: readiness check for gate G0 (MCR)
subtitle: Held retrospectively: the project started in October 2026, before the engineering management system
version: 1.1
date: 2026-11-16
status: Approved
history: 1.0 | 2026-11-15 | Readiness check (card G0-02)
history: 1.1 | 2026-11-16 | The delta review: the findings of the independent assessment answered; the delta re-review and its findings answered (section 6)
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV G0 readiness
file: NASL_GRR-001-RC_Shaddad_G0_Readiness_v1.1
outdir: work/reports/gates
related: NASL-EMS-HBK-001 sections 4.4 and 4.5; NASL-SHD-PMP-001; tools/dashboard/project_cards.json (gate G0)
-->
# Readiness check: gate G0, mission concept review

The project owner chose on 2026-11-15 to hold G0 now ("Yes, hold G0 now (Recommended)"). The lead scored each success criterion on its evidence (NASL-EMS-HBK-001 section 4.5, step 1). The independent assessment and the owner's decision follow.

## 1. The cards under the gate

Table 1. The cards of G0.

| Card | Title | Status | Record |
|---|---|---|---|
| G0-00 | Project start: scope, operating concept and the model build plan | done (heritage) | docs/architecture.md; docs/build_plan.md |
| H-05 | Engineering team set-up, dashboard and work rules | done (heritage) | the team playbook and work rules; the project dashboard (tools/dashboard/) |
| G0-01 | Engineering management system: the framework and the project plan | in review (the owner reviews the handbook and the plan at the board) | NASL-EMS-HBK-001; NASL-SHD-PMP-001; framework/templates |
| G0-02 | Hold the G0 review (retrospective) | in progress | this checklist; the gate review report to follow |

## 2. The success criteria

Table 2. The G0 criteria scored on 2026-11-16.

| ID | Criterion | Class | Evidence | Measured | Threshold | Score |
|---|---|---|---|---|---|---|
| G0-SC1 | The need, the goals and the constraints are written | M | the charter NASL-SHD-CHR-001 (need, measurable goals, constraints, scope); the Phase 1 report NASL-SHD-ANR-003 | written | approved by the owner | met when the owner approves the charter at the board |
| G0-SC2 | The stakeholders and their needs are listed | M | NASL-SHD-PMP-001 v1.2 Table 1 | 7 stakeholders (maintainers and regulators added), each need with a first-hand source | each need has a source | met |
| G0-SC3 | An operating concept describes how the product is used | M | the charter section 6 (this phase: mowing; places, modes, normal and abnormal use) and section 5 (the user roles); docs/architecture.md; scenarios S0-S8 | the main use of this phase (mowing) with its scenarios; tillage and spraying moved to a later phase by the owner | scenarios for the main uses | met |
| G0-SC4 | The life cycle is tailored, the WBS and the first cards exist | M | NASL-SHD-PMP-001 sections 2 and 3; tools/dashboard/project_cards.json | 56 cards, each with a WBS code; the G1 cards with entry and exit criteria | every phase-1 card has entry and exit criteria | met |
| G0-SC5 | The top risks are known | D | docs/registers/risks.json | 57 risks (26 High) and 3 issues, each with an owner and a card | each High risk has an owner | met |

**Readiness:** SC2, SC3 and SC4 met; SC1 met on the owner's approval of the charter at the board; the desirable criterion SC5 met.

## 3. The open risks and findings

- **High risks and issues open:** 26 High risks and 3 issues, each with an owner and a card (docs/registers/risks.json). They are the work of G1 to G3. The descent envelope they include is settled by CR-001 and stated in the charter.
- **Carried findings:** the 87 review items of 2026-11-15, each on a card (NASL-SHD-PLN-001 Appendix A).

## 4. The lead's recommendation

Hold the review now. The evidence predates the framework (the project started in October 2026), so the independent assessment should check that each criterion is shown by the evidence and not only asserted.

## 5. The delta review: the findings answered

The owner decided a delta review on 2026-11-16 ("Delta review (Recommended)") and chose the main uses of this phase ("Mowing now, the rest later (Recommended)").

Table 3. The findings of the independent assessment and how they are answered.

| RID | Severity | Answer | Evidence |
|---|---|---|---|
| RID-G0-01 | Critical | The charter is written. The owner approves it at the delta board. | NASL-SHD-CHR-001 |
| RID-G0-02 | Critical | Maintainers and regulators (ISO 18497, ISO 13849, ISO 5395) added. Every need has a first-hand source: the owner's dated directions, the Phase 1 operations table, the manual, the procedures, the standards. | NASL-SHD-PMP-001 v1.2 Table 1 |
| RID-G0-03 | Critical | The owner set the main uses of this phase: mowing now, tillage and spraying in a later phase as their own cards; solar farms out until a site is named. The user roles are added. | the charter sections 4 to 6; cards G2-TIL and G2-SPR (proposed) |
| RID-G0-04 | Major | The charter states the as-built robot and the CR-001 envelope. It supersedes the scope section of the architecture until G1-01. | the charter sections 3 and 6 |
| RID-G0-05 | Major | G1-01 now produces the trace from needs, the quality check and the allocation. G1-05 offers the four outcomes. | the cards file (G1-01, G1-05) |
| RID-G0-06 | Minor | SHD-SC1 applies from G1 onward (G0 locks no model). | NASL-SHD-PMP-001 v1.2 Table 4 |
| RID-G0-07 | Minor | G0-01 is in review until the owner reviews the handbook and the plan at the board. | the cards file (G0-01) |
| RID-G0-08 | Minor | The three risks that happened are issues now. Cause, effect and a review date for every risk and issue come at the register review, an exit criterion of card G1-01, due by G1-05. | docs/registers/risks.json (RSK-008, RSK-017, RSK-022) |
| RID-G0-09 | Minor | The plan no longer repeats the card counts; it points to the task cards. The cards cite the handbook in place of docs/lifecycle.md, and the plan puts the closure of G2-DSC in the future. | NASL-SHD-PMP-001 v1.2 |

**Readiness after the delta:**
- **Met:** SC2 to SC5.
- **SC1:** met on the owner's approval of the charter at the board.
- **The lead's recommendation:** the independent reviewer re-reviews SC1 to SC3, then the owner decides.

## 6. The delta re-review: the findings answered

The independent reviewer re-reviewed SC1 to SC3 on 2026-11-16 (docs/gates/G0_delta_rereview.md):
- **SC2 and SC3:** met.
- **SC1:** met once the owner's approval of the charter is recorded.
- **Closed:** RID-G0-02 to RID-G0-06.
- **Recommended outcome:** go with actions, once the owner approves the charter at the board.

Table 4. The open and new findings of the delta re-review and how they are answered.

| RID | Severity | Answer | Evidence |
|---|---|---|---|
| RID-G0-01 | Critical until the board | The charter is in the document register. The owner approves it at the board. | docs/registers/documents.json (NASL-SHD-CHR-001) |
| RID-G0-07 | Minor | Closes when the owner's review of the handbook and the plan is recorded at the board. | the cards file (G0-01) |
| RID-G0-08 | Minor | The register review is an exit criterion of card G1-01, due by G1-05. | the cards file (G1-01) |
| RID-G0-09 | Minor | The cards cite the handbook in place of docs/lifecycle.md; the plan puts the closure of G2-DSC in the future. | the cards file; NASL-SHD-PMP-001 v1.2 |
| RID-G0-10 | Minor | This checklist corrected: G0-01 in review, 7 stakeholders, 56 cards, 57 risks (26 High) and 3 issues, the readiness line, the date of Table 2. | this checklist, sections 1 to 3 |
| RID-G0-11 | Minor | The charter registered; the scope section of the architecture marked as superseded, there and in the register. The scope decision gets its decision row at the board. | docs/registers/documents.json; docs/architecture.md section 1 |
| RID-G0-12 | Minor | The charter v1.1: goal 4 with at least 10 % margin; the scenario coverage case by case, with the two uncovered cases on cards G2-INT and T-03; the tiller record cited, and its risk on card G2-TIL. Card G1-01 brings architecture section 12 up to date. | NASL-SHD-CHR-001 v1.1; the cards file (G2-TIL, G1-01) |
| RID-G0-13 | Minor | The risk register names roles only; the plan names the reviewer and the approver by role. | docs/registers/risks.json; NASL-SHD-PMP-001 v1.2 |

**Readiness for the board:**
- **Met:** SC2 to SC5.
- **SC1:** met when the owner approves the charter.
- **Open findings:** RID-G0-01 and RID-G0-07 close at the board. RID-G0-08 and the section 12 part of RID-G0-12 stay open on card G1-01, due before G1-05. The others are answered in this issue; the gate review report records their closure.
- **The lead's recommendation:** go with actions.
