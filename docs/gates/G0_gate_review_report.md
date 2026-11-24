<!--
doc_no: NASL-SHD-GRR-001
doc_type: GATE REVIEW REPORT
title: Shaddad UGV: gate G0 mission concept review
subtitle: The system, held retrospectively with a delta review
summary: The project owner passed gate G0 with actions on 2026-11-16. The charter NASL-SHD-CHR-001 v1.1 is approved and locked as the baseline of the scope, and the handbook and the project plan are accepted. All five success criteria are met. Two Minor findings stay open on card G1-01, due before the G1-05 review: the risk register review and the update of the architecture's scenario section.
version: 1.0
date: 2026-11-16
status: Released
history: 1.0 | 2026-11-16 | The gate decision
prepared_by: the lead systems engineer (chair)
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV G0 gate review
file: NASL_GRR-001_Shaddad_G0_MCR_v1.0
outdir: work/reports/gates
chapter_breaks: no
cover_image: shaddad
request: Card G0-02; the owner's decisions of 2026-11-15 ("Yes, hold G0 now (Recommended)") and 2026-11-16 ("Delta review (Recommended)", then the board)
related: NASL-SHD-GRR-001-RC v1.1 (the readiness checklist); docs/gates/G0_independent_assessment.md; docs/gates/G0_delta_rereview.md; NASL-SHD-CHR-001; NASL-SHD-PMP-001; NASL-EMS-HBK-001
-->
# Gate G0 review

## Summary

**Decision: Go with actions**, by the project owner on 2026-11-16, in the owner's words: "Go with actions (Recommended)".

- **The charter** NASL-SHD-CHR-001 v1.1 is approved ("Approve the charter (Recommended)") and locked as the baseline of the scope.
- **The handbook** NASL-EMS-HBK-001 v1.1 and **the project plan** NASL-SHD-PMP-001 v1.2 are accepted as reviewed ("Accept both (Recommended)").
- **Readiness:** 4 of 4 mandatory criteria met, and the desirable criterion met.
- **Open:** two Minor findings on card G1-01, due before the G1-05 review:
  - the risk register review (RID-G0-08);
  - the update of the architecture's scenario section (the open part of RID-G0-12).
- **What starts now:** phase 1, the requirements baseline (gate G1), starting with the re-baseline of the system requirements on the as-built robot (card G1-01). The descent card G2-DSC goes on under CR-001.

## 1. Scope and the board

- **Scope:** the system. The cards under the gate:
  - G0-00 and H-05 (heritage);
  - G0-01, the engineering management system;
  - G0-02, this review.
- **How the review ran.** It was held retrospectively: the project started in October 2026, before the engineering management system.
  1. The readiness check, 2026-11-15, by the lead.
  2. The independent assessment, 2026-11-15 to 16: SC1 to SC3 partly met; 9 RIDs (3 Critical, 2 Major, 4 Minor); a delta review recommended.
  3. The owner's decision, 2026-11-16: "Delta review (Recommended)", with the main uses of this phase: "Mowing now, the rest later (Recommended)".
  4. The answers: the charter; the stakeholders with first-hand sources; the scope of this phase; the minor findings.
  5. The delta re-review, 2026-11-16: SC2 and SC3 met; SC1 met once the owner approves the charter; four new Minor RIDs; go with actions recommended.
  6. The corrections of the re-review, then the board, 2026-11-16.
- **The board:**
  - the project owner (approver);
  - the lead systems engineer (chair);
  - the independent reviewer (assessor).

## 2. The readiness check

Table 1. The success criteria and their scores at the board.

| ID | Criterion | Class | Evidence | Score |
|---|---|---|---|---|
| G0-SC1 | The need, the goals and the constraints are written | M | the charter NASL-SHD-CHR-001 v1.1, approved at the board | met |
| G0-SC2 | The stakeholders and their needs are listed | M | NASL-SHD-PMP-001 v1.2 Table 1: 7 stakeholders, each need with a first-hand source | met |
| G0-SC3 | An operating concept describes how the product is used | M | the charter sections 5 and 6: mowing in this phase, with its places, modes, normal and abnormal use and their scenarios; tillage and spraying moved to a later phase by the owner | met |
| G0-SC4 | The life cycle is tailored, the WBS and the first cards exist | M | NASL-SHD-PMP-001 sections 2 and 3; 56 cards, each with a WBS code; the G1 cards with entry and exit criteria | met |
| G0-SC5 | The top risks are known | D | docs/registers/risks.json: 57 risks (26 High) and 3 issues, each with an owner and a card | met |

The readiness checklist NASL-SHD-GRR-001-RC v1.1 gives the evidence in full, with the answer to every RID.

## 3. The independent assessment: review items (RIDs)

Table 2. The RIDs of the assessment and of the delta re-review.

| RID | Criterion | Finding | Severity | Action | Owner | Due | Card | Status |
|---|---|---|---|---|---|---|---|---|
| RID-G0-01 | SC1 | No charter, and no recorded owner approval of any document | Critical | The charter NASL-SHD-CHR-001, approved at the board | lead systems engineer | 2026-11-16 | G0-02 | closed |
| RID-G0-02 | SC2 | Needs sourced to the project's own outputs; maintainers and regulators missing | Critical | Maintainers and regulators added; every need with a first-hand source | lead systems engineer | 2026-11-16 | G0-01 | closed |
| RID-G0-03 | SC3 | No scenario for tillage or spraying; no solar-farm world; no user roles | Critical | The owner's scope decision (mowing now); the user roles in the charter | lead systems engineer | 2026-11-16 | G2-TIL, G2-SPR | closed |
| RID-G0-04 | SC1, SC3 | The scope predates the as-built robot and CR-001 | Major | The charter states both; the scope section of the architecture is superseded | lead systems engineer | 2026-11-16 | G0-02 | closed |
| RID-G0-05 | SC4 | No G1 exit criterion for the trace, the quality check or the allocation; no no-go at G1-05 | Major | The exit criteria of G1-01; the four outcomes at G1-05 | lead systems engineer | 2026-11-16 | G1-01, G1-05 | closed |
| RID-G0-06 | SHD-SC1 | The project's own criterion is not scored | Minor | SHD-SC1 applies from G1 onward | lead systems engineer | 2026-11-16 | G0-01 | closed |
| RID-G0-07 | SC4 | G0-01 shown done before the owner's review | Minor | The owner's review at the board | project owner | 2026-11-16 | G0-01 | closed |
| RID-G0-08 | SC5 | Cause, effect and review dates missing from the risk register | Minor | The register review, an exit criterion of G1-01 | lead systems engineer | before G1-05 | G1-01 | open |
| RID-G0-09 | SC4 | Stale references: the superseded life-cycle note; G2-DSC said to be closed | Minor | The cards cite the handbook; the plan v1.2 | lead systems engineer | 2026-11-16 | G0-01 | closed |
| RID-G0-10 | the records | The readiness checklist out of date in six places | Minor | The checklist v1.1 corrected | lead systems engineer | 2026-11-16 | G0-02 | closed |
| RID-G0-11 | SC1 | The charter not registered; the superseded scope not marked; no decision row for the scope | Minor | The charter registered; architecture section 1 marked; decision row D76 | documentation owner | 2026-11-16 | G0-02 | closed |
| RID-G0-12 | SC1, SC3 | Three points in the charter (the margin, the scenario coverage, the tiller record); architecture section 12 out of date | Minor | The charter v1.1; section 12 by card G1-01 | lead systems engineer | before G1-05 | G1-01 | open (section 12 only) |
| RID-G0-13 | the records | The roles-only decision not yet applied to the risk register and the plan | Minor | Both converted | lead systems engineer | 2026-11-16 | G0-01 | closed |

The board closed RID-G0-09 to RID-G0-13 on the lead's answers (the checklist, Table 4), without a third review round. The independent reviewer may sample them at the G1-05 review.

## 4. The decision

Table 3. The decision rule applied (NASL-EMS-HBK-001 Table 14).

| Rule | Met? |
|---|---|
| Every mandatory criterion met or waived | yes: SC1 to SC4 met |
| No Critical RID open | yes |
| No Major RID open (go), or each on a card with a date (go with actions) | yes: none open |
| No open item touches safety | yes: the two open items are records (the risk register and the scenario section); the charter's coverage line was corrected before the approval, as the independent reviewer advised |

- **Why go with actions:** the rule would also allow a plain go, since no Critical or Major RID is open. Go with actions binds the two open Minor RIDs to card G1-01, with a date before G1-05.
- **Waivers granted:** none.
- **The owner's words at the board, 2026-11-16:**
  - on the charter: "Approve the charter (Recommended)";
  - on the handbook and the plan: "Accept both (Recommended)";
  - on the outcome: "Go with actions (Recommended)".
- **Decision rows:** D76 (the scope of this phase, with the charter as its baseline) and D77 (this gate decision).

## 5. The baseline

- **Locked:** the concept baseline, held by the charter.
- **Its documents:**
  - NASL-SHD-CHR-001 v1.1, the project charter (approved);
  - NASL-SHD-PMP-001 v1.2, the project plan (approved);
  - NASL-EMS-HBK-001 v1.1, the Engineering Handbook (approved);
  - NASL-SHD-PLN-001 v1.3, the task cards;
  - the registers of 2026-11-16: risks, decisions, changes and documents.
- **Its models and code:** none locked at G0. The models are baselined from G1 onward (SHD-SC1).
- **Its test baseline:** none at G0.

## 6. Lessons learned

- **Keep:**
  - the assessment against the evidence, not the claims: it found three Critical gaps in criteria that the lead's first scoring had marked as met;
  - the delta review on the failed criteria only, which kept the second round short.
- **Change:**
  - write the charter before the first readiness check: a retrospective G0 had no charter to point to;
  - take the counts in the readiness checklist from the registers at each issue: they went stale within a day (RID-G0-10);
  - apply an owner's decision to every record in the same issue (RID-G0-13).
- **For the next version of the handbook:** the G0 readiness check counts the stakeholders, cards and risks from the registers, and asks for the charter before the check starts.
