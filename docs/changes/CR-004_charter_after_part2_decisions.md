<!--
doc_no: NASL-SHD-CR-004
doc_type: CHANGE REQUEST
title: Shaddad UGV: CR-004 the charter after the part-2 decisions
subtitle: Class 1 change: the planned 1 kW motors (D95) and the mission energy (D101) in the charter
summary: The project owner kept the proposed 1 kW traction motor as the twin's default, as a planned upgrade of the robot's four 750 W motors (D95), and replaced the fixed shift by a mowing mission on one charge with a defined margin (D101). The charter, locked at G0, still lists four 750 W motors as a constraint, measures goal 1 on the robot as built, and marks goal 4's margin as open on decision 16. This change brings the charter in line with the two decisions.
version: 1.1
date: 2026-11-18
status: Approved
history: 1.0 | 2026-11-18 | Raised from card G1-01 part 2, after the owner's answers on review decisions 6 and 16
history: 1.1 | 2026-11-18 | Decision of the change board: option 1
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner (the change board)
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV CR-004
file: NASL_CR-004_Shaddad_Charter_After_Part2_Decisions_v1.1
outdir: work/reports/changes
request: Card G1-01 part 2 and card G1-03; the owner's answers of 2026-11-18 on review decisions 6 ("Keep the 1 kW default", with "Planned upgrade to 1 kW") and 16 ("Per mission, per implement (Recommended)")
related: NASL-SHD-CHR-001 v1.3 (section 3, goals 1 and 4); NASL-SHD-PMP-001 (SHD-SC3); decision rows D95 and D101; docs/requirements/system.yaml (N-16)
-->
# CR-004: the charter after the part-2 decisions

**Class 1:** it changes the charter, a baseline locked at G0 (NASL-EMS-HBK-001 Table 19). **Status:** approved on 2026-11-18 (option 1); implemented in the charter v1.4.

## 1. The change

- **What:**
  - **The constraint in section 3,** which reads today: "four 750 W BLDC motors with their ESCs;".
  - **Goal 1's measure** (Table 1): "every parameter traced to the as-built record or a recorded estimate; the model-against-robot audit with no unexplained difference".
  - **Goal 4's measure** (Table 1): "the greenhouse and palm-grove mowing scenarios run to their end within the energy of one charge, with at least 10 % margin [EST; decision 16; SHD-SC3]".
- **Why:**
  - **D95** (the owner, 2026-11-18, review decision 6): the model's default traction motor stays the 1 kW unit, as a planned upgrade of the robot's four Movematic 750 W motors. The baseline values are the 1 kW unit's, and the 750 W stays selectable. No firmware from the twin goes on the robot until its torque limits match the motors fitted. As written, the charter's constraint and goal 1 describe the 750 W robot only.
  - **D101** (the owner, 2026-11-18, review decision 16): no fixed shift. Each mowing mission finishes on one charge with at least 10 % of the usable energy above the 0.20 reserve, at the pack's end of life, from SOC 0.95. Goal 4 still marks its margin as open on decision 16.
- **Raised by:** the lead systems engineer, on 2026-11-18, from card G1-01 part 2.

## 2. Impact analysis

Table 1. The impact.

| Area | Impact | By |
|---|---|---|
| Requirements | The charter to v1.4: section 3, goals 1 and 4. Need N-16 in the register gets the same words as goal 1, at the merge of G1-01 part 2 (step 13). The plan's SHD-SC3 follows goal 4. | the lead systems engineer; the integration engineer |
| Design and models | None now. The default stays the 1 kW unit, as it is today. Every result records its motor. | the drive engineer |
| Tests | None now. | |
| Safety | The flashing condition of D95 becomes a constraint of the charter: firmware from the twin goes on the robot only with torque limits that match the motors fitted. This keeps the 19.1 N m limit of the 1 kW unit off the 750 W motors, whose estimated peak is 14.32 N m per side. | the functional safety engineer, on cards G2-MCU and B-00 |
| Cost and schedule | Two document revisions (the charter and the plan). The purchase of the 1 kW units is the owner's and the supplier's, outside the scope of this phase (charter Table 2). | the project owner |

## 3. Options

1. **Amend the charter to both decisions (recommended).**
   - **Section 3, the constraint:** "four 750 W BLDC motors with their ESCs, as delivered; the owner plans to replace them with four 1 kW units (D95), so the twin's baseline is the 1 kW unit, and firmware from the twin goes on the robot only with torque limits that match the motors fitted;".
   - **Goal 1's measure:** "every parameter traced to the as-built record, the owner's decided upgrade (D95) or a recorded estimate; the model-against-robot audit with no unexplained difference".
   - **Goal 4's measure:** "each greenhouse and palm-grove mowing mission (phase 2) runs to its end on one charge, with at least 10 % of the usable energy above the 0.20 reserve, at the pack's end of life, from SOC 0.95 [EST until the phase-2 mowing energy run; D101; SHD-SC3]".
   - *Consequence:* the charter states the decisions as they stand. Until the 1 kW units are fitted, the model-against-robot audit must be run with the 750 W option.
2. **Amend the motor only:** the constraint and goal 1 as in option 1, and goal 4 later, when the phase-2 run gives its numbers.
   - *Consequence:* goal 4 keeps pointing at a decision that is settled.
3. **No change:** the charter keeps the 750 W constraint and goal 1 on the robot as built.
   - *Consequence:* the twin's default contradicts the charter until the upgrade is fitted.

## 4. Decision of the change board

**Approved: option 1.** The project owner decided on 2026-11-18, as the change board, in the owner's words: "Option 1: both (Recommended)".
- The charter's constraint and goal 1 name the planned upgrade to 1 kW units (D95), with the flashing condition.
- Goal 4 is measured by the mission energy of D101. The charter's section 6 records that S3 is re-scoped under D101 (found by the documentation checklist of D102).
- The plan's SHD-SC3 follows goal 4, in the plan's next issue together with the standards row of G1-01 part 2; need N-16 follows goal 1 at the register merge of G1-01 part 2 (step 13).
- Recorded: the change register, the document register, the journal.

## 5. Implementation and closure (on approval)

- **Documents:** the charter NASL-SHD-CHR-001 v1.4; the plan NASL-SHD-PMP-001 (SHD-SC3), where goal 4 changes; need N-16 in the register (G1-01 part 2, step 13).
- **Records:** the change register, the journal; decision rows D95 and D101 already record the decisions.
- **Closure:** when the charter and the plan are issued.
