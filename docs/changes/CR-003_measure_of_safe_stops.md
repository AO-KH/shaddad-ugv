<!--
doc_no: NASL-SHD-CR-003
doc_type: CHANGE REQUEST
title: Shaddad UGV: CR-003 the measure of safe stops
subtitle: Class 1 change: goal 3 of the charter and the plan's criterion SHD-SC2, stated by the park criterion
summary: The charter's goal 3 and the plan's criterion SHD-SC2 measure safe stops as "every brakes-locked stop starts at or below the descent cap". The descent card's system runs show a weaker but physically sound property: every brakes-locked stop meets the park criterion of the structure spec, but two sprayer stops start above the cap of their pitch class, because the step cap falls below the speed the robot already has when the pitch crosses a breakpoint. The owner, as the change board, decides whether the measure is restated by the park criterion or the design is changed to meet the measure as written.
version: 1.1
date: 2026-11-16
status: Approved
history: 1.0 | 2026-11-16 | Raised from card G2-DSC, after the independent review (RID-DSC-01)
history: 1.1 | 2026-11-16 | Decision of the change board: option 1
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner (the change board)
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV CR-003
file: NASL_CR-003_Shaddad_Measure_Of_Safe_Stops_v1.1
outdir: work/reports/changes
request: Card G2-DSC; the independent review of the card (RID-DSC-01) and the docs keeper's finding that the same claim stands in the charter and the plan
related: NASL-SHD-CHR-001 v1.2 (goal 3); NASL-SHD-PMP-001 v1.3 (SHD-SC2); docs/specs/structure.md section 2.11 (the park criterion); NASL-SHD-CR-001; decision row D74
-->
# CR-003: the measure of safe stops

**Class 1:** it changes the measure of a goal of the charter and a mandatory criterion of the plan, both baselines locked at G0 (NASL-EMS-HBK-001 Table 19). **Status:** approved on 2026-11-16 (option 1); implemented in the charter v1.3 and the plan v1.4.

## 1. The change

- **What:**
  - **Goal 3 of the charter** (Table 1), whose measure today reads: "in every system scenario, every brakes-locked stop starts at or below the descent cap; every person stop holds; no tip-over".
  - **The plan's criterion SHD-SC2** (Table 4), mandatory at G2 and G3: "every brakes-locked stop starts at or below the descent cap of its slope and implement (P.structure.descent)".
- **Why:** card G2-DSC built the always-on descent cap and verified it in the system runs.
  - **What the runs show:** every brakes-locked stop meets the park criterion of the structure spec (section 2.11). A stop starts within the safe speed of its held pitch and implement, V_lin(held pitch) + Δv, and keeps the pitch bound (b). No run tips over.
  - **Where the measure as written fails:** two sprayer stops in S2c, with regeneration blocked, start above the cap of their pitch class. When the pitch crosses a breakpoint, the step cap falls below a speed the robot already has, and with no regeneration the robot cannot slow first. The tightest, S2c park 2, starts at 0.787 m/s at a held 15.2°, against a cap of 0.25 m/s in that class. It still meets the park criterion, with margins of 0.056 m/s and 2.1°.
  - **The cap and the criterion are different things.** The cap is a control target, set below the safe speed with a margin. The park criterion is the safety property: the stop does not tip the robot over.
- **Raised by:** the lead systems engineer, on 2026-11-16. Source: the independent review of G2-DSC (RID-DSC-01, which corrected the same claim in D74, the structure spec and STATUS), and the docs keeper's finding that the charter and the plan carry it too.

## 2. Impact analysis

Table 1. The impact.

| Area | Impact | By |
|---|---|---|
| Requirements | Goal 3's measure and SHD-SC2: the charter to v1.3, the plan to v1.4 | the lead systems engineer |
| Design and models | Option 1: none now. Option 2: the MCU slows the robot before a breakpoint crossing, or interpolates the cap between breakpoints, so no stop starts above its class cap; this needs regeneration or a look-ahead the RC mode does not have | the MCU firmware engineer |
| Tests | None: the S2c and S2d criteria already use the park criterion (`utilities/scn_descent_parks.m`) | the integration engineer |
| Safety | Option 1 states the property the physics rests on; the margin at the tightest stop is 0.056 m/s and 2.1°. The safety goal SG-04 (stability, PL d) is unchanged; its function is assessed in the safety concept (G2-FSC) | the functional safety engineer |
| Cost and schedule | Option 1: two document revisions. Option 2: an MCU design task on card G2-MCU, and G2 and G3 cannot meet SHD-SC2 until it is done | the lead systems engineer |

## 3. Options

1. **State the measure by the park criterion (recommended).**
   - **Goal 3:** "in every system scenario, every brakes-locked stop meets the park criterion of the structure spec (it starts within the safe speed of its held pitch and implement and keeps the pitch bound); every person stop holds; no tip-over".
   - **SHD-SC2:** the same, with the threshold "in every system scenario".
   - **A desirable improvement goes on card G2-MCU:** the cap smoothed at breakpoint crossings, to win back margin.
   - *Consequence:* the measure says what the physics needs, and the descent card can close. The margin at the tightest stop is small, so it stays visible on G2-MCU.
2. **Keep the measure as written, and change the design.**
   - G2-MCU must make every stop start at or below its class cap.
   - *Consequence:* G2 and G3 cannot meet SHD-SC2 until then, and the design work is open-ended without regeneration.
3. **Both:** state the measure by the park criterion, and make the smoothed cap a mandatory exit criterion of G2-MCU.
   - *Consequence:* the measure is right now, and the margin must improve before G2 passes for the MCU.

## 4. Decision of the change board

**Approved: option 1.** The project owner decided on 2026-11-16, as the change board, in the owner's words: "Restate by the safe-stop rule (Recommended)".
- Goal 3 of the charter and the plan's criterion SHD-SC2 measure a safe stop by the park criterion.
- The cap smoothed at breakpoint crossings is a desirable exit criterion of card G2-MCU.
- Recorded: decision row D82, the change register, the journal.

## 5. Implementation and closure (on approval)

- **Documents:** the charter NASL-SHD-CHR-001 v1.3 and the plan NASL-SHD-PMP-001 v1.4; for option 2 or 3, the exit criterion on card G2-MCU.
- **Verification:** the descent card's runs already meet the restated measure (the S2c and S2d park criteria, every stop passing).
- **Records:** a decision row, the change register, the journal.
- **Closure:** when the two documents are issued.
