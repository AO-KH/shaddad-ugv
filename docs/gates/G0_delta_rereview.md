Gate G0, delta re-review (the independent reviewer), saved by the lead 2026-11-16

Read only: I changed no file. This re-review covers the files as they stood at 00:09 on 2026-11-16. Paths are relative to `C:\Users\aomar\Desktop\Robot project\`; the full paths are at the end.

# Delta re-review of gate G0 (2026-11-16)

The owner's three decisions are recorded word for word in the journal: the delta review (journal.jsonl:968), "Mowing now, the rest later" (:969) and roles only (:970).

## 1. Scores

| ID | Before | Now | Evidence |
|---|---|---|---|
| G0-SC1 | partly met | **partly met until the board; met once the owner's approval is recorded** | The charter states the need with its sources (charter.md:34-38) and six goals, each with a measure and a gate (:44-51). It says outright that no budget or end date is set (:53). The constraints cover the as-built robot (13S15P, about 2.95 kWh, four 750 W BLDC motors) and the CR-001 envelope (:57-70). Scope in and out is at :76-83 and the approval step at :119. |
| G0-SC2 | partly met | **met** | Table 1 now has 7 stakeholders, with maintainers and regulators added (project_plan.md:68-74). Each need has a dated or documentary source, and none is sourced to the concept or the scenarios any more. |
| G0-SC3 | partly met | **met** | The owner limited this phase to mowing (journal.jsonl:969). Charter section 6 gives the places, the modes, normal use and abnormal use (:96-113), and section 5 the roles (:87-90). Palm-grove mowing is covered by S3 and greenhouse mowing by S4. Tillage and spraying are out of scope (:78-79) and sit on proposed cards G2-TIL and G2-SPR. |

**The charter is fit to be approved.** No content gap blocks it. I advise correcting three lines before the board (RID-G0-12), so the approved text makes no overstated claim about which safety cases the scenarios cover.

## 2. The RIDs

**Closed:**
- **RID-G0-02 (was Critical):** closed by Table 1. One suggestion, not a RID: the regulators' row finds the standards through the project's own specs, gives no editions and names no jurisdiction (project_plan.md:72). Settle that at G1-01, where the trace is built.
- **RID-G0-03 (was Critical):** closed by the owner's decision (journal.jsonl:969), charter sections 4 to 6, and the cards G2-TIL and G2-SPR.
- **RID-G0-04 (was Major):** closed. Sections 3 and 6 state the as-built robot and the CR-001 envelope, and the charter supersedes architecture section 1 (charter.md:6, :115).
- **RID-G0-05 (was Major):** closed. G1-01's exit criteria now cover the trace to the needs, the reviewer's quality check and the allocation. G1-05 offers the four outcomes.
- **RID-G0-06 (was Minor):** closed. SHD-SC1 now applies from G1 onward (project_plan.md:147).

**Still open:**

| RID | Severity | What remains |
|---|---|---|
| RID-G0-01 | Critical until the board | It closes when the owner's approval is recorded word for word, and CHR-001 is entered in the document register (RID-G0-11). |
| RID-G0-07 | Minor | G0-01 is now "in review" in the cards and the plan (project_plan.md:205). It closes when the owner's review of HBK-001 and PMP-001 is recorded at the board. |
| RID-G0-08 | Minor | Partly done. RSK-008, RSK-017 and RSK-022 are now issues. Cause, effect and review dates are put off to "the G1 register review", with no card and no date (checklist:74). |
| RID-G0-09 | Minor | Partly done. The card counts and the early "met" claim are gone. Two items remain: the cards file still cites the superseded docs/lifecycle.md (its `about` text and card G1-05's links), and the plan says G2-DSC is closed (:193) while listing it as under way (:207). |

**New, all Minor:**

| RID | What is wrong | Action |
|---|---|---|
| RID-G0-10 | The checklist v1.1 contradicts itself or is out of date: <br>- Table 1 still shows G0-01 as done (:33). <br>- It says "6 stakeholders" where Table 1 has 7 (:43). <br>- It says "54 cards" where there are now 56 (:45). <br>- It says "60 risks, 29 High" (:46, :52); the register now has 57 risks (26 High) plus 3 High issues. <br>- "4 of 4 met" (:48) contradicts :42 and :79. <br>- Table 2 is still dated 2026-11-15 (:38). | Correct these before the gate report. |
| RID-G0-11 | Three records of the delta are missing: <br>- CHR-001 is not in documents.json (unchanged since 2026-11-15 23:39). <br>- Architecture section 1 (CON-001) is not marked as superseded anywhere. <br>- The scope decision of journal.jsonl:969 has no decision-log row. | Register CHR-001, note that CON-001 section 1 is superseded, and add a decision row at the board. |
| RID-G0-12 | Four points in the charter: <br>- Goal 4 says "with margin" without a figure (:49); SHD-SC3 sets ≥ 10 % (project_plan.md:149). <br>- "S0 to S8 exercise these cases" (:113) overstates. Only S7's GNSS loss injects a sensor fault, and low energy is checked only as an SOC floor at the end of the shift, not as a return to dock (scn_S3_endurance.m:12, :64-65). <br>- The raised-tiller instability (:65) cites no record; G2-TIL's description states it but its risks list is empty. <br>- The architecture's section 12, which the charter points to, still has S2 at 609 kg on 20 deg and S3 at 4 h (architecture.md:453-454), while the S3 script now defaults to the 3 h shift (scn_S3_endurance.m:6). | Fix the first three before the board. Section 12 goes to G1-01. |
| RID-G0-13 | The owner's roles-only decision (journal.jsonl:970) is not yet applied: <br>- risks.json still names owners by internal handles, not by role (56 occurrences, for example RSK-008's owner). <br>- The plan's reviewer and approver are still [TBD] (project_plan.md:13-14). <br>- Its rule "signatures stay [TBD]" (:137) contradicts the decision. | Convert these at the next issue. |

## 3. Recommended outcome (Table 14)

**Go with actions, once the owner approves the charter at the board.**
- With that approval recorded, every mandatory criterion is met: SC1 to SC4, with SHD-SC1 applying from G1.
- No Critical or Major RID stays open. RID-G0-07 closes at the board.
- RID-G0-08 to RID-G0-13 are Minor. If RID-G0-12's coverage line is fixed before the approval, none of them touches safety.
- Table 14 would also allow a plain "go", since no Critical or Major RID would be open. "Go with actions" is the outcome that requires each open RID to have a card and a date and to close before G1-05.

**If the owner does not approve the charter,** SC1 stays not met and the delta review stays open on SC1 alone.

**Files:**
- C:\Users\aomar\Desktop\Robot project\docs\project\charter.md
- C:\Users\aomar\Desktop\Robot project\docs\project\project_plan.md
- C:\Users\aomar\Desktop\Robot project\docs\gates\G0_readiness_checklist.md
- C:\Users\aomar\Desktop\Robot project\tools\dashboard\project_cards.json
- C:\Users\aomar\Desktop\Robot project\tools\dashboard\journal.jsonl
- C:\Users\aomar\Desktop\Robot project\docs\registers\risks.json
- C:\Users\aomar\Desktop\Robot project\docs\registers\documents.json
- C:\Users\aomar\Desktop\Robot project\docs\architecture.md
- C:\Users\aomar\Desktop\Robot project\scripts\scenarios\scn_S3_endurance.m
