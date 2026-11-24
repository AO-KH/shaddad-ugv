Gate G0, independent assessment (the independent reviewer), saved by the lead 2026-11-16

Read only: I changed no file. I assessed the files as they stood at 23:57 on 2026-11-15. The checklist's dates were corrected at 23:54 and the cards file changed again at 23:57, during my assessment.

Cited paths are relative to `C:\Users\aomar\Desktop\Robot project\`. The full paths are listed at the end.

# Independent assessment of gate G0 (first assessment, 2026-11-15)

## 1. Scores per criterion

| ID | Class | Lead | Reviewer |
|---|---|---|---|
| G0-SC1 | M | met | **partly met** |
| G0-SC2 | M | met | **partly met** |
| G0-SC3 | M | met | **partly met** |
| G0-SC4 | M | met | met |
| G0-SC5 | D | met | met |
| SHD-SC1 (the project's own criterion, every gate) | M | not scored | **not assessed** |

**Readiness on my scores:** 1 of 4 standard mandatory criteria met (25 %), not 4 of 4. SHD-SC1 was not scored.

**G0-SC1: need, goals and constraints written, and approved by the owner.**
- **No charter exists.** The handbook names the charter as the evidence (engineering_handbook.md:224). It is what G0 locks (:173) and it has its own document type, CHR (:797). `documents.json` lists 31 documents and none is a CHR. The checklist uses three other sources in its place (G0_readiness_checklist.md:41).
- **Partly written.** PMP-001 section 1 (project_plan.md:39-54) gives the purpose and four constraints. It has no goal with a measure, and no budget or schedule.
- **Stale scope.** architecture.md:19-30 (revision B, 2026-10-19, :6) describes a 16S3P LFP pack of 15.36 kWh, two 2.5 kW PMSM motors and 469-609 kg. The model and the as-built record have a 13S15P NMC pack of about 2.95 kWh and four 750 W BLDC motors (param_power.m:9,44-45; real_robot_components.md:45,66).
- **Not approved.** PMP-001 is "Issued for approval" with reviewer and approver [TBD] (project_plan.md:9,12-13). "Met (the owner's approval at this gate)" counts an approval that has not been given. The handbook defines met as "the evidence shows it" (:304).

**G0-SC2: stakeholders and needs listed; each need has a source.**
- Five rows, each with a source (project_plan.md:58-64). The count is right.
- **Two sources are the project's own outputs.**
  - The operators' need cites the operating concept and the review (:61).
  - The farm users' need for mowing, tilling and spraying cites "the scenarios S3 to S5" (:62). Those scenarios hold no tilling or spraying: S3 and S4 carry the mower (scn_S3_endurance.m:25, scn_S4_greenhouse.m:16) and S5 carries no implement (scn_S5_palm_obstacles.m:33).
  - A first-hand source does exist: the "Operations offered (nasl-tech.com)" table of the Phase 1 report (docs/report/chapters/ch01_shaddad.json:22).
- **Missing stakeholders.** Maintainers and regulators are not listed (engineering_handbook.md:497). Yet the specs already cite ISO 18497 and ISO 13849 (electronics.md:27, power.md:848) and ISO 5395 (power.md:234). At G1-SC1 those requirements would have no need to trace to.

**G0-SC3: an operating concept, with scenarios for the main uses.**
- **Present.**
  - architecture.md:16-46 covers the product, the places, RC and autonomous operation, the mode authority and the independent E-stop chain.
  - S0-S8 have pass criteria (:451-459).
  - The abnormal cases are well covered: a stop on the ramp (S2), RC loss (S6), GNSS loss (S7), and E-stop with reset and re-arm (S8).
- **Gap: two of the three main uses have no scenario.** The main uses are mowing, tilling and spraying (project_plan.md:39; architecture.md:21). Only mowing has missions (S3, S4).
  - Tillage and fertilizer spraying have no scenario.
  - Pesticide spraying appears only as a full-tank load on the ramp (scn_S2_slope.m:65).
  - The project's own Phase 1 report records this in its table of operations (ch01_shaddad.json:22).
- No solar-farm world exists, though architecture.md:21 names solar farms.
- No user roles are given (the "who", handbook :498).

**G0-SC4: life cycle tailored, WBS, first cards with entry and exit criteria.** Met.
- The tailoring has its reasons (project_plan.md:68-80; project_cards.json:4264).
- The WBS is at project_plan.md:88-103.
- By script: all 54 cards carry one WBS code from that table, and all 7 G1 cards have entry and exit criteria.
- The weaknesses are in RID-G0-05.

**G0-SC5: each High risk has an owner.** Met.
- By script: 60 risks (29 High, 23 Medium, 8 Low).
- Every High risk has an owner and a mitigation card that exists.
- Every rating agrees with Table 20 (:628-634).
- The weaknesses are in RID-G0-08.

## 2. RID list (most serious first)

| RID | Criterion | Severity | What is wrong | Proposed action (owner) |
|---|---|---|---|---|
| RID-G0-01 | SC1 | Critical | There is no charter, and no recorded owner approval of any document. The threshold is not shown (see SC1 above). | Issue NASL-SHD-CHR-001 (1-2 pages): the need, goals each with a measure, constraints (with budget and schedule if any), and scope in and out. Register it, and record the owner's approval word for word at the board. (The lead drafts; the owner approves.) |
| RID-G0-02 | SC2 | Critical (mandatory criterion; the missing regulators touch safety) | Two needs cite the project's own outputs, and S3-S5 do not contain the cited need. Maintainers and regulators are missing although ISO 18497, 13849 and 5395 are already in the specs. | Add maintainers, and regulators with the standards that apply. Give each need a first-hand source: the operations table of ch01, and dated owner directions from the journal. (The lead; the owner confirms.) |
| RID-G0-03 | SC3 | Critical (mandatory criterion) | No scenario covers tillage or spraying, two of the three main uses. No solar-farm world; no user roles. | Either describe a tillage mission and a spraying mission (to be simulated later under planned S-numbers), or record the owner's decision that they are outside the current twin, with the phase in which they come in. Settle "solar farms". Add the roles. (The lead; the owner decides.) |
| RID-G0-04 | SC1, SC3 | Major (a baseline; the slope envelope touches safety) | The scope offered as the charter describes the robot from before 2026-10-27. Its 20 deg slope envelope (architecture.md:20) is overtaken by CR-001: the owner limited the full-tank sprayer to about 18.5 deg on 2026-11-15 (changes.json:12; CR-001_sprayer_descent_envelope.md:45). Locking it would lock a baseline known to be wrong. "None blocks G0" (checklist:51) misses this. | The charter states the as-built robot and the CR-001 envelope. It declares architecture.md section 1 superseded for scope until G1-01 re-baselines it. (The lead.) |
| RID-G0-05 | SC4 | Major (the G1 functional baseline) | No G1 exit criterion produces the trace from needs to requirements (G1-SC1), the quality check (G1-SC2) or the allocation (G1-SC4). See the G1-01 and G1-02 exits (project_cards.json:800, :894). The requirements template already has `source` and `allocation` fields (requirements_register.yaml:14,16). The G1-05 exit (:1166) offers "pass, pass with actions, or a delta review": no no-go, and not the outcomes of Table 14 (:320-325). | Add these exit criteria to G1-01 and G1-02 (or add a card). Align G1-05 with Table 14. Do it before G1-01 starts. (The lead.) |
| RID-G0-06 | SHD-SC1 | Minor | SHD-SC1 (M, every gate, project_plan.md:136) is not scored, though the template asks for the project's own criteria (template :36). "4 of 4" (checklist:47) is a miscount. | Amend PMP-001 Table 4 to "G1 onward" (G0 locks no model) and score it n/a with that reason, or score it on a sorted test run. (The lead.) |
| RID-G0-07 | SC4 (record) | Minor | G0-01 is shown as done without its stated check, "the owner reviews the handbook and the plan" (project_cards.json:598). HBK-001 and PMP-001 are both "Issued for approval", so the G0-02 entry criterion "G0-01 done" is not shown (definition of done, :405). Its effort is not given as a figure. | Record the owner's review of HBK-001 and PMP-001 at the board, or set G0-01 to "in review". Record the effort. |
| RID-G0-08 | SC5 | Minor | The register has no cause and effect, and gives a gate instead of a review date (:636-640). Risks that have already happened are kept as risks, though the handbook makes them issues (:642): RSK-008 "found 2026-11-15" (risks.json:134), RSK-017 "confirmed" (:296), and RSK-022 (:386), whose failure the BUILD gate already records (project_cards.json:299). Every entry is copied from a card. | Convert the risks that have happened to issues. Add cause, effect and a review date at the G1 register review. (The lead.) |
| RID-G0-09 | Records | Minor | PMP-001 counts 13 done and 37 to do (project_plan.md:33); the cards file has 14 and 36. PMP-001 states the G0 criteria are met before this assessment (:187) and lists G0-01 as under way (:190). The cards file still cites docs/lifecycle.md (project_cards.json:2, :1144), which the register marks as superseded (LCY-001). | Correct them at the next issue of PMP-001 and of the cards file. |

## 3. Checklist errors

**Checked and correct:**
- The four G0 cards and their statuses.
- 54 cards, each with one WBS code.
- 7 G1 cards with entry and exit criteria.
- 60 risks, 29 High, each with an owner and a card.
- 5 stakeholders.
- All 87 review items on a card (PLN-001 Appendix A, build_task_cards.py:576).
- Every evidence path exists.

**Errors:**
1. SC1 is scored "met" on an approval that has not yet happened (:41). It should be "partly met" (RID-G0-01).
2. SHD-SC1 is missing, so the readiness line is miscounted (:47) (RID-G0-06).
3. G0-01 is shown as done without its stated check (:32) (RID-G0-07).
4. The owner's words are not quoted in full.
   - :22 quotes "Yes, hold G0 now". The journal records "Yes, hold G0 now (Recommended)" (journal.jsonl:961): the owner chose the lead's recommended option.
   - That journal entry still dates the choice 2026-11-16, corrected at journal.jsonl:963. The brief I was given also said 2026-11-16. The decision record should carry 2026-11-15.
5. The SC1 evidence is given as "the Phase 1 reports in docs/report" (:41). That is a folder, not a document number. It should cite ANR-003 (the ch01 operations table). It also differs from the cards file's note for G0-SC1, which says "the report" and omits PMP-001.
6. "None blocks G0" (:51) is wrong: RSK-008 and CR-001 change the envelope that the charter locks (RID-G0-04).

## 4. Recommended outcome: delta review on SC1, SC2 and SC3

- **Why a delta review:**
  - Three mandatory criteria are only partly met. Table 14 (:324) gives a delta review for "one or more mandatory criteria not met, and the gaps can be closed within the phase".
  - The gaps are all in documents: a charter page, the stakeholder table and the operating concept's main uses. No model work is needed.
  - Tailoring SHD-SC1 in PMP-001 (RID-G0-06) should go into the same delta.
- **Why not go with actions:** that outcome needs every mandatory criterion met or waived, and no open RID touching safety (:323). RID-G0-02 and RID-G0-04 touch safety.
- **Why not no-go:** the concept is sound and worth developing. The model runs on the as-built robot, the plan is tailored, and the cards and the risk register are consistent.
- **The owner's alternative:** waive the SC3 tillage and spraying gap, with a reason, its risk and an expiry date at G1-05 (:327). Approve a corrected charter at the board and fix the stakeholder table. Then decide go with actions, with RID-G0-04 to RID-G0-09 on cards. Under Table 14, RID-G0-04 touches safety, so it still has to close before that decision.

**Evidence files (full paths):**
- C:\Users\aomar\Desktop\Robot project\docs\gates\G0_readiness_checklist.md
- C:\Users\aomar\Desktop\Robot project\docs\project\project_plan.md
- C:\Users\aomar\Desktop\Robot project\docs\architecture.md
- C:\Users\aomar\Desktop\Robot project\tools\dashboard\project_cards.json
- C:\Users\aomar\Desktop\Robot project\tools\dashboard\journal.jsonl
- C:\Users\aomar\Desktop\Robot project\docs\registers\risks.json
- C:\Users\aomar\Desktop\Robot project\docs\registers\documents.json
- C:\Users\aomar\Desktop\Robot project\docs\registers\changes.json
- C:\Users\aomar\Desktop\Robot project\docs\changes\CR-001_sprayer_descent_envelope.md
- C:\Users\aomar\Desktop\Robot project\docs\hardware\real_robot_components.md
- C:\Users\aomar\Desktop\Robot project\params\param_power.m
- C:\Users\aomar\Desktop\Robot project\scripts\scenarios\scn_S2_slope.m
- C:\Users\aomar\Desktop\Robot project\scripts\scenarios\scn_S3_endurance.m
- C:\Users\aomar\Desktop\Robot project\scripts\scenarios\scn_S4_greenhouse.m
- C:\Users\aomar\Desktop\Robot project\scripts\scenarios\scn_S5_palm_obstacles.m
- C:\Users\aomar\Desktop\Robot project\docs\report\chapters\ch01_shaddad.json
- C:\Users\aomar\Desktop\Robot project\docs\specs\electronics.md
- C:\Users\aomar\Desktop\Robot project\docs\specs\power.md
- C:\Users\aomar\Desktop\Robot project\tools\reports\build_task_cards.py
- C:\Users\aomar\Desktop\Robot project\framework\handbook\engineering_handbook.md
- C:\Users\aomar\Desktop\Robot project\framework\templates\gate_readiness_checklist.md
- C:\Users\aomar\Desktop\Robot project\framework\templates\requirements_register.yaml
