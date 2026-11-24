## Summary
**<The result in one sentence.>** <What was done, by whom.>
- **<Key output>:** `<path>`.
**Effort:** <engineering passes and days> against <the estimate>.

## 1. Purpose and scope
**Purpose.** <Why the task group exists, in the product's terms.> **In scope:** <...> **Out of scope:** <...>

## 2. Background and the request
<The owner's words with the date; what led to the task; its card(s) and gate.>

## 3. Approach and rationale
- **<Choice>.** <Why.> *Alternative:* <...>. *Rejected:* <why>.

## 4. Organisation and method
| Step | Who | What |
|---|---|---|
| 1 | <role> | <what> |

<The rules (one pass, snapshots, batch runs), the checks.>

## 5. Work performed
1. **<Step>:** <what was done; the files; the decision records>.

## 6. Results
![Figure 1: <caption, with what the axes show>](<path>)

Table 1. <caption>

| ... | ... |
|---|---|

## 7. Verification and validation
<Tests in counts (passed, failed, known, new); system runs; the independent review; the exit criteria of the card, each with its evidence; what is not verified, and the test that would verify it.>

## 8. Decisions and rationale
| Decision | By | Choice | Why |
|---|---|---|---|

## 9. Issues, risks and limitations
<Limits; what is not confirmed; the risks left (with RSK IDs); slips and how they were repaired; corrections to earlier reports.>

## 10. Conclusions and recommendations
1. **<...>**

## 11. Cost and schedule
- **Period:** <date, start to end>.
- **Effort:** <engineering passes and days> against <the estimate>.

<!-- How to use: this is the lead's account of a task group (NASL-EMS-HBK-001 section 6.5). Save it as <reports folder>/src/<id>.md; tools/dashboard/task_report.py adds the appendices (the work by role, the passes, the journal) and the references; tools/reports/build_engineering_report.py <id> makes the NASL PDF (register the report in tools/reports/engineering_reports.json first). -->
