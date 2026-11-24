<!--
doc_no: NASL-EMS-PRO-001
doc_type: PROCEDURE
title: Procedure: making an engineering report
subtitle: From the records of a task group to the released PDF
summary: How the team makes the engineering report (ER) of a task group: what it is built from, the eight steps from closing the work to releasing the PDF, the checks, who does each step, and when the report is done.
version: 1.0
date: 2026-11-16
status: Issued
history: 1.0 | 2026-11-16 | First issue, at the owner's request ("give md file on the steps of making the report")
prepared_by: the lead systems engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies
classification: Internal
footer: Procedure: making an engineering report
file: NASL_PRO-001_Engineering_Report_Procedure_v1.0
outdir: framework/build
related: NASL-EMS-HBK-001 (the handbook; its v1.2, card G1-09, adds the PRO type for procedures); framework/templates/engineering_report.md
-->
# Procedure: making an engineering report

## Summary

**An engineering report (ER) documents one task group, from the owner's request to the result.** The team builds it from records it keeps while it works, so writing the report is mostly assembling and checking, not remembering.

- **When:** at the end of every task group, without being asked. That is the owner's standing rule of 2026-11-15.
- **Who:** the lead systems engineer, with the technical writer if wanted.
- **How long:** after a well-kept task, one pass.

In the commands below, `<WORK>` is the team's work folder (today `work/agentic/`) and `<id>` is the task group's short id, for example `descent`.

## 1. When a report is made

- **Every task group ends with an ER.** A task group is a card with several steps or several engineers.
- **A single-step task** keeps its markdown task report only.
- **Other events have their own documents:**
  - a gate review: the gate review report (GRR);
  - a change: its change request (CR);
  - a test campaign: its test report (TR).

## 2. What the report is built from

Keep these records during the task. The report cites them; it never replaces them.

Table 1. The records behind an engineering report.

| Record | Where | Kept by |
|---|---|---|
| The owner's request, word for word, with the date | the journal: `python tools/dashboard/log.py decision "..."` | the lead systems engineer |
| Every step: its start, its result in numbers, its end | the journal: `log.py start`, `result`, `done` | each engineer |
| The engineering passes: who, what, how long, the effort | the work log, on the dashboard's Team tab | the lead systems engineer |
| Each engineer's step report | `<WORK>/<task>/<role>/report_<step>.md` | each engineer |
| Logs and results | `<WORK>/logs/`, `work/results/` | each engineer |
| Decisions and changes | the decision log (the D-rows of `docs/architecture.md`), the change requests in `docs/changes/` | the lead systems engineer; the documentation engineer |
| The independent review and its findings | `<WORK>/<task>/review/report.md` | the independent reviewer |
| The task card | `tools/dashboard/project_cards.json` | the lead systems engineer |

## 3. The steps

### Step 1. Close the work first

Write the report only once the work has stopped changing:
- the independent review is done and its fixes are in;
- the docs are updated: decision rows, specs, interfaces, STATUS;
- the tests are run and sorted in counts: passed, failed, known, new;
- every open point is on a card or in STATUS.

### Step 2. Write the account

The account is the report's body, in `<WORK>/reports/src/<id>.md`, in eleven sections. The template is `framework/templates/engineering_report.md`.

Table 2. The eleven sections of the account.

| Section | What it holds |
|---|---|
| Summary | The result in one sentence, what was done and by whom, the key outputs, the cost |
| 1. Purpose and scope | Why the task exists; what is in and out of scope |
| 2. Background and the request | The owner's words with their dates; what led to the task; the order of work |
| 3. Approach and rationale | Each choice, its alternatives and why they were rejected |
| 4. Organisation and method | Who did what, by role; the rules (read only, batch, one pass); the checks |
| 5. Work performed | Each step: what was done, the files, the decision rows |
| 6. Results | The numbers, before and after, in numbered tables and figures |
| 7. Verification and validation | The tests in counts, the system runs, the independent review; what is not verified, and the command that would verify it |
| 8. Decisions and rationale | Each decision: who made it, the choice, why, and the owner's words |
| 9. Issues, risks and limitations | The limits, what is not confirmed, the slips and how they were repaired |
| 10. Conclusions and recommendations | What follows, and on which card |
| 11. Cost and schedule | The period, and the effort against the estimate |

**Writing rules:**
1. Lead with the result, then the numbers.
2. Give numbers before and after, with units, and name the case each comes from.
3. Quote the owner's request and decisions word for word, with their dates.
4. Say what is not verified, and how it would be. Mark estimates `[EST]` and measured or vendor values `[DATA]`. Never invent a number.
5. Cite sources as `file:line`, log paths and report paths.
6. Write dates in full (2026-11-16), never "today".
7. State the cost in engineering passes and time, against the estimate.
8. Number every figure and table, and refer to each one in the text.
9. Keep the style plain: short sentences, lists, tables for comparisons.
10. Name the team by its roles only, and say nothing about AI. That is the owner's rule of 2026-11-16.

### Step 3. Generate the task report

```bash
python tools/dashboard/task_report.py <id> --journal "<journal task name>" --intro <WORK>/reports/src/<id>.md --ref <path> --ref <path>
```

It writes `<WORK>/reports/<id>.md`, which contains:
- a header table: the number, the task, the period, the status, the effort;
- the account of step 2;
- Appendix A, the work by role, with a timeline and an effort figure;
- Appendix B, the engineering passes;
- Appendix C, the journal entries of the task;
- the references.

Add the same command line to `<WORK>/reports/src/regenerate.sh`, so that the report can be rebuilt from the records at any time.

### Step 4. Register the report

Add one entry to `tools/reports/engineering_reports.json`.

Table 3. The fields of a report entry.

| Field | Content |
|---|---|
| `id` | the task group's id, as in step 3 |
| `doc_no` | NASL-<CODE>-ER-NNN, the next number in order |
| `title`, `subtitle`, `footer` | for example "Shaddad UGV: the downhill speed cap", "Task group of 15 to 16 November 2026" |
| `summary` | two or three sentences, for the cover |
| `version`, `date`, `history` | 1.0, the date, and [version, date, change] rows |
| `request` | the owner's words that started the task, with the date |
| `scope` | one line: the items and decision rows the report covers |
| `file` | `NASL_ER-NNN_<Project>_<Task>_<date>_v<version>` |

### Step 5. Build the PDF

```bash
python tools/reports/build_engineering_report.py <id>
```

- **What it does:** puts the task report into the NASL Word template, renders the figures, and makes the PDF through Word.
- **Output:** `work/reports/engineering/<file>.docx` and `<file>.pdf`.
- **Options:** `--no-pdf` makes the Word file only; `--all` rebuilds every report.

### Step 6. Check every page

```bash
powershell -File tools/reports/pdf_pages_to_png.ps1 -Pdf <absolute pdf path> -OutDir <absolute folder> -Pages "1,2,3" -Width 900
```

Look at every page. The builder handles these traps, but check for them:
- a blank page before a chapter;
- a narrow table column breaking words;
- an id such as R-05 wrapping at its hyphen;
- a tall figure pushing its caption to the next page.

**Scan the text for AI wording.** The team is named by roles only:

```bash
python -c "import pymupdf, re, sys; d = pymupdf.open(sys.argv[1]); p = re.compile(r'(?i)\bAI\b|agent|claude|\btokens?\b|\bsessions?\b'); print([(i + 1, m.group()) for i, pg in enumerate(d) for m in p.finditer(pg.get_text())])" <pdf>
```

The scan must print an empty list.

### Step 7. Release the report

1. **Register it** in `docs/registers/documents.json`: the number, type ER, the title, the source, and the status "Released (v1.0)".
2. **Close the card** in `tools/dashboard/project_cards.json`: `report` is the PDF path, `effort` is the effort, and the status is done once its exit criteria are met.
3. **Log it:**

   ```bash
   python tools/dashboard/log.py report "<doc_no> issued: <one line>" --task "<task>" --ref <pdf path>
   ```
4. **Send the PDF to the owner,** with one line on the result and what is still open.

### Step 8. Correct it, when needed

- **Never edit the PDF or the Word file by hand.**
- **To correct a report:**
  1. change the account, or the record it comes from;
  2. regenerate it (step 3);
  3. rebuild it (step 5) with a new version and a history row.
- **The old version stays as it was issued.**

## 4. Done when

- [ ] All eleven sections are filled; the result comes first, and every number has its unit.
- [ ] The owner's request and decisions are quoted word for word, with their dates.
- [ ] The verification is given in counts, with the independent review's verdict and its fixes.
- [ ] What is not verified is stated, with the command that would verify it.
- [ ] The cost is given against the estimate.
- [ ] Every figure and table is numbered and referred to.
- [ ] Every page is checked, and the AI-wording scan is empty.
- [ ] The report is registered in both registers, the card is closed with its report, the release is logged, and the PDF is sent to the owner.

## 5. Who does what

Table 4. The roles in making a report.

| Role | Does |
|---|---|
| Lead systems engineer | Closes the work, writes the account, generates, builds, checks and releases the report |
| Technical writer | May draft the account from the records, and formats it; never invents a number |
| Independent reviewer | Checks the report's facts on request, above all when it carries a safety claim |
| Each engineer | Keeps the step reports and journal entries that the report cites |
| Project owner | Receives the report; asks for corrections |

## 6. Example: the descent card, ER-005

```bash
python tools/dashboard/task_report.py descent --journal "descent" --intro <WORK>/reports/src/descent.md --ref docs/changes/CR-001_sprayer_descent_envelope.md --ref docs/changes/CR-003_measure_of_safe_stops.md
python tools/reports/build_engineering_report.py descent
```

The account goes in `<WORK>/reports/src/descent.md` and the entry in `tools/reports/engineering_reports.json` under `doc_no` NASL-SHD-ER-005. The PDF then lands in `work/reports/engineering/`.
