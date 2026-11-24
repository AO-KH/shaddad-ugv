# Reports: the dashboard's Reports section, and how to write each report

This is the reporting part of the agent-team workflow (see `AGENT_TEAM_PLAYBOOK.md`; the look is in `DASHBOARD_STYLE.md`). It sits under the Engineering Handbook NASL-EMS-HBK-001: controlled documents (plans, procedures, change requests, gate reports) are built with `tools/reports/build_document.py` from the templates in `framework/templates/` (handbook section 8.5). It has two parts:
- what the dashboard's Reports section does;
- how each kind of report is written, by whom, where it goes, and in what shape.

The reference implementation is the Shaddad project in `C:/Users/aomar/Desktop/Robot project`:
- `tools/dashboard/collect.py`: `REPORT_GLOBS` and `reports()`;
- `tools/dashboard/static/index.html`: `REPORT_CATS` and the reader;
- `tools/dashboard/task_report.py`;
- `work/agentic/reports/`.

---

## 1. The Reports section

**Finding reports.**
- The collector looks for report files by path pattern. `REPORT_GLOBS` is a list of `(glob, kind)` pairs, read again every 30 s.
- A report's title is its first `#` heading for Markdown, its file name for logs and PDFs, and its folder name for HTML test reports.
- Reports are listed newest first, up to 200.

**Grouping.**
- `REPORT_CATS` in the page maps kinds to categories.
- A row of category buttons shows the counts. "All" shows every category with its 8 newest reports and a "Show N more" button.
- A kind with no category lands in **Other**.

**Reading.**
- Markdown renders in the page, with headings, tables, lists and code.
- A figure is written as `![Figure n: caption](path from the project root)` and served through `/raw/<path>`.
- Logs open as text. HTML reports and PDFs open raw in a new tab.

**Adding a new kind of report:**
1. Add a `REPORT_GLOBS` entry `('<folder>/<pattern>', '<Kind>')` to `collect.py`, then restart the server.
2. Add a `REPORT_CATS` row `['<id>', '<Category label>', (r) => r.kind === '<Kind>']` to `index.html`, then reload the page. Put it first if it is what the user reads most.

**The default categories:**

| Category | Kind(s) | Files | Written by |
|---|---|---|---|
| Engineering reports (PDF) | Engineering report | `work/reports/engineering/*.pdf` | the lead, through `tools/reports/build_engineering_report.py` (section 2.7) |
| Review findings | Review findings, Review section | `work/agentic/review/*.md`, `work/agentic/review/sections/*.md`, `work/reports/review_*/*.pdf` | the engineers (sections), the integration engineer (system and ranked list), the report writer (PDF) |
| Agent task reports | Agent task report | `work/agentic/reports/<task-id>.md` | the lead, through `task_report.py` |
| Agent step reports | Agent step report, Agent summary | `work/agentic/<task>/<agent>/report.md`, `work/agentic/<task>/report.md` | each agent; the lead saves them |
| Plan tasks and reviews | Task report, Task decisions, Review | the plan workspaces | the plan's implementers and reviewers |
| Project status | Project status | `docs/STATUS.md` | the docs keeper |
| Test reports | Test report | `work/reports/*/index.html`, `report.html` | the test tools |
| Run logs | Run log | `work/reports/*.log` | the batch runs |

---

## 2. The report kinds

### 2.1 Agent step report: one agent, one step
- **Who:** every engineer or specialist, as its final message. Subagents often cannot write report files themselves.
- **Saving:** the lead runs `python tools/agentic/save_agent_result.py <agentId> work/agentic/<task>/<agent>/report.md --header "Agents task <id>, step N (<agent>, <model>), saved by the lead <date>"`.
  - The header becomes an HTML comment at the top.
  - Check the file's size. If the agent's last message was only a short closing note, write the real report from the notification by hand, with the same header.
- **Several passes** of one agent get separate files: `report.md`, `report_part2.md`, `report_layoutpass.md`.
- **Shape**, always in this order:
  1. **Status:** one line (done, blocked, or needs a decision).
  2. **Files changed:** `path:lines` with a one-line reason each, plus where the snapshots are. New files are listed as new.
  3. **Checks run:** the command, the log path, and the result in counts. Name new failures one by one; say "0 new failures" when there are none.
  4. **Behaviour before and after:** a table in numbers, with units and the case each number comes from.
  5. **For the docs keeper:** the exact facts that changed (interface rows, bits, enum codes, transitions, parameters with units and `[EST]` or `[DATA]`), so the docs can be updated without reading the code.
  6. **Open points and questions:** numbered. Each one says what the lead or the user must decide.
- **Optional headings,** when they apply:
  - "Test expectations changed (lead-approved)";
  - "Choices beyond the brief";
  - "My slips": what went wrong and what was restored.

### 2.2 Agent task report (ATR-<id>): the whole task
- **Who:** the lead. Write it as the task progresses, and once more when it ends.
- **How:**
  1. Write the lead's account in `work/agentic/reports/src/<id>.md` (template in section 4).
  2. Run `python tools/dashboard/task_report.py <id> --journal "<journal task name>" [--journal ...] --intro work/agentic/reports/src/<id>.md --ref <step reports and other sources>`.
  3. Add that exact line to `work/agentic/reports/src/regenerate.sh`, so `bash work/agentic/reports/src/regenerate.sh` rebuilds every report.
  4. **If the task is a task group** (several steps or several agents), build its PDF engineering report from this ATR (section 2.7) and update the task card (section 2.8). Do it without being asked.
- **What the generator adds:**
  - **A header table:** report ATR-<id>, task, period, status (steps done), prepared by (the agents and the lead), reviewed by (or "no independent review in this task"), and effort (runs, tokens, agent time).
  - **Your intro.**
  - **Appendix A, the agent work:** two SVG figures (the run timeline with one lane per agent, and the tokens by agent), numbered after your own figures, then a table of the steps.
  - **Appendix B:** every run, with its model, time, tokens and result.
  - **Appendix C:** the journal entries of the task, taken from the `--journal` names.
  - **References:** the journal's `--ref` paths and your `--ref` paths that exist.
- **Figures from data** (state strips, paths, signals against thresholds, before-and-after bars):
  - Make them with a script in a batch run, for example `work/agentic/<task>/lead/<task>_figures.m`, or Python plotting if it is installed. Installing a plotting library needs the user's OK.
  - Save PNGs under `work/`, and number and caption each one.
  - One idea per figure. Put the axes in the units of the text, and add a threshold line where there is a criterion.

### 2.3 Review findings: what to improve, ranked
- **Who:**
  - each subsystem engineer reviews its own section and its recent tasks;
  - the integration engineer reviews the whole system, then merges everything into one ranked list;
  - the lead calibrates the ranks;
  - the report writer makes the formal PDF.
- **The brief:** one shared file, `work/agentic/review/REVIEW_BRIEF.md`, gives:
  - the rules (read only, and no new simulations unless asked);
  - the severity scale and the template;
  - at most 15 findings per section;
  - an ID prefix per section.
- **The severity scale:**
  - **Critical:** wrong, unsafe, or blocks results; must be fixed or removed;
  - **High:** affects results or the real product; fix soon;
  - **Mid:** worth fixing, with a limited effect;
  - **Low:** cosmetic.
- **One finding** (template in section 4): ID and title; severity and confidence; confirmed (yes, partly or no) with the command that would confirm it; where (`file:line`); what is wrong; why it matters; the evidence; the suggested action with a size (S, M or L); related findings and decisions.
- **The ranked list** (`ranked.md`) has one row per improvement area:

  `| R-ID | Severity (with any rank change and its reason) | Title | Source IDs | Effect | Action and size | Owner(s) | Confirmed | Needs the user |`

  Every source finding ID must appear in some row; the lead checks this. The list ends with **Decisions for the user** (numbered, recommended option first) and a **Suggested order of work**.

### 2.4 Formal report (Word and PDF)
- **Who:** the report writer agent, from the results the lead hands over: the ranked list, the step reports, the logs and the figures.
- **House style:** set per organisation, and recorded in memory or the report writer's agent file:
  - the template and branding;
  - the language;
  - the document number and version scheme;
  - sign-off names, left as `[TBD]` until the user names them.
- **Building it:**
  - copy the organisation's Word template, fill it with a script, and keep the build scripts beside the output so it can be rebuilt;
  - make the PDF with Word (COM automation on Windows) or LibreOffice;
  - check every page before delivering;
  - without a docx library, fill the template's XML with the Python standard library (zipfile and xml).
- **Chapters:** executive summary; scope and method; results by area; the ranked findings; the decisions for the user (their own chapter); the suggested order of work; appendices.
- **Deliver** the PDF to the user with SendUserFile, and keep the Word file beside it.

### 2.5 Project status (`docs/STATUS.md`)
A living document that the docs keeper keeps current from the step reports:
- a dated current-state section;
- the model or component map;
- numbered open problems, each with its cause, its consequence and its next step;
- fixed items closed with the date and the reference.

Never leave a fixed problem listed as open.

### 2.6 Test reports and run logs
These are generated by the tools. They need only a path that `REPORT_GLOBS` covers and a category.

### 2.7 Engineering report (PDF): every task group
The user's standing rule (Shaddad, 2026-11-15): every task group ends with a PDF engineering report that documents everything in the task: the purpose, the approach, the reasons and the rest. Tasks of a single step keep the markdown ATR only.
- **Who:** the lead, when the task ends, without being asked.
- **The source** is the ATR of section 2.2: the lead's intro in the **11-section structure** (template in section 4), plus the generated appendices A-C and the references.
- **Register it** in `tools/reports/engineering_reports.json`, one entry per report:
  - `doc_no` (NASL-<project>-ER-NNN, numbered in order), `version` and `date`, `history`;
  - `title`, `subtitle` and the page `footer`;
  - `summary`, `request` (the user's words) and `scope` (one line each, for the cover and the document control page);
  - `file` (the PDF name: `<ORG>_ER-NNN_<Project>_<Task>_<date>_v<version>.pdf`).
- **Build it:** `python tools/reports/build_engineering_report.py <id>` (or `--all`; `--no-pdf` for the Word file only).
  - It parses the ATR markdown (headings, tables with their "Table N." captions, figures, lists, quotes, code) into the organisation's Word template (`tools/reports/nasl_docx.py`: the `Doc` class and `package()`).
  - SVG figures are rendered to PNG by headless Edge at 2x; Word makes the PDF through COM (`tools/reports/convert_to_pdf.ps1`).
  - Output: `work/reports/engineering/<file>.docx` and `.pdf`.
- **Check every page** before delivering: `powershell -File tools/reports/pdf_pages_to_png.ps1 -Pdf <absolute pdf> -OutDir <absolute folder> -Pages "1,2,3" -Width 900`, then look at the PNGs. Known traps, all handled by the builder but worth a look:
  - a page break before every chapter leaves blank pages (only the summary, the appendices and the references start a page);
  - narrow table columns break words; each column gets at least its longest word;
  - IDs such as R-05 wrap at the hyphen (the builder makes ID hyphens non-breaking);
  - a tall figure pushes its caption to the next page (figures are capped at 16 cm).
- **Deliver** the PDF with SendUserFile, and log it (`log.py report`).
- **Corrections** go into the source ATR, then rebuild with a new version and a history row.

### 2.8 Project task cards and the life-cycle gates
The project runs to a defined end through review gates (tailored for a small team: `docs/lifecycle.md`):
- **G1 SRR + SFR** (system, with the verification matrix), **G2 PDR** and **G3 CDR** per subsystem, **Build**, **G4 TRR** per test campaign, **G5 FCA / SVR**, **G7 PCA**; a PRR only if production starts.
- **Each gate review** is recorded as a PDF engineering report (section 2.7) and a decision row.

The work is defined as **task cards** in `tools/dashboard/project_cards.json`, shown on the dashboard's Project tab:
- `gates`: `{id, name, title, scope, question, locks, status (open | passed), assessment}`;
- `cards`: `{id, gate, title, lead, support, status (todo | in progress | needs you | done), priority (now | next | later | done), purpose, entry {gate, criteria}, exit {gate, criteria}, predecessors, successors, inputs, outputs, verification, size (S < 0.5 M tokens, M 0.5-1.5 M, L 1.5-4 M, XL > 4 M), effort, risks [{risk, likelihood, impact, mitigation}], escalation [{when, to}], decisions, links, report, task, note}`.
  - `successors` are derived from the other cards' `predecessors` by every consumer, so only the predecessors are edited.
  - `links` holds the review items (R-xx), decisions, STATUS items and other cards.
- **The project is done when every card is done and every gate is passed.** Every review item sits on a card.
- **When a task starts,** set its card to `in progress` (priority `now`) with the Agents task id in `task`. **When it ends,** set `done` and the PDF path in `report`. New work gets a new card under the gate it serves, with every field filled.
- **The card definitions as a PDF:** `python tools/reports/build_task_cards.py` writes `work/reports/project/NASL_PLN-001_<project>_Task_Cards_<date>_v<version>.pdf` (dashboard category "Project documents"). It holds the gates, the card fields, the roles, the escalation ladder, the risk and size scales, the card map, the longest chain of open work, one table per card, and the review items and decisions by card. Rebuild it, with a new version, after the cards change.

---

## 3. Writing rules for every report

1. **Lead with the result,** in one or two sentences, then the numbers. Put the detail in appendices or the linked step reports.
2. **Give the numbers before and after,** with units, and name the case each number comes from. Test results are counts: passed, failed, known, new.
3. **Quote the user's request** in their own words in the Background, with the date.
4. **Say what is not verified,** and give the command that would verify it. Mark estimates `[EST]` and measured or vendor values `[DATA]`. Never invent a number.
5. **Cite sources** as `file:line`, log paths and report paths. Line numbers drift, so a row in a table is cited by its text.
6. **Dates are absolute** (2026-11-15), never "today" or "yesterday".
7. **Report the cost:** agent runs, tokens and time, against the estimate given beforehand.
8. **Decisions for the user** are numbered, with the recommended option first and the consequence of each option in plain words. Recorded decisions are quoted.
9. **Keep the style plain:** short sentences, lists rather than long paragraphs, tables for comparisons, and each abbreviation defined once.
10. **Figures and tables are numbered and captioned,** and every figure is referred to in the text.

---

## 4. Templates

### Agent step report (an agent's final message)
```markdown
## <Task> step N (<agent>): <one-line subject>
**1. Status:** <done | blocked | needs a decision>, <one line>.
**2. Files changed** (snapshots in work/agentic/snap/<date>/<same path>):
- `<path>:<lines>`: <reason>
**3. Checks run:**
| Command | Log | Result |
|---|---|---|
| `<command>` | `work/agentic/logs/<log>` | <n passed, n failed (known: ...), 0 new> |
**4. Behaviour before and after:**
| Case | Before | After |
|---|---|---|
**5. For the docs keeper:** <exact facts: rows, codes, parameters with units and [EST]>
**6. Open points and questions:**
1. <what, and who decides>
```

### Lead's intro of an agent task report (`work/agentic/reports/src/<id>.md`)
The 11-section structure of the engineering report (section 2.7). The ATR generator appends appendices A-C and the references.
```markdown
## Summary
**<The result in one sentence.>** <What was done, by whom.>
- **<Key output>:** `<path>`.
**Cost:** <runs, tokens, against the estimate>.

## 1. Purpose and scope
**Purpose.** <Why the task exists.> **In scope:** <...> **Out of scope:** <...>

## 2. Background and the request
<The user's words and the date; what led to the task; the order of work.>

## 3. Approach and rationale
- **<Choice>.** <Why.> *Alternative:* <...>. *Rejected:* <why>.

## 4. Organisation and method
| Step | Agent | What |
|---|---|---|
<Who did what, the rules (read only, batch, one pass), the checks.>

## 5. Work performed
1. **<Step>:** <what was done, the files, the decision rows>.

## 6. Results
![Figure 1: <caption, with what the axes show>](work/<path>.png)

Table 1. <caption>
| ... | ... |

## 7. Verification and validation
<Tests in counts (passed, failed, known, new); system runs; the independent review; what is not verified and the command that would verify it.>

## 8. Decisions and rationale
| Decision | By | Choice | Why |
|---|---|---|---|

## 9. Issues, risks and limitations
<Limits; what is not confirmed; slips and how they were repaired; corrections to earlier reports.>

## 10. Conclusions and recommendations
1. **<...>**

## 11. Cost and schedule
- **Period:** <date, start to end>.
- **Effort:** <runs, tokens, agent time> against <the estimate>.
```

### One review finding
```markdown
### <PFX>-NN <Title>
- **Severity:** <Critical | High | Mid | Low>. **Confidence:** <high | medium | low>.
- **Confirmed:** <yes | partly | no>. <What confirms it, or the exact command that would.>
- **Where:** `<path>:<lines>`; ...
- **What is wrong:** <the logic, in plain steps>
- **Why it matters:** <the effect on results or on the real product>
- **Evidence:** <file:line quotes, log lines, numbers>
- **Suggested action:** <steps>. Size <S | M | L>.
- **Related:** <other IDs, decisions, open problems>
```

### Ranked-list row and the decision list
```markdown
| R-01 | Critical (<rank change and why>) | <title> | <SRC-01, SYS-01> | <effect> | <action>. <size> | <owners> | <yes/partly/no> | <yes/no> |

### Decisions for the user
1. **<Topic> (R-xx).** The options: (a) <...> (recommended); (b) <...>.
```

### Status item
```markdown
16x. **<Title>** (<date>, <reference>). <Cause in one sentence.> <Consequence.> Next: <step, owner>.
```
