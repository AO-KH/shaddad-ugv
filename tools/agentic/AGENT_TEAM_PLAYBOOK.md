# Agent-team workflow for engineering projects: a playbook

> **Since 2026-11-15 this playbook sits under the NASL Engineering Handbook NASL-EMS-HBK-001** (`framework/handbook/engineering_handbook.md`, the kit in `framework/`). The handbook sets the life cycle and gates, the task cards, change control, requirements, risks and documents. This playbook is its chapter on the agent team, in detail.

**What it is.** A way to run technical work with Claude Code as a small engineering team:
- one **lead**: the main session you talk to;
- one **engineer agent per subsystem**;
- a few **specialists**: integration, tester, reviewer, docs keeper, log analyst and report writer.

A local **dashboard** shows the work live:
- a kanban of the agents' steps and a list of the team;
- every agent run with its time and tokens;
- a journal of everything that happened;
- the open items and the reports.

The workflow came out of the Shaddad UGV Simulink project (Nov 2026). It works for any engineering project, with or without MATLAB/Simulink.

**How to start it in a new session.**
1. Copy this file, `DASHBOARD_STYLE.md` (the dashboard's look, without colours) and `REPORTS_GUIDE.md` (the Reports section, the PDF engineering reports, the task cards, and how to write each report) into the new project, for example to `tools/agentic/`. For the PDF reports, also copy `tools/reports/` from the reference project.
2. Paste this into the session:

   > Read tools/agentic/AGENT_TEAM_PLAYBOOK.md and set up this workflow for this project. Start with "2. Set it up", step 1: map the project into subsystems, and show me the proposed team table before you write any agent file.

**Reference implementation.** Every file named here exists in `C:\Users\aomar\Desktop\Robot project` on this PC. Copy and adapt from there when the new project is on the same PC. Paths like `tools/dashboard/` are relative to that folder.

---

## 1. The team

```
                         you
                          |
        lead (main session)  ----  dashboard (localhost): kanban, runs, journal, notes, reports
                          |
   +--------------+--------------+----------------+---------------------------------------+
   |              |              |                |                                       |
 engineer A    engineer B    engineer C     integration engineer              specialists
 (subsystem)   (subsystem)   (subsystem)    (top level, interfaces,           tester, reviewer (read only),
 own files     own files     own files       scenarios, system tests)         docs keeper, log analyst,
                                                                              report writer
```

| Role | Does | Never | Model |
|---|---|---|---|
| **Lead** (main session) | Talks with you; splits each task; picks the agents; starts each member's session and sends it its tasks; writes their briefs; answers their questions with rulings; merges their results; asks you for the real decisions; keeps the dashboard, the notes and memory | Big file sweeps itself (it delegates them); edits an engineer's files while that engineer runs | the session's |
| **Subsystem engineer**, one per subsystem | Diagnoses, changes, tests and writes requirements in its own files only, one task at a time, then reports | Touches another subsystem's files without the lead's OK; changes a test's expected values without approval; starts other agents | most capable |
| **Integration engineer** | The top level, the interfaces between subsystems, scenarios and system tests; checks any change that crosses a border | Changes a subsystem's internals | most capable |
| **Tester** | Runs tests and sorts the results against the known-failure baseline | Fixes anything | cheap |
| **Reviewer** | Independent, read-only check of a finished change: the diff against the snapshots, the logs and the docs, then a verdict: keep, fix, or ask the user | Edits anything; asks for the engineer's reasoning | most capable |
| **Docs keeper** | Brings the docs in line with a change, from the engineers' "for the docs keeper" sections | Edits code or models | cheap with a checklist; capable for wide sweeps |
| **Log analyst** | Analyses run logs, compares runs, finds when and why a signal goes wrong, makes plots | Edits project files | capable |
| **Report writer** | Writes formal reports (Word or PDF, house branding) with figures from the lead's results | Invents numbers | capable |
| **Functional safety engineer** (Shaddad CR-002) | Leads the safety life cycle: the safety plan, the hazard log, the safety concept, the FMEA and fault trees, the safety case | Designs or changes the safety functions it assesses (independence) | most capable |

**Models.** In the Shaddad sessions, Sonnet and Fable subagents hit safety false positives, while Opus and Haiku worked. Pick the models that work in your environment, and give the model explicitly at every dispatch.

**One session per member.** Since 2026-11-16 (the Shaddad user's request), every member of the team works in its own Claude Code session, linked to the lead's, instead of as a background subagent:
- **The user sees the team at work.** Each member is a row in the sidebar. The user can open it, follow it, write to it and stop it.
- **A member keeps its context.** Its first task starts its session; its later tasks go to the same session as a message, so the second brief can be short.
- **Reports are files.** A member writes its report where the brief says and ends its turn; the app wakes the lead.
- **Subagents stay** for one-off, read-only sweeps the user does not need to see.

The registry `tools/agentic/sessions.json` maps each role to its session (name, id, model, started). The lead keeps it current.

---

## 2. Set it up in a new project

**Step 1: map the project.** Read the code, the docs and the build and test setup, then propose a team table, one row per subsystem engineer, and show it to the user before writing anything. For each subsystem give:
- its owned files (source, parameters, builders or build files, tests, spec);
- how to build it and how to test it;
- its borders with the other subsystems;
- the long-running tools it needs.

Add the specialists from section 1.

**Step 2: agent files.** Write one `.claude/agents/<prefix>-<name>.md` per role (Template A).
- The `description` says when to use the agent. Don't put ": " inside a plain YAML description.
- The body says what the agent owns, and the facts it would otherwise spend tokens digging for: the key decisions, the known failures, how long the tests take.
- New project agents can take about 20 minutes to be found in the same session. Meanwhile, use `general-purpose` with the line "act as <agent>, read .claude/agents/<file>".

**Step 3: shared rules.** Write `tools/agentic/AGENT_RULES.md` (Template B). Every agent reads it first.

**Step 4: helpers** (copy from the reference implementation and adapt):
- `tools/agentic/run_batch.sh`: runs one long or heavy job at a time, with a lock, a time cap and a log (Template C). Use it for anything over about 5 minutes or anything memory-heavy.
- `tools/agentic/test_baseline.py`:
  - `build [--merge] <logs> -o tests/test_baseline.json` keeps every test's last result, plus a note per class on why its failures are known;
  - `compare <log>` sorts a new run into new failures, known failures and fixed tests.

  It needs the test runner to print one line per test (status, name, seconds) and a summary line per class. Adapt the regexes to your runner (pytest, ctest, MATLAB).
- `tools/agentic/save_agent_result.py <agentId> <dest.md> --header "..."` saves an agent's final message as a file. Subagents often can't write report files themselves. It reads the transcript `~/.claude/projects/*/<session>/subagents/agent-<id>.jsonl` and takes the last assistant message with text. If the agent's last message is only a short note, save the real report from its notification by hand.
- Procedures as project skills in `.claude/skills/<prefix>-<procedure>/SKILL.md`, one per kind of work:
  - **test-and-fix:** run, sort against the baseline, route each new failure to its owner, fix the cause once, rerun, report;
  - **change on request:** restate, inspect, find where the change really lives (the generator or builder, not the generated file), ask first when it ripples, edit, rebuild, check, update the docs, report;
  - **requirements to tests:** draft the requirements from the specs, have the user review them, write the tests, run them, report.

**Step 5: the dashboard.** See section 5. Copy `tools/dashboard/` and adapt its collector, or build the minimal version described there. Give it the look in `DASHBOARD_STYLE.md`, in the colour theme the user describes. Add a launch entry so the session can open it (`.claude/launch.json`, name `<project>-dashboard`, port 8765).

**Step 6: record the user's standing rules** in memory: what must never be run, which folders are never edited, the cost limits, when to ask, and the report conventions.

---

## 3. How the lead runs a task

### 3.1 Before anyone starts
1. **Restate** the request in a sentence or two. Look for earlier work on it: the dashboard notes and journal, the open problems, old reports, earlier session notes.
2. **Separate the decisions that are the user's** from the details the lead can rule on.
   - The user's: anything that changes what the product does, safety behaviour, test expected values, interfaces other teams depend on, cost well beyond the estimate.
   - Ask with the recommended option first, each option with its consequence in plain words. One focused question at a time.
   - Log the answer: `log.py decision "..."`.
3. **Estimate the cost** in agent runs and tokens, and say it. A capable-model engineer run costs about 200-700 k tokens. A cheap tester run costs about 20-50 k.

### 3.2 Plan and brief
1. **Split the work into steps,** one per agent. Steps run in parallel when they touch disjoint files. They run in order when one needs another's result, or when they share a border.
2. **Create the dashboard task:** `agents.py task --id <id> --title "..." --steps "agent: what; agent: what; user: decision; lead: step"`.
3. **Write a shared brief file** for the task (Template D): the rules for this task, where the findings are, the report format.
4. **Write one brief per agent** (Template E):
   - the exact change, with `file:line` pointers;
   - the decisions already made;
   - the checks to run, named one by one (layout and style checks included);
   - **who else is changing which files right now.**

   Precise briefs are the biggest cost lever. A brief with exact checks cost 361 k tokens, against 448 k for a vague one.

### 3.3 Dispatch
- **A member's first task starts its session** (`start_session`):
  - `initiation` "user_asked", since the user asked for one session per member;
  - `context` "fresh", with a `background` of the project and the task;
  - `use_worktree` false when there is no git: the members share the folder and keep to their own files;
  - `name` the role's handle (`mcu-engineer`) and `title` the role ("MCU firmware engineer");
  - the model: omit it to inherit the lead's capable model; give a cheaper one for the tester and the docs keeper;
  - the brief (Template E), opening with "You are the <role>. Read `.claude/agents/<file>` and `tools/agentic/AGENT_RULES.md` first", and naming the report file.
- **Its later tasks go to the same session:** `SendMessage` to its `local_…` id, with the new brief. Never start a second session for a role that has one.
- **Parallel members** start in one message, when their files are disjoint. The locked runner still queues their heavy runs.
- Record each session in `tools/agentic/sessions.json`, and register each run at once: `run=$(python tools/dashboard/agents.py start --task <id> --agent <name> --what "..." --step N --model opus)`.
- Request keep-awake for long work, so the PC doesn't sleep under a batch run.

### 3.4 While the agents work
- **Never predict results.** Report only what a notification or a log shows.
- **Answer the agents' questions with rulings,** and log them: `log.py ruling "..."`.
  - Approve what is the lead's to approve: an internal interface, a bit or enum reuse, a test expectation that encoded the defect being fixed. Say "lead-approved" in the ruling, and have the agent list it under its own heading.
  - Take to the user only what is theirs.
- **Relay facts between agents** when one agent's change affects another's checks. Send it to the affected member's session with SendMessage. A message costs the receiving member a turn, so send only what it needs.
- **The user may write to a member directly** in its session. Those words are the user's: the member follows them and reports them, and the lead reads that report before the member's next task.
- **Watch for tool collisions:** two heavy jobs at once, two runs writing the same log, a builder that deletes its output before failing. Tell the agent concerned.
- **Keep the user posted** in a line or two per notification, in plain text: what finished, with numbers, and what is still running.

### 3.5 When an agent reports
1. **Read the report:** a member's session writes its own report file, where the brief said. Check that it is the report and not a closing note. For a subagent, save it: `python tools/agentic/save_agent_result.py <agentId> work/agentic/<task>/<agent>/report.md --header "..."`.
2. **Close the run:** `agents.py done --run <run> --tokens <N> --seconds <S> --summary "..."`. Take the seconds from the session's turn (or the agent's usage), and the tokens when the app reports them.
3. **Log the result in numbers:** `log.py result "before X, after Y; tests a/b, 0 new failures" --task "..." --ref <report>`.
4. **Merge the test results:** `test_baseline.py build --merge <log> -o tests/test_baseline.json`. Then check the class counts. A log of several classes once gave one class another's summary.
5. **Rule on the open points.** Do the small lead items yourself (an allowance with its reason, a label), and give the next step its context.

### 3.6 Closing the task
1. **System check:** the integration engineer runs the cross-border checks and the system scenarios for the whole change.
2. **Docs:** the docs keeper works from the engineers' "for the docs keeper" sections. That is a decision row, interface rows, the spec and STATUS.
3. **Tests:** the tester runs the changed classes and the fast tier, sorted against the baseline.
4. **One review:** the reviewer gets the task as asked, the changed files with their snapshots, and the logs, but not the engineers' reasoning. Fix only what it calls real breakage, once.
5. **The task report** (shapes and rules in `REPORTS_GUIDE.md`):
   - write the lead's account in `work/agentic/reports/src/<id>.md` (Template G);
   - run `python tools/dashboard/task_report.py <id> --journal "<journal task name>" --intro <src> --ref <reports>`;
   - add that line to `work/agentic/reports/src/regenerate.sh`;
   - **for a task group** (several steps or agents): the PDF engineering report, built from the ATR with `tools/reports/build_engineering_report.py <id>`, every page checked, and sent to the user (`REPORTS_GUIDE.md` section 2.7). This is a standing rule; don't wait to be asked.
6. **Close the dashboard task:** `agents.py end --task <id> --summary "..."`; set the task's card in `tools/dashboard/project_cards.json` to done with its report (`REPORTS_GUIDE.md` section 2.8); then update `notes.json` and memory.
7. **Close with the user:** what changed, the numbers, what is not done or not verified, the cost against the estimate, and the decisions still open.

---

## 4. Rules that keep it cheap and safe

1. **One pass.** Each task gets one implementer pass, accepted when its own checks pass. Fix real breakage once, with the same agent. No repeated review-and-fix rounds for polish (the user's rule, after rounds that moved a crossing count from 480 to 466).
2. **The owner edits.** Only the owning agent edits a file. Stop and report when a change needs another owner's file, a shared interface, or a test's expected values.
3. **Snapshots before edits** when there is no git: `work/agentic/snap/<yyyymmdd>/<same relative path>`. With git, commit per step on a branch instead.
4. **Generated files are changed at their generator.** Edit the builder or the parameter file, then rebuild. A change made only in a generated file is lost at the next build.
5. **Heavy tools one at a time,** through the locked runner, with a time cap and a log per run. The memory of a 16 GB PC fits one heavy MATLAB.
6. **Agents don't start agents or sessions,** and don't spawn their own reviewers. The lead dispatches the review, in the reviewer's own session.
6a. **One session per role, kept.** A member's next task goes to its existing session. Archive a member's session only when the user asks.
7. **Ask the user only what is theirs,** with the recommendation first and the consequences in plain words. Record every answer, so no later turn reopens it.
8. **Count the tokens of every run,** compare them with the estimate, and tell the user as soon as the estimate is passed. Drop optional steps rather than overspend, and say which.
9. **No plan documents or plan reviews unless the user asks;** the dashboard task is the plan. This was the Shaddad user's rule.
10. **Never:** change security settings, push or publish without asking, put personal data into external requests, or edit folders the user has closed. In Shaddad these were `docs/report/**` and the user's own MATLAB workspace.
11. **When the user works remotely,** end every turn with a plain-text summary they can read on the phone.

---

## 5. The dashboard

A local web page (Python standard library only, `127.0.0.1:8765`) that reads the project's files every 30 s and never writes to the project. The reference is `tools/dashboard/`:

| File | Role | Generic? |
|---|---|---|
| `server.py` | web server: `/` page, `/api/status`, `/api/agents`, `/api/logs`, `/api/text?path=` (a text file for the reader), `/raw/<path>` (images and HTML reports) | yes |
| `journal.py`, `log.py`, `journal.jsonl` | the journal: one JSON line per entry. The server adds entries by itself when it sees a change between two readings (a batch run that starts or ends, a new test result, a report written, model files changed, a sleep of the PC) | yes |
| `agents.py`, `agents.jsonl` | the agent work: `task`, `start` (prints the run id), `done`, `step`, `end`, plus the team list with model, runs and tokens | yes; edit the team list |
| `notes.json` | open items: `{id, item, area, state, kind, since, next}`. State is one of In progress, Your call, Proposed, Done. Kind `you` marks a decision for the user | yes |
| `collect.py` | reads the project: subsystems, test results, models, plans, reports (REPORT_GLOBS with categories), STATUS, running jobs, memory and disk | **project-specific: adapt** |
| `static/index.html` | the page: Overview, Project, Agents, Reports, Journal and project tabs | Project, Agents, Reports and Journal are generic |
| `task_report.py` | engineering report per agent task: a header table, the lead's intro, auto SVG figures (run timeline, tokens by agent), the steps, the runs, the journal and references | yes |
| `snapshot.py` | one self-contained HTML file of the dashboard, with the API data built in, to send as an attachment | yes |
| `project_cards.json` | the project's task cards under the life-cycle gates (the Engineering Handbook NASL-EMS-HBK-001), shown on the Project tab: a gate strip with progress, filters by gate and status, and each card's purpose, owner, deliverables, acceptance and links. The project is done when every card is done and every gate is passed | yes; write the cards per project |
| `tools/reports/` | the PDF engineering report pipeline: `build_engineering_report.py`, `nasl_docx.py` (the organisation's Word template), `engineering_reports.json`, `convert_to_pdf.ps1`, `pdf_pages_to_png.ps1` | yes; swap the template for the organisation's |

**The Agents tab,** as the Shaddad user chose it:
- a kanban with columns To do, In progress, In review, Needs you and Done; a card is an engineer step together with the reviewer step that follows it; "Show N more" past 6 cards;
- the team list, grouped into engineers and specialists;
- a timeline with one lane per agent: bars colored by task, handoff arrows and a now line;
- a table of every run.

The page polls `/api/agents` every 4 s.

**Logging** (do it for every step, in plain words): `python tools/dashboard/log.py <kind> "<text>" --task "<task name>" [--ref <path>] [--by <agent>]`.
- The kinds: start, done, result, ruling, decision, needs-you, resolved, issue, note, run, report, change, sleep.
- A `needs-you` counts under "Needs you" until a `resolved` with the same `--task` follows it.

**Reports section:** sorted by category (`REPORT_CATS` in index.html). A new kind of report needs a category there, or it lands in Other. Markdown reports render figures written as `![Figure n: caption](path)`. How the section finds and groups reports, and how each kind of report is written, is in `REPORTS_GUIDE.md`.

**Restarts:** after editing `collect.py`, `journal.py`, `agents.py` or `server.py`, restart the server. After editing `index.html`, reload the page.

**Opening it:** preview_start `<project>-dashboard` shows it in the app's side pane. To open it in the PC's own browser, run `Start-Process "http://localhost:8765/#agents"`.

**Minimal version,** if you build from scratch: `server.py` with the endpoints above, `journal.jsonl` plus `log.py`, `agents.jsonl` plus `agents.py`, `notes.json`, and one page with the Agents, Journal, Notes and Reports tabs. Add the project panels later.

### 5.1 Look and feel

The dashboard's layout, type and components are in `DASHBOARD_STYLE.md` next to this file. It sets no colours: the user describes the colour theme in the session.

---

## 6. When the project uses MATLAB and Simulink

- **Tools.** The MATLAB MCP server, with the Simulink Agentic Toolkit tools (model_overview, model_read, model_query_params, model_resolve_params, model_scan, model_check, model_test, model_read_diagnostics) and `evaluate_matlab_code`.
  - The user shares their session with `shareMATLABSession()` or `satk_initialize`, and the server runs with `--matlab-session-mode=existing`, so it never starts its own MATLAB.
- **The user's MATLAB session** is for short work only. Never `clear`, `close all` or `bdclose all`. Never assign variables in the base workspace: call a function that prints. Never leave a model dirty, because model_test saves dirty models.
- **Long work** runs in a batch MATLAB through the locked runner. That covers test classes, rebuilds, scenario runs and sweeps.
  - The command needs `addpath` for scripts under `work/`.
  - A batch MATLAB builds fresh parameters at start.
- **Models are generated by builder scripts:** change the builder, then rebuild only the builders involved. `model_edit` is for trying things out only.
- **After a builder change:** the layout check, the interface check, a compile (`set_param(m, 'SimulationCommand', 'update')`), the algebraic loops when the top level changed, then the domain's tests.
- **Known R2026b traps:**
  - add-ons installed from `matlab -batch` aren't kept;
  - MATLAB Function blocks nested deep in subsystem references need their parameters on inputs (a JIT relink assertion);
  - Windows Code Integrity can block unsigned MEX files (accelerator mode). That is a security setting: report it, don't change it.

**Other stacks.**

| MATLAB piece | Python / C / embedded equivalent |
|---|---|
| test classes | pytest or ctest suites |
| builders | code generators, CMake |
| model checks | linters and static analysis |
| batch MATLAB | the same locked runner around any long command, such as builds, simulations, FEA/CFD solvers or hardware-in-the-loop rigs |

---

## 7. Templates

### A. Agent file (`.claude/agents/<prefix>-<name>.md`)
```markdown
---
name: <prefix>-<name>
description: <Role> of the <project>. Owns <subsystem: parts, files>, with its parameters, tests and spec. Use it for a failure, a change, requirements or tests in <subsystem>. The lead gives it one task at a time.
model: opus
---

You are the <role> of the <project>. Read `tools/agentic/AGENT_RULES.md` first: its rules bind you.

## What you own
- Source: <paths>
- Build: <builder or build file>
- Parameters: <paths>
- Tests: <paths>; the class takes about <N> min
- Spec: <path>

## Know this
- <the decisions that bind this subsystem, by number>
- <the borders: what reaches you from which subsystem, and who owns it>
- <the known failures, and why they are known>
```
For the reviewer, add `tools:` with read-only tools only.

### B. Shared rules (`tools/agentic/AGENT_RULES.md`)
```markdown
# Rules for the <project> agents
You work in your own session; the lead starts it with your first task and sends your later tasks to it.
Do only the task in hand, write your report to the file the brief names, then end your turn.
You cannot start sessions or other agents. The user may write to you in your session: follow it and report it.

## The project
- Where it is, the language and toolchain, and whether there is git.
- What is generated, and from what (change the generator, then rebuild).
- Snapshots before editing: work/agentic/snap/<yyyymmdd>/<relative path>. Keep each file's line endings.
- The docs: which files hold the interfaces, the decisions, the status and the specs. Which folders are never edited.

## Tools
- Short work: <interactive tools and how to behave in the user's session>.
- Long or heavy work: bash tools/agentic/run_batch.sh <cap_s> work/agentic/logs/<name>.log "<command>" (one at a time).
- Tests: <how to run a class, a name, a tag>; sort a log with python tools/agentic/test_baseline.py compare <log>.
- Never: <the commands that must never run>.

## How to work
- The procedures are the project skills in .claude/skills/. Read the one that fits.
- Stay in your own files. Stop and report when a change needs another subsystem's files, a shared interface, or a test's expected values.
- One pass. Fix only real breakage. If a fix does not hold, report the evidence instead of trying again.
- Log each check's result: python tools/dashboard/log.py result "<text>" --task "<task>" --by <your name>.

## Your report to the lead
1. Status, in one line.  2. Files changed (path:lines, reason, snapshot).  3. Checks run (command, log, counts).
4. Behaviour before and after, in numbers.  5. For the docs keeper.  6. Open points and questions.
```

### C. Locked runner (`tools/agentic/run_batch.sh`)
The reference is the Shaddad version: it starts the job only when it holds `work/agentic/.batch_lock`, created with an atomic `mkdir`.
- The lock is stale, and removed, when its owner shell is gone (`kill -0`) or it is older than its cap plus 10 min.
- The job is killed at the cap.
- It logs the start, the end and the exit code.
- For MATLAB the command is `matlab.exe -batch "cd('<root>'); openProject('<prj>'); <cmd> % <marker>"`, and a run that hits the cap kills only the MATLAB whose command line carries `<marker>`.
- For other stacks, replace that line with the job. The waiting rule is "no other job holds the lock". Without the lock, two waiting callers start together.

### D. Shared brief of one task (`work/agentic/<task>/BRIEF_COMMON.md`)
```markdown
# <Task> (Agents task <id>): rules for every engineer
The user approved <what> on <date>. Findings: <files>. Read only the IDs your brief names.
- Tools: <batch only / shared session>; other engineers run at the same time, the runner queues the heavy jobs.
- Snapshots: work/agentic/snap/<date>/<path>. Stay in your files; your brief names the files others are changing now.
- Never: <list>. New tests are welcome; expected values of existing tests are not yours to change.
- Checks after a change: <style/layout check>, <compile/build>, <your new tests>, <the existing classes of the area>; compare with the baseline.
- One pass. Log each check's result yourself; the lead logs your start and end.
Report: status; files; checks; before/after in numbers; for the docs keeper; open points.
```

### E. Brief of one agent (the prompt)
```text
You are the <role> on Agents task <id> (step N), in your own session. First read .claude/agents/<file>.md, AGENT_RULES.md,
work/agentic/<task>/BRIEF_COMMON.md and the <procedure> skill. Write your report to work/agentic/<task>/<role>/report_<step>.md
and end your turn with its status line. (A later task to the same session only needs the new part.)
## The change (<finding IDs>)
1. <what is wrong today, with file:line> -> <what it must do after>; <the decision already made>.
2. ...
New tests: <names and what each must show>. Run: <classes>, <style check> for <domains>. Compare with the baseline.
## Who else is working now (don't touch their files)
- <agent>: <files>
Your report: the BRIEF_COMMON format.
```

### F. A decision question to the user
State the problem in one or two sentences: what happens today, and why it matters. Then give two options.
- **Option 1 (Recommended):** what changes, and its cost or consequence.
- **Option 2:** what changes, and its cost or consequence.

Record the answer as `log.py decision`, and in `notes.json` when it stays open.

### G. The lead's task report intro (`work/agentic/reports/src/<id>.md`)
The 11 sections of the engineering report; the full template is in `REPORTS_GUIDE.md` section 4.
```markdown
## Summary
## 1. Purpose and scope              (why the task exists; in and out of scope)
## 2. Background and the request     (the user's words with the date, the order of work)
## 3. Approach and rationale         (each choice, its alternatives, why they were rejected)
## 4. Organisation and method        (who did what, the rules, the checks)
## 5. Work performed
## 6. Results                        (tables in numbers; figures as ![Figure n: caption](path))
## 7. Verification and validation    (tests in counts, system runs, the review, what is not verified)
## 8. Decisions and rationale
## 9. Issues, risks and limitations
## 10. Conclusions and recommendations
## 11. Cost and schedule
```
