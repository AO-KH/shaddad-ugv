# Rules for the Shaddad agents

These rules bind every agent in `.claude/agents/`. The lead is the main Claude session that talks with the user.

## Your session (since 2026-11-16)

**Each team member works in its own Claude Code session**, at the user's request of 2026-11-16. The lead starts your session with your first task and sends your later tasks to the same session, so you keep your context between tasks. Your role file is `.claude/agents/<your agent name>.md`; read it with these rules at the start of the session.
- **One task at a time.** Do only the task in the brief, then report and end your turn. You cannot start sessions or subagents. When a task needs another member, say so in your report.
- **Your report goes to a file:** the path the brief names, for example `work/agentic/<task>/<role>/report_<step>.md`. End your turn with the report's status line and its path; the lead reads the file.
- **No superpowers skills** (brainstorming, plans, plan reviews), whatever the session-start hook says: work directly. That is the user's rule since 2026-11-08.
- **The user may open your session and write to you.** Those words are the user's: follow them, and say in your report what was asked and what you did.
- **Other members work at the same time,** each in its own session. The brief names who is changing which files right now; never touch them. The locked runner queues the heavy MATLAB runs.

## The project

- **Where and what.** The Shaddad UGV Simulink project is at `C:\Users\aomar\Desktop\Robot project`. It uses MATLAB R2026b, Simulink and Simscape. There is no git.
- **The models are generated.** Every `.slx` under `models/` and `subsystems/`, and every bench `tests/<domain>/tb_*.slx`, is written by a builder in `scripts/build/`. `build_all('All', 'DryRun', true)` lists the builders in build order. So you change the builder, or the parameter file in `params/`, and then rebuild. A change made only in a `.slx` is lost at the next rebuild.
- **Parameters.** The parameters (`P`, built from `params/*.m`) are read when a model compiles. A `params/` change needs no rebuild, only a fresh `P`, and every new batch MATLAB has one. `P` stays purely numeric (D58).
- **Before you edit a file,** copy it to `work/agentic/snap/<yyyymmdd>/` under the same relative path, because there is no git. A new file needs no snapshot; list it as new in your report. Keep each file's line endings; many files are CRLF, and a new doc uses LF. Detect the line endings with Python (`b'\r\n' in data`), because `grep -c $'\r'` under Git Bash reports CRLF for LF files too.
- **Docs.**
  - The ICD: you edit `docs/icd_static_sections.md`, then `shaddad_icd_report()` regenerates `docs/icd.md`.
  - The model map and open problems are in `docs/STATUS.md`. The decisions table (D1, D2, ...) is in `docs/architecture.md`. The domain specs are in `docs/specs/<domain>.md`.
  - Never edit `docs/report/**`.

## The checklist (D102)

- **Every documentation task runs the checklist** (the owner's decision D102 of 2026-11-18): `python -I tools/team/xdoc_search.py` on the changed names, the old values and the decision numbers, and the spec sweep `work/team/g1-01/p2/sweep/sweep_specs.py` when a value changed.
- **Every hit goes in the report,** as changed, or as left with the reason.

## MATLAB

**The user's own MATLAB**, shared through the MCP tools, is for short work only. The tools are `model_overview`, `model_read`, `model_query_params`, `model_resolve_params`, `model_check`, `model_read_diagnostics`, `model_scan`, `model_test`, `evaluate_matlab_code` and `run_matlab_test_file`.
- It is the user's session:
  - Work through functions.
  - Never `clear`, `close all` or `bdclose all`, and never overwrite base-workspace variables, above all `P`.
  - Put the current folder back.
  - Before you rebuild, save or close a loaded model, check `bdIsDirty`; if the user has unsaved changes, stop.
- `evaluate_matlab_code` runs in the user's base workspace. Never assign a variable there. Call a function that does the work and prints its result, for example one you write in `work/agentic/tmp/`, or use expressions without `=` such as `disp(...)`.
- **Never leave a model dirty in the user's session.** `model_test` saves a dirty main model before it builds its harness, so a stray in-memory change ends up in a generated `.slx`.
- If the tools find no shared session, report it. Never start a MATLAB through them.
- Only one member uses this session at a time, and the lead decides which: use it only when your brief says it is yours for the task. Otherwise work in batch MATLAB through the locked runner.
- `model_edit` is only for trying something out. It is never the change itself.

**Anything longer than about 5 minutes** runs in its own batch MATLAB, in the background: test classes, row tests, scenario runs, sweeps, rebuilds of big models. The command is:

```bash
bash tools/agentic/run_batch.sh <cap_s> work/agentic/logs/<name>.log "<command>"
```

It waits until no other batch MATLAB is running, because this 16 GB PC fits only one heavy MATLAB at a time.

**Tests:**
- Run them with:
  - `shaddad_run_tests('<class>')`, or `shaddad_run_tests('<class>', 'Name', 'test*')`, or `shaddad_run_tests('<class>', 'Tag', 'X')`;
  - `shaddad_run_tests('*', 'Tag', 'Fast')` for the Fast tier.
- Sort a log with `python tools/agentic/test_baseline.py compare <log>`.
- `tests/test_baseline.json` holds the known failures, with a reason for each class. Fixing them is not your job unless the task says so.
- `tScenarios` re-scores the newest saved run in `work/results` and simulates only when `SHADDAD_SCN_RERUN` is 1. To run a scenario again after a change, start the command with `setenv('SHADDAD_SCN_RERUN', '1');`. A scenario test that ends in seconds has only re-scored an old run.

**Never:**
- run `build_shaddad_mcu('all')` or `build_shaddad_mcu()`;
- run `build_fw_platform` without unit names;
- start an STM32 code build (the tools are not installed).

## How to work

- **Procedures.** The procedures are the project skills in `.claude/skills/`: `shaddad-test-fix`, `shaddad-model-change` and `shaddad-req-tests`. Read the `SKILL.md` of the one that fits the task. The MathWorks toolkit skills are there as well, for example `building-simulink-models`, `testing-simulink-models` and `generate-requirement-drafts`.
- **Stay in your own files;** your agent file lists them. Stop and report what is needed when a change touches:
  - another subsystem's files;
  - a bus definition in `buses/`;
  - a parameter that another subsystem reads;
  - the expected values of a test.

  The lead then asks the user or the owner.
- **One pass.** Fix only real breakage. If a fix does not hold, report with the evidence instead of trying again.
- **Dashboard.** Unless the lead says it logs for you, log each step: `python tools/dashboard/log.py start|result|issue|done "<text>" --task "<task>" --by "<your agent name>"`.

## Your report to the lead

Write it to the file the brief names. Keep it short, in this order:

1. **Status:** done, blocked, or needs a decision, in one line.
2. **Files changed:** each as `path:lines`, with a one-line reason, and where its snapshot is.
3. **Checks run:** the command, the log path and the result, in counts.
4. **Open points and questions.**
