# Agentic workflow helpers

## The agent team (`.claude/agents/`, agreed with the user on 2026-11-12)

The lead is the main Claude session. It talks with you, splits a task, picks the agents, merges their results, asks for your approvals and keeps the dashboard. Every agent first reads `tools/agentic/AGENT_RULES.md`.

**One session per team member (since 2026-11-16, your request).** Each agent below works in its own Claude Code session, linked to the lead's, so you can open it from the sidebar, follow its work and write to it. The lead starts a member's session with its first task and sends its later tasks to the same session, which keeps its context. `tools/agentic/sessions.json` lists the sessions by role. A member's session ends its turn with its report in a file; the app then wakes the lead.

**Subsystem engineers.** Each diagnoses, changes, and writes requirements and tests in its own files only.

| Agent | Owns |
|---|---|
| `shaddad-mcu` | Shaddad_MCU firmware, its parameters, tests and code generation |
| `shaddad-bms-power` | Shaddad_BMS and the plant's power domain |
| `shaddad-drive` | the drive domain |
| `shaddad-structure` | the vehicle mechanics, the implements, the world and terrain |
| `shaddad-electronics-comms` | the electronics plant, CAN1/CAN2, the RC link, the operator console, the row test |
| `shaddad-sensors-nav` | the sensor models, NavFusion and Perception |
| `shaddad-gnc` | Shaddad_GNC and the Companion |
| `shaddad-integration` | the top level, the plant root, the monitors, the buses, the scenarios, the layout engine |

**Specialists:**

| Agent | Job | Model |
|---|---|---|
| `shaddad-tester` | runs tests in batch and sorts them against the baseline | Haiku |
| `shaddad-reviewer` | independent read-only review of each change | Opus |
| `shaddad-log-analyst` | robot and simulation logs | Opus |
| `shaddad-report-writer` | NASL reports | Opus |
| `shaddad-safety` | the functional safety engineer: the safety plan, the hazard log, the safety concept, the FMEA and fault trees, the safety case (CR-002, 2026-11-16) | Opus |
| `shaddad-docs-keeper` | the ICD, specs, decisions, STATUS and dashboard notes | Haiku |

**How a task flows:**
- **Test-and-fix:** the tester runs and sorts; each new failure goes to its owning subsystem engineer, who fixes it in its builder; the tester reruns; the reviewer checks.
- **Change on request:** you approve the change; the owning engineer makes it, one engineer after another when it crosses a border; integration checks the top level; the docs keeper updates the docs; the tester runs the tests; the reviewer checks.
- **Requirements to tests:** the owning engineer drafts the requirements; you review them; the engineer writes and runs the tests; the reviewer checks.

Only one agent works in your MATLAB at a time, the one the lead names in its brief, and batch runs go one heavy MATLAB at a time: `run_batch.sh` holds a lock, so sessions that run at the same time queue their heavy runs.

## The helpers

These helpers back the three project skills in `.claude/skills/`:
- `shaddad-test-fix`: run, sort and fix tests;
- `shaddad-model-change`: make model changes on request;
- `shaddad-req-tests`: turn the specs into requirements and model tests.

The skills sit on MathWorks' Simulink Agentic Toolkit and MATLAB Agentic Toolkit. The agent works in your own MATLAB once you connect it. Run this in each MATLAB session:

```matlab
addpath("C:\Users\aomar\.matlab\agentic-toolkits\simulink")
satk_initialize
```

`satk_initialize` shares the session and loads the toolkit's MATLAB side. Long runs go to separate batch MATLABs.

How it was installed on 2026-11-12:
- Installed: MCP server v0.16.0, MATLAB Agentic Toolkit 2026.11.a and Simulink Agentic Toolkit 2026.11.b, all under `~/.matlab/agentic-toolkits`, with telemetry off.
- Claude Code's `matlab` server uses `--matlab-session-mode=existing` and the Simulink tools file.
- Six skill groups (43 skills) are linked into `~/.claude/skills`: matlab-core, matlab-software-development, model-based-design-core, verification-validation-and-test, simulink-simulation and physical-modeling. To change them, run `setupAgenticToolkit("configure", SkillGroups=[...])`.
- In R2026b an add-on installed from a batch MATLAB is not kept after that MATLAB closes, so install or update add-ons from a normal (desktop or `-nodesktop -r`) MATLAB.

| File | What it does |
|---|---|
| `test_baseline.py` | `build` makes `tests/test_baseline.json` from run logs: every test's last result, plus a note per class on why its failures are known. `compare` sorts a new run into new failures, known failures and fixed tests. |
| `run_batch.sh` | Runs one MATLAB command in its own batch MATLAB with the project open. It waits until no other batch MATLAB is running and stops the run at a time cap. |
| `../../utilities/shaddad_run_tests.m` | Runs a test class, or tests picked by name or tag. It prints one line per test, in the format that `test_baseline.py` reads. |

Example, a whole class in the background:

```bash
bash tools/agentic/run_batch.sh 3600 work/agentic/logs/t_sensors.log "shaddad_run_tests('t_sensors')"
python tools/agentic/test_baseline.py compare work/agentic/logs/t_sensors.log
```
