# Shaddad UGV dashboard

A local web page that shows the state of the whole project, read live from its files on this computer.

- **Start:** double-click `start_dashboard.bat`, or run `python tools/dashboard/server.py` and open http://localhost:8765.
- **What it shows:** each subsystem (models, tests, open problems, build tasks), the plans in `docs/superpowers/plans`
  with their work logs, the latest logged result of every test class next to the Oct 23 acceptance, saved scenario
  results, model files, documents (with a reader for `docs/STATUS.md`), running MATLAB jobs, memory, disk and sleeps.
- **How it updates:** the server reads the project again every 30 seconds and the page asks for it every 20 seconds.
  It listens on 127.0.0.1 only and never writes to the project.
- **Open items:** `notes.json` holds the items Claude keeps (decisions, queued work, findings). Edit it freely.
- **Journal:** `journal.jsonl`, one entry per line, shown in the Journal tab and on the overview. The server adds
  entries by itself when it sees a change between two readings: a MATLAB batch run that starts or ends, a new test
  result, a plan task that starts or finishes, a report written, model files changed, a sleep of the computer.
  People and tools add the rest:
  `python tools/dashboard/log.py <kind> "<text>" [--task T] [--area A] [--ref path]`, with kind one of start, done,
  result, ruling, decision, needs-you, resolved, issue, note. A `needs-you` entry counts under "Needs you" until a
  `resolved` entry with the same `--task` follows it.
- **Agents tab** (added 2026-11-12, redrawn the same day to the layout the user picked). From top to bottom:
  - **A board of the work:** columns Waiting, Working, Needs you and Done today, with a card per agent step, and a line naming the agents that are idle today;
  - **Today's runs as a timeline:** a lane per agent and a bar per run, colored by task (dashed while working). An arrow marks each handoff, from the latest run of a task that ended before the next one started; parallel runs get none. A line marks now, and hovering a bar gives its details;
  - **Every run:** a table of all runs.

  The data is `agents.jsonl`. The lead writes it with:
  - `python tools/dashboard/agents.py task --id ID --title "T" --steps "agent: what; ..."` when a task starts;
  - `start --task ID --agent NAME --what "W" [--step N]` when it dispatches an agent;
  - `done --run RUN --status done --tokens N --seconds S --summary "S"` when the agent reports;
  - `step` for a step without an agent run;
  - `end` when the task ends.

  The page reads `/api/agents` every 4 seconds, so the tab follows the agents within seconds.
- **Reports:** every task report and decision file of the plans, the layout reports, `docs/STATUS.md` and the HTML
  regression reports in `work/reports`, with a reader.
- **Up next:** the running and waiting tasks of every unfinished plan, and the open items.

Files: `server.py` (the web server), `collect.py` (reads the project; `python collect.py` prints a summary),
`static/index.html` (the page), `notes.json`. It needs only Python 3 (standard library).

The look follows NASL's MARSAD x Fusion design system (the Marsad frontend): flat hairline panels, square corners,
navy marker labels, the NASL purple-to-lilac gradient on accents and progress bars, Effra type and Material Symbols
icons. The fonts load from the internet; offline, the page falls back to the system font and hides the icons. The
theme button cycles auto, light and dark.
