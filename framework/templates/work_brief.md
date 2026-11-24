# <Task group> (card <id>), step <N>: brief for <role>

How to use this template (delete this paragraph):
- The lead writes one brief per engineer pass (NASL-EMS-HBK-001 section 6.2).
- The rules shared by all engineers of the task group go in a shared brief, `<work folder>/<task>/BRIEF_COMMON.md`. This brief points to it.
- Exact values (numbers, names, signatures) appear here once, verbatim.

## Read first
- `<shared brief>`, the team's work rules.
- `<files, reports and logs that bear on the change>`.

## The request
<The owner's words and the decisions already taken, with dates.>

## The change
<What to change, at which generator (builder, parameter file, source), with the exact values. What will behave differently.>

## The rules for this pass
- **Edit only:** `<your files>`.
- **Snapshot first:** `<work folder>/snap/<date>/<same path>`.
- **Heavy runs** through the locked batch runner (`run_batch.sh <cap_s> <log> "<command>"`), one at a time.
- **One pass.** Stop and report when the change needs another owner's file, an interface change or a test's expected value.

## Who else is working now (do not touch their files)
- <engineer>: <files>.

## Checks to run
<Structure, interfaces, layout, the subsystem's tests sorted against the baseline; the expected result.>

## Budget
<Size: S, M, L or XL.>

## Your report
In the step-report shape (`framework/templates/agent_step_report.md`). Your final message is the report; the lead saves it word for word.
