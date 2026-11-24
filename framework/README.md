# NASL Engineering Management System: the framework

The framework is how NASL runs every engineering project: one handbook, one set of templates, and the tools, the same for every project. Each project is an instance made with it. Shaddad UGV (code SHD) is the first one.

**Start here:** the Engineering Handbook, NASL-EMS-HBK-001.
- Source: `framework/handbook/engineering_handbook.md`.
- PDF: `framework/build/NASL_EMS-HBK-001_Engineering_Handbook_v1.0.pdf`.

## What is in the kit

| Part | Where | What |
|---|---|---|
| Handbook | `framework/handbook/` | the handbook source, its figures (`figures.py` draws them) |
| Templates | `framework/templates/` | project config, project plan, controlled document, task card, gate success criteria, gate readiness checklist, gate review report, change request, decision record, nonconformance report, risk register, requirements register, test procedure and report, engineering report, work brief and step report, lessons learned |
| Procedures | `framework/procedures/` | step-by-step procedures: NASL-EMS-PRO-001, making an engineering report |
| Framework tools | `framework/tools/` | `extract_gate_criteria.py` (the gate criteria JSON from the handbook); `index_decisions.py` (the decision log index from the decisions table) |
| Document tools | `tools/reports/` | `build_document.py` (any controlled document), `build_task_cards.py` (the task-card PDF), `build_engineering_report.py` (task-group reports), `nasl_docx.py` (the NASL Word look), the Word template, `convert_to_pdf.ps1`, `pdf_pages_to_png.ps1` (page checks) |
| Dashboard | `tools/dashboard/` | the live status page: Project (gates, readiness, cards), Team, Journal, Reports and more; `log.py` for the journal, and the work log |
| Team kit | the team kit folder (today `tools/agentic/`) | the team playbook, the work rules, the dashboard style, the reports guide, the locked batch runner, the test baseline tool, the record saver |
| Built documents | `framework/build/` | the generated PDFs (never edited by hand) |

## Start a new project (handbook chapter 12)

1. Copy `framework/`, `tools/reports/`, `tools/dashboard/` and the team kit folder into the new project folder.
2. Copy `framework/templates/project_config.json` to `project.json` at the root, and fill it in.
3. Write the charter and the project plan (`framework/templates/project_plan.md`). Hold G0.
4. Create the cards file with the gates (criteria from `framework/templates/gate_success_criteria.json`) and the first cards (`framework/templates/task_card.json`). Set up the registers.
5. Set up the team from the team playbook (the roles, the work rules, the briefs), start the dashboard, and run the work as cards.

## Build the documents

```
python framework/handbook/figures.py
python tools/reports/build_document.py framework/handbook/engineering_handbook.md
python tools/reports/build_document.py --index
python tools/reports/build_task_cards.py
```

No installs: Python standard library, Microsoft Word (for the PDF) and Microsoft Edge (for SVG figures).

## Change the framework

The framework is itself under change control. A change to the handbook or a template:
1. comes from a lessons-learned item or a change request;
2. raises the handbook's version, with a history line in its header;
3. is rebuilt with the tools above.

Projects started earlier keep their version until their owner decides to move.
