"""Derive build/build_manual.js (brief user manual) from build/build_report.js (full report)."""
import os
import re

B = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'build')
src = open(os.path.join(B, 'build_report.js'), encoding='utf-8').read()


def rep(old, new, count=1):
    global src
    if old not in src:
        raise SystemExit('pattern not found: ' + old[:80])
    src = src.replace(old, new, count)


# header comment
rep(" * Shaddad UGV Digital Twin - Phase 1 Demonstration Report and User Manual\n",
    " * Shaddad UGV Digital Twin - User Manual (Phase 1), brief edition.\n"
    " * Derived from build_report.js by manual/patch_builder.py; chapters in ../manual/um*.json\n")
# inputs / outputs
rep("const CH_DIR = path.join(REPORT_DIR, 'chapters');", "const CH_DIR = path.join(REPORT_DIR, 'manual');")
rep("const OUT_BASE = 'NASL_Report_Shaddad_Phase1_Demo_2026-10-23_v1';", "const OUT_BASE = 'NASL_UserManual_Shaddad_Phase1_2026-10-27_v1.2';")
src = re.sub(r"const CHAPTER_ORDER = \[[^\]]*\];",
             "const CHAPTER_ORDER = ['um01_start', 'um02_open', 'um03_drive', 'um04_scenarios', 'um05_live', 'um06_results', 'um07_params', 'um08_trouble'];",
             src, count=1, flags=re.S)
src = re.sub(r"const FIG_WIDTH_CM = \{[^}]*\};",
             "const FIG_WIDTH_CM = { 'F16_matlab_desktop_crop.png': 15.0, 'F23_d1_dashboard.png': 15.9, 'F35_live3d_side.png': 14.0, 'F37_live3d_sprayer.png': 13.0 };",
             src, count=1, flags=re.S)
rep("const TITLECASE_CHAPTERS = new Set(['ch00_front', 'ch03_structure', 'ch05_drive', 'ch06_electronics', 'ch07_phase2_preview']);",
    "const TITLECASE_CHAPTERS = new Set([]);")
rep("const IGNORE_PAGEBREAKS = new Set(['ch06_electronics']);", "const IGNORE_PAGEBREAKS = new Set([]);")
# no layout moves, no injected sign-off (the manual has its own)
src = re.sub(r"const LAYOUT_FIG_AFTER = \[.*?\n\];", "const LAYOUT_FIG_AFTER = [];", src, count=1, flags=re.S)
rep("const ch10 = chapters.find((c) => c.id === 'ch10_conclusions');", "const ch10 = null;")
rep("  warn('ch10_conclusions missing - sign-off table not placed');", "  // sign-off is part of um08_trouble")
# chapters flow on: no page break before every Heading 1 (except the first one after the contents page)
rep("heading: HeadingLevel.HEADING_1, pageBreakBefore: true,", "heading: HeadingLevel.HEADING_1, pageBreakBefore: s.num === 1,")
# cover
rep("text: 'TECHNICAL REPORT'", "text: 'USER MANUAL'")
rep("new TextRun({ text: 'Phase 1 Demonstration Report and User Manual', size: 34, color: C.purple })",
    "new TextRun({ text: 'User Manual - Phase 1 (Remote-Control Operation)', size: 34, color: C.purple })")
rep("text: \"MATLAB/Simulink/Simscape model of NASL's tracked autonomous agricultural platform - SBUS remote-control operation\"",
    "text: \"How to open, run and use the MATLAB/Simulink/Simscape model of NASL's tracked agricultural platform\"")
rep("['Report type', 'Technical / Engineering - Demonstration and User Manual'],", "['Document type', 'User manual (brief)'],")
rep("['Reporting period', 'As of 23 October 2026'],", "['Model state', 'As of 23 October 2026 (Phase 1 fixes applied)'],")
rep("['Date', '2026-10-23'],", "['Date', '2026-10-27'],")
rep("['Version', '1.0 - Draft for review'],", "['Version', '1.2 - Draft for review'],")
# document control: same page as the contents
rep("const out = [pageTitle('Document Control', true)];", "const out = [pageTitle('Document Control', true)];")
rep("['1.0', '2026-10-23', '[TBD - author]', '[TBD - reviewer]', 'Initial Phase 1 demonstration report and user manual. Status: Draft for review (until signed off)']",
    "['1.0', '2026-10-24', '[TBD - author]', '[TBD - reviewer]', 'Brief Phase 1 user manual'].map((t, i) => cell(t, i, false)) }),\n"
    "      new TableRow({ children: ['1.1', '2026-10-27', '[TBD - author]', '[TBD - reviewer]', 'Live drive with PS5 controller or RC transmitter and 3-D view (section 5); troubleshooting rows'].map((t, i) => cell(t, i, false)) }),\n"
    "      new TableRow({ children: ['1.2', '2026-10-27', '[TBD - author]', '[TBD - reviewer]', 'Plotting a run with plot_run: six plot pages explained (section 6). Status: Draft for review (until signed off)']")
src = re.sub(r"  out\.push\(spacer\(16\)\);\n  out\.push\(new Paragraph\(\{ spacing: \{ before: 240, after: 80 \}, children: \[new TextRun\(\{ text: 'Document information'.*?\], 26\)\);\n",
             "  out.push(new Paragraph({ spacing: { before: 160, after: 0 }, children: [new TextRun({ text: 'Full model description, design and test results: NASL_Report_Shaddad_Phase1_Demo_2026-10-23_v1. Live drive details: docs/live_rc_drive.md.', italics: true, size: 18, color: C.grey })] }));\n",
             src, count=1, flags=re.S)
rep("body.push(pageTitle('Contents', true));", "body.push(new Paragraph({ style: 'NaslTitle', spacing: { before: 480, after: 200 }, children: [new TextRun('Contents')] }));")
rep("headingStyleRange: '1-2'", "headingStyleRange: '1-1'")
# footer and metadata
rep("text: 'NASL Technologies | Shaddad UGV Phase 1 Demonstration Report | Internal'", "text: 'NASL Technologies | Shaddad UGV User Manual (Phase 1) | Internal'")
rep("title: 'Shaddad UGV Digital Twin - Phase 1 Demonstration Report and User Manual',", "title: 'Shaddad UGV Digital Twin - User Manual (Phase 1)',")
rep("subject: 'Phase 1 SBUS remote-control demonstration of the Shaddad_UGV MATLAB/Simulink/Simscape digital twin',",
    "subject: 'How to use the Shaddad_UGV MATLAB/Simulink/Simscape digital twin in Phase 1 (SBUS remote control)',")
rep("description: 'NASL Technologies (Robotat) - Internal - Version 1.0 - 2026-10-23',", "description: 'NASL Technologies (Robotat) - Internal - Version 1.2 - 2026-10-27',")
rep("const orderFile = path.join(BUILD_DIR, 'figure_order.json');", "const orderFile = path.join(BUILD_DIR, 'figure_order_manual.json');")

open(os.path.join(B, 'build_manual.js'), 'w', encoding='utf-8').write(src)
print('build_manual.js written')
