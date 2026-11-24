"""Derive build/build_mbd.js (model-based design report of the real robot) from build/build_report.js."""
import os
import re

B = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'build')
src = open(os.path.join(B, 'build_report.js'), encoding='utf-8').read()


def rep(old, new, count=1):
    global src
    if old not in src:
        raise SystemExit('pattern not found: ' + old[:80])
    src = src.replace(old, new, count)


VERSION = '1.1'
OUT = 'NASL_Report_Shaddad_RealRobot_MBD_2026-10-27_v' + VERSION

# header comment, inputs and outputs
rep(" * Shaddad UGV Digital Twin - Phase 1 Demonstration Report and User Manual\n",
    " * Shaddad UGV - Model-Based Design of the Real Robot (components, specifications, models).\n"
    " * Derived from build_report.js by mbd/patch_builder_mbd.py; chapters in ../mbd/mb*.json\n")
rep("const CH_DIR = path.join(REPORT_DIR, 'chapters');", "const CH_DIR = path.join(REPORT_DIR, 'mbd');")
rep("const OUT_BASE = 'NASL_Report_Shaddad_Phase1_Demo_2026-10-23_v1';", "const OUT_BASE = '" + OUT + "';")
src = re.sub(r"const CHAPTER_ORDER = \[[^\]]*\];",
             "const CHAPTER_ORDER = ['mb01_intro', 'mb02_mbd', 'mb03_vehicle', 'mb04_drive', 'mb05_battery', "
             "'mb06_electronics', 'mb07_sprayer', 'mb08_results', 'mb09_verification', 'mb10_open', 'mb11_appendix'];",
             src, count=1, flags=re.S)
rep("const OVERVIEW_H_CM = { 'F03_mcu_top_overview.png': 16, 'F04_mcu_task10_overview.png': 16 };",
    "const OVERVIEW_H_CM = { 'F03_mcu_top_overview.png': 12 };")
src = re.sub(r"const FIG_WIDTH_CM = \{[^}]*\};",
             "const FIG_WIDTH_CM = {\n"
             "  'F21_photo_shaddad.png': 12.5, 'M01_spec_sheet.png': 13.0, 'M02_motor_nameplate.jpg': 9.0,\n"
             "  'M16_real_components_collage.jpg': 14.5, 'F20_mcu_modes.png': 15.0, 'M42_d1_motion.png': 15.9,\n"
             "  'M43_d1_energy.png': 15.9, 'M03_sprayer_pump.jpg': 9.0,\n"
             "  'M15_battery_spec_p1.png': 14.5, 'M15_battery_spec_p2.png': 14.5,\n"
             "  'M30_battery_module_chart.png': 13.5, 'M31_pack_grouped.png': 13.0, 'M33_robot3d_side.png': 13.5,\n"
             "  'M34_robot3d_sprayer.png': 13.5, 'M35_robot3d_chase.png': 12.5, 'M25_chi_soils.png': 14.5,\n"
             "  'M21_motor_envelope.png': 15.0, 'M22_motor_efficiency.png': 15.0, 'M54_spr_board_3d.png': 13.5,\n"
             "  'M52_ads1115_3d.png': 14.0, 'F12_structure_plant.png': 15.0, 'F14_bms_top.png': 15.0,\n"
             "};",
             src, count=1, flags=re.S)
rep("const keepTogether = b.rows.length <= 4;", "const keepTogether = b.keep || b.rows.length <= 4;")
# contents: 11 chapters and 38 sections fill page 3; tighter entries keep the field end off a blank page 4
rep("run: { bold: true, color: C.navy, size: 21 }, paragraph: { spacing: { before: 140, after: 30, line: 240 } } },",
    "run: { bold: true, color: C.navy, size: 21 }, paragraph: { spacing: { before: 90, after: 20, line: 240 } } },")
rep("run: { size: 19 }, paragraph: { spacing: { before: 0, after: 10, line: 240 }, indent: { left: 300 } } },",
    "run: { size: 19 }, paragraph: { spacing: { before: 0, after: 6, line: 228 }, indent: { left: 300 } } },")
rep("const TITLECASE_CHAPTERS = new Set(['ch00_front', 'ch03_structure', 'ch05_drive', 'ch06_electronics', 'ch07_phase2_preview']);",
    "const TITLECASE_CHAPTERS = new Set([]);")
rep("const IGNORE_PAGEBREAKS = new Set(['ch06_electronics']);", "const IGNORE_PAGEBREAKS = new Set([]);")
src = re.sub(r"const LAYOUT_FIG_AFTER = \[.*?\n\];", "const LAYOUT_FIG_AFTER = [];", src, count=1, flags=re.S)
rep("const ch10 = chapters.find((c) => c.id === 'ch10_conclusions');", "const ch10 = null;")
rep("  warn('ch10_conclusions missing - sign-off table not placed');", "  // sign-off is part of mb10_open")
# cover
rep("new TextRun({ text: 'Phase 1 Demonstration Report and User Manual', size: 34, color: C.purple })",
    "new TextRun({ text: 'Model-Based Design of the Real Robot', size: 34, color: C.purple })")
rep("text: \"MATLAB/Simulink/Simscape model of NASL's tracked autonomous agricultural platform - SBUS remote-control operation\"",
    "text: 'XMachines MAX tracked platform with sprayer module, 48 V Li-ion battery, Movematic drives and STM32 electronics: the real components, their specifications and their MATLAB/Simulink/Simscape models'")
rep("['Report type', 'Technical / Engineering - Demonstration and User Manual'],", "['Report type', 'Technical / Engineering - model-based design and specification'],")
rep("['Reporting period', 'As of 23 October 2026'],", "['Model state', 'As of 27 October 2026 (model re-parameterised to the real robot)'],")
rep("['Version', '1.0 - Draft for review'],", "['Version', '" + VERSION + " - Draft for review'],")
rep("['Date', '2026-10-23'],", "['Date', '2026-10-27'],")
# document control
rep("['1.0', '2026-10-23', '[TBD - author]', '[TBD - reviewer]', 'Initial Phase 1 demonstration report and user manual. Status: Draft for review (until signed off)']",
    "['1.0', '2026-10-27', '[TBD - author]', '[TBD - reviewer]', 'Initial model-based design report of the real robot'].map((t, i) => cell(t, i, false)) }),\n"
    "      new TableRow({ children: ['1.1', '2026-10-27', '[TBD - author]', '[TBD - reviewer]', 'Four traction motors with the proposed 1.0 kW unit and motor sizing; speed loop re-tuned and brake model adjusted for two motors per side (D64); track relaxation length fix; unit-test findings and Phase 1 Full suite results; S2 tilt fix (IMU level calibration, TILT_WARN 25 deg) and known issues. Status: Draft for review (until signed off)']")
rep("['Title', 'Shaddad UGV Digital Twin - Phase 1 Demonstration Report and User Manual'],",
    "['Title', 'Shaddad UGV Digital Twin - Model-Based Design of the Real Robot'],")
rep("['Phase covered', 'Phase 1 - SBUS remote-control operation, simulation target'],",
    "['Scope', 'Real robot (XMachines MAX with sprayer module) and its Phase 1 model as of 2026-10-27'],")
rep("['Open points', 'Release: author, reviewer and approver to be named before v1.0 is released. Platform origin: markings on the body in {fig:F21_photo_shaddad.png} (XMachines, MRX) suggest a third-party tracked carrier [TBD - to be confirmed by product].'],",
    "['Open points', 'Release: author, reviewer and approver to be named before release. Estimated values [EST] are listed with the way to measure them in section 10.'],")
# footer and metadata
rep("text: 'NASL Technologies | Shaddad UGV Phase 1 Demonstration Report | Internal'", "text: 'NASL Technologies | Shaddad UGV Real Robot and Model (MBD) | Internal'")
rep("title: 'Shaddad UGV Digital Twin - Phase 1 Demonstration Report and User Manual',", "title: 'Shaddad UGV Digital Twin - Model-Based Design of the Real Robot',")
rep("subject: 'Phase 1 SBUS remote-control demonstration of the Shaddad_UGV MATLAB/Simulink/Simscape digital twin',",
    "subject: 'Real components, specifications and models of the Shaddad robot (XMachines MAX) in the Shaddad_UGV digital twin',")
rep("description: 'NASL Technologies (Robotat) - Internal - Version 1.0 - 2026-10-23',", "description: 'NASL Technologies (Robotat) - Internal - Version " + VERSION + " - 2026-10-27',")
rep("keywords: 'Shaddad, UGV, digital twin, Simulink, Simscape, SBUS, NASL, Robotat',",
    "keywords: 'Shaddad, XMachines, UGV, model-based design, digital twin, Simulink, Simscape, NASL, Robotat',")
rep("const orderFile = path.join(BUILD_DIR, 'figure_order.json');", "const orderFile = path.join(BUILD_DIR, 'figure_order_mbd.json');")

open(os.path.join(B, 'build_mbd.js'), 'w', encoding='utf-8').write(src)
print('build_mbd.js written (' + OUT + ')')
