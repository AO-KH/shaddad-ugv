"""fix_round2_content.py - content fixes of review round 2 (2026-10-23), applied to ../chapters/*.json.

Idempotent: every edit checks the old text and is skipped when the new text is already present.
Run:  python -I fix_round2_content.py
"""
import json
import os
import sys

sys.stdout.reconfigure(encoding='utf-8')
CH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'chapters'))
docs = {}


def load(c):
    if c not in docs:
        docs[c] = json.load(open(os.path.join(CH, c + '.json'), encoding='utf8'))
    return docs[c]


def save_all():
    for c, d in docs.items():
        head = {k: v for k, v in d.items() if k != 'blocks'}
        lines = ['{', ', '.join('%s: %s' % (json.dumps(k), json.dumps(v, ensure_ascii=False)) for k, v in head.items()) + ',', '"blocks": [']
        bl = [json.dumps(b, ensure_ascii=False) for b in d['blocks']]
        lines.append(',\n'.join(bl))
        lines.append(']')
        lines.append('}')
        open(os.path.join(CH, c + '.json'), 'w', encoding='utf8').write('\n'.join(lines) + '\n')


def blocks(c):
    return load(c)['blocks']


def find(c, pred, what):
    hits = [b for b in blocks(c) if pred(b)]
    if len(hits) != 1:
        raise SystemExit('%s: %d matches for %s' % (c, len(hits), what))
    return hits[0]


def by_text(c, sub, typ=None, alt=None):
    """block whose text contains sub (or, when the edit was already applied, alt)"""
    subs = [sub] + ([alt] if alt else [])
    return find(c, lambda b: (typ is None or b['type'] == typ) and any(x in b.get('text', '') for x in subs), repr(sub[:60]))


def table(c, tid):
    return find(c, lambda b: b['type'] == 'table' and b.get('id') == tid, tid)


def fig(c, f):
    return find(c, lambda b: b['type'] == 'figure' and b.get('file') == f, f)


def idx(c, b):
    return next(i for i, x in enumerate(blocks(c)) if x is b)


def rep(c, b, old, new, key='text'):
    s = b[key]
    if (new in s and old not in s) or (new and old in new and new in s):
        return
    if old not in s:
        raise SystemExit('%s: old text not found: %r' % (c, old[:80]))
    b[key] = s.replace(old, new, 1)


def rep_item(c, b, k, old, new):
    s = b['items'][k]
    if new in s:
        return
    if old not in s:
        raise SystemExit('%s: old item text not found: %r' % (c, old[:80]))
    b['items'][k] = s.replace(old, new, 1)


def insert_before(c, anchor, block):
    bl = blocks(c)
    i = idx(c, anchor)
    if i > 0 and bl[i - 1].get('text') == block.get('text'):
        return
    bl.insert(i, block)


def insert_after(c, anchor, block):
    bl = blocks(c)
    i = idx(c, anchor)
    if i + 1 < len(bl) and bl[i + 1].get('text') == block.get('text') and bl[i + 1]['type'] == block['type']:
        return
    bl.insert(i + 1, block)


def append_sentence(c, b, sentence):
    if sentence in b['text']:
        return
    b['text'] = b['text'].rstrip() + ' ' + sentence


MEM = '{sec:ch08_user_manual/PC requirements}'
REGEN = '{sec:ch08_user_manual/Regenerating models}'
P2 = '{sec:ch07_phase2_preview/Current Status}'

# ---------------------------------------------------------------- ch00: executive summary next step 4 (R8, R10)
b = find('ch00_front', lambda b: b['type'] == 'numbered' and any('Phase 2 fix pass' in x or '(R10)' in x for x in b['items']), 'next steps')
rep_item('ch00_front', b, 3, 'Update the stale unit tests, then finish the Phase 2 fix pass before hardware-in-the-loop and field tests.',
         'Update the stale unit tests (R8), then fix the Phase 2 findings (R10) before hardware-in-the-loop and field tests.')

# ---------------------------------------------------------------- ch01: Figure 1 caption without supplier speculation; Phase 2 status -> 7.3
b = fig('ch01_shaddad', 'F21_photo_shaddad.png')
rep('ch01_shaddad', b, ' Markings on the body (XMachines, MRX) indicate a third-party tracked carrier [TBD - to be confirmed by product].', '', key='caption')
b = by_text('ch01_shaddad', 'Phase 2 is not yet validated and is outside the scope of this report')
b['text'] = 'Phase 2 is not yet validated and is outside the scope of this report; {sec:ch07_phase2_preview/Current Status} gives the status of its scenarios S4, S5 and S7.'

# ---------------------------------------------------------------- ch02: memory and MCU-build warnings -> cross-references
b = by_text('ch02_architecture', 'Run in Normal mode. Windows Application Control blocks', 'note')
rep('ch02_architecture', b, 'Each full compile of `Shaddad_System` grows the MATLAB process by 0.5-1 GB. Restart MATLAB when it exceeds about 9 GB, otherwise the PC starts paging and runs 4-5 times slower.',
    'MATLAB memory grows with every compile; see ' + MEM + '.')
b = by_text('ch02_architecture', 'Do not edit the generated `.slx` files by hand', 'note')
rep('ch02_architecture', b, "Also, do not run `build_shaddad_mcu('all')` or `build_fw_platform()` with the default list: both overwrite the five Task_10/Task_50/Task_1k control units (`fw_safety_supervisor`, `fw_mode_manager`, `fw_vehicle_ctrl`, `fw_implement_aux`, `fw_watchdog_mgr`) with port stubs. Use `build_shaddad_mcu('model')`.",
    'Before you rebuild any part of the MCU model, read the warning in ' + REGEN + '.')

# ---------------------------------------------------------------- ch03: table lead-in
t = table('ch03_structure', 'tab_struct_params')
insert_before('ch03_structure', t, {'type': 'p', 'text': '{tab:tab_struct_params} lists the key structure parameters and the fields of `P` that hold them.'})

# ---------------------------------------------------------------- ch04: figure and table citations
b = by_text('ch04_power', 'The files are generated by `scripts/build/build_shaddad_battery.m`')
append_sentence('ch04_power', b, '{tab:tab_pwr_key} lists the key ratings of the power system.')
b = by_text('ch04_power', '`build_shaddad_battery` assembles the pack programmatically')
append_sentence('ch04_power', b, '{tab:tab_pwr_packvar} compares the two pack variants, and {fig:F10_battery_pack.png} shows the generated module inside `Pack_Grouped16`.')
b = fig('ch04_power', 'F10_battery_pack.png')
b['caption'] = ('Pack_Grouped16 (print -s; the Probe block is widened for legibility): the generated Battery Builder module (block Module) '
                'with the pack terminals pack_p / pack_n and the thermal port AmbH, the balancing-command path Bal_Cmd -> RT_bal -> Bal_double -> '
                'Bal_PS, and the Probe whose outputs batteryTemperature, socParallelAssembly and vParallelAssembly become T_cell_true, SOC_true and V_cell_true.')
b = by_text('ch04_power', '`Shaddad_BMS` is the firmware model of a dedicated STM32G4 BMS board')
append_sentence('ch04_power', b, '{fig:F14_bms_top.png} shows the top level of `Shaddad_BMS`, and {tab:tab_pwr_bmscore} lists the units inside `BMS_Core`.')
t = table('ch04_power', 'tab_pwr_prot')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_prot} lists the BMS protections with their parameters in `P.power.bms` and the bits they set in `Fault_Word` and `Warning_Word`.'})
b = by_text('ch04_power', '`BMS_ModeManager` runs every 10 ms')
append_sentence('ch04_power', b, '{tab:tab_pwr_fsm} lists the contactor states.')
b = by_text('ch04_power', 'CAN1 is classic CAN 2.0B at 500 kbit/s')
append_sentence('ch04_power', b, '{tab:tab_pwr_can} lists the BMS frames.')
t = table('ch04_power', 'tab_pwr_pdu')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_pdu} lists the switching and protection elements of the PDU and the blocks that model them.'})
t = table('ch04_power', 'tab_pwr_lv')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_lv} lists the low-voltage rails of `LV_Distribution`, their sources, ratings and loads.'})
b = by_text('ch04_power', '`Safety_Chain` models the hard-wired E-stop loop')
append_sentence('ch04_power', b, '{fig:F11_safety_chain.png} shows `Power_Plant/Safety_Chain`; the logic is summarised below.')
t = table('ch04_power', 'tab_pwr_safety')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_safety} lists the safety-chain signals between the MCU, the plant and the drives.'})
t = table('ch04_power', 'tab_pwr_impl')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_impl} lists the electrical variants of the implement drives, selected by `VC_IMPL`.'})
b = by_text('ch04_power', 'The pack is sized for the 4 h design shift of D46')
append_sentence('ch04_power', b, '{tab:tab_pwr_budget} gives the budget for three working speeds.')
t = table('ch04_power', 'tab_pwr_ph1energy')
insert_before('ch04_power', t, {'type': 'p', 'text': '{tab:tab_pwr_ph1energy} lists the pack consumption measured in two Phase 1 runs.'})

# ---------------------------------------------------------------- ch05: memory note -> cross-reference; table lead-ins
b = by_text('ch05_drive', 'The first compile after loading a model that contains `drive_esc`', 'note', alt='three Subsystem Reference levels deep) sometimes stops')
rep('ch05_drive', b, ' Each full `Shaddad_System` compile also grows MATLAB memory by 0.5-1 GB. Restart MATLAB above about 9 GB.', '')
rep('ch05_drive', b, ' For the memory growth per compile, see ' + MEM + '.', '')
t = table('ch05_drive', 'tab_drive_files')
insert_before('ch05_drive', t, {'type': 'p', 'text': '{tab:tab_drive_files} lists the traction-drive files and what each one contains.'})
b = by_text('ch05_drive', 'The drive spec sizes the drive for a 600 kg design mass')
rep('ch05_drive', b, 'The table also shows the check at 609 kg.', '{tab:tab_drive_loads} lists the load cases and also shows the check at 609 kg.')
b = by_text('ch05_drive', 'a usable motor top speed of about 4000 rpm gives N = 24')
append_sentence('ch05_drive', b, '{tab:tab_drive_design} summarises the main design values.')
b = by_text('ch05_drive', 'The states follow the CiA 402 power-drive state machine')
append_sentence('ch05_drive', b, '{tab:tab_esc_states} lists the states.')
t = table('ch05_drive', 'tab_esc_faults')
insert_before('ch05_drive', t, {'type': 'p', 'text': '{tab:tab_esc_faults} lists the bits of the ESC fault word with their triggers and debounce times.'})
b = by_text('ch05_drive', 'The value `VC_DRIVE_FID = 2` is reserved and rejected.')
append_sentence('ch05_drive', b, '{tab:tab_drive_fid} compares the three fidelity levels.')
t = table('ch05_drive', 'tab_drive_thermal')
insert_before('ch05_drive', t, {'type': 'p', 'text': '{tab:tab_drive_thermal} lists the elements of the per-side thermal network and their values.'})
b = by_text('ch05_drive', 'The four codec files are shared between the two ends.')
append_sentence('ch05_drive', b, '{tab:tab_drv_frames} lists the DRV frames.')
b = by_text('ch05_drive', 'No run of T24 is recorded. The system scenario S8a checks the same behaviour.')
append_sentence('ch05_drive', b, '{tab:tab_tdrive} lists the result of every test.')

# ---------------------------------------------------------------- ch06: MCU build warning -> cross-reference; N m
b = by_text('ch06_electronics', '`build_shaddad_mcu()` without an argument also runs `build_fw_platform()`', 'note', alt='Before you rebuild any MCU unit')
b['text'] = ('Before you rebuild any MCU unit, read the warning in ' + REGEN +
             ': `build_shaddad_mcu()` without an argument replaces the five MCU control units with stubs.')
t = table('ch06_electronics', 'tab_drv_cmd')
for r in t['rows']:
    for k, v in enumerate(r):
        r[k] = v.replace('N·m', 'N m')

# ---------------------------------------------------------------- ch08: user manual
c = 'ch08_user_manual'
b = by_text(c, "rt  = run_regression('Fast');   % fast static regression checks", 'code')
rep(c, b, '% fast static regression checks (about 3 min)', '% fast static regression checks (about 1-3 min)')
b = by_text(c, "**Expected result of `run_regression('Fast')`** on the model of 2026-10-23: 24 tests, 22 pass and 2 fail. The two failures", alt='This is the result of the last run')
b['text'] = ("**Expected result of `run_regression('Fast')`**: 24 tests, 22 pass and 2 fail. The two failures, `t_infra` testStartup and "
             'testPlantStructure, are stale test expectations, not model defects ({tab:tab_regr_fail}). This is the result of the last run '
             'on the model after the fix pass (2026-10-23 23:31, report `work/reports/Fast_20261023_233144`). Any other failure is a real problem.')
# Table 87: two required toolboxes
t = table(c, 'tab_products')
names = [r[0] for r in t['rows']]
if 'Communications Toolbox' not in names:
    k = names.index('Motor Control Blockset') + 1
    t['rows'][k:k] = [
        ['Communications Toolbox', 'Integer to Bit / Bit to Integer Converter blocks (`commutil2`): SBUS encoder (`RC_Link_Sim`), SBUS decoder (`fw_sbus_decoder`), MCU control units, CAN/CAN-FD codecs, BMS, electronics plant and sensor models', 'Installed'],
        ['DSP System Toolbox (requires Signal Processing Toolbox)', 'Minimum, Maximum and Moving Average (`dspstat3`) in `Shaddad_BMS`, Sort in `Sim_Monitors`; Moving RMS and Cumulative Sum in the Phase 2 GNC mission, Minimum in Phase 2 perception', 'Installed'],
    ]
t = table(c, 'tab_optional')
for r in t['rows']:
    if r[0] == 'Simulink 3D Animation':
        r[1] = 'Joystick operator input (`VC_OPSRC` 3); `vrlib/Joystick Input` failed its smoke test in R2026b on this installation.'
# 8.2.4 work/ folder
b = find(c, lambda b: b['type'] == 'numbered' and b['items'] and b['items'][0].startswith('Copy (or clone, once a repository exists)'), 'getting the project')
rep_item(c, b, 0, 'copy `work/results/` only if you want the saved runs of this report.',
         'copy `work/results/` only if you want the saved runs of this report. The project defines `work/` as its Simulink cache and code-generation folder, so if you do not copy it, create an empty `work/` folder in the project root before `openProject` (`mkdir work`).')
# 8.3: cite Figure 32; new 8.3.1 Exploring the model in Simulink
b = by_text(c, 'The report starts with a header line with the MATLAB version')
if not b['text'].startswith('{fig:F16_matlab_desktop_crop.png}'):
    b['text'] = '{fig:F16_matlab_desktop_crop.png} shows the startup report on the reference PC. ' + b['text']
anchor = by_text(c, 'The scenario scripts set the variant controls they need through `Simulink.SimulationInput`', 'note')
if not any(x.get('text') == 'Exploring the model in Simulink' for x in blocks(c)):
    new = [
        {'type': 'h3', 'text': 'Exploring the model in Simulink'},
        {'type': 'p', 'text': 'After the startup you can open the models in the Simulink editor to see how the twin is built. '
                              '{sec:ch02_architecture/Top model and model hierarchy} explains the hierarchy; {fig:F01_system_top_overview.png} and '
                              '{fig:F17_simulink_editor_callouts.png} show the top model, and the callout numbers are those of {tab:tab_top}.'},
        {'type': 'code', 'text': "open_system('Shaddad_System');               % top model\n"
                                 "open_system('Shaddad_System/Operator_RC');   % a Subsystem Reference (subsystems/rc/RC_Link_Sim.slx)\n"
                                 "open_system('Shaddad_MCU');                  % a referenced model on its own\n"
                                 "close_system('Shaddad_System', 0);           % close without saving"},
        {'type': 'bullets', 'items': [
            '**Move down the hierarchy.** Double-click the Model blocks `MCU`, `BMS` or `Plant` to open the referenced models `Shaddad_MCU`, '
            '`Shaddad_BMS` and `Shaddad_Plant`. A Subsystem Reference, for example `Operator_RC` or `Power_Plant`, opens like a normal subsystem; '
            'its content comes from its file in `subsystems/<domain>/`. The address bar above the canvas shows the current path; click an entry in it to go back up.',
            '**Model Browser.** Show it with the Hide/Show Model Browser button at the lower-left corner of the canvas. Its tree lists the subsystems '
            'of the open model; click an entry to open it.',
            '**Active variants.** The Variant Subsystems and Variant Sources (for example `GNC_Variant`, the `IO_In` / `IO_Out` blocks of the MCU and the '
            'pack and drive-plant variants) follow the `VC_*` values in the base workspace ({tab:tab_vc_arch}). After an update of the diagram '
            '(Ctrl+D, which compiles the model) the inactive choices are greyed out. On the **Debug** tab, **Information Overlays > Variant Legend** '
            'lists the variant controls with their current values and conditions.',
        ]},
        {'type': 'note', 'kind': 'warning', 'text': 'Never save a model from the editor: every `.slx` file is generated, so change the build script instead ({tab:tab_build}).'},
    ]
    i = idx(c, anchor) + 1
    blocks(c)[i:i] = new
# 8.5: table lead-ins, SoC -> SOC
code = by_text(c, "res = phase1_test_drive()                                    % default demo drive", 'code')
insert_before(c, code, {'type': 'p', 'text': '`phase1_test_drive` takes the name-value options in {tab:tab_d1_options}:'})
b = by_text(c, 'After the run the function prints a summary in this format')
b['text'] = 'After the run the function prints a summary in this format; {tab:tab_d1_summary} explains each line and gives the values of the default drive:'
b = by_text(c, 'The title line gives the time, the MCU mode, the SoC and the position-estimate error', alt='the MCU mode, the SOC and the position-estimate error')
rep(c, b, 'the MCU mode, the SoC and', 'the MCU mode, the SOC and')
b = by_text(c, "VC_VIZ = 1;                         % optional: live bird's-eye view (time, mode, SoC)", 'code', alt="(time, mode, SOC)")
rep(c, b, "(time, mode, SoC)", "(time, mode, SOC)")
code = by_text(c, "r = run_scenario('S1');                             % RC drive, Normal mode", 'code')
insert_before(c, code, {'type': 'p', 'text': '`run_scenario` takes the scenario id and the name-value options in {tab:tab_rs_options}:'})
# 8.6.3 memory -> cross-reference; drop the repeated warning note
b = by_text(c, 'Do not run all nine Phase 1 scenarios in one MATLAB session')
rep(c, b, 'Do not run all nine Phase 1 scenarios in one MATLAB session. Each compile of `Shaddad_System` adds 0.5-1 GB to the MATLAB process, and the 16 GB reference PC starts paging above about 9 GB, after which the runs are 4-5 times slower.',
    'Do not run all nine Phase 1 scenarios in one MATLAB session, because MATLAB memory grows with every compile (' + MEM + ').')
bl = blocks(c)
bl[:] = [x for x in bl if not (x['type'] == 'note' and x['text'].startswith('A serial batch compiles the system model once per run, and MATLAB memory grows by up to 1 GB'))]
# 8.11 running tests
b = by_text(c, "rt = run_regression('Fast');                 % static checks (t_infra, t_framework), about", 'code')
rep(c, b, '% static checks (t_infra, t_framework), about 3 min', '% static checks (t_infra, t_framework), about 1-3 min')
b = by_text(c, "**Expected result of `run_regression('Fast')`** on the model of 2026-10-23: 24 tests, 22 pass and 2 fail (`t_infra`", alt='gave exactly this result')
b['text'] = ("**Expected result of `run_regression('Fast')`**: 24 tests, 22 pass and 2 fail (`t_infra` testStartup and testPlantStructure, "
             'stale expectations, {tab:tab_regr_fail}). The last run on the model after the fix pass (2026-10-23 23:31, '
             '`work/reports/Fast_20261023_233144`, 1.0 min in a running session) gave exactly this result. Any other failure is a real problem. '
             "`run_regression('All')` runs for several hours; run the heavy classes one by one with a MATLAB restart in between (" + MEM + ').')
b = by_text(c, '`run_regression` writes an HTML report to `work/reports/<tag>_<time>`')
rep(c, b, ' Run heavy classes one at a time and restart MATLAB between them.', '')
# troubleshooting
t = table(c, 'tab_trouble')
for r in t['rows']:
    if r[0] == 'Simulation much slower than before; the PC pages':
        r[1] = 'MATLAB memory grows with every system compile and is not released (' + MEM + ')'
if not any(r[0].startswith('Every `t_framework` test errors') for r in t['rows']):
    k = next(i for i, r in enumerate(t['rows']) if r[0].startswith('Tests fail on bus counts'))
    t['rows'].insert(k + 1, ['Every `t_framework` test errors in its class setup (`dot indexing is not supported for variables of type double`)',
                             '`t_framework` reads `P` from the base workspace, and `P` was overwritten by an earlier experiment in the session',
                             'Run `shaddad_startup()` to restore `P`, then run the tests again'])

# ---------------------------------------------------------------- ch09: Fast regression re-run; Phase 2 -> 7.3
c = 'ch09_results'
b = find(c, lambda b: b['type'] == 'bullets' and any(x.startswith('**Fast regression:**') for x in b['items']), 're-verification bullets')
k = next(i for i, x in enumerate(b['items']) if x.startswith('**Fast regression:**'))
b['items'][k] = ("**Fast regression:** re-run on the model after the fix pass (2026-10-23 23:31, `work/reports/Fast_20261023_233144`): 24 tests, "
                 '22 passed, 2 failed (`t_infra` testStartup and testPlantStructure, both stale expectations), the same result as the earlier report '
                 '`Fast_20261023_002921`. All 13 Fast tests of `t_framework` passed.')
k = next(i for i, x in enumerate(b['items']) if x.startswith('**Not re-run:**'))
b['items'][k] = b['items'][k].replace('the full suite, `tScenarios` (now 15 tests, S6d added) and `t_framework`.',
                                      'the full suite, `tScenarios` (now 15 tests, S6d added) and the `t_framework` tests outside the Fast tag.')
if 'outside the Fast tag' not in b['items'][k]:
    raise SystemExit('ch09 not re-run item not updated')
b = by_text(c, 'Phase 2 is outside the scope of this report ({sec:ch07_phase2_preview/Current Status}). After the fix pass the S5 palm patrol', alt='Phase 2 is outside the scope of this report; {sec:')
b['text'] = 'Phase 2 is outside the scope of this report; {sec:ch07_phase2_preview/Current Status} gives the status of S4, S5 and S7.'

# ---------------------------------------------------------------- ch10: memory, Phase 2, timing
c = 'ch10_conclusions'
MEMK = ('does not release the memory', 'MATLAB memory grows with every compile of')
b = find(c, lambda b: b['type'] == 'bullets' and any(m in x for x in b['items'] for m in MEMK), 'speed bullets')
k = next(i for i, x in enumerate(b['items']) if any(m in x for m in MEMK))
b['items'][k] = 'MATLAB memory grows with every compile of `Shaddad_System`, so long sessions must be split with a MATLAB restart in between (' + MEM + ').'
b = by_text(c, 'Phase 2 is outside this report ({sec:ch07_phase2_preview/Current Status}). After the fix pass, S5 (palm patrol)', alt='Phase 2 is outside this report; {sec:')
b['text'] = ('Phase 2 is outside this report; {sec:ch07_phase2_preview/Current Status} gives the status of S4, S5 and S7. The open items are '
             'policy and tuning issues in the GNC, perception and navigation units. They do not affect the Phase 1 chain, which runs with the GNC stub.')
b = by_text(c, "rt  = run_regression('Fast')          % Fast-tagged static checks and unit benches, about", 'code')
rep(c, b, 'unit benches, about 3 min', 'unit benches, about 1-3 min')

# ---------------------------------------------------------------- ch11: MCU warning, DSP reference, table citations
c = 'ch11_appendices'
b = by_text(c, "Do not run `build_shaddad_mcu('all')` or `build_fw_platform()` with its default file list.", 'note', alt='Before you rebuild any part of the MCU model')
b['text'] = 'Before you rebuild any part of the MCU model, read the warning in ' + REGEN + '.'
b = find(c, lambda b: b['type'] == 'numbered' and any(x.startswith('Communications Toolbox:') for x in b['items']), 'MathWorks references')
if not any(x.startswith('DSP System Toolbox') for x in b['items']):
    k = next(i for i, x in enumerate(b['items']) if x.startswith('Communications Toolbox:'))
    b['items'].insert(k + 1, 'DSP System Toolbox (requires Signal Processing Toolbox): Minimum, Maximum, Moving Average, Sort, Moving RMS and Cumulative Sum (BMS statistics, `Sim_Monitors`, Phase 2 GNC mission and perception).')
t = table(c, 'tab_app_scripts')
for r in t['rows']:
    r[1] = r[1].replace("`run_regression('Fast')` takes about 3 min", "`run_regression('Fast')` takes about 1-3 min")
b = by_text(c, '`models/` holds eight models.')
append_sentence(c, b, '{tab:tab_app_models} lists them.')
b = by_text(c, 'Stateflow is not installed, so every state machine and every byte-level parser is a MATLAB function')
append_sentence(c, b, '{tab:tab_app_mfun} lists these functions.')
t = table(c, 'tab_app_scripts')
insert_before(c, t, {'type': 'p', 'text': '{tab:tab_app_scripts} lists the entry points for running, checking and rebuilding the model.'})
b = by_text(c, 'The tables below give the values that define the Phase 1 vehicle.', alt='give the values that define the Phase 1 vehicle')
rep(c, b, 'The tables below give the values that define the Phase 1 vehicle.',
    '{tab:tab_app_par_vehicle}, {tab:tab_app_par_drive}, {tab:tab_app_par_power}, {tab:tab_app_par_limits}, {tab:tab_app_par_rc} and '
    '{tab:tab_app_par_rates} give the values that define the Phase 1 vehicle.')
t = table(c, 'tab_app_glossary')
insert_before(c, t, {'type': 'p', 'text': '{tab:tab_app_glossary} explains the terms and abbreviations used in this report.'})

# ---------------------------------------------------------------- ch06: Figure RC_Link_Sim as two readable detail panels
c = 'ch06_electronics'
fb = find(c, lambda b: b['type'] == 'figure' and b['file'] in ('F05_rc_link_sim.png', 'F05_rc_link_sim_split.png'), 'RC_Link_Sim figure')
fb['file'] = 'F05_rc_link_sim_split.png'
fb['caption'] = ('RC_Link_Sim (print -s, shown in two parts so that the block names stay readable): (a) the operator input op and the fault input flt '
                 'pass the 10 ms, 1 ms and 14 ms rate chain (RT_op_1k, RT_op_frame, RT_flt_1k, RT_flt_frame, BS_flt_frame) into RC_Transmitter, '
                 'RC_Radio_Channel and RC_Receiver; (b) the receiver frame goes to RT_rc_frame (logging output rc_frame) and to SBUS_Encoder, and '
                 'UART_Line_Sim produces the 1 ms SBUS byte stream sbus.')
b = by_text(c, '{fig:F05_rc_link_sim', 'p')
b['text'] = b['text'].replace('{fig:F05_rc_link_sim.png}', '{fig:F05_rc_link_sim_split.png}')

# ---------------------------------------------------------------- ch09: provenance memory note -> cross-reference
c = 'ch09_results'
b = by_text(c, 'MATLAB was restarted between runs, because')
rep(c, b, 'MATLAB was restarted between runs, because every full-system compile adds 0.5 to 1 GB to the MATLAB process; above about 9 GB the PC pages and the simulation runs 4 to 5 times slower (STATUS section 3).',
    'MATLAB was restarted between runs, because MATLAB memory grows with every full-system compile (' + MEM + ').')

# ---------------------------------------------------------------- ch05: end of chapter fits on its last page (layout)
c = 'ch05_drive'
b = by_text(c, '% regenerate drive files after editing build_drive.m or param_drive.m', 'code')
b['text'] = b['text'].replace(chr(10) + chr(10), chr(10))
b = by_text(c, 'sometimes stops with the R2026b JIT relink assertion', 'note', alt='three Subsystem Reference levels deep) sometimes stops')
b['text'] = ('The first compile after loading a model with `drive_esc` (three Subsystem Reference levels deep) sometimes stops with the '
             'R2026b JIT relink assertion `o != nullptr`. Run the command again; `t_drive` retries once for the same reason.')
b = by_text(c, 'The ESC model is a behavioural approximation of third-party firmware', 'note', alt='The ESC model approximates third-party firmware')
b['text'] = ('The ESC model approximates third-party firmware: its speed-loop gains, 5 ms torque lag and motor loss and thermal data are '
             'estimates. Once the ESC and motor are chosen, identify them on the bench and update `Kp_w`, `Ki_w`, `Tc` and the loss maps in `param_drive.m`.')

# ---------------------------------------------------------------- ch04: 4.10 listing in two parts (layout: the section no longer
# starts on a new page and the chapter no longer spills three lines onto an extra page)
c = 'ch04_power'
b = by_text(c, "open_system('Power_Plant')       % plant, subsystems/power/Power_Plant.slx", 'code')
SEP = chr(10) + 'shaddad_params();' + chr(10) + chr(10)
if SEP + "r0  = run_scenario('S0');" in b['text']:
    first, second = b['text'].split(SEP, 1)
    b['text'] = first + chr(10) + 'shaddad_params();'
    i = idx(c, b)
    blocks(c)[i + 1:i + 1] = [
        {'type': 'p', 'text': 'Then run the checks: the power-up scenario, the demonstration drive with the battery signals, the precharge finding with the specification resistor, the lumped pack variant and the power benches:'},
        {'type': 'code', 'text': second},
    ]

save_all()
print('saved:', ', '.join(sorted(docs)))
