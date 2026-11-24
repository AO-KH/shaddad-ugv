"""Map the blocks of a Simulink model to their generated C code, from the trace comments Embedded Coder writes.

usage:
  python tools/reports/codegen_trace.py <code dir> [--out trace.json] [--find "<block or subsystem path>" ...]

<code dir> is an Embedded Coder build folder, for example work/fw_sbus_decoder_ert_rtw or slprj/ert/fw_sbus_decoder.
The tool reads the system hierarchy at the end of the model's header ('<S12>' : 'model/path') and every .c file in the
folder. For each C function it finds the blocks its body names, and for each block the C functions and line ranges
that implement it. The tags it reads:
  /* <BlockType>: '<S12>/Name' */ and the 'incorporates:' lists     a block of system S12 (or '<Root>/Name')
  '<S12>:1:34'                                                     line 34 of the MATLAB code of block S12
                                                                   (written when MATLABSourceComments is on)
  'sbus_frame_sync:34'                                             line 34 of an external MATLAB file
--find prints the C functions and lines of a block, or of every block under a subsystem path (a prefix match), so an
engineer can check that the paths of an inventory resolve.

Plain python, no installs. Written for card T-07 (embedded software verification and design description).
"""
import argparse
import glob
import json
import os
import re
import sys

HIER = re.compile(r"^\s\*\s+'<(Root|S\d+)>'\s*:\s*'(.*)'\s*$")
BLOCK_TAG = re.compile(r"(?:([A-Za-z][\w ]*?):\s*)?'<(Root|S\d+)>/((?:[^'\\]|\\.|'')+?)'")
ML_LINE_TAG = re.compile(r"'<(S\d+)>:(\d+):(\d+)'")
EXT_LINE_TAG = re.compile(r"'([A-Za-z]\w*):(\d+)'")
FUNC_HEAD = re.compile(r'^(?!typedef|struct|union|enum|extern\s+"|#)'
                       r'(?:static\s+|extern\s+)?(?:const\s+)?[A-Za-z_][\w\s\*]*?\b([A-Za-z_]\w*)\s*\(')
HEADER_KEYS = {'generated_on': 'C/C++ source code generated on', 'hardware': 'Embedded hardware selection',
               'objectives': 'Code generation objectives', 'model_version': 'Model version',
               'coder_version': 'Embedded Coder version'}


def read(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read().splitlines()


def hierarchy(code_dir):
    """System id -> path, from every header that holds the system hierarchy."""
    sys_map = {}
    for h in sorted(glob.glob(os.path.join(code_dir, '*.h'))):
        for line in read(h):
            m = HIER.match(line)
            if m:
                sys_map.setdefault(m.group(1), m.group(2).replace("''", "'"))
    return sys_map


def functions(lines):
    """(name, first line, last line) of each C function defined at column 0 (1-based line numbers)."""
    out, i, n = [], 0, len(lines)
    while i < n:
        line = lines[i]
        m = FUNC_HEAD.match(line)
        if m and not line.rstrip().endswith(';') and not line.startswith((' ', '\t')):
            j = i
            head = line
            while j < n and ')' not in head and j - i < 12:
                j += 1
                head += lines[j] if j < n else ''
            k = j
            while k < n and k - j < 3 and lines[k].strip() != '{':
                if lines[k].rstrip().endswith(';'):
                    break
                k += 1
            if k < n and lines[k].strip() == '{' and not lines[k].startswith((' ', '\t')):
                e = k + 1
                while e < n and lines[e] != '}':
                    e += 1
                out.append((m.group(1), i + 1, e + 1))
                i = e + 1
                continue
        i += 1
    return out


def header_info(lines):
    info = {}
    for line in lines[:60]:
        for key, text in HEADER_KEYS.items():
            if text in line and key not in info:
                info[key] = line.split(':', 1)[1].strip() if ':' in line else ''
    return info


def resolve(sys_map, sid, name):
    base = sys_map.get(sid)
    if base is None:
        return None
    return base + '/' + name.replace("''", "'").replace('\\n', ' ')


def trace(code_dir):
    sys_map = hierarchy(code_dir)
    model = sys_map.get('Root') or os.path.basename(os.path.normpath(code_dir)).replace('_ert_rtw', '')
    data = {'model': model, 'code_dir': code_dir.replace('\\', '/'), 'systems': len(sys_map), 'info': {},
            'files': [], 'functions': [], 'blocks': {}, 'matlab_lines': {}, 'external_lines': {}}
    for c in sorted(glob.glob(os.path.join(code_dir, '*.c'))):
        lines = read(c)
        fname = os.path.basename(c)
        data['files'].append({'name': fname, 'lines': len(lines)})
        if not data['info']:
            data['info'] = header_info(lines)
        funcs = functions(lines)
        owner = [None] * (len(lines) + 2)
        for name, a, b in funcs:
            for k in range(a, b + 1):
                owner[k] = name
        for name, a, b in funcs:            # the comment block just above a function belongs to it
            k = a - 1
            while k >= 1 and owner[k] is None and lines[k - 1].strip().startswith(('/*', '*')):
                owner[k] = name
                k -= 1
        nblk = {}
        for no, line in enumerate(lines, 1):
            if '<' not in line and ':' not in line:
                continue
            fn = owner[no] or '(file scope)'
            for m in BLOCK_TAG.finditer(line):
                path = resolve(sys_map, m.group(2), m.group(3))
                if path is None:
                    continue
                btype = (m.group(1) or '').strip()
                if btype and btype not in ('incorporates', 'Function for', 'Outputs for', 'End of', 'Update for'):
                    data.setdefault('types', {}).setdefault(path, btype.split(' for ')[-1].split('End of ')[-1])
                rows = data['blocks'].setdefault(path, [])
                for r in rows:
                    if r['file'] == fname and r['function'] == fn:
                        r['first'], r['last'], r['refs'] = min(r['first'], no), max(r['last'], no), r['refs'] + 1
                        break
                else:
                    rows.append({'file': fname, 'function': fn, 'first': no, 'last': no, 'refs': 1})
                nblk.setdefault(fn, set()).add(path)
            for m in ML_LINE_TAG.finditer(line):
                blk = sys_map.get(m.group(1))
                if blk:
                    key = blk + ' [script ' + m.group(2) + ']'
                    data['matlab_lines'].setdefault(key, {}).setdefault(m.group(3), []).append(
                        {'file': fname, 'function': fn, 'line': no})
            if '/*' in line or line.lstrip().startswith('*'):
                for m in EXT_LINE_TAG.finditer(line):
                    data['external_lines'].setdefault(m.group(1), {}).setdefault(m.group(2), []).append(
                        {'file': fname, 'function': fn, 'line': no})
        for name, a, b in funcs:
            data['functions'].append({'file': fname, 'name': name, 'first': a, 'last': b,
                                      'blocks': len(nblk.get(name, ()))})
    return data


def find(data, path):
    """C locations of a block, or of every block under a subsystem path."""
    hits = {}
    for blk, rows in data['blocks'].items():
        if blk == path or blk.startswith(path + '/'):
            for r in rows:
                key = (r['file'], r['function'])
                a, b, n = hits.get(key, (r['first'], r['last'], 0))
                hits[key] = (min(a, r['first']), max(b, r['last']), n + r['refs'])
    return [{'file': f, 'function': fn, 'first': a, 'last': b, 'refs': n}
            for (f, fn), (a, b, n) in sorted(hits.items(), key=lambda kv: (kv[0][0], kv[1][0]))]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('code_dir')
    ap.add_argument('--out', help='write the trace as JSON here')
    ap.add_argument('--find', action='append', default=[], help='a block or subsystem path to look up (repeatable)')
    a = ap.parse_args()
    if not os.path.isdir(a.code_dir):
        sys.exit('not a folder: ' + a.code_dir)
    data = trace(a.code_dir)
    print('%s: %d C files, %d functions, %d systems, %d blocks traced, %d MATLAB blocks with line tags'
          % (data['model'], len(data['files']), len(data['functions']), data['systems'], len(data['blocks']),
             len(data['matlab_lines'])))
    for k, v in data['info'].items():
        print('  %s: %s' % (k, v))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(data, f, indent=1, ensure_ascii=True)
        print('wrote', a.out)
    bad = 0
    for p in a.find:
        rows = find(data, p)
        if not rows:
            bad += 1
            print('NOT FOUND:', p)
        for r in rows:
            print('  %s -> %s %s() lines %d-%d (%d tags)' % (p, r['file'], r['function'], r['first'], r['last'],
                                                            r['refs']))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
