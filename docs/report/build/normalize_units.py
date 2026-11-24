"""normalize_units.py - one notation for units and symbols in the chapter JSON (report prose and tables).

Outside code blocks and outside `inline code` spans:
  m/s^2, m/s2 -> m/s²   (also rad/s, ^3)    kg m^2, m2 -> m²   m3 -> m³
  <= -> ≤   >= -> ≥   +/- -> ±   -/+ -> ∓
  Ohm -> Ω (mOhm, kOhm, uOhm)   degC -> °C   20 deg -> 20°   us, uH, uF, uT -> µs, µH, µF, µT
Code blocks and inline code keep the ASCII forms (they are MATLAB text).
The file layout is preserved: only the changed string literals are re-encoded.

usage: python normalize_units.py [chapters folder]
"""
import glob
import json
import os
import re
import sys

D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'chapters')

RULES = [
    (re.compile(r'/s\^?2(?![\w^])'), '/s²'),
    (re.compile(r'/s\^?3(?![\w^])'), '/s³'),
    (re.compile(r'(?<![\w.])m\^?2(?![\w^])'), 'm²'),
    (re.compile(r'(?<![\w.])m\^?3(?![\w^])'), 'm³'),
    (re.compile(r'<='), '≤'),
    (re.compile(r'>='), '≥'),
    (re.compile(r'\+/-'), '±'),
    (re.compile(r'-/\+'), '∓'),
    (re.compile(r'(?<![A-Za-z])mOhm\b'), 'mΩ'),
    (re.compile(r'(?<![A-Za-z])kOhm\b'), 'kΩ'),
    (re.compile(r'(?<![A-Za-z])uOhm\b'), 'µΩ'),
    (re.compile(r'(?<![A-Za-z])Ohm\b'), 'Ω'),
    (re.compile(r'(\d) ?degC\b'), r'\1 °C'),
    (re.compile(r'(?<![A-Za-z_])degC\b'), '°C'),
    (re.compile(r'(\d) ?deg\b(?!/)'), r'\1°'),
    (re.compile(r'(\d) ?us\b'), r'\1 µs'),
    (re.compile(r'(?<![A-Za-z_])uH\b'), 'µH'),
    (re.compile(r'(\d) ?uF\b'), r'\1 µF'),
    (re.compile(r'(?<![A-Za-z_])uT\b'), 'µT'),
]


def norm_text(t):
    parts = re.split(r'(`[^`]*`)', t)
    out = []
    for p in parts:
        if p.startswith('`') and p.endswith('`') and len(p) >= 2:
            out.append(p)
            continue
        for rx, rep in RULES:
            p = rx.sub(rep, p)
        out.append(p)
    return ''.join(out)


def code_texts(obj):
    s = set()
    for b in obj.get('blocks', []):
        if b.get('type') == 'code':
            s.add(b.get('text', ''))
    return s


STR = re.compile(r'"(?:[^"\\]|\\.)*"')


def main():
    total = 0
    for f in sorted(glob.glob(os.path.join(D, '*.json'))):
        raw = open(f, encoding='utf8').read()
        obj = json.loads(raw)
        skip = code_texts(obj)
        n = 0

        def repl(m):
            nonlocal n
            lit = m.group(0)
            val = json.loads(lit)
            if val in skip or len(val) < 2:
                return lit
            new = norm_text(val)
            if new == val:
                return lit
            n += 1
            return json.dumps(new, ensure_ascii=False)

        out = STR.sub(repl, raw)
        json.loads(out)
        if out != raw:
            open(f, 'w', encoding='utf8').write(out)
        print('%-28s %4d strings changed' % (os.path.basename(f), n))
        total += n
    print('total', total)


if __name__ == '__main__':
    main()
