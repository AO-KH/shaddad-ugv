"""Write a self-contained snapshot of the dashboard: one HTML file that opens in any browser, offline.

usage:
  python tools/dashboard/snapshot.py [--port 8765] [--out <file.html>]

The user asked on 2026-11-14 for "the dashboard as an attachment". The live dashboard is a page fed by
tools/dashboard/server.py; this script reads the running server once and writes static/index.html with:
  - the data of /api/status, /api/agents and /api/logs built in, and a fetch() stub that answers the page from it;
  - the text of every Markdown report of the Reports section (and the last 400 lines of its run logs);
  - the figures of those reports as data URIs;
  - a banner that says it is a snapshot and when it was taken.
Left out to keep the file small: the plot images of the Plots tab, the HTML test reports and the run logs of the
logs panel (each shows as not in the snapshot). Default output: work/dashboard_snapshot/shaddad_dashboard_<time>.html.
"""
import argparse
import base64
import datetime as dt
import json
import mimetypes
import os
import re
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
FIG_RE = re.compile(r'^(!\[[^\]]*\]\()([^)]+)(\)\s*)$', re.M)


def get(base, path, as_text=False):
    with urllib.request.urlopen(base + path, timeout=120) as r:
        body = r.read().decode('utf-8', errors='replace')
    return body if as_text else json.loads(body)


def data_uri(rel):
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        return None
    mt = mimetypes.guess_type(p)[0] or ('image/svg+xml' if p.endswith('.svg') else 'application/octet-stream')
    with open(p, 'rb') as f:
        return f'data:{mt};base64,' + base64.b64encode(f.read()).decode('ascii')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--out')
    a = ap.parse_args()
    base = f'http://127.0.0.1:{a.port}'
    now = dt.datetime.now()

    status = get(base, '/api/status?refresh=1')
    agents = get(base, '/api/agents')
    logs = get(base, '/api/logs')
    texts, nfig, missing = {}, 0, 0
    for r in status.get('reports', []):
        if not r.get('readable'):
            continue
        try:
            t = get(base, '/api/text?path=' + urllib.parse.quote(r['file']), as_text=True)
        except Exception:  # noqa: BLE001  (a report that cannot be read is left out)
            continue
        if r['file'].endswith('.log'):
            t = '\n'.join(t.replace('\r', '').split('\n')[-400:])
        else:
            def repl(m):
                nonlocal nfig, missing
                u = data_uri(m.group(2))
                if u is None:
                    missing += 1
                    return m.group(0)
                nfig += 1
                return m.group(1) + u + m.group(3)
            t = FIG_RE.sub(repl, t)
        texts[r['file']] = t

    with open(os.path.join(HERE, 'static', 'index.html'), encoding='utf-8') as f:
        page = f.read()
    # the reader shows a figure from a data URI as it is (the live page passes paths through /raw/)
    page = page.replace("const src = /^(https?:|\\/)/.test(im[2])", "const src = /^(https?:|\\/|data:)/.test(im[2])")
    snap = {'status': status, 'agents': agents, 'logs': logs, 'text': texts, 'taken': now.strftime('%Y-%m-%d %H:%M')}
    stub = (
        "<script>\n"
        "window.__SNAP = " + json.dumps(snap, ensure_ascii=False).replace('</', '<\\/') + ";\n"
        "(function () {\n"
        "  const S = window.__SNAP, real = window.fetch ? window.fetch.bind(window) : null;\n"
        "  const reply = (o) => new Response(o === undefined ? 'not in the snapshot' : (typeof o === 'string' ? o : JSON.stringify(o)),\n"
        "    { status: o === undefined ? 404 : 200, headers: { 'Content-Type': typeof o === 'string' ? 'text/plain; charset=utf-8' : 'application/json' } });\n"
        "  window.fetch = async function (url, opts) {\n"
        "    const u = String(url);\n"
        "    if (u.startsWith('/api/status')) return reply(S.status);\n"
        "    if (u.startsWith('/api/agents')) return reply(S.agents);\n"
        "    if (u.startsWith('/api/logs')) return reply(S.logs);\n"
        "    if (u.startsWith('/api/text')) { const m = u.match(/[?&]path=([^&]*)/); return reply(S.text[m ? decodeURIComponent(m[1]) : '']); }\n"
        "    return real ? real(url, opts) : reply(undefined);\n"
        "  };\n"
        "})();\n"
        "</script>\n")
    banner = (f"<div style=\"position:sticky;top:0;z-index:50;padding:7px 16px;font:600 12.5px/1.4 system-ui,sans-serif;"
              f"background:#6722ba;color:#fff;text-align:center\">Snapshot of the Shaddad UGV dashboard, taken {now:%d %b %Y %H:%M}. "
              f"A static copy: it does not update; plot images, HTML test reports and the logs panel's logs are not included.</div>\n")
    i = page.index('<script>')
    page = page[:i] + stub + page[i:]
    j = page.index('<body>') + len('<body>')
    page = page[:j] + '\n' + banner + page[j:]

    out = a.out or os.path.join(ROOT, 'work', 'dashboard_snapshot', f'shaddad_dashboard_{now:%Y%m%d_%H%M}.html')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(page)
    print(f'wrote {os.path.relpath(out, ROOT)}: {os.path.getsize(out) / 2**20:.1f} MB, {len(texts)} reports, '
          f'{nfig} figures built in, {missing} figure files missing')


if __name__ == '__main__':
    main()
