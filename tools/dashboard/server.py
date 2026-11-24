"""Shaddad UGV dashboard: a small local web server that shows the project's state, read from its files.

usage: python tools/dashboard/server.py [--port 8765] [--every 30]

It listens on 127.0.0.1 only and writes nothing in the project outside tools/dashboard: a background thread reads the
project (collect.py) every --every seconds, notes in the journal what changed since the reading before (journal.py),
and the page (static/index.html) polls /api/status and redraws. /api/status?refresh=1 reads now. /api/text?path=<file>
returns a text file of the project (.md, .txt, .csv, .log) for the report reader; /raw/<path> serves the HTML test
reports under work/reports.
"""
import argparse
import json
import mimetypes
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import agents  # noqa: E402
import collect  # noqa: E402
import journal  # noqa: E402

INDEX = os.path.join(HERE, 'static', 'index.html')
ROOT = os.path.realpath(collect.ROOT)
RAW_ROOT = os.path.realpath(os.path.join(ROOT, 'work', 'reports'))
WORK_ROOT = os.path.realpath(os.path.join(ROOT, 'work'))
TEXT_EXT = ('.md', '.txt', '.csv', '.log', '.json')
STATE = {'body': b'{}', 'at': 0.0}
LOCK = threading.Lock()


def refresh():
    data = collect.collect()
    try:
        journal.observe(data)
    except Exception as e:  # the journal must never stop the dashboard
        data.setdefault('errors', []).append(f'journal: {type(e).__name__}: {e}')
    data['journal'] = journal.read()
    for k in ('all',):
        (data.get('models') or {}).pop(k, None)
    body = json.dumps(data, ensure_ascii=False).encode('utf-8')
    with LOCK:
        STATE.update(body=body, at=time.time())


def loop(every):
    while True:
        time.sleep(every)
        try:
            refresh()
        except Exception as e:  # keep serving the last state
            print(f'{time.strftime("%H:%M:%S")} refresh failed: {e}', file=sys.stderr, flush=True)


def inside(path, base):
    p = os.path.realpath(path)
    return p == base or p.startswith(base + os.sep)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path, _, query = self.path.partition('?')
        if path in ('/', '/index.html'):
            with open(INDEX, 'rb') as f:
                self.reply(200, 'text/html; charset=utf-8', f.read())
        elif path == '/api/status':
            if 'refresh=1' in query:
                refresh()
            with LOCK:
                body = STATE['body']
            self.reply(200, 'application/json; charset=utf-8', body)
        elif path == '/api/agents':
            # read live on every request (a small file), so the Agents tab follows the agents within seconds
            self.reply(200, 'application/json; charset=utf-8', json.dumps(agents.read(), ensure_ascii=False).encode('utf-8'))
        elif path == '/api/logs':
            self.reply(200, 'application/json; charset=utf-8', json.dumps(collect.run_logs(), ensure_ascii=False).encode('utf-8'))
        elif path == '/api/text':
            rel = (parse_qs(query).get('path') or [''])[0]
            full = os.path.join(ROOT, rel)
            if not rel or not inside(full, ROOT) or not full.lower().endswith(TEXT_EXT) or not os.path.isfile(full):
                return self.reply(404, 'text/plain; charset=utf-8', b'not found')
            if os.path.getsize(full) > 4 * 2 ** 20:
                return self.reply(413, 'text/plain; charset=utf-8', b'file too large for the reader')
            with open(full, 'rb') as f:
                self.reply(200, 'text/plain; charset=utf-8', f.read())
        elif path.startswith('/raw/'):
            # the HTML test reports under work/reports with their files, and plot images anywhere under work/
            full = os.path.join(ROOT, unquote(path[len('/raw/'):]))
            ok = inside(full, RAW_ROOT) or (inside(full, WORK_ROOT) and full.lower().endswith(collect.IMG_EXT))
            if not ok or not os.path.isfile(full):
                return self.reply(404, 'text/plain; charset=utf-8', b'not found')
            with open(full, 'rb') as f:
                self.reply(200, mimetypes.guess_type(full)[0] or 'application/octet-stream', f.read())
        elif path == '/favicon.ico':
            self.reply(204, 'image/x-icon', b'')
        else:
            self.reply(404, 'text/plain; charset=utf-8', b'not found')

    def reply(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def main():
    ap = argparse.ArgumentParser(description='Shaddad UGV dashboard')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--every', type=float, default=30.0, help='seconds between refreshes')
    a = ap.parse_args()
    refresh()
    threading.Thread(target=loop, args=(a.every,), daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', a.port), Handler)
    print(f'Shaddad UGV dashboard on http://localhost:{a.port} (project {collect.ROOT}); refresh every {a.every:g} s', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
