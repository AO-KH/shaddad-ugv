function file = phase1_report(R, K)
%PHASE1_REPORT  Write the HTML report of a Phase 1 test-suite run (called by run_phase1_tests).
%
%   file = phase1_report(R, K)   R from run_phase1_tests, K = phase1_known_issues(); writes <R.folder>/report.html

if nargin < 2, K = phase1_known_issues(); end
file = fullfile(R.folder, 'report.html');
col = struct('PASS', '#1f8a4c', 'KNOWN', '#b7791f', 'FAIL', '#c53030', 'ERROR', '#c53030', ...
    'BLOCKED', '#5f6070', 'SKIPPED', '#8a8a96');
st = {R.cases.status};
h = {};
h{end+1} = '<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">';
h{end+1} = '<meta name="viewport" content="width=device-width, initial-scale=1">';
h{end+1} = '<title>Shaddad Phase 1 Tests</title><style>';
h{end+1} = [':root{--navy:#2B2F7E;--purple:#6E4FA3;--magenta:#A64B9C;--link:#2B2F7E;--bg:#ffffff;--text:#1f1f26;--muted:#5f6070;--row:#f2f2f5;--line:#d6d6de}' ...
    'body{margin:0;background:var(--bg);color:var(--text);font:15px/1.45 Calibri,Segoe UI,Arial,sans-serif}' ...
    'main{max-width:1100px;margin:0 auto;padding:24px 16px 48px}' ...
    'h1{color:var(--navy);margin:0 0 4px;font-size:28px}h2{color:var(--navy);border-bottom:2px solid var(--magenta);padding-bottom:4px;margin-top:32px}' ...
    'h3{color:var(--purple);margin:22px 0 6px}.sub{color:var(--muted);margin:0 0 18px}' ...
    '.cards{display:flex;flex-wrap:wrap;gap:10px;margin:16px 0}.card{flex:1 1 120px;border:1px solid var(--line);border-radius:8px;padding:10px 14px}' ...
    '.card b{display:block;font-size:24px}.verdict{font-size:20px;font-weight:700;padding:10px 14px;border-radius:8px;color:#fff;display:inline-block}' ...
    'table{border-collapse:collapse;width:100%;margin:6px 0 10px;font-size:14px}th{background:#2B2F7E;color:#fff;text-align:left;padding:6px 8px}' ...
    'a{color:var(--link);font-weight:700}' ...
    'td{padding:5px 8px;border-bottom:1px solid var(--line);vertical-align:top}tr:nth-child(even) td{background:var(--row)}' ...
    '.tag{display:inline-block;min-width:62px;text-align:center;color:#fff;border-radius:4px;padding:1px 6px;font-weight:700;font-size:12px}' ...
    'code{font-family:Consolas,monospace;font-size:13px}.wrap{overflow-x:auto}.muted{color:var(--muted)}' ...
    '@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#15151c;--text:#e8e8ef;--muted:#a3a3b2;--row:#1f1f29;--line:#33333f;--navy:#8f94ff;--purple:#b59cf0;--link:#a9adff}}'];
h{end+1} = '</style></head><body><main>';
h{end+1} = '<h1>Shaddad UGV &mdash; Phase 1 test report</h1>';
h{end+1} = sprintf('<p class="sub">Level <b>%s</b> &middot; run %s &middot; %d cases &middot; %.0f min &middot; MATLAB %s</p>', ...
    esc(R.level), esc(R.date), numel(R.cases), R.wall_s/60, esc(version));
h{end+1} = sprintf('<span class="verdict" style="background:%s">Verdict: %s</span>', col.(R.verdict), R.verdict);
h{end+1} = '<div class="cards">';
for s = {'PASS', 'KNOWN', 'FAIL', 'ERROR'}
    h{end+1} = sprintf('<div class="card"><b style="color:%s">%d</b>%s cases</div>', col.(s{1}), nnz(strcmp(st, s{1})), s{1}); %#ok<AGROW>
end
h{end+1} = '</div>';
h{end+1} = '<p class="muted">PASS = all checks met. KNOWN = only accepted known issues failed (see the list at the end). FAIL = a check failed that is not a known issue. ERROR = the case did not run. BLOCKED = cannot be checked on this PC. SKIPPED = test gated or not applicable.</p>';

h{end+1} = '<h2>Summary</h2><div class="wrap"><table><tr><th>Case</th><th>What it tests</th><th>Result</th><th>Checks</th><th>Key numbers</th><th>Time</th></tr>';
for k = 1:numel(R.cases)
    c = R.cases(k);
    h{end+1} = sprintf('<tr><td><a href="#%s">%s</a></td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%.1f min</td></tr>', ...
        c.id, c.id, esc(c.title), tag(c.status, col), counts(c.n), esc(c.metrics), c.wall_s/60); %#ok<AGROW>
end
h{end+1} = '</table></div>';

h{end+1} = '<h2>Details</h2>';
for k = 1:numel(R.cases)
    c = R.cases(k);
    h{end+1} = sprintf('<h3 id="%s">%s &mdash; %s %s</h3>', c.id, c.id, esc(c.title), tag(c.status, col)); %#ok<AGROW>
    if ~isempty(c.file), h{end+1} = sprintf('<p class="muted">Log: <code>%s</code></p>', esc(c.file)); end %#ok<AGROW>
    if ~isempty(c.msg), h{end+1} = sprintf('<p class="muted">Message: %s</p>', esc(c.msg(1:min(end, 600)))); end %#ok<AGROW>
    h{end+1} = '<div class="wrap"><table><tr><th>Check</th><th>Result</th><th>Value</th><th>Limit</th><th>Description</th></tr>'; %#ok<AGROW>
    for j = 1:numel(c.checks)
        x = c.checks(j);
        h{end+1} = sprintf('<tr><td><code>%s</code></td><td>%s</td><td>%s %s</td><td>%s</td><td>%s</td></tr>', ...
            esc(x.id), tag(x.status, col), num(x.value), esc(x.unit), esc(x.limit), esc(x.desc)); %#ok<AGROW>
    end
    h{end+1} = '</table></div>'; %#ok<AGROW>
end

h{end+1} = '<h2>Known issues (accepted, reported as KNOWN)</h2><div class="wrap"><table><tr><th>Case</th><th>Check</th><th>Note</th></tr>';
for k = 1:numel(K)
    h{end+1} = sprintf('<tr><td>%s</td><td><code>%s</code></td><td>%s</td></tr>', esc(K(k).case), esc(K(k).check), esc(K(k).note)); %#ok<AGROW>
end
h{end+1} = '</table></div>';
h{end+1} = '<p class="muted">Generated by <code>tests/phase1/run_phase1_tests.m</code>. Cases: <code>phase1_cases.m</code>; known issues: <code>phase1_known_issues.m</code>.</p>';
h{end+1} = '</main></body></html>';
fid = fopen(file, 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', h{:});
fclose(fid);
end

% ------------------------------------------------------------------------------------------------
function s = tag(status, col)
s = sprintf('<span class="tag" style="background:%s">%s</span>', col.(status), status);
end

function s = counts(n)
lab = {'pass', 'known', 'fail', 'blocked', 'skipped'};
p = {};
for k = 1:5
    if n(k) > 0 || k == 1, p{end+1} = sprintf('%d %s', n(k), lab{k}); end %#ok<AGROW>
end
s = strjoin(p, ', ');
end

function s = num(v)
if isnumeric(v) && isscalar(v) && ~isnan(v)
    if isinf(v), s = 'not reached'; else, s = sprintf('%.4g', v); end
else
    s = '&ndash;';
end
end

function s = esc(t)
s = char(string(t));
s = strrep(s, '&', '&amp;'); s = strrep(s, '<', '&lt;'); s = strrep(s, '>', '&gt;'); s = strrep(s, '"', '&quot;');
end
