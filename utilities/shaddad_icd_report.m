function outFile = shaddad_icd_report(outFile)
%SHADDAD_ICD_REPORT  Generate docs/icd.md = docs/icd_static_sections.md + generated bus tables.
%   shaddad_icd_report()            writes <project>/docs/icd.md
%   shaddad_icd_report(file)        writes to file
%   Reads bus definitions with shaddad_buses('defs') (no base-workspace access, no models).
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(outFile), outFile = fullfile(root, 'docs', 'icd.md'); end
static = fileread(fullfile(root, 'docs', 'icd_static_sections.md'));
defs = shaddad_buses('defs');
owners = {'buses_sim','integration'; 'buses_structure','structure'; 'buses_power','power'; ...
          'buses_drive','drive'; 'buses_electronics','electronics / rc'; 'buses_sensors','sensors'; ...
          'buses_gnc','gnc'};
own = cell(1, numel(defs));
for k = 1:size(owners,1)
    nm = {feval(owners{k,1}).name};
    own(ismember({defs.name}, nm)) = owners(k,2);
end

L = {};
L{end+1} = '## 10. Bus definitions (generated from buses/buses_<domain>.m)';
L{end+1} = '';
L{end+1} = sprintf('%d buses. Ts = nominal producer sample time (0 continuous, Inf constant, inherited = sub-bus type used inside other buses).', numel(defs));
L{end+1} = '';
L{end+1} = '### 10.1 Index';
L{end+1} = '';
L{end+1} = '| Bus | Owner | Ts [s] | Producer | Consumers | Elements |';
L{end+1} = '|---|---|---|---|---|---|';
for k = 1:numel(defs)
    d = defs(k);
    L{end+1} = sprintf('| [%s](#%s) | %s | %s | %s | %s | %d |', d.name, lower(d.name), own{k}, ...
        local_ts(d.ts), local_esc(d.producer), local_esc(d.consumers), numel(d.elems)); %#ok<AGROW>
end
L{end+1} = '';
L{end+1} = '### 10.2 Elements';
for k = 1:numel(defs)
    d = defs(k);
    L{end+1} = ''; %#ok<AGROW>
    L{end+1} = sprintf('<a id="%s"></a>', lower(d.name)); %#ok<AGROW>
    L{end+1} = sprintf('#### %s', d.name); %#ok<AGROW>
    L{end+1} = ''; %#ok<AGROW>
    L{end+1} = sprintf('%s  ', local_esc(d.desc)); %#ok<AGROW>
    L{end+1} = sprintf('**Owner:** %s. **Ts:** %s s. **Producer:** %s. **Consumers:** %s.', ...
        own{k}, local_ts(d.ts), local_esc(d.producer), local_esc(d.consumers)); %#ok<AGROW>
    L{end+1} = ''; %#ok<AGROW>
    L{end+1} = '| # | Element | Type | Dims | Unit | Range | Description |'; %#ok<AGROW>
    L{end+1} = '|---|---|---|---|---|---|---|'; %#ok<AGROW>
    for m = 1:numel(d.elems)
        e = d.elems{m};
        L{end+1} = sprintf('| %d | `%s` | %s | %s | %s | %s | %s |', m, e.name, local_esc(e.dtype), ...
            local_dims(e.dims), local_unit(e.unit), local_range(e.min, e.max), local_esc(e.desc)); %#ok<AGROW>
    end
end
txt = strrep(strrep([static newline strjoin(L, newline) newline], sprintf('\r\n'), newline), newline, sprintf('\r\n'));  % CRLF throughout, as the static file
fid = fopen(outFile, 'w', 'n', 'UTF-8');
if fid < 0, error('shaddad_icd_report:open', 'Cannot write %s', outFile); end
c = onCleanup(@() fclose(fid));
fwrite(fid, txt, 'char');
end

function s = local_ts(ts)
if isinf(ts), s = 'Inf'; elseif ts == -1, s = 'inherited'; elseif ts == 0, s = '0'; else, s = num2str(ts); end
end
function s = local_dims(d)
if isscalar(d), s = num2str(d); else, s = ['[' strjoin(arrayfun(@num2str, d, 'UniformOutput', false), ' x ') ']']; end
end
function s = local_unit(u)
if isempty(u), s = '-'; else, s = u; end
end
function s = local_range(mn, mx)
if isempty(mn) && isempty(mx), s = '-'; return; end
a = '-inf'; b = 'inf';
if ~isempty(mn), a = num2str(mn); end
if ~isempty(mx), b = num2str(mx); end
s = ['[' a ', ' b ']'];
end
function s = local_esc(s)
s = strrep(char(s), '|', '\|');
end
