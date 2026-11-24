function T = shaddad_layout_sweep(varargin)
%SHADDAD_LAYOUT_SWEEP  Lay out every model file in memory with the layout engine and check it; never saves.
%
%   T = shaddad_layout_sweep()                      the .slx of models, subsystems and tests (the inventory's files)
%   T = shaddad_layout_sweep('Folders', folders)    the .slx in these folders and below (cellstr; relative to the
%                                                   project root, or absolute)
%   T = shaddad_layout_sweep(..., 'Csv', file)      also write T to file (relative to the project root, or
%                                                   absolute; the folder is created when missing)
%
%   For each file: load it, run shaddad_layout(name, 'Recursive', true) on it in memory, check every diagram
%   with shaddad_layout_check, then close it, and every file its loading brought in, without saving
%   (shaddad_close_since). Nothing is ever saved: the files on disk keep their bytes and dates. Libraries are
%   skipped (their blocks are templates, never rebuilt by the layout plan), and so is a file that is loaded
%   before its turn (the sweep never lays out a model someone has open; it gets an error row).
%   T   one row per diagram (model layout spec 8):
%         the 18 columns of shaddad_layout_check (file, diagram, depth, blocks, ... unconnected, line_overlaps)
%         error    '' or the error of the load or the layout; the file then has one row in the check's ERROR
%                  form (diagram 'ERROR <message>', -1 in every count)
%         seconds  the time of the file's layout, the same in each of its rows
%   Prints one line per file and the totals. Every change to the engine (shaddad_layout*.m) or to the check
%   reruns it (ledger, Task 4 decision 16); the before numbers are work/layout/inventory_before.csv.

ip = inputParser;
ip.addParameter('Folders', {'models', 'subsystems', 'tests'});
ip.addParameter('Csv', '');
ip.parse(varargin{:});
folders = cellstr(ip.Results.Folders);
csv = char(ip.Results.Csv);
root = fileparts(fileparts(mfilename('fullpath')));
f = cell(numel(folders), 1);
for k = 1:numel(folders)
    d = folders{k};
    if ~local_absolute(d), d = fullfile(root, d); end
    f{k} = dir(fullfile(d, '**', '*.slx'));
end
f = vertcat(f{:});
rows = cell(0, 1);
skipped = 0;
total = 0;
for k = 1:numel(f)
    name = regexprep(f(k).name, '\.slx$', '');
    file = fullfile(f(k).folder, f(k).name);
    before = find_system('SearchDepth', 0, 'type', 'block_diagram');
    msg = '';
    sec = NaN;
    Tk = [];
    isLib = false;
    try
        if bdIsLoaded(name)
            error('shaddad_layout_sweep:loaded', '%s is loaded already: not laid out', name);
        end
        load_system(file);
        isLib = strcmp(get_param(name, 'BlockDiagramType'), 'library');
        if ~isLib
            t0 = tic;
            shaddad_layout(name, 'Recursive', true);
            sec = toc(t0);
            Tk = shaddad_layout_check({name});
        end
    catch tmp_sw_err
        msg = regexprep(tmp_sw_err.message, '\s+', ' ');
    end
    shaddad_close_since(before);
    if isLib
        skipped = skipped + 1;
        fprintf('%3d %-32s library, skipped\n', k, name);
        continue
    end
    if isempty(Tk)
        Tk = local_error_row(file, root, msg);
    end
    Tk.error = repmat({msg}, height(Tk), 1);
    Tk.seconds = repmat(sec, height(Tk), 1);
    rows{end+1, 1} = Tk; %#ok<AGROW>
    if ~isnan(sec), total = total + sec; end
    ok = Tk.blocks >= 0;
    fprintf('%3d %-32s %3d diagrams %6.1f s  overlaps %d through %d line_overlaps %d crossings %d  %s\n', k, name, ...
        nnz(ok), sec, sum(Tk.overlaps(ok)), sum(Tk.through_blocks(ok)), sum(Tk.line_overlaps(ok)), ...
        sum(Tk.crossings(ok)), msg);
end
if isempty(rows)
    T = local_error_row('', root, '');
    T.error = {''};
    T.seconds = NaN;
    T(1, :) = [];                                            % no file: the columns without a row
else
    T = vertcat(rows{:});
end
ok = T.blocks >= 0;
fprintf(['shaddad_layout_sweep: %d files laid out, %d diagrams, %d errors, %d libraries skipped; overlaps %d, ' ...
    'through %d, line_overlaps %d, crossings %d; layout time %.0f s\n'], numel(rows) - nnz(~ok), nnz(ok), ...
    nnz(~ok), skipped, sum(T.overlaps(ok)), sum(T.through_blocks(ok)), sum(T.line_overlaps(ok)), ...
    sum(T.crossings(ok)), total);
if ~isempty(csv)
    if ~local_absolute(csv), csv = fullfile(root, csv); end
    d = fileparts(csv);
    if ~isempty(d) && ~isfolder(d), mkdir(d); end
    writetable(T, csv);
    fprintf('-> %s\n', csv);
end
end

% -------------------------------------------------------------------------------------------
function tf = local_absolute(p)
% true for an absolute path (a drive letter or a leading separator)
tf = ~isempty(regexp(p, '^([A-Za-z]:[\\/]|[\\/])', 'once'));
end

function T = local_error_row(file, root, msg)
% one row in the check's ERROR form: diagram 'ERROR <message>', -1 in every count
rel = char(file);
if startsWith(rel, [root filesep], 'IgnoreCase', true), rel = rel(numel(root)+2:end); end
names = {'file', 'diagram', 'depth', 'blocks', 'sig_lines', 'phys_lines', 'crossings', 'through_blocks', ...
    'unnamed', 'bus_sel', 'bus_cre', 'terminators', 'inports', 'outports', 'max_fanin', 'overlaps', ...
    'unconnected', 'line_overlaps'};
T = table({rel}, {['ERROR ' msg]}, 'VariableNames', names(1:2));
for j = 3:numel(names)
    T.(names{j}) = -1;
end
end
