function varargout = shaddad_layout_check(target, varargin)
%SHADDAD_LAYOUT_CHECK  Per-diagram layout check of the Shaddad files and its targets (model layout spec 8).
%
%   T = shaddad_layout_check(target)                one row per diagram of the target files
%   T = shaddad_layout_check(target, 'Csv', file)   also write T to file (a path relative to the project root,
%                                                   or absolute; the folder is created when missing)
%   [pass, F] = shaddad_layout_check('targets', T)                  check the rows of T against spec 8
%   [pass, F] = shaddad_layout_check('targets', T, 'Allowance', A)  with allowance table A (default
%                                                                   shaddad_layout_allowance())
%
%   target  one of:
%             - a cellstr of file base names, e.g. {'PDU', 'tb_pack_bms'}: a block diagram that is loaded
%               already (saved or not) is checked as it is in memory, any other name is the .slx of that name
%               under models/, subsystems/ or tests/ (an unknown name is the error shaddad_layout_check:target)
%             - a folder relative to the project root, e.g. 'models', 'subsystems/power', 'tests/drive': every
%               .slx in it and below it
%             - 'all': models, subsystems and tests (153 files on 2026-10-29)
%   Diagrams (as the inventory of spec 2.1, work/layout/inv_diagrams.m): each file's root and every subsystem
%   level of that file, in every variant choice and under masks; not the inside of Subsystem Reference blocks,
%   library links or Stateflow / MATLAB Function blocks (each of those is checked in its own file). The
%   selection is shaddad_layout_diagrams, which shaddad_layout(sys, 'Recursive', true) uses too; the geometry
%   rules below (name label band and overlaps, line trees, on top, crossing, through a block) are
%   shaddad_layout_rules, which the layout engine places and scores its routing with.
%   Commented-out blocks are not counted. T, one row per diagram, the files in target order:
%     file            the .slx path relative to the project root ('' for a block diagram without a file)
%     diagram         full path of the diagram: the file root or a subsystem
%     depth           number of '/' in diagram
%     blocks          blocks directly in the diagram
%     sig_lines       signal line trees (a line with its branches)
%     phys_lines      physical line trees: one per physical node, every segment on it (joined through
%                     LineParent / LineChildren, or meeting at one of its ports; Task 4 decision 14, a Ruling E
%                     fix: before it, each physical segment counted as a tree of its own, and one without a
%                     source port as an unnamed signal line)
%     crossings       pairs of crossing segments of two different line trees (one line across a bundle of 25
%                     lines counts 25); frozen: exactly what inv_diagrams.m counts, the instrument of the
%                     54,221 that the 90 % target of spec 8 compares with, its tree identity included (up
%                     LineParent, so the segments of one physical node are trees of their own here)
%     through_blocks  (segment, block) pairs where a segment passes through the block's rectangle shrunk by
%                     2 px on each side: an exact test (a dimension of 4 px or less keeps its centre line;
%                     R2026b makes no block smaller than 5 px). Simulink reports an input port's position
%                     ~10 px before the end of the line drawn into it: that arrowhead lies outside an ordinary
%                     block and inside a round Sum's box. So only against the block that owns the port at a
%                     line's end is the line's last segment cut back to its point nearest the port's position
%                     (and the first segment out of a source port likewise; a cut only shortens). Against every
%                     other block the whole line counts, arrowhead included. No end exemption otherwise: a line
%                     through its own block or a sibling branch's block counts (decision 2 amended in fix rounds
%                     1 and 2; inv_diagrams.m sampled 25 points per segment and exempted every end of the line
%                     tree)
%     unnamed         signal line trees whose root has an empty Name, except lines out of a Bus Selector
%                     (spec L5; R2026b names those '<element>' itself and refuses another label; unlike
%                     inv_diagrams.m, Demux outputs count); a physical line is never an unnamed signal line
%     bus_sel, bus_cre, terminators, inports, outports    Bus Selector, Bus Creator, Terminator, Inport and
%                     Outport blocks
%     max_fanin       most input ports of one block
%     overlaps        pairs of blocks whose rectangles intersect, each rectangle extended by 16 px for its name
%                     label: below the block for NamePlacement 'normal', above it for 'alternate', not at all
%                     when ShowName is 'off'; rectangles that only touch do not count
%     unconnected     input, output and physical (LConn / RConn) ports without a line; not counted for the
%                     blocks directly inside a Variant Subsystem (its choices and its port blocks, which
%                     Simulink connects by port name), nor for any block of a library file (BlockDiagramType
%                     'library', at every level of it: its blocks are templates, wired in the models that link
%                     to them)
%     line_overlaps   pairs of segments of two different line trees, signal or physical, that run on top of
%                     each other: parallel, at most 2 px apart and overlapping by more than 2 px along their
%                     direction (a line tree's own branches never count, so neither do two links of one
%                     physical node; two nodes on one track do; lines that meet end to end do not)
%   A file that fails to load or to inspect gives a row with diagram 'ERROR <message>' and -1 in every count.
%   Files that are not loaded are loaded and closed again (shaddad_close_since), with every file their loading
%   brought in; nothing is saved. With 'Csv' or without an output argument, prints the number of files,
%   diagrams and errors and the totals.
%
%   Targets (spec 8): a row fails when overlaps, through_blocks, unnamed, unconnected or line_overlaps is above
%   0, when crossings is above the allowance of its diagram, and when it is an ERROR row (blocks -1).
%     F     the failing rows of T, with T's columns
%     pass  isempty(F)
%     A     table with columns diagram (full diagram path, e.g. 'PDU/Isolation'), max_crossings and reason
%           (see shaddad_layout_allowance), matched on the full path; a diagram A does not list allows 0
%           crossings. A row with an empty reason (or a malformed table) is the error
%           shaddad_layout_check:allowance; a T without the target columns is shaddad_layout_check:table.

if (ischar(target) && isrow(target)) || (isstring(target) && isscalar(target))
    target = char(target);
    if strcmpi(target, 'targets')
        ip = inputParser;
        ip.addRequired('T');
        ip.addParameter('Allowance', []);
        ip.parse(varargin{:});
        if any(strcmp(ip.UsingDefaults, 'Allowance'))
            A = shaddad_layout_allowance();
        else
            A = ip.Results.Allowance;
        end
        [varargout{1:2}] = local_targets(ip.Results.T, A);
        return
    end
end
ip = inputParser;
ip.addParameter('Csv', '');
ip.parse(varargin{:});
csv = char(ip.Results.Csv);
root = fileparts(fileparts(mfilename('fullpath')));
[names, files] = local_files(target, root);
before = find_system('SearchDepth', 0, 'type', 'block_diagram');   % block diagrams loaded before the check
cl = onCleanup(@() shaddad_close_since(before));    % closes what the check loaded, also after an error
rows = cell(0, 18);
for k = 1:numel(names)
    rows = [rows; local_file(names{k}, files{k}, root)]; %#ok<AGROW>
end
T = local_table(rows);
if ~isempty(csv)
    if isempty(regexp(csv, '^([A-Za-z]:[\\/]|[\\/])', 'once')), csv = fullfile(root, csv); end
    d = fileparts(csv);
    if ~isempty(d) && ~isfolder(d), mkdir(d); end
    writetable(T, csv);
end
if ~isempty(csv) || nargout == 0
    ok = T.blocks >= 0;
    fprintf('shaddad_layout_check: %d files, %d diagrams, %d errors\n', numel(names), nnz(ok), nnz(~ok));
    fprintf(['totals: blocks %d, signal lines %d, physical lines %d, crossings %d, through %d, unnamed %d, ' ...
        'overlaps %d, unconnected %d, line overlaps %d\n'], sum(T.blocks(ok)), sum(T.sig_lines(ok)), ...
        sum(T.phys_lines(ok)), sum(T.crossings(ok)), sum(T.through_blocks(ok)), sum(T.unnamed(ok)), ...
        sum(T.overlaps(ok)), sum(T.unconnected(ok)), sum(T.line_overlaps(ok)));
    if ~isempty(csv), fprintf('-> %s\n', csv); end
end
if nargout > 0, varargout{1} = T; end
end

% -------------------------------------------------------------------------------------------
function [names, files] = local_files(target, root)
% base names and .slx paths of the target ('' for a block diagram that is loaded and has no file)
if iscellstr(target) %#ok<ISCLSTR>
    names = reshape(target, [], 1);
    files = repmat({''}, size(names));
    f = local_slx(root, {'models', 'subsystems', 'tests'});
    base = regexprep({f.name}', '\.slx$', '');
    for k = 1:numel(names)
        if bdIsLoaded(names{k}), continue, end      % checked as it is in memory
        at = find(strcmp(base, names{k}));
        if isempty(at)
            error('shaddad_layout_check:target', 'No file %s.slx under models, subsystems or tests', names{k});
        elseif numel(at) > 1
            error('shaddad_layout_check:target', '%s.slx is in more than one folder', names{k});
        end
        files{k} = fullfile(f(at).folder, f(at).name);
    end
elseif ischar(target) && isrow(target)
    if strcmpi(target, 'all')
        f = local_slx(root, {'models', 'subsystems', 'tests'});
    elseif isfolder(fullfile(root, target))
        f = local_slx(root, {target});
    else
        error('shaddad_layout_check:target', ['target must be a cellstr of file base names, a folder of the ' ...
            'project (e.g. ''subsystems/power'') or ''all'', not ''%s'''], target);
    end
    names = regexprep({f.name}', '\.slx$', '');
    files = cellfun(@(d, n) fullfile(d, n), {f.folder}', {f.name}', 'UniformOutput', false);
else
    error('shaddad_layout_check:target', ['target must be a cellstr of file base names, a folder of the ' ...
        'project (e.g. ''subsystems/power'') or ''all''']);
end
end

function f = local_slx(root, folders)
% every .slx in the folders (relative to root) and below them, folder by folder
f = cell(numel(folders), 1);
for k = 1:numel(folders)
    f{k} = dir(fullfile(root, folders{k}, '**', '*.slx'));
end
f = vertcat(f{:});
end

function rows = local_file(name, f, root)
% the rows of one file: loaded when it is not loaded yet, closed again when this call loaded it
rows = cell(0, 18);
rel = local_rel(f, root);
try
    wasLoaded = bdIsLoaded(name);
    if ~wasLoaded, load_system(f); end
    if isempty(rel), rel = local_rel(get_param(name, 'FileName'), root); end
    d = shaddad_layout_diagrams(name);
    for j = 1:numel(d)
        m = local_metrics(d{j});
        rows(end+1, :) = {rel, d{j}, m.depth, m.blocks, m.sig, m.phys, m.cross, m.through, m.unnamed, ...
            m.bussel, m.buscre, m.term, m.inports, m.outports, m.maxfanin, m.overlaps, m.unconn, ...
            m.lineov}; %#ok<AGROW>
    end
    if ~wasLoaded && bdIsLoaded(name), close_system(name, 0); end
catch tmp_lc_err
    rows(end+1, :) = [{rel, ['ERROR ' regexprep(tmp_lc_err.message, '\s+', ' ')]}, num2cell(-ones(1, 16))];
end
end

function r = local_rel(f, root)
% a file path relative to the project root ('' for no file)
r = char(f);
if startsWith(r, [root filesep], 'IgnoreCase', true), r = r(numel(root)+2:end); end
end

function T = local_table(rows)
% the rows as a table: file and diagram as cellstr columns, the counts as double columns
names = {'file', 'diagram', 'depth', 'blocks', 'sig_lines', 'phys_lines', 'crossings', 'through_blocks', ...
    'unnamed', 'bus_sel', 'bus_cre', 'terminators', 'inports', 'outports', 'max_fanin', 'overlaps', ...
    'unconnected', 'line_overlaps'};
T = table(rows(:, 1), rows(:, 2), 'VariableNames', names(1:2));
for j = 3:numel(names)
    T.(names{j}) = reshape(double(cell2mat(rows(:, j))), [], 1);
end
end

% -------------------------------------------------------------------------------------------
function m = local_metrics(diag)
% the counts of one diagram
m.depth = numel(strfind(diag, '/'));
blk = find_system(diag, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
blk = blk(~strcmp(blk, diag));
m.blocks = numel(blk);
types = {};
if ~isempty(blk), types = get_param(blk, 'BlockType'); end
if ischar(types), types = {types}; end
m.bussel = sum(strcmp(types, 'BusSelector'));
m.buscre = sum(strcmp(types, 'BusCreator'));
m.term = sum(strcmp(types, 'Terminator'));
m.inports = sum(strcmp(types, 'Inport'));
m.outports = sum(strcmp(types, 'Outport'));
m.maxfanin = 0;
m.unconn = 0;
inVariant = local_is_variant(diag);                 % its blocks are connected by port name, not by lines
inLibrary = strcmp(get_param(bdroot(diag), 'BlockDiagramType'), 'library');   % templates, wired where linked
R = shaddad_layout_rules();                         % the rules the layout engine uses too
pos = zeros(numel(blk), 4); bh = zeros(numel(blk), 1); lab = zeros(numel(blk), 1);
for k = 1:numel(blk)
    ph = get_param(blk{k}, 'PortHandles');
    m.maxfanin = max(m.maxfanin, numel(ph.Inport));
    pos(k, :) = get_param(blk{k}, 'Position');
    bh(k) = get_param(blk{k}, 'Handle');
    lab(k) = R.label(blk{k});
    if ~inVariant && ~inLibrary
        p = [ph.Inport, ph.Outport, ph.LConn, ph.RConn];
        for j = 1:numel(p)
            if get_param(p(j), 'Line') == -1, m.unconn = m.unconn + 1; end
        end
    end
end
m.overlaps = size(R.overlaps(pos, lab), 1);
lh = find_system(diag, 'SearchDepth', 1, 'FindAll', 'on', 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(lh)
    par = get_param(lh, 'Parent');
    if ischar(par), par = {par}; end
    lh = lh(strcmp(par, diag));
end
[tree, treeX, phys] = R.trees(lh);
S = R.pieces(lh, [tree, treeX]);                     % [x1 y1 x2 y2 tree treeX k]
roots = unique(tree);
m.sig = 0; m.phys = 0; m.unnamed = 0;
for k = 1:numel(roots)
    isPhys = any(phys(tree == roots(k)));
    if isPhys
        m.phys = m.phys + 1;
    else
        m.sig = m.sig + 1;
        sb = get_param(roots(k), 'SrcBlockHandle');
        fromSel = sb ~= -1 && strcmp(get_param(sb, 'BlockType'), 'BusSelector');
        if isempty(get_param(roots(k), 'Name')) && ~fromSel, m.unnamed = m.unnamed + 1; end
    end
end
% crossings between segments of different line trees (crossings' own, frozen tree identity: treeX)
m.cross = 0;
for i = 1:size(S, 1) - 1
    j = (i+1:size(S, 1))';
    j = j(S(j, 6) ~= S(i, 6));
    if isempty(j), continue, end
    m.cross = m.cross + sum(R.cross(S(i, 1:4), S(j, 1:4)));
end
% segments passing through a block, exactly, with no end exemption (a line through its own block or a sibling
% branch's block is a line behind a block); only against the block that owns the port at a segment's end is
% the segment cut back to that port's position, against every other block it counts whole
m.through = sum(R.through(lh, pos, bh));
% segments of different line trees on top of each other
m.lineov = 0;
for i = 1:size(S, 1) - 1
    j = (i+1:size(S, 1))';
    j = j(S(j, 5) ~= S(i, 5));
    if isempty(j), continue, end
    m.lineov = m.lineov + sum(R.on_top(S(i, 1:4), S(j, 1:4)));
end
end

function tf = local_is_variant(diag)
% true when the diagram is a Variant Subsystem
tf = false;
try
    tf = strcmp(get_param(diag, 'Type'), 'block') && strcmp(get_param(diag, 'BlockType'), 'SubSystem') && ...
        strcmp(get_param(diag, 'Variant'), 'on');
catch %#ok<CTCH> a block diagram root, or a block without the parameter
end
end

% -------------------------------------------------------------------------------------------
function [pass, F] = local_targets(T, A)
% the rows of T that fail spec 8
need = {'diagram', 'blocks', 'crossings', 'through_blocks', 'unnamed', 'overlaps', 'unconnected', ...
    'line_overlaps'};
if ~istable(T) || ~all(ismember(need, T.Properties.VariableNames))
    error('shaddad_layout_check:table', 'T must be a table of shaddad_layout_check with the columns %s', ...
        strjoin(need, ', '));
end
A = local_allowance(A);
allow = zeros(height(T), 1);
[tf, at] = ismember(cellstr(T.diagram), A.diagram);
allow(tf) = A.max_crossings(at(tf));
fail = T.blocks < 0 | T.overlaps > 0 | T.through_blocks > 0 | T.unnamed > 0 | T.unconnected > 0 | ...
    T.line_overlaps > 0 | T.crossings > allow;
F = T(fail, :);
pass = isempty(F);
end

function A = local_allowance(A)
% the allowance table checked: columns diagram, max_crossings and reason, a reason in every row, whole
% max_crossings >= 0, each diagram once
cols = {'diagram', 'max_crossings', 'reason'};
if ~istable(A) || ~all(ismember(cols, A.Properties.VariableNames))
    error('shaddad_layout_check:allowance', 'The allowance must be a table with the columns %s', ...
        strjoin(cols, ', '));
end
try
    d = reshape(cellstr(A.diagram), [], 1);
    why = reshape(cellstr(A.reason), [], 1);
catch %#ok<CTCH> not text
    error('shaddad_layout_check:allowance', 'The allowance columns diagram and reason must be text');
end
noWhy = cellfun(@(s) isempty(strtrim(s)), why);
if any(noWhy)
    error('shaddad_layout_check:allowance', 'The allowance row of %s has no reason', d{find(noWhy, 1)});
end
mx = A.max_crossings;
if ~isnumeric(mx) || any(~isfinite(mx(:)) | mx(:) < 0 | mx(:) ~= round(mx(:)))
    error('shaddad_layout_check:allowance', 'max_crossings must be whole numbers >= 0');
end
[~, first] = unique(d, 'stable');
if numel(first) < numel(d)
    twice = d(setdiff(1:numel(d), first));
    error('shaddad_layout_check:allowance', 'The allowance lists %s more than once', twice{1});
end
A = table(d, double(mx(:)), why, 'VariableNames', cols);
end
