function info = shaddad_layout(sys, varargin)
%SHADDAD_LAYOUT  Lay out a diagram: place its blocks, re-add its lines, assert its wiring (model layout spec 7).
%
%   info = shaddad_layout(sys)                    lay out the diagram sys
%   info = shaddad_layout(sys, 'Recursive', tf)   tf true: also every subsystem level of sys in its file
%   info = shaddad_layout(..., 'Keep', diagrams)  keep the block positions (and sizes) of these diagrams, a
%                                                 cellstr of full paths among those laid out (another is the
%                                                 error shaddad_layout:keep): steps 1, 2, 4 and 5 only, so
%                                                 their lines are still deleted, added back with smart
%                                                 autorouting and asserted (Task 5 decision 9: a builder that
%                                                 places a diagram itself, e.g. the top level of
%                                                 Shaddad_System); default {} (every diagram is placed)
%   sys    a block diagram name or a subsystem path that shaddad_layout_diagrams lists for its file (any
%          other path is the error shaddad_layout:diagram)
%   info   .diagrams  cellstr column of the diagrams laid out: sys, or with 'Recursive' true
%                     shaddad_layout_diagrams(sys), the selection of shaddad_layout_check (every subsystem
%                     level of the file, in every variant choice and under masks; not inside Subsystem
%                     Reference blocks, Model blocks, library links, Stateflow or MATLAB Function blocks)
%          .blocks    blocks placed, over all those diagrams
%          .lines     connections re-added: one per signal destination port, one per physical link
%
%   Each diagram, in five steps (spec 7):
%   1 Read    the blocks (commented-out ones included), and from shaddad_layout_fingerprint the signal
%             connections (source port, destination ports, line name) and the physical nodes (port sets).
%   2 Fingerprint  fp0 = shaddad_layout_fingerprint(diagram).
%   3 Place   (rules L1-L8 of spec 6.1)
%             - Sizes: subsystems, Variant Subsystems and Model blocks (not library links) get pitch x their
%               most ports on one side as height (at least minSub), Bus Creators and Muxes pitch x inputs, Bus
%               Selectors and Demuxes pitch x outputs (drawn as a bar: bar wide); any other block with side
%               ports gets the least height from its own up that puts its ports on the 5 px grid. Other widths
%               stay. A Connection Port block faces into the diagram (Orientation 'right' on the left side,
%               'left' on the right side). Fans may then make a block taller (below).
%             - Columns follow the flow (L1): longest path from the inputs over the signal lines (weight 1)
%               and the physical links (a block's right-side port before the left-side port it joins, a top
%               or bottom port taking the side its partner lacks; weight 1 / (members - 1) of the node). The
%               feedback edges are those that point back in the weighted vertex order of Eades, Lin and
%               Smyth; an edge out of a Unit Delay, Delay, Memory, Integrator or Transport Delay block weighs
%               0.01, so a cycle is broken there first. A block joined only by links without a direction
%               goes to its neighbours' column.
%             - Helpers (L4): a Rate Transition, Unit Delay, Data Type Conversion, Signal Specification or
%               Zero-Order Hold takes the latest column its forward consumers allow, the one right before its
%               nearest consumer (in front of an outport: the column next to the outports). So a chain of
%               helpers sits right in front of the block it serves, and a pick that feeds a helper in front of
%               the helper (L3). A helper that an Inport block of the first column feeds directly keeps its
%               column right after it; one without a forward consumer (a Unit Delay whose only output is a
%               cut feedback edge) stays next to its producer, and its line runs back as feedback (L1). The
%               pull runs from the outputs back (each consumer before its producers), ties by block name.
%             - Source chains (Task 6, a Ruling E change): a block fed by sources only (a source that is not
%               an inport, and a block that only such blocks feed: a Constant and the Gain it alone feeds, a
%               clock and its lookup table) takes the latest column its forward consumers allow, as a helper
%               does, in the same pull: the one right before its nearest consumer (in front of an outport:
%               the column next to the outports). So such a chain sits right in front of the block it serves,
%               not at the left edge.
%             - Inports (and In Bus Element blocks that are no pick) sit in the first column and outports
%               in the last, in port order (L2); Connection Ports in the first or last column by their
%               Side, in port order.
%             - Picks (L3): an In Bus Element block or a Bus Selector whose lines all go to one block sits
%               directly left of that block, pickGap from it, its output level with the input it feeds. A pick
%               that would overlap another block (name labels included, e.g. a Bus Selector taller than the
%               gap to the next consumer) is placed as an ordinary block instead, and the diagram again.
%             - Physical blocks (only connection ports) and Connection Ports sit in a band below the signal
%               blocks, in the same columns (L7).
%             - Within a column the blocks are ordered to reduce crossings (barycenter sweeps on the port
%               positions); a line that passes a column keeps a slot of its own there. Then (Task 6, a Ruling E
%               change) two neighbours of a column in one band, neither a pinned port, swap places when that
%               lowers the crossings of their own lines with the columns beside (adjacent exchanges, the
%               transpose heuristic), in passes over every column until no swap helps (at most 10).
%             - Fans (Task 5 fix round 1, I1): the lines that converge on or diverge from one block's ports run
%               straight. A fan is a block's side (inputs or outputs) whose ports each join, by one line, a
%               different block in an earlier column (fan-in: a producer that feeds nothing else) or a later one
%               (fan-out: a consumer that nothing else feeds), at least two such members. The block faces right and
%               is a multi-port block of one of these kinds: a Subsystem of any kind (Variant Subsystem, Subsystem
%               Reference, MATLAB Function, chart), a Model block, a Bus Creator, Bus Selector, Mux, Demux, Vector
%               Concatenate or Merge, or a Variant Source or Variant Sink; other blocks (a Switch, a Sum, a logic
%               block) keep their size. The
%               block is sized so that its port pitch on that side equals the row pitch its members need: in every
%               column the fan's lines pass, the members and the slots of their lines, next in port order, keep the
%               gaps of the rows (rowGap between two blocks, dummy next to a slot, label for a name label) over
%               every node the column order puts between them, rounded up to 10 px and at least pitch. A fan counts
%               only when it is clean: in every such column the column order keeps them in port order with nothing
%               between two of them but slots of lines that end at the block (fan-in) or start there (fan-out) and
%               blocks joined to nothing but the fan's members and slots, and the pitch is at most maxPitch
%               (otherwise straightening it would push other blocks off their own lines). A bundle (a block's n
%               outputs feeding the n inputs of one block in order, one line each) gives both blocks one height. A
%               resized block's height is the least multiple of 10 x lcm(inputs, outputs) (every port on the 5 px
%               grid; above maxUnit, of 10 x lcm of the fan sides' port counts, and above it still, of 10 x the
%               larger fan side's port count) from what its fans need up, never less than before; a block whose
%               ports would not spread evenly keeps its size (a Subsystem, Variant Subsystem, Model block, Bus
%               Creator or Mux spreads them evenly). Then the rows place each member, and each slot of its line,
%               level with the port it joins (a target weighted fanWeight in the rows' least squares, then set
%               exactly where the column's gaps allow; a bundle's second block level with the first), so each fan
%               line is one horizontal segment. The same targets put an outport fed by a block's output that feeds
%               nothing else level with that output, with the slots of its line (a straight chain to the right
%               edge). A diagram whose routing with fans leaves a line through a block or a line overlap is placed
%               and routed again without fans (no block resized for them, no fan or outport targets); the routing
%               with fewer lines through blocks, then fewer line overlaps, is kept, and a tie keeps the fans.
%             - Rows: each block moves level with the ports it connects to (least squares under the column
%               order), so straight chains run straight, at least rowGap (plus a name label) from the next.
%             - Trunks (Task 6, a Ruling E change; fix round 1): smart autorouting starts every further branch of
%               a line tree upright at its source port (step 4), on the block's right edge. A signal block whose
%               output line tree has two or more destination ports spans the rows from that port to its
%               farthest destination (a pick's: the input its chain feeds). In each column, in the order of
%               their tops, each such block stands right of the column's left edge by the least multiple of
%               track that puts its right edge at least track (in x) from the right edge of every block of the
%               column whose span overlaps its own and that took its place before it; blocks of different
%               widths whose right edges lie that far apart already stay at the left edge. The column's width
%               includes the offsets. So the trunks of two such trees never share a track.
%             - Label room (Task 5 fix round 1, M2; L6): the labels the check does not count get room. A label's
%               width is estimated as labelChar per character of its longest text line (the default font draws about
%               5 px), plus labelMargin. A pick that is an In Bus Element draws 'port . element' left of its dot:
%               the room in front of its consumer's column holds that label, the pick and its run (one run for all
%               the picks in front of a column, so that they stand right-aligned), so no block of the column before
%               reaches into the label (the first column starts that far right of x0 for the In Bus Element inports
%               there). A line's name label is drawn on its first segment from its source port: a pick's run, and
%               the gap after a column, give every named line that leaves it that width from port to port (twice it
%               when the line turns in the gap, about midway). An Out Bus Element draws its label right of its dot;
%               outports stand in the last column, so no block lies in that label.
%             - A Variant Subsystem level (L8): inports and left Connection Ports in the first column, the
%               choices stacked in one column, outports and right Connection Ports in the last.
%             - Blocks without any line: one row below everything (bandGap below y0 in a diagram that has
%               no other block, and so no column).
%   4 Route   Every line is deleted and each connection added back between the same port handles with
%             add_line(diagram, src, dst, 'autorouting', 'smart'): the signal trees from the left, each to
%             its farthest destination (in y) first (smart autorouting turns a tree's first line midway but
%             starts every further branch upright at the source port); then each physical node as a chain
%             through its members, the nearest port next, each link joining two members of the node (so it
%             keeps its member set; a star from one member stacks its branches on one track). The line names
%             are set back, except on lines out of a Bus Selector, which R2026b names '<element>' itself.
%             Repair: for every pair of segments that shaddad_layout_check would count as line_overlaps
%             (up to two rounds), each of the two lines is added again with each of its destinations first
%             in turn, farthest first, at most 12 (a physical node: its chain from the top left member both
%             ways, by y and by x), until the line runs on no other line's track and through no block, and
%             kept the way the check would score best (through_blocks, line_overlaps, crossings).
%             Riser order (Task 6 fix round 3, F3-R1, a Ruling E change): of two signal lines that change
%             rows the same way (both down or both up) over overlapping spans of one channel and keep their
%             order (a staircase: no crossing is forced), the leading line turns first, going down the lower
%             line, going up the upper one; the other order crosses the pair twice. Smart autorouting places
%             each riser (a vertical piece between two horizontal pieces that run right) where the lines
%             added before it leave room, so the trailing line may turn first. Then each group of such pairs
%             (each riser inside the other's run) tries its risers' x sorted, the leading first, and its
%             risers one track apart in that order; each pair still in the trailing order tries three moves:
%             swap the two riser x, the leading riser track left of the trailing one, or the trailing riser
%             track right of the leading one. A change is kept when it makes none of through_blocks,
%             line_overlaps, crossings and the named lines whose first piece is shorter than its label room
%             (labelChar, labelMargin) worse and one of them better, in passes until none is kept (at most
%             10). Only a riser's two points move, so every end point and branch point stays. This is the
%             only place where the engine sets line points.
%             In a dense diagram smart autorouting may still run a line through a block; such lines are added
%             first in a new routing of the whole diagram (at most twice; each routing with its repair and
%             riser order), and the routing the check would score best is kept.
%   5 Assert  Every member of every physical node is connected again (else the error shaddad_layout:physical)
%             and shaddad_layout_fingerprint(diagram) equals fp0 (else shaddad_layout:fingerprint, naming the
%             first differences). A line that cannot be added back raises the same errors.
%   The engine never renames, adds or deletes a block: it sets positions, and the orientation of Connection
%   Port blocks. It is deterministic: ties are broken by block name, never by handle or creation order, and
%   every coordinate lies on Simulink's 5 px grid.
%
%   Files: this one (the entry: the options, the diagrams, steps 1, 2 and 5), shaddad_layout_place (step 3)
%   and shaddad_layout_route (step 4). The geometry rules that steps 1, 3 and 4 use are those of
%   shaddad_layout_check, in shaddad_layout_rules (the name labels, the label band and overlaps; the line
%   trees, on top, crossing and through a block).
%
%   Spacing (px, local_spacing): x0 40 and y0 40, the top left corner; colGap 80, the least gap between
%   two columns, plus track 10 for every line that turns in that gap; rowGap 30 between two blocks of a
%   column, plus label 16 for a name label between them (the label band of shaddad_layout_rules, with which
%   shaddad_layout_check counts overlaps); dummy 20 around a line that passes a column; pitch 30, minSub 40
%   and bar 5 (sizes); pickGap 30 from a pick's right edge to its consumer; bandGap 60 from the signal band
%   to the physical band and to the row of blocks without lines; labelChar 6 per character and labelMargin 10
%   (label room); maxPitch 200, the largest pitch of a fan, maxUnit 2000, the largest height unit of a resized
%   block, and fanWeight 1e4, the weight of a fan target in the rows (fans).

ip = inputParser;
ip.addRequired('sys');
ip.addParameter('Recursive', false);
ip.addParameter('Keep', {});
ip.parse(sys, varargin{:});
if isnumeric(sys), sys = getfullname(sys); end
sys = char(sys);
if ~strcmp(get_param(sys, 'Type'), 'block_diagram') && ~any(strcmp(shaddad_layout_diagrams(bdroot(sys)), sys))
    error('shaddad_layout:diagram', ['%s is not a diagram of its file (a subsystem level that ' ...
        'shaddad_layout_diagrams lists)'], sys);
end
if logical(ip.Results.Recursive)
    d = shaddad_layout_diagrams(sys);
else
    d = {sys};
end
keep = reshape(cellstr(ip.Results.Keep), [], 1);
bad = setdiff(keep, d);
if ~isempty(bad)
    error('shaddad_layout:keep', '%s is not among the diagrams laid out (%s)', bad{1}, sys);
end
info = struct('diagrams', {reshape(d, [], 1)}, 'blocks', 0, 'lines', 0);
for k = 1:numel(d)
    [nb, nl] = local_layout(d{k}, any(strcmp(keep, d{k})));
    info.blocks = info.blocks + nb;
    info.lines = info.lines + nl;
end
end

% ===========================================================================================
function S = local_spacing()
% the spacing constants (px), listed in the help; label is the check's label band (shaddad_layout_rules)
R = shaddad_layout_rules();
S.x0 = 40;  S.y0 = 40;  S.colGap = 80;  S.track = 10;  S.rowGap = 30;  S.label = R.label_band;  S.dummy = 20;
S.pitch = 30;  S.minSub = 40;  S.bar = 5;  S.pickGap = 30;  S.bandGap = 60;
S.labelChar = 6;  S.labelMargin = 10;  S.maxUnit = 2000;  S.maxPitch = 200;  S.fanWeight = 1e4;  S.fans = true;
end

function [nb, nl] = local_layout(diag, keep)
% one diagram: read, fingerprint, place (shaddad_layout_place; with keep true the blocks stay where they are,
% at their own sizes), route (shaddad_layout_route), assert. A placement with fans whose routing leaves lines
% through blocks or line overlaps is tried again without fans; the better routing is kept (a tie keeps the fans)
[fp0, C] = shaddad_layout_fingerprint(diag);
G = local_read(diag, C);
nb = G.n;
nl = 0;
if G.n == 0, return, end
if keep
    nl = shaddad_layout_route(diag, G, C, G.pos);
else
    S = local_spacing();
    [P, G1, used] = shaddad_layout_place(G, S);
    nl = shaddad_layout_route(diag, G1, C, P);
    v = local_defects(diag, G1);
    if used && any(v > 0)
        S0 = S;
        S0.fans = false;
        local_restore(G);
        [P0, G0] = shaddad_layout_place(G, S0);
        nl = shaddad_layout_route(diag, G0, C, P0);
        v0 = local_defects(diag, G0);
        if ~(v0(1) < v(1) || (v0(1) == v(1) && v0(2) < v(2)))     % the fans route no worse: their placement again
            local_restore(G);
            [P, G1] = shaddad_layout_place(G, S);
            nl = shaddad_layout_route(diag, G1, C, P);
        end
    end
end
local_assert(diag, C, fp0);
end

function local_restore(G)
% every block back at its position and size as read (the router deletes and adds the lines itself)
for k = 1:G.n, set_param(G.h(k), 'Position', G.pos(k, :)); end
end

function v = local_defects(diag, G)
% [pieces of lines through blocks, line_overlaps] of the diagram as shaddad_layout_check counts them (the rules of
% shaddad_layout_rules, as the router scores a routing)
v = [0 0];
seg = find_system(diag, 'SearchDepth', 1, 'FindAll', 'on', 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if isempty(seg), return, end
seg = reshape(seg(strcmp(cellstr(get_param(seg, 'Parent')), diag)), [], 1);
if isempty(seg), return, end
rl = shaddad_layout_rules();
rc = zeros(G.n, 4);
for k = 1:G.n, rc(k, :) = get_param(G.h(k), 'Position'); end
v(1) = sum(rl.through(seg, rc(~G.commented, :), G.h(~G.commented)));
[tree, treeX] = rl.trees(seg);
S = rl.pieces(seg, [tree, treeX]);
for i = 1:size(S, 1) - 1
    j = (i + 1:size(S, 1))';
    v(2) = v(2) + nnz(rl.on_top(S(i, 1:4), S(j(S(j, 5) ~= S(i, 5)), 1:4)));
end
end

% ===========================================================================================
%  read
% ===========================================================================================
function G = local_read(diag, C)
% the blocks of the diagram sorted by name, their ports, and the connections of C by block
b = find_system(diag, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'IncludeCommented', 'on', 'Type', 'block');
b = b(~strcmp(b, diag));
[G.name, o] = sort(cellstr(get_param(b, 'Name')));
G.path = b(o);
G.n = numel(G.path);
G.variant = false;
try
    G.variant = strcmp(get_param(diag, 'Type'), 'block') && strcmp(get_param(diag, 'Variant'), 'on');
catch %#ok<CTCH> a block diagram root, or a block without the parameter
end
n = G.n;
R = shaddad_layout_rules();
G.h = zeros(n, 1); G.type = cell(n, 1); G.pos = zeros(n, 4); G.ph = cell(n, 1); G.orient = cell(n, 1);
G.label = zeros(n, 1); G.port = nan(n, 1); G.be = false(n, 1); G.side = repmat({''}, n, 1);
G.linked = false(n, 1); G.commented = false(n, 1);
for k = 1:n
    p = G.path{k};
    G.h(k) = get_param(p, 'Handle');
    G.commented(k) = ~strcmp(get_param(p, 'Commented'), 'off');      % the check does not count it
    G.type{k} = get_param(p, 'BlockType');
    G.pos(k, :) = get_param(p, 'Position');
    G.ph{k} = get_param(p, 'PortHandles');
    G.orient{k} = get_param(p, 'Orientation');
    G.label(k) = R.label(p);                                         % 1 below, -1 above, 0 none
    switch G.type{k}
        case {'Inport', 'Outport'}
            G.port(k) = str2double(get_param(p, 'Port'));
            G.be(k) = strcmp(get_param(p, 'IsBusElementPort'), 'on');
        case 'PMIOPort'
            G.port(k) = str2double(get_param(p, 'Port'));
            G.side{k} = get_param(p, 'Side');
    end
    try
        G.linked(k) = any(strcmp(get_param(p, 'LinkStatus'), {'resolved', 'implicit'}));
    catch %#ok<CTCH> no such parameter
    end
end
% port table: handle, block, kind (1 input, 2 output, 3 control, 4 state, 5 LConn, 6 RConn), index
f = {'Inport', 1; 'Outport', 2; 'Enable', 3; 'Trigger', 3; 'Ifaction', 3; 'Reset', 3; 'Event', 3; ...
    'State', 4; 'LConn', 5; 'RConn', 6};
PT = zeros(0, 4);
for k = 1:n
    for j = 1:size(f, 1)
        if ~isfield(G.ph{k}, f{j, 1}), continue, end
        hs = reshape(G.ph{k}.(f{j, 1}), [], 1);
        PT = [PT; hs, repmat([k f{j, 2}], numel(hs), 1), (1:numel(hs))']; %#ok<AGROW>
    end
end
G.PT = PT;
% signal edges [srcBlock srcPort dstBlock dstPort group], one per destination port
E = zeros(0, 5);
for i = 1:numel(C.src)
    sb = PT(PT(:, 1) == C.src(i), 2);
    for d = reshape(C.dst{i}, 1, [])
        E(end+1, :) = [sb, C.src(i), PT(PT(:, 1) == d, 2), d, i]; %#ok<AGROW>
    end
end
G.E = E;
% physical nodes: per node, rows [block port]
G.nodes = cell(numel(C.nodes), 1);
for j = 1:numel(C.nodes)
    h = reshape(C.nodes{j}, [], 1);
    [~, at] = ismember(h, PT(:, 1));
    G.nodes{j} = [PT(at, 2), h];
end
% the label texts the check does not count (M2): each signal tree's name label (E(:, 5) indexes it; R2026b's
% '<element>' on a line out of a Bus Selector), and the text a bus element block draws beside its dot
G.ltext = reshape(C.name, [], 1);
G.btext = repmat({''}, n, 1);
for k = reshape(find(G.be), 1, [])
    el = get_param(G.path{k}, 'Element');
    G.btext{k} = get_param(G.path{k}, 'PortName');
    if ~isempty(el), G.btext{k} = [G.btext{k} ' . ' el]; end
end
end

% ===========================================================================================
%  assert
% ===========================================================================================
function local_assert(diag, C, fp0)
% every member of every physical node connected, and the fingerprint unchanged
for j = 1:numel(C.nodes)
    h = C.nodes{j};
    for k = 1:numel(h)
        if get_param(h(k), 'Line') == -1
            error('shaddad_layout:physical', '%s: the physical port %s lost its node {%s}', diag, ...
                fp0.nodes{j}{k}, strjoin(fp0.nodes{j}, ', '));
        end
    end
end
fp1 = shaddad_layout_fingerprint(diag);
if ~isequal(fp1, fp0)
    error('shaddad_layout:fingerprint', '%s: the layout changed the fingerprint: %s', diag, local_diff(fp0, fp1));
end
end

function s = local_diff(a, b)
% the first differences between two fingerprints, as text
d = {};
src = union({a.signals.src}, {b.signals.src});
f = fieldnames(a.signals);
for k = 1:numel(src)
    x = a.signals(strcmp({a.signals.src}, src{k}));
    y = b.signals(strcmp({b.signals.src}, src{k}));
    if isempty(y)
        d{end+1} = sprintf('%s only before', src{k}); %#ok<AGROW>
        continue
    elseif isempty(x)
        d{end+1} = sprintf('%s only after', src{k}); %#ok<AGROW>
        continue
    end
    for j = 2:numel(f)
        if ~isequal(x.(f{j}), y.(f{j}))
            d{end+1} = sprintf('%s.%s %s -> %s', src{k}, f{j}, local_text(x.(f{j})), local_text(y.(f{j}))); %#ok<AGROW>
        end
    end
end
na = cellfun(@(c) strjoin(c, ', '), a.nodes, 'UniformOutput', false);
nb = cellfun(@(c) strjoin(c, ', '), b.nodes, 'UniformOutput', false);
gone = setdiff(na, nb);
new = setdiff(nb, na);
for k = 1:numel(gone), d{end+1} = sprintf('node {%s} only before', gone{k}); end %#ok<AGROW>
for k = 1:numel(new), d{end+1} = sprintf('node {%s} only after', new{k}); end %#ok<AGROW>
if isempty(d), d = {'(no listed difference)'}; end
s = strjoin(d(1:min(5, end)), '; ');
if numel(d) > 5, s = sprintf('%s; and %d more', s, numel(d) - 5); end
end

function t = local_text(v)
% a fingerprint value as text
if iscell(v), t = ['{' strjoin(v, ', ') '}']; else, t = ['''' char(v) '''']; end
end
