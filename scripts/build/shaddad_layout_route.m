function nl = shaddad_layout_route(diag, G, C, P)
%SHADDAD_LAYOUT_ROUTE  Step 4 of shaddad_layout: move the blocks and add every line back (model layout spec 7).
%
%   nl = shaddad_layout_route(diag, G, C, P)
%   diag  the diagram; G its blocks as shaddad_layout_place returns them; C its connections by port handle (the
%         second output of shaddad_layout_fingerprint); P the new block positions
%   nl    the connections added back (one per signal destination port, one per physical link)
%   Every line of the diagram is deleted, the blocks are moved to P, and each connection is added back between
%   the same port handles with add_line(diag, src, dst, 'autorouting', 'smart'): the signal trees, the physical
%   nodes as chains, then the repair of stacked lines, the riser order and the through-block re-route (step 4
%   of the help of shaddad_layout). It scores the routing with shaddad_layout_rules, the rules of
%   shaddad_layout_check. It sets points by hand only in the riser order (local_risers, Task 6 fix round 3):
%   the two points of a riser (a vertical piece) move to another x; every end point stays. Called by
%   shaddad_layout.

local_delete_lines(diag);
for k = 1:G.n
    if ~isequal(G.pos(k, :), P(k, :)), set_param(G.h(k), 'Position', P(k, :)); end
end
nl = local_route(diag, G, C);
end

% ===========================================================================================
%  lines
% ===========================================================================================
function local_delete_lines(diag)
% delete every line of the diagram (a deleted root takes its branches along)
for pass = 1:3
    L = local_lines(diag);
    if isempty(L), return, end
    for k = 1:numel(L)
        try
            delete_line(L(k));
        catch %#ok<CTCH> gone with a line deleted before it
        end
    end
end
if ~isempty(local_lines(diag))
    error('shaddad_layout:fingerprint', '%s: %d lines could not be deleted', diag, numel(local_lines(diag)));
end
end

function L = local_lines(diag)
% the line segments directly in the diagram
L = find_system(diag, 'SearchDepth', 1, 'FindAll', 'on', 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'Type', 'line');
if ~isempty(L)
    par = get_param(L, 'Parent');
    L = L(strcmp(cellstr(par), diag));
end
end

% ===========================================================================================
%  routing
% ===========================================================================================
function nl = local_route(diag, G, C)
% add every connection back (local_route_once). In a dense diagram smart autorouting may still run a line
% through a block (it weighs blocks against the lines already there; the same line added first goes
% around): such lines are added first in a new routing of the whole diagram, at most twice, and the routing
% that scores best as shaddad_layout_check counts it (lines through blocks, line_overlaps, crossings) is kept
first = zeros(0, 1);
tried = {first};
nl = local_route_once(diag, G, C, first);
[scores, bad] = local_route_score(diag, G, C);
for pass = 1:2
    if isempty(bad), break, end
    first = [first; bad(~ismember(bad, first))]; %#ok<AGROW>
    local_delete_lines(diag);
    local_route_once(diag, G, C, first);
    [sc, bad] = local_route_score(diag, G, C);
    tried{end+1} = first; %#ok<AGROW>
    scores(end+1, :) = sc; %#ok<AGROW>
end
[~, best] = sortrows([scores, (1:size(scores, 1))']);
if best(1) ~= size(scores, 1)
    local_delete_lines(diag);
    local_route_once(diag, G, C, tried{best(1)});
end
end

function nl = local_route_once(diag, G, C, first)
% add every connection back by port handle with smart autorouting: the lines of first (owners: +i signal
% tree i, -j physical node j) before all others; then the signal trees from the left, each to its farthest
% destination (in y) first; then each physical node as a chain, the nearest member next; then the repair and
% the riser order
x = zeros(numel(C.src), 2);
for i = 1:numel(C.src), x(i, :) = get_param(C.src(i), 'Position'); end
[~, o] = sortrows([x, (1:numel(C.src))']);
own = [reshape(first, [], 1); setdiff(reshape(o, [], 1), first, 'stable'); ...
    setdiff(-(1:numel(C.nodes))', first, 'stable')];
nl = 0;
for t = reshape(own, 1, [])
    if t > 0
        nl = nl + local_add_tree(diag, G, C, t, local_dst_order(C, t, 0));
    else
        nl = nl + local_add_node(diag, G, C, -t, 0);
    end
end
local_repair(diag, G, C);
local_risers(diag, G);
end

function [v, bad] = local_route_score(diag, G, C)
% [through_blocks line_overlaps crossings] of the diagram as shaddad_layout_check counts them, and the
% owners of the lines that run through a block (sorted)
rl = shaddad_layout_rules();
[seg, comp, owner] = local_owners(diag, C);
rc = zeros(G.n, 4);
for k = 1:G.n, rc(k, :) = get_param(G.h(k), 'Position'); end
th = rl.through(seg, rc(~G.commented, :), G.h(~G.commented));
bad = unique(owner(comp(th > 0)));
bad = bad(bad ~= 0);
S = local_pieces(seg);
v = [sum(th), 0, 0];
for i = 1:size(S, 1) - 1
    j = (i + 1:size(S, 1))';
    v(2:3) = v(2:3) + [nnz(rl.on_top(S(i, 1:4), S(j(S(j, 5) ~= S(i, 5)), 1:4))), ...
        nnz(rl.cross(S(i, 1:4), S(j(S(j, 6) ~= S(i, 6)), 1:4)))];
end
end

function [seg, comp, owner] = local_owners(diag, C)
% the segments of the diagram, the line each belongs to (shaddad_layout_rules' line: segments joined through
% LineParent / LineChildren) and each line's owner (+i: signal tree i of C, -j: physical node j of C, 0: none)
rl = shaddad_layout_rules();
seg = reshape(local_lines(diag), [], 1);
[~, ~, ~, comp] = rl.trees(seg);
owner = zeros(numel(seg), 1);
for k = 1:numel(seg)
    sp = get_param(seg(k), 'SrcPortHandle');
    if sp == -1, continue, end
    i = find(C.src == sp, 1);
    if ~isempty(i), owner(comp(k)) = i; continue, end
    for j = 1:numel(C.nodes)
        if any(C.nodes{j} == sp), owner(comp(k)) = -j; break, end
    end
end
end

function od = local_dst_order(C, i, f)
% the order in which tree i is added: destination f first (0: the farthest in y from the source), then the
% others farthest first (smart autorouting turns a tree's first line in the middle of its way, but starts
% every further branch at the source port, upright there)
d = C.dst{i};
q = get_param(C.src(i), 'Position');
p = zeros(numel(d), 2);
for j = 1:numel(d), p(j, :) = get_param(d(j), 'Position'); end
[~, od] = sortrows([-abs(p(:, 2) - q(2)), p, (1:numel(d))']);
od = reshape(od, 1, []);
if f > 0, od = [f, od(od ~= f)]; end
end

function n = local_add_tree(diag, G, C, i, od)
% add the lines of signal tree i in the order od and set its name back (not out of a Bus Selector)
d = C.dst{i};
for j = od
    try
        add_line(diag, C.src(i), d(j), 'autorouting', 'smart');
    catch tmp_lay_err
        error('shaddad_layout:fingerprint', '%s: the line %s -> %s could not be added back: %s', diag, ...
            local_pname(G, C.src(i)), local_pname(G, d(j)), tmp_lay_err.message);
    end
end
n = numel(od);
L = get_param(C.src(i), 'Line');
if ~C.busSel(i) && ~strcmp(get_param(L, 'Name'), C.name{i}), set_param(L, 'Name', C.name{i}); end
end

function n = local_add_node(diag, G, C, j, mode)
% add physical node j as a chain through its members, mode 0: from the top left member, the nearest one
% next; 1: that chain the other way; 2: by y, then x; 3: by x, then y
h = C.nodes{j};
p = zeros(numel(h), 2);
for k = 1:numel(h), p(k, :) = get_param(h(k), 'Position'); end
idx = (1:numel(h))';
switch mode
    case {0, 1}
        [~, at] = min(p(:, 1) + p(:, 2) + 1e-6*idx);
        chain = at;
        left = setdiff(idx', at);
        while ~isempty(left)
            [~, i] = min(sum(abs(p(left, :) - p(chain(end), :)), 2) + 1e-6*left');
            chain(end+1) = left(i); %#ok<AGROW>
            left(i) = [];
        end
        if mode == 1, chain = fliplr(chain); end
    case 2
        [~, chain] = sortrows([p(:, [2 1]), idx]);
    otherwise
        [~, chain] = sortrows([p, idx]);
end
chain = reshape(chain, 1, []);
for k = 1:numel(chain) - 1
    try
        add_line(diag, h(chain(k)), h(chain(k + 1)), 'autorouting', 'smart');
    catch tmp_lay_err
        error('shaddad_layout:physical', '%s: the physical link %s - %s could not be added back: %s', ...
            diag, local_pname(G, h(chain(k))), local_pname(G, h(chain(k + 1))), tmp_lay_err.message);
    end
end
n = numel(chain) - 1;
end

% ===========================================================================================
%  repair
% ===========================================================================================
function local_repair(diag, G, C)
% smart autorouting may stack two line trees on one track (a branch starts upright at its source port, so
% the trees of ports at one x can share it), and the links of a physical chain may run back over each other.
% Per pair that shaddad_layout_check would count, up to two rounds: each of the two lines is added again
% with each of its destinations first in turn, the farthest (in y) first, at most 12 (a physical node: its
% chain in the four modes of local_add_node), until one leaves the line with no piece through a block and
% none on another line's track; the way that scores best as the check counts it is kept (its pieces through
% blocks, then line_overlaps, then crossings against every other segment: the rest is the same each time)
rl = shaddad_layout_rules();
rc = zeros(G.n, 4);
for k = 1:G.n, rc(k, :) = get_param(G.h(k), 'Position'); end
rc = rc(~G.commented, :);                                    % blocks as the check counts them
bh = G.h(~G.commented);
for round = 1:2
    own = local_stacked(diag, C);                            % pairs of lines, by owner
    if isempty(own), return, end
    done = zeros(0, 1);
    for p = 1:size(own, 1)
        for t = own(p, [2 1])
            if any(done == t), continue, end
            done(end+1, 1) = t; %#ok<AGROW>
            seg = reshape(local_lines(diag), [], 1);
            [~, ~, ~, comp] = rl.trees(seg);
            mine = false(numel(seg), 1);
            if t > 0, h = C.src(t); else, h = C.nodes{-t}; end
            for q = reshape(h, 1, [])
                L = get_param(q, 'Line');
                if L ~= -1, mine = mine | comp == comp(seg == L); end
            end
            others = seg(~mine);
            O = local_pieces(others);
            if t > 0
                opts = local_dst_order(C, t, 0);
                opts = opts(1:min(numel(opts), 12));
            else
                opts = 0:3;
            end
            score = inf(numel(opts), 3);
            for k = 1:numel(opts)
                mine = local_readd(diag, G, C, t, opts(k), others);
                score(k, :) = [sum(rl.through(mine, rc, bh)), local_score(local_pieces(mine), O)];
                if all(score(k, 1:2) == 0), break, end
            end
            [~, best] = sortrows([score(1:k, :), (1:k)']);
            if best(1) ~= k, local_readd(diag, G, C, t, opts(best(1)), others); end
        end
    end
end
end

function mine = local_readd(diag, G, C, own, opt, others)
% delete one line (own > 0: signal tree own; own < 0: physical node -own; its segments are all those that are
% not others) and add it again with option opt; mine: its new segments
for pass = 1:3
    mine = setdiff(local_lines(diag), others);
    if isempty(mine), break, end
    for s = reshape(mine, 1, [])
        try
            delete_line(s);
        catch %#ok<CTCH> gone with a segment deleted before it
        end
    end
end
if own > 0
    local_add_tree(diag, G, C, own, local_dst_order(C, own, opt));
else
    local_add_node(diag, G, C, -own, opt);
end
mine = setdiff(local_lines(diag), others);
end

% ===========================================================================================
%  riser order (Task 6 fix round 3, F3-R1, a Ruling E change)
% ===========================================================================================
function local_risers(diag, G)
% Of two signal lines that change rows the same way (both down or both up) over overlapping spans of one channel
% and keep their order (a staircase: no crossing is forced), the leading line turns first: going down the lower
% one, going up the upper one. Then the pair does not cross; the other riser order crosses it twice. Smart
% autorouting places each riser where the lines added before leave room, so the trailing line may turn first. A
% riser is a vertical piece of a signal segment (not on a physical node) between two horizontal pieces that both
% run right; its run is the x range from the start of the piece before it to the end of the piece after it. Two
% risers form a staircase pair when they belong to two check trees, change rows the same way over overlapping
% spans, keep their order, and each lies inside the other's run. First every group of risers joined by such
% pairs, in which the leading riser of a pair turns later, tries two moves: the group's riser x sorted (the
% leading first), and its risers one track apart from the group's leftmost riser in that order (smart autorouting
% may pack them closer than a track), each riser inside its run; then each pair still in that order tries three
% moves: swap the two riser x, the leading riser track left of the trailing one, the trailing riser track right
% of the leading one. A change is kept when it makes none of these counts of its segments worse and one of them
% better: the lines through blocks, line_overlaps and crossings (as shaddad_layout_check counts them,
% shaddad_layout_rules), and the named lines that turn while their first piece (from the source port; it carries
% the name label) is shorter than the label room (labelChar per character of the name's longest line plus
% labelMargin, as step 3 estimates it); of a group's or a pair's moves, the one that lowers them most (lines
% through blocks first, then line_overlaps, then crossings, then short labels). Passes over the pairs until none
% is kept, at most 10. The risers are taken in the order of their points, never of handles. Only the two points
% of a riser move, so every connection and branch point stays
track = 10; labelChar = 6; labelMargin = 10;                % shaddad_layout's spacing (local_spacing)
rl = shaddad_layout_rules();
seg = reshape(local_lines(diag), [], 1);
if numel(seg) < 2, return, end
[tree, treeX, phys] = rl.trees(seg);
P = arrayfun(@(s) get_param(s, 'Points'), seg, 'UniformOutput', false);
lab = zeros(numel(seg), 1);                                  % the label room of a named line (its root segment)
for k = reshape(find(~phys), 1, [])
    nm = get_param(seg(k), 'Name');
    if ~isempty(nm) && get_param(seg(k), 'LineParent') == -1
        lab(k) = labelChar*max(cellfun(@numel, strsplit(nm, newline))) + labelMargin;
    end
end
E = zeros(0, 7);                                             % [k i x0 ys r yd x1]: a riser of segment k
for k = reshape(find(~phys), 1, [])
    p = P{k};
    for i = 2:size(p, 1) - 2
        if p(i-1, 2) == p(i, 2) && p(i-1, 1) < p(i, 1) && p(i, 1) == p(i+1, 1) && p(i, 2) ~= p(i+1, 2) && ...
                p(i+1, 2) == p(i+2, 2) && p(i+1, 1) < p(i+2, 1)
            E(end+1, :) = [k, i, p(i-1, 1), p(i, 2), p(i, 1), p(i+1, 2), p(i+2, 1)]; %#ok<AGROW>
        end
    end
end
n = size(E, 1);
if n < 2, return, end
[~, o] = sortrows(E(:, [3 4 6 7 5]));
E = E(o, :);
rc = zeros(G.n, 4);
for k = 1:G.n, rc(k, :) = get_param(G.h(k), 'Position'); end
X = struct('seg', seg, 'tree', tree, 'treeX', treeX, 'rl', rl, 'rc', rc(~G.commented, :), ...
    'bh', G.h(~G.commented), 'lab', lab);                    % blocks as the check counts them
S = rl.pieces(seg, [tree, treeX]);                           % [x1 y1 x2 y2 tree treeX k]
% groups: the risers joined by staircase pairs (union-find)
up = 1:n;
for a = 1:n - 1
    for b = reshape(a + find(local_stair(E, treeX, a, (a+1:n)')), 1, [])
        ra = local_find(up, a); rb = local_find(up, b);
        if ra ~= rb, up(max(ra, rb)) = min(ra, rb); end
    end
end
grp = arrayfun(@(a) local_find(up, a), 1:n);
for g = unique(grp)
    m = find(grp == g)';
    if numel(m) < 2 || numel(unique(E(m, 1))) < numel(m), continue, end
    d = sign(E(m(1), 6) - E(m(1), 4));
    [~, lead] = sortrows([-d*E(m, 4), -d*E(m, 6), E(m, 5)]);  % the leading riser first
    r = zeros(numel(m), 2);
    r(lead, 1) = sort(E(m, 5));                              % the group's riser x, sorted
    r(lead, 2) = min(E(m, 5)) + track*(0:numel(m) - 1)';     % one track apart from the leftmost
    if isequal(r(:, 1), E(m, 5)), continue, end              % the leading risers turn first already
    best = 0; bd = [];
    for q = 1:2
        if any(r(:, q) <= E(m, 3) | r(:, q) >= E(m, 7)), continue, end
        [v0, v1] = local_trial(X, S, P, E, m, r(:, q));
        if local_better(v1, v0) && (best == 0 || local_lexless(v1 - v0, bd)), best = q; bd = v1 - v0; end
    end
    if best > 0
        [~, ~, Q, M] = local_trial(X, S, P, E, m, r(:, best));
        [S, P, E] = local_commit(X, S, P, E, m, Q, M);
    end
end
% pairs
for pass = 1:10
    kept = 0;
    for a = 1:n - 1
        for b = reshape(a + find(local_stair(E, treeX, a, (a+1:n)')), 1, [])
            if ~local_stair(E, treeX, a, b), continue, end   % E may have changed since
            d = sign(E(a, 6) - E(a, 4));
            if d*E(a, 4) > d*E(b, 4), L = a; T = b; else, L = b; T = a; end
            if E(L, 5) <= E(T, 5), continue, end              % the leading riser turns first already
            mv = {[L; T], [E(T, 5); E(L, 5)]; L, E(T, 5) - track; T, E(L, 5) + track};
            best = 0; bd = [];
            for q = 1:size(mv, 1)
                if any(mv{q, 2} <= E(mv{q, 1}, 3) | mv{q, 2} >= E(mv{q, 1}, 7)), continue, end
                [v0, v1] = local_trial(X, S, P, E, mv{q, 1}, mv{q, 2});
                if local_better(v1, v0) && (best == 0 || local_lexless(v1 - v0, bd)), best = q; bd = v1 - v0; end
            end
            if best > 0
                [~, ~, Q, M] = local_trial(X, S, P, E, mv{best, 1}, mv{best, 2});
                [S, P, E] = local_commit(X, S, P, E, mv{best, 1}, Q, M);
                kept = kept + 1;
            end
        end
    end
    if kept == 0, break, end
end
end

function tf = local_stair(E, treeX, a, b)
% per riser b (a column of rows of E): risers a and b form a staircase pair (two check trees, the same way
% over overlapping spans, their order kept, each riser inside the other's run)
ea = E(a, :); eb = E(b, :);
lo = max(min(ea(4), ea(6)), min(eb(:, 4), eb(:, 6)));
hi = min(max(ea(4), ea(6)), max(eb(:, 4), eb(:, 6)));
tf = treeX(eb(:, 1)) ~= treeX(ea(1)) & sign(eb(:, 6) - eb(:, 4)) == sign(ea(6) - ea(4)) & lo < hi & ...
    (ea(4) - eb(:, 4)).*(ea(6) - eb(:, 6)) > 0 & eb(:, 5) > ea(3) & eb(:, 5) < ea(7) & ea(5) > eb(:, 3) & ...
    ea(5) < eb(:, 7);
end

function r = local_find(up, i)
r = i;
while up(r) ~= r, r = up(r); end
end

function [v0, v1, Q, M] = local_trial(X, S, P, E, rows, r)
% the score [through_blocks line_overlaps crossings short] of the segments of risers rows of E before (v0) and
% with those risers at x r (v1), against all other segments (short: their named lines whose first piece is shorter
% than the label room, local_short); the points are set back. Q: their new points, M: their new pieces; v1 is inf
% when Simulink does not keep the points
ks = unique(E(rows, 1));
mine = ismember(S(:, 7), ks);
v0 = [sum(X.rl.through(X.seg(ks), X.rc, X.bh)), local_score(S(mine, :), S(~mine, :)), local_short(X, P, ks)];
Q = P;
for j = 1:numel(rows)
    Q{E(rows(j), 1)}(E(rows(j), 2) + [0; 1], 1) = r(j);
end
same = true;
for k = reshape(ks, 1, [])
    set_param(X.seg(k), 'Points', Q{k});
    same = same && isequal(get_param(X.seg(k), 'Points'), Q{k});
end
M = zeros(0, 7);
for k = reshape(ks, 1, [])
    p = Q{k};
    m = size(p, 1) - 1;
    M = [M; p(1:m, :), p(2:m+1, :), repmat([X.tree(k), X.treeX(k), k], m, 1)]; %#ok<AGROW>
end
v1 = [sum(X.rl.through(X.seg(ks), X.rc, X.bh)), local_score(M, S(~mine, :)), local_short(X, Q, ks)];
if ~same, v1 = inf(1, 4); end
for k = reshape(ks, 1, []), set_param(X.seg(k), 'Points', P{k}); end
end

function [S, P, E] = local_commit(X, S, P, E, rows, Q, M)
% keep a trial of the risers rows of E: its new points Q and pieces M (local_trial)
ks = unique(E(rows, 1));
for k = reshape(ks, 1, []), set_param(X.seg(k), 'Points', Q{k}); end
S = [S(~ismember(S(:, 7), ks), :); M];
P(ks) = Q(ks);
for j = reshape(find(ismember(E(:, 1), ks)), 1, [])       % the risers of those segments, and their runs
    p = P{E(j, 1)}; i = E(j, 2);
    E(j, 3:7) = [p(i-1, 1), p(i, 2), p(i, 1), p(i+1, 2), p(i+2, 1)];
end
end

function n = local_short(X, P, ks)
% the named lines among segments ks (points P) that turn and whose first piece, which runs right from the source
% port and carries the name label, is shorter than the label room X.lab
n = 0;
for k = reshape(ks, 1, [])
    p = P{k};
    n = n + (X.lab(k) > 0 && size(p, 1) > 2 && p(1, 2) == p(2, 2) && p(2, 1) > p(1, 1) && ...
        p(2, 1) - p(1, 1) < X.lab(k));
end
end

function tf = local_better(v, w)
% v makes none of [through_blocks line_overlaps crossings short] worse than w and one of them better
tf = all(v <= w) && any(v < w);
end

function tf = local_lexless(a, b)
% a comes before b in lexicographic order
k = find(a ~= b, 1);
tf = ~isempty(k) && a(k) < b(k);
end

function P = local_pieces(seg)
% the straight pieces [x1 y1 x2 y2 tree treeX k] of segments: tree and treeX are shaddad_layout_check's line
% trees (the identities of line_overlaps and of crossings, shaddad_layout_rules), k the segment's index in seg
rl = shaddad_layout_rules();
[tree, treeX] = rl.trees(seg);
P = rl.pieces(seg, [tree, treeX]);
end

function v = local_score(M, O)
% [line_overlaps crossings] as shaddad_layout_check counts them, of the pieces M of one line against the
% pieces O of all the others and among its own pieces of different check trees (each pair once)
rl = shaddad_layout_rules();
v = [0 0];
for i = 1:size(M, 1)
    B = [O; M(i+1:end, :)];
    v = v + [nnz(rl.on_top(M(i, 1:4), B(B(:, 5) ~= M(i, 5), 1:4))), ...
        nnz(rl.cross(M(i, 1:4), B(B(:, 6) ~= M(i, 6), 1:4)))];
end
end

function own = local_stacked(diag, C)
% the pairs of segments that shaddad_layout_check counts as line_overlaps (two of its line trees on one
% track; one physical node is one tree), as pairs of the owners of the lines they belong to (+i: signal tree
% i of C, -j: physical node j of C)
rl = shaddad_layout_rules();
[seg, comp, owner] = local_owners(diag, C);
S = local_pieces(seg);
own = zeros(0, 2);
for i = 1:size(S, 1) - 1
    j = (i + 1:size(S, 1))';
    j = j(S(j, 5) ~= S(i, 5));
    for q = reshape(j(rl.on_top(S(i, 1:4), S(j, 1:4))), 1, [])
        o = reshape(owner(comp([S(i, 7); S(q, 7)])), 1, 2);
        if all(o ~= 0), own(end+1, :) = o; end %#ok<AGROW>
    end
end
own = unique(own, 'rows', 'stable');
end

function s = local_pname(G, h)
% a port as text, e.g. 'Gain/1 (output)'
r = local_prow(G, h);
kind = {'input', 'output', 'control', 'state', 'LConn', 'RConn'};
s = sprintf('%s/%d (%s)', G.name{G.PT(r, 2)}, G.PT(r, 4), kind{G.PT(r, 3)});
end

function r = local_prow(G, h)
% rows of the port table of port handles h
[~, r] = ismember(h, G.PT(:, 1));
end
