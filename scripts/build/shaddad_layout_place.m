function [P, G, used] = shaddad_layout_place(G, S)
%SHADDAD_LAYOUT_PLACE  Step 3 of shaddad_layout: size and place the blocks of one diagram (model layout spec 7).
%
%   [P, G, used] = shaddad_layout_place(G, S)
%   G     one diagram as shaddad_layout reads it (its blocks sorted by name, their ports, the signal edges and
%         the physical nodes); returned with the blocks' sizes, port offsets and port sides after sizing
%   S     the spacing constants of shaddad_layout (its help lists them); S.fans false places without fans (no
%         block resized for a fan, no fan or outport targets)
%   P     the new position [l t r b] of each block of G, on Simulink's 5 px grid
%   used  true when fans resized a block or set a target
%   It sets the blocks' sizes (and the orientation of Connection Port blocks) in the diagram; it moves no block
%   (shaddad_layout_route does). The rules it follows are step 3 of the help of shaddad_layout: sizes, columns
%   (weighted Eades-Lin-Smyth order, longest path), helpers (L4: right before their nearest forward consumer),
%   picks (one that overlaps a block by the check's rule, shaddad_layout_rules, placed as an ordinary block,
%   the diagram placed again from the same sizes, at most 3 passes), the physical band, the order in a column,
%   the fans (I1: sizes at the members' pitch, targets for the rows), the rows, the columns' x with the label
%   room (M2), a Variant Subsystem level, the row of blocks without lines. Called by shaddad_layout.

G = local_size(G, S);
used = false;
if G.variant
    P = local_place_variant(G, S);
else
    nopick = false(G.n, 1);                                  % picks that would overlap a block: none, then
    for pass = 1:3                                           % placed as ordinary blocks
        [P, isPick, G1, used] = local_place(G, nopick, S);
        bad = isPick & local_overlapping(P, G1);
        if ~any(bad) || pass == 3, break, end
        nopick = nopick | bad;
        local_unsize(G, G1);                                 % the next pass starts from the same sizes
    end
    G = G1;
end
end

function local_unsize(G, G1)
% the blocks that a pass resized (fans, I1) back at their size before it
for k = reshape(find(any(G1.pos ~= G.pos, 2)), 1, [])
    set_param(G.h(k), 'Position', G.pos(k, :));
end
end

% ===========================================================================================
%  sizes
% ===========================================================================================
function G = local_size(G, S)
% sizes by port count, Connection Ports facing in, every block on the 5 px grid; then the positions, the
% port offsets from each block's top left corner and the side of each connection port
for k = 1:G.n
    p = G.path{k};
    ph = G.ph{k};
    if strcmp(G.type{k}, 'PMIOPort')
        o = 'right';
        if strcmpi(G.side{k}, 'Right'), o = 'left'; end
        if ~strcmp(G.orient{k}, o), set_param(p, 'Orientation', o); G.orient{k} = o; end
    end
    w = G.pos(k, 3) - G.pos(k, 1);
    h = G.pos(k, 4) - G.pos(k, 2);
    nIn = numel(ph.Inport); nOut = numel(ph.Outport);
    nSide = max([nIn + numel(ph.LConn), nOut + numel(ph.RConn), 1]);
    if any(strcmp(G.orient{k}, {'right', 'left'}))
        switch G.type{k}
            case {'SubSystem', 'ModelReference'}
                if G.linked(k)
                    h = local_grid_height(h, nSide);
                else
                    h = max(S.minSub, S.pitch*nSide);
                end
            case {'BusCreator', 'Mux'}
                h = S.pitch*max(nIn, 1);
                if local_bar(p), w = S.bar; end
            case {'BusSelector', 'Demux'}
                h = S.pitch*max(nOut, 1);
                if local_bar(p), w = S.bar; end
            otherwise
                h = local_grid_height(h, nSide);
        end
    end
    tl = 5*round(G.pos(k, 1:2)/5);
    q = [tl, tl + 5*round([w h]/5)];
    if ~isequal(q, G.pos(k, :)), set_param(p, 'Position', q); end
end
for k = 1:G.n, G.pos(k, :) = get_param(G.h(k), 'Position'); end
G.w = G.pos(:, 3) - G.pos(:, 1);
G.ht = G.pos(:, 4) - G.pos(:, 2);
G.off = zeros(size(G.PT, 1), 2);
G.pside = repmat('S', size(G.PT, 1), 1);                    % 'L', 'R', 'V' (top / bottom), 'S' signal
for r = 1:size(G.PT, 1)
    k = G.PT(r, 2);
    G.off(r, :) = get_param(G.PT(r, 1), 'Position') - G.pos(k, 1:2);
    if G.PT(r, 3) < 5, continue, end
    if strcmp(G.type{k}, 'PMIOPort')                         % the side it feeds into the diagram
        G.pside(r) = 'R';
        if strcmpi(G.side{k}, 'Right'), G.pside(r) = 'L'; end
    elseif strcmp(G.orient{k}, 'right')
        G.pside(r) = 'L' + ('R' - 'L')*(G.PT(r, 3) == 6);
    elseif strcmp(G.orient{k}, 'left')
        G.pside(r) = 'R' + ('L' - 'R')*(G.PT(r, 3) == 6);
    else
        G.pside(r) = 'V';
    end
end
end

function tf = local_bar(p)
% true for a Mux, Demux, Bus Creator or Bus Selector drawn as a bar
try
    tf = strcmpi(get_param(p, 'DisplayOption'), 'bar');
catch %#ok<CTCH> no DisplayOption: the Bus Selector, always a bar
    tf = strcmp(get_param(p, 'BlockType'), 'BusSelector');
end
end

function h = local_grid_height(h, n)
% the least height from h up that puts n ports evenly on a side onto the 5 px grid (a multiple of 10 n)
q = 10*n;
h = max(q, q*ceil(h/q - 1e-9));
end

% ===========================================================================================
%  placement
% ===========================================================================================
function P = local_place_variant(G, S)
% a Variant Subsystem level: inports and left Connection Ports, the choices in one column, outports and
% right Connection Ports (it has no lines)
col = 2*ones(G.n, 1);
key = zeros(G.n, 1);
for k = 1:G.n
    switch G.type{k}
        case 'Inport', col(k) = 1; key(k) = G.port(k);
        case 'Outport', col(k) = 3; key(k) = G.port(k);
        case 'PMIOPort', col(k) = 1 + 2*strcmpi(G.side{k}, 'Right'); key(k) = 1000 + G.port(k);
    end
end
P = zeros(G.n, 4);
x = S.x0;
for c = 1:3
    in = find(col == c);
    [~, o] = sortrows([key(in), in]);
    in = in(o);
    y = S.y0;
    for k = reshape(in, 1, [])
        y = y + S.label*(G.label(k) < 0);
        P(k, :) = [x, y, x + G.w(k), y + G.ht(k)];
        y = y + G.ht(k) + S.rowGap + S.label*(G.label(k) > 0);
    end
    if ~isempty(in), x = x + max(G.w(in)) + S.colGap; end
end
P = local_snap(P, G);
end

function [P, isPick, G, used] = local_place(G, nopick, S)
% columns, order and rows of an ordinary diagram (see the help); the blocks of nopick are no picks; G returned
% with the sizes of the blocks that fans resized; used true when fans resized a block or set a target
n = G.n;
E = G.E;
[cat, pick, final] = local_categories(G, nopick);
isPick = cat == 6;
isLay = cat <= 5;
lay = find(isLay);
% ---- weighted directed edges for the columns: a signal line weighs 1 (a Bus Selector pick hands its input
% on to its consumer), 0.01 out of a Unit Delay, Delay, Memory, Integrator or Transport Delay block, so a
% cycle is broken there first; a physical link from a right-side port to a left-side port weighs
% 1 / (members - 1) of its node, a top or bottom port taking the side its partner lacks
delay = {'UnitDelay', 'Delay', 'Memory', 'Integrator', 'DiscreteIntegrator', 'TransportDelay', ...
    'VariableTransportDelay'};
D = zeros(0, 3);
for r = 1:size(E, 1)
    u = E(r, 1); v = E(r, 3);
    if cat(u) == 6, continue, end
    if cat(v) == 6, v = final(v); end
    if v > 0 && isLay(u) && isLay(v) && u ~= v
        D(end+1, :) = [u, v, 1 - 0.99*any(strcmp(G.type{u}, delay))]; %#ok<AGROW>
    end
end
for j = 1:numel(G.nodes)
    m = G.nodes{j};
    sd = G.pside(local_prow(G, m(:, 2)));
    w = 1/max(1, size(m, 1) - 1);
    for a = 1:size(m, 1)
        for b = 1:size(m, 1)
            u = m(a, 1); v = m(b, 1);
            fwd = (sd(a) == 'R' && any(sd(b) == 'LV')) || (sd(a) == 'V' && sd(b) == 'L');
            if fwd && u ~= v && isLay(u) && isLay(v), D(end+1, :) = [u, v, w]; end %#ok<AGROW>
        end
    end
end
if ~isempty(D)
    [uv, ~, ix] = unique(D(:, 1:2), 'rows');
    D = [uv, accumarray(ix, D(:, 3))];
end
% ---- feedback edges: those that point back in the vertex order of Eades, Lin and Smyth
isIn = cat == 1 | (cat == 5 & local_cp_side(G) == 'L');
isOut = cat == 2 | (cat == 5 & local_cp_side(G) == 'R');
F = D(~local_feedback(D, lay, n), 1:2);
% ---- columns: longest path; blocks joined only by links without a direction by their neighbours; helpers
% (L4) right before their nearest forward consumer, unless an Inport block feeds them, and the same for every
% block fed by sources only (a source that is no inport, and a block that only such blocks feed; Task 6); inputs
% first, outputs last
[layer, topo] = local_longest(F, lay, n);
free = lay(~ismember(lay, F(:)) & ~isIn(lay) & ~isOut(lay));
if ~isempty(free)
    nbr = local_neighbours(G, isLay);
    for it = 1:2
        for k = reshape(free, 1, [])
            w = nbr{k}(~ismember(nbr{k}, free));
            if ~isempty(w), layer(k) = round(mean(layer(w))); end
        end
    end
end
helper = local_helpers(G, cat);
srcOnly = false(n, 1);                                       % fed by sources only (a source itself included)
for k = reshape(topo, 1, [])
    srcOnly(k) = ~isIn(k) && ~isOut(k) && all(srcOnly(F(F(:, 2) == k, 1)));
end
oc = max([0; layer(isLay & ~isIn & ~isOut)]) + 1;            % the outports' column (no pull below moves it)
for k = fliplr(reshape(topo, 1, []))                         % every consumer before its producers
    if isIn(k) || isOut(k), continue, end
    s = F(F(:, 1) == k, 2);                                  % its forward consumers
    if isempty(s), continue, end
    if helper(k) || srcOnly(k)                               % the column right before the nearest one
        t = layer(s);
        t(isOut(s)) = oc;
        layer(k) = max(layer(k), min(t) - 1);
    end
end
layer(isIn) = 0;
layer(isOut) = max([0; layer(isLay & ~isIn & ~isOut)]) + 1;
used = unique(layer(lay));
[~, layer(lay)] = ismember(layer(lay), used);                % columns 1..m without an empty one
m = numel(used);
% ---- the column graph: blocks, plus a dummy node in each column a signal line passes; A = adjacent
% pairs [node1 node2 port1 port2] (node1 one column left of node2; port 0 at a dummy or a pick's consumer)
band = ones(n, 1) + (cat == 4 | cat == 5);                   % 1 signal band, 2 physical band
nk = lay;                                                    % node -> block (0: dummy)
nl = layer(lay);
nb = band(lay);
nodeOf = zeros(n, 1);
nodeOf(lay) = 1:numel(lay);
A = zeros(0, 4);
for i = reshape(unique(E(:, 5)), 1, [])
    r = find(E(:, 5) == i);
    u = E(r(1), 1);
    if ~isLay(u), continue, end
    v = E(r, 3);
    port = E(r, 4);
    for q = reshape(find(cat(v) == 6), 1, [])                 % through a pick: the input it feeds
        [v(q), port(q)] = local_pick_target(E, v(q), pick, cat);
    end
    ok = v > 0;
    ok(ok) = isLay(v(ok)) & v(ok) ~= u;
    ok(ok) = layer(v(ok)) > layer(u);
    v = v(ok); port = port(ok);
    if isempty(v), continue, end
    prev = nodeOf(u); prevPort = E(r(1), 2);
    for l = layer(u) + 1:max(layer(v))
        for q = reshape(find(layer(v) == l), 1, [])
            A(end+1, :) = [prev, nodeOf(v(q)), prevPort, port(q)]; %#ok<AGROW>
        end
        if l < max(layer(v))
            nk(end+1, 1) = 0; nl(end+1, 1) = l; nb(end+1, 1) = 1; %#ok<AGROW>
            A(end+1, :) = [prev, numel(nk), prevPort, 0]; %#ok<AGROW>
            prev = numel(nk); prevPort = 0;
        end
    end
end
for j = 1:numel(G.nodes)
    mb = G.nodes{j};
    for a = 1:size(mb, 1) - 1
        for b = a + 1:size(mb, 1)
            u = mb(a, 1); v = mb(b, 1);
            if u == v || ~isLay(u) || ~isLay(v), continue, end
            if layer(u) + 1 == layer(v), A(end+1, :) = [nodeOf(u), nodeOf(v), mb(a, 2), mb(b, 2)]; end %#ok<AGROW>
            if layer(v) + 1 == layer(u), A(end+1, :) = [nodeOf(v), nodeOf(u), mb(b, 2), mb(a, 2)]; end %#ok<AGROW>
        end
    end
end
A = unique(A, 'rows');
% ---- order in each column: barycenter sweeps on the port positions; ports pinned in port order
nn = numel(nk);
pin = zeros(nn, 1);                                          % pin key (0: free)
for q = 1:nn
    if nk(q) > 0 && any(cat(nk(q)) == [1 2 5]), pin(q) = G.port(nk(q)); end
end
order = cell(m, 1);
for l = 1:m
    q = find(nl == l);
    [~, o] = sortrows([nb(q), pin(q), nk(q) == 0, q]);
    order{l} = q(o);
end
fr = local_fracs(G, A, nk);
best = order;
bestX = local_crossings(order, A, fr, nl, nn);
for it = 1:8
    for l = 2:m, order{l} = local_sweep(order, l, l - 1, A, fr, nb, pin, nn); end
    for l = m - 1:-1:1, order{l} = local_sweep(order, l, l + 1, A, fr, nb, pin, nn); end
    x = local_crossings(order, A, fr, nl, nn);
    if x < bestX, best = order; bestX = x; end
end
order = best;
order = local_transpose(order, A, fr, nn, nb, pin);          % adjacent exchanges (Task 6)
% ---- fans and bundles (I1): blocks resized to the port pitch at which those lines run straight, and the targets
% that put the members and the slots of their lines level with the ports they join (the column order is
% independent of the sizes: the ports of a resized block keep their fractions of its height)
ht0 = G.ht;
T = zeros(0, 3);
if S.fans, [G, T] = local_fans(G, cat, layer, A, nk, nl, nodeOf, order, S); end
used = ~isempty(T) || any(G.ht ~= ht0);
% ---- rows: the signal band from y0 (the fan targets weighted fanWeight, then snapped), the physical band below it
[hgt, lab] = local_node_sizes(G, nk);
offy = zeros(size(A, 1), 2);
for r = 1:size(A, 1)
    offy(r, :) = [local_offy(G, A(r, 3)), local_offy(G, A(r, 4))];
end
y = zeros(nn, 1);
y = local_rows(order, y, A, offy, nb, hgt, lab, nk, 1, S.y0, S, T);
y = local_fan_snap(y, T, order, nk, nl, hgt, lab, S);
bottom = S.y0;
q = find(nb == 1);
if ~isempty(q), bottom = max(y(q) + hgt(q) + S.label*(lab(q) > 0)) + S.bandGap; end
if any(nb == 2)
    y = local_rows(order, y, A, offy, nb, hgt, lab, nk, 2, bottom, S, T);
end
% ---- trunks (Task 6): every further branch of a line tree starts upright at its source port, on the block's
% right edge, so the branching trees of two blocks of one column whose spans overlap would run their trunks on
% one track when their right edges meet; such a block stands right of the column's left edge by the multiple of
% track that keeps its right edge track from theirs (local_trunks)
off = local_trunks(G, E, cat, pick, nodeOf, layer, y, S);
% ---- columns: widths, room for picks and their labels, a track for every line that turns in a gap, room for the
% name label of every line that leaves a column (M2), and for the labels left of the first column's bus element ports
W = zeros(m, 1);
for l = 1:m
    q = order{l}(nk(order{l}) > 0);
    if ~isempty(q), W(l) = max(G.w(nk(q)) + off(nk(q))); end
end
R = S.pickGap*ones(m, 1);                                    % one run for the picks in front of a column, so
for k = reshape(find(cat == 6), 1, [])                       % that they stand right-aligned as before
    if cat(pick(k)) ~= 6, R(layer(pick(k))) = max(R(layer(pick(k))), local_pick_run(G, k, S)); end
end
pickW = zeros(m, 1);
for k = reshape(find(cat == 6), 1, [])
    l = layer(final(k));
    pickW(l) = max(pickW(l), local_pick_room(G, k, pick, cat, S, R(l)));
end
lead = 0;
for k = reshape(find(cat == 1 & G.be), 1, [])
    lead = max(lead, local_label_w(G.btext{k}, S) + S.labelMargin);
end
X = zeros(m, 1);                                             % each on the 5 px grid, rounded up
if m > 0, X(1) = 5*ceil((S.x0 + max(pickW(1), lead))/5); end % m 0: no column, only blocks without lines
for l = 1:m - 1
    r = find(nl(A(:, 1)) == l);
    turn = y(A(r, 1)) + offy(r, 1) ~= y(A(r, 2)) + offy(r, 2);
    run = local_label_run(G, A(r, :), nk, turn, S);
    X(l + 1) = 5*ceil((X(l) + W(l) + max([S.colGap, S.track*(nnz(turn) + 2), run]) + pickW(l + 1))/5);
end
P = zeros(n, 4);
for q = reshape(find(nk > 0), 1, [])
    k = nk(q);
    P(k, :) = [X(nl(q)) + off(k), y(q), X(nl(q)) + off(k) + G.w(k), y(q) + G.ht(k)];
end
% ---- picks directly left of their consumers, level with the input they feed (a pick of a pick after it)
todo = reshape(find(cat == 6), 1, []);
for pass = 1:numel(todo)
    for k = todo
        c = pick(k);
        if any(todo == c), continue, end
        r = find(E(:, 1) == k & E(:, 3) == c, 1);
        yIn = P(c, 2) + local_offy(G, E(r, 4));
        top = yIn - local_offy(G, E(r, 2));
        if cat(c) == 6, run = local_pick_run(G, k, S); else, run = R(layer(c)); end
        P(k, :) = [P(c, 1) - run - G.w(k), top, P(c, 1) - run, top + G.ht(k)];
        todo(todo == k) = [];
    end
    if isempty(todo), break, end
end
% ---- blocks without lines: one row below everything
park = reshape(find(cat == 7), 1, []);
if ~isempty(park)
    top = S.y0;
    for k = reshape(find(cat ~= 7), 1, [])
        top = max(top, P(k, 4) + S.label*(G.label(k) > 0));
    end
    top = top + S.bandGap;
    x = S.x0;
    for k = park
        t = top + S.label*(G.label(k) < 0);
        P(k, :) = [x, t, x + G.w(k), t + G.ht(k)];
        x = x + G.w(k) + S.colGap;
    end
end
P = local_snap(P, G);
end

function off = local_trunks(G, E, cat, pick, nodeOf, layer, y, S)
% per block, its offset right of its column's left edge: 0, or a multiple of track. A signal block (category 3)
% whose output line tree has two or more destination ports spans the rows from its source port to the farthest of
% them (a pick's: the input its chain feeds); smart autorouting runs the trunk of such a tree upright at the
% source port, which stands on the block's right edge. In each column, in the order of their tops, each such
% block takes the least offset that puts its right edge at least track (in x) from the right edge of every block
% of the column whose span overlaps its own (3 px apart counts) and that took its offset before it; blocks of
% different widths whose right edges already lie that far apart keep offset 0.
n = G.n;
off = zeros(n, 1);
lo = inf(n, 1);
hi = -inf(n, 1);
for i = reshape(unique(E(:, 5)), 1, [])
    r = find(E(:, 5) == i);
    u = E(r(1), 1);
    if numel(r) < 2 || cat(u) ~= 3 || nodeOf(u) == 0, continue, end
    v = E(r, 3);
    p = E(r, 4);
    for q = reshape(find(cat(v) == 6), 1, [])
        [v(q), p(q)] = local_pick_target(E, v(q), pick, cat);
    end
    ok = v > 0;
    ok(ok) = nodeOf(v(ok)) > 0;
    if ~any(ok), continue, end
    ys = y(nodeOf(u)) + local_offy(G, E(r(1), 2));
    yd = arrayfun(@(a, b) y(nodeOf(a)) + local_offy(G, b), v(ok), p(ok));
    lo(u) = min([lo(u); ys; yd]);
    hi(u) = max([hi(u); ys; yd]);
end
b = find(isfinite(lo));
for l = reshape(unique(layer(b)), 1, [])
    q = b(layer(b) == l);
    [~, o] = sort(lo(q));
    q = q(o);
    for a = 2:numel(q)
        qa = q(1:a - 1);
        near = qa(hi(qa) + 3 > lo(q(a)) & lo(qa) - 3 < hi(q(a)));     % the earlier blocks whose spans overlap
        edge = G.w(near) + off(near);                        % their right edges, from the column's left edge
        while any(abs(G.w(q(a)) + off(q(a)) - edge) < S.track)
            off(q(a)) = off(q(a)) + S.track;
        end
    end
end
end

function bad = local_overlapping(P, G)
% per block: true when its rectangle overlaps another block's as shaddad_layout_check counts it (the rule of
% shaddad_layout_rules: each extended by its name label, rectangles that only touch do not count)
R = shaddad_layout_rules();
pr = R.overlaps(P, G.label);
bad = false(G.n, 1);
bad(pr(:)) = true;
end

function [cat, pick, final] = local_categories(G, nopick)
% cat: 1 inport, 2 outport, 3 signal block, 4 physical block (only connection ports), 5 Connection Port,
% 6 pick, 7 block without any line; pick: the block a pick feeds; final: the first block down a chain of
% picks that is no pick; a block of nopick is never a pick (an In Bus Element block is then an inport)
n = G.n;
E = G.E;
cat = 3*ones(n, 1);
pick = zeros(n, 1);
used = false(n, 1);
used([E(:, 1); E(:, 3)]) = true;
for j = 1:numel(G.nodes), used(G.nodes{j}(:, 1)) = true; end
for k = 1:n
    ph = G.ph{k};
    sig = numel(ph.Inport) + numel(ph.Outport) + numel(ph.Enable) + numel(ph.Trigger) + numel(ph.Ifaction) + ...
        numel(ph.Reset) + numel(ph.State);
    if isfield(ph, 'Event'), sig = sig + numel(ph.Event); end
    to = unique(E(E(:, 1) == k & E(:, 3) ~= k, 3));
    switch G.type{k}
        case 'Inport'
            cat(k) = 1;
            if G.be(k) && isscalar(to) && ~nopick(k), cat(k) = 6; pick(k) = to; end
        case 'Outport'
            cat(k) = 2;
        case 'PMIOPort'
            cat(k) = 5;
        case 'BusSelector'
            if isscalar(to) && ~nopick(k), cat(k) = 6; pick(k) = to; end
        otherwise
            if sig == 0 && numel(ph.LConn) + numel(ph.RConn) > 0, cat(k) = 4; end
    end
    if any(cat(k) == [3 4]) && ~used(k), cat(k) = 7; end
end
final = zeros(n, 1);
for k = reshape(find(cat == 6), 1, [])
    c = pick(k);
    seen = k;
    while cat(c) == 6 && ~any(seen == c)
        seen(end+1) = c; %#ok<AGROW>
        c = pick(c);
    end
    if cat(c) == 6, cat(k) = 3; pick(k) = 0; else, final(k) = c; end   % a ring of picks: none is a pick
end
end

function tf = local_helpers(G, cat)
% per block: true for a helper of L4 (a Rate Transition, Unit Delay, Data Type Conversion, Signal Specification
% or Zero-Order Hold) that no Inport block of the first column (category 1) feeds directly
tf = ismember(G.type, {'RateTransition', 'UnitDelay', 'DataTypeConversion', 'SignalSpecification', ...
    'ZeroOrderHold'});
tf(G.E(cat(G.E(:, 1)) == 1, 3)) = false;
end

function [c, port] = local_pick_target(E, k, pick, cat)
% the block at the end of the chain of picks from pick k, and its input port that the last pick feeds
while cat(pick(k)) == 6, k = pick(k); end
c = pick(k);
port = E(find(E(:, 1) == k & E(:, 3) == c, 1), 4);
end

% ===========================================================================================
%  fans (I1: the lines that converge on or diverge from one block's ports run straight)
% ===========================================================================================
function [G, T] = local_fans(G, cat, layer, A, nk, nl, nodeOf, order, S)
% A block (facing right, a multi-port block: a Subsystem of any kind, a Model block, a Bus Creator, Bus Selector,
% Mux, Demux, Vector Concatenate, Merge, Variant Source or Variant Sink) whose ports on one side each join, by one line,
% a different block that has
% no other line on its side facing it (fan-in: producers in earlier columns, each feeding one input and nothing
% else; fan-out: consumers in later columns, each fed by one output and by nothing else; at least two members) gets
% on that side the port pitch at which those members, the slots of their lines in the columns between, and the
% nodes the column order puts between them stand level with their ports (local_fan_pitch), when the fan is clean
% and that pitch is at most maxPitch (local_fan_pitch; acc: the sides that count). A block whose n outputs
% feed the n inputs of one block in order, one line each (a bundle), gets that block's height. A resized block's
% height is the least multiple of its unit (local_unit: 10 x lcm(inputs, outputs), every port on the 5 px grid)
% from the height its fans need up, never below its height; a block
% whose ports do not spread evenly at the new height keeps its size. Repeated while a size changes (heights only
% grow, at most 6 passes).
% T: targets [node anchor dy], node's top level with the anchor's top + dy: each member (its port level with the
% port it joins), the slots of its line (level with that port), a bundle's partner (its input 1 level with the
% block's output 1) and the slots of the bundle's lines, and an outport fed by a block's output that feeds nothing
% else (level with that output) and the slots of its line.
E = G.E;
n = G.n;
nIn = cellfun(@(ph) numel(ph.Inport), G.ph);
nOut = cellfun(@(ph) numel(ph.Outport), G.ph);
face = strcmp(G.orient, 'right');
cand = cat <= 3 & face;                                      % members: signal blocks and ports facing right
ok = cat == 3 & face & ismember(G.type, {'SubSystem', 'ModelReference', 'BusCreator', 'BusSelector', 'Mux', ...
    'Demux', 'Concatenate', 'Merge', 'VariantSource', 'VariantSink'});   % fan blocks: multi-port blocks
acc = false(n, 2);                                           % the fan sides that count (last pass)
for pass = 1:6
    [hgt, lab] = local_node_sizes(G, nk);
    need = zeros(n, 1);
    unit = zeros(n, 1);
    acc(:) = false;
    for b = reshape(find(ok), 1, [])
        for side = 1:2
            [M, ch] = local_fan(G, b, side, layer, cand, A, nk, nodeOf);
            if size(M, 1) < 2, continue, end
            p = local_fan_pitch(G, M, ch, order, nk, nl, hgt, lab, S, A, nodeOf(b), side);
            if ~isfinite(p), continue, end                   % not a clean fan
            need(b) = max(need(b), p*[nIn(b), nOut(b)]*((1:2)' == side));
            acc(b, side) = true;
        end
        if any(acc(b, :)), unit(b) = local_unit(nIn(b), nOut(b), acc(b, :), S); end
    end
    for b = reshape(find(ok & nOut >= 2), 1, [])
        v = local_bundle(G, E, b, nOut(b), layer);
        if v == 0 || ~ok(v), continue, end
        u = lcm(max(unit(b), local_unit(nIn(b), nOut(b), [false true], S)), ...
            max(unit(v), local_unit(nIn(v), nOut(v), [true false], S)));
        if u > S.maxUnit, continue, end
        need([b v]) = max([need(b), need(v), G.ht(b), G.ht(v)]);
        unit([b v]) = u;
    end
    changed = false;
    for b = reshape(find(need > 0), 1, [])
        H = unit(b)*ceil(max(need(b), G.ht(b))/unit(b) - 1e-9);
        if H == G.ht(b), continue, end
        [G, done] = local_resize(G, b, H);
        if ~done, ok(b) = false; acc(b, :) = false; end
        changed = true;
    end
    if ~changed, break, end
end
% the targets, at the final sizes
T = zeros(0, 3);
for b = reshape(find(ok & any(acc, 2)), 1, [])
    for side = reshape(find(acc(b, :)), 1, [])
        [M, ch] = local_fan(G, b, side, layer, cand, A, nk, nodeOf);
        if side == 1, hs = G.ph{b}.Inport; else, hs = G.ph{b}.Outport; end
        for i = 1:size(M, 1)
            dy = local_offy(G, hs(M(i, 3)));
            s = ch{i}(2:end);
            T = [T; nodeOf(M(i, 1)), nodeOf(b), dy - local_offy(G, M(i, 2)); ...
                s, repmat([nodeOf(b), dy], numel(s), 1)]; %#ok<AGROW>
        end
    end
end
for b = reshape(find(ok & nOut >= 2), 1, [])
    v = local_bundle(G, E, b, nOut(b), layer);
    if v == 0 || G.ht(v) ~= G.ht(b), continue, end            % equal heights: equal pitches
    hs = G.ph{b}.Outport;
    T(end+1, :) = [nodeOf(v), nodeOf(b), local_offy(G, hs(1)) - local_offy(G, G.ph{v}.Inport(1))]; %#ok<AGROW>
    for k = 1:numel(hs)
        s = local_fan_slots(A, nk, nodeOf(b), hs(k), 1);
        T = [T; s, repmat([nodeOf(b), local_offy(G, hs(k))], numel(s), 1)]; %#ok<AGROW>
    end
end
% an outport (last column, L2) whose line comes from a block's output that feeds nothing else: level with that
% output, with the slots of its line (a straight chain to the edge)
for v = reshape(find(cat == 2 & face), 1, [])
    r = find(E(:, 3) == v);
    if numel(r) ~= 1 || any(T(:, 1) == nodeOf(v)), continue, end
    u = E(r, 1);
    if nodeOf(u) == 0 || layer(u) >= layer(v) || nnz(E(:, 2) == E(r, 2)) ~= 1, continue, end
    dy = local_offy(G, E(r, 2));
    s = local_fan_slots(A, nk, nodeOf(u), E(r, 2), 1);
    T = [T; nodeOf(v), nodeOf(u), dy - local_offy(G, E(r, 4)); s, repmat([nodeOf(u), dy], numel(s), 1)]; %#ok<AGROW>
end
end

function [M, ch] = local_fan(G, b, side, layer, cand, A, nk, nodeOf)
% block b's fan on one side (1 inputs, 2 outputs): its members M (local_fan_members) and, per member, the nodes of
% its line from the member's end: the member's node, then the slots of the line in the columns between
M = local_fan_members(G, G.E, b, side, layer, cand);
ch = cell(size(M, 1), 1);
for i = 1:size(M, 1)
    ch{i} = [nodeOf(M(i, 1)); local_fan_slots(A, nk, nodeOf(M(i, 1)), M(i, 2), side)];
end
end

function M = local_fan_members(G, E, b, side, layer, cand)
% the members of block b's fan on one side: rows [member memberPort index] for each port of that side whose one
% line joins a candidate block (cand) in an earlier column (inputs) or a later one (outputs) through an output port
% of the producer and an input port of the consumer, that block having no other line on its side facing b (fan-in:
% it feeds nothing else; fan-out: nothing else feeds it)
ph = G.ph{b};
if side == 1, hs = ph.Inport; else, hs = ph.Outport; end
M = zeros(0, 3);
for k = 1:numel(hs)
    if side == 1
        r = find(E(:, 4) == hs(k));
        if numel(r) ~= 1, continue, end
        u = E(r, 1); up = E(r, 2);
        if ~cand(u) || layer(u) >= layer(b) || nnz(E(:, 1) == u) ~= 1, continue, end
        if G.PT(local_prow(G, up), 3) ~= 2, continue, end
    else
        r = find(E(:, 2) == hs(k));
        if numel(r) ~= 1, continue, end
        u = E(r, 3); up = E(r, 4);
        if ~cand(u) || layer(u) <= layer(b) || nnz(E(:, 3) == u) ~= 1, continue, end
        if G.PT(local_prow(G, up), 3) ~= 1, continue, end
    end
    M(end+1, :) = [u, up, k]; %#ok<AGROW>
end
end

function s = local_fan_slots(A, nk, q, h, side)
% the slots (dummy nodes) of the line at port h of node q, column by column away from q: towards its destination
% (side 1: q is the line's source) or towards its source (side 2: q is its destination)
s = zeros(0, 1);
if side == 1
    r = find(A(:, 1) == q & A(:, 3) == h, 1);
    while ~isempty(r) && nk(A(r, 2)) == 0
        s(end+1, 1) = A(r, 2); %#ok<AGROW>
        r = find(A(:, 1) == A(r, 2), 1);
    end
else
    r = find(A(:, 2) == q & A(:, 4) == h, 1);
    while ~isempty(r) && nk(A(r, 1)) == 0
        s(end+1, 1) = A(r, 1); %#ok<AGROW>
        r = find(A(:, 2) == A(r, 1), 1);
    end
end
end

function p = local_fan_pitch(G, M, ch, order, nk, nl, hgt, lab, S, A, qb, side)
% the least port pitch (rounded up to 10 px, at least pitch) at which the members M of block qb's fan (a node of
% the column graph; side 1 inputs, 2 outputs) and the slots of their lines stand level with their ports: in each
% column they pass, two of them next in port order need the gaps of local_stack (local_node_gap) from the one to
% the other over every node the column order puts between them. Inf when the fan is not clean (it does not count):
% the column order puts two of them the other way round; or a node between two of them is a slot of a line that
% does not end at the block (it would cross the fan's lines), or a block that joins anything but the fan's members
% and slots (it would be pushed off its own lines); or the pitch is above maxPitch
p = S.pitch;
q = cell2mat(ch);
for c = reshape(unique(nl(q)), 1, [])
    F = zeros(0, 3);                                         % [node index portOffset]
    for i = 1:size(M, 1)
        x = ch{i}(nl(ch{i}) == c);
        if isempty(x), continue, end
        o = 0;
        if nk(x) > 0, o = local_offy(G, M(i, 2)); end
        F(end+1, :) = [x, M(i, 3), o]; %#ok<AGROW>
    end
    if size(F, 1) < 2, continue, end
    F = sortrows(F, 2);
    col = order{c};
    [~, at] = ismember(F(:, 1), col);
    for i = 1:size(F, 1) - 1
        if at(i + 1) <= at(i), p = Inf; return, end
        for j = at(i) + 1:at(i + 1) - 1
            if ~local_rides(col(j), q, qb, side, A, nk), p = Inf; return, end
        end
        g = 0;
        for j = at(i):at(i + 1) - 1, g = g + local_node_gap(col(j), col(j + 1), hgt, lab, nk, S); end
        p = max(p, (g - F(i, 3) + F(i + 1, 3))/(F(i + 1, 2) - F(i, 2)));
    end
end
p = 10*ceil(p/10 - 1e-9);
if p > S.maxPitch, p = Inf; end
end

function tf = local_rides(x, q, qb, side, A, nk)
% true when node x, between two nodes q of block qb's fan in a column, does not spoil the fan: a slot of a line that
% ends at the block (fan-in, side 1) or starts there (fan-out, side 2), or a block whose neighbours in the column
% graph are all nodes of the fan
if nk(x) > 0
    nb = [A(A(:, 1) == x, 2); A(A(:, 2) == x, 1)];
    tf = ~isempty(nb) && all(ismember(nb, q));
    return
end
e = 2 + (side == 2)*(-1);                                    % the end the line is followed to: 2 forward, 1 back
while nk(x) == 0
    if side == 1, r = find(A(:, 1) == x, 1); else, r = find(A(:, 2) == x, 1); end
    if isempty(r), tf = false; return, end
    x = A(r, e);
end
tf = x == qb;
end

function u = local_unit(nIn, nOut, sides, S)
% the height unit of a resized block: 10 x lcm(inputs, outputs) puts every port on the 5 px grid; above maxUnit,
% 10 x lcm of the port counts of the fan sides only, and above it still, 10 x the larger fan side's port count
u = 10*lcm(max(nIn, 1), max(nOut, 1));
if u > S.maxUnit
    c = [nIn, nOut].*sides;
    u = 10*lcm(max(c(1), 1), max(c(2), 1));
    if u > S.maxUnit, u = 10*max(c); end
end
end

function v = local_bundle(G, E, b, nOut, layer)
% the block in a later column whose inputs 1..n block b's outputs 1..n feed in order, one line each, when it has
% exactly those inputs and they come from nothing else (a bundle); 0 otherwise
v = 0;
ph = G.ph{b};
for k = 1:nOut
    r = find(E(:, 2) == ph.Outport(k));
    if numel(r) ~= 1, v = 0; return, end
    pr = local_prow(G, E(r, 4));
    if G.PT(pr, 3) ~= 1 || G.PT(pr, 4) ~= k || (k > 1 && E(r, 3) ~= v), v = 0; return, end
    v = E(r, 3);
end
if v == b || numel(G.ph{v}.Inport) ~= nOut || layer(v) <= layer(b), v = 0; end
end

function [G, done] = local_resize(G, b, H)
% block b at height H, its top left corner kept; done false (and the block as it was) when its input or output
% ports do not spread evenly over that height (H (2k - 1) / 2n from the top, within 1 px)
p0 = G.pos(b, :);
q = [p0(1:3), p0(2) + H];
set_param(G.h(b), 'Position', q);
rows = find(G.PT(:, 2) == b);
off = zeros(numel(rows), 2);
for i = 1:numel(rows), off(i, :) = get_param(G.PT(rows(i), 1), 'Position') - q(1:2); end
done = true;
for kind = 1:2
    r = find(G.PT(rows, 3) == kind);
    if isempty(r), continue, end
    [~, o] = sort(G.PT(rows(r), 4));
    r = r(o);
    want = H*(2*(1:numel(r))' - 1)/(2*numel(r));
    if any(abs(off(r, 2) - want) > 1), done = false; end
end
if ~done
    set_param(G.h(b), 'Position', p0);
    return
end
G.pos(b, :) = q;
G.ht(b) = H;
G.off(rows, :) = off;
end

function y = local_fan_snap(y, T, order, nk, nl, hgt, lab, S)
% each target node of T exactly at its target (on the 5 px grid) where its column allows it: the gaps of
% local_stack to its neighbours there and the top of the band; passes until nothing moves (at most 4)
for pass = 1:4
    moved = false;
    for r = 1:size(T, 1)
        q = T(r, 1);
        t = 5*round((y(T(r, 2)) + T(r, 3))/5);
        if y(q) == t || t < S.y0 + S.label*(nk(q) > 0 && lab(q) < 0), continue, end
        o = order{nl(q)};
        i = find(o == q, 1);
        if i > 1 && t < y(o(i - 1)) + local_node_gap(o(i - 1), q, hgt, lab, nk, S), continue, end
        if i < numel(o) && y(o(i + 1)) < t + local_node_gap(q, o(i + 1), hgt, lab, nk, S), continue, end
        y(q) = t;
        moved = true;
    end
    if ~moved, break, end
end
end

function [hgt, lab] = local_node_sizes(G, nk)
% height and name label side of each node of the column graph (a slot: 0 and 0)
hgt = zeros(numel(nk), 1);
lab = zeros(numel(nk), 1);
hgt(nk > 0) = G.ht(nk(nk > 0));
lab(nk > 0) = G.label(nk(nk > 0));
end

function g = local_node_gap(a, z, hgt, lab, nk, S)
% the least distance from the top of node a to the top of node z right below it in a column: rowGap between two
% blocks, dummy next to a slot, plus label for a name label between them
g = S.dummy;
if nk(a) > 0 && nk(z) > 0, g = S.rowGap; end
g = hgt(a) + g + S.label*(nk(a) > 0 && lab(a) > 0) + S.label*(nk(z) > 0 && lab(z) < 0);
end

function s = local_cp_side(G)
% per block: 'L' for a Connection Port on the diagram's left side, 'R' on its right side, ' ' otherwise
s = repmat(' ', G.n, 1);
for k = 1:G.n
    if strcmp(G.type{k}, 'PMIOPort'), s(k) = 'L' + ('R' - 'L')*strcmpi(G.side{k}, 'Right'); end
end
end

function r = local_prow(G, h)
% rows of the port table of port handles h
[~, r] = ismember(h, G.PT(:, 1));
end

function v = local_offy(G, h)
% the y offset of port h from its block's top (0 for port 0: a dummy or a pick's consumer)
v = 0;
if h ~= 0, v = G.off(local_prow(G, h), 2); end
end

function nbr = local_neighbours(G, isLay)
% per block, the other column blocks it shares a physical node with
nbr = cell(G.n, 1);
for j = 1:numel(G.nodes)
    b = unique(G.nodes{j}(:, 1));
    b = b(isLay(b));
    for k = reshape(b, 1, []), nbr{k} = [nbr{k}; b(b ~= k)]; end
end
end

function back = local_feedback(D, lay, n)
% the edges of D [from to weight] that point back in the vertex order of the greedy heuristic of Eades, Lin
% and Smyth: sinks go to the back, sources (and blocks without edges) to the front, otherwise the vertex with
% the largest out-weight minus in-weight to the front; ties by index, that is by block name
alive = false(n, 1);
alive(lay) = true;
front = zeros(0, 1);
rear = zeros(0, 1);
outw = zeros(n, 1);
inw = zeros(n, 1);
while any(alive)
    while true
        e = alive(D(:, 1)) & alive(D(:, 2));
        outw = accumarray([D(e, 1); n], [D(e, 3); 0]);
        inw = accumarray([D(e, 2); n], [D(e, 3); 0]);
        s = find(alive & outw == 0 & inw > 0);
        if ~isempty(s), rear = [s; rear]; alive(s) = false; continue, end %#ok<AGROW>
        s = find(alive & inw == 0);
        if ~isempty(s), front = [front; s]; alive(s) = false; continue, end %#ok<AGROW>
        break
    end
    c = find(alive);
    if isempty(c), break, end
    [~, i] = max(outw(c) - inw(c));
    front(end+1, 1) = c(i); %#ok<AGROW>
    alive(c(i)) = false;
end
pos = zeros(n, 1);
pos([front; rear]) = 1:numel(front) + numel(rear);
back = pos(D(:, 1)) > pos(D(:, 2));
end

function [layer, topo] = local_longest(F, lay, n)
% longest path from the sources over the acyclic edges F (Kahn's order, smallest index first)
layer = zeros(n, 1);
indeg = accumarray([F(:, 2); n], [ones(size(F, 1), 1); 0]);
avail = sort(lay(indeg(lay) == 0));
topo = zeros(0, 1);
while ~isempty(avail)
    v = avail(1);
    avail(1) = [];
    topo(end+1, 1) = v; %#ok<AGROW>
    for w = reshape(F(F(:, 1) == v, 2), 1, [])
        layer(w) = max(layer(w), layer(v) + 1);
        indeg(w) = indeg(w) - 1;
        if indeg(w) == 0, avail = sort([avail; w]); end
    end
end
end

function fr = local_fracs(G, A, nk)
% for each adjacent pair, where its ports sit on their nodes: the port's y offset over the block height
% (0.5 at a dummy or a pick's consumer)
fr = 0.5*ones(size(A, 1), 2);
for r = 1:size(A, 1)
    for e = 1:2
        k = nk(A(r, e));
        if k > 0 && A(r, e + 2) ~= 0 && G.ht(k) > 0
            fr(r, e) = min(max(local_offy(G, A(r, e + 2))/G.ht(k), 0), 1);
        end
    end
end
end

function o = local_sweep(order, l, ref, A, fr, nb, pin, nn)
% reorder column l by the barycenters of its neighbours in column ref (keeping the bands apart and the
% pinned ports in port order)
rank = zeros(nn, 1);
rank(order{ref}) = 1:numel(order{ref});
cur = zeros(nn, 1);
cur(order{l}) = 1:numel(order{l});
q = order{l};
bary = cur(q);
if ref < l, me = 2; other = 1; else, me = 1; other = 2; end
for i = 1:numel(q)
    r = A(:, me) == q(i);
    if any(r)
        bary(i) = mean(rank(A(r, other)) + fr(r, other));
    else
        bary(i) = NaN;
    end
end
% a node without neighbours there keeps its place among the others
for i = 1:numel(q)
    if isnan(bary(i))
        lo = bary(1:i-1); lo = lo(~isnan(lo));
        if isempty(lo), bary(i) = 0; else, bary(i) = lo(end); end
    end
end
[~, s] = sortrows([nb(q), bary, cur(q)]);
o = q(s);
% the pinned ports back in port order on the places they hold
p = find(pin(o) > 0);
if numel(p) > 1
    [~, s] = sortrows([nb(o(p)), pin(o(p))]);
    o(p) = o(p(s));
end
end

function order = local_transpose(order, A, fr, nn, nb, pin)
% adjacent exchanges after the barycenter sweeps (the transpose heuristic): two neighbours of a column (one band,
% neither a pinned port) swap places when that lowers the crossings between their own adjacent pairs, the only
% crossings a swap changes; passes over every column until no swap helps (at most 10)
rank = zeros(nn, 1);
for l = 1:numel(order), rank(order{l}) = 1:numel(order{l}); end
for pass = 1:10
    moved = false;
    for l = 1:numel(order)
        o = order{l};
        for i = 1:numel(o) - 1
            a = o(i); b = o(i + 1);
            if nb(a) ~= nb(b) || pin(a) > 0 || pin(b) > 0, continue, end
            c0 = local_pair_crossings(a, b, rank, A, fr);
            rank(a) = i + 1; rank(b) = i;
            c1 = local_pair_crossings(a, b, rank, A, fr);
            if c1 < c0
                o([i, i + 1]) = [b, a];
                moved = true;
            else
                rank(a) = i; rank(b) = i + 1;
            end
        end
        order{l} = o;
    end
    if ~moved, break, end
end
end

function c = local_pair_crossings(a, b, rank, A, fr)
% the crossings between the adjacent pairs of node a and those of node b, on each side of their column
c = 0;
for e = 1:2                                                  % e: the end at a / b (2 left side, 1 right side)
    o = 3 - e;
    ra = find(A(:, e) == a); rb = find(A(:, e) == b);
    if isempty(ra) || isempty(rb), continue, end
    pa = rank(A(ra, o)) + fr(ra, o); qa = rank(a) + fr(ra, e);
    pb = rank(A(rb, o)) + fr(rb, o); qb = rank(b) + fr(rb, e);
    c = c + nnz((pa - pb').*(qa - qb') < 0);
end
end

function x = local_crossings(order, A, fr, nl, nn)
% the crossings between adjacent columns of the adjacent pairs A, by rank and port position
rank = zeros(nn, 1);
for l = 1:numel(order), rank(order{l}) = 1:numel(order{l}); end
x = 0;
for l = 1:numel(order) - 1
    r = find(nl(A(:, 1)) == l);
    a = rank(A(r, 1)) + fr(r, 1);
    b = rank(A(r, 2)) + fr(r, 2);
    for i = 1:numel(r) - 1
        x = x + nnz((a(i) - a(i+1:end)).*(b(i) - b(i+1:end)) < 0);
    end
end
end

function y = local_rows(order, y, A, offy, nb, hgt, lab, nk, band, top, S, T)
% rows of one band: start stacked, then sweeps that move each node level with its neighbours' ports (least
% squares under the column order and the gaps, pool adjacent violators); a node with fan targets T (local_fans)
% wants the mean of its targets instead, with the weight fanWeight per target
m = numel(order);
for l = 1:m
    q = order{l}(nb(order{l}) == band);
    y(q) = local_stack(zeros(numel(q), 1) - Inf, q, hgt, lab, nk, top, S, ones(numel(q), 1));
end
for it = 1:5
    for l = [2:m, m-1:-1:1, 2:m]
        q = order{l}(nb(order{l}) == band);
        if isempty(q), continue, end
        want = nan(numel(q), 1);
        w = ones(numel(q), 1);
        for i = 1:numel(q)
            f = T(:, 1) == q(i);
            if any(f)
                want(i) = mean(y(T(f, 2)) + T(f, 3));
                w(i) = S.fanWeight*nnz(f);
                continue
            end
            r1 = A(:, 2) == q(i) & nb(A(:, 1)) == band;
            r2 = A(:, 1) == q(i) & nb(A(:, 2)) == band;
            t = [y(A(r1, 1)) + offy(r1, 1) - offy(r1, 2); y(A(r2, 2)) + offy(r2, 2) - offy(r2, 1)];
            if ~isempty(t), want(i) = median(t); end
        end
        want(isnan(want)) = y(q(isnan(want)));
        y(q) = local_stack(want, q, hgt, lab, nk, top, S, w);
    end
end
end

function y = local_stack(want, q, hgt, lab, nk, top, S, w)
% tops of the nodes q (in order, top to bottom) as near their wanted tops as the gaps allow (least squares with
% the weights w, by pool adjacent violators), the first at least top
k = numel(q);
y = zeros(0, 1);
if k == 0, return, end
gap = zeros(k, 1);                                           % from the top of node i to the top of node i+1
for i = 1:k - 1, gap(i) = local_node_gap(q(i), q(i + 1), hgt, lab, nk, S); end
offs = [0; cumsum(gap(1:end-1))];
z = want - offs;                                             % non-decreasing z keeps every gap
lvl = z; wt = ones(k, 1); len = ones(k, 1); nb = 0;
for i = 1:k
    nb = nb + 1;
    lvl(nb) = z(i); wt(nb) = w(i); len(nb) = 1;
    while nb > 1 && lvl(nb - 1) > lvl(nb)
        lvl(nb - 1) = (lvl(nb - 1)*wt(nb - 1) + lvl(nb)*wt(nb))/(wt(nb - 1) + wt(nb));
        wt(nb - 1) = wt(nb - 1) + wt(nb);
        len(nb - 1) = len(nb - 1) + len(nb);
        nb = nb - 1;
    end
end
z = reshape(repelem(lvl(1:nb), len(1:nb)), [], 1);
first = top + S.label*(nk(q(1)) > 0 && lab(q(1)) < 0);
z = max(z, first);
z(isinf(z)) = first;
y = 5*round((z + offs)/5);
for i = 2:k                                                  % rounding never closes a gap
    y(i) = max(y(i), y(i - 1) + gap(i - 1));
end
end

function w = local_pick_room(G, k, pick, cat, S, R)
% the room a chain of picks from k needs in front of its consumer's column: per pick the label an In Bus Element
% draws left of its dot (M2), its width and its run (local_pick_run; the last pick's run is R, the one run of all
% the picks in front of that column)
w = 0;
while true
    c = pick(k);
    if cat(c) == 6, run = local_pick_run(G, k, S); else, run = R; end
    w = w + local_pick_lead(G, k, S) + G.w(k) + run;
    if cat(c) ~= 6, return, end
    k = c;
end
end

function w = local_pick_lead(G, k, S)
% the room for the label a pick draws left of its dot: an In Bus Element's 'port . element' (M2); 0 otherwise
w = 0;
if strcmp(G.type{k}, 'Inport') && G.be(k), w = local_label_w(G.btext{k}, S) + S.labelMargin; end
end

function run = local_pick_run(G, k, S)
% the run from a pick's right edge to its consumer's left edge: at least pickGap, and room from port to port for
% the widest name label of its lines plus labelMargin (the label is drawn along that run, M2)
run = S.pickGap;
for i = reshape(unique(G.E(G.E(:, 1) == k, 5)), 1, [])
    w = local_label_w(G.ltext{i}, S);
    if w == 0, continue, end
    r = find(G.E(:, 5) == i, 1);
    run = max(run, w + S.labelMargin + local_stub(G, G.E(r, 2)) + local_stub(G, G.E(r, 4)));
end
run = 5*ceil(run/5);
end

function run = local_label_run(G, Ar, nk, turn, S)
% the gap the lines of the adjacent pairs Ar need between two columns for their name labels (M2): from port to
% port the label's width plus labelMargin, twice that for a line that turns in the gap (Simulink draws the label
% on the line's first segment, from its source; smart autorouting turns a line about midway); pairs from a slot (a
% line passing the column) carry no label there
run = 0;
for i = 1:size(Ar, 1)
    if nk(Ar(i, 1)) == 0 || Ar(i, 3) == 0, continue, end
    t = unique(G.E(G.E(:, 2) == Ar(i, 3), 5));
    for j = reshape(t, 1, [])
        w = local_label_w(G.ltext{j}, S);
        if w == 0, continue, end
        run = max(run, (w + S.labelMargin)*(1 + turn(i)) + local_stub(G, Ar(i, 3)) + local_stub(G, Ar(i, 4)));
    end
end
end

function d = local_stub(G, h)
% how far port h lies outside its block's edge, the side its line leaves or enters (0 for port 0, a slot)
d = 0;
if h == 0, return, end
r = local_prow(G, h);
k = G.PT(r, 2);
switch G.PT(r, 3)
    case 1, d = max(0, -G.off(r, 1));
    case 2, d = max(0, G.off(r, 1) - G.w(k));
end
end

function w = local_label_w(txt, S)
% the estimated width of a label: labelChar per character of its longest line (0 for no text)
w = 0;
if isempty(txt), return, end
w = S.labelChar*max(cellfun(@numel, strsplit(char(txt), newline)));
end

function P = local_snap(P, G)
% every block on the 5 px grid, at its own size
P(:, 1:2) = 5*round(P(:, 1:2)/5);
P(:, 3:4) = P(:, 1:2) + [G.w, G.ht];
end
