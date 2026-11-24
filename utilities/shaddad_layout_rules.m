function R = shaddad_layout_rules()
%SHADDAD_LAYOUT_RULES  The geometry rules of the model layout check, shared by the check and the layout engine.
%
%   R = shaddad_layout_rules()   a struct of function handles and the label band. shaddad_layout_check counts
%                                with them, and the layout engine uses them: shaddad_layout reads the name
%                                labels with R.label, shaddad_layout_place spaces the blocks by R.label_band and
%                                tests its picks with R.overlaps, shaddad_layout_route scores its repair and
%                                through-block re-route with the line rules. So the engine optimises exactly
%                                what the check measures (model layout spec 8; Task 4 decision 15, review M2).
%                                Neither keeps a copy.
%
%   s = R.label(b)              the side of block b's name label: 1 below the block (NamePlacement 'normal'),
%                               -1 above it ('alternate'), 0 none (ShowName 'off')
%   R.label_band                16, the px a name label takes below or above its block
%   P = R.overlaps(rc, lab)     the pairs [i j] (i < j) of block rectangles rc [l t r b] (one row per block)
%                               that intersect, each extended by label_band on the side of its name label lab
%                               (R.label, one per block); rectangles that only touch do not
%   [tree, treeX, phys, line] = R.trees(seg)    seg: the line segment handles of one diagram (a vector)
%       tree    per segment, the line tree that line_overlaps, sig_lines / phys_lines and unnamed count with
%               (Task 4 decision 14): a signal line's tree is found up LineParent to its root (the root's
%               handle); one physical node is one tree, every segment on it: the segments joined through
%               LineParent / LineChildren, or meeting at one of its connection ports (the handle of its
%               first segment in seg). Physical lines set no LineParent, so without this every physical
%               segment would be a tree of its own
%       treeX   per segment, the line tree that crossings counts with: up LineParent to the root, for physical
%               segments too (frozen, constraints.md: the 54,221 baseline was counted so)
%       phys    per segment, true on a physical node (its tree has a segment that starts or ends at a
%               connection port)
%       line    per segment, the index in seg of the first segment of its connected line: segments joined
%               through LineParent / LineChildren only (the branches of a signal line; the segments of a
%               physical line that meet at its branch points)
%   P = R.pieces(seg, id)       the straight pieces [x1 y1 x2 y2 id k] of the segments: id one row per segment
%                               (one or more columns), k the segment's index in seg
%   t = R.on_top(a, B)          rows of B that run on top of segment a = [x1 y1 x2 y2]: parallel, at most 2 px
%                               from its line and overlapping it by more than 2 px along it
%   t = R.cross(a, B)           rows of B that cross segment a in both interiors (parallel or collinear never)
%   n = R.through(seg, rc, bh)  per segment, the (piece, block) pairs where a piece passes through a block's
%                               rectangle rc [l t r b] (block handles bh) shrunk by 2 px on each side, a
%                               dimension of 4 px or less kept as its centre line. Against the block that owns
%                               the port at a line's end, that end piece is cut back to the port's position
%                               (R.ends); against every other block the whole piece counts
%   E = R.ends(L, p, isRoot)    the pieces of segment L (points p) as the block owning a port at one of its ends
%                               sees them: [x1 y1 x2 y2 srcOwner dstOwner], owner -1 where nothing is cut
%   e = R.cut_at(a, e, q), tf = R.inside(seg, rc), I = R.cut(I, J)    the steps of R.through and R.ends

R = struct('label', @local_label, 'label_band', local_band(), 'overlaps', @local_overlaps, ...
    'trees', @local_trees, 'pieces', @local_pieces, 'on_top', @local_on_top, 'cross', @local_cross, ...
    'through', @local_through, 'ends', @local_ends, 'cut_at', @local_cut_at, 'inside', @local_inside, ...
    'cut', @local_cut);
end

% -------------------------------------------------------------------------------------------
function b = local_band()
% the label band: the px a name label takes below or above its block
b = 16;
end

function s = local_label(b)
% the side of block b's name label: 1 below the block, -1 above it, 0 no label
s = 0;
if strcmp(get_param(b, 'ShowName'), 'off'), return, end
if strcmp(get_param(b, 'NamePlacement'), 'alternate'), s = -1; else, s = 1; end
end

function P = local_overlaps(rc, lab)
% the pairs [i j] (i < j) of block rectangles rc that intersect (touching does not count), each extended by
% the label band on the side of its name label lab
r = rc;
r(lab > 0, 4) = r(lab > 0, 4) + local_band();
r(lab < 0, 2) = r(lab < 0, 2) - local_band();
P = zeros(0, 2);
for i = 1:size(r, 1) - 1
    j = (i+1:size(r, 1))';
    j = j(r(i, 1) < r(j, 3) & r(j, 1) < r(i, 3) & r(i, 2) < r(j, 4) & r(j, 2) < r(i, 4));
    P = [P; repmat(i, numel(j), 1), j]; %#ok<AGROW>
end
end

% -------------------------------------------------------------------------------------------
function [tree, treeX, phys, line] = local_trees(seg)
% the line tree identities of the segments seg (see the help)
seg = reshape(seg, [], 1);
n = numel(seg);
treeX = seg;                                         % up LineParent to the root (crossings' identity)
for k = 1:n
    while get_param(treeX(k), 'LineParent') ~= -1, treeX(k) = get_param(treeX(k), 'LineParent'); end
end
up = 1:n;                                            % connected lines: joined through LineParent / LineChildren
for k = 1:n
    nb = [reshape(get_param(seg(k), 'LineChildren'), [], 1); get_param(seg(k), 'LineParent')];
    [tf, at] = ismember(nb, seg);
    for j = reshape(at(tf), 1, [])
        up = local_union(up, k, j);
    end
end
line = arrayfun(@(k) local_root(up, k), (1:n)');
% physical lines: a segment that starts or ends at a connection port marks its line physical, and the lines
% that meet at one connection port are one node
conn = false(n, 1);
at = containers.Map('KeyType', 'double', 'ValueType', 'double');   % connection port -> first segment on it
for k = 1:n
    p = [get_param(seg(k), 'SrcPortHandle'); reshape(get_param(seg(k), 'DstPortHandle'), [], 1)];
    for h = reshape(p(p ~= -1), 1, [])
        if ~strcmp(get_param(h, 'PortType'), 'connection'), continue, end
        conn(k) = true;
        if isKey(at, h), up = local_union(up, k, at(h)); else, at(h) = k; end
    end
end
node = arrayfun(@(k) local_root(up, k), (1:n)');
isNode = false(n, 1);
isNode(node(conn)) = true;
phys = isNode(node);
tree = treeX;
tree(phys) = seg(node(phys));                        % one physical node, one tree
end

function up = local_union(up, i, j)
% join the sets of i and j in the union-find array up (the smaller root stays)
a = local_root(up, i); b = local_root(up, j);
if a ~= b, up(max(a, b)) = min(a, b); end
end

function r = local_root(up, i)
% the root of i in the union-find array up
r = i;
while up(r) ~= r, r = up(r); end
end

function P = local_pieces(seg, id)
% the straight pieces [x1 y1 x2 y2 id k] of the segments
P = zeros(0, 5 + size(id, 2));
for k = 1:numel(seg)
    p = get_param(seg(k), 'Points');
    m = size(p, 1) - 1;
    P = [P; p(1:m, :), p(2:m+1, :), repmat([id(k, :), k], m, 1)]; %#ok<AGROW>
end
end

function t = local_on_top(a, B)
% segments of B that run on top of segment a: parallel to it, at most 2 px from its line and overlapping it by
% more than 2 px along it (a zero-length segment never)
p = a(1:2); r = a(3:4) - p; len = hypot(r(1), r(2));
if len == 0, t = false(size(B, 1), 1); return, end
u = r/len;
s = B(:, 3:4) - B(:, 1:2);
par = abs(r(1)*s(:, 2) - r(2)*s(:, 1)) <= 1e-9*len*hypot(s(:, 1), s(:, 2)) & any(s ~= 0, 2);
q0 = B(:, 1:2) - p; q1 = B(:, 3:4) - p;
dist = abs(u(1)*q0(:, 2) - u(2)*q0(:, 1));          % B's start from a's line (= the gap when parallel)
t0 = q0*u'; t1 = q1*u';                             % B's ends along a
ov = min(len, max(t0, t1)) - max(0, min(t0, t1));    % length of the overlap along a
t = par & dist <= 2 + 1e-9 & ov > 2 + 1e-9;
end

function t = local_cross(a, B)
% segments of B that cross segment a in both interiors (parallel and collinear segments do not cross)
p = a(1:2); r = a(3:4) - p;
q = B(:, 1:2); s = B(:, 3:4) - q;
rxs = r(1)*s(:, 2) - r(2)*s(:, 1);
qp = q - p;
tt = (qp(:, 1).*s(:, 2) - qp(:, 2).*s(:, 1)) ./ rxs;
uu = (qp(:, 1)*r(2) - qp(:, 2)*r(1)) ./ rxs;
t = rxs ~= 0 & tt > 1e-9 & tt < 1 - 1e-9 & uu > 1e-9 & uu < 1 - 1e-9;
end

function n = local_through(seg, rc, bh)
% per segment, the (piece, block) pairs where a piece passes through a block (see the help); a block whose
% rectangle the piece's box does not reach is never inside, so only those are tested
n = zeros(numel(seg), 1);
for k = 1:numel(seg)
    p = get_param(seg(k), 'Points');
    m = size(p, 1) - 1;
    if m < 1, continue, end
    E = local_ends(seg(k), p, get_param(seg(k), 'LineParent') == -1);
    for i = 1:m
        lo = min(p(i, :), p(i+1, :)); hi = max(p(i, :), p(i+1, :));
        for b = reshape(find(rc(:, 1) <= hi(1) & rc(:, 3) >= lo(1) & rc(:, 2) <= hi(2) & rc(:, 4) >= lo(2)), 1, [])
            s = [p(i, :), p(i+1, :)];
            if bh(b) == E(i, 5), s(1:2) = E(i, 1:2); end   % its own source block: from the port's position
            if bh(b) == E(i, 6), s(3:4) = E(i, 3:4); end   % its own destination block: to the port's position
            n(k) = n(k) + local_inside(s, rc(b, :));
        end
    end
end
end

function E = local_ends(L, p, isRoot)
% per segment of line L (points p), the segment as the block owning the port at one of its ends sees it:
% [x1 y1 x2 y2 srcOwner dstOwner], owner -1 where nothing is cut. Simulink reports an input port's position
% ~10 px before the end of the line drawn into it (the arrowhead, outside an ordinary block, inside a round
% Sum's box), so against the destination block the last segment of a line into a port is cut back to its point
% nearest that port's position, and against the source block the first segment of a root line likewise (a cut
% only shortens: where the line ends at the port position nothing changes)
n = size(p, 1) - 1;
E = [p(1:n, :), p(2:n+1, :), -ones(n, 2)];
if n < 1, return, end
if isRoot
    sp = get_param(L, 'SrcPortHandle');
    if sp ~= -1
        E(1, 1:2) = local_cut_at(p(2, :), p(1, :), get_param(sp, 'Position'));
        E(1, 5) = get_param(L, 'SrcBlockHandle');
    end
end
if isempty(get_param(L, 'LineChildren'))
    dp = get_param(L, 'DstPortHandle');
    if isscalar(dp) && dp ~= -1
        E(n, 3:4) = local_cut_at(p(n, :), p(n+1, :), get_param(dp, 'Position'));
        E(n, 6) = get_param(L, 'DstBlockHandle');
    end
end
end

function e = local_cut_at(a, e, q)
% the end e of segment a -> e moved back along the segment to its point nearest q (e itself when that is e)
d = e - a;
n = d*d';
if n == 0, return, end
e = a + min(max(((q - a)*d')/n, 0), 1)*d;
end

function tf = local_inside(seg, rc)
% true when segment seg = [x1 y1 x2 y2] passes through rect rc = [l t r b] shrunk by 2 px on each side (the
% open inside), where a dimension of 4 px or less shrinks to its centre line instead (closed). Exact: the
% parameters s in [0, 1] of the segment inside each dimension are intersected as intervals
I = [0 1 1 1];                                       % [lo hi loClosed hiClosed] of s
for d = 1:2
    p0 = seg(d); dp = seg(d+2) - seg(d);
    a = rc(d); b = rc(d+2);
    if b - a <= 4                                    % a narrow dimension: its centre line
        c = (a + b)/2;
        if dp == 0
            if p0 ~= c, tf = false; return, end
        else
            I = local_cut(I, [(c - p0)/dp, (c - p0)/dp, 1, 1]);
        end
    else                                             % the open interval (a + 2, b - 2)
        lo = a + 2; hi = b - 2;
        if dp == 0
            if ~(p0 > lo && p0 < hi), tf = false; return, end
        else
            s1 = (lo - p0)/dp; s2 = (hi - p0)/dp;
            I = local_cut(I, [min(s1, s2), max(s1, s2), 0, 0]);
        end
    end
    if isempty(I), tf = false; return, end
end
tf = true;
end

function I = local_cut(I, J)
% intersection of two intervals [lo hi loClosed hiClosed]; [] when it is empty
if J(1) > I(1) || (J(1) == I(1) && ~J(3)), lo = J(1); loC = J(3); else, lo = I(1); loC = I(3); end
if J(2) < I(2) || (J(2) == I(2) && ~J(4)), hi = J(2); hiC = J(4); else, hi = I(2); hiC = I(4); end
if lo < hi || (lo == hi && loC && hiC), I = [lo hi loC hiC]; else, I = []; end
end
