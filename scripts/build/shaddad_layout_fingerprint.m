function [fp, C] = shaddad_layout_fingerprint(sys)
%SHADDAD_LAYOUT_FINGERPRINT  Canonical wiring and line / port settings of one diagram (model layout spec 7).
%
%   fp = shaddad_layout_fingerprint(sys)
%   [fp, C] = shaddad_layout_fingerprint(sys)    also the connections by port handle (for shaddad_layout)
%   sys   one diagram: a block diagram name or a subsystem path (only the blocks directly in it)
%
%   fp    struct; isequal(fp1, fp2) is the check that a layout changed no connection and no line or port
%         setting (shaddad_layout asserts it for every diagram it lays out):
%     .signals  struct column, one element per signal source port of the blocks in sys (every Outport and
%               State port, connected or not), sorted by src:
%                 src    'Block/k' (output port k) or 'Block/State'
%                 dst    1xK cellstr, the sorted destination ports of its line: 'Block/k' (input port k) or a
%                        control port 'Block/Enable', 'Block/Trigger', 'Block/Ifaction', 'Block/Reset',
%                        'Block/Event'; {} without a line
%                 name   the line's name (R2026b names a line out of a Bus Selector '<element>'); without a
%                        line, the signal name the port keeps
%                 DataLogging, DataLoggingNameMode, DataLoggingName, DataLoggingDecimateData,
%                 DataLoggingDecimation, TestPoint, MustResolveToSignalObject
%                        the port parameters of those names
%                 StorageClass, RTWStorageTypeQualifier
%                        the storage-class parameters a port has in R2026b
%                 SignalObject
%                        the storage class of a Simulink.Signal object embedded in the port,
%                        '<class>:<CoderInfo.StorageClass>', '' without one
%                 MappedStorageClass
%                        the storage class of the port in the model's code mapping (coder.mapping.api.get
%                        and getSignal), '' when the model has no code mapping or does not map the port
%     .nodes    cell column, one 1xK sorted cellstr per physical node (two or more connection ports joined
%               by physical lines: 'Block/LConn k', 'Block/RConn k'), the nodes sorted by first member
%   Blocks are named by their name in sys, ports by kind and number: no handle, SID or UUID enters fp, and
%   nothing beyond the list above (no rebuild churn such as resistor auto_seed, the overflow-warning
%   suppressions Simulink writes, or fresh UUIDs). Commented-out blocks count like the others.
%
%   C     the same connections by handle, in fp's order: .src (column of source ports with at least one
%         destination), .dst (cell column, the destination ports of each, ordered as in fp), .name (cell
%         column), .busSel (true where the source block is a Bus Selector), .nodes (cell column, the member
%         ports of each node in fp's order)
%
%   Nodes come from PortConnectivity (the other ports on each connection port's node), joined over shared
%   ports, so a node drawn as several separate lines that meet at one port is still one node.

sys = char(sys);
blk = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'MatchFilter', @Simulink.match.allVariants, 'IncludeCommented', 'on', 'Type', 'block');
blk = blk(~strcmp(blk, sys));
nm = cellstr(get_param(blk, 'Name'));
[nm, o] = sort(nm);
blk = blk(o);
% every port of every block: handle, name, kind (1 input, 2 output, 3 control, 4 state, 5 connection)
kinds = {'Inport', 1; 'Outport', 2; 'Enable', 3; 'Trigger', 3; 'Ifaction', 3; 'Reset', 3; 'Event', 3; ...
    'State', 4; 'LConn', 5; 'RConn', 5};
H = zeros(0, 1); N = cell(0, 1); K = zeros(0, 1); B = zeros(0, 1);
for k = 1:numel(blk)
    ph = get_param(blk{k}, 'PortHandles');
    for j = 1:size(kinds, 1)
        if ~isfield(ph, kinds{j, 1}), continue, end
        hs = ph.(kinds{j, 1});
        for i = 1:numel(hs)
            switch kinds{j, 2}
                case {1, 2}
                    s = sprintf('%s/%d', nm{k}, i);
                case 5
                    s = sprintf('%s/%s %d', nm{k}, kinds{j, 1}, i);
                otherwise
                    s = [nm{k} '/' kinds{j, 1}];
                    if numel(hs) > 1, s = sprintf('%s %d', s, i); end
            end
            H(end+1, 1) = hs(i); N{end+1, 1} = s; K(end+1, 1) = kinds{j, 2}; B(end+1, 1) = k; %#ok<AGROW>
        end
    end
end
% ---- signals
props = {'DataLogging', 'DataLoggingNameMode', 'DataLoggingName', 'DataLoggingDecimateData', ...
    'DataLoggingDecimation', 'TestPoint', 'MustResolveToSignalObject', 'StorageClass', 'RTWStorageTypeQualifier'};
[mapH, mapSC] = local_mapping(bdroot(sys));
src = find(K == 2 | K == 4);
[~, o] = sort(N(src));
src = src(o);
n = numel(src);
f = [{'src', 'dst', 'name'}, props, {'SignalObject', 'MappedStorageClass'}];
V = cell(n, numel(f));
C = struct('src', zeros(0, 1), 'dst', {cell(0, 1)}, 'name', {cell(0, 1)}, 'busSel', false(0, 1), ...
    'nodes', {cell(0, 1)});
for i = 1:n
    h = H(src(i));
    L = get_param(h, 'Line');
    if L ~= -1
        d = local_dst(L);
        [tf, at] = ismember(d, H);
        [dn, o] = sort(N(at(tf)));
        dh = H(at(tf));
        dh = dh(o);
        name = get_param(L, 'Name');
    else
        dn = cell(0, 1); dh = zeros(0, 1);
        name = get_param(h, 'Name');
    end
    V(i, 1:3) = {N{src(i)}, reshape(dn, 1, []), name};
    for j = 1:numel(props)
        V{i, 3+j} = local_get(h, props{j});
    end
    V{i, end-1} = local_sigobj(h);
    m = find(mapH == h, 1);
    if isempty(m), V{i, end} = ''; else, V{i, end} = mapSC{m}; end
    if ~isempty(dh)
        C.src(end+1, 1) = h;
        C.dst{end+1, 1} = reshape(dh, 1, []);
        C.name{end+1, 1} = name;
        C.busSel(end+1, 1) = strcmp(get_param(blk{B(src(i))}, 'BlockType'), 'BusSelector');
    end
end
args = reshape([f; num2cell(V, 1)], 1, []);
for j = 2:2:numel(args), args{j} = reshape(args{j}, [], 1); end
fp.signals = reshape(struct(args{:}), [], 1);
if n == 0, fp.signals = reshape(cell2struct(cell(numel(f), 0), f, 1), [], 1); end
% ---- physical nodes: ports joined by PortConnectivity, over shared ports
c = find(K == 5);
hc = H(c);
up = 1:numel(c);
for k = 1:numel(blk)
    if ~any(B(c) == k), continue, end
    ph = get_param(blk{k}, 'PortHandles');
    pc = get_param(blk{k}, 'PortConnectivity');
    for e = 1:numel(pc)
        tok = regexp(pc(e).Type, '^(LConn|RConn)(\d+)$', 'tokens', 'once');
        if isempty(tok), continue, end
        i = find(hc == ph.(tok{1})(str2double(tok{2})), 1);
        o = [reshape(pc(e).SrcPort, [], 1); reshape(pc(e).DstPort, [], 1)];
        [tf, at] = ismember(o, hc);
        for j = reshape(at(tf), 1, [])
            up = local_union(up, i, j);
        end
    end
end
r = arrayfun(@(i) local_find(up, i), 1:numel(c));
groups = unique(r);
nodes = cell(0, 1); nodeH = cell(0, 1);
for g = groups
    in = find(r == g);
    if numel(in) < 2, continue, end
    [s, o] = sort(N(c(in)));
    nodes{end+1, 1} = reshape(s, 1, []); %#ok<AGROW>
    nodeH{end+1, 1} = reshape(hc(in(o)), 1, []); %#ok<AGROW>
end
[~, o] = sort(cellfun(@(s) s{1}, nodes, 'UniformOutput', false));
fp.nodes = reshape(nodes(o), [], 1);
C.nodes = reshape(nodeH(o), [], 1);
end

% -------------------------------------------------------------------------------------------
function d = local_dst(L)
% the destination ports of the line tree whose root segment is L (its own list and its leaves' lists)
d = reshape(get_param(L, 'DstPortHandle'), [], 1);
todo = L;
seen = zeros(0, 1);
while ~isempty(todo)
    s = todo(1);
    todo(1) = [];
    if any(seen == s), continue, end
    seen(end+1, 1) = s; %#ok<AGROW>
    ch = get_param(s, 'LineChildren');
    if isempty(ch)
        d = [d; reshape(get_param(s, 'DstPortHandle'), [], 1)]; %#ok<AGROW>
    else
        todo = [todo; reshape(ch, [], 1)]; %#ok<AGROW>
    end
end
d = unique(d(d ~= -1));
end

function v = local_get(h, p)
% a port parameter as text ('' where the port has no such parameter)
try
    v = get_param(h, p);
catch %#ok<CTCH> no such parameter on this port kind
    v = '';
end
if isnumeric(v) || islogical(v), v = num2str(v); end
end

function v = local_sigobj(h)
% the storage class of a Simulink.Signal embedded in the port, '' without one
v = '';
try
    o = get_param(h, 'SignalObject');
catch %#ok<CTCH> no such parameter on this port kind
    return
end
if isempty(o), return, end
try
    v = sprintf('%s:%s', class(o), o.CoderInfo.StorageClass);
catch %#ok<CTCH> an object without CoderInfo
    v = class(o);
end
end

function [h, sc] = local_mapping(mdl)
% the mapped signals of the model's code mapping and their storage classes (none without a code mapping)
h = zeros(0, 1); sc = cell(0, 1);
try
    cm = coder.mapping.api.get(mdl);
catch %#ok<CTCH> no code mapping (coderdictionary:api:NoCodeMapping) or no Embedded Coder
    return
end
h = reshape(find(cm, 'Signals'), [], 1);
sc = cell(numel(h), 1);
for k = 1:numel(h)
    sc{k} = getSignal(cm, h(k), 'StorageClass');
end
end

function up = local_union(up, i, j)
a = local_find(up, i);
b = local_find(up, j);
if a ~= b, up(max(a, b)) = min(a, b); end
end

function r = local_find(up, i)
r = i;
while up(r) ~= r, r = up(r); end
end
