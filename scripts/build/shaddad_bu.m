function varargout = shaddad_bu(op, varargin)
%SHADDAD_BU  Programmatic-build helpers shared by the integration build scripts (owner: integration).
%
%   shaddad_bu('fresh', name, file, kind)     close model 'name' if loaded, delete 'file', new_system
%                                             (kind 'ssr' -> new_system(name,'Subsystem'), 'model')
%   p = shaddad_bu('add', lib, path, nv...)   add_block(lib, path, nv...) and return the full path
%   h = shaddad_bu('line', sys, src, dst, name)
%                                             add_line(sys, 'Blk/k', 'Blk/j') (branches allowed) and name it
%   h = shaddad_bu('nline', sys, src, dst)    a signal line like 'line', named (model layout spec 6.1, L5): the
%   h = shaddad_bu('nline', sys, src, dst, name)  name the source port's line already has, else the source
%                                             block's name (without the '_m' of a meter's PS-Simulink converter,
%                                             with '_k' for output k of a block with several outputs); with
%                                             name, that name. A line out of a Bus Selector keeps R2026b's
%                                             '<element>'
%   p = shaddad_bu('pick', sys, port, el, dst)
%   p = shaddad_bu('pick', sys, port, el, dst, name)
%                                             pick the element el of the In Bus Element port 'port' of diagram
%                                             sys for the input dst ('Blk/k'), right in front of it (layout spec
%                                             6.2, L3): the block <port>_<el> (a further pick of the same element
%                                             <port>_<el>_2, ...), the port's first block from the library, every
%                                             other one copied from it (spec 13); its line named name (default
%                                             el) by 'nline'; p the block's name
%   shaddad_bu('outbus', sys, port, el, src)
%   shaddad_bu('outbus', sys, port, el, src, name)
%                                             the element el of the Out Bus Element port 'port' of sys, fed by src
%                                             ('Blk/k'): the block <port>_<el>, the port's first block from the
%                                             library, every other one copied from it (spec 13); its line named
%                                             name (default el) by 'nline'
%   shaddad_bu('vbus', sys, n, el)            a virtual Bus Creator n (layout spec 6.3), el rows {elementName,
%                                             'Blk/k'}: input k fed by el{k,2}, its line named el{k,1} by 'nline'
%                                             (a virtual bus takes its element names from its input lines); a
%                                             source whose line already carries another name is the error
%                                             shaddad_bu:vbus (one line, one name)
%   shaddad_bu('portorder', sys, ports, kind) number the ports of sys of kind 'Inport' or 'Outport' in the order
%                                             of the cellstr ports: the classic port block of that name, else the
%                                             bus element port of that name (all its blocks take the number); a
%                                             name that is neither is the error shaddad_bu:portorder
%   shaddad_bu('pline', sys, hA, hB)          physical line between two port handles
%   h = shaddad_bu('pport', blk, portName)    handle of the physical port of a subsystem / SSR block
%                                             that belongs to its Connection Port named portName
%   s = shaddad_bu('tsexpr', ts)              Ts.<field> expression of a port rate (0 -> 'Ts.base')
%   shaddad_bu('save', name, file)            shaddad_layout(name, 'Recursive', true) (every level; an
%                                             engine error stops the save) + save_system to file; with
%                                             the environment variable SHADDAD_LAYOUT exactly 'arrange':
%                                             arrangeSystem (top level, errors ignored) + save_system
%   shaddad_bu('save', name, file, opts)      the same with layout options, a struct: .Keep (cellstr) the
%                                             diagrams whose block positions the builder set, passed to
%                                             shaddad_layout as 'Keep' (lines still re-added and asserted);
%                                             ignored with SHADDAD_LAYOUT 'arrange' (Task 5 decision 9)
%   shaddad_bu('finalize', name, file)        update diagram (needs P, Ts, VC_*, Bus_* in base) and save
%                                             again when the first compile marked the model dirty (Simscape
%                                             / Subsystem Reference caches), so a freshly built model stays
%                                             clean when it is compiled later; never fails the build
%   shaddad_bu('closedirty', names)           close the listed (integration-owned) models without saving
%                                             when they are loaded and dirty (a dirty referenced model
%                                             blocks save_system of its parent)
%
%   All helpers are pure Simulink API calls on models the caller created (build plan rule 3).

switch lower(op)
    case 'fresh'
        [name, file, kind] = varargin{1:3};
        if bdIsLoaded(name), close_system(name, 0); end
        if isfile(file), delete(file); end
        if strcmpi(kind, 'ssr'), new_system(name, 'Subsystem'); else, new_system(name); end
    case 'add'
        [lib, p] = varargin{1:2};
        h = add_block(lib, p, varargin{3:end});
        varargout{1} = getfullname(h);
    case 'line'
        [sys, src, dst] = varargin{1:3};
        h = add_line(sys, src, dst, 'autorouting', 'smart');
        if numel(varargin) >= 4 && ~isempty(varargin{4})
            srcBlk = get_param(get_param(h, 'SrcBlockHandle'), 'BlockType');
            if ~strcmp(srcBlk, 'BusSelector')   % Bus Selector outputs carry the element name
                set_param(h, 'Name', varargin{4});
            end
        end
        varargout{1} = h;
    case 'nline'
        [sys, src, dst] = varargin{1:3};
        nm = '';
        if numel(varargin) >= 4, nm = varargin{4}; end
        varargout{1} = local_nline(sys, src, dst, nm);
    case 'pick'
        [sys, port, el, dst] = varargin{1:4};
        nm = el;
        if numel(varargin) >= 5 && ~isempty(varargin{5}), nm = varargin{5}; end
        varargout{1} = local_pick(sys, port, el, dst, nm);
    case 'outbus'
        [sys, port, el, src] = varargin{1:4};
        nm = el;
        if numel(varargin) >= 5 && ~isempty(varargin{5}), nm = varargin{5}; end
        local_outbus(sys, port, el, src, nm);
    case 'vbus'
        [sys, n, el] = varargin{1:3};
        local_vbus(sys, n, el);
    case 'portorder'
        [sys, ports, kind] = varargin{1:3};
        local_portorder(sys, ports, kind);
    case 'pline'
        [sys, hA, hB] = varargin{1:3};
        varargout{1} = add_line(sys, hA, hB, 'autorouting', 'smart');
    case 'pport'
        [blk, pname] = varargin{1:2};
        varargout{1} = local_pport(blk, pname);
    case 'tsexpr'
        varargout{1} = local_tsexpr(varargin{1});
    case 'save'
        [name, file] = varargin{1:2};
        lay = {};
        if numel(varargin) >= 3 && isfield(varargin{3}, 'Keep'), lay = {'Keep', varargin{3}.Keep}; end
        if strcmp(getenv('SHADDAD_LAYOUT'), 'arrange')   % the pre-engine save (layout spec 7, ledger Ruling C)
            try
                Simulink.BlockDiagram.arrangeSystem(name);
            catch %#ok<CTCH> layout only
            end
        else
            shaddad_layout(name, 'Recursive', true, lay{:});   % no try: a fingerprint or physical error stops it
        end
        save_system(name, file);
    case 'finalize'
        [name, file] = varargin{1:2};
        try
            evalc('set_param(name, ''SimulationCommand'', ''update'')');
        catch tmp_bu_err
            warning('shaddad_bu:finalize', '%s: update after build failed (model saved): %s', name, tmp_bu_err.message);
            return
        end
        if strcmp(get_param(name, 'Dirty'), 'on')
            try
                save_system(name, file);
            catch tmp_bu_err
                warning('shaddad_bu:finalize', '%s: re-save after update skipped: %s', name, tmp_bu_err.message);
            end
        end
    case 'closedirty'
        names = varargin{1};
        for k = 1:numel(names)
            if bdIsLoaded(names{k}) && strcmp(get_param(names{k}, 'Dirty'), 'on')
                close_system(names{k}, 0);
            end
        end
    otherwise
        error('shaddad_bu:op', 'Unknown op %s', op);
end
end

% -------------------------------------------------------------------------------------------
function h = local_pport(blk, pname)
ports = find_system(blk, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'BlockType', 'PMIOPort');
ports = setdiff(ports, {blk}, 'stable');
num = zeros(numel(ports), 1); side = cell(numel(ports), 1); nm = cell(numel(ports), 1);
for k = 1:numel(ports)
    num(k) = str2double(get_param(ports{k}, 'Port'));
    side{k} = get_param(ports{k}, 'Side');
    nm{k} = get_param(ports{k}, 'Name');
end
j = find(strcmp(nm, pname), 1);
if isempty(j)
    error('shaddad_bu:pport', '%s has no Connection Port named %s', blk, pname);
end
same = find(strcmp(side, side{j}));
[~, ord] = sort(num(same));
idx = find(same(ord) == j);
ph = get_param(blk, 'PortHandles');
if strcmpi(side{j}, 'Left'), h = ph.LConn(idx); else, h = ph.RConn(idx); end
end

function h = local_nline(sys, src, dst, nm)
% the line src -> dst ('Blk/k') with smart autorouting, named nm, or (nm empty) the name its source port's line
% already has, else the source block's name without a trailing '_m', with '_k' for output k > 1 of a block with
% several outputs; a Bus Selector's output keeps the '<element>' name R2026b gives it
blk = [sys '/' extractBefore(src, '/')];
k = str2double(extractAfter(src, '/'));
ph = get_param(blk, 'PortHandles');
if isempty(nm)
    old = get_param(ph.Outport(k), 'Line');
    if old ~= -1, nm = get_param(old, 'Name'); end
    if isempty(nm)
        nm = regexprep(extractBefore(src, '/'), '_m$', '');
        if numel(ph.Outport) > 1, nm = sprintf('%s_%d', nm, k); end
    end
end
h = add_line(sys, src, dst, 'autorouting', 'smart');
if ~strcmp(get_param(blk, 'BlockType'), 'BusSelector')
    set_param(get_param(ph.Outport(k), 'Line'), 'Name', nm);
end
end

function p = local_pick(sys, port, el, dst, nm)
% the pick block of element el of the In Bus Element port 'port' of sys for dst (see the help)
base = [port '_' el];
p = base;
k = 1;
while getSimulinkBlockHandle([sys '/' p]) ~= -1
    k = k + 1;
    p = sprintf('%s_%d', base, k);
end
first = local_port_block(sys, port, 'Inport');
if isempty(first)
    h = add_block('simulink/Ports & Subsystems/In Bus Element', [sys '/' p]);
    set_param(h, 'PortName', port);
else
    h = add_block(first, [sys '/' p]);
end
set_param(h, 'Element', el);
local_nline(sys, [p '/1'], dst, nm);
end

function local_outbus(sys, port, el, src, nm)
% the block of element el of the Out Bus Element port 'port' of sys, fed by src (see the help)
p = [port '_' el];
first = local_port_block(sys, port, 'Outport');
if isempty(first)
    h = add_block('simulink/Ports & Subsystems/Out Bus Element', [sys '/' p]);
    set_param(h, 'PortName', port);
else
    h = add_block(first, [sys '/' p]);
end
set_param(h, 'Element', el);
local_nline(sys, src, [p '/1'], nm);
end

function local_vbus(sys, n, el)
% a virtual Bus Creator n whose input k is el{k,2}, its line named el{k,1} (see the help)
add_block('simulink/Signal Routing/Bus Creator', [sys '/' n], 'Inputs', num2str(size(el, 1)));
for k = 1:size(el, 1)
    src = el{k, 2};
    ph = get_param([sys '/' extractBefore(src, '/')], 'PortHandles');
    old = get_param(ph.Outport(str2double(extractAfter(src, '/'))), 'Line');
    if old ~= -1 && ~isempty(get_param(old, 'Name')) && ~strcmp(get_param(old, 'Name'), el{k, 1})
        error('shaddad_bu:vbus', '%s/%s: input %d (%s) is the line %s, already named %s', sys, n, k, src, ...
            el{k, 1}, get_param(old, 'Name'));
    end
    local_nline(sys, src, sprintf('%s/%d', n, k), el{k, 1});
end
end

function local_portorder(sys, ports, kind)
% number the ports of kind 'Inport' | 'Outport' of sys in the order of ports (see the help)
for k = 1:numel(ports)
    b = [sys '/' ports{k}];
    if getSimulinkBlockHandle(b) == -1 || ~strcmp(get_param(b, 'BlockType'), kind)
        b = local_port_block(sys, ports{k}, kind);
    end
    if isempty(b)
        error('shaddad_bu:portorder', '%s has no %s port named %s', sys, kind, ports{k});
    end
    set_param(b, 'Port', num2str(k));
end
end

function b = local_port_block(sys, port, kind)
% a block of the bus element port 'port' (BlockType kind, 'Inport' or 'Outport') directly in sys, '' when none
b = '';
c = find_system(sys, 'SearchDepth', 1, 'BlockType', kind);
for k = 1:numel(c)
    if ~strcmp(c{k}, sys) && strcmp(get_param(c{k}, 'IsBusElementPort'), 'on') && ...
            strcmp(get_param(c{k}, 'PortName'), port)
        b = c{k};
        return
    end
end
end

function s = local_tsexpr(ts)
map = {1e-3, 'Ts.base'; 5e-3, 'Ts.imu'; 1e-2, 'Ts.ctrl'; 5e-2, 'Ts.comm'; 0.1, 'Ts.hk'; ...
       0.2, 'Ts.gnc_status'; 1.0, 'Ts.plan'};
if ts == 0, s = 'Ts.base'; return, end
k = find(abs([map{:, 1}] - ts) < 1e-12, 1);
if isempty(k), error('shaddad_bu:ts', 'Rate %g s is not in the allowed set', ts); end
s = map{k, 2};
end
