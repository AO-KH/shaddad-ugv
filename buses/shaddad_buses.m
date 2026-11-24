function out = shaddad_buses(varargin)
%SHADDAD_BUSES  Create every Simulink.Bus object (Bus_*) of the Shaddad UGV in the base workspace.
%
%   shaddad_buses()            create / overwrite all Bus_* objects in the base workspace
%   names = shaddad_buses()    ... and return the list of bus names
%   defs  = shaddad_buses('defs')   return the plain definition structs only (NO base-workspace write);
%                                   used by utilities/shaddad_icd_report.m
%   rep   = shaddad_buses('check')  create in base, then verify that every nested type resolves and that
%                                   Simulink.Bus.createMATLABStruct succeeds for every bus (returns report)
%
%   Single source of truth: buses/buses_<domain>.m (one file per owning domain, so parallel builders
%   never edit the same file). Element rows are built with bus_el, buses with bus_def.
%   This function writes ONLY variables named Bus_* to the base workspace.
%
%   Conventions (binding, see docs/architecture.md section 3):
%     * SI units; temperatures K (except fields suffixed _degC); magnetic field uT; angles rad
%       (except fields suffixed _deg); battery current discharge positive.
%     * World ENU at P.world.lla0; body FLU; yaw CCW from East; pitch + = nose down.
%     * Accelerometer data on buses = PHYSICAL specific force.
%     * Enumerations are carried as uint8 with documented codes (enum classes in buses/*.m are used
%       inside models only).

mode = 'create';
if nargin >= 1, mode = lower(char(varargin{1})); end

defs = [buses_sim(), buses_structure(), buses_power(), buses_drive(), ...
        buses_electronics(), buses_sensors(), buses_gnc()];
names = {defs.name};

% ---- static checks on the definitions ------------------------------------------------------
[u, ~, j] = unique(names);
cnt = accumarray(j(:), 1);
if any(cnt > 1)
    error('shaddad_buses:duplicate', 'Duplicate bus names: %s', strjoin(u(cnt > 1), ', '));
end
for k = 1:numel(defs)
    if ~startsWith(defs(k).name, 'Bus_')
        error('shaddad_buses:name', 'Bus name must start with Bus_: %s', defs(k).name);
    end
    en = cellfun(@(e) e.name, defs(k).elems, 'UniformOutput', false);
    [ue, ~, je] = unique(en);
    ce = accumarray(je(:), 1);
    if any(ce > 1)
        error('shaddad_buses:dupElem', '%s has duplicate elements: %s', defs(k).name, strjoin(ue(ce > 1), ', '));
    end
    for m = 1:numel(defs(k).elems)
        dt = defs(k).elems{m}.dtype;
        if startsWith(dt, 'Bus: ')
            ref = strtrim(extractAfter(dt, 'Bus: '));
            if ~any(strcmp(names, ref))
                error('shaddad_buses:unresolved', '%s.%s references undefined %s', ...
                      defs(k).name, defs(k).elems{m}.name, ref);
            end
        end
    end
end

if strcmp(mode, 'defs')
    out = defs;
    return
end

% ---- create Simulink.Bus objects ----------------------------------------------------------
for k = 1:numel(defs)
    dk = defs(k);
    els = Simulink.BusElement.empty(0, 1);
    for m = 1:numel(dk.elems)
        r = dk.elems{m};
        el = Simulink.BusElement;
        el.Name           = r.name;
        el.DataType       = r.dtype;
        el.Dimensions     = r.dims;
        el.DimensionsMode = 'Fixed';
        el.Complexity     = 'real';
        el.Unit           = r.unit;
        el.Description    = r.desc;
        if ~isempty(r.min), el.Min = r.min; end
        if ~isempty(r.max), el.Max = r.max; end
        els(end+1, 1) = el; %#ok<AGROW>
    end
    b = Simulink.Bus;
    b.Elements    = els;
    b.Description = sprintf('%s | Ts=%s s | Producer: %s | Consumers: %s', ...
                            dk.desc, local_ts(dk.ts), dk.producer, dk.consumers);
    assignin('base', dk.name, b);
end

switch mode
    case 'check'
        rep = struct('name', names, 'ok', false, 'msg', '');
        for k = 1:numel(names)
            try
                s = Simulink.Bus.createMATLABStruct(names{k}); %#ok<NASGU>
                rep(k).ok = true;
            catch ME
                rep(k).msg = ME.message;
            end
        end
        out = rep;
    otherwise
        if nargout > 0, out = names; end
end
end

function s = local_ts(ts)
if isinf(ts),      s = 'Inf';
elseif ts == -1,   s = 'inherited';
elseif ts == 0,    s = '0 (continuous)';
else,              s = num2str(ts);
end
end
