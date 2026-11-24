function S = tb_drive_bus_ts(busName, t, vals)
%TB_DRIVE_BUS_TS  Struct of timeseries for a From Workspace bus input of the T06 drive benches.
%   S = tb_drive_bus_ts(busName, t, vals)
%     busName  bus object name in the base workspace (e.g. 'Bus_DriveSideCmd', nested buses allowed)
%     t        time vector (column or row); [] or scalar 0 = constant
%     vals     struct with one field per element (scalar or numel(t) vector; nested struct for sub-buses);
%              missing fields are 0 (false). Values are cast to the element data type.
if nargin < 3, vals = struct(); end
if isempty(t), t = 0; end
t = t(:);
B = evalin('base', busName);
S = struct();
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    if startsWith(e.DataType, 'Bus:')
        sub = strtrim(extractAfter(e.DataType, 'Bus:'));
        if isfield(vals, e.Name), sv = vals.(e.Name); else, sv = struct(); end
        S.(e.Name) = tb_drive_bus_ts(sub, t, sv);
    else
        if isfield(vals, e.Name), v = vals.(e.Name); else, v = 0; end
        v = v(:);
        if isscalar(v), v = repmat(v, numel(t), 1); end
        if strcmp(e.DataType, 'boolean'), v = logical(v); else, v = cast(v, e.DataType); end
        S.(e.Name) = timeseries(v, t);
    end
end
end
