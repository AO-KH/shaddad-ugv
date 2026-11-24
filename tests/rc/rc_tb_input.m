function ds = rc_tb_input(T, opTab, fltTab)
%RC_TB_INPUT  External input Dataset (op, flt) for the T07 bench tests/rc/tb_rc_link.slx.
%
%   ds = rc_tb_input(T, opTab, fltTab)
%   T       stop time (s); samples on the 0.01 s grid 0:0.01:T (rate of Bus_OperatorCmd / Bus_FaultInject)
%   opTab   struct, one field per Bus_OperatorCmd element (missing: defaults below)
%   fltTab  struct, one field per Bus_FaultInject element (missing: 0 / false)
%           a field value is either an M x 2 ZOH table [t value] (value from t on), or a full
%           column of numel(0:0.01:T) samples (random stimuli)
%   Operator defaults: sticks 0, switches 0 (position 1 / off), tx_power_on = 1.
%   Element data types follow the bus objects in the base workspace (Bus_* from shaddad_params).

if nargin < 2 || isempty(opTab), opTab = struct(); end
if nargin < 3 || isempty(fltTab), fltTab = struct(); end
n = round(T/0.01);
t = (0:n)'*0.01;
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(local_bus('Bus_OperatorCmd', t, opTab, struct('tx_power_on', 1)), 'op');
ds = ds.addElement(local_bus('Bus_FaultInject', t, fltTab, struct()), 'flt');
end

function s = local_bus(busName, t, tab, def)
B = evalin('base', busName);
s = struct();
fn = fieldnames(tab);
known = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
bad = setdiff(fn, known);
assert(isempty(bad), 'rc_tb_input:field', 'unknown %s element(s): %s', busName, strjoin(bad, ', '));
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    v = zeros(numel(t), 1);
    if isfield(def, e.Name), v(:) = def.(e.Name); end
    if isfield(tab, e.Name)
        x = tab.(e.Name);
        if size(x, 2) == 2 && size(x, 1) ~= numel(t)
            for r = 1:size(x, 1)
                v(t >= x(r, 1) - 1e-9) = x(r, 2);
            end
        else
            assert(numel(x) == numel(t), 'rc_tb_input:len', '%s.%s: %d samples, expected %d', ...
                busName, e.Name, numel(x), numel(t));
            v = double(x(:));
        end
    end
    if strcmp(e.DataType, 'boolean')
        v = v ~= 0;
    else
        v = cast(v, e.DataType);
    end
    s.(e.Name) = timeseries(v, t);
end
end
