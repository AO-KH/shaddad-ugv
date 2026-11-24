function ds = mcu_cl_input(T, tab)
%MCU_CL_INPUT  External input Dataset of the T09 bench tests/electronics/tb_mcu_closed_loop.slx.
%
%   ds = mcu_cl_input(T, tab)
%   T     stop time (s)
%   tab   struct with optional fields op, flt, hmi, gnc, bms (structs, one field per bus element) and plant:
%         a field value is a constant, an M x (1 + width) ZOH table [t value...] (value from t on), or a full
%         column of samples on the port's time grid
%   Ports and grids: op Bus_OperatorCmd, flt Bus_FaultInject, hmi Bus_HmiInputs, plant double[9]
%   [yaw_gain pitch roll ground_ratio a_run brake_fail impl m f_rr] on 0:0.01:T (a 4- or 6-wide plant value or table,
%   as before, gets the later elements 0: no runaway, no brake fault, no implement, the first-order plant; brake_fail
%   b0 L / b1 R a brake that never engages, b2 the physical plant's ESCs without a hold-torque preload, a roll-back at
%   each brake release on a slope, see mcu_cl_track_dyn); gnc
%   Bus_GncCmd on 0:0.05:T; bms Bus_BMS_Telemetry on 0:0.1:T (I_chg_lim_A is the CCL, I_dis_lim_A the DCL).
%   Defaults: TX on, speed knob 100 %, switches MANUAL / disarmed / implement OFF / lift HOLD / mission none;
%   no faults; HMI idle; companion alive (seq +1 per frame, gnc_ready, no motion command; tab.gnc_seq_stop = t
%   freezes seq from t on); BMS HV ready (V 51.2 V, DCL 300 A, CCL 100 A, alive counting); plant [1 0 0 1 0 0 0 0 0].

if nargin < 2 || isempty(tab), tab = struct(); end
P = evalin('base', 'P');
t10 = (0:round(T/0.01))'*0.01; t50 = (0:round(T/0.05))'*0.05; t100 = (0:round(T/0.1))'*0.1;
def = struct();
def.op = struct('tx_power_on', 1, 'knob_speed', 1, 'sw_mode', 1, 'sw_impl', 1, 'sw_lift', 2, 'sw_mission', 1);
def.flt = struct();
def.hmi = struct();
g = P.ic.Bus_GncCmd; g.gnc_ready = true; g.mode_req = uint8(GncModeReq.AUTO);
def.gnc = g;
b = P.ic.Bus_BMS_Telemetry;
b.HV_Ready = true; b.K1_Closed = true; b.Contactor_State = uint8(BmsContactorState.HV_ON); b.SOC_pack = 0.8;
b.SOH = 1; b.V_pack_V = 51.2; b.V_bus_V = 51.2; b.I_dis_lim_A = 300; b.I_chg_lim_A = 100; b.T_cell_min_K = 298;
b.T_cell_max_K = 299; b.V_cell_min_V = 3.2; b.V_cell_max_V = 3.21; b.V_chg_lim_V = 58.4; b.V_dis_lim_V = 44.8;
b.P_dis_lim_W = 15000; b.P_chg_lim_W = 5000; b.E_remaining_Wh = 2000; b.Runtime_Remaining_s = 7200;
def.bms = b;
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(local_bus('Bus_OperatorCmd', t10, local_get(tab, 'op'), def.op), 'op');
ds = ds.addElement(local_bus('Bus_FaultInject', t10, local_get(tab, 'flt'), def.flt), 'flt');
ds = ds.addElement(local_bus('Bus_HmiInputs', t10, local_get(tab, 'hmi'), def.hmi), 'hmi');
gt = local_get(tab, 'gnc');
if ~isfield(gt, 'seq')
    seq = mod(1:numel(t50), 256)';
    if isfield(tab, 'gnc_seq_stop')
        k0 = find(t50 >= tab.gnc_seq_stop - 1e-9, 1);
        if ~isempty(k0), seq(k0:end) = seq(max(k0 - 1, 1)); end
    end
    gt.seq = seq;
end
ds = ds.addElement(local_bus('Bus_GncCmd', t50, gt, def.gnc), 'gnc');
bt = local_get(tab, 'bms');
if ~isfield(bt, 'AliveCounter'), bt.AliveCounter = mod(0:numel(t100) - 1, 256)'; end
ds = ds.addElement(local_bus('Bus_BMS_Telemetry', t100, bt, def.bms), 'bms');
pl = repmat([1 0 0 1 0 0 0 0 0], numel(t10), 1);
if isfield(tab, 'plant')
    x = double(tab.plant);
    w = size(x, 2);
    if w == 4 || w == 6                                 % 4 / 6 values (a constant row or columns of samples)
        x = [x zeros(size(x, 1), 9 - w)];               % the later elements 0, as before
    elseif w == 5 || w == 7                             % a ZOH table [t values] of 4 / 6 values
        x = [x zeros(size(x, 1), 10 - w)];
    end
    pl = local_series(x, pl, t10);
end
ds = ds.addElement(timeseries(pl, t10), 'plant');
end

function s = local_get(tab, f)
s = struct();
if isfield(tab, f), s = tab.(f); end
end

function s = local_bus(busName, t, tab, def)
B = evalin('base', busName);
known = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
bad = setdiff(fieldnames(tab), known);
assert(isempty(bad), 'mcu_cl_input:field', 'unknown %s element(s): %s', busName, strjoin(bad, ', '));
s = struct();
for k = 1:numel(B.Elements)
    e = B.Elements(k);
    w = prod(e.Dimensions);
    v = zeros(numel(t), w);
    if isfield(def, e.Name), v = repmat(double(def.(e.Name)(:)'), numel(t), 1); end
    if isfield(tab, e.Name), v = local_series(tab.(e.Name), v, t); end
    if strcmp(e.DataType, 'boolean'), v = v ~= 0; else, v = cast(v, e.DataType); end
    s.(e.Name) = timeseries(v, t);
end
end

function v = local_series(x, v, t)
% x: constant row (width of v), ZOH table [t value...], or a full column / matrix of samples
x = double(x);
w = size(v, 2);
if size(x, 1) == numel(t) && size(x, 2) == w && ~(numel(t) == 1)
    v = x;
elseif numel(x) == w
    v = repmat(x(:)', numel(t), 1);
elseif size(x, 2) == w + 1
    for r = 1:size(x, 1)
        k = t >= x(r, 1) - 1e-9;
        v(k, :) = repmat(x(r, 2:end), sum(k), 1);
    end
else
    error('mcu_cl_input:value', 'value of size %s for a %d-wide input on %d samples', mat2str(size(x)), w, numel(t));
end
end
