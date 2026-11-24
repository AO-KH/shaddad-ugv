function [R, out] = pp_run(mdl, TB, varargin)
%PP_RUN  Run one T05 bench simulation independent of the shared base workspace (owner: power, T05).
%
%   R = pp_run(mdl, TB, 'StopTime', T, ...)   mdl = 'tb_power_plant' | 'tb_shift_cycle' | 'tb_lv_rails' | 'tb_impl_codec'
%   Options:
%     'StopTime'  simulated time (s)
%     'VC'        struct of variant controls overriding shaddad_params defaults (e.g. struct('VC_PACK',2,'VC_IMPL',1))
%     'Mode'      'normal' (1 ms plant), 'mission' (1 ms rates and the Simscape step -> P.power.tb.Ts_mission,
%                 multi-hour runs), 'fault' (-> P.sim.ssc.Ts_fault = 1e-4 s, short-circuit tests)
%     'PMod'      function handle P = f(P) applied to the parameters (e.g. Q0, dock air)
%     'InitialState' / 'SaveState'   operating-point continuation (chunked long runs)
%     'CompileOnly'  true: update diagram only (sim with StopTime 0), R.ok / R.err
%     'SignalLogging' false: no signal logging (telem / power_truth / ecu), multi-hour runs use the To Workspace logs
%     'Ts_plant'  explicit plant step for the mission mode (overrides P.power.tb.Ts_mission)
%   P, Ts and VC_* come from shaddad_params('Assign', false) and are passed with SimulationInput.setVariable
%   (global scope, also seen by the referenced Shaddad_BMS); TB goes into the bench model workspace. Only the
%   Bus_* objects are taken from the base workspace. The SDI runs created by this call are deleted again.
%   R: log_fast / log_slow / log_lv / cmd_out / fb_out arrays (as present), telem / power_truth (struct of
%   time + fields), ecu (internal impl_ecu signals), wall (s), T_end, xFinal (when SaveState).
ip = inputParser;
ip.addParameter('StopTime', 1);
ip.addParameter('VC', struct());
ip.addParameter('Mode', 'normal');
ip.addParameter('PMod', []);
ip.addParameter('InitialState', []);
ip.addParameter('SaveState', false);
ip.addParameter('CompileOnly', false);
ip.addParameter('SignalLogging', true);
ip.addParameter('Ts_plant', []);
ip.parse(varargin{:});
o = ip.Results;
[P, Ts, VC] = shaddad_params('Assign', false);
if ~isempty(o.PMod), P = o.PMod(P); end
switch lower(o.Mode)
    case 'mission', tm = P.power.tb.Ts_mission;
    case 'fault',   tm = P.sim.ssc.Ts_fault;
    otherwise,      tm = [];
end
if ~isempty(o.Ts_plant), tm = o.Ts_plant; end
if ~isempty(tm)
    for fn = {'base', 'ssc', 'esc', 'safety', 'enc'}
        Ts.(fn{1}) = tm;
    end
    P.sim.ssc.Ts = tm;
    if tm > 1e-3
        % On-Off Delay blocks need non-zero delays >= their sample time: relay pickup / dropout times below the
        % mission step are rounded up to one step
        for k = {'K1', 'K2', 'K3', 'K4'}
            for fd = {'t_on', 't_off'}
                v = P.power.pdu.(k{1}).(fd{1});
                if v > 0 && v < tm, P.power.pdu.(k{1}).(fd{1}) = tm; end
            end
        end
    end
end
vn = fieldnames(o.VC);
for k = 1:numel(vn), VC.(vn{k}) = o.VC.(vn{k}); end
if ~evalin('base', 'exist(''Bus_PowerStatus'', ''var'')')
    shaddad_buses();
end
load_system(mdl);
if strcmp(get_param(mdl, 'FastRestart'), 'on'), set_param(mdl, 'FastRestart', 'off'); end
in = Simulink.SimulationInput(mdl);
if ~isempty(TB), in = in.setVariable('TB', TB, 'Workspace', mdl); end
in = in.setVariable('P', P);
in = in.setVariable('Ts', Ts);
vn = fieldnames(VC);
for k = 1:numel(vn), in = in.setVariable(vn{k}, VC.(vn{k})); end
if o.CompileOnly
    in = in.setModelParameter('StopTime', '0');
else
    in = in.setModelParameter('StopTime', sprintf('%.15g', o.StopTime));
end
if ~isempty(tm) && tm < 1e-3
    % fault mode: base step below the 1 ms of cs_Shaddad (the referenced Shaddad_BMS keeps its 1 ms base, so the
    % mission mode keeps FixedStep = 1 ms)
    in = in.setModelParameter('FixedStep', sprintf('%.15g', tm));
end
if o.SaveState
    in = in.setModelParameter('SaveFinalState', 'on', 'SaveOperatingPoint', 'on', 'FinalStateName', 'xFinal');
end
if ~isempty(o.InitialState)
    in = in.setInitialState(o.InitialState);
end
in = in.setModelParameter('SimscapeLogType', 'none');
if ~o.SignalLogging, in = in.setModelParameter('SignalLogging', 'off'); end
ids0 = Simulink.sdi.getAllRunIDs;
t0 = tic;
R = struct('ok', true, 'err', '');
if o.CompileOnly
    try
        out = sim(in);
        if ~isempty(out.ErrorMessage), R.ok = false; R.err = local_diag(out); end
    catch e
        R.ok = false; R.err = getReport(e, 'extended', 'hyperlinks', 'off'); out = [];
    end
    R.wall = toc(t0);
    local_sdi_clean(ids0, mdl);
    return
end
out = sim(in);
R.wall = toc(t0);
if ~isempty(out.ErrorMessage)
    R.ok = false; R.err = local_diag(out);
    local_sdi_clean(ids0, mdl);
    error('pp_run:sim', '%s: %s', mdl, R.err);
end
R.T_end = out.tout(end);
for v = {'log_fast', 'log_slow', 'log_bms', 'log_truth', 'log_lv', 'cmd_out', 'fb_out'}
    if any(strcmp(out.who, v{1})), R.(v{1}) = out.(v{1}); end
end
if any(strcmp(out.who, 'logsout')) && ~isempty(out.logsout)
    ls = out.logsout;
    for v = {'telem', 'power_truth'}
        e = ls.find('Name', v{1});
        if e.numElements > 0, R.(v{1}) = local_bus(e.getElement(1).Values); end
    end
    R.ecu = struct();
    for v = {'ecu_w_target', 'ecu_enable'}
        e = ls.find('Name', v{1});
        if e.numElements > 0
            ts = e.getElement(1).Values;
            R.ecu.(v{1}) = [ts.Time double(ts.Data(:))];
        end
    end
end
if o.SaveState, R.xFinal = out.xFinal; end
local_sdi_clean(ids0, mdl);
end

function S = local_bus(B)
% Simulink bus timeseries struct -> struct(t, fields as N x width double arrays)
fn = fieldnames(B);
S = struct('t', B.(fn{1}).Time);
for k = 1:numel(fn)
    d = B.(fn{k}).Data;
    n = numel(S.t);
    if size(d, ndims(d)) == n && ndims(d) > 2 || (ismatrix(d) && size(d, 1) ~= n && size(d, 2) == n)
        d = reshape(d, [], n).';
    else
        d = reshape(d, n, []);
    end
    S.(fn{k}) = double(d);
end
end

function s = local_diag(out)
% full text of the simulation error diagnostic (all causes)
s = out.ErrorMessage;
try
    d = out.SimulationMetadata.ExecutionInfo.ErrorDiagnostic.Diagnostic;
    s = local_flat(d, 0);
catch
end
end

function s = local_flat(d, lev)
s = sprintf('%s%s\n', repmat('  ', 1, lev), d.message);
for k = 1:numel(d.cause)
    s = [s local_flat(d.cause{k}, lev + 1)]; %#ok<AGROW>
end
end

function local_sdi_clean(ids0, mdl)
ids = setdiff(Simulink.sdi.getAllRunIDs, ids0);
for k = 1:numel(ids)
    try
        if strcmp(Simulink.sdi.getRun(ids(k)).Model, mdl), Simulink.sdi.deleteRun(ids(k)); end
    catch
    end
end
end
