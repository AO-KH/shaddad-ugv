function in = scn_base_input(P, name, varargin)
%SCN_BASE_INPUT  Shared defaults of every scenario: a Simulink.SimulationInput('Shaddad_System').
%
%   in = scn_base_input(P, name)
%   in = scn_base_input(P, name, 'StopTime', T, 'Input', tab, 'VC', vcStruct, 'Ts', Ts)
%
%   Sets (build plan T01.8, D58):
%     * setVariable 'P' (the scenario's parameter struct), 'Ts' (option or base), every 'VC_*' (base values,
%       overridden by the fields of option 'VC', e.g. struct('VC_PHASE',2)) and 'mpcRowObj' when it
%       exists in the base workspace (T14 row-tracker MPC object)
%     * StopTime (option 'StopTime', default 10 s)
%     * external input: scn_make_input(P, tab, StopTime) on the root port ScenarioIn
%       (option 'Input' = struct of [t value] ZOH tables per Bus_ScenarioIn field path; missing = 0)
%     * in.UserString = name
%   The base workspace is read, never written; the SimulationInput carries every value, so it is
%   parsim-safe (workers: 'SetupFcn', @() shaddad_startup('quiet')).

ip = inputParser;
ip.addParameter('StopTime', 10);
ip.addParameter('Input', struct());
ip.addParameter('VC', struct());
ip.addParameter('Ts', []);
ip.parse(varargin{:});
o = ip.Results;
if nargin < 2 || isempty(name), name = 'scenario'; end

in = Simulink.SimulationInput('Shaddad_System');
in.UserString = char(name);
in = in.setVariable('P', P);
if isempty(o.Ts), TsV = evalin('base', 'Ts'); else, TsV = o.Ts; end
in = in.setVariable('Ts', TsV);
vcNames = evalin('base', 'who(''VC_*'')');
for k = 1:numel(vcNames)
    v = evalin('base', vcNames{k});
    if isfield(o.VC, vcNames{k}), v = o.VC.(vcNames{k}); end
    in = in.setVariable(vcNames{k}, v);
end
extra = setdiff(fieldnames(o.VC), vcNames);
for k = 1:numel(extra)
    in = in.setVariable(extra{k}, o.VC.(extra{k}));
end
if evalin('base', 'exist(''mpcRowObj'', ''var'')')
    in = in.setVariable('mpcRowObj', evalin('base', 'mpcRowObj'));
end
in = in.setModelParameter('StopTime', num2str(o.StopTime, 17));
in = in.setExternalInput(scn_make_input(P, o.Input, o.StopTime));
end
