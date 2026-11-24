function R = per_tb_run(P, ds, T, varargin)
%PER_TB_RUN  Simulate tests/sensors/tb_perception.slx (T13) and unpack the outputs.
%
%   R = per_tb_run(P, ds, T)                         ds from per_tb_input, T = StopTime [s]
%   R = per_tb_run(P, ds, T, 'VC_LIDAR', 2, 'Model', 'tb_perception', 'Map', true)
%   R fields: per (struct of arrays, time along dim 1: t, t_scan, scanValid, ranges_B, angles_B, minSectorRange,
%     protectiveStop, protFieldLen, obsCount, obsPos_B (N x 20 x 2), obsRadius, obsRange, row.*, scanMatch.*,
%     map (only with 'Map' true: Resolution, GridLocationInWorld (N x 2), OccupancyMatrix (300 x 300 x N))),
%     sm (scanmatch port at 0.2 s: t, valid, pose (N x 3), cov (3 x 3 x N), score, nPts), wall (sim wall-clock),
%     exec (execution wall time without compile), out (Simulink.SimulationOutput)
%   P is passed with SimulationInput.setVariable (the base workspace is not modified).

ip = inputParser;
ip.addParameter('VC_LIDAR', 1);
ip.addParameter('VC_TARGET', 0);
ip.addParameter('Model', 'tb_perception');
ip.addParameter('Map', false);
ip.addParameter('Mode', 'normal');
ip.addParameter('RefMode', 'Normal');          % SimulationMode of the Perception Model block
ip.parse(varargin{:});
o = ip.Results;
mdl = o.Model;
if ~bdIsLoaded(mdl), load_system(mdl); end
in = Simulink.SimulationInput(mdl);
if ~strcmp(o.RefMode, 'Normal')
    in = in.setBlockParameter([mdl '/Perception'], 'SimulationMode', o.RefMode);
end
in = in.setExternalInput(ds);
in = in.setVariable('P', P);
in = in.setVariable('VC_LIDAR', o.VC_LIDAR);
in = in.setVariable('VC_TARGET', o.VC_TARGET);
in = in.setModelParameter('StopTime', num2str(T, 17), 'SimulationMode', o.Mode);
t0 = tic;
try
    out = sim(in);
catch e
    % MATLAB Function JIT relink assertion seen after rebuilt charts in a long-lived session (lessons): retry once
    if ~contains(e.identifier, 'Stateflow') && ~contains(e.message, 'o != nullptr'), rethrow(e); end
    t0 = tic;
    out = sim(in);
end
R.wall = toc(t0);
R.out = out;
if ~isempty(out.ErrorMessage), error('per_tb_run:sim', '%s', out.ErrorMessage); end
try
    R.exec = out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
catch
    R.exec = NaN;
end
if ~strcmp(mdl, 'tb_perception'), return, end
y = out.yout;
pv = y.getElement('perception').Values;
R.per = local_unpack(pv, o.Map);
sv = y.getElement('scanmatch').Values;
R.sm = local_unpack(sv, true);
end

function s = local_unpack(v, withMap)
fn = fieldnames(v);
s = struct();
for k = 1:numel(fn)
    f = fn{k};
    if strcmp(f, 'map') && ~withMap, continue, end
    x = v.(f);
    if isstruct(x)
        s.(f) = local_unpack(x, withMap);
    else
        if ~isfield(s, 't'), s.t = x.Time; end
        d = x.Data;
        s.(f) = d;
    end
end
end
