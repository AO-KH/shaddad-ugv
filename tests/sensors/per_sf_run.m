function [Y, R] = per_sf_run(P, ranges, angles)
%PER_SF_RUN  Run tests/sensors/tb_per_scanfilter.slx on K scans and the reference implementation (T13).
%
%   [Y, R] = per_sf_run(P, ranges, angles)    ranges K x 360 single, angles 1 x 360 single
%     Y   block outputs (fields rangesF, rangesB, anglesB, xB, yB, keep; K x 360 each)
%     R   sens_scan_filter_ref outputs, same layout
mdl = 'tb_per_scanfilter';
if ~bdIsLoaded(mdl), load_system(mdl); end
K = size(ranges, 1);
t = (0:K-1)'*0.1;
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(single(ranges), t), 'ranges');
ds = ds.addElement(timeseries(repmat(single(angles(:)'), K, 1), t), 'angles');
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(ds);
in = in.setVariable('P', P);
in = in.setModelParameter('StopTime', num2str(t(end), 17));
out = sim(in);
if ~isempty(out.ErrorMessage), error('per_sf_run:sim', '%s', out.ErrorMessage); end
fn = {'rangesF', 'rangesB', 'anglesB', 'xB', 'yB', 'keep'};
for k = 1:numel(fn)
    v = out.yout.getElement(fn{k}).Values;
    d = v.Data;
    if size(d, 1) ~= K, d = squeeze(d)'; end
    Y.(fn{k}) = d;
end
for k = 1:K
    [a, b, c, d, e, f] = sens_scan_filter_ref(ranges(k, :)', angles(:), P);
    R.rangesF(k, :) = a';  R.rangesB(k, :) = b';  R.anglesB(k, :) = c';
    R.xB(k, :) = d';  R.yB(k, :) = e';  R.keep(k, :) = f';
end
end
