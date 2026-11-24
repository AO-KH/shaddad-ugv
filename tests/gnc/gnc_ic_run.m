function r = gnc_ic_run(P, prof)
%GNC_IC_RUN  Run the Subsystem Reference gnc_InputConditioning alone and return its Bus_NavCond flags.
%
%   r = gnc_ic_run(P, prof)
%     P     parameter struct (tc.P1; passed with Simulink.SimulationInput.setVariable, base P is not touched)
%     prof  struct of column vectors on the 0.01 s grid prof.t (t(1) = 0, step 0.01):
%           sE, sN    posStd(1:2) of Bus_NavState [m] (E, N; posStd(3) = 0.03)
%           cEN       covPose(1,2) = covPose(2,1) [m^2]          (default 0; covPose(1,1) = sE^2, (2,2) = sN^2)
%           psi       Bus_PathError.psi_ref [rad] of the lane      (default 0)
%           valid     Bus_PathError.valid                          (default true)
%           quality   NavQuality                                   (default 5, LIDAR_LOCALIZED)
%     r     t (0.05 s column), nav_degraded, nav_ok (logical), sigma_pos [m]
%
%   The harness is built in memory and closed unsaved (no project file changes): Constants with the P.ic values
%   of Bus_NavState, Bus_McuStatus (valid false: no odometry term), Bus_BmsStatus, Bus_MissionCmd, Bus_SprayDemand
%   and Bus_PathError at the port sample times; Bus Assignments of the profiled elements from root Inports (external
%   inputs); the pe input only when the port table of build_shaddad_gnc('ports') has it. Needs Ts and Bus_* in base.
%
%   See also t_gnc_mission, build_shaddad_gnc.

T = build_shaddad_gnc('ports');
R = T.gnc_InputConditioning;
hasPe = any(strcmp({R.in.name}, 'pe'));
t = prof.t(:);
n = numel(t);
def = struct('cEN', zeros(n, 1), 'psi', zeros(n, 1), 'valid', true(n, 1), 'quality', 5*ones(n, 1));
for f = fieldnames(def)'
    if ~isfield(prof, f{1}), prof.(f{1}) = def.(f{1}); end
end
nm = 'tmp_gnc_ic_harness';
if bdIsLoaded(nm), close_system(nm, 0); end
icLoaded = bdIsLoaded('gnc_InputConditioning');
new_system(nm);
closeH = onCleanup(@() local_close(nm, icLoaded));
set_param(nm, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', ...
    'StopTime', sprintf('%.6g', t(end)), 'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
A = @(lib, b, varargin) add_block(lib, [nm '/' b], varargin{:});
Ln = @(s, d) add_line(nm, s, d, 'autorouting', 'on');
% ---- profiled inputs (root Inports, external inputs) ----
A('simulink/Sources/In1', 'std', 'Port', '1', 'OutDataTypeStr', 'double', 'PortDimensions', '3', 'SampleTime', 'Ts.nav');
A('simulink/Sources/In1', 'cov', 'Port', '2', 'OutDataTypeStr', 'double', 'PortDimensions', '[3 3]', 'SampleTime', 'Ts.nav');
A('simulink/Sources/In1', 'q', 'Port', '3', 'OutDataTypeStr', 'uint8', 'PortDimensions', '1', 'SampleTime', 'Ts.nav');
A('simulink/Sources/In1', 'psi', 'Port', '4', 'OutDataTypeStr', 'double', 'PortDimensions', '1', 'SampleTime', 'Ts.gnc');
A('simulink/Sources/In1', 'pv', 'Port', '5', 'OutDataTypeStr', 'boolean', 'PortDimensions', '1', 'SampleTime', 'Ts.gnc');
% ---- bus sources at the port sample times ----
src = {'NavIC', 'Bus_NavState', 'Ts.nav'; 'McuIC', 'Bus_McuStatus', 'Ts.gnc'; 'BmsIC', 'Bus_BmsStatus', 'Ts.hk'; ...
       'McmdIC', 'Bus_MissionCmd', 'Ts.mcmd'; 'SprayIC', 'Bus_SprayDemand', 'Ts.spray'; 'PeIC', 'Bus_PathError', 'Ts.gnc'};
for k = 1:size(src, 1)
    A('simulink/Sources/Constant', src{k, 1}, 'Value', ['P.ic.' src{k, 2}], 'OutDataTypeStr', ['Bus: ' src{k, 2}], ...
        'SampleTime', src{k, 3});
end
A('simulink/Sources/Constant', 'valid_on', 'Value', 'true', 'OutDataTypeStr', 'boolean', 'SampleTime', 'Ts.nav');
A('simulink/Signal Routing/Bus Assignment', 'NavState', 'AssignedSignals', 'posStd,covPose,quality,navValid');
Ln('NavIC/1', 'NavState/1');  Ln('std/1', 'NavState/2');  Ln('cov/1', 'NavState/3');  Ln('q/1', 'NavState/4');
Ln('valid_on/1', 'NavState/5');
A('simulink/Signal Routing/Bus Assignment', 'Pe', 'AssignedSignals', 'psi_ref,valid');
Ln('PeIC/1', 'Pe/1');  Ln('psi/1', 'Pe/2');  Ln('pv/1', 'Pe/3');
% ---- device under test ----
A('simulink/Ports & Subsystems/Subsystem Reference', 'IC', 'ReferencedSubsystem', 'gnc_InputConditioning');
Ln('NavState/1', 'IC/1');  Ln('McuIC/1', 'IC/2');  Ln('BmsIC/1', 'IC/3');  Ln('McmdIC/1', 'IC/4');  Ln('SprayIC/1', 'IC/5');
if hasPe
    Ln('Pe/1', ['IC/' num2str(find(strcmp({R.in.name}, 'pe')))]);
else
    A('simulink/Sinks/Terminator', 'T_pe');  Ln('Pe/1', 'T_pe/1');
end
A('simulink/Signal Routing/Bus Selector', 'NavSel', 'OutputSignals', 'nav_degraded,nav_ok,sigma_pos');
Ln('IC/1', 'NavSel/1');
outs = {'nav_degraded', 'nav_ok', 'sigma_pos'};
for k = 1:3
    A('simulink/Sinks/Out1', outs{k}, 'Port', num2str(k));
    Ln(sprintf('NavSel/%d', k), [outs{k} '/1']);
end
for k = 2:numel(R.out)
    A('simulink/Sinks/Terminator', sprintf('T_out%d', k));
    Ln(sprintf('IC/%d', k), sprintf('T_out%d/1', k));
end
% ---- external inputs, in port order ----
cov = zeros(3, 3, n);
cov(1, 1, :) = prof.sE.^2;  cov(2, 2, :) = prof.sN.^2;  cov(3, 3, :) = 0.03^2;
cov(1, 2, :) = prof.cEN;  cov(2, 1, :) = prof.cEN;
ds = Simulink.SimulationData.Dataset();
ds = ds.addElement(timeseries([prof.sE(:) prof.sN(:) 0.03*ones(n, 1)], t), 'std');
ds = ds.addElement(timeseries(cov, t), 'cov');
ds = ds.addElement(timeseries(uint8(prof.quality(:)), t), 'q');
ds = ds.addElement(timeseries(double(prof.psi(:)), t), 'psi');
ds = ds.addElement(timeseries(logical(prof.valid(:)), t), 'pv');
in = Simulink.SimulationInput(nm);
in = in.setVariable('P', P);
in = in.setExternalInput(ds);
out = sim(in);
y = out.yout;                                  % elements in Outport order: nav_degraded, nav_ok, sigma_pos
g = @(k) y.getElement(k).Values;
v = g(1);
r = struct('t', v.Time(:), 'nav_degraded', logical(v.Data(:)), 'nav_ok', logical(g(2).Data(:)), ...
    'sigma_pos', double(g(3).Data(:)), 'has_pe', hasPe);
end

function local_close(nm, icLoaded)
if bdIsLoaded(nm), close_system(nm, 0); end
if ~icLoaded && bdIsLoaded('gnc_InputConditioning'), close_system('gnc_InputConditioning', 0); end
end
