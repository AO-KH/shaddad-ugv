function [out, wall] = structure_tb_run(P, Ts, tbs, varargin)
%STRUCTURE_TB_RUN  One simulation of the STRUCTURE bench tests/structure/tb_structure.slx (T03 test helper).
%
%   [out, wall] = structure_tb_run(P, Ts, tbs, 'StopTime', T, 'Drive', 'speed'|'torque', ...
%                                  'Impl', VC_IMPL, 'ImplDrive', VC_IMPL_DRIVE, 'Fid', VC_STRUCT_FID, ...
%                                  'SimMode', 'normal'|'accelerator', 'Profile', true|false)
%   With 'Profile' true the Simulink Profiler is on (out.SimulationMetadata.TimingInfo.ProfilerData).
%   Execution-only wall time: out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime.
%   P, Ts, tbs go to the run through Simulink.SimulationInput.setVariable (P, Ts and the VC_* globally for
%   this run only, tbs into the bench model workspace): the shared base workspace is never modified.
%   The first compile after a rebuild of a file with MATLAB Function blocks can hit a stale JIT engine
%   ("o != nullptr"); the run is then repeated once. wall = elapsed time of sim() [s] (incl. compile).
ip = inputParser;
ip.addParameter('StopTime', 10);
ip.addParameter('Drive', 'speed');
ip.addParameter('Impl', 0);
ip.addParameter('ImplDrive', 1);
ip.addParameter('Fid', 1);
ip.addParameter('SimMode', 'normal');
ip.addParameter('Profile', false);
ip.parse(varargin{:});
o = ip.Results;
mdl = 'tb_structure';
if ~bdIsLoaded(mdl), load_system(mdl); end
in = Simulink.SimulationInput(mdl);
in = in.setVariable('P', P).setVariable('Ts', Ts).setVariable('tbs', tbs, 'Workspace', mdl);
in = in.setVariable('VC_IMPL', o.Impl).setVariable('VC_IMPL_DRIVE', o.ImplDrive).setVariable('VC_STRUCT_FID', o.Fid);
in = in.setBlockParameter([mdl '/Drivers'], 'LabelModeActiveChoice', o.Drive);
in = in.setModelParameter('StopTime', num2str(o.StopTime, 17), 'SimulationMode', o.SimMode);
in = in.setModelParameter('ReturnWorkspaceOutputs', 'on');
if o.Profile
    in = in.setModelParameter('Profile', 'on');
end
t0 = tic;
[out, msg] = local_sim(in);
if ~isempty(msg) && contains(msg, 'nullptr')
    t0 = tic;
    [out, msg] = local_sim(in);
end
wall = toc(t0);
if ~isempty(msg)
    error('structure_tb_run:sim', '%s', msg);
end
end

function [out, msg] = local_sim(in)
% evalc keeps the Stateflow JIT relink assertion trace (printed on the first compile after a variant switch
% or a rebuild, harmless, the run is repeated) out of the command window
msg = '';
out = [];
try
    [~, out] = evalc('sim(in, ''ShowProgress'', ''off'')');
    if ~isempty(out.ErrorMessage), msg = out.ErrorMessage; end
catch tmp_T03_err
    msg = tmp_T03_err.message;
end
end
