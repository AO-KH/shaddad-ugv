function r = rc_tb_run(T, opTab, fltTab, P, varargin)
%RC_TB_RUN  Simulate the T07 bench tests/rc/tb_rc_link.slx and return its outputs as plain arrays.
%
%   r = rc_tb_run(T, opTab, fltTab)          base-workspace P
%   r = rc_tb_run(T, opTab, fltTab, P)       P override for this run only (Simulink.SimulationInput
%                                            setVariable, the base workspace is not modified)
%   r = rc_tb_run(..., 'SimulationMode', m)  'normal' (default) | 'accelerator'
%   Inputs as rc_tb_input. Output r (all sampled at 0.001 s, row k = time r.t(k)):
%     .t (N x 1)  .bytes (N x 32 uint8)  .len (N x 1)  .uart_err (N x 1 logical)
%     .ch_raw (N x 16 uint16)  .ch17 .ch18 .frame_lost .failsafe .sent (N x 1 logical)  - rc_frame port
%     .wall (s, execution wall time without compile), .meta (SimulationMetadata)

ip = inputParser;
ip.addParameter('SimulationMode', 'normal');
ip.parse(varargin{:});
mdl = 'tb_rc_link';
if ~bdIsLoaded(mdl), load_system(mdl); end
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(rc_tb_input(T, opTab, fltTab));
in = in.setModelParameter('StopTime', sprintf('%.6f', T), 'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on', ...
    'SimulationMode', ip.Results.SimulationMode);
if nargin >= 4 && ~isempty(P)
    in = in.setVariable('P', P);
end
out = sim(in);
if ~isempty(out.ErrorMessage), error('rc_tb_run:sim', '%s', out.ErrorMessage); end
y = out.yout;
sb = y.get('sbus').Values;
rf = y.get('rc_frame').Values;
r.t = sb.len.Time;
r.bytes = local_rows(sb.bytes.Data, numel(r.t));
r.len = double(sb.len.Data(:));
r.uart_err = logical(sb.uart_err.Data(:));
r.ch_raw = local_rows(rf.ch_raw.Data, numel(r.t));
for f = {'ch17', 'ch18', 'frame_lost', 'failsafe', 'sent'}
    r.(f{1}) = logical(rf.(f{1}).Data(:));
end
r.meta = out.SimulationMetadata;
r.wall = r.meta.TimingInfo.ExecutionElapsedWallTime;
end

function A = local_rows(D, n)
% logged vector data -> n x width
if size(D, 1) == n && ismatrix(D)
    A = D;
else
    A = reshape(D, [], n).';
end
end
