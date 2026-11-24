function R = nav_tb_run(P, prof, tbn, varargin)
%NAV_TB_RUN  Run the navigation bench tests/sensors/tb_nav.slx and return its logs as plain arrays (T12).
%
%   R = nav_tb_run(P, prof)              P: parameter struct (setVariable, the base workspace is not changed)
%   R = nav_tb_run(P, prof, tbn)         tbn: bench model-workspace struct overrides (pose0, seedSm, seedVel)
%   R = nav_tb_run(..., 'Mode', m)       simulation mode of the bench ('normal' default)
%   R = nav_tb_run(..., 'GpsVelNoise', s)  'bench' (default): the velocity noise of the plant GPS blocks is
%                                        switched off (P.sensors.gnss.sigVel = 0) and the bench adds zero-mean white
%                                        noise of the reported sAcc (gpsSensor adds |noise| to the ground speed, an
%                                        along-track bias of 1.2533 sigVel); 'plant': keep the plant GPS velocity noise
%   prof: see nav_tb_input. Output R:
%     R.nav    struct of Bus_NavState fields (time first, matrices r x c x N), plus .time
%     R.diag   struct of Bus_NavDiag fields
%     R.truth  N x 15 [t x y z psi theta phi v vy r sL sR chi vG_L vG_R] at 0.01 s (aligned with R.nav)
%     R.gnss   Bus_GnssRaw fields (0.1 s, as fed to NavFusion);  R.sdiag Bus_SensTruthDiag;  R.odom Bus_OdomMeas;
%     R.sm     Bus_ScanMatchMeas (0.2 s);  R.wall wall-clock time of sim [s]
ip = inputParser;
ip.addParameter('Mode', 'normal');
ip.addParameter('GpsVelNoise', 'bench');
ip.parse(varargin{:});
if nargin < 3 || isempty(tbn), tbn = struct(); end
tb = struct('pose0', [0 0 0], 'seedSm', [9101 9102 9103 9104], 'seedVel', [9201 9202 9203]);
fn = fieldnames(tbn);
for k = 1:numel(fn), tb.(fn{k}) = tbn.(fn{k}); end
if strcmpi(ip.Results.GpsVelNoise, 'bench')
    P.sensors.gnss.sigVel(:) = 0;
end
if ~bdIsLoaded('tb_nav'), load_system('tb_nav'); end
ds = nav_tb_input(P, prof);
in = Simulink.SimulationInput('tb_nav');
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(prof.T, 17), 'SimulationMode', ip.Results.Mode);
in = in.setVariable('P', P);
in = in.setVariable('tbn', tb, 'Workspace', 'tb_nav');
t0 = tic;
so = sim(in);
R.wall = toc(t0);
if ~isempty(so.ErrorMessage), error('nav_tb_run:sim', '%s', so.ErrorMessage); end
y = so.yout;
R.nav = local_bus(y{1}.Values);
R.diag = local_bus(y{2}.Values);
tr = y{3}.Values;
R.truth = squeeze(tr.Data);
if size(R.truth, 1) ~= numel(tr.Time), R.truth = R.truth.'; end
R.gnss = local_bus(y{4}.Values);
R.sdiag = local_bus(y{5}.Values);
R.odom = local_bus(y{6}.Values);
R.sm = local_bus(y{7}.Values);
end

function S = local_bus(V)
% struct of timeseries -> struct of data arrays (+ .time of the first leaf)
S = struct();
f = fieldnames(V);
for k = 1:numel(f)
    v = V.(f{k});
    if isstruct(v)
        S.(f{k}) = local_bus(v);
        continue
    end
    if ~isfield(S, 'time'), S.time = v.Time; end
    S.(f{k}) = v.Data;
end
end
