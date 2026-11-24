function [so, wall] = tb_drive_run(model, tend, mwVars, globVars, modelParams)
%TB_DRIVE_RUN  Simulate a T06 drive bench with overrides (no base-workspace writes).
%   [so, wall] = tb_drive_run(model, tend, mwVars, globVars, modelParams)
%     model        'tb_drive' | 'tb_drive_sys' | 'tb_drive_2mass' | 'tb_drive_foc'
%     tend         stop time (s)
%     mwVars       struct of bench model-workspace variables (tb_*), applied with setVariable(...,'Workspace',model)
%     globVars     struct of global variables shadowing the base workspace for this run (e.g. P, VC_DRIVE_FID)
%     modelParams  cell {name, value, ...} of model parameters (e.g. {'SimulationMode', 'accelerator'} for long runs)
%   The bench load (tb_drive_load: tb_J, tb_Trr / tb_Jb_spr) follows the P of the run unless mwVars sets it; the
%   defaults stored in the bench are those of the P it was built with.
if nargin < 3 || isempty(mwVars), mwVars = struct(); end
if nargin < 4 || isempty(globVars), globVars = struct(); end
if nargin < 5, modelParams = {}; end
if ~bdIsLoaded(model), load_system(model); end
if isfield(globVars, 'P'), Pr = globVars.P; else, Pr = evalin('base', 'P'); end
ld = tb_drive_load(model, Pr); fn = fieldnames(ld);
for k = 1:numel(fn), if ~isfield(mwVars, fn{k}), mwVars.(fn{k}) = ld.(fn{k}); end, end
si = Simulink.SimulationInput(model);
fn = fieldnames(mwVars);
for k = 1:numel(fn), si = si.setVariable(fn{k}, mwVars.(fn{k}), 'Workspace', model); end
fn = fieldnames(globVars);
for k = 1:numel(fn), si = si.setVariable(fn{k}, globVars.(fn{k})); end
si = si.setModelParameter('StopTime', num2str(tend, 12));
si = si.setModelParameter('SimulationMode', 'normal');   % default; modelParams may select 'accelerator' (JIT, no C compiler needed)
for k = 1:2:numel(modelParams)
    if strcmp(modelParams{k}, 'InitialState') && ~ischar(modelParams{k + 1})
        si = si.setInitialState(modelParams{k + 1});           % operating point of a previous chunk
    else
        si = si.setModelParameter(modelParams{k}, modelParams{k + 1});
    end
end
w = warning('off', 'Simulink:Parameters:ParamPrecisionLossWarn');
c = onCleanup(@() warning(w));
t0 = tic;
try
    so = sim(si);
catch err
    % the first compile after drive_esc.slx was regenerated may fail once in R2026b with a Stateflow JIT relink
    % assertion ('o != nullptr'); a second compile passes
    if ~contains(getReport(err, 'extended', 'hyperlinks', 'off'), 'o != nullptr'), rethrow(err); end
    t0 = tic;
    so = sim(si);
end
wall = toc(t0);
if ~isempty(so.ErrorMessage), error('tb_drive_run:sim', '%s: %s', model, so.ErrorMessage); end
end
