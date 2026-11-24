function rep = build_project()
%BUILD_PROJECT  Idempotent MATLAB-project setup of the Shaddad UGV (build plan T01.1).
%
%   rep = build_project()  creates the integration folders, adds them to the project, puts
%   subsystems/integration, scripts/build, scripts/scenarios and the tests/<domain> folders on the
%   project path, registers scripts/shaddad_startup.m / shaddad_shutdown.m as startup / shutdown
%   files and adds the T01 files to the project. Does not cd and does not reload the project.

proj = currentProject;
root = char(proj.RootFolder);
rep = struct('folders', {{}}, 'path', {{}}, 'files', {{}}, 'startup', false, 'shutdown', false);

dom = {'structure', 'power', 'drive', 'electronics', 'rc', 'sensors', 'gnc', 'integration', 'system'};
folders = [{'subsystems/integration', 'scripts/build', 'scripts/scenarios', 'scripts/missions', ...
            'utilities/stm32', 'work'}, strcat('tests/', dom)];
for k = 1:numel(folders)
    f = fullfile(root, folders{k});
    if ~isfolder(f), mkdir(f); end
    if ~local_inproject(proj, f)
        addFolderIncludingChildFiles(proj, f);
        rep.folders{end+1} = folders{k};
    end
end

pathFolders = [{'subsystems/integration', 'scripts/build', 'scripts/scenarios'}, strcat('tests/', dom)];
onPath = arrayfun(@(p) char(p.File), proj.ProjectPath, 'UniformOutput', false);
for k = 1:numel(pathFolders)
    f = fullfile(root, pathFolders{k});
    if ~any(strcmpi(onPath, f))
        addPath(proj, f);
        rep.path{end+1} = pathFolders{k};
    end
end

su = fullfile(root, 'scripts', 'shaddad_startup.m');
sd = fullfile(root, 'scripts', 'shaddad_shutdown.m');
startupFiles = cellfun(@char, cellstr(string(proj.StartupFiles)), 'UniformOutput', false);
shutdownFiles = cellfun(@char, cellstr(string(proj.ShutdownFiles)), 'UniformOutput', false);
if isfile(su) && ~any(strcmpi(startupFiles, su))
    if ~local_inproject(proj, su), addFile(proj, su); end
    addStartupFile(proj, su); rep.startup = true;
end
if isfile(sd) && ~any(strcmpi(shutdownFiles, sd))
    if ~local_inproject(proj, sd), addFile(proj, sd); end
    addShutdownFile(proj, sd); rep.shutdown = true;
end

t01 = {'scripts/shaddad_startup.m', 'scripts/shaddad_shutdown.m', 'scripts/run_scenario.m', ...
       'params/shaddad_configsets.m', 'params/param_sim.m', 'utilities/shaddad_check_interfaces.m', ...
       'utilities/scn_make_input.m', 'utilities/shaddad_rate_audit.m', 'scripts/scenarios/scn_base_input.m', ...
       'scripts/build/build_project.m', 'scripts/build/shaddad_bu.m', 'scripts/build/build_stubs.m', ...
       'scripts/build/build_shaddad_plant.m', 'scripts/build/build_shaddad_system.m', ...
       'scripts/build/build_shaddad_companion.m', 'tests/integration/t_infra.m', ...
       'models/Shaddad_System.slx', 'models/Shaddad_Plant.slx', 'models/Shaddad_Companion.slx'};
T = build_stubs('table');
t01 = [t01, {T([T.stub]).file}];
for k = 1:numel(t01)
    f = fullfile(root, t01{k});
    if isfile(f) && ~local_inproject(proj, f)
        addFile(proj, f);
        rep.files{end+1} = t01{k};
    end
end
end

function tf = local_inproject(proj, f)
tf = false;
try
    tf = ~isempty(findFile(proj, f));
catch %#ok<CTCH>
end
end
