function [r, file] = scn_latest(id)
%SCN_LATEST  Latest saved run_scenario result of a scenario id (work/results/<id>_<yyyymmdd_HHMMSS>.mat).
%
%   [r, file] = scn_latest('S1')     r = loaded struct (id, S, eval, rtf, wall_s, ...) or [] when none
%   The tests read these files so that the long system runs are evaluated without re-simulating them in
%   every regression; tests/system/tScenarios re-simulates when SHADDAD_SCN_RERUN=1 or no result exists.

r = [];  file = '';
d = dir(fullfile(char(currentProject().RootFolder), 'work', 'results', [id '_2*.mat']));
if isempty(d), return, end
[~, k] = max([d.datenum]);
file = fullfile(d(k).folder, d(k).name);
r = load(file);
end
