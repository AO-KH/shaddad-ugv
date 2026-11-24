function file = latest_file(id)
%LATEST_FILE  newest work/results/<id>_2*.mat (no load).
d = dir(fullfile('C:\Users\aomar\Desktop\Robot project', 'work', 'results', [id '_2*.mat']));
if isempty(d), file = ''; return, end
[~, k] = max([d.datenum]);
file = fullfile(d(k).folder, d(k).name);
end
