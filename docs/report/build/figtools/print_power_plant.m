% print_power_plant.m - re-print Power_Plant after arrangeSystem in memory (file never saved)
wd = 'C:\Users\aomar\AppData\Local\Temp\claude\C--Users-aomar-Desktop-Robot-project\6467a6df-9442-40e9-b20a-e0690423e424\scratchpad\fix1\figwork';
mdl = 'Power_Plant';
load_system(mdl);
Simulink.BlockDiagram.arrangeSystem(mdl);
print(['-s' mdl], '-dpng', '-r200', fullfile(wd, 'F09_power_plant_arranged.png'));
blks = find_system(mdl, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
blks = blks(~strcmp(blks, mdl));
B = struct('name', {}, 'type', {}, 'pos', {});
for i = 1:numel(blks)
    B(end+1) = struct('name', get_param(blks{i}, 'Name'), 'type', get_param(blks{i}, 'BlockType'), 'pos', get_param(blks{i}, 'Position')); %#ok<SAGROW>
end
fid = fopen(fullfile(wd, 'pp_positions.json'), 'w'); fwrite(fid, jsonencode(B), 'char'); fclose(fid);
close_system(mdl, 0);
info = imfinfo(fullfile(wd, 'F09_power_plant_arranged.png')); fprintf('%d x %d\n', info.Width, info.Height);
