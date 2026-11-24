wd = 'C:\Users\aomar\AppData\Local\Temp\claude\C--Users-aomar-Desktop-Robot-project\6467a6df-9442-40e9-b20a-e0690423e424\scratchpad\fix1\figwork';
S = jsondecode(fileread(fullfile(wd, 'positions.json')));
B = jsondecode(fileread(fullfile(wd, 'pp_positions.json')));
S.F09_power_plant_png.blocks = B;
S.F09_power_plant_png.png = 'F09_power_plant_arranged.png';
S.F09_power_plant_png.linepts = zeros(0,2);
fid = fopen(fullfile(wd, 'positions.json'), 'w'); fwrite(fid, jsonencode(S), 'char'); fclose(fid);
