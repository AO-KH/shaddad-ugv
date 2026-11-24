% export_positions.m - read-only: block positions of the printed Simulink diagrams
out = 'C:\Users\aomar\AppData\Local\Temp\claude\C--Users-aomar-Desktop-Robot-project\6467a6df-9442-40e9-b20a-e0690423e424\scratchpad\fix1\figwork\positions.json';
specs = {
  'F01_system_top.png',  'Shaddad_System',        'Shaddad_System',  false
  'F03_mcu_top.png',     'Shaddad_MCU',           'Shaddad_MCU',     false
  'F04_mcu_task10.png',  'Shaddad_MCU',           'Shaddad_MCU/Task_10', false
  'F07_drive_side.png',  'drive_side',            'drive_side',      false
  'F08_drive_esc.png',   'drive_esc',             'drive_esc',       false
  'F15_mode_manager.png','fw_mode_manager',       'fw_mode_manager', false
  'F09_power_plant.png', 'Power_Plant',           'Power_Plant',     true
  };
S = struct();
for k = 1:size(specs,1)
    [png, mdl, sys, arrange] = specs{k,:};
    wasLoaded = bdIsLoaded(mdl);
    load_system(mdl);
    if arrange
        Simulink.BlockDiagram.arrangeSystem(sys);
    end
    blks = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
    blks = blks(~strcmp(blks, sys));
    B = struct('name', {}, 'type', {}, 'pos', {});
    for i = 1:numel(blks)
        B(end+1) = struct('name', get_param(blks{i}, 'Name'), 'type', get_param(blks{i}, 'BlockType'), ...
            'pos', get_param(blks{i}, 'Position')); %#ok<SAGROW>
    end
    L = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    pts = zeros(0,2);
    for i = 1:numel(L), pts = [pts; get_param(L(i), 'Points')]; end %#ok<AGROW>
    A = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'annotation');
    apos = zeros(0,4);
    for i = 1:numel(A), apos = [apos; get_param(A(i), 'Position')]; end %#ok<AGROW>
    key = matlab.lang.makeValidName(png);
    S.(key) = struct('png', png, 'model', mdl, 'sys', sys, 'blocks', B, 'linepts', pts, 'annot', apos, ...
        'fontsize', get_param(bdroot(sys), 'DefaultBlockFontSize'));
    if arrange
        % layout change in memory only: discard
        close_system(mdl, 0);
    elseif ~wasLoaded
        % keep loaded models loaded only if they were; never save
        try, close_system(mdl, 0); catch, end
    end
end
fid = fopen(out, 'w'); fwrite(fid, jsonencode(S), 'char'); fclose(fid);
disp('positions written');
