% fit_offsets.m - find the model-to-image offset of each printed diagram (read-only)
wd = 'C:\Users\aomar\AppData\Local\Temp\claude\C--Users-aomar-Desktop-Robot-project\6467a6df-9442-40e9-b20a-e0690423e424\scratchpad\fix1\figwork';
FIG = wd;
srcd = 'C:\Users\aomar\Desktop\Robot project\docs\report\figs';
fl = {'F01_system_top.png','F03_mcu_top.png','F04_mcu_task10.png','F07_drive_side.png','F08_drive_esc.png','F15_mode_manager.png'};
for ii = 1:numel(fl), if ~isfile(fullfile(wd, fl{ii})), copyfile(fullfile(srcd, fl{ii}), wd); end, end
S = jsondecode(fileread(fullfile(wd, 'positions.json')));
s = 200/72;
keys = fieldnames(S);
R = struct();
for k = 1:numel(keys)
    v = S.(keys{k});
    B = v.blocks;
    im = imread(fullfile(FIG, v.png));
    if size(im,3) > 1, g = rgb2gray(im(:,:,1:3)); else, g = im; end
    a = g < 140; [H, W] = size(a);
    [yy, xx] = find(a);
    P = reshape([B.pos], 4, [])';
    lp = v.linepts; if isempty(lp), lp = zeros(0,2); end
    ox0 = min(xx) - min([P(:,1); lp(:,1)])*s;
    oy0 = min(yy) - min([P(:,2); lp(:,2)])*s;
    keep = (P(:,3)-P(:,1)) > 25 & (P(:,4)-P(:,2)) > 25;
    Q = P(keep,:);
    % dilate horizontally/vertically by 1 px for tolerance
    ah = a | circshift(a,[0 1]) | circshift(a,[0 -1]);
    av = a | circshift(a,[1 0]) | circshift(a,[-1 0]);
    sc = @(ox, oy) scoreFn(ah, av, Q, s, ox, oy, H, W);
    best = [-1 0 0];
    for dx = -150:3:150
        for dy = -150:3:150
            q = sc(ox0+dx, oy0+dy);
            if q > best(1), best = [q ox0+dx oy0+dy]; end
        end
    end
    b0 = best;
    for dx = -3:3
        for dy = -3:3
            q = sc(b0(2)+dx, b0(3)+dy);
            if q > best(1), best = [q b0(2)+dx b0(3)+dy]; end
        end
    end
    mx = sum(2*((Q(:,4)-Q(:,2))*s-6) + 2*((Q(:,3)-Q(:,1))*s-6));
    R.(keys{k}) = struct('png', v.png, 'ox', best(2), 'oy', best(3), 's', s, 'match', best(1)/mx);
    fprintf('%s offset %.1f %.1f (est %.1f %.1f) match %.2f\n', v.png, best(2), best(3), ox0, oy0, best(1)/mx);
end
fid = fopen(fullfile(wd, 'offsets.json'), 'w'); fwrite(fid, jsonencode(R), 'char'); fclose(fid);

function t = scoreFn(ah, av, Q, s, ox, oy, H, W)
t = 0;
for i = 1:size(Q,1)
    x0 = round(ox + Q(i,1)*s); x1 = round(ox + Q(i,3)*s); y0 = round(oy + Q(i,2)*s); y1 = round(oy + Q(i,4)*s);
    if x0 < 2 || y0 < 2 || x1 > W-1 || y1 > H-1, continue; end
    t = t + nnz(ah(y0+3:y1-3, x0)) + nnz(ah(y0+3:y1-3, x1)) + nnz(av(y0, x0+3:x1-3)) + nnz(av(y1, x0+3:x1-3));
end
end
