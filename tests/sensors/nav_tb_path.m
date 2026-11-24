function kin = nav_tb_path(segs, base)
%NAV_TB_PATH  Breakpoint table prof.kin of tb_nav from a list of motion segments (build task T12).
%
%   kin = nav_tb_path(segs, base)
%   base: 1 x 8 [v r vy sL sR chi gx gy] starting values (v and r are overwritten by the segments)
%   segs: cell array of segments, each a cell {type, args...}:
%     {'wait', T}             hold v = 0, r = 0 for T s
%     {'speed', v, T}         ramp v linearly to v in T s (r = 0), e.g. {'speed', 1, 2}
%     {'straight', T}         hold the current v for T s (r = 0)
%     {'turn', ang, w}        turn by ang rad at body rate sign(ang)*abs(w) with the current v (0.2 s rate ramps)
%     {'set', idx, val, T}    ramp prof element idx (3..8: vy sL sR chi gx gy) to val in T s (others held)
%   kin rows [t v r vy sL sR chi gx gy] (linear interpolation in the bench root port).
t = 0;
x = base(:).';
kin = [t x];
tr = 0.2;   % rate ramp time of a turn
for k = 1:numel(segs)
    s = segs{k};
    switch s{1}
        case 'wait'
            x(1) = 0; x(2) = 0;
            t = t + s{2};
            kin(end+1, :) = [t x]; %#ok<AGROW>
        case 'speed'
            x(2) = 0; x(1) = s{2};
            t = t + s{3};
            kin(end+1, :) = [t x]; %#ok<AGROW>
        case 'straight'
            x(2) = 0;
            t = t + s{2};
            kin(end+1, :) = [t x]; %#ok<AGROW>
        case 'turn'
            ang = s{2}; w = abs(s{3})*sign(ang);
            % ramps of tr s at both ends cover w*tr of angle in total; hold covers the rest
            th = abs(ang)/abs(w) - tr;
            x(2) = w;  t = t + tr;  kin(end+1, :) = [t x]; %#ok<AGROW>
            t = t + max(th, 0);   kin(end+1, :) = [t x]; %#ok<AGROW>
            x(2) = 0;  t = t + tr;  kin(end+1, :) = [t x]; %#ok<AGROW>
        case 'set'
            x(s{2}) = s{3};
            t = t + s{4};
            kin(end+1, :) = [t x]; %#ok<AGROW>
        otherwise
            error('nav_tb_path:seg', 'Unknown segment %s', s{1});
    end
end
end
