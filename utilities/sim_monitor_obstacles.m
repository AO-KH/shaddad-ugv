function L = sim_monitor_obstacles(W, kind, mon)
%SIM_MONITOR_OBSTACLES  Static obstacle tables of Sim_Monitors (evaluated once at model initialisation).
%
%   L = sim_monitor_obstacles(W, 'cyl', mon)   N x 3 [xE yN r]          (N >= mon.Kc)
%   L = sim_monitor_obstacles(W, 'box', mon)   N x 5 [xE yN yaw hx hy]  (N >= mon.Kb)
%     W    world struct P.world (obstacles.cyl/nCyl, box/nBox, film/nFilm; rows 1..n* only)
%     mon  P.sim.mon (Kc, Kb, z_robot, film_thick, far)
%
%   Cylinders and boxes whose vertical extent [zBase, zBase + h] does not reach the robot height band
%   [0, mon.z_robot] are dropped (e.g. canopy-level parts). Greenhouse film segments [x1 y1 x2 y2]
%   become mon.film_thick thick boxes along the segment (D42). The tables are padded with far-away
%   dummy obstacles (at mon.far) so the nearest-K selectors always have K rows. Used in the Constant
%   blocks of Sim_Monitors and by tests (integration spec 4.5, build plan T16.1).

O = W.obstacles;
switch lower(kind)
    case 'cyl'
        C = zeros(0, 3);
        for k = 1:O.nCyl
            c = O.cyl(k, :);                          % [xE yN r h zBase class]
            if c(5) < mon.z_robot && c(5) + c(4) > 0
                C(end+1, :) = c(1:3); %#ok<AGROW>
            end
        end
        nPad = max(0, mon.Kc - size(C, 1));
        pad = [mon.far + 10*(1:nPad)' , mon.far*ones(nPad, 1), 0.01*ones(nPad, 1)];
        L = [C; pad];
    case 'box'
        B = zeros(0, 5);
        for k = 1:O.nBox
            b = O.box(k, :);                          % [xE yN zBase lx ly lz yaw class id]
            if b(3) < mon.z_robot && b(3) + b(6) > 0
                B(end+1, :) = [b(1) b(2) b(7) b(4)/2 b(5)/2]; %#ok<AGROW>
            end
        end
        for k = 1:O.nFilm
            f = O.film(k, :);                         % [x1 y1 x2 y2]
            len = hypot(f(3) - f(1), f(4) - f(2));
            if len < eps, continue, end
            B(end+1, :) = [(f(1) + f(3))/2, (f(2) + f(4))/2, atan2(f(4) - f(2), f(3) - f(1)), ...
                           len/2, mon.film_thick/2]; %#ok<AGROW>
        end
        nPad = max(0, mon.Kb - size(B, 1));
        pad = [mon.far + 10*(1:nPad)', mon.far*ones(nPad, 1), zeros(nPad, 1), 0.01*ones(nPad, 2)];
        L = [B; pad];
    otherwise
        error('sim_monitor_obstacles:kind', 'kind must be ''cyl'' or ''box''');
end
end
