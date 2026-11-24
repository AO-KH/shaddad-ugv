classdef gnc_slopeValidator < validatorOccupancyMap
%GNC_SLOPEVALIDATOR  Occupancy validator that also forbids turning on steep terrain (D53; used by gnc_globalReplanner).
%
%   sv = gnc_slopeValidator(ss, map, terr, prm, start)
%     ss, map  stateSpaceSE2 and the inflated binaryOccupancyMap (as validatorOccupancyMap)
%     terr     struct xE, yN, dHdE, dHdN of P.world.terrain (Table(i, j) <-> (xE(i), yN(j)))
%     prm      P.gnc.hastar: slope_turn_max (rad), slope_head_tol (rad), slope_base (m, averaging length of the DEM
%              slope), R_min (start exemption radius)
%     start    [x y psi] of the plan (states within R_min of it are exempt: the robot must be able to leave)
%
%   A state on terrain steeper than slope_turn_max is valid only when its heading is within slope_head_tol of the
%   fall line (straight up or down the slope). plannerHybridAStar validates both its motion primitives and its
%   analytic (Dubins) expansions with isStateValid, so no turn - and no drive across the slope (roll) - is
%   planned there.

    properties
        AnySteep = false % terrain has cells steeper than slope_turn_max
        GiSteep          % griddedInterpolant of the steep mask (nearest)
        GiFall           % griddedInterpolant of the fall-line angle atan2(dHdN, dHdE)
        SinTol = 0
        Start = [0 0]
        RExempt = 0
    end

    methods
        function obj = gnc_slopeValidator(ss, map, terr, prm, start)
            obj@validatorOccupancyMap(ss, 'Map', map);
            g = {double(terr.xE(:)), double(terr.yN(:))};
            % slope of the DEM averaged over the footprint length (box filter): the vehicle attitude follows the
            % terrain under its tracks, so short features (basin berms in the palm farm, 0.3 m wide, 21 deg) do not count
            res = max(abs(g{1}(min(2, end)) - g{1}(1)), 1e-6);
            n = max(1, round(prm.slope_base/res));
            k = ones(n)/n^2;
            gx = conv2(double(terr.dHdE), k, 'same');  gy = conv2(double(terr.dHdN), k, 'same');
            if numel(g{1}) < 2 || numel(g{2}) < 2, gx = double(terr.dHdE); gy = double(terr.dHdN); end
            steep = atan(hypot(gx, gy)) > prm.slope_turn_max;
            obj.AnySteep = any(steep(:));
            obj.GiSteep = griddedInterpolant(g, double(steep), 'nearest', 'nearest');
            obj.GiFall = griddedInterpolant(g, atan2(gy, gx), 'nearest', 'nearest');
            obj.SinTol = sin(prm.slope_head_tol);
            obj.Start = double(start(1:2));
            obj.RExempt = prm.R_min;
        end

        function isValid = isStateValid(obj, state)
            isValid = isStateValid@validatorOccupancyMap(obj, state);
            if ~obj.AnySteep
                return
            end
            x = state(:, 1);  y = state(:, 2);
            steep = obj.GiSteep(x, y) > 0.5 & hypot(x - obj.Start(1), y - obj.Start(2)) > obj.RExempt;
            if any(steep)
                across = abs(sin(state(:, 3) - obj.GiFall(x, y))) > obj.SinTol;
                isValid = isValid & ~(steep & across);
            end
        end
    end
end
