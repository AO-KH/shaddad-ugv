classdef HoldReason < uint8
%HoldReason  GNC hold / pause reason.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        NONE (0)
        OBSTACLE (1)
        PERSON (2)
        VFH_BLOCKED (3)
        NAV_DEGRADED (4)
        OPERATOR (5)
        IMPLEMENT_NOT_READY (6)
        RC_LOSS (7)
        OBST_TIMEOUT (8)
        PLANNER_FAIL (9)
        SENSOR_LOST (10)    % LiDAR scan invalid (Bus_Perception.scanValid false for P.gnc.fsm.t_scan_lost)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
