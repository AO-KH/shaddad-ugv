classdef AutoSub < uint8
%AutoSub  GNC AUTO sub-mode.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        NONE (0)
        ALIGN (1)
        TRANSIT (2)
        ROW (3)
        TURN (4)
        BYPASS (5)
        REJOIN (6)
        RTH_PLANNING (7)
        RTH_TRANSIT (8)
        DOCK_APPROACH (9)
        DOCKED (10)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
