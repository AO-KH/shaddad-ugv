classdef GncState < uint8
%GncState  GNC mission-manager top state.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        IDLE (0)
        MANUAL (1)
        AUTO_MISSION (2)
        PAUSE (3)
        OBSTACLE_HOLD (4)
        RETURN_TO_HOME (5)
        FAULT (6)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
