classdef MissionType < uint8
%MissionType  Mission type.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        NONE (0)
        PALM_ROW_PATROL (1)
        GREENHOUSE_COVERAGE (2)
        SOLAR_ROW_MOWING (3)
        GOTO (4)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
