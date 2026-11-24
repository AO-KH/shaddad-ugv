classdef MissionCmd < uint8
%MissionCmd  Mission command (ground station / RC aux).
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        NONE (0)
        LOAD (1)
        START (2)
        PAUSE (3)
        RESUME (4)
        ABORT (5)
        RTH (6)
        RESET_FAULT (7)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
