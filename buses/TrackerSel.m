classdef TrackerSel < uint8
%TrackerSel  GNC active path tracker.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        STOP (0)
        PP (1)
        ROW (2)
        TEB (3)
        ALIGN (4)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
