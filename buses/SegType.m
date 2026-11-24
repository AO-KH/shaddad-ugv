classdef SegType < uint8
%SegType  Mission segment type.
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        NONE (0)
        TRANSIT (1)
        ROW (2)
        TURN (3)
        DOCK (4)
        TERMINAL (5)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
