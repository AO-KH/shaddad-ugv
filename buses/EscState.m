classdef EscState < uint8
%EscState  Traction ESC DS402-like state (drive_esc ESC_SM).
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        INIT (0)
        NOT_READY (1)
        READY (2)
        ENABLING (3)
        OPERATION (4)
        DISABLING (5)
        QUICK_STOP (6)
        FAULT (7)
        STO (8)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
