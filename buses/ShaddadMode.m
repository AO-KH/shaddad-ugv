classdef ShaddadMode < uint8
%ShaddadMode  MCU mode (Shaddad_MCU ModeManager).
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        BOOT (0)
        SELFTEST (1)
        STANDBY (2)
        MANUAL (3)
        HOLD (4)
        AUTO_START (5)
        AUTO (6)
        CTRL_STOP (7)
        ESTOP (8)
        FAULT (9)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
