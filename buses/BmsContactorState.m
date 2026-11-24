classdef BmsContactorState < uint8
%BmsContactorState  BMS mode manager state (Shaddad_BMS ModeManager).
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        INIT (0)
        STANDBY (1)
        PRECHARGE (2)
        CLOSE_MAIN (3)
        HV_ON (4)
        OPENING (5)
        WELD_CHECK (6)
        PRE_FAIL (7)
        CHARGING (8)
        CHARGE_DONE (9)
        FAULT (10)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
