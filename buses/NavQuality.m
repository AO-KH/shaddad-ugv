classdef NavQuality < uint8
%NavQuality  Navigation solution quality (Shaddad_NavFusion).
%   Integer enumeration (storage uint8) used INSIDE models (MATLAB Function FSMs, Enumerated Constant).
%   Inter-model buses carry the same codes as uint8 (see docs/icd.md). Do not renumber.
    enumeration
        INVALID (0)
        DR (1)
        GNSS_SINGLE (2)
        GNSS_FLOAT (3)
        RTK_FIXED (4)
        LIDAR_LOCALIZED (5)
    end
    methods (Static)
        function tf = addClassNameToEnumNames()
            % Prefix generated C enum literals with the class name (several enums share names).
            tf = true;
        end
    end
end
