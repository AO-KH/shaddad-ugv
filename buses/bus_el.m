function e = bus_el(name, dtype, dims, unit, desc, mn, mx)
%BUS_EL  One bus-element definition row (plain struct, no Simulink objects).
%   e = bus_el(name, dtype, dims, unit, desc)
%   e = bus_el(name, dtype, dims, unit, desc, min, max)
%
%   name  : element name (valid MATLAB identifier)
%   dtype : 'double' | 'single' | 'boolean' | 'uint8' | 'int8' | 'uint16' | 'int16' |
%           'uint32' | 'int32' | 'Bus: Bus_<Name>'
%   dims  : 1 (scalar), n (vector), [r c] or [r c p] (matrix / array)
%   unit  : Simulink unit string ('' for booleans, enums, bit-fields and counts, '1' = dimensionless)
%   desc  : one-line description, incl. sign convention / enum codes
%   min/max : optional physical range ([] = unbounded). Stored in Simulink.BusElement Min/Max.
%
%   Used by buses_<domain>.m; consumed by shaddad_buses.m and utilities/shaddad_icd_report.m.
if nargin < 6, mn = []; end
if nargin < 7, mx = []; end
e = struct('name', name, 'dtype', dtype, 'dims', dims, 'unit', unit, ...
           'desc', desc, 'min', mn, 'max', mx);
end
