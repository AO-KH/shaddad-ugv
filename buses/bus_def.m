function d = bus_def(name, ts, producer, consumers, desc, elems)
%BUS_DEF  One bus definition (plain struct) used by shaddad_buses / ICD generator.
%   d = bus_def(name, ts, producer, consumers, desc, elems)
%   name      : 'Bus_<Name>'
%   ts        : nominal sample time at the producer [s] (0 = continuous, Inf = constant,
%               -1 = inherited / internal sub-bus)
%   producer  : producing file/block (exactly one)
%   consumers : consuming files/blocks (char, comma separated)
%   desc      : purpose of the bus
%   elems     : cell array of bus_el(...) structs (order = element order)
d = struct('name', name, 'ts', ts, 'producer', producer, 'consumers', consumers, ...
           'desc', desc, 'elems', {elems});
end
