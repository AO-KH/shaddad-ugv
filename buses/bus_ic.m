function ic = bus_ic(defs)
%BUS_IC  Initial-condition structs for every bus, built directly from the definitions.
%   ic = bus_ic(defs)   defs = shaddad_buses('defs'); ic.Bus_<Name> = IC struct
%   Same field order, sizes and classes as Simulink.Bus.createMATLABStruct but about 1000x faster
%   (no Simulink round trip). Values are zero, except that an element whose declared range
%   [Min, Max] excludes zero gets the range bound nearest to zero (e.g. Bus_EnvState.T_amb = 233.15 K,
%   soil_id = 1): Simulink checks Constant / Unit Delay / Rate Transition bus ICs against the element
%   Min/Max ("Inconsistent numeric values ... less than the minimum"), so pure zeros would not compile.
%   Scalar/vector Dimensions n -> n x 1, [r c ...] -> r x c x ...; boolean -> logical.
names = {defs.name};
ic = struct();
for k = 1:numel(defs)
    ic.(names{k}) = local_build(k, defs, names);
end
end

function s = local_build(k, defs, names)
s = struct();
el = defs(k).elems;
for m = 1:numel(el)
    e = el{m};
    d = e.dims; if isscalar(d), d = [d 1]; end
    if startsWith(e.dtype, 'Bus: ')
        j = find(strcmp(names, strtrim(extractAfter(e.dtype, 'Bus: '))), 1);
        sub = local_build(j, defs, names);
        s.(e.name) = repmat(sub, d);
    elseif strcmp(e.dtype, 'boolean')
        s.(e.name) = false(d);
    else
        v = 0;
        if ~isempty(e.min) && isfinite(e.min) && e.min > 0, v = e.min; end
        if ~isempty(e.max) && isfinite(e.max) && e.max < 0, v = e.max; end
        s.(e.name) = cast(v * ones(d), e.dtype);
    end
end
end
