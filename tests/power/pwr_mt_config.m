function pwr_mt_config(mdl, action)
%PWR_MT_CONFIG  In-memory config swap that lets model_test run on a Shaddad model (owner: power).
%
%   pwr_mt_config('Shaddad_BMS', 'local')    before model_test
%   pwr_mt_config('Shaddad_BMS', 'restore')  after model_test
%
%   Every Shaddad model uses cs_Shaddad through a Simulink.ConfigSetRef. model_test (Simulink Agentic
%   Toolkit) rebuilds its harness from the main model on every run and then calls set_param on the
%   harness configuration, which a reference refuses (Simulink:ConfigSet:ConfigSetRef_SetParamNotAllowed).
%   'local' attaches a copy of the referenced set and makes it active; 'restore' makes the reference
%   active again and detaches the copy. The model is never saved. The dirty flag is cleared after each
%   swap, because model_test saves a dirty main model before it creates a harness, and that would write
%   the local copy into the generated .slx. 'local' refuses a model that is already dirty.
%   Use it in the shared MATLAB through a function call only (no base-workspace variables).
switch action
    case 'local'
        cref = getActiveConfigSet(mdl);
        if ~isa(cref, 'Simulink.ConfigSetRef')
            fprintf('%s: active config is already local (%s)\n', mdl, cref.Name);
            return
        end
        if ~strcmp(get_param(mdl, 'Dirty'), 'off')
            error('pwr_mt_config:dirty', '%s has unsaved changes: not swapped', mdl);
        end
        cs = cref.getRefConfigSet().copy;
        cs.Name = 'mt_local_copy';
        attachConfigSet(mdl, cs, true);
        setActiveConfigSet(mdl, cs.Name);
        set_param(mdl, 'Dirty', 'off');
    case 'restore'
        names = getConfigSets(mdl);
        refName = '';
        for k = 1:numel(names)
            if isa(getConfigSet(mdl, names{k}), 'Simulink.ConfigSetRef'), refName = names{k}; end
        end
        if isempty(refName)
            error('pwr_mt_config:noref', '%s has no configuration reference to restore', mdl);
        end
        setActiveConfigSet(mdl, refName);
        for k = 1:numel(names)
            if startsWith(names{k}, 'mt_local_copy'), detachConfigSet(mdl, names{k}); end
        end
        set_param(mdl, 'Dirty', 'off');
    otherwise
        error('pwr_mt_config:action', 'action must be ''local'' or ''restore''');
end
fprintf('%s: active config %s (%s), config sets: %s, dirty %s\n', mdl, getActiveConfigSet(mdl).Name, ...
    class(getActiveConfigSet(mdl)), strjoin(getConfigSets(mdl), ', '), get_param(mdl, 'Dirty'));
end
