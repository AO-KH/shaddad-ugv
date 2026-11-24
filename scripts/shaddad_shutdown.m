function names = shaddad_shutdown(mode)
%SHADDAD_SHUTDOWN  Project shutdown: remove the Shaddad data from the base workspace.
%
%   shaddad_shutdown()                 clear P, Ts, P_meta, mpcRowObj, VC_*, Bus_*, cs_* from base
%   names = shaddad_shutdown('dryrun') return the variable names that would be cleared, clear nothing
%
%   Only these names are touched (build plan T01.1); models are not closed, no clear all.

if nargin < 1, mode = 'clear'; end
w = evalin('base', 'who');
fixed = {'P', 'Ts', 'P_meta', 'mpcRowObj'};
sel = ismember(w, fixed) | startsWith(w, 'VC_') | startsWith(w, 'Bus_') | startsWith(w, 'cs_');
names = w(sel);
if strcmpi(mode, 'dryrun') || isempty(names)
    return
end
evalin('base', ['clear ' strjoin(names(:)', ' ')]);
end
