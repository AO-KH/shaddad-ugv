function d = shaddad_layout_diagrams(sys)
%SHADDAD_LAYOUT_DIAGRAMS  The diagrams of one file that the layout check measures and the layout engine lays out.
%
%   d = shaddad_layout_diagrams(sys)
%   sys   a block diagram name (the root of a file) or a subsystem path
%   d     cellstr column: sys, then every subsystem level below it that belongs to the same file, in every
%         variant choice and under masks, in find_system order. Not the inside of Subsystem Reference blocks,
%         library links or Stateflow / MATLAB Function blocks (each of those is another file's diagram or no
%         Simulink diagram), nor anything below one of them; Model blocks are not subsystems, so never.
%
%   One selection for both tools (model layout spec 7 and 8, Task 4 decision 4): shaddad_layout_check reports
%   one row per diagram of shaddad_layout_diagrams(file), and shaddad_layout(sys, 'Recursive', true) lays out
%   exactly shaddad_layout_diagrams(sys). It is the selection of work/layout/inv_diagrams.m.

sys = char(sys);
d = {sys};
ss = find_system(sys, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'MatchFilter', @Simulink.match.allVariants, ...
    'BlockType', 'SubSystem');
for k = 1:numel(ss)
    if ~strcmp(ss{k}, sys) && ~local_skipped_chain(ss{k}, sys), d{end+1, 1} = ss{k}; end %#ok<AGROW>
end
end

% -------------------------------------------------------------------------------------------
function tf = local_skipped_chain(b, sys)
% true when b or any ancestor below sys is a Subsystem Reference block, a library link or a Stateflow /
% MATLAB Function block
tf = false;
p = b;
while ~strcmp(p, sys)
    if local_skip(p), tf = true; return, end
    p = get_param(p, 'Parent');
end
end

function tf = local_skip(b)
tf = false;
try
    if ~isempty(get_param(b, 'ReferencedSubsystem')), tf = true; return, end
catch %#ok<CTCH> no such parameter
end
try
    if any(strcmp(get_param(b, 'LinkStatus'), {'resolved', 'implicit'})), tf = true; return, end
catch %#ok<CTCH>
end
try
    if ~strcmp(get_param(b, 'SFBlockType'), 'NONE'), tf = true; return, end
catch %#ok<CTCH>
end
end
