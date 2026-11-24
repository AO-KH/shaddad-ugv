function left = shaddad_close_since(before)
%SHADDAD_CLOSE_SINCE  Close, without saving, every block diagram loaded since a list of loaded diagrams.
%
%   before = find_system('SearchDepth', 0, 'type', 'block_diagram');   % taken first
%   ...                                     (code that loads models, Subsystem Reference files, libraries)
%   shaddad_close_since(before)             close every block diagram loaded since, without saving
%   left = shaddad_close_since(before)      also return the names still loaded (a cellstr column, empty when
%                                           everything closed)
%
%   A diagram that another loaded diagram still references may refuse to close until that one is closed, so
%   the closing runs in passes, at most 3, and stops early when a pass closes nothing. A diagram still loaded
%   after the last pass (for example a model in compiled state) is named, with the last error closing it
%   gave, in one warning shaddad_close_since:left. The diagrams of before are never closed.
%   shaddad_layout_check and shaddad_variant_sweep call it in their onCleanup, so a check or a sweep leaves the
%   session with the block diagrams it found, also after an error.

before = cellstr(before);
why = containers.Map('KeyType', 'char', 'ValueType', 'char');   % name -> last error while closing it
for pass = 1:3
    new = setdiff(find_system('SearchDepth', 0, 'type', 'block_diagram'), before);
    closed = 0;
    for k = 1:numel(new)
        if ~bdIsLoaded(new{k}), closed = closed + 1; continue, end   % closed with another one
        try
            close_system(new{k}, 0);
            closed = closed + 1;
        catch tmp_cs_err
            why(new{k}) = tmp_cs_err.message;   % still referenced or busy: the next pass, else the warning
        end
    end
    if isempty(new) || closed == 0, break, end
end
left = setdiff(find_system('SearchDepth', 0, 'type', 'block_diagram'), before);
left = reshape(left, [], 1);
if ~isempty(left)
    txt = cell(size(left));
    for k = 1:numel(left)
        if isKey(why, left{k}), txt{k} = sprintf('%s (%s)', left{k}, why(left{k})); else, txt{k} = left{k}; end
    end
    warning('shaddad_close_since:left', 'Block diagrams still loaded after closing: %s', strjoin(txt, '; '));
end
end
