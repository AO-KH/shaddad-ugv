function [t, v] = sg(s, col)
%SG  time and double value of a logged signal struct (fields t, v); optional column.
t = double(s.t(:));
v = double(s.v);
if ndims(v) > 2, v = reshape(permute(v, [ndims(v) 1:ndims(v)-1]), size(v, ndims(v)), []); end
if size(v, 1) ~= numel(t) && size(v, 2) == numel(t), v = v.'; end
if nargin > 1 && ~isempty(col), v = v(:, col); end
end
