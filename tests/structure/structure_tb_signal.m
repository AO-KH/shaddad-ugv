function [t, x] = structure_tb_signal(ts)
%STRUCTURE_TB_SIGNAL  Time and data of a logged bench signal as an [Nt x n] matrix (T03 test helper).
%   [t, x] = structure_tb_signal(out.tb_truth.pos_W)
%   ts : timeseries from a To Workspace block of tests/structure/tb_structure.slx (bus element).
%   Vector elements logged as [n x 1 x Nt] or [Nt x n] and matrices [r x c x Nt] become [Nt x n] / [Nt x r*c].
t = ts.Time(:);
d = ts.Data;
Nt = numel(t);
if isvector(d) && numel(d) == Nt
    x = d(:);
elseif size(d, 1) == Nt && ndims(d) == 2
    x = d;
else
    sz = size(d);
    if sz(end) == Nt
        x = reshape(d, [], Nt).';
    else
        x = reshape(d, Nt, []);
    end
end
x = double(x);
end
