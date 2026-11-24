function [n, t_done] = row_test_count_rows(t, xy, rows, tol, frac)
%ROW_TEST_COUNT_ROWS  Rows driven along a path, counted from the true positions (RC row test, spec section 9).
%   [n, t_done] = row_test_count_rows(t, xy, rows, tol, frac)
%     t      N sample times, s
%     xy     N x 2 robot positions [x y], m
%     rows   M x 5 centre lines [x1 y1 x2 y2 laneId] (row_test_rows)
%     tol    largest distance from the centre line that still counts as on the row, m (R.row_tol)
%     frac   part of the row length one pass must cover, 0..1 (R.row_frac)
%   A traversal of a row counts when one contiguous run of samples within tol of its centre line (a segment, so the
%   headland does not count) covers at least frac of its length. The cover is the span of the run along the line,
%   ends clipped to the row, so either direction counts. A run is an unbroken stretch of consecutive samples: leaving
%   the band ends it (a row driven again after that counts again) and two part-runs do not add up.
%   n       number of traversals counted, all rows together, = numel(t_done)
%   t_done  end time of each counted traversal (time of the last sample of its run), sorted, n x 1, s

t = t(:);
t_done = zeros(0, 1);
n = 0;
if isempty(t), return, end
if size(xy, 1) ~= numel(t) || size(xy, 2) ~= 2
    error('row_test_count_rows:size', 'xy must be numel(t) x 2 [x y], got %s for %d samples', ...
        mat2str(size(xy)), numel(t));
end
for r = 1:size(rows, 1)
    a = rows(r, 1:2);
    d = rows(r, 3:4) - a;
    L = hypot(d(1), d(2));
    u = d/L;                                               % unit vector along the row
    px = xy(:, 1) - a(1);
    py = xy(:, 2) - a(2);
    s = min(max(px*u(1) + py*u(2), 0), L);                 % position along the row, clipped to the segment
    near = hypot(px - s*u(1), py - s*u(2)) <= tol;         % within tol of the centre line
    e = diff([0; near; 0]);
    i0 = find(e == 1);                                     % first sample of each run
    i1 = find(e == -1) - 1;                                % last sample of each run
    for k = 1:numel(i0)
        sr = s(i0(k):i1(k));
        if max(sr) - min(sr) >= frac*L
            t_done(end + 1, 1) = t(i1(k)); %#ok<AGROW>
        end
    end
end
t_done = sort(t_done);
n = numel(t_done);
end
