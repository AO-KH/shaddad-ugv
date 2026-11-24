function rows = row_test_rows(R)
%ROW_TEST_ROWS  Centre lines of the row test field (spec docs/superpowers/specs/2026-10-27-rc-row-test-design.md, section 5).
%   rows = row_test_rows(R)     R = param_rowtest() or P.scen.rowtest
%   rows : R.n_rows x 5 [x1 y1 x2 y2 laneId] in m (x east, y north). Row i runs east from (0, (i-1)*spacing)
%          to (L_row, (i-1)*spacing); laneId = i. The headlands and the world margin lie outside these lines.

n = R.n_rows;
k = (1:n)';
y = (k - 1)*R.spacing;
rows = [zeros(n, 1), y, R.L_row*ones(n, 1), y, k];
end
