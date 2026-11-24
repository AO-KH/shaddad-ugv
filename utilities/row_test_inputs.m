function [ok, msg, cfg, est_min] = row_test_inputs(cfg, R)
%ROW_TEST_INPUTS  Check the five inputs of an RC row test against their limits and estimate its run time (spec sections 4
%   and 8, RC row test).
%   [ok, msg, cfg] = row_test_inputs(cfg, R)
%   [ok, msg, cfg, est_min] = row_test_inputs(cfg, R)
%     cfg   param_rowtest() with the five inputs set: n_rows, minutes, T_amb_C, sprayer, fast (no other field is read)
%     R     param_rowtest(): the limits n_rows_range, minutes_range, T_amb_C_range and the run-time figures rtf, t_compile
%   ok       true when every input is inside its limits: n_rows a whole number in n_rows_range (at least 2: with one row
%            the return to row 1 would need a loop turn), minutes in minutes_range, T_amb_C in T_amb_C_range (degC),
%            sprayer and fast true or false (logical, or the numbers 0 and 1)
%   msg      '' when ok; else one sentence on the first input outside its limits, by the name the user gives it (Rows,
%            Minutes, Ambient, Sprayer, Fast: the arguments of phase1_row_test and the fields of its window) with the range
%   cfg      the inputs as numbers (D58): n_rows, minutes and T_amb_C double, sprayer and fast double 0 / 1; unchanged
%            when not ok
%   est_min  estimated wall-clock time of the run, min: R.t_compile/60 + minutes/R.rtf(k), k = 1 full detail, 2 full
%            detail + implement (sprayer), 3 fast, 4 fast + implement; NaN when not ok

est_min = NaN;
msg = local_range(cfg.n_rows, 'Rows', R.n_rows_range, true, '');
if isempty(msg), msg = local_range(cfg.minutes, 'Minutes', R.minutes_range, false, ' min'); end
if isempty(msg), msg = local_range(cfg.T_amb_C, 'Ambient', R.T_amb_C_range, false, ' degC'); end
if isempty(msg), msg = local_flag(cfg.sprayer, 'Sprayer'); end
if isempty(msg), msg = local_flag(cfg.fast, 'Fast'); end
ok = isempty(msg);
if ~ok, return, end

cfg.n_rows = double(cfg.n_rows);
cfg.minutes = double(cfg.minutes);
cfg.T_amb_C = double(cfg.T_amb_C);
cfg.sprayer = double(cfg.sprayer ~= 0);
cfg.fast = double(cfg.fast ~= 0);
est_min = R.t_compile/60 + cfg.minutes/R.rtf(1 + cfg.sprayer + 2*cfg.fast);
end

% -------------------------------------------------------------------------------------------
function msg = local_range(x, name, lim, whole, unit)
% '' when x is a real number inside lim (a whole number when whole), else the sentence that says what it must be
msg = '';
good = isnumeric(x) && isscalar(x) && isreal(x) && x >= lim(1) && x <= lim(2);   % NaN fails the comparisons
if good && whole, good = x == round(x); end
if good, return, end
kind = 'a number';
if whole, kind = 'a whole number'; end
msg = sprintf('%s must be %s from %g to %g%s (got %s).', name, kind, lim(1), lim(2), unit, local_show(x));
end

function msg = local_flag(x, name)
% '' when x is true / false or 0 / 1, else the sentence that says so
msg = '';
if (islogical(x) || isnumeric(x)) && isscalar(x) && (x == 0 || x == 1), return, end
msg = sprintf('%s must be true or false (got %s).', name, local_show(x));
end

function s = local_show(x)
% the value as text for the message: small numeric, logical or char arrays as MATLAB writes them, else size and class
if (isnumeric(x) || islogical(x) || ischar(x)) && ismatrix(x) && numel(x) <= 10
    s = mat2str(x);
else
    s = sprintf('%s %s', strjoin(string(size(x)), 'x'), class(x));
end
end
