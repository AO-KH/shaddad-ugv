function out = tb_drive_results(op, varargin)
%TB_DRIVE_RESULTS  Collect the measured numbers of t_drive (T06) across test methods.
%   tb_drive_results('reset')
%   tb_drive_results('add', id, criterion, measured, result)   result: 'pass' | 'fail' | 'blocked' | 'conflict'
%   T = tb_drive_results('table')
%   tb_drive_results('save', file)     MAT file with table T (+ printed summary)
persistent R
if isempty(R), R = struct('id', {}, 'criterion', {}, 'measured', {}, 'result', {}); end
out = [];
switch op
    case 'reset'
        R = struct('id', {}, 'criterion', {}, 'measured', {}, 'result', {});
    case 'add'
        R(end + 1) = struct('id', varargin{1}, 'criterion', varargin{2}, 'measured', varargin{3}, 'result', varargin{4});
        fprintf('  [%s] %-8s %s | measured: %s\n', upper(varargin{4}), varargin{1}, varargin{2}, varargin{3});
    case 'table'
        out = struct2table(R, 'AsArray', true);
    case 'save'
        T = struct2table(R, 'AsArray', true); %#ok<NASGU>
        save(varargin{1}, 'T');
        fprintf('\n==== t_drive measured values (%d rows) ====\n', numel(R));
        for k = 1:numel(R)
            fprintf('%-9s %-9s %s | %s\n', R(k).id, upper(R(k).result), R(k).criterion, R(k).measured);
        end
end
end
