function nasl_mode_ax(ax, used)
%NASL_MODE_AX  y axis with ShaddadMode names (only the codes in USED, default all).
modes = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', 'CTRL_STOP', 'ESTOP', 'FAULT'};
if nargin < 2 || isempty(used), used = 0:9; end
used = unique(used(:)');
set(ax, 'YTick', used, 'YTickLabel', modes(used + 1), 'TickLabelInterpreter', 'none');
ylim(ax, [min(used) - 0.5, max(used) + 0.5]);
end
