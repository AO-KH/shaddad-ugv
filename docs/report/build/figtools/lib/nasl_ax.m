function nasl_ax(ax, ttl, yl, xl)
%NASL_AX  Common axes styling.
c = nasl_pal();
grid(ax, 'on'); box(ax, 'on');
set(ax, 'FontName', 'Arial', 'FontSize', 8, 'GridAlpha', 0.18, 'XColor', c.dgrey, 'YColor', c.dgrey, ...
    'TickLabelInterpreter', 'none', 'LineWidth', 0.6);
if nargin > 1 && ~isempty(ttl), title(ax, ttl, 'FontSize', 9, 'FontWeight', 'bold', 'Color', c.navy, 'Interpreter', 'none'); end
if nargin > 2 && ~isempty(yl), ylabel(ax, yl, 'Interpreter', 'none'); end
if nargin > 3 && ~isempty(xl), xlabel(ax, xl, 'Interpreter', 'none'); end
end
