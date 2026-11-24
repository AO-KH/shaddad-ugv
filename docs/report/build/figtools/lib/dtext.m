function dtext(ax, x, y, s, varargin)
%DTEXT  Arial text in data units (Interpreter none). Extra name/value pairs go to text().
c = nasl_pal();
text(ax, x, y, s, 'FontName', 'Arial', 'FontSize', 6.5, 'Color', c.dgrey, 'Interpreter', 'none', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', varargin{:});
end
