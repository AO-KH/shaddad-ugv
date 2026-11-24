function darrow(ax, P, varargin)
%DARROW  Polyline arrow with a filled head at the last point (data units).
%   darrow(ax, [x y; ...], 'Color', rgb, 'Width', lw, 'Style', '-', 'Head', len, 'Both', tf, 'Label', txt,
%          'LabelPos', [x y], 'LabelSize', pt, 'LabelAlign', 'center', 'LabelColor', rgb)
c = nasl_pal();
ip = inputParser;
ip.addParameter('Color', c.navy); ip.addParameter('Width', 1.0); ip.addParameter('Style', '-');
ip.addParameter('Head', 1.6); ip.addParameter('Both', false); ip.addParameter('NoHead', false);
ip.addParameter('Label', ''); ip.addParameter('LabelPos', []); ip.addParameter('LabelSize', 6.3);
ip.addParameter('LabelAlign', 'center'); ip.addParameter('LabelColor', []); ip.addParameter('LabelRot', 0);
ip.parse(varargin{:}); o = ip.Results;
hl = o.Head;
Q = P;
if ~o.NoHead
    d = P(end, :) - P(end-1, :); Q(end, :) = P(end, :) - hl*0.85*d/norm(d);
    if o.Both, d0 = P(1, :) - P(2, :); Q(1, :) = P(1, :) - hl*0.85*d0/norm(d0); end
end
plot(ax, Q(:, 1), Q(:, 2), o.Style, 'Color', o.Color, 'LineWidth', o.Width);
if ~o.NoHead
    local_head(ax, P(end-1, :), P(end, :), hl, o.Color);
    if o.Both, local_head(ax, P(2, :), P(1, :), hl, o.Color); end
end
if ~isempty(o.Label)
    lp = o.LabelPos; if isempty(lp), lp = mean(P([1 end], :), 1); end
    lc = o.LabelColor; if isempty(lc), lc = o.Color; end
    text(ax, lp(1), lp(2), o.Label, 'HorizontalAlignment', o.LabelAlign, 'VerticalAlignment', 'middle', ...
        'FontName', 'Arial', 'FontSize', o.LabelSize, 'Color', lc, 'Interpreter', 'none', 'Rotation', o.LabelRot);
end
end

function local_head(ax, a, b, hl, col)
d = (b - a)/norm(b - a); n = [-d(2) d(1)];
w = hl*0.45;
p = [b; b - hl*d + w*n; b - hl*d - w*n];
patch(ax, p(:, 1), p(:, 2), col, 'EdgeColor', 'none');
end
