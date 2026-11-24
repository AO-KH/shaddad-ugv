function dbox(ax, x, y, w, h, ttl, sub, varargin)
%DBOX  Rounded box with a bold title and optional sub lines (cellstr).
%   dbox(ax, x, y, w, h, title, {lines}, 'Fill', rgb, 'Edge', rgb, 'Text', rgb, 'Radius', r, 'Dashed', tf,
%        'TitleSize', pt, 'SubSize', pt, 'Align', 'center'|'left', 'TitleTop', tf, 'LineWidth', lw)
c = nasl_pal();
ip = inputParser;
ip.addParameter('Fill', c.lgrey); ip.addParameter('Edge', c.navy); ip.addParameter('Text', c.navy);
ip.addParameter('SubText', c.dgrey); ip.addParameter('Radius', 1.6); ip.addParameter('Dashed', false);
ip.addParameter('TitleSize', 8); ip.addParameter('SubSize', 6.5); ip.addParameter('Align', 'center');
ip.addParameter('TitleTop', false); ip.addParameter('LineWidth', 0.9); ip.addParameter('Gap', 1.0);
ip.parse(varargin{:}); o = ip.Results;
r = o.Radius;
cv = [min(1, 2*r/w), min(1, 2*r/h)];
ls = '-'; if o.Dashed, ls = '--'; end
rectangle(ax, 'Position', [x y w h], 'Curvature', cv, 'FaceColor', o.Fill, 'EdgeColor', o.Edge, 'LineWidth', o.LineWidth, 'LineStyle', ls);
if nargin < 7 || isempty(sub), sub = {}; end
if ischar(sub), sub = {sub}; end
n = numel(sub);
% vertical layout in points -> data units: estimate from the axes scale
pt = local_pt2data(ax);
lt = o.TitleSize*1.25*pt; ls_ = o.SubSize*1.25*pt;
tot = lt + n*ls_ + (n > 0)*o.Gap*pt;
if o.TitleTop
    y0 = y + h - 0.8 - lt/2;
else
    y0 = y + h/2 + tot/2 - lt/2;
end
if strcmp(o.Align, 'left'), xa = x + 1.2; ha = 'left'; else, xa = x + w/2; ha = 'center'; end
if ~isempty(ttl)
    text(ax, xa, y0, ttl, 'HorizontalAlignment', ha, 'VerticalAlignment', 'middle', 'FontName', 'Arial', ...
        'FontSize', o.TitleSize, 'FontWeight', 'bold', 'Color', o.Text, 'Interpreter', 'none');
else
    y0 = y + h/2 + n*ls_/2 + lt/2 + o.Gap*pt;
end
for k = 1:n
    yk = y0 - lt/2 - o.Gap*pt - (k - 0.5)*ls_;
    text(ax, xa, yk, sub{k}, 'HorizontalAlignment', ha, 'VerticalAlignment', 'middle', 'FontName', 'Arial', ...
        'FontSize', o.SubSize, 'Color', o.SubText, 'Interpreter', 'none');
end
end

function s = local_pt2data(ax)
% data units per typographic point
f = ancestor(ax, 'figure');
u = f.Units; f.Units = 'points'; fp = f.Position; f.Units = u;
s = diff(ax.XLim)/(fp(3)*ax.Position(3));
end
