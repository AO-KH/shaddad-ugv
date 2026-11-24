function [f, ax] = dcanvas(W, H, w_cm)
%DCANVAS  White drawing canvas: axes in data units [0 W] x [0 H], equal aspect, figure w_cm wide.
h_cm = w_cm*H/W;
f = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 w_cm h_cm], 'Visible', 'off');
ax = axes(f, 'Units', 'normalized', 'Position', [0 0 1 1]);
hold(ax, 'on'); axis(ax, 'off');
set(ax, 'XLim', [0 W], 'YLim', [0 H], 'DataAspectRatio', [1 1 1]);
end
