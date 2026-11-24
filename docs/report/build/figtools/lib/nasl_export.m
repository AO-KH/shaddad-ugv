function nasl_export(f, name)
%NASL_EXPORT  exportgraphics at 200 dpi into docs/report/figs and close the figure.
figs = 'C:\Users\aomar\Desktop\Robot project\docs\report\figs';
exportgraphics(f, fullfile(figs, name), 'Resolution', 200, 'BackgroundColor', 'white');
close(f);
fprintf('exported %s\n', name);
end
