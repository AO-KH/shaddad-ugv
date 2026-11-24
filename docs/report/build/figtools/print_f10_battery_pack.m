% print_f10_battery_pack.m - re-print of Pack_Grouped16 (Battery Builder module) for the report figure
% F10_battery_pack.png. The Probe block is widened in memory only so that its three port labels
% (batteryTemperature, socParallelAssembly, vParallelAssembly) do not overlap the block icon; the model
% is closed without saving, so subsystems/power/Pack_Grouped16.slx is unchanged. (fix round 2, 2026-10-23)
out = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'figs', 'F10_battery_pack.png');
assert(~bdIsLoaded('Pack_Grouped16'), 'close Pack_Grouped16 first (it is edited in memory)');
load_system('Pack_Grouped16');
set_param('Pack_Grouped16/Probe', 'Position', [-110 40 90 150]);
print('-sPack_Grouped16', '-dpng', '-r200', out);
close_system('Pack_Grouped16', 0);
