function calls = build_all(scope, varargin)
%BUILD_ALL  Rebuild the generated models with their builders, dependencies first (owner: integration, spec 9).
%
%   build_all()                               = build_all('NoMCU')
%   build_all(scope)                          run every builder of the scope in the Build order below
%   calls = build_all(scope, 'DryRun', true)  only the calls: nothing is built and no model is loaded
%   calls   cellstr column, the calls in order, e.g. 'build_power_plant()', 'build_shaddad_mcu(''model'')'
%
%   scope   'NoMCU' (default)  every builder below except the MCU part
%           'MCU'              the MCU part
%           'All'              'NoMCU', then 'MCU', then build_shaddad_system()
%
%   Build order (plan docs/superpowers/plans/2026-10-29-model-layout-cleanup.md, section Build order): a
%   Subsystem Reference or Model file after the files it instantiates, a bench after the files it loads; each
%   builder keeps its own internal order and is called once, with no arguments unless listed.
%     NoMCU 1  build_drive, build_shaddad_bms (the four codecs, then Shaddad_BMS), build_rc_link (the six
%              subsystem files and the four tests/rc benches), build_sensor_models, build_navfusion,
%              build_perception, build_structure_env, build_sim_monitors, build_viz_logging,
%              build_shaddad_gnc (the T14 files and Shaddad_GNC)
%           2  build_power_pack, build_structure, build_electronics_plant, build_gnc_mission (the five T15
%              files and tb_gnc_mission, which loads Shaddad_GNC)
%           3  build_power_plant
%           4  build_shaddad_plant, build_shaddad_companion
%           5  the benches: build_tb_pack_bms, build_tb_power_plant, build_tb_drive, build_tb_structure,
%              build_tb_structure_env, build_tb_elec_plant, build_tb_sens_plant, build_tb_nav,
%              build_tb_perception, build_tb_gnc, build_tb_sim_monitors, build_tb_frames
%     MCU      build_can2_codecs, build_fw_platform with the 11 T08 units, build_fw_control,
%              build_shaddad_mcu('model'), build_tb_mcu_rc, build_tb_mcu_closed_loop. Never
%              build_shaddad_mcu() / ('all') or build_fw_platform() without unit names: they rebuild the
%              five T09 units as stubs (docs/STATUS.md 6, item 2).
%   'NoMCU' builds companion_io (build_electronics_plant) and tb_elec_plant on the existing CAN2 codecs and
%   Shaddad_MCU of the MCU part. Shaddad_GNC instantiates the five T15 files that build_gnc_mission rebuilds
%   after it (by name, with the T14 port contract), because tb_gnc_mission loads Shaddad_GNC.
%   Not called: build_shaddad_battery (its generated libraries are clean, and it rewrites the parameter
%   files), build_stubs and build_project (no models), build_targets (board builds).
%
%   A run closes every loaded model without saving (bdclose all) before the first builder, because
%   build_rc_link refuses to rebuild a file that a loaded model references. An 'All' run closes them again
%   just before the MCU part (build_can2_codecs): Shaddad_Companion (its companion_io) and other models that
%   the NoMCU builders leave loaded reference the CAN2 codecs that build_can2_codecs rebuilds. A run therefore
%   works only in a batch MATLAB (matlab -batch; build plan rule 1 forbids bdclose all in a shared session):
%   elsewhere it is the error build_all:batch. It stops at the first builder error: error build_all:builder,
%   message '<call> failed: <builder message>', with the builder's error as its cause. Each call and its time
%   are printed.

if nargin < 1 || isempty(scope), scope = 'NoMCU'; end
ip = inputParser;
ip.addParameter('DryRun', false, @(x) isscalar(x) && (islogical(x) || isnumeric(x)));
ip.parse(varargin{:});
scope = local_scope(scope);
steps = local_steps(scope);
calls = cellfun(@local_call, steps(:, 1), steps(:, 2), 'UniformOutput', false);
if ip.Results.DryRun, return, end
if ~batchStartupOptionUsed
    error('build_all:batch', ['build_all closes every loaded model without saving, so it runs only in a ' ...
        'batch MATLAB (matlab -batch); build_all(scope, ''DryRun'', true) lists the calls']);
end

%% ---- run the builders ----
bdclose('all');
n = numel(calls);
t0 = tic;
for k = 1:n
    if strcmp(scope, 'All') && strcmp(steps{k, 1}, 'build_can2_codecs')
        bdclose('all');   % NoMCU models left loaded (Shaddad_Companion: companion_io) reference the CAN2 codecs
        fprintf('build_all All: every loaded model closed without saving before the MCU part\n');
    end
    fprintf('build_all %s %d/%d: %s\n', scope, k, n, calls{k});
    t1 = tic;
    try
        feval(steps{k, 1}, steps{k, 2}{:});
    catch tmp_ba_err
        err = MException('build_all:builder', '%s failed: %s', calls{k}, tmp_ba_err.message);
        throw(addCause(err, tmp_ba_err));
    end
    fprintf('build_all %s %d/%d: %s done in %.1f s\n', scope, k, n, calls{k}, toc(t1));
end
fprintf('build_all %s: %d builders in %.1f min\n', scope, n, toc(t0)/60);
end

% ===========================================================================================
function s = local_scope(s)
% the scope name, case-insensitive
scopes = {'NoMCU', 'MCU', 'All'};
if isstring(s) && isscalar(s), s = char(s); end
k = [];
if ischar(s), k = find(strcmpi(s, scopes)); end
if isempty(k)
    error('build_all:scope', 'scope must be ''NoMCU'', ''MCU'' or ''All''');
end
s = scopes{k};
end

function S = local_steps(scope)
% {builder, arguments} of each call of the scope, in the Build order (rows)
T08 = {'fw_sbus_decoder', 'fw_rc_mapping', 'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', 'fw_adc_scaling', ...
    'fw_telemetry', 'fw_io_in_sim', 'fw_io_in_hw', 'fw_io_out_sim', 'fw_io_out_hw'};
noMcu = [ ...
    {'build_drive'; 'build_shaddad_bms'; 'build_rc_link'; 'build_sensor_models'; 'build_navfusion'; ...
     'build_perception'; 'build_structure_env'; 'build_sim_monitors'; 'build_viz_logging'; 'build_shaddad_gnc'}
    {'build_power_pack'; 'build_structure'; 'build_electronics_plant'; 'build_gnc_mission'}
    {'build_power_plant'}
    {'build_shaddad_plant'; 'build_shaddad_companion'}
    {'build_tb_pack_bms'; 'build_tb_power_plant'; 'build_tb_drive'; 'build_tb_structure'; ...
     'build_tb_structure_env'; 'build_tb_elec_plant'; 'build_tb_sens_plant'; 'build_tb_nav'; ...
     'build_tb_perception'; 'build_tb_gnc'; 'build_tb_sim_monitors'; 'build_tb_frames'}];
noMcu(:, 2) = {{}};
mcu = {'build_can2_codecs', {}; 'build_fw_platform', {T08}; 'build_fw_control', {}; ...
    'build_shaddad_mcu', {'model'}; 'build_tb_mcu_rc', {}; 'build_tb_mcu_closed_loop', {}};
switch scope
    case 'NoMCU'
        S = noMcu;
    case 'MCU'
        S = mcu;
    otherwise
        S = [noMcu; mcu; {'build_shaddad_system', {}}];
end
end

function c = local_call(name, args)
% the call as text: name(arg, ...), a char argument quoted, a cellstr as {'a', 'b'}
a = cellfun(@local_arg, args, 'UniformOutput', false);
c = sprintf('%s(%s)', name, strjoin(a, ', '));
end

function s = local_arg(v)
if ischar(v)
    s = ['''' strrep(v, '''', '''''') ''''];
else
    s = ['{' strjoin(cellfun(@local_arg, v, 'UniformOutput', false), ', ') '}'];
end
end
