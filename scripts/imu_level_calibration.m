function cal = imu_level_calibration(file, varargin)
%IMU_LEVEL_CALIBRATION  Level calibration of the MCU tilt estimate (commissioning, D47).
%
%   cal = imu_level_calibration()               newest work/results/S0_*.mat (power-up at standstill on level ground)
%   cal = imu_level_calibration(file)           a log of the robot standing still on level ground
%   cal = imu_level_calibration(..., 'Write', true)   also write the result into params/param_electronics.m
%
%   The installed IMU has a static attitude error (accelerometer bias and temperature bias, mounting
%   misalignment); on the real robot it is measured once on level ground at commissioning. fw_imu_tilt
%   subtracts P.electronics.imu_cal.roll0 / pitch0 from its roll / pitch, so the MCU reads 0 with the
%   robot standing on level ground. This function takes the mean MCU roll / pitch over the standstill
%   part of the log (t >= 4 s, |v| < 0.01 m/s) and adds the calibration that was active in that run, so
%   it gives the raw offset of the IMU. Benches with an ideal IMU (tb_mcu_rc, tb_mcu_closed_loop, the
%   tilt harness) set imu_cal to zero.
%   cal: struct roll0, pitch0 [rad], n (samples), file.

ip = inputParser;
ip.addParameter('Write', false);
ip.parse(varargin{:});
root = char(currentProject().RootFolder);
if nargin < 1 || isempty(file)
    L = dir(fullfile(root, 'work', 'results', 'S0_*.mat'));
    [~, i] = max([L.datenum]);
    file = fullfile(L(i).folder, L(i).name);
end
S = load(file).S;
t = double(S.telem.pitch.t(:));
v = interp1(double(S.truth.speed_fwd.t(:)), abs(double(S.truth.speed_fwd.v(:, 1))), t, 'previous', 'extrap');
m = t >= 4 & v < 0.01;
assert(nnz(m) > 100, 'imu_level_calibration:data', 'no standstill samples after 4 s in %s', file);
P = shaddad_params('Assign', false);
old = struct('roll0', 0, 'pitch0', 0);
if isfield(P.electronics, 'imu_cal'), old = P.electronics.imu_cal; end
cal.roll0 = mean(double(S.telem.roll.v(m))) + old.roll0;
cal.pitch0 = mean(double(S.telem.pitch.v(m))) + old.pitch0;
cal.n = nnz(m);
cal.file = file;
fprintf('IMU level calibration from %s (%d samples): roll0 %.5f rad (%.2f deg), pitch0 %.5f rad (%.2f deg)\n', ...
    file, cal.n, cal.roll0, rad2deg(cal.roll0), cal.pitch0, rad2deg(cal.pitch0));
if ip.Results.Write
    f = fullfile(root, 'params', 'param_electronics.m');
    txt = fileread(f);
    [~, nm] = fileparts(file);
    line = sprintf('E.imu_cal = struct(''roll0'', %.5f, ''pitch0'', %.5f);   %% rad, imu_level_calibration(%s)', ...
        cal.roll0, cal.pitch0, nm);
    pat = 'E\.imu_cal = struct\([^\n]*';
    assert(~isempty(regexp(txt, pat, 'once')), 'imu_level_calibration:param', 'no E.imu_cal line in %s', f);
    txt = regexprep(txt, pat, regexptranslate('escape', line), 'once');
    fid = fopen(f, 'w'); fwrite(fid, txt, 'char'); fclose(fid);
    fprintf('written to params/param_electronics.m\n');
end
end
