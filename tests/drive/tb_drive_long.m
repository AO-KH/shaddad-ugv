function R = tb_drive_long(caseId, opt)
%TB_DRIVE_LONG  Long T06 bench scenarios (drive spec 8 T5, T8) with their measured values (owner drive).
%   R = tb_drive_long(caseId)        caseId 'T5a' | 'T5b' | 'T8'
%   R = tb_drive_long(caseId, opt)   opt.chunk (s, 900): chunk length of tb_drive_run_chunks (operating-point
%                                    continuation, bit-exact); opt.mode ('accelerator'); opt.worker (false):
%                                    true when called on a parallel pool worker (opens the project, private
%                                    Simulink cache folder so the shared session's slprj is never touched);
%                                    T5 only: opt.t_stop (s) ends the call early with a partial R (R.partial,
%                                    R.op, R.out) and opt.resume = that R continues it, so a 2 h run can be
%                                    spread over several bounded MATLAB calls with identical results
%   Used by t_drive (T06_LONG=1) and, for unattended runs that must not block the shared MATLAB session,
%   through parfeval on a process pool:  f = parfeval(pool, @tb_drive_long, 1, 'T5a', struct('worker', true))
%
%   T5a  dyno 3000 rpm, TORQUE 8 N m, 40 degC, no solar, 7200 s: R.Tend_C, R.t63_min
%   T5b  same at 50 degC + dT_solar: R.Tmax_C, R.derating (status b3 after 60 s), R.derate_t_min, R.t140_min,
%        R.tau_avail_min
%   T8   brake hold on 20 deg, drive disabled at 1.5 s, 605 s: R.dt_ms, R.creep_mph, R.wmax, R.hold_s
if nargin < 2, opt = struct(); end
d = struct('chunk', 900, 'mode', 'accelerator', 'worker', false, 't_stop', inf, 'resume', []);
fn = fieldnames(opt);
for k = 1:numel(fn), d.(fn{k}) = opt.(fn{k}); end
opt = d;
if opt.worker, local_worker_setup(); end
P = evalin('base', 'P');
mp = {'SimulationMode', opt.mode};
t0 = tic;
switch caseId
    case {'T5a', 'T5b'}
        w_m = 3000*pi/30; N = P.drive.gear.N; Pm = P;
        if strcmp(caseId, 'T5a')
            Pm.drive.thermal.dT_solar = 0; Pm.drive.thermal.T_amb0 = 313.15;
        else
            Pm.drive.thermal.T_amb0 = 323.15;
        end
        Tamb = Pm.drive.thermal.T_amb0;
        [c, ~] = tb_drive_profile(struct('t_end', 7200, 'Ts', P.drive.Ts_can, 'mode', 2, 'Kff', 0, 'tau_ff_tab', [0 0; 0.3 8]));
        tw = (0:0.001:2)'; wd = timeseries(w_m/N*min(max((tw - 0.2)/1.0, 0), 1), tw);
        mw = struct('tb_cmd', c, 'tb_load', 2, 'tb_w_dyno', wd, 'tb_T_amb', Tamb, 'tb_dec', 1000);
        t1 = min(opt.t_stop, 7200);
        if isempty(opt.resume)
            [out, op] = tb_drive_run_chunks('tb_drive', t1, opt.chunk, mw, struct('P', Pm), mp);
        else
            [out, op] = tb_drive_run_chunks('tb_drive', t1, opt.chunk, mw, struct('P', Pm), mp, opt.resume.op, opt.resume.out);
        end
        if t1 < 7200
            % partial run (bounded MATLAB call): continue with tb_drive_long(caseId, struct('resume', R, ...))
            R = struct('caseId', caseId, 'partial', true, 't', t1, 'op', op, 'out', out, 'wall', toc(t0), 'mode', opt.mode);
            if ~isempty(opt.resume), R.wall = R.wall + opt.resume.wall; end
            return
        end
        T = out.truth.T_winding.Data; t = out.truth.T_winding.Time;
        R.Tend_C = T(end) - 273.15;
        R.Tmax_C = max(T) - 273.15;
        R.t63_min = t(find(T - Tamb >= 0.632*(T(end) - Tamb), 1))/60;
        % DERATING (status b3) after the power-up transient (the DC link charging sets f_uv < 1 for a few ms)
        st = out.fb.status; hot = st.Time > 60;
        der = bitand(st.Data(hot), uint16(8)) ~= 0; ts = st.Time(hot);
        R.derating = any(der);
        if R.derating, R.derate_t_min = ts(find(der, 1))/60; else, R.derate_t_min = NaN; end
        i140 = find(T >= 413.15, 1);
        if isempty(i140), R.t140_min = NaN; else, R.t140_min = t(i140)/60; end
        R.tau_avail_min = min(double(out.fb.tau_avail.Data(out.fb.tau_avail.Time > 60)));
        R.tau_m_mean = mean(out.truth.tau_m.Data(out.truth.tau_m.Time > 60));
        R.T_amb_C = Tamb - 273.15; R.dT_solar = Pm.drive.thermal.dT_solar;
        R.T_esc_max_C = max(out.truth.T_esc.Data) - 273.15;
        R.fault_code_end = out.fb.fault_code.Data(end);
        R.tw = t; R.Tw = T;
    case 'T8'
        b = P.drive.bench;
        g = -b.m_side*b.g*sind(20)*P.drive.gear.r_spr;
        [c, ~] = tb_drive_profile(struct('t_end', 605, 'enable', [0 1; 1.5 0], 'v_tgt', [0 0]));
        so = tb_drive_run('tb_drive', 605, struct('tb_cmd', c, 'tb_Trr', 1e-6, 'tb_grade', timeseries(g, 0), 'tb_dec', 10), ...
            struct(), mp);
        tr = so.truth; fb = so.fb;
        tb = fb.brake_released.Time; br = fb.brake_released.Data;
        t_cmd = tb(find(tb > 1.5 & ~br, 1));
        tf = tr.f_brake_rel.Time; f = tr.f_brake_rel.Data;
        t_eng = tf(find(tf > t_cmd & f <= 1e-9, 1));
        R.dt_ms = 1e3*(t_eng - t_cmd);
        h = tf >= t_eng + 1;
        th = tr.theta_spr.Data(h); tt = tf(h);
        R.hold_s = tt(end) - tt(1);
        R.creep_mph = abs(th(end) - th(1))*P.drive.gear.r_spr/R.hold_s*3600;
        R.wmax = max(abs(tr.w_m.Data(h)));
        R.state_end = double(fb.state.Data(end));
    otherwise
        error('tb_drive_long:case', 'Unknown case %s', caseId);
end
R.caseId = caseId; R.wall = toc(t0); R.mode = opt.mode; R.partial = false;
if ~isempty(opt.resume), R.wall = R.wall + opt.resume.wall; end
end

function local_worker_setup()
% parallel pool worker: open the project (path, P, buses), private cache folder, bench models loaded
if isempty(matlab.project.rootProject)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    openProject(root);
end
if ~evalin('base', 'exist(''P'',''var'') && exist(''Bus_DriveSideCmd'',''var'')'), shaddad_params(); end
if ~evalin('base', 'exist(''cs_SimFixed'',''var'')'), shaddad_configsets('fixed'); end
cf = fullfile(tempdir, sprintf('t06_worker_%d', matlabProcessID));
Simulink.fileGenControl('set', 'CacheFolder', cf, 'CodeGenFolder', cf, 'createDir', true);
end
