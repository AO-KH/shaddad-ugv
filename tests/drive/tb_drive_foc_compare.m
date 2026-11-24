function R = tb_drive_foc_compare(P, t_end)
%TB_DRIVE_FOC_COMPARE  T15: SYS (tb_drive) vs FOC (tb_drive_foc, bench-only, D52) equivalence (T06).
%   R = tb_drive_foc_compare(P)          T1 and T3 profiles, 10 s each (drive spec 8 T15)
%   R = tb_drive_foc_compare(P, t_end)   shorter runs (development)
%   Both benches get the same DRV_CMD frames, DC source and load. Metrics per profile, worst case reported:
%     speed_rms_pct  RMS(v_foc - v_sys) over [0, t_end] in % of max |v_sys| (track speed, m/s)
%     energy_pct     |E_dc,foc - E_dc,sys| / |E_dc,sys| * 100, E_dc = int p_dc over [t_a, t_end]
%     idc_peak_pct   |max i_dc,foc - max i_dc,sys| / max i_dc,sys * 100 over [t_a, t_end]
%   t_a = 0.5 s skips the identical DC-link charging transient at t = 0 (same C_link / R_esr in both plants).
%   R.wall_foc: wall-clock of the 10 s FOC runs (T17: FOC 10 s scenario < 10 min).
if nargin < 2, t_end = 10; end
t_a = 0.5;
r = P.drive.gear.r_spr; b = P.drive.bench;
g = -b.m_side*b.g*sind(20)*r;                                   % 20 deg climb at the sprocket
Trr = 0.08*b.m_side*b.g*cosd(20)*r;
tg = (0:0.001:t_end)'; gv = g*min(max(tg - 3, 0), 1);
[c, ~] = tb_drive_profile(struct('t_end', t_end, 'v_tgt', [0 0; 1 1.0]));
cases = {'T1 flat', struct('tb_cmd', c); ...
         'T3 20 deg climb', struct('tb_cmd', c, 'tb_Trr', Trr, 'tb_grade', timeseries(gv, tg))};
R.speed_rms_pct = 0; R.energy_pct = 0; R.idc_peak_pct = 0; R.wall_foc = 0; R.note = '';
for k = 1:size(cases, 1)
    s1 = tb_drive_run('tb_drive', t_end, cases{k, 2});
    [s2, w2] = tb_drive_run('tb_drive_foc', t_end, cases{k, 2});
    R.wall_foc = max(R.wall_foc, w2);
    a = s1.truth; f = s2.truth;
    t = a.w_spr.Time; v1 = a.w_spr.Data*r;
    v2 = interp1(f.w_spr.Time, f.w_spr.Data*r, t, 'linear', 'extrap');
    sp = 100*sqrt(mean((v2 - v1).^2))/max(abs(v1));
    m1 = a.p_dc.Time >= t_a; m2 = f.p_dc.Time >= t_a;
    E1 = trapz(a.p_dc.Time(m1), a.p_dc.Data(m1)); E2 = trapz(f.p_dc.Time(m2), f.p_dc.Data(m2));
    en = 100*abs(E2 - E1)/abs(E1);
    i1 = max(a.i_dc.Data(a.i_dc.Time >= t_a)); i2 = max(f.i_dc.Data(f.i_dc.Time >= t_a));
    ip = 100*abs(i2 - i1)/abs(i1);
    R.speed_rms_pct = max(R.speed_rms_pct, sp); R.energy_pct = max(R.energy_pct, en);
    R.idc_peak_pct = max(R.idc_peak_pct, ip);
    R.cases(k) = struct('name', cases{k, 1}, 'speed_rms_pct', sp, 'energy_pct', en, 'idc_peak_pct', ip, ...
        'E_sys', E1, 'E_foc', E2, 'idc_sys', i1, 'idc_foc', i2, 'wall_foc', w2); %#ok<AGROW>
    R.note = [R.note sprintf('%s: v %.2f %%, E %.0f/%.0f J, i_dc %.1f/%.1f A, FOC wall %.0f s; ', ...
        cases{k, 1}, sp, E1, E2, i1, i2, w2)];
end
R.note = strtrim(R.note);
end
