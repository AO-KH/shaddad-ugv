function R = tb_drive_2mass_lti(P, k_x, m_side, eta)
%TB_DRIVE_2MASS_LTI  Linear model of the ESC speed loop on the two-mass bench tb_drive_2mass (D50, T21).
%   R = tb_drive_2mass_lti(P, k_x, m_side)        eta (gear efficiency in the small-signal model) = P.drive.gear.eta
%   R = tb_drive_2mass_lti(P, k_x, m_side, eta)
%   Discrete loop at Ts_esc, exactly as built in drive_esc: Backward-Euler PI C(z) = Kp + Ki Ts z/(z-1),
%   1-sample command Unit Delay (D36), ZOH plant (Motor & Drive torque lag Tc, belt-side inertia J_b at the
%   motor incl. motor viscous losses, shear spring k = k_x r^2/N^2 and damper c = tau_d k to the vehicle
%   inertia m_side r^2/N^2), speed estimate F(z) = LPF(z) (1 - z^-4)/(4 Ts) with LPF(z) = (1-a)/(z-a).
%   R: .L (loop, motor-angle path), .S, .T, .Ms, .w_Ms, .zeta_min, .wn_zeta, .poles, .PM, .GM_dB,
%      .Tr (closed loop w_ref -> w_meas), .Tv (w_ref -> vehicle speed at the motor), .Pc (continuous plant)
if nargin < 4, eta = P.drive.gear.eta; end
Ts = P.drive.Ts_esc; N = P.drive.gear.N; r = P.drive.gear.r_spr;
e = P.drive.esc; mo = P.drive.motor;
J1 = mo.J + (P.drive.bench2m.J_b*N^2 - mo.J*N^2)/(eta*N^2);   % belt side at the motor (gear output side / eta)
J2 = m_side*r^2/(eta*N^2);
k = k_x*r^2/(eta*N^2); c = P.drive.bench2m.tau_d*k;
b1 = mo.lam + P.drive.gear.c_nl;
% states x = [th1; w1; th2; w2], input T_e (motor electromagnetic torque)
A = [0 1 0 0; -k/J1 -(b1 + c)/J1 k/J1 c/J1; 0 0 0 1; k/J2 c/J2 -k/J2 -c/J2];
B = [0; 1/J1; 0; 0];
Cm = [1 0 0 0; 0 0 0 1];               % outputs: motor angle, vehicle speed (at the motor)
G = ss(A, B, Cm, 0);
lag = tf(1, [mo.Tc 1]);
Pc = G*lag;
Pd = c2d(Pc, Ts, 'zoh');
z = tf('z', Ts);
C = e.Kp_w + e.Ki_w*Ts*z/(z - 1);
D1 = z^-1;
a = e.a_wf;
F = (1 - a)/(z - a)*(1 - z^-e.n_diff)/(e.n_diff*Ts);
Pth = tf(Pd(1, 1)); Pv = tf(Pd(2, 1));
PthF = minreal(Pth*F, 1e-6);                              % cancels the angle integrator (z = 1) with the difference
L = minreal(C*D1*PthF, 1e-6);
S = feedback(1, L);
T = feedback(L, 1);
Tr = minreal(feedback(C*D1*Pth, F)*F, 1e-9);             % w_ref -> w_meas
Tv = minreal(C*D1*Pv/(1 + C*D1*Pth*F), 1e-9);             % w_ref -> vehicle speed (at the motor)
w = logspace(-1, log10(pi/Ts), 4000);
Sm = squeeze(abs(freqresp(S, w)));
[R.Ms, i] = max(Sm); R.w_Ms = w(i);
pcl = pole(feedback(C*D1*Pth, F));
% exclude the unobservable motor-angle integrator (z = 1) and pure delays (z = 0); modes up to pi/(3 Ts)
pm = pcl(abs(pcl) > 1e-6 & abs(pcl - 1) > 1e-4);
s = log(pm)/Ts;
R.stable = all(abs(pm) < 1);
cx = abs(imag(s)) > 1e-6 & abs(s) < pi/(3*Ts);
zz = -real(s(cx))./abs(s(cx));
if isempty(zz), R.zeta_min = 1; R.wn_zeta = NaN; else, [R.zeta_min, j] = min(zz); sc = s(cx); R.wn_zeta = abs(sc(j)); end
ws = warning('off', 'all');                               % margin's closed-loop check sees the z = 1 angle mode
[GM, PM] = margin(L);
warning(ws);
R.GM_dB = 20*log10(GM); R.PM = PM;
R.L = L; R.S = S; R.T = T; R.Tr = Tr; R.Tv = Tv; R.Pc = Pc; R.poles = pcl;
R.J1 = J1; R.J2 = J2; R.k = k; R.c = c;
end
