function [en, R1, R2, R3, R4, R5, nis, rej] = sens_nav_gate(xP, PP, uP, QP, rst, z1, z2, z3, z4, z5, avail, ...
    accG, s3, C5, u1, u2, u3, u4, u5, fix, prm)
%SENS_NAV_GATE  Innovation gating and time-varying R of the navigation EKF (sensors spec F5, build task T12).
%
%   Runs at 0.01 s before the EKF (Shaddad_NavFusion/NavGate). The EKF has no innovation gating, so every
%   correction channel j is enabled (en(j)) only on a new, consistent measurement (D63):
%     1 GNSS position [E N U]   2 GNSS velocity [vE vN]   3 heading   4 track speeds [vL vR]   5 LiDAR pose
%   Inputs
%     xP, PP     x(k-1|k-1), P(k-1|k-1) (Unit Delays on the EKF outputs)
%     uP, QP     state-transition input and process noise of the previous step (Unit Delays): the gate
%                predicts x(k|k-1), P(k|k-1) exactly as the EKF does before its corrections
%     rst        delayed reset pulse (clears the rejection counters)
%     z1..z5     measurements (z3 = selected heading source, z5 = scan-match pose)
%     avail(5)   new measurement present this sample (slow-to-fast new-data flags, channel masks)
%     accG(3)    reported GNSS [hAcc vAcc sAcc] (1 sigma)
%     s3         1-sigma of the selected heading (kR*headAcc dual antenna, or ecompass sigma)
%     C5 (3x3)   scan-match covariance (W)
%     u1..u5     extra inputs of the measurement functions (as passed to the EKF)
%     fix        GNSS fix type of the epoch (0 NOFIX, 1 SINGLE, 4 RTK_FIXED, 5 RTK_FLOAT)
%     prm        P.sensors.navgate (kR, kRpos, kRvel, nBypass, chi2, Nmax, kForce, od0, od1, smFloor, smInfl,
%                smWeakRatio, smCap, x0, P0)
%   Outputs
%     en(5)      correction enables;  R1..R5 measurement noise passed to the EKF
%     nis(5)     normalised innovation squared (0 when the channel was not evaluated)
%     rej(5)     consecutive rejections per channel
%   R: R1 = kRpos(fix)^2 diag([hAcc^2 hAcc^2 vAcc^2]) (update; the gate uses kR),
%      R2 = kRvel(fix)^2 diag([sAcc^2 sAcc^2]) (update; the gate uses kR), R3 = s3^2,
%      R4 = diag((od0 + od1 |vTrack|)^2), R5 = C5 with its diagonal floored at smFloor, times smInfl, its weak
%      translation axis raised by the start guard when it applies (local_weak_guard).
%   Chi-square gate p = 0.999 (prm.chi2 for m = 1/2/3); after Nmax(j) consecutive rejections the measurement
%   is force-accepted with R x kForce(j) (no lock-out after a genuine jump).
%   Revision (T12 bench): the GNSS position error of FLOAT / SINGLE is a Gauss-Markov bias (correlation 20 / 50 s),
%   so the R passed to the EKF update is inflated by kRpos = [SINGLE FLOAT FIXED] = prm.kRpos; the chi-square test
%   keeps the spec R (kR), otherwise 2-5 m multipath outliers would pass the gate of an inflated FLOAT R;
%   and when the fix improves (SINGLE -> FLOAT -> FIXED, e.g. an RTK re-fix) the next nBypass position epochs
%   bypass the chi-square gate: the old biased estimate must not lock out the better solution.
%   Revision (S4 nav fix round 2, 2026-11-14): the GNSS velocity R of the update is inflated per fix as well,
%   kRvel = [SINGLE FLOAT FIXED] = prm.kRvel (FIXED = kR, unchanged): without RTK (under the greenhouse film roof,
%   multipath, few satellites) the receiver's Doppler velocity is less certain than its sAcc says, and an
%   accepted biased velocity drags the slip states (12-15 % in S4 with the old GPS plant model). The chi-square test
%   keeps the spec R (kR), as for channel 1, so a gross velocity error is still rejected.
%   Channel 5 (same revision): the matcher reports its along-aisle sigma from the scan geometry now (about 0.15 m
%   in the S4 aisle instead of the 0.52 m sampling window), but along the aisle it keeps part of a start error ahead
%   of the truth. A start guard (local_weak_guard, prm.smWeakRatio / prm.smCap) down-weights the weak axis of a
%   match that did not move from an uncertain start, before the chi-square test. It only delays such a match: each
%   firing halves the prior variance, so from a 0.6 m prior it fires about 3 times and then lets go.
%#codegen
persistent rc rank0 nByp
if isempty(rc)
    rc = zeros(5, 1);
    rank0 = 0;
    nByp = 0;
end
if rst
    rc(:) = 0;
    rank0 = 0;
    nByp = 0;
end
% ---- prediction x(k|k-1), P(k|k-1) (same model as the EKF) ----
xp = sens_ekf_f(xP, uP);
F = sens_ekf_dfdx(xP, uP);
Pp = F*PP*F.' + QP;
Pp = (Pp + Pp.')/2;

% ---- measurement noise ----
kR2 = prm.kR^2;
rank = local_rank(fix);
kP = prm.kRpos(max(rank, 1));
R1g = diag(kR2*[accG(1)^2 accG(1)^2 accG(2)^2]);    % gating (spec R)
R1 = diag(kP^2*[accG(1)^2 accG(1)^2 accG(2)^2]);     % update (correlated-error inflation)
kV = prm.kRvel(max(rank, 1));
R2g = diag(kR2*[accG(3)^2 accG(3)^2]);               % gating (spec R)
R2 = diag(kV^2*[accG(3)^2 accG(3)^2]);               % update (no-RTK Doppler inflation)
R3 = s3^2;
R4 = diag((prm.od0 + prm.od1*abs(z4(:))).^2);
R5 = (C5 + C5.')/2;
for i = 1:3
    R5(i, i) = max(R5(i, i), prm.smFloor(i));
end
R5 = R5*prm.smInfl;

en = false(5, 1);
nis = zeros(5, 1);

% ---- channel 1: GNSS position ----
if avail(1) && all(isfinite(z1)) && all(isfinite(R1(:)))
    H = sens_ekf_dh_gnsspos(xp, u1);
    nu = z1(:) - sens_ekf_h_gnsspos(xp, u1);
    S = H*Pp*H.' + R1g;
    nis(1) = nu.'*(S\nu);
    if rank > rank0
        nByp = prm.nBypass;      % fix improved: the next epochs bypass the gate
    end
    rank0 = rank;
    if nByp > 0
        en(1) = true;
        rc(1) = 0;
        nByp = nByp - 1;
    else
        [en(1), f, rc] = local_pass(nis(1), prm.chi2(3), 1, rc, prm);
        R1 = R1*f;
    end
end
% ---- channel 2: GNSS velocity ----
if avail(2) && all(isfinite(z2)) && all(isfinite(R2(:)))
    H = sens_ekf_dh_gnssvel(xp, u2);
    nu = z2(:) - sens_ekf_h_gnssvel(xp, u2);
    S = H*Pp*H.' + R2g;
    nis(2) = nu.'*(S\nu);
    [en(2), f, rc] = local_pass(nis(2), prm.chi2(2), 2, rc, prm);
    R2 = R2*f;
end
% ---- channel 3: heading (wrapped residual) ----
if avail(3) && isfinite(z3) && isfinite(R3)
    H = sens_ekf_dh_heading(xp, u3);
    [zh, ~] = sens_ekf_h_heading(xp, u3);
    nu = local_wrap(z3 - zh);
    S = H*Pp*H.' + R3;
    nis(3) = nu*nu/S;
    [en(3), f, rc] = local_pass(nis(3), prm.chi2(1), 3, rc, prm);
    R3 = R3*f;
end
% ---- channel 4: track speeds ----
if avail(4) && all(isfinite(z4))
    H = sens_ekf_dh_odom(xp, u4);
    nu = z4(:) - sens_ekf_h_odom(xp, u4);
    S = H*Pp*H.' + R4;
    nis(4) = nu.'*(S\nu);
    [en(4), f, rc] = local_pass(nis(4), prm.chi2(2), 4, rc, prm);
    R4 = R4*f;
end
% ---- channel 5: LiDAR scan-to-map pose (wrapped yaw residual) ----
if avail(5) && all(isfinite(z5)) && all(isfinite(R5(:)))
    H = sens_ekf_dh_lidarpose(xp, u5);
    [zh, ~] = sens_ekf_h_lidarpose(xp, u5);
    nu = z5(:) - zh;
    nu(3) = local_wrap(nu(3));
    Pz = H*Pp*H.';
    R5 = local_weak_guard(R5, Pz, nu, prm);
    S = Pz + R5;
    nis(5) = nu.'*(S\nu);
    [en(5), f, rc] = local_pass(nis(5), prm.chi2(3), 5, rc, prm);
    R5 = R5*f;
end
rej = rc;
end

function [pass, f, rc] = local_pass(q, thr, j, rc, p)
% chi-square test with forced acceptance after Nmax(j) consecutive rejections
f = 1;
pass = q < thr;
if pass
    rc(j) = 0;
else
    rc(j) = rc(j) + 1;
    if rc(j) > p.Nmax(j)
        pass = true;
        f = p.kForce(j);
        rc(j) = 0;
    end
end
end

function R = local_weak_guard(R, Pz, nu, prm)
% start guard on the weak translation axis of a scan match (an aisle; S4 nav fix round 2): the larger eigenvalue
% lam2 of R(1:2, 1:2) is a weak axis when lam2 >= smWeakRatio lam1. When the predicted pose was uncertain along it
% (variance sS2 > smCap^2 and sS2 > lam2, about 0.24 m in S4) and the match moved less than the matcher's own sigma
% along it (|v2' nu| < sqrt(lam2 / smInfl)), the match may have followed its start; its weak-axis variance is then
% raised to sS2 (congruence transform: the correlations with the other axes scale with it), so the update at most
% halves the prior variance along that axis instead of collapsing it. A match that moved, or one from a confident
% start, keeps its R. The guard delays a match that keeps returning its start by a few scans; it does not catch it
% (STATUS 16d).
a = R(1, 1);  b = R(1, 2);  d = R(2, 2);
df = sqrt(((a - d)/2)^2 + b^2);
lam1 = (a + d)/2 - df;  lam2 = (a + d)/2 + df;
if ~(lam1 > 0) || lam2 < prm.smWeakRatio*lam1
    return
end
if abs(b) > 1e-15
    v2 = [lam2 - d; b];
elseif a >= d
    v2 = [1; 0];
else
    v2 = [0; 1];
end
v2 = v2/sqrt(v2(1)^2 + v2(2)^2);  v1 = [-v2(2); v2(1)];
sS2 = v2.'*Pz(1:2, 1:2)*v2;
dW = v2.'*nu(1:2);
if sS2 > prm.smCap^2 && sS2 > lam2 && dW*dW < lam2/prm.smInfl
    s = sqrt(sS2/lam2);
    T = eye(3);
    T(1:2, 1:2) = v1*v1.' + s*(v2*v2.');
    R = T*R*T.';
    R = 0.5*(R + R.');
end
end

function r = local_rank(fix)
% fix quality rank: 0 NOFIX, 1 SINGLE, 2 RTK_FLOAT, 3 RTK_FIXED
r = 0;
if fix == 1
    r = 1;
elseif fix == 5
    r = 2;
elseif fix == 4
    r = 3;
end
end

function a = local_wrap(a)
a = atan2(sin(a), cos(a));
end
