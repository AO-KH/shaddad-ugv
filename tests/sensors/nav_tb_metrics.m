function M = nav_tb_metrics(R, tWin)
%NAV_TB_METRICS  Navigation errors of a tb_nav run against the bench truth (build task T12).
%
%   M = nav_tb_metrics(R)            R from nav_tb_run
%   M = nav_tb_metrics(R, [t0 t1])   statistics over t0 <= t <= t1 (default whole run after 15 s)
%   Fields (time series N x 1 unless noted, then window statistics):
%     t, ePos (N x 3, estimate - truth ENU), eH (horizontal norm), eYaw (rad, wrapped), eV (speed), eVy,
%     ePitch, eRoll, eSlip (N x 2), eChi, nees3 (3-DOF pose NEES [pE pN psi] with covPose),
%     eAlong / eCross (horizontal error along / across the true heading)
%     rmsH, maxH, rmsYaw, maxYaw, rmsV, rmsCross, maxCross, rmsAlong, maxAlong, meanNees3, rmsPitch, maxPitch
if nargin < 2 || isempty(tWin), tWin = [15 Inf]; end
n = R.nav; tr = R.truth;
N = min(numel(n.time), size(tr, 1));
t = n.time(1:N);
pos = n.pos_W(1:N, :);  eul = n.eul(1:N, :);
M.t = t;
M.ePos = pos - tr(1:N, 2:4);
M.eH = hypot(M.ePos(:, 1), M.ePos(:, 2));
M.eYaw = local_wrap(eul(:, 1) - tr(1:N, 5));
M.ePitch = eul(:, 2) - tr(1:N, 6);
M.eRoll = eul(:, 3) - tr(1:N, 7);
M.eV = n.vel_B(1:N, 1) - tr(1:N, 8);
M.eVy = n.vel_B(1:N, 2) - tr(1:N, 9);
M.eSlip = n.slip_LR(1:N, :) - tr(1:N, 11:12);
M.eChi = n.chi(1:N) - tr(1:N, 13);
psi = tr(1:N, 5);
M.eAlong = M.ePos(:, 1).*cos(psi) + M.ePos(:, 2).*sin(psi);
M.eCross = -M.ePos(:, 1).*sin(psi) + M.ePos(:, 2).*cos(psi);
C = n.covPose;
M.nees3 = zeros(N, 1);
for k = 1:N
    e = [M.ePos(k, 1); M.ePos(k, 2); M.eYaw(k)];
    Ck = C(:, :, k);
    if size(C, 3) < N, Ck = squeeze(C(k, :, :)); end
    M.nees3(k) = e.'*(Ck\e);
end
w = t >= tWin(1) & t <= tWin(2);
M.win = w;
M.rmsH = sqrt(mean(M.eH(w).^2));  M.maxH = max(M.eH(w));
M.rmsYaw = sqrt(mean(M.eYaw(w).^2));  M.maxYaw = max(abs(M.eYaw(w)));
M.rmsV = sqrt(mean(M.eV(w).^2));
M.rmsCross = sqrt(mean(M.eCross(w).^2));  M.maxCross = max(abs(M.eCross(w)));
M.rmsAlong = sqrt(mean(M.eAlong(w).^2));  M.maxAlong = max(abs(M.eAlong(w)));
M.meanNees3 = mean(M.nees3(w));
M.rmsPitch = sqrt(mean(M.ePitch(w).^2));  M.maxPitch = max(abs(M.ePitch(w)));
M.rmsVy = sqrt(mean(M.eVy(w).^2));  M.maxVy = max(abs(M.eVy(w)));
end

function a = local_wrap(a)
a = atan2(sin(a), cos(a));
end
