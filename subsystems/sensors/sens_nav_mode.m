function [modeState, quality, navValid, aligned, headSel, lidarLocOn, rstOut, tSinceAbs, denied, resetCount, ...
    headingValid, chMask] = sens_nav_mode(t, navMode, rstExt, imuValid, still, iLow, headOkG, fix, avail, en, nis, Pd, prm)
%SENS_NAV_MODE  Navigation mode manager FSM (sensors spec F6, build task T12; Stateflow not installed).
%
%   States (modeState, uint8): 0 INIT, 1 ALIGN, 2 RUN_GNSS, 3 RUN_LIDAR, 4 DEAD_RECKONING, 5 FAULT
%     INIT       IMU valid for prm.tImuValid (2 s) -> ALIGN
%     ALIGN      heading source headSel = 1 dual antenna if its heading is valid, else 2 ecompass when standing
%                still with abs(I_pack) < 5 A, else 0; aligned when sigma_pos < sigPos (0.15 m) and
%                sigma_psi < sigYaw (2 deg) -> RUN_GNSS (RUN_LIDAR when the LiDAR mode is requested)
%     RUN_GNSS   -> RUN_LIDAR when navMode == 2, or AUTO (0) with a prior map and no RTK fix (4/5) for
%                tLidarSwitch (5 s); -> DEAD_RECKONING when no absolute update for drTimeout (2 s)
%     RUN_LIDAR  lidarLocOn; -> RUN_GNSS when navMode == 1, or AUTO with RTK fixed for tGnssReturn (2 s)
%     DEAD_RECKONING  -> RUN_GNSS / RUN_LIDAR on the next accepted absolute update; navValid false after
%                tAbsMax (10 s) without one; lidarLocOn while a prior map can be used
%     FAULT      moving-average normalised NIS > nisK (3) for nisT (5 s) -> one reset pulse (rstOut) -> INIT (transient);
%                IMU invalid for tImuLossFault (1 s) in ALIGN, RUN_GNSS, RUN_LIDAR or DEAD_RECKONING -> one reset pulse,
%                FAULT held while the IMU stays invalid, then a second reset pulse -> INIT (re-alignment from x0 / P0)
%   IMU loss (critfix R-05, NAV-03): the IMU-invalid time counts in every state; from tImuLossInv (0.1 s) on, navValid
%   is false and quality INVALID (meanwhile the EKF propagates the held IMU sample of NavPreproc).
%   navReset (rising edge rstExt from Bus_NavCmd) -> rstOut pulse -> INIT, aligned = false.
%   Inputs: t [s], navMode (0 AUTO, 1 GNSS, 2 LIDAR_PRIOR_MAP, 3 DR_TEST), rstExt, imuValid, still, iLow,
%     headOkG (dual-antenna heading valid), fix (latest GNSS fix type 0/1/4/5), avail(5) new measurement,
%     en(5) accepted corrections (sens_nav_gate), nis(5), Pd = [P11 P22 P44] (delayed EKF covariance),
%     prm = P.sensors.navmode.
%   Outputs: modeState, quality (NavQuality codes 0..5), navValid, aligned, headSel (0/1/2), lidarLocOn,
%     rstOut (EKF reset pulse), tSinceAbs (time since the last accepted GNSS / LiDAR position), denied
%     (no GNSS position / velocity and no scan match present for > tDenied, accepted or not: slip hold of D49), resetCount, headingValid,
%     chMask(5) correction-channel mask (DR_TEST uses odometry only once aligned, GNSS mode drops LiDAR).
%#codegen
persistent st tImu isAl tLastAbs tNoRtk tRtk ema tHigh nRst lastQ tLastSm tLastAv tImuBad imuF
if isempty(st)
    st = uint8(0); tImu = 0; isAl = false; tLastAbs = -1e9; tNoRtk = 0; tRtk = 0; ema = 1; tHigh = 0;
    nRst = 0; lastQ = uint8(0); tLastSm = -1e9; tLastAv = -1e9; tImuBad = 0; imuF = false;
end
dt = prm.dt;
rstOut = false;
if rstExt
    rstOut = true;
    st = uint8(0); tImu = 0; isAl = false; tLastAbs = -1e9; ema = 1; tHigh = 0; lastQ = uint8(0);
    tLastSm = -1e9; imuF = false;
    nRst = nRst + 1;
end

% ---- timers ----
if imuValid
    tImu = tImu + dt;  tImuBad = 0;
else
    tImu = 0;  tImuBad = tImuBad + dt;       % IMU-invalid time, every state (critfix R-05)
end
imuLost = tImuBad >= prm.tImuLossInv - 1e-9;
if en(1) || en(5)
    tLastAbs = t;
end
if avail(1) || avail(2) || avail(5)       % GNSS or scan match present (accepted or not)
    tLastAv = t;
end
if en(5)
    tLastSm = t;
end
if en(1)
    if fix == 4, lastQ = uint8(4); elseif fix == 5, lastQ = uint8(3); elseif fix == 1, lastQ = uint8(2); end
end
if avail(1) && (fix == 4 || fix == 5)    % RTK epoch (fixed or float)
    tNoRtk = 0;
else
    tNoRtk = tNoRtk + dt;
end
if fix == 4, tRtk = tRtk + dt; else, tRtk = 0; end
tSinceAbs = min(t - tLastAbs, 1e6);

% ---- consistency monitor: EMA of the mean normalised NIS of the accepted channels ----
nAcc = 0; s = 0;
for j = 1:5
    if en(j)
        nAcc = nAcc + 1;
        s = s + nis(j)/prm.m(j);
    end
end
if nAcc > 0
    ema = ema + (s/nAcc - ema)*min(1, dt/prm.nisTau);
end
if isAl && st >= 2 && st <= 4 && ema > prm.nisK
    tHigh = tHigh + dt;
else
    tHigh = 0;
end

% ---- transitions ----
sigP = sqrt(max(Pd(1), Pd(2)));
sigY = sqrt(Pd(3));
wantLidar = navMode == 2 || (navMode == 0 && prm.priorMap > 0 && tNoRtk > prm.tLidarSwitch);
switch st
    case 0  % INIT
        if tImu >= prm.tImuValid
            st = uint8(1);
        end
    case 1  % ALIGN
        if sigP < prm.sigPos && sigY < prm.sigYaw
            isAl = true;
            if wantLidar, st = uint8(3); else, st = uint8(2); end
        end
    case 2  % RUN_GNSS
        if tSinceAbs > prm.drTimeout
            st = uint8(4);
        elseif wantLidar
            st = uint8(3);
        end
    case 3  % RUN_LIDAR
        if tSinceAbs > prm.drTimeout
            st = uint8(4);
        elseif navMode == 1 || (navMode == 0 && tRtk > prm.tGnssReturn)
            st = uint8(2);
        end
    case 4  % DEAD_RECKONING
        if tSinceAbs <= prm.drTimeout
            if t - tLastSm < prm.tLidarFresh && navMode ~= 1, st = uint8(3); else, st = uint8(2); end
        end
    otherwise  % FAULT: transient after the NIS monitor; after an IMU loss held until the IMU is valid again
        if ~imuF
            st = uint8(0);
        elseif imuValid
            st = uint8(0);  imuF = false;
            rstOut = true;  nRst = nRst + 1;     % EKF from x0 / P0 again: the held IMU sample was propagated meanwhile
            tLastAbs = -1e9; ema = 1; tHigh = 0; lastQ = uint8(0); tLastSm = -1e9;
        end
end
imuFault = st >= 1 && st <= 4 && tImuBad >= prm.tImuLossFault - 1e-9;
if tHigh > prm.nisT || imuFault
    st = uint8(5);
    rstOut = true;
    nRst = nRst + 1;
    isAl = false; tImu = 0; tLastAbs = -1e9; ema = 1; tHigh = 0; lastQ = uint8(0); tLastSm = -1e9;
    imuF = imuFault;
end
modeState = st;

% ---- outputs ----
aligned = isAl;
run = st >= 2 && st <= 4;
navValid = isAl && run && tSinceAbs < prm.tAbsMax && sigP < prm.sigPosValid && ~imuLost;
headingValid = isAl && sigY < prm.sigYaw;
if headOkG
    headSel = 1;
elseif st <= 1 && still && iLow
    headSel = 2;
else
    headSel = 0;
end
lidarLocOn = st == 3 || (st == 4 && prm.priorMap > 0 && navMode ~= 1 && navMode ~= 3);
if ~navValid
    quality = uint8(0);
elseif st == 4
    quality = uint8(1);
elseif st == 3 && t - tLastSm < prm.tLidarFresh
    quality = uint8(5);
elseif lastQ > 0
    quality = lastQ;
else
    quality = uint8(1);
end
denied = t - tLastAv > prm.tDenied;      % D49: GNSS and scan matching both unavailable
resetCount = uint16(min(nRst, 65535));
chMask = true(5, 1);
if navMode == 3 && isAl
    chMask = [false; false; false; true; false];
elseif navMode == 1
    chMask(5) = false;
end
end
