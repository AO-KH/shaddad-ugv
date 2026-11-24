function [ranges, valid, nPts] = sens_lidar_cloud2scan(xyz, nAz, rp, cloudValid, c)
%SENS_LIDAR_CLOUD2SCAN  Organised 3-D LiDAR cloud -> 360-beam height-band scan (sensors spec 3.4 / F4).
%#codegen
%   [ranges, valid, nPts] = sens_lidar_cloud2scan(xyz, nAz, rp, cloudValid, c)
%     xyz        single[32 x 1024 x 3] organised cloud in the LiDAR frame (NaN padded; FULL3D: 31 x 360 used,
%                HW Ouster: 32 x 1024)
%     nAz        used azimuth columns (Bus_LidarCloudRaw.nAz)
%     rp         [roll pitch] of base_link [rad] (Bus_NavState.eul(3), eul(2); pitch + = nose down)
%     cloudValid cloud valid flag
%     c          sens_per_prm(P, 'c2s') = [rLx rLy rLz psiL zBand(1:2) selfBox(1:4) beamRes nBeams minPts rMin
%                rMax level]
%     ranges     single[nBeams x 1], beam k at -pi + (k-1)*beamRes in the LiDAR frame (Bus_LidarScanRaw), NaN =
%                no point in the band
%     valid      cloudValid and nPts > minPts
%     nPts       band points used
%
%   Each finite point p (LiDAR frame) -> base_link pB = Rz(psiL) p + r_L (level mount). Height band: with
%   level = 0 (default) z_B of pB in [zBand(1), zBand(2)] (band parallel to the body x-y plane through base_link
%   at ground level, i.e. height above a ground plane under the tracks; the FAST2D band semantics, no ground
%   returns on uniform slopes); with level = 1 the point is first levelled with the estimated roll / pitch
%   (Ry(pitch) Rx(roll) pB, as in the spec pseudo-code). Points inside the self box (x, y of pB) or with a
%   horizontal range outside [rMin, rMax] are dropped. Bin = nearest beam of the azimuth seen from the LiDAR,
%   range = minimum horizontal distance per bin.

nB = 360;                                   % Bus_LidarScanRaw.ranges is single[360] (ICD 10, fixed size)
assert(round(c(12)) == nB, 'sens_lidar_cloud2scan: nBeams must equal the bus size 360');
ranges = NaN(nB, 1, 'single');
nPts = 0;
rL = [c(1); c(2); c(3)];  psiL = c(4);  zb = [c(5) c(6)];  sb = [c(7) c(8) c(9) c(10)];
res = c(11);  minPts = c(13);  rMin = c(14);  rMax = c(15);  lev = c(16) > 0.5;
cz = cos(psiL);  sz = sin(psiL);
cr = cos(rp(1));  sr = sin(rp(1));  cp = cos(rp(2));  sp = sin(rp(2));
nRow = size(xyz, 1);
nCol = min(size(xyz, 2), max(0, round(double(nAz))));
best = inf(nB, 1);
for j = 1:nCol
    for i = 1:nRow
        x = double(xyz(i, j, 1));  y = double(xyz(i, j, 2));  z = double(xyz(i, j, 3));
        if ~(isfinite(x) && isfinite(y) && isfinite(z)), continue, end
        xb = cz*x - sz*y + rL(1);  yb = sz*x + cz*y + rL(2);  zb0 = z + rL(3);
        if lev
            % Rx(roll) then Ry(pitch) (ZYX, pitch + = nose down): level frame without yaw
            y1 = cr*yb - sr*zb0;  z1 = sr*yb + cr*zb0;
            x2 = cp*xb + sp*z1;   z2 = -sp*xb + cp*z1;
            xh = x2;  yh = y1;  zh = z2;
        else
            xh = xb;  yh = yb;  zh = zb0;
        end
        if zh <= zb(1) || zh >= zb(2), continue, end
        if xb >= sb(1) && xb <= sb(2) && yb >= sb(3) && yb <= sb(4), continue, end
        dx = xh - rL(1);  dy = yh - rL(2);
        rr = hypot(dx, dy);
        if rr < rMin || rr > rMax, continue, end
        az = atan2(dy, dx) - psiL;
        k = floor((az + pi)/res + 0.5) + 1;
        if k > nB, k = k - nB; end
        if k < 1, k = k + nB; end
        nPts = nPts + 1;
        if rr < best(k), best(k) = rr; end
    end
end
for k = 1:nB
    if isfinite(best(k)), ranges(k) = single(best(k)); end
end
valid = cloudValid && nPts > minPts;
end
