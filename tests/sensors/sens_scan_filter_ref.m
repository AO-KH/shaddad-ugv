function [rangesF, rangesB, anglesB, xB, yB, keep] = sens_scan_filter_ref(ranges, angles, P)
%SENS_SCAN_FILTER_REF  Reference MATLAB implementation of subsystems/sensors/sens_scan_filter.slx (T13).
%
%   [rangesF, rangesB, anglesB, xB, yB, keep] = sens_scan_filter_ref(ranges, angles, P)
%     ranges, angles  single[360] (Bus_LidarScanRaw, LiDAR frame, NaN = no return)
%     rangesF  single  filtered ranges, LiDAR frame (NaN = no return or removed)
%     rangesB  single  ranges about base_link (NaN where not kept)       (Bus_Perception.ranges_B)
%     anglesB  single  angles about base_link (sensor angle + mount yaw where not kept)  (angles_B)
%     xB, yB   double  points in base_link (NaN where not kept)
%     keep     logical kept beams
%
%   Same operations, order and data types as the block implementation (sensors spec 3.7 / F7, D59), used by
%   t_perception/tScanFilterBitExact (bit-for-bit comparison on 100 scans):
%     NaN removal: r0 = r (valid) | Inf (isNaN)                                   [single]
%     dust:        r0 < dustR and |r0 - r0(k-1)| > dustDr and |r0 - r0(k+1)| > dustDr (circular)  [single]
%     frame:       a = double(angle) + psiL; xB = double(r0) cos a + rLx; yB = double(r0) sin a + rLy  [double]
%     self box:    selfBox(1) <= xB <= selfBox(2) and selfBox(3) <= yB <= selfBox(4)
%     keep = ~isNaN & ~selfBox & ~dust;  rangesB = single(hypot(xB, yB)); anglesB = single(atan2(yB, xB))

per = P.sensors.per;  L = P.sensors.lidar;
r = single(ranges(:));  ang = single(angles(:));
psiL = L.eul_deg(1)*pi/180;
nanM = isnan(r);
valid0 = ~nanM;
r0 = r;  r0(nanM) = single(Inf);
n = numel(r0);
rPrev = r0([n 1:n-1]);  rNext = r0([2:n 1]);
dPrev = abs(r0 - rPrev);  dNext = abs(r0 - rNext);
iso = (r0 < single(per.dustR)) & (dPrev > single(per.dustDr)) & (dNext > single(per.dustDr));
aS = double(ang) + psiL;
XB = double(r0).*cos(aS) + L.r(1);
YB = double(r0).*sin(aS) + L.r(2);
sb = per.selfBox;
inBox = (XB >= sb(1)) & (XB <= sb(2)) & (YB >= sb(3)) & (YB <= sb(4));
keep = valid0 & ~inBox & ~iso;
rangesF = NaN(n, 1, 'single');  rangesF(keep) = r(keep);
rB = single(hypot(XB, YB));
rangesB = NaN(n, 1, 'single');  rangesB(keep) = rB(keep);
aB = single(atan2(YB, XB));
anglesB = single(aS);  anglesB(keep) = aB(keep);
xB = NaN(n, 1);  xB(keep) = XB(keep);
yB = NaN(n, 1);  yB(keep) = YB(keep);
end
