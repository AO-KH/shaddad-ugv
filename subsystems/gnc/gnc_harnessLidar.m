function [ranges, angles, prot, protLen, occL, gl] = gnc_harnessLidar(pose, v, t, occ, dyn, prm) %#codegen
%GNC_HARNESSLIDAR  Test-harness LiDAR + perception surrogate for the GNC mission bench (tests only).
%
%   [ranges, angles, prot, protLen, occL, gl] = gnc_harnessLidar(pose, v, t, occ, dyn, prm)
%     pose     true base_link pose [x y psi] (ENU)         v  signed forward speed (m/s)     t  time (s)
%     occ      P.world.occ.grid (row 1 = max y, prm.res cells/m, lower-left corner prm.origin)
%     dyn      [n x 8] dynamic obstacles [type x y a b yaw t_on t_off]: type 1 circle (radius a),
%              2 box (a x b, yaw), present for t_on <= t < t_off; type 0 rows are ignored
%     prm      struct: res, origin, rmin, rmax, nBeams (360), on, protXF, protXR, protL0, protKv, protW,
%              protStatic, mapN, mapRes
%     ranges   single [360] body-frame ranges, NaN = no return (rangeSensor on P.world.occ, plus an analytic
%              ray cast of the dynamic obstacles)
%     angles   single [360] beam angles -pi .. pi - 2 pi/360 (P.sensors.lidar.angles)
%     prot     protective-field stop: a return inside x in [protXF, protXF + protL0 + protKv |v|], |y| < protW/2
%              (rear field from protXR when v < 0), T13 geometry; with protStatic = 0 only dynamic-obstacle
%              returns count (the harness models a person / object detector; static structure is handled by
%              the corridor and VFH)
%     protLen  single protL0 + protKv |v|
%     occL     logical [mapN x mapN] robot-centred occupancy grid (static + dynamic), row 1 = max y
%     gl       [1 x 2] GridLocationInWorld of occL, snapped to the 1/mapRes world grid (Bus_TebMapInfo)
%
%   Replaced by the sensors-domain LiDAR + Shaddad_Perception in system simulation (gnc spec 4.5). The
%   rangeSensor System object runs compiled in the MATLAB Function block (persistent map and sensor).

persistent rs smap
nB = prm.nBeams;
angles = single(-pi + (0:nB - 1)'*(2*pi/nB));
mN = prm.mapN;
ranges = nan(nB, 1, 'single');
prot = false;
protLen = single(prm.protL0 + prm.protKv*abs(v));
occL = false(mN, mN);
res = prm.mapRes;
gl = round((pose(1:2)' - mN/(2*res))*res)/res;
if prm.on == 0
    return
end
if isempty(rs)
    smap = binaryOccupancyMap(occ, prm.res);
    smap.GridLocationInWorld = prm.origin;
    rs = rangeSensor('Range', [prm.rmin prm.rmax], 'HorizontalAngle', [-pi, pi - 2*pi/nB], ...
        'HorizontalAngleResolution', 2*pi/nB);
end
[rS, ~] = rs(pose(:)', smap);
r = double(rS(:));
r(isnan(r)) = inf;
% ---- dynamic obstacles: analytic ray cast ----
nd = size(dyn, 1);
isDyn = false(nB, 1);
px = pose(1);  py = pose(2);  ps = pose(3);
for k = 1:nd
    typ = dyn(k, 1);
    if typ < 1 || t < dyn(k, 7) || t >= dyn(k, 8)
        continue
    end
    cx = dyn(k, 2);  cy = dyn(k, 3);
    for b = 1:nB
        th = ps + double(angles(b));
        dx = cos(th);  dy = sin(th);
        d = inf;
        if typ == 1
            ox = px - cx;  oy = py - cy;
            bb = ox*dx + oy*dy;
            cc = ox*ox + oy*oy - dyn(k, 4)^2;
            disc = bb*bb - cc;
            if disc >= 0
                d1 = -bb - sqrt(disc);
                if d1 > 0, d = d1; elseif -bb + sqrt(disc) > 0, d = 0; end
            end
        else
            cyw = cos(dyn(k, 6));  syw = sin(dyn(k, 6));
            ox = (px - cx)*cyw + (py - cy)*syw;  oy = -(px - cx)*syw + (py - cy)*cyw;
            ux = dx*cyw + dy*syw;  uy = -dx*syw + dy*cyw;
            hx = dyn(k, 4)/2;  hy = dyn(k, 5)/2;
            tmin = -inf;  tmax = inf;
            [tmin, tmax] = local_slab(ox, ux, hx, tmin, tmax);
            [tmin, tmax] = local_slab(oy, uy, hy, tmin, tmax);
            if tmax >= max(tmin, 0)
                d = max(tmin, 0);
            end
        end
        if d >= prm.rmin && d <= prm.rmax && d < r(b)
            r(b) = d;
            isDyn(b) = true;
        end
    end
end
% ---- protective field (body frame) ----
L = prm.protL0 + prm.protKv*abs(v);
for b = 1:nB
    if isfinite(r(b)) && (prm.protStatic > 0 || isDyn(b))
        xb = r(b)*cos(double(angles(b)));  yb = r(b)*sin(double(angles(b)));
        inF = xb >= prm.protXF && xb <= prm.protXF + L;
        inR = v < 0 && xb <= prm.protXR && xb >= prm.protXR - L;
        if abs(yb) < prm.protW/2 && (inF || inR)
            prot = true;
        end
    end
end
r(~isfinite(r)) = nan;
ranges = single(r);
% ---- robot-centred occupancy grid: static crop + dynamic obstacles ----
NRw = size(occ, 1);  NCw = size(occ, 2);
org = prm.origin;
dc = round((gl(1) - org(1))*res);
dr = NRw - round((gl(2) + mN/res - org(2))*res);
i1 = max(1, 1 - dr);  i2 = min(mN, NRw - dr);
j1 = max(1, 1 - dc);  j2 = min(mN, NCw - dc);
if i2 >= i1 && j2 >= j1
    occL(i1:i2, j1:j2) = occ((i1:i2) + dr, (j1:j2) + dc);
end
for k = 1:nd
    typ = dyn(k, 1);
    if typ < 1 || t < dyn(k, 7) || t >= dyn(k, 8)
        continue
    end
    ext = max(dyn(k, 4), dyn(k, 5));
    ja = max(1, floor((dyn(k, 2) - ext - gl(1))*res));  jb = min(mN, ceil((dyn(k, 2) + ext - gl(1))*res) + 1);
    ia = max(1, floor((gl(2) + mN/res - dyn(k, 3) - ext)*res));  ib = min(mN, ceil((gl(2) + mN/res - dyn(k, 3) + ext)*res) + 1);
    for i = ia:ib
        yc = gl(2) + mN/res - (i - 0.5)/res;
        for j = ja:jb
            xc = gl(1) + (j - 0.5)/res;
            if typ == 1
                hit = hypot(xc - dyn(k, 2), yc - dyn(k, 3)) <= dyn(k, 4);
            else
                cyw = cos(dyn(k, 6));  syw = sin(dyn(k, 6));
                lx = (xc - dyn(k, 2))*cyw + (yc - dyn(k, 3))*syw;
                ly = -(xc - dyn(k, 2))*syw + (yc - dyn(k, 3))*cyw;
                hit = abs(lx) <= dyn(k, 4)/2 && abs(ly) <= dyn(k, 5)/2;
            end
            if hit
                occL(i, j) = true;
            end
        end
    end
end
end

function [tmin, tmax] = local_slab(o, u, h, tmin, tmax)
if abs(u) < 1e-12
    if abs(o) > h
        tmin = inf;  tmax = -inf;
    end
else
    t1 = (-h - o)/u;  t2 = (h - o)/u;
    tmin = max(tmin, min(t1, t2));
    tmax = min(tmax, max(t1, t2));
end
end
