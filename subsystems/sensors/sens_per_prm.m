function c = sens_per_prm(P, which)
%SENS_PER_PRM  Pack the constants of the Shaddad_Perception MATLAB Function blocks into numeric row vectors.
%
%   c = sens_per_prm(P, which)       which: 'occ' | 'obs' | 'row' | 'sm' | 'c2s'
%
%   The perception MATLAB Function blocks take their constants on inputs (Constant blocks), not as parameter
%   data (same pattern as sens_lidar2d_fast_prm: R2026b 'Stateflow:Assertion o != nullptr' on masked / SSR
%   parameter data). Every value comes from P.sensors.per, P.sensors.lidar and P.world.occ.
%
%   'occ'  (sens_occmap_update)    [mapSize mapRes hit miss satLo satHi maxRange occThr]
%   'obs'  (sens_obstacle_extract) [segLambda sigR minPts minPtsFarRange maxObs maxRadius compactR nClMax
%                                   rLx rLy psiL beamRes]
%   'row'  (sens_row_detect)       [rRange(1:2) xWin(1:2) yWin(1:2) inlier iter minInl seed]
%   'sm'   (sens_scanmatch_loc)    [mapRes x0 y0 smRes smMaxRange smTrans(1:2) smRot smMinScoreRatio smNMin
%                                   rLx rLy psiL smCovFloor(1:3) rMin beamRes smGeoGap smGeoDmax]
%   'c2s'  (sens_lidar_cloud2scan) [rLx rLy rLz psiL zBand(1:2) selfBox(1:4) beamRes nBeams cloudMinPts rMin
%                                   rMax cloudLevel]

S = P.sensors;  per = S.per;  L = S.lidar;
psiL = L.eul_deg(1)*pi/180;
switch lower(which)
    case 'occ'
        c = [per.mapSize per.mapRes per.hit per.miss per.sat(1) per.sat(2) per.maxRange per.occThr];
    case 'obs'
        c = [per.segLambda L.sigR per.minPts per.minPtsFarRange per.maxObs per.maxRadius per.compactR ...
             per.nClMax L.r(1) L.r(2) psiL L.res];
    case 'row'
        c = [per.rowRRange(1) per.rowRRange(2) per.rowXWin(1) per.rowXWin(2) per.rowYWin(1) per.rowYWin(2) ...
             per.rowInlier per.rowIter per.rowMinInl per.rowSeed];
    case 'sm'
        occ = P.world.occ;
        c = [occ.res occ.origin(1) occ.origin(2) per.smRes per.smMaxRange per.smTrans(1) per.smTrans(2) ...
             per.smRot per.smMinScoreRatio per.smNMin L.r(1) L.r(2) psiL per.smCovFloor(1) per.smCovFloor(2) ...
             per.smCovFloor(3) L.rMin L.res per.smGeoGap per.smGeoDmax];
    case 'c2s'
        c = [L.r(1) L.r(2) L.r(3) psiL L.zBand(1) L.zBand(2) per.selfBox(1) per.selfBox(2) per.selfBox(3) ...
             per.selfBox(4) L.res L.nBeams per.cloudMinPts L.rMin L.full3d.MaxRange per.cloudLevel];
    otherwise
        error('sens_per_prm:which', 'Unknown block %s (occ | obs | row | sm | c2s)', which);
end
c = double(c);
end
