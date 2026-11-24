function c = sens_lidar2d_fast_prm(L, W)
%SENS_LIDAR2D_FAST_PRM  Pack the FAST2D constants of P.sensors.lidar and P.world into the numeric vector that the
%   MATLAB Function block Scan2D (sens_lidar_model/Fidelity/FAST2D) receives on its input 15.
%
%   c = sens_lidar2d_fast_prm(P.sensors.lidar, P.world)          c: 1 x 23 double
%
%   The block takes its constants on inputs (Constant blocks), not as masked parameter data: in R2026b a
%   MATLAB Function parameter two Subsystem Reference levels below a model fails the first compile after a
%   rebuild with 'Stateflow:Assertion o != nullptr' (see sens_gnss_fixchain_prm).
%   Index  field                         Index  field
%    1..4  [W.xlim W.ylim] raster extent   14    pDustMax
%     5    rasterRes (cells/m)             15    dustRange(1)
%     6    rMin                            16    dustRange(2)
%     7    rMax                            17    dustRangeLoss
%     8    sigR                            18    pRetFilm
%     9    rangeNoiseK                     19    filmBoxClass
%    10    zBand(1)                        20    tiltedBoxClass
%    11    zBand(2)                        21    W.obstacles.nCyl
%    12    pDrop                           22    W.obstacles.nBox
%    13    pDropDust                       23    W.obstacles.nFilm
%   sens_lidar2d_fast unpacks the same order; the obstacle tables W.obstacles.cyl / box / film and the beam
%   angles L.angles arrive on their own inputs.

O = W.obstacles;
c = [W.xlim(1) W.xlim(2) W.ylim(1) W.ylim(2), L.rasterRes, L.rMin, L.rMax, L.sigR, L.rangeNoiseK, ...
     L.zBand(1) L.zBand(2), L.pDrop, L.pDropDust, L.pDustMax, L.dustRange(1) L.dustRange(2), L.dustRangeLoss, ...
     L.pRetFilm, L.filmBoxClass, L.tiltedBoxClass, O.nCyl, O.nBox, O.nFilm];
c = double(c);
end
