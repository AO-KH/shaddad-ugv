function res = structure_identify_chi(varargin)
%STRUCTURE_IDENTIFY_CHI  Identify the skid-steer gauge factor chi from the HiFi model (owner structure, T03.12, D39).
%
%   res = structure_identify_chi()                       identify on world 6 (soil_strips) and WRITE
%                                                        chi_nom / chi_soil into params/param_structure.m
%   res = structure_identify_chi('Write', false, 'Soils', 1:5, 'R', 2, 'v', 0.5, 'vSpot', 0.5, 'Iter', 3)
%
%   For every soil class k (strip k of soil_strips, centre line y = 10k - 5) the HiFi bench
%   tests/structure/tb_structure.slx runs with ideal sprocket speed sources (open loop):
%     spot turn    belts L +vSpot / R -vSpot            -> res.chi_spot(k), res.slip_spot(k, 1:2)
%     R = 2 m turn at v (left, outer = right)           -> res.chi_turn(k), res.R_turn(k), res.v_turn(k),
%                  belts v -/+ chi B Omega/2, iterated ('Iter') with the measured chi so that the steady
%                  radius approaches R; chi = (v_bR - v_bL)/(B omega) from Bus_StructureDiag.chi_eff
%                  (mean over the steady window), sprocket torques res.T_turn(k, :) and turning power
%                  res.P_turn(k)
%   'Write' true: tr.chi_nom = chi_turn(farm soil 2) and tr.chi_soil = chi_turn(1:5) are written into
%   params/param_structure.m (owner structure). This single source updates P.electronics.mix.chi,
%   P.sensors.ekf.chi0, P.structure.lofi.chi and the GNC limits through their derivations (D39).
%   The shared base workspace is not modified (P / Ts / VC_* go to the runs by setVariable).
ip = inputParser;
ip.addParameter('Write', true);
ip.addParameter('Soils', 1:5);
ip.addParameter('R', 2);
ip.addParameter('v', 0.5);
ip.addParameter('vSpot', 0.5);
ip.addParameter('Iter', 3);
ip.addParameter('Quiet', false);
ip.parse(varargin{:});
o = ip.Results;

[P, Ts] = shaddad_params('World', 6, 'Assign', false);
rs = P.structure.r_sprocket;  B = P.structure.geom.B;
ns = numel(o.Soils);
res = struct('soils', o.Soils, 'chi_spot', nan(1, ns), 'slip_spot', nan(ns, 2), 'chi_turn', nan(1, ns), ...
    'R_turn', nan(1, ns), 'v_turn', nan(1, ns), 'T_turn', nan(ns, 2), 'P_turn', nan(1, ns), 'slip_turn', nan(ns, 2), ...
    'R', o.R, 'v', o.v, 'vSpot', o.vSpot, 'written', false);
for k = 1:ns
    soil = o.Soils(k);
    yc = 10*soil - 5;
    % ---- spot turn (clockwise: left belt forward) ----
    Pk = P;
    Pk.structure.init = structure_initial_state(Pk, 'Pose', [20 yc 0]);
    tbs = tb_structure_defaults(Pk);
    tbs.decim = 10;
    tbs.t = [0 1 1e3];  tbs.rref = [0 0 0];
    tbs.uL = [0 1 1]*o.vSpot/rs;  tbs.uR = -tbs.uL;
    out = structure_tb_run(Pk, Ts, tbs, 'StopTime', 7);
    [t, chi] = structure_tb_signal(out.tb_diag.chi_eff);
    [ts, sl] = structure_tb_signal(out.tb_tracks.slip);
    res.chi_spot(k) = mean(chi(t >= 4));
    res.slip_spot(k, :) = mean(sl(ts >= 4, :), 1);
    % ---- steady R turn (left) ----
    Pk.structure.init = structure_initial_state(Pk, 'Pose', [20 yc - o.R 0]);
    chiG = P.structure.track.chi_nom;
    vAct = o.v;
    for it = 1:o.Iter
        Om = vAct/o.R;                                 % yaw rate for R at the achieved ground speed
        dv = chiG*B*Om/2;
        tbs.uL = [0 1 1]*(o.v - dv)/rs;  tbs.uR = [0 1 1]*(o.v + dv)/rs;
        tbs.t = [0 2 1e3];
        out = structure_tb_run(Pk, Ts, tbs, 'StopTime', 16);
        [t, chi] = structure_tb_signal(out.tb_diag.chi_eff);
        [tt, om] = structure_tb_signal(out.tb_truth.omega_B);
        [~, vb] = structure_tb_signal(out.tb_truth.vel_B);
        [tk, tq] = structure_tb_signal(out.tb_tracks.sprTorque);
        [~, sp] = structure_tb_signal(out.tb_tracks.sprSpeed);
        [~, slt] = structure_tb_signal(out.tb_tracks.slip);
        w = tt >= 8;  wk = tk >= 8;
        chiG = mean(chi(t >= 8));
        res.chi_turn(k) = chiG;
        res.v_turn(k) = mean(vb(w, 1));
        vAct = res.v_turn(k);
        res.R_turn(k) = res.v_turn(k)/mean(om(w, 3));
        res.T_turn(k, :) = mean(tq(wk, :), 1);
        res.P_turn(k) = mean(sum(tq(wk, :).*sp(wk, :), 2));
        res.slip_turn(k, :) = mean(slt(wk, :), 1);
    end
    if ~o.Quiet
        fprintf('structure_identify_chi: soil %d  spot chi %.4f (slip %.3f / %.3f)  R-turn chi %.4f (R %.3f m, v %.3f m/s)\n', ...
            soil, res.chi_spot(k), res.slip_spot(k, 1), res.slip_spot(k, 2), res.chi_turn(k), res.R_turn(k), res.v_turn(k));
    end
end
if o.Write
    assert(isequal(sort(o.Soils), 1:5), 'structure_identify_chi: writing needs all five soil classes');
    chi_soil = res.chi_turn(:)';
    chi_nom = chi_soil(o.Soils == 2);
    assert(chi_nom > 1.2 && chi_nom < 1.8, 'identified chi_nom = %.4f outside the plausibility range 1.2..1.8', chi_nom);
    f = fullfile(char(currentProject().RootFolder), 'params', 'param_structure.m');
    txt = fileread(f);
    txt = regexprep(txt, 'tr\.chi_nom  = [0-9.]+;[^\n]*', ...
        sprintf('tr.chi_nom  = %.4f;                        %% IDENTIFIED by structure_identify_chi (HiFi, farm soil, R = %g m turn at %g m/s), D39', ...
        chi_nom, o.R, o.v), 'once');
    txt = regexprep(txt, 'tr\.chi_soil = \[[^\]]*\];[^\n]*', ...
        sprintf('tr.chi_soil = [%s];   %% IDENTIFIED per soil class 1..5 (R = %g m, %g m/s)', ...
        strjoin(arrayfun(@(x) sprintf('%.4f', x), chi_soil, 'UniformOutput', false), ' '), o.R, o.v), 'once');
    fid = fopen(f, 'w');
    fwrite(fid, txt);
    fclose(fid);
    clear param_structure
    res.written = true;
    res.chi_nom = chi_nom;
    res.chi_soil = chi_soil;
    if ~o.Quiet
        fprintf('structure_identify_chi: wrote chi_nom = %.4f, chi_soil = [%s] to %s\n', chi_nom, ...
            num2str(chi_soil, '%.4f '), f);
    end
end
end
