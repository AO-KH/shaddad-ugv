function E = tb_drive_energy(so, P, ta)
%TB_DRIVE_ENERGY  T13 energy audit of one drive side in tb_drive from the Simscape log (T06).
%   E = tb_drive_energy(so, P)       window [0.5 s, end] (skips the DC-link inrush at t = 0)
%   E = tb_drive_energy(so, P, ta)   window [ta, end]
%   so = SimulationOutput with simlog (SimscapeLogType 'all') and truth. Balance on the drive boundary
%   (DC terminals in, sprocket out):
%     E_dc = E_spr + E_loss + dE_kin + dE_cap
%   E_loss = sum of int power_dissipated over every logged drive block (motor + ESC losses that enter the thermal
%   port, resistors, coil, gear drag) + mesh-loss source energy (-int T w) + brake friction (int |T w_rel|).
%   dE_th (energy stored in the 3-node thermal network) is reported as a cross-check of where E_loss goes.
%   E.err_pct = |E_dc - (E_spr + E_loss + dE_kin + dE_cap)| / int |p_dc| * 100
if nargin < 3, ta = 0.5; end
L = so.simlog.Drive;
tr = so.truth;
s0 = L.Plant.SYS.C_link.vc.series; t0 = s0.time;
if t0(1) > ta
    error('tb_drive_energy:log', ['Simscape log starts at %.3f s > window start %.3f s: run with ' ...
        '''SimscapeLogLimitData'', ''off'''], t0(1), ta);
end
win = @(t) t >= ta - 1e-12;
I = @(t, y) trapz(t(win(t)), y(win(t)));
D = @(t, y) y(end) - interp1(t, y, ta);
t = tr.p_dc.Time;
E.E_dc = I(t, tr.p_dc.Data);
E.abs_dc = I(t, abs(tr.p_dc.Data));
E.E_spr = I(t, tr.p_spr.Data);
[Ed, names] = local_diss(L, '', I);
ml = L.Gearbox.MeshLoss;
tm = ml.t.series.time; Em = -I(tm, ml.t.series.values('N*m').*ml.w.series.values('rad/s'));
fr = L.Brake.Friction;
tb = fr.t.series.time; Eb = I(tb, abs(fr.t.series.values('N*m').*fr.w_rel.series.values('rad/s')));
E.E_loss = Ed + Em + Eb;
E.E_mesh = Em; E.E_brake = Eb; E.E_diss = Ed;
E.diss_blocks = names;
J = P.drive.motor.J; Jo = P.drive.gear.J_out;
E.dE_kin = 0.5*J*D(t, tr.w_m.Data.^2) + 0.5*Jo*D(t, tr.w_spr.Data.^2);
cl = L.Plant.SYS.C_link.vc.series;
E.dE_cap = 0.5*P.drive.esc.C_link*D(cl.time, cl.values('V').^2);
E.residual = E.E_dc - (E.E_spr + E.E_loss + E.dE_kin + E.dE_cap);
E.err_pct = 100*abs(E.residual)/E.abs_dc;
th = P.drive.thermal;
E.dE_th = th.C_w*D(t, tr.T_winding.Data) + th.m_h*th.cp_h*D(t, tr.T_housing.Data) + th.m_esc*th.cp_esc*D(t, tr.T_esc.Data);
end

function [Esum, names] = local_diss(node, path, I)
Esum = 0; names = {};
ids = node.childIds;
for k = 1:numel(ids)
    ch = node.(ids{k});
    if strcmp(ids{k}, 'power_dissipated')
        try
            v = ch.series.values('W'); tt = ch.series.time;
            if size(v, 2) > 1, v = sum(v, 2); end
            Esum = Esum + I(tt, v);
            names{end + 1} = path; %#ok<AGROW>
        catch
        end
    elseif ~isempty(ch.childIds)
        [e, n] = local_diss(ch, [path '/' ids{k}], I);
        Esum = Esum + e; names = [names, n]; %#ok<AGROW>
    end
end
end
