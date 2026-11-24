function v = tb_drive_load(model, P)
%TB_DRIVE_LOAD  Bench-load variables of a T06 drive bench derived from P (bench model-workspace names).
%   v = tb_drive_load(model, P)   struct of tb_* values
%     tb_drive, tb_drive_sys, tb_drive_foc   tb_J (m_side r^2 at the sprocket), tb_Trr (rolling torque f_rr m g r)
%     tb_drive_2mass                         tb_Jb_spr (belt-side inertia at the sprocket not modelled in drive_side)
%   build_tb_drive stores these as the bench defaults; tb_drive_run sets them from the P of every run, so a bench
%   built for an older robot (2026-10-27: 300 kg per side on a 0.128 m sprocket) runs with the current load.
b = P.drive.bench; g = P.drive.gear;
if strcmp(model, 'tb_drive_2mass')
    v.tb_Jb_spr = P.drive.bench2m.J_b*g.N^2 - P.drive.motor.J*g.N^2 - g.J_out;
else
    v.tb_J = b.J_load;
    v.tb_Trr = b.f_rr*b.m_side*b.g*g.r_spr;
end
end
