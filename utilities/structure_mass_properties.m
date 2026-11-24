function mp = structure_mass_properties(S, config)
%STRUCTURE_MASS_PROPERTIES  Independent recomputation of the chassis mass properties (owner: structure, T03 T1).
%
%   mp = structure_mass_properties(S, config)
%     S      : P.structure (uses S.mass.components and S.impl only, never S.mp)
%     config : 'base' | 'mower' | 'tiller' | 'sprayer_full' | 'sprayer_empty'
%   mp.m [kg], mp.rcg [1x3] FRD w.r.t. R, mp.J [3x3] about the CG (FRD), mp.config
%
%   Method (deliberately different from param_structure's local_massprops, which sums box inertias about
%   the CG directly): every component is a homogeneous box; its inertia tensor is formed about the
%   reference point R with the parallel-axis term m(|p|^2 I - p p'), summed, and the total is shifted to
%   the CG with J_cg = J_R - m(|c|^2 I - c c'). J13 = -int(x z dm) is the off-diagonal element.

C = S.mass.components;                       % rows [m x y z lx ly lz]
im = S.impl;
switch lower(config)
    case 'base'
    case 'mower'
        C = [C; im.mower.mass im.mower.pos im.mower.dims];
    case 'tiller'
        C = [C; im.tiller.mass im.tiller.pos im.tiller.dims];
    case 'sprayer_empty'
        C = [C; im.sprayer.mass_dry im.sprayer.pos_dry im.sprayer.dims_dry];
    case 'sprayer_full'
        C = [C; im.sprayer.mass_dry im.sprayer.pos_dry im.sprayer.dims_dry; ...
                im.sprayer.V_max*im.sprayer.rho_liq im.sprayer.pos_liq im.sprayer.dims_liq];
    otherwise
        error('structure_mass_properties:config', 'Unknown configuration %s', config);
end
m = 0;  first = zeros(1,3);  JR = zeros(3);
for k = 1:size(C, 1)
    mk = C(k,1);  p = C(k,2:4);  a = C(k,5);  b = C(k,6);  c = C(k,7);
    m = m + mk;
    first = first + mk*p;
    Jbox = (mk/12)*[b^2 + c^2, 0, 0; 0, a^2 + c^2, 0; 0, 0, a^2 + b^2];
    JR = JR + Jbox + mk*((p*p.')*eye(3) - p.'*p);
end
rcg = first/m;
J = JR - m*((rcg*rcg.')*eye(3) - rcg.'*rcg);
mp = struct('config', config, 'm', m, 'rcg', rcg, 'J', J);
end
