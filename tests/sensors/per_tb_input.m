function ds = per_tb_input(P, S, nav, C)
%PER_TB_INPUT  External input Dataset of tests/sensors/tb_perception.slx / tb_per_tebvfh_top.slx (T13).
%
%   ds = per_tb_input(P, S, nav)        elements in root-port order: scan, cloud (ground), nav_state
%   ds = per_tb_input(P, S, nav, C)     with a cloud sequence C (t, xyz [32 x 1024 x 3 x N] single, nAz, valid)
%     S    scans (per_scan_gen): t (N x 1, multiples of Ts.lidar), ranges (N x 360), angles, valid; [] -> ground
%     nav  struct: t (M x 1, multiples of Ts.nav), pos (M x 3), eul (M x 3 [yaw pitch roll]), and optional
%          vel_B (M x 3), lidarLocActive (M x 1), quality (M x 1); other Bus_NavState elements = P.ic values
%   Root inports have Interpolate 'off' (zero-order hold of the samples).

ds = Simulink.SimulationData.Dataset;
%% ---- scan ----
if isempty(S)
    ds = ds.addElement([], 'scan');
else
    t = S.t(:);  N = numel(t);
    sc.t_scan = timeseries(t, t);
    sc.ranges = timeseries(single(S.ranges), t);
    sc.angles = timeseries(repmat(single(S.angles(:)'), N, 1), t);
    sc.valid = timeseries(logical(S.valid(:)), t);
    ds = ds.addElement(sc, 'scan');
end
%% ---- cloud ----
if nargin < 4 || isempty(C)
    ds = ds.addElement([], 'cloud');
else
    t = C.t(:);
    cl.t_scan = timeseries(t, t);
    cl.xyz = timeseries(single(C.xyz), t);
    cl.nAz = timeseries(uint16(C.nAz(:)), t);
    cl.valid = timeseries(logical(C.valid(:)), t);
    ds = ds.addElement(cl, 'cloud');
end
%% ---- nav_state ----
t = nav.t(:);  M = numel(t);
ic = P.ic.Bus_NavState;
fn = fieldnames(ic);
ns = struct();
for k = 1:numel(fn)
    v = ic.(fn{k});
    if isscalar(v)
        d = repmat(v, M, 1);
    elseif isvector(v)
        d = repmat(v(:)', M, 1);
    else
        d = repmat(v, [1 1 M]);
    end
    ns.(fn{k}) = d;
end
ns.t = t;
ns.pos_W = nav.pos;
ns.eul = nav.eul;
if isfield(nav, 'vel_B'), ns.vel_B = nav.vel_B; end
if isfield(nav, 'lidarLocActive'), ns.lidarLocActive = logical(nav.lidarLocActive(:)); end
if isfield(nav, 'quality'), ns.quality = uint8(nav.quality(:)); end
ns.navValid = true(M, 1);
for k = 1:numel(fn)
    d = ns.(fn{k});
    d = cast(d, class(ic.(fn{k})));
    ns.(fn{k}) = timeseries(d, t);
end
ds = ds.addElement(ns, 'nav_state');
end
