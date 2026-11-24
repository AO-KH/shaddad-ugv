function bev = shaddad_replay_birdseye(file, varargin)
%SHADDAD_REPLAY_BIRDSEYE  Offline bird's-eye replay of a saved scenario run (integration spec 5.2, T16).
%
%   shaddad_replay_birdseye()                         latest work/results/S*.mat
%   shaddad_replay_birdseye(file)                     a run_scenario results file (or a results struct)
%   bev = shaddad_replay_birdseye(file, 'Step', k, 'Video', 'out.mp4', 'Visible', tf, 'Pause', s)
%     Step     replay every k-th 0.1 s frame (default 1)
%     Video    MP4 file name: frames written with VideoWriter (MPEG-4)
%     Visible  figure visibility (default true)
%     Pause    pause between frames [s] (default 0, as fast as the graphics allow)
%   Uses the same System object as the live view (shaddad.BirdsEyeView), so live and replay drawing share
%   one code path. Needs res.S (truth.pose2D, nav_state, telem) and res.viz (map, path, footprint).

ip = inputParser;
ip.addParameter('Step', 1);
ip.addParameter('Video', '');
ip.addParameter('Visible', true);
ip.addParameter('Pause', 0);
ip.parse(varargin{:});
o = ip.Results;
if nargin < 1 || isempty(file)
    d = dir(fullfile(char(currentProject().RootFolder), 'work', 'results', 'S*.mat'));
    if isempty(d), error('shaddad_replay_birdseye:none', 'No results in work/results'); end
    [~, k] = max([d.datenum]);
    file = fullfile(d(k).folder, d(k).name);
end
if isstruct(file), r = file; else, r = load(file); end
S = r.S;
vz = struct('occ', struct('grid', false(2), 'res', 10, 'origin', [0 0]), 'path_xy', nan(1, 2), 'footprint', [1.0 0.78]);
if isfield(r, 'viz') && ~isempty(r.viz), vz = r.viz; end
bev = shaddad.BirdsEyeView('MapImage', vz.occ.grid, 'MapResolution', vz.occ.res, 'MapOrigin', vz.occ.origin, ...
    'Footprint', vz.footprint, 'PathXY', vz.path_xy, 'Visible', logical(o.Visible));
ps = S.truth.pose2D;
t = (ps.t(1):0.1*o.Step:ps.t(end))';
pT = scn_metrics('resample', ps, t);
if isfield(S, 'nav_state')
    pn = scn_metrics('resample', S.nav_state.pos_W, t);
    yn = scn_metrics('resample', S.nav_state.eul, t);
    pE = [pn(:, 1:2) yn(:, 1)];
else
    pE = pT;
end
md = zeros(numel(t), 1);  soc = zeros(numel(t), 1);
if isfield(S, 'telem')
    md = double(scn_metrics('resample', S.telem.mode, t));
    soc = double(scn_metrics('resample', S.telem.SOC, t));
end
vw = [];
if ~isempty(o.Video)
    vw = VideoWriter(o.Video, 'MPEG-4');
    vw.FrameRate = 10;
    open(vw);
end
rg = nan(360, 1, 'single');  an = single(linspace(-pi, pi, 360)');
for k = 1:numel(t)
    bev(pT(k, :)', pE(k, :)', rg, an, uint8(md(k)), soc(k));
    if ~isempty(vw), writeVideo(vw, getframe(bev.figureHandle)); end
    if o.Pause > 0, pause(o.Pause); end
end
if ~isempty(vw), close(vw); end
end
