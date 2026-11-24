classdef Robot3DView < matlab.System
    %ROBOT3DVIEW  Live 3-D view of the Shaddad tracked platform: body, moving tracks, implement, follow camera.
    %
    %   Simulation-only MATLAB System block (Interpreted execution) in Viz_Logging/Viz_Variant/Robot3D
    %   (VC_VIZ == 3), and the offline replay shaddad.Robot3DView.replay(file). Simscape Multibody and
    %   Simulink 3D Animation are not installed, so the robot is drawn with MATLAB graphics: the chassis
    %   (red body, deck rack, front display, E-stop, lights), both rubber tracks with tread bars that move
    %   with the measured belt speeds, rotating sprockets / idlers / road wheels, the mounted implement
    %   (mower, tiller or sprayer), the trail, a follow camera and a heads-up display (time, MCU mode,
    %   speed, SOC, RC sticks and switches).
    %
    %   Inputs: 1 pos_W [x y z] of the reference point R (ENU, on the ground under the track centre)
    %           2 R_WB 3x3 DCM body (FLU) -> world (ENU)
    %           3 vBelt_LR [L R] belt speeds [m/s], + = forward
    %           4 mode (uint8 ShaddadMode)   5 SOC (0..1)   6 speed_fwd [m/s]
    %           7 rc_ch_norm (16 normalised RC channels, -1..1)
    %
    %   obj = shaddad.Robot3DView('Implement', 1);  obj(pos, R, vb, mode, soc, v, rc)   % one frame
    %   shaddad.Robot3DView.replay(res.file)                     % replay a saved run in 3-D
    %   shaddad.Robot3DView.replay(res.file, 'Speed', 2, 'Video', 'drive.mp4', 'Camera', 'side')

    properties (Nontunable)
        Implement   = 0             % 0 none, 1 mower, 2 tiller, 3 sprayer (VC_IMPL)
        SampleTime  = 0.05          % s
        Camera      = 1             % 1 chase | 2 side | 3 top | 4 free (mouse); MATLAB calls may also pass the names
        ModelName   = 'Shaddad_System'  % model stopped by the Stop button (live runs)
        Obstacles   = [0 0 0]       % [x y r] rows: static cylinders (e.g. palm trunks) drawn 2.5 m high; r = 0 rows are skipped
        Visible     = true
    end

    properties (Access = private)
        hFig; hAx; hRobot; hTread; hWheel; wheelC; treadFaces; hTrail; trail; kTrail
        hGrid; hHud; hModeBox; hStickDot; hStick2Dot; hSw; sL; sR; t; camPos; camTgt; loop; camMode; stopReq; acc
    end

    properties (Constant, Hidden)
        ModeNames = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', ...
                     'CTRL_STOP', 'ESTOP', 'FAULT'};
        ModeColors = [0.5 0.5 0.5; 0.5 0.5 0.5; 0.45 0.45 0.55; 0.12 0.6 0.25; 0.85 0.65 0.1; 0.2 0.4 0.85; ...
                      0.2 0.4 0.85; 0.95 0.5 0.1; 0.85 0.1 0.1; 0.6 0 0.2];
        % geometry (body FLU, origin R on the ground under the track centre), from params/param_structure.m
        % (real robot, docs/hardware/real_robot_components.md: track 0.94 m, gauge 0.60, width 0.13, sprocket 0.09)
        Ltr = 0.94; Htr = 0.26; B = 0.60; b = 0.13; nTread = 24
    end

    methods
        function obj = Robot3DView(varargin)
            setProperties(obj, nargin, varargin{:});
        end

        function f = figureHandle(obj)
            f = obj.hFig;
        end

        function tf = stopRequested(obj)
            %STOPREQUESTED  True after the Stop button (or Esc) was pressed.
            tf = isequal(obj.stopReq, true);
        end
    end

    methods (Static)
        function replay(file, varargin)
            %REPLAY  Play a saved run (work/results/*.mat from run_scenario / phase1_test_drive) in 3-D.
            %   shaddad.Robot3DView.replay(file, 'Speed', s, 'Camera', c, 'Implement', k, 'Video', mp4, 'Step', n)
            ip = inputParser;
            ip.addParameter('Speed', 1);           % 1 = real time, 2 = twice as fast, Inf = as fast as possible
            ip.addParameter('Camera', 1);
            ip.addParameter('Implement', []);
            ip.addParameter('Video', '');
            ip.addParameter('Ts', 0.05);
            ip.parse(varargin{:});
            o = ip.Results;
            if isstruct(file), R = file; else, R = load(file); end
            S = R.S;
            tr = S.truth;  te = S.telem;
            tq = (0:o.Ts:tr.pos_W.t(end))';
            itp = @(sig) interp1(double(sig.t(:)), double(sig.v), tq, 'previous', 'extrap');
            pos = itp(tr.pos_W);  vb = itp(tr.vBelt_LR);  v = itp(tr.speed_fwd);
            mode = itp(te.mode);  soc = itp(te.SOC);  rc = itp(te.rc_ch_norm);
            Rt = double(tr.R_WB.v);  tR = double(tr.R_WB.t(:));
            if ndims(Rt) == 3 && size(Rt, 3) == numel(tR)          % 3 x 3 x N -> N x 9 (column-major)
                Rt = reshape(Rt, 9, numel(tR))';
            end
            iR = min(max(1, round(interp1(tR, (1:numel(tR))', tq, 'nearest', 'extrap'))), numel(tR));
            impl = o.Implement;
            if isempty(impl)
                impl = 0;
                try, impl = double(R.in.getVariable('VC_IMPL')); catch, end %#ok<NOCOM>
            end
            obj = shaddad.Robot3DView('Implement', impl, 'SampleTime', o.Ts, 'Camera', o.Camera);
            vw = [];
            if ~isempty(o.Video)
                vw = VideoWriter(o.Video, 'MPEG-4'); vw.FrameRate = round(1/o.Ts); open(vw);
            end
            t0 = tic;
            for k = 1:numel(tq)
                obj(pos(k, :)', reshape(Rt(iR(k), :), 3, 3), vb(k, :)', uint8(mode(k)), soc(k), v(k), rc(k, :)');
                f = obj.figureHandle;
                if isempty(f) || ~isvalid(f) || obj.stopRequested, break, end
                if ~isempty(vw), writeVideo(vw, getframe(f)); end
                if isfinite(o.Speed)
                    lag = tq(k)/o.Speed - toc(t0);
                    if lag > 0, pause(lag); end
                end
            end
            if ~isempty(vw), close(vw); fprintf('Video written: %s\n', o.Video); end
        end
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 7;
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function setupImpl(obj, varargin)
            vis = 'on';  if ~obj.Visible, vis = 'off'; end
            obj.hFig = figure('Name', 'Shaddad 3-D view', 'NumberTitle', 'off', 'Visible', vis, 'Color', [0.93 0.95 0.98], ...
                'Position', [80 80 960 600]);
            obj.hAx = axes(obj.hFig, 'Position', [0 0 1 1], 'Visible', 'off', 'Projection', 'perspective', ...
                'DataAspectRatio', [1 1 1], 'Clipping', 'off');
            hold(obj.hAx, 'on');
            % ground, grid (re-centred on the robot) and obstacles
            patch(obj.hAx, 'XData', [-500 500 500 -500], 'YData', [-500 -500 500 500], 'ZData', -0.012*[1 1 1 1], ...
                'FaceColor', [0.80 0.78 0.66], 'EdgeColor', 'none');
            obj.hGrid = hgtransform('Parent', obj.hAx);
            g = -40:1:40;  gz = zeros(size(g));
            X = [g; g; nan(size(g))];  Y = [-40*ones(size(g)); 40*ones(size(g)); nan(size(g))];
            line('Parent', obj.hGrid, 'XData', [X(:); Y(:)], 'YData', [Y(:); X(:)], 'ZData', 0.004 + [gz gz gz gz gz gz]', 'Color', [0.68 0.66 0.55], 'LineWidth', 0.5);
            for k = 1:size(obj.Obstacles, 1)
                if obj.Obstacles(k, 3) <= 0, continue, end
                [cx, cy, cz] = cylinder(obj.Obstacles(k, 3), 16);
                surf(obj.hAx, cx + obj.Obstacles(k, 1), cy + obj.Obstacles(k, 2), 2.5*cz, 'FaceColor', [0.45 0.32 0.2], ...
                    'EdgeColor', 'none');
            end
            obj.trail = nan(3000, 3);  obj.kTrail = 0;
            obj.hTrail = line(obj.hAx, nan, nan, nan, 'Color', [0.17 0.19 0.5], 'LineWidth', 1.5);
            % robot
            obj.hRobot = hgtransform('Parent', obj.hAx);
            obj.buildRobot();
            light(obj.hAx, 'Position', [30 -40 60], 'Style', 'infinite');
            light(obj.hAx, 'Position', [-30 40 30], 'Style', 'infinite', 'Color', [0.45 0.45 0.5]);
            obj.buildHud();
            camva(obj.hAx, 38);
            obj.camPos = [];  obj.camTgt = [];
            obj.sL = 0;  obj.sR = 0;  obj.t = 0;  obj.stopReq = false;
            cams = {'chase', 'side', 'top', 'free'};
            if isnumeric(obj.Camera), obj.camMode = cams{max(1, min(4, round(obj.Camera)))}; else, obj.camMode = char(obj.Camera); end
            if ~ismember(obj.camMode, cams), obj.camMode = 'chase'; end
            obj.buildButtons(cams);
            if strcmp(obj.camMode, 'free')
                view(obj.hAx, -40, 25);  rotate3d(obj.hFig, 'on');
            end
        end

        function stepImpl(obj, pos, R, vb, mode, soc, v, rc)
            if isempty(obj.hFig) || ~isvalid(obj.hFig)
                return                                   % window closed by the user
            end
            pos = double(pos(:));  R = reshape(double(R), 3, 3);  vb = double(vb(:));  rc = double(rc(:));
            if ~all(isfinite(R(:))) || abs(det(R) - 1) > 0.1, R = eye(3); end   % initial zero sample of the RT
            if ~all(isfinite(pos)), pos = zeros(3, 1); end
            vb(~isfinite(vb)) = 0;
            dt = obj.SampleTime;
            obj.t = obj.t + dt;
            obj.hRobot.Matrix = [R pos; 0 0 0 1];
            % tracks: tread bars and wheels advance with the belt travel
            obj.sL = obj.sL + vb(1)*dt;  obj.sR = obj.sR + vb(2)*dt;
            obj.hTread(1).Vertices = obj.treadVerts(obj.sL, +obj.B/2);
            obj.hTread(2).Vertices = obj.treadVerts(obj.sR, -obj.B/2);
            s = [obj.sL obj.sR];
            for side = 1:2
                for j = 1:size(obj.wheelC, 1)
                    c = obj.wheelC(j, :);
                    obj.hWheel(side, j).Matrix = makehgtform('translate', [c(1) 0 c(2)], 'yrotate', s(side)/c(3));
                end
            end
            % trail and grid
            obj.kTrail = mod(obj.kTrail, size(obj.trail, 1)) + 1;
            obj.trail(obj.kTrail, :) = [pos(1) pos(2) pos(3) + 0.01];
            tr = [obj.trail(obj.kTrail + 1:end, :); obj.trail(1:obj.kTrail, :)];
            set(obj.hTrail, 'XData', tr(:, 1), 'YData', tr(:, 2), 'ZData', tr(:, 3));
            obj.hGrid.Matrix = makehgtform('translate', [10*round(pos(1)/10) 10*round(pos(2)/10) 0]);
            % camera
            psi = atan2(R(2, 1), R(1, 1));
            hd = [cos(psi); sin(psi); 0];
            tgt = pos + [0; 0; 0.32];                % camera distances sized for the 1.0 m robot
            switch obj.camMode
                case 'chase', cp = pos - 3.4*hd + [0; 0; 1.4];  tgt = pos + 1.1*hd + [0; 0; 0.2];
                case 'side',  cp = tgt + 3.9*[-sin(psi); cos(psi); 0] + [0; 0; 0.85];
                case 'top',   cp = tgt - 0.4*hd + [0; 0; 6.5];
                otherwise,    cp = [];
            end
            if ~isempty(cp)
                if isempty(obj.camPos), obj.camPos = cp; obj.camTgt = tgt; end
                a = 0.25;
                obj.camPos = (1 - a)*obj.camPos + a*cp;  obj.camTgt = (1 - a)*obj.camTgt + a*tgt;
                campos(obj.hAx, obj.camPos');  camtarget(obj.hAx, obj.camTgt');  camup(obj.hAx, [0 0 1]);
            elseif obj.t <= dt
                camtarget(obj.hAx, tgt');
            end
            obj.updateHud(double(mode), double(soc), double(v), vb, rc);
            drawnow limitrate
        end

        function releaseImpl(~)
            % keep the figure open for inspection
        end
    end

    methods (Access = private)
        % ------------------------------------------------------------------ robot geometry
        function buildRobot(obj)
            P = obj.hRobot;
            r = obj.Htr/2;  a = obj.Ltr/2 - r;
            obj.loop = struct('r', r, 'a', a, 'P', 4*a + 2*pi*r);
            rubber = [0.13 0.13 0.14];
            for side = [1 -1]
                yc = side*obj.B/2;
                [x, z] = obj.loopPts(linspace(0, obj.loop.P, 90));
                n = numel(x);
                y1 = yc - obj.b/2;  y2 = yc + obj.b/2;
                V = [x' y1*ones(n, 1) z'; x' y2*ones(n, 1) z'];
                obj.addFaces(V, 1:n, rubber);
                obj.addFaces(V, n + (1:n), rubber);
                obj.addFaces(V, [(1:n - 1)' (2:n)' (n + 2:2*n)' (n + 1:2*n - 1)'], rubber*0.8);
                % track frame (side plate) between the wheels
                obj.box(P, [-a a], yc + side*(obj.b/2 + 0.004) + [-0.003 0.003], [0.06 0.20], [0.22 0.22 0.24]);
            end
            % tread bars (one patch per side, vertices recomputed every frame)
            N = obj.nTread;
            f1 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
            obj.treadFaces = zeros(6*N, 4);
            for k = 1:N, obj.treadFaces(6*(k - 1) + (1:6), :) = f1 + 8*(k - 1); end
            obj.hTread = gobjects(1, 2);
            obj.hTread(1) = patch('Parent', P, 'Vertices', obj.treadVerts(0, +obj.B/2), 'Faces', obj.treadFaces, ...
                'FaceColor', [0.05 0.05 0.05], 'EdgeColor', 'none');
            obj.hTread(2) = patch('Parent', P, 'Vertices', obj.treadVerts(0, -obj.B/2), 'Faces', obj.treadFaces, ...
                'FaceColor', [0.05 0.05 0.05], 'EdgeColor', 'none');
            % wheels on the outer face: sprocket (rear), idler (front), 3 road wheels [x z radius]
            obj.wheelC = [-a r 0.090; a r 0.090];                       % animated: sprocket, idler [x z radius]
            obj.hWheel = gobjects(2, size(obj.wheelC, 1));
            sides = [1 -1];
            th = linspace(0, 2*pi, 24);
            for si = 1:2
                yo = sides(si)*(obj.B/2 + obj.b/2 + 0.012);
                for xr = linspace(-a + 0.08, a - 0.08, 3)                 % road wheels (static)
                    obj.addFaces([xr + 0.077*cos(th') yo*ones(numel(th), 1) 0.082 + 0.077*sin(th')], 1:numel(th), [0.92 0.92 0.94]);
                end
                for j = 1:size(obj.wheelC, 1)
                    h = hgtransform('Parent', P);
                    rw = obj.wheelC(j, 3);
                    patch('Parent', h, 'XData', rw*cos(th), 'YData', yo*ones(size(th)), 'ZData', rw*sin(th), ...
                        'FaceColor', [0.62 0.63 0.66], 'EdgeColor', [0.3 0.3 0.32]);
                    for sp = 0:2                          % spokes show the rotation
                        ang = sp*2*pi/3;
                        line('Parent', h, 'XData', [0 0.85*rw*cos(ang)], 'YData', yo*[1 1] + sides(si)*0.002, 'ZData', [0 0.85*rw*sin(ang)], ...
                            'Color', [0.15 0.15 0.15], 'LineWidth', 2);
                    end
                    obj.hWheel(si, j) = h;
                end
            end
            % chassis frame between the tracks, red body (chamfered front), deck with rail, display, E-stop, lights
            % (XMachines MAX: 0.78 m wide, 0.57 m high over the rail)
            obj.box(P, [-0.45 0.45], [-0.235 0.235], [0.12 0.30], [0.2 0.2 0.22]);
            bx = [-0.48 0.42 0.48 0.48 0.44 -0.48];  bz = [0.30 0.30 0.34 0.44 0.47 0.47];
            obj.prism(P, bx, bz, [-0.26 0.26], [0.86 0.13 0.1]);
            obj.box(P, [-0.50 0.50], [-0.39 0.39], [0.47 0.49], [0.08 0.08 0.09]);
            for xx = [-0.47 0.47], for yy = [-0.36 0.36]
                obj.box(P, xx + [-0.015 0.015], yy + [-0.015 0.015], [0.49 0.57], [0.08 0.08 0.09]);
            end, end
            obj.box(P, [-0.49 0.49], [0.345 0.375], [0.555 0.57], [0.08 0.08 0.09]);
            obj.box(P, [-0.49 0.49], [-0.375 -0.345], [0.555 0.57], [0.08 0.08 0.09]);
            obj.box(P, [-0.49 -0.46], [-0.375 0.375], [0.555 0.57], [0.08 0.08 0.09]);
            obj.box(P, [0.46 0.49], [-0.375 0.375], [0.555 0.57], [0.08 0.08 0.09]);
            obj.box(P, [0.482 0.488], [-0.12 0.12], [0.34 0.43], [0.04 0.06 0.09]);           % display
            obj.box(P, [0.482 0.490], [0.17 0.23], [0.33 0.36], [1 1 0.92]);                  % lights
            obj.box(P, [0.482 0.490], [-0.23 -0.17], [0.33 0.36], [1 1 0.92]);
            obj.box(P, [0.482 0.496], [-0.24 -0.18], [0.39 0.45], [0.98 0.82 0.1]);           % E-stop base
            [cy, cz, cx] = cylinder(0.025, 20);
            surface('Parent', P, 'XData', 0.496 + 0.03*cx, 'YData', cy - 0.21, 'ZData', cz + 0.42, 'FaceColor', [0.85 0.05 0.05], 'EdgeColor', 'none');
            % implement
            switch obj.Implement
                case 1      % mower deck (front)
                    obj.box(P, [0.50 1.00], [-0.40 0.40], [0.04 0.22], [0.18 0.30 0.20]);
                    obj.box(P, [0.48 0.56], [-0.20 -0.15], [0.22 0.34], [0.2 0.2 0.22]);
                    obj.box(P, [0.48 0.56], [0.15 0.20], [0.22 0.34], [0.2 0.2 0.22]);
                case 2      % tiller (rear)
                    obj.box(P, [-0.90 -0.52], [-0.30 0.30], [0.08 0.40], [0.92 0.52 0.1]);
                    obj.box(P, [-0.55 -0.49], [-0.06 0.06], [0.25 0.38], [0.2 0.2 0.22]);
                case 3      % sprayer module: 40 L tank on the deck, 1.2 m boom with 8 nozzles at the rear
                    obj.box(P, [-0.35 0.15], [-0.20 0.20], [0.49 0.69], [0.93 0.94 0.96]);
                    obj.box(P, [-0.58 -0.55], [-0.60 0.60], [0.30 0.33], [0.25 0.25 0.28]);
                    obj.box(P, [-0.55 -0.49], [-0.03 0.03], [0.30 0.49], [0.25 0.25 0.28]);
                    for yy = linspace(-0.525, 0.525, 8)
                        obj.box(P, [-0.575 -0.555], yy + [-0.012 0.012], [0.25 0.30], [0.1 0.35 0.75]);
                    end
            end
            obj.flushStatic(P);
            material(obj.hAx, 'dull');
        end

        function addFaces(obj, V, F, col)
            % collect static geometry; flushStatic draws it as one patch (fewer graphics objects, faster frames)
            if isempty(obj.acc), obj.acc = struct('V', {{}}, 'F', {{}}, 'C', {{}}, 'n', 0); end
            obj.acc.V{end + 1} = V;
            obj.acc.F{end + 1} = F + obj.acc.n;
            obj.acc.C{end + 1} = repmat(col, size(F, 1), 1);
            obj.acc.n = obj.acc.n + size(V, 1);
        end

        function flushStatic(obj, P)
            w = max(cellfun(@(f) size(f, 2), obj.acc.F));
            F = cellfun(@(f) [f nan(size(f, 1), w - size(f, 2))], obj.acc.F, 'UniformOutput', false);
            patch('Parent', P, 'Vertices', vertcat(obj.acc.V{:}), 'Faces', vertcat(F{:}), ...
                'FaceVertexCData', vertcat(obj.acc.C{:}), 'FaceColor', 'flat', 'EdgeColor', 'none', 'FaceLighting', 'flat');
            obj.acc = [];
        end

        function [x, z, th] = loopPts(obj, s)
            % track loop: bottom run from front to rear, rear arc up, top run forward, front arc down
            r = obj.loop.r;  a = obj.loop.a;  Pm = obj.loop.P;
            s = mod(s, Pm);
            x = zeros(size(s));  z = x;  th = x;
            L1 = 2*a;  L2 = L1 + pi*r;  L3 = L2 + 2*a;
            k = s < L1;               x(k) = a - s(k);        z(k) = 0;            th(k) = -pi/2;
            k = s >= L1 & s < L2;     u = (s(k) - L1)/r;      x(k) = -a - r*sin(u); z(k) = r - r*cos(u); th(k) = -pi/2 - u;
            k = s >= L2 & s < L3;     x(k) = -a + (s(k) - L2); z(k) = 2*r;         th(k) = pi/2;
            k = s >= L3;              u = (s(k) - L3)/r;      x(k) = a + r*sin(u);  z(k) = r + r*cos(u); th(k) = pi/2 - u;
        end

        function V = treadVerts(obj, s0, yc)
            % tread bars along the loop; a positive belt travel moves the top run forward
            N = obj.nTread;  Pm = obj.loop.P;
            s = mod(s0 + (0:N - 1)*Pm/N, Pm);
            [x, z, th] = obj.loopPts(s);
            nx = cos(th);  nz = sin(th);                 % outward normal
            tx = -nz;  tz = nx;                          % tangent
            hl = 0.018;  t1 = 0.0;  t2 = 0.022;  y1 = yc - obj.b/2 - 0.004;  y2 = yc + obj.b/2 + 0.004;
            V = zeros(8*N, 3);
            for k = 1:N
                c0 = [x(k) z(k)];  tt = [tx(k) tz(k)];  nn = [nx(k) nz(k)];
                q = [c0 - hl*tt + t1*nn; c0 + hl*tt + t1*nn; c0 + hl*tt + t2*nn; c0 - hl*tt + t2*nn];
                V(8*(k - 1) + (1:8), :) = [q(:, 1) y1*ones(4, 1) q(:, 2); q(:, 1) y2*ones(4, 1) q(:, 2)];
            end
        end

        function box(obj, ~, xr, yr, zr, col)
            [X, Y, Z] = ndgrid(xr, yr, zr);
            obj.addFaces([X(:) Y(:) Z(:)], [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6], col);
        end

        function prism(obj, ~, px, pz, yr, col)
            n = numel(px);
            V = [px' yr(1)*ones(n, 1) pz'; px' yr(2)*ones(n, 1) pz'];
            obj.addFaces(V, 1:n, col);
            obj.addFaces(V, n + (1:n), col);
            obj.addFaces(V, [(1:n)' [2:n 1]' [n + 2:2*n n + 1]' (n + 1:2*n)'], col*0.92);
        end

        % ------------------------------------------------------------------ buttons
        function buildButtons(obj, cams)
            uicontrol(obj.hFig, 'Style', 'pushbutton', 'String', 'Stop', 'FontWeight', 'bold', 'FontSize', 11, ...
                'BackgroundColor', [0.85 0.15 0.1], 'ForegroundColor', 'w', 'Units', 'normalized', ...
                'Position', [0.89 0.925 0.095 0.055], 'Callback', @(~, ~) obj.onStop());
            for k = 1:numel(cams)
                uicontrol(obj.hFig, 'Style', 'pushbutton', 'String', [upper(cams{k}(1)) cams{k}(2:end)], 'FontSize', 10, ...
                    'Units', 'normalized', 'Position', [0.685 + 0.075*(k - 1) 0.865 0.07 0.045], ...
                    'Callback', @(~, ~) obj.onCamera(cams{k}));
            end
            set(obj.hFig, 'KeyPressFcn', @(~, e) obj.onKey(e), 'CloseRequestFcn', @(f, ~) obj.onClose(f));
        end

        function onStop(obj)
            obj.stopReq = true;
            try
                if bdIsLoaded(obj.ModelName) && ~strcmp(get_param(obj.ModelName, 'SimulationStatus'), 'stopped')
                    set_param(obj.ModelName, 'SimulationCommand', 'stop');
                end
            catch
            end
        end

        function onCamera(obj, c)
            obj.camMode = c;
            obj.camPos = [];
            if strcmp(c, 'free'), rotate3d(obj.hFig, 'on'); else, rotate3d(obj.hFig, 'off'); end
        end

        function onKey(obj, e)
            switch e.Key
                case 'escape', obj.onStop();
                case {'1', '2', '3', '4'}
                    cams = {'chase', 'side', 'top', 'free'};
                    obj.onCamera(cams{str2double(e.Key)});
            end
        end

        function onClose(obj, f)
            obj.onStop();
            delete(f);
        end

        % ------------------------------------------------------------------ heads-up display
        function buildHud(obj)
            h = axes(obj.hFig, 'Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1], 'HitTest', 'off');
            hold(h, 'on');
            obj.hModeBox = rectangle(h, 'Position', [0.015 0.925 0.15 0.055], 'Curvature', 0.2, ...
                'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
            obj.hHud = gobjects(1, 3);
            obj.hHud(1) = text(h, 0.09, 0.952, 'BOOT', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 13, ...
                'HorizontalAlignment', 'center');
            obj.hHud(2) = text(h, 0.18, 0.952, '', 'FontSize', 12, 'Color', [0.1 0.1 0.2], 'FontName', 'Consolas');
            obj.hHud(3) = text(h, 0.015, 0.03, ['PS5: Cross arm, Circle E-stop, Triangle reset, R1/L1 speed, left stick ' ...
                'throttle, right stick steer  |  RC: CH6 arm, CH12 E-stop, CH9 reset'], 'FontSize', 9, 'Color', [0.3 0.3 0.4]);
            % stick panels (right stick: steer / throttle; left stick: CH4 / CH3)
            for kk = 1:2
                x0 = 0.84 - (kk - 1)*0.13;
                rectangle(h, 'Position', [x0 0.06 0.11 0.17], 'FaceColor', [1 1 1], 'EdgeColor', [0.4 0.4 0.5]);
                plot(h, x0 + 0.055*[1 1], [0.07 0.22], ':', 'Color', [0.5 0.5 0.6]);
                plot(h, x0 + [0.005 0.105], 0.145*[1 1], ':', 'Color', [0.5 0.5 0.6]);
            end
            text(h, 0.895, 0.245, 'CH1 / CH2', 'HorizontalAlignment', 'center', 'FontSize', 9);
            text(h, 0.765, 0.245, 'CH4 / CH3', 'HorizontalAlignment', 'center', 'FontSize', 9);
            obj.hStickDot = plot(h, 0.895, 0.145, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.17 0.19 0.5], 'MarkerEdgeColor', 'w');
            obj.hStick2Dot = plot(h, 0.765, 0.145, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.43 0.31 0.64], 'MarkerEdgeColor', 'w');
            obj.hSw = text(h, 0.62, 0.27, '', 'FontSize', 10, 'FontName', 'Consolas', 'Color', [0.1 0.1 0.2]);
        end

        function updateHud(obj, mode, soc, v, vb, rc)
            m = mode + 1;
            if m >= 1 && m <= numel(obj.ModeNames)
                obj.hHud(1).String = obj.ModeNames{m};
                obj.hModeBox.FaceColor = obj.ModeColors(m, :);
            end
            obj.hHud(2).String = sprintf('t %6.1f s   v %5.2f m/s   belts L %5.2f  R %5.2f m/s   SOC %5.1f %%', ...
                obj.t, v, vb(1), vb(2), 100*soc);
            if numel(rc) >= 4
                obj.hStickDot.XData = 0.895 + 0.05*max(-1, min(1, rc(1)));
                obj.hStickDot.YData = 0.145 + 0.07*max(-1, min(1, rc(2)));
                obj.hStick2Dot.XData = 0.765 + 0.05*max(-1, min(1, rc(4)));
                obj.hStick2Dot.YData = 0.145 + 0.07*max(-1, min(1, rc(3)));
            end
            if numel(rc) >= 12
                pos3 = {'MANUAL', 'HOLD', 'AUTO'};
                sw = pos3{min(3, max(1, round(rc(5)) + 2))};
                onoff = {'off', 'ON'};
                obj.hSw.String = sprintf('CH5 %-6s  CH6 arm %-3s  CH12 E-stop %s', sw, onoff{1 + (rc(6) > 0.5)}, ...
                    onoff{1 + (rc(12) > 0.5)});
            end
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
