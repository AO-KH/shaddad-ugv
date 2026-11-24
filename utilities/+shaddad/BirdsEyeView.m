classdef BirdsEyeView < matlab.System
    %BIRDSEYEVIEW  Live 2-D bird's-eye view of the Shaddad UGV (ENU, map frame = world).
    %
    %   Simulation-only MATLAB System block (Interpreted execution), Viz_Logging/Viz_Variant/BEV
    %   (VC_VIZ == 1), sample time P.viz.Ts = 0.1 s (integration spec 5.1). The offline replay
    %   utilities/shaddad_replay_birdseye.m uses the same class, so live and replay drawing share one
    %   code path. No installed block draws a ground robot's oriented footprint over a 2-D field map
    %   with the planned path, the LiDAR returns and the trail (UAV Animation: 3-D UAV mesh, XY Graph:
    %   points / lines only), hence this System object.
    %
    %   Inputs: 1 pose_true [x; y; psi] (double 3), 2 pose_est [x; y; psi] (double 3),
    %           3 ranges_B (360, NaN = no return), 4 angles_B (360, rad CCW from body x),
    %           5 mode (uint8 ShaddadMode), 6 soc (0..1)
    %
    %   obj = shaddad.BirdsEyeView('MapImage', P.world.occ.grid, 'MapResolution', P.world.occ.res, ...
    %         'MapOrigin', P.world.occ.origin, 'PathXY', shaddad.BirdsEyeView.planPath(P.gnc.mission));
    %   obj(pT, pE, ranges, angles, mode, soc)          % one frame (MATLAB or Simulink)

    properties (Nontunable)
        MapImage      = false(600, 1200)   % logical occupancy grid (row 1 = max y), P.world.occ.grid
        MapResolution = 10                 % cells per metre, P.world.occ.res
        MapOrigin     = [0 0]              % [x y] of the lower-left grid corner in ENU [m]
        Footprint     = [1.0 0.78]          % [L W] m, P.viz.footprint
        TrailLength   = 20000              % trail points, P.viz.trail_len
        ScanDecim     = 4                  % plot every n-th beam, P.viz.scan_decim
        SampleTime    = 0.1                % s, P.viz.Ts
        PathXY        = nan(1, 2)          % planned path [x y] rows, NaN rows separate segments
        Visible       = true               % false: off-screen figure (tests, video export)
    end

    properties (Access = private)
        hFig; hAx; hBody; hTrail; hEst; hPath; hScan; hTxt; buf; k; nFrames
    end

    properties (Constant, Hidden)
        ModeNames = {'BOOT', 'SELFTEST', 'STANDBY', 'MANUAL', 'HOLD', 'AUTO_START', 'AUTO', ...
                     'CTRL_STOP', 'ESTOP', 'FAULT'};
    end

    methods
        function obj = BirdsEyeView(varargin)
            setProperties(obj, nargin, varargin{:});
        end

        function f = figureHandle(obj)
            %FIGUREHANDLE  Figure of the view (for getframe / VideoWriter in the replay).
            f = obj.hFig;
        end

        function n = frameCount(obj)
            n = obj.nFrames;
        end
    end

    methods (Static)
        function xy = planPath(plan)
            %PLANPATH  Bus_MissionPlan -> [x y] rows of all used segments, NaN row between segments.
            xy = nan(1, 2);
            if ~isstruct(plan) || ~isfield(plan, 'path') || ~plan.valid || plan.n_seg < 1
                return
            end
            n = double(plan.n_seg);
            nr = size(plan.path, 1) / size(plan.seg, 1);       % rows per segment (100)
            xy = nan(n*(nr + 1), 2);
            for s = 1:n
                rows = (s - 1)*nr + (1:nr);
                xy((s - 1)*(nr + 1) + (1:nr), :) = double(plan.path(rows, 1:2));
            end
        end
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 6;
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function setupImpl(obj, varargin)
            vis = 'on';
            if ~obj.Visible, vis = 'off'; end
            obj.hFig = figure('Name', 'Shaddad bird''s-eye view', 'NumberTitle', 'off', 'Visible', vis, ...
                'Color', 'w');
            obj.hAx = axes(obj.hFig);
            hold(obj.hAx, 'on'); axis(obj.hAx, 'equal'); grid(obj.hAx, 'on');
            xlabel(obj.hAx, 'x_E [m]'); ylabel(obj.hAx, 'y_N [m]');
            % map raster (row 1 = max y)
            M = logical(obj.MapImage);
            [ny, nx] = size(M);
            r = obj.MapResolution;  o = obj.MapOrigin;
            if nx > 1 && ny > 1
                xd = o(1) + [0.5, nx - 0.5]/r;
                yd = o(2) + [ny - 0.5, 0.5]/r;
                image(obj.hAx, 'XData', xd, 'YData', yd, 'CData', repmat(1 - 0.6*double(M), [1 1 3]));
                set(obj.hAx, 'YDir', 'normal');
                xlim(obj.hAx, o(1) + [0 nx]/r); ylim(obj.hAx, o(2) + [0 ny]/r);
            end
            % planned path, trail, estimate, scan
            obj.hPath  = plot(obj.hAx, obj.PathXY(:, 1), obj.PathXY(:, 2), '-', 'Color', [0.2 0.6 0.2], 'LineWidth', 0.8);
            obj.hTrail = plot(obj.hAx, nan, nan, '-', 'Color', [0 0.3 0.8], 'LineWidth', 1.2);
            obj.hEst   = plot(obj.hAx, nan, nan, 'o', 'Color', [0.85 0.33 0.1], 'MarkerSize', 6);
            obj.hScan  = plot(obj.hAx, nan, nan, '.', 'Color', [0.8 0 0], 'MarkerSize', 4);
            % body: footprint rectangle + heading triangle in an hgtransform
            obj.hBody = hgtransform('Parent', obj.hAx);
            L = obj.Footprint(1);  W = obj.Footprint(2);
            patch('Parent', obj.hBody, 'XData', [-L L L -L]/2, 'YData', [-W -W W W]/2, ...
                'FaceColor', [1 0.85 0.3], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
            patch('Parent', obj.hBody, 'XData', [L/2 L/6 L/6], 'YData', [0 W/4 -W/4], 'FaceColor', 'k');
            obj.hTxt = title(obj.hAx, '');
            obj.buf = nan(obj.TrailLength, 2);
            obj.k = 0;
            obj.nFrames = 0;
        end

        function stepImpl(obj, pT, pE, ranges, angles, mode, soc)
            if isempty(obj.hFig) || ~isvalid(obj.hFig)
                return                                   % window closed by the user
            end
            pT = double(pT(:));  pE = double(pE(:));
            obj.hBody.Matrix = makehgtform('translate', [pT(1) pT(2) 0], 'zrotate', pT(3));
            obj.k = mod(obj.k, obj.TrailLength) + 1;
            obj.buf(obj.k, :) = pT(1:2)';
            tr = [obj.buf(obj.k + 1:end, :); obj.buf(1:obj.k, :)];
            set(obj.hTrail, 'XData', tr(:, 1), 'YData', tr(:, 2));
            set(obj.hEst, 'XData', pE(1), 'YData', pE(2));
            rg = double(ranges(:));  an = double(angles(:));
            idx = 1:max(1, round(obj.ScanDecim)):numel(rg);
            rg = rg(idx);  an = an(idx);
            ok = isfinite(rg);
            xb = rg(ok).*cos(an(ok));  yb = rg(ok).*sin(an(ok));
            c = cos(pT(3));  s = sin(pT(3));
            set(obj.hScan, 'XData', pT(1) + c*xb - s*yb, 'YData', pT(2) + s*xb + c*yb);
            m = double(mode) + 1;
            if m >= 1 && m <= numel(obj.ModeNames), mn = obj.ModeNames{m}; else, mn = '?'; end
            obj.nFrames = obj.nFrames + 1;
            obj.hTxt.String = sprintf('t = %.1f s   %s   SoC %.1f %%   |e_{pos}| = %.2f m', ...
                obj.nFrames*obj.SampleTime, mn, 100*double(soc), norm(pT(1:2) - pE(1:2)));
            drawnow limitrate
        end

        function releaseImpl(~)
            % keep the figure for inspection
        end

        function flag = isInactivePropertyImpl(~, ~)
            flag = false;
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
