function M = gnc_mission_metrics(r, what, varargin)
%GNC_MISSION_METRICS  Post-processing of gnc_mission_run results (T15 test helper).
%
%   c = gnc_mission_metrics(r, 'clearance', obs)   minimum distance between the body rectangle (P.gnc.geom L x W,
%                                                  centred on base_link) and obstacles obs = dyn rows [type x y a b yaw]
%                                                  over the run (truth poses)
%   e = gnc_mission_metrics(r, 'xte', k)           truth cross-track error to the raw path of segment k (+ = left),
%                                                  per truth sample (NaN beyond the segment ends)
%   t = gnc_mission_metrics(r, 'tfirst', field, cond)   first time (fsm 0.05 s grid) where cond(r.(field)) holds
%   L = gnc_mission_metrics(r, 'transitions')      [t state sub trans_id] rows at every change of the FSM output

switch what
    case 'clearance'
        obs = varargin{1};
        Lb = r.P.gnc.geom.L;  Wb = r.P.gnc.geom.W;
        M = inf;
        for k = 1:size(r.truth, 1)
            p = r.truth(k, 1:3);
            c = [cos(p(3)) sin(p(3))];  s = [-c(2) c(1)];
            for j = 1:size(obs, 1)
                if obs(j, 1) < 1, continue; end
                M = min(M, local_rect_obs(p(1:2), c, s, Lb/2, Wb/2, obs(j, :)));
            end
        end
    case 'xte'
        k = varargin{1};
        N = r.P.gnc.path.N_raw;
        q = double(r.plan.path((k - 1)*N + (1:N), 1:2));
        M = nan(size(r.truth, 1), 1);
        for i = 1:size(r.truth, 1)
            p = r.truth(i, 1:2);
            best = inf;
            for j = 1:N - 1
                a = q(j, :);  u = q(j + 1, :) - a;  L2 = u*u';
                if L2 < 1e-12, continue; end
                t = (p - a)*u'/L2;
                if (t < 0 && j > 1) || (t > 1 && j < N - 1), continue; end
                if t < 0 || t > 1, continue; end
                d = p - (a + t*u);
                if norm(d) < abs(best)
                    best = sign(u(1)*d(2) - u(2)*d(1))*norm(d);
                end
            end
            if isfinite(best), M(i) = best; end
        end
    case 'tfirst'
        f = varargin{1};  cond = varargin{2};
        x = r.(f);
        i = find(cond(x), 1);
        if isempty(i), M = NaN; else, M = r.tf(i); end
    case 'transitions'
        i = [1; find(diff(r.f_state) ~= 0 | diff(r.f_sub) ~= 0 | r.f_trans_id(2:end) ~= 0) + 1];
        M = [r.tf(i) r.f_state(i) r.f_sub(i) r.f_trans_id(i)];
    otherwise
        error('gnc_mission_metrics:what', 'unknown metric %s', what);
end
end

function d = local_rect_obs(p, c, s, hx, hy, ob)
% distance between the rectangle (centre p, axes c / s, half sizes hx / hy) and a circle / box obstacle
if ob(1) == 1
    q = ob(2:3) - p;
    lx = q*c';  ly = q*s';
    d = hypot(max(abs(lx) - hx, 0), max(abs(ly) - hy, 0)) - ob(4);
else
    % box: sample its outline (0.02 m) against the rectangle
    cy = cos(ob(6));  sy = sin(ob(6));
    [u, v] = meshgrid(linspace(-ob(4)/2, ob(4)/2, max(2, ceil(ob(4)/0.02))), linspace(-ob(5)/2, ob(5)/2, max(2, ceil(ob(5)/0.02))));
    X = ob(2) + u(:)*cy - v(:)*sy;  Y = ob(3) + u(:)*sy + v(:)*cy;
    q = [X Y] - p;
    lx = q*c';  ly = q*s';
    d = min(hypot(max(abs(lx) - hx, 0), max(abs(ly) - hy, 0)));
end
end
