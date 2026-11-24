classdef t_t07_nav_sil < matlab.unittest.TestCase
%T_T07_NAV_SIL  Card T-07 step 7: back-to-back tests of Shaddad_NavFusion and Shaddad_Perception, each through its
%   bench Model block (tb_nav/Nav, tb_perception/Perception) in normal mode and in software-in-the-loop (SIL), the
%   deployed model-reference code, with the same stimulus.
%
%   runtests('t_t07_nav_sil')
%
%   Code generation: the batch session's cs_Shaddad, which both models reference (Shaddad_Ref), is switched in memory
%   to plain Embedded Coder C (ert.tlc, MinGW toolchain, MATLABDynamicMemAlloc on; SupportVariableSizeSignals off for
%   NavFusion and on for Perception's scan matcher, as in pass A, with PropagateVarSize 'During execution' that SIL
%   then needs) and put back at the teardown; no model, builder or parameter file is changed and the
%   benches are closed with close_system(m, 0). The SIL code is built in work/team/t07/sil/build_nav, so the traced
%   code of pass A (work/<model>_ert_rtw) and the shared work/slprj stay as they are. PortableWordSizes stays off:
%   the production hardware of these two models' record is x86-64, the host's (step 5 report, section 5).
%   The comparison (work/team/t07/reports/step05_nav_A.md, section 5), every root output, at the same sample hits:
%     booleans and integers bit-exact (the gate's enables, navValid, headingValid, aligned, lidarLocActive, quality,
%       modeState, resetCount, rejCount, protectiveStop, scanValid, obsCount, the occupancy matrix, sm.valid, nPts);
%     double within 1e-9 x max(1, |reference|): the same double-precision algorithms on the same x86-64, so the runs
%       differ by a few units in the last place per step and the square-root EKF keeps that from growing;
%     a covariance element (covPose, the scan match's cov) within 1e-7 x sqrt(|C_ii C_jj|), P_diag within
%       1e-7 x |reference|: P spans about seven decades, so a bound relative to each element would fail on the
%       smallest ones and an absolute one would not test them;
%     the gate's nis within 1e-6 x max(1, |reference|): a quadratic form of the innovation, a difference of nearly
%       equal numbers, through the inverse of S;
%     single within 1e-5 x max(1, |reference|) (the perception outputs); NaN and Inf at the same places.
%   The gate's and the mode manager's decisions come from floating-point thresholds (the chi-square bounds, the 0.15 m
%   and 2 m position sigmas, the 2 deg heading sigma): the record gives, for the reference run, the nearest nis to its
%   bound and the nearest sigma to its limit, relative, so that a decision flipped at a threshold (below 1e-9) can be
%   told from a code defect. Records: work/team/t07/sil/<model>.json.

    properties
        Root
        Cs0
        OldGen
    end

    properties (Constant)
        SilMode = 'Software-in-the-loop (SIL)'
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.Root = char(currentProject().RootFolder);
            if ~evalin('base', ['exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''Bus_NavState'',''var'') && ' ...
                    'exist(''Bus_Perception'',''var'') && exist(''cs_Shaddad'',''var'') && exist(''cs_SimFixed'',''var'')'])
                shaddad_params();  shaddad_configsets('fixed');
            end
            % the SIL code in its own folder, emptied first: on 2026-11-23 the up-to-date check of an earlier SIL build
            % there stopped both models with an internal error ("Unable to update models referenced by ...: The logical
            % indices contain a true value outside of the array bounds"), while builds in an empty folder went through;
            % each case then builds in a subfolder of its own (useFolder)
            tc.OldGen = Simulink.fileGenControl('getConfig');
            bdir = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'build_nav');
            if isfolder(bdir), rmdir(bdir, 's'); end
            mkdir(bdir);
            Simulink.fileGenControl('set', 'CodeGenFolder', bdir, 'CacheFolder', bdir, 'createDir', true);
            tc.addTeardown(@() Simulink.fileGenControl('set', 'CodeGenFolder', tc.OldGen.CodeGenFolder, ...
                'CacheFolder', tc.OldGen.CacheFolder));
            % cs_Shaddad to ERT C in memory (both models reference it), put back at the end
            cs = evalin('base', 'cs_Shaddad');
            tc.Cs0 = copy(cs);
            tc.addTeardown(@() assignin('base', 'cs_Shaddad', tc.Cs0));
            nv = {'HardwareBoard', 'None'; 'SystemTargetFile', 'ert.tlc'; 'TargetLang', 'C'; ...
                  'Toolchain', 'MinGW64 | gmake (64-bit Windows)'; 'GenCodeOnly', 'off'; ...
                  'MATLABDynamicMemAlloc', 'on'; 'GenerateReport', 'off'; 'LaunchReport', 'off'};
            for k = 1:size(nv, 1)
                try
                    set_param(cs, nv{k, 1}, nv{k, 2});
                catch e
                    warning('t_t07_nav_sil:cs', '%s: %s', nv{k, 1}, e.message);
                end
            end
            fprintf('T07 SIL: cs_Shaddad target %s, toolchain %s, hardware %s, parameters %s\n', ...
                get_param(cs, 'SystemTargetFile'), get_param(cs, 'Toolchain'), get_param(cs, 'ProdHWDeviceType'), ...
                get_param(cs, 'DefaultParameterBehavior'));
            for b = {'Shaddad_NavFusion', 'Shaddad_Perception'}    % loaded by the benches: closed last, unsaved
                tc.addTeardown(@() t_t07_nav_sil.closeIf(b{1}));
            end
            for b = {'tb_nav', 'tb_perception'}
                if ~bdIsLoaded(b{1}), load_system(b{1}); end
                tc.addTeardown(@() t_t07_nav_sil.closeIf(b{1}));
            end
        end
    end

    methods (Static)
        function closeIf(m)
            if bdIsLoaded(m), close_system(m, 0); end
        end

        function P = warm(P)
            P.sensors.gnss.initFix = 4;                       % RTK receiver already fixed (as t_nav)
        end

        function b = base(P, s)
            b = [0 0 0 s s P.structure.track.chi_nom 0 0];    % prof base row [v r vy sL sR chi gx gy] (as t_nav)
        end

        function P = wallWorld(P)
            % as t_perception's tPER04: open field with a 50 m wall along x, face at y = 3.0 (box y 3.0 .. 3.2)
            P.world.obstacles.box(1, :) = [20 3.1 0 50 0.2 1.0 0 5 1];
            P.world.obstacles.nBox = 1;
            P.world.occ = world_occupancy(P.world);
        end

        function so = navRun(P, prof, tbn, mode)
        %NAVRUN  tb_nav as nav_tb_run runs it (bench GNSS velocity noise), with the Nav Model block in MODE.
            tb = struct('pose0', [0 0 0], 'seedSm', [9101 9102 9103 9104], 'seedVel', [9201 9202 9203]);
            fn = fieldnames(tbn);
            for k = 1:numel(fn), tb.(fn{k}) = tbn.(fn{k}); end
            P.sensors.gnss.sigVel(:) = 0;
            in = Simulink.SimulationInput('tb_nav');
            in = in.setExternalInput(nav_tb_input(P, prof));
            in = in.setModelParameter('StopTime', num2str(prof.T, 17), 'SimulationMode', 'normal');
            in = in.setVariable('P', P);
            in = in.setVariable('tbn', tb, 'Workspace', 'tb_nav');
            if ~strcmp(mode, 'Normal'), in = in.setBlockParameter('tb_nav/Nav', 'SimulationMode', mode); end
            so = sim(in);
            if ~isempty(so.ErrorMessage), error('t_t07_nav_sil:sim', '%s', so.ErrorMessage); end
        end

        function L = leaves(V, pre)
        %LEAVES  {name, timeseries} of every leaf of a logged bus (a struct of timeseries, nested).
            L = cell(0, 2);
            f = fieldnames(V);
            for k = 1:numel(f)
                v = V.(f{k});
                nm = [pre f{k}];
                if isstruct(v)
                    L = [L; t_t07_nav_sil.leaves(v, [nm '.'])]; %#ok<AGROW>
                else
                    L(end+1, :) = {nm, v}; %#ok<AGROW>
                end
            end
        end

        function d = tmajor(d, nt)
        %TMAJOR  Data as nt x m with time along the first dimension (logged matrices are r x c x nt).
            if ismatrix(d) && size(d, 1) == nt, return, end
            k = find(size(d) == nt, 1, 'last');
            if isempty(k), d = reshape(d, nt, []); return, end
            d = reshape(permute(d, [k, setdiff(1:ndims(d), k)]), nt, []);
        end

        function [err, tol, ok, rule] = compare(name, a, b)
        %COMPARE  Largest error of SIL (b) against normal (a), the tolerance it is judged by, and the rule.
            ta = a.Time(:);  tb = b.Time(:);
            if numel(ta) ~= numel(tb) || any(ta ~= tb)
                err = Inf; tol = 0; ok = false; rule = 'sample hits differ'; return
            end
            nt = numel(ta);
            A = t_t07_nav_sil.tmajor(a.Data, nt);  B = t_t07_nav_sil.tmajor(b.Data, nt);
            if ~isequal(size(A), size(B)) || ~strcmp(class(A), class(B))
                err = Inf; tol = 0; ok = false; rule = 'size or class differ'; return
            end
            leaf = regexprep(name, '^.*\.', '');
            if islogical(A) || isinteger(A) || strcmp(leaf, 'rejCount')
                rule = 'bit-exact';
                tol = 0;  d = double(A ~= B);  err = max([0; d(:)]);  ok = err == 0;
                if ~ok, rule = sprintf('bit-exact: %d of %d values differ, first at t = %.3f s', nnz(d), numel(d), ...
                        ta(find(any(d, 2), 1))); end
                return
            end
            A = double(A);  B = double(B);
            fa = isfinite(A);  fb = isfinite(B);
            same = (A == B) | (isnan(A) & isnan(B));
            if any(fa(:) ~= fb(:)) || any(~fa(:) & ~same(:))
                err = Inf; tol = 0; ok = false; rule = 'NaN or Inf at different places'; return
            end
            d = abs(A - B);  d(~fa) = 0;
            if any(strcmp(leaf, {'covPose', 'cov'})) && size(A, 2) == 9
                rule = 'covariance: 1e-7 x sqrt(|C_ii C_jj|)';
                [ii, jj] = ndgrid(1:3, 1:3);
                T = 1e-7*sqrt(abs(A(:, (ii(:) - 1)*3 + ii(:)).*A(:, (jj(:) - 1)*3 + jj(:)))) + 1e-15;
            elseif strcmp(leaf, 'P_diag')
                rule = 'P diagonal: 1e-7 x |ref|';
                T = 1e-7*abs(A) + 1e-15;
            elseif strcmp(leaf, 'nis')
                rule = 'nis: 1e-6 x max(1, |ref|)';
                T = 1e-6*max(1, abs(A));
            elseif isa(a.Data, 'single')
                rule = 'single: 1e-5 x max(1, |ref|)';
                T = 1e-5*max(1, abs(A));
            else
                rule = 'double: 1e-9 x max(1, |ref|)';
                T = 1e-9*max(1, abs(A));
            end
            r = d./T;
            [rmax, k] = max(r(:));
            if isempty(rmax), rmax = 0; k = 1; end
            if isempty(d), err = 0; tol = 0; else, err = d(k); tol = T(k); end
            ok = rmax <= 1;
        end

        function writeRecord(file, rec)
            fid = fopen(file, 'w');
            fprintf(fid, '%s\n', jsonencode(rec, 'PrettyPrint', true));
            fclose(fid);
        end

        function [outs, allok, nsteps, worst] = compareRuns(tag, A, B)
        %COMPARERUNS  Compare two lists of {name, Values} buses: one output row per leaf, and the worst
        %   error-to-tolerance ratio of the floating-point outputs.
            outs = struct('name', {}, 'type', {}, 'max_abs_err', {}, 'tol', {}, 'pass', {}, 'rule', {});
            allok = true;  nsteps = 0;  worst = struct('name', '', 'err', 0, 'tol', 0, 'ratio', 0);
            for g = 1:size(A, 1)
                LA = t_t07_nav_sil.leaves(A{g, 2}, [A{g, 1} '.']);
                LB = t_t07_nav_sil.leaves(B{g, 2}, [B{g, 1} '.']);
                for k = 1:size(LA, 1)
                    j = find(strcmp(LB(:, 1), LA{k, 1}), 1);
                    if isempty(j)
                        e = Inf; t = 0; ok = false; rule = 'missing in SIL';
                    else
                        [e, t, ok, rule] = t_t07_nav_sil.compare(LA{k, 1}, LA{k, 2}, LB{j, 2});
                    end
                    nsteps = max(nsteps, numel(LA{k, 2}.Time));
                    outs(end+1) = struct('name', [tag ': ' LA{k, 1}], 'type', class(LA{k, 2}.Data), ...
                        'max_abs_err', e, 'tol', t, 'pass', ok, 'rule', rule); %#ok<AGROW>
                    allok = allok && ok;
                    if t > 0 && e/t > worst.ratio
                        worst = struct('name', LA{k, 1}, 'err', e, 'tol', t, 'ratio', e/t);
                    end
                end
            end
        end

        function [mNis, mSig] = margins(D, P)
        %MARGINS  For the reference run: the smallest relative distance of an evaluated nis to its chi-square bound
        %   (sens_nav_gate: chi2(m(j)), m = [3 2 1 2 3]) and of the position or heading sigma to a mode limit
        %   (sens_nav_mode: sqrt(max(P11, P22)) against sigPos and sigPosValid, sqrt(P33) against sigYaw).
            M = P.sensors.navmode;
            nis = t_t07_nav_sil.tmajor(D.nis.Data, numel(D.nis.Time));
            bnd = P.sensors.navgate.chi2(M.m);
            rel = abs(nis - bnd)./bnd;
            rel(nis <= 0) = Inf;
            mNis = min(rel(:));
            Pd = t_t07_nav_sil.tmajor(D.P_diag.Data, numel(D.P_diag.Time));
            sP = sqrt(max(Pd(:, 1), Pd(:, 2)));  sY = sqrt(Pd(:, 3));
            mSig = min([abs(sP - M.sigPos)/M.sigPos; abs(sP - M.sigPosValid)/M.sigPosValid; ...
                abs(sY - M.sigYaw)/M.sigYaw]);
        end

        function on = sacOn()
        %SACON  Smart App Control on (the state that refused locally built programs on 2026-11-22).
            try
                on = double(winqueryreg('HKEY_LOCAL_MACHINE', 'SYSTEM\CurrentControlSet\Control\CI\Policy', ...
                    'VerifiedAndReputablePolicyState')) == 1;
            catch
                on = false;
            end
        end

        function tf = refused(msg)
        %REFUSED  True when the error says the SIL program could not be started (an operating-system refusal).
            tf = ~isempty(regexpi(msg, ['(access is denied|smart app|code integrity|blocked by|has been blocked|' ...
                'failed to start|could not start|unable to start|cannot start|could not be started|failed to launch|' ...
                'unable to launch|could not launch|error launching|0xc0000|connection.*(timed out|refused))'], 'once'));
        end
    end

    methods (Access = private)
        function s = varSize(~, on)
        %VARSIZE  Variable-size signals as in each model's pass A record (gen_nav_A.json): off for NavFusion, on for
        %   Perception's scan matcher. A referenced model in SIL with them on also needs PropagateVarSize 'During
        %   execution' (the 2026-11-23 run stopped on it); the normal reference run uses the same set.
            cs = evalin('base', 'cs_Shaddad');
            if on
                set_param(cs, 'SupportVariableSizeSignals', 'on');
                set_param(cs, 'PropagateVarSize', 'During execution');
            else
                set_param(cs, 'SupportVariableSizeSignals', 'off');
            end
            s = sprintf(['cs_Shaddad in memory: %s, %s, %s, MATLABDynamicMemAlloc %s, SupportVariableSizeSignals %s, ' ...
                'PropagateVarSize %s, DefaultParameterBehavior %s'], get_param(cs, 'SystemTargetFile'), ...
                get_param(cs, 'TargetLang'), get_param(cs, 'Toolchain'), get_param(cs, 'MATLABDynamicMemAlloc'), ...
                get_param(cs, 'SupportVariableSizeSignals'), get_param(cs, 'PropagateVarSize'), ...
                get_param(cs, 'DefaultParameterBehavior'));
            fprintf('T07 SIL: %s\n', s);
        end

        function d = useFolder(tc, name)
        %USEFOLDER  An empty code-generation and cache folder of its own for one case (normal run, SIL build, SIL run).
            d = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'build_nav', name);
            if isfolder(d), rmdir(d, 's'); end
            mkdir(d);
            Simulink.fileGenControl('set', 'CodeGenFolder', d, 'CacheFolder', d, 'createDir', true);
        end

        function finish(tc, file, rec, outs, allok, cs, tw, errs)
        %FINISH  Write the record and judge it: blocked when a SIL program was refused with Smart App Control on
        %   (assumeFail, common brief section 6), else pass or fail; a case whose SIL build or run failed is a fail.
            blocked = false;
            for k = 1:numel(errs)
                blocked = blocked || (t_t07_nav_sil.sacOn() && t_t07_nav_sil.refused(errs{k}));
            end
            if blocked, rec.status = 'blocked'; elseif allok, rec.status = 'pass'; else, rec.status = 'fail'; end
            rec.steps = sum([cs.steps]);
            rec.outputs = outs;
            rec.cases = cs;
            rec.wall_s = round(toc(tw), 1);
            if ~isempty(errs)
                rec.note = [rec.note ' SIL build or run errors: ' strjoin(errs, ' | ')];
            end
            t_t07_nav_sil.writeRecord(file, rec);
            if blocked
                tc.assumeFail(['BLOCKED: a SIL program was refused (Smart App Control on): ' strjoin(errs, ' | ')]);
            end
            bad = outs(~[outs.pass]);
            for k = 1:numel(bad)
                tc.verifyTrue(false, sprintf('%s (%s): error %.3g against %.3g (%s)', bad(k).name, bad(k).type, ...
                    bad(k).max_abs_err, bad(k).tol, bad(k).rule));
            end
            for k = 1:numel(cs)
                fprintf(['T07 SIL %s %s: %d steps, %d outputs, %d failed; worst %s %.3g against %.3g; wall normal ' ...
                    '%.1f s, SIL %.1f s\n'], rec.model, cs(k).tag, cs(k).steps, cs(k).outputs, cs(k).failed, ...
                    cs(k).worst.name, cs(k).worst.err, cs(k).worst.tol, cs(k).wall_normal_s, cs(k).wall_sil_s);
            end
            tc.verifyTrue(allok, sprintf('%s: SIL against normal mode', rec.model));
        end
    end

    methods (Test, TestTags = {'T07', 'SIL'})
        function navfusion_back_to_back(tc)
            file = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'Shaddad_NavFusion.json');
            rec = struct('model', 'Shaddad_NavFusion', 'status', 'fail', 'stimulus', ...
                ['(1) tNAV02''s lawn-mower case, its first 60 s (flat test world, RTK fixed, warm start: a 10 s wait, ' ...
                 'the speed-up to 1 m/s, a 40 m row and a turn); (2) tNAV07''s greenhouse LiDAR case, its first 60 s ' ...
                 '(navMode 2, emulated scan matches, GNSS cold start under the roof, ecompass heading)'], ...
                'steps', 0, 'outputs', {{}}, 'cases', {{}}, 'wall_s', 0, 'note', 'The SIL run was started and gave no result.');
            rec.settings = tc.varSize(false);
            t_t07_nav_sil.writeRecord(file, rec);
            tw = tic;
            P1 = t_t07_nav_sil.warm(shaddad_params('Assign', false));
            P4 = shaddad_params('World', 4, 'Assign', false);
            seg = {{'wait', 10}, {'speed', 1, 2}};
            for k = 1:12
                seg = [seg, {{'straight', 40}, {'turn', (-1)^(k+1)*pi, 0.5}}]; %#ok<AGROW>
            end
            C = struct('tag', {}, 'P', {}, 'prof', {}, 'tbn', {});
            C(1).tag = 'lawn-mower';  C(1).P = P1;  C(1).tbn = struct('pose0', [0 0 0]);
            C(1).prof = struct('T', 60, 'kin', nav_tb_path([seg, {{'straight', 40}}], t_t07_nav_sil.base(P1, 0.03)));
            v = 0.6;  L = 31;
            seg = {{'wait', 10}, {'speed', v, 1}, {'straight', L/v - 1}, {'turn', -pi, 0.5}, {'straight', L/v}, ...
                   {'turn', pi, 0.5}, {'straight', L/v}, {'turn', pi, 0.5}, {'straight', L/v - 1}, {'speed', 0, 1}};
            C(2).tag = 'greenhouse';  C(2).P = P4;  C(2).tbn = struct('pose0', [2.0 3.0 pi/2]);
            C(2).prof = struct('T', 60, 'kin', nav_tb_path(seg, t_t07_nav_sil.base(P4, 0.03)), ...
                'sm', [0 1 0.03 0.5*pi/180 0.1], 'navcmd', {{'navMode', [0 2]}});
            outs = struct('name', {}, 'type', {}, 'max_abs_err', {}, 'tol', {}, 'pass', {}, 'rule', {});
            allok = true;  cs = [];  errs = {};
            for c = 1:numel(C)
                tc.useFolder(sprintf('nav%d', c));
                t0 = tic;
                sn = t_t07_nav_sil.navRun(C(c).P, C(c).prof, C(c).tbn, 'Normal');
                wn = toc(t0);
                [mN, mS] = t_t07_nav_sil.margins(sn.yout{2}.Values, C(c).P);
                t0 = tic;
                try
                    ss = t_t07_nav_sil.navRun(C(c).P, C(c).prof, C(c).tbn, t_t07_nav_sil.SilMode);
                    err = '';
                catch e
                    err = getReport(e, 'basic', 'hyperlinks', 'off');
                end
                wsil = toc(t0);
                if isempty(err)
                    A = {'nav', sn.yout{1}.Values; 'diag', sn.yout{2}.Values};
                    B = {'nav', ss.yout{1}.Values; 'diag', ss.yout{2}.Values};
                    [o, ok, ns, worst] = t_t07_nav_sil.compareRuns(C(c).tag, A, B);
                else
                    errs{end+1} = sprintf('%s: %s', C(c).tag, err); %#ok<AGROW>
                    o = struct('name', [C(c).tag ': SIL run'], 'type', '', 'max_abs_err', Inf, 'tol', 0, 'pass', false, ...
                        'rule', ['the SIL build or run failed: ' err]);
                    ok = false;  ns = 0;  worst = struct('name', '', 'err', 0, 'tol', 0, 'ratio', 0);
                end
                outs = [outs, o]; %#ok<AGROW>
                allok = allok && ok;
                cs = [cs, struct('tag', C(c).tag, 'T_s', C(c).prof.T, 'steps', ns, 'outputs', numel(o), ...
                    'failed', nnz(~[o.pass]), 'worst', worst, 'nis_margin', mN, 'sigma_margin', mS, ...
                    'wall_normal_s', round(wn, 1), 'wall_sil_s', round(wsil, 1), 'error', err)]; %#ok<AGROW>
            end
            rec.note = sprintf(['Reference run: nearest evaluated nis to its chi-square bound %.3g and nearest position ' ...
                'or heading sigma to a mode limit %.3g (relative; lawn-mower), %.3g and %.3g (greenhouse); a decision ' ...
                'is at a threshold below 1e-9. Each case ran in an empty build folder of its own, so its SIL wall time ' ...
                'includes a full SIL build.'], cs(1).nis_margin, cs(1).sigma_margin, cs(2).nis_margin, cs(2).sigma_margin);
            tc.finish(file, rec, outs, allok, cs, tw, errs);
        end

        function perception_back_to_back(tc)
            file = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'Shaddad_Perception.json');
            rec = struct('model', 'Shaddad_Perception', 'status', 'fail', 'stimulus', ...
                ['(1) tPER02''s person crossing the protective field (world 6, 0.5 m/s, 6 s); (2) tPER07''s lost scans ' ...
                 '(open field, scans lost from 1.0 to 1.9 s and one invalid at 2.5 s, 3 s); (3) tPER04''s local map ' ...
                 '(open field with a 50 m wall, 5 s at rest 3 m from it, then a 20 m drive along it, 25 s); ' ...
                 '(4) tScanMatchGreenhouse''s logged aisle scans (world 4, 0.6 m/s, lidarLocActive, 30 s). Every case ' ...
                 'logs the whole Bus_Perception, the local map included, and the scan-match port.'], ...
                'steps', 0, 'outputs', {{}}, 'cases', {{}}, 'wall_s', 0, 'note', 'The SIL run was started and gave no result.');
            rec.settings = tc.varSize(true);
            t_t07_nav_sil.writeRecord(file, rec);
            tw = tic;
            W6 = shaddad_params('World', 6, 'Assign', false);
            W4 = shaddad_params('World', 4, 'Assign', false);
            P0 = shaddad_params('Assign', false);
            C = struct('tag', {}, 'P', {}, 'ds', {}, 'T', {});
            % (1) a person crossing the protective field (tPER02)
            T = 6;  v = 0.5;  t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            S = per_scan_gen(W6, t, [5 + v*t, 20 + 0*t, 0*t], 'obs', @(tt) [8.6, 24.95 - tt, 0.25], 'seed', 13);
            nav = struct('t', tn, 'pos', [5 + v*tn, 20 + 0*tn, 0*tn], 'eul', zeros(numel(tn), 3), ...
                'vel_B', repmat([v 0 0], numel(tn), 1));
            C(1).tag = 'field';  C(1).P = W6;  C(1).ds = per_tb_input(W6, S, nav);  C(1).T = T;
            % (2) lost scans (tPER07)
            ang = single(P0.sensors.lidar.angles);
            t = (0:30)'*0.1;  tn = (0:300)'*0.01;
            vv = true(numel(t), 1);  vv(t > 0.95 & t < 1.95) = false;  vv(abs(t - 2.5) < 1e-6) = false;
            S = struct('t', t, 'ranges', NaN(numel(t), 360, 'single'), 'angles', ang, 'valid', vv);
            nav = struct('t', tn, 'pos', zeros(numel(tn), 3), 'eul', zeros(numel(tn), 3), ...
                'vel_B', repmat([1 0 0], numel(tn), 1));
            C(2).tag = 'lost';  C(2).P = P0;  C(2).ds = per_tb_input(P0, S, nav);  C(2).T = 3;
            % (3) the local map along a wall (tPER04)
            Pw = t_t07_nav_sil.wallWorld(W6);
            t = (0:250)'*0.1;  x = 5 + max(0, t - 5);
            S = per_scan_gen(Pw, t, [x, 0*t, 0*t], 'seed', 6);
            tn = (0:2500)'*0.01;  xn = 5 + max(0, tn - 5);
            nav = struct('t', tn, 'pos', [xn, 0*tn, 0*tn], 'eul', zeros(numel(tn), 3), ...
                'vel_B', [double(tn > 5) zeros(numel(tn), 2)]);
            C(3).tag = 'wall map';  C(3).P = Pw;  C(3).ds = per_tb_input(Pw, S, nav);  C(3).T = 25;
            % (4) the greenhouse aisle with the scan matcher (tScanMatchGreenhouse)
            T = 30;  t = (0:T*10)'*0.1;  tn = (0:T*100)'*0.01;
            pf = @(tt) [4.8 + 0.2*sin(2*pi*tt/15), 4 + 0.6*tt, pi/2 + 0.03*sin(2*pi*tt/11)];
            S = per_scan_gen(W4, t, pf(t), 'seed', 4);
            pn = pf(tn);
            rs = RandStream('mt19937ar', 'Seed', 9);
            e0 = [0.05*randn(rs, 1, 2) 0.005*randn(rs)];
            nav = struct('t', tn, 'pos', [pn(:, 1:2) + e0(1:2), 0*tn], 'eul', [pn(:, 3) + e0(3) zeros(numel(tn), 2)], ...
                'vel_B', repmat([0.6 0 0], numel(tn), 1), 'lidarLocActive', true(numel(tn), 1));
            C(4).tag = 'greenhouse';  C(4).P = W4;  C(4).ds = per_tb_input(W4, S, nav);  C(4).T = T;
            outs = struct('name', {}, 'type', {}, 'max_abs_err', {}, 'tol', {}, 'pass', {}, 'rule', {});
            allok = true;  cs = [];  errs = {};
            for c = 1:numel(C)
                tc.useFolder(sprintf('per%d', c));
                Rn = per_tb_run(C(c).P, C(c).ds, C(c).T, 'Map', true);
                t0 = tic;
                try
                    Rs = per_tb_run(C(c).P, C(c).ds, C(c).T, 'Map', true, 'RefMode', t_t07_nav_sil.SilMode);
                    err = '';
                catch e
                    err = getReport(e, 'basic', 'hyperlinks', 'off');
                end
                wsil = toc(t0);
                if isempty(err)
                    yn = Rn.out.yout;  ys = Rs.out.yout;
                    A = {'per', yn.getElement('perception').Values; 'sm', yn.getElement('scanmatch').Values};
                    B = {'per', ys.getElement('perception').Values; 'sm', ys.getElement('scanmatch').Values};
                    [o, ok, ns, worst] = t_t07_nav_sil.compareRuns(C(c).tag, A, B);
                else
                    errs{end+1} = sprintf('%s: %s', C(c).tag, err); %#ok<AGROW>
                    o = struct('name', [C(c).tag ': SIL run'], 'type', '', 'max_abs_err', Inf, 'tol', 0, 'pass', false, ...
                        'rule', ['the SIL build or run failed: ' err]);
                    ok = false;  ns = 0;  worst = struct('name', '', 'err', 0, 'tol', 0, 'ratio', 0);
                end
                outs = [outs, o]; %#ok<AGROW>
                allok = allok && ok;
                cs = [cs, struct('tag', C(c).tag, 'T_s', C(c).T, 'steps', ns, 'outputs', numel(o), ...
                    'failed', nnz(~[o.pass]), 'worst', worst, 'sm_valid', nnz(Rn.sm.valid), ...
                    'wall_normal_s', round(Rn.wall, 1), 'wall_sil_s', round(wsil, 1), 'error', err)]; %#ok<AGROW>
            end
            rec.note = sprintf(['Scan matches valid in the reference run: %d (field), %d (lost), %d (wall map), %d ' ...
                '(greenhouse); a pose differing by one grid cell would be a tie of the correlative matcher. The ' ...
                'occupancy matrix is compared bit-exact: the bus carries the thresholded cells, not the probabilities. ' ...
                'Each case ran in an empty build folder of its own, so its SIL wall time includes a full SIL build.'], ...
                cs(1).sm_valid, cs(2).sm_valid, cs(3).sm_valid, cs(4).sm_valid);
            tc.finish(file, rec, outs, allok, cs, tw, errs);
        end
    end
end
