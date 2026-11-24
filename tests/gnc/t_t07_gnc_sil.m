classdef t_t07_gnc_sil < matlab.unittest.TestCase
%T_T07_GNC_SIL  Card T-07 step 7 (GNC, pass B): back-to-back tests of the GNC units, each unit in normal mode against
%   its generated C in software-in-the-loop (SIL), with the same stimulus.
%
%   runtests('t_t07_gnc_sil')                 the 12 harnesses that generate C, and the record of the two that do not
%   setenv('T07_GNC_SIL_ONLY', 'cg_gnc_PathAnalyzer,cg_gnc_MissionManager')   only these harnesses (a probe)
%   setenv('T07_GNC_SIL_T', '12')             shorter stimulus runs, in s (a probe); unset: the full runs
%
%   The units are Subsystem Reference files (subsystems/gnc/gnc_*.slx). Each is built, as in pass A
%   (work/team/t07/gnc/t07_gnc_codegen.m), in a harness top model cg_<unit> made in memory: one Subsystem Reference
%   block named after the unit file, with root ports that copy the unit's own (type, sample time, dimensions).
%
%   The stimulus is recorded closed loop in the GNC mission bench (tests/gnc/tb_gnc_mission.slx, through
%   gnc_mission_run): the source of every input of every unit is marked for logging in Shaddad_GNC, in memory only
%   (the model is closed with close_system(m, 0)), and the bench runs. Each harness then replays the logged inputs of
%   its unit, open loop, in normal mode and in SIL. The plan input is the run's compiled plan, held constant.
%     Run A: one 30 m mower row in world 3 (the bench's bypass case) with a 1 m box on the lane centre at 15 m, a
%            side drift of 0.03 m/s, the MPC row tracker and the VFH bypass (VC_TEB 0): the robot works the row,
%            holds at the box, goes round it, rejoins and ends the row. Every harness but the TEB one replays it.
%     Run B: the same row without an implement and with the TEB bypass (VC_TEB 1), for the TEB harness.
%   Pure Pursuit, the MPC and the Stanley law compute in every step (their yaw rates are root outputs of the path
%   tracker), so each law runs whichever tracker the state machine selects.
%
%   Both simulations of a harness use one configuration set made in memory: the pass A settings (ert.tlc, C,
%   MinGW64, ARM 64-bit LP64, Inlined parameters, the trace comments) with PortableWordSizes on, because the code is
%   for LP64 and the host is 64-bit Windows, and GenCodeOnly off. The SIL code is built in work/team/t07/sil/build,
%   so the traced code in work/cg_*_ert_rtw stays as generated. Every root output is compared leaf by leaf:
%   integers, booleans and enumerations bit-exact; single within 1e-5 x max(1, |reference|); double within
%   1e-9 x max(1, |reference|), unless the table TOL gives a tolerance with its reason; the same sample times.
%   One record per harness: work/team/t07/sil/<harness>.json; the outputs of both runs:
%   work/team/t07/sil/data/<harness>.mat. Run it in a batch MATLAB with the project open; it assigns P,
%   VC_ROW_TRACKER, VC_TEB and mpcRowObj in that MATLAB's own base workspace and puts them back.
%   Card T-07 step 7, the GNC engineer.

    properties (Constant)
        % harness, unit file, variant settings, stimulus run
        H = {
            'cg_gnc_InputConditioning',   'gnc_InputConditioning', {},                    'A'
            'cg_gnc_MissionManager',      'gnc_MissionManager',    {},                    'A'
            'cg_gnc_PathServer',          'gnc_PathServer',        {},                    'A'
            'cg_gnc_PathAnalyzer',        'gnc_PathAnalyzer',      {},                    'A'
            'cg_gnc_LocalAvoidance',      'gnc_LocalAvoidance',    {'VC_TEB', 0},         'A'
            'cg_gnc_LocalAvoidance_teb',  'gnc_LocalAvoidance',    {'VC_TEB', 1},         'B'
            'cg_gnc_SpeedGovernor',       'gnc_SpeedGovernor',     {},                    'A'
            'cg_gnc_PathTracker',         'gnc_PathTracker',       {'VC_ROW_TRACKER', 1}, 'A'
            'cg_gnc_PathTracker_stanley', 'gnc_PathTracker',       {'VC_ROW_TRACKER', 2}, 'A'
            'cg_gnc_ImplementManager',    'gnc_ImplementManager',  {},                    'A'
            'cg_gnc_CommandShaping',      'gnc_CommandShaping',    {},                    'A'
            'cg_gnc_StatusMetrics',       'gnc_StatusMetrics',     {},                    'A'
            }
        % tolerance overrides: harness (regexp), output leaf (regexp), relative tolerance, absolute floor, reason
        TOL = {
            '^cg_gnc_LocalAvoidance_teb$', '^teb_cmd$', 1e-3, 0, ['TEB optimiser, 1e-3 x max(1, |ref|): in normal ' ...
            'mode the TEB block runs interpreted MATLAB with MATLAB''s sparse linear algebra (D56), in SIL the ' ...
            'generated C with the CXSparse solver, and every 0.2 s call warm-starts from the last band, so the two ' ...
            'drift apart by rounding during a bypass. Measured on run B (2026-11-23): at most 7.0e-5 m/s or rad/s; ' ...
            'the tolerance leaves 14 times that and is 0.17 % of the 0.6 m/s cap']
            '^cg_gnc_LocalAvoidance_teb$', '^teb_info$', 1e-4, 0, ['TEB SolnInfo, 1e-4 x max(1, |ref|): the same ' ...
            'cause; measured at most 2.2e-6 relative, in the trajectory cost, so 45 times margin. ExitFlag, ' ...
            'LastFeasibleIdx and HasReachedGoal stay exact under it, since a change of 1 is far above 1e-4 x ' ...
            'max(1, |ref|)']
            }
        % extra code-generation settings of one harness, with the reason (set in memory, after the common ones)
        EXTRA = {
            'cg_gnc_LocalAvoidance_teb', {'CoderTypedefsCompatibility', 'on'}, ['the TEB block''s CXSparse ' ...
            'sources include rtwtypes.h, which ert.tlc does not generate by default in R2026b; without it the SIL ' ...
            'build stops at the compiler (2026-11-23, finding T07-GNC-07). The generated algorithm code is the same']
            }
        TFULL = struct('A', 52, 'B', 52)          % s, the stimulus runs
    end

    properties (TestParameter)
        harness = {'cg_gnc_InputConditioning', 'cg_gnc_MissionManager', 'cg_gnc_PathServer', 'cg_gnc_PathAnalyzer', ...
            'cg_gnc_LocalAvoidance', 'cg_gnc_LocalAvoidance_teb', 'cg_gnc_SpeedGovernor', 'cg_gnc_PathTracker', ...
            'cg_gnc_PathTracker_stanley', 'cg_gnc_ImplementManager', 'cg_gnc_CommandShaping', 'cg_gnc_StatusMetrics'}
    end

    properties
        Root
        Runs = struct()
        Base
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.Root = char(currentProject().RootFolder);
            if ~evalin('base', 'exist(''P'', ''var'') && exist(''Ts'', ''var'') && exist(''Bus_GncCmd'', ''var'')')
                shaddad_params();
            end
            tc.Base = struct('P', evalin('base', 'P'), 'VC_ROW_TRACKER', evalin('base', 'VC_ROW_TRACKER'), ...
                'VC_TEB', evalin('base', 'VC_TEB'));
            b0 = tc.Base;
            tc.addTeardown(@() t_t07_gnc_sil.restoreBase(b0));
            d = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'data');
            if ~isfolder(d), mkdir(d); end
            sel = t_t07_gnc_sil.selected();
            runs = unique(t_t07_gnc_sil.H(ismember(t_t07_gnc_sil.H(:, 1), sel), 4))';
            if isempty(runs), return, end
            % the stimulus: the inputs of every unit, logged inside Shaddad_GNC (marked in memory, never saved)
            map = t_t07_gnc_sil.markLogging();
            tc.addTeardown(@() t_t07_gnc_sil.closeQuiet('Shaddad_GNC'));
            tc.addTeardown(@() t_t07_gnc_sil.closeQuiet('tb_gnc_mission'));
            for r = runs
                tc.Runs.(r{1}) = t_t07_gnc_sil.record(r{1}, map);
                R = tc.Runs.(r{1});
                fprintf('MEASURED stimulus run %s: %.1f s simulated in %.1f s; %s\n', r{1}, R.T, R.wall, R.summary);
            end
            % the SIL builds go to their own folder, so the traced pass A code stays as generated
            old = Simulink.fileGenControl('getConfig');
            bdir = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'build');
            if ~isfolder(bdir), mkdir(bdir); end
            Simulink.fileGenControl('set', 'CodeGenFolder', bdir, 'CacheFolder', bdir, 'createDir', true);
            tc.addTeardown(@() Simulink.fileGenControl('set', 'CodeGenFolder', old.CodeGenFolder, ...
                'CacheFolder', old.CacheFolder));
        end
    end

    methods (Test, TestTags = {'T07', 'SIL'})
        function back_to_back(tc, harness)
            tc.assumeTrue(ismember(harness, t_t07_gnc_sil.selected()), 'not selected (T07_GNC_SIL_ONLY)');
            j = find(strcmp(t_t07_gnc_sil.H(:, 1), harness), 1);
            [w, u, vset, run] = t_t07_gnc_sil.H{j, :};
            R = tc.Runs.(run);
            file = fullfile(tc.Root, 'work', 'team', 't07', 'sil', [w '.json']);
            rec = struct('model', w, 'unit', u, 'status', 'fail', 'stimulus', R.stimulus, 'steps', 0, ...
                'outputs', {{}}, 'wall_s', 0, 'note', 'The run was started and gave no result.', ...
                'variants', '', 'settings', '');
            t_t07_gnc_sil.writeRecord(file, rec);
            tw = tic;
            % base workspace of this batch MATLAB: the run's P and the harness's variant choice
            assignin('base', 'P', R.P);
            vc = struct('VC_ROW_TRACKER', 1, 'VC_TEB', R.teb);
            for v = 1:2:numel(vset), vc.(vset{v}) = vset{v + 1}; end
            assignin('base', 'VC_ROW_TRACKER', vc.VC_ROW_TRACKER);
            assignin('base', 'VC_TEB', vc.VC_TEB);
            rec.variants = sprintf('VC_ROW_TRACKER %g, VC_TEB %g', vc.VC_ROW_TRACKER, vc.VC_TEB);
            if strcmp(u, 'gnc_PathTracker') && vc.VC_ROW_TRACKER == 1
                gnc_designMPC(R.P);                             % mpcRowObj in this MATLAB's base workspace
            end
            mdl = t_t07_gnc_sil.wrapper(w, u);
            tc.addTeardown(@() t_t07_gnc_sil.closeQuiet(w));
            jx = find(strcmp(t_t07_gnc_sil.EXTRA(:, 1), w), 1);
            extra = {};
            if ~isempty(jx)
                extra = t_t07_gnc_sil.EXTRA{jx, 2};
                rec.settings_extra = sprintf('%s: %s', strjoin(cellfun(@char, extra, 'UniformOutput', false), ' '), ...
                    t_t07_gnc_sil.EXTRA{jx, 3});
            end
            [cs, okS] = t_t07_gnc_sil.config(['cs_t07sil_' w], extra);
            attachConfigSet(mdl, cs, true);
            setActiveConfigSet(mdl, cs.Name);
            rec.settings = strjoin(okS, ', ');
            ds = t_t07_gnc_sil.inputs(mdl, u, R);
            Yn = t_t07_gnc_sil.simulate(mdl, ds, R.T, 'normal');
            try
                Ys = t_t07_gnc_sil.simulate(mdl, ds, R.T, 'sil');
            catch e
                msg = getReport(e, 'basic');
                rec.wall_s = round(toc(tw), 1);
                rec.note = ['The SIL build or run failed: ' msg];
                t_t07_gnc_sil.writeRecord(file, rec);
                tc.verifyFail(rec.note);
                return
            end
            [outs, allpass, nsteps] = t_t07_gnc_sil.compare(w, Yn.y, Ys.y);
            for k = 1:numel(outs)
                o = outs{k};
                tc.verifyTrue(o.pass, sprintf('%s %s (%s): max |SIL - normal| %g, tolerance %g, same samples %d', w, ...
                    o.name, o.type, o.max_abs_err, o.tol, o.same_samples));
            end
            yN = Yn.y; yS = Ys.y; %#ok<NASGU>
            save(fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'data', [w '.mat']), 'yN', 'yS', '-v7.3');
            rec.outputs = outs;
            rec.steps = nsteps;
            rec.wall_s = round(toc(tw), 1);
            if allpass, rec.status = 'pass'; end
            rec.note = sprintf(['%d root output leaves compared; normal %.1f s, SIL %.1f s (the SIL build included); ' ...
                'the stimulus replays run %s.'], numel(outs), Yn.wall, Ys.wall, run);
            t_t07_gnc_sil.writeRecord(file, rec);
            fprintf('MEASURED SIL %s: %s, %d leaves, %d steps, normal %.1f s, SIL %.1f s\n', w, rec.status, ...
                numel(outs), nsteps, Yn.wall, Ys.wall);
        end

        function blocked_units(tc)
            % the two models with no C (finding T07-GNC-02): recorded as blocked, never a pass
            why = ['BLOCKED: no C. The Hybrid A* MATLAB Function (gnc_GlobalReplanner/Replan/HybridAStar) fails code ' ...
                'generation with Stateflow:cdr:InvalidEnumType3 (coder.internal.interpolate.interpMethodsEnum first ' ...
                'used behind coder.target), finding T07-GNC-02, card B-02.'];
            for m = {'cg_gnc_GlobalReplanner', 'Shaddad_GNC'}
                rec = struct('model', m{1}, 'status', 'blocked', 'stimulus', '', 'steps', 0, 'outputs', {{}}, ...
                    'wall_s', 0, 'note', why);
                t_t07_gnc_sil.writeRecord(fullfile(tc.Root, 'work', 'team', 't07', 'sil', [m{1} '.json']), rec);
            end
            tc.assumeFail(why);
        end
    end

    methods (Static)
        function s = selected()
            s = t_t07_gnc_sil.H(:, 1)';
            v = strtrim(getenv('T07_GNC_SIL_ONLY'));
            if ~isempty(v)
                s = intersect(s, strtrim(strsplit(v, ',')), 'stable');
            end
        end

        function T = runTime(run)
            T = t_t07_gnc_sil.TFULL.(run);
            v = str2double(getenv('T07_GNC_SIL_T'));
            if isfinite(v) && v > 0, T = v; end
        end

        function map = markLogging()
        %MARKLOGGING  Mark, in memory, the source port of every input of every unit in Shaddad_GNC for logging, with a
        %   custom name; map.(unit) lists the logged name of each of its inputs ('#plan' for the plan constant).
            m = 'Shaddad_GNC';
            load_system(m);
            blocks = find_system(m, 'SearchDepth', 1, 'BlockType', 'SubSystem');
            map = struct();
            for i = 1:numel(blocks)
                b = blocks{i};
                u = get_param(b, 'ReferencedSubsystem');
                if isempty(u), continue, end
                ph = get_param(b, 'PortHandles');
                names = cell(1, numel(ph.Inport));
                for k = 1:numel(ph.Inport)
                    ln = get_param(ph.Inport(k), 'Line');
                    src = get_param(ln, 'SrcPortHandle');
                    sb = get_param(src, 'Parent');
                    if strcmp(get_param(sb, 'Name'), 'MissionSource')
                        names{k} = '#plan';
                        continue
                    end
                    nm = matlab.lang.makeValidName(sprintf('t07_%s_%d', get_param(sb, 'Name'), ...
                        get_param(src, 'PortNumber')));
                    set_param(src, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', nm);
                    names{k} = nm;
                end
                map.(u) = names;
            end
        end

        function R = record(run, map)
        %RECORD  Run the mission bench closed loop and keep what the harnesses replay.
            T = t_t07_gnc_sil.runTime(run);
            P = shaddad_params('World', 3, 'Assign', false);
            S = struct('mission_type', 'GOTO', 'rows_enu', [16 20 16 50], 'home_enu', [16 20 pi/2], ...
                'speed_work', 1.0, 'allow_bypass', true, 'lane_halfwidth_m', 2.5);
            if strcmp(run, 'A')
                S.implement = struct('type', 'MOWER', 'speed_rpm', 2800, 'height_m', 0.06);
                teb = 0;  drift = 0.03;
                txt = ['mission bench, world 3: one 30 m mower row (16, 20) to (16, 50) at 1.0 m/s, START at 1 s, a 1 m ' ...
                    'box on the lane centre at (16, 35), side drift 0.03 m/s, the MPC row tracker, the VFH bypass ' ...
                    '(VC_TEB 0); %.0f s'];
            else
                teb = 1;  drift = 0;
                txt = ['mission bench, world 3: the same 30 m row with no implement, the box at (16, 35), the TEB ' ...
                    'bypass (VC_TEB 1); %.0f s'];
            end
            plan = gnc_compileMission(P, S, 'Validate', false);
            dyn = [1 16 35 0.5 0 0 0 1e5];
            r = gnc_mission_run(P, plan, 'StopTime', T, 'cmd', [1 2], 'dyn', dyn, 'Teb', teb, 'Tracker', 1, ...
                'Drift', drift);
            R = struct('T', T, 'P', r.P, 'plan', plan, 'teb', teb, 'wall', r.wall, 'logs', r.logsout, 'map', map, ...
                'stimulus', sprintf(txt, T));
            have = r.logsout.getElementNames();
            c = struct2cell(map);
            want = unique([c{:}]);
            want = want(~strcmp(want, '#plan'));
            missing = setdiff(want, have);
            fprintf('stimulus run %s: %d logged signals, %d unit inputs marked, missing %s\n', run, numel(have), ...
                numel(want), strjoin(missing, ' '));
            st = unique(r.f_state(:))';  sb = unique(r.f_sub(:))';  ids = unique(r.f_trans_id(r.f_trans_id > 0))';
            R.summary = sprintf(['states %s, sub-modes %s, transitions %s; teb_enable %d steps, impl on %d steps, ' ...
                'd_obs min %.2f m'], mat2str(st), mat2str(sb), mat2str(ids), nnz(r.f_teb_enable), ...
                nnz(r.mower_enable), min(r.o_d_obs));
            R.stimulus = [R.stimulus '; ' R.summary];
        end

        function w = wrapper(w, u)
        %WRAPPER  The harness of pass A: a top model with one Subsystem Reference block (unit file u) and root ports
        %   copied from the unit's own ports.
            if bdIsLoaded(w), close_system(w, 0); end
            new_system(w);
            load_system(u);
            add_block('simulink/Ports & Subsystems/Subsystem Reference', [w '/' u], 'ReferencedSubsystem', u);
            ins = t_t07_gnc_sil.sortedPorts(find_system(u, 'SearchDepth', 1, 'BlockType', 'Inport'));
            outs = t_t07_gnc_sil.sortedPorts(find_system(u, 'SearchDepth', 1, 'BlockType', 'Outport'));
            names = {};
            for k = 1:numel(ins)
                b = ins{k};
                nm = t_t07_gnc_sil.uniqueName(get_param(b, 'Name'), names);
                names{end + 1} = nm; %#ok<AGROW>
                add_block('built-in/Inport', [w '/' nm], 'Port', get_param(b, 'Port'), ...
                    'OutDataTypeStr', get_param(b, 'OutDataTypeStr'), 'SampleTime', get_param(b, 'SampleTime'), ...
                    'PortDimensions', get_param(b, 'PortDimensions'));
                add_line(w, [nm '/1'], [u '/' get_param(b, 'Port')]);
            end
            for k = 1:numel(outs)
                b = outs{k};
                nm = t_t07_gnc_sil.uniqueName(get_param(b, 'Name'), names);
                names{end + 1} = nm; %#ok<AGROW>
                add_block('built-in/Outport', [w '/' nm], 'Port', get_param(b, 'Port'), ...
                    'OutDataTypeStr', get_param(b, 'OutDataTypeStr'));
                add_line(w, [u '/' get_param(b, 'Port')], [nm '/1']);
            end
        end

        function c = sortedPorts(c)
            p = cellfun(@(b) str2double(get_param(b, 'Port')), c);
            [~, i] = sort(p);
            c = c(i);
        end

        function nm = uniqueName(nm, names)
            base = nm;
            k = 1;
            while any(strcmp(names, nm))
                nm = sprintf('%s_%d', base, k);
                k = k + 1;
            end
        end

        function [cs, ok] = config(name, extra)
        %CONFIG  The pass A code-generation settings, with PortableWordSizes on and GenCodeOnly off for SIL; extra
        %   name/value pairs come last (table EXTRA).
            if nargin < 2, extra = {}; end
            cs = Simulink.ConfigSet;
            cs.Name = name;
            nv = {
                'SolverType', 'Fixed-step'; 'Solver', 'FixedStepDiscrete'; 'FixedStep', 'auto'; 'StartTime', '0.0';
                'EnableMultiTasking', 'on'; 'AutoInsertRateTranBlk', 'off';
                'SystemTargetFile', 'ert.tlc'; 'TargetLang', 'C'; 'Toolchain', 'MinGW64 | gmake (64-bit Windows)';
                'ProdHWDeviceType', 'ARM Compatible->ARM 64-bit (LP64)'; 'GenCodeOnly', 'off'; 'PortableWordSizes', 'on';
                'DefaultParameterBehavior', 'Inlined'; 'MATLABDynamicMemAlloc', 'on'; 'SupportVariableSizeSignals', 'on';
                'GenerateComments', 'on'; 'SimulinkBlockComments', 'on'; 'MATLABSourceComments', 'on';
                'MATLABFcnDesc', 'on'; 'ShowEliminatedStatement', 'on'; 'InsertBlockDesc', 'on'; 'MaxIdLength', '63';
                'GenerateReport', 'off'; 'LaunchReport', 'off'; 'RTWVerbose', 'off'};
            nv = [nv; reshape(extra, 2, [])'];
            ok = {};
            for k = 1:size(nv, 1)
                try
                    set_param(cs, nv{k, 1}, nv{k, 2});
                    ok{end + 1} = sprintf('%s %s', nv{k, 1}, nv{k, 2}); %#ok<AGROW>
                catch e
                    ok{end + 1} = sprintf('%s NOT SET (%s)', nv{k, 1}, e.message); %#ok<AGROW>
                end
            end
        end

        function ds = inputs(mdl, u, R)
        %INPUTS  The root-inport Dataset of a harness: each input is the logged signal of the unit's input in the
        %   stimulus run, the plan the run's compiled plan held constant.
            ports = t_t07_gnc_sil.sortedPorts(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport'));
            names = R.map.(u);
            ds = Simulink.SimulationData.Dataset;
            for k = 1:numel(ports)
                nm = names{k};
                if strcmp(nm, '#plan')
                    v = t_t07_gnc_sil.busTs('Bus_MissionPlan', R.plan, [0; R.T]);
                else
                    el = R.logs.getElement(nm);
                    if isa(el, 'Simulink.SimulationData.Dataset'), el = el.getElement(1); end
                    v = el.Values;
                end
                ds = ds.addElement(v, get_param(ports{k}, 'Name'));
            end
        end

        function v = busTs(busName, x, t)
        %BUSTS  A struct of timeseries of a bus value held constant over the times t.
            bo = evalin('base', busName);
            v = struct();
            n = numel(t);
            for e = 1:numel(bo.Elements)
                el = bo.Elements(e);
                dt = el.DataType;
                val = x.(el.Name);
                if startsWith(dt, 'Bus:')
                    v.(el.Name) = t_t07_gnc_sil.busTs(strtrim(dt(5:end)), val, t);
                    continue
                end
                if strcmp(dt, 'boolean'), val = logical(val); else, val = cast(val, dt); end
                dims = el.Dimensions;
                if prod(dims) == 1
                    d = repmat(val, n, 1);
                elseif numel(dims) == 1
                    d = repmat(reshape(val, 1, []), n, 1);
                else
                    d = repmat(reshape(val, dims), [1 1 n]);
                end
                v.(el.Name) = timeseries(d, t, 'Name', el.Name);
            end
        end

        function Y = simulate(mdl, ds, T, mode)
            in = Simulink.SimulationInput(mdl);
            in = in.setExternalInput(ds);                        % this also turns the loading of the inputs on
            in = in.setModelParameter('StopTime', sprintf('%.6g', T), ...
                'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'SaveTime', 'off', ...
                'SaveState', 'off', 'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');
            switch mode
                case 'normal', in = in.setModelParameter('SimulationMode', 'normal');
                case 'sil', in = in.setModelParameter('SimulationMode', 'software-in-the-loop (sil)');
            end
            t0 = tic;
            out = sim(in);
            Y.wall = toc(t0);
            if ~isempty(out.ErrorMessage), error('t_t07_gnc_sil:sim', '%s', out.ErrorMessage); end
            y = out.yout;
            Y.y = struct();
            for k = 1:numElements(y)
                e = y.getElement(k);
                nm = '';
                try
                    nm = get_param(e.BlockPath.getBlock(1), 'Name');      % the root Outport (the lines are unnamed)
                catch
                end
                if isempty(nm), nm = e.Name; end
                if isempty(nm), nm = sprintf('out%d', k); end
                Y.y.(matlab.lang.makeValidName(nm)) = e.Values;
            end
        end

        function [outs, allpass, nsteps] = compare(w, A, B)
        %COMPARE  Every leaf of every root output of the normal run (A) against the SIL run (B).
            outs = {};
            allpass = true;
            nsteps = 0;
            fa = fieldnames(A);
            for g = 1:numel(fa)
                LA = t_t07_gnc_sil.leaves(A.(fa{g}), fa{g});
                if isfield(B, fa{g}), LB = t_t07_gnc_sil.leaves(B.(fa{g}), fa{g}); else, LB = cell(0, 2); end
                for k = 1:size(LA, 1)
                    a = LA{k, 2};
                    jb = find(strcmp(LB(:, 1), LA{k, 1}), 1);
                    if isempty(jb), s = timeseries(); else, s = LB{jb, 2}; end
                    ty = class(a.Data);
                    nsteps = max(nsteps, numel(a.Time));
                    sameT = isequal(size(a.Time), size(s.Time)) && max(abs(a.Time - s.Time)) <= 1e-9;
                    da = t_t07_gnc_sil.tmajor(double(a.Data), numel(a.Time));
                    db = t_t07_gnc_sil.tmajor(double(s.Data), numel(s.Time));
                    sameT = sameT && isequal(size(da), size(db));
                    [rel, absf, why] = t_t07_gnc_sil.tolerance(w, LA{k, 1}, ty);
                    if sameT
                        nanA = isnan(da(:));  nanB = isnan(db(:));
                        e = abs(da(:) - db(:));
                        e(nanA & nanB) = 0;
                        e(xor(nanA, nanB)) = inf;
                        same_inf = isinf(da(:)) & isinf(db(:)) & sign(da(:)) == sign(db(:));
                        e(same_inf) = 0;
                        allow = max(rel*max(1, abs(da(:))), absf);
                        allow(~isfinite(allow)) = 0;
                        ok = all(e <= allow);
                        if isempty(e)
                            err = 0; tol = 0; relerr = 0;
                        else
                            [err, kmax] = max(e);
                            tol = allow(kmax);
                            relerr = max(e ./ max(1, abs(da(:))));
                        end
                        % when the two runs differ: the steps with any difference, the first such time, the last
                        % step's difference
                        es = max(reshape(e, size(da)), [], 2);
                        ndiff = nnz(es > 0);
                        tfirst = NaN;
                        if ndiff > 0, tfirst = a.Time(find(es > 0, 1)); end
                        elast = 0;
                        if ~isempty(es), elast = es(end); end
                    else
                        err = NaN; tol = 0; relerr = NaN; ok = false; ndiff = NaN; tfirst = NaN; elast = NaN;
                    end
                    outs{end + 1} = struct('name', LA{k, 1}, 'type', ty, 'max_abs_err', err, 'max_rel_err', relerr, ...
                        'tol', tol, 'tol_rule', why, 'same_samples', sameT, 'pass', ok, 'steps_differing', ndiff, ...
                        'first_difference_s', tfirst, 'last_step_err', elast); %#ok<AGROW>
                    allpass = allpass && ok;
                end
            end
        end

        function [rel, absf, why] = tolerance(w, leaf, ty)
            absf = 0;
            switch ty
                case 'single', rel = 1e-5; why = 'single: 1e-5 x max(1, |ref|) (the common brief)';
                case 'double', rel = 1e-9; why = 'double: 1e-9 x max(1, |ref|) (the common brief)';
                otherwise,     rel = 0;    why = [ty ': bit-exact'];
            end
            T = t_t07_gnc_sil.TOL;
            for k = 1:size(T, 1)
                if ~isempty(regexp(w, T{k, 1}, 'once')) && ~isempty(regexp(leaf, T{k, 2}, 'once')) && ...
                        any(strcmp(ty, {'double', 'single'}))
                    rel = T{k, 3}; absf = T{k, 4}; why = T{k, 5};
                    return
                end
            end
        end

        function L = leaves(v, prefix)
            L = cell(0, 2);
            if isa(v, 'timeseries')
                L = {prefix, v};
                return
            end
            if isa(v, 'Simulink.SimulationData.Signal'), v = v.Values; end
            fn = fieldnames(v);
            for k = 1:numel(fn)
                L = [L; t_t07_gnc_sil.leaves(v.(fn{k}), [prefix '.' fn{k}])]; %#ok<AGROW>
            end
        end

        function d = tmajor(d, nt)
        %TMAJOR  Logged data as an nt x m matrix with time along the first dimension.
            if size(d, 1) == nt && ismatrix(d), return, end
            k = find(size(d) == nt, 1, 'last');
            if isempty(k), return, end
            d = reshape(permute(d, [k, setdiff(1:ndims(d), k)]), nt, []);
        end

        function writeRecord(file, rec)
            fid = fopen(file, 'w');
            fprintf(fid, '%s\n', jsonencode(rec, 'PrettyPrint', true));
            fclose(fid);
        end

        function closeQuiet(m)
            try
                if bdIsLoaded(m), close_system(m, 0); end
            catch
            end
        end

        function restoreBase(b)
            assignin('base', 'P', b.P);
            assignin('base', 'VC_ROW_TRACKER', b.VC_ROW_TRACKER);
            assignin('base', 'VC_TEB', b.VC_TEB);
        end
    end
end
