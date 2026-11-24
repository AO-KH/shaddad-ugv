classdef t_infra < matlab.unittest.TestCase
    %T_INFRA  Acceptance tests of build task T01 (integration infrastructure, stubs, top/plant/companion).
    %
    %   results = runtests('tests/integration/t_infra.m')        everything (about 6-9 min)
    %   runtests('t_infra', 'Tag', 'Fast')          static checks (< 1 min)
    %   runtests('t_infra', 'Tag', 'Integration')   compile / simulate (about 2 min)
    %   runtests('t_infra', 'Tag', 'Build')         regenerate every T01 file with the build scripts (about 1 min)
    %   runtests('t_infra', 'Tag', 'Cold')          shaddad_startup in a fresh MATLAB process (about 1 min)
    %   runtests('t_infra', 'Tag', 'Slow')          rebuild of the complete codegen config sets (minutes)
    %   runtests('t_infra', 'Tag', 'Codegen')       code generation (BLOCKED while Simulink Coder is missing)
    %
    %   Criteria (docs/build_plan.md T01 "Acceptance"): shaddad_startup < 15 s, 87 Bus_*, P_meta, cs_Shaddad,
    %   product report with "codegen BLOCKED" while Simulink Coder is missing; build scripts run; check
    %   interfaces = 0; Shaddad_System update for VC_PHASE 1 and 2 < 180 s with 0 algebraic loops and no
    %   implicit rate transitions; Shaddad_Plant updates for VC_IMPL 0..3 x VC_DRIVE_FID {1,3}; 2 s
    %   simulation with Simulink Profiler report work/profile_T01.html; compiled rates within
    %   P.sim.Ts_compiled_allowed, 0.014 / 0.007 only inside RC_Link_Sim.
    %   Start-up budget (see shaddad_startup): every call in a running session (testStartup) and the
    %   start-up in a fresh MATLAB process counted without the Simulink product start, which is logged
    %   separately (testStartupFreshProcess); codegen HardwareBoard fields are set on first codegen use.
    %   The shared base workspace is restored (VC_*, cs_Shaddad); models this test loads are closed.

    properties
        VC0 struct
        Loaded cell
        Root char
    end

    properties (Constant)
        Models = {'Shaddad_System', 'Shaddad_Companion', 'Shaddad_Plant'};
    end

    methods (TestClassSetup)
        function setupData(tc)
            if ~evalin('base', 'exist(''P'',''var'') && exist(''Ts'',''var'') && exist(''cs_Shaddad'',''var'')')
                shaddad_startup('quiet');
            end
            tc.Root = char(currentProject().RootFolder);
            names = evalin('base', 'who(''VC_*'')');
            s = struct();
            for k = 1:numel(names), s.(names{k}) = evalin('base', names{k}); end
            tc.VC0 = s;
            tc.Loaded = find_system('type', 'block_diagram');
            tc.addTeardown(@() t_infra.restore(tc.VC0, tc.Loaded));
        end
    end

    methods (Static)
        function restore(vc, loaded)
            fn = fieldnames(vc);
            for k = 1:numel(fn), assignin('base', fn{k}, vc.(fn{k})); end
            cur = find_system('type', 'block_diagram');
            for m = [t_infra.Models, setdiff(cur, loaded)']
                if bdIsLoaded(m{1}) && ~any(strcmp(loaded, m{1}))
                    try, close_system(m{1}, 0); catch, end %#ok<CTCH>
                end
            end
        end

        function n = loops(mdl) %#ok<INUSD> used inside evalc
            evalc('L = Simulink.BlockDiagram.getAlgebraicLoops(mdl);');
            n = numel(L);
        end

        function S = zeroTs(ic, T)
            % struct of timeseries holding the IC value over [0 T] (helper for direct model inputs)
            S = struct();
            fn = fieldnames(ic);
            for k = 1:numel(fn)
                v = ic.(fn{k});
                if isstruct(v)
                    S.(fn{k}) = t_infra.zeroTs(v, T);
                elseif isscalar(v) || (ismatrix(v) && size(v, 2) == 1)
                    S.(fn{k}) = timeseries(repmat(v(:)', 2, 1), [0; T]);
                else
                    S.(fn{k}) = timeseries(repmat(v, [ones(1, ndims(v)) 2]), [0; T]);
                end
            end
        end
    end

    %% ---------------------------------------------------------------- fast static checks
    methods (Test, TestTags = {'Fast'})
        function testProjectSetup(tc)
            proj = currentProject;
            onPath = arrayfun(@(p) char(p.File), proj.ProjectPath, 'UniformOutput', false);
            for f = {'subsystems/integration', 'scripts/build', 'scripts/scenarios'}
                d = fullfile(tc.Root, f{1});
                tc.verifyTrue(isfolder(d), d);
                tc.verifyTrue(any(strcmpi(onPath, d)), ['not on the project path: ' f{1}]);
            end
            for f = {'scripts/missions', 'utilities/stm32', 'tests/structure', 'tests/power', 'tests/drive', ...
                     'tests/electronics', 'tests/rc', 'tests/sensors', 'tests/gnc', 'tests/integration', 'tests/system'}
                tc.verifyTrue(isfolder(fullfile(tc.Root, f{1})), f{1});
            end
            su = cellfun(@char, cellstr(string(proj.StartupFiles)), 'UniformOutput', false);
            sd = cellfun(@char, cellstr(string(proj.ShutdownFiles)), 'UniformOutput', false);
            tc.verifyTrue(any(endsWith(su, fullfile('scripts', 'shaddad_startup.m'))), 'startup file not registered');
            tc.verifyTrue(any(endsWith(sd, fullfile('scripts', 'shaddad_shutdown.m'))), 'shutdown file not registered');
        end

        function testStartup(tc)
            coder = any(strcmp({ver().Name}, 'Simulink Coder'));
            t0 = tic;
            txt = evalc('rep = shaddad_startup();');
            dt = toc(t0);
            tc.log(1, sprintf('shaddad_startup (running session): %.2f s wall, project %.2f s, Simulink start %.2f s', ...
                dt, rep.t_project, rep.t_steps.simulink));
            tc.verifyLessThan(dt, 15, 'shaddad_startup must take < 15 s');
            tc.verifyLessThan(rep.t_project, 15);
            tc.verifyTrue(contains(txt, 'Codegen config sets'), 'report row of the codegen config sets');
            tc.verifyTrue(isempty(rep.deferred) || numel(rep.deferred) == 3, 'deferred = the 3 HardwareBoard fields');
            w = evalin('base', 'who');
            tc.verifyEqual(sum(startsWith(w, 'Bus_')), 87, 'number of Bus_* objects');
            tc.verifyTrue(any(strcmp(w, 'P_meta')) && any(strcmp(w, 'P')) && any(strcmp(w, 'Ts')));
            tc.verifyTrue(any(strcmp(w, 'cs_Shaddad')));
            tc.verifyClass(evalin('base', 'cs_Shaddad'), 'Simulink.ConfigSet');
            for p = {'Simulink Coder', 'Embedded Coder', 'STM32 Microcontroller Blockset', 'C compiler', ...
                     'C++ compiler', 'STM32CubeMX'}
                tc.verifyTrue(contains(txt, p{1}), ['product report misses ' p{1}]);
            end
            tc.verifyEqual(rep.codegen_blocked, ~coder);
            if ~coder
                tc.verifyTrue(contains(txt, 'codegen BLOCKED'), 'report must say codegen BLOCKED');
                row = rep.table(rep.table.Item == "Simulink Coder", :);
                tc.verifyEqual(row.Status, "MISSING");
            end
        end

        function testShutdownDryRun(tc)
            names = shaddad_shutdown('dryrun');
            tc.verifyTrue(all(ismember(names, {'P', 'Ts', 'P_meta', 'mpcRowObj'}) | startsWith(names, {'VC_', 'Bus_', 'cs_'})));
            tc.verifyTrue(all(ismember({'P', 'Ts', 'P_meta', 'cs_Shaddad', 'VC_PHASE', 'Bus_ScenarioIn'}, names)));
            tc.verifyTrue(evalin('base', 'exist(''P'',''var'')') == 1, 'dryrun must not clear');
        end

        function testConfigSets(tc)
            F = evalin('base', 'cs_SimFixed'); R = evalin('base', 'cs_SimRef'); S = evalin('base', 'cs_Shaddad');
            exp = {'SolverType', 'Fixed-step'; 'Solver', 'ode3'; 'FixedStep', '1e-3'; 'EnableMultiTasking', 'on'; ...
                   'AutoInsertRateTranBlk', 'off'; 'MultiTaskRateTransMsg', 'error'; 'SingleTaskRateTransMsg', 'error'; ...
                   'AlgebraicLoopMsg', 'error'; 'ArtificialAlgebraicLoopMsg', 'error'; ...
                   'StrictBusMsg', 'ErrorOnBusTreatedAsVector'; 'UnconnectedInputMsg', 'warning'; ...
                   'InheritedTsInSrcMsg', 'warning'; 'ProdHWDeviceType', 'Intel->x86-64 (Windows64)'; ...
                   'SignalLogging', 'on'; 'SignalLoggingName', 'logsout'; 'SaveFormat', 'Dataset'; ...
                   'ReturnWorkspaceOutputs', 'on'; 'SimscapeLogType', 'none'; 'SimCompilerOptimization', 'on'; ...
                   'UpdateModelReferenceTargets', 'IfOutOfDate'; 'DefaultParameterBehavior', 'Tunable'};
            for k = 1:size(exp, 1)
                tc.verifyEqual(get_param(F, exp{k, 1}), exp{k, 2}, ['cs_SimFixed.' exp{k, 1}]);
                tc.verifyEqual(get_param(S, exp{k, 1}), exp{k, 2}, ['cs_Shaddad.' exp{k, 1}]);
            end
            expR = {'SolverType', 'Variable-step'; 'Solver', 'daessc'; 'MaxStep', '1e-3'; 'RelTol', '1e-4'; ...
                    'AbsTol', '1e-6'; 'SimscapeLogType', 'local'; 'AutoInsertRateTranBlk', 'off'};
            for k = 1:size(expR, 1)
                tc.verifyEqual(get_param(R, expR{k, 1}), expR{k, 2}, ['cs_SimRef.' expR{k, 1}]);
            end
            for c = {'cs_GenARM', 'cs_STM32H7', 'cs_STM32G4', 'cs_Companion'}
                cs = evalin('base', c{1});
                tc.verifyEqual(get_param(cs, 'DefaultParameterBehavior'), 'Inlined', [c{1} ' parameters']);
            end
        end

        function testModelsUseShaddadRef(tc)
            T = build_stubs('table');
            T = T(strcmp({T.kind}, 'model'));
            for k = 1:numel(T)
                wasLoaded = bdIsLoaded(T(k).name);
                load_system(fullfile(tc.Root, T(k).file));
                c = getActiveConfigSet(T(k).name);
                tc.verifyClass(c, 'Simulink.ConfigSetRef', T(k).name);
                tc.verifyEqual(c.Name, 'Shaddad_Ref', T(k).name);
                tc.verifyEqual(c.SourceName, 'cs_Shaddad', T(k).name);
                if ~wasLoaded && ~any(bdIsLoaded(t_infra.Models)), close_system(T(k).name, 0); end
            end
        end

        function testCheckInterfaces(tc)
            [n, rep] = shaddad_check_interfaces('Quiet', true);
            for k = find(~[rep.ok])
                tc.log(1, sprintf('%s: %s', rep(k).name, strjoin(rep(k).mismatch, '; ')));
            end
            tc.verifyEqual(n, 0, 'shaddad_check_interfaces must report 0 mismatches');
            tc.verifyGreaterThanOrEqual(numel(rep), 17);
        end

        function testBuildStubsIdempotent(tc)
            created = build_stubs();
            tc.verifyEmpty(created, 'build_stubs() must not recreate existing files');
            T = build_stubs('table');
            tc.verifyEqual(sum([T.stub]), 16);
            for k = find([T.stub])
                f = fullfile(tc.Root, T(k).file);
                tc.verifyTrue(isfile(f), f);
            end
        end

        function testPlantStructure(tc)
            load_system('Shaddad_Plant');
            sc = find_system('Shaddad_Plant', 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'RegExp', 'on', ...
                'ReferenceBlock', '^nesl_utility/Solver.Configuration$');
            tc.verifyEqual(numel(sc), 1, 'exactly one Solver Configuration in Shaddad_Plant (stubs)');
            b = 'Shaddad_Plant/Solver Configuration';
            tc.verifyEqual(get_param(b, 'UseLocalSolver'), 'on');
            tc.verifyEqual(get_param(b, 'LocalSolverChoice'), 'NE_BACKWARD_EULER_ADVANCER');
            tc.verifyEqual(get_param(b, 'LocalSolverSampleTime'), 'P.sim.ssc.Ts');
            tc.verifyEqual(get_param(b, 'DoFixedCost'), 'on');
            % the Solver Configuration sits on the DC48_TracL_n node (Drive side port)
            pc = get_param(b, 'PortConnectivity');
            dst = getfullname([pc.DstBlock]);
            if ischar(dst), dst = {dst}; end
            tc.verifyTrue(any(strcmp(dst, 'Shaddad_Plant/Drive')) || any(strcmp(dst, 'Shaddad_Plant/Power')));
            refs = {'Structure', 'structure_plant'; 'Power', 'Power_Plant'; 'Drive', 'drive_system'; ...
                    'Electronics', 'Electronics_Plant'};
            for k = 1:size(refs, 1)
                tc.verifyEqual(get_param(['Shaddad_Plant/' refs{k, 1}], 'ReferencedSubsystem'), refs{k, 2});
            end
            z = {'ZOH_truth', 'Ts.base'; 'ZOH_tracks', 'Ts.base'; 'ZOH_struct_diag', 'Ts.ctrl'};
            for k = 1:size(z, 1)
                tc.verifyEqual(get_param(['Shaddad_Plant/' z{k, 1}], 'SampleTime'), z{k, 2});
            end
        end

        function testSystemStructure(tc)
            load_system('Shaddad_System');
            tc.verifyEqual(get_param('Shaddad_System/In_ScenarioIn', 'PortName'), 'ScenarioIn');
            tc.verifyEqual(get_param('Shaddad_System/In_ScenarioIn', 'OutDataTypeStr'), 'Bus: Bus_ScenarioIn');
            tc.verifyEqual(get_param('Shaddad_System/Spec_ScenarioIn', 'SampleTime'), 'Ts.scen');
            vs = 'Shaddad_System/Operator_Console/OpSource';
            tc.verifyEqual(get_param(vs, 'VariantControls'), {'VC_OPSRC == 1'; 'VC_OPSRC == 2'; 'VC_OPSRC == 3'; 'VC_OPSRC == 4'});
            tc.verifyEqual(get_param(vs, 'VariantActivationTime'), 'update diagram');
            g = 'Shaddad_System/GNC/GNC_Variant';
            tc.verifyEqual(get_param(g, 'VariantActivationTime'), 'update diagram');
            tc.verifyEqual(get_param([g '/GNC_Stub'], 'VariantControl'), 'VC_PHASE == 1');
            tc.verifyEqual(get_param([g '/Companion'], 'VariantControl'), 'VC_PHASE == 2');
            tc.verifyEqual(get_param([g '/Companion'], 'ModelName'), 'Shaddad_Companion');
            for m = {'MCU/Shaddad_MCU', 'BMS/Shaddad_BMS', 'Plant/Shaddad_Plant'}
                tc.verifyEqual(get_param(['Shaddad_System/' m{1}], 'SimulationMode'), 'Normal');
            end
            rt = 'Shaddad_System/GNC/MissionCmd_RT';
            tc.verifyEqual({get_param(rt, 'OutPortSampleTime'), get_param(rt, 'Integrity'), get_param(rt, 'Deterministic')}, ...
                {'Ts.mcmd', 'on', 'on'});
            % every Rate Transition of the top is explicit, integrity + deterministic
            rts = find_system('Shaddad_System', 'MatchFilter', @Simulink.match.allVariants, 'LookUnderMasks', 'all', ...
                'BlockType', 'RateTransition');
            for k = 1:numel(rts)
                tc.verifyEqual({get_param(rts{k}, 'Integrity'), get_param(rts{k}, 'Deterministic')}, {'on', 'on'}, rts{k});
            end
            load_system('Shaddad_Companion');
            ud = 'Shaddad_Companion/NavState_Delay';
            tc.verifyEqual({get_param(ud, 'SampleTime'), get_param(ud, 'InitialCondition')}, {'Ts.nav', 'P.ic.Bus_NavState'});
            ph = get_param('Shaddad_Companion/GNC', 'PortHandles');
            tc.verifyNumElements(ph.Inport, 6, 'Shaddad_GNC has 6 inputs (no plan port, D57)');
        end

        function testScnMakeInput(tc)
            P = evalin('base', 'P');
            tab = struct();
            tab.op.stick_thr = [0 0; 0.05 0.5; 0.123 -1];
            tab.hmi.estop_front_pressed = [0 0; 0.1 1];
            tab.obs.xy = [0 1:8; 0.07 11:18];
            tab.flt.imu_fault = [0 0; 0.08 3];
            ds = scn_make_input(P, tab, 0.2);
            tc.verifyClass(ds, 'Simulink.SimulationData.Dataset');
            tc.verifyEqual(ds.numElements, 1);
            tc.verifyEqual(ds.getElementNames, {'ScenarioIn'});
            s = ds{1};
            ts = s.op.stick_thr;
            tc.verifyEqual(ts.Time(:)', [0 0.05 0.05 0.12 0.12 0.2], 'AbsTol', 1e-12);
            tc.verifyEqual(ts.Data(:)', [0 0 0.5 0.5 -1 -1]);
            tc.verifyClass(s.hmi.estop_front_pressed.Data, 'logical');
            tc.verifyClass(s.flt.imu_fault.Data, 'uint8');
            tc.verifyEqual(size(s.obs.xy.Data), [4 2 4]);
            tc.verifyEqual(s.obs.xy.Data(:, :, end), reshape(11:18, 4, 2));
            % untouched leaf = ground value over [0 T]
            tc.verifyEqual(s.op.sw_arm.Time(:)', [0 0.2]);
            tc.verifyEqual(double(s.op.sw_arm.Data(:)'), [0 0]);
        end

        function testScnBaseInput(tc)
            P = evalin('base', 'P');
            in = scn_base_input(P, 'unit', 'StopTime', 3, 'Input', struct('op', struct('tx_power_on', [0 1])), ...
                'VC', struct('VC_PHASE', 2));
            tc.verifyClass(in, 'Simulink.SimulationInput');
            tc.verifyEqual(in.ModelName, 'Shaddad_System');
            vn = {in.Variables.Name};
            tc.verifyTrue(all(ismember({'P', 'Ts', 'VC_PHASE', 'VC_IMPL', 'VC_OPSRC'}, vn)));
            tc.verifyEqual(in.Variables(strcmp(vn, 'VC_PHASE')).Value, 2);
            tc.verifyEqual(in.getModelParameter('StopTime'), '3');
            tc.verifyClass(in.ExternalInput, 'Simulink.SimulationData.Dataset');
        end
    end

    %% ---------------------------------------------------------------- compile / simulate
    methods (Test, TestTags = {'Integration'})
        function testSystemPhase1(tc)
            tc.checkPhase(1);
        end

        function testSystemPhase2(tc)
            tc.checkPhase(2);
        end

        function testPlantVariantMatrix(tc)
            load_system('Shaddad_Plant');
            for impl = 0:3
                for fid = [1 3]
                    assignin('base', 'VC_IMPL', impl);
                    assignin('base', 'VC_DRIVE_FID', fid);
                    ok = true; msg = '';
                    try
                        evalc('set_param(''Shaddad_Plant'', ''SimulationCommand'', ''update'')');
                    catch tmp_ti_err
                        ok = false; msg = tmp_ti_err.message;
                    end
                    tc.verifyTrue(ok, sprintf('Shaddad_Plant update VC_IMPL %d VC_DRIVE_FID %d: %s', impl, fid, msg));
                    tc.verifyFalse(contains(msg, 'Solver Configuration'));
                end
            end
            assignin('base', 'VC_IMPL', tc.VC0.VC_IMPL);
            assignin('base', 'VC_DRIVE_FID', tc.VC0.VC_DRIVE_FID);
        end

        function testSim2sProfile(tc)
            pf = fullfile(tc.Root, 'work', 'profile_T01.html');
            if isfile(pf), delete(pf); end
            evalc('res = run_scenario(''T01smoke'', ''StopTime'', 2, ''Profile'', true, ''ProfileFile'', pf);');
            tc.verifyTrue(res.ok, res.msg);
            tc.verifyEqual(res.out.tout(end), 2, 'AbsTol', 1e-9);
            tc.verifyTrue(isfile(pf), 'Simulink Profiler report work/profile_T01.html');
            tc.log(1, sprintf('2 s simulation: %.2f s wall, RTF %.3f (Normal mode, stubs)', res.wall_s, res.rtf));
        end

        function testRunScenarioRef(tc)
            evalc('res = run_scenario(''T01ref'', ''StopTime'', 0.05, ''Solver'', ''ref'');');
            tc.verifyTrue(res.ok, res.msg);
            tc.verifyEqual(res.out.SimulationMetadata.ModelInfo.SolverInfo.Solver, 'daessc');
            tc.verifyEqual(get_param('Shaddad_Plant/Solver Configuration', 'UseLocalSolver'), 'on', 'restored');
            tc.verifyEqual(get_param(evalin('base', 'cs_Shaddad'), 'Solver'), 'ode3', 'cs_Shaddad restored');
            evalc('set_param(''Shaddad_System'', ''SimulationCommand'', ''update'')');   % no stale ConfigSetRef
        end

        function testMcuInFaultWordAndClock(tc)
            % fault_inject = task_stall | can1_busoff<<4 | can2_busoff<<5 | (imu_fault==3)<<6 ; t_us = t*1e6
            load_system('Shaddad_System');
            dirty0 = get_param('Shaddad_System', 'Dirty');
            ph = get_param('Shaddad_System/MCU/MCU_In', 'PortHandles');
            set_param(ph.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'T01_in1k');
            tc.addTeardown(@() t_infra.unlog(ph.Outport(1), dirty0));
            P = evalin('base', 'P');
            tab = struct();
            tab.flt.task_stall = [0 0; 0.05 5];
            tab.flt.can1_busoff = [0 0; 0.05 1];
            tab.flt.imu_fault = [0 0; 0.05 3];
            tab.flt.can2_busoff = [0 0; 0.08 1];
            in = scn_base_input(P, 'T01_mcu_in', 'StopTime', 0.12, 'Input', tab); %#ok<NASGU> used inside evalc
            evalc('out = sim(in);');
            el = out.logsout.get('T01_in1k');
            fi = el.Values.fault_inject;
            tu = el.Values.t_us;
            at = @(ts, t) ts.Data(find(abs(ts.Time - t) < 1e-9, 1));
            tc.verifyEqual(at(fi, 0.04), uint32(0));
            tc.verifyEqual(at(fi, 0.075), uint32(5 + 16 + 64));            % +1 slow sample (deterministic RT)
            tc.verifyEqual(at(fi, 0.105), uint32(5 + 16 + 32 + 64));
            tc.verifyEqual(at(tu, 0.1), uint32(100000));
            tc.verifyEqual(at(tu, 0.003), uint32(3000));
        end

        function testPlantInMapping(tc)
            % architecture 6.3: tau_scale = 1 - motor_fault, enc_fail / brake_stuck mapped per side, can_drop false
            load_system('Shaddad_Plant');
            dirty0 = get_param('Shaddad_Plant', 'Dirty');
            ph = get_param('Shaddad_Plant/Plant_In', 'PortHandles');
            set_param(ph.Outport(3), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'T01_drv_flt');
            tc.addTeardown(@() t_infra.unlog(ph.Outport(3), dirty0));
            P = evalin('base', 'P');
            tab = struct();
            tab.flt.motor_fault_L = [0 0; 0.05 1];
            tab.flt.enc_fault_R = [0 0; 0.03 1];
            tab.flt.brake_stuck_L = [0 0; 0.02 1];
            ds = scn_make_input(P, tab, 0.1);
            T = build_stubs('table');
            R = T(strcmp({T.name}, 'Shaddad_Plant'));
            for k = 2:numel(R.in)
                bus = strtrim(extractAfter(R.in(k).type, 'Bus:'));
                ds = ds.addElement(Simulink.SimulationData.createStructOfTimeseries(bus, ...
                    t_infra.zeroTs(P.ic.(bus), 0.1)), R.in(k).name);
            end
            in = Simulink.SimulationInput('Shaddad_Plant');
            in = in.setModelParameter('StopTime', '0.1');
            in = in.setExternalInput(ds); %#ok<NASGU> used inside evalc
            evalc('out = sim(in);');
            v = out.logsout.get('T01_drv_flt').Values;
            at = @(ts, t) ts.Data(find(abs(ts.Time - t) < 1e-9, 1));
            tc.verifyEqual(at(v.L.tau_scale, 0.04), 1);
            tc.verifyEqual(at(v.L.tau_scale, 0.065), 0);
            tc.verifyEqual(at(v.R.tau_scale, 0.065), 1);
            tc.verifyEqual(at(v.R.enc_fail, 0.045), true);
            tc.verifyEqual(at(v.L.enc_fail, 0.045), false);
            tc.verifyEqual(at(v.L.brake_stuck, 0.035), true);
            tc.verifyEqual(at(v.R.brake_stuck, 0.035), false);
            tc.verifyEqual(at(v.L.can_drop, 0.09), false);
        end
    end

    %% ---------------------------------------------------------------- build scripts
    methods (Test, TestTags = {'Build'})
        function testBuildScripts(tc)
            % build_project, build_stubs(true), build_shaddad_plant/companion/system run without error,
            % regenerate every T01 file at its table path and leave the integration models clean
            T = build_stubs('table');
            names = {T.name};
            % the build closes (without saving) every model it regenerates: never discard unsaved work of
            % a model that was already loaded before this test class started
            dirty = names(cellfun(@(n) any(strcmp(tc.Loaded, n)) && bdIsLoaded(n) && ...
                strcmp(get_param(n, 'Dirty'), 'on'), names));
            tc.assumeEmpty(dirty, ['models with unsaved changes are loaded: ' strjoin(dirty, ', ')]);
            loaded0 = names(cellfun(@bdIsLoaded, names));
            tc.addTeardown(@() t_infra.reload(loaded0));
            tStart = datetime('now') - seconds(2);          % file-time resolution
            steps = {'build_project', @build_project; 'build_stubs(true)', @() build_stubs(true); ...
                     'build_shaddad_plant', @build_shaddad_plant; 'build_shaddad_companion', @build_shaddad_companion; ...
                     'build_shaddad_system', @build_shaddad_system};
            tAll = tic;
            for k = 1:size(steps, 1)
                ok = true; msg = ''; t1 = tic;
                try
                    evalc('feval(steps{k, 2});');
                catch tmp_ti_err
                    ok = false; msg = tmp_ti_err.message;
                end
                tc.log(1, sprintf('%s: %.1f s', steps{k, 1}, toc(t1)));
                tc.verifyTrue(ok, sprintf('%s failed: %s', steps{k, 1}, msg));
            end
            tc.log(1, sprintf('all build scripts: %.1f s', toc(tAll)));
            nRegen = 0;
            for k = 1:numel(T)
                f = fullfile(tc.Root, T(k).file);
                tc.verifyTrue(isfile(f), ['missing ' T(k).file]);
                if ~isfile(f), continue, end
                w = which(T(k).name);
                tc.verifyTrue(strcmpi(strrep(w, '/', filesep), strrep(f, '/', filesep)), ...
                    sprintf('%s resolves to %s, table path %s', T(k).name, w, T(k).file));
                info = Simulink.MDLInfo(f);
                if ~T(k).stub || strcmp(strtrim(info.Description), 'SHADDAD_STUB')
                    % integration model or (still) a stub: regenerated by this test
                    d = dir(f);
                    tc.verifyGreaterThanOrEqual(datetime(d.datenum, 'ConvertFrom', 'datenum'), tStart, ...
                        [T(k).file ' was not regenerated']);
                    nRegen = nRegen + 1;
                else
                    tc.log(1, sprintf('%s: replaced by %s (not a stub), left untouched', T(k).name, T(k).owner));
                end
            end
            tc.log(1, sprintf('%d of %d table files regenerated', nRegen, numel(T)));
            mdls = {'Shaddad_Plant', 'Shaddad_Companion', 'Shaddad_System'};
            for m = mdls
                if bdIsLoaded(m{1})
                    tc.verifyEqual(get_param(m{1}, 'Dirty'), 'off', [m{1} ' is dirty after its build']);
                end
            end
            % a later compile keeps the freshly built models clean (shaddad_bu('finalize'))
            for phase = [1 2]
                assignin('base', 'VC_PHASE', phase);
                evalc('set_param(''Shaddad_System'', ''SimulationCommand'', ''update'')');
                for m = mdls
                    if bdIsLoaded(m{1})
                        tc.verifyEqual(get_param(m{1}, 'Dirty'), 'off', sprintf('%s dirty after update (VC_PHASE %d)', m{1}, phase));
                    end
                end
            end
            assignin('base', 'VC_PHASE', tc.VC0.VC_PHASE);
        end
    end

    %% ---------------------------------------------------------------- start-up in a fresh process / codegen sets
    methods (Test, TestTags = {'Cold'})
        function testStartupFreshProcess(tc)
            % project start-up as on project open: fresh MATLAB process, project path, shaddad_startup
            body = { ...
                'R.rep = shaddad_startup(''quiet'');'
                'R.nbus = numel(who(''Bus_*''));'
                'R.vars = who;'
                'R.cls = class(cs_Shaddad);'
                'R.solver = get_param(cs_Shaddad, ''Solver'');'
                'R.h7 = get_param(cs_STM32H7, ''HardwareBoard'');'};
            [R, wall] = t_infra.fresh(tc, body);
            tc.assertFalse(isfield(R, 'error'), sprintf('fresh-process start-up failed: %s', t_infra.field(R, 'error')));
            s = R.rep.t_steps;
            tc.log(1, sprintf(['fresh process: shaddad_startup %.1f s = Simulink product start %.1f s + project %.1f s ' ...
                '(params %.1f, config sets %.1f, report %.1f, SDI %.1f); process wall %.0f s'], R.rep.t_startup, ...
                s.simulink, R.rep.t_project, s.params, s.configsets, s.report, s.sdi, wall));
            tc.verifyLessThan(R.rep.t_project, 15, 'project start-up (without the Simulink product start) < 15 s');
            tc.verifyEqual(R.nbus, 87);
            tc.verifyTrue(all(ismember({'P', 'Ts', 'P_meta', 'cs_SimFixed', 'cs_SimRef', 'cs_GenARM', 'cs_STM32H7', ...
                'cs_STM32G4', 'cs_Companion', 'cs_Shaddad'}, R.vars)));
            tc.verifyEqual(R.cls, 'Simulink.ConfigSet');
            tc.verifyEqual(R.solver, 'ode3');
            tc.verifyEqual(numel(R.rep.deferred), 3, 'HardwareBoard deferred in a fresh process');
            tc.verifyEqual(R.rep.codegen_blocked, ~any(strcmp({ver().Name}, 'Simulink Coder')));
        end
    end

    methods (Test, TestTags = {'Slow'})
        function testCodegenSetsRebuild(tc)
            % shaddad_configsets('codegen') builds the complete sets on a cache miss, writes the cache,
            % then loads them from it; a temporary cache file keeps the project cache untouched
            tmpc = [tempname '.mat'];
            old = getenv('SHADDAD_CS_CACHE');
            setenv('SHADDAD_CS_CACHE', tmpc);
            tc.addTeardown(@() setenv('SHADDAD_CS_CACHE', old));
            tc.addTeardown(@() t_infra.delfile(tmpc));
            t1 = tic; r = shaddad_configsets('codegen'); dt = toc(t1);
            tc.log(1, sprintf('codegen sets rebuilt (cache miss, running session): %.1f s', dt));
            tc.verifyEqual(r.cache, 'rebuilt');
            tc.verifyTrue(isfile(tmpc), 'cache written');
            tc.verifyEmpty(r.deferred);
            exp = {'cs_GenARM', 'SystemTargetFile', 'ert.tlc'; 'cs_GenARM', 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M'; ...
                   'cs_GenARM', 'GenCodeOnly', 'on'; 'cs_STM32H7', 'HardwareBoard', 'STM32H7xx Based (Single-core)'; ...
                   'cs_STM32G4', 'HardwareBoard', 'STM32G4xx Based'; ...
                   'cs_Companion', 'HardwareBoard', 'Robot Operating System 2 (ROS 2)'; 'cs_Companion', 'TargetLang', 'C++'};
            for k = 1:size(exp, 1)
                cs = evalin('base', exp{k, 1});
                if any(contains(r.blocked, [exp{k, 1} '.' exp{k, 2}]))
                    tc.log(1, ['BLOCKED: ' exp{k, 1} '.' exp{k, 2}]);
                    continue
                end
                tc.verifyEqual(get_param(cs, exp{k, 2}), exp{k, 3}, [exp{k, 1} '.' exp{k, 2}]);
                tc.verifyEqual(get_param(cs, 'DefaultParameterBehavior'), 'Inlined', exp{k, 1});
            end
            t1 = tic; r2 = shaddad_configsets('codegen'); dt2 = toc(t1);
            tc.log(1, sprintf('codegen sets from the cache: %.1f s', dt2));
            tc.verifyEqual(r2.cache, 'hit');
            r3 = shaddad_configsets('fixed');
            tc.verifyEqual(r3.cache, 'hit', 'after codegen, later fixed/ref calls keep the complete sets');
            tc.verifyEmpty(r3.deferred);
        end
    end

    %% ---------------------------------------------------------------- code generation (BLOCKED, D31)
    methods (Test, TestTags = {'Codegen'})
        function testCodegenGenARM(tc)
            tc.assumeTrue(any(strcmp({ver().Name}, 'Simulink Coder')), 'BLOCKED: Simulink Coder not installed');
            shaddad_configsets('codegen');          % first codegen use: complete codegen sets
            cs0 = evalin('base', 'cs_Shaddad');
            tc.addTeardown(@() assignin('base', 'cs_Shaddad', cs0));
            cs = copy(evalin('base', 'cs_GenARM')); cs.Name = 'cs_Shaddad';
            assignin('base', 'cs_Shaddad', cs);
            evalc('slbuild(''Shaddad_MCU'');');
            tc.verifyTrue(isfolder(fullfile(tc.Root, 'work')));
        end
    end

    %% ---------------------------------------------------------------- helpers
    methods
        function checkPhase(tc, phase)
            assignin('base', 'VC_PHASE', phase);
            load_system('Shaddad_System');
            t0 = tic;
            evalc('set_param(''Shaddad_System'', ''SimulationCommand'', ''update'')');
            dt = toc(t0);
            tc.log(1, sprintf('VC_PHASE %d: update %.1f s', phase, dt));
            tc.verifyLessThan(dt, 180, 'update diagram must take < 180 s');
            tc.verifyEqual(t_infra.loops('Shaddad_System'), 0, 'algebraic loops');
            evalc('r = shaddad_rate_audit(''Shaddad_System'');');
            tc.verifyEmpty(r.bad, 'compiled rates outside P.sim.Ts_compiled_allowed');
            tc.verifyEmpty(r.rc_outside, '0.007 / 0.014 s outside RC_Link_Sim');
            tc.verifyTrue(all(strcmp(r.autoRT, 'off')), 'AutoInsertRateTranBlk must be off (no implicit RTs)');
            tc.verifyTrue(all(ismember(r.rates, r.allowed)));
            if phase == 2
                tc.verifyTrue(any(strcmp(r.models, 'Shaddad_Companion')));
            else
                tc.verifyFalse(any(strcmp(r.models, 'Shaddad_Companion')));
            end
            [n, rep] = shaddad_check_interfaces('Compiled', true, 'Quiet', true);
            for k = find(~[rep.ok])
                tc.log(1, sprintf('%s: %s', rep(k).name, strjoin(rep(k).mismatch, '; ')));
            end
            tc.verifyEqual(n, 0, 'compiled interface check');
            assignin('base', 'VC_PHASE', tc.VC0.VC_PHASE);
        end
    end

    methods (Static)
        function unlog(ph, dirty0)
            try
                set_param(ph, 'DataLogging', 'off');
                if strcmp(dirty0, 'off'), set_param(bdroot(get_param(ph, 'Parent')), 'Dirty', 'off'); end
            catch %#ok<CTCH>
            end
        end

        function reload(names)
            % load again the models that were loaded before a build closed them
            for k = 1:numel(names)
                if ~bdIsLoaded(names{k})
                    try, load_system(names{k}); catch, end %#ok<CTCH>
                end
            end
        end

        function delfile(f)
            if isfile(f), delete(f); end
        end

        function deldir(d)
            if isfolder(d), rmdir(d, 's'); end
        end

        function v = field(S, f)
            if isfield(S, f), v = S.(f); else, v = ''; end
        end

        function [R, wall] = fresh(tc, body)
            % Run the MATLAB lines 'body' in a fresh MATLAB process (matlab -batch) with the project path
            % (the project itself is not opened, so the shared session's project is never touched); the
            % lines fill a struct R that is returned. wall = process wall time in seconds.
            d = tempname; mkdir(d);
            tc.addTeardown(@() t_infra.deldir(d));
            resFile = fullfile(d, 'res.mat');
            pp = arrayfun(@(p) char(p.File), currentProject().ProjectPath, 'UniformOutput', false);
            q = @(s) strrep(s, '''', '''''');
            L = {'R = struct();', 'try'};
            for k = 1:numel(pp), L{end+1} = sprintf('    addpath(''%s'');', q(pp{k})); end %#ok<AGROW>
            L = [L, strcat({'    '}, body(:)'), {'catch tmp_err', ...
                 '    R = struct(''error'', getReport(tmp_err, ''basic''));', 'end', ...
                 sprintf('save(''%s'', ''R'');', q(resFile))}];
            fid = fopen(fullfile(d, 't01_fresh.m'), 'w');
            fprintf(fid, '%s\n', L{:});
            fclose(fid);
            if ispc, exe = fullfile(matlabroot, 'bin', 'matlab.exe'); else, exe = fullfile(matlabroot, 'bin', 'matlab'); end
            cmd = sprintf('"%s" -sd "%s" -batch t01_fresh', exe, d);
            t = tic;
            [st, out] = system(cmd);
            wall = toc(t);
            if isfile(resFile)
                S = load(resFile, 'R');
                R = S.R;
            else
                R = struct('error', sprintf('no result from the fresh process (exit %d): %s', st, out));
            end
        end
    end
end
