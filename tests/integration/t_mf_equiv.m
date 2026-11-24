classdef t_mf_equiv < matlab.unittest.TestCase
    %T_MF_EQUIV  Bit-identical checks of the MATLAB Function conversions (D70, plan 2026-11-01).
    %
    %   shaddad_run_tests('t_mf_equiv'), or runtests('tests/integration/t_mf_equiv.m'). Tag 'Fast'.
    %
    %   The tool (Task 1): utilities/mf_equiv runs two versions of a subsystem side by side on the same inputs
    %   and asks for bit-identical outputs. The model of these tests is built here and saved in a temporary
    %   folder, under one name in two subfolders (like a task snapshot and its rebuilt file): subsystem Sub
    %   holds 3 blocks, In1 -> Gain -> Out1.
    %   testSameCopies      the subsystem against its own copy: same over all 101 samples
    %   testOneGainChanged  the Gain 2 of the second file is 2.0000001: not same, and the first difference is
    %                       output 1 at t = 0
    %   Collision monitors (Task 2), subsystems/integration/Sim_Monitors.slx:
    %   testMonitorsUseFunctions  FE_Cyl, FE_Box and FE_Dyn each hold exactly one MATLAB Function, which calls
    %                       mon_sep_circle( (Cyl, Dyn) or mon_sep_rect( (Box) and holds a current verbatim copy
    %                       of that file (the R2026b JIT workaround of build_sim_monitors), and no block network:
    %                       besides it only the ports, the For Each block and the Constant hA
    %   testMonitorsEquivalent    each of the three against its copy in the task-2 snapshot
    %                       (work/agentic/snap/mfplan/task-2), with mf_equiv: 500 samples of random tables
    %                       (fixed seed; x and y within +/-20 m, sizes 0.2-3 m, yaw +/-pi; a random footprint
    %                       pose) and the edge rows of the plan's Review Focus 1 (rows that just touch the
    %                       footprint, zero-size rows), at two footprint poses. Before the conversion it compares
    %                       the file with itself; after it, it guards the rebuild.
    %   drive_esc Feedback and Torque_Limiter (Task 3), subsystems/drive/drive_esc.slx:
    %   testDriveEscUsesFunctions  Feedback holds one MATLAB Function, FbPack, which calls esc_feedback( and holds
    %                       a current verbatim copy of subsystems/drive/esc_feedback.m (the R2026b JIT workaround
    %                       of build_drive); Torque_Limiter one, LimitLogic, with esc_torque_limit. Besides it only
    %                       the ports and the kept blocks: in Feedback the Constant FbPar and the alive tick Tick
    %                       (Counter Limited, a state); in Torque_Limiter the Constant TlPar, the speed in rpm in
    %                       front of the Lookup Tables (AbsW, ToRpm), the six Lookup Tables, the I2t integrator and
    %                       its Relay Exhausted
    %   testDriveEscEquivalent    each of the two against its copy in the task-3 snapshot
    %                       (work/agentic/snap/mfplan/task-3), with mf_equiv: Feedback twice (the input types of
    %                       drive_esc; then the status inputs derating, lim_active and stall_warn as doubles, so
    %                       that their conversions to uint16 see ties, values past the limits and non-finite
    %                       values), Torque_Limiter once; 2000 or more random samples with a fixed seed and the
    %                       edge samples of the plan's Review Focus 2 and 3 (see escInput); same, and no zero of
    %                       the opposite sign. Before the conversion it compares the file with itself; after it,
    %                       it guards the rebuild.

    methods (TestClassSetup)
        function setup(~)
            if ~evalin('base', 'exist(''P'', ''var'')'), shaddad_startup('quiet'); end
        end
    end

    methods (Test, TestTags = {'Fast'})
        function testSameCopies(tc)
            tmp = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            f = t_mf_equiv.gainModel(tmp, 'a', '2');
            r = mf_equiv(f, f, 'Sub', t_mf_equiv.rampInput());
            tc.verifyTrue(r.same, r.firstDiff);
            tc.verifyEqual(r.nSamples, 101);
            tc.verifyEqual(r.firstDiff, '');
        end

        function testOneGainChanged(tc)
            tmp = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            fOld = t_mf_equiv.gainModel(tmp, 'a', '2');
            fNew = t_mf_equiv.gainModel(tmp, 'b', '2.0000001');
            r = mf_equiv(fOld, fNew, 'Sub', t_mf_equiv.rampInput());
            tc.verifyFalse(r.same);
            tc.verifyEqual(r.nSamples, 101);
            tc.verifyNotEmpty(regexp(r.firstDiff, '^output 1 \(.*\), t = 0,', 'once'), r.firstDiff);
        end

        function testMonitorsUseFunctions(tc)
            m = 'Sim_Monitors';
            t_mf_equiv.loadModel(tc, fullfile(t_mf_equiv.root(), 'subsystems', 'integration', [m '.slx']));
            want = {'FE_Cyl', 'mon_sep_circle('; 'FE_Box', 'mon_sep_rect('; 'FE_Dyn', 'mon_sep_circle('};
            for k = 1:size(want, 1)
                fe = [m '/' want{k, 1}];
                mf = find_system(fe, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'BlockType', 'SubSystem', ...
                    'SFBlockType', 'MATLAB Function');
                tc.verifyNumElements(mf, 1, [want{k, 1} ': MATLAB Function blocks']);
                if isscalar(mf)
                    cfg = get_param(mf{1}, 'MATLABFunctionConfiguration');
                    tc.verifyTrue(contains(cfg.FunctionScript, want{k, 2}), ...
                        sprintf('%s calls: %s', want{k, 1}, cfg.FunctionScript));
                    % the block holds a verbatim copy of the function file (R2026b JIT workaround): it must be current
                    f = fullfile(t_mf_equiv.root(), 'subsystems', 'integration', [extractBefore(want{k, 2}, '(') '.m']);
                    body = strrep(fileread(f), sprintf('\r\n'), newline);
                    tc.verifyTrue(contains(strrep(cfg.FunctionScript, sprintf('\r\n'), newline), body), ...
                        sprintf('%s: the copy of %s in the block is not current: rebuild (build_sim_monitors)', ...
                        want{k, 1}, f));
                end
                blk = setdiff(find_system(fe, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'Type', 'block'), [{fe}; mf]);
                rest = setdiff(cellstr(get_param(blk, 'BlockType')), {'Inport', 'Outport', 'ForEach', 'Constant'});
                tc.verifyEmpty(rest, sprintf('%s: blocks besides the MATLAB Function: %s', want{k, 1}, strjoin(rest, ', ')));
            end
        end

        function testMonitorsEquivalent(tc)
            root = t_mf_equiv.root();
            snap = fullfile(root, 'work', 'agentic', 'snap', 'mfplan', 'task-2', 'subsystems', 'integration', ...
                'Sim_Monitors.slx');
            tc.assumeTrue(isfile(snap), ['no task-2 snapshot: ' snap]);
            cur = fullfile(root, 'subsystems', 'integration', 'Sim_Monitors.slx');
            mon = evalin('base', 'P.sim.mon');
            ts = evalin('base', 'Ts.monitor');
            cases = {'FE_Cyl', 'circle', mon.Kc; 'FE_Box', 'rect', mon.Kb; 'FE_Dyn', 'circle', mon.Nd};
            for k = 1:size(cases, 1)
                U = t_mf_equiv.monitorInput(cases{k, 2}, cases{k, 3}, mon, ts);
                r = mf_equiv(snap, cur, cases{k, 1}, U);
                tc.verifyTrue(r.same, sprintf('%s: %s', cases{k, 1}, r.firstDiff));
                tc.verifyEqual(r.nSamples, numel(U.getElement(1).Time), cases{k, 1});
                fprintf('%s: same %d over %d samples, %d zeros of opposite sign\n', cases{k, 1}, r.same, ...
                    r.nSamples, r.nZeroSign);
                tc.verifyEqual(r.nZeroSign, 0, sprintf('%s: zeros of opposite sign (Review Focus 1, the sign of sep)', cases{k, 1}));
            end
        end

        function testDriveEscUsesFunctions(tc)
            m = 'drive_esc';
            root = t_mf_equiv.root();
            t_mf_equiv.loadModel(tc, fullfile(root, 'subsystems', 'drive', [m '.slx']));
            want = {'Feedback', 'FbPack', 'esc_feedback', {'FbPar'; 'Tick'}; ...
                'Torque_Limiter', 'LimitLogic', 'esc_torque_limit', {'AbsW'; 'EnvCont'; 'EnvPeak'; 'Exhausted'; ...
                'I2t'; 'TlPar'; 'ToRpm'; 'f_Te'; 'f_Tm'; 'f_ov'; 'f_uv'}};
            for k = 1:size(want, 1)
                s = [m '/' want{k, 1}];
                mf = find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'BlockType', 'SubSystem', ...
                    'SFBlockType', 'MATLAB Function');
                names = {};
                if ~isempty(mf), names = cellstr(get_param(mf, 'Name')); end
                tc.verifyEqual(names, want(k, 2), [want{k, 1} ': its MATLAB Function blocks']);
                if isscalar(mf)
                    cfg = get_param(mf{1}, 'MATLABFunctionConfiguration');
                    script = strrep(cfg.FunctionScript, sprintf('\r\n'), newline);
                    tc.verifyTrue(contains(script, [want{k, 3} '(']), sprintf('%s calls: %s', want{k, 2}, script));
                    % the block holds a verbatim copy of the function file (R2026b JIT workaround): it must be current
                    f = fullfile(root, 'subsystems', 'drive', [want{k, 3} '.m']);
                    body = strrep(fileread(f), sprintf('\r\n'), newline);
                    tc.verifyTrue(contains(script, body), sprintf(['%s: the copy of %s in the block is not ' ...
                        'current: rebuild (build_drive({''esc''}))'], want{k, 2}, f));
                end
                blk = setdiff(find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'Type', 'block'), [{s}; mf]);
                bt = cellstr(get_param(blk, 'BlockType'));
                rest = sort(cellstr(get_param(blk(~ismember(bt, {'Inport', 'Outport'})), 'Name')));
                tc.verifyEqual(rest, sort(want{k, 4}), sprintf('%s: the blocks besides the ports and %s', ...
                    want{k, 1}, want{k, 2}));
            end
        end

        function testDriveEscEquivalent(tc)
            root = t_mf_equiv.root();
            snap = fullfile(root, 'work', 'agentic', 'snap', 'mfplan', 'task-3', 'subsystems', 'drive', 'drive_esc.slx');
            tc.assumeTrue(isfile(snap), ['no task-3 snapshot: ' snap]);
            cur = fullfile(root, 'subsystems', 'drive', 'drive_esc.slx');
            P = evalin('base', 'P');
            t_mf_equiv.escBuses(tc);
            cases = {'Feedback', 'model'; 'Feedback', 'cast'; 'Torque_Limiter', 'model'};
            for k = 1:size(cases, 1)
                [U, types] = t_mf_equiv.escInput(cases{k, 1}, cases{k, 2}, P);
                r = mf_equiv(snap, cur, cases{k, 1}, U, 'InportTypes', types);
                lab = sprintf('%s (%s inputs)', cases{k, 1}, cases{k, 2});
                tc.verifyTrue(r.same, sprintf('%s: %s', lab, r.firstDiff));
                tc.verifyEqual(r.nSamples, numel(U.getElement(1).mode.Time), lab);
                fprintf('%s: same %d over %d samples, %d zeros of opposite sign\n', lab, r.same, r.nSamples, ...
                    r.nZeroSign);
                tc.verifyEqual(r.nZeroSign, 0, sprintf('%s: zeros of opposite sign', lab));
            end
        end
    end

    methods (Static)
        function f = gainModel(tmp, folder, gain)
            % tmp/folder/mfeq_t.slx: root In1 -> Sub (In1 -> Gain gain -> Out1) -> root Out1; closed again
            d = fullfile(tmp, folder);
            if ~isfolder(d), mkdir(d); end
            m = 'mfeq_t';
            if bdIsLoaded(m), close_system(m, 0); end
            new_system(m);
            closeIt = onCleanup(@() close_system(m, 0));
            add_block('simulink/Sources/In1', [m '/In1']);
            add_block('simulink/Ports & Subsystems/Subsystem', [m '/Sub']);
            delete_line([m '/Sub'], 'In1/1', 'Out1/1');
            add_block('simulink/Math Operations/Gain', [m '/Sub/Gain'], 'Gain', gain);
            add_line([m '/Sub'], 'In1/1', 'Gain/1');
            add_line([m '/Sub'], 'Gain/1', 'Out1/1');
            add_block('simulink/Sinks/Out1', [m '/Out1']);
            add_line(m, 'In1/1', 'Sub/1');
            add_line(m, 'Sub/1', 'Out1/1');
            f = fullfile(d, [m '.slx']);
            save_system(m, f);
            clear closeIt
        end

        function U = rampInput()
            % u = 1 + t on t = 0:0.01:1 (101 samples), so the outputs of the two Gains differ from t = 0 on
            t = (0:100)'*0.01;
            U = Simulink.SimulationData.Dataset;
            U = U.addElement(timeseries(1 + t, t), 'u');
        end

        function U = monitorInput(kind, K, mon, ts)
            % inputs row (K x 3 circles or K x 5 boxes), p (2 x 1), R (2 x 2) of a For Each FE_*, every ts s:
            % 500 samples of random tables (own random stream, fixed seed; the global one is left alone), then
            % the edge rows at the footprint pose 0 and at a quarter turn, K rows a sample, padded with the far
            % rows of sim_monitor_obstacles
            rs = RandStream('twister', 'Seed', 20261114);
            circle = strcmp(kind, 'circle');
            nCol = 5 - 2*circle;
            N = 500;
            T = zeros(K, nCol, N);
            p = zeros(2, 1, N);
            R = zeros(2, 2, N);
            for i = 1:N
                xy = 40*rand(rs, K, 2) - 20;
                yaw = pi*(2*rand(rs, K, 1) - 1);
                sz = 0.2 + 2.8*rand(rs, K, 2);
                if circle, T(:, :, i) = [xy, sz(:, 1)]; else, T(:, :, i) = [xy, yaw, sz]; end
                p(:, :, i) = 4*rand(rs, 2, 1) - 2;
                psi = pi*(2*rand(rs) - 1);
                R(:, :, i) = [cos(psi), -sin(psi); sin(psi), cos(psi)];
            end
            % Review Focus 1, in the body frame: a = half length, b = half width of the footprint
            a = mon.footprint(1)/2;
            b = mon.footprint(2)/2;
            if circle
                E = [2*a, 0, a; -2*a, 0, a; 0, 2*b, b; 0, -2*b, b; ...    % touch an edge: sep = 0
                     a, b, 0; a, 0, 0; 0, 0, 0; 0.1, -0.2, 0; 3, 4, 0; ...  % zero size: corner, edge, centre, in, out
                     0, 0, 1; 2*a + 1e-12, 0, a; 2*a - 1e-12, 0, a];        % deep inside, just apart, just touching
                pad = [mon.far + 10, mon.far, 0.01];
            else
                E = [2*a, 0, 0, a, 0.3; 0, 2*b, 0, 0.4, b; 2*a, 2*b, 0, a, b; ...   % edge-edge x, y; corner-corner
                     a + 0.5*sqrt(2), 0, pi/4, 0.5, 0.5; ...                       % a corner at the front edge
                     a, 0, 0, 0, 0; a, b, 0, 0, 0; 0, 0, 0, 0, 0; 0.1, 0.2, 0.3, 0, 0; 3, 4, 0.3, 0, 0; ...
                     0, 0, 0.5, 1, 1; 0, 0, 0, 5, 5; ...                           % deep overlap, around it
                     2*a + 1e-12, 0, 0, a, 0.3; 2*a - 1e-12, 0, 0, a, 0.3];        % just apart, just overlapping
                pad = [mon.far + 10, mon.far, 0, 0.01, 0.01];
            end
            nE = ceil(size(E, 1)/K);
            E = [E; repmat(pad, nE*K - size(E, 1), 1)];
            poses = {[0; 0], eye(2); [1; -2], [0, -1; 1, 0]};
            for k = 1:size(poses, 1)
                [pk, Rk] = poses{k, :};
                Ew = E;
                Ew(:, 1:2) = (pk + Rk*E(:, 1:2).').';
                if ~circle, Ew(:, 3) = E(:, 3) + (k - 1)*pi/2; end
                for j = 1:nE
                    T(:, :, end + 1) = Ew((j - 1)*K + (1:K), :); %#ok<AGROW>
                    p(:, :, end + 1) = pk; %#ok<AGROW>
                    R(:, :, end + 1) = Rk; %#ok<AGROW>
                end
            end
            % Review Focus 2 (added by the lead after the review of 2026-11-14, from the Task 2 dry run): one non-finite
            % entry a sample, NaN or Inf in a row, then NaN in p and in R; MinMax and MATLAB min / max skip NaN, the
            % one place where the blocks and the code could differ
            if circle
                base = [1.5, 0.3, 0.4];
                bad = {[NaN 0.3 0.4], [1.5 NaN 0.4], [1.5 0.3 NaN], [Inf 0.3 0.4], [-Inf 0.3 0.4], [1.5 0.3 Inf]};
            else
                base = [1.5, 0.3, 0.2, 0.4, 0.3];
                bad = {[NaN 0.3 0.2 0.4 0.3], [1.5 0.3 NaN 0.4 0.3], [1.5 0.3 0.2 NaN 0.3], [Inf 0.3 0.2 0.4 0.3], ...
                    [1.5 0.3 Inf 0.4 0.3], [1.5 0.3 0.2 0.4 Inf]};
            end
            for i = 1:numel(bad) + 2
                Tb = repmat(base, K, 1);  pb = [0; 0];  Rb = eye(2);
                if i <= numel(bad), Tb(1, :) = bad{i}; elseif i == numel(bad) + 1, pb = [NaN; 0]; else, Rb = [NaN 0; 0 1]; end
                T(:, :, end + 1) = Tb; %#ok<AGROW>
                p(:, :, end + 1) = pb; %#ok<AGROW>
                R(:, :, end + 1) = Rb; %#ok<AGROW>
            end
            t = (0:size(T, 3) - 1)'*ts;
            U = Simulink.SimulationData.Dataset;
            U = U.addElement(timeseries(T, t), 'row');
            U = U.addElement(timeseries(p, t), 'p');
            U = U.addElement(timeseries(R, t), 'R');
        end

        function escBuses(tc)
            % temporary bus objects (base workspace, removed after the test) for the harness inports cmd_in and sm:
            % the elements that the In Bus Element ports of Feedback and Torque_Limiter pick, with their drive_esc
            % types (cmd_in from Cmd_In, sm from ESC_SM / esc_sm.m)
            b = {'Bus_MfeqEscCmd', {'mode', 'double'; 'tau_lim_mot', 'double'; 'tau_lim_reg', 'double'}; ...
                'Bus_MfeqEscSm', {'state', 'uint8'; 'fault_word', 'uint16'; 'brake_released', 'boolean'}};
            for k = 1:size(b, 1)
                nm = b{k, 1};
                tc.assertEqual(evalin('base', sprintf('exist(''%s'', ''var'')', nm)), 0, [nm ' exists already']);
                el = b{k, 2};
                for j = 1:size(el, 1)
                    x = Simulink.BusElement;
                    x.Name = el{j, 1};
                    x.DataType = el{j, 2};
                    if j == 1, es = x; else, es(j, 1) = x; end
                end
                o = Simulink.Bus;
                o.Elements = es;
                assignin('base', nm, o);
                tc.addTeardown(@() evalin('base', ['clear ' nm]));
            end
        end

        function [U, types] = escInput(sub, kind, P)
            % inputs of drive_esc/<sub> for testDriveEscEquivalent, every P.drive.Ts_esc s, in port order (cmd_in
            % and sm as structures of timeseries, of the buses of escBuses), and the harness types of its inports.
            % Own random stream (fixed seed; the global one is left alone).
            %   Feedback, kind 'model': the types drive_esc gives every input; kind 'cast': the status inputs
            %     derating, lim_active and stall_warn as doubles. 2000 random samples, the edge samples, then 600
            %     random samples more (the alive tick wraps after 256*n_alive = 2560 samples). The edges: every
            %     state 0-8 with both modes; fault words with and without bit 10 (STO_ACTIVE); odd modes (NaN, 0,
            %     1.5, 3, -0); enc_cnt at its int32 limits; each of the eight measured inputs at NaN, +-Inf, 0, -0,
            %     past the range of single, at single rounding ties and in the subnormal range (v_dc = 0 among
            %     them); each status input at its conversion edges (castEdges) or at both values.
            %   Torque_Limiter ('model'): 1000 random samples with |tau_applied| below 0.95 of the continuous rating
            %     (the I2t stays 0, the Relay Exhausted off), then the edge samples with tau_applied 0; 300 samples
            %     at 500 rad/s and 20 N m, which exhaust the boost (the Relay on); 1000 random samples over the full
            %     ranges (the Relay stays on: x falls at most 1/t_recover per second) and the edge samples (tlEdges);
            %     then a NaN in each input in turn (tau_lim_mot, tau_lim_reg, v_dc, T_mot_meas, T_esc, then w_meas
            %     and tau_applied, which can make the I2t state NaN) and 20 random samples.
            rs = RandStream('twister', 'Seed', 20261143);
            ts = P.drive.Ts_esc;
            e = P.drive.esc;
            mo = P.drive.motor;
            U = Simulink.SimulationData.Dataset;
            if strcmp(sub, 'Feedback')
                cast = strcmp(kind, 'cast');
                S = t_mf_equiv.catRows(t_mf_equiv.fbRows(rs, 2000, cast), t_mf_equiv.fbEdges(cast), ...
                    t_mf_equiv.fbRows(rs, 600, cast));
                t = (0:numel(S.mode) - 1)'*ts;
                z = zeros(size(t));
                U = U.addElement(struct('mode', timeseries(S.mode, t), 'tau_lim_mot', timeseries(z, t), ...
                    'tau_lim_reg', timeseries(z, t)), 'cmd_in');
                U = U.addElement(struct('state', timeseries(S.state, t), 'fault_word', timeseries(S.fault_word, t), ...
                    'brake_released', timeseries(S.brake_released, t)), 'sm');
                ins = {'derating', 'lim_active', 'sto_ok', 'stall_warn', 'w_meas', 'enc_cnt', 'tau_est', ...
                    'tau_avail', 'i_q', 'i_dc', 'v_dc', 'T_mot_meas', 'T_esc'};
                types = [{'Bus: Bus_MfeqEscCmd', 'Bus: Bus_MfeqEscSm'}, repmat({''}, 1, numel(ins))];
            else
                N0 = t_mf_equiv.tlNominal();
                force = N0;
                force.w_meas = 500;
                force.tau_applied = 20;
                nanRows = t_mf_equiv.repRow(N0, 7);
                nanIn = {'tau_lim_mot', 'tau_lim_reg', 'v_dc', 'T_mot_meas', 'T_esc', 'w_meas', 'tau_applied'};
                for k = 1:numel(nanIn), nanRows.(nanIn{k})(k) = NaN; end
                S = t_mf_equiv.catRows(t_mf_equiv.tlRows(rs, 1000, mo, e, false), t_mf_equiv.tlEdges(mo, e, true), ...
                    t_mf_equiv.repRow(force, 300), t_mf_equiv.tlRows(rs, 1000, mo, e, true), ...
                    t_mf_equiv.tlEdges(mo, e, false), nanRows, t_mf_equiv.tlRows(rs, 20, mo, e, true));
                t = (0:numel(S.w_meas) - 1)'*ts;
                z = zeros(size(t));
                U = U.addElement(struct('mode', timeseries(z, t), 'tau_lim_mot', timeseries(S.tau_lim_mot, t), ...
                    'tau_lim_reg', timeseries(S.tau_lim_reg, t)), 'cmd_in');
                ins = {'w_meas', 'v_dc', 'T_mot_meas', 'T_esc', 'tau_applied'};
                types = [{'Bus: Bus_MfeqEscCmd'}, repmat({''}, 1, numel(ins))];
            end
            for k = 1:numel(ins)
                U = U.addElement(timeseries(S.(ins{k}), t), ins{k});
            end
        end

        function S = fbRows(rs, n, cast)
            % n random samples of the Feedback inputs (cmd_in.mode and sm.* by element name)
            S.mode = 1 + double(rand(rs, n, 1) < 0.5);          % SPEED or TORQUE, a tenth odd
            odd = rand(rs, n, 1) < 0.1;
            pool = [0, 3, 1.5, -1, 0.5];
            S.mode(odd) = pool(randi(rs, numel(pool), nnz(odd), 1));
            S.state = uint8(randi(rs, [0 8], n, 1));
            S.fault_word = uint16(randi(rs, [0 65535], n, 1));
            S.brake_released = rand(rs, n, 1) < 0.5;
            S.derating = t_mf_equiv.statusIn(rs, n, cast);
            S.lim_active = t_mf_equiv.statusIn(rs, n, cast);
            S.sto_ok = rand(rs, n, 1) < 0.5;
            S.stall_warn = t_mf_equiv.statusIn(rs, n, cast);
            S.w_meas = 1200*rand(rs, n, 1) - 600;
            S.enc_cnt = int32(randi(rs, [-2^31, 2^31 - 1], n, 1));
            S.tau_est = 50*rand(rs, n, 1) - 25;
            S.tau_avail = 20*rand(rs, n, 1);
            S.i_q = 600*rand(rs, n, 1) - 300;
            S.i_dc = 400*rand(rs, n, 1) - 200;
            S.v_dc = 65*rand(rs, n, 1);
            S.T_mot_meas = 250 + 220*rand(rs, n, 1);
            S.T_esc = 250 + 150*rand(rs, n, 1);
        end

        function x = statusIn(rs, n, cast)
            % a status input of Feedback (derating, lim_active, stall_warn): boolean; with cast double, half of
            % them 0 or 1, a quarter from castEdges, a quarter uniform in [-3, 70000]
            if ~cast
                x = rand(rs, n, 1) < 0.5;
                return
            end
            x = double(rand(rs, n, 1) < 0.5);
            edge = t_mf_equiv.castEdges();
            r = rand(rs, n, 1);
            i = r >= 0.5 & r < 0.75;
            x(i) = edge(randi(rs, numel(edge), nnz(i), 1));
            i = r >= 0.75;
            x(i) = -3 + 70003*rand(rs, nnz(i), 1);
        end

        function v = castEdges()
            % inputs of a conversion to uint16 (Nearest, saturating): ties, just below a tie, past 0 and 65535,
            % non-finite
            v = [0; -0; 1; 0.5; 1.5; 2.5; 3.5; -0.5; -1.5; -2.5; 0.49999999999999994; 255.5; 65534.5; 65535; ...
                65535.49; 65535.5; 65536; 70000; 1e10; -1e10; Inf; -Inf; NaN];
        end

        function S = fbEdges(cast)
            % the edge samples of the Feedback inputs (see escInput), each a nominal sample with some inputs changed
            N0 = struct('mode', 1, 'state', uint8(4), 'fault_word', uint16(0), 'brake_released', true, ...
                'derating', false, 'lim_active', false, 'sto_ok', true, 'stall_warn', false, 'w_meas', 100.25, ...
                'enc_cnt', int32(12345), 'tau_est', 3.5, 'tau_avail', 12.75, 'i_q', 40.5, 'i_dc', 20.125, ...
                'v_dc', 48.3, 'T_mot_meas', 320.15, 'T_esc', 310.15);
            if cast
                N0.derating = 0; N0.lim_active = 0; N0.stall_warn = 0;
            end
            [st, md] = ndgrid(0:8, [1 2]);                       % every state with both modes
            R1 = t_mf_equiv.repRow(N0, numel(st));
            R1.state = uint8(st(:));
            R1.mode = md(:);
            R1.brake_released = mod((1:numel(st))', 2) == 0;
            fw = uint16([0; 1024; 65535; 64511; 1023; 2048; 3072]);   % with and without bit 10, in STO and OPERATION
            R2 = t_mf_equiv.repRow(N0, 2*numel(fw));
            R2.fault_word = [fw; fw];
            R2.state = uint8([8*ones(numel(fw), 1); 4*ones(numel(fw), 1)]);
            R3 = t_mf_equiv.repRow(N0, 5);                        % odd modes
            R3.mode = [NaN; 0; 1.5; 3; -0];
            R4 = t_mf_equiv.repRow(N0, 4);                        % enc_cnt limits
            R4.enc_cnt = int32([intmin('int32'); intmax('int32'); 0; -1]);
            rows = {R1, R2, R3, R4};
            rsg = double(realmax('single'));
            mid = (2 - 2^-24)*2^127;                              % half way from realmax('single') to 2^128
            sg = [NaN; Inf; -Inf; 0; -0; 1e39; -1e39; rsg; mid; mid - 2^75; -mid; 1e-46; 1.4e-45; 2^-149; ...
                2^-150; 3*2^-150; 1 + 2^-24; 1 + 3*2^-24; 0.1];
            for f = {'w_meas', 'tau_est', 'tau_avail', 'i_q', 'i_dc', 'v_dc', 'T_mot_meas', 'T_esc'}
                R = t_mf_equiv.repRow(N0, numel(sg));
                R.(f{1}) = sg;
                rows{end + 1} = R; %#ok<AGROW>
            end
            for f = {'derating', 'lim_active', 'stall_warn'}
                if cast, x = t_mf_equiv.castEdges(); else, x = [false; true]; end
                R = t_mf_equiv.repRow(N0, numel(x));
                R.(f{1}) = x;
                rows{end + 1} = R; %#ok<AGROW>
            end
            S = t_mf_equiv.catRows(rows{:});
        end

        function N0 = tlNominal()
            % a nominal sample of the Torque_Limiter inputs
            N0 = struct('tau_lim_mot', 16, 'tau_lim_reg', 16, 'w_meas', 100, 'v_dc', 48, 'T_mot_meas', 300, ...
                'T_esc', 300, 'tau_applied', 1);
        end

        function S = tlRows(rs, n, mo, e, full)
            % n random samples of the Torque_Limiter inputs; full false: |tau_applied| below 0.95 of the continuous
            % rating at that speed (floored at tau_cont_min), so that the I2t stays 0
            S.tau_lim_mot = 20*rand(rs, n, 1);
            S.tau_lim_reg = 20*rand(rs, n, 1);
            S.w_meas = 1200*rand(rs, n, 1) - 600;
            S.v_dc = 65*rand(rs, n, 1);
            S.T_mot_meas = 250 + 220*rand(rs, n, 1);
            S.T_esc = 250 + 150*rand(rs, n, 1);
            if full
                S.tau_applied = 50*rand(rs, n, 1) - 25;
            else
                rpm = min(abs(S.w_meas)*30/pi, mo.w_t_rpm(end));
                cont = max(interp1(mo.w_t_rpm(:), mo.T_cont_nom(:), rpm), e.tau_cont_min);
                S.tau_applied = 0.95*cont.*(2*rand(rs, n, 1) - 1);
            end
        end

        function S = tlEdges(mo, e, tauZero)
            % the edge samples of the Torque_Limiter inputs (see escInput), each a nominal sample with one input
            % changed; tauZero: tau_applied 0 throughout (the I2t stays 0). The speed at 0, -0, +-w_quad and
            % +-w_min_dc and one ulp beside them, at the rpm breakpoints and +-Inf; v_dc at 0 (Review Focus 2), -0,
            % the derating knees, the envelope breakpoints and Inf; the temperatures at, between and past their
            % derating knees and at the DERATING threshold 1 - derate_eps; the torque limits at 0, -0, Inf;
            % tau_applied at 0, -0, +-Inf and at the continuous rating (ratio 1, at standstill and on the floor)
            N0 = t_mf_equiv.tlNominal();
            if tauZero, N0.tau_applied = 0; end
            wq = e.w_quad;
            wd = e.w_min_dc;
            ed.w_meas = [0, -0, wq, -wq, wq + eps(wq), wq - eps(wq), -wq - eps(wq), -wq + eps(wq), wd, -wd, ...
                wd + eps(wd), wd - eps(wd), mo.w_t_rpm*pi/30, -mo.w_t_rpm(end)*pi/30, 1e-300, 1e6, Inf, -Inf]';
            ed.v_dc = [0, -0, mo.vdc_t, e.V_uv_derate, mean(e.V_uv_derate), e.V_ov_derate, mean(e.V_ov_derate), ...
                62, 100, 1e-300, Inf]';
            dT = e.T_mot_derate;
            dE = e.T_esc_derate;
            thr = dT(1) + e.derate_eps*diff(dT);                  % f_Tm about 1 - derate_eps
            ed.T_mot_meas = [dT, mean(dT), dT(1) - 1e-9, dT(2) + 1e-9, thr, thr - 1e-9, thr + 1e-9, 1000, 0, Inf]';
            ed.T_esc = [dE, mean(dE), dE(1) - 1e-9, dE(2) + 1e-9, dE(1) + e.derate_eps*diff(dE), 500, 0, Inf]';
            ed.tau_lim_mot = [0; -0; 1e-300; 25; Inf; -1];
            ed.tau_lim_reg = [0; -0; 1e-300; 25; Inf; -1];
            if ~tauZero, ed.tau_applied = [0; -0; 1e300; Inf; -Inf; 25; -25]; end
            rows = {};
            for f = fieldnames(ed)'
                R = t_mf_equiv.repRow(N0, numel(ed.(f{1})));
                R.(f{1}) = ed.(f{1});
                rows{end + 1} = R; %#ok<AGROW>
            end
            if ~tauZero
                R = t_mf_equiv.repRow(N0, 4);
                R.w_meas = [0; 0; 600; -600];
                R.tau_applied = [mo.T_cont_nom(1); -mo.T_cont_nom(1); e.tau_cont_min; -e.tau_cont_min];
                rows{end + 1} = R;
            end
            S = t_mf_equiv.catRows(rows{:});
        end

        function R = repRow(S0, n)
            % n copies of the one-sample structure S0, as columns
            R = S0;
            f = fieldnames(S0);
            for j = 1:numel(f), R.(f{j}) = repmat(S0.(f{j}), n, 1); end
        end

        function S = catRows(varargin)
            % the column structures concatenated field by field
            S = varargin{1};
            f = fieldnames(S);
            for k = 2:numel(varargin)
                for j = 1:numel(f), S.(f{j}) = [S.(f{j}); varargin{k}.(f{j})]; end
            end
        end

        function r = root()
            r = char(currentProject().RootFolder);
        end

        function loadModel(tc, f)
            % load the project file f once; close it after the test when this test loaded it
            [~, m] = fileparts(f);
            if bdIsLoaded(m)
                tc.assertEqual(lower(get_param(m, 'FileName')), lower(f), [m ' is loaded from another file']);
            else
                load_system(f);
                tc.addTeardown(@() close_system(m, 0));
            end
        end
    end
end
