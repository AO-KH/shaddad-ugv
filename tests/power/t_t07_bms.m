classdef t_t07_bms < matlab.unittest.TestCase
%T_T07_BMS  Card T-07 step 3: MIL tests of the BMS firmware functions that had no test of their own.
%
%   runtests('t_t07_bms')            Shaddad_BMS as a top model, in normal mode, driven open loop through its two
%                                    root inports (meas, Bus_BMS_Meas; can_rx, Bus_Can1_McuSlow with the BMS_REQ
%                                    frame 0x3D0) by a scripted stimulus; one test steps the mode manager's MATLAB
%                                    function directly, as t_pack_bms/mode_manager_table does
%
%   The harness (static methods stim / inputs / simulate) is shared with t_t07_bms_sil. Every expected value comes
%   from P (D114: a check of the model's own consistency takes its value from P); a test that compares the model
%   with a requirement only logs the comparison (MEASURED ...), and the disagreement is a finding of the T-07
%   inventory (work/team/t07/inv/bms.json), not a failure here.
%   The model is never changed: the configuration is a copy attached in memory, and the model is closed with
%   close_system(model, 0).

    properties (Constant)
        Mdl = 'Shaddad_BMS';
    end

    methods (TestClassSetup)
        function setupModel(tc)
            if ~evalin('base', 'exist(''Bus_BMS_Meas'', ''var'') && exist(''P'', ''var'') && exist(''Ts'', ''var'')')
                shaddad_params();
            end
            load_system(t_t07_bms.Mdl);
            tc.addTeardown(@() close_system(t_t07_bms.Mdl, 0));
            t_t07_bms.useConfig('saved');
        end
    end

    % =====================================================================================
    methods (Static)
        function m(fmt, varargin)
            fprintf(['MEASURED ' fmt '\n'], varargin{:});
        end

        function useConfig(which)
        %USECONFIG  Activate an in-memory copy of a configuration: 'saved' (the model's own set, or the set its
        %   reference points to) or 'genarm' (cs_GenARM, the generic ARM Cortex-M code-generation set, with
        %   PortableWordSizes on for SIL). The copy is discarded with close_system(model, 0).
            mdl = t_t07_bms.Mdl;
            acs = getActiveConfigSet(mdl);
            switch which
                case 'saved'
                    if startsWith(acs.Name, 't07_'), return, end
                    if isa(acs, 'Simulink.ConfigSetRef'), src = getRefConfigSet(acs); else, src = acs; end
                    name = 't07_saved';
                case 'genarm'
                    if strcmp(acs.Name, 't07_genarm'), return, end
                    src = evalin('base', 'cs_GenARM');
                    name = 't07_genarm';
            end
            csc = attachConfigSetCopy(mdl, src, true);
            csc.Name = name;
            setActiveConfigSet(mdl, name);
            if strcmp(which, 'genarm')
                set_param(mdl, 'PortableWordSizes', 'on');
            end
        end

        function S = stim(T)
        %STIM  Default stimulus: STANDBY, 13 cells at 3.9 V and 300 K, no current, key on, relay armed.
        %   Each field holds breakpoints [t, value...]: the value holds from t on (zero-order hold).
            Ns = evalin('base', 'P.power.pack.Ns');
            S.T = T;
            S.meas.V_cell_V = [0, 3.9*ones(1, Ns)];
            S.meas.T_cell_K = [0, 300*ones(1, Ns)];
            S.meas.I_pack_A = [0 0];
            S.meas.V_bus_V = [0 0];
            S.meas.V_dock_V = [0 0];
            S.meas.T_amb_K = [0 300];
            S.meas.T_air_K = [0 300];
            S.meas.K1_fb = [0 0]; S.meas.K2_fb = [0 0]; S.meas.K4_fb = [0 0];
            S.meas.Dock_Present = [0 0];
            S.meas.Safety_OK = [0 1]; S.meas.Safety_HV_OK = [0 1];
            S.meas.Key_On = [0 1]; S.meas.FuseMain_OK = [0 1]; S.meas.Fuse_Blown = [0 0];
            S.vpack_offset = [0 0];          % V_pack = sum(V_cell) + offset, unless S.meas.V_pack_V is given
            S.req.HV_Enable_Req = [0 0]; S.req.Charge_Enable_Req = [0 0];
            S.req.Fault_Reset_Req = [0 0]; S.req.Shutdown_Req = [0 0];
        end

        function ds = inputs(S)
        %INPUTS  The root-inport Dataset of Shaddad_BMS for a stimulus. Port 1 meas (Bus_BMS_Meas, Ts.bms_fast):
        %   element order, types and widths from the bus object. Port 2 can_rx (Bus_Can1_McuSlow, Ts.can_slow):
        %   the 0x3D0 BMS_REQ frame packed as the MCU packs it (ICD section 4: byte 0 flags b0 HV_Enable_Req,
        %   b1 Charge_Enable_Req, b2 Fault_Reset_Req, b3 Shutdown_Req; byte 1 the alive counter, +1 per frame),
        %   valid in every sample; the IMPL_CMD slot idle (not valid).
            mdl = t_t07_bms.Mdl;
            ds = Simulink.SimulationData.Dataset;
            ports = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport');
            [~, ix] = sort(cellfun(@(p) str2double(get_param(p, 'Port')), ports));
            ports = ports(ix);
            for k = 1:numel(ports)
                bus = strtrim(erase(get_param(ports{k}, 'OutDataTypeStr'), 'Bus:'));
                switch bus
                    case 'Bus_BMS_Meas'
                        v = t_t07_bms.measInput(S, t_t07_bms.grid(S.T, evalin('base', 'Ts.bms_fast')));
                    case 'Bus_Can1_McuSlow'
                        v = t_t07_bms.canInput(S, t_t07_bms.grid(S.T, evalin('base', 'Ts.can_slow')));
                    otherwise
                        error('t_t07_bms:port', 'unexpected root inport bus %s', bus);
                end
                ds = ds.addElement(v, get_param(ports{k}, 'Name'));
            end
        end

        function t = grid(T, dt)
            t = (0:round(T/dt))'*dt;
        end

        function v = measInput(S, t)
            bo = evalin('base', 'Bus_BMS_Meas');
            v = struct();
            for e = 1:numel(bo.Elements)
                el = bo.Elements(e);
                w = prod(el.Dimensions);
                if isfield(S.meas, el.Name)
                    x = t_t07_bms.hold(S.meas.(el.Name), t, w);
                elseif strcmp(el.Name, 'V_pack_V')
                    x = sum(t_t07_bms.hold(S.meas.V_cell_V, t, size(S.meas.V_cell_V, 2) - 1), 2) + ...
                        t_t07_bms.hold(S.vpack_offset, t, 1);
                else
                    error('t_t07_bms:element', 'no stimulus for Bus_BMS_Meas.%s', el.Name);
                end
                if strcmp(el.DataType, 'boolean'), x = logical(x); else, x = cast(x, el.DataType); end
                v.(el.Name) = timeseries(x, t, 'Name', el.Name);
            end
        end

        function v = canInput(S, t)
            n = numel(t);
            flags = t_t07_bms.hold(S.req.HV_Enable_Req, t, 1) + 2*t_t07_bms.hold(S.req.Charge_Enable_Req, t, 1) + ...
                    4*t_t07_bms.hold(S.req.Fault_Reset_Req, t, 1) + 8*t_t07_bms.hold(S.req.Shutdown_Req, t, 1);
            data = zeros(n, 8, 'uint8');
            data(:, 1) = uint8(flags);
            data(:, 2) = uint8(mod((0:n - 1)', 256));
            ts = @(x, nm) timeseries(x, t, 'Name', nm);
            req = struct('id', ts(uint32(hex2dec('3D0'))*ones(n, 1, 'uint32'), 'id'), 'len', ts(uint8(8)*ones(n, 1, 'uint8'), 'len'), ...
                         'data', ts(data, 'data'), 'valid', ts(true(n, 1), 'valid'));
            imp = struct('id', ts(uint32(hex2dec('140'))*ones(n, 1, 'uint32'), 'id'), 'len', ts(uint8(8)*ones(n, 1, 'uint8'), 'len'), ...
                         'data', ts(zeros(n, 8, 'uint8'), 'data'), 'valid', ts(false(n, 1), 'valid'));
            v = struct('impl_cmd', imp, 'bms_req', req);
        end

        function x = hold(bp, t, w)
        %HOLD  Zero-order-hold values of breakpoints [t, v(1..w)] on the grid t (N x w).
            x = zeros(numel(t), w);
            for i = 1:size(bp, 1)
                x(t >= bp(i, 1) - 1e-9, :) = repmat(bp(i, 2:end), nnz(t >= bp(i, 1) - 1e-9), 1);
            end
        end

        function R = simulate(S, mode)
        %SIMULATE  Simulate Shaddad_BMS with the stimulus S in 'normal' or 'sil' mode (with the configuration that is
        %   active). R.cmd, R.can and R.tel hold the three root outputs (cmd Bus_BMS_Cmd, can_tx Bus_Can1_Bms,
        %   telem Bus_BMS_Telemetry) as structs of timeseries.
            mdl = t_t07_bms.Mdl;
            in = Simulink.SimulationInput(mdl);
            in = in.setExternalInput(t_t07_bms.inputs(S));
            in = in.setModelParameter('StopTime', sprintf('%.6g', S.T), ...      % setExternalInput turns the input on
                'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'SaveTime', 'off', ...
                'SaveState', 'off', 'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');
            switch mode
                case 'normal', in = in.setModelParameter('SimulationMode', 'normal');
                case 'sil', in = in.setModelParameter('SimulationMode', 'software-in-the-loop (sil)');
            end
            t0 = tic;
            out = sim(in);
            R.wall = toc(t0);
            if ~isempty(out.ErrorMessage), error('t_t07_bms:sim', '%s', out.ErrorMessage); end
            y = out.yout;
            names = {'cmd', 'can', 'tel'};
            ports = {'cmd', 'can_tx', 'telem'};
            for k = 1:numElements(y)
                e = y.getElement(k);
                bp = e.BlockPath.getBlock(1);
                j = find(strcmp(ports, get_param(bp, 'Name')), 1);
                if ~isempty(j), R.(names{j}) = e.Values; end
            end
        end

        function L = leaves(v, prefix)
        %LEAVES  The timeseries of a (nested) struct of timeseries, as {name, timeseries} rows.
            L = cell(0, 2);
            if isa(v, 'timeseries')
                L = {prefix, v};
                return
            end
            fn = fieldnames(v);
            for k = 1:numel(fn)
                L = [L; t_t07_bms.leaves(v.(fn{k}), [prefix '.' fn{k}])]; %#ok<AGROW>
            end
        end

        function v = at(ts, t)
        %AT  The value of a timeseries at time t (its last sample at or before t), as a row.
            k = find(ts.Time <= t + 1e-9, 1, 'last');
            d = ts.Data;
            if isvector(d), v = d(k); else, v = d(k, :); end
        end

        function t = first(ts, cond, t0)
        %FIRST  The first sample time at or after t0 where cond(value) holds (NaN when never).
            if nargin < 3, t0 = 0; end
            d = double(ts.Data);
            if ~isvector(d), d = d(:, :); end
            ok = false(numel(ts.Time), 1);
            for k = 1:numel(ts.Time)
                if isvector(ts.Data), x = d(k); else, x = d(k, :); end
                ok(k) = ts.Time(k) >= t0 - 1e-9 && cond(x);
            end
            k = find(ok, 1);
            if isempty(k), t = NaN; else, t = ts.Time(k); end
        end

        function b = bit(w, n)
        %BIT  Bit n (0-based) of a uint32 fault or warning word.
            b = bitand(uint32(w), bitshift(uint32(1), n)) > 0;
        end
    end

    % =====================================================================================
    methods (Test, TestTags = {'T07'})

        function uv_protection_and_reset(tc)
            % E1 cell under-voltage: 2.80 V for 2 s latches Fault_Word b1, opens K1 from HV_ON; a reset while the
            % cell stays low keeps FAULT; a reset after the cell recovers clears b1 and returns to STANDBY.
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(20);
            v0 = S.meas.V_cell_V(1, 2:end); vlow = v0; vlow(3) = b.V_uv - 0.05;
            Vp = sum(v0);
            S.req.HV_Enable_Req = [0 0; 1.0 1];
            S.meas.V_bus_V = [0 0; 1.2 0.98*Vp; 5.5 0];
            S.meas.V_cell_V = [0, v0; 3.0, vlow; 14.0, v0];
            S.req.Fault_Reset_Req = [0 0; 12.0 1; 12.2 0; 16.0 1; 16.2 0];
            R = t_t07_bms.simulate(S, 'normal');
            st = R.tel.Contactor_State; fw = R.tel.Fault_Word;
            tUV = t_t07_bms.first(fw, @(w) t_t07_bms.bit(w, 1), 2);
            tK1 = t_t07_bms.first(R.cmd.K1_Cmd, @(x) x == 0, 2);
            t_t07_bms.m(['UV: cell 3 at %.2f V from 3.000 s; HV_ON at 2 s %d; UV fault b1 at %.3f s (qualification ' ...
                '%.1f s); K1_Cmd off at %.3f s; state at 11.9 s %d, after a reset with the cell low %d (b1 %d), after a ' ...
                'reset with the cell recovered %d (b1 %d); UV warning b0 at 9 s %d'], vlow(3), ...
                t_t07_bms.at(st, 2.0) == 4, tUV, b.qual.UV, tK1, t_t07_bms.at(st, 11.9), t_t07_bms.at(st, 13.0), ...
                t_t07_bms.bit(t_t07_bms.at(fw, 13.0), 1), t_t07_bms.at(st, 17.0), t_t07_bms.bit(t_t07_bms.at(fw, 17.0), 1), ...
                t_t07_bms.bit(t_t07_bms.at(R.tel.Warning_Word, 9.0), 0));
            tc.verifyEqual(t_t07_bms.at(st, 2.0), uint8(4), 'HV_ON before the fault');
            tc.verifyEqual(tUV - 3.0, b.qual.UV, 'AbsTol', 0.15, 'UV qualification time (P.power.bms.qual.UV)');
            tc.verifyLessThanOrEqual(tK1 - tUV, 0.3, 'K1 opens within t_open_wait plus the output delay');
            tc.verifyEqual(t_t07_bms.at(st, 11.9), uint8(10), 'FAULT after the weld check');
            tc.verifyTrue(bitand(uint32(t_t07_bms.at(fw, 11.9)), uint32(b.fault.mask_level3)) > 0, ...
                'the UV bit is a level-3 bit (P.power.bms.fault.mask_level3)');
            tc.verifyEqual(t_t07_bms.at(R.tel.Fault_Level, 11.9), uint8(3), 'Fault_Level 3 for a bit of mask_level3');
            tc.verifyEqual(t_t07_bms.at(st, 13.0), uint8(10), 'a reset while the cell is low keeps FAULT');
            tc.verifyTrue(t_t07_bms.bit(t_t07_bms.at(fw, 13.0), 1), 'b1 stays latched while the cell is low');
            tc.verifyFalse(t_t07_bms.bit(t_t07_bms.at(fw, 17.0), 1), 'b1 cleared by the reset after recovery');
            tc.verifyEqual(t_t07_bms.at(st, 17.0), uint8(1), 'FAULT -> STANDBY on the reset after recovery');
        end

        function ot_protection(tc)
            % E4 over-temperature: above T_ot for qual.OT latches b3 (and FAULT from STANDBY); the warning b2 first
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(10);
            T0 = S.meas.T_cell_K(1, 2:end); Th = T0; Th(5) = b.T_ot + 2;
            S.meas.T_cell_K = [0, T0; 2.0, Th];
            R = t_t07_bms.simulate(S, 'normal');
            tOT = t_t07_bms.first(R.tel.Fault_Word, @(w) t_t07_bms.bit(w, 3), 0);
            tW = t_t07_bms.first(R.tel.Warning_Word, @(w) t_t07_bms.bit(w, 2), 0);
            t_t07_bms.m('OT: cell 5 at %.2f K from 2.000 s: warning b2 at %.3f s, fault b3 at %.3f s (qualification %.1f s), state at 9 s %d', ...
                Th(5), tW, tOT, b.qual.OT, t_t07_bms.at(R.tel.Contactor_State, 9.0));
            tc.verifyEqual(tOT - 2.0, b.qual.OT, 'AbsTol', 0.15, 'OT qualification (P.power.bms.qual.OT)');
            tc.verifyLessThanOrEqual(tW, tOT, 'the OT warning comes no later than the fault');
            tc.verifyEqual(t_t07_bms.at(R.tel.Contactor_State, 9.0), uint8(10), 'STANDBY -> FAULT on FaultCritical');
        end

        function ut_protection(tc)
            % E4 under-temperature: below T_ut for qual.OT latches b4 (and FAULT from STANDBY)
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(10);
            T0 = S.meas.T_cell_K(1, 2:end); Tl = T0; Tl(5) = b.T_ut - 2;
            S.meas.T_cell_K = [0, T0; 2.0, Tl];
            R = t_t07_bms.simulate(S, 'normal');
            tUT = t_t07_bms.first(R.tel.Fault_Word, @(w) t_t07_bms.bit(w, 4), 0);
            t_t07_bms.m('UT: cell 5 at %.2f K from 2.000 s: fault b4 at %.3f s (qualification %.1f s), state at 9 s %d', ...
                Tl(5), tUT, b.qual.OT, t_t07_bms.at(R.tel.Contactor_State, 9.0));
            tc.verifyEqual(tUT - 2.0, b.qual.OT, 'AbsTol', 0.15, 'UT qualification (P.power.bms.qual.OT, shared)');
            tc.verifyEqual(t_t07_bms.at(R.tel.Contactor_State, 9.0), uint8(10), 'STANDBY -> FAULT on FaultCritical');
        end

        function sensor_plausibility(tc)
            % E1 sensor fault: |sum(V_cell) - V_pack| above FaultySensorThreshold for qual.sensor latches b5
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(8);
            S.vpack_offset = [0 0; 2.0, b.FaultySensorThreshold + 0.5];
            R = t_t07_bms.simulate(S, 'normal');
            tS = t_t07_bms.first(R.tel.Fault_Word, @(w) t_t07_bms.bit(w, 5), 0);
            t_t07_bms.m('V sensor: V_pack offset %.2f V from 2.000 s: fault b5 at %.3f s (qualification %.1f s), state at 7.5 s %d', ...
                b.FaultySensorThreshold + 0.5, tS, b.qual.sensor, t_t07_bms.at(R.tel.Contactor_State, 7.5));
            tc.verifyEqual(tS - 2.0, b.qual.sensor, 'AbsTol', 0.15, 'sensor qualification (P.power.bms.qual.sensor)');
            tc.verifyEqual(t_t07_bms.at(R.tel.Contactor_State, 7.5), uint8(10), 'a sensor fault is critical');
        end

        function charger_ovp_and_charge_paths(tc)
            % CHARGING entry and exit by the request, the CC-CV request, then the charger OVP: V_dock above
            % V_dock_ovp for qual_aux latches b14, CHARGING -> FAULT and K4 opens
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(11);
            S.meas.Dock_Present = [0 0; 1.0 1];
            S.req.Charge_Enable_Req = [0 0; 1.0 1; 4.0 0; 6.0 1];
            S.meas.V_dock_V = [0 0; 1.0 54.0; 8.0 b.V_dock_ovp + 0.6];
            S.meas.I_pack_A = [0 0; 1.5 -15; 4.0 0; 6.5 -15];
            R = t_t07_bms.simulate(S, 'normal');
            st = R.tel.Contactor_State;
            tO = t_t07_bms.first(R.tel.Fault_Word, @(w) t_t07_bms.bit(w, 14), 0);
            tK4 = t_t07_bms.first(R.cmd.K4_Cmd, @(x) x == 0, 7.0);
            t_t07_bms.m(['charge: state at 1.5 s %d, Charger_Enable at 1.5 s %d, request %.2f A; after the request ' ...
                'drop at 4.5 s %d (K4 %d); again at 6.5 s %d; OVP b14 at %.3f s (V_dock %.2f V, qualification %.2f s), K4 ' ...
                'off at %.3f s, state at 9 s %d'], t_t07_bms.at(st, 1.5), t_t07_bms.at(R.cmd.Charger_Enable, 1.5), ...
                t_t07_bms.at(R.cmd.Charger_I_Req_A, 3.5), t_t07_bms.at(st, 4.5), t_t07_bms.at(R.cmd.K4_Cmd, 4.5), ...
                t_t07_bms.at(st, 6.5), tO, b.V_dock_ovp + 0.6, b.qual_aux, tK4, t_t07_bms.at(st, 9.0));
            tc.verifyEqual(t_t07_bms.at(st, 1.5), uint8(8), 'STANDBY -> CHARGING (dock and request)');
            tc.verifyTrue(logical(t_t07_bms.at(R.cmd.Charger_Enable, 1.5)), 'charger enabled after t_chg_en');
            tc.verifyGreaterThan(t_t07_bms.at(R.cmd.Charger_I_Req_A, 3.5), 0, 'CC-CV requests a current');
            tc.verifyLessThanOrEqual(t_t07_bms.at(R.cmd.Charger_I_Req_A, 3.5), b.I_chg_max + 1e-3, 'request within I_chg_max');
            tc.verifyEqual(t_t07_bms.at(st, 4.5), uint8(1), 'CHARGING -> STANDBY when the request drops');
            tc.verifyFalse(logical(t_t07_bms.at(R.cmd.K4_Cmd, 4.5)), 'K4 open in STANDBY');
            tc.verifyEqual(t_t07_bms.at(st, 6.5), uint8(8), 'CHARGING again on the new request');
            tc.verifyEqual(tO - 8.0, b.qual_aux, 'AbsTol', 0.15, 'OVP qualification (P.power.bms.qual_aux)');
            tc.verifyLessThanOrEqual(tK4 - tO, 0.15, 'K4 opens on the OVP fault');
            tc.verifyEqual(t_t07_bms.at(st, 9.0), uint8(10), 'CHARGING -> FAULT on FaultCritical');
        end

        function current_limit_tables(tc)
            % E13 / E14: DCL and CCL follow the temperature tables of P.power.bms (uniform cells: the coolest and
            % the hottest cell are equal); DERATE (warning b5) below I_dis_derate - 1 or I_chg_derate - 1.
            % The comparison with REQ-SYS-060 (D94) is logged only (a finding of the inventory).
            b = evalin('base', 'P.power.bms');
            Tseg = [298.15 313.15 315.65 318.15 320.65 323.15 328.15 288.15 283.15 278.15 273.15];
            S = t_t07_bms.stim(4*numel(Tseg));
            Ns = numel(S.meas.T_cell_K(1, 2:end));
            S.meas.T_cell_K = [(0:numel(Tseg) - 1)'*4, repmat(Tseg', 1, Ns)];
            R = t_t07_bms.simulate(S, 'normal');
            lim = @(Tv, I) interp1(Tv, I, min(max(Tseg, Tv(1)), Tv(end)));
            ccl = min(lim(b.chg.T_vec, b.chg.I_cool), lim(b.chg.T_vec, b.chg.I_hot));
            dcl = min(lim(b.dis.T_vec, b.dis.I_cool), lim(b.dis.T_vec, b.dis.I_hot));
            for k = 1:numel(Tseg)
                te = 4*k - 0.1;
                c = double(t_t07_bms.at(R.tel.I_chg_lim_A, te)); d = double(t_t07_bms.at(R.tel.I_dis_lim_A, te));
                der = t_t07_bms.bit(t_t07_bms.at(R.tel.Warning_Word, te), 5);
                t_t07_bms.m('limits at %.2f deg C: CCL %.2f A (table %.2f), DCL %.2f A (table %.2f), DERATE %d', ...
                    Tseg(k) - 273.15, c, ccl(k), d, dcl(k), der);
                tc.verifyEqual(c, ccl(k), 'AbsTol', 0.5, sprintf('CCL at %.2f K', Tseg(k)));
                tc.verifyEqual(d, dcl(k), 'AbsTol', 0.5, sprintf('DCL at %.2f K', Tseg(k)));
                tc.verifyEqual(der, ccl(k) < b.warn.I_chg_derate - 1 || dcl(k) < b.warn.I_dis_derate - 1, ...
                    sprintf('DERATE at %.2f K', Tseg(k)));
            end
            k45 = find(Tseg == 318.15); k475 = find(Tseg == 320.65); k5 = find(Tseg == 278.15);
            t_t07_bms.m(['REQ-SYS-060 (D94: no charge above 45 deg C or below 10 deg C, a start only at 40 deg C or ' ...
                'less): the model allows %.1f A at 45 deg C, %.1f A at 47.5 deg C and %.1f A at 5 deg C'], ...
                double(t_t07_bms.at(R.tel.I_chg_lim_A, 4*k45 - 0.1)), double(t_t07_bms.at(R.tel.I_chg_lim_A, 4*k475 - 0.1)), ...
                double(t_t07_bms.at(R.tel.I_chg_lim_A, 4*k5 - 0.1)));
        end

        function fan_control(tc)
            % E17 and its conditions: on at TempOn, only when the cells are warmer than the ambient less dT_amb,
            % forced above T_force, off below TempOff (the vented variant; the sealed pack has no fan, D94)
            th = evalin('base', 'P.power.thermal');
            S = t_t07_bms.stim(18);
            Ns = numel(S.meas.T_cell_K(1, 2:end));
            S.meas.T_cell_K = [0, 300*ones(1, Ns); 2, (th.TempOn + 2)*ones(1, Ns); 10, (th.T_force + 2)*ones(1, Ns); ...
                               14, (th.TempOff - 2)*ones(1, Ns)];
            S.meas.T_amb_K = [0 300; 6 th.TempOn + 5; 10 th.T_force + 7; 14 300];
            R = t_t07_bms.simulate(S, 'normal');
            f = @(t) double(t_t07_bms.at(R.cmd.Fan_Cmd, t));
            t_t07_bms.m('fan: cool %.2f; warm cells, cool air %.2f; warm cells, hotter air %.2f; forced above T_force %.2f; below TempOff %.2f', ...
                f(1.9), f(5.9), f(9.9), f(13.9), f(17.9));
            tc.verifyEqual(f(1.9), 0, 'AbsTol', 1e-6, 'off below TempOn');
            tc.verifyEqual(f(5.9), 1, 'AbsTol', 1e-6, 'on above TempOn with cooler air');
            tc.verifyEqual(f(9.9), 0, 'AbsTol', 1e-6, 'off when the air is hotter than the cells less dT_amb');
            tc.verifyEqual(f(13.9), 1, 'AbsTol', 1e-6, 'forced above T_force');
            tc.verifyEqual(f(17.9), 0, 'AbsTol', 1e-6, 'off below TempOff');
        end

        function balancing_enable(tc)
            % E16 and its enable: only at rest or charging and above V_bal_enable; the high group bleeds
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(26);
            Ns = numel(S.meas.V_cell_V(1, 2:end));
            hi = (b.V_bal_enable + 0.05)*ones(1, Ns); hi(7) = hi(7) + 2*b.bal.Threshold;
            lo = (b.V_bal_enable - 0.05)*ones(1, Ns); lo(7) = lo(7) + 2*b.bal.Threshold;
            S.meas.V_cell_V = [0, 3.9*ones(1, Ns); 1, hi; 17, lo];
            S.meas.I_pack_A = [0 0; 9 10; 17 0];
            R = t_t07_bms.simulate(S, 'normal');
            bc = @(t) logical(t_t07_bms.at(R.cmd.Bal_Cmd, t));
            on = bc(8.5); dis = bc(16.5); low = bc(25.5);
            t_t07_bms.m('balancing: at rest above %.2f V: groups on %s; while discharging 10 A: %s; at rest below: %s', ...
                b.V_bal_enable, mat2str(find(on)), mat2str(find(dis)), mat2str(find(low)));
            tc.verifyEqual(find(on), 7, 'only the high group bleeds at rest above V_bal_enable');
            tc.verifyEmpty(find(dis), 'no balancing while discharging');
            tc.verifyEmpty(find(low), 'no balancing below V_bal_enable');
        end

        function warnings_ov_imbalance(tc)
            % E2 warnings: a cell above V_ov_warn (below V_ov) sets OV_warn b1 after 5 s; the spread above
            % dV_imb with V_max above V_imb_top sets IMBALANCE b6; no OV fault
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(12);
            Ns = numel(S.meas.V_cell_V(1, 2:end));
            v = 4.15*ones(1, Ns); v(2) = (b.V_ov_warn + b.V_ov)/2;
            S.meas.V_cell_V = [0, 3.9*ones(1, Ns); 1, v];
            R = t_t07_bms.simulate(S, 'normal');
            ww = R.tel.Warning_Word;
            tW = t_t07_bms.first(ww, @(w) t_t07_bms.bit(w, 1), 0);
            t_t07_bms.m('OV warning: cell 2 at %.3f V from 1.000 s: b1 at %.3f s; IMBALANCE b6 at 11 s %d; OV fault b0 at 11 s %d', ...
                v(2), tW, t_t07_bms.bit(t_t07_bms.at(ww, 11), 6), t_t07_bms.bit(t_t07_bms.at(R.tel.Fault_Word, 11), 0));
            tc.verifyEqual(tW - 1.0, 5, 'AbsTol', 0.15, 'OV warning qualification (E2, 5 s)');
            tc.verifyTrue(t_t07_bms.bit(t_t07_bms.at(ww, 11), 6), 'IMBALANCE above V_imb_top');
            tc.verifyFalse(t_t07_bms.bit(t_t07_bms.at(R.tel.Fault_Word, 11), 0), 'no OV fault below V_ov');
            tc.verifyEqual(t_t07_bms.at(R.tel.Fault_Word, 11), uint32(0), 'no fault bit at all');
            tc.verifyEqual(t_t07_bms.at(R.tel.Fault_Level, 11), uint8(1), 'Fault_Level 1 for a warning');
        end

        function mode_manager_rearm_after_precharge_drop(tc)
            % the HV re-entry latch (armed) on the path mode_manager_table does not drive: PRECHARGE -> STANDBY on
            % a Safety_OK drop with HV_Enable_Req still high disarms; Safety_OK back alone does not restart the
            % precharge; the request's fall and rise does. Steps the MATLAB function at the chart tick.
            prm = evalin('base', 'P.power.bms.chart');
            clear bms_mode_manager
            c = onCleanup(@() clear('bms_mode_manager'));
            u = struct('HV', false, 'CE', false, 'RST', false, 'SD', false, 'SOK', true, 'DOCK', false, ...
                       'FC', false, 'Vp', single(50.7), 'Vb', single(0), 'I', single(0), 'DONE', false);
            nInit = round(prm.t_init/prm.dt);
            st = step(u, nInit + 1);
            tc.verifyEqual(st, uint8(1), 'STANDBY after t_init');
            u.HV = true;
            st = step(u, 1);
            tc.verifyEqual(st, uint8(2), 'STANDBY -> PRECHARGE');
            u.SOK = false;
            st = step(u, 1);
            tc.verifyEqual(st, uint8(1), 'PRECHARGE -> STANDBY on the Safety_OK drop');
            u.SOK = true;
            st = step(u, 20);
            tc.verifyEqual(st, uint8(1), 'disarmed: Safety_OK back with the request still high does not restart');
            u.HV = false; step(u, 1); u.HV = true;
            st = step(u, 1);
            tc.verifyEqual(st, uint8(2), 're-armed by the request''s fall and rise');
            t_t07_bms.m('mode manager: PRECHARGE -> STANDBY on Safety_OK drop disarms; re-armed by the request edge');
            function s = step(u, n)
                for i = 1:n
                    [~, ~, ~, ~, ~, ~, ~, ~, s] = bms_mode_manager(u.HV, u.CE, u.RST, u.SD, u.SOK, u.DOCK, u.FC, ...
                        u.Vp, u.Vb, u.I, u.DONE, prm);
                end
            end
        end

        function mode_paths(tc)
            % the transitions mode_manager_table does not drive through the model: PRECHARGE -> STANDBY on a
            % dropped request, CHARGING -> STANDBY when the dock goes, HV_ON -> OPENING on Shutdown_Req, and
            % WELD_CHECK -> STANDBY when the bus decays
            S = t_t07_bms.stim(16);
            Vp = sum(S.meas.V_cell_V(1, 2:end));
            S.req.HV_Enable_Req = [0 0; 1.0 1; 1.1 0; 7.0 1];
            S.meas.Dock_Present = [0 0; 3.0 1; 5.0 0];
            S.req.Charge_Enable_Req = [0 0; 3.0 1; 5.0 0];
            S.meas.V_bus_V = [0 0; 7.2 0.98*Vp; 9.2 0];
            S.req.Shutdown_Req = [0 0; 9.0 1];
            R = t_t07_bms.simulate(S, 'normal');
            st = R.tel.Contactor_State;
            s = @(t) t_t07_bms.at(st, t);
            t_t07_bms.m('mode paths: %d at 1.05 s, %d at 1.3 s; %d at 3.3 s, %d at 5.3 s; %d at 8.5 s, %d at 9.15 s, %d at 9.6 s, %d at 15 s; weld b7 %d', ...
                s(1.05), s(1.3), s(3.3), s(5.3), s(8.5), s(9.15), s(9.6), s(15), ...
                t_t07_bms.bit(t_t07_bms.at(R.tel.Fault_Word, 15), 7));
            tc.verifyEqual(s(1.3), uint8(1), 'PRECHARGE -> STANDBY when the request drops');
            tc.verifyEqual(s(3.3), uint8(8), 'STANDBY -> CHARGING');
            tc.verifyEqual(s(5.3), uint8(1), 'CHARGING -> STANDBY when the dock goes');
            tc.verifyEqual(s(8.5), uint8(4), 'STANDBY -> PRECHARGE -> CLOSE_MAIN -> HV_ON');
            tc.verifyEqual(s(9.6), uint8(6), 'HV_ON -> OPENING -> WELD_CHECK on Shutdown_Req');
            tc.verifyEqual(s(15), uint8(1), 'WELD_CHECK -> STANDBY when the bus decayed');
            tc.verifyFalse(t_t07_bms.bit(t_t07_bms.at(R.tel.Fault_Word, 15), 7), 'no weld fault');
        end

        function telemetry_alive_period(tc)
            % E25 / Telemetry: one frame every Ts.telem with the alive counter +1 (mod 256)
            Tt = evalin('base', 'P.power.Ts.telem');
            R = t_t07_bms.simulate(t_t07_bms.stim(5), 'normal');
            a = double(R.tel.AliveCounter.Data(:)); t = R.tel.AliveCounter.Time;
            dt = diff(t); da = mod(diff(a), 256);
            t_t07_bms.m('telemetry: %d frames, period %.4f..%.4f s (Ts.telem %.2f), alive steps %d..%d', ...
                numel(t), min(dt), max(dt), Tt, min(da), max(da));
            tc.verifyEqual([min(dt) max(dt)], [Tt Tt], 'AbsTol', 1e-9);
            tc.verifyTrue(all(da == 1), 'the alive counter steps by one per frame');
        end
    end
end
