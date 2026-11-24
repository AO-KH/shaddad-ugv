classdef t_t07_mcu_sil < matlab.unittest.TestCase
%T_T07_MCU_SIL  Card T-07 step 7: back-to-back tests of the MCU firmware in software-in-the-loop (SIL).
%
%   runtests('t_t07_mcu_sil')                                          13 units and Shaddad_MCU
%   runtests('t_t07_mcu_sil', 'ProcedureName', 'unit_back_to_back')   the units only
%
%   Each unit of Shaddad_MCU that generates code on its own (T-07 pass A, work/team/t07/reports/step01_mcu_A.md)
%   is built in memory in a harness model cg_<unit>: a Subsystem Reference block to the unit, with root ports that
%   take the unit's interface from Shaddad_MCU as compiled (types, dimensions, sample times). A virtual bus gets
%   the bus object that Simulink.Bus.createObject makes from the Bus Creator that forms it, in this MATLAB's base
%   workspace only. The harness, and Shaddad_MCU itself (the deployed code, fw_telemetry included), take an
%   in-memory copy of cs_GenARM (ert.tlc, generic ARM Cortex-M) with PortableWordSizes on and GenCodeOnly off.
%   The SIL code is built in work/team/t07/sil/build_mcu, so the traced code of pass A stays as generated.
%   The same stimulus runs in normal mode and in SIL, and every root output is compared: integers, booleans and
%   enumerations bit-exact, single within 1e-5 x max(1, |reference|), double within 1e-9 x max(1, |reference|),
%   NaN against NaN and Inf against the same Inf equal, the same sample times. One record per model goes to
%   work/team/t07/sil/<model>.json. When Windows refuses the SIL program while Smart App Control is on, the test
%   is BLOCKED through an assumption, never a pass; with Smart App Control off, any SIL build or run error is a
%   failure. No model file is saved: every harness, unit and Shaddad_MCU is closed with close_system(name, 0).
%
%   The stimuli (scenario): a leaf that a scenario does not set gets a deterministic pattern of its type held
%   for 7 to 19 samples (the minimum, the maximum, zero and edge values of an integer; moderate values of a
%   float; both values of a boolean). A CAN frame slot (id, len, data, valid) carries the identifier of its slot
%   (the IO layer's dispatch table) and cycles as t_t07_frames does: 8 normal frames, a dropped frame, a foreign
%   identifier, a short frame (length 4), a normal frame, with patterned payload bytes. The SBUS decoder gets one
%   stream through the cases of TEL01 to TEL06; the mode manager, the safety supervisor, the vehicle control,
%   the implement unit and the watchdog manager get sequences through their modes and faults, built on the
%   healthy defaults of mcu_ctrl_harness (the T09 unit tests).

    properties (TestParameter)
        unit = {'fw_sbus_decoder', 'fw_rc_mapping', 'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', ...
            'fw_adc_scaling', 'fw_io_in_sim', 'fw_io_out_sim', 'fw_mode_manager', 'fw_safety_supervisor', ...
            'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
    end

    properties
        Root
        Ctx
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.Root = char(currentProject().RootFolder);
            if ~evalin('base', ['exist(''P'', ''var'') && exist(''Ts'', ''var'') && exist(''Bus_McuIn10'', ''var'')' ...
                    ' && exist(''VC_TARGET'', ''var'')'])
                shaddad_params();
            end
            tc.assumeEqual(evalin('base', 'VC_TARGET'), 0, 'the SIM IO variants (VC_TARGET 0) are the ones under test');
            shaddad_configsets('codegen');
            old = Simulink.fileGenControl('getConfig');
            bdir = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'build_mcu');
            if ~isfolder(bdir), mkdir(bdir); end
            Simulink.fileGenControl('set', 'CodeGenFolder', bdir, 'CacheFolder', bdir, 'createDir', true);
            tc.addTeardown(@() Simulink.fileGenControl('set', 'CodeGenFolder', old.CodeGenFolder, ...
                'CacheFolder', old.CacheFolder));
            u = t_t07_mcu_sil.unitList();
            tc.Ctx = t_t07_mcu_sil.context(u);
            tc.assertEmpty(tc.Ctx.error, ['Shaddad_MCU could not be compiled for the unit interfaces: ' tc.Ctx.error]);
        end
    end

    methods (Test, TestTags = {'T07', 'SIL'})
        function unit_back_to_back(tc, unit)
            tc.assertTrue(isfield(tc.Ctx.units, unit), [unit ' not found in Shaddad_MCU']);
            m = t_t07_mcu_sil.harness(unit, tc.Ctx.units.(unit));
            tc.addTeardown(@() t_t07_mcu_sil.closeQuiet({m, unit}));
            sc = t_t07_mcu_sil.scenario(unit);
            tc.backToBack(m, sc, unit);
        end

        function shaddad_mcu_back_to_back(tc)
            mdl = 'Shaddad_MCU';
            load_system(mdl);
            tc.addTeardown(@() t_t07_mcu_sil.closeQuiet({mdl}));
            t_t07_mcu_sil.useGenArm(mdl);
            sc = t_t07_mcu_sil.scenario(mdl);
            tc.backToBack(mdl, sc, mdl);
        end
    end

    methods
        function backToBack(tc, mdl, sc, name)
            file = fullfile(tc.Root, 'work', 'team', 't07', 'sil', [name '.json']);
            [sac0, sactxt0] = t_t07_mcu_sil.sacOn();
            if sac0, st0 = 'blocked'; else, st0 = 'fail'; end      % the record if the run never comes back
            rec = struct('model', name, 'status', st0, 'stimulus', sc.desc, 'steps', 0, 'outputs', {{}}, ...
                'wall_s', 0, 'note', ['The SIL run was started and gave no result (record written before it; ' ...
                sactxt0 ').']);
            tw = tic;
            [ds, steps, used] = t_t07_mcu_sil.inputs(mdl, sc);
            rec.steps = steps;
            unused = setdiff(sc.ovKeys, used);
            tc.assertEmpty(unused, ['scenario overrides that match no input leaf: ' strjoin(unused, ', ')]);
            Rn = t_t07_mcu_sil.simulate(mdl, ds, sc.T, 'normal');
            t_t07_mcu_sil.writeRecord(file, rec);
            try
                Rs = t_t07_mcu_sil.simulate(mdl, ds, sc.T, 'sil');
            catch e
                msg = getReport(e, 'basic', 'hyperlinks', 'off');
                rec.wall_s = round(toc(tw), 1);
                [sac, sactxt] = t_t07_mcu_sil.sacOn();
                if t_t07_mcu_sil.refused(msg) && sac
                    rec.status = 'blocked';
                    rec.note = ['BLOCKED: the SIL program was refused (' sactxt '; T-07 common brief section 6). ' msg];
                    t_t07_mcu_sil.writeRecord(file, rec);
                    tc.assumeFail(rec.note);
                else
                    rec.status = 'fail';
                    rec.note = ['The SIL build or run failed (' sactxt '): ' msg];
                    t_t07_mcu_sil.writeRecord(file, rec);
                    tc.verifyFail(rec.note);
                    return
                end
            end
            [outs, allpass] = t_t07_mcu_sil.compare(Rn.y, Rs.y);
            for k = 1:numel(outs)
                o = outs{k};
                tc.verifyTrue(o.pass, sprintf('%s %s (%s): max |SIL - normal| %g, tolerance %g, same samples %d', ...
                    name, o.name, o.type, o.max_abs_err, o.tol, o.same_samples));
            end
            tc.verifyNotEmpty(outs, [name ': no root output was logged']);
            rec.outputs = outs;
            rec.wall_s = round(toc(tw), 1);
            if allpass && ~isempty(outs), rec.status = 'pass'; else, rec.status = 'fail'; end
            rec.note = sprintf(['%d root output signals compared; normal %.1f s, SIL %.1f s (build included); ' ...
                '%s.'], numel(outs), Rn.wall, Rs.wall, sactxt0);
            t_t07_mcu_sil.writeRecord(file, rec);
            fprintf('MEASURED SIL back-to-back %s: %s, %d outputs, normal %.1f s, SIL %.1f s\n', name, rec.status, ...
                numel(outs), Rn.wall, Rs.wall);
        end
    end

    methods (Static)
        function u = unitList()
            u = {'fw_sbus_decoder', 'fw_rc_mapping', 'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', ...
                'fw_adc_scaling', 'fw_io_in_sim', 'fw_io_out_sim', 'fw_mode_manager', 'fw_safety_supervisor', ...
                'fw_vehicle_ctrl', 'fw_implement_aux', 'fw_watchdog_mgr'};
        end

        % ================================================================== the units' interface in Shaddad_MCU
        function ctx = context(units)
        %CONTEXT  The units' ports in Shaddad_MCU as compiled, and a bus object for each virtual bus (made in the base
        %   workspace by Simulink.Bus.createObject from the Bus Creator that forms the bus).
            ctx = struct('units', struct(), 'bus_objects', {{}}, 'error', '');
            mdl = 'Shaddad_MCU';
            try
                load_system(mdl);
                blk = struct();
                for k = 1:numel(units)
                    b = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'SubSystem', ...
                        'ReferencedSubsystem', units{k});
                    if ~isempty(b), blk.(units{k}) = b{1}; end
                end
                feval(mdl, [], [], [], 'compile');
                cleanup = onCleanup(@() t_t07_mcu_sil.term(mdl));
                srcs = cell(0, 4);
                for k = 1:numel(units)
                    u = units{k};
                    if ~isfield(blk, u), continue, end
                    ph = get_param(blk.(u), 'PortHandles');
                    c = struct('block', blk.(u), 'in', {cell(1, numel(ph.Inport))}, 'out', {cell(1, numel(ph.Outport))});
                    for j = 1:numel(ph.Inport)
                        c.in{j} = t_t07_mcu_sil.cinfo(ph.Inport(j));
                        ln = get_param(ph.Inport(j), 'Line');
                        if ln > 0 && strcmp(c.in{j}.bus, 'VIRTUAL_BUS')
                            srcs(end + 1, :) = {u, 'in', j, get_param(ln, 'SrcPortHandle')}; %#ok<AGROW>
                        end
                    end
                    for j = 1:numel(ph.Outport)
                        c.out{j} = t_t07_mcu_sil.cinfo(ph.Outport(j));
                        if strcmp(c.out{j}.bus, 'VIRTUAL_BUS')
                            srcs(end + 1, :) = {u, 'out', j, ph.Outport(j)}; %#ok<AGROW>
                        end
                    end
                    ctx.units.(u) = c;
                end
                clear cleanup
                bc = cell(1, size(srcs, 1));
                for k = 1:size(srcs, 1), bc{k} = t_t07_mcu_sil.busSource(srcs{k, 4}); end
                paths = unique(bc(~cellfun(@isempty, bc)), 'stable');
                names = repmat({''}, 1, numel(paths));
                if ~isempty(paths)
                    try
                        bi = Simulink.Bus.createObject(mdl, paths);
                        if numel(bi) == numel(paths), names = {bi.busName}; end
                    catch
                    end
                end
                for j = find(cellfun(@isempty, names))
                    try
                        bi = Simulink.Bus.createObject(mdl, paths{j});
                        names{j} = bi(1).busName;
                    catch e
                        ctx.bus_objects{end + 1} = sprintf('createObject failed for %s: %s', paths{j}, e.message);
                    end
                end
                for k = 1:size(srcs, 1)
                    j = find(strcmp(paths, bc{k}), 1);
                    if isempty(j) || isempty(names{j}), continue, end
                    ctx.units.(srcs{k, 1}).(srcs{k, 2}){srcs{k, 3}}.object = names{j};
                    ctx.bus_objects{end + 1} = sprintf('%s %s%d: %s (%s)', srcs{k, 1}, srcs{k, 2}, srcs{k, 3}, ...
                        names{j}, bc{k});
                end
            catch e
                ctx.error = getReport(e, 'basic', 'hyperlinks', 'off');
            end
            t_t07_mcu_sil.closeQuiet({mdl});
        end

        function s = cinfo(h)
            s = struct('type', '', 'dims', '', 'ts', '', 'bus', 'NOT_BUS', 'object', '');
            try, s.type = get_param(h, 'CompiledPortDataType'); catch, end %#ok<*NOCOM>
            try, s.dims = mat2str(get_param(h, 'CompiledPortDimensions')); catch, end
            try
                t = get_param(h, 'CompiledSampleTime');
                if iscell(t), t = t{1}; end
                s.ts = mat2str(t);
            catch
            end
            try, s.bus = get_param(h, 'CompiledBusType'); catch, end
        end

        function term(mdl)
            try, feval(mdl, [], [], [], 'term'); catch, end
        end

        function p = busSource(ph)
        %BUSSOURCE  The block that forms the virtual bus on output port ph: back through rate, delay and conversion
        %   blocks, into the subsystem whose output it is, up out of a subsystem through its Inport, to a Bus Creator.
            p = '';
            for hop = 1:40
                blk = get_param(ph, 'Parent');
                switch get_param(blk, 'BlockType')
                    case 'BusCreator'
                        p = blk;
                        return
                    case {'RateTransition', 'UnitDelay', 'SignalConversion', 'Memory', 'ZeroOrderHold', 'Delay'}
                        pin = get_param(blk, 'PortHandles');
                        ph = t_t07_mcu_sil.srcOf(pin.Inport(1));
                    case 'SubSystem'
                        n = get_param(ph, 'PortNumber');
                        ob = find_system(blk, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                            'BlockType', 'Outport', 'Port', num2str(n));
                        if isempty(ob), p = blk; return, end
                        pin = get_param(ob{1}, 'PortHandles');
                        ph = t_t07_mcu_sil.srcOf(pin.Inport(1));
                    case 'Inport'
                        par = get_param(blk, 'Parent');
                        if strcmp(get_param(par, 'Type'), 'block_diagram'), p = blk; return, end
                        pp = get_param(par, 'PortHandles');
                        ph = t_t07_mcu_sil.srcOf(pp.Inport(str2double(get_param(blk, 'Port'))));
                    otherwise
                        p = blk;
                        return
                end
                if isempty(ph) || ph < 0, p = ''; return, end
            end
        end

        function h = srcOf(inport)
            h = -1;
            ln = get_param(inport, 'Line');
            if ln > 0, h = get_param(ln, 'SrcPortHandle'); end
        end

        % ================================================================== the harness and the configuration
        function m = harness(u, c)
        %HARNESS  The in-memory harness cg_<u>: a Subsystem Reference block to the unit, root ports with the unit's
        %   interface as Shaddad_MCU compiles it (input data held between samples), and the cs_GenARM copy.
            load_system(u);
            m = ['cg_' u];
            if bdIsLoaded(m), close_system(m, 0); end
            new_system(m);
            ins = t_t07_mcu_sil.plist(find_system(u, 'SearchDepth', 1, 'BlockType', 'Inport'), true);
            outs = t_t07_mcu_sil.plist(find_system(u, 'SearchDepth', 1, 'BlockType', 'Outport'), false);
            n = max(numel(ins), numel(outs));
            sr = [m '/' u];
            add_block('simulink/Ports & Subsystems/Subsystem Reference', sr, 'ReferencedSubsystem', u, ...
                'Position', [400 40 640 60 + 40*n]);
            ph = get_param(sr, 'PortHandles');
            for k = 1:numel(ins)
                p = ins{k};
                b = sprintf('%s/%s', m, p.name);
                add_block('simulink/Sources/In1', b, 'Port', num2str(p.port), 'Position', [100 40+40*k 130 54+40*k]);
                [typ, dims, ts, virt] = t_t07_mcu_sil.portset(p, c, 'in', p.port);
                t_t07_mcu_sil.trySet(b, 'OutDataTypeStr', typ);
                t_t07_mcu_sil.trySet(b, 'PortDimensions', dims);
                t_t07_mcu_sil.trySet(b, 'SampleTime', ts);
                t_t07_mcu_sil.trySet(b, 'Interpolate', 'off');
                if startsWith(typ, 'Bus:')
                    if virt, t_t07_mcu_sil.trySet(b, 'BusOutputAsStruct', 'off');
                    else, t_t07_mcu_sil.trySet(b, 'BusOutputAsStruct', 'on'); end
                end
                q = get_param(b, 'PortHandles');
                add_line(m, q.Outport(1), ph.Inport(p.port));
            end
            for k = 1:numel(outs)
                p = outs{k};
                b = sprintf('%s/%s', m, p.name);
                add_block('simulink/Sinks/Out1', b, 'Port', num2str(p.port), 'Position', [800 40+40*k 830 54+40*k]);
                [typ, dims] = t_t07_mcu_sil.portset(p, c, 'out', p.port);
                if startsWith(typ, 'Bus:'), t_t07_mcu_sil.trySet(b, 'OutDataTypeStr', typ); end
                if ~startsWith(typ, 'Bus:') && ~strcmp(dims, '-1'), t_t07_mcu_sil.trySet(b, 'PortDimensions', dims); end
                q = get_param(b, 'PortHandles');
                add_line(m, ph.Outport(p.port), q.Inport(1));
            end
            t_t07_mcu_sil.useGenArm(m);
        end

        function useGenArm(mdl)
        %USEGENARM  Activate an in-memory copy of cs_GenARM with PortableWordSizes on and GenCodeOnly off (SIL).
            acs = getActiveConfigSet(mdl);
            if strcmp(acs.Name, 't07_genarm'), return, end
            csc = attachConfigSetCopy(mdl, evalin('base', 'cs_GenARM'), true);
            csc.Name = 't07_genarm';
            setActiveConfigSet(mdl, 't07_genarm');
            set_param(mdl, 'PortableWordSizes', 'on', 'GenCodeOnly', 'off');
            t_t07_mcu_sil.trySet(mdl, 'GenerateReport', 'off');
            t_t07_mcu_sil.trySet(mdl, 'LaunchReport', 'off');
        end

        function L = plist(blocks, isIn)
            L = cell(1, numel(blocks));
            nums = zeros(1, numel(blocks));
            for k = 1:numel(blocks)
                b = blocks{k};
                s = struct('port', str2double(get_param(b, 'Port')), 'name', get_param(b, 'Name'), ...
                    'type', get_param(b, 'OutDataTypeStr'), 'dims', get_param(b, 'PortDimensions'), ...
                    'ts', get_param(b, 'SampleTime'), 'bus_struct', '');
                if isIn
                    try, s.bus_struct = get_param(b, 'BusOutputAsStruct'); catch, end
                end
                L{k} = s;
                nums(k) = s.port;
            end
            [~, i] = sort(nums);
            L = L(i);
        end

        function [typ, dims, ts, virt] = portset(p, c, dir, k)
            typ = p.type; dims = p.dims; ts = p.ts;
            virt = startsWith(typ, 'Bus:') && ~strcmp(p.bus_struct, 'on');
            ci = c.(dir);
            if k > numel(ci) || isempty(ci{k}), return, end
            ci = ci{k};
            if ~isempty(ci.object)
                typ = ['Bus: ' ci.object]; dims = '-1'; virt = true;
            elseif startsWith(typ, 'Inherit') && ~isempty(ci.type) && ~strcmp(ci.bus, 'VIRTUAL_BUS')
                if strcmp(ci.bus, 'NON_VIRTUAL_BUS')
                    typ = ['Bus: ' ci.type]; virt = false;
                else
                    typ = ci.type;
                    if ~any(strcmp(typ, {'double', 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', ...
                            'int64', 'uint64', 'boolean'})) && exist(typ, 'class') == 8
                        typ = ['Enum: ' typ];
                    end
                end
            end
            if strcmp(dims, '-1') && ~isempty(ci.dims) && ~strcmp(ci.bus, 'VIRTUAL_BUS') && ~startsWith(typ, 'Bus:')
                d = str2num(ci.dims); %#ok<ST2NM>
                if numel(d) >= 2 && d(1) == 1, dims = num2str(d(2));
                elseif numel(d) >= 3 && d(1) == 2, dims = mat2str(d(2:3)); end
            end
            if strcmp(ts, '-1') && ~isempty(ci.ts)
                t = str2num(ci.ts); %#ok<ST2NM>
                if ~isempty(t) && isfinite(t(1)) && t(1) > 0, ts = num2str(t(1)); end
            end
        end

        function trySet(b, prm, val)
            try, set_param(b, prm, val); catch, end
        end

        function closeQuiet(names)
            for k = 1:numel(names)
                try
                    if bdIsLoaded(names{k}), close_system(names{k}, 0); end
                catch
                end
            end
        end

        % ================================================================== the stimulus
        function [ds, steps, used] = inputs(mdl, sc)
        %INPUTS  The root-inport Dataset of model mdl for scenario sc, in port order; steps = the samples of the
        %   fastest input; used = the override keys that matched an input leaf.
            ports = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport');
            [~, ix] = sort(cellfun(@(p) str2double(get_param(p, 'Port')), ports));
            ports = ports(ix);
            ds = Simulink.SimulationData.Dataset;
            steps = 0;
            used = {};
            k0 = 0;
            for k = 1:numel(ports)
                nm = get_param(ports{k}, 'Name');
                typ = strtrim(get_param(ports{k}, 'OutDataTypeStr'));
                ts = evalin('base', get_param(ports{k}, 'SampleTime'));
                if ts(1) <= 0, ts = evalin('base', 'Ts.base'); end
                t = (0:round(sc.T/ts(1)))'*ts(1);
                steps = max(steps, numel(t));
                dims = str2num(get_param(ports{k}, 'PortDimensions')); %#ok<ST2NM>
                if isempty(dims) || any(dims < 1), dims = 1; end
                [v, used, k0] = t_t07_mcu_sil.value(typ, dims, t, sc, nm, {}, used, k0, nm);
                ds = ds.addElement(v, nm);
            end
            used = unique(used);
        end

        function [v, used, k] = value(typ, dims, t, sc, port, comps, used, k, slot)
        %VALUE  The stimulus of one input leaf (a timeseries) or bus (a struct of timeseries); comps are the element
        %   names below the root port, slot the name of the enclosing element (the CAN slot of a frame). A nested
        %   element that Simulink.Bus.createObject made names its bus object without the 'Bus: ' prefix.
            typ = strtrim(typ);
            if ~startsWith(typ, 'Bus:') && isvarname(typ) && evalin('base', sprintf( ...
                    'exist(''%s'', ''var'') == 1 && isa(%s, ''Simulink.Bus'')', typ, typ))
                typ = ['Bus: ' typ];
            end
            if startsWith(typ, 'Bus:')
                B = evalin('base', strtrim(typ(5:end)));
                en = arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false);
                if all(ismember({'id', 'len', 'data', 'valid'}, en))
                    k = k + 1;
                    [v, used] = t_t07_mcu_sil.frame(B, t, sc, port, comps, used, k, slot);
                    return
                end
                v = struct();
                for e = 1:numel(B.Elements)
                    el = B.Elements(e);
                    [v.(el.Name), used, k] = t_t07_mcu_sil.value(el.DataType, el.Dimensions, t, sc, port, ...
                        [comps, {el.Name}], used, k, el.Name);
                end
                return
            end
            k = k + 1;
            w = prod(dims);
            key = strjoin([{port}, comps], '.');
            [x, hit] = t_t07_mcu_sil.lookup(sc, port, comps, t, w);
            if hit
                used{end + 1} = key;
            else
                try
                    x = t_t07_mcu_sil.pattern(typ, numel(t), w, k);
                catch e
                    error('t_t07_mcu_sil:leaf', 'input leaf %s of type ''%s'': %s', key, typ, e.message);
                end
            end
            nm = port; if ~isempty(comps), nm = comps{end}; end
            try
                v = timeseries(t_t07_mcu_sil.castTo(x, typ, dims), t, 'Name', nm);
            catch e
                error('t_t07_mcu_sil:leaf', 'input leaf %s of type ''%s'': %s', key, typ, e.message);
            end
        end

        function [x, hit] = lookup(sc, port, comps, t, w)
        %LOOKUP  A scenario value for the leaf port.comps: an override (sc.ov, key 'port.a.b'), or a default of the
        %   flat struct sc.d (mcu_ctrl_harness names: 'a_b' below a bus port, the port name for a plain port; a
        %   bus-typed default is walked by its field names).
            x = []; hit = false;
            key = strjoin([{port}, comps], '.');
            if isKey(sc.ov, key)
                x = t_t07_mcu_sil.series(sc.ov(key), t, w); hit = true;
                return
            end
            if isempty(sc.d), return, end
            if isempty(comps), c = {port}; else, c = comps; end
            for i = numel(c):-1:1
                f = strjoin(c(1:i), '_');
                if isfield(sc.d, f)
                    v = sc.d.(f);
                    for j = i+1:numel(c)
                        if isstruct(v) && isfield(v, c{j}), v = v.(c{j}); else, return, end
                    end
                    if isstruct(v) && ~isfield(v, 'series'), return, end
                    x = t_t07_mcu_sil.series(v, t, w); hit = true;
                    return
                end
            end
        end

        function [v, used] = frame(B, t, sc, port, comps, used, k, slot)
        %FRAME  A CAN frame slot: the slot's identifier, 8 normal frames, a dropped one, a foreign identifier, a short
        %   frame (length 4) and a normal one, payload bytes patterned per sample; scenario overrides win.
            N = numel(t); i = (0:N-1)'; c = mod(i, 12);
            dl = B.Elements(strcmp(arrayfun(@(e) e.Name, B.Elements, 'UniformOutput', false), 'data')).Dimensions;
            w = prod(dl);
            id0 = t_t07_mcu_sil.slotId(slot);
            if isnan(id0), id0 = 256 + mod(37*k, 1024); end
            id = id0*ones(N, 1); id(c == 9) = id0 + 1;
            len = w*ones(N, 1); len(c == 10) = 4;
            valid = c ~= 8;
            data = mod(37*i + 11*(1:w) + 7*k, 256);
            data(c == 10, 5:end) = 0;
            vals = struct('id', id, 'len', len, 'data', data, 'valid', valid);
            v = struct();
            for e = 1:numel(B.Elements)
                el = B.Elements(e);
                [x, hit] = t_t07_mcu_sil.lookup(sc, port, [comps, {el.Name}], t, prod(el.Dimensions));
                if hit
                    used{end + 1} = strjoin([{port}, comps, {el.Name}], '.'); %#ok<AGROW>
                elseif isfield(vals, el.Name)
                    x = vals.(el.Name);
                else
                    x = t_t07_mcu_sil.pattern(el.DataType, N, prod(el.Dimensions), k + e);
                end
                v.(el.Name) = timeseries(t_t07_mcu_sil.castTo(x, el.DataType, el.Dimensions), t, 'Name', el.Name);
            end
        end

        function id = slotId(slot)
        %SLOTID  The CAN identifier of a receive slot (fw_io_in_hw's dispatch table); NaN for another slot.
            P = evalin('base', 'P'); ci = P.electronics.can.id; dc = P.drive.can;
            m = struct('fb1_L', dc.id_fb1 + 1, 'fb1_R', dc.id_fb1 + 2, 'fb3_L', dc.id_fb3 + 1, 'fb3_R', dc.id_fb3 + 2, ...
                'fb2_L', dc.id_fb2 + 1, 'fb2_R', dc.id_fb2 + 2, 'impl_fb', ci.impl_fb, 'limits', ci.bms_limits, ...
                'soc', ci.bms_soc, 'vit', ci.bms_vit, 'alarm', ci.bms_alarm, 'cells', ci.bms_cells, ...
                'state', ci.bms_state, 'power', ci.bms_power, 'comp_rx', ci.comp_cmd, 'can2_comp', ci.comp_cmd);
            if isfield(m, slot), id = double(m.(slot)); else, id = NaN; end
        end

        function x = pattern(type, N, w, k)
        %PATTERN  A deterministic N x w pattern of the type, each value held for 7 to 19 samples.
            i = (0:N-1)'; j = 0:w-1;
            H = 7 + mod(5*k, 13);
            s = floor(i/H);
            idx = mod(s + j + k, 8) + 1;
            type = t_t07_mcu_sil.enumName(strtrim(type));
            if startsWith(type, 'Enum:')
                ev = double(enumeration(strtrim(type(6:end))));
                x = ev(mod(s + j + k, numel(ev)) + 1);
                return
            end
            switch type
                case 'boolean'
                    x = mod(s + j + k, 2);
                case {'single', 'double'}
                    b = [0 1.5 -2.25 10.125 -0.0078125 100 -100 0.5];
                    x = b(idx);
                otherwise
                    lo = double(intmin(type)); hi = double(intmax(type));
                    b = [lo hi 0 1 floor((lo + hi)/2) hi - 1 lo + 1 37];
                    x = b(idx);
            end
            x = reshape(x, N, w);
        end

        function type = enumName(type)
        %ENUMNAME  'Enum: <class>' for an enumeration class named with or without the 'Enum: ' prefix.
            if ~startsWith(type, 'Enum:') && ~any(strcmp(type, {'double', 'single', 'boolean', 'int8', 'uint8', ...
                    'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'})) && exist(type, 'class') == 8
                mc = meta.class.fromName(type);
                if ~isempty(mc) && ~isempty(mc.EnumerationMemberList), type = ['Enum: ' type]; end
            end
        end

        function y = castTo(x, type, dims)
            type = t_t07_mcu_sil.enumName(strtrim(type));
            if startsWith(type, 'Enum:')
                y = feval(strtrim(type(6:end)), x);
            elseif strcmp(type, 'boolean')
                y = logical(x);
            else
                y = cast(x, type);
            end
            N = size(y, 1);
            if numel(dims) >= 2 && all(dims > 1)            % a matrix signal: time along the last dimension
                y = permute(reshape(y, [N, dims(:)']), [2:numel(dims) + 1, 1]);
            end
        end

        function x = series(v, t, w)
        %SERIES  N x w values on the grid t: a function handle of t, struct('series', N x w), a constant row of
        %   width w, or a ZOH table [t v(1..w)] (each row applies from its time on).
            N = numel(t);
            if isa(v, 'function_handle')
                x = double(v(t));
            elseif isstruct(v)
                x = double(v.series);
            else
                v = double(v);
                if numel(v) == w
                    x = repmat(v(:)', N, 1);
                elseif size(v, 2) == w + 1
                    x = zeros(N, w);
                    for r = 1:size(v, 1)
                        sel = t >= v(r, 1) - 1e-9;
                        x(sel, :) = repmat(v(r, 2:end), nnz(sel), 1);
                    end
                else
                    error('t_t07_mcu_sil:series', 'a value of %d columns for a leaf of width %d', size(v, 2), w);
                end
            end
            if size(x, 1) ~= N, error('t_t07_mcu_sil:series', '%d samples for a grid of %d', size(x, 1), N); end
        end

        function d = setd(d, path, v, optional)
        %SETD  Set a default of a flat mcu_ctrl_harness struct by its dotted path ('rc.arm' sets d.rc_arm,
        %   'drive_fb.L.w_m' the field of the bus default d.drive_fb); an unknown path is an error unless optional.
            c = strsplit(path, '.');
            for i = numel(c):-1:1
                f = strjoin(c(1:i), '_');
                if isfield(d, f)
                    if i == numel(c)
                        d.(f) = v;
                        return
                    end
                    s = d.(f);
                    ok = true;
                    for j = i+1:numel(c)
                        if isstruct(s) && isfield(s, c{j}), s = s.(c{j}); else, ok = false; break, end
                    end
                    if ok
                        d.(f) = setfield(d.(f), c{i+1:end}, v); %#ok<SFLD>
                        return
                    end
                end
            end
            if nargin < 4 || ~optional, error('t_t07_mcu_sil:setd', 'no default %s', path); end
        end

        function sc = newScenario(T, desc, d)
            sc = struct('T', T, 'desc', desc, 'd', d, 'ov', containers.Map('KeyType', 'char', 'ValueType', 'any'), ...
                'ovKeys', {{}});
        end

        function sc = ovSet(sc, key, v)
            sc.ov(key) = v;
            sc.ovKeys = unique([sc.ovKeys, {key}]);
        end

        function [b, l, er] = sbusStream(N)
        %SBUSSTREAM  1 ms SBUS chunks (Bus_SbusStream) through the cases of TEL01 to TEL06: clean frames every 14 ms
        %   with random channels (TEL01) to 2 s; byte errors (bit flips with uart_err in each chunk, a flagged chunk
        %   without a visible flip, channels 2047 and 100, a trailing byte) to 3 s (TEL03); the SBUS2 footers 0x04,
        %   0x14, 0x24, 0x34 and three invalid 0x01 footers to 3.5 s (TEL04); frame_lost on one frame in five and
        %   failsafe on some to 4.2 s (TEL06); no frame from 4.2 to 4.6 s, the link lost (TEL05); frames again with a
        %   dropped byte in one in seven (TEL02).
            rng(707);
            st = (5:14:N - 30)';
            tt = st*1e-3;
            keep = ~(tt >= 4.2 & tt < 4.6);
            st = st(keep); tt = tt(keep);
            M = numel(st);
            ch = 172 + floor(rand(M, 16)*(1811 - 172 + 1));
            flags = false(M, 4); footer = zeros(M, 1);
            fB = find(tt >= 2.0 & tt < 3.0); fC = find(tt >= 3.0 & tt < 3.5);
            fD = find(tt >= 3.5 & tt < 4.2); fF = find(tt >= 4.6);
            if numel(fB) >= 6, ch(fB(2), 3) = 2047; ch(fB(5), 1) = 100; end
            fts = [4 20 36 52];
            if ~isempty(fC)
                footer(fC) = fts(mod(0:numel(fC)-1, 4) + 1);
                footer(fC(max(1, end-2):end)) = 1;
            end
            flags(fD(1:5:end), 3) = true; flags(fD(3:9:end), 4) = true;
            fr = sbus_ref_encode(ch, flags, footer);
            [b, l, er] = mcu_sbus_stream(fr, st, N);
            for q = 1:3:numel(fB)                          % bit flip + uart_err, chunk 1, 2, 3 in turn
                m = fB(q); cnk = mod(q, 3) + 1; tk = st(m) + cnk - 1;
                if tk <= N && l(tk) > 2
                    b(tk, 2) = bitxor(b(tk, 2), uint8(4)); er(tk) = true;
                end
            end
            if numel(fB) >= 8
                er(st(fB(8)) + 1) = true;                   % flagged chunk, no visible flip
                tk = st(fB(7)) + 2; if tk <= N, l(tk) = l(tk) + 1; end   % trailing byte 0x00 after the footer
            end
            for q = 1:7:numel(fF)                          % one byte dropped in chunk 2
                tk = st(fF(q)) + 1;
                if tk <= N && l(tk) > 1
                    b(tk, 3:end) = [b(tk, 4:end), uint8(0)]; l(tk) = l(tk) - 1;
                end
            end
        end

        function w = diWord(names)
            di = evalin('base', 'P.electronics.di');
            w = 0;
            for k = 1:numel(names)
                if isfield(di, names{k}), w = w + 2^di.(names{k}); end
            end
        end

        % ================================================================== the scenarios
        function sc = scenario(u)
            P = evalin('base', 'P');
            healthy = t_t07_mcu_sil.diWord({'estop_ch1_ok', 'estop_ch2_ok', 'safety_ok', 'lid_closed', 'key_on', ...
                'safety_hv_ok'});
            switch u
                case 'fw_sbus_decoder'
                    T = 6.0;
                    sc = t_t07_mcu_sil.newScenario(T, ['6 s of 1 ms SBUS chunks through the cases of TEL01 to TEL06: ' ...
                        'clean frames with random channels, byte errors with uart_err, channels 2047 and 100, a ' ...
                        'trailing byte, SBUS2 footers and an invalid footer, frame_lost and failsafe flags, a link ' ...
                        'loss of 0.4 s, dropped bytes'], []);
                    N = round(T/1e-3) + 1;
                    [b, l, er] = t_t07_mcu_sil.sbusStream(N);
                    sc = t_t07_mcu_sil.ovSet(sc, 'sbus.bytes', struct('series', b));
                    sc = t_t07_mcu_sil.ovSet(sc, 'sbus.len', struct('series', l));
                    sc = t_t07_mcu_sil.ovSet(sc, 'sbus.uart_err', struct('series', er));
                case 'fw_rc_mapping'
                    T = 6.0;
                    sc = t_t07_mcu_sil.newScenario(T, ['6 s of decoded frames: each channel steps through 172, ' ...
                        '400, 992, 1500 and 1811 at its own rate (sticks, dead band and expo; the 2- and 3-position ' ...
                        'switches with their debounce; CH12 remote E-stop; CH13 mission and CH14 return-to-home ' ...
                        'edges), a new frame on two ticks in three, the link lost with failsafe from 4.0 to 4.3 s'], []);
                    lv = [172 400 992 1500 1811];
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.ch_raw', @(t) t_t07_mcu_sil.stairs(t, lv, 16));
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.frame_new', @(t) double(mod(round(t/0.01), 3) ~= 2));
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.frame_valid', [0 1; 4.0 0; 4.3 1]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.rc_ok', [0 1; 4.0 0; 4.3 1]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.rc_link_lost', [0 0; 4.1 1; 4.3 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'rc_dec.failsafe', [0 0; 4.0 1; 4.3 0]);
                case 'fw_imu_tilt'
                    T = 8.0;
                    sc = t_t07_mcu_sil.newScenario(T, ['8 s of IMU samples of a body that rolls up to 0.4 rad and ' ...
                        'pitches up to 0.35 rad, with yaw and pitch rates, a track acceleration and speeds, and the ' ...
                        'IMU invalid from 3.0 to 3.2 s'], []);
                    g = 9.80665;
                    rl = @(t) 0.4*sin(2*pi*t/4); pt = @(t) 0.35*sin(2*pi*t/6);
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.accel_B', @(t) g*[-sin(pt(t)), sin(rl(t)).*cos(pt(t)), ...
                        cos(rl(t)).*cos(pt(t))] + [0.3*sin(3*t), zeros(numel(t), 2)]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.gyro_B', @(t) [0.4*pi/2*cos(2*pi*t/4), ...
                        0.35*pi/3*cos(2*pi*t/6), 0.2*sin(0.5*t)]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.mag_B', [20 0 -40]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.temp_degC', [0 30]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.t_us', @(t) round(t*1e6));
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.seq', @(t) mod(round(t/0.005), 65536));
                    sc = t_t07_mcu_sil.ovSet(sc, 'imu_meas.valid', [0 1; 3.0 0; 3.2 1]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'a_track', @(t) 0.5*sin(t));
                    sc = t_t07_mcu_sil.ovSet(sc, 'v_track_LR', @(t) [0.8 + 0.2*sin(t), 0.8 - 0.2*sin(t)]);
                case {'fw_can1_codec', 'fw_can2_codec', 'fw_io_in_sim', 'fw_io_out_sim'}
                    T = 4.0;
                    if startsWith(u, 'fw_io'), T = 1.5; end
                    sc = t_t07_mcu_sil.newScenario(T, sprintf(['%.1f s: every CAN frame slot with its identifier, ' ...
                        '8 normal frames, a dropped frame, a foreign identifier and a short frame (length 4) in turn, ' ...
                        'patterned payload bytes (as t_t07_frames); every other input a held pattern of its type ' ...
                        '(minimum, maximum, zero and edge values)'], T), []);
                    if strcmp(u, 'fw_can2_codec')
                        sc = t_t07_mcu_sil.ovSet(sc, 't_ms', @(t) round(t*1e3));
                    end
                case 'fw_adc_scaling'
                    T = 4.0;
                    sc = t_t07_mcu_sil.newScenario(T, ['4 s: each ADC channel steps through 0, 1, 1000, 2048, 3000, ' ...
                        '4094, 4095 and 2500 counts at its own rate; the MCU temperature through -40, 25, 85, 125 and ' ...
                        '0 degC'], []);
                    sc = t_t07_mcu_sil.ovSet(sc, 'adc_raw', @(t) t_t07_mcu_sil.stairs(t, [0 1 1000 2048 3000 4094 ...
                        4095 2500], 8));
                    sc = t_t07_mcu_sil.ovSet(sc, 'mcu_temp_degC', [0 -40; 0.8 25; 1.6 85; 2.4 125; 3.2 0]);
                case 'fw_mode_manager'
                    d = mcu_ctrl_harness('defaults', 'ctrl');
                    S = @t_t07_mcu_sil.setd;
                    rst = [0 0; 12.0 1; 12.05 0; 14.0 1; 14.05 0; 16.5 1; 16.55 0];
                    d = S(d, 'rc.arm', [0 0; 1.5 1]);
                    d = S(d, 'rc.arm_req', [0 0; 1.5 1; 1.52 0; 9.2 1; 9.22 0; 10.5 1; 10.52 0]);
                    d = S(d, 'rc.sw_mode', [0 1; 2.5 2; 3.0 1; 4.0 3; 8.0 1; 8.3 3; 9.0 1]);
                    d = S(d, 'comp_ok', [0 0; 3.5 1]);
                    d = S(d, 'gnc_cmd.gnc_ready', [0 0; 3.5 1]);
                    d = S(d, 'gnc_cmd.mode_req', [0 0; 3.5 2; 8.6 4; 8.8 2]);
                    d = S(d, 'rc.override', [0 0; 7.5 1; 7.7 0]);
                    d = S(d, 'rc.rc_ok', [0 1; 9.6 0; 10.0 1]);
                    d = S(d, 'rc.estop_remote', [0 0; 11.0 1; 11.5 0]);
                    d = S(d, 'reset_req', rst);
                    d = S(d, 'rc.reset_req', rst);
                    d = S(d, 'reset_evt', [0 0; 14.5 1; 14.51 0]);
                    sc = t_t07_mcu_sil.newScenario(17.0, ['17 s through the modes (healthy mcu_ctrl_harness defaults): ' ...
                        'self-test to STANDBY, arm to MANUAL, HOLD and back, AUTO_START and AUTO with the companion ' ...
                        'ready, the operator override to MANUAL, AUTO_START again and the companion FAULT_STOP to ' ...
                        'CTRL_STOP and the park to STANDBY, MANUAL and the RC link lost to CTRL_STOP, a class-B fault ' ...
                        'in MANUAL, the remote E-stop and the reset, a class-A fault to FAULT and the reset, a watchdog ' ...
                        'reset event to BOOT, the hardware E-stop (bit 0) and the reset'], d);
                    sc = t_t07_mcu_sil.ovSet(sc, 'fault_word', [0 0; 10.7 2^13; 10.8 0; 12.5 2^7; 13.5 0; 15.8 1; ...
                        16.2 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'fault_cls', [0 0 0 0; 10.7 0 1 0; 10.8 0 0 0; 12.5 1 0 0; ...
                        13.5 0 0 0; 15.8 1 0 0; 16.2 0 0 0]);
                case 'fw_safety_supervisor'
                    d = mcu_ctrl_harness('defaults', 'sup');
                    S = @t_t07_mcu_sil.setd;
                    rst = [0 0; 16.0 1; 16.05 0];
                    d = S(d, 'mode_prev', [0 2; 1.0 3; 4.5 6; 5.5 3]);
                    d = S(d, 'di_word', [0 healthy; 1.5 healthy - t_t07_mcu_sil.diWord({'estop_ch1_ok'}); 2.0 healthy; ...
                        14.5 healthy - t_t07_mcu_sil.diWord({'lid_closed'}); 15.0 healthy; ...
                        15.2 healthy + t_t07_mcu_sil.diWord({'bumper_front'}); 15.4 healthy]);
                    d = S(d, 'rc.estop_remote', [0 0; 2.5 1; 3.0 0]);
                    d = S(d, 'rc.rc_ok', [0 1; 3.5 0; 4.0 1]);
                    d = S(d, 'rc.rc_link_lost', [0 0; 3.5 1; 4.0 0]);
                    d = S(d, 'comp_ok', [0 1; 4.7 0; 5.3 1]);
                    d = S(d, 'comp_age', [0 0; 4.7 0.5; 5.3 0]);
                    d = S(d, 'drv_age', [0 0 0; 6.0 0.2 0; 6.5 0 0]);
                    d = S(d, 'drive_fb.L.fault_code', [0 0; 7.0 1; 7.5 0]);
                    d = S(d, 'bms_age', [0 0; 8.0 1.0; 8.5 0]);
                    d = S(d, 'adc.V24', [0 24; 9.0 18; 9.5 24]);
                    d = S(d, 'adc.V5', [0 5; 9.2 4.0; 9.5 5]);
                    d = S(d, 'adc.T_encl', [0 318.15; 10.0 365; 10.5 318.15]);
                    d = S(d, 'tilt.roll', [0 0; 11.0 0.5; 11.5 0]);
                    d = S(d, 'tilt.pitch', [0 0; 11.0 0.45; 11.5 0]);
                    d = S(d, 'tilt.imu_valid', [0 1; 11.6 0; 12.0 1]);
                    d = S(d, 'odom.vTrack_LR', [0 0 0; 12.5 2.5 2.5; 13.0 0 0]);
                    d = S(d, 'vtrack_ref_prev', [0 0 0; 12.5 0.5 0.5; 13.0 0 0]);
                    d = S(d, 'overrun', [0 0; 13.5 1; 13.6 0]);
                    d = S(d, 'busoff', [0 0 0; 13.7 1 0; 14.0 0 0]);
                    d = S(d, 'impl.valid', [0 0; 13.9 1]);
                    d = S(d, 'impl.rx_ok', [0 0; 13.9 1]);
                    d = S(d, 'impl.fault', [0 0; 14.0 1; 14.4 0]);
                    d = S(d, 'reset_req', rst);
                    d = S(d, 'rc.reset_req', rst);
                    d = S(d, 'stop_fail_prev', [0 0; 16.5 1; 16.6 0]);
                    sc = t_t07_mcu_sil.newScenario(17.5, ['17.5 s of faults on healthy mcu_ctrl_harness defaults, in ' ...
                        'MANUAL and AUTO: one E-stop channel open, the remote E-stop, the RC link lost, the companion ' ...
                        'lost in AUTO, a drive feedback time-out, a drive fault code, a BMS time-out, the 24 V and 5 V ' ...
                        'rails low, the enclosure hot, a tilt beyond the limits, the IMU invalid, an overspeed against ' ...
                        'the track reference, an overrun, a CAN bus-off, an implement fault, the lid open, the front ' ...
                        'bumper, the reset, and a stop-monitor failure'], d);
                case 'fw_vehicle_ctrl'
                    d = mcu_ctrl_harness('defaults', 'ctrl');
                    S = @t_t07_mcu_sil.setd;
                    dw = 0.8*P.electronics.safety.dcl_warn;
                    d = S(d, 'rc.v_rc', [0 0; 2.0 1.0; 4.0 0; 6.5 2.0; 8.0 0]);
                    d = S(d, 'rc.w_rc', [0 0; 4.5 0.5; 6.0 0; 6.5 1.0; 8.0 0]);
                    d = S(d, 'tilt.yaw_rate_meas', [0 0; 6.5 0.5; 8.0 0]);
                    d = S(d, 'drive_fb.L.w_m', [0 0; 8.5 300; 10.5 0]);
                    d = S(d, 'drive_fb.R.w_m', [0 0; 8.5 -300; 10.5 0]);
                    d = S(d, 'bms.DCL', [0 300; 8.5 100; 9.0 dw; 9.5 100]);
                    d = S(d, 'bms.CCL', [0 100; 9.5 0; 10.0 100]);
                    d = S(d, 'tilt.imu_valid', [0 1; 10.0 0; 10.4 1]);
                    d = S(d, 'gnc_cmd.cmd_valid', [0 0; 5.0 1; 6.0 0]);
                    d = S(d, 'gnc_cmd.v_cmd', [0 0; 5.0 0.8; 6.0 0]);
                    d = S(d, 'gnc_cmd.omega_cmd', [0 0; 5.0 0.3; 6.0 0]);
                    sc = t_t07_mcu_sil.newScenario(12.5, ['12.5 s on healthy mcu_ctrl_harness defaults: MANUAL with ' ...
                        'the shaper steps of TEL10 (1.0 m/s, then 0.5 rad/s), the companion command in AUTO, the ' ...
                        'mixer case of TEL09 (2.0 m/s and 1.0 rad/s, desaturated), the torque-limit cases of TEL11 ' ...
                        '(motor speeds +/-300 rad/s, DCL 100 A and below dcl_warn, CCL 0, the IMU invalid), a derate ' ...
                        'to 0.25, limits_hold, CTRL_STOP, ESTOP and the drive disabled'], d);
                    sc = t_t07_mcu_sil.ovSet(sc, 'mode', [0 2; 1.5 3; 5.0 6; 6.0 3; 11.0 7; 11.4 8; 11.8 2]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'cmd_src', [0 0; 1.5 1; 5.0 2; 6.0 1; 11.8 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'drive_enable', [0 0; 1.5 1; 11.8 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'fault_word', [0 0; 10.6 2^24; 11.0 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'k_der', [0 1; 10.6 0.25; 11.0 1]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'limits_hold', [0 0; 11.0 1; 11.3 0]);
                case 'fw_implement_aux'
                    d = mcu_ctrl_harness('defaults', 'aux');
                    S = @t_t07_mcu_sil.setd;
                    d = S(d, 'st.mode', [0 2; 1.0 3; 5.0 6; 8.0 7; 9.0 8; 10.0 9; 11.0 2]);
                    d = S(d, 'st.cmd_src', [0 0; 1.0 1; 5.0 2; 8.0 0]);
                    d = S(d, 'st.rc.sw_impl', [0 1; 1.5 3; 3.0 2; 4.0 3; 6.0 1]);
                    d = S(d, 'st.rc.sw_lift', [0 2; 2.0 1; 2.5 3; 3.0 2]);
                    d = S(d, 'st.rc.lights', [0 0; 3.0 1; 6.0 0]);
                    d = S(d, 'st.v_track', [0 0; 2.0 0.8; 7.5 0]);
                    d = S(d, 'st.fault_word', [0 0; 10.0 2^7; 11.0 0]);
                    d = S(d, 'impl.impl_type', [0 0; 0.5 1; 4.0 2; 7.0 3]);
                    d = S(d, 'impl.valid', [0 1]);
                    d = S(d, 'impl.rx_ok', [0 1]);
                    d = S(d, 'impl.ready', [0 0; 1.0 1]);
                    d = S(d, 'impl.running', [0 0; 2.0 1; 8.0 0]);
                    d = S(d, 'impl.fault', [0 0; 9.5 1; 10.5 0]);
                    d = rmfield(d, 'gnc_cmd');               % the AUTO implement commands: held patterns
                    sc = t_t07_mcu_sil.newScenario(12.0, ['12 s on healthy mcu_ctrl_harness defaults: STANDBY, MANUAL ' ...
                        'with the implement switch through OFF, AUTO and ON and the lift switch, AUTO with the ' ...
                        'companion''s implement commands (held patterns), CTRL_STOP, ESTOP and FAULT; the mower, the ' ...
                        'tiller and the sprayer fitted in turn, running, an implement fault, the lights'], d);
                case 'fw_watchdog_mgr'
                    d = mcu_ctrl_harness('defaults', 'wdg');
                    sc = t_t07_mcu_sil.newScenario(3.0, ['3 s at 1 ms: the alive counters of Task_10, Task_50 and ' ...
                        'Task_100 at their rates, Task_10''s frozen from 1.0 to 1.3 s and Task_100''s from 2.0 to 2.5 s ' ...
                        '(no kick), a stall injected, a maskDO0 fault bit, the mode from BOOT to STANDBY and FAULT, and ' ...
                        'two re-arm requests (the DO1 pulse)'], d);
                    sc = t_t07_mcu_sil.ovSet(sc, 'alive', @t_t07_mcu_sil.alive);
                    sc = t_t07_mcu_sil.ovSet(sc, 'fault_inject', [0 0; 1.6 1; 1.7 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'fault_word', [0 0; 0.5 2^7; 0.7 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'mode', [0 0; 0.1 1; 0.3 2; 2.7 9]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'rearm_req', [0 0; 0.8 1; 0.85 0; 2.8 1; 2.85 0]);
                case 'Shaddad_MCU'
                    T = 6.0;
                    sc = t_t07_mcu_sil.newScenario(T, ['6 s of the MCU''s root inputs: the SBUS stream of the decoder ' ...
                        'test (TEL01 to TEL06 cases), a healthy DI word with one E-stop channel open from 2.0 to 2.5 s, ' ...
                        'the microsecond clock, every CAN frame slot with its identifier in the cycle of normal, ' ...
                        'dropped, foreign and short frames with patterned payloads, every other input a held pattern ' ...
                        'of its type'], []);
                    N = round(T/1e-3) + 1;
                    [b, l, er] = t_t07_mcu_sil.sbusStream(N);
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.sbus.bytes', struct('series', b));
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.sbus.len', struct('series', l));
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.sbus.uart_err', struct('series', er));
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.di_word', [0 healthy; 2.0 healthy - ...
                        t_t07_mcu_sil.diWord({'estop_ch1_ok'}); 2.5 healthy]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.t_us', @(t) round(t*1e6));
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.reset_flag', [0 0]);
                    sc = t_t07_mcu_sil.ovSet(sc, 'in1k.fault_inject', [0 0]);
                otherwise
                    error('t_t07_mcu_sil:scenario', 'no scenario for %s', u);
            end
        end

        function x = stairs(t, lv, w)
        %STAIRS  N x w levels: column c steps through lv, each level held (0.3 + 0.05 c) s, offset by c.
            x = zeros(numel(t), w);
            for c = 1:w
                H = 0.3 + 0.05*c;
                x(:, c) = lv(mod(floor(t/H) + c, numel(lv)) + 1);
            end
        end

        function a = alive(t)
        %ALIVE  The alive counters of Task_10, Task_50 and Task_100 (+1 per task period, mod 256); Task_10's frozen
        %   from 1.0 to 1.3 s, Task_100's from 2.0 to 2.5 s.
            t1 = t; t1(t >= 1.0 & t < 1.3) = 1.0;
            t3 = t; t3(t >= 2.0 & t < 2.5) = 2.0;
            a = mod([floor(t1/0.01 + 1e-9), floor(t/0.05 + 1e-9), floor(t3/0.1 + 1e-9)], 256);
        end

        % ================================================================== simulate and compare
        function R = simulate(mdl, ds, T, mode)
            in = Simulink.SimulationInput(mdl);
            in = in.setVariable('t07_u', ds);           % the inputs by name (setExternalInput clashes with the copy)
            in = in.setModelParameter('StopTime', sprintf('%.6g', T), 'LoadExternalInput', 'on', 'ExternalInput', ...
                't07_u', ...
                'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'SaveTime', 'off', ...
                'SaveState', 'off', 'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');
            switch mode
                case 'normal', in = in.setModelParameter('SimulationMode', 'normal');
                case 'sil', in = in.setModelParameter('SimulationMode', 'software-in-the-loop (sil)');
            end
            t0 = tic;
            out = sim(in);
            R.wall = toc(t0);
            if ~isempty(out.ErrorMessage), error('t_t07_mcu_sil:sim', '%s', out.ErrorMessage); end
            R.y = out.yout;
        end

        function [outs, allpass] = compare(yA, yB)
        %COMPARE  Every leaf of every root output of yA (normal) against yB (SIL).
            outs = {};
            allpass = true;
            for k = 1:numElements(yA)
                eA = yA.getElement(k);
                nm = eA.Name;
                if isempty(nm), nm = get_param(eA.BlockPath.getBlock(1), 'Name'); end
                eB = [];
                for j = 1:numElements(yB)
                    e = yB.getElement(j);
                    if strcmp(e.Name, eA.Name) && isequal(e.BlockPath.getBlock(1), eA.BlockPath.getBlock(1))
                        eB = e; break
                    end
                end
                LA = t_t07_mcu_sil.leaves(eA.Values, nm);
                if isempty(eB), LB = cell(0, 2); else, LB = t_t07_mcu_sil.leaves(eB.Values, nm); end
                for i = 1:size(LA, 1)
                    a = LA{i, 2};
                    j = find(strcmp(LB(:, 1), LA{i, 1}), 1);
                    if isempty(j), s = timeseries(); else, s = LB{j, 2}; end
                    ty = class(a.Data);
                    if isenum(a.Data), ty = 'enumeration'; end
                    sameT = isequal(size(a.Time), size(s.Time)) && max([0; abs(a.Time - s.Time)]) <= 1e-9;
                    da = t_t07_mcu_sil.tmajor(double(a.Data), numel(a.Time));
                    db = t_t07_mcu_sil.tmajor(double(s.Data), numel(s.Time));
                    sameT = sameT && isequal(size(da), size(db));
                    if sameT
                        e = abs(da(:) - db(:));
                        e(da(:) == db(:) | (isnan(da(:)) & isnan(db(:)))) = 0;
                        e(isnan(e)) = Inf;
                        switch ty
                            case 'single', rel = 1e-5;
                            case 'double', rel = 1e-9;
                            otherwise, rel = 0;
                        end
                        allow = rel*max(1, abs(da(:)));
                        allow(~isfinite(allow)) = 0;
                        ok = all(e <= allow);
                        [err, kmax] = max(e);
                        if isempty(err), err = 0; tol = 0; else, tol = allow(kmax); end
                    else
                        err = NaN; tol = 0; ok = false;
                    end
                    % what the stimulus made the output do (normal run): the values an integer, boolean or
                    % enumeration output took (up to 16), or the count of its distinct values
                    uv = unique(da(isfinite(da)));
                    if ~any(strcmp(ty, {'single', 'double'})) && numel(uv) <= 16
                        seen = num2cell(uv(:)');                % a JSON list, also for one value
                    else
                        seen = sprintf('%d distinct values', numel(uv));
                    end
                    outs{end + 1} = struct('name', LA{i, 1}, 'type', ty, 'max_abs_err', err, 'tol', tol, ...
                        'same_samples', sameT, 'pass', ok, 'values_seen', {seen}); %#ok<AGROW>
                    allpass = allpass && ok;
                end
            end
        end

        function L = leaves(v, prefix)
            L = cell(0, 2);
            if isa(v, 'timeseries')
                L = {prefix, v};
                return
            end
            if ~isstruct(v), return, end
            fn = fieldnames(v);
            for k = 1:numel(fn)
                for q = 1:numel(v)
                    nm = [prefix '.' fn{k}];
                    if numel(v) > 1, nm = sprintf('%s(%d).%s', prefix, q, fn{k}); end
                    L = [L; t_t07_mcu_sil.leaves(v(q).(fn{k}), nm)]; %#ok<AGROW>
                end
            end
        end

        function d = tmajor(d, nt)
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

        function tf = refused(msg)
            tf = ~isempty(regexpi(msg, ['(access is denied|smart app|code integrity|blocked by|has been blocked|' ...
                'failed to start|could not start|unable to start|cannot start|not able to start|could not be ' ...
                'started|failed to launch|unable to launch|could not launch|error launching|0xc0000|policy|' ...
                'connection.*(timed out|refused)|timed out.*connect|did not respond)'], 'once'));
        end

        function [on, txt] = sacOn()
            try
                v = winqueryreg('HKEY_LOCAL_MACHINE', 'SYSTEM\CurrentControlSet\Control\CI\Policy', ...
                    'VerifiedAndReputablePolicyState');
                on = double(v) == 1;
                txt = sprintf('Smart App Control state %d (1 = on)', double(v));
            catch
                on = true;
                txt = 'Smart App Control state unreadable';
            end
        end
    end
end
