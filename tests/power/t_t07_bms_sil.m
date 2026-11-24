classdef t_t07_bms_sil < matlab.unittest.TestCase
%T_T07_BMS_SIL  Card T-07 step 3: back-to-back test of the BMS firmware, Shaddad_BMS in normal mode against its
%   generated C in software-in-the-loop (SIL), with the same open-loop stimulus (a discharge, a charge, a fault).
%
%   runtests('t_t07_bms_sil')
%
%   Both runs use an in-memory copy of cs_GenARM (ert.tlc, generic ARM Cortex-M) with PortableWordSizes on and
%   GenCodeOnly off, and the SIL code is built in work/team/t07/sil/build, so the traced code in
%   work/Shaddad_BMS_ert_rtw stays as generated. Every root output is compared: integers, booleans and
%   enumerations bit-exact, single within 1e-5 x max(1, |reference|), the same sample times. The record goes to
%   work/team/t07/sil/Shaddad_BMS.json. When Windows refuses the SIL program while Smart App Control is on
%   (CI policy VerifiedAndReputablePolicyState 1), the test is BLOCKED through an assumption, never a pass, and
%   the record says "blocked"; with Smart App Control off, any SIL build or run error is a failure.
%   The harness is t_t07_bms's (stim, inputs, simulate). The model is closed with close_system(model, 0).

    properties
        RecordFile
        Root
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.Root = char(currentProject().RootFolder);
            tc.RecordFile = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'Shaddad_BMS.json');
            if ~evalin('base', 'exist(''Bus_BMS_Meas'', ''var'') && exist(''P'', ''var'') && exist(''Ts'', ''var'')')
                shaddad_params();
            end
            shaddad_configsets('codegen');
            old = Simulink.fileGenControl('getConfig');
            bdir = fullfile(tc.Root, 'work', 'team', 't07', 'sil', 'build');
            if ~isfolder(bdir), mkdir(bdir); end
            Simulink.fileGenControl('set', 'CodeGenFolder', bdir, 'CacheFolder', bdir, 'createDir', true);
            tc.addTeardown(@() Simulink.fileGenControl('set', 'CodeGenFolder', old.CodeGenFolder, ...
                'CacheFolder', old.CacheFolder));
            load_system(t_t07_bms.Mdl);
            tc.addTeardown(@() close_system(t_t07_bms.Mdl, 0));
            t_t07_bms.useConfig('genarm');
            set_param(t_t07_bms.Mdl, 'GenCodeOnly', 'off');
        end
    end

    methods (Static)
        function S = scenario()
        %SCENARIO  Precharge and HV_ON, a 40 A discharge, the opening, a docked charge at -15 A, then a cell
        %   over-voltage (OV fault from CHARGING) and its reset after recovery.
            b = evalin('base', 'P.power.bms');
            S = t_t07_bms.stim(34);
            v0 = S.meas.V_cell_V(1, 2:end); Vp = sum(v0); vov = v0; vov(1) = b.V_ov + 0.05;
            S.req.HV_Enable_Req = [0 0; 1.0 1; 10.0 0];
            S.meas.V_bus_V = [0 0; 1.2 0.98*Vp; 10.1 0];
            S.meas.K1_fb = [0 0; 1.3 1; 10.05 0];
            S.meas.I_pack_A = [0 0; 2.0 40; 10.0 0; 16.5 -15; 28.0 0];
            S.meas.Dock_Present = [0 0; 16.0 1];
            S.req.Charge_Enable_Req = [0 0; 16.0 1; 29.0 0];
            S.meas.V_dock_V = [0 0; 16.0 54.0];
            S.meas.V_cell_V = [0, v0; 25.0, vov; 28.0, v0];
            S.req.Fault_Reset_Req = [0 0; 30.0 1; 30.3 0];
        end

        function writeRecord(file, rec)
            fid = fopen(file, 'w');
            fprintf(fid, '%s\n', jsonencode(rec, 'PrettyPrint', true));
            fclose(fid);
        end

        function tf = refused(msg)
        %REFUSED  True when the error says that the SIL program could not be started (an operating-system
        %   refusal such as Smart App Control), as opposed to a build or a model error.
            tf = ~isempty(regexpi(msg, ['(access is denied|smart app|code integrity|blocked by|has been blocked|' ...
                'failed to start|could not start|unable to start|cannot start|not able to start|could not be ' ...
                'started|failed to launch|unable to launch|could not launch|error launching|0xc0000|policy|' ...
                'connection.*(timed out|refused)|timed out.*connect|did not respond)'], 'once'));
        end

        function d = tmajor(d, nt)
        %TMAJOR  Logged data as an nt x m matrix with time along the first dimension, whichever dimension the
        %   logging put time on (a vector signal may be logged N x w or w x 1 x N).
            if size(d, 1) == nt && ismatrix(d), return, end
            k = find(size(d) == nt, 1, 'last');
            if isempty(k), return, end
            d = reshape(permute(d, [k, setdiff(1:ndims(d), k)]), nt, []);
        end

        function [on, txt] = sacOn()
        %SACON  True when Windows Smart App Control is enforcing (CI policy VerifiedAndReputablePolicyState 1).
        %   When the state cannot be read, it answers true, so the error text alone decides.
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

    methods (Test, TestTags = {'T07', 'SIL'})
        function back_to_back(tc)
            S = t_t07_bms_sil.scenario();
            [sac0, sactxt0] = t_t07_bms_sil.sacOn();
            if sac0, st0 = 'blocked'; else, st0 = 'fail'; end     % the record if the run never comes back
            rec = struct('model', t_t07_bms.Mdl, 'status', st0, ...
                'stimulus', ['open-loop: precharge and HV_ON at 1 s, a 40 A discharge from 2 to 10 s, the opening at ' ...
                '10 s, a docked charge at -15 A from 16.5 s, cell 1 above V_ov from 25 to 28 s (OV fault from ' ...
                'CHARGING), the reset at 30 s; 34 s'], ...
                'steps', round(S.T/0.01), 'outputs', {{}}, 'wall_s', 0, ...
                'note', ['The SIL run was started and gave no result (record written before it; ' sactxt0 ').']);
            tw = tic;
            Rn = t_t07_bms.simulate(S, 'normal');
            t_t07_bms_sil.writeRecord(tc.RecordFile, rec);
            try
                Rs = t_t07_bms.simulate(S, 'sil');
            catch e
                msg = getReport(e, 'basic');
                rec.wall_s = round(toc(tw), 1);
                [sac, sactxt] = t_t07_bms_sil.sacOn();
                if t_t07_bms_sil.refused(msg) && sac
                    rec.status = 'blocked';
                    rec.note = ['BLOCKED: the SIL program was refused (' sactxt '; T-07 common brief section 6). ' msg];
                    t_t07_bms_sil.writeRecord(tc.RecordFile, rec);
                    tc.assumeFail(rec.note);
                else
                    rec.status = 'fail';
                    rec.note = ['The SIL build or run failed (' sactxt '): ' msg];
                    t_t07_bms_sil.writeRecord(tc.RecordFile, rec);
                    tc.verifyFail(rec.note);
                    return
                end
            end
            outs = {};
            allpass = true;
            for g = {'cmd', 'can', 'tel'}
                LA = t_t07_bms.leaves(Rn.(g{1}), g{1}); LB = t_t07_bms.leaves(Rs.(g{1}), g{1});
                for k = 1:size(LA, 1)
                    a = LA{k, 2};
                    j = find(strcmp(LB(:, 1), LA{k, 1}), 1);
                    if isempty(j), s = timeseries(); else, s = LB{j, 2}; end    % missing in SIL: a failed compare
                    ty = class(a.Data);
                    sameT = isequal(size(a.Time), size(s.Time)) && max(abs(a.Time - s.Time)) <= 1e-9;
                    da = t_t07_bms_sil.tmajor(double(a.Data), numel(a.Time));
                    ds = t_t07_bms_sil.tmajor(double(s.Data), numel(s.Time));
                    sameT = sameT && isequal(size(da), size(ds));
                    if sameT
                        e = abs(da(:) - ds(:));
                        switch ty
                            case 'single', rel = 1e-5;     % the common brief's default for single
                            case 'double', rel = 1e-9;
                            otherwise, rel = 0;            % integers, booleans, enumerations: bit-exact
                        end
                        allow = rel*max(1, abs(da(:)));
                        ok = all(e <= allow);
                        [err, kmax] = max(e);
                        if isempty(err), err = 0; tol = 0; else, tol = allow(kmax); end
                    else
                        err = NaN; tol = 0; ok = false;
                    end
                    if isempty(err), err = 0; end
                    outs{end+1} = struct('name', LA{k, 1}, 'type', ty, 'max_abs_err', err, ...
                        'tol', tol, 'same_samples', sameT, 'pass', ok); %#ok<AGROW>
                    allpass = allpass && ok;
                    tc.verifyTrue(ok, sprintf('%s (%s): max |SIL - normal| %g, tolerance %g, same samples %d', ...
                        LA{k, 1}, ty, err, tol, sameT));
                end
            end
            rec.outputs = outs;
            rec.wall_s = round(toc(tw), 1);
            if allpass, rec.status = 'pass'; else, rec.status = 'fail'; end
            rec.note = sprintf(['%d root output signals compared (the leaves of Bus_BMS_Cmd, Bus_Can1_Bms and ' ...
                'Bus_BMS_Telemetry); normal %.1f s, SIL %.1f s (build included); %s.'], numel(outs), Rn.wall, ...
                Rs.wall, sactxt0);
            t_t07_bms_sil.writeRecord(tc.RecordFile, rec);
            fprintf('MEASURED SIL back-to-back: %s, %d outputs, normal %.1f s, SIL %.1f s\n', rec.status, numel(outs), ...
                Rn.wall, Rs.wall);
        end
    end
end
