classdef gnc_fsm_driver < handle
%GNC_FSM_DRIVER  MATLAB-level driver of gnc_missionFSM for scripted mode-logic tests (T15 test helper, T11).
%
%   F = gnc_fsm_driver(P, plan)   inputs start from P.ic (RTK nav ok, MCU AUTO / link ok / drive_ready,
%                                 SOC 0.8, clear obstacles); the persistent FSM is cleared by F.reset()
%   F.set(path, value)            e.g. F.set('mcu.rc_mode_sw', uint8(0)), F.set('obs.d_obs', 1.0); 'freeze'
%                                 holds the MCU heartbeat, 'teb' the TEB info vector, 'scanOk' the scan validity
%                                 obs.scan_valid (Bus_Perception.scanValid after its Rate Transition; true after reset)
%   F.cmd(MissionCmd.X)           mission command in the next step
%   F.run(n)                      n steps of 0.05 s; the path error is emulated: s advances by v_sim Ts while
%                                 the FSM selects a tracker (not in F.hold), s restarts at 0 on every window
%                                 change (segment, path source, replan version)
%   F.drive(n, stop)              run up to n steps or until stop(F) is true
%   n = F.runUntil(cond, nmax)    steps after the first one until cond(F) (0 = in the first step)
%   F.rpAnswer(ok, delay)         wait for a replan request, answer it after delay steps (ok: ver + 1, else
%                                 fail_count + 1)
%   [ids, M] = F.collect(ids, M)  append the transition IDs fired since the last collect and the 7 x 7 state
%                                 transition counts (rows = from, columns = to, GncState + 1)

    properties
        P
        plan
        nav
        mcu
        bms
        mcmd
        obs
        pe
        rp
        teb = [0; 12; 0; 1]
        socRth = 0.21
        out
        st = GncState.IDLE
        sub = AutoSub.NONE
        freeze = false
        hold = false
        vSim = 1.0
        log = zeros(0, 3)
        nextCmd = uint8(0)
        lastKey = [0 0 0]
        lastReq = false
    end

    methods
        function F = gnc_fsm_driver(P, plan)
            F.P = P;
            F.plan = plan;
            F.reset();
        end

        function reset(F)
            clear gnc_missionFSM
            P = F.P;
            F.nav = P.ic.Bus_NavCond;  F.nav.nav_ok = true;  F.nav.pose = [0; 0; 0];  F.nav.sigma_pos = 0.015;
            F.nav.gnss_fix = uint8(4);
            F.mcu = P.ic.Bus_McuStatus;  F.mcu.rc_mode_sw = uint8(2);  F.mcu.rc_link_ok = true;
            F.mcu.drive_ready = true;  F.mcu.valid = true;  F.mcu.implement_ready = true;  F.mcu.power_ready = true;
            F.bms = P.ic.Bus_BmsStatus;  F.bms.SOC = single(0.8);  F.bms.valid = true;
            F.mcmd = P.ic.Bus_MissionCmd;
            F.obs = P.ic.Bus_ObsFlags;  F.obs.d_obs = inf;  F.obs.scan_valid = true;
            F.pe = P.ic.Bus_PathError;  F.pe.valid = true;
            F.rp = P.ic.Bus_ReplanResult;
            F.teb = [0; 12; 0; 1];
            F.freeze = false;  F.hold = false;
            F.log = zeros(0, 3);
            F.nextCmd = uint8(0);
            F.lastKey = [0 0 0];
            F.lastReq = false;
            F.step();                                % initial IDLE output
        end

        function set(F, path, value)
            switch path
                case 'freeze'
                    F.freeze = value;
                case 'teb'
                    F.teb = value;
                case 'scanOk'
                    F.obs.scan_valid = logical(value);
                otherwise
                    p = strsplit(path, '.');
                    s = F.(p{1});
                    s.(p{2}) = cast(value, 'like', s.(p{2}));
                    if numel(value) > 1, s.(p{2}) = value; end
                    F.(p{1}) = s;
            end
        end

        function cmd(F, c)
            F.nextCmd = uint8(c);
        end

        function run(F, n)
            for k = 1:n
                F.step();
            end
        end

        function drive(F, n, stop)
            for k = 1:n
                F.step();
                if nargin >= 3 && stop(F), return; end
            end
        end

        function n = runUntil(F, cond, nmax)
            n = -1;
            for k = 1:nmax
                F.step();
                if cond(F)
                    n = k - 1;
                    return
                end
            end
        end

        function startAuto(F)
            F.cmd(MissionCmd.START);
            F.runUntil(@(f) f.st == GncState.AUTO_MISSION, 10);
        end

        function rpAnswer(F, ok, delay)
            if nargin < 3, delay = 4; end
            F.runUntil(@(f) f.out.replan_req ~= f.lastReq, 400);
            F.lastReq = F.out.replan_req;
            F.run(delay);
            if ok
                F.rp.ok = true;
                F.rp.ver = F.rp.ver + 1;
                F.rp.n = uint16(100);
            else
                F.rp.ok = false;
                F.rp.fail_count = F.rp.fail_count + 1;
            end
            F.step();
        end

        function [ids, M] = collect(F, ids, M)
            L = F.log;
            ids = [ids; L(L(:, 3) > 0, 3)];
            for k = 2:size(L, 1)
                if L(k, 1) ~= L(k - 1, 1)
                    M(L(k - 1, 1) + 1, L(k, 1) + 1) = M(L(k - 1, 1) + 1, L(k, 1) + 1) + 1;
                end
            end
            F.log = L(end, :);
        end

        function step(F)
            m = F.mcmd;
            m.cmd = F.nextCmd;
            F.nextCmd = uint8(0);
            if ~F.freeze
                F.mcu.heartbeat = F.mcu.heartbeat + uint8(1);
                if F.mcu.heartbeat == 255, F.mcu.heartbeat = uint8(0); end
            end
            f = gnc_missionFSM(F.nav, F.mcu, F.bms, m, F.obs, F.pe, F.plan, F.rp, F.teb, F.socRth, F.P.gnc.fsm);
            F.out = f;
            F.st = GncState(f.state);
            F.sub = AutoSub(f.sub);
            F.log(end + 1, :) = [double(f.state) double(f.sub) double(f.trans_id)];
            % path-error emulation for the next step (window of this step's outputs)
            key = [double(f.seg_idx) double(f.path_src) double(F.rp.ver)];
            if any(key ~= F.lastKey)
                F.pe.s = 0;
                F.lastKey = key;
            elseif f.trk_sel ~= 0 && f.cmd_valid && ~F.hold
                F.pe.s = F.pe.s + F.vSim*F.P.gnc.Ts;
            end
            if f.path_src == 1
                L = 8;
            else
                L = double(F.plan.seg(max(1, f.seg_idx), 3));
            end
            F.pe.s_rem = L - F.pe.s;
        end
    end
end
