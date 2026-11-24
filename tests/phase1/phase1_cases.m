function C = phase1_cases(level)
%PHASE1_CASES  Test cases of the Phase 1 (SBUS remote-control) test suite.
%
%   C = phase1_cases('Quick')   everyday check, about 20 min
%   C = phase1_cases('Full')    every Phase 1 scenario, test drives and unit-test class, about 1.5-2 h
%   C = phase1_cases('All')     same as 'Full' (case lookup by id)
%
%   Each case: id, level ('quick' | 'full'), type ('scenario' | 'drive' | 'unit'), title, and
%     scenario: scn      run_scenario id (criteria from scripts/scenarios/scn_*.m)
%     drive:    args     phase1_test_drive options; checks in phase1_check_drive
%     unit:     file     test class under tests/; tags (only these tags, {} = all); exclude (procedure-name
%               substrings that are not run: code generation, model rebuilds, multi-hour cached cases)

if nargin < 1, level = 'Quick'; end
C = struct('id', {}, 'level', {}, 'type', {}, 'title', {}, 'scn', {}, 'args', {}, 'file', {}, 'tags', {}, 'exclude', {});

% ---------------------------------------------------------------- Quick
C(end+1) = sc('S0', 'quick', 'Power-up and safety re-arm');
C(end+1) = sc('S1', 'quick', 'RC (SBUS) driving: speed, steering, stop distance, reverse, latency');
C(end+1) = sc('S6c', 'quick', 'Corrupted SBUS bytes (1 %): frames rejected, robot keeps driving');
C(end+1) = sc('S8a', 'quick', 'Front hardware E-stop, reset and restart');
C(end+1) = ut('U_rc_link', 'quick', 'RC link: transmitter, SBUS encoder, radio, UART timing', 'rc/t_rc_link.m', {}, {});
C(end+1) = ut('U_elec_plant', 'quick', 'Electronics plant: DI/ADC, watchdog, CAN networks', 'electronics/t_elec_plant.m', {}, {});

% ---------------------------------------------------------------- Full only
C(end+1) = sc('S2', 'full', 'Slope: 20 deg ramp without an implement (CR-001), hold and restart');
C(end+1) = sc('S6a', 'full', 'Radio link lost: controlled stop, re-arm');
C(end+1) = sc('S6b', 'full', 'Silent SBUS line: controlled stop, re-arm');
C(end+1) = sc('S6d', 'full', 'Corrupted SBUS bytes (5 %): link loss, controlled stop');
C(end+1) = sc('S8b', 'full', 'Remote E-stop (CH12): stop profile, latched');

C(end+1) = dr('D1', 'Test drive: default demo (arcs, pivot turn, reverse)', {});
C(end+1) = dr('D2', 'Test drive: remote E-stop at 12 s, reset at 17 s, resume', ...
    {'Plan', [0 0 0; 4 1.0 0; 30 0 0], 'EStopAt', 12, 'ResetAt', 17});
C(end+1) = dr('D3', 'Test drive: radio link lost 12-14 s, re-arm, resume', ...
    {'Plan', [0 0 0; 4 1.0 0; 30 0 0], 'RCLoss', [12 14]});
C(end+1) = dr('D4', 'Test drive with the mower mounted', ...
    {'Plan', [0 0 0; 4 0.8 0; 14 0.8 0.3; 24 0 0], 'Implement', 1});
C(end+1) = dr('D5', 'Test drive with the sprayer (full tank, heaviest load)', ...
    {'Plan', [0 0 0; 4 0.8 0; 14 0.8 -0.3; 24 0 0], 'Implement', 3});

noCG = {'odegen'};                            % code generation is checked separately (not a Phase 1 simulation test)
C(end+1) = ut('U_mcu_control', 'full', 'MCU control: mode manager, safety supervisor, vehicle control', 'electronics/t_mcu_control.m', {}, noCG);
C(end+1) = ut('U_mcu_platform', 'full', 'MCU platform: SBUS decoder, RC mapping, CAN codecs, tilt', 'electronics/t_mcu_platform.m', {}, [noCG {'testHwVariantCompile', 'testSystemUpdate'}]);
C(end+1) = ut('U_drive', 'full', 'Traction drive: ESC, motor, gearbox, brake, STO, undervoltage', 'drive/t_drive.m', {}, [noCG {'testT17_speed'}]);
C(end+1) = ut('U_power_plant', 'full', 'Power: precharge, safety chain, LV rails, implement drives', 'power/t_power_plant.m', {}, ...
    {'overload_T06b', 'energy_audit_1h', 'shift_cycle_T20', 'shift_bench_speed'});
C(end+1) = ut('U_pack_bms', 'full', 'Battery pack and BMS (fast tests)', 'power/t_pack_bms.m', {'fast'}, noCG);
C(end+1) = ut('U_structure', 'full', 'Structure: chassis, tracks, soil contact', 'structure/t_structure.m', {'Data', 'Bench'}, {});
C(end+1) = ut('U_world', 'full', 'World, terrain and environment', 'structure/t_world.m', {'World', 'Bench'}, {});

lv = lower(char(level));
switch lv
    case 'quick', C = C(strcmp({C.level}, 'quick'));
    case {'full', 'all'}
    otherwise, error('phase1_cases:level', 'level must be Quick or Full');
end
end

% ------------------------------------------------------------------------------------------------
function c = blank()
c = struct('id', '', 'level', '', 'type', '', 'title', '', 'scn', '', 'args', {{}}, 'file', '', 'tags', {{}}, 'exclude', {{}});
end

function c = sc(id, level, title)
c = blank(); c.id = id; c.level = level; c.type = 'scenario'; c.title = title; c.scn = id;
end

function c = dr(id, title, args)
c = blank(); c.id = id; c.level = 'full'; c.type = 'drive'; c.title = title; c.args = args;
end

function c = ut(id, level, title, file, tags, exclude)
c = blank(); c.id = id; c.level = level; c.type = 'unit'; c.title = title; c.file = file; c.tags = tags; c.exclude = exclude;
end
