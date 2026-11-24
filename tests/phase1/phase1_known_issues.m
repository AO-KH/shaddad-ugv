function K = phase1_known_issues()
%PHASE1_KNOWN_ISSUES  Known, accepted Phase 1 deviations: reported as KNOWN instead of FAIL.
%
%   K: struct array with fields
%     case    test-case id (phase1_cases)
%     check   criterion id (scenario), check id (drive) or test procedure name (unit, substring match)
%     note    why it fails and what would fix it
%   A KNOWN result keeps its measured value in the report. Remove an entry when the issue is fixed, so a
%   regression shows as FAIL again.

K = struct('case', {}, 'check', {}, 'note', {});
% vehicle measurements of the former 470 kg robot (S2 tilt / pitch / ramp speed, S6 / S8b decelerations) were
% removed on 2026-10-27 with the change to the real robot (docs/hardware/real_robot_components.md); re-add only
% after a run of the real-robot model shows the deviation. Re-added 2026-10-27 after the Full suite on the real
% robot (user decision; analysis in docs/report NASL_Report_Shaddad_RealRobot_MBD section 9.3). Values of the
% four-motor model (D64) from the Full suite of 2026-10-27 20:43:
K(end+1) = k('S2', 'S2.v_mean', ['Mean ground speed about 0.725 vs >= 0.75 m/s on the 20 deg ramp at 0.8 m/s target, ' ...
    'with no implement (226 kg) since CR-001 (2026-11-16; 0.724 with the full-tank sprayer, 316 kg, before): about 9.7 % ' ...
    'track slip at about 0.44 traction demand (tan 20 deg plus rolling resistance, the same for any mass) on farm soil ' ...
    'with the 0.75 m track (estimated track size; review after the slope test).']);
K(end+1) = k('S6a', 'S6a.decel', ['Peak 100 ms deceleration 1.100 vs 1.05 m/s^2 for about 0.2 s at the start of the 1.0 m/s^2 ' ...
    'controlled stop: the robot coasts at about 1.05 m/s^2 on farm soil, so the speed-loop catch-up adds the peak.']);
K(end+1) = k('S6b', 'S6b.decel', 'Peak 100 ms deceleration 1.099 vs 1.05 m/s^2 in the controlled stop, as S6a.decel.');
% unit tests with stale expectations (docs/STATUS.md 4.1)
K(end+1) = k('U_mcu_platform', 'testFilesAndBuildOutputs', 'Stale test: expects the T08 stub units that T09 replaced.');
K(end+1) = k('U_mcu_platform', 'testModelStructure', 'Stale test: expects the T08 stub units that T09 replaced.');
K(end+1) = k('U_mcu_platform', 'testTelemetryStatusFrames', 'Stale test: expects the T08 stub units that T09 replaced.');
K(end+1) = k('U_mcu_platform', 'testD37_Latency', 'Test throws an arrayfun error; the same latency is checked by S1.lat_tx_drv.');
end

function e = k(c, chk, note)
e = struct('case', c, 'check', chk, 'note', note);
end
