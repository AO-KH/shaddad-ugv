function [in, crit] = scn_L1_live_drive(P, varargin)
%SCN_L1_LIVE_DRIVE  Live RC drive: real transmitter (or dashboard / demo plan), 3-D view, real-time pacing.
%
%   [in, crit] = scn_L1_live_drive(P, ...)        (normally called through phase1_live_drive)
%   'Source'     'joystick' (default: USB controller, VC_OPSRC 3) | 'dashboard'
%                (on-screen sliders, VC_OPSRC 2) | 'demo' (scripted plan, VC_OPSRC 1: no hardware needed)
%   'Controller' 'ps5' (PS5 DualSense layout, P.scen.joy.profile 2) | 'transmitter' (RC radio in USB
%                joystick mode, profile 1) | 'auto' / [] (keep P.scen.joy.profile)
%   'Fidelity'   'fast' (default: the endurance set P.sim.endurance - lumped pack, ideal drive, kinematic LoFi
%                chassis - so the run comes close to real time) | 'full' (every high-fidelity variant)
%   'View'       '3d' (default, VC_VIZ 3) | 'birdseye' (VC_VIZ 1) | 'none'
%   'Camera'     1 chase (default) | 2 side | 3 top | 4 free (mouse)
%   'ViewRate'   3-D view updates per second (default 1/P.viz.Ts3d = 10 Hz); lower = faster simulation
%   'Pacing'     true (default): simulation pacing locks simulated time to the wall clock
%   'PacingRate' 1 (default) = real time
%   'Duration'   simulated seconds (default 3600; stop earlier with the Stop button or Esc in the 3-D view)
%   'Log'        true (default): log at 10 Hz for the summary and the replay; false: no logging (faster)
%   'Plan'       demo plan [t v w] (as phase1_test_drive), default: the demo drive
%   'World', 'Implement'  as phase1_test_drive (default 1 flat test field, 0 no implement)

ip = inputParser;
ip.addParameter('Source', 'joystick');
ip.addParameter('Controller', []);
ip.addParameter('Fidelity', 'fast');
ip.addParameter('View', '3d');
ip.addParameter('Camera', 1);
ip.addParameter('ViewRate', []);
ip.addParameter('Pacing', true);
ip.addParameter('PacingRate', 1);
ip.addParameter('Duration', 3600);
ip.addParameter('Log', true);
ip.addParameter('Plan', []);
ip.addParameter('World', 1);
ip.addParameter('Implement', 0);
ip.addParameter('StopTime', []);
ip.parse(varargin{:});
o = ip.Results;
src = lower(char(o.Source));
T = o.Duration;
if ~isempty(o.StopTime), T = o.StopTime; end

vc = struct('VC_PHASE', 1, 'VC_IMPL', o.Implement);
switch lower(char(o.View))
    case '3d', vc.VC_VIZ = 3;
    case 'birdseye', vc.VC_VIZ = 1;
    case 'none', vc.VC_VIZ = 0;
    otherwise, error('scn_L1:view', 'View must be 3d, birdseye or none');
end
switch src
    case 'demo'
        args = {'World', o.World, 'Implement', o.Implement};
        if ~isempty(o.Plan), args = [args {'Plan', o.Plan}]; end
        if ~isempty(o.StopTime), args = [args {'StopTime', o.StopTime}]; end
        [in, crit] = scn_D1_test_drive(P, args{:});      % scripted transmitter, same as phase1_test_drive
        P = in.Variables(strcmp({in.Variables.Name}, 'P')).Value;
        vc.VC_OPSRC = 1;
        T = str2double(in.getModelParameter('StopTime'));
    case {'joystick', 'dashboard'}
        P = shaddad_params('World', o.World, 'Implement', o.Implement, 'Phase', 1, 'Assign', false);
        P = scn_integration_overrides(P);
        crit = [];
        if strcmp(src, 'joystick'), vc.VC_OPSRC = 3; else, vc.VC_OPSRC = 2; end
        in = scn_base_input(P, 'L1', 'StopTime', T, 'Input', struct(), 'VC', vc);
    otherwise
        error('scn_L1:source', 'Source must be joystick, dashboard or demo');
end
switch lower(char(o.Fidelity))
    case 'fast'
        E = P.sim.endurance;
        for f = {'VC_PACK', 'VC_DRIVE_FID', 'VC_STRUCT_FID'}
            vc.(f{1}) = E.(f{1});
        end
    case 'full'
    otherwise
        error('scn_L1:fidelity', 'Fidelity must be fast or full');
end
P.viz.camera = o.Camera;
switch lower(char(o.Controller))
    case 'ps5', P.scen.joy.profile = 2;
    case 'transmitter', P.scen.joy.profile = 1;
end
if ~isempty(o.ViewRate), P.viz.Ts3d = 1/o.ViewRate; end
in = in.setVariable('P', P);
fn = fieldnames(vc);
for k = 1:numel(fn)
    in = in.setVariable(fn{k}, vc.(fn{k}));
end
if o.Pacing
    in = in.setModelParameter('EnablePacing', 'on', 'PacingRate', o.PacingRate);
end
% live runs log at the 10 Hz mission rate (light), demo runs as the test drive; 'Log' false: no logging
if ~o.Log
    in = in.setModelParameter('SignalLogging', 'off');
elseif ~strcmp(src, 'demo')
    in = scn_logging('apply', in, P, max(T, P.log.to_file_threshold + 1));
end
end
