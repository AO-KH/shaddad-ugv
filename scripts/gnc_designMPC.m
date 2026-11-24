function mpcobj = gnc_designMPC(P, varargin)
%GNC_DESIGNMPC  Build the row-tracking Adaptive MPC object mpcRowObj from P.gnc.mpc (owner: gnc).
%
%   gnc_designMPC(P)                  create mpcRowObj in the base workspace (the only base write)
%   obj = gnc_designMPC(P)            ... and return it
%   obj = gnc_designMPC(P, 'Assign', false)   return only (no base write; tests / SimulationInput)
%
%   Prediction model (docs/specs/gnc.md 2.3, revision B D41): Frenet error model with states
%   x = [e_y e_psi r u(k-1) u(k-2)] and inputs u = [r_cmd (MV); kappa_ref (MD); d; w_y; w_psi (UD)],
%   y = [e_y; e_psi], discretised at P.gnc.mpc.Ts by subsystems/gnc/gnc_mpcPlantUpdate (the same
%   function that feeds the Adaptive MPC 'model' inport every step) at the design speed
%   P.gnc.mpc.v_design, with P.gnc.mpc.n_delay = 2 input-delay samples (GNC -> ESC + MCU AUTO shaper).
%
%   Settings (spec 6, F8): p 40, m 8, |r_cmd| <= u_max, |du| <= du_max per step, OV weights W_ov = [1 0.1]
%   (small e_psi weight: the drift-compensating crab angle must not be penalised), MV-rate weight W_dmv,
%   OV scale factors scale_ov, measurement noise model diag(noise) (scaled units), no output-disturbance
%   integrators (an e_y output integrator is unobservable together with the drift random walk; one on
%   e_psi would fight the crab angle). Input-disturbance model (setindist), revision B tuning with the
%   2-sample dead time (T14, offline mpcmoveAdaptive vs a tau_r 0.30 s plant, RTK noise):
%     d     random walk, gain ud_gain (0.04)  -> offset-free e_y under side-slope drift
%     w_y   white, gain w_ey (0.7)            -> an e_y step (row entry) is a state jump, not drift
%     w_psi white, gain w_epsi (0.01)
%   With the spec's single random walk (gain 0.02) a 0.3 m entry offset overshot by 0.2-0.27 m; with
%   the jump channels: settle (|e_y| < 5 cm) 1.65 s, overshoot 3.0 cm, drift 0.03 m/s mean 3.3 mm.
%   The object is placed in the base workspace because the Adaptive MPC mask parameter 'mpcobj' needs
%   a variable name (spec risk R8a).

ip = inputParser;
ip.addParameter('Assign', true);
ip.parse(varargin{:});
prm = P.gnc.mpc;
assert(prm.nx == 3 + prm.n_delay && prm.n_delay == 2 && prm.nu == 5, ...
    'gnc_designMPC: Bus_MpcPlant is sized for n_delay = 2 and 5 model inputs');

mdl = gnc_mpcPlantUpdate(prm.v_design, prm);
plant = ss(mdl.Plant.A, mdl.Plant.B, mdl.Plant.C, mdl.Plant.D, prm.Ts);
plant.InputName  = {'r_cmd', 'kappa_ref', 'drift', 'w_ey', 'w_epsi'};
plant.OutputName = {'e_y', 'e_psi'};
plant = setmpcsignals(plant, 'MV', 1, 'MD', 2, 'UD', [3 4 5], 'MO', [1 2]);

old = mpcverbosity('off');
cleanup = onCleanup(@() mpcverbosity(old));
obj = mpc(plant, prm.Ts, prm.p, prm.m);
obj.MV(1).Min = prm.u_min;
obj.MV(1).Max = prm.u_max;
obj.MV(1).RateMin = -prm.du_max;
obj.MV(1).RateMax = prm.du_max;
obj.OV(2).Min = -prm.epsi_max;                                % soft approach-angle bound (default ECR 1)
obj.OV(2).Max = prm.epsi_max;
obj.OV(1).ScaleFactor = prm.scale_ov(1);
obj.OV(2).ScaleFactor = prm.scale_ov(2);
obj.Weights.OutputVariables = prm.W_ov;
obj.Weights.ManipulatedVariables = 0;
obj.Weights.ManipulatedVariablesRate = prm.W_dmv;
% input-disturbance model: one random-walk state (drift) + two white jump channels
setindist(obj, 'model', ss(1, [1 0 0], [prm.ud_gain; 0; 0], ...
    [0 0 0; 0 prm.w_ey 0; 0 0 prm.w_epsi], prm.Ts));
setoutdist(obj, 'model', tf(zeros(2, 1)));                    % no output-disturbance integrators
obj.Model.Noise = ss(diag(prm.noise));
setEstimator(obj, 'default');
mpcstate(obj);                                                % fails here if the estimator does not converge

if ip.Results.Assign
    assignin('base', 'mpcRowObj', obj);
end
if nargout > 0, mpcobj = obj; end
end
