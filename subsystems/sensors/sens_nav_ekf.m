classdef sens_nav_ekf < matlab.System
%SENS_NAV_EKF  Navigation EKF as a MATLAB System block (risk R6 fallback, build task T12).
%
%   The library block cstblocks_estimation/Extended Kalman Filter keeps x and P in Data Store Memory blocks
%   that a Resettable Subsystem does not re-initialise (measured in the T12 first-step prototype: the
%   Resettable Subsystem reset fires - a counter inside restarts at the trigger - but the block's
%   'DataStoreMemory - x' / '- P' keep their values), so T-NAV-09 cannot pass with the block. As the build
%   plan prescribes, the filter is this System object wrapping the toolbox extendedKalmanFilter object
%   (code generation supported) with the same functions and settings as spec 4.3.1 / D63:
%     predict:  sens_ekf_f / sens_ekf_dfdx, additive time-varying process noise Q (input)
%     correct:  five channels, each only when its enable input is true (sens_nav_gate), additive
%               time-varying R (inputs), residual wrapping on channels 3 (heading) and 5 (LiDAR pose)
%               1 sens_ekf_h_gnsspos (u1)  2 sens_ekf_h_gnssvel (u2)  3 sens_ekf_h_heading (u3)
%               4 sens_ekf_h_odom (u4)     5 sens_ekf_h_lidarpose (u5)
%     current estimator: outputs x(k|k), P(k|k) (UseCurrentEstimator 'on'), then predicts to k+1
%   One extendedKalmanFilter object per measurement model (the measurement function of an object is
%   fixed for code generation); the state and covariance are carried between them. resetImpl restores
%   InitialState / InitialStateCovariance, so the block inside a Resettable Subsystem resets on navReset.
%
%   D49 slip hold (input hold = GNSS and scan matching both unavailable for > t_denied): besides the process model
%   (u carries tau_s_denied and the slip estimate held at denial onset in u(9:10), process noise Qd_denied), the
%   slip states are constrained to that held value after the corrections and the prediction, with their
%   cross-covariances removed (they are then treated as known parameters): odometry corrections could otherwise
%   move them through the v - slip covariance (T12 bench: 0.008-0.012 drift in a 30 s outage without the constraint).
%
%   Inputs (23): u Q  z1 en1 R1 u1  z2 en2 R2 u2  z3 en3 R3 u3  z4 en4 R4 u4  z5 en5 R5 u5  hold
%   Outputs: xhat (11x1), P (11x11)
%#codegen

    properties (Nontunable)
        InitialState = zeros(11, 1)            % x0 (P.sensors.ekf.x0)
        InitialStateCovariance = eye(11)       % P0 (P.sensors.ekf.P0)
    end

    properties (DiscreteState)
        X
        PX
    end

    properties (Access = private)
        F1
        F2
        F3
        F4
        F5
    end

    methods
        function obj = sens_nav_ekf(varargin)
            % sens_nav_ekf('InitialState', x0, 'InitialStateCovariance', P0)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, varargin)
            x0 = obj.InitialState(:);
            P0 = obj.InitialStateCovariance;
            obj.F1 = extendedKalmanFilter(@sens_ekf_f, @sens_ekf_h_gnsspos, x0, ...
                'StateTransitionJacobianFcn', @sens_ekf_dfdx, 'MeasurementJacobianFcn', @sens_ekf_dh_gnsspos, ...
                'StateCovariance', P0, 'ProcessNoise', eye(11), 'MeasurementNoise', eye(3));
            obj.F2 = extendedKalmanFilter(@sens_ekf_f, @sens_ekf_h_gnssvel, x0, ...
                'StateTransitionJacobianFcn', @sens_ekf_dfdx, 'MeasurementJacobianFcn', @sens_ekf_dh_gnssvel, ...
                'StateCovariance', P0, 'ProcessNoise', eye(11), 'MeasurementNoise', eye(2));
            obj.F3 = extendedKalmanFilter(@sens_ekf_f, @sens_ekf_h_heading, x0, ...
                'StateTransitionJacobianFcn', @sens_ekf_dfdx, 'MeasurementJacobianFcn', @sens_ekf_dh_heading, ...
                'StateCovariance', P0, 'ProcessNoise', eye(11), 'MeasurementNoise', 1, ...
                'HasMeasurementWrapping', true);
            obj.F4 = extendedKalmanFilter(@sens_ekf_f, @sens_ekf_h_odom, x0, ...
                'StateTransitionJacobianFcn', @sens_ekf_dfdx, 'MeasurementJacobianFcn', @sens_ekf_dh_odom, ...
                'StateCovariance', P0, 'ProcessNoise', eye(11), 'MeasurementNoise', eye(2));
            obj.F5 = extendedKalmanFilter(@sens_ekf_f, @sens_ekf_h_lidarpose, x0, ...
                'StateTransitionJacobianFcn', @sens_ekf_dfdx, 'MeasurementJacobianFcn', @sens_ekf_dh_lidarpose, ...
                'StateCovariance', P0, 'ProcessNoise', eye(11), 'MeasurementNoise', eye(3), ...
                'HasMeasurementWrapping', true);
        end

        function resetImpl(obj)
            obj.X = obj.InitialState(:);
            obj.PX = obj.InitialStateCovariance;
        end

        function [xhat, Pout] = stepImpl(obj, u, Q, z1, en1, R1, u1, z2, en2, R2, u2, z3, en3, R3, u3, ...
                z4, en4, R4, u4, z5, en5, R5, u5, hold)
            x = obj.X;
            P = obj.PX;
            if en1
                obj.F1.State = x; obj.F1.StateCovariance = P; obj.F1.MeasurementNoise = R1;
                correct(obj.F1, z1, u1);
                x = obj.F1.State; P = obj.F1.StateCovariance;
            end
            if en2
                obj.F2.State = x; obj.F2.StateCovariance = P; obj.F2.MeasurementNoise = R2;
                correct(obj.F2, z2, u2);
                x = obj.F2.State; P = obj.F2.StateCovariance;
            end
            if en3
                obj.F3.State = x; obj.F3.StateCovariance = P; obj.F3.MeasurementNoise = R3;
                correct(obj.F3, z3, u3);
                x = obj.F3.State; P = obj.F3.StateCovariance;
            end
            if en4
                obj.F4.State = x; obj.F4.StateCovariance = P; obj.F4.MeasurementNoise = R4;
                correct(obj.F4, z4, u4);
                x = obj.F4.State; P = obj.F4.StateCovariance;
            end
            if en5
                obj.F5.State = x; obj.F5.StateCovariance = P; obj.F5.MeasurementNoise = R5;
                correct(obj.F5, z5, u5);
                x = obj.F5.State; P = obj.F5.StateCovariance;
            end
            P = (P + P.')/2;
            if hold
                [x, P] = local_hold(x, P, u);
            end
            xhat = x;
            Pout = P;
            % predict to k+1 with the current input and process noise
            obj.F1.State = x; obj.F1.StateCovariance = P; obj.F1.ProcessNoise = Q;
            predict(obj.F1, u);
            xn = obj.F1.State;
            Pn = obj.F1.StateCovariance;
            Pn = (Pn + Pn.')/2;
            if hold
                [xn, Pn] = local_hold(xn, Pn, u);
            end
            obj.X = xn;
            obj.PX = Pn;
        end

        function num = getNumInputsImpl(~)
            num = 23;
        end

        function varargout = getInputNamesImpl(~)
            varargout = {'u', 'Q', 'z1', 'en1', 'R1', 'u1', 'z2', 'en2', 'R2', 'u2', 'z3', 'en3', 'R3', 'u3', ...
                         'z4', 'en4', 'R4', 'u4', 'z5', 'en5', 'R5', 'u5', 'hold'};
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end

        function [n1, n2] = getOutputNamesImpl(~)
            n1 = 'xhat';
            n2 = 'P';
        end

        function [s1, s2] = getOutputSizeImpl(~)
            s1 = [11 1];
            s2 = [11 11];
        end

        function [d1, d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end

        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end

        function [f1, f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end

        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            if strcmp(name, 'X')
                sz = [11 1];
            else
                sz = [11 11];
            end
            dt = 'double';
            cp = false;
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Code generation';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end

function [x, P] = local_hold(x, P, u)
% D49 slip hold: slip states = held prior u(9:10), decoupled from the other states
x(9) = u(9);
x(10) = u(10);
for i = 1:11
    if i ~= 9 && i ~= 10
        P(9, i) = 0;  P(i, 9) = 0;
        P(10, i) = 0;  P(i, 10) = 0;
    end
end
end
