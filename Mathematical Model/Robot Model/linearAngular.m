classdef linearAngular < handle
    properties
        odometry = [];          % translational and rotational velocity
        groundTruth = [];       % groundtruth of the robot
        estimatorEKF = [];      % estimator of the state by EKF
        estimatorDR = [];       % estimator of the state by dead reckoning
        observation = {};       % observation from sensor
        controlParameter = [0.3, 0.01, 0.01, 0.3];  % control noise parameter alpha
        sensorNoise = [0.03, 0.03];      % noise of the sensor
        sigmaEKF = {};          % covariance of the state
        ID = 0;                 % identification of robot
    end
    
    methods
        function obj = linearAngular(id, initialGround, initialOdometry, initialSigma, varargin)
            % Initiation of robot with velocity model
            obj.ID = id;
            obj.groundTruth(:, 1) = initialGround;
            obj.odometry(:, 1) = initialOdometry;
            obj.estimatorEKF(:, 1) = initialGround;
            obj.estimatorDR(:, 1) = initialGround;
            obj.sigmaEKF{1} = initialSigma; %0.001*eye(3);
            if (nargin == 6)
                obj.controlParameter = varargin(1);
                obj.sensorNoise = varargin(2);
            end
        end
    end
end