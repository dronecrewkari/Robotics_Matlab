classdef linearAngular < handle
    properties
        odometry = [];           % translational and rotational velocity
        groundTruth = [];    % groundtruth of the robot
        estimatorEKF = [];      % estimator of the robot
        estimatorDR = [];
        observation = {};
        controlParameter = [0.1, 0.01, 0.01, 0.1]; % control noise parameter
        sensorNoise = [0.01, 0.01];      % noise of the sensor
        sigmaEKF = {};
        ID = 0;
    end
    
    methods  
        function obj = linearAngular(id, initialGround, initialOdometry, initialSigma, varargin)
            % DIFFERENTIALDRIVE Construct an instance of this class
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