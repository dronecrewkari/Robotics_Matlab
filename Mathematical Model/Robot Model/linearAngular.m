classdef linearAngular < handle
    properties
        odometry = [];           % translational and rotational velocity
        groundTruth = [];    % groundtruth of the robot
        estimatorEKF = [];      % estimator of the robot
        estimatorDR = [];
        observation = {};
        controlNoise = [0.01, 0.001, 0.001, 0.01]; % control noise parameter
        sensorNoise = [0.0001, 0.0001];      % noise of the sensor
        sigmaEKF = [];
        ID = 0;
    end
    
    methods  
        function obj = linearAngular(id, initialGround, initialOdometry, initialSigma, controlNoise, sensorNoise)
            % DIFFERENTIALDRIVE Construct an instance of this class
            obj.groundTruth = initialGround;      
            obj.odometry = initialOdometry;
            obj.estimatorEKF = initialGround;
            obj.estimatorDR = initialGround;
            obj.sigmaEKF = initialSigma; %0.001*eye(3);
            obj.controlNoise = sensorNoise;
            obj.sensorNoise = controlNoise;
            obj.ID = id;
        end
    end    
end