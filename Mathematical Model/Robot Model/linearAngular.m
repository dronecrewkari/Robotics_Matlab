classdef linearAngular < handle
    properties
        odometry = [];           % translational and rotational velocity
        groundTruth = [];    % groundtruth of the robot
        estimatorEKF = [];      % estimator of the robot
        estimatorDR = [];
        observation = {};
        controlNoise = [0.0001, 0.001, 0.001, 0.001]; % control noise parameter
        sensorNoise = [0.0001, 0.0001];      % noise of the sensor
        sigma = [];
        ID = 0;
    end
    
    methods  
        function obj = linearAngular(id, initialGround, initialOdometry, initialSigma)
            % DIFFERENTIALDRIVE Construct an instance of this class
            obj.groundTruth = initialGround;      
            obj.odometry = initialOdometry;
            obj.estimatorEKF = initialGround;
            obj.estimatorDR = initialGround;
            obj.sigma = initialSigma;%0.001*eye(3);
            obj.ID = id;
        end
    end    
end