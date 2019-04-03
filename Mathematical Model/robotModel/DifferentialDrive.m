classdef DifferentialDrive < handle
    % DIFFERENTIALDRIVE Differential Drive robot utilities
    %
    % For more information, see <a href="matlab:edit mrsDocDifferentialDrive">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;  % Wheel radius [m]
        wheelBase = 0.5;    % Wheelbase [m]
        odometry = [];           % speed of left and right wheel
        groundTruth = [];    % groundtruth of the robot
        preState = [];
        preSigma = {};
        estimatorEKF = [];      % estimator of the robot
        estimatorDR = [0; 0; 0;];
        observation = {};
        controlNoise = [0.01, 0.001, 0.001, 0.01]; % control noise parameter
        sensorNoise = [0.0001, 0.0001];      % noise of the sensor
        sigma = {};
        ID = 0;
    end
    
    methods
        function obj = DifferentialDrive(wheelRadius,wheelBase)
            % DIFFERENTIALDRIVE Construct an instance of this class
            obj.wheelRadius = wheelRadius;
            obj.wheelBase = wheelBase;
        end
        
        function odometry = forwardKinematics(obj, wL, wR)
            % Calculates linear and angular velocity from wheel speeds
            v = 0.5*obj.wheelRadius*(wL+wR);
            w = (wR-wL)*obj.wheelRadius/obj.wheelBase;
            odometry = [v; w];
        end
        
        function [wL,wR] = inverseKinematics(obj,v,w)
            % Calculates wheel speeds from linear and angular velocity
            wL = (v - w*obj.wheelBase/2)/obj.wheelRadius;
            wR = (v + w*obj.wheelBase/2)/obj.wheelRadius;
            obj.odometry(1) = wL;
            obj.odometry(2) = wL;
        end
        
        function initialRobot(obj, number, initPose, sampleTime, odometryLR, ID)
            global model; 
            obj.odometry = zeros(2, number);
            obj.groundTruth = zeros(3, number);
            obj.groundTruth(:, 1) = initPose;
            obj.estimatorDR(:, 1) = initPose;
            obj.estimatorEKF(:, 1) = initPose;
            obj.sigma{1} = 0.001*eye(3);
            obj.ID = ID;
            if (model == 1)
                for i = 1:number
                    if  i < number/2
                        obj.odometry(:,i) = forwardKinematics(obj, odometryLR(1), odometryLR(2));               
                    else
                        obj.odometry(:,i) = forwardKinematics(obj, odometryLR(2), odometryLR(1));
                    end
                    if (i + 1 <= number)
                        obj.groundTruth(:,i+1) = calculateGroundTruth(obj.groundTruth(:,i), obj.odometry(:,i), sampleTime);
                        obj.estimatorDR(:,i+1) = calculateDR(obj.estimatorDR(:,i), obj.odometry(:,i), sampleTime);
                    end
                end        
            end
            
            
        end
        
        
        
    end
end

