function [varargout] = MCL_demo(varargin)
clear global; close all; clc;
global delt iterator;
    
    %% initialise
    if nargin == 0
        disp('Monte Carlo localization (MCL) demonstration start!!');
        % setup of the simulation
        start_time = 0;
        end_time = 60;
        time = 0;
        delt = 0.1;   %sampletime
        iterator = ceil((end_time - start_time)/delt);
        interval = 5; %sample interval
        
        % setting of the parameter of the robot
        robot = linearAngular(1, [0, 0, 0], [0; 0], eye(3), [0.3, 0.1, 0.1, 0.3], [0.03, 0.03]); %robot model
        alpha_noise = robot.controlParameter; %control noise parameter
        SigmaQ_eigenvalue = robot.sensorNoise; % Sensor noise
            
        mut_1 = robot.groundTruth(:, 1);
        ground_1 = robot.groundTruth(:, 1);
        DR_1 = robot.groundTruth(:, 1);
        Sigmat_1 = robot.sigmaEKF{1};

        m = [1, 2.5, 1; 2.5, 3, 2; 3.4, 4, 3];  % object matrix 
        
    elseif nargin == 5 && nargout == 2
        
        robot = varargin{1};
        
        mut_1 = robot.groundTruth(:, 1);
        ground_1 = robot.groundTruth(:, 1);
        DR_1 = robot.groundTruth(:, 1);
        Sigmat_1 = robot.sigmaEKF{1}; 
        
        alpha_noise = robot.controlParameter; %control noise parameter
        SigmaQ_eigenvalue = robot.sensorNoise; % Sensor noise  
    end
    
    
    
        
        
        
        
        
