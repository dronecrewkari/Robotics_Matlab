%-------------------------------------------------------------------------
%
% File : EKF_Localization_demo.m
%
% Discription : Mobible robot localization sample code with
% Extended Kalman Filter (EKF)
%
% Environment : Matlab
%
% Author : Herry
% -------------------------------------------------------------------------

function [varargout] = EKF_Localization_demo(varargin)
    clear global; close all; clc;
    global delt iterator;
    
    %% initialise
    if nargin == 0
        disp('Extended Kalman Filter (EKF) localization demonstration start!!');
        % setup of the simulation
        start_time = 0;
        end_time = 60;
        time = 0;
        delt = 0.1;   %sampletime
        iterator = ceil((end_time - start_time)/delt);
        interval = 5; %sample interval
        
        % setting of the parameter of the robot
        robot = linearAngular(1, [0, 0, 0], [0; 0], eye(3), [0.3, 0.1, 0.1, 0.3, 0, 0], [0.03, 0.03, 0]); %robot model
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
        
   %% calculation and plot
        tic;
        for i = 1 : iterator    
            plot(m(1, :), m(2, :), 'kh', 'MarkerSize', 7);
            time = time + i * delt; 
            ut = calOdom(1, 5, time); 
            
            robot.odometry(:, i + 1) = ut;
            ground = calGround(ground_1, ut, alpha_noise, delt);
            zt = calObservation(ground, m, SigmaQ_eigenvalue);
            
            robot.estimatorDR(:, i + 1) = DeadReckoning(DR_1, ut, delt);
            robot.groundTruth(:, i + 1) = ground;
            [robot.estimatorEKF(:, i + 1), robot.sigmaEKF{i + 1}] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, alpha_noise, SigmaQ_eigenvalue, delt);
                      
            DR_1 = robot.estimatorDR(:, i + 1);
            ground_1 = ground;
            mut_1 = robot.estimatorEKF(:, i + 1);
            Sigmat_1 = robot.sigmaEKF{i + 1};
            
            figure(1);
            set(gcf,'outerposition',get(0,'screensize'));
            t = 1 : interval : i+1;
            if rem(i, interval)==0
               plot(robot.groundTruth(1, t), robot.groundTruth(2, t), '-g', 'linewidth', 2); hold on;
               plot(robot.estimatorEKF(1, t), robot.estimatorEKF(2, t), 'xr', 'linewidth', 2); hold on;
               plot(robot.estimatorDR(1, t), robot.estimatorDR(2, t), '--b', 'linewidth', 2); hold on;
               axis equal;
               ShowErrorEllipse(robot.estimatorEKF(:, i + 1), robot.sigmaEKF{i + 1}, 1);
               drawnow;    
            end
        end
             
        drawGraphTemplate(robot.groundTruth, robot.estimatorEKF, robot.estimatorDR, 'Ground Truth','EKF Estimator','Dead Reckoning Estimator');
        drawRMSE(robot.groundTruth, robot.estimatorEKF, robot.estimatorDR, 'RMSE of EKF Estimator','RMSE of Dead Reckoning Estimator');
end

















    































