% -------------------------------------------------------------------------
%
% File : EKFpara_Localization.m
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
    if nargin == 0
        
        start_time = 0;
        end_time = 60;
        time = 0;
        delt = 0.1;
        iterator = ceil((end_time - start_time)/delt);
        
        robot = linearAngular(1, [10, 10, 0], [0; 0], eye(3));

        alpha_noise = robot.controlParameter;
        SigmaQ = robot.sensorNoise; % Sensor noise
        
        mu_init = robot.groundTruth;
        Sigma_init = robot.sigmaEKF{1};
        
        mut_1 = mu_init;
        ground_1 = mu_init;
        DR_1 = mu_init;
        Sigmat_1 = Sigma_init;

        m = [1, 2.5, 1; 2.5, 3, 2; 3.4, 4, 3];
        
        for i = 1 : iterator          
            time = time + i * delt; 
            ut = calOdom(time);    
            robot.odometry(:, i + 1) = ut;
            ground = calGround(ground_1, ut, alpha_noise, delt);
            zt = calObservation(ground, m, SigmaQ);
            
            robot.estimatorDR(:, i + 1) = DeadReckoning(DR_1, ut, delt);
            robot.groundTruth(:, i + 1) = ground;
            [robot.estimatorEKF(:, i + 1), robot.sigmaEKF{i + 1}] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, alpha_noise, SigmaQ, delt);
            
            DR_1 = robot.estimatorDR(:, i + 1);
            ground_1 = ground;
            mut_1 = robot.estimatorEKF(:, i + 1);
            Sigmat_1 = robot.sigmaEKF{i + 1};
            
            if rem(i,3)==0
               plot(robot.groundTruth(1,:), robot.groundTruth(2,:), '.g'); hold on;
               plot(robot.estimatorEKF(1,:), robot.estimatorEKF(2,:), '.r'); hold on;
               plot(robot.estimatorDR(1,:), robot.estimatorDR(2,:), '.b'); hold on;
               axis equal;
               ShowErrorEllipse(robot.estimatorEKF(:, i + 1), robot.sigmaEKF{i + 1}, 2.5);
               drawnow;    
            end
        end
              
%         drawGraph(robot);
        
    elseif nargin == 5 && nargout == 2

    end
end


















    






























