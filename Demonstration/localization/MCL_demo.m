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
        delt = 0.1;   %samletime
        iterator = ceil((end_time - start_time)/delt);
        interval = 5; %sample interval
        
        % setup of the parameter of the robot
        robot = linearAngular(1, [0, 0, 0], [0; 0], eye(3), [0.03, 0.01, 0.01, 0.03, 0, 0], [0.03, 0.3, 0]); %robot model
        alpha_noise = robot.controlParameter; %control noise parameter
        SigmaQ_eigenvalue = robot.sensorNoise; % Sensor noise
        numParticle = 200;
        
        
        % setup of the initial state of the robot
        pre_particle = zeros(3, numParticle) + robot.groundTruth(:, 1);
        %pre_dead = zeros(3, numParticle) + robot.groundTruth(:, 1);
        ground_1 = robot.groundTruth(:, 1);
        DR_1 = robot.groundTruth(:, 1);

        m = [10, 2.5, 1; 2.5, 3, 2; 3.4, 4, 3];  % object matrix 
        
    elseif nargin == 5 && nargout == 2
        
       
    end
    
    for i = 1 : iterator    
            plot(m(1, :), m(2, :), 'kh', 'MarkerSize', 7);
            time = time + i * delt; 
            ut = calOdom(1, 5, time); 
            
            robot.odometry(:, i + 1) = ut;
            ground = calGround(ground_1, ut, alpha_noise, delt);
            zt = calObservation(ground, m, SigmaQ_eigenvalue);
            
            robot.estimatorDR(:, i + 1) = DeadReckoning(DR_1, ut, delt);
            robot.groundTruth(:, i + 1) = ground;
            
            [particleState, particleWeight] = MCL(pre_particle, ut, zt, m, alpha_noise, SigmaQ_eigenvalue, delt);
            %[particleDead] = MCL_dead(pre_dead, ut, alpha_noise, delt);
            
            robot.estimatorPF(:, i + 1) = particleState * particleWeight';
            
            
            DR_1 = robot.estimatorDR(:, i + 1);
            ground_1 = ground;
            %particleState(:, 1) = ground;
            pre_particle = particleState;
            
            %pre_dead = particleDead;
            
            figure(1);
            %set(gcf,'outerposition',get(0,'screensize'));
            t = 1 : interval : i+1;
            if rem(i, interval) == 0
               hold off;
               arrow=0.5;
               for ip = 1:numParticle
                  quiver(particleState(1, ip), particleState(2,ip), arrow*cos(particleState(3, ip)), arrow*sin(particleState(3,ip)), 'ok'); hold on;
                   %quiver(particleDead(1, ip), particleDead(2,ip), arrow*cos(particleDead(3, ip)), arrow*sin(particleDead(3,ip)), 'ok'); hold on;
               end
               plot(robot.groundTruth(1, t), robot.groundTruth(2, t), '-g', 'linewidth', 2); hold on;
               plot(robot.estimatorPF(1, t), robot.estimatorPF(2, t), 'xr', 'linewidth', 2); hold on;
               plot(robot.estimatorDR(1, t), robot.estimatorDR(2, t), '--b', 'linewidth', 2); hold on;
               axis equal;
               drawnow;    
            end
            
             
    end
end
    
    
    
    
        
        
        
        
        
