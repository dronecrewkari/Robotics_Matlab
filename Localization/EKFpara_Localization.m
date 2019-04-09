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

function [varargout] = EKF_LocalizationWithdemo(varargin)
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
        SigmaQ = robot.sensorNoise    ; % Sensor noise
        
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

% function of ekf localization
function [mut, sigmat] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, alpha_noise, SigmaQ, sampletime)


theta = mut_1(3);
vt = ut(1);
omegat = ut(2);
deltat = sampletime;
mx = m(:, 1);
my = m(:, 2);

% prediction

% linear move and non-linear move
if omegat ~= 0

    Gt = [1, 0, -vt/omegat*cos(theta)+vt/omegat*cos(theta+omegat*deltat);
        0, 1, -vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat);
        0, 0, 1];
    Vt = [(-sin(theta) + sin(theta+omegat*deltat))/omegat,vt*(sin(theta) - sin(theta+omegat*deltat))/omegat^2 + vt*cos(theta+omegat*deltat)*deltat/omegat;
        (cos(theta) - cos(theta + omegat*deltat))/omegat,-vt*(cos(theta) - cos(theta+omegat*deltat))/omegat^2 + vt*sin(theta+omegat*deltat)*deltat/omegat;
        0,deltat];
    Mt = [alpha_noise(1)*vt^2 + alpha_noise(2)*omegat^2, 0;
        0, alpha_noise(3)*vt^2 + alpha_noise(4)*omegat^2];

    mu_t = mut_1 + [(-vt/omegat*sin(theta) + vt/omegat*sin(theta + omegat*deltat));
                     vt/omegat*cos(theta) - vt/omegat*cos(theta + omegat*deltat);
                     wrapToPi(omegat*deltat)];
    
    mu_t(3) = wrapToPi(mu_t(3));
    Sigma_t = Gt * Sigmat_1* Gt' + Vt * Mt * Vt';

elseif omegat == 0
    Gt = [1, 0, -deltat*vt*sin(theta);
          0, 1, deltat*vt*cos(theta);
          0, 0, 1];
    Vt = [deltat*cos(theta),0;
          deltat*sin(theta),0;
          0,0];
    Mt = [alpha_noise(1)*vt^2 + alpha_noise(2)*omegat^2, 0;
          0, alpha_noise(3)*vt^2 + alpha_noise(4)*omegat^2];
    mu_t = mut_1 + [vt*deltat*cos(theta);
                    vt*deltat*sin(theta);0];
    Sigma_t = Gt*Sigmat_1*Gt' + Vt*Mt*Vt';    
end

% correction 

if ~isempty(zt)
    
    zn = numel(zt(:,1));   % number of the objects which is observated by the camera
    Qt = diag([SigmaQ(1)^2, SigmaQ(2)^2]);
    for n = 1:zn     
        N = zt(n,3);  % id of the observed object      
        q = (mx(N) - mu_t(1))^2 + (my(N) - mu_t(2))^2;
        zt_Lambda = [sqrt(q); atan2(my(N) - mu_t(2), mx(N) - mu_t(1)) - mu_t(3)];
        zt_Lambda(2) = wrapToPi(zt_Lambda(2));
        
        Ht=[-(mx(N)-mu_t(1))/sqrt(q), -(my(N)-mu_t(2))/sqrt(q), 0;(my(N)-mu_t(2))/q, -(mx(N)-mu_t(1))/q, -1];
%       Hm=[(mx(N)-mu_t(1))/sqrt(q),(my(N)-mu_t(2))/sqrt(q),0;-(my(N)-mu_t(2))/q,(mx(N)-mu_t(1))/q,0];
        St = Ht*Sigma_t*(Ht)' + Qt;
        %St=Ht*Sigma_t*(Ht)'+Qt+Hm*Sigma_m(:, 1+(N-1)*3 :3+(N-1)*3)*(Hm)';
        Kt = Sigma_t*(Ht)'*inv(St);
        diff = zt(n,1:2)' - zt_Lambda;
        if diff <= 0.1
            mu_t = mu_t + Kt*diff;
            mu_t(3) = wrapToPi(mu_t(3));
            Sigma_t = (eye(3) - Kt*Ht)*Sigma_t;
        end
%         disp(zt(n,1:2)' - zt_Lambda);
    end
end

mut = mu_t;
sigmat = Sigma_t;

end
















    































