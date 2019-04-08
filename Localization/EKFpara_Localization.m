
function [varargout] = EKFpara_Localization(varargin)
    clear global; close all; clc;
    global delt iterator alpha_noise SigmaQ;
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

        m = [1, 2, 1; 2, 2, 2; 3, 3, 3];
        
        for i = 1 : iterator          
            time = time + i * delt; 
            ut = doControl(time);    
            robot.odometry(:, i + 1) = ut;
            ground = CalGround(ground_1, ut, delt);
            zt = CalObservation(ground, m);
            
            robot.estimatorDR(:, i + 1) = DeadReckoning(DR_1, ut, delt);
            robot.groundTruth(:, i + 1) = ground;
            [robot.estimatorEKF(:, i + 1), robot.sigmaEKF{i + 1}] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, delt);
            
            DR_1 = robot.estimatorDR(:, i + 1);
            ground_1 = ground;
            mut_1 = robot.estimatorEKF(:, i + 1);
            Sigmat_1 = robot.sigmaEKF{i + 1};
        end
        
        draw_process(robot);
        
        
        
        
        
        
    elseif nargin == 5 && nargout == 2




    end
end

% function of ekf localization
function [mut, sigmat] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, sampletime)

global alpha_noise SigmaQ;

theta = mut_1(3);
vt = ut(1);
omegat = ut(2);
deltat = sampletime;
mx = m(:, 1);
my = m(:, 2);

%% prediction

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

%% correction 

if ~isempty(zt)
    
    zn = numel(zt(:,1));   % number of the objects which is observated by the camera
    
    for n = 1:zn     
        N = zt(n,3);  % id of the observed object      
        q = (mx(N) - mu_t(1))^2 + (my(N) - mu_t(2))^2;
        zt_Lambda = [sqrt(q);
                     atan2(my(N) - mu_t(2), mx(N) - mu_t(1)) - mu_t(3)];
        zt_Lambda(2) = wrapToPi(zt_Lambda(2));
        
        Ht=[-(mx(N)-mu_t(1))/sqrt(q), -(my(N)-mu_t(2))/sqrt(q), 0;
            (my(N)-mu_t(2))/q, -(mx(N)-mu_t(1))/q, -1];
%       Hm=[(mx(N)-mu_t(1))/sqrt(q),(my(N)-mu_t(2))/sqrt(q),0;-(my(N)-mu_t(2))/q,(mx(N)-mu_t(1))/q,0];
        St = Ht*Sigma_t*(Ht)' + SigmaQ;
        %St=Ht*Sigma_t*(Ht)'+Qt+Hm*Sigma_m(:, 1+(N-1)*3 :3+(N-1)*3)*(Hm)';
        Kt = Sigma_t*(Ht)'*inv(St);
        mu_t = mu_t + Kt*(zt(n,1:2)' - zt_Lambda);
        mu_t(3) = wrapToPi(mu_t(3));
        Sigma_t = (eye(3) - Kt*Ht)*Sigma_t;
%         disp(zt(n,1:2)' - zt_Lambda);
    end
end

mut = mu_t;
sigmat = Sigma_t;

end

function u = doControl(time)
%Calc Input Parameter

T = 10; % [sec]
 
% [V yawrate]
V = 1.0; % [m/s]
omega = 5; % [deg/s]
 
u =[ V*(1-exp(-time/T)) ToRadian(omega)*(1-exp(-time/T))]';
end

function zt = CalObservation(ground, object)
     global SigmaQ;
     n = numel(object(:,1));
     q = (ground(1) - object(:, 1)).^2 + (ground(2) - object(:, 2)).^2;
     zt = [sqrt(q), wrapToPi(atan2(object(:, 2) - ground(2), object(:, 1) - ground(1)) - ground(3)), object(:, 3)] + [rand(3, 2) .* SigmaQ, zeros(n, 1)] ;
     
end

function groundTruth = CalGround(mut_pre, ut, sampletime)
    global alpha_noise;
    vt = ut(1) + (alpha_noise(1)*ut(1)^2 + alpha_noise(2)*ut(2)^2) * rand(1);
    omegat = ut(2) + (alpha_noise(3)*ut(1)^2 + alpha_noise(4)*ut(2)^2) * rand(1);
    theta = wrapToPi(mut_pre(3));
      
    if omegat~=0
        groundTruth = mut_pre + [-vt/omegat*sin(theta) + vt/omegat*sin(theta+omegat*sampletime);
                                 vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*sampletime);
                                 omegat*sampletime];
    else
        groundTruth = mut_pre + [vt*sampletime*cos(theta); 
                                 vt*sampletime*sin(theta);
                                 0];
    end
        groundTruth(3) = wrapToPi(groundTruth(3));
end






    































