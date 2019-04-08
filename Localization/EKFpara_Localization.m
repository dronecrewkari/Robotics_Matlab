function [varargout] = EKFpara_Localization(varargin)

global delt iterator alpha SigmaQ

if nargin == 0
    
    clear; close all; clc;
    
    delt = 0.1;
    iterator = 1000;
    
    alpha = [0.01, 0.001, 0.001, 0.01];
    SigmaQ = diag([0.1 ToRadian(20)]).^2; % Sensor noise
    
    mut_1 = [0; 0; 0];
    Sigmat_1 = eye(3);
    
    m = [1, 2; 2, 2; 3, 3];
    
    theta = mut_1(3);
    
    
    
    
    
  
elseif nargin == 5 && nargout == 2
        
        

       
end

% function of ekf localization
function [mut, sigmat] = EKF_localization(mut_1, Sigmat_1, ut, zt, m, sampletime)

theta = mut_1(3);
vt = ut(1);
omegat = ut(2);
deltat = sampletime;

%% prediction

% linear move and non-linear move
if omegat ~= 0

    Gt = [1, 0, -vt/omegat*cos(theta)+vt/omegat*cos(theta+omegat*deltat);
        0, 1, -vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat);
        0, 0, 1];
    Vt = [(-sin(theta) + sin(theta+omegat*deltat))/omegat,vt*(sin(theta) - sin(theta+omegat*deltat))/omegat^2 + vt*cos(theta+omegat*deltat)*deltat/omegat;
        (cos(theta) - cos(theta + omegat*deltat))/omegat,-vt*(cos(theta) - cos(theta+omegat*deltat))/omegat^2 + vt*sin(theta+omegat*deltat)*deltat/omegat;
        0,deltat];
    Mt = [alpha(1)*vt^2 + alpha(2)*omegat^2, 0;
        0, alpha(3)*vt^2 + alpha(4)*omegat^2];

    mu_t = mut_1 + [(-vt/omegat*sin(theta) + vt/omegat*sin(theta + omegat*deltat));
                     vt/omegat*cos(theta) - vt/omegat*cos(theta + omegat*deltat);
                     wrapToPi(omegat*deltat)];

    Sigma_t = Gt*Sigmat_1*Gt' + Vt*Mt*Vt';

elseif omegat == 0
    Gt = [1, 0, -deltat*vt*sin(theta);
          0, 1, deltat*vt*cos(theta);
          0, 0, 1];
    Vt = [deltat*cos(theta),0;
          deltat*sin(theta),0;
          0,0];
    Mt = [alpha(1)*vt^2 + alpha(2)*omegat^2, 0;
          0, alpha(3)*vt^2 + alpha(4)*omegat^2];
    mu_t = mut_1 + [vt*deltat*cos(theta);
                    vt*deltat*sin(theta);0];
    Sigma_t = Gt*Sigmat_1*Gt' + Vt*Mt*Vt';    
end

%% correction 


    































