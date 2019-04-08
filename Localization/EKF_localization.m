function [varargout] = EKF_sim_localization(varargin)

if nargin == 0
 
    delt = 0.1;
    iterator = 1000;
    
    alpha = [0.01, 0.001, 0.001, 0.01];
    SigmaQ = diag([0.1 toRadian(20)]).^2; % Sensor noise
    
    mut_1 = [0; 0; 0];
    Sigmat_1 = eye(3);
    
    m = [1, 2; 2, 2; 3, 3];
    
    theta = mut_1(3);
    
    
    
    
    
  
else if nargin == 5 && nargout == 2
        
        

        
    end
end

function [mut, sigmat] = EKF_localization(mut_1, Sigmat_1, ut, zt, m)

theta = 

