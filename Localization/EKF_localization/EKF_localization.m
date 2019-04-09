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