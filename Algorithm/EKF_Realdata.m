function [estimatorT,sigmaT] = EKF_Realdata(robot, detections, estimatorObj, sampleTime, iterator)

%% initial parameter
deltat = sampleTime;   % sample time

theta = robot.estimatorEKF(3, iterator-1); % theta estimated in t-1;
vt = robot.odometry(1, iterator);
omegat = robot.odometry(2, iterator);

Sigmat_1 = robot.sigma{iterator - 1};
estimator_1 = robot.estimatorEKF(1:3, iterator-1);

alpha_1 = robot.controlNoise(1);
alpha_2 = robot.controlNoise(2);
alpha_3 = robot.controlNoise(3);
alpha_4 = robot.controlNoise(4);

sigma_r = robot.sensorNoise(1);
sigma_Phi = robot.sensorNoise(2);

z_t = detections;
mx = estimatorObj(:,1);
my = estimatorObj(:,2);
%Sigma_m = objectsSigama;
%ms=m.ms;

I=eye(3);
%% prediction
if omegat==0
    omegat=0.0001*randn(1);
end
Gt=[1,0,-vt/omegat*cos(theta)+vt/omegat*cos(theta+omegat*deltat);0,1,-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat);0,0,1];
Vt=[(-sin(theta)+sin(theta+omegat*deltat))/omegat,vt*(sin(theta)-sin(theta+omegat*deltat))/omegat^2+vt*cos(theta+omegat*deltat)*deltat/omegat;(cos(theta)-cos(theta+omegat*deltat))/omegat,-vt*(cos(theta)-cos(theta+omegat*deltat))/omegat^2+vt*sin(theta+omegat*deltat)*deltat/omegat;0,deltat];
Mt=[alpha_1*vt^2+alpha_2*omegat^2,0;0,alpha_3*vt^2+alpha_4*omegat^2];
mu_t=estimator_1+[(-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat));vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*deltat);wrapToPi(omegat*deltat)];
mu_t(3) = wrapToPi(mu_t(3));
Sigma_t=Gt*Sigmat_1*Gt'+Vt*Mt*Vt';

% elseif omegat==0
%     Gt=[1,0,-deltat*vt*sin(theta);0,1,deltat*vt*cos(theta);0,0,1];
%     Vt=[deltat*cos(theta),0;deltat*sin(theta),0;0,0];
%     Mt=[alpha_1*vt^2+alpha_2*omegat^2,0;0,alpha_3*vt^2+alpha_4*omegat^2];
%     mu_t=mut_1+[vt*deltat*cos(theta);vt*deltat*sin(theta);0];
%     Sigma_t=Gt*Sigmat_1*Gt'+Vt*Mt*Vt';
%     premu=mu_t;

%% correction step

Qt = diag([sigma_r^2,sigma_Phi^2]);
if ~isempty(detections)
    zn = numel(detections(:,1));   % number of the objects which is observated by the camera
    for n = 1:zn
        N = z_t(n,3);
        if (N > 5)
            N = N -5;
            q = (mx(N)-mu_t(1))^2+(my(N)-mu_t(2))^2;
            zt_Lambda = [sqrt(q);atan2(my(N)-mu_t(2),mx(N)-mu_t(1))-mu_t(3)];
            zt_Lambda(2) = wrapToPi(zt_Lambda(2));
            Ht=[-(mx(N)-mu_t(1))/sqrt(q),-(my(N)-mu_t(2))/sqrt(q),0;(my(N)-mu_t(2))/q,-(mx(N)-mu_t(1))/q,-1];
    %         Hm=[(mx(N)-mu_t(1))/sqrt(q),(my(N)-mu_t(2))/sqrt(q),0;-(my(N)-mu_t(2))/q,(mx(N)-mu_t(1))/q,0];
            St=Ht*Sigma_t*(Ht)'+Qt;
            %St=Ht*Sigma_t*(Ht)'+Qt+Hm*Sigma_m(:, 1+(N-1)*3 :3+(N-1)*3)*(Hm)';
            g = robot.groundTruth(:, iterator);
            Kt=Sigma_t*(Ht)'*inv(St);
            mu_t = mu_t+Kt*(z_t(n,1:2)'- zt_Lambda);
            Sigma_t=(I-Kt*Ht)*Sigma_t;            
        end
    end
end
estimatorT = mu_t;
sigmaT = Sigma_t;
%% Measurement Likelihood
% for i=1:N
%     p_zt=p_zt*(det(2*pi*St(i)))^(-1/2)*exp(-1/2*(z_t(i)-zt_Lambda(i))'*inverse(St(i))*(z_t(i)-zt_Lambda(i)));
% end
end