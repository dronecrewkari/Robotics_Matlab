function [upState, upSigma ] = updateStep(robot, estimatorObj, objSigama, iterator, valid)

if (iterator == numel(robot.estimatorEKF(1,:)))
    mu_t = robot.estimatorEKF(:, iterator);
    Sigma_t = robot.sigma{iterator};
else
    mu_t = robot.preState;
    Sigma_t = robot.preSigma;
end
   
sigma_r = robot.sensorNoise(1);
sigma_Phi = robot.sensorNoise(2);

z_t = robot.observation{iterator};

mx = estimatorObj(:,1);
my = estimatorObj(:,2);
Sigma_m = objSigama;

Qt = diag([sigma_r^2,sigma_Phi^2]);
I=eye(3); 
if ~isempty(z_t)
    zn = numel(z_t(:,1));   % number of the objects which is observated by the camera
    if (ismember(6, z_t(:,3)) && ismember(7, z_t(:,3)))
        for N = 6:7
            n = find(z_t(:,3) == N);
            q = (mx(N)-mu_t(1))^2+(my(N)-mu_t(2))^2;
            zt_Lambda=[sqrt(q);atan2(my(N)-mu_t(2),mx(N)-mu_t(1))-mu_t(3)];
            zt_Lambda(2) = wrapToPi(zt_Lambda(2));
            Ht=[-(mx(N)-mu_t(1))/sqrt(q),-(my(N)-mu_t(2))/sqrt(q),0;(my(N)-mu_t(2))/q,-(mx(N)-mu_t(1))/q,-1];
    %         Hm=[(mx(N)-mu_t(1))/sqrt(q),(my(N)-mu_t(2))/sqrt(q),0;-(my(N)-mu_t(2))/q,(mx(N)-mu_t(1))/q,0];
                                     St=Ht*Sigma_t*(Ht)'+Qt;
    %         St=Ht*Sigma_t*(Ht)'+Qt+Hm*Sigma_m(:, 1+(N-1)*3 :3+(N-1)*3)*(Hm)';
            g = robot.groundTruth(:, iterator);
            Kt=Sigma_t*(Ht)'*inv(St);
            error = g - mu_t;
            error2 = Kt*(z_t(n,1:2)'-zt_Lambda);
            mu_t = mu_t +Kt*(z_t(n,1:2)'-zt_Lambda);
            Sigma_t=(I-Kt*Ht)*Sigma_t;
        end
    else
        for n = 1:zn
            N = z_t(n,3);
            if (ismember(N,valid))
                q = (mx(N)-mu_t(1))^2+(my(N)-mu_t(2))^2;
                zt_Lambda=[sqrt(q);atan2(my(N)-mu_t(2),mx(N)-mu_t(1))-mu_t(3)];
                zt_Lambda(2) = wrapToPi(zt_Lambda(2));
                Ht=[-(mx(N)-mu_t(1))/sqrt(q),-(my(N)-mu_t(2))/sqrt(q),0;(my(N)-mu_t(2))/q,-(mx(N)-mu_t(1))/q,-1];
        %         Hm=[(mx(N)-mu_t(1))/sqrt(q),(my(N)-mu_t(2))/sqrt(q),0;-(my(N)-mu_t(2))/q,(mx(N)-mu_t(1))/q,0];
                                         St=Ht*Sigma_t*(Ht)'+Qt;
        %         St=Ht*Sigma_t*(Ht)'+Qt+Hm*Sigma_m(:, 1+(N-1)*3 :3+(N-1)*3)*(Hm)';
                Kt=Sigma_t*(Ht)'*inv(St);
                g = robot.groundTruth(:, iterator);
                mu_t=mu_t+Kt*(z_t(n,1:2)'-zt_Lambda);
                Sigma_t=(I-Kt*Ht)*Sigma_t;
            end
        end
    end
end

upState = mu_t;
upSigma = Sigma_t;
end