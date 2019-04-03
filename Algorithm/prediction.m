function [preState, preSigma] = prediction(robot, sampleTime, iterator)

deltat = sampleTime;   % sample time
theta = robot.estimatorEKF(3, iterator - 1); % theta estimated in t-1;
vt = robot.odometry(1, iterator);
omegat = robot.odometry(2, iterator);

Sigmat_1 = robot.sigma{iterator - 1};
estimator_1 = robot.estimatorEKF(1:3, iterator-1);

alpha_1=robot.controlNoise(1);
alpha_2=robot.controlNoise(2);
alpha_3=robot.controlNoise(3);
alpha_4=robot.controlNoise(4);

if omegat==0
    omegat=0.001*randn(1);
end
Gt=[1,0,-vt/omegat*cos(theta)+vt/omegat*cos(theta+omegat*deltat);0,1,-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat);0,0,1];
Vt=[(-sin(theta)+sin(theta+omegat*deltat))/omegat,vt*(sin(theta)-sin(theta+omegat*deltat))/omegat^2+vt*cos(theta+omegat*deltat)*deltat/omegat;(cos(theta)-cos(theta+omegat*deltat))/omegat,-vt*(cos(theta)-cos(theta+omegat*deltat))/omegat^2+vt*sin(theta+omegat*deltat)*deltat/omegat;0,deltat];
Mt=[alpha_1*vt^2+alpha_2*omegat^2,0;0,alpha_3*vt^2+alpha_4*omegat^2];
mu_t=estimator_1+[(-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*deltat));vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*deltat);wrapToPi(omegat*deltat)];
mu_t(3) = wrapToPi(mu_t(3)); 
Sigma_t = Gt*Sigmat_1*Gt'+Vt*Mt*Vt';

preState = mu_t;
preSigma = Sigma_t;

