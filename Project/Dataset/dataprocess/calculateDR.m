function [estimatorDR]=calculateDR(period,odometry,simpletime)
%%
vt=odometry(1);
omegat=odometry(2);
theta=wrapToPi(period(3));

if omegat~=0
    estimatorDR=period+[-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*simpletime);vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*simpletime);omegat*simpletime];
else
    estimatorDR=period+[vt*simpletime*cos(theta);vt*simpletime*sin(theta);0];
end
estimatorDR(3)=wrapToPi(estimatorDR(3));
end