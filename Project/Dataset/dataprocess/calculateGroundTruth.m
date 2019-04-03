function [GroundTruth]=calculateGroundTruth(period,odometry,simpletime)
%%
vt=odometry(1) + 0.1*randn(1);
omegat=odometry(2) + 0.05*randn(1);
theta=wrapToPi(period(3));

if omegat~=0
    GroundTruth=period+[-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*simpletime);vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*simpletime);omegat*simpletime];
else
    GroundTruth=period+[vt*simpletime*cos(theta);vt*simpletime*sin(theta);0];
end
GroundTruth(3)=wrapToPi(GroundTruth(3));
end