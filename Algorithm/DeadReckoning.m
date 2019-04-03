function [estimatorDR] = DeadReckoning(robot, sampletime, iterator)

odometry = robot.odometry(:, iterator);
vt=odometry(1);
omegat=odometry(2);
period = robot.estimatorDR(:, iterator - 1);
theta = wrapToPi(period(3));

if omegat~=0
    estimatorDR = period + [-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*sampletime);vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*sampletime);omegat*sampletime];
else
    estimatorDR = period + [vt*sampletime*cos(theta);vt*sampletime*sin(theta);0];
end
estimatorDR(3)=wrapToPi(estimatorDR(3));
end

