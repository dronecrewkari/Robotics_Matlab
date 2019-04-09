function [estimatorDR] = DeadReckoning(mut_1, ut, sampletime)

vt = ut(1);
omegat = ut(2);
period = mut_1;
theta = wrapToPi(period(3));

if omegat~=0
    estimatorDR = period + [-vt/omegat*sin(theta)+vt/omegat*sin(theta+omegat*sampletime);vt/omegat*cos(theta) - vt/omegat*cos(theta+omegat*sampletime);omegat*sampletime];
else
    estimatorDR = period + [vt*sampletime*cos(theta);vt*sampletime*sin(theta);0];
end
    estimatorDR(3)=wrapToPi(estimatorDR(3));
end

