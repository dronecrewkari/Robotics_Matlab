function groundTruth = calGround(mut_pre, ut, alpha_noise, sampletime)
   
    vt = ut(1) + sqrt(alpha_noise(1)*ut(1)^2 + alpha_noise(2)*ut(2)^2) * randn(1);
    omegat = ut(2) + sqrt(alpha_noise(3)*ut(1)^2 + alpha_noise(4)*ut(2)^2) * randn(1);
    theta = wrapToPi(mut_pre(3));
    
    groundTruth = transformationVelocity(mut_pre, vt, omegat, theta, sampletime);
end