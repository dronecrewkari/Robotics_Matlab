function groundTruth = calGround(mut_pre, ut, alpha_noise, sampletime)
   
    vt = ut(1) + (alpha_noise(1)*ut(1)^2 + alpha_noise(2)*ut(2)^2) * rand(1);
    omegat = ut(2) + (alpha_noise(3)*ut(1)^2 + alpha_noise(4)*ut(2)^2) * rand(1);
    theta = wrapToPi(mut_pre(3));
      
    if omegat~=0
        groundTruth = mut_pre + [-vt/omegat*sin(theta) + vt/omegat*sin(theta+omegat*sampletime);
                                 vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*sampletime);
                                 omegat*sampletime];
    else
        groundTruth = mut_pre + [vt*sampletime*cos(theta); 
                                 vt*sampletime*sin(theta);
                                 0];
    end
        groundTruth(3) = wrapToPi(groundTruth(3));
end