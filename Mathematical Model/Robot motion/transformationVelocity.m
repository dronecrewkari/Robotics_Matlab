function  state = transformationVelocity(state_pre, vt, omegat, theta, sampletime)

    if omegat~=0
        state = state_pre + [-vt/omegat*sin(theta) + vt/omegat*sin(theta+omegat*sampletime);
                                 vt/omegat*cos(theta)-vt/omegat*cos(theta+omegat*sampletime);
                                 omegat*sampletime];
    else
        state = state_pre + [vt*sampletime*cos(theta); 
                                 vt*sampletime*sin(theta);
                                 0];
    end
        state(3) = wrapToPi(state(3));
end
