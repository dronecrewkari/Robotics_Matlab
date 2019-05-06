function [x, y, theta] = sample_landmark_known(fi, m, parameters)
    %% initialise
    numObser = size(fi, 1);
    ri = fi(1, 1);
    Phi = fi(1, 2);
    si = fi(1, 3);

    j = si;
    upsilon = 2*pi*rand;
    
    m_x = m(j, 1);
    m_y = m(j, 2);

    sigma_r = parameters(1);
    sigma_Phi = parameters(2);
    
    rLamda = ri + sample_distribution(sigma_r);
    PhiLamda = Phi + sample_distribution(sigma_Phi);
    
    x = m_x + rLamda * cos(upsilon);
    y = m_y + rLamda * sin(upsilon);
    theta = wrapToPi(upsilon - pi - PhiLamda);
end