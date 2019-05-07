% -------------------------------------------------------------------------
% File : Augmented_MCL.m
%
% Discription : Mobible robot localization sample code with Monte Carlo Localization
% 
% pamameter: pre_particle ------- previous particle --------- 3 * number of particle matrix
%            ut ----------------- odometry ------------------ 2 * 1 matrix
%            zt ----------------- obervation ---------------- 3 * 1 matrix
%            m ------------------ position of landmark ------ 3 * 1 matrix
%            numParticle -------- number of particles ------- number
%            parameter_motion --- nosise in motion ---------- 1 * 6 matrix
%            parameter_obser ---- nosise in observation ----- 1 * 3 matrix
%        
% Environment : Matlab
%
% Author : Herry
%
% Copyright (c): 2019 Herry
% -------------------------------------------------------------------------

function [particleState, particleWeight] = Augmented_MCL(pre_particle, ut, zt, m, parameter_motion, parameter_obser, sampleTime)

% initialise the state
global wslow wfast;
alphaSlow = 0.2;
alphaFast = 0.8;
numParticle = size(pre_particle, 2);
particleState = zeros(3, numParticle);
particleState_bar = particleState;
particleWeight = zeros(1, numParticle) + 1/numParticle;
wavg = 0;
% calculate particle and weight
for i = 1 : numParticle
    particleState_bar(:, i) = sample_velocity_model(ut, pre_particle(:, i), sampleTime, parameter_motion);
    particleWeight(:, i) = landmark_known(zt, particleState_bar(:, i), m, parameter_obser); 
    wavg = wavg + particleWeight(:, i)/numParticle;
end
    wslow = wslow + alphaSlow * (wavg - wslow);
    wfast = wfast + alphaFast * (wavg - wfast);
    
    if wslow == 0
        ratio = 0;
    else
        ratio = wfast/wslow;
    end
    
% resample
particleWeight = normal_vector(particleWeight);
particleState = particleState_bar;
Neff = 1/(particleWeight * particleWeight'); % valid particle
Nth = numParticle + 1;

if Neff < Nth
    wcum = cumsum(particleWeight);
    r = rand/numParticle;
    ind = 1;
    pp = particleState;
    for nump = 1:numParticle
        if rand < max(0, 1 - ratio) % the probablity
            [a, b, c] = sample_landmark_known(zt, m, parameter_obser);
            if (rem(nump, 5) == 0)
                particleState(:, nump) = [a, b, c]';
                particleWeight(nump) = 0;
            end
        else
            U = r + (nump - 1)/numParticle;
            while ( U > wcum(ind))
                ind = ind + 1;
            end
            particleState(:, nump) = pp(:, ind);
            particleWeight(nump) = 1/numParticle;
        end    
    end
end



