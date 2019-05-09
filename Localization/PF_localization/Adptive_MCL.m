% -------------------------------------------------------------------------
% File : MCL.m
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

function [particleState, particleWeight] = Adptive_MCL(pre_particle, pre_particleWeight, ut, zt, m, parameter_motion, parameter_obser, sampleTime, Delta, Epsilon)

% initialise the state
numParticle = size(pre_particle, 2);
particleState = zeros(3, numParticle);

M = 1, Mx = 1, k = 0;

while M < Mx || M < Mxmin 
    
    wcum = cumsum(pre_particleWeight);
    r = rand/numParticle;
    ind = 1;
    pp = pre_particle;

    U = r + (M - 1)/numParticle;
    while ( U > wcum(ind))
         ind = ind + 1;
    end
    pre_state = pp(:, ind);
    
    particleState(:, M) = sample_velocity_model(ut, pre_state, sampleTime, parameter_motion);
    particleWeight(M) = landmark_known(zt, particleState(:, M), m, parameter_obser);
    
    if 
        
    end
end
   
%resample




    