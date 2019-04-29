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

function [particleState, particleWeight] = MCL(pre_particle, ut, zt, m, parameter_motion, parameter_obser, sampleTime)

% initialise the state
numParticle = size(pre_particle, 2);
particleState = zeros(3, numParticle);
particleState_bar = particleState;
particleWeight = zeros(1, numParticle) + 1/numParticle;

% calculate particle and weight
for i = 1 : numParticle
    particleState_bar(:, i) = sample_velocity_model(ut, pre_particle(:, i), sampleTime, parameter_motion);
    particleWeight(:, i) = landmark_model_knownn_correspondence(zt, particleState_bar(:, i), m, parameter_obser); 
end
    particleWeight = normal_vector(particleWeight);
    
%resample
[particleState, particleWeight] = resampling_low_variance(particleState_bar, particleWeight);




    










