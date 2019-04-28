% -------------------------------------------------------------------------
% File : MCL.m
%
% Discription : Mobible robot localization sample code with
%               Monte Carlo Localization
%        
% Environment : Matlab
%
% Author : Herry
%
% Copyright (c): 2019 Herry
% -------------------------------------------------------------------------

function [particleState] = MCL(pre_particle, ut, zt, m, numParticle)

% initialise the state
particleState = zeros(3, numParticle);
particleState_bar = particleState;
particleWeight = zeros(1, numParticle);

% initialise the configuration
delt = 0.1;
parameter = [0.1, 0.05, 0.05, 0.1, 0.05, 0.05];

for i = 1 : numParticle
    particleState_bar(:, i) = sample_velocity_model(ut, pre_particle(:, i), delt, parameter);
    
    particleState_bar = particleState_bar;
end



