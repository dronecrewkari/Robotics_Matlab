function [particleState_bar] = MCL_dead(pre_particle, ut, parameter_motion, sampleTime)

numParticle = size(pre_particle, 2);
particleState = zeros(3, numParticle);
particleState_bar = particleState;

% calculate particle and weight
for i = 1 : numParticle
    particleState_bar(:, i) = sample_velocity_model(ut, pre_particle(:, i), sampleTime, parameter_motion);
end
   