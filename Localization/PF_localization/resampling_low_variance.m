function [particle, particleWeight] = resampling_low_variance(particle, particleWeight)

numParticle = numel(particleWeight);
Neff = 1/(particleWeight * particleWeight'); % valid particle
Nth = numParticle * 2/3;

if Neff < Nth
    wcum = cumsum(particleWeight);
    r = rand/numParticle;
    ind = 1;
    pp = particle;
    for m = 1:numParticle
        U = r + (m - 1)/numParticle;
        while ( U > wcum(m))
            ind = ind + 1;
        end
        particle(m) = pp(ind);
        particleWeight(m) = 1/numParticle;
    end
end

end


