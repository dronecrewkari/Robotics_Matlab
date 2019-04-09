function zt = calObservation(ground, object, SigmaQ)
     n = numel(object(:, 1));
     [ran, rad] = landmark_model(ground, object);
     noise = [rand(n, 2) .* SigmaQ, zeros(n, 1)];
     zt = [ran, rad, object(:, 3)] + noise;
end