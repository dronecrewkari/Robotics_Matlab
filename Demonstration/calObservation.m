function zt = calObservation(ground, object, SigmaQ)
     n = numel(object(:, 1));
     [ran, rad] = landmark_model(ground, object);
     noise = randn(n, 1) * SigmaQ;
     zt = [ran, rad, object(:, 3)] + noise;
end