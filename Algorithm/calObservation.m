function zt = calObservation(ground, object, SigmaQ)
     n = numel(object(:,1));
     q = (ground(1) - object(:, 1)).^2 + (ground(2) - object(:, 2)).^2;
     zt = [sqrt(q), wrapToPi(atan2(object(:, 2) - ground(2), object(:, 1) - ground(1)) - ground(3)), object(:, 3)] + [rand(n, 2) .* SigmaQ, zeros(n, 1)] ;
end