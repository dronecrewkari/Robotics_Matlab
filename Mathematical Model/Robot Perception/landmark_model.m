function [range, radian] = landmark_model(trueState, object)
     range = sqrt((trueState(1) - object(:, 1)).^2 + (trueState(2) - object(:, 2)).^2);
     radian = wrapToPi(atan2(object(:, 2) - trueState(2), object(:, 1) - trueState(1)) - trueState(3)); 
end