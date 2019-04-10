function [] = drawRMSE(groundTruth, targetEstimator, compareEstimator, nameTarget, nameCompare)

n = numel(groundTruth(1, :));
figure;
sum = 0;
sum2 = 0;
rmseTarget = zeros(1, n);
rmseCompare = zeros(1, n);
for t = 1:n   
    actual = sqrt(groundTruth(1, t)^2 + groundTruth(2, t)^2);
    estimateTarget = sqrt(targetEstimator(1, t)^2 + targetEstimator(2, t)^2);
    estimateCompare = sqrt(compareEstimator(1, t)^2 + compareEstimator(2, t)^2);
    
    sum = sum + (actual - estimateTarget)^2;  
    sum2 = sum2 + (actual - estimateCompare)^2;

    rmseTarget(t) = sqrt(sum / t);
    rmseCompare(t) = sqrt(sum2 / t);
    plot(1:t, rmseTarget(1:t), 'r', 1:t, rmseCompare(1:t), 'b', 'linewidth', 2); hold on;
end
leg = legend(nameTarget, nameCompare);
% set(gca,'FontSize',15,'FontName','Times New Roman');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('RMSE of x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('RMSE of y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');

end