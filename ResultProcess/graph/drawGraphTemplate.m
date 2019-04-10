function [] = drawGraphTemplate(groundTruth, estimatorTarget, estimatorCompare)

figEKF = figure(1);
set(figEKF, 'name', 'localization by EKF')

plot(groundTruth(1, :), groundTruth(2, :), '-g', 'linewidth', 2); hold on;
plot(estimatorTarget(1, :), estimatorTarget(2, :), 'xr', 'linewidth', 2); hold on;
plot(estimatorCompare(1, :), estimatorCompare(2, :), '--b', 'linewidth', 2); hold on;

leg = legend('Ground Truth','Target Estimator','Compare Estimator', 'Error Ellipse');
% set(gca,'FontSize',15,'FontName','Times New Roman');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
zoom on;
end


