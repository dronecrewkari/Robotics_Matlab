function [] = drawGraphTemplate(groundTruth, estimatorTarget, estimatorCompare, nameGround, nameTarget, nameCompare)

figEKF = figure(1);
set(figEKF, 'name', 'localization by EKF')

n = numel(groundTruth, 2);
t = 1:3:n;
plot(groundTruth(1, t), groundTruth(2, t), '-g', 'linewidth', 2); hold on;
plot(estimatorTarget(1, t), estimatorTarget(2, t), 'xr', 'linewidth', 2); hold on;
plot(estimatorCompare(1, t), estimatorCompare(2, t), '--b', 'linewidth', 2); hold on;

leg = legend(nameGround, nameTarget, nameCompare, 'Error Ellipse');
% set(gca,'FontSize',15,'FontName','Times New Roman');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
end


