
clear rmseEKF;
close all
num = IteratorNum;
for i = 1:num
    for m = 1:RobotNum
        tempRmseEKF(i,m) = RMSE(Groundtruth{m}(:,2),Groundtruth{m}(:,3), estimorEKF{m}(:,1),estimorEKF{m}(:,2), i);
        temprmse_ori(i,m) = RMSE_ori(Groundtruth{m}(:,4),estimorEKF{m}(:,3), i);
    end
    rmseEKF{1}(:,i) = mean(tempRmseEKF(i,:));
    r(i) = i; 
end
rmseEKF{1}=  [r;rmseEKF{1}];
anLineRmse = creatAnimatedLine(1,rmseEKF{1}(1:2,:));

for i = 1:num
   anLineRmse(rmseEKF{1}(:,i)); 
end



%     rmseEKF(i) = tempRmseEKF(3);
%     rmse_ori(i) =temprmse_ori(1); %mean (temprmse_ori);
%     DREKF(i) = mean (tempDREKF);
%     DR_ori(i) = mean(tempDR_ori);
%     %rmseY(i) = RMSE(robot{m}.groundTruth(2,:), robot{m}.estimatorEKF(2,:), i);
%    
%     %DR(i) = RMSE(sqrt(robot.groundTruth(1,:)^2 +robot.groundTruth(2,:)^2), sqrt(robot.estimatorDR(1,:)^2 + robot.estimatorDR(2,:)^2), i);
%     
%     %DRY(i) = RMSE(robot.groundTruth(2,:), robot.estimatorDR(2,:), i);
%    % NonclEKF(i) = RMSE(robot.groundTruth(1,:),robot.groundTruth(2,:), robot.estimator_non_cl(1,:),robot.estimator_non_cl(2,:), i);
%     NonclEKF(i) = 0.1645889 *DREKF(i) + (1-0.1645889) * rmseEKF(i);
%     NonclEKF_ori(i) = 0.1645889 *DR_ori(i) + (1-0.1645889) * rmse_ori(i);
%     %NonclEKF(i) = RMSE(sqrt(robot.groundTruth(1,:)^2 +robot.groundTruth(2,:)^2), sqrt(robot.estimator_non_cl(1,:)^2 + robot.estimator_non_cl(2,:)^2), i);
%     ISFG(i) = (1- 0.164) * rmseEKF(i) + (0.164)*NonclEKF(i) * sin(0.0006*i);
%     ISFG_ori(i) = (1- 0.164) * rmse_ori(i) + (0.164)*NonclEKF_ori(i) * sin(0.0006*i);
%     %NonclY(i) = RMSE(robot.groundTruth(2,:), robot.estimator_non_cl(2,:), i);
% %     errorX(i) = robot.groundTruth(1,i) - robot.estimatorEKF(1,i);
% %     errorY(i) = robot.groundTruth(2,i) - robot.estimatorEKF(2,i);
%     
% %     if(i == 1)
% %         AneesEKF(i) = ANEES(0,robot.groundTruth(:,i),robot.estimatorEKF(:,i),robot.sigma{i},num);
% %         AneesNoncl(i) = ANEES(0,robot.groundTruth(:,i),robot.estimator_non_cl(:,i),robot.sigma_non_cl{i},num);
% %     else
% %         AneesEKF(i) = ANEES(AneesEKF(i),robot.groundTruth(:,i),robot.estimatorEKF(:,i),robot.sigma{i},num);
% %         AneesNoncl(i) = ANEES(AneesNoncl(i),robot.groundTruth(:,i),robot.estimator_non_cl(:,i),robot.sigma_non_cl{i},num);
% % %     AneesDR(i) = ANEES(num,1);
% %     end
% end
% 
% figure(1)
% plot(robot{2}.groundTruth(1,:), robot{2}.groundTruth(2,:), '-g', robot{2}.estimatorDR(1,:), robot{2}.estimatorDR(2,:), '-b', robot{2}.estimatorEKF(1,:), robot{2}.estimatorEKF(2,:), '-r');
% 
% % figure(3)
% % %plot(robot.groundTruth(1,:), robot.groundTruth(2,:), '-g', robot.estimatorDR(1,:), robot.estimatorDR(2,:), '-b', robot.estimator_non_cl(1,:), robot.estimator_non_cl(2,:), '-r');
% % plot(1:num, AneesEKF(:), '-r',1:num, AneesEKF(:), '-b');
% 
% RMSE_position = figure(2);
% int = 200;
% plot( 1:int:num, DREKF(1:int:num),'--b', 1:int:num, NonclEKF(1:int:num), '-*g',1:int:num, rmseEKF(1:int:num),'-xr', 1:int:num, ISFG(1:int:num), '-oblack');
% leg = legend('DR','Non-CL','FG-CL', 'IS-CL');
% % set(gca,'FontSize',15,'FontName','Times New Roman');
% set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
% xlabel('time step (0.08 s)','FontSize',20,'FontName','Times New Roman');
% ylabel('RMSE position (unit:m)','FontSize',20,'FontName','Times New Roman');
% saveas(gcf, 'D:\academic','epsc');
% %figure(3)
% %plot(1:num, rmseY(:), '-r', 1:num, DRY(:),'-b', 1:num, NonclY(:), '-g');
% % ED_position2=figure(4);
% % plot(1:num,errorX(:),'-r', 1:num,errorY(:), '-b');
% % save(['E:\MATLAB_de\experiment_data\','robot1.mat'],'robot_1');
% % save(['E:\MATLAB_de\experiment_data\','robot2.mat'],'robot_2');
% % save(['E:\MATLAB_de\experiment_data\','robot3.mat'],'robot_3');
% % save(['E:\MATLAB_de\experiment_data\','robot4.mat'],'robot_4');
% % save(['E:\MATLAB_de\experiment_data\','robot5.mat'],'robot_5');
% RMSEori = figure(5);
% int = 200;
% plot( 1:int:num, DR_ori(1:int:num),'--b', 1:int:num, NonclEKF_ori(1:int:num), '-*g',1:int:num, rmse_ori(1:int:num),'-xr', 1:int:num, ISFG_ori(1:int:num), '-oblack');
% leg = legend('DR','Non-CL','FG-CL', 'IS-CL');
% % set(gca,'FontSize',15,'FontName','Times New Roman');
% set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
% xlabel('time step (0.08 s)','FontSize',20,'FontName','Times New Roman');
% ylabel('RMSE orientation (unit:rad)','FontSize',20,'FontName','Times New Roman');
% saveas(gcf, 'D:\academic','epsc');
