
for i = 1:RobotNum
    estimorEKF{i} = Groundtruth{i}(1,2:4);
    estimorEKF_sigma{i} = 0.001*eye(3);
    estimorDR{i} = Groundtruth{i}(1,2:4);
end

for idx = 2:IteratorNum
    
    for i = 1:RobotNum
        Objects(i,:) = [Groundtruth{i}(idx,2:3),i];
    end
    
    for i = 1:RobotNum
        Pre_robot{i} = robot{i};
        robot{i}.groundTruth = Groundtruth{i}(idx,2:4);
        robot{i}.odometry = Odometry{i}(idx, 2:3);
        temp_obj = Objects(i,:);
        Objects(i,:) = [0,0,0];
        detections = detector(robot{i}.groundTruth', Objects);
%         d = (Objects(13,1)-robot{i}.groundTruth(1))^2+(Objects(13,2)-robot{i}.groundTruth(2))^2;
%         d = sqrt(d);
        [robot{i}.estimatorEKF, robot{i}.sigma] = EKF_Realdata (robot{i}, Pre_robot{i}, detections, Objects, sampleTime);
         robot{i}.estimatorDR  = DeadReckoning (robot{i}, Pre_robot{i}, sampleTime);
        estimorEKF{i}(idx, :) = robot{i}.estimatorEKF;
        estimorEKF_sigma{i,idx} = robot{i}.sigma;
        estimorDR{i}(idx, :) = robot{i}.estimatorDR;
        Objects(i,:) = temp_obj;
    end
    
    
   
    
end