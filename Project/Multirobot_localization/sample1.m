IteratorNum = 5000;
sampleTime = 0.02;

RobotNum = 5;
landmarkNum = 15;

Odometry = {5};
Groundtruth = {5};
Landmark = {15};
offset = [0,10,10,0];
offset_land = [10,10];
for i = 1 :RobotNum
    Groundtruth {i} = eval(['Robot' num2str(i) '_Groundtruth']) + offset;
    Odometry{i} = eval(['Robot' num2str(i) '_Odometry']);
end
for i = 1 : landmarkNum
    Landmark{i} = Landmark_Groundtruth(i, 2:3) + offset_land;   
end

clearvars -except Groundtruth Odometry Landmark IteratorNum sampleTime RobotNum landmarkNum;





