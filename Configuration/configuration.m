CLMap = robotics.OccupancyGrid(20,20);
robot= {1};
landmark=[];
for i = 1:RobotNum
    robot{i} = linearAngular(i, Groundtruth{i}(1,2:4), Odometry{i}(1,2:3), 0.001*eye(3)); 
    Objects(i,:) = [Groundtruth{i}(1,2:3),i];
end

for i = 1 : landmarkNum
    landmark(i,:) =[Landmark{i}, i + RobotNum];    
    Objects(i + RobotNum, :) = landmark(i,:);
end

detector = ObjectDetector;
detector.mapName = 'CLMap';



