close all;

viz = Visualizer2D;
viz.mapName = 'CLMap';
attachObjectDetector(viz,detector);



for n = 1:RobotNum
  anLine{n} = creatAnimatedLine(2, Groundtruth{n}(:,2:4)');  
end


for i = 1:20
    if i < 6
        viz.objectColors(i,:) = [1, 0, 0];
    else
        viz.objectColors(i,:) = [0, 1, 0];
    end
end

for i = 1:IteratorNum
    for m = 1:RobotNum
        Objects(m,:) = [Groundtruth{m}(i,2:3),m];
        anLine{m}(estimorEKF{m}(i,1:2), Groundtruth{m}(i,2:3));
    end
    viz(Groundtruth{1}(i,2:4)',Objects);
    
end






