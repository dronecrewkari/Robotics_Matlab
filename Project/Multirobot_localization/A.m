anLineW = creatAnimatedLine(2, robot{1}(:,2:4)'); 

for i = 1:5000
    anLine(robot{1}(i,1:2), robot{1}(i,2:3));
end