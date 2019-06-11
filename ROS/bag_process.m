bag = rosbag('D:/dataset/VO_UWB/2019-06-03-23-08-50.bag');
groundtruth = select(bag,'Topic','/slave1/ground_pose');
GroundStructs = readMessages(groundtruth,'DataFormat','struct');

uwb = select(bag,'Topic','/uwb_position');
uwbStructs = readMessages(uwb,'DataFormat','struct');

vo = select(bag,'Topic','/vo_position');
voStructs = readMessages(vo,'DataFormat','struct');

figure(1) 
xPoints = cellfun(@(m) double(m.X), GroundStructs);
yPoints = cellfun(@(m) double(m.Y), GroundStructs);

%xPoints = fliplr(xPoints);
%yPoints = fliplr(yPoints);
%xPoints = smoothdata(xPoints, 'movmedian',5);
%yPoints = smoothdata(yPoints, 'movmedian',5);
plot(xPoints,yPoints);

leg = legend('Ground truth');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');

figure(2)
xUwb = cellfun(@(m) double(m.Position.X), uwbStructs);
yUwb = cellfun(@(m) double(m.Position.Y), uwbStructs);
yUwb = - yUwb;
%xUwb = round(xUwb, 3);
%yUwb = round(yUwb, 3);
%xUwb = smoothdata(xUwb);
%yUwb = smoothdata(yUwb);
plot(xUwb,yUwb);

leg = legend('UWB');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');

figure(3)
xvo = cellfun(@(m) double(m.Point.Y), voStructs);
yvo = cellfun(@(m) double(m.Point.Z), voStructs);

locate1 = find(xvo == 0);
locate2 = find(yvo == 0);

xvo(locate1) = [];
yvo(locate2) = [];
%xvo = smoothdata(xvo);
%yvo = smoothdata(yvo);
%xvoo = xvo(288:end);
%xvooo = abs(xvoo - xvo(288)) +  xvo(288);
%xvo(288:end) = xvooo;
%xvo = smoothdata(xvo, 'movmedian',5);
%yvo = smoothdata(yvo, 'movmedian',5);

plot(xvo,yvo);
leg = legend('VO');
% set(gca,'FontSize',15,'FontName','Times New Roman');
set(leg,'FontSize',15,'location','NorthWest','FontName','Times New Roman');
xlabel('x position (unit:m)', 'FontSize',20,'FontName','Times New Roman');
ylabel('y position (unit:m)', 'FontSize',20,'FontName','Times New Roman');



