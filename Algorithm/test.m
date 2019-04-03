g1 = robot_1.groundTruth;
g2 = robot_2.groundTruth;
o = zeros(8,1000);
for i = 1:1000
    q = (g2(1,i)-g1(1,i))^2+(g2(2,i)-g1(2,i))^2;
    z=[sqrt(q);atan2(g2(2,i)-g1(2,i),g2(1,i)-g1(1,i))]; 
    z(2) = wrapToPi(z(2));
    o(1:2,i) = z;
    q2 = (g1(1,i)-g2(1,i))^2+(g1(2,i)-g2(2,i))^2;
    z2=[sqrt(q2);atan2(g1(2,i)-g2(2,i),g1(1,i)-g2(1,i))]; 
    z2(2) = wrapToPi(z2(2));
    o(3:4,i) = z2; 
    o(5,i) = wrapToPi(z(2) - pi);
end

for i = 1:numel(robot_1.observation(:,1))
    N = Measurement1(i, 3);
    n = fix(Measurement1(i, 4)/0.02) + 1;
    if(N > 5 && n < sampleNum) 
        N = N - 5;
        lmx = Landmark_Groundtruth(N, 2);
        lmy = Landmark_Groundtruth(N, 3);
        x = robot1.Ground(1, n);
        y = robot1.Ground(2, n);
        theta = robot1.Ground(3, n);
        r = sqrt((lmx - x)^2 + (lmy - y)^2);
        phi = atan2(lmy - y, lmx - x) - theta;
        ob = [r; phi];
        j = j + 1;
        Diff_ob(:,i) =  Measurement1(i, 1:2)' - ob;
        if (abs(Diff_ob(1, i)) > 0.1 || abs(Diff_ob(2, i) > 0.1))    %filter the value with error
            Measurement1(i, 3) = -1;
        end
    end
end

