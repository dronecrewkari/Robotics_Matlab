sampleNum = 5000;
for n = 1:5
        eval(['robot' num2str(n) '.Ground(1:3,1:sampleNum) = Robot' num2str(n) '_Groundtruth(1:sampleNum,2:4)''']);
        eval(['robot' num2str(n) '.Odometry(:,1:sampleNum) = Robot' num2str(n) '_Odometry(1:sampleNum,2:3)''']);
        eval(['Measurement' num2str(n) '= decoder(Robot' num2str(n) '_Measurement,Barcodes)']);
end

% test
for n = 1:5
    j = 1;
    eval(['Measurement = Measurement' num2str(n) '']);
    eval(['robot = robot' num2str(n) '']);
    for i = 1:numel(Measurement(:,1))
        N = Measurement(i, 3);
        n = fix(Measurement(i, 4)/0.02) + 1;
        if(N > 5 && n < sampleNum) 
            N = N - 5;
            lmx = Landmark_Groundtruth(N, 2);
            lmy = Landmark_Groundtruth(N, 3);
            x = robot2.Ground(1, n);
            y = robot2.Ground(2, n);
            theta = robot2.Ground(3, n);
            r = sqrt((lmx - x)^2 + (lmy - y)^2);
            phi = atan2(lmy - y, lmx - x) - theta;
            ob = [r; phi];
            j = j + 1;
            Diff_ob(:,i) =  Measurement(i, 1:2)' - ob;
            if (abs(Diff_ob(1, i)) > 0.2 || abs(Diff_ob(2, i) > 0.2))    %filter the value with error
                Measurement(i, 3) = -1;
            end
        end
    end
end
%

Landmark_Groundtruth(:,4) = Landmark_Groundtruth(:,1);
Landmark_Groundtruth(:,1) = Landmark_Groundtruth(:,2);
Landmark_Groundtruth(:,2) = Landmark_Groundtruth(:,3);
Landmark_Groundtruth(:,3) = Landmark_Groundtruth(:,4);

for n = 1:5
    p = 1;
    q = 1;
    r = 1;
    for i = 0:0.02:(sampleNum*0.02 - 0.02) 
        eval(['robot' num2str(n) '.measurement{q} = []'])
        while (i == eval(['Robot' num2str(n) '_Measurement(p,1)']))
            eval(['robot' num2str(n) '.measurement{q}(r,1:3) = Measurement' num2str(n) '(p,1:3)']);
            p = p + 1;
            r = r + 1;
        end
        q = q + 1;
        r = 1;   
    end
end
