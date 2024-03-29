function ShowErrorEllipse(estimator, sigma, scale)

covXy = sigma(1:2,1:2); % 
[eigvec, eigval]=eig(covXy); % 

if eigval(1,1)>=eigval(2,2)
    bigind=1;
    smallind=2;
else
    bigind=2;
    smallind=1;
end

chi=9.21; 

t = 0:10:360;
a = sqrt(eigval(bigind,bigind)*chi);
b = sqrt(eigval(smallind,smallind)*chi);
x = [a*cosd(t);
   b*sind(t)];

angle = atan2(eigvec(bigind,2),eigvec(bigind,1));
if(angle < 0)
    angle = angle + 2*pi;
end

R=[cos(angle) sin(angle);
   -sin(angle) cos(angle)];
x = scale * R * x;
plot(x(1,:) + estimator(1), x(2,:) + estimator(2), '-black')
end