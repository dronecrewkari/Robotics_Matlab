function rmse = RMSE(actualX, actualY, estimateX, estimateY,n)
b=0;
for a=1:n
    actual = sqrt(actualX(a)^2 + actualY(a)^2);
    estimate = sqrt(estimateX(a)^2 + estimateY(a)^2);
    b=b+(actual-estimate)^2;
end

rmse = sqrt(b/n);
end