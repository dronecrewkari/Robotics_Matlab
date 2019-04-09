function rmseOri = RMSE_ori(actual_ori, estimate_ori, n)
b=0;
for a=1:n
    actual = actual_ori(a);
    estimate = estimate_ori(a);
    b=b+(actual-estimate)^2;
end

rmseOri = sqrt(b/n);
end