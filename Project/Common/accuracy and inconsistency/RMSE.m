function rmse=RMSE(actual,estimate,n)
b=0;
for a=1:n
    b=b+(actual(a)-estimate(a))^2;
end

rmse=sqrt(b/n);
end