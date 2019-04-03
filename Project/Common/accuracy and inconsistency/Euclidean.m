function [euclidean]=Euclidean(actual,estimate,n)
b=0;
for i=1:n
    b=b+norm(actual-estimate,2);
end
euclidean=b/n;