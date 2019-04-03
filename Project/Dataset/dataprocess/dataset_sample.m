function [relative]=dataset_sample(Robot_Measurement,Barcodes)

c=1;
for i=1:length(Robot_Measurement(:,1))
    a=Robot_Measurement(i,2);
    b=Barcodes(:,2);
    if ismember(a,b)
        relative(c,:)=Robot_Measurement(i,:);
        c=c+1;
    end
end
relative=decoder(relative,Barcodes);
end