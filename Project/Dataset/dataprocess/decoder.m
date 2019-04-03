function Measurement_de=decoder(Measurement,Barcodes)
for i=1:size(Measurement,1)
    n = find(Barcodes(:,2) == Measurement(i,2));
    if (~isempty(n))
        Measurement(i,2) = Barcodes(n,1);
    else
       Measurement(i,2) = -1;
    end
end
Measurement_de(:,1)=Measurement(:,3);
Measurement_de(:,2)=Measurement(:,4);
Measurement_de(:,3)=Measurement(:,2);
Measurement_de(:,4)=Measurement(:,1);
end
