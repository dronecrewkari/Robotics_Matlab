function vector_nor = normal_vector(vector)
    num = numel(vector);
    sumVector = sum(array);

    if sumVector ~= 0
        vector_nor = vector/sumVector;
    else
        vector_nor = zeros(1, num) + 1/num;
    end
end