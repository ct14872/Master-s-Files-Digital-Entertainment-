function t_or_f = isclose(v1,v2,dp)
% My implementation of isclose function like in Python's library
    len = length(v1);
    len2 = length(v2);
    if(len == len2)
        v = zeros(size(v1));
        v1 = round(v1,dp);
        v2 = round(v2,dp);
        for i = 1:len
            diff = v1(i)-v2(i);
            v(i) = diff;
        end
        if(sum(v) == 0)
            t_or_f = 1;
        else
            t_or_f = 0;
        end
    else
        error('Vectors must have the same size');
    end
end
                
                