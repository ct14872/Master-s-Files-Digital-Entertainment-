function out = ease(t,k1,k2)
% Taken from Computer Animation book by Rick Parent, page 82
    f = k1*2/pi + k2 - k1 + (1.0 - k2)*2/pi;
    if(t < k1) 
        s = k1*(2/pi)*(sin((t/k1)*pi/2 - pi/2)+1);
    elseif(t < k2) 
        s = (2*k1/pi + t - k1);
    else
        s = 2*k1/pi + k2 - k1 + ((1-k2)*(2/pi)) *sin(((t - k2)/(1.0 - k2))*pi/2);
    end

    out = (s/f);

end