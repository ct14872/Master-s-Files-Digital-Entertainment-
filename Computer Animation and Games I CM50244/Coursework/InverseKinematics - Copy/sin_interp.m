function frames = sin_interp(A, stop_acc, start_dec)
% 2 Frame interpolation
    start = A(1,:);
    goal = A(end,:);
    diff = abs(goal-start);
    time = 0.01:0.01:1;
    frames = start;
    coeff = zeros(size(A,2),1);
    for i = 1:length(coeff);
        if start(i)<goal(i)
            coeff(i) = +1;
        else
            coeff(i) = -1;
        end
    end
    next = start;
    total = 0;
    in = ease(time(1),stop_acc,start_dec);
    for j = 1:size(A,2)
            next(j) = next(j) + in*coeff(j)*diff(j);
    end
    for t = 2:length(time)-1
        frames = [frames; next];
        scalar = ease(time(t), stop_acc, start_dec)-ease(time(t-1), stop_acc, start_dec);
        total = total + scalar;
        for i = 1:size(A,2)
            next(i) = next(i) + scalar*coeff(i)*diff(i);
        end
        
    end
    frames = [frames; goal];
    total = total + in;
end