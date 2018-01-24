function frames = sin_interp2(A, stop_acc, start_dec)
% 4 Frame interpolation - Not very proud of
    fr_until_now = size(A,1);
    stop_acc_frame = ceil(fr_until_now*stop_acc);
    start_dec_frame = floor(fr_until_now*start_dec);
    
    frame_span = [1 stop_acc_frame start_dec_frame fr_until_now];
    time1 = 0.01:0.01:stop_acc-0.01;
    time2 = stop_acc:0.01:start_dec-0.01;
    time3 = start_dec:0.01:1;
    frames = [];
    total_start = A(1,:);
    total_goal = A(end,:);
    total_diff = abs(total_goal-total_start);
    
    for fr = 1:length(frame_span)-1
        start = A(frame_span(fr),:);
        goal = A(frame_span(fr+1),:); %if problem add 1
        diff = abs(goal-start);
        these_frames = start;
        coeff = zeros(size(A,2),1);
        for i = 1:length(coeff);
            if start(i)<goal(i)
                coeff(i) = +1;
            else
                coeff(i) = -1;
            end
        end
        if fr==1
            time_now = time1;
        elseif fr==2
            time_now = time2;
        elseif fr==3
            time_now = time3;
        end
        
        s = ease(time_now,stop_acc,start_dec);
        s = s-s(1);
        s = (s)/s(end);
        
        next = start;
        total = 0;
        in = s(1);
        
        for j = 1:length(next)
            next(j) = next(j) + in*coeff(j)*diff(j);
        end
        for t = 2:length(s)-1
            these_frames = [these_frames; next];
            scalar = s(t)-s(t-1);
            total = total + scalar;
            for i = 1:size(A,2)
                next(i) = next(i) + scalar*coeff(i)*diff(i);
            end
        
        end
        these_frames = [these_frames; goal];
        frames = [frames; these_frames]

    end

    total = total + in
end
    
