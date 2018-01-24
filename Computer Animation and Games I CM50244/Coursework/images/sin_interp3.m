function frames = sin_interp3(A, stop_acc, start_dec)

    fr_until_now = size(A,1);
    turns = fr_until_now/10;
    
    frames = [];
    total_start = A(1,:);
    total_goal = A(end,:);
    total_diff = abs(total_goal-total_start);
    prev_portion = 0;
    portions = 0;
    for fr = 1:fr_until_now
        start = A(fr,:);
        goal = A(fr+1,:); %if problem add 1
        diff = abs(goal-start);
        portion = sum(diff)/sum(total_diff);
        portions = portions + portion
        portion = portion - mod(portion,0.01);
        if fr == 1
            time_now = 0:0.01:portion;
        elseif fr == fr_until_now
            time_now = previous_portion:0.01:1;
        else
            time_now = prev_portion:0.01:portion;
        end
        time_now
        prev_portion = portion;
        
        these_frames = start;
        coeff = zeros(size(A,2),1);
        for i = 1:length(coeff);
            if start(i)<goal(i)
                coeff(i) = +1;
            else
                coeff(i) = -1;
            end
        end
        
        s = ease(time_now,stop_acc,start_dec)
        s = s-s(1);
        s = (s)/s(end);
        
        next = start;
        total = 0;
        in = s(1);%ease(time_now(1),stop_acc,start_dec);
        
        for j = 1:length(next)
            next(j) = next(j) + in*coeff(j)*diff(j);
        end
        for t = 2:length(s)-1
            these_frames = [these_frames; next];
            scalar = s(t)-s(t-1);%ease(time_now(t), stop_acc, start_dec)-ease(time_now(t-1), stop_acc, start_dec);
            total = total + scalar;
            for i = 1:size(A,2)
                next(i) = next(i) + scalar*coeff(i)*diff(i);
            end
        
        end
        these_frames = [these_frames; goal];
        frames = [frames; these_frames]
%         frames = [frames; goal]
%         total = total + in
    end
%     frames = [these_frames; goal]
    total = total + in
end
