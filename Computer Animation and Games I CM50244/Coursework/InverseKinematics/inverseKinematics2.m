function inverseKinematics2(p0,thetas,lens,rate,J_mode,interp_mode,sa,sd,ignore,goal_mode)
    % For the equations below I used this paper:
    % www.seas.upenn.edu/~meam520/notes02/IntroRobotKinematics5.pdf
    % www.seas.upenn.edu/~meam520/notes02/IntroRobotKinematics5.pdf
    % and Rick Parent's book for Computer Animation
    
    close all;

    theta1 = thetas(1);
    theta2 = thetas(2);
    theta3 = thetas(3);
    
    % Boundaries of angle constraints
    range_theta1 = [-45 45];
    range_theta2 = [-90 90];
    range_theta3 = [-170 170];

    [x,y] = joints_pos(p0,thetas,lens);
    init_plot = plot(x,y,'-bo','LineWidth',3, 'MarkerEdgeColor','black','MarkerFaceColor','r');
    init_t = text(x(end),y(end),['  (', num2str(x(end),3), ', ', num2str(y(end),3), ')']);
    axis([-1,1,-1,1]);
    grid on, hold on;
    if goal_mode == 2
        set(gcf,'WindowButtonMotionFcn',@mouseMove);
    end
    B = [];

    step = 10;
    max_stretch = 0;
    
    for th1 = min(range_theta1):step:max(range_theta1)
        for th2 = min(range_theta2):step:max(range_theta2)
            for th3 = min(range_theta3):step:max(range_theta3)
                ths = [th1 th2 th3];
                [m,n] = FK(lens,ths);
                B = [B;[m n]];
                this_max_stretch = norm(p0-[m,n]);
                if this_max_stretch>max_stretch
                    max_stretch = this_max_stretch;
                end
            end
        end
    end
    
    draw_circle_boundaries(p0,max_stretch);
    reach_plot = plot(B(:,1), B(:,2),'.','Color',[0 0 0]+0.7);uistack(reach_plot,'bottom');%'MarkerFaceColor','lightgray');
    hold on;
    
    goals_until_now = 0;
    this_plot = zeros(100,1);
    t = zeros(100,1);
    
    while 1
        if goal_mode == 1
            [mx, my] = ginput(1);
        elseif goal_mode ==2
            loc = get(gca, 'CurrentPoint');
            pause(0.01);
            mx = loc(1,1);
            my = loc(2,2);
        end
        
        plot_circle = plot(mx,my,'ro','LineWidth',2);
        hold on;
        goal_position = [mx my];
        reachable = 0;

        dx = goal_position(1)-p0(1); % distance to goal on x_dimension
        dy = goal_position(2)-p0(2); % distance to goal on y_dimension

        dis_to_goal = norm(p0-goal_position);
        if max_stretch>= dis_to_goal
            reachable = 1;
        else
            continue
        end
        
        A = thetas;
        ctr = 0;
        penalty = 0;
        
        while penalty ~= 100 && (abs(dx)>0.001 || abs(dy) > 0.001)%~=0 && dy~=0%(abs(dx) > 0.002 || abs(dy) > 0.002)

            prev_thetas = thetas;
            % Jacobian Matrix elements (derivatives)
            J = comp_jacob(thetas,lens);
            [this_x,this_y] = FK(lens,thetas);
            
            %Which Jacobian method is used
            if J_mode == 1 %pseudoinverse
                J_pseudo_inv = J'*inv(J*J');
                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                % thetadot
                change =  J_pseudo_inv*change_vec; 
            elseif J_mode == 2 %damped least squares
                lambda = 2;
                I = [1 0; 0 1];
                J_damped = J'*inv(J*J' + lambda^2*I);
                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                % thetadot
                change =  J_damped*change_vec; 
            elseif J_mode == 3
                % Adding more control method. UPDATE: Not working properly
                J_pseudo_inv = J'*inv(J*J');
                z = zeros(size(thetas));
                for i = 1:length(thetas)
                    z(i) = rate(i)*((thetas(i)-mean(range_theta1))^2);
                end

                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                I = [1 0 0; 0 1 0; 0 0 1];
                change = J_pseudo_inv*change_vec +(J_pseudo_inv*J - I)*z';
                change = mod(change,360);%';%J_pseudo_inv(change_vec+J*z)-z;
            end

            theta1 = theta1 + rate(1)*change(1);
            theta2 = theta2 + rate(2)*change(2);
            theta3 = theta3 + rate(3)*change(3);

            %Clamping angles so they stay in their range of angles
            if theta1>max(range_theta1)
                theta1 = max(range_theta1);
            elseif theta1<min(range_theta1)
                theta1 = min(range_theta1);
            end
            if theta2>max(range_theta2)
                theta2 = max(range_theta2);
            elseif theta2<min(range_theta2)
                theta2 = min(range_theta2);
            end
            if theta3>max(range_theta3)
                theta3 = max(range_theta3);
            elseif theta3<min(range_theta3)
                theta3 = min(range_theta3);
            end
            
            thetas = [theta1 theta2 theta3];

            if mod(ctr,ignore)==0
                A = [A; thetas];
            end
            if J_mode ~= 2
                if isclose(prev_thetas, thetas,4) 
                    penalty = penalty +1;
                end
            else
                if isclose(prev_thetas, thetas,8) 
                    penalty = penalty +1;
                end
                
            end

            % Using this to find the new distace to the goal_position
            [this_x,this_y] = FK(lens,thetas);

            dx = this_x - goal_position(1);
            dy = this_y - goal_position(2);
            size(A,1);
            ctr = ctr +1;

        end
        thetas
        A = [A; thetas];
        frames = [];
        if reachable == 1
            % Which interpolation method is used
            if interp_mode == 0
                frames = A;
            elseif interp_mode == 1
                frames = sin_interp(A,sa,sd);
            elseif interp_mode == 2
                frames = sin_interp2(A,sa,sd);
            end
            if goals_until_now == 0
                set(init_plot,'Visible','off')
                set(init_t,'Visible','off')
            else
                cla
                draw_circle_boundaries(p0,max_stretch);
                reach_plot = plot(B(:,1), B(:,2),'.','Color',[0 0 0]+0.7);uistack(reach_plot,'bottom');%'MarkerFaceColor','lightgray');
                plot_circle = plot(mx,my,'ro','LineWidth',2);
                %                 set(this_plot(size(frames,1)),'Visible','off');
%                 set(t(size(frames,1)),'Visible','off');
            end

            for i=1:size(frames,1)
                if i ~=1
                    set(this_plot(i-1),'Visible','off')
                    set(t(i-1),'Visible','off')
                end
                [x,y] = joints_pos(p0,frames(i,:),lens);
                this_plot(i) = plot(x,y,'-bo','LineWidth',3, 'MarkerEdgeColor','black','MarkerFaceColor','r');hold on;
                t(i) = text(x(end),y(end),['  (', num2str(x(end),3), ', ', num2str(y(end),3), ')']);
                pause(0.01)
            end
            set(plot_circle,'Visible','off')
            goals_until_now = goals_until_now +1;
        else
            continue
        end
    end

end