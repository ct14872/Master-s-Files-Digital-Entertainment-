%% Open Chain 3R, 3DOF 
% For the equations below I used this paper:
% www.seas.upenn.edu/~meam520/notes02/IntroRobotKinematics5.pdf
close all;
%start
p0 = [0 0];
% measured counter-clockwise
theta1 = 5;%0.175;
theta2 = 5;%0.275;
theta3 = 5;%0.175;
range_theta1 = [-45 45];
range_theta2 = [-90 90];% 180];
range_theta3 = [-170 170];
thetas = [theta1 theta2 theta3];
% thetas_init = thetas;

flex = [1 0.1 2];

% link length assumed to be positive
len1 = 0.3;
len2 = 0.3;
len3 = 0.3;

lens = [len1 len2 len3];

% MODE foe jacobian calculation: mode = 1 -> Pseudoinverse
%                                mode = 2 -> Damped Least Squares
mode = 1;

[x,y] = joints_pos(p0,thetas,lens);
init_plot = plot(x,y,'-bo','LineWidth',2, 'MarkerEdgeColor','black','MarkerFaceColor','r');
axis([-1,1,-1,1]);
grid on, hold on;
B = [];
check = 1;
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
draw_circle_boundaris(p0,max_stretch);
reach_plot = plot(B(:,1), B(:,2),'.','Color',[0 0 0]+0.7);uistack(reach_plot,'bottom');%'MarkerFaceColor','lightgray');
hold on;
goals_until_now = 0;
this_plot = zeros(100,1);
while 1
    [mx, my] = ginput(1);
    plot_circle = plot(mx,my,'ro','LineWidth',2);
    hold on;
    goal_position = [mx my]
    reachable = 0;
    
    dx = goal_position(1)-p0(1); % distance to goal on x_dimension
    dy = goal_position(2)-p0(2); % distance to goal on y_dimension
    
    dis_to_goal = norm(p0-goal_position);
    if max_stretch>= dis_to_goal
        reachable = 1
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
        if mode == 1
            J_pseudo_inv = J'*inv(J*J');
            change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
            change =  J_pseudo_inv*change_vec; % same as inv(jacobian)*change_vec (suggested by matlab)
        elseif mode == 2
            lambda = 2;
            I = [1 0; 0 1];
            J_damped = J'*inv(J*J' + lambda^2*I);
            change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
            change =  J_damped*change_vec; % same as inv(jacobian)*change_vec (suggested by matlab)
        elseif mode == 3
            J_pseudo_inv = J'*inv(J*J');
            z = flex.*((thetas-[5 5 5]).^2)
           
            change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
            beta = J_pseudo_inv*(change_vec+J*z')
            change = beta-z';%J_pseudo_inv(change_vec+J*z)-z;
%             change =  J_mode*change_vec; % same as inv(jacobian)*change_vec (suggested by matlab)
        
        end
        
        theta1 = theta1 + flex(1)*change(1);
        theta2 = theta2 + flex(2)*change(2);
        theta3 = theta3 + flex(3)*change(3);
        
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
        if mod(ctr,20)==0
            A = [A; thetas];
            check = 0;
        end
        if isclose(prev_thetas, thetas,4) 
            penalty = penalty +1;
        end

        % Using this to find the new distace to the goal_position
        [this_x,this_y] = FK(lens,thetas);

        dx = this_x - goal_position(1);
        dy = this_y - goal_position(2);
        size(A,1);
        ctr = ctr +1;

    end
    
    A = [A; thetas];
    
    if reachable == 1
        frames = sin_interp(A,0.3,0.7);
        if goals_until_now == 0
            set(init_plot,'Visible','off')
        else
            set(this_plot(100),'Visible','off');
        end

        for i=1:size(frames,1)
            if i ~=1
                set(this_plot(i-1),'Visible','off')
            end
            [x,y] = joints_pos(p0,frames(i,:),lens);
            this_plot(i) = plot(x,y,'-bo','LineWidth',2, 'MarkerEdgeColor','black','MarkerFaceColor','r');hold on;
            pause(0.01)
        end
        goals_until_now = goals_until_now +1;
    else
        continue
    end
end

%%
 t = 0.01:0.01:1
 a = zeros(length(t),1);
 size(a)
 for i = 2:length(t)%0.01:0.01:1
     a(i-1) = ease(t(i),0,0.9)-ease(t(i-1),0.0,0.9);
     
 end
a = [a; ease(0.01,0,0.9)]
 sum(a)
%% Open Chain 3R, 3DOF 
% For the equations below I used this paper:
% www.seas.upenn.edu/~meam520/notes02/IntroRobotKinematics5.pdf
close all;
%start
p0 = [0 0];
% measured counter-clockwise
theta1 = 5;%0.175;
theta2 = 5;%0.275;
theta3 = 5;%0.175;
range_theta1 = [-45 45];
range_theta2 = [-90 90];% 180];
range_theta3 = [-170 170];
thetas = [theta1 theta2 theta3];
thetas_init = thetas;

flex = [1 1 1];

% link length assumed to be positive
len1 = 0.3;
len2 = 0.4;
len3 = 0.3;

lens = [len1 len2 len3];

% MODE foe jacobian calculation: mode = 1 -> Pseudoinverse
%                                mode = 2 -> Damped Least Squares
mode = 1;

[x,y] = joints_pos(p0,thetas,lens);
plot(x,y,'-o','LineWidth',3);
axis([-1,1,-1,1]);
grid on, hold on;
B = [];
check = 1;
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
% plot(B(:,1), B(:,2),'r.');
% hold on;

while 1
    [mx, my] = ginput(1);
    plot(mx,my,'r*');
    hold on;
    %pause(0.1);
    goal_position = [mx my]
    %patch(mx,my,'m')
    reachable = 0;
    if(goal_position(1)<max(B(:,1)) && goal_position(1)>min(B(:,1)) && goal_position(2)<max(B(:,2)) && goal_position(2)>min(B(:,2)))
        reachable = 1
    end
    
    dx = goal_position(1)-p0(1); % distance to goal on x_dimension
    dy = goal_position(2)-p0(2); % distance to goal on y_dimension
    
    dis_to_goal = norm(p0-goal_position)
%     extension = sum(lens)
    A = thetas;
    ctr = 0;
    penalty = 0;
    while  reachable ==1 &&penalty ~= 100 && (abs(dx)>0.01 || abs(dy) > 0.01)%~=0 && dy~=0%(abs(dx) > 0.002 || abs(dy) > 0.002)
        cla
        prev_thetas = thetas;
        % Jacobian Matrix elements (derivatives)
        J = comp_jacob(thetas,lens);

        [this_x,this_y] = FK(lens,thetas);
        if mode == 1 %pseudoinverse
                J_pseudo_inv = J'*inv(J*J');
                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                % thetadot
                change =  J_pseudo_inv*change_vec; 
            elseif mode == 2 %damped least squares
                lambda = 2;
                I = [1 0; 0 1];
                J_damped = J'*inv(J*J' + lambda^2*I);
                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                % thetadot
                change =  J_damped*change_vec; 
            elseif mode == 3
                J_pseudo_inv = J'*inv(J*J');
                z = flex.*((thetas-[5 5 5]).^2);

                change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
                beta = J_pseudo_inv*(change_vec+J*z');
                % thetadot
                change = beta-z';%J_pseudo_inv(change_vec+J*z)-z;
    %             change =  J_mode*change_vec; % same as inv(jacobian)*change_vec (suggested by matlab)

            end

        change_vec = [goal_position(1)-this_x; goal_position(2)-this_y];
        change =  J_mode*change_vec; % same as inv(jacobian)*change_vec (suggested by matlab)
        
        theta1 = theta1 + flex(1)*change(1);
        theta2 = theta2 + flex(2)*change(2);
        theta3 = theta3 + flex(3)*change(3);
        
        %Clamping angles to their stay in their range of angles
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
        thetas = [mod(theta1,360) mod(theta2,360) mod(theta3,360)];%thetas = [theta1 theta2 theta3];
        if mod(ctr,10)==0
            A = [A; thetas];
            check = 0;
        end
        if isclose(prev_thetas, thetas,4) 
            penalty = penalty +1;
        end
%         [x,y] = joints_pos(p0,thetas,lens);

%         plot(x,y,'-o','LineWidth',3);
%         pause(0.01)

        % Using this to find the new distace to the goal_position
        [this_x,this_y] = FK(lens,thetas);

        dx = this_x - goal_position(1);
        dy = this_y - goal_position(2);
        size(A,1);
        ctr = ctr +1;
        if reachable ~=1 || penalty == 100 || ((abs(dx)==0.001 && abs(dy) == 0.001))
           A = [A; thetas];
        end
    end
    if reachable == 1
        frames = sin_interp(A,0.3,0.7)

        for i=1:size(frames,1)
            
            [x,y] = joints_pos(p0,frames(i,:),lens);
            plot(x,y,'x','LineWidth',0.5, 'MarkerEdgeColor','black','MarkerFaceColor','r');
            pause(0.001)
        end
    else
        continue
    end
%     [x,y] = joints_pos(p0,thetas,lens);
%     plot(x,y,'-o','LineWidth',3);
end