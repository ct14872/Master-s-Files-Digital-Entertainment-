%% Main function. Where this program starts.

% IK_MODE for mode of IK solution: IK_MODE = 1 -> without constraints
%                                  IK_MODE = 2 -> with constraints
IK_mode = 1;

% GOAL_MODE for mode of goal position capturing: GOAL_MODE = 1 -> mouse click
%                                                GOAL_MODE = 2 -> cursor follow
%                                                
goal_mode = 1;

% J_MODE foR jacobian calculation: J_MODE = 1 -> Pseudoinverse
%                                  J_MODE = 2 -> Damped Least Squares
J_mode = 1;

% INTERP_MODE for interpolation method: INTERP_MODE = 0 -> no interpolation
% Sine Interpolation                    INTERP_MODE = 1 -> 2 frame interp
%                                       INTERP_MODE = 2 -> 4 frame interp
interp_mode =1;

% The variables below are used to tell the interpolation method when to
% make the animation stop accelerating and when to start decelerating
% respectively

%Recommended for interp_mode = 1
stop_acc = 0.3;
start_dec = 0.7;

%Recommended for interp_mode = 2
% stop_acc = 0.4;
% start_dec = 0.8;

%How many frames to ignore before storing one.
ignore = 10;

p0 = [0 0];

% measured counter-clockwise
theta1 = 5;%0.175;
theta2 = 5;%0.275;
theta3 = 5;%0.175;

thetas = [theta1 theta2 theta3];

% Rate of change for each joint angle
rate = [1 1 1];

% link length assumed to be positive
len1 = 0.3;
len2 = 0.3;
len3 = 0.3;

lens = [len1 len2 len3];

% The parameters are all set and the program will execute
if IK_mode == 1 
    inverseKinematics1(p0,thetas,lens,rate,J_mode,interp_mode,stop_acc, start_dec,ignore,goal_mode);
elseif IK_mode == 2
    inverseKinematics2(p0,thetas,lens,rate,J_mode,interp_mode,stop_acc, start_dec,ignore,goal_mode);
end

