
To run the code, please proceed to main.m and using the keyboard press CTRL + ENTER or press the Run button on MATLAB interface.

Unfortunately I was unable to produce a mechanism that would shut the program down, therefore you are advised to press CTRL + C whenever you wish to stop
the procedure or to change the parameters to be used which will be located in the main.m file.

The changeable parameters passed are as follows, and also described appended as comments in the main.m file:

1.  % IK_MODE for mode of IK solution: IK_MODE = 1 -> without constraints
	%                                  IK_MODE = 2 -> with constraints
	
	This will change the inverse kinematics file to be used. With ik_mode you can change between 1, with constraints or 2, without constraints.

2 	% GOAL_MODE for mode of goal position capturing: GOAL_MODE = 1 -> mouse click
	%                                                GOAL_MODE = 2 -> cursor follow 
	
	This will allow the user to choose whether to set the goal position with a mouse click, 1, or have the goal position change with cursor location, 2.

3.  % J_MODE for jacobian calculation: J_MODE = 1 -> Pseudoinverse
	%                                  J_MODE = 2 -> Damped Least Squares
	
	This parameter will change the method used to compile the JACOBIAN matrix, setting it to one will proceed using the pseudoinverse,
	changing this to 2 will use the damped least-squares method.

4. 	% INTERP_MODE for interpolation method: INTERP_MODE = 0 -> no interpolation
	% Sine Interpolation                    INTERP_MODE = 1 -> 2 frame interp
	%                                       INTERP_MODE = 2 -> 4 frame interp
	
	The interpolation used is sine interpolation. Choosing interp_mode to be 0 will not produce any kind of interpolation and instead show all frames acquired,
	changing this to 1 will interpolate using to frames, start and end frame, and changing this to 2 will use 4 keyframe interpolation.

5.	%Recommended for interp_mode = 1
	stop_acc = 0.3;
	start_dec = 0.7;
	
	%Recommended for interp_mode = 2
	% stop_acc = 0.4;
	% start_dec = 0.8;
	
	This variables will decide the time at which the sine interpolation will produce the acceleration and deceleration of movement. Reccommendations are provided.
	
6.  %How many frames to ignore before storing one.
	ignore = 10;
	
	The parameter ignore will store frames with index divisible by this value. In this case the frames stored are multiples of 10.

7.  % Where the linkage is attached on the grid
	p0 = [0 0]; % The linkage is attached on (0,0)

8.	% measured counter-clockwise
	theta1 = 5;
	theta2 = 5;
	theta3 = 5;
	thetas = [theta1 theta2 theta3];
	
	Starting orientation of the joints. Setting all to 0 might produce unexpected failures.

9.  % Rate of change for each joint angle
	rate = [1 1 1];
	
	The values stored in rate are the rate of change of each angles individually. Set to [1 1 1] as default.
	
If you have any questions please send your queries to cct50@bath.ac.uk. Alternatively, the demo will provide more insight.


Thank you and have a nice day,
Constantinos Theophilou.
MSc Digital Entertainment