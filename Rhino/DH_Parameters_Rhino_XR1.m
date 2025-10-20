% Clear command window, workspace, and close all figures
clc         % Clear command window display
clear       % Clear all variables from workspace
close all   % Close all open figure windows

% Define link lengths for the Rhino XR1 robot (in appropriate units, likely cm or inches)
L1 = 10;     % Length of first link (base height)
L2 = 10;     % Length of second link (upper arm)
L3 = 10;     % Length of third link (forearm)
L5 = 10;     % Length of fifth link (end effector offset)

% Create Link using Denavit-Hartenberg parameters
% L = Link ([Theta d a alpha])      for Revelute
% L = Link ([Theta d a alpha ,'p']) for Prismatic
% Where: Theta = joint angle, d = link offset, a = link length, alpha = link twist
    
    L(1) = Link ([0 L1 0 pi/2]);    % Joint 1: Revolute base joint, height L1, 90° twist
    L(2) = Link ([0 0 L2 0]);       % Joint 2: Revolute shoulder joint, length L2, no twist
    L(3) = Link ([0 0 L3 0]);       % Joint 3: Revolute elbow joint, length L3, no twist
    L(4) = Link ([0 0 0 pi/2]);     % Joint 4: Revolute wrist pitch joint, 90° twist
    L(5) = Link ([0 L5 0 0 1]);       % Joint 5: Revolute wrist roll joint, offset L5
    L(5).qlim = [10 20];   % extension range
    
    % Create serial link robot object from the defined links
    Rob = SerialLink (L);
    
    % Assign name to the robot for identification
    Rob.name ='Rhino XR1';
    
    % Define joint angles for robot configuration (in radians)
    q1 = 0;     % Joint 1 angle: 90 degrees converted to radians
    q2 = 0;     % Joint 2 angle: 0 radians (straight)
    q3 = 0;     % Joint 3 angle: 0 radians (straight)
    q4 = 0;     % Joint 4 angle: 0 radians (straight)
    q5 = 0;     % Joint 5 angle: 0 radians (straight)
    
    % Plot the robot in 3D with the specified joint configuration
    Rob.plot ([q1,q2,q3,q4,q5])
    
    % Calculate and display forward kinematics (end effector pose)
    % Returns homogeneous transformation matrix for given joint angles
    Rob.fkine([q1,q2,q3,q4,q5])


%write ROB in command window to show Robot
%{
for th1 = 0 : 0.1 : pi/2
    Rob.plot([th1 0 0 0 0]);
    pause(0.25);
end

for th2 = 0 : 0.1 : pi/4
    Rob.plot([pi/2 th2 0 0 0]);
    pause(0.25);
end

for th3 = 0 : 0.1 : pi/4
    Rob.plot([pi/2 pi/4 0 th3 0]);
    pause(0.25);
end
%}


Rob.teach
