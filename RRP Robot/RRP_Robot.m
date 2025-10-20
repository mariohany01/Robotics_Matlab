% Clear command window, workspace, and close all figures
clc         % Clear command window display
clear       % Clear all variables from workspace
close all   % Close all open figure windows
% Define link lengths for the Rhino XR1 robot (in appropriate units, likely cm or inches)
L1 = 50;     % Length of first link (base height)
L2 = 50;     % Length of second link (upper arm)

% Create Link using Denavit-Hartenberg parameters
% L = Link ([Theta d a alpha])      for Revolute
% L = Link ([Theta d a alpha ,'p']) for Prismatic
% Where: Theta = joint angle, d = link offset, a = link length, alpha = link twist
    
    L(1) = Link ([0 L1 0 pi/2]);    % Joint 1: Revolute base joint, height L1, 90° twist
    L(2) = Link ([0 0 L2 0]);       % Joint 2: Revolute shoulder joint, length L2, no twist
    % Changed the fifth joint from revolute to prismatic ('p')
    L(3) = Link ([0 0 0 0 1]);  % Joint 3: Prismatic wrist joint, offset is now controlled by joint variable
    L(3).qlim = [0 80];   % extension range

    % Create serial link robot object from the defined links
    Rob = SerialLink (L);
    
    % Assign name to the robot for identification
    Rob.name ='Rhino XR1';
    
    % Define joint angles for robot configuration (in radians)
    q1 = 0;     % Joint 1 angle
    q2 = 0;     % Joint 2 angle
    % q3 now represents a displacement for the prismatic joint
    q3 = 0;     % Joint 5 displacement (0 for no extension)
    
    % Plot the robot in 3D with the specified joint configuration
    Rob.plot ([q1,q2,q3])
    
    % Calculate and display forward kinematics (end effector pose)
    % Returns homogeneous transformation matrix for given joint angles
    Rob.fkine([q1,q2,q3])


%disp('Starting animation of the prismatic joint...');
%for L2 = 10 : 0.1 : 20
    % The plot function takes a vector of joint variables.
    % For our RRP robot, this vector should be [theta1, theta2, d3].
    % We are setting theta1=0 and theta2=0, and varying d3 (pr3).
%    Rob.plot([0 0 L2]);
    
    % Pause to allow the animation to be visible
%    pause(0.1); % Adjusted pause for a smoother animation
%end


Rob.teach
% You can also check the forward kinematics for a specific configuration
disp('Forward kinematics for a configuration with prismatic extension of 10:');
fk_matrix = Rob.fkine([0, 0, 10]);
disp(fk_matrix);