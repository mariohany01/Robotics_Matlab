%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% --- ROBOT PARAMETERS ---
% This robot is a cylindrical RPP (Revolute-Prismatic-Prismatic) type.
% It has a fixed vertical offset for its base.
base_height = 5; % The constant vertical distance (d1) from the base to the second joint.

%% --- DEFINE ROBOT LINKS using Denavit-Hartenberg (DH) Parameters ---
% We create each link specifying its properties clearly.

% Link 1: Revolute (R) - Rotates around the base Z-axis
% 'd' is the constant height offset. 'alpha' is 0 because the next Z-axis (z1)
% is parallel to the base Z-axis (z0).
L(1) = Link('d', base_height, 'a', 0, 'alpha', 0, 'revolute');
L(1).qlim = [-pi, pi]; % Set joint limit for rotation to +/- 180 degrees

% Link 2: Prismatic (P) - Moves vertically along the Z1-axis
% We add an 'alpha' of -90 degrees (-pi/2) to make the next joint's axis (z2)
% point horizontally (radially). 'prismatic' specifies the joint type.
L(2) = Link('theta', 0, 'a', 0, 'alpha', -pi/2, 'prismatic');
L(2).qlim = [2, 6];   % Set joint limits for vertical travel

% Link 3: Prismatic (P) - Moves radially along the Z2-axis
% 'alpha' is 0 as this is the end of the chain.
L(3) = Link('theta', 0, 'a', 0, 'alpha', 0, 'prismatic');
L(3).qlim = [2, 7];   % Set joint limits for radial travel

%% --- ASSEMBLE AND DISPLAY ROBOT ---
% Create the robot model from the links and give it a descriptive name.
Cylindrical_RPP_Robot = SerialLink(L, 'name', 'Cylindrical RPP');

% Display the robot's DH parameter table in the command window to verify.
disp('Robot DH Parameters:');
disp(Cylindrical_RPP_Robot);

%% --- FORWARD KINEMATICS ---
% Define the robot's joint variables for this specific pose.
theta1 = 30 * (pi/180); % Joint 1: 30 degrees rotation
d2 = 3;                 % Joint 2: 3 units of vertical extension
d3 = 4;                 % Joint 3: 4 units of radial extension

% Create the joint vector q
q = [theta1, d2, d3];

% Calculate the forward kinematics to get the end-effector's transformation matrix.
fprintf('\nCalculating FK for joint state q = [%.2f rad, %.2f, %.2f]\n', q(1), q(2), q(3));
T = Cylindrical_RPP_Robot.fkine(q);

% Display the resulting homogeneous transformation matrix.
disp('Resulting Transformation Matrix (T):');
disp(T);

%% --- VISUALIZATION ---
% Plot the robot in its defined pose 'q'.
Cylindrical_RPP_Robot.plot(q, 'workspace', [-10 10 -10 10 0 12], 'floorlevel', 0);

% Open the interactive slider GUI to teach the robot new poses.
Cylindrical_RPP_Robot.teach;


