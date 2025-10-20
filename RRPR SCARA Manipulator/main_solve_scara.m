%% Inverse Kinematics Solver for a 4-DOF SCARA Robot
% This script defines the robot's geometry and a target pose,
% then calls a dedicated function to calculate the inverse kinematics.
%--------------------------------------------------------------------------

clc;            % Clear command window
clear;          % Clear workspace variables
close all;      % Close all figures

%% 1. Define Robot and Target Parameters
% Grouping parameters into structs makes the code cleaner.

% Robot link lengths and vertical offsets
robot.L1 = 100; % Height of the base column
robot.L2 = 50;  % Length of the first arm (in XY plane)
robot.L3 = 50;  % Length of the second arm (in XY plane)
robot.L4 = 40;  % Vertical length of the end-effector/tool
robot.L5 = 10;  % Vertical offset of the tool tip

% Target pose (position and orientation) for the end-effector
target.position = [43.3; 75; 70]; % [x; y; z] in units
target.rotation = [0.5,       sqrt(3)/2,  0;
                   sqrt(3)/2, 0.5,        0;
                   0,         0,         -1];

%% 2. Solve for Inverse Kinematics
% Call the solver function to get the joint configurations.
% The function returns multiple solutions (e.g., elbow up/down).
[solutions, isReachable, message] = solveScaraIK(robot, target);

%% 3. Display the Results
%--------------------------------------------------------------------------
fprintf('--- SCARA Inverse Kinematics Results ---\n');

if isReachable
    fprintf('%s\n\n', message);
    % Loop through all found solutions and display them
    for i = 1:length(solutions)
        fprintf('Solution %d (%s):\n', i, solutions(i).configuration);
        fprintf('  Theta 1 (Base):   %7.2f degrees\n', rad2deg(solutions(i).joints(1)));
        fprintf('  Theta 2 (Elbow):  %7.2f degrees\n', rad2deg(solutions(i).joints(2)));
        fprintf('  d3 (Prismatic): %7.2f units\n', solutions(i).joints(3));
        fprintf('  Theta 4 (Wrist):  %7.2f degrees\n', rad2deg(solutions(i).joints(4)));
        fprintf('\n');
    end
else
    % If the target is not reachable, display the error message.
    fprintf('Error: %s\n', message);
end