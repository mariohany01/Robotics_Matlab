%==========================================================================
% Inverse Kinematics for a 2-Link Planar Manipulator
%==========================================================================
% This script calculates the joint angles (theta1, theta2) for a 2R robot
% given a desired end-effector position and orientation.

%% 1. Initialization and Setup
%--------------------------------------------------------------------------
clc;    % Clear the command window
clear;  % Clear all variables from the workspace
close all; % Close all figure windows

%% 2. Robot and Target Parameters
%--------------------------------------------------------------------------
% Link lengths of the robot
link_a1 = 2;
link_a2 = 2;

% Target end-effector position vector [x; y; z]
target_position = [2.249; 2.931; 0];

% Target end-effector rotation matrix
target_rotation = [0.25819, -0.9659,  0;
                   0.9659,   0.25819,  0;
                   0,        0,        1];

%% 3. Inverse Kinematics Calculation (Geometric Method)
%--------------------------------------------------------------------------
disp('Calculating Inverse Kinematics...');

% Step 1: Extract the end-effector orientation angle (phi) from the rotation matrix.
% The atan2 function is used to ensure the angle is in the correct quadrant.
phi_rad = atan2(target_rotation(2,1), target_rotation(1,1));

% Step 2: Calculate the position of the wrist (joint between link 1 and 2).
% This is found by starting at the end-effector and moving backward along link 2.
wrist_x = target_position(1) - link_a2 * cos(phi_rad);
wrist_y = target_position(2) - link_a2 * sin(phi_rad);

% Step 3: Calculate the angle of the first link (theta1).
% This angle positions the wrist at the calculated (wx, wy) coordinates.
theta1_rad = atan2(wrist_y, wrist_x);

% Step 4: Calculate the angle of the second link (theta2).
% This is the difference between the final orientation and the first link's angle.
theta2_rad = phi_rad - theta1_rad;

%% 4. Convert Results to Degrees for Readability
%--------------------------------------------------------------------------
theta1_deg = rad2deg(theta1_rad);
theta2_deg = rad2deg(theta2_rad);

%% 5. Display the Results
%--------------------------------------------------------------------------
fprintf('\n--- Solution ---\n');
fprintf('Theta 1: %.2f degrees (%.4f radians)\n', theta1_deg, theta1_rad);
fprintf('Theta 2: %.2f degrees (%.4f radians)\n', theta2_deg, theta2_rad);
fprintf('----------------\n\n');

%% 6. Verification (Forward Kinematics)
%--------------------------------------------------------------------------
% Let's use the calculated angles to find the end-effector position
% and see if it matches our target. This is a crucial sanity check.
disp('Verifying solution with Forward Kinematics...');

final_x = link_a1 * cos(theta1_rad) + link_a2 * cos(theta1_rad + theta2_rad);
final_y = link_a1 * sin(theta1_rad) + link_a2 * sin(theta1_rad + theta2_rad);

fprintf('Target Position: (%.3f, %.3f)\n', target_position(1), target_position(2));
fprintf('Calculated Position: (%.3f, %.3f)\n', final_x, final_y);

% Check if the calculated position is close to the target
error_margin = 1e-3; % A small tolerance for floating-point errors
if abs(final_x - target_position(1)) < error_margin && abs(final_y - target_position(2)) < error_margin
    disp('Verification successful: The calculated angles achieve the target position.');
else
    disp('Verification failed: There is a discrepancy in the solution.');
end