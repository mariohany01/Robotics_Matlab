%% this is inverse kinematics Of the RPP robot solved with the Geometric Method (Corrected)
%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% 1. Robot and Target Parameters
%--------------------------------------------------------------------------
% Link parameters of the robot model.
% Note: This model assumes fixed offsets, which are slightly different from
% the DH parameter model in your first script. This IK is for THIS model.
link_a1 = 5; % Corresponds to d1, the base height
link_a2 = 2; % An additional vertical offset
link_a3 = 2; % A fixed radial offset (end-effector length)

% Target end-effector position vector [x; y; z]
target_position = [-3 ; 3*sqrt(3); 10];

%% 2. Inverse Kinematics Calculation (Geometric Method)
%--------------------------------------------------------------------------
disp('Calculating Inverse Kinematics...');

% Define target positions for clarity
Px = target_position(1);
Py = target_position(2);
Pz = target_position(3);

% --- Solve for d2 ---
% d2 is the variable vertical travel.
% Pz = link_a1 + d2 + link_a2
d2 = Pz - link_a1 - link_a2;

% --- Solve for d3 and theta1 ---
% First, calculate the total radial distance (R) in the XY plane.
R = sqrt(Px^2 + Py^2);

% The joint variable d3 is the total radial distance minus the fixed offset.
d3 = R - link_a3;

% --- CORRECTION ---
% Solve for theta1 using atan2.
% From FK: Px = -R * sin(theta1) and Py = R * cos(theta1)
% This means sin(theta1) = -Px/R and cos(theta1) = Py/R.
% The atan2 function is atan2(sin, cos), so we use atan2(-Px, Py).
% This is the key fix to get the correct angle and sign for the X position.
theta01_rad = atan2(-Px, Py);
theta01_deg = rad2deg(theta01_rad);

%% 3. Display the Results
%--------------------------------------------------------------------------
fprintf('\n--- Solution ---\n');
fprintf('Theta 1: %.2f degrees (%.4f radians)\n', theta01_deg, theta01_rad);
fprintf('D2:      %.2f Units\n', d2);
fprintf('D3:      %.2f Units\n', d3);
fprintf('----------------\n\n');

%% 4. Verification (Forward Kinematics)
%--------------------------------------------------------------------------
% Let's use the calculated joint variables to find the end-effector position
% and see if it matches our target. This is a crucial sanity check.
disp('Verifying solution with Forward Kinematics...');

% Use the same FK equations that the inverse kinematics were derived from.
final_x = -sin(theta01_rad) * (d3 + link_a3);
final_y =  cos(theta01_rad) * (d3 + link_a3);
final_z = link_a1 + d2 + link_a2;

fprintf('Target Position:     (%.3f, %.3f, %.3f)\n', Px, Py, Pz);
fprintf('Calculated Position: (%.3f, %.3f, %.3f)\n', final_x, final_y, final_z);

% Check if the calculated position is close to the target
error_margin = 1e-3; % A small tolerance for floating-point errors
if abs(final_x - Px) < error_margin && abs(final_y - Py) < error_margin && abs(final_z - Pz) < error_margin
    disp('Verification successful: The calculated joint variables achieve the target position.');
else
    disp('Verification failed: There is a discrepancy in the solution.');
end
