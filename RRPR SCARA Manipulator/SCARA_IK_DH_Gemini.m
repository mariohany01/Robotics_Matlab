%% Inverse Kinematics for a SCARA Robot (Both Geometric Solutions)
% This script calculates the two possible sets of joint variables for a
% SCARA robot to reach a specified target end-effector pose.

%% --- SETUP ---
%--------------------------------------------------------------------------
clc;            % Clear the command window
clear;          % Clear all variables from the workspace
close all;      % Close all open figures

%% 1. Robot and Target Parameters
%--------------------------------------------------------------------------
% Link lengths of the SCARA robot (e.g., in mm)
link_a1 = 100;
link_a2 = 50;
link_a3 = 50;
link_a4 = 40;
link_a5 = 10;

% Target pose for the end-effector
target_position = [43.3; 75; 70]; % Target position [x; y; z]
target_rotation = [0.5,         sqrt(3)/2,  0;
                   sqrt(3)/2,   0.5,        0;
                   0,           0,          -1]; % Target orientation

%% 2. Inverse Kinematics Calculation
%--------------------------------------------------------------------------
disp('Calculating Inverse Kinematics for Both Solutions...');

% Extract target coordinates for clarity
px = target_position(1);
py = target_position(2);
pz = target_position(3);

%% --- Shared Calculations ---
% These values are the same for both solutions
d3 = pz - link_a1 + link_a5 + link_a4; % d3 is independent of the elbow config

numerator_th1 = px^2 + py^2 - link_a2^2 - link_a3^2;
denominator_th1 = 2 * link_a2 * link_a3;
orientation_phi_deg = acosd(target_rotation(1,1));
%% --- SOLUTION 1 (Elbow-Out, positive angle for theta1) ---
theta1_sol1_deg = acosd(numerator_th1 / denominator_th1);

% Recalculate dependent angles for Solution 1
k1_sol1 = link_a2 + link_a3 * cosd(theta1_sol1_deg);
k2_sol1 = link_a3 * sind(theta1_sol1_deg);
theta0_sol1_deg = atan2d(py, px) - atan2d(k2_sol1, k1_sol1);
theta2_sol1_deg = theta0_sol1_deg + theta1_sol1_deg - orientation_phi_deg;

%% --- SOLUTION 2 (Elbow-In, negative angle for theta1) ---
theta1_sol2_deg = -acosd(numerator_th1 / denominator_th1);

% Recalculate dependent angles for Solution 2
k1_sol2 = link_a2 + link_a3 * cosd(theta1_sol2_deg);
k2_sol2 = link_a3 * sind(theta1_sol2_deg);
theta0_sol2_deg = atan2d(py, px) - atan2d(k2_sol2, k1_sol2);
theta2_sol2_deg = theta0_sol2_deg + theta1_sol2_deg - orientation_phi_deg;


%% 3. Display the Results
%--------------------------------------------------------------------------
% Display Solution 1
fprintf('\n--- Solution 1 (Elbow-Out) ---\n');
fprintf('Theta 0: %.2f degrees \n', theta0_sol1_deg);
fprintf('Theta 1: %.2f degrees \n', theta1_sol1_deg);
fprintf('Theta 2: %.2f degrees \n', theta2_sol1_deg);
fprintf('D3:      %.2f Units\n', d3);

% Display Solution 2
fprintf('\n--- Solution 2 (Elbow-In) ---\n');
fprintf('Theta 0: %.2f degrees \n', theta0_sol2_deg);
fprintf('Theta 1: %.2f degrees \n', theta1_sol2_deg);
fprintf('Theta 2: %.2f degrees \n', theta2_sol2_deg);
fprintf('D3:      %.2f Units\n', d3);