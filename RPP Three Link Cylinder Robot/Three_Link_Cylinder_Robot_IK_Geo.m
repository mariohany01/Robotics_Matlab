%% this is inverse kinematics Of the RPP robot solved with the forward Kinematics Method
%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% 1. Robot and Target Parameters
%--------------------------------------------------------------------------
% Link lengths of the robot
link_a1 = 5;
link_a2 = 2;
link_a3 = 2;

% Target end-effector position vector [x; y; z] 
target_position = [-3 ; 3*sqrt(3); 10];

% Target end-effector rotation matrix
target_rotation = [sqrt(3)/2, 0,  -0.5;
                   0.5,   0,  sqrt(3)/2;
                   0,        -1,        0];
%% 2. Inverse Kinematics Calculation (Geometric Method)
%--------------------------------------------------------------------------
disp('Calculating Inverse Kinematics...');
d2=target_position(3)-link_a1-link_a2;

d3=sqrt((target_position(1)^2)+(target_position(2)^2))-link_a3;

cos_theta01 = target_position(2)/d3;
sin_theta01 = target_position(1)/d3;

%hena hot el negative di ashan el Axis negative
theta01_rad = atan2(-sin_theta01 , cos_theta01);

theta01_deg = rad2deg(theta01_rad);


%% 3. Display the Results
%--------------------------------------------------------------------------
fprintf('\n--- Solution ---\n');
fprintf('Theta 1: %.2f degrees\n', theta01_deg);
fprintf('D2: %.2f Units\n', d2);
fprintf('D3: %.2f Units\n', d3);
fprintf('----------------\n\n');

%% 6. Verification (Forward Kinematics)
%--------------------------------------------------------------------------
% Let's use the calculated angles to find the end-effector position
% and see if it matches our target. This is a crucial sanity check.
disp('Verifying solution with Forward Kinematics...');

final_x = -sin(theta01_rad)*(d3+link_a3);
final_y = cos(theta01_rad)*(d3+link_a3);
final_z = link_a1+d2+link_a2;

fprintf('Target Position: (%.3f, %.3f,%.3f)\n', target_position(1), target_position(2), target_position(3));
fprintf('Calculated Position: (%.3f, %.3f, %.3f)\n', final_x, final_y, final_z);

% Check if the calculated position is close to the target
error_margin = 1e-3; % A small tolerance for floating-point errors

if abs(final_x - target_position(1)) < error_margin && abs(final_y - target_position(2)) < error_margin && abs(final_z - target_position(3)) < error_margin
    disp('Verification successful: The calculated angles achieve the target position.');
else
    disp('Verification failed: There is a discrepancy in the solution.');
end




