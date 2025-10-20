%% this is inverse kinematics Of the SCARA solved with the forward Kinematics Method
%% --- SETUP ---
%--------------------------------------------------------------------------
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures
%% 1. Robot and Target Parameters
%--------------------------------------------------------------------------
% Link lengths of the robot
link_a1 = 100;
link_a2 = 50;
link_a3 = 50;
link_a4 = 40;
link_a5 = 10;

% Target end-effector position vector [x; y; z]
target_position = [43.3 ; 75 ; 70];

% Target end-effector rotation matrix
target_rotation = [0.5,         sqrt(3)/2,  0;
                   sqrt(3)/2,   0.5,        0;
                   0,           0,          -1];
%% 2. Inverse Kinematics Calculation (Geometric Method)
%--------------------------------------------------------------------------
disp('Calculating Inverse Kinematics...');

numerator_th_01 = target_position(1)^2 + target_position(2)^2 - link_a3^2 - link_a2^2;
denominator_th_01 = 2 * link_a2 * link_a3;
theta_01_deg = acosd(numerator_th_01 / denominator_th_01);
theta_01_rad=deg2rad(theta_01_deg);

cos_theta01_deg = cosd(theta_01_deg);
sin_theta01_deg = sind(theta_01_deg);


k1=((link_a3*cos_theta01_deg)+link_a2);
k2=(link_a3*sin_theta01_deg);

beta=atan2(k2,k1);

theta_00=atan2(target_position(2),target_position(1))-beta;

theta_00_deg=rad2deg(theta_00);

theta_02_deg = theta_00_deg + theta_01_deg - acosd(target_rotation(1,1));

d2=target_position(3)-link_a1+link_a5+link_a4;

%% 3. Display the Results
%--------------------------------------------------------------------------
fprintf('\n--- Solution ---\n');
fprintf('Theta 0: %.2f degrees \n', theta_00_deg);
fprintf('Theta 1: %.2f degrees \n', theta_01_deg);
fprintf('Theta 2: %.2f degrees \n', theta_02_deg);
fprintf('D3: %.2f Units\n', d2);

