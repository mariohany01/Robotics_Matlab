%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures
%% --- ROBOT PARAMETERS ---
% This robot is a cylindrical RPP (Revolute-Prismatic-Prismatic) type.
base_height = 5; % The constant vertical distance (d1) from the base to the second joint.
%% --- DEFINE ROBOT LINKS using Denavit-Hartenberg (DH) Parameters ---
% Link 1: Revolute (R) - Rotates around the base Z-axis
L(1) = Link('d', base_height, 'a', 0, 'alpha', 0, 'revolute');
L(1).qlim = [-pi, pi]; % Set joint limit for rotation
% Link 2: Prismatic (P) - Moves vertically along the Z1-axis
% Alpha is -90 degrees to align the Z2 axis radially (horizontally).
L(2) = Link('theta', 0, 'a', 0, 'alpha', -pi/2, 'prismatic');
L(2).qlim = [2, 6];   % Set joint limits for vertical travel (d2)
% Link 3: Prismatic (P) - Moves radially along the Z2-axis
L(3) = Link('theta', 0, 'a', 0, 'alpha', 0, 'prismatic');
L(3).qlim = [2, 7];   % Set joint limits for radial travel (d3)
%% --- ASSEMBLE AND DISPLAY ROBOT ---
Cylindrical_RPP_Robot = SerialLink(L, 'name', 'Cylindrical RPP');
disp('Robot DH Parameters:');
disp(Cylindrical_RPP_Robot);

%% --- DEFINE JOINT STATE (q) FOR ANALYSIS ---
% Define a starting joint state (q_start)
theta1_start = 30 * (pi/180); % Joint 1: 30 degrees rotation
d2_start = 3;                 % Joint 2: 3 units of vertical extension
d3_start = 4;                 % Joint 3: 4 units of radial extension
q_start = [theta1_start, d2_start, d3_start];

fprintf('\n--- Analysis Starting at Joint State q = [%.2f rad, %.2f, %.2f] ---\n', q_start(1), q_start(2), q_start(3));

% Calculate the initial forward kinematics (FK)
T = Cylindrical_RPP_Robot.fkine(q_start);
disp('Initial End-Effector Pose (T):');
disp(T);

%% --- JACOBIAN MATRIX CALCULATION (Initial State) ---
J_start = Cylindrical_RPP_Robot.jacob0(q_start);
disp('Initial 6x3 Jacobian Matrix J (Referenced to Base Frame {0}):');
disp(J_start);

%% --- JACOBIAN SINGULARITY CHECK (Initial State) ---
J_pos = J_start(1:3, 1:3);
fprintf('Initial Determinant of J_pos: %.4f\n', det(J_pos));
fprintf('Initial Condition Number of J_pos: %.4f\n', cond(J_pos));

%% --- TRAJECTORY DEFINITION (Fast Start, Slow End) ---
% Define an end joint state (q_end) for the movement
theta1_end = -90 * (pi/180); % Joint 1: -90 degrees rotation
d2_end = 5.5;                % Joint 2: 5.5 units vertical extension
d3_end = 2.5;                % Joint 3: 2.5 units radial extension
q_end = [theta1_end, d2_end, d3_end];

% --- Phase 1: FAST MOVEMENT (80% of distance in 10 steps) ---
mid_ratio = 0.80; % Target 80% of the distance covered
% Calculate the joint waypoint that is 80% of the way to the end
q_mid = q_start + mid_ratio * (q_end - q_start); 

T_steps_fast = 10; % Only 10 steps for the initial fast movement
% Generate a smooth trajectory from start to midpoint
q_trajectory_fast = jtraj(q_start, q_mid, T_steps_fast);

% --- Phase 2: SLOW MOVEMENT (Remaining 20% of distance in 20 steps, making it faster) ---
T_steps_slow = 20; % REDUCED from 40 to 20 steps
% Generate a smooth trajectory from midpoint to end
q_trajectory_slow = jtraj(q_mid, q_end, T_steps_slow); 
% Remove the first point of the slow trajectory as it duplicates q_mid
q_trajectory_slow = q_trajectory_slow(2:end, :); 

% Combine the trajectories
q_trajectory = [q_trajectory_fast; q_trajectory_slow];
T_steps = size(q_trajectory, 1); % Update total steps
fprintf('Trajectory split: %d fast steps, %d slow steps. Total steps: %d.\n', T_steps_fast, size(q_trajectory_slow, 1), T_steps);

%% --- VISUALIZATION AND DYNAMIC JACOBIAN DISPLAY ---
figure(1);
Cylindrical_RPP_Robot.plot(q_start, 'workspace', [-10 10 -10 10 0 12], 'floorlevel', 0);
title('Cylindrical RPP Robot Animation');

fprintf('\n--- Starting Animation and Dynamic Jacobian Update ---\n');

for k = 1:T_steps
    q_current = q_trajectory(k, :);

    % 1. Plot the robot in the current pose
    Cylindrical_RPP_Robot.plot(q_current);

    % 2. Calculate the Jacobian for the current pose
    J_current = Cylindrical_RPP_Robot.jacob0(q_current);

    % 3. Display the updated Jacobian matrix in the command window
    clc; % Clear command window for a cleaner update
    
    fprintf('Step %d/%d\n', k, T_steps);
    fprintf('Current Joint State q (rad/unit): [%.2f, %.2f, %.2f]\n\n', q_current(1), q_current(2), q_current(3));
    
    fprintf('DYNAMIC JACOBIAN MATRIX J (Base Frame {0}):\n');
    % Print the matrix row by row for clear formatting
    fprintf('  Linear X: [%.4f, %.4f, %.4f]\n', J_current(1,:));
    fprintf('  Linear Y: [%.4f, %.4f, %.4f]\n', J_current(2,:));
    fprintf('  Linear Z: [%.4f, %.4f, %.4f]\n', J_current(3,:));
    fprintf('Angular X: [%.4f, %.4f, %.4f]\n', J_current(4,:));
    fprintf('Angular Y: [%.4f, %.4f, %.4f]\n', J_current(5,:));
    fprintf('Angular Z: [%.4f, %.4f, %.4f]\n', J_current(6,:));
    
    % Force the figure to update
    drawnow;

    % Removed the pause(0.05) to make the first 10 steps very fast
    % pause(0.05); 
end

fprintf('\n--- Animation Complete ---\n');

%% --- FORWARD DIFFERENTIAL KINEMATICS (Velocity Mapping) ---
% Re-include the velocity and force analysis sections for the final pose (optional)
q = q_end;
fprintf('\n--- Final Pose Analysis for q = [%.2f rad, %.2f, %.2f] ---\n', q(1), q(2), q(3));

J = Cylindrical_RPP_Robot.jacob0(q);
% Define arbitrary joint velocities
dq = [0.1; -0.2; 0.3]; 
V = J * dq;

disp('Resulting End-Effector Spatial Velocity (V):');
fprintf('  Linear Velocity [vx, vy, vz] (m/s): [%.4f, %.4f, %.4f]\n', V(1), V(2), V(3));
fprintf('  Angular Velocity [wx, wy, wz] (rad/s): [%.4f, %.4f, %.4f]\n', V(4), V(5), V(6));


%% --- STATIC FORCE ANALYSIS (Force Mapping) ---
F_ee = [0; 10; 5; 0; 0; 0]; % 10N force in Y, 5N force in Z
tau = J' * F_ee;

disp('Required Joint Torques/Forces (tau):');
fprintf('  Joint 1 (Revolute) Torque: %.4f Nm\n', tau(1));
fprintf('  Joint 2 (Prismatic) Force: %.4f N\n', tau(2));
fprintf('  Joint 3 (Prismatic) Force: %.4f N\n', tau(3));
