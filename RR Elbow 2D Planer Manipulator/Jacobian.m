% robot_jacobian_control.m
%
% Defines a 2-link planar RR robot (from the user's setup) and uses the
% analytic Jacobian to control its end-effector velocity, implementing a
% smooth deceleration profile based on the distance to the target.

clc         % Clear command window display
clear       % Clear all variables from workspace
close all   % Close all open figure windows

%% 1. Robot Setup (2-Link RR Planar Robot)

% Define link lengths
a1 = 5;
a2 = 2;

% DH parameters table [theta, d, a, alpha]
% Note: Planar R-R robot has zero 'd' and 'alpha'
L(1) = Link('d', 0, 'a', a1, 'alpha', 0, 'offset', 0);
L(2) = Link('d', 0, 'a', a2, 'alpha', 0, 'offset', 0);

% Create the SerialLink object
Rob = SerialLink(L, 'name', 'Planar RR');

% Configure display settings
Rob.plotopt = {'workspace', [-8 8 -8 8 -8 8], 'scale', 0.5};

%% 2. Control Parameters and Target Definition

% Initial joint configuration (start position)
q_start = [0, 0];
q_current = q_start;

% Desired Cartesian Target Position [X, Y]
P_target = [3.5, 5.0];

% Simulation Parameters
dt = 0.05;           % Time step for integration (s)
lambda_max = 0.5;    % Maximum velocity gain (controls initial speed)
tolerance = 0.01;    % Stop condition (distance threshold)
max_iterations = 500; % Safety limit

% Initial Kinematics
T_start = Rob.fkine(q_start);
P_start = T_start.t(1:2)'; % Extract [X, Y]
initial_error_mag = norm(P_target - P_start);

% Log data for plotting
history = zeros(max_iterations, 4); % [time, X, Y, speed_factor]
k = 1;

fprintf('Starting Position: [%.2f, %.2f]\n', P_start(1), P_start(2));
fprintf('Target Position:   [%.2f, %.2f]\n', P_target(1), P_target(2));
fprintf('Starting Control Loop...\n');

%% 3. Jacobian-Based Velocity Control Loop with Deceleration

while (k <= max_iterations)

    % A. Calculate Current End-Effector State
    T_current = Rob.fkine(q_current);
    P_current = T_current.t(1:2)'; % Current [X, Y] position
    
    % B. Calculate Error Vector and Magnitude
    error_vector = P_target - P_current; % Delta P
    error_mag = norm(error_vector);

    % Stop condition
    if error_mag < tolerance
        disp('Target reached. Stopping simulation.');
        break;
    end

    % C. Implement Decreasing Speed Logic (Deceleration)
    % The control gain (lambda) is proportional to the remaining distance.
    % We use a power of 0.5 for smoother, high-speed start and graceful stop.
    % This is the "step-by-step decreasing speed" requirement.
    speed_factor = (error_mag / initial_error_mag)^0.5;
    lambda = lambda_max * speed_factor;

    % D. Get the Robot's Jacobian
    J0 = Rob.jacob0(q_current);
    
    % The planar robot only moves in the X-Y plane (v_x, v_y),
    % so we only need the top 2 rows of the Jacobian.
    J_xy = J0(1:2, 1:2); % 2x2 Jacobian for [vx; vy] = J_xy * [qdot1; qdot2]

    % E. Calculate Desired Cartesian Velocity (proportional control)
    % P_dot_desired = lambda * error_vector (P_dot_desired is a 1x2 row vector)
    P_dot_desired = lambda * error_vector;
    
    % F. Inverse Kinematics (Velocity Control)
    % Calculate the joint velocities: q_dot = J_xy_pseudo_inverse * P_dot_desired_COLUMN
    % FIX: Transpose P_dot_desired (1x2) to (2x1) for matrix multiplication.
    J_inv = pinv(J_xy);
    q_dot = J_inv * P_dot_desired'; % <-- FIX APPLIED HERE

    % G. Update Joint Angles (Integration)
    % q_dot is now 2x1, so q_dot' is 1x2, which matches q_current (1x2)
    q_current = q_current + q_dot' * dt;
    
    % H. Plot the current state
    Rob.plot(q_current);
    drawnow;
    
    % I. Log the step
    history(k, :) = [k*dt, P_current, speed_factor];
    k = k + 1;
end

%% 4. Results and Visualization

% Trim history data
history = history(1:k-1, :);

figure;

% Plot 1: End-effector Path
subplot(1, 2, 1);
plot(history(:, 2), history(:, 3), 'b-', 'LineWidth', 2);
hold on;
plot(P_start(1), P_start(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(P_target(1), P_target(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
title('End-Effector Path (X-Y)');
xlabel('X Position');
ylabel('Y Position');
legend('Path', 'Start', 'Target', 'Location', 'best');
grid on; axis equal;

% Plot 2: Speed Factor over Time (Deceleration Profile)
subplot(1, 2, 2);
plot(history(:, 1), history(:, 4), 'm-', 'LineWidth', 2);
title('Deceleration Profile (Speed Factor)');
xlabel('Time (s)');
ylabel('Relative Speed (Lambda * Distance Ratio)');
grid on;

fprintf('\nSimulation Finished.\n');
fprintf('Final Position: [%.2f, %.2f]\n', P_current(1), P_current(2));

% Run Rob.teach; if you want to manually test the robot in the final position
Rob.teach(q_current);
