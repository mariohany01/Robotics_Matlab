function [solutions, isReachable, message] = solveScaraIK(robot, target)
% solveScaraIK Calculates the inverse kinematics for a 4-DOF SCARA robot.
%
% INPUTS:
%   robot   - A struct containing the robot's link lengths (L1, L2, L3, L4, L5).
%   target  - A struct with the target end-effector pose, containing:
%             target.position: a 3x1 vector [x; y; z]
%             target.rotation: a 3x3 rotation matrix
%
% OUTPUTS:
%   solutions   - A struct array with the joint variables for each valid solution.
%                 Each solution has .joints [theta1; theta2; d3; theta4] in radians
%                 and .configuration ('Elbow Up' or 'Elbow Down').
%   isReachable - A boolean flag (true/false) indicating if the target can be reached.
%   message     - A string describing the outcome.

    % --- Initialize Outputs ---
    solutions = [];
    isReachable = false;
    message = 'Target is unreachable.';

    % --- Deconstruct Parameters for Readability ---
    L2 = robot.L2;
    L3 = robot.L3;
    px = target.position(1);
    py = target.position(2);
    pz = target.position(3);
    R = target.rotation;

    %% --- Step 1: Solve for Theta 2 (Elbow Joint) ---
    % Using the Law of Cosines on the XY plane projection of the arm.
    % The triangle is formed by L2, L3, and the distance from origin to wrist (r).
    r_squared = px^2 + py^2;
    cos_theta2_num = r_squared - L2^2 - L3^2;
    cos_theta2_den = 2 * L2 * L3;
    
    % Check for reachability: Is the argument for acos within [-1, 1]?
    if abs(cos_theta2_num / cos_theta2_den) > 1
        return; % Exit the function, target is not reachable.
    end
    
    cos_theta2 = cos_theta2_num / cos_theta2_den;
    
    % Two possible solutions for theta2 exist: elbow up and elbow down.
    theta2_up = -acos(cos_theta2);  % Elbow "Up" configuration
    theta2_down = acos(cos_theta2); % Elbow "Down" configuration

    %% --- Step 2: Solve for Theta 1 (Base Joint) ---
    % theta1 depends on theta2. We solve it for both elbow configurations.
    % theta1 = atan2(py, px) - atan2(L3*sin(theta2), L2 + L3*cos(theta2))
    
    % Solution 1: Elbow Up
    k1_up = L2 + L3 * cos(theta2_up);
    k2_up = L3 * sin(theta2_up);
    theta1_up = atan2(py, px) - atan2(k2_up, k1_up);
    
    % Solution 2: Elbow Down
    k1_down = L2 + L3 * cos(theta2_down);
    k2_down = L3 * sin(theta2_down);
    theta1_down = atan2(py, px) - atan2(k2_down, k1_down);

    %% --- Step 3: Solve for d3 (Prismatic Joint) ---
    % This is the vertical translation and is independent of elbow configuration.
    % Based on your formula: pz = (L1 - L4 - L5) - d3
    d3 = robot.L1 - robot.L4 - robot.L5 - pz;
    
    %% --- Step 4: Solve for Theta 4 (Wrist Roll) ---
    % The overall orientation in the XY plane (phi) is the sum of joint angles.
    % phi = theta1 + theta2 + theta4. We can find phi from the rotation matrix.
    phi = atan2(R(2,1), R(1,1));
    
    % Solve for theta4 for both configurations.
    theta4_up = phi - theta1_up - theta2_up;
    theta4_down = phi - theta1_down - theta2_down;
    
    %% --- Step 5: Package and Return the Solutions ---
    isReachable = true;
    message = 'Solution(s) found successfully.';
    
    % Package the "Elbow Up" solution
    solutions(1).joints = [theta1_up; theta2_up; d3; wrapToPi(theta4_up)];
    solutions(1).configuration = 'Elbow Up';

    % Package the "Elbow Down" solution
    solutions(2).joints = [theta1_down; theta2_down; d3; wrapToPi(theta4_down)];
    solutions(2).configuration = 'Elbow Down';
end