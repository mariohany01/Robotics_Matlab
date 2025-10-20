clc; close; clear;

d1 = 0.25;
d2 = 0.15; 
d6 = 0.1;

L(1) = Link([0 d1 0 -pi/2]); % Joint 1: Revolute
L(2) = Link([0 d2 0 pi/2]);  % Joint 2: Revolute
L(3) = Link([-pi/2 0 0 0 1]); % Joint 3: Prismatic
L(4) = Link([0 0 0 -pi/2]);  % Joint 4: Revolute
L(5) = Link([0 0 0 pi/2]);   % Joint 5: Revolute
L(6) = Link([0 d6 0 0]);     % Joint 6: Revolute
L(3).qlim = [0 0.5];   % Extension range

Rob = SerialLink(L);
    
Rob.name = 'RRPRRR';

% Initial Joint Angles (Start Pose)
q_start = [0, 0, 0, 0, 0, 0];
Rob.plot(q_start, 'workspace', [-0.7 0.7 -0.7 0.7 -0.1 0.7]);

% Desired Joint Angles (End Pose)
theta1 = 30*pi/180;
theta2 = -90*pi/180 + 45*pi/180;
d3 = 0.2;
theta4 = 60*pi/180;
theta5 = 20*pi/180;
theta6 = 90*pi/180;

q_end = [theta1, theta2, d3, theta4, theta5, theta6];

% Calculate forward kinematics for the end pose
fk_matrix = Rob.fkine(q_end);

disp('Forward kinematics for the end configuration:');
disp(fk_matrix);

% Add animation
% Generate a joint-space trajectory with 50 steps
num_steps = 50;
q_trajectory = jtraj(q_start, q_end, num_steps);

% Animate the robot's movement using the plot function
disp('Animating the robot''s movement...');
Rob.plot(q_trajectory);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%show trajectory in deg
disp('Joint trajectory in degrees (except Joint 3 which is in meters):');
q_trajectory_deg = q_trajectory;
% Convert revolute joints (columns 1,2,4,5,6) from radians to degrees
q_trajectory_deg(:,[1,2,4,5,6]) = q_trajectory(:,[1,2,4,5,6]) * 180/pi;

% Display with column headers
fprintf('Step\tJ1(deg)\tJ2(deg)\tJ3(m)\tJ4(deg)\tJ5(deg)\tJ6(deg)\n');
fprintf('----\t-------\t-------\t-----\t-------\t-------\t-------\n');
for i = 1:size(q_trajectory_deg,1)
    fprintf('%2d\t%7.2f\t%7.2f\t%5.3f\t%7.2f\t%7.2f\t%7.2f\n', ...
            i, q_trajectory_deg(i,1), q_trajectory_deg(i,2), q_trajectory_deg(i,3), ...
            q_trajectory_deg(i,4), q_trajectory_deg(i,5), q_trajectory_deg(i,6));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%


