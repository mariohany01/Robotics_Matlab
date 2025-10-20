clc;close;clear;

d1=0.25;
d2=0.15; 
d6=0.1;

L(1) = Link([0 d1 0 -pi/2]); % Joint 1: Revolute
L(2) = Link([0 d2 0 pi/2]);  % Joint 2: Revolute (removed -pi/2 offset) but add it in the angle
L(3) = Link([-pi/2 0 0 0 1]); % Joint 3: Prismatic (simplified)
L(4) = Link([0 0 0 -pi/2]);  % Joint 4: Revolute
L(5) = Link([0 0 0 pi/2]);   % Joint 5: Revolute
L(6) = Link([0 d6 0 0]);     % Joint 6: Revolute
L(3).qlim = [0 0.5];   % extension range

Rob = SerialLink (L);
    
Rob.name ='RRPRRR';
    
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
fk_matrix = Rob.fkine(q_end); %Change q_end - q_start

disp('Forward kinematics for the end configuration:');
disp(fk_matrix);

%Add plot
Rob.plot ([q_end]) %Change q_end - q_start
Rob.teach




