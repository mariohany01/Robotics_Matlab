% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% --- SYMBOLIC ANALYSIS ---
fprintf('--- Symbolic Forward Kinematics ---\n\n');

% Define symbolic variables for joints and parameters
syms theta0 theta1 theta3 d3 real % Joint variables
syms a1 a2 a3 a5 real  % Link parameters

% Define the links using symbolic variables
% Link([theta, d, a, alpha])
L_sym(1) = Link([theta0   a1     a2      0]);       % Revolute
L_sym(2) = Link([theta1        0      a3      pi]);      % Prismatic (Revoluted in DH notation but prismatic in function)
L_sym(3) = Link([0        d3     0       0      1]); % Prismatic (1 indicates prismatic)
L_sym(4) = Link([theta3        a5     0       0]);       % Fixed offset

% Create the symbolic robot model
Rob_sym = SerialLink(L_sym, 'name', 'Symbolic RPP');

% Define the symbolic joint variable vector
q_sym = [theta0 theta1 d3 theta3];

% Calculate the symbolic forward kinematics transformation matrix
T_sym = Rob_sym.fkine(q_sym);

% Extract the 4x4 matrix from the symbolic SE3 object before displaying
T_matrix_sym = T_sym.T;

% Display the simplified symbolic transformation matrix
disp('Symbolic Transformation Matrix (T):');
disp(simplify(T_matrix_sym));


% --- NUMERICAL ANALYSIS (Original Code) ---
fprintf('\n\n--- Numeric Forward Kinematics and Simulation ---\n\n');

% --- ROBOT PARAMETERS ---
% This robot is a cylindrical RPP (Revolute-Prismatic-Prismatic) type.
% It has a fixed vertical offset for its base.
a1_num = 100; 
a2_num = 50;
a3_num = 50;
a5_num = 10;

% Define numeric links
L(1) = Link([0     a1_num     a2_num      0]);        % R
L(2) = Link([0     0          a3_num      pi]);   
L(3) = Link([0     0          0           0      1]);  % Prismatic joint
L(3).qlim = [10 150]; % Set joint limits for the prismatic joint
L(4) = Link([0     a5_num     0           0]);   

% Create the numeric robot model
Rob = SerialLink(L, 'name', 'RPP');

% Define a sample joint configuration
q = [pi/6 pi/3 20 pi/6]; % Example: 45 degrees, d3 = 70

% Plot the robot in the specified configuration
figure; % Create a new figure window
Rob.plot(q, 'workspace', [-200 200 -200 200 -10 200]);

% Calculate and display the numeric transformation matrix for the given q
T = Rob.fkine(q);
disp('Numeric Transformation Matrix for q = [pi/4 0 70 0]:');
disp(T);

% Open the interactive slider GUI
disp('Starting interactive teach pendant...');
Rob.teach;

