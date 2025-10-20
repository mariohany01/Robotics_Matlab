% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% --- SYMBOLIC ANALYSIS ---
fprintf('--- Symbolic Forward Kinematics ---\n\n');

% Define symbolic variables for joints and parameters
syms theta1 theta2  % Joint variables
syms a1_s a2_s  % Link parameters

% Link([theta, d, a, alpha])
L_sym(1) = Link([theta1    0   0       0]);            % R
L_sym(2) = Link([theta2    0   a1_s       0]);            % R
L_sym(3) = Link([0         0   a2_s       0]);            % R


% Create the symbolic robot model
Rob_sym = SerialLink(L_sym, 'name', 'Symbolic R');

% Define the symbolic joint variable vector
q_sym = [0 theta1 theta2];

% Calculate the symbolic forward kinematics transformation matrix
T_sym = Rob_sym.fkine(q_sym);

% Extract the 4x4 matrix from the symbolic SE3 object before displaying
disp('Not Simple Symbolic Transformation Matrix (T):');
T_matrix_sym = T_sym.T

% Display the simplified symbolic transformation matrix
disp('Symbolic Transformation Matrix (T):');
disp(simplify(T_matrix_sym));



a1 = 10;
a2 = 10;

% Link([theta, d, a, alpha])
L(1) = Link([0    a2   a2       0]);            % R
L(2) = Link([0    0   a1       0]);            % R    
L(3) = Link([0    0   a2       0]);            % R        


Rob = SerialLink(L, 'name', 'RRRRR');
q=[0 0 0];
%q = [30*pi/180 90*pi/180 45*pi/180 60*pi/180 20*pi/180 90*pi/180]; %theta 2 keda da el zero bta3ha
Rob.plot(q, 'workspace', [-20 20 -20 20 -1 20]);
T = Rob.fkine(q); disp(T);
% Sliders
Rob.teach;