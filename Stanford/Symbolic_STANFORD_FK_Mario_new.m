% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

% --- SYMBOLIC ANALYSIS ---
fprintf('--- Symbolic Forward Kinematics ---\n\n');

% Define symbolic variables for joints and parameters
syms theta1 theta2 d3  theta4 theta5 theta6 % Joint variables
syms d1_s d2_s d6_s % Link parameters

L_sym(1) = Link([theta1     d1_s     0      -pi/2]);            % R
L_sym(2) = Link([theta2      d2_s     0      pi/2]);            % R
L_sym(3) = Link([-pi/2      0      0       0      1]);       % P
L_sym(4) = Link([theta4      0     0     -pi/2]);            % R
L_sym(5) = Link([theta5      0      0    pi/2]);            % R
L_sym(6) = Link([theta6      d6_s     0     0]);            % R


% Create the symbolic robot model
Rob_sym = SerialLink(L_sym, 'name', 'Symbolic RRPRRR');

% Define the symbolic joint variable vector
q_sym = [theta1 theta2 d3 theta4 theta5 theta6];

% Calculate the symbolic forward kinematics transformation matrix
T_sym = Rob_sym.fkine(q_sym);

% Extract the 4x4 matrix from the symbolic SE3 object before displaying
T_matrix_sym = T_sym.T;

% Display the simplified symbolic transformation matrix
disp('Symbolic Transformation Matrix (T):');
disp(simplify(T_matrix_sym));

