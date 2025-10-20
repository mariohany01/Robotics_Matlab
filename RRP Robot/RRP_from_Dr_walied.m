clc; clear; close all;

% Geometry (units arbitrary, e.g., mm)
L1 = 100;
L2 = 120;

% --- R R P with standard DH (SerialLink uses standard DH) ---
% Link i parameters: (theta, d, a, alpha, [sigma])
% sigma = 0 -> revolute (default), 1 -> prismatic

% Joint 1: revolute about z0, raise the shoulder by L1, twist +pi/2
L(1) = Link([0     L1     0      +pi/2]);            % R

% Joint 2: revolute about z1, forearm length L2 along x2,
% choose alpha2 = -pi/2 so that z2 points along the forearm direction.
L(2) = Link([0      0     L2     -pi/2]);            % R

% Joint 3: prismatic — moves along z2 (which we've aligned to the forearm)
L(3) = Link([0      0      0       0      1]);       % P
L(3).qlim = [0 80];   % extension range

Rob = SerialLink(L, 'name', 'RRP');

% Home pose
q = [0 0 0];
Rob.plot(q, 'workspace', [-200 200 -200 200 -50 250]);
T = Rob.fkine(q); disp(T);

% Sliders
Rob.teach;