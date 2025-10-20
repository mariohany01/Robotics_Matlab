clc         % Clear command window display
clear       % Clear all variables from workspace
close all   % Close all open figure windows

% Define link lengths for the planer robot
a1 = 5;     
a2 = 2;
a3 = 2;

% DH parameters table [theta, d, a, alpha]
L(1) = Link([0     0     a1      0]);            % R
L(2) = Link([0     0     a2      0]);            % R
L(2) = Link([0     0     a2      0]);            % R

Rob = SerialLink(L, 'name', 'RR');

q = [30*pi/180 45*pi/180];

Rob.plot(q, 'workspace', [-5 5 -5 5 -5 5]);
T = Rob.fkine(q); disp(T);

% Sliders
Rob.teach;