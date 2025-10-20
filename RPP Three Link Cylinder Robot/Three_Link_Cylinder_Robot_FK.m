clc         % Clear command window display
clear       % Clear all variables from workspace
close all   % Close all open figure windows

% Define link lengths for the planer robot
a1 = 5;     
a2 = 2;
a3 = 2;

% DH parameters table [theta, d, a, alpha]
L(1) = Link([0     a1     0      0]);            % R
L(2) = Link([0     0     0       -pi/2      1]);  
L(2).qlim = [2 6];  

L(3) = Link([0     0     0       0      1]);  
L(3).qlim = [2 7]; 

Rob = SerialLink(L, 'name', 'RPP');

q = [30*pi/180 3 4];

Rob.plot(q, 'workspace', [-10 10 -10 10 -1 10]);
T = Rob.fkine(q); disp(T);

% Sliders
Rob.teach;
