clc; clear; close all;

d1 = 0.25;
d2 = 0.15;
d6 = 0.1 ;


L(1) = Link([0     d1     0      -pi/2]);            % R
L(2) = Link([0      d2     0      pi/2]);            % R
L(3) = Link([-pi/2      0      0       0      1]);       % P
L(4) = Link([0      0     0     -pi/2]);            % R
L(5) = Link([0      0      0    pi/2]);            % R
L(6) = Link([0      d6     0     0]);            % R

L(3).qlim = [0 0.5];  

Rob = SerialLink(L, 'name', 'RRP');

q = [30*pi/180 -45*pi/180 0.2 60*pi/180 20*pi/180 90*pi/180];
Rob.plot(q, 'workspace', [-1 1 -1 1 -1 1]);
T = Rob.fkine(q); disp(T);

% Sliders
Rob.teach;