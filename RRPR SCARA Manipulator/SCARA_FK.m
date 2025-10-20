%% --- SETUP ---
clc;         % Clear command window
clear;       % Clear workspace variables
close all;   % Close all figures

%% --- ROBOT PARAMETERS ---
% This robot is a cylindrical RPP (Revolute-Prismatic-Prismatic) type.
% It has a fixed vertical offset for its base.
a1 = 100; 
a2 = 50;
a3 = 50;
a5 = 10;

L(1) = Link([0     a1     a2      0]);            % R
L(2) = Link([0     0     a3      pi]);   

L(3) = Link([0     0     0       0      1]);  
L(3).qlim = [40 50];
L(4) = Link([0     a5     0      0]);   


Rob = SerialLink(L, 'name', 'RPP');

q = [0 0 0 0];

Rob.plot(q, 'workspace', [-200 200 -200 200 -10 200]);
T = Rob.fkine(q); disp(T);

% Sliders
Rob.teach;

